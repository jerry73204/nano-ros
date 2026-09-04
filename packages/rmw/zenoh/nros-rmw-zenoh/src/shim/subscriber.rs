//! ZenohSubscriber and ZenohZeroCopySubscriber implementations

use core::marker::PhantomData;

use atomic_waker::AtomicWaker;
use portable_atomic::{AtomicU32, AtomicUsize, Ordering};

use nros_rmw::{Subscription, TransportError};

use super::{
    KEYEXPR_BUFFER_SIZE, KEYEXPR_STRING_SIZE, MAX_LARGE_SUBSCRIBERS, MessageInfo,
    RMW_ATTACHMENT_SIZE, SUBSCRIBER_ATTACHMENT_BUF_SIZE, SUBSCRIBER_BUFFER_SIZE,
    SUBSCRIBER_LARGE_SIZE, SUBSCRIBER_RING_DEPTH, SUBSCRIBER_SIZE_THRESHOLD,
};
use crate::{
    keyexpr::TopicKeyExpr,
    zpico::{Context, ZPICO_MAX_SUBSCRIBERS, zpico_ring_desc_t},
};

#[cfg(feature = "safety-e2e")]
use super::SAFETY_CRC_SIZE;

#[cfg(feature = "std")]
use super::signal_executor_wake;

// ============================================================================
// SubscriberBuffer
// ============================================================================

/// Shared buffer for subscriber callbacks.
///
/// Phase 124.D.3.c — SPSC ring. The C shim is the sole producer
/// (writes payload + attachment + lengths into the slot at
/// `ring_tail % SUBSCRIBER_RING_DEPTH`, then Release-stores
/// `ring_tail + 1`). The Rust shim is the sole consumer (reads the
/// slot at `ring_head % SUBSCRIBER_RING_DEPTH`, then Release-stores
/// `ring_head + 1`). Ring empty when `head == tail`, full when
/// `tail - head == SUBSCRIBER_RING_DEPTH`.
///
/// Replaces the previous single-slot + `locked` flag design — a
/// burst of up to `SUBSCRIBER_RING_DEPTH` messages arriving between
/// two `take` calls is now buffered instead of dropped, and
/// `take_sequence` can drain the whole ring in one call. No
/// lock is needed: the SPSC discipline + the Release/Acquire fence
/// on `ring_tail` / `ring_head` covers the cross-FFI handoff.
pub(super) struct SubscriberBuffer {
    /// Ring of attachment slots, parallel to the (externally-stored) payload
    /// ring. Phase 231 (RFC-0038): the payload storage lives in a size-class
    /// array (`SMALL_PAYLOADS` / `LARGE_PAYLOADS`); this metadata struct is
    /// size-independent and reaches the payload through `ring_desc.payload_base`.
    pub(super) ring_att: [[u8; SUBSCRIBER_ATTACHMENT_BUF_SIZE]; SUBSCRIBER_RING_DEPTH],
    /// Per-slot payload byte length. Written by the C shim before
    /// its Release-store to `ring_tail`; read by the Rust shim
    /// after its Acquire-load.
    pub(super) ring_len: [usize; SUBSCRIBER_RING_DEPTH],
    /// Per-slot attachment byte length.
    pub(super) ring_att_len: [usize; SUBSCRIBER_RING_DEPTH],
    /// Consumer counter — advanced only by the Rust shim.
    pub(super) ring_head: AtomicUsize,
    /// Producer counter — advanced only by the C shim.
    pub(super) ring_tail: AtomicUsize,
    /// Descriptor handed to the C shim at subscribe time. The raw
    /// pointers reference this same `SubscriberBuffer` (a
    /// `static mut` element — its address is stable for the
    /// program's lifetime). Filled in `ZenohSubscriber::new`.
    pub(super) ring_desc: zpico_ring_desc_t,
    /// Async waker — registered by `Future::poll()`, woken from the
    /// notify callback when data arrives. Enables event-driven
    /// async without busy-polling.
    pub(super) waker: AtomicWaker,
}

impl SubscriberBuffer {
    pub(super) const fn new() -> Self {
        Self {
            ring_att: [[0u8; SUBSCRIBER_ATTACHMENT_BUF_SIZE]; SUBSCRIBER_RING_DEPTH],
            ring_len: [0usize; SUBSCRIBER_RING_DEPTH],
            ring_att_len: [0usize; SUBSCRIBER_RING_DEPTH],
            ring_head: AtomicUsize::new(0),
            ring_tail: AtomicUsize::new(0),
            ring_desc: zpico_ring_desc_t {
                payload_base: core::ptr::null_mut(),
                payload_stride: 0,
                att_base: core::ptr::null_mut(),
                att_stride: 0,
                slot_count: 0,
                payload_len: core::ptr::null_mut(),
                att_len: core::ptr::null_mut(),
                head: core::ptr::null_mut(),
                tail: core::ptr::null_mut(),
            },
            waker: AtomicWaker::new(),
        }
    }

    /// True when the ring holds at least one un-consumed message.
    pub(super) fn has_data(&self) -> bool {
        self.ring_head.load(Ordering::Acquire) != self.ring_tail.load(Ordering::Acquire)
    }

    /// Index of the head slot if the ring is non-empty. Does NOT
    /// advance `ring_head` — the caller reads the slot, then calls
    /// [`consume_head`](Self::consume_head). The Acquire-load of
    /// `ring_tail` synchronises-with the C producer's Release-store,
    /// so the per-slot payload / attachment / length writes that
    /// happened-before that store are visible here.
    pub(super) fn peek_head_slot(&self) -> Option<usize> {
        let head = self.ring_head.load(Ordering::Acquire);
        let tail = self.ring_tail.load(Ordering::Acquire);
        if head == tail {
            None
        } else {
            Some(head % SUBSCRIBER_RING_DEPTH)
        }
    }

    /// Advance `ring_head` past the slot returned by the most recent
    /// [`peek_head_slot`](Self::peek_head_slot). Release-store so the
    /// C producer's Acquire-load of `ring_head` sees the slot freed.
    pub(super) fn consume_head(&self) {
        let head = self.ring_head.load(Ordering::Acquire);
        self.ring_head
            .store(head.wrapping_add(1), Ordering::Release);
    }

    /// Populate `ring_desc` so the C shim can produce into this buffer. Must be
    /// called once, after the buffer's static address is known and a payload
    /// block has been allocated (i.e. from `ZenohSubscriber::new`). Phase 231
    /// (RFC-0038): the payload storage is external (a size-class array), so its
    /// base + stride are passed in; the att/len/head/tail still reference this
    /// metadata struct.
    pub(super) fn init_ring_desc(&mut self, payload_base: *mut u8, payload_stride: usize) {
        self.ring_desc = zpico_ring_desc_t {
            payload_base,
            payload_stride,
            att_base: self.ring_att.as_mut_ptr() as *mut u8,
            att_stride: SUBSCRIBER_ATTACHMENT_BUF_SIZE,
            slot_count: SUBSCRIBER_RING_DEPTH,
            payload_len: self.ring_len.as_mut_ptr(),
            att_len: self.ring_att_len.as_mut_ptr(),
            head: self.ring_head.as_ptr(),
            tail: self.ring_tail.as_ptr(),
        };
    }

    /// Borrow the payload bytes of ring slot `slot` (`len` bytes). Phase 231
    /// (RFC-0038): reads through `ring_desc.payload_base` (the external
    /// size-class storage) + `payload_stride`, so the consumer is agnostic to
    /// which size class backs this subscriber.
    pub(super) fn payload_slot(&self, slot: usize, len: usize) -> &'static [u8] {
        // Safety: `ring_desc` was populated in `init_ring_desc` with a stable
        // 'static payload base; `slot < slot_count` and `len <= payload_stride`
        // hold by the SPSC producer contract.
        unsafe {
            core::slice::from_raw_parts(
                self.ring_desc
                    .payload_base
                    .add(slot * self.ring_desc.payload_stride),
                len,
            )
        }
    }

    /// Mutable counterpart to [`payload_slot`](Self::payload_slot) — for the
    /// Rust-side producer paths that write into a ring slot directly (non-C-shim
    /// push / loopback). Writes through the external size-class storage.
    /// (Only some platform/feature configs exercise the Rust-side producer.)
    #[allow(dead_code)]
    pub(super) fn payload_slot_mut(&mut self, slot: usize, len: usize) -> &'static mut [u8] {
        unsafe {
            core::slice::from_raw_parts_mut(
                self.ring_desc
                    .payload_base
                    .add(slot * self.ring_desc.payload_stride),
                len,
            )
        }
    }

    /// Payload slot capacity (the size-class stride) for this subscriber.
    pub(super) fn payload_stride(&self) -> usize {
        self.ring_desc.payload_stride
    }
}

/// Phase 231 (RFC-0038) — size-class payload storage, external to the
/// (size-independent) `SubscriberBuffer` metadata. A subscription allocates one
/// payload block from the class its `rx_buffer_hint` selects; the block's
/// address + stride are baked into the subscriber's `ring_desc`. RAM is
/// `MAX_SUBS×DEPTH×SMALL + MAX_LARGE×DEPTH×LARGE` instead of
/// `MAX_SUBS×DEPTH×LARGE` — only the few `large` blocks are big.
type SmallPayloadBlock = [[u8; SUBSCRIBER_BUFFER_SIZE]; SUBSCRIBER_RING_DEPTH];
type LargePayloadBlock = [[u8; SUBSCRIBER_LARGE_SIZE]; SUBSCRIBER_RING_DEPTH];

// issue 0739 — declare the arithmetic so the pool inventory can price it.
// These two are the pools issue 0271 measured: `LARGE_PAYLOADS` at its
// defaults is 131,072 bytes, which is what a correctly-rightsized 256 KB image
// silently inherited because nobody knew the three knobs existed.
// nros-pool: SMALL_PAYLOADS = ZPICO_MAX_SUBSCRIBERS * ZPICO_SUBSCRIBER_RING_DEPTH * NROS_SUBSCRIBER_BUFFER_SIZE
// nros-pool: LARGE_PAYLOADS = ZPICO_MAX_LARGE_SUBSCRIBERS * ZPICO_SUBSCRIBER_RING_DEPTH * ZPICO_SUBSCRIBER_LARGE_SIZE
static mut SMALL_PAYLOADS: [SmallPayloadBlock; ZPICO_MAX_SUBSCRIBERS] =
    [[[0u8; SUBSCRIBER_BUFFER_SIZE]; SUBSCRIBER_RING_DEPTH]; ZPICO_MAX_SUBSCRIBERS];
static mut LARGE_PAYLOADS: [LargePayloadBlock; MAX_LARGE_SUBSCRIBERS] =
    [[[0u8; SUBSCRIBER_LARGE_SIZE]; SUBSCRIBER_RING_DEPTH]; MAX_LARGE_SUBSCRIBERS];

/// Issue 0841 — the largest hint the small class can actually serve.
///
/// `SUBSCRIBER_SIZE_THRESHOLD` and `SUBSCRIBER_BUFFER_SIZE` are independent
/// knobs and disagree at their shipped defaults (2048 vs 1024), so routing on
/// the threshold alone sent every hint in 1025..=2048 to a block half its size.
/// The effective ceiling is whichever is SMALLER: the block must hold what was
/// routed into it, and a threshold set below the block size is a legitimate way
/// to push borderline topics large early.
///
/// A `const fn` rather than `Ord::min`, which is not const-callable on stable.
const SMALL_CLASS_CEILING: usize = if SUBSCRIBER_SIZE_THRESHOLD < SUBSCRIBER_BUFFER_SIZE {
    SUBSCRIBER_SIZE_THRESHOLD
} else {
    SUBSCRIBER_BUFFER_SIZE
};

/// Phase 403 W4 — the largest sample any subscription in this image can be
/// handed, whatever class it routes to.
///
/// This is the top end of issue 0841, which the fix for that issue left open.
/// 0841 closed the SMALL/LARGE boundary: a hint the small block could not hold
/// was small-classed and every sample dropped at the transport. The identical
/// defect one class up was never closed — a hint above `SUBSCRIBER_LARGE_SIZE`
/// still routed to the large class, got a block too small for it, and dropped
/// every sample the same way, with no build assertion and no runtime error at
/// create time.
///
/// The large class only exists when the image declares slots for it, so with
/// `MAX_LARGE_SUBSCRIBERS == 0` the ceiling is the small block. That is the
/// case W4 makes expressible: an image whose types all fit the small class
/// declares no large slots and stops paying for a class it never routes into.
const LARGEST_PAYLOAD_CLASS: usize =
    if MAX_LARGE_SUBSCRIBERS > 0 && SUBSCRIBER_LARGE_SIZE > SUBSCRIBER_BUFFER_SIZE {
        SUBSCRIBER_LARGE_SIZE
    } else {
        SUBSCRIBER_BUFFER_SIZE
    };

/// Phase 403 W4 — the answer to `rmw_vtable.h`'s `required_rx_bytes` for this
/// backend: the MINIMUM number of take-buffer bytes a subscription created at
/// `rx_buffer_hint` can need, or `None` when zenoh-pico cannot size it.
///
/// Minimum, not the class stride. Reporting the stride would be arithmetically
/// safe and would collapse exactly the distinction the slot exists to recover —
/// a 68-byte type and a 1000-byte type would both come back 1024, which is the
/// two-arbitrary-constants answer the runtime already had. zenoh-pico can be
/// exact here because it adds NOTHING to the payload the caller must hold:
/// `take_serialized` copies `ring_len[slot]` bytes out of the ring slot and the
/// attachment travels in its own parallel ring (`ring_att`), so the take buffer
/// carries serialized message bytes and no framing.
///
/// * `hint == 0` — the CALLER stated nothing (never "this type is unbounded";
///   every message type carries a derived bound or the build fails). zenoh-pico
///   knows a type only by its NAME across the C ABI and holds no schema, so it
///   cannot answer about the type. It can still answer exactly about the
///   subscription it would create: that hint routes to the small class, whose
///   slot stride is the most `take` can ever hand back. That is a real ceiling,
///   not a guess, and it beats the runtime's own global default because it
///   tracks this backend's class rather than a separate constant.
/// * `0 < hint <= LARGEST_PAYLOAD_CLASS` — `hint`. The class the hint routes to
///   is guaranteed to hold it (that is 0841's property, asserted by
///   `a_routed_block_always_fits_its_hint`), and nothing is added on top.
/// * `hint > LARGEST_PAYLOAD_CLASS` — `None`, which the vtable trampoline turns
///   into `NROS_RMW_RET_UNSUPPORTED`. No class in this image can hold a sample
///   that big, so there is no take-buffer size that makes the subscription work
///   and saying a number would be a claim this backend cannot honour.
///   `alloc_payload_block` refuses the same hint, so the two agree: asking is
///   the create-time failure, in advance.
pub fn required_rx_bytes(rx_buffer_hint: usize) -> Option<usize> {
    if rx_buffer_hint == 0 {
        return Some(SUBSCRIBER_BUFFER_SIZE);
    }
    if rx_buffer_hint > LARGEST_PAYLOAD_CLASS {
        record_alloc(rx_buffer_hint, 1);
        return None;
    }
    Some(rx_buffer_hint)
}

/// phase-412 -- a RAM record of what the payload-class allocator decided.
///
/// # Why this is its own record and not a field of `boot_report`
///
/// `nros-node` owns the boot report, and `nros-node` DEPENDS on this crate --
/// the edge runs the other way, so calling into it from here would be a cycle.
/// The alternative, plumbing the refusal up through `TransportError`, cannot
/// carry the numbers either: the variant that surfaces is
/// `SubscriberCreationFailed`, which is one variant for three distinct exits.
///
/// So the crate that HAS the facts keeps them, and the reader takes two
/// symbols instead of one. That is the honest shape here.
///
/// # What it answers
///
/// `alloc_payload_block` refuses on three conditions and the caller cannot tell
/// them apart. On the mr-canhubk344 island the refusal cost a subscription and
/// stopped boot, and reasoning from the source narrowed it to "one of three"
/// twice without settling it -- the ceilings are compile-time and readable from
/// the build, but WHICH exit fired and WITH WHAT HINT are runtime facts that
/// nothing recorded.
///
/// Written unconditionally: it is 48 bytes of `.bss` and a few relaxed stores
/// on a path that runs once per subscription at registration. Unlike the boot
/// report this is not opt-in, because a refusal here is always a build
/// misconfiguration worth explaining and there is no spin-path cost to weigh
/// against it.
#[repr(C)]
pub struct SubscriberAllocReport {
    magic: AtomicU32,
    version: AtomicU32,
    struct_size: AtomicU32,
    /// 0 = no refusal, 1 = hint above every class, 2 = large class full,
    /// 3 = small class full. The three exits of `alloc_payload_block`.
    refusal: AtomicU32,
    /// The hint that was refused, or the last one accepted.
    rx_hint: AtomicU32,
    /// Compile-time ceilings, so a dump says what the image was built with
    /// rather than what the reader assumes it was built with.
    largest_payload_class: AtomicU32,
    small_class_ceiling: AtomicU32,
    max_large_subscribers: AtomicU32,
    subscriber_large_size: AtomicU32,
    subscriber_buffer_size: AtomicU32,
    /// How many blocks each class has handed out. The small counter reaching
    /// `ZPICO_MAX_SUBSCRIBERS` is exit 3, and its value ABOVE the number of
    /// subscriptions the image declares is the tell that something other than
    /// the application is taking blocks.
    small_taken: AtomicU32,
    large_taken: AtomicU32,
    /// Bytes in the data keyexpr of the last subscription attempted.
    ///
    /// `to_key_wildcard` builds it into a `heapless::String<KEYEXPR_STRING_SIZE>`
    /// and the writer DISCARDS the overflow, so a keyexpr too long for the buffer
    /// is silently TRUNCATED rather than refused. A length exactly equal to
    /// [`Self::keyexpr_cap`] is that truncation; the explicit
    /// `bytes.len() >= keyexpr_buf.len()` guard below cannot see it, because a
    /// truncated string is by construction short enough to pass.
    keyexpr_len: AtomicU32,
    /// `KEYEXPR_STRING_SIZE`, so a reader can spot the equality above without
    /// knowing how the image was configured.
    keyexpr_cap: AtomicU32,
    /// Which `ZpicoError` the declare returned, or 0 if it succeeded.
    ///
    /// The variant, not the mapped error: crossing the C ABI turns
    /// `Generic` and `Session` BOTH into `ConnectionFailed` (issue 0870) and
    /// then into an indistinguishable `Backend("rmw_ret error")` on the way
    /// back. Issue 0465 records what that collapse cost the last time -- an
    /// exhausted pool "spent two months looking like a router problem". This
    /// keeps the identity on the near side of the ABI.
    zpico_err: AtomicU32,
    /// How many METADATA slots have been handed out (`NEXT_BUFFER_INDEX`).
    ///
    /// A THIRD counter, and the one that guards first: its bound check runs
    /// BEFORE `alloc_payload_block`, so an image that exhausts it never reaches
    /// the payload classes and the refusal field stays 0. That is exactly how
    /// this record read on the island before the counter was added -- ten
    /// blocks taken, no refusal, and an error anyway.
    buffer_taken: AtomicU32,
}

/// `"SUBA"` -- subscriber alloc. Written LAST by [`record_alloc_ceilings`].
pub const SUBSCRIBER_ALLOC_MAGIC: u32 = 0x53554241;
/// Layout version for [`SubscriberAllocReport`].
pub const SUBSCRIBER_ALLOC_VERSION: u32 = 3;

/// The record, findable by symbol from a debugger.
#[unsafe(no_mangle)]
#[used]
pub static NROS_SUBSCRIBER_ALLOC_REPORT: SubscriberAllocReport = SubscriberAllocReport {
    magic: AtomicU32::new(0),
    version: AtomicU32::new(0),
    struct_size: AtomicU32::new(0),
    refusal: AtomicU32::new(0),
    rx_hint: AtomicU32::new(0),
    largest_payload_class: AtomicU32::new(0),
    small_class_ceiling: AtomicU32::new(0),
    max_large_subscribers: AtomicU32::new(0),
    subscriber_large_size: AtomicU32::new(0),
    subscriber_buffer_size: AtomicU32::new(0),
    small_taken: AtomicU32::new(0),
    large_taken: AtomicU32::new(0),
    keyexpr_len: AtomicU32::new(0),
    keyexpr_cap: AtomicU32::new(0),
    zpico_err: AtomicU32::new(0),
    buffer_taken: AtomicU32::new(0),
};

/// Record the metadata-slot index that was refused.
///
/// The INDEX, not the counter: the refusing path rolls the counter back, so
/// reading it afterwards reports the pool as it looks once everyone has
/// retreated rather than at the moment of refusal. `fetch_max` so a rollback
/// can never make the record claim less than what actually refused.
pub(super) fn record_buffer_index(idx: usize) {
    NROS_SUBSCRIBER_ALLOC_REPORT
        .buffer_taken
        .fetch_max(u32::try_from(idx).unwrap_or(u32::MAX), Ordering::Relaxed);
}

/// Record the keyexpr the declare is about to use.
///
/// Separate from [`record_alloc`] because it is known LATER: the payload block
/// is reserved before the key is built.
pub(super) fn record_keyexpr(len: usize, cap: usize) {
    let r = &NROS_SUBSCRIBER_ALLOC_REPORT;
    let sat = |v: usize| u32::try_from(v).unwrap_or(u32::MAX);
    r.keyexpr_len.store(sat(len), Ordering::Relaxed);
    r.keyexpr_cap.store(sat(cap), Ordering::Relaxed);
}

/// Stable code per `ZpicoError` variant, for the report.
///
/// EXHAUSTIVE and no wildcard arm, the rule issue 0586 states for the ret
/// mappers: rustc must refuse a new variant until someone numbers it.
pub(super) fn zpico_err_class(e: &crate::zpico::ZpicoError) -> u32 {
    use crate::zpico::ZpicoError as Z;
    match e {
        Z::Generic => 1,
        Z::Config => 2,
        Z::Session => 3,
        Z::Task => 4,
        Z::KeyExpr => 5,
        Z::Full => 6,
        Z::Invalid => 7,
        Z::Publish => 8,
        Z::NotOpen => 9,
        Z::Timeout => 10,
    }
}

/// Record which `ZpicoError` the subscriber declare returned. FIRST wins.
pub(super) fn record_zpico_err(code: u32) {
    let r = &NROS_SUBSCRIBER_ALLOC_REPORT;
    let _ = r
        .zpico_err
        .compare_exchange(0, code, Ordering::Relaxed, Ordering::Relaxed);
    r.magic.store(SUBSCRIBER_ALLOC_MAGIC, Ordering::Relaxed);
}

/// Stamp the compile-time ceilings and the running counters.
///
/// Called on EVERY `alloc_payload_block`, success or refusal, so a healthy
/// image still reports what its classes are -- the ceilings are half the
/// answer even when nothing refused, and an image that never fails is exactly
/// the one nobody would think to dump.
fn record_alloc(hint: usize, refusal: u32) {
    let r = &NROS_SUBSCRIBER_ALLOC_REPORT;
    let sat = |v: usize| u32::try_from(v).unwrap_or(u32::MAX);
    r.version.store(SUBSCRIBER_ALLOC_VERSION, Ordering::Relaxed);
    r.struct_size.store(
        core::mem::size_of::<SubscriberAllocReport>() as u32,
        Ordering::Relaxed,
    );
    r.rx_hint.store(sat(hint), Ordering::Relaxed);
    r.largest_payload_class
        .store(sat(LARGEST_PAYLOAD_CLASS), Ordering::Relaxed);
    r.small_class_ceiling
        .store(sat(SMALL_CLASS_CEILING), Ordering::Relaxed);
    r.max_large_subscribers
        .store(sat(MAX_LARGE_SUBSCRIBERS), Ordering::Relaxed);
    r.subscriber_large_size
        .store(sat(SUBSCRIBER_LARGE_SIZE), Ordering::Relaxed);
    r.subscriber_buffer_size
        .store(sat(SUBSCRIBER_BUFFER_SIZE), Ordering::Relaxed);
    r.small_taken.store(
        sat(NEXT_SMALL_PAYLOAD.load(Ordering::SeqCst)),
        Ordering::Relaxed,
    );
    r.large_taken.store(
        sat(NEXT_LARGE_PAYLOAD.load(Ordering::SeqCst)),
        Ordering::Relaxed,
    );
    r.keyexpr_cap
        .store(sat(KEYEXPR_STRING_SIZE), Ordering::Relaxed);
    r.buffer_taken.fetch_max(
        sat(NEXT_BUFFER_INDEX.load(Ordering::SeqCst)),
        Ordering::Relaxed,
    );
    // FIRST refusal wins: the one that stopped the boot is the one that
    // explains it, and a later refusal is a consequence.
    if refusal != 0 {
        let _ = r
            .refusal
            .compare_exchange(0, refusal, Ordering::Relaxed, Ordering::Relaxed);
    }
    r.magic.store(SUBSCRIBER_ALLOC_MAGIC, Ordering::Relaxed);
}

pub(super) static NEXT_SMALL_PAYLOAD: AtomicUsize = AtomicUsize::new(0);
pub(super) static NEXT_LARGE_PAYLOAD: AtomicUsize = AtomicUsize::new(0);

// Phase 231 Wave 3 (RFC-0038) — RAM-scaling proof (compile-time). The whole
// point of size classes: the big `large` slots are bounded by
// `MAX_LARGE_SUBSCRIBERS`, NOT by `ZPICO_MAX_SUBSCRIBERS`, so raising
// `SUBSCRIBER_LARGE_SIZE` (e.g. to 64 KB for images) does not multiply across
// every subscriber. Total receive payload RAM is
// `MAX_SUBS×DEPTH×SMALL + MAX_LARGE×DEPTH×LARGE`, and the `small` array uses the
// small slot size — never the large one.
const _: () = {
    assert!(
        core::mem::size_of::<[LargePayloadBlock; MAX_LARGE_SUBSCRIBERS]>()
            == MAX_LARGE_SUBSCRIBERS * SUBSCRIBER_RING_DEPTH * SUBSCRIBER_LARGE_SIZE,
        "large-class storage must be bounded by MAX_LARGE_SUBSCRIBERS, not MAX_SUBSCRIBERS"
    );
    assert!(
        core::mem::size_of::<[SmallPayloadBlock; ZPICO_MAX_SUBSCRIBERS]>()
            == ZPICO_MAX_SUBSCRIBERS * SUBSCRIBER_RING_DEPTH * SUBSCRIBER_BUFFER_SIZE,
        "small-class storage must use the small slot size"
    );
};

/// Reserve a payload block for a subscription whose receive-buffer hint is
/// `rx_buffer_hint` bytes. Returns `(payload_base, payload_stride)` for
/// `init_ring_desc`, or `None` if that size class is exhausted.
pub(super) fn alloc_payload_block(rx_buffer_hint: usize) -> Option<(*mut u8, usize)> {
    // Issue 0841 — route on whether the hint FITS THE SMALL BLOCK, not on the
    // threshold alone. The two knobs are independent and disagree at their
    // shipped defaults (threshold 2048, small block 1024), so `hint > threshold`
    // sent every hint in 1025..=2048 to a block that could not hold it. That
    // window is reachable by following `create_subscription`'s own build error,
    // which says to raise `NROS_SUBSCRIPTION_BUFFER_SIZE` — at 2048 the hint is
    // not > 2048, so it lands small and the sample is dropped at the transport:
    // the exact failure the assertion exists to prevent.
    //
    // `min`, not a replacement: setting the threshold BELOW the block size is a
    // legitimate way to push borderline topics into the large class early, and
    // that keeps working. What must not happen is small-routing a hint the small
    // block cannot hold.
    //
    // Phase 403 W4 — the same rule at the TOP end, which 0841 left open. A hint
    // above the largest class was routed large anyway, into a block that could
    // not hold it, and every sample was dropped at the transport exactly as in
    // the case 0841 fixed. Refuse instead: `ZenohSubscriber::new` surfaces
    // `SubscriberCreationFailed`, so the image fails where the person who set
    // the knobs is standing rather than receiving nothing and reporting nothing.
    // This is also what makes `MAX_LARGE_SUBSCRIBERS == 0` safe to declare: the
    // large class does not exist, so a hint past the small block has nowhere
    // legal to go and is refused rather than silently under-served.
    if rx_buffer_hint > LARGEST_PAYLOAD_CLASS {
        record_alloc(rx_buffer_hint, 1);
        return None;
    }
    if rx_buffer_hint > SMALL_CLASS_CEILING {
        let idx = NEXT_LARGE_PAYLOAD.fetch_add(1, Ordering::SeqCst);
        if idx >= MAX_LARGE_SUBSCRIBERS {
            NEXT_LARGE_PAYLOAD.fetch_sub(1, Ordering::SeqCst);
            record_alloc(rx_buffer_hint, 2);
            return None;
        }
        // Safety: idx is in-bounds; LARGE_PAYLOADS is a static with a stable address.
        let base = unsafe { (&raw mut LARGE_PAYLOADS[idx]) as *mut u8 };
        record_alloc(rx_buffer_hint, 0);
        Some((base, SUBSCRIBER_LARGE_SIZE))
    } else {
        let idx = NEXT_SMALL_PAYLOAD.fetch_add(1, Ordering::SeqCst);
        if idx >= ZPICO_MAX_SUBSCRIBERS {
            NEXT_SMALL_PAYLOAD.fetch_sub(1, Ordering::SeqCst);
            record_alloc(rx_buffer_hint, 3);
            return None;
        }
        // Safety: idx is in-bounds; SMALL_PAYLOADS is a static with a stable address.
        let base = unsafe { (&raw mut SMALL_PAYLOADS[idx]) as *mut u8 };
        record_alloc(rx_buffer_hint, 0);
        Some((base, SUBSCRIBER_BUFFER_SIZE))
    }
}

/// Static buffers for subscribers.
///
/// Count matches `ZPICO_MAX_SUBSCRIBERS` from zpico-sys (the C shim
/// allocates the same number of subscriber entries). We use static buffers
/// because the shim callback mechanism requires a static context pointer.
static mut SUBSCRIBER_BUFFERS: [SubscriberBuffer; ZPICO_MAX_SUBSCRIBERS] =
    [const { SubscriberBuffer::new() }; ZPICO_MAX_SUBSCRIBERS];

/// Next available buffer index
pub(super) static NEXT_BUFFER_INDEX: AtomicUsize = AtomicUsize::new(0);

// ============================================================================
// SubscriberBufferRef — safe accessor wrapper
// ============================================================================

/// Safe accessor for a statically-allocated subscriber buffer.
///
/// Encapsulates the `unsafe` access to `SUBSCRIBER_BUFFERS` by validating
/// the index once at construction time. Subsequent accesses via [`get()`]
/// are safe because the index is guaranteed in-bounds.
///
/// # Safety invariant
///
/// `SUBSCRIBER_BUFFERS` is a module-level `static mut` with a fixed address
/// and element count equal to `ZPICO_MAX_SUBSCRIBERS`. The index is validated
/// at construction and never changes, so every `get()` / `get_mut()` call
/// dereferences a valid, in-bounds element.
pub(super) struct SubscriberBufferRef {
    index: usize,
}

impl SubscriberBufferRef {
    /// Create a new buffer reference with bounds validation.
    ///
    /// # Panics
    ///
    /// Panics if `index >= ZPICO_MAX_SUBSCRIBERS`.
    pub(super) fn new(index: usize) -> Self {
        assert!(
            index < ZPICO_MAX_SUBSCRIBERS,
            "subscriber buffer index out of bounds: {index} >= {ZPICO_MAX_SUBSCRIBERS}"
        );
        Self { index }
    }

    /// Get an immutable reference to the subscriber buffer.
    ///
    /// Returns a `'static` reference — `SUBSCRIBER_BUFFERS` is a
    /// module-level `static mut` whose elements live for the
    /// program's lifetime, so the borrow is genuinely `'static` and
    /// callers don't have to keep the `SubscriberBufferRef` alive.
    ///
    /// Safety is guaranteed by the bounds check at construction time.
    /// All shared fields use atomic types, preventing data races.
    pub(super) fn get(&self) -> &'static SubscriberBuffer {
        // Safety: index was validated at construction time.
        // SUBSCRIBER_BUFFERS is a module-level static with fixed address.
        unsafe { &SUBSCRIBER_BUFFERS[self.index] }
    }

    /// Get a mutable reference to the subscriber buffer.
    ///
    /// Only called from callbacks, which are invoked synchronously
    /// (single-threaded) by zenoh-pico — no concurrent mutable access.
    pub(super) fn get_mut(&mut self) -> &mut SubscriberBuffer {
        // Safety: index was validated at construction time.
        // Mutable access is only used by callbacks invoked synchronously
        // by zenoh-pico, so there are no concurrent mutable accesses.
        unsafe { &mut SUBSCRIBER_BUFFERS[self.index] }
    }
}

/// Notify callback invoked by the C shim once per message arrival.
///
/// Phase 124.D.3.c — in ring mode the C shim has already written the
/// payload, attachment, and per-slot lengths into the next free ring
/// slot and Release-stored `ring_tail` before calling this. So the
/// callback only has to fire the async waker / executor wake — there
/// is nothing left for it to copy. The `len` / `attachment` args are
/// unused (the consumer reads them from the ring slot). On a
/// full-ring or oversized-payload drop the C shim still calls us so
/// the waker observes the arrival attempt.
extern "C" fn subscriber_notify_callback(
    len: usize,
    _attachment: *const u8,
    _attachment_len: usize,
    ctx: *mut core::ffi::c_void,
) {
    let buffer_index = ctx as usize;
    if buffer_index >= ZPICO_MAX_SUBSCRIBERS {
        return;
    }

    // Phase 160.L.2 — C shim signals an oversized-payload drop by
    // calling notify with `len > SUBSCRIBER_BUFFER_SIZE` and a NULL
    // payload (see `zpico.c:595-599`). Bump a per-subscriber counter
    // so user code can observe drops that would otherwise be silent
    // — the test harness asserts on this in
    // `test_zenoh_overflow_detection`, and it doubles as a
    // user-visible signal that the subscriber's QoS / buffer sizing
    // is wrong for the producer's payload size.
    let buf_ref = SubscriberBufferRef {
        index: buffer_index,
    };
    let buffer = buf_ref.get();

    // Phase 231 (RFC-0038) — the oversize threshold is now the subscriber's own
    // size-class slot stride, not the global small size, so a large-class
    // subscriber isn't falsely counted as overflowing.
    if len > buffer.payload_stride() {
        OVERFLOW_DROPS.fetch_add(1, Ordering::Relaxed);
    }

    // Wake any async task waiting for data on this subscriber.
    buffer.waker.wake();

    // Issue #0317 — fire the nros-node runtime wake callback (→ wake-latency
    // probe `on_wake` T0 + executor cv-signal) at the real arrival instant. On
    // the multi-threaded backend THIS callback runs on the zenoh-pico read task,
    // so it is the only place the transport-arrival wake can be timestamped
    // (`drive_io`'s poll path returns 0 and never fires it). no_std-safe; a no-op
    // when no executor installed a wake callback.
    super::fire_runtime_wake();

    // Wake the (std BasicExecutor) spin loop's own cvar, if waiting.
    #[cfg(feature = "std")]
    signal_executor_wake();
}

/// Cumulative count of incoming samples that exceeded
/// `SUBSCRIBER_BUFFER_SIZE` and were therefore dropped by the C shim
/// before they could land in any subscriber ring. Bumped by
/// [`subscriber_notify_callback`] when the C side signals an
/// oversized-payload drop. Process-wide counter — every subscriber
/// shares the same atomic, which mirrors how the C shim drops are
/// reported (the notify callback doesn't carry a subscriber-specific
/// slot index past the `ctx` we already use for waker dispatch).
static OVERFLOW_DROPS: portable_atomic::AtomicU32 = portable_atomic::AtomicU32::new(0);

/// Read the cumulative overflow-drop count. Useful for tests that
/// want to assert on the silent-drop path; production code should
/// size the subscriber buffer up-front.
pub fn overflow_drops_total() -> u32 {
    OVERFLOW_DROPS.load(Ordering::Relaxed)
}

// ============================================================================
// ZenohSubscriber
// ============================================================================

/// Zenoh subscriber wrapping nros-rmw-zenoh ZenohSubscriber
pub struct ZenohSubscriber {
    /// The subscriber handle (kept alive to maintain subscription)
    _subscriber: crate::zpico::Subscriber<'static>,
    /// Safe accessor for the static subscriber buffer
    buf: SubscriberBufferRef,
    /// Liveliness token for ROS 2 graph discovery (kept alive for subscriber lifetime)
    _liveliness: Option<super::LivelinessToken>,
    /// E2E safety validator (tracks sequence numbers, validates CRC)
    #[cfg(feature = "safety-e2e")]
    safety_validator: nros_rmw::SafetyValidator,
    /// Phase 108.C.zenoh.5 — next expected sequence number, used to
    /// detect publisher gaps in the attachment-encoded seq stream and
    /// fire `MessageLost` events. Initialised to `0` (= "no message
    /// observed yet"); first `take_serialized` synchronises to the
    /// publisher's seq w/o reporting a gap.
    next_expected_seq: core::cell::Cell<i64>,
    /// Cumulative count of messages dropped between this subscriber's
    /// observed seq stream and the publisher's seq stream. Used as
    /// `CountStatus::total_count` per the nros event contract.
    msg_lost_total: core::cell::Cell<u32>,
    /// Phase 108.A — registered `MessageLost` callback slot.
    msg_lost_cb: core::cell::Cell<Option<EventReg>>,
    /// Issue 0971 — a `take_sequence` that stopped because a message did
    /// not fit parks the reason here, and the NEXT take reports it.
    ///
    /// A batch drain cannot both deliver its count and return an error, so the
    /// answer arrives one call later: the caller gets every message it was
    /// handed, then the reason the drain stopped. `Cell` rather than a plain
    /// field because every take path here works through `&self`.
    ///
    /// Before this the batch path dropped the oversized message and kept
    /// draining, which is the option 0971 rejects outright — "delivers more,
    /// still destroys the message with no signal". The SINGLE take
    /// (`take_serialized`) has always done consume-then-refuse, so the two entry
    /// points contradicted each other on the same subscriber.
    pending_too_small: core::cell::Cell<bool>,
    /// Phase 108.C.zenoh.3 — sample lifespan in ms (`0` = infinite).
    /// Captured from QoS at create time; samples whose attachment
    /// timestamp is older than `now - lifespan_ms` are dropped in
    /// `take_serialized` (return `Ok(None)` as if no data was present).
    lifespan_ms: u32,
    /// Phase 108.C.zenoh.2 — deadline period in ms (`0` = infinite).
    /// Captured from QoS at create time; if `now - last_msg_at_ms`
    /// exceeds it, fire `RequestedDeadlineMissed`.
    deadline_ms: u32,
    /// Last successful receive timestamp in ms (platform clock).
    /// Initialised at creation time to suppress an immediate "missed"
    /// at sub-create.
    last_msg_at_ms: core::cell::Cell<u64>,
    /// Last `RequestedDeadlineMissed` fire-time so we don't spam
    /// callbacks for a continually-late publisher; we fire at most
    /// once per deadline period.
    last_deadline_fire_ms: core::cell::Cell<u64>,
    /// Cumulative `RequestedDeadlineMissed` count, used as
    /// `CountStatus::total_count`.
    deadline_total: core::cell::Cell<u32>,
    /// Cumulative dropped-by-lifespan count (folded into
    /// `MessageLost` events — lifespan-expired samples count as lost).
    deadline_cb: core::cell::Cell<Option<EventReg>>,
    /// Phase 108.C.zenoh.4 — registered `LivelinessChanged` callback.
    /// Fired from `has_data` / `take_serialized` after a periodic
    /// `liveliness_get_*` poll detects an alive-state transition for
    /// any publisher matching the subscriber's wildcard liveliness
    /// keyexpr.
    #[cfg(not(feature = "platform-bare-metal"))]
    liveliness_cb: core::cell::Cell<Option<EventReg>>,
    /// Wildcard liveliness keyexpr matching any publisher on this
    /// subscriber's (topic, type). Populated at create.
    #[cfg(not(feature = "platform-bare-metal"))]
    liveliness_keyexpr: heapless::String<256>,
    /// Liveliness-poll context — handle of an in-flight
    /// `liveliness_get_start` query (None = idle), the timestamp of
    /// the most recent poll start, and the previously observed alive
    /// state.
    #[cfg(not(feature = "platform-bare-metal"))]
    liveliness_poll: core::cell::Cell<LivelinessPoll>,
    /// Raw pointer to the owning session's `Context`. Used by the
    /// LIVELINESS poll loop to issue `liveliness_get_*` queries.
    /// SAFETY: the Context is owned by `ZenohSession`, which outlives
    /// every entity it spawns (entities are created via Session and
    /// dropped before Session::close).
    #[cfg(not(feature = "platform-bare-metal"))]
    context: *const Context,
    /// Phantom to indicate we don't own the buffer
    _phantom: PhantomData<()>,
}

/// Phase 108.C.zenoh.4 — liveliness-poll state. Owned by the
/// subscriber via `Cell` since the subscriber is `!Sync`.
#[derive(Clone, Copy)]
#[cfg(not(feature = "platform-bare-metal"))]
struct LivelinessPoll {
    /// Slot handle of an in-flight `liveliness_get_start` query, or
    /// `-1` when idle.
    handle: i32,
    /// Wall-clock ms when the most recent poll was started.
    started_at_ms: u64,
    /// Last observed alive-state (any matching publisher visible).
    /// Initialised to `false`; the first transition to `true` fires
    /// `alive_count_change = +1`.
    last_alive: bool,
    /// Cumulative running count for `LivelinessChangedStatus.alive_count`.
    alive_count: u16,
}

#[cfg(not(feature = "platform-bare-metal"))]
impl LivelinessPoll {
    const IDLE: Self = Self {
        handle: -1,
        started_at_ms: 0,
        last_alive: false,
        alive_count: 0,
    };
}

/// Liveliness-poll cadence. We don't expose a knob because polling
/// faster than ~1 Hz spams the network without benefit; coarser than
/// ~5 s loses transitions. Sub side honors `liveliness_lease_ms` from
/// QoS by clamping the poll window to half the lease (so we observe
/// at least two probes per lease period).
#[cfg(not(feature = "platform-bare-metal"))]
const LIVELINESS_POLL_DEFAULT_MS: u64 = 1_000;
#[cfg(not(feature = "platform-bare-metal"))]
const LIVELINESS_POLL_TIMEOUT_MS: u32 = 100;

/// Phase 108.A — single-slot event registration. The cb is
/// `unsafe extern "C" fn` (always Send); user_ctx outlives the
/// subscriber per Phase 108.A.7's per-entity event registry.
#[derive(Clone, Copy)]
struct EventReg {
    cb: nros_rmw::EventCallback,
    user_ctx: *mut core::ffi::c_void,
}

/// Phase 108.C.zenoh — read the platform clock in ms.
///
/// Phase 129.C.3.a — call the canonical `nros_platform_*` C
/// symbol directly instead of routing through `ConcretePlatform`.
fn now_ms() -> u64 {
    unsafe extern "C" {
        fn nros_platform_time_now_ns() -> u64;
    }
    // Issue 0532 item 5 — the ABI is nanoseconds now; this caller wants ms.
    unsafe { nros_platform_time_now_ns() / 1_000_000 }
}

impl ZenohSubscriber {
    /// Create a new subscriber for the given topic
    pub fn new(
        context: &Context,
        topic: &nros_rmw::TopicInfo,
        liveliness: Option<super::LivelinessToken>,
        qos: &nros_rmw::QoSProfile,
    ) -> Result<Self, TransportError> {
        // Phase 108.C.zenoh.4 — wildcard liveliness keyexpr matching
        // any publisher on this (topic, type). Built once and stored
        // for reuse on each LIVELINESS poll on hosted targets.
        #[cfg(not(feature = "platform-bare-metal"))]
        let liveliness_keyexpr: heapless::String<256> =
            super::Ros2Liveliness::publisher_keyexpr_wildcard(topic.domain_id, topic);
        // Allocate a buffer index
        let buffer_index = NEXT_BUFFER_INDEX.fetch_add(1, Ordering::SeqCst);
        if buffer_index >= ZPICO_MAX_SUBSCRIBERS {
            // Roll back and return error
            // BEFORE the rollback, and the INDEX rather than the counter --
            // the first cut read the counter afterwards and reported 0 for a
            // pool of 10, which is the wrong-number-that-looks-like-a-number
            // this record exists to stop producing.
            record_buffer_index(buffer_index);
            NEXT_BUFFER_INDEX.fetch_sub(1, Ordering::SeqCst);
            // phase-412 -- refusal 4. This guard runs BEFORE the payload classes,
            // so without recording here the report shows every block allocated
            // and no refusal, and the failure looks like it came from somewhere
            // else entirely. `SubscriberCreationFailed` then crosses the C ABI
            // through `ret_from_error`'s `_ => NROS_RMW_RET_ERROR` fallback and
            // comes back as `Backend("rmw_ret error")`, losing the last of its
            // identity.
            record_alloc(topic.rx_buffer_hint, 4);
            return Err(TransportError::SubscriberCreationFailed);
        }

        // Phase 231 (RFC-0038) — reserve a size-class payload block (small vs
        // large by the topic's rx_buffer_hint). Roll back the metadata index if
        // the class is exhausted.
        let Some((payload_base, payload_stride)) = alloc_payload_block(topic.rx_buffer_hint) else {
            NEXT_BUFFER_INDEX.fetch_sub(1, Ordering::SeqCst);
            return Err(TransportError::SubscriberCreationFailed);
        };

        let mut buf = SubscriberBufferRef::new(buffer_index);

        // Generate the topic key with wildcard for type hash
        let key: heapless::String<KEYEXPR_STRING_SIZE> = topic.to_key_wildcard();

        #[cfg(feature = "std")]
        log::debug!("Subscriber data keyexpr: {}", key.as_str());

        // Create null-terminated keyexpr
        let mut keyexpr_buf = [0u8; KEYEXPR_BUFFER_SIZE];
        let bytes = key.as_bytes();
        record_keyexpr(bytes.len(), KEYEXPR_STRING_SIZE);
        if bytes.len() >= keyexpr_buf.len() {
            return Err(TransportError::TopicNameInvalid);
        }
        keyexpr_buf[..bytes.len()].copy_from_slice(bytes);
        keyexpr_buf[bytes.len()] = 0;

        // Phase 124.D.3.c — create subscriber with the SPSC ring.
        // The C shim reads each payload directly into the next free
        // ring slot of SUBSCRIBER_BUFFERS[buffer_index] via
        // `z_bytes_reader_read()`, advances `ring_tail`, and fires
        // the notify callback. A burst is buffered up to
        // SUBSCRIBER_RING_DEPTH deep instead of overwriting a single
        // slot. `init_ring_desc` populates the descriptor's raw
        // pointers from the buffer's (stable) static address.
        let subscriber = unsafe {
            let buffer = buf.get_mut();
            buffer.init_ring_desc(payload_base, payload_stride);
            let desc_ptr: *mut zpico_ring_desc_t = &mut buffer.ring_desc;
            let sub_result = context.declare_subscriber_ring_raw(
                &keyexpr_buf,
                desc_ptr,
                subscriber_notify_callback,
                buffer_index as *mut core::ffi::c_void,
            );
            match sub_result {
                Ok(s) => core::mem::transmute::<
                    crate::zpico::Subscriber<'_>,
                    crate::zpico::Subscriber<'static>,
                >(s),
                Err(e) => {
                    NEXT_BUFFER_INDEX.fetch_sub(1, Ordering::SeqCst);
                    record_zpico_err(zpico_err_class(&e));
                    return Err(TransportError::from(e));
                }
            }
        };

        let now = now_ms();
        Ok(Self {
            _subscriber: subscriber,
            buf,
            _liveliness: liveliness,
            #[cfg(feature = "safety-e2e")]
            safety_validator: nros_rmw::SafetyValidator::new(),
            next_expected_seq: core::cell::Cell::new(0),
            msg_lost_total: core::cell::Cell::new(0),
            msg_lost_cb: core::cell::Cell::new(None),
            pending_too_small: core::cell::Cell::new(false),
            lifespan_ms: qos.lifespan_ms,
            deadline_ms: qos.deadline_ms,
            last_msg_at_ms: core::cell::Cell::new(now),
            last_deadline_fire_ms: core::cell::Cell::new(now),
            deadline_total: core::cell::Cell::new(0),
            deadline_cb: core::cell::Cell::new(None),
            #[cfg(not(feature = "platform-bare-metal"))]
            liveliness_cb: core::cell::Cell::new(None),
            #[cfg(not(feature = "platform-bare-metal"))]
            liveliness_keyexpr,
            #[cfg(not(feature = "platform-bare-metal"))]
            liveliness_poll: core::cell::Cell::new(LivelinessPoll::IDLE),
            #[cfg(not(feature = "platform-bare-metal"))]
            context: context as *const Context,
            _phantom: PhantomData,
        })
    }

    pub(super) fn set_liveliness(&mut self, liveliness: Option<super::LivelinessToken>) {
        self._liveliness = liveliness;
    }

    /// Phase 108.C.zenoh.4 — liveliness poll loop. Polls `zpico`'s
    /// one-shot `liveliness_get_*` API on a coarse cadence (default
    /// 1s, halved when QoS sets `liveliness_lease_ms`) and fires
    /// `LivelinessChanged` on alive-state transitions. Single-slot
    /// alive (any matching publisher) — DDS's per-publisher
    /// alive_count is approximated to {0, 1}; ROS 2 apps that only
    /// care about "any publisher present" get correct semantics, apps
    /// counting individual publishers see one entry. Exact per-pub
    /// counting needs a long-lived `z_liveliness_declare_subscriber`
    /// shim, which is the next sub-phase if requested.
    fn check_liveliness_and_fire(&self) {
        #[cfg(feature = "platform-bare-metal")]
        {
            return;
        }

        #[cfg(not(feature = "platform-bare-metal"))]
        {
            if self.liveliness_cb.get().is_none() {
                return; // No callback registered → don't burn cycles polling.
            }
            // SAFETY: see `context` field doc.
            let context: &Context = unsafe { &*self.context };
            let now = now_ms();
            let mut state = self.liveliness_poll.get();

            // 1. If a query is in flight, poll it; on completion record
            //    the new alive state and clear the handle.
            //
            // Phase 108.C.zenoh.4-followup — read `liveliness_get_count`
            // BEFORE `liveliness_get_check` because the latter releases the
            // slot on terminal result.
            if state.handle >= 0 {
                let count = context.liveliness_get_count(state.handle).unwrap_or(0);
                match context.liveliness_get_check(state.handle) {
                    Ok(true) => {
                        // At least one matching token responded; `count` is
                        // the exact reply count.
                        self.handle_count_transition(count.max(1) as u16, &mut state);
                    }
                    Ok(false) => {
                        // Still waiting; keep handle for next poll.
                    }
                    Err(_) => {
                        // Timeout (no matching publisher) or error → 0 alive.
                        self.handle_count_transition(0, &mut state);
                    }
                }
            }

            // 2. If idle and the cadence has elapsed, start a fresh query.
            if state.handle < 0 {
                let interval = self.liveliness_poll_interval_ms();
                if now >= state.started_at_ms.saturating_add(interval) {
                    // Liveliness keyexpr must be null-terminated for the
                    // C bridge.
                    let mut nul = heapless::Vec::<u8, 257>::new();
                    let _ = nul.extend_from_slice(self.liveliness_keyexpr.as_bytes());
                    let _ = nul.push(0);
                    if let Ok(handle) =
                        context.liveliness_get_start(nul.as_slice(), LIVELINESS_POLL_TIMEOUT_MS)
                    {
                        state.handle = handle;
                        state.started_at_ms = now;
                    }
                }
            }

            self.liveliness_poll.set(state);
        }
    }

    /// Phase 108.C.zenoh.4-followup — fire `LivelinessChanged` with
    /// the actual delta between the previous and new alive count.
    /// `new_count` is the number of unique publishers that responded
    /// to the most recent wildcard liveliness query.
    #[cfg(not(feature = "platform-bare-metal"))]
    fn handle_count_transition(&self, new_count: u16, state: &mut LivelinessPoll) {
        // Always clear the handle on terminal result.
        state.handle = -1;
        let prev = state.alive_count;
        if new_count == prev {
            // No transition — also keep last_alive in sync for any
            // legacy field dependents.
            state.last_alive = new_count > 0;
            return;
        }
        let (alive_count_change, not_alive_count_change) = if new_count > prev {
            ((new_count - prev) as i16, 0i16)
        } else {
            (-((prev - new_count) as i16), (prev - new_count) as i16)
        };
        state.alive_count = new_count;
        state.last_alive = new_count > 0;
        if let Some(reg) = self.liveliness_cb.get() {
            let status = nros_rmw::LivelinessChangedStatus {
                alive_count: new_count,
                not_alive_count: 0,
                alive_count_change,
                not_alive_count_change,
            };
            // SAFETY: cb is `unsafe extern "C" fn`; user_ctx outlives
            // entity per Phase 108.A.7.
            unsafe {
                (reg.cb)(
                    nros_rmw::EventKind::LivelinessChanged,
                    &status as *const _ as *const core::ffi::c_void,
                    reg.user_ctx,
                );
            }
        }
    }

    #[cfg(not(feature = "platform-bare-metal"))]
    fn liveliness_poll_interval_ms(&self) -> u64 {
        // Half the lease so we observe ≥ 2 probes per lease window.
        // 0 (no lease set) → default 1s.
        // Any backend that fires this code path also has a working
        // platform clock so non-zero `now` is guaranteed.
        // We don't propagate the QoS field through to here yet (would
        // need another `Cell<u32>` field); use the default for now.
        LIVELINESS_POLL_DEFAULT_MS
    }

    /// Phase 108.C.zenoh.3 — read the publisher-supplied timestamp
    /// out of the most recent attachment. Returns `0` if no attachment
    /// is present. Called from `take_serialized` to enforce LIFESPAN.
    fn attachment_timestamp_ms(&self) -> u64 {
        let buffer = self.buf.get();
        // Inspect the head ring slot — the message `take_serialized` is
        // about to deliver. Empty ring → no timestamp.
        let Some(slot) = buffer.peek_head_slot() else {
            return 0;
        };
        let attachment_len = buffer.ring_att_len[slot];
        if attachment_len < RMW_ATTACHMENT_SIZE {
            return 0;
        }
        let att = &buffer.ring_att[slot];
        // Bytes 8..16 are the i64 timestamp (LE) per
        // ZenohPublisher::serialize_attachment. Convert ns → ms.
        let ts_ns = i64::from_le_bytes([
            att[8], att[9], att[10], att[11], att[12], att[13], att[14], att[15],
        ]);
        if ts_ns <= 0 {
            0
        } else {
            (ts_ns as u64) / 1_000_000
        }
    }

    /// Phase 108.C.zenoh.2 — fire the registered `RequestedDeadlineMissed`
    /// callback when the gap since the last successful receive exceeds
    /// `deadline_ms`. Called from `has_data` / `take_serialized` so deadline
    /// is checked on every spin cycle that touches this subscriber.
    /// Rate-limited: at most one fire per deadline period.
    fn check_deadline_and_fire(&self) {
        if self.deadline_ms == 0 {
            return;
        }
        let now = now_ms();
        let last = self.last_msg_at_ms.get();
        if now < last.saturating_add(self.deadline_ms as u64) {
            return; // Within deadline.
        }
        let last_fire = self.last_deadline_fire_ms.get();
        if now < last_fire.saturating_add(self.deadline_ms as u64) {
            return; // Already fired this deadline period.
        }
        self.last_deadline_fire_ms.set(now);
        let total = self.deadline_total.get().saturating_add(1);
        self.deadline_total.set(total);
        if let Some(reg) = self.deadline_cb.get() {
            let status = nros_rmw::CountStatus {
                total_count: total,
                total_count_change: 1,
            };
            // SAFETY: cb is `unsafe extern "C" fn`; user_ctx outlives
            // entity per Phase 108.A.7's per-entity event registry.
            unsafe {
                (reg.cb)(
                    nros_rmw::EventKind::RequestedDeadlineMissed,
                    &status as *const _ as *const core::ffi::c_void,
                    reg.user_ctx,
                );
            }
        }
    }

    /// Phase 108.C.zenoh.5 — peek the just-received attachment for a
    /// sequence number, detect gaps against `next_expected_seq`, and
    /// fire the registered `MessageLost` callback if any are dropped.
    /// Called from `take_serialized` AFTER the payload is copied so the
    /// status-event delivery is observable to the user as a side-
    /// effect of receive (matching dust-DDS sample-lost semantics).
    fn check_msg_lost_and_fire(&self) {
        let buffer = self.buf.get();
        // Inspect the head ring slot — the message just copied out by
        // `take_serialized`, not yet consumed.
        let Some(slot) = buffer.peek_head_slot() else {
            return;
        };
        let attachment_len = buffer.ring_att_len[slot];
        if attachment_len < RMW_ATTACHMENT_SIZE {
            return; // No attachment, no seq → can't detect gaps.
        }
        let att = &buffer.ring_att[slot];
        let seq = i64::from_le_bytes([
            att[0], att[1], att[2], att[3], att[4], att[5], att[6], att[7],
        ]);
        let expected = self.next_expected_seq.get();
        // First message: synchronise w/o reporting; expected stays 0
        // until we see a real seq, then we set expected = seq + 1.
        let gap = if expected == 0 {
            0
        } else if seq > expected {
            (seq - expected) as u64
        } else {
            // Out-of-order or duplicate — treat as zero loss.
            0
        };
        self.next_expected_seq.set(seq.saturating_add(1));
        if gap == 0 {
            return;
        }
        let delta = u32::try_from(gap).unwrap_or(u32::MAX);
        let total = self.msg_lost_total.get().saturating_add(delta);
        self.msg_lost_total.set(total);
        if let Some(reg) = self.msg_lost_cb.get() {
            let status = nros_rmw::CountStatus {
                total_count: total,
                total_count_change: delta,
            };
            // SAFETY: cb is `unsafe extern "C" fn` matching
            // EventCallback; user_ctx outlives this call (entity owns
            // the Box backing it; freed in nros-node's per-entity
            // event-registry on Drop).
            unsafe {
                (reg.cb)(
                    nros_rmw::EventKind::MessageLost,
                    &status as *const _ as *const core::ffi::c_void,
                    reg.user_ctx,
                );
            }
        }
    }
}

impl ZenohSubscriber {
    /// Try to receive a validated message with E2E integrity status.
    ///
    /// Checks CRC-32 integrity and sequence continuity. Returns
    /// `(payload_len, IntegrityStatus)` so the caller can decide whether
    /// to trust the data.
    ///
    /// The payload bytes are written to `buf[..len]`.
    #[cfg(feature = "safety-e2e")]
    pub fn take_validated(
        &mut self,
        buf: &mut [u8],
    ) -> Result<Option<(usize, nros_rmw::IntegrityStatus)>, TransportError> {
        let buffer = self.buf.get();

        let Some(slot) = buffer.peek_head_slot() else {
            return Ok(None);
        };

        let len = buffer.ring_len[slot];
        if len > buf.len() {
            // Oversized for the caller's buffer — drop the slot so the
            // subscription isn't permanently stuck.
            buffer.consume_head();
            return Err(TransportError::BufferTooSmall);
        }

        // Copy payload out of the ring slot. SPSC: the C producer
        // never touches this slot while head points at it.
        buf[..len].copy_from_slice(buffer.payload_slot(slot, len));

        // Parse attachment for sequence number and CRC.
        let attachment_len = buffer.ring_att_len[slot];
        let (message_seq, crc_valid) = if attachment_len >= RMW_ATTACHMENT_SIZE {
            let att = &buffer.ring_att[slot];
            let seq = i64::from_le_bytes([
                att[0], att[1], att[2], att[3], att[4], att[5], att[6], att[7],
            ]);

            let crc_result = if attachment_len >= RMW_ATTACHMENT_SIZE + SAFETY_CRC_SIZE {
                let received_crc = u32::from_le_bytes([
                    att[RMW_ATTACHMENT_SIZE],
                    att[RMW_ATTACHMENT_SIZE + 1],
                    att[RMW_ATTACHMENT_SIZE + 2],
                    att[RMW_ATTACHMENT_SIZE + 3],
                ]);
                let computed_crc = nros_rmw::crc32(&buf[..len]);
                Some(received_crc == computed_crc)
            } else {
                None
            };

            (seq, crc_result)
        } else {
            (0, None)
        };

        buffer.consume_head();

        let status = self.safety_validator.validate(message_seq, crc_valid);
        Ok(Some((len, status)))
    }

    /// Try to receive raw data along with message info from attachment
    ///
    /// Returns `Ok(Some((len, info)))` if data is available, where:
    /// - `len` is the number of bytes written to the buffer
    /// - `info` is the parsed message info (if attachment was present)
    ///
    /// Returns `Ok(None)` if no data is available.
    pub fn take_with_info(
        &mut self,
        buf: &mut [u8],
    ) -> Result<Option<(usize, Option<MessageInfo>)>, TransportError> {
        let buffer = self.buf.get();

        let Some(slot) = buffer.peek_head_slot() else {
            return Ok(None);
        };

        let len = buffer.ring_len[slot];
        if len > buf.len() {
            // Oversized for the caller's buffer — drop the slot; the
            // subscription recovers on the next message.
            buffer.consume_head();
            return Err(TransportError::BufferTooSmall);
        }

        buf[..len].copy_from_slice(buffer.payload_slot(slot, len));

        let attachment_len = buffer.ring_att_len[slot];
        let message_info = if attachment_len > 0 {
            MessageInfo::from_attachment(&buffer.ring_att[slot][..attachment_len])
        } else {
            None
        };

        buffer.consume_head();

        Ok(Some((len, message_info)))
    }
}

impl Subscription for ZenohSubscriber {
    type Error = TransportError;

    fn register_waker(&self, waker: &core::task::Waker) {
        self.buf.get().waker.register(waker);
    }

    fn take_serialized(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Self::Error> {
        // Issue 0971 — report a drain that stopped on an oversized message
        // before taking anything. Cyclone clears its flag in BOTH take entry
        // points (`subscriber.cpp:226` and `:296`) for the same reason: the
        // caller that next asks this subscriber for data is the one owed the
        // explanation, and it may well ask through the single take rather than
        // another batch.
        if self.pending_too_small.replace(false) {
            return Err(TransportError::BufferTooSmall);
        }
        let buffer = self.buf.get();

        // Phase 108.C.zenoh.2 — check deadline expiry on every poll
        // (whether or not data is ready). Rate-limited internally.
        self.check_deadline_and_fire();

        let Some(slot) = buffer.peek_head_slot() else {
            return Ok(None);
        };

        let len = buffer.ring_len[slot];
        if len > buf.len() {
            // Oversized for the caller's buffer — drop the slot; the
            // subscription recovers on the next message.
            buffer.consume_head();
            return Err(TransportError::BufferTooSmall);
        }

        // Phase 108.C.zenoh.3 — LIFESPAN check. If the sample's
        // attachment timestamp is older than `now - lifespan_ms`, drop
        // it. The dropped sample counts as a missed delivery from the
        // subscriber's POV, but we don't fire MessageLost here —
        // lifespan-expired samples aren't "lost in transit", they
        // arrived but were filtered.
        if self.lifespan_ms != 0 {
            let ts = self.attachment_timestamp_ms();
            if ts != 0 {
                let now = now_ms();
                if now > ts.saturating_add(self.lifespan_ms as u64) {
                    buffer.consume_head();
                    return Ok(None);
                }
            }
        }

        // Copy data out of the ring slot. SPSC: the C producer never
        // touches this slot while head points at it.
        buf[..len].copy_from_slice(buffer.payload_slot(slot, len));

        // Phase 108.C.zenoh.5 — detect publisher seq gap before
        // advancing head so the attachment is still valid.
        self.check_msg_lost_and_fire();
        // Phase 108.C.zenoh.2 — successful receive resets deadline.
        self.last_msg_at_ms.set(now_ms());

        buffer.consume_head();

        Ok(Some(len))
    }

    fn has_data(&self) -> bool {
        // Phase 108.C.zenoh.2 — opportunistically check deadline on
        // every has_data poll. Cheap (one clock read + compare). The
        // executor calls has_data each spin to scan the readiness
        // bitmap, so this gives deadline checks the same cadence as
        // message dispatch.
        self.check_deadline_and_fire();
        // Phase 108.C.zenoh.4 — drive the LIVELINESS poll loop on the
        // same cadence. The loop has its own internal time-gated
        // start, so calling on every has_data is cheap (one clock
        // read + cell-load + cell-store when idle).
        self.check_liveliness_and_fire();
        self.buf.get().has_data()
    }

    fn supports_event(&self, kind: nros_rmw::EventKind) -> bool {
        // Phase 108.C.zenoh — MessageLost via attachment seq gap (.5),
        // RequestedDeadlineMissed via clock-based poll (.2),
        // LivelinessChanged surface only (.4) — global liveliness-
        // subscriber bridge fires it from a session-side
        // z_liveliness_declare_subscriber callback. LIFESPAN is a
        // filter, not an event, so no event kind for it.
        if matches!(
            kind,
            nros_rmw::EventKind::MessageLost | nros_rmw::EventKind::RequestedDeadlineMissed
        ) {
            return true;
        }

        #[cfg(not(feature = "platform-bare-metal"))]
        {
            matches!(kind, nros_rmw::EventKind::LivelinessChanged)
        }
        #[cfg(feature = "platform-bare-metal")]
        {
            false
        }
    }

    unsafe fn register_event_callback(
        &mut self,
        kind: nros_rmw::EventKind,
        deadline_ms: u32,
        cb: nros_rmw::EventCallback,
        user_ctx: *mut core::ffi::c_void,
    ) -> Result<(), TransportError> {
        match kind {
            nros_rmw::EventKind::MessageLost => {
                self.msg_lost_cb.set(Some(EventReg { cb, user_ctx }));
                Ok(())
            }
            nros_rmw::EventKind::RequestedDeadlineMissed => {
                // The Phase 108 doc says deadline_ms is consulted only
                // for this event kind; if QoS already declared a
                // non-zero deadline_ms at create time, prefer that.
                // Otherwise allow the registration to set/upgrade it.
                if self.deadline_ms == 0 && deadline_ms != 0 {
                    // SAFETY: lifespan_ms / deadline_ms are inherent
                    // u32 fields; we set via an interior write. No
                    // aliasing concern because Subscriber is owned by
                    // a single thread (`!Sync`).
                    let p = self as *const Self as *mut Self;
                    unsafe { (*p).deadline_ms = deadline_ms };
                }
                self.deadline_cb.set(Some(EventReg { cb, user_ctx }));
                Ok(())
            }
            nros_rmw::EventKind::LivelinessChanged => {
                #[cfg(feature = "platform-bare-metal")]
                {
                    Err(TransportError::Unsupported)
                }
                #[cfg(not(feature = "platform-bare-metal"))]
                {
                    // Slot landed; the session-side liveliness shim that
                    // routes z_liveliness_declare_subscriber callbacks to
                    // these slots is part of 108.C.zenoh.4 follow-up; for
                    // now the slot accepts registrations but never fires.
                    self.liveliness_cb.set(Some(EventReg { cb, user_ctx }));
                    Ok(())
                }
            }
            _ => Err(TransportError::Unsupported),
        }
    }

    fn supports_process_in_place(&self) -> bool {
        true
    }

    fn process_raw_in_place(&mut self, f: impl FnOnce(&[u8])) -> Result<bool, Self::Error> {
        let buffer = self.buf.get();

        let Some(slot) = buffer.peek_head_slot() else {
            return Ok(false);
        };

        let len = buffer.ring_len[slot];
        // Process in-place out of the ring slot, then advance head.
        f(buffer.payload_slot(slot, len));
        buffer.consume_head();

        Ok(true)
    }

    // Phase 231 Wave 0.1 — in-place dispatch with the co-located attachment.
    // Borrows the ring slot's payload + parses its attachment into the canonical
    // `nros_core::MessageInfo` (same conversion as `take_serialized_with_info`) for
    // `f`, then advances head. Promoted from the former inherent method.
    fn process_raw_in_place_with_info(
        &mut self,
        f: impl FnOnce(&[u8], Option<nros_core::MessageInfo>),
    ) -> Result<bool, Self::Error> {
        let buffer = self.buf.get();

        let Some(slot) = buffer.peek_head_slot() else {
            return Ok(false);
        };

        let len = buffer.ring_len[slot];

        // Parse attachment (small: 33-37 bytes) into the core MessageInfo.
        let attachment_len = buffer.ring_att_len[slot];
        let core_info = if attachment_len > 0 {
            MessageInfo::from_attachment(&buffer.ring_att[slot][..attachment_len]).map(|zi| {
                let mut info = nros_core::MessageInfo::new();
                info.set_publication_sequence_number(zi.sequence_number);
                info.set_source_timestamp(nros_core::Time::from_nanos(zi.timestamp_ns));
                info.set_publisher_gid(zi.publisher_gid);
                info
            })
        } else {
            None
        };

        f(buffer.payload_slot(slot, len), core_info);

        buffer.consume_head();

        Ok(true)
    }

    // Phase 124.D.3.c — native batch take. Drains up to `max_msgs`
    // queued messages out of the SPSC ring in one call, each into a
    // `per_msg_cap`-strided slot of `buf`. Returns the count actually
    // delivered.
    //
    // Issue 0971 — an oversized message (payload > per_msg_cap) STOPS the
    // drain: it is consumed, the reason is parked, and the next take on this
    // subscriber returns `BufferTooSmall`. The count already drained is still
    // returned, so the caller keeps every message it was handed and then learns
    // why the drain ended.
    //
    // This comment used to say oversized messages "are dropped individually
    // rather than erroring the whole batch — the burst-drain caller wants
    // forward progress", and the code did exactly that. Forward progress was
    // the right instinct and silence was the wrong price: the caller could not
    // distinguish "the burst ended" from "a message was destroyed". 0971
    // rejects that option by name. Parking keeps the progress and adds the
    // signal, one call later.
    fn take_sequence(
        &mut self,
        buf: &mut [u8],
        per_msg_cap: usize,
        max_msgs: usize,
        out_lens: &mut [usize],
    ) -> Result<usize, Self::Error> {
        // Issue 0971 — a previous drain stopped on a message that did not fit.
        // Report it now, before taking anything: the caller already has the
        // messages that drain delivered, and this is the call that tells it why
        // the drain ended. Cleared as it is reported, so the next call proceeds
        // normally.
        if self.pending_too_small.replace(false) {
            return Err(TransportError::BufferTooSmall);
        }
        if per_msg_cap == 0 || max_msgs == 0 {
            return Ok(0);
        }
        let buffer = self.buf.get();
        let limit = max_msgs.min(out_lens.len());
        let need = limit
            .checked_mul(per_msg_cap)
            .ok_or(TransportError::BufferTooSmall)?;
        if buf.len() < need {
            return Err(TransportError::BufferTooSmall);
        }

        let mut count = 0;
        while count < limit {
            let Some(slot) = buffer.peek_head_slot() else {
                break;
            };
            let len = buffer.ring_len[slot];
            if len > per_msg_cap {
                // Issue 0971 — the message does not fit the caller's per-slot
                // cap. Consume it (leaving it would wedge the subscription on a
                // message nobody can ever take), PARK the reason, and STOP.
                //
                // It used to `continue`: drop it and keep draining. That is the
                // option 0971 rejects — the caller gets more messages and never
                // learns one was destroyed. Stopping matches Cyclone's batch
                // (`subscriber.cpp:341`) so the two backends now agree on the
                // count as well as the status; continuing would have made zenoh
                // deliver a different number of messages for the same burst.
                buffer.consume_head();
                self.pending_too_small.set(true);
                break;
            }
            let off = count * per_msg_cap;
            buf[off..off + len].copy_from_slice(buffer.payload_slot(slot, len));
            out_lens[count] = len;
            buffer.consume_head();
            count += 1;
        }
        // A successful drain resets the deadline like a single recv.
        if count > 0 {
            self.last_msg_at_ms.set(now_ms());
        }
        Ok(count)
    }

    fn take_serialized_with_info(
        &mut self,
        buf: &mut [u8],
    ) -> Result<Option<(usize, Option<nros_core::MessageInfo>)>, Self::Error> {
        // Delegate to the inherent method which parses the zenoh attachment
        match self.take_with_info(buf)? {
            Some((len, zenoh_info)) => {
                let core_info = zenoh_info.map(|zi| {
                    let mut info = nros_core::MessageInfo::new();
                    info.set_publication_sequence_number(zi.sequence_number);
                    info.set_source_timestamp(nros_core::Time::from_nanos(zi.timestamp_ns));
                    info.set_publisher_gid(zi.publisher_gid);
                    info
                });
                Ok(Some((len, core_info)))
            }
            None => Ok(None),
        }
    }

    #[cfg(feature = "safety-e2e")]
    fn take_validated(
        &mut self,
        buf: &mut [u8],
    ) -> Result<Option<(usize, nros_rmw::IntegrityStatus)>, Self::Error> {
        // Delegate to the inherent safety validation method
        ZenohSubscriber::take_validated(self, buf)
    }

    fn deserialization_error(&self) -> Self::Error {
        TransportError::DeserializationError
    }
}

// ============================================================================
// Phase 99.F — ZenohSubscriber SlotBorrowing (zero-copy receive)
// ============================================================================

#[cfg(feature = "lending")]
mod borrowing {
    use super::*;

    /// Backend-lent read-only view into the subscriber's static receive
    /// buffer. Phase 124.D.3.c — borrows the head ring slot's payload
    /// for the lifetime of the view. The SPSC discipline guarantees
    /// the C producer never writes the slot `ring_head` points at, so
    /// no explicit lock is needed; `Drop` advances `ring_head`
    /// (consume-on-borrow semantics, matching `take_serialized`).
    pub struct ZenohView<'a> {
        bytes: &'a [u8],
        buffer: &'a SubscriberBuffer,
    }

    impl<'a> AsRef<[u8]> for ZenohView<'a> {
        fn as_ref(&self) -> &[u8] {
            self.bytes
        }
    }

    impl<'a> Drop for ZenohView<'a> {
        fn drop(&mut self) {
            // Advance the consumer counter so the borrowed slot is
            // released back to the C producer.
            self.buffer.consume_head();
        }
    }

    impl nros_rmw::SlotBorrowing for ZenohSubscriber {
        type View<'a>
            = ZenohView<'a>
        where
            Self: 'a;

        fn try_borrow(&mut self) -> Result<Option<Self::View<'_>>, TransportError> {
            // SubscriberBufferRef::get() returns a &SubscriberBuffer whose
            // backing storage is 'static (lives in SUBSCRIBER_BUFFERS).
            // Re-tie that 'static reference to the lifetime of `&mut self`
            // by wrapping it in ZenohView (whose `'_` is implicit on
            // `Self::View<'_>` and bound by `Self: 'a` in the trait def).
            let buffer = self.buf.get();

            let Some(slot) = buffer.peek_head_slot() else {
                return Ok(None);
            };
            let len = buffer.ring_len[slot];

            // SAFETY: SPSC — the C producer never writes the head slot
            // while `ring_head` points at it (its full-check stops it
            // from lapping the consumer). The borrow is valid until
            // ZenohView::drop advances `ring_head`.
            let bytes = buffer.payload_slot(slot, len);

            Ok(Some(ZenohView { bytes, buffer }))
        }
    }
}

#[cfg(feature = "lending")]
pub use borrowing::ZenohView;

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
pub(super) mod tests {
    extern crate alloc;
    use super::*;
    use nros_rmw::TransportError;

    // --- Subscription buffer helpers ---

    /// Phase 124.D.3.c — simulate the C-shim SPSC ring producer.
    /// Pushes `payload` into the slot at `ring_tail % DEPTH` and
    /// advances `ring_tail`. Drops the message (no advance) when the
    /// ring is full, the payload is oversized, or the payload is
    /// empty — exactly mirroring the C `sample_handler` ring branch.
    pub(in crate::shim) fn simulate_subscription_callback(slot: usize, payload: &[u8]) {
        let mut buf_ref = SubscriberBufferRef::new(slot);
        let buffer = buf_ref.get_mut();

        // Phase 231 (RFC-0038) — these test buffers don't go through
        // `ZenohSubscriber::new`, so wire a small-class payload block (same
        // index) into the descriptor the first time, mirroring `init_ring_desc`.
        if buffer.ring_desc.payload_base.is_null() {
            let base = unsafe { (&raw mut SMALL_PAYLOADS[slot]) as *mut u8 };
            buffer.init_ring_desc(base, SUBSCRIBER_BUFFER_SIZE);
        }

        if payload.is_empty() {
            return; // Empty probe — dropped by the C producer.
        }
        let head = buffer.ring_head.load(Ordering::Acquire);
        let tail = buffer.ring_tail.load(Ordering::Acquire);
        if tail - head >= SUBSCRIBER_RING_DEPTH {
            return; // Ring full — drop.
        }
        if payload.len() > SUBSCRIBER_BUFFER_SIZE {
            return; // Oversized for a slot — drop.
        }
        let s = tail % SUBSCRIBER_RING_DEPTH;
        let plen = payload.len();
        buffer.payload_slot_mut(s, plen).copy_from_slice(payload);
        buffer.ring_len[s] = payload.len();
        buffer.ring_att_len[s] = 0;
        buffer.ring_tail.store(tail + 1, Ordering::Release);
    }

    /// Reset a subscriber ring to the empty state.
    pub(in crate::shim) fn reset_subscriber_buffer(slot: usize) {
        let mut buf_ref = SubscriberBufferRef::new(slot);
        let buffer = buf_ref.get_mut();
        buffer.ring_head.store(0, Ordering::Release);
        buffer.ring_tail.store(0, Ordering::Release);
    }

    /// Try to receive one message from a subscriber ring slot.
    /// Replicates `take_serialized` logic for testing without a zenoh session.
    pub(in crate::shim) fn take_subscription(
        slot: usize,
        recv_buf: &mut [u8],
    ) -> Result<Option<usize>, TransportError> {
        let buf_ref = SubscriberBufferRef::new(slot);
        let buffer = buf_ref.get();

        let Some(s) = buffer.peek_head_slot() else {
            return Ok(None);
        };
        let len = buffer.ring_len[s];
        if len > recv_buf.len() {
            buffer.consume_head();
            return Err(TransportError::BufferTooSmall);
        }
        recv_buf[..len].copy_from_slice(buffer.payload_slot(s, len));
        buffer.consume_head();
        Ok(Some(len))
    }

    /// Process subscription data in-place (mirrors `process_raw_in_place` logic).
    fn process_in_place_subscription(
        slot: usize,
    ) -> Result<Option<alloc::vec::Vec<u8>>, TransportError> {
        let buf_ref = SubscriberBufferRef::new(slot);
        let buffer = buf_ref.get();

        let Some(s) = buffer.peek_head_slot() else {
            return Ok(None);
        };
        let len = buffer.ring_len[s];
        let data = buffer.payload_slot(s, len).to_vec();
        buffer.consume_head();
        Ok(Some(data))
    }

    // ========================================================================
    // 37.1a: Subscription buffer state machine tests
    // ========================================================================

    #[test]
    fn sub_buf_idle_poll() {
        let slot = 0;
        reset_subscriber_buffer(slot);

        let mut buf = [0u8; 1024];
        let result = take_subscription(slot, &mut buf);
        assert!(matches!(result, Ok(None)));

        // Ring empty.
        let buffer = SubscriberBufferRef::new(slot).get();
        assert!(!buffer.has_data());
    }

    #[test]
    fn sub_buf_normal_delivery() {
        let slot = 1;
        reset_subscriber_buffer(slot);

        let payload = [0x42u8; 100];
        simulate_subscription_callback(slot, &payload);

        let buffer = SubscriberBufferRef::new(slot).get();
        assert!(buffer.has_data());

        let mut recv_buf = [0u8; 1024];
        let result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(result, Ok(Some(100))));
        assert_eq!(&recv_buf[..100], &payload);

        assert!(!buffer.has_data());
    }

    /// Issue 0841 — every block handed out must be able to hold the hint that
    /// was routed into it.
    ///
    /// The pre-existing routing test probes `SUBSCRIBER_SIZE_THRESHOLD + 1`,
    /// one byte ABOVE the threshold, so it never visits the window where the
    /// two independent knobs disagree — at the shipped defaults, hints of
    /// 1025..=2048 took a 1024-byte block. This asserts the property directly
    /// instead of the threshold's arithmetic, so it stays meaningful whatever a
    /// consumer sets the knobs to.
    #[test]
    fn a_routed_block_always_fits_its_hint() {
        NEXT_SMALL_PAYLOAD.store(0, Ordering::SeqCst);
        NEXT_LARGE_PAYLOAD.store(0, Ordering::SeqCst);

        // One byte past what the small block can hold — inside the old gap
        // whenever the threshold exceeds the block size.
        //
        // Phase 403 W4 — the property is "a block that is handed out fits its
        // hint", and REFUSING is the other way to satisfy it. That is not a
        // weakening: it is the same rule, and it is the only answer available
        // once `MAX_LARGE_SUBSCRIBERS = 0` makes an image with no large class
        // legal. Demanding a block here would have made this test assert the
        // arithmetic it was written to stop asserting.
        let hint = SUBSCRIBER_BUFFER_SIZE + 1;
        if let Some((_b, stride)) = alloc_payload_block(hint) {
            assert!(
                stride >= hint,
                "hint {hint} was routed to a {stride}-byte block: the sample cannot \
                 fit, and the drop happens at the transport where no build assertion \
                 can see it"
            );
        } else {
            assert!(
                hint > LARGEST_PAYLOAD_CLASS,
                "hint {hint} was refused while a {LARGEST_PAYLOAD_CLASS}-byte class \
                 could have held it: refusal is only correct when no class fits"
            );
        }

        // And the boundary itself still takes the cheap class.
        NEXT_SMALL_PAYLOAD.store(0, Ordering::SeqCst);
        NEXT_LARGE_PAYLOAD.store(0, Ordering::SeqCst);
        let (_b, stride) = alloc_payload_block(SMALL_CLASS_CEILING).expect("small alloc");
        assert_eq!(
            stride, SUBSCRIBER_BUFFER_SIZE,
            "a hint exactly at the ceiling must stay in the small class — \
             rounding it up would double every subscriber's cost"
        );
    }

    /// Phase 403 W4 — a hint no size class in this image can hold is REFUSED,
    /// not routed to the biggest class and quietly under-served.
    ///
    /// This is the top end of issue 0841. That issue's fix stopped a hint the
    /// SMALL block could not hold from being small-classed; the same hint one
    /// class up — larger than `SUBSCRIBER_LARGE_SIZE` — still got a large block
    /// too small for it, and the drop happened at the transport where nothing
    /// could see it. Asserted as the property (`stride >= hint` for every block
    /// handed out) rather than as the arithmetic, so it survives any knob
    /// setting, including `MAX_LARGE_SUBSCRIBERS = 0`.
    #[test]
    fn a_hint_no_class_can_hold_is_refused() {
        NEXT_SMALL_PAYLOAD.store(0, Ordering::SeqCst);
        NEXT_LARGE_PAYLOAD.store(0, Ordering::SeqCst);

        assert!(
            alloc_payload_block(LARGEST_PAYLOAD_CLASS + 1).is_none(),
            "a hint of {} bytes exceeds every class ({LARGEST_PAYLOAD_CLASS} is the \
             largest) and must fail create_subscription, not be served a block \
             that cannot hold it",
            LARGEST_PAYLOAD_CLASS + 1
        );

        // The boundary itself is still served, and by a block that fits it.
        NEXT_SMALL_PAYLOAD.store(0, Ordering::SeqCst);
        NEXT_LARGE_PAYLOAD.store(0, Ordering::SeqCst);
        let (_b, stride) = alloc_payload_block(LARGEST_PAYLOAD_CLASS)
            .expect("the largest class serves its own size");
        assert!(stride >= LARGEST_PAYLOAD_CLASS);

        NEXT_SMALL_PAYLOAD.store(0, Ordering::SeqCst);
        NEXT_LARGE_PAYLOAD.store(0, Ordering::SeqCst);
    }

    /// Phase 403 W4 — what this backend answers `rmw_vtable.h`'s
    /// `required_rx_bytes`, and specifically that it answers the MINIMUM
    /// sufficient size rather than the class stride it rounded to.
    ///
    /// Returning the stride would be sufficient and would collapse the whole
    /// point: a 68-byte type and a 1000-byte type would both come back
    /// `SUBSCRIBER_BUFFER_SIZE`, which is the two-arbitrary-constants answer the
    /// runtime already had without asking.
    #[test]
    fn required_rx_bytes_is_minimal_not_the_class_stride() {
        // Two hints that share a size class must NOT share an answer.
        let small = required_rx_bytes(68).expect("a bounded hint is answerable");
        let bigger = required_rx_bytes(1000).expect("a bounded hint is answerable");
        assert_eq!(
            small, 68,
            "the answer is the type's own bound, not the block it lands in"
        );
        assert_eq!(bigger, 1000);
        assert_ne!(
            small, bigger,
            "rounding both up to the class stride is the distinction this slot exists to recover"
        );

        // A take buffer of the answer can always hold what `take` will copy:
        // the block routed to is at least as large as the hint (0841), and
        // nothing is added on top of the payload.
        NEXT_SMALL_PAYLOAD.store(0, Ordering::SeqCst);
        NEXT_LARGE_PAYLOAD.store(0, Ordering::SeqCst);
        for hint in [1usize, 68, SMALL_CLASS_CEILING, LARGEST_PAYLOAD_CLASS] {
            assert_eq!(
                required_rx_bytes(hint),
                Some(hint),
                "a hint every class boundary can serve is its own answer"
            );
        }

        // `0` is the CALLER saying nothing, never "this type is unbounded".
        // zenoh-pico has no schema for a type it knows only by name, so it
        // answers about the SUBSCRIPTION it would create: that hint routes
        // small, and the small stride is the most `take` can ever hand back.
        assert_eq!(
            required_rx_bytes(0),
            Some(SUBSCRIBER_BUFFER_SIZE),
            "hint 0 gets the small class stride — a real ceiling, not a guess"
        );

        // Past every class: no take-buffer size makes this subscription work,
        // which is the per-type inability NULLing the whole slot cannot say.
        assert_eq!(
            required_rx_bytes(LARGEST_PAYLOAD_CLASS + 1),
            None,
            "a hint no class can hold must be UNSUPPORTED, not a number this \
             backend cannot honour"
        );
    }

    /// Phase 403 W4 — `required_rx_bytes` and `alloc_payload_block` agree about
    /// which hints this image can serve.
    ///
    /// The slot's value is that it is answerable BEFORE any entity exists, so
    /// the runtime can ask instead of discovering the answer at
    /// `create_subscription`. That is only true while the two agree; if they
    /// drifted, asking would be worse than not asking.
    #[test]
    fn the_sizing_query_and_the_allocator_agree() {
        for hint in [
            0usize,
            1,
            SMALL_CLASS_CEILING,
            SMALL_CLASS_CEILING + 1,
            LARGEST_PAYLOAD_CLASS,
            LARGEST_PAYLOAD_CLASS + 1,
            LARGEST_PAYLOAD_CLASS * 2,
        ] {
            NEXT_SMALL_PAYLOAD.store(0, Ordering::SeqCst);
            NEXT_LARGE_PAYLOAD.store(0, Ordering::SeqCst);
            let allocated = alloc_payload_block(hint);
            assert_eq!(
                required_rx_bytes(hint).is_some(),
                allocated.is_some(),
                "hint {hint}: the query says {:?} and the allocator says {:?}",
                required_rx_bytes(hint),
                allocated.map(|(_, stride)| stride)
            );
            if let (Some(answer), Some((_, stride))) = (required_rx_bytes(hint), allocated) {
                assert!(
                    stride >= answer,
                    "hint {hint}: answered {answer} bytes but routed to a {stride}-byte block"
                );
            }
        }
        NEXT_SMALL_PAYLOAD.store(0, Ordering::SeqCst);
        NEXT_LARGE_PAYLOAD.store(0, Ordering::SeqCst);
    }

    // Phase 231 Wave 3 (RFC-0038) — size-class routing + exhaustion.
    #[test]
    fn wave3_size_class_routing_and_exhaustion() {
        // The allocators are process-global statics; reset so the assertions are
        // deterministic (no real ZenohSubscriber is created in lib tests).
        NEXT_SMALL_PAYLOAD.store(0, Ordering::SeqCst);
        NEXT_LARGE_PAYLOAD.store(0, Ordering::SeqCst);

        // A small hint routes to the small class (small slot stride).
        let (_b, stride) = alloc_payload_block(64).expect("small alloc");
        assert_eq!(
            stride, SUBSCRIBER_BUFFER_SIZE,
            "hint <= threshold routes to the small class"
        );

        // A hint above the threshold routes to the large class. Phase 403 W4 —
        // guarded, because `MAX_LARGE_SUBSCRIBERS = 0` is now a legal thing for
        // an image to declare, and there the large class does not exist at all.
        // The refusal that replaces it is `a_hint_no_class_can_hold_is_refused`.
        if MAX_LARGE_SUBSCRIBERS > 0 {
            let (_b, stride) =
                alloc_payload_block(SUBSCRIBER_SIZE_THRESHOLD + 1).expect("large alloc");
            assert_eq!(
                stride, SUBSCRIBER_LARGE_SIZE,
                "hint > threshold routes to the large class"
            );
        }

        // The large class exhausts at MAX_LARGE_SUBSCRIBERS, then returns None
        // (the caller surfaces SubscriberCreationFailed rather than over-allocate).
        NEXT_LARGE_PAYLOAD.store(0, Ordering::SeqCst);
        for _ in 0..MAX_LARGE_SUBSCRIBERS {
            assert!(alloc_payload_block(SUBSCRIBER_LARGE_SIZE).is_some());
        }
        assert!(
            alloc_payload_block(SUBSCRIBER_LARGE_SIZE).is_none(),
            "large class is bounded by MAX_LARGE_SUBSCRIBERS"
        );
        // Leave the allocators reset for any sibling test.
        NEXT_SMALL_PAYLOAD.store(0, Ordering::SeqCst);
        NEXT_LARGE_PAYLOAD.store(0, Ordering::SeqCst);
    }

    // Phase 231 Wave 3 — per-subscriber isolation: each subscriber owns its ring
    // (separate meta + payload block), so one subscriber's traffic never touches
    // another's buffer (stronger than the shared-pool depth budget).
    #[test]
    fn wave3_per_sub_isolation() {
        let (a, b) = (5, 6);
        reset_subscriber_buffer(a);
        reset_subscriber_buffer(b);

        simulate_subscription_callback(a, &[0xAAu8; 50]);

        assert!(
            SubscriberBufferRef::new(a).get().has_data(),
            "the pushed-to subscriber has data"
        );
        assert!(
            !SubscriberBufferRef::new(b).get().has_data(),
            "a sibling subscriber is unaffected by the other's traffic"
        );
    }

    #[test]
    fn sub_buf_max_payload() {
        let slot = 2;
        reset_subscriber_buffer(slot);

        // Exactly SUBSCRIBER_BUFFER_SIZE = max slot capacity.
        let payload = [0xFFu8; SUBSCRIBER_BUFFER_SIZE];
        simulate_subscription_callback(slot, &payload);

        let buffer = SubscriberBufferRef::new(slot).get();
        assert!(buffer.has_data());

        let mut recv_buf = [0u8; SUBSCRIBER_BUFFER_SIZE];
        let result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(result, Ok(Some(n)) if n == SUBSCRIBER_BUFFER_SIZE));
        assert_eq!(&recv_buf, &payload);
    }

    #[test]
    fn sub_buf_oversized_dropped_by_producer() {
        let slot = 3;
        reset_subscriber_buffer(slot);

        // Payload larger than a ring slot — the C producer (here the
        // simulate helper) drops it silently without advancing tail.
        let payload = [0xAAu8; SUBSCRIBER_BUFFER_SIZE + 1];
        simulate_subscription_callback(slot, &payload);

        let buffer = SubscriberBufferRef::new(slot).get();
        assert!(!buffer.has_data(), "oversized message must be dropped");

        let mut recv_buf = [0u8; SUBSCRIBER_BUFFER_SIZE];
        let result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(result, Ok(None)));

        // Recovery: next normal callback is accepted.
        simulate_subscription_callback(slot, b"recovered");
        let result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(result, Ok(Some(9))));
        assert_eq!(&recv_buf[..9], b"recovered");
    }

    #[test]
    fn sub_buf_caller_too_small() {
        let slot = 4;
        reset_subscriber_buffer(slot);

        // Store 512 bytes, try to receive into a 256-byte buffer.
        let payload = [0xBBu8; 512];
        simulate_subscription_callback(slot, &payload);

        let mut small_buf = [0u8; 256];
        let result = take_subscription(slot, &mut small_buf);
        assert!(matches!(result, Err(TransportError::BufferTooSmall)));

        // Slot consumed (the message that didn't fit is dropped).
        let buffer = SubscriberBufferRef::new(slot).get();
        assert!(!buffer.has_data());

        // Recovery: next callback accepted.
        simulate_subscription_callback(slot, b"small");
        let mut recv_buf = [0u8; 1024];
        let result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(result, Ok(Some(5))));
        assert_eq!(&recv_buf[..5], b"small");
    }

    #[test]
    fn sub_buf_ring_buffers_burst() {
        // Phase 124.D.3.c — two callbacks without an intervening recv
        // are BOTH buffered (ring), not last-message-wins.
        let slot = 5;
        reset_subscriber_buffer(slot);

        simulate_subscription_callback(slot, b"first_msg");
        simulate_subscription_callback(slot, b"second_msg");

        let mut recv_buf = [0u8; 1024];
        let result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(result, Ok(Some(9))));
        assert_eq!(&recv_buf[..9], b"first_msg");

        let result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(result, Ok(Some(10))));
        assert_eq!(&recv_buf[..10], b"second_msg");

        // Ring drained.
        assert!(matches!(take_subscription(slot, &mut recv_buf), Ok(None)));
    }

    #[test]
    fn sub_buf_ring_full_drops_excess() {
        // Filling the ring past SUBSCRIBER_RING_DEPTH drops the
        // overflow message; the buffered ones still drain in order.
        let slot = 6;
        reset_subscriber_buffer(slot);

        for i in 0..SUBSCRIBER_RING_DEPTH {
            let msg = [i as u8; 4];
            simulate_subscription_callback(slot, &msg);
        }
        // One more — ring is full, this is dropped.
        simulate_subscription_callback(slot, &[0xFFu8; 4]);

        let mut recv_buf = [0u8; 16];
        for i in 0..SUBSCRIBER_RING_DEPTH {
            let result = take_subscription(slot, &mut recv_buf);
            assert!(matches!(result, Ok(Some(4))));
            assert_eq!(&recv_buf[..4], &[i as u8; 4]);
        }
        // The dropped message never appears.
        assert!(matches!(take_subscription(slot, &mut recv_buf), Ok(None)));
    }

    #[test]
    fn sub_buf_double_consume() {
        let slot = 6;
        reset_subscriber_buffer(slot);

        simulate_subscription_callback(slot, b"data");

        let mut recv_buf = [0u8; 1024];
        let result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(result, Ok(Some(4))));

        // Second recv returns None — ring drained.
        let result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(result, Ok(None)));
    }

    #[test]
    fn sub_buf_oversized_then_normal() {
        let slot = 7;
        reset_subscriber_buffer(slot);

        // Oversized → dropped by producer → normal → delivered.
        simulate_subscription_callback(slot, &[0u8; SUBSCRIBER_BUFFER_SIZE + 1]);
        let mut recv_buf = [0u8; 1024];
        let result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(result, Ok(None)));

        simulate_subscription_callback(slot, b"after_oversized");
        let result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(result, Ok(Some(15))));
        assert_eq!(&recv_buf[..15], b"after_oversized");
    }

    #[test]
    fn sub_buf_zero_length_payload_dropped() {
        // Empty probes are dropped by the producer — they never
        // occupy a ring slot.
        let slot = 0;
        reset_subscriber_buffer(slot);

        simulate_subscription_callback(slot, b"");

        let buffer = SubscriberBufferRef::new(slot).get();
        assert!(!buffer.has_data());

        let mut recv_buf = [0u8; 1024];
        let result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(result, Ok(None)));
    }

    #[test]
    fn sub_buf_all_slots_independent() {
        let slot_a = 0;
        let slot_b = 7;
        reset_subscriber_buffer(slot_a);
        reset_subscriber_buffer(slot_b);

        simulate_subscription_callback(slot_a, b"slot_zero");
        simulate_subscription_callback(slot_b, b"slot_seven");

        // Consume slot_b first.
        let mut recv_buf = [0u8; 1024];
        let result = take_subscription(slot_b, &mut recv_buf);
        assert!(matches!(result, Ok(Some(10))));
        assert_eq!(&recv_buf[..10], b"slot_seven");

        // slot_a still has data.
        let buffer_a = SubscriberBufferRef::new(slot_a).get();
        assert!(buffer_a.has_data());

        let result = take_subscription(slot_a, &mut recv_buf);
        assert!(matches!(result, Ok(Some(9))));
        assert_eq!(&recv_buf[..9], b"slot_zero");
    }

    // ========================================================================
    // Phase 124.D.3.c — in-place processing over the ring
    // ========================================================================

    #[test]
    fn sub_buf_in_place_matches_copy() {
        let slot = 0;
        reset_subscriber_buffer(slot);

        // Write 100-byte payload, take (copy path) → capture bytes.
        let payload = [0x42u8; 100];
        simulate_subscription_callback(slot, &payload);

        let mut recv_buf = [0u8; 1024];
        let copy_result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(copy_result, Ok(Some(100))));
        let copy_bytes = recv_buf[..100].to_vec();

        // Reset, write same payload, process_in_place → capture bytes.
        reset_subscriber_buffer(slot);
        simulate_subscription_callback(slot, &payload);

        let in_place_result = process_in_place_subscription(slot);
        assert!(matches!(in_place_result, Ok(Some(_))));
        let in_place_bytes = in_place_result.unwrap().unwrap();

        // Both paths must produce identical data.
        assert_eq!(copy_bytes, in_place_bytes);
    }

    #[test]
    fn sub_buf_in_place_idle() {
        let slot = 1;
        reset_subscriber_buffer(slot);

        // Empty ring → process_in_place returns Ok(None).
        let result = process_in_place_subscription(slot);
        assert!(matches!(result, Ok(None)));
    }

    #[test]
    fn sub_buf_in_place_drains_ring_in_order() {
        // Successive in-place reads drain the ring head-first.
        let slot = 2;
        reset_subscriber_buffer(slot);

        simulate_subscription_callback(slot, b"aaa");
        simulate_subscription_callback(slot, b"bbbb");

        let first = process_in_place_subscription(slot).unwrap().unwrap();
        assert_eq!(&first, b"aaa");
        let second = process_in_place_subscription(slot).unwrap().unwrap();
        assert_eq!(&second, b"bbbb");
        assert!(matches!(process_in_place_subscription(slot), Ok(None)));
    }

    #[test]
    fn sub_buf_consume_advances_head() {
        // Consuming N messages advances ring_head by exactly N.
        let slot = 3;
        reset_subscriber_buffer(slot);

        simulate_subscription_callback(slot, b"one");
        simulate_subscription_callback(slot, b"two");

        let buffer = SubscriberBufferRef::new(slot).get();
        let head_before = buffer.ring_head.load(Ordering::Acquire);
        assert_eq!(head_before, 0);

        let mut recv_buf = [0u8; 16];
        let _ = take_subscription(slot, &mut recv_buf);
        let _ = take_subscription(slot, &mut recv_buf);

        let head_after = buffer.ring_head.load(Ordering::Acquire);
        assert_eq!(head_after, 2);
        assert_eq!(
            buffer.ring_tail.load(Ordering::Acquire),
            head_after,
            "ring drained → head == tail"
        );
    }
}
