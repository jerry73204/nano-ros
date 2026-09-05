//! ZenohServiceServer and ZenohServiceClient implementations

use core::marker::PhantomData;

use atomic_waker::AtomicWaker;
use portable_atomic::{AtomicBool, AtomicUsize, Ordering};

use nros_rmw::{ClientTrait, ServiceInfo, ServiceRequest, ServiceTrait, TransportError};

use super::{
    AtomicSeqCounter, Context, KEYEXPR_BUFFER_SIZE, KEYEXPR_STRING_SIZE, RMW_ATTACHMENT_SIZE,
    RMW_GID_SIZE, RmwAttachment, SERVICE_BUFFER_SIZE, SeqScalar,
};
use crate::{
    keyexpr::ServiceKeyExpr,
    zpico::{
        self, Queryable, ZPICO_MAX_QUERYABLES, ZPICO_MAX_SESSIONS, ZPICO_QUERYABLE_TABLE_DECLARED,
    },
};

#[cfg(feature = "std")]
use super::signal_executor_wake;

// ============================================================================
// ServiceBuffer
// ============================================================================

/// Phase 237 follow-up — depth of the per-server request ring. The single
/// request buffer dropped a request that arrived before the previous one was
/// drained (a burst of queries delivered in one read-task batch — concurrent
/// goals under load). A ring buffers the burst so each request is read in order.
/// Mirrors the subscriber SPSC ring (Phase 124.D.3.c).
pub(super) const SERVICE_REQUEST_RING_DEPTH: usize = 4;

/// One buffered request in the service-server inbox ring.
pub(super) struct ServiceRequestSlot {
    /// Buffer for received request data.
    pub(super) data: [u8; SERVICE_BUFFER_SIZE],
    /// Length of valid data.
    pub(super) len: AtomicUsize,
    /// Reply-correlation token (the C shim's reply-slot index).
    pub(super) seq: AtomicSeqCounter,
    /// Set when the incoming request exceeded `data.len()`.
    pub(super) overflow: AtomicBool,
}

impl ServiceRequestSlot {
    pub(super) const fn new() -> Self {
        Self {
            data: [0u8; SERVICE_BUFFER_SIZE],
            len: AtomicUsize::new(0),
            seq: AtomicSeqCounter::new(0),
            overflow: AtomicBool::new(false),
        }
    }
}

/// Shared buffer for service server callbacks — a single-producer (the queryable
/// callback on the zenoh read task) single-consumer (`take_request` on the
/// executor) ring. `head`/`tail` are monotonic wrapping counters; the slot index
/// is `counter % depth`. `tail - head` is the queued count; full → the callback
/// drops the newest (preserving in-order delivery).
pub(super) struct ServiceBuffer {
    pub(super) ring: [ServiceRequestSlot; SERVICE_REQUEST_RING_DEPTH],
    /// Consumer cursor (written only by `take_request`).
    pub(super) head: AtomicUsize,
    /// Producer cursor (written only by the callback).
    pub(super) tail: AtomicUsize,
    /// Reply keyexpr — constant per server (same rr/ topic for every request),
    /// so a single copy suffices.
    pub(super) keyexpr: [u8; 256],
    /// Length of keyexpr.
    pub(super) keyexpr_len: AtomicUsize,
    /// Phase 122.3.c.6.e — waker registered by event-driven service
    /// servers. Woken by `queryable_callback` after a request lands.
    pub(super) waker: AtomicWaker,
    /// phase-328 (issue 0348) — the owning zpico session pool slot, recorded
    /// at server-registration time. `queryable_callback` reads it back so
    /// `zpico_queryable_take_reply_seq(session, …)` addresses the correct
    /// session's reply-slot table (this buffer array is process-global, so the
    /// handle cannot be recovered from the buffer index alone).
    pub(super) session: core::sync::atomic::AtomicPtr<zpico_sys::zpico_session_t>,
}

impl ServiceBuffer {
    pub(super) const fn new() -> Self {
        Self {
            ring: [const { ServiceRequestSlot::new() }; SERVICE_REQUEST_RING_DEPTH],
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            keyexpr: [0u8; 256],
            keyexpr_len: AtomicUsize::new(0),
            waker: AtomicWaker::new(),
            session: core::sync::atomic::AtomicPtr::new(core::ptr::null_mut()),
        }
    }
}

/// Static buffers for service servers.
///
/// phase-328 / issue 0376 — sized `ZPICO_MAX_SESSIONS * ZPICO_MAX_QUERYABLES`
/// and indexed by `session_index * ZPICO_MAX_QUERYABLES + local`, so two zenoh
/// sessions in one process get disjoint buffer ranges (the C shim's queryable
/// tables are already per-session). At the default `ZPICO_MAX_SESSIONS == 1`
/// this is `[ServiceBuffer; ZPICO_MAX_QUERYABLES]` with `session_index == 0`,
/// identical to the pre-0376 layout.
const SERVICE_BUFFER_COUNT: usize = ZPICO_MAX_SESSIONS * ZPICO_MAX_QUERYABLES;
static mut SERVICE_BUFFERS: [ServiceBuffer; SERVICE_BUFFER_COUNT] =
    [const { ServiceBuffer::new() }; SERVICE_BUFFER_COUNT];

/// Next available LOCAL service-buffer index, per session pool slot. The global
/// index handed to the callback is `session_index * ZPICO_MAX_QUERYABLES + local`.
static NEXT_SERVICE_BUFFER_INDEX: [AtomicUsize; ZPICO_MAX_SESSIONS] =
    [const { AtomicUsize::new(0) }; ZPICO_MAX_SESSIONS];

// ============================================================================
// ServiceBufferRef — safe accessor wrapper
// ============================================================================

/// Safe accessor for a statically-allocated service buffer.
///
/// Encapsulates the `unsafe` access to `SERVICE_BUFFERS` by validating
/// the index once at construction time. Subsequent accesses via [`get()`]
/// are safe because the index is guaranteed in-bounds.
///
/// # Safety invariant
///
/// `SERVICE_BUFFERS` is a module-level `static mut` with a fixed address
/// and element count equal to `SERVICE_BUFFER_COUNT`. The index is validated
/// at construction and never changes, so every `get()` / `get_mut()` call
/// dereferences a valid, in-bounds element.
pub(super) struct ServiceBufferRef {
    index: usize,
}

impl ServiceBufferRef {
    /// Create a new buffer reference with bounds validation.
    ///
    /// # Panics
    ///
    /// Panics if `index >= SERVICE_BUFFER_COUNT`.
    pub(super) fn new(index: usize) -> Self {
        assert!(
            index < SERVICE_BUFFER_COUNT,
            "service buffer index out of bounds: {index} >= {SERVICE_BUFFER_COUNT}"
        );
        Self { index }
    }

    /// Get an immutable reference to the service buffer.
    ///
    /// Safety is guaranteed by the bounds check at construction time.
    /// All shared fields use atomic types, preventing data races.
    pub(super) fn get(&self) -> &ServiceBuffer {
        // Safety: index was validated at construction time.
        // SERVICE_BUFFERS is a module-level static with fixed address.
        unsafe { &SERVICE_BUFFERS[self.index] }
    }

    /// Get a mutable reference to the service buffer.
    ///
    /// Only called from callbacks, which are invoked synchronously
    /// (single-threaded) by zenoh-pico — no concurrent mutable access.
    pub(super) fn get_mut(&mut self) -> &mut ServiceBuffer {
        // Safety: index was validated at construction time.
        // Mutable access is only used by callbacks invoked synchronously
        // by zenoh-pico, so there are no concurrent mutable accesses.
        unsafe { &mut SERVICE_BUFFERS[self.index] }
    }
}

/// Sequence counter for service requests
// Phase 237 — the production queryable callback now records the C shim's
// reply-slot index as the correlation token (seq); only the `#[cfg(test)]`
// service-buffer simulator still hands out monotonic counter values.
#[allow(dead_code)]
pub(super) static SERVICE_SEQ_COUNTER: AtomicSeqCounter = AtomicSeqCounter::new(0);

/// Callback function invoked by the C shim when queries arrive
// `c_char` is `u8` on ARM/aarch64 and `i8` on x86 — so `ptr as *const u8` is a
// no-op on one and a real reinterpret on the other, and `clippy::unnecessary_cast`
// fires under `-D warnings` on ARM hosts only. Repo-wide idiom is `.cast::<u8>()`,
// which compiles identically on both and is never linted; never an `as` cast plus
// an `#[allow]`, which only silences the site it is written on.
extern "C" fn queryable_callback(
    keyexpr: *const core::ffi::c_char,
    keyexpr_len: usize,
    payload: *const u8,
    payload_len: usize,
    ctx: *mut core::ffi::c_void,
) {
    // phase-328/#376 — `ctx` is the GLOBAL buffer index
    // (session_index * ZPICO_MAX_QUERYABLES + local), set at server registration.
    let buffer_index = ctx as usize;
    if buffer_index >= SERVICE_BUFFER_COUNT {
        return;
    }

    let mut buf_ref = ServiceBufferRef {
        index: buffer_index,
    };
    let buffer = buf_ref.get_mut();

    // Copy keyexpr
    let keyexpr_copy_len = keyexpr_len.min(buffer.keyexpr.len() - 1);
    // Safety: keyexpr pointer is valid for keyexpr_copy_len bytes (from C shim)
    unsafe {
        core::ptr::copy_nonoverlapping(
            keyexpr.cast::<u8>(),
            buffer.keyexpr.as_mut_ptr(),
            keyexpr_copy_len,
        );
    }
    buffer.keyexpr[keyexpr_copy_len] = 0; // Null terminate
    buffer
        .keyexpr_len
        .store(keyexpr_copy_len, Ordering::Release);

    // Drop empty-payload queries — they come from background discovery /
    // liveliness probes that zenoh-pico delivers through the same
    // queryable callback as real service requests. Flagging them as
    // `has_request` consumes the slot before the actual CDR-prefixed
    // request lands; the deserializer then trips on the empty buffer
    // and `handle_request` reports `ServiceReplyFailed`.
    if payload.is_null() || payload_len == 0 {
        return;
    }

    // Phase 237 follow-up — enqueue into the request ring. Drop the newest when
    // full (preserves in-order delivery of buffered requests), so a burst of
    // concurrent arrivals doesn't clobber an unread request.
    let head = buffer.head.load(Ordering::Acquire);
    let tail = buffer.tail.load(Ordering::Relaxed);
    if tail.wrapping_sub(head) >= SERVICE_REQUEST_RING_DEPTH {
        return;
    }
    let slot = &mut buffer.ring[tail % SERVICE_REQUEST_RING_DEPTH];

    // Phase 237 — the reply correlation token is the C shim's reply-slot index
    // (the cloned query held for a possibly-deferred reply), not a free-running
    // counter. `buffer_index` is the queryable handle.
    // FFI returns i64; narrow to the counter's native width (i32 on 32-bit
    // targets, where AtomicSeqCounter is AtomicI32). Reply-slot indices are
    // small and fit. Symmetric with the `.into()` widening on load.
    let session = buffer.session.load(Ordering::Acquire);
    let seq = unsafe { zpico_sys::zpico_queryable_take_reply_seq(session, buffer_index as i32) };
    slot.seq.store(seq as SeqScalar, Ordering::Relaxed);

    if payload_len > slot.data.len() {
        // Request exceeds static slot capacity — flag overflow, skip payload.
        slot.overflow.store(true, Ordering::Relaxed);
        slot.len.store(0, Ordering::Relaxed);
    } else {
        slot.overflow.store(false, Ordering::Relaxed);
        // Safety: payload pointer is valid for payload_len bytes (from C shim).
        unsafe {
            core::ptr::copy_nonoverlapping(payload, slot.data.as_mut_ptr(), payload_len);
        }
        slot.len.store(payload_len, Ordering::Relaxed);
    }

    // Publish the slot: the Release pairs with the consumer's Acquire load of
    // `tail`, so the data/len/seq writes above are visible before the request.
    buffer.tail.store(tail.wrapping_add(1), Ordering::Release);

    // Phase 122.3.c.6.e — wake any task that registered a Waker on
    // this server (event-driven callers).
    buffer.waker.wake();

    // Wake the executor spin loop (if waiting)
    #[cfg(feature = "std")]
    signal_executor_wake();
}

// ============================================================================
// ZenohServiceServer
// ============================================================================

/// Zenoh service server using queryables
///
/// Receives service requests via queryable callbacks.
/// Note: The reply mechanism is limited due to the callback model.
pub struct ZenohServiceServer {
    /// The queryable handle (kept alive to maintain registration)
    _queryable: Queryable,
    /// Safe accessor for the static service buffer
    buf: ServiceBufferRef,
    /// Liveliness token for ROS 2 graph discovery (kept alive for server lifetime)
    _liveliness: Option<super::LivelinessToken>,
    /// Keyexpr buffer for replying (copied from last request)
    reply_keyexpr: [u8; 256],
    /// Keyexpr length
    reply_keyexpr_len: usize,
    /// Reference to context for replying
    context: *const Context,
    /// Phantom to indicate ownership
    _phantom: PhantomData<()>,
}

impl ZenohServiceServer {
    /// Create a new service server for the given service
    pub fn new(
        context: &Context,
        service: &ServiceInfo,
        liveliness: Option<super::LivelinessToken>,
    ) -> Result<Self, TransportError> {
        // phase-328/#376 — allocate a per-session LOCAL buffer index and map it
        // to a global `SERVICE_BUFFERS` slot, so two sessions' servers never
        // share a slot. `session_index` is the C shim's session pool slot.
        let session_index = unsafe { zpico_sys::zpico_session_index(context.handle()) };
        if session_index < 0 || (session_index as usize) >= ZPICO_MAX_SESSIONS {
            return Err(TransportError::ServiceServerCreationFailed);
        }
        let session_index = session_index as usize;
        let local = NEXT_SERVICE_BUFFER_INDEX[session_index].fetch_add(1, Ordering::SeqCst);
        if local >= ZPICO_MAX_QUERYABLES {
            NEXT_SERVICE_BUFFER_INDEX[session_index].fetch_sub(1, Ordering::SeqCst);
            // issue 0406 — this table is the reason, and the bare
            // `ServiceServerCreationFailed` never said so. A service server IS a
            // queryable, and the runtime registers its own before the
            // application declares anything, so an entry enabling the parameter
            // and lifecycle services overflowed an 8-slot table AT BOOT and
            // reported only "creation failed" — which reads as a transport or
            // naming fault, not a capacity limit. Name the knob.
            //
            // issue 0827 — this message used to quote those counts ("param
            // services use 6 and lifecycle services use 6"). It was wrong
            // (lifecycle is 5) and it could not be otherwise: this crate does
            // not depend on `nros-node` and cannot see either the counts or
            // whether their features are even compiled in. A number stated
            // where it cannot be derived is a number that drifts, and this was
            // one of six such spellings. The counts now live beside the code
            // that creates them, as `PARAM_SERVICE_QUERYABLES` and
            // `LIFECYCLE_SERVICE_QUERYABLES`; the message names the knob and
            // the cause, which is all this layer actually knows.
            // phase-392 W5.e — the same exhaustion, TWO different faults, and
            // the message has to say which. Sized from the backend's own
            // budget, the table is simply too small and the fix is the knob.
            // Sized from the entry's DECLARATION (`ZPICO_QUERYABLE_TABLE_DECLARED`),
            // it holds exactly what the model said this image would create, so
            // reaching this point means the image created a service server the
            // model does not declare — and pointing that reader at the knob
            // sends them to enlarge a table that was already right.
            //
            // Being authoritative costs precisely this: "the declaration is
            // wrong" and "the table is too small" become the same event, so the
            // message must name the first (phase-392 W5.b2).
            #[cfg(feature = "std")]
            if ZPICO_QUERYABLE_TABLE_DECLARED {
                log::error!(
                    "service server rejected: this image created an UNDECLARED service \
                     server. Its queryable table holds {} slot(s) for session {}, sized \
                     from what the resolved SystemModel declares — so the table is not \
                     too small, the declaration is incomplete. Declare the service \
                     server in the launch file (a service server is a queryable, and an \
                     action server is three). Raising ZPICO_MAX_QUERYABLES also works \
                     and leaves the model disagreeing with the image.",
                    ZPICO_MAX_QUERYABLES,
                    session_index,
                );
            } else {
                log::error!(
                    "service server rejected: ZPICO_MAX_QUERYABLES={} exhausted for \
                     session {} (a service server is a queryable, and the ROS \
                     parameter and REP-2002 lifecycle services claim theirs before \
                     the application declares anything). Raise ZPICO_MAX_QUERYABLES \
                     and rebuild.",
                    ZPICO_MAX_QUERYABLES,
                    session_index,
                );
            }
            // issue 0460 — the `log::error!` above is the ONLY place that named
            // the knob, and it is `cfg(feature = "std")`: on every embedded
            // image the caller got a bare `ServiceServerCreationFailed` and no
            // explanation. `Backend` carries a `&'static str` through `no_std`
            // with no logger and no allocator, and the capability seam in
            // `nros` prints it verbatim — which is how the three zephyr
            // `workspaces/features` entries finally named their own failure
            // instead of dying quietly after "Network ready".
            // issue 0827 — this string used to quote the counts (6 and 5, and
            // 11 for the pair). Correct at the time, and still the wrong place
            // for them: this crate cannot see `nros-node`'s constants or
            // whether their features are compiled in, so the numbers could only
            // ever be copies. Seven copies existed; two had drifted to the
            // wrong value. Say what this layer knows — the table, the knob, and
            // that the runtime claims slots first.
            // The `no_std` half of the same split. `Backend` carries a
            // `&'static str`, so both spellings are compile-time constants and
            // the branch costs nothing at runtime.
            return Err(TransportError::Backend(if ZPICO_QUERYABLE_TABLE_DECLARED {
                "undeclared service server — the zenoh queryable table was sized from \
                 what this entry's SystemModel declares, and it is full. The table is \
                 not too small; the declaration is incomplete. Declare the service \
                 server in the launch file (an action server declares three)."
            } else {
                "zenoh queryable table exhausted — raise CONFIG_NROS_MAX_QUERYABLES \
                 (env ZPICO_MAX_QUERYABLES). A service server IS a queryable, and the \
                 ROS parameter and REP-2002 lifecycle services claim theirs before an \
                 entry's own callbacks."
            }));
        }
        let buffer_index = session_index * ZPICO_MAX_QUERYABLES + local;

        // Generate the service key
        let key: heapless::String<KEYEXPR_STRING_SIZE> = service.to_key();

        // Create null-terminated keyexpr
        let mut keyexpr_buf = [0u8; KEYEXPR_BUFFER_SIZE];
        let bytes = key.as_bytes();
        if bytes.len() >= keyexpr_buf.len() {
            return Err(TransportError::TopicNameInvalid);
        }
        keyexpr_buf[..bytes.len()].copy_from_slice(bytes);
        keyexpr_buf[bytes.len()] = 0;

        // phase-328 — record the owning session BEFORE declaring, so a query
        // that arrives during declaration finds the right pool slot.
        ServiceBufferRef::new(buffer_index)
            .get()
            .session
            .store(context.handle(), Ordering::Release);

        // Create queryable with callback
        let queryable = unsafe {
            context.declare_queryable_raw(
                &keyexpr_buf,
                queryable_callback,
                buffer_index as *mut core::ffi::c_void,
            )
        }
        .map_err(|e| {
            NEXT_SERVICE_BUFFER_INDEX[session_index].fetch_sub(1, Ordering::SeqCst);
            TransportError::from(e)
        })?;

        Ok(Self {
            _queryable: queryable,
            buf: ServiceBufferRef::new(buffer_index),
            _liveliness: liveliness,
            reply_keyexpr: [0u8; 256],
            reply_keyexpr_len: 0,
            context: context as *const Context,
            _phantom: PhantomData,
        })
    }

    pub(super) fn set_liveliness(&mut self, liveliness: Option<super::LivelinessToken>) {
        self._liveliness = liveliness;
    }
}

impl ServiceTrait for ZenohServiceServer {
    type Error = TransportError;

    fn has_request(&self) -> bool {
        let b = self.buf.get();
        b.head.load(Ordering::Relaxed) != b.tail.load(Ordering::Acquire)
    }

    fn register_waker(&self, waker: &core::task::Waker) {
        self.buf.get().waker.register(waker);
    }

    fn take_request<'a>(
        &mut self,
        buf: &'a mut [u8],
    ) -> Result<Option<ServiceRequest<'a>>, Self::Error> {
        let buffer = self.buf.get();

        // Phase 237 follow-up — dequeue the head of the request ring. The Acquire
        // load of `tail` pairs with the callback's Release store, making the
        // slot's data/len/seq visible before we read them.
        let head = buffer.head.load(Ordering::Relaxed);
        let tail = buffer.tail.load(Ordering::Acquire);
        if head == tail {
            return Ok(None);
        }
        let slot = &buffer.ring[head % SERVICE_REQUEST_RING_DEPTH];

        // Advance past the head entry (drop it).
        let pop = || buffer.head.store(head.wrapping_add(1), Ordering::Release);

        if slot.overflow.load(Ordering::Acquire) {
            pop();
            return Err(TransportError::MessageTooLarge);
        }

        let len = slot.len.load(Ordering::Acquire);
        if len > buf.len() {
            // Oversized request dropped; the service recovers on the next one.
            pop();
            return Err(TransportError::BufferTooSmall);
        }

        // Copy data + keyexpr under FFI guard so the callback can't write a slot
        // mid-read (the ring keeps producer/consumer on different slots, but the
        // shared keyexpr is copied here too).
        zpico::ffi_guard(|| {
            // Safety: slot data + keyexpr are valid up to their respective lengths.
            unsafe {
                core::ptr::copy_nonoverlapping(slot.data.as_ptr(), buf.as_mut_ptr(), len);

                // Save keyexpr for potential reply (constant per server).
                let keyexpr_len = buffer.keyexpr_len.load(Ordering::Acquire);
                core::ptr::copy_nonoverlapping(
                    buffer.keyexpr.as_ptr(),
                    self.reply_keyexpr.as_mut_ptr(),
                    keyexpr_len,
                );
                self.reply_keyexpr[keyexpr_len] = 0;
                self.reply_keyexpr_len = keyexpr_len;
            }
        });

        #[allow(clippy::useless_conversion)] // i32→i64 on embedded, no-op on std
        let seq: i64 = slot.seq.load(Ordering::Acquire).into();
        pop();

        Ok(Some(ServiceRequest {
            data: &buf[..len],
            sequence_number: seq,
        }))
    }

    fn send_response(&mut self, sequence_number: i64, data: &[u8]) -> Result<(), Self::Error> {
        if self.reply_keyexpr_len == 0 {
            return Err(TransportError::ServiceReplyFailed);
        }

        // Get context reference
        let context = unsafe { &*self.context };

        // Phase 237 — `sequence_number` selects the cloned query the C shim is
        // holding for this request (the reply-slot index from `take_request`),
        // so a deferred get_result reply reaches the original requester even
        // after later requests arrived. The reply keyexpr is constant per server
        // (same rr/ topic), so it is NOT cleared — subsequent deferred replies
        // reuse it; it is re-set on every `take_request` regardless.
        context
            .query_reply(
                self._queryable.handle(),
                sequence_number,
                &self.reply_keyexpr[..=self.reply_keyexpr_len],
                data,
                None,
            )
            .map_err(|_| TransportError::ServiceReplyFailed)?;

        Ok(())
    }
}

// ============================================================================
// Reply Wakers (for async service client)
// ============================================================================

use crate::zpico::ZPICO_MAX_PENDING_GETS;

/// One AtomicWaker per (session, pending-get slot). phase-328 / issue 0376 —
/// sized `ZPICO_MAX_SESSIONS * ZPICO_MAX_PENDING_GETS` and indexed by
/// `session_index * ZPICO_MAX_PENDING_GETS + slot`, so a reply on session A's
/// slot N wakes A's future, not session B's future parked on the same C slot
/// index. At the default `ZPICO_MAX_SESSIONS == 1` this is unchanged.
/// Registered by `Promise::poll()`, woken from the C shim when a reply arrives
/// or the channel closes.
const REPLY_WAKER_COUNT: usize = ZPICO_MAX_SESSIONS * ZPICO_MAX_PENDING_GETS;
static REPLY_WAKERS: [AtomicWaker; REPLY_WAKER_COUNT] =
    [const { AtomicWaker::new() }; REPLY_WAKER_COUNT];

/// C callback invoked by zpico.c when a pending get slot gets a reply.
///
/// # Safety
///
/// Called from C (pending_get_reply_handler / pending_get_dropper) with the
/// owning session's pool index and the per-session slot (issue 0376).
/// `slot` must be in [0, ZPICO_MAX_PENDING_GETS); `session_index` in
/// [0, ZPICO_MAX_SESSIONS).
unsafe extern "C" fn reply_waker_callback(session_index: i32, slot: i32) {
    if session_index >= 0
        && (session_index as usize) < ZPICO_MAX_SESSIONS
        && slot >= 0
        && (slot as usize) < ZPICO_MAX_PENDING_GETS
    {
        let idx = session_index as usize * ZPICO_MAX_PENDING_GETS + slot as usize;
        REPLY_WAKERS[idx].wake();
    }
}

/// Register the reply waker callback with the C shim.
///
/// Called once during session initialization.
pub(super) fn register_reply_waker(session: *mut zpico_sys::zpico_session_t) {
    unsafe {
        zpico_sys::zpico_set_reply_waker(session, Some(reply_waker_callback));
    }
}

// ============================================================================
// Service Client
// ============================================================================

// SERVICE_DEFAULT_TIMEOUT_MS is generated by build.rs from the
// NROS_SERVICE_TIMEOUT_MS env var (default 30000).
use crate::config::SERVICE_DEFAULT_TIMEOUT_MS;

/// Zenoh service client using z_get queries
///
/// Service clients send requests via z_get and receive responses from queryables.
pub struct ZenohServiceClient {
    /// Service key expression (null-terminated)
    keyexpr: [u8; 257],
    /// Length of valid keyexpr
    keyexpr_len: usize,
    /// Wildcard liveliness keyexpr matching any service-server token for
    /// this service (null-terminated). Used by `start_server_discovery`.
    discovery_keyexpr: [u8; 257],
    /// Length of valid `discovery_keyexpr`.
    discovery_keyexpr_len: usize,
    /// Latched result of the most recent `start_server_discovery`/poll
    /// pair. Set to `Some(true)` once the first liveliness reply arrives
    /// so subsequent `is_server_ready` calls can answer without a round
    /// trip. Reset to `None` when discovery hasn't been started.
    server_seen: bool,
    /// Slot handle of an in-flight liveliness query (None if idle).
    discovery_handle: Option<i32>,
    /// Liveliness token for ROS 2 graph discovery (kept alive for client lifetime)
    _liveliness: Option<super::LivelinessToken>,
    /// Reference to context for making queries
    context: *const Context,
    /// phase-328/#376 — the owning session's pool index, cached at construction.
    /// Scopes this client's `REPLY_WAKERS` registrations so a reply on another
    /// session's same-numbered slot cannot wake this client's future.
    session_index: usize,
    /// Timeout in milliseconds
    timeout_ms: u32,
    /// Handles for outstanding non-blocking get operations.
    ///
    /// Was `Option<i32>` (single handle). The C-API blocking
    /// `nros_client_call` resends the request every ~500 ms during a
    /// discovery race (Phase 89.12 cold-boot fix), each resend calling
    /// `send_request_raw` → `zpico_get_start` → fresh slot. Storing only
    /// the latest handle dropped the older slots: when the server's
    /// reply finally arrived on slot N (older than the current handle),
    /// `pending_get_reply_handler` set `received=true` on slot N but
    /// nothing polled it. The slot eventually had its dropper fire on
    /// zenoh-pico's query timeout (`Z_CONFIG_SOCKET_TIMEOUT`, 5 s on
    /// Zephyr), so `zpico_get_check` never returned the data to the
    /// caller. Tracking ALL outstanding handles + polling each in
    /// `take_response_raw` returns the first reply that lands,
    /// regardless of which generation of resend produced it.
    /// Capacity matches the C-side slot pool so we can never lose a
    /// handle the C allocator successfully returned.
    /// Issue 0778 — each outstanding get, PAIRED WITH THE SEQUENCE ID of the
    /// request that produced it. It was a bare `Vec<i32>` of handles, which is
    /// why the reply poll below could only take "first reply wins": with no id
    /// on either side there was nothing to tell a retry generation of the
    /// current request from a reply to a DIFFERENT one.
    pending_handles: heapless::Vec<(i32, i64), ZPICO_MAX_PENDING_GETS>,
    /// Issue 0153 — client GID for the rmw request attachment. rmw_zenoh_cpp
    /// service servers REQUIRE the (sequence_number, source_timestamp, gid)
    /// attachment on the query — `service_take_request` errors without it and
    /// the ROS 2 server never replies (nano↔nano tolerates its absence,
    /// which kept this invisible in-tree).
    rmw_gid: [u8; RMW_GID_SIZE],
    /// Issue 0153 — per-client request sequence counter for the attachment.
    request_seq: AtomicSeqCounter,
    /// Phantom to indicate ownership
    _phantom: PhantomData<()>,
}

/// Issue 0153 — platform clock in ms → attachment source_timestamp.
/// Mirrors `publisher.rs`'s `now_ms` (canonical `nros_platform_*` C symbol);
/// falls back to the sequence number when no real clock exists, preserving
/// monotonicity like the publisher path.
fn current_timestamp_ms(fallback_seq: i64) -> i64 {
    unsafe extern "C" {
        fn nros_platform_time_now_ns() -> u64;
    }
    // Issue 0532 item 5 — the ABI is nanoseconds now; this caller wants ms.
    let ms = unsafe { nros_platform_time_now_ns() / 1_000_000 };
    if ms == 0 { fallback_seq } else { ms as i64 }
}

impl ZenohServiceClient {
    /// Create a new service client for the given service
    pub fn new(
        context: &Context,
        service: &ServiceInfo,
        liveliness: Option<super::LivelinessToken>,
    ) -> Result<Self, TransportError> {
        // Generate wildcard service key for queries (matches any type hash from ROS 2).
        let key: heapless::String<KEYEXPR_STRING_SIZE> = service.to_key_wildcard();

        // Create null-terminated keyexpr
        let mut keyexpr_buf = [0u8; KEYEXPR_BUFFER_SIZE];
        let bytes = key.as_bytes();
        if bytes.len() >= keyexpr_buf.len() {
            return Err(TransportError::TopicNameInvalid);
        }
        keyexpr_buf[..bytes.len()].copy_from_slice(bytes);
        keyexpr_buf[bytes.len()] = 0;

        // Build the wildcard liveliness keyexpr we'll query in
        // `start_server_discovery`. Null-terminate for the C shim.
        let liv: heapless::String<KEYEXPR_STRING_SIZE> =
            super::Ros2Liveliness::service_server_keyexpr_wildcard(service.domain_id, service);
        let mut discovery_buf = [0u8; KEYEXPR_BUFFER_SIZE];
        let liv_bytes = liv.as_bytes();
        if liv_bytes.len() >= discovery_buf.len() {
            return Err(TransportError::TopicNameInvalid);
        }
        discovery_buf[..liv_bytes.len()].copy_from_slice(liv_bytes);
        discovery_buf[liv_bytes.len()] = 0;

        #[cfg(feature = "std")]
        log::debug!("Service client keyexpr: {}", key.as_str());

        // phase-328/#376 — cache the owning session's pool slot for
        // session-scoped REPLY_WAKERS indexing.
        let session_index = unsafe { zpico_sys::zpico_session_index(context.handle()) };
        if session_index < 0 || (session_index as usize) >= ZPICO_MAX_SESSIONS {
            return Err(TransportError::ServiceClientCreationFailed);
        }

        Ok(Self {
            keyexpr: keyexpr_buf,
            keyexpr_len: bytes.len(),
            discovery_keyexpr: discovery_buf,
            discovery_keyexpr_len: liv_bytes.len(),
            server_seen: false,
            discovery_handle: None,
            _liveliness: liveliness,
            context: context as *const Context,
            session_index: session_index as usize,
            timeout_ms: SERVICE_DEFAULT_TIMEOUT_MS,
            pending_handles: heapless::Vec::new(),
            rmw_gid: RmwAttachment::generate_gid(),
            request_seq: AtomicSeqCounter::new(0),
            _phantom: PhantomData,
        })
    }

    /// Set the timeout for service calls
    pub fn set_timeout(&mut self, timeout_ms: u32) {
        self.timeout_ms = timeout_ms;
    }

    /// Append a newly-allocated slot handle to the outstanding list.
    ///
    /// When the list is full we drop the OLDEST handle, not the new
    /// one — the C side has handed us a real slot and refusing to
    /// remember it would lose its reply. The dropped handle's reply
    /// (if any) is forfeited; that slot is recycled by the C
    /// allocator once its dropper fires (zenoh-pico query timeout).
    /// In practice this only triggers when `nros_client_call`'s
    /// resend loop produces more than `ZPICO_MAX_PENDING_GETS`
    /// generations in a single user-visible call — unusual.
    fn track_outstanding(&mut self, handle: i32, seq: i64) {
        if self.pending_handles.is_full() {
            self.pending_handles.remove(0);
        }
        // Cannot fail — we just made room above.
        let _ = self.pending_handles.push((handle, seq));
    }
}

impl ClientTrait for ZenohServiceClient {
    type Error = TransportError;

    fn register_waker(&self, waker: &core::task::Waker) {
        // Wake on any outstanding handle — `nros_client_call`'s resend
        // can leave several gens in flight; any of them could complete
        // first (see `pending_handles` docs).
        for &(handle, _seq) in &self.pending_handles {
            if (handle as usize) < ZPICO_MAX_PENDING_GETS {
                // phase-328/#376 — session-scoped slot (matches reply_waker_callback).
                let idx = self.session_index * ZPICO_MAX_PENDING_GETS + handle as usize;
                REPLY_WAKERS[idx].register(waker);
            }
        }
    }

    fn send_request_raw(&mut self, request: &[u8]) -> Result<i64, Self::Error> {
        let context = unsafe { &*self.context };

        // Issue 0153 — rmw request attachment, same 33-byte layout as the
        // publisher path ([seq le][ts le][gid len][gid]). Built once; the
        // retry loop below re-sends the SAME logical request, so it keeps
        // one sequence number.
        #[allow(clippy::useless_conversion)] // i32→i64 on embedded, no-op on std
        let seq: i64 = (self.request_seq.fetch_add(1, Ordering::Relaxed) + 1).into();
        let ts = current_timestamp_ms(seq);
        let mut attachment = [0u8; RMW_ATTACHMENT_SIZE];
        attachment[0..8].copy_from_slice(&seq.to_le_bytes());
        attachment[8..16].copy_from_slice(&ts.to_le_bytes());
        attachment[16] = RMW_GID_SIZE as u8;
        attachment[17..33].copy_from_slice(&self.rmw_gid);

        // Phase 89.12 #14 + Phase 89.13 flake fix: retry `zpico_get_start`
        // with a bounded wall-clock budget to cover two distinct race
        // classes on multi-threaded zpico backends (POSIX / Zephyr /
        // NuttX / FreeRTOS+lwIP):
        //
        // 1. **Dropper-pending race** (tens of microseconds). A z_get
        //    issued while zenoh-pico is mid-finalization of a *previous*
        //    query — the dropper callback for the prior get_check is
        //    enqueued but hasn't run yet — can be transiently rejected
        //    by the session's pending-query table. Typical surface:
        //        let (_, mut p) = client.send_goal(&g)?;
        //        ... p.take() sees the accept reply ...
        //        let r = client.get_result(&id)?;  // flaked here
        //    Resolves within a few μs once the scheduler runs the
        //    lease / read tasks.
        //
        // 2. **Cold-boot discovery race** (hundreds of milliseconds).
        //    On NuttX QEMU cold start, the Rust client boots in
        //    parallel with the server (the test harness can't delay
        //    the in-guest client, and the pubsub shape already
        //    requires parallel launch). The first `call()` can fire
        //    before zenoh-pico has discovered the server's queryable
        //    via router gossip. 3 tight retries all hit the same
        //    unresolved state within microseconds — the test saw
        //    `Application error: ServiceRequestFailed` as the first
        //    call on NuttX Rust service / action E2E.
        //
        // 800 ms total budget on std covers both cases comfortably
        // (cold-boot discovery empirically lands in 200–600 ms on
        // QEMU NuttX). Between attempts, yield ~5 ms via
        // `thread::sleep` so zenoh-pico's background pthread(s) can
        // advance the session state — spin-looping here starves the
        // lease / read task on single-core QEMU hosts. On no_std
        // fallback we keep the original tight 3-retry count: bare
        // metal / single-threaded zpico has no parallel progress to
        // wait on, and the dropper-pending race there is the only
        // reproducible failure mode.
        // rustc warns "value assigned to `last_err` is never read" because
        // only the *last* assignment in the loop is observable, and the
        // happy path exits via `return Ok(())`. Suppress — the value IS
        // read on the timeout/exhaustion fallthrough at the bottom.
        #[allow(unused_assignments)]
        let mut last_err = None;
        #[cfg(feature = "std")]
        {
            let deadline = std::time::Instant::now() + std::time::Duration::from_millis(800);
            loop {
                match context.get_start_with_attachment(
                    &self.keyexpr[..=self.keyexpr_len],
                    request,
                    &attachment,
                    self.timeout_ms,
                ) {
                    Ok(handle) => {
                        self.track_outstanding(handle, seq);
                        return Ok(seq);
                    }
                    Err(e) => last_err = Some(e),
                }
                if std::time::Instant::now() >= deadline {
                    break;
                }
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
        }
        #[cfg(not(feature = "std"))]
        {
            // 80 × 5 ms = 400 ms budget. Covers cold-boot discovery on
            // multi-threaded zpico backends (FreeRTOS+lwIP, ThreadX+NetX)
            // where the lease / read task needs scheduler quanta to
            // advance the session state past pending-query / queryable
            // gossip. `z_sleep_ms` yields cooperatively on those
            // backends; on bare-metal single-threaded zpico it's a
            // busy-loop fallback but the count keeps it bounded.
            #[cfg(not(feature = "platform-threadx"))]
            unsafe extern "C" {
                fn z_sleep_ms(time: usize) -> i8;
            }
            const MAX_ATTEMPTS: u32 = 80;
            const SLEEP_MS: usize = 5;
            for attempt in 0..MAX_ATTEMPTS {
                match context.get_start_with_attachment(
                    &self.keyexpr[..=self.keyexpr_len],
                    request,
                    &attachment,
                    self.timeout_ms,
                ) {
                    Ok(handle) => {
                        self.track_outstanding(handle, seq);
                        return Ok(seq);
                    }
                    Err(e) => last_err = Some(e),
                }
                if attempt + 1 < MAX_ATTEMPTS {
                    #[cfg(feature = "platform-threadx")]
                    unsafe {
                        let _ = zpico_sys::zpico_spin_once(context.handle(), SLEEP_MS as u32);
                    }
                    #[cfg(not(feature = "platform-threadx"))]
                    unsafe {
                        z_sleep_ms(SLEEP_MS)
                    };
                }
            }
        }
        Err(TransportError::from(last_err.unwrap()))
    }

    fn take_response_raw(
        &mut self,
        reply_buf: &mut [u8],
    ) -> Result<Option<(usize, i64)>, Self::Error> {
        if self.pending_handles.is_empty() {
            return Ok(None);
        }

        let context = unsafe { &*self.context };

        #[cfg(not(feature = "std"))]
        {
            let _ = context.spin_once(0);
        }

        // Poll every outstanding handle and REPORT WHICH REQUEST replied.
        //
        // Issue 0778 — this used to be "first reply wins", justified by
        // "queryable is idempotent at the application layer". That is true of
        // the resend loop in the C-API blocking caller, which leaves several
        // generations of ONE logical request in flight — and false the moment
        // two DIFFERENT requests are outstanding, which this list cannot
        // distinguish on its own. `send_goal` and `SetParameters` both travel
        // this path and neither is idempotent. Each handle now carries the
        // sequence id of the request that produced it, so the caller can tell
        // which one it got back instead of the ABI assuming it does not matter.
        //
        // Newest first matches the common case where the latest send
        // is what completed — most calls allocate only one slot, so
        // we get out in one iteration.
        let mut hit_idx: Option<usize> = None;
        let mut hit_len: usize = 0;
        let mut hit_seq: i64 = 0;
        let mut hard_err: Option<Self::Error> = None;
        for (idx, &(handle, seq)) in self.pending_handles.iter().enumerate().rev() {
            match context.get_check(handle, reply_buf) {
                Ok(Some(len)) => {
                    hit_idx = Some(idx);
                    hit_len = len;
                    hit_seq = seq;
                    break;
                }
                Ok(None) => continue,
                Err(e) => {
                    // Note the error but keep checking the others —
                    // one slot's dropper-only timeout shouldn't lose
                    // a sibling's still-pending reply. If everyone
                    // errored we'll surface the last one.
                    hard_err = Some(TransportError::from(e));
                }
            }
        }

        if let Some(idx) = hit_idx {
            // Issue 0778 — drop only the generations of the request that
            // ANSWERED (everything sharing its sequence id), not every
            // outstanding slot. Clearing the lot is what made a second
            // in-flight request disappear when the first one replied.
            let answered = self.pending_handles[idx].1;
            self.pending_handles.retain(|&(_, seq)| seq != answered);
            return Ok(Some((hit_len, hit_seq)));
        }

        if let Some(e) = hard_err {
            // Every outstanding handle errored (e.g. each got a
            // dropper-only timeout without data). Surface the failure.
            self.pending_handles.clear();
            return Err(e);
        }

        Ok(None)
    }

    fn start_server_discovery(&mut self, timeout_ms: u32) -> Result<(), Self::Error> {
        // Idempotent: a previous query in flight is fine — let it run.
        if self.discovery_handle.is_some() {
            return Ok(());
        }
        // Already proven the server is visible; no need to re-query.
        if self.server_seen {
            return Ok(());
        }
        let context = unsafe { &*self.context };
        let handle = context
            .liveliness_get_start(
                &self.discovery_keyexpr[..=self.discovery_keyexpr_len],
                timeout_ms,
            )
            .map_err(TransportError::from)?;
        self.discovery_handle = Some(handle);
        Ok(())
    }

    fn poll_server_discovery(&mut self) -> Result<Option<bool>, Self::Error> {
        // issue 1087 — the latch stays, and the justification that used to sit
        // here does not. It claimed "rclcpp's `service_is_ready` snapshot
        // semantic"; there is no such semantic.
        // `rclcpp::ClientBase::service_is_ready()` calls
        // `rcl_service_server_is_available` on EVERY invocation, and upstream
        // `rmw.h` says the outcome reflects a QoS-compatibility CHANGE, in
        // either direction.
        //
        // What the latch actually is: a positive cache over a single-shot
        // liveliness query, with NO invalidation. It is right that a
        // `wait_for_service` which already succeeded need not re-query, and
        // wrong that a server which has since died still reads available for
        // the client's lifetime.
        //
        // The invalidation is NOT implemented here and issue 1087 stays open
        // for it: clearing this needs the liveliness subscription to report a
        // token DROP, which the shim does not surface today. Saying so rather
        // than shipping a comment that describes a hook nobody wrote.
        if self.server_seen {
            return Ok(Some(true));
        }
        let handle = match self.discovery_handle {
            Some(h) => h,
            None => return Ok(Some(false)),
        };
        let context = unsafe { &*self.context };
        match context.liveliness_get_check(handle) {
            Ok(true) => {
                self.discovery_handle = None;
                self.server_seen = true;
                Ok(Some(true))
            }
            Ok(false) => Ok(None),
            Err(crate::zpico::ZpicoError::Timeout) => {
                // Dropper fired with no replies — no server seen.
                self.discovery_handle = None;
                Ok(Some(false))
            }
            Err(e) => {
                self.discovery_handle = None;
                Err(TransportError::from(e))
            }
        }
    }

    // phase-379 W6 — the `is_server_ready` override was deleted with the trait
    // method. It returned `self.server_seen`, which is exactly what
    // `service_is_ready` below already reports, so nothing is lost: zenoh was
    // the ONLY backend that implemented the bool form honestly, and every other
    // one inherited a default of `true` (issue 1008).

    fn service_is_ready(&self) -> Result<bool, TransportError> {
        // Phase 124.C.2 — zenoh-pico tracks matched queryables via the
        // session's liveliness subscription. `server_seen` already
        // reflects "at least one matching queryable advertised", which
        // is the answer this probe wants.
        Ok(self.server_seen)
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
pub(super) mod tests {
    use super::*;
    use nros_rmw::TransportError;

    // --- Service buffer helpers ---

    /// Simulate a service request callback by enqueuing into the buffer ring.
    pub(in crate::shim) fn simulate_service_request(slot: usize, payload: &[u8], keyexpr: &[u8]) {
        let mut buf_ref = ServiceBufferRef::new(slot);
        let buffer = buf_ref.get_mut();

        let klen = keyexpr.len().min(buffer.keyexpr.len() - 1);
        buffer.keyexpr[..klen].copy_from_slice(&keyexpr[..klen]);
        buffer.keyexpr[klen] = 0;
        buffer.keyexpr_len.store(klen, Ordering::Release);

        let tail = buffer.tail.load(Ordering::Relaxed);
        let entry = &mut buffer.ring[tail % SERVICE_REQUEST_RING_DEPTH];
        let copy_len = payload.len().min(entry.data.len());
        entry.data[..copy_len].copy_from_slice(&payload[..copy_len]);
        entry.len.store(copy_len, Ordering::Relaxed);
        entry.overflow.store(false, Ordering::Relaxed);
        let seq = SERVICE_SEQ_COUNTER.fetch_add(1, Ordering::Relaxed);
        entry.seq.store(seq, Ordering::Relaxed);
        buffer.tail.store(tail.wrapping_add(1), Ordering::Release);
    }

    /// Reset a service buffer to idle state (empty ring).
    pub(in crate::shim) fn reset_service_buffer(slot: usize) {
        let mut buf_ref = ServiceBufferRef::new(slot);
        let buffer = buf_ref.get_mut();
        buffer.head.store(0, Ordering::Release);
        buffer.tail.store(0, Ordering::Release);
        buffer.keyexpr_len.store(0, Ordering::Release);
    }

    /// Try to receive a service request from a buffer slot.
    /// Replicates `take_request` logic for testing without a zenoh queryable.
    pub(in crate::shim) fn take_service(
        slot: usize,
        recv_buf: &mut [u8],
    ) -> Result<Option<usize>, TransportError> {
        let buf_ref = ServiceBufferRef::new(slot);
        let buffer = buf_ref.get();

        let head = buffer.head.load(Ordering::Relaxed);
        let tail = buffer.tail.load(Ordering::Acquire);
        if head == tail {
            return Ok(None);
        }
        let entry = &buffer.ring[head % SERVICE_REQUEST_RING_DEPTH];

        let len = entry.len.load(Ordering::Acquire);
        if len > recv_buf.len() {
            buffer.head.store(head.wrapping_add(1), Ordering::Release);
            return Err(TransportError::BufferTooSmall);
        }

        // Safety: Data is valid up to len bytes
        unsafe {
            core::ptr::copy_nonoverlapping(entry.data.as_ptr(), recv_buf.as_mut_ptr(), len);
        }

        buffer.head.store(head.wrapping_add(1), Ordering::Release);
        Ok(Some(len))
    }

    /// Read the keyexpr from a service buffer slot (for keyexpr preservation tests).
    fn read_service_keyexpr(slot: usize) -> heapless::Vec<u8, 256> {
        let buf_ref = ServiceBufferRef::new(slot);
        let buffer = buf_ref.get();
        let klen = buffer.keyexpr_len.load(Ordering::Acquire);
        let mut v = heapless::Vec::new();
        for i in 0..klen {
            let _ = v.push(buffer.keyexpr[i]);
        }
        v
    }

    /// Read the sequence number of the next-to-consume request in a slot's ring.
    fn read_service_seq(slot: usize) -> i64 {
        let buf_ref = ServiceBufferRef::new(slot);
        let b = buf_ref.get();
        let head = b.head.load(Ordering::Relaxed);
        b.ring[head % SERVICE_REQUEST_RING_DEPTH]
            .seq
            .load(Ordering::Acquire)
    }

    /// Test-only: does the slot's request ring hold an unread request?
    fn service_buf_has_request(slot: usize) -> bool {
        let buf_ref = ServiceBufferRef::new(slot);
        let b = buf_ref.get();
        b.head.load(Ordering::Acquire) != b.tail.load(Ordering::Acquire)
    }

    // ========================================================================
    // 37.1: Service buffer bug fix tests
    // ========================================================================

    #[test]
    fn service_buf_oversized_request_clears_has_request() {
        let slot = 6;
        reset_service_buffer(slot);

        let payload = [0xABu8; 512];
        simulate_service_request(slot, &payload, b"test/service");

        let mut small_buf = [0u8; 256];
        let result = take_service(slot, &mut small_buf);
        assert!(matches!(result, Err(TransportError::BufferTooSmall)));

        assert!(
            !service_buf_has_request(slot),
            "ring must be drained after BufferTooSmall to avoid stuck state"
        );

        simulate_service_request(slot, b"hello", b"test/service");
        let mut recv_buf = [0u8; 1024];
        let result = take_service(slot, &mut recv_buf);
        assert!(matches!(result, Ok(Some(5))));
        assert_eq!(&recv_buf[..5], b"hello");

        reset_service_buffer(slot);
    }

    #[test]
    fn service_buf_normal_request_after_stuck_recovery() {
        let slot = 5;
        reset_service_buffer(slot);

        simulate_service_request(slot, b"first", b"svc/a");
        let mut buf = [0u8; 1024];
        let result = take_service(slot, &mut buf);
        assert!(matches!(result, Ok(Some(5))));
        assert_eq!(&buf[..5], b"first");

        let result = take_service(slot, &mut buf);
        assert!(matches!(result, Ok(None)));

        simulate_service_request(slot, b"second", b"svc/a");
        let result = take_service(slot, &mut buf);
        assert!(matches!(result, Ok(Some(6))));
        assert_eq!(&buf[..6], b"second");

        reset_service_buffer(slot);
    }

    // ========================================================================
    // 37.1a: Service buffer state machine tests
    // ========================================================================

    #[test]
    fn svc_buf_idle_poll() {
        let slot = 0;
        reset_service_buffer(slot);

        let mut buf = [0u8; 1024];
        let result = take_service(slot, &mut buf);
        assert!(matches!(result, Ok(None)));

        assert!(!service_buf_has_request(slot));
    }

    #[test]
    fn svc_buf_normal_request() {
        let slot = 1;
        reset_service_buffer(slot);

        simulate_service_request(slot, b"request_data", b"svc/test");

        assert!(service_buf_has_request(slot));

        let mut recv_buf = [0u8; 1024];
        let result = take_service(slot, &mut recv_buf);
        assert!(matches!(result, Ok(Some(12))));
        assert_eq!(&recv_buf[..12], b"request_data");

        assert!(!service_buf_has_request(slot));
    }

    #[test]
    fn svc_buf_max_payload() {
        let slot = 2;
        reset_service_buffer(slot);

        // Exactly 1024 bytes = max capacity
        let payload = [0xCCu8; 1024];
        simulate_service_request(slot, &payload, b"svc/big");

        let mut recv_buf = [0u8; 1024];
        let result = take_service(slot, &mut recv_buf);
        assert!(matches!(result, Ok(Some(1024))));
        assert_eq!(&recv_buf, &payload);
    }

    #[test]
    fn svc_buf_caller_too_small_recovery() {
        let slot = 3;
        reset_service_buffer(slot);

        // Store 512 bytes, receive into 256-byte buffer
        let payload = [0xDDu8; 512];
        simulate_service_request(slot, &payload, b"svc/test");

        let mut small_buf = [0u8; 256];
        let result = take_service(slot, &mut small_buf);
        assert!(matches!(result, Err(TransportError::BufferTooSmall)));

        // Oversized head entry dropped → ring drained.
        assert!(!service_buf_has_request(slot));

        // Next request accepted
        simulate_service_request(slot, b"ok", b"svc/test");
        let mut recv_buf = [0u8; 1024];
        let result = take_service(slot, &mut recv_buf);
        assert!(matches!(result, Ok(Some(2))));
        assert_eq!(&recv_buf[..2], b"ok");
    }

    #[test]
    fn svc_buf_ring_buffers_unread() {
        // Phase 237 follow-up — two requests arriving before a drain are both
        // buffered in the ring (in order), not overwritten (the old single-buffer
        // behaviour dropped the first).
        let slot = 4;
        reset_service_buffer(slot);

        simulate_service_request(slot, b"first_req", b"svc/a");
        simulate_service_request(slot, b"second_req", b"svc/a");

        let mut recv_buf = [0u8; 1024];
        let r1 = take_service(slot, &mut recv_buf);
        assert!(matches!(r1, Ok(Some(9))));
        assert_eq!(&recv_buf[..9], b"first_req");

        let r2 = take_service(slot, &mut recv_buf);
        assert!(matches!(r2, Ok(Some(10))));
        assert_eq!(&recv_buf[..10], b"second_req");

        assert!(!service_buf_has_request(slot));
    }

    #[test]
    fn svc_buf_double_consume() {
        let slot = 0;
        reset_service_buffer(slot);

        simulate_service_request(slot, b"once", b"svc/a");

        let mut recv_buf = [0u8; 1024];
        let result = take_service(slot, &mut recv_buf);
        assert!(matches!(result, Ok(Some(4))));

        let result = take_service(slot, &mut recv_buf);
        assert!(matches!(result, Ok(None)));
    }

    #[test]
    fn svc_buf_sequence_numbers() {
        let slot = 7;
        reset_service_buffer(slot);

        // Three sequential requests — sequence numbers should increment
        simulate_service_request(slot, b"r1", b"svc/a");
        let seq1 = read_service_seq(slot);

        // Consume before next request
        let mut buf = [0u8; 1024];
        let _ = take_service(slot, &mut buf);

        simulate_service_request(slot, b"r2", b"svc/a");
        let seq2 = read_service_seq(slot);
        let _ = take_service(slot, &mut buf);

        simulate_service_request(slot, b"r3", b"svc/a");
        let seq3 = read_service_seq(slot);
        let _ = take_service(slot, &mut buf);

        assert!(seq2 > seq1, "seq2 ({seq2}) should be > seq1 ({seq1})");
        assert!(seq3 > seq2, "seq3 ({seq3}) should be > seq2 ({seq2})");
    }

    #[test]
    fn svc_buf_keyexpr_preserved() {
        let slot = 1;
        reset_service_buffer(slot);

        let keyexpr = b"0/my_service/example_interfaces::srv::dds_::AddTwoInts/Reply";
        simulate_service_request(slot, b"payload", keyexpr);

        let stored = read_service_keyexpr(slot);
        assert_eq!(stored.as_slice(), keyexpr);

        // Consume and verify keyexpr was available during request
        let mut recv_buf = [0u8; 1024];
        let result = take_service(slot, &mut recv_buf);
        assert!(matches!(result, Ok(Some(7))));
    }

    #[test]
    fn svc_buf_all_slots_independent() {
        let slot_a = 0;
        let slot_b = 7;
        reset_service_buffer(slot_a);
        reset_service_buffer(slot_b);

        simulate_service_request(slot_a, b"req_zero", b"svc/0");
        simulate_service_request(slot_b, b"req_seven", b"svc/7");

        // Consume slot_b first
        let mut recv_buf = [0u8; 1024];
        let result = take_service(slot_b, &mut recv_buf);
        assert!(matches!(result, Ok(Some(9))));
        assert_eq!(&recv_buf[..9], b"req_seven");

        // slot_a still has its request
        assert!(service_buf_has_request(slot_a));

        let result = take_service(slot_a, &mut recv_buf);
        assert!(matches!(result, Ok(Some(8))));
        assert_eq!(&recv_buf[..8], b"req_zero");
    }
}
