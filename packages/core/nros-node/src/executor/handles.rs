//! Entity wrapper types for the embedded executor.

use core::marker::PhantomData;

use nros_core::{CdrReader, CdrWriter, Deserialize, RosAction, RosMessage, RosService, Serialize};
use nros_rmw::{ClientTrait, Publisher, ServiceTrait, Subscription as _, TransportError};

use crate::session;

use super::types::{DEFAULT_TX_BUF, NodeError};

/// Default polling interval (ms) for sync wait loops.
const DEFAULT_SPIN_INTERVAL_MS: u64 = 10;

/// Check whether the given budget has been exhausted.
///
/// `std` builds measure wall-clock against `Instant::now()`; `no_std`
/// builds count iterations and exhaust after `max_iterations` calls.
///
/// **Phase 89.8**: the plain `max_iters` approach is insufficient on
/// multi-threaded zpico backends (POSIX/Zephyr/NuttX). There,
/// `executor.spin_once(10ms)` waits on a condvar that zenoh-pico's
/// background tasks signal on any inbound frame (keep-alives,
/// discovery gossip, interest messages). Each signal returns the
/// spin well before the 10 ms budget, so a nominal
/// `1000 × 10 ms = 10 s` iteration count collapses to milliseconds
/// of real time and the wait returns `Timeout` long before the
/// awaited reply can arrive.
///
/// Same class of bug 89.2 fixed in `nros-c`'s blocking service call
/// and 89.3 fixed in `nros-cpp`'s action-client helpers. The
/// maintainer explicitly flagged this `Promise::wait` / `wait_next`
/// path in the 89.2 commit: *"Promise::wait in nros-node has the
/// same structural bug but currently passes all tests. Left on
/// max_spins until a test surfaces it."* The NuttX Rust action
/// E2E is that test.
struct WaitBudget {
    /// The clock this budget is counting against, when the build has one.
    ///
    /// phase-359 W10 — this used to be a `std`/`no_std` PAIR: a
    /// `std::time::Instant` deadline on one side, an iteration count on the
    /// other. Which one a build got was decided by whether some crate in the
    /// graph named `std`, so dropping that feature from a hosted consumer
    /// silently converted every timeout in this file into "N spins" — a
    /// different quantity wearing the same name.
    ///
    /// The choice is now made on what the build can actually DO. Any build with
    /// a clock — every platform port exports one — gets a real deadline. The
    /// iteration count survives as the honest fallback for a build with no
    /// clock at all, which is what it always was.
    clock: Option<fn() -> u64>,
    /// Absolute deadline in `clock`'s µs, meaningful when `clock` is `Some`.
    deadline_us: u64,
    /// Iterations left, meaningful when `clock` is `None`.
    remaining: u64,
}

impl WaitBudget {
    fn new(max_iterations: u64, timeout: core::time::Duration) -> Self {
        let clock = super::spin::default_clock_us_fn();
        let timeout_us = timeout.as_micros().min(u64::MAX as u128) as u64;
        Self {
            clock,
            deadline_us: clock.map(|c| c().saturating_add(timeout_us)).unwrap_or(0),
            remaining: max_iterations,
        }
    }

    fn tick(&mut self) -> bool {
        match self.clock {
            Some(clock) => clock() < self.deadline_us,
            None => {
                if self.remaining == 0 {
                    false
                } else {
                    self.remaining -= 1;
                    true
                }
            }
        }
    }
}

/// UUID byte count in a ROS 2 GoalId.
///
/// CDR encoding: a fixed `uint8[16]` array — ROS 2 `unique_identifier_msgs/UUID`
/// — with **no** length prefix (fixed arrays are unprefixed in CDR).
const GOAL_UUID_SIZE: usize = 16;

// ============================================================================
// EmbeddedPublisher
// ============================================================================

/// Typed publisher handle.
///
/// Two methods, both byte-oriented at the wire:
///
/// - [`publish`](Self::publish) / [`publish_with_buffer`](Self::publish_with_buffer)
///   — accept `&M: RosMessage`, CDR-encode into a stack buffer, then
///   call [`Publisher::publish_raw`](nros_rmw::Publisher::publish_raw).
/// - [`publish_raw`](Self::publish_raw) — accepts pre-encoded CDR bytes
///   for callers that already produced the wire payload.
///
/// **No typed `loan()` exists.** Loan/borrow live exclusively on
/// [`EmbeddedRawPublisher`] / [`RawSubscription`]. `try_loan(len)`
/// requires the byte length up front, which CDR ser/de can only
/// discover after encoding — the two APIs are incompatible by
/// construction. See `docs/design/0010-zero-copy-raw-api.md` decision D7.
pub struct EmbeddedPublisher<M> {
    pub(crate) handle: session::RmwPublisher,
    /// Phase 108 — registered event closures kept alive for the
    /// publisher's lifetime; freed in `Drop`.
    pub(crate) event_regs: EventRegs,
    /// RFC-0052 W3b.4 — contracted endpoint's publish counter (`None`
    /// for uncontracted publishers; one relaxed atomic bump when set).
    pub(crate) monitor: Option<&'static crate::executor::monitor::PubMonitorCell>,
    /// Epoch clock for the publish-stamp observation, paired with `monitor`
    /// the way the subscriber's `AgeMon` pairs cell and clock. Separate
    /// field rather than a tuple so an uncontracted publisher costs nothing
    /// and the existing `monitor` construction sites keep their shape.
    pub(crate) epoch: Option<fn() -> u64>,
    pub(crate) _phantom: PhantomData<M>,
}

impl<M> Drop for EmbeddedPublisher<M> {
    fn drop(&mut self) {
        drop_event_regs(&mut self.event_regs);
    }
}

impl<M: RosMessage> EmbeddedPublisher<M> {
    /// Publish a message using the default buffer size.
    pub fn publish(&self, msg: &M) -> Result<(), NodeError> {
        self.publish_with_buffer::<DEFAULT_TX_BUF>(msg)
    }

    /// Publish a message with a custom buffer size.
    pub fn publish_with_buffer<const BUF: usize>(&self, msg: &M) -> Result<(), NodeError> {
        let mut buffer = [0u8; BUF];
        let mut writer = crate::tx_writer(&mut buffer).map_err(|_| NodeError::BufferTooSmall)?;
        msg.serialize(&mut writer)
            .map_err(|_| NodeError::Serialization)?;
        let len = writer.position();
        self.bump_monitor();
        self.observe_publish_stamp(&buffer[..len]);
        self.handle
            .publish_raw(&buffer[..len])
            .map_err(|_| NodeError::Transport(TransportError::PublishFailed))
    }

    /// RFC-0052 W3b.4 — one relaxed bump per publish on contracted
    /// endpoints; a predictable no-op otherwise.
    /// Record how old the data we just put on the wire was.
    ///
    /// Folds away completely for a type with no `STAMP_OFFSET`, an
    /// uncontracted publisher, or a build with no epoch source -- the same
    /// three exits as the subscriber's `observe_age`.
    #[inline]
    fn observe_publish_stamp(&self, raw: &[u8]) {
        if let (Some(cell), Some(epoch), Some(off)) =
            (self.monitor, self.epoch, <M as RosMessage>::STAMP_OFFSET)
        {
            crate::executor::monitor::observe_publish_stamp(cell, raw, off, epoch());
        }
    }

    #[inline]
    fn bump_monitor(&self) {
        if let Some(cell) = self.monitor {
            cell.count
                .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
    }

    /// Publish raw CDR-encoded data (must include CDR header).
    pub fn publish_raw(&self, data: &[u8]) -> Result<(), NodeError> {
        self.bump_monitor();
        self.handle
            .publish_raw(data)
            .map_err(|_| NodeError::Transport(TransportError::PublishFailed))
    }

    /// Phase 124.E.1 — streamed publish. Use a closure-driven writer
    /// to serialise straight into the backend's outbound buffer
    /// without a per-publisher staging copy. Saves on RAM-constrained
    /// nodes that publish multi-KB payloads.
    ///
    /// The `writer` closure receives a [`StreamWriter`] mutable
    /// reference and uses [`StreamWriter::write`] / [`extend`] /
    /// [`reserved_len`] to fill the slot in chunks. The total
    /// payload length must be declared up-front via
    /// [`StreamWriter::reserve_total`]; the backend allocates that
    /// many bytes in its outbound buffer before any chunks land.
    ///
    /// Backends without a native stream slot fall through to a
    /// stack-allocated staging buffer (capped at 4 KiB) + a single
    /// `publish_raw` — same observable result, just no zero-staging
    /// win for the big-message case.
    pub fn publish_streamed<F>(&self, total_len: usize, writer: F) -> Result<(), NodeError>
    where
        F: FnMut(&mut [u8]) -> usize,
    {
        // Wrap the closure in a `*mut c_void` so it survives the
        // crossing into the C callback contract the vtable expects.
        // The closure is consumed by reference, so the lifetime is
        // bounded by this function's frame — no escape.
        use nros_rmw::Publisher;
        struct Ctx<W> {
            writer: W,
            total: usize,
        }
        unsafe extern "C" fn size_cb<W>(
            out_total_len: *mut usize,
            user_ctx: *mut core::ffi::c_void,
        ) {
            unsafe {
                let ctx = &*(user_ctx as *const Ctx<W>);
                *out_total_len = ctx.total;
            }
        }
        unsafe extern "C" fn chunk_cb<W: FnMut(&mut [u8]) -> usize>(
            out_buf: *mut u8,
            cap: usize,
            out_written: *mut usize,
            user_ctx: *mut core::ffi::c_void,
        ) {
            unsafe {
                let ctx = &mut *(user_ctx as *mut Ctx<W>);
                let slot = core::slice::from_raw_parts_mut(out_buf, cap);
                let n = (ctx.writer)(slot);
                *out_written = n;
            }
        }
        let mut ctx = Ctx {
            writer,
            total: total_len,
        };
        let ctx_ptr = &mut ctx as *mut Ctx<F> as *mut core::ffi::c_void;
        // SAFETY: `ctx` lives until this call returns, and both callbacks
        // only cast `user_ctx` back to that stack-local `Ctx<F>`.
        unsafe {
            self.handle
                .publish_streamed(size_cb::<F>, chunk_cb::<F>, ctx_ptr)
        }
        .map_err(NodeError::Transport)
    }

    /// Phase 108.B — manually assert this publisher's liveliness.
    /// Required for publishers configured with
    /// [`QoSLivelinessPolicy::ManualByTopic`] /
    /// [`QoSLivelinessPolicy::ManualByNode`]. No-op for AUTOMATIC /
    /// NONE kinds. Returns `Err(Unsupported)` if the backend doesn't
    /// implement manual liveliness.
    pub fn assert_liveliness(&self) -> Result<(), NodeError> {
        use nros_rmw::Publisher as _;
        self.handle
            .assert_liveliness()
            .map_err(NodeError::Transport)
    }

    // ====================================================================
    // Phase 108 — status events
    // ====================================================================
    //
    // Publisher-side: `LivelinessLost` and `OfferedDeadlineMissed`.
    // Returns `NodeError::Transport(TransportError::Unsupported)` if
    // the active backend doesn't generate the event for this entity.

    /// `true` if the active backend can fire the named event for this
    /// publisher.
    #[cfg(feature = "alloc")]
    pub fn supports_event(&self, kind: nros_rmw::EventKind) -> bool {
        use nros_rmw::Publisher as _;
        self.handle.supports_event(kind)
    }

    /// Register a callback for `LivelinessLost`. Fires when this
    /// publisher misses its own liveliness assertion deadline.
    #[cfg(feature = "alloc")]
    pub fn on_liveliness_lost<F>(&mut self, cb: F) -> Result<(), NodeError>
    where
        F: FnMut(nros_rmw::CountStatus) + Send + 'static,
    {
        register_pub_event::<F, _>(
            &mut self.handle,
            &mut self.event_regs,
            nros_rmw::EventKind::LivelinessLost,
            0,
            cb,
            |payload, f| {
                if let nros_rmw::EventPayload::LivelinessLost(s) = payload {
                    f(*s);
                }
            },
        )
    }

    /// Register a callback for `OfferedDeadlineMissed`. Fires when
    /// this publisher promised `deadline` and falls behind.
    #[cfg(feature = "alloc")]
    pub fn on_offered_deadline_missed<F>(
        &mut self,
        deadline: core::time::Duration,
        cb: F,
    ) -> Result<(), NodeError>
    where
        F: FnMut(nros_rmw::CountStatus) + Send + 'static,
    {
        register_pub_event::<F, _>(
            &mut self.handle,
            &mut self.event_regs,
            nros_rmw::EventKind::OfferedDeadlineMissed,
            deadline.as_millis().min(u32::MAX as u128) as u32,
            cb,
            |payload, f| {
                if let nros_rmw::EventPayload::OfferedDeadlineMissed(s) = payload {
                    f(*s);
                }
            },
        )
    }
}

/// Cap on registered event callbacks per entity. Subscribers can hold
/// up to 3 (LivelinessChanged + RequestedDeadlineMissed + MessageLost);
/// publishers up to 2 (LivelinessLost + OfferedDeadlineMissed). One vec
/// type fits both — extra slots are unused on publishers.
#[cfg(feature = "alloc")]
pub(crate) const MAX_EVENTS_PER_ENTITY: usize = 3;

/// One row of the per-entity event-callback registry. Stores enough to
/// type-erase the boxed closure for `Drop`-time deallocation.
#[cfg(feature = "alloc")]
#[derive(Clone, Copy)]
pub(crate) struct EventReg {
    /// `Box::into_raw`-derived pointer; valid for the entity's lifetime.
    pub(crate) ctx: *mut core::ffi::c_void,
    /// Type-erased destructor. Calls `Box::from_raw` w/ the originating
    /// monomorphic type, dropping the closure + freeing the heap slot.
    pub(crate) drop_fn: unsafe fn(*mut core::ffi::c_void),
}

#[cfg(feature = "alloc")]
pub(crate) type EventRegs = heapless::Vec<EventReg, MAX_EVENTS_PER_ENTITY>;

/// Empty placeholder for no-alloc builds — keeps struct layout stable
/// across feature combinations without paying any space.
#[cfg(not(feature = "alloc"))]
#[derive(Default, Clone, Copy)]
pub(crate) struct EventRegs;

/// Empty initial value for the `event_regs` field. Selected at compile
/// time so call sites are uniform across feature combinations
/// (`heapless::Vec::new()` for `alloc`; unit-struct constructor
/// otherwise — clippy's `default_constructed_unit_structs` lint
/// rejects the `EventRegs::default()` form on the unit-struct branch).
#[cfg(feature = "alloc")]
#[inline]
pub(crate) fn empty_event_regs() -> EventRegs {
    heapless::Vec::new()
}
#[cfg(not(feature = "alloc"))]
#[inline]
pub(crate) fn empty_event_regs() -> EventRegs {
    EventRegs
}

#[cfg(feature = "alloc")]
pub(crate) fn drop_event_regs(regs: &mut EventRegs) {
    while let Some(reg) = regs.pop() {
        // SAFETY: `reg.ctx` was obtained from `Box::into_raw` of the
        // monomorphic type that `reg.drop_fn` knows about. Each reg is
        // visited exactly once because we drain via `pop`.
        unsafe { (reg.drop_fn)(reg.ctx) };
    }
}

#[cfg(not(feature = "alloc"))]
#[inline]
pub(crate) fn drop_event_regs(_regs: &mut EventRegs) {}

#[cfg(feature = "alloc")]
fn register_pub_event<F, D>(
    handle: &mut session::RmwPublisher,
    regs: &mut EventRegs,
    kind: nros_rmw::EventKind,
    deadline_ms: u32,
    user_cb: F,
    dispatch: D,
) -> Result<(), NodeError>
where
    F: FnMut(nros_rmw::CountStatus) + Send + 'static,
    D: Fn(nros_rmw::EventPayload<'_>, &mut F) + 'static,
{
    use nros_rmw::Publisher as _;
    if regs.is_full() {
        return Err(NodeError::Transport(TransportError::Unsupported));
    }
    let state = alloc::boxed::Box::new(EventClosureState { user_cb, dispatch });
    let user_ctx = alloc::boxed::Box::into_raw(state) as *mut core::ffi::c_void;
    // SAFETY: trampoline downcasts `user_ctx` back to the boxed
    // EventClosureState. Box ownership is recorded in `regs`; entity
    // Drop walks the registry and frees via `drop_event_state::<F, D>`.
    let res = unsafe {
        handle.register_event_callback(kind, deadline_ms, event_trampoline::<F, D>, user_ctx)
    };
    match res {
        Ok(()) => {
            // is_full check above guarantees push() succeeds.
            let _ = regs.push(EventReg {
                ctx: user_ctx,
                drop_fn: drop_event_state::<F, D>,
            });
            Ok(())
        }
        Err(e) => {
            // SAFETY: backend rejected the registration; reclaim the
            // box we just leaked into raw form.
            unsafe {
                drop(alloc::boxed::Box::from_raw(
                    user_ctx as *mut EventClosureState<F, D>,
                ));
            }
            Err(NodeError::Transport(e))
        }
    }
}

#[cfg(feature = "alloc")]
struct EventClosureState<F, D> {
    user_cb: F,
    dispatch: D,
}

#[cfg(feature = "alloc")]
unsafe extern "C" fn event_trampoline<F, D>(
    kind: nros_rmw::EventKind,
    payload_ptr: *const core::ffi::c_void,
    user_ctx: *mut core::ffi::c_void,
) where
    F: FnMut(nros_rmw::CountStatus) + Send + 'static,
    D: Fn(nros_rmw::EventPayload<'_>, &mut F) + 'static,
{
    let state = unsafe { &mut *(user_ctx as *mut EventClosureState<F, D>) };
    let payload = unsafe { nros_rmw::payload_from_raw(kind, payload_ptr) };
    (state.dispatch)(payload, &mut state.user_cb);
}

#[cfg(feature = "alloc")]
unsafe fn drop_event_state<F, D>(ctx: *mut core::ffi::c_void)
where
    F: FnMut(nros_rmw::CountStatus) + Send + 'static,
    D: Fn(nros_rmw::EventPayload<'_>, &mut F) + 'static,
{
    // SAFETY: caller guarantees `ctx` was obtained from
    // `Box::into_raw::<EventClosureState<F, D>>` and not yet freed.
    unsafe {
        drop(alloc::boxed::Box::from_raw(
            ctx as *mut EventClosureState<F, D>,
        ));
    }
}

#[cfg(feature = "alloc")]
fn register_sub_event_count<F, D>(
    handle: &mut session::RmwSubscriber,
    regs: &mut EventRegs,
    kind: nros_rmw::EventKind,
    deadline_ms: u32,
    user_cb: F,
    dispatch: D,
) -> Result<(), NodeError>
where
    F: FnMut(nros_rmw::CountStatus) + Send + 'static,
    D: Fn(nros_rmw::EventPayload<'_>, &mut F) + 'static,
{
    use nros_rmw::Subscription as _;
    if regs.is_full() {
        return Err(NodeError::Transport(TransportError::Unsupported));
    }
    let state = alloc::boxed::Box::new(EventClosureState { user_cb, dispatch });
    let user_ctx = alloc::boxed::Box::into_raw(state) as *mut core::ffi::c_void;
    let res = unsafe {
        handle.register_event_callback(kind, deadline_ms, event_trampoline::<F, D>, user_ctx)
    };
    match res {
        Ok(()) => {
            let _ = regs.push(EventReg {
                ctx: user_ctx,
                drop_fn: drop_event_state::<F, D>,
            });
            Ok(())
        }
        Err(e) => {
            unsafe {
                drop(alloc::boxed::Box::from_raw(
                    user_ctx as *mut EventClosureState<F, D>,
                ));
            }
            Err(NodeError::Transport(e))
        }
    }
}

#[cfg(feature = "alloc")]
fn register_sub_event_liveliness<F>(
    handle: &mut session::RmwSubscriber,
    regs: &mut EventRegs,
    user_cb: F,
) -> Result<(), NodeError>
where
    F: FnMut(nros_rmw::LivelinessChangedStatus) + Send + 'static,
{
    use nros_rmw::Subscription as _;
    if regs.is_full() {
        return Err(NodeError::Transport(TransportError::Unsupported));
    }
    let state = alloc::boxed::Box::new(LivelinessClosureState { user_cb });
    let user_ctx = alloc::boxed::Box::into_raw(state) as *mut core::ffi::c_void;
    let res = unsafe {
        handle.register_event_callback(
            nros_rmw::EventKind::LivelinessChanged,
            0,
            liveliness_trampoline::<F>,
            user_ctx,
        )
    };
    match res {
        Ok(()) => {
            let _ = regs.push(EventReg {
                ctx: user_ctx,
                drop_fn: drop_liveliness_state::<F>,
            });
            Ok(())
        }
        Err(e) => {
            unsafe {
                drop(alloc::boxed::Box::from_raw(
                    user_ctx as *mut LivelinessClosureState<F>,
                ));
            }
            Err(NodeError::Transport(e))
        }
    }
}

#[cfg(feature = "alloc")]
unsafe fn drop_liveliness_state<F>(ctx: *mut core::ffi::c_void)
where
    F: FnMut(nros_rmw::LivelinessChangedStatus) + Send + 'static,
{
    unsafe {
        drop(alloc::boxed::Box::from_raw(
            ctx as *mut LivelinessClosureState<F>,
        ));
    }
}

#[cfg(feature = "alloc")]
struct LivelinessClosureState<F> {
    user_cb: F,
}

#[cfg(feature = "alloc")]
unsafe extern "C" fn liveliness_trampoline<F>(
    kind: nros_rmw::EventKind,
    payload_ptr: *const core::ffi::c_void,
    user_ctx: *mut core::ffi::c_void,
) where
    F: FnMut(nros_rmw::LivelinessChangedStatus) + Send + 'static,
{
    let state = unsafe { &mut *(user_ctx as *mut LivelinessClosureState<F>) };
    let payload = unsafe { nros_rmw::payload_from_raw(kind, payload_ptr) };
    if let nros_rmw::EventPayload::LivelinessChanged(s) = payload {
        (state.user_cb)(*s);
    }
}

// ============================================================================
// EmbeddedRawPublisher — typeless publisher for non-ROS message wire formats
// ============================================================================

/// Default size of each per-publisher arena slot, in bytes.
pub const DEFAULT_LOAN_BUF: usize = 1024;

use core::cell::UnsafeCell;
// portable-atomic AtomicBool — resolves to native on targets that support it,
// software fallback on those that don't (e.g. some Xtensa ESP32 SoCs). Use
// portable-atomic's Ordering too so the type sees the matching trait
// implementation across all targets.
use portable_atomic::{AtomicBool, Ordering};

/// Typeless publisher handle. Use when the wire format is not ROS CDR
/// (e.g. PX4 uORB raw POD bytes, custom binary protocols).
///
/// Two publish paths:
///
/// - [`publish_raw`](Self::publish_raw): user supplies a `&[u8]`, backend
///   memcpys into its outbound buffer. One copy.
/// - [`try_loan`](Self::try_loan): backend (or per-publisher arena fallback)
///   hands user a `&mut [u8]` slice. User writes in place. [`PublishLoan::commit`]
///   triggers the wire write. Zero-copy on backends with native lending
///   (Phase 99: zenoh-pico `unstable-zenoh-api`, XRCE-DDS); single-memcpy
///   fallback on backends without (uORB).
///
/// The const-generic `TX_BUF` sizes the inline arena slot (default
/// [`DEFAULT_LOAN_BUF`]). Loans larger than `TX_BUF` return
/// `LoanError::TooLarge`.
pub struct EmbeddedRawPublisher<const TX_BUF: usize = DEFAULT_LOAN_BUF> {
    pub(crate) handle: session::RmwPublisher,
    /// Single-slot arena: writable buffer + busy flag. SLOTS=1 in v1
    /// (concurrent loans on the same publisher return WouldBlock).
    /// Unused when the `rmw-lending` feature is on — `try_loan`
    /// dispatches to the backend's `SlotLending` instead.
    #[allow(dead_code)]
    pub(crate) arena: TxArena<TX_BUF>,
    /// Phase 108 — registered event closures.
    pub(crate) event_regs: EventRegs,
}

impl<const TX_BUF: usize> Drop for EmbeddedRawPublisher<TX_BUF> {
    fn drop(&mut self) {
        drop_event_regs(&mut self.event_regs);
    }
}

/// Single-slot per-publisher arena. Concurrent `try_loan` calls on the
/// same publisher race on the busy flag; loser gets `WouldBlock`.
///
/// `waker` lets `loan().await` register a waker before returning
/// `Pending`; `release()` wakes it so the next `try_loan` succeeds
/// without polling the executor loop. Phase 99.H' — replaces the
/// earlier `wake_by_ref + Pending` busy yield with an event-driven
/// wake, and gives `LoanFuture::Drop` a place to release a pending
/// reservation cleanly.
#[allow(dead_code)]
pub(crate) struct TxArena<const TX_BUF: usize> {
    busy: AtomicBool,
    buf: UnsafeCell<[u8; TX_BUF]>,
    waker: atomic_waker::AtomicWaker,
}

// SAFETY: Sync-ness of the arena is enforced by the `busy` flag — only
// the thread that won the CAS may access `buf`, and only until commit/
// discard releases the slot.
unsafe impl<const TX_BUF: usize> Sync for TxArena<TX_BUF> {}

#[allow(dead_code)] // unused when `rmw-lending` is on
impl<const TX_BUF: usize> TxArena<TX_BUF> {
    pub(crate) const fn new() -> Self {
        Self {
            busy: AtomicBool::new(false),
            buf: UnsafeCell::new([0u8; TX_BUF]),
            waker: atomic_waker::AtomicWaker::new(),
        }
    }

    /// Try to claim the arena slot. Returns a raw pointer + len pair on
    /// success; caller wraps it in a `PublishLoan`. Returns `false` if
    /// the slot is already loaned.
    ///
    /// `&self` returning `&mut` is sound because the `busy` flag
    /// gates exclusivity at runtime — the CAS in this function is
    /// the only writer, and `release()` is only callable through the
    /// loan's `Drop`.
    #[allow(clippy::mut_from_ref)]
    fn try_claim(&self, len: usize) -> Result<&mut [u8], LoanError> {
        if len > TX_BUF {
            return Err(LoanError::TooLarge);
        }
        if self
            .busy
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return Err(LoanError::WouldBlock);
        }
        // SAFETY: we just won the busy CAS; we hold exclusive access
        // until release(). Lifetime is tied to `&self` for the loan.
        let buf_ref: &mut [u8; TX_BUF] = unsafe { &mut *self.buf.get() };
        Ok(&mut buf_ref[..len])
    }

    fn release(&self) {
        self.busy.store(false, Ordering::Release);
        // Wake any pending `LoanFuture` waiting on this arena. Cheap
        // no-op if no one is waiting.
        self.waker.wake();
    }
}

impl<const TX_BUF: usize> EmbeddedRawPublisher<TX_BUF> {
    /// RFC-0088 — the serialization format this publisher expects its raw
    /// bytes to already be in. Counterpart to [`RawSubscription::format`].
    pub const fn format(&self) -> nros_serdes::format::SerializationFormatId {
        crate::session::IMAGE_SERIALIZATION_FORMAT_ID
    }

    /// Construct an [`EmbeddedRawPublisher`] from a backend-allocated
    /// `RmwPublisher` handle. Public so external extension crates
    /// (e.g. `nros-px4` for typed uORB wrappers) can wrap a handle
    /// they obtained directly from the active session via
    /// [`crate::Node::session_mut`] + a backend-specific create method.
    ///
    /// Most users should not call this — use [`crate::Node::create_publisher`]
    /// or [`crate::Node::create_publisher_raw`] instead.
    pub fn new(handle: session::RmwPublisher) -> Self {
        Self {
            handle,
            arena: TxArena::new(),
            event_regs: empty_event_regs(),
        }
    }

    /// Phase 108.A — `true` if the active backend can fire the named
    /// event for this raw publisher.
    #[cfg(feature = "alloc")]
    pub fn supports_event(&self, kind: nros_rmw::EventKind) -> bool {
        use nros_rmw::Publisher as _;
        self.handle.supports_event(kind)
    }

    /// Publish a pre-encoded byte slice. The byte format depends entirely
    /// on the active RMW backend:
    ///
    /// - **zenoh / XRCE-DDS / DDS**: CDR-encoded payload including the
    ///   4-byte CDR header.
    /// - **uORB**: raw POD struct bytes (no header). Length must equal
    ///   `size_of::<T::Msg>()` for the registered topic.
    pub fn publish_raw(&self, data: &[u8]) -> Result<(), NodeError> {
        self.handle
            .publish_raw(data)
            .map_err(|_| NodeError::Transport(TransportError::PublishFailed))
    }

    /// Phase 128.F.4 — raw publish with a wire-level attachment block.
    ///
    /// `attachment` rides alongside the payload on backends that
    /// natively support it (zenoh-pico, Cyclone DDS). Backends without
    /// native support silently discard `attachment` and fall back to
    /// the regular [`publish_raw`](Self::publish_raw) path — the
    /// default `Publisher::publish_raw_with_attachment` body in
    /// `nros-rmw` does this delegation.
    ///
    /// Primary use case: cross-RMW bridges stamp the source backend's
    /// RMW name as `bridge_origin` so a paired return bridge can drop
    /// echoed frames deterministically.
    pub fn publish_raw_with_attachment(
        &self,
        data: &[u8],
        attachment: &[u8],
    ) -> Result<(), NodeError> {
        self.handle
            .publish_raw_with_attachment(data, attachment)
            .map_err(|_| NodeError::Transport(TransportError::PublishFailed))
    }

    /// Phase 108.B — manually assert this publisher's liveliness.
    /// Required for `QoSLivelinessPolicy::ManualByTopic` /
    /// `ManualByNode`. No-op for AUTOMATIC / NONE.
    pub fn assert_liveliness(&self) -> Result<(), NodeError> {
        use nros_rmw::Publisher as _;
        self.handle
            .assert_liveliness()
            .map_err(NodeError::Transport)
    }

    /// Reserve a writable slot of `len` bytes. Caller writes into the
    /// returned [`PublishLoan`] and calls [`PublishLoan::commit`] to
    /// publish. Never blocks; returns [`LoanError::WouldBlock`] when the
    /// slot is already in use (arena fallback) or the backend's outbound
    /// stream is full (lending path), and [`LoanError::TooLarge`] when
    /// `len` exceeds the publisher's slot capacity.
    ///
    /// With the `rmw-lending` cargo feature on, this dispatches to the
    /// active backend's [`SlotLending::try_lend_slot`](nros_rmw::SlotLending::try_lend_slot)
    /// — zero-copy on backends that natively lend (zenoh-pico via
    /// `z_bytes_from_static_buf`, XRCE-DDS via `uxr_prepare_output_stream`).
    /// Without `rmw-lending`, the arena fallback is used: caller fills a
    /// per-publisher inline slot, [`commit`](PublishLoan::commit) calls
    /// the backend's `publish_raw` (single memcpy into the backend's
    /// outbound buffer, same as `publish_raw` directly).
    #[cfg(not(feature = "rmw-lending"))]
    pub fn try_loan(&self, len: usize) -> Result<PublishLoan<'_, TX_BUF>, LoanError> {
        let slice = self.arena.try_claim(len)?;
        Ok(PublishLoan {
            publisher: self,
            slice,
            committed: false,
        })
    }

    /// `rmw-lending` variant — see the no-lending [`try_loan`] for the docs.
    #[cfg(feature = "rmw-lending")]
    pub fn try_loan(&self, len: usize) -> Result<PublishLoan<'_, TX_BUF>, LoanError> {
        use nros_rmw::SlotLending;
        match self.handle.try_lend_slot(len) {
            Ok(Some(slot)) => Ok(PublishLoan {
                publisher: self,
                backend_slot: Some(slot),
                committed: false,
            }),
            Ok(None) => Err(LoanError::WouldBlock),
            Err(e) => Err(LoanError::Backend(e)),
        }
    }

    /// Sync blocking loan with timeout. Spins the executor until the
    /// arena slot is free or `timeout` elapses.
    ///
    /// Useful when you publish from a sync context that owns the
    /// executor and want to block on a busy arena (rare — single-slot
    /// arena means contention only when concurrent task tries the same
    /// publisher, in which case the offending other task should have
    /// committed promptly).
    pub fn loan_with_timeout(
        &self,
        len: usize,
        executor: &mut super::Executor,
        timeout: core::time::Duration,
    ) -> Result<PublishLoan<'_, TX_BUF>, LoanError> {
        if len > TX_BUF {
            return Err(LoanError::TooLarge);
        }
        let spin_interval = core::time::Duration::from_millis(DEFAULT_SPIN_INTERVAL_MS);
        let timeout_ms = timeout.as_millis().min(u64::MAX as u128) as u64;
        let max_spins = (timeout_ms / DEFAULT_SPIN_INTERVAL_MS).max(1);
        let mut budget = WaitBudget::new(max_spins, timeout);
        loop {
            match self.try_loan(len) {
                Ok(loan) => return Ok(loan),
                Err(LoanError::WouldBlock) => {
                    executor.spin_once(spin_interval);
                    if !budget.tick() {
                        return Err(LoanError::WouldBlock);
                    }
                }
                Err(other) => return Err(other),
            }
        }
    }

    /// Async-await on a free loan slot. Returns the loan as soon as
    /// the arena's busy flag clears (no-lending path) or as soon as
    /// the backend's outbound stream has room (lending path).
    ///
    /// Phase 99.H': cancellation-safe Future. Registers the task's
    /// waker on the arena's [`AtomicWaker`] before checking
    /// [`try_loan`]; another task's `commit` / `discard` calls
    /// [`TxArena::release`] which wakes us. Dropping the future before
    /// it resolves removes nothing from any wait queue (single-slot
    /// AtomicWaker semantics: only the latest registration matters)
    /// and explicitly wakes another waiter so the next task in line
    /// gets a poll. No `PublishLoan` is materialised on cancel paths.
    pub fn loan(&self, len: usize) -> LoanFuture<'_, TX_BUF> {
        LoanFuture {
            publisher: self,
            len,
            registered: false,
        }
    }
}

/// Future returned by [`EmbeddedRawPublisher::loan`]. Phase 99.H'
/// cancellation-safe variant: if dropped before resolving, it wakes
/// the next pending waiter so the busy-flag-clear signal isn't lost
/// to the cancelled task.
#[must_use = "futures do nothing unless polled"]
pub struct LoanFuture<'a, const TX_BUF: usize> {
    publisher: &'a EmbeddedRawPublisher<TX_BUF>,
    len: usize,
    /// Set on the first `Pending` return so `Drop` knows whether a
    /// waker was registered (and thus another waiter may need a wake).
    registered: bool,
}

impl<'a, const TX_BUF: usize> core::future::Future for LoanFuture<'a, TX_BUF> {
    type Output = Result<PublishLoan<'a, TX_BUF>, LoanError>;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        // SAFETY: LoanFuture is `Unpin` for all practical purposes —
        // it holds only `&publisher`, `len`, and a bool. Move out of
        // Pin for the body.
        let this = self.get_mut();

        // Register-then-check: closes the race where another task's
        // `release` fires between `try_loan` returning WouldBlock and
        // the waker landing. The arena's AtomicWaker stores the
        // latest waker; we update it on every poll so a `select!` /
        // re-poll under a different waker observes the right one.
        loan_register_waker(this.publisher, cx.waker());
        this.registered = true;

        match this.publisher.try_loan(this.len) {
            Ok(loan) => core::task::Poll::Ready(Ok(loan)),
            Err(LoanError::WouldBlock) => core::task::Poll::Pending,
            Err(other) => core::task::Poll::Ready(Err(other)),
        }
    }
}

impl<'a, const TX_BUF: usize> Drop for LoanFuture<'a, TX_BUF> {
    fn drop(&mut self) {
        // If we registered a waker but never resolved, the busy flag
        // may have just cleared and we'd swallow the wake. Forward it
        // to the next waiter so the line keeps moving. Cheap no-op
        // when no one else is waiting.
        if self.registered {
            loan_wake_next(self.publisher);
        }
    }
}

// Indirection so the `rmw-lending` build (which has no arena) can
// stub these. With `rmw-lending` on, the lending Future variant uses
// the executor's drive_io spin to drain the backend stream — there's
// no arena-level wake source, so the helpers degrade to a self-wake.
#[cfg(not(feature = "rmw-lending"))]
fn loan_register_waker<const TX_BUF: usize>(
    pub_: &EmbeddedRawPublisher<TX_BUF>,
    waker: &core::task::Waker,
) {
    pub_.arena.waker.register(waker);
}

#[cfg(not(feature = "rmw-lending"))]
fn loan_wake_next<const TX_BUF: usize>(pub_: &EmbeddedRawPublisher<TX_BUF>) {
    pub_.arena.waker.wake();
}

#[cfg(feature = "rmw-lending")]
fn loan_register_waker<const TX_BUF: usize>(
    _pub_: &EmbeddedRawPublisher<TX_BUF>,
    waker: &core::task::Waker,
) {
    // No arena wake source under lending; self-wake so the runtime
    // re-polls after the next executor tick (which drains the
    // backend's outbound stream via `drive_io`).
    waker.wake_by_ref();
}

#[cfg(feature = "rmw-lending")]
fn loan_wake_next<const TX_BUF: usize>(_pub_: &EmbeddedRawPublisher<TX_BUF>) {
    // No-op: no AtomicWaker on the arena under the lending build.
}

/// Error type for [`EmbeddedRawPublisher::try_loan`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LoanError {
    /// Requested length exceeds the publisher's arena slot capacity.
    TooLarge,
    /// Arena slot already in use; another publish is in flight on this
    /// publisher. Retry after the other loan commits or discards.
    WouldBlock,
    /// Backend rejected the publish at commit time.
    Backend(TransportError),
}

impl From<TransportError> for LoanError {
    fn from(e: TransportError) -> Self {
        LoanError::Backend(e)
    }
}

/// Writable loan into a [`EmbeddedRawPublisher`]'s slot.
///
/// User fills `as_mut()` then calls [`commit`](Self::commit) to publish,
/// or [`discard`](Self::discard) to release the slot without publishing.
/// Dropping without either silently discards (slot freed); a
/// `#[must_use]` warning catches accidental drops at compile time.
///
/// Two backings, selected at compile time by the `rmw-lending` feature:
///
/// - **Arena (default)**: per-publisher inline `[u8; TX_BUF]` slot. On
///   commit, `publish_raw` memcpys into the backend's outbound buffer.
/// - **Backend lending (`rmw-lending`)**: slot owned by the backend
///   (zenoh-pico's static buffer aliased via `z_bytes_from_static_buf`,
///   XRCE's `ucdrBuffer` reservation). True zero-copy publish.
#[must_use = "PublishLoan must be committed or discarded; dropping silently rolls back"]
#[cfg(not(feature = "rmw-lending"))]
pub struct PublishLoan<'a, const TX_BUF: usize> {
    publisher: &'a EmbeddedRawPublisher<TX_BUF>,
    slice: &'a mut [u8],
    committed: bool,
}

#[must_use = "PublishLoan must be committed or discarded; dropping silently rolls back"]
#[cfg(feature = "rmw-lending")]
pub struct PublishLoan<'a, const TX_BUF: usize> {
    publisher: &'a EmbeddedRawPublisher<TX_BUF>,
    /// `Option` so `commit` can move the slot out via `take()` without
    /// triggering Drop's release path. Always `Some(_)` until `commit`
    /// or `discard` runs.
    backend_slot: Option<<session::RmwPublisher as nros_rmw::SlotLending>::Slot<'a>>,
    committed: bool,
}

#[cfg(not(feature = "rmw-lending"))]
impl<'a, const TX_BUF: usize> PublishLoan<'a, TX_BUF> {
    /// Mutable view into the loaned bytes. Caller writes message data here.
    #[allow(clippy::should_implement_trait)]
    pub fn as_mut(&mut self) -> &mut [u8] {
        self.slice
    }

    /// Commit the loan: hand the bytes to the backend's `publish_raw`,
    /// then release the arena slot. Returns the backend's publish error
    /// if any (slot is released regardless).
    pub fn commit(mut self) -> Result<(), LoanError> {
        let res = self
            .publisher
            .handle
            .publish_raw(self.slice)
            .map_err(|_| LoanError::Backend(TransportError::PublishFailed));
        self.committed = true;
        // Drop runs and releases the slot.
        res
    }

    /// Discard the loan without publishing. Equivalent to dropping, but
    /// explicit (no #[must_use] warning).
    pub fn discard(mut self) {
        self.committed = true; // Suppress Drop's "discard" log if any.
        drop(self);
    }
}

#[cfg(feature = "rmw-lending")]
impl<'a, const TX_BUF: usize> PublishLoan<'a, TX_BUF> {
    /// Mutable view into the backend-lent bytes.
    #[allow(clippy::should_implement_trait)]
    pub fn as_mut(&mut self) -> &mut [u8] {
        // SAFETY-invariant: `backend_slot` is `Some` for the whole life of
        // a `PublishLoan` — only `commit`/`discard` take it, and both
        // consume `self` by value, so no `&mut self` method can observe
        // `None`.
        self.backend_slot
            .as_mut()
            .expect("PublishLoan slot already consumed")
            .as_mut()
    }

    /// Commit the loan: hand the slot to the backend's `commit_slot` for
    /// flushing. The slot's bytes are written to the wire without an
    /// extra user-side memcpy.
    pub fn commit(mut self) -> Result<(), LoanError> {
        use nros_rmw::SlotLending;
        // SAFETY-invariant: first and only `take` — `commit` consumes
        // `self`, so the slot is still `Some` here.
        let slot = self
            .backend_slot
            .take()
            .expect("PublishLoan slot already consumed");
        self.committed = true;
        self.publisher
            .handle
            .commit_slot(slot)
            .map_err(LoanError::Backend)
    }

    /// Discard the loan without publishing. The backend-owned slot is
    /// released by its own Drop when this `PublishLoan` is dropped.
    pub fn discard(mut self) {
        self.committed = true;
        // backend_slot's Option<Slot> drops here, releasing the slot.
        drop(self.backend_slot.take());
    }
}

#[cfg(not(feature = "rmw-lending"))]
impl<'a, const TX_BUF: usize> Drop for PublishLoan<'a, TX_BUF> {
    fn drop(&mut self) {
        // Slot always returned to the free pool; whether the bytes were
        // actually published is encoded in `committed`. Future telemetry
        // hook could log uncommitted drops in debug builds.
        self.publisher.arena.release();
    }
}

// rmw-lending variant relies on the backend's `Slot` Drop impl to release
// the underlying buffer/stream slot. No explicit nros-side Drop needed.

// ============================================================================
// Subscription
// ============================================================================

/// Typed subscription handle with internal receive buffer.
///
/// Two methods, both byte-oriented at the wire:
///
/// - [`take`](Self::take) / [`recv`](Self::recv) — pull bytes
///   from the backend, CDR-decode into `M: RosMessage`, hand back
///   ownership of the typed message.
/// - [`take_serialized`](Self::take_serialized) — copy bytes into the
///   subscription's internal buffer and return the length, leaving CDR
///   decoding to the caller.
///
/// **No typed `borrow()` exists.** Borrow lives exclusively on
/// [`RawSubscription`]. `RecvView` is `&[u8]` semantics; CDR decoding
/// into a typed `M` requires owning the bytes (or running the decoder
/// in place), which the borrow contract doesn't fit. See
/// `docs/design/0010-zero-copy-raw-api.md` decision D7.
pub struct Subscription<M, const RX_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE }> {
    pub(crate) handle: session::RmwSubscriber,
    pub(crate) buffer: [u8; RX_BUF],
    /// Phase 108 — registered event closures.
    pub(crate) event_regs: EventRegs,
    /// W3b.5 — contracted-endpoint age hook (cell + epoch clock);
    /// `None` = uncontracted.
    pub(crate) age_mon: Option<crate::executor::arena::AgeMon>,
    pub(crate) _phantom: PhantomData<M>,
}

impl<M, const RX_BUF: usize> Drop for Subscription<M, RX_BUF> {
    fn drop(&mut self) {
        drop_event_regs(&mut self.event_regs);
    }
}

impl<M: RosMessage, const RX_BUF: usize> Subscription<M, RX_BUF> {
    /// Try to receive a typed message (non-blocking).
    /// Take one message if the middleware has one — phase-379 W3.
    ///
    /// Named `take` after rcl (`rcl_take`) and rclcpp (`Subscription::take`),
    /// which spell the non-blocking receive that way — as does our own C
    /// surface. NOT after rclrs: its subscription API is callback/worker-driven
    /// and has no public polling counterpart at all (its `take_*` methods are
    /// private), which is why the ledger records this as an `extension` against
    /// the Rust reference rather than a match. `take` was Rust
    /// channel vocabulary that reads as a different contract to a ROS 2 user:
    /// both are non-blocking and both report emptiness without failing, so
    /// nothing asked for the other word. Renamed as a clean break, no shim.
    pub fn take(&mut self) -> Result<Option<M>, NodeError> {
        match self
            .handle
            .take_serialized(&mut self.buffer)
            .map_err(NodeError::Transport)?
        {
            Some(len) => {
                // W3b.5 — record take-age for contracted endpoints.
                crate::executor::arena::observe_age::<M>(&self.buffer[..len], &self.age_mon);
                let mut reader = CdrReader::new_with_header(&self.buffer[..len])
                    .map_err(|_| NodeError::Transport(TransportError::DeserializationError))?;
                let msg = M::deserialize(&mut reader)
                    .map_err(|_| NodeError::Transport(TransportError::DeserializationError))?;
                Ok(Some(msg))
            }
            None => Ok(None),
        }
    }

    /// Try to receive raw CDR-encoded data (non-blocking).
    pub fn take_serialized(&mut self) -> Result<Option<usize>, NodeError> {
        self.handle
            .take_serialized(&mut self.buffer)
            .map_err(NodeError::Transport)
    }

    /// Get the receive buffer (valid after `take_serialized`).
    pub fn buffer(&self) -> &[u8] {
        &self.buffer
    }

    // ====================================================================
    // Phase 108 — status events
    // ====================================================================
    //
    // Subscriber-side: `LivelinessChanged`, `RequestedDeadlineMissed`,
    // `MessageLost`. Returns
    // `NodeError::Transport(TransportError::Unsupported)` if the
    // active backend doesn't generate the event for this entity.

    /// `true` if the active backend can fire the named event for this
    /// subscriber.
    #[cfg(feature = "alloc")]
    pub fn supports_event(&self, kind: nros_rmw::EventKind) -> bool {
        use nros_rmw::Subscription as _;
        self.handle.supports_event(kind)
    }

    /// Register a callback for `LivelinessChanged`. Fires when a
    /// tracked publisher's liveliness state changes.
    #[cfg(feature = "alloc")]
    pub fn on_liveliness_changed<F>(&mut self, cb: F) -> Result<(), NodeError>
    where
        F: FnMut(nros_rmw::LivelinessChangedStatus) + Send + 'static,
    {
        register_sub_event_liveliness::<F>(&mut self.handle, &mut self.event_regs, cb)
    }

    /// Register a callback for `RequestedDeadlineMissed`. Fires when
    /// an expected sample doesn't arrive within `deadline`.
    #[cfg(feature = "alloc")]
    pub fn on_requested_deadline_missed<F>(
        &mut self,
        deadline: core::time::Duration,
        cb: F,
    ) -> Result<(), NodeError>
    where
        F: FnMut(nros_rmw::CountStatus) + Send + 'static,
    {
        register_sub_event_count::<F, _>(
            &mut self.handle,
            &mut self.event_regs,
            nros_rmw::EventKind::RequestedDeadlineMissed,
            deadline.as_millis().min(u32::MAX as u128) as u32,
            cb,
            |payload, f| {
                if let nros_rmw::EventPayload::RequestedDeadlineMissed(s) = payload {
                    f(*s);
                }
            },
        )
    }

    /// Register a callback for `MessageLost`. Fires when the backend
    /// drops a sample (overflow, etc.).
    #[cfg(feature = "alloc")]
    pub fn on_message_lost<F>(&mut self, cb: F) -> Result<(), NodeError>
    where
        F: FnMut(nros_rmw::CountStatus) + Send + 'static,
    {
        register_sub_event_count::<F, _>(
            &mut self.handle,
            &mut self.event_regs,
            nros_rmw::EventKind::MessageLost,
            0,
            cb,
            |payload, f| {
                if let nros_rmw::EventPayload::MessageLost(s) = payload {
                    f(*s);
                }
            },
        )
    }

    /// Check if data is available without consuming it.
    pub fn has_data(&self) -> bool {
        self.handle.has_data()
    }

    /// Process the received message in-place without copying.
    pub fn process_in_place(&mut self, f: impl FnOnce(&M)) -> Result<bool, NodeError> {
        let mut deser_err = false;
        let processed = self
            .handle
            .process_raw_in_place(|raw| {
                match CdrReader::new_with_header(raw).and_then(|mut r| M::deserialize(&mut r)) {
                    Ok(msg) => f(&msg),
                    Err(_) => deser_err = true,
                }
            })
            .map_err(|_| NodeError::Transport(TransportError::DeserializationError))?;

        if deser_err {
            return Err(NodeError::Transport(TransportError::DeserializationError));
        }
        Ok(processed)
    }

    /// Async: wait for the next message (no `futures` dependency needed).
    ///
    /// Requires a background task running `executor.spin_async()` to drive
    /// I/O. Returns `Ok(msg)` on the next received message, or `Err` if the
    /// transport reports an error.
    ///
    /// When the `stream` feature is enabled, prefer `StreamExt::next()` /
    /// `TryStreamExt::try_next()` for combinator support.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let mut sub = node.create_subscription::<Int32>("/topic")?;
    /// loop {
    ///     let msg = sub.recv().await?;
    ///     /* handle msg */
    /// }
    /// ```
    pub async fn recv(&mut self) -> Result<M, NodeError> {
        core::future::poll_fn(|cx| {
            // Register the waker FIRST, then check for data. This ordering
            // closes the race window where a subscriber callback fires
            // between `take` returning `None` and the waker being
            // registered — the wake would otherwise be delivered to the
            // previous waker (or nowhere) and the task would hang.
            self.handle.register_waker(cx.waker());
            match self.take() {
                Ok(Some(msg)) => core::task::Poll::Ready(Ok(msg)),
                Ok(None) => core::task::Poll::Pending,
                Err(e) => core::task::Poll::Ready(Err(e)),
            }
        })
        .await
    }

    /// Sync: wait for the next message, spinning the executor.
    ///
    /// Returns `Ok(Some(msg))` if a message arrives within `timeout_ms`,
    /// or `Ok(None)` on timeout. Unlike [`Promise::wait()`], timeout is
    /// not an error — the caller typically retries in a loop.
    ///
    /// # Example
    ///
    /// ```ignore
    /// while let Some(msg) = sub.wait_next(&mut executor, core::time::Duration::from_millis(1000))? {
    ///     /* handle msg */
    /// }
    /// ```
    pub fn wait_next(
        &mut self,
        executor: &mut super::Executor,
        timeout: core::time::Duration,
    ) -> Result<Option<M>, NodeError> {
        let spin_interval = core::time::Duration::from_millis(DEFAULT_SPIN_INTERVAL_MS);
        let timeout_ms = timeout.as_millis().min(u64::MAX as u128) as u64;
        let max_spins = (timeout_ms / DEFAULT_SPIN_INTERVAL_MS).max(1);
        let mut budget = WaitBudget::new(max_spins, timeout);
        loop {
            executor.spin_once(spin_interval);
            if let Some(msg) = self.take()? {
                return Ok(Some(msg));
            }
            if !budget.tick() {
                return Ok(None);
            }
        }
    }
}

// ============================================================================
// RawSubscription — typeless subscription for non-ROS message wire formats
// ============================================================================

/// Typeless subscription handle. Counterpart of [`EmbeddedRawPublisher`].
///
/// The user owns the decoding step: call [`take_serialized`](Self::take_serialized)
/// to fill an internal buffer with bytes whose format depends on the active
/// RMW backend, then interpret them however is appropriate (memcpy, custom
/// parser, …).
pub struct RawSubscription<const RX_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE }> {
    pub(crate) handle: session::RmwSubscriber,
    pub(crate) buffer: [u8; RX_BUF],
    /// Phase 108 — registered event closures.
    pub(crate) event_regs: EventRegs,
}

impl<const RX_BUF: usize> Drop for RawSubscription<RX_BUF> {
    fn drop(&mut self) {
        drop_event_regs(&mut self.event_regs);
    }
}

impl<const RX_BUF: usize> RawSubscription<RX_BUF> {
    /// RFC-0088 — the serialization format of the bytes this subscription
    /// yields, as an image-local discriminant.
    ///
    /// A single-backend image knows this at compile time; the accessor exists
    /// for the case that does not, a bridge image built with
    /// `Executor::open_multi`, where two sessions speak two formats and the
    /// answer is per session rather than per image.
    pub const fn format(&self) -> nros_serdes::format::SerializationFormatId {
        crate::session::IMAGE_SERIALIZATION_FORMAT_ID
    }

    /// Construct a [`RawSubscription`] from a backend-allocated
    /// `RmwSubscriber` handle. Public so external extension crates
    /// (e.g. `nros-px4` for typed uORB wrappers) can wrap a handle
    /// they obtained directly from the active session via
    /// [`crate::Node::session_mut`] + a backend-specific create method.
    ///
    /// Most users should not call this — use
    /// [`crate::Node::create_subscription`] or
    /// [`crate::Node::create_subscription_raw`] instead.
    pub fn new(handle: session::RmwSubscriber) -> Self {
        Self {
            handle,
            buffer: [0u8; RX_BUF],
            event_regs: empty_event_regs(),
        }
    }

    /// Try to receive raw bytes (non-blocking). Returns `Ok(Some(len))`
    /// with the message length on success; the bytes live in
    /// [`buffer`](Self::buffer) until the next call.
    pub fn take_serialized(&mut self) -> Result<Option<usize>, NodeError> {
        self.handle
            .take_serialized(&mut self.buffer)
            .map_err(NodeError::Transport)
    }

    /// Phase 128.F.4 — raw receive that also surfaces the incoming
    /// sample's wire-level attachment block.
    ///
    /// Returns `Ok(Some((payload_len, attachment_len)))`. The payload
    /// lives in [`buffer`](Self::buffer); the attachment is written
    /// into caller-supplied `att_buf`. `attachment_len == 0` means
    /// the incoming sample carried no attachment.
    ///
    /// Backends without native attachment support delegate to
    /// [`take_serialized`](Self::take_serialized) and always report
    /// `attachment_len == 0` (default `Subscriber` trait body in
    /// `nros-rmw`).
    pub fn take_serialized_with_attachment(
        &mut self,
        att_buf: &mut [u8],
    ) -> Result<Option<(usize, usize)>, NodeError> {
        self.handle
            .take_serialized_with_attachment(&mut self.buffer, att_buf)
            .map_err(NodeError::Transport)
    }

    /// Phase 252 / issue 0073 — raw receive that also returns the E2E
    /// [`IntegrityStatus`](nros_rmw::IntegrityStatus) (CRC + sequence gap/dup) for
    /// the C/C++ `nros_subscription_take_validated` path. The validator lives
    /// in the backend handle (`take_validated`), so no typed message is needed;
    /// the payload lives in [`buffer`](Self::buffer). `crc_valid == None` when the
    /// wire sample carried no CRC (e.g. a publisher built without `safety-e2e`).
    #[cfg(feature = "safety-e2e")]
    pub fn take_validated(
        &mut self,
    ) -> Result<Option<(usize, nros_rmw::IntegrityStatus)>, NodeError> {
        use nros_rmw::Subscription as _;
        self.handle
            .take_validated(&mut self.buffer)
            .map_err(NodeError::Transport)
    }

    /// Phase 124.D.1 — burst-take. Drain up to `max_msgs` queued
    /// samples into the caller-supplied contiguous block in one
    /// call, with the i-th sample at
    /// `buf[i * per_msg_cap .. i * per_msg_cap + out_lens[i]]`.
    /// Returns the number of messages delivered.
    ///
    /// Backends without a native batch take inherit the
    /// `Subscriber::take_sequence` default body which loop-drives
    /// `take_serialized` — same shape, same observable result; the
    /// batched API just lets sensor loops commit to the call shape
    /// regardless of backend support.
    pub fn take_sequence(
        &mut self,
        buf: &mut [u8],
        per_msg_cap: usize,
        max_msgs: usize,
        out_lens: &mut [usize],
    ) -> Result<usize, NodeError> {
        use nros_rmw::Subscription as _;
        self.handle
            .take_sequence(buf, per_msg_cap, max_msgs, out_lens)
            .map_err(NodeError::Transport)
    }

    /// Phase 122.3.c.6.e — register a `Waker` that fires when a new
    /// message arrives. Mirror of the existing service-server /
    /// service-client wake plumbing. No-op on backends that don't
    /// support waking — caller falls back to polling.
    pub fn register_waker(&self, waker: &core::task::Waker) {
        use nros_rmw::Subscription as _;
        self.handle.register_waker(waker);
    }

    /// Phase 108.A — `true` if the active backend can fire the named
    /// event for this raw subscription.
    #[cfg(feature = "alloc")]
    pub fn supports_event(&self, kind: nros_rmw::EventKind) -> bool {
        use nros_rmw::Subscription as _;
        self.handle.supports_event(kind)
    }

    /// Phase 108.A — register a callback for `LivelinessChanged`.
    #[cfg(feature = "alloc")]
    pub fn on_liveliness_changed<F>(&mut self, cb: F) -> Result<(), NodeError>
    where
        F: FnMut(nros_rmw::LivelinessChangedStatus) + Send + 'static,
    {
        register_sub_event_liveliness::<F>(&mut self.handle, &mut self.event_regs, cb)
    }

    /// Phase 108.A — register a callback for `RequestedDeadlineMissed`.
    #[cfg(feature = "alloc")]
    pub fn on_requested_deadline_missed<F>(
        &mut self,
        deadline: core::time::Duration,
        cb: F,
    ) -> Result<(), NodeError>
    where
        F: FnMut(nros_rmw::CountStatus) + Send + 'static,
    {
        register_sub_event_count::<F, _>(
            &mut self.handle,
            &mut self.event_regs,
            nros_rmw::EventKind::RequestedDeadlineMissed,
            deadline.as_millis().min(u32::MAX as u128) as u32,
            cb,
            |payload, f| {
                if let nros_rmw::EventPayload::RequestedDeadlineMissed(s) = payload {
                    f(*s);
                }
            },
        )
    }

    /// Phase 108.A — register a callback for `MessageLost`.
    #[cfg(feature = "alloc")]
    pub fn on_message_lost<F>(&mut self, cb: F) -> Result<(), NodeError>
    where
        F: FnMut(nros_rmw::CountStatus) + Send + 'static,
    {
        register_sub_event_count::<F, _>(
            &mut self.handle,
            &mut self.event_regs,
            nros_rmw::EventKind::MessageLost,
            0,
            cb,
            |payload, f| {
                if let nros_rmw::EventPayload::MessageLost(s) = payload {
                    f(*s);
                }
            },
        )
    }

    /// Get the receive buffer (valid after [`take_serialized`](Self::take_serialized)).
    pub fn buffer(&self) -> &[u8] {
        &self.buffer
    }

    /// Check if data is available without consuming it.
    pub fn has_data(&self) -> bool {
        self.handle.has_data()
    }

    /// Try to borrow the next available message in place. Returns
    /// `Ok(None)` if no message is ready; never blocks.
    ///
    /// The returned [`RecvView`] borrows the subscriber's internal
    /// receive buffer. Lifetime is tied to `&mut self` — only one view
    /// can be live at a time, and the next `try_borrow` / `take_serialized`
    /// call invalidates the previous view's bytes.
    ///
    /// View is `!Send + !Sync` to discourage holding it across `.await`
    /// or thread boundaries (would block subsequent receives on the
    /// same subscriber).
    #[cfg(not(feature = "rmw-lending"))]
    pub fn try_borrow(&mut self) -> Result<Option<RecvView<'_>>, NodeError> {
        match self.take_serialized()? {
            Some(len) => Ok(Some(RecvView {
                bytes: &self.buffer[..len],
                _marker: core::marker::PhantomData,
            })),
            None => Ok(None),
        }
    }

    /// `rmw-lending` variant — dispatches to the backend's
    /// [`SlotBorrowing::try_borrow`](nros_rmw::SlotBorrowing::try_borrow)
    /// for true zero-copy receive (zenoh-pico's static buffer borrowed
    /// directly via `z_bytes_get_contiguous_view`, XRCE's slot borrowed
    /// in place). The bytes never touch `self.buffer`.
    #[cfg(feature = "rmw-lending")]
    pub fn try_borrow(&mut self) -> Result<Option<RecvView<'_>>, NodeError> {
        use nros_rmw::SlotBorrowing;
        match self.handle.try_borrow() {
            Ok(Some(view)) => Ok(Some(RecvView {
                view: Some(view),
                _marker: core::marker::PhantomData,
            })),
            Ok(None) => Ok(None),
            Err(e) => Err(NodeError::Transport(e)),
        }
    }

    /// Async-await on the next message, returning a [`RecvView`].
    /// Mirrors the `Subscription::recv` pattern but typeless.
    ///
    /// Backend wake source: `Subscriber::register_waker`. Same race-
    /// safe register-then-check ordering as `Subscription::recv`.
    pub async fn borrow(&mut self) -> Result<RecvView<'_>, NodeError> {
        // Wait for `has_data` to flip true via the backend's
        // AtomicWaker, *without* holding any borrow that `try_borrow`
        // would need afterwards. Borrow `&self.handle` immutably
        // inside poll_fn so the borrow checker can prove `&mut self`
        // is free by the time we return Ok(view) below.
        //
        // Phase 99.H' cancellation safety: there is no reservation
        // taken inside poll. Dropping the future before it resolves
        // simply abandons whatever waker registration the backend
        // accepted; the next call to `borrow().await` (or
        // `try_borrow`) re-registers. No leaked state.
        {
            let handle = &self.handle;
            core::future::poll_fn(|cx| {
                // Register-then-check: closes the race where a backend
                // callback fires between has_data returning false and
                // the waker landing.
                handle.register_waker(cx.waker());
                if handle.has_data() {
                    core::task::Poll::Ready(())
                } else {
                    core::task::Poll::Pending
                }
            })
            .await;
        }
        // has_data was true at some point; in the single-threaded
        // executor there's no other reader, so try_borrow returns Some.
        // A spurious wake (very unlikely on shipping backends) returns
        // WouldBlock and the caller can retry.
        match self.try_borrow()? {
            Some(view) => Ok(view),
            None => Err(NodeError::Transport(TransportError::WouldBlock)),
        }
    }

    /// Sync blocking borrow with timeout. Spins the executor until a
    /// message is available or `timeout` elapses.
    ///
    /// Returns `Ok(Some(view))` on success, `Ok(None)` on timeout.
    /// The view's lifetime is tied to `&mut self`.
    pub fn borrow_with_timeout(
        &mut self,
        executor: &mut super::Executor,
        timeout: core::time::Duration,
    ) -> Result<Option<RecvView<'_>>, NodeError> {
        let spin_interval = core::time::Duration::from_millis(DEFAULT_SPIN_INTERVAL_MS);
        let timeout_ms = timeout.as_millis().min(u64::MAX as u128) as u64;
        let max_spins = (timeout_ms / DEFAULT_SPIN_INTERVAL_MS).max(1);
        let mut budget = WaitBudget::new(max_spins, timeout);
        loop {
            executor.spin_once(spin_interval);
            if self.has_data() {
                return self.try_borrow();
            }
            if !budget.tick() {
                return Ok(None);
            }
        }
    }
}

// ============================================================================
// RawServiceServer / RawServiceClient (Phase 122.3.c — L1 polling, typeless)
// ============================================================================

/// Typeless service-server handle. L1 counterpart of
/// [`EmbeddedServiceServer`] for callers that own their own scheduler
/// (RTIC, embassy, FreeRTOS-task-per-entity) and the C / C++ FFI
/// shims.
///
/// Holds the transport handle plus an inline request buffer. The
/// caller polls [`take_request_raw`](Self::take_request_raw)
/// and sends replies via [`send_response_raw`](Self::send_response_raw)
/// with raw CDR bytes.
pub struct RawServiceServer<
    const REQ_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const RESP_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    pub(crate) handle: session::RmwServiceServer,
    pub(crate) req_buffer: [u8; REQ_BUF],
    pub(crate) _phantom_resp: PhantomData<[u8; RESP_BUF]>,
}

impl<const REQ_BUF: usize, const RESP_BUF: usize> RawServiceServer<REQ_BUF, RESP_BUF> {
    /// Phase 122.3.c.6.e — register a `Waker` that fires when a new
    /// request arrives. Mirror of the existing subscriber /
    /// service-client wake plumbing. No-op on backends that don't
    /// support waking — caller falls back to polling.
    pub fn register_waker(&self, waker: &core::task::Waker) {
        use nros_rmw::ServiceTrait;
        self.handle.register_waker(waker);
    }

    /// Construct a [`RawServiceServer`] from a backend-allocated
    /// `RmwServiceServer` handle. Public so external crates and the
    /// C / C++ FFI shims can wrap a handle obtained directly from
    /// [`crate::Node::session_mut`].
    pub fn new(handle: session::RmwServiceServer) -> Self {
        Self {
            handle,
            req_buffer: [0u8; REQ_BUF],
            _phantom_resp: PhantomData,
        }
    }

    /// Try to receive a service request (non-blocking).
    ///
    /// Returns `Ok(Some((len, sequence_number)))` when a request is
    /// available — the raw CDR bytes live in
    /// [`req_buffer`](Self::req_buffer) at `&req_buffer()[..len]`
    /// until the next call. The sequence number is required by
    /// [`send_response_raw`](Self::send_response_raw).
    pub fn take_request_raw(&mut self) -> Result<Option<(usize, i64)>, NodeError> {
        match self.handle.take_request(&mut self.req_buffer) {
            Ok(Some(req)) => Ok(Some((req.data.len(), req.sequence_number))),
            Ok(None) => Ok(None),
            Err(_) => Err(NodeError::Transport(TransportError::ServiceRequestFailed)),
        }
    }

    /// Borrow the inline request buffer. Valid after a successful
    /// [`take_request_raw`](Self::take_request_raw) call.
    pub fn req_buffer(&self) -> &[u8] {
        &self.req_buffer
    }

    /// Send a reply with raw CDR bytes. `sequence_number` must match
    /// the value returned by the most recent
    /// [`take_request_raw`](Self::take_request_raw).
    pub fn send_response_raw(
        &mut self,
        sequence_number: i64,
        data: &[u8],
    ) -> Result<(), NodeError> {
        self.handle
            .send_response(sequence_number, data)
            .map_err(|_| NodeError::ServiceReplyFailed)
    }
}

/// Typeless service-client handle. L1 counterpart of
/// [`EmbeddedServiceClient`] for the same audience as
/// [`RawServiceServer`].
pub struct RawServiceClient<
    const REQ_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const REPLY_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    pub(crate) handle: session::RmwServiceClient,
    pub(crate) reply_buffer: [u8; REPLY_BUF],
    pub(crate) _phantom_req: PhantomData<[u8; REQ_BUF]>,
}

impl<const REQ_BUF: usize, const REPLY_BUF: usize> RawServiceClient<REQ_BUF, REPLY_BUF> {
    /// Phase 122.3.c.6.e — register a `Waker` that fires when the
    /// reply to a previously-sent request lands.
    pub fn register_waker(&self, waker: &core::task::Waker) {
        use nros_rmw::ClientTrait;
        self.handle.register_waker(waker);
    }

    /// Construct from a backend-allocated handle. Same audience as
    /// [`RawServiceServer::new`].
    pub fn new(handle: session::RmwServiceClient) -> Self {
        Self {
            handle,
            reply_buffer: [0u8; REPLY_BUF],
            _phantom_req: PhantomData,
        }
    }

    /// Send a raw CDR request. Non-blocking; the reply arrives via
    /// [`take_response_raw`](Self::take_response_raw).
    pub fn send_request_raw(&mut self, request: &[u8]) -> Result<(), NodeError> {
        // Issue 0778 — the backend now hands back a sequence id. This raw
        // handle keeps one call in flight (`in_flight_flag`), so it has no use
        // for the id yet and drops it explicitly rather than by omission.
        self.handle
            .send_request_raw(request)
            .map(|_seq| ())
            .map_err(|_| NodeError::ServiceRequestFailed)
    }

    /// Phase 124.G.3 — graph-aware "is the matching server up?"
    /// probe. Mirrors [`Client::server_available`] for the raw API.
    pub fn service_is_ready(&self) -> Result<bool, NodeError> {
        use nros_rmw::ClientTrait;
        self.handle.service_is_ready().map_err(NodeError::Transport)
    }

    /// Try to receive a reply (non-blocking). Returns
    /// `Ok(Some(len))` with the reply length on success; bytes
    /// live in [`reply_buffer`](Self::reply_buffer) until the next
    /// call.
    pub fn take_response_raw(&mut self) -> Result<Option<usize>, NodeError> {
        self.handle
            .take_response_raw(&mut self.reply_buffer)
            .map(|opt| opt.map(|(len, _seq)| len))
            .map_err(|_| NodeError::Transport(TransportError::ServiceRequestFailed))
    }

    /// Borrow the inline reply buffer. Valid after a successful
    /// [`take_response_raw`](Self::take_response_raw) call.
    pub fn reply_buffer(&self) -> &[u8] {
        &self.reply_buffer
    }
}

/// Read-only view into a [`RawSubscription`]'s receive buffer.
///
/// `!Send + !Sync`: cannot cross `.await` or threads. Drop releases
/// any backend lock + lets the next message advance.
///
/// Two backings, selected at compile time by the `rmw-lending` feature:
/// the no-lending variant points at `RawSubscription::buffer` (filled by
/// `take_serialized`'s memcpy); the lending variant holds the backend's
/// own [`SlotBorrowing::View`](nros_rmw::SlotBorrowing::View) — zero
/// copies on the receive path, with the backend's Drop taking care of
/// releasing the buffer lock.
#[cfg(not(feature = "rmw-lending"))]
pub struct RecvView<'a> {
    bytes: &'a [u8],
    _marker: core::marker::PhantomData<*const ()>,
}

#[cfg(feature = "rmw-lending")]
pub struct RecvView<'a> {
    /// `Option` for symmetry with `PublishLoan::backend_slot`. Always
    /// `Some(_)` until the view is dropped.
    view: Option<<session::RmwSubscriber as nros_rmw::SlotBorrowing>::View<'a>>,
    _marker: core::marker::PhantomData<*const ()>,
}

#[cfg(not(feature = "rmw-lending"))]
impl<'a> core::ops::Deref for RecvView<'a> {
    type Target = [u8];
    fn deref(&self) -> &[u8] {
        self.bytes
    }
}

#[cfg(feature = "rmw-lending")]
impl<'a> core::ops::Deref for RecvView<'a> {
    type Target = [u8];
    fn deref(&self) -> &[u8] {
        // SAFETY-invariant: `view` is `Some` for the whole life of a
        // `RecvView` — only `Drop` takes it, after which no method (incl.
        // this `deref`) is reachable.
        self.view
            .as_ref()
            .expect("RecvView accessed after drop")
            .as_ref()
    }
}

#[cfg(not(feature = "rmw-lending"))]
impl<'a> AsRef<[u8]> for RecvView<'a> {
    fn as_ref(&self) -> &[u8] {
        self.bytes
    }
}

#[cfg(feature = "rmw-lending")]
impl<'a> AsRef<[u8]> for RecvView<'a> {
    fn as_ref(&self) -> &[u8] {
        // SAFETY-invariant: `view` is `Some` until `Drop`; see the `Deref`
        // impl above.
        self.view
            .as_ref()
            .expect("RecvView accessed after drop")
            .as_ref()
    }
}

#[cfg(feature = "stream")]
impl<M: RosMessage + Unpin, const RX_BUF: usize> futures_core::Stream for Subscription<M, RX_BUF> {
    type Item = Result<M, NodeError>;

    fn poll_next(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Option<Self::Item>> {
        let this = self.get_mut();
        // Register-then-check: see Subscription::recv for rationale.
        this.handle.register_waker(cx.waker());
        match this.take() {
            Ok(Some(msg)) => core::task::Poll::Ready(Some(Ok(msg))),
            Ok(None) => core::task::Poll::Pending,
            Err(e) => core::task::Poll::Ready(Some(Err(e))),
        }
    }
}

// ============================================================================
// EmbeddedServiceServer
// ============================================================================

/// Typed service server handle with internal buffers.
pub struct EmbeddedServiceServer<
    Svc: RosService,
    const REQ_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const REPLY_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    pub(crate) handle: session::RmwServiceServer,
    pub(crate) req_buffer: [u8; REQ_BUF],
    pub(crate) reply_buffer: [u8; REPLY_BUF],
    pub(crate) _phantom: PhantomData<Svc>,
}

impl<Svc: RosService, const REQ_BUF: usize, const REPLY_BUF: usize>
    EmbeddedServiceServer<Svc, REQ_BUF, REPLY_BUF>
{
    /// Handle an incoming service request.
    ///
    /// Returns `Ok(true)` if a request was handled, `Ok(false)` if none available.
    pub fn handle_request(
        &mut self,
        handler: impl FnOnce(&Svc::Request) -> Svc::Reply,
    ) -> Result<bool, NodeError> {
        self.handle
            .handle_request::<Svc>(&mut self.req_buffer, &mut self.reply_buffer, handler)
            .map_err(|_| NodeError::ServiceReplyFailed)
    }

    /// Handle a request with a heap-allocated reply (for large response types).
    ///
    /// Used by parameter services and lifecycle services (large response structs
    /// that overflow the stack). Returns `Ok(true)` if a request was handled,
    /// `Ok(false)` if none available.
    #[cfg(any(feature = "param-services", feature = "lifecycle-services"))]
    pub fn handle_request_boxed(
        &mut self,
        handler: impl FnOnce(&Svc::Request) -> alloc::boxed::Box<Svc::Reply>,
    ) -> Result<bool, NodeError> {
        self.handle
            .handle_request_boxed::<Svc>(&mut self.req_buffer, &mut self.reply_buffer, handler)
            .map_err(|_| NodeError::ServiceReplyFailed)
    }

    /// Handle a request by STREAMING it — the handler reads the request's fields
    /// off the wire and writes the reply's fields straight back, so neither is
    /// ever materialised as a value.
    ///
    /// phase-382 W1'. This is what `handle_request_boxed` should have been: that
    /// one boxes the reply and leaves the REQUEST as a stack local, which is how
    /// `ros2 param set` came to put 1.19 MB on the calling task's stack. Streaming
    /// removes both, and needs no allocator — see
    /// `ServiceTrait::handle_request_raw` for why it is byte-identical to the
    /// generated `Serialize` impls, and for the drift risk it carries.
    #[cfg(any(feature = "param-services", feature = "lifecycle-services"))]
    pub fn handle_request_raw(
        &mut self,
        handler: impl FnOnce(
            &mut nros_core::CdrReader<'_>,
            &mut nros_core::CdrWriter<'_>,
        ) -> Result<(), nros_rmw::TransportError>,
    ) -> Result<bool, NodeError> {
        self.handle
            .handle_request_raw(&mut self.req_buffer, &mut self.reply_buffer, handler)
            .map_err(|_| NodeError::ServiceReplyFailed)
    }

    /// Check if a request is available.
    pub fn has_request(&self) -> bool {
        self.handle.has_request()
    }
}

// ============================================================================
// ServiceClientCallback (RFC-0041, Phase 239.1)
// ============================================================================

/// Send handle for a **callback-based** typed service client.
///
/// Returned by `create_client_with_callback`: the reply is delivered to the
/// registered closure at `spin_once` (no `Promise` poll). This handle only
/// **sends** — it holds a `*mut` to the arena entry's
/// [`ServiceClientSendHeader`](super::arena::ServiceClientSendHeader) (pinned in
/// the executor arena, like a guard-condition flag), so a single outstanding
/// request is gated by `hdr.pending`.
///
/// # Safety / lifetime
/// Valid only while the owning executor lives (the arena backs the header). Do
/// not use after the executor is dropped.
pub struct ServiceClientCallback<
    Svc: RosService,
    const REQ_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const REPLY_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    hdr: *mut super::arena::ServiceClientSendHeader<REPLY_BUF>,
    _phantom: PhantomData<Svc>,
}

impl<Svc: RosService, const REQ_BUF: usize, const REPLY_BUF: usize>
    ServiceClientCallback<Svc, REQ_BUF, REPLY_BUF>
{
    /// Wrap an arena-resident send header. `hdr` must point at a live
    /// `ServiceClientCallbackEntry`'s header for the executor's lifetime.
    pub(crate) fn new(hdr: *mut super::arena::ServiceClientSendHeader<REPLY_BUF>) -> Self {
        Self {
            hdr,
            _phantom: PhantomData,
        }
    }

    /// Send a typed request. The reply is delivered to the registered callback
    /// at a later `spin_once`. Returns `RequestInFlight` if a prior request has
    /// not yet been answered (single outstanding request).
    pub fn call(&mut self, request: &Svc::Request) -> Result<(), NodeError> {
        let hdr = unsafe { &mut *self.hdr };
        if hdr.pending {
            return Err(NodeError::RequestInFlight);
        }
        let mut buf = [0u8; REQ_BUF];
        let mut writer = crate::tx_writer(&mut buf).map_err(|_| NodeError::BufferTooSmall)?;
        request
            .serialize(&mut writer)
            .map_err(|_| NodeError::Serialization)?;
        let req_len = writer.position();
        hdr.handle
            .send_request_raw(&buf[..req_len])
            .map_err(|_| NodeError::ServiceRequestFailed)?;
        hdr.pending = true;
        Ok(())
    }
}

// ============================================================================
// ActionClientCallback (RFC-0041, Phase 239.2)
// ============================================================================

/// Send handle for a **callback-based** typed action client.
///
/// Returned by `create_action_client_with_callbacks`: goal-response, feedback,
/// and result are delivered to the registered closures at `spin_once` (no
/// `Promise` poll). This handle only **sends** — it holds a `*mut` to the arena
/// entry's [`ActionClientCore`](super::action_core::ActionClientCore) (offset 0,
/// pinned in the executor arena, like a guard-condition flag).
///
/// # Safety / lifetime
/// Valid only while the owning executor lives.
pub struct ActionClientCallback<
    A: RosAction,
    const GOAL_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const RESULT_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const FEEDBACK_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    core: *mut super::action_core::ActionClientCore<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>,
    _phantom: PhantomData<A>,
}

impl<A: RosAction, const GOAL_BUF: usize, const RESULT_BUF: usize, const FEEDBACK_BUF: usize>
    ActionClientCallback<A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>
{
    /// Wrap an arena-resident core. `core` must point at a live
    /// `ActionClientCallbackEntry`'s core for the executor's lifetime.
    pub(crate) fn new(
        core: *mut super::action_core::ActionClientCore<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>,
    ) -> Self {
        Self {
            core,
            _phantom: PhantomData,
        }
    }

    /// Send a typed goal. Returns its `GoalId`; acceptance arrives via the
    /// registered goal-response callback.
    pub fn send_goal(&mut self, goal: &A::Goal) -> Result<nros_core::GoalId, NodeError> {
        let core = unsafe { &mut *self.core };
        let mut buf = [0u8; GOAL_BUF];
        let mut writer = crate::tx_writer(&mut buf).map_err(|_| NodeError::BufferTooSmall)?;
        goal.serialize(&mut writer)
            .map_err(|_| NodeError::Serialization)?;
        let len = writer.position();
        core.send_goal_raw(&buf[..len])
    }

    /// Request the result for `goal_id`; the result arrives via the registered
    /// result callback.
    pub fn get_result(&mut self, goal_id: &nros_core::GoalId) -> Result<(), NodeError> {
        let core = unsafe { &mut *self.core };
        core.send_get_result_request(goal_id)
    }
}

// ============================================================================
// EmbeddedServiceClient
// ============================================================================

/// Typed service client handle with internal buffers.
pub struct EmbeddedServiceClient<
    Svc: RosService,
    const REQ_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const REPLY_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    pub(crate) handle: session::RmwServiceClient,
    pub(crate) req_buffer: [u8; REQ_BUF],
    pub(crate) reply_buffer: [u8; REPLY_BUF],
    /// Phase 84.D3: set after a successful `send_request`, cleared on a
    /// successful `Promise::take`. Guards against "drop Promise
    /// without awaiting, then `call()` again" which would otherwise
    /// deliver the stale reply to the new caller.
    pub(crate) in_flight: bool,
    pub(crate) _phantom: PhantomData<Svc>,
}

impl<Svc: RosService, const REQ_BUF: usize, const REPLY_BUF: usize>
    EmbeddedServiceClient<Svc, REQ_BUF, REPLY_BUF>
{
    /// Call the service (non-blocking). Returns a [`Promise`] that can be polled.
    ///
    /// Use with `Executor::spin_once()` to drive I/O while waiting:
    ///
    /// ```ignore
    /// let mut promise = client.call(&request)?;
    /// loop {
    ///     executor.spin_once(core::time::Duration::from_millis(10));
    ///     if let Some(reply) = promise.take()? {
    ///         break;
    ///     }
    /// }
    /// ```
    ///
    /// # Errors
    ///
    /// Returns [`NodeError::RequestInFlight`] if a previous call's reply
    /// has not been received. This prevents the old hazard where dropping
    /// a [`Promise`] without awaiting its reply left the stale reply
    /// queued to land on the next [`call`](Self::call). Resolve by
    /// polling the existing promise to completion or calling
    /// [`reset_in_flight`](Self::reset_in_flight).
    pub fn call(&mut self, request: &Svc::Request) -> Result<Promise<'_, Svc::Reply>, NodeError> {
        if self.in_flight {
            return Err(NodeError::RequestInFlight);
        }

        // Serialize request into req_buffer
        let mut writer =
            crate::tx_writer(&mut self.req_buffer).map_err(|_| NodeError::BufferTooSmall)?;
        request
            .serialize(&mut writer)
            .map_err(|_| NodeError::Serialization)?;
        let req_len = writer.position();

        // Send the request (non-blocking)
        self.handle
            .send_request_raw(&self.req_buffer[..req_len])
            .map_err(|_| NodeError::ServiceRequestFailed)?;

        self.in_flight = true;

        Ok(Promise {
            handle: &mut self.handle,
            reply_buffer: &mut self.reply_buffer,
            parse: cdr_deserialize_reply::<Svc>,
            in_flight_flag: &mut self.in_flight,
        })
    }

    /// Explicitly clear the in-flight flag (Phase 84.D3).
    ///
    /// Call this if a previous [`Promise`] was dropped without completing
    /// and you want to abandon the pending reply. The next
    /// [`call`](Self::call) will proceed but may still observe the stale
    /// reply if one is in the transport's queue — callers that need strict
    /// correctness should drain / ignore one extra `take` first.
    pub fn reset_in_flight(&mut self) {
        self.in_flight = false;
    }

    /// Block until at least one matching service server is discoverable on
    /// the network, or `timeout` elapses.
    ///
    /// Returns `Ok(true)` if a matching server reported back inside the
    /// budget; `Ok(false)` on timeout (no server visible). Mirrors
    /// `rclcpp::ClientBase::wait_for_service` and
    /// `rclpy.client.Client.wait_for_service`.
    ///
    /// On the Zenoh backend this issues a `z_liveliness_get` against the
    /// matching server's wildcarded liveliness keyexpr; the executor is
    /// spun cooperatively while the query is in flight so other
    /// subscribers / timers continue to make progress.
    ///
    /// issue 1087 — a backend without liveliness discovery answers `Ok(None)`
    /// ("cannot say"), NOT `Ok(true)`. It used to answer yes, so this returned
    /// immediately on cyclone, XRCE and uORB without probing anything. Such a
    /// backend now waits out the budget and reports `Ok(false)`: slower, and
    /// the direction that does not send a request into the void.
    ///
    /// Recommended usage — gate the first `call()` on this:
    ///
    /// ```ignore
    /// let mut client = node.create_client::<AddTwoInts>("/add_two_ints")?;
    /// if !client.wait_for_service(&mut executor, Duration::from_secs(5))? {
    ///     return Err(NodeError::Timeout);
    /// }
    /// let mut promise = client.call(&request)?;
    /// ```
    ///
    /// Once the server is observed, the result is latched: subsequent
    /// `service_is_ready` checks return `true` without another round
    /// trip. This matches `rclcpp`'s snapshot semantic — discovery isn't
    /// re-proven on every call.
    pub fn wait_for_service(
        &mut self,
        executor: &mut super::Executor,
        timeout: core::time::Duration,
    ) -> Result<bool, NodeError> {
        // Already proven once — don't re-query. `Ok(true)` ONLY: issue 1008.
        //
        // This read `is_server_ready()`, whose trait default is `true` and which
        // only zenoh overrode — so on every cffi-backed image this fast path
        // fired unconditionally and `wait_for_service` returned `Ok(true)`
        // without waiting or probing. `Err` (backend cannot answer) and
        // `Ok(false)` must both fall through to the wait loop, which is what the
        // comment above always claimed.
        if matches!(self.handle.service_is_ready(), Ok(true)) {
            return Ok(true);
        }
        let spin_interval = core::time::Duration::from_millis(DEFAULT_SPIN_INTERVAL_MS);
        let max_spins = (timeout.as_millis() as u64 / DEFAULT_SPIN_INTERVAL_MS).max(1);
        let mut budget = WaitBudget::new(max_spins, timeout);
        // Per-query budget. A liveliness_get is a single-shot probe of the
        // router's current token list; if the server hasn't declared its
        // token yet when our query arrives, the router replies "no
        // matching tokens" and the query terminates. We loop, re-issuing
        // shorter probes until either a matching token is observed or the
        // outer wall-clock budget expires (issue #224 — shared cadence).
        const PROBE_TIMEOUT_MS: u32 = crate::SERVER_DISCOVERY_PROBE_TIMEOUT_MS;
        loop {
            self.handle
                .start_server_discovery(PROBE_TIMEOUT_MS)
                .map_err(|_| NodeError::ServiceRequestFailed)?;
            // Drain this probe to completion (token reply or empty FINAL).
            loop {
                executor.spin_once(spin_interval);
                match self
                    .handle
                    .poll_server_discovery()
                    .map_err(|_| NodeError::ServiceRequestFailed)?
                {
                    Some(true) => return Ok(true),
                    Some(false) => break, // probe finished empty — re-issue
                    None => {}            // still in flight
                }
                if !budget.tick() {
                    return Ok(false);
                }
            }
            if !budget.tick() {
                return Ok(false);
            }
        }
    }

    // phase-379 W6 decision 2 — the bool-returning `service_is_ready` was
    // DELETED here. It forwarded to `ClientTrait::is_server_ready`, whose trait
    // default is `true` and which only zenoh overrode, so on every cffi-backed
    // image (cyclonedds, XRCE, uORB) it answered "ready" without asking anything
    // — issue 1008. The `Result` form below is the same query with rcl's two
    // channels kept, and it is now the only one.

    /// Phase 124.C.3 — graph-aware server-availability probe.
    ///
    /// Returns `Ok(true)` / `Ok(false)` when the backend can answer
    /// (zenoh queryable interest, DDS built-in topic reader), or
    /// `Err(NodeError::Transport(Unsupported))` when it can't (XRCE
    /// agent without participant enumeration). Distinct from
    /// rclcpp collapses this to a bare `bool` and throws on error;
    /// RFC-0018 forbids exceptions, so the `Result` carries what the
    /// exception would have (phase-379 W6).
    ///
    /// Used to gate the first request so a startup-ordering race
    /// (client opens before server's discovery announcement lands)
    /// doesn't surface as a request-side timeout.
    pub fn service_is_ready(&self) -> Result<bool, NodeError> {
        use nros_rmw::ClientTrait;
        self.handle.service_is_ready().map_err(NodeError::Transport)
    }
}

// ============================================================================
// Promise
// ============================================================================

/// A pending reply from a non-blocking service or action call.
///
/// Poll with [`take()`](Promise::take) to check for the reply.
/// Implements [`Future`](core::future::Future) for use with async executors.
pub struct Promise<'a, T> {
    pub(crate) handle: &'a mut session::RmwServiceClient,
    pub(crate) reply_buffer: &'a mut [u8],
    pub(crate) parse: fn(&[u8]) -> Result<T, NodeError>,
    /// Phase 84.D3: cleared on a successful `take` so the client's
    /// next `call()` can proceed. If the `Promise` is dropped before the
    /// reply is consumed, the flag stays set — forcing the user to
    /// explicitly acknowledge the abandoned call via
    /// `reset_in_flight()`.
    pub(crate) in_flight_flag: &'a mut bool,
}

impl<T> Promise<'_, T> {
    /// Try to receive the reply (non-blocking).
    ///
    /// Returns `Ok(Some(reply))` if the reply has arrived,
    /// `Ok(None)` if still pending.
    /// Take one message if the middleware has one — phase-379 W3.
    ///
    /// Named `take` after rcl (`rcl_take`) and rclcpp (`Subscription::take`),
    /// which spell the non-blocking receive that way — as does our own C
    /// surface. NOT after rclrs: its subscription API is callback/worker-driven
    /// and has no public polling counterpart at all (its `take_*` methods are
    /// private), which is why the ledger records this as an `extension` against
    /// the Rust reference rather than a match. `take` was Rust
    /// channel vocabulary that reads as a different contract to a ROS 2 user:
    /// both are non-blocking and both report emptiness without failing, so
    /// nothing asked for the other word. Renamed as a clean break, no shim.
    pub fn take(&mut self) -> Result<Option<T>, NodeError> {
        // Phase 120: NoData (no reply yet) is the steady-state polling
        // condition — map to Ok(None) instead of ServiceRequestFailed.
        match match self.handle.take_response_raw(self.reply_buffer) {
            Ok(opt) => opt,
            Err(TransportError::NoData) => return Ok(None),
            Err(e) => return Err(NodeError::Transport(e)),
        } {
            Some((len, _seq)) => {
                let reply = (self.parse)(&self.reply_buffer[..len])?;
                // Reply consumed — allow the client to issue another call.
                *self.in_flight_flag = false;
                Ok(Some(reply))
            }
            None => Ok(None),
        }
    }
}

impl<T> Promise<'_, T> {
    /// Block until the reply arrives, spinning the executor.
    ///
    /// Internally calls `executor.spin_once()` in a loop until the reply
    /// arrives or `timeout_ms` is exhausted. This is equivalent to the
    /// manual spin+poll loop pattern but more ergonomic for simple use cases.
    ///
    /// No borrow conflict: `executor` and `self` (which borrows the standalone
    /// client) are disjoint objects.
    ///
    /// # Errors
    ///
    /// Returns [`NodeError::Timeout`] if the reply does not arrive within
    /// `timeout_ms` milliseconds.
    pub fn wait(
        &mut self,
        executor: &mut super::Executor,
        timeout: core::time::Duration,
    ) -> Result<T, NodeError> {
        let spin_interval = core::time::Duration::from_millis(DEFAULT_SPIN_INTERVAL_MS);
        let timeout_ms = timeout.as_millis().min(u64::MAX as u128) as u64;
        let max_spins = (timeout_ms / DEFAULT_SPIN_INTERVAL_MS).max(1);
        let mut budget = WaitBudget::new(max_spins, timeout);
        // Always spin at least once so a zero-timeout still polls.
        loop {
            executor.spin_once(spin_interval);
            if let Some(result) = self.take()? {
                return Ok(result);
            }
            if !budget.tick() {
                return Err(NodeError::Timeout);
            }
        }
    }
}

impl<T> core::future::Future for Promise<'_, T> {
    type Output = Result<T, NodeError>;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        let this = self.get_mut();
        // Register-then-check (closes the race where a reply lands
        // between take returning None and the waker registering).
        this.handle.register_waker(cx.waker());
        match this.take() {
            Ok(Some(reply)) => core::task::Poll::Ready(Ok(reply)),
            Ok(None) => core::task::Poll::Pending,
            Err(e) => core::task::Poll::Ready(Err(e)),
        }
    }
}

impl<T> Promise<'_, T> {
    /// Async poll-until-ready helper for environments where the
    /// backend's `register_waker` path can't deliver a wake.
    ///
    /// The plain `.await` (via the `Future` impl above) parks the
    /// caller until the backend's listener fires the stored Waker.
    /// That works on `std` builds where the backend has a
    /// background-thread pool actively polling its listener tasks,
    /// but it deadlocks on the `nostd-runtime` DDS path (and any
    /// other cooperative backend whose listener future only runs
    /// when something actively drives the runtime). The 160.B.1
    /// trace pinned this to DDS's nostd runtime: listener
    /// futures only advance inside `runtime.block_on(...)`, and the
    /// parked `.await` consumer never issues such a call.
    ///
    /// `poll_until_ready(yield_fn)` instead actively polls
    /// `take()` on each turn and awaits the caller-supplied
    /// yield future between attempts. The yield gives the executor
    /// a chance to run other ready tasks (typically a `spin_task`
    /// that drives the backend runtime via `executor.spin_once()`),
    /// which in turn pumps the listener future. On the `std` path
    /// this devolves to a fast poll-then-yield loop with no
    /// correctness penalty; on `nostd-runtime` it's the only shape
    /// that completes.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let reply = client
    ///     .call(&req)?
    ///     .poll_until_ready(|| embassy_time::Timer::after_millis(5))
    ///     .await?;
    /// ```
    pub async fn poll_until_ready<F, Fut>(&mut self, mut yield_fn: F) -> Result<T, NodeError>
    where
        F: FnMut() -> Fut,
        Fut: core::future::Future<Output = ()>,
    {
        loop {
            match self.take()? {
                Some(reply) => return Ok(reply),
                None => yield_fn().await,
            }
        }
    }
}

/// Deserialize a CDR-encoded service reply.
fn cdr_deserialize_reply<Svc: RosService>(data: &[u8]) -> Result<Svc::Reply, NodeError> {
    let mut reader = CdrReader::new_with_header(data).map_err(|_| NodeError::Deserialization)?;
    Svc::Reply::deserialize(&mut reader).map_err(|_| NodeError::Deserialization)
}

// ============================================================================
// Action types
// ============================================================================

/// Active goal tracking for action server.
#[derive(Clone)]
pub struct ActiveGoal<A: RosAction> {
    /// Goal ID.
    pub goal_id: nros_core::GoalId,
    /// Current status.
    pub status: nros_core::GoalStatus,
    /// The goal data.
    pub goal: A::Goal,
}

/// Completed goal with result.
pub struct CompletedGoal<A: RosAction> {
    /// Goal ID.
    pub goal_id: nros_core::GoalId,
    /// Final status.
    pub status: nros_core::GoalStatus,
    /// The result data.
    pub result: A::Result,
}

// ============================================================================
// ActionServer
// ============================================================================

/// Typed action server with goal state management.
///
/// Wraps [`ActionServerCore`](super::action_core::ActionServerCore) for
/// raw-bytes protocol handling, adding typed goal/feedback/result
/// serialization at the boundary.
pub struct ActionServer<
    A: RosAction,
    const GOAL_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const RESULT_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const FEEDBACK_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const MAX_GOALS: usize = 4,
> {
    pub(crate) core:
        super::action_core::ActionServerCore<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>,
    /// Typed goal data parallel to `core.active_goals`.
    pub(crate) typed_goals: heapless::Vec<A::Goal, MAX_GOALS>,
    /// Completed goals with typed results.
    pub(crate) completed_goals: heapless::Vec<CompletedGoal<A>, MAX_GOALS>,
}

impl<
    A: RosAction,
    const GOAL_BUF: usize,
    const RESULT_BUF: usize,
    const FEEDBACK_BUF: usize,
    const MAX_GOALS: usize,
> ActionServer<A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>
{
    /// Try to accept a new goal.
    ///
    /// Checks for incoming send_goal requests. If one is available, calls the
    /// handler to decide acceptance. Returns the goal ID if accepted.
    pub fn try_accept_goal(
        &mut self,
        goal_handler: impl FnOnce(&nros_core::GoalId, &A::Goal) -> nros_core::GoalResponse,
    ) -> Result<Option<nros_core::GoalId>, NodeError>
    where
        A::Goal: Clone,
    {
        let raw_req = self.core.try_recv_goal_request()?;
        let raw_req = match raw_req {
            Some(r) => r,
            None => return Ok(None),
        };

        // Deserialize the goal from the buffer at the offset captured
        // by the core (DDS prepends an 8-byte seq prefix; zenoh uses 0).
        let buf = self.core.goal_buffer();
        let start = raw_req.data_offset;
        let end = start + raw_req.data_len;
        let mut reader = CdrReader::new_with_header(&buf[start..end])
            .map_err(|_| NodeError::Transport(TransportError::DeserializationError))?;
        // Skip past the GoalId — a fixed `uint8[16]` UUID, no length prefix
        // (ROS 2 `unique_identifier_msgs/UUID`; see action_core::read_goal_id).
        for _ in 0..GOAL_UUID_SIZE {
            let _ = reader.read_u8();
        }
        let goal = A::Goal::deserialize(&mut reader)
            .map_err(|_| NodeError::Transport(TransportError::DeserializationError))?;

        let response = goal_handler(&raw_req.goal_id, &goal);
        let accepted = response.is_accepted();

        if accepted {
            self.core
                .accept_goal(raw_req.goal_id, raw_req.sequence_number)?;
            let _ = self.typed_goals.push(goal);
            Ok(Some(raw_req.goal_id))
        } else {
            self.core.reject_goal(raw_req.sequence_number)?;
            Ok(None)
        }
    }

    /// Publish feedback for a goal.
    pub fn publish_feedback(
        &mut self,
        goal_id: &nros_core::GoalId,
        feedback: &A::Feedback,
    ) -> Result<(), NodeError> {
        // Serialize feedback into a temp buffer (without CDR header or GoalId)
        let mut tmp = [0u8; FEEDBACK_BUF];
        let mut writer = CdrWriter::new(&mut tmp);
        feedback
            .serialize(&mut writer)
            .map_err(|_| NodeError::Serialization)?;
        let feedback_len = writer.position();

        self.core
            .publish_feedback_raw(goal_id, &tmp[..feedback_len])
    }

    /// Set a goal's status.
    ///
    /// Also publishes the updated `GoalStatusArray` on the status topic.
    pub fn set_goal_status(&mut self, goal_id: &nros_core::GoalId, status: nros_core::GoalStatus) {
        self.core.set_goal_status(goal_id, status);
    }

    /// Complete a goal and store the result.
    ///
    /// Also publishes the updated `GoalStatusArray` on the status topic.
    ///
    /// # Errors
    ///
    /// `NodeError::Serialization` if `result` does not serialize, or
    /// `NodeError::BufferTooSmall` if the serialized result exceeds
    /// `RESULT_BUF` and so cannot be retained for a later `get_result`.
    /// Issue 0796: this returned `()`, so both failures were invisible — and
    /// the second one used to strand every client waiting on the result.
    pub fn complete_goal(
        &mut self,
        goal_id: &nros_core::GoalId,
        status: nros_core::GoalStatus,
        result: A::Result,
    ) -> Result<(), NodeError> {
        // Serialize result for the core slab. Issue 0796: a serialization
        // failure used to degrade to a zero-length result stored as if it were
        // the real one; it is now reported.
        let mut tmp = [0u8; RESULT_BUF];
        let mut writer = CdrWriter::new(&mut tmp);
        let serialized = result.serialize(&mut writer);
        let result_len = writer.position();

        // Remove typed goal parallel to core's active_goals removal
        if let Some(pos) = self
            .core
            .active_goals()
            .iter()
            .position(|g| g.goal_id.uuid == goal_id.uuid)
        {
            self.typed_goals.swap_remove(pos);
        }

        let stored = match serialized {
            Ok(()) => self
                .core
                .complete_goal_raw(goal_id, status, &tmp[..result_len]),
            Err(_) => {
                // Still retire the goal (terminal status, waiting requesters)
                // with an empty payload, then report the failure.
                let _ = self.core.complete_goal_raw(goal_id, status, &[]);
                Err(NodeError::Serialization)
            }
        };

        // Issue 0796 — the typed mirror is reclaimed with the core it mirrors.
        // `completed_goals` was push-only: after MAX_GOALS completions every
        // later push was silently dropped, so the table said "the last four
        // goals" while actually holding "the first four, forever".
        self.completed_goals
            .retain(|c| self.core.has_completed_result(&c.goal_id));
        if self.core.has_completed_result(goal_id) {
            let _ = self.completed_goals.push(CompletedGoal {
                goal_id: *goal_id,
                status,
                result,
            });
        }

        stored
    }

    /// Try to handle a cancel_goal request.
    pub fn try_handle_cancel(
        &mut self,
        cancel_handler: impl FnOnce(
            &nros_core::GoalId,
            nros_core::GoalStatus,
        ) -> nros_core::CancelResponse,
    ) -> Result<Option<(nros_core::GoalId, nros_core::CancelResponse)>, NodeError> {
        self.core.try_handle_cancel(cancel_handler)
    }

    /// Try to handle a get_result request.
    pub fn try_handle_get_result(&mut self) -> Result<Option<nros_core::GoalId>, NodeError>
    where
        A::Result: Clone + Default,
    {
        // Serialize default result for non-completed goals
        let mut default_buf = [0u8; RESULT_BUF];
        let mut writer = CdrWriter::new(&mut default_buf);
        let default_len = match A::Result::default().serialize(&mut writer) {
            Ok(()) => writer.position(),
            Err(_) => 0,
        };

        self.core
            .try_handle_get_result_raw(&default_buf[..default_len])
    }

    /// Drain all pending server-side work in one call.
    ///
    /// Calls `try_accept_goal`, `try_handle_cancel`, and
    /// `try_handle_get_result` in sequence. Invoke this on every
    /// `spin_once` iteration in manual-poll code — otherwise clients
    /// will hang on `get_result` because `create_action_server()`
    /// servers are not arena-registered.
    ///
    /// The two callbacks may be called zero or one times per `poll()`:
    ///   * `on_goal` fires when a new goal arrives.
    ///   * `on_cancel` fires when a cancel request arrives.
    ///
    /// Get-result requests are drained unconditionally (no callback
    /// needed — the result is pulled from the goal's stored state).
    ///
    /// # Example
    /// ```ignore
    /// let mut server = node.create_action_server::<Fibonacci>("/fibonacci")?;
    /// loop {
    ///     executor.spin_once(Duration::from_millis(10));
    ///     server.poll(
    ///         |id, goal| {
    ///             /* accept or reject based on `goal` */
    ///             GoalResponse::AcceptAndExecute
    ///         },
    ///         |_id, _status| CancelResponse::Accept,
    ///     )?;
    /// }
    /// ```
    pub fn poll<GF, CF>(&mut self, mut on_goal: GF, mut on_cancel: CF) -> Result<(), NodeError>
    where
        GF: FnMut(&nros_core::GoalId, &A::Goal) -> nros_core::GoalResponse,
        CF: FnMut(&nros_core::GoalId, nros_core::GoalStatus) -> nros_core::CancelResponse,
        A::Goal: Clone,
        A::Result: Clone + Default,
    {
        let _ = self.try_accept_goal(|id, goal| on_goal(id, goal))?;
        let _ = self.try_handle_cancel(|id, status| on_cancel(id, status))?;
        let _ = self.try_handle_get_result()?;
        Ok(())
    }

    /// Get a reference to an active goal.
    pub fn get_goal(&self, goal_id: &nros_core::GoalId) -> Option<ActiveGoal<A>>
    where
        A::Goal: Clone,
    {
        self.core
            .active_goals()
            .iter()
            .enumerate()
            .find(|(_, g)| g.goal_id.uuid == goal_id.uuid)
            .map(|(i, raw)| ActiveGoal {
                goal_id: raw.goal_id,
                status: raw.status,
                goal: self.typed_goals[i].clone(),
            })
    }

    /// Get the number of active goals.
    pub fn active_goal_count(&self) -> usize {
        self.core.active_goal_count()
    }
}

// ============================================================================
// ActionClient
// ============================================================================

/// Typed action client handle.
///
/// Wraps [`ActionClientCore`](super::action_core::ActionClientCore) for
/// raw-bytes protocol handling, adding typed goal/feedback/result
/// serialization at the boundary.
pub struct ActionClient<
    A: RosAction,
    const GOAL_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const RESULT_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const FEEDBACK_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    pub(crate) core: super::action_core::ActionClientCore<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>,
    pub(crate) _phantom: PhantomData<A>,
}

impl<A: RosAction, const GOAL_BUF: usize, const RESULT_BUF: usize, const FEEDBACK_BUF: usize>
    ActionClient<A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>
{
    /// Send a goal (non-blocking). Returns the goal ID and a [`Promise`] for acceptance.
    ///
    /// The promise resolves to `true` if accepted, `false` if rejected.
    pub fn send_goal(
        &mut self,
        goal: &A::Goal,
    ) -> Result<(nros_core::GoalId, Promise<'_, bool>), NodeError> {
        if self.core.in_flight_send_goal {
            return Err(NodeError::RequestInFlight);
        }

        // Serialize goal into a temp buffer (without CDR header or GoalId)
        let mut tmp = [0u8; GOAL_BUF];
        let mut writer = CdrWriter::new(&mut tmp);
        goal.serialize(&mut writer)
            .map_err(|_| NodeError::Serialization)?;
        let goal_len = writer.position();

        let goal_id = self.core.send_goal_raw(&tmp[..goal_len])?;
        self.core.in_flight_send_goal = true;

        Ok((
            goal_id,
            Promise {
                handle: &mut self.core.send_goal_client,
                reply_buffer: &mut self.core.result_buffer,
                parse: parse_goal_accepted,
                in_flight_flag: &mut self.core.in_flight_send_goal,
            },
        ))
    }

    /// Try to receive feedback (non-blocking).
    pub fn try_recv_feedback(
        &mut self,
    ) -> Result<Option<(nros_core::GoalId, A::Feedback)>, NodeError> {
        let (goal_id, len) = match self.core.try_recv_feedback_raw()? {
            Some(v) => v,
            None => return Ok(None),
        };

        // Deserialize feedback from the core's feedback buffer (after GoalId)
        let mut reader = CdrReader::new_with_header(&self.core.feedback_buffer[..len])
            .map_err(|_| NodeError::Transport(TransportError::DeserializationError))?;
        // Skip GoalId — a fixed `uint8[16]` UUID, no length prefix
        // (ROS 2 `unique_identifier_msgs/UUID`; see action_core::read_goal_id).
        for _ in 0..GOAL_UUID_SIZE {
            let _ = reader.read_u8();
        }

        let feedback = A::Feedback::deserialize(&mut reader)
            .map_err(|_| NodeError::Transport(TransportError::DeserializationError))?;

        Ok(Some((goal_id, feedback)))
    }

    /// Cancel a goal (non-blocking). Returns a [`Promise`] for the
    /// `action_msgs/srv/CancelGoal` return code.
    ///
    /// Issue 0796 — the promise resolves to a
    /// [`nros_core::CancelReturnCode`], the RPC-level outcome. The per-goal
    /// accept/reject decision a SERVER's cancel callback returns is the
    /// separate [`nros_core::CancelResponse`]; both were spelled
    /// `CancelResponse` until the two were split.
    pub fn cancel_goal(
        &mut self,
        goal_id: &nros_core::GoalId,
    ) -> Result<Promise<'_, nros_core::CancelReturnCode>, NodeError> {
        if self.core.in_flight_cancel {
            return Err(NodeError::RequestInFlight);
        }
        self.core.send_cancel_request(goal_id)?;
        self.core.in_flight_cancel = true;

        Ok(Promise {
            handle: &mut self.core.cancel_goal_client,
            reply_buffer: &mut self.core.result_buffer,
            parse: parse_cancel_response,
            in_flight_flag: &mut self.core.in_flight_cancel,
        })
    }

    /// Get the result of a completed goal (non-blocking). Returns a [`Promise`].
    pub fn get_result(
        &mut self,
        goal_id: &nros_core::GoalId,
    ) -> Result<Promise<'_, (nros_core::GoalStatus, A::Result)>, NodeError> {
        if self.core.in_flight_get_result {
            return Err(NodeError::RequestInFlight);
        }
        self.core.send_get_result_request(goal_id)?;
        self.core.in_flight_get_result = true;

        Ok(Promise {
            handle: &mut self.core.get_result_client,
            reply_buffer: &mut self.core.result_buffer,
            parse: parse_result_response::<A>,
            in_flight_flag: &mut self.core.in_flight_get_result,
        })
    }

    /// Explicitly clear the "send_goal reply in flight" flag (Phase 84.D3).
    pub fn reset_send_goal_in_flight(&mut self) {
        self.core.in_flight_send_goal = false;
    }

    /// Block until the action server's send-goal queryable is discoverable
    /// on the network, or `timeout` elapses.
    ///
    /// Returns `Ok(true)` on discovery, `Ok(false)` on timeout. Mirrors
    /// `rclcpp_action::Client::wait_for_action_server`.
    ///
    /// Implementation: probes the action's `send_goal` service-server
    /// liveliness keyexpr via the same primitive as
    /// [`Client::wait_for_service`]. Once that service is reachable the
    /// remaining four action entities (cancel queryable + feedback /
    /// status / result publishers) are also reachable in practice — they
    /// were declared by the same server in one batch.
    pub fn wait_for_action_server(
        &mut self,
        executor: &mut super::Executor,
        timeout: core::time::Duration,
    ) -> Result<bool, NodeError> {
        // issue 1008 — `Ok(true)` only; see `wait_for_service` above.
        if matches!(self.core.send_goal_client.service_is_ready(), Ok(true)) {
            return Ok(true);
        }
        let spin_interval = core::time::Duration::from_millis(DEFAULT_SPIN_INTERVAL_MS);
        let max_spins = (timeout.as_millis() as u64 / DEFAULT_SPIN_INTERVAL_MS).max(1);
        let mut budget = WaitBudget::new(max_spins, timeout);
        // See `Client::wait_for_service` for the re-probe rationale: a
        // single liveliness_get samples the router's current token list
        // and terminates; we loop with shorter per-probe timeouts so the
        // outer budget covers servers that come up after we start
        // waiting.
        const PROBE_TIMEOUT_MS: u32 = crate::SERVER_DISCOVERY_PROBE_TIMEOUT_MS; // issue #224
        loop {
            self.core
                .send_goal_client
                .start_server_discovery(PROBE_TIMEOUT_MS)
                .map_err(|_| NodeError::ServiceRequestFailed)?;
            loop {
                executor.spin_once(spin_interval);
                match self
                    .core
                    .send_goal_client
                    .poll_server_discovery()
                    .map_err(|_| NodeError::ServiceRequestFailed)?
                {
                    Some(true) => return Ok(true),
                    Some(false) => break,
                    None => {}
                }
                if !budget.tick() {
                    return Ok(false);
                }
            }
            if !budget.tick() {
                return Ok(false);
            }
        }
    }

    /// Snapshot whether the action server is currently visible.
    /// Mirrors `rclcpp_action::Client::action_server_is_ready`.
    pub fn action_server_is_ready(&self) -> bool {
        matches!(self.core.send_goal_client.service_is_ready(), Ok(true))
    }

    /// Explicitly clear the "cancel reply in flight" flag (Phase 84.D3).
    pub fn reset_cancel_in_flight(&mut self) {
        self.core.in_flight_cancel = false;
    }

    /// Explicitly clear the "get_result reply in flight" flag (Phase 84.D3).
    pub fn reset_get_result_in_flight(&mut self) {
        self.core.in_flight_get_result = false;
    }

    /// Create a feedback stream (receives feedback for all goals).
    ///
    /// The stream borrows `&mut self` exclusively. Drop it before calling
    /// `get_result()` or `cancel_goal()`.
    pub fn feedback_stream(&mut self) -> FeedbackStream<'_, A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF> {
        FeedbackStream { client: self }
    }

    /// Create a goal-filtered feedback stream.
    ///
    /// Only yields feedback for the given `goal_id`, returning `A::Feedback`
    /// directly (without the `GoalId` wrapper).
    pub fn feedback_stream_for(
        &mut self,
        goal_id: nros_core::GoalId,
    ) -> GoalFeedbackStream<'_, A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF> {
        GoalFeedbackStream {
            client: self,
            goal_id,
        }
    }
}

// ============================================================================
// FeedbackStream
// ============================================================================

/// A stream of feedback messages from an action server.
///
/// Created by [`ActionClient::feedback_stream()`]. Receives feedback for
/// all active goals. The stream never self-terminates — use combinators
/// like `take_while` or `break` to stop.
///
/// Three access modes:
/// - **Async (`Stream`)**: Enable the `stream` feature for
///   `futures_core::Stream` + `StreamExt` combinators
/// - **Async (no deps)**: Use `next()` in
///   `while let` loops (always available)
/// - **Sync**: Use [`wait_next()`](FeedbackStream::wait_next) which
///   drives the executor internally
pub struct FeedbackStream<
    'a,
    A: RosAction,
    const GOAL_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const RESULT_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const FEEDBACK_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    client: &'a mut ActionClient<A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>,
}

impl<A: RosAction, const GOAL_BUF: usize, const RESULT_BUF: usize, const FEEDBACK_BUF: usize>
    FeedbackStream<'_, A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>
{
    /// Async: wait for the next feedback message (no `futures` dependency needed).
    ///
    /// Requires a background task running `executor.spin_async()` to drive
    /// I/O. Returns `None` only on error.
    ///
    /// When the `stream` feature is enabled, prefer `StreamExt::next()` or
    /// `TryStreamExt::try_next()` for combinator support.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let mut stream = client.feedback_stream();
    /// while let Some(result) = stream.recv().await {
    ///     let (goal_id, feedback) = result?;
    ///     // process feedback...
    /// }
    /// ```
    pub async fn recv(&mut self) -> Option<Result<(nros_core::GoalId, A::Feedback), NodeError>> {
        core::future::poll_fn(|cx| {
            // Register-then-check (closes the AtomicWaker race).
            self.client
                .core
                .feedback_subscriber
                .register_waker(cx.waker());
            match self.client.try_recv_feedback() {
                Ok(Some(item)) => core::task::Poll::Ready(Some(Ok(item))),
                Ok(None) => core::task::Poll::Pending,
                Err(e) => core::task::Poll::Ready(Some(Err(e))),
            }
        })
        .await
    }

    /// Sync: wait for the next feedback message, spinning the executor.
    ///
    /// Returns `Ok(Some(feedback))` if a message arrives within `timeout_ms`,
    /// or `Ok(None)` on timeout. Unlike [`Promise::wait()`], timeout is not
    /// an error — the caller typically retries in a loop.
    pub fn wait_next(
        &mut self,
        executor: &mut super::Executor,
        timeout: core::time::Duration,
    ) -> Result<Option<(nros_core::GoalId, A::Feedback)>, NodeError> {
        let spin_interval = core::time::Duration::from_millis(DEFAULT_SPIN_INTERVAL_MS);
        let timeout_ms = timeout.as_millis().min(u64::MAX as u128) as u64;
        let max_spins = (timeout_ms / DEFAULT_SPIN_INTERVAL_MS).max(1);
        let mut budget = WaitBudget::new(max_spins, timeout);
        loop {
            executor.spin_once(spin_interval);
            if let Some(item) = self.client.try_recv_feedback()? {
                return Ok(Some(item));
            }
            if !budget.tick() {
                return Ok(None);
            }
        }
    }
}

#[cfg(feature = "stream")]
impl<A: RosAction, const GOAL_BUF: usize, const RESULT_BUF: usize, const FEEDBACK_BUF: usize>
    futures_core::Stream for FeedbackStream<'_, A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>
{
    type Item = Result<(nros_core::GoalId, A::Feedback), NodeError>;

    fn poll_next(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Option<Self::Item>> {
        let this = self.get_mut();
        // Register-then-check (closes the AtomicWaker race).
        this.client
            .core
            .feedback_subscriber
            .register_waker(cx.waker());
        match this.client.try_recv_feedback() {
            Ok(Some(item)) => core::task::Poll::Ready(Some(Ok(item))),
            Ok(None) => core::task::Poll::Pending,
            Err(e) => core::task::Poll::Ready(Some(Err(e))),
        }
    }
}

// ============================================================================
// GoalFeedbackStream
// ============================================================================

/// A goal-filtered feedback stream.
///
/// Created by [`ActionClient::feedback_stream_for()`]. Only yields feedback
/// messages matching the specified goal ID.
pub struct GoalFeedbackStream<
    'a,
    A: RosAction,
    const GOAL_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const RESULT_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const FEEDBACK_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    client: &'a mut ActionClient<A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>,
    goal_id: nros_core::GoalId,
}

impl<A: RosAction, const GOAL_BUF: usize, const RESULT_BUF: usize, const FEEDBACK_BUF: usize>
    GoalFeedbackStream<'_, A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>
{
    /// Async: wait for the next feedback message for this goal (no `futures` dependency needed).
    ///
    /// When the `stream` feature is enabled, prefer `StreamExt::next()` or
    /// `TryStreamExt::try_next()` for combinator support.
    pub async fn recv(&mut self) -> Option<Result<A::Feedback, NodeError>> {
        core::future::poll_fn(|cx| {
            // Register-then-check (closes the AtomicWaker race). The
            // waker is registered once for both the "no data" and
            // "wrong goal" branches that fall through to Pending.
            self.client
                .core
                .feedback_subscriber
                .register_waker(cx.waker());
            match self.client.try_recv_feedback() {
                Ok(Some((id, feedback))) if id.uuid == self.goal_id.uuid => {
                    core::task::Poll::Ready(Some(Ok(feedback)))
                }
                // Feedback for a different goal — keep waiting.
                Ok(Some(_)) => core::task::Poll::Pending,
                Ok(None) => core::task::Poll::Pending,
                Err(e) => core::task::Poll::Ready(Some(Err(e))),
            }
        })
        .await
    }

    /// Sync: wait for the next feedback message for this goal, spinning the executor.
    pub fn wait_next(
        &mut self,
        executor: &mut super::Executor,
        timeout: core::time::Duration,
    ) -> Result<Option<A::Feedback>, NodeError> {
        let spin_interval = core::time::Duration::from_millis(DEFAULT_SPIN_INTERVAL_MS);
        let timeout_ms = timeout.as_millis().min(u64::MAX as u128) as u64;
        let max_spins = (timeout_ms / DEFAULT_SPIN_INTERVAL_MS).max(1);
        let mut budget = WaitBudget::new(max_spins, timeout);
        loop {
            executor.spin_once(spin_interval);
            if let Some((id, feedback)) = self.client.try_recv_feedback()?
                && id.uuid == self.goal_id.uuid
            {
                return Ok(Some(feedback));
            }
            if !budget.tick() {
                return Ok(None);
            }
        }
    }
}

#[cfg(feature = "stream")]
impl<A: RosAction, const GOAL_BUF: usize, const RESULT_BUF: usize, const FEEDBACK_BUF: usize>
    futures_core::Stream for GoalFeedbackStream<'_, A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>
{
    type Item = Result<A::Feedback, NodeError>;

    fn poll_next(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Option<Self::Item>> {
        let this = self.get_mut();
        // Register-then-check (closes the AtomicWaker race).
        this.client
            .core
            .feedback_subscriber
            .register_waker(cx.waker());
        match this.client.try_recv_feedback() {
            Ok(Some((id, feedback))) if id.uuid == this.goal_id.uuid => {
                core::task::Poll::Ready(Some(Ok(feedback)))
            }
            Ok(Some(_)) => core::task::Poll::Pending,
            Ok(None) => core::task::Poll::Pending,
            Err(e) => core::task::Poll::Ready(Some(Err(e))),
        }
    }
}

/// Parse a goal acceptance response (bool).
///
/// Issue #223 — CDR read failures PROPAGATE: a truncated/corrupt frame used
/// to collapse to `accepted = false` via `unwrap_or(0)`, silently reporting
/// "goal rejected" for a wire error. Same rule in the two parsers below.
fn parse_goal_accepted(data: &[u8]) -> Result<bool, NodeError> {
    let mut reader =
        CdrReader::new_with_header(data).map_err(|_| NodeError::ServiceRequestFailed)?;
    let accepted = reader
        .read_u8()
        .map_err(|_| NodeError::ServiceRequestFailed)?
        != 0;
    Ok(accepted)
}

/// Parse a cancel response (issue #223 — read errors propagate; an
/// out-of-range enum value is still mapped through `from_i8`'s default,
/// which is a PROTOCOL value question, not a truncation).
fn parse_cancel_response(data: &[u8]) -> Result<nros_core::CancelReturnCode, NodeError> {
    let mut reader =
        CdrReader::new_with_header(data).map_err(|_| NodeError::ServiceRequestFailed)?;
    let return_code = reader
        .read_i8()
        .map_err(|_| NodeError::ServiceRequestFailed)?;
    Ok(nros_core::CancelReturnCode::from_i8(return_code).unwrap_or_default())
}

/// Parse an action result response (status + result; issue #223 — read
/// errors propagate).
fn parse_result_response<A: RosAction>(
    data: &[u8],
) -> Result<(nros_core::GoalStatus, A::Result), NodeError> {
    let mut reader =
        CdrReader::new_with_header(data).map_err(|_| NodeError::ServiceRequestFailed)?;
    let status_code = reader
        .read_i8()
        .map_err(|_| NodeError::ServiceRequestFailed)?;
    let status = nros_core::GoalStatus::from_i8(status_code).unwrap_or_default();
    let result =
        A::Result::deserialize(&mut reader).map_err(|_| NodeError::ServiceRequestFailed)?;
    Ok((status, result))
}

#[cfg(test)]
mod parse_response_tests {
    // Issue #223 — truncated action-response frames must ERROR, not collapse
    // to plausible defaults ("goal rejected" / CancelReturnCode::default()).
    use super::{parse_cancel_response, parse_goal_accepted};

    /// A valid 4-byte CDR encapsulation header with NO payload — the
    /// truncation case the pre-#223 parsers silently defaulted on.
    const HEADER_ONLY: &[u8] = &[0x00, 0x01, 0x00, 0x00];

    #[test]
    fn truncated_goal_accepted_errors() {
        assert!(parse_goal_accepted(HEADER_ONLY).is_err());
    }

    #[test]
    fn truncated_cancel_response_errors() {
        assert!(parse_cancel_response(HEADER_ONLY).is_err());
    }

    #[test]
    fn valid_goal_accepted_still_parses() {
        let frame = [0x00, 0x01, 0x00, 0x00, 0x01];
        assert!(parse_goal_accepted(&frame).unwrap());
        let frame0 = [0x00, 0x01, 0x00, 0x00, 0x00];
        assert!(!parse_goal_accepted(&frame0).unwrap());
    }
}
