//! Timer API for nros C API.
//!
//! Timers provide periodic callbacks for time-based operations.

use core::{ffi::c_void, ptr};

use crate::{
    error::*,
    support::{nros_support_state_t, nros_support_t},
};

/// Timer callback function type.
///
/// # Parameters
/// * `timer` - Pointer to the timer that triggered
/// * `context` - User-provided context pointer
pub type nros_timer_callback_t =
    Option<unsafe extern "C" fn(timer: *mut nros_timer_t, context: *mut c_void)>;

/// Timer state
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_timer_state_t {
    /// Not initialized
    NROS_TIMER_STATE_UNINITIALIZED = 0,
    /// Initialized and running
    NROS_TIMER_STATE_RUNNING = 1,
    /// Initialized but canceled
    NROS_TIMER_STATE_CANCELED = 2,
    /// Shutdown
    NROS_TIMER_STATE_SHUTDOWN = 3,
}

/// Timer structure.
#[repr(C)]
pub struct nros_timer_t {
    /// Current state
    pub state: nros_timer_state_t,
    /// Period in nanoseconds
    pub period_ns: u64,
    /// Last trigger time in nanoseconds
    pub last_call_time_ns: u64,
    /// User callback function
    pub callback: nros_timer_callback_t,
    /// User context pointer
    pub context: *mut c_void,
    /// Pointer to parent support context
    pub support: *const nros_support_t,
    /// Handle ID from executor registration (SIZE_MAX = not registered)
    pub handle_id: usize,
    /// Opaque pointer to internal executor (set by rclc_executor_add_timer)
    pub _executor: *mut c_void,
}

impl Default for nros_timer_t {
    fn default() -> Self {
        Self {
            state: nros_timer_state_t::NROS_TIMER_STATE_UNINITIALIZED,
            period_ns: 0,
            last_call_time_ns: 0,
            callback: None,
            context: ptr::null_mut(),
            support: ptr::null(),
            handle_id: usize::MAX,
            _executor: ptr::null_mut(),
        }
    }
}

// Internal helper methods for executor
impl nros_timer_t {
    /// Get the callback function
    pub(crate) fn get_callback(&self) -> nros_timer_callback_t {
        self.callback
    }

    /// Get the user context
    pub(crate) fn get_context(&self) -> *mut c_void {
        self.context
    }

    /// Set the handle ID from executor registration
    pub(crate) fn set_handle_id(&mut self, id: nros_node::HandleId) {
        self.handle_id = id.0;
    }

    /// Set the executor pointer (called by rclc_executor_add_timer)
    pub(crate) fn set_executor_ptr(&mut self, executor: *mut c_void) {
        self._executor = executor;
    }
}

/// Get a zero-initialized timer.
#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_timer() -> nros_timer_t {
    nros_timer_t::default()
}

/// Initialize a timer.
///
/// # Parameters
/// * `timer` - Pointer to a zero-initialized timer
/// * `support` - Pointer to an initialized support context
/// * `period_ns` - Timer period in nanoseconds
/// * `callback` - Callback function to invoke when timer fires
/// * `context` - User context pointer passed to callback (can be NULL)
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if any required pointer is NULL or period is 0
/// * `NROS_RET_NOT_INIT` if support is not initialized
///
/// # Safety
/// * All required pointers must be valid
/// * `callback` must be a valid function pointer
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_timer_init(
    timer: *mut nros_timer_t,
    support: *const nros_support_t,
    period_ns: u64,
    callback: nros_timer_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    validate_not_null!(timer, support);

    if callback.is_none() || period_ns == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let timer = &mut *timer;
    let support_ref = &*support;

    validate_state!(
        timer,
        nros_timer_state_t::NROS_TIMER_STATE_UNINITIALIZED,
        NROS_RET_BAD_SEQUENCE
    );
    validate_state!(
        support_ref,
        nros_support_state_t::NROS_SUPPORT_STATE_INITIALIZED
    );

    timer.period_ns = period_ns;
    timer.callback = callback;
    timer.context = context;
    timer.support = support;
    timer.last_call_time_ns = 0;
    timer.state = nros_timer_state_t::NROS_TIMER_STATE_RUNNING;

    NROS_RET_OK
}

/// Cancel a timer.
///
/// A canceled timer will not fire, but can be reset to start again.
/// If registered with an executor, forwards to the executor's cancel_timer.
///
/// # Parameters
/// * `timer` - Pointer to an initialized timer
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if timer is NULL
/// * `NROS_RET_NOT_INIT` if not initialized
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_timer_cancel(timer: *mut nros_timer_t) -> nros_ret_t {
    validate_not_null!(timer);

    let timer = &mut *timer;

    match timer.state {
        nros_timer_state_t::NROS_TIMER_STATE_RUNNING => {
            // Forward to executor if registered
            if !timer._executor.is_null() && timer.handle_id != usize::MAX {
                let exec = &mut *(timer._executor as *mut crate::executor::CExecutor);
                let _ = exec.cancel_timer(nros_node::HandleId(timer.handle_id));
            }

            timer.state = nros_timer_state_t::NROS_TIMER_STATE_CANCELED;
            NROS_RET_OK
        }
        nros_timer_state_t::NROS_TIMER_STATE_CANCELED => {
            // Already canceled
            NROS_RET_OK
        }
        _ => NROS_RET_NOT_INIT,
    }
}

/// Reset a timer.
///
/// This resets the timer's last call time and starts it running again
/// if it was canceled. If registered with an executor, forwards to the
/// executor's reset_timer.
///
/// # Parameters
/// * `timer` - Pointer to an initialized timer
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if timer is NULL
/// * `NROS_RET_NOT_INIT` if not initialized
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_timer_reset(timer: *mut nros_timer_t) -> nros_ret_t {
    validate_not_null!(timer);

    let timer = &mut *timer;

    match timer.state {
        nros_timer_state_t::NROS_TIMER_STATE_RUNNING
        | nros_timer_state_t::NROS_TIMER_STATE_CANCELED => {
            // Forward to executor if registered
            if !timer._executor.is_null() && timer.handle_id != usize::MAX {
                let exec = &mut *(timer._executor as *mut crate::executor::CExecutor);
                let _ = exec.reset_timer(nros_node::HandleId(timer.handle_id));
            }

            timer.last_call_time_ns = 0;
            timer.state = nros_timer_state_t::NROS_TIMER_STATE_RUNNING;
            NROS_RET_OK
        }
        _ => NROS_RET_NOT_INIT,
    }
}

/// Finalize a timer.
///
/// # Parameters
/// * `timer` - Pointer to an initialized timer
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if timer is NULL
/// * `NROS_RET_NOT_INIT` if not initialized
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_timer_fini(timer: *mut nros_timer_t) -> nros_ret_t {
    validate_not_null!(timer);

    let timer = &mut *timer;

    if timer.state == nros_timer_state_t::NROS_TIMER_STATE_UNINITIALIZED
        || timer.state == nros_timer_state_t::NROS_TIMER_STATE_SHUTDOWN
    {
        return NROS_RET_NOT_INIT;
    }

    timer.callback = None;
    timer.context = ptr::null_mut();
    timer.support = ptr::null();
    timer.handle_id = usize::MAX;
    timer._executor = ptr::null_mut();
    timer.state = nros_timer_state_t::NROS_TIMER_STATE_SHUTDOWN;

    NROS_RET_OK
}

// `rcl_timer_is_ready` and `nros_timer_call` were previously exposed
// as public C symbols for users who wanted to drive timers manually.
// The executor arena now owns timer readiness evaluation and callback
// dispatch end-to-end (see `packages/api/nros-c/src/executor.rs`'s
// timer handling), so those entry points never fired in normal flow
// and duplicated logic that the arena was already doing. Both
// functions are removed from the public C ABI as of Phase 84.B5.

/// Check if timer is valid (initialized and not shutdown).
///
/// # Parameters
/// * `timer` - Pointer to a timer
///
/// # Returns
/// * `true` if valid, `false` if invalid or NULL
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_timer_is_valid(timer: *const nros_timer_t) -> bool {
    if timer.is_null() {
        return false;
    }

    let timer = &*timer;
    matches!(
        timer.state,
        nros_timer_state_t::NROS_TIMER_STATE_RUNNING
            | nros_timer_state_t::NROS_TIMER_STATE_CANCELED
    )
}

/// Get the timer period in nanoseconds.
///
/// # Parameters
/// * `timer` - Pointer to a timer
///
/// # Returns
/// * Period in nanoseconds, or 0 if invalid
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_timer_get_period(timer: *const nros_timer_t) -> u64 {
    if timer.is_null() {
        return 0;
    }

    let timer = &*timer;
    timer.period_ns
}

/// Get the time until next timer firing.
///
/// # Parameters
/// * `timer` - Pointer to a timer
/// * `current_time_ns` - Current time in nanoseconds
///
/// # Returns
/// * Time until next firing in nanoseconds, or 0 if ready now or invalid
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_timer_get_time_until_next_call(
    timer: *const nros_timer_t,
    current_time_ns: u64,
) -> u64 {
    if timer.is_null() {
        return 0;
    }

    let timer = &*timer;

    if timer.state != nros_timer_state_t::NROS_TIMER_STATE_RUNNING {
        return 0;
    }

    let elapsed = current_time_ns.saturating_sub(timer.last_call_time_ns);
    timer.period_ns.saturating_sub(elapsed)
}

// ============================================================================
// phase-417 W5.c — the timer accessors filed as `gap`
//
// All three are FORWARDERS onto the executor arena's `TimerEntry`, reached
// through the `(handle_id, _executor)` pair `rclc_executor_add_timer`
// installs — the same pair `rcl_timer_cancel` and `rcl_timer_reset` have
// always used. That indirection is why these were gaps: a registered timer's
// live state is the arena's, not this struct's, and this struct's copy is
// stale by construction (nothing writes `last_call_time_ns` after
// registration). Reading the arena is a forward to the source of truth, not
// a second one (RFC-0002 / RFC-0019).
//
// Shape: `nros_ret_t` + an out-param, per rcl's own
// `rcl_timer_is_ready(timer, bool *)`. A bare `bool`/`uint64_t` return would
// have to spend a legal value on "cannot answer" — an unregistered timer has
// no elapsed time, and reporting that as `0` is the defect issue 1008 was.
// ============================================================================

/// Resolve a timer to its executor + arena handle, or `None` when it is not
/// registered with one.
///
/// # Safety
/// `timer` must be non-NULL and point to a valid `nros_timer_t`.
unsafe fn registered_handle(
    timer: &nros_timer_t,
) -> Option<(&'static mut crate::executor::CExecutor, nros_node::HandleId)> {
    if timer._executor.is_null() || timer.handle_id == usize::MAX {
        return None;
    }
    let exec = unsafe { crate::executor::get_executor_from_ptr(timer._executor) };
    Some((exec, nros_node::HandleId(timer.handle_id)))
}

/// Has this timer been cancelled?
///
/// rcl's `rcl_timer_is_canceled(timer, bool *)`. Gap `c:timer_is_canceled`
/// records that our C timer had no cancel predicate under any spelling while
/// C++ and Rust both did.
///
/// Forwards to `Executor::timer_is_canceled` for a registered timer — the
/// arena flag `arena::timer_try_process` actually consults, and the one
/// `rcl_timer_cancel` sets through `Executor::cancel_timer`. A timer that
/// has been initialised but not yet added to an executor has no arena entry,
/// so its own `nros_timer_state_t` is the whole truth and is read instead.
///
/// # Returns
/// * `NROS_RET_OK` with `*is_canceled` written
/// * `NROS_RET_INVALID_ARGUMENT` if either pointer is NULL
/// * `NROS_RET_NOT_INIT` if the timer is uninitialised or finalised
///
/// # Safety
/// * `timer` must be NULL or point to a valid `nros_timer_t`.
/// * `is_canceled` must be NULL or writable.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_timer_is_canceled(
    timer: *const nros_timer_t,
    is_canceled: *mut bool,
) -> nros_ret_t {
    validate_not_null!(timer, is_canceled);
    let timer_ref = &*timer;

    match timer_ref.state {
        nros_timer_state_t::NROS_TIMER_STATE_RUNNING
        | nros_timer_state_t::NROS_TIMER_STATE_CANCELED => {}
        _ => return NROS_RET_NOT_INIT,
    }

    *is_canceled = match registered_handle(timer_ref) {
        Some((exec, id)) => exec.timer_is_canceled(id),
        None => timer_ref.state == nros_timer_state_t::NROS_TIMER_STATE_CANCELED,
    };
    NROS_RET_OK
}

/// Would this timer fire on the next `nros_executor_spin_*` pass?
///
/// rcl's `rcl_timer_is_ready(timer, bool *)`. Gap `c:timer_is_ready` records
/// that Rust had the predicate and C did not.
///
/// Forwards to `Executor::timer_is_ready`, which evaluates
/// `arena::timer_try_process`'s own guard against the arena entry — cancelled
/// is never ready, a fired one-shot is never ready again, otherwise
/// `elapsed >= period`. Deliberately not re-derived here: a readiness answer
/// that can disagree with the dispatcher is worse than no answer.
///
/// # Returns
/// * `NROS_RET_OK` with `*is_ready` written
/// * `NROS_RET_INVALID_ARGUMENT` if either pointer is NULL
/// * `NROS_RET_NOT_INIT` if the timer is uninitialised, finalised, or not
///   registered with an executor — an unregistered timer is dispatched by
///   nobody, so it has no readiness rather than a readiness of `false`
///
/// # Safety
/// * `timer` must be NULL or point to a valid `nros_timer_t`.
/// * `is_ready` must be NULL or writable.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_timer_is_ready(
    timer: *const nros_timer_t,
    is_ready: *mut bool,
) -> nros_ret_t {
    validate_not_null!(timer, is_ready);
    let timer_ref = &*timer;

    match timer_ref.state {
        nros_timer_state_t::NROS_TIMER_STATE_RUNNING
        | nros_timer_state_t::NROS_TIMER_STATE_CANCELED => {}
        _ => return NROS_RET_NOT_INIT,
    }

    let Some((exec, id)) = registered_handle(timer_ref) else {
        return NROS_RET_NOT_INIT;
    };
    match exec.timer_is_ready(id) {
        Some(ready) => {
            *is_ready = ready;
            NROS_RET_OK
        }
        None => NROS_RET_NOT_INIT,
    }
}

/// Nanoseconds accumulated since this timer last fired.
///
/// rcl's `rcl_timer_get_time_since_last_call(timer, int64_t *)`. Gap
/// `c:timer_get_time_since_last_call` records that Rust had it and C did not.
///
/// Forwards to `Executor::timer_elapsed_us`, the arena's `elapsed_us`
/// counter. **This is why the accessor could not read `nros_timer_t` and had
/// to reach the executor:** `nros_timer_t::last_call_time_ns` is written by
/// `nros_timer_init` and `rcl_timer_reset` and by nothing else — no
/// dispatch path updates it — so a struct-local computation answers `0` for
/// every timer that has ever fired.
///
/// **Divergence from rcl, inherited:** the executor's timer accounting is
/// MICROSECOND-based (issue #505), so the nanosecond value this reports is a
/// microsecond quantity scaled by 1000, not a nanosecond measurement. The
/// unit is rcl's; the resolution is ours. Unsigned because elapsed time
/// since a past event cannot be negative, where rcl's `int64_t` inherits the
/// signedness of its clock difference.
///
/// # Returns
/// * `NROS_RET_OK` with `*time_since_last_call_ns` written
/// * `NROS_RET_INVALID_ARGUMENT` if either pointer is NULL
/// * `NROS_RET_NOT_INIT` if the timer is uninitialised, finalised, or not
///   registered with an executor — nothing has been advancing its clock, so
///   there is no elapsed time rather than an elapsed time of zero
///
/// # Safety
/// * `timer` must be NULL or point to a valid `nros_timer_t`.
/// * `time_since_last_call_ns` must be NULL or writable.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_timer_get_time_since_last_call(
    timer: *const nros_timer_t,
    time_since_last_call_ns: *mut u64,
) -> nros_ret_t {
    validate_not_null!(timer, time_since_last_call_ns);
    let timer_ref = &*timer;

    match timer_ref.state {
        nros_timer_state_t::NROS_TIMER_STATE_RUNNING
        | nros_timer_state_t::NROS_TIMER_STATE_CANCELED => {}
        _ => return NROS_RET_NOT_INIT,
    }

    let Some((exec, id)) = registered_handle(timer_ref) else {
        return NROS_RET_NOT_INIT;
    };
    match exec.timer_elapsed_us(id) {
        Some(elapsed_us) => {
            *time_since_last_call_ns = elapsed_us.saturating_mul(1_000);
            NROS_RET_OK
        }
        None => NROS_RET_NOT_INIT,
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::{
        nros_duration_from_nanoseconds, nros_duration_t, nros_time_from_nanoseconds, nros_time_t,
    };

    /// A timer that has been `nros_timer_init`'d but never added to an
    /// executor. Its arena entry does not exist, so nothing is advancing its
    /// clock.
    fn unregistered_running_timer() -> nros_timer_t {
        nros_timer_t {
            state: nros_timer_state_t::NROS_TIMER_STATE_RUNNING,
            period_ns: 1_000_000,
            ..nros_timer_t::default()
        }
    }

    /// phase-417 W5.c — "no executor" is reported, not answered as zero.
    ///
    /// This is the whole reason the accessor forwards to the arena instead of
    /// reading `nros_timer_t`: the struct's `last_call_time_ns` is written by
    /// `init`/`reset` and by nothing else, so a struct-local computation
    /// returns `0` for every timer — including this one, which has no elapsed
    /// time at all rather than an elapsed time of zero (issue 1008's shape).
    #[test]
    fn elapsed_on_an_unregistered_timer_is_unanswerable_not_zero() {
        let timer = unregistered_running_timer();
        let mut out: u64 = 0xdead_beef;
        assert_eq!(
            unsafe { nros_timer_get_time_since_last_call(&timer, &mut out) },
            NROS_RET_NOT_INIT
        );
        assert_eq!(
            out, 0xdead_beef,
            "a failed read must not write the out-param"
        );

        let mut ready = true;
        assert_eq!(
            unsafe { rcl_timer_is_ready(&timer, &mut ready) },
            NROS_RET_NOT_INIT
        );
        assert!(ready, "a failed read must not write the out-param");
    }

    /// Cancellation is the one predicate an unregistered timer CAN answer:
    /// with no arena entry, its own `nros_timer_state_t` is the whole truth,
    /// and `rcl_timer_cancel` maintains it on this path.
    #[test]
    fn cancel_state_is_readable_without_an_executor() {
        let mut timer = unregistered_running_timer();
        let mut canceled = true;
        assert_eq!(
            unsafe { rcl_timer_is_canceled(&timer, &mut canceled) },
            NROS_RET_OK
        );
        assert!(!canceled);

        assert_eq!(unsafe { rcl_timer_cancel(&mut timer) }, NROS_RET_OK);
        assert_eq!(
            unsafe { rcl_timer_is_canceled(&timer, &mut canceled) },
            NROS_RET_OK
        );
        assert!(canceled);
    }

    /// A finalised or never-initialised timer has no state to report under
    /// any of the three accessors.
    #[test]
    fn an_uninitialised_timer_answers_nothing() {
        let timer = nros_timer_t::default();
        let mut b = false;
        let mut n: u64 = 0;
        assert_eq!(
            unsafe { rcl_timer_is_canceled(&timer, &mut b) },
            NROS_RET_NOT_INIT
        );
        assert_eq!(
            unsafe { rcl_timer_is_ready(&timer, &mut b) },
            NROS_RET_NOT_INIT
        );
        assert_eq!(
            unsafe { nros_timer_get_time_since_last_call(&timer, &mut n) },
            NROS_RET_NOT_INIT
        );
    }

    /// NULL in any position is refused, never dereferenced.
    #[test]
    fn null_arguments_are_refused() {
        let timer = unregistered_running_timer();
        let mut b = false;
        let mut n: u64 = 0;
        assert_eq!(
            unsafe { rcl_timer_is_canceled(core::ptr::null(), &mut b) },
            NROS_RET_INVALID_ARGUMENT
        );
        assert_eq!(
            unsafe { rcl_timer_is_canceled(&timer, core::ptr::null_mut()) },
            NROS_RET_INVALID_ARGUMENT
        );
        assert_eq!(
            unsafe { rcl_timer_is_ready(&timer, core::ptr::null_mut()) },
            NROS_RET_INVALID_ARGUMENT
        );
        assert_eq!(
            unsafe { nros_timer_get_time_since_last_call(core::ptr::null(), &mut n) },
            NROS_RET_INVALID_ARGUMENT
        );
    }

    // ========================================================================
    // `nros_difference_times` — the arithmetic behind the `static inline` in
    // <nros/timer.h>.
    //
    // The function itself is header-only (no state, so no exported symbol),
    // and the C probe `tests/compile/node_timer_accessors.c` runs its BODY.
    // What is asserted here is the pair of claims the header makes about that
    // body, which are properties of the two entry points it composes and are
    // therefore testable in Rust: the subtraction cannot overflow, and the
    // re-encode saturates rather than wrapping.
    // ========================================================================

    /// The composition the header's `static inline` performs, verbatim.
    fn difference_times(start: nros_time_t, finish: nros_time_t) -> nros_duration_t {
        let start_ns = unsafe { crate::clock::nros_time_to_nanoseconds(&start) };
        let finish_ns = unsafe { crate::clock::nros_time_to_nanoseconds(&finish) };
        nros_duration_from_nanoseconds(finish_ns - start_ns)
    }

    #[test]
    fn difference_is_finish_minus_start() {
        let start = nros_time_from_nanoseconds(1_000_000_000);
        let finish = nros_time_from_nanoseconds(2_500_000_000);
        let d = difference_times(start, finish);
        assert_eq!((d.sec, d.nanosec), (1, 500_000_000));
    }

    /// Negative spans are ordinary, and `builtin_interfaces` encodes them by
    /// FLOOR division with a non-negative remainder: -1.5 s is
    /// `{sec: -2, nanosec: 5e8}`. Truncation would produce `{0, 5e8}`, which
    /// decodes as +0.5 s — the sign flip issue 0799 fixed one layer down, and
    /// the reason this inline delegates to `nros_duration_from_nanoseconds`
    /// instead of splitting the value itself.
    #[test]
    fn a_finish_before_start_is_negative_and_floors() {
        let start = nros_time_from_nanoseconds(2_500_000_000);
        let finish = nros_time_from_nanoseconds(1_000_000_000);
        let d = difference_times(start, finish);
        assert_eq!((d.sec, d.nanosec), (-2, 500_000_000));

        let zero = difference_times(start, start);
        assert_eq!((zero.sec, zero.nanosec), (0, 0));
    }

    /// The header claims the SUBTRACTION cannot overflow, and the claim rests
    /// on `nros_time_t`'s `int32_t sec`: a decoded time point is bounded by
    /// ~±2.15e18 ns, so any difference is bounded by ~±4.3e18 — inside
    /// `i64`'s ±9.22e18. Asserted at the extremes with a CHECKED subtraction,
    /// so a widening of the time encoding fails here rather than becoming
    /// undefined behaviour in a header nobody re-reads. (rcl's own
    /// `rcl_difference_times` has no such bound: its time point is a bare
    /// `int64_t`, and it subtracts with plain signed arithmetic.)
    #[test]
    fn the_subtraction_cannot_overflow_at_the_encodings_extremes() {
        let max = nros_time_t {
            sec: i32::MAX,
            nanosec: 999_999_999,
        };
        let min = nros_time_t {
            sec: i32::MIN,
            nanosec: 0,
        };
        let max_ns = unsafe { crate::clock::nros_time_to_nanoseconds(&max) };
        let min_ns = unsafe { crate::clock::nros_time_to_nanoseconds(&min) };

        assert!(max_ns.checked_sub(min_ns).is_some());
        assert!(min_ns.checked_sub(max_ns).is_some());
        // And the bound itself, so a widened `sec` type trips this test with
        // an arithmetic message rather than silently.
        assert!(max_ns.saturating_sub(min_ns) < i64::MAX / 2);
    }

    /// What CAN exceed its type is the re-encoded result: a span of ±4.3e9
    /// seconds does not fit `nros_duration_t`'s `int32_t sec`.
    /// `nros_duration_from_nanoseconds` saturates, keeping the encoding
    /// monotone — it does not wrap, which would flip the sign of a span.
    #[test]
    fn an_unrepresentable_span_saturates_rather_than_wrapping() {
        let max = nros_time_t {
            sec: i32::MAX,
            nanosec: 999_999_999,
        };
        let min = nros_time_t {
            sec: i32::MIN,
            nanosec: 0,
        };

        let forward = difference_times(min, max);
        assert_eq!(forward.sec, i32::MAX, "must clamp, not wrap negative");
        let backward = difference_times(max, min);
        assert_eq!(backward.sec, i32::MIN, "must clamp, not wrap positive");
    }
}
