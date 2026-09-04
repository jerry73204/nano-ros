//! phase-417 W4.d — rate-limited logging, on the `Logger` everything else uses.
//!
//! ## Why this lives here and not in `nros-core`
//!
//! Before this module there WAS a throttle in the tree —
//! `nros_core::logger::Logger::{debug,info,warn,error,trace}_throttle` — on a
//! DIFFERENT `Logger` type: one that forwards to the `log` crate and therefore
//! never reaches `nros_platform_log_write`. So the capability existed on the
//! logger nothing dispatches through, and was missing from the logger
//! everything dispatches through. RFC-0019 makes the Rust API the source of
//! truth for C and C++, and the type the FFI hands out is
//! [`crate::Logger`] — so the throttle has to be here for the wrappers to be
//! thin.
//!
//! ## The decision function is shared, deliberately
//!
//! Two storages need this rule: a `'static` [`ThrottleState`] (what the Rust
//! macros declare at the call site) and a caller-owned `uint64_t` (what the C
//! macros declare, because a C call site cannot name a Rust type without a
//! hand-mirrored struct — the drift class issue 0160 catalogues). Both call
//! [`throttle_admits`]. RFC-0089: a second *spelling* is free, a second *code
//! path that can produce a different answer* is not.
//!
//! ## Contract
//!
//! - The FIRST record at a call site always emits. `rclcpp`'s
//!   `RCLCPP_*_THROTTLE` does the same; `nros_core`'s old `should_log_throttled`
//!   did NOT (it started from `last = 0` and compared `now >= last + interval`,
//!   so at `now < interval` the first message was dropped).
//! - The window is measured from the last EMITTED record, not from the last
//!   attempt.
//! - The severity threshold is checked BEFORE the window, so a record filtered
//!   by [`crate::Logger::level`] does not consume the window.

use core::cell::UnsafeCell;

use portable_atomic::{AtomicBool, Ordering};

/// `0` means "nothing emitted yet at this site". A real timestamp of `0` is
/// stored as `1` so the sentinel stays unambiguous — a 1 ns skew on the very
/// first record, against silently re-arming the throttle every time a clock
/// reports zero.
const NEVER: u64 = 0;

/// The whole rule, as a pure function. Both storages call this.
///
/// Returns `true` iff a record raised at `now_ns` should be EMITTED given that
/// the last emitted record at this site was at `last_ns` ([`NEVER`] = none).
///
/// `now_ns.wrapping_sub(last_ns)` rather than `last_ns + interval_ns`: a
/// platform clock that wraps (or a caller passing a `u64` derived from a
/// 32-bit counter) then costs at most one extra record instead of silencing
/// the site forever.
#[must_use]
pub const fn throttle_admits(last_ns: u64, now_ns: u64, interval_ns: u64) -> bool {
    last_ns == NEVER || now_ns.wrapping_sub(last_ns) >= interval_ns
}

/// Normalise a timestamp for STORAGE, keeping [`NEVER`] unambiguous.
#[must_use]
const fn storable(now_ns: u64) -> u64 {
    if now_ns == NEVER { 1 } else { now_ns }
}

/// Milliseconds to nanoseconds, saturating.
///
/// Public because the `nros_*_throttle!` macros expand it in user crates, and
/// because the C shim converts the same way — one conversion, not two.
#[must_use]
pub const fn interval_ms_to_ns(interval_ms: u64) -> u64 {
    interval_ms.saturating_mul(1_000_000)
}

/// Per-call-site throttle window.
///
/// Declare one `static` per site — the `nros_*_throttle!` macros do this for
/// you.
///
/// ## Why not an `AtomicU64`
///
/// There isn't one to use. `portable_atomic::AtomicU64` does not exist on
/// 32-bit bare-metal targets under this crate's dependency spelling
/// (`default-features = false`, so no lock-based `fallback`), and the features
/// that would conjure one are exactly the ones CLAUDE.md records as
/// hard-conflicting with the `critical-section` the `zephyr` crate enables on
/// thumbv7m. A plain non-atomic `u64` would TEAR on those targets — a half-old,
/// half-new timestamp is a window of arbitrary length.
///
/// So: a one-bit guard around a `u64`. The critical section is a load, a
/// comparison and a store, with no call inside it, and a thread that finds the
/// guard taken does NOT spin — it treats the record as suppressed and returns.
/// That is what makes this safe to reach from an ISR on a single core, where
/// spinning on another context's progress would be a deadlock rather than a
/// delay.
///
/// The cost is that two threads deciding the SAME call site at the same instant
/// resolve to one record rather than two. `rclcpp`'s state is not synchronised
/// at all, so this is the tighter of the two.
pub struct ThrottleState {
    /// `true` while a decision is in flight. See the type docs.
    deciding: AtomicBool,
    last_ns: UnsafeCell<u64>,
}

// SAFETY: `last_ns` is read and written only between the `Acquire`
// `compare_exchange` that claims `deciding` and the `Release` store that frees
// it, so exactly one thread touches it at a time and the release/acquire pair
// publishes the write to the next claimant.
unsafe impl Sync for ThrottleState {}

impl ThrottleState {
    /// A window that has never emitted.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            deciding: AtomicBool::new(false),
            last_ns: UnsafeCell::new(NEVER),
        }
    }

    /// Whether a record raised at `now_ns` should emit, updating the window if
    /// it does.
    pub fn should_log(&self, now_ns: u64, interval_ns: u64) -> bool {
        if !self.claim() {
            return false;
        }
        // SAFETY: `claim` succeeded, so this thread owns `last_ns` until
        // `release`.
        let last = unsafe { *self.last_ns.get() };
        let admit = throttle_admits(last, now_ns, interval_ns);
        if admit {
            // SAFETY: as above.
            unsafe { *self.last_ns.get() = storable(now_ns) };
        }
        self.release();
        admit
    }

    /// Timestamp of the last emitted record, or `0` if none.
    ///
    /// Returns `0` if a decision is in flight — this is an observability
    /// accessor, and blocking on it would give the guard the property the
    /// type docs say it must not have.
    #[must_use]
    pub fn last_emitted_ns(&self) -> u64 {
        if !self.claim() {
            return NEVER;
        }
        // SAFETY: `claim` succeeded.
        let last = unsafe { *self.last_ns.get() };
        self.release();
        last
    }

    /// Re-arm: the next record emits whatever the window says.
    pub fn reset(&self) {
        if !self.claim() {
            return;
        }
        // SAFETY: `claim` succeeded.
        unsafe { *self.last_ns.get() = NEVER };
        self.release();
    }

    /// Take the guard, or report that someone else has it. Never spins.
    fn claim(&self) -> bool {
        self.deciding
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Acquire)
            .is_ok()
    }

    fn release(&self) {
        self.deciding.store(false, Ordering::Release);
    }
}

impl Default for ThrottleState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_record_always_admits() {
        // The behaviour `nros_core`'s throttle got wrong: at `now < interval`
        // its `now >= last + interval` test dropped the very first message.
        assert!(throttle_admits(NEVER, 0, 1_000));
        assert!(throttle_admits(NEVER, 5, 1_000_000_000));
        let state = ThrottleState::new();
        assert!(state.should_log(5, 1_000_000_000));
    }

    #[test]
    fn window_suppresses_then_admits() {
        let state = ThrottleState::new();
        let interval = interval_ms_to_ns(100); // 100 ms
        assert!(state.should_log(1_000_000_000, interval), "first emits");
        assert!(
            !state.should_log(1_050_000_000, interval),
            "50 ms into a 100 ms window is suppressed"
        );
        assert!(
            !state.should_log(1_099_999_999, interval),
            "1 ns short of the window is suppressed"
        );
        assert!(
            state.should_log(1_100_000_000, interval),
            "exactly at the window admits"
        );
        assert!(
            !state.should_log(1_150_000_000, interval),
            "the window restarts from the record that emitted, not from the attempt"
        );
        assert_eq!(state.last_emitted_ns(), 1_100_000_000);
    }

    #[test]
    fn zero_interval_admits_every_record() {
        let state = ThrottleState::new();
        for now in [10_u64, 10, 11, 11] {
            assert!(state.should_log(now, 0));
        }
    }

    #[test]
    fn zero_timestamp_is_stored_as_one_so_never_stays_unambiguous() {
        let state = ThrottleState::new();
        assert!(state.should_log(0, 1_000));
        assert_eq!(state.last_emitted_ns(), 1, "not the NEVER sentinel");
        assert!(
            !state.should_log(500, 1_000),
            "a second record at t=500 must still be inside the window"
        );
    }

    #[test]
    fn reset_rearms_the_window() {
        let state = ThrottleState::new();
        let interval = interval_ms_to_ns(1_000);
        assert!(state.should_log(1_000_000_000, interval));
        assert!(!state.should_log(1_000_000_001, interval));
        state.reset();
        assert!(state.should_log(1_000_000_002, interval));
    }

    #[test]
    fn a_wrapped_clock_still_measures_real_elapsed_time() {
        // `last_ns + interval_ns` would overflow here and (in release) wrap to
        // a tiny number, admitting every record for the rest of the program.
        // `now.wrapping_sub(last)` is the elapsed time and stays right.
        let last = u64::MAX - 5;
        assert!(
            !throttle_admits(last, 10, interval_ms_to_ns(1)),
            "16 ns after the last record is inside a 1 ms window, wrap or no wrap"
        );
        assert!(
            throttle_admits(last, 2_000_000, interval_ms_to_ns(1)),
            "2 ms after the last record admits, across the wrap"
        );
        // And the same through the stateful form.
        let state = ThrottleState::new();
        assert!(state.should_log(u64::MAX - 5, interval_ms_to_ns(1)));
        assert!(!state.should_log(10, interval_ms_to_ns(1)));
        assert!(state.should_log(2_000_000, interval_ms_to_ns(1)));
    }

    #[test]
    fn interval_conversion_saturates_rather_than_wrapping() {
        assert_eq!(interval_ms_to_ns(1), 1_000_000);
        assert_eq!(interval_ms_to_ns(u64::MAX), u64::MAX);
    }
}
