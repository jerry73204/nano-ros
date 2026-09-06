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
//! [`throttle_decide`]. RFC-0089: a second *spelling* is free, a second *code
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
//! - A clock that does not move is a window that never elapses: a site whose
//!   clock is stuck (at ANY value, `0` included) emits once and then never
//!   (issue 1152). A clock that wraps still measures real elapsed time across
//!   the wrap. Both are tested below.

use core::cell::UnsafeCell;

use portable_atomic::{AtomicBool, Ordering};

/// The state word of a site that has never emitted. `0` because the C macros
/// declare their storage as a zero-initialised `static uint64_t`, so this is
/// the one value a fresh C site can carry without an initialiser.
const NEVER: u64 = 0;

/// Bit 0 of the state word: set once the site has emitted. The sentinel is a
/// FLAG, not a value in the timestamp domain — see the module docs.
const ARMED: u64 = 1;

/// Elapsed time is measured in this many bits: the state word keeps 63 bits of
/// timestamp above the flag, so the clock is reduced mod 2^63 on the way in
/// and the subtraction is reduced the same way on the way out.
const ELAPSED_MASK: u64 = u64::MAX >> 1;

/// The whole rule, as a pure function. Both storages call this.
///
/// `state` is the site's state word — [`NEVER`] for a site that has not
/// emitted, otherwise what an earlier call returned. Returns `Some(next)` iff a
/// record raised at `now_ns` should be EMITTED, and `next` is the word to store
/// in its place; `None` means suppressed and the word is left alone. Returning
/// the encoded word rather than a bare `bool` is what keeps the encoding in
/// ONE place: a caller cannot admit with this rule and then arm with a
/// different spelling of "last emitted" (issue 1152 was exactly that — the
/// arming step stored `1` for a clock reading `0`, and the rule then measured
/// `u64::MAX` of elapsed time against it).
///
/// ## Two clock pathologies, one rule (issue 1152)
///
/// * A WRAPPING clock (a platform counter rolling over, or a `u64` derived from
///   a 32-bit counter) must still measure real elapsed time across the wrap,
///   so the elapsed time is `now - last` in modular arithmetic, never
///   `last + interval` (which overflows to a tiny number and admits everything
///   for the rest of the program).
/// * A STUCK clock (a port whose clock is broken and reports a constant —
///   including `0`, which is what a build without `platform-clock` reports)
///   must admit ONCE and then never, because a stuck clock is exactly when a
///   log flood is most likely. That needs `now == last` to read as zero
///   elapsed, which a sentinel living in the timestamp domain cannot deliver:
///   some real reading has to be displaced to keep the sentinel unambiguous,
///   and a clock stuck on that reading then never equals what was stored.
///
/// The word is `(last_ns << 1) | ARMED`: the flag is bit 0 and the timestamp
/// keeps 63 bits above it. A clock that wraps at 2^64 also wraps at 2^63, so
/// reducing both the stored reading and the subtraction mod 2^63 measures
/// elapsed time exactly across the wrap — the only thing given up is that a
/// jump of exactly 2^63 ns (292 years) between two records reads as no time
/// at all.
#[must_use]
pub const fn throttle_decide(state: u64, now_ns: u64, interval_ns: u64) -> Option<u64> {
    let admit = if state & ARMED == 0 {
        true
    } else {
        let last_ns = state >> 1;
        let elapsed_ns = now_ns.wrapping_sub(last_ns) & ELAPSED_MASK;
        elapsed_ns >= interval_ns
    };
    if admit {
        Some((now_ns << 1) | ARMED)
    } else {
        None
    }
}

/// The timestamp a state word records, if the site has emitted. Reduced mod
/// 2^63 — see [`throttle_decide`].
#[must_use]
const fn recorded_ns(state: u64) -> Option<u64> {
    if state & ARMED == 0 {
        None
    } else {
        Some(state >> 1)
    }
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
/// So: a one-bit guard around a `u64` — the same state word the C storage
/// holds, decoded by the same [`throttle_decide`]. The critical section is a load, a
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
    /// The state word [`throttle_decide`] reads and returns.
    word: UnsafeCell<u64>,
}

// SAFETY: `word` is read and written only between the `Acquire`
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
            word: UnsafeCell::new(NEVER),
        }
    }

    /// Whether a record raised at `now_ns` should emit, updating the window if
    /// it does.
    pub fn should_log(&self, now_ns: u64, interval_ns: u64) -> bool {
        if !self.claim() {
            return false;
        }
        // SAFETY: `claim` succeeded, so this thread owns `word` until
        // `release`.
        let word = unsafe { *self.word.get() };
        let next = throttle_decide(word, now_ns, interval_ns);
        if let Some(next) = next {
            // SAFETY: as above.
            unsafe { *self.word.get() = next };
        }
        self.release();
        next.is_some()
    }

    /// Timestamp of the last emitted record, or `None` if none.
    ///
    /// Reduced mod 2^63 (see [`throttle_decide`]), so a reading at or above
    /// 2^63 ns comes back with its top bit cleared. Also `None` if a decision
    /// is in flight — this is an observability accessor, and blocking on it
    /// would give the guard the property the type docs say it must not have.
    #[must_use]
    pub fn last_emitted_ns(&self) -> Option<u64> {
        if !self.claim() {
            return None;
        }
        // SAFETY: `claim` succeeded.
        let word = unsafe { *self.word.get() };
        self.release();
        recorded_ns(word)
    }

    /// Re-arm: the next record emits whatever the window says.
    pub fn reset(&self) {
        if !self.claim() {
            return;
        }
        // SAFETY: `claim` succeeded.
        unsafe { *self.word.get() = NEVER };
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
        assert!(throttle_decide(NEVER, 0, 1_000).is_some());
        assert!(throttle_decide(NEVER, 5, 1_000_000_000).is_some());
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
        assert_eq!(state.last_emitted_ns(), Some(1_100_000_000));
    }

    #[test]
    fn zero_interval_admits_every_record() {
        let state = ThrottleState::new();
        for now in [10_u64, 10, 11, 11] {
            assert!(state.should_log(now, 0));
        }
    }

    #[test]
    fn zero_timestamp_is_recorded_as_zero_and_distinct_from_never() {
        // The pre-1152 code displaced a reading of `0` to `1` to keep a
        // `0`-sentinel unambiguous; the armed FLAG makes the displacement
        // unnecessary, so what was emitted is what is recorded.
        let state = ThrottleState::new();
        assert_eq!(state.last_emitted_ns(), None, "fresh site");
        assert!(state.should_log(0, 1_000));
        assert_eq!(state.last_emitted_ns(), Some(0), "recorded verbatim");
        assert!(
            !state.should_log(500, 1_000),
            "a second record at t=500 must still be inside the window"
        );
        assert!(
            state.should_log(1_000, 1_000),
            "and t=1000 is exactly the window"
        );
    }

    #[test]
    fn normal_cadence_emits_at_the_interval_and_no_faster() {
        // A 1 kHz caller against a 100 ms window: exactly one record per 100
        // attempts, at the attempt where the window elapses.
        let state = ThrottleState::new();
        let interval = interval_ms_to_ns(100);
        let mut emitted = 0;
        for tick in 0..1_000_u64 {
            let now = 7 + tick * interval_ms_to_ns(1);
            if state.should_log(now, interval) {
                emitted += 1;
                assert_eq!(tick % 100, 0, "emits on the tick where 100 ms elapsed");
            }
        }
        assert_eq!(emitted, 10);
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
        let armed = throttle_decide(NEVER, u64::MAX - 5, 0).expect("first record");
        assert!(
            throttle_decide(armed, 10, interval_ms_to_ns(1)).is_none(),
            "16 ns after the last record is inside a 1 ms window, wrap or no wrap"
        );
        assert!(
            throttle_decide(armed, 2_000_000, interval_ms_to_ns(1)).is_some(),
            "2 ms after the last record admits, across the wrap"
        );
        // And the same through the stateful form.
        let state = ThrottleState::new();
        assert!(state.should_log(u64::MAX - 5, interval_ms_to_ns(1)));
        assert!(!state.should_log(10, interval_ms_to_ns(1)));
        assert!(state.should_log(2_000_000, interval_ms_to_ns(1)));
    }

    #[test]
    fn a_wrapping_clock_keeps_cadence_through_the_wrap() {
        // Issue 1152 asks for both properties at once: a real wrap must not
        // look like a stuck clock, and a stuck clock must not look like a
        // wrap. Walk a 1 kHz clock through u64::MAX with a 100 ms window and
        // count — the wrap must cost neither a lost nor an extra record.
        let state = ThrottleState::new();
        let interval = interval_ms_to_ns(100);
        let start = u64::MAX - 350 * interval_ms_to_ns(1);
        let mut emitted = 0;
        for tick in 0..1_000_u64 {
            let now = start.wrapping_add(tick * interval_ms_to_ns(1));
            if state.should_log(now, interval) {
                emitted += 1;
                assert_eq!(tick % 100, 0, "tick {tick} (now={now})");
            }
        }
        assert_eq!(emitted, 10);
        // The 2^63 boundary is a wrap of the REDUCED clock; same property.
        let state = ThrottleState::new();
        let start = (1_u64 << 63) - 350 * interval_ms_to_ns(1);
        let mut emitted = 0;
        for tick in 0..1_000_u64 {
            let now = start + tick * interval_ms_to_ns(1);
            if state.should_log(now, interval) {
                emitted += 1;
                assert_eq!(tick % 100, 0, "tick {tick} (now={now})");
            }
        }
        assert_eq!(emitted, 10);
    }

    #[test]
    fn a_stuck_clock_at_the_word_boundaries_stays_shut() {
        // Readings whose top bit is lost to the flag, or which are the
        // sentinel itself: the flag, not the value, says whether the site is
        // armed, so none of them re-arm it.
        for stuck in [0_u64, 1, 1 << 63, u64::MAX >> 1, u64::MAX] {
            let armed = throttle_decide(NEVER, stuck, 1).expect("first");
            assert_ne!(armed, NEVER, "an armed site never reads as fresh");
            assert!(
                throttle_decide(armed, stuck, 1).is_none(),
                "stuck at {stuck}"
            );
        }
    }

    /// Issue 1152 — the negative control. A port whose clock is broken reports
    /// a constant; the throttle must then admit ONCE and never again, because
    /// a stuck clock is when a log flood is most likely. Before the fix the
    /// `0 -> 1` storage skew made `now.wrapping_sub(last)` read `u64::MAX` and
    /// every record after the first was admitted.
    #[test]
    fn a_clock_stuck_at_zero_admits_once_then_never() {
        let state = ThrottleState::new();
        let interval = interval_ms_to_ns(100);
        assert!(
            state.should_log(0, interval),
            "the first record always emits"
        );
        for _ in 0..1_000 {
            assert!(
                !state.should_log(0, interval),
                "a clock that never moves is a window that never elapses"
            );
        }
    }

    #[test]
    fn a_clock_stuck_at_a_nonzero_value_admits_once_then_never() {
        for stuck in [1_u64, 42, 1_000_000_000, u64::MAX - 1, u64::MAX] {
            let state = ThrottleState::new();
            let interval = interval_ms_to_ns(100);
            assert!(state.should_log(stuck, interval), "first record at {stuck}");
            for _ in 0..1_000 {
                assert!(!state.should_log(stuck, interval), "stuck at {stuck}");
            }
        }
    }

    #[test]
    fn interval_conversion_saturates_rather_than_wrapping() {
        assert_eq!(interval_ms_to_ns(1), 1_000_000);
        assert_eq!(interval_ms_to_ns(u64::MAX), u64::MAX);
    }
}
