//! Phase 121.6.posix-c — runtime tests against the POSIX C timer port.

#![cfg(feature = "posix-c-port")]

use core::ffi::c_void;
use std::{
    sync::atomic::{AtomicU32, Ordering},
    thread,
    time::{Duration, Instant},
};

#[allow(unused_imports)]
use nros_platform_cffi::CffiPlatform;

/// A generous ceiling for "the timer got to run at all".
///
/// Every deadline below is two orders of magnitude above the period being
/// tested. That is deliberate: the number is not a performance claim and must
/// never be read as one — it is the point past which the timer is not late but
/// ABSENT, and the test says so rather than reporting a rate.
const TIMER_DEADLINE: Duration = Duration::from_secs(2);

/// Poll `ready` until it holds, returning how long that took.
///
/// **Why no test here sleeps for a fixed window.** `thread::sleep(40ms)` then
/// `assert!(count >= 4)` does not assert that a periodic timer is periodic; it
/// asserts how much work this machine did in 40 ms, which is a property of the
/// machine's load. `run-gates-parallel.sh` fans out to `nproc` gates and each
/// one starts its own `cargo` and `cargo nextest`, so the surrounding demand is
/// a multiple of the core count and this process is not guaranteed a core at
/// all. Measured on a 12-core host with 24 busy loops: `got 2` for the `>= 4`
/// assertion, twice in three runs, in a test that passes 3 of 3 idle.
///
/// Polling to a deadline asserts the PROPERTY — the callback keeps arriving —
/// and the caller keeps the lower bound that actually matters by checking the
/// returned elapsed against the period it asked for. A timer that fires N times
/// instantly is still a bug, and still caught.
///
/// Returns `None` if the deadline passed, which every caller turns into a
/// FAILURE. An unmet precondition is never a pass here: a timer that never
/// fired is exactly what these tests exist to catch.
fn wait_for(ready: impl Fn() -> bool) -> Option<Duration> {
    let start = Instant::now();
    loop {
        if ready() {
            return Some(start.elapsed());
        }
        if start.elapsed() >= TIMER_DEADLINE {
            return None;
        }
        // Sleep rather than spin: a busy poll competes with the very callback
        // it is waiting for, which is the contention this helper exists to stop
        // mattering.
        thread::sleep(Duration::from_millis(1));
    }
}

unsafe extern "C" {
    fn nros_platform_timer_create_periodic(
        period_us: u32,
        callback: unsafe extern "C" fn(*mut c_void),
        user_data: *mut c_void,
    ) -> *mut c_void;
    fn nros_platform_timer_create_oneshot(
        timeout_us: u32,
        callback: unsafe extern "C" fn(*mut c_void),
        user_data: *mut c_void,
    ) -> *mut c_void;
    fn nros_platform_timer_destroy(handle: *mut c_void);
    fn nros_platform_timer_cancel(handle: *mut c_void) -> i8;
}

unsafe extern "C" fn bump(user_data: *mut c_void) {
    let counter = unsafe { &*(user_data as *const AtomicU32) };
    counter.fetch_add(1, Ordering::SeqCst);
}

#[test]
fn periodic_timer_fires_repeatedly() {
    let counter = AtomicU32::new(0);
    let handle = unsafe {
        nros_platform_timer_create_periodic(
            5_000, // 5 ms
            bump,
            &counter as *const _ as *mut c_void,
        )
    };
    assert!(!handle.is_null(), "create_periodic must succeed");

    let elapsed = wait_for(|| counter.load(Ordering::SeqCst) >= 4);
    unsafe { nros_platform_timer_destroy(handle) };

    let count = counter.load(Ordering::SeqCst);
    let elapsed = elapsed.unwrap_or_else(|| {
        panic!(
            "periodic timer reached only {count} of 4 fires in {TIMER_DEADLINE:?} \
             (5 ms period). Not slow — absent."
        )
    });
    // The lower bound the deadline does not carry: four fires of a 5 ms period
    // cannot arrive in under ~15 ms, so a "timer" that calls back in a hot loop
    // fails here even though it reaches the count instantly.
    assert!(
        elapsed >= Duration::from_millis(15),
        "4 fires of a 5 ms periodic timer arrived in {elapsed:?} — faster than \
         the period allows, so this is not pacing on the period"
    );
}

#[test]
fn oneshot_timer_fires_once() {
    let counter = AtomicU32::new(0);
    let handle = unsafe {
        nros_platform_timer_create_oneshot(5_000, bump, &counter as *const _ as *mut c_void)
    };
    assert!(!handle.is_null());

    // Two claims, and they need different waits: that it fires (a deadline),
    // and that it fires ONCE (a window after that, during which a periodic bug
    // would fire again).
    assert!(
        wait_for(|| counter.load(Ordering::SeqCst) >= 1).is_some(),
        "oneshot timer never fired within {TIMER_DEADLINE:?} (5 ms timeout)"
    );
    thread::sleep(Duration::from_millis(40));
    let count = counter.load(Ordering::SeqCst);
    assert_eq!(count, 1, "oneshot must fire exactly once");

    unsafe { nros_platform_timer_destroy(handle) };
}

#[test]
fn cancel_prevents_oneshot_fire() {
    let counter = AtomicU32::new(0);
    let handle = unsafe {
        nros_platform_timer_create_oneshot(
            100_000, // 100 ms — comfortable cancellation margin
            bump,
            &counter as *const _ as *mut c_void,
        )
    };
    assert!(!handle.is_null());

    // The opposite hazard to the tests above, and it needs the opposite guard.
    // Here the RACE is ours to lose: the assertion is only meaningful if this
    // thread reaches the cancel before the timer's own 100 ms deadline, and
    // under load a `sleep(5ms)` can return at 150 ms. That is not a cancel that
    // failed to prevent a fire — it is a cancel that arrived after one, and
    // reporting it as "cancel must report prevent-fire" sends the reader into
    // the port. Measure the window and fail on the real cause instead.
    let created = Instant::now();
    thread::sleep(Duration::from_millis(5));
    let before_cancel = created.elapsed();
    let rc = unsafe { nros_platform_timer_cancel(handle) };
    assert!(
        before_cancel < Duration::from_millis(100),
        "the test could not conclude: {before_cancel:?} elapsed before the cancel, \
         past the timer's own 100 ms timeout. This process was descheduled — \
         nothing here says anything about cancel."
    );
    assert_eq!(rc, 1, "cancel must report prevent-fire");

    thread::sleep(Duration::from_millis(120));
    let count = counter.load(Ordering::SeqCst);
    assert_eq!(count, 0, "callback must not have fired after cancel");

    unsafe { nros_platform_timer_destroy(handle) };
}

// ----------------------------------------------------------------------
// Phase 110.E.b — Rust trait-side coverage.
// ----------------------------------------------------------------------

extern "C" fn bump_safe(user_data: *mut c_void) {
    // Same as `bump` above, expressed as a safe `extern "C" fn` so it
    // satisfies the `PlatformTimer` trait's callback type. The trait
    // takes a non-`unsafe` fn pointer; the C impl signature is
    // `unsafe extern "C" fn`. The cffi shim coerces between them.
    let counter = unsafe { &*(user_data as *const AtomicU32) };
    counter.fetch_add(1, Ordering::SeqCst);
}

#[test]
fn rust_trait_periodic_fires() {
    use nros_platform_api::PlatformTimer;

    let counter = AtomicU32::new(0);
    let handle = CffiPlatform::create_periodic(
        5_000, // 5 ms
        bump_safe,
        &counter as *const _ as *mut c_void,
    )
    .expect("create_periodic via Rust trait");

    let elapsed = wait_for(|| counter.load(Ordering::SeqCst) >= 4);
    CffiPlatform::destroy(handle);

    let count = counter.load(Ordering::SeqCst);
    let elapsed = elapsed.unwrap_or_else(|| {
        panic!(
            "periodic timer via the trait surface reached only {count} of 4 fires \
             in {TIMER_DEADLINE:?} (5 ms period). Not slow — absent."
        )
    });
    assert!(
        elapsed >= Duration::from_millis(15),
        "4 fires of a 5 ms periodic timer arrived in {elapsed:?} — faster than \
         the period allows, so this is not pacing on the period"
    );
}

#[test]
fn rust_trait_cancel_returns_true_when_prevented() {
    use nros_platform_api::PlatformTimer;

    let counter = AtomicU32::new(0);
    let mut handle = CffiPlatform::create_oneshot(
        100_000, // 100 ms
        bump_safe,
        &counter as *const _ as *mut c_void,
    )
    .expect("create_oneshot via Rust trait");

    // Same window guard as the C-surface cancel test above, and for the same
    // reason: a descheduled test thread reaches the cancel after the fire, and
    // "cancel must return true" is then a report about the scheduler.
    let created = Instant::now();
    thread::sleep(Duration::from_millis(5));
    let before_cancel = created.elapsed();
    let prevented = CffiPlatform::cancel(&mut handle);
    assert!(
        before_cancel < Duration::from_millis(100),
        "the test could not conclude: {before_cancel:?} elapsed before the cancel, \
         past the timer's own 100 ms timeout. This process was descheduled."
    );
    assert!(
        prevented,
        "cancel via Rust trait must return true when fire prevented"
    );
    CffiPlatform::destroy(handle);
}

// ----------------------------------------------------------------------
// Phase 110.E.b — End-to-end Sporadic-state refill via the trait.
// ----------------------------------------------------------------------
//
// This is the headline integration: drive
// `AtomicSporadicState::budget_remaining_us` from a real platform
// timer + the shipped `atomic_sporadic_refill_thunk`. Demonstrates
// the wake-up path the Executor will take inside
// `register_sporadic_timer` without pulling the rest of `nros-node`
// into this test crate.

#[test]
fn rust_trait_atomic_sporadic_refill_round_trip() {
    use nros_node::executor::sched_context::{AtomicSporadicState, atomic_sporadic_refill_thunk};
    use nros_platform_api::PlatformTimer;
    use std::sync::Arc;

    let state = Arc::new(AtomicSporadicState::new(10_000, 5_000));

    // Drain the budget so we can prove the refill thunk restored it.
    state.consume(10_000);
    assert!(
        !state.has_budget(),
        "budget should be exhausted before refill"
    );

    let user_data = Arc::as_ptr(&state) as *mut c_void;
    let handle = CffiPlatform::create_periodic(
        2_000, // 2 ms — refill fires several times within the wait window
        atomic_sporadic_refill_thunk,
        user_data,
    )
    .expect("create_periodic via Rust trait");

    let fired = wait_for(|| state.has_budget()).is_some();
    CffiPlatform::destroy(handle);

    assert!(
        fired,
        "the refill thunk never ran within {TIMER_DEADLINE:?} (2 ms period), so \
         the budget was never restored — the timer did not fire at all"
    );
    assert!(
        state.has_budget(),
        "atomic sporadic state should be refilled by the timer callback"
    );
    let remaining = state.budget_remaining_us.load(Ordering::Acquire);
    assert_eq!(
        remaining, 10_000,
        "refill thunk must restore budget to its declared capacity"
    );
}
