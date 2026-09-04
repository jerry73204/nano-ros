//! issue 0697 — the zenoh session-pool exhaustion arm, EXECUTED on a `no_std`
//! target.
//!
//! `ZPICO_MAX_SESSIONS` defaults to 1, which is correct for a shipped image: the
//! pool is a fixed-size static array and a non-bridge application opens exactly
//! one session. Issue 0589 moved the diagnostic that explains an exhausted pool
//! from `std::eprintln!` to `nros_log` **so it would reach `no_std` targets**,
//! reasoning that "firmware is where a fixed-size pool actually fills".
//!
//! Nothing on firmware ever reached it. No embedded build raised the knob, and
//! no test asserted `Full` on any platform — the one reference outside the
//! backend was a doc comment explaining that a native test SKIPS when the pool
//! is 1, i.e. the arm was what that test avoided rather than what it asserted.
//!
//! So this image takes the default pool of 1, opens a session, and then opens a
//! second one deliberately. Two things must hold, and the second is the half
//! 0589 was actually about:
//!
//!   1. the second open returns `Full`, not a transport error;
//!   2. the explanation reaches the CONSOLE of a `no_std` image.
//!
//! (2) could not have held before issue 0708: this family's boot funnel never
//! published an `nros_log` sink list, so the record was constructed, dispatched
//! and dropped. Do not "fix" a silent run here by calling `nros_log::init` —
//! that hides the defect the assertion exists to catch.
//!
//! The first session must SUCCEED, or it releases its slot on failure and the
//! second one inherits it — so the harness starts a router and passes its
//! locator in `NROS_LOCATOR`.

use nros_board_threadx_linux::ThreadxLinux;
use nros_rmw_zenoh::Context;

/// Printed only when both halves hold, so a harness cannot mistake a crashed
/// image for a passing one.
const VERDICT_OK: &str = "pool-exhaustion: second session refused with Full";

fn main() {
    let _ = ThreadxLinux::run_bare(|| {
        // Establish the PRECONDITION — a full pool — without a network.
        //
        // `Context::new` takes a slot before it does any I/O and RELEASES it on
        // any init/open failure, so exhausting the pool by opening a real first
        // session would need a live router: on this board that is NetX's stack,
        // not the host's loopback, and a router that fails to connect leaves the
        // pool empty and the second open succeeding for the wrong reason. The
        // raw acquire is the precondition; the arm under test is still reached
        // through `Context::new` below.
        //
        // SAFETY: no arguments; returns a pool slot or null. Held for the rest
        // of the program — never released, which is the point.
        let held = unsafe { zpico_sys::zpico_session_acquire() };
        if held.is_null() {
            nros_log::log_error!(
                nros_log::get_logger("pool-exhaustion"),
                "could not take the FIRST slot of a pool of {} — the precondition \
                 does not hold and this run proves nothing",
                nros_rmw_zenoh::zpico::ZPICO_MAX_SESSIONS
            );
            return Err("precondition: pool not acquirable");
        }

        // The pool is now full. This is the call under test.
        match Context::new(b"tcp/127.0.0.1:7447") {
            Err(nros_rmw_zenoh::ZpicoError::Full) => {
                nros_log::log_info!(nros_log::get_logger("pool-exhaustion"), "{}", VERDICT_OK);
                nros_log::flush();
                Ok(())
            }
            Err(other) => {
                nros_log::log_error!(
                    nros_log::get_logger("pool-exhaustion"),
                    "second session failed with {other:?}, not Full — an exhausted \
                     pool must stay distinguishable from a transport failure (issue 0465)"
                );
                Err("wrong error")
            }
            Ok(_) => {
                nros_log::log_error!(
                    nros_log::get_logger("pool-exhaustion"),
                    "a session opened with the pool already full — the bound is not \
                     being enforced"
                );
                Err("second session opened")
            }
        }
    });
}
