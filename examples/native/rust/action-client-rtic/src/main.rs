//! Native RTIC-pattern Action Client
//!
//! Validates the RTIC action client pattern on native x86:
//! - `Executor<_, 0, 0>` (zero callback arena)
//! - `spin_once(0)` (non-blocking I/O drive)
//! - `client.send_goal()` + `promise.take()` for acceptance
//! - `client.try_recv_feedback()` for feedback
//!
//! Note: `Promise::wait()` and `FeedbackStream::wait_next()` are NOT usable
//! in RTIC because they require `&mut Executor`. Use `take()` loops instead.
//!
//! This is the native equivalent of `examples/stm32f4/rust/rtic-action-client/`.

use example_interfaces::action::{Fibonacci, FibonacciGoal};
use nros::prelude::*;
use nros_log::{Logger, log_error, log_info};

// Diagnostics route through `nros-log`.
static LOGGER: Logger = Logger::new("action-client-rtic");

extern crate nros_platform_cffi as _;

fn main() {
    // Register the RMW backend the build linked (idempotent; must run before
    // the executor opens). RMW selection is build/config, never source.
    nros_board_linux::register_linked_rmw();

    nros_log::register_logger(&LOGGER);
    nros_log::init(nros_platform_cffi::log::default_sinks());

    log_info!(&LOGGER, "nros RTIC-pattern Action Client (native)");

    let config = ExecutorConfig::from_env().node_name("fibonacci_action_client");
    let mut executor = Executor::open(&config).expect("Failed to open session");

    let mut node = executor
        .create_node("fibonacci_action_client")
        .expect("Failed to create node");
    let mut client = node
        .create_action_client::<Fibonacci>("/fibonacci")
        .expect("Failed to create action client");

    log_info!(
        &LOGGER,
        "Action client created for /fibonacci (RTIC pattern)"
    );

    // Stabilization delay
    for _ in 0..300 {
        executor.spin_once(core::time::Duration::from_millis(0));
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    let goal = FibonacciGoal { order: 10 };
    log_info!(&LOGGER, "Sending goal");

    let (goal_id, mut promise) = client.send_goal(&goal).expect("Failed to send goal");

    // Poll for goal acceptance (~10s timeout)
    let mut accepted = false;
    for _ in 0..1000 {
        executor.spin_once(core::time::Duration::from_millis(0));
        if let Ok(Some(result)) = promise.take() {
            accepted = result;
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    if !accepted {
        log_error!(&LOGGER, "Goal not accepted (timeout)");
        std::process::exit(1);
    }
    log_info!(&LOGGER, "Goal accepted by server, waiting for result");

    // Receive feedback via try_recv_feedback() loop; this client is
    // feedback-stream-only, so the final/full sequence doubles as the result.
    let mut got_result = false;
    for _ in 0..500 {
        executor.spin_once(core::time::Duration::from_millis(0));

        if let Ok(Some((id, feedback))) = client.try_recv_feedback()
            && id.uuid == goal_id.uuid
        {
            log_info!(
                &LOGGER,
                "Next number in sequence received: {:?}",
                &feedback.sequence[..]
            );
            if feedback.sequence.len() as i32 > goal.order {
                log_info!(&LOGGER, "Result received: {:?}", &feedback.sequence[..]);
                got_result = true;
                break;
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    if !got_result {
        log_error!(&LOGGER, "Timeout waiting for the full sequence");
        std::process::exit(1);
    }
}
