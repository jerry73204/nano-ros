//! Native RTIC-pattern Service Client
//!
//! Validates the RTIC service client pattern on native x86:
//! - `Executor<_, 0, 0>` (zero callback arena)
//! - `spin_once(0)` (non-blocking I/O drive)
//! - `client.call()` + `promise.take()` loop (manual polling)
//!
//! Note: `Promise::wait()` is NOT usable in RTIC because it requires `&mut Executor`,
//! which is `#[local]` to the `net_poll` task. Use `take()` loop instead.
//!
//! This is the native equivalent of `examples/stm32f4/rust/rtic-service-client/`.

use example_interfaces::srv::{AddTwoInts, AddTwoIntsRequest};
use nros::prelude::*;
use nros_log::{Logger, log_error, log_info};

// Diagnostics route through `nros-log`.
static LOGGER: Logger = Logger::new("service-client-rtic");

extern crate nros_platform_cffi as _;

fn main() {
    // Register the RMW backend the build linked (idempotent; must run before
    // the executor opens). RMW selection is build/config, never source.
    nros_board_linux::register_linked_rmw();

    nros_log::register_logger(&LOGGER);
    nros_log::init(nros_platform_cffi::log::default_sinks());

    log_info!(&LOGGER, "nros RTIC-pattern Service Client (native)");

    let config = ExecutorConfig::from_env().node_name("add_two_ints_client");
    let mut executor = Executor::open(&config).expect("Failed to open session");

    let mut node = executor
        .create_node("add_two_ints_client")
        .expect("Failed to create node");
    let mut client = node
        .create_client::<AddTwoInts>("/add_two_ints")
        .expect("Failed to create client");

    log_info!(
        &LOGGER,
        "Service client created for /add_two_ints (RTIC pattern)"
    );

    // Stabilization delay
    for _ in 0..200 {
        executor.spin_once(core::time::Duration::from_millis(0));
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    // One fixed request — mirrors the embedded RTIC clients (no argv).
    let request = AddTwoIntsRequest { a: 2, b: 3 };

    let mut promise = match client.call(&request) {
        Ok(p) => p,
        Err(e) => {
            log_error!(&LOGGER, "Failed to send request: {:?}", e);
            std::process::exit(1);
        }
    };

    // Poll for reply with timeout (~30 seconds) — RTIC-compatible pattern.
    // Only one request can be in flight per client, so give the single
    // query generous discovery slack instead of re-issuing it.
    let mut got_reply = false;
    for _ in 0..3000 {
        executor.spin_once(core::time::Duration::from_millis(0));

        if let Ok(Some(reply)) = promise.take() {
            log_info!(&LOGGER, "Result of add_two_ints: {}", reply.sum);
            got_reply = true;
            break;
        }

        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    if !got_reply {
        log_error!(&LOGGER, "Timeout waiting for reply");
        std::process::exit(1);
    }
}
