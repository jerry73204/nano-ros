//! Native RTIC-pattern Service Server
//!
//! Validates the RTIC service server pattern on native x86:
//! - `Executor<_, 0, 0>` (zero callback arena)
//! - `spin_once(0)` (non-blocking I/O drive)
//! - `service.handle_request()` (manual polling)
//!
//! This is the native equivalent of `examples/stm32f4/rust/rtic-service-server/`.

use example_interfaces::srv::{AddTwoInts, AddTwoIntsResponse};
use nros::prelude::*;
use nros_log::{Logger, log_error, log_info};

// Diagnostics route through `nros-log`.
static LOGGER: Logger = Logger::new("service-server-rtic");

extern crate nros_platform_cffi as _;

fn main() {
    // Register the RMW backend the build linked (idempotent; must run before
    // the executor opens). RMW selection is build/config, never source.
    nros_board_linux::register_linked_rmw();

    nros_log::register_logger(&LOGGER);
    nros_log::init(nros_platform_cffi::log::default_sinks());

    log_info!(&LOGGER, "nros RTIC-pattern Service Server (native)");

    let config = ExecutorConfig::from_env().node_name("add_two_ints_server");
    let mut executor = Executor::open(&config).expect("Failed to open session");

    let mut node = executor
        .create_node("add_two_ints_server")
        .expect("Failed to create node");
    let mut service = node
        .create_service::<AddTwoInts>("/add_two_ints")
        .expect("Failed to create service");

    log_info!(&LOGGER, "Service server ready: /add_two_ints");
    log_info!(&LOGGER, "Waiting for service requests (RTIC pattern)...");

    let mut handled = 0u32;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);

    while std::time::Instant::now() < deadline {
        executor.spin_once(core::time::Duration::from_millis(0));

        match service.handle_request(|req| {
            log_info!(&LOGGER, "Incoming request");
            log_info!(&LOGGER, "a: {} b: {}", req.a, req.b);
            AddTwoIntsResponse { sum: req.a + req.b }
        }) {
            Ok(true) => handled += 1,
            Ok(false) => {}
            Err(e) => log_error!(&LOGGER, "Service error: {:?}", e),
        }

        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    log_info!(&LOGGER, "Done. Handled {} requests", handled);
}
