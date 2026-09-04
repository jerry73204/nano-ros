//! Native RTIC-pattern Talker
//!
//! Validates the RTIC integration pattern on native x86:
//! - `Executor<_, 0, 0>` (zero callback arena)
//! - `spin_once(0)` (non-blocking I/O drive)
//! - `publisher.publish()` (independent of executor)
//!
//! This is the native equivalent of `examples/stm32f4/rust/rtic-talker/`.

use core::fmt::Write as _;

use nros::prelude::*;
use nros_log::{Logger, log_error, log_info};
use std_msgs::msg::String as StringMsg;

// Phase 88.16.B — diagnostics route through `nros-log`.
static LOGGER: Logger = Logger::new("talker-rtic");

extern crate nros_platform_cffi as _;

fn main() {
    // Register the RMW backend the build linked (idempotent; must run before
    // the executor opens). RMW selection is build/config, never source.
    nros_board_linux::register_linked_rmw();

    nros_log::register_logger(&LOGGER);
    nros_log::init(nros_platform_cffi::log::default_sinks());

    log_info!(&LOGGER, "nros RTIC-pattern Talker (native)");

    let config = ExecutorConfig::from_env().node_name("talker");
    let mut executor = Executor::open(&config).expect("Failed to open session");

    let mut node = executor
        .create_node("talker")
        .expect("Failed to create node");
    let publisher = node
        .create_publisher::<StringMsg>("/chatter")
        .expect("Failed to create publisher");

    log_info!(&LOGGER, "Publishing on /chatter (RTIC pattern)...");

    // Stabilization delay (like RTIC Mono::delay(2000.millis()))
    for _ in 0..200 {
        executor.spin_once(core::time::Duration::from_millis(0));
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    let mut count: i32 = 0;
    loop {
        count = count.wrapping_add(1);
        let mut msg = StringMsg::default();
        let _ = write!(msg.data, "Hello World: {count}");
        match publisher.publish(&msg) {
            Ok(()) => log_info!(&LOGGER, "Publishing: '{}'", msg.data),
            Err(e) => log_error!(&LOGGER, "Publish error: {:?}", e),
        }

        // Drive I/O with spin_once(0) — non-blocking, like RTIC net_poll task
        for _ in 0..100 {
            executor.spin_once(core::time::Duration::from_millis(0));
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }
}
