//! XRCE-DDS serial talker — publishes std_msgs/String on /chatter via serial transport.
//!
//! Uses the timer+spin pattern: registers a timer callback that publishes
//! messages periodically, then spins the executor.
//!
//! Environment variables:
//!   XRCE_SERIAL_PTY  — PTY device path (required)
//!   XRCE_DOMAIN_ID   — ROS domain ID (default: 0)

use core::fmt::Write as _;

use nros::{Executor, ExecutorConfig, TimerDuration};
use std::sync::{
    Arc,
    atomic::{AtomicI32, Ordering},
};
use std_msgs::msg::String as StringMsg;

use nros_log::{Logger, log_info, log_warn};

// Phase 88.16.B — diagnostics route through `nros-log`.
static LOGGER: Logger = Logger::new("serial-talker");

extern crate nros_platform_cffi as _;

fn main() {
    // Register the RMW backend the build linked (idempotent; must run before
    // the executor opens). RMW selection is build/config, never source.
    nros_board_linux::register_linked_rmw();

    nros_log::register_logger(&LOGGER);
    nros_log::init(nros_platform_cffi::log::default_sinks());
    let pty_path = std::env::var("XRCE_SERIAL_PTY")
        .expect("XRCE_SERIAL_PTY must be set to the PTY device path");
    let domain_id: u32 = std::env::var("XRCE_DOMAIN_ID")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);

    log_warn!(
        &LOGGER,
        "XRCE Serial Talker: pty={}, domain={}",
        pty_path,
        domain_id
    );

    // Phase 115.K.2.5.1.5-serial — register the C XRCE backend's
    // vtable before opening the session. Idempotent.

    // The C backend's `session.c` parses `serial://<path>` (or a bare
    // `/dev/...` path) and routes to `xrce_posix_serial_init`. Wrap
    // arbitrary tty/pty paths so the `serial://` scheme drives the
    // selector regardless of where the device lives in the filesystem.
    let locator = if pty_path.starts_with("serial://") || pty_path.starts_with("/dev/") {
        pty_path.clone()
    } else {
        format!("serial://{pty_path}")
    };

    // Open session with callback arena
    let config = ExecutorConfig::new(&locator)
        .domain_id(domain_id)
        .node_name("xrce_serial_talker");
    let mut executor: Executor = Executor::open(&config).expect("Failed to open XRCE session");
    log_warn!(&LOGGER, "Session created");

    // Create publisher
    let mut node = executor
        .create_node("xrce_serial_talker")
        .expect("Failed to create node");
    let publisher = node
        .create_publisher::<StringMsg>("/chatter")
        .expect("Failed to create publisher");
    log_warn!(&LOGGER, "Publisher created on /chatter");

    // Register timer callback that publishes every 500ms
    let counter = Arc::new(AtomicI32::new(0));
    let counter_cb = counter.clone();
    executor
        .register_timer(TimerDuration::from_millis(500), move || {
            let i = counter_cb.fetch_add(1, Ordering::SeqCst) + 1;
            let mut msg = StringMsg::default();
            let _ = write!(msg.data, "Hello World: {i}");
            match publisher.publish(&msg) {
                Ok(()) => log_info!(&LOGGER, "Publishing: '{}'", msg.data),
                Err(e) => log_warn!(&LOGGER, "Publish error: {:?}", e),
            }
        })
        .expect("Failed to add timer");

    // Spin until 20 messages published
    while counter.load(Ordering::SeqCst) < 20 {
        executor.spin_once(core::time::Duration::from_millis(100));
    }

    // Clean up
    let _ = executor.close();
    log_warn!(&LOGGER, "Talker done");
}
