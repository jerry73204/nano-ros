//! XRCE-DDS serial listener — subscribes to std_msgs/String on /chatter via serial transport.
//!
//! Uses the callback+spin pattern: registers a subscription callback, then
//! spins the executor which drives I/O and dispatches callbacks automatically.
//!
//! Environment variables:
//!   XRCE_SERIAL_PTY  — PTY device path (required)
//!   XRCE_DOMAIN_ID   — ROS domain ID (default: 0)
//!   XRCE_MSG_COUNT   — Messages to receive before exiting (default: 5)

use nros::{Executor, ExecutorConfig};
use std::{
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    },
    time::Instant,
};
use std_msgs::msg::String as StringMsg;

use nros_log::{Logger, log_info, log_warn};

// Phase 88.16.B — diagnostics route through `nros-log`.
static LOGGER: Logger = Logger::new("serial-listener");

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
    let msg_count: usize = std::env::var("XRCE_MSG_COUNT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(5);

    log_warn!(
        &LOGGER,
        "XRCE Serial Listener: pty={}, domain={}, count={}",
        pty_path,
        domain_id,
        msg_count
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
        .node_name("xrce_serial_listener");
    let mut executor: Executor = Executor::open(&config).expect("Failed to open XRCE session");
    log_warn!(&LOGGER, "Session created");

    // Register subscription callback
    let received = Arc::new(AtomicUsize::new(0));
    let received_cb = received.clone();
    let nid = executor
        .node_builder("xrce_serial_listener")
        .build()
        .expect("Failed to build node");
    executor
        .node_mut(nid)
        .create_subscription::<StringMsg, _>("/chatter", move |msg: &StringMsg| {
            let n = received_cb.fetch_add(1, Ordering::SeqCst) + 1;
            log_info!(&LOGGER, "[{}] I heard: [{}]", n, msg.data);
        })
        .expect("Failed to add subscription");
    // Issue 0480 — the shared readiness spelling (`output::LISTENER_READY_MARKER`).
    // Was "Subscriber created on /chatter", the second of two spellings for one
    // fact; the XRCE suites waited on the ambiguous "Waiting for" instead.
    log_warn!(&LOGGER, "Subscriber created for topic: /chatter");

    // Spin loop with timeout
    log_info!(&LOGGER, "Waiting for messages...");
    let start = Instant::now();
    let timeout = std::time::Duration::from_secs(30);

    while received.load(Ordering::SeqCst) < msg_count && start.elapsed() < timeout {
        executor.spin_once(core::time::Duration::from_millis(100));
    }

    let final_count = received.load(Ordering::SeqCst);
    if final_count >= msg_count {
        log_info!(&LOGGER, "Received {} messages, exiting", final_count);
    } else {
        log_warn!(
            &LOGGER,
            "Timeout: received only {}/{} messages",
            final_count,
            msg_count
        );
        std::process::exit(1);
    }

    // Clean up
    let _ = executor.close();
}
