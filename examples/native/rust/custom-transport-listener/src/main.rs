//! Phase 115.F — Custom-transport loopback example (listener).
//!
//! Mirror of `custom-transport-talker`. Subscribes to `/chatter`
//! over a custom-transport-bridged TCP connection to the same
//! zenohd. See the talker for the design walkthrough.

use std::time::Duration;

use nros::prelude::*;
use nros_log::{Logger, log_error, log_info};
use std_msgs::msg::String as StringMsg;

// Phase 88.16.B — diagnostics route through `nros-log`.
static LOGGER: Logger = Logger::new("custom-transport-listener");

extern crate nros_platform_cffi as _;

fn main() {
    // Register the RMW backend the build linked (idempotent; must run before
    // the executor opens). RMW selection is build/config, never source.
    nros_board_linux::register_linked_rmw();

    nros_log::register_logger(&LOGGER);
    nros_log::init(nros_platform_cffi::log::default_sinks());

    let target =
        std::env::var("NROS_CUSTOM_TCP_TARGET").unwrap_or_else(|_| "127.0.0.1:7447".to_string());

    log_info!(
        &LOGGER,
        "nros Custom-Transport Listener — bridging to TCP {target}"
    );

    // Phase 244 D4/E2 — the TCP-bridge custom-transport vtable comes from the
    // reusable `nros-transport-callbacks` factory (the TcpStream + the four
    // `extern "C"` callbacks live in the library; the factory `Box::leak`s its
    // backing state so `user_data` outlives the session). Mirrors the talker.
    let ops = match nros_transport_callbacks::tcp_transport_ops(&target) {
        Ok(ops) => ops,
        Err(e) => {
            log_error!(&LOGGER, "TCP connect to {target} failed: {e}");
            std::process::exit(1);
        }
    };
    // SAFETY: the factory leaks its backing state, so it lives until process exit.
    unsafe { nros_rmw::set_custom_transport(Some(ops)).expect("abi v1 ok") };
    log_info!(&LOGGER, "Custom transport vtable registered");

    // Phase 115.L.5-custom-transport — install zenoh-pico C-vtable
    // backend after staging the custom-transport slot (zenoh-pico
    // drains the slot during session_open).

    let config = ExecutorConfig::new("custom/loopback").node_name("listener");
    let mut executor: Executor = Executor::open(&config).expect("Failed to open session");

    let nid = executor
        .node_builder("listener")
        .build()
        .expect("Failed to build node");
    executor
        .node_mut(nid)
        .create_subscription::<StringMsg, _>("/chatter", |msg: &StringMsg| {
            log_info!(&LOGGER, "I heard: [{}]", msg.data);
        })
        .expect("Failed to add subscription");
    // Issue 0480 — the shared readiness spelling (`output::LISTENER_READY_MARKER`).
    // This bin and `serial-listener` were the only two saying "created on", a
    // second spelling of the same fact that no shared constant could cover.
    log_info!(&LOGGER, "Subscriber created for topic: /chatter");

    let max_secs: u64 = std::env::var("NROS_LISTENER_SECS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(10);

    let deadline = std::time::Instant::now() + Duration::from_secs(max_secs);
    while std::time::Instant::now() < deadline {
        let _ = executor.spin_once(Duration::from_millis(50));
    }

    log_info!(&LOGGER, "Listener done");
}
