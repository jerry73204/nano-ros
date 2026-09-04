//! Phase 115.F — Custom-transport loopback example (talker).
//!
//! Demonstrates a fully-runtime-pluggable transport: the user
//! supplies four C function pointers that bridge raw zenoh wire
//! bytes to the medium of their choice. This example bridges to a
//! real zenohd over TCP, but the same vtable shape covers
//! USB-CDC, BLE GATT, RS-485 with framing, ring-buffer loopback,
//! semihosting bridge, and so on. See
//! `book/src/porting/custom-transport.md` for the full design.
//!
//! # Wire layout for this example
//!
//! ```text
//! talker (this binary)            zenohd (separate process)
//! ──────────────────              ─────────────
//! Publisher<String>               tcp/127.0.0.1:N
//!     │
//!     ▼
//! zenoh-pico session
//!     │ wire bytes via custom://
//!     ▼
//! NrosTransportOps callbacks ──tcp──▶ zenohd
//! ```
//!
//! The other end of the loop (a subscriber) runs the same shape
//! through the matching `custom-transport-listener` example.
//!
//! # Usage
//!
//! ```bash
//! # Start zenohd:
//! ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd
//!
//! # Run talker (bridges to that zenohd):
//! NROS_CUSTOM_TCP_TARGET=127.0.0.1:7447 cargo run -p native-rs-custom-transport-talker
//! ```

use std::time::Duration;

use core::fmt::Write as _;
use nros::prelude::*;
use nros_log::{Logger, log_error, log_info};
use std_msgs::msg::String as StringMsg;

// Phase 88.16.B — diagnostics route through `nros-log`.
static LOGGER: Logger = Logger::new("custom-transport-talker");

extern crate nros_platform_cffi as _;

// ============================================================================
// Main
// ============================================================================

fn main() {
    // Register the RMW backend the build linked (idempotent; must run before
    // the executor opens). RMW selection is build/config, never source.
    nros_board_linux::register_linked_rmw();

    nros_log::register_logger(&LOGGER);
    nros_log::init(nros_platform_cffi::log::default_sinks());

    let target = std::env::var("NROS_CUSTOM_TCP_TARGET").unwrap_or_else(|_| {
        log_info!(
            &LOGGER,
            "NROS_CUSTOM_TCP_TARGET not set; defaulting to 127.0.0.1:7447"
        );
        "127.0.0.1:7447".to_string()
    });

    log_info!(
        &LOGGER,
        "nros Custom-Transport Talker — bridging to TCP {target}"
    );

    // Phase 244 D4/E2 — the TCP-bridge custom-transport vtable now comes from
    // the reusable `nros-transport-callbacks` factory. The example just names a
    // transport endpoint and plugs it in; the bridge's TcpStream + the four
    // `extern "C"` callbacks live in the library. The factory `Box::leak`s its
    // backing state so `user_data` outlives the session (custom-transport
    // contract).
    let ops = match nros_transport_callbacks::tcp_transport_ops(&target) {
        Ok(ops) => ops,
        Err(e) => {
            log_error!(&LOGGER, "TCP connect to {target} failed: {e}");
            std::process::exit(1);
        }
    };

    // SAFETY: the factory leaks its backing state, so it lives until process exit.
    unsafe {
        nros_rmw::set_custom_transport(Some(ops)).expect("abi_version v1 ok");
    }
    log_info!(&LOGGER, "Custom transport vtable registered");

    // Phase 115.L.5-custom-transport — install zenoh-pico C-vtable
    // backend before Executor::open. Order matters: the custom-
    // transport slot set above is drained by zenoh-pico during
    // session open, so the cffi register must happen first so the
    // runtime knows which backend's open() to dispatch to.

    // Open zenoh session via the custom-link locator. Address is
    // opaque to v1; just needs to be non-empty.
    let config = ExecutorConfig::new("custom/loopback").node_name("talker");
    let mut executor: Executor = Executor::open(&config).expect("Failed to open session");

    let mut node = executor
        .create_node("talker")
        .expect("Failed to create node");

    let publisher = node
        .create_publisher::<StringMsg>("/chatter")
        .expect("Failed to create publisher");
    log_info!(&LOGGER, "Publisher created on /chatter");

    let max_msgs: i32 = std::env::var("NROS_TALKER_COUNT")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(20);

    for i in 1..=max_msgs {
        let mut msg = StringMsg::default();
        let _ = write!(msg.data, "Hello World: {i}");
        if let Err(e) = publisher.publish(&msg) {
            log_error!(&LOGGER, "Publish failed: {e:?}");
        } else {
            log_info!(&LOGGER, "Publishing: '{}'", msg.data);
        }
        std::thread::sleep(Duration::from_millis(100));
        // Drive session I/O so writes flush.
        let _ = executor.spin_once(Duration::from_millis(10));
    }

    log_info!(&LOGGER, "Talker done — published {max_msgs} messages");
}
