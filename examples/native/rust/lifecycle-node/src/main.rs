//! Native Lifecycle Node Example (REP-2002)
//!
//! Implements the safe `nros::lifecycle::LifecycleCallbacks` trait — the Rust
//! counterpart to rclcpp's `LifecycleNode` override shape. Registers the five
//! ROS 2 lifecycle services on a node, wires up the transition callbacks, and
//! spins indefinitely so `ros2 lifecycle set|get` can drive the state machine
//! from another terminal.
//!
//! # Usage
//!
//! ```bash
//! # Start the router:
//! ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd
//!
//! # Run the node:
//! cargo run -p native-rs-lifecycle-node
//!
//! # From another terminal (requires a ROS 2 install + rmw_zenoh):
//! ros2 lifecycle nodes
//! ros2 lifecycle get /lifecycle_demo
//! ros2 lifecycle set /lifecycle_demo configure
//! ros2 lifecycle set /lifecycle_demo activate
//! ros2 lifecycle set /lifecycle_demo deactivate
//! ros2 lifecycle set /lifecycle_demo cleanup
//! ros2 lifecycle list /lifecycle_demo
//! ```

use core::time::Duration;

use nros::{
    Executor,
    ExecutorConfig,
    // `ExecutorConfig::from_env()` is an extension trait: nano-ros reads the
    // process environment at the `nros` edge, so the core works on targets that
    // have no environment at all. `use nros::prelude::*` brings it in too.
    ExecutorConfigEnvExt as _,
    lifecycle::{LifecycleCallbacks, TransitionResult},
};
use nros_log::{Logger, log_info};

// Phase 88.16.B — diagnostics route through `nros-log`.
static LOGGER: Logger = Logger::new("lifecycle-node");

extern crate nros_platform_cffi as _;

/// The managed node. Implement the transitions you care about; the rest default
/// to `Success` (`on_error` to `Failure`), exactly like rclcpp's
/// `LifecycleNodeInterface` and the C++ `nros::LifecycleNode`.
struct LifecycleDemo;

impl LifecycleCallbacks for LifecycleDemo {
    fn on_configure(&mut self) -> TransitionResult {
        log_info!(&LOGGER, "[callback] on_configure — allocating resources");
        TransitionResult::Success
    }

    fn on_activate(&mut self) -> TransitionResult {
        log_info!(&LOGGER, "[callback] on_activate — starting work");
        TransitionResult::Success
    }

    fn on_deactivate(&mut self) -> TransitionResult {
        log_info!(&LOGGER, "[callback] on_deactivate — pausing work");
        TransitionResult::Success
    }

    fn on_cleanup(&mut self) -> TransitionResult {
        log_info!(&LOGGER, "[callback] on_cleanup — releasing resources");
        TransitionResult::Success
    }

    fn on_shutdown(&mut self) -> TransitionResult {
        log_info!(&LOGGER, "[callback] on_shutdown — finalizing");
        TransitionResult::Success
    }
}

fn main() {
    // Register the RMW backend the build linked (idempotent; must run before
    // the executor opens). RMW selection is build/config, never source.
    nros_board_linux::register_linked_rmw();

    nros_log::register_logger(&LOGGER);
    nros_log::init(nros_platform_cffi::log::default_sinks());
    log_info!(&LOGGER, "Lifecycle demo starting…");

    // `node` is declared before `executor` so it outlives the lifecycle
    // registration (the executor holds a context pointer to it for the run).
    let mut node = LifecycleDemo;

    let config = ExecutorConfig::from_env().node_name("lifecycle_demo");
    let mut executor: Executor = Executor::open(&config).expect("Failed to open session");

    // Binds the five REP-2002 services and wires each transition to the trait
    // impl above — no `unsafe`, no `extern "C"`.
    executor
        .register_lifecycle_node(&mut node)
        .expect("Failed to register lifecycle node");
    log_info!(&LOGGER, "Lifecycle services registered on /lifecycle_demo");

    log_info!(
        &LOGGER,
        "Ready. Drive the lifecycle with `ros2 lifecycle set /lifecycle_demo configure`, etc."
    );

    // Spin indefinitely. Each spin_once drains the lifecycle services so
    // `ros2 lifecycle` queries round-trip.
    loop {
        let _ = executor.spin_once(Duration::from_millis(100));
    }
}
