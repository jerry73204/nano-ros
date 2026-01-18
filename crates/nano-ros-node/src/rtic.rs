//! RTIC integration for nano-ros
//!
//! This module provides helpers for integrating nano-ros with RTIC (Real-Time
//! Interrupt-driven Concurrency) applications.
//!
//! # Overview
//!
//! RTIC applications use cooperative multitasking without background threads.
//! This requires manually driving the zenoh session through periodic polling.
//!
//! # Required Tasks
//!
//! 1. **zenoh_poll** - Process incoming messages (every 10ms recommended)
//! 2. **zenoh_keepalive** - Maintain session connection (every 1s recommended)
//! 3. **zenoh_join** (optional) - Peer discovery in peer mode (every 1s)
//!
//! # Example RTIC Application
//!
//! ```ignore
//! #![no_std]
//! #![no_main]
//!
//! use rtic::app;
//! use nano_ros_node::{ConnectedNode, NodeConfig};
//! use nano_ros_transport::SessionMode;
//!
//! #[app(device = stm32f4xx_hal::pac, dispatchers = [SPI1, SPI2])]
//! mod app {
//!     use super::*;
//!     use rtic_monotonics::systick::Systick;
//!
//!     #[shared]
//!     struct Shared {
//!         node: ConnectedNode,
//!     }
//!
//!     #[local]
//!     struct Local {}
//!
//!     #[init]
//!     fn init(cx: init::Context) -> (Shared, Local) {
//!         // Initialize monotonic timer
//!         let systick = cx.core.SYST;
//!         Systick::start(systick, 168_000_000);
//!
//!         // Create node without background tasks
//!         let config = NodeConfig::new("my_node", "/");
//!         let node = ConnectedNode::connect_without_tasks(
//!             config,
//!             "tcp/192.168.1.1:7447",
//!             SessionMode::Client,
//!         ).expect("Failed to connect");
//!
//!         // Spawn periodic tasks
//!         zenoh_poll::spawn().ok();
//!         zenoh_keepalive::spawn().ok();
//!
//!         (Shared { node }, Local {})
//!     }
//!
//!     /// Poll zenoh for incoming messages (every 10ms)
//!     #[task(priority = 2, shared = [node])]
//!     async fn zenoh_poll(mut cx: zenoh_poll::Context) {
//!         use rtic_monotonics::systick::Systick;
//!
//!         loop {
//!             cx.shared.node.lock(|node| {
//!                 if let Err(e) = node.poll_read() {
//!                     // Handle error (log, retry, etc.)
//!                 }
//!             });
//!             Systick::delay(10.millis()).await;
//!         }
//!     }
//!
//!     /// Send keepalive to maintain session (every 1s)
//!     #[task(priority = 1, shared = [node])]
//!     async fn zenoh_keepalive(mut cx: zenoh_keepalive::Context) {
//!         use rtic_monotonics::systick::Systick;
//!
//!         loop {
//!             cx.shared.node.lock(|node| {
//!                 if let Err(e) = node.send_keepalive() {
//!                     // Handle error
//!                 }
//!             });
//!             Systick::delay(1000.millis()).await;
//!         }
//!     }
//!
//!     /// Application task using the node
//!     #[task(priority = 1, shared = [node])]
//!     async fn publisher_task(mut cx: publisher_task::Context) {
//!         use rtic_monotonics::systick::Systick;
//!         use nano_ros_types::std_msgs::Int32;
//!
//!         // Create publisher (one-time setup)
//!         let publisher = cx.shared.node.lock(|node| {
//!             node.create_publisher::<Int32>("/counter")
//!         }).expect("Failed to create publisher");
//!
//!         let mut count = 0i32;
//!         loop {
//!             publisher.publish(&Int32 { data: count }).ok();
//!             count = count.wrapping_add(1);
//!             Systick::delay(100.millis()).await;
//!         }
//!     }
//! }
//! ```
//!
//! # Recommended Task Priorities
//!
//! | Task | Priority | Rationale |
//! |------|----------|-----------|
//! | zenoh_poll | Higher (2+) | Timely message processing |
//! | zenoh_keepalive | Lower (1) | Less time-sensitive |
//! | Application tasks | Varies | Based on requirements |
//!
//! # Feature Flags
//!
//! Enable these features in `nano-ros-transport`:
//! - `rtic` - RTIC executor support
//! - `sync-critical-section` - RTIC-compatible mutex
//!
//! ```toml
//! [dependencies]
//! nano-ros-transport = { version = "0.1", features = ["zenoh", "rtic", "sync-critical-section"] }
//! ```

/// Recommended poll interval in milliseconds
pub const POLL_INTERVAL_MS: u32 = 10;

/// Recommended keepalive interval in milliseconds
pub const KEEPALIVE_INTERVAL_MS: u32 = 1000;

/// Recommended join interval in milliseconds (for peer mode)
pub const JOIN_INTERVAL_MS: u32 = 1000;

/// Generates RTIC task stubs for zenoh polling
///
/// This macro generates the task function signatures that you can copy
/// into your RTIC application. You'll need to adapt them to your specific
/// monotonic timer and priority requirements.
///
/// # Usage
///
/// ```ignore
/// // Generate task stubs
/// nano_ros_node::rtic_task_stubs!();
///
/// // Copy the generated code to your RTIC app and adapt as needed
/// ```
#[macro_export]
#[cfg(feature = "rtic")]
macro_rules! rtic_task_stubs {
    () => {
        /// Generated stub for zenoh polling task
        ///
        /// Copy this into your RTIC app and adapt the priority and monotonic.
        /// ```ignore
        /// #[task(priority = 2, shared = [node])]
        /// async fn zenoh_poll(mut cx: zenoh_poll::Context) {
        ///     loop {
        ///         cx.shared.node.lock(|node| {
        ///             let _ = node.poll_read();
        ///         });
        ///         Systick::delay(10.millis()).await;
        ///     }
        /// }
        /// ```
        const _ZENOH_POLL_STUB: () = ();

        /// Generated stub for keepalive task
        ///
        /// Copy this into your RTIC app and adapt the priority and monotonic.
        /// ```ignore
        /// #[task(priority = 1, shared = [node])]
        /// async fn zenoh_keepalive(mut cx: zenoh_keepalive::Context) {
        ///     loop {
        ///         cx.shared.node.lock(|node| {
        ///             let _ = node.send_keepalive();
        ///         });
        ///         Systick::delay(1000.millis()).await;
        ///     }
        /// }
        /// ```
        const _ZENOH_KEEPALIVE_STUB: () = ();
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_interval_constants() {
        assert_eq!(POLL_INTERVAL_MS, 10);
        assert_eq!(KEEPALIVE_INTERVAL_MS, 1000);
        assert_eq!(JOIN_INTERVAL_MS, 1000);
    }
}
