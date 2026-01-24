//! nano-ros Zephyr Listener Example (Rust)
//!
//! This example demonstrates a ROS 2 compatible subscriber running on
//! Zephyr RTOS using the zenoh-pico-shim crate.
//!
//! Architecture:
//! ```text
//! Rust Application (this file)
//!     └── zenoh-pico-shim (Rust wrapper)
//!         └── zenoh_shim.c (C shim, compiled by Zephyr)
//!             └── zenoh-pico (C library)
//!                 └── Zephyr network stack
//! ```

#![no_std]

use core::ffi::c_void;
use core::sync::atomic::{AtomicU32, Ordering};

use log::{error, info};

// nano-ros CDR serialization
use nano_ros_core::{Deserialize, RosMessage};
use nano_ros_serdes::CdrReader;

// zenoh-pico shim crate
use zenoh_pico_shim::{ShimCallback, ShimContext, ShimError, ShimSubscriber};

// Generated message types
use std_msgs::msg::Int32;

// =============================================================================
// Zephyr Node API (high-level wrapper over shim)
// =============================================================================

/// Error type for Zephyr operations
#[derive(Debug, Clone, Copy)]
pub enum ZephyrError {
    /// Shim error
    Shim(ShimError),
    /// Deserialization error
    DeserializationError,
    /// Topic name too long
    TopicTooLong,
}

impl From<ShimError> for ZephyrError {
    fn from(err: ShimError) -> Self {
        ZephyrError::Shim(err)
    }
}

/// Zephyr node that provides a high-level API for subscribers
///
/// This wraps the ShimContext and provides node naming support.
pub struct ZephyrNode<'a> {
    ctx: &'a ShimContext,
    #[allow(dead_code)] // Used for logging/diagnostics
    name: &'a str,
    #[allow(dead_code)] // Used for logging/diagnostics
    namespace: &'a str,
}

impl<'a> ZephyrNode<'a> {
    /// Create a new node with the given context
    pub fn new(ctx: &'a ShimContext, name: &'a str, namespace: &'a str) -> Self {
        info!("Created node: {}/{}", namespace, name);
        Self {
            ctx,
            name,
            namespace,
        }
    }

    /// Create a subscriber for the given topic with a raw callback
    ///
    /// # Safety
    ///
    /// The callback and context must remain valid for the lifetime of the subscriber.
    pub unsafe fn create_subscriber_raw(
        &self,
        topic: &str,
        callback: ShimCallback,
        ctx: *mut c_void,
    ) -> Result<ZephyrSubscriber<'a>, ZephyrError> {
        // Create null-terminated topic string
        let mut topic_buf = [0u8; 128];
        let topic_bytes = topic.as_bytes();
        if topic_bytes.len() >= topic_buf.len() {
            return Err(ZephyrError::TopicTooLong);
        }
        topic_buf[..topic_bytes.len()].copy_from_slice(topic_bytes);
        topic_buf[topic_bytes.len()] = 0;

        let subscriber = self
            .ctx
            .declare_subscriber_raw(&topic_buf[..=topic_bytes.len()], callback, ctx)?;

        info!("Declared subscriber for '{}'", topic);

        Ok(ZephyrSubscriber { subscriber })
    }
}

/// Zephyr subscriber handle
pub struct ZephyrSubscriber<'a> {
    #[allow(dead_code)] // Subscriber is kept alive for RAII drop
    subscriber: ShimSubscriber<'a>,
}

// =============================================================================
// Message callback and handling
// =============================================================================

/// Counter for received messages
static MSG_COUNT: AtomicU32 = AtomicU32::new(0);

/// Callback invoked when a message is received
extern "C" fn on_int32_message(data: *const u8, len: usize, _ctx: *mut c_void) {
    let count = MSG_COUNT.fetch_add(1, Ordering::Relaxed);

    // Safety: data is valid for len bytes, provided by C shim
    let payload = unsafe { core::slice::from_raw_parts(data, len) };

    // Deserialize the Int32 message
    let mut reader = CdrReader::new(payload);
    match Int32::deserialize(&mut reader) {
        Ok(msg) => {
            info!("[{}] Received: data={} ({} bytes)", count, msg.data, len);
        }
        Err(_) => {
            info!(
                "[{}] Received {} bytes (deserialization failed)",
                count, len
            );
        }
    }
}

// =============================================================================
// Main entry point
// =============================================================================

/// Entry point for Zephyr (called by zephyr-lang-rust)
#[no_mangle]
extern "C" fn rust_main() {
    // Initialize logging
    unsafe {
        zephyr::set_logger().ok();
    }

    info!("nano-ros Zephyr Listener Starting");
    info!("Board: {}", zephyr::kconfig::CONFIG_BOARD);

    // Connection parameters (for QEMU/native_sim, connect to host at 192.0.2.2)
    let locator = b"tcp/192.0.2.2:7447\0";

    info!("Connecting to zenoh router at tcp/192.0.2.2:7447");

    // Create ShimContext (connects to zenoh)
    let ctx = match ShimContext::new(locator) {
        Ok(ctx) => ctx,
        Err(e) => {
            error!("Failed to create context: {}", e);
            return;
        }
    };
    info!("Session opened");

    // Create node (high-level wrapper)
    let node = ZephyrNode::new(&ctx, "listener", "/demo");

    // Create subscriber for Int32 messages
    let _subscriber = match unsafe {
        node.create_subscriber_raw("/chatter", on_int32_message, core::ptr::null_mut())
    } {
        Ok(s) => s,
        Err(e) => {
            error!("Failed to create subscriber: {:?}", e);
            return;
        }
    };

    info!("Waiting for messages on /chatter...");

    // Main loop - callbacks run in background threads on Zephyr
    loop {
        // Sleep and report status periodically
        zephyr::time::sleep(zephyr::time::Duration::secs(10));
        let count = MSG_COUNT.load(Ordering::Relaxed);
        info!("Total messages received: {}", count);
    }
}
