//! nano-ros Zephyr Listener Example (Rust)
//!
//! This example demonstrates a ROS 2 compatible subscriber running on
//! Zephyr RTOS using the nano-ros executor API pattern.
//!
//! Architecture:
//! ```text
//! Rust Application (this file)
//!     └── ZephyrExecutor (Rust wrapper)
//!         └── zenoh_shim.c (C layer)
//!             └── zenoh-pico (C library)
//!                 └── Zephyr network stack
//! ```

#![no_std]

use core::ffi::{c_char, c_void};
use core::sync::atomic::{AtomicU32, Ordering};

use log::{error, info};

// nano-ros CDR serialization
use nano_ros_core::{Deserialize, RosMessage};
use nano_ros_serdes::CdrReader;

// Generated message types
use std_msgs::msg::Int32;

// =============================================================================
// zenoh-pico shim FFI (from zenoh_shim.c)
// =============================================================================

type RustSampleCallback = extern "C" fn(data: *const u8, len: usize, ctx: *mut c_void);

extern "C" {
    fn zenoh_init_config(locator: *const c_char) -> i32;
    fn zenoh_open_session() -> i32;
    fn zenoh_is_session_open() -> i32;
    fn zenoh_declare_subscriber(
        keyexpr: *const c_char,
        callback: RustSampleCallback,
        ctx: *mut c_void,
    ) -> i32;
    fn zenoh_undeclare_subscriber(handle: i32) -> i32;
    fn zenoh_close();
}

// =============================================================================
// Zephyr Executor API (mirrors native executor pattern)
// =============================================================================

/// Error type for Zephyr transport operations
#[derive(Debug, Clone, Copy)]
pub enum ZephyrError {
    ConfigError(i32),
    SessionError(i32),
    SubscriberError(i32),
    DeserializationError,
    NotConnected,
}

/// Zephyr-specific context that manages the zenoh session
pub struct ZephyrContext {
    _private: (),
}

impl ZephyrContext {
    /// Create a new context and connect to the zenoh router
    pub fn new(locator: &[u8]) -> Result<Self, ZephyrError> {
        unsafe {
            let ret = zenoh_init_config(locator.as_ptr() as *const c_char);
            if ret < 0 {
                return Err(ZephyrError::ConfigError(ret));
            }

            let ret = zenoh_open_session();
            if ret < 0 {
                return Err(ZephyrError::SessionError(ret));
            }
        }

        Ok(Self { _private: () })
    }

    /// Check if the session is open
    pub fn is_connected(&self) -> bool {
        unsafe { zenoh_is_session_open() == 1 }
    }

    /// Create a polling executor from this context
    pub fn create_polling_executor(&self) -> ZephyrExecutor {
        ZephyrExecutor { _private: () }
    }
}

impl Drop for ZephyrContext {
    fn drop(&mut self) {
        unsafe {
            zenoh_close();
        }
    }
}

/// Zephyr polling executor (manages nodes)
pub struct ZephyrExecutor {
    _private: (),
}

impl ZephyrExecutor {
    /// Create a node within this executor
    pub fn create_node(&self, name: &str, namespace: &str) -> ZephyrNode {
        info!("Created node: {}/{}", namespace, name);
        ZephyrNode {
            _name: name,
            _namespace: namespace,
        }
    }

    /// Spin once - process any pending callbacks
    /// For Zephyr, callbacks are invoked from background tasks
    pub fn spin_once(&mut self, _timeout_ms: u64) {
        // zenoh-pico read/lease tasks run in background threads on Zephyr
        // Callbacks are invoked directly from those tasks
    }
}

/// Zephyr node that can create publishers and subscribers
pub struct ZephyrNode<'a> {
    _name: &'a str,
    _namespace: &'a str,
}

impl<'a> ZephyrNode<'a> {
    /// Create a subscriber for the given topic with a callback
    ///
    /// The callback function receives the raw payload bytes. For type-safe
    /// message handling, use `create_subscription` instead.
    pub fn create_subscriber_raw(
        &self,
        topic: &str,
        callback: RustSampleCallback,
        ctx: *mut c_void,
    ) -> Result<ZephyrSubscriber, ZephyrError> {
        // Create null-terminated topic string
        let mut topic_buf = [0u8; 128];
        let topic_bytes = topic.as_bytes();
        if topic_bytes.len() >= topic_buf.len() {
            return Err(ZephyrError::SubscriberError(-1));
        }
        topic_buf[..topic_bytes.len()].copy_from_slice(topic_bytes);
        topic_buf[topic_bytes.len()] = 0;

        let handle = unsafe {
            zenoh_declare_subscriber(topic_buf.as_ptr() as *const c_char, callback, ctx)
        };
        if handle < 0 {
            return Err(ZephyrError::SubscriberError(handle));
        }

        info!("Declared subscriber for '{}'", topic);

        Ok(ZephyrSubscriber { handle })
    }
}

/// Zephyr subscriber handle
pub struct ZephyrSubscriber {
    handle: i32,
}

impl Drop for ZephyrSubscriber {
    fn drop(&mut self) {
        unsafe {
            zenoh_undeclare_subscriber(self.handle);
        }
    }
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

    // Create context (connects to zenoh)
    let ctx = match ZephyrContext::new(locator) {
        Ok(ctx) => ctx,
        Err(e) => {
            error!("Failed to create context: {:?}", e);
            return;
        }
    };
    info!("Session opened");

    // Create executor and node (mirrors native API)
    let mut executor = ctx.create_polling_executor();
    let node = executor.create_node("listener", "/demo");

    // Create subscriber for Int32 messages
    let _subscriber = match node.create_subscriber_raw(
        "/chatter",
        on_int32_message,
        core::ptr::null_mut(),
    ) {
        Ok(s) => s,
        Err(e) => {
            error!("Failed to create subscriber: {:?}", e);
            return;
        }
    };

    info!("Waiting for messages on /chatter...");

    // Main loop - callbacks run in background
    loop {
        // Spin the executor (no-op for Zephyr, but mirrors native API)
        executor.spin_once(1000);

        // Sleep and report status periodically
        zephyr::time::sleep(zephyr::time::Duration::secs(10));
        let count = MSG_COUNT.load(Ordering::Relaxed);
        info!("Total messages received: {}", count);
    }
}
