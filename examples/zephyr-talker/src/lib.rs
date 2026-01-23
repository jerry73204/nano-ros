//! nano-ros Zephyr Talker Example (Rust)
//!
//! This example demonstrates a ROS 2 compatible publisher running on
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

use core::ffi::c_char;

use log::{error, info};

// nano-ros CDR serialization
use nano_ros_core::{RosMessage, Serialize};
use nano_ros_serdes::CdrWriter;

// Generated message types
use std_msgs::msg::Int32;

// =============================================================================
// zenoh-pico shim FFI (from zenoh_shim.c)
// =============================================================================

extern "C" {
    fn zenoh_init_config(locator: *const c_char) -> i32;
    fn zenoh_open_session() -> i32;
    fn zenoh_is_session_open() -> i32;
    fn zenoh_declare_publisher(keyexpr: *const c_char) -> i32;
    fn zenoh_publish(handle: i32, data: *const u8, len: usize) -> i32;
    fn zenoh_undeclare_publisher(handle: i32) -> i32;
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
    PublisherError(i32),
    PublishError(i32),
    SerializationError,
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
    /// For Zephyr, this is handled by background tasks, so this is a no-op
    pub fn spin_once(&mut self, _timeout_ms: u64) {
        // zenoh-pico read/lease tasks run in background threads on Zephyr
    }
}

/// Zephyr node that can create publishers and subscribers
pub struct ZephyrNode<'a> {
    _name: &'a str,
    _namespace: &'a str,
}

impl<'a> ZephyrNode<'a> {
    /// Create a publisher for the given topic
    pub fn create_publisher<M: RosMessage>(&self, topic: &str) -> Result<ZephyrPublisher<M>, ZephyrError> {
        // Create null-terminated topic string
        let mut topic_buf = [0u8; 128];
        let topic_bytes = topic.as_bytes();
        if topic_bytes.len() >= topic_buf.len() {
            return Err(ZephyrError::PublisherError(-1));
        }
        topic_buf[..topic_bytes.len()].copy_from_slice(topic_bytes);
        topic_buf[topic_bytes.len()] = 0;

        let handle = unsafe { zenoh_declare_publisher(topic_buf.as_ptr() as *const c_char) };
        if handle < 0 {
            return Err(ZephyrError::PublisherError(handle));
        }

        info!("Declared publisher for '{}' (type: {})", topic, M::TYPE_NAME);

        Ok(ZephyrPublisher {
            handle,
            _phantom: core::marker::PhantomData,
        })
    }
}

/// Zephyr publisher for a specific message type
pub struct ZephyrPublisher<M> {
    handle: i32,
    _phantom: core::marker::PhantomData<M>,
}

impl<M: RosMessage + Serialize> ZephyrPublisher<M> {
    /// Publish a message
    pub fn publish(&self, msg: &M) -> Result<(), ZephyrError> {
        let mut buf = [0u8; 256];
        let mut writer = CdrWriter::new(&mut buf);

        msg.serialize(&mut writer).map_err(|_| ZephyrError::SerializationError)?;

        let len = writer.position();
        let ret = unsafe { zenoh_publish(self.handle, buf.as_ptr(), len) };

        if ret < 0 {
            return Err(ZephyrError::PublishError(ret));
        }

        Ok(())
    }
}

impl<M> Drop for ZephyrPublisher<M> {
    fn drop(&mut self) {
        unsafe {
            zenoh_undeclare_publisher(self.handle);
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

    info!("nano-ros Zephyr Talker Starting");
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
    let executor = ctx.create_polling_executor();
    let node = executor.create_node("talker", "/demo");

    // Create publisher
    let publisher: ZephyrPublisher<Int32> = match node.create_publisher("/chatter") {
        Ok(p) => p,
        Err(e) => {
            error!("Failed to create publisher: {:?}", e);
            return;
        }
    };

    // Publish loop
    let mut counter: i32 = 0;

    info!("Starting publish loop...");

    loop {
        let msg = Int32 { data: counter };

        match publisher.publish(&msg) {
            Ok(()) => {
                info!("[{}] Published: data={}", counter, counter);
            }
            Err(e) => {
                error!("Failed to publish: {:?}", e);
            }
        }

        counter = counter.wrapping_add(1);

        // Sleep 1 second
        zephyr::time::sleep(zephyr::time::Duration::secs(1));
    }
}
