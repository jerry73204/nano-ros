//! nano-ros Zephyr Talker Example (Rust)
//!
//! This example demonstrates a ROS 2 compatible publisher running on
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

use core::marker::PhantomData;

use log::{error, info};

// nano-ros CDR serialization
use nano_ros_core::{RosMessage, Serialize};
use nano_ros_serdes::CdrWriter;

// zenoh-pico shim crate
use zenoh_pico_shim::{ShimContext, ShimError, ShimPublisher};

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
    /// Serialization error
    SerializationError,
    /// Topic name too long
    TopicTooLong,
}

impl From<ShimError> for ZephyrError {
    fn from(err: ShimError) -> Self {
        ZephyrError::Shim(err)
    }
}

/// Zephyr node that provides a high-level API for publishers
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

    /// Create a publisher for the given topic
    pub fn create_publisher<M: RosMessage>(
        &self,
        topic: &str,
    ) -> Result<ZephyrPublisher<'a, M>, ZephyrError> {
        // Create null-terminated topic string
        let mut topic_buf = [0u8; 128];
        let topic_bytes = topic.as_bytes();
        if topic_bytes.len() >= topic_buf.len() {
            return Err(ZephyrError::TopicTooLong);
        }
        topic_buf[..topic_bytes.len()].copy_from_slice(topic_bytes);
        topic_buf[topic_bytes.len()] = 0;

        let publisher = self.ctx.declare_publisher(&topic_buf[..=topic_bytes.len()])?;

        info!("Declared publisher for '{}' (type: {})", topic, M::TYPE_NAME);

        Ok(ZephyrPublisher {
            publisher,
            _phantom: PhantomData,
        })
    }
}

/// Zephyr publisher for a specific message type
pub struct ZephyrPublisher<'a, M> {
    publisher: ShimPublisher<'a>,
    _phantom: PhantomData<M>,
}

impl<M: RosMessage + Serialize> ZephyrPublisher<'_, M> {
    /// Publish a message
    pub fn publish(&self, msg: &M) -> Result<(), ZephyrError> {
        let mut buf = [0u8; 256];
        let mut writer = CdrWriter::new(&mut buf);

        msg.serialize(&mut writer)
            .map_err(|_| ZephyrError::SerializationError)?;

        let len = writer.position();
        self.publisher
            .publish(&buf[..len])
            .map_err(ZephyrError::Shim)
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
    let node = ZephyrNode::new(&ctx, "talker", "/demo");

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
