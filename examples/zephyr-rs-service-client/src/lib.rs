//! nano-ros Zephyr Service Client Example (Rust)
//!
//! This example demonstrates a ROS 2 compatible service client running on
//! Zephyr RTOS using the zenoh-pico-shim crate.
//!
//! The client sends AddTwoInts service requests.
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

use log::{error, info};

// nano-ros CDR serialization
use nano_ros_core::{Deserialize, RosMessage, Serialize};
use nano_ros_serdes::{CdrReader, CdrWriter};

// zenoh-pico shim crate
use zenoh_pico_shim::{ShimContext, ShimError};

// Generated service types
use example_interfaces::srv::{AddTwoIntsRequest, AddTwoIntsResponse};

// =============================================================================
// Zephyr Service Client
// =============================================================================

/// Error type for service client operations
#[derive(Debug, Clone, Copy)]
pub enum ServiceError {
    /// Shim error
    Shim(ShimError),
    /// Serialization error
    SerializationError,
    /// Deserialization error
    DeserializationError,
    /// Key expression too long
    KeyExprTooLong,
    /// Timeout waiting for response
    Timeout,
}

impl From<ShimError> for ServiceError {
    fn from(err: ShimError) -> Self {
        ServiceError::Shim(err)
    }
}

/// Zephyr service client for AddTwoInts
pub struct ZephyrServiceClient<'a> {
    ctx: &'a ShimContext,
    keyexpr: [u8; 256],
    keyexpr_len: usize,
}

impl<'a> ZephyrServiceClient<'a> {
    /// Create a new service client
    pub fn new(ctx: &'a ShimContext, service_name: &str) -> Result<Self, ServiceError> {
        let mut keyexpr = [0u8; 256];
        let keyexpr_bytes = service_name.as_bytes();
        if keyexpr_bytes.len() >= keyexpr.len() {
            return Err(ServiceError::KeyExprTooLong);
        }
        keyexpr[..keyexpr_bytes.len()].copy_from_slice(keyexpr_bytes);
        keyexpr[keyexpr_bytes.len()] = 0;

        info!("Created service client for '{}'", service_name);

        Ok(Self {
            ctx,
            keyexpr,
            keyexpr_len: keyexpr_bytes.len() + 1,
        })
    }

    /// Send a service request and wait for response
    pub fn call(
        &self,
        request: &AddTwoIntsRequest,
        timeout_ms: u32,
    ) -> Result<AddTwoIntsResponse, ServiceError> {
        // Serialize request
        let mut request_buf = [0u8; 256];
        let mut writer = CdrWriter::new(&mut request_buf);
        request
            .serialize(&mut writer)
            .map_err(|_| ServiceError::SerializationError)?;
        let request_len = writer.position();

        // Send query and wait for reply
        let mut reply_buf = [0u8; 256];
        let reply_len = self
            .ctx
            .get(
                &self.keyexpr[..self.keyexpr_len],
                &request_buf[..request_len],
                &mut reply_buf,
                timeout_ms,
            )
            .map_err(|e| match e {
                ShimError::Timeout => ServiceError::Timeout,
                other => ServiceError::Shim(other),
            })?;

        // Deserialize response
        let mut reader = CdrReader::new(&reply_buf[..reply_len]);
        AddTwoIntsResponse::deserialize(&mut reader)
            .map_err(|_| ServiceError::DeserializationError)
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

    info!("nano-ros Zephyr Service Client Starting");
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

    // Create service client
    let client = match ZephyrServiceClient::new(&ctx, "demo/add_two_ints") {
        Ok(c) => c,
        Err(e) => {
            error!("Failed to create service client: {:?}", e);
            return;
        }
    };

    info!("Service client ready: demo/add_two_ints");
    info!("Sending service requests...");
    info!("(Make sure a service server is running)");

    // Allow some time for connection to stabilize
    zephyr::time::sleep(zephyr::time::Duration::secs(2));

    // Send service requests in a loop
    let mut count: i64 = 0;

    loop {
        // Create request
        let request = AddTwoIntsRequest {
            a: count,
            b: count + 1,
        };

        info!("[{}] Sending request: {} + {}", count, request.a, request.b);

        // Call service with 5 second timeout
        match client.call(&request, 5000) {
            Ok(response) => {
                info!(
                    "[{}] Received response: sum={}",
                    count, response.sum
                );
            }
            Err(ServiceError::Timeout) => {
                error!("[{}] Request timed out", count);
            }
            Err(e) => {
                error!("[{}] Service call failed: {:?}", count, e);
            }
        }

        count += 1;

        // Sleep between requests
        zephyr::time::sleep(zephyr::time::Duration::secs(2));
    }
}
