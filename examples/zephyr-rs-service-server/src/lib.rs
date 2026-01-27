//! nano-ros Zephyr Service Server Example (Rust)
//!
//! This example demonstrates a ROS 2 compatible service server running on
//! Zephyr RTOS using the zenoh-pico-shim crate.
//!
//! The server responds to AddTwoInts service requests.
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
use nano_ros_core::{Deserialize, RosMessage, Serialize};
use nano_ros_serdes::{CdrReader, CdrWriter};

// zenoh-pico shim crate
use zenoh_pico_shim::{ShimContext, ShimError, ShimQueryCallback, ShimQueryable};

// Generated service types
use example_interfaces::srv::{AddTwoIntsRequest, AddTwoIntsResponse};

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
    /// Key expression too long
    KeyExprTooLong,
}

impl From<ShimError> for ZephyrError {
    fn from(err: ShimError) -> Self {
        ZephyrError::Shim(err)
    }
}

/// Zephyr node that provides a high-level API for services
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

    /// Create a service server with a raw callback
    ///
    /// # Safety
    ///
    /// The callback and context must remain valid for the lifetime of the queryable.
    pub unsafe fn create_service_raw(
        &self,
        service_name: &str,
        callback: ShimQueryCallback,
        ctx: *mut c_void,
    ) -> Result<ZephyrServiceServer<'a>, ZephyrError> {
        // Create null-terminated key expression
        let mut keyexpr_buf = [0u8; 256];
        let keyexpr_bytes = service_name.as_bytes();
        if keyexpr_bytes.len() >= keyexpr_buf.len() {
            return Err(ZephyrError::KeyExprTooLong);
        }
        keyexpr_buf[..keyexpr_bytes.len()].copy_from_slice(keyexpr_bytes);
        keyexpr_buf[keyexpr_bytes.len()] = 0;

        let queryable = self
            .ctx
            .declare_queryable_raw(&keyexpr_buf[..=keyexpr_bytes.len()], callback, ctx)?;

        info!("Declared service server for '{}'", service_name);

        Ok(ZephyrServiceServer { queryable })
    }
}

/// Zephyr service server handle
pub struct ZephyrServiceServer<'a> {
    #[allow(dead_code)] // Queryable is kept alive for RAII drop
    queryable: ShimQueryable,
    // Phantom to keep 'a alive (future: store context ref)
    #[allow(dead_code)]
    _phantom: core::marker::PhantomData<&'a ()>,
}

impl<'a> ZephyrServiceServer<'a> {
    // Future: add methods for getting server info
}

// Implement manually since ShimQueryable doesn't need 'a
impl<'a> From<ShimQueryable> for ZephyrServiceServer<'a> {
    fn from(queryable: ShimQueryable) -> Self {
        ZephyrServiceServer {
            queryable,
            _phantom: core::marker::PhantomData,
        }
    }
}

// =============================================================================
// Service callback and handling
// =============================================================================

/// Counter for received requests
static REQUEST_COUNT: AtomicU32 = AtomicU32::new(0);

/// Static context pointer for the ShimContext (needed for query_reply)
static mut SHIM_CTX: Option<*const ShimContext> = None;

/// Service key expression for reply (stored globally since callback is C-style)
static SERVICE_KEYEXPR: &[u8] = b"demo/add_two_ints\0";

/// Callback invoked when a service request is received
extern "C" fn on_service_request(
    _keyexpr: *const i8,
    _keyexpr_len: usize,
    payload: *const u8,
    payload_len: usize,
    _ctx: *mut c_void,
) {
    let count = REQUEST_COUNT.fetch_add(1, Ordering::Relaxed);

    // Safety: payload is valid for payload_len bytes, provided by C shim
    let payload_slice = if payload.is_null() || payload_len == 0 {
        &[]
    } else {
        unsafe { core::slice::from_raw_parts(payload, payload_len) }
    };

    // Deserialize the request
    let mut reader = CdrReader::new(payload_slice);
    let request = match AddTwoIntsRequest::deserialize(&mut reader) {
        Ok(req) => req,
        Err(_) => {
            info!(
                "[{}] Failed to deserialize request ({} bytes)",
                count, payload_len
            );
            return;
        }
    };

    // Process the request
    let sum = request.a + request.b;
    info!(
        "[{}] Received request: {} + {} = {}",
        count, request.a, request.b, sum
    );

    // Create response
    let response = AddTwoIntsResponse { sum };

    // Serialize response
    let mut response_buf = [0u8; 256];
    let mut writer = CdrWriter::new(&mut response_buf);
    if response.serialize(&mut writer).is_err() {
        info!("[{}] Failed to serialize response", count);
        return;
    }
    let response_len = writer.position();

    // Send reply using the stored context
    // Safety: SHIM_CTX is set before the callback is registered and remains valid
    unsafe {
        if let Some(ctx_ptr) = SHIM_CTX {
            let ctx = &*ctx_ptr;
            if let Err(e) = ctx.query_reply(SERVICE_KEYEXPR, &response_buf[..response_len], None) {
                info!("[{}] Failed to send reply: {}", count, e);
            } else {
                info!("[{}] Sent reply: sum={}", count, sum);
            }
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

    info!("nano-ros Zephyr Service Server Starting");
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

    // Store context pointer for callback (needed for query_reply)
    unsafe {
        SHIM_CTX = Some(&ctx as *const ShimContext);
    }

    // Create node (high-level wrapper)
    let node = ZephyrNode::new(&ctx, "add_two_ints_server", "/demo");

    // Create service server
    // Note: zenoh key expressions cannot start with '/', so we use 'demo/add_two_ints'
    let _service = match unsafe {
        node.create_service_raw(
            "demo/add_two_ints",
            on_service_request,
            core::ptr::null_mut(),
        )
    } {
        Ok(s) => s,
        Err(e) => {
            error!("Failed to create service server: {:?}", e);
            return;
        }
    };

    info!("Service server ready: demo/add_two_ints");
    info!("Waiting for service requests...");
    info!("(Run native-rs-service-client or a zenoh-based client)");

    // Main loop - callbacks run in background threads on Zephyr
    loop {
        // Sleep and report status periodically
        zephyr::time::sleep(zephyr::time::Duration::secs(10));
        let count = REQUEST_COUNT.load(Ordering::Relaxed);
        info!("Total requests processed: {}", count);
    }
}
