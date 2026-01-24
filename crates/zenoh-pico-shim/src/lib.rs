//! zenoh-pico-shim: High-level Rust API for zenoh-pico
//!
//! This crate provides a safe Rust wrapper around the zenoh-pico C shim,
//! enabling embedded applications to use zenoh for communication.
//!
//! # Platform Backends
//!
//! Select one backend via feature flags:
//! - `posix` - Uses POSIX threads, for desktop testing
//! - `zephyr` - Uses Zephyr RTOS threads
//! - `smoltcp` - Uses polling with smoltcp network stack (bare-metal)
//!
//! # Example
//!
//! ```ignore
//! use zenoh_pico_shim::{ShimContext, ShimPublisher};
//!
//! let ctx = ShimContext::new(b"tcp/127.0.0.1:7447\0")?;
//! let publisher = ctx.declare_publisher(b"demo/topic\0")?;
//! publisher.publish(b"Hello, World!")?;
//! ```

#![no_std]

#[cfg(any(feature = "std", test))]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
use core::ffi::c_void;
#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
use core::marker::PhantomData;

// Re-export FFI types and constants from sys crate
pub use zenoh_pico_shim_sys::{
    PollCallback, ShimCallback, ShimQueryCallback, ZENOH_SHIM_ERR_CONFIG, ZENOH_SHIM_ERR_FULL,
    ZENOH_SHIM_ERR_GENERIC, ZENOH_SHIM_ERR_INVALID, ZENOH_SHIM_ERR_KEYEXPR, ZENOH_SHIM_ERR_PUBLISH,
    ZENOH_SHIM_ERR_SESSION, ZENOH_SHIM_ERR_TASK, ZENOH_SHIM_MAX_LIVELINESS,
    ZENOH_SHIM_MAX_PUBLISHERS, ZENOH_SHIM_MAX_QUERYABLES, ZENOH_SHIM_MAX_SUBSCRIBERS,
    ZENOH_SHIM_OK, ZENOH_SHIM_RMW_GID_SIZE, ZENOH_SHIM_ZID_SIZE,
};

// Re-export platform module for smoltcp
#[cfg(feature = "smoltcp")]
pub use zenoh_pico_shim_sys::platform_smoltcp;

// Import FFI functions from sys crate
#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
use zenoh_pico_shim_sys::{
    zenoh_shim_close, zenoh_shim_declare_liveliness, zenoh_shim_declare_publisher,
    zenoh_shim_declare_queryable, zenoh_shim_declare_subscriber, zenoh_shim_get_zid,
    zenoh_shim_init, zenoh_shim_is_open, zenoh_shim_open, zenoh_shim_poll, zenoh_shim_publish,
    zenoh_shim_publish_with_attachment, zenoh_shim_query_reply, zenoh_shim_spin_once,
    zenoh_shim_undeclare_liveliness, zenoh_shim_undeclare_publisher,
    zenoh_shim_undeclare_queryable, zenoh_shim_undeclare_subscriber, zenoh_shim_uses_polling,
};

// ============================================================================
// Error Types
// ============================================================================

/// Error type for shim operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShimError {
    /// Generic error
    Generic,
    /// Configuration error
    Config,
    /// Session error
    Session,
    /// Task creation error
    Task,
    /// Invalid key expression
    KeyExpr,
    /// Resource limit reached
    Full,
    /// Invalid handle
    Invalid,
    /// Publish error
    Publish,
    /// Session not open
    NotOpen,
}

#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp", test))]
impl ShimError {
    fn from_code(code: i32) -> Self {
        match code {
            -1 => ShimError::Generic,
            -2 => ShimError::Config,
            -3 => ShimError::Session,
            -4 => ShimError::Task,
            -5 => ShimError::KeyExpr,
            -6 => ShimError::Full,
            -7 => ShimError::Invalid,
            -8 => ShimError::Publish,
            _ => ShimError::Generic,
        }
    }
}

impl core::fmt::Display for ShimError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ShimError::Generic => write!(f, "generic error"),
            ShimError::Config => write!(f, "configuration error"),
            ShimError::Session => write!(f, "session error"),
            ShimError::Task => write!(f, "task creation error"),
            ShimError::KeyExpr => write!(f, "invalid key expression"),
            ShimError::Full => write!(f, "resource limit reached"),
            ShimError::Invalid => write!(f, "invalid handle"),
            ShimError::Publish => write!(f, "publish error"),
            ShimError::NotOpen => write!(f, "session not open"),
        }
    }
}

/// Result type for shim operations
pub type Result<T> = core::result::Result<T, ShimError>;

// ============================================================================
// ShimZenohId
// ============================================================================

/// A 16-byte Zenoh session ID
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ShimZenohId {
    /// The raw 16-byte ID
    pub id: [u8; 16],
}

impl ShimZenohId {
    /// Create a new ZenohId from raw bytes
    pub fn from_bytes(bytes: [u8; 16]) -> Self {
        Self { id: bytes }
    }

    /// Format the ID as a hexadecimal string (for liveliness key expressions)
    ///
    /// Uses LSB-first (little-endian) order to match zenoh's z_id_to_string format.
    #[cfg(any(feature = "std", feature = "alloc"))]
    pub fn to_hex_string(&self) -> alloc::string::String {
        use alloc::format;
        // zenoh uses LSB-first (reversed) byte order for ID strings
        format!(
            "{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
            self.id[15], self.id[14], self.id[13], self.id[12],
            self.id[11], self.id[10], self.id[9], self.id[8],
            self.id[7], self.id[6], self.id[5], self.id[4],
            self.id[3], self.id[2], self.id[1], self.id[0]
        )
    }

    /// Format the ID into a fixed-size buffer (for no_std)
    ///
    /// Returns the number of bytes written (always 32).
    pub fn to_hex_bytes(&self, buf: &mut [u8; 32]) {
        const HEX_CHARS: &[u8; 16] = b"0123456789abcdef";
        // LSB-first order
        for i in 0..16 {
            let byte = self.id[15 - i];
            buf[i * 2] = HEX_CHARS[(byte >> 4) as usize];
            buf[i * 2 + 1] = HEX_CHARS[(byte & 0xf) as usize];
        }
    }
}

// ============================================================================
// ShimLivelinessToken
// ============================================================================

/// A liveliness token for ROS 2 discovery
///
/// When a liveliness token is declared, subscribers on intersecting key expressions
/// will receive a PUT sample when connectivity is achieved, and a DELETE sample
/// if it's lost.
///
/// Liveliness tokens are automatically undeclared when dropped.
#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
pub struct ShimLivelinessToken<'a> {
    handle: i32,
    _ctx: PhantomData<&'a ShimContext>,
}

#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
impl<'a> ShimLivelinessToken<'a> {
    /// Get the liveliness handle
    pub fn handle(&self) -> i32 {
        self.handle
    }
}

#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
impl<'a> Drop for ShimLivelinessToken<'a> {
    fn drop(&mut self) {
        unsafe {
            zenoh_shim_undeclare_liveliness(self.handle);
        }
    }
}

// ============================================================================
// ShimQueryable
// ============================================================================

/// A queryable for receiving service requests
///
/// Queryables receive queries and can send replies. This is used to implement
/// ROS 2 service servers.
#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
pub struct ShimQueryable<'a> {
    handle: i32,
    _ctx: PhantomData<&'a ShimContext>,
}

#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
impl<'a> ShimQueryable<'a> {
    /// Get the queryable handle
    pub fn handle(&self) -> i32 {
        self.handle
    }
}

#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
impl<'a> Drop for ShimQueryable<'a> {
    fn drop(&mut self) {
        unsafe {
            zenoh_shim_undeclare_queryable(self.handle);
        }
    }
}

// ============================================================================
// ShimContext
// ============================================================================

/// Context for managing zenoh-pico shim session
///
/// The context manages the zenoh session lifecycle and provides methods
/// for creating publishers and subscribers.
///
/// # Note
///
/// Only one `ShimContext` can exist at a time due to the global state
/// in the C shim.
#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
pub struct ShimContext {
    _private: PhantomData<*const ()>,
}

#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
impl ShimContext {
    /// Create a new shim context with the given locator
    ///
    /// The locator should be a null-terminated string like `b"tcp/127.0.0.1:7447\0"`.
    ///
    /// # Errors
    ///
    /// Returns an error if initialization or session opening fails.
    pub fn new(locator: &[u8]) -> Result<Self> {
        // Safety: locator is a valid byte slice, cast to c_char for C string
        let ret = unsafe { zenoh_shim_init(locator.as_ptr().cast()) };
        if ret < 0 {
            return Err(ShimError::from_code(ret));
        }

        let ret = unsafe { zenoh_shim_open() };
        if ret < 0 {
            return Err(ShimError::from_code(ret));
        }

        Ok(ShimContext {
            _private: PhantomData,
        })
    }

    /// Check if the session is open
    pub fn is_open(&self) -> bool {
        unsafe { zenoh_shim_is_open() != 0 }
    }

    /// Check if this backend uses polling
    ///
    /// If true, you must call `poll()` or `spin_once()` regularly to
    /// process network data and dispatch callbacks.
    pub fn uses_polling(&self) -> bool {
        unsafe { zenoh_shim_uses_polling() }
    }

    /// Declare a publisher for the given key expression
    ///
    /// The key expression should be a null-terminated string like `b"demo/topic\0"`.
    ///
    /// # Errors
    ///
    /// Returns an error if the session is not open, the key expression is invalid,
    /// or the maximum number of publishers has been reached.
    pub fn declare_publisher(&self, keyexpr: &[u8]) -> Result<ShimPublisher<'_>> {
        let handle = unsafe { zenoh_shim_declare_publisher(keyexpr.as_ptr().cast()) };
        if handle < 0 {
            return Err(ShimError::from_code(handle));
        }

        Ok(ShimPublisher {
            handle,
            _ctx: PhantomData,
        })
    }

    /// Declare a subscriber for the given key expression
    ///
    /// The key expression should be a null-terminated string like `b"demo/topic\0"`.
    /// The callback will be invoked when samples arrive.
    ///
    /// # Safety
    ///
    /// The callback and context must remain valid for the lifetime of the subscriber.
    /// The context pointer must be valid for the callback to dereference.
    ///
    /// # Errors
    ///
    /// Returns an error if the session is not open, the key expression is invalid,
    /// or the maximum number of subscribers has been reached.
    pub unsafe fn declare_subscriber_raw<'a>(
        &'a self,
        keyexpr: &[u8],
        callback: ShimCallback,
        ctx: *mut c_void,
    ) -> Result<ShimSubscriber<'a>> {
        let handle = zenoh_shim_declare_subscriber(keyexpr.as_ptr().cast(), callback, ctx);
        if handle < 0 {
            return Err(ShimError::from_code(handle));
        }

        Ok(ShimSubscriber {
            handle,
            _ctx: PhantomData,
        })
    }

    /// Poll for incoming data and process callbacks
    ///
    /// For threaded backends (POSIX, Zephyr), this is a no-op as background
    /// tasks handle polling automatically.
    ///
    /// For polling backends (smoltcp), this must be called regularly.
    ///
    /// # Arguments
    ///
    /// * `timeout_ms` - Maximum time to wait for data (0 = non-blocking)
    ///
    /// # Returns
    ///
    /// Number of events processed, or error
    pub fn poll(&self, timeout_ms: u32) -> Result<i32> {
        let ret = unsafe { zenoh_shim_poll(timeout_ms) };
        if ret < 0 {
            return Err(ShimError::from_code(ret));
        }
        Ok(ret)
    }

    /// Combined poll and keepalive operation
    ///
    /// This is equivalent to calling `poll()` and performing any necessary
    /// keepalive operations.
    ///
    /// # Arguments
    ///
    /// * `timeout_ms` - Maximum time to wait (0 = non-blocking)
    ///
    /// # Returns
    ///
    /// Number of events processed, or error
    pub fn spin_once(&self, timeout_ms: u32) -> Result<i32> {
        let ret = unsafe { zenoh_shim_spin_once(timeout_ms) };
        if ret < 0 {
            return Err(ShimError::from_code(ret));
        }
        Ok(ret)
    }

    /// Get the session's Zenoh ID
    ///
    /// The Zenoh ID uniquely identifies this session in the Zenoh network.
    /// It is used in liveliness token key expressions for ROS 2 discovery.
    pub fn zid(&self) -> Result<ShimZenohId> {
        let mut id = [0u8; 16];
        let ret = unsafe { zenoh_shim_get_zid(id.as_mut_ptr()) };
        if ret < 0 {
            return Err(ShimError::from_code(ret));
        }
        Ok(ShimZenohId::from_bytes(id))
    }

    /// Declare a liveliness token for ROS 2 discovery
    ///
    /// The key expression should be a null-terminated string.
    ///
    /// # Errors
    ///
    /// Returns an error if the session is not open, the key expression is invalid,
    /// or the maximum number of liveliness tokens has been reached.
    pub fn declare_liveliness(&self, keyexpr: &[u8]) -> Result<ShimLivelinessToken<'_>> {
        let handle = unsafe { zenoh_shim_declare_liveliness(keyexpr.as_ptr().cast()) };
        if handle < 0 {
            return Err(ShimError::from_code(handle));
        }

        Ok(ShimLivelinessToken {
            handle,
            _ctx: PhantomData,
        })
    }

    /// Declare a queryable for receiving service requests
    ///
    /// The key expression should be a null-terminated string.
    /// The callback will be invoked when queries arrive.
    ///
    /// # Safety
    ///
    /// The callback and context must remain valid for the lifetime of the queryable.
    /// The context pointer must be valid for the callback to dereference.
    ///
    /// # Errors
    ///
    /// Returns an error if the session is not open, the key expression is invalid,
    /// or the maximum number of queryables has been reached.
    pub unsafe fn declare_queryable_raw<'a>(
        &'a self,
        keyexpr: &[u8],
        callback: ShimQueryCallback,
        ctx: *mut c_void,
    ) -> Result<ShimQueryable<'a>> {
        let handle = zenoh_shim_declare_queryable(keyexpr.as_ptr().cast(), callback, ctx);
        if handle < 0 {
            return Err(ShimError::from_code(handle));
        }

        Ok(ShimQueryable {
            handle,
            _ctx: PhantomData,
        })
    }

    /// Reply to a query (must be called within query callback)
    ///
    /// This sends a reply to the current query being processed.
    /// Must only be called from within a queryable callback.
    ///
    /// # Parameters
    ///
    /// * `keyexpr` - Reply key expression (null-terminated)
    /// * `data` - Reply payload
    /// * `attachment` - Optional attachment data
    ///
    /// # Errors
    ///
    /// Returns an error if not called from within a query callback,
    /// or if the reply operation fails.
    pub fn query_reply(
        &self,
        keyexpr: &[u8],
        data: &[u8],
        attachment: Option<&[u8]>,
    ) -> Result<()> {
        let (att_ptr, att_len) = match attachment {
            Some(att) => (att.as_ptr(), att.len()),
            None => (core::ptr::null(), 0),
        };

        let ret = unsafe {
            zenoh_shim_query_reply(
                keyexpr.as_ptr().cast(),
                data.as_ptr(),
                data.len(),
                att_ptr,
                att_len,
            )
        };
        if ret < 0 {
            return Err(ShimError::from_code(ret));
        }
        Ok(())
    }
}

#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
impl Drop for ShimContext {
    fn drop(&mut self) {
        unsafe {
            zenoh_shim_close();
        }
    }
}

// ============================================================================
// ShimPublisher
// ============================================================================

/// Publisher handle for sending data
///
/// Created via `ShimContext::declare_publisher()`.
#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
pub struct ShimPublisher<'a> {
    handle: i32,
    _ctx: PhantomData<&'a ShimContext>,
}

#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
impl<'a> ShimPublisher<'a> {
    /// Publish data
    ///
    /// # Errors
    ///
    /// Returns an error if the publish operation fails.
    pub fn publish(&self, data: &[u8]) -> Result<()> {
        let ret = unsafe { zenoh_shim_publish(self.handle, data.as_ptr(), data.len()) };
        if ret < 0 {
            return Err(ShimError::from_code(ret));
        }
        Ok(())
    }

    /// Publish data with an attachment
    ///
    /// This is used for RMW compatibility, where an attachment contains
    /// metadata like sequence number, timestamp, and GID.
    ///
    /// # Parameters
    ///
    /// * `data` - The message payload
    /// * `attachment` - Optional attachment data (for RMW compatibility)
    ///
    /// # Errors
    ///
    /// Returns an error if the publish operation fails.
    pub fn publish_with_attachment(&self, data: &[u8], attachment: Option<&[u8]>) -> Result<()> {
        let (att_ptr, att_len) = match attachment {
            Some(att) => (att.as_ptr(), att.len()),
            None => (core::ptr::null(), 0),
        };

        let ret = unsafe {
            zenoh_shim_publish_with_attachment(
                self.handle,
                data.as_ptr(),
                data.len(),
                att_ptr,
                att_len,
            )
        };
        if ret < 0 {
            return Err(ShimError::from_code(ret));
        }
        Ok(())
    }

    /// Get the publisher handle
    pub fn handle(&self) -> i32 {
        self.handle
    }
}

#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
impl<'a> Drop for ShimPublisher<'a> {
    fn drop(&mut self) {
        unsafe {
            zenoh_shim_undeclare_publisher(self.handle);
        }
    }
}

// ============================================================================
// ShimSubscriber
// ============================================================================

/// Subscriber handle for receiving data
///
/// Created via `ShimContext::declare_subscriber_raw()`.
#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
pub struct ShimSubscriber<'a> {
    handle: i32,
    _ctx: PhantomData<&'a ShimContext>,
}

#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
impl<'a> ShimSubscriber<'a> {
    /// Get the subscriber handle
    pub fn handle(&self) -> i32 {
        self.handle
    }
}

#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
impl<'a> Drop for ShimSubscriber<'a> {
    fn drop(&mut self) {
        unsafe {
            zenoh_shim_undeclare_subscriber(self.handle);
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::format;

    #[test]
    fn test_error_from_code() {
        assert_eq!(ShimError::from_code(-1), ShimError::Generic);
        assert_eq!(ShimError::from_code(-2), ShimError::Config);
        assert_eq!(ShimError::from_code(-3), ShimError::Session);
        assert_eq!(ShimError::from_code(-4), ShimError::Task);
        assert_eq!(ShimError::from_code(-5), ShimError::KeyExpr);
        assert_eq!(ShimError::from_code(-6), ShimError::Full);
        assert_eq!(ShimError::from_code(-7), ShimError::Invalid);
        assert_eq!(ShimError::from_code(-8), ShimError::Publish);
        assert_eq!(ShimError::from_code(-99), ShimError::Generic);
    }

    #[test]
    fn test_error_display() {
        assert_eq!(format!("{}", ShimError::Generic), "generic error");
        assert_eq!(format!("{}", ShimError::Session), "session error");
    }
}
