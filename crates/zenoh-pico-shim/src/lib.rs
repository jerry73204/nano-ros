//! zenoh-pico-shim: High-level C shim for zenoh-pico with platform backends
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

// Always use no_std - import std when the feature is enabled
#![no_std]

// std is needed for tests or when std feature is enabled
#[cfg(any(feature = "std", test))]
extern crate std;

// Only need alloc when we have a backend but not std
#[cfg(all(
    not(feature = "std"),
    any(feature = "posix", feature = "zephyr", feature = "smoltcp")
))]
extern crate alloc;

use core::ffi::c_void;
use core::marker::PhantomData;

// ============================================================================
// FFI Declarations
// ============================================================================

mod ffi;
pub use ffi::*;

// Extern imports from C shim
// Note: These are kept in lib.rs (not ffi.rs) for separation.
// cbindgen parses the entire crate and generates duplicate declarations from this block.
// The generated header is post-processed to remove the duplicates (see ffi.rs for commands).
#[allow(improper_ctypes)]
extern "C" {
    pub(crate) fn zenoh_shim_init(locator: *const core::ffi::c_char) -> i32;
    pub(crate) fn zenoh_shim_open() -> i32;
    pub(crate) fn zenoh_shim_is_open() -> i32;
    pub(crate) fn zenoh_shim_close();
    pub(crate) fn zenoh_shim_declare_publisher(keyexpr: *const core::ffi::c_char) -> i32;
    pub(crate) fn zenoh_shim_publish(handle: i32, data: *const u8, len: usize) -> i32;
    pub(crate) fn zenoh_shim_undeclare_publisher(handle: i32) -> i32;
    pub(crate) fn zenoh_shim_declare_subscriber(
        keyexpr: *const core::ffi::c_char,
        callback: ShimCallback,
        ctx: *mut c_void,
    ) -> i32;
    pub(crate) fn zenoh_shim_undeclare_subscriber(handle: i32) -> i32;
    pub(crate) fn zenoh_shim_poll(timeout_ms: u32) -> i32;
    pub(crate) fn zenoh_shim_spin_once(timeout_ms: u32) -> i32;
    pub(crate) fn zenoh_shim_uses_polling() -> bool;
}

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
pub struct ShimContext {
    _private: PhantomData<*const ()>,
}

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
}

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
pub struct ShimPublisher<'a> {
    handle: i32,
    _ctx: PhantomData<&'a ShimContext>,
}

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

    /// Get the publisher handle
    pub fn handle(&self) -> i32 {
        self.handle
    }
}

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
pub struct ShimSubscriber<'a> {
    handle: i32,
    _ctx: PhantomData<&'a ShimContext>,
}

impl<'a> ShimSubscriber<'a> {
    /// Get the subscriber handle
    pub fn handle(&self) -> i32 {
        self.handle
    }
}

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
