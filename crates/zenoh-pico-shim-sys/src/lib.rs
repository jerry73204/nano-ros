//! zenoh-pico-shim-sys: C shim library for zenoh-pico with FFI bindings
//!
//! This crate provides:
//! - The compiled C shim library (zenoh_shim.c)
//! - Platform-specific C code (system.c, network.c for smoltcp)
//! - FFI constants and types
//! - Platform Rust implementations (smoltcp_* functions)
//!
//! # Platform Backends
//!
//! Select one backend via feature flags:
//! - `posix` - Uses POSIX threads, for desktop testing
//! - `zephyr` - Uses Zephyr RTOS threads
//! - `smoltcp` - Uses polling with smoltcp network stack (bare-metal)

#![no_std]

#[cfg(any(feature = "std", test))]
extern crate std;

// Note: The smoltcp platform uses a custom bump allocator for C FFI (zenoh-pico),
// not Rust's global allocator. The `alloc` crate is NOT needed.

#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
use core::ffi::c_void;

// ============================================================================
// FFI Declarations
// ============================================================================

mod ffi;
pub use ffi::*;

// ============================================================================
// Platform-specific Modules
// ============================================================================

/// smoltcp platform layer for bare-metal systems
///
/// This module provides the Rust FFI functions that the C platform layer calls
/// to interact with smoltcp for network I/O and system services.
///
/// Note: Excluded from cbindgen to avoid duplicate function declarations.
/// The FFI declarations are in ffi.rs for header generation.
#[cfg(not(cbindgen))]
#[cfg(feature = "smoltcp")]
pub mod platform_smoltcp;

// ============================================================================
// Extern C Functions from the Shim
// ============================================================================

// These extern declarations import the C shim functions.
// The actual implementations are in c/shim/zenoh_shim.c
//
// Note: Excluded from cbindgen - these are Rust imports of C functions,
// not declarations for the header file.
#[cfg(all(
    any(feature = "posix", feature = "zephyr", feature = "smoltcp"),
    not(cbindgen)
))]
#[allow(improper_ctypes)]
extern "C" {
    // Session lifecycle
    pub fn zenoh_shim_init(locator: *const core::ffi::c_char) -> i32;
    pub fn zenoh_shim_open() -> i32;
    pub fn zenoh_shim_is_open() -> i32;
    pub fn zenoh_shim_close();

    // ZenohId
    pub fn zenoh_shim_get_zid(zid_out: *mut u8) -> i32;

    // Publishers
    pub fn zenoh_shim_declare_publisher(keyexpr: *const core::ffi::c_char) -> i32;
    pub fn zenoh_shim_publish(handle: i32, data: *const u8, len: usize) -> i32;
    pub fn zenoh_shim_publish_with_attachment(
        handle: i32,
        data: *const u8,
        len: usize,
        attachment: *const u8,
        attachment_len: usize,
    ) -> i32;
    pub fn zenoh_shim_undeclare_publisher(handle: i32) -> i32;

    // Subscribers
    pub fn zenoh_shim_declare_subscriber(
        keyexpr: *const core::ffi::c_char,
        callback: ShimCallback,
        ctx: *mut c_void,
    ) -> i32;
    pub fn zenoh_shim_undeclare_subscriber(handle: i32) -> i32;

    // Liveliness
    pub fn zenoh_shim_declare_liveliness(keyexpr: *const core::ffi::c_char) -> i32;
    pub fn zenoh_shim_undeclare_liveliness(handle: i32) -> i32;

    // Queryables (for services)
    pub fn zenoh_shim_declare_queryable(
        keyexpr: *const core::ffi::c_char,
        callback: ShimQueryCallback,
        ctx: *mut c_void,
    ) -> i32;
    pub fn zenoh_shim_undeclare_queryable(handle: i32) -> i32;
    pub fn zenoh_shim_query_reply(
        keyexpr: *const core::ffi::c_char,
        data: *const u8,
        len: usize,
        attachment: *const u8,
        attachment_len: usize,
    ) -> i32;

    // Service client (queries)
    pub fn zenoh_shim_get(
        keyexpr: *const core::ffi::c_char,
        payload: *const u8,
        payload_len: usize,
        reply_buf: *mut u8,
        reply_buf_size: usize,
        timeout_ms: u32,
    ) -> i32;

    // Polling
    pub fn zenoh_shim_poll(timeout_ms: u32) -> i32;
    pub fn zenoh_shim_spin_once(timeout_ms: u32) -> i32;
    pub fn zenoh_shim_uses_polling() -> bool;
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_constants() {
        assert_eq!(ZENOH_SHIM_OK, 0);
        assert_eq!(ZENOH_SHIM_ERR_GENERIC, -1);
        assert_eq!(ZENOH_SHIM_MAX_PUBLISHERS, 8);
        assert_eq!(ZENOH_SHIM_MAX_SUBSCRIBERS, 8);
    }
}
