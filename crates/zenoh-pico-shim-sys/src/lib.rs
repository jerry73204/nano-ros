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

#[cfg(all(not(feature = "std"), feature = "smoltcp"))]
extern crate alloc;

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
#[cfg(feature = "smoltcp")]
pub mod platform_smoltcp;

// ============================================================================
// Extern C Functions from the Shim
// ============================================================================

// These extern declarations import the C shim functions.
// The actual implementations are in c/shim/zenoh_shim.c
#[cfg(any(feature = "posix", feature = "zephyr", feature = "smoltcp"))]
#[allow(improper_ctypes)]
extern "C" {
    pub fn zenoh_shim_init(locator: *const core::ffi::c_char) -> i32;
    pub fn zenoh_shim_open() -> i32;
    pub fn zenoh_shim_is_open() -> i32;
    pub fn zenoh_shim_close();
    pub fn zenoh_shim_declare_publisher(keyexpr: *const core::ffi::c_char) -> i32;
    pub fn zenoh_shim_publish(handle: i32, data: *const u8, len: usize) -> i32;
    pub fn zenoh_shim_undeclare_publisher(handle: i32) -> i32;
    pub fn zenoh_shim_declare_subscriber(
        keyexpr: *const core::ffi::c_char,
        callback: ShimCallback,
        ctx: *mut c_void,
    ) -> i32;
    pub fn zenoh_shim_undeclare_subscriber(handle: i32) -> i32;
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
