//! Test fixtures for nano-ros integration tests
//!
//! Provides RAII-based fixtures for:
//! - `ZenohRouter` - Managed zenohd process
//! - `QemuProcess` - QEMU ARM emulator
//! - Binary build helpers

mod binaries;
mod qemu;
mod zenohd_fixture;

pub use binaries::*;
pub use qemu::*;
pub use zenohd_fixture::*;
