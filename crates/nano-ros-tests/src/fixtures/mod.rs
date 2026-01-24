//! Test fixtures for nano-ros integration tests
//!
//! Provides RAII-based fixtures for:
//! - `ZenohRouter` - Managed zenohd process
//! - `QemuProcess` - QEMU ARM emulator
//! - `Ros2Process` - ROS 2 command wrapper
//! - Binary build helpers

mod binaries;
mod qemu;
mod ros2;
mod zenohd_fixture;

pub use binaries::*;
pub use qemu::*;
pub use ros2::*;
pub use zenohd_fixture::*;
