//! rstest fixtures for integration tests
//!
//! Provides `#[rstest::fixture]` functions for:
//! - `zenohd`, `zenohd_unique` - Managed zenohd router
//! - `qemu_binary`, `talker_binary`, `listener_binary` - Binary build fixtures
//!
//! Also re-exports utilities from sibling modules for convenience.

mod binaries;
mod zenohd_router;

pub use binaries::*;
pub use zenohd_router::*;

// Re-export utilities for backwards compatibility
pub use crate::process::*;
pub use crate::qemu::*;
pub use crate::ros2::*;
