//! CDR serialization/deserialization for nano-ros
//!
//! Implements OMG Common Data Representation (CDR) encoding compatible with ROS 2.
//!
//! # Features
//!
//! - `std` - Enable standard library support
//! - `alloc` - Enable heap allocation (String, Vec)

#![no_std]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod cdr;
pub mod error;
pub mod primitives;
pub mod traits;

#[cfg(test)]
mod compat_tests;

pub use cdr::{CdrReader, CdrWriter};
pub use error::{DeserError, SerError};
pub use traits::{Deserialize, Serialize};

/// CDR encapsulation header for little-endian encoding
pub const CDR_LE_HEADER: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

/// CDR encapsulation header for big-endian encoding
pub const CDR_BE_HEADER: [u8; 4] = [0x00, 0x00, 0x00, 0x00];
