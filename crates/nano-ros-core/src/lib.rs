//! Core types, traits, and abstractions for nano-ros
//!
//! This crate provides the foundational types and traits for nano-ros:
//! - `RosMessage` trait for message types
//! - `RosService` trait for service types
//! - Time and Duration types
//! - Error types

#![no_std]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod error;
pub mod time;
pub mod types;

pub use error::Error;
pub use time::{Duration, Time};
pub use types::{RosMessage, RosService};

// Re-export serdes types for convenience
pub use nano_ros_serdes::{CdrReader, CdrWriter, DeserError, Deserialize, SerError, Serialize};
