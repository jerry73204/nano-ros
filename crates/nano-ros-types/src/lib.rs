//! Standard ROS message types for nano-ros
//!
//! This crate provides pre-defined message types matching ROS 2 standard interfaces:
//! - `builtin_interfaces` - Time, Duration
//! - `std_msgs` - Header, String, primitives
//! - `geometry_msgs` - Point, Vector3, Quaternion, Pose, Twist

#![no_std]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

// Re-export dependencies for macro-generated code
pub use nano_ros_core;
pub use nano_ros_serdes;

pub mod builtin_interfaces;
pub mod geometry_msgs;
pub mod std_msgs;

// Re-export commonly used types
pub use builtin_interfaces::{Duration, Time};
pub use std_msgs::Header;
