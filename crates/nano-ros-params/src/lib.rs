//! Parameter server for nano-ros
//!
//! This crate provides a ROS 2 compatible parameter server for embedded systems.
//! Parameters are stored in static memory with compile-time configurable capacity.
//!
//! # Example
//!
//! ```
//! use nano_ros_params::{ParameterServer, ParameterValue, ParameterDescriptor, ParameterType};
//!
//! let mut server = ParameterServer::new();
//!
//! // Declare a simple parameter
//! server.declare("max_speed", ParameterValue::Double(1.0));
//!
//! // Declare a parameter with constraints
//! let desc = ParameterDescriptor::new("velocity", ParameterType::Double)
//!     .unwrap()
//!     .with_description("Maximum velocity in m/s")
//!     .with_float_range(0.0, 10.0, 0.1);
//! server.declare_with_descriptor("velocity", ParameterValue::Double(5.0), Some(desc));
//!
//! // Get and set parameters
//! assert_eq!(server.get_double("max_speed"), Some(1.0));
//! server.set_double("max_speed", 2.0);
//! ```
//!
//! # Features
//!
//! - `std` - Enable standard library support
//! - `alloc` - Enable heap allocation

#![no_std]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod server;
pub mod types;

// Re-export main types
pub use server::{ParameterBuilder, ParameterServer, MAX_PARAMETERS};
pub use types::{
    FloatingPointRange, IntegerRange, Parameter, ParameterDescriptor, ParameterRange,
    ParameterType, ParameterValue, SetParameterResult, MAX_ARRAY_LEN, MAX_BYTE_ARRAY_LEN,
    MAX_PARAM_NAME_LEN, MAX_STRING_VALUE_LEN,
};
