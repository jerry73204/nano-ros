//! Transport abstraction layer for nano-ros
//!
//! This crate provides the transport layer abstraction for nano-ros,
//! allowing different backends (zenoh-pico, etc.) to be used interchangeably.
//!
//! # Features
//!
//! - `std` - Enable standard library support
//! - `alloc` - Enable heap allocation
//! - `zenoh` - Enable zenoh-pico backend

#![no_std]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod traits;

#[cfg(feature = "zenoh")]
pub mod zenoh;

// Re-export main types
pub use traits::{
    Publisher, QosSettings, ServiceClientTrait, ServiceInfo, ServiceRequest, ServiceServerTrait,
    Session, SessionMode, Subscriber, TopicInfo, Transport, TransportConfig, TransportError,
};

// Re-export zenoh types when feature is enabled
#[cfg(feature = "zenoh")]
pub use zenoh::{
    RmwAttachment, Ros2Liveliness, ZenohPublisher, ZenohServiceClient, ZenohServiceServer,
    ZenohSession, ZenohSubscriber, ZenohTransport,
};

// Re-export zenoh-pico types for liveliness support
#[cfg(feature = "zenoh")]
pub use zenoh_pico::{LivelinessToken, ZenohId};
