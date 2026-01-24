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
//!
//! # Executor Support
//!
//! - `rtic` - RTIC executor support (uses critical sections, no background threads)
//! - `embassy` - Embassy executor support
//! - `polling` - Simple polling mode without background threads
//!
//! # Synchronization Backends
//!
//! - `sync-spin` - Use spin::Mutex (default, works everywhere)
//! - `sync-critical-section` - Use critical sections (RTIC/Embassy compatible)
//!
//! For RTIC applications, enable `rtic` and `sync-critical-section` features.

#![no_std]

// Compile-time check: zenoh requires alloc
#[cfg(all(feature = "zenoh", not(feature = "alloc")))]
compile_error!("The `zenoh` feature requires `alloc`. Enable the `alloc` feature or use `zenoh` feature which implies `alloc`.");

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod sync;
pub mod traits;

#[cfg(feature = "zenoh")]
pub mod zenoh;

#[cfg(feature = "shim")]
pub mod shim;

// Re-export main types
pub use traits::{
    ActionInfo, Publisher, QosDurabilityPolicy, QosHistoryPolicy, QosReliabilityPolicy,
    QosSettings, ServiceClientTrait, ServiceInfo, ServiceRequest, ServiceServerTrait, Session,
    SessionMode, Subscriber, TopicInfo, Transport, TransportConfig, TransportError,
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

// Re-export shim types when feature is enabled
#[cfg(feature = "shim")]
pub use shim::{
    RmwAttachment as ShimRmwAttachment, Ros2Liveliness as ShimRos2Liveliness, ShimPublisher,
    ShimServiceClient, ShimServiceServer, ShimSession, ShimSubscriber, ShimTransport,
    ZenohId as ShimZenohId, RMW_GID_SIZE as SHIM_RMW_GID_SIZE,
};

// Re-export zenoh-pico-shim types for liveliness support
#[cfg(feature = "shim")]
pub use zenoh_pico_shim::ShimLivelinessToken;
