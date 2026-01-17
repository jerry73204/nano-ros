//! Safe Rust bindings for zenoh-pico
//!
//! This crate provides safe, idiomatic Rust wrappers around the zenoh-pico C library.
//!
//! # Example
//!
//! ```no_run
//! use zenoh_pico::{Config, Session, KeyExpr};
//!
//! // Create a session
//! let config = Config::client("tcp/127.0.0.1:7447").unwrap();
//! let session = Session::open(config).expect("Failed to open session");
//!
//! // Create a publisher
//! let keyexpr = KeyExpr::new("demo/example").unwrap();
//! let publisher = session.declare_publisher(&keyexpr).unwrap();
//!
//! // Publish data
//! publisher.put(b"Hello, World!").unwrap();
//! ```

#![no_std]

#[cfg(feature = "std")]
extern crate std;

extern crate alloc;

mod config;
mod error;
mod keyexpr;
mod publisher;
mod session;
mod subscriber;

pub use config::Config;
pub use error::{Error, Result};
pub use keyexpr::KeyExpr;
pub use publisher::Publisher;
pub use session::Session;
pub use subscriber::{Sample, Subscriber};

/// Re-export raw FFI types for advanced usage
pub mod ffi {
    pub use zenoh_pico_sys::*;
}
