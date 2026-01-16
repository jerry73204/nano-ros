#![no_std]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod server;
pub mod types;

pub use server::ParameterServer;
pub use types::{Parameter, ParameterType, ParameterValue};
