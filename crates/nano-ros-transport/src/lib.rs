#![no_std]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod traits;

#[cfg(feature = "zenoh")]
pub mod zenoh;

pub use traits::Transport;
