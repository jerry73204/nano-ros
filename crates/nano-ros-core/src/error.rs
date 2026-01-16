//! Unified error types for nano-ros

use core::fmt;
use nano_ros_serdes::{DeserError, SerError};

/// Unified error type for nano-ros operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Serialization error
    Serialization(SerError),
    /// Deserialization error
    Deserialization(DeserError),
    /// Transport error
    Transport,
    /// Timeout
    Timeout,
    /// Not ready
    NotReady,
    /// Invalid state
    InvalidState,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Serialization(e) => write!(f, "serialization error: {}", e),
            Error::Deserialization(e) => write!(f, "deserialization error: {}", e),
            Error::Transport => write!(f, "transport error"),
            Error::Timeout => write!(f, "timeout"),
            Error::NotReady => write!(f, "not ready"),
            Error::InvalidState => write!(f, "invalid state"),
        }
    }
}

impl From<SerError> for Error {
    fn from(e: SerError) -> Self {
        Error::Serialization(e)
    }
}

impl From<DeserError> for Error {
    fn from(e: DeserError) -> Self {
        Error::Deserialization(e)
    }
}
