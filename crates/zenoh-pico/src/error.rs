//! Error types for zenoh-pico

use core::fmt;

/// Result type for zenoh-pico operations
pub type Result<T> = core::result::Result<T, Error>;

/// Error type for zenoh-pico operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Failed to open a session
    SessionOpenFailed,
    /// Session is closed
    SessionClosed,
    /// Invalid configuration
    InvalidConfig,
    /// Invalid key expression
    InvalidKeyExpr,
    /// Failed to declare a publisher
    PublisherDeclarationFailed,
    /// Failed to declare a subscriber
    SubscriberDeclarationFailed,
    /// Failed to publish data
    PublishFailed,
    /// Failed to start background task
    TaskStartFailed,
    /// Buffer is too small
    BufferTooSmall,
    /// Generic error with zenoh-pico result code
    ZenohError(i8),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::SessionOpenFailed => write!(f, "Failed to open session"),
            Error::SessionClosed => write!(f, "Session is closed"),
            Error::InvalidConfig => write!(f, "Invalid configuration"),
            Error::InvalidKeyExpr => write!(f, "Invalid key expression"),
            Error::PublisherDeclarationFailed => write!(f, "Failed to declare publisher"),
            Error::SubscriberDeclarationFailed => write!(f, "Failed to declare subscriber"),
            Error::PublishFailed => write!(f, "Failed to publish data"),
            Error::TaskStartFailed => write!(f, "Failed to start background task"),
            Error::BufferTooSmall => write!(f, "Buffer is too small"),
            Error::ZenohError(code) => write!(f, "Zenoh error: {}", code),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}
