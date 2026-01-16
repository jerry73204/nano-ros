//! Error types for CDR serialization/deserialization

use core::fmt;

/// Serialization error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerError {
    /// Buffer is too small to hold the serialized data
    BufferTooSmall,
    /// String is too long to serialize
    StringTooLong,
    /// Sequence is too long to serialize
    SequenceTooLong,
}

impl fmt::Display for SerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SerError::BufferTooSmall => write!(f, "buffer too small"),
            SerError::StringTooLong => write!(f, "string too long"),
            SerError::SequenceTooLong => write!(f, "sequence too long"),
        }
    }
}

/// Deserialization error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeserError {
    /// Unexpected end of buffer
    UnexpectedEof,
    /// Invalid data encountered
    InvalidData,
    /// Invalid UTF-8 in string
    InvalidUtf8,
    /// Sequence length exceeds capacity
    CapacityExceeded,
    /// Invalid encapsulation header
    InvalidHeader,
}

impl fmt::Display for DeserError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DeserError::UnexpectedEof => write!(f, "unexpected end of buffer"),
            DeserError::InvalidData => write!(f, "invalid data"),
            DeserError::InvalidUtf8 => write!(f, "invalid UTF-8"),
            DeserError::CapacityExceeded => write!(f, "capacity exceeded"),
            DeserError::InvalidHeader => write!(f, "invalid encapsulation header"),
        }
    }
}
