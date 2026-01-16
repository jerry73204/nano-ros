//! Serialization traits

use crate::cdr::{CdrReader, CdrWriter};
use crate::error::{DeserError, SerError};

/// Trait for types that can be serialized to CDR format
pub trait Serialize {
    /// Serialize this value to the CDR writer
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError>;
}

/// Trait for types that can be deserialized from CDR format
pub trait Deserialize: Sized {
    /// Deserialize a value from the CDR reader
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError>;
}
