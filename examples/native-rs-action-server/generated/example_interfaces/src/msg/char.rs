// nano-ros message type - pure Rust, no_std compatible
// Package: example_interfaces
// Message: Char

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// Char message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Char {
    pub data: u8,
}

impl Serialize for Char {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_u8(self.data)?;
        Ok(())
    }
}

impl Deserialize for Char {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            data: reader.read_u8()?,
        })
    }
}

impl RosMessage for Char {
    const TYPE_NAME: &'static str = "example_interfaces::msg::dds_::Char_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}