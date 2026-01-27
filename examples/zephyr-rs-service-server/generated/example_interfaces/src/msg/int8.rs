// nano-ros message type - pure Rust, no_std compatible
// Package: example_interfaces
// Message: Int8

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// Int8 message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Int8 {
    pub data: i8,
}

impl Serialize for Int8 {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i8(self.data)?;
        Ok(())
    }
}

impl Deserialize for Int8 {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            data: reader.read_i8()?,
        })
    }
}

impl RosMessage for Int8 {
    const TYPE_NAME: &'static str = "example_interfaces::msg::dds_::Int8_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}