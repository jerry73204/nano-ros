// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: UInt8

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// UInt8 message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct UInt8 {
    pub data: u8,
}

impl Serialize for UInt8 {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_u8(self.data)?;
        Ok(())
    }
}

impl Deserialize for UInt8 {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            data: reader.read_u8()?,
        })
    }
}

impl RosMessage for UInt8 {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::UInt8_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}