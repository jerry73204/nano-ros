// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: Int32

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// Int32 message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Int32 {
    pub data: i32,
}

impl Serialize for Int32 {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i32(self.data)?;
        Ok(())
    }
}

impl Deserialize for Int32 {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            data: reader.read_i32()?,
        })
    }
}

impl RosMessage for Int32 {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::Int32_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}