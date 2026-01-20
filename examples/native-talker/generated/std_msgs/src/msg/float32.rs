// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: Float32

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// Float32 message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Float32 {
    pub data: f32,
}

impl Serialize for Float32 {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_f32(self.data)?;
        Ok(())
    }
}

impl Deserialize for Float32 {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            data: reader.read_f32()?,
        })
    }
}

impl RosMessage for Float32 {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::Float32_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}