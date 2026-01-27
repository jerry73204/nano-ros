// nano-ros message type - pure Rust, no_std compatible
// Package: example_interfaces
// Message: Float64

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// Float64 message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Float64 {
    pub data: f64,
}

impl Serialize for Float64 {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_f64(self.data)?;
        Ok(())
    }
}

impl Deserialize for Float64 {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            data: reader.read_f64()?,
        })
    }
}

impl RosMessage for Float64 {
    const TYPE_NAME: &'static str = "example_interfaces::msg::dds_::Float64_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}