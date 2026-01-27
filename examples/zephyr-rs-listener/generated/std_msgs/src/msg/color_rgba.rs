// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: ColorRGBA

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// ColorRGBA message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ColorRGBA {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl Serialize for ColorRGBA {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_f32(self.r)?;
        writer.write_f32(self.g)?;
        writer.write_f32(self.b)?;
        writer.write_f32(self.a)?;
        Ok(())
    }
}

impl Deserialize for ColorRGBA {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            r: reader.read_f32()?,
            g: reader.read_f32()?,
            b: reader.read_f32()?,
            a: reader.read_f32()?,
        })
    }
}

impl RosMessage for ColorRGBA {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::ColorRGBA_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}