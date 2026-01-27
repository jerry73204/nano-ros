// nano-ros message type - pure Rust, no_std compatible
// Package: example_interfaces
// Message: MultiArrayDimension

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// MultiArrayDimension message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MultiArrayDimension {
    pub label: heapless::String<256>,
    pub size: u32,
    pub stride: u32,
}

impl Serialize for MultiArrayDimension {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_string(self.label.as_str())?;
        writer.write_u32(self.size)?;
        writer.write_u32(self.stride)?;
        Ok(())
    }
}

impl Deserialize for MultiArrayDimension {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            label: {
                let s = reader.read_string()?;
                heapless::String::try_from(s).map_err(|_| DeserError::CapacityExceeded)?
            },
            size: reader.read_u32()?,
            stride: reader.read_u32()?,
        })
    }
}

impl RosMessage for MultiArrayDimension {
    const TYPE_NAME: &'static str = "example_interfaces::msg::dds_::MultiArrayDimension_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}