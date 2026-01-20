// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: MultiArrayLayout

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// MultiArrayLayout message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MultiArrayLayout {
    pub dim: heapless::Vec<crate::msg::MultiArrayDimension, 64>,
    pub data_offset: u32,
}

impl Serialize for MultiArrayLayout {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_u32(self.dim.len() as u32)?;
        for item in &self.dim {
            item.serialize(writer)?;
        }
        writer.write_u32(self.data_offset)?;
        Ok(())
    }
}

impl Deserialize for MultiArrayLayout {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            dim: {
                let len = reader.read_u32()? as usize;
                let mut vec = heapless::Vec::new();
                for _ in 0..len {
                    vec.push(Deserialize::deserialize(reader)?).map_err(|_| DeserError::CapacityExceeded)?;
                }
                vec
            },
            data_offset: reader.read_u32()?,
        })
    }
}

impl RosMessage for MultiArrayLayout {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::MultiArrayLayout_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}