// nano-ros message type - pure Rust, no_std compatible
// Package: example_interfaces
// Message: UInt64MultiArray

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// UInt64MultiArray message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct UInt64MultiArray {
    pub layout: crate::msg::MultiArrayLayout,
    pub data: heapless::Vec<u64, 64>,
}

impl Serialize for UInt64MultiArray {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        self.layout.serialize(writer)?;
        writer.write_u32(self.data.len() as u32)?;
        for item in &self.data {
            writer.write_u64(*item)?;
        }
        Ok(())
    }
}

impl Deserialize for UInt64MultiArray {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            layout: Deserialize::deserialize(reader)?,
            data: {
                let len = reader.read_u32()? as usize;
                let mut vec = heapless::Vec::new();
                for _ in 0..len {
                    vec.push(reader.read_u64()?).map_err(|_| DeserError::CapacityExceeded)?;
                }
                vec
            },
        })
    }
}

impl RosMessage for UInt64MultiArray {
    const TYPE_NAME: &'static str = "example_interfaces::msg::dds_::UInt64MultiArray_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}