// nano-ros message type - pure Rust, no_std compatible
// Package: example_interfaces
// Message: Int32MultiArray

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// Int32MultiArray message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Int32MultiArray {
    pub layout: crate::msg::MultiArrayLayout,
    pub data: heapless::Vec<i32, 64>,
}

impl Serialize for Int32MultiArray {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        self.layout.serialize(writer)?;
        writer.write_u32(self.data.len() as u32)?;
        for item in &self.data {
            writer.write_i32(*item)?;
        }
        Ok(())
    }
}

impl Deserialize for Int32MultiArray {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            layout: Deserialize::deserialize(reader)?,
            data: {
                let len = reader.read_u32()? as usize;
                let mut vec = heapless::Vec::new();
                for _ in 0..len {
                    vec.push(reader.read_i32()?).map_err(|_| DeserError::CapacityExceeded)?;
                }
                vec
            },
        })
    }
}

impl RosMessage for Int32MultiArray {
    const TYPE_NAME: &'static str = "example_interfaces::msg::dds_::Int32MultiArray_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}