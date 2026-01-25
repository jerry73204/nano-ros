// nano-ros message type - pure Rust, no_std compatible
// Package: action_msgs
// Message: GoalStatusArray

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// GoalStatusArray message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GoalStatusArray {
    pub status_list: heapless::Vec<crate::msg::GoalStatus, 64>,
}

impl Serialize for GoalStatusArray {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_u32(self.status_list.len() as u32)?;
        for item in &self.status_list {
            item.serialize(writer)?;
        }
        Ok(())
    }
}

impl Deserialize for GoalStatusArray {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            status_list: {
                let len = reader.read_u32()? as usize;
                let mut vec = heapless::Vec::new();
                for _ in 0..len {
                    vec.push(Deserialize::deserialize(reader)?).map_err(|_| DeserError::CapacityExceeded)?;
                }
                vec
            },
        })
    }
}

impl RosMessage for GoalStatusArray {
    const TYPE_NAME: &'static str = "action_msgs::msg::dds_::GoalStatusArray_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}