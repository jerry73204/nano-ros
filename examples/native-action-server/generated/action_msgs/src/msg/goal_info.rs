// nano-ros message type - pure Rust, no_std compatible
// Package: action_msgs
// Message: GoalInfo

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// GoalInfo message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GoalInfo {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub stamp: builtin_interfaces::msg::Time,
}

impl Serialize for GoalInfo {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        self.goal_id.serialize(writer)?;
        self.stamp.serialize(writer)?;
        Ok(())
    }
}

impl Deserialize for GoalInfo {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            goal_id: Deserialize::deserialize(reader)?,
            stamp: Deserialize::deserialize(reader)?,
        })
    }
}

impl RosMessage for GoalInfo {
    const TYPE_NAME: &'static str = "action_msgs::msg::dds_::GoalInfo_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}