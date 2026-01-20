// nano-ros service type - pure Rust, no_std compatible
// Package: action_msgs
// Service: CancelGoal

use nano_ros_core::{RosMessage, RosService, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// CancelGoal request message
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CancelGoalRequest {
    pub goal_info: crate::msg::GoalInfo,
}

impl Serialize for CancelGoalRequest {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        self.goal_info.serialize(writer)?;
        Ok(())
    }
}

impl Deserialize for CancelGoalRequest {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            goal_info: Deserialize::deserialize(reader)?,
        })
    }
}

impl RosMessage for CancelGoalRequest {
    const TYPE_NAME: &'static str = "action_msgs::srv::dds_::CancelGoal_Request_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}
pub const ERROR_NONE: i8 = 0;
pub const ERROR_REJECTED: i8 = 1;
pub const ERROR_UNKNOWN_GOAL_ID: i8 = 2;
pub const ERROR_GOAL_TERMINATED: i8 = 3;

/// CancelGoal response message
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CancelGoalResponse {
    pub return_code: i8,
    pub goals_canceling: heapless::Vec<crate::msg::GoalInfo, 64>,
}

impl Serialize for CancelGoalResponse {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i8(self.return_code)?;
        writer.write_u32(self.goals_canceling.len() as u32)?;
        for item in &self.goals_canceling {
            item.serialize(writer)?;
        }
        Ok(())
    }
}

impl Deserialize for CancelGoalResponse {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            return_code: reader.read_i8()?,
            goals_canceling: {
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

impl RosMessage for CancelGoalResponse {
    const TYPE_NAME: &'static str = "action_msgs::srv::dds_::CancelGoal_Response_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}

/// CancelGoal service definition
pub struct CancelGoal;

impl RosService for CancelGoal {
    type Request = CancelGoalRequest;
    type Reply = CancelGoalResponse;

    const SERVICE_NAME: &'static str = "action_msgs::srv::dds_::CancelGoal_";
    const SERVICE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}