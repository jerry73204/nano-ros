// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: Empty

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};



/// Empty message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Empty {
    
}

impl Serialize for Empty {
    
    // Empty message - no fields to serialize
    fn serialize(&self, _writer: &mut CdrWriter) -> Result<(), SerError> {
        Ok(())
    }
    
}

impl Deserialize for Empty {
    
    // Empty message - no fields to deserialize
    fn deserialize(_reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {})
    }
    
}

impl RosMessage for Empty {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::Empty_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}