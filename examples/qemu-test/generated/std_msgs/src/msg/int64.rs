// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: Int64

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};



/// Int64 message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Int64 {
    
    pub data: i64,
    
}

impl Serialize for Int64 {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        
        
        writer.write_i64(self.data)?;
        
        
        Ok(())
    }
}

impl Deserialize for Int64 {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            
            
            data: reader.read_i64()?,
            
            
        })
    }
}

impl RosMessage for Int64 {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::Int64_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}