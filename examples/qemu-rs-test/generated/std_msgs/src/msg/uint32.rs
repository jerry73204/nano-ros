// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: UInt32

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};



/// UInt32 message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct UInt32 {
    
    pub data: u32,
    
}

impl Serialize for UInt32 {
    
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        
        
        writer.write_u32(self.data)?;
        
        
        Ok(())
    }
    
}

impl Deserialize for UInt32 {
    
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            
            
            data: reader.read_u32()?,
            
            
        })
    }
    
}

impl RosMessage for UInt32 {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::UInt32_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}