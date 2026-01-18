// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: UInt16

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};



/// UInt16 message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct UInt16 {
    
    pub data: u16,
    
}

impl Serialize for UInt16 {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        
        
        writer.write_u16(self.data)?;
        
        
        Ok(())
    }
}

impl Deserialize for UInt16 {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            
            
            data: reader.read_u16()?,
            
            
        })
    }
}

impl RosMessage for UInt16 {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::UInt16_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}