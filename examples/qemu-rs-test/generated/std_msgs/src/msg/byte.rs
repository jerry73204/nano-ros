// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: Byte

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};



/// Byte message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Byte {
    
    pub data: u8,
    
}

impl Serialize for Byte {
    
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        
        
        writer.write_u8(self.data)?;
        
        
        Ok(())
    }
    
}

impl Deserialize for Byte {
    
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            
            
            data: reader.read_u8()?,
            
            
        })
    }
    
}

impl RosMessage for Byte {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::Byte_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}