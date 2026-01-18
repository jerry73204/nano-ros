// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: Int16

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};



/// Int16 message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Int16 {
    
    pub data: i16,
    
}

impl Serialize for Int16 {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        
        
        writer.write_i16(self.data)?;
        
        
        Ok(())
    }
}

impl Deserialize for Int16 {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            
            
            data: reader.read_i16()?,
            
            
        })
    }
}

impl RosMessage for Int16 {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::Int16_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}