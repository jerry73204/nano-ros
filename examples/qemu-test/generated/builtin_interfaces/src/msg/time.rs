// nano-ros message type - pure Rust, no_std compatible
// Package: builtin_interfaces
// Message: Time

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};



/// Time message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Time {
    
    pub sec: i32,
    
    pub nanosec: u32,
    
}

impl Serialize for Time {
    
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        
        
        writer.write_i32(self.sec)?;
        
        
        
        writer.write_u32(self.nanosec)?;
        
        
        Ok(())
    }
    
}

impl Deserialize for Time {
    
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            
            
            sec: reader.read_i32()?,
            
            
            
            nanosec: reader.read_u32()?,
            
            
        })
    }
    
}

impl RosMessage for Time {
    const TYPE_NAME: &'static str = "builtin_interfaces::msg::dds_::Time_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}