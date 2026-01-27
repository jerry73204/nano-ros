// nano-ros message type - pure Rust, no_std compatible
// Package: builtin_interfaces
// Message: Duration

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};



/// Duration message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Duration {
    
    pub sec: i32,
    
    pub nanosec: u32,
    
}

impl Serialize for Duration {
    
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        
        
        writer.write_i32(self.sec)?;
        
        
        
        writer.write_u32(self.nanosec)?;
        
        
        Ok(())
    }
    
}

impl Deserialize for Duration {
    
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            
            
            sec: reader.read_i32()?,
            
            
            
            nanosec: reader.read_u32()?,
            
            
        })
    }
    
}

impl RosMessage for Duration {
    const TYPE_NAME: &'static str = "builtin_interfaces::msg::dds_::Duration_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}