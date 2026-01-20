// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: Bool

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};



/// Bool message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Bool {
    
    pub data: bool,
    
}

impl Serialize for Bool {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        
        
        writer.write_bool(self.data)?;
        
        
        Ok(())
    }
}

impl Deserialize for Bool {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            
            
            data: reader.read_bool()?,
            
            
        })
    }
}

impl RosMessage for Bool {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::Bool_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}