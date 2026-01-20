// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: Header

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};



/// Header message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Header {
    
    pub stamp: builtin_interfaces::msg::Time,
    
    pub frame_id: heapless::String<256>,
    
}

impl Serialize for Header {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        
        
        self.stamp.serialize(writer)?;
        
        
        
        writer.write_string(self.frame_id.as_str())?;
        
        
        Ok(())
    }
}

impl Deserialize for Header {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            
            
            stamp: Deserialize::deserialize(reader)?,
            
            
            
            frame_id: {
                let s = reader.read_string()?;
                heapless::String::try_from(s).map_err(|_| DeserError::CapacityExceeded)?
            },
            
            
        })
    }
}

impl RosMessage for Header {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::Header_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}