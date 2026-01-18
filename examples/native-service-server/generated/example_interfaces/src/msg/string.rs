// nano-ros message type - pure Rust, no_std compatible
// Package: example_interfaces
// Message: String

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};



/// String message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct String {
    
    pub data: heapless::String<256>,
    
}

impl Serialize for String {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        
        
        writer.write_string(self.data.as_str())?;
        
        
        Ok(())
    }
}

impl Deserialize for String {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            
            
            data: {
                let s = reader.read_string()?;
                heapless::String::try_from(s).map_err(|_| DeserError::CapacityExceeded)?
            },
            
            
        })
    }
}

impl RosMessage for String {
    const TYPE_NAME: &'static str = "example_interfaces::msg::dds_::String_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}