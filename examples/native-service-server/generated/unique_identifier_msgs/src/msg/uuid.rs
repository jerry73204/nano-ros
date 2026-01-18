// nano-ros message type - pure Rust, no_std compatible
// Package: unique_identifier_msgs
// Message: UUID

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};



/// UUID message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct UUID {
    
    pub uuid: [u8; 16],
    
}

impl Serialize for UUID {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        
        
        for item in &self.uuid {
            
            writer.write_u8(*item)?;
            
        }
        
        
        Ok(())
    }
}

impl Deserialize for UUID {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            
            
            uuid: {
                let mut arr: [u8; 16] = Default::default();
                for i in 0..16 {
                    
                    arr[i] = reader.read_u8()?;
                    
                }
                arr
            },
            
            
        })
    }
}

impl RosMessage for UUID {
    const TYPE_NAME: &'static str = "unique_identifier_msgs::msg::dds_::UUID_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}