// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: Int8MultiArray

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};



/// Int8MultiArray message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Int8MultiArray {
    
    pub layout: crate::msg::MultiArrayLayout,
    
    pub data: heapless::Vec<i8, 64>,
    
}

impl Serialize for Int8MultiArray {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        
        
        self.layout.serialize(writer)?;
        
        
        
        writer.write_u32(self.data.len() as u32)?;
        for item in &self.data {
            
            writer.write_i8(*item)?;
            
        }
        
        
        Ok(())
    }
}

impl Deserialize for Int8MultiArray {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            
            
            layout: Deserialize::deserialize(reader)?,
            
            
            
            data: {
                let len = reader.read_u32()? as usize;
                let mut vec = heapless::Vec::new();
                for _ in 0..len {
                    
                    vec.push(reader.read_i8()?).map_err(|_| DeserError::CapacityExceeded)?;
                    
                }
                vec
            },
            
            
        })
    }
}

impl RosMessage for Int8MultiArray {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::Int8MultiArray_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}