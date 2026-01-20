// nano-ros message type - pure Rust, no_std compatible
// Package: std_msgs
// Message: Int16MultiArray

use nano_ros_core::{RosMessage, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};



/// Int16MultiArray message type
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Int16MultiArray {
    
    pub layout: crate::msg::MultiArrayLayout,
    
    pub data: heapless::Vec<i16, 64>,
    
}

impl Serialize for Int16MultiArray {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        
        
        self.layout.serialize(writer)?;
        
        
        
        writer.write_u32(self.data.len() as u32)?;
        for item in &self.data {
            
            writer.write_i16(*item)?;
            
        }
        
        
        Ok(())
    }
}

impl Deserialize for Int16MultiArray {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            
            
            layout: Deserialize::deserialize(reader)?,
            
            
            
            data: {
                let len = reader.read_u32()? as usize;
                let mut vec = heapless::Vec::new();
                for _ in 0..len {
                    
                    vec.push(reader.read_i16()?).map_err(|_| DeserError::CapacityExceeded)?;
                    
                }
                vec
            },
            
            
        })
    }
}

impl RosMessage for Int16MultiArray {
    const TYPE_NAME: &'static str = "std_msgs::msg::dds_::Int16MultiArray_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}