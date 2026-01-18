//! nano-ros listener example (no_std)
//!
//! Demonstrates message deserialization without std.

#![no_std]

use nano_ros_core::{Deserialize, RosMessage};
use nano_ros_serdes::CdrReader;
use std_msgs::msg::Int32;

/// Deserialize an Int32 message from a buffer
pub fn deserialize_int32(buffer: &[u8]) -> Result<i32, ()> {
    // Skip CDR header (4 bytes)
    if buffer.len() < 4 {
        return Err(());
    }

    let mut reader = CdrReader::new(&buffer[4..]);
    let msg = Int32::deserialize(&mut reader).map_err(|_| ())?;

    Ok(msg.data)
}

/// Get the type name for Int32
pub fn get_type_name() -> &'static str {
    Int32::TYPE_NAME
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_deserialize_int32() {
        // CDR header + i32 value 42
        let buffer = [
            0x00, 0x01, 0x00, 0x00, // CDR header (little-endian)
            0x2a, 0x00, 0x00, 0x00, // i32 value 42 (little-endian)
        ];

        let value = deserialize_int32(&buffer).unwrap();
        assert_eq!(value, 42);
    }

    #[test]
    fn test_roundtrip() {
        // Serialize
        let original = 12345i32;
        let mut buffer = [0u8; 64];

        // Write CDR header
        buffer[0] = 0x00;
        buffer[1] = 0x01;
        buffer[2] = 0x00;
        buffer[3] = 0x00;

        // Write value
        buffer[4..8].copy_from_slice(&original.to_le_bytes());

        // Deserialize
        let recovered = deserialize_int32(&buffer).unwrap();
        assert_eq!(recovered, original);
    }
}
