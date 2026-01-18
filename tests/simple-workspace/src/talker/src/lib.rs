//! nano-ros talker example (no_std)
//!
//! Demonstrates message serialization without std.

#![no_std]

use nano_ros_core::{RosMessage, Serialize};
use nano_ros_serdes::CdrWriter;
use std_msgs::msg::Int32;

/// Serialize an Int32 message to a buffer
pub fn serialize_int32(value: i32, buffer: &mut [u8]) -> Result<usize, ()> {
    let msg = Int32 { data: value };

    let mut writer = CdrWriter::new(buffer);

    // Write CDR header (little-endian)
    writer.write_u8(0x00).map_err(|_| ())?;
    writer.write_u8(0x01).map_err(|_| ())?;
    writer.write_u8(0x00).map_err(|_| ())?;
    writer.write_u8(0x00).map_err(|_| ())?;

    // Serialize message
    msg.serialize(&mut writer).map_err(|_| ())?;

    Ok(writer.position())
}

/// Get the type name for Int32
pub fn get_type_name() -> &'static str {
    Int32::TYPE_NAME
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_serialize_int32() {
        let mut buffer = [0u8; 64];
        let len = serialize_int32(42, &mut buffer).unwrap();

        // CDR header (4 bytes) + i32 (4 bytes) = 8 bytes
        assert_eq!(len, 8);

        // Check CDR header
        assert_eq!(&buffer[0..4], &[0x00, 0x01, 0x00, 0x00]);

        // Check i32 value (little-endian)
        assert_eq!(&buffer[4..8], &42i32.to_le_bytes());
    }
}
