//! std_msgs message types
//!
//! Standard message types including Header, String, and primitive wrappers.

// Import for macro-generated code
use crate::nano_ros_core;
use crate::nano_ros_serdes;

use crate::builtin_interfaces::Time;
use nano_ros_macros::RosMessage;

// Note: ROS 2 Humble doesn't support RIHS01 type hashes, so rmw_zenoh uses
// "TypeHashNotSupported" as a placeholder. We use the same value for compatibility.
// For ROS 2 Iron/Jazzy, proper RIHS01 hashes would be needed.

/// Header message (std_msgs/msg/Header)
///
/// Standard metadata for higher-level stamped data types.
#[derive(Debug, Clone, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Header_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct Header<const N: usize = 64> {
    /// Timestamp
    pub stamp: Time,
    /// Frame this data is associated with
    pub frame_id: heapless::String<N>,
}

impl<const N: usize> Header<N> {
    /// Create a new Header with the given stamp and frame_id
    pub fn new(stamp: Time, frame_id: &str) -> Self {
        Self {
            stamp,
            frame_id: heapless::String::try_from(frame_id).unwrap_or_default(),
        }
    }
}

/// String message (std_msgs/msg/String)
#[derive(Debug, Clone, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::String_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct String<const N: usize = 256> {
    /// String data
    pub data: heapless::String<N>,
}

impl<const N: usize> String<N> {
    /// Create a new String message
    pub fn new(data: &str) -> Self {
        Self {
            data: heapless::String::try_from(data).unwrap_or_default(),
        }
    }
}

/// Bool message (std_msgs/msg/Bool)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Bool_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct Bool {
    /// Boolean data
    pub data: bool,
}

/// Byte message (std_msgs/msg/Byte)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Byte_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct Byte {
    /// Byte data
    pub data: u8,
}

/// Char message (std_msgs/msg/Char)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Char_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct Char {
    /// Char data (single byte)
    pub data: u8,
}

/// Int8 message (std_msgs/msg/Int8)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Int8_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct Int8 {
    /// Int8 data
    pub data: i8,
}

/// Int16 message (std_msgs/msg/Int16)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Int16_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct Int16 {
    /// Int16 data
    pub data: i16,
}

/// Int32 message (std_msgs/msg/Int32)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Int32_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct Int32 {
    /// Int32 data
    pub data: i32,
}

/// Int64 message (std_msgs/msg/Int64)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Int64_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct Int64 {
    /// Int64 data
    pub data: i64,
}

/// UInt8 message (std_msgs/msg/UInt8)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::UInt8_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct UInt8 {
    /// UInt8 data
    pub data: u8,
}

/// UInt16 message (std_msgs/msg/UInt16)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::UInt16_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct UInt16 {
    /// UInt16 data
    pub data: u16,
}

/// UInt32 message (std_msgs/msg/UInt32)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::UInt32_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct UInt32 {
    /// UInt32 data
    pub data: u32,
}

/// UInt64 message (std_msgs/msg/UInt64)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::UInt64_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct UInt64 {
    /// UInt64 data
    pub data: u64,
}

/// Float32 message (std_msgs/msg/Float32)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Float32_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct Float32 {
    /// Float32 data
    pub data: f32,
}

/// Float64 message (std_msgs/msg/Float64)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Float64_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct Float64 {
    /// Float64 data
    pub data: f64,
}

/// Empty message (std_msgs/msg/Empty)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Empty_")]
#[ros(hash = "TypeHashNotSupported")]
pub struct Empty {}

#[cfg(test)]
mod tests {
    use super::*;
    use nano_ros_core::{CdrReader, CdrWriter, Deserialize, RosMessage, Serialize};

    #[test]
    fn test_string_roundtrip() {
        let mut buf = [0u8; 64];
        let msg = String::<32>::new("Hello, ROS!");

        let mut writer = CdrWriter::new(&mut buf);
        msg.serialize(&mut writer).unwrap();

        let mut reader = CdrReader::new(&buf);
        let result = String::<32>::deserialize(&mut reader).unwrap();
        assert_eq!(result.data.as_str(), "Hello, ROS!");
    }

    #[test]
    fn test_header_roundtrip() {
        let mut buf = [0u8; 128];
        let header = Header::<32>::new(Time::new(100, 500), "base_link");

        let mut writer = CdrWriter::new(&mut buf);
        header.serialize(&mut writer).unwrap();

        let mut reader = CdrReader::new(&buf);
        let result = Header::<32>::deserialize(&mut reader).unwrap();
        assert_eq!(result.stamp, header.stamp);
        assert_eq!(result.frame_id.as_str(), "base_link");
    }

    #[test]
    fn test_int32_roundtrip() {
        let mut buf = [0u8; 16];
        let msg = Int32 { data: -42 };

        let mut writer = CdrWriter::new(&mut buf);
        msg.serialize(&mut writer).unwrap();

        let mut reader = CdrReader::new(&buf);
        let result = Int32::deserialize(&mut reader).unwrap();
        assert_eq!(result.data, -42);
    }

    #[test]
    fn test_type_names() {
        assert_eq!(String::<32>::TYPE_NAME, "std_msgs::msg::dds_::String_");
        assert_eq!(Int32::TYPE_NAME, "std_msgs::msg::dds_::Int32_");
        assert_eq!(Empty::TYPE_NAME, "std_msgs::msg::dds_::Empty_");
    }
}
