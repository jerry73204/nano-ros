//! std_msgs message types
//!
//! Standard message types including Header, String, and primitive wrappers.

// Import for macro-generated code
use crate::nano_ros_core;
use crate::nano_ros_serdes;

use crate::builtin_interfaces::Time;
use nano_ros_macros::RosMessage;

/// Header message (std_msgs/msg/Header)
///
/// Standard metadata for higher-level stamped data types.
#[derive(Debug, Clone, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Header_")]
#[ros(hash = "f49fb3ae2cf070f793645ff749683ac6b06203e41c891e17701b1cb597ce6a01")]
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
#[ros(hash = "df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18")]
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
#[ros(hash = "feb91e995ff9ebd09c0cb3d2aed18b11077585839fb5db80193b62d74528f6c9")]
pub struct Bool {
    /// Boolean data
    pub data: bool,
}

/// Byte message (std_msgs/msg/Byte)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Byte_")]
#[ros(hash = "41e1a3345f73fe93ede006da826a6ee274af23dd4653976ff249b0f44e3e798f")]
pub struct Byte {
    /// Byte data
    pub data: u8,
}

/// Char message (std_msgs/msg/Char)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Char_")]
#[ros(hash = "8914cecc0520f475fc3f8767a9f9e529341a75440e1b184a7e152195b2ffce5f")]
pub struct Char {
    /// Char data (single byte)
    pub data: u8,
}

/// Int8 message (std_msgs/msg/Int8)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Int8_")]
#[ros(hash = "26525065a403d972cb672f0777e333f0c799ad444ae5fcd79e43d1e73bd0f440")]
pub struct Int8 {
    /// Int8 data
    pub data: i8,
}

/// Int16 message (std_msgs/msg/Int16)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Int16_")]
#[ros(hash = "1dcc3464e47c288a55f943a389d337cdb06804de3f5cd7a266b0de718eee17e5")]
pub struct Int16 {
    /// Int16 data
    pub data: i16,
}

/// Int32 message (std_msgs/msg/Int32)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Int32_")]
#[ros(hash = "b6578ded3c58c626cfe8d1a6fb6e04f706f97e9f03d2727c9ff4e74b1cef0deb")]
pub struct Int32 {
    /// Int32 data
    pub data: i32,
}

/// Int64 message (std_msgs/msg/Int64)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Int64_")]
#[ros(hash = "8cd1048c2f186b6bd9a92472dc1ce51723c0833a221e2b7aecfff111774f4b49")]
pub struct Int64 {
    /// Int64 data
    pub data: i64,
}

/// UInt8 message (std_msgs/msg/UInt8)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::UInt8_")]
#[ros(hash = "6138bd83d8c3569cb80a667db03cfc1629f529fee79d944c39c34e352e72f010")]
pub struct UInt8 {
    /// UInt8 data
    pub data: u8,
}

/// UInt16 message (std_msgs/msg/UInt16)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::UInt16_")]
#[ros(hash = "08a406e4b022bc22e907f985d6a9e9dd1d4fbecae573549cf49350113e7757b1")]
pub struct UInt16 {
    /// UInt16 data
    pub data: u16,
}

/// UInt32 message (std_msgs/msg/UInt32)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::UInt32_")]
#[ros(hash = "a5c874829b752bc5fa190024b0ad76f578cc278271e855c7d02a818b3516fb4a")]
pub struct UInt32 {
    /// UInt32 data
    pub data: u32,
}

/// UInt64 message (std_msgs/msg/UInt64)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::UInt64_")]
#[ros(hash = "fbdc52018fc13755dce18024d1a671c856aa8b4aaf63adfb095b608f98e8c943")]
pub struct UInt64 {
    /// UInt64 data
    pub data: u64,
}

/// Float32 message (std_msgs/msg/Float32)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Float32_")]
#[ros(hash = "7170d3d8f841f7be3172ce5f4f59f3a4d7f63b0447e8b33327601ad64d83d6e2")]
pub struct Float32 {
    /// Float32 data
    pub data: f32,
}

/// Float64 message (std_msgs/msg/Float64)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Float64_")]
#[ros(hash = "705ba9c3d1a09df43737eb67095534de36fd426c0587779bda2bc51fe790182a")]
pub struct Float64 {
    /// Float64 data
    pub data: f64,
}

/// Empty message (std_msgs/msg/Empty)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::Empty_")]
#[ros(hash = "af191b259a9f5d32af085f8bc5b730e66d31196fa3297ff03bc458c3705ecf5e")]
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
