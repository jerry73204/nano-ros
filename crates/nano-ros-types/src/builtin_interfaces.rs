//! builtin_interfaces message types
//!
//! - `Time` - Timestamp with seconds and nanoseconds
//! - `Duration` - Time duration with seconds and nanoseconds

// Import for macro-generated code
use crate::nano_ros_core;
use crate::nano_ros_serdes;

use nano_ros_macros::RosMessage;

/// Time message (builtin_interfaces/msg/Time)
///
/// Represents a point in time as seconds since epoch plus nanoseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, RosMessage)]
#[ros(type_name = "builtin_interfaces::msg::dds_::Time_")]
#[ros(hash = "2a7e58fae8c3d029ca13feb70bdf0e71f11a9e9935f53c29e3b29a8ce548327f")]
pub struct Time {
    /// Seconds since epoch
    pub sec: i32,
    /// Nanoseconds (0-999999999)
    pub nanosec: u32,
}

impl Time {
    /// Create a new Time
    pub const fn new(sec: i32, nanosec: u32) -> Self {
        Self { sec, nanosec }
    }

    /// Create a Time representing zero
    pub const fn zero() -> Self {
        Self { sec: 0, nanosec: 0 }
    }
}

/// Duration message (builtin_interfaces/msg/Duration)
///
/// Represents a time duration as seconds plus nanoseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, RosMessage)]
#[ros(type_name = "builtin_interfaces::msg::dds_::Duration_")]
#[ros(hash = "8e3d03a1ae1b9524ed3e0c0f2a3c5c74c14a674eb8c68ece2c91e5c7e8d5c67e")]
pub struct Duration {
    /// Seconds
    pub sec: i32,
    /// Nanoseconds (0-999999999)
    pub nanosec: u32,
}

impl Duration {
    /// Create a new Duration
    pub const fn new(sec: i32, nanosec: u32) -> Self {
        Self { sec, nanosec }
    }

    /// Create a Duration representing zero
    pub const fn zero() -> Self {
        Self { sec: 0, nanosec: 0 }
    }

    /// Create a Duration from seconds
    pub const fn from_secs(secs: i32) -> Self {
        Self {
            sec: secs,
            nanosec: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nano_ros_core::{CdrReader, CdrWriter, Deserialize, RosMessage, Serialize};

    #[test]
    fn test_time_roundtrip() {
        let mut buf = [0u8; 16];
        let time = Time::new(1234567890, 123456789);

        let mut writer = CdrWriter::new(&mut buf);
        time.serialize(&mut writer).unwrap();

        let mut reader = CdrReader::new(&buf);
        let result = Time::deserialize(&mut reader).unwrap();
        assert_eq!(result, time);
    }

    #[test]
    fn test_time_type_info() {
        assert_eq!(Time::TYPE_NAME, "builtin_interfaces::msg::dds_::Time_");
    }
}
