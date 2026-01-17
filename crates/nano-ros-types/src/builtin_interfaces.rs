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
#[ros(hash = "b106235e25a4c5ed35098aa0a61a3ee9c9b18d197f398b0e4206cea9acf9c197")]
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
#[ros(hash = "e8d009f659816f758b75334ee1a9ca5b5c0b859843261f14c7f937349599d93b")]
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
