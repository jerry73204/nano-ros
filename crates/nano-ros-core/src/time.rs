//! ROS time types

use nano_ros_serdes::{CdrReader, CdrWriter, DeserError, Deserialize, SerError, Serialize};

/// ROS Time representation
///
/// Matches `builtin_interfaces/msg/Time`:
/// - `sec`: Seconds since epoch (signed for pre-epoch times)
/// - `nanosec`: Nanoseconds within the second (0-999999999)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Time {
    /// Seconds component
    pub sec: i32,
    /// Nanoseconds component (0-999999999)
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

    /// Check if this time is zero
    pub const fn is_zero(&self) -> bool {
        self.sec == 0 && self.nanosec == 0
    }
}

impl Serialize for Time {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i32(self.sec)?;
        writer.write_u32(self.nanosec)?;
        Ok(())
    }
}

impl Deserialize for Time {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            sec: reader.read_i32()?,
            nanosec: reader.read_u32()?,
        })
    }
}

/// ROS Duration representation
///
/// Matches `builtin_interfaces/msg/Duration`:
/// - `sec`: Seconds (signed for negative durations)
/// - `nanosec`: Nanoseconds (0-999999999)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Duration {
    /// Seconds component
    pub sec: i32,
    /// Nanoseconds component (0-999999999)
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

    /// Check if this duration is zero
    pub const fn is_zero(&self) -> bool {
        self.sec == 0 && self.nanosec == 0
    }

    /// Create a Duration from seconds
    pub const fn from_secs(secs: i32) -> Self {
        Self {
            sec: secs,
            nanosec: 0,
        }
    }

    /// Create a Duration from milliseconds
    pub const fn from_millis(millis: i64) -> Self {
        let sec = (millis / 1000) as i32;
        let nanosec = ((millis % 1000) * 1_000_000) as u32;
        Self { sec, nanosec }
    }
}

impl Serialize for Duration {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i32(self.sec)?;
        writer.write_u32(self.nanosec)?;
        Ok(())
    }
}

impl Deserialize for Duration {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            sec: reader.read_i32()?,
            nanosec: reader.read_u32()?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn test_duration_roundtrip() {
        let mut buf = [0u8; 16];
        let duration = Duration::new(-10, 500000000);

        let mut writer = CdrWriter::new(&mut buf);
        duration.serialize(&mut writer).unwrap();

        let mut reader = CdrReader::new(&buf);
        let result = Duration::deserialize(&mut reader).unwrap();
        assert_eq!(result, duration);
    }
}
