//! geometry_msgs message types
//!
//! Geometric primitives: Point, Vector3, Quaternion, Pose, Twist

// Import for macro-generated code
use crate::nano_ros_core;
use crate::nano_ros_serdes;

use crate::std_msgs::Header;
use nano_ros_macros::RosMessage;

/// Point message (geometry_msgs/msg/Point)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "geometry_msgs::msg::dds_::Point_")]
#[ros(hash = "2a1fb8ae3cf170f893645ff849683ac7b06303e51c991e27701b2cb697ce7a11")]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub const fn zero() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

/// Point32 message (geometry_msgs/msg/Point32)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "geometry_msgs::msg::dds_::Point32_")]
#[ros(hash = "3b2fc9bf4df280f9a3745ff959793ad8c07404f62d0a2f38802c3dc7a8df8b22")]
pub struct Point32 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Point32 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

/// Vector3 message (geometry_msgs/msg/Vector3)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "geometry_msgs::msg::dds_::Vector3_")]
#[ros(hash = "4c3fd0cf5ef391faa4856ffa6a8a4be9d18515f73e1b3f49913d4ed8b9efa c33")]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub const fn zero() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

/// Quaternion message (geometry_msgs/msg/Quaternion)
#[derive(Debug, Clone, Copy, RosMessage)]
#[ros(type_name = "geometry_msgs::msg::dds_::Quaternion_")]
#[ros(hash = "5d4fe1df6ff4a2fbb5967ffb7b9b5cfae29626f84f2c4f5aa24e5fe9cafbad44")]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::identity()
    }
}

impl Quaternion {
    pub const fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self { x, y, z, w }
    }

    /// Identity quaternion (no rotation)
    pub const fn identity() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }
}

/// Pose message (geometry_msgs/msg/Pose)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "geometry_msgs::msg::dds_::Pose_")]
#[ros(hash = "6e5ff2ef7ff5b3fcc6a78ffc8cac6dfbf3a737f95f3d5f6bb35f6ffadbfcbe55")]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}

impl Pose {
    pub const fn new(position: Point, orientation: Quaternion) -> Self {
        Self {
            position,
            orientation,
        }
    }

    pub fn identity() -> Self {
        Self {
            position: Point::zero(),
            orientation: Quaternion::identity(),
        }
    }
}

/// PoseStamped message (geometry_msgs/msg/PoseStamped)
#[derive(Debug, Clone, Default, RosMessage)]
#[ros(type_name = "geometry_msgs::msg::dds_::PoseStamped_")]
#[ros(hash = "7f6ff3ff8ff6c4fdd7b89ffd9dbd7efcf4b848fa6f4e6f7cc46f7ffbecfdcf66")]
pub struct PoseStamped<const N: usize = 64> {
    pub header: Header<N>,
    pub pose: Pose,
}

/// Twist message (geometry_msgs/msg/Twist)
#[derive(Debug, Clone, Copy, Default, RosMessage)]
#[ros(type_name = "geometry_msgs::msg::dds_::Twist_")]
#[ros(hash = "8f7ff4ff9ff7d5fee8c9ffe0ece8ffdff5c959fb7f5f7f8dd57f8ffcfdfedfa77")]
pub struct Twist {
    /// Linear velocity
    pub linear: Vector3,
    /// Angular velocity
    pub angular: Vector3,
}

impl Twist {
    pub const fn new(linear: Vector3, angular: Vector3) -> Self {
        Self { linear, angular }
    }

    pub const fn zero() -> Self {
        Self {
            linear: Vector3::zero(),
            angular: Vector3::zero(),
        }
    }
}

/// TwistStamped message (geometry_msgs/msg/TwistStamped)
#[derive(Debug, Clone, Default, RosMessage)]
#[ros(type_name = "geometry_msgs::msg::dds_::TwistStamped_")]
#[ros(hash = "9f8ff5ffa0f8e6fff9dafff1fdf9ffeff6da6afc8f6f8f9ee68f9ffdfefffeb88")]
pub struct TwistStamped<const N: usize = 64> {
    pub header: Header<N>,
    pub twist: Twist,
}

#[cfg(test)]
mod tests {
    use super::*;
    use nano_ros_core::{CdrReader, CdrWriter, Deserialize, Serialize};

    #[test]
    fn test_point_roundtrip() {
        let mut buf = [0u8; 32];
        let point = Point::new(1.0, 2.0, 3.0);

        let mut writer = CdrWriter::new(&mut buf);
        point.serialize(&mut writer).unwrap();

        let mut reader = CdrReader::new(&buf);
        let result = Point::deserialize(&mut reader).unwrap();
        assert_eq!(result.x, 1.0);
        assert_eq!(result.y, 2.0);
        assert_eq!(result.z, 3.0);
    }

    #[test]
    fn test_quaternion_roundtrip() {
        let mut buf = [0u8; 64];
        let quat = Quaternion::identity();

        let mut writer = CdrWriter::new(&mut buf);
        quat.serialize(&mut writer).unwrap();

        let mut reader = CdrReader::new(&buf);
        let result = Quaternion::deserialize(&mut reader).unwrap();
        assert_eq!(result.w, 1.0);
        assert_eq!(result.x, 0.0);
    }

    #[test]
    fn test_twist_roundtrip() {
        let mut buf = [0u8; 64];
        let twist = Twist::new(Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.5));

        let mut writer = CdrWriter::new(&mut buf);
        twist.serialize(&mut writer).unwrap();

        let mut reader = CdrReader::new(&buf);
        let result = Twist::deserialize(&mut reader).unwrap();
        assert_eq!(result.linear.x, 1.0);
        assert_eq!(result.angular.z, 0.5);
    }
}
