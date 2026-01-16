//! Core ROS type traits

use nano_ros_serdes::{Deserialize, Serialize};

/// Trait for ROS message types
///
/// All ROS message types must implement this trait along with `Serialize` and `Deserialize`.
/// The type name and hash are used for topic matching and type safety.
pub trait RosMessage: Sized + Serialize + Deserialize {
    /// Full ROS type name in DDS format
    ///
    /// Example: `"std_msgs::msg::dds_::String_"`
    const TYPE_NAME: &'static str;

    /// RIHS (ROS Interface Hashing Standard) type hash
    ///
    /// Used for type validation between publishers and subscribers.
    /// Format: 64-character hex string (SHA-256)
    const TYPE_HASH: &'static str;
}

/// Trait for ROS service types
///
/// Associates request and reply message types with service metadata.
pub trait RosService {
    /// The request message type
    type Request: RosMessage;

    /// The reply message type
    type Reply: RosMessage;

    /// Full ROS service type name in DDS format
    ///
    /// Example: `"std_srvs::srv::dds_::Empty_"`
    const SERVICE_NAME: &'static str;

    /// RIHS type hash for the service
    const SERVICE_HASH: &'static str;
}
