//! # nano-ros
//!
//! A lightweight ROS 2 client library for embedded systems.
//!
//! This crate provides a unified API for building ROS 2 nodes in Rust,
//! with support for `no_std` environments and embedded targets.
//!
//! ## Features
//!
//! - **no_std compatible**: Works on bare-metal and RTOS targets
//! - **Zero-copy where possible**: Minimizes memory allocations
//! - **Type-safe**: Compile-time verification of message types
//! - **ROS 2 compatible**: Interoperates with standard ROS 2 nodes via rmw_zenoh
//!
//! ## Quick Start
//!
//! ```no_run
//! use nano_ros::prelude::*;
//! use nano_ros::types::std_msgs::Int32;
//!
//! // Create a node
//! let config = NodeConfig::new("my_node");
//! let mut node = ConnectedNode::connect(config, "tcp/127.0.0.1:7447")
//!     .expect("Failed to connect");
//!
//! // Create a publisher
//! let publisher = node.create_publisher::<Int32>("/my_topic")
//!     .expect("Failed to create publisher");
//!
//! // Publish a message
//! let msg = Int32 { data: 42 };
//! publisher.publish(&msg).expect("Failed to publish");
//! ```
//!
//! ## Crate Features
//!
//! - `std` (default) - Enable standard library support
//! - `alloc` - Enable heap allocation without full std
//! - `zenoh` (default) - Enable zenoh-pico transport backend
//! - `all-types` - Enable all message type modules

#![no_std]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

// Re-export core types
pub use nano_ros_core::{
    CdrReader, CdrWriter, Deserialize, RosMessage, RosService, Serialize, Time,
};

// Re-export node types
pub use nano_ros_node::{Node, NodeConfig, PublisherHandle, SubscriberHandle};

// Re-export connected node types (with zenoh feature)
#[cfg(feature = "zenoh")]
pub use nano_ros_node::{
    ConnectedNode, ConnectedNodeError, ConnectedPublisher, ConnectedServiceClient,
    ConnectedServiceServer, ConnectedSubscriber,
};

// Re-export transport types
pub use nano_ros_transport::{
    QosSettings, ServiceInfo, SessionMode, TopicInfo, TransportConfig, TransportError,
};

// Re-export zenoh-specific types
#[cfg(feature = "zenoh")]
pub use nano_ros_transport::{Ros2Liveliness, ZenohSession, ZenohTransport};

// Re-export service types
pub use nano_ros_core::{ServiceClient, ServiceServer};

/// Standard message types
///
/// This module provides commonly used ROS 2 message types.
pub mod types {
    pub use nano_ros_types::builtin_interfaces;
    pub use nano_ros_types::geometry_msgs;
    pub use nano_ros_types::std_msgs;

    // Re-export commonly used types at the top level
    pub use nano_ros_types::std_msgs::{Bool, Float32, Float64, Int32, Int64, String};
}

/// Prelude module for convenient imports
///
/// Import everything you need with a single statement:
/// ```
/// use nano_ros::prelude::*;
/// ```
pub mod prelude {
    pub use crate::{
        CdrReader, CdrWriter, Deserialize, Node, NodeConfig, PublisherHandle, QosSettings,
        RosMessage, RosService, Serialize, SubscriberHandle, TopicInfo, TransportConfig,
    };

    #[cfg(feature = "zenoh")]
    pub use crate::{
        ConnectedNode, ConnectedNodeError, ConnectedPublisher, ConnectedServiceClient,
        ConnectedServiceServer, ConnectedSubscriber, SessionMode,
    };

    // Re-export Time from core
    pub use nano_ros_core::Time;
}

/// Derive macros for message types
///
/// Use these macros to generate message serialization code.
/// These macros help you create custom message types that are compatible
/// with ROS 2's CDR serialization format.
pub mod derive {
    pub use nano_ros_macros::RosMessage;
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_prelude_imports() {
        // This test just verifies that the prelude compiles
        use crate::prelude::*;

        let _ = NodeConfig::new("test_node", "/");
        let _ = QosSettings::BEST_EFFORT;
    }

    #[test]
    fn test_types_module() {
        use crate::types::std_msgs::Int32;

        let msg = Int32 { data: 42 };
        assert_eq!(msg.data, 42);
    }
}
