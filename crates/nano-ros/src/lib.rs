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
//! Message types are generated from ROS 2 interface packages using `cargo nano-ros generate`.
//! See the examples for how to set up bindings.
//!
//! ```ignore
//! use nano_ros::prelude::*;
//! use std_msgs::msg::Int32;  // Generated bindings
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

#![no_std]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

// Re-export core types
pub use nano_ros_core::{
    CdrReader, CdrWriter, Clock, ClockType, Deserialize, Duration, RosMessage, RosService,
    Serialize, Time,
};

// Re-export node types
pub use nano_ros_node::{NodeConfig, PublisherHandle, StandaloneNode, SubscriberHandle};

// Re-export timer types
pub use nano_ros_node::{
    TimerCallbackFn, TimerDuration, TimerHandle, TimerMode, TimerState, DEFAULT_MAX_TIMERS,
};

// Re-export connected node types (with zenoh feature)
#[cfg(feature = "zenoh")]
pub use nano_ros_node::{
    ConnectedActionClient, ConnectedActionServer, ConnectedNode, ConnectedNodeError,
    ConnectedPublisher, ConnectedServiceClient, ConnectedServiceServer, ConnectedSubscriber,
};

// Re-export new rclrs-style API types (with zenoh feature)
#[cfg(feature = "zenoh")]
pub use nano_ros_node::{
    Context, InitOptions, IntoNodeOptions, IntoPublisherOptions, IntoSubscriberOptions, Node,
    NodeNameExt, NodeOptions, PublisherOptions, RclrsError, SubscriberOptions,
};

// Re-export transport types
pub use nano_ros_transport::{
    QosDurabilityPolicy, QosHistoryPolicy, QosReliabilityPolicy, QosSettings, ServiceInfo,
    SessionMode, TopicInfo, TransportConfig, TransportError,
};

// Re-export zenoh-specific types
#[cfg(feature = "zenoh")]
pub use nano_ros_transport::{Ros2Liveliness, ZenohSession, ZenohTransport};

// Re-export service types
pub use nano_ros_core::{ServiceClient, ServiceServer};

// Re-export action types
pub use nano_ros_core::{
    ActionClient, ActionServer, CancelResponse, GoalId, GoalResponse, GoalStatus, RosAction,
};

// Re-export parameter types
pub use nano_ros_params::{
    Parameter, ParameterDescriptor, ParameterServer, ParameterType, ParameterValue,
    SetParameterResult,
};

/// Prelude module for convenient imports
///
/// Import everything you need with a single statement:
/// ```
/// use nano_ros::prelude::*;
/// ```
pub mod prelude {
    pub use crate::{
        CdrReader, CdrWriter, Deserialize, NodeConfig, PublisherHandle, QosDurabilityPolicy,
        QosHistoryPolicy, QosReliabilityPolicy, QosSettings, RosMessage, RosService, Serialize,
        StandaloneNode, SubscriberHandle, TopicInfo, TransportConfig,
    };

    #[cfg(feature = "zenoh")]
    pub use crate::{
        ConnectedActionClient, ConnectedActionServer, ConnectedNode, ConnectedNodeError,
        ConnectedPublisher, ConnectedServiceClient, ConnectedServiceServer, ConnectedSubscriber,
        SessionMode,
    };

    // Re-export new rclrs-style API
    #[cfg(feature = "zenoh")]
    pub use crate::{
        Context, InitOptions, IntoNodeOptions, IntoPublisherOptions, IntoSubscriberOptions, Node,
        NodeNameExt, NodeOptions, PublisherOptions, RclrsError, SubscriberOptions,
    };

    // Re-export parameter types
    pub use crate::{ParameterServer, ParameterType, ParameterValue};

    // Re-export action types
    pub use crate::{GoalId, GoalResponse, GoalStatus, RosAction};

    // Re-export Time, Duration, Clock from core
    pub use nano_ros_core::{Clock, ClockType, Duration, Time};

    // Re-export timer types
    pub use crate::{TimerCallbackFn, TimerDuration, TimerHandle, TimerMode};
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
}
