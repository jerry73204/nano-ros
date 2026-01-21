//! Node abstraction for nano-ros
//!
//! This crate provides the high-level Node API for creating ROS 2 compatible
//! publishers and subscribers on embedded systems.
//!
//! # Example (Standalone Node)
//!
//! ```ignore
//! use nano_ros_node::{Node, NodeConfig};
//! use nano_ros_types::std_msgs::Int32;
//!
//! // Create a node (without transport)
//! let config = NodeConfig::new("my_node", "/my_namespace");
//! let mut node = Node::<4, 4>::new(config);
//!
//! // Create a publisher handle
//! let pub_handle = node.create_publisher::<Int32>("/counter").unwrap();
//!
//! // Serialize a message (caller handles transport)
//! let msg = Int32 { data: 42 };
//! let bytes = node.serialize_message(&pub_handle, &msg).unwrap();
//! ```
//!
//! # Example (Connected Node with Zenoh)
//!
//! ```ignore
//! use nano_ros_node::{ConnectedNode, NodeConfig};
//! use nano_ros_types::std_msgs::Int32;
//!
//! // Create a connected node
//! let config = NodeConfig::new("talker", "/demo");
//! let mut node = ConnectedNode::connect(config, "tcp/127.0.0.1:7447").unwrap();
//!
//! // Create a publisher and publish
//! let publisher = node.create_publisher::<Int32>("/counter").unwrap();
//! publisher.publish(&Int32 { data: 42 }).unwrap();
//! ```
//!
//! # Features
//!
//! - `std` - Enable standard library support
//! - `alloc` - Enable heap allocation
//! - `zenoh` - Enable zenoh transport integration

#![no_std]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

mod node;
mod publisher;
mod subscriber;

#[cfg(feature = "zenoh")]
mod connected;

#[cfg(all(feature = "zenoh", feature = "rtic"))]
pub mod rtic;

pub use node::{Node, NodeConfig, NodeError};
pub use publisher::PublisherHandle;
pub use subscriber::SubscriberHandle;

// Re-export transport types for convenience
pub use nano_ros_transport::{ActionInfo, QosSettings, TopicInfo, TransportConfig, TransportError};

// Re-export connected types when zenoh feature is enabled
#[cfg(feature = "zenoh")]
pub use connected::{
    // Action types
    ActiveGoal,
    ConnectedActionClient,
    ConnectedActionServer,
    // Node types
    ConnectedNode,
    ConnectedNodeError,
    ConnectedPublisher,
    ConnectedServiceClient,
    ConnectedServiceServer,
    ConnectedSubscriber,
    GoalHandle,
    // Buffer size constants
    DEFAULT_FEEDBACK_BUFFER_SIZE,
    DEFAULT_GOAL_BUFFER_SIZE,
    DEFAULT_MAX_ACTIVE_GOALS,
    DEFAULT_MAX_TOKENS,
    DEFAULT_REPLY_BUFFER_SIZE,
    DEFAULT_REQ_BUFFER_SIZE,
    DEFAULT_RESULT_BUFFER_SIZE,
    DEFAULT_RX_BUFFER_SIZE,
};

// Re-export zenoh transport types for convenience
#[cfg(feature = "zenoh")]
pub use nano_ros_transport::SessionMode;
