//! Node abstraction for nano-ros
//!
//! This crate provides the high-level Node API for creating ROS 2 compatible
//! publishers and subscribers on embedded systems.
//!
//! # Recommended: Executor-Based API
//!
//! The executor-based API provides a unified interface that works on both
//! std (desktop) and no_std (embedded) targets.
//!
//! ## Desktop Example (BasicExecutor)
//!
//! ```ignore
//! use nano_ros_node::{Context, InitOptions, SpinOptions};
//! use std_msgs::msg::Int32;
//!
//! // Create context and executor
//! let ctx = Context::new(InitOptions::new().locator("tcp/127.0.0.1:7447"))?;
//! let mut executor = ctx.create_basic_executor();
//!
//! // Create node through executor
//! let node = executor.create_node("my_node")?;
//!
//! // Create subscription with callback
//! node.create_subscription::<Int32>("/topic", |msg| {
//!     println!("Received: {}", msg.data);
//! })?;
//!
//! // Create publisher and publish
//! let publisher = node.create_publisher::<Int32>("/counter")?;
//! publisher.publish(&Int32 { data: 42 })?;
//!
//! // Blocking spin (processes callbacks)
//! executor.spin(SpinOptions::default());
//! ```
//!
//! ## Embedded Example (PollingExecutor)
//!
//! ```ignore
//! use nano_ros_node::{Context, InitOptions};
//! use std_msgs::msg::Int32;
//!
//! // Create context and polling executor
//! let ctx = Context::new(InitOptions::new().locator("tcp/192.168.1.1:7447"))?;
//! let mut executor: PollingExecutor<2> = ctx.create_polling_executor();
//!
//! // Create node through executor
//! let node = executor.create_node("robot_node")?;
//!
//! // Create subscription with function pointer callback (no_std compatible)
//! fn handle_msg(msg: &Int32) {
//!     // process message...
//! }
//! node.create_subscription::<Int32>("/cmd", handle_msg)?;
//!
//! // In your RTIC task or main loop:
//! loop {
//!     executor.spin_once(10);  // 10ms delta
//!     // platform delay...
//! }
//! ```
//!
//! # Legacy: Direct Node Creation
//!
//! For simpler use cases or migration from older code:
//!
//! ```ignore
//! use nano_ros_node::{Context, InitOptions};
//!
//! let ctx = Context::new(InitOptions::new())?;
//! let node = ctx.create_node("my_node")?;  // Deprecated
//! ```
//!
//! # Features
//!
//! - `std` - Enable standard library support (BasicExecutor, blocking spin)
//! - `alloc` - Enable heap allocation (closures, dynamic callbacks)
//! - `zenoh` - Enable zenoh transport integration

#![no_std]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

mod node;
mod publisher;
mod subscriber;
pub mod timer;

#[cfg(feature = "zenoh")]
mod connected;

#[cfg(feature = "zenoh")]
mod context;

#[cfg(feature = "zenoh")]
mod options;

#[cfg(feature = "zenoh")]
pub mod executor;

#[cfg(all(feature = "zenoh", feature = "rtic"))]
pub mod rtic;

#[cfg(feature = "shim")]
pub mod shim;

// Export standalone node (without transport)
pub use node::{Node as StandaloneNode, NodeConfig, NodeError};

pub use publisher::PublisherHandle;
pub use subscriber::SubscriberHandle;

// Re-export transport types for convenience
pub use nano_ros_transport::{
    ActionInfo, QosDurabilityPolicy, QosHistoryPolicy, QosReliabilityPolicy, QosSettings,
    TopicInfo, TransportConfig, TransportError,
};

// Re-export connected types when zenoh feature is enabled
#[cfg(feature = "zenoh")]
pub use connected::{
    // Action types
    ActiveGoal,
    CompletedGoal,
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
    DEFAULT_CANCEL_BUFFER_SIZE,
    DEFAULT_FEEDBACK_BUFFER_SIZE,
    DEFAULT_GOAL_BUFFER_SIZE,
    DEFAULT_MAX_ACTIVE_GOALS,
    DEFAULT_MAX_TOKENS,
    DEFAULT_REPLY_BUFFER_SIZE,
    DEFAULT_REQ_BUFFER_SIZE,
    DEFAULT_RESULT_BUFFER_SIZE,
    DEFAULT_RX_BUFFER_SIZE,
    DEFAULT_STATUS_BUFFER_SIZE,
};

// Re-export context types when zenoh feature is enabled (rclrs-style API)
#[cfg(feature = "zenoh")]
pub use context::{
    Context, InitOptions, IntoNodeOptions, Node, NodeNameExt, NodeOptions, RclrsError,
};

// Re-export executor types
#[cfg(feature = "zenoh")]
pub use executor::{
    Executor, ExecutorTimerCallback, SpinOnceResult, SpinOptions, SubscriptionCallback,
    SubscriptionHandle, DEFAULT_MAX_NODES, DEFAULT_MAX_SUBSCRIPTIONS,
};

#[cfg(all(feature = "zenoh", feature = "alloc"))]
pub use executor::{NodeHandle, NodeState, PollingExecutor};

#[cfg(all(feature = "zenoh", feature = "std"))]
pub use executor::BasicExecutor;

// Re-export options types when zenoh feature is enabled
#[cfg(feature = "zenoh")]
pub use options::{
    IntoPublisherOptions, IntoSubscriberOptions, PublisherOptions, SubscriberOptions,
};

// Re-export options for standalone node when zenoh feature is not enabled
#[cfg(not(feature = "zenoh"))]
pub use node::{PublisherOptions, SubscriberOptions};

// Re-export zenoh transport types for convenience
#[cfg(feature = "zenoh")]
pub use nano_ros_transport::SessionMode;

// Re-export timer types
pub use timer::{
    TimerCallbackFn, TimerDuration, TimerHandle, TimerMode, TimerState, DEFAULT_MAX_TIMERS,
};

// Re-export shim types when shim feature is enabled
#[cfg(feature = "shim")]
pub use shim::{ShimExecutor, ShimNode, ShimNodePublisher, ShimNodeSubscriber};
