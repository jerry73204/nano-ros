//! Context and initialization for rclrs-style API
//!
//! This module provides the Context type and related initialization types
//! that match the rclrs 0.6.0 API pattern.

use crate::NodeConfig;

#[cfg(feature = "zenoh")]
use crate::ConnectedNode;

#[cfg(feature = "zenoh")]
use nano_ros_transport::{SessionMode, TransportConfig};

/// Context for creating nodes
///
/// The Context holds shared initialization state and is the entry point
/// for creating nodes. This matches the rclrs API pattern.
///
/// # Examples
///
/// ```ignore
/// use nano_ros::Context;
///
/// // Create context from environment
/// let context = Context::from_env()?;
///
/// // Create context with custom domain ID
/// let context = Context::new(InitOptions::new().with_domain_id(Some(42)))?;
/// ```
#[derive(Debug, Clone)]
pub struct Context {
    /// ROS 2 domain ID (defaults to 0)
    domain_id: u32,
}

impl Context {
    /// Create a new context with the given options
    ///
    /// # Arguments
    /// * `options` - Initialization options
    ///
    /// # Returns
    /// A new Context instance
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let context = Context::new(InitOptions::new().with_domain_id(Some(42)))?;
    /// ```
    pub fn new(options: InitOptions) -> Result<Self, RclrsError> {
        let domain_id = options.domain_id.unwrap_or(0);
        Ok(Self { domain_id })
    }

    /// Create a context from environment variables
    ///
    /// Reads ROS_DOMAIN_ID environment variable if set.
    /// Falls back to domain ID 0 if not set.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let context = Context::from_env()?;
    /// ```
    #[cfg(feature = "std")]
    pub fn from_env() -> Result<Self, RclrsError> {
        let domain_id = std::env::var("ROS_DOMAIN_ID")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(0);
        Ok(Self { domain_id })
    }

    /// Create a context with default settings
    ///
    /// This is equivalent to `Context::new(InitOptions::new())` and
    /// uses domain ID 0.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let context = Context::default_from_env()?;
    /// ```
    pub fn default_from_env() -> Result<Self, RclrsError> {
        Ok(Self { domain_id: 0 })
    }

    /// Get the domain ID for this context
    pub fn domain_id(&self) -> u32 {
        self.domain_id
    }

    /// Check if the context is still valid
    ///
    /// Currently always returns true.
    pub fn ok(&self) -> bool {
        true
    }

    /// Create a node using this context (zenoh feature only)
    ///
    /// # Arguments
    /// * `options` - Node name or NodeOptions with optional namespace
    ///
    /// # Returns
    /// A new Node (Arc<ConnectedNode> on std, &mut ConnectedNode otherwise)
    ///
    /// # Examples
    ///
    /// ```ignore
    /// // Simple node creation
    /// let node = context.create_node("my_node")?;
    ///
    /// // With namespace
    /// let node = context.create_node("my_node".namespace("/ns"))?;
    /// ```
    #[cfg(feature = "zenoh")]
    pub fn create_node<'a>(&self, options: impl IntoNodeOptions<'a>) -> Result<Node, RclrsError> {
        let node_options = options.into_node_options();

        let config = NodeConfig {
            name: node_options.name,
            namespace: node_options.namespace.unwrap_or("/"),
            domain_id: self.domain_id,
        };

        // Default to client mode connecting to localhost
        let transport_config = TransportConfig {
            locator: Some("tcp/127.0.0.1:7447"),
            mode: SessionMode::Client,
        };
        let node = ConnectedNode::new(config, &transport_config)
            .map_err(|_| RclrsError::NodeCreationFailed)?;

        Ok(node)
    }
}

/// Initialization options for creating a Context
///
/// # Examples
///
/// ```
/// use nano_ros_node::InitOptions;
///
/// let options = InitOptions::new()
///     .with_domain_id(Some(42));
/// ```
#[derive(Debug, Clone, Default)]
pub struct InitOptions {
    /// ROS 2 domain ID (None means use default of 0)
    domain_id: Option<u32>,
}

impl InitOptions {
    /// Create new initialization options with defaults
    pub fn new() -> Self {
        Self { domain_id: None }
    }

    /// Set the ROS 2 domain ID
    ///
    /// # Arguments
    /// * `domain_id` - Optional domain ID (None means use default of 0)
    pub fn with_domain_id(mut self, domain_id: Option<u32>) -> Self {
        self.domain_id = domain_id;
        self
    }
}

/// Options for creating a node
///
/// # Examples
///
/// ```
/// use nano_ros_node::NodeOptions;
///
/// let options = NodeOptions::new("my_node")
///     .namespace("/my_namespace");
/// ```
#[derive(Debug, Clone)]
pub struct NodeOptions<'a> {
    /// Node name
    pub name: &'a str,
    /// Node namespace (optional, defaults to "/")
    pub namespace: Option<&'a str>,
}

impl<'a> NodeOptions<'a> {
    /// Create new node options with the given name
    pub fn new(name: &'a str) -> Self {
        Self {
            name,
            namespace: None,
        }
    }

    /// Set the namespace for this node
    ///
    /// # Arguments
    /// * `ns` - Namespace string (should start with "/")
    pub fn namespace(mut self, ns: &'a str) -> Self {
        self.namespace = Some(ns);
        self
    }
}

/// Trait for types that can be converted into NodeOptions
///
/// This enables the fluent API pattern:
/// ```ignore
/// context.create_node("my_node".namespace("/ns"))
/// ```
pub trait IntoNodeOptions<'a> {
    /// Convert into NodeOptions
    fn into_node_options(self) -> NodeOptions<'a>;
}

impl<'a> IntoNodeOptions<'a> for &'a str {
    fn into_node_options(self) -> NodeOptions<'a> {
        NodeOptions::new(self)
    }
}

impl<'a> IntoNodeOptions<'a> for NodeOptions<'a> {
    fn into_node_options(self) -> NodeOptions<'a> {
        self
    }
}

/// Extension trait for string slices to enable fluent node creation
pub trait NodeNameExt<'a> {
    /// Set the namespace for a node name
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let node = context.create_node("my_node".namespace("/ns"))?;
    /// ```
    fn namespace(self, ns: &'a str) -> NodeOptions<'a>;
}

impl<'a> NodeNameExt<'a> for &'a str {
    fn namespace(self, ns: &'a str) -> NodeOptions<'a> {
        NodeOptions::new(self).namespace(ns)
    }
}

/// Node type alias for convenience
///
/// Currently returns raw ConnectedNode. In future phases, this will be
/// wrapped with interior mutability (Mutex/RefCell) for shared ownership.
#[cfg(feature = "zenoh")]
pub type Node<const MAX_TOKENS: usize = { crate::DEFAULT_MAX_TOKENS }> = ConnectedNode<MAX_TOKENS>;

/// Error type for rclrs-style API
///
/// This will eventually replace ConnectedNodeError to match rclrs naming.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RclrsError {
    /// Failed to create context
    ContextCreationFailed,
    /// Failed to create node
    NodeCreationFailed,
    /// Failed to connect to transport
    ConnectionFailed,
    /// Failed to create publisher
    PublisherCreationFailed,
    /// Failed to create subscriber
    SubscriberCreationFailed,
    /// Failed to create service server
    ServiceServerCreationFailed,
    /// Failed to create service client
    ServiceClientCreationFailed,
    /// Failed to create action server
    ActionServerCreationFailed,
    /// Failed to create action client
    ActionClientCreationFailed,
    /// Failed to publish message
    PublishFailed,
    /// Serialization failed
    SerializationFailed,
    /// Deserialization failed
    DeserializationFailed,
    /// Buffer too small
    BufferTooSmall,
    /// No message available
    NoMessage,
    /// Service request failed
    ServiceRequestFailed,
    /// Service reply failed
    ServiceReplyFailed,
    /// Failed to start background tasks
    TaskStartFailed,
    /// Failed to poll for incoming messages
    PollFailed,
    /// Failed to send keepalive
    KeepaliveFailed,
    /// Failed to send join message
    JoinFailed,
    /// Goal was rejected
    GoalRejected,
    /// Goal not found
    GoalNotFound,
    /// Action server is full (too many active goals)
    ActionServerFull,
}

impl RclrsError {
    /// Return the first error from a list, or Ok if the list is empty
    ///
    /// This is useful for error handling in spin loops.
    pub fn first_error(errors: impl IntoIterator<Item = Self>) -> Result<(), Self> {
        errors.into_iter().next().map(Err).unwrap_or(Ok(()))
    }
}

#[cfg(feature = "zenoh")]
impl From<crate::ConnectedNodeError> for RclrsError {
    fn from(e: crate::ConnectedNodeError) -> Self {
        use crate::ConnectedNodeError;
        match e {
            ConnectedNodeError::ConnectionFailed => RclrsError::ConnectionFailed,
            ConnectedNodeError::PublisherCreationFailed => RclrsError::PublisherCreationFailed,
            ConnectedNodeError::SubscriberCreationFailed => RclrsError::SubscriberCreationFailed,
            ConnectedNodeError::ServiceServerCreationFailed => {
                RclrsError::ServiceServerCreationFailed
            }
            ConnectedNodeError::ServiceClientCreationFailed => {
                RclrsError::ServiceClientCreationFailed
            }
            ConnectedNodeError::ActionServerCreationFailed => {
                RclrsError::ActionServerCreationFailed
            }
            ConnectedNodeError::ActionClientCreationFailed => {
                RclrsError::ActionClientCreationFailed
            }
            ConnectedNodeError::PublishFailed => RclrsError::PublishFailed,
            ConnectedNodeError::SerializationFailed => RclrsError::SerializationFailed,
            ConnectedNodeError::DeserializationFailed => RclrsError::DeserializationFailed,
            ConnectedNodeError::BufferTooSmall => RclrsError::BufferTooSmall,
            ConnectedNodeError::NoMessage => RclrsError::NoMessage,
            ConnectedNodeError::ServiceRequestFailed => RclrsError::ServiceRequestFailed,
            ConnectedNodeError::ServiceReplyFailed => RclrsError::ServiceReplyFailed,
            ConnectedNodeError::TaskStartFailed => RclrsError::TaskStartFailed,
            ConnectedNodeError::PollFailed => RclrsError::PollFailed,
            ConnectedNodeError::KeepaliveFailed => RclrsError::KeepaliveFailed,
            ConnectedNodeError::JoinFailed => RclrsError::JoinFailed,
            ConnectedNodeError::GoalRejected => RclrsError::GoalRejected,
            ConnectedNodeError::GoalNotFound => RclrsError::GoalNotFound,
            ConnectedNodeError::ActionServerFull => RclrsError::ActionServerFull,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "alloc")]
    extern crate alloc;
    #[cfg(feature = "alloc")]
    use alloc::vec;

    #[test]
    fn test_init_options() {
        let options = InitOptions::new();
        assert_eq!(options.domain_id, None);

        let options = InitOptions::new().with_domain_id(Some(42));
        assert_eq!(options.domain_id, Some(42));
    }

    #[test]
    fn test_context_creation() {
        let context = Context::new(InitOptions::new()).unwrap();
        assert_eq!(context.domain_id(), 0);
        assert!(context.ok());

        let context = Context::new(InitOptions::new().with_domain_id(Some(42))).unwrap();
        assert_eq!(context.domain_id(), 42);
    }

    #[test]
    fn test_context_default() {
        let context = Context::default_from_env().unwrap();
        assert_eq!(context.domain_id(), 0);
    }

    #[test]
    fn test_node_options() {
        let options = NodeOptions::new("test_node");
        assert_eq!(options.name, "test_node");
        assert_eq!(options.namespace, None);

        let options = NodeOptions::new("test_node").namespace("/test_ns");
        assert_eq!(options.name, "test_node");
        assert_eq!(options.namespace, Some("/test_ns"));
    }

    #[test]
    fn test_into_node_options() {
        let options: NodeOptions = "test_node".into_node_options();
        assert_eq!(options.name, "test_node");
        assert_eq!(options.namespace, None);
    }

    #[test]
    fn test_node_name_ext() {
        let options = "test_node".namespace("/test_ns");
        assert_eq!(options.name, "test_node");
        assert_eq!(options.namespace, Some("/test_ns"));
    }

    #[test]
    #[cfg(feature = "alloc")]
    fn test_rclrs_error_first_error() {
        let errors = vec![RclrsError::ConnectionFailed, RclrsError::PublishFailed];
        let result = RclrsError::first_error(errors);
        assert_eq!(result, Err(RclrsError::ConnectionFailed));

        let errors: alloc::vec::Vec<RclrsError> = vec![];
        let result = RclrsError::first_error(errors);
        assert_eq!(result, Ok(()));
    }
}
