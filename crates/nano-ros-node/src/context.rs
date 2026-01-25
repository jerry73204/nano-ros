//! Context and initialization for rclrs-style API
//!
//! This module provides the Context type and related initialization types
//! that match the rclrs 0.6.0 API pattern.
//!
//! # Unified Executor API
//!
//! The recommended way to use nano-ros is through the executor API:
//!
//! ```ignore
//! use nano_ros::prelude::*;
//!
//! // Create context
//! let ctx = Context::new(InitOptions::new().locator("tcp/127.0.0.1:7447"))?;
//!
//! // Create executor (choose one)
//! let mut executor = ctx.create_basic_executor();      // std: has spin()
//! // let mut executor = ctx.create_polling_executor(); // no_std: manual spin_once()
//!
//! // Create node through executor
//! let node = executor.create_node("my_node")?;
//!
//! // Create subscriptions with callbacks
//! node.create_subscription::<Int32>("/topic", |msg| {
//!     println!("Received: {}", msg.data);
//! })?;
//!
//! // Run the executor
//! executor.spin(SpinOptions::default());
//! ```

use crate::NodeConfig;

#[cfg(feature = "zenoh")]
use crate::ConnectedNode;

#[cfg(feature = "zenoh")]
use nano_ros_transport::{SessionMode, TransportConfig};

#[cfg(all(feature = "zenoh", feature = "alloc"))]
use crate::executor::{PollingExecutor, DEFAULT_MAX_NODES};

#[cfg(all(feature = "zenoh", feature = "std"))]
use crate::executor::BasicExecutor;

/// Context for creating executors and nodes
///
/// The Context holds shared initialization state and is the entry point
/// for creating executors. This matches the rclrs API pattern.
///
/// # Recommended: Executor API
///
/// ```ignore
/// use nano_ros::prelude::*;
///
/// let ctx = Context::new(InitOptions::new().locator("tcp/127.0.0.1:7447"))?;
/// let mut executor = ctx.create_basic_executor();
/// let node = executor.create_node("my_node")?;
/// ```
///
/// # Legacy: Direct Node Creation
///
/// ```ignore
/// // Deprecated - use executor API instead
/// let ctx = Context::new(InitOptions::new())?;
/// let node = ctx.create_node("my_node")?;  // Deprecated
/// ```
#[derive(Debug, Clone)]
pub struct Context {
    /// ROS 2 domain ID (defaults to 0)
    domain_id: u32,
    /// Transport configuration for zenoh connections
    #[cfg(feature = "zenoh")]
    transport_config: TransportConfig<'static>,
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
    /// let context = Context::new(InitOptions::new()
    ///     .with_domain_id(Some(42))
    ///     .locator("tcp/127.0.0.1:7447"))?;
    /// ```
    #[cfg(feature = "zenoh")]
    pub fn new(options: InitOptions) -> Result<Self, RclrsError> {
        let domain_id = options.domain_id.unwrap_or(0);
        let transport_config = TransportConfig {
            locator: options.locator,
            mode: options.session_mode,
        };
        Ok(Self {
            domain_id,
            transport_config,
        })
    }

    /// Create a new context with the given options (non-zenoh version)
    #[cfg(not(feature = "zenoh"))]
    pub fn new(options: InitOptions) -> Result<Self, RclrsError> {
        let domain_id = options.domain_id.unwrap_or(0);
        Ok(Self { domain_id })
    }

    /// Create a context from environment variables
    ///
    /// Reads the following environment variables:
    /// - `ROS_DOMAIN_ID`: Domain ID (default: 0)
    /// - `ZENOH_MODE`: Session mode - "peer" for peer mode, otherwise client mode
    /// - `ZENOH_LOCATOR`: Locator for client mode (default: "tcp/127.0.0.1:7447")
    ///
    /// In peer mode, no locator is needed as peers discover each other via multicast.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// // Client mode (default)
    /// let context = Context::from_env()?;
    ///
    /// // Peer mode (set ZENOH_MODE=peer)
    /// std::env::set_var("ZENOH_MODE", "peer");
    /// let context = Context::from_env()?;
    /// ```
    #[cfg(all(feature = "std", feature = "zenoh"))]
    pub fn from_env() -> Result<Self, RclrsError> {
        let domain_id = std::env::var("ROS_DOMAIN_ID")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(0);

        // Check for peer mode
        let is_peer_mode = std::env::var("ZENOH_MODE")
            .map(|v| v.eq_ignore_ascii_case("peer"))
            .unwrap_or(false);

        let transport_config = if is_peer_mode {
            TransportConfig {
                locator: None,
                mode: SessionMode::Peer,
            }
        } else {
            // Client mode - use ZENOH_LOCATOR or default
            // Note: We leak the string to get a 'static lifetime. This is acceptable
            // for CLI applications where the context lives for the program duration.
            let locator: Option<&'static str> = std::env::var("ZENOH_LOCATOR")
                .ok()
                .map(|s| -> &'static str { std::boxed::Box::leak(s.into_boxed_str()) });
            TransportConfig {
                locator: locator.or(Some("tcp/127.0.0.1:7447")),
                mode: SessionMode::Client,
            }
        };

        Ok(Self {
            domain_id,
            transport_config,
        })
    }

    /// Create a context from environment variables (non-zenoh version)
    #[cfg(all(feature = "std", not(feature = "zenoh")))]
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
    /// uses domain ID 0 with default locator.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let context = Context::default_from_env()?;
    /// ```
    #[cfg(feature = "zenoh")]
    pub fn default_from_env() -> Result<Self, RclrsError> {
        let transport_config = TransportConfig {
            locator: Some("tcp/127.0.0.1:7447"),
            mode: SessionMode::Client,
        };
        Ok(Self {
            domain_id: 0,
            transport_config,
        })
    }

    /// Create a context with default settings (non-zenoh version)
    #[cfg(not(feature = "zenoh"))]
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

    // ═══════════════════════════════════════════════════════════════════════
    // EXECUTOR CREATION (New API)
    // ═══════════════════════════════════════════════════════════════════════

    /// Create a polling executor (no_std compatible)
    ///
    /// The polling executor requires manual calls to `spin_once()` and is
    /// suitable for RTIC, Embassy, or bare-metal applications.
    ///
    /// # Type Parameters
    ///
    /// - `MAX_NODES`: Maximum number of nodes this executor can manage (default: 4)
    ///
    /// # Example
    ///
    /// ```ignore
    /// let ctx = Context::new(InitOptions::new().locator("tcp/192.168.1.1:7447"))?;
    /// let mut executor: PollingExecutor<2> = ctx.create_polling_executor();
    /// let node = executor.create_node("my_node")?;
    ///
    /// // In main loop or RTIC task:
    /// loop {
    ///     executor.spin_once(10);  // 10ms delta
    ///     // delay...
    /// }
    /// ```
    #[cfg(all(feature = "zenoh", feature = "alloc"))]
    pub fn create_polling_executor<const MAX_NODES: usize>(&self) -> PollingExecutor<MAX_NODES> {
        PollingExecutor::new(self.domain_id, self.transport_config.clone())
    }

    /// Create a polling executor with default capacity
    #[cfg(all(feature = "zenoh", feature = "alloc"))]
    pub fn create_polling_executor_default(&self) -> PollingExecutor<DEFAULT_MAX_NODES> {
        self.create_polling_executor()
    }

    /// Create a basic executor with full spin support (std only)
    ///
    /// The basic executor provides `spin()` for blocking spin loops and
    /// `halt()` for stopping from another thread.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let ctx = Context::new(InitOptions::new())?;
    /// let mut executor = ctx.create_basic_executor();
    /// let node = executor.create_node("my_node")?;
    ///
    /// node.create_subscription::<Int32>("/topic", |msg| {
    ///     println!("Received: {}", msg.data);
    /// })?;
    ///
    /// // Blocking spin
    /// executor.spin(SpinOptions::default());
    /// ```
    #[cfg(all(feature = "zenoh", feature = "std"))]
    pub fn create_basic_executor(&self) -> BasicExecutor {
        BasicExecutor::new(self.domain_id, self.transport_config.clone())
    }

    // ═══════════════════════════════════════════════════════════════════════
    // LEGACY NODE CREATION (Deprecated)
    // ═══════════════════════════════════════════════════════════════════════

    /// Create a node using this context (zenoh feature only)
    ///
    /// **Deprecated**: Use `create_polling_executor()` or `create_basic_executor()`
    /// instead for the new executor-based API.
    ///
    /// # Arguments
    /// * `options` - Node name or NodeOptions with optional namespace
    ///
    /// # Returns
    /// A new Node (ConnectedNode)
    ///
    /// # Examples
    ///
    /// ```ignore
    /// // Simple node creation (deprecated)
    /// let node = context.create_node("my_node")?;
    ///
    /// // Recommended: use executor API instead
    /// let mut executor = context.create_basic_executor();
    /// let node = executor.create_node("my_node")?;
    /// ```
    #[cfg(feature = "zenoh")]
    #[deprecated(
        since = "0.2.0",
        note = "Use create_polling_executor() or create_basic_executor() instead"
    )]
    #[allow(deprecated)] // Internal use of ConnectedNode::new() is intentional
    pub fn create_node<'a>(&self, options: impl IntoNodeOptions<'a>) -> Result<Node, RclrsError> {
        let node_options = options.into_node_options();

        let config = NodeConfig {
            name: node_options.name,
            namespace: node_options.namespace.unwrap_or("/"),
            domain_id: self.domain_id,
        };

        let node = ConnectedNode::new(config, &self.transport_config)
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
///
/// With zenoh transport:
///
/// ```ignore
/// use nano_ros_node::InitOptions;
/// use nano_ros_transport::SessionMode;
///
/// let options = InitOptions::new()
///     .with_domain_id(Some(0))
///     .locator("tcp/192.168.1.1:7447")
///     .session_mode(SessionMode::Client);
/// ```
#[derive(Debug, Clone)]
pub struct InitOptions {
    /// ROS 2 domain ID (None means use default of 0)
    pub(crate) domain_id: Option<u32>,
    /// Zenoh locator (e.g., "tcp/127.0.0.1:7447")
    #[cfg(feature = "zenoh")]
    pub(crate) locator: Option<&'static str>,
    /// Session mode (Client or Peer)
    #[cfg(feature = "zenoh")]
    pub(crate) session_mode: SessionMode,
}

impl Default for InitOptions {
    fn default() -> Self {
        Self::new()
    }
}

impl InitOptions {
    /// Create new initialization options with defaults
    pub fn new() -> Self {
        Self {
            domain_id: None,
            #[cfg(feature = "zenoh")]
            locator: Some("tcp/127.0.0.1:7447"),
            #[cfg(feature = "zenoh")]
            session_mode: SessionMode::Client,
        }
    }

    /// Set the ROS 2 domain ID
    ///
    /// # Arguments
    /// * `domain_id` - Optional domain ID (None means use default of 0)
    pub fn with_domain_id(mut self, domain_id: Option<u32>) -> Self {
        self.domain_id = domain_id;
        self
    }

    /// Set the domain ID directly
    pub fn domain_id(mut self, id: u32) -> Self {
        self.domain_id = Some(id);
        self
    }

    /// Set the zenoh locator
    ///
    /// # Arguments
    /// * `locator` - Locator string (e.g., "tcp/127.0.0.1:7447")
    #[cfg(feature = "zenoh")]
    pub fn locator(mut self, locator: &'static str) -> Self {
        self.locator = Some(locator);
        self
    }

    /// Set the session mode
    ///
    /// # Arguments
    /// * `mode` - Session mode (Client or Peer)
    #[cfg(feature = "zenoh")]
    pub fn session_mode(mut self, mode: SessionMode) -> Self {
        self.session_mode = mode;
        self
    }

    /// Configure for peer mode (no router required)
    #[cfg(feature = "zenoh")]
    pub fn peer_mode(mut self) -> Self {
        self.session_mode = SessionMode::Peer;
        self.locator = None;
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
    /// Failed to create timer
    TimerCreationFailed,
    /// Timer not found
    TimerNotFound,
    /// Timer storage is full (too many timers)
    TimerStorageFull,
    /// Executor is full (too many nodes)
    ExecutorFull,
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
            ConnectedNodeError::TimerCreationFailed => RclrsError::TimerCreationFailed,
            ConnectedNodeError::TimerNotFound => RclrsError::TimerNotFound,
            ConnectedNodeError::TimerStorageFull => RclrsError::TimerStorageFull,
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

        let options = InitOptions::new().domain_id(42);
        assert_eq!(options.domain_id, Some(42));
    }

    #[test]
    #[cfg(feature = "zenoh")]
    fn test_init_options_zenoh() {
        let options = InitOptions::new().locator("tcp/192.168.1.1:7447");
        assert_eq!(options.locator, Some("tcp/192.168.1.1:7447"));

        let options = InitOptions::new().peer_mode();
        assert_eq!(options.session_mode, SessionMode::Peer);
        assert_eq!(options.locator, None);
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
