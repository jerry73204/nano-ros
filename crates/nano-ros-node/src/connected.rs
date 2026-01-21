//! Connected node implementation with transport integration
//!
//! This module provides types that integrate the Node API with actual
//! transport backends (like zenoh-pico) for sending and receiving messages.
//!
//! # Buffer Sizes
//!
//! All types use const generics for configurable buffer sizes:
//!
//! - `ConnectedNode<MAX_TOKENS>` - Maximum liveliness tokens (default: 16)
//! - `ConnectedSubscriber<M, RX_BUF>` - Receive buffer size (default: 1024)
//! - `ConnectedServiceServer<S, REQ_BUF, REPLY_BUF>` - Request/reply buffers (default: 1024)
//! - `ConnectedServiceClient<S, REQ_BUF, REPLY_BUF>` - Request/reply buffers (default: 1024)
//!
//! # Memory Usage
//!
//! For embedded systems, calculate memory as:
//! - ConnectedNode: ~256 bytes + (MAX_TOKENS * sizeof(LivelinessToken))
//! - ConnectedSubscriber: RX_BUF bytes + ~64 bytes overhead
//! - ConnectedServiceServer: REQ_BUF + REPLY_BUF bytes + ~64 bytes overhead
//! - ConnectedServiceClient: REQ_BUF + REPLY_BUF bytes + ~64 bytes overhead

use nano_ros_core::{
    Deserialize, GoalId, GoalResponse, GoalStatus, RosAction, RosMessage, RosService, Serialize,
};
use nano_ros_transport::{
    ActionInfo, Publisher as PublisherTrait, QosSettings, ServiceClientTrait, ServiceInfo,
    ServiceServerTrait, Session, Subscriber as SubscriberTrait, TopicInfo, Transport,
    TransportConfig, TransportError,
};

#[cfg(feature = "zenoh")]
use nano_ros_transport::{
    LivelinessToken, Ros2Liveliness, ZenohId, ZenohPublisher, ZenohServiceClient,
    ZenohServiceServer, ZenohSession, ZenohSubscriber, ZenohTransport,
};

use crate::NodeConfig;

/// Default receive buffer size for subscribers
pub const DEFAULT_RX_BUFFER_SIZE: usize = 1024;

/// Default request buffer size for services
pub const DEFAULT_REQ_BUFFER_SIZE: usize = 1024;

/// Default reply buffer size for services
pub const DEFAULT_REPLY_BUFFER_SIZE: usize = 1024;

/// Default goal buffer size for action servers/clients
pub const DEFAULT_GOAL_BUFFER_SIZE: usize = 1024;

/// Default feedback buffer size for action servers/clients
pub const DEFAULT_FEEDBACK_BUFFER_SIZE: usize = 1024;

/// Default result buffer size for action servers/clients
pub const DEFAULT_RESULT_BUFFER_SIZE: usize = 1024;

/// Error type for connected node operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectedNodeError {
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

impl From<TransportError> for ConnectedNodeError {
    fn from(e: TransportError) -> Self {
        match e {
            TransportError::ConnectionFailed => ConnectedNodeError::ConnectionFailed,
            TransportError::PublisherCreationFailed => ConnectedNodeError::PublisherCreationFailed,
            TransportError::SubscriberCreationFailed => {
                ConnectedNodeError::SubscriberCreationFailed
            }
            TransportError::ServiceServerCreationFailed => {
                ConnectedNodeError::ServiceServerCreationFailed
            }
            TransportError::ServiceClientCreationFailed => {
                ConnectedNodeError::ServiceClientCreationFailed
            }
            TransportError::PublishFailed => ConnectedNodeError::PublishFailed,
            TransportError::SerializationError => ConnectedNodeError::SerializationFailed,
            TransportError::DeserializationError => ConnectedNodeError::DeserializationFailed,
            TransportError::BufferTooSmall => ConnectedNodeError::BufferTooSmall,
            TransportError::ServiceRequestFailed => ConnectedNodeError::ServiceRequestFailed,
            TransportError::ServiceReplyFailed => ConnectedNodeError::ServiceReplyFailed,
            TransportError::TaskStartFailed => ConnectedNodeError::TaskStartFailed,
            TransportError::PollFailed => ConnectedNodeError::PollFailed,
            TransportError::KeepaliveFailed => ConnectedNodeError::KeepaliveFailed,
            TransportError::JoinFailed => ConnectedNodeError::JoinFailed,
            _ => ConnectedNodeError::ConnectionFailed,
        }
    }
}

/// Default maximum number of liveliness tokens (publishers + subscribers)
pub const DEFAULT_MAX_TOKENS: usize = 16;

/// A connected node that can publish and subscribe via zenoh transport
///
/// # Type Parameters
///
/// - `MAX_TOKENS`: Maximum number of liveliness tokens (publishers + subscribers).
///   Each publisher and subscriber requires one token for ROS 2 discovery.
///   Default is 16, allowing up to 16 combined publishers and subscribers.
#[cfg(feature = "zenoh")]
pub struct ConnectedNode<const MAX_TOKENS: usize = DEFAULT_MAX_TOKENS> {
    /// Node name
    name: heapless::String<64>,
    /// Node namespace
    namespace: heapless::String<64>,
    /// Domain ID
    domain_id: u32,
    /// Zenoh session
    session: ZenohSession,
    /// Zenoh session ID (for liveliness key expressions)
    zid: ZenohId,
    /// Node liveliness token (for ROS 2 discovery)
    _node_token: Option<LivelinessToken>,
    /// Publisher/subscriber liveliness tokens (static allocation)
    _entity_tokens: heapless::Vec<LivelinessToken, MAX_TOKENS>,
}

#[cfg(feature = "zenoh")]
impl<const MAX_TOKENS: usize> ConnectedNode<MAX_TOKENS> {
    /// Create a new connected node
    ///
    /// # Arguments
    /// * `config` - Node configuration (name, namespace, domain_id)
    /// * `transport_config` - Transport configuration (locator, mode)
    pub fn new(
        config: NodeConfig,
        transport_config: &TransportConfig,
    ) -> Result<Self, ConnectedNodeError> {
        let session = ZenohTransport::open(transport_config)?;

        let mut name = heapless::String::new();
        let _ = name.push_str(config.name);

        let mut namespace = heapless::String::new();
        let _ = namespace.push_str(config.namespace);

        // Get session ID for liveliness tokens
        let zid = session.zid();

        // Declare node liveliness token for ROS 2 discovery
        let node_keyexpr = Ros2Liveliness::node_keyexpr(config.domain_id, &zid, config.name);
        #[cfg(feature = "log")]
        log::debug!("Node liveliness keyexpr: {}", node_keyexpr);
        let node_token = session.declare_liveliness(&node_keyexpr).ok();

        Ok(Self {
            name,
            namespace,
            domain_id: config.domain_id,
            session,
            zid,
            _node_token: node_token,
            _entity_tokens: heapless::Vec::new(),
        })
    }

    /// Create a new connected node without starting background tasks
    ///
    /// Use this for RTIC or other single-threaded executors where you need
    /// manual control over when network I/O occurs.
    ///
    /// After creating, you must periodically call:
    /// - `poll_read()` to process incoming messages (recommended: every 10ms)
    /// - `send_keepalive()` to maintain the session (recommended: every 1s)
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub fn new_without_tasks(
        config: NodeConfig,
        transport_config: &TransportConfig,
    ) -> Result<Self, ConnectedNodeError> {
        let session = ZenohSession::new_without_tasks(transport_config)?;

        let mut name = heapless::String::new();
        let _ = name.push_str(config.name);

        let mut namespace = heapless::String::new();
        let _ = namespace.push_str(config.namespace);

        // Get session ID for liveliness tokens
        let zid = session.zid();

        // Declare node liveliness token for ROS 2 discovery
        let node_keyexpr = Ros2Liveliness::node_keyexpr(config.domain_id, &zid, config.name);
        #[cfg(feature = "log")]
        log::debug!("Node liveliness keyexpr: {}", node_keyexpr);
        let node_token = session.declare_liveliness(&node_keyexpr).ok();

        Ok(Self {
            name,
            namespace,
            domain_id: config.domain_id,
            session,
            zid,
            _node_token: node_token,
            _entity_tokens: heapless::Vec::new(),
        })
    }

    /// Connect to a zenoh router without starting background tasks
    ///
    /// Use this for RTIC or other single-threaded executors.
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub fn connect_without_tasks(
        config: NodeConfig,
        locator: &str,
    ) -> Result<Self, ConnectedNodeError> {
        let transport_config = TransportConfig {
            locator: Some(locator),
            mode: SessionMode::Client,
        };
        Self::new_without_tasks(config, &transport_config)
    }

    /// Connect in peer mode without starting background tasks
    ///
    /// Use this for RTIC or other single-threaded executors.
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub fn connect_peer_without_tasks(config: NodeConfig) -> Result<Self, ConnectedNodeError> {
        let transport_config = TransportConfig {
            locator: None,
            mode: SessionMode::Peer,
        };
        Self::new_without_tasks(config, &transport_config)
    }

    /// Start background read and lease tasks
    ///
    /// Only call this if you used `new_without_tasks()` and later want to
    /// switch to background threads.
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub fn start_tasks(&mut self) -> Result<(), ConnectedNodeError> {
        self.session.start_tasks().map_err(ConnectedNodeError::from)
    }

    /// Stop background tasks
    ///
    /// After stopping, you must manually call `poll_read()` and `send_keepalive()`.
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub fn stop_tasks(&mut self) {
        self.session.stop_tasks();
    }

    /// Poll for incoming messages and events
    ///
    /// Call this periodically (recommended: every 10ms) when not using background tasks.
    /// This processes any pending network data and dispatches callbacks.
    ///
    /// For RTIC applications, call this from a periodic software task.
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub fn poll_read(&mut self) -> Result<(), ConnectedNodeError> {
        self.session.poll_read().map_err(ConnectedNodeError::from)
    }

    /// Send keepalive to maintain the session
    ///
    /// Call this periodically (recommended: every 1s) when not using background tasks.
    /// This sends keepalive messages to prevent the session from timing out.
    ///
    /// For RTIC applications, call this from a periodic software task.
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub fn send_keepalive(&mut self) -> Result<(), ConnectedNodeError> {
        self.session
            .send_keepalive()
            .map_err(ConnectedNodeError::from)
    }

    /// Send join message for peer discovery
    ///
    /// Call this periodically in peer mode to announce presence to other peers.
    /// Not needed in client mode.
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub fn send_join(&mut self) -> Result<(), ConnectedNodeError> {
        self.session.send_join().map_err(ConnectedNodeError::from)
    }

    /// Check if background read task is running
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub fn is_read_task_running(&self) -> bool {
        self.session.is_read_task_running()
    }

    /// Check if background lease task is running
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub fn is_lease_task_running(&self) -> bool {
        self.session.is_lease_task_running()
    }

    /// Get the node name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the node namespace
    pub fn namespace(&self) -> &str {
        &self.namespace
    }

    /// Get the domain ID
    pub fn domain_id(&self) -> u32 {
        self.domain_id
    }

    /// Create a publisher for the given topic
    pub fn create_publisher<M: RosMessage>(
        &mut self,
        topic: &str,
    ) -> Result<ConnectedPublisher<M>, ConnectedNodeError> {
        self.create_publisher_with_qos(topic, QosSettings::BEST_EFFORT)
    }

    /// Create a publisher with custom QoS settings
    pub fn create_publisher_with_qos<M: RosMessage>(
        &mut self,
        topic: &str,
        qos: QosSettings,
    ) -> Result<ConnectedPublisher<M>, ConnectedNodeError> {
        let topic_info =
            TopicInfo::new(topic, M::TYPE_NAME, M::TYPE_HASH).with_domain(self.domain_id);

        let publisher = self.session.create_publisher(&topic_info, qos)?;

        // Declare publisher liveliness token for ROS 2 discovery
        let pub_keyexpr =
            Ros2Liveliness::publisher_keyexpr(self.domain_id, &self.zid, &self.name, &topic_info);
        #[cfg(feature = "log")]
        log::debug!("Publisher liveliness keyexpr: {}", pub_keyexpr);
        if let Ok(token) = self.session.declare_liveliness(&pub_keyexpr) {
            // Silently ignore if token storage is full (MAX_TOKENS exceeded)
            let _ = self._entity_tokens.push(token);
        }

        Ok(ConnectedPublisher {
            publisher,
            _marker: core::marker::PhantomData,
        })
    }

    /// Create a subscriber for the given topic
    ///
    /// Uses the default receive buffer size (1024 bytes).
    /// For larger messages, use `create_subscriber_sized`.
    pub fn create_subscriber<M: RosMessage>(
        &mut self,
        topic: &str,
    ) -> Result<ConnectedSubscriber<M, DEFAULT_RX_BUFFER_SIZE>, ConnectedNodeError> {
        self.create_subscriber_sized::<M, DEFAULT_RX_BUFFER_SIZE>(topic, QosSettings::BEST_EFFORT)
    }

    /// Create a subscriber with custom QoS settings
    ///
    /// Uses the default receive buffer size (1024 bytes).
    /// For larger messages, use `create_subscriber_sized`.
    pub fn create_subscriber_with_qos<M: RosMessage>(
        &mut self,
        topic: &str,
        qos: QosSettings,
    ) -> Result<ConnectedSubscriber<M, DEFAULT_RX_BUFFER_SIZE>, ConnectedNodeError> {
        self.create_subscriber_sized::<M, DEFAULT_RX_BUFFER_SIZE>(topic, qos)
    }

    /// Create a subscriber with custom buffer size
    ///
    /// # Type Parameters
    ///
    /// - `M`: The ROS message type
    /// - `RX_BUF`: Receive buffer size in bytes
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Create subscriber with 4KB buffer for large messages
    /// let sub = node.create_subscriber_sized::<LargeMsg, 4096>("/large_topic", QosSettings::BEST_EFFORT)?;
    /// ```
    pub fn create_subscriber_sized<M: RosMessage, const RX_BUF: usize>(
        &mut self,
        topic: &str,
        qos: QosSettings,
    ) -> Result<ConnectedSubscriber<M, RX_BUF>, ConnectedNodeError> {
        let topic_info =
            TopicInfo::new(topic, M::TYPE_NAME, M::TYPE_HASH).with_domain(self.domain_id);

        let subscriber = self.session.create_subscriber(&topic_info, qos)?;

        // Declare subscriber liveliness token for ROS 2 discovery
        let sub_keyexpr =
            Ros2Liveliness::subscriber_keyexpr(self.domain_id, &self.zid, &self.name, &topic_info);
        if let Ok(token) = self.session.declare_liveliness(&sub_keyexpr) {
            // Silently ignore if token storage is full (MAX_TOKENS exceeded)
            let _ = self._entity_tokens.push(token);
        }

        Ok(ConnectedSubscriber {
            subscriber,
            rx_buffer: [0u8; RX_BUF],
            _marker: core::marker::PhantomData,
        })
    }

    /// Get the session's Zenoh ID
    pub fn zid(&self) -> &ZenohId {
        &self.zid
    }

    /// Create a service server for the given service
    ///
    /// # Arguments
    /// * `service_name` - The service name (e.g., "/add_two_ints")
    pub fn create_service<S: RosService>(
        &mut self,
        service_name: &str,
    ) -> Result<
        ConnectedServiceServer<S, DEFAULT_REQ_BUFFER_SIZE, DEFAULT_REPLY_BUFFER_SIZE>,
        ConnectedNodeError,
    > {
        self.create_service_sized::<S, DEFAULT_REQ_BUFFER_SIZE, DEFAULT_REPLY_BUFFER_SIZE>(
            service_name,
        )
    }

    /// Create a service server with custom buffer sizes
    ///
    /// # Type Parameters
    ///
    /// - `S`: The ROS service type
    /// - `REQ_BUF`: Request buffer size in bytes
    /// - `REPLY_BUF`: Reply buffer size in bytes
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Create service with 4KB buffers for large messages
    /// let server = node.create_service_sized::<MyService, 4096, 4096>("/my_service")?;
    /// ```
    pub fn create_service_sized<S: RosService, const REQ_BUF: usize, const REPLY_BUF: usize>(
        &mut self,
        service_name: &str,
    ) -> Result<ConnectedServiceServer<S, REQ_BUF, REPLY_BUF>, ConnectedNodeError> {
        let service_info = ServiceInfo::new(service_name, S::SERVICE_NAME, S::SERVICE_HASH)
            .with_domain(self.domain_id);

        let server = self.session.create_service_server(&service_info)?;

        Ok(ConnectedServiceServer {
            server,
            req_buffer: [0u8; REQ_BUF],
            reply_buffer: [0u8; REPLY_BUF],
            _marker: core::marker::PhantomData,
        })
    }

    /// Create a service client for the given service
    ///
    /// # Arguments
    /// * `service_name` - The service name (e.g., "/add_two_ints")
    pub fn create_client<S: RosService>(
        &mut self,
        service_name: &str,
    ) -> Result<
        ConnectedServiceClient<S, DEFAULT_REQ_BUFFER_SIZE, DEFAULT_REPLY_BUFFER_SIZE>,
        ConnectedNodeError,
    > {
        self.create_client_sized::<S, DEFAULT_REQ_BUFFER_SIZE, DEFAULT_REPLY_BUFFER_SIZE>(
            service_name,
        )
    }

    /// Create a service client with custom buffer sizes
    ///
    /// # Type Parameters
    ///
    /// - `S`: The ROS service type
    /// - `REQ_BUF`: Request buffer size in bytes
    /// - `REPLY_BUF`: Reply buffer size in bytes
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Create client with 4KB buffers for large messages
    /// let client = node.create_client_sized::<MyService, 4096, 4096>("/my_service")?;
    /// ```
    pub fn create_client_sized<S: RosService, const REQ_BUF: usize, const REPLY_BUF: usize>(
        &mut self,
        service_name: &str,
    ) -> Result<ConnectedServiceClient<S, REQ_BUF, REPLY_BUF>, ConnectedNodeError> {
        let service_info = ServiceInfo::new(service_name, S::SERVICE_NAME, S::SERVICE_HASH)
            .with_domain(self.domain_id);

        let client = self.session.create_service_client(&service_info)?;

        Ok(ConnectedServiceClient {
            client,
            req_buffer: [0u8; REQ_BUF],
            reply_buffer: [0u8; REPLY_BUF],
            _marker: core::marker::PhantomData,
        })
    }

    /// Create an action server for the given action
    ///
    /// # Arguments
    /// * `action_name` - The action name (e.g., "/fibonacci")
    pub fn create_action_server<A: RosAction>(
        &mut self,
        action_name: &str,
    ) -> Result<
        ConnectedActionServer<
            A,
            DEFAULT_GOAL_BUFFER_SIZE,
            DEFAULT_RESULT_BUFFER_SIZE,
            DEFAULT_FEEDBACK_BUFFER_SIZE,
            DEFAULT_MAX_ACTIVE_GOALS,
        >,
        ConnectedNodeError,
    > {
        self.create_action_server_sized::<
            A,
            DEFAULT_GOAL_BUFFER_SIZE,
            DEFAULT_RESULT_BUFFER_SIZE,
            DEFAULT_FEEDBACK_BUFFER_SIZE,
            DEFAULT_MAX_ACTIVE_GOALS,
        >(action_name)
    }

    /// Create an action server with custom buffer sizes
    ///
    /// # Type Parameters
    ///
    /// - `A`: The ROS action type
    /// - `GOAL_BUF`: Goal buffer size in bytes
    /// - `RESULT_BUF`: Result buffer size in bytes
    /// - `FEEDBACK_BUF`: Feedback buffer size in bytes
    /// - `MAX_GOALS`: Maximum number of concurrent active goals
    pub fn create_action_server_sized<
        A: RosAction,
        const GOAL_BUF: usize,
        const RESULT_BUF: usize,
        const FEEDBACK_BUF: usize,
        const MAX_GOALS: usize,
    >(
        &mut self,
        action_name: &str,
    ) -> Result<
        ConnectedActionServer<A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>,
        ConnectedNodeError,
    > {
        let action_info = ActionInfo::new(action_name, A::ACTION_NAME, A::ACTION_HASH)
            .with_domain(self.domain_id);

        // Create send_goal service server
        let send_goal_keyexpr: heapless::String<256> = action_info.send_goal_key();
        let send_goal_info =
            ServiceInfo::new(&send_goal_keyexpr, A::ACTION_NAME, A::ACTION_HASH).with_domain(0); // Domain already in key
        let send_goal_server = self
            .session
            .create_service_server(&send_goal_info)
            .map_err(|_| ConnectedNodeError::ActionServerCreationFailed)?;

        // Create feedback publisher
        let feedback_keyexpr: heapless::String<256> = action_info.feedback_key();
        let feedback_topic =
            TopicInfo::new(&feedback_keyexpr, A::ACTION_NAME, A::ACTION_HASH).with_domain(0);
        let feedback_publisher = self
            .session
            .create_publisher(&feedback_topic, QosSettings::BEST_EFFORT)
            .map_err(|_| ConnectedNodeError::ActionServerCreationFailed)?;

        Ok(ConnectedActionServer {
            send_goal_server,
            feedback_publisher,
            active_goals: heapless::Vec::new(),
            goal_buffer: [0u8; GOAL_BUF],
            result_buffer: [0u8; RESULT_BUF],
            feedback_buffer: [0u8; FEEDBACK_BUF],
            goal_counter: 0,
        })
    }

    /// Create an action client for the given action
    ///
    /// # Arguments
    /// * `action_name` - The action name (e.g., "/fibonacci")
    pub fn create_action_client<A: RosAction>(
        &mut self,
        action_name: &str,
    ) -> Result<
        ConnectedActionClient<
            A,
            DEFAULT_GOAL_BUFFER_SIZE,
            DEFAULT_RESULT_BUFFER_SIZE,
            DEFAULT_FEEDBACK_BUFFER_SIZE,
        >,
        ConnectedNodeError,
    > {
        self.create_action_client_sized::<
            A,
            DEFAULT_GOAL_BUFFER_SIZE,
            DEFAULT_RESULT_BUFFER_SIZE,
            DEFAULT_FEEDBACK_BUFFER_SIZE,
        >(action_name)
    }

    /// Create an action client with custom buffer sizes
    ///
    /// # Type Parameters
    ///
    /// - `A`: The ROS action type
    /// - `GOAL_BUF`: Goal buffer size in bytes
    /// - `RESULT_BUF`: Result buffer size in bytes
    /// - `FEEDBACK_BUF`: Feedback buffer size in bytes
    pub fn create_action_client_sized<
        A: RosAction,
        const GOAL_BUF: usize,
        const RESULT_BUF: usize,
        const FEEDBACK_BUF: usize,
    >(
        &mut self,
        action_name: &str,
    ) -> Result<ConnectedActionClient<A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>, ConnectedNodeError>
    {
        let action_info = ActionInfo::new(action_name, A::ACTION_NAME, A::ACTION_HASH)
            .with_domain(self.domain_id);

        // Create send_goal service client
        let send_goal_keyexpr: heapless::String<256> = action_info.send_goal_key();
        let send_goal_info =
            ServiceInfo::new(&send_goal_keyexpr, A::ACTION_NAME, A::ACTION_HASH).with_domain(0);
        let send_goal_client = self
            .session
            .create_service_client(&send_goal_info)
            .map_err(|_| ConnectedNodeError::ActionClientCreationFailed)?;

        // Create feedback subscriber
        let feedback_keyexpr: heapless::String<256> = action_info.feedback_key();
        let feedback_topic =
            TopicInfo::new(&feedback_keyexpr, A::ACTION_NAME, A::ACTION_HASH).with_domain(0);
        let feedback_subscriber = self
            .session
            .create_subscriber(&feedback_topic, QosSettings::BEST_EFFORT)
            .map_err(|_| ConnectedNodeError::ActionClientCreationFailed)?;

        Ok(ConnectedActionClient {
            send_goal_client,
            feedback_subscriber,
            goal_buffer: [0u8; GOAL_BUF],
            result_buffer: [0u8; RESULT_BUF],
            feedback_buffer: [0u8; FEEDBACK_BUF],
            goal_counter: 0,
            _marker: core::marker::PhantomData,
        })
    }
}

/// A connected publisher that can send messages via transport
#[cfg(feature = "zenoh")]
pub struct ConnectedPublisher<M> {
    publisher: ZenohPublisher,
    _marker: core::marker::PhantomData<M>,
}

#[cfg(feature = "zenoh")]
impl<M: RosMessage> ConnectedPublisher<M> {
    /// Publish a message
    ///
    /// Serializes the message and sends it via the transport.
    pub fn publish(&self, msg: &M) -> Result<(), ConnectedNodeError> {
        let mut buf = [0u8; 1024];
        self.publisher
            .publish(msg, &mut buf)
            .map_err(|_| ConnectedNodeError::PublishFailed)
    }

    /// Publish with a custom buffer
    ///
    /// Use this when you need a larger buffer for big messages.
    pub fn publish_with_buffer(&self, msg: &M, buf: &mut [u8]) -> Result<(), ConnectedNodeError> {
        self.publisher
            .publish(msg, buf)
            .map_err(|_| ConnectedNodeError::PublishFailed)
    }

    /// Publish raw bytes (already serialized)
    pub fn publish_raw(&self, data: &[u8]) -> Result<(), ConnectedNodeError> {
        self.publisher
            .publish_raw(data)
            .map_err(|_| ConnectedNodeError::PublishFailed)
    }
}

/// A connected subscriber that can receive messages via transport
///
/// # Type Parameters
///
/// - `M`: The ROS message type to receive
/// - `RX_BUF`: Receive buffer size in bytes (default: 1024)
#[cfg(feature = "zenoh")]
pub struct ConnectedSubscriber<M, const RX_BUF: usize = DEFAULT_RX_BUFFER_SIZE> {
    subscriber: ZenohSubscriber,
    rx_buffer: [u8; RX_BUF],
    _marker: core::marker::PhantomData<M>,
}

#[cfg(feature = "zenoh")]
impl<M: RosMessage, const RX_BUF: usize> ConnectedSubscriber<M, RX_BUF> {
    /// Try to receive a message (non-blocking)
    ///
    /// Returns `Ok(Some(msg))` if a message was received,
    /// `Ok(None)` if no message is available,
    /// or `Err` on deserialization failure.
    pub fn try_recv(&mut self) -> Result<Option<M>, ConnectedNodeError> {
        self.subscriber
            .try_recv::<M>(&mut self.rx_buffer)
            .map_err(|_| ConnectedNodeError::DeserializationFailed)
    }

    /// Try to receive raw bytes (non-blocking)
    ///
    /// Returns the number of bytes received, or None if no message available.
    pub fn try_recv_raw(&mut self, buf: &mut [u8]) -> Result<Option<usize>, ConnectedNodeError> {
        self.subscriber
            .try_recv_raw(buf)
            .map_err(|_| ConnectedNodeError::DeserializationFailed)
    }

    /// Get the buffer size
    pub const fn buffer_size(&self) -> usize {
        RX_BUF
    }
}

/// A connected service server that can handle service requests
///
/// # Type Parameters
///
/// - `S`: The ROS service type
/// - `REQ_BUF`: Request buffer size in bytes (default: 1024)
/// - `REPLY_BUF`: Reply buffer size in bytes (default: 1024)
#[cfg(feature = "zenoh")]
pub struct ConnectedServiceServer<
    S: RosService,
    const REQ_BUF: usize = DEFAULT_REQ_BUFFER_SIZE,
    const REPLY_BUF: usize = DEFAULT_REPLY_BUFFER_SIZE,
> {
    server: ZenohServiceServer,
    req_buffer: [u8; REQ_BUF],
    reply_buffer: [u8; REPLY_BUF],
    _marker: core::marker::PhantomData<S>,
}

#[cfg(feature = "zenoh")]
impl<S: RosService, const REQ_BUF: usize, const REPLY_BUF: usize>
    ConnectedServiceServer<S, REQ_BUF, REPLY_BUF>
{
    /// Handle a single service request if one is available
    ///
    /// Returns `Ok(true)` if a request was handled, `Ok(false)` if no request was available.
    pub fn handle_request(
        &mut self,
        handler: impl FnOnce(&S::Request) -> S::Reply,
    ) -> Result<bool, ConnectedNodeError> {
        self.server
            .handle_request::<S>(&mut self.req_buffer, &mut self.reply_buffer, handler)
            .map_err(ConnectedNodeError::from)
    }

    /// Try to receive a raw service request (non-blocking)
    ///
    /// Returns the request data and sequence number if available.
    pub fn try_recv_request(&mut self) -> Result<Option<(usize, i64)>, ConnectedNodeError> {
        match self.server.try_recv_request(&mut self.req_buffer)? {
            Some(request) => Ok(Some((request.data.len(), request.sequence_number))),
            None => Ok(None),
        }
    }

    /// Get the request buffer size
    pub const fn request_buffer_size(&self) -> usize {
        REQ_BUF
    }

    /// Get the reply buffer size
    pub const fn reply_buffer_size(&self) -> usize {
        REPLY_BUF
    }
}

/// A connected service client that can send service requests
///
/// # Type Parameters
///
/// - `S`: The ROS service type
/// - `REQ_BUF`: Request buffer size in bytes (default: 1024)
/// - `REPLY_BUF`: Reply buffer size in bytes (default: 1024)
#[cfg(feature = "zenoh")]
pub struct ConnectedServiceClient<
    S: RosService,
    const REQ_BUF: usize = DEFAULT_REQ_BUFFER_SIZE,
    const REPLY_BUF: usize = DEFAULT_REPLY_BUFFER_SIZE,
> {
    client: ZenohServiceClient,
    req_buffer: [u8; REQ_BUF],
    reply_buffer: [u8; REPLY_BUF],
    _marker: core::marker::PhantomData<S>,
}

#[cfg(feature = "zenoh")]
impl<S: RosService, const REQ_BUF: usize, const REPLY_BUF: usize>
    ConnectedServiceClient<S, REQ_BUF, REPLY_BUF>
{
    /// Call the service with a request
    ///
    /// Blocks until a reply is received or an error occurs.
    pub fn call(&mut self, request: &S::Request) -> Result<S::Reply, ConnectedNodeError> {
        self.client
            .call::<S>(request, &mut self.req_buffer, &mut self.reply_buffer)
            .map_err(ConnectedNodeError::from)
    }

    /// Call the service with raw data
    ///
    /// Returns the number of bytes received in the reply.
    pub fn call_raw(&mut self, request: &[u8]) -> Result<usize, ConnectedNodeError> {
        self.client
            .call_raw(request, &mut self.reply_buffer)
            .map_err(ConnectedNodeError::from)
    }

    /// Get the request buffer size
    pub const fn request_buffer_size(&self) -> usize {
        REQ_BUF
    }

    /// Get the reply buffer size
    pub const fn reply_buffer_size(&self) -> usize {
        REPLY_BUF
    }
}

/// Maximum number of active goals for an action server
pub const DEFAULT_MAX_ACTIVE_GOALS: usize = 8;

/// An active goal being processed by an action server
#[derive(Clone)]
pub struct ActiveGoal<A: RosAction> {
    /// Goal ID
    pub goal_id: GoalId,
    /// Current status
    pub status: GoalStatus,
    /// The goal data
    pub goal: A::Goal,
}

/// A connected action server that handles action goals
///
/// # Type Parameters
///
/// - `A`: The ROS action type
/// - `GOAL_BUF`: Goal message buffer size in bytes (default: 1024)
/// - `RESULT_BUF`: Result message buffer size in bytes (default: 1024)
/// - `FEEDBACK_BUF`: Feedback message buffer size in bytes (default: 1024)
/// - `MAX_GOALS`: Maximum number of concurrent active goals (default: 8)
#[cfg(feature = "zenoh")]
pub struct ConnectedActionServer<
    A: RosAction,
    const GOAL_BUF: usize = DEFAULT_GOAL_BUFFER_SIZE,
    const RESULT_BUF: usize = DEFAULT_RESULT_BUFFER_SIZE,
    const FEEDBACK_BUF: usize = DEFAULT_FEEDBACK_BUFFER_SIZE,
    const MAX_GOALS: usize = DEFAULT_MAX_ACTIVE_GOALS,
> {
    /// Service server for send_goal
    send_goal_server: ZenohServiceServer,
    /// Publisher for feedback
    feedback_publisher: ZenohPublisher,
    /// Active goals being processed
    active_goals: heapless::Vec<ActiveGoal<A>, MAX_GOALS>,
    /// Buffers for serialization
    goal_buffer: [u8; GOAL_BUF],
    result_buffer: [u8; RESULT_BUF],
    feedback_buffer: [u8; FEEDBACK_BUF],
    /// Goal counter for generating IDs (embedded-friendly)
    goal_counter: u64,
}

#[cfg(feature = "zenoh")]
impl<
        A: RosAction,
        const GOAL_BUF: usize,
        const RESULT_BUF: usize,
        const FEEDBACK_BUF: usize,
        const MAX_GOALS: usize,
    > ConnectedActionServer<A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>
{
    /// Try to receive and handle a goal request
    ///
    /// Returns `Ok(Some(goal_id))` if a goal was accepted, `Ok(None)` if no goal request was available.
    /// The goal_handler function receives the goal and returns whether to accept it.
    pub fn try_accept_goal(
        &mut self,
        goal_handler: impl FnOnce(&A::Goal) -> GoalResponse,
    ) -> Result<Option<GoalId>, ConnectedNodeError> {
        use nano_ros_core::CdrReader;

        // Try to receive a goal request
        let request = match self
            .send_goal_server
            .try_recv_request(&mut self.goal_buffer)?
        {
            Some(req) => req,
            None => return Ok(None),
        };

        let data_len = request.data.len();
        let sequence_number = request.sequence_number;

        // Deserialize goal (skip UUID header - first 16 bytes)
        // ROS 2 action SendGoal request format: UUID (16 bytes) + Goal message
        if data_len < 16 {
            return Err(ConnectedNodeError::DeserializationFailed);
        }

        let mut reader = CdrReader::new_with_header(&self.goal_buffer[..data_len])
            .map_err(|_| ConnectedNodeError::DeserializationFailed)?;

        // Read UUID (16 bytes) - we'll generate our own for embedded
        let mut _uuid = [0u8; 16];
        for byte in &mut _uuid {
            *byte = reader
                .read_u8()
                .map_err(|_| ConnectedNodeError::DeserializationFailed)?;
        }

        // Deserialize the goal
        let goal = A::Goal::deserialize(&mut reader)
            .map_err(|_| ConnectedNodeError::DeserializationFailed)?;

        // Call handler to decide whether to accept
        let response = goal_handler(&goal);

        // Generate response
        let goal_id = if response.is_accepted() {
            // Generate new goal ID
            self.goal_counter += 1;
            let new_goal_id = GoalId::from_counter(self.goal_counter);

            // Try to add to active goals
            if self.active_goals.len() >= MAX_GOALS {
                // Server is full, reject
                self.send_goal_response(sequence_number, GoalResponse::Reject, &GoalId::zero())?;
                return Err(ConnectedNodeError::ActionServerFull);
            }

            let active_goal = ActiveGoal {
                goal_id: new_goal_id,
                status: GoalStatus::Accepted,
                goal,
            };
            self.active_goals
                .push(active_goal)
                .map_err(|_| ConnectedNodeError::ActionServerFull)?;

            self.send_goal_response(sequence_number, response, &new_goal_id)?;
            Some(new_goal_id)
        } else {
            self.send_goal_response(sequence_number, response, &GoalId::zero())?;
            None
        };

        Ok(goal_id)
    }

    /// Send a goal response
    fn send_goal_response(
        &mut self,
        sequence_number: i64,
        response: GoalResponse,
        goal_id: &GoalId,
    ) -> Result<(), ConnectedNodeError> {
        use nano_ros_core::CdrWriter;

        // Serialize response: accepted (bool) + timestamp (sec, nsec)
        let mut writer = CdrWriter::new_with_header(&mut self.result_buffer)
            .map_err(|_| ConnectedNodeError::BufferTooSmall)?;

        // Write accepted flag
        writer
            .write_bool(response.is_accepted())
            .map_err(|_| ConnectedNodeError::SerializationFailed)?;

        // Write timestamp (placeholder - 0,0)
        writer
            .write_i32(0)
            .map_err(|_| ConnectedNodeError::SerializationFailed)?;
        writer
            .write_u32(0)
            .map_err(|_| ConnectedNodeError::SerializationFailed)?;

        // Write goal ID for identification
        for byte in &goal_id.uuid {
            writer
                .write_u8(*byte)
                .map_err(|_| ConnectedNodeError::SerializationFailed)?;
        }

        let len = writer.position();
        self.send_goal_server
            .send_reply(sequence_number, &self.result_buffer[..len])?;

        Ok(())
    }

    /// Publish feedback for an active goal
    pub fn publish_feedback(
        &mut self,
        goal_id: &GoalId,
        feedback: &A::Feedback,
    ) -> Result<(), ConnectedNodeError> {
        use nano_ros_core::CdrWriter;

        // Find the goal
        let goal_exists = self.active_goals.iter().any(|g| &g.goal_id == goal_id);
        if !goal_exists {
            return Err(ConnectedNodeError::GoalNotFound);
        }

        // Serialize feedback message
        // Format: UUID (16 bytes) + Feedback
        let mut writer = CdrWriter::new_with_header(&mut self.feedback_buffer)
            .map_err(|_| ConnectedNodeError::BufferTooSmall)?;

        // Write goal ID
        for byte in &goal_id.uuid {
            writer
                .write_u8(*byte)
                .map_err(|_| ConnectedNodeError::SerializationFailed)?;
        }

        // Write feedback
        feedback
            .serialize(&mut writer)
            .map_err(|_| ConnectedNodeError::SerializationFailed)?;

        let len = writer.position();
        self.feedback_publisher
            .publish_raw(&self.feedback_buffer[..len])
            .map_err(|_| ConnectedNodeError::PublishFailed)?;

        Ok(())
    }

    /// Update the status of an active goal
    pub fn set_goal_status(
        &mut self,
        goal_id: &GoalId,
        status: GoalStatus,
    ) -> Result<(), ConnectedNodeError> {
        for goal in &mut self.active_goals {
            if &goal.goal_id == goal_id {
                goal.status = status;
                return Ok(());
            }
        }
        Err(ConnectedNodeError::GoalNotFound)
    }

    /// Complete a goal with a result
    ///
    /// This removes the goal from active goals.
    pub fn complete_goal(
        &mut self,
        goal_id: &GoalId,
        status: GoalStatus,
        _result: &A::Result,
    ) -> Result<(), ConnectedNodeError> {
        // Find and remove the goal
        let index = self
            .active_goals
            .iter()
            .position(|g| &g.goal_id == goal_id)
            .ok_or(ConnectedNodeError::GoalNotFound)?;

        // Update status before removing
        self.active_goals[index].status = status;

        // Remove from active goals
        self.active_goals.swap_remove(index);

        Ok(())
    }

    /// Get an active goal by ID
    pub fn get_goal(&self, goal_id: &GoalId) -> Option<&ActiveGoal<A>> {
        self.active_goals.iter().find(|g| &g.goal_id == goal_id)
    }

    /// Get all active goals
    pub fn active_goals(&self) -> &[ActiveGoal<A>] {
        &self.active_goals
    }

    /// Get the number of active goals
    pub fn active_goal_count(&self) -> usize {
        self.active_goals.len()
    }
}

/// A goal handle for tracking a sent goal on the client side
#[derive(Clone)]
pub struct GoalHandle {
    /// Goal ID
    pub goal_id: GoalId,
    /// Whether the goal was accepted
    pub accepted: bool,
}

/// A connected action client that can send goals and receive feedback
///
/// # Type Parameters
///
/// - `A`: The ROS action type
/// - `GOAL_BUF`: Goal message buffer size in bytes (default: 1024)
/// - `RESULT_BUF`: Result message buffer size in bytes (default: 1024)
/// - `FEEDBACK_BUF`: Feedback message buffer size in bytes (default: 1024)
#[cfg(feature = "zenoh")]
pub struct ConnectedActionClient<
    A: RosAction,
    const GOAL_BUF: usize = DEFAULT_GOAL_BUFFER_SIZE,
    const RESULT_BUF: usize = DEFAULT_RESULT_BUFFER_SIZE,
    const FEEDBACK_BUF: usize = DEFAULT_FEEDBACK_BUFFER_SIZE,
> {
    /// Service client for send_goal
    send_goal_client: ZenohServiceClient,
    /// Subscriber for feedback
    feedback_subscriber: ZenohSubscriber,
    /// Buffers for serialization
    goal_buffer: [u8; GOAL_BUF],
    result_buffer: [u8; RESULT_BUF],
    feedback_buffer: [u8; FEEDBACK_BUF],
    /// Goal counter for generating IDs
    goal_counter: u64,
    /// Marker for action type
    _marker: core::marker::PhantomData<A>,
}

#[cfg(feature = "zenoh")]
impl<A: RosAction, const GOAL_BUF: usize, const RESULT_BUF: usize, const FEEDBACK_BUF: usize>
    ConnectedActionClient<A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>
{
    /// Send a goal to the action server
    ///
    /// Returns a GoalHandle if the goal was accepted by the server.
    pub fn send_goal(&mut self, goal: &A::Goal) -> Result<GoalHandle, ConnectedNodeError> {
        use nano_ros_core::{CdrReader, CdrWriter};

        // Generate goal ID
        self.goal_counter += 1;
        let goal_id = GoalId::from_counter(self.goal_counter);

        // Serialize goal request
        // Format: UUID (16 bytes) + Goal
        let mut writer = CdrWriter::new_with_header(&mut self.goal_buffer)
            .map_err(|_| ConnectedNodeError::BufferTooSmall)?;

        // Write goal ID
        for byte in &goal_id.uuid {
            writer
                .write_u8(*byte)
                .map_err(|_| ConnectedNodeError::SerializationFailed)?;
        }

        // Write goal
        goal.serialize(&mut writer)
            .map_err(|_| ConnectedNodeError::SerializationFailed)?;

        let req_len = writer.position();

        // Send goal and wait for response
        let reply_len = self
            .send_goal_client
            .call_raw(&self.goal_buffer[..req_len], &mut self.result_buffer)
            .map_err(ConnectedNodeError::from)?;

        // Parse response
        let mut reader = CdrReader::new_with_header(&self.result_buffer[..reply_len])
            .map_err(|_| ConnectedNodeError::DeserializationFailed)?;

        // Read accepted flag
        let accepted = reader
            .read_bool()
            .map_err(|_| ConnectedNodeError::DeserializationFailed)?;

        Ok(GoalHandle { goal_id, accepted })
    }

    /// Try to receive feedback (non-blocking)
    ///
    /// Returns the goal ID and feedback if available.
    pub fn try_recv_feedback(
        &mut self,
    ) -> Result<Option<(GoalId, A::Feedback)>, ConnectedNodeError> {
        use nano_ros_core::CdrReader;

        // Try to receive raw feedback
        let data_len = match self
            .feedback_subscriber
            .try_recv_raw(&mut self.feedback_buffer)
        {
            Ok(Some(len)) => len,
            Ok(None) => return Ok(None),
            Err(_) => return Err(ConnectedNodeError::DeserializationFailed),
        };

        // Parse feedback
        // Format: UUID (16 bytes) + Feedback
        let mut reader = CdrReader::new_with_header(&self.feedback_buffer[..data_len])
            .map_err(|_| ConnectedNodeError::DeserializationFailed)?;

        // Read goal ID
        let mut uuid = [0u8; 16];
        for byte in &mut uuid {
            *byte = reader
                .read_u8()
                .map_err(|_| ConnectedNodeError::DeserializationFailed)?;
        }
        let goal_id = GoalId::new(uuid);

        // Read feedback
        let feedback = A::Feedback::deserialize(&mut reader)
            .map_err(|_| ConnectedNodeError::DeserializationFailed)?;

        Ok(Some((goal_id, feedback)))
    }

    /// Get the goal buffer size
    pub const fn goal_buffer_size(&self) -> usize {
        GOAL_BUF
    }

    /// Get the result buffer size
    pub const fn result_buffer_size(&self) -> usize {
        RESULT_BUF
    }

    /// Get the feedback buffer size
    pub const fn feedback_buffer_size(&self) -> usize {
        FEEDBACK_BUF
    }
}

#[cfg(all(test, feature = "zenoh"))]
mod tests {
    // Tests require a running zenoh router or peer mode
    // They are in the integration test file
}
