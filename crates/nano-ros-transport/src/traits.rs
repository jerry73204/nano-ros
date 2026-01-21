//! Transport abstraction traits
//!
//! These traits define the interface for transport backends (zenoh-pico, etc.)

use nano_ros_core::{Deserialize, RosMessage, RosService, Serialize};

/// Topic information for pub/sub
#[derive(Debug, Clone)]
pub struct TopicInfo<'a> {
    /// Topic name (e.g., "/chatter")
    pub name: &'a str,
    /// ROS type name (e.g., "std_msgs::msg::dds_::String_")
    pub type_name: &'a str,
    /// Type hash for compatibility checking
    pub type_hash: &'a str,
    /// Domain ID (default: 0)
    pub domain_id: u32,
}

impl<'a> TopicInfo<'a> {
    /// Create new topic info
    pub const fn new(name: &'a str, type_name: &'a str, type_hash: &'a str) -> Self {
        Self {
            name,
            type_name,
            type_hash,
            domain_id: 0,
        }
    }

    /// Create topic info with custom domain ID
    pub const fn with_domain(mut self, domain_id: u32) -> Self {
        self.domain_id = domain_id;
        self
    }

    /// Generate the full topic key in rmw_zenoh format
    /// Format: `<domain_id>/<topic_name>/<type_name>/<hash>`
    /// Note: Leading/trailing slashes are stripped from topic name
    /// Note: For Humble, the hash is `TypeHashNotSupported` (no RIHS01_ prefix)
    /// Note: For newer versions (Iron+), use `RIHS01_<hash>` format
    pub fn to_key<const N: usize>(&self) -> heapless::String<N> {
        let mut key = heapless::String::new();
        // Strip leading/trailing slashes from topic name (matching rmw_zenoh's strip_slashes)
        let topic_stripped = self.name.trim_matches('/');
        // Best effort - ignore errors if key is too long
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/{}/{}",
                self.domain_id, topic_stripped, self.type_name, self.type_hash
            ),
        );
        key
    }

    /// Generate a wildcard topic key for subscribing
    /// Format: `<domain_id>/<topic_name>/<type_name>/*`
    /// This matches any type hash, allowing interop with ROS 2 nodes using different hashes
    pub fn to_key_wildcard<const N: usize>(&self) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let topic_stripped = self.name.trim_matches('/');
        let _ = core::fmt::write(
            &mut key,
            format_args!("{}/{}/{}/*", self.domain_id, topic_stripped, self.type_name),
        );
        key
    }
}

/// Service information for service client/server
#[derive(Debug, Clone)]
pub struct ServiceInfo<'a> {
    /// Service name (e.g., "/add_two_ints")
    pub name: &'a str,
    /// ROS service type name (e.g., "example_interfaces::srv::dds_::AddTwoInts_")
    pub type_name: &'a str,
    /// Type hash for compatibility checking
    pub type_hash: &'a str,
    /// Domain ID (default: 0)
    pub domain_id: u32,
}

/// Action information for action client/server
///
/// Actions in ROS 2 use 5 communication channels:
/// - `send_goal` service: `<action_name>/_action/send_goal`
/// - `cancel_goal` service: `<action_name>/_action/cancel_goal`
/// - `get_result` service: `<action_name>/_action/get_result`
/// - `feedback` topic: `<action_name>/_action/feedback`
/// - `status` topic: `<action_name>/_action/status`
#[derive(Debug, Clone)]
pub struct ActionInfo<'a> {
    /// Action name (e.g., "/fibonacci")
    pub name: &'a str,
    /// ROS action type name (e.g., "example_interfaces::action::dds_::Fibonacci_")
    pub type_name: &'a str,
    /// Type hash for compatibility checking
    pub type_hash: &'a str,
    /// Domain ID (default: 0)
    pub domain_id: u32,
}

impl<'a> ActionInfo<'a> {
    /// Create new action info
    pub const fn new(name: &'a str, type_name: &'a str, type_hash: &'a str) -> Self {
        Self {
            name,
            type_name,
            type_hash,
            domain_id: 0,
        }
    }

    /// Create action info with custom domain ID
    pub const fn with_domain(mut self, domain_id: u32) -> Self {
        self.domain_id = domain_id;
        self
    }

    /// Generate the send_goal service key
    pub fn send_goal_key<const N: usize>(&self) -> heapless::String<N> {
        self.service_key::<N>("SendGoal")
    }

    /// Generate the cancel_goal service key
    pub fn cancel_goal_key<const N: usize>(&self) -> heapless::String<N> {
        self.service_key::<N>("CancelGoal")
    }

    /// Generate the get_result service key
    pub fn get_result_key<const N: usize>(&self) -> heapless::String<N> {
        self.service_key::<N>("GetResult")
    }

    /// Generate the feedback topic key
    pub fn feedback_key<const N: usize>(&self) -> heapless::String<N> {
        self.topic_key::<N>("FeedbackMessage")
    }

    /// Generate the status topic key
    pub fn status_key<const N: usize>(&self) -> heapless::String<N> {
        self.topic_key::<N>("GoalStatusArray")
    }

    /// Generate a service key for an action sub-service
    fn service_key<const N: usize>(&self, suffix: &str) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let action_stripped = self.name.trim_matches('/');
        // Format: <domain>/<action>/_action/<service_type>/RIHS01_<hash>
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/_action/{}_{}/RIHS01_{}",
                self.domain_id, action_stripped, self.type_name, suffix, self.type_hash
            ),
        );
        key
    }

    /// Generate a topic key for an action sub-topic
    fn topic_key<const N: usize>(&self, suffix: &str) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let action_stripped = self.name.trim_matches('/');
        // Format: <domain>/<action>/_action/<topic_type>/<hash>
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/_action/{}_{}/{}",
                self.domain_id, action_stripped, self.type_name, suffix, self.type_hash
            ),
        );
        key
    }
}

impl<'a> ServiceInfo<'a> {
    /// Create new service info
    pub const fn new(name: &'a str, type_name: &'a str, type_hash: &'a str) -> Self {
        Self {
            name,
            type_name,
            type_hash,
            domain_id: 0,
        }
    }

    /// Create service info with custom domain ID
    pub const fn with_domain(mut self, domain_id: u32) -> Self {
        self.domain_id = domain_id;
        self
    }

    /// Generate the service key in rmw_zenoh format
    /// Format: `<domain_id>/<service_name>/<type_name>/RIHS01_<hash>`
    pub fn to_key<const N: usize>(&self) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let service_stripped = self.name.trim_matches('/');
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/{}/RIHS01_{}",
                self.domain_id, service_stripped, self.type_name, self.type_hash
            ),
        );
        key
    }
}

/// Transport error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportError {
    /// Failed to connect to transport
    ConnectionFailed,
    /// Connection was closed
    Disconnected,
    /// Failed to create publisher
    PublisherCreationFailed,
    /// Failed to create subscriber
    SubscriberCreationFailed,
    /// Failed to create service server
    ServiceServerCreationFailed,
    /// Failed to create service client
    ServiceClientCreationFailed,
    /// Failed to publish message
    PublishFailed,
    /// Failed to send service request
    ServiceRequestFailed,
    /// Failed to send service reply
    ServiceReplyFailed,
    /// Serialization error
    SerializationError,
    /// Deserialization error
    DeserializationError,
    /// Buffer too small
    BufferTooSmall,
    /// Timeout waiting for message
    Timeout,
    /// Invalid configuration
    InvalidConfig,
    /// Failed to start background tasks
    TaskStartFailed,
    /// Failed to poll for incoming messages
    PollFailed,
    /// Failed to send keepalive
    KeepaliveFailed,
    /// Failed to send join message
    JoinFailed,
}

/// QoS (Quality of Service) settings
#[derive(Debug, Clone, Copy, Default)]
pub struct QosSettings {
    /// Reliability: true for reliable, false for best-effort
    pub reliable: bool,
    /// History depth for subscribers (0 = keep last)
    pub history_depth: u8,
}

impl QosSettings {
    /// Best-effort QoS (default for real-time)
    pub const BEST_EFFORT: Self = Self {
        reliable: false,
        history_depth: 1,
    };

    /// Reliable QoS
    pub const RELIABLE: Self = Self {
        reliable: true,
        history_depth: 10,
    };
}

/// Transport session configuration
#[derive(Debug, Clone)]
pub struct TransportConfig<'a> {
    /// Peer locator (e.g., "tcp/192.168.1.1:7447")
    pub locator: Option<&'a str>,
    /// Session mode: client, peer, or router
    pub mode: SessionMode,
}

impl Default for TransportConfig<'_> {
    fn default() -> Self {
        Self {
            locator: None,
            mode: SessionMode::Client,
        }
    }
}

/// Session mode
#[derive(Debug, Clone, Copy, Default)]
pub enum SessionMode {
    /// Connect as client to a router
    #[default]
    Client,
    /// Connect as peer for peer-to-peer communication
    Peer,
}

/// Transport session trait
pub trait Session {
    /// Error type for this session
    type Error;
    /// Publisher handle type
    type PublisherHandle;
    /// Subscriber handle type
    type SubscriberHandle;
    /// Service server handle type
    type ServiceServerHandle;
    /// Service client handle type
    type ServiceClientHandle;

    /// Create a publisher for a topic
    fn create_publisher(
        &mut self,
        topic: &TopicInfo,
        qos: QosSettings,
    ) -> Result<Self::PublisherHandle, Self::Error>;

    /// Create a subscriber for a topic
    fn create_subscriber(
        &mut self,
        topic: &TopicInfo,
        qos: QosSettings,
    ) -> Result<Self::SubscriberHandle, Self::Error>;

    /// Create a service server
    fn create_service_server(
        &mut self,
        service: &ServiceInfo,
    ) -> Result<Self::ServiceServerHandle, Self::Error>;

    /// Create a service client
    fn create_service_client(
        &mut self,
        service: &ServiceInfo,
    ) -> Result<Self::ServiceClientHandle, Self::Error>;

    /// Close the session
    fn close(&mut self) -> Result<(), Self::Error>;
}

/// Publisher trait for sending messages
pub trait Publisher {
    /// Error type for publish operations
    type Error;

    /// Publish a serialized message
    fn publish_raw(&self, data: &[u8]) -> Result<(), Self::Error>;

    /// Publish a typed message (serializes automatically)
    fn publish<M: RosMessage>(&self, msg: &M, buf: &mut [u8]) -> Result<(), Self::Error> {
        use nano_ros_core::CdrWriter;

        let mut writer = CdrWriter::new_with_header(buf).map_err(|_| self.buffer_error())?;
        msg.serialize(&mut writer)
            .map_err(|_| self.serialization_error())?;
        let len = writer.position();
        self.publish_raw(&buf[..len])
    }

    /// Return a buffer-too-small error (implementation specific)
    fn buffer_error(&self) -> Self::Error;

    /// Return a serialization error (implementation specific)
    fn serialization_error(&self) -> Self::Error;
}

/// Subscriber trait for receiving messages
pub trait Subscriber {
    /// Error type for receive operations
    type Error;

    /// Try to receive a raw message (non-blocking)
    /// Returns None if no message is available
    fn try_recv_raw(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Self::Error>;

    /// Try to receive a typed message (non-blocking)
    fn try_recv<M: RosMessage>(&mut self, buf: &mut [u8]) -> Result<Option<M>, Self::Error> {
        use nano_ros_core::CdrReader;

        match self.try_recv_raw(buf)? {
            Some(len) => {
                let mut reader = CdrReader::new_with_header(&buf[..len])
                    .map_err(|_| self.deserialization_error())?;
                let msg = M::deserialize(&mut reader).map_err(|_| self.deserialization_error())?;
                Ok(Some(msg))
            }
            None => Ok(None),
        }
    }

    /// Return a deserialization error (implementation specific)
    fn deserialization_error(&self) -> Self::Error;
}

/// Service request from a client
pub struct ServiceRequest<'a> {
    /// Raw request data (CDR encoded)
    pub data: &'a [u8],
    /// Sequence number for request/response matching
    pub sequence_number: i64,
}

/// Service server trait for handling requests
pub trait ServiceServerTrait {
    /// Error type for service operations
    type Error;

    /// Try to receive a service request (non-blocking)
    /// The returned ServiceRequest references data in the provided buffer
    fn try_recv_request<'a>(
        &mut self,
        buf: &'a mut [u8],
    ) -> Result<Option<ServiceRequest<'a>>, Self::Error>;

    /// Send a reply to a service request
    fn send_reply(&mut self, sequence_number: i64, data: &[u8]) -> Result<(), Self::Error>;

    /// Handle a service request with typed messages
    fn handle_request<S: RosService>(
        &mut self,
        req_buf: &mut [u8],
        reply_buf: &mut [u8],
        handler: impl FnOnce(&S::Request) -> S::Reply,
    ) -> Result<bool, Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        use nano_ros_core::{CdrReader, CdrWriter};

        // First, try to receive a request and extract necessary data
        let (data_len, sequence_number) = match self.try_recv_request(req_buf)? {
            Some(request) => (request.data.len(), request.sequence_number),
            None => return Ok(false),
        };

        // Now we can work with req_buf directly since ServiceRequest has been dropped
        // Deserialize request
        let mut reader = CdrReader::new_with_header(&req_buf[..data_len])
            .map_err(|_| TransportError::DeserializationError)?;
        let req = S::Request::deserialize(&mut reader)
            .map_err(|_| TransportError::DeserializationError)?;

        // Call handler
        let reply = handler(&req);

        // Serialize reply
        let mut writer =
            CdrWriter::new_with_header(reply_buf).map_err(|_| TransportError::BufferTooSmall)?;
        reply
            .serialize(&mut writer)
            .map_err(|_| TransportError::SerializationError)?;
        let len = writer.position();

        // Send reply (now we can borrow self mutably again)
        self.send_reply(sequence_number, &reply_buf[..len])?;
        Ok(true)
    }
}

/// Service client trait for sending requests
pub trait ServiceClientTrait {
    /// Error type for service operations
    type Error;

    /// Send a service request and wait for reply
    fn call_raw(&mut self, request: &[u8], reply_buf: &mut [u8]) -> Result<usize, Self::Error>;

    /// Call a service with typed messages
    fn call<S: RosService>(
        &mut self,
        request: &S::Request,
        req_buf: &mut [u8],
        reply_buf: &mut [u8],
    ) -> Result<S::Reply, Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        use nano_ros_core::{CdrReader, CdrWriter};

        // Serialize request
        let mut writer =
            CdrWriter::new_with_header(req_buf).map_err(|_| TransportError::BufferTooSmall)?;
        request
            .serialize(&mut writer)
            .map_err(|_| TransportError::SerializationError)?;
        let req_len = writer.position();

        // Send request and wait for reply
        let reply_len = self.call_raw(&req_buf[..req_len], reply_buf)?;

        // Deserialize reply
        let mut reader = CdrReader::new_with_header(&reply_buf[..reply_len])
            .map_err(|_| TransportError::DeserializationError)?;
        let reply =
            S::Reply::deserialize(&mut reader).map_err(|_| TransportError::DeserializationError)?;

        Ok(reply)
    }
}

/// Transport backend trait
pub trait Transport {
    /// Error type for this transport
    type Error;
    /// Session type for this transport
    type Session: Session;

    /// Open a new session with the given configuration
    fn open(config: &TransportConfig) -> Result<Self::Session, Self::Error>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_topic_info() {
        let topic = TopicInfo::new("/chatter", "std_msgs::msg::dds_::String_", "abc123");
        assert_eq!(topic.name, "/chatter");
        assert_eq!(topic.domain_id, 0);
    }

    #[test]
    fn test_topic_key_generation() {
        let topic =
            TopicInfo::new("/chatter", "std_msgs::msg::dds_::String_", "abc123").with_domain(42);

        let key: heapless::String<128> = topic.to_key();
        assert!(key.contains("42"));
        assert!(key.contains("chatter")); // Leading slash stripped
        assert!(key.contains("/abc123")); // Type hash at the end (no RIHS01_ prefix for Humble)
    }

    #[test]
    fn test_qos_defaults() {
        let qos = QosSettings::default();
        assert!(!qos.reliable);
    }

    #[test]
    fn test_action_info() {
        let action = ActionInfo::new(
            "/fibonacci",
            "example_interfaces::action::dds_::Fibonacci_",
            "abc123",
        );
        assert_eq!(action.name, "/fibonacci");
        assert_eq!(action.domain_id, 0);
    }

    #[test]
    fn test_action_info_with_domain() {
        let action = ActionInfo::new(
            "/fibonacci",
            "example_interfaces::action::dds_::Fibonacci_",
            "abc123",
        )
        .with_domain(42);
        assert_eq!(action.domain_id, 42);
    }

    #[test]
    fn test_action_send_goal_key() {
        let action = ActionInfo::new(
            "/fibonacci",
            "example_interfaces::action::dds_::Fibonacci_",
            "abc123",
        )
        .with_domain(0);

        let key: heapless::String<256> = action.send_goal_key();
        assert!(key.contains("fibonacci"));
        assert!(key.contains("_action"));
        assert!(key.contains("SendGoal"));
        assert!(key.contains("RIHS01_"));
    }

    #[test]
    fn test_action_feedback_key() {
        let action = ActionInfo::new(
            "/fibonacci",
            "example_interfaces::action::dds_::Fibonacci_",
            "abc123",
        )
        .with_domain(0);

        let key: heapless::String<256> = action.feedback_key();
        assert!(key.contains("fibonacci"));
        assert!(key.contains("_action"));
        assert!(key.contains("FeedbackMessage"));
    }
}
