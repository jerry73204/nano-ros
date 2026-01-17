//! Transport abstraction traits
//!
//! These traits define the interface for transport backends (zenoh-pico, etc.)

use nano_ros_core::RosMessage;

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
    /// Format: `<domain_id>/<topic_name>/<type_name>/RIHS01_<hash>`
    /// Note: Leading/trailing slashes are stripped from topic name
    pub fn to_key<const N: usize>(&self) -> heapless::String<N> {
        let mut key = heapless::String::new();
        // Strip leading/trailing slashes from topic name (matching rmw_zenoh's strip_slashes)
        let topic_stripped = self.name.trim_matches('/');
        // Best effort - ignore errors if key is too long
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/{}/RIHS01_{}",
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
    /// Failed to publish message
    PublishFailed,
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
        assert!(key.contains("/chatter"));
        assert!(key.contains("RIHS01_abc123"));
    }

    #[test]
    fn test_qos_defaults() {
        let qos = QosSettings::default();
        assert!(!qos.reliable);
    }
}
