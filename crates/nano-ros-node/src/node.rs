//! Node implementation

use heapless::Vec;
use nano_ros_core::RosMessage;
use nano_ros_transport::{QosSettings, TopicInfo};

use crate::publisher::PublisherHandle;
use crate::subscriber::SubscriberHandle;

/// Node configuration
#[derive(Debug, Clone)]
pub struct NodeConfig<'a> {
    /// Node name
    pub name: &'a str,
    /// Node namespace
    pub namespace: &'a str,
    /// Domain ID (default: 0)
    pub domain_id: u32,
}

impl<'a> NodeConfig<'a> {
    /// Create a new node configuration
    pub const fn new(name: &'a str, namespace: &'a str) -> Self {
        Self {
            name,
            namespace,
            domain_id: 0,
        }
    }

    /// Set the domain ID
    pub const fn with_domain(mut self, domain_id: u32) -> Self {
        self.domain_id = domain_id;
        self
    }
}

impl Default for NodeConfig<'_> {
    fn default() -> Self {
        Self::new("nano_ros_node", "/")
    }
}

/// Node error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeError {
    /// Maximum publishers reached
    MaxPublishersReached,
    /// Maximum subscribers reached
    MaxSubscribersReached,
    /// Invalid publisher handle
    InvalidPublisherHandle,
    /// Invalid subscriber handle
    InvalidSubscriberHandle,
    /// Serialization failed
    SerializationFailed,
    /// Deserialization failed
    DeserializationFailed,
    /// Buffer too small
    BufferTooSmall,
    /// Transport error
    TransportError,
    /// Not connected
    NotConnected,
}

/// Publisher registration info
#[derive(Debug, Clone)]
#[allow(dead_code)] // Fields used when transport is connected
struct PublisherInfo {
    /// Topic name
    topic_name: heapless::String<64>,
    /// Type name
    type_name: &'static str,
    /// Type hash
    type_hash: &'static str,
    /// QoS settings
    qos: QosSettings,
    /// Active flag
    active: bool,
}

/// Subscriber registration info
#[derive(Debug, Clone)]
#[allow(dead_code)] // Fields used when transport is connected
struct SubscriberInfo {
    /// Topic name
    topic_name: heapless::String<64>,
    /// Type name
    type_name: &'static str,
    /// Type hash
    type_hash: &'static str,
    /// QoS settings
    qos: QosSettings,
    /// Active flag
    active: bool,
}

/// ROS 2 Node for embedded systems
///
/// The node manages publishers and subscribers with static allocation.
/// `MAX_PUBS` and `MAX_SUBS` define the maximum number of publishers
/// and subscribers that can be created.
///
/// # Type Parameters
///
/// - `MAX_PUBS`: Maximum number of publishers
/// - `MAX_SUBS`: Maximum number of subscribers
pub struct Node<const MAX_PUBS: usize = 8, const MAX_SUBS: usize = 8> {
    /// Node name
    name: heapless::String<64>,
    /// Node namespace
    namespace: heapless::String<64>,
    /// Domain ID
    domain_id: u32,
    /// Registered publishers
    publishers: Vec<PublisherInfo, MAX_PUBS>,
    /// Registered subscribers
    subscribers: Vec<SubscriberInfo, MAX_SUBS>,
    /// Serialization buffer for publishing
    tx_buffer: [u8; 1024],
    /// Reception buffer for subscribing
    #[allow(dead_code)] // Used when transport is connected
    rx_buffer: [u8; 1024],
}

impl<const MAX_PUBS: usize, const MAX_SUBS: usize> Node<MAX_PUBS, MAX_SUBS> {
    /// Create a new node with the given configuration
    pub fn new(config: NodeConfig) -> Self {
        let mut name = heapless::String::new();
        let _ = name.push_str(config.name);

        let mut namespace = heapless::String::new();
        let _ = namespace.push_str(config.namespace);

        Self {
            name,
            namespace,
            domain_id: config.domain_id,
            publishers: Vec::new(),
            subscribers: Vec::new(),
            tx_buffer: [0u8; 1024],
            rx_buffer: [0u8; 1024],
        }
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

    /// Get the fully qualified node name
    pub fn fully_qualified_name(&self) -> heapless::String<128> {
        let mut fqn = heapless::String::new();
        let _ = fqn.push_str(&self.namespace);
        if !self.namespace.ends_with('/') {
            let _ = fqn.push('/');
        }
        let _ = fqn.push_str(&self.name);
        fqn
    }

    /// Create a publisher for the given topic
    ///
    /// Returns a handle that can be used to publish messages.
    pub fn create_publisher<M: RosMessage>(
        &mut self,
        topic: &str,
    ) -> Result<PublisherHandle<M>, NodeError> {
        self.create_publisher_with_qos::<M>(topic, QosSettings::BEST_EFFORT)
    }

    /// Create a publisher with custom QoS settings
    pub fn create_publisher_with_qos<M: RosMessage>(
        &mut self,
        topic: &str,
        qos: QosSettings,
    ) -> Result<PublisherHandle<M>, NodeError> {
        if self.publishers.len() >= MAX_PUBS {
            return Err(NodeError::MaxPublishersReached);
        }

        let mut topic_name = heapless::String::new();
        let _ = topic_name.push_str(topic);

        let info = PublisherInfo {
            topic_name,
            type_name: M::TYPE_NAME,
            type_hash: M::TYPE_HASH,
            qos,
            active: true,
        };

        let index = self.publishers.len();
        self.publishers
            .push(info)
            .map_err(|_| NodeError::MaxPublishersReached)?;

        Ok(PublisherHandle::new(index))
    }

    /// Create a subscriber for the given topic
    ///
    /// Returns a handle that can be used to receive messages.
    pub fn create_subscriber<M: RosMessage>(
        &mut self,
        topic: &str,
    ) -> Result<SubscriberHandle<M>, NodeError> {
        self.create_subscriber_with_qos::<M>(topic, QosSettings::BEST_EFFORT)
    }

    /// Create a subscriber with custom QoS settings
    pub fn create_subscriber_with_qos<M: RosMessage>(
        &mut self,
        topic: &str,
        qos: QosSettings,
    ) -> Result<SubscriberHandle<M>, NodeError> {
        if self.subscribers.len() >= MAX_SUBS {
            return Err(NodeError::MaxSubscribersReached);
        }

        let mut topic_name = heapless::String::new();
        let _ = topic_name.push_str(topic);

        let info = SubscriberInfo {
            topic_name,
            type_name: M::TYPE_NAME,
            type_hash: M::TYPE_HASH,
            qos,
            active: true,
        };

        let index = self.subscribers.len();
        self.subscribers
            .push(info)
            .map_err(|_| NodeError::MaxSubscribersReached)?;

        Ok(SubscriberHandle::new(index))
    }

    /// Get topic info for a publisher
    pub fn publisher_topic_info(&self, handle: PublisherHandle<()>) -> Option<TopicInfo<'_>> {
        self.publishers.get(handle.index()).map(|info| {
            TopicInfo::new(&info.topic_name, info.type_name, info.type_hash)
                .with_domain(self.domain_id)
        })
    }

    /// Get topic info for a subscriber
    pub fn subscriber_topic_info(&self, handle: SubscriberHandle<()>) -> Option<TopicInfo<'_>> {
        self.subscribers.get(handle.index()).map(|info| {
            TopicInfo::new(&info.topic_name, info.type_name, info.type_hash)
                .with_domain(self.domain_id)
        })
    }

    /// Serialize a message for publishing
    ///
    /// Returns the serialized bytes. The caller is responsible for
    /// sending the bytes via the transport layer.
    pub fn serialize_message<M: RosMessage>(
        &mut self,
        _handle: &PublisherHandle<M>,
        msg: &M,
    ) -> Result<&[u8], NodeError> {
        use nano_ros_core::CdrWriter;

        let mut writer = CdrWriter::new_with_header(&mut self.tx_buffer)
            .map_err(|_| NodeError::BufferTooSmall)?;
        msg.serialize(&mut writer)
            .map_err(|_| NodeError::SerializationFailed)?;
        let len = writer.position();

        Ok(&self.tx_buffer[..len])
    }

    /// Deserialize a received message
    ///
    /// The caller provides the raw bytes received from the transport layer.
    pub fn deserialize_message<M: RosMessage>(
        &self,
        _handle: &SubscriberHandle<M>,
        data: &[u8],
    ) -> Result<M, NodeError> {
        use nano_ros_core::CdrReader;

        let mut reader =
            CdrReader::new_with_header(data).map_err(|_| NodeError::DeserializationFailed)?;
        M::deserialize(&mut reader).map_err(|_| NodeError::DeserializationFailed)
    }

    /// Get the number of active publishers
    pub fn publisher_count(&self) -> usize {
        self.publishers.iter().filter(|p| p.active).count()
    }

    /// Get the number of active subscribers
    pub fn subscriber_count(&self) -> usize {
        self.subscribers.iter().filter(|s| s.active).count()
    }
}

impl<const MAX_PUBS: usize, const MAX_SUBS: usize> Default for Node<MAX_PUBS, MAX_SUBS> {
    fn default() -> Self {
        Self::new(NodeConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Mock message type for testing
    #[derive(Debug, Clone, Default)]
    struct TestMessage {
        data: i32,
    }

    impl RosMessage for TestMessage {
        const TYPE_NAME: &'static str = "test_msgs::msg::TestMessage";
        const TYPE_HASH: &'static str = "abc123";
    }

    impl nano_ros_core::Serialize for TestMessage {
        fn serialize(
            &self,
            writer: &mut nano_ros_core::CdrWriter,
        ) -> Result<(), nano_ros_core::SerError> {
            self.data.serialize(writer)
        }
    }

    impl nano_ros_core::Deserialize for TestMessage {
        fn deserialize(
            reader: &mut nano_ros_core::CdrReader,
        ) -> Result<Self, nano_ros_core::DeserError> {
            Ok(Self {
                data: i32::deserialize(reader)?,
            })
        }
    }

    #[test]
    fn test_node_creation() {
        let config = NodeConfig::new("test_node", "/test");
        let node = Node::<4, 4>::new(config);

        assert_eq!(node.name(), "test_node");
        assert_eq!(node.namespace(), "/test");
        assert_eq!(node.domain_id(), 0);
    }

    #[test]
    fn test_fully_qualified_name() {
        let config = NodeConfig::new("my_node", "/my_ns");
        let node = Node::<4, 4>::new(config);

        assert_eq!(node.fully_qualified_name().as_str(), "/my_ns/my_node");
    }

    #[test]
    fn test_create_publisher() {
        let mut node = Node::<4, 4>::default();
        let handle = node.create_publisher::<TestMessage>("/test_topic");

        assert!(handle.is_ok());
        assert_eq!(node.publisher_count(), 1);
    }

    #[test]
    fn test_create_subscriber() {
        let mut node = Node::<4, 4>::default();
        let handle = node.create_subscriber::<TestMessage>("/test_topic");

        assert!(handle.is_ok());
        assert_eq!(node.subscriber_count(), 1);
    }

    #[test]
    fn test_max_publishers() {
        let mut node = Node::<2, 2>::default();

        let _ = node.create_publisher::<TestMessage>("/topic1");
        let _ = node.create_publisher::<TestMessage>("/topic2");
        let result = node.create_publisher::<TestMessage>("/topic3");

        assert_eq!(result, Err(NodeError::MaxPublishersReached));
    }

    #[test]
    fn test_serialize_deserialize() {
        let mut node = Node::<4, 4>::default();
        let pub_handle = node.create_publisher::<TestMessage>("/test").unwrap();
        let sub_handle = node.create_subscriber::<TestMessage>("/test").unwrap();

        let msg = TestMessage { data: 42 };

        // Copy bytes since serialize_message borrows the internal buffer
        let mut buf = [0u8; 128];
        let bytes = node.serialize_message(&pub_handle, &msg).unwrap();
        let len = bytes.len();
        buf[..len].copy_from_slice(bytes);

        let received: TestMessage = node.deserialize_message(&sub_handle, &buf[..len]).unwrap();
        assert_eq!(received.data, 42);
    }
}
