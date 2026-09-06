//! Node implementation

use heapless::Vec;
use nros_core::RosMessage;
use nros_rmw::{QoSProfile, TopicInfo};

use crate::{publisher::PublisherHandle, subscriber::SubscriptionHandle};

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
        Self::new("nros_node", "/")
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
    /// A topic/endpoint name exceeded the bounded string capacity (Phase 192.1 —
    /// surfaced instead of silently truncating, which would corrupt the wire key).
    TopicNameTooLong,
    /// A node name exceeded the bounded string capacity.
    NameTooLong,
    /// A node namespace exceeded the bounded string capacity.
    NamespaceTooLong,
}

/// Default serialization (tx) / reception (rx) buffer length for a `Node`.
/// Phase 192.5 — names the inline `1024` (was duplicated at the field decl + the
/// `Node::new` initializer).
pub const NODE_TX_BUF_LEN: usize = 1024;
/// Default reception buffer length for a `Node`.
pub const NODE_RX_BUF_LEN: usize = 1024;

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
    qos: QoSProfile,
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
    qos: QoSProfile,
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
    tx_buffer: [u8; NODE_TX_BUF_LEN],
    /// Reception buffer for subscribing
    #[allow(dead_code)] // Used when transport is connected
    rx_buffer: [u8; NODE_RX_BUF_LEN],
}

/// Options for creating a publisher
#[derive(Debug, Clone)]
pub struct PublisherOptions<'a> {
    /// Topic name
    pub topic: &'a str,
    /// QoS settings
    pub qos: QoSProfile,
}

impl<'a> PublisherOptions<'a> {
    /// Create new publisher options with the given topic and default QoS
    pub fn new(topic: &'a str) -> Self {
        Self {
            topic,
            qos: QoSProfile::default(),
        }
    }

    /// Set the QoS settings
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }
}

/// Options for creating a subscriber
#[derive(Debug, Clone)]
pub struct SubscriptionOptions<'a> {
    /// Topic name
    pub topic: &'a str,
    /// QoS settings
    pub qos: QoSProfile,
}

impl<'a> SubscriptionOptions<'a> {
    /// Create new subscriber options with the given topic and default QoS
    pub fn new(topic: &'a str) -> Self {
        Self {
            topic,
            qos: QoSProfile::default(),
        }
    }

    /// Set the QoS settings
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }
}

impl<const MAX_PUBS: usize, const MAX_SUBS: usize> Node<MAX_PUBS, MAX_SUBS> {
    /// Create a new node with the given configuration
    pub fn new(config: NodeConfig) -> Result<Self, NodeError> {
        // Phase 192.1 — validate the name/namespace fit the bounded buffers
        // instead of silently truncating (a truncated node identity mis-routes).
        let mut name = heapless::String::new();
        name.push_str(config.name)
            .map_err(|_| NodeError::NameTooLong)?;

        let mut namespace = heapless::String::new();
        namespace
            .push_str(config.namespace)
            .map_err(|_| NodeError::NamespaceTooLong)?;

        Ok(Self {
            name,
            namespace,
            domain_id: config.domain_id,
            publishers: Vec::new(),
            subscribers: Vec::new(),
            tx_buffer: [0u8; NODE_TX_BUF_LEN],
            rx_buffer: [0u8; NODE_RX_BUF_LEN],
        })
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

    /// Get the fully qualified node name.
    ///
    /// Phase 192.1 — fallible: namespace (≤64) + `/` + name (≤64) can reach 129
    /// bytes, one over the `String<128>` capacity, so a maxed name+namespace
    /// would otherwise silently truncate the FQN.
    pub fn fully_qualified_name(&self) -> Result<heapless::String<128>, NodeError> {
        // Delegates rather than joining again: `crate::names::fully_qualified_name`
        // is the one implementation, and this used to be the second of three.
        //
        // The old body agreed with it on the cases anyone tested — `/my_ns` +
        // `my_node`, and a root `/` collapsed correctly — and differed on a
        // namespace with NO leading slash: it returned `my_ns/my_node`, which is
        // not fully qualified. ROS 2 namespaces are absolute, so the shared
        // helper adds the slash.
        crate::names::fully_qualified_name(&self.name, &self.namespace)
            .map_err(|()| NodeError::NameTooLong)
    }

    /// Create a publisher with the given options
    pub fn create_publisher<M: RosMessage>(
        &mut self,
        options: PublisherOptions,
    ) -> Result<PublisherHandle<M>, NodeError> {
        if self.publishers.len() >= MAX_PUBS {
            return Err(NodeError::MaxPublishersReached);
        }

        let mut topic_name = heapless::String::new();
        // Phase 192.1 — error on overflow instead of silently truncating the
        // topic name (a truncated key expression mis-routes on the wire).
        topic_name
            .push_str(options.topic)
            .map_err(|_| NodeError::TopicNameTooLong)?;

        let info = PublisherInfo {
            topic_name,
            type_name: M::TYPE_NAME,
            type_hash: M::TYPE_HASH,
            qos: options.qos,
            active: true,
        };

        let index = self.publishers.len();
        self.publishers
            .push(info)
            .map_err(|_| NodeError::MaxPublishersReached)?;

        Ok(PublisherHandle::new(index))
    }

    /// Create a subscriber with the given options
    pub fn create_subscription<M: RosMessage>(
        &mut self,
        options: SubscriptionOptions,
    ) -> Result<SubscriptionHandle<M>, NodeError> {
        if self.subscribers.len() >= MAX_SUBS {
            return Err(NodeError::MaxSubscribersReached);
        }

        let mut topic_name = heapless::String::new();
        // Phase 192.1 — error on overflow instead of silently truncating.
        topic_name
            .push_str(options.topic)
            .map_err(|_| NodeError::TopicNameTooLong)?;

        let info = SubscriberInfo {
            topic_name,
            type_name: M::TYPE_NAME,
            type_hash: M::TYPE_HASH,
            qos: options.qos,
            active: true,
        };

        let index = self.subscribers.len();
        self.subscribers
            .push(info)
            .map_err(|_| NodeError::MaxSubscribersReached)?;

        Ok(SubscriptionHandle::new(index))
    }

    /// Get topic info for a publisher
    pub fn publisher_topic_info(&self, handle: PublisherHandle<()>) -> Option<TopicInfo<'_>> {
        self.publishers.get(handle.index()).map(|info| {
            TopicInfo::new(&info.topic_name, info.type_name, info.type_hash)
                .with_domain(self.domain_id)
        })
    }

    /// Get topic info for a subscriber
    pub fn subscription_topic_info(&self, handle: SubscriptionHandle<()>) -> Option<TopicInfo<'_>> {
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
        let mut writer =
            crate::tx_writer(&mut self.tx_buffer).map_err(|_| NodeError::BufferTooSmall)?;
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
        _handle: &SubscriptionHandle<M>,
        data: &[u8],
    ) -> Result<M, NodeError> {
        use nros_core::CdrReader;

        let mut reader =
            CdrReader::new_with_header(data).map_err(|_| NodeError::DeserializationFailed)?;
        M::deserialize(&mut reader).map_err(|_| NodeError::DeserializationFailed)
    }

    /// Get the number of active publishers
    pub fn publisher_count(&self) -> usize {
        self.publishers.iter().filter(|p| p.active).count()
    }

    /// Get the number of active subscribers
    pub fn subscription_count(&self) -> usize {
        self.subscribers.iter().filter(|s| s.active).count()
    }
}

impl<const MAX_PUBS: usize, const MAX_SUBS: usize> Default for Node<MAX_PUBS, MAX_SUBS> {
    fn default() -> Self {
        // The default config name ("nros_node") + namespace ("/") are short
        // literals that always fit the bounded buffers, so `new` can't fail here.
        Self::new(NodeConfig::default())
            .expect("default NodeConfig fits the bounded name/namespace")
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

    impl nros_core::Serialize for TestMessage {
        fn serialize(&self, writer: &mut nros_core::CdrWriter) -> Result<(), nros_core::SerError> {
            self.data.serialize(writer)
        }
    }

    impl nros_core::Deserialize for TestMessage {
        fn deserialize(reader: &mut nros_core::CdrReader) -> Result<Self, nros_core::DeserError> {
            Ok(Self {
                data: i32::deserialize(reader)?,
            })
        }
    }

    #[test]
    fn test_node_creation() {
        let config = NodeConfig::new("test_node", "/test");
        let node = Node::<4, 4>::new(config).unwrap();

        assert_eq!(node.name(), "test_node");
        assert_eq!(node.namespace(), "/test");
        assert_eq!(node.domain_id(), 0);
    }

    #[test]
    fn test_fully_qualified_name() {
        let config = NodeConfig::new("my_node", "/my_ns");
        let node = Node::<4, 4>::new(config).unwrap();

        assert_eq!(
            node.fully_qualified_name().unwrap().as_str(),
            "/my_ns/my_node"
        );
    }

    /// The two cases that separate a correct join from a plausible one, and
    /// the reason this delegates to `names::fully_qualified_name` instead of
    /// keeping its own three lines.
    #[test]
    fn fully_qualified_name_collapses_root_and_absolutises() {
        for (ns, want) in [
            // Root must not double the slash.
            ("/", "/my_node"),
            ("/my_ns", "/my_ns/my_node"),
            // A namespace without a leading slash is still absolute. The
            // hand-rolled body this replaced returned `my_ns/my_node`.
            ("my_ns", "/my_ns/my_node"),
        ] {
            let node = Node::<4, 4>::new(NodeConfig::new("my_node", ns)).unwrap();
            assert_eq!(
                node.fully_qualified_name().unwrap().as_str(),
                want,
                "namespace {ns:?}"
            );
        }
    }

    #[test]
    fn test_create_publisher() {
        let mut node = Node::<4, 4>::default();
        let handle = node.create_publisher::<TestMessage>(PublisherOptions::new("/test_topic"));

        assert!(handle.is_ok());
        assert_eq!(node.publisher_count(), 1);
    }

    #[test]
    fn test_create_subscriber() {
        let mut node = Node::<4, 4>::default();
        let handle =
            node.create_subscription::<TestMessage>(SubscriptionOptions::new("/test_topic"));

        assert!(handle.is_ok());
        assert_eq!(node.subscription_count(), 1);
    }

    #[test]
    fn test_max_publishers() {
        let mut node = Node::<2, 2>::default();

        let _ = node.create_publisher::<TestMessage>(PublisherOptions::new("/topic1"));
        let _ = node.create_publisher::<TestMessage>(PublisherOptions::new("/topic2"));
        let result = node.create_publisher::<TestMessage>(PublisherOptions::new("/topic3"));

        assert_eq!(result, Err(NodeError::MaxPublishersReached));
    }

    #[test]
    fn test_serialize_deserialize() {
        let mut node = Node::<4, 4>::default();
        let pub_handle = node
            .create_publisher::<TestMessage>(PublisherOptions::new("/test"))
            .unwrap();
        let sub_handle = node
            .create_subscription::<TestMessage>(SubscriptionOptions::new("/test"))
            .unwrap();

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
