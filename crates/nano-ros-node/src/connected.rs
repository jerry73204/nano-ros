//! Connected node implementation with transport integration
//!
//! This module provides types that integrate the Node API with actual
//! transport backends (like zenoh-pico) for sending and receiving messages.

use nano_ros_core::{RosMessage, RosService};
use nano_ros_transport::{
    Publisher as PublisherTrait, QosSettings, ServiceClientTrait, ServiceInfo, ServiceServerTrait,
    Session, SessionMode, Subscriber as SubscriberTrait, TopicInfo, Transport, TransportConfig,
    TransportError,
};

#[cfg(feature = "zenoh")]
use nano_ros_transport::{
    LivelinessToken, Ros2Liveliness, ZenohId, ZenohPublisher, ZenohServiceClient,
    ZenohServiceServer, ZenohSession, ZenohSubscriber, ZenohTransport,
};

#[cfg(feature = "zenoh")]
extern crate alloc;
#[cfg(feature = "zenoh")]
use alloc::vec::Vec;

use crate::NodeConfig;

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
            _ => ConnectedNodeError::ConnectionFailed,
        }
    }
}

/// A connected node that can publish and subscribe via zenoh transport
#[cfg(feature = "zenoh")]
pub struct ConnectedNode {
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
    /// Publisher/subscriber liveliness tokens
    _entity_tokens: Vec<LivelinessToken>,
}

#[cfg(feature = "zenoh")]
impl ConnectedNode {
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
            _entity_tokens: Vec::new(),
        })
    }

    /// Connect to a zenoh router
    pub fn connect(config: NodeConfig, locator: &str) -> Result<Self, ConnectedNodeError> {
        let transport_config = TransportConfig {
            locator: Some(locator),
            mode: SessionMode::Client,
        };
        Self::new(config, &transport_config)
    }

    /// Connect in peer mode (no router required)
    pub fn connect_peer(config: NodeConfig) -> Result<Self, ConnectedNodeError> {
        let transport_config = TransportConfig {
            locator: None,
            mode: SessionMode::Peer,
        };
        Self::new(config, &transport_config)
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
            self._entity_tokens.push(token);
        }

        Ok(ConnectedPublisher {
            publisher,
            _marker: core::marker::PhantomData,
        })
    }

    /// Create a subscriber for the given topic
    pub fn create_subscriber<M: RosMessage>(
        &mut self,
        topic: &str,
    ) -> Result<ConnectedSubscriber<M>, ConnectedNodeError> {
        self.create_subscriber_with_qos(topic, QosSettings::BEST_EFFORT)
    }

    /// Create a subscriber with custom QoS settings
    pub fn create_subscriber_with_qos<M: RosMessage>(
        &mut self,
        topic: &str,
        qos: QosSettings,
    ) -> Result<ConnectedSubscriber<M>, ConnectedNodeError> {
        let topic_info =
            TopicInfo::new(topic, M::TYPE_NAME, M::TYPE_HASH).with_domain(self.domain_id);

        let subscriber = self.session.create_subscriber(&topic_info, qos)?;

        // Declare subscriber liveliness token for ROS 2 discovery
        let sub_keyexpr =
            Ros2Liveliness::subscriber_keyexpr(self.domain_id, &self.zid, &self.name, &topic_info);
        if let Ok(token) = self.session.declare_liveliness(&sub_keyexpr) {
            self._entity_tokens.push(token);
        }

        Ok(ConnectedSubscriber {
            subscriber,
            rx_buffer: [0u8; 1024],
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
    ) -> Result<ConnectedServiceServer<S>, ConnectedNodeError> {
        let service_info = ServiceInfo::new(service_name, S::SERVICE_NAME, S::SERVICE_HASH)
            .with_domain(self.domain_id);

        let server = self.session.create_service_server(&service_info)?;

        Ok(ConnectedServiceServer {
            server,
            req_buffer: [0u8; 1024],
            reply_buffer: [0u8; 1024],
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
    ) -> Result<ConnectedServiceClient<S>, ConnectedNodeError> {
        let service_info = ServiceInfo::new(service_name, S::SERVICE_NAME, S::SERVICE_HASH)
            .with_domain(self.domain_id);

        let client = self.session.create_service_client(&service_info)?;

        Ok(ConnectedServiceClient {
            client,
            req_buffer: [0u8; 1024],
            reply_buffer: [0u8; 1024],
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
#[cfg(feature = "zenoh")]
pub struct ConnectedSubscriber<M> {
    subscriber: ZenohSubscriber,
    rx_buffer: [u8; 1024],
    _marker: core::marker::PhantomData<M>,
}

#[cfg(feature = "zenoh")]
impl<M: RosMessage> ConnectedSubscriber<M> {
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
}

/// A connected service server that can handle service requests
#[cfg(feature = "zenoh")]
pub struct ConnectedServiceServer<S: RosService> {
    server: ZenohServiceServer,
    req_buffer: [u8; 1024],
    reply_buffer: [u8; 1024],
    _marker: core::marker::PhantomData<S>,
}

#[cfg(feature = "zenoh")]
impl<S: RosService> ConnectedServiceServer<S> {
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
}

/// A connected service client that can send service requests
#[cfg(feature = "zenoh")]
pub struct ConnectedServiceClient<S: RosService> {
    client: ZenohServiceClient,
    req_buffer: [u8; 1024],
    reply_buffer: [u8; 1024],
    _marker: core::marker::PhantomData<S>,
}

#[cfg(feature = "zenoh")]
impl<S: RosService> ConnectedServiceClient<S> {
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
}

#[cfg(all(test, feature = "zenoh"))]
mod tests {
    // Tests require a running zenoh router or peer mode
    // They are in the integration test file
}
