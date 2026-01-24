//! Shim-based executor and node for embedded platforms
//!
//! This module provides a simplified executor and node API using zenoh-pico-shim.
//! It's designed for embedded platforms that need manual polling without
//! background threads.
//!
//! # Differences from zenoh-based API
//!
//! - No liveliness tokens (no ROS 2 discovery)
//! - No services (not yet implemented)
//! - No RMW attachments
//! - Simpler, no_std friendly API
//!
//! # Example
//!
//! ```ignore
//! use nano_ros_node::shim::{ShimExecutor, ShimNode};
//! use std_msgs::msg::Int32;
//!
//! // Create executor with locator
//! let mut executor = ShimExecutor::new(b"tcp/192.168.1.1:7447\0")?;
//!
//! // Create node
//! let node = executor.create_node("my_node")?;
//!
//! // Create publisher
//! let publisher = node.create_publisher::<Int32>("/chatter")?;
//!
//! // In your main loop or RTIC task:
//! loop {
//!     // Poll network and dispatch callbacks
//!     executor.spin_once(10)?;
//!
//!     // Publish periodically
//!     publisher.publish(&Int32 { data: 42 })?;
//!
//!     // platform delay...
//! }
//! ```

use core::marker::PhantomData;

use heapless::String;
use nano_ros_core::{CdrWriter, RosMessage};
use nano_ros_transport::{
    Publisher, QosSettings, Session, ShimPublisher, ShimSession, ShimSubscriber, ShimTransport,
    Subscriber, TopicInfo, Transport, TransportConfig, TransportError,
};

// ============================================================================
// Error Types
// ============================================================================

/// Error type for shim operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShimNodeError {
    /// Transport error
    Transport(TransportError),
    /// Node name too long
    NameTooLong,
    /// Topic name too long
    TopicTooLong,
    /// Maximum publishers reached
    TooManyPublishers,
    /// Maximum subscribers reached
    TooManySubscribers,
    /// Serialization error
    Serialization,
    /// Buffer too small
    BufferTooSmall,
}

impl From<TransportError> for ShimNodeError {
    fn from(err: TransportError) -> Self {
        ShimNodeError::Transport(err)
    }
}

// ============================================================================
// ShimExecutor
// ============================================================================

/// Polling-based executor for embedded platforms
///
/// This executor manages a single zenoh session and provides manual polling
/// control. Unlike the full executor, it does not manage multiple nodes
/// internally - you get one node at a time.
///
/// # Type Parameters
///
/// * `MAX_PUBLISHERS` - Maximum number of publishers (default: 8)
/// * `MAX_SUBSCRIBERS` - Maximum number of subscribers (default: 8)
pub struct ShimExecutor {
    session: ShimSession,
}

impl ShimExecutor {
    /// Create a new executor with the given locator
    ///
    /// # Arguments
    ///
    /// * `locator` - Null-terminated connection string (e.g., `b"tcp/192.168.1.1:7447\0"`)
    ///
    /// # Example
    ///
    /// ```ignore
    /// let executor = ShimExecutor::new(b"tcp/192.168.1.1:7447\0")?;
    /// ```
    pub fn new(locator: &[u8]) -> Result<Self, ShimNodeError> {
        // Convert locator to str (removing null terminator for config)
        let locator_str = core::str::from_utf8(locator)
            .map_err(|_| ShimNodeError::Transport(TransportError::InvalidConfig))?
            .trim_end_matches('\0');

        let config = TransportConfig {
            locator: Some(locator_str),
            mode: nano_ros_transport::SessionMode::Client,
        };

        let session = ShimTransport::open(&config)?;

        Ok(Self { session })
    }

    /// Create a new executor with custom transport configuration
    ///
    /// # Arguments
    ///
    /// * `config` - Transport configuration
    pub fn with_config(config: &TransportConfig) -> Result<Self, ShimNodeError> {
        let session = ShimTransport::open(config)?;
        Ok(Self { session })
    }

    /// Create a node on this executor
    ///
    /// # Arguments
    ///
    /// * `name` - Node name
    ///
    /// # Returns
    ///
    /// A new node that can create publishers and subscribers
    pub fn create_node(&mut self, name: &str) -> Result<ShimNode<'_>, ShimNodeError> {
        if name.len() > 64 {
            return Err(ShimNodeError::NameTooLong);
        }

        let mut node_name = String::<64>::new();
        node_name
            .push_str(name)
            .map_err(|_| ShimNodeError::NameTooLong)?;

        Ok(ShimNode {
            name: node_name,
            session: &mut self.session,
            domain_id: 0,
        })
    }

    /// Poll for incoming data and process callbacks
    ///
    /// Call this periodically (recommended: every 10ms) to process network
    /// data and dispatch subscriber callbacks.
    ///
    /// # Arguments
    ///
    /// * `timeout_ms` - Maximum time to wait for data (0 = non-blocking)
    ///
    /// # Returns
    ///
    /// Number of events processed, or error
    pub fn spin_once(&self, timeout_ms: u32) -> Result<i32, ShimNodeError> {
        self.session.spin_once(timeout_ms).map_err(|e| e.into())
    }

    /// Poll for incoming data without keepalive
    ///
    /// Use `spin_once` instead unless you need separate control over
    /// polling and keepalive.
    pub fn poll(&self, timeout_ms: u32) -> Result<i32, ShimNodeError> {
        self.session.poll(timeout_ms).map_err(|e| e.into())
    }

    /// Check if the session is open
    pub fn is_open(&self) -> bool {
        self.session.is_open()
    }

    /// Get a reference to the underlying session
    pub fn session(&self) -> &ShimSession {
        &self.session
    }

    /// Get a mutable reference to the underlying session
    pub fn session_mut(&mut self) -> &mut ShimSession {
        &mut self.session
    }
}

// ============================================================================
// ShimNode
// ============================================================================

/// Node for creating publishers and subscribers
///
/// A node is created through `ShimExecutor::create_node()` and provides
/// methods to create publishers and subscribers for topics.
pub struct ShimNode<'a> {
    name: String<64>,
    session: &'a mut ShimSession,
    domain_id: u32,
}

impl<'a> ShimNode<'a> {
    /// Get the node name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the domain ID
    pub fn domain_id(&self) -> u32 {
        self.domain_id
    }

    /// Set the domain ID
    pub fn set_domain_id(&mut self, domain_id: u32) {
        self.domain_id = domain_id;
    }

    /// Create a publisher for the given topic
    ///
    /// # Arguments
    ///
    /// * `topic_name` - Topic name (e.g., "/chatter")
    ///
    /// # Type Parameters
    ///
    /// * `M` - Message type implementing RosMessage
    ///
    /// # Example
    ///
    /// ```ignore
    /// let publisher = node.create_publisher::<Int32>("/chatter")?;
    /// publisher.publish(&Int32 { data: 42 })?;
    /// ```
    pub fn create_publisher<M: RosMessage>(
        &mut self,
        topic_name: &str,
    ) -> Result<ShimNodePublisher<M>, ShimNodeError> {
        self.create_publisher_with_qos::<M>(topic_name, QosSettings::default())
    }

    /// Create a publisher with custom QoS settings
    pub fn create_publisher_with_qos<M: RosMessage>(
        &mut self,
        topic_name: &str,
        qos: QosSettings,
    ) -> Result<ShimNodePublisher<M>, ShimNodeError> {
        let topic =
            TopicInfo::new(topic_name, M::TYPE_NAME, M::TYPE_HASH).with_domain(self.domain_id);

        let publisher = self.session.create_publisher(&topic, qos)?;

        Ok(ShimNodePublisher {
            publisher,
            _phantom: PhantomData,
        })
    }

    /// Create a subscriber for the given topic
    ///
    /// # Arguments
    ///
    /// * `topic_name` - Topic name (e.g., "/chatter")
    ///
    /// # Type Parameters
    ///
    /// * `M` - Message type implementing RosMessage
    /// * `RX_BUF` - Receive buffer size (default: 1024)
    ///
    /// # Example
    ///
    /// ```ignore
    /// let mut subscriber = node.create_subscriber::<Int32>("/chatter")?;
    ///
    /// // In your polling loop:
    /// if let Some(msg) = subscriber.try_recv()? {
    ///     // process message
    /// }
    /// ```
    pub fn create_subscriber<M: RosMessage>(
        &mut self,
        topic_name: &str,
    ) -> Result<ShimNodeSubscriber<M, 1024>, ShimNodeError> {
        self.create_subscriber_sized::<M, 1024>(topic_name)
    }

    /// Create a subscriber with custom buffer size
    pub fn create_subscriber_sized<M: RosMessage, const RX_BUF: usize>(
        &mut self,
        topic_name: &str,
    ) -> Result<ShimNodeSubscriber<M, RX_BUF>, ShimNodeError> {
        self.create_subscriber_with_qos::<M, RX_BUF>(topic_name, QosSettings::default())
    }

    /// Create a subscriber with custom QoS settings and buffer size
    pub fn create_subscriber_with_qos<M: RosMessage, const RX_BUF: usize>(
        &mut self,
        topic_name: &str,
        qos: QosSettings,
    ) -> Result<ShimNodeSubscriber<M, RX_BUF>, ShimNodeError> {
        let topic =
            TopicInfo::new(topic_name, M::TYPE_NAME, M::TYPE_HASH).with_domain(self.domain_id);

        let subscriber = self.session.create_subscriber(&topic, qos)?;

        Ok(ShimNodeSubscriber {
            subscriber,
            buffer: [0u8; RX_BUF],
            _phantom: PhantomData,
        })
    }
}

// ============================================================================
// ShimNodePublisher
// ============================================================================

/// Publisher handle for a typed message
///
/// Created via `ShimNode::create_publisher()`.
pub struct ShimNodePublisher<M: RosMessage> {
    publisher: ShimPublisher,
    _phantom: PhantomData<M>,
}

impl<M: RosMessage> ShimNodePublisher<M> {
    /// Publish a message
    ///
    /// # Arguments
    ///
    /// * `msg` - Message to publish
    ///
    /// # Returns
    ///
    /// Ok(()) on success, error on failure
    pub fn publish(&self, msg: &M) -> Result<(), ShimNodeError> {
        self.publish_with_buffer::<1024>(msg)
    }

    /// Publish a message with custom buffer size
    pub fn publish_with_buffer<const BUF: usize>(&self, msg: &M) -> Result<(), ShimNodeError> {
        let mut buffer = [0u8; BUF];
        let mut writer =
            CdrWriter::new_with_header(&mut buffer).map_err(|_| ShimNodeError::BufferTooSmall)?;

        msg.serialize(&mut writer)
            .map_err(|_| ShimNodeError::Serialization)?;

        let len = writer.position();
        self.publisher
            .publish_raw(&buffer[..len])
            .map_err(|e| e.into())
    }

    /// Publish raw CDR-encoded data
    ///
    /// The data should already include the CDR header.
    pub fn publish_raw(&self, data: &[u8]) -> Result<(), ShimNodeError> {
        self.publisher.publish_raw(data).map_err(|e| e.into())
    }
}

// ============================================================================
// ShimNodeSubscriber
// ============================================================================

/// Subscriber handle for a typed message
///
/// Created via `ShimNode::create_subscriber()`.
pub struct ShimNodeSubscriber<M: RosMessage, const RX_BUF: usize = 1024> {
    subscriber: ShimSubscriber,
    buffer: [u8; RX_BUF],
    _phantom: PhantomData<M>,
}

impl<M: RosMessage, const RX_BUF: usize> ShimNodeSubscriber<M, RX_BUF> {
    /// Try to receive a message (non-blocking)
    ///
    /// # Returns
    ///
    /// - `Ok(Some(msg))` if a message is available
    /// - `Ok(None)` if no message is available
    /// - `Err(...)` on error
    pub fn try_recv(&mut self) -> Result<Option<M>, ShimNodeError> {
        use nano_ros_core::CdrReader;

        match self.subscriber.try_recv_raw(&mut self.buffer)? {
            Some(len) => {
                let mut reader = CdrReader::new_with_header(&self.buffer[..len])
                    .map_err(|_| ShimNodeError::Transport(TransportError::DeserializationError))?;

                let msg = M::deserialize(&mut reader)
                    .map_err(|_| ShimNodeError::Transport(TransportError::DeserializationError))?;

                Ok(Some(msg))
            }
            None => Ok(None),
        }
    }

    /// Try to receive raw CDR-encoded data (non-blocking)
    ///
    /// # Returns
    ///
    /// - `Ok(Some(len))` if data is available, with `len` bytes in the buffer
    /// - `Ok(None)` if no data is available
    /// - `Err(...)` on error
    pub fn try_recv_raw(&mut self) -> Result<Option<usize>, ShimNodeError> {
        self.subscriber
            .try_recv_raw(&mut self.buffer)
            .map_err(|e| e.into())
    }

    /// Get the receive buffer
    ///
    /// Use this after `try_recv_raw()` to access the raw data.
    pub fn buffer(&self) -> &[u8] {
        &self.buffer
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_conversion() {
        let transport_err = TransportError::ConnectionFailed;
        let node_err: ShimNodeError = transport_err.into();
        assert_eq!(
            node_err,
            ShimNodeError::Transport(TransportError::ConnectionFailed)
        );
    }
}
