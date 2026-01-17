//! Zenoh transport backend
//!
//! Provides a transport backend using the safe zenoh-pico wrapper.
//! Requires the `zenoh` feature flag.
//!
//! # Example
//!
//! ```no_run
//! use nano_ros_transport::{ZenohTransport, Transport, TransportConfig, SessionMode};
//!
//! // Create config
//! let config = TransportConfig {
//!     locator: Some("tcp/192.168.1.1:7447"),
//!     mode: SessionMode::Client,
//! };
//!
//! // Open session
//! let mut session = ZenohTransport::open(&config).expect("Failed to open session");
//! ```

extern crate alloc;
use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicI64, AtomicUsize, Ordering};

use crate::traits::{
    Publisher, QosSettings, Session, SessionMode, Subscriber, TopicInfo, Transport,
    TransportConfig, TransportError,
};

use zenoh_pico::{Config, KeyExpr, LivelinessToken, Sample, Session as ZenohPicoSession, ZenohId};

/// RMW attachment structure required by rmw_zenoh
///
/// This metadata is attached to each published message and is required
/// for ROS 2 nodes using rmw_zenoh_cpp to receive messages.
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct RmwAttachment {
    /// Message sequence number (incremented per publish)
    pub sequence_number: i64,
    /// Timestamp in nanoseconds
    pub timestamp: i64,
    /// Size of RMW GID (always 16)
    pub rmw_gid_size: u8,
    /// RMW Global Identifier (random, generated once per publisher)
    pub rmw_gid: [u8; 16],
}

impl RmwAttachment {
    /// Size of the attachment in bytes
    pub const SIZE: usize = core::mem::size_of::<Self>();

    /// Create a new attachment with a random GID
    pub fn new() -> Self {
        Self {
            sequence_number: 0,
            timestamp: 0,
            rmw_gid_size: 16,
            rmw_gid: Self::generate_gid(),
        }
    }

    /// Generate a random GID using zenoh's random function
    fn generate_gid() -> [u8; 16] {
        let mut gid = [0u8; 16];
        // Use a simple pseudo-random based on memory address and counter
        // In production, this should use zenoh's z_random_u8() or a proper RNG
        static COUNTER: AtomicI64 = AtomicI64::new(0);
        let seed = COUNTER.fetch_add(1, Ordering::Relaxed) as u64;
        let addr = &gid as *const _ as u64;
        let mixed = seed.wrapping_mul(0x517cc1b727220a95) ^ addr;

        for (i, byte) in gid.iter_mut().enumerate() {
            let shift = (i % 8) * 8;
            *byte = ((mixed.wrapping_mul((i as u64).wrapping_add(1))) >> shift) as u8;
        }
        gid
    }

    /// Convert to bytes for transmission
    pub fn as_bytes(&self) -> &[u8] {
        // SAFETY: RmwAttachment is repr(C, packed) with known size
        unsafe { core::slice::from_raw_parts(self as *const Self as *const u8, Self::SIZE) }
    }
}

impl Default for RmwAttachment {
    fn default() -> Self {
        Self::new()
    }
}

/// Zenoh transport backend
pub struct ZenohTransport;

impl Transport for ZenohTransport {
    type Error = TransportError;
    type Session = ZenohSession;

    fn open(config: &TransportConfig) -> Result<Self::Session, Self::Error> {
        ZenohSession::new(config)
    }
}

/// Zenoh session wrapping the safe zenoh-pico Session
pub struct ZenohSession {
    session: ZenohPicoSession,
}

impl ZenohSession {
    /// Create a new Zenoh session with the given configuration
    pub fn new(config: &TransportConfig) -> Result<Self, TransportError> {
        let zconfig = match (&config.mode, config.locator) {
            (SessionMode::Client, Some(locator)) => {
                Config::client(locator).map_err(|_| TransportError::InvalidConfig)?
            }
            (SessionMode::Client, None) => {
                // Client mode requires a locator
                return Err(TransportError::InvalidConfig);
            }
            (SessionMode::Peer, _) => Config::peer().map_err(|_| TransportError::InvalidConfig)?,
        };

        let session =
            ZenohPicoSession::open(zconfig).map_err(|_| TransportError::ConnectionFailed)?;

        Ok(Self { session })
    }

    /// Get a reference to the underlying zenoh-pico session
    pub fn inner(&self) -> &ZenohPicoSession {
        &self.session
    }

    /// Get the session's Zenoh ID
    pub fn zid(&self) -> ZenohId {
        self.session.zid()
    }

    /// Declare a liveliness token for ROS 2 discovery
    ///
    /// This creates a liveliness token at the given key expression,
    /// allowing ROS 2 nodes using rmw_zenoh to discover this entity.
    pub fn declare_liveliness(&self, keyexpr: &str) -> Result<LivelinessToken, TransportError> {
        let ke = KeyExpr::new(keyexpr).map_err(|_| TransportError::InvalidConfig)?;
        self.session
            .declare_liveliness(&ke)
            .map_err(|_| TransportError::ConnectionFailed)
    }
}

/// ROS 2 liveliness key expression builder
///
/// Generates the key expressions required for ROS 2 discovery via rmw_zenoh.
pub struct Ros2Liveliness;

impl Ros2Liveliness {
    /// Build a node liveliness key expression
    ///
    /// Format: `@ros2_lv/<domain_id>/<zid>/0/0/NN/%%/%%/<node_name>`
    pub fn node_keyexpr(domain_id: u32, zid: &ZenohId, node_name: &str) -> alloc::string::String {
        use alloc::format;
        format!(
            "@ros2_lv/{}/{}/0/0/NN/%%/%%/{}",
            domain_id,
            zid.to_hex_string(),
            node_name
        )
    }

    /// Build a publisher liveliness key expression
    ///
    /// Format: `@ros2_lv/<domain_id>/<zid>/0/11/MP/%/%/<node_name>/<topic>/<type>/RIHS01_<hash>/::,:,:,:,,`
    pub fn publisher_keyexpr(
        domain_id: u32,
        zid: &ZenohId,
        node_name: &str,
        topic: &TopicInfo,
    ) -> alloc::string::String {
        use alloc::format;
        // Mangle topic name: replace slashes with percent signs
        // rmw_zenoh's mangle_name replaces '/' with '%'
        // e.g., "/chatter" -> "%chatter"
        let topic_mangled = topic.name.replace('/', "%");
        format!(
            "@ros2_lv/{}/{}/0/11/MP/%/%/{}/{}/{}/RIHS01_{}/::,:,:,:,,",
            domain_id,
            zid.to_hex_string(),
            node_name,
            topic_mangled,
            topic.type_name,
            topic.type_hash
        )
    }

    /// Build a subscriber liveliness key expression
    ///
    /// Format: `@ros2_lv/<domain_id>/<zid>/0/11/MS/%/%/<node_name>/<topic>/<type>/RIHS01_<hash>/::,:,:,:,,`
    pub fn subscriber_keyexpr(
        domain_id: u32,
        zid: &ZenohId,
        node_name: &str,
        topic: &TopicInfo,
    ) -> alloc::string::String {
        use alloc::format;
        // Mangle topic name: replace slashes with percent signs
        let topic_mangled = topic.name.replace('/', "%");
        format!(
            "@ros2_lv/{}/{}/0/11/MS/%/%/{}/{}/{}/RIHS01_{}/::,:,:,:,,",
            domain_id,
            zid.to_hex_string(),
            node_name,
            topic_mangled,
            topic.type_name,
            topic.type_hash
        )
    }
}

impl Session for ZenohSession {
    type Error = TransportError;
    type PublisherHandle = ZenohPublisher;
    type SubscriberHandle = ZenohSubscriber;

    fn create_publisher(
        &mut self,
        topic: &TopicInfo,
        _qos: QosSettings,
    ) -> Result<Self::PublisherHandle, Self::Error> {
        ZenohPublisher::new(&self.session, topic)
    }

    fn create_subscriber(
        &mut self,
        topic: &TopicInfo,
        _qos: QosSettings,
    ) -> Result<Self::SubscriberHandle, Self::Error> {
        ZenohSubscriber::new(&self.session, topic)
    }

    fn close(&mut self) -> Result<(), Self::Error> {
        // Session will be closed on drop
        // We can't call close() here because we don't own it by value
        // The session is closed when ZenohSession is dropped
        Ok(())
    }
}

/// Zenoh publisher wrapping the safe zenoh-pico Publisher
///
/// Includes RMW attachment support for rmw_zenoh compatibility.
pub struct ZenohPublisher {
    publisher: zenoh_pico::Publisher,
    /// RMW attachment with GID and sequence counter
    attachment: spin::Mutex<RmwAttachment>,
}

impl ZenohPublisher {
    /// Create a new publisher for the given topic
    pub fn new(session: &ZenohPicoSession, topic: &TopicInfo) -> Result<Self, TransportError> {
        // Generate the topic key
        let key: heapless::String<256> = topic.to_key();

        let keyexpr =
            KeyExpr::new(key.as_str()).map_err(|_| TransportError::PublisherCreationFailed)?;

        let publisher = session
            .declare_publisher(&keyexpr)
            .map_err(|_| TransportError::PublisherCreationFailed)?;

        Ok(Self {
            publisher,
            attachment: spin::Mutex::new(RmwAttachment::new()),
        })
    }

    /// Get current timestamp in nanoseconds
    fn current_timestamp() -> i64 {
        // For now, use a simple counter as timestamp
        // In production, this should use actual time from the platform
        static TIMESTAMP: AtomicI64 = AtomicI64::new(0);
        TIMESTAMP.fetch_add(1_000_000, Ordering::Relaxed) // Increment by 1ms equivalent
    }
}

impl Publisher for ZenohPublisher {
    type Error = TransportError;

    fn publish_raw(&self, data: &[u8]) -> Result<(), Self::Error> {
        // Update attachment with new sequence number and timestamp
        let attachment_bytes = {
            let mut attachment = self.attachment.lock();
            attachment.sequence_number += 1;
            attachment.timestamp = Self::current_timestamp();
            // Copy to local buffer while lock is held
            let mut buf = [0u8; RmwAttachment::SIZE];
            buf.copy_from_slice(attachment.as_bytes());
            buf
        };

        // Publish with attachment for rmw_zenoh compatibility
        self.publisher
            .put_with_attachment(data, &attachment_bytes)
            .map_err(|_| TransportError::PublishFailed)
    }

    fn buffer_error(&self) -> Self::Error {
        TransportError::BufferTooSmall
    }

    fn serialization_error(&self) -> Self::Error {
        TransportError::SerializationError
    }
}

/// Shared buffer for subscriber callbacks
struct SubscriberBuffer {
    /// Buffer for received data
    data: spin::Mutex<Vec<u8>>,
    /// Flag indicating new data is available
    has_data: AtomicBool,
    /// Length of valid data
    len: AtomicUsize,
}

impl SubscriberBuffer {
    fn new() -> Self {
        Self {
            data: spin::Mutex::new(Vec::with_capacity(1024)),
            has_data: AtomicBool::new(false),
            len: AtomicUsize::new(0),
        }
    }

    fn store(&self, sample: &Sample) {
        let mut data = self.data.lock();
        data.clear();
        data.extend_from_slice(&sample.payload);
        self.len.store(sample.payload.len(), Ordering::Release);
        self.has_data.store(true, Ordering::Release);
    }

    fn take(&self, buf: &mut [u8]) -> Option<usize> {
        if !self.has_data.load(Ordering::Acquire) {
            return None;
        }

        let len = self.len.load(Ordering::Acquire);
        if len > buf.len() {
            return None; // Buffer too small
        }

        let data = self.data.lock();
        buf[..len].copy_from_slice(&data[..len]);
        drop(data);

        self.has_data.store(false, Ordering::Release);
        Some(len)
    }
}

/// Type alias for the subscriber callback
type SubscriberCallback = Box<dyn FnMut(Sample) + Send + 'static>;

/// Zenoh subscriber wrapping the safe zenoh-pico Subscriber
pub struct ZenohSubscriber {
    /// Keep the subscriber alive
    _subscriber: zenoh_pico::Subscriber<SubscriberCallback>,
    /// Shared buffer for received data
    buffer: Arc<SubscriberBuffer>,
}

impl ZenohSubscriber {
    /// Create a new subscriber for the given topic
    pub fn new(session: &ZenohPicoSession, topic: &TopicInfo) -> Result<Self, TransportError> {
        // Generate the topic key with wildcard for type hash
        // This allows receiving messages from ROS 2 nodes with different type hashes
        let key: heapless::String<256> = topic.to_key_wildcard();

        let keyexpr =
            KeyExpr::new(key.as_str()).map_err(|_| TransportError::SubscriberCreationFailed)?;

        // Create shared buffer
        let buffer = Arc::new(SubscriberBuffer::new());
        let buffer_clone = buffer.clone();

        // Create callback that stores data in the shared buffer
        // Coerce to trait object to match the type alias
        let callback: SubscriberCallback = Box::new(move |sample: Sample| {
            buffer_clone.store(&sample);
        });

        let subscriber = session
            .declare_subscriber(&keyexpr, callback)
            .map_err(|_| TransportError::SubscriberCreationFailed)?;

        Ok(Self {
            _subscriber: subscriber,
            buffer,
        })
    }
}

impl Subscriber for ZenohSubscriber {
    type Error = TransportError;

    fn try_recv_raw(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Self::Error> {
        match self.buffer.take(buf) {
            Some(len) => Ok(Some(len)),
            None => Ok(None),
        }
    }

    fn deserialization_error(&self) -> Self::Error {
        TransportError::DeserializationError
    }
}

#[cfg(test)]
mod tests {
    // Tests are in tests/zenoh_integration.rs
}
