//! Zenoh transport backend
//!
//! Provides a transport backend using the safe zenoh-pico wrapper.
//! Requires the `zenoh` feature flag.
//!
//! # Executor Support
//!
//! This module supports different executor backends via feature flags:
//!
//! - Default: Background threads for read/lease tasks (requires OS support)
//! - `rtic` or `polling`: No background threads, manual polling required
//!
//! For RTIC applications, also enable `sync-critical-section` for proper mutex handling.
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

use crate::sync::Mutex;

use crate::traits::{
    Publisher, QosSettings, ServiceClientTrait, ServiceInfo, ServiceRequest, ServiceServerTrait,
    Session, SessionMode, Subscriber, TopicInfo, Transport, TransportConfig, TransportError,
};

use zenoh_pico::{
    serialize_rmw_attachment, Config, KeyExpr, LivelinessToken, Query, Queryable, Sample,
    Session as ZenohPicoSession, ZenohId, RMW_GID_SIZE,
};

/// RMW attachment data for rmw_zenoh
///
/// This metadata is attached to each published message and is required
/// for ROS 2 nodes using rmw_zenoh_cpp to receive messages.
///
/// The actual serialization is done using zenoh's serializer to ensure
/// compatibility with rmw_zenoh_cpp's deserializer.
#[derive(Debug, Clone, Copy)]
pub struct RmwAttachment {
    /// Message sequence number (incremented per publish)
    pub sequence_number: i64,
    /// Timestamp in nanoseconds
    pub timestamp: i64,
    /// RMW Global Identifier (random, generated once per publisher)
    /// 16 bytes for humble compatibility
    pub rmw_gid: [u8; RMW_GID_SIZE],
}

impl RmwAttachment {
    /// Size of the attachment in bytes (for reference only)
    pub const SIZE: usize = core::mem::size_of::<Self>();

    /// Create a new attachment with a random GID
    pub fn new() -> Self {
        Self {
            sequence_number: 0,
            timestamp: 0,
            rmw_gid: Self::generate_gid(),
        }
    }

    /// Generate a random GID
    fn generate_gid() -> [u8; RMW_GID_SIZE] {
        let mut gid = [0u8; RMW_GID_SIZE];
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
    ///
    /// This starts background threads for read and lease tasks.
    /// For RTIC or single-threaded executors, use `new_without_tasks()` instead.
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

    /// Create a new Zenoh session without starting background tasks
    ///
    /// Use this for RTIC or other single-threaded executors where you need
    /// manual control over when network I/O occurs.
    ///
    /// After opening, you must periodically call:
    /// - `poll_read()` to process incoming messages (recommended: every 10ms)
    /// - `send_keepalive()` to maintain the session (recommended: every 1s)
    ///
    /// Optionally, you can later call `start_tasks()` to switch to background threads.
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub fn new_without_tasks(config: &TransportConfig) -> Result<Self, TransportError> {
        let zconfig = match (&config.mode, config.locator) {
            (SessionMode::Client, Some(locator)) => {
                Config::client(locator).map_err(|_| TransportError::InvalidConfig)?
            }
            (SessionMode::Client, None) => {
                return Err(TransportError::InvalidConfig);
            }
            (SessionMode::Peer, _) => Config::peer().map_err(|_| TransportError::InvalidConfig)?,
        };

        let session = ZenohPicoSession::open_without_tasks(zconfig)
            .map_err(|_| TransportError::ConnectionFailed)?;

        Ok(Self { session })
    }

    /// Start background read and lease tasks
    ///
    /// This is called automatically by `new()`. Only call this manually if you
    /// used `new_without_tasks()` and later want to switch to background threads.
    ///
    /// Returns an error if tasks are already started.
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub fn start_tasks(&mut self) -> Result<(), TransportError> {
        self.session
            .start_tasks()
            .map_err(|_| TransportError::TaskStartFailed)
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
    pub fn poll_read(&mut self) -> Result<(), TransportError> {
        self.session
            .poll_read()
            .map_err(|_| TransportError::PollFailed)
    }

    /// Send keepalive to maintain the session
    ///
    /// Call this periodically (recommended: every 1s) when not using background tasks.
    /// This sends keepalive messages to prevent the session from timing out.
    ///
    /// For RTIC applications, call this from a periodic software task.
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub fn send_keepalive(&mut self) -> Result<(), TransportError> {
        self.session
            .send_keepalive()
            .map_err(|_| TransportError::KeepaliveFailed)
    }

    /// Send join message for peer discovery
    ///
    /// Call this periodically in peer mode to announce presence to other peers.
    /// Not needed in client mode.
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub fn send_join(&mut self) -> Result<(), TransportError> {
        self.session
            .send_join()
            .map_err(|_| TransportError::JoinFailed)
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

    /// Get a reference to the underlying zenoh-pico session
    pub fn inner(&self) -> &ZenohPicoSession {
        &self.session
    }

    /// Get a mutable reference to the underlying zenoh-pico session
    ///
    /// Use this for manual polling operations when background tasks are stopped.
    pub fn inner_mut(&mut self) -> &mut ZenohPicoSession {
        &mut self.session
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
    /// Format: `@ros2_lv/<domain_id>/<zid>/0/0/NN/%/%/<node_name>`
    pub fn node_keyexpr(domain_id: u32, zid: &ZenohId, node_name: &str) -> alloc::string::String {
        use alloc::format;
        format!(
            "@ros2_lv/{}/{}/0/0/NN/%/%/{}",
            domain_id,
            zid.to_hex_string(),
            node_name
        )
    }

    /// Build a publisher liveliness key expression
    ///
    /// Format: `@ros2_lv/<domain_id>/<zid>/0/11/MP/%/%/<node_name>/<topic>/<type>/RIHS01_<hash>/<qos>`
    ///
    /// QoS format: `reliability:durability:history,depth:deadline_sec,deadline_nsec:lifespan_sec,lifespan_nsec:liveliness,liveliness_sec,liveliness_nsec`
    /// Values: reliability=2 (BEST_EFFORT), durability=1 (VOLATILE), history=1 (KEEP_LAST), depth=1
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
        // QoS: BEST_EFFORT (2), VOLATILE (2), KEEP_LAST (1), depth=1
        format!(
            "@ros2_lv/{}/{}/0/11/MP/%/%/{}/{}/{}/RIHS01_{}/2:2:1,1:,:,:,,",
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
    /// Format: `@ros2_lv/<domain_id>/<zid>/0/11/MS/%/%/<node_name>/<topic>/<type>/RIHS01_<hash>/<qos>`
    ///
    /// QoS format: `reliability:durability:history,depth:deadline_sec,deadline_nsec:lifespan_sec,lifespan_nsec:liveliness,liveliness_sec,liveliness_nsec`
    /// Values: reliability=2 (BEST_EFFORT), durability=1 (VOLATILE), history=1 (KEEP_LAST), depth=1
    pub fn subscriber_keyexpr(
        domain_id: u32,
        zid: &ZenohId,
        node_name: &str,
        topic: &TopicInfo,
    ) -> alloc::string::String {
        use alloc::format;
        // Mangle topic name: replace slashes with percent signs
        let topic_mangled = topic.name.replace('/', "%");
        // QoS: BEST_EFFORT (2), VOLATILE (2), KEEP_LAST (1), depth=1
        format!(
            "@ros2_lv/{}/{}/0/11/MS/%/%/{}/{}/{}/RIHS01_{}/2:2:1,1:,:,:,,",
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
    type ServiceServerHandle = ZenohServiceServer;
    type ServiceClientHandle = ZenohServiceClient;

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

    fn create_service_server(
        &mut self,
        service: &ServiceInfo,
    ) -> Result<Self::ServiceServerHandle, Self::Error> {
        ZenohServiceServer::new(&self.session, service)
    }

    fn create_service_client(
        &mut self,
        service: &ServiceInfo,
    ) -> Result<Self::ServiceClientHandle, Self::Error> {
        ZenohServiceClient::new(&self.session, service)
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
    attachment: Mutex<RmwAttachment>,
}

impl ZenohPublisher {
    /// Create a new publisher for the given topic
    pub fn new(session: &ZenohPicoSession, topic: &TopicInfo) -> Result<Self, TransportError> {
        // Generate the topic key
        let key: heapless::String<256> = topic.to_key();

        #[cfg(feature = "log")]
        log::debug!("Publisher data keyexpr: {}", key.as_str());

        let keyexpr =
            KeyExpr::new(key.as_str()).map_err(|_| TransportError::PublisherCreationFailed)?;

        let publisher = session
            .declare_publisher(&keyexpr)
            .map_err(|_| TransportError::PublisherCreationFailed)?;

        Ok(Self {
            publisher,
            attachment: Mutex::new(RmwAttachment::new()),
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
        let (seq, ts, gid) = {
            let mut attachment = self.attachment.lock();
            attachment.sequence_number += 1;
            attachment.timestamp = Self::current_timestamp();
            (
                attachment.sequence_number,
                attachment.timestamp,
                attachment.rmw_gid,
            )
        };

        // Serialize attachment using zenoh serializer for rmw_zenoh compatibility
        let serialized_attachment = serialize_rmw_attachment(seq, ts, &gid)
            .map_err(|_| TransportError::SerializationError)?;

        // Publish with properly serialized attachment
        self.publisher
            .put_with_serialized_attachment(data, serialized_attachment)
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
    data: Mutex<Vec<u8>>,
    /// Flag indicating new data is available
    has_data: AtomicBool,
    /// Length of valid data
    len: AtomicUsize,
}

impl SubscriberBuffer {
    fn new() -> Self {
        Self {
            data: Mutex::new(Vec::with_capacity(1024)),
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

/// Shared buffer for service server callbacks
struct ServiceServerBuffer {
    /// Buffer for received request data
    data: Mutex<Vec<u8>>,
    /// Flag indicating new request is available
    has_request: AtomicBool,
    /// Length of valid data
    len: AtomicUsize,
    /// Sequence number from attachment
    sequence_number: AtomicI64,
    /// The keyexpr to reply on
    reply_keyexpr: Mutex<alloc::string::String>,
}

impl ServiceServerBuffer {
    fn new() -> Self {
        Self {
            data: Mutex::new(Vec::with_capacity(1024)),
            has_request: AtomicBool::new(false),
            len: AtomicUsize::new(0),
            sequence_number: AtomicI64::new(0),
            reply_keyexpr: Mutex::new(alloc::string::String::new()),
        }
    }

    fn store(&self, query: &Query) {
        let mut data = self.data.lock();
        data.clear();
        data.extend_from_slice(&query.payload);
        self.len.store(query.payload.len(), Ordering::Release);

        // Store reply keyexpr
        {
            let mut keyexpr = self.reply_keyexpr.lock();
            keyexpr.clear();
            keyexpr.push_str(&query.keyexpr);
        }

        // Parse sequence number from attachment if present
        // For now, use a counter since attachment parsing is complex
        static SEQ_COUNTER: AtomicI64 = AtomicI64::new(0);
        self.sequence_number.store(
            SEQ_COUNTER.fetch_add(1, Ordering::Relaxed),
            Ordering::Release,
        );

        self.has_request.store(true, Ordering::Release);
    }

    fn take(&self, buf: &mut [u8]) -> Option<(usize, i64)> {
        if !self.has_request.load(Ordering::Acquire) {
            return None;
        }

        let len = self.len.load(Ordering::Acquire);
        if len > buf.len() {
            return None; // Buffer too small
        }

        let data = self.data.lock();
        buf[..len].copy_from_slice(&data[..len]);
        drop(data);

        let seq = self.sequence_number.load(Ordering::Acquire);
        self.has_request.store(false, Ordering::Release);
        Some((len, seq))
    }
}

/// Type alias for the service server callback
type ServiceCallback = Box<dyn FnMut(Query) + Send + 'static>;

/// Zenoh service server using queryable
pub struct ZenohServiceServer {
    /// Keep the queryable alive
    _queryable: Queryable<ServiceCallback>,
    /// Shared buffer for received requests
    buffer: Arc<ServiceServerBuffer>,
    /// Key expression for replies (will be used when reply is implemented)
    #[allow(dead_code)]
    keyexpr: KeyExpr,
}

impl ZenohServiceServer {
    /// Create a new service server for the given service
    pub fn new(session: &ZenohPicoSession, service: &ServiceInfo) -> Result<Self, TransportError> {
        // Generate the service key (request key)
        let key: heapless::String<256> = service.to_key();

        #[cfg(feature = "log")]
        log::debug!("Service server keyexpr: {}", key.as_str());

        let keyexpr =
            KeyExpr::new(key.as_str()).map_err(|_| TransportError::ServiceServerCreationFailed)?;

        // Create shared buffer
        let buffer = Arc::new(ServiceServerBuffer::new());
        let buffer_clone = buffer.clone();

        // Create callback that stores query data in the shared buffer
        let callback: ServiceCallback = Box::new(move |query: Query| {
            buffer_clone.store(&query);
            // Note: The actual reply needs to be sent from send_reply
            // For now, store the query data and let send_reply handle it
        });

        let queryable = session
            .declare_queryable(&keyexpr, callback)
            .map_err(|_| TransportError::ServiceServerCreationFailed)?;

        Ok(Self {
            _queryable: queryable,
            buffer,
            keyexpr,
        })
    }
}

impl ServiceServerTrait for ZenohServiceServer {
    type Error = TransportError;

    fn try_recv_request<'a>(
        &mut self,
        buf: &'a mut [u8],
    ) -> Result<Option<ServiceRequest<'a>>, Self::Error> {
        match self.buffer.take(buf) {
            Some((len, seq)) => Ok(Some(ServiceRequest {
                data: &buf[..len],
                sequence_number: seq,
            })),
            None => Ok(None),
        }
    }

    fn send_reply(&mut self, _sequence_number: i64, _data: &[u8]) -> Result<(), Self::Error> {
        // TODO: Implement proper reply using zenoh query reply mechanism
        // This requires keeping the Query object alive or using a different approach
        // For now, return an error since the callback-based approach needs rework
        #[cfg(feature = "log")]
        log::warn!("Service reply not yet implemented in callback model");
        Err(TransportError::ServiceReplyFailed)
    }
}

/// Zenoh service client using z_get
pub struct ZenohServiceClient {
    /// Service key expression (will be used when call_raw is implemented)
    #[allow(dead_code)]
    keyexpr: KeyExpr,
    /// Session reference for making queries
    /// Note: This is unsafe because we store a raw pointer to the session
    /// The session must outlive the client
    #[allow(dead_code)]
    session_ptr: *const zenoh_pico::ffi::z_loaned_session_t,
}

impl ZenohServiceClient {
    /// Create a new service client for the given service
    pub fn new(session: &ZenohPicoSession, service: &ServiceInfo) -> Result<Self, TransportError> {
        // Generate the service key
        let key: heapless::String<256> = service.to_key();

        #[cfg(feature = "log")]
        log::debug!("Service client keyexpr: {}", key.as_str());

        let keyexpr =
            KeyExpr::new(key.as_str()).map_err(|_| TransportError::ServiceClientCreationFailed)?;

        // Store a reference to the session (this is safe as long as client doesn't outlive session)
        let session_ptr = session.loan();

        Ok(Self {
            keyexpr,
            session_ptr,
        })
    }
}

// Safety: ZenohServiceClient is Send as long as session outlives it
unsafe impl Send for ZenohServiceClient {}

impl ServiceClientTrait for ZenohServiceClient {
    type Error = TransportError;

    fn call_raw(&mut self, _request: &[u8], _reply_buf: &mut [u8]) -> Result<usize, Self::Error> {
        // TODO: Implement z_get call to send request and receive reply
        // This is complex because z_get is callback-based in zenoh-pico
        // Need to use z_closure_reply and wait for the reply
        #[cfg(feature = "log")]
        log::warn!("Service client call not yet implemented");
        Err(TransportError::ServiceRequestFailed)
    }
}

#[cfg(test)]
mod tests {
    // Tests are in tests/zenoh_integration.rs
}
