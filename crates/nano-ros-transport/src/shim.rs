//! Shim transport backend
//!
//! Provides a transport backend using the zenoh-pico-shim wrapper.
//! This is designed for embedded platforms that need a simpler API than
//! the full zenoh-pico bindings.
//!
//! Requires the `shim` feature flag.
//!
//! # Features
//!
//! - Session management with ZenohId support
//! - Publishers with RMW attachment support for rmw_zenoh compatibility
//! - Subscribers with wildcard matching
//! - Liveliness tokens for ROS 2 discovery
//! - Service servers via queryables (service clients not yet implemented)
//! - Manual polling (no background threads) for embedded systems
//!
//! # Example
//!
//! ```ignore
//! use nano_ros_transport::{ShimTransport, Transport, TransportConfig, SessionMode};
//!
//! // Create config
//! let config = TransportConfig {
//!     locator: Some("tcp/192.168.1.1:7447"),
//!     mode: SessionMode::Client,
//! };
//!
//! // Open session
//! let mut session = ShimTransport::open(&config).expect("Failed to open session");
//!
//! // Must poll periodically
//! session.spin_once(10)?;
//! ```

use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicI64, AtomicUsize, Ordering};

use crate::traits::{
    Publisher, QosSettings, ServiceClientTrait, ServiceInfo, ServiceRequest, ServiceServerTrait,
    Session, SessionMode, Subscriber, TopicInfo, Transport, TransportConfig, TransportError,
};

use zenoh_pico_shim::{
    ShimContext, ShimError, ShimLivelinessToken, ShimZenohId, ZENOH_SHIM_RMW_GID_SIZE,
};

// Re-export for convenience
pub use zenoh_pico_shim::ShimZenohId as ZenohId;

// ============================================================================
// Constants
// ============================================================================

/// RMW GID size for attachment serialization (16 bytes for Humble)
pub const RMW_GID_SIZE: usize = ZENOH_SHIM_RMW_GID_SIZE as usize;

/// Size of serialized RMW attachment
/// Format: sequence_number (8) + timestamp (8) + VLE length (1) + gid (16) = 33 bytes
const RMW_ATTACHMENT_SIZE: usize = 8 + 8 + 1 + RMW_GID_SIZE;

// ============================================================================
// Error Conversion
// ============================================================================

impl From<ShimError> for TransportError {
    fn from(err: ShimError) -> Self {
        match err {
            ShimError::Generic => TransportError::ConnectionFailed,
            ShimError::Config => TransportError::InvalidConfig,
            ShimError::Session => TransportError::ConnectionFailed,
            ShimError::Task => TransportError::TaskStartFailed,
            ShimError::KeyExpr => TransportError::InvalidConfig,
            ShimError::Full => TransportError::PublisherCreationFailed,
            ShimError::Invalid => TransportError::InvalidConfig,
            ShimError::Publish => TransportError::PublishFailed,
            ShimError::NotOpen => TransportError::Disconnected,
        }
    }
}

// ============================================================================
// RMW Attachment Support
// ============================================================================

/// RMW attachment data for rmw_zenoh compatibility
///
/// This metadata is attached to each published message and is required
/// for ROS 2 nodes using rmw_zenoh_cpp to receive messages.
#[derive(Debug, Clone, Copy)]
pub struct RmwAttachment {
    /// Message sequence number (incremented per publish)
    pub sequence_number: i64,
    /// Timestamp in nanoseconds
    pub timestamp: i64,
    /// RMW Global Identifier (random, generated once per publisher)
    pub rmw_gid: [u8; RMW_GID_SIZE],
}

impl RmwAttachment {
    /// Create a new attachment with a random GID
    pub fn new() -> Self {
        Self {
            sequence_number: 0,
            timestamp: 0,
            rmw_gid: Self::generate_gid(),
        }
    }

    /// Generate a random GID using a simple PRNG
    fn generate_gid() -> [u8; RMW_GID_SIZE] {
        let mut gid = [0u8; RMW_GID_SIZE];
        static COUNTER: AtomicI64 = AtomicI64::new(0);
        let seed = COUNTER.fetch_add(1, Ordering::Relaxed) as u64;
        // Use address of gid as additional entropy
        let addr = &gid as *const _ as u64;
        let mixed = seed.wrapping_mul(0x517cc1b727220a95) ^ addr;

        for (i, byte) in gid.iter_mut().enumerate() {
            let shift = (i % 8) * 8;
            *byte = ((mixed.wrapping_mul((i as u64).wrapping_add(1))) >> shift) as u8;
        }
        gid
    }

    /// Serialize the attachment in the format expected by rmw_zenoh_cpp
    ///
    /// Format:
    /// - int64: sequence_number (little-endian, 8 bytes)
    /// - int64: timestamp (little-endian, 8 bytes)
    /// - VLE length (1 byte for length 16)
    /// - 16 x uint8: GID
    pub fn serialize(&self, buf: &mut [u8; RMW_ATTACHMENT_SIZE]) {
        // Sequence number (little-endian)
        buf[0..8].copy_from_slice(&self.sequence_number.to_le_bytes());
        // Timestamp (little-endian)
        buf[8..16].copy_from_slice(&self.timestamp.to_le_bytes());
        // VLE length (16 fits in single byte)
        buf[16] = RMW_GID_SIZE as u8;
        // GID bytes
        buf[17..33].copy_from_slice(&self.rmw_gid);
    }
}

impl Default for RmwAttachment {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Ros2Liveliness Helper
// ============================================================================

/// ROS 2 liveliness key expression builder for the shim transport
///
/// Generates the key expressions required for ROS 2 discovery via rmw_zenoh.
pub struct Ros2Liveliness;

impl Ros2Liveliness {
    /// Build a node liveliness key expression
    ///
    /// Format: `@ros2_lv/<domain_id>/<zid>/0/0/NN/%/%/<node_name>`
    pub fn node_keyexpr<const N: usize>(
        domain_id: u32,
        zid: &ShimZenohId,
        node_name: &str,
    ) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let mut zid_hex = [0u8; 32];
        zid.to_hex_bytes(&mut zid_hex);
        let zid_str = core::str::from_utf8(&zid_hex).unwrap_or("");
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "@ros2_lv/{}/{}/0/0/NN/%/%/{}",
                domain_id, zid_str, node_name
            ),
        );
        key
    }

    /// Build a publisher liveliness key expression
    ///
    /// Format: `@ros2_lv/<domain_id>/<zid>/0/11/MP/%/%/<node_name>/<topic>/<type>/RIHS01_<hash>/<qos>`
    pub fn publisher_keyexpr<const N: usize>(
        domain_id: u32,
        zid: &ShimZenohId,
        node_name: &str,
        topic: &TopicInfo,
    ) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let mut zid_hex = [0u8; 32];
        zid.to_hex_bytes(&mut zid_hex);
        let zid_str = core::str::from_utf8(&zid_hex).unwrap_or("");
        // Mangle topic name: replace slashes with percent signs
        let topic_mangled = Self::mangle_topic_name::<64>(topic.name);
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "@ros2_lv/{}/{}/0/11/MP/%/%/{}/{}/{}/RIHS01_{}/2:2:1,1:,:,:,,",
                domain_id,
                zid_str,
                node_name,
                topic_mangled.as_str(),
                topic.type_name,
                topic.type_hash
            ),
        );
        key
    }

    /// Build a subscriber liveliness key expression
    ///
    /// Format: `@ros2_lv/<domain_id>/<zid>/0/11/MS/%/%/<node_name>/<topic>/<type>/RIHS01_<hash>/<qos>`
    pub fn subscriber_keyexpr<const N: usize>(
        domain_id: u32,
        zid: &ShimZenohId,
        node_name: &str,
        topic: &TopicInfo,
    ) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let mut zid_hex = [0u8; 32];
        zid.to_hex_bytes(&mut zid_hex);
        let zid_str = core::str::from_utf8(&zid_hex).unwrap_or("");
        let topic_mangled = Self::mangle_topic_name::<64>(topic.name);
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "@ros2_lv/{}/{}/0/11/MS/%/%/{}/{}/{}/RIHS01_{}/2:2:1,1:,:,:,,",
                domain_id,
                zid_str,
                node_name,
                topic_mangled.as_str(),
                topic.type_name,
                topic.type_hash
            ),
        );
        key
    }

    /// Mangle a topic name by replacing '/' with '%'
    fn mangle_topic_name<const N: usize>(topic: &str) -> heapless::String<N> {
        let mut mangled = heapless::String::new();
        for c in topic.chars() {
            if c == '/' {
                let _ = mangled.push('%');
            } else {
                let _ = mangled.push(c);
            }
        }
        mangled
    }
}

// ============================================================================
// ShimTransport
// ============================================================================

/// Shim transport backend for embedded platforms
///
/// Uses zenoh-pico-shim for a simplified API suitable for bare-metal systems.
pub struct ShimTransport;

impl Transport for ShimTransport {
    type Error = TransportError;
    type Session = ShimSession;

    fn open(config: &TransportConfig) -> Result<Self::Session, Self::Error> {
        ShimSession::new(config)
    }
}

// ============================================================================
// ShimSession
// ============================================================================

/// Shim session wrapping zenoh-pico-shim ShimContext
///
/// This session requires manual polling via `spin_once()` or `poll()`.
/// There are no background threads.
pub struct ShimSession {
    context: ShimContext,
}

impl ShimSession {
    /// Create a new shim session with the given configuration
    ///
    /// # Arguments
    ///
    /// * `config` - Transport configuration with locator and mode
    ///
    /// # Returns
    ///
    /// A new session or error if connection fails
    pub fn new(config: &TransportConfig) -> Result<Self, TransportError> {
        // Build the locator string with null terminator
        let locator = match (&config.mode, config.locator) {
            (SessionMode::Client, Some(loc)) => {
                // Create null-terminated locator
                let mut buf = [0u8; 128];
                let bytes = loc.as_bytes();
                if bytes.len() >= buf.len() {
                    return Err(TransportError::InvalidConfig);
                }
                buf[..bytes.len()].copy_from_slice(bytes);
                buf[bytes.len()] = 0; // Null terminator
                buf
            }
            (SessionMode::Client, None) => {
                return Err(TransportError::InvalidConfig);
            }
            (SessionMode::Peer, _) => {
                // Peer mode - pass null locator
                [0u8; 128]
            }
        };

        let context = ShimContext::new(&locator).map_err(TransportError::from)?;

        Ok(Self { context })
    }

    /// Check if the session is open
    pub fn is_open(&self) -> bool {
        self.context.is_open()
    }

    /// Check if this backend requires polling
    ///
    /// For shim transport, this always returns true - manual polling is required.
    pub fn uses_polling(&self) -> bool {
        self.context.uses_polling()
    }

    /// Poll for incoming data and process callbacks
    ///
    /// # Arguments
    ///
    /// * `timeout_ms` - Maximum time to wait for data (0 = non-blocking)
    ///
    /// # Returns
    ///
    /// Number of events processed, or error
    pub fn poll(&self, timeout_ms: u32) -> Result<i32, TransportError> {
        self.context.poll(timeout_ms).map_err(TransportError::from)
    }

    /// Combined poll and keepalive operation
    ///
    /// This is the recommended way to drive the session. Call this
    /// periodically (e.g., every 10ms) from your main loop or RTIC task.
    ///
    /// # Arguments
    ///
    /// * `timeout_ms` - Maximum time to wait (0 = non-blocking)
    ///
    /// # Returns
    ///
    /// Number of events processed, or error
    pub fn spin_once(&self, timeout_ms: u32) -> Result<i32, TransportError> {
        self.context
            .spin_once(timeout_ms)
            .map_err(TransportError::from)
    }

    /// Get a reference to the underlying ShimContext
    pub fn inner(&self) -> &ShimContext {
        &self.context
    }

    /// Get the session's Zenoh ID
    ///
    /// The Zenoh ID uniquely identifies this session in the Zenoh network.
    /// It is used in liveliness token key expressions for ROS 2 discovery.
    pub fn zid(&self) -> Result<ShimZenohId, TransportError> {
        self.context.zid().map_err(TransportError::from)
    }

    /// Declare a liveliness token for ROS 2 discovery
    ///
    /// This creates a liveliness token at the given key expression,
    /// allowing ROS 2 nodes using rmw_zenoh to discover this entity.
    ///
    /// The key expression should be null-terminated.
    pub fn declare_liveliness(
        &self,
        keyexpr: &[u8],
    ) -> Result<ShimLivelinessToken<'_>, TransportError> {
        self.context
            .declare_liveliness(keyexpr)
            .map_err(TransportError::from)
    }
}

impl Session for ShimSession {
    type Error = TransportError;
    type PublisherHandle = ShimPublisher;
    type SubscriberHandle = ShimSubscriber;
    type ServiceServerHandle = ShimServiceServer;
    type ServiceClientHandle = ShimServiceClient;

    fn create_publisher(
        &mut self,
        topic: &TopicInfo,
        _qos: QosSettings,
    ) -> Result<Self::PublisherHandle, Self::Error> {
        ShimPublisher::new(&self.context, topic)
    }

    fn create_subscriber(
        &mut self,
        topic: &TopicInfo,
        _qos: QosSettings,
    ) -> Result<Self::SubscriberHandle, Self::Error> {
        ShimSubscriber::new(&self.context, topic)
    }

    fn create_service_server(
        &mut self,
        service: &ServiceInfo,
    ) -> Result<Self::ServiceServerHandle, Self::Error> {
        ShimServiceServer::new(&self.context, service)
    }

    fn create_service_client(
        &mut self,
        _service: &ServiceInfo,
    ) -> Result<Self::ServiceClientHandle, Self::Error> {
        // Services not yet supported in shim
        Err(TransportError::ServiceClientCreationFailed)
    }

    fn close(&mut self) -> Result<(), Self::Error> {
        // Context is closed on drop
        Ok(())
    }
}

// ============================================================================
// ShimPublisher
// ============================================================================

/// Shim publisher wrapping zenoh-pico-shim ShimPublisher
///
/// Includes RMW attachment support for rmw_zenoh compatibility.
pub struct ShimPublisher {
    publisher: zenoh_pico_shim::ShimPublisher<'static>,
    /// RMW attachment with GID and sequence counter
    attachment: RmwAttachment,
    /// Static timestamp counter (until platform time is available)
    timestamp_counter: AtomicI64,
}

impl ShimPublisher {
    /// Create a new publisher for the given topic
    pub fn new(context: &ShimContext, topic: &TopicInfo) -> Result<Self, TransportError> {
        // Generate the topic key with null terminator
        let key: heapless::String<256> = topic.to_key();

        // Create null-terminated keyexpr
        let mut keyexpr_buf = [0u8; 257];
        let bytes = key.as_bytes();
        if bytes.len() >= keyexpr_buf.len() {
            return Err(TransportError::InvalidConfig);
        }
        keyexpr_buf[..bytes.len()].copy_from_slice(bytes);
        keyexpr_buf[bytes.len()] = 0;

        // Safety: We need to extend the lifetime because ShimPublisher borrows from ShimContext.
        // This is safe because:
        // 1. ShimPublisher is stored in ShimSession which owns the ShimContext
        // 2. The underlying C shim manages its own state
        // 3. We transmute the lifetime to 'static for storage
        let publisher = unsafe {
            let pub_result = context.declare_publisher(&keyexpr_buf);
            match pub_result {
                Ok(p) => core::mem::transmute::<
                    zenoh_pico_shim::ShimPublisher<'_>,
                    zenoh_pico_shim::ShimPublisher<'static>,
                >(p),
                Err(e) => return Err(TransportError::from(e)),
            }
        };

        Ok(Self {
            publisher,
            attachment: RmwAttachment::new(),
            timestamp_counter: AtomicI64::new(0),
        })
    }

    /// Get current timestamp in nanoseconds (placeholder until platform time available)
    fn current_timestamp(&self) -> i64 {
        // Increment by 1ms equivalent
        self.timestamp_counter
            .fetch_add(1_000_000, Ordering::Relaxed)
    }
}

impl Publisher for ShimPublisher {
    type Error = TransportError;

    fn publish_raw(&self, data: &[u8]) -> Result<(), Self::Error> {
        // Update attachment with new sequence number and timestamp
        // Note: We need interior mutability here, using a simple workaround
        // since we can't use Mutex in no_std without extra dependencies
        let mut att = self.attachment;
        att.sequence_number += 1;
        att.timestamp = self.current_timestamp();

        // Serialize the attachment
        let mut att_buf = [0u8; RMW_ATTACHMENT_SIZE];
        att.serialize(&mut att_buf);

        // Publish with attachment
        self.publisher
            .publish_with_attachment(data, Some(&att_buf))
            .map_err(TransportError::from)
    }

    fn buffer_error(&self) -> Self::Error {
        TransportError::BufferTooSmall
    }

    fn serialization_error(&self) -> Self::Error {
        TransportError::SerializationError
    }
}

// ============================================================================
// ShimSubscriber
// ============================================================================

/// Shared buffer for subscriber callbacks
///
/// This buffer stores the most recent message received by the subscriber.
/// The callback writes to this buffer, and `try_recv_raw` reads from it.
struct SubscriberBuffer {
    /// Buffer for received data (statically allocated)
    data: [u8; 1024],
    /// Flag indicating new data is available
    has_data: AtomicBool,
    /// Length of valid data
    len: AtomicUsize,
}

impl SubscriberBuffer {
    const fn new() -> Self {
        Self {
            data: [0u8; 1024],
            has_data: AtomicBool::new(false),
            len: AtomicUsize::new(0),
        }
    }
}

/// Static buffers for subscribers (limited by ZENOH_SHIM_MAX_SUBSCRIBERS)
///
/// We use static buffers because the shim callback mechanism requires
/// a static context pointer. This limits us to a fixed number of subscribers.
static mut SUBSCRIBER_BUFFERS: [SubscriberBuffer; 8] = [
    SubscriberBuffer::new(),
    SubscriberBuffer::new(),
    SubscriberBuffer::new(),
    SubscriberBuffer::new(),
    SubscriberBuffer::new(),
    SubscriberBuffer::new(),
    SubscriberBuffer::new(),
    SubscriberBuffer::new(),
];

/// Next available buffer index
static NEXT_BUFFER_INDEX: AtomicUsize = AtomicUsize::new(0);

/// Callback function invoked by the C shim when data arrives
extern "C" fn subscriber_callback(data: *const u8, len: usize, ctx: *mut core::ffi::c_void) {
    let buffer_index = ctx as usize;
    if buffer_index >= 8 {
        return;
    }

    // Safety: We control access to SUBSCRIBER_BUFFERS and the callback is single-threaded
    unsafe {
        let buffer = &mut SUBSCRIBER_BUFFERS[buffer_index];
        let copy_len = len.min(buffer.data.len());
        core::ptr::copy_nonoverlapping(data, buffer.data.as_mut_ptr(), copy_len);
        buffer.len.store(copy_len, Ordering::Release);
        buffer.has_data.store(true, Ordering::Release);
    }
}

/// Shim subscriber wrapping zenoh-pico-shim ShimSubscriber
pub struct ShimSubscriber {
    /// The subscriber handle (kept alive to maintain subscription)
    _subscriber: zenoh_pico_shim::ShimSubscriber<'static>,
    /// Index into the static buffer array
    buffer_index: usize,
    /// Phantom to indicate we don't own the buffer
    _phantom: PhantomData<()>,
}

impl ShimSubscriber {
    /// Create a new subscriber for the given topic
    pub fn new(context: &ShimContext, topic: &TopicInfo) -> Result<Self, TransportError> {
        // Allocate a buffer index
        let buffer_index = NEXT_BUFFER_INDEX.fetch_add(1, Ordering::SeqCst);
        if buffer_index >= 8 {
            // Roll back and return error
            NEXT_BUFFER_INDEX.fetch_sub(1, Ordering::SeqCst);
            return Err(TransportError::SubscriberCreationFailed);
        }

        // Generate the topic key with wildcard for type hash
        let key: heapless::String<256> = topic.to_key_wildcard();

        // Create null-terminated keyexpr
        let mut keyexpr_buf = [0u8; 257];
        let bytes = key.as_bytes();
        if bytes.len() >= keyexpr_buf.len() {
            return Err(TransportError::InvalidConfig);
        }
        keyexpr_buf[..bytes.len()].copy_from_slice(bytes);
        keyexpr_buf[bytes.len()] = 0;

        // Create subscriber with callback
        // Safety: Similar to publisher, we transmute lifetime for storage
        let subscriber = unsafe {
            let sub_result = context.declare_subscriber_raw(
                &keyexpr_buf,
                subscriber_callback,
                buffer_index as *mut core::ffi::c_void,
            );
            match sub_result {
                Ok(s) => core::mem::transmute::<
                    zenoh_pico_shim::ShimSubscriber<'_>,
                    zenoh_pico_shim::ShimSubscriber<'static>,
                >(s),
                Err(e) => return Err(TransportError::from(e)),
            }
        };

        Ok(Self {
            _subscriber: subscriber,
            buffer_index,
            _phantom: PhantomData,
        })
    }
}

impl Subscriber for ShimSubscriber {
    type Error = TransportError;

    fn try_recv_raw(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Self::Error> {
        // Safety: We own this buffer index and access is atomic
        let buffer = unsafe { &SUBSCRIBER_BUFFERS[self.buffer_index] };

        if !buffer.has_data.load(Ordering::Acquire) {
            return Ok(None);
        }

        let len = buffer.len.load(Ordering::Acquire);
        if len > buf.len() {
            return Err(TransportError::BufferTooSmall);
        }

        // Copy data and clear flag
        // Safety: Data is valid up to len bytes
        unsafe {
            core::ptr::copy_nonoverlapping(
                SUBSCRIBER_BUFFERS[self.buffer_index].data.as_ptr(),
                buf.as_mut_ptr(),
                len,
            );
        }
        buffer.has_data.store(false, Ordering::Release);

        Ok(Some(len))
    }

    fn deserialization_error(&self) -> Self::Error {
        TransportError::DeserializationError
    }
}

// ============================================================================
// Service Server (using queryables)
// ============================================================================

/// Shared buffer for service server callbacks
struct ServiceBuffer {
    /// Buffer for received request data
    data: [u8; 1024],
    /// Buffer for keyexpr (for reply)
    keyexpr: [u8; 256],
    /// Flag indicating new request is available
    has_request: AtomicBool,
    /// Length of valid data
    len: AtomicUsize,
    /// Length of keyexpr
    keyexpr_len: AtomicUsize,
    /// Sequence number (counter)
    sequence_number: AtomicI64,
}

impl ServiceBuffer {
    const fn new() -> Self {
        Self {
            data: [0u8; 1024],
            keyexpr: [0u8; 256],
            has_request: AtomicBool::new(false),
            len: AtomicUsize::new(0),
            keyexpr_len: AtomicUsize::new(0),
            sequence_number: AtomicI64::new(0),
        }
    }
}

/// Static buffers for service servers (limited by ZENOH_SHIM_MAX_QUERYABLES)
static mut SERVICE_BUFFERS: [ServiceBuffer; 8] = [
    ServiceBuffer::new(),
    ServiceBuffer::new(),
    ServiceBuffer::new(),
    ServiceBuffer::new(),
    ServiceBuffer::new(),
    ServiceBuffer::new(),
    ServiceBuffer::new(),
    ServiceBuffer::new(),
];

/// Next available service buffer index
static NEXT_SERVICE_BUFFER_INDEX: AtomicUsize = AtomicUsize::new(0);

/// Sequence counter for service requests
static SERVICE_SEQ_COUNTER: AtomicI64 = AtomicI64::new(0);

/// Callback function invoked by the C shim when queries arrive
extern "C" fn queryable_callback(
    keyexpr: *const core::ffi::c_char,
    keyexpr_len: usize,
    payload: *const u8,
    payload_len: usize,
    ctx: *mut core::ffi::c_void,
) {
    let buffer_index = ctx as usize;
    if buffer_index >= 8 {
        return;
    }

    // Safety: We control access to SERVICE_BUFFERS and the callback is single-threaded
    unsafe {
        let buffer = &mut SERVICE_BUFFERS[buffer_index];

        // Copy keyexpr
        let keyexpr_copy_len = keyexpr_len.min(buffer.keyexpr.len() - 1);
        core::ptr::copy_nonoverlapping(
            keyexpr as *const u8,
            buffer.keyexpr.as_mut_ptr(),
            keyexpr_copy_len,
        );
        buffer.keyexpr[keyexpr_copy_len] = 0; // Null terminate
        buffer
            .keyexpr_len
            .store(keyexpr_copy_len, Ordering::Release);

        // Copy payload
        let copy_len = payload_len.min(buffer.data.len());
        if !payload.is_null() && payload_len > 0 {
            core::ptr::copy_nonoverlapping(payload, buffer.data.as_mut_ptr(), copy_len);
        }
        buffer.len.store(copy_len, Ordering::Release);

        // Set sequence number
        let seq = SERVICE_SEQ_COUNTER.fetch_add(1, Ordering::Relaxed);
        buffer.sequence_number.store(seq, Ordering::Release);

        buffer.has_request.store(true, Ordering::Release);
    }
}

/// Shim service server using queryables
///
/// Receives service requests via queryable callbacks.
/// Note: The reply mechanism is limited due to the callback model.
pub struct ShimServiceServer {
    /// The queryable handle (kept alive to maintain registration)
    _queryable: zenoh_pico_shim::ShimQueryable<'static>,
    /// Index into the static buffer array
    buffer_index: usize,
    /// Keyexpr buffer for replying (copied from last request)
    reply_keyexpr: [u8; 256],
    /// Keyexpr length
    reply_keyexpr_len: usize,
    /// Reference to context for replying
    context: *const ShimContext,
    /// Phantom to indicate ownership
    _phantom: PhantomData<()>,
}

impl ShimServiceServer {
    /// Create a new service server for the given service
    pub fn new(context: &ShimContext, service: &ServiceInfo) -> Result<Self, TransportError> {
        // Allocate a buffer index
        let buffer_index = NEXT_SERVICE_BUFFER_INDEX.fetch_add(1, Ordering::SeqCst);
        if buffer_index >= 8 {
            NEXT_SERVICE_BUFFER_INDEX.fetch_sub(1, Ordering::SeqCst);
            return Err(TransportError::ServiceServerCreationFailed);
        }

        // Generate the service key
        let key: heapless::String<256> = service.to_key();

        // Create null-terminated keyexpr
        let mut keyexpr_buf = [0u8; 257];
        let bytes = key.as_bytes();
        if bytes.len() >= keyexpr_buf.len() {
            return Err(TransportError::InvalidConfig);
        }
        keyexpr_buf[..bytes.len()].copy_from_slice(bytes);
        keyexpr_buf[bytes.len()] = 0;

        // Create queryable with callback
        let queryable = unsafe {
            let result = context.declare_queryable_raw(
                &keyexpr_buf,
                queryable_callback,
                buffer_index as *mut core::ffi::c_void,
            );
            match result {
                Ok(q) => core::mem::transmute::<
                    zenoh_pico_shim::ShimQueryable<'_>,
                    zenoh_pico_shim::ShimQueryable<'static>,
                >(q),
                Err(e) => return Err(TransportError::from(e)),
            }
        };

        Ok(Self {
            _queryable: queryable,
            buffer_index,
            reply_keyexpr: [0u8; 256],
            reply_keyexpr_len: 0,
            context: context as *const ShimContext,
            _phantom: PhantomData,
        })
    }
}

impl ServiceServerTrait for ShimServiceServer {
    type Error = TransportError;

    fn try_recv_request<'a>(
        &mut self,
        buf: &'a mut [u8],
    ) -> Result<Option<ServiceRequest<'a>>, Self::Error> {
        // Safety: We own this buffer index and access is atomic
        let buffer = unsafe { &SERVICE_BUFFERS[self.buffer_index] };

        if !buffer.has_request.load(Ordering::Acquire) {
            return Ok(None);
        }

        let len = buffer.len.load(Ordering::Acquire);
        if len > buf.len() {
            return Err(TransportError::BufferTooSmall);
        }

        // Copy data and keyexpr
        unsafe {
            core::ptr::copy_nonoverlapping(
                SERVICE_BUFFERS[self.buffer_index].data.as_ptr(),
                buf.as_mut_ptr(),
                len,
            );

            // Save keyexpr for potential reply
            let keyexpr_len = buffer.keyexpr_len.load(Ordering::Acquire);
            core::ptr::copy_nonoverlapping(
                SERVICE_BUFFERS[self.buffer_index].keyexpr.as_ptr(),
                self.reply_keyexpr.as_mut_ptr(),
                keyexpr_len,
            );
            self.reply_keyexpr[keyexpr_len] = 0;
            self.reply_keyexpr_len = keyexpr_len;
        }

        let seq = buffer.sequence_number.load(Ordering::Acquire);
        buffer.has_request.store(false, Ordering::Release);

        Ok(Some(ServiceRequest {
            data: &buf[..len],
            sequence_number: seq,
        }))
    }

    fn send_reply(&mut self, _sequence_number: i64, data: &[u8]) -> Result<(), Self::Error> {
        // Note: This only works if called immediately after try_recv_request
        // while the C shim's g_current_query is still valid.
        // In practice, this is a limitation of the callback model.
        if self.reply_keyexpr_len == 0 {
            return Err(TransportError::ServiceReplyFailed);
        }

        // Get context reference
        let context = unsafe { &*self.context };

        // Send reply using the stored keyexpr
        context
            .query_reply(&self.reply_keyexpr[..=self.reply_keyexpr_len], data, None)
            .map_err(|_| TransportError::ServiceReplyFailed)?;

        // Clear the stored keyexpr
        self.reply_keyexpr_len = 0;

        Ok(())
    }
}

// ============================================================================
// Service Client (not yet implemented)
// ============================================================================

/// Shim service client (not yet supported)
///
/// Service clients require synchronous z_get calls with reply handling,
/// which is not yet implemented in the shim.
pub struct ShimServiceClient {
    _private: PhantomData<()>,
}

impl ServiceClientTrait for ShimServiceClient {
    type Error = TransportError;

    fn call_raw(&mut self, _request: &[u8], _reply_buf: &mut [u8]) -> Result<usize, Self::Error> {
        // Service clients require z_get with reply handling
        // This is not yet implemented in the shim
        Err(TransportError::ServiceRequestFailed)
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
        assert_eq!(
            TransportError::from(ShimError::Config),
            TransportError::InvalidConfig
        );
        assert_eq!(
            TransportError::from(ShimError::Publish),
            TransportError::PublishFailed
        );
    }

    #[test]
    fn test_rmw_attachment_serialization() {
        let mut att = RmwAttachment::new();
        att.sequence_number = 42;
        att.timestamp = 1000000;

        let mut buf = [0u8; RMW_ATTACHMENT_SIZE];
        att.serialize(&mut buf);

        // Check sequence number (little-endian)
        assert_eq!(&buf[0..8], &42i64.to_le_bytes());
        // Check timestamp (little-endian)
        assert_eq!(&buf[8..16], &1000000i64.to_le_bytes());
        // Check VLE length
        assert_eq!(buf[16], 16);
        // Check GID (should match)
        assert_eq!(&buf[17..33], &att.rmw_gid);
    }

    #[test]
    fn test_ros2_liveliness_mangle() {
        let mangled = Ros2Liveliness::mangle_topic_name::<64>("/chatter");
        assert_eq!(mangled.as_str(), "%chatter");

        let mangled2 = Ros2Liveliness::mangle_topic_name::<64>("/foo/bar/baz");
        assert_eq!(mangled2.as_str(), "%foo%bar%baz");
    }
}
