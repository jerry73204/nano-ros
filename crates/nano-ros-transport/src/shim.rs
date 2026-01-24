//! Shim transport backend
//!
//! Provides a transport backend using the zenoh-pico-shim wrapper.
//! This is designed for embedded platforms that need a simpler API than
//! the full zenoh-pico bindings.
//!
//! Requires the `shim` feature flag.
//!
//! # Limitations
//!
//! Compared to the full `zenoh` transport, this shim transport:
//! - Does not support liveliness tokens (no ROS 2 discovery)
//! - Does not support services (not yet implemented in shim)
//! - Does not include RMW attachments (simplified protocol)
//! - Requires manual polling (no background threads)
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
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::traits::{
    Publisher, QosSettings, ServiceClientTrait, ServiceInfo, ServiceRequest, ServiceServerTrait,
    Session, SessionMode, Subscriber, TopicInfo, Transport, TransportConfig, TransportError,
};

use zenoh_pico_shim::{ShimContext, ShimError};

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
        _service: &ServiceInfo,
    ) -> Result<Self::ServiceServerHandle, Self::Error> {
        // Services not yet supported in shim
        Err(TransportError::ServiceServerCreationFailed)
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
pub struct ShimPublisher {
    publisher: zenoh_pico_shim::ShimPublisher<'static>,
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

        Ok(Self { publisher })
    }
}

impl Publisher for ShimPublisher {
    type Error = TransportError;

    fn publish_raw(&self, data: &[u8]) -> Result<(), Self::Error> {
        self.publisher.publish(data).map_err(TransportError::from)
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
// Service Stubs (not yet implemented)
// ============================================================================

/// Shim service server (not yet supported)
pub struct ShimServiceServer {
    _private: PhantomData<()>,
}

impl ServiceServerTrait for ShimServiceServer {
    type Error = TransportError;

    fn try_recv_request<'a>(
        &mut self,
        _buf: &'a mut [u8],
    ) -> Result<Option<ServiceRequest<'a>>, Self::Error> {
        // Services not yet supported
        Err(TransportError::ServiceServerCreationFailed)
    }

    fn send_reply(&mut self, _sequence_number: i64, _data: &[u8]) -> Result<(), Self::Error> {
        Err(TransportError::ServiceReplyFailed)
    }
}

/// Shim service client (not yet supported)
pub struct ShimServiceClient {
    _private: PhantomData<()>,
}

impl ServiceClientTrait for ShimServiceClient {
    type Error = TransportError;

    fn call_raw(&mut self, _request: &[u8], _reply_buf: &mut [u8]) -> Result<usize, Self::Error> {
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
}
