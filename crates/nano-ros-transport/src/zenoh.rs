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
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::traits::{
    Publisher, QosSettings, Session, SessionMode, Subscriber, TopicInfo, Transport,
    TransportConfig, TransportError,
};

use zenoh_pico::{Config, KeyExpr, Sample, Session as ZenohPicoSession};

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
pub struct ZenohPublisher {
    publisher: zenoh_pico::Publisher,
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

        Ok(Self { publisher })
    }
}

impl Publisher for ZenohPublisher {
    type Error = TransportError;

    fn publish_raw(&self, data: &[u8]) -> Result<(), Self::Error> {
        self.publisher
            .put(data)
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
        // Generate the topic key
        let key: heapless::String<256> = topic.to_key();

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
