//! Publisher and subscriber options with fluent API
//!
//! This module provides rclrs-style options types for creating publishers
//! and subscribers with a fluent API pattern.

use nano_ros_transport::QosSettings;

/// Options for creating a publisher
///
/// # Examples
///
/// ```ignore
/// // Use string directly (uses default QoS)
/// let pub = node.create_publisher::<Int32>("/topic")?;
///
/// // Use fluent builder
/// let pub = node.create_publisher::<Int32>(
///     "/topic".keep_last(10).reliable()
/// )?;
/// ```
#[derive(Debug, Clone)]
pub struct PublisherOptions<'a> {
    /// Topic name
    pub topic: &'a str,
    /// QoS settings
    pub qos: QosSettings,
}

impl<'a> PublisherOptions<'a> {
    /// Create new publisher options with the given topic and default QoS
    pub fn new(topic: &'a str) -> Self {
        Self {
            topic,
            qos: QosSettings::default(),
        }
    }

    /// Set history to keep last N messages
    pub const fn keep_last(mut self, depth: u32) -> Self {
        self.qos = self.qos.keep_last(depth);
        self
    }

    /// Set history to keep all messages
    pub const fn keep_all(mut self) -> Self {
        self.qos = self.qos.keep_all();
        self
    }

    /// Set reliability to reliable
    pub const fn reliable(mut self) -> Self {
        self.qos = self.qos.reliable();
        self
    }

    /// Set reliability to best-effort
    pub const fn best_effort(mut self) -> Self {
        self.qos = self.qos.best_effort();
        self
    }

    /// Set durability to volatile
    pub const fn volatile(mut self) -> Self {
        self.qos = self.qos.volatile();
        self
    }

    /// Set durability to transient local
    pub const fn transient_local(mut self) -> Self {
        self.qos = self.qos.transient_local();
        self
    }
}

/// Trait for types that can be converted into PublisherOptions
pub trait IntoPublisherOptions<'a> {
    /// Convert into PublisherOptions
    fn into_publisher_options(self) -> PublisherOptions<'a>;
}

impl<'a> IntoPublisherOptions<'a> for &'a str {
    fn into_publisher_options(self) -> PublisherOptions<'a> {
        PublisherOptions::new(self)
    }
}

impl<'a> IntoPublisherOptions<'a> for PublisherOptions<'a> {
    fn into_publisher_options(self) -> PublisherOptions<'a> {
        self
    }
}

// Note: No extension trait on &str for publishers to avoid conflicts with subscribers.
// Users should use PublisherOptions::new("topic").keep_last(10) instead.

/// Options for creating a subscriber
///
/// # Examples
///
/// ```ignore
/// // Use string directly (uses default QoS)
/// let sub = node.create_subscription::<Int32>("/topic", callback)?;
///
/// // Use fluent builder
/// let sub = node.create_subscription::<Int32>(
///     "/topic".keep_last(10).reliable(),
///     callback
/// )?;
/// ```
#[derive(Debug, Clone)]
pub struct SubscriberOptions<'a> {
    /// Topic name
    pub topic: &'a str,
    /// QoS settings
    pub qos: QosSettings,
}

impl<'a> SubscriberOptions<'a> {
    /// Create new subscriber options with the given topic and default QoS
    pub fn new(topic: &'a str) -> Self {
        Self {
            topic,
            qos: QosSettings::default(),
        }
    }

    /// Set history to keep last N messages
    pub const fn keep_last(mut self, depth: u32) -> Self {
        self.qos = self.qos.keep_last(depth);
        self
    }

    /// Set history to keep all messages
    pub const fn keep_all(mut self) -> Self {
        self.qos = self.qos.keep_all();
        self
    }

    /// Set reliability to reliable
    pub const fn reliable(mut self) -> Self {
        self.qos = self.qos.reliable();
        self
    }

    /// Set reliability to best-effort
    pub const fn best_effort(mut self) -> Self {
        self.qos = self.qos.best_effort();
        self
    }

    /// Set durability to volatile
    pub const fn volatile(mut self) -> Self {
        self.qos = self.qos.volatile();
        self
    }

    /// Set durability to transient local
    pub const fn transient_local(mut self) -> Self {
        self.qos = self.qos.transient_local();
        self
    }
}

/// Trait for types that can be converted into SubscriberOptions
pub trait IntoSubscriberOptions<'a> {
    /// Convert into SubscriberOptions
    fn into_subscriber_options(self) -> SubscriberOptions<'a>;
}

impl<'a> IntoSubscriberOptions<'a> for &'a str {
    fn into_subscriber_options(self) -> SubscriberOptions<'a> {
        SubscriberOptions::new(self)
    }
}

impl<'a> IntoSubscriberOptions<'a> for SubscriberOptions<'a> {
    fn into_subscriber_options(self) -> SubscriberOptions<'a> {
        self
    }
}

// Note: No extension trait on &str for subscribers to avoid conflicts with publishers.
// Users should use SubscriberOptions::new("topic").keep_last(10) instead.

#[cfg(test)]
mod tests {
    use super::*;
    use nano_ros_transport::{QosDurabilityPolicy, QosHistoryPolicy, QosReliabilityPolicy};

    #[test]
    fn test_publisher_options() {
        let options = PublisherOptions::new("/test");
        assert_eq!(options.topic, "/test");
        assert_eq!(options.qos.reliability, QosReliabilityPolicy::Reliable);
    }

    #[test]

    fn test_publisher_builder() {
        let options = PublisherOptions::new("/test").keep_last(5).reliable();

        assert_eq!(options.topic, "/test");

        assert_eq!(options.qos.reliability, QosReliabilityPolicy::Reliable);

        assert_eq!(options.qos.history, QosHistoryPolicy::KeepLast);

        assert_eq!(options.qos.depth, 5);
    }

    #[test]

    fn test_publisher_fluent_api() {
        let options = PublisherOptions::new("/test").keep_last(10).reliable();

        assert_eq!(options.topic, "/test");

        assert_eq!(options.qos.reliability, QosReliabilityPolicy::Reliable);

        assert_eq!(options.qos.history, QosHistoryPolicy::KeepLast);

        assert_eq!(options.qos.depth, 10);
    }

    #[test]

    fn test_subscriber_options() {
        let options = SubscriberOptions::new("/test");

        assert_eq!(options.topic, "/test");

        assert_eq!(options.qos.reliability, QosReliabilityPolicy::Reliable);
    }

    #[test]

    fn test_subscriber_builder() {
        let options = SubscriberOptions::new("/test")
            .keep_last(5)
            .reliable()
            .transient_local();

        assert_eq!(options.topic, "/test");

        assert_eq!(options.qos.reliability, QosReliabilityPolicy::Reliable);

        assert_eq!(options.qos.history, QosHistoryPolicy::KeepLast);

        assert_eq!(options.qos.depth, 5);

        assert_eq!(options.qos.durability, QosDurabilityPolicy::TransientLocal);
    }

    #[test]
    fn test_subscriber_fluent_api() {
        let options = SubscriberOptions::new("/test").keep_last(10).reliable();
        assert_eq!(options.topic, "/test");
        assert_eq!(options.qos.reliability, QosReliabilityPolicy::Reliable);
    }
}
