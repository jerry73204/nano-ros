//! Integration tests for RTIC support in nano-ros-transport
//!
//! These tests verify that the RTIC-specific features work correctly:
//! - Mutex synchronization primitives
//! - QoS settings types

#![cfg(feature = "std")]

/// Test that Mutex type is available and usable
#[test]
fn test_mutex_available() {
    use nano_ros_transport::sync::Mutex;

    let mutex = Mutex::new(42);
    let guard = mutex.lock();
    assert_eq!(*guard, 42);
}

/// Test Mutex with() closure API if available
#[test]
#[cfg(feature = "sync-critical-section")]
fn test_mutex_with_api() {
    use nano_ros_transport::sync::Mutex;

    let mutex = Mutex::new(0);
    mutex.with(|val| {
        *val = 42;
    });
    let result = mutex.with(|val| *val);
    assert_eq!(result, 42);
}

/// Test QoS settings creation
#[test]
fn test_qos_settings() {
    use nano_ros_transport::QosSettings;

    // Default QoS - derive(Default) gives 0 for history_depth
    let qos = QosSettings::default();
    assert!(!qos.reliable);
    assert_eq!(qos.history_depth, 0);

    // Reliable QoS constant
    let reliable_qos = QosSettings::RELIABLE;
    assert!(reliable_qos.reliable);

    // Best effort QoS constant
    let best_effort_qos = QosSettings::BEST_EFFORT;
    assert!(!best_effort_qos.reliable);
}

/// Test TopicInfo creation
#[test]
fn test_topic_info() {
    use nano_ros_transport::TopicInfo;

    let topic = TopicInfo::new(
        "/chatter",
        "std_msgs::msg::dds_::String_",
        "TypeHashNotSupported",
    );

    assert_eq!(topic.name, "/chatter");
    assert!(topic.type_name.contains("String_"));
    assert_eq!(topic.type_hash, "TypeHashNotSupported");
    assert_eq!(topic.domain_id, 0);
}

/// Test TopicInfo with custom domain
#[test]
fn test_topic_info_with_domain() {
    use nano_ros_transport::TopicInfo;

    let topic = TopicInfo::new(
        "/chatter",
        "std_msgs::msg::dds_::String_",
        "TypeHashNotSupported",
    )
    .with_domain(42);

    assert_eq!(topic.domain_id, 42);
}

/// Test ServiceInfo creation
#[test]
fn test_service_info() {
    use nano_ros_transport::ServiceInfo;

    let service = ServiceInfo::new(
        "/add_two_ints",
        "example_interfaces::srv::AddTwoInts",
        "abc123",
    );

    assert_eq!(service.name, "/add_two_ints");
    assert!(service.type_name.contains("AddTwoInts"));
    assert_eq!(service.domain_id, 0);
}

/// Test TransportConfig defaults
#[test]
fn test_transport_config() {
    use nano_ros_transport::{SessionMode, TransportConfig};

    let config = TransportConfig {
        locator: Some("tcp/127.0.0.1:7447"),
        mode: SessionMode::Client,
    };

    assert!(config.locator.unwrap().contains("7447"));
    match config.mode {
        SessionMode::Client => {}
        _ => panic!("Expected client mode"),
    }
}

/// Test SessionMode enum
#[test]
fn test_session_mode() {
    use nano_ros_transport::SessionMode;

    let client = SessionMode::Client;
    let peer = SessionMode::Peer;

    // Just verify these can be created
    match client {
        SessionMode::Client => {}
        SessionMode::Peer => panic!("Expected client"),
    }
    match peer {
        SessionMode::Client => panic!("Expected peer"),
        SessionMode::Peer => {}
    }
}
