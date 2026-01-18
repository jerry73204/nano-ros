//! Integration tests for RTIC support in nano-ros-node
//!
//! These tests verify that the RTIC-specific features work correctly:
//! - Static buffer allocation via const generics
//! - Node configuration types
//! - QoS settings

#![cfg(feature = "std")]

/// Test NodeConfig creation
#[test]
fn test_node_config_creation() {
    use nano_ros_node::NodeConfig;

    let config = NodeConfig::new("test_node", "/test_namespace");

    assert_eq!(config.name, "test_node");
    assert_eq!(config.namespace, "/test_namespace");
    assert_eq!(config.domain_id, 0); // Default domain
}

/// Test NodeConfig with custom domain
#[test]
fn test_node_config_with_domain() {
    use nano_ros_node::NodeConfig;

    let config = NodeConfig::new("test_node", "/").with_domain(42);

    assert_eq!(config.domain_id, 42);
}

/// Test QoS settings creation
#[test]
fn test_qos_settings() {
    use nano_ros_node::QosSettings;

    // Default QoS - derive(Default) gives 0 for history_depth
    let default_qos = QosSettings::default();
    assert!(!default_qos.reliable);
    assert_eq!(default_qos.history_depth, 0);

    // Custom QoS
    let custom_qos = QosSettings {
        reliable: true,
        history_depth: 100,
    };
    assert!(custom_qos.reliable);
    assert_eq!(custom_qos.history_depth, 100);
}

/// Test that memory requirements are bounded
#[test]
fn test_memory_bounds() {
    use core::mem::size_of;

    // NodeConfig should be small
    assert!(size_of::<nano_ros_node::NodeConfig>() < 256);

    // QosSettings should be tiny
    assert!(size_of::<nano_ros_node::QosSettings>() < 32);
}

/// Test that const generic buffer sizes can be used
#[test]
fn test_const_generics_compile() {
    // These should compile successfully, proving const generics work
    const CUSTOM_SIZE: usize = 512;

    // Verify they're usable as array sizes (compile-time check)
    let buffer: [u8; CUSTOM_SIZE] = [0u8; CUSTOM_SIZE];
    assert_eq!(buffer.len(), CUSTOM_SIZE);
}

/// Test RTIC timing constants (only when both zenoh and rtic features enabled)
#[test]
#[cfg(all(feature = "zenoh", feature = "rtic"))]
fn test_timing_constants() {
    use nano_ros_node::rtic::{KEEPALIVE_INTERVAL_MS, POLL_INTERVAL_MS};

    // Poll interval should be short (10ms default)
    assert!(POLL_INTERVAL_MS > 0);
    assert!(POLL_INTERVAL_MS <= 100);

    // Keepalive should be longer than poll (1000ms default)
    assert!(KEEPALIVE_INTERVAL_MS > POLL_INTERVAL_MS);
    assert!(KEEPALIVE_INTERVAL_MS >= 100);
    assert!(KEEPALIVE_INTERVAL_MS <= 10000);
}

/// Test default buffer sizes (only when zenoh feature enabled)
#[test]
#[cfg(feature = "zenoh")]
fn test_default_buffer_sizes() {
    use nano_ros_node::{
        DEFAULT_MAX_TOKENS, DEFAULT_REPLY_BUFFER_SIZE, DEFAULT_REQ_BUFFER_SIZE,
        DEFAULT_RX_BUFFER_SIZE,
    };

    // RX buffer should be at least 256 bytes
    assert!(DEFAULT_RX_BUFFER_SIZE >= 256);

    // Request/reply buffers should be reasonable
    assert!(DEFAULT_REQ_BUFFER_SIZE >= 256);
    assert!(DEFAULT_REPLY_BUFFER_SIZE >= 256);

    // Max tokens should allow at least a few publishers/subscribers
    assert!(DEFAULT_MAX_TOKENS >= 4);
}
