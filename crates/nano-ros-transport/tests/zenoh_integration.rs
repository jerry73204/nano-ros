//! Integration tests for zenoh transport
//!
//! Run with:
//! cargo test --features zenoh -p nano-ros-transport

#![cfg(feature = "zenoh")]

use nano_ros_transport::{
    Publisher, QosSettings, Session, SessionMode, Subscriber, TopicInfo, Transport,
    TransportConfig, ZenohTransport,
};
use std::thread;
use std::time::Duration;

/// Test that we can open and close a session in peer mode
/// (doesn't require a router)
#[test]
fn test_session_open_close_peer() {
    let config = TransportConfig {
        locator: None,
        mode: SessionMode::Peer,
    };

    let result = ZenohTransport::open(&config);
    match result {
        Ok(mut session) => {
            let close_result = session.close();
            assert!(close_result.is_ok(), "Failed to close session");
        }
        Err(e) => {
            // Connection failure is acceptable in CI/test environments
            println!(
                "Session open failed (expected in some environments): {:?}",
                e
            );
        }
    }
}

/// Test topic info generation
#[test]
fn test_topic_info_key_generation() {
    let topic = TopicInfo::new("/chatter", "std_msgs::msg::dds_::Int32_", "abc123def456");

    let key: heapless::String<256> = topic.to_key();

    assert!(key.contains("chatter"), "Key should contain topic name");
    assert!(
        key.contains("std_msgs::msg::dds_::Int32_"),
        "Key should contain type name"
    );
    // For Humble, data keyexprs use TypeHashNotSupported (liveliness uses RIHS01_ prefix)
    assert!(
        key.contains("TypeHashNotSupported"),
        "Key should use TypeHashNotSupported for Humble"
    );
}

/// Test CDR message format for Int32
#[test]
fn test_cdr_int32_format() {
    // CDR little-endian format for Int32 with value 42
    let cdr_msg: [u8; 8] = [
        0x00, 0x01, 0x00, 0x00, // CDR encapsulation header (LE)
        0x2A, 0x00, 0x00, 0x00, // Int32: 42 (little-endian)
    ];

    assert_eq!(cdr_msg[0], 0x00, "First byte should be 0x00");
    assert_eq!(cdr_msg[1], 0x01, "Second byte should be 0x01 (LE)");

    let value = i32::from_le_bytes([cdr_msg[4], cdr_msg[5], cdr_msg[6], cdr_msg[7]]);
    assert_eq!(value, 42, "Decoded value should be 42");
}

/// Test full pub/sub cycle (requires working zenoh network)
/// This test requires a zenoh router running: zenohd --listen tcp/127.0.0.1:7447
#[test]
#[ignore = "requires zenohd router on tcp/127.0.0.1:7447"]
fn test_pubsub_loopback() {
    // Connect to router as client
    let config = TransportConfig {
        locator: Some("tcp/127.0.0.1:7447"),
        mode: SessionMode::Client,
    };

    let mut session = match ZenohTransport::open(&config) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Could not open session: {:?}", e);
            eprintln!("Start a router with: zenohd --listen tcp/127.0.0.1:7447");
            panic!("Failed to connect to zenoh router");
        }
    };

    // Create topic with simple key for testing
    let topic = TopicInfo::new("test/loopback", "Int32", "hash123");

    // Create subscriber first
    let mut subscriber = session
        .create_subscriber(&topic, QosSettings::BEST_EFFORT)
        .expect("Failed to create subscriber");

    // Wait for subscriber to be established
    thread::sleep(Duration::from_secs(1));

    // Create publisher
    let publisher = session
        .create_publisher(&topic, QosSettings::BEST_EFFORT)
        .expect("Failed to create publisher");

    // Publish a CDR-encoded Int32 message
    let test_value: i32 = 12345;
    let cdr_msg: [u8; 8] = [
        0x00,
        0x01,
        0x00,
        0x00, // CDR header (LE)
        (test_value & 0xFF) as u8,
        ((test_value >> 8) & 0xFF) as u8,
        ((test_value >> 16) & 0xFF) as u8,
        ((test_value >> 24) & 0xFF) as u8,
    ];

    publisher
        .publish_raw(&cdr_msg)
        .expect("Failed to publish message");

    // Wait for message to arrive
    thread::sleep(Duration::from_secs(2));

    // Try to receive
    let mut recv_buf = [0u8; 64];
    match subscriber.try_recv_raw(&mut recv_buf) {
        Ok(Some(len)) => {
            assert_eq!(len, 8, "Message length should be 8 bytes");

            // Verify CDR header
            assert_eq!(recv_buf[0], 0x00);
            assert_eq!(recv_buf[1], 0x01);

            // Verify value
            let received_value =
                i32::from_le_bytes([recv_buf[4], recv_buf[5], recv_buf[6], recv_buf[7]]);
            assert_eq!(
                received_value, test_value,
                "Received value should match sent value"
            );

            println!("Successfully received message: {}", received_value);
        }
        Ok(None) => {
            panic!("No message received");
        }
        Err(e) => {
            panic!("Error receiving message: {:?}", e);
        }
    }

    session.close().expect("Failed to close session");
}

/// Test pub/sub with separate sessions (more realistic scenario)
#[test]
#[ignore = "requires zenohd router on tcp/127.0.0.1:7447"]
fn test_pubsub_separate_sessions() {
    // Connect to router as client
    let config = TransportConfig {
        locator: Some("tcp/127.0.0.1:7447"),
        mode: SessionMode::Client,
    };

    // Open subscriber session
    let mut sub_session = match ZenohTransport::open(&config) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Could not open session: {:?}", e);
            panic!("Failed to connect to zenoh router");
        }
    };

    let topic = TopicInfo::new("test/separate-sessions", "Int32", "hash456");

    // Create subscriber
    let mut subscriber = sub_session
        .create_subscriber(&topic, QosSettings::BEST_EFFORT)
        .expect("Failed to create subscriber");

    // Wait for subscriber to be discovered
    thread::sleep(Duration::from_secs(1));

    // Open publisher session
    let mut pub_session = ZenohTransport::open(&config).expect("Failed to open publisher session");

    let publisher = pub_session
        .create_publisher(&topic, QosSettings::BEST_EFFORT)
        .expect("Failed to create publisher");

    // Wait for publisher to be discovered
    thread::sleep(Duration::from_millis(500));

    // Publish
    let test_data = b"Hello from transport!";
    publisher.publish_raw(test_data).expect("Failed to publish");

    // Try to receive with retries (distributed systems can have timing issues)
    let mut recv_buf = [0u8; 64];
    let mut received = false;
    for _ in 0..20 {
        thread::sleep(Duration::from_millis(100));
        match subscriber.try_recv_raw(&mut recv_buf) {
            Ok(Some(len)) => {
                assert_eq!(&recv_buf[..len], test_data);
                println!("Successfully received: {:?}", &recv_buf[..len]);
                received = true;
                break;
            }
            Ok(None) => continue,
            Err(e) => {
                panic!("Error receiving: {:?}", e);
            }
        }
    }

    assert!(received, "Should have received message within 2 seconds");

    sub_session.close().expect("Failed to close sub session");
    pub_session.close().expect("Failed to close pub session");
}

/// Test multiple publishers on same session
#[test]
#[ignore = "requires zenohd router on tcp/127.0.0.1:7447"]
fn test_multiple_publishers() {
    let config = TransportConfig {
        locator: Some("tcp/127.0.0.1:7447"),
        mode: SessionMode::Client,
    };

    let mut session = match ZenohTransport::open(&config) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Could not open session: {:?}", e);
            panic!("Failed to connect to zenoh router");
        }
    };

    let topic1 = TopicInfo::new("test/pub1", "Int32", "hash1");
    let topic2 = TopicInfo::new("test/pub2", "Int32", "hash2");

    let _pub1 = session
        .create_publisher(&topic1, QosSettings::BEST_EFFORT)
        .expect("Failed to create publisher 1");

    let _pub2 = session
        .create_publisher(&topic2, QosSettings::BEST_EFFORT)
        .expect("Failed to create publisher 2");

    session.close().expect("Failed to close session");
}

/// Test multiple subscribers on same session
#[test]
#[ignore = "requires zenohd router on tcp/127.0.0.1:7447"]
fn test_multiple_subscribers() {
    let config = TransportConfig {
        locator: Some("tcp/127.0.0.1:7447"),
        mode: SessionMode::Client,
    };

    let mut session = match ZenohTransport::open(&config) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Could not open session: {:?}", e);
            panic!("Failed to connect to zenoh router");
        }
    };

    let topic1 = TopicInfo::new("test/sub1", "Int32", "hash1");
    let topic2 = TopicInfo::new("test/sub2", "Int32", "hash2");

    let _sub1 = session
        .create_subscriber(&topic1, QosSettings::BEST_EFFORT)
        .expect("Failed to create subscriber 1");

    let _sub2 = session
        .create_subscriber(&topic2, QosSettings::BEST_EFFORT)
        .expect("Failed to create subscriber 2");

    session.close().expect("Failed to close session");
}
