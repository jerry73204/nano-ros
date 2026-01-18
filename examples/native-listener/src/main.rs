//! Native Listener Example
//!
//! Demonstrates subscribing to messages using nano-ros on native x86.
//!
//! # Without zenoh feature (simulation mode):
//! ```bash
//! cargo run -p native-listener
//! ```
//!
//! # With zenoh feature (real transport):
//! ```bash
//! # Start zenoh router first:
//! zenohd --listen tcp/127.0.0.1:7447
//!
//! # Then run the listener:
//! cargo run -p native-listener --features zenoh
//! ```
//!
//! # Enabling debug logs:
//! ```bash
//! RUST_LOG=debug cargo run -p native-listener --features zenoh
//! ```

#[cfg(not(feature = "zenoh"))]
use log::{debug, error, info};
#[cfg(feature = "zenoh")]
use log::{debug, error, info, warn};
use nano_ros::prelude::*;
use std_msgs::msg::Int32;

#[cfg(feature = "zenoh")]
fn main() {
    env_logger::init();

    info!("nano-ros Native Listener (Zenoh Transport)");
    info!("==========================================");

    // Create a connected node
    let config = NodeConfig::new("listener", "/demo");

    // Try to connect to zenoh router
    let mut node = match ConnectedNode::connect(config.clone(), "tcp/127.0.0.1:7447") {
        Ok(node) => {
            info!("Connected to zenoh router at tcp/127.0.0.1:7447");
            node
        }
        Err(e) => {
            warn!("Failed to connect to zenoh router: {:?}", e);
            warn!("Trying peer mode...");

            // Fall back to peer mode
            match ConnectedNode::connect_peer(config) {
                Ok(node) => {
                    info!("Connected in peer mode");
                    node
                }
                Err(e) => {
                    error!("Failed to connect in peer mode: {:?}", e);
                    std::process::exit(1);
                }
            }
        }
    };

    info!("Node: {}/{}", node.namespace(), node.name());

    // Create a subscriber for Int32 messages on /chatter topic
    // Using /chatter to match ROS 2 demo_nodes_cpp talker
    let mut subscriber = match node.create_subscriber::<Int32>("/chatter") {
        Ok(sub) => {
            info!("Subscriber created for topic: /chatter");
            debug!("Message type: {}", Int32::TYPE_NAME);
            sub
        }
        Err(e) => {
            error!("Failed to create subscriber: {:?}", e);
            std::process::exit(1);
        }
    };

    info!("Waiting for Int32 messages on /chatter...");
    info!("(Press Ctrl+C to exit)");

    // Receiving loop
    let mut count: u64 = 0;
    loop {
        match subscriber.try_recv() {
            Ok(Some(msg)) => {
                count += 1;
                info!("[{}] Received: data={}", count, msg.data);
            }
            Ok(None) => {
                // No message available, sleep briefly
            }
            Err(e) => {
                error!("Receive error: {:?}", e);
            }
        }

        // Small sleep to avoid busy-waiting
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

#[cfg(not(feature = "zenoh"))]
fn main() {
    env_logger::init();

    info!("nano-ros Native Listener (Simulation Mode)");
    info!("==========================================");
    info!("Note: Running without zenoh transport.");
    info!("To use real transport, run with: --features zenoh");

    // Create a node (without transport)
    let config = NodeConfig::new("listener", "/demo");
    let mut node = Node::<4, 4>::new(config);

    info!("Node created: {}", node.fully_qualified_name());

    // Create a subscriber for Int32 messages
    let subscriber = node
        .create_subscriber::<Int32>("/chatter")
        .expect("Failed to create subscriber");

    info!("Subscriber created for topic: /chatter");
    debug!("Message type: {}", Int32::TYPE_NAME);

    // Simulate receiving messages (in real implementation, bytes come from transport)
    // Here we create some test CDR data to demonstrate deserialization
    info!("Simulating received messages...");

    for i in 0..10 {
        // Create test CDR data (header + i32)
        // CDR header: [0x00, 0x01, 0x00, 0x00] (little-endian)
        // i32 value: little-endian bytes
        let value: i32 = i * 10;
        let mut test_data = vec![0x00, 0x01, 0x00, 0x00]; // CDR header
        test_data.extend_from_slice(&value.to_le_bytes()); // i32 payload

        // Deserialize the message
        match node.deserialize_message::<Int32>(&subscriber, &test_data) {
            Ok(msg) => {
                info!(
                    "[{}] Received (simulated): data={}, from {} bytes",
                    i,
                    msg.data,
                    test_data.len()
                );
            }
            Err(e) => {
                error!("Deserialization error: {:?}", e);
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(500));
    }

    info!("Listener finished (simulation mode).");
}
