//! Native Talker Example
//!
//! Demonstrates publishing messages using nano-ros on native x86.
//!
//! # Without zenoh feature (simulation mode):
//! ```bash
//! cargo run -p native-talker
//! ```
//!
//! # With zenoh feature (real transport):
//! ```bash
//! # Start zenoh router first:
//! zenohd --listen tcp/127.0.0.1:7447
//!
//! # Then run the talker:
//! cargo run -p native-talker --features zenoh
//! ```
//!
//! # Enabling debug logs:
//! ```bash
//! RUST_LOG=debug cargo run -p native-talker --features zenoh
//! ```

#[cfg(not(feature = "zenoh"))]
use log::{debug, error, info};
#[cfg(feature = "zenoh")]
use log::{debug, error, info, warn};
use nano_ros::prelude::*;
use nano_ros::types::std_msgs::Int32;

#[cfg(feature = "zenoh")]
fn main() {
    env_logger::init();

    info!("nano-ros Native Talker (Zenoh Transport)");
    info!("=========================================");

    // Create a connected node
    let config = NodeConfig::new("talker", "/demo");

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

    // Create a publisher for Int32 messages on /chatter topic
    // Using /chatter to match ROS 2 demo_nodes_cpp talker
    let publisher = match node.create_publisher::<Int32>("/chatter") {
        Ok(pub_) => {
            info!("Publisher created for topic: /chatter");
            debug!("Message type: {}", Int32::TYPE_NAME);
            debug!("Type hash: {}", Int32::TYPE_HASH);
            debug!("ZID: {}", node.zid().to_hex_string());
            pub_
        }
        Err(e) => {
            error!("Failed to create publisher: {:?}", e);
            std::process::exit(1);
        }
    };

    info!("Publishing Int32 messages...");

    // Publishing loop
    let mut count: i32 = 0;
    loop {
        let msg = Int32 { data: count };

        match publisher.publish(&msg) {
            Ok(()) => {
                info!("[{}] Published: data={}", count, msg.data);
            }
            Err(e) => {
                error!("Publish error: {:?}", e);
            }
        }

        count = count.wrapping_add(1);

        // Sleep 1 second between messages (like ROS 2 demo)
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}

#[cfg(not(feature = "zenoh"))]
fn main() {
    env_logger::init();

    info!("nano-ros Native Talker (Simulation Mode)");
    info!("=========================================");
    info!("Note: Running without zenoh transport.");
    info!("To use real transport, run with: --features zenoh");

    // Create a node (without transport)
    let config = NodeConfig::new("talker", "/demo");
    let mut node = Node::<4, 4>::new(config);

    info!("Node created: {}", node.fully_qualified_name());

    // Create a publisher for Int32 messages
    let publisher = node
        .create_publisher::<Int32>("/chatter")
        .expect("Failed to create publisher");

    info!("Publisher created for topic: /chatter");
    debug!("Message type: {}", Int32::TYPE_NAME);

    // Simulate publishing loop
    for i in 0..10 {
        let msg = Int32 { data: i };

        // Serialize the message (but don't actually send it)
        match node.serialize_message(&publisher, &msg) {
            Ok(bytes) => {
                info!(
                    "[{}] Serialized: data={}, {} bytes: {:02x?}...",
                    i,
                    msg.data,
                    bytes.len(),
                    &bytes[..bytes.len().min(16)]
                );
            }
            Err(e) => {
                error!("Serialization error: {:?}", e);
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(500));
    }

    info!("Talker finished (simulation mode).");
}
