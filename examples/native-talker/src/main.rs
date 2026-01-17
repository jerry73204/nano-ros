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

use nano_ros_core::RosMessage;
use nano_ros_types::std_msgs::Int32;

#[cfg(feature = "zenoh")]
fn main() {
    use nano_ros_node::{ConnectedNode, NodeConfig};

    println!("nano-ros Native Talker (Zenoh Transport)");
    println!("=========================================");

    // Create a connected node
    let config = NodeConfig::new("talker", "/demo");

    // Try to connect to zenoh router
    let mut node = match ConnectedNode::connect(config.clone(), "tcp/127.0.0.1:7447") {
        Ok(node) => {
            println!("Connected to zenoh router at tcp/127.0.0.1:7447");
            node
        }
        Err(e) => {
            eprintln!("Failed to connect to zenoh router: {:?}", e);
            eprintln!("Trying peer mode...");

            // Fall back to peer mode
            match ConnectedNode::connect_peer(config) {
                Ok(node) => {
                    println!("Connected in peer mode");
                    node
                }
                Err(e) => {
                    eprintln!("Failed to connect in peer mode: {:?}", e);
                    std::process::exit(1);
                }
            }
        }
    };

    println!("Node: {}/{}", node.namespace(), node.name());

    // Create a publisher for Int32 messages on /chatter topic
    // Using /chatter to match ROS 2 demo_nodes_cpp talker
    let publisher = match node.create_publisher::<Int32>("/chatter") {
        Ok(pub_) => {
            println!("Publisher created for topic: /chatter");
            println!("Message type: {}", Int32::TYPE_NAME);
            pub_
        }
        Err(e) => {
            eprintln!("Failed to create publisher: {:?}", e);
            std::process::exit(1);
        }
    };

    println!();
    println!("Publishing Int32 messages...");
    println!();

    // Publishing loop
    let mut count: i32 = 0;
    loop {
        let msg = Int32 { data: count };

        match publisher.publish(&msg) {
            Ok(()) => {
                println!("[{}] Published: data={}", count, msg.data);
            }
            Err(e) => {
                eprintln!("Publish error: {:?}", e);
            }
        }

        count = count.wrapping_add(1);

        // Sleep 1 second between messages (like ROS 2 demo)
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}

#[cfg(not(feature = "zenoh"))]
fn main() {
    use nano_ros_node::{Node, NodeConfig};

    println!("nano-ros Native Talker (Simulation Mode)");
    println!("=========================================");
    println!();
    println!("Note: Running without zenoh transport.");
    println!("To use real transport, run with: --features zenoh");
    println!();

    // Create a node (without transport)
    let config = NodeConfig::new("talker", "/demo");
    let mut node = Node::<4, 4>::new(config);

    println!("Node created: {}", node.fully_qualified_name());

    // Create a publisher for Int32 messages
    let publisher = node
        .create_publisher::<Int32>("/chatter")
        .expect("Failed to create publisher");

    println!("Publisher created for topic: /chatter");
    println!("Message type: {}", Int32::TYPE_NAME);
    println!();

    // Simulate publishing loop
    for i in 0..10 {
        let msg = Int32 { data: i };

        // Serialize the message (but don't actually send it)
        match node.serialize_message(&publisher, &msg) {
            Ok(bytes) => {
                println!(
                    "[{}] Serialized: data={}, {} bytes: {:02x?}...",
                    i,
                    msg.data,
                    bytes.len(),
                    &bytes[..bytes.len().min(16)]
                );
            }
            Err(e) => {
                eprintln!("Serialization error: {:?}", e);
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(500));
    }

    println!();
    println!("Talker finished (simulation mode).");
}
