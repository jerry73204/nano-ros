//! Native Listener Example
//!
//! Demonstrates subscribing to messages using nano-ros on native x86.
//! Uses the unified executor API with callback-based subscriptions.
//!
//! # Without zenoh feature (simulation mode):
//! ```bash
//! cargo run -p native-rs-listener
//! ```
//!
//! # With zenoh feature (real transport):
//! ```bash
//! # Start zenoh router first:
//! zenohd --listen tcp/127.0.0.1:7447
//!
//! # Then run the listener:
//! cargo run -p native-rs-listener --features zenoh
//! ```
//!
//! # Enabling debug logs:
//! ```bash
//! RUST_LOG=debug cargo run -p native-rs-listener --features zenoh
//! ```

#[cfg(not(feature = "zenoh"))]
use log::{debug, error, info};
#[cfg(feature = "zenoh")]
use log::{debug, error, info};
use nano_ros::prelude::*;
use std_msgs::msg::Int32;

#[cfg(feature = "zenoh")]
use std::sync::atomic::{AtomicU64, Ordering};

#[cfg(feature = "zenoh")]
static MESSAGE_COUNT: AtomicU64 = AtomicU64::new(0);

#[cfg(feature = "zenoh")]
fn main() {
    env_logger::init();

    info!("nano-ros Native Listener (Zenoh Transport)");
    info!("==========================================");

    // Create context using rclrs-style API
    let context = match Context::from_env() {
        Ok(ctx) => ctx,
        Err(e) => {
            error!("Failed to create context: {:?}", e);
            std::process::exit(1);
        }
    };

    // Create executor - owns and manages nodes
    let mut executor = context.create_basic_executor();

    // Create node through executor
    let mut node = match executor.create_node("listener".namespace("/demo")) {
        Ok(node) => {
            info!("Node created: listener in namespace /demo");
            node
        }
        Err(e) => {
            error!("Failed to create node: {:?}", e);
            std::process::exit(1);
        }
    };

    info!("Node: {}/{}", node.namespace(), node.name());

    // Create a subscription with callback for Int32 messages on /chatter topic
    // Using /chatter to match ROS 2 demo_nodes_cpp talker
    match node.create_subscription::<Int32, _>(
        SubscriberOptions::new("/chatter").reliable().keep_last(10),
        |msg: &Int32| {
            let count = MESSAGE_COUNT.fetch_add(1, Ordering::SeqCst) + 1;
            info!("[{}] Received: data={}", count, msg.data);
        },
    ) {
        Ok(_handle) => {
            info!("Subscriber created for topic: /chatter");
            debug!("Message type: {}", Int32::TYPE_NAME);
        }
        Err(e) => {
            error!("Failed to create subscriber: {:?}", e);
            std::process::exit(1);
        }
    }

    info!("Waiting for Int32 messages on /chatter...");
    info!("(Press Ctrl+C to exit)");

    // Run the executor - callbacks will be invoked automatically
    executor.spin(SpinOptions::default());
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
    let mut node = StandaloneNode::<4, 4>::new(config);

    info!("Node created: {}", node.fully_qualified_name());

    // Create a subscriber for Int32 messages
    let subscriber = node
        .create_subscriber::<Int32>(SubscriberOptions::new("/chatter"))
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
