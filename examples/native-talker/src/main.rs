//! Native Talker Example
//!
//! Demonstrates publishing messages using nano-ros on native x86.
//! Uses the unified executor API with spin_once() for manual control.
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
use log::{debug, error, info};
use nano_ros::prelude::*;
use std_msgs::msg::Int32;

#[cfg(feature = "zenoh")]
fn main() {
    env_logger::init();

    info!("nano-ros Native Talker (Zenoh Transport)");
    info!("=========================================");

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
    let node = match executor.create_node("talker".namespace("/demo")) {
        Ok(node) => {
            info!("Node created: talker in namespace /demo");
            node
        }
        Err(e) => {
            error!("Failed to create node: {:?}", e);
            std::process::exit(1);
        }
    };

    info!("Node: {}/{}", node.namespace(), node.name());

    // Declare a typed parameter
    let counter_start_value = node
        .declare_parameter("start_value")
        .default(0i64)
        .description("Initial value for the counter")
        .integer_range(0, 1000, 1)
        .unwrap()
        .mandatory()
        .unwrap();

    info!("Counter start value: {}", counter_start_value.get());

    // Create a publisher for Int32 messages on /chatter topic
    // Using /chatter to match ROS 2 demo_nodes_cpp talker
    let publisher = match node
        .create_publisher::<Int32>(PublisherOptions::new("/chatter").reliable().keep_last(10))
    {
        Ok(pub_) => {
            info!("Publisher created for topic: /chatter");
            debug!("Message type: {}", Int32::TYPE_NAME);
            pub_
        }
        Err(e) => {
            error!("Failed to create publisher: {:?}", e);
            std::process::exit(1);
        }
    };

    info!("Publishing Int32 messages...");

    // Publishing loop with spin_once() - demonstrates manual control pattern
    // This pattern is also used in RTIC/embedded where you control the main loop
    let mut count: i32 = counter_start_value.get() as i32;
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

        // Process any pending callbacks (timers, subscriptions)
        // The delta_ms parameter updates internal timers
        let _result = executor.spin_once(1000); // 1000ms delta

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
    let mut node = StandaloneNode::<4, 4>::new(config);

    info!("Node created: {}", node.fully_qualified_name());

    // Create a publisher for Int32 messages
    let publisher = node
        .create_publisher::<Int32>(PublisherOptions::new("/chatter"))
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
