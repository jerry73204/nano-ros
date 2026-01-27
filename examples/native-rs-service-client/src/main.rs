//! Native Service Client Example
//!
//! Demonstrates a ROS 2 service client using nano-ros with the executor API.
//! Service clients use blocking calls, so no spin() is needed.
//!
//! # Usage
//!
//! ```bash
//! # Start zenoh router first:
//! zenohd --listen tcp/127.0.0.1:7447
//!
//! # Start the service server:
//! cargo run -p native-rs-service-server
//!
//! # Run the client:
//! cargo run -p native-rs-service-client
//! ```

#[cfg(not(feature = "zenoh"))]
use log::info;
#[cfg(feature = "zenoh")]
use log::{error, info};

#[cfg(feature = "zenoh")]
use example_interfaces::srv::{AddTwoInts, AddTwoIntsRequest};
#[cfg(feature = "zenoh")]
use nano_ros::prelude::*;

#[cfg(feature = "zenoh")]
fn main() {
    env_logger::init();

    info!("nano-ros Service Client Example");
    info!("================================");

    // Create context using rclrs-style API
    let context = match Context::from_env() {
        Ok(ctx) => ctx,
        Err(e) => {
            error!("Failed to create context: {:?}", e);
            std::process::exit(1);
        }
    };

    // Create executor and node through executor
    let mut executor = context.create_basic_executor();
    let mut node = match executor.create_node("add_two_ints_client") {
        Ok(node) => {
            info!("Node created: add_two_ints_client");
            node
        }
        Err(e) => {
            error!("Failed to create node: {:?}", e);
            std::process::exit(1);
        }
    };

    info!("Node: {}", node.name());

    // Create service client
    let mut client = match node.create_client::<AddTwoInts>("/add_two_ints") {
        Ok(c) => {
            info!("Service client created for: /add_two_ints");
            c
        }
        Err(e) => {
            error!("Failed to create service client: {:?}", e);
            std::process::exit(1);
        }
    };

    // Make several service calls
    let test_cases = [(5, 3), (10, 20), (100, 200), (-5, 10)];

    for (a, b) in test_cases {
        let request = AddTwoIntsRequest { a, b };
        info!("Calling service: {} + {} = ?", a, b);

        match client.call(&request) {
            Ok(response) => {
                info!("Response: {} + {} = {}", a, b, response.sum);
                assert_eq!(response.sum, a + b, "Sum mismatch!");
            }
            Err(e) => {
                error!("Service call failed: {:?}", e);
                error!("Make sure the service server is running:");
                error!("  cargo run -p native-rs-service-server");
                std::process::exit(1);
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(500));
    }

    info!("All service calls completed successfully!");
}

#[cfg(not(feature = "zenoh"))]
fn main() {
    env_logger::init();
    info!("nano-ros Service Client Example");
    info!("================================");
    info!("This example requires the 'zenoh' feature.");
    info!("Run with: cargo run -p native-rs-service-client --features zenoh");
}
