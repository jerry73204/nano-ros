//! Native Service Client Example
//!
//! Demonstrates a ROS 2 service client using nano-ros.
//!
//! # Usage
//!
//! ```bash
//! # Start zenoh router first:
//! zenohd --listen tcp/127.0.0.1:7447
//!
//! # Start the service server:
//! cargo run -p native-service-server
//!
//! # Run the client:
//! cargo run -p native-service-client
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

    // Create node configuration
    let config = NodeConfig::new("add_two_ints_client", "/");

    // Connect to zenoh router
    let mut node: ConnectedNode = match ConnectedNode::connect(config.clone(), "tcp/127.0.0.1:7447")
    {
        Ok(node) => {
            info!("Connected to zenoh router at tcp/127.0.0.1:7447");
            node
        }
        Err(e) => {
            info!("Failed to connect to router: {:?}, trying peer mode...", e);
            match ConnectedNode::connect_peer(config) {
                Ok(node) => {
                    info!("Connected in peer mode");
                    node
                }
                Err(e) => {
                    error!("Failed to connect: {:?}", e);
                    std::process::exit(1);
                }
            }
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
                error!("  cargo run -p native-service-server");
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
    info!("Run with: cargo run -p native-service-client --features zenoh");
}
