//! Native Service Server Example
//!
//! Demonstrates a ROS 2 service server using nano-ros.
//!
//! # Usage
//!
//! ```bash
//! # Start zenoh router first:
//! zenohd --listen tcp/127.0.0.1:7447
//!
//! # Run the service server:
//! cargo run -p native-service-server
//!
//! # In another terminal, run the client:
//! cargo run -p native-service-client
//! ```

#[cfg(not(feature = "zenoh"))]
use log::info;
#[cfg(feature = "zenoh")]
use log::{error, info};

#[cfg(feature = "zenoh")]
use example_interfaces::srv::{AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse};
#[cfg(feature = "zenoh")]
use nano_ros::prelude::*;

#[cfg(feature = "zenoh")]
#[allow(deprecated)] // TODO: Update to use executor API once service callbacks are supported
fn main() {
    env_logger::init();

    info!("nano-ros Service Server Example");
    info!("================================");

    // Create context and node using rclrs-style API
    let context = match Context::from_env() {
        Ok(ctx) => ctx,
        Err(e) => {
            error!("Failed to create context: {:?}", e);
            std::process::exit(1);
        }
    };

    let mut node = match context.create_node("add_two_ints_server") {
        Ok(node) => {
            info!("Node created: add_two_ints_server");
            node
        }
        Err(e) => {
            error!("Failed to create node: {:?}", e);
            std::process::exit(1);
        }
    };

    info!("Node: {}", node.name());

    // Create service server
    let mut server = match node.create_service::<AddTwoInts>("/add_two_ints") {
        Ok(s) => {
            info!("Service server created: /add_two_ints");
            s
        }
        Err(e) => {
            error!("Failed to create service server: {:?}", e);
            std::process::exit(1);
        }
    };

    info!("Waiting for service requests...");
    info!("(Run native-service-client in another terminal)");

    // Service handler function
    fn handle_add(request: &AddTwoIntsRequest) -> AddTwoIntsResponse {
        let sum = request.a + request.b;
        info!("Received request: {} + {} = {}", request.a, request.b, sum);
        AddTwoIntsResponse { sum }
    }

    // Main loop - handle incoming requests
    loop {
        match server.handle_request(handle_add) {
            Ok(true) => {
                // Request was handled
            }
            Ok(false) => {
                // No request available, sleep briefly
                std::thread::sleep(std::time::Duration::from_millis(100));
            }
            Err(e) => {
                error!("Error handling request: {:?}", e);
            }
        }
    }
}

#[cfg(not(feature = "zenoh"))]
fn main() {
    env_logger::init();
    info!("nano-ros Service Server Example");
    info!("================================");
    info!("This example requires the 'zenoh' feature.");
    info!("Run with: cargo run -p native-service-server --features zenoh");
}
