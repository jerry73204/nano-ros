//! Native Service Server Example
//!
//! Demonstrates a ROS 2 service server using nano-ros with the executor API.
//! Uses callback-based service handling via spin().
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
fn main() {
    env_logger::init();

    info!("nano-ros Service Server Example");
    info!("================================");

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
    let mut node = match executor.create_node("add_two_ints_server") {
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

    // Create service server with callback
    // The callback is invoked automatically during spin()
    match node.create_service::<AddTwoInts, _>("/add_two_ints", |request: &AddTwoIntsRequest| {
        let sum = request.a + request.b;
        info!("Received request: {} + {} = {}", request.a, request.b, sum);
        AddTwoIntsResponse { sum }
    }) {
        Ok(_handle) => {
            info!("Service server created: /add_two_ints");
        }
        Err(e) => {
            error!("Failed to create service server: {:?}", e);
            std::process::exit(1);
        }
    }

    info!("Waiting for service requests...");
    info!("(Run native-service-client in another terminal)");

    // Run the executor - service callbacks will be invoked automatically
    executor.spin(SpinOptions::default());
}

#[cfg(not(feature = "zenoh"))]
fn main() {
    env_logger::init();
    info!("nano-ros Service Server Example");
    info!("================================");
    info!("This example requires the 'zenoh' feature.");
    info!("Run with: cargo run -p native-service-server --features zenoh");
}
