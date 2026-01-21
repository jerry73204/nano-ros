//! Native Action Client Example
//!
//! Demonstrates a ROS 2 action client using nano-ros.
//! This example sends a Fibonacci action goal and receives feedback
//! as the sequence is computed.
//!
//! # Usage
//!
//! ```bash
//! # Start zenoh router first:
//! zenohd --listen tcp/127.0.0.1:7447
//!
//! # Run the action server:
//! cargo run -p native-action-server
//!
//! # In another terminal, run the client:
//! cargo run -p native-action-client
//! ```

#[cfg(not(feature = "zenoh"))]
use log::info;
#[cfg(feature = "zenoh")]
use log::{error, info, warn};

#[cfg(feature = "zenoh")]
use example_interfaces::action::{Fibonacci, FibonacciGoal};
#[cfg(feature = "zenoh")]
use nano_ros::prelude::*;

#[cfg(feature = "zenoh")]
fn main() {
    env_logger::init();

    info!("nano-ros Action Client Example");
    info!("================================");

    // Create node configuration
    let config = NodeConfig::new("fibonacci_action_client", "/");

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

    // Create action client
    let mut client = match node.create_action_client::<Fibonacci>("/fibonacci") {
        Ok(c) => {
            info!("Action client created: /fibonacci");
            c
        }
        Err(e) => {
            error!("Failed to create action client: {:?}", e);
            std::process::exit(1);
        }
    };

    // Create goal - compute Fibonacci sequence up to order 10
    let goal = FibonacciGoal { order: 10 };
    info!("Sending goal: order={}", goal.order);

    // Send goal
    let goal_handle = match client.send_goal(&goal) {
        Ok(handle) => {
            if handle.accepted {
                info!("Goal accepted! ID: {}", handle.goal_id);
            } else {
                warn!("Goal was rejected by the server");
                std::process::exit(1);
            }
            handle
        }
        Err(e) => {
            error!("Failed to send goal: {:?}", e);
            std::process::exit(1);
        }
    };

    info!("Waiting for feedback and result...");

    // Poll for feedback
    let mut feedback_count = 0;
    let start_time = std::time::Instant::now();
    let timeout = std::time::Duration::from_secs(30);

    loop {
        // Check timeout
        if start_time.elapsed() > timeout {
            error!("Timeout waiting for result");
            break;
        }

        // Try to receive feedback
        match client.try_recv_feedback() {
            Ok(Some((goal_id, feedback))) => {
                if goal_id == goal_handle.goal_id {
                    feedback_count += 1;
                    info!("Feedback #{}: {:?}", feedback_count, feedback.sequence);

                    // Check if we've received all expected feedback (order + 1 values)
                    if feedback.sequence.len() as i32 > goal.order {
                        info!("Received all feedback, action completed!");
                        info!("Final sequence: {:?}", feedback.sequence);
                        break;
                    }
                }
            }
            Ok(None) => {
                // No feedback available, sleep briefly
                std::thread::sleep(std::time::Duration::from_millis(50));
            }
            Err(e) => {
                error!("Error receiving feedback: {:?}", e);
            }
        }
    }

    info!("Action client finished");
}

#[cfg(not(feature = "zenoh"))]
fn main() {
    env_logger::init();
    info!("nano-ros Action Client Example");
    info!("================================");
    info!("This example requires the 'zenoh' feature.");
    info!("Run with: cargo run -p native-action-client --features zenoh");
}
