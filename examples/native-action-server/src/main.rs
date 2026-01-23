//! Native Action Server Example
//!
//! Demonstrates a ROS 2 action server using nano-ros.
//! This example implements a Fibonacci action that computes the Fibonacci
//! sequence up to a given order, sending feedback as it computes.
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
use log::{error, info};

#[cfg(feature = "zenoh")]
use example_interfaces::action::{Fibonacci, FibonacciFeedback, FibonacciGoal, FibonacciResult};
#[cfg(feature = "zenoh")]
use nano_ros::prelude::*;

#[cfg(feature = "zenoh")]
fn main() {
    env_logger::init();

    info!("nano-ros Action Server Example");
    info!("================================");

    // Create context
    let context = match Context::from_env() {
        Ok(ctx) => ctx,
        Err(e) => {
            error!("Failed to create context: {:?}", e);
            std::process::exit(1);
        }
    };

    // Create executor and node
    let mut executor = context.create_basic_executor();
    let mut node = match executor.create_node("fibonacci_action_server") {
        Ok(node) => {
            info!("Node created: fibonacci_action_server");
            node
        }
        Err(e) => {
            error!("Failed to create node: {:?}", e);
            std::process::exit(1);
        }
    };

    info!("Node: {}", node.name());

    // Create action server
    let mut server = match node.create_action_server::<Fibonacci>("/fibonacci") {
        Ok(s) => {
            info!("Action server created: /fibonacci");
            s
        }
        Err(e) => {
            error!("Failed to create action server: {:?}", e);
            std::process::exit(1);
        }
    };

    info!("Waiting for action goals...");
    info!("(Run native-action-client in another terminal)");

    // Main loop - handle incoming goals
    loop {
        // Try to accept new goals
        match server.try_accept_goal(|goal: &FibonacciGoal| {
            info!("Received goal request: order={}", goal.order);
            // Accept all goals
            GoalResponse::AcceptAndExecute
        }) {
            Ok(Some(goal_id)) => {
                info!("Goal accepted: {}", goal_id);

                // Get the goal data
                if let Some(active_goal) = server.get_goal(&goal_id) {
                    let order = active_goal.goal.order;

                    // Set status to executing
                    let _ = server.set_goal_status(&goal_id, GoalStatus::Executing);

                    // Compute Fibonacci sequence with feedback
                    let mut sequence: heapless::Vec<i32, 64> = heapless::Vec::new();

                    for i in 0..=order {
                        let next_val = if i == 0 {
                            0
                        } else if i == 1 {
                            1
                        } else {
                            let len = sequence.len();
                            sequence[len - 1] + sequence[len - 2]
                        };

                        let _ = sequence.push(next_val);

                        // Send feedback (clone the partial sequence)
                        let feedback = FibonacciFeedback {
                            sequence: sequence.clone(),
                        };

                        if let Err(e) = server.publish_feedback(&goal_id, &feedback) {
                            error!("Failed to publish feedback: {:?}", e);
                        } else {
                            info!("Feedback: {:?}", feedback.sequence);
                        }

                        // Simulate computation time
                        std::thread::sleep(std::time::Duration::from_millis(500));
                    }

                    // Create result
                    let result = FibonacciResult { sequence };
                    info!("Goal completed: {:?}", result.sequence);

                    // Complete the goal
                    if let Err(e) = server.complete_goal(&goal_id, GoalStatus::Succeeded, &result) {
                        error!("Failed to complete goal: {:?}", e);
                    }
                }
            }
            Ok(None) => {
                // No goal request available, sleep briefly
                std::thread::sleep(std::time::Duration::from_millis(100));
            }
            Err(e) => {
                error!("Error accepting goal: {:?}", e);
            }
        }
    }
}

#[cfg(not(feature = "zenoh"))]
fn main() {
    env_logger::init();
    info!("nano-ros Action Server Example");
    info!("================================");
    info!("This example requires the 'zenoh' feature.");
    info!("Run with: cargo run -p native-action-server --features zenoh");
}
