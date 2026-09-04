//! Async Action Client Example — tokio background spin + StreamExt
//!
//! Demonstrates the async action client pattern:
//! 1. Create executor → create action client (owned, no lifetime to executor)
//! 2. Move executor to a background `spin_async()` task
//! 3. `.await` the goal acceptance Promise directly
//! 4. Stream feedback with `futures::StreamExt` combinators
//! 5. `.await` the get_result Promise
//!
//! # Usage
//!
//! ```bash
//! # Terminal 1: Start zenoh router
//! ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd
//!
//! # Terminal 2: Start the action server
//! cargo run -p native-rs-action-server
//!
//! # Terminal 3: Run this async client
//! cargo run -p native-rs-async-action-client
//! ```

use example_interfaces::action::{Fibonacci, FibonacciGoal};
use futures::StreamExt;
use nros::prelude::*;
use nros_log::{Logger, log_error, log_info, log_warn};

// Diagnostics route through `nros-log`.
static LOGGER: Logger = Logger::new("action-client-async");

extern crate nros_platform_cffi as _;

#[tokio::main(flavor = "current_thread")]
async fn main() {
    // Register the RMW backend the build linked (idempotent; must run before
    // the executor opens). RMW selection is build/config, never source.
    nros_board_linux::register_linked_rmw();

    nros_log::register_logger(&LOGGER);
    nros_log::init(nros_platform_cffi::log::default_sinks());

    // Create executor
    let config = ExecutorConfig::from_env().node_name("fibonacci_action_client_async");
    let mut executor = Executor::open(&config).expect("Failed to open session");

    // Create action client — owned type, no lifetime tied to node or executor.
    // The node is dropped at the end of this block, freeing the executor.
    let mut client = {
        let mut node = executor
            .create_node("fibonacci_action_client_async")
            .expect("Failed to create node");
        node.create_action_client::<Fibonacci>("/fibonacci")
            .expect("Failed to create action client")
    };

    let goal = FibonacciGoal { order: 10 };
    let order = goal.order;
    log_info!(&LOGGER, "Sending goal");

    // LocalSet enables spawn_local (single-threaded, no Send bound needed)
    let local = tokio::task::LocalSet::new();
    local
        .run_until(async move {
            // Spawn spin_async() as a background task on the same thread.
            // This drives I/O so action messages can arrive.
            tokio::task::spawn_local(async move {
                executor.spin_async().await;
            });

            // ── Step 1: Send goal and await acceptance ──────────────
            let (goal_id, promise) = match client.send_goal(&goal) {
                Ok(pair) => pair,
                Err(e) => {
                    log_error!(&LOGGER, "Failed to send goal: {:?}", e);
                    return;
                }
            };

            let accepted = match promise.await {
                Ok(accepted) => accepted,
                Err(e) => {
                    log_error!(&LOGGER, "Goal acceptance failed: {:?}", e);
                    return;
                }
            };

            if !accepted {
                log_warn!(&LOGGER, "Goal was rejected by the server");
                return;
            }
            log_info!(&LOGGER, "Goal accepted by server, waiting for result");

            // ── Step 2: Stream feedback with StreamExt ──────────────
            {
                let mut stream = client.feedback_stream_for(goal_id);

                // StreamExt::next() drives the stream one item at a time.
                // The background spin_async() task processes I/O concurrently.
                while let Some(result) = stream.next().await {
                    match result {
                        Ok(feedback) => {
                            log_info!(
                                &LOGGER,
                                "Next number in sequence received: {:?}",
                                feedback.sequence
                            );

                            if feedback.sequence.len() as i32 > order {
                                break;
                            }
                        }
                        Err(e) => {
                            log_error!(&LOGGER, "Feedback error: {:?}", e);
                            break;
                        }
                    }
                }
            } // stream dropped — releases &mut client

            // ── Step 3: Get final result ────────────────────────────
            match client.get_result(&goal_id) {
                Ok(promise) => match promise.await {
                    Ok((status, result)) => {
                        if status != GoalStatus::Succeeded {
                            log_warn!(&LOGGER, "Goal finished with status {:?}", status);
                        }
                        log_info!(&LOGGER, "Result received: {:?}", result.sequence);
                    }
                    Err(e) => log_error!(&LOGGER, "get_result failed: {:?}", e),
                },
                Err(e) => log_error!(&LOGGER, "get_result failed: {:?}", e),
            }
        })
        .await;
}
