//! Async Service Client Example — tokio background spin
//!
//! Demonstrates the background spin pattern for async service calls:
//! 1. Create executor → create client (owned, no lifetime to executor)
//! 2. Move executor to a background `spin_async()` task via `spawn_local`
//! 3. `.await` the Promise directly from the main task
//!
//! This pattern uses tokio's `current_thread` runtime with `LocalSet` for
//! single-threaded cooperative concurrency — no multi-threading needed.
//! The same pattern works with Embassy on embedded targets.
//!
//! Sends one `AddTwoInts` request (summands from argv, default `2 3`) and
//! logs `Result of add_two_ints: N`, like the official `demo_nodes_cpp`
//! `add_two_ints_client` demo.
//!
//! # Usage
//!
//! ```bash
//! # Terminal 1: Start zenoh router
//! ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd
//!
//! # Terminal 2: Start the service server
//! cargo run -p native-rs-service-server
//!
//! # Terminal 3: Run this async client
//! cargo run -p native-rs-async-service-client -- 2 3
//! ```

use example_interfaces::srv::{AddTwoInts, AddTwoIntsRequest};
use nros::prelude::*;
use nros_log::{Logger, log_error, log_info};

// Diagnostics route through `nros-log`.
static LOGGER: Logger = Logger::new("service-client-async");

extern crate nros_platform_cffi as _;

#[tokio::main(flavor = "current_thread")]
async fn main() {
    // Register the RMW backend the build linked (idempotent; must run before
    // the executor opens). RMW selection is build/config, never source.
    nros_board_linux::register_linked_rmw();

    nros_log::register_logger(&LOGGER);
    nros_log::init(nros_platform_cffi::log::default_sinks());

    // Summands from argv, defaulting to the official demo's `2 3`.
    let mut args = std::env::args().skip(1).filter_map(|s| s.parse().ok());
    let a: i64 = args.next().unwrap_or(2);
    let b: i64 = args.next().unwrap_or(3);

    // Create executor
    let config = ExecutorConfig::from_env().node_name("add_two_ints_client_async");
    let mut executor: Executor = Executor::open(&config).expect("Failed to open session");

    // Create client — it's an owned type (no lifetime tied to node or executor).
    // After this block, the node is dropped and the executor is free to move.
    let mut client = {
        let mut node = executor
            .create_node("add_two_ints_client_async")
            .expect("Failed to create node");
        node.create_client::<AddTwoInts>("/add_two_ints")
            .expect("Failed to create client")
    };

    // LocalSet enables spawn_local (single-threaded, no Send bound needed)
    let local = tokio::task::LocalSet::new();
    local
        .run_until(async move {
            // Spawn spin_async() as a background task on the same thread.
            // This drives I/O so the service reply can arrive.
            tokio::task::spawn_local(async move {
                executor.spin_async().await;
            });

            // Let discovery settle before the single request.
            tokio::time::sleep(std::time::Duration::from_millis(1000)).await;

            // One service call — just .await the Promise directly. The
            // background spin task drives I/O concurrently.
            let request = AddTwoIntsRequest { a, b };
            let reply = match client.call(&request) {
                Ok(promise) => match promise.await {
                    Ok(reply) => reply,
                    Err(e) => {
                        log_error!(&LOGGER, "Service call failed: {:?}", e);
                        std::process::exit(1);
                    }
                },
                Err(e) => {
                    log_error!(&LOGGER, "Failed to send request: {:?}", e);
                    std::process::exit(1);
                }
            };

            log_info!(&LOGGER, "Result of add_two_ints: {}", reply.sum);
        })
        .await;
}
