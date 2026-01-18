//! Embassy Example for nano-ros on STM32F4
//!
//! This example demonstrates how to use nano-ros with Embassy on an STM32F4
//! microcontroller. It shows:
//!
//! - Async task-based architecture
//! - Manual zenoh session polling using async tasks
//! - Periodic publishing using embassy timers
//! - Channel-based communication between tasks
//!
//! # Hardware
//!
//! Tested on NUCLEO-F429ZI, but should work on any STM32F4 with:
//! - Ethernet or serial connection for zenoh transport
//! - At least 64KB RAM
//!
//! # Building
//!
//! ```bash
//! cargo build --release
//! ```
//!
//! # Flashing
//!
//! ```bash
//! cargo run --release  # Uses probe-rs
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use panic_halt as _;

/// Timing constants for Embassy tasks (same as nano_ros_node::rtic when zenoh is enabled)
/// Poll interval in milliseconds - how often to poll zenoh for incoming messages
const POLL_INTERVAL_MS: u64 = 10;
/// Keepalive interval in milliseconds - how often to send keepalive to maintain session
const KEEPALIVE_INTERVAL_MS: u64 = 1000;

/// Channel for sending publish requests from publisher to a shared node task
static PUBLISH_CHANNEL: Channel<CriticalSectionRawMutex, PublishRequest, 4> = Channel::new();

/// A request to publish data
struct PublishRequest {
    counter: u32,
}

/// Main entry point - spawns all async tasks
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("nano-ros Embassy example starting...");

    // Initialize Embassy HAL
    let p = embassy_stm32::init(Default::default());

    defmt::info!("Embassy HAL initialized");

    // Configure GPIO for LED (PB7 on NUCLEO-F429ZI)
    let led = Output::new(p.PB7, Level::Low, Speed::Low);

    // In a real application, you would:
    // 1. Initialize the network interface (Ethernet or UART)
    // 2. Create the zenoh session without background tasks:
    //
    // ```
    // let config = NodeConfig::new("embassy_node", "/demo");
    // let node = ConnectedNode::connect_without_tasks(config, "tcp/192.168.1.1:7447")?;
    // ```

    defmt::info!("Spawning async tasks...");

    // Spawn all the background tasks
    spawner.spawn(zenoh_poll_task()).unwrap();
    spawner.spawn(zenoh_keepalive_task()).unwrap();
    spawner.spawn(publisher_task(led)).unwrap();
    spawner.spawn(node_handler_task()).unwrap();

    defmt::info!("All tasks spawned, entering main loop");

    // Main task can do other work or just idle
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

/// Poll zenoh for incoming messages
///
/// This task runs every 10ms to process incoming network data.
/// In Embassy, this replaces the background read thread.
#[embassy_executor::task]
async fn zenoh_poll_task() {
    loop {
        // In a real application with ConnectedNode:
        // NODE.lock(|node| {
        //     if let Err(e) = node.poll_read() {
        //         defmt::warn!("Poll error: {:?}", e);
        //     }
        // });

        defmt::trace!("zenoh_poll tick");

        Timer::after(Duration::from_millis(POLL_INTERVAL_MS)).await;
    }
}

/// Send keepalive to maintain session
///
/// This task runs every 1s to send keepalive messages.
/// In Embassy, this replaces the background lease thread.
#[embassy_executor::task]
async fn zenoh_keepalive_task() {
    loop {
        // In a real application with ConnectedNode:
        // NODE.lock(|node| {
        //     if let Err(e) = node.send_keepalive() {
        //         defmt::warn!("Keepalive error: {:?}", e);
        //     }
        // });

        defmt::trace!("zenoh_keepalive tick");

        Timer::after(Duration::from_millis(KEEPALIVE_INTERVAL_MS)).await;
    }
}

/// Handle node operations (publishing, subscribing)
///
/// This task receives publish requests via channel and processes them.
/// Using a channel decouples the publisher logic from the actual zenoh operations.
#[embassy_executor::task]
async fn node_handler_task() {
    loop {
        // Wait for a publish request
        let request = PUBLISH_CHANNEL.receive().await;

        // In a real application with a publisher:
        // publisher.publish(&Int32 { data: request.counter as i32 }).ok();

        defmt::info!("Node handler: published counter = {}", request.counter);
    }
}

/// Publish messages periodically
///
/// This task demonstrates publishing ROS 2 messages at a fixed rate.
/// It sends publish requests to the node handler via a channel.
#[embassy_executor::task]
async fn publisher_task(mut led: Output<'static>) {
    let mut counter: u32 = 0;

    loop {
        // Increment counter
        counter = counter.wrapping_add(1);

        // Toggle LED to show activity
        led.toggle();

        // Send publish request to node handler
        PUBLISH_CHANNEL
            .send(PublishRequest { counter })
            .await;

        defmt::info!("Publisher: sent counter = {}", counter);

        // Publish at 10 Hz
        Timer::after(Duration::from_millis(100)).await;
    }
}
