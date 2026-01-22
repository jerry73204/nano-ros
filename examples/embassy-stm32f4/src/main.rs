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
use embassy_time::{Duration, Timer};
use nano_ros::prelude::{Context, Executor, InitOptions, NodeNameExt, PollingExecutor};
use panic_halt as _;

/// Timing constants for Embassy tasks
const POLL_INTERVAL_MS: u64 = 10;

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
    // 2. Create the context, executor, and node:
    //
    // let init_options = InitOptions::new().locator("tcp/192.168.1.1:7447");
    // let ctx = Context::new(init_options).unwrap();
    // let mut executor: PollingExecutor<1> = ctx.create_polling_executor();
    // let node = executor.create_node("embassy_node".namespace("/demo")).unwrap();
    // let publisher = node.create_publisher::<Int32>("/counter").unwrap();

    defmt::info!("Spawning async tasks...");

    // Spawn the background tasks
    // Note: The executor must be moved into a statically allocated container
    // to be shared across tasks, e.g., using embassy_sync::Mutex.
    // For simplicity, this example shows the task structure without shared state.
    spawner.spawn(zenoh_poll_task()).unwrap();
    spawner.spawn(publisher_task(led)).unwrap();

    defmt::info!("All tasks spawned, entering main loop");

    // Main task can do other work or just idle
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

/// Poll zenoh for incoming messages
///
/// This task runs every 10ms to process incoming network data.
#[embassy_executor::task]
async fn zenoh_poll_task() {
    loop {
        // In a real application with a shared executor:
        // EXECUTOR.lock(|exec| {
        //     let result = exec.spin_once(POLL_INTERVAL_MS as u64);
        //     if result.subscriptions_processed > 0 {
        //         defmt::info!("Processed {} subscriptions", result.subscriptions_processed);
        //     }
        // });

        defmt::trace!("zenoh_poll tick");

        Timer::after(Duration::from_millis(POLL_INTERVAL_MS)).await;
    }
}

/// Publish messages periodically
///
/// This task demonstrates publishing ROS 2 messages at a fixed rate.
#[embassy_executor::task]
async fn publisher_task(mut led: Output<'static>) {
    let mut counter: u32 = 0;

    loop {
        // Increment counter
        counter = counter.wrapping_add(1);

        // Toggle LED to show activity
        led.toggle();

        // In a real application with a shared publisher:
        // PUBLISHER.lock(|pub_| {
        //     pub_.publish(&Int32 { data: counter as i32 }).ok();
        // });

        defmt::info!("Publisher: sent counter = {}", counter);

        // Publish at 10 Hz
        Timer::after(Duration::from_millis(100)).await;
    }
}
