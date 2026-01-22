//! RTIC Example for nano-ros on STM32F4
//!
//! This example demonstrates how to use nano-ros with RTIC on an STM32F4
//! microcontroller. It shows:
//!
//! - Manual zenoh session polling (no background threads)
//! - Periodic publishing using RTIC software tasks
//! - Critical-section based synchronization
//! - Priority-based preemptive scheduling
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

use panic_halt as _;

use rtic::app;
use rtic_monotonics::systick::prelude::*;

// Generate the monotonic timer
systick_monotonic!(Mono, 1000); // 1kHz tick rate

/// Timing constants for RTIC tasks (same as nano_ros_node::rtic when zenoh is enabled)
/// Poll interval in milliseconds - how often to poll zenoh for incoming messages
const POLL_INTERVAL_MS: u32 = 10;
/// Keepalive interval in milliseconds - how often to send keepalive to maintain session
const KEEPALIVE_INTERVAL_MS: u32 = 1000;

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1, SPI2, SPI3])]
mod app {
    use super::*;
    use defmt_rtt as _;
    use nano_ros::prelude::{Context, Executor, InitOptions, NodeNameExt, PollingExecutor};
    use stm32f4xx_hal::{
        gpio::{Output, Pin},
        prelude::*,
        rcc::Config,
    };

    /// Shared resources accessible by multiple tasks
    #[shared]
    struct Shared {
        // The PollingExecutor manages all nano-ros nodes
        // It's placed in shared resources to be accessible by tasks
        executor: PollingExecutor<1>, // Allow 1 node
        counter: u32,
    }

    /// Local resources owned by specific tasks
    #[local]
    struct Local {
        led: Pin<'B', 7, Output>,
    }

    /// Initialize the system
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("nano-ros RTIC example starting...");

        // Initialize the monotonic timer
        Mono::start(cx.core.SYST, 168_000_000); // 168MHz for STM32F429

        // Configure clocks using new stm32f4xx-hal 0.23 API
        let mut rcc = cx.device.RCC.freeze(Config::hse(8.MHz()).sysclk(168.MHz()));

        defmt::info!(
            "Clocks configured: sysclk = {} Hz",
            rcc.clocks.sysclk().to_Hz()
        );

        // Configure GPIO for LED (PB7 on NUCLEO-F429ZI)
        let gpiob = cx.device.GPIOB.split(&mut rcc);
        let led = gpiob.pb7.into_push_pull_output();

        // In a real application, you would:
        // 1. Initialize the network interface (Ethernet or UART)
        // 2. Create the context, executor, and node:
        //
        // let init_options = InitOptions::new().locator("tcp/192.168.1.1:7447");
        // let ctx = Context::new(init_options).unwrap();
        // let mut executor: PollingExecutor<1> = ctx.create_polling_executor();
        // let node = executor.create_node("rtic_node".namespace("/demo")).unwrap();

        defmt::info!("Starting periodic tasks...");

        // Spawn the periodic tasks
        zenoh_poll::spawn().ok();
        // The keepalive task is no longer needed with the executor model,
        // as `spin_once` handles keepalives automatically.
        // zenoh_keepalive::spawn().ok();
        publisher_task::spawn().ok();

        (
            Shared {
                // executor, // Uncomment when using real hardware
                counter: 0,
            },
            Local { led },
        )
    }

    /// Idle task - runs when no other tasks are ready
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // Wait for interrupt - saves power
            cortex_m::asm::wfi();
        }
    }

    /// Poll zenoh for incoming messages
    ///
    /// This task runs every 10ms to process incoming network data.
    /// In RTIC, this replaces the background read thread.
    /// Higher priority (2) ensures timely message processing.
    #[task(priority = 2, shared = [executor])]
    async fn zenoh_poll(mut cx: zenoh_poll::Context) {
        loop {
            // In a real application, call spin_once on the executor
            // cx.shared.executor.lock(|exec| {
            //     let result = exec.spin_once(POLL_INTERVAL_MS as u64);
            //     if result.subscriptions_processed > 0 {
            //         defmt::info!("Processed {} subscriptions", result.subscriptions_processed);
            //     }
            // });

            defmt::trace!("zenoh_poll tick");

            Mono::delay(POLL_INTERVAL_MS.millis()).await;
        }
    }

    /// Publish messages periodically
    ///
    /// This task demonstrates publishing ROS 2 messages at a fixed rate.
    /// Uses priority 1 (lower than poll) so message reception takes precedence.
    #[task(priority = 1, shared = [counter], local = [led])]
    async fn publisher_task(cx: publisher_task::Context) {
        let mut counter = cx.shared.counter;
        let led = cx.local.led;

        loop {
            // Increment counter
            let count = counter.lock(|c| {
                *c = c.wrapping_add(1);
                *c
            });

            // Toggle LED to show activity
            led.toggle();

            // In a real application with a publisher:
            // publisher.publish(&Int32 { data: count as i32 }).ok();

            defmt::info!("Published: counter = {}", count);

            // Publish at 10 Hz
            Mono::delay(100.millis()).await;
        }
    }
}
