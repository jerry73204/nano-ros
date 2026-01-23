//! RTIC Example for nano-ros on STM32F4
//!
//! This example demonstrates the PollingExecutor pattern for nano-ros with RTIC
//! on an STM32F4 microcontroller. It shows:
//!
//! - The `Context → PollingExecutor → Node` API pattern
//! - Using `spin_once()` in periodic RTIC tasks (no background threads)
//! - Callback-based subscriptions invoked during `spin_once()`
//! - Priority-based preemptive scheduling
//!
//! # PollingExecutor Pattern
//!
//! The PollingExecutor is designed for `no_std` embedded environments:
//!
//! ```ignore
//! // In init():
//! let ctx = Context::new(InitOptions::new().locator("tcp/192.168.1.1:7447"))?;
//! let mut executor: PollingExecutor<1> = ctx.create_polling_executor();
//! let node = executor.create_node("my_node")?;
//!
//! // Create subscription with callback (invoked during spin_once)
//! node.create_subscription::<Int32>("/topic", handle_message as fn(&Int32))?;
//!
//! // In periodic task (e.g., every 10ms):
//! let result = executor.spin_once(10); // 10ms delta
//! // result.subscriptions_processed shows how many callbacks were invoked
//! ```
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
//! # Note
//!
//! The actual zenoh integration is commented out because it requires a C
//! cross-compilation toolchain (e.g., arm-none-eabi-gcc) for zenoh-pico.
//! The example demonstrates the correct RTIC task structure.

#![no_std]
#![no_main]

use panic_halt as _;

use rtic::app;
use rtic_monotonics::systick::prelude::*;

// Generate the monotonic timer
systick_monotonic!(Mono, 1000); // 1kHz tick rate

/// Poll interval in milliseconds - how often to poll zenoh for incoming messages
/// and process subscription callbacks via spin_once()
const POLL_INTERVAL_MS: u32 = 10;

/// Timer process interval - passed to spin_once() for timer delta calculation
/// Currently unused as zenoh code is commented out pending cross-compilation support
#[allow(dead_code)]
const TIMER_PROCESS_INTERVAL_MS: u64 = 10;

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1, SPI2, SPI3])]
mod app {
    use super::*;
    use defmt_rtt as _;
    // When zenoh is available with embedded cross-compilation:
    // use nano_ros_node::{
    //     Context, Executor, InitOptions, NodeHandle, NodeNameExt, PollingExecutor,
    //     SpinOnceResult,
    // };
    use stm32f4xx_hal::{
        gpio::{Output, Pin},
        prelude::*,
        rcc::Config,
    };

    /// Shared resources accessible by multiple tasks
    #[shared]
    struct Shared {
        // PollingExecutor manages all nano-ros nodes and their callbacks
        // It's placed in shared resources to be accessible by tasks
        // executor: PollingExecutor<1>, // Allow 1 node, uses heapless::Vec internally
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

        // Configure clocks
        let mut rcc = cx.device.RCC.freeze(Config::hse(8.MHz()).sysclk(168.MHz()));

        defmt::info!(
            "Clocks configured: sysclk = {} Hz",
            rcc.clocks.sysclk().to_Hz()
        );

        // Configure GPIO for LED (PB7 on NUCLEO-F429ZI)
        let gpiob = cx.device.GPIOB.split(&mut rcc);
        let led = gpiob.pb7.into_push_pull_output();

        // ═══════════════════════════════════════════════════════════════════
        // NANO-ROS SETUP (when zenoh cross-compilation is available)
        // ═══════════════════════════════════════════════════════════════════
        //
        // // 1. Create context with transport configuration
        // let ctx = Context::new(
        //     InitOptions::new().locator("tcp/192.168.1.1:7447")
        // ).expect("Failed to create context");
        //
        // // 2. Create PollingExecutor (no_std compatible, uses heapless)
        // let mut executor: PollingExecutor<1> = ctx.create_polling_executor();
        //
        // // 3. Create node through executor
        // let node = executor
        //     .create_node("rtic_node".namespace("/demo"))
        //     .expect("Failed to create node");
        //
        // // 4. Create subscription with callback
        // // The callback is invoked during spin_once()
        // node.create_subscription::<Int32>(
        //     SubscriberOptions::new("/commands"),
        //     handle_command as fn(&Int32),  // Function pointer (no alloc needed)
        // ).expect("Failed to create subscription");
        //
        // // 5. Create publisher for outgoing messages
        // let publisher = node
        //     .create_publisher::<Int32>(PublisherOptions::new("/sensor_data"))
        //     .expect("Failed to create publisher");

        defmt::info!("Starting periodic tasks...");

        // Spawn the periodic tasks
        zenoh_poll::spawn().ok();
        publisher_task::spawn().ok();

        (
            Shared {
                // executor,
                counter: 0,
            },
            Local { led },
        )
    }

    // Message handler for subscriptions (function pointer, no alloc)
    // fn handle_command(msg: &Int32) {
    //     defmt::info!("Received command: {}", msg.data);
    // }

    /// Idle task - runs when no other tasks are ready
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi(); // Wait for interrupt - saves power
        }
    }

    /// Poll zenoh and process callbacks via spin_once()
    ///
    /// This task runs every 10ms to:
    /// - Poll the zenoh transport for incoming messages
    /// - Invoke subscription callbacks for received messages
    /// - Process timer callbacks
    ///
    /// Higher priority (2) ensures timely message processing.
    #[task(priority = 2, shared = [/* executor */])]
    async fn zenoh_poll(_cx: zenoh_poll::Context) {
        loop {
            // Process all pending callbacks with spin_once()
            // cx.shared.executor.lock(|exec| {
            //     let result: SpinOnceResult = exec.spin_once(TIMER_PROCESS_INTERVAL_MS);
            //
            //     // SpinOnceResult contains:
            //     // - subscriptions_processed: messages received and callbacks invoked
            //     // - timers_fired: timer callbacks that fired
            //     // - services_handled: service requests processed
            //
            //     if result.any_work() {
            //         defmt::debug!(
            //             "spin_once: {} subs, {} timers, {} services",
            //             result.subscriptions_processed,
            //             result.timers_fired,
            //             result.services_handled
            //         );
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

            // Publish message (when zenoh is available):
            // publisher.publish(&Int32 { data: count as i32 }).ok();

            defmt::info!("Published: counter = {}", count);

            // Publish at 10 Hz
            Mono::delay(100.millis()).await;
        }
    }
}
