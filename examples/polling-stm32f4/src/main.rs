//! Bare-metal Polling Example for nano-ros on STM32F4
//!
//! This example demonstrates how to use nano-ros without any async runtime
//! (no RTIC, no Embassy). It uses a simple polling loop that's suitable for:
//!
//! - Very simple systems with minimal overhead
//! - Systems where you need full control over timing
//! - Learning/prototyping before adopting an executor
//!
//! # Architecture
//!
//! The main loop manually:
//! 1. Polls zenoh for incoming messages
//! 2. Sends keepalive messages
//! 3. Publishes data at a fixed rate
//!
//! Timing is done with a simple cycle counter. For production use,
//! consider using a hardware timer or upgrading to RTIC/Embassy.
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

use cortex_m_rt::entry;
use defmt_rtt as _;
use nano_ros::prelude::{Context, Executor, InitOptions, NodeNameExt, PollingExecutor};
use panic_halt as _;
use stm32f4xx_hal::{prelude::*, rcc::Config};

/// Timing constants for polling tasks (same as nano_ros_node::rtic when zenoh is enabled)
/// Poll interval in milliseconds - how often to poll zenoh for incoming messages
const POLL_INTERVAL_MS: u32 = 10;
/// Keepalive interval in milliseconds - how often to send keepalive to maintain session
const KEEPALIVE_INTERVAL_MS: u32 = 1000;

/// Simple timing tracker using DWT cycle counter
struct SimpleTimer {
    last_tick: u32,
    ticks_per_ms: u32,
}

impl SimpleTimer {
    fn new(sysclk_hz: u32) -> Self {
        Self {
            last_tick: 0,
            ticks_per_ms: sysclk_hz / 1000,
        }
    }

    /// Get current tick count (wraps around)
    fn now(&self) -> u32 {
        // Read DWT cycle counter (configured in main)
        cortex_m::peripheral::DWT::cycle_count()
    }

    /// Check if the given number of milliseconds have elapsed since last check
    fn elapsed_ms(&mut self, ms: u32) -> bool {
        let now = self.now();
        let elapsed_ticks = now.wrapping_sub(self.last_tick);
        let required_ticks = ms * self.ticks_per_ms;

        if elapsed_ticks >= required_ticks {
            self.last_tick = now;
            true
        } else {
            false
        }
    }
}

/// Application state for the polling loop
struct App {
    executor: PollingExecutor<1>,
    counter: u32,
    publish_timer: SimpleTimer,
}

impl App {
    fn new(sysclk_hz: u32, executor: PollingExecutor<1>) -> Self {
        Self {
            executor,
            counter: 0,
            publish_timer: SimpleTimer::new(sysclk_hz),
        }
    }
}

#[entry]
fn main() -> ! {
    defmt::info!("nano-ros polling example starting...");

    // Get access to device peripherals
    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Configure clocks using new stm32f4xx-hal 0.23 API
    let mut rcc = dp.RCC.freeze(Config::hse(8.MHz()).sysclk(168.MHz()));

    let sysclk_hz = rcc.clocks.sysclk().to_Hz();
    defmt::info!("Clocks configured: sysclk = {} Hz", sysclk_hz);

    // Enable DWT cycle counter for timing
    let mut dcb = cp.DCB;
    let mut dwt = cp.DWT;
    dcb.enable_trace();
    dwt.enable_cycle_counter();

    // Configure GPIO for LED (PB7 on NUCLEO-F429ZI)
    let gpiob = dp.GPIOB.split(&mut rcc);
    let mut led = gpiob.pb7.into_push_pull_output();

    // In a real application, you would:
    // 1. Initialize the network interface (Ethernet or UART)
    // 2. Create the context, executor, and node:
    //
    // let init_options = InitOptions::new().locator("tcp/192.168.1.1:7447");
    // let ctx = Context::new(init_options).unwrap();
    // let mut executor: PollingExecutor<1> = ctx.create_polling_executor();
    // let node = executor.create_node("polling_node".namespace("/demo")).unwrap();
    // let publisher = node.create_publisher::<Int32>("/counter").unwrap();

    // Initialize application state
    // let mut app = App::new(sysclk_hz, executor); // Uncomment for real hardware

    defmt::info!("Entering main polling loop...");

    // Main polling loop
    loop {
        // In a real application, call spin_once on the executor
        // let result = app.executor.spin_once(POLL_INTERVAL_MS as u64);
        // if result.subscriptions_processed > 0 {
        //     defmt::info!("Processed {} subscriptions", result.subscriptions_processed);
        // }

        // Task: Publish at 10 Hz (every 100ms)
        // Note: The main loop spin rate should be fast enough to accommodate this
        if app.publish_timer.elapsed_ms(100) {
            app.counter = app.counter.wrapping_add(1);

            // Toggle LED to show activity
            led.toggle();

            // In a real application:
            // publisher.publish(&Int32 { data: app.counter as i32 }).ok();

            defmt::info!("Published: counter = {}", app.counter);
        }

        // Optional: WFI to save power between polls
        // Note: This will wake on any interrupt, so timing may vary
        // cortex_m::asm::wfi();
    }
}
