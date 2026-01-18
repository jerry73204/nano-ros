//! nano-ros Zephyr Listener Example (Rust)
//!
//! This example demonstrates a ROS 2 compatible subscriber running on
//! Zephyr RTOS using zenoh-pico for communication.

#![no_std]

use core::ffi::{c_char, c_void};
use core::sync::atomic::{AtomicU32, Ordering};

use log::{error, info};

// nano-ros CDR deserialization
use nano_ros_serdes::{CdrReader, Deserialize};
use nano_ros_types::std_msgs::Int32;

// =============================================================================
// zenoh-pico shim FFI (from zenoh_shim.c)
// =============================================================================

type SampleCallback = extern "C" fn(data: *const u8, len: usize, ctx: *mut c_void);

extern "C" {
    fn zenoh_init_config(locator: *const c_char) -> i32;
    fn zenoh_open_session() -> i32;
    fn zenoh_declare_subscriber(keyexpr: *const c_char, callback: SampleCallback, ctx: *mut c_void) -> i32;
    fn zenoh_close();
}

// =============================================================================
// Callback and message handling
// =============================================================================

/// Counter for received messages
static MSG_COUNT: AtomicU32 = AtomicU32::new(0);

/// Deserialize an Int32 message from CDR format
fn deserialize_int32(buf: &[u8]) -> Result<i32, ()> {
    let mut reader = CdrReader::new(buf);
    let msg = Int32::deserialize(&mut reader).map_err(|_| ())?;
    Ok(msg.data)
}

/// Callback invoked when a message is received
extern "C" fn on_sample(data: *const u8, len: usize, _ctx: *mut c_void) {
    let count = MSG_COUNT.fetch_add(1, Ordering::Relaxed);

    // Safety: data is valid for len bytes, provided by C shim
    let payload = unsafe { core::slice::from_raw_parts(data, len) };

    match deserialize_int32(payload) {
        Ok(value) => {
            info!("[{}] Received: data={} ({} bytes)", count, value, len);
        }
        Err(_) => {
            info!("[{}] Received {} bytes (deserialization failed)", count, len);
        }
    }
}

// =============================================================================
// Main entry point
// =============================================================================

/// Entry point for Zephyr (called by zephyr-lang-rust)
#[no_mangle]
extern "C" fn rust_main() {
    // Initialize logging
    unsafe {
        zephyr::set_logger().ok();
    }

    info!("nano-ros Zephyr Listener (Rust) Starting");
    info!("Board: {}", zephyr::kconfig::CONFIG_BOARD);

    // zenoh connection parameters
    // For QEMU/native_sim, connect to host at 192.0.2.2
    let locator = b"tcp/192.0.2.2:7447\0";
    let keyexpr = b"demo/chatter\0";

    info!(
        "Connecting to zenoh router at {}",
        core::str::from_utf8(&locator[..locator.len() - 1]).unwrap_or("?")
    );

    // Initialize zenoh
    unsafe {
        let ret = zenoh_init_config(locator.as_ptr() as *const c_char);
        if ret < 0 {
            error!("Failed to init config: {}", ret);
            return;
        }

        info!("Opening zenoh session...");
        let ret = zenoh_open_session();
        if ret < 0 {
            error!("Failed to open session: {}", ret);
            return;
        }
        info!("Session opened");

        info!(
            "Subscribing to '{}'...",
            core::str::from_utf8(&keyexpr[..keyexpr.len() - 1]).unwrap_or("?")
        );
        let ret = zenoh_declare_subscriber(
            keyexpr.as_ptr() as *const c_char,
            on_sample,
            core::ptr::null_mut(),
        );
        if ret < 0 {
            error!("Failed to declare subscriber: {}", ret);
            zenoh_close();
            return;
        }
        info!("Subscriber declared");
    }

    info!("Waiting for messages...");

    // Keep the main thread alive - callbacks run in background
    loop {
        zephyr::time::sleep(zephyr::time::Duration::secs(10));
        let count = MSG_COUNT.load(Ordering::Relaxed);
        info!("Total messages received: {}", count);
    }
}
