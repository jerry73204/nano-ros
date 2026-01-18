//! nano-ros Zephyr Talker Example (Rust)
//!
//! This example demonstrates a ROS 2 compatible publisher running on
//! Zephyr RTOS using zenoh-pico for communication.

#![no_std]

use core::ffi::c_char;

use log::{error, info};

// nano-ros CDR serialization
use nano_ros_serdes::{CdrWriter, Serialize};
use nano_ros_types::std_msgs::Int32;

// =============================================================================
// zenoh-pico shim FFI (from zenoh_shim.c)
// =============================================================================

extern "C" {
    fn zenoh_init_config(locator: *const c_char) -> i32;
    fn zenoh_open_session() -> i32;
    fn zenoh_declare_publisher(keyexpr: *const c_char) -> i32;
    fn zenoh_publish(data: *const u8, len: usize) -> i32;
    fn zenoh_close();
}

// =============================================================================
// Helper functions
// =============================================================================

/// Serialize an Int32 message to CDR format
fn serialize_int32(buf: &mut [u8], value: i32) -> Result<usize, ()> {
    let msg = Int32 { data: value };
    let mut writer = CdrWriter::new(buf);
    msg.serialize(&mut writer).map_err(|_| ())?;
    Ok(writer.position())
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

    info!("nano-ros Zephyr Talker (Rust) Starting");
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
            "Declaring publisher for '{}'...",
            core::str::from_utf8(&keyexpr[..keyexpr.len() - 1]).unwrap_or("?")
        );
        let ret = zenoh_declare_publisher(keyexpr.as_ptr() as *const c_char);
        if ret < 0 {
            error!("Failed to declare publisher: {}", ret);
            zenoh_close();
            return;
        }
        info!("Publisher declared");
    }

    // Message buffer for CDR serialization
    let mut msg_buf = [0u8; 64];
    let mut counter: i32 = 0;

    info!("Starting publish loop...");

    loop {
        // Serialize message using nano-ros CDR
        match serialize_int32(&mut msg_buf, counter) {
            Ok(len) => unsafe {
                let ret = zenoh_publish(msg_buf.as_ptr(), len);
                if ret < 0 {
                    error!("Failed to publish: {}", ret);
                } else {
                    info!("[{}] Published: data={} ({} bytes)", counter, counter, len);
                }
            },
            Err(_) => {
                error!("Serialization failed");
            }
        }

        counter = counter.wrapping_add(1);

        // Sleep 1 second
        zephyr::time::sleep(zephyr::time::Duration::secs(1));
    }
}
