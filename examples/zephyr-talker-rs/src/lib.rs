//! nano-ros Zephyr Talker Example (Rust)
//!
//! This example demonstrates a ROS 2 compatible publisher running on
//! Zephyr RTOS using nano-ros with zenoh-pico transport.
//!
//! The message format is CDR-serialized std_msgs/Int32, compatible
//! with ROS 2 nodes using rmw_zenoh.

#![no_std]
#![no_main]

use core::ffi::c_void;
use zephyr::raw;
use zephyr::printkln;
use zephyr::time::Duration;

// CDR serialization from nano-ros
use nano_ros_serdes::{CdrSerializer, Serialize};
use nano_ros_types::std_msgs::Int32;

/// CDR message buffer size
const MSG_BUF_SIZE: usize = 64;

/// Topic for publishing (ROS 2 rmw_zenoh format)
/// Format: <domain>/<topic>/<type>/<hash>
const TOPIC_KEY: &[u8] = b"0/chatter/std_msgs::msg::dds_::Int32_/TypeHashNotSupported\0";

/// zenoh-pico FFI bindings for Zephyr
/// These match the zenoh-pico C API
mod zenoh_ffi {
    use core::ffi::{c_char, c_int, c_void};

    // Opaque types
    #[repr(C)]
    pub struct z_owned_session_t {
        _data: [u8; 256],
    }

    #[repr(C)]
    pub struct z_owned_publisher_t {
        _data: [u8; 128],
    }

    #[repr(C)]
    pub struct z_owned_config_t {
        _data: [u8; 128],
    }

    #[repr(C)]
    pub struct z_owned_bytes_t {
        _data: [u8; 64],
    }

    #[repr(C)]
    pub struct z_view_keyexpr_t {
        _data: [u8; 32],
    }

    extern "C" {
        // Config
        pub fn z_config_default(config: *mut z_owned_config_t) -> c_int;
        pub fn z_config_insert(
            config: *mut z_owned_config_t,
            key: *const c_char,
            value: *const c_char,
        ) -> c_int;

        // Session
        pub fn z_open(
            session: *mut z_owned_session_t,
            config: *mut z_owned_config_t,
            opts: *const c_void,
        ) -> c_int;
        pub fn z_close(session: *mut z_owned_session_t, opts: *const c_void) -> c_int;
        pub fn z_session_loan(session: *const z_owned_session_t) -> *const c_void;

        // Key expression
        pub fn z_view_keyexpr_from_str(
            keyexpr: *mut z_view_keyexpr_t,
            s: *const c_char,
        ) -> c_int;
        pub fn z_view_keyexpr_loan(keyexpr: *const z_view_keyexpr_t) -> *const c_void;

        // Publisher
        pub fn z_declare_publisher(
            session: *const c_void,
            publisher: *mut z_owned_publisher_t,
            keyexpr: *const c_void,
            opts: *const c_void,
        ) -> c_int;
        pub fn z_publisher_loan(publisher: *const z_owned_publisher_t) -> *const c_void;
        pub fn z_undeclare_publisher(publisher: *mut z_owned_publisher_t) -> c_int;

        // Publish
        pub fn z_bytes_copy_from_buf(
            bytes: *mut z_owned_bytes_t,
            data: *const u8,
            len: usize,
        ) -> c_int;
        pub fn z_bytes_drop(bytes: *mut z_owned_bytes_t);
        pub fn z_publisher_put(
            publisher: *const c_void,
            payload: *mut z_owned_bytes_t,
            opts: *const c_void,
        ) -> c_int;

        // Read task (for receiving responses)
        pub fn zp_read(session: *const c_void, opts: *const c_void) -> c_int;
        pub fn zp_send_keep_alive(session: *const c_void, opts: *const c_void) -> c_int;
    }
}

/// Serialize an Int32 message to CDR format
fn serialize_int32(buf: &mut [u8], value: i32) -> Result<usize, ()> {
    let msg = Int32 { data: value };
    let mut serializer = CdrSerializer::new(buf);
    msg.serialize(&mut serializer).map_err(|_| ())?;
    Ok(serializer.position())
}

/// Main entry point for Zephyr
#[no_mangle]
pub extern "C" fn main() -> ! {
    printkln!("nano-ros Zephyr Talker (Rust) Starting");
    printkln!("Topic: /chatter (std_msgs/Int32)");

    // Initialize zenoh session
    let mut config: zenoh_ffi::z_owned_config_t = unsafe { core::mem::zeroed() };
    let mut session: zenoh_ffi::z_owned_session_t = unsafe { core::mem::zeroed() };
    let mut publisher: zenoh_ffi::z_owned_publisher_t = unsafe { core::mem::zeroed() };
    let mut keyexpr: zenoh_ffi::z_view_keyexpr_t = unsafe { core::mem::zeroed() };

    unsafe {
        // Configure zenoh
        if zenoh_ffi::z_config_default(&mut config) != 0 {
            printkln!("ERROR: Failed to create config");
            loop {
                zephyr::time::sleep(Duration::secs_f32(1.0));
            }
        }

        // Set client mode connecting to router
        let mode_key = b"mode\0";
        let mode_val = b"client\0";
        zenoh_ffi::z_config_insert(
            &mut config,
            mode_key.as_ptr() as *const _,
            mode_val.as_ptr() as *const _,
        );

        // Connect to zenoh router on host
        let connect_key = b"connect/endpoints\0";
        let connect_val = b"[\"tcp/192.0.2.2:7447\"]\0"; // Gateway IP
        zenoh_ffi::z_config_insert(
            &mut config,
            connect_key.as_ptr() as *const _,
            connect_val.as_ptr() as *const _,
        );

        // Open session
        printkln!("Connecting to zenoh router...");
        if zenoh_ffi::z_open(&mut session, &mut config, core::ptr::null()) != 0 {
            printkln!("ERROR: Failed to open session");
            loop {
                zephyr::time::sleep(Duration::secs_f32(1.0));
            }
        }
        printkln!("Connected to zenoh router");

        // Declare publisher
        if zenoh_ffi::z_view_keyexpr_from_str(&mut keyexpr, TOPIC_KEY.as_ptr() as *const _) != 0 {
            printkln!("ERROR: Invalid key expression");
            loop {
                zephyr::time::sleep(Duration::secs_f32(1.0));
            }
        }

        let session_loan = zenoh_ffi::z_session_loan(&session);
        let keyexpr_loan = zenoh_ffi::z_view_keyexpr_loan(&keyexpr);

        if zenoh_ffi::z_declare_publisher(
            session_loan,
            &mut publisher,
            keyexpr_loan,
            core::ptr::null(),
        ) != 0
        {
            printkln!("ERROR: Failed to declare publisher");
            loop {
                zephyr::time::sleep(Duration::secs_f32(1.0));
            }
        }
        printkln!("Publisher created for /chatter");
    }

    // Publish loop
    let mut counter: i32 = 0;
    let mut msg_buf = [0u8; MSG_BUF_SIZE];

    loop {
        // Serialize message using nano-ros CDR
        match serialize_int32(&mut msg_buf, counter) {
            Ok(len) => {
                printkln!(
                    "Publishing: {} (CDR {} bytes)",
                    counter,
                    len
                );

                unsafe {
                    // Create payload
                    let mut payload: zenoh_ffi::z_owned_bytes_t = core::mem::zeroed();
                    zenoh_ffi::z_bytes_copy_from_buf(&mut payload, msg_buf.as_ptr(), len);

                    // Publish
                    let pub_loan = zenoh_ffi::z_publisher_loan(&publisher);
                    if zenoh_ffi::z_publisher_put(pub_loan, &mut payload, core::ptr::null()) != 0 {
                        printkln!("WARN: Publish failed");
                    }

                    // Process zenoh events
                    let session_loan = zenoh_ffi::z_session_loan(&session);
                    zp_read(session_loan, core::ptr::null());
                    zp_send_keep_alive(session_loan, core::ptr::null());
                }
            }
            Err(_) => {
                printkln!("ERROR: Serialization failed");
            }
        }

        counter = counter.wrapping_add(1);
        zephyr::time::sleep(Duration::secs_f32(1.0));
    }
}

// FFI helpers
use zenoh_ffi::{zp_read, zp_send_keep_alive};

// Panic handler for no_std
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    printkln!("PANIC: {:?}", info);
    loop {}
}
