//! nano-ros Zephyr Listener Example (Rust)
//!
//! This example demonstrates a ROS 2 compatible subscriber running on
//! Zephyr RTOS using nano-ros with zenoh-pico transport.
//!
//! Receives CDR-serialized std_msgs/Int32 messages, compatible
//! with ROS 2 nodes using rmw_zenoh.

#![no_std]
#![no_main]

use core::ffi::c_void;
use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};
use zephyr::printkln;
use zephyr::time::Duration;

// CDR deserialization from nano-ros
use nano_ros_serdes::{CdrDeserializer, Deserialize};
use nano_ros_types::std_msgs::Int32;

/// Received message counter
static MSG_COUNT: AtomicU32 = AtomicU32::new(0);
static LAST_VALUE: AtomicI32 = AtomicI32::new(0);

/// Topic for subscribing (ROS 2 rmw_zenoh format with wildcard for type hash)
/// Format: <domain>/<topic>/<type>/*
const TOPIC_KEY: &[u8] = b"0/chatter/std_msgs::msg::dds_::Int32_/*\0";

/// zenoh-pico FFI bindings for Zephyr
mod zenoh_ffi {
    use core::ffi::{c_char, c_int, c_void};

    // Opaque types
    #[repr(C)]
    pub struct z_owned_session_t {
        _data: [u8; 256],
    }

    #[repr(C)]
    pub struct z_owned_subscriber_t {
        _data: [u8; 128],
    }

    #[repr(C)]
    pub struct z_owned_config_t {
        _data: [u8; 128],
    }

    #[repr(C)]
    pub struct z_view_keyexpr_t {
        _data: [u8; 32],
    }

    #[repr(C)]
    pub struct z_loaned_sample_t {
        _data: [u8; 128],
    }

    #[repr(C)]
    pub struct z_loaned_bytes_t {
        _data: [u8; 64],
    }

    // Closure type for subscriber callback
    pub type z_closure_sample_callback_t =
        Option<unsafe extern "C" fn(*const z_loaned_sample_t, *mut c_void)>;

    #[repr(C)]
    pub struct z_owned_closure_sample_t {
        pub call: z_closure_sample_callback_t,
        pub drop: Option<unsafe extern "C" fn(*mut c_void)>,
        pub context: *mut c_void,
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
        pub fn z_view_keyexpr_from_str(keyexpr: *mut z_view_keyexpr_t, s: *const c_char) -> c_int;
        pub fn z_view_keyexpr_loan(keyexpr: *const z_view_keyexpr_t) -> *const c_void;

        // Subscriber
        pub fn z_declare_subscriber(
            session: *const c_void,
            subscriber: *mut z_owned_subscriber_t,
            keyexpr: *const c_void,
            callback: *const z_owned_closure_sample_t,
            opts: *const c_void,
        ) -> c_int;
        pub fn z_undeclare_subscriber(subscriber: *mut z_owned_subscriber_t) -> c_int;

        // Sample access
        pub fn z_sample_payload(sample: *const z_loaned_sample_t) -> *const z_loaned_bytes_t;
        pub fn z_bytes_len(bytes: *const z_loaned_bytes_t) -> usize;
        pub fn z_bytes_data(bytes: *const z_loaned_bytes_t) -> *const u8;

        // Read task
        pub fn zp_read(session: *const c_void, opts: *const c_void) -> c_int;
        pub fn zp_send_keep_alive(session: *const c_void, opts: *const c_void) -> c_int;
    }
}

/// Subscriber callback - called when a message is received
unsafe extern "C" fn subscriber_callback(
    sample: *const zenoh_ffi::z_loaned_sample_t,
    _context: *mut c_void,
) {
    let payload = zenoh_ffi::z_sample_payload(sample);
    let len = zenoh_ffi::z_bytes_len(payload);
    let data = zenoh_ffi::z_bytes_data(payload);

    if data.is_null() || len == 0 {
        printkln!("WARN: Empty payload received");
        return;
    }

    // Create slice from payload
    let buf = core::slice::from_raw_parts(data, len);

    // Deserialize using nano-ros CDR
    match deserialize_int32(buf) {
        Ok(value) => {
            let count = MSG_COUNT.fetch_add(1, Ordering::SeqCst) + 1;
            LAST_VALUE.store(value, Ordering::SeqCst);
            printkln!("[{}] Received: data={}", count, value);
        }
        Err(_) => {
            printkln!("ERROR: Failed to deserialize message ({} bytes)", len);
        }
    }
}

/// Deserialize an Int32 message from CDR format
fn deserialize_int32(buf: &[u8]) -> Result<i32, ()> {
    let mut deserializer = CdrDeserializer::new(buf);
    let msg = Int32::deserialize(&mut deserializer).map_err(|_| ())?;
    Ok(msg.data)
}

/// Main entry point for Zephyr
#[no_mangle]
pub extern "C" fn main() -> ! {
    printkln!("nano-ros Zephyr Listener (Rust) Starting");
    printkln!("==========================================");
    printkln!("Subscribing to: /chatter (std_msgs/Int32)");

    // Initialize zenoh session
    let mut config: zenoh_ffi::z_owned_config_t = unsafe { core::mem::zeroed() };
    let mut session: zenoh_ffi::z_owned_session_t = unsafe { core::mem::zeroed() };
    let mut subscriber: zenoh_ffi::z_owned_subscriber_t = unsafe { core::mem::zeroed() };
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

        // Create key expression
        if zenoh_ffi::z_view_keyexpr_from_str(&mut keyexpr, TOPIC_KEY.as_ptr() as *const _) != 0 {
            printkln!("ERROR: Invalid key expression");
            loop {
                zephyr::time::sleep(Duration::secs_f32(1.0));
            }
        }

        // Create closure for callback
        let closure = zenoh_ffi::z_owned_closure_sample_t {
            call: Some(subscriber_callback),
            drop: None,
            context: core::ptr::null_mut(),
        };

        // Declare subscriber
        let session_loan = zenoh_ffi::z_session_loan(&session);
        let keyexpr_loan = zenoh_ffi::z_view_keyexpr_loan(&keyexpr);

        if zenoh_ffi::z_declare_subscriber(
            session_loan,
            &mut subscriber,
            keyexpr_loan,
            &closure,
            core::ptr::null(),
        ) != 0
        {
            printkln!("ERROR: Failed to declare subscriber");
            loop {
                zephyr::time::sleep(Duration::secs_f32(1.0));
            }
        }
        printkln!("Subscriber created for /chatter");
        printkln!("Waiting for Int32 messages...");
        printkln!("(Press Ctrl+C to exit)");
    }

    // Event loop - process zenoh events
    loop {
        unsafe {
            let session_loan = zenoh_ffi::z_session_loan(&session);

            // Process incoming messages
            zenoh_ffi::zp_read(session_loan, core::ptr::null());

            // Send keep-alive
            zenoh_ffi::zp_send_keep_alive(session_loan, core::ptr::null());
        }

        // Small sleep to avoid busy-waiting
        zephyr::time::sleep(Duration::millis(10));
    }
}

// Panic handler for no_std
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    printkln!("PANIC: {:?}", info);
    loop {}
}
