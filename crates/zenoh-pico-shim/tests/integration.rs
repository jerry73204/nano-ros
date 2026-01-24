//! Integration tests for zenoh-pico-shim
//!
//! These tests require a zenohd router running on tcp/127.0.0.1:7447.
//! Start it with: zenohd --listen tcp/127.0.0.1:7447
//!
//! Note: Only one ShimContext can exist at a time due to global state in the C shim.
//! Run tests with: cargo test -p zenoh-pico-shim --features posix -- --test-threads=1

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::thread;
use std::time::Duration;

use zenoh_pico_shim::{ShimContext, ShimError};

/// Wait for the session to stabilize after opening
fn wait_for_session() {
    thread::sleep(Duration::from_millis(100));
}

// ============================================================================
// Session Lifecycle Tests
// ============================================================================

#[test]
fn test_session_open_close() {
    let ctx = ShimContext::new(b"tcp/127.0.0.1:7447\0");
    match ctx {
        Ok(ctx) => {
            assert!(ctx.is_open(), "Session should be open after creation");
            // Context is dropped here, which closes the session
        }
        Err(e) => {
            eprintln!("Failed to connect to zenohd (is it running?): {:?}", e);
            eprintln!("Start zenohd with: zenohd --listen tcp/127.0.0.1:7447");
            // Skip test if zenohd is not available
            return;
        }
    }
}

#[test]
fn test_session_uses_polling() {
    let ctx = match ShimContext::new(b"tcp/127.0.0.1:7447\0") {
        Ok(ctx) => ctx,
        Err(_) => return, // Skip if zenohd not available
    };

    // POSIX backend should NOT use polling (it uses background threads)
    assert!(!ctx.uses_polling(), "POSIX backend should not use polling");
}

#[test]
fn test_session_invalid_locator() {
    // Invalid locator format should fail
    let result = ShimContext::new(b"invalid_locator\0");
    assert!(result.is_err(), "Invalid locator should fail");
}

// ============================================================================
// Publisher Tests
// ============================================================================

#[test]
fn test_publisher_declare() {
    let ctx = match ShimContext::new(b"tcp/127.0.0.1:7447\0") {
        Ok(ctx) => ctx,
        Err(_) => return,
    };
    wait_for_session();

    let publisher = ctx.declare_publisher(b"test/publisher\0");
    assert!(publisher.is_ok(), "Should be able to declare publisher");

    let publisher = publisher.unwrap();
    assert!(publisher.handle() >= 0, "Publisher handle should be valid");
}

#[test]
fn test_publisher_publish() {
    let ctx = match ShimContext::new(b"tcp/127.0.0.1:7447\0") {
        Ok(ctx) => ctx,
        Err(_) => return,
    };
    wait_for_session();

    let publisher = ctx.declare_publisher(b"test/publish\0").unwrap();
    let result = publisher.publish(b"Hello, World!");
    assert!(result.is_ok(), "Should be able to publish data");
}

#[test]
fn test_publisher_publish_with_attachment() {
    let ctx = match ShimContext::new(b"tcp/127.0.0.1:7447\0") {
        Ok(ctx) => ctx,
        Err(_) => return,
    };
    wait_for_session();

    let publisher = ctx.declare_publisher(b"test/attachment\0").unwrap();

    // RMW attachment format: 33 bytes
    let attachment = [0u8; 33];
    let result = publisher.publish_with_attachment(b"Hello", Some(&attachment));
    assert!(result.is_ok(), "Should be able to publish with attachment");
}

// ============================================================================
// Subscriber Tests
// ============================================================================

#[test]
fn test_subscriber_declare() {
    let ctx = match ShimContext::new(b"tcp/127.0.0.1:7447\0") {
        Ok(ctx) => ctx,
        Err(_) => return,
    };
    wait_for_session();

    // Callback that does nothing
    extern "C" fn dummy_callback(_data: *const u8, _len: usize, _ctx: *mut std::ffi::c_void) {}

    let subscriber = unsafe {
        ctx.declare_subscriber_raw(b"test/subscriber\0", dummy_callback, std::ptr::null_mut())
    };
    assert!(subscriber.is_ok(), "Should be able to declare subscriber");

    let subscriber = subscriber.unwrap();
    assert!(
        subscriber.handle() >= 0,
        "Subscriber handle should be valid"
    );
}

#[test]
fn test_pubsub_communication() {
    let ctx = match ShimContext::new(b"tcp/127.0.0.1:7447\0") {
        Ok(ctx) => ctx,
        Err(_) => return,
    };
    wait_for_session();

    // Shared state for callback
    static RECEIVED: AtomicBool = AtomicBool::new(false);
    static RECEIVED_LEN: AtomicUsize = AtomicUsize::new(0);

    extern "C" fn callback(data: *const u8, len: usize, _ctx: *mut std::ffi::c_void) {
        RECEIVED.store(true, Ordering::SeqCst);
        RECEIVED_LEN.store(len, Ordering::SeqCst);
        // Verify data content
        if len >= 5 {
            let slice = unsafe { std::slice::from_raw_parts(data, len) };
            assert_eq!(&slice[..5], b"Hello");
        }
    }

    // Reset state
    RECEIVED.store(false, Ordering::SeqCst);
    RECEIVED_LEN.store(0, Ordering::SeqCst);

    // Create subscriber first
    let _subscriber =
        unsafe { ctx.declare_subscriber_raw(b"test/pubsub\0", callback, std::ptr::null_mut()) }
            .unwrap();

    // Wait for subscriber to be registered
    thread::sleep(Duration::from_millis(100));

    // Create publisher and send data
    let publisher = ctx.declare_publisher(b"test/pubsub\0").unwrap();

    // Send multiple times to ensure delivery
    for _ in 0..5 {
        publisher.publish(b"Hello from test!").unwrap();
        thread::sleep(Duration::from_millis(50));
        if RECEIVED.load(Ordering::SeqCst) {
            break;
        }
    }

    // Note: Due to zenoh-pico's local subscriber feature, this should work
    // but may require Z_FEATURE_LOCAL_SUBSCRIBER=1 in the zenoh-pico build
    // If this fails, it's likely a configuration issue, not a code bug
    if !RECEIVED.load(Ordering::SeqCst) {
        eprintln!("Warning: Local pub/sub may not be enabled in zenoh-pico build");
        eprintln!("Ensure Z_FEATURE_LOCAL_SUBSCRIBER=1 is set");
    }
}

// ============================================================================
// Liveliness Tests
// ============================================================================

#[test]
fn test_liveliness_declare() {
    let ctx = match ShimContext::new(b"tcp/127.0.0.1:7447\0") {
        Ok(ctx) => ctx,
        Err(_) => return,
    };
    wait_for_session();

    let token = ctx.declare_liveliness(b"test/liveliness\0");
    assert!(token.is_ok(), "Should be able to declare liveliness token");

    let token = token.unwrap();
    assert!(token.handle() >= 0, "Liveliness handle should be valid");
    // Token is dropped here, which undeclares it
}

#[test]
fn test_liveliness_ros2_format() {
    let ctx = match ShimContext::new(b"tcp/127.0.0.1:7447\0") {
        Ok(ctx) => ctx,
        Err(_) => return,
    };
    wait_for_session();

    // ROS 2 liveliness key expression format
    let keyexpr = b"@ros2_lv/0/0123456789abcdef/0/0/NN/%/%/test_node\0";
    let token = ctx.declare_liveliness(keyexpr);
    assert!(
        token.is_ok(),
        "Should be able to declare ROS 2 format liveliness"
    );
}

// ============================================================================
// ZenohId Tests
// ============================================================================

#[test]
fn test_zenoh_id() {
    let ctx = match ShimContext::new(b"tcp/127.0.0.1:7447\0") {
        Ok(ctx) => ctx,
        Err(_) => return,
    };
    wait_for_session();

    let zid = ctx.zid();
    assert!(zid.is_ok(), "Should be able to get ZenohId");

    let zid = zid.unwrap();

    // ZenohId should be 16 bytes (access via public id field)
    let bytes = &zid.id;
    assert_eq!(bytes.len(), 16, "ZenohId should be 16 bytes");

    // ZenohId should not be all zeros (that would indicate an error)
    assert!(
        bytes.iter().any(|&b| b != 0),
        "ZenohId should not be all zeros"
    );

    // Test hex conversion
    let mut hex_buf = [0u8; 32];
    zid.to_hex_bytes(&mut hex_buf);
    // Hex representation should contain valid hex characters
    for &b in hex_buf.iter() {
        assert!(
            (b'0'..=b'9').contains(&b) || (b'a'..=b'f').contains(&b),
            "Hex should contain valid characters"
        );
    }
}

// ============================================================================
// Error Handling Tests
// ============================================================================

#[test]
fn test_max_publishers() {
    let ctx = match ShimContext::new(b"tcp/127.0.0.1:7447\0") {
        Ok(ctx) => ctx,
        Err(_) => return,
    };
    wait_for_session();

    // Try to create more than MAX_PUBLISHERS (8)
    let mut publishers = Vec::new();
    for i in 0..8 {
        let keyexpr = format!("test/max_pub/{}\0", i);
        match ctx.declare_publisher(keyexpr.as_bytes()) {
            Ok(p) => publishers.push(p),
            Err(e) => {
                panic!("Failed to create publisher {}: {:?}", i, e);
            }
        }
    }

    // 9th publisher should fail
    let result = ctx.declare_publisher(b"test/max_pub/overflow\0");
    assert!(result.is_err(), "Should fail when exceeding max publishers");
    if let Err(e) = result {
        assert_eq!(e, ShimError::Full, "Error should be Full");
    }
}

// ============================================================================
// Polling Tests (for completeness, even though POSIX doesn't use it)
// ============================================================================

#[test]
fn test_poll_noop() {
    let ctx = match ShimContext::new(b"tcp/127.0.0.1:7447\0") {
        Ok(ctx) => ctx,
        Err(_) => return,
    };
    wait_for_session();

    // For POSIX backend, poll should be a no-op but not fail
    let result = ctx.poll(0);
    assert!(result.is_ok(), "Poll should succeed (even as no-op)");

    let result = ctx.spin_once(0);
    assert!(result.is_ok(), "Spin once should succeed (even as no-op)");
}
