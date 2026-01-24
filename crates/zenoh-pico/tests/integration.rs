//! Integration tests for zenoh-pico
//!
//! Tests requiring a zenoh router will automatically start zenohd if available.
//! If zenohd is not installed, those tests are skipped gracefully.
//!
//! To install zenohd: https://zenoh.io/docs/getting-started/installation/

mod common;

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use common::{ensure_router, open_session_with_retry};
use zenoh_pico::{Config, Error, KeyExpr, Sample, Session};

/// Helper macro for tests that require a router.
/// Skips the test if zenohd is not available.
macro_rules! require_router {
    () => {
        if let Err(e) = ensure_router() {
            eprintln!("Skipping test: {}", e);
            return;
        }
    };
}

// ============================================================================
// Error Tests
// ============================================================================

/// Test Error Display implementation
#[test]
fn test_error_display() {
    assert_eq!(
        format!("{}", Error::SessionOpenFailed),
        "Failed to open session"
    );
    assert_eq!(format!("{}", Error::SessionClosed), "Session is closed");
    assert_eq!(format!("{}", Error::InvalidConfig), "Invalid configuration");
    assert_eq!(
        format!("{}", Error::InvalidKeyExpr),
        "Invalid key expression"
    );
    assert_eq!(
        format!("{}", Error::PublisherDeclarationFailed),
        "Failed to declare publisher"
    );
    assert_eq!(
        format!("{}", Error::SubscriberDeclarationFailed),
        "Failed to declare subscriber"
    );
    assert_eq!(
        format!("{}", Error::PublishFailed),
        "Failed to publish data"
    );
    assert_eq!(
        format!("{}", Error::TaskStartFailed),
        "Failed to start background task"
    );
    assert_eq!(format!("{}", Error::BufferTooSmall), "Buffer is too small");
    assert_eq!(format!("{}", Error::ZenohError(-5)), "Zenoh error: -5");
}

/// Test Error Clone and PartialEq
#[test]
fn test_error_clone_eq() {
    let e1 = Error::SessionOpenFailed;
    let e2 = e1.clone();
    assert_eq!(e1, e2);

    let e3 = Error::ZenohError(-1);
    let e4 = Error::ZenohError(-1);
    let e5 = Error::ZenohError(-2);
    assert_eq!(e3, e4);
    assert_ne!(e3, e5);
}

/// Test Error Copy trait
#[test]
fn test_error_copy() {
    let e1 = Error::InvalidKeyExpr;
    let e2 = e1; // Copy
    let _e3 = e1; // Can use e1 again because it's Copy
    assert_eq!(e1, e2);
}

/// Test Error Debug
#[test]
fn test_error_debug() {
    let e = Error::SessionOpenFailed;
    let debug_str = format!("{:?}", e);
    assert!(debug_str.contains("SessionOpenFailed"));
}

/// Test std::error::Error implementation
#[test]
fn test_error_std_error() {
    fn assert_error<E: std::error::Error>(_: &E) {}
    let e = Error::InvalidConfig;
    assert_error(&e);
}

// ============================================================================
// Config Tests
// ============================================================================

/// Test that we can create a configuration
#[test]
fn test_config_creation() {
    let config = Config::new().expect("Failed to create config");
    drop(config);
}

/// Test client configuration
#[test]
fn test_client_config() {
    let config = Config::client("tcp/127.0.0.1:7447").expect("Failed to create client config");
    drop(config);
}

/// Test peer configuration
#[test]
fn test_peer_config() {
    let config = Config::peer().expect("Failed to create peer config");
    drop(config);
}

/// Test Config::default()
#[test]
fn test_config_default() {
    let config = Config::default();
    drop(config);
}

/// Test client config with different locator formats
#[test]
fn test_client_config_locators() {
    // TCP locator
    let config = Config::client("tcp/192.168.1.1:7447").expect("TCP locator");
    drop(config);

    // UDP locator
    let config = Config::client("udp/192.168.1.1:7447").expect("UDP locator");
    drop(config);

    // TCP with IPv6 (bracketed)
    let config = Config::client("tcp/[::1]:7447").expect("IPv6 locator");
    drop(config);

    // Multiple locators (comma separated might not work, but test doesn't crash)
    let config = Config::client("tcp/localhost:7447").expect("localhost locator");
    drop(config);
}

/// Test config with long locator (truncation behavior)
#[test]
fn test_config_long_locator() {
    // Create a locator longer than 255 chars - should be truncated but not crash
    let long_locator = format!("tcp/{}.example.com:7447", "a".repeat(300));
    let config = Config::client(&long_locator);
    // Config creation should succeed (locator is truncated internally)
    assert!(config.is_ok());
    drop(config);
}

// ============================================================================
// KeyExpr Tests
// ============================================================================

/// Test key expression creation
#[test]
fn test_keyexpr() {
    let keyexpr = KeyExpr::new("demo/example").expect("Failed to create keyexpr");
    assert!(format!("{:?}", keyexpr).contains("demo/example"));
}

/// Test invalid key expression
#[test]
fn test_invalid_keyexpr() {
    // Very long key should fail
    let long_key = "a".repeat(300);
    let result = KeyExpr::new(&long_key);
    assert!(result.is_err());
}

/// Test key expression at boundary (255 chars - just under limit)
#[test]
fn test_keyexpr_boundary() {
    // 255 chars should work (buffer is 256, need null terminator)
    let key = "a".repeat(255);
    let result = KeyExpr::new(&key);
    assert!(result.is_ok());
}

/// Test key expression exactly at limit
#[test]
fn test_keyexpr_at_limit() {
    // 256 chars should fail (buffer is 256, no room for null terminator)
    let key = "a".repeat(256);
    let result = KeyExpr::new(&key);
    assert!(result.is_err());
}

/// Test wildcard key expressions
#[test]
fn test_keyexpr_wildcards() {
    // Single-level wildcard
    let keyexpr = KeyExpr::new("demo/*/example").expect("Single wildcard");
    assert!(format!("{:?}", keyexpr).contains("demo/*/example"));

    // Multi-level wildcard
    let keyexpr = KeyExpr::new("demo/**").expect("Double wildcard");
    assert!(format!("{:?}", keyexpr).contains("demo/**"));
}

/// Test key expression with special characters
#[test]
fn test_keyexpr_special_chars() {
    // Underscores and dashes
    let keyexpr = KeyExpr::new("demo/test-topic_v2").expect("Underscores and dashes");
    assert!(format!("{:?}", keyexpr).contains("demo/test-topic_v2"));

    // Numbers
    let keyexpr = KeyExpr::new("robot/sensor/123").expect("Numbers");
    assert!(format!("{:?}", keyexpr).contains("robot/sensor/123"));
}

/// Test KeyExpr Debug format
#[test]
fn test_keyexpr_debug_format() {
    let keyexpr = KeyExpr::new("test/topic").expect("Failed to create keyexpr");
    let debug = format!("{:?}", keyexpr);
    // Should be in format: KeyExpr("test/topic")
    assert!(debug.starts_with("KeyExpr("));
    assert!(debug.contains("test/topic"));
    assert!(debug.ends_with(")"));
}

/// Test empty key expression
#[test]
fn test_keyexpr_empty() {
    // Empty string - behavior depends on zenoh-pico
    let result = KeyExpr::new("");
    // May succeed or fail depending on zenoh-pico validation
    // Just ensure it doesn't crash
    drop(result);
}

// ============================================================================
// Sample Tests
// ============================================================================

/// Test Sample Debug
#[test]
fn test_sample_debug() {
    let sample = Sample {
        keyexpr: "test/topic".to_string(),
        payload: vec![1, 2, 3, 4, 5],
        attachment: None,
    };
    let debug = format!("{:?}", sample);
    assert!(debug.contains("Sample"));
    assert!(debug.contains("payload"));
}

/// Test Sample with empty payload
#[test]
fn test_sample_empty_payload() {
    let sample = Sample {
        keyexpr: "test/empty".to_string(),
        payload: vec![],
        attachment: None,
    };
    assert_eq!(sample.payload.len(), 0);
    let debug = format!("{:?}", sample);
    assert!(debug.contains("Sample"));
}

/// Test Sample payload access
#[test]
fn test_sample_payload_access() {
    let data = b"Hello, Zenoh!".to_vec();
    let sample = Sample {
        keyexpr: "test/hello".to_string(),
        payload: data.clone(),
        attachment: Some(vec![0x01, 0x02, 0x03]),
    };
    assert_eq!(sample.payload, data);
    assert_eq!(sample.payload.len(), 13);
    assert!(sample.attachment.is_some());
    assert_eq!(sample.attachment.unwrap().len(), 3);
}

// ============================================================================
// Session Tests
// ============================================================================

/// Test session open with peer mode (doesn't require router)
#[test]
fn test_session_peer_mode() {
    let config = Config::peer().expect("Failed to create config");
    // This may fail if networking isn't available, but shouldn't panic
    let result = Session::open(config);
    if let Ok(session) = result {
        session.close().expect("Failed to close session");
    }
}

/// Test full pub/sub cycle with separate sessions.
#[test]
fn test_pubsub() {
    require_router!();

    let received = Arc::new(AtomicBool::new(false));
    let received_count = Arc::new(AtomicUsize::new(0));
    let received_clone = received.clone();
    let count_clone = received_count.clone();

    // Create subscriber session first (with retry for robustness)
    let sub_session = open_session_with_retry().expect("Failed to open sub session");

    let sub_keyexpr = KeyExpr::new("test/zenoh-pico-rust").expect("Failed to create keyexpr");
    let _subscriber = sub_session
        .declare_subscriber(&sub_keyexpr, move |_sample| {
            received_clone.store(true, Ordering::SeqCst);
            count_clone.fetch_add(1, Ordering::SeqCst);
        })
        .expect("Failed to declare subscriber");

    // Give subscriber time to be discovered
    thread::sleep(Duration::from_secs(1));

    // Create publisher session (separate, with retry for robustness)
    let pub_session = open_session_with_retry().expect("Failed to open pub session");

    let pub_keyexpr = KeyExpr::new("test/zenoh-pico-rust").expect("Failed to create keyexpr");
    let publisher = pub_session
        .declare_publisher(&pub_keyexpr)
        .expect("Failed to declare publisher");

    thread::sleep(Duration::from_millis(500));

    // Publish data
    publisher
        .put(b"Hello from Rust!")
        .expect("Failed to publish");
    publisher.put(b"Second message").expect("Failed to publish");

    thread::sleep(Duration::from_secs(2));

    assert!(
        received.load(Ordering::SeqCst),
        "Should have received messages"
    );
    assert!(
        received_count.load(Ordering::SeqCst) >= 1,
        "Should have received at least 1 message"
    );

    pub_session.close().expect("Failed to close pub session");
    sub_session.close().expect("Failed to close sub session");
}

/// Test pub/sub on same session (requires Z_FEATURE_LOCAL_SUBSCRIBER).
#[test]
fn test_pubsub_same_session() {
    require_router!();

    let received = Arc::new(AtomicBool::new(false));
    let received_clone = received.clone();

    let session = open_session_with_retry().expect("Failed to open session");

    let sub_keyexpr = KeyExpr::new("test/same-session").expect("Failed to create keyexpr");
    let pub_keyexpr = KeyExpr::new("test/same-session").expect("Failed to create keyexpr");

    let _subscriber = session
        .declare_subscriber(&sub_keyexpr, move |_sample| {
            received_clone.store(true, Ordering::SeqCst);
        })
        .expect("Failed to declare subscriber");

    thread::sleep(Duration::from_secs(1));

    let publisher = session
        .declare_publisher(&pub_keyexpr)
        .expect("Failed to declare publisher");

    publisher
        .put(b"Hello from same session!")
        .expect("Failed to publish");

    thread::sleep(Duration::from_secs(2));

    assert!(
        received.load(Ordering::SeqCst),
        "Should have received messages"
    );

    session.close().expect("Failed to close session");
}

/// Test session drop without explicit close
#[test]
fn test_session_drop() {
    let config = Config::peer().expect("Failed to create config");
    if let Ok(session) = Session::open(config) {
        // Just drop without close - should not panic
        drop(session);
    }
}

/// Test declaring publisher in peer mode
#[test]
fn test_declare_publisher_peer_mode() {
    let config = Config::peer().expect("Failed to create config");
    if let Ok(session) = Session::open(config) {
        let keyexpr = KeyExpr::new("test/publisher").expect("Failed to create keyexpr");
        let publisher = session.declare_publisher(&keyexpr);
        // May or may not succeed depending on network state
        if let Ok(pub_) = publisher {
            drop(pub_);
        }
        session.close().expect("Failed to close session");
    }
}

/// Test declaring multiple publishers on same session
#[test]
fn test_multiple_publishers_peer_mode() {
    let config = Config::peer().expect("Failed to create config");
    if let Ok(session) = Session::open(config) {
        let keyexpr1 = KeyExpr::new("test/pub1").expect("Failed to create keyexpr");
        let keyexpr2 = KeyExpr::new("test/pub2").expect("Failed to create keyexpr");

        let pub1 = session.declare_publisher(&keyexpr1);
        let pub2 = session.declare_publisher(&keyexpr2);

        // Drop in reverse order
        drop(pub2);
        drop(pub1);

        session.close().expect("Failed to close session");
    }
}

/// Test declaring subscriber in peer mode
#[test]
fn test_declare_subscriber_peer_mode() {
    let config = Config::peer().expect("Failed to create config");
    if let Ok(session) = Session::open(config) {
        let keyexpr = KeyExpr::new("test/subscriber").expect("Failed to create keyexpr");
        let callback_called = Arc::new(AtomicBool::new(false));
        let callback_clone = callback_called.clone();

        let subscriber = session.declare_subscriber(&keyexpr, move |_sample| {
            callback_clone.store(true, Ordering::SeqCst);
        });

        // May or may not succeed
        if let Ok(sub) = subscriber {
            drop(sub);
        }

        session.close().expect("Failed to close session");
    }
}

/// Test subscriber callback using raw FFI
/// Validates that zenoh-pico callbacks work at the FFI level
#[test]
fn test_raw_subscriber_callback() {
    use core::ptr;
    use std::sync::atomic::AtomicU32;
    use zenoh_pico::ffi::*;

    static CALLBACK_COUNT: AtomicU32 = AtomicU32::new(0);

    struct TestContext {
        counter: AtomicU32,
    }

    unsafe extern "C" fn raw_callback(_sample: *mut _z_sample_t, ctx: *mut cty::c_void) {
        CALLBACK_COUNT.fetch_add(1, Ordering::SeqCst);
        if !ctx.is_null() {
            let context = &*(ctx as *const TestContext);
            context.counter.fetch_add(1, Ordering::SeqCst);
        }
    }

    unsafe extern "C" fn raw_drop(ctx: *mut cty::c_void) {
        if !ctx.is_null() {
            let _ = Box::from_raw(ctx as *mut TestContext);
        }
    }

    unsafe {
        CALLBACK_COUNT.store(0, Ordering::SeqCst);

        // Create config
        let mut config = core::mem::MaybeUninit::<z_owned_config_t>::uninit();
        z_config_default(config.as_mut_ptr());
        let mut config = config.assume_init();

        let config_mut = z_config_loan_mut(&mut config);
        zp_config_insert(
            config_mut,
            Z_CONFIG_MODE_KEY as u8,
            Z_CONFIG_MODE_CLIENT.as_ptr() as *const i8,
        );
        zp_config_insert(
            config_mut,
            Z_CONFIG_CONNECT_KEY as u8,
            b"tcp/127.0.0.1:7447\0".as_ptr() as *const i8,
        );

        // Open session
        let mut session = core::mem::MaybeUninit::<z_owned_session_t>::uninit();
        let res = z_open(
            session.as_mut_ptr(),
            z_config_move(&mut config),
            ptr::null(),
        );
        if res < 0 {
            return; // Router not available
        }
        let mut session = session.assume_init();

        // Start tasks
        let session_mut = z_session_loan_mut(&mut session);
        zp_start_read_task(session_mut, ptr::null());
        zp_start_lease_task(session_mut, ptr::null());

        // Create subscriber keyexpr
        let mut sub_ke = core::mem::MaybeUninit::<z_view_keyexpr_t>::uninit();
        z_view_keyexpr_from_str(
            sub_ke.as_mut_ptr(),
            b"test/raw-callback\0".as_ptr() as *const i8,
        );
        let sub_ke = sub_ke.assume_init();

        // Create context
        let context = Box::new(TestContext {
            counter: AtomicU32::new(0),
        });
        let context_ptr = Box::into_raw(context);

        // Create closure
        let mut closure = core::mem::MaybeUninit::<z_owned_closure_sample_t>::uninit();
        z_closure_sample(
            closure.as_mut_ptr(),
            Some(raw_callback),
            Some(raw_drop),
            context_ptr as *mut cty::c_void,
        );
        let mut closure = closure.assume_init();

        // Declare subscriber
        let mut subscriber = core::mem::MaybeUninit::<z_owned_subscriber_t>::uninit();
        let res = z_declare_subscriber(
            z_session_loan(&session),
            subscriber.as_mut_ptr(),
            z_view_keyexpr_loan(&sub_ke),
            z_closure_sample_move(&mut closure),
            ptr::null(),
        );
        if res < 0 {
            z_close(z_session_loan_mut(&mut session), ptr::null());
            return;
        }
        let mut subscriber = subscriber.assume_init();

        thread::sleep(Duration::from_secs(1));

        // Create publisher keyexpr
        let mut pub_ke = core::mem::MaybeUninit::<z_view_keyexpr_t>::uninit();
        z_view_keyexpr_from_str(
            pub_ke.as_mut_ptr(),
            b"test/raw-callback\0".as_ptr() as *const i8,
        );
        let pub_ke = pub_ke.assume_init();

        // Declare publisher
        let mut publisher = core::mem::MaybeUninit::<z_owned_publisher_t>::uninit();
        let res = z_declare_publisher(
            z_session_loan(&session),
            publisher.as_mut_ptr(),
            z_view_keyexpr_loan(&pub_ke),
            ptr::null(),
        );
        if res < 0 {
            z_undeclare_subscriber(z_subscriber_move(&mut subscriber));
            z_close(z_session_loan_mut(&mut session), ptr::null());
            return;
        }
        let mut publisher = publisher.assume_init();

        // Publish data
        let data = b"Hello from raw test!";
        let mut bytes = core::mem::MaybeUninit::<z_owned_bytes_t>::uninit();
        z_bytes_copy_from_buf(bytes.as_mut_ptr(), data.as_ptr(), data.len());
        let mut bytes = bytes.assume_init();

        z_publisher_put(
            z_publisher_loan(&publisher),
            z_bytes_move(&mut bytes),
            ptr::null(),
        );

        thread::sleep(Duration::from_secs(2));

        let count = CALLBACK_COUNT.load(Ordering::SeqCst);
        assert!(count > 0, "Raw callback should have been invoked");

        // Cleanup
        z_undeclare_publisher(z_publisher_move(&mut publisher));
        z_undeclare_subscriber(z_subscriber_move(&mut subscriber));
        zp_stop_read_task(z_session_loan_mut(&mut session));
        zp_stop_lease_task(z_session_loan_mut(&mut session));
        z_close(z_session_loan_mut(&mut session), ptr::null());
    }
}

// ============================================================================
// Publisher Tests (with router)
// Run with: zenohd --listen tcp/127.0.0.1:7447
// ============================================================================

/// Test publishing empty payload.
#[test]
fn test_publish_empty() {
    require_router!();

    let session = open_session_with_retry().expect("Failed to open session");

    let keyexpr = KeyExpr::new("test/empty").expect("Failed to create keyexpr");
    let publisher = session
        .declare_publisher(&keyexpr)
        .expect("Failed to declare publisher");

    // Publish empty payload
    publisher.put(b"").expect("Failed to publish empty payload");

    session.close().expect("Failed to close session");
}

/// Test publishing large payload.
#[test]
fn test_publish_large() {
    require_router!();

    let session = open_session_with_retry().expect("Failed to open session");

    let keyexpr = KeyExpr::new("test/large").expect("Failed to create keyexpr");
    let publisher = session
        .declare_publisher(&keyexpr)
        .expect("Failed to declare publisher");

    // Publish 64KB payload
    let large_data = vec![0x42u8; 64 * 1024];
    publisher
        .put(&large_data)
        .expect("Failed to publish large payload");

    session.close().expect("Failed to close session");
}

/// Test multiple publishes in sequence.
#[test]
fn test_publish_sequence() {
    require_router!();

    let session = open_session_with_retry().expect("Failed to open session");

    let keyexpr = KeyExpr::new("test/sequence").expect("Failed to create keyexpr");
    let publisher = session
        .declare_publisher(&keyexpr)
        .expect("Failed to declare publisher");

    // Publish 100 messages
    for i in 0..100 {
        let msg = format!("Message {}", i);
        publisher.put(msg.as_bytes()).expect("Failed to publish");
    }

    session.close().expect("Failed to close session");
}

// ============================================================================
// Concurrency Tests
// ============================================================================

/// Test creating configs from multiple threads
#[test]
fn test_config_thread_safety() {
    let handles: Vec<_> = (0..4)
        .map(|_| {
            thread::spawn(|| {
                for _ in 0..10 {
                    let config = Config::new().expect("Failed to create config");
                    drop(config);
                }
            })
        })
        .collect();

    for handle in handles {
        handle.join().expect("Thread panicked");
    }
}

/// Test creating keyexprs from multiple threads
#[test]
fn test_keyexpr_thread_safety() {
    let handles: Vec<_> = (0..4)
        .map(|i| {
            thread::spawn(move || {
                for j in 0..10 {
                    let key = format!("test/thread{}/key{}", i, j);
                    let keyexpr = KeyExpr::new(&key).expect("Failed to create keyexpr");
                    drop(keyexpr);
                }
            })
        })
        .collect();

    for handle in handles {
        handle.join().expect("Thread panicked");
    }
}

// ============================================================================
// Result Type Tests
// ============================================================================

/// Test Result type with Error
#[test]
fn test_result_type() {
    fn failing_op() -> zenoh_pico::Result<()> {
        Err(Error::InvalidKeyExpr)
    }

    let result = failing_op();
    assert!(result.is_err());
    assert_eq!(result.unwrap_err(), Error::InvalidKeyExpr);
}

/// Test Result type with Ok
#[test]
fn test_result_ok() {
    fn successful_op() -> zenoh_pico::Result<i32> {
        Ok(42)
    }

    let result = successful_op();
    assert!(result.is_ok());
    assert_eq!(result.unwrap(), 42);
}

/// Test with generic callback at FFI level
/// Validates that monomorphized generic callbacks work correctly
#[test]
fn test_generic_subscriber_callback() {
    use core::ptr;
    use std::sync::atomic::AtomicU32;
    use zenoh_pico::ffi::*;

    static CALLBACK_COUNT: AtomicU32 = AtomicU32::new(0);

    struct CallbackContext<F> {
        callback: F,
    }

    unsafe extern "C" fn generic_callback<F>(sample: *mut _z_sample_t, ctx: *mut cty::c_void)
    where
        F: FnMut(&[u8]) + Send + 'static,
    {
        CALLBACK_COUNT.fetch_add(1, Ordering::SeqCst);
        if sample.is_null() || ctx.is_null() {
            return;
        }

        let context = &mut *(ctx as *mut CallbackContext<F>);
        let sample_ref = &*sample;
        let payload = &sample_ref.payload;
        let len = z_bytes_len(payload as *const _z_bytes_t as *const z_loaned_bytes_t);

        let mut data = vec![0u8; len];
        let mut reader =
            z_bytes_get_reader(payload as *const _z_bytes_t as *const z_loaned_bytes_t);
        z_bytes_reader_read(&mut reader, data.as_mut_ptr(), len);

        (context.callback)(&data);
    }

    unsafe extern "C" fn generic_drop<F>(ctx: *mut cty::c_void) {
        if !ctx.is_null() {
            let _ = Box::from_raw(ctx as *mut CallbackContext<F>);
        }
    }

    let user_received = Arc::new(AtomicBool::new(false));
    let user_received_clone = user_received.clone();
    let user_callback = move |_data: &[u8]| {
        user_received_clone.store(true, Ordering::SeqCst);
    };

    unsafe {
        CALLBACK_COUNT.store(0, Ordering::SeqCst);

        let mut config = core::mem::MaybeUninit::<z_owned_config_t>::uninit();
        z_config_default(config.as_mut_ptr());
        let mut config = config.assume_init();

        let config_mut = z_config_loan_mut(&mut config);
        zp_config_insert(
            config_mut,
            Z_CONFIG_MODE_KEY as u8,
            Z_CONFIG_MODE_CLIENT.as_ptr() as *const i8,
        );
        zp_config_insert(
            config_mut,
            Z_CONFIG_CONNECT_KEY as u8,
            b"tcp/127.0.0.1:7447\0".as_ptr() as *const i8,
        );

        let mut session = core::mem::MaybeUninit::<z_owned_session_t>::uninit();
        let res = z_open(
            session.as_mut_ptr(),
            z_config_move(&mut config),
            ptr::null(),
        );
        if res < 0 {
            return; // Router not available
        }
        let mut session = session.assume_init();

        let session_mut = z_session_loan_mut(&mut session);
        zp_start_read_task(session_mut, ptr::null());
        zp_start_lease_task(session_mut, ptr::null());

        let mut sub_ke = core::mem::MaybeUninit::<z_view_keyexpr_t>::uninit();
        z_view_keyexpr_from_str(
            sub_ke.as_mut_ptr(),
            b"test/generic-callback\0".as_ptr() as *const i8,
        );
        let sub_ke = sub_ke.assume_init();

        type UserCallback = Box<dyn FnMut(&[u8]) + Send + 'static>;
        let boxed_callback: UserCallback = Box::new(user_callback);
        let context = Box::new(CallbackContext {
            callback: boxed_callback,
        });
        let context_ptr = Box::into_raw(context);

        let mut closure = core::mem::MaybeUninit::<z_owned_closure_sample_t>::uninit();
        z_closure_sample(
            closure.as_mut_ptr(),
            Some(generic_callback::<UserCallback>),
            Some(generic_drop::<UserCallback>),
            context_ptr as *mut cty::c_void,
        );
        let mut closure = closure.assume_init();

        let mut subscriber = core::mem::MaybeUninit::<z_owned_subscriber_t>::uninit();
        let res = z_declare_subscriber(
            z_session_loan(&session),
            subscriber.as_mut_ptr(),
            z_view_keyexpr_loan(&sub_ke),
            z_closure_sample_move(&mut closure),
            ptr::null(),
        );
        if res < 0 {
            z_close(z_session_loan_mut(&mut session), ptr::null());
            return;
        }
        let mut subscriber = subscriber.assume_init();

        thread::sleep(Duration::from_secs(1));

        let mut pub_ke = core::mem::MaybeUninit::<z_view_keyexpr_t>::uninit();
        z_view_keyexpr_from_str(
            pub_ke.as_mut_ptr(),
            b"test/generic-callback\0".as_ptr() as *const i8,
        );
        let pub_ke = pub_ke.assume_init();

        let mut publisher = core::mem::MaybeUninit::<z_owned_publisher_t>::uninit();
        let res = z_declare_publisher(
            z_session_loan(&session),
            publisher.as_mut_ptr(),
            z_view_keyexpr_loan(&pub_ke),
            ptr::null(),
        );
        if res < 0 {
            z_undeclare_subscriber(z_subscriber_move(&mut subscriber));
            z_close(z_session_loan_mut(&mut session), ptr::null());
            return;
        }
        let mut publisher = publisher.assume_init();

        let data = b"Hello from generic test!";
        let mut bytes = core::mem::MaybeUninit::<z_owned_bytes_t>::uninit();
        z_bytes_copy_from_buf(bytes.as_mut_ptr(), data.as_ptr(), data.len());
        let mut bytes = bytes.assume_init();

        z_publisher_put(
            z_publisher_loan(&publisher),
            z_bytes_move(&mut bytes),
            ptr::null(),
        );

        thread::sleep(Duration::from_secs(2));

        assert!(
            CALLBACK_COUNT.load(Ordering::SeqCst) > 0,
            "Generic callback should have been invoked"
        );
        assert!(
            user_received.load(Ordering::SeqCst),
            "User callback should have been called"
        );

        z_undeclare_publisher(z_publisher_move(&mut publisher));
        z_undeclare_subscriber(z_subscriber_move(&mut subscriber));
        zp_stop_read_task(z_session_loan_mut(&mut session));
        zp_stop_lease_task(z_session_loan_mut(&mut session));
        z_close(z_session_loan_mut(&mut session), ptr::null());
    }
}
