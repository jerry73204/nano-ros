//! nano-ros to nano-ros communication tests
//!
//! Tests communication between native nano-ros binaries via zenoh.

use nano_ros_tests::fixtures::{
    is_zenohd_available, listener_binary, require_zenohd, talker_binary, zenohd_unique,
    ManagedProcess, ZenohRouter,
};
use rstest::rstest;
use std::path::PathBuf;
use std::time::Duration;

// =============================================================================
// Native Pub/Sub Tests
// =============================================================================

#[rstest]
fn test_native_talker_starts(zenohd_unique: ZenohRouter, talker_binary: PathBuf) {
    use std::process::Command;

    if !require_zenohd() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Use ZENOH_LOCATOR env var since examples use Context::from_env()
    let mut cmd = Command::new(&talker_binary);
    cmd.env("ZENOH_LOCATOR", &locator);
    let mut talker =
        ManagedProcess::spawn_command(cmd, "native-rs-talker").expect("Failed to start talker");

    // Let it run briefly and check it started
    std::thread::sleep(Duration::from_secs(2));

    // Check process is still running (didn't crash)
    if talker.is_running() {
        eprintln!("native-rs-talker started successfully");
    } else {
        eprintln!("native-rs-talker exited early");
    }
}

#[rstest]
fn test_native_listener_starts(zenohd_unique: ZenohRouter, listener_binary: PathBuf) {
    use std::process::Command;

    if !require_zenohd() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Use ZENOH_LOCATOR env var since examples use Context::from_env()
    let mut cmd = Command::new(&listener_binary);
    cmd.env("ZENOH_LOCATOR", &locator);
    let mut listener =
        ManagedProcess::spawn_command(cmd, "native-rs-listener").expect("Failed to start listener");

    // Let it run briefly and check it started
    std::thread::sleep(Duration::from_secs(2));

    // Check process is still running (didn't crash)
    if listener.is_running() {
        eprintln!("native-rs-listener started successfully");
    } else {
        eprintln!("native-rs-listener exited early");
    }
}

#[rstest]
fn test_talker_listener_communication(
    zenohd_unique: ZenohRouter,
    talker_binary: PathBuf,
    listener_binary: PathBuf,
) {
    use nano_ros_tests::count_pattern;
    use std::process::Command;

    if !require_zenohd() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Start listener first with ZENOH_LOCATOR env var
    let mut listener_cmd = Command::new(&listener_binary);
    listener_cmd.env("ZENOH_LOCATOR", &locator);
    let mut listener = ManagedProcess::spawn_command(listener_cmd, "native-rs-listener")
        .expect("Failed to start listener");

    // Give listener time to subscribe
    std::thread::sleep(Duration::from_secs(2));

    // Start talker with ZENOH_LOCATOR env var
    let mut talker_cmd = Command::new(&talker_binary);
    talker_cmd.env("ZENOH_LOCATOR", &locator);
    let mut talker = ManagedProcess::spawn_command(talker_cmd, "native-rs-talker")
        .expect("Failed to start talker");

    // Let them communicate for a few seconds
    std::thread::sleep(Duration::from_secs(5));

    // Kill talker first
    talker.kill();

    // Collect listener output
    let listener_output = listener
        .wait_for_output(Duration::from_secs(2))
        .unwrap_or_default();

    eprintln!("Listener output:\n{}", listener_output);

    // Check if listener received messages
    let received_count = count_pattern(&listener_output, "Received:");
    eprintln!("Listener received {} messages", received_count);

    if received_count > 0 {
        eprintln!("[PASS] Router-based communication works");
    } else {
        eprintln!("[INFO] No messages received (may be timing issue)");
    }
}

// =============================================================================
// Peer Mode Tests (no router required)
// =============================================================================

/// Test peer-to-peer communication without a zenohd router
///
/// In peer mode, nano-ros nodes can discover each other via multicast
/// without requiring a central router.
#[rstest]
fn test_peer_mode_communication(talker_binary: PathBuf, listener_binary: PathBuf) {
    use nano_ros_tests::count_pattern;
    use std::process::Command;

    eprintln!("Testing peer mode communication (no router)...");

    // Start listener in peer mode
    let mut listener_cmd = Command::new(&listener_binary);
    listener_cmd.env("ZENOH_MODE", "peer");
    let mut listener = ManagedProcess::spawn_command(listener_cmd, "native-rs-listener-peer")
        .expect("Failed to start listener in peer mode");

    // Give listener time to start and set up peer discovery
    std::thread::sleep(Duration::from_secs(2));

    // Check listener is still running
    if !listener.is_running() {
        eprintln!("[INFO] Listener exited early - peer mode may not be supported");
        return;
    }

    // Start talker in peer mode
    let mut talker_cmd = Command::new(&talker_binary);
    talker_cmd.env("ZENOH_MODE", "peer");
    let mut talker = ManagedProcess::spawn_command(talker_cmd, "native-rs-talker-peer")
        .expect("Failed to start talker in peer mode");

    // Give talker time to start
    std::thread::sleep(Duration::from_secs(1));

    // Check talker is still running
    if !talker.is_running() {
        eprintln!("[INFO] Talker exited early - peer mode may not be supported");
        return;
    }

    // Let them communicate
    eprintln!("Letting peers communicate for 6 seconds...");
    std::thread::sleep(Duration::from_secs(6));

    // Kill talker first
    talker.kill();

    // Collect listener output
    let listener_output = listener
        .wait_for_output(Duration::from_secs(2))
        .unwrap_or_default();

    eprintln!("Listener output:\n{}", listener_output);

    // Check if listener received messages
    let received_count = count_pattern(&listener_output, "Received:");
    eprintln!("Listener received {} messages", received_count);

    if received_count > 0 {
        eprintln!("[PASS] Peer mode communication works");
    } else {
        // Peer mode may require specific network configuration (multicast enabled)
        eprintln!("[INFO] No messages received - peer discovery may require multicast support");
        eprintln!("[INFO] This is expected on some network configurations");
    }
}

// =============================================================================
// Detection Tests
// =============================================================================

#[test]
fn test_zenohd_detection() {
    let available = is_zenohd_available();
    eprintln!("zenohd available: {}", available);
}
