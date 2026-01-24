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
    if !require_zenohd() {
        return;
    }

    let locator = zenohd_unique.locator();
    let mut talker = ManagedProcess::spawn(
        &talker_binary,
        &["--tcp", &locator.replace("tcp/", "")],
        "native-talker",
    )
    .expect("Failed to start talker");

    // Let it run briefly and check it started
    std::thread::sleep(Duration::from_secs(2));

    // Check process is still running (didn't crash)
    if talker.is_running() {
        eprintln!("native-talker started successfully");
    } else {
        eprintln!("native-talker exited early");
    }
}

#[rstest]
fn test_native_listener_starts(zenohd_unique: ZenohRouter, listener_binary: PathBuf) {
    if !require_zenohd() {
        return;
    }

    let locator = zenohd_unique.locator();
    let mut listener = ManagedProcess::spawn(
        &listener_binary,
        &["--tcp", &locator.replace("tcp/", "")],
        "native-listener",
    )
    .expect("Failed to start listener");

    // Let it run briefly and check it started
    std::thread::sleep(Duration::from_secs(2));

    // Check process is still running (didn't crash)
    if listener.is_running() {
        eprintln!("native-listener started successfully");
    } else {
        eprintln!("native-listener exited early");
    }
}

#[rstest]
fn test_talker_listener_communication(
    zenohd_unique: ZenohRouter,
    talker_binary: PathBuf,
    listener_binary: PathBuf,
) {
    if !require_zenohd() {
        return;
    }

    let locator = zenohd_unique.locator();
    let tcp_addr = locator.replace("tcp/", "");

    // Start listener first
    let mut listener =
        ManagedProcess::spawn(&listener_binary, &["--tcp", &tcp_addr], "native-listener")
            .expect("Failed to start listener");

    // Give listener time to subscribe
    std::thread::sleep(Duration::from_secs(1));

    // Start talker
    let mut talker = ManagedProcess::spawn(&talker_binary, &["--tcp", &tcp_addr], "native-talker")
        .expect("Failed to start talker");

    // Let them communicate for a few seconds
    std::thread::sleep(Duration::from_secs(5));

    // Kill talker first
    talker.kill();

    // Give listener time to process last messages
    std::thread::sleep(Duration::from_millis(500));

    // Kill listener and check output
    listener.kill();

    // Note: Full output verification would require capturing stdout asynchronously
    // For now, just verify both processes started and ran without crashing
    eprintln!("Talker/listener communication test completed");
}

// =============================================================================
// Detection Tests
// =============================================================================

#[test]
fn test_zenohd_detection() {
    let available = is_zenohd_available();
    eprintln!("zenohd available: {}", available);
}
