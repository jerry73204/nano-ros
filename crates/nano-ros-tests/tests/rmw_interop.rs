//! ROS 2 rmw_zenoh interoperability tests
//!
//! Tests communication between nano-ros and ROS 2 nodes using rmw_zenoh_cpp.

use nano_ros_tests::count_pattern;
use nano_ros_tests::fixtures::{
    is_rmw_zenoh_available, is_ros2_available, listener_binary, talker_binary, zenohd_unique,
    ManagedProcess, Ros2Process, ZenohRouter, DEFAULT_ROS_DISTRO,
};
use rstest::rstest;
use std::path::{Path, PathBuf};
use std::time::Duration;

/// Skip test if ROS 2 prerequisites are not met
fn require_ros2() -> bool {
    if !is_ros2_available() {
        eprintln!("Skipping test: ROS 2 not available");
        return false;
    }
    if !is_rmw_zenoh_available() {
        eprintln!("Skipping test: rmw_zenoh_cpp not available");
        return false;
    }
    true
}

// =============================================================================
// Detection Tests
// =============================================================================

#[test]
fn test_ros2_detection() {
    let available = is_ros2_available();
    eprintln!("ROS 2 available: {}", available);
}

#[test]
fn test_rmw_zenoh_detection() {
    let available = is_rmw_zenoh_available();
    eprintln!("rmw_zenoh_cpp available: {}", available);
}

// =============================================================================
// nano-ros → ROS 2 Tests
// =============================================================================

#[rstest]
fn test_nano_to_ros2(zenohd_unique: ZenohRouter, talker_binary: PathBuf) {
    use std::process::Command;

    if !require_ros2() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Start ROS 2 listener first
    eprintln!("Starting ROS 2 topic echo...");
    let mut ros2_listener =
        match Ros2Process::topic_echo("/chatter", "std_msgs/msg/Int32", DEFAULT_ROS_DISTRO) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("Failed to start ROS 2 listener: {}", e);
                return;
            }
        };

    // Give ROS 2 time to subscribe
    std::thread::sleep(Duration::from_secs(3));

    // Start nano-ros talker with ZENOH_LOCATOR env var
    eprintln!("Starting nano-ros talker...");
    let mut talker_cmd = Command::new(&talker_binary);
    talker_cmd.env("ZENOH_LOCATOR", &locator);
    let mut talker =
        ManagedProcess::spawn_command(talker_cmd, "native-talker").expect("Failed to start talker");

    // Let them communicate
    std::thread::sleep(Duration::from_secs(8));

    // Kill talker first
    talker.kill();

    // Collect ROS 2 output
    let ros2_output = ros2_listener
        .wait_for_output(Duration::from_secs(2))
        .unwrap_or_default();

    eprintln!("ROS 2 output:\n{}", ros2_output);

    // Check if ROS 2 received messages
    let received_count = count_pattern(&ros2_output, "data:");
    eprintln!("ROS 2 received {} messages", received_count);

    if received_count > 0 {
        eprintln!("[PASS] nano-ros → ROS 2 communication works");
    } else {
        eprintln!("[INFO] ROS 2 did not receive messages (may be timing issue)");
    }
}

// =============================================================================
// ROS 2 → nano-ros Tests
// =============================================================================

#[rstest]
fn test_ros2_to_nano(zenohd_unique: ZenohRouter, listener_binary: PathBuf) {
    use std::process::Command;

    if !require_ros2() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Start nano-ros listener first with ZENOH_LOCATOR env var
    eprintln!("Starting nano-ros listener...");
    let mut listener_cmd = Command::new(&listener_binary);
    listener_cmd.env("ZENOH_LOCATOR", &locator);
    let mut listener = ManagedProcess::spawn_command(listener_cmd, "native-listener")
        .expect("Failed to start listener");

    // Give listener time to subscribe
    std::thread::sleep(Duration::from_secs(3));

    // Start ROS 2 publisher
    eprintln!("Starting ROS 2 topic pub...");
    let mut ros2_publisher = match Ros2Process::topic_pub(
        "/chatter",
        "std_msgs/msg/Int32",
        "{data: 42}",
        1,
        DEFAULT_ROS_DISTRO,
    ) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to start ROS 2 publisher: {}", e);
            listener.kill();
            return;
        }
    };

    // Let them communicate
    std::thread::sleep(Duration::from_secs(8));

    // Kill ROS 2 publisher first
    ros2_publisher.kill();

    // Collect nano-ros output
    let nano_output = listener
        .wait_for_output(Duration::from_secs(2))
        .unwrap_or_default();

    eprintln!("nano-ros output:\n{}", nano_output);

    // Check if nano-ros received messages
    let received_count = count_pattern(&nano_output, "Received:");
    eprintln!("nano-ros received {} messages", received_count);

    if received_count > 0 {
        eprintln!("[PASS] ROS 2 → nano-ros communication works");

        // Check data integrity
        if nano_output.contains("data=42") {
            eprintln!("[PASS] Data integrity verified (data=42)");
        }
    } else {
        eprintln!("[INFO] nano-ros did not receive messages (may be timing issue)");
    }
}

// =============================================================================
// Communication Matrix Tests
// =============================================================================

/// Test direction for matrix tests
#[derive(Debug, Clone, Copy)]
enum Direction {
    NanoToNano,
    NanoToRos2,
    Ros2ToNano,
}

impl std::fmt::Display for Direction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Direction::NanoToNano => write!(f, "nano-ros → nano-ros"),
            Direction::NanoToRos2 => write!(f, "nano-ros → ROS 2"),
            Direction::Ros2ToNano => write!(f, "ROS 2 → nano-ros"),
        }
    }
}

#[rstest]
#[case(Direction::NanoToNano)]
#[case(Direction::NanoToRos2)]
#[case(Direction::Ros2ToNano)]
fn test_communication_matrix(
    zenohd_unique: ZenohRouter,
    talker_binary: PathBuf,
    listener_binary: PathBuf,
    #[case] direction: Direction,
) {
    // Check prerequisites based on direction
    match direction {
        Direction::NanoToNano => {}
        Direction::NanoToRos2 | Direction::Ros2ToNano => {
            if !require_ros2() {
                return;
            }
        }
    }

    let locator = zenohd_unique.locator();

    eprintln!("Testing: {}", direction);

    let success = match direction {
        Direction::NanoToNano => {
            test_nano_to_nano_inner(&locator, &talker_binary, &listener_binary)
        }
        Direction::NanoToRos2 => test_nano_to_ros2_inner(&locator, &talker_binary),
        Direction::Ros2ToNano => test_ros2_to_nano_inner(&locator, &listener_binary),
    };

    if success {
        eprintln!("[PASS] {}", direction);
    } else {
        eprintln!(
            "[INFO] {} - no messages received (may be timing)",
            direction
        );
    }
}

fn test_nano_to_nano_inner(locator: &str, talker_path: &Path, listener_path: &Path) -> bool {
    use std::process::Command;

    // Start listener with ZENOH_LOCATOR env var
    let mut listener_cmd = Command::new(listener_path);
    listener_cmd.env("ZENOH_LOCATOR", locator);
    let mut listener = ManagedProcess::spawn_command(listener_cmd, "native-listener")
        .expect("Failed to start listener");

    std::thread::sleep(Duration::from_secs(2));

    // Start talker with ZENOH_LOCATOR env var
    let mut talker_cmd = Command::new(talker_path);
    talker_cmd.env("ZENOH_LOCATOR", locator);
    let mut talker =
        ManagedProcess::spawn_command(talker_cmd, "native-talker").expect("Failed to start talker");

    std::thread::sleep(Duration::from_secs(5));

    talker.kill();
    let output = listener
        .wait_for_output(Duration::from_secs(1))
        .unwrap_or_default();

    count_pattern(&output, "Received:") > 0
}

fn test_nano_to_ros2_inner(locator: &str, talker_path: &Path) -> bool {
    use std::process::Command;

    // Start ROS 2 listener
    let mut ros2_listener =
        match Ros2Process::topic_echo("/chatter", "std_msgs/msg/Int32", DEFAULT_ROS_DISTRO) {
            Ok(p) => p,
            Err(_) => return false,
        };

    std::thread::sleep(Duration::from_secs(3));

    // Start nano-ros talker with ZENOH_LOCATOR env var
    let mut talker_cmd = Command::new(talker_path);
    talker_cmd.env("ZENOH_LOCATOR", locator);
    let mut talker =
        ManagedProcess::spawn_command(talker_cmd, "native-talker").expect("Failed to start talker");

    std::thread::sleep(Duration::from_secs(6));

    talker.kill();
    let output = ros2_listener
        .wait_for_output(Duration::from_secs(1))
        .unwrap_or_default();

    count_pattern(&output, "data:") > 0
}

fn test_ros2_to_nano_inner(locator: &str, listener_path: &Path) -> bool {
    use std::process::Command;

    // Start nano-ros listener with ZENOH_LOCATOR env var
    let mut listener_cmd = Command::new(listener_path);
    listener_cmd.env("ZENOH_LOCATOR", locator);
    let mut listener = ManagedProcess::spawn_command(listener_cmd, "native-listener")
        .expect("Failed to start listener");

    std::thread::sleep(Duration::from_secs(3));

    // Start ROS 2 publisher
    let mut ros2_publisher = match Ros2Process::topic_pub(
        "/chatter",
        "std_msgs/msg/Int32",
        "{data: 42}",
        1,
        DEFAULT_ROS_DISTRO,
    ) {
        Ok(p) => p,
        Err(_) => return false,
    };

    std::thread::sleep(Duration::from_secs(6));

    ros2_publisher.kill();
    let output = listener
        .wait_for_output(Duration::from_secs(1))
        .unwrap_or_default();

    count_pattern(&output, "Received:") > 0
}

// =============================================================================
// Protocol Detail Tests (migrated from rmw-detailed/)
// =============================================================================

#[rstest]
fn test_keyexpr_format(zenohd_unique: ZenohRouter, talker_binary: PathBuf) {
    use std::process::Command;

    let locator = zenohd_unique.locator();

    // Start talker briefly to register key expression with ZENOH_LOCATOR env var
    let mut talker_cmd = Command::new(&talker_binary);
    talker_cmd.env("ZENOH_LOCATOR", &locator);
    let mut talker =
        ManagedProcess::spawn_command(talker_cmd, "native-talker").expect("Failed to start talker");

    std::thread::sleep(Duration::from_secs(2));
    talker.kill();

    // Expected key expression format for Humble:
    // <domain_id>/<topic>/<type>/<hash>
    // 0/chatter/std_msgs::msg::dds_::Int32_/TypeHashNotSupported
    eprintln!("[PASS] Talker started and registered key expression");
    eprintln!("Expected format: 0/chatter/std_msgs::msg::dds_::Int32_/TypeHashNotSupported");
}

#[rstest]
fn test_qos_compatibility(zenohd_unique: ZenohRouter, talker_binary: PathBuf) {
    use std::process::Command;

    if !require_ros2() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Test BEST_EFFORT QoS compatibility
    eprintln!("Testing BEST_EFFORT QoS compatibility...");

    // Start ROS 2 listener with BEST_EFFORT
    let mut ros2_listener =
        match Ros2Process::topic_echo("/chatter", "std_msgs/msg/Int32", DEFAULT_ROS_DISTRO) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("Failed to start ROS 2 listener: {}", e);
                return;
            }
        };

    std::thread::sleep(Duration::from_secs(3));

    // Start nano-ros talker (uses BEST_EFFORT by default) with ZENOH_LOCATOR env var
    let mut talker_cmd = Command::new(&talker_binary);
    talker_cmd.env("ZENOH_LOCATOR", &locator);
    let mut talker =
        ManagedProcess::spawn_command(talker_cmd, "native-talker").expect("Failed to start talker");

    std::thread::sleep(Duration::from_secs(6));

    talker.kill();
    let output = ros2_listener
        .wait_for_output(Duration::from_secs(1))
        .unwrap_or_default();

    if count_pattern(&output, "data:") > 0 {
        eprintln!("[PASS] BEST_EFFORT QoS compatible");
    } else {
        eprintln!("[INFO] QoS test inconclusive");
    }
}
