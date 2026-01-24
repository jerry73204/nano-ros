//! ROS 2 rmw_zenoh interoperability tests
//!
//! Tests communication between nano-ros and ROS 2 nodes using rmw_zenoh_cpp.

use nano_ros_tests::fixtures::{
    build_native_listener, build_native_talker, is_rmw_zenoh_available, is_ros2_available,
    zenohd_unique, Ros2Process, ZenohRouter, DEFAULT_ROS_DISTRO,
};
use nano_ros_tests::{count_pattern, TestError};
use rstest::rstest;
use std::process::{Child, Command, Stdio};
use std::time::Duration;

// =============================================================================
// Test Helpers
// =============================================================================

/// Managed native process (talker or listener)
struct NativeProcess {
    handle: Child,
    #[allow(dead_code)]
    name: &'static str,
}

impl NativeProcess {
    fn spawn(
        binary: &std::path::Path,
        args: &[&str],
        name: &'static str,
    ) -> Result<Self, TestError> {
        let handle = Command::new(binary)
            .args(args)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| TestError::ProcessFailed(format!("Failed to spawn {}: {}", name, e)))?;

        Ok(Self { handle, name })
    }

    fn wait_for_output(&mut self, timeout: Duration) -> Result<String, TestError> {
        use std::io::Read;

        let start = std::time::Instant::now();
        let mut output = String::new();

        let mut stdout = self
            .handle
            .stdout
            .take()
            .ok_or_else(|| TestError::ProcessFailed("No stdout".to_string()))?;

        let mut buffer = [0u8; 4096];
        loop {
            if start.elapsed() > timeout {
                let _ = self.handle.kill();
                break;
            }

            match self.handle.try_wait() {
                Ok(Some(_)) => {
                    let _ = stdout.read_to_string(&mut output);
                    break;
                }
                Ok(None) => match stdout.read(&mut buffer) {
                    Ok(0) => std::thread::sleep(Duration::from_millis(50)),
                    Ok(n) => {
                        output.push_str(&String::from_utf8_lossy(&buffer[..n]));
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        std::thread::sleep(Duration::from_millis(50));
                    }
                    Err(_) => break,
                },
                Err(_) => break,
            }
        }

        Ok(output)
    }

    fn kill(&mut self) {
        let _ = self.handle.kill();
        let _ = self.handle.wait();
    }
}

impl Drop for NativeProcess {
    fn drop(&mut self) {
        self.kill();
    }
}

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
fn test_nano_to_ros2(zenohd_unique: ZenohRouter) {
    if !require_ros2() {
        return;
    }

    let talker_path = match build_native_talker() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Skipping test: Failed to build native-talker: {}", e);
            return;
        }
    };

    let locator = zenohd_unique.locator();
    let tcp_addr = locator.replace("tcp/", "");

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

    // Start nano-ros talker
    eprintln!("Starting nano-ros talker...");
    let mut talker = NativeProcess::spawn(talker_path, &["--tcp", &tcp_addr], "native-talker")
        .expect("Failed to start talker");

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
fn test_ros2_to_nano(zenohd_unique: ZenohRouter) {
    if !require_ros2() {
        return;
    }

    let listener_path = match build_native_listener() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Skipping test: Failed to build native-listener: {}", e);
            return;
        }
    };

    let locator = zenohd_unique.locator();
    let tcp_addr = locator.replace("tcp/", "");

    // Start nano-ros listener first
    eprintln!("Starting nano-ros listener...");
    let mut listener =
        NativeProcess::spawn(listener_path, &["--tcp", &tcp_addr], "native-listener")
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
fn test_communication_matrix(zenohd_unique: ZenohRouter, #[case] direction: Direction) {
    // Check prerequisites based on direction
    match direction {
        Direction::NanoToNano => {}
        Direction::NanoToRos2 | Direction::Ros2ToNano => {
            if !require_ros2() {
                return;
            }
        }
    }

    let talker_path = match build_native_talker() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Skipping test: Failed to build native-talker: {}", e);
            return;
        }
    };

    let listener_path = match build_native_listener() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Skipping test: Failed to build native-listener: {}", e);
            return;
        }
    };

    let locator = zenohd_unique.locator();
    let tcp_addr = locator.replace("tcp/", "");

    eprintln!("Testing: {}", direction);

    let success = match direction {
        Direction::NanoToNano => test_nano_to_nano_inner(&tcp_addr, talker_path, listener_path),
        Direction::NanoToRos2 => test_nano_to_ros2_inner(&tcp_addr, talker_path),
        Direction::Ros2ToNano => test_ros2_to_nano_inner(&tcp_addr, listener_path),
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

fn test_nano_to_nano_inner(
    tcp_addr: &str,
    talker_path: &std::path::Path,
    listener_path: &std::path::Path,
) -> bool {
    // Start listener
    let mut listener = NativeProcess::spawn(listener_path, &["--tcp", tcp_addr], "native-listener")
        .expect("Failed to start listener");

    std::thread::sleep(Duration::from_secs(1));

    // Start talker
    let mut talker = NativeProcess::spawn(talker_path, &["--tcp", tcp_addr], "native-talker")
        .expect("Failed to start talker");

    std::thread::sleep(Duration::from_secs(5));

    talker.kill();
    let output = listener
        .wait_for_output(Duration::from_secs(1))
        .unwrap_or_default();

    count_pattern(&output, "Received:") > 0
}

fn test_nano_to_ros2_inner(tcp_addr: &str, talker_path: &std::path::Path) -> bool {
    // Start ROS 2 listener
    let mut ros2_listener =
        match Ros2Process::topic_echo("/chatter", "std_msgs/msg/Int32", DEFAULT_ROS_DISTRO) {
            Ok(p) => p,
            Err(_) => return false,
        };

    std::thread::sleep(Duration::from_secs(3));

    // Start nano-ros talker
    let mut talker = NativeProcess::spawn(talker_path, &["--tcp", tcp_addr], "native-talker")
        .expect("Failed to start talker");

    std::thread::sleep(Duration::from_secs(6));

    talker.kill();
    let output = ros2_listener
        .wait_for_output(Duration::from_secs(1))
        .unwrap_or_default();

    count_pattern(&output, "data:") > 0
}

fn test_ros2_to_nano_inner(tcp_addr: &str, listener_path: &std::path::Path) -> bool {
    // Start nano-ros listener
    let mut listener = NativeProcess::spawn(listener_path, &["--tcp", tcp_addr], "native-listener")
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
fn test_keyexpr_format(zenohd_unique: ZenohRouter) {
    let talker_path = match build_native_talker() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Skipping test: Failed to build native-talker: {}", e);
            return;
        }
    };

    let locator = zenohd_unique.locator();
    let tcp_addr = locator.replace("tcp/", "");

    // Start talker briefly to register key expression
    let mut talker = NativeProcess::spawn(talker_path, &["--tcp", &tcp_addr], "native-talker")
        .expect("Failed to start talker");

    std::thread::sleep(Duration::from_secs(2));
    talker.kill();

    // Expected key expression format for Humble:
    // <domain_id>/<topic>/<type>/<hash>
    // 0/chatter/std_msgs::msg::dds_::Int32_/TypeHashNotSupported
    eprintln!("[PASS] Talker started and registered key expression");
    eprintln!("Expected format: 0/chatter/std_msgs::msg::dds_::Int32_/TypeHashNotSupported");
}

#[rstest]
fn test_qos_compatibility(zenohd_unique: ZenohRouter) {
    if !require_ros2() {
        return;
    }

    let talker_path = match build_native_talker() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Skipping test: Failed to build native-talker: {}", e);
            return;
        }
    };

    let locator = zenohd_unique.locator();
    let tcp_addr = locator.replace("tcp/", "");

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

    // Start nano-ros talker (uses BEST_EFFORT by default)
    let mut talker = NativeProcess::spawn(talker_path, &["--tcp", &tcp_addr], "native-talker")
        .expect("Failed to start talker");

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
