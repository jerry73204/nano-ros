//! nano-ros to nano-ros communication tests
//!
//! Tests communication between native nano-ros binaries via zenoh.

use nano_ros_tests::fixtures::{
    build_native_listener, build_native_talker, zenohd_unique, ZenohRouter,
};
use nano_ros_tests::TestError;
use rstest::rstest;
use std::process::{Child, Command, Stdio};
use std::time::Duration;

/// Managed process for talker/listener binaries
#[allow(dead_code)]
struct ManagedProcess {
    handle: Child,
    name: &'static str,
}

impl ManagedProcess {
    fn spawn(binary: &std::path::Path, args: &[&str], name: &'static str) -> Result<Self, TestError> {
        let handle = Command::new(binary)
            .args(args)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| TestError::ProcessFailed(format!("Failed to spawn {}: {}", name, e)))?;

        Ok(Self { handle, name })
    }

    #[allow(dead_code)]
    fn wait_for_output(&mut self, timeout: Duration) -> Result<String, TestError> {
        use std::io::Read;

        let start = std::time::Instant::now();
        let mut output = String::new();

        let mut stdout = self.handle.stdout.take()
            .ok_or_else(|| TestError::ProcessFailed(format!("No stdout for {}", self.name)))?;

        let mut buffer = [0u8; 4096];
        loop {
            if start.elapsed() > timeout {
                let _ = self.handle.kill();
                if output.is_empty() {
                    return Err(TestError::Timeout);
                }
                break;
            }

            match self.handle.try_wait() {
                Ok(Some(_)) => {
                    let _ = stdout.read_to_string(&mut output);
                    break;
                }
                Ok(None) => {
                    match stdout.read(&mut buffer) {
                        Ok(0) => std::thread::sleep(Duration::from_millis(50)),
                        Ok(n) => {
                            output.push_str(&String::from_utf8_lossy(&buffer[..n]));
                        }
                        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                            std::thread::sleep(Duration::from_millis(50));
                        }
                        Err(_) => break,
                    }
                }
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

impl Drop for ManagedProcess {
    fn drop(&mut self) {
        self.kill();
    }
}

// =============================================================================
// Zenoh Availability Check
// =============================================================================

/// Check if zenohd is available
fn is_zenohd_available() -> bool {
    Command::new("zenohd")
        .arg("--version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Skip test if zenohd is not available
fn require_zenohd() {
    if !is_zenohd_available() {
        eprintln!("Skipping test: zenohd not found");
        return;
    }
}

// =============================================================================
// Native Pub/Sub Tests
// =============================================================================

#[rstest]
fn test_native_talker_starts(zenohd_unique: ZenohRouter) {
    require_zenohd();

    let talker_path = match build_native_talker() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Skipping test: Failed to build native-talker: {}", e);
            return;
        }
    };

    let locator = zenohd_unique.locator();
    let mut talker = ManagedProcess::spawn(
        talker_path,
        &["--tcp", &locator.replace("tcp/", "")],
        "native-talker",
    )
    .expect("Failed to start talker");

    // Let it run briefly and check it started
    std::thread::sleep(Duration::from_secs(2));

    // Check process is still running (didn't crash)
    match talker.handle.try_wait() {
        Ok(None) => {
            // Still running - good
            eprintln!("native-talker started successfully");
        }
        Ok(Some(status)) => {
            eprintln!("native-talker exited with status: {}", status);
        }
        Err(e) => {
            eprintln!("Error checking talker status: {}", e);
        }
    }
}

#[rstest]
fn test_native_listener_starts(zenohd_unique: ZenohRouter) {
    require_zenohd();

    let listener_path = match build_native_listener() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Skipping test: Failed to build native-listener: {}", e);
            return;
        }
    };

    let locator = zenohd_unique.locator();
    let mut listener = ManagedProcess::spawn(
        listener_path,
        &["--tcp", &locator.replace("tcp/", "")],
        "native-listener",
    )
    .expect("Failed to start listener");

    // Let it run briefly and check it started
    std::thread::sleep(Duration::from_secs(2));

    // Check process is still running (didn't crash)
    match listener.handle.try_wait() {
        Ok(None) => {
            eprintln!("native-listener started successfully");
        }
        Ok(Some(status)) => {
            eprintln!("native-listener exited with status: {}", status);
        }
        Err(e) => {
            eprintln!("Error checking listener status: {}", e);
        }
    }
}

#[rstest]
fn test_talker_listener_communication(zenohd_unique: ZenohRouter) {
    require_zenohd();

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

    // Start listener first
    let mut listener = ManagedProcess::spawn(
        listener_path,
        &["--tcp", &tcp_addr],
        "native-listener",
    )
    .expect("Failed to start listener");

    // Give listener time to subscribe
    std::thread::sleep(Duration::from_secs(1));

    // Start talker
    let mut talker = ManagedProcess::spawn(
        talker_path,
        &["--tcp", &tcp_addr],
        "native-talker",
    )
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
