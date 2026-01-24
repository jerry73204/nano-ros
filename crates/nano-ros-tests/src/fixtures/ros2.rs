//! ROS 2 process fixtures for integration tests
//!
//! Provides helpers for running ROS 2 commands and processes.

use crate::{TestError, TestResult};
use std::process::{Child, Command, Stdio};
use std::time::Duration;

/// Default ROS 2 distro to use
pub const DEFAULT_ROS_DISTRO: &str = "humble";

/// Check if ROS 2 is available
pub fn is_ros2_available() -> bool {
    // Check if ros2 command exists by trying to get help
    Command::new("bash")
        .args(["-c", "source /opt/ros/humble/setup.bash && ros2 --help"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Check if rmw_zenoh_cpp is available
pub fn is_rmw_zenoh_available() -> bool {
    Command::new("bash")
        .args([
            "-c",
            "source /opt/ros/humble/setup.bash && ros2 pkg list | grep -q rmw_zenoh_cpp",
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Get ROS 2 environment setup command
pub fn ros2_env_setup(distro: &str) -> String {
    format!(
        "source /opt/ros/{distro}/setup.bash && \
         export RMW_IMPLEMENTATION=rmw_zenoh_cpp && \
         export ZENOH_CONFIG_OVERRIDE='mode=\"client\";connect/endpoints=[\"tcp/127.0.0.1:7447\"]'"
    )
}

/// Managed ROS 2 process
///
/// Wraps a ROS 2 command with proper environment setup.
/// Automatically kills the process on drop.
pub struct Ros2Process {
    handle: Child,
    name: String,
}

impl Ros2Process {
    /// Start a ROS 2 topic echo subscriber
    ///
    /// # Arguments
    /// * `topic` - Topic name (e.g., "/chatter")
    /// * `msg_type` - Message type (e.g., "std_msgs/msg/Int32")
    /// * `distro` - ROS distro (e.g., "humble")
    pub fn topic_echo(topic: &str, msg_type: &str, distro: &str) -> TestResult<Self> {
        let env_setup = ros2_env_setup(distro);
        let cmd = format!(
            "{env_setup} && timeout 30 ros2 topic echo {topic} {msg_type} --qos-reliability best_effort"
        );

        let handle = Command::new("bash")
            .args(["-c", &cmd])
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| {
                TestError::ProcessFailed(format!("Failed to start ros2 topic echo: {e}"))
            })?;

        Ok(Self {
            handle,
            name: format!("ros2 topic echo {topic}"),
        })
    }

    /// Start a ROS 2 topic pub publisher
    ///
    /// # Arguments
    /// * `topic` - Topic name (e.g., "/chatter")
    /// * `msg_type` - Message type (e.g., "std_msgs/msg/Int32")
    /// * `data` - Message data as YAML (e.g., "{data: 42}")
    /// * `rate` - Publishing rate in Hz
    /// * `distro` - ROS distro (e.g., "humble")
    pub fn topic_pub(
        topic: &str,
        msg_type: &str,
        data: &str,
        rate: u32,
        distro: &str,
    ) -> TestResult<Self> {
        let env_setup = ros2_env_setup(distro);
        let cmd = format!(
            "{env_setup} && timeout 30 ros2 topic pub -r {rate} {topic} {msg_type} \"{data}\" --qos-reliability best_effort"
        );

        let handle = Command::new("bash")
            .args(["-c", &cmd])
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| {
                TestError::ProcessFailed(format!("Failed to start ros2 topic pub: {e}"))
            })?;

        Ok(Self {
            handle,
            name: format!("ros2 topic pub {topic}"),
        })
    }

    /// Wait for output and return it
    pub fn wait_for_output(&mut self, timeout: Duration) -> TestResult<String> {
        use std::io::Read;

        let start = std::time::Instant::now();
        let mut output = String::new();

        let mut stdout = self
            .handle
            .stdout
            .take()
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

    /// Kill the process
    pub fn kill(&mut self) {
        let _ = self.handle.kill();
        let _ = self.handle.wait();
    }

    /// Check if process is still running
    pub fn is_running(&mut self) -> bool {
        matches!(self.handle.try_wait(), Ok(None))
    }
}

impl Drop for Ros2Process {
    fn drop(&mut self) {
        self.kill();
    }
}

/// Helper to collect output from a process with timeout
pub fn collect_ros2_output(process: &mut Ros2Process, timeout: Duration) -> String {
    process.wait_for_output(timeout).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ros2_env_setup() {
        let setup = ros2_env_setup("humble");
        assert!(setup.contains("/opt/ros/humble"));
        assert!(setup.contains("rmw_zenoh_cpp"));
    }

    #[test]
    fn test_ros2_detection() {
        // Just verify detection works, don't require ROS 2
        let available = is_ros2_available();
        eprintln!("ROS 2 available: {}", available);
    }

    #[test]
    fn test_rmw_zenoh_detection() {
        let available = is_rmw_zenoh_available();
        eprintln!("rmw_zenoh_cpp available: {}", available);
    }
}
