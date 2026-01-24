//! Managed process utilities for integration tests
//!
//! Provides RAII-based process management with automatic cleanup.

use crate::TestError;
use std::process::{Child, Command, Stdio};
use std::time::Duration;

/// Managed process with automatic cleanup
///
/// Wraps a child process and ensures it is killed on drop.
/// Used for running talker/listener binaries and other test processes.
///
/// # Example
///
/// ```ignore
/// let mut proc = ManagedProcess::spawn(&binary_path, &["--tcp", "127.0.0.1:7447"], "talker")?;
/// std::thread::sleep(Duration::from_secs(5));
/// let output = proc.wait_for_output(Duration::from_secs(2))?;
/// // Process is automatically killed on drop
/// ```
pub struct ManagedProcess {
    handle: Child,
    name: String,
}

impl ManagedProcess {
    /// Spawn a new managed process
    ///
    /// # Arguments
    /// * `binary` - Path to the executable
    /// * `args` - Command line arguments
    /// * `name` - Human-readable name for error messages
    pub fn spawn(
        binary: &std::path::Path,
        args: &[&str],
        name: impl Into<String>,
    ) -> Result<Self, TestError> {
        let name = name.into();
        let handle = Command::new(binary)
            .args(args)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| TestError::ProcessFailed(format!("Failed to spawn {}: {}", name, e)))?;

        Ok(Self { handle, name })
    }

    /// Spawn a process from a Command builder
    ///
    /// # Arguments
    /// * `command` - Pre-configured Command builder
    /// * `name` - Human-readable name for error messages
    pub fn spawn_command(mut command: Command, name: impl Into<String>) -> Result<Self, TestError> {
        let name = name.into();
        let handle = command
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| TestError::ProcessFailed(format!("Failed to spawn {}: {}", name, e)))?;

        Ok(Self { handle, name })
    }

    /// Get the process name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Check if process is still running
    pub fn is_running(&mut self) -> bool {
        matches!(self.handle.try_wait(), Ok(None))
    }

    /// Get mutable access to the underlying Child handle
    ///
    /// Use with caution - modifications may affect cleanup behavior.
    pub fn handle_mut(&mut self) -> &mut Child {
        &mut self.handle
    }

    /// Wait for output with timeout
    ///
    /// Collects stdout from the process until:
    /// - The timeout is reached
    /// - The process exits
    /// - An error occurs
    ///
    /// The process is killed when the timeout is reached.
    pub fn wait_for_output(&mut self, timeout: Duration) -> Result<String, TestError> {
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
                    // Process exited, read remaining output
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

    /// Kill the process and wait for it to exit
    pub fn kill(&mut self) {
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

/// Check if zenohd is available in PATH
pub fn is_zenohd_available() -> bool {
    Command::new("zenohd")
        .arg("--version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Skip test if zenohd is not available
///
/// Returns `false` if zenohd is not available, printing a skip message.
/// Returns `true` if zenohd is available and the test should proceed.
///
/// # Example
///
/// ```ignore
/// #[test]
/// fn test_something() {
///     if !require_zenohd() {
///         return;
///     }
///     // ... test code
/// }
/// ```
pub fn require_zenohd() -> bool {
    if !is_zenohd_available() {
        eprintln!("Skipping test: zenohd not found");
        return false;
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zenohd_detection() {
        let available = is_zenohd_available();
        eprintln!("zenohd available: {}", available);
    }
}
