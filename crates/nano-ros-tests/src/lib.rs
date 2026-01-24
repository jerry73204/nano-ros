//! Integration test framework for nano-ros
//!
//! This crate provides fixtures and utilities for testing nano-ros components:
//! - Process management (zenohd, QEMU, Zephyr)
//! - Binary building helpers
//! - Output assertion utilities
//!
//! # Example
//!
//! ```ignore
//! use nano_ros_tests::fixtures::zenohd;
//! use rstest::rstest;
//!
//! #[rstest]
//! fn test_pubsub(zenohd: ZenohRouter) {
//!     // zenohd is automatically started and cleaned up
//! }
//! ```

pub mod fixtures;
pub mod process;
pub mod qemu;
pub mod ros2;

use std::io::{BufRead, BufReader};
use std::net::TcpStream;
use std::process::{Child, ChildStdout};
use std::time::{Duration, Instant};

/// Error type for test utilities
#[derive(Debug, thiserror::Error)]
pub enum TestError {
    #[error("Process failed to start: {0}")]
    ProcessStart(#[from] std::io::Error),

    #[error("Process failed: {0}")]
    ProcessFailed(String),

    #[error("Timeout waiting for condition")]
    Timeout,

    #[error("Build failed: {0}")]
    BuildFailed(String),

    #[error("Output parsing error: {0}")]
    OutputParse(String),
}

pub type TestResult<T> = Result<T, TestError>;

/// Wait for a TCP port to become available
///
/// # Arguments
/// * `port` - The port number to check
/// * `timeout` - Maximum time to wait
///
/// # Returns
/// `true` if the port is available within the timeout, `false` otherwise
pub fn wait_for_port(port: u16, timeout: Duration) -> bool {
    let start = Instant::now();
    let addr = format!("127.0.0.1:{}", port);

    while start.elapsed() < timeout {
        if TcpStream::connect(&addr).is_ok() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    false
}

/// Wait for a specific pattern in process output
///
/// # Arguments
/// * `reader` - A buffered reader from the process stdout
/// * `pattern` - The pattern to search for
/// * `timeout` - Maximum time to wait
///
/// # Returns
/// The matching line if found within timeout
pub fn wait_for_pattern(
    reader: &mut BufReader<ChildStdout>,
    pattern: &str,
    timeout: Duration,
) -> TestResult<String> {
    let start = Instant::now();
    let mut line = String::new();

    while start.elapsed() < timeout {
        line.clear();
        match reader.read_line(&mut line) {
            Ok(0) => {
                // EOF - process may have exited
                std::thread::sleep(Duration::from_millis(50));
                continue;
            }
            Ok(_) => {
                if line.contains(pattern) {
                    return Ok(line);
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                std::thread::sleep(Duration::from_millis(50));
                continue;
            }
            Err(e) => return Err(TestError::ProcessStart(e)),
        }
    }
    Err(TestError::Timeout)
}

/// Collect all output from a process until it exits or timeout
///
/// # Arguments
/// * `child` - The child process
/// * `timeout` - Maximum time to wait
///
/// # Returns
/// The collected stdout as a string
pub fn collect_output(mut child: Child, timeout: Duration) -> TestResult<String> {
    use std::io::Read;

    let start = Instant::now();
    let mut output = String::new();

    if let Some(mut stdout) = child.stdout.take() {
        // Set up non-blocking read with timeout
        let mut buffer = [0u8; 4096];
        while start.elapsed() < timeout {
            match stdout.read(&mut buffer) {
                Ok(0) => break, // EOF
                Ok(n) => {
                    output.push_str(&String::from_utf8_lossy(&buffer[..n]));
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    std::thread::sleep(Duration::from_millis(50));
                }
                Err(_) => break,
            }

            // Check if process exited
            if let Ok(Some(_)) = child.try_wait() {
                // Read any remaining output
                let _ = stdout.read_to_string(&mut output);
                break;
            }
        }
    }

    // Ensure process is terminated
    let _ = child.kill();
    let _ = child.wait();

    Ok(output)
}

/// Assert that output contains all specified patterns
///
/// # Arguments
/// * `output` - The output string to check
/// * `patterns` - Patterns that must all be present
///
/// # Panics
/// If any pattern is not found in the output
pub fn assert_output_contains(output: &str, patterns: &[&str]) {
    for pattern in patterns {
        assert!(
            output.contains(pattern),
            "Expected output to contain '{}', but it was not found.\nOutput:\n{}",
            pattern,
            output
        );
    }
}

/// Assert that output contains none of the specified patterns
///
/// # Arguments
/// * `output` - The output string to check
/// * `patterns` - Patterns that must not be present
///
/// # Panics
/// If any pattern is found in the output
pub fn assert_output_excludes(output: &str, patterns: &[&str]) {
    for pattern in patterns {
        assert!(
            !output.contains(pattern),
            "Expected output to NOT contain '{}', but it was found.\nOutput:\n{}",
            pattern,
            output
        );
    }
}

/// Count occurrences of a pattern in output
pub fn count_pattern(output: &str, pattern: &str) -> usize {
    output.matches(pattern).count()
}

/// Get the project root directory
pub fn project_root() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_project_root() {
        let root = project_root();
        assert!(root.join("Cargo.toml").exists());
        assert!(root.join("crates").exists());
    }

    #[test]
    fn test_count_pattern() {
        let output = "[PASS] test1\n[PASS] test2\n[FAIL] test3\n[PASS] test4";
        assert_eq!(count_pattern(output, "[PASS]"), 3);
        assert_eq!(count_pattern(output, "[FAIL]"), 1);
    }

    #[test]
    fn test_assert_output_contains() {
        let output = "Hello world\nTest passed";
        assert_output_contains(output, &["Hello", "passed"]);
    }

    #[test]
    #[should_panic(expected = "Expected output to contain")]
    fn test_assert_output_contains_fails() {
        let output = "Hello world";
        assert_output_contains(output, &["missing"]);
    }
}
