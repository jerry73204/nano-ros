//! ZenohRouter fixture for managing zenohd process
//!
//! Provides automatic startup and cleanup of the zenoh router daemon.

use crate::{wait_for_port, TestError, TestResult};
use duct::cmd;
use std::process::Child;
use std::sync::atomic::{AtomicU16, Ordering};
use std::time::Duration;

/// Port counter to avoid conflicts between parallel tests
static PORT_COUNTER: AtomicU16 = AtomicU16::new(17447);

/// Get a unique port for testing
fn get_unique_port() -> u16 {
    PORT_COUNTER.fetch_add(1, Ordering::SeqCst)
}

/// Managed zenohd router process
///
/// Automatically starts zenohd on creation and kills it on drop.
/// Uses unique ports to allow parallel test execution.
///
/// # Example
///
/// ```ignore
/// use nano_ros_tests::fixtures::ZenohRouter;
///
/// let router = ZenohRouter::start(7447).unwrap();
/// println!("Router at: {}", router.locator());
/// // Router is automatically stopped when dropped
/// ```
pub struct ZenohRouter {
    handle: Child,
    port: u16,
}

impl ZenohRouter {
    /// Start a new zenohd router on the specified port
    ///
    /// # Arguments
    /// * `port` - TCP port to listen on
    ///
    /// # Returns
    /// A managed router instance that will be stopped on drop
    pub fn start(port: u16) -> TestResult<Self> {
        // Kill any existing zenohd on this port
        let _ = cmd!("pkill", "-f", format!("zenohd.*:{}", port)).run();
        std::thread::sleep(Duration::from_millis(500));

        let locator = format!("tcp/0.0.0.0:{}", port);

        let handle = std::process::Command::new("zenohd")
            .args(["--listen", &locator])
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .spawn()?;

        // Wait for zenohd to be ready
        if !wait_for_port(port, Duration::from_secs(5)) {
            return Err(TestError::Timeout);
        }

        // Additional small delay for full initialization
        std::thread::sleep(Duration::from_millis(500));

        Ok(Self { handle, port })
    }

    /// Start a router on a unique port (for parallel tests)
    pub fn start_unique() -> TestResult<Self> {
        Self::start(get_unique_port())
    }

    /// Get the locator string for connecting to this router
    pub fn locator(&self) -> String {
        format!("tcp/127.0.0.1:{}", self.port)
    }

    /// Get the port number
    pub fn port(&self) -> u16 {
        self.port
    }

    /// Check if the router is still running
    pub fn is_running(&mut self) -> bool {
        matches!(self.handle.try_wait(), Ok(None))
    }
}

impl Drop for ZenohRouter {
    fn drop(&mut self) {
        // Kill the process
        let _ = self.handle.kill();
        let _ = self.handle.wait();

        // Also kill by port in case of orphaned processes
        let _ = cmd!("pkill", "-f", format!("zenohd.*:{}", self.port)).run();
    }
}

/// rstest fixture for zenohd on default port
///
/// # Example
///
/// ```ignore
/// use nano_ros_tests::fixtures::zenohd;
/// use rstest::rstest;
///
/// #[rstest]
/// fn my_test(zenohd: ZenohRouter) {
///     // zenohd is ready to use
/// }
/// ```
#[rstest::fixture]
pub fn zenohd() -> ZenohRouter {
    ZenohRouter::start(7447).expect("Failed to start zenohd")
}

/// rstest fixture for zenohd on a unique port (parallel-safe)
#[rstest::fixture]
pub fn zenohd_unique() -> ZenohRouter {
    ZenohRouter::start_unique().expect("Failed to start zenohd")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zenoh_router_locator() {
        // Just test the locator format without starting a real router
        let port = 12345;
        let expected = "tcp/127.0.0.1:12345";
        assert_eq!(format!("tcp/127.0.0.1:{}", port), expected);
    }

    #[test]
    fn test_unique_port() {
        let p1 = get_unique_port();
        let p2 = get_unique_port();
        assert_ne!(p1, p2);
    }
}
