//! Common test utilities for zenoh-pico integration tests.
//!
//! Provides a singleton zenohd process manager for tests that require network communication.
//!
//! # How it works
//!
//! 1. First test to need a router starts zenohd and records PID in a file
//! 2. Subsequent tests reuse the running zenohd
//! 3. When the test process exits, an atexit handler kills zenohd
//!
//! This handles both parallel and serial test execution.

use std::fs;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::time::Duration;

/// PID file location for the test router
fn pid_file_path() -> PathBuf {
    std::env::temp_dir().join("zenoh-pico-test-router.pid")
}

/// Lock file to prevent race conditions when starting the router
fn lock_file_path() -> PathBuf {
    std::env::temp_dir().join("zenoh-pico-test-router.lock")
}

/// Check if a process with the given PID is running
fn is_process_running(pid: u32) -> bool {
    // On Unix, sending signal 0 checks if process exists
    #[cfg(unix)]
    {
        // kill -0 just checks if process exists
        unsafe { libc::kill(pid as i32, 0) == 0 }
    }
    #[cfg(not(unix))]
    {
        // On non-Unix, just assume it's running if we have a PID
        let _ = pid;
        true
    }
}

/// Read PID from the PID file
fn read_pid_file() -> Option<u32> {
    let pid_file = pid_file_path();
    if !pid_file.exists() {
        return None;
    }

    let mut file = fs::File::open(&pid_file).ok()?;
    let mut contents = String::new();
    file.read_to_string(&mut contents).ok()?;
    contents.trim().parse().ok()
}

/// Write PID to the PID file
fn write_pid_file(pid: u32) -> std::io::Result<()> {
    let pid_file = pid_file_path();
    let mut file = fs::File::create(&pid_file)?;
    write!(file, "{}", pid)?;
    Ok(())
}

/// Remove the PID file
fn remove_pid_file() {
    let _ = fs::remove_file(pid_file_path());
}

/// Kill the router process (unused but kept for manual cleanup)
#[allow(dead_code)]
fn kill_router(pid: u32) {
    #[cfg(unix)]
    {
        unsafe {
            libc::kill(pid as i32, libc::SIGTERM);
        }
        // Give it time to exit gracefully
        std::thread::sleep(Duration::from_millis(100));
        // Force kill if still running
        if is_process_running(pid) {
            unsafe {
                libc::kill(pid as i32, libc::SIGKILL);
            }
        }
    }
    #[cfg(not(unix))]
    {
        // On non-Unix, try taskkill
        let _ = Command::new("taskkill")
            .args(["/PID", &pid.to_string(), "/F"])
            .output();
    }
}

/// Start zenohd router
fn start_zenohd() -> std::io::Result<Child> {
    Command::new("zenohd")
        .args(["--listen", "tcp/127.0.0.1:7447"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
}

/// Acquire a file lock (simple implementation)
struct FileLock {
    _file: fs::File,
}

impl FileLock {
    fn acquire() -> std::io::Result<Self> {
        let lock_file = lock_file_path();

        // Try to create lock file exclusively
        for _ in 0..100 {
            match fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&lock_file)
            {
                Ok(file) => return Ok(FileLock { _file: file }),
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
                    // Lock held by another process, wait and retry
                    std::thread::sleep(Duration::from_millis(50));
                }
                Err(e) => return Err(e),
            }
        }

        // Timeout - force remove stale lock
        let _ = fs::remove_file(&lock_file);
        let file = fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&lock_file)?;
        Ok(FileLock { _file: file })
    }
}

impl Drop for FileLock {
    fn drop(&mut self) {
        let _ = fs::remove_file(lock_file_path());
    }
}

/// Ensure a zenohd router is running.
///
/// This function:
/// 1. Checks if zenohd is already running (via PID file)
/// 2. If not, starts zenohd and records its PID
/// 3. Registers an atexit handler to clean up on process exit
///
/// Returns `Ok(())` if router is available, `Err` if zenohd couldn't be started.
pub fn ensure_router() -> Result<(), String> {
    // Acquire lock to prevent race conditions
    let _lock = FileLock::acquire().map_err(|e| format!("Failed to acquire lock: {}", e))?;

    // Check if router is already running
    if let Some(pid) = read_pid_file() {
        if is_process_running(pid) {
            // Verify it's actually accepting connections
            if std::net::TcpStream::connect("127.0.0.1:7447").is_ok() {
                return Ok(());
            }
            // Process exists but not accepting connections - wait a bit
            for _ in 0..30 {
                if std::net::TcpStream::connect("127.0.0.1:7447").is_ok() {
                    return Ok(());
                }
                std::thread::sleep(Duration::from_millis(100));
            }
        }
        // Stale PID file or unresponsive, remove it
        remove_pid_file();
    }

    // Start zenohd
    let child = start_zenohd()
        .map_err(|e| format!("Failed to start zenohd: {}. Is zenohd installed?", e))?;

    let pid = child.id();
    write_pid_file(pid).map_err(|e| format!("Failed to write PID file: {}", e))?;

    // NOTE: We intentionally do NOT kill zenohd on exit.
    // With nextest, each test runs in its own process, and killing zenohd
    // when one test exits would break other running tests.
    // Instead, we rely on stale PID detection on the next run.
    // If you need to manually clean up: kill $(cat /tmp/zenoh-pico-test-router.pid)

    // Wait for router to be ready
    std::thread::sleep(Duration::from_millis(1000));

    // Verify router is accepting connections
    for _ in 0..50 {
        if std::net::TcpStream::connect("127.0.0.1:7447").is_ok() {
            // Extra delay to ensure router is fully ready
            std::thread::sleep(Duration::from_millis(200));
            return Ok(());
        }
        std::thread::sleep(Duration::from_millis(100));
    }

    Err("Router started but not accepting connections".to_string())
}

/// Router address for tests
pub const TEST_ROUTER_ADDR: &str = "tcp/127.0.0.1:7447";

/// Open a zenoh-pico session with retries.
/// This is needed because the router might not be fully ready immediately.
pub fn open_session_with_retry() -> Result<zenoh_pico::Session, String> {
    use zenoh_pico::{Config, Session};

    for attempt in 0..5 {
        let config =
            Config::client(TEST_ROUTER_ADDR).map_err(|e| format!("Config error: {:?}", e))?;
        match Session::open(config) {
            Ok(session) => return Ok(session),
            Err(e) => {
                if attempt < 4 {
                    std::thread::sleep(Duration::from_millis(500));
                } else {
                    return Err(format!("Failed to open session after 5 attempts: {:?}", e));
                }
            }
        }
    }
    unreachable!()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ensure_router() {
        // This test verifies the router management works
        let result = ensure_router();
        if result.is_err() {
            eprintln!("Note: zenohd not available, skipping: {:?}", result);
            return;
        }

        // Verify we can connect
        assert!(std::net::TcpStream::connect("127.0.0.1:7447").is_ok());
    }
}
