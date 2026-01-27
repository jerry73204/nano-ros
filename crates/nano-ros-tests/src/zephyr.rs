//! Zephyr process fixture for embedded testing
//!
//! Provides managed Zephyr processes for testing native_sim and QEMU targets.

use crate::{project_root, TestError, TestResult};
use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

/// Zephyr platform variants
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZephyrPlatform {
    /// Native simulator (x86_64, runs directly on host)
    NativeSim,
    /// QEMU ARM Cortex-M3 emulation
    QemuArm,
}

impl ZephyrPlatform {
    /// Get the west board specifier for this platform
    pub fn board_spec(&self) -> &'static str {
        match self {
            ZephyrPlatform::NativeSim => "native_sim/native/64",
            ZephyrPlatform::QemuArm => "qemu_cortex_m3",
        }
    }
}

/// Managed Zephyr process for native_sim or QEMU
///
/// Starts a Zephyr application and captures output.
/// Automatically kills the process on drop.
///
/// # Example
///
/// ```ignore
/// use nano_ros_tests::zephyr::{ZephyrProcess, ZephyrPlatform};
/// use std::path::Path;
/// use std::time::Duration;
///
/// let workspace = zephyr_workspace_path().unwrap();
/// let binary = workspace.join("build/zephyr/zephyr.exe");
/// let mut zephyr = ZephyrProcess::start(&binary, ZephyrPlatform::NativeSim).unwrap();
/// let output = zephyr.wait_for_output(Duration::from_secs(15)).unwrap();
/// ```
pub struct ZephyrProcess {
    handle: Child,
    platform: ZephyrPlatform,
}

impl ZephyrProcess {
    /// Start a Zephyr application
    ///
    /// # Arguments
    /// * `binary` - Path to the Zephyr executable (zephyr.exe for native_sim, zephyr.elf for QEMU)
    /// * `platform` - Target platform
    ///
    /// # Returns
    /// A managed Zephyr process
    pub fn start(binary: &Path, platform: ZephyrPlatform) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Zephyr binary not found: {}",
                binary.display()
            )));
        }

        let handle = match platform {
            ZephyrPlatform::NativeSim => {
                // native_sim runs directly
                Command::new(binary)
                    .stdout(Stdio::piped())
                    .stderr(Stdio::piped())
                    .spawn()?
            }
            ZephyrPlatform::QemuArm => {
                // QEMU ARM requires qemu-system-arm
                Command::new("qemu-system-arm")
                    .args([
                        "-cpu",
                        "cortex-m3",
                        "-machine",
                        "lm3s6965evb",
                        "-nographic",
                        "-kernel",
                    ])
                    .arg(binary)
                    .stdout(Stdio::piped())
                    .stderr(Stdio::piped())
                    .spawn()?
            }
        };

        Ok(Self { handle, platform })
    }

    /// Get the platform this process is running on
    pub fn platform(&self) -> ZephyrPlatform {
        self.platform
    }

    /// Wait for output with timeout
    ///
    /// Collects stdout from the process. Since Zephyr native_sim processes
    /// typically output everything quickly and then wait indefinitely,
    /// this uses a thread to avoid blocking on read().
    ///
    /// # Arguments
    /// * `timeout` - Maximum time to wait
    ///
    /// # Returns
    /// The collected stdout as a string
    pub fn wait_for_output(&mut self, timeout: Duration) -> TestResult<String> {
        use std::sync::mpsc;
        use std::thread;

        let mut stdout = self
            .handle
            .stdout
            .take()
            .ok_or_else(|| TestError::ProcessFailed("No stdout".to_string()))?;

        // Spawn a thread to read stdout (avoids blocking)
        let (tx, rx) = mpsc::channel();
        thread::spawn(move || {
            let mut output = String::new();
            let mut buffer = [0u8; 4096];
            loop {
                match stdout.read(&mut buffer) {
                    Ok(0) => break, // EOF
                    Ok(n) => {
                        output.push_str(&String::from_utf8_lossy(&buffer[..n]));
                        // Send partial output
                        let _ = tx.send(output.clone());
                    }
                    Err(_) => break,
                }
            }
        });

        // Wait for output with timeout
        let start = Instant::now();
        let mut last_output = String::new();

        while start.elapsed() < timeout {
            // Check if process exited
            if let Ok(Some(_)) = self.handle.try_wait() {
                // Process exited, collect any remaining output
                std::thread::sleep(Duration::from_millis(100));
                while let Ok(output) = rx.try_recv() {
                    last_output = output;
                }
                break;
            }

            // Check for new output
            if let Ok(output) = rx.try_recv() {
                last_output = output;

                // Check for completion/error markers (Zephyr outputs error and stops)
                if last_output.contains("Failed to create context")
                    || last_output.contains("session error")
                    || last_output.contains("SUCCESS")
                    || last_output.contains("COMPLETE")
                {
                    // Give it a moment for any trailing output
                    std::thread::sleep(Duration::from_millis(200));
                    while let Ok(output) = rx.try_recv() {
                        last_output = output;
                    }
                    break;
                }
            }

            std::thread::sleep(Duration::from_millis(100));
        }

        // Kill the process if still running
        let _ = self.handle.kill();

        if last_output.is_empty() {
            Err(TestError::Timeout)
        } else {
            Ok(last_output)
        }
    }

    /// Kill the Zephyr process
    pub fn kill(&mut self) -> TestResult<()> {
        self.handle.kill()?;
        Ok(())
    }

    /// Check if process is still running
    pub fn is_running(&mut self) -> bool {
        matches!(self.handle.try_wait(), Ok(None))
    }
}

impl Drop for ZephyrProcess {
    fn drop(&mut self) {
        let _ = self.handle.kill();
        let _ = self.handle.wait();
    }
}

// =============================================================================
// Zephyr Availability Checks
// =============================================================================

/// Get the path to the Zephyr workspace
///
/// Checks in order:
/// 1. `ZEPHYR_NANO_ROS` environment variable
/// 2. `zephyr-workspace` symlink in project root
/// 3. Sibling workspace `../nano-ros-workspace/`
///
/// # Returns
/// Path to the workspace, or None if not found
pub fn zephyr_workspace_path() -> Option<PathBuf> {
    // 1. Environment variable
    if let Ok(path) = std::env::var("ZEPHYR_NANO_ROS") {
        let path = PathBuf::from(path);
        if path.exists() {
            return Some(path);
        }
    }

    let root = project_root();

    // 2. zephyr-workspace symlink
    let symlink = root.join("zephyr-workspace");
    if symlink.is_symlink() || symlink.is_dir() {
        if let Ok(resolved) = std::fs::canonicalize(&symlink) {
            if resolved.exists() {
                return Some(resolved);
            }
        }
    }

    // 3. Sibling workspace
    let sibling = root.parent()?.join("nano-ros-workspace");
    if sibling.exists() {
        return Some(sibling);
    }

    None
}

/// Check if west command is available
pub fn is_west_available() -> bool {
    Command::new("west")
        .arg("--version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Check if Zephyr workspace is configured
pub fn is_zephyr_workspace_available() -> bool {
    zephyr_workspace_path()
        .map(|p| p.join("zephyr").exists())
        .unwrap_or(false)
}

/// Check if TAP interface is configured for Zephyr networking
///
/// Returns true if either the bridge network (zeth0/zeth1) or legacy single
/// interface (zeth) is available.
pub fn is_tap_interface_available() -> bool {
    // Check for bridge network first (preferred)
    if is_bridge_network_available() {
        return true;
    }

    // Fall back to checking for any zeth* interface
    Command::new("sh")
        .args(["-c", "ip link show | grep -q 'zeth'"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Check if bridge network is configured for multiple Zephyr instances
///
/// This checks for the full bridge setup with zeth0 and zeth1 interfaces,
/// which is required for E2E talker-listener tests.
pub fn is_bridge_network_available() -> bool {
    // Check if bridge exists
    let bridge_ok = Command::new("ip")
        .args(["link", "show", "zeth-br"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false);

    // Check if zeth0 (talker) exists
    let zeth0_ok = Command::new("ip")
        .args(["link", "show", "zeth0"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false);

    // Check if zeth1 (listener) exists
    let zeth1_ok = Command::new("ip")
        .args(["link", "show", "zeth1"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false);

    bridge_ok && zeth0_ok && zeth1_ok
}

/// Require bridge network for E2E tests
///
/// Returns `false` if bridge network is not configured, printing a skip message.
pub fn require_bridge_network() -> bool {
    if !is_bridge_network_available() {
        eprintln!("Skipping test: Bridge network not configured");
        eprintln!("  Run: sudo ./scripts/zephyr/setup-network.sh");
        return false;
    }
    true
}

/// Check if all Zephyr prerequisites are available
///
/// Checks:
/// - west command available
/// - Zephyr workspace configured
/// - TAP interface (zeth) available for networking tests
pub fn is_zephyr_available() -> bool {
    is_west_available() && is_zephyr_workspace_available() && is_tap_interface_available()
}

/// Skip test if Zephyr is not available
///
/// Returns `false` if Zephyr prerequisites are not met, printing a skip message.
/// Returns `true` if Zephyr is available and the test should proceed.
pub fn require_zephyr() -> bool {
    if !is_west_available() {
        eprintln!("Skipping test: west not found");
        return false;
    }
    if !is_zephyr_workspace_available() {
        eprintln!("Skipping test: Zephyr workspace not found");
        eprintln!("  Run: ./scripts/zephyr/setup.sh");
        return false;
    }
    if !is_tap_interface_available() {
        eprintln!("Skipping test: TAP interface (zeth) not found");
        eprintln!("  Run: sudo ./scripts/zephyr/setup-network.sh");
        return false;
    }
    true
}

// =============================================================================
// Zephyr Build Helpers
// =============================================================================

use once_cell::sync::OnceCell;

/// Cached path to built zephyr-rs-talker binary
static ZEPHYR_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to built zephyr-rs-listener binary
static ZEPHYR_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Get the build directory name for an example
///
/// Returns a unique build directory to allow simultaneous builds of talker and listener.
fn build_dir_for_example(example_name: &str) -> &'static str {
    match example_name {
        "zephyr-rs-talker" => "build-talker",
        "zephyr-rs-listener" => "build-listener",
        _ => "build",
    }
}

/// Get path to Zephyr binary, using existing build if available
///
/// This function checks if a Zephyr binary already exists in the build directory
/// and returns it without rebuilding. Only builds if forced or binary doesn't exist.
///
/// # Arguments
/// * `example_name` - Name of the example directory (e.g., "zephyr-rs-talker")
/// * `platform` - Target platform
/// * `force_build` - If true, always rebuild even if binary exists
///
/// # Returns
/// Path to the binary
pub fn get_or_build_zephyr_example(
    example_name: &str,
    platform: ZephyrPlatform,
    force_build: bool,
) -> TestResult<PathBuf> {
    let workspace = zephyr_workspace_path()
        .ok_or_else(|| TestError::BuildFailed("Zephyr workspace not found".to_string()))?;

    let build_dir = build_dir_for_example(example_name);

    // Determine binary path based on platform
    let binary_path = match platform {
        ZephyrPlatform::NativeSim => workspace.join(format!("{}/zephyr/zephyr.exe", build_dir)),
        ZephyrPlatform::QemuArm => workspace.join(format!("{}/zephyr/zephyr.elf", build_dir)),
    };

    // If binary exists and we're not forcing a rebuild, use it
    if !force_build && binary_path.exists() {
        eprintln!("Using existing Zephyr binary: {}", binary_path.display());
        return Ok(binary_path);
    }

    // Otherwise, build it
    build_zephyr_example(example_name, platform)
}

/// Build a Zephyr example using west (cached)
///
/// For zephyr-rs-talker and zephyr-rs-listener, results are cached to avoid
/// repeated builds within the same test run.
pub fn build_zephyr_example_cached(
    example_name: &str,
    platform: ZephyrPlatform,
) -> TestResult<&'static Path> {
    match example_name {
        "zephyr-rs-talker" => ZEPHYR_TALKER_BINARY
            .get_or_try_init(|| build_zephyr_example(example_name, platform))
            .map(|p| p.as_path()),
        "zephyr-rs-listener" => ZEPHYR_LISTENER_BINARY
            .get_or_try_init(|| build_zephyr_example(example_name, platform))
            .map(|p| p.as_path()),
        _ => build_zephyr_example(example_name, platform)
            .map(|p| Box::leak(Box::new(p)) as &'static Path),
    }
}

/// Build a Zephyr example using west
///
/// Each example is built to its own directory (build-talker/, build-listener/)
/// to allow both to exist simultaneously.
///
/// # Arguments
/// * `example_name` - Name of the example directory (e.g., "zephyr-rs-talker")
/// * `platform` - Target platform
///
/// # Returns
/// Path to the built binary
pub fn build_zephyr_example(example_name: &str, platform: ZephyrPlatform) -> TestResult<PathBuf> {
    let workspace = zephyr_workspace_path()
        .ok_or_else(|| TestError::BuildFailed("Zephyr workspace not found".to_string()))?;

    let root = project_root();
    let example_path = root.join(format!("examples/{}", example_name));

    if !example_path.exists() {
        return Err(TestError::BuildFailed(format!(
            "Example not found: {}",
            example_path.display()
        )));
    }

    let build_dir = build_dir_for_example(example_name);
    eprintln!(
        "Building {} for {} (build dir: {})...",
        example_name,
        platform.board_spec(),
        build_dir
    );

    // Build with separate build directory for each example
    // This allows talker and listener to coexist
    let output = Command::new("west")
        .args([
            "build",
            "-b",
            platform.board_spec(),
            "-d",
            build_dir,
            "-p",
            "auto",
        ])
        .arg(&example_path)
        .current_dir(&workspace)
        .env("ZEPHYR_BASE", workspace.join("zephyr"))
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .map_err(|e| TestError::BuildFailed(format!("Failed to run west: {}", e)))?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        let stdout = String::from_utf8_lossy(&output.stdout);
        return Err(TestError::BuildFailed(format!(
            "west build failed:\nstdout: {}\nstderr: {}",
            stdout, stderr
        )));
    }

    // Determine binary path based on platform
    let binary_path = match platform {
        ZephyrPlatform::NativeSim => workspace.join(format!("{}/zephyr/zephyr.exe", build_dir)),
        ZephyrPlatform::QemuArm => workspace.join(format!("{}/zephyr/zephyr.elf", build_dir)),
    };

    if !binary_path.exists() {
        return Err(TestError::BuildFailed(format!(
            "Binary not found after build: {}",
            binary_path.display()
        )));
    }

    Ok(binary_path)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_platform_board_spec() {
        assert_eq!(
            ZephyrPlatform::NativeSim.board_spec(),
            "native_sim/native/64"
        );
        assert_eq!(ZephyrPlatform::QemuArm.board_spec(), "qemu_cortex_m3");
    }

    #[test]
    fn test_west_detection() {
        let available = is_west_available();
        eprintln!("west available: {}", available);
    }

    #[test]
    fn test_workspace_detection() {
        if let Some(path) = zephyr_workspace_path() {
            eprintln!("Zephyr workspace: {}", path.display());
            assert!(path.exists());
        } else {
            eprintln!("Zephyr workspace not found");
        }
    }

    #[test]
    fn test_tap_interface_detection() {
        let available = is_tap_interface_available();
        eprintln!("TAP interface (zeth) available: {}", available);
    }

    #[test]
    fn test_bridge_network_detection() {
        let available = is_bridge_network_available();
        eprintln!("Bridge network available: {}", available);
    }
}
