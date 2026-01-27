//! Binary build helpers for integration tests
//!
//! Provides functions to build test binaries with caching support.

use crate::{project_root, TestError, TestResult};
use duct::cmd;
use once_cell::sync::OnceCell;
use std::path::{Path, PathBuf};

/// Cached path to the qemu-test binary
static QEMU_TEST_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native-rs-talker binary
static NATIVE_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native-rs-listener binary
static NATIVE_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Build the qemu-test example and return its path
///
/// Uses OnceLock to cache the build, so subsequent calls are fast.
pub fn build_qemu_test() -> TestResult<&'static Path> {
    QEMU_TEST_BINARY
        .get_or_try_init(|| {
            let root = project_root();
            let example_dir = root.join("examples/qemu-rs-test");

            eprintln!("Building qemu-test...");

            let output = cmd!(
                "cargo",
                "build",
                "--release",
                "--target",
                "thumbv7m-none-eabi"
            )
            .dir(&example_dir)
            .stderr_to_stdout()
            .stdout_capture()
            .unchecked()
            .run()
            .map_err(|e| TestError::BuildFailed(e.to_string()))?;

            if !output.status.success() {
                return Err(TestError::BuildFailed(
                    String::from_utf8_lossy(&output.stdout).to_string(),
                ));
            }

            let binary_path = example_dir.join("target/thumbv7m-none-eabi/release/qemu-rs-test");

            if !binary_path.exists() {
                return Err(TestError::BuildFailed(format!(
                    "Binary not found after build: {}",
                    binary_path.display()
                )));
            }

            Ok(binary_path)
        })
        .map(|p| p.as_path())
}

/// Build an example from the examples directory
///
/// # Arguments
/// * `name` - Example directory name (e.g., "native-rs-talker")
/// * `binary_name` - Actual binary name (e.g., "talker")
/// * `features` - Optional features to enable
/// * `target` - Optional target triple (e.g., "thumbv7m-none-eabi")
///
/// # Returns
/// Path to the built binary
pub fn build_example(
    name: &str,
    binary_name: &str,
    features: Option<&[&str]>,
    target: Option<&str>,
) -> TestResult<PathBuf> {
    let root = project_root();
    let example_dir = root.join(format!("examples/{}", name));

    if !example_dir.exists() {
        return Err(TestError::BuildFailed(format!(
            "Example directory not found: {}",
            example_dir.display()
        )));
    }

    eprintln!("Building {}...", name);

    let mut args = vec!["build", "--release"];

    if let Some(target) = target {
        args.push("--target");
        args.push(target);
    }

    let features_str: String;
    if let Some(features) = features {
        features_str = features.join(",");
        args.push("--features");
        args.push(&features_str);
    }

    let output = cmd("cargo", &args)
        .dir(&example_dir)
        .stderr_to_stdout()
        .stdout_capture()
        .unchecked()
        .run()
        .map_err(|e| TestError::BuildFailed(e.to_string()))?;

    if !output.status.success() {
        return Err(TestError::BuildFailed(
            String::from_utf8_lossy(&output.stdout).to_string(),
        ));
    }

    // Determine binary path using the actual binary name
    let binary_path = if let Some(target) = target {
        example_dir.join(format!("target/{}/release/{}", target, binary_name))
    } else {
        example_dir.join(format!("target/release/{}", binary_name))
    };

    if !binary_path.exists() {
        return Err(TestError::BuildFailed(format!(
            "Binary not found after build: {}",
            binary_path.display()
        )));
    }

    Ok(binary_path)
}

/// Build native-rs-talker with zenoh feature (cached)
pub fn build_native_talker() -> TestResult<&'static Path> {
    NATIVE_TALKER_BINARY
        .get_or_try_init(|| build_example("native-rs-talker", "talker", Some(&["zenoh"]), None))
        .map(|p| p.as_path())
}

/// Build native-rs-listener with zenoh feature (cached)
pub fn build_native_listener() -> TestResult<&'static Path> {
    NATIVE_LISTENER_BINARY
        .get_or_try_init(|| build_example("native-rs-listener", "listener", Some(&["zenoh"]), None))
        .map(|p| p.as_path())
}

/// rstest fixture that provides the qemu-test binary path
#[rstest::fixture]
pub fn qemu_binary() -> PathBuf {
    build_qemu_test()
        .expect("Failed to build qemu-test")
        .to_path_buf()
}

/// rstest fixture that provides the native-rs-talker binary path
#[rstest::fixture]
pub fn talker_binary() -> PathBuf {
    build_native_talker()
        .expect("Failed to build native-rs-talker")
        .to_path_buf()
}

/// rstest fixture that provides the native-rs-listener binary path
#[rstest::fixture]
pub fn listener_binary() -> PathBuf {
    build_native_listener()
        .expect("Failed to build native-rs-listener")
        .to_path_buf()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_project_root_has_examples() {
        let root = project_root();
        assert!(root.join("examples").exists());
    }
}
