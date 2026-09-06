// Compilation tests - verify generated code compiles successfully
use rosidl_codegen::{GeneratorError, generate_message_package};
use rosidl_parser::parse_message;
use std::{collections::HashSet, fs, process::Command};
use tempfile::TempDir;

/// Helper to create a minimal Cargo.toml for testing compilation
fn create_test_cargo_toml(pkg_name: &str, needs_big_array: bool) -> String {
    let big_array_dep = if needs_big_array {
        r#"big-array = { version = "0.5", features = ["serde"] }
"#
    } else {
        ""
    };

    format!(
        r#"[package]
name = "{}"
version = "0.1.0"
edition = "2021"

[dependencies]
serde = {{ version = "1.0", features = ["derive"], optional = true }}
{}
[features]
default = []
serde = ["dep:serde"]

[lib]
path = "src/lib.rs"
"#,
        pkg_name, big_array_dep
    )
}

/// Helper to create a stub for rosidl_runtime_rs types (for compilation testing)
fn create_rosidl_runtime_stub() -> String {
    r#"
// Stub implementations of rosidl_runtime_rs types for compilation testing
pub mod rosidl_runtime_rs {
    use serde::{Deserialize, Serialize};

    pub type String = std::string::String;
    pub type WString = std::string::String;

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct BoundedString<const N: usize>(std::string::String);

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct BoundedWString<const N: usize>(std::string::String);

    #[repr(C)]
    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct Sequence<T>(Vec<T>);

    #[repr(C)]
    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct BoundedSequence<T, const N: usize>(Vec<T>);

    impl<T> Default for Sequence<T> {
        fn default() -> Self {
            Sequence(Vec::new())
        }
    }

    impl<T, const N: usize> Default for BoundedSequence<T, N> {
        fn default() -> Self {
            BoundedSequence(Vec::new())
        }
    }

    impl<const N: usize> Default for BoundedString<N> {
        fn default() -> Self {
            BoundedString(std::string::String::new())
        }
    }

    impl<const N: usize> Default for BoundedWString<N> {
        fn default() -> Self {
            BoundedWString(std::string::String::new())
        }
    }

    // Trait definitions for ROS runtime
    pub trait SequenceElement: Sized {
        type RmwType;
    }

    pub trait SequenceAlloc: Sized {
        fn sequence_init(seq: &mut Sequence<Self>, size: usize) -> bool;
        fn sequence_fini(seq: &mut Sequence<Self>);
        fn sequence_copy(in_seq: &Sequence<Self>, out_seq: &mut Sequence<Self>) -> bool;
    }

    pub trait Message: Clone {
        type RmwMsg: Clone;
        fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg>;
        fn from_rmw_message(msg: Self::RmwMsg) -> Self;
    }

    pub trait RmwMessage: Sized {
        const TYPE_NAME: &'static str;
        fn get_type_support() -> *const std::ffi::c_void;
    }

    pub trait Service {
        type Request;
        type Response;
        fn get_type_support() -> *const std::ffi::c_void;
    }

    pub trait Action {
        type Goal;
        type Result;
        type Feedback;
        type FeedbackMessage;
        type SendGoalService;
        type CancelGoalService;
        type GetResultService;
        fn get_type_support() -> *const std::ffi::c_void;
    }
}
"#
    .to_string()
}

/// Issue 1160 — this was `cargo_available() -> bool` and four test bodies wrote
/// `if !cargo_available() { eprintln!("Skipping…"); return Ok(()); }`, which is
/// a PASS.
///
/// The branch was DEAD, and that is the reason to remove it rather than the
/// reason to keep it: every one of these tests is spawned BY cargo and then
/// spawns `cargo check` itself, so a host where the probe answers `false` is a
/// host where the test binary could not have been launched. Issue 1135 fixed
/// exactly this shape in `native_api.rs` (×19) and `rtos_e2e.rs` (×3) — a dead
/// guard is the live defect's shape and a standing invitation to add an arm
/// that returns `false`.
///
/// If cargo really is unreachable, that is a broken toolchain and a red is the
/// honest answer, not a green over four compilations nobody ran.
fn require_cargo() {
    assert!(
        Command::new("cargo").arg("--version").output().is_ok(),
        "`cargo` is not spawnable, yet this test process was launched by cargo — \
         the generated code cannot be compile-checked. This is a broken toolchain, \
         not a skippable precondition (issues 1135, 1160)."
    );
}

/// Issue 1160 — `test_clippy_no_warnings` opened with
///
/// ```ignore
/// let clippy_check = Command::new("cargo").arg("clippy").arg("--version").output();
/// if clippy_check.is_err() { eprintln!("Skipping clippy test …"); return Ok(()); }
/// ```
///
/// which is wrong TWICE. `output()` is `Err` only when `cargo` itself cannot be
/// spawned; a host with cargo and no `clippy-driver` gets an `Ok` carrying a
/// FAILING status, so the probe answered "clippy is available" and the test then
/// ran `cargo clippy`, read its "no such command" stderr, found no `"error"`
/// substring and reported PASS. And on the host where it did fire it reported
/// PASS too — the 1135 shape, in a test NAMED for the lint verdict.
///
/// So: measure the exit status, and make it an assertion. `rustup component add
/// clippy` is a one-line remedy and this repo's whole `just check` line already
/// requires it; a green over a lint run that never happened is not a trade worth
/// making.
fn require_clippy() {
    let probe = Command::new("cargo")
        .arg("clippy")
        .arg("--version")
        .output();
    let available = probe.as_ref().is_ok_and(|o| o.status.success());
    assert!(
        available,
        "`cargo clippy` is not available — install it with `rustup component add \
         clippy`. This test is named for the lint verdict on generated code; \
         passing without one claims coverage nobody measured (issues 1135, 1160)."
    );
}

#[test]
fn test_simple_message_compiles() -> Result<(), GeneratorError> {
    require_cargo();

    let msg_def = "int32 x\nfloat64 y\nstring name\n";
    let msg = parse_message(msg_def).unwrap();

    let message_name = "SimpleMsg";
    let result = generate_message_package("test_msgs", message_name, &msg, &HashSet::new())?;

    // Create temp directory for test package
    let temp_dir = TempDir::new().unwrap();
    let pkg_dir = temp_dir.path().join("test_msgs");
    fs::create_dir_all(&pkg_dir).unwrap();

    // Write Cargo.toml
    let cargo_toml = create_test_cargo_toml("test_msgs", false);
    fs::write(pkg_dir.join("Cargo.toml"), cargo_toml).unwrap();

    // Write generated lib.rs
    let src_dir = pkg_dir.join("src");
    fs::create_dir_all(&src_dir).unwrap();

    let lib_rs = format!(
        r#"
{}

// Idiomatic layer at package root
pub mod msg {{
    // RMW (FFI) layer
    pub mod rmw {{
        use super::super::rosidl_runtime_rs;
        {}
    }}

    // Idiomatic types
    {}
}}
"#,
        create_rosidl_runtime_stub(),
        result.message_rmw,
        result.message_idiomatic
    );

    fs::write(src_dir.join("lib.rs"), lib_rs).unwrap();

    // Try to compile it
    let output = Command::new("cargo")
        .arg("check")
        .arg("--features")
        .arg("serde")
        .arg("--manifest-path")
        .arg(pkg_dir.join("Cargo.toml"))
        .output()
        .expect("Failed to run cargo check");

    if !output.status.success() {
        eprintln!("Compilation failed:");
        eprintln!("stdout: {}", String::from_utf8_lossy(&output.stdout));
        eprintln!("stderr: {}", String::from_utf8_lossy(&output.stderr));
        panic!("Generated code failed to compile");
    }

    Ok(())
}

#[test]
fn test_message_with_arrays_compiles() -> Result<(), GeneratorError> {
    require_cargo();

    // Use array sizes ≤ 32 to avoid needing big-array (serde limitation)
    let msg_def = "int32[5] small_array\nint32[32] large_array\n";
    let msg = parse_message(msg_def).unwrap();

    let message_name = "ArrayMsg";
    let result = generate_message_package("test_msgs", message_name, &msg, &HashSet::new())?;

    // Create temp directory for test package
    let temp_dir = TempDir::new().unwrap();
    let pkg_dir = temp_dir.path().join("test_msgs_arrays");
    fs::create_dir_all(&pkg_dir).unwrap();

    // Write Cargo.toml (no big-array needed for arrays ≤ 32)
    let cargo_toml = create_test_cargo_toml("test_msgs_arrays", false);
    fs::write(pkg_dir.join("Cargo.toml"), cargo_toml).unwrap();

    // Write generated lib.rs
    let src_dir = pkg_dir.join("src");
    fs::create_dir_all(&src_dir).unwrap();

    let lib_rs = format!(
        r#"
{}

// Idiomatic layer at package root
pub mod msg {{
    // RMW (FFI) layer
    pub mod rmw {{
        use super::super::rosidl_runtime_rs;
        {}
    }}

    // Idiomatic types
    {}
}}
"#,
        create_rosidl_runtime_stub(),
        result.message_rmw,
        result.message_idiomatic
    );

    fs::write(src_dir.join("lib.rs"), lib_rs).unwrap();

    // Try to compile it
    let output = Command::new("cargo")
        .arg("check")
        .arg("--features")
        .arg("serde")
        .arg("--manifest-path")
        .arg(pkg_dir.join("Cargo.toml"))
        .output()
        .expect("Failed to run cargo check");

    if !output.status.success() {
        eprintln!("Compilation failed:");
        eprintln!("stdout: {}", String::from_utf8_lossy(&output.stdout));
        eprintln!("stderr: {}", String::from_utf8_lossy(&output.stderr));
        panic!("Generated code with arrays failed to compile");
    }

    Ok(())
}

#[test]
fn test_check_no_warnings() -> Result<(), GeneratorError> {
    require_cargo();

    let msg_def = "int32 x\nfloat64 y\n";
    let msg = parse_message(msg_def).unwrap();

    let message_name = "Point";
    let result = generate_message_package("test_msgs", message_name, &msg, &HashSet::new())?;

    // Create temp directory
    let temp_dir = TempDir::new().unwrap();
    let pkg_dir = temp_dir.path().join("test_msgs_warnings");
    fs::create_dir_all(&pkg_dir).unwrap();

    // Write files
    let cargo_toml = create_test_cargo_toml("test_msgs_warnings", false);
    fs::write(pkg_dir.join("Cargo.toml"), cargo_toml).unwrap();

    let src_dir = pkg_dir.join("src");
    fs::create_dir_all(&src_dir).unwrap();

    let lib_rs = format!(
        r#"
#![deny(warnings)]

{}

// Idiomatic layer at package root
pub mod msg {{
    // RMW (FFI) layer
    pub mod rmw {{
        #[allow(unused_imports)]
        use super::super::rosidl_runtime_rs;
        {}
    }}

    // Idiomatic types
    {}
}}
"#,
        create_rosidl_runtime_stub(),
        result.message_rmw,
        result.message_idiomatic
    );

    fs::write(src_dir.join("lib.rs"), lib_rs).unwrap();

    // Compile with warnings as errors
    let output = Command::new("cargo")
        .arg("check")
        .arg("--features")
        .arg("serde")
        .arg("--manifest-path")
        .arg(pkg_dir.join("Cargo.toml"))
        .output()
        .expect("Failed to run cargo check");

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);

        // Check if it's actually a warning turned error
        if stderr.contains("warning") {
            eprintln!("Generated code has warnings:");
            eprintln!("{}", stderr);
            panic!("Generated code produced warnings");
        }
    }

    Ok(())
}

#[test]
fn test_clippy_no_warnings() -> Result<(), GeneratorError> {
    require_clippy();

    let msg_def = "int32 value\nstring name\n";
    let msg = parse_message(msg_def).unwrap();

    let message_name = "TestMsg";
    let result = generate_message_package("test_msgs", message_name, &msg, &HashSet::new())?;

    // Create temp directory
    let temp_dir = TempDir::new().unwrap();
    let pkg_dir = temp_dir.path().join("test_msgs_clippy");
    fs::create_dir_all(&pkg_dir).unwrap();

    // Write files
    let cargo_toml = create_test_cargo_toml("test_msgs_clippy", false);
    fs::write(pkg_dir.join("Cargo.toml"), cargo_toml).unwrap();

    let src_dir = pkg_dir.join("src");
    fs::create_dir_all(&src_dir).unwrap();

    let lib_rs = format!(
        r#"
{}

// Idiomatic layer at package root
pub mod msg {{
    // RMW (FFI) layer
    pub mod rmw {{
        use super::super::rosidl_runtime_rs;
        {}
    }}

    // Idiomatic types
    {}
}}
"#,
        create_rosidl_runtime_stub(),
        result.message_rmw,
        result.message_idiomatic
    );

    fs::write(src_dir.join("lib.rs"), lib_rs).unwrap();

    // Run clippy
    let output = Command::new("cargo")
        .arg("clippy")
        .arg("--features")
        .arg("serde")
        .arg("--manifest-path")
        .arg(pkg_dir.join("Cargo.toml"))
        .arg("--")
        .arg("-W")
        .arg("clippy::all")
        .output()
        .expect("Failed to run cargo clippy");

    let stderr = String::from_utf8_lossy(&output.stderr);

    // Clippy might have suggestions, but let's check there are no serious issues
    if stderr.contains("error") {
        eprintln!("Clippy found errors:");
        eprintln!("{}", stderr);
        panic!("Generated code failed clippy checks");
    }

    Ok(())
}
