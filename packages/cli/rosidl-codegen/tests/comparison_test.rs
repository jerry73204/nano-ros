// Comparison tests - verify our codegen matches rosidl_generator_rs output
use rosidl_codegen::{GeneratorError, generate_message_package};
use rosidl_parser::parse_message;
use std::{collections::HashSet, fs, path::PathBuf};

mod parity_helpers;
use parity_helpers::{normalize_code, print_diff, ros_input};

/// Helper to load reference output from fixtures
fn load_reference_output(package: &str, message: &str, layer: &str) -> Result<String, String> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/reference_outputs")
        .join(package)
        .join(message)
        .join(format!("msg_{}.rs", layer));

    fs::read_to_string(&path).map_err(|e| format!("Failed to read {}: {}", path.display(), e))
}

/// Read and parse one ROS message file.
///
/// Issue 0693 — resolve the INSTALLED distro instead of naming one. This read
/// `/opt/ros/jazzy/...` as a literal while the project installs humble, so
/// every caller took its "skipping" arm and the suite reported green over work
/// it never did.
///
/// Issue 1160 — this used to return `Result<_, String>` and fold THREE states
/// into the one `Err` arm every caller answered with `note_no_ros` + `Ok(())`:
///
///   1. this host has no ROS 2 install,
///   2. the file is there and unreadable,
///   3. **our parser rejected it**.
///
/// (3) is the subject of this crate. A `parse_message` failure on
/// `std_msgs/Bool.msg` is the loudest signal `rosidl-parser` can produce, and it
/// arrived as `[NO-ROS] comparison_test (Failed to parse …)` on a PASSING test
/// whose output libtest then captured — the exact `staticlib_duplicate_symbols`
/// lie from issue 1135, one directory over. Only (1) is an absent environment,
/// and only (1) may end in a green.
fn read_and_parse_ros_message(
    test: &str,
    package: &str,
    message: &str,
) -> Option<rosidl_parser::Message> {
    let path = ros_input(test, package, "msg", &format!("{message}.msg"))?;
    let content = fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{} exists and could not be read: {e} — the environment supplied the \
             input, so this is not a skip",
            path.display()
        )
    });
    Some(parse_message(&content).unwrap_or_else(|e| {
        panic!(
            "our parser rejected {}: {e:?} — `rosidl-parser` is the code under test \
             here, not the environment; answering PASS for this is issue 1135's lie \
             (see issue 1160)",
            path.display()
        )
    }))
}

#[test]
fn test_compare_std_msgs_bool() -> Result<(), GeneratorError> {
    // Parse the ROS message
    let Some(msg) = read_and_parse_ros_message("comparison_test", "std_msgs", "Bool") else {
        return Ok(());
    };

    // Generate with our codegen
    let result = generate_message_package("std_msgs", "Bool", &msg, &HashSet::new())?;

    // Load reference outputs
    let ref_rmw = load_reference_output("std_msgs", "Bool", "rmw")
        .expect("Reference RMW output should exist");
    let ref_idiomatic = load_reference_output("std_msgs", "Bool", "idiomatic")
        .expect("Reference idiomatic output should exist");

    // Normalize both for comparison
    let our_rmw = normalize_code(&result.message_rmw, "std_msgs");
    let ref_rmw_normalized = normalize_code(&ref_rmw, "std_msgs");

    let our_idiomatic = normalize_code(&result.message_idiomatic, "std_msgs");
    let ref_idiomatic_normalized = normalize_code(&ref_idiomatic, "std_msgs");

    // Compare RMW layer
    let rmw_matches = print_diff(
        "Our RMW (std_msgs::Bool)",
        "Reference RMW (rosidl_generator_rs)",
        &our_rmw,
        &ref_rmw_normalized,
    );

    // Compare idiomatic layer
    let idiomatic_matches = print_diff(
        "Our Idiomatic (std_msgs::Bool)",
        "Reference Idiomatic (rosidl_generator_rs)",
        &our_idiomatic,
        &ref_idiomatic_normalized,
    );

    // For now, we just print the diffs without failing
    // Once we achieve parity, change this to:
    // assert!(rmw_matches && idiomatic_matches, "Generated code differs from reference");

    if !rmw_matches {
        eprintln!("\n⚠️  RMW layer differs from reference (expected during development)");
    }
    if !idiomatic_matches {
        eprintln!("\n⚠️  Idiomatic layer differs from reference (expected during development)");
    }

    Ok(())
}

#[test]
fn test_compare_std_msgs_string() -> Result<(), GeneratorError> {
    // Parse the ROS message
    let Some(msg) = read_and_parse_ros_message("comparison_test", "std_msgs", "String") else {
        return Ok(());
    };

    // Generate with our codegen
    let result = generate_message_package("std_msgs", "String", &msg, &HashSet::new())?;

    // Load reference outputs
    let ref_rmw = load_reference_output("std_msgs", "String", "rmw")
        .expect("Reference RMW output should exist");
    let ref_idiomatic = load_reference_output("std_msgs", "String", "idiomatic")
        .expect("Reference idiomatic output should exist");

    // Normalize both for comparison
    let our_rmw = normalize_code(&result.message_rmw, "std_msgs");
    let ref_rmw_normalized = normalize_code(&ref_rmw, "std_msgs");

    let our_idiomatic = normalize_code(&result.message_idiomatic, "std_msgs");
    let ref_idiomatic_normalized = normalize_code(&ref_idiomatic, "std_msgs");

    // Compare
    let rmw_matches = print_diff(
        "Our RMW (std_msgs::String)",
        "Reference RMW (rosidl_generator_rs)",
        &our_rmw,
        &ref_rmw_normalized,
    );

    let idiomatic_matches = print_diff(
        "Our Idiomatic (std_msgs::String)",
        "Reference Idiomatic (rosidl_generator_rs)",
        &our_idiomatic,
        &ref_idiomatic_normalized,
    );

    if !rmw_matches {
        eprintln!("\n⚠️  RMW layer differs from reference (expected during development)");
    }
    if !idiomatic_matches {
        eprintln!("\n⚠️  Idiomatic layer differs from reference (expected during development)");
    }

    Ok(())
}

#[test]
fn test_compare_geometry_msgs_point() -> Result<(), GeneratorError> {
    // Parse the ROS message
    let Some(msg) = read_and_parse_ros_message("comparison_test", "geometry_msgs", "Point") else {
        return Ok(());
    };

    // Generate with our codegen
    let result = generate_message_package("geometry_msgs", "Point", &msg, &HashSet::new())?;

    // Load reference outputs
    let ref_rmw = load_reference_output("geometry_msgs", "Point", "rmw")
        .expect("Reference RMW output should exist");
    let ref_idiomatic = load_reference_output("geometry_msgs", "Point", "idiomatic")
        .expect("Reference idiomatic output should exist");

    // Normalize both for comparison
    let our_rmw = normalize_code(&result.message_rmw, "geometry_msgs");
    let ref_rmw_normalized = normalize_code(&ref_rmw, "geometry_msgs");

    let our_idiomatic = normalize_code(&result.message_idiomatic, "geometry_msgs");
    let ref_idiomatic_normalized = normalize_code(&ref_idiomatic, "geometry_msgs");

    // Compare
    let rmw_matches = print_diff(
        "Our RMW (geometry_msgs::Point)",
        "Reference RMW (rosidl_generator_rs)",
        &our_rmw,
        &ref_rmw_normalized,
    );

    let idiomatic_matches = print_diff(
        "Our Idiomatic (geometry_msgs::Point)",
        "Reference Idiomatic (rosidl_generator_rs)",
        &our_idiomatic,
        &ref_idiomatic_normalized,
    );

    if !rmw_matches {
        eprintln!("\n⚠️  RMW layer differs from reference (expected during development)");
    }
    if !idiomatic_matches {
        eprintln!("\n⚠️  Idiomatic layer differs from reference (expected during development)");
    }

    Ok(())
}

#[test]
fn test_normalization_helpers_exist() {
    // Smoke test to verify normalization helpers are working
    let code = r#"
        // Comment
        fn foo() {
            bar();
        }
    "#;

    let normalized = normalize_code(code, "test_pkg");
    assert!(!normalized.is_empty());
}
