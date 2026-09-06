// Utilities for comparing generated code with reference outputs from rosidl_generator_rs
//
// Issue 0693 — this module is `mod`-included by BOTH `comparison_test` and
// `parity_test`, and each binary compiles the whole file. A helper used by only
// one of them is dead code in the other, which `-D warnings` rejects; the
// alternative is duplicating the ROS-discovery logic per binary, which is the
// drift this file exists to prevent.
#![allow(dead_code)]
use similar::{ChangeTag, TextDiff};

/// Normalize whitespace in code for comparison
/// - Trims leading/trailing whitespace from each line
/// - Removes empty lines
/// - Normalizes to single newlines between content
pub fn normalize_whitespace(code: &str) -> String {
    code.lines()
        .map(|line| line.trim_end())
        .collect::<Vec<_>>()
        .join("\n")
}

/// Strip single-line comments from code
/// Removes lines starting with // or ///
pub fn strip_comments(code: &str) -> String {
    code.lines()
        .filter(|line| {
            let trimmed = line.trim_start();
            !trimmed.starts_with("//") && !trimmed.starts_with("///")
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// Normalize package paths for comparison
/// Converts `package::msg::Type` to `crate::msg::Type` for local types
/// This handles differences in how rosidl_generator_rs and our codegen refer to types
pub fn normalize_paths(code: &str, package_name: &str) -> String {
    code.replace(&format!("{}::", package_name), "crate::")
        .replace(&format!("::{}", package_name), "::crate")
}

/// Normalize use statements
/// Sorts use statements and removes duplicates for easier comparison
pub fn normalize_use_statements(code: &str) -> String {
    let mut lines = Vec::new();
    let mut use_statements = Vec::new();
    let mut in_use_block = false;

    for line in code.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("use ") {
            use_statements.push(line.to_string());
            in_use_block = true;
        } else if in_use_block && trimmed.is_empty() {
            // End of use block - sort and add
            use_statements.sort();
            use_statements.dedup();
            lines.append(&mut use_statements);
            lines.push(line.to_string());
            in_use_block = false;
        } else {
            if in_use_block && !trimmed.is_empty() {
                // Flush use statements before non-use content
                use_statements.sort();
                use_statements.dedup();
                lines.append(&mut use_statements);
                in_use_block = false;
            }
            lines.push(line.to_string());
        }
    }

    // Flush any remaining use statements
    if !use_statements.is_empty() {
        use_statements.sort();
        use_statements.dedup();
        lines.extend(use_statements);
    }

    lines.join("\n")
}

/// Apply all normalization steps for comparison
pub fn normalize_code(code: &str, package_name: &str) -> String {
    let mut normalized = code.to_string();
    normalized = strip_comments(&normalized);
    normalized = normalize_paths(&normalized, package_name);
    normalized = normalize_use_statements(&normalized);
    normalized = normalize_whitespace(&normalized);
    normalized
}

/// Print a colored diff between two code strings
/// Returns true if codes are identical, false otherwise
pub fn print_diff(label_ours: &str, label_reference: &str, ours: &str, reference: &str) -> bool {
    if ours == reference {
        return true;
    }

    println!("\n❌ diff between {} and {}:", label_ours, label_reference);
    println!("{}", "=".repeat(80));

    let diff = TextDiff::from_lines(reference, ours);

    for change in diff.iter_all_changes() {
        let sign = match change.tag() {
            ChangeTag::Delete => "-",
            ChangeTag::Insert => "+",
            ChangeTag::Equal => " ",
        };
        print!("{}{}", sign, change);
    }

    println!("{}", "=".repeat(80));

    // Print statistics
    let stats = diff
        .ops()
        .iter()
        .fold((0, 0, 0), |(del, ins, eq), op| match op {
            similar::DiffOp::Delete { old_len, .. } => (del + old_len, ins, eq),
            similar::DiffOp::Insert { new_len, .. } => (del, ins + new_len, eq),
            similar::DiffOp::Equal { len, .. } => (del, ins, eq + len),
            _ => (del, ins, eq),
        });

    println!(
        "Lines: {} deleted, {} inserted, {} equal",
        stats.0, stats.1, stats.2
    );

    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_whitespace() {
        let code = "  fn foo()  \n\n  {  \n    bar();\n  }  ";
        let normalized = normalize_whitespace(code);
        assert_eq!(normalized, "  fn foo()\n\n  {\n    bar();\n  }");
    }

    #[test]
    fn test_strip_comments() {
        let code = r#"
// This is a comment
fn foo() {
    /// Doc comment
    bar(); // inline comment (kept)
}
"#;
        let stripped = strip_comments(code);
        assert!(!stripped.contains("// This is a comment"));
        assert!(!stripped.contains("/// Doc comment"));
        assert!(stripped.contains("bar(); // inline comment"));
    }

    #[test]
    fn test_normalize_paths() {
        let code = "std_msgs::msg::String";
        let normalized = normalize_paths(code, "std_msgs");
        assert_eq!(normalized, "crate::msg::String");
    }

    #[test]
    fn test_normalize_use_statements() {
        let code = r#"
use std::collections::HashMap;
use serde::{Deserialize, Serialize};
use std::string::String;

fn foo() {}
"#;
        let normalized = normalize_use_statements(code);
        let lines: Vec<&str> = normalized.lines().collect();

        // Use statements should be sorted
        assert!(lines[1].contains("serde"));
        assert!(lines[2].contains("std::collections"));
        assert!(lines[3].contains("std::string"));
    }

    #[test]
    fn test_print_diff_identical() {
        let code1 = "fn foo() {}";
        let code2 = "fn foo() {}";
        assert!(print_diff("ours", "reference", code1, code2));
    }

    #[test]
    fn test_print_diff_different() {
        let code1 = "fn foo() {}";
        let code2 = "fn bar() {}";
        assert!(!print_diff("ours", "reference", code1, code2));
    }
}

// ---------------------------------------------------------------------------
// ROS installation discovery — issue 0693
// ---------------------------------------------------------------------------

/// The `share/` root of the installed ROS 2 distro, or `None` when there is no
/// ROS on this host.
///
/// Issue 0693 — the comparison and parity suites read `/opt/ros/jazzy/...` as a
/// literal. This project installs **humble** (`DEFAULT_ROS_DISTRO` in
/// `nros-tests`), so every one of those reads failed, every test took its
/// `eprintln!("Skipping"); return Ok(())` arm, and the suites answered
/// "19 tests, 0.027 s, all green" while running nothing. The inputs were
/// present the whole time, one directory over.
///
/// Resolution order, most explicit first:
///   1. `$ROS_DISTRO` — set by `source /opt/ros/<distro>/setup.bash`, and what
///      the rest of the repo keys on.
///   2. the sole entry under `/opt/ros`, when there is exactly one. Ambiguity
///      is not resolved by guessing: with several installed and no
///      `$ROS_DISTRO`, this returns `None` rather than picking.
pub fn ros_share_root() -> Option<std::path::PathBuf> {
    if let Ok(distro) = std::env::var("ROS_DISTRO") {
        let p = std::path::PathBuf::from(format!("/opt/ros/{distro}/share"));
        if p.is_dir() {
            return Some(p);
        }
    }
    let entries: Vec<_> = std::fs::read_dir("/opt/ros")
        .ok()?
        .flatten()
        .filter(|e| e.path().join("share").is_dir())
        .collect();
    match entries.as_slice() {
        [only] => Some(only.path().join("share")),
        _ => None,
    }
}

/// `<share>/<package>/msg/<message>.msg` for the installed distro.
pub fn ros_msg_path(package: &str, message: &str) -> Option<std::path::PathBuf> {
    Some(
        ros_share_root()?
            .join(package)
            .join("msg")
            .join(format!("{message}.msg")),
    )
}

/// Print the one line a reader needs when a ROS-dependent test cannot run.
///
/// `cargo test` has no runtime skip and this crate lives in the `packages/cli`
/// sub-workspace, which `check-cli-tests` runs with a plain `cargo test` — no
/// junit, so no `nros_tests::skip!` rewrite is available here. Early-returning
/// is therefore the only option, and the mitigation is
/// `ros_discovery_is_not_silently_broken` below: whenever ROS IS installed,
/// that test FAILS if discovery returns `None`. So "no ROS" stays quiet and
/// "discovery regressed" does not.
///
/// Issue 1160 — be precise about what this line buys, because it is less than
/// it reads as. libtest CAPTURES the output of a PASSING test, and
/// `check-cli-tests` runs `cargo test … --quiet`, so on the one lane that
/// actually executes these suites nobody ever sees it. The marker is for a
/// human running the suite with `--nocapture`; it is NOT the safety net. The
/// safety net is that every condition which is not literally "this host has no
/// ROS 2 install" now FAILS instead of returning — see [`ros_input`] and
/// `comparison_test`'s parse arm.
pub fn note_no_ros(test: &str) {
    eprintln!(
        "[NO-ROS] {test}: no ROS 2 install found (set ROS_DISTRO or install one under /opt/ros) \
         — this test did not run. See issue 0693."
    );
}

/// Print the line for the OTHER absent-input state: ROS 2 IS installed, and the
/// package that supplies this input is not.
///
/// Issue 1160 — before this existed both states printed `[NO-ROS]`, so a host
/// carrying a perfectly good ROS install that merely lacks `sensor_msgs` was
/// told to go install ROS. Two states, one message, and the message was false
/// in one of them.
pub fn note_no_package(test: &str, package: &str, path: &std::path::Path) {
    eprintln!(
        "[NO-PKG] {test}: ROS 2 is installed but `{package}` is not — {} is absent, \
         so this test did not run. See issues 0693, 1160.",
        path.display()
    );
}

/// Resolve one ROS input file, or explain — accurately — why this host cannot
/// supply it.
///
/// Issue 1160. Every caller used to spell this as TWO guards with the SAME exit
/// and the SAME message, one of which was a lie:
///
/// ```ignore
/// let Some(p) = ros_file("geometry_msgs", "msg", "Pose.msg") else {
///     note_no_ros("parity_test");
///     return Ok(());          // no ROS at all — true
/// };
/// if !p.exists() {
///     note_no_ros("parity_test: Pose.msg not found");
///     return Ok(());          // ROS IS installed; "no ROS 2 install found" is false
/// }
/// ```
///
/// One helper, one verdict, one accurate line. It hands back a real value
/// (`Option<PathBuf>`), which is the spelling `check-test-precondition-guards`
/// blesses: with `let Some(x) = f() else { … }` the caller gets something it
/// needs rather than a bool it can drop with a bare `return`.
///
/// It PANICS in exactly one case, and that case is what the 1135 family is
/// about: the package directory IS installed and the file named inside it is
/// not. That is not an absent environment, it is a wrong expectation in this
/// test, and answering PASS for it claims coverage nobody measured.
pub fn ros_input(test: &str, package: &str, kind: &str, file: &str) -> Option<std::path::PathBuf> {
    ros_input_under(ros_share_root(), test, package, kind, Some(file))
}

/// Resolve one ROS input DIRECTORY (`<share>/<package>/<kind>`), or explain why
/// this host cannot supply it. Companion to [`ros_input`] for the tests that
/// walk a whole package rather than naming one file.
pub fn ros_input_dir(test: &str, package: &str, kind: &str) -> Option<std::path::PathBuf> {
    ros_input_under(ros_share_root(), test, package, kind, None)
}

/// The whole decision, as a function of the share root — so it can be tested
/// against a synthesized tree instead of against whatever ROS this host happens
/// to carry.
///
/// A guard whose three outcomes are only reachable on three different machines
/// is a guard nobody has ever seen fail. `guard_decides_by_state` below runs all
/// three here, on any host.
fn ros_input_under(
    share: Option<std::path::PathBuf>,
    test: &str,
    package: &str,
    kind: &str,
    file: Option<&str>,
) -> Option<std::path::PathBuf> {
    let Some(share) = share else {
        note_no_ros(test);
        return None;
    };
    let dir = share.join(package).join(kind);
    if !dir.is_dir() {
        note_no_package(test, package, &dir);
        return None;
    }
    let Some(file) = file else {
        return Some(dir);
    };
    let path = dir.join(file);
    assert!(
        path.is_file(),
        "{package}/{kind} is installed but {} is not in it — the input this test \
         names does not exist where ROS says it should. Reporting PASS here would \
         claim coverage nobody measured (issues 1135, 1160).",
        path.display()
    );
    Some(path)
}

#[cfg(test)]
mod input_guard_tests {
    use super::*;

    /// Issue 1160 — the three states this guard exists to tell apart, all three
    /// exercised on any host, with no ROS install involved.
    ///
    /// The middle one is the whole point: an installed package whose named file
    /// is absent must FAIL. Before this, both absent-input states returned and
    /// the test reported PASS, and the one that fires on a host WITH ROS printed
    /// `[NO-ROS]` while doing it.
    #[test]
    fn guard_decides_by_state() {
        let tmp = std::env::temp_dir().join(format!("nros-1160-{}", std::process::id()));
        let msg_dir = tmp.join("std_msgs").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join("Bool.msg"), "bool data\n").unwrap();

        // 1. no ROS at all -> None, and the caller returns quietly.
        assert!(ros_input_under(None, "t", "std_msgs", "msg", Some("Bool.msg")).is_none());

        // 2. ROS present, package absent -> None. Not an error: this host simply
        //    does not carry `sensor_msgs`.
        assert!(
            ros_input_under(
                Some(tmp.clone()),
                "t",
                "sensor_msgs",
                "msg",
                Some("Image.msg")
            )
            .is_none()
        );

        // 3. package installed, named file absent -> PANIC. The environment is
        //    fine and the expectation is wrong; a green here is issue 1135's lie.
        let missing = std::panic::catch_unwind(|| {
            ros_input_under(
                Some(tmp.clone()),
                "t",
                "std_msgs",
                "msg",
                Some("NotAThing.msg"),
            )
        });
        assert!(
            missing.is_err(),
            "an installed package missing the named input must FAIL, not return"
        );

        // 4. everything present -> the path the caller needs.
        let found = ros_input_under(Some(tmp.clone()), "t", "std_msgs", "msg", Some("Bool.msg"));
        assert_eq!(found, Some(msg_dir.join("Bool.msg")));

        // 5. the directory form, for the walk tests.
        assert_eq!(
            ros_input_under(Some(tmp.clone()), "t", "std_msgs", "msg", None),
            Some(msg_dir.clone())
        );

        std::fs::remove_dir_all(&tmp).ok();
    }
}

#[cfg(test)]
mod ros_discovery_tests {
    use super::*;

    /// Issue 0693 — the guard that makes the early return above safe.
    ///
    /// A suite that bails when ROS is absent is indistinguishable from a suite
    /// that bails because its path is wrong — which is exactly how a hardcoded
    /// `/opt/ros/jazzy` went unnoticed on a humble host. This test cannot tell
    /// you ROS is installed, but it CAN tell you that when it is, discovery
    /// finds it.
    #[test]
    fn ros_discovery_is_not_silently_broken() {
        let any_ros = std::fs::read_dir("/opt/ros")
            .map(|d| d.flatten().any(|e| e.path().join("share").is_dir()))
            .unwrap_or(false);
        if !any_ros {
            note_no_ros("ros_discovery_is_not_silently_broken");
            return;
        }
        let root = ros_share_root().expect(
            "an ROS install exists under /opt/ros but discovery returned None — \
             the parity/comparison suites are silently running nothing (issue 0693)",
        );
        assert!(
            root.join("std_msgs/msg/Bool.msg").is_file(),
            "discovered {} but it has no std_msgs/msg/Bool.msg — the suites that \
             read from here will bail and report PASS",
            root.display()
        );
    }
}
