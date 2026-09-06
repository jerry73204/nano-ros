//! Phase 212.H.8 — LoC budget gates.
//!
//! Phase 212 §Acceptance freezes two hard LoC budgets (the third —
//! `nros-build` `src/` ≤ 550 LoC — was retired with Phase 212.C):
//!
//!   * each RTOS adapter shim ≤ 200 LoC
//!   * cmake `nano_ros_workspace_metadata()` ≤ 150 LoC
//!
//! Each gate measures the `code` line count under a path — comments +
//! blanks do not count toward the budget.
//!
//! ## Why the `tokei` crate, not the `tokei` CLI
//!
//! The earlier revision of this test shelled out to a `tokei` binary
//! on `$PATH`. That binary isn't installed on a stock dev machine
//! (and isn't in our toolchain-tier provisioner), so every CI / dev
//! run hit the `nros_tests::skip!` branch and the gate effectively
//! never ran. Phase 212.H.8 activation pulls `tokei` in as a Rust
//! dev-dependency (`tokei = { version = "14", default-features = false }`
//! in `Cargo.toml` — `default-features = false` drops the CLI's
//! clap/colored/env_logger tree, leaving only the counter library).
//! All counting is now in-process and deterministic; no external
//! install, no skip path.
//!
//! Counts match the CLI's `Total.code` exactly because we drive the
//! same `tokei::Languages::get_statistics()` entry point the CLI does.
//!
//! Skip discipline: there is no skip path. Every failure mode is a
//! hard `assert!` — silent early-return is forbidden.
//!
//! ## Adapter shim path mapping (verified 2026-06-01)
//!
//! | RTOS       | Shim path                                              |
//! |------------|--------------------------------------------------------|
//! | Zephyr     | `zephyr/cmake/nros_system_generate.cmake`              |
//! | NuttX      | `integrations/nuttx/` (dir sum — Makefile/CMake/glue)  |
//! | ThreadX    | SKIPPED — phase-432 W2.6 removed the per-RTOS entry     |
//! |            | template; the entry TU comes from the shared pack, so   |
//! |            | ThreadX ships no adapter shim of its own (see `SHIMS`). |
//! | ESP-IDF    | `integrations/nano-ros/CMakeLists.txt` (esp-idf component) |
//! | PlatformIO | `integrations/platformio/nros_codegen.py` (extra_script) |
//! | PX4        | `integrations/px4/module-template/` (dir sum)          |
//! | FreeRTOS   | SKIPPED — H.3 makes the cargo path itself the adapter; |
//! |            | no separate shim file ships in-tree (per-board BSP     |
//! |            | crate's `build.rs` carries the integration).           |
//!
//! When you re-home one of these, update the table AND the `SHIMS`
//! const below — the test reads the table for its assertion loop.

use std::path::{Path, PathBuf};

use nros_tests::project_root;
use tokei::{Config, Languages};

const BUDGET_WORKSPACE_METADATA: u64 = 150;
const BUDGET_ADAPTER_SHIM: u64 = 200;

/// Adapter shim entries: `(label, repo-relative path)`. A path may name
/// a single file (Zephyr / ThreadX / ESP-IDF / PlatformIO) or a
/// directory (NuttX / PX4) — in the directory case we sum the `code`
/// counts across all languages tokei recognises inside it.
const SHIMS: &[(&str, &str)] = &[
    ("zephyr", "zephyr/cmake/nros_system_generate.cmake"),
    ("nuttx", "integrations/nuttx"),
    // ThreadX has no row, and the absence is the measurement.
    //
    // Phase 246 pointed this at `cmake/templates/threadx_entry_main_typed.cpp.in`
    // — the TYPED-carrier entry template. Issue 1003's fix (8fa26022e) merged
    // the six per-family entry templates into one, deleting that exact path,
    // and this row was not updated. `tokei_code_loc` hard-asserts
    // `path.exists()`, so this test has been RED since that commit; nothing
    // noticed, because no merge-gating lane runs it.
    //
    // phase-432 W2.6 then removed the merged template too:
    // `nano_ros_node_register()` renders the entry TU through the SHARED pack
    // (`nros codegen entry-node`), so there is no per-RTOS entry shim left to
    // measure for ANY family — which is the outcome this budget existed to
    // push toward, reached rather than evaded.
    //
    // The row is not repointed at `cmake/platform/nano-ros-threadx.cmake`
    // (391 LoC): that is a PLATFORM module, not an adapter shim, and the nuttx
    // row's analog is `integrations/nuttx`, an integration directory ThreadX
    // does not have. Repointing would swap a missing-path red for a
    // budget-violation red while measuring a different thing.
    ("esp-idf", "integrations/nano-ros/CMakeLists.txt"),
    ("platformio", "integrations/platformio/nros_codegen.py"),
    ("px4", "integrations/px4/module-template"),
];

/// Returns the `code` LoC count under `path`, summed across every
/// language tokei recognises. `path` may be a single file or a
/// directory; tokei recurses on dirs.
///
/// Counts comments + blanks-excluded `code` lines only — the same
/// figure the `tokei` CLI prints in the `Code` column / `Total.code`
/// JSON field. We drive the library entry-point directly so there's
/// no process spawn and no need for an external `tokei` binary on
/// `$PATH`.
fn tokei_code_loc(path: &Path) -> u64 {
    assert!(
        path.exists(),
        "tokei target missing — adapter shim moved? update the SHIMS table in \
         phase212_h8_loc_budgets.rs. Missing path: {}",
        path.display()
    );

    let config = Config::default();
    let mut languages = Languages::new();
    // `get_statistics` accepts paths as `AsRef<Path>` and ignored
    // patterns as `AsRef<str>` — pass empty ignore list, the library
    // already skips hidden/vcs dirs by default.
    let path_arg: [&Path; 1] = [path];
    let ignored: [&str; 0] = [];
    languages.get_statistics(&path_arg, &ignored, &config);

    let mut total: u64 = 0;
    for (_lang_type, lang) in &languages {
        total += lang.code as u64;
    }
    total
}

#[test]
fn cmake_workspace_metadata_under_150_loc() {
    let file = project_root().join("cmake/nano_ros_workspace_metadata.cmake");
    let code = tokei_code_loc(&file);
    assert!(
        code <= BUDGET_WORKSPACE_METADATA,
        "Phase 212 acceptance budget violated: \
         cmake/nano_ros_workspace_metadata.cmake = {code} LoC \
         > {BUDGET_WORKSPACE_METADATA}."
    );
    eprintln!("[OK] nano_ros_workspace_metadata.cmake = {code} / {BUDGET_WORKSPACE_METADATA} LoC");
}

#[test]
fn rtos_adapter_shims_under_200_loc_each() {
    let root = project_root();
    let mut violations: Vec<String> = Vec::new();
    let mut reports: Vec<(String, u64)> = Vec::new();

    for (label, rel) in SHIMS {
        let abs: PathBuf = root.join(rel);
        // Sum across every language tokei recognises — adapter shim
        // dirs (nuttx, px4) mix CMake + Makefile + Kconfig + C++ +
        // Python, and the 200-LoC budget covers them as one unit.
        let code = tokei_code_loc(&abs);
        reports.push((label.to_string(), code));
        if code > BUDGET_ADAPTER_SHIM {
            violations.push(format!(
                "  - {label} ({rel}): {code} LoC > {BUDGET_ADAPTER_SHIM}"
            ));
        }
    }

    for (label, code) in &reports {
        eprintln!("[OK] adapter shim {label} = {code} / {BUDGET_ADAPTER_SHIM} LoC");
    }

    assert!(
        violations.is_empty(),
        "Phase 212.H.8 adapter-shim 200-LoC budget violated:\n{}",
        violations.join("\n")
    );
}
