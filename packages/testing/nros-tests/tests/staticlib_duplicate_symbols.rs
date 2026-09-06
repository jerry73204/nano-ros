//! Phase 241.D / RFC-0042 §D3 (issue #62, #70) — single-runtime link-determinism
//! validator.
//!
//! **History.** The pre-241.D3-rev model linked the C umbrella `libnros_c.a`
//! next to a STANDALONE RMW archive (`libnros_rmw_zenoh_staticlib.a`). Both were
//! self-contained Rust `crate-type=["staticlib"]` archives, so each bundled its
//! own copy of the shared dependency closure (nros-core, …) + the `nros_rmw_cffi`
//! C shim; the link reconciled the duplicates with `-Wl,--allow-multiple-definition`
//! — a blind mask that also hides real ODR violations. That old validator
//! diffed the duplicate set across the *pair* and asserted it was only shared-dep
//! bundling.
//!
//! **Now (241.D3-rev / phase-249 single-runtime).** The C umbrella bundles the
//! zenoh backend (rlib dep) into ONE archive (`cargo build -p nros-c --features
//! platform-posix,rmw-zenoh`). There is no second archive, so "duplicate symbols
//! across a pair" is moot — and the D3 goal it was a precondition for is now the
//! direct assertion: the single `libnros_c.a` links a host binary with
//! **`-u nros_rmw_zenoh_register`** and **NO `--allow-multiple-definition`**, with
//!   * the forced backend register entry actually included (`-u` did its job), and
//!   * exactly ONE cffi `REGISTRY` instance (a split registry is the #48
//!     `NoBackend` hazard).
//!
//! This is the host (posix) proxy for the cross C++ staticlib link; the dependency
//! closure is target-agnostic, so it is faithful + always reproducible.
//!
//! ## phase-329 W5 — link out of test
//!
//! The `-u`-forced link itself moved to the BUILD stage
//! (`scripts/build/link-determinism-fixture.sh` now links `bare.c` + the umbrella
//! archive into `build/link-determinism/lkproof` with `-u nros_rmw_zenoh_register`
//! and NO `--allow-multiple-definition`). The link SUCCEEDING is the build-stage
//! assertion — a strong-symbol collision or a missing forced entry aborts the
//! fixture build under `set -e`, failing the hard PR gate. This test now only runs
//! `nm` on the PREBUILT `lkproof` (pure inspection, no compilation at test time).
//! Skips when the fixture or `nm` is absent (CLAUDE.md: skip on unmet
//! preconditions, never silent-pass).

use std::{path::PathBuf, process::Command};

/// Resolve an available `nm` (prefer `llvm-nm`, fall back to GNU `nm`). The
/// symbol checks below match C symbols by exact name, so either tool works.
fn nm_tool() -> Option<String> {
    for nm in ["llvm-nm", "nm"] {
        if Command::new(nm).arg("--version").output().is_ok() {
            return Some(nm.to_string());
        }
    }
    None
}

/// The prebuilt single-runtime link proof (`build/link-determinism/lkproof`,
/// linked at the build stage with `-u` force + NO `--allow-multiple-definition`),
/// gated on the `.compile-ok` stamp.
fn link_proof_exe(root: &std::path::Path) -> Option<PathBuf> {
    let fx = root.join("build/link-determinism");
    let exe = fx.join("lkproof");
    (fx.join(".compile-ok").is_file() && exe.is_file()).then_some(exe)
}

/// Phase 241.D3-rev (issue #62 / #70) — the single-runtime link proof.
///
/// `--allow-multiple-definition` was needed only because the pre-D3 link pulled a
/// standalone RMW backend archive in with broad `--whole-archive` (to force its
/// register/ctor symbols), dragging in every member — including the shared
/// closure's strong defs, which then collided with the umbrella's copies. The
/// single-runtime fix bundles the backend INTO `libnros_c.a` and forces only the
/// backend register entry via `-u <symbol>`; there is one archive, one `std`, one
/// `REGISTRY`, so the host binary links with NO `--allow-multiple-definition`.
#[test]
fn single_archive_links_via_u_force_without_allow_multiple_definition() {
    let root = nros_tests::project_root();
    let Some(exe) = link_proof_exe(&root) else {
        nros_tests::skip!(
            "no single-runtime link proof (build/link-determinism/lkproof) — run \
             `scripts/build/link-determinism-fixture.sh` first (it links with `-u` force \
             + NO --allow-multiple-definition at the build stage)"
        );
    };

    // The link itself (`-u nros_rmw_zenoh_register`, NO `--allow-multiple-definition`)
    // succeeded at the BUILD stage — its failure aborts the fixture build under
    // `set -e`, so the presence of `lkproof` here IS the link-success proof. This
    // test asserts the resulting symbol shape.

    // Symbol checks need an `nm`. Issue 1135 — this used to `eprintln!` and
    // `return`, i.e. report PASS. The link half is already proven above, but
    // the two assertions BELOW are the substance this test is named for (the
    // register entry got forced in; there is exactly one `REGISTRY`), and a
    // green with neither of them run claims coverage nobody measured. On any
    // host with binutils the skip never fires.
    let Some(nm) = nm_tool() else {
        nros_tests::skip_class!(
            capability,
            "no `nm` on PATH — the link itself succeeded at the build stage (lkproof exists), \
             but the register/REGISTRY symbol assertions cannot run"
        );
    };

    let syms = Command::new(&nm).arg(&exe).output().unwrap();
    let listing = String::from_utf8_lossy(&syms.stdout);
    assert!(
        listing
            .lines()
            .any(|l| l.ends_with(" T nros_rmw_zenoh_register")
                || l.ends_with(" t nros_rmw_zenoh_register")),
        "`-u nros_rmw_zenoh_register` did not pull the backend register entry into the image \
         — forcing the entry is the whole point of the `-u` replacement for `--whole-archive`",
    );
    let registry_defs = listing
        .lines()
        .filter(|l| {
            l.ends_with(" T REGISTRY")
                || l.ends_with(" D REGISTRY")
                || l.ends_with(" B REGISTRY")
                || l.ends_with(" t REGISTRY")
                || l.ends_with(" d REGISTRY")
                || l.ends_with(" b REGISTRY")
        })
        .count();
    assert_eq!(
        registry_defs, 1,
        "expected exactly ONE cffi `REGISTRY` instance in the linked image (single shared \
         registry), found {registry_defs} — a split registry is the #48 `NoBackend` hazard",
    );

    eprintln!(
        "D3 single-runtime: libnros_c.a (zenoh bundled) links with `-u nros_rmw_zenoh_register` \
         and NO `--allow-multiple-definition` — register entry included, exactly one REGISTRY. \
         The two-archive `--allow-multiple-definition` mask is retired."
    );
}
