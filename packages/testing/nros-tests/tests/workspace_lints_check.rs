//! Phase 212.L — `nros check --workspace` lints.
//!
//! Three rules:
//!
//! * **L.4** — `[[component]].class` must be `<pkg>::<Type>`-shaped. The
//!   bringup `system.toml` carries `pkg` + `class`; the class string MUST
//!   start with `<pkg>::` so codegen and humans land in the same crate.
//! * **L.8** — `system.toml` is a bringup-only file. A component pkg (has
//!   `Cargo.toml` or `CMakeLists.txt` at root) with a stray `system.toml`
//!   next to it is rejected.
//! * **L.11** — per-pkg `.cargo/config.toml` with `[patch.crates-io]` is
//!   warn-only. Cargo reads patches from BOTH `Cargo.toml` AND
//!   `.cargo/config.toml`, and the config-file shadows the manifest.

use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
};

fn nros_bin() -> Option<PathBuf> {
    nros_tests::nros_cli_bin_path()
}

/// Issue 1135 — was `-> Option<()>`, forcing every caller to spell
/// `if require_nros_cli().is_none() { skip!(..) }`. An `Option<()>` carries no
/// value, only a verdict the caller may drop on the floor; the guard keeps it.
fn require_nros_cli() {
    if !nros_tests::require_nros_cli() {
        nros_tests::skip!("nros CLI not on PATH — run `just setup-cli` + `source ./activate.sh`");
    }
}

fn write_bringup(parent: &Path, name: &str, components: &[(&str, &str, &str)]) -> PathBuf {
    let dir = parent.join(name);
    fs::create_dir_all(dir.join("launch")).unwrap();
    let mut system = String::from("[system]\nname = \"demo\"\nrmw = \"zenoh\"\ndomain_id = 0\n");
    let mut exec_depends = String::new();
    for (pkg, class, cname) in components {
        system.push_str(&format!(
            "\n[[component]]\npkg = \"{pkg}\"\nclass = \"{class}\"\nname = \"{cname}\"\n"
        ));
        exec_depends.push_str(&format!("  <exec_depend>{pkg}</exec_depend>\n"));
    }
    fs::write(dir.join("system.toml"), system).unwrap();
    fs::write(
        dir.join("package.xml"),
        format!(
            "<?xml version=\"1.0\"?>\n<package format=\"3\">\n  \
             <name>{name}</name>\n  <version>0.1.0</version>\n  \
             <description>demo</description>\n  \
             <maintainer email=\"d@e\">d</maintainer>\n  \
             <license>MIT</license>\n{exec_depends}  \
             <export><build_type>ament_cmake</build_type></export>\n</package>\n"
        ),
    )
    .unwrap();
    fs::write(dir.join("launch/system.launch.xml"), "<launch></launch>\n").unwrap();
    dir
}

fn write_component_pkg(parent: &Path, name: &str) -> PathBuf {
    let dir = parent.join(name);
    fs::create_dir_all(dir.join("src")).unwrap();
    fs::write(
        dir.join("Cargo.toml"),
        format!("[package]\nname = \"{name}\"\nversion = \"0.1.0\"\nedition = \"2021\"\n"),
    )
    .unwrap();
    fs::write(dir.join("src/lib.rs"), "// stub\n").unwrap();
    dir
}

fn run_check_workspace(root: &Path) -> (bool, String, String) {
    let out = Command::new(nros_bin().expect("nros CLI"))
        .args(["check", "--workspace", "."])
        .current_dir(root)
        .output()
        .expect("spawn nros check --workspace");
    (
        out.status.success(),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

// ---------------------------------------------------------------------------
// L.4 — class prefix
// ---------------------------------------------------------------------------

#[test]
fn nros_check_rejects_unqualified_class() {
    // RFC-0057 D2 retired the pkg-prefix rule (any upstream namespace is
    // fine); the live L.4 lint rejects a class that is not namespace-
    // qualified at all.
    require_nros_cli();
    let tmp = tempfile::tempdir().unwrap();
    write_bringup(
        tmp.path(),
        "demo_bringup",
        &[("talker_pkg", "Talker", "talker")],
    );

    let (ok, _stdout, stderr) = run_check_workspace(tmp.path());
    assert!(
        !ok,
        "expected failure for an unqualified class; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("namespace-qualified"),
        "stderr should mention namespace-qualified, got:\n{stderr}"
    );
    assert!(
        stderr.contains("class=\"Talker\""),
        "stderr should name the offending component class, got:\n{stderr}"
    );
}

#[test]
fn nros_check_accepts_correct_class_pkg_prefix() {
    require_nros_cli();
    let tmp = tempfile::tempdir().unwrap();
    write_bringup(
        tmp.path(),
        "demo_bringup",
        &[
            ("talker_pkg", "talker_pkg::Talker", "talker"),
            ("listener_pkg", "listener_pkg::Listener", "listener"),
        ],
    );

    let (ok, _stdout, stderr) = run_check_workspace(tmp.path());
    assert!(ok, "expected pass, got stderr:\n{stderr}");
}

// ---------------------------------------------------------------------------
// L.8 — stray system.toml
// ---------------------------------------------------------------------------

#[test]
fn nros_check_rejects_system_toml_in_component_pkg() {
    require_nros_cli();
    let tmp = tempfile::tempdir().unwrap();
    let pkg = write_component_pkg(tmp.path(), "talker_pkg");
    fs::write(
        pkg.join("system.toml"),
        "[system]\nname=\"x\"\nrmw=\"zenoh\"\ndomain_id=0\n",
    )
    .unwrap();

    let (ok, _stdout, stderr) = run_check_workspace(tmp.path());
    assert!(
        !ok,
        "expected failure for stray system.toml; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("stray system.toml"),
        "stderr should mention `stray system.toml`, got:\n{stderr}"
    );
    assert!(
        stderr.contains("talker_pkg"),
        "stderr should name the offending pkg, got:\n{stderr}"
    );
}

#[test]
fn nros_check_accepts_system_toml_in_bringup_pkg() {
    require_nros_cli();
    let tmp = tempfile::tempdir().unwrap();
    // Bringup-shape pkg: system.toml + package.xml only, no Cargo.toml,
    // no CMakeLists.txt, no src/. L.8 must NOT trip.
    write_bringup(tmp.path(), "demo_bringup", &[]);

    let (ok, _stdout, stderr) = run_check_workspace(tmp.path());
    assert!(ok, "bringup pkg must pass; stderr:\n{stderr}");
}

// ---------------------------------------------------------------------------
// L.11 — per-pkg .cargo/config.toml [patch.crates-io] is warn-only
// ---------------------------------------------------------------------------

#[test]
fn nros_check_warns_on_per_pkg_cargo_config_patch() {
    require_nros_cli();
    let tmp = tempfile::tempdir().unwrap();
    let pkg = write_component_pkg(tmp.path(), "talker_pkg");
    fs::create_dir_all(pkg.join(".cargo")).unwrap();
    fs::write(
        pkg.join(".cargo/config.toml"),
        "[patch.crates-io]\nzenoh = { git = \"https://example.com/x.git\" }\n",
    )
    .unwrap();

    let (ok, _stdout, stderr) = run_check_workspace(tmp.path());
    assert!(
        ok,
        "L.11 is warn-only; nros check must exit 0. stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("warning") && stderr.contains("[patch.crates-io]"),
        "stderr should carry warning about [patch.crates-io], got:\n{stderr}"
    );
    assert!(
        stderr.contains("talker_pkg"),
        "stderr should name offending pkg, got:\n{stderr}"
    );
}
