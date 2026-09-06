//! Phase 212.G — `nros check --bringup` cross-validates the bringup's
//! `package.xml` `<exec_depend>` rows against `[[component]].pkg` rows
//! in `system.toml`. Replaces the retired `nros emit package-xml` verb:
//! users hand-write `package.xml`; drift detection catches stale rows
//! after a component rename or add/remove.

use std::{fs, path::PathBuf, process::Command};

fn nros_bin() -> PathBuf {
    nros_tests::nros_cli_bin_path()
        .expect("nros CLI available (require_test_prereqs gate would skip earlier)")
}

/// Issue 1135 — was `-> Option<()>`, forcing every caller to spell
/// `if require_nros_cli().is_none() { skip!(..) }`. An `Option<()>` carries no
/// value, only a verdict the caller may drop on the floor; the guard keeps it.
fn require_nros_cli() {
    if !nros_tests::require_nros_cli() {
        nros_tests::skip!("nros CLI not on PATH — run `just setup-cli` + `source ./activate.sh`");
    }
}

/// Drop a minimal pure-declarative bringup pkg under `parent` with the
/// given `[[component]]` `pkg` rows + `<exec_depend>` rows. The caller
/// picks the two lists independently so we can deliberately create drift.
fn write_bringup(parent: &std::path::Path, components: &[&str], exec_deps: &[&str]) -> PathBuf {
    let dir = parent.join("demo_bringup");
    fs::create_dir_all(dir.join("launch")).unwrap();

    let mut system_toml =
        String::from("[system]\nname = \"demo\"\nrmw = \"zenoh\"\ndomain_id = 0\n\n");
    for pkg in components {
        system_toml.push_str(&format!(
            "[[component]]\npkg = \"{pkg}\"\nclass = \"{pkg}::node\"\nname = \"{pkg}\"\n\n",
        ));
    }
    fs::write(dir.join("system.toml"), system_toml).unwrap();

    let mut pkg_xml = String::from(
        r#"<?xml version="1.0"?>
<package format="3">
  <name>demo_bringup</name>
  <version>0.1.0</version>
  <description>demo bringup</description>
  <maintainer email="dev@example.com">dev</maintainer>
  <license>MIT</license>
"#,
    );
    for dep in exec_deps {
        pkg_xml.push_str(&format!("  <exec_depend>{dep}</exec_depend>\n"));
    }
    pkg_xml.push_str("  <export><build_type>ament_cmake</build_type></export>\n</package>\n");
    fs::write(dir.join("package.xml"), pkg_xml).unwrap();
    fs::write(dir.join("launch/system.launch.xml"), "<launch></launch>\n").unwrap();
    dir
}

fn run_check_bringup(dir: &std::path::Path) -> (bool, String) {
    let out = Command::new(nros_bin())
        .args(["check", "--bringup"])
        .arg(dir)
        .output()
        .expect("spawn nros check");
    (
        out.status.success(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

#[test]
fn check_passes_when_exec_depend_matches_components() {
    require_nros_cli();
    let tmp = tempfile::tempdir().unwrap();
    let dir = write_bringup(
        tmp.path(),
        &["talker_pkg", "listener_pkg"],
        &["talker_pkg", "listener_pkg"],
    );
    let (ok, stderr) = run_check_bringup(&dir);
    assert!(ok, "expected pass, got stderr:\n{stderr}");
}

#[test]
fn check_rejects_missing_exec_depend() {
    require_nros_cli();
    let tmp = tempfile::tempdir().unwrap();
    // system.toml declares two components, package.xml lists only one.
    let dir = write_bringup(tmp.path(), &["talker_pkg", "listener_pkg"], &["talker_pkg"]);
    let (ok, stderr) = run_check_bringup(&dir);
    assert!(
        !ok,
        "expected failure for missing <exec_depend>; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("listener_pkg") && stderr.to_lowercase().contains("missing"),
        "stderr should call out missing listener_pkg, got:\n{stderr}"
    );
}

#[test]
fn check_rejects_stray_exec_depend() {
    require_nros_cli();
    let tmp = tempfile::tempdir().unwrap();
    // package.xml lists a stray dep not in system.toml.
    let dir = write_bringup(tmp.path(), &["talker_pkg"], &["talker_pkg", "ghost_pkg"]);
    let (ok, stderr) = run_check_bringup(&dir);
    assert!(
        !ok,
        "expected failure for stray <exec_depend>; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("ghost_pkg") && stderr.to_lowercase().contains("stray"),
        "stderr should call out stray ghost_pkg, got:\n{stderr}"
    );
}
