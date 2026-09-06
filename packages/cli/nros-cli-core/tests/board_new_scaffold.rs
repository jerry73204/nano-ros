//! A scaffolded board resolves, and its acceptance is the gates, not the files.
//!
//! phase-375 W2/W9. The wave exists because `s32z270` landed RED on five gates
//! and `freertos-posix` on two — each gate correct, the cost being that they
//! were discovered serially, on main, by whoever noticed. So the assertions
//! below are about what the gates and the catalog think of the output, not
//! about which files appeared.

use std::process::Command;

fn nros() -> PathBuf {
    // The test binary sits in `target/<profile>/deps/`; the CLI is two up.
    let mut p = std::env::current_exe().expect("test binary path");
    p.pop();
    p.pop();
    p.push("nros");
    assert!(p.is_file(), "build the CLI first: {}", p.display());
    p
}

use std::path::PathBuf;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(3)
        .expect("repo root")
        .to_path_buf()
}

fn scaffold(tmp: &std::path::Path, args: &[&str]) -> String {
    let out = Command::new(nros())
        .arg("board")
        .arg("new")
        .args(args)
        .arg("--out-of-tree")
        .arg(tmp)
        .arg("--workspace")
        .arg(repo_root())
        .output()
        .expect("run nros board new");
    assert!(
        out.status.success(),
        "scaffold failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8_lossy(&out.stdout).into_owned()
}

/// The whole claim of RFC-0064 R5 D1, end to end: a user's board is our board's
/// shape, and the only difference is which root found it.
#[test]
fn a_scaffolded_out_of_tree_board_resolves_by_name() {
    let tmp = tempfile::tempdir().expect("tempdir");
    scaffold(tmp.path(), &["vendorboard", "--platform", "freertos"]);

    let out = Command::new(nros())
        .args(["board", "info", "vendorboard", "--workspace"])
        .arg(repo_root())
        .env("NROS_EXTRA_BOARD_PATH", tmp.path())
        .output()
        .expect("run nros board info");
    assert!(
        out.status.success(),
        "a scaffolded board must resolve with no in-tree edit beyond the search \
         root: {}",
        String::from_utf8_lossy(&out.stderr)
    );
}

/// The announcement is not optional, so the scaffold must write it (RFC-0064
/// R5 D5). Without this the catalog refuses the board — which is the behaviour
/// we want, and exactly why the scaffold has to get it right.
#[test]
fn the_scaffold_writes_the_announcement_the_catalog_requires() {
    let tmp = tempfile::tempdir().expect("tempdir");
    scaffold(tmp.path(), &["announcedboard", "--platform", "freertos"]);
    let dir = tmp.path().join("nros-board-announcedboard");

    let xml = std::fs::read_to_string(dir.join("package.xml")).expect("package.xml written");
    assert!(
        xml.contains(r#"<nano_ros_provides kind="board" name="announcedboard"/>"#),
        "the announcement must name the board: {xml}"
    );

    // And the negative: remove it, and the board stops resolving. The scaffold
    // is only load-bearing if its absence is fatal.
    std::fs::remove_file(dir.join("package.xml")).unwrap();
    let out = Command::new(nros())
        .args(["board", "info", "announcedboard", "--workspace"])
        .arg(repo_root())
        .env("NROS_EXTRA_BOARD_PATH", tmp.path())
        .output()
        .expect("run nros board info");
    assert!(
        !out.status.success(),
        "a board with no package.xml must NOT resolve — if it does, the \
         announcement is decoration"
    );
}

/// A Zephyr board's derived filenames are the ones the projection looks for.
///
/// The scaffold PRINTS those paths, and the projection DERIVES them. If the two
/// rules ever disagree, a board author creates `boards/<x>.conf` at the printed
/// path and the build silently never reads it.
#[test]
fn the_paths_the_scaffold_prints_are_the_paths_the_projection_derives() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let stdout = scaffold(
        tmp.path(),
        &[
            "zephyrboard",
            "--platform",
            "zephyr",
            "--west-board",
            "plank_r5/plank/smp",
        ],
    );
    let dir = tmp.path().join("nros-board-zephyrboard");
    let conf = dir.join("boards").join("plank_r5_plank_smp.conf");
    assert!(
        stdout.contains(&conf.display().to_string()),
        "the scaffold must print the derived conf path; got:\n{stdout}"
    );

    // Create it at the printed path, and the projection must pick it up.
    std::fs::create_dir_all(conf.parent().unwrap()).unwrap();
    std::fs::write(&conf, "# board conf\n").unwrap();
    let out_file = tmp.path().join("vars.cmake");
    let out = Command::new(nros())
        .args(["board", "cmake-vars", "zephyrboard", "--workspace"])
        .arg(repo_root())
        .arg("--out")
        .arg(&out_file)
        .env("NROS_EXTRA_BOARD_PATH", tmp.path())
        .output()
        .expect("run nros board cmake-vars");
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let vars = std::fs::read_to_string(&out_file).unwrap();
    assert!(
        vars.contains(&format!(
            "set(NROS_BOARD_BOARD_CONF \"{}\")",
            conf.display()
        )),
        "a conf created at the printed path must reach cmake; got:\n{vars}"
    );
}

/// A key that is legal as a directory but not as a cargo feature or a cmake
/// value fails HERE, not three tools later.
#[test]
fn an_illegal_board_key_is_refused_up_front() {
    let tmp = tempfile::tempdir().expect("tempdir");
    for bad in ["Vendor_Board", "vendor board", "vendor/board"] {
        let out = Command::new(nros())
            .args([
                "board",
                "new",
                bad,
                "--platform",
                "freertos",
                "--out-of-tree",
            ])
            .arg(tmp.path())
            .arg("--workspace")
            .arg(repo_root())
            .output()
            .expect("run nros board new");
        assert!(!out.status.success(), "`{bad}` must be refused");
    }
}

/// `--platform zephyr` without `--west-board` cannot produce a usable board:
/// the conf/overlay filenames and the Rust-support Kconfig symbol all derive
/// from it.
#[test]
fn a_zephyr_board_without_its_board_id_is_refused() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let out = Command::new(nros())
        .args([
            "board",
            "new",
            "noid",
            "--platform",
            "zephyr",
            "--out-of-tree",
        ])
        .arg(tmp.path())
        .arg("--workspace")
        .arg(repo_root())
        .output()
        .expect("run nros board new");
    assert!(!out.status.success());
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(
        err.contains("--west-board"),
        "the refusal must name the fix: {err}"
    );
}
