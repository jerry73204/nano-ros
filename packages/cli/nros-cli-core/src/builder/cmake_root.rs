//! Stage 4b — emit the cmake workspace root (phase-383 W4, RFC-0065 D3).
//!
//! ## What this replaces
//!
//! `examples/workspaces/c/CMakeLists.txt` — ~70 lines doing four jobs, none of
//! them user intent:
//!
//! 1. map the board to a `CMAKE_TOOLCHAIN_FILE`, **before `project()`**,
//!    because that is the first compiler probe;
//! 2. list the packages by hand;
//! 3. filter which entries belong to the active platform, by hand;
//! 4. promote `NUTTX_DIR` out of a cmake directory scope.
//!
//! All four are derivable. There are nine such roots in-tree and each can drift
//! independently; a workspace that gains a package but forgets the `SUBDIRS`
//! line simply does not build it, silently, because an absent subdir is not an
//! error.
//!
//! ## Unlike the cargo root, this DOES live under `build/`
//!
//! Cargo pins its workspace manifest to the workspace root (a package belongs
//! to one workspace, found by walking up; members must sit below the root — see
//! [`super::cargo_root`]). CMake has neither rule: `add_subdirectory` takes an
//! arbitrary source dir, so the generated root sits at
//! `build/<coord>/CMakeLists.txt` where RFC-0065 D8 wants it, and switching
//! board or RMW selects a different coordinate instead of thrashing one tree.
//!
//! One consequence worth stating: `add_subdirectory(<src> <bin>)` needs the
//! second argument when the source is outside the tree, and every subdir here
//! is. That is why the emitted calls carry a binary dir.
//!
//! ## What it does NOT derive
//!
//! **The preamble** (W4.c). `autoware-safety-island`'s root calls
//! `find_package(Eigen3 REQUIRED)`, which nothing in the tree implies. An
//! optional `<bringup>/cmake/preamble.cmake` is included before `project()` if
//! present.
//!
//! **Whether a package builds.** ASI adds `src/s32z2_board_glue` only when the
//! NXP SDK is provisioned — "the pkg's own CMakeLists gates and reports". The
//! emitter lists it; the package decides. A package that excludes itself is
//! normal, not an error (W4.d).

use std::{
    collections::BTreeSet,
    path::{Path, PathBuf},
};

use super::discover::Discovered;

/// The `nano_ros_add_executable(...)` call a generated root emits (W4.b).
///
/// A C/C++ entry package is that ONE call: `SOURCES` is optional when `LAUNCH`
/// is present, because the verb generates the TU that carries `main`. So unlike
/// the Rust side there is no package to write — no `CMakeLists.txt`, no
/// `main.c`, no `package.xml`, which is exactly what W4.b says ("deletes the
/// package and emits its one call").
///
/// `BOARD` and `DEPLOY` come from the resolved image rather than being written
/// as literals, which is what closes issue 0798: a generated entry cannot
/// disagree with the board it was generated for.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CmakeEntry {
    /// Target name — `<image>_entry`, the same derivation the Rust side uses.
    pub name: String,
    /// `LAUNCH` — a launch file name, or `default` for the bringup's own.
    pub launch: String,
    /// `LAUNCH_ARGS k=v` — how an image selects a machine.
    pub args: Vec<(String, String)>,
    /// `LANG` — `c` or `cpp`, from the workspace's own packages.
    pub lang: String,
    /// `DEPLOY` — the board token the macro resolves against.
    pub deploy: String,
    /// `PANIC` — RFC-0077 policy, when the image declares one.
    pub panic: Option<String>,
}

/// Everything the emitted root needs that is not in [`Discovered`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CmakeRootSpec {
    /// Workspace root, so subdir paths can be made relative to the manifest.
    pub workspace: PathBuf,
    /// Bringup package name — `nano_ros_workspace(SYSTEM …)`.
    pub system: String,
    /// nano-ros platform token (`posix`, `freertos`, `nuttx`, …).
    pub platform: String,
    /// nano-ros board id, or `None` for a host build that names none.
    pub board: Option<String>,
    /// RMW backend.
    pub rmw: String,
    /// Repo-relative toolchain file from the board descriptor's `[board.cmake]`,
    /// or `None` for a host board that needs none.
    pub toolchain_file: Option<String>,
    /// nano-ros checkout, for resolving the toolchain file.
    pub nano_ros_root: PathBuf,
    /// Package dirs to omit — west/idf entries, and entries for other boards.
    pub excluded: BTreeSet<PathBuf>,
    /// The entries this root must EMIT (W4.b).
    ///
    /// EVERY image that lands on this coordinate, not just the one being built:
    /// they share `build/<coord>/`, so a root carrying one image's entry is
    /// rewritten by the next image's build and the workspace never has more
    /// than one executable declared at a time. Same defect the cargo root had
    /// (its member list changed per image, which made `Cargo.lock` churn), and
    /// the same answer — the root is a property of the WORKSPACE.
    ///
    /// An image whose hand-written package still exists contributes nothing: it
    /// is a discovered SUBDIR and a second target of that name would collide.
    /// That is what makes D13's migration incremental.
    pub entries: Vec<CmakeEntry>,
}

/// Render the root `CMakeLists.txt` written to `manifest_dir`.
pub fn render(
    discovered: &Discovered,
    manifest_dir: &Path,
    spec: &CmakeRootSpec,
) -> Result<String, String> {
    // A cmake subdir must carry a CMakeLists. A pure-Rust package in a mixed
    // workspace does not, and reaches the image through corrosion from a
    // package that does — listing it here would be a configure error.
    let mut subdirs: Vec<(String, String)> = Vec::new();
    for pkg in &discovered.packages {
        if spec.excluded.contains(&pkg.dir) || !pkg.dir.join("CMakeLists.txt").is_file() {
            continue;
        }
        // An INTERFACE package has a CMakeLists and still must not be a subdir
        // (issue 0862). Its CMakeLists is deliberately verbatim upstream ROS —
        // `find_package(ament_cmake REQUIRED)` on line one — because it must
        // also build under `colcon`. nano-ros never configures it: `nros sync`
        // routes `rosidl_generate_interfaces` through the codegen pipeline and
        // emits the `generated/<pkg>` crate the node packages depend on.
        //
        // The hand-written roots knew this and left these packages out of
        // `_ws_subdirs` by hand, with a comment saying why; the derivation had
        // only "has a CMakeLists" and swept them back in. On a host WITH ROS
        // that configures and looks fine, which is why it landed — it fails
        // only where tier 1 is contracted to run, on a host with no ROS.
        if crate::interface_package::dir_is_interface_package(&pkg.dir) {
            continue;
        }
        // Relative to the WORKSPACE, not to this file.
        //
        // `nano_ros_workspace` hands SUBDIRS to `nros ws order --workspace
        // <root> --subdir <s>`, which resolves each against the workspace and
        // walks it for `package.xml`. A hand-written root sits AT the workspace
        // root, so the two bases coincide and nothing distinguished them; this
        // root sits in `build/<coord>/`, and manifest-relative paths made the
        // ordering tool look inside the build directory ("no package.xml under
        // .../build/posix-zenoh"). Workspace-relative is also what a reader
        // wants to see: `src/talker_pkg`, not `../../src/talker_pkg`.
        let rel = super::paths::relative_or_err(&spec.workspace, &pkg.dir)?;
        subdirs.push((rel, pkg.name.clone()));
    }
    if subdirs.is_empty() {
        return Err(
            "no cmake packages in this workspace — nothing for a cmake root to \
             add_subdirectory. A pure-Rust workspace builds through the cargo \
             root instead (phase-383 W3)."
                .to_string(),
        );
    }
    // Sorted for byte-identical output (W3.c). ORDER_FROM_DEPENDS re-derives
    // the BUILD order from each package's `<depend>` tags, so the order written
    // here carries no meaning and must not churn.
    subdirs.sort();

    // ONE translation, used everywhere the platform is emitted: the `set()`
    // and the `nano_ros_workspace(PLATFORM …)` argument both feed
    // `nros_feature_set`, and fixing only one leaves the other failing with the
    // same message from a different line — which is what happened.
    let cmake_platform = spec.platform.replace('-', "_");

    let mut out = String::new();
    out.push_str(
        "# GENERATED by `nros build` (phase-383 W4) — DO NOT EDIT.\n\
         #\n\
         # Regenerated on every build. Edit the workspace, not this file.\n\
         #\n\
         # Paths are RELATIVE and the subdir list is SORTED, so this file is\n\
         # byte-identical across machines (phase-383 W3.c). Build ORDER comes\n\
         # from ORDER_FROM_DEPENDS, not from the order written here.\n\n",
    );
    out.push_str("cmake_minimum_required(VERSION 3.22)\n\n");

    // The toolchain file must precede project(): that call runs the first
    // compiler probe, and a toolchain set afterwards is a toolchain nobody used.
    if let Some(tc) = &spec.toolchain_file {
        let abs = spec.nano_ros_root.join(tc);
        let rel = super::paths::relative(manifest_dir, &abs)
            .ok_or_else(|| format!("cannot express toolchain file {} relatively", abs.display()))?;
        out.push_str(&format!(
            "# Before project(): that call is the first compiler probe, so a\n\
             # toolchain file set after it is one nobody used.\n\
             if(NOT CMAKE_TOOLCHAIN_FILE)\n    \
             set(CMAKE_TOOLCHAIN_FILE \"${{CMAKE_CURRENT_LIST_DIR}}/{rel}\")\nendif()\n\n"
        ));
    }

    // W4.c — the user preamble, if the bringup ships one.
    out.push_str(
        "# W4.c — optional user preamble (`<bringup>/cmake/preamble.cmake`).\n\
         # For what the builder cannot derive: `find_package(Eigen3 REQUIRED)`\n\
         # in autoware-safety-island's root is the motivating case.\n\
         if(DEFINED NROS_WS_PREAMBLE AND EXISTS \"${NROS_WS_PREAMBLE}\")\n    \
         include(\"${NROS_WS_PREAMBLE}\")\nendif()\n\n",
    );

    out.push_str(&format!(
        "project({}_nros_workspace LANGUAGES C CXX)\n\n",
        sanitize(&spec.system)
    ));
    // BOARD and PLATFORM before `find_package(nano_ros)`, not after.
    //
    // A hand-written root receives them on the command line, so they are set
    // before anything runs; the package config and the netstack resolution read
    // them. Emitting them AFTER the find_package meant nros-cpp was configured
    // without knowing the board: the `freertos-posix` image built zenoh-pico's
    // freertos/lwip backend instead of its POSIX one, and the failure surfaced
    // only at link, as a wall of `undefined reference to lwip_*` from an
    // archive that had no business containing lwIP code at all.
    // cmake's platform vocabulary is snake_case; the board catalog's is kebab.
    //
    // `nros_feature_set` accepts `threadx_linux` / `threadx_riscv64` /
    // `nuttx_armv7a`; `PlatformKind::kebab()` produces `threadx-linux`. Emitting
    // the kebab form fails the configure outright —
    //
    //   nros_feature_set: unknown PLATFORM 'threadx-linux'
    //
    // — which is the good outcome: the pre-migration fixture row passed a THIRD
    // spelling (`threadx`, the generic one), and a silently accepted wrong
    // platform picks a different feature set. Every token in that function's
    // accepted list is the kebab form with `-` replaced, so this is a
    // translation between two vocabularies rather than a guess.
    out.push_str(&format!(
        "set(NANO_ROS_PLATFORM {cmake_platform} CACHE STRING \"\" FORCE)\n"
    ));
    if let Some(board) = &spec.board {
        out.push_str(&format!(
            "set(NANO_ROS_BOARD {board} CACHE STRING \"\" FORCE)\n"
        ));
    }
    out.push('\n');

    // W4.a — promote the NuttX trees out of the environment.
    //
    // The hand-written roots did this by hand, and the module doc has always
    // said this emitter would; it did not. `nros-nuttx.cmake` reads the CACHE
    // variables, and an `$ENV{}` value does not reach a directory scope that
    // never sets it.
    if spec.platform == "nuttx" {
        out.push_str(
            "# W4.a — the promotion the hand-written roots did by hand.\n\
             if(DEFINED ENV{NUTTX_DIR} AND NOT NUTTX_DIR)\n\
             \x20   set(NUTTX_DIR \"$ENV{NUTTX_DIR}\" CACHE PATH \"NuttX kernel tree\")\n\
             endif()\n\
             if(DEFINED ENV{NUTTX_APPS_DIR} AND NOT NUTTX_APPS_DIR)\n\
             \x20   set(NUTTX_APPS_DIR \"$ENV{NUTTX_APPS_DIR}\" CACHE PATH \"NuttX apps tree\")\n\
             endif()\n\n",
        );
    }

    // Issue 1111 — where a C++ package finds a WORKSPACE-LOCAL interface
    // package.
    //
    // An interface package is deliberately NOT a subdir (see the loop above,
    // issue 0862), so a component's `find_package(my_msgs REQUIRED)` resolves
    // through the compat layer's Find-stub, which scans
    // `NROS_INTERFACE_SEARCH_PATH`. The hand-written roots set it — the
    // documented shape in `NanoRosGenerateInterfaces.cmake` is
    // `set(NROS_INTERFACE_SEARCH_PATH "${CMAKE_SOURCE_DIR}/src")` — and this
    // emitter did not, so a workspace whose C++ node consumes its own msg
    // package configured fine with `nros ws env` sourced and failed with a raw
    // cmake "Could not find a package configuration file provided by
    // <pkg>" without it. The builder knows the workspace root (it writes
    // WORKSPACE_ROOT below), so it can answer the question rather than leaving
    // it to the caller's environment — which is also the rule the tree already
    // follows for knobs: a value that arrives through a dependency edge cannot
    // be poisoned by an ambient variable.
    //
    // MUST precede `find_package(nano_ros)`: the compat layer reads this while
    // it is being included, and the ordering is the hand-written roots' own.
    //
    // Prepends rather than assigns, so a caller who HAS sourced
    // `nros ws env` (or set it for an out-of-workspace package tree) keeps
    // their entry and its precedence — earlier roots win the shadowing order.
    let src_rel = super::paths::relative_or_err(manifest_dir, &spec.workspace.join("src"))?;
    out.push_str(&format!(
        "# Issue 1111 — the workspace's own interface packages. Prepended, so an\n\
         # entry the caller already exported keeps its precedence.\n\
         get_filename_component(_nros_ws_src \"{src_rel}\" ABSOLUTE)\n\
         if(EXISTS \"${{_nros_ws_src}}\")\n\
         \x20   if(DEFINED ENV{{NROS_INTERFACE_SEARCH_PATH}})\n\
         \x20       set(NROS_INTERFACE_SEARCH_PATH\n\
         \x20           \"${{_nros_ws_src}};$ENV{{NROS_INTERFACE_SEARCH_PATH}}\")\n\
         \x20   else()\n\
         \x20       set(NROS_INTERFACE_SEARCH_PATH \"${{_nros_ws_src}}\")\n\
         \x20   endif()\n\
         endif()\n\n"
    ));

    out.push_str("find_package(nano_ros REQUIRED COMPONENTS workspace)\n");
    out.push('\n');

    // Relative like every other path here, so the file stays byte-identical
    // across machines (W3.c).
    let ws_rel = super::paths::relative_or_err(manifest_dir, &spec.workspace)?;
    out.push_str("nano_ros_workspace(\n");
    // WHERE THE WORKSPACE IS, which is not where this file is.
    //
    // A hand-written root lives at the workspace root, so `nano_ros_workspace`
    // could read `CMAKE_SOURCE_DIR` for both. This one sits in `build/<coord>/`
    // (RFC-0065 D3/D8), and without this the bringup lookup searches the build
    // directory: "no bringup pkg named 'demo_bringup' in .../build/posix-zenoh".
    out.push_str(&format!("    WORKSPACE_ROOT \"{ws_rel}\"\n"));
    out.push_str(&format!("    BACKEND  {}\n", spec.rmw));
    out.push_str(&format!("    PLATFORM \"{}\"\n", cmake_platform));
    out.push_str(&format!("    SYSTEM   {}\n", spec.system));
    out.push_str("    ORDER_FROM_DEPENDS\n");
    out.push_str("    SUBDIRS\n");
    for (rel, name) in &subdirs {
        out.push_str(&format!("        \"{rel}\"   # {name}\n"));
    }
    out.push_str(")\n");

    for e in &spec.entries {
        let bringup_rel = super::paths::relative_or_err(
            manifest_dir,
            &spec.workspace.join("src").join(&spec.system),
        )?;
        out.push_str("\n# GENERATED entry (phase-383 W4.b) — the package this replaces was\n");
        out.push_str("# three files, and only this call carried information. `SOURCES` is\n");
        out.push_str("# omitted deliberately: it is optional when LAUNCH is present, and the\n");
        out.push_str("# verb generates the TU that carries `main`.\n");
        out.push_str("#\n");
        out.push_str("# BOARD and DEPLOY come from the resolved image, so this entry cannot\n");
        out.push_str("# disagree with the board it was generated for (issue 0798).\n");
        out.push_str(&format!("nano_ros_add_executable({}\n", e.name));
        if let Some(b) = &spec.board {
            out.push_str(&format!("    BOARD   {b}\n"));
        }
        out.push_str(&format!(
            "    BRINGUP \"${{CMAKE_CURRENT_SOURCE_DIR}}/{bringup_rel}\"\n"
        ));
        out.push_str(&format!("    LAUNCH  {}\n", e.launch));
        for (k, v) in &e.args {
            out.push_str(&format!("    LAUNCH_ARGS {k}={v}\n"));
        }
        out.push_str(&format!("    LANG    {}\n", e.lang));
        if let Some(p) = &e.panic {
            out.push_str(&format!("    PANIC   {p}\n"));
        }
        // Every one of the 57 C/C++ entries in this tree is TYPED — it routes
        // each launch node to the real executor through its component object.
        // An untyped generated entry would be a different program.
        out.push_str("    TYPED\n");
        out.push_str(&format!("    DEPLOY  {})\n", e.deploy));
    }

    Ok(out)
}

/// A cmake `project()` name: letters, digits and underscores only.
fn sanitize(s: &str) -> String {
    s.chars()
        .map(|c| if c.is_alphanumeric() { c } else { '_' })
        .collect()
}

/// Write the root, creating `manifest_dir` if needed.
pub fn write(
    discovered: &Discovered,
    manifest_dir: &Path,
    spec: &CmakeRootSpec,
) -> Result<PathBuf, String> {
    let body = render(discovered, manifest_dir, spec)?;
    std::fs::create_dir_all(manifest_dir)
        .map_err(|e| format!("creating {}: {e}", manifest_dir.display()))?;
    let path = manifest_dir.join("CMakeLists.txt");
    // Rewrite only on change: cmake re-configures when its root is newer than
    // the cache, and a gratuitous touch costs a full reconfigure.
    if std::fs::read_to_string(&path).ok().as_deref() != Some(body.as_str()) {
        std::fs::write(&path, &body).map_err(|e| format!("writing {}: {e}", path.display()))?;
    }
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cargo_nano_ros::provider_scan::WorkspacePackage;

    fn pkg(root: &Path, name: &str, cmake: bool) -> WorkspacePackage {
        let dir = root.join("src").join(name);
        std::fs::create_dir_all(&dir).unwrap();
        if cmake {
            std::fs::write(dir.join("CMakeLists.txt"), "# pkg\n").unwrap();
        }
        WorkspacePackage {
            name: name.to_string(),
            dir,
            depends: Default::default(),
        }
    }

    fn discovered(packages: Vec<WorkspacePackage>) -> Discovered {
        Discovered {
            packages,
            cargo_only: Default::default(),
            warnings: Vec::new(),
        }
    }

    fn spec(root: &Path) -> CmakeRootSpec {
        CmakeRootSpec {
            workspace: root.to_path_buf(),
            system: "demo_bringup".to_string(),
            platform: "posix".to_string(),
            board: None,
            rmw: "zenoh".to_string(),
            toolchain_file: None,
            nano_ros_root: PathBuf::from("/nros"),
            excluded: Default::default(),
            entries: Vec::new(),
        }
    }

    #[test]
    fn subdirs_are_relative_and_sorted() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let d = discovered(vec![pkg(root, "zzz_pkg", true), pkg(root, "aaa_pkg", true)]);
        let body = render(&d, &root.join("build/posix"), &spec(root)).expect("renders");
        // Workspace-relative, not manifest-relative — see the note at the
        // computation. A hand-written root cannot tell the difference; this one
        // can, and got it wrong.
        assert!(body.contains("\"src/aaa_pkg\""), "{body}");
        assert!(
            !body.contains("\"../../src/aaa_pkg\""),
            "subdirs must not be relative to the generated file: {body}"
        );
        assert!(
            !body.contains(root.to_str().unwrap()),
            "no absolute path (W3.c): {body}"
        );
        let a = body.find("aaa_pkg").unwrap();
        let z = body.find("zzz_pkg").unwrap();
        assert!(a < z, "sorted: {body}");
    }

    #[test]
    fn the_interface_search_path_precedes_find_package_nano_ros() {
        // Issue 1111. An interface package is not a subdir, so a component's
        // `find_package(my_msgs)` resolves through the compat layer's
        // Find-stub, which reads `NROS_INTERFACE_SEARCH_PATH`. The layer reads
        // it while it is being INCLUDED, so a value set after
        // `find_package(nano_ros)` is a value nobody used — the same shape as
        // the toolchain-after-project() bug below.
        //
        // Compares LINE positions among non-comment lines: the explanatory
        // comment above the block names `find_package(nano_ros)`, and a naive
        // substring search matches that first.
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let d = discovered(vec![pkg(root, "a_pkg", true)]);
        let body = render(&d, &root.join("build/posix"), &spec(root)).expect("renders");
        let code: Vec<&str> = body
            .lines()
            .filter(|l| !l.trim_start().starts_with('#'))
            .collect();
        let set = code
            .iter()
            .position(|l| l.contains("set(NROS_INTERFACE_SEARCH_PATH"))
            .unwrap_or_else(|| panic!("the root must set the search path: {body}"));
        let fp = code
            .iter()
            .position(|l| l.contains("find_package(nano_ros"))
            .unwrap_or_else(|| panic!("no find_package(nano_ros): {body}"));
        assert!(
            set < fp,
            "NROS_INTERFACE_SEARCH_PATH must precede find_package(nano_ros): {body}"
        );
        // Workspace-relative, never absolute (W3.c) — the file must stay
        // byte-identical across machines.
        assert!(
            !body.contains(root.to_str().unwrap()),
            "no absolute path: {body}"
        );
        // An entry the caller already exported keeps its precedence.
        assert!(
            body.contains("$ENV{NROS_INTERFACE_SEARCH_PATH}"),
            "must prepend to, not clobber, a caller's value: {body}"
        );
    }

    #[test]
    fn the_toolchain_file_precedes_project() {
        // project() runs the first compiler probe, so a toolchain set after it
        // is a toolchain nobody used — the bug every hand-written root guards
        // against with a comment.
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let d = discovered(vec![pkg(root, "a_pkg", true)]);
        let mut s = spec(root);
        s.toolchain_file = Some("cmake/toolchain/arm-freertos-armcm3.cmake".to_string());
        s.board = Some("mps2-an385-freertos".to_string());
        s.platform = "freertos".to_string();
        let body = render(&d, &root.join("build/freertos"), &s).expect("renders");
        // Compare LINE positions among non-comment lines: the explanatory
        // comment above the toolchain block legitimately contains the text
        // "project(", and a naive substring search matches that first.
        let code: Vec<&str> = body
            .lines()
            .filter(|l| !l.trim_start().starts_with('#'))
            .collect();
        let tc = code
            .iter()
            .position(|l| l.contains("CMAKE_TOOLCHAIN_FILE"))
            .expect("toolchain emitted");
        let proj = code
            .iter()
            .position(|l| l.starts_with("project("))
            .expect("project emitted");
        assert!(tc < proj, "toolchain must precede project(): {body}");
    }

    #[test]
    fn a_host_board_emits_no_toolchain_file() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let d = discovered(vec![pkg(root, "a_pkg", true)]);
        let body = render(&d, &root.join("build/posix"), &spec(root)).expect("renders");
        assert!(!body.contains("CMAKE_TOOLCHAIN_FILE"), "{body}");
    }

    #[test]
    fn a_rust_only_package_is_not_a_subdir() {
        // It has no CMakeLists; listing it would be a configure error. It
        // reaches the image through corrosion from a package that does.
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let d = discovered(vec![pkg(root, "c_pkg", true), pkg(root, "rust_pkg", false)]);
        let body = render(&d, &root.join("build/posix"), &spec(root)).expect("renders");
        assert!(body.contains("c_pkg"), "{body}");
        assert!(!body.contains("rust_pkg"), "{body}");
    }

    #[test]
    fn a_generated_root_emits_the_entry_call_when_the_package_is_gone() {
        // W4.b. A C/C++ entry package is ONE `nano_ros_add_executable` call —
        // `SOURCES` is optional when `LAUNCH` is present because the verb
        // generates the TU carrying `main` — so there is no package to write.
        //
        // This wave was marked complete while the emitter never emitted it. The
        // existing test passed anyway, because it asserted the root MENTIONS
        // `native_entry`, which it did as a discovered SUBDIR. Deleting the
        // hand-written package produced a workspace that built its node
        // libraries and no executable at all.
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let d = discovered(vec![pkg(root, "talker_pkg", true)]);
        let mut sp = spec(root);
        sp.board = Some("native".to_string());
        sp.entries = vec![CmakeEntry {
            name: "native_robot1_entry".to_string(),
            launch: "multihost.launch.xml".to_string(),
            args: vec![("host".to_string(), "robot1".to_string())],
            lang: "c".to_string(),
            deploy: "native".to_string(),
            panic: None,
        }];
        let body = render(&d, &root.join("build/posix"), &sp).expect("renders");

        assert!(
            body.contains("nano_ros_add_executable(native_robot1_entry"),
            "{body}"
        );
        assert!(body.contains("LAUNCH  multihost.launch.xml"), "{body}");
        assert!(
            body.contains("LAUNCH_ARGS host=robot1"),
            "how an image selects a machine: {body}"
        );
        assert!(body.contains("BOARD   native"), "{body}");
        assert!(body.contains("DEPLOY  native"), "{body}");
        assert!(
            body.contains("TYPED"),
            "all 57 entries in this tree are TYPED: {body}"
        );
        // Non-comment lines only. The emitted text EXPLAINS that SOURCES is
        // omitted, so a bare `contains` matches the explanation and passes for
        // the wrong reason — the trap this phase hit twice already.
        let code: String = body
            .lines()
            .filter(|l| !l.trim_start().starts_with('#'))
            .collect::<Vec<_>>()
            .join("\n");
        assert!(
            !code.contains("SOURCES"),
            "SOURCES is optional under LAUNCH and the verb writes the main: {code}"
        );
        assert!(
            !body.contains(root.to_str().unwrap()),
            "no absolute path (W3.c): {body}"
        );
    }

    #[test]
    fn two_images_differing_only_in_args_render_differently() {
        // Issue 1136. `native_robot1` and `native_robot2` share a board, a
        // launch file, a language and a deploy — `args` is the ONLY thing that
        // distinguishes them, and it is what picks the per-host `[[model]]`
        // variant. Stop emitting it and the two calls become the same call:
        // two binaries, one program, no error anywhere. The resolver half is
        // `model_location::per_host_images_resolve_to_distinct_models`.
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let d = discovered(vec![pkg(root, "talker_pkg", true)]);
        let mut sp = spec(root);
        sp.board = Some("native".to_string());
        sp.entries = ["robot1", "robot2"]
            .iter()
            .map(|host| CmakeEntry {
                name: format!("native_{host}_entry"),
                launch: "multihost.launch.xml".to_string(),
                args: vec![("host".to_string(), (*host).to_string())],
                lang: "c".to_string(),
                deploy: "native".to_string(),
                panic: None,
            })
            .collect();
        let body = render(&d, &root.join("build/posix"), &sp).expect("renders");

        let calls: Vec<&str> = body.split("nano_ros_add_executable(").skip(1).collect();
        assert_eq!(calls.len(), 2, "{body}");
        // Compare with the target NAME removed: that difference is free, and
        // the question is whether anything the BUILD reads differs too.
        let strip = |c: &str| {
            c.replacen("native_robot1_entry", "", 1)
                .replacen("native_robot2_entry", "", 1)
        };
        assert_ne!(
            strip(calls[0]),
            strip(calls[1]),
            "the two per-host images render identically apart from their target \
             name — they resolve one model and are one program: {body}"
        );
    }

    #[test]
    fn no_entry_is_emitted_while_the_package_still_exists() {
        // The property that makes D13's migration incremental: two targets of
        // one name would collide, so the emitter stays silent until the
        // hand-written package is deleted.
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let d = discovered(vec![pkg(root, "native_entry", true)]);
        let body = render(&d, &root.join("build/posix"), &spec(root)).expect("renders");
        assert!(
            !body.contains("nano_ros_add_executable"),
            "a hand-written entry is a SUBDIR; do not emit a second one: {body}"
        );
    }

    #[test]
    fn an_excluded_entry_is_omitted() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let zephyr = pkg(root, "zephyr_entry", true);
        let d = discovered(vec![pkg(root, "native_entry", true), zephyr.clone()]);
        let mut s = spec(root);
        s.excluded = [zephyr.dir.clone()].into_iter().collect();
        let body = render(&d, &root.join("build/posix"), &s).expect("renders");
        assert!(body.contains("native_entry"), "{body}");
        assert!(!body.contains("zephyr_entry"), "{body}");
    }

    #[test]
    fn the_root_orders_from_depends_not_from_the_written_order() {
        // The SET is chosen here; the ORDER is derived. phase-348 W4 made that
        // possible, and it means this file's ordering carries no meaning.
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let d = discovered(vec![pkg(root, "a_pkg", true)]);
        let body = render(&d, &root.join("build/posix"), &spec(root)).expect("renders");
        assert!(body.contains("ORDER_FROM_DEPENDS"), "{body}");
    }

    #[test]
    fn a_workspace_with_no_cmake_packages_points_at_the_cargo_root() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let d = discovered(vec![pkg(root, "rust_pkg", false)]);
        let e = render(&d, &root.join("build/posix"), &spec(root)).expect_err("nothing to add");
        assert!(e.contains("W3"), "{e}");
    }

    #[test]
    fn output_is_byte_identical_across_runs() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let d = discovered(vec![pkg(root, "a_pkg", true), pkg(root, "b_pkg", true)]);
        let dir = root.join("build/posix");
        assert_eq!(
            render(&d, &dir, &spec(root)).unwrap(),
            render(&d, &dir, &spec(root)).unwrap()
        );
    }

    #[test]
    fn writing_twice_does_not_touch_the_file() {
        // cmake reconfigures when its root outdates the cache; a gratuitous
        // touch costs a full reconfigure.
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let d = discovered(vec![pkg(root, "a_pkg", true)]);
        let dir = root.join("build/posix");
        let p = write(&d, &dir, &spec(root)).expect("first");
        let m1 = std::fs::metadata(&p).unwrap().modified().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(20));
        write(&d, &dir, &spec(root)).expect("second");
        let m2 = std::fs::metadata(&p).unwrap().modified().unwrap();
        assert_eq!(m1, m2);
    }
}
