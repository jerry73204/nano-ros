//! `nros build` — the workspace build entry point (RFC-0065, phase-383 W2.c).
//!
//! Five stages, and the last one replaces this process:
//!
//! ```text
//!   1. DISCOVER   package.xml ∪ cargo members → topological order
//!   2. RESOLVE    the image: argument > default_images > list and fail
//!   3. PREFLIGHT  toolchains / SDKs / sources present?
//!   4. GENERATE   msg bindings + model + the ROOT BUILD FILE  (W3/W4)
//!   5. EXEC       cargo / cmake / west / idf.py — stderr untouched
//! ```
//!
//! Stage 4 is not wired yet. That is deliberate and shippable: RFC-0065 D3 says
//! a root is emitted only where one would otherwise be hand-written, and west
//! and ESP-IDF apps ship their own, so those targets go 1→2→3→5 today and work.
//! A cargo/cmake image reports what stage 4 will do and stops, rather than
//! silently building the wrong thing.

use std::path::PathBuf;

use clap::Parser;
use eyre::{Result, WrapErr};

use crate::builder::{
    discover,
    handoff::Handoff,
    plan::{self, Driver},
};

#[derive(Parser, Debug)]
pub struct Args {
    /// Image(s) to build — `native`, or `<bringup>:native` when two bringups
    /// declare the same id. Empty uses `[system] default_images`.
    pub images: Vec<String>,

    /// Workspace root. Defaults to the current directory.
    #[arg(long)]
    pub workspace: Option<PathBuf>,

    /// nano-ros checkout holding `packages/boards`. Defaults to
    /// `NROS_REPO_DIR`, then an autodetect walk from the workspace.
    #[arg(long)]
    pub nano_ros_path: Option<PathBuf>,

    /// The west workspace holding Zephyr — the directory that contains
    /// `zephyr/`. Zephyr images only.
    ///
    /// The explicit rung of the resolution ladder, above `$ZEPHYR_BASE` and
    /// `$NROS_ZEPHYR_WORKSPACE`. An environment variable is ambient state that
    /// a user has to remember to set and cannot see in the command they ran;
    /// this makes the same fact reviewable in a script and in shell history.
    ///
    /// Pointing it at the `zephyr/` directory itself also works — that is the
    /// commonest way to get this wrong, and both spellings name one place.
    #[arg(long, value_name = "DIR")]
    pub zephyr_workspace: Option<PathBuf>,

    /// Build every declared image.
    #[arg(long)]
    pub all: bool,

    /// Print the stages and the command that would run, then stop.
    ///
    /// Safe by construction: a `Handoff` performs no I/O until `exec`.
    #[arg(long)]
    pub dry_run: bool,

    /// Do not fetch anything; fail naming what is missing (RFC-0065 D14).
    ///
    /// Note this is a SCOPED guarantee: stages 1–4 touch no network, and stage
    /// 5 gets the native tool's own offline spelling. It cannot promise an
    /// arbitrary user `CMakeLists.txt` refrains from fetching.
    #[arg(long)]
    pub offline: bool,

    /// Build only these packages — no dependencies, no dependents
    /// (RFC-0087 D7, phase-420 W7).
    ///
    /// colcon's flag, with colcon's meaning, and two deliberate divergences
    /// documented on `builder::discover::select`: a name matching no package is
    /// an ERROR here rather than a warning, and a selection that drops a
    /// package another selected package depends on is REFUSED rather than left
    /// to resolve against an install prefix nano-ros does not have.
    ///
    /// The selection narrows what the build CONTAINS — the generated cargo
    /// root's member list and the generated CMake root's subdirectories. It
    /// does not narrow which images exist: an image is declared by a bringup's
    /// `system.toml`, which is a property of the workspace, not of the
    /// selection, so `nros build native --packages-select talker_pkg` still
    /// means the `native` image, built from a narrowed workspace.
    #[arg(long, value_name = "PKG", num_args = 1..)]
    pub packages_select: Vec<String>,

    /// Build these packages and everything they depend on, transitively, and
    /// nothing else (RFC-0087 D7, phase-420 W7).
    ///
    /// Composes with `--packages-select` as an INTERSECTION, which is colcon's
    /// composition too: each flag deselects independently, so adding one can
    /// only ever narrow the build.
    #[arg(long, value_name = "PKG", num_args = 1..)]
    pub packages_up_to: Vec<String>,

    /// Arguments after `--` go to the native tool verbatim.
    #[arg(last = true)]
    pub native_args: Vec<String>,
}

/// One resolved build: what to say, and what to run.
#[derive(Debug, Clone)]
pub struct ResolvedBuild {
    /// `<bringup>:<image>`.
    pub qualified: String,
    /// nano-ros board id as authored.
    pub board: String,
    /// The board's platform token, resolved through the board catalog.
    pub platform: String,
    pub driver: Driver,
    /// The native command. `None` when stage 4 must run first and is not
    /// implemented for this driver yet.
    pub handoff: Option<Handoff>,
    /// The image's RMW, resolved the one way (RFC-0085 D2).
    ///
    /// The fact two derivations most easily disagree about: a west build reads
    /// `CONFIG_NROS_RMW_*` out of Kconfig, while `[image.*]` says `rmw`, and
    /// nothing made them agree. Exposed so `nros image-facts` can hand cmake
    /// the image's answer instead of cmake inferring its own.
    pub rmw: Option<String>,
    /// The entry package this image builds, when one was resolved.
    pub entry_package: Option<String>,
    /// The rustc target triple the board pins, if any.
    pub target: Option<String>,
    /// The cargo profile the image declares, if any.
    pub profile: Option<String>,

    /// A configure that must run BEFORE the handoff, for drivers that need one.
    ///
    /// cmake is the only such driver: `cmake --build` on an unconfigured tree
    /// fails, and configure+build is two invocations at our 3.22 floor
    /// (`--workflow` is 3.25+). Stage 5 execs ONE command and cannot do both,
    /// so the configure belongs to generation — which is what it is: writing
    /// the build system next to the root that was just written.
    ///
    /// Kept on the plan rather than performed during planning so `plan_builds`
    /// stays side-effect free and `--dry-run` can PRINT it. [`run`] performs it.
    pub configure: Option<Handoff>,
}

/// Stages 1-4: everything up to the handoff, with NO side effects.
///
/// Separated from [`run`] so the composition is testable without a built
/// binary and without exec'ing anything. That separation is also why
/// `--dry-run` is trivially correct rather than a second code path.
pub fn plan_builds(args: &Args) -> Result<Vec<ResolvedBuild>> {
    let root = match &args.workspace {
        Some(w) => w.clone(),
        None => std::env::current_dir().wrap_err("resolving cwd as the workspace root")?,
    };
    // ABSOLUTE from here on. `--workspace .` is an ordinary invocation — the
    // fixture driver cd's into the workspace and passes exactly that — and
    // every generated file computes paths RELATIVE to this root against the
    // nano-ros checkout and the user's packages. `relative_or_err` needs two
    // absolute paths and correctly refuses otherwise, so a relative root
    // surfaced as "cannot express /abs/packages/api/nros relative to
    // ./build/posix-zenoh/native_entry" — an error about the wrong thing.
    //
    // `canonicalize` rather than `absolute`: symlinked checkouts are normal
    // here, and two spellings of one directory would produce two different
    // relative paths in generated files that are supposed to be byte-identical.
    let root = std::fs::canonicalize(&root)
        .wrap_err_with(|| format!("resolving workspace root {}", root.display()))?;

    // ---- stage 1 --------------------------------------------------------
    let members = discover::cargo_members_or_walk(&root);
    let found = discover::discover(&root, &members).map_err(|e| eyre::eyre!("{e}"))?;
    for w in &found.warnings {
        eprintln!("nros build: warning: {w}");
    }
    if found.packages.is_empty() {
        eyre::bail!(
            "no packages under {} - is this a workspace root? A workspace has \
             packages carrying `package.xml`, or a `[workspace] members` list.",
            root.display()
        );
    }

    // ---- stage 2 --------------------------------------------------------
    // Images come from the FULL workspace, before any selection: an image is
    // declared by a bringup's `system.toml` and is a property of the workspace,
    // exactly as the generated cargo root's member list is. A selection that
    // happened to drop a bringup package must not make its images cease to
    // exist — that would answer `--packages-select` with "this workspace
    // declares no `[image.*]`", an error about the wrong thing.
    let bringups = collect_images(&found.packages)?;

    // ---- stage 1b — the selection verbs (RFC-0087 D7, phase-420 W7) -----
    //
    // Filters the topological order stage 1 already computed; it does not sort
    // again. `all_packages` keeps the unnarrowed set, because
    // `check_declared_depends` walks the whole tree itself and would read a
    // deliberately-dropped package as an unresolved `<depend>`.
    let all_packages = found.packages.clone();
    let found = discover::select(
        &found,
        &discover::Selection {
            select: args.packages_select.clone(),
            up_to: args.packages_up_to.clone(),
        },
    )
    .map_err(|e| eyre::eyre!("{e}"))?;

    let requested: Vec<String> = if args.all {
        plan::all_images(&bringups)
            .into_iter()
            .map(|(b, _, i, _)| plan::qualified(&b, &i))
            .collect()
    } else {
        args.images.clone()
    };
    let resolved = plan::resolve(&bringups, &requested).map_err(|e| eyre::eyre!("{e}"))?;

    // The driver is chosen by the board's PLATFORM, never by its name - a
    // Zephyr board is spelled `native_sim/native/64`, which says nothing about
    // being Zephyr. Resolving it needs the board catalog, which lives in a
    // nano-ros checkout, NOT in the user's workspace.
    let nano_ros_root = args
        .nano_ros_path
        .clone()
        .or_else(|| std::env::var_os("NROS_REPO_DIR").map(PathBuf::from))
        .or_else(|| crate::cmd::ws::autodetect_nano_ros_path(&root));
    // phase-398 W3 — every `<depend>` resolves, or the build stops.
    //
    // Runs once per invocation, before anything is generated, because an
    // undeclared prerequisite is cheapest to report before a toolchain is
    // touched (RFC-0065 D2's reasoning, applied to dependencies).
    check_declared_depends(&root, &all_packages, nano_ros_root.as_deref())?;

    // The workspace's OWN packages can carry board descriptors, so a board is
    // declared where everything else about this workspace is declared.
    let pkg_dirs: Vec<PathBuf> = found.packages.iter().map(|p| p.dir.clone()).collect();
    let catalog = match &nano_ros_root {
        Some(r) => {
            crate::orchestration::board_descriptor::BoardCatalog::load_with_packages(r, &pkg_dirs)
                .map_err(|e| eyre::eyre!("loading board descriptors from {}: {e}", r.display()))?
        }
        None => eyre::bail!(
            "no nano-ros checkout found, so board ids cannot be resolved. \
             Pass --nano-ros-path, or set NROS_REPO_DIR."
        ),
    };

    // Does the package graph cross languages? A CMakeLists is the signal — but
    // NOT the one a framework entry carries.
    //
    // phase-383 W8.a: `nano-ros-rt-eval` is pure Rust and holds exactly one
    // CMakeLists, `src/zephyr_entry/CMakeLists.txt`, which belongs to WEST.
    // Counting it routed every native image through cmake, which would have
    // failed on a workspace with no C or C++ in it at all. A framework entry's
    // build file is its framework's, not evidence about the graph.
    let framework_entries = framework_entry_dirs(&found, &catalog);
    let ws_non_rust: Vec<&str> = found
        .packages
        .iter()
        .filter(|p| !framework_entries.contains(&p.dir))
        .filter(|p| p.dir.join("CMakeLists.txt").is_file())
        .map(|p| p.name.as_str())
        .collect();

    // Does THIS IMAGE's graph cross languages? Not the workspace's.
    //
    // `examples/workspaces/safety` is the case the workspace-wide answer got
    // wrong: it holds C, C++ AND Rust node packages against one bringup, so
    // `has_non_rust` was true for every image and the two Rust images were
    // emitted as `nano_ros_add_executable` calls. That failed one layer down,
    // in the typed-entry codegen, with the Rust package named:
    //
    //     typed entry: launch node pkg `rust_safety_listener_pkg` exec
    //     `safe_listener` has no matching component in nros-metadata.json
    //
    // — correct, and about the wrong thing. It is the same refinement W8.a
    // already made once for framework entries ("a framework entry's build file
    // is its framework's, not evidence about the graph"), taken one level
    // finer: a C package the image never links is not evidence either.
    //
    // Evidence, not assumption: an image whose launch names NO known package
    // (unreadable file, `<include>`-only, a pkg outside this workspace) falls
    // back to the workspace answer rather than guessing Rust, because guessing
    // wrong toward cargo drops the C half of a graph silently, while guessing
    // wrong toward cmake fails loudly at configure.
    let image_has_non_rust = |image: &crate::orchestration::image::ImageBlock,
                              bringup_dir: &std::path::Path| {
        let pkgs = crate::orchestration::image::launch_node_pkgs(image, bringup_dir);
        let known: Vec<&String> = pkgs
            .iter()
            .filter(|n| found.packages.iter().any(|p| &p.name == *n))
            .collect();
        if known.is_empty() {
            return !ws_non_rust.is_empty();
        }
        known.iter().any(|n| ws_non_rust.contains(&n.as_str()))
    };

    let mut out = Vec::new();
    for (bringup, bringup_dir, image_id, image) in resolved {
        let qual = plan::qualified(&bringup, &image_id);
        let want_entry = crate::builder::entry::package_name(&image_id);
        // A `launch` that names no file is a typo, and W9.a wrote three of
        // them as PROSE fragments that survived two waves because nothing
        // built from the declarations. Caught here, against the bringup, with
        // the available names in the message.
        crate::orchestration::image::validate_image_launch(&image_id, &image, &bringup_dir)
            .map_err(|e| eyre::eyre!("{e}"))?;
        let descriptor =
            crate::orchestration::image::resolve_image_board(&catalog, &image_id, &image)
                .map_err(|e| eyre::eyre!("{e}"))?;
        let platform = descriptor.platform.kebab().to_string();
        let board = image.board.clone().unwrap_or_default();
        let driver = plan::driver_for(&platform, image_has_non_rust(&image, &bringup_dir));

        // ---- stage 3 ----------------------------------------------------
        // Before anything is generated or compiled: a missing prerequisite
        // fails HERE, naming the command that fixes it (RFC-0065 D2).
        let missing = crate::builder::preflight::check(descriptor, &root, nano_ros_root.as_deref());
        if !missing.is_empty() {
            eyre::bail!("{}", crate::builder::preflight::report(&missing));
        }

        // ---- stage 4 ----------------------------------------------------
        let mut cmake_configure: Option<Handoff> = None;
        let mut cargo_prepare: Option<Handoff> = None;
        let handoff = match driver {
            Driver::Cargo => {
                // W3.b — generate the entry package. This is D4's headline
                // claim: the entry stops being hand-written.
                let generated = generate_entry(
                    &root,
                    &bringup_dir,
                    &bringup,
                    &image_id,
                    &image,
                    descriptor,
                    &platform,
                    nano_ros_root.as_deref(),
                )?;
                let entry_dir = generated.as_ref().map(|(d, _)| d.clone());
                let entity_facts = generated.map(|(_, f)| f).unwrap_or_default();
                if let Some(d) = &entry_dir {
                    eprintln!("nros build:   entry → {}", d.display());
                }

                // W7.a — the declarative escapes reach cargo here. `panic` is
                // forwarded to the ENTRY (the macro consumes it) rather than to
                // cargo; `profile` names a cargo profile.
                if let Some(p) = image.panic.as_deref() {
                    crate::orchestration::image::validate_panic(Some(p))
                        .map_err(|e| eyre::eyre!("`[image.{image_id}]`: {e}"))?;
                }

                // `rmw` reaches the build on this driver too, since issue 0831.
                //
                // It used to be inert here and the build REFUSED an image whose
                // rmw differed from `[system] rmw`, because the backend came
                // from the `<entry>_nros_selection` facade and nothing
                // consulted the image — so `[image.native_cyclonedds]` produced
                // `build/posix-cyclonedds/` holding a zenoh binary. The facade
                // now reads the image (`facade::image_rmw`), so the refusal is
                // gone and the coordinate directory names what it contains.
                // The cargo root lives at the WORKSPACE root, not under
                // build/ — cargo requires members to sit below their root and
                // resolves a package's workspace by walking up. An existing
                // hand-written root is used as-is, never overwritten.
                // Did WE write this root? `has_tracked_root` answers it the
                // same way `cargo_root::ensure` decides whether to write.
                let generated_root = !crate::builder::cargo_root::has_tracked_root(&root);
                let excluded = cargo_excluded_entry_dirs(&found, &catalog);
                // EVERY cargo image's entry, not just this one.
                //
                // The root is a property of the WORKSPACE; making its member
                // list depend on which image is being built means the list —
                // and therefore `Cargo.lock` — changes on every image switch.
                // With the `--locked` the cargo shim injects project-wide, that
                // is a hard error ("cannot update the lock file ... because
                // --frozen was passed"), and without it, a silent re-resolve
                // plus a full fingerprint invalidation on every switch.
                //
                // Generating them all is cheap (an entry is two small files)
                // and restores the shape the hand-written roots had: the rust
                // workspace's listed all seventeen entries and built one with
                // `-p`. phase-383 W9.b found this the first time a driver built
                // two images of one workspace in a row.
                let extra = all_cargo_entry_dirs(
                    &bringups,
                    &bringup,
                    &root,
                    &catalog,
                    &image_has_non_rust,
                    nano_ros_root.as_deref(),
                    entry_dir.clone(),
                )?;
                crate::builder::cargo_root::ensure(
                    &found,
                    &root,
                    &excluded,
                    &extra,
                    Some(&bringup),
                )
                .map_err(|e| eyre::eyre!("{e}"))?;
                let mut a = vec!["build".to_string()];
                // Build ONLY this image's entry. A bare `cargo build` at the
                // root builds every member, and nano-ros-rt-eval's own manifest
                // records why that is wrong: a cross-target member "would try
                // [it] for the host and fail".
                if let Some(d) = &entry_dir
                    && let Some(name) = d.file_name().and_then(|n| n.to_str())
                {
                    a.push("-p".to_string());
                    a.push(name.to_string());
                } else if root
                    .join("src")
                    .join(&want_entry)
                    .join("Cargo.toml")
                    .is_file()
                {
                    a.push("-p".to_string());
                    a.push(want_entry.clone());
                }
                // A cross board pins a triple, and dropping it builds the image
                // for the HOST — silently, since cargo is happy to. phase-383
                // W9 caught this on the freertos image, whose board declares
                // thumbv7m-none-eabi.
                if let Some(triple) = descriptor.target.as_deref() {
                    a.push("--target".to_string());
                    a.push(triple.to_string());
                }
                if let Some(profile) = image.profile.as_deref() {
                    // `--profile` rather than `--release`: a named profile is
                    // what `[image.<id>].profile` declares, and `release` is
                    // just one of its legal values.
                    a.push("--profile".to_string());
                    a.push(profile.to_string());
                }
                if args.offline {
                    // `--frozen` is `--locked --offline` by definition; issue
                    // 0676 records why `--offline` alone is the wrong spelling
                    // (it restricts the cache without pinning resolution).
                    a.push("--frozen".to_string());

                    // A GENERATED root has a GENERATED lock, and `--frozen`
                    // forbids creating one:
                    //
                    //   error: cannot update the lock file … because --frozen
                    //   was passed to prevent this
                    //
                    // `--locked` exists to stop a build silently re-resolving
                    // an AUTHORED lock — a promise that someone else's build
                    // resolves what yours did. This lock is build output of a
                    // root this process just wrote, so there is no promise to
                    // protect, and the first build after a clone has nothing to
                    // be frozen against.
                    //
                    // So resolve ONCE, offline, before the frozen build. The
                    // build itself stays frozen, which is the property issue
                    // 0676 wants. `NROS_CARGO_FLAGS=` because the PATH shim
                    // injects `--locked` project-wide and would forbid this
                    // step too.
                    // Whenever the root is ours — not only when the lock is
                    // absent. A workspace migrated from a hand-written root
                    // still carries that root's lock, and it does not describe
                    // the generated member list, so `--frozen` refuses it just
                    // the same. `generate-lockfile --offline` is a no-op when
                    // the lock already satisfies the manifest, so the common
                    // case costs nothing.
                    if generated_root {
                        cargo_prepare = Some(
                            Handoff::new(
                                "cargo",
                                vec!["generate-lockfile".to_string(), "--offline".to_string()],
                            )
                            .in_dir(&root)
                            .with_env("NROS_CARGO_FLAGS", ""),
                        );
                    }
                }
                a.extend(args.native_args.iter().cloned());
                // Run FROM the workspace root, not the manifest dir: cargo
                // discovers `.cargo/config.toml` by walking up from the CWD,
                // and the leaf `[patch.crates-io]` redirects `nros sync` writes
                // live there. Building from build/<coord> would lose every one
                // of them and resolve message crates against the public
                // registry — issue 0378 by a different road.
                // The ENTITY facts ride the handoff (phase-392 W5). The env is
                // the only carrier that reaches a build script inside the cargo
                // invocation, which is what `entity_facts` was designed around —
                // and this is the cargo half of that delivery, the CMake half
                // being `NanoRosEntityFacts.cmake`. Without it the zenoh backend
                // sizes SERVICE_BUFFERS for 8 service servers an image may not
                // have, which overflows DRAM on esp32.
                let mut h = Handoff::new("cargo", a).in_dir(&root);
                for (k, v) in &entity_facts {
                    h = h.with_env(k, v);
                }
                Some(h)
            }
            Driver::CMake => {
                // Unlike cargo, cmake imposes no root/member hierarchy rule, so
                // this root DOES live under build/<coord> (RFC-0065 D8).
                let manifest_dir = root.join("build").join(cmake_coordinate(&platform, &image));
                // W4.b — every image that lands on THIS coordinate.
                //
                // They share `build/<coord>/`, so emitting only the image being
                // built means the root is rewritten on every image switch and
                // the workspace never declares more than one executable at a
                // time. Same shape as the cargo root's member list, same
                // answer: the root is a property of the WORKSPACE.
                //
                // An image still carrying a hand-written package contributes
                // nothing — it is a discovered SUBDIR, and a second target of
                // that name would collide. Delete the package and the next
                // build emits its call (D13, incremental).
                let coord = cmake_coordinate(&platform, &image);
                // A C++ source ANYWHERE in a package, not just at its top.
                //
                // These packages keep sources in `src/` — `talker_pkg/src/Talker.cpp` —
                // so a top-level scan called the pure-C++ workspace `c`, and
                // `nros codegen entry` refused with the right complaint from the
                // wrong layer: "node pkg `talker_pkg` exec `talker` is lang
                // `cpp`, not `c`". The model knows each exec's language; until
                // the emitter reads it, look where the sources actually are.
                let has_cpp = found.packages.iter().any(|p| {
                    [p.dir.clone(), p.dir.join("src")].iter().any(|d| {
                        d.read_dir()
                            .map(|rd| {
                                rd.flatten().any(|e| {
                                    let n = e.file_name();
                                    let n = n.to_string_lossy();
                                    n.ends_with(".cpp") || n.ends_with(".cc") || n.ends_with(".cxx")
                                })
                            })
                            .unwrap_or(false)
                    })
                });
                let cmake_entries = plan::all_images(&bringups)
                    .into_iter()
                    .filter(|(b, bd, _, img)| {
                        b == &bringup
                            && crate::orchestration::image::resolve_image_board(&catalog, "", img)
                                .map(|d| {
                                    plan::driver_for(
                                        d.platform.kebab(),
                                        image_has_non_rust(img, bd),
                                    ) == Driver::CMake
                                        && cmake_coordinate(d.platform.kebab(), img) == coord
                                })
                                .unwrap_or(false)
                    })
                    .filter_map(|(_, _, id, img)| {
                        let name = crate::builder::entry::package_name(&id);
                        if root
                            .join("src")
                            .join(&name)
                            .join("CMakeLists.txt")
                            .is_file()
                        {
                            return None;
                        }
                        let b = img.board.clone().unwrap_or_default();
                        Some(crate::builder::cmake_root::CmakeEntry {
                            launch: img.launch.clone().unwrap_or_else(|| "default".to_string()),
                            args: img
                                .args
                                .iter()
                                .map(|(k, v)| (k.clone(), v.clone()))
                                .collect(),
                            // The workspace's own language: the generated TU has
                            // to compile against what it links.
                            lang: if has_cpp { "cpp" } else { "c" }.to_string(),
                            // The SAME candidate search the Rust entry uses:
                            // DEPLOY is what the macro looks up, and an image is
                            // not always named after a board.
                            // The BOARD, verbatim — not `macro_deploy_token`.
                            //
                            // That function answers for the RUST macro's board
                            // table, which is keyed on tokens like `freertos`
                            // and does not know `mps2-an385-freertos`.
                            // `nano_ros_add_executable(DEPLOY …)` resolves
                            // against the board CATALOG, which does, and the
                            // hand-written entry said exactly the board id.
                            // Routing it through the macro's table picked the
                            // GENERIC freertos board, and nothing failed until
                            // the link, where the mps2 board's lwIP glue was
                            // absent: `undefined reference to lwip_setsockopt`.
                            deploy: if b.is_empty() {
                                platform.clone()
                            } else {
                                b.clone()
                            },
                            panic: img.panic.clone(),
                            name,
                        })
                    })
                    .collect();

                let spec = crate::builder::cmake_root::CmakeRootSpec {
                    entries: cmake_entries,
                    workspace: root.clone(),
                    system: bringup.clone(),
                    platform: platform.clone(),
                    board: image.board.clone(),
                    rmw: image.rmw.clone().unwrap_or_else(|| "zenoh".to_string()),
                    toolchain_file: descriptor.cmake.as_ref().map(|c| c.toolchain_file.clone()),
                    nano_ros_root: nano_ros_root.clone().unwrap_or_default(),
                    excluded: {
                        let mut e = framework_entries.clone();
                        e.extend(entries_for_other_boards(&found, &board, &platform));
                        e
                    },
                };
                crate::builder::cmake_root::write(&found, &manifest_dir, &spec)
                    .map_err(|e| eyre::eyre!("{e}"))?;
                let rel_src = manifest_dir
                    .strip_prefix(&root)
                    .unwrap_or(&manifest_dir)
                    .display()
                    .to_string();
                // The configure. Its own step, not the handoff.
                //
                // The comment here used to say "configure and build in one
                // handoff", and the args only ever configured — so `nros build`
                // on a cmake workspace wrote a build system and produced no
                // binary. CMake cannot do both in one invocation at our 3.22
                // floor (`--workflow` is 3.25+), and stage 5 execs exactly one
                // command, so the configure moves to generation where it
                // belongs: it WRITES the build system, next to the root file
                // this stage just wrote. [`run`] performs it before the exec.
                let mut a = vec![
                    "-S".to_string(),
                    rel_src.clone(),
                    "-B".to_string(),
                    format!("{rel_src}/cmake"),
                ];
                // The preamble path is passed rather than discovered inside the
                // generated file, so the generated file stays workspace-agnostic.
                let preamble = bringup_dir.join("cmake/preamble.cmake");
                if preamble.is_file() {
                    a.push(format!("-DNROS_WS_PREAMBLE={}", preamble.display()));
                }
                a.extend(args.native_args.iter().cloned());
                cmake_configure = Some(Handoff::new("cmake", a).in_dir(&root));
                Some(
                    Handoff::new(
                        "cmake",
                        vec!["--build".to_string(), format!("{rel_src}/cmake")],
                    )
                    .in_dir(&root),
                )
            }
            Driver::West => {
                // W5 — overlays reach Zephyr through EXTRA_CONF_FILE and
                // APPLICATION_CONFIG_DIR. Never CONF_FILE: that suppresses
                // Zephyr's own boards/ and socs/ discovery entirely.
                // The application is resolved FIRST: its directory is where a
                // Zephyr app keeps its own `prj-*.conf`, so the overlay search
                // needs it (issue 0892).
                let app = west_application_dir(&image_id, &image, descriptor, &found, &catalog)?
                    .unwrap_or_else(|| bringup_dir.clone());
                let overlays =
                    crate::builder::zephyr::resolve_in(&bringup_dir, Some(&app), &board, &image)
                        .map_err(|e| eyre::eyre!("{e}"))?;
                // issue 0892 — Zephyr is not like the other drivers, and the
                // handoff has to say so.
                //
                // cargo and cmake let us OWN the root: we generate it from the
                // images and hand the tool a directory we wrote. west does not
                // work that way. The user owns the west workspace (`.west/`,
                // `zephyr/`, `modules/nano-ros/`, `apps/`), the application is a
                // stock Zephyr app whose `prj.conf` and `CMakeLists.txt` carry
                // authored Kconfig no image declaration expresses (RFC-0065
                // D5), and `west` refuses to run outside that workspace.
                //
                // So: point at the real APPLICATION, run from the USER's
                // workspace, and when that workspace cannot be found, print the
                // command instead of emitting one that cannot work. That last
                // part is `nros setup --system`'s sudo boundary applied here —
                // compose the command, hand it over, do not pretend.
                // `west build` has two argument zones and our single `--` can
                // only name one, so the passthrough is SPLIT by west's own flag
                // list rather than dropped whole into the second zone: that put
                // `-- --pristine` in front of cmake, which failed as
                // `CMake Error: Unknown argument --pristine`, naming the wrong
                // tool for the user's mistake.
                let (west_extra, cmake_extra) =
                    crate::builder::zephyr::split_native_args(&args.native_args)
                        .map_err(|e| eyre::eyre!("{e}"))?;

                // The board id WEST knows, which is not always the name the
                // image authored — see `BoardDescriptor::west_board`.
                let west_board = descriptor
                    .west_board
                    .clone()
                    .unwrap_or_else(|| board.clone());
                let mut a = vec!["build".to_string(), "-b".to_string(), west_board];
                if overlays.sysbuild {
                    a.push("--sysbuild".to_string());
                }
                a.push(app.display().to_string());
                // AFTER the application path, which looks unusual and is the
                // point. `-p`/`--pristine` takes an OPTIONAL value
                // (`nargs='?'`), so argparse greedily reads whatever follows —
                // put it before the positional and west takes the application
                // path as the pristine mode:
                //
                //   west build: error: argument -p/--pristine: invalid choice:
                //   '…/src/zephyr_entry' (choose from 'auto', 'always', 'never')
                //
                // Placing the user's options last is correct for every flag
                // shape without this code having to model west's argparse
                // arities, and west accepts options after the positional.
                a.extend(west_extra);
                let west_opts = crate::builder::zephyr::west_args(&overlays);
                if !west_opts.is_empty() || !cmake_extra.is_empty() {
                    // Everything after `--` is a cmake option for the app.
                    a.push("--".to_string());
                    a.extend(west_opts);
                    a.extend(cmake_extra);
                }
                // Resolved here, ENFORCED at exec. A plan is an answer to
                // "what would you run", and `--dry-run` must be able to answer
                // it from a machine with no west workspace at all — refusing
                // there withholds the very command the message tells the user
                // to run. `plan_builds` is also how the pipeline tests assert
                // driver selection, which needs no workspace either.
                let zbase = zephyr_base(&root, args.zephyr_workspace.as_deref());
                if zbase.is_none() && !args.dry_run {
                    let opts = crate::builder::zephyr::west_args(&overlays);
                    let line = format!(
                        "west {}{}{}",
                        a.join(" "),
                        if opts.is_empty() { "" } else { " -- " },
                        opts.join(" ")
                    );
                    eyre::bail!(
                        "no Zephyr found, so `west build` cannot be run for \
                         `{image_id}`.\n\n\
                         Zephyr differs from the other drivers: YOU own the west \
                         workspace and the application. `west build` needs a \
                         Zephyr — with `ZEPHYR_BASE` set it runs from anywhere, \
                         which is how a FREESTANDING application (one outside the \
                         west workspace) builds.\n\n\
                         Point nros at your workspace:\n\n    \
                         nros build {image_id} --zephyr-workspace <dir>\n\n\
                         where <dir> contains `zephyr/`. Or set it once for the \
                         shell:\n\n    \
                         export NROS_ZEPHYR_WORKSPACE=<dir>\n\n\
                         or run west yourself:\n\n    {line}\n\n\
                         (searched: --zephyr-workspace, $ZEPHYR_BASE, \
                         $NROS_ZEPHYR_WORKSPACE, <workspace>/zephyr-workspace, \
                         ../nano-ros-workspace[-4.4])"
                    );
                }
                // Run from the nros workspace with ZEPHYR_BASE set, which is
                // exactly what `west-fixtures.sh` does and what makes a
                // freestanding application work. No cwd gymnastics: the
                // application path is absolute.
                let h = Handoff::new("west", a).in_dir(&root);
                Some(match &zbase {
                    Some(z) => h.with_env("ZEPHYR_BASE", z.as_os_str()),
                    None => h,
                })
            }
            _ => Some(native_handoff(driver, &root, &bringup_dir, &board, args)),
        };

        out.push(ResolvedBuild {
            configure: cmake_configure.or(cargo_prepare),
            qualified: qual,
            board,
            platform,
            driver,
            handoff,
            // `image` is already merged with `[image_defaults]` by the
            // resolver, so this is the image's effective answer rather than
            // only what its own block spelled.
            rmw: image.rmw.clone(),
            entry_package: Some(want_entry.clone()),
            target: descriptor.target.clone(),
            profile: image.profile.clone(),
        });
    }
    Ok(out)
}

pub fn run(args: Args) -> Result<()> {
    // phase-429 W2 — the codegen version guard, BEFORE planning. `nros build`
    // is RFC-0065's front door and was the one codegen path with no guard on
    // it at all; planning is where the generated root is written, so the check
    // has to precede it rather than sit beside the handoff.
    //
    // Anchor: an explicitly-named nano-ros checkout if the caller gave one —
    // that IS the runtime this build links — else the workspace, which the
    // resolver walks up from.
    let guard_anchor = match (&args.nano_ros_path, &args.workspace) {
        (Some(p), _) => p.clone(),
        (None, Some(ws)) => ws.clone(),
        (None, None) => std::env::current_dir()?,
    };
    crate::abi_guard::check_workspace(&guard_anchor, crate::abi_guard::Verb::Build)?;

    let plans = plan_builds(&args)?;
    for p in &plans {
        eprintln!(
            "nros build: {} -> board {} (platform {}), driver {}",
            p.qualified,
            p.board,
            p.platform,
            p.driver.program()
        );
        let Some(hand) = &p.handoff else {
            eyre::bail!(
                "stage 4 (generate the root build file) is not implemented yet \
                 - phase-383 W3 (cargo) / W4 (cmake).\n\
                 `{}` needs a generated root, so it cannot be built through \
                 `nros build` today. Until W3/W4 land, build it the existing \
                 way (cargo build / cmake --build).\n\
                 Images on Zephyr and ESP32 boards work now - they need no \
                 generated root (RFC-0065 D3).",
                p.qualified
            );
        };
        if args.dry_run {
            if let Some(cfg) = &p.configure {
                println!("{}", cfg.display());
            }
            println!("{}", hand.display());
            continue;
        }
        // The configure, for drivers that need one (cmake). Runs HERE rather
        // than during planning so `plan_builds` stays side-effect free — the
        // property that makes `--dry-run` trivially correct instead of a second
        // code path. It is a subprocess, not an exec: the exec below has to
        // survive it.
        if let Some(cfg) = &p.configure {
            run_configure(cfg)?;
        }
        // Never returns on success: this process BECOMES the build.
        let err = crate::builder::handoff::exec(hand).unwrap_err();
        eyre::bail!("{err}");
    }
    Ok(())
}

/// Run a configure step, escalating an `--offline` failure to one
/// online retry.
///
/// `--offline` is an OPTIMIZATION on this path, never a semantic
/// choice. The step resolves a lock for a root this process just
/// generated, and issue 0676's frozen property belongs to the BUILD,
/// which stays `--locked` either way — so offline buys "do not touch
/// the network when the answer is already local", and nothing else.
///
/// Offline resolution fails for exactly one reason a retry fixes: the
/// registry cache does not hold some member's dependency yet. On CI
/// that cache is cold every run, which is why all three nightly
/// platform lanes died here — on `esp-backtrace` and
/// `panic-semihosting`, dependencies of workspace MEMBERS the built
/// platform never uses, reached only because `examples/workspaces/rust`
/// is one cargo workspace and resolving it resolves every member
/// (issue 0873). Reported as five OVERCLAIMED platforms, which is a
/// claim about the platforms rather than about our registry cache.
///
/// The retry cannot change WHAT resolves: the offline cache is a subset
/// of the registry, so a lock that resolves offline resolves identically
/// online. It changes only whether resolution can happen at all. A
/// genuinely missing package still fails, now with the registry's own
/// error instead of cargo's "offline mode (via `--offline`) can
/// sometimes cause surprising resolution failures" note.
fn run_configure(cfg: &Handoff) -> Result<()> {
    let st = cfg
        .command()
        .status()
        .wrap_err_with(|| format!("running `{}`", cfg.display()))?;
    if st.success() {
        return Ok(());
    }

    let Some(online) = cfg.without_offline() else {
        eyre::bail!("configure failed: `{}` exited {}", cfg.display(), st);
    };
    eprintln!(
        "nros build: warning: `{}` failed against a cold registry cache; \
         retrying online (issue 0873)",
        cfg.display()
    );
    let st = online
        .command()
        .status()
        .wrap_err_with(|| format!("running `{}`", online.display()))?;
    if !st.success() {
        eyre::bail!("configure failed: `{}` exited {}", online.display(), st);
    }
    Ok(())
}

/// The capability axes a bringup's `system.toml` turns on — the axes a missing
/// selection facade would silently drop (phase-413 W2).
///
/// Empty when the file is absent or unparseable ON PURPOSE: this decides whether
/// to ESCALATE a warning into a hard error, and a system that cannot be read is
/// not evidence that a capability is declared. Whatever is wrong with it will be
/// reported by the code whose job that is, with better words than this has.
fn declared_capabilities(bringup_dir: &std::path::Path) -> Vec<&'static str> {
    let Ok(raw) = std::fs::read_to_string(bringup_dir.join("system.toml")) else {
        return Vec::new();
    };
    let Ok(sys) = toml::from_str::<crate::orchestration::cargo_metadata_schema::SystemToml>(&raw)
    else {
        return Vec::new();
    };
    cargo_nano_ros::capability_resolver::CAPABILITIES
        .iter()
        .filter(|c| sys.capability_enabled(c.declared))
        .map(|c| c.declared)
        .collect()
}

/// Generate the entry package for a cargo image (W3.b), returning its
/// directory. `None` when the launch tree cannot be resolved — reported as a
/// warning rather than a failure, because a workspace whose entries are still
/// hand-written must keep building through the migration (RFC-0065 D13).
//
// Eight arguments: this is the entry generator's whole input, every parameter is
// a distinct fact about the image being generated, and bundling them into a
// struct would move the argument list rather than shorten it.
//
// This allow is not new. It has been here since the eighth argument arrived, and
// phase-413 W2 inserted `declared_capabilities` BETWEEN it and this function, so
// both it and the doc comment above re-attached to that one-argument function —
// silently, because a detached attribute is still valid syntax on whatever
// follows it. Clippy then reported `too_many_arguments` against `generate_entry`,
// naming the item that LOST the attribute and never the insertion that took it.
#[allow(clippy::too_many_arguments)]
fn generate_entry(
    root: &std::path::Path,
    bringup_dir: &std::path::Path,
    bringup: &str,
    image_id: &str,
    image: &crate::orchestration::image::ImageBlock,
    descriptor: &crate::orchestration::board_descriptor::BoardDescriptor,
    platform: &str,
    nano_ros_root: Option<&std::path::Path>,
) -> Result<Option<(PathBuf, std::collections::BTreeMap<String, String>)>> {
    use crate::{
        builder::entry::{BoardFacts, EntrySpec},
        orchestration::model_location,
    };

    let Some(nros_root) = nano_ros_root else {
        return Ok(None);
    };

    // A workspace that still has its hand-written entry keeps it. Generating a
    // second one would be redundant at best and a conflicting `[[bin]]` name at
    // worst — and D13's migration is a DELETION: remove the hand-written entry
    // and the next build generates it. This is what makes the migration
    // incremental, one entry at a time.
    // Keyed on the MANIFEST, not the directory. `git rm -r src/<entry>` leaves
    // gitignored residue behind — `.cargo/` holds the sync-written sidecar —
    // so a directory-existence check reads a deleted entry as still present and
    // silently generates nothing. phase-383 W10 tripped over exactly that on
    // the first workspace it tried.
    let want = crate::builder::entry::package_name(image_id);
    let hand_written = root.join("src").join(&want);
    if hand_written.join("Cargo.toml").is_file() || hand_written.join("CMakeLists.txt").is_file() {
        return Ok(None);
    }

    // (launch, args) → model → plan → the node packages the launch names.
    let args_vec: Vec<(String, String)> = image
        .args
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect();
    let model_rel = match model_location::launch_to_model_rel(
        bringup_dir,
        image.launch.as_deref(),
        &args_vec,
    ) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("nros build: warning: cannot resolve launch for `{image_id}`: {e}");
            return Ok(None);
        }
    };
    let model_path = match model_location::ensure_model(bringup_dir, &model_rel) {
        Ok((p, _inputs)) => p,
        Err(e) => {
            eprintln!("nros build: warning: cannot resolve the model for `{image_id}`: {e}");
            return Ok(None);
        }
    };
    let plan = match crate::codegen::entry::plan_from_model(&model_path, image.board.clone()) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("nros build: warning: cannot plan `{image_id}`: {e}");
            return Ok(None);
        }
    };

    // A launch file may name one package several times; cargo needs it once.
    let mut seen = std::collections::BTreeSet::new();
    let mut nodes = Vec::new();
    for n in &plan.nodes {
        if !seen.insert(n.pkg.clone()) {
            continue;
        }
        let dir = root.join("src").join(&n.pkg);
        if dir.is_dir() {
            nodes.push((n.pkg.clone(), dir));
        }
    }

    let launch = match image.launch.as_deref() {
        Some(f) => format!("{bringup}:{f}"),
        None => bringup.to_string(),
    };
    // DEFERRED, not immediate — the fatal below fires AFTER the entry package is
    // written. See the block comment there.
    let mut facade_missing_fatal: Option<String> = None;
    let facade_dir = {
        let d = root
            .join("generated/nros-selection")
            .join(crate::builder::entry::package_name(image_id));
        if d.is_dir() {
            Some(d)
        } else {
            // SAY SO. The facade carries the RMW, the ROS edition and the
            // capability features, and the Entry is emitted without any of them
            // when it is missing — silently, until the build fails somewhere
            // that names none of it. Issue 0937 was one of these: the NuttX
            // Entry reached the tree's single `#[global_allocator]` only through
            // the facade, so a missing facade surfaced as
            //
            //     error: no global memory allocator found but one is required
            //
            // several hundred lines into a cross build, and the nightly cell
            // stayed red while the same Entry built fine in the workspace next
            // door, which happened to have one.
            //
            // WARN or FAIL, decided by what the facade would have CARRIED —
            // phase-413 W2.
            //
            // "An unsynced workspace is a tolerable state" is true exactly when
            // the facade adds nothing the Entry cannot do without. It is false
            // the moment the system declares a CAPABILITY: those reach the
            // Entry only through the facade's `nros` feature list, so building
            // without it produces an Entry that provably cannot be correct, and
            // the failure lands somewhere that names neither sync nor the
            // facade. `host-tests` spent four runs on the far end of that:
            //
            //   error[E0080]: evaluation panicked: this system declares
            //   `[param_services]` but this `nros` build does not carry the
            //   `param-services` feature
            //     --> build/posix-zenoh/native_rust_qos_entry/src/main.rs:9:1
            //
            // — a const-eval panic in a generated `main.rs`, six lines below
            // this very warning in the same log. The warning was right and
            // nobody read it, which is what a warning is worth on a build that
            // continues.
            //
            // The RMW and ROS edition USED to stay a warning, on the grounds
            // that "both have defaults the Entry can build against". Issue 0831
            // took that away and the warning did not follow.
            //
            // The board crate's `default = ["rmw-zenoh"]` was that default. It
            // is gone: 0831 made the facade the ONE place that names the RMW,
            // so `builder::entry` now emits the board dep
            // `default-features = false` (cargo cannot subtract a default —
            // issue 0270 — so both declarations have to be silent about it).
            // Measured on this tree, `[image.native_cyclonedds] rmw =
            // "cyclonedds"` with no facade:
            //
            //     nros-board-linux = { path = "…", default-features = false }
            //
            // No `rmw-*` at all, and no `ethernet` / `image-runtime` either —
            // the facade is what re-supplies the board's non-RMW defaults. So
            // the Entry that warning lets through has NO backend, NO ROS
            // edition and none of the board's own defaults, while the message
            // says "nothing else is lost". That is 0831's shape exactly, one
            // door over: `[image.<id>].rmw` is declared, cannot take effect,
            // and the build says so in a line that reads like reassurance.
            //
            // So there is no tolerable case left, and the split goes away. The
            // heal below is what makes that affordable — it WRITES the facade
            // from the entry just generated, so nothing is refused that can be
            // repaired, and only a facade that cannot be written is fatal.
            // `declares_capability` survives because it makes the refusal name
            // the right remedy.
            let declares_capability = declared_capabilities(bringup_dir);

            if !declares_capability.is_empty() {
                // RECORDED, and raised after `builder::entry::write` below.
                //
                // Bailing HERE deadlocks a fresh checkout, and it did: this
                // check shipped as an immediate `bail!` and `host-tests` went
                // from a const-eval panic to
                //
                //   Error: nros build: `native_rust_qos` needs a selection
                //   facade and there is none at …
                //
                // which is this message — correct, and unescapable. `nros sync`
                // writes a facade per ENTRY PACKAGE, and the entry package is
                // GENERATED BY THIS FUNCTION: the workspace root lists
                // `build/posix-zenoh/<entry>` as a cargo member, but the
                // directory is untracked, so on a clone sync finds no
                // `Cargo.toml` there and skips it (`ws.rs`: `if
                // !cargo_toml.is_file() { continue; }`). Bail before writing
                // the entry and sync can never see it, so the facade can never
                // exist, so the build can never proceed.
                //
                // The pre-existing WARNING was self-healing for exactly this
                // reason — it wrote a featureless Entry, the next sync saw the
                // package and produced the facade, and the build after that was
                // correct. That is what "an unsynced workspace is a state the
                // build is documented to tolerate" bought, and turning the
                // warning into an immediate error spent it without noticing.
                //
                // So: generate the entry (cheap, and it is what unblocks sync),
                // THEN refuse to go further. Loud at the point of decision, and
                // the remedy it names actually works.
                facade_missing_fatal = Some(format!(
                    "nros build: `{image_id}` needs a selection facade and there is none at {}.\n\
                     \n\
                     Its system declares {} — capabilities reach the Entry ONLY through the\n\
                     facade's `nros` feature list, so building without it yields an Entry that\n\
                     cannot be correct. The failure would surface later as a const-eval panic\n\
                     in a generated `main.rs` naming neither sync nor this facade (issue 0937).\n\
                     \n\
                     The entry package HAS been generated, so `nros sync` can see it now.\n\
                     Run `nros sync` in the workspace, then build again (or `just build\n\
                     <scope>`, which runs codegen for you).",
                    d.display(),
                    declares_capability
                        .iter()
                        .map(|c| format!("`[{c}]`"))
                        .collect::<Vec<_>>()
                        .join(", "),
                ));
            } else {
                // Issue 0831. Same deferral, same self-heal, different reason:
                // the facade carries the RMW this image DECLARED and the ROS
                // edition, and since 0831 the Entry has no other way to get
                // either.
                facade_missing_fatal = Some(format!(
                    "nros build: `{image_id}` needs a selection facade and there is none at {}.\n\
                     \n\
                     The facade is the one place that names {} and the ROS edition: the\n\
                     generated entry declares its board crate `default-features = false`\n\
                     precisely so the facade can name the backend once (issue 0831), so\n\
                     without it the entry links NO RMW backend, no ROS edition and none of\n\
                     the board's own defaults. It would compile and then refuse to select a\n\
                     backend at run time, in a build directory named for the one it declared.\n\
                     \n\
                     The entry package HAS been generated, so `nros sync` can see it now.\n\
                     Run `nros sync` in the workspace, then build again (or `just build\n\
                     <scope>`, which runs codegen for you).",
                    d.display(),
                    match image.rmw.as_deref() {
                        Some(r) => format!("this image's `rmw = \"{r}\"`"),
                        None => "the bringup's `[system] rmw`".to_string(),
                    },
                ));
            }
            None
        }
    };

    // Most specific first: the image id IS the deploy key when an image is
    // named after a board, but `[image.native_service_server]` is not, and
    // `[image.robot1]` is not — so the board and platform back it up. ONE list,
    // consumed by both the deploy token and the board crate, because the macro
    // resolves the crate FROM the token: two searches could disagree, and the
    // disagreement is a generated entry that does not compile.
    let board_name = image.board.clone().unwrap_or_default();
    // BOARD first, then the image id, then the platform.
    //
    // The board is what the user DECLARED; the image id is a label that may or
    // may not happen to be a board token. Taking the id first resolved
    // `[image.freertos] board = "mps2-an385-freertos"` to the generic
    // `freertos` board — a real board, so nothing failed until the link, where
    // the mps2 board's lwIP glue was simply absent:
    //
    //   undefined reference to `lwip_setsockopt' … `lwip_socket_thread_init'
    //
    // The hand-written entry said `DEPLOY mps2-an385-freertos`. A generated one
    // must not quietly pick a different board than the image names — that is
    // issue 0798 with the roles reversed.
    let candidates = [board_name.as_str(), image_id, platform];

    // Where the entry will be written — needed to make its dependency paths
    // relative, and computed before the spec because the spec carries them.
    let entry_dir_for_deps = root
        .join("build")
        .join(coordinate(platform, image))
        .join(crate::builder::entry::package_name(image_id));

    let mut spec = EntrySpec {
        image_id: image_id.to_string(),
        deploy: crate::builder::entry::macro_deploy_token(&candidates),
        launch,
        args: image.args.clone(),
        panic: image.panic.clone(),
        nodes,
        nano_ros_root: nros_root.to_path_buf(),
        facade_dir,
        // A `[[bridge]]` in the bringup makes the macro emit a call into
        // `nros_bridge`; nothing in the package graph implies it, and the
        // hand-written bridge entries listed it by hand.
        bringup_deps: {
            let mut v = Vec::new();
            let toml_path = bringup_dir.join("system.toml");
            if std::fs::read_to_string(&toml_path)
                .ok()
                .and_then(|t| t.parse::<toml::Value>().ok())
                .and_then(|d| {
                    d.get("bridge")
                        .map(|b| b.as_array().is_some_and(|a| !a.is_empty()))
                })
                .unwrap_or(false)
            {
                let rel = crate::builder::paths::relative_or_err(
                    &entry_dir_for_deps,
                    &nros_root.join("packages/rmw/bridge"),
                )
                .map_err(|e| eyre::eyre!("{e}"))?;
                v.push(format!(
                    "nros-bridge = {{ path = \"{rel}\", features = [\"std\", \"config\"] }}"
                ));
            }
            v.extend(
                bridge_entry_deps(bringup_dir, &entry_dir_for_deps, nros_root, &candidates)
                    .map_err(|e| eyre::eyre!("{e}"))?,
            );
            v
        },
    };
    let facts = BoardFacts::from_descriptor_for(descriptor, &candidates);
    let parent = root.join("build").join(coordinate(platform, image));
    let dir = crate::builder::entry::write(&spec, &facts, &parent)
        .map_err(|e| eyre::eyre!("generating the entry for `{image_id}`: {e}"))?;

    // NOW refuse — the entry package exists, so `nros sync` can produce the
    // facade this build needed. Raising it earlier deadlocks a clone: sync
    // keys facades off the entry package, and this function is what writes it.
    if let Some(msg) = facade_missing_fatal {
        // SELF-HEAL, because CI only ever gets one pass.
        //
        // Deferring the refusal until after the entry was written (the previous
        // fix) made a DEVELOPER's tree recoverable: sync could finally see the
        // entry package, so the next `nros build` succeeded. It did nothing for
        // CI, and `host-tests` proved it — the run does
        //
        //     just _codegen        (sync: no entry yet, so no facade)
        //     nros build           (writes the entry, then refuses)
        //
        // and there is no second sync, so the lane failed on the same message
        // with the fix in place. A fresh checkout every run means the two-pass
        // recovery never happens.
        //
        // But nothing here actually needs sync. `write_facade` wants the entry
        // name, its directory, its manifest, the system and the facade root —
        // and we hold all five, because we just generated the entry. The one
        // input sync was missing is the thing this function produces.
        //
        // So: write the facade, then regenerate the entry with it. The second
        // write is not optional — the first ran with `facade_dir = None`, so
        // the entry carries no dependency on the facade crate, and a facade no
        // entry depends on selects nothing (cargo feature unification is the
        // whole mechanism). Two file writes, one build, no second invocation.
        //
        // Still fatal if the facade cannot be written: that is a real problem
        // and the message already names it.
        //
        // Issue 0831 — this now runs for EVERY missing facade, not only the
        // capability case. Once the board dep is emitted
        // `default-features = false`, a facade-less entry has no backend at
        // all, so there is nothing left for a warning to be tolerant of; and
        // healing is two file writes against inputs this function already
        // holds, so nothing that can be repaired is refused.
        //
        // The heal's own failure is REPORTED, not swallowed. It used to be
        // `.ok().flatten()`, which was tolerable while this ran only for a
        // workspace that declares capabilities; now that every missing facade
        // comes through here, "could not write it" is a message a user will
        // actually meet, and `resolve_rmw` refusing an unknown `rmw` is one of
        // the ways to get it. Naming sync when the real answer is a typo in
        // `[image.<id>] rmw` is the diagnostic this issue is about.
        let facade_root = root.join("generated/nros-selection");
        let entry_name = crate::builder::entry::package_name(image_id);
        let healed = std::fs::read_to_string(bringup_dir.join("system.toml"))
            .map_err(|e| eyre::eyre!("reading {}: {e}", bringup_dir.join("system.toml").display()))
            .and_then(|raw| {
                toml::from_str::<crate::orchestration::cargo_metadata_schema::SystemToml>(&raw)
                    .map_err(|e| eyre::eyre!("parsing the bringup's system.toml: {e}"))
            })
            .and_then(|sys| {
                crate::orchestration::facade::write_facade(
                    &entry_name,
                    &dir,
                    &dir.join("Cargo.toml"),
                    &sys,
                    &facade_root,
                )
            });

        let healed = match healed {
            Ok(Some(f)) => Some(f),
            Ok(None) => None,
            Err(e) => eyre::bail!("{msg}\n\nWriting it here failed too: {e}"),
        };
        let Some(_) = healed else {
            eyre::bail!("{msg}");
        };
        eprintln!(
            "nros build: wrote the missing selection facade for `{image_id}` from the entry \
             just generated, and regenerated the entry against it."
        );
        spec.facade_dir = Some(facade_root.join(&entry_name));
        crate::builder::entry::write(&spec, &facts, &parent)
            .map_err(|e| eyre::eyre!("regenerating the entry for `{image_id}`: {e}"))?;
    }

    // phase-392 W5 — the ENTITY facts, from the SAME model resolved above.
    //
    // `entity_facts`'s own docs say these are "delivered through the process
    // environment because that is the only carrier that reaches the cargo
    // invocation a workspace member is built by" — and until now only
    // `cmake/NanoRosEntityFacts.cmake` wired them, so the CMake path got
    // declaration-sized tables and the CARGO path silently kept the undeclared
    // fallback (32 hosted / 8 embedded).
    //
    // That is not a missing optimisation, it is a hard failure on a small
    // target: `esp32_entry` overflowed DRAM by 8,804 B while carrying
    // `SERVICE_BUFFERS` sized for 8 service servers it does not have. Measured
    // on a native talker when the CMake side landed: 144,128 -> 4,504 B.
    //
    // Resolved here rather than by a second call so there is ONE model read,
    // which is the property `entity_facts` was written to have.
    let entity_facts = match std::fs::read_to_string(&model_path)
        .map_err(|e| e.to_string())
        .and_then(|y| {
            ros_launch_manifest_model::SystemModel::from_yaml_str(&y).map_err(|e| e.to_string())
        }) {
        Ok(m) => crate::cmd::entity_facts::facts_from_model(&m),
        Err(e) => {
            // Not fatal: an unreadable model here means the entry was still
            // generated, and the undeclared fallback is the historical
            // behaviour. Say so rather than silently sizing for 8.
            eprintln!(
                "nros build: warning: cannot read `{}` for entity facts, so the \
                 backend keeps its undeclared table budget: {e}",
                model_path.display()
            );
            std::collections::BTreeMap::new()
        }
    };
    Ok(Some((dir, entity_facts)))
}

/// The crates a BRIDGE entry must name directly, derived from the bringup.
///
/// `nros::main!` emits `::<crate>::register()` for every backend the bridge
/// spans and `::nros_rmw_cyclonedds::register::<M>()` for each non-flat egress
/// type, so those crates have to be in the ENTRY's dependency list — a feature
/// on the board crate compiles and links them but does not put their names in
/// scope. The two hand-written bridge entries listed them by hand; this is the
/// same list, derived.
///
/// **Backends.** The set is the image's own RMW plus every `[[domain]].rmw` —
/// the same derivation `facade::image_backends` and the MACRO's `bridge_rmws`
/// both make. The crate name is then read out of the BOARD crate's own
/// `rmw-<x> = ["dep:<crate>"]` feature rather than from a table here. That is
/// deliberate: the macro carries its own three-arm `rmw_crate_ident`, and a
/// fourth copy would be the extra spelling that drifts. The board's answers
/// agree with the macro's today (`zenoh` → `nros_rmw_zenoh`, `cyclonedds` →
/// `nros_rmw_cyclonedds_sys`, `xrce` → `nros_rmw_xrce_cffi`), and if one ever
/// stops agreeing the build fails with the missing name rather than silently
/// linking the wrong backend.
///
/// **Message crates.** `nros sync` writes `<bringup>/nros-bridge.toml` with a
/// `[[register_type]] rust_path = "std_msgs::msg::Header"` per non-flat egress
/// type; the crate is the first segment, and it lives in `generated/`.
fn bridge_entry_deps(
    bringup_dir: &std::path::Path,
    entry_dir: &std::path::Path,
    nros_root: &std::path::Path,
    candidates: &[&str],
) -> Result<Vec<String>> {
    let Ok(text) = std::fs::read_to_string(bringup_dir.join("system.toml")) else {
        return Ok(Vec::new());
    };
    let Ok(doc) = text.parse::<toml::Value>() else {
        return Ok(Vec::new());
    };
    // Only a bringup that declares a bridge needs any of this.
    if doc
        .get("bridge")
        .and_then(|b| b.as_array())
        .is_none_or(|a| a.is_empty())
    {
        return Ok(Vec::new());
    }

    let mut out = Vec::new();

    // ---- backends, via the board crate's own feature table -----------------
    let board_dir = crate::builder::entry::macro_board_crate(candidates)
        .map(|k| nros_root.join("packages/boards").join(k));
    let board_manifest = board_dir
        .as_ref()
        .and_then(|d| std::fs::read_to_string(d.join("Cargo.toml")).ok())
        .and_then(|t| t.parse::<toml::Value>().ok());

    let mut rmws: Vec<String> = doc
        .get("domain")
        .and_then(|d| d.as_array())
        .map(|a| {
            a.iter()
                .filter_map(|d| d.get("rmw").and_then(|r| r.as_str()).map(str::to_string))
                .collect()
        })
        .unwrap_or_default();
    rmws.sort();
    rmws.dedup();

    for rmw in &rmws {
        let feature = format!("rmw-{rmw}");
        let Some(krate) = board_manifest
            .as_ref()
            .and_then(|m| m.get("features")?.get(&feature)?.as_array().cloned())
            .and_then(|arr| {
                arr.iter()
                    .filter_map(|v| v.as_str())
                    .find_map(|v| v.strip_prefix("dep:").map(str::to_string))
            })
        else {
            // The board does not carry this backend. Not an error here: a board
            // that cannot host a domain is the SYSTEM's problem and is reported
            // where the system resolves, with the board named.
            continue;
        };
        // `packages/rmw/<family>/<crate>` — located rather than assumed, since
        // the family directory is not the crate name (`nros-rmw-xrce-cffi`
        // lives under `xrce/`).
        let Some(path) = ["zenoh", "cyclonedds", "xrce", "uorb", "dds"]
            .iter()
            .map(|fam| nros_root.join("packages/rmw").join(fam).join(&krate))
            .find(|p| p.join("Cargo.toml").is_file())
        else {
            continue;
        };
        let rel = crate::builder::paths::relative_or_err(entry_dir, &path)
            .map_err(|e| eyre::eyre!("{e}"))?;
        out.push(format!("{krate} = {{ path = \"{rel}\" }}"));
    }

    // ---- message crates named by the generated bridge config ---------------
    if let Ok(cfg) = std::fs::read_to_string(bringup_dir.join("nros-bridge.toml"))
        && let Ok(cfg) = cfg.parse::<toml::Value>()
    {
        let rows = cfg
            .get("register_type")
            .and_then(|r| r.as_array())
            .cloned()
            .unwrap_or_default();

        // A cyclonedds `register_type` makes the macro emit
        // `::nros_rmw_cyclonedds::register::<M>()` — the WRAPPER crate, not the
        // `-sys` one the board feature pulls. `-sys` depends on it, so it is
        // already linked; what is missing is only the NAME in the entry's
        // scope. Keyed on the row's own `rmw` field, which is what the macro
        // filters on, rather than on "cyclonedds is among the domains": a
        // bridge can span cyclonedds and still register no non-flat type, and
        // then the macro emits no such call.
        if rows
            .iter()
            .any(|t| t.get("rmw").and_then(|r| r.as_str()) == Some("cyclonedds"))
            && let Some(path) = ["cyclonedds"]
                .iter()
                .map(|fam| {
                    nros_root
                        .join("packages/rmw")
                        .join(fam)
                        .join("nros-rmw-cyclonedds")
                })
                .find(|p| p.join("Cargo.toml").is_file())
        {
            let rel = crate::builder::paths::relative_or_err(entry_dir, &path)
                .map_err(|e| eyre::eyre!("{e}"))?;
            out.push(format!("nros-rmw-cyclonedds = {{ path = \"{rel}\" }}"));
        }

        let mut msg_crates: Vec<String> = rows
            .iter()
            .filter_map(|t| t.get("rust_path")?.as_str())
            .filter_map(|p| p.split("::").next().map(str::to_string))
            .collect();
        msg_crates.sort();
        msg_crates.dedup();
        for m in msg_crates {
            let path = bringup_dir
                .parent()
                .and_then(|p| p.parent())
                .map(|ws| ws.join("generated").join(&m));
            let Some(path) = path.filter(|p| p.join("Cargo.toml").is_file()) else {
                continue;
            };
            let rel = crate::builder::paths::relative_or_err(entry_dir, &path)
                .map_err(|e| eyre::eyre!("{e}"))?;
            out.push(format!(
                "{m} = {{ path = \"{rel}\", default-features = false }}"
            ));
        }
    }

    Ok(out)
}

/// Every cargo-driver entry directory of `bringup`, generated if need be.
///
/// The generated root lists all of them (see the call site): a member list that
/// depends on which image is being built makes `Cargo.lock` churn on every
/// switch, which `--locked` turns into a hard error.
///
/// `already` is the entry this build just generated — passed in rather than
/// regenerated so the caller's diagnostics and this list cannot disagree.
/// Images whose driver is west or idf.py contribute nothing: they are not cargo
/// members, and `framework_entry_dirs` excludes their hand-written packages.
#[allow(clippy::too_many_arguments)]
fn all_cargo_entry_dirs(
    bringups: &[(String, PathBuf, plan::ImageSet)],
    bringup: &str,
    root: &std::path::Path,
    catalog: &crate::orchestration::board_descriptor::BoardCatalog,
    image_has_non_rust: &dyn Fn(&crate::orchestration::image::ImageBlock, &std::path::Path) -> bool,
    nano_ros_root: Option<&std::path::Path>,
    already: Option<PathBuf>,
) -> Result<Vec<PathBuf>> {
    let mut out: Vec<PathBuf> = already.into_iter().collect();
    // `all_images` rather than reading `set.images` directly: it folds
    // `[image_defaults]` in, and a sibling generated WITHOUT that fold would
    // differ from the same entry generated as the build target — the same
    // entry, two contents, depending on which image was asked for.
    for (b, bringup_dir, id, image) in plan::all_images(bringups) {
        if b != bringup {
            continue;
        }
        let Ok(descriptor) = crate::orchestration::image::resolve_image_board(catalog, &id, &image)
        else {
            // An image whose board does not resolve is reported where it is
            // BUILT, with the context to explain it. Skipping here keeps a
            // sibling's misdeclaration from failing an unrelated build.
            continue;
        };
        let platform = descriptor.platform.kebab().to_string();
        if plan::driver_for(&platform, image_has_non_rust(&image, &bringup_dir)) != Driver::Cargo {
            continue;
        }
        let Some((dir, _facts)) = generate_entry(
            root,
            &bringup_dir,
            bringup,
            &id,
            &image,
            descriptor,
            &platform,
            nano_ros_root,
        )?
        else {
            continue;
        };
        if !out.contains(&dir) {
            out.push(dir);
        }
    }
    Ok(out)
}

/// Deploy tokens a package's entry declaration names, if it is an entry.
///
/// Two spellings, because the two languages declare it in different files:
/// Rust in `[package.metadata.nros.entry] deploy`, C/C++ in the
/// `nano_ros_add_executable(… DEPLOY <token>…)` call. Both are read; a package
/// that is not an entry yields an empty list.
fn entry_deploy_tokens(dir: &std::path::Path) -> Vec<String> {
    let mut out = Vec::new();
    if let Ok(text) = std::fs::read_to_string(dir.join("Cargo.toml"))
        && let Ok(doc) = text.parse::<toml::Value>()
        && let Some(d) = doc
            .get("package")
            .and_then(|p| p.get("metadata"))
            .and_then(|m| m.get("nros"))
            .and_then(|n| n.get("entry"))
            .and_then(|e| e.get("deploy"))
            .and_then(|d| d.as_str())
    {
        out.push(d.to_string());
    }
    if let Ok(text) = std::fs::read_to_string(dir.join("CMakeLists.txt")) {
        for line in text.lines() {
            // Comments explain the keyword constantly, so only a line whose
            // FIRST token is DEPLOY is a declaration.
            let t = line.trim();
            if t.starts_with('#') {
                continue;
            }
            if let Some(rest) = t.strip_prefix("DEPLOY") {
                out.extend(
                    rest.trim_end_matches(')')
                        .split_whitespace()
                        .filter(|w| !w.starts_with("${"))
                        .map(|w| w.trim_matches('"').to_string()),
                );
            }
        }
    }
    out
}

/// Entry packages that belong to a DIFFERENT board than the one being built.
///
/// RFC-0065's Problem statement names this as one of the four jobs a
/// hand-written root does by hand: *"which entries belong to the active
/// platform, by hand"*. phase-383 W8.b caught the emitter skipping it —
/// `autoware-safety-island` has three FreeRTOS entries (an536, posix, s32z2)
/// and a `freertos-posix` build listed all three.
///
/// An entry naming NO deploy token is kept: it has expressed no opinion, and
/// silently dropping a package is the failure this whole phase exists to
/// remove.
fn entries_for_other_boards(
    found: &crate::builder::discover::Discovered,
    board: &str,
    platform: &str,
) -> std::collections::BTreeSet<PathBuf> {
    let mut out = std::collections::BTreeSet::new();
    for pkg in &found.packages {
        let tokens = entry_deploy_tokens(&pkg.dir);
        if tokens.is_empty() {
            continue;
        }
        // The same three spellings `nano_ros_entry` itself accepts.
        let mine = tokens
            .iter()
            .any(|t| t == board || t == platform || t.is_empty());
        if !mine {
            out.insert(pkg.dir.clone());
        }
    }
    out
}

/// The build-tree coordinate for an image — RFC-0070 R2's vocabulary
/// (platform, rmw), never a new ad-hoc suffix.
///
/// Used by the cmake root (W4). The cargo root cannot use it: cargo pins its
/// workspace manifest to the workspace root, so there is no per-coordinate
/// cargo root to name.
fn coordinate(platform: &str, image: &crate::orchestration::image::ImageBlock) -> String {
    match image.rmw.as_deref() {
        Some(rmw) => format!("{platform}-{rmw}"),
        None => platform.to_string(),
    }
}

/// The coordinate for a CMAKE root, which must also separate BOARDS.
///
/// A CMake workspace is one board per configure — CMake pins the compiler at
/// the first configure and will not swap it on reconfigure, which is issue
/// 0391's whole subject. `examples/workspaces/c` declares `freertos`
/// (mps2-an385-freertos, cross arm-none-eabi) and `freertos_posix`
/// (freertos-posix, host cc) on the SAME platform token, so a platform-only
/// coordinate put two toolchains in one `build/freertos-zenoh/` and whichever
/// configured first would poison the cache for the other.
///
/// Cargo needs no such split: it separates by `--target` inside one dir, and
/// widening its coordinate would rename every generated entry directory for no
/// gain. So this is the cmake driver's own rule, not a change to
/// [`coordinate`].
fn cmake_coordinate(platform: &str, image: &crate::orchestration::image::ImageBlock) -> String {
    let base = coordinate(platform, image);
    match image.board.as_deref() {
        Some(b) if b != platform => format!("{base}-{}", b.replace(['/', '.'], "-")),
        _ => base,
    }
}

/// The west APPLICATION for an image — the entry package, not the bringup.
///
/// issue 0892. The bringup is `launch/ package.xml system.toml`; it has no
/// `CMakeLists.txt`, so `west build <bringup>` cannot work. The application is
/// the framework entry package for this image's board — the one thing in the
/// workspace that IS a Zephyr app, with `find_package(Zephyr)`, `prj.conf` and
/// its RMW overlays.
///
/// Matched by BOARD, through the same `[package.metadata.nros.entry] deploy`
/// resolution `framework_entry_dirs` uses, so a workspace with several zephyr
/// entries (`zephyr_entry`, `zephyr_entry_robot1`) picks the one whose deploy
/// resolves to the board the image asked for.
///
/// `None` when no entry claims the board: the caller then keeps its previous
/// target, and the failure is west's own "not an application", which names the
/// directory. Inventing an application here would be worse — RFC-0065 D5 is
/// explicit that a framework entry's authored Kconfig is not derivable.
/// The workspace package that IS the framework application for `image_id`.
///
/// Two ways a package says it serves a deploy target, and BOTH are load-bearing
/// because the two languages declare it in different files:
///
/// * Rust — `[package.metadata.nros.entry] deploy = "zephyr"` in `Cargo.toml`
/// * C/C++ — `nano_ros_add_executable(... DEPLOY zephyr)` in `CMakeLists.txt`
///
/// Reading only the Rust one was a silent Rust-only restriction: the C, C++,
/// mixed, realtime-c and realtime-cpp workspaces all have a `zephyr_entry`
/// declaring `DEPLOY zephyr`, none of them has a `Cargo.toml` for it, so all
/// five fell through to the bringup directory. That fallback is a real
/// directory, so nothing errored — west was simply pointed at the wrong place,
/// and the first thing to notice was a conf fragment "not found" in two paths
/// that were the same path twice.
///
/// Ambiguity is an ERROR, not a first match. `realtime-cpp` has `zephyr_entry`
/// and `fvp_entry`, both `DEPLOY zephyr`, both on `native_sim/native/64`, for
/// two images that differ in payload rather than board. Whichever one a scan
/// returns first is right half the time and silently wrong the other half, so
/// the image names it (`entry = "fvp_entry"`) and this refuses until it does.
fn west_application_dir(
    image_id: &str,
    image: &crate::orchestration::image::ImageBlock,
    descriptor: &crate::orchestration::board_descriptor::BoardDescriptor,
    found: &crate::builder::discover::Discovered,
    catalog: &crate::orchestration::board_descriptor::BoardCatalog,
) -> eyre::Result<Option<PathBuf>> {
    use crate::orchestration::board_descriptor::DeployResolution;

    // An explicit `entry` wins outright — it is the answer to the ambiguity
    // below, so it must not be re-derived or second-guessed.
    if let Some(name) = &image.entry {
        let hit = found.packages.iter().find(|p| {
            p.name.as_str() == name.as_str()
                || p.dir.file_name().and_then(|n| n.to_str()) == Some(name.as_str())
        });
        return match hit {
            Some(p) => Ok(Some(p.dir.clone())),
            None => Err(eyre::eyre!(
                "[image.{image_id}] names entry `{name}`, which is not a package in this \
                 workspace.\n  Known packages: {}",
                found
                    .packages
                    .iter()
                    .map(|p| p.name.clone())
                    .collect::<Vec<_>>()
                    .join(", ")
            )),
        };
    }

    let mut hits: Vec<PathBuf> = Vec::new();
    for pkg in &found.packages {
        let Some(deploy) = package_deploy_token(&pkg.dir) else {
            continue;
        };
        if let DeployResolution::Board(d) = catalog.resolve_deploy(&deploy)
            // Identity is the NAME SET: a descriptor has several spellings
            // (`native_sim/native/64`, `zephyr`, …) and two descriptors are the
            // same board when their name lists are.
            && d.names == descriptor.names
            && pkg.dir.join("CMakeLists.txt").is_file()
        {
            hits.push(pkg.dir.clone());
        }
    }

    match hits.len() {
        0 => Ok(None),
        1 => Ok(Some(hits.remove(0))),
        _ => Err(eyre::eyre!(
            "[image.{image_id}] matches {} entry packages, so the application cannot be \
             derived:\n{}\n  Name the one this image builds:\n\n    [image.{image_id}]\n    \
             entry = \"{}\"",
            hits.len(),
            hits.iter()
                .map(|h| format!("  {}", h.display()))
                .collect::<Vec<_>>()
                .join("\n"),
            hits[0]
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("<pkg>")
        )),
    }
}

/// The deploy token a package declares, from whichever file its language uses.
///
/// The CMake side is matched textually rather than by evaluating cmake: the
/// call is authored by hand in a fixed shape (`nano_ros_add_executable(` …
/// `DEPLOY <token>`), and the alternative — configuring the project to ask it —
/// costs a cmake run per candidate package during a plan that is supposed to be
/// cheap enough for `--dry-run`.
fn package_deploy_token(dir: &std::path::Path) -> Option<String> {
    if let Ok(text) = std::fs::read_to_string(dir.join("Cargo.toml"))
        && let Ok(doc) = text.parse::<toml::Value>()
        && let Some(d) = doc
            .get("package")
            .and_then(|p| p.get("metadata"))
            .and_then(|m| m.get("nros"))
            .and_then(|n| n.get("entry"))
            .and_then(|e| e.get("deploy"))
            .and_then(|d| d.as_str())
    {
        return Some(d.to_string());
    }
    let text = std::fs::read_to_string(dir.join("CMakeLists.txt")).ok()?;
    cmake_deploy_token(&text)
}

/// `DEPLOY <token>` out of a `nano_ros_add_executable`/`nano_ros_entry` call.
///
/// Scoped to the call, not to the file: `DEPLOY` also appears in comments
/// ("`nano_ros_node_register` has no DEPLOY → component-only") and a
/// file-wide regex would read those as declarations.
fn cmake_deploy_token(text: &str) -> Option<String> {
    let mut rest = text;
    while let Some(i) = rest
        .find("nano_ros_add_executable(")
        .or_else(|| rest.find("nano_ros_entry("))
    {
        let open = rest[i..].find('(')? + i;
        let close = rest[open..].find(')')? + open;
        let call = &rest[open + 1..close];
        let mut it = call.split_whitespace();
        while let Some(tok) = it.next() {
            if tok == "DEPLOY" {
                if let Some(v) = it.next() {
                    return Some(v.trim_matches('"').to_string());
                }
            }
        }
        rest = &rest[close + 1..];
    }
    None
}

/// `ZEPHYR_BASE` for a west build — the thing that actually makes `west build`
/// runnable.
///
/// **Not a `.west/` search.** Measured: with `ZEPHYR_BASE` exported, `west
/// build --help` works from any directory; without it, west says
/// `unknown command "build"; do you need to run this inside a workspace?` even
/// standing in the repo. So the requirement is a Zephyr, not a workspace —
/// which is also why Zephyr's FREESTANDING application works, and why every
/// zephyr fixture in this tree builds from the repo root rather than from
/// `zephyr-workspace/`.
///
/// The ladder is `scripts/build/west-fixtures.sh`'s, verbatim, and
/// `NROS_ZEPHYR_WORKSPACE` is the established spelling — 60 references across
/// the tree. An earlier version of this invented `NROS_WEST_WORKSPACE`, which
/// would have been a 61st name for one thing.
fn zephyr_base(root: &std::path::Path, flag: Option<&std::path::Path>) -> Option<PathBuf> {
    zephyr_base_with(
        flag,
        std::env::var_os("ZEPHYR_BASE"),
        std::env::var_os("NROS_ZEPHYR_WORKSPACE"),
        root,
    )
}

/// Is this directory a Zephyr tree rather than the workspace above one?
///
/// `Kconfig.zephyr` is the file Zephyr's own `find_package` machinery keys on,
/// so it is the marker rather than a name match — a workspace is free to check
/// Zephyr out under any directory name.
fn is_zephyr_tree(p: &std::path::Path) -> bool {
    p.join("Kconfig.zephyr").is_file()
}

/// [`zephyr_base`] with the flag and the two env reads passed IN, so the resolution is
/// testable without touching process-global state.
fn zephyr_base_with(
    flag: Option<&std::path::Path>,
    base: Option<std::ffi::OsString>,
    workspace: Option<std::ffi::OsString>,
    root: &std::path::Path,
) -> Option<PathBuf> {
    // `--zephyr-workspace` first: it is the only rung the user states in the
    // command itself, so an env left over from another project must not win
    // over what this invocation says.
    //
    // It names a WORKSPACE, like `$NROS_ZEPHYR_WORKSPACE` — but a user who
    // passes the `zephyr/` directory has named the same place by its other
    // name, and refusing that would be pedantry over a distinction only this
    // code cares about. So both resolve.
    if let Some(f) = flag {
        let nested = f.join("zephyr");
        if nested.is_dir() {
            return Some(nested);
        }
        if is_zephyr_tree(f) {
            return Some(f.to_path_buf());
        }
    }
    if let Some(b) = base {
        let p = PathBuf::from(b);
        if p.is_dir() {
            return Some(p);
        }
    }
    let mut candidates: Vec<PathBuf> = Vec::new();
    if let Some(w) = workspace {
        candidates.push(PathBuf::from(w));
    }
    candidates.push(root.join("zephyr-workspace"));
    if let Some(parent) = root.parent() {
        candidates.push(parent.join("nano-ros-workspace"));
        candidates.push(parent.join("nano-ros-workspace-4.4"));
    }
    candidates
        .into_iter()
        .map(|ws| ws.join("zephyr"))
        .find(|z| z.is_dir())
}

/// Package directories a cargo root must NOT list as members.
///
/// A west or ESP-IDF entry is built by its own framework; listing it makes a
/// host `cargo build` try to compile a Zephyr staticlib.
/// `examples/workspaces/rust` excludes exactly these by hand today.
fn framework_entry_dirs(
    found: &crate::builder::discover::Discovered,
    catalog: &crate::orchestration::board_descriptor::BoardCatalog,
) -> std::collections::BTreeSet<PathBuf> {
    entry_dirs_where(found, catalog, |d| !d.needs_generated_root())
}

/// Entry packages the generated cargo root must EXCLUDE.
///
/// A strictly smaller set than [`framework_entry_dirs`] — see
/// [`Driver::excluded_from_cargo_root`] for why the two questions differ.
fn cargo_excluded_entry_dirs(
    found: &crate::builder::discover::Discovered,
    catalog: &crate::orchestration::board_descriptor::BoardCatalog,
) -> std::collections::BTreeSet<PathBuf> {
    entry_dirs_where(found, catalog, Driver::excluded_from_cargo_root)
}

/// Entry package directories whose resolved driver satisfies `want`.
fn entry_dirs_where(
    found: &crate::builder::discover::Discovered,
    catalog: &crate::orchestration::board_descriptor::BoardCatalog,
    want: impl Fn(Driver) -> bool,
) -> std::collections::BTreeSet<PathBuf> {
    use crate::orchestration::board_descriptor::DeployResolution;
    let mut out = std::collections::BTreeSet::new();
    for pkg in &found.packages {
        let Ok(text) = std::fs::read_to_string(pkg.dir.join("Cargo.toml")) else {
            continue;
        };
        let Ok(doc) = text.parse::<toml::Value>() else {
            continue;
        };
        let deploy = doc
            .get("package")
            .and_then(|p| p.get("metadata"))
            .and_then(|m| m.get("nros"))
            .and_then(|n| n.get("entry"))
            .and_then(|e| e.get("deploy"))
            .and_then(|d| d.as_str());
        let Some(deploy) = deploy else { continue };
        if let DeployResolution::Board(d) = catalog.resolve_deploy(deploy)
            && want(plan::driver_for(d.platform.kebab(), false))
        {
            out.insert(pkg.dir.clone());
        }
    }
    out
}

fn native_handoff(
    driver: Driver,
    root: &std::path::Path,
    bringup_dir: &std::path::Path,
    board: &str,
    args: &Args,
) -> Handoff {
    match driver {
        Driver::West => {
            let mut a = vec!["build".to_string(), "-b".to_string(), board.to_string()];
            a.push(bringup_dir.display().to_string());
            a.extend(args.native_args.iter().cloned());
            Handoff::new("west", a).in_dir(root)
        }
        Driver::IdfPy => {
            let mut a = vec!["build".to_string()];
            a.extend(args.native_args.iter().cloned());
            Handoff::new("idf.py", a).in_dir(bringup_dir)
        }
        // Unreachable today — the caller bails before here for these two.
        Driver::Cargo | Driver::CMake => {
            let mut a = vec!["build".to_string()];
            a.extend(args.native_args.iter().cloned());
            Handoff::new(driver.program(), a).in_dir(root)
        }
    }
}

/// `(bringup name, bringup dir, its images)` per bringup — the shape every
/// stage after DISCOVER passes around.
type Bringups = Vec<(String, PathBuf, plan::ImageSet)>;

/// Read every bringup's `[image.*]`.
///
/// Bringups are derived from the packages stage 1 ALREADY found — a bringup is
/// simply a package carrying a `system.toml`. Deliberately not
/// `cmd::bringup::discover_bringups`, which walks one level of the workspace
/// root and so cannot see the canonical `<root>/src/<name>_bringup/` layout;
/// and deliberately not a second walk of our own, which would be a third
/// opinion about what a package is (issue 0809's class).
fn collect_images(
    packages: &[cargo_nano_ros::provider_scan::WorkspacePackage],
) -> Result<Bringups> {
    let (out, warnings) = collect_images_with_warnings(
        packages,
        crate::orchestration::image::deprecation_suppressed(),
    )?;
    for w in warnings {
        eprintln!("nros build: {w}");
    }
    Ok(out)
}

/// phase-398 W3 — resolve every `<depend>` a workspace declares, or fail.
///
/// The ladder is RFC-0062's, amended 2026-08-29: workspace package → generated
/// message → `[prereq.*]` key → ROS package (ament index) → error.
///
/// Fails by default. The alternative is what shipped for years: a name matching
/// nothing was dropped in silence, and the first run of this check over the
/// tree found three `<exec_depend>` entries naming packages that do not exist,
/// stale since a rename, in workspaces that build green.
///
/// `NROS_ALLOW_UNRESOLVED_DEPS=1` downgrades it to a warning. That is an escape
/// hatch for a tree mid-migration, not a mode — it names itself in the output
/// so a build that used it cannot be mistaken for one that passed.
fn check_declared_depends(
    root: &std::path::Path,
    packages: &[cargo_nano_ros::provider_scan::WorkspacePackage],
    nano_ros_root: Option<&std::path::Path>,
) -> Result<()> {
    use crate::orchestration::prereq_resolve as pr;

    let declared = pr::declared_depends(root);
    if declared.is_empty() {
        return Ok(());
    }

    let ws: std::collections::BTreeSet<String> = packages.iter().map(|p| p.name.clone()).collect();

    // Generated message crates: whatever `nros sync` has already written, plus
    // the core pre-generated set. A message package that has not been generated
    // YET must not read as unresolved — that is a `nros sync` away, not a
    // missing declaration.
    let mut generated: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
    for dir in [root.join("generated"), root.join("build/nros/generated")] {
        if let Ok(rd) = std::fs::read_dir(dir) {
            generated.extend(
                rd.flatten()
                    .map(|e| e.file_name().to_string_lossy().into_owned()),
            );
        }
    }
    if let Some(nr) = nano_ros_root
        && let Ok(rd) = std::fs::read_dir(nr.join("packages/interfaces"))
    {
        // The committed `nros-`prefixed msg crates are reached by their ROS
        // name in a `package.xml`, so strip the prefix the crate carries.
        generated.extend(rd.flatten().map(|e| {
            let n = e.file_name().to_string_lossy().into_owned();
            n.strip_prefix("nros-").unwrap_or(&n).to_string()
        }));
    }

    let prereq_map: std::collections::BTreeMap<String, crate::orchestration::sdk_index::PrereqDep> =
        nano_ros_root
            .map(|nr| nr.join("nros-sdk-index.toml"))
            .filter(|p| p.is_file())
            .and_then(|p| crate::orchestration::sdk_index::SdkIndex::load(&p).ok())
            .map(|i| i.prereqs())
            .unwrap_or_default();
    let prereq_keys: std::collections::BTreeSet<String> = prereq_map.keys().cloned().collect();

    let ros = pr::ros_packages();
    // Off-ROS safety: a package's own buildtool is satisfied by the builder that
    // is building it. Without this, adding the `<buildtool_depend>` that rosdep
    // expects would hard-fail every host with no `AMENT_PREFIX_PATH`.
    let self_buildtools = pr::self_satisfied_buildtools(root);

    let mut unresolved: Vec<pr::Unresolved> = Vec::new();
    // phase-422 W8 — a dep that RESOLVES but names infrastructure. `role` says
    // who may name a key; `infra` (emulators, cross toolchains, debug probes)
    // comes from WHERE you deploy, not from what your code needs, so
    // `<depend>qemu-system-arm</depend>` is a category error rather than a
    // missing package. Refused separately because the remedy is different:
    // nothing to install, the declaration itself is wrong.
    //
    // Scoped to `infra` DELIBERATELY. `workspace` and `vendor` are not
    // obviously category errors from a user's side — a package that builds
    // against a vendored source tree naming it is arguable — so refusing them
    // would risk more than it buys. Measured before landing: ZERO packages in
    // this tree name a non-`package` key, so this breaks nothing here and the
    // blast radius is entirely out-of-tree.
    let mut wrong_role: Vec<(pr::Unresolved, &'static str)> = Vec::new();
    for (name, files) in &declared {
        let res = pr::classify(name, &ws, &generated, &prereq_keys, &ros, &self_buildtools);
        if res == pr::Resolution::Unknown {
            unresolved.push(pr::Unresolved {
                name: name.clone(),
                declared_by: files.clone(),
            });
        } else if res == pr::Resolution::Prereq
            && let Some(dep) = prereq_map.get(name)
            && dep.role == crate::orchestration::sdk_index::PrereqRole::Infra
        {
            wrong_role.push((
                pr::Unresolved {
                    name: name.clone(),
                    declared_by: files.clone(),
                },
                "infra",
            ));
        }
    }

    if !wrong_role.is_empty() {
        let mut m = format!(
            "{} <depend> name(s) declare INFRASTRUCTURE, not a dependency:\n",
            wrong_role.len()
        );
        for (u, role) in &wrong_role {
            m.push_str(&format!(
                "  {} (role = {role}) — declared by {}\n",
                u.name,
                u.declared_by.join(", ")
            ));
        }
        m.push_str(
            "\nA package.xml declares what the package's CONTENT needs. An emulator, \
             cross toolchain or debug probe comes from WHERE the package deploys — \
             declare that instead:\n\
             \n  <export><nano_ros deploy=\"<platform>\" board=\"<board>\"/></export>\n\
             \nand provision it with `nros setup <board>` (see `nros setup --workspace`, \
             which reports what a workspace needs).\n\
             \n  NROS_ALLOW_INFRA_DEPS=1  to continue with a warning.",
        );
        if std::env::var_os("NROS_ALLOW_INFRA_DEPS").is_some() {
            eprintln!("nros build: WARNING (NROS_ALLOW_INFRA_DEPS=1): {m}");
        } else {
            eyre::bail!("{m}")
        }
    }

    if unresolved.is_empty() {
        return Ok(());
    }

    let mut msg = format!(
        "{} <depend> name(s) resolve to nothing:\n",
        unresolved.len()
    );
    for u in &unresolved {
        msg.push_str(&format!(
            "  {} — declared by {}\n",
            u.name,
            u.declared_by.join(", ")
        ));
    }
    msg.push_str(
        "\nEach must be one of: a package in this workspace, a message package \
         `nros sync` generates, a `[prereq.*]` key in nros-sdk-index.toml whose \
         `role` is `package`, or a package the ambient ROS install provides \
         (source its setup.bash so AMENT_PREFIX_PATH is set).\n\
         \nNOTE the role: a key for an emulator, cross toolchain or vendored \
         source tree is NOT declarable here — that comes from the deploy target \
         in `<export><nano_ros deploy=.. board=../></export>`. Adding a \
         `[prereq.*]` entry to make this resolve is the wrong fix if the thing \
         is infrastructure.\n\
         \n  NROS_ALLOW_UNRESOLVED_DEPS=1  to continue with a warning.",
    );

    if std::env::var_os("NROS_ALLOW_UNRESOLVED_DEPS").is_some() {
        eprintln!("nros build: WARNING (NROS_ALLOW_UNRESOLVED_DEPS=1): {msg}");
        return Ok(());
    }
    eyre::bail!("{msg}")
}

/// [`collect_images`], with the deprecation warnings RETURNED rather than
/// printed and the suppression flag passed in.
///
/// Split for the reason the lint's own doc comment gives for taking
/// `suppressed` as a parameter: a warning that can only be observed on stderr,
/// under an ambient env var, cannot be tested deterministically — and the
/// previous shape is exactly how W1.f shipped a correct, well-tested lint that
/// no production path called. The test below asserts the WIRING, not the lint.
fn collect_images_with_warnings(
    packages: &[cargo_nano_ros::provider_scan::WorkspacePackage],
    suppressed: bool,
) -> Result<(Bringups, Vec<String>)> {
    let mut warnings: Vec<String> = Vec::new();
    let mut out = Vec::new();
    for pkg in packages {
        let system_toml = pkg.dir.join("system.toml");
        if !system_toml.is_file() {
            continue;
        }
        let text = std::fs::read_to_string(&system_toml)
            .wrap_err_with(|| format!("reading {}", system_toml.display()))?;
        let sys: crate::orchestration::cargo_metadata_schema::SystemToml =
            toml::from_str(&text).wrap_err_with(|| format!("parsing {}", system_toml.display()))?;

        // W1.f's deprecation lint, ACTUALLY REACHED.
        //
        // It shipped with four passing tests and no production caller, so no
        // user has ever seen the warning it promised ("warn on every invocation
        // while still working"). That is the shape `check-no-vacuous-tests`
        // exists for, one level up: the function is correct and its tests are
        // honest, and the feature is still absent because nothing calls it.
        //
        // Here rather than in `nros doctor` alone, because this is where a
        // build reads the declaration it is warning about — the user is looking
        // at the output already, and it costs one pass over a table that is
        // typically empty.
        //
        // Field PRESENCE comes from the raw document, not from `sys.deploy`:
        // `DeployTarget` is upstream's typed struct, so an absent key and a key
        // set to its default are the same value once parsed, and a lint about
        // "you wrote this key" must see what was written.
        if let Ok(raw) = text.parse::<toml::Value>()
            && let Some(deploys) = raw.get("deploy").and_then(|d| d.as_table())
        {
            let present: std::collections::BTreeMap<String, Vec<String>> = deploys
                .iter()
                .filter_map(|(id, block)| {
                    let t = block.as_table()?;
                    Some((id.clone(), t.keys().cloned().collect()))
                })
                .collect();
            for w in crate::orchestration::image::deprecated_deploy_build_field_warnings(
                &present, &sys.image, suppressed,
            ) {
                warnings.push(format!("{}: {w}", system_toml.display()));
            }
        }

        out.push((
            pkg.name.clone(),
            pkg.dir.clone(),
            plan::ImageSet {
                images: sys.image.clone(),
                defaults: sys.image_defaults.clone(),
                default_images: sys.system.default_images.clone(),
            },
        ));
    }
    Ok((out, warnings))
}

#[cfg(test)]
mod facade_absence_tests {
    use super::*;

    fn bringup_with(system_toml: &str) -> tempfile::TempDir {
        let d = tempfile::tempdir().expect("tempdir");
        std::fs::write(d.path().join("system.toml"), system_toml).expect("write system.toml");
        d
    }

    /// Every REQUIRED field of `[system]`. `domain_id` is one of them, and the
    /// first draft of these tests omitted it: `SystemToml` then failed to parse,
    /// `declared_capabilities` returned empty for both spellings, and the tests
    /// looked like they had caught a production bug. They had caught a bad
    /// fixture. Keep this complete.
    const BASE: &str = r#"
[system]
name = "t"
rmw = "zenoh"
domain_id = 0
"#;

    /// The form that produced the `host-tests` red: capabilities declared with
    /// the phase-261 `[system].features` list and no typed block in sight.
    /// Reading only the typed blocks would return empty here and re-open the bug.
    #[test]
    fn phase_261_features_list_is_seen() {
        let d = bringup_with(&format!(
            "{BASE}features = [\"param_services\", \"lifecycle\"]\n"
        ));
        let mut got = declared_capabilities(d.path());
        got.sort_unstable();
        assert_eq!(got, vec!["lifecycle", "param_services"]);
    }

    /// The deprecated typed block still counts — both spellings flip the axis.
    #[test]
    fn typed_block_is_seen() {
        let d = bringup_with(&format!("{BASE}\n[param_services]\nenabled = true\n"));
        assert_eq!(declared_capabilities(d.path()), vec!["param_services"]);
    }

    /// A system that declares nothing keeps the WARNING path: a missing facade
    /// there costs the RMW and the ROS edition, both of which have defaults, and
    /// escalating it would fail builds that are documented to tolerate it.
    #[test]
    fn no_capabilities_is_not_fatal() {
        let d = bringup_with(BASE);
        assert!(declared_capabilities(d.path()).is_empty());
    }

    /// Absent or unparseable input must NOT read as "capabilities declared".
    /// This predicate only escalates an existing warning, so silence is the
    /// safe answer; the malformed file is someone else's error to report.
    #[test]
    fn unreadable_system_is_empty_not_fatal() {
        let empty = tempfile::tempdir().expect("tempdir");
        assert!(declared_capabilities(empty.path()).is_empty());

        let junk = bringup_with("this is not toml {{{");
        assert!(declared_capabilities(junk.path()).is_empty());
    }
}

#[cfg(test)]
mod deprecation_wiring_tests {
    use super::*;

    /// The lint W1.f promised must be REACHED by the build path.
    ///
    /// It shipped with four passing unit tests and no production caller, so the
    /// warning it specified ("warn on every invocation while still working")
    /// reached nobody: `nros build` was silent and `nros doctor` never grew the
    /// check either. The unit tests could not catch that — they call the lint
    /// directly, which is precisely what nothing else did.
    ///
    /// So this asserts the WIRING rather than the lint: parse a bringup through
    /// the real `collect_images` path and require the warning to come back.
    fn pkg(dir: &std::path::Path) -> Vec<cargo_nano_ros::provider_scan::WorkspacePackage> {
        vec![cargo_nano_ros::provider_scan::WorkspacePackage {
            name: "demo_bringup".to_string(),
            dir: dir.to_path_buf(),
            depends: Default::default(),
        }]
    }

    fn write_system(dir: &std::path::Path, body: &str) {
        std::fs::create_dir_all(dir).expect("mkdir");
        std::fs::write(dir.join("system.toml"), body).expect("write");
    }

    #[test]
    fn a_deploy_build_field_warns_through_the_build_path() {
        let d = std::env::temp_dir().join(format!("nros-w10b-{}-{}", std::process::id(), line!()));
        write_system(
            &d,
            "[system]\nname = \"s\"\nrmw = \"zenoh\"\ndomain_id = 0\n\n\
             [deploy.legacy]\nkind = \"embedded\"\nboard = \"mps2-an385-freertos\"\n",
        );

        let (_, warnings) = collect_images_with_warnings(&pkg(&d), false).expect("collects");
        assert_eq!(warnings.len(), 1, "expected one warning, got {warnings:?}");
        assert!(
            warnings[0].contains("[deploy.legacy]") && warnings[0].contains("board"),
            "warning must name the block and the field: {}",
            warnings[0]
        );

        // Suppression reaches the same path.
        let (_, quiet) = collect_images_with_warnings(&pkg(&d), true).expect("collects");
        assert!(quiet.is_empty(), "suppressed run must be silent: {quiet:?}");

        std::fs::remove_dir_all(&d).ok();
    }

    /// A block that HAS its `[image.*]` is migrated, and a placement-only block
    /// was never in scope — `[deploy.*]` keeps `kind`/`nodes`/`launch` and is
    /// not being retired. Without this the test above would pass on a lint that
    /// warned about everything.
    #[test]
    fn migrated_and_placement_only_blocks_stay_silent() {
        let d = std::env::temp_dir().join(format!("nros-w10b-{}-{}", std::process::id(), line!()));
        write_system(
            &d,
            "[system]\nname = \"s\"\nrmw = \"zenoh\"\ndomain_id = 0\n\n\
             [deploy.migrated]\nkind = \"embedded\"\nboard = \"mps2-an385-freertos\"\n\n\
             [deploy.placement]\nkind = \"self\"\nnodes = [\"a\"]\n\n\
             [image.migrated]\nboard = \"mps2-an385-freertos\"\n",
        );

        let (_, warnings) = collect_images_with_warnings(&pkg(&d), false).expect("collects");
        assert!(warnings.is_empty(), "expected silence, got {warnings:?}");

        std::fs::remove_dir_all(&d).ok();
    }
}

#[cfg(test)]
mod zephyr_base_tests {
    use super::*;

    fn zdir(base: &std::path::Path, name: &str) -> PathBuf {
        let ws = base.join(name);
        std::fs::create_dir_all(ws.join("zephyr")).expect("mkdir");
        ws
    }

    /// The ladder is `west-fixtures.sh`'s, and what it produces is a
    /// ZEPHYR_BASE — not a `.west/` directory. Measured: with `ZEPHYR_BASE`
    /// exported, `west build` runs from anywhere; without it, west refuses even
    /// inside the repo. That is also why a FREESTANDING application works
    /// (issue 0892 / RFC-0085).
    #[test]
    fn the_ladder_resolves_a_zephyr_not_a_workspace_marker() {
        let base = std::env::temp_dir().join(format!("nros-zb-{}-{}", std::process::id(), line!()));
        let root = base.join("nano-ros");
        std::fs::create_dir_all(&root).expect("mkdir");

        // Nothing anywhere.
        assert_eq!(zephyr_base_with(None, None, None, &root), None);

        // In-repo `zephyr-workspace/` — the common contributor layout.
        let inrepo = zdir(&root, "zephyr-workspace");
        assert_eq!(
            zephyr_base_with(None, None, None, &root),
            Some(inrepo.join("zephyr"))
        );

        // `NROS_ZEPHYR_WORKSPACE` — the ESTABLISHED spelling (60 references in
        // the tree), not a new one — outranks the in-repo default.
        let explicit = zdir(&base, "elsewhere");
        assert_eq!(
            zephyr_base_with(None, None, Some(explicit.clone().into_os_string()), &root),
            Some(explicit.join("zephyr")),
        );

        // An already-exported ZEPHYR_BASE wins outright: the user has chosen.
        let direct = base.join("chosen");
        std::fs::create_dir_all(&direct).expect("mkdir");
        assert_eq!(
            zephyr_base_with(
                None,
                Some(direct.clone().into_os_string()),
                Some(explicit.into_os_string()),
                &root
            ),
            Some(direct),
        );

        std::fs::remove_dir_all(&base).ok();
    }

    /// A pointer at something that is not a Zephyr must not be honoured — it is
    /// a typo, and taking it moves the failure into west with a worse message.
    #[test]
    fn a_pointer_without_a_zephyr_dir_is_not_honoured() {
        let base = std::env::temp_dir().join(format!("nros-zb-{}-{}", std::process::id(), line!()));
        let root = base.join("nano-ros");
        let empty = base.join("not-a-workspace");
        std::fs::create_dir_all(&root).expect("mkdir");
        std::fs::create_dir_all(&empty).expect("mkdir");

        assert_eq!(
            zephyr_base_with(None, None, Some(empty.clone().into_os_string()), &root),
            None,
            "a workspace with no zephyr/ is not a workspace",
        );
        assert_eq!(
            zephyr_base_with(None, Some(empty.join("nope").into_os_string()), None, &root),
            None,
            "a ZEPHYR_BASE that does not exist is not a Zephyr",
        );
        std::fs::remove_dir_all(&base).ok();
    }
}

#[cfg(test)]
mod west_application_tests {
    use super::*;

    /// The CMake half of the deploy declaration must be READ, not just the
    /// Cargo half.
    ///
    /// Reading only `Cargo.toml` made the west application resolver silently
    /// Rust-only: five workspaces (`c`, `cpp`, `mixed`, `realtime-c`,
    /// `realtime-cpp`) declare their entry with
    /// `nano_ros_add_executable(... DEPLOY zephyr)` and have no `Cargo.toml`
    /// for it, so every one of them fell through to the bringup directory —
    /// a real directory, so nothing errored and west was simply pointed at the
    /// wrong tree.
    #[test]
    fn a_cmake_entry_declares_its_deploy_token() {
        let text = r#"
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
nano_ros_add_executable(zephyr_entry
    BOARD   zephyr
    LANG    c
    DEPLOY  zephyr)
"#;
        assert_eq!(cmake_deploy_token(text).as_deref(), Some("zephyr"));
    }

    /// `DEPLOY` in prose is not a declaration.
    ///
    /// `examples/workspaces/cpp/src/zephyr_entry/CMakeLists.txt` contains the
    /// comment "nano_ros_node_register has no DEPLOY → component-only", one
    /// line above the real call. A file-wide regex reads that as the answer;
    /// scoping to the call arguments is what makes the difference, so it is
    /// what gets tested.
    #[test]
    fn deploy_in_a_comment_is_not_a_declaration() {
        let text = r#"
# nano_ros_node_register has no DEPLOY -> component-only; the sidecar links them.
nano_ros_add_executable(entry
    BOARD zephyr
    DEPLOY zephyr)
"#;
        assert_eq!(cmake_deploy_token(text).as_deref(), Some("zephyr"));

        // …and a file with ONLY the comment declares nothing.
        let comment_only = "# has no DEPLOY zephyr, component-only\n";
        assert_eq!(cmake_deploy_token(comment_only), None);
    }

    /// A file with no entry call at all resolves to nothing rather than
    /// panicking on the `?` chain.
    #[test]
    fn a_plain_cmakelists_declares_nothing() {
        assert_eq!(
            cmake_deploy_token("project(foo)\nadd_executable(a a.c)\n"),
            None
        );
    }

    /// A quoted token is the same token.
    #[test]
    fn a_quoted_deploy_token_is_unquoted() {
        let text = "nano_ros_entry(e\n    DEPLOY \"zephyr\")\n";
        assert_eq!(cmake_deploy_token(text).as_deref(), Some("zephyr"));
    }
}

#[cfg(test)]
mod zephyr_workspace_flag_tests {
    use super::*;

    /// `--zephyr-workspace` outranks both environment variables.
    ///
    /// An env is ambient: it survives a shell, it is not visible in the command
    /// that ran, and one left over from another project would otherwise decide
    /// this build. What the invocation says has to win over what the shell
    /// remembers.
    #[test]
    fn the_flag_outranks_both_env_vars() {
        let tmp = tempfile::tempdir().unwrap();
        let flagged = tmp.path().join("flagged");
        let env_ws = tmp.path().join("from-env");
        std::fs::create_dir_all(flagged.join("zephyr")).unwrap();
        std::fs::create_dir_all(env_ws.join("zephyr")).unwrap();
        let stale_base = tmp.path().join("stale");
        std::fs::create_dir_all(&stale_base).unwrap();

        assert_eq!(
            zephyr_base_with(
                Some(&flagged),
                Some(stale_base.into_os_string()),
                Some(env_ws.into_os_string()),
                tmp.path(),
            ),
            Some(flagged.join("zephyr")),
        );
    }

    /// Passing the `zephyr/` directory itself resolves too.
    ///
    /// The flag names a WORKSPACE, but "the directory containing zephyr" and
    /// "the Zephyr directory" are one place under two descriptions, and
    /// confusing them is the commonest way to get this wrong. Refusing would
    /// be pedantry about a distinction only the resolver cares about.
    #[test]
    fn the_flag_also_accepts_the_zephyr_tree_itself() {
        let tmp = tempfile::tempdir().unwrap();
        let zephyr = tmp.path().join("ws").join("zephyr");
        std::fs::create_dir_all(&zephyr).unwrap();
        // `Kconfig.zephyr` is the marker, not the directory NAME — a workspace
        // may check Zephyr out anywhere.
        std::fs::write(zephyr.join("Kconfig.zephyr"), "").unwrap();

        assert_eq!(
            zephyr_base_with(Some(&zephyr), None, None, tmp.path()),
            Some(zephyr),
        );
    }

    /// A directory that is neither a workspace nor a Zephyr resolves to
    /// nothing, so the caller reports the miss instead of running west against
    /// a path that cannot work.
    #[test]
    fn a_flag_naming_neither_resolves_to_nothing() {
        let tmp = tempfile::tempdir().unwrap();
        let empty = tmp.path().join("empty");
        std::fs::create_dir_all(&empty).unwrap();
        assert_eq!(zephyr_base_with(Some(&empty), None, None, tmp.path()), None);
    }
}

/// phase-420 W7 — the selection verbs, asserted where they are WIRED.
///
/// `builder::discover` unit-tests the filter itself. These assert the two
/// things a unit test of a pure function cannot: that clap spells the flags the
/// way colcon does, and that `plan_builds` actually applies them. This crate
/// has shipped a lint with four passing unit tests and no production caller
/// before (`deprecation_wiring_tests`, below), which is why the wiring gets its
/// own assertions rather than being assumed.
#[cfg(test)]
mod selection_wiring_tests {
    use super::*;
    use clap::Parser;

    fn args_for(root: &std::path::Path, select: &[&str], up_to: &[&str]) -> Args {
        Args {
            images: Vec::new(),
            workspace: Some(root.to_path_buf()),
            nano_ros_path: None,
            zephyr_workspace: None,
            all: false,
            dry_run: true,
            offline: true,
            packages_select: select.iter().map(|s| (*s).to_string()).collect(),
            packages_up_to: up_to.iter().map(|s| (*s).to_string()).collect(),
            native_args: Vec::new(),
        }
    }

    fn pkg(root: &std::path::Path, name: &str, depends: &[&str]) {
        let dir = root.join("src").join(name);
        std::fs::create_dir_all(&dir).expect("mkdir");
        let deps: String = depends
            .iter()
            .map(|d| format!("  <depend>{d}</depend>\n"))
            .collect();
        std::fs::write(
            dir.join("package.xml"),
            format!(
                "<?xml version=\"1.0\"?>\n<package format=\"3\">\n  \
                 <name>{name}</name>\n  <version>0.0.0</version>\n  \
                 <description>t</description>\n  \
                 <maintainer email=\"a@b.c\">m</maintainer>\n  \
                 <license>Apache-2.0</license>\n{deps}</package>\n"
            ),
        )
        .expect("write package.xml");
    }

    /// colcon's spelling, and both flags taking several names.
    #[test]
    fn the_flags_parse_with_colcons_spelling() {
        let a = Args::try_parse_from([
            "build",
            "--packages-select",
            "talker_pkg",
            "msgs_pkg",
            "--packages-up-to",
            "entry",
        ])
        .expect("parses");
        assert_eq!(a.packages_select, vec!["talker_pkg", "msgs_pkg"]);
        assert_eq!(a.packages_up_to, vec!["entry"]);
        assert!(a.images.is_empty(), "no image was named: {:?}", a.images);
    }

    /// A selection composes with the flags the verb already has — here the
    /// positional image, which must not be swallowed by the multi-valued flag.
    #[test]
    fn an_image_argument_survives_beside_a_selection() {
        let a = Args::try_parse_from(["build", "--packages-up-to", "entry", "--", "-j4"])
            .expect("parses");
        assert_eq!(a.packages_up_to, vec!["entry"]);
        assert_eq!(a.native_args, vec!["-j4"]);

        let b = Args::try_parse_from(["build", "native", "--packages-select", "talker_pkg"])
            .expect("parses");
        assert_eq!(b.images, vec!["native"]);
        assert_eq!(b.packages_select, vec!["talker_pkg"]);
    }

    /// The wiring: an unknown name must be refused by `plan_builds` itself.
    ///
    /// It is also refused EARLY — before the board catalog, which needs a
    /// nano-ros checkout this temp workspace does not have. If the selection
    /// ran later, this test would see "no nano-ros checkout found" instead, so
    /// the assertion doubles as the ordering check.
    #[test]
    fn plan_builds_refuses_an_unknown_selected_package() {
        let tmp = tempfile::tempdir().expect("tempdir");
        pkg(tmp.path(), "talker_pkg", &[]);
        pkg(tmp.path(), "msgs_pkg", &[]);

        let e = plan_builds(&args_for(tmp.path(), &["talkr_pkg"], &[]))
            .expect_err("an unknown package must not be warned past");
        let msg = format!("{e:#}");
        assert!(msg.contains("no such package"), "{msg}");
        assert!(msg.contains("talker_pkg"), "lists what exists: {msg}");
    }

    /// The other wiring direction: an incomplete `--packages-select` is refused
    /// by `plan_builds`, not left to fail later as a missing artifact.
    #[test]
    fn plan_builds_refuses_a_selection_that_drops_a_dependency() {
        let tmp = tempfile::tempdir().expect("tempdir");
        pkg(tmp.path(), "entry", &["talker_pkg"]);
        pkg(tmp.path(), "talker_pkg", &[]);

        let e = plan_builds(&args_for(tmp.path(), &["entry"], &[]))
            .expect_err("an open selection must not build");
        let msg = format!("{e:#}");
        assert!(msg.contains("entry needs talker_pkg"), "{msg}");
        assert!(msg.contains("--packages-up-to entry"), "{msg}");
    }

    /// And a well-formed selection is NOT refused here — it passes stage 1b and
    /// fails later, on the image, which is a different message. Without this,
    /// the two tests above would pass on a `select` that refused everything.
    #[test]
    fn a_closed_selection_passes_stage_1b() {
        let tmp = tempfile::tempdir().expect("tempdir");
        pkg(tmp.path(), "entry", &["talker_pkg"]);
        pkg(tmp.path(), "talker_pkg", &[]);

        let e = plan_builds(&args_for(tmp.path(), &[], &["entry"]))
            .expect_err("this workspace declares no image");
        let msg = format!("{e:#}");
        assert!(
            !msg.contains("no such package") && !msg.contains("needs talker_pkg"),
            "the selection is closed and must not be the complaint: {msg}"
        );
        assert!(msg.contains("[image.*]"), "{msg}");
    }
}
