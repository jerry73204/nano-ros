//! Build script for `nros-rmw-xrce-cffi`.
//!
//! Compiles the K.2.0–K.2.4 C backend (`packages/rmw/xrce/nros-rmw-xrce/src/*.c`)
//! plus the vendored micro-XRCE-DDS-Client + micro-CDR sources directly
//! into a single static archive, then exposes the
//! `nros_rmw_xrce_register` symbol to the Rust side via `extern "C"`.
//!
//! Issue 1068 — this script OWNS NO SOURCE LIST. The set of C files compiled
//! here is read from `packages/rmw/xrce/xrce-sources.txt`, and
//! `nros-rmw-xrce/CMakeLists.txt` (the C/C++ consumers' entry point) reads the
//! same file. There used to be two hand-copied lists held together by a
//! comment; they drifted, and the drift was silent — `NROS_LINK_IP=0` dropped
//! `udp_transport{,_posix}.c` on this side and not on the CMake side, so a
//! serial-only XRCE node could not be built from C or C++ and the symptom was
//! a bigger image rather than an error. Adding a source file to only one lane
//! is now impossible: neither lane names one.
//!
//! phase-321 W1.d — the old comment named `packages/rmw/xrce/xrce-sys/build.rs`
//! as the mirror; that crate is deleted and only the DIRECTORY survives,
//! because it hosts the micro-XRCE-DDS-Client and micro-CDR submodules both
//! lanes compile from.

use std::{env, fs, path::PathBuf};

// NROS-XRCE-COMPILED-TREES-BEGIN
/// The `xrce-sources.txt` trees THIS lane compiles — phase-420 W9 step 4.
///
/// All three, and that is the point: the archive this script produces is the
/// one both lanes link, so it has to hold every TU. `nros-rmw-xrce/CMakeLists.txt`
/// declares the complementary set (`_xrce_compiled_trees`, `backend` only) and
/// LINKS this archive for the vendored halves instead of compiling them a second
/// time. Gate: `just check xrce-one-vendored-compile`.
///
/// Load-bearing, not a comment: the row loop asserts membership, so dropping a
/// tree here fails the build rather than silently shrinking the archive.
///
/// Why the split cannot go the other way — measured, phase-420 W9 step 4: the
/// backend TU `platform_aliases.c` DEFINES `uxr_millis`/`uxr_nanos`, which the
/// vendored `uxr` TUs call, while the backend TUs call the vendored session API.
/// The two halves are mutually recursive at link time, so they cannot be two
/// archives without `--start-group`, which rustc does not emit. One archive, or
/// a link that resolves by luck of ordering.
const COMPILED_TREES: &[&str] = &["uxr", "ucdr", "backend"];
// NROS-XRCE-COMPILED-TREES-END

/// The name of the pointer file this script writes into `OUT_DIR`.
///
/// `nros-rmw-xrce/CMakeLists.txt` links the archive cc-rs produces here, and
/// the archive's NAME and the generated headers' LOCATION are facts of this
/// script — so this script states them, in the one place a consumer can find
/// them from `OUT_DIR` alone. `just check rmw-xrce` locates `OUT_DIR` itself,
/// out of `cargo build --message-format=json`'s `build-script-executed`
/// message; it is NOT globbed, because `<hash>` is not predictable and taking
/// the first match of a glob is issue 0500's defect.
///
/// NOT `cargo::metadata=` — measured 2026-09-05: with no `links` key that
/// channel's `env` array comes back EMPTY in the JSON, and a `links` key buys
/// nothing here because the consumer is a CMake project, which can read no
/// `DEP_<LINKS>_*` at all.
const VENDOR_BUILD_POINTER: &str = "nros-xrce-vendor-build.txt";

// phase-420 W9 — this script OWNS NO CONFIGURATION VALUE either. The MTU
// defaults that used to live here as `XRCE_TRANSPORT_MTU_DEFAULT` /
// `XRCE_SERIAL_MTU_DEFAULT`, every other `@TOKEN@` the two upstream
// `config.h.in` templates take, every `#cmakedefine` toggle, and every knob's
// minimum are read from `packages/rmw/xrce/xrce-config.txt`, which
// `nros-rmw-xrce/CMakeLists.txt` reads too. Same remedy as the source list one
// file over (issue 1068): the values were restated in both lanes, and the
// lanes had already stopped agreeing.

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    // phase-321 W2.d — FOUR parents, not three. The crate sits at
    // packages/rmw/xrce/nros-rmw-xrce-cffi/, one level deeper than the old
    // packages/xrce/nros-rmw-xrce-cffi/. With three the walk stopped at
    // `packages/` and every vendored path came out doubled
    // (`<repo>/packages/packages/rmw/xrce/...`). A `.parent()` chain is a
    // relative path that no grep for "../" can find — only a build does.
    let workspace = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .and_then(|p| p.parent())
        .and_then(|p| p.parent())
        .expect("workspace root")
        .to_path_buf();
    let xrce_sys = workspace.join("packages/rmw/xrce/xrce-sys");
    let xrce_c = workspace.join("packages/rmw/xrce/nros-rmw-xrce");
    let microcdr = xrce_sys.join("micro-cdr");
    let microxrce = xrce_sys.join("micro-xrce-dds-client");

    // Phase 145.4 — source-list drift / submodule-presence gate (mirrors the
    // zpico-sys 136.6 gate). The vendored uxr / micro-cdr C sources come from
    // git submodules; a missing checkout or an upstream bump that renamed a
    // source dir would otherwise surface as a confusing cc-rs "file not found"
    // mid-compile. Verify each vendored root resolves to a directory with .c
    // files (or subdirs) up front, with a clear init hint, and emit
    // rerun-if-changed so a submodule bump retriggers the build.
    for (label, root, hint) in [
        (
            "micro-xrce-dds-client",
            microxrce.join("src/c"),
            // #0390 — lead with the `nros setup` vocabulary a CLI-provisioned
            // user has; the label IS the `[source.*]` name. git line kept as the
            // underlying mechanism.
            "nros setup --source micro-xrce-dds-client  \
             (or: git submodule update --init packages/rmw/xrce/xrce-sys/micro-xrce-dds-client)",
        ),
        (
            "micro-cdr",
            microcdr.join("src/c"),
            "nros setup --source micro-cdr  \
             (or: git submodule update --init packages/rmw/xrce/xrce-sys/micro-cdr)",
        ),
        (
            "nros-rmw-xrce",
            xrce_c.join("src"),
            "in-repo wrapper — expected at packages/rmw/xrce/nros-rmw-xrce/src",
        ),
    ] {
        let has_sources = std::fs::read_dir(&root)
            .map(|entries| {
                entries.flatten().any(|e| {
                    e.path().extension().is_some_and(|x| x == "c")
                        || e.file_type().map(|t| t.is_dir()).unwrap_or(false)
                })
            })
            .unwrap_or(false);
        if !root.is_dir() || !has_sources {
            panic!(
                "nros-rmw-xrce-cffi: vendored `{label}` source root {} is missing or has no \
                 .c files — submodule not initialised or upstream layout drifted. Fix: {hint}",
                root.display()
            );
        }
        println!("cargo:rerun-if-changed={}", root.display());
    }

    // Issue 1069 — the two generated config headers take their version from
    // these files (`vendored_project_version`), so a submodule bump that moves
    // only the `project(… VERSION …)` line must still re-run this script. The
    // loop above watches `src/c`, which a version-only bump need not touch.
    for cml in [
        microxrce.join("CMakeLists.txt"),
        microcdr.join("CMakeLists.txt"),
    ] {
        println!("cargo:rerun-if-changed={}", cml.display());
    }

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Phase 129.C.1 — platform fanout driven by `target_os` alone.
    // `nros-rmw-xrce-cffi` is platform-blind after 129.NET.3: the
    // session UDP path runs `xrce_nros_udp_init` on top of
    // `nros_platform_udp_*` regardless of platform. The build script
    // only still cares about `target_os` for two narrow reasons:
    //   1. Whether to compile the upstream `udp_transport*.c` and
    //      `util/time.c` POSIX-only TUs (they call libc directly).
    //   2. Whether to define `_POSIX_C_SOURCE` (needed to unlock
    //      `clock_gettime` / `getaddrinfo` in POSIX libc headers).
    // No `CARGO_FEATURE_PLATFORM_*` reads — the features that used
    // to gate these were deleted in 129.C.1.
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let host_is_posix = matches!(
        target_os.as_str(),
        "linux" | "macos" | "freebsd" | "netbsd" | "openbsd"
    );
    // 129.C.1 — `transport_zephyr_udp` was superseded by `transport_nros_udp`,
    // so upstream's Zephyr platform is off everywhere. That used to be a
    // `let feat_zephyr = false;` here feeding a dead `else if` branch in the
    // header generator; phase-420 W9 moved it to `flag uxr
    // UCLIENT_PLATFORM_ZEPHYR never` in the shared manifest, where the CMake
    // lane makes the same claim from the same line.
    let is_posix = host_is_posix;
    let is_embedded = !host_is_posix;
    // Phase 204.7 — `NROS_LINK_IP=0` drops the IP (UDP/TCP) transport on a
    // serial-only hosted node (mirrors the zenoh `Z_FEATURE_LINK_*` gate). It gates
    // both the upstream `udp_transport*.c` sources and the `UCLIENT_PROFILE_UDP/TCP`
    // defines below. Embedded XRCE already excludes IP (custom transport), so this
    // only matters on POSIX. Default (unset) → IP on, unchanged.
    println!("cargo:rerun-if-env-changed=NROS_LINK_IP");
    let ip = !matches!(
        env::var("NROS_LINK_IP").ok().as_deref(),
        Some("0") | Some("false") | Some("off")
    );

    // Issue 1068 — the source list is DERIVED, not mirrored. Both this build
    // script and `nros-rmw-xrce/CMakeLists.txt` read
    // `packages/rmw/xrce/xrce-sources.txt`; neither holds a source path of its
    // own. See that file's header for the format and for why the conditions
    // live there rather than here.
    let manifest_path = workspace.join("packages/rmw/xrce/xrce-sources.txt");
    println!("cargo:rerun-if-changed={}", manifest_path.display());
    let manifest = SourceManifest::read(&manifest_path);

    // phase-420 W9 — the same treatment for the VALUES. One statement of every
    // template token, every profile toggle and every knob, read by both lanes.
    let config_path = workspace.join("packages/rmw/xrce/xrce-config.txt");
    println!("cargo:rerun-if-changed={}", config_path.display());
    let config = ConfigManifest::read(&config_path);
    let knobs = KnobResolver::new();

    // NROS-XRCE-CONDITIONS-BEGIN — the boolean this lane supplies for each
    // condition token. `check-xrce-source-manifest` asserts the CMake lane
    // answers exactly this token set, and that it is exactly the set the two
    // manifests use; a manifest, not this match, decides what a token covers.
    //
    // ONE vocabulary for both manifests on purpose: `xrce-sources.txt` selects
    // FILES with these tokens and `xrce-config.txt` selects PROFILE DEFINES
    // with them, and those two answers have to agree — a header promising
    // `UCLIENT_PROFILE_UDP` whose `udp_transport.c` was not compiled is the
    // 1068 failure wearing a link error.
    let condition = |token: &str| -> bool {
        match token {
            "always" => true,
            // `util/time.c` calls `clock_gettime` / `nanosleep`;
            // `transport_posix_{udp,serial}.c` need <sys/socket.h> /
            // <termios.h>. Embedded targets supply their own time and
            // transport through the registry.
            "posix" => is_posix,
            // Phase 204.7 — `NROS_LINK_IP=0` sheds the IP link on a
            // serial-only node. Embedded XRCE already excludes IP.
            "posix_ip" => is_posix && ip,
            // phase-420 W9 — a toggle compiled off on every target. Stated
            // rather than omitted: an omitted `#cmakedefine` is `/* #undef */`
            // under `configure_file` and an UNTOUCHED `#cmakedefine` line
            // under a hand substitution, so silence is two different headers.
            "never" => false,
            other => panic!(
                "nros-rmw-xrce-cffi: an xrce manifest uses condition token `{other}`, which \
                 this build script does not answer. Add an arm here AND in \
                 nros-rmw-xrce/CMakeLists.txt — a token only one lane answers is issue 1068 \
                 again.",
            ),
        }
    };
    // NROS-XRCE-CONDITIONS-END

    // Generate config headers.
    generate_config(
        &out_dir,
        &microcdr.join("include/ucdr/config.h.in"),
        "include/ucdr/config.h",
        "ucdr",
        vendored_project_version(&microcdr.join("CMakeLists.txt"), "microcdr"),
        &config,
        &config_path,
        &knobs,
        &condition,
    );
    generate_config(
        &out_dir,
        &microxrce.join("include/uxr/client/config.h.in"),
        "include/uxr/client/config.h",
        "uxr",
        vendored_project_version(&microxrce.join("CMakeLists.txt"), "microxrcedds_client"),
        &config,
        &config_path,
        &knobs,
        &condition,
    );

    let mut build = cc::Build::new();
    // issue 0383 — implicit-function-declaration / int-conversion as errors
    // (`warnings(false)` only omits `-Wall`/`-Wextra`; cc-rs passes no `-w`).
    nros_cc_flags::strict_decls(&mut build);
    build
        .std("c99")
        .warnings(false)
        // Phase 204.9 — size: `-Os` + per-fn/data sections so the embedded
        // link path's `--gc-sections` (204.8) can strip unused XRCE surface.
        .opt_level_str("s")
        .flag_if_supported("-ffunction-sections")
        .flag_if_supported("-fdata-sections")
        .define("_DEFAULT_SOURCE", None)
        .include(out_dir.join("include"))
        .include(microcdr.join("include"))
        .include(microxrce.join("include"))
        .include(microxrce.join("src/c"))
        .include(xrce_c.join("src"))
        .include(xrce_c.join("include"))
        .include(workspace.join("packages/core/nros-rmw-abi/include"))
        .include(workspace.join("packages/platform/nros-platform-api/include"));
    if is_posix {
        // `_POSIX_C_SOURCE` is what unlocks `clock_gettime`,
        // `getaddrinfo`, etc in `<sys/socket.h>` + `<time.h>` on
        // glibc / musl / macOS. Bare-metal & Zephyr stdlibs don't
        // ship these — gating the define keeps the embedded build
        // from pulling in headers it can't satisfy.
        build.define("_POSIX_C_SOURCE", Some("200809L"));
    }

    let tree_root = |tree: &str| -> PathBuf {
        match tree {
            "uxr" => microxrce.join("src/c"),
            "ucdr" => microcdr.join("src/c"),
            "backend" => xrce_c.join("src"),
            other => panic!(
                "nros-rmw-xrce-cffi: {} names unknown source tree `{other}`",
                manifest_path.display()
            ),
        }
    };

    let mut compiled = 0usize;
    for row in &manifest.rows {
        assert!(
            COMPILED_TREES.contains(&row.tree.as_str()),
            "nros-rmw-xrce-cffi: {} has a row in tree `{}`, which COMPILED_TREES does not \
             list. This lane compiles EVERY tree — the archive it produces is the one \
             `nros-rmw-xrce/CMakeLists.txt` links, so a tree missing here is a tree missing \
             from that link. phase-420 W9 step 4.",
            manifest_path.display(),
            row.tree,
        );
        if condition(manifest.condition_of(&row.group, &manifest_path)) {
            build.file(tree_root(&row.tree).join(&row.path));
            compiled += 1;
        }
    }
    assert!(
        compiled > 0,
        "nros-rmw-xrce-cffi: {} selected no sources — manifest or condition drift",
        manifest_path.display()
    );

    if is_embedded {
        // Tell `<uxr/client/config_internal.h>` not to require the
        // POSIX TUs we've just dropped from the source list.
        build.define("UCLIENT_PLATFORM_NO_POSIX", None);
    }

    // phase-420 W9 — the backend's `-D` pool knobs, from the shared manifest's
    // `define` records. Phase 207.6 is why they exist: a pub-only bare-metal
    // node drops subscribers/services to 1, the ring to 1 and the buffer to
    // 256, and with `STREAM_HISTORY=4` plus a 512-byte MTU the session struct
    // falls from ~390 KB to ~10–20 KB.
    //
    // A `define` row states NO default — `nros-rmw-xrce/src/internal.h` holds
    // it in an `#ifndef`, and that is the one statement of it. Nothing stated
    // ⇒ nothing defined ⇒ the header's default stands, identically in both
    // lanes.
    for row in &config.defines {
        if let Some(n) = knobs.stated(&row.env, row.min, &config_path) {
            build.define(&row.macro_name, n.to_string().as_str());
        }
    }

    let archive_stem = "nros_rmw_xrce_c_inline";
    build.compile(archive_stem);

    // phase-420 W9 step 4 — SAY WHERE THE ARCHIVE IS, in the one place a
    // consumer reaching `OUT_DIR` can read it.
    //
    // `nros-rmw-xrce/CMakeLists.txt` used to compile the vendored
    // micro-XRCE-DDS-Client and micro-CDR TUs a SECOND time, from the same
    // manifest with its own flags, so its CTest harness validated objects no
    // image contains. It now links THIS archive. Two facts have to cross that
    // seam — what cc-rs called the archive, and where `generate_config` put the
    // headers those objects were compiled against — and both are facts of this
    // file, so this file states them rather than the consumer guessing.
    //
    // A `#`-commented `key=value` file, the same dependency-free shape as
    // `xrce-sources.txt` / `xrce-config.txt`, so the reader needs no parser.
    let archive = out_dir.join(format!("lib{archive_stem}.a"));
    assert!(
        archive.is_file(),
        "nros-rmw-xrce-cffi: cc-rs did not leave `{}` where this script expects it. \
         `nros-rmw-xrce/CMakeLists.txt` links that path.",
        archive.display()
    );
    let pointer = out_dir.join(VENDOR_BUILD_POINTER);
    fs::write(
        &pointer,
        format!(
            "# Written by nros-rmw-xrce-cffi/build.rs — phase-420 W9 step 4.\n\
             # Read by nros-rmw-xrce/CMakeLists.txt, which LINKS this archive instead of\n\
             # compiling the vendored micro-XRCE-DDS-Client / micro-CDR sources again.\n\
             # Do not hand-edit: it is regenerated on every build of this crate.\n\
             archive={}\n\
             include={}\n",
            archive.display(),
            out_dir.join("include").display(),
        ),
    )
    .unwrap_or_else(|e| {
        panic!(
            "nros-rmw-xrce-cffi: cannot write {} ({e})",
            pointer.display()
        )
    });

    // Phase 129.NET.3 — `transport_nros_udp.c` references the
    // canonical `nros_platform_udp_*` ABI. Ship the sibling
    // `nros-platform-posix` C port inside this crate's static
    // archive whenever the build resolves to a POSIX host
    // (explicit `platform-posix` feature or host-OS auto-detect).
    // Consumers that bring their own platform-provider library
    // (e.g. the C SDK linked under cmake with `nano_ros_link_platform`)
    // must opt out by NOT selecting `posix` / `platform-posix`
    // and forcing a non-host target — otherwise the link hits
    // duplicate-symbol errors.
    if is_posix {
        let posix_src = workspace.join("packages/platform/nros-platform-posix/src");
        let mut posix_build = cc::Build::new();
        // issue 0383 — implicit-function-declaration / int-conversion as errors.
        nros_cc_flags::strict_decls(&mut posix_build);
        posix_build
            .std("c11")
            .warnings(false)
            .define("_DEFAULT_SOURCE", None)
            .define("_POSIX_C_SOURCE", Some("200809L"))
            .include(workspace.join("packages/platform/nros-platform-api/include"))
            .file(posix_src.join("platform.c"))
            .file(posix_src.join("net.c"))
            .file(posix_src.join("timer.c"));
        posix_build.compile("nros_platform_posix_link");
        println!("cargo:rerun-if-changed={}", posix_src.display());
    }

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}", xrce_c.join("src").display());
    println!(
        "cargo:rerun-if-changed={}",
        xrce_c.join("include").display()
    );
}

// NROS-XRCE-VERSIONS-BEGIN
/// The vendored tree's OWN statement of its version — issue 1069.
///
/// `MICROCDR_VERSION_STR` compiled as `"2.0.2"` here and `"2.4.1"` under the
/// CMake lane, for one tree that is 2.0.2: the versions were hand-restated in
/// four places (both lanes, plus two `nros-sdk-index.toml` rows) and disagreed.
/// Correcting the literals would have left the same four literals, so both
/// lanes read the fact instead — each vendored `CMakeLists.txt` says
/// `project(<name> VERSION "X.Y.Z")`, which is upstream telling us what the
/// tree is. `nros-rmw-xrce/CMakeLists.txt` has the CMake twin,
/// `_nros_xrce_project_version()`. Gate: `just check xrce-vendored-versions`.
///
/// NOT `git describe --tags`, which issue 1069 proposed: a submodule is fetched
/// by SHA with no tags, so `git -C …/micro-xrce-dds-client describe --tags`
/// answers "No tags can describe" on an ordinary checkout.
///
/// Panics rather than defaulting: a wrong version compiles into a public macro
/// and past upstream's own `#if UXR_CLIENT_VERSION_MAJOR >= 4` tripwire, so
/// "close enough" is exactly the failure mode being retired.
fn vendored_project_version(cmakelists: &std::path::Path, project: &str) -> [String; 3] {
    let text = fs::read_to_string(cmakelists).unwrap_or_else(|e| {
        panic!(
            "nros-rmw-xrce-cffi: cannot read {} ({e}) — the vendored {project} checkout \
             is missing. Run `nros setup --source micro-cdr --source \
             micro-xrce-dds-client` (or `git submodule update --init`).",
            cmakelists.display()
        )
    });
    let mut found: Vec<[String; 3]> = Vec::new();
    for line in text.lines() {
        let line = line.split('#').next().unwrap_or("").trim();
        let Some(rest) = line.strip_prefix("project(") else {
            continue;
        };
        let mut tok = rest.split_whitespace();
        if tok.next() != Some(project) {
            continue;
        }
        if !tok
            .next()
            .is_some_and(|k| k.eq_ignore_ascii_case("VERSION"))
        {
            continue;
        }
        let raw = tok.next().unwrap_or("").trim_matches('"');
        let parts: Vec<&str> = raw.split('.').collect();
        if parts.len() != 3
            || !parts
                .iter()
                .all(|p| !p.is_empty() && p.bytes().all(|b| b.is_ascii_digit()))
        {
            panic!(
                "nros-rmw-xrce-cffi: `project({project} VERSION …)` in {} does not state \
                 an X.Y.Z version (got `{raw}`).",
                cmakelists.display()
            );
        }
        found.push([
            parts[0].to_string(),
            parts[1].to_string(),
            parts[2].to_string(),
        ]);
    }
    match found.len() {
        1 => found.pop().unwrap(),
        n => panic!(
            "nros-rmw-xrce-cffi: expected exactly one `project({project} VERSION …)` line in \
             {}, found {n}. Upstream changed how it states its version — update BOTH lanes \
             (this function and `_nros_xrce_project_version()` in \
             nros-rmw-xrce/CMakeLists.txt) together.",
            cmakelists.display()
        ),
    }
}
// NROS-XRCE-VERSIONS-END

/// Fill in one upstream `config.h.in` from the shared manifest — phase-420 W9.
///
/// ONE implementation for both templates, and the CMake lane's `configure_file`
/// calls are the twin. Before this there were four: a hand-written
/// `generate_ucdr_config` and `generate_uxr_config` here, and two
/// `configure_file`s there, each restating the token values. The two lanes
/// already disagreed about the version (issue 1069) for exactly that reason.
///
/// `@PROJECT_VERSION*@` is the one thing NOT read from the manifest: it comes
/// from the vendored tree's own `project(<name> VERSION …)` line, because a
/// version stated in our manifest would be one more hand-written restatement
/// of a fact upstream already holds.
#[allow(clippy::too_many_arguments)]
fn generate_config(
    out_dir: &std::path::Path,
    template_path: &std::path::Path,
    out_rel: &str,
    template: &str,
    version: [String; 3],
    config: &ConfigManifest,
    config_path: &std::path::Path,
    knobs: &KnobResolver,
    condition: &dyn Fn(&str) -> bool,
) {
    let text = fs::read_to_string(template_path).unwrap_or_else(|e| {
        panic!(
            "nros-rmw-xrce-cffi: cannot read the upstream template {} ({e}) — the vendored \
             checkout is missing. Run `nros setup --source micro-cdr --source \
             micro-xrce-dds-client` (or `git submodule update --init`).",
            template_path.display()
        )
    });
    println!("cargo:rerun-if-changed={}", template_path.display());

    let [maj, min, pat] = version;
    let mut h = text
        .replace("@PROJECT_VERSION_MAJOR@", &maj)
        .replace("@PROJECT_VERSION_MINOR@", &min)
        .replace("@PROJECT_VERSION_PATCH@", &pat)
        .replace("@PROJECT_VERSION@", &format!("{maj}.{min}.{pat}"));

    // `value` — a fixed substitution.
    for row in config.values_for(template) {
        h = h.replace(&format!("@{}@", row.token), &row.literal);
    }

    // `knob` — a substitution someone may choose. The ladder and the minimum
    // both live in the manifest, so the CMake lane enforces the same floor on
    // the same input.
    for row in config.knobs_for(template) {
        let v = knobs.value(&row.env, row.default, row.min, config_path);
        h = h.replace(&format!("@{}@", row.token), &v.to_string());
    }

    // `flag` — a `#cmakedefine` toggle. CMake writes `#define NAME` when the
    // variable is truthy and `/* #undef NAME */` when it is not; match that
    // byte for byte, because these two headers are diffed as an acceptance
    // criterion and a stylistic difference would read as a real one.
    //
    // Match the whole LINE (`\n` boundary) so the `UCLIENT_PLATFORM_POSIX`
    // rule does not also fire on `UCLIENT_PLATFORM_POSIX_NOPOLL`.
    for row in config.flags_for(template) {
        let on = condition(&row.condition);
        let line = if on {
            format!("#define {}\n", row.token)
        } else {
            format!("/* #undef {} */\n", row.token)
        };
        h = h.replace(&format!("#cmakedefine {}\n", row.token), &line);
    }

    let out = out_dir.join(out_rel);
    fs::create_dir_all(out.parent().expect("config.h has a parent dir")).unwrap();
    fs::write(&out, h).unwrap();
}

/// The knob ladder — ONE implementation, shared by the template `knob` rows and
/// the backend `define` rows. phase-420 W9.
///
/// Rungs, highest first:
///
///   1. the environment variable    — a person, right now
///   2. `CONFIG_<env>` in `$DOTCONFIG` — a person, in the tree (Kconfig)
///   3. the `[knobs.xrce]` rung     — this lane only; see below
///   4. the manifest's `<default>`  — nobody stated one ([`Self::value`] only)
///
/// issue 0460 — rung 2 is why this is not a bare `env::var`. Every one of these
/// knobs is forwarded by `_nros_resolve_knob()` in
/// `zephyr/cmake/nros_cargo_build.cmake` with `set(ENV{...})`, which reaches
/// the C lane's re-baked command and NOT the Rust lane's: zephyr-lang-rust's
/// `rust_cargo_application` builds its own cargo invocation and inherits
/// nothing, so a Zephyr Rust image read every one of them as unset whatever
/// Kconfig said. The env name is the Kconfig name minus `CONFIG_`, so the pair
/// is DERIVED rather than tabulated; `check-kconfig-knob-forwarding` proves the
/// cmake list and the readers agree.
///
/// Rung 3 is the one asymmetry between the lanes, and it is stated in
/// `xrce-config.txt` rather than left for a reader to discover: reading a
/// `[knobs.xrce]` TOML needs a parser the CMake lane does not have, it covers
/// exactly two knobs, and it can only be delivered by a cargo build — so it
/// cannot fire in a lane with no cargo.
struct KnobResolver {
    rungs: nros_platform_config::platform_config::XrceKnobs,
}

impl KnobResolver {
    fn new() -> Self {
        // phase-400 W6 — the `[knobs.xrce]` rungs sit under the env front-end
        // and above the manifest defaults.
        Self {
            rungs: nros_platform_config::platform_config::BuildRungs::from_build_env()
                .map(|r| r.xrce_rungs())
                .unwrap_or_default(),
        }
    }

    fn rung(&self, env_name: &str) -> Option<usize> {
        match env_name {
            "NROS_XRCE_CUSTOM_TRANSPORT_MTU" => self.rungs.custom_transport_mtu,
            "NROS_XRCE_STREAM_HISTORY" => self.rungs.stream_history,
            _ => None,
        }
    }

    /// Rungs 1–3: what a person or a platform config STATED, or `None`.
    ///
    /// An environment value that is not a number PANICS — someone typed it in
    /// this shell and falling through to a default would answer a question they
    /// did not ask. A Kconfig value that is not a `usize` is ABSENT instead:
    /// `-1` is the documented DERIVE sentinel (phase-403 W8), and a knob left
    /// on it means "nothing stated", not "malformed".
    fn stated(&self, env_name: &str, min: usize, config_path: &std::path::Path) -> Option<usize> {
        println!("cargo:rerun-if-env-changed={env_name}");
        let stated = match env::var(env_name) {
            Ok(raw) if !raw.is_empty() => Some(raw.parse::<usize>().unwrap_or_else(|_| {
                panic!("nros-rmw-xrce-cffi: {env_name}='{raw}' is not a number")
            })),
            _ => nros_zephyr_build::dotconfig_usize(&format!("CONFIG_{env_name}"))
                .or_else(|| self.rung(env_name)),
        };
        if let Some(n) = stated {
            if n < min {
                panic!(
                    "nros-rmw-xrce-cffi: {env_name}={n} is below the minimum {min} stated in {}",
                    config_path.display()
                );
            }
        }
        stated
    }

    /// Rungs 1–4: the value to compile with.
    fn value(
        &self,
        env_name: &str,
        default: usize,
        min: usize,
        config_path: &std::path::Path,
    ) -> usize {
        self.stated(env_name, min, config_path).unwrap_or(default)
    }
}

/// One `src <group> <tree> <path>` record from the shared source manifest.
struct SourceRow {
    group: String,
    tree: String,
    path: String,
}

/// `packages/rmw/xrce/xrce-sources.txt`, parsed — issue 1068.
///
/// The list of C files this backend compiles used to exist TWICE: here and in
/// `nros-rmw-xrce/CMakeLists.txt`, with a comment asserting they stayed in
/// lockstep. They did not — `NROS_LINK_IP=0` dropped the UDP transports on this
/// side and not on the CMake side, so a serial-only XRCE node could not be
/// built from C or C++. The list is now read from one file by both lanes.
///
/// The format is deliberately line-oriented and dependency-free: CMake reads
/// the same file with `file(STRINGS)`, and `cargo` is `--locked`-shimmed here,
/// so a TOML crate is not available to a build script that must also work from
/// a bare clone.
struct SourceManifest {
    /// group name → condition token, in declaration order.
    groups: Vec<(String, String)>,
    rows: Vec<SourceRow>,
}

impl SourceManifest {
    fn read(path: &std::path::Path) -> Self {
        let text = fs::read_to_string(path).unwrap_or_else(|e| {
            panic!(
                "nros-rmw-xrce-cffi: cannot read the shared XRCE source manifest {} ({e}). It is \
                 tracked in-repo; a missing one means the checkout is incomplete.",
                path.display()
            )
        });
        let mut groups: Vec<(String, String)> = Vec::new();
        let mut rows = Vec::new();
        for (n, raw) in text.lines().enumerate() {
            let line = raw.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            let f: Vec<&str> = line.split_whitespace().collect();
            match f.as_slice() {
                ["group", name, cond] => {
                    if groups.iter().any(|(g, _)| g == name) {
                        panic!(
                            "{}:{}: group `{name}` declared twice",
                            path.display(),
                            n + 1
                        );
                    }
                    groups.push(((*name).to_string(), (*cond).to_string()));
                }
                ["src", group, tree, rel] => rows.push(SourceRow {
                    group: (*group).to_string(),
                    tree: (*tree).to_string(),
                    path: (*rel).to_string(),
                }),
                _ => panic!(
                    "{}:{}: expected `group <name> <condition>` or `src <group> <tree> <path>`, \
                     got `{line}`",
                    path.display(),
                    n + 1
                ),
            }
        }
        Self { groups, rows }
    }

    fn condition_of<'a>(&'a self, group: &str, path: &std::path::Path) -> &'a str {
        self.groups
            .iter()
            .find(|(g, _)| g == group)
            .map(|(_, c)| c.as_str())
            .unwrap_or_else(|| {
                panic!(
                    "{}: source names group `{group}`, which no `group` line declares",
                    path.display()
                )
            })
    }
}

/// `value <template> <token> <literal>` — a fixed `@token@` substitution.
struct ValueRow {
    template: String,
    token: String,
    literal: String,
}

/// `knob <template> <token> <env> <default> <min>` — a tunable substitution.
struct KnobRow {
    template: String,
    token: String,
    env: String,
    default: usize,
    min: usize,
}

/// `flag <template> <token> <condition>` — a `#cmakedefine` toggle.
struct FlagRow {
    template: String,
    token: String,
    condition: String,
}

/// `define <macro> <env> <min>` — a `-D` on the backend compile. No default
/// column by design: `nros-rmw-xrce/src/internal.h` holds it in an `#ifndef`.
struct DefineRow {
    macro_name: String,
    env: String,
    min: usize,
}

/// `packages/rmw/xrce/xrce-config.txt`, parsed — phase-420 W9.
///
/// Sibling of [`SourceManifest`]: that one answers "which files", this one
/// "with what values". Both lanes read both, and for the same reason — the
/// values used to exist TWICE (here, and as `set(UCLIENT_…)` plus
/// `configure_file` in `nros-rmw-xrce/CMakeLists.txt`) with nothing but
/// proximity holding them together. See the file's own header for the format,
/// the knob ladder, and the one rung this lane has that the other cannot.
struct ConfigManifest {
    values: Vec<ValueRow>,
    knobs: Vec<KnobRow>,
    flags: Vec<FlagRow>,
    defines: Vec<DefineRow>,
}

impl ConfigManifest {
    fn read(path: &std::path::Path) -> Self {
        let text = fs::read_to_string(path).unwrap_or_else(|e| {
            panic!(
                "nros-rmw-xrce-cffi: cannot read the shared XRCE config manifest {} ({e}). It is \
                 tracked in-repo; a missing one means the checkout is incomplete.",
                path.display()
            )
        });
        let mut m = Self {
            values: Vec::new(),
            knobs: Vec::new(),
            flags: Vec::new(),
            defines: Vec::new(),
        };
        let num = |field: &str, n: usize| -> usize {
            field.parse().unwrap_or_else(|_| {
                panic!("{}:{}: `{field}` is not a number", path.display(), n + 1)
            })
        };
        for (n, raw) in text.lines().enumerate() {
            let line = raw.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            let f: Vec<&str> = line.split_whitespace().collect();
            match f.as_slice() {
                ["value", template, token, literal] => m.values.push(ValueRow {
                    template: (*template).to_string(),
                    token: (*token).to_string(),
                    literal: (*literal).to_string(),
                }),
                ["knob", template, token, env, default, min] => m.knobs.push(KnobRow {
                    template: (*template).to_string(),
                    token: (*token).to_string(),
                    env: (*env).to_string(),
                    default: num(default, n),
                    min: num(min, n),
                }),
                ["flag", template, token, condition] => m.flags.push(FlagRow {
                    template: (*template).to_string(),
                    token: (*token).to_string(),
                    condition: (*condition).to_string(),
                }),
                ["define", macro_name, env, min] => m.defines.push(DefineRow {
                    macro_name: (*macro_name).to_string(),
                    env: (*env).to_string(),
                    min: num(min, n),
                }),
                _ => panic!(
                    "{}:{}: expected `value <template> <token> <literal>`, \
                     `knob <template> <token> <env> <default> <min>`, \
                     `flag <template> <token> <condition>` or \
                     `define <macro> <env> <min>`, got `{line}`",
                    path.display(),
                    n + 1
                ),
            }
        }
        if m.values.is_empty() && m.knobs.is_empty() && m.flags.is_empty() {
            panic!(
                "{}: no substitutions at all — manifest drift",
                path.display()
            );
        }
        m
    }

    fn values_for(&self, template: &str) -> impl Iterator<Item = &ValueRow> {
        self.values.iter().filter(move |r| r.template == template)
    }

    fn knobs_for(&self, template: &str) -> impl Iterator<Item = &KnobRow> {
        self.knobs.iter().filter(move |r| r.template == template)
    }

    fn flags_for(&self, template: &str) -> impl Iterator<Item = &FlagRow> {
        self.flags.iter().filter(move |r| r.template == template)
    }
}
