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

// Phase 214.C.2 — single source-of-truth for XRCE transport MTU defaults.
// UDP/TCP/custom share a 4096-byte default; serial uses a smaller 512-byte
// default (UART throughput floor).
const XRCE_TRANSPORT_MTU_DEFAULT: &str = "4096";
const XRCE_SERIAL_MTU_DEFAULT: &str = "512";

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
    let feat_zephyr = false; // 129.C.1 — `transport_zephyr_udp` superseded by `transport_nros_udp`.
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

    // Generate config headers.
    generate_ucdr_config(&out_dir, &microcdr);
    generate_uxr_config(&out_dir, &microxrce, feat_zephyr, is_posix);

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

    // Issue 1068 — the source list is DERIVED, not mirrored. Both this build
    // script and `nros-rmw-xrce/CMakeLists.txt` read
    // `packages/rmw/xrce/xrce-sources.txt`; neither holds a source path of its
    // own. See that file's header for the format and for why the conditions
    // live there rather than here.
    let manifest_path = workspace.join("packages/rmw/xrce/xrce-sources.txt");
    println!("cargo:rerun-if-changed={}", manifest_path.display());
    let manifest = SourceManifest::read(&manifest_path);

    // NROS-XRCE-CONDITIONS-BEGIN — the boolean this lane supplies for each
    // condition token. `check-xrce-source-manifest` asserts the CMake lane
    // answers exactly this token set, and that it is exactly the set the
    // manifest uses; the manifest, not this match, decides which files a token
    // covers.
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
            other => panic!(
                "nros-rmw-xrce-cffi: {} uses condition token `{other}`, which this build \
                 script does not answer. Add an arm here AND in \
                 nros-rmw-xrce/CMakeLists.txt — a token only one lane answers is issue 1068 \
                 again.",
                manifest_path.display()
            ),
        }
    };
    // NROS-XRCE-CONDITIONS-END

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

    // Phase 130.6 — tunable reliable-stream history. Tight-RAM
    // targets that don't run server-side action callbacks can drop
    // from the default 16 (= 64 KiB per-session output buffer) to 8
    // or 4. `internal.h` enforces `>= 4`.
    // phase-400 W6 — the `[knobs.xrce]` rungs sit under the env front-end and
    // above the builtins below.
    let xrce_rungs = nros_platform_config::platform_config::BuildRungs::from_build_env()
        .map(|r| r.xrce_rungs())
        .unwrap_or_default();
    if let Some(v) = knob("NROS_XRCE_STREAM_HISTORY")
        .or_else(|| xrce_rungs.stream_history.map(|n| n.to_string()))
    {
        let n: u32 = v
            .parse()
            .unwrap_or_else(|_| panic!("NROS_XRCE_STREAM_HISTORY='{}' is not a number", v));
        if n < 4 {
            panic!("NROS_XRCE_STREAM_HISTORY={} too small (minimum 4)", n);
        }
        build.define("XRCE_STREAM_HISTORY", n.to_string().as_str());
    }
    println!("cargo:rerun-if-env-changed=NROS_XRCE_STREAM_HISTORY");
    println!("cargo:rerun-if-env-changed=NROS_XRCE_CUSTOM_TRANSPORT_MTU");
    println!("cargo:rerun-if-env-changed=NROS_XRCE_TRANSPORT_MTU");

    // Phase 207.6 — per-session pool sizes. A pub-only bare-metal node
    // can drop `MAX_SUBSCRIBERS` to 0, `MAX_SERVICE_SERVERS` /
    // `MAX_SERVICE_CLIENTS` to 0, `SUBSCRIBER_RING_DEPTH` to 1, and
    // `BUFFER_SIZE` to 256. Combined with `STREAM_HISTORY=4` +
    // `NROS_XRCE_CUSTOM_TRANSPORT_MTU=512` the session struct drops
    // from ~390 KB to ~10–20 KB.
    // issue 1033 — the entity caps admit ZERO, and that is the whole saving.
    //
    // The minimum used to be 1, justified as "zero-length arrays aren't
    // standard C; 1 is the practical minimum". Measured, that is wrong in the
    // way that mattered:
    //
    //  * these are PLAIN arrays MID-struct, not flexible array members, so the
    //    `flexible array member not at end of struct` failure this repo has
    //    seen came from an EMPTY define (`arr[]`), not a zero one (`arr[0]`) —
    //    two different errors that read alike;
    //  * `slot_t arr[0];` mid-struct compiles on gnu11, c11 AND gnu99; only
    //    `-pedantic` rejects it, and nothing here passes `-pedantic`;
    //  * nothing indexes slot 0 unconditionally — every walk is
    //    `for (i = 0; i < MAX; ++i)`, which at 0 simply does not run.
    //
    // And clamping was the thing issue 0827 forbids in as many words: it
    // "reserved the memory anyway while reading as though the knob had been
    // honoured". A cap of 0 on an image that creates none of that entity is the
    // honest answer, and on the zephyr cpp listener it is what takes
    // `xrce_session_state_t` from 76,624 bytes to 54,928 — under its 66,048
    // heap, so the image boots.
    for (env_name, define_name, min) in [
        ("NROS_XRCE_MAX_SUBSCRIBERS", "XRCE_MAX_SUBSCRIBERS", 0),
        (
            "NROS_XRCE_MAX_SERVICE_SERVERS",
            "XRCE_MAX_SERVICE_SERVERS",
            0,
        ),
        (
            "NROS_XRCE_MAX_SERVICE_CLIENTS",
            "XRCE_MAX_SERVICE_CLIENTS",
            0,
        ),
        (
            "NROS_XRCE_SUBSCRIBER_RING_DEPTH",
            "XRCE_SUBSCRIBER_RING_DEPTH",
            1,
        ),
        ("NROS_XRCE_BUFFER_SIZE", "XRCE_BUFFER_SIZE", 64),
    ] {
        if let Some(v) = knob(env_name) {
            let n: u32 = v
                .parse()
                .unwrap_or_else(|_| panic!("{env_name}='{v}' is not a number"));
            if n < min {
                panic!("{env_name}={n} too small (minimum {min})");
            }
            build.define(define_name, n.to_string().as_str());
        }
        println!("cargo:rerun-if-env-changed={env_name}");
    }

    build.compile("nros_rmw_xrce_c_inline");

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

fn generate_ucdr_config(out_dir: &std::path::Path, microcdr: &std::path::Path) {
    let template = fs::read_to_string(microcdr.join("include/ucdr/config.h.in"))
        .expect("read ucdr config.h.in");
    // Issue 1069 — read out of the vendored tree, not restated here.
    let [maj, min, pat] = vendored_project_version(&microcdr.join("CMakeLists.txt"), "microcdr");
    let header = template
        .replace("@PROJECT_VERSION_MAJOR@", &maj)
        .replace("@PROJECT_VERSION_MINOR@", &min)
        .replace("@PROJECT_VERSION_PATCH@", &pat)
        .replace("@PROJECT_VERSION@", &format!("{maj}.{min}.{pat}"))
        // ucdrEndianness enum: BIG=0, LITTLE=1. Set 1 for x86 / ARM.
        .replace("@CONFIG_MACHINE_ENDIANNESS@", "1");
    let dir = out_dir.join("include/ucdr");
    fs::create_dir_all(&dir).unwrap();
    fs::write(dir.join("config.h"), header).unwrap();
}

fn generate_uxr_config(
    out_dir: &std::path::Path,
    microxrce: &std::path::Path,
    is_zephyr: bool,
    is_posix: bool,
) {
    let template = fs::read_to_string(microxrce.join("include/uxr/client/config.h.in"))
        .expect("read uxr config.h.in");
    // issue 0968 — the general transport MTU, min-guarded like CUSTOM's.
    //
    // `knob_usize`, NOT `env::var`: a Zephyr RUST image inherits none of cmake's
    // `set(ENV{...})` exports, so a bare read yields the crate default whatever
    // Kconfig says (issue 0460). `check-kconfig-knob-forwarding` caught exactly
    // that in the first version of this fix, which would have repaired the C
    // lane and left the Rust one on 4096 — half-fixing the thing this change is
    // about.
    let xrce_transport_mtu = nros_zephyr_build::knob_usize(
        "NROS_XRCE_TRANSPORT_MTU",
        "CONFIG_NROS_XRCE_TRANSPORT_MTU",
        XRCE_TRANSPORT_MTU_DEFAULT
            .parse()
            .expect("XRCE_TRANSPORT_MTU_DEFAULT is a number"),
    );
    if xrce_transport_mtu < 128 {
        panic!("NROS_XRCE_TRANSPORT_MTU={xrce_transport_mtu} too small (minimum 128)");
    }
    let xrce_transport_mtu = xrce_transport_mtu.to_string();

    // Substitute @TOKEN@ placeholders.
    //
    // Issue 1069 — the version comes out of the vendored tree. It was `2.4.1`
    // here against a 3.0.1 checkout, which also disarmed upstream's own
    // `#if UXR_CLIENT_VERSION_MAJOR >= 4` tripwire by a whole major version.
    let [maj, min, pat] =
        vendored_project_version(&microxrce.join("CMakeLists.txt"), "microxrcedds_client");
    let mut h = template
        .replace("@PROJECT_VERSION_MAJOR@", &maj)
        .replace("@PROJECT_VERSION_MINOR@", &min)
        .replace("@PROJECT_VERSION_PATCH@", &pat)
        .replace("@PROJECT_VERSION@", &format!("{maj}.{min}.{pat}"))
        .replace("@UCLIENT_MAX_OUTPUT_BEST_EFFORT_STREAMS@", "1")
        .replace("@UCLIENT_MAX_OUTPUT_RELIABLE_STREAMS@", "1")
        .replace("@UCLIENT_MAX_INPUT_BEST_EFFORT_STREAMS@", "1")
        .replace("@UCLIENT_MAX_INPUT_RELIABLE_STREAMS@", "1")
        .replace("@UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS@", "10")
        .replace("@UCLIENT_MIN_SESSION_CONNECTION_INTERVAL@", "1000")
        .replace("@UCLIENT_MIN_HEARTBEAT_TIME_INTERVAL@", "100")
        // Phase 214.C.2 — MTU defaults from named consts at file top.
        // issue 0968 — UDP and TCP honour `NROS_XRCE_TRANSPORT_MTU` too. Only
        // CUSTOM was tunable, so a UDP image (which every Zephyr XRCE example
        // is — it dials an agent at `CONFIG_NROS_XRCE_AGENT_ADDR:PORT`) was
        // pinned to 4096 whatever Kconfig said. `STREAM_BUFFER_SIZE = MTU x
        // STREAM_HISTORY` twice per session, so the default cost 131072 bytes
        // where the configured 512 costs 16384.
        .replace("@UCLIENT_UDP_TRANSPORT_MTU@", &xrce_transport_mtu)
        .replace("@UCLIENT_TCP_TRANSPORT_MTU@", &xrce_transport_mtu)
        .replace("@UCLIENT_SERIAL_TRANSPORT_MTU@", XRCE_SERIAL_MTU_DEFAULT)
        .replace(
            "@UCLIENT_CUSTOM_TRANSPORT_MTU@",
            // Phase 207.6 — env-tunable so RAM-tight bare-metal nodes can
            // drop the per-session stream buffers (`STREAM_BUFFER_SIZE =
            // CUSTOM_TRANSPORT_MTU × STREAM_HISTORY`) by an order of
            // magnitude. Min 128 (smaller breaks the framing/header
            // assumptions); default tracks XRCE_TRANSPORT_MTU_DEFAULT.
            // phase-400 W6 — env, then the `[knobs.xrce]` rung, then the
            // default. `knob` rather than a bare `env::var` so the Kconfig
            // front-end reaches this the way it reaches the pool sizes above
            // (issue 0460).
            &knob("NROS_XRCE_CUSTOM_TRANSPORT_MTU")
                .or_else(|| {
                    nros_platform_config::platform_config::BuildRungs::from_build_env()
                        .and_then(|r| r.xrce_rungs().custom_transport_mtu)
                        .map(|n| n.to_string())
                })
                .unwrap_or_else(|| XRCE_TRANSPORT_MTU_DEFAULT.into()),
        )
        .replace("@UCLIENT_SHARED_MEMORY_MAX_ENTITIES@", "4")
        .replace("@UCLIENT_SHARED_MEMORY_STATIC_MEM_SIZE@", "10")
        .replace("@UCLIENT_HARD_LIVELINESS_CHECK_TIMEOUT@", "10000");

    // #cmakedefine handling. The template uses `#cmakedefine NAME` —
    // CMake replaces with `#define NAME` when var is set, `/* #undef
    // NAME */` otherwise.
    let mut enabled = vec![
        "UCLIENT_PROFILE_DISCOVERY",
        "UCLIENT_PROFILE_CUSTOM_TRANSPORT",
        "UCLIENT_PROFILE_STREAM_FRAMING",
        "UCLIENT_TWEAK_XRCE_WRITE_LIMIT",
    ];
    let mut disabled = vec![
        "UCLIENT_PROFILE_MULTITHREAD",
        "UCLIENT_PROFILE_SHARED_MEMORY",
        "UCLIENT_PROFILE_CAN",
        "UCLIENT_HARD_LIVELINESS_CHECK",
    ];
    // Platform fanout — POSIX gets the full UDP/TCP/SERIAL profile
    // set; Zephyr emits its own platform define so any upstream
    // `#ifdef UCLIENT_PLATFORM_ZEPHYR` branch picks the right path.
    // Pure bare-metal / FreeRTOS / NuttX / ThreadX gets the
    // freestanding core only — consumers wire their own transport
    // via `nros_rmw_cffi_set_custom_transport(...)`.
    // Phase 204.7 — recomputed here (separate fn from the source-file build);
    // gates the UDP/TCP profile defines to match the gated source files.
    let ip = !matches!(
        env::var("NROS_LINK_IP").ok().as_deref(),
        Some("0") | Some("false") | Some("off")
    );
    if is_posix {
        if ip {
            enabled.push("UCLIENT_PROFILE_UDP");
            enabled.push("UCLIENT_PROFILE_TCP");
        } else {
            disabled.push("UCLIENT_PROFILE_UDP");
            disabled.push("UCLIENT_PROFILE_TCP");
        }
        enabled.push("UCLIENT_PROFILE_SERIAL");
        enabled.push("UCLIENT_PLATFORM_POSIX");
        disabled.push("UCLIENT_PLATFORM_POSIX_NOPOLL");
        disabled.push("UCLIENT_PLATFORM_WINDOWS");
        disabled.push("UCLIENT_PLATFORM_FREERTOS_PLUS_TCP");
        disabled.push("UCLIENT_PLATFORM_RTEMS_BSD_NET");
        disabled.push("UCLIENT_PLATFORM_ZEPHYR");
    } else if is_zephyr {
        enabled.push("UCLIENT_PLATFORM_ZEPHYR");
        // UDP / TCP / SERIAL profile defines stay off — Zephyr's
        // transport is custom (CMake glue wires the callbacks).
        disabled.push("UCLIENT_PROFILE_UDP");
        disabled.push("UCLIENT_PROFILE_TCP");
        disabled.push("UCLIENT_PROFILE_SERIAL");
        disabled.push("UCLIENT_PLATFORM_POSIX");
        disabled.push("UCLIENT_PLATFORM_POSIX_NOPOLL");
        disabled.push("UCLIENT_PLATFORM_WINDOWS");
        disabled.push("UCLIENT_PLATFORM_FREERTOS_PLUS_TCP");
        disabled.push("UCLIENT_PLATFORM_RTEMS_BSD_NET");
    } else {
        // Bare-metal / FreeRTOS / NuttX / ThreadX.
        disabled.push("UCLIENT_PROFILE_UDP");
        disabled.push("UCLIENT_PROFILE_TCP");
        disabled.push("UCLIENT_PROFILE_SERIAL");
        disabled.push("UCLIENT_PLATFORM_POSIX");
        disabled.push("UCLIENT_PLATFORM_POSIX_NOPOLL");
        disabled.push("UCLIENT_PLATFORM_WINDOWS");
        disabled.push("UCLIENT_PLATFORM_FREERTOS_PLUS_TCP");
        disabled.push("UCLIENT_PLATFORM_RTEMS_BSD_NET");
        disabled.push("UCLIENT_PLATFORM_ZEPHYR");
    }
    // Match the entire line (`\n` boundary) so e.g. the
    // `UCLIENT_PLATFORM_POSIX` rule does not accidentally also
    // match `UCLIENT_PLATFORM_POSIX_NOPOLL`.
    for name in enabled {
        h = h.replace(
            &format!("#cmakedefine {name}\n"),
            &format!("#define {name}\n"),
        );
    }
    for name in disabled {
        h = h.replace(
            &format!("#cmakedefine {name}\n"),
            &format!("/* #undef {name} */\n"),
        );
    }
    let dir = out_dir.join("include/uxr/client");
    fs::create_dir_all(&dir).unwrap();
    fs::write(dir.join("config.h"), h).unwrap();
}

/// A pool knob's value: explicit env var, else the Zephyr Kconfig it is
/// resolved from, else `None` (the vendored header's own default stands).
///
/// issue 0460 — every one of these is forwarded by `_nros_resolve_knob()` in
/// `zephyr/cmake/nros_cargo_build.cmake` with `set(ENV{...})`, which reaches
/// the C lane's re-baked command and NOT the Rust lane's: zephyr-lang-rust's
/// `rust_cargo_application` builds its own cargo invocation and inherits
/// nothing. So a Zephyr Rust image read every one of these as unset whatever
/// Kconfig said. The env name here is the Kconfig name minus `CONFIG_`, so the
/// pair is derived; `check-kconfig-knob-forwarding` proves the cmake list and
/// the readers agree.
fn knob(env_name: &str) -> Option<String> {
    if let Ok(v) = env::var(env_name) {
        return Some(v);
    }
    nros_zephyr_build::dotconfig_usize(&format!("CONFIG_{env_name}")).map(|v| v.to_string())
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
