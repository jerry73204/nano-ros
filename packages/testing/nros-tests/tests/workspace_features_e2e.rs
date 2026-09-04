//! phase-295 W3.b — THE native workspace-feature matrix consumer (RFC-0051).
//!
//! Consolidates the per-cell native workspace-workload files —
//! `{c_,cpp_,mixed_}custom_msg_workspace_e2e`,
//! `{,c_,cpp_,mixed_}logging_workspace_e2e`,
//! `{c_,cpp_,mixed_}qos_workspace_e2e`, `lifecycle_workspace_e2e` +
//! `cpp_c_lifecycle_autostart_e2e`, and `safety_workspace_e2e` +
//! `cpp_c_safety_integrity_e2e` — into one parametrized test over the
//! native `Workload::{CustomMsg,Logging,Qos,Lifecycle,Safety,Remap}` workspace
//! cells of the test matrix (`nros_tests::matrix`). The task list named
//! only the two rust lifecycle/safety files, but the C/C++ halves of those
//! matrix rows lived in the `cpp_c_*` twin files — they are the same
//! family and fold in here (the `cpp_lifecycle_node_wrapper_e2e` stays: it
//! pins the `nros::LifecycleNode` WRAPPER API, not the workspace cell).
//!
//! Observation styles, preserved 1:1 from the per-cell files ([`Proof`]):
//! - **CustomMsg** (phase-263 B6): the workspace-local `custom_msgs/Reading`
//!   schema flows cross-process — C/C++/mixed talker + listener entries,
//!   the listener prints ≥3 decoded `reading seq=` lines AND the `temp=`
//!   second field (full CDR layout, not just a counter).
//! - **Logging** (phase-263 A5 / phase-264 W3): a Node pkg's
//!   `log_info!` / `NROS_LOG_INFO(nros_log_default_logger(), …)` reaches
//!   the entry's OWN stdout — process-local, no subscriber (issue 0096
//!   does not apply to logging). Per-lang markers differ (the mixed ws
//!   reuses the C talker).
//! - **Qos** (phase-263 B4): a NON-DEFAULT per-entity profile (reliable +
//!   transient_local + keep_last(10), set IN CODE on both endpoints)
//!   connects + delivers cross-process; the talker boots FIRST so the
//!   listener joins late. No transient-local *replay* assertion (zenoh
//!   provides none out of the box) — a QoS mismatch delivers nothing.
//! - **Lifecycle** (phase-263 A3 / phase-269 W2): `[lifecycle] autostart =
//!   "active"` drives Configure→Activate at boot with NO manual `ros2
//!   lifecycle set`, observed over the REP-2002 service surface (requires
//!   ROS 2 + rmw_zenoh_cpp; skips when absent).
//! - **Safety** (phase-263 B1 / phase-269 W3): CRC-validated delivery —
//!   talker attaches a CRC per `/chatter` publish, the listener's
//!   validated subscription republishes the CRC-valid count on `/safe_ok`,
//!   an external `int32-sink` sees the count climb.
//! - **Remap** (phase-306 W4 / issue 0255): the model namespaces the node
//!   under `/island` and remaps its PRIVATE `~/out` to `/remapped_out`; a
//!   sink on the remapped topic receives, one on the `~` expansion stays
//!   silent — the launch/model remap reaches the WIRE.
//!
//! All cells are native. The pre-consolidation files pinned arbitrary
//! fixed router ports (17881–17883 logging, 17911/17933/17934 custom-msg,
//! 17921/17931/17932 qos); none is a fixture bake — `examples/fixtures.toml`
//! carries NO port/locator for any of these workspaces and every native
//! entry takes `NROS_LOCATOR` at runtime — so the consolidation moves the
//! whole family onto `ZenohRouter::start_unique` ephemeral isolation
//! (lifecycle/safety already used ephemeral ports).
//!
//! Run with: `cargo nextest run -p nros-tests --test workspace_features_e2e`
//! (filter one workload: `-E 'binary(workspace_features_e2e) and test(qos)'`).

use nros_tests::{
    TestResult,
    fixtures::{
        ManagedProcess, ZenohRouter, build_native_workspace_c_custom_msg_listener_entry,
        build_native_workspace_c_custom_msg_talker_entry, build_native_workspace_c_entry,
        build_native_workspace_c_lifecycle_entry, build_native_workspace_c_qos_listener_entry,
        build_native_workspace_c_qos_talker_entry, build_native_workspace_c_safety_listener_entry,
        build_native_workspace_c_safety_talker_entry,
        build_native_workspace_cpp_custom_msg_listener_entry,
        build_native_workspace_cpp_custom_msg_talker_entry, build_native_workspace_cpp_entry,
        build_native_workspace_cpp_lifecycle_entry, build_native_workspace_cpp_qos_listener_entry,
        build_native_workspace_cpp_qos_talker_entry,
        build_native_workspace_cpp_safety_listener_entry,
        build_native_workspace_cpp_safety_talker_entry,
        build_native_workspace_mixed_custom_msg_listener_entry,
        build_native_workspace_mixed_custom_msg_talker_entry, build_native_workspace_mixed_entry,
        build_native_workspace_mixed_qos_listener_entry,
        build_native_workspace_mixed_qos_talker_entry, build_native_workspace_rust_entry,
        build_native_workspace_rust_lifecycle_entry, build_native_workspace_rust_remap_entry,
        build_native_workspace_rust_safety_listener_entry,
        build_native_workspace_rust_safety_talker_entry, require_zenohd,
    },
    matrix::{Cell as MCell, Lang as ML, Tier as MT, W1Consumer, Workload as MW, w1_consumer_of},
    ros2::{DEFAULT_ROS_DISTRO, require_ros2, ros2_env_setup_with_locator},
};
use rstest::rstest;
use std::{
    path::PathBuf,
    process::Command,
    time::{Duration, Instant},
};

// =============================================================================
// Cell table types
// =============================================================================

type Resolver = fn() -> TestResult<PathBuf>;

/// The per-cell topology + assertion, preserved 1:1 from the
/// pre-consolidation files.
#[derive(Copy, Clone, Debug)]
enum Proof {
    /// Talker-first pair; the listener must print ≥3 decoded
    /// `reading seq=` lines AND the `temp=` second field.
    CustomMsgFields,
    /// Single entry; ≥3 lines carrying the per-lang log marker must reach the
    /// entry's OWN stdout (process-local — no subscriber), AND each of those
    /// lines must carry the `[INFO]` level tag the logging facade adds.
    ///
    /// Issue 0309 — the marker alone is the message TEXT, so a bare
    /// `printf("<marker> seq=%d")` would satisfy it: the proof could not tell
    /// the facade from a direct write, nor notice the level/logger metadata
    /// being lost. Every one of these nodes logs through
    /// `log_info!` / `NROS_LOG_INFO` → the default sink → the posix writer,
    /// whose output is `[INFO] nros: <marker> seq=N`, so requiring the tag on
    /// the SAME line makes the assertion about the facade rather than about
    /// stdout.
    LoggingLines(&'static str),
    /// Talker-first pair with the non-default QoS profile in code on both
    /// endpoints: the late-joining listener must print ≥3 `Received:`, AND
    /// (when ROS 2 is present) a stock `rmw_zenoh_cpp` peer must report the
    /// DECLARED profile on both endpoints.
    ///
    /// Issue 0309 — the count alone proved nothing about QoS. A default
    /// publisher talking to a default subscriber delivers exactly as many
    /// messages as a QoS-matched pair, so this proof stayed green through
    /// issue 0306, which made every declarative Rust entity run
    /// `QoSProfile::default()`. The profile assertion is what makes the cell
    /// about QoS rather than about delivery.
    QosMatchedProfile {
        /// The topic the pair uses — the C-family demos publish on `/chatter`.
        topic: &'static str,
    },
    /// Single autostart entry; `ros2 lifecycle get` on the discovered
    /// managed node must report `active` with no manual transition.
    LifecycleActive,
    /// listener entry + talker entry + external `/safe_ok` sink; the sink
    /// must see ≥3 climbing CRC-valid counts. Per-cell spin/wait budgets
    /// preserved from the rust (16 s/22 s) vs C-family (20 s/25 s) files.
    SafetyCrcCount { spin_ms: u32, wait_secs: u64 },
    /// Single entry publishing on a PRIVATE `~/out` name the model remaps
    /// (phase-306 W4, issue 0255): a sink on the REMAPPED absolute topic
    /// must see ≥3 messages, and a sink on the unremapped `~` expansion
    /// must stay silent — the remap reached the WIRE.
    RemapWireName,
}

/// The per-cell EXECUTION data for one native workspace-feature matrix cell.
/// The coordinate lives in `matrix::Cell`; this carries the entr(y/ies) + proof.
/// Keyed by coordinate in [`exec_for`].
struct Exec {
    /// The (only / first-booted) entry: the logging/lifecycle entry, or
    /// the pair's TALKER (custom-msg, qos) / safety talker.
    entry: Resolver,
    /// The pair's LISTENER entry (`None` for single-entry workloads).
    peer: Option<Resolver>,
    proof: Proof,
    /// Provenance / nuance — folded into failure messages so a red cell
    /// still names the seam it pins.
    note: &'static str,
}

fn wl_str(w: MW) -> &'static str {
    match w {
        MW::CustomMsg => "custom_msg",
        MW::Logging => "logging",
        MW::Qos => "qos",
        MW::Lifecycle => "lifecycle",
        MW::Safety => "safety",
        MW::Remap => "remap",
        _ => "?",
    }
}

/// Map a native workspace-feature `(lang, workload)` coordinate to its execution
/// data. An unmapped coordinate is a HARD panic: adding a cell that
/// `w1_consumer_of` assigns to `WorkspaceFeatures` forces an arm here
/// (phase-329 W1).
fn exec_for(lang: ML, workload: MW) -> Exec {
    match (lang, workload) {
        (ML::C, MW::CustomMsg) => Exec {
            entry: || build_native_workspace_c_custom_msg_talker_entry().map(|p| p.to_path_buf()),
            peer: Some(|| {
                build_native_workspace_c_custom_msg_listener_entry().map(|p| p.to_path_buf())
            }),
            proof: Proof::CustomMsgFields,
            note: "phase-263 B6 C projection: reading_{talker,listener}_pkg carry the type name \
                   as a string + hand-code the CDR (RFC-0043 typed-component idiom); the \
                   differentiator is the WORKSPACE-LOCAL schema",
        },
        (ML::Cpp, MW::CustomMsg) => Exec {
            entry: || build_native_workspace_cpp_custom_msg_talker_entry().map(|p| p.to_path_buf()),
            peer: Some(|| {
                build_native_workspace_cpp_custom_msg_listener_entry().map(|p| p.to_path_buf())
            }),
            proof: Proof::CustomMsgFields,
            note: "phase-263 B6 C++ projection: raw-CDR idiom, no generated interface archive \
                   linked — dodges any cpp codegen edge",
        },
        (ML::Mixed, MW::CustomMsg) => Exec {
            entry: || {
                build_native_workspace_mixed_custom_msg_talker_entry().map(|p| p.to_path_buf())
            },
            peer: Some(|| {
                build_native_workspace_mixed_custom_msg_listener_entry().map(|p| p.to_path_buf())
            }),
            proof: Proof::CustomMsgFields,
            note: "phase-263 B6 MIXED projection: the C reading pkgs reused verbatim, driven by \
                   a C++ TYPED entry carrier",
        },
        (ML::Rust, MW::Logging) => Exec {
            entry: || build_native_workspace_rust_entry().map(|p| p.to_path_buf()),
            peer: None,
            proof: Proof::LoggingLines(nros_tests::output::WS_RUST_LOGGING_MARKER),
            note: "phase-338 W7: talker_pkg logs through the `log` facade like every other \
                   example body, and nros-board-linux's stdout bridge carries it — no \
                   per-app logging init. This cell USED to prove the same for the \
                   nros_log chain (`log_info!` + the board's boot-time platform sink); \
                   that property now rests on the C and C++ siblings below, which reach \
                   the identical board mechanism through their projections",
        },
        (ML::C, MW::Logging) => Exec {
            entry: || build_native_workspace_c_entry().map(|p| p.to_path_buf()),
            peer: None,
            proof: Proof::LoggingLines(nros_tests::output::WS_C_LOGGING_MARKER),
            note: "phase-263 A5 C projection: NROS_LOG_INFO(nros_log_default_logger(), …) — a \
                   NULL logger handle DROPS the record (the A5 C/C++ finding)",
        },
        (ML::Cpp, MW::Logging) => Exec {
            entry: || build_native_workspace_cpp_entry().map(|p| p.to_path_buf()),
            peer: None,
            proof: Proof::LoggingLines(nros_tests::output::WS_CPP_LOGGING_MARKER),
            note: "phase-263 A5 C++ projection: same facade chain (nros_log_emit_fmt → \
                   DEFAULT_LOGGER → lazy default sink → posix writer)",
        },
        (ML::Mixed, MW::Logging) => Exec {
            entry: || build_native_workspace_mixed_entry().map(|p| p.to_path_buf()),
            peer: None,
            proof: Proof::LoggingLines(nros_tests::output::WS_C_LOGGING_MARKER),
            note: "phase-263 A5 MIXED projection: the mixed ws reuses the C talker, so its cell \
                   greps the C marker",
        },
        (ML::C, MW::Qos) => Exec {
            entry: || build_native_workspace_c_qos_talker_entry().map(|p| p.to_path_buf()),
            peer: Some(|| build_native_workspace_c_qos_listener_entry().map(|p| p.to_path_buf())),
            proof: Proof::QosMatchedProfile { topic: "/chatter" },
            note: "phase-263 B4 C projection: nros_cpp_qos_t by value to nros_cpp_publisher_create \
                   (not nros_c_qos_default()); listener declares the BYTE-IDENTICAL profile",
        },
        (ML::Cpp, MW::Qos) => Exec {
            entry: || build_native_workspace_cpp_qos_talker_entry().map(|p| p.to_path_buf()),
            peer: Some(|| build_native_workspace_cpp_qos_listener_entry().map(|p| p.to_path_buf())),
            proof: Proof::QosMatchedProfile { topic: "/chatter" },
            note: "phase-263 B4 C++ projection: fluent nros::QoS builder \
                   (.reliable().transient_local().keep_last(10)) into Node::create_publisher",
        },
        (ML::Mixed, MW::Qos) => Exec {
            entry: || build_native_workspace_mixed_qos_talker_entry().map(|p| p.to_path_buf()),
            peer: Some(|| {
                build_native_workspace_mixed_qos_listener_entry().map(|p| p.to_path_buf())
            }),
            proof: Proof::QosMatchedProfile { topic: "/chatter" },
            note: "phase-263 B4 MIXED projection: the C qos pkgs reused verbatim under a C++ \
                   TYPED entry carrier (run_components)",
        },
        (ML::Rust, MW::Lifecycle) => Exec {
            entry: || build_native_workspace_rust_lifecycle_entry().map(|p| p.to_path_buf()),
            peer: None,
            proof: Proof::LifecycleActive,
            note: "phase-263 A3: `[lifecycle] autostart = \"active\"` + nros/lifecycle-services — \
                   nros::main! (phase-264 W2) registers the 5 REP-2002 services and drives \
                   Configure→Activate at boot",
        },
        (ML::C, MW::Lifecycle) => Exec {
            entry: || build_native_workspace_c_lifecycle_entry().map(|p| p.to_path_buf()),
            peer: None,
            proof: Proof::LifecycleActive,
            note: "phase-269 W2 C: the generated __nros_entry_setup (emit_c.rs) calls \
                   nros_cpp_lifecycle_autostart(executor, 2u)",
        },
        (ML::Cpp, MW::Lifecycle) => Exec {
            entry: || build_native_workspace_cpp_lifecycle_entry().map(|p| p.to_path_buf()),
            peer: None,
            proof: Proof::LifecycleActive,
            note: "phase-269 W2 C++: __nros_entry_setup calls nros_cpp_lifecycle_autostart(__exec, \
                   2u) via ::nros::global_handle()",
        },
        (ML::Rust, MW::Safety) => Exec {
            entry: || build_native_workspace_rust_safety_talker_entry().map(|p| p.to_path_buf()),
            peer: Some(|| {
                build_native_workspace_rust_safety_listener_entry().map(|p| p.to_path_buf())
            }),
            proof: Proof::SafetyCrcCount {
                spin_ms: 16000,
                wait_secs: 22,
            },
            note: "phase-263 B1: entry bakes `safety-e2e` → backend-attached CRC-32 + seq per \
                   publish; safe_listener validates + reads CallbackCtx::integrity() and \
                   republishes the valid count",
        },
        (ML::C, MW::Safety) => Exec {
            entry: || build_native_workspace_c_safety_talker_entry().map(|p| p.to_path_buf()),
            peer: Some(|| {
                build_native_workspace_c_safety_listener_entry().map(|p| p.to_path_buf())
            }),
            proof: Proof::SafetyCrcCount {
                spin_ms: 20000,
                wait_secs: 25,
            },
            note: "phase-269 W3 C: nros_cpp_subscription_register_validated callback receives \
                   crc_valid == 1 per integrity-passing frame (NANO_ROS_SAFETY_E2E=ON fixture)",
        },
        (ML::Cpp, MW::Safety) => Exec {
            entry: || build_native_workspace_cpp_safety_talker_entry().map(|p| p.to_path_buf()),
            peer: Some(|| {
                build_native_workspace_cpp_safety_listener_entry().map(|p| p.to_path_buf())
            }),
            proof: Proof::SafetyCrcCount {
                spin_ms: 20000,
                wait_secs: 25,
            },
            note: "phase-269 W3 C++: node.create_subscription_with_safety<M>() delivers \
                   (const M&, const nros_cpp_integrity_status_t&) with crc_valid == 1",
        },
        (ML::Rust, MW::Remap) => Exec {
            entry: || build_native_workspace_rust_remap_entry().map(|p| p.to_path_buf()),
            peer: None,
            proof: Proof::RemapWireName,
            note: "phase-306 W3/W4: nros::main!(model = …) bakes the model's <remap> rules into \
                   runtime.remaps; entity creation expands ~/out against /island/remap_talker \
                   and resolves it to /remapped_out before it reaches the RMW",
        },
        (l, w) => panic!(
            "workspace_features_e2e: no execution mapping for matrix cell {l:?}/{w:?} — add an \
             `exec_for` arm (phase-329 W1)"
        ),
    }
}

// =============================================================================
// Shared helpers
// =============================================================================

/// The level tag the nano-ros logging facade's posix sink writes ahead of every
/// record (`[INFO] nros: <message>`). Issue 0309 — asserted alongside each
/// logging marker so the proof cannot be satisfied by a direct `printf`.
const LOG_LEVEL_TAG: &str = "[INFO]";

/// Spawn a C-family custom-msg/qos entry (spins on its own; only the
/// locator — the pre-consolidation shape).
fn spawn_locator_only(entry: &PathBuf, label: &'static str, locator: &str) -> ManagedProcess {
    let mut cmd = Command::new(entry);
    cmd.env("NROS_LOCATOR", locator);
    ManagedProcess::spawn_command(cmd, label).unwrap_or_else(|e| panic!("spawn {label}: {e}"))
}

/// Spawn a hosted-spin entry (`nros::main!` / generated C entry) with the
/// standard env set. `step_ms: None` preserves the C-family logging shape
/// (those files never set `NROS_ENTRY_SPIN_STEP_MS`).
fn spawn_spinning(
    entry: &PathBuf,
    label: &'static str,
    locator: &str,
    spin_ms: u32,
    step_ms: Option<u32>,
) -> ManagedProcess {
    let mut cmd = Command::new(entry);
    cmd.env("RUST_LOG", "info")
        .env("NROS_LOCATOR", locator)
        .env("NROS_SESSION_MODE", "client")
        .env("NROS_ENTRY_SPIN_MS", spin_ms.to_string());
    if let Some(step) = step_ms {
        cmd.env("NROS_ENTRY_SPIN_STEP_MS", step.to_string());
    }
    ManagedProcess::spawn_command(cmd, label).unwrap_or_else(|e| panic!("spawn {label}: {e}"))
}

/// Run `ros2 <subcommand>` against `locator`; return combined stdout+stderr.
fn run_ros2(locator: &str, subcommand: &str) -> String {
    let (env, _config_guard) = ros2_env_setup_with_locator(DEFAULT_ROS_DISTRO, locator);
    let script = format!("{env} && timeout 10 ros2 {subcommand} 2>&1");
    let out = Command::new("bash")
        .args(["-c", &script])
        .output()
        .expect("failed to spawn bash for ros2 invocation");
    String::from_utf8_lossy(&out.stdout).into_owned()
}

/// Poll `ros2 <subcommand>` until its output contains `marker`
/// (case-insensitive) or timeout.
fn poll_ros2_until(locator: &str, subcommand: &str, marker: &str, timeout: Duration) -> String {
    let deadline = Instant::now() + timeout;
    let marker_lc = marker.to_lowercase();
    let mut last = String::new();
    while Instant::now() < deadline {
        last = run_ros2(locator, subcommand);
        if last.to_lowercase().contains(&marker_lc) {
            return last;
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    last
}

/// First `/`-prefixed node name in `ros2 lifecycle nodes` output, if any.
fn first_lifecycle_node(nodes_out: &str) -> Option<String> {
    nodes_out
        .lines()
        .map(|l| l.trim())
        .find(|l| l.starts_with('/'))
        .map(|l| l.to_string())
}

/// Resolve a cell entry, skipping when the fixture is not built.
fn resolve(r: Resolver, lang: &str, workload: &str, role: &str) -> PathBuf {
    r().unwrap_or_else(|e| nros_tests::skip!("{lang} {workload} {role} entry not built: {e}"))
}

// =============================================================================
// The parametrized matrix consumer
// =============================================================================

/// THE native workspace-feature matrix consumer (phase-329 W1). Iterates every
/// cell `w1_consumer_of` assigns to `WorkspaceFeatures` (native
/// CustomMsg/Logging/Qos/Lifecycle/Safety/Remap) — derived from `matrix::CELLS`
/// — and runs each in one process, catching per-cell skips/failures so one
/// missing fixture never aborts the rest.
/// The cells this file consumes — ONE definition, shared by the per-case lookup
/// and the coverage tripwire so they cannot disagree.
fn is_wsfeature_cell(c: &MCell) -> bool {
    w1_consumer_of(c) == Some(W1Consumer::WorkspaceFeatures) && matches!(c.tier, MT::Runtime)
}

/// The `(lang, workload)` pairs the `#[case]`s declare. Every cell here is
/// Zenoh, so the pair is unique. A SPARSE grid, not a product: rust carries no
/// CustomMsg/Qos cell and mixed carries only three — which is why this list is
/// written out and gated rather than generated from the enums.
const DECLARED_CASES: &[(ML, MW)] = &[
    (ML::Rust, MW::Lifecycle),
    (ML::Rust, MW::Logging),
    (ML::Rust, MW::Remap),
    (ML::Rust, MW::Safety),
    (ML::C, MW::CustomMsg),
    (ML::C, MW::Lifecycle),
    (ML::C, MW::Logging),
    (ML::C, MW::Qos),
    (ML::C, MW::Safety),
    (ML::Cpp, MW::CustomMsg),
    (ML::Cpp, MW::Lifecycle),
    (ML::Cpp, MW::Logging),
    (ML::Cpp, MW::Qos),
    (ML::Cpp, MW::Safety),
    (ML::Mixed, MW::CustomMsg),
    (ML::Mixed, MW::Logging),
    (ML::Mixed, MW::Qos),
];

/// THE consumer — ONE TEST PER CELL (phase-342 W1). Was a single #[test]
/// folding 17 cells at 62.9 s; see `native_example_pubsub_e2e.rs` for the full
/// reasoning. Same two wins: the cells now schedule, and a failure names the
/// `(lang, workload)` instead of reporting `N of 17`.
#[rstest]
#[case::rust_lifecycle(ML::Rust, MW::Lifecycle)]
#[case::rust_logging(ML::Rust, MW::Logging)]
#[case::rust_remap(ML::Rust, MW::Remap)]
#[case::rust_safety(ML::Rust, MW::Safety)]
#[case::c_custom_msg(ML::C, MW::CustomMsg)]
#[case::c_lifecycle(ML::C, MW::Lifecycle)]
#[case::c_logging(ML::C, MW::Logging)]
#[case::c_qos(ML::C, MW::Qos)]
#[case::c_safety(ML::C, MW::Safety)]
#[case::cpp_custom_msg(ML::Cpp, MW::CustomMsg)]
#[case::cpp_lifecycle(ML::Cpp, MW::Lifecycle)]
#[case::cpp_logging(ML::Cpp, MW::Logging)]
#[case::cpp_qos(ML::Cpp, MW::Qos)]
#[case::cpp_safety(ML::Cpp, MW::Safety)]
#[case::mixed_custom_msg(ML::Mixed, MW::CustomMsg)]
#[case::mixed_logging(ML::Mixed, MW::Logging)]
#[case::mixed_qos(ML::Mixed, MW::Qos)]
fn workspace_features(#[case] lang: ML, #[case] workload: MW) {
    let cell = nros_tests::matrix::CELLS
        .iter()
        .find(|c| is_wsfeature_cell(c) && c.lang == lang && c.workload == workload)
        .unwrap_or_else(|| {
            panic!(
                "matrix regression: no WorkspaceFeatures runtime cell for {}/{}",
                lang.as_str(),
                wl_str(workload)
            )
        });
    run_cell(cell);
}

/// Tripwire — keeps the hand-written `#[case]` list bound to the derived truth.
/// Load-bearing here in particular, because the grid is sparse: an eyeball
/// cannot tell a missing cell from a deliberately absent one.
#[test]
fn workspace_features_cases_cover_every_matrix_cell() {
    use std::collections::BTreeSet;
    let key = |l: ML, w: MW| (l.as_str().to_string(), wl_str(w).to_string());
    let from_matrix: BTreeSet<_> = nros_tests::matrix::CELLS
        .iter()
        .filter(|c| is_wsfeature_cell(c))
        .map(|c| key(c.lang, c.workload))
        .collect();
    let declared: BTreeSet<_> = DECLARED_CASES.iter().map(|(l, w)| key(*l, *w)).collect();

    assert!(
        !from_matrix.is_empty(),
        "matrix regression: no WorkspaceFeatures runtime cells for this consumer"
    );
    let missing: Vec<_> = from_matrix.difference(&declared).collect();
    let extra: Vec<_> = declared.difference(&from_matrix).collect();
    assert!(
        missing.is_empty() && extra.is_empty(),
        "workspace_features #[case] list has drifted from matrix::CELLS.\n  \
         cells with no case (would never run): {missing:?}\n  \
         cases with no cell (would panic):     {extra:?}"
    );
}

/// Boot the workspace entr(y/ies) on an ephemeral router and prove the
/// workload's contract per the cell's [`Proof`]. Panics with `[SKIPPED] …` on an
/// unmet precondition; the caller classifies.
fn run_cell(pcell: &MCell) {
    let lang = pcell.lang.as_str();
    let workload = wl_str(pcell.workload);
    let cell = exec_for(pcell.lang, pcell.workload);
    // Gate: lifecycle cells assert over the ros2 CLI (skip without ROS 2 +
    // rmw_zenoh_cpp — same contract as the other interop tests); everything
    // else needs only zenohd.
    if matches!(cell.proof, Proof::LifecycleActive) {
        if !require_ros2() {
            nros_tests::skip!(
                "ROS 2 / rmw_zenoh_cpp not available — install it from apt \
             (`ros-$ROS_DISTRO-rmw-zenoh-cpp`, declared in nros-sdk-index.toml)."
            );
        }
    } else if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    let entry = resolve(cell.entry, lang, workload, "talker/entry");
    let peer = cell.peer.map(|r| resolve(r, lang, workload, "listener"));

    // Native-only family: every cell gets an ephemeral router (no fixture
    // bakes a port for any of these workspaces — see the module doc).
    let router = ZenohRouter::start_unique()
        .unwrap_or_else(|e| nros_tests::skip!("zenohd failed to start: {e}"));
    let locator = router.locator();

    match cell.proof {
        Proof::CustomMsgFields => {
            // Talker first so the publisher is discoverable when the
            // listener joins.
            let mut tlk = spawn_locator_only(&entry, "custom-msg-talker", &locator);
            tlk.wait_for_output_pattern(
                nros_tests::output::WS_CUSTOM_MSG_SENT_PREFIX,
                Duration::from_secs(10),
            )
            .unwrap_or_else(|_| {
                tlk.kill();
                panic!(
                    "[{} {}] reading_talker never published ({})",
                    lang, workload, cell.note
                )
            });
            let peer = peer.expect("custom-msg cells carry a listener entry");
            let mut lis = spawn_locator_only(&peer, "custom-msg-listener", &locator);

            let prefix = nros_tests::output::WS_CUSTOM_MSG_READING_PREFIX;
            let out = lis
                .wait_for_output_count(prefix, 3, Duration::from_secs(60))
                .unwrap_or_else(|_| {
                    lis.kill();
                    tlk.kill();
                    panic!(
                        "[{} {}] reading_listener never received 3 custom-msg samples — \
                         the cross-process workspace-local custom-message delivery did \
                         not work ({})",
                        lang, workload, cell.note
                    )
                });
            lis.kill();
            tlk.kill();

            // The talker ramps `sequence` 0,1,2,…; early pre-discovery
            // samples may be missed, so assert the field appears ≥3× rather
            // than a strict value prefix.
            let n = nros_tests::count_pattern(&out, prefix);
            assert!(
                n >= 3,
                "[{} {}] expected ≥3 custom-msg receives, got {n}.\n{out}",
                lang,
                workload
            );
            // The decoded temperature field must also be present (a
            // non-trivial second field), proving the full CDR layout — not
            // just a counter — survives the round-trip.
            assert!(
                out.contains(nros_tests::output::WS_CUSTOM_MSG_TEMP_FIELD),
                "[{} {}] listener output missing the decoded temperature field — CDR \
                 decode wrong ({}).\n{out}",
                lang,
                workload,
                cell.note
            );
        }

        Proof::LoggingLines(marker) => {
            // Rust's `nros::main!` hosted spin historically also got a
            // STEP_MS; the C-family entries never did — preserved.
            let step = (lang == "rust").then_some(10);
            let mut proc = spawn_spinning(&entry, "logging-entry", &locator, 8000, step);

            // The talker logs once per 1 Hz tick; 3 lines confirms the sink
            // is live and the node's log calls keep reaching stdout.
            let out = proc
                .wait_for_output_count(marker, 3, Duration::from_secs(18))
                .unwrap_or_else(|_| {
                    proc.kill();
                    panic!(
                        "[{} {}] the workspace node's log line never reached the entry's \
                         stdout — the node-log facade chain is broken ({})",
                        lang, workload, cell.note
                    )
                });
            proc.kill();

            // Issue 0309 — count lines carrying the marker AND the level tag
            // the facade emits, not merely the marker.
            let n = out
                .lines()
                .filter(|l| l.contains(marker) && l.contains(LOG_LEVEL_TAG))
                .count();
            assert!(
                n >= 3,
                "[{} {}] expected ≥3 node log lines carrying `{LOG_LEVEL_TAG}` on stdout, got \
                 {n}. A line with the marker but no level tag means the record bypassed the \
                 logging facade (a direct write) or lost its metadata.\n{out}",
                lang,
                workload
            );
        }

        Proof::QosMatchedProfile { topic } => {
            // Talker first (the QoS-tagged publisher boots + keeps
            // publishing at 1 Hz), so the listener joins LATE — proving the
            // QoS-matched endpoints discover + connect across processes.
            let mut tlk = spawn_locator_only(&entry, "qos-talker", &locator);
            tlk.wait_for_output_pattern(
                nros_tests::output::INT32_TALKER_LOG_PREFIX,
                Duration::from_secs(10),
            )
            .unwrap_or_else(|_| {
                tlk.kill();
                panic!(
                    "[{} {}] qos_talker never published ({})",
                    lang, workload, cell.note
                )
            });
            let peer = peer.expect("qos cells carry a listener entry");
            // Issue 0309 — a spin budget, not `spawn_locator_only`: the profile
            // check below reads what BOTH endpoints advertise, and a listener
            // that has already exited reports `Subscription count: 0`. The
            // budget only has to outlive the report.
            let mut lis = spawn_spinning(&peer, "qos-listener", &locator, 30000, None);

            let prefix = nros_tests::output::INT32_LISTENER_LOG_PREFIX;
            let out = lis
                .wait_for_output_count(prefix, 3, Duration::from_secs(60))
                .unwrap_or_else(|_| {
                    lis.kill();
                    tlk.kill();
                    panic!(
                        "[{} {}] qos_listener never received 3 QoS-matched samples — the \
                         cross-process per-entity QoS-matched delivery did not work (QoS \
                         mismatch or wiring break) ({})",
                        lang, workload, cell.note
                    )
                });
            // Issue 0309 — the delivery count above cannot tell the DECLARED
            // profile from any compatible one, so read what the endpoints
            // actually advertise while both are still alive. Skips (rather than
            // fails) without ROS 2, so the cell keeps its delivery coverage
            // everywhere and gains the profile check where a peer exists.
            if require_ros2() {
                // POLL, do not sample once. ROS 2 discovery is eventually
                // consistent: the delivery wait above already proves both
                // endpoints exist and are QoS-matched, but a liveliness token
                // reaching THIS `ros2` invocation is a separate event, and
                // under sweep load it can lag the first message by seconds.
                //
                // Issue 0705 — a single-shot query is a race by construction,
                // and the way it failed hid that. The report showed
                // `Publisher count: 1` naming a node `talker` (some other
                // test's; 24 launch files use that name on /chatter) while this
                // cell's `qos_talker` was absent — so it read as "the wrong
                // graph" rather than "not yet propagated". Selecting by node,
                // which issue 0690 added, already makes a foreign endpoint
                // harmless; what remained was asserting before our own had
                // arrived.
                //
                // This does not weaken the assertion: it still requires OUR
                // node's endpoint and still asserts the full declared profile.
                // On timeout it fails exactly as before, carrying the last
                // report.
                // The poll this loop used to spell inline now lives in
                // `await_topic_endpoints` — issue 0761 found `qos_override_e2e`
                // still single-shotting, i.e. this remedy had been applied at
                // one site and not its sibling. One helper, so the next site is
                // a call rather than a re-derivation.
                let (report, found) = nros_tests::ros2::await_topic_endpoints(
                    &locator,
                    DEFAULT_ROS_DISTRO,
                    topic,
                    &[
                        ("PUBLISHER", "qos_talker"),
                        ("SUBSCRIPTION", "qos_listener"),
                    ],
                    Duration::from_secs(20),
                )
                .unwrap_or_else(|e| {
                    lis.kill();
                    tlk.kill();
                    panic!("[{} {}] ros2 topic info failed: {e}", lang, workload)
                });
                let (pub_eps, sub_eps) = (found[0].clone(), found[1].clone());
                // Issue 0312 (fixed) — both endpoints are asserted. The
                // subscription used to be absent from discovery entirely: these
                // C examples pass an EMPTY type hash, which landed in the
                // liveliness keyexpr as an empty segment, so `ros2 topic info`
                // never counted the entity. Delivery was unaffected, so only a
                // profile assertion like this one could see it.
                // issue 0690 — select by the NODE under test, not by position.
                //
                // `topic_endpoint_block` returns the FIRST block of a kind, so
                // any other publisher on `/chatter` could be the one asserted
                // against. That is what `case_08_c_qos` did when it failed
                // in-sweep and passed solo: the profile it printed was exactly
                // `nros_c_qos_default()`, i.e. somebody else's endpoint.
                //
                // The launch files name these nodes (`demo_bringup/launch/
                // *qos*.xml`), and all three qos cells use the SAME two names —
                // so this rules out a foreign process but NOT a sibling cell.
                // That is why more than one match is reported rather than
                // silently taking the first: the two need different remedies,
                // and collapsing them here would rebuild the bug one level up.
                let blocks: Vec<(&str, &str, Vec<nros_tests::ros2::TopicEndpoint>)> = vec![
                    ("PUBLISHER", "qos_talker", pub_eps),
                    ("SUBSCRIPTION", "qos_listener", sub_eps),
                ];
                for (kind, node, found) in blocks {
                    if found.len() > 1 {
                        lis.kill();
                        tlk.kill();
                        panic!(
                            "[{} {}] {} {kind} endpoints on {topic} name themselves `{node}` \
                             ({}). Node name cannot pick between them — this is a SIBLING \
                             cell, not a foreign process, and needs the GID or the namespace \
                             as the discriminator (issue 0690).\nfull report:\n{report}",
                            lang,
                            workload,
                            found.len(),
                            cell.note
                        );
                    }
                    let block = found
                        .into_iter()
                        .next()
                        .map(|e| e.block)
                        .unwrap_or_else(|| {
                            lis.kill();
                            tlk.kill();
                            panic!(
                                "[{} {}] ros2 still discovered no {kind} named `{node}` on \
                                 {topic} after polling for 20s ({}). The delivery assertion \
                                 above already passed, so the endpoints exist and are \
                                 QoS-matched — this is discovery, not wiring (issue \
                                 0705):\n{report}",
                                lang, workload, cell.note
                            )
                        });
                    // The workspaces declare `reliable + transient_local +
                    // keep_last(10)` per entity, in code.
                    // Every one of these differs from `QoSProfile::default()`
                    // except reliability, so together they cannot be satisfied
                    // by a defaulted entity.
                    for expect in [
                        "Reliability: RELIABLE",
                        "Durability: TRANSIENT_LOCAL",
                        "History (Depth): KEEP_LAST (10)",
                    ] {
                        if !block.contains(expect) {
                            lis.kill();
                            tlk.kill();
                            // The BLOCK alone cannot say which endpoint it is.
                            // `topic_endpoint_block` returns the FIRST block of
                            // this kind, so when the topic carries more than one
                            // endpoint the assertion may be reading someone
                            // else's — and this cell fails in-sweep while
                            // passing solo, which is exactly the shape a foreign
                            // endpoint would produce. Printing the block only
                            // (issue 0445's class) leaves that indistinguishable
                            // from the profile genuinely being dropped, so carry
                            // the whole report and the endpoint count.
                            let n = report.matches("Endpoint type: ").count();
                            panic!(
                                "[{} {}] the {kind} does not advertise its code-declared QoS \
                                 (`{expect}` missing) — either the per-entity profile was \
                                 dropped between the node and the wire, or this block belongs \
                                 to a different endpoint ({}).\n\
                                 asserted block (`{kind}` of node `{node}`, {n} endpoint(s) \
                                 on {topic}):\n{block}\nfull report:\n{report}",
                                lang, workload, cell.note
                            );
                        }
                    }
                }
            }

            lis.kill();
            tlk.kill();

            // Early pre-discovery samples may be missed, so assert ≥3
            // receives (the endpoints connect + deliver end-to-end).
            let n = nros_tests::count_pattern(&out, prefix);
            assert!(
                n >= 3,
                "[{} {}] expected ≥3 QoS-matched receives, got {n}.\n{out}",
                lang,
                workload
            );
        }

        Proof::LifecycleActive => {
            let mut node = spawn_spinning(&entry, "lifecycle-entry", &locator, 30000, Some(10));

            // Discover the managed node (the entry's executor node hosting
            // the 5 services) — robust to the executor node name.
            let nodes_out =
                // `--no-daemon`, like the `lifecycle get` below already had
                // (issue 0763). This polls at 200 ms for up to 20 s, so via the
                // daemon it is up to 100 queries against a singleton shared by
                // every concurrent test — and the setup it goes through used to
                // stop that daemon each time. The remedy was already spelled
                // one call down; this is the site that never got it.
                poll_ros2_until(
                    &locator,
                    "lifecycle nodes --no-daemon --spin-time 0.1",
                    "/",
                    Duration::from_secs(20),
                );
            let lifecycle_node = first_lifecycle_node(&nodes_out).unwrap_or_else(|| {
                node.kill();
                panic!(
                    "[{} {}] `ros2 lifecycle nodes` listed no managed node — the workspace \
                     entry's REP-2002 services are not on the wire ({}):\n{nodes_out}",
                    lang, workload, cell.note
                )
            });

            // Autostart should already have driven it to active — no manual
            // `ros2 lifecycle set` issued.
            let state = poll_ros2_until(
                &locator,
                &format!("lifecycle get --no-daemon --spin-time 0.1 {lifecycle_node}"),
                "active",
                Duration::from_secs(20),
            );
            node.kill();

            assert!(
                state.to_lowercase().contains("active"),
                "[{} {}] expected the autostart-managed node {lifecycle_node} to be \
                 `active` at boot ({}), got:\n{state}",
                lang,
                workload,
                cell.note
            );
        }

        Proof::SafetyCrcCount { spin_ms, wait_secs } => {
            // /safe_ok sink first, then the listener entry (its /chatter
            // safety subscription must be up before the talker publishes).
            let mut sub = nros_tests::fixtures::spawn_int32_sink(Some("/safe_ok"), &locator);
            let peer = peer.expect("safety cells carry a listener entry");
            let mut listener =
                spawn_spinning(&peer, "safety-listener", &locator, spin_ms, Some(10));
            std::thread::sleep(Duration::from_millis(1000));
            let mut tlk = spawn_spinning(&entry, "safety-talker", &locator, spin_ms, Some(10));

            // The talker publishes /chatter at 1 Hz with a CRC; each
            // CRC-validated receive republishes the count on /safe_ok.
            // Seeing 3 confirms the validate path holds.
            let prefix = nros_tests::output::INT32_LISTENER_LOG_PREFIX;
            let out = sub
                .wait_for_output_count(prefix, 3, Duration::from_secs(wait_secs))
                .unwrap_or_else(|_| {
                    tlk.kill();
                    listener.kill();
                    sub.kill();
                    panic!(
                        "[{} {}] /safe_ok never saw 3 CRC-validated publishes — the \
                         cross-process E2E safety path (talker → backend CRC → runtime \
                         validate → integrity-read → republish) failed ({})",
                        lang, workload, cell.note
                    )
                });
            tlk.kill();
            listener.kill();
            sub.kill();

            let n = nros_tests::count_pattern(&out, prefix);
            assert!(
                n >= 3,
                "[{} {}] expected ≥3 CRC-validated /safe_ok publishes, got {n}\n{out}",
                lang,
                workload
            );
        }

        Proof::RemapWireName => {
            // Both sinks first so their subscriptions are live before the
            // entry publishes: the REMAPPED topic must deliver; the
            // unremapped `~` expansion must stay silent.
            let mut remapped =
                nros_tests::fixtures::spawn_int32_sink(Some("/remapped_out"), &locator);
            let mut unremapped =
                nros_tests::fixtures::spawn_int32_sink(Some("/island/remap_talker/out"), &locator);
            let mut entry_proc = spawn_spinning(&entry, "remap-entry", &locator, 16000, Some(10));

            let prefix = nros_tests::output::INT32_LISTENER_LOG_PREFIX;
            let out = remapped
                .wait_for_output_count(prefix, 3, Duration::from_secs(22))
                .unwrap_or_else(|_| {
                    entry_proc.kill();
                    remapped.kill();
                    unremapped.kill();
                    panic!(
                        "[{} {}] /remapped_out never received 3 samples — the launch/model \
                         <remap> did not reach the wire (the publisher is on the wrong \
                         name) ({})",
                        lang, workload, cell.note
                    )
                });
            entry_proc.kill();
            remapped.kill();

            let n = nros_tests::count_pattern(&out, prefix);
            assert!(
                n >= 3,
                "[{} {}] expected ≥3 receives on the REMAPPED topic, got {n}.\n{out}",
                lang,
                workload
            );

            // Negative half: nothing may have leaked onto the unremapped
            // expansion. A short window suffices — the positive proof above
            // already guarantees ≥3 publish cycles happened.
            let leak = unremapped.wait_for_output_count(prefix, 1, Duration::from_secs(2));
            unremapped.kill();
            assert!(
                leak.is_err(),
                "[{} {}] the UNREMAPPED expansion /island/remap_talker/out received \
                 traffic — the remap did not take effect on the wire ({}):\n{}",
                lang,
                workload,
                cell.note,
                leak.unwrap_or_default()
            );
        }
    }
}
