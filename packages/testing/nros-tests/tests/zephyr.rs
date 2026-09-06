//! phase-295 W3.c — THE Zephyr native_sim example matrix consumer (RFC-0051).
//!
//! Parametrizes the hand-written per-cell zephyr families into cells of the
//! test matrix (`nros_tests::matrix`): every
//! `(ZephyrNativeSim, lang, rmw, workload, Example, Runtime)` row is ONE
//! `#[case]` of [`example_e2e`] — (rmw ∈ zenoh/cyclonedds/xrce) ×
//! (lang ∈ rust/c/cpp) × (workload ∈ pubsub/service/action), 27 cells — plus
//! the historical boot smokes as [`boot_smoke`] cells. The zephyr cyclone
//! runtime family previously split across
//! `tests/zephyr_cyclonedds_native_sim_e2e.rs` (pubsub + service e2e, rust
//! boot smokes; ex-`phase_118_collapse`) now lives HERE — one home per
//! family, that file is retired.
//!
//! Isolation, preserved 1:1 from the per-cell tests (until the phase-295 W4
//! allocator re-bake, the derivations below MIRROR the fixture bakes):
//! - **Ephemeral zenohd** (#166 / phase-286 W1): rust/cpp zenoh cells spin a
//!   per-test router; the image dials it via `-testargs --nros-locator`
//!   (`zephyr_component_main!` / `ZephyrBoard::run_components` read the
//!   override) — fully parallel.
//! - **Baked zenohd port**: C zenoh cells (+ the rust service cell) bake
//!   their router port at `west build` time —
//!   `PlatformConfig::zenohd_port_for(variant, lang)` (the allocator's
//!   7400-window with variant offset and lang stride since the phase-295
//!   W4 re-bake; the west lane computes the SAME formula).
//! - **Baked XRCE Agent port**: xrce cells start a MicroXRCEAgent on
//!   `xrce_agent_port_for(variant, lang)` (2400 + offsets), matching the
//!   `CONFIG_NROS_XRCE_AGENT_PORT` bake.
//! - **Baked Cyclone domain**: cyclonedds cells need no router/agent — SPDP
//!   multicast discovery on the per-(variant, lang) `CONFIG_NROS_DOMAIN_ID`
//!   bake (`alloc::domain_of` — domains 22–30; pairs share, sets differ →
//!   distinct RTPS ports).
//!
//! Skip semantics are identical to the per-cell tests: no zephyr workspace /
//! missing or STALE west image (`just zephyr build-fixtures`) / missing XRCE
//! agent → `nros_tests::skip!`. The `--seed` uniqueness (`ZephyrProcess`
//! injects one per spawn — identical native_sim entropy otherwise yields
//! identical GUIDs and discovery sees the peer as itself) and the
//! `--nros-locator` runtime-dial mechanics are unchanged.
//!
//! Bespoke tests kept below the matrix (NOT (rmw × lang × workload) cells):
//! the zephyr↔native cross-platform interop pairs (pubsub both directions +
//! bidirectional, service both directions, C++ pubsub both directions), the
//! workspace-Entry native_sim e2e (Workspace kind), and the availability
//! probe.
//!
//! Run with: `cargo nextest run -p nros-tests --test zephyr`
//! (slice a family: `-E 'binary(zephyr) and test(pubsub)'`).

use nros_tests::{
    count_pattern,
    fixtures::{
        XrceAgent, ZenohRouter, build_native_listener, build_native_service_client,
        build_native_service_server, build_native_talker, require_xrce_agent,
    },
    matrix::{Lang, Rmw, Workload},
    output, platform,
    zephyr::{
        ZephyrPlatform, ZephyrProcess, get_prebuilt_zephyr_example,
        get_prebuilt_zephyr_workspace_entry, require_zephyr,
    },
};
use rstest::rstest;
use std::{
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

// =============================================================================
// Shared markers + helpers
// =============================================================================

/// Zephyr boot banner (printed by every native_sim image before app code).
const ZEPHYR_BOOT_BANNER: &str = "Booting Zephyr";

/// M-F.23: the single-node `zephyr_component_main!` macro emits the
/// canonical `"Waiting for messages"` readiness marker for every node kind
/// (pub/sub/service/action); the C/C++ listeners print the same line.
///
/// phase-342 W7 — bound to the shared table rather than respelled. The value is
/// byte-identical to what was here, so this changes nothing at runtime; what it
/// changes is ownership. A file-local copy of a marker string is how a test and
/// the example it watches drift apart, which is issue 0481 in one sentence.
const NODE_READY_MARKER: &str = nros_tests::output::LISTENER_WAITING_BANNER;

/// Lax server-readiness prefix: matches BOTH the component-main
/// `"Waiting for messages"` marker (rust servers) and the canonical
/// `SERVICE_SERVER_READY_MARKER` (`"Waiting for service requests"`, C/C++
/// servers) — the historical per-cell service tests keyed on this prefix.
///
/// phase-342 W7 — this one is DELIBERATE, not the issue-0481 defect it
/// resembles. 0481 was about a literal that matched some binaries by accident;
/// this is a union that must match two legitimate spellings, because a zephyr
/// server cell's readiness line depends on whether the node is rust
/// (component-main) or C/C++ (the canonical server marker). One constant cannot
/// be both, so the prefix is the honest encoding of "either of these".
///
/// The real improvement is per-cell markers keyed on the cell's LANGUAGE — the
/// shape `output::ready_marker(role, lang)` already uses — which would remove
/// the union entirely. NOT done here: it changes which line each cell waits on,
/// and no zephyr tree is provisioned on this host to verify it against
/// (`third-party/zephyr` is absent; provisioning is a west init plus SDK). A
/// readiness change that cannot be run is exactly the change that fails
/// silently.
const SERVER_READY_LAX: &str = "Waiting";

fn count_zephyr_received(output: &str) -> usize {
    // All Zephyr listener fixtures (c/cpp/rust) print the canonical
    // listener sample line (Phase 198.2 normalized the rust fixture).
    output
        .lines()
        .filter(|line| line.contains(nros_tests::output::LISTENER_LOG_PREFIX))
        .count()
}

fn rmw_str(r: Rmw) -> &'static str {
    match r {
        Rmw::Zenoh => "zenoh",
        Rmw::Cyclonedds => "cyclonedds",
        Rmw::Xrce => "xrce",
        Rmw::Uorb => "uorb",
    }
}

/// Resolve a prebuilt Zephyr native_sim example image for
/// (lang, case, rmw), or skip. Missing OR stale (`is_binary_stale`) west
/// images skip with the `just zephyr build-fixtures` remedy — tests never
/// build fixtures in their bodies (Phase 179.I).
fn resolve_example(lang: Lang, case: &str, rmw: Rmw) -> PathBuf {
    let alias = match (lang, rmw) {
        (Lang::Rust, Rmw::Zenoh) => format!("zephyr-rs-{case}"),
        (Lang::Rust, Rmw::Xrce) => format!("zephyr-xrce-rs-{case}"),
        (Lang::Rust, Rmw::Cyclonedds) => format!("zephyr-dds-rs-{case}"),
        (Lang::C, Rmw::Zenoh) => format!("zephyr-c-{case}"),
        (Lang::C, Rmw::Xrce) => format!("zephyr-xrce-c-{case}"),
        (Lang::C, Rmw::Cyclonedds) => format!("zephyr-dds-c-{case}"),
        (Lang::Cpp, Rmw::Zenoh) => format!("zephyr-cpp-{case}"),
        (Lang::Cpp, Rmw::Xrce) => format!("zephyr-xrce-cpp-{case}"),
        (Lang::Cpp, Rmw::Cyclonedds) => format!("zephyr-dds-cpp-{case}"),
        (_, Rmw::Uorb) => unreachable!("uORB is PX4-SITL only; no zephyr example (issue 0341)"),
        (Lang::Mixed, _) => unreachable!("no mixed example cells"),
    };
    get_prebuilt_zephyr_example(&alias, ZephyrPlatform::NativeSim).unwrap_or_else(|e| {
        nros_tests::skip!(
            "zephyr/{}/{case} {} image not prebuilt or stale \
             (run `just zephyr build-fixtures`): {e:?}",
            lang.as_str(),
            rmw_str(rmw)
        )
    })
}

/// Poll `proc`'s accumulated output until it carries at least `min`
/// listener sample lines, or `timeout` elapses. Returns everything seen.
fn wait_for_received_count(proc: &ZephyrProcess, min: usize, timeout: Duration) -> String {
    let deadline = Instant::now() + timeout;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        let out = proc.wait_for_pattern(output::LISTENER_LOG_PREFIX, remaining);
        if count_pattern(&out, output::LISTENER_LOG_PREFIX) >= min || Instant::now() >= deadline {
            return out;
        }
        std::thread::sleep(Duration::from_millis(250));
    }
}

// =============================================================================
// Cell table types
// =============================================================================

/// How the cell's pair is isolated from its siblings. Every baked value is
/// the MIRROR of the fixture's compile-time bake until the phase-295 W4
/// allocator re-bake.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Iso {
    /// Per-test ephemeral zenohd; both images dial it at runtime via
    /// `-testargs --nros-locator` (#166 / phase-286 W1).
    EphemeralZenohd,
    /// zenohd on the image's compile-time-baked port —
    /// `ZEPHYR.zenohd_port_for(variant, lang)` (the allocator's number).
    BakedZenohd,
    /// MicroXRCEAgent on the image's baked UDP port —
    /// `ZEPHYR.xrce_agent_port_for(variant, lang)` (the allocator's number).
    BakedXrceAgent,
    /// Nothing to spawn: Cyclone SPDP multicast on the baked
    /// per-(variant, lang) `CONFIG_NROS_DOMAIN_ID` (allocator domains 22–30).
    BakedCycloneDomain,
}

/// Server-side corroboration for action cells (preserved per-cell from the
/// pre-consolidation asserts).
#[derive(Copy, Clone, Debug)]
enum ServerCheck {
    /// The server must have received/executed the goal
    /// (`ACTION_GOAL_REQUEST_PREFIX` or `ACTION_EXECUTING_MARKER`).
    GoalReceived,
    /// The server must have completed the goal
    /// (`ACTION_GOAL_SUCCEEDED_MARKER`).
    GoalSucceeded,
}

/// One (rmw × lang × workload) Zephyr native_sim matrix cell.
struct Cell {
    rmw: Rmw,
    lang: Lang,
    workload: Workload,
    iso: Iso,
    /// Server/listener readiness marker waited on before the peer starts.
    ready_marker: &'static str,
    ready_secs: u64,
    /// Whether a missed readiness marker panics (some historical cells
    /// tolerated it and let the client-side wait carry the failure).
    ready_fatal: bool,
    /// Extra settle after readiness (xrce action: the Agent needs time to
    /// propagate the server's CREATE_REPLIER ack under load).
    post_ready_ms: u64,
    /// Client/listener success window.
    window_secs: u64,
    /// Pubsub only: minimum delivered sample lines.
    min_received: usize,
    /// Action only: the client must also log feedback frames.
    require_feedback: bool,
    /// Action only: server-side corroboration.
    server_check: Option<ServerCheck>,
    /// Provenance / nuance — folded into failure messages so a red cell
    /// still names the seam it pins.
    note: &'static str,
}

impl Cell {
    fn id(&self) -> String {
        format!(
            "{}/{}/{:?}",
            rmw_str(self.rmw),
            self.lang.as_str(),
            self.workload
        )
    }
}

/// Isolation guards — held for the whole cell (Drop tears them down).
#[allow(dead_code)] // held for Drop, never read
enum IsoGuard {
    Router(ZenohRouter),
    Agent(XrceAgent),
    None,
}

/// Start the cell's isolation resource. Returns the guard plus the runtime
/// locator to dial (ephemeral cells only — baked cells' images carry their
/// endpoint at compile time).
fn setup_isolation(cell: &Cell) -> (IsoGuard, Option<String>) {
    let variant = cell
        .workload
        .as_test_variant()
        .expect("example cells use the classic pubsub/service/action variants");
    let lang = cell.lang.as_test_lang();
    match cell.iso {
        Iso::EphemeralZenohd => {
            let router = nros_tests::fixtures::or_skip(ZenohRouter::start_unique());
            let locator = router.locator();
            eprintln!("[{}] ephemeral zenohd on {locator}", cell.id());
            (IsoGuard::Router(router), Some(locator))
        }
        Iso::BakedZenohd => {
            // Baked router port — the allocator's number, which is also
            // the fixture's locator bake.
            let port = platform::ZEPHYR.zenohd_port_for(variant, lang);
            let router = nros_tests::fixtures::or_skip(ZenohRouter::start(port));
            eprintln!("[{}] zenohd on baked port {port}", cell.id());
            (IsoGuard::Router(router), None)
        }
        Iso::BakedXrceAgent => {
            // Baked Agent port (matches the `CONFIG_NROS_XRCE_AGENT_PORT`
            // west bake — `just/zephyr.just` — until the W4 re-bake).
            let port = platform::ZEPHYR.xrce_agent_port_for(variant, lang);
            let agent = XrceAgent::start(port).expect("Failed to start XRCE Agent");
            eprintln!("[{}] XRCE Agent on baked port {port}", cell.id());
            (IsoGuard::Agent(agent), None)
        }
        Iso::BakedCycloneDomain => {
            // Cyclone SPDP multicast on the baked per-(variant, lang)
            // allocator domain (22–30) — nothing to spawn.
            (IsoGuard::None, None)
        }
    }
}

/// Boot a native_sim image, dialing `locator` when the cell is ephemeral.
fn start_image(bin: &Path, locator: &Option<String>, what: &str) -> ZephyrProcess {
    match locator {
        Some(loc) => ZephyrProcess::start_with_locator(bin, ZephyrPlatform::NativeSim, loc),
        None => ZephyrProcess::start(bin, ZephyrPlatform::NativeSim),
    }
    .unwrap_or_else(|e| panic!("Failed to start {what}: {e:?}"))
}

// =============================================================================
// The parametrized matrix consumer — 27 (rmw × lang × workload) cells
// =============================================================================

/// One Zephyr native_sim example cell: boot the on-target pair against the
/// cell's isolation resource and prove the workload contract. Case names
/// carry `<rmw>_<lang>_<workload>_e2e` so the `.config/nextest.toml` groups
/// can slice by family (e.g. `test(xrce)`, `test(zenoh_cpp_service_e2e)`).
#[rstest]
// ── zenoh ────────────────────────────────────────────────────────────────
#[case::zenoh_rust_pubsub_e2e(Cell {
    rmw: Rmw::Zenoh, lang: Lang::Rust, workload: Workload::Pubsub,
    iso: Iso::EphemeralZenohd,
    ready_marker: NODE_READY_MARKER, ready_secs: 30, ready_fatal: true,
    post_ready_ms: 0, window_secs: 40, min_received: 1,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_talker_to_listener_e2e — zenoh-pico session over NSOS; \
           readiness poll replaced the Phase 89.12 fixed-sleep flake",
})]
#[case::zenoh_c_pubsub_e2e(Cell {
    rmw: Rmw::Zenoh, lang: Lang::C, workload: Workload::Pubsub,
    iso: Iso::BakedZenohd,
    ready_marker: NODE_READY_MARKER, ready_secs: 30, ready_fatal: true,
    post_ready_ms: 0, window_secs: 30, min_received: 1,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_c_talker_to_listener_e2e (Phase 183.1 closed the zephyr/c hole)",
})]
#[case::zenoh_cpp_pubsub_e2e(Cell {
    rmw: Rmw::Zenoh, lang: Lang::Cpp, workload: Workload::Pubsub,
    iso: Iso::EphemeralZenohd,
    ready_marker: NODE_READY_MARKER, ready_secs: 30, ready_fatal: true,
    post_ready_ms: 0, window_secs: 40, min_received: 3,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_cpp_talker_to_listener_e2e — ≥3 deliveries (historical margin)",
})]
#[case::zenoh_rust_service_e2e(Cell {
    rmw: Rmw::Zenoh, lang: Lang::Rust, workload: Workload::Service,
    iso: Iso::BakedZenohd,
    ready_marker: SERVER_READY_LAX, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 30, min_received: 0,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_rust_service_e2e (Phase 183.3 — rust had pubsub+action but \
           no service); baked port 7466",
})]
#[case::zenoh_c_service_e2e(Cell {
    rmw: Rmw::Zenoh, lang: Lang::C, workload: Workload::Service,
    iso: Iso::BakedZenohd,
    ready_marker: SERVER_READY_LAX, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 30, min_received: 0,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_c_service_server_to_client_e2e (Phase 183.1)",
})]
#[case::zenoh_cpp_service_e2e(Cell {
    rmw: Rmw::Zenoh, lang: Lang::Cpp, workload: Workload::Service,
    iso: Iso::EphemeralZenohd,
    ready_marker: SERVER_READY_LAX, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 30, min_received: 0,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_cpp_service_server_to_client_e2e — one-shot AddTwoIntsClient \
           (a single reply is the whole contract)",
})]
#[case::zenoh_rust_action_e2e(Cell {
    rmw: Rmw::Zenoh, lang: Lang::Rust, workload: Workload::Action,
    iso: Iso::EphemeralZenohd,
    ready_marker: NODE_READY_MARKER, ready_secs: 60, ready_fatal: true,
    post_ready_ms: 0, window_secs: 150, min_received: 0,
    require_feedback: true, server_check: Some(ServerCheck::GoalReceived),
    note: "ex test_zephyr_action_e2e — 3 queryable declarations serialize at ~10 s each \
           on zenoh-pico (Phase 160.C), hence the 60 s readiness + 150 s client budget",
})]
#[case::zenoh_c_action_e2e(Cell {
    rmw: Rmw::Zenoh, lang: Lang::C, workload: Workload::Action,
    iso: Iso::BakedZenohd,
    ready_marker: output::ACTION_SERVER_READY_MARKER, ready_secs: 30, ready_fatal: true,
    post_ready_ms: 0, window_secs: 60, min_received: 0,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_c_action_server_to_client_e2e — un-ignored 2026-07-12 after the \
           stale readiness-marker false diagnosis (the #174 class)",
})]
#[case::zenoh_cpp_action_e2e(Cell {
    rmw: Rmw::Zenoh, lang: Lang::Cpp, workload: Workload::Action,
    iso: Iso::EphemeralZenohd,
    ready_marker: output::ACTION_SERVER_READY_MARKER, ready_secs: 60, ready_fatal: true,
    post_ready_ms: 0, window_secs: 90, min_received: 0,
    require_feedback: false, server_check: Some(ServerCheck::GoalSucceeded),
    note: "ex test_zephyr_cpp_action_server_to_client_e2e — client needs ~25 s to reach \
           send_goal (3 service-client declarations, Phase 160.C)",
})]
// ── cyclonedds (SPDP multicast; baked allocator domains 22–30) ───────────
#[case::cyclonedds_rust_pubsub_e2e(Cell {
    rmw: Rmw::Cyclonedds, lang: Lang::Rust, workload: Workload::Pubsub,
    iso: Iso::BakedCycloneDomain,
    ready_marker: NODE_READY_MARKER, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 20, min_received: 1,
    require_feedback: false, server_check: None,
    note: "ex zephyr_cyclonedds_native_sim_e2e::test_zephyr_rust_cyclonedds_pubsub_e2e — \
           NSOS getifaddrs + IP_ADD_MEMBERSHIP forwarding + per-spawn --seed (identical \
           GUID prefixes otherwise stall SPDP)",
})]
#[case::cyclonedds_c_pubsub_e2e(Cell {
    rmw: Rmw::Cyclonedds, lang: Lang::C, workload: Workload::Pubsub,
    iso: Iso::BakedCycloneDomain,
    ready_marker: NODE_READY_MARKER, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 20, min_received: 1,
    require_feedback: false, server_check: None,
    note: "ex zephyr_cyclonedds_native_sim_e2e::test_zephyr_c_cyclonedds_pubsub_e2e — the \
           C CMake generates the Cyclone dds_topic_descriptor_t the nros C codegen omits",
})]
#[case::cyclonedds_cpp_pubsub_e2e(Cell {
    rmw: Rmw::Cyclonedds, lang: Lang::Cpp, workload: Workload::Pubsub,
    iso: Iso::BakedCycloneDomain,
    ready_marker: NODE_READY_MARKER, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 20, min_received: 1,
    require_feedback: false, server_check: None,
    note: "ex zephyr_cyclonedds_native_sim_e2e::test_zephyr_cpp_cyclonedds_pubsub_e2e — \
           16 MiB malloc arena + NSOS offload overlay parity with the Rust image",
})]
#[case::cyclonedds_rust_service_e2e(Cell {
    rmw: Rmw::Cyclonedds, lang: Lang::Rust, workload: Workload::Service,
    iso: Iso::BakedCycloneDomain,
    ready_marker: SERVER_READY_LAX, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 20, min_received: 0,
    require_feedback: false, server_check: None,
    note: "ex zephyr_cyclonedds_native_sim_e2e::test_zephyr_rust_cyclonedds_service_e2e — \
           pins the backend service_type_name trailing-underscore strip",
})]
#[case::cyclonedds_c_service_e2e(Cell {
    rmw: Rmw::Cyclonedds, lang: Lang::C, workload: Workload::Service,
    iso: Iso::BakedCycloneDomain,
    ready_marker: SERVER_READY_LAX, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 20, min_received: 0,
    require_feedback: false, server_check: None,
    note: "ex zephyr_cyclonedds_native_sim_e2e::test_zephyr_c_cyclonedds_service_e2e — \
           Phase 171.0.a volatile-reader-match fix (client wrote before the match)",
})]
#[case::cyclonedds_cpp_service_e2e(Cell {
    rmw: Rmw::Cyclonedds, lang: Lang::Cpp, workload: Workload::Service,
    iso: Iso::BakedCycloneDomain,
    ready_marker: SERVER_READY_LAX, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 20, min_received: 0,
    require_feedback: false, server_check: None,
    note: "ex zephyr_cyclonedds_native_sim_e2e::test_zephyr_cpp_cyclonedds_service_e2e",
})]
#[case::cyclonedds_rust_action_e2e(Cell {
    rmw: Rmw::Cyclonedds, lang: Lang::Rust, workload: Workload::Action,
    iso: Iso::BakedCycloneDomain,
    ready_marker: NODE_READY_MARKER, ready_secs: 30, ready_fatal: true,
    post_ready_ms: 0, window_secs: 90, min_received: 0,
    require_feedback: false, server_check: Some(ServerCheck::GoalReceived),
    note: "ex test_zephyr_dds_rs_action_e2e",
})]
#[case::cyclonedds_c_action_e2e(Cell {
    rmw: Rmw::Cyclonedds, lang: Lang::C, workload: Workload::Action,
    iso: Iso::BakedCycloneDomain,
    ready_marker: output::ACTION_SERVER_READY_MARKER, ready_secs: 60, ready_fatal: true,
    post_ready_ms: 0, window_secs: 60, min_received: 0,
    require_feedback: false, server_check: Some(ServerCheck::GoalSucceeded),
    note: "ex test_zephyr_dds_c_action_e2e — Cyclone-on-Zephyr declares action entities \
           at ~10 s each, hence the 60 s readiness window",
})]
#[case::cyclonedds_cpp_action_e2e(Cell {
    rmw: Rmw::Cyclonedds, lang: Lang::Cpp, workload: Workload::Action,
    iso: Iso::BakedCycloneDomain,
    ready_marker: output::ACTION_SERVER_READY_MARKER, ready_secs: 60, ready_fatal: true,
    post_ready_ms: 0, window_secs: 60, min_received: 0,
    require_feedback: false, server_check: Some(ServerCheck::GoalSucceeded),
    note: "ex test_zephyr_dds_cpp_action_e2e",
})]
// ── xrce (MicroXRCEAgent on the baked per-(variant, lang) port) ──────────
#[case::xrce_rust_pubsub_e2e(Cell {
    rmw: Rmw::Xrce, lang: Lang::Rust, workload: Workload::Pubsub,
    iso: Iso::BakedXrceAgent,
    ready_marker: NODE_READY_MARKER, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 30, min_received: 1,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_xrce_rust_talker_listener",
})]
#[case::xrce_c_pubsub_e2e(Cell {
    rmw: Rmw::Xrce, lang: Lang::C, workload: Workload::Pubsub,
    iso: Iso::BakedXrceAgent,
    ready_marker: NODE_READY_MARKER, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 30, min_received: 1,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_xrce_c_talker_listener — pins the C talker's immediate \
           XRCE output-stream flush after publish",
})]
#[case::xrce_cpp_pubsub_e2e(Cell {
    rmw: Rmw::Xrce, lang: Lang::Cpp, workload: Workload::Pubsub,
    iso: Iso::BakedXrceAgent,
    ready_marker: NODE_READY_MARKER, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 30, min_received: 1,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_xrce_cpp_talker_listener (Phase 96.1) — needs distinct XRCE \
           session_names per cpp process (shared-key hash collided as one client)",
})]
#[case::xrce_rust_service_e2e(Cell {
    rmw: Rmw::Xrce, lang: Lang::Rust, workload: Workload::Service,
    iso: Iso::BakedXrceAgent,
    ready_marker: SERVER_READY_LAX, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 30, min_received: 0,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_xrce_rust_service_e2e (Phase 95.A)",
})]
#[case::xrce_c_service_e2e(Cell {
    rmw: Rmw::Xrce, lang: Lang::C, workload: Workload::Service,
    iso: Iso::BakedXrceAgent,
    ready_marker: SERVER_READY_LAX, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 30, min_received: 0,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_xrce_c_service_e2e (Phase 183.1)",
})]
#[case::xrce_cpp_service_e2e(Cell {
    rmw: Rmw::Xrce, lang: Lang::Cpp, workload: Workload::Service,
    iso: Iso::BakedXrceAgent,
    ready_marker: SERVER_READY_LAX, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 30, min_received: 0,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_xrce_cpp_service_e2e (Phase 96.1)",
})]
#[case::xrce_rust_action_e2e(Cell {
    rmw: Rmw::Xrce, lang: Lang::Rust, workload: Workload::Action,
    iso: Iso::BakedXrceAgent,
    ready_marker: output::ACTION_SERVER_READY_MARKER, ready_secs: 30, ready_fatal: true,
    post_ready_ms: 1500, window_secs: 60, min_received: 0,
    require_feedback: true, server_check: None,
    note: "ex test_zephyr_xrce_rust_action_e2e — the 1500 ms settle lets the Agent \
           propagate the server's CREATE_REPLIER ack under full-suite load",
})]
#[case::xrce_c_action_e2e(Cell {
    rmw: Rmw::Xrce, lang: Lang::C, workload: Workload::Action,
    iso: Iso::BakedXrceAgent,
    ready_marker: output::ACTION_SERVER_READY_MARKER, ready_secs: 30, ready_fatal: true,
    post_ready_ms: 0, window_secs: 60, min_received: 0,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_xrce_c_action_e2e (Phase 183.1)",
})]
#[case::xrce_cpp_action_e2e(Cell {
    rmw: Rmw::Xrce, lang: Lang::Cpp, workload: Workload::Action,
    iso: Iso::BakedXrceAgent,
    ready_marker: output::ACTION_SERVER_READY_MARKER, ready_secs: 30, ready_fatal: false,
    post_ready_ms: 0, window_secs: 60, min_received: 0,
    require_feedback: false, server_check: None,
    note: "ex test_zephyr_xrce_cpp_action_e2e — this Fibonacci server completes with a \
           result and streams NO feedback (result-only gate; the #164 marker class)",
})]
fn example_e2e(#[case] cell: Cell) {
    if !require_zephyr() {
        nros_tests::skip!("Zephyr not available");
    }
    if matches!(cell.iso, Iso::BakedXrceAgent) && !require_xrce_agent() {
        nros_tests::skip!("XRCE agent not available");
    }

    let (first_role, second_role) = match cell.workload {
        Workload::Pubsub => ("listener", "talker"),
        Workload::Service => ("service-server", "service-client"),
        Workload::Action => ("action-server", "action-client"),
        other => unreachable!("no zephyr example cell for {other:?}"),
    };
    // Resolve BOTH images before spawning anything (skip fast + cleanly).
    let first_bin = resolve_example(cell.lang, first_role, cell.rmw);
    let second_bin = resolve_example(cell.lang, second_role, cell.rmw);

    let (_iso_guard, locator) = setup_isolation(&cell);

    // Subscriber/server first, so its declarations are live before the
    // talker/client starts (Phase 89.12 flake class).
    let first = start_image(&first_bin, &locator, first_role);
    let ready_out = first.wait_for_pattern(cell.ready_marker, Duration::from_secs(cell.ready_secs));
    if cell.ready_fatal && !ready_out.contains(cell.ready_marker) {
        // issue 0557 / phase-358 W5 — lead with the GUEST's error when it has
        // one. "didn't reach readiness within 60 s" is true and is the wrong
        // headline for a boot that died immediately: the marker was never
        // going to arrive, and the reader starts investigating latency instead
        // of the `run_components failed rc=-100` four lines up. Same shape as
        // issue 0445 one layer out — a self-explaining verdict standing in
        // front of the real result.
        let verdict = match nros_tests::output::first_guest_failure(&ready_out) {
            Some(line) => format!(
                "FAILED AT BOOT: {line}\n  (readiness marker `{}` never arrived; \
                 the {} s wait observed the failure, it did not cause it)",
                cell.ready_marker, cell.ready_secs
            ),
            None => format!(
                "didn't reach readiness (`{}`) within {} s",
                cell.ready_marker, cell.ready_secs
            ),
        };
        panic!(
            "[{}] {first_role} {verdict} — {}.\nOutput:\n{}",
            cell.id(),
            cell.note,
            ready_out
        );
    }
    let mut first = first;
    if cell.post_ready_ms > 0 {
        std::thread::sleep(Duration::from_millis(cell.post_ready_ms));
    }

    let mut second = start_image(&second_bin, &locator, second_role);
    let window = Duration::from_secs(cell.window_secs);

    match cell.workload {
        Workload::Pubsub => {
            let listener_out = wait_for_received_count(&first, cell.min_received, window);
            second.kill();
            first.kill();
            eprintln!("[{}] listener output:\n{listener_out}", cell.id());
            let n = count_zephyr_received(&listener_out);
            assert!(
                n >= cell.min_received,
                "[{}] listener received {n} sample(s), expected ≥{} — {}.\nOutput:\n{}",
                cell.id(),
                cell.min_received,
                cell.note,
                listener_out
            );
        }
        Workload::Service => {
            let client_out =
                second.wait_for_pattern(nros_tests::output::SERVICE_RESULT_PREFIX, window);
            let server_out = first
                .wait_for_output(Duration::from_secs(3))
                .unwrap_or_default();
            second.kill();
            first.kill();
            eprintln!("[{}] service client output:\n{client_out}", cell.id());
            eprintln!("[{}] service server output:\n{server_out}", cell.id());
            assert!(
                client_out.contains(nros_tests::output::SERVICE_RESULT_PREFIX),
                "[{}] service client got no reply — {}.\nClient:\n{}\nServer:\n{}",
                cell.id(),
                cell.note,
                client_out,
                server_out
            );
        }
        Workload::Action => {
            let client_out =
                second.wait_for_pattern(nros_tests::output::ACTION_RESULT_PREFIX, window);
            let server_out = first
                .wait_for_output(Duration::from_secs(5))
                .unwrap_or_default();
            second.kill();
            first.kill();
            eprintln!("[{}] action client output:\n{client_out}", cell.id());
            eprintln!("[{}] action server output:\n{server_out}", cell.id());

            assert!(
                client_out.contains(nros_tests::output::ACTION_RESULT_PREFIX),
                "[{}] action client never received the result — {}.\nClient:\n{}\nServer:\n{}",
                cell.id(),
                cell.note,
                client_out,
                server_out
            );
            if cell.require_feedback {
                let feedback =
                    count_pattern(&client_out, nros_tests::output::ACTION_FEEDBACK_PREFIX);
                assert!(
                    feedback > 0,
                    "[{}] action client completed but streamed no feedback — {}.\nClient:\n{}",
                    cell.id(),
                    cell.note,
                    client_out
                );
            }
            match cell.server_check {
                Some(ServerCheck::GoalReceived) => {
                    assert!(
                        server_out.contains(nros_tests::output::ACTION_GOAL_REQUEST_PREFIX)
                            || server_out.contains(nros_tests::output::ACTION_EXECUTING_MARKER),
                        "[{}] action server never logged the goal — {}.\nServer:\n{}",
                        cell.id(),
                        cell.note,
                        server_out
                    );
                }
                Some(ServerCheck::GoalSucceeded) => {
                    assert!(
                        server_out.contains(nros_tests::output::ACTION_GOAL_SUCCEEDED_MARKER),
                        "[{}] action server never completed the goal — {}.\nServer:\n{}",
                        cell.id(),
                        cell.note,
                        server_out
                    );
                }
                None => {}
            }
        }
        other => unreachable!("no zephyr example cell for {other:?}"),
    }
}

/// Tripwire: the case list above must track the matrix (RFC-0051 W1 SSoT).
/// A new `(ZephyrNativeSim, Example, Runtime)` row means a new `#[case]` in
/// [`example_e2e`]; a retired row means removing one. Update BOTH.
#[test]
fn example_e2e_case_count_tracks_matrix() {
    use nros_tests::matrix::{Kind, PlatformId, runtime_cells};
    let n = runtime_cells()
        .filter(|c| {
            matches!(c.platform, PlatformId::ZephyrNativeSim) && matches!(c.kind, Kind::Example)
        })
        .count();
    assert_eq!(
        n, 27,
        "the matrix declares {n} (ZephyrNativeSim, Example, Runtime) cells but example_e2e \
         carries 27 #[case]s — add/remove the matching case AND update this tripwire"
    );
}

// =============================================================================
// Boot smokes — image boots + initializes on native_sim, no peer/no router
// =============================================================================

/// What the smoke proves, preserved from the pre-consolidation tests.
#[derive(Copy, Clone, Debug)]
enum SmokeProof {
    /// The image boots and prints the Zephyr banner / nros init output.
    /// Connection failures are EXPECTED (no router/agent is started).
    Banner,
    /// The image boots AND its talker publishes (cyclone participant init —
    /// Phase 11W.10 asserts a real publish line, not just the banner).
    TalkerPublishes,
    /// The image boots AND its subscription reaches the wait state.
    ListenerReady,
}

struct Smoke {
    rmw: Rmw,
    lang: Lang,
    case: &'static str,
    proof: SmokeProof,
}

/// One boot-smoke cell: start the image with NO isolation resource and
/// assert it initializes per the cell's [`SmokeProof`]. Case names end in
/// `_boots` so the nextest e2e filters never capture them.
#[rstest]
// zenoh rust (the historical `test_zephyr_*_smoke` six-pack).
#[case::zenoh_rust_talker_boots(Smoke { rmw: Rmw::Zenoh, lang: Lang::Rust, case: "talker", proof: SmokeProof::Banner })]
#[case::zenoh_rust_listener_boots(Smoke { rmw: Rmw::Zenoh, lang: Lang::Rust, case: "listener", proof: SmokeProof::Banner })]
#[case::zenoh_rust_service_server_boots(Smoke { rmw: Rmw::Zenoh, lang: Lang::Rust, case: "service-server", proof: SmokeProof::Banner })]
#[case::zenoh_rust_service_client_boots(Smoke { rmw: Rmw::Zenoh, lang: Lang::Rust, case: "service-client", proof: SmokeProof::Banner })]
#[case::zenoh_rust_action_server_boots(Smoke { rmw: Rmw::Zenoh, lang: Lang::Rust, case: "action-server", proof: SmokeProof::Banner })]
#[case::zenoh_rust_action_client_boots(Smoke { rmw: Rmw::Zenoh, lang: Lang::Rust, case: "action-client", proof: SmokeProof::Banner })]
// xrce cpp (Phase 95.C — the examples block in nros::init without an agent,
// so the banner proves the binary linked + booted clean).
#[case::xrce_cpp_talker_boots(Smoke { rmw: Rmw::Xrce, lang: Lang::Cpp, case: "talker", proof: SmokeProof::Banner })]
#[case::xrce_cpp_listener_boots(Smoke { rmw: Rmw::Xrce, lang: Lang::Cpp, case: "listener", proof: SmokeProof::Banner })]
// cyclonedds cpp + c (Phase 95.D/95.E).
#[case::cyclonedds_cpp_talker_boots(Smoke { rmw: Rmw::Cyclonedds, lang: Lang::Cpp, case: "talker", proof: SmokeProof::Banner })]
#[case::cyclonedds_cpp_listener_boots(Smoke { rmw: Rmw::Cyclonedds, lang: Lang::Cpp, case: "listener", proof: SmokeProof::Banner })]
#[case::cyclonedds_cpp_service_server_boots(Smoke { rmw: Rmw::Cyclonedds, lang: Lang::Cpp, case: "service-server", proof: SmokeProof::Banner })]
#[case::cyclonedds_cpp_service_client_boots(Smoke { rmw: Rmw::Cyclonedds, lang: Lang::Cpp, case: "service-client", proof: SmokeProof::Banner })]
#[case::cyclonedds_c_talker_boots(Smoke { rmw: Rmw::Cyclonedds, lang: Lang::C, case: "talker", proof: SmokeProof::Banner })]
#[case::cyclonedds_c_listener_boots(Smoke { rmw: Rmw::Cyclonedds, lang: Lang::C, case: "listener", proof: SmokeProof::Banner })]
#[case::cyclonedds_c_service_server_boots(Smoke { rmw: Rmw::Cyclonedds, lang: Lang::C, case: "service-server", proof: SmokeProof::Banner })]
#[case::cyclonedds_c_service_client_boots(Smoke { rmw: Rmw::Cyclonedds, lang: Lang::C, case: "service-client", proof: SmokeProof::Banner })]
// cyclonedds rust (ex zephyr_cyclonedds_native_sim_e2e boot smokes —
// Phase 11W.9/.10: participant init proven by a real publish / wait state).
#[case::cyclonedds_rust_talker_boots(Smoke { rmw: Rmw::Cyclonedds, lang: Lang::Rust, case: "talker", proof: SmokeProof::TalkerPublishes })]
#[case::cyclonedds_rust_listener_boots(Smoke { rmw: Rmw::Cyclonedds, lang: Lang::Rust, case: "listener", proof: SmokeProof::ListenerReady })]
fn boot_smoke(#[case] smoke: Smoke) {
    if !require_zephyr() {
        nros_tests::skip!("Zephyr not available");
    }
    let bin = resolve_example(smoke.lang, smoke.case, smoke.rmw);
    let id = format!(
        "{}/{}/{}",
        rmw_str(smoke.rmw),
        smoke.lang.as_str(),
        smoke.case
    );
    let mut p = ZephyrProcess::start(&bin, ZephyrPlatform::NativeSim)
        .unwrap_or_else(|e| panic!("Failed to start {id}: {e:?}"));

    match smoke.proof {
        SmokeProof::Banner => {
            let out = p.wait_for_pattern(ZEPHYR_BOOT_BANNER, Duration::from_secs(10));
            p.kill();
            eprintln!("[{id}] output:\n{out}");
            assert!(
                out.contains(ZEPHYR_BOOT_BANNER) || out.contains("nros"),
                "[{id}] failed to boot — no initialization output:\n{out}"
            );
        }
        SmokeProof::TalkerPublishes => {
            // 1 Hz timer — first publish lands ~1.1 s in; allow margin.
            let out = p.wait_for_pattern(output::TALKER_LOG_PREFIX, Duration::from_secs(10));
            p.kill();
            eprintln!("[{id}] output:\n{out}");
            assert!(
                out.contains(ZEPHYR_BOOT_BANNER) || out.contains("nros"),
                "[{id}] failed to print init banner:\n{out}"
            );
            output::assert_talker(&out, 1);
        }
        SmokeProof::ListenerReady => {
            let out = p.wait_for_pattern(NODE_READY_MARKER, Duration::from_secs(10));
            p.kill();
            eprintln!("[{id}] output:\n{out}");
            assert!(
                out.contains(NODE_READY_MARKER),
                "[{id}] did not reach the subscription wait state:\n{out}"
            );
        }
    }
}

// =============================================================================
// Availability probe (bespoke — informational)
// =============================================================================

// `test_zephyr_availability_checks` removed. Its own last line said what was
// wrong with it — "These are informational - don't fail if Zephyr isn't set
// up" — which is a description of something that is not a test: it printed the
// workspace path and one boolean and asserted nothing, so it reported PASS with
// Zephyr entirely absent. `zephyr_workspace_path()` / `is_zephyr_available()`
// stay load-bearing as the `skip!` guards on the real Zephyr tests, where a
// `false` stops the run. Forbidden repo-wide by `check-no-vacuous-tests`.

// =============================================================================
// Bespoke: Zephyr ↔ native cross-platform pubsub (rust) — NOT
// (rmw × lang × workload) cells: each pairs a Zephyr image with a NATIVE
// process (Interop-shaped, single-platform matrix cells can't express it).
// =============================================================================

/// Test: Zephyr talker → Native listener communication
///
/// Tests that a Zephyr talker can send messages to a native Rust listener.
#[test]
fn test_zephyr_to_native_e2e() {
    if !require_zephyr() {
        nros_tests::skip!("Zephyr not available");
    }

    // #166 / phase-286 W1 — per-test ephemeral zenohd + locator override (both
    // the native listener via NROS_LOCATOR and the Zephyr talker via
    // `-testargs --nros-locator` dial THIS router), so this test no longer needs
    // the fixed per-(variant,lang) port and can run parallel with its siblings.
    eprintln!("Starting per-test zenohd router (ephemeral, #166)...");
    let router = nros_tests::fixtures::or_skip(ZenohRouter::start_unique());
    let locator = router.locator();
    eprintln!("zenohd locator: {locator}");

    // Build native listener
    let listener_path = build_native_listener().expect("Failed to build native-rs-listener");

    // Get Zephyr talker
    let zephyr_binary = resolve_example(Lang::Rust, "talker", Rmw::Zenoh);
    eprintln!("Zephyr talker binary: {}", zephyr_binary.display());

    // Start native listener connecting to zenohd
    use nros_tests::process::ManagedProcess;
    use std::process::Command;

    let mut listener_cmd = Command::new(listener_path);
    // Both native and Zephyr NSOS processes connect to zenohd on localhost
    listener_cmd
        .env("NROS_LOCATOR", &locator)
        .env("RUST_LOG", "info");
    let mut listener = ManagedProcess::spawn_command(listener_cmd, "native-rs-listener")
        .expect("Failed to start listener");

    listener
        .wait_for_output_pattern(
            nros_tests::output::LISTENER_READY_MARKER,
            Duration::from_secs(5),
        )
        .expect("native listener did not become ready");

    // Start Zephyr talker
    eprintln!("Starting Zephyr talker...");
    let mut zephyr =
        ZephyrProcess::start_with_locator(&zephyr_binary, ZephyrPlatform::NativeSim, &locator)
            .expect("Failed to start Zephyr talker");

    // Wait for communication
    eprintln!("Waiting for Zephyr → Native communication...");

    // Wait for the listener's first SAMPLE line — `collect_until` reads stderr
    // too (where env_logger writes), returns as soon as the marker lands, and
    // kills nothing.
    //
    // issue 1026 — this was `wait_for_all_output(40s)`, which has no stop
    // condition and KILLS at the deadline, so it always ran the full 40 s and
    // the window was the listener's lifetime rather than a bound on the wait.
    // 40 s is still the right cap: on a slow native_sim host the Zephyr
    // talker's zenoh-pico session setup + first publish lands ~20 s after boot
    // (issue #17).
    //
    // Bound stated: FIRST delivery only. This cell cannot see a session that
    // lapses after the first sample — for that, count samples across a lease
    // interval, which is what issue 1013 put on the RTOS pubsub cell.
    let listener_output = listener.collect_until(
        nros_tests::output::LISTENER_LOG_PREFIX,
        Duration::from_secs(40),
    );

    // Get Zephyr output for debugging
    let zephyr_output = zephyr
        .wait_for_output(Duration::from_secs(1))
        .unwrap_or_default();

    // Kill processes
    zephyr.kill();
    drop(listener);
    drop(router);

    eprintln!("\n=== Zephyr output ===\n{}", zephyr_output);
    eprintln!("\n=== Native listener output ===\n{}", listener_output);

    // Strict delivery check: the native listener must log at least one
    // real sample line (not setup text like "Waiting for Int32 ...").
    let received_count = count_pattern(&listener_output, nros_tests::output::LISTENER_LOG_PREFIX);
    let zephyr_transport_err = zephyr_output.contains("Transport(ConnectionFailed)")
        || zephyr_output.contains("z_publisher_put failed")
        || zephyr_output.contains("Failed to publish");

    if received_count >= 1 {
        eprintln!(
            "\nSUCCESS: Native listener received {} messages from Zephyr talker",
            received_count
        );
    } else if zephyr_transport_err {
        panic!(
            "Zephyr talker transport failed — check zenoh-pico session setup. \
             Listener received 0 messages."
        );
    } else {
        panic!(
            "No messages delivered from Zephyr talker to native listener. \
             Listener received 0 sample lines."
        );
    }
}

/// Test: Native talker → Zephyr listener communication
///
/// Tests that a native Rust talker can send messages to a Zephyr listener.
/// This is the reverse direction of `test_zephyr_to_native_e2e`.
#[test]
fn test_native_to_zephyr_e2e() {
    if !require_zephyr() {
        nros_tests::skip!("Zephyr not available");
    }

    // #166 / phase-286 W1 — per-test ephemeral zenohd + locator override.
    eprintln!("Starting per-test zenohd router (ephemeral, #166)...");
    let router = nros_tests::fixtures::or_skip(ZenohRouter::start_unique());
    let locator = router.locator();
    eprintln!("zenohd locator: {locator}");

    // Build native talker
    let talker_path = build_native_talker().expect("Failed to build native-rs-talker");

    // Get Zephyr listener
    let zephyr_binary = resolve_example(Lang::Rust, "listener", Rmw::Zenoh);
    eprintln!("Zephyr listener binary: {}", zephyr_binary.display());

    // Start Zephyr listener first (so it subscribes before talker publishes)
    eprintln!("Starting Zephyr listener...");
    let mut zephyr =
        ZephyrProcess::start_with_locator(&zephyr_binary, ZephyrPlatform::NativeSim, &locator)
            .expect("Failed to start Zephyr listener");

    let _ = zephyr.wait_for_pattern(NODE_READY_MARKER, Duration::from_secs(30));

    // Start native talker connecting to zenohd
    use nros_tests::process::ManagedProcess;
    use std::process::Command;

    let mut talker_cmd = Command::new(talker_path);
    // Both connect to zenohd on localhost
    talker_cmd
        .env("NROS_LOCATOR", &locator)
        .env("RUST_LOG", "info");
    let mut talker = ManagedProcess::spawn_command(talker_cmd, "native-rs-talker")
        .expect("Failed to start talker");

    // Wait for communication
    eprintln!("Waiting for Native → Zephyr communication...");

    // Wait for the Zephyr listener's first SAMPLE line. 40 s: its zenoh-pico
    // subscription setup is slow on a slow native_sim host (issue #17); the
    // fast native talker only delivers once the subscriber is declared.
    //
    // issue 1026 — `wait_for_output` here was a terminal drain that killed the
    // image at the deadline, so the timeout was the guest's lifetime; on the
    // failure path it also short-circuited on markers this cell never asserts.
    // `wait_for_pattern` waits on the CONDITION, returns the accumulated
    // output either way, and leaves the image running for the `zephyr.kill()`
    // below.
    //
    // Bound stated: FIRST delivery only — nothing after the first received
    // sample is observed (issue 1013 is the counting shape).
    let zephyr_output = zephyr.wait_for_pattern(
        nros_tests::output::LISTENER_LOG_PREFIX,
        Duration::from_secs(40),
    );

    // Get native talker output for debugging
    let talker_output = talker
        .wait_for_all_output(Duration::from_secs(1))
        .unwrap_or_default();

    // Kill processes
    zephyr.kill();
    drop(talker);
    drop(router);

    eprintln!("\n=== Native talker output ===\n{}", talker_output);
    eprintln!("\n=== Zephyr listener output ===\n{}", zephyr_output);

    // Strict delivery check: the Zephyr listener must log at least one
    // canonical sample line (all c/cpp/rust fixtures, 198.2).
    let received_count = count_zephyr_received(&zephyr_output);
    let zephyr_transport_err = zephyr_output.contains("Transport(ConnectionFailed)")
        || zephyr_output.contains("z_declare_subscriber failed")
        || zephyr_output.contains("Failed to create subscriber");
    let talker_published = talker_output.contains(nros_tests::output::TALKER_LOG_PREFIX);

    if received_count >= 1 {
        eprintln!(
            "\nSUCCESS: Zephyr listener received {} messages from native talker",
            received_count
        );
    } else if zephyr_transport_err {
        panic!(
            "Zephyr listener transport failed — check zenoh-pico session setup. \
             Listener received 0 messages."
        );
    } else if !talker_published {
        panic!("Native talker did not publish — check talker output for errors");
    } else {
        panic!(
            "Native talker published but Zephyr listener received 0 messages. \
             Check Zephyr output for subscription/session errors."
        );
    }
}

/// Test: Bidirectional Native ↔ Zephyr communication
///
/// Tests that communication works in both directions simultaneously:
/// - Native talker → Zephyr listener
/// - Zephyr talker → Native listener
///
/// This test verifies that the bridge network and zenohd can handle
/// multiple clients and bidirectional traffic.
#[test]
fn test_bidirectional_native_zephyr_e2e() {
    if !require_zephyr() {
        nros_tests::skip!("Zephyr not available");
    }

    // #166 / phase-286 W1 — per-test ephemeral zenohd + locator override (all
    // four peers dial THIS router: natives via NROS_LOCATOR, Zephyr images via
    // `-testargs --nros-locator`).
    eprintln!("Starting per-test zenohd router (ephemeral, #166)...");
    let router = nros_tests::fixtures::or_skip(ZenohRouter::start_unique());
    let locator = router.locator();
    eprintln!("zenohd locator: {locator}");

    // Build all binaries
    let native_talker_path = build_native_talker().expect("Failed to build native-rs-talker");
    let native_listener_path = build_native_listener().expect("Failed to build native-rs-listener");
    let zephyr_talker_binary = resolve_example(Lang::Rust, "talker", Rmw::Zenoh);
    let zephyr_listener_binary = resolve_example(Lang::Rust, "listener", Rmw::Zenoh);

    eprintln!("Native talker: {}", native_talker_path.display());
    eprintln!("Native listener: {}", native_listener_path.display());
    eprintln!("Zephyr talker: {}", zephyr_talker_binary.display());
    eprintln!("Zephyr listener: {}", zephyr_listener_binary.display());

    use nros_tests::process::ManagedProcess;
    use std::process::Command;

    // Start listeners first (both native and Zephyr)
    eprintln!("Starting listeners...");

    let mut native_listener_cmd = Command::new(native_listener_path);
    native_listener_cmd
        .env("NROS_LOCATOR", &locator)
        .env("RUST_LOG", "info");
    let mut native_listener =
        ManagedProcess::spawn_command(native_listener_cmd, "native-rs-listener")
            .expect("Failed to start native listener");

    // Note: Running multiple Zephyr processes simultaneously can cause issues
    // due to TAP interface conflicts. For this test, we use a staggered approach.
    let mut zephyr_listener = ZephyrProcess::start_with_locator(
        &zephyr_listener_binary,
        ZephyrPlatform::NativeSim,
        &locator,
    )
    .expect("Failed to start Zephyr listener");

    native_listener
        .wait_for_output_pattern(
            nros_tests::output::LISTENER_READY_MARKER,
            Duration::from_secs(5),
        )
        .expect("native listener did not become ready");
    let _ = zephyr_listener.wait_for_pattern(NODE_READY_MARKER, Duration::from_secs(30));

    // Start talkers
    eprintln!("Starting talkers...");

    let mut native_talker_cmd = Command::new(native_talker_path);
    native_talker_cmd
        .env("NROS_LOCATOR", &locator)
        .env("RUST_LOG", "info");
    let mut native_talker = ManagedProcess::spawn_command(native_talker_cmd, "native-rs-talker")
        .expect("Failed to start native talker");

    let mut zephyr_talker = ZephyrProcess::start_with_locator(
        &zephyr_talker_binary,
        ZephyrPlatform::NativeSim,
        &locator,
    )
    .expect("Failed to start Zephyr talker");

    eprintln!("Waiting for bidirectional communication...");
    // 45 s: both directions gate on a slow native_sim Zephyr endpoint (issue
    // #17) — the native listener waits on the slow Zephyr talker's first
    // publish (~20 s after boot), and the Zephyr listener's own subscription
    // setup is slow before the fast native talker's samples land.
    let native_ready_output = native_listener
        .wait_for_output_count(
            nros_tests::output::LISTENER_LOG_PREFIX,
            1,
            Duration::from_secs(45),
        )
        .unwrap_or_default();
    let _ = zephyr_listener.wait_for_pattern(
        nros_tests::output::LISTENER_LOG_PREFIX,
        Duration::from_secs(45),
    );

    // Collect outputs
    let native_remaining = native_listener
        .wait_for_all_output(Duration::from_secs(2))
        .unwrap_or_default();
    let native_listener_output = format!("{native_ready_output}{native_remaining}");
    let zephyr_listener_output = zephyr_listener
        .wait_for_output(Duration::from_secs(2))
        .unwrap_or_default();
    let native_talker_output = native_talker
        .wait_for_all_output(Duration::from_secs(1))
        .unwrap_or_default();
    let zephyr_talker_output = zephyr_talker
        .wait_for_output(Duration::from_secs(1))
        .unwrap_or_default();

    // Kill all processes
    zephyr_talker.kill();
    zephyr_listener.kill();
    drop(native_talker);
    drop(native_listener);
    drop(router);

    eprintln!("\n=== Native talker output ===\n{}", native_talker_output);
    eprintln!("\n=== Zephyr talker output ===\n{}", zephyr_talker_output);
    eprintln!(
        "\n=== Native listener output ===\n{}",
        native_listener_output
    );
    eprintln!(
        "\n=== Zephyr listener output ===\n{}",
        zephyr_listener_output
    );

    // Strict delivery counts: match only real sample lines, not setup
    // text like "Waiting for Int32 messages ...". All fixtures log the
    // canonical sample line (198.2).
    let native_received_count = count_pattern(
        &native_listener_output,
        nros_tests::output::LISTENER_LOG_PREFIX,
    );
    let zephyr_received_count = count_zephyr_received(&zephyr_listener_output);

    eprintln!("\n=== Results ===");
    eprintln!(
        "Direction 1 (Zephyr → Native): {} messages received",
        native_received_count
    );
    eprintln!(
        "Direction 2 (Native → Zephyr): {} messages received",
        zephyr_received_count
    );

    match (native_received_count >= 1, zephyr_received_count >= 1) {
        (true, true) => {
            eprintln!("\nSUCCESS: Bidirectional communication works!");
        }
        (true, false) => panic!(
            "Zephyr → Native works ({} msgs), Native → Zephyr failed (0 msgs)",
            native_received_count
        ),
        (false, true) => panic!(
            "Native → Zephyr works ({} msgs), Zephyr → Native failed (0 msgs)",
            zephyr_received_count
        ),
        (false, false) => {
            panic!("Bidirectional communication failed — 0 messages in both directions")
        }
    }
}

// =============================================================================
// Bespoke: Zephyr ↔ native cross-platform service (rust)
// =============================================================================

/// Test: Native service server + Zephyr service client
///
/// Tests cross-platform service communication with native server and Zephyr client.
#[test]
fn test_native_server_zephyr_client() {
    if !require_zephyr() {
        nros_tests::skip!("Zephyr not available");
    }

    // Start zenohd router
    // #166 / phase-286 W1 slice 4 — per-test ephemeral zenohd + locator override.
    eprintln!("Starting per-test zenohd router (ephemeral, #166)...");
    let router = nros_tests::fixtures::or_skip(ZenohRouter::start_unique());
    let locator = router.locator();
    eprintln!("zenohd locator: {locator}");

    // Build native service server
    let server_path =
        build_native_service_server().expect("Failed to build native-rs-service-server");

    // Get Zephyr service client
    let zephyr_binary = resolve_example(Lang::Rust, "service-client", Rmw::Zenoh);
    eprintln!("Zephyr client binary: {}", zephyr_binary.display());

    // Start native service server first
    use nros_tests::process::ManagedProcess;
    use std::process::Command;

    let mut server_cmd = Command::new(server_path);
    server_cmd
        .env("NROS_LOCATOR", &locator)
        .env("RUST_LOG", "info");
    let mut server = ManagedProcess::spawn_command(server_cmd, "native-rs-service-server")
        .expect("Failed to start native service server");

    server
        .wait_for_output_pattern(
            nros_tests::output::SERVICE_SERVER_READY_MARKER,
            Duration::from_secs(5),
        )
        .expect("native service server did not become ready");

    if !server.is_running() {
        let output = server
            .wait_for_all_output(Duration::from_secs(1))
            .unwrap_or_default();
        eprintln!("[FAIL] Native service server exited early");
        eprintln!("Output: {}", output);
        panic!("Native service server failed to start");
    }

    // Start Zephyr service client
    eprintln!("Starting Zephyr service client...");
    let mut zephyr =
        ZephyrProcess::start_with_locator(&zephyr_binary, ZephyrPlatform::NativeSim, &locator)
            .expect("Failed to start Zephyr service client");

    // Wait for service communication
    eprintln!("Waiting for Native server ↔ Zephyr client communication...");

    // Wait for Zephyr output
    let zephyr_output = zephyr
        .wait_for_output(Duration::from_secs(10))
        .unwrap_or_default();

    // Get native server output
    let server_output = server
        .wait_for_all_output(Duration::from_secs(2))
        .unwrap_or_default();

    // Kill processes
    zephyr.kill();
    drop(server);
    drop(router);

    eprintln!("\n=== Native server output ===\n{}", server_output);
    eprintln!("\n=== Zephyr client output ===\n{}", zephyr_output);

    // Check Zephyr client status
    // "Session opened" or "Service client ready" or "Sending:" all indicate connection
    let zephyr_connected = zephyr_output.contains("Session opened")
        || zephyr_output.contains("Service client ready")
        || zephyr_output.contains("Sending:");
    let zephyr_sent_request = zephyr_output.contains("Sending request")
        || zephyr_output.contains("Request:")
        || zephyr_output.contains("Sending:");
    let zephyr_got_response = zephyr_output.contains(nros_tests::output::SERVICE_RESULT_PREFIX);

    // Check native server status
    let server_received = server_output
        .contains(nros_tests::output::SERVICE_INCOMING_REQUEST_MARKER)
        || server_output.contains("Received request")
        || server_output.contains("Request:");

    if zephyr_got_response {
        let response_count =
            count_pattern(&zephyr_output, nros_tests::output::SERVICE_RESULT_PREFIX);
        eprintln!(
            "\nSUCCESS: Zephyr client received {} responses from native server",
            response_count
        );
    } else if zephyr_connected && zephyr_sent_request {
        panic!(
            "Zephyr service E2E failed — client sent requests but all timed out.\n\
             Server received request: {}\n\
             This indicates a zenoh queryable discovery issue. Verify:\n\
             - Zephyr binary rebuilt after CMakeLists.txt changes: `just zephyr build-fixtures`\n\
             - zenohd running on bridge IP and reachable from both native and Zephyr processes",
            server_received
        );
    } else if !zephyr_connected {
        panic!(
            "Zephyr service E2E failed — client did not connect to zenohd.\n\
             Verify:\n\
             - Zephyr binary up to date: run `just zephyr build-fixtures`\n\
             - zenohd reachable on the baked allocator port (NSOS forwards sockets to host loopback)"
        );
    } else {
        panic!(
            "Zephyr service E2E failed — incomplete communication.\n\
             Zephyr connected: {}, sent request: {}, got response: {}, server received: {}",
            zephyr_connected, zephyr_sent_request, zephyr_got_response, server_received
        );
    }
}

/// Test: Zephyr service server + Native service client
///
/// Tests cross-platform service communication with Zephyr server and native client.
#[test]
fn test_zephyr_server_native_client() {
    if !require_zephyr() {
        nros_tests::skip!("Zephyr not available");
    }

    // Start zenohd router
    eprintln!("Starting zenohd router...");
    // #166 / phase-286 W1 slice 4 — per-test ephemeral zenohd + locator override.
    let router = nros_tests::fixtures::or_skip(ZenohRouter::start_unique());
    let locator = router.locator();
    eprintln!("zenohd locator: {locator}");

    // Build native service client
    let client_path =
        build_native_service_client().expect("Failed to build native-rs-service-client");

    // Get Zephyr service server
    let zephyr_binary = resolve_example(Lang::Rust, "service-server", Rmw::Zenoh);
    eprintln!("Zephyr server binary: {}", zephyr_binary.display());

    // Start Zephyr service server first
    eprintln!("Starting Zephyr service server...");
    let mut zephyr =
        ZephyrProcess::start_with_locator(&zephyr_binary, ZephyrPlatform::NativeSim, &locator)
            .expect("Failed to start Zephyr service server");

    let _ = zephyr.wait_for_pattern(
        nros_tests::output::SERVICE_SERVER_READY_MARKER,
        Duration::from_secs(30),
    );

    // Start native service client
    use nros_tests::process::ManagedProcess;
    use std::process::Command;

    let mut client_cmd = Command::new(client_path);
    client_cmd
        .env("NROS_LOCATOR", &locator)
        .env("RUST_LOG", "info");
    let mut client = ManagedProcess::spawn_command(client_cmd, "native-rs-service-client")
        .expect("Failed to start native service client");

    // Get outputs
    let client_output = client
        .wait_for_output_count(
            nros_tests::output::SERVICE_RESULT_PREFIX,
            1,
            Duration::from_secs(30),
        )
        .unwrap_or_default();
    let zephyr_output = zephyr
        .wait_for_output(Duration::from_secs(2))
        .unwrap_or_default();

    // Kill processes
    zephyr.kill();
    drop(client);
    drop(router);

    eprintln!("\n=== Zephyr server output ===\n{}", zephyr_output);
    eprintln!("\n=== Native client output ===\n{}", client_output);

    // Check Zephyr server status. Reaching the readiness marker implies the
    // session opened + the service was declared (you cannot wait for service
    // requests without a session), so readiness IS the connection signal — the
    // old literal `"Session opened"` grep is stale (the example never prints it).
    let zephyr_ready = zephyr_output.contains(nros_tests::output::SERVICE_SERVER_READY_MARKER);
    let zephyr_connected = zephyr_ready;
    let zephyr_received =
        zephyr_output.contains(nros_tests::output::SERVICE_INCOMING_REQUEST_MARKER);
    let zephyr_replied = zephyr_output.contains("a: ");

    // Check native client status
    let client_got_response = client_output.contains(nros_tests::output::SERVICE_RESULT_PREFIX);

    if client_got_response {
        let response_count =
            count_pattern(&client_output, nros_tests::output::SERVICE_RESULT_PREFIX);
        eprintln!(
            "\nSUCCESS: Native client received {} responses from Zephyr server",
            response_count
        );
        if zephyr_replied {
            eprintln!("  - Zephyr server processed and replied to requests");
        }
    } else if zephyr_connected && zephyr_ready && !zephyr_received {
        panic!("Zephyr server ready but didn't receive requests");
    } else if !zephyr_connected {
        panic!("Zephyr server failed to connect to zenohd");
    } else {
        panic!(
            "Service communication failed:\n  zephyr_connected={}\n  zephyr_ready={}\n  zephyr_received={}\n  zephyr_replied={}\n  client_response={}",
            zephyr_connected, zephyr_ready, zephyr_received, zephyr_replied, client_got_response
        );
    }
}

// =============================================================================
// Bespoke: Zephyr ↔ native cross-platform pubsub (cpp)
// =============================================================================

/// Test: Zephyr C++ talker → native Rust listener (cross-platform)
#[test]
fn test_zephyr_cpp_talker_to_native_listener() {
    if !require_zephyr() {
        nros_tests::skip!("Zephyr not available");
    }

    // #166 / phase-286 W1 slice 2 — per-test ephemeral zenohd + locator override.
    let router = nros_tests::fixtures::or_skip(ZenohRouter::start_unique());
    let locator = router.locator();
    // Build native Rust listener
    let native_listener = match build_native_listener() {
        Ok(p) => p.to_path_buf(),
        Err(e) => {
            nros_tests::skip!("could not build native listener: {}", e);
        }
    };

    // Build Zephyr C++ talker
    let talker_binary = resolve_example(Lang::Cpp, "talker", Rmw::Zenoh);

    // Start native listener first (connects to zenohd)
    let mut listener_cmd = std::process::Command::new(&native_listener);
    listener_cmd.env("NROS_LOCATOR", &locator);
    listener_cmd.env("RUST_LOG", "info");
    let mut listener =
        nros_tests::fixtures::ManagedProcess::spawn_command(listener_cmd, "native-listener")
            .expect("Failed to start native listener");

    listener
        .wait_for_output_pattern(
            nros_tests::output::LISTENER_READY_MARKER,
            Duration::from_secs(5),
        )
        .expect("native listener did not become ready");

    // Start Zephyr C++ talker
    let mut talker =
        ZephyrProcess::start_with_locator(&talker_binary, ZephyrPlatform::NativeSim, &locator)
            .unwrap();

    // Wait for 2 messages: this test asserts `received_count >= 2` below, so
    // waiting for only 1 returned as soon as the first arrived and captured a
    // single sample line, failing deterministically. The Zephyr C++
    // talker publishes repeatedly (~every 2.5 s after a 5 s warm-up), so 2
    // messages arrive well within the 30 s budget.
    let listener_output = listener
        .wait_for_output_count(
            nros_tests::output::LISTENER_LOG_PREFIX,
            2,
            Duration::from_secs(30),
        )
        .unwrap_or_default();
    let talker_output = talker
        .wait_for_output(Duration::from_secs(2))
        .unwrap_or_default();

    eprintln!("\n=== Native listener output ===\n{}", listener_output);
    eprintln!("\n=== Zephyr C++ talker output ===\n{}", talker_output);

    let received_count = count_pattern(&listener_output, nros_tests::output::LISTENER_LOG_PREFIX);

    if received_count >= 2 {
        eprintln!(
            "\nSUCCESS: Native listener received {} messages from Zephyr C++ talker",
            received_count
        );
    } else if output::parse_talker(&talker_output).published_count > 0 {
        panic!(
            "Talker published but listener got only {} messages (expected >= 2)",
            received_count
        );
    } else {
        panic!(
            "Cross-platform C++ talker→native listener test failed (received {})",
            received_count
        );
    }
}

/// Test: native Rust talker → Zephyr C++ listener (cross-platform)
#[test]
fn test_native_talker_to_zephyr_cpp_listener() {
    if !require_zephyr() {
        nros_tests::skip!("Zephyr not available");
    }

    // #166 / phase-286 W1 slice 2 — per-test ephemeral zenohd + locator override.
    let router = nros_tests::fixtures::or_skip(ZenohRouter::start_unique());
    let locator = router.locator();
    // Build native Rust talker
    let native_talker = match build_native_talker() {
        Ok(p) => p.to_path_buf(),
        Err(e) => {
            nros_tests::skip!("could not build native talker: {}", e);
        }
    };

    // Build Zephyr C++ listener
    let listener_binary = resolve_example(Lang::Cpp, "listener", Rmw::Zenoh);

    // Start Zephyr listener first; wait for its subscription-ready
    // output marker so the native talker doesn't race a still-booting
    // subscriber (Phase 89.12 flake).
    let listener =
        ZephyrProcess::start_with_locator(&listener_binary, ZephyrPlatform::NativeSim, &locator)
            .unwrap();
    let listener_ready = listener.wait_for_pattern(NODE_READY_MARKER, Duration::from_secs(30));
    if !listener_ready.contains(NODE_READY_MARKER) {
        panic!(
            "Zephyr C++ listener didn't reach readiness within 30 s.\nOutput:\n{}",
            listener_ready
        );
    }
    let mut listener = listener;

    // Start native talker (connects to zenohd)
    let mut talker_cmd = std::process::Command::new(&native_talker);
    talker_cmd.env("NROS_LOCATOR", &locator);
    talker_cmd.env("RUST_LOG", "info");
    let mut talker =
        nros_tests::fixtures::ManagedProcess::spawn_command(talker_cmd, "native-talker")
            .expect("Failed to start native talker");

    // Probe for the 3rd sample line on the Zephyr side (early-exits
    // instead of the old 8 s+3 s blind sleep that couldn't keep
    // up with parallel-load variance).
    let _ = listener.wait_for_pattern(
        nros_tests::output::listener_line(3).as_str(),
        Duration::from_secs(30),
    );

    let talker_output = talker
        .wait_for_all_output(Duration::from_secs(2))
        .unwrap_or_default();
    let listener_output = listener
        .wait_for_output(Duration::from_secs(2))
        .unwrap_or_default();

    eprintln!("\n=== Native talker output ===\n{}", talker_output);
    eprintln!("\n=== Zephyr C++ listener output ===\n{}", listener_output);

    let received_count = count_pattern(&listener_output, nros_tests::output::LISTENER_LOG_PREFIX);

    if received_count >= 2 {
        eprintln!(
            "\nSUCCESS: Zephyr C++ listener received {} messages from native talker",
            received_count
        );
    } else if talker_output.contains(nros_tests::output::TALKER_LOG_PREFIX) {
        panic!(
            "Talker published but Zephyr got only {} messages (expected >= 2)",
            received_count
        );
    } else {
        panic!(
            "Cross-platform native talker→C++ listener test failed (received {})",
            received_count
        );
    }
}

// =============================================================================
// Bespoke: Zephyr workspace Entry E2E (Phase 225.P) — a WORKSPACE-kind cell
// (`(ZephyrNativeSim, Rust, Zenoh, EntryPubsub)` observes via an external
// native listener), not an example cell.
//
// The workspace Entry (`examples/workspaces/rust/src/zephyr_entry`) is the
// Zephyr sibling of the native / FreeRTOS / ThreadX workspace Entries: a
// SINGLE Zephyr application that hosts the whole launch-defined node set —
// talker AND listener — in one process via
// `nros::main!(launch = "demo_bringup:system.launch.xml")`. Built by the
// 225.P west lane into `build-ws-rs-entry-zenoh` and resolved here through
// `get_prebuilt_zephyr_workspace_entry()`.
//
// Single-session caveat: zenoh does NOT loop a session's own publications
// back to a subscriber in that same session, so the Entry's in-process
// listener cannot observe the in-process talker. We therefore assert
// delivery to a SECOND, EXTERNAL native listener — the same shape as the
// single-node Zephyr rust pubsub E2E — which is a real cross-process
// pub/sub observation through generated `std_msgs/Int32` on `/chatter`.
// =============================================================================

/// Zephyr workspace Entry boots on native_sim, brings up its launch node set
/// (talker + listener in one process), and its `/chatter` publications are
/// delivered cross-process to an external native listener.
#[test]
fn test_zephyr_workspace_entry_native_sim_e2e() {
    if !require_zephyr() {
        nros_tests::skip!("Zephyr not available");
    }

    // Resolve the prebuilt workspace-Entry binary. Tests never build
    // fixtures in-body; a missing/stale image fails fast with a
    // `just zephyr build-fixtures` hint.
    let entry_binary = get_prebuilt_zephyr_workspace_entry().expect(
        "Failed to resolve prebuilt Zephyr workspace Entry — \
         run `just zephyr build-fixtures` first",
    );
    eprintln!("Workspace Entry binary: {}", entry_binary.display());

    // #166 / phase-286 W1 — per-test ephemeral zenohd + locator override. The
    // ws-runtime Entry now reads `-testargs --nros-locator=<loc>` (the generated
    // `rust_main` prefers it over the baked locator, mirroring
    // `zephyr_component_main!`), so this test dials its own ephemeral router
    // instead of the shared baked rust-pubsub port — no longer serial.
    eprintln!("Starting per-test zenohd router (ephemeral port)...");
    let router = nros_tests::fixtures::or_skip(ZenohRouter::start_unique());
    let locator = router.locator();
    eprintln!("zenohd started on {locator}");

    // Build + start an EXTERNAL native listener on the same locator. The
    // Entry's talker publishes `/chatter`; this listener is the observable
    // delivery endpoint (the Entry's own in-process listener sees nothing —
    // no same-session zenoh loopback).
    // The ws demo Entry (`talker_pkg`) publishes std_msgs/Int32 on /chatter;
    // the observability listener must subscribe the SAME type (the message type
    // is baked into the wire keyexpr) or the router never matches it.
    //
    // phase-338 W3 — this was the EXAMPLE (`native/rust/listener`) with a
    // test-only `NROS_SUB_TYPE=int32` switch; the switch moved to
    // `bins/int32-sink`, which subscribes Int32 natively. Readiness is
    // unchanged ("Waiting for"); the per-message marker is not — the sink
    // prints "Received:", the example printed "I heard:".
    //
    // HARD FAILURE, not a skip — issue 1124, the sibling of the `.expect()` on
    // the Entry twenty lines up. One test, two spellings, and the skipping one
    // hid a lane gap: the row (`int32-sink-zenoh`, `linux,rust,zenoh`) is in
    // EVERY lane's coordinate set and in every lane's build set (measured over
    // tier1 / tier2 / tier2-nightly), and out-of-lane never reaches this arm
    // anyway — `require_coord_in_lane` panics `[SKIPPED:lane]` inside the
    // resolver, before the existence check. This is issue 1112's site one file
    // over.
    let listener_path = nros_tests::fixtures::build_int32_sink()
        .expect("int32-sink fixture (row `int32-sink-zenoh`)")
        .to_path_buf();
    use nros_tests::process::ManagedProcess;
    use std::process::Command;
    let mut listener_cmd = Command::new(listener_path);
    listener_cmd
        .env("NROS_LOCATOR", &locator)
        .env("RUST_LOG", "info");
    let mut listener = ManagedProcess::spawn_command(listener_cmd, "native-rs-listener")
        .expect("Failed to start listener");
    listener
        .wait_for_output_pattern(
            nros_tests::output::INT32_SINK_READY_MARKER,
            Duration::from_secs(5),
        )
        .expect("native listener did not become ready");

    // Boot the single-process Entry (talker + listener). `ZephyrProcess::Drop`
    // kills it, so no manual teardown is required on an early panic.
    eprintln!("Starting Zephyr workspace Entry...");
    let mut entry =
        ZephyrProcess::start_with_locator(&entry_binary, ZephyrPlatform::NativeSim, &locator)
            .expect("Failed to start Zephyr workspace Entry");

    // The external listener must log at least one real sample line.
    // Timeout is generous: on a slow native_sim host the Entry's zenoh-pico
    // session setup + first publish lands ~20 s after boot (steady-state
    // cadence then tracks the ~2.5 s lease keepalive).
    //
    // issue 1026 — was `wait_for_all_output(40s)`, which has no stop condition
    // and kills at the deadline, so the window WAS the listener's lifetime and
    // the cell always paid the full 40 s. `collect_until` waits on the sample
    // line the assertion names.
    //
    // Bound stated: FIRST cross-process delivery only. Whether the Entry keeps
    // publishing past that (the lease-lapse question of issue 1013) is not
    // observed here.
    let listener_output = listener.collect_until(
        nros_tests::output::INT32_LISTENER_LOG_PREFIX,
        Duration::from_secs(40),
    );
    let entry_output = entry
        .wait_for_output(Duration::from_secs(1))
        .unwrap_or_default();

    entry.kill();
    drop(listener);
    drop(router);

    eprintln!("\n=== Workspace Entry output ===\n{entry_output}");
    eprintln!("\n=== Native listener output ===\n{listener_output}");

    let received = count_pattern(
        &listener_output,
        nros_tests::output::INT32_LISTENER_LOG_PREFIX,
    );
    assert!(
        received >= 1,
        "Workspace Entry talker delivered no messages to the external native \
         listener (0 sample lines). The Entry boots talker+listener in one \
         process; cross-process delivery on `/chatter` is the asserted signal.\n\
         Entry output:\n{entry_output}\nListener output:\n{listener_output}",
    );

    eprintln!(
        "SUCCESS: workspace Entry talker delivered {received} message(s) to the external listener"
    );
}
