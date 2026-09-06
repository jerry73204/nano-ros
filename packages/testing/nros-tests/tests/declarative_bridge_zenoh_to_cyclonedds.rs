//! phase-267 W-B — DECLARATIVE cross-RMW bridge e2e (zenoh → Cyclone DDS).
//!
//! The declarative sibling of `bridge_zenoh_to_cyclonedds.rs` (which drives the
//! hand-written `bridge-zenoh-to-cyclonedds-fwd` bin). Here the bridge is the
//! `bridge-cyclonedds` `native_entry` — a PLAIN `nros::main!` whose
//! `demo_bringup/system.toml` declares a `[[bridge]]`. `nros sync` bakes
//! `nros-bridge.toml` (incl. the `std_msgs/Int32` field schema), the macro emits
//! `run_from_config_str` + the backend `register()` calls, and the runtime stages
//! the Cyclone descriptor from the config schema and forwards. This test codifies
//! the manual e2e that closed phase-267 W-B (issues 0106 / 0107 / 0109).
//!
//! ## Topology
//!
//! ```text
//!   native_rs_talker ── zenoh ──► zenohd ──► native_entry (declarative bridge)
//!   (rmw-zenoh fixture)           (router)        │  run_from_config pump
//!                                                 ▼
//!                                      nano cyclonedds C listener
//! ```
//!
//! ## Endpoints (runtime-overridable, #113)
//!
//! The declarative entry BAKES its endpoints from `demo_bringup/system.toml`, but
//! phase-267 #113 lets `run_from_config` override each `[[node]]`'s locator/domain
//! at runtime via `NROS_BRIDGE_<NODE>_{LOCATOR,DOMAIN}`. So this test — like the
//! imperative one — uses an EPHEMERAL zenohd + a `unique_ros_domain_id()` cyclone
//! domain (overriding the baked `s0` locator + `s1` domain), with no fixed-port /
//! fixed-domain collision risk.
//!
//! Skips cleanly when `zenohd` is absent, or when this run's lane does not
//! select the fixtures' coordinates.
//!
//! ## A failed fixture resolution is a FAILURE here, never a skip (issue 1124)
//!
//! Every resolver below is `.expect()`ed. That is not a blanket policy — it is
//! what the lane machinery leaves once out-of-lane is subtracted:
//!
//! * **Out of lane never reaches the caller.** `require_coord_in_lane` /
//!   `require_workspace_in_lane` run INSIDE the resolver, BEFORE the existence
//!   check, and panic `[SKIPPED:lane]`. So an `Err` here can never mean "this
//!   lane deliberately did not build it".
//! * **Every fixture this file names has a manifest row, in the build set of
//!   every lane that selects its coordinate.** Measured:
//!   `workspace-rust-native-bridge` and `int32-sink-cyclonedds` are
//!   `linux,rust,cyclonedds`; `header-chatter-talker` is `linux,rust,zenoh`.
//!   tier 1 and tier2-nightly select both coordinates and build all three rows;
//!   tier 2 selects only `linux,rust,zenoh`, so the cyclone rows lane-skip in
//!   the resolver above and this file's `Err` arm is unreachable there.
//! * So an `Err` is a missing, stale or failed-to-build IN-LANE fixture, which
//!   issue 0584 makes a hard failure. `check-skip-budget` already fails the run
//!   for any `not prebuilt` skip — converting only moves the red from the gate,
//!   twenty minutes later and site-less, onto the site.
//!
//! The workspace-entry resolver is the one that mattered most:
//! `require_prebuilt_workspace_binary` has no `gate_promised_fixtures()` panic,
//! so unlike a cargo row its `Err` reached this file even in a GATED run.

use std::{
    path::{Path, PathBuf},
    process::Command,
    time::Duration,
};

use nros_tests::{
    count_pattern,
    fixtures::{
        DEFAULT_ROS_DISTRO, ManagedProcess, Rmw, Ros2DdsProcess, ZenohRouter, build_int32_sink_rmw,
        build_native_talker_header, build_native_workspace_rust_bridge_entry,
        require_ros2_cyclonedds, require_zenohd,
    },
};
use rstest::rstest;

/// The generated `nros-bridge.toml` interns sessions to `s0` (zenoh ingress) and
/// `s1` (cyclonedds egress). phase-267 #113 — override the baked endpoints at
/// runtime via `NROS_BRIDGE_<NODE>_{LOCATOR,DOMAIN}` (node name upper-cased).
const ZENOH_NODE: &str = "S0";
const CYCLONE_NODE: &str = "S1";

/// Resolve the Int32 sink built for cyclonedds.
///
/// issue 0449 — this used to drive `examples/native/c/listener` with
/// `NROS_SUB_TYPE=int32`, a runtime message-type switch that existed ONLY for
/// this test. The examples follow the ROS demos, which publish
/// `std_msgs/String`; a test that needs a different payload uses a test bin.
/// `bins/int32-sink` gained the cyclone axis so this could stop bending an
/// example around a test.
///
/// HARD FAILURE, not a skip — issue 1124; see the module note below.
fn nano_cyclone_listener() -> PathBuf {
    build_int32_sink_rmw(Rmw::Cyclonedds)
        .map(Path::to_path_buf)
        .expect("int32-sink cyclonedds fixture (row `int32-sink-cyclonedds`)")
}

fn spawn_cyclone_listener(binary: &Path, domain: u8) -> ManagedProcess {
    let mut cmd = Command::new(binary);
    // The ws-bridge demo forwards std_msgs/Int32 on /chatter (issue #183), so the
    // observability listener must subscribe Int32 — the type is baked into the
    // wire keyexpr, and the Int32-typed cyclone topic won't match a String sub.
    cmd.env("ROS_DOMAIN_ID", domain.to_string())
        .env("RUST_LOG", "info");
    ManagedProcess::spawn_command(cmd, "nano-cyclone-listener-declarative-bridge")
        .expect("spawn nano cyclone listener")
}

fn spawn_zenoh_talker(bin: &Path, locator: &str) -> ManagedProcess {
    let mut cmd = Command::new(bin);
    // Match the Int32 bridge/listener (issue #183): publish std_msgs/Int32.
    // phase-338 W3 — this used the native rust talker with `NROS_PUB_TYPE=int32`;
    // that switch moved to `bins/header-chatter-talker`, which publishes Int32 on
    // /chatter natively, so the example can drop its test-only branching.
    cmd.env("RUST_LOG", "info").env("NROS_LOCATOR", locator);
    let mut talker = ManagedProcess::spawn_command(cmd, "native-rs-talker-declarative-bridge")
        .expect("spawn talker");
    talker
        .wait_for_output_pattern(
            nros_tests::output::INT32_TALKER_LOG_PREFIX,
            Duration::from_secs(8),
        )
        .expect("talker did not publish first sample");
    talker
}

/// zenoh talker → `bridge-cyclonedds` declarative bridge → nano cyclone listener.
/// Asserts the listener RECEIVES the forwarded `std_msgs/Int32` — the full
/// declarative path (descriptor staged from the config field schema, raw publish
/// accepted, sample delivered), no ROS 2 install needed.
#[rstest]
fn declarative_zenoh_to_cyclonedds_bridge_to_nano_listener() {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }
    // An unprovisioned cyclonedds submodule does NOT silently leave this
    // fixture absent: `workspace-fixtures-build.sh` fails loud and actionable
    // for a `NROS_RMW=cyclonedds` row on linux (issue 0120). So a resolver
    // `Err` here means the fixture is missing or stale in a lane that selected
    // it — hard failure (issue 1124).
    let bridge_bin = build_native_workspace_rust_bridge_entry()
        .expect(
            "bridge-cyclonedds native_entry fixture (row `workspace-rust-native-bridge`); run \
             `just native build-workspace-fixtures` (needs `just cyclonedds setup`)",
        )
        .to_path_buf();
    let listener_bin = nano_cyclone_listener();

    // Ephemeral router + unique cyclone domain (overriding the baked endpoints
    // below) so this test never collides with a concurrent one.
    let zenohd = nros_tests::fixtures::or_skip(ZenohRouter::start_unique());
    let zenoh_locator = zenohd.locator();
    let domain = nros_tests::unique_ros_domain_id();

    // Listener first — its subscription must be discoverable before the bridge's
    // cyclone egress publisher matches over SPDP.
    let mut listener = spawn_cyclone_listener(&listener_bin, domain);
    std::thread::sleep(Duration::from_secs(3));

    // phase-267 #113 — override the baked endpoints: point the zenoh ingress (`s0`)
    // at the ephemeral router and the cyclone egress (`s1`) at the unique domain.
    // The entry connects to zenohd, opens the cyclone egress, stages the descriptor,
    // and pumps. It has no startup banner, so give it a moment.
    let mut bridge_cmd = Command::new(&bridge_bin);
    bridge_cmd
        .env("RUST_LOG", "info")
        .env(format!("NROS_BRIDGE_{ZENOH_NODE}_LOCATOR"), &zenoh_locator)
        .env(
            format!("NROS_BRIDGE_{CYCLONE_NODE}_DOMAIN"),
            domain.to_string(),
        );
    let mut bridge = ManagedProcess::spawn_command(bridge_cmd, "bridge-cyclonedds-native_entry")
        .expect("spawn declarative bridge entry");
    std::thread::sleep(Duration::from_secs(2));

    let talker_binary = build_native_talker_header()
        .expect("talker `header` fixture (row `header-chatter-talker`)")
        .to_path_buf();
    let mut talker = spawn_zenoh_talker(&talker_binary, &zenoh_locator);

    let listener_output = listener
        .wait_for_output_count(
            nros_tests::output::INT32_LISTENER_LOG_PREFIX,
            2,
            Duration::from_secs(12),
        )
        .unwrap_or_default();

    talker.kill();
    bridge.kill();
    listener.kill();
    drop(zenohd);

    eprintln!("nano cyclone listener output:\n{listener_output}");
    let received = count_pattern(
        &listener_output,
        nros_tests::output::INT32_LISTENER_LOG_PREFIX,
    );
    eprintln!("nano cyclone listener received {received} bridged sample(s)");
    assert!(
        received >= 2,
        "expected ≥ 2 bridged samples to reach the nano cyclone listener \
         (zenoh → declarative bridge-cyclonedds entry → cyclonedds), got {received}.\n\
         Full listener output:\n{listener_output}"
    );
}

/// phase-267 (non-flat types) — NESTED forwarding: the talker (`--features
/// header`) publishes `std_msgs/Header` (= `builtin_interfaces/Time` stamp +
/// string) on /header, which the bridge stages via a typed `register::<Header>()`
/// and forwards; a stock `rmw_cyclonedds_cpp` `ros2 topic echo` receives it with
/// the nested `stamp` intact. Env-gated on a ROS 2 + cyclonedds install (the nano
/// cyclone listener is Int32-only, so the receiver is ros2). Skips otherwise.
#[rstest]
fn declarative_zenoh_to_cyclonedds_nested_header_to_ros2() {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }
    if !require_ros2_cyclonedds() {
        nros_tests::skip!("ROS 2 + rmw_cyclonedds_cpp not available");
    }
    let bridge_bin = build_native_workspace_rust_bridge_entry()
        .expect("bridge-cyclonedds native_entry fixture (row `workspace-rust-native-bridge`)")
        .to_path_buf();
    let talker_bin = build_native_talker_header()
        .expect("talker `header` fixture (row `header-chatter-talker`)")
        .to_path_buf();

    let zenohd = nros_tests::fixtures::or_skip(ZenohRouter::start_unique());
    let zenoh_locator = zenohd.locator();
    let domain = nros_tests::unique_ros_domain_id();

    let mut ros2_sub = Ros2DdsProcess::topic_echo_cyclonedds_with_domain(
        "/header",
        "std_msgs/msg/Header",
        DEFAULT_ROS_DISTRO,
        domain,
    )
    .expect("start ros2 cyclone echo /header");
    std::thread::sleep(Duration::from_secs(2));

    // #113 overrides: zenoh ingress → ephemeral router, cyclone egress → unique domain.
    let mut bridge_cmd = Command::new(&bridge_bin);
    bridge_cmd
        .env("RUST_LOG", "info")
        .env(format!("NROS_BRIDGE_{ZENOH_NODE}_LOCATOR"), &zenoh_locator)
        .env(
            format!("NROS_BRIDGE_{CYCLONE_NODE}_DOMAIN"),
            domain.to_string(),
        );
    let mut bridge =
        ManagedProcess::spawn_command(bridge_cmd, "bridge-cyclonedds-native_entry-nested")
            .expect("spawn declarative bridge entry");
    std::thread::sleep(Duration::from_secs(2));

    let mut talker_cmd = Command::new(&talker_bin);
    talker_cmd
        .env("RUST_LOG", "info")
        .env("NROS_LOCATOR", &zenoh_locator);
    let mut talker = ManagedProcess::spawn_command(talker_cmd, "native-rs-talker-header-bridge")
        .expect("spawn header talker");
    talker
        .wait_for_output_pattern("Published Header", Duration::from_secs(8))
        .expect("talker did not publish a Header");

    let ros2_output = ros2_sub
        .wait_for_output(Duration::from_secs(12))
        .unwrap_or_default();

    talker.kill();
    bridge.kill();

    eprintln!("ros2 cyclone /header echo:\n{ros2_output}");
    // A received Header renders the nested `stamp` (`sec:`) — proof the nested
    // descriptor round-tripped, not just that bytes arrived.
    let got = count_pattern(&ros2_output, "sec:");
    assert!(
        got > 0,
        "stock ros2 cyclone subscriber received no bridged Header on /header \
         (zenoh → declarative bridge → rmw_cyclonedds_cpp), got:\n{ros2_output}"
    );
}

// Issue 0352 / phase-324 — bind this test to `interop::CELLS`. The coordinates
// below must equal what the list declares for `declarative_bridge_zenoh_to_cyclonedds`; adding/retiring an
// interop cell for this test, or drifting a cell's coordinate (issue 0341
// defect 2), turns this RED. Needs no fixtures — runs in tier 1.
#[test]
fn cases_bound_to_interop_cells() {
    #[allow(unused_imports)]
    use nros_tests::matrix::{Lang::*, PlatformId::*, Rmw::*, Workload::*};
    nros_tests::interop::assert_test_bound(
        "declarative_bridge_zenoh_to_cyclonedds",
        &[(Linux, Rust, Zenoh, Pubsub)],
    );
}
