//! phase-267 (xrce variant) — DECLARATIVE cross-RMW bridge e2e (zenoh → XRCE).
//!
//! The declarative sibling of `bridge_mixed_rmw::test_zenoh_to_xrce_bridge_e2e`
//! (which drives the hand-written `bridge-zenoh-to-xrce-fwd` bin). Here the bridge
//! is the `bridge-xrce` `native_entry` — a PLAIN `nros::main!` whose
//! `system.toml` declares a `[[bridge]]` to `xrce:agent`. `nros sync` bakes
//! `nros-bridge.toml` (the xrce egress node carries the agent locator), the macro
//! emits `run_from_config_str` + the backend `register()` calls, and the runtime
//! forwards. xrce is LAZY-registration, so there is NO descriptor staging.
//!
//! ## Topology
//!
//! ```text
//!   native_rs_talker ─ zenoh ─► zenohd ─► native_entry (xrce bridge) ─ XRCE ─►
//!                                                                   MicroXRCEAgent
//!                                                                        │ DDS
//!                                                                        ▼
//!                                                          nros xrce listener
//! ```
//!
//! The entry BAKES its endpoints; phase-267 #113 overrides them at runtime so the
//! test uses an ephemeral zenohd + a unique agent + a unique cyclone domain
//! (`NROS_BRIDGE_S0_LOCATOR` / `NROS_BRIDGE_S1_LOCATOR` / `NROS_BRIDGE_S1_DOMAIN`).
//! The agent's DDS participant joins the XRCE-client-requested domain, so the
//! bridge egress and the listener share it.
//!
//! Skips cleanly when `zenohd` or the XRCE Agent is absent, or when this run's
//! lane does not select the fixtures' coordinates.
//!
//! ## A failed fixture resolution is a FAILURE here, never a skip (issue 1124)
//!
//! Out-of-lane never reaches this file: `require_workspace_in_lane` /
//! `require_coord_in_lane` run inside the resolver, BEFORE the existence check,
//! and panic `[SKIPPED:lane]`. All three fixtures named below have manifest
//! rows — `workspace-rust-native-bridge-xrce` and `int32-sink-xrce` at
//! `linux,rust,xrce`, `header-chatter-talker` at `linux,rust,zenoh` — and each
//! is in the build set of every lane selecting its coordinate (measured: tier 1
//! and tier2-nightly build all three; tier 2 selects only `linux,rust,zenoh`,
//! so the xrce entry lane-skips in the resolver before this file's first
//! `Err` arm is reachable). What is left is a missing / stale / failed-build
//! IN-LANE fixture, which issue 0584 makes a hard failure.
//!
//! `require_prebuilt_workspace_binary` carries no `gate_promised_fixtures()`
//! panic, so — unlike a cargo row — the entry's `Err` reached this file even in
//! a GATED run. That is the one that was actually laundering.

use std::{process::Command, time::Duration};

use nros_tests::{
    count_pattern,
    fixtures::{
        ManagedProcess, XrceAgent, ZenohRouter, build_int32_sink_rmw, build_native_talker_header,
        build_native_workspace_rust_bridge_xrce_entry, require_xrce_agent, require_zenohd,
        zenohd_unique,
    },
};
use rstest::rstest;

/// The generated `nros-bridge.toml` interns sessions to `s0` (zenoh ingress) and
/// `s1` (xrce egress); #113 overrides each at runtime.
const ZENOH_NODE: &str = "S0";
const XRCE_NODE: &str = "S1";

#[rstest]
fn declarative_zenoh_to_xrce_bridge_to_nros_listener(zenohd_unique: ZenohRouter) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }
    if !require_xrce_agent() {
        nros_tests::skip!("XRCE-DDS Agent not found");
    }
    let bridge_bin = build_native_workspace_rust_bridge_xrce_entry()
        .expect(
            "bridge-xrce native_entry fixture (row `workspace-rust-native-bridge-xrce`); run \
             `just native build-workspace-fixtures`",
        )
        .to_path_buf();

    let zenoh_locator = zenohd_unique.locator();
    let agent = XrceAgent::start_unique().expect("XRCE Agent start");
    let xrce_locator = agent.addr().to_string();
    let domain = nros_tests::unique_ros_domain_id();
    // The agent's accept loop needs ~1 s to settle before clients connect (else
    // the first XRCE connect hits Transport(ConnectionFailed)).
    std::thread::sleep(Duration::from_secs(1));

    // Bridge first — its egress XRCE publisher is what the listener discovers.
    // #113 overrides: zenoh ingress → the ephemeral router, xrce egress → the
    // unique agent, on the unique domain. No "Spinning" banner (plain nros::main!),
    // so give it a moment.
    let mut bridge_cmd = Command::new(&bridge_bin);
    bridge_cmd
        .env("RUST_LOG", "info")
        .env(format!("NROS_BRIDGE_{ZENOH_NODE}_LOCATOR"), &zenoh_locator)
        .env(
            format!("NROS_BRIDGE_{XRCE_NODE}_LOCATOR"),
            format!("udp/{xrce_locator}"),
        )
        .env(
            format!("NROS_BRIDGE_{XRCE_NODE}_DOMAIN"),
            domain.to_string(),
        );
    let mut bridge = ManagedProcess::spawn_command(bridge_cmd, "bridge-xrce-native_entry")
        .expect("spawn declarative xrce bridge entry");
    std::thread::sleep(Duration::from_secs(2));

    // xrce listener — connects to the same agent on the same domain. The
    // ws-bridge demo forwards std_msgs/Int32 on /chatter (issue #183), so the
    // observability listener must subscribe Int32; the Int32-typed agent topic
    // won't match a String sub.
    //
    // phase-338 W3 — this drove the EXAMPLE (`native/rust/listener`) with a
    // test-only `NROS_SUB_TYPE=int32` switch, because `bins/int32-sink` was
    // hardcoded to zenoh and could not serve an XRCE test. The sink now carries
    // the same `rmw-*` axis the examples do, so the example is free of the
    // switch. Marker moves with the binary: the sink prints "Received:"
    // (INT32_LISTENER_LOG_PREFIX), not the example's "I heard:".
    let xrce_listener_binary = build_int32_sink_rmw(nros_tests::fixtures::Rmw::Xrce)
        .expect("int32-sink xrce fixture (row `int32-sink-xrce`)")
        .to_path_buf();
    let mut listener_cmd = Command::new(&xrce_listener_binary);
    listener_cmd
        .env("RUST_LOG", "info")
        .env("ROS_DOMAIN_ID", domain.to_string())
        .env("NROS_LOCATOR", format!("udp/{xrce_locator}"));
    let mut listener =
        ManagedProcess::spawn_command(listener_cmd, "xrce-listener-declarative-bridge")
            .expect("spawn xrce listener");
    listener
        .wait_for_output_pattern(
            nros_tests::output::INT32_SINK_READY_MARKER,
            Duration::from_secs(8),
        )
        .expect("xrce listener did not become ready");

    let talker_binary = build_native_talker_header()
        .expect("talker `header` fixture (row `header-chatter-talker`)")
        .to_path_buf();
    let mut talker_cmd = Command::new(&talker_binary);
    // Match the Int32 bridge/listener (issue #183): publish std_msgs/Int32.
    // phase-338 W3 — this used the native rust talker with `NROS_PUB_TYPE=int32`;
    // that switch moved to `bins/header-chatter-talker`, which publishes Int32 on
    // /chatter natively, so the example can drop its test-only branching.
    talker_cmd
        .env("RUST_LOG", "info")
        .env("NROS_LOCATOR", &zenoh_locator);
    let mut talker = ManagedProcess::spawn_command(talker_cmd, "native-rs-talker-xrce-bridge")
        .expect("spawn talker");
    talker
        .wait_for_output_pattern(
            nros_tests::output::INT32_TALKER_LOG_PREFIX,
            Duration::from_secs(8),
        )
        .expect("talker did not publish first sample");

    let listener_output = listener
        .wait_for_output_count(
            nros_tests::output::INT32_LISTENER_LOG_PREFIX,
            2,
            Duration::from_secs(10),
        )
        .unwrap_or_default();

    talker.kill();
    bridge.kill();
    listener.kill();
    drop(agent);

    eprintln!("xrce listener output:\n{listener_output}");
    let received = count_pattern(
        &listener_output,
        nros_tests::output::INT32_LISTENER_LOG_PREFIX,
    );
    eprintln!("xrce listener received {received} bridged sample(s)");
    assert!(
        received >= 2,
        "expected ≥ 2 bridged samples to reach the xrce listener \
         (zenoh → declarative bridge-xrce entry → xrce agent), got {received}.\n\
         Full listener output:\n{listener_output}"
    );
}

// Issue 0352 / phase-324 — bind this test to `interop::CELLS`. The coordinates
// below must equal what the list declares for `declarative_bridge_zenoh_to_xrce`; adding/retiring an
// interop cell for this test, or drifting a cell's coordinate (issue 0341
// defect 2), turns this RED. Needs no fixtures — runs in tier 1.
#[test]
fn cases_bound_to_interop_cells() {
    #[allow(unused_imports)]
    use nros_tests::matrix::{Lang::*, PlatformId::*, Rmw::*, Workload::*};
    nros_tests::interop::assert_test_bound(
        "declarative_bridge_zenoh_to_xrce",
        &[(Linux, Rust, Zenoh, Pubsub)],
    );
}
