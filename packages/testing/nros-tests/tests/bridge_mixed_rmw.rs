//! Phase 211.I — mixed-RMW bridge e2e.
//!
//! Reuses the Phase 110.G `tt-zenoh-to-xrce` bridge example as the gateway
//! fixture: it opens TWO RMW sessions in one process (zenoh ingress + XRCE
//! egress) via `Executor::open_with_rmw` + a second per-node `rmw()` builder
//! step, copies bytes from the zenoh `/chatter` subscription into a shared
//! buffer, and republishes them on XRCE under a TT-gated egress window.
//!
//! ## Topology
//!
//! ```text
//!   native_rs_talker  ─── zenoh ───►  zenohd  ───► tt-zenoh-to-xrce bridge
//!   (rmw-zenoh fixture)               (router)         │
//!                                                      │  in-process pump
//!                                                      ▼
//!                                              XRCE-DDS Agent
//!                                                      │
//!                                                      ▼
//!                                              xrce_listener
//!                                            (rmw-xrce fixture)
//! ```
//!
//! Skips cleanly when any of `zenohd` / XRCE-DDS Agent / the prebuilt bridge
//! binary is missing — same `require_*` pattern as the rest of the suite.
//!
//! Closes Phase 211.I's "headline use case" bullet: nano-ros nodes on
//! different RMWs discovering each other through `nros-bridge` /
//! `Executor::open_multi`-style multi-session topology.
//!
//! ## Stock-cyclonedds variant
//!
//! 211.I's original text named "stock cyclonedds Autoware listener" on the
//! receive side. That variant needs a cyclonedds-enabled bridge build (the
//! tt example currently links zenoh + XRCE only) and ROS 2 demo nodes
//! installed for the listener. The follow-up is captured as a `[ ]` bullet
//! in the phase doc; the in-tree zenoh-↔-XRCE round-trip below is the
//! foundation it builds on.

use std::{path::PathBuf, process::Command, time::Duration};

use nros_tests::{
    count_pattern,
    fixtures::{
        ManagedProcess, XrceAgent, ZenohRouter, build_bridge_zenoh_to_xrce_fwd, require_xrce_agent,
        require_zenohd, talker_binary, xrce_listener_binary, zenohd_unique,
    },
};
use rstest::rstest;

#[rstest]
fn test_zenoh_to_xrce_bridge_e2e(
    zenohd_unique: ZenohRouter,
    talker_binary: PathBuf,
    xrce_listener_binary: PathBuf,
) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }
    if !require_xrce_agent() {
        nros_tests::skip!("XRCE-DDS Agent not found");
    }

    // HARD FAILURE, not a skip — issue 1124. `require_in_lane` runs inside the
    // resolver BEFORE the existence check, so an out-of-lane coordinate has
    // already panicked `[SKIPPED:lane]` and never returns `Err` here. The row
    // (`bins/bridge-zenoh-to-xrce-fwd`, `linux,rust,xrce`) is in the build set
    // of every lane that selects that coordinate — measured: tier 1 and
    // tier2-nightly build it; tier 2 does not select `linux,rust,xrce`, so this
    // arm is unreachable there. What is left is a missing / stale / failed-build
    // IN-LANE fixture, which issue 0584 makes a hard failure.
    let bridge_bin = build_bridge_zenoh_to_xrce_fwd()
        .expect("bridge-zenoh-to-xrce-fwd fixture (bins/bridge-zenoh-to-xrce-fwd)")
        .to_path_buf();

    let zenoh_locator = zenohd_unique.locator();
    let agent = XrceAgent::start_unique().expect("XRCE Agent start");
    let xrce_locator = agent.addr().to_string();

    // Sibling `xrce_ros2_interop` tests show the agent's accept loop needs ~1 s
    // settle before clients can open sessions; without the gap, the first
    // XRCE connect hits `Transport(ConnectionFailed)` (observed locally).
    std::thread::sleep(Duration::from_secs(1));

    // 1. Spawn the bridge first — its egress XRCE publisher is what the
    //    listener needs to discover. ZENOH_LOCATOR → ingress, XRCE_LOCATOR →
    //    egress. The TT schedule has a 3 ms ingress window in a 10 ms major
    //    frame; at 1 Hz publish, each sample lands within an ingress window
    //    with overwhelming probability.
    let mut bridge_cmd = Command::new(&bridge_bin);
    bridge_cmd
        .env("RUST_LOG", "info")
        .env("ZENOH_LOCATOR", &zenoh_locator)
        .env("XRCE_LOCATOR", &xrce_locator);
    let mut bridge = ManagedProcess::spawn_command(bridge_cmd, "bridge-zenoh-to-xrce-fwd")
        .expect("spawn bridge");
    bridge
        .wait_for_output_pattern("Spinning", Duration::from_secs(8))
        .expect("bridge did not finish session setup");

    // 2. Spawn xrce listener — bridge's egress publisher is now declared, so
    //    the listener's subscribe matches on first poll. The XRCE backend
    //    parses `NROS_LOCATOR` after stripping a `udp/` / `udp4://` / `udp://`
    //    prefix (`session.c::locator_strip_udp_prefix`); without the prefix
    //    `parse_host_port` would split `tcp/127.0.0.1:port` at the last colon
    //    and feed `tcp/127.0.0.1` to `getaddrinfo`. `XRCE_AGENT_ADDR` is
    //    accepted by the Zephyr `nros::init` overload only — `from_env()`
    //    used by the native examples ignores it. (Some existing xrce_*
    //    tests set XRCE_AGENT_ADDR; those tests are diagnostic-only and
    //    don't assert a successful session.)
    let mut listener_cmd = Command::new(&xrce_listener_binary);
    listener_cmd
        .env("RUST_LOG", "info")
        .env("NROS_LOCATOR", format!("udp/{xrce_locator}"));
    let mut listener = ManagedProcess::spawn_command(listener_cmd, "xrce-listener-bridge-e2e")
        .expect("spawn xrce listener");
    listener
        .wait_for_output_pattern(
            nros_tests::output::LISTENER_READY_MARKER,
            Duration::from_secs(8),
        )
        .expect("xrce listener did not become ready");

    // 3. Spawn zenoh talker.
    let mut talker_cmd = Command::new(&talker_binary);
    talker_cmd
        .env("RUST_LOG", "info")
        .env("NROS_LOCATOR", &zenoh_locator);
    let mut talker = ManagedProcess::spawn_command(talker_cmd, "native-rs-talker-bridge-e2e")
        .expect("spawn talker");
    talker
        .wait_for_output_pattern(
            nros_tests::output::TALKER_LOG_PREFIX,
            Duration::from_secs(8),
        )
        .expect("talker did not publish first sample");

    // 4. Collect listener output for a window that comfortably covers the
    //    bridge's worst-case TT schedule (ingress copy → egress drain happens
    //    at most one major frame = 10 ms later, so 8 s of 1 Hz publishes ≥ 5
    //    bridged samples).
    let listener_output = listener
        .wait_for_output_count(
            nros_tests::output::LISTENER_LOG_PREFIX,
            2,
            Duration::from_secs(10),
        )
        .unwrap_or_default();

    talker.kill();
    bridge.kill();
    listener.kill();
    drop(agent);

    eprintln!("xrce listener output:\n{listener_output}");
    let received = count_pattern(&listener_output, nros_tests::output::LISTENER_LOG_PREFIX);
    eprintln!("xrce listener received {received} bridged sample(s)");
    assert!(
        received >= 2,
        "expected ≥ 2 bridged samples to reach the xrce listener (zenoh → bridge → xrce), got {received}.\n\
         Full listener output:\n{listener_output}"
    );
}
