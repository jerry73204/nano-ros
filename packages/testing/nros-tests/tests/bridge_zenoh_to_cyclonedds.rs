//! Issue #53 — zenoh → Cyclone DDS mixed-RMW bridge e2e (stock-cyclonedds
//! variant; the cyclonedds sibling of `bridge_mixed_rmw.rs`).
//!
//! ## Topology
//!
//! ```text
//!   native_rs_talker  ─── zenoh ───►  zenohd  ───► bridge-zenoh-to-cyclonedds-fwd
//!   (rmw-zenoh fixture)               (router)         │  in-process pump
//!                                                      ▼
//!                                              Cyclone DDS /chatter (RTPS)
//! ```
//!
//! ## What it asserts
//!
//! The bridge logs `forwarded <n> bytes zenoh→cyclonedds` for each sample whose
//! zenoh ingress callback fires AND whose Cyclone `publish_raw` returns `Ok` —
//! i.e. the cyclonedds-specific path (descriptor staged via
//! `nros_rmw::register_type_descriptor`, raw publisher created, `dds_write`
//! accepted) succeeds on **live** data crossing the RMW boundary. This is the
//! cyclonedds analogue of `bridge_mixed_rmw`'s XRCE round-trip; it needs no DDS
//! receiver (the bridge links the vendored CycloneDDS), so it runs with just
//! zenohd + the zenoh talker fixture.
//!
//! Skips cleanly when `zenohd` or the prebuilt bridge binary is missing — same
//! `require_*` pattern as the rest of the suite.
//!
//! ## Receiver-asserting variants ("both A and B")
//!
//! The base test above proves the egress `dds_write` is *accepted*, but not that
//! any subscriber *receives* — so two follow-ups close the loop end-to-end:
//!
//! * **A — always-on gate** (`test_zenoh_to_cyclonedds_bridge_to_nano_listener`):
//!   receive side is an *in-tree* nano-ros Cyclone listener (`native/c/listener`,
//!   `rmw-cyclonedds` fixture). No ROS 2 install needed; composes with
//!   `cyclonedds_ros2_interop.rs` (nano cyclone pub → stock `rmw_cyclonedds_cpp`,
//!   already gated) to give full zenoh→stock coverage.
//! * **B — env-gated companion** (`test_zenoh_to_cyclonedds_bridge_ros2`): receive
//!   side is stock `ros2 topic echo` over `rmw_cyclonedds_cpp`. Skips unless ROS 2
//!   + cyclone are installed; the `cross-backend-bridges.md` recipe's live demo.
//!
//! ## A failed fixture resolution is a FAILURE here, never a skip (issue 1124)
//!
//! Out-of-lane never reaches this file: `require_in_lane` /
//! `require_coord_in_lane` run inside the resolver, BEFORE the existence check,
//! and panic `[SKIPPED:lane]`. Both fixtures this file names have manifest rows
//! at `linux,rust,cyclonedds` (`bins/bridge-zenoh-to-cyclonedds-fwd`) and
//! `linux,c,cyclonedds` (`examples/native/c/listener`, `build-cyclonedds`), and
//! both are in the build set of every lane that selects those coordinates
//! (measured: tier 1 and tier2-nightly build them; tier 2 selects neither, so
//! the resolver lane-skips there and this file's `Err` arm is unreachable).
//! What is left is a missing / stale / failed-build IN-LANE fixture, which
//! issue 0584 makes a hard failure — and which `check-skip-budget` already
//! fails the run for, site-less, twenty minutes downstream.

use std::{
    path::{Path, PathBuf},
    process::Command,
    time::Duration,
};

use nros_tests::{
    count_pattern,
    fixtures::{
        DEFAULT_ROS_DISTRO, ManagedProcess, Rmw, Ros2DdsProcess, ZenohRouter,
        build_bridge_zenoh_to_cyclonedds_fwd, build_native_c_example_rmw, require_ros2_cyclonedds,
        require_zenohd, talker_binary, zenohd_unique,
    },
};
use rstest::rstest;

const TOPIC: &str = "/chatter";
const MSG_TYPE: &str = "std_msgs/msg/String";

/// Resolve the prebuilt native Cyclone C listener. Mirrors
/// `cyclonedds_ros2_interop::nano_cyclone_c_binary`.
///
/// HARD FAILURE, not a skip — issue 1124; see the module note above.
fn nano_cyclone_listener() -> PathBuf {
    build_native_c_example_rmw("listener", "c_listener", Rmw::Cyclonedds)
        .expect("native/c/listener cyclonedds fixture (run `just cyclonedds setup`)")
}

fn spawn_cyclone_listener(binary: &Path, domain: u8) -> ManagedProcess {
    // The native/c/listener cyclone fixture statically links `libddsc.a`
    // (self-provisioned from source, Phase 186) — no `LD_LIBRARY_PATH` wiring.
    let mut cmd = Command::new(binary);
    cmd.env("ROS_DOMAIN_ID", domain.to_string())
        .env("RUST_LOG", "info");
    ManagedProcess::spawn_command(cmd, "nano-cyclone-listener-bridge-e2e")
        .expect("spawn nano cyclone listener")
}

/// Spawn the bridge (zenoh ingress @ `locator`, Cyclone egress @ `domain`).
fn spawn_bridge(bin: &Path, locator: &str, domain: u8, label: &'static str) -> ManagedProcess {
    let mut cmd = Command::new(bin);
    cmd.env("RUST_LOG", "info")
        .env("ZENOH_LOCATOR", locator)
        .env("ROS_DOMAIN_ID", domain.to_string());
    let mut bridge = ManagedProcess::spawn_command(cmd, label).expect("spawn bridge");
    bridge
        .wait_for_output_pattern("Spinning", Duration::from_secs(10))
        .expect("bridge did not finish session setup (zenoh + cyclonedds egress)");
    bridge
}

fn spawn_zenoh_talker(bin: &Path, locator: &str, label: &'static str) -> ManagedProcess {
    let mut cmd = Command::new(bin);
    cmd.env("RUST_LOG", "info").env("NROS_LOCATOR", locator);
    let mut talker = ManagedProcess::spawn_command(cmd, label).expect("spawn talker");
    talker
        .wait_for_output_pattern(
            nros_tests::output::TALKER_LOG_PREFIX,
            Duration::from_secs(8),
        )
        .expect("talker did not publish first sample");
    talker
}

#[rstest]
fn test_zenoh_to_cyclonedds_bridge_e2e(zenohd_unique: ZenohRouter, talker_binary: PathBuf) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    let bridge_bin = build_bridge_zenoh_to_cyclonedds_fwd()
        .expect("bridge-zenoh-to-cyclonedds-fwd fixture (bins/bridge-zenoh-to-cyclonedds-fwd)")
        .to_path_buf();

    let zenoh_locator = zenohd_unique.locator();

    // 1. Spawn the bridge: zenoh ingress + Cyclone DDS egress.
    //
    // issue 0580 — an ASSIGNED domain, like the other two tests in this file.
    // This one had `"0"` hardcoded, which is the worst literal available: it is
    // where every process that never thought about a domain also lands. Nothing
    // here consumes the Cyclone side (the assertion reads the bridge's own
    // "forwarded" log), so the domain is free — which is exactly why it must
    // not be shared.
    let domain = nros_tests::unique_ros_domain_id();
    let mut bridge_cmd = Command::new(&bridge_bin);
    bridge_cmd
        .env("RUST_LOG", "info")
        .env("ZENOH_LOCATOR", &zenoh_locator)
        .env("ROS_DOMAIN_ID", domain.to_string());
    let mut bridge = ManagedProcess::spawn_command(bridge_cmd, "bridge-zenoh-to-cyclonedds-fwd")
        .expect("spawn bridge");
    bridge
        .wait_for_output_pattern("Spinning", Duration::from_secs(10))
        .expect("bridge did not finish session setup (zenoh + cyclonedds egress)");

    // 2. Spawn the zenoh talker — its String samples land on the bridge's zenoh
    //    ingress, which republishes them onto Cyclone DDS.
    let mut talker_cmd = Command::new(&talker_binary);
    talker_cmd
        .env("RUST_LOG", "info")
        .env("NROS_LOCATOR", &zenoh_locator);
    let mut talker =
        ManagedProcess::spawn_command(talker_cmd, "native-rs-talker-cyclonedds-bridge")
            .expect("spawn talker");
    talker
        .wait_for_output_pattern(
            nros_tests::output::TALKER_LOG_PREFIX,
            Duration::from_secs(8),
        )
        .expect("talker did not publish first sample");

    // 3. Collect the bridge's forward log for a window covering ≥ 2 of the 1 Hz
    //    publishes (each ingress→egress hop is sub-millisecond).
    let bridge_output = bridge
        .wait_for_output_count("forwarded", 2, Duration::from_secs(10))
        .unwrap_or_default();

    talker.kill();
    bridge.kill();

    eprintln!("bridge output:\n{bridge_output}");
    let forwarded = count_pattern(&bridge_output, "forwarded");
    eprintln!("bridge forwarded {forwarded} sample(s) zenoh→cyclonedds");
    assert!(
        forwarded >= 2,
        "expected ≥ 2 samples forwarded zenoh→cyclonedds (descriptor staged + raw publish \
         accepted on live data), got {forwarded}.\nFull bridge output:\n{bridge_output}"
    );
}

/// Path A — always-on receiver gate. zenoh talker → bridge → in-tree nano-ros
/// Cyclone listener. Asserts the listener *receives* (not just that the bridge
/// forwarded), closing the egress loop without a ROS 2 install.
#[rstest]
fn test_zenoh_to_cyclonedds_bridge_to_nano_listener(
    zenohd_unique: ZenohRouter,
    talker_binary: PathBuf,
) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }
    let bridge_bin = build_bridge_zenoh_to_cyclonedds_fwd()
        .expect("bridge-zenoh-to-cyclonedds-fwd fixture (bins/bridge-zenoh-to-cyclonedds-fwd)")
        .to_path_buf();
    let listener_bin = nano_cyclone_listener();

    let zenoh_locator = zenohd_unique.locator();
    // PID-seeded so the cyclone egress/listener never share a domain with a
    // concurrent interop test (DDS multicast discovery on the shared domain).
    let domain = nros_tests::unique_ros_domain_id();

    // Listener first — its subscription must be discoverable before the bridge's
    // egress publisher matches over SPDP.
    let mut listener = spawn_cyclone_listener(&listener_bin, domain);
    std::thread::sleep(Duration::from_secs(3));

    let mut bridge = spawn_bridge(
        &bridge_bin,
        &zenoh_locator,
        domain,
        "bridge-zenoh-to-cyclonedds-fwd-nano",
    );
    let mut talker = spawn_zenoh_talker(
        &talker_binary,
        &zenoh_locator,
        "native-rs-talker-cyclone-bridge-nano",
    );

    let listener_output = listener
        .wait_for_output_count(
            nros_tests::output::LISTENER_LOG_PREFIX,
            2,
            Duration::from_secs(12),
        )
        .unwrap_or_default();

    talker.kill();
    bridge.kill();
    listener.kill();

    eprintln!("nano cyclone listener output:\n{listener_output}");
    let received = count_pattern(&listener_output, nros_tests::output::LISTENER_LOG_PREFIX);
    eprintln!("nano cyclone listener received {received} bridged sample(s)");
    assert!(
        received >= 2,
        "expected ≥ 2 bridged samples to reach the nano cyclone listener \
         (zenoh → bridge → cyclonedds), got {received}.\nFull listener output:\n{listener_output}"
    );
}

/// Path B — env-gated companion (the book recipe's stock-ROS 2 demo).
/// zenoh talker → bridge → stock `ros2 topic echo` over `rmw_cyclonedds_cpp`.
#[rstest]
fn test_zenoh_to_cyclonedds_bridge_ros2(zenohd_unique: ZenohRouter, talker_binary: PathBuf) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }
    if !require_ros2_cyclonedds() {
        nros_tests::skip!("ROS 2 + rmw_cyclonedds_cpp not available");
    }
    let bridge_bin = build_bridge_zenoh_to_cyclonedds_fwd()
        .expect("bridge-zenoh-to-cyclonedds-fwd fixture (bins/bridge-zenoh-to-cyclonedds-fwd)")
        .to_path_buf();

    let zenoh_locator = zenohd_unique.locator();
    let domain = nros_tests::unique_ros_domain_id();

    let mut ros2_sub = Ros2DdsProcess::topic_echo_cyclonedds_with_domain(
        TOPIC,
        MSG_TYPE,
        DEFAULT_ROS_DISTRO,
        domain,
    )
    .expect("start ros2 cyclone echo");
    std::thread::sleep(Duration::from_secs(2));

    let mut bridge = spawn_bridge(
        &bridge_bin,
        &zenoh_locator,
        domain,
        "bridge-zenoh-to-cyclonedds-fwd-ros2",
    );
    let mut talker = spawn_zenoh_talker(
        &talker_binary,
        &zenoh_locator,
        "native-rs-talker-cyclone-bridge-ros2",
    );

    let ros2_output = ros2_sub
        .wait_for_output(Duration::from_secs(12))
        .unwrap_or_default();

    talker.kill();
    bridge.kill();

    eprintln!("ROS 2 cyclone subscriber output:\n{ros2_output}");
    let n = count_pattern(&ros2_output, "data:");
    assert!(
        n > 0,
        "stock ROS 2 cyclone subscriber received no bridged samples \
         (zenoh → bridge → rmw_cyclonedds_cpp), got:\n{ros2_output}"
    );
}

// phase-329 W3 — bind this test to `interop::CELLS` (the pattern from
// xrce_ros2_interop). The coordinate below must equal what the list declares
// for `bridge_zenoh_to_cyclonedds`; drift turns this RED. Needs no fixtures — runs in tier 1.
#[test]
fn cases_bound_to_interop_cells() {
    #[allow(unused_imports)]
    use nros_tests::matrix::{Lang::*, PlatformId::*, Rmw::*, Workload::*};
    nros_tests::interop::assert_test_bound(
        "bridge_zenoh_to_cyclonedds",
        &[(Linux, Rust, Zenoh, Pubsub)],
    );
}
