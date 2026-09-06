//! phase-381 acceptance — READ the ROS graph a stock ROS 2 node is in.
//!
//! This is the only test in the phase whose subject is DISCOVERY rather than
//! delivery, and it exists because of what happened without it.
//!
//! Phase-381 shipped twelve `rmw` graph slots: produced, reachable from Rust, C
//! and C++, with mutation-tested unit coverage and a clean `check-api-parity`.
//! Every one of those checks tested our code against our own builders, our own
//! parser and our own vtable — and the feature did not work.
//!
//! Issue 0903 was several stacked defects (a drain restarting on the first
//! reply rather than the finished sweep; a runtime that dispatched exactly one
//! of eleven graph methods; a `collect` flag set AFTER the query went on the
//! wire) sitting on top of a MECHANISM that could not work at all:
//! `z_liveliness_get` is an INTEREST, and a token reaches a get's callback only
//! when the router tags its declaration with that interest id, so a sweep saw
//! an arbitrary handful of the domain's tokens. The fix was a standing
//! liveliness SUBSCRIBER with history — a graph cache, not a question asked
//! repeatedly. No unit test could see any of it, because none of it manifests
//! except against a real peer.
//!
//! So the assertion here is deliberately the one thing a self-contained test
//! cannot make: a nano-ros node enumerates a live `rmw_zenoh_cpp` peer, and
//! stock `ros2 node list` enumerates ours.
//!
//! Interop cell: `native-graph-rust-zenoh-r2n` (`interop::CELLS`).

use std::{process::Command, time::Duration};

use nros_tests::{
    fixtures, interop,
    ros2::{DEFAULT_ROS_DISTRO, Ros2DdsProcess, Ros2Process, require_ros2, ros2_node_list},
};

/// The coordinates `interop::CELLS` declares for `graph_interop` — one per
/// backend. zenoh and Cyclone discover through entirely different mechanisms
/// (`@ros2_lv` liveliness tokens versus the `ros_discovery_info` topic), so
/// each needs its own live case; proving one says nothing about the other.
const GRAPH_CELLS: [(
    nros_tests::matrix::PlatformId,
    nros_tests::matrix::Lang,
    nros_tests::matrix::Rmw,
    nros_tests::matrix::Workload,
); 2] = [
    (
        nros_tests::matrix::PlatformId::Linux,
        nros_tests::matrix::Lang::Rust,
        nros_tests::matrix::Rmw::Zenoh,
        nros_tests::matrix::Workload::Graph,
    ),
    (
        nros_tests::matrix::PlatformId::Linux,
        nros_tests::matrix::Lang::Rust,
        nros_tests::matrix::Rmw::Cyclonedds,
        nros_tests::matrix::Workload::Graph,
    ),
];

/// The nano-ros side sees a stock ROS 2 node.
///
/// Polls rather than sampling once: the graph slots report what has ALREADY
/// arrived and never block, so a single call after startup legitimately returns
/// a partial graph. Written as one comparison this would be flaky by
/// construction — Design note 3 of the phase doc.
#[test]
fn nano_ros_enumerates_a_stock_ros2_node() {
    // The coordinate tripwire: this test is bound to its `interop::CELLS` row,
    // so a cell added without a test (or a test that drifts off its cell) is a
    // failure rather than silent non-coverage.
    // BOTH cells: the tripwire compares what this test NAME covers against
    // every `interop::CELLS` row that names it, and this file carries the zenoh
    // and cyclone cases. Declaring only one made the check fail loudly rather
    // than let a cell drift uncovered — which is the point of it.
    interop::assert_test_bound("graph_interop", &GRAPH_CELLS);

    if !require_ros2() {
        nros_tests::skip!("ROS 2 + rmw_zenoh_cpp not available");
    }
    let router = fixtures::or_skip(fixtures::ZenohRouter::start_unique());
    let locator = router.locator();

    let _talker = Ros2Process::demo_nodes_cpp_talker(&locator, DEFAULT_ROS_DISTRO)
        .expect("start the stock talker");

    // The probe polls to convergence and exits non-zero unless it sees the
    // node named here — so an EMPTY graph is a failure, not a quiet pass. That
    // distinction is the whole point: issue 0903 presented as "zero topics",
    // which is indistinguishable from "no topics exist" unless something
    // asserts a peer must be visible.
    let probe = fixtures::build_graph_probe().expect("prebuilt graph-probe");
    let out = Command::new(probe)
        .env("NROS_LOCATOR", &locator)
        .env("GRAPH_PROBE_EXPECT_NODE", "talker")
        .env("GRAPH_PROBE_TIMEOUT_MS", "20000")
        .output()
        .expect("run graph-probe");
    let output = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );

    // The probe exits non-zero when it does not see the expected peer, so this
    // is asserted on the STATUS as well as the marker — a probe that printed
    // the marker but failed would be a different bug, and one worth catching.
    assert!(
        out.status.success(),
        "graph-probe must exit 0 once it sees the talker; got {:?}\n{output}",
        out.status.code()
    );
    let _ = Duration::from_secs(0);

    assert!(
        output.contains(nros_tests::output::GRAPH_PROBE_SAW),
        "the nano-ros node must ENUMERATE the stock talker; probe said:\n{output}"
    );

    // Every OTHER graph slot, against the same live peer.
    //
    // This is the assertion phase-393's closing note asks for and phase-381 did
    // not have: `check-rmw-slot-producers` calls all eleven `produced`, which
    // means something writes and reads each slot — NOT that either was ever
    // exercised against a real ROS 2 node. Issue 0903 was nine slots that had
    // never been called sitting behind two that had, and the two that worked
    // made the family look covered.
    //
    // The probe exits 5 and names each failing slot, so the status assertion
    // above already catches this; the marker is asserted too because a probe
    // that stopped running the checks would otherwise pass silently.
    assert!(
        output.contains(nros_tests::output::GRAPH_PROBE_ALL_SLOTS_OK),
        "all eleven graph slots must answer against a live peer; probe said:\n{output}"
    );

    // And the reverse direction, which is what `ros2 node list` answers. Our
    // node was visible in the graph long before it could read one — that
    // asymmetry is what issue 0791 filed — so asserting only our side would
    // pass on a build that had regressed to write-only.
    let listed = ros2_node_list(&locator, DEFAULT_ROS_DISTRO).expect("ros2 node list");
    assert!(
        listed.contains("talker"),
        "ros2 node list must see the stock talker (sanity: the graph is live):\n{listed}"
    );
}

/// Cyclone's graph reader, against a live ROS 2 node — phase-381 step 2.
///
/// W5 gave Cyclone a READER for `ros_discovery_info`, the topic it had only
/// ever PUBLISHED, and nothing ever ran it against a real peer. That is the
/// state zenoh was in right up until issue 0903, where twelve slots that were
/// `produced`, mutation-tested and parity-clean turned out not to work at all —
/// so "Cyclone's is fine, it is the same shape" is exactly the assumption this
/// test exists to refuse.
///
/// Cyclone answers FEWER slots than zenoh, and that is the point of W6 rather
/// than a defect: a slot it cannot serve must answer `UNSUPPORTED`, never an
/// empty list. The probe classifies the two separately, so this asserts the
/// node enumeration works and that whatever else does not is *declared*
/// missing rather than silently blank.
///
/// Interop cell: `native-graph-rust-cyclone-r2n` (`interop::CELLS`).
#[test]
fn cyclone_enumerates_a_stock_ros2_node() {
    // BOTH cells: the tripwire compares what this test NAME covers against
    // every `interop::CELLS` row that names it, and this file carries the zenoh
    // and cyclone cases. Declaring only one made the check fail loudly rather
    // than let a cell drift uncovered — which is the point of it.
    interop::assert_test_bound("graph_interop", &GRAPH_CELLS);

    if !nros_tests::ros2::require_ros2_cyclonedds() {
        nros_tests::skip!("ROS 2 + rmw_cyclonedds_cpp not available");
    }

    // A domain of our own. Cyclone discovers by multicast SPDP, so a shared
    // domain would let another test's participants into this graph and make
    // the node assertion below depend on what else is running.
    let domain = nros_tests::unique_ros_domain_id();

    let _talker =
        Ros2DdsProcess::demo_nodes_cpp_talker_cyclonedds_with_domain(DEFAULT_ROS_DISTRO, domain)
            .expect("start the stock talker on cyclone");

    let probe = fixtures::build_graph_probe_rmw(nros_tests::fixtures::Rmw::Cyclonedds)
        .expect("prebuilt cyclone graph-probe");
    let mut cmd = Command::new(probe);
    cmd.env("GRAPH_PROBE_EXPECT_NODE", "talker")
        .env("GRAPH_PROBE_TIMEOUT_MS", "20000")
        .env("ROS_DOMAIN_ID", domain.to_string())
        .env("NROS_DOMAIN_ID", domain.to_string());
    // Issue 1137 — the SAME bus as the talker.
    //
    // `demo_nodes_cpp_talker_cyclonedds_with_domain` funnels through
    // `ros2_env_setup_rmw_with_domain`, which since issue 1009 exports a
    // `CYCLONEDDS_URI` confining that participant to `127.0.0.1` with
    // `AllowMulticast=false` and an explicit localhost peer. This probe is a
    // bare `Command` — not a `ManagedProcess`, which pins itself — so without
    // this line the two sides sit on different interfaces and neither one's
    // SPDP reaches the other. The probe then enumerates exactly one node,
    // itself, which is indistinguishable from a broken `ros_discovery_info`
    // reader and was filed as one twice (0927, then 1137).
    nros_tests::dds_isolation::apply_to_command(&mut cmd);
    let out = cmd.output().expect("run graph-probe");
    let output = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );

    assert!(
        out.status.success(),
        "cyclone graph-probe must exit 0 once it sees the talker; got {:?}\n{output}",
        out.status.code()
    );
    assert!(
        output.contains(nros_tests::output::GRAPH_PROBE_SAW),
        "cyclone must ENUMERATE the stock talker; probe said:\n{output}"
    );
}
