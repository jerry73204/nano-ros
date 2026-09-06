//! phase-433 W6 (jobs 3–5) — what we ADVERTISE about ourselves, checked
//! against what a live ROS 2 peer reads for the same entities.
//!
//! Four slot families are `produced` per `just check rmw-slot-producers` and
//! have never been exercised against a peer. Phase-381 is why that matters:
//! twelve graph slots shipped produced, reachable from three languages,
//! mutation-tested and `check-api-parity`-clean, and the feature did not work
//! at all (issue 0903). Every check that passed compared our code against our
//! own builders, our own parser and our own vtable.
//!
//! | family | slots | what only a peer can say |
//! | --- | --- | --- |
//! | matched counts | `publisher_count_matched_subscriptions`, `subscription_count_matched_publishers` | the number is a statement ABOUT a peer; a self-contained test cannot move it in both directions |
//! | GID | `get_gid_for_publisher` | a gid's whole purpose is that another participant recognises it |
//! | actual QoS | `publisher_get_actual_qos`, `subscription_get_actual_qos` | issue 0823 found these reporting the REQUESTED profile as GRANTED, and the fix has never met a peer |
//! | serialization format | `get_serialization_format` | it is what a peer's rmw reads to decide CDR compatibility |
//!
//! ## Cyclone, and only Cyclone
//!
//! Cyclone is the one backend that fills any of these. zenoh reaches the vtable
//! through `RustBackendAdapter::VTABLE`, which ends `..EMPTY_VTABLE` and names
//! none of them; `nros-rmw-xrce`'s designated initialiser stops before them and
//! uORB's positional one does too. So the cell here is `linux/rust/cyclonedds`
//! and there is no zenoh sibling to write — a zenoh probe would assert against
//! six NULL pointers, which is a fact about the vtable, not about a peer.
//!
//! ## Why the nano side is a raw vtable driver
//!
//! Because there is no other side to drive. Three of these six slots have NO
//! consumer anywhere outside `nros-rmw-cyclonedds/tests/graph_counts.cpp` — no
//! Rust method on `Publisher`, no `nros_publisher_*` C entry, no dispatcher in
//! `nros-rmw-cffi`; `publisher_get_actual_qos` has exactly one caller, which
//! logs a downgrade warning and discards the profile. The probe's own module
//! doc carries the consequence: this proves the SLOTS against a live peer, not
//! an API path above them, because there is no API path above them to prove.
//!
//! ## The oracle for actual QoS — issue 0823's acceptance
//!
//! The probe asks for a ZERO-FILLED profile
//! (`NROS_RMW_QOS_PROFILE_SYSTEM_DEFAULT`) and Cyclone's `make_dds_qos`
//! resolves every field of it before the entity exists: SYSTEM_DEFAULT
//! reliability becomes RELIABLE (issue 0829), durability becomes VOLATILE,
//! history becomes KEEP_LAST, and a KEEP_LAST depth of 0 is clamped to 1
//! because Cyclone's `validate_history_qospolicy` rejects depth < 1 outright.
//!
//! | field | requested | granted | what the peer prints |
//! | --- | --- | --- | --- |
//! | reliability | `SYSTEM_DEFAULT` (0) | `RELIABLE` (1) | `Reliability: RELIABLE` |
//! | durability | `SYSTEM_DEFAULT` (0) | `VOLATILE` (2) | `Durability: VOLATILE` |
//! | history/depth | `SYSTEM_DEFAULT` (0) / 0 | `KEEP_LAST` (1) / 1 | `KEEP_LAST (1)` |
//! | liveliness | `SYSTEM_DEFAULT` (0) | `AUTOMATIC` (1) | `Liveliness: AUTOMATIC` |
//!
//! So a read-back that echoed the request would report 0 for all four while the
//! peer reports the resolved values, and the two would disagree in four places.
//! That is what makes this falsifiable rather than a tautology — and it is why
//! the expectation the peer is checked against is DERIVED from what the slot
//! said, never written out as a literal here.
//!
//! **What this does NOT prove, stated because the issue's own headline case is
//! the thing it cannot reach.** `dds_get_qos` reports the LOCAL entity's
//! effective QoS, and DDS never mutates a local entity's QoS as a result of
//! matching — request-vs-offered either matches or raises
//! `OFFERED/REQUESTED_INCOMPATIBLE_QOS`. So "a RELIABLE reader that matched a
//! BEST_EFFORT writer", which is the failure 0823 was filed about, is still
//! invisible to this slot; the mechanism that answers it is
//! `subscription_take_event`, which is W6 job 2's subject and not this file's.
//! What is proven here is that the read-back reports the entity Cyclone
//! actually built rather than the profile we handed it, and that a stock peer
//! sees the same entity we describe.
//!
//! Interop cell: `native-advertised-state-rust-cyclone-bidir` (`interop::CELLS`).
//! Focused runner: `just native test-ros2-advertised-state`.

use std::{path::Path, process::Command, time::Duration};

use nros_tests::{
    fixtures::{ManagedProcess, build_advertised_state_probe},
    interop, output,
    ros2::{
        DEFAULT_ROS_DISTRO, Ros2DdsProcess, await_topic_endpoints_cyclonedds, endpoint_gid_bytes,
    },
    skip, skip_class,
};

/// The coordinate `interop::CELLS` declares for this file. One cell: the four
/// families share one probe, one peer bus and one build.
///
/// NOT named `*_CELLS`: `no_local_axis_tables` forbids a `const` whose name
/// says "axis table" outside `matrix.rs` / `interop.rs`, and it is right to —
/// this is not a table of cells, it is the coordinate set
/// `assert_test_bound` COMPARES against the one in `interop::CELLS`. The name
/// is the only thing that made it look like a second SSoT.
const COVERED_COORDS: [(
    nros_tests::matrix::PlatformId,
    nros_tests::matrix::Lang,
    nros_tests::matrix::Rmw,
    nros_tests::matrix::Workload,
); 1] = [(
    nros_tests::matrix::PlatformId::Linux,
    nros_tests::matrix::Lang::Rust,
    nros_tests::matrix::Rmw::Cyclonedds,
    nros_tests::matrix::Workload::AdvertisedState,
)];

/// The node the probe registers, and therefore the `Node name:` a peer reports
/// for both of its endpoints. Selecting the endpoint by NODE rather than by
/// "first block of this kind" is issue 0690's rule: the report is a flat list
/// and reading the wrong block asserts against somebody else's profile.
const NODE: &str = "advertised_state_probe";
/// The topic the probe PUBLISHES on — a peer subscribing here moves
/// `publisher_count_matched_subscriptions`.
const PUB_TOPIC: &str = "/nros_advertised_pub";
/// The topic the probe SUBSCRIBES to — a peer publishing here moves
/// `subscription_count_matched_publishers`.
const SUB_TOPIC: &str = "/nros_advertised_sub";
const MSG_TYPE: &str = "std_msgs/msg/String";
/// The string the probe publishes. A peer that prints this decoded it.
const PAYLOAD: &str = "advertised-state-probe";

/// How long to wait for the graph to carry an endpoint. A DEADLINE, not a
/// sleep: `await_topic_endpoints_cyclonedds` polls, because "not there yet"
/// and "not there" print the same thing and under load the first is far more
/// likely (issues 0705, 0761).
const DISCOVERY_BUDGET: Duration = Duration::from_secs(25);
/// How long to wait for one matched-count EDGE after the peer that causes it
/// has been started or has exited.
const EDGE_BUDGET: Duration = Duration::from_secs(30);

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/// The precondition every live case shares. Returns the probe path.
///
/// `skip!` rather than a bare `return`: a bare return reports PASS, which is
/// issue 1135's whole subject and the reason `params` could not produce a
/// verdict at all.
fn require_probe() -> &'static Path {
    if !nros_tests::ros2::require_ros2_cyclonedds() {
        skip!(
            "ROS 2 + rmw_cyclonedds_cpp not available — install it from apt \
             (`ros-$ROS_DISTRO-rmw-cyclonedds-cpp`)."
        );
    }
    build_advertised_state_probe()
        .unwrap_or_else(|e| skip!("advertised-state-probe fixture not built: {e:?}"))
}

/// Spawn the probe on `domain`, holding its entities open for `hold`.
///
/// Mirrors `interop_e2e::spawn_nano_cyclone` — same `LD_LIBRARY_PATH` wiring to
/// the in-tree `libddsc`, and deliberately the same absence of a
/// `CYCLONEDDS_URI`: the loopback pin reaches the ROS side through
/// `ros2_env_setup_cyclonedds_with_domain` and the nano side of every existing
/// Cyclone interop cell runs without it. Changing that here would put this
/// file on a discovery configuration nothing else in the tree has run.
fn spawn_probe(probe: &Path, domain: u8, hold: Duration) -> ManagedProcess {
    let mut cmd = Command::new(probe);
    cmd.env("ROS_DOMAIN_ID", domain.to_string())
        .env("ADV_PROBE_NODE", NODE)
        .env("ADV_PROBE_PUB_TOPIC", PUB_TOPIC.trim_start_matches('/'))
        .env("ADV_PROBE_SUB_TOPIC", SUB_TOPIC.trim_start_matches('/'))
        .env("ADV_PROBE_HOLD_MS", hold.as_millis().to_string());
    let cyclone_lib = nros_tests::project_root().join("build/install/lib");
    let ld = match std::env::var_os("LD_LIBRARY_PATH") {
        Some(existing) if !existing.is_empty() => {
            let mut paths = vec![cyclone_lib];
            paths.extend(std::env::split_paths(&existing));
            std::env::join_paths(paths).expect("valid LD_LIBRARY_PATH")
        }
        _ => cyclone_lib.into_os_string(),
    };
    cmd.env("LD_LIBRARY_PATH", ld);
    ManagedProcess::spawn_command(cmd, "advertised-state-probe").expect("spawn probe")
}

/// Wait for the probe's READY line and return everything it printed up to it —
/// which is every read-back, because the probe prints them all before READY.
fn await_ready(probe: &mut ManagedProcess) -> String {
    probe
        .wait_for_output_pattern(output::ADV_PROBE_READY, Duration::from_secs(30))
        .unwrap_or_else(|e| {
            probe.kill();
            panic!(
                "the probe never reached {}. It prints {} before any entity exists, so a \
                 failure before that is a NULL slot rather than a peer problem.\n{e}",
                output::ADV_PROBE_READY,
                output::ADV_PROBE_SLOTS_OK
            )
        })
}

/// The text after `marker` on the line that carries it.
fn marker_value<'a>(out: &'a str, marker: &str) -> &'a str {
    out.lines()
        .find_map(|l| l.trim().strip_prefix(marker))
        .unwrap_or_else(|| panic!("the probe printed no `{marker}` line.\nOutput:\n{out}"))
        .trim()
}

/// One `key=value key=value …` QoS line, parsed.
fn parse_qos(line: &str) -> std::collections::BTreeMap<String, u64> {
    line.split_whitespace()
        .filter_map(|kv| kv.split_once('='))
        .map(|(k, v)| {
            (
                k.to_string(),
                v.parse()
                    .unwrap_or_else(|_| panic!("non-numeric QoS field `{k}={v}` in `{line}`")),
            )
        })
        .collect()
}

fn field(q: &std::collections::BTreeMap<String, u64>, k: &str) -> u64 {
    *q.get(k)
        .unwrap_or_else(|| panic!("QoS line is missing `{k}`: {q:?}"))
}

/// The token rclpy renders for an ABI reliability value.
///
/// Derived from what the SLOT said, never written as a literal expectation:
/// that is what turns "the peer prints RELIABLE" (which a hardcoded profile
/// would also satisfy) into "the peer prints what our read-back claims".
fn peer_reliability(v: u64) -> &'static str {
    match v {
        1 => "RELIABLE",
        2 => "BEST_EFFORT",
        0 => "SYSTEM_DEFAULT",
        _ => "UNKNOWN",
    }
}

fn peer_durability(v: u64) -> &'static str {
    match v {
        1 => "TRANSIENT_LOCAL",
        2 => "VOLATILE",
        0 => "SYSTEM_DEFAULT",
        _ => "UNKNOWN",
    }
}

fn peer_history(v: u64) -> &'static str {
    match v {
        1 => "KEEP_LAST",
        2 => "KEEP_ALL",
        0 => "SYSTEM_DEFAULT",
        _ => "UNKNOWN",
    }
}

fn peer_liveliness(v: u64) -> &'static str {
    match v {
        1 => "AUTOMATIC",
        2 => "MANUAL_BY_NODE",
        3 => "MANUAL_BY_TOPIC",
        0 => "SYSTEM_DEFAULT",
        _ => "UNKNOWN",
    }
}

/// The one endpoint of `kind` belonging to [`NODE`], or a panic that says which
/// of the two failure modes happened (issues 0690, 0761).
fn one_endpoint(
    found: &[nros_tests::ros2::TopicEndpoint],
    kind: &str,
    topic: &str,
    report: &str,
) -> String {
    match found {
        [] => panic!(
            "ros2 did not discover the nano-ros {kind} `{NODE}` on {topic} within {}s.\n\
             This is a DEADLINE, not a single shot, so the graph really did not carry it — \
             check the probe started and the domain matches, not discovery timing.\n{report}",
            DISCOVERY_BUDGET.as_secs()
        ),
        [e] => e.block.clone(),
        many => panic!(
            "{} endpoints named `{NODE}` of kind {kind} on {topic} — another participant is \
             using this node name on this domain (issue 0690), so asserting a profile here \
             would read someone else's.\n{report}",
            many.len()
        ),
    }
}

// ---------------------------------------------------------------------------
// The tripwire — no fixtures, no peer, runs in tier 1
// ---------------------------------------------------------------------------

/// Bind this file to `interop::CELLS`. Adding, retiring or mutating the cell
/// without tracking the test — or the test drifting off the cell's declared
/// coordinate (issue 0341 defect 2) — turns this RED.
///
/// The Lang here is `Rust` because the binary the cases spawn is a Rust bin
/// (`packages/testing/nros-tests/bins/advertised-state-probe`), which is the
/// phase-433 W3 rule: derive the axis from what is spawned. W3 exists because
/// `scenario_coord` returned `Lang::Rust` unconditionally while three cases
/// spawned C binaries, so the axis was inverted and agreed with itself.
#[test]
fn cases_bound_to_interop_cells() {
    interop::assert_test_bound("advertised_state_interop", &COVERED_COORDS);
}

// ---------------------------------------------------------------------------
// Job 4 (+ 5) — the actual-QoS read-back, against a peer that reads the same
// endpoints
// ---------------------------------------------------------------------------

/// Issue 0823's acceptance, run for the first time.
///
/// Three claims, in order of what they rule out:
///
/// 1. **The read-back is not an echo.** Requested and granted differ in
///    reliability, durability and depth. If the slot reported the request as
///    the grant — the state 0823 found — they would be equal.
/// 2. **A stock peer reads what the slot claims.** Each field's expectation is
///    computed FROM the granted value, so this is "the peer agrees with our
///    self-report", not "the peer prints a profile somebody wrote down here".
/// 3. **`get_serialization_format` says `cdr`,** which is what a peer's rmw
///    reads to decide compatibility. The peer-side half of that claim — that
///    a stock node actually decodes our bytes — is asserted by the
///    matched-count case, which already runs an echo peer.
#[test]
fn actual_qos_read_back_matches_what_a_stock_peer_reads() {
    interop::assert_test_bound("advertised_state_interop", &COVERED_COORDS);
    let probe_bin = require_probe();
    let domain = nros_tests::unique_ros_domain_id();
    let mut probe = spawn_probe(probe_bin, domain, Duration::from_secs(60));
    let ready = await_ready(&mut probe);

    let format = marker_value(&ready, output::ADV_FORMAT);
    let requested = parse_qos(marker_value(&ready, output::ADV_QOS_REQUESTED));
    let pub_granted = parse_qos(marker_value(&ready, output::ADV_QOS_PUB_GRANTED));
    let sub_granted = parse_qos(marker_value(&ready, output::ADV_QOS_SUB_GRANTED));

    let (report, found) = await_topic_endpoints_cyclonedds(
        DEFAULT_ROS_DISTRO,
        domain,
        PUB_TOPIC,
        &[("PUBLISHER", NODE)],
        DISCOVERY_BUDGET,
    )
    .unwrap_or_else(|e| {
        probe.kill();
        panic!("ros2 topic info failed: {e}")
    });
    let publisher = one_endpoint(&found[0], "PUBLISHER", PUB_TOPIC, &report);

    let (sub_report, sub_found) = await_topic_endpoints_cyclonedds(
        DEFAULT_ROS_DISTRO,
        domain,
        SUB_TOPIC,
        &[("SUBSCRIPTION", NODE)],
        DISCOVERY_BUDGET,
    )
    .unwrap_or_else(|e| {
        probe.kill();
        panic!("ros2 topic info failed: {e}")
    });
    let subscription = one_endpoint(&sub_found[0], "SUBSCRIPTION", SUB_TOPIC, &sub_report);
    probe.kill();

    // (3) The cheap one.
    assert_eq!(
        format, "cdr",
        "get_serialization_format must report the encoding a peer's rmw checks for \
         compatibility; it said {format:?}"
    );

    // (1) Non-vacuity. Without this the peer comparison below could be
    // satisfied by an echo, if the requested profile were ever changed to the
    // resolved one — which is exactly how a test stops testing anything.
    for f in ["reliability", "durability", "depth"] {
        assert_ne!(
            field(&requested, f),
            field(&pub_granted, f),
            "publisher_get_actual_qos reported the REQUESTED `{f}` as GRANTED — that is issue \
             0823 back. The probe asks for a zero-filled (SYSTEM_DEFAULT) profile precisely so \
             Cyclone must resolve it before the entity exists.\nrequested: {requested:?}\n\
             granted:   {pub_granted:?}"
        );
    }

    // (2) The live half. Each expectation is DERIVED from the slot's answer.
    let want = |q: &std::collections::BTreeMap<String, u64>| {
        vec![
            format!("Reliability: {}", peer_reliability(field(q, "reliability"))),
            format!("Durability: {}", peer_durability(field(q, "durability"))),
            format!(
                "{} ({})",
                peer_history(field(q, "history")),
                field(q, "depth")
            ),
            format!(
                "Liveliness: {}",
                peer_liveliness(field(q, "liveliness_kind"))
            ),
        ]
    };
    for expected in want(&pub_granted) {
        assert!(
            publisher.contains(&expected),
            "publisher_get_actual_qos says the granted profile contains `{expected}`, and the \
             stock peer reading the SAME endpoint does not agree. Our self-report and what is \
             on the wire have diverged.\nslot: {pub_granted:?}\npeer:\n{publisher}"
        );
    }
    for expected in want(&sub_granted) {
        assert!(
            subscription.contains(&expected),
            "subscription_get_actual_qos says the granted profile contains `{expected}`, and \
             the stock peer reading the SAME endpoint does not agree.\nslot: {sub_granted:?}\n\
             peer:\n{subscription}"
        );
    }
}

// ---------------------------------------------------------------------------
// Job 3a — matched counts, BOTH edges, BOTH slots
// ---------------------------------------------------------------------------

/// A peer appears and our count rises; the peer goes away and it falls.
///
/// Both edges, because only the pair rules out a stub: a slot returning a
/// constant, or the number of LOCAL entities, satisfies "returns OK" and
/// satisfies a single sample taken while a peer happens to be up. Only the
/// fall says the number tracks a peer.
///
/// Both slots, because they read different Cyclone primitives
/// (`dds_get_matched_subscriptions` on the writer,
/// `dds_get_matched_publications` on the reader) and the probe holds them on
/// SEPARATE topics — a writer and a reader on one topic in one participant
/// match each other, which would make "rises from zero" unobservable.
///
/// Polled, never sampled: these slots report what discovery has ALREADY
/// delivered and never block, so a single call after the peer starts
/// legitimately returns the answer from before it did. The probe does the
/// polling and prints an EDGE; this test waits for the edge.
///
/// It also carries the peer-side half of `get_serialization_format`: the echo
/// peer is a stock `rmw_cyclonedds_cpp` node with its own CDR deserializer, so
/// printing our payload back is the peer AGREEING with what that slot reports.
#[test]
fn matched_counts_rise_when_a_peer_appears_and_fall_when_it_leaves() {
    interop::assert_test_bound("advertised_state_interop", &COVERED_COORDS);
    let probe_bin = require_probe();
    let domain = nros_tests::unique_ros_domain_id();
    // Long enough for: READY, the echo peer's whole 10 s window plus its fall
    // edge, then the pub peer's rise and fall. A CEILING over four bounded
    // waits, not a sleep anybody is relying on — each wait below has its own
    // deadline and fails on it.
    let mut probe = spawn_probe(probe_bin, domain, Duration::from_secs(120));
    let _ready = await_ready(&mut probe);

    // ---- publisher side: a peer SUBSCRIBES to what we publish -------------
    let mut echo = Ros2DdsProcess::topic_echo_cyclonedds_with_domain(
        PUB_TOPIC,
        MSG_TYPE,
        DEFAULT_ROS_DISTRO,
        domain,
    )
    .unwrap_or_else(|e| {
        probe.kill();
        panic!("start the stock echo peer: {e}")
    });

    let rose = probe
        .wait_for_output_pattern(output::ADV_PUB_MATCHED_ROSE, EDGE_BUDGET)
        .unwrap_or_else(|e| {
            echo.kill();
            probe.kill();
            panic!(
                "publisher_count_matched_subscriptions never left 0 while a stock \
                 `ros2 topic echo {PUB_TOPIC}` was running on domain {domain}.\n{e}"
            )
        });
    assert!(
        rose.contains(output::ADV_PUB_MATCHED_ROSE),
        "internal: the wait returned Ok without the marker:\n{rose}"
    );

    // The peer exits on its own 10 s window. Drain it BEFORE asserting the
    // fall, because `ros2 topic echo`'s stdout is buffered until exit (issue
    // 1044) — there is nothing to read while it is alive.
    //
    // The error carries what the peer PRINTED, so it goes in the message
    // rather than into `echoed` (issue 0670): collapsing it to `""` would make
    // the payload assertion at the end report an empty transcript, and folding
    // the error TEXT in would make `contains(PAYLOAD)` match a complaint that
    // names the pattern it was looking for.
    let echoed = match echo.wait_for_output(Duration::from_secs(30)) {
        Ok(out) => out,
        Err(e) => {
            echo.kill();
            probe.kill();
            panic!("the stock echo peer produced no output before exiting: {e}")
        }
    };
    echo.kill();

    let fell = probe
        .wait_for_output_pattern(output::ADV_PUB_MATCHED_FELL, EDGE_BUDGET)
        .unwrap_or_else(|e| {
            probe.kill();
            panic!(
                "publisher_count_matched_subscriptions never came back to 0 after the peer \
                 left. A count that only ever rises is indistinguishable from one that is \
                 latched.\n{e}"
            )
        });
    assert!(
        fell.contains(output::ADV_PUB_MATCHED_FELL),
        "internal: the wait returned Ok without the marker:\n{fell}"
    );

    // ---- subscription side: a peer PUBLISHES what we subscribe to ---------
    let mut pubber = Ros2DdsProcess::topic_pub_cyclonedds_with_domain(
        SUB_TOPIC,
        MSG_TYPE,
        "{data: from-ros2}",
        5,
        DEFAULT_ROS_DISTRO,
        domain,
    )
    .unwrap_or_else(|e| {
        probe.kill();
        panic!("start the stock publisher peer: {e}")
    });

    let sub_rose = probe
        .wait_for_output_pattern(output::ADV_SUB_MATCHED_ROSE, EDGE_BUDGET)
        .unwrap_or_else(|e| {
            pubber.kill();
            probe.kill();
            panic!(
                "subscription_count_matched_publishers never left 0 while a stock \
                 `ros2 topic pub {SUB_TOPIC}` was running on domain {domain}.\n{e}"
            )
        });
    assert!(
        sub_rose.contains(output::ADV_SUB_MATCHED_ROSE),
        "internal: the wait returned Ok without the marker:\n{sub_rose}"
    );

    // Killed rather than drained: nothing here reads what `ros2 topic pub`
    // printed, and its own 10 s window would only delay the fall edge below.
    pubber.kill();

    let sub_fell = probe
        .wait_for_output_pattern(output::ADV_SUB_MATCHED_FELL, EDGE_BUDGET)
        .unwrap_or_else(|e| {
            probe.kill();
            panic!(
                "subscription_count_matched_publishers never came back to 0 after the peer \
                 left.\n{e}"
            )
        });
    assert!(
        sub_fell.contains(output::ADV_SUB_MATCHED_FELL),
        "internal: the wait returned Ok without the marker:\n{sub_fell}"
    );
    probe.kill();

    // The peer-side half of `get_serialization_format`. A stock
    // `rmw_cyclonedds_cpp` node deserialized our bytes as
    // `std_msgs/msg/String`; if the encoding it expects and the encoding that
    // slot names had diverged, this is where it would show.
    assert!(
        echoed.contains(PAYLOAD),
        "the stock echo peer matched our publisher (the rise above) but never DECODED a \
         sample. `get_serialization_format` reports the encoding a peer's rmw checks — this \
         is the peer's half of that claim.\nechoed:\n{echoed}"
    );
}

// ---------------------------------------------------------------------------
// Job 3b — the GID, as the peer knows it
// ---------------------------------------------------------------------------

/// The gid `get_gid_for_publisher` reports is the one that identifies us to a
/// stock ROS 2 peer.
///
/// Cyclone's slot reads `dds_get_guid` on the writer — 16 bytes, zero-padded
/// into the 24-byte `rmw_gid_t`. A peer's `rmw_cyclonedds_cpp` fills its
/// endpoint gid from the same writer GUID it discovered, so the first 16 bytes
/// must agree. Only those 16 are compared: the padded WIDTH is
/// distro-dependent (24 bytes through Humble, 16 later), and the identity is
/// the GUID, not the padding.
///
/// **Skips rather than asserting when the peer's report carries no `GID:`
/// line.** That is not defensive coding — `ros2::topic_endpoints` keys only on
/// `Node name:` and `Endpoint type:` precisely because the gid line appears on
/// some distros and not others, and a distro that does not print it has not
/// disagreed with us, it has declined to answer. `skip_class!(capability, …)`
/// says which.
#[test]
fn publisher_gid_identifies_us_to_the_peer() {
    interop::assert_test_bound("advertised_state_interop", &COVERED_COORDS);
    let probe_bin = require_probe();
    let domain = nros_tests::unique_ros_domain_id();
    let mut probe = spawn_probe(probe_bin, domain, Duration::from_secs(60));
    let ready = await_ready(&mut probe);

    let ours_hex = marker_value(&ready, output::ADV_GID).to_string();
    let ours: Vec<u8> = (0..ours_hex.len() / 2)
        .map(|i| u8::from_str_radix(&ours_hex[i * 2..i * 2 + 2], 16).expect("hex gid"))
        .collect();
    assert_eq!(
        ours.len(),
        24,
        "rmw_gid_t is 24 bytes; probe said {ours_hex}"
    );
    assert!(
        ours[..16].iter().any(|b| *b != 0),
        "an all-zero gid is the ABI's spelling for `unknown`, not an identifier: {ours_hex}"
    );

    let (report, found) = await_topic_endpoints_cyclonedds(
        DEFAULT_ROS_DISTRO,
        domain,
        PUB_TOPIC,
        &[("PUBLISHER", NODE)],
        DISCOVERY_BUDGET,
    )
    .unwrap_or_else(|e| {
        probe.kill();
        panic!("ros2 topic info failed: {e}")
    });
    let publisher = one_endpoint(&found[0], "PUBLISHER", PUB_TOPIC, &report);
    probe.kill();

    let Some(theirs) = endpoint_gid_bytes(&publisher) else {
        skip_class!(
            capability,
            "`ros2 topic info --verbose` on {DEFAULT_ROS_DISTRO} prints no `GID:` line for a \
             discovered endpoint, so this tooling cannot answer whether our gid is the one \
             that identifies us. The endpoint WAS discovered:\n{publisher}"
        );
    };
    assert!(
        theirs.len() >= 16,
        "the peer's gid is {} bytes; a Cyclone GUID is 16 and the comparison below needs them \
         all.\n{publisher}",
        theirs.len()
    );
    assert_eq!(
        &theirs[..16],
        &ours[..16],
        "get_gid_for_publisher reports a gid the peer does not know us by. Ours: {ours_hex}; \
         the peer's first 16 bytes: {}. Cyclone's slot reads `dds_get_guid` on the writer and \
         the peer fills its endpoint gid from the writer GUID it discovered — these are the \
         same 16 bytes or the gid identifies nobody.\n{publisher}",
        theirs[..16]
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect::<String>()
    );
}
