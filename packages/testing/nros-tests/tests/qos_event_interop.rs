//! phase-433 W6 acceptance — make a QoS STATUS EVENT fire, and report it.
//!
//! Four vtable slots — `publisher_event_init`, `publisher_take_event`,
//! `subscription_event_init`, `subscription_take_event` — are classified
//! `produced` by `just check rmw-slot-producers` and, until this file, had
//! never been exercised against a live peer.
//!
//! Why no self-contained test could have covered them, stated as concretely as
//! `graph_interop.rs` states its own version: **a QoS event's input is another
//! process's state.** Our `rmw_event_type_t` defines five kinds, and in the
//! zenoh shim four of them are our own clock compared against our own
//! timestamps — `check_deadline_and_fire` reads `last_publish_at_ms`,
//! `check_liveliness_lost` reads `last_assert_at_ms`, and `MessageLost` counts
//! gaps in a sequence we have already received. Every one of those fires in a
//! single image with no peer at all, so running them against one proves
//! nothing that a unit test does not.
//!
//! `LivelinessChanged` on the SUBSCRIPTION side is the exception, and it is an
//! interop claim twice over:
//!
//! 1. Its trigger is the set of *remote* publishers holding an `@ros2_lv`
//!    liveliness token that matches this subscription's topic. Nothing we do
//!    to our own entity changes it.
//! 2. The token is written by `rmw_zenoh_cpp` and matched by our wildcard
//!    (`Ros2Liveliness::publisher_keyexpr_wildcard`). The tree's only existing
//!    check of that wildcard — `shim/mod.rs`, `wildcard_matches_any_entity_id`
//!    and friends — matches it against a keyexpr **our own builder produced**.
//!    That is precisely the shape of check that was green throughout issue
//!    0903, where twelve `produced`, mutation-tested, parity-clean graph slots
//!    did not work at all.
//!
//! ## What we could NOT test, and why
//!
//! The obvious QoS event to provoke from a stock peer is incompatible QoS — a
//! RELIABLE requester against a BEST_EFFORT offerer. **Our ABI has no such
//! event kind.** `rmw_event_type_t` is `LIVELINESS_CHANGED`,
//! `REQUESTED_DEADLINE_MISSED`, `MESSAGE_LOST`, `LIVELINESS_LOST`,
//! `OFFERED_DEADLINE_MISSED` — upstream's `RMW_EVENT_OFFERED_QOS_INCOMPATIBLE`
//! and `RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE` have no counterpart, so there is
//! nothing to register a callback for and nothing to assert.
//!
//! ## Cyclone is absent on purpose
//!
//! `native-qos-event-rust-cyclone-r2n-CARVED` records it: cyclonedds leaves
//! both `*_event_init` slots NULL, and the `*_take_event` pair it *does*
//! implement — reading real `dds_get_liveliness_changed_status` counters — has
//! no caller in the tree outside cyclonedds' own `tests/status_events.cpp`.
//! The Rust adapter hardcodes both slots to `None` and `nros-node` exposes no
//! poll API, so a Cyclone application cannot observe a status event through
//! either half of the surface. A live peer changes nothing about that; the
//! missing piece is a runtime poll path. Issue 1164.
//!
//! Interop cell: `native-qos-event-rust-zenoh-r2n` (`interop::CELLS`).

use std::process::Command;

use nros_tests::{
    fixtures, interop,
    output::{
        QOS_EVENT_LIVELINESS_CHANGED, QOS_EVENT_PROBE_NONE, QOS_EVENT_PROBE_READY,
        QOS_EVENT_PROBE_UNSUPPORTED, QOS_EVENT_SUPPORTS,
    },
    ros2::{DEFAULT_ROS_DISTRO, Ros2Process, require_ros2},
};

/// The coordinates `interop::CELLS` declares for `qos_event_interop`.
///
/// One row, not two: the Cyclone shape is a `CarveOut`, and `coords_for` /
/// `assert_test_bound` only compare `Tier::Runtime` cells, so listing it here
/// would make the tripwire red.
const QOS_EVENT_CELLS: [(
    nros_tests::matrix::PlatformId,
    nros_tests::matrix::Lang,
    nros_tests::matrix::Rmw,
    nros_tests::matrix::Workload,
); 1] = [(
    nros_tests::matrix::PlatformId::Linux,
    // Derived from the binary this test actually spawns —
    // `fixtures::build_qos_event_probe()`, whose `examples/fixtures.toml` row is
    // `lang = "rust"`. Phase-433 W3's lesson: `scenario_coord` returned
    // `Lang::Rust` unconditionally while the cases spawned C binaries, so the
    // language axis was inverted and agreed with itself.
    nros_tests::matrix::Lang::Rust,
    nros_tests::matrix::Rmw::Zenoh,
    nros_tests::matrix::Workload::QosEvents,
)];

/// A stock ROS 2 publisher appearing on our topic delivers a
/// `LivelinessChanged` event to a nano-ros application callback.
///
/// The probe's exit code carries the verdict and the markers say WHICH verdict,
/// because the three ways this can end are three different findings:
///
/// * exit 3 / `QOS_EVENT_PROBE_UNSUPPORTED` — the backend refused the kind.
/// * exit 4 / `QOS_EVENT_PROBE_NONE` — it accepted and never fired. This is
///   the one `produced` cannot see, and the reason the assertion below is on a
///   POSITIVE `alive_count_change` rather than merely on the callback running:
///   a poll that finds nothing also reports "no change".
/// * exit 0 / `QOS_EVENT LIVELINESS_CHANGED alive_change=+N` — it worked.
#[test]
fn stock_ros2_publisher_raises_a_liveliness_event() {
    // The coordinate tripwire — a cell added without a test, or a test that
    // drifts off its cell, is a failure rather than silent non-coverage. It
    // needs no fixtures and no peer, so it runs on this host too.
    interop::assert_test_bound("qos_event_interop", &QOS_EVENT_CELLS);

    if !require_ros2() {
        nros_tests::skip!("ROS 2 + rmw_zenoh_cpp not available");
    }
    let router = fixtures::or_skip(fixtures::ZenohRouter::start_unique());
    let locator = router.locator();

    // The talker starts FIRST and the probe's `alive_count` starts at zero, so
    // the first poll that resolves the token is a 0 -> 1 transition. Starting
    // the probe first would be more faithful to "a peer appeared", but it makes
    // the observable depend on whether the peer came up inside the budget,
    // which is a race and not the claim.
    let _talker = Ros2Process::demo_nodes_cpp_talker(&locator, DEFAULT_ROS_DISTRO)
        .expect("start the stock talker");

    let probe = fixtures::build_qos_event_probe().expect("prebuilt qos-event-probe");
    let out = Command::new(probe)
        .env("NROS_LOCATOR", &locator)
        .env("QOS_EVENT_PROBE_TOPIC", "/chatter")
        .env("QOS_EVENT_PROBE_TIMEOUT_MS", "25000")
        .output()
        .expect("run qos-event-probe");
    let output = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );

    // Name the two failure modes before the generic status assertion, so the
    // message says which one happened instead of printing an exit code.
    assert!(
        !output.contains(QOS_EVENT_PROBE_UNSUPPORTED),
        "the zenoh backend must accept a `LivelinessChanged` registration — \
         `supports_event` says it does. Probe said:\n{output}"
    );
    assert!(
        !output.contains(QOS_EVENT_PROBE_NONE),
        "the callback was registered and never fired against a LIVE \
         `rmw_zenoh_cpp` talker holding an `@ros2_lv` token on this topic. \
         This is the finding phase-433 W6 exists to be able to state: the slot \
         is `produced` and inert. Probe said:\n{output}"
    );
    assert!(
        out.status.success(),
        "qos-event-probe must exit 0 once a liveliness event arrives; got {:?}\n{output}",
        out.status.code()
    );

    // The probe reached the point where a verdict is meaningful at all. Without
    // this, a probe that died before registering would satisfy the two
    // "does not contain" assertions above by printing nothing — the classic
    // absorbing-STALE shape (issue 0445).
    assert!(
        output.contains(QOS_EVENT_PROBE_READY),
        "the probe must reach READY (subscription + callback in place) before \
         any verdict here means anything:\n{output}"
    );
    assert!(
        output.contains(QOS_EVENT_SUPPORTS),
        "the probe must report the backend's declared capability:\n{output}"
    );
    assert!(
        output.contains(QOS_EVENT_LIVELINESS_CHANGED),
        "a `LivelinessChanged` event must reach the application callback:\n{output}"
    );

    // The DELTA, not just the line. `alive_change=0` is what a poll that
    // resolves no tokens reports, and it would satisfy the marker above while
    // proving the opposite of what this test claims.
    let positive = output.lines().any(|l| {
        l.contains(QOS_EVENT_LIVELINESS_CHANGED)
            && l.split_whitespace()
                .find_map(|f| f.strip_prefix("alive_change="))
                .and_then(|v| v.parse::<i32>().ok())
                .is_some_and(|d| d > 0)
    });
    assert!(
        positive,
        "the event must report a POSITIVE `alive_count_change` — the stock \
         talker appearing. A zero delta is what a liveliness poll that resolved \
         no tokens also reports:\n{output}"
    );
}
