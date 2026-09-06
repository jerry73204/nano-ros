//! XRCE-DDS ↔ ROS 2 DDS interoperability tests
//!
//! Tests communication between nros XRCE nodes and ROS 2 nodes using
//! the standard micro-ROS architecture:
//!
//! ```text
//! nros XRCE node → XRCE Agent (Fast-DDS) ←DDS multicast→ ROS 2 node (rmw_fastrtps_cpp)
//! ```
//!
//! The XRCE Agent creates DDS participants on behalf of XRCE clients,
//! so ROS 2 nodes using the same DDS domain can discover and communicate
//! with them via standard DDS multicast.
//!
//! **These tests HARD-FAIL.** This note used to say the opposite — that they
//! were "diagnostic/informational" and reported interop status without failing
//! the suite. That stopped being true: every case here ends in an `assert!` on
//! what actually crossed the bus, and the only non-failing exits are explicit
//! `nros_tests::skip!`s for an absent prerequisite (no XRCE Agent, no ROS 2 DDS,
//! a peer that would not start). A stale "does not hard-fail" line invites the
//! next reader to dismiss a red here as noise, which is precisely the
//! false-PASS reading issue 1135 is about.
//!
//! Prerequisites:
//!   just build-xrce-agent        # Build Micro-XRCE-DDS Agent
//!   ROS 2 Humble installed       # /opt/ros/humble/
//!   rmw_fastrtps_cpp available   # Default in Humble
//!   example_interfaces installed # AddTwoInts service + Fibonacci action types

use nros_tests::{
    count_pattern,
    fixtures::{
        DEFAULT_ROS_DISTRO, ManagedProcess, Ros2DdsProcess, XrceAgent, require_ros2_dds,
        require_xrce_agent, xrce_action_client_binary, xrce_action_server_binary,
        xrce_action_server_concurrent_binary, xrce_listener_binary, xrce_service_client_binary,
        xrce_service_server_binary, xrce_talker_binary,
    },
    unique_ros_domain_id,
};
use rstest::rstest;
use std::{path::PathBuf, time::Duration};

// =============================================================================
// Detection Tests
// =============================================================================

// `test_ros2_dds_detection` removed: it read two `is_*_available()` booleans
// and printed them, asserting nothing, so it reported PASS on a host with no
// ROS 2 and no rmw_fastrtps_cpp. Both probes stay load-bearing as the `skip!`
// guards on the real interop tests, where a `false` stops the run. Forbidden
// repo-wide by `check-no-vacuous-tests`.

// =============================================================================
// XRCE → ROS 2 Pub/Sub
// =============================================================================

/// nros XRCE talker → ROS 2 DDS listener
///
/// Architecture:
///   xrce-talker → XRCE Agent (UDP) → DDS multicast → ros2 topic echo (rmw_fastrtps_cpp)
#[rstest]
fn test_xrce_to_ros2_pubsub(xrce_talker_binary: PathBuf) {
    use std::process::Command;

    if !require_xrce_agent() {
        nros_tests::skip!("XRCE agent not available");
    }

    if !require_ros2_dds() {
        nros_tests::skip!("ROS 2 DDS not available");
    }

    // Start XRCE Agent on ephemeral port
    let agent = XrceAgent::start_unique().expect("Failed to start XRCE Agent");
    let addr = agent.addr();
    let domain_id = unique_ros_domain_id();

    // Start ROS 2 DDS listener (uses rmw_fastrtps_cpp, DDS multicast discovery)
    eprintln!("Starting ROS 2 DDS topic echo...");
    let mut ros2_listener = match Ros2DdsProcess::topic_echo_with_domain(
        "/chatter",
        "std_msgs/msg/String",
        DEFAULT_ROS_DISTRO,
        domain_id,
    ) {
        Ok(p) => p,
        Err(e) => {
            nros_tests::skip!(
                "ROS 2 DDS listener could not start (missing ROS 2 demo nodes / tooling?): {}",
                e
            );
        }
    };

    // Wait for DDS discovery + XRCE Agent to propagate
    std::thread::sleep(Duration::from_secs(1));

    // Start XRCE talker
    eprintln!("Starting XRCE talker...");
    let mut talker_cmd = Command::new(&xrce_talker_binary);
    talker_cmd
        .env("NROS_LOCATOR", &addr)
        .env("XRCE_AGENT_ADDR", &addr)
        .env("ROS_DOMAIN_ID", domain_id.to_string());
    let mut talker =
        ManagedProcess::spawn_command(talker_cmd, "xrce-talker").expect("Failed to start talker");

    // Wait for talker to publish
    let _ = talker.wait_for_output_pattern(
        nros_tests::output::TALKER_LOG_PREFIX,
        Duration::from_secs(5),
    );

    // phase-342 W9 — wait for the DELIVERY this test asserts (`data:` lines from
    // the ROS 2 listener) instead of sleeping 2 s for it. W8 could not convert
    // this site: `Ros2Process` had only `wait_for_output(timeout)`, which drains
    // to its deadline and cannot be asked about a pattern. W9 added the shared
    // `collect_until` engine, so the wait is now the condition rather than a
    // guess at how long it takes.
    let ros2_output = ros2_listener
        .wait_for_output_count("data:", 1, Duration::from_secs(20))
        .unwrap_or_else(|e| {
            eprintln!("ROS 2 listener saw no `data:` within 20s: {e}");
            String::new()
        });

    talker.kill();

    eprintln!("ROS 2 DDS output:\n{}", ros2_output);

    let received_count = count_pattern(&ros2_output, "data:");
    eprintln!("ROS 2 received {} messages via DDS", received_count);

    // Drop the agent before asserting so a failure doesn't leak the process.
    drop(agent);

    assert!(
        received_count > 0,
        "ROS 2 DDS listener received no messages from the XRCE talker — the \
         header-stripped XRCE topic payload must parse as DDS CDR (233.6). Output:\n{}",
        ros2_output
    );
    eprintln!("[PASS] XRCE → ROS 2 DDS pub/sub works");
}

// =============================================================================
// ROS 2 → XRCE Pub/Sub
// =============================================================================

/// ROS 2 DDS talker → nros XRCE listener
///
/// Architecture:
///   ros2 topic pub (rmw_fastrtps_cpp) → DDS multicast → XRCE Agent → xrce-listener (UDP)
#[rstest]
fn test_ros2_to_xrce_pubsub(xrce_listener_binary: PathBuf) {
    use std::process::Command;

    if !require_xrce_agent() {
        nros_tests::skip!("XRCE agent not available");
    }

    if !require_ros2_dds() {
        nros_tests::skip!("ROS 2 DDS not available");
    }

    // Start XRCE Agent on ephemeral port
    let agent = XrceAgent::start_unique().expect("Failed to start XRCE Agent");
    let addr = agent.addr();
    let domain_id = unique_ros_domain_id();

    // Start XRCE listener (subscribe before publishing)
    eprintln!("Starting XRCE listener...");
    let mut listener_cmd = Command::new(&xrce_listener_binary);
    listener_cmd
        .env("NROS_LOCATOR", &addr)
        .env("XRCE_AGENT_ADDR", &addr)
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .env("XRCE_MSG_COUNT", "3")
        // nros_tests::output::INT32_LISTENER_LOG_PREFIX logs at INFO — surface them so the test can see receipt.
        .env("RUST_LOG", "info");
    let mut listener = ManagedProcess::spawn_command(listener_cmd, "xrce-listener")
        .expect("Failed to start listener");

    // Wait for listener to be ready
    let _ = listener.wait_for_output_pattern(
        nros_tests::output::LISTENER_READY_MARKER,
        Duration::from_secs(5),
    );

    // Wait for DDS discovery + XRCE Agent to propagate subscription
    std::thread::sleep(Duration::from_secs(1));

    // Start ROS 2 DDS publisher
    eprintln!("Starting ROS 2 DDS topic pub...");
    let mut ros2_publisher = match Ros2DdsProcess::topic_pub_with_domain(
        "/chatter",
        "std_msgs/msg/String",
        "{data: 'Hello World: 42'}",
        2,
        DEFAULT_ROS_DISTRO,
        domain_id,
    ) {
        Ok(p) => p,
        Err(e) => {
            listener.kill();
            nros_tests::skip!(
                "ROS 2 DDS publisher could not start (missing ROS 2 demo nodes / tooling?): {}",
                e
            );
        }
    };

    // Wait for XRCE listener to receive messages
    let listener_output = listener.collect_until(
        nros_tests::output::LISTENER_LOG_PREFIX,
        Duration::from_secs(5),
    );

    ros2_publisher.kill();
    listener.kill();

    eprintln!("XRCE listener output:\n{}", listener_output);

    let received_count = count_pattern(&listener_output, nros_tests::output::LISTENER_LOG_PREFIX);
    eprintln!(
        "XRCE listener received {} messages from ROS 2 DDS",
        received_count
    );

    drop(agent);

    assert!(
        received_count > 0,
        "XRCE listener received no messages from the ROS 2 DDS publisher — the \
         re-prepended CDR header on the inbound XRCE topic must parse (233.6). Output:\n{}",
        listener_output
    );
    eprintln!("[PASS] ROS 2 DDS → XRCE pub/sub works");
}

// =============================================================================
// XRCE Service Server + ROS 2 DDS Client
// =============================================================================

/// nros XRCE service server + ROS 2 DDS service client
///
/// Architecture:
///   ros2 service call (rmw_fastrtps_cpp) → DDS → XRCE Agent → xrce-service-server (UDP)
#[rstest]
fn test_xrce_service_ros2_client(xrce_service_server_binary: PathBuf) {
    use std::process::Command;

    if !require_xrce_agent() {
        nros_tests::skip!("XRCE agent not available");
    }

    if !require_ros2_dds() {
        nros_tests::skip!("ROS 2 DDS not available");
    }

    // Start XRCE Agent on ephemeral port
    let agent = XrceAgent::start_unique().expect("Failed to start XRCE Agent");
    let addr = agent.addr();
    let domain_id = unique_ros_domain_id();

    // issue 0741 — the precondition this test never stated: nobody else may be
    // serving `/add_two_ints` on this domain.
    //
    // A ROS 2 service name is not owned by the node that declares it. If a
    // second server holds it, the client may talk to THAT one, and if that one
    // speaks `rmw_cyclonedds_cpp` the reply is framed differently enough that
    // Fast-DDS refuses it outright — see `cross_rmw_service_framing_note`. That
    // is the entire content of issue 0741, and it read for weeks as an XRCE
    // type-registration defect because the failure was attributed to the only
    // server the test knew about.
    //
    // Checked HERE, before the fixture starts, because that is the one moment
    // the answer is unambiguous: `ros2 service list` cannot say whose endpoint
    // it found (Humble has no `ros2 service info`), so anything present now is
    // foreign by construction.
    //
    // `skip!`, not `assert!`: a poisoned bus cannot answer the question this
    // test asks, and calling that an interop regression is what produced four
    // retracted diagnoses. `None` (probe could not run) proceeds — a probe that
    // cannot see must not invent.
    if nros_tests::ros2::service_present_on_domain(DEFAULT_ROS_DISTRO, domain_id, "/add_two_ints")
        == Some(true)
    {
        drop(agent);
        nros_tests::skip!(
            "issue 0741 — `/add_two_ints` is ALREADY served on ROS domain {domain_id} by a peer \
             this test did not start, so the reply the ROS 2 client receives is not necessarily \
             ours. Kill the orphan (`pgrep -a add_two_ints_server`) and re-run; a Cyclone one \
             makes this test fail with a `RTPS_READER_HISTORY … cannot be resized` that has \
             nothing to do with XRCE."
        );
    }

    // Start XRCE service server
    eprintln!("Starting XRCE service server...");
    let mut server_cmd = Command::new(&xrce_service_server_binary);
    server_cmd
        .env("NROS_LOCATOR", &addr)
        .env("XRCE_AGENT_ADDR", &addr)
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .env("XRCE_TIMEOUT", "30");
    let mut server = ManagedProcess::spawn_command(server_cmd, "xrce-service-server")
        .expect("Failed to start service server");

    // Wait for the server to SAY it is ready, and KEEP what it said.
    //
    // This was `let _ = wait_for_output_pattern(…, 5s)`, and the discard is the
    // defect: a failure downstream could not distinguish "the server never came
    // up" from "the server came up and replied wrong", which are different bugs
    // with different fixes. The sibling test below (`test_ros2_service_xrce_client`)
    // was given exactly this treatment and the reason recorded there; the lesson
    // did not travel the 350 lines to here, so this one kept reporting an
    // interop regression for whatever had actually happened.
    //
    // 20 s, not 5 s, for the same reason the sibling uses 20: the bound exists to
    // catch a server that never starts, not to race one that starts slowly, and a
    // wait that returns as soon as the marker appears costs nothing when it is fast.
    let server_startup = server
        .wait_for_output_pattern(
            nros_tests::output::SERVICE_SERVER_READY_MARKER,
            Duration::from_secs(20),
        )
        .unwrap_or_else(|e| format!("<server never reported ready: {e}>"));

    // Wait for DDS discovery to propagate the service
    std::thread::sleep(Duration::from_secs(1));

    // Call service from ROS 2 DDS
    eprintln!("Calling service from ROS 2 DDS...");
    let mut ros2_client = match Ros2DdsProcess::service_call_with_domain(
        "/add_two_ints",
        "example_interfaces/srv/AddTwoInts",
        "{a: 5, b: 3}",
        DEFAULT_ROS_DISTRO,
        domain_id,
    ) {
        Ok(p) => p,
        Err(e) => {
            server.kill();
            nros_tests::skip!(
                "ROS 2 DDS service call could not start (missing ROS 2 tooling?): {}",
                e
            );
        }
    };

    // Wait for service call to complete
    let ros2_output = ros2_client
        .wait_for_output(Duration::from_secs(5))
        .unwrap_or_default();

    eprintln!("ROS 2 DDS service call output:\n{}", ros2_output);

    // Check if service call succeeded
    let has_sum = ros2_output.contains("sum");
    let has_correct_value = ros2_output.contains("8");

    // issue 0741 — snapshot the bus BEFORE teardown, and only on failure.
    //
    // Three hosts now agree on every static axis, down to the `libfastrtps` /
    // `librmw_fastrtps_cpp` binaries being byte-identical, and still disagree on
    // the verdict. If the software is the same, what differs is who else is on
    // the bus — and the failure's mechanism allows exactly that: the reply
    // reader sizes its history from a max-serialized-size learned AT DISCOVERY,
    // with nothing requiring the endpoint it learned from to be ours.
    //
    // Ordering is the point. `server.kill()` and `drop(agent)` used to run
    // above this; after them the bus is empty and any snapshot is worthless, so
    // the capture moved ahead of teardown rather than into the assert.
    //
    // Only on failure: on the happy path this is three `ros2` invocations of
    // pure cost, and the passing hosts do not need it.
    let bus = if has_sum && has_correct_value {
        String::new()
    } else {
        nros_tests::ros2::dds_bus_snapshot(DEFAULT_ROS_DISTRO, domain_id)
    };

    server.kill();
    drop(agent);

    // Phase 233.6 — this direction (real ROS 2 service client ↔ nano-ros XRCE
    // service server) is the acceptance for the XRCE-DDS service CDR-header
    // interop fix. Hard-assert (unlike the other diagnostic interop tests): a
    // ROS 2 `rmw_fastrtps` client must get the correct `sum=8` reply. Before the
    // header strip/prepend in `service.c` the request deserialized misaligned
    // and the server never replied with a valid value.
    // The server's own startup output is half the evidence: "no reply" from a
    // client whose server never came up is a DIFFERENT failure from "no reply"
    // out of a server that was listening, and only one of them is the interop
    // regression this assert names.
    //
    // issue 0741 — but say WHICH failure this is before naming a regression.
    // The `RTPS_READER_HISTORY … cannot be resized` signature is provably not an
    // XRCE defect (a stock Cyclone server and a stock Fast-DDS client reproduce
    // it with no nano-ros in the picture), and the old message asserted it was
    // one. That sentence is what five sessions of this issue were spent chasing.
    let cross_rmw =
        nros_tests::ros2::cross_rmw_service_framing_note(&ros2_output).unwrap_or_default();
    assert!(
        has_sum && has_correct_value,
        "ROS 2 service client did not get sum=8 from the nano-ros XRCE service \
         server — XRCE-DDS service interop regression (233.6).\n\
         {cross_rmw}\
         --- server startup ---\n{server_startup}\n\
         --- ros2 client output ---\n{ros2_output}\n\
         {bus}{env}",
        env = nros_tests::ros2::interop_environment_fingerprint()
    );
    eprintln!("[PASS] XRCE service server ↔ ROS 2 DDS client: sum=8 verified");
}

// =============================================================================
// Phase 183.6 — XRCE ↔ ROS 2: action (both directions) + reverse-direction
// service. The existing tests cover pub/sub both ways + service
// (xrce-server / ros2-client). These add the missing cells. DDS interop is
// best-effort (naming/version drift), so — like the tests above — they log
// PASS/INFO and only hard-fail on a clear local error, never on a discovery
// miss. nano-XRCE nodes bridge to DDS via the XRCE Agent; ROS 2 uses
// rmw_fastrtps_cpp on the same ROS_DOMAIN_ID.
// =============================================================================

/// nano-XRCE action server ↔ ROS 2 (DDS) action client (`ros2 action send_goal`).
#[rstest]
fn test_xrce_action_ros2_client(xrce_action_server_binary: PathBuf) {
    use std::process::Command;
    if !require_xrce_agent() {
        nros_tests::skip!("XRCE agent not available");
    }
    if !require_ros2_dds() {
        nros_tests::skip!("ROS 2 DDS not available");
    }
    let agent = XrceAgent::start_unique().expect("Failed to start XRCE Agent");
    let addr = agent.addr();
    let domain_id = unique_ros_domain_id();

    let mut server_cmd = Command::new(&xrce_action_server_binary);
    server_cmd
        .env("NROS_LOCATOR", &addr)
        .env("XRCE_AGENT_ADDR", &addr)
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .env("XRCE_TIMEOUT", "30");
    let mut server = ManagedProcess::spawn_command(server_cmd, "xrce-action-server")
        .expect("Failed to start xrce action server");
    let _ = server.wait_for_output_pattern("Action server ready", Duration::from_secs(8));
    std::thread::sleep(Duration::from_secs(1));

    let mut ros2_client = match Ros2DdsProcess::action_send_goal_with_domain(
        "/fibonacci",
        "example_interfaces/action/Fibonacci",
        "{order: 5}",
        DEFAULT_ROS_DISTRO,
        domain_id,
    ) {
        Ok(p) => p,
        Err(e) => {
            server.kill();
            nros_tests::skip!(
                "ROS 2 DDS action client could not start (requires ros-humble-example-interfaces): {e}"
            );
        }
    };
    let ros2_output = ros2_client
        .wait_for_output(Duration::from_secs(20))
        .unwrap_or_default();
    server.kill();
    drop(agent);

    eprintln!("ROS 2 DDS action client output:\n{ros2_output}");
    // A real `ros2 action send_goal --feedback` against the nano-ros XRCE
    // action server accepts the goal, streams feedback, and delivers the final
    // result — exercising the send_goal/get_result services + feedback topic
    // with ROS-2-correct names/types, the fixed `uint8[16]` goal-id framing
    // (233.6), and the deferred get_result reply (Phase 237): rclcpp_action
    // sends get_result right after acceptance and the server holds the reply
    // until the goal terminates.
    let accepted = ros2_output.contains("Goal accepted");
    let got_feedback = ros2_output.contains("- 0");
    let got_result = ros2_output.contains("SUCCEEDED") || ros2_output.contains("Result:");
    assert!(
        accepted && got_feedback && got_result,
        "XRCE action server ↔ ROS 2 DDS client did not complete (233.6 / 237): \
         accepted={accepted} feedback={got_feedback} result={got_result}.\n{ros2_output}"
    );
    eprintln!("[PASS] XRCE action server ↔ ROS 2 DDS client: accept + feedback + result");
}

/// Phase 237 — TWO concurrent ROS 2 action clients against one nano-ros XRCE
/// action server (concurrent mode). Both goals are active at once, so both
/// `rclcpp_action` clients have an early `get_result` deferred SIMULTANEOUSLY —
/// exercising the XRCE seq-keyed reply-token table with multiple in-flight
/// replies (the case a single stored `SampleIdentity` would cross-wire). Each
/// client must get its OWN `SUCCEEDED` result.
#[rstest]
fn test_xrce_action_ros2_concurrent(xrce_action_server_concurrent_binary: PathBuf) {
    use std::process::Command;
    if !require_xrce_agent() {
        nros_tests::skip!("XRCE agent not available");
    }
    if !require_ros2_dds() {
        nros_tests::skip!("ROS 2 DDS not available");
    }
    let agent = XrceAgent::start_unique().expect("Failed to start XRCE Agent");
    let addr = agent.addr();
    let domain_id = unique_ros_domain_id();

    // Concurrent fixture bin: accepts + advances several goals at once,
    // draining get_result every spin so two early get_result requests are
    // held simultaneously (phase-277 W5 — was the example's
    // NROS_ACTION_CONCURRENT mode).
    let mut server_cmd = Command::new(&xrce_action_server_concurrent_binary);
    server_cmd
        .env("NROS_LOCATOR", &addr)
        .env("XRCE_AGENT_ADDR", &addr)
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .env("XRCE_TIMEOUT", "40")
        .env("RUST_LOG", "info");
    let mut server = ManagedProcess::spawn_command(server_cmd, "xrce-action-server-concurrent")
        .expect("Failed to start xrce action server");
    let _ = server.wait_for_output_pattern("Action server ready", Duration::from_secs(8));
    std::thread::sleep(Duration::from_secs(1));

    // Long-running goals (order 20 ≈ 2 s) so both overlap and both early
    // get_results are deferred at the same time (the reply-token table this test
    // guards). A short 0.5 s stagger absorbs the agent-side DDS discovery /
    // acceptance-reply timing between the two clients — the request-inbox ring
    // (Phase 237 follow-up) removes the *drop-on-collision* failure mode (and is
    // covered unstaggered by the Zenoh concurrent e2e + the buffer unit test),
    // but exact sub-millisecond simultaneity over the agent is otherwise flaky.
    let spawn_client = || {
        Ros2DdsProcess::action_send_goal_with_domain(
            "/fibonacci",
            "example_interfaces/action/Fibonacci",
            "{order: 20}",
            DEFAULT_ROS_DISTRO,
            domain_id,
        )
    };
    let mut c1 = match spawn_client() {
        Ok(p) => p,
        Err(e) => {
            server.kill();
            nros_tests::skip!("ROS 2 DDS action client could not start: {e}");
        }
    };
    std::thread::sleep(Duration::from_millis(500));
    let mut c2 = match spawn_client() {
        Ok(p) => p,
        Err(e) => {
            server.kill();
            nros_tests::skip!("ROS 2 DDS action client 2 could not start: {e}");
        }
    };

    let out1 = c1
        .wait_for_output(Duration::from_secs(30))
        .unwrap_or_default();
    let out2 = c2
        .wait_for_output(Duration::from_secs(30))
        .unwrap_or_default();
    server.kill();
    drop(agent);

    eprintln!("client 1 output:\n{out1}\nclient 2 output:\n{out2}");
    let ok1 = out1.contains("SUCCEEDED");
    let ok2 = out2.contains("SUCCEEDED");
    assert!(
        ok1 && ok2,
        "concurrent action: both clients must get their own SUCCEEDED result \
         (237 reply-token table): client1={ok1} client2={ok2}"
    );
    eprintln!("[PASS] two concurrent ROS 2 action clients ↔ nano-ros XRCE server: both SUCCEEDED");
}

/// ROS 2 (DDS) action server ↔ nano-XRCE action client (reverse direction).
#[rstest]
fn test_ros2_action_xrce_client(xrce_action_client_binary: PathBuf) {
    use std::process::Command;
    if !require_xrce_agent() {
        nros_tests::skip!("XRCE agent not available");
    }
    if !require_ros2_dds() {
        nros_tests::skip!("ROS 2 DDS not available");
    }
    let agent = XrceAgent::start_unique().expect("Failed to start XRCE Agent");
    let addr = agent.addr();
    let domain_id = unique_ros_domain_id();

    let mut ros2_server =
        match Ros2DdsProcess::action_server_fibonacci_with_domain(DEFAULT_ROS_DISTRO, domain_id) {
            Ok(p) => p,
            Err(e) => {
                // NOT `action_tutorials_py` — this peer is an inline rclpy script
                // (`Ros2DdsProcess::action_server_fibonacci_with_domain`) whose only
                // import is `example_interfaces.action.Fibonacci`, chosen precisely
                // so the cell does not depend on the tutorials package. Naming the
                // wrong package sends the reader to install something that would
                // not have helped.
                nros_tests::skip!(
                    "ROS 2 DDS fibonacci action server (inline rclpy, needs rclpy + \
                 example_interfaces) could not start: {e}"
                );
            }
        };
    // Issue 0687 — WAIT for the server to say it is ready; do not sleep a
    // constant and hope.
    //
    // This was `sleep(6)`, with a comment recording that 3 s "was too tight
    // under test load" — tuning a race window rather than removing it. The
    // window is real: rclpy must import, create 5 action entities and announce
    // them over DDS before the client's `send_goal` fires, and the nano-ros
    // action client sends its goal ONCE with no wait_for/retry. Miss the match
    // and the goal is simply dropped, so the client then waits out its full 20 s
    // budget for a result that can never arrive — a ~30 s failure whose cause is
    // six seconds earlier and invisible.
    //
    // The script has printed `SERVER READY` with `flush=True` all along, under
    // `python3 -u … 2>&1`; nothing was reading it. Waiting on it costs the ~4 s
    // the server actually needs instead of a fixed 6, and does not expire when a
    // loaded host needs 9.
    ros2_server
        .wait_for_output_count(
            nros_tests::output::ROS2_ACTION_SERVER_READY,
            1,
            Duration::from_secs(30),
        )
        .expect("ROS 2 fibonacci action server never announced readiness");
    // The entities are announced; DDS still has to propagate them to the agent's
    // requesters. This is a genuine settle, not a stand-in for a signal.
    std::thread::sleep(Duration::from_secs(1));

    let mut client_cmd = Command::new(&xrce_action_client_binary);
    client_cmd
        .env("NROS_LOCATOR", &addr)
        .env("XRCE_AGENT_ADDR", &addr)
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .env("XRCE_TIMEOUT", "30")
        // Action result / status logs are INFO-level — surface them.
        .env("RUST_LOG", "info");
    let mut client = ManagedProcess::spawn_command(client_cmd, "xrce-action-client")
        .expect("Failed to start xrce action client");
    // The example streams `Next number in sequence received: [...]` feedback
    // and ends with the terminal `Result received: [...]` line.
    let client_output = client.collect_until(
        nros_tests::output::ACTION_RESULT_PREFIX,
        Duration::from_secs(20),
    );
    client.kill();
    // Drain the ROS 2 server BEFORE killing it. Its script prints `SERVER READY`
    // at startup and `SERVER DONE [...]` after `execute` returns, so this one
    // capture says whether the goal ever reached the server — which is exactly
    // what the client's output cannot tell us (issue 0448: the client logs
    // "Goal accepted by server" on local SEND success, not on acceptance).
    // `wait_for_output` TAKES stdout, so it is called once, here.
    let server_output = ros2_server
        .wait_for_output(Duration::from_secs(2))
        .unwrap_or_default();
    ros2_server.kill();
    drop(agent);

    eprintln!("XRCE action client output:\n{client_output}");
    eprintln!("ROS 2 action server output:\n{server_output}");
    // Goal acceptance proves the send_goal request crossed XRCE→agent→DDS and
    // the reply came back (goal_id framing + per-channel service types). At
    // least one non-empty feedback proves the feedback topic + UUID framing
    // round-trips from a real `rcl_action` server.
    let accepted = client_output.contains("Goal accepted");
    let got_feedback = client_output.contains(&format!(
        "{} [0, 1",
        nros_tests::output::ACTION_FEEDBACK_PREFIX
    ));
    assert!(
        accepted && got_feedback,
        "ROS 2 DDS action server ↔ XRCE action client did not complete (233.6): \
         accepted={accepted} got_feedback={got_feedback}.\n\
         client:\n{client_output}\n\
         server (SERVER DONE present ⇒ the goal reached it and it replied):\n{server_output}"
    );

    // Issue 0448 — the payload assertion. `accepted` and `got_feedback` are both
    // satisfiable without the goal ever reaching the server (the client logs
    // acceptance on local SEND success), so neither proves delivery. The result
    // sequence is a FUNCTION of `goal.order`, and this is the only action test
    // whose peer is a real `rcl_action` server that computes it — the native
    // example servers use their own conventions and one ignores `order`
    // entirely (issue 0453). Order 10 ⇒ `order + 1` = 11 elements; a dropped
    // goal yields the zeroed 12-byte default result and an empty sequence.
    assert!(
        client_output.contains(nros_tests::output::FIBONACCI_ORDER_10_SEQUENCE),
        "XRCE action client did not log the order-10 sequence `{}` — the goal \
         payload did not survive the round-trip (0448: a `Result received:` line \
         proves decode, not delivery).\n\
         client:\n{client_output}\n\
         server:\n{server_output}",
        nros_tests::output::FIBONACCI_ORDER_10_SEQUENCE,
    );
    eprintln!(
        "[PASS] ROS 2 DDS action server ↔ XRCE action client: goal accepted + feedback + result payload"
    );
}

/// ROS 2 (DDS) service server ↔ nano-XRCE service client (reverse direction).
#[rstest]
fn test_ros2_service_xrce_client(xrce_service_client_binary: PathBuf) {
    use std::process::Command;
    if !require_xrce_agent() {
        nros_tests::skip!("XRCE agent not available");
    }
    if !require_ros2_dds() {
        nros_tests::skip!("ROS 2 DDS not available");
    }
    let agent = XrceAgent::start_unique().expect("Failed to start XRCE Agent");
    let addr = agent.addr();
    let domain_id = unique_ros_domain_id();

    let mut ros2_server = match Ros2DdsProcess::add_two_ints_server_with_domain(
        DEFAULT_ROS_DISTRO,
        domain_id,
    ) {
        Ok(p) => p,
        Err(e) => {
            nros_tests::skip!(
                "ROS 2 DDS add_two_ints server could not start (requires ros-humble-example-interfaces): {e}"
            );
        }
    };
    // Wait for the server to SAY it is ready, rather than sleeping a fixed
    // budget and hoping. This used to be `let _ = wait_for_output(5s)`, which
    // could not work for two compounding reasons: it discarded the output (so a
    // failure could not say whether the server ever came up), and the rclpy
    // helper ran under a BUFFERED `python3`, so "Service server ready" was still
    // sitting in Python's stdio buffer when the wait expired. The wait was
    // therefore a 5 s timeout that never observed readiness, and the client was
    // started against a server that might or might not be listening — the race
    // this test kept losing. The helpers now run `python3 -u`.
    let server_startup = ros2_server
        .wait_for_output_count(
            nros_tests::output::ROS2_SERVICE_SERVER_READY,
            1,
            Duration::from_secs(20),
        )
        .unwrap_or_else(|e| format!("<server never reported ready: {e}>"));
    std::thread::sleep(Duration::from_secs(1));

    let mut client_cmd = Command::new(&xrce_service_client_binary);
    client_cmd
        .env("NROS_LOCATOR", &addr)
        .env("XRCE_AGENT_ADDR", &addr)
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .env("XRCE_TIMEOUT", "30")
        // The client logs "Result of add_two_ints: ..." at INFO — surface it
        // so the assert below can see the reply (without this the test can't
        // tell success from failure).
        .env("RUST_LOG", "info");
    let mut client = ManagedProcess::spawn_command(client_cmd, "xrce-service-client")
        .expect("Failed to start xrce service client");
    let client_output = client.collect_until(
        nros_tests::output::SERVICE_RESULT_PREFIX,
        Duration::from_secs(20),
    );
    // The ROS 2 server's own output is half the evidence: "no reply" from a
    // client that never had a server to talk to is a DIFFERENT failure from a
    // client that could not reach a healthy one, and discarding it makes the two
    // read identically (the shape of issue 0670).
    let server_output = format!("{server_startup}\n[domain_id={domain_id} agent={addr}]");
    client.kill();
    ros2_server.kill();
    drop(agent);

    eprintln!("XRCE service client output:\n{client_output}");
    eprintln!("ROS 2 add_two_ints server output:\n{server_output}");
    // Phase 233.6 wave 2 — reverse direction (nano-ros XRCE service client →
    // real ROS 2 service server). Hard assert: the client must get a reply.
    assert!(
        client_output.contains(nros_tests::output::SERVICE_RESULT_PREFIX),
        "nano-ros XRCE service client got no reply from the ROS 2 service server \
         — XRCE-DDS reverse service interop regression.\nclient:\n{client_output}\nROS 2 server:\n{server_output}"
    );
    eprintln!("[PASS] ROS 2 DDS service server ↔ XRCE service client: reply received");
}

// Issue 0352 / phase-324 — bind this test to `interop::CELLS`. The coordinates
// below must equal what the list declares for `xrce_ros2_interop`; adding/retiring an
// interop cell for this test, or drifting a cell's coordinate (issue 0341
// defect 2), turns this RED. Needs no fixtures — runs in tier 1.
#[test]
fn cases_bound_to_interop_cells() {
    #[allow(unused_imports)]
    use nros_tests::matrix::{Lang::*, PlatformId::*, Rmw::*, Workload::*};
    nros_tests::interop::assert_test_bound(
        "xrce_ros2_interop",
        &[(Linux, Rust, Xrce, Pubsub), (Linux, Rust, Xrce, Service)],
    );
}
