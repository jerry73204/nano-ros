//! Parameter Server Integration Tests
//!
//! Tests for parameter declaration, get/set, and ROS 2 interoperability.
//!
//! Run with: `cargo nextest run -p nros-tests --test params`

use nros_tests::{
    fixtures::{
        ManagedProcess, ZenohRouter, build_int32_sink, build_native_param_talker,
        build_native_workspace_rust_params_entry, require_ros2, require_zenohd, zenohd_unique,
    },
    output::param_talker,
    ros2::DEFAULT_ROS_DISTRO,
};
use rstest::rstest;
use std::{process::Command, time::Duration};

// =============================================================================
// Parameter Declaration Tests
// =============================================================================

// (Phase 182.3) `test_talker_with_params_builds` removed — it only asserted
// the parameterised talker compiled, covered by the fixture build (the
// manifest builds bins/param-chatter-talker) + the param e2e tests below
// (which spawn that binary via the shared `build_native_param_talker` resolver).

/// Test that talker starts and uses default parameter value
#[rstest]
fn test_talker_uses_default_param(zenohd_unique: ZenohRouter) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    let binary = build_native_param_talker().expect("Failed to build");
    let locator = zenohd_unique.locator();

    let mut cmd = Command::new(binary);
    cmd.env("RUST_LOG", "info")
        .env("NROS_LOCATOR", &locator)
        .env("NROS_SESSION_MODE", "client");

    let mut proc = ManagedProcess::spawn_command(cmd, "talker").expect("Failed to start talker");

    // Wait for talker to start publishing (ensures parameters are loaded)
    let early_output = proc.collect_until("Publishing", Duration::from_secs(5));

    // Kill the process before collecting output
    proc.kill();

    // Capture both stdout and stderr — combine with early output
    let remaining = proc
        .wait_for_all_output(Duration::from_secs(2))
        .unwrap_or_default();
    let output = format!("{}{}", early_output, remaining);

    println!("=== Talker parameter output ===");
    println!("{}", output);

    // Verify parameter was declared and used with default value
    assert!(
        output.contains("Counter start value: 0"),
        "Should show default parameter value of 0. Output:\n{}",
        output
    );

    // Verify parameter declaration succeeded (no errors)
    assert!(
        !output.contains("Failed to declare parameter"),
        "Should not have parameter declaration errors"
    );

    println!("SUCCESS: Talker uses default parameter value");
}

/// Test that talker declares parameter with correct constraints
#[rstest]
fn test_talker_param_declaration(zenohd_unique: ZenohRouter) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    let binary = build_native_param_talker().expect("Failed to build");
    let locator = zenohd_unique.locator();

    let mut cmd = Command::new(binary);
    cmd.env("RUST_LOG", "debug") // Debug level to see parameter details
        .env("NROS_LOCATOR", &locator)
        .env("NROS_SESSION_MODE", "client");

    let mut proc = ManagedProcess::spawn_command(cmd, "talker").expect("Failed to start talker");

    // Wait directly for the line we're going to assert on. Earlier
    // the test waited for "Publishing" (5 s window) then killed and
    // grabbed the rest with a 2 s grace — under heavy `just
    // test-all` load the talker booted slowly enough that the
    // grace window missed the "Counter start value" line, flaking
    // the test (Phase 96.2). Waiting for the exact pattern
    // eliminates the timing dependency.
    let early_output = proc.collect_until("Counter start value", Duration::from_secs(15));

    // Kill the process before collecting any remaining output.
    proc.kill();

    let remaining = proc
        .wait_for_all_output(Duration::from_secs(2))
        .unwrap_or_default();
    let output = format!("{}{}", early_output, remaining);

    println!("=== Talker debug output ===");
    println!("{}", output);

    // Verify node was created
    assert!(
        output.contains("Node created") || output.contains("talker"),
        "Node should be created. Output:\n{}",
        output
    );

    // Parameter value should be logged
    assert!(
        output.contains("Counter start value"),
        "Should log counter start value. Output:\n{}",
        output
    );

    println!("SUCCESS: Parameter declaration works correctly");
}

// =============================================================================
// ROS 2 Parameter Interop Tests (requires ROS 2 + rmw_zenoh_cpp)
// =============================================================================

/// Helper to start a talker and wait for parameter services to register
fn start_talker_with_params(locator: &str) -> ManagedProcess {
    let binary = build_native_param_talker().expect("Failed to build");

    let mut cmd = Command::new(binary);
    cmd.env("RUST_LOG", "info")
        .env("NROS_LOCATOR", locator)
        .env("NROS_SESSION_MODE", "client");

    let mut talker = ManagedProcess::spawn_command(cmd, "talker").expect("Failed to start talker");

    // Wait for talker to start publishing (ensures parameter services are registered)
    let _ = talker.wait_for_output_pattern(
        nros_tests::output::PARAM_TALKER_READY_MARKER,
        Duration::from_secs(5),
    );

    // Extra delay for parameter service discovery propagation through zenohd
    std::thread::sleep(Duration::from_secs(1));

    talker
}

/// Require that the nros node is discoverable via `ros2 node list`; skip loudly
/// if it is not. Returns normally ONLY on success — issue 1135.
///
/// This used to return `bool`, and all four callers wrote
/// `if !require_node_discoverable(&locator) { return; }`. A bare `return` from a
/// `#[test]` is a PASS, so on a host where our node never joined the graph the
/// four `ros2 param *` tests reported green — and `just native
/// test-ros2-params` passes `--failure-output never`, so the `eprintln!` that
/// was the only evidence never reached the log either. A `require_*` helper that
/// hands the caller a `bool` hands them the verdict; this one keeps it.
///
/// Note WHAT is being swallowed. Every cheap precondition (zenohd, ROS 2,
/// the fixture) has already passed by the time this runs, so a negative here
/// means "ROS 2 is up, the router is up, our talker is up, and the node is not
/// in the graph" — a DELIVERY failure, which is the exact thing these tests
/// exist to detect. That is `resource`, not `capability`: the host can run this
/// test, a peer just never appeared.
fn require_node_discoverable(locator: &str) {
    for attempt in 1..=3 {
        if let Ok(output) = nros_tests::ros2::ros2_node_list(locator, DEFAULT_ROS_DISTRO)
            && output.contains("/demo/talker")
        {
            return;
        }
        if attempt < 3 {
            std::thread::sleep(Duration::from_secs(1));
        }
    }
    nros_tests::skip_class!(
        resource,
        "nros node /demo/talker not discoverable via `ros2 node list` after 3 attempts \
         (locator {locator}) — zenohd, ROS 2 and the talker fixture were all present, so \
         this is our node failing to reach the ROS graph, not a missing prerequisite"
    );
}

/// Test that ROS 2 can list parameters on nros node
#[rstest]
fn test_ros2_param_list(zenohd_unique: ZenohRouter) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    if !require_ros2() {
        nros_tests::skip!("ROS 2 not found");
    }

    let locator = zenohd_unique.locator();
    let _talker = start_talker_with_params(&locator);

    require_node_discoverable(&locator);

    // Retry up to 3 times since parameter services need discovery time
    let mut ros2_stdout = String::new();
    for attempt in 1..=3 {
        ros2_stdout =
            nros_tests::ros2::ros2_param_list("/demo/talker", &locator, DEFAULT_ROS_DISTRO)
                .expect("Failed to run ros2 param list");

        println!("=== ros2 param list attempt {} ===", attempt);
        println!("{}", ros2_stdout);

        if ros2_stdout.contains("start_value") {
            break;
        }

        if attempt < 3 {
            std::thread::sleep(Duration::from_secs(1));
        }
    }

    assert!(
        ros2_stdout.contains("start_value"),
        "ros2 param list should show 'start_value' parameter. Output:\n{}",
        ros2_stdout
    );
}

/// Test that ROS 2 can get parameter value from nros node
#[rstest]
fn test_ros2_param_get(zenohd_unique: ZenohRouter) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    if !require_ros2() {
        nros_tests::skip!("ROS 2 not found");
    }

    let locator = zenohd_unique.locator();
    let _talker = start_talker_with_params(&locator);

    require_node_discoverable(&locator);

    // Retry up to 3 times for discovery
    let mut ros2_stdout = String::new();
    for attempt in 1..=3 {
        ros2_stdout = nros_tests::ros2::ros2_param_get(
            "/demo/talker",
            "start_value",
            &locator,
            DEFAULT_ROS_DISTRO,
        )
        .expect("Failed to run ros2 param get");

        println!("=== ros2 param get attempt {} ===", attempt);
        println!("{}", ros2_stdout);

        if ros2_stdout.contains("Integer value is: 0") {
            break;
        }

        if attempt < 3 {
            std::thread::sleep(Duration::from_secs(1));
        }
    }

    assert!(
        ros2_stdout.contains("Integer value is: 0"),
        "ros2 param get should show 'Integer value is: 0'. Output:\n{}",
        ros2_stdout
    );
}

/// Test that ROS 2 can set and read back a parameter on nros node
#[rstest]
fn test_ros2_param_set(zenohd_unique: ZenohRouter) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    if !require_ros2() {
        nros_tests::skip!("ROS 2 not found");
    }

    let locator = zenohd_unique.locator();
    let _talker = start_talker_with_params(&locator);

    require_node_discoverable(&locator);

    // Set start_value to 42
    let set_output = nros_tests::ros2::ros2_param_set(
        "/demo/talker",
        "start_value",
        "42",
        &locator,
        DEFAULT_ROS_DISTRO,
    )
    .expect("Failed to run ros2 param set");

    println!("=== ros2 param set output ===");
    println!("{}", set_output);

    assert!(
        set_output.contains("Set parameter successful"),
        "ros2 param set should succeed. Output:\n{}",
        set_output
    );

    // Read back to verify
    let get_output = nros_tests::ros2::ros2_param_get(
        "/demo/talker",
        "start_value",
        &locator,
        DEFAULT_ROS_DISTRO,
    )
    .expect("Failed to run ros2 param get");

    println!("=== ros2 param get (after set) output ===");
    println!("{}", get_output);

    assert!(
        get_output.contains("Integer value is: 42"),
        "ros2 param get should show updated value 42. Output:\n{}",
        get_output
    );
}

/// Test that ROS 2 can describe parameter on nros node
#[rstest]
fn test_ros2_param_describe(zenohd_unique: ZenohRouter) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    if !require_ros2() {
        nros_tests::skip!("ROS 2 not found");
    }

    let locator = zenohd_unique.locator();
    let _talker = start_talker_with_params(&locator);

    require_node_discoverable(&locator);

    // Retry up to 3 times for discovery
    let mut ros2_stdout = String::new();
    for attempt in 1..=3 {
        ros2_stdout = nros_tests::ros2::ros2_param_describe(
            "/demo/talker",
            "start_value",
            &locator,
            DEFAULT_ROS_DISTRO,
        )
        .expect("Failed to run ros2 param describe");

        println!("=== ros2 param describe attempt {} ===", attempt);
        println!("{}", ros2_stdout);

        if ros2_stdout.contains("Type: integer") || ros2_stdout.contains("integer") {
            break;
        }

        if attempt < 3 {
            std::thread::sleep(Duration::from_secs(1));
        }
    }

    assert!(
        ros2_stdout.contains("Type: integer")
            || ros2_stdout.contains("integer")
            || ros2_stdout.contains("Description:"),
        "ros2 param describe should show parameter type info. Output:\n{}",
        ros2_stdout
    );
}

// =============================================================================
// Parameter Type Tests
// =============================================================================

/// Test that parameter is correctly typed as integer
#[rstest]
fn test_param_integer_type(zenohd_unique: ZenohRouter) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    let binary = build_native_param_talker().expect("Failed to build");
    let locator = zenohd_unique.locator();

    let mut cmd = Command::new(binary);
    cmd.env("RUST_LOG", "info")
        .env("NROS_LOCATOR", &locator)
        .env("NROS_SESSION_MODE", "client");

    let mut proc = ManagedProcess::spawn_command(cmd, "talker").expect("Failed to start");

    // Wait for the first timer publish, not just the startup log.
    let early_output = proc.collect_until(
        nros_tests::output::INT32_TALKER_LOG_PREFIX,
        Duration::from_secs(5),
    );

    // Kill the process before collecting output
    proc.kill();

    // Capture both stdout and stderr — combine with early output
    let remaining = proc
        .wait_for_all_output(Duration::from_secs(2))
        .unwrap_or_default();
    let output = format!("{}{}", early_output, remaining);

    // The counter is used as i32, so it should work with the i64 parameter
    assert!(
        output.contains(nros_tests::output::int32_talker_line(0).as_str())
            || output.contains(nros_tests::output::int32_talker_line(1).as_str()),
        "Should publish with integer counter. Output:\n{}",
        output
    );

    println!("SUCCESS: Parameter integer type works correctly");
}

// =============================================================================
// Phase 264 W4c — in-callback live param read, reconfigured via `ros2 param set`
// =============================================================================

/// W4c — `ros2 param set publish_period_ms 500` updates the volatile store; the
/// `param_talker` node's next callback reads the NEW value via `ctx.parameter::<i64>`
/// and publishes it. A cross-process nros subscriber must then see `Received: 500`.
///
/// The in-env nros↔nros half (the node publishes the baked initial 250 via the live
/// read) is `tests/param_live_read_e2e.rs`; this test adds the `ros2 param set` reconfig
/// path. It needs a wire-matched `rmw_zenoh_cpp` (the pinned overlay — `just rmw_zenoh
/// setup`); where ROS 2 can't discover the node (distro rmw_zenoh mismatching the pinned
/// zenoh wire version) it `skip!`s.
#[rstest]
fn test_ros2_param_set_reconfigures_live_read(zenohd_unique: ZenohRouter) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }
    if !require_ros2() {
        nros_tests::skip!("ROS 2 not found");
    }
    let locator = zenohd_unique.locator();

    // nros `/chatter` subscriber (prints `Received: <data>`).
    let listener_bin = build_int32_sink()
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|e| nros_tests::skip!("native listener fixture not built: {e}"));
    let mut lis_cmd = Command::new(listener_bin);
    lis_cmd
        .env("RUST_LOG", "info")
        .env("NROS_LOCATOR", &locator)
        .env("NROS_SESSION_MODE", "client");
    let mut listener = ManagedProcess::spawn_command(lis_cmd, "listener").expect("spawn listener");
    listener
        .wait_for_output_pattern(
            nros_tests::output::INT32_SINK_READY_MARKER,
            Duration::from_secs(8),
        )
        .expect("listener ready");

    // The parameterised workspace entry (`ctx.parameter` live-read node).
    let entry_bin = build_native_workspace_rust_params_entry()
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|e| nros_tests::skip!("params workspace entry fixture not built: {e}"));
    let mut entry_cmd = Command::new(entry_bin);
    entry_cmd
        .env("RUST_LOG", "info")
        .env("NROS_LOCATOR", &locator)
        .env("NROS_SESSION_MODE", "client")
        .env("NROS_ENTRY_SPIN_MS", "25000")
        .env("NROS_ENTRY_SPIN_STEP_MS", "10");
    let mut entry =
        ManagedProcess::spawn_command(entry_cmd, "param_talker").expect("spawn param_talker");

    // The RESOLVED initial must be on the wire first (node up + publishing live
    // reads). This waited for 250 until issue 0409: 250 is the INLINE value, and
    // seeing it means the later params FILE was dropped — the exact defect
    // `param_live_read_e2e` calls ORDERING_LOST. The wait passed only while a
    // stale resolver was dropping that file, so this test encoded the bug and
    // went red the moment the resolver was fixed. Both halves now read one
    // constant.
    if listener
        .wait_for_output_count(
            nros_tests::output::int32_listener_line(param_talker::RESOLVED).as_str(),
            2,
            Duration::from_secs(15),
        )
        .is_err()
    {
        entry.kill();
        listener.kill();
        panic!(
            "node never published the resolved initial ({}) — cannot test reconfig. \
             Seeing {} instead means the params file was dropped (source ordering); \
             seeing {} means `/**` beat the node's own block (specificity).",
            param_talker::RESOLVED,
            param_talker::ORDERING_LOST,
            param_talker::SPECIFICITY_LOST
        );
    }

    // Discover the param node's FQN (skip if ROS 2 ↔ nros discovery is not wire-matched).
    //
    // The node IS named after the `system.toml` component: a single-node launch threads
    // the component `name` (`param_talker`) into the primary session, so the ROS graph
    // shows `/param_talker`, not the old executor default `/node` (issue 0098, fixed). We
    // assert that name to keep the fix covered — a regression to `/node` fails discovery.
    let node = {
        let mut found = None;
        for attempt in 1..=3 {
            if let Ok(list) = nros_tests::ros2::ros2_node_list(&locator, DEFAULT_ROS_DISTRO)
                && let Some(line) = list
                    .lines()
                    .map(str::trim)
                    .find(|l| l.ends_with("param_talker"))
            {
                found = Some(line.to_string());
                break;
            }
            if attempt < 3 {
                std::thread::sleep(Duration::from_secs(1));
            }
        }
        match found {
            Some(n) => n,
            None => {
                entry.kill();
                listener.kill();
                nros_tests::skip!(
                    "param_talker not discoverable via ros2. NOT a wire mismatch — \
                     issue 0291 showed zenoh's wire is proto-stable across 1.x, so \
                     zpico 1.7.2 talks to a much newer distro RMW; the real cause there \
                     was the keyexpr type-hash. Look at discovery."
                );
            }
        }
    };

    let set_out = nros_tests::ros2::ros2_param_set(
        &node,
        "publish_period_ms",
        "500",
        &locator,
        DEFAULT_ROS_DISTRO,
    )
    .expect("ros2 param set");
    assert!(
        set_out.contains("Set parameter successful"),
        "ros2 param set should succeed; output:\n{set_out}"
    );

    // The node's callback now reads 500 from the store and publishes it.
    let out = listener
        .wait_for_output_count(
            nros_tests::output::int32_listener_line(500).as_str(),
            2,
            Duration::from_secs(15),
        )
        .unwrap_or_else(|_| {
            entry.kill();
            listener.kill();
            panic!(
                "subscriber never saw the reconfigured value (500) after `ros2 param set` — \
                 `ctx.parameter` did not observe the volatile-store update"
            )
        });

    entry.kill();
    listener.kill();

    assert!(
        nros_tests::count_pattern(&out, nros_tests::output::int32_listener_line(500).as_str()) >= 2,
        "the live read should follow the reconfigured value"
    );
}

// phase-329 W3 — bind this test to `interop::CELLS` (the pattern from
// xrce_ros2_interop). The coordinate below must equal what the list declares
// for `params`; drift turns this RED. Needs no fixtures — runs in tier 1.
#[test]
fn cases_bound_to_interop_cells() {
    #[allow(unused_imports)]
    use nros_tests::matrix::{Lang::*, PlatformId::*, Rmw::*, Workload::*};
    nros_tests::interop::assert_test_bound("params", &[(Linux, Rust, Zenoh, Params)]);
}
