//! Service integration tests
//!
//! Tests for ROS 2 service communication between nros nodes.
//!
//! **Bucket (phase-329): KEEP — behavior one-offs, not matrix cells.** The plain
//! rust/zenoh single-request DELIVERY case folded into the native-example
//! req/resp matrix consumer (`native_example_reqresp_e2e.rs`).
//! Uses the AddTwoInts service from example_interfaces.

use nros_tests::{
    count_pattern,
    fixtures::{
        ManagedProcess, ZenohRouter, require_zenohd, service_client_binary, service_server_binary,
        zenohd_unique,
    },
    output::SERVICE_RESULT_PREFIX,
};
use rstest::rstest;
use std::{path::PathBuf, process::Command, time::Duration};

/// The native Rust service client's per-attempt FAILURE marker.
///
/// `examples/native/rust/service-client/src/lib.rs` logs
/// `Service call failed, retrying: {:?}` from the `Err` arm of
/// `call_for_name` — the same wording every Rust group copy uses
/// (`examples/{qemu-arm-nuttx,qemu-arm-freertos,threadx-linux}/rust/
/// service-client`). MEASURED at 1 Hz, the timer period, starting ~1 s after
/// spawn.
///
/// Issue 1026: the two tests below used to grep `"Timed out waiting"`, a
/// string NOTHING in this tree prints for `/add_two_ints` — the only producer
/// of that wording is the unrelated `service-client-callback` example, and it
/// spells it `Timed out waiting for reply to {} + {}`. Both waits therefore
/// always hit their deadline; the timeout test's third disjunct
/// (`!client.is_running()`) then made it true by construction, because the
/// `wait_for_all_output` fallback had just killed the process.
///
/// Promoted to `nros_tests::output` by issue 1044, beside `SERVICE_RESULT_PREFIX`
/// and its C/C++ twin — CLAUDE.md's rule is that a test grep names a constant,
/// never a literal, precisely so a reworded example breaks the build instead of
/// silently turning a wait into a timeout.
use nros_tests::output::SERVICE_NOT_AVAILABLE_MARKER;

/// How many consecutive failures prove the client is RETRYING rather than
/// reporting once and wedging. At the measured 1 Hz this is reached in ~3 s.
const SERVICE_RETRY_EVIDENCE: usize = 3;

// =============================================================================
// (Phase 182.3) `test_service_{server,client}_builds` removed — they only
// asserted the service fixtures compiled, covered by `build-all` + the service
// e2e tests below (which build the same binaries via the shared
// `build_native_service_*` resolvers).
// =============================================================================

// =============================================================================
// Server Startup Tests
// =============================================================================

#[rstest]
fn test_service_server_starts(zenohd_unique: ZenohRouter, service_server_binary: PathBuf) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    let locator = zenohd_unique.locator();

    let mut cmd = Command::new(&service_server_binary);
    cmd.env("NROS_LOCATOR", &locator);
    cmd.env("RUST_LOG", "info");
    let mut server = ManagedProcess::spawn_command(cmd, "native-rs-service-server")
        .expect("Failed to start service server");

    // Wait for server readiness. Marker found → fall through to test
    // success. Phase 214.A.3 — dropped the `eprintln!("[PASS]") + return`
    // verbosity; the harness reports PASS on clean fn return.
    if server
        .wait_for_output_pattern("Waiting for service", Duration::from_secs(5))
        .is_ok()
    {
        return;
    }

    // Marker not printed within 5s. Distinguish: process still alive
    // = readiness unverified → SKIP (CLAUDE.md-banned to claim PASS on
    // an unmet precondition). Process exited → real failure → panic.
    if server.is_running() {
        nros_tests::skip!(
            "native-rs-service-server did not print 'Waiting for service' marker within 5s"
        );
    } else {
        // Collect any output for debugging
        let output = server
            .wait_for_all_output(Duration::from_secs(1))
            .unwrap_or_default();
        eprintln!("[FAIL] native-rs-service-server exited early");
        eprintln!("Output: {}", output);
        panic!("Service server failed to start");
    }
}

// =============================================================================
// Client Startup Tests
// =============================================================================

#[rstest]
fn test_service_client_starts_without_server(
    zenohd_unique: ZenohRouter,
    service_client_binary: PathBuf,
) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    let locator = zenohd_unique.locator();

    let mut cmd = Command::new(&service_client_binary);
    cmd.env("NROS_LOCATOR", &locator);
    cmd.env("RUST_LOG", "info");
    let mut client = ManagedProcess::spawn_command(cmd, "native-rs-service-client")
        .expect("Failed to start service client");

    // Without a server the client must SAY SO and send nothing. That is the
    // whole property here: "no server" is audible, and it is not a request
    // into the void. phase-428 W13 — this binary gates its first call on
    // `service_is_ready`, which the zenoh backend answers from its
    // matched-server set, so the marker is rclcpp's "service not available,
    // waiting again..." rather than a failed-call report (that one is what a
    // backend WITHOUT discovery still prints — `SERVICE_CALL_FAILED_MARKER`).
    //
    // Issue 1026 — this waits on the CONDITION, not on a lifetime. The client
    // is `spin = "forever"` (issue 0274) and never exits, so the old
    // `wait_for_output_pattern(dead pattern) → wait_for_all_output` pair spent
    // 12 s and then KILLED it; worse, the `or_else` threw away the 10 s of
    // output the first wait had collected, so the assertion only ever saw the
    // last 2 s.
    let (output, why) =
        client.collect_until_count(SERVICE_NOT_AVAILABLE_MARKER, 1, Duration::from_secs(15));
    let alive = client.is_running();
    // We own the lifetime; nothing above kills it.
    client.kill();

    eprintln!("Client output (no server):\n{}", output);

    assert!(
        output.contains(SERVICE_NOT_AVAILABLE_MARKER),
        "client without a server must say so (`{SERVICE_NOT_AVAILABLE_MARKER}`); \
         got:\n{output}{}",
        why.unwrap_or_default()
    );
    assert!(
        !output.contains(SERVICE_RESULT_PREFIX),
        "no server is running, so no reply can exist — yet the client printed \
         `{SERVICE_RESULT_PREFIX}`:\n{output}"
    );
    assert!(
        alive,
        "the client must REPORT the failure and keep retrying, not die on it; \
         it had exited by the time the first failure was observed:\n{output}"
    );
}

// =============================================================================
// Request/Response Communication Tests
// =============================================================================

// phase-329 W4 — the single-request DELIVERY test (rust/zenoh) FOLDED into the
// native-example req/resp matrix consumer (`native_example_reqresp_e2e.rs`, the
// `(Linux, Rust, Zenoh, Service)` cell). The tests kept here are one-offs no
// cell covers: server startup, client-without-server, multiple sequential
// calls, client timeout, and one server serving multiple clients.

#[rstest]
fn test_service_multiple_sequential_calls(
    zenohd_unique: ZenohRouter,
    service_server_binary: PathBuf,
    service_client_binary: PathBuf,
) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    let locator = zenohd_unique.locator();

    // Start server
    let mut server_cmd = Command::new(&service_server_binary);
    server_cmd.env("NROS_LOCATOR", &locator);
    server_cmd.env("RUST_LOG", "info");
    let mut server = ManagedProcess::spawn_command(server_cmd, "service-server")
        .expect("Failed to start service server");

    // Wait for server readiness
    if server
        .wait_for_output_pattern("Waiting for service", Duration::from_secs(5))
        .is_err()
        && !server.is_running()
    {
        panic!("Service server failed to start");
    }

    // Run client multiple times to verify repeated calls work
    let mut total_responses = 0;

    for run in 1..=2 {
        eprintln!("--- Client run {} ---", run);

        let mut client_cmd = Command::new(&service_client_binary);
        client_cmd.env("NROS_LOCATOR", &locator);
        client_cmd.env("RUST_LOG", "info");
        let mut client = ManagedProcess::spawn_command(client_cmd, "service-client")
            .expect("Failed to start service client");

        // Wait for the client's single result line (event-driven)
        let output = client
            .wait_for_output_pattern(SERVICE_RESULT_PREFIX, Duration::from_secs(15))
            .or_else(|_| client.wait_for_all_output(Duration::from_secs(2)))
            .unwrap_or_default();

        let responses = count_pattern(&output, SERVICE_RESULT_PREFIX);
        eprintln!("Run {}: {} responses", run, responses);
        total_responses += responses;
    }

    server.kill();

    eprintln!("Total responses across all runs: {}", total_responses);

    // One result per client run.
    if total_responses >= 2 {
        eprintln!("[PASS] Multiple sequential client runs work correctly");
    } else {
        eprintln!(
            "[FAIL] Expected at least 2 total responses, got {}",
            total_responses
        );
        panic!("Multiple sequential calls failed");
    }
}

// =============================================================================
// Timeout Handling Test
// =============================================================================

#[rstest]
fn test_service_client_timeout(zenohd_unique: ZenohRouter, service_client_binary: PathBuf) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    let locator = zenohd_unique.locator();

    // Start client WITHOUT server - should timeout
    let mut client_cmd = Command::new(&service_client_binary);
    client_cmd.env("NROS_LOCATOR", &locator);
    client_cmd.env("RUST_LOG", "info");
    let mut client = ManagedProcess::spawn_command(client_cmd, "service-client-timeout")
        .expect("Failed to start service client");

    // What this test is FOR, restated after issue 1026 and phase-428 W13: a
    // client with no server must keep saying so at the timer cadence — not
    // report once and wedge, not fall silent, and never manufacture a reply.
    // The sibling above proves "no server" is audible at all; this one proves
    // it stays audible. (It was "must time out EVERY attempt": the client now
    // gates the attempt on `service_is_ready`, so with no server there is no
    // attempt to time out, and what recurs is the waiting line.)
    //
    // The old shape could not fail on any build: it greped a string nothing
    // prints, so the `or_else` fallback always ran, `wait_for_all_output`
    // killed the process when ITS window closed, and the assertion's third
    // disjunct `!client.is_running()` was therefore TRUE BY CONSTRUCTION.
    //
    // `collect_until_count` returns as soon as the count is reached and KILLS
    // NOTHING on timeout (unlike `wait_for_output`/`wait_for_all_output`), so
    // the 20 s here is a deadline, not the node's lifetime. MEASURED: the
    // marker arrives at 1 Hz from ~1 s, so the wait returns in ~3 s.
    let (output, why) = client.collect_until_count(
        SERVICE_NOT_AVAILABLE_MARKER,
        SERVICE_RETRY_EVIDENCE,
        Duration::from_secs(20),
    );
    let failures = count_pattern(&output, SERVICE_NOT_AVAILABLE_MARKER);
    let alive = client.is_running();
    client.kill();

    eprintln!("Timeout test output ({failures} failures):\n{output}");

    assert!(
        failures >= SERVICE_RETRY_EVIDENCE,
        "a client with no server must say so on every tick: \
         expected >= {SERVICE_RETRY_EVIDENCE} `{SERVICE_NOT_AVAILABLE_MARKER}` within 20s, \
         saw {failures}:\n{output}{}",
        why.unwrap_or_default()
    );
    assert!(
        !output.contains(SERVICE_RESULT_PREFIX),
        "no server is running, so no call can succeed — yet the client printed \
         `{SERVICE_RESULT_PREFIX}`:\n{output}"
    );
    assert!(
        alive,
        "the client must survive waiting and keep checking; it had \
         exited after {failures} waits:\n{output}"
    );

    // Not observed here (stated per issue 1026's acceptance): whether the
    // client would EVENTUALLY give up, and whether a call succeeds once a
    // server appears. The first is not a contract the client has; the second
    // is `test_service_multiple_sequential_calls` above.
}

// =============================================================================
// Server Handles Multiple Clients Test
// =============================================================================

#[rstest]
fn test_service_server_multiple_clients(
    zenohd_unique: ZenohRouter,
    service_server_binary: PathBuf,
    service_client_binary: PathBuf,
) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    let locator = zenohd_unique.locator();

    // Start server
    let mut server_cmd = Command::new(&service_server_binary);
    server_cmd.env("NROS_LOCATOR", &locator);
    server_cmd.env("RUST_LOG", "info");
    let mut server = ManagedProcess::spawn_command(server_cmd, "service-server")
        .expect("Failed to start service server");

    // Wait for server readiness
    if server
        .wait_for_output_pattern("Waiting for service", Duration::from_secs(5))
        .is_err()
        && !server.is_running()
    {
        panic!("Service server failed to start");
    }

    // Start two clients with staggered starts to avoid zenoh queryable race
    let mut client1_cmd = Command::new(&service_client_binary);
    client1_cmd.env("NROS_LOCATOR", &locator);
    client1_cmd.env("RUST_LOG", "info");
    let mut client1 = ManagedProcess::spawn_command(client1_cmd, "service-client-1")
        .expect("Failed to start client 1");

    // Stagger client 2 start by 2 seconds so both clients don't race
    // for the zenoh queryable registration simultaneously
    std::thread::sleep(Duration::from_secs(2));

    let mut client2_cmd = Command::new(&service_client_binary);
    client2_cmd.env("NROS_LOCATOR", &locator);
    client2_cmd.env("RUST_LOG", "info");
    let mut client2 = ManagedProcess::spawn_command(client2_cmd, "service-client-2")
        .expect("Failed to start client 2");

    // Wait for both clients' single result line (event-driven)
    let output1 = client1
        .wait_for_output_pattern(SERVICE_RESULT_PREFIX, Duration::from_secs(15))
        .or_else(|_| client1.wait_for_all_output(Duration::from_secs(2)))
        .unwrap_or_default();
    let output2 = client2
        .wait_for_output_pattern(SERVICE_RESULT_PREFIX, Duration::from_secs(15))
        .or_else(|_| client2.wait_for_all_output(Duration::from_secs(2)))
        .unwrap_or_default();

    server.kill();

    eprintln!("=== Client 1 output ===\n{}", output1);
    eprintln!("=== Client 2 output ===\n{}", output2);

    let responses1 = count_pattern(&output1, SERVICE_RESULT_PREFIX);
    let responses2 = count_pattern(&output2, SERVICE_RESULT_PREFIX);

    eprintln!(
        "Client 1 responses: {}, Client 2 responses: {}",
        responses1, responses2
    );

    // Both clients should get responses
    if responses1 > 0 && responses2 > 0 {
        eprintln!("[PASS] Server handles multiple concurrent clients");
    } else if responses1 > 0 || responses2 > 0 {
        eprintln!("[PARTIAL] At least one client got responses");
        // This might indicate a concurrency issue
        eprintln!("[INFO] Concurrent client handling may need investigation");
    } else {
        eprintln!("[FAIL] Neither client received responses");
        panic!("Multiple client test failed");
    }
}
