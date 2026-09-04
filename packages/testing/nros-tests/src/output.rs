//! Shared output validation utilities for integration tests.
//!
//! All nano-ros standalone examples match the official ROS 2 demo wording
//! (phase-277 W4 chatter, W5 service/action):
//! - Talker: `"Publishing: 'Hello World: N'"`
//! - Listener: `"I heard: [Hello World: N]"`
//! - Service server: `"Incoming request"` + `"a: A b: B"`;
//!   client: `"Result of add_two_ints: N"`
//! - Action server: `"Received goal request with order N"`, `"Executing goal"`,
//!   `"Publish feedback"`, `"Goal succeeded"`; client: `"Sending goal"`,
//!   `"Goal accepted by server, waiting for result"`,
//!   `"Next number in sequence received: [...]"`, `"Result received: [...]"`
//!
//! This module provides `parse_*` functions to extract structured data from
//! process output, and `assert_*` convenience functions that panic with
//! diagnostic messages on failure.
//!
//! phase-277 W2.a — [`TALKER_LOG_PREFIX`] / [`LISTENER_LOG_PREFIX`] (plus the
//! [`talker_line`] / [`listener_line`] helpers) are the SINGLE source of truth
//! for the standalone talker/listener chatter wording. Every test that
//! asserts on the plain talker/listener example output (any platform / RMW /
//! language variant of `examples/*/talker` + `examples/*/listener`) should go
//! through these instead of hard-coding the wording, so a future wording flip
//! stays a one-file change. This does NOT apply to nodes with their own
//! wording (workspace feature packages like the QoS/lifecycle demos, bridge
//! forwarders, or purpose-built test bins) — see
//! `packages/testing/nros-tests/tests/*.rs` call sites for the per-test
//! rationale.

/// The talker (publisher) log-line prefix used by the standalone
/// talker/listener chatter examples (`"Publishing:"`, as in the official
/// ROS 2 demo `Publishing: 'Hello World: N'`).
pub const TALKER_LOG_PREFIX: &str = "Publishing:";

/// The listener (subscriber) log-line prefix used by the standalone
/// talker/listener chatter examples (`"I heard:"`, as in the official
/// ROS 2 demo `I heard: [Hello World: N]`).
pub const LISTENER_LOG_PREFIX: &str = "I heard:";

/// Readiness marker: the talker is considered alive once it prints its
/// first chatter line. phase-277 W4 dropped the separate
/// `"Publishing messages"` boot banner, so "talker up" == "it printed its
/// first `Publishing:` line". Kept as a distinct constant so call sites
/// that only need liveness (not a specific N) stay self-documenting.
pub const TALKER_READY_MARKER: &str = TALKER_LOG_PREFIX;

/// Readiness marker: the Rust chatter LISTENER is subscribed and ready.
///
/// `examples/native/rust/listener` prints
/// `Subscriber created for topic: /chatter` once its subscription exists —
/// the line its own source names as the readiness gate.
///
/// Issue 0471: several suites (`qos`, `multi_node`, `safety_e2e`,
/// `nano2nano`) waited for the literal `"Waiting for"` instead, a banner this
/// binary does not print. That wait could never succeed, and NOTHING noticed,
/// because `wait_for_output_pattern` returned `Ok` on timeout as long as the
/// process had printed anything — which a starting listener always has. The
/// literal is exactly what the "use `output::*` constants, never literal
/// strings" rule exists to prevent (phase-277); this constant is the fix, and
/// the strict wait is what made the breakage visible.
pub const LISTENER_READY_MARKER: &str = "Subscriber created for topic:";

/// Readiness marker for the SAFETY chatter listener
/// (`safety_chatter_listener`), which prints
/// `Safety subscriber created for topic: /chatter`.
///
/// Deliberately a separate constant rather than a prefix of
/// [`LISTENER_READY_MARKER`]: the safety binary spells it
/// `Safety subscriber` (lower-case `s`), so the plain listener's marker is
/// NOT a substring of it and matching one against the other would silently
/// never fire — the same failure mode issue 0471 exposed.
pub const SAFETY_LISTENER_READY_MARKER: &str = "Safety subscriber created for topic:";

/// Talker line WITH the opening payload quote — distinguishes a real
/// publish line from setup prose containing "Publishing" (phase-295 W2).
pub const TALKER_PAYLOAD_PREFIX: &str = "Publishing: '";

/// MessageInfo attachment trace (issue 0429). The zenoh publisher shim logs the
/// per-message MessageInfo it stamps into the wire attachment under `RUST_LOG=trace`
/// (`nros-rmw-zenoh/src/shim/publisher.rs`): `… with attachment: seq=N, ts=…,
/// gid=[..]`. This is the authoritative source of the sequence/GID a subscriber
/// then reads — the DEMO listener is slim and no longer traces the receive side, so
/// tests observe the values here. The line marker proves the attachment path fired.
pub const MESSAGE_INFO_ATTACHMENT_MARKER: &str = "with attachment:";
/// The per-message sequence number inside [`MESSAGE_INFO_ATTACHMENT_MARKER`]
/// (`seq=N,`). Monotonic per publisher.
pub const MESSAGE_INFO_SEQ_PREFIX: &str = "seq=";
/// The publisher GID inside [`MESSAGE_INFO_ATTACHMENT_MARKER`] (`gid=[..]`).
/// Constant per publisher.
pub const MESSAGE_INFO_GID_PREFIX: &str = "gid=";

/// Pre-W4 Int32 chatter wording, retained by nodes OUTSIDE the phase-277 W4
/// demo-parity flip: the purpose-built fixture bins
/// (`packages/testing/nros-tests/bins/{param,safety,header}-chatter-*`,
/// `int32-sink`), the workspace demo packages
/// (`examples/workspaces/{rust,c,cpp,mixed,ws-*}`), and the nros-bench
/// stress bins. Tests that assert on THOSE outputs use these constants, so
/// the standalone-example constants above can evolve independently.
pub const INT32_TALKER_LOG_PREFIX: &str = "Published:";

/// See [`INT32_TALKER_LOG_PREFIX`] — the listener/sink side (`"Received:"`).
pub const INT32_LISTENER_LOG_PREFIX: &str = "Received:";

/// phase-370 — the PURE-C workspace talker's publish marker
/// (`examples/workspaces/c/src/talker_pkg/src/Talker.c`).
///
/// A separate constant because the two workspaces do not agree: the C++ talker
/// prints [`INT32_TALKER_LOG_PREFIX`] (`"Published:"`) and the C one prints
/// `"[talker_pkg] sent:"`. The doc on that constant claims it covers "the
/// workspace demo packages", which is true of the listener side (both print
/// `"Received:"`) and NOT of the talker side. Naming the C spelling here is
/// cheaper than reconciling two examples' output, and it keeps the divergence
/// written down instead of rediscovered by the next test that greps the wrong
/// one — which is what this module exists to prevent.
pub const WORKSPACE_C_TALKER_LOG_PREFIX: &str = "[talker_pkg] sent:";

/// issue 0441 — the receive-side `MessageInfo` marker emitted by the
/// `message-info-observer` bin (`seq=<n> gid=<hex> ts=<t>`).
///
/// A constant rather than a literal for the reason this whole module exists:
/// the previous zero-copy assertion grepped `seq=` out of the listener EXAMPLE,
/// and when phase-277 slimmed that example to the two lines a ROS 2 demo prints
/// the test kept looking for a string nothing emitted any more. The observer is
/// now the one producer, and this is the one spelling of what it produces.
pub const MESSAGE_INFO_LOG_PREFIX: &str = "seq=";

/// issues 0459 / 0460 — tell "produced output that LACKS marker X" apart from
/// "produced nothing at all", and say which.
///
/// A narrow assertion at the end of a chain names the last missing thing, so an
/// image that emitted nothing after its boot banner gets reported as, say, a
/// missing EDF marker — and the reader goes looking at the scheduler. Issue
/// 0459 was exactly that: the Zephyr C++ realtime entry produced four lines
/// total and the failure said `expected exactly 1 "nros: EDF deadline set
/// tier=", saw 0`. It is not a scheduling problem; it never reached tier
/// startup. Issue 0460's message went further and blamed a subsystem
/// ("the embedded LAUNCH-entry runtime delivery did not work") from the
/// OBSERVER's silence, without showing the guest's output at all.
///
/// The signal is whether the nano-ros runtime ever spoke. Every runtime line
/// this project emits carries `nros` (`nros: …`, `[nros] …`, or a target of
/// `nros_*` under `env_logger`), so zero such lines means the image did not
/// reach application code and a missing application marker says nothing about
/// that marker.
///
/// Returns `None` when the runtime did speak — then a missing marker really is
/// about the marker, and the caller's own message is the right one.
pub fn runtime_silence_note(log: &str) -> Option<String> {
    let lines: Vec<&str> = log.lines().filter(|l| !l.trim().is_empty()).collect();
    if lines.iter().any(|l| l.contains("nros")) {
        return None;
    }
    let tail = lines
        .iter()
        .rev()
        .take(3)
        .rev()
        .map(|l| format!("    {}", l.trim_end()))
        .collect::<Vec<_>>()
        .join("\n");
    Some(format!(
        "NO RUNTIME OUTPUT: {} non-empty line(s), none from the nano-ros runtime.\n           The image did not reach application code, so a missing marker below is NOT \n           evidence about that marker — look between boot and the first `nros` line \n           (issues 0459, 0460).{}",
        lines.len(),
        if tail.is_empty() {
            String::new()
        } else {
            format!("\n  last line(s) seen:\n{tail}")
        }
    ))
}

/// The exact `int32-sink` / workspace-listener log line for value `n`
/// (`"Received: N"`).
pub fn int32_listener_line(n: impl std::fmt::Display) -> String {
    format!("{INT32_LISTENER_LOG_PREFIX} {n}")
}

/// The parameter values the `features` workspace's `param_talker` resolves to,
/// and the two WRONG values that each name a specific resolution rule.
///
/// `demo_bringup/system.toml` gives the node an inline `publish_period_ms` and a
/// LATER `params_files` entry whose `param_talker:` block sets 120 and whose
/// `/**:` block sets 999. ROS applies parameter sources in list order, and a
/// node's own block beats `/**` within a file, so the resolved value is 120.
///
/// These live here rather than in one test file because two suites assert on
/// them — `param_live_read_e2e` (the nros<->nros half) and `params`
/// (the `ros2 param set` reconfig half) — and they disagreed: the reconfig test
/// waited for 250, which is the value that means the params FILE was dropped.
/// It passed only while a stale resolver was dropping it (issue 0409), so the
/// test encoded the bug and turned RED when the bug was fixed. One constant, one
/// meaning.
pub mod param_talker {
    /// What the node must publish: the params-file value for its own block.
    pub const RESOLVED: i64 = 120;
    /// The inline value. Seeing it means source ORDERING was lost — an inline
    /// value beat a later param file (play_launch issue 0007).
    pub const ORDERING_LOST: i64 = 250;
    /// The `/**` value. Seeing it means within-file SPECIFICITY was lost — the
    /// wildcard block beat the node's own because it is written later.
    pub const SPECIFICITY_LOST: i64 = 999;
}

/// The exact Int32 fixture-talker log line for value `n` (`"Published: N"`).
pub fn int32_talker_line(n: impl std::fmt::Display) -> String {
    format!("{INT32_TALKER_LOG_PREFIX} {n}")
}

/// Readiness marker of the `int32-sink` fixture bin: it prints
/// `"Waiting for Int32 messages on <topic>..."` once its subscription is
/// live (its boot banner also contains `"Listener"`, but this line is the
/// post-subscribe signal every observer spawn should key on).
pub const INT32_SINK_READY_MARKER: &str = "Waiting for Int32";

/// Readiness marker of the `param-chatter-talker` fixture bin: it prints
/// `"Publishing Int32 messages every 1s..."` once its publisher AND its
/// parameter services are registered.
///
/// NOT [`TALKER_READY_MARKER`] (`"Publishing:"`, with the colon): this bin
/// publishes Int32 under the pre-W4 wording, so the chatter talker's marker
/// never matches it. Waiting on this line is what tells a parameter test the
/// services it is about to call actually exist.
pub const PARAM_TALKER_READY_MARKER: &str = "Publishing Int32 messages";

/// FreeRTOS realtime-tier workspace nodes (`ws-realtime-{c,cpp}-mps2`)
/// print `"[<tier>] tick=N"` on the QEMU serial console **only when the
/// tier's publish succeeds** — the marker doubles as a delivery proof for
/// lanes observed via serial output instead of host-side subscribers.
pub fn tier_tick_marker(tier: impl std::fmt::Display) -> String {
    format!("[{tier}] tick=")
}

/// issue 0636 gap 2 — the RUST realtime nodes' per-tier dispatch marker.
///
/// `realtime-rust`'s `ctrl_pkg` / `telem_pkg` print
/// `on_<x>: first publish OK (tier `<name>` is dispatching)` on their FIRST
/// successful publish and nothing per tick after it — a recorded decision
/// (issue 0572), because the 10 ms tier would swamp the serial console the
/// e2e observers read. So the C/C++ [`tier_tick_marker`] has no Rust
/// counterpart, and a serial-console proof for a Rust cell has to anchor here.
///
/// It is the right anchor for what #0636 is about: the defect that issue keeps
/// finding is a tier that is NEVER SCHEDULED, and "this tier dispatched and its
/// publish succeeded" is exactly that property. It does not prove ONGOING
/// publication — `CounterRatio3x` is the proof for that, and it needs host
/// observers the freertos/mps2 lane does not use.
pub fn tier_dispatch_marker(tier: impl std::fmt::Display) -> String {
    format!("(tier `{tier}` is dispatching)")
}

/// The exact talker log line for sequence value `n`
/// (`"Publishing: 'Hello World: N'"`).
pub fn talker_line(n: impl std::fmt::Display) -> String {
    format!("{TALKER_LOG_PREFIX} 'Hello World: {n}'")
}

// ---------------------------------------------------------------------------
// Workspace entry-pkg wording (phase-295 W3.b consolidation).
//
// Markers printed by the `examples/workspaces/*` demo packages and the
// `nros::main!` hosted spin — consumed by the multihost / roundtrip matrix
// files. Single source, same one-file-flip rationale as the demo constants.
// ---------------------------------------------------------------------------

/// `nros::main!` env-gated hosted spin exit marker: the entry prints a
/// `"nros: hosted spin complete …"` line (with its callback counters) once
/// its `NROS_ENTRY_SPIN_MS` budget elapses.
pub const HOSTED_SPIN_COMPLETE_MARKER: &str = "hosted spin complete";

/// Counter key inside the hosted-spin exit line (`"message_callbacks=N"`) —
/// N is how many subscription callbacks fired during the spin.
pub const HOSTED_SPIN_MESSAGE_CALLBACKS_KEY: &str = "message_callbacks=";

/// The trailing "now blocking on input" banner — `"Waiting for messages"` —
/// printed by a subscriber AFTER its subscription exists.
///
/// Shared, not workspace-specific: the C workspace listener component, the
/// native C/C++ listeners, `qos-override-pubsub`'s listener role and
/// `serial-listener` all print it, so it has exactly one spelling here. (It
/// was `LISTENER_WAITING_BANNER` until issue 0480 converted the non-workspace
/// sites and the workspace-only name stopped being true.) The C++ workspace
/// listener prints NO ready marker — its observers settle on a fixed delay.
///
/// Prefer the role's own marker where one exists ([`LISTENER_READY_MARKER`],
/// [`INT32_SINK_READY_MARKER`], …): this banner is printed LAST, so keying on
/// it waits longer than the readiness it stands for. Use it when it is the only
/// line the binary prints, or when a test deliberately wants the last marker.
pub const LISTENER_WAITING_BANNER: &str = "Waiting for messages";

/// Readiness marker of the C/C++ workspace service + action SERVER
/// components (`"server ready"`).
pub const WS_SERVER_READY_MARKER: &str = "server ready";

/// Per-reply prefix of the C/C++ workspace service CLIENT component
/// (`"sum: N"` for each server-computed AddTwoInts reply).
pub const WS_SERVICE_CLIENT_SUM_PREFIX: &str = "sum:";

/// The exact C/C++ workspace service-client reply line for `sum`
/// (`"sum: N"`).
pub fn ws_service_client_sum_line(sum: impl std::fmt::Display) -> String {
    format!("{WS_SERVICE_CLIENT_SUM_PREFIX} {sum}")
}

/// The C/C++ workspace action CLIENT result line for the last sequence
/// element (`"result last=N"` — 55 for a Fibonacci goal of order 10).
pub fn ws_action_result_last_line(n: impl std::fmt::Display) -> String {
    format!("result last={n}")
}

/// Per-publish prefix of the ws-custom-msg workspace talker components
/// (`"sent seq=N"` — C/C++/mixed `reading_talker_pkg`).
pub const WS_CUSTOM_MSG_SENT_PREFIX: &str = "sent seq=";

/// Per-receive prefix of the ws-custom-msg workspace listener components
/// (`"reading seq=N …"` — the decoded `sequence` field of
/// `custom_msgs/Reading`).
pub const WS_CUSTOM_MSG_READING_PREFIX: &str = "reading seq=";

/// Decoded second field of the ws-custom-msg listener line (`"temp="`) —
/// proves the full CDR layout, not just a counter, survives the trip.
pub const WS_CUSTOM_MSG_TEMP_FIELD: &str = "temp=";

/// The Rust workspace `talker_pkg`'s per-tick `log_info!` line marker
/// (`"talker publishing chatter seq=N"` — phase-263 A5 logging demo).
pub const WS_RUST_LOGGING_MARKER: &str = "talker publishing chatter";

/// Issue 0469 — the per-tick line the phase-209 port template publishes. The
/// template vendors the upstream ROS 2 tutorial's `minimal_publisher.cpp`
/// VERBATIM, so this string is the tutorial's, not ours: changing the template
/// to make a test pass would defeat the point of the fixture (a stock rclcpp
/// node building and running against nano-ros unmodified).
pub const CPP_PORT_PUBLISH_MARKER: &str = "Publishing: 'Hello, world!";

/// The C workspace talker's per-tick `NROS_LOG_INFO` line marker
/// (`"c_talker logging seq=N"`); the MIXED workspace reuses the C talker,
/// so its logging cell greps the same marker.
pub const WS_C_LOGGING_MARKER: &str = "c_talker logging";

/// The C++ workspace talker's per-tick `NROS_LOG_INFO` line marker
/// (`"cpp_talker logging seq=N"`).
pub const WS_CPP_LOGGING_MARKER: &str = "cpp_talker logging";

// ---------------------------------------------------------------------------
// Service (AddTwoInts) demo wording — phase-277 W5.
//
// Single source of truth for the standalone service-server / service-client
// example wording (any platform / RMW / language variant), matching the
// official `demo_nodes_cpp` `add_two_ints` demo. Same one-file-flip rationale
// as the chatter constants above. Workspace feature packages and purpose-built
// test bins keep their own wording.
// ---------------------------------------------------------------------------

/// First line the service server logs per request (`"Incoming request"`).
pub const SERVICE_INCOMING_REQUEST_MARKER: &str = "Incoming request";

/// Second line the service server logs per request (`"a: A b: B"`).
pub fn service_request_line(a: impl std::fmt::Display, b: impl std::fmt::Display) -> String {
    format!("a: {a} b: {b}")
}

/// Readiness marker the rclpy `add_two_ints` helper logs once its service is
/// advertised (`ros2.rs`'s `add_two_ints_server_with_domain`). A test that
/// starts a client against this server waits on THIS rather than sleeping a
/// budget: the helper runs `python3 -u` precisely so the line arrives while the
/// waiter is still looking.
pub const ROS2_SERVICE_SERVER_READY: &str = "Service server ready";

/// Readiness marker printed on STDOUT by the ROS 2 `fibonacci` action server the
/// interop suite spawns (`ros2.rs`), and its terminal marker.
///
/// Issue 0687 — the action test slept a fixed 6 s instead of waiting for this,
/// even though the script has always printed it with `flush=True`. The nano-ros
/// action client sends its goal ONCE with no retry, so when rclpy's import + 5
/// entity creations + DDS discovery ran past the sleep, the goal was dropped and
/// the test waited out its full 20 s budget for a result that could never come.
pub const ROS2_ACTION_SERVER_READY: &str = "SERVER READY";

/// Prefix of the service client's single result line
/// (`"Result of add_two_ints:"`, as in the official demo
/// `Result of add_two_ints: 5`).
pub const SERVICE_RESULT_PREFIX: &str = "Result of add_two_ints:";

/// What a service client prints when a call fails and it will retry
/// (issue 1044 — moved here from a file-local `const` in `tests/services.rs`).
///
/// There are TWO of these because the example groups word it differently, and a
/// test that greps the wrong one waits out its whole budget against a client
/// that is reporting the failure perfectly well. This is the RUST wording,
/// shared verbatim by the `qemu-arm-nuttx`, `qemu-arm-freertos` and
/// `threadx-linux` copies.
pub const SERVICE_CALL_FAILED_MARKER: &str = "Service call failed, retrying:";

/// The C and C++ groups' wording of [`SERVICE_CALL_FAILED_MARKER`]
/// (`"Service call failed with error %d"`, so only the prefix is stable).
///
/// Verified across all six C/C++ service-client copies; no test greps it yet,
/// and it is here so the next one does not invent a third spelling.
pub const SERVICE_CALL_FAILED_MARKER_C: &str = "Service call failed with error";

/// The exact service client result line for `sum`
/// (`"Result of add_two_ints: N"`).
pub fn service_result_line(sum: impl std::fmt::Display) -> String {
    format!("{SERVICE_RESULT_PREFIX} {sum}")
}

/// Marker the zenoh backend logs when `NROS_SESSION_MODE=peer` is requested of
/// a shim compiled without multicast transport / scouting (issue 0682).
///
/// The test greps this rather than guessing from a timeout: a build that never
/// had peer mode and a peer mode that regressed used to look identical.
pub const ZENOH_PEER_MODE_UNSUPPORTED_MARKER: &str = "peer mode unsupported:";

/// Readiness marker: the service server prints a line containing this once
/// its service is up (`"Waiting for service requests"`).
pub const SERVICE_SERVER_READY_MARKER: &str = "Waiting for service requests";

/// issue 0697 — the marker an exhausted zenoh session pool prints.
///
/// Mirrors `nros_rmw_zenoh::zpico::SESSION_POOL_EXHAUSTED_MARKER`, which is the
/// definition. Kept as a constant here rather than a literal at the grep site
/// because a slimmed banner has broken ~10 tests before; kept as a MIRROR rather
/// than a re-export because `nros-tests` reaches this module without the zenoh
/// backend in every feature set. `pool_exhaustion_marker_matches_the_backend`
/// (in the consuming test) asserts the two agree wherever both are present.
pub const ZENOH_SESSION_POOL_EXHAUSTED: &str = "zenoh session pool exhausted";

/// The verdict line the issue-0697 fixture prints when BOTH halves hold.
pub const POOL_EXHAUSTION_VERDICT: &str = "second session refused with Full";

// ---------------------------------------------------------------------------
// Action (Fibonacci) demo wording — phase-277 W5.
//
// Single source of truth for the standalone action-server / action-client
// example wording, matching the official `action_tutorials` fibonacci demo
// (feedback/result sequences printed rclpy-style: `[0, 1, 1, 2, ...]`).
// ---------------------------------------------------------------------------

/// Action server log prefix when a goal request arrives
/// (`"Received goal request with order"`, followed by the order).
pub const ACTION_GOAL_REQUEST_PREFIX: &str = "Received goal request with order";

/// issue 0461 — the order every `action-client` example sends.
///
/// Named so a test can assert the value ROUND-TRIPS rather than merely that a
/// goal arrived. Nothing checked this, which is how a server that read `1`
/// (Rust), `256` (C/C++) or `0` for every goal passed every action e2e for
/// months: the assertions covered delivery markers, and the one consumer of the
/// decoded order was a log line.
pub const ACTION_GOAL_ORDER: i32 = 10;

/// Parse the order out of a server's `Received goal request with order N` line.
///
/// `None` when the line is absent or unparseable — the caller decides whether
/// that is a failure, since some cells legitimately never receive a goal.
pub fn goal_order_in(log: &str) -> Option<i32> {
    log.lines()
        .find_map(|l| l.split_once(ACTION_GOAL_REQUEST_PREFIX))
        .and_then(|(_, rest)| rest.split_whitespace().next())
        .and_then(|tok| tok.trim_end_matches(['.', ',']).parse().ok())
}

/// Issue 0454 / phase-354 W3 — the raw-goal wire probe's readiness line.
pub const RAW_GOAL_PROBE_READY_MARKER: &str = "raw-goal probe ready";

/// The probe's success line: it saw a result consistent with the order it sent.
pub const RAW_GOAL_SINGLE_HEADER_MARKER: &str = "raw goal shipped exactly one encapsulation header";

/// The order the raw-goal probe sends.
///
/// Deliberately NOT [`ACTION_GOAL_ORDER`]: the examples' 10 would also be a
/// plausible misparse of some other layout, whereas 7 is not reachable from the
/// double-header shift (which yields 65536). Must match `GOAL_ORDER` in
/// `packages/testing/nros-tests/bins/action-raw-goal-probe/src/main.c` — a
/// mismatch fails the test rather than weakening it.
pub const RAW_GOAL_PROBE_ORDER: i32 = 7;

/// What the server actually reads as the order when the goal carries TWO
/// encapsulation headers: **256**, MEASURED by reintroducing the defect and
/// reading the server's own log, not derived on paper.
///
/// The obvious arithmetic — "the header bytes `00 01 00 00` land in `order`, so
/// a little-endian peer reads 0x00010000 = 65536" — is wrong, because the
/// request the server parses is `[encap][GoalId(16)][order]`: the extra four
/// bytes shift the whole tail, so what lands in `order` is a straddle reading
/// 0x00000100. Issue 0461 recorded the same 256 from the other direction.
///
/// Named so the test can assert the regression's exact signature is absent,
/// not merely that some order arrived.
pub const RAW_GOAL_DOUBLE_HEADER_ORDER: i32 = 256;

/// The WCET bench's refusal when the cycle counter is dead (issue 0403).
///
/// `packages/testing/nros-bench/wcet-cycles-qemu` measures with the Cortex-M
/// DWT cycle counter. QEMU does not implement DWT counting, so the counter
/// never advances, and issue 0403 decided the right response is to emit NO
/// measurements and fail rather than report zeros: a zero is indistinguishable
/// from "this operation is free", which is the most optimistic WCET there is
/// and always errs toward "schedulable".
///
/// That makes an emulator an unmet CAPABILITY for this benchmark, not a
/// regression — which is what the consuming test keys on.
pub const WCET_DEAD_COUNTER_MARKER: &str = "the DWT cycle counter is not counting";

/// The WCET bench's completion line, printed only after real measurements.
pub const WCET_COMPLETE_MARKER: &str = "Benchmark complete";

/// Action server log line when goal execution starts (`"Executing goal"`).
pub const ACTION_EXECUTING_MARKER: &str = "Executing goal";

/// Action server log line on each feedback publish (`"Publish feedback"`).
pub const ACTION_PUBLISH_FEEDBACK_MARKER: &str = "Publish feedback";

/// Action server log line when the goal succeeds (`"Goal succeeded"`).
pub const ACTION_GOAL_SUCCEEDED_MARKER: &str = "Goal succeeded";

/// Readiness marker: the action server prints a line containing this once
/// its action is up (`"Waiting for action goals"`).
pub const ACTION_SERVER_READY_MARKER: &str = "Waiting for action goals";

/// Action client log line before sending the goal (`"Sending goal"`).
pub const ACTION_SENDING_GOAL_MARKER: &str = "Sending goal";

/// Multi-goal stress fixture (`bins/action-client-multigoal`, issue 0322) —
/// the one summary line it prints after sending every goal.
///
/// The whole regression lives in this line's numbers: with a server whose
/// `active_goals` table holds `MAX_GOALS` (4), a 6-goal run must report
/// `accepted=4 rejected=2`. Before the fix it reported `accepted=6 rejected=0`
/// — the overflow goals were acknowledged and then dropped.
pub const MULTIGOAL_SUMMARY_PREFIX: &str = "multigoal: summary accepted=";

/// The exact summary line for a completed multi-goal run.
pub fn multigoal_summary_line(accepted: usize, rejected: usize, total: usize) -> String {
    format!("{MULTIGOAL_SUMMARY_PREFIX}{accepted} rejected={rejected} of {total}")
}

/// Action client log PREFIX once the server accepts the goal
/// (`"Goal accepted by server"`). The stock demo continues
/// `", waiting for result"` — see [`ACTION_GOAL_ACCEPTED_MARKER`] for the full
/// line. The prefix exists separately because `output_marker_gate` scans for
/// the shortest form a test might spell; matching only the full line would let
/// a bare `"Goal accepted by server"` literal through.
pub const ACTION_GOAL_ACCEPTED_PREFIX: &str = "Goal accepted by server";

/// Action client log line once the server accepts the goal
/// (`"Goal accepted by server, waiting for result"`) —
/// [`ACTION_GOAL_ACCEPTED_PREFIX`] plus `", waiting for result"`.
pub const ACTION_GOAL_ACCEPTED_MARKER: &str = "Goal accepted by server, waiting for result";

/// Action client log prefix when the server RECEIVED the goal and DECLINED it
/// (`"Goal was rejected by server"`).
///
/// Issue 0868 — this line used to be printed for every non-OK `send_goal`
/// return, so a test asserting it proved only "something failed": a timeout,
/// a failed send and a real rejection all produced it. `send_goal` now
/// returns `Rejected` only for a server decision, and the examples branch on
/// it, so this marker means what it says — and
/// [`ACTION_GOAL_NO_RESPONSE_PREFIX`] is what a timeout prints instead.
pub const ACTION_GOAL_REJECTED_PREFIX: &str = "Goal was rejected by server";

/// Action client log prefix when no goal response arrived within the
/// `send_goal` budget (`"No goal response from server"`).
///
/// Issue 0868 — the case that used to masquerade as a server rejection. The
/// server may never have received the goal at all, so this names the missing
/// response rather than a decision nobody made.
pub const ACTION_GOAL_NO_RESPONSE_PREFIX: &str = "No goal response from server";

/// Action client log prefix for each feedback sample
/// (`"Next number in sequence received:"`, followed by the partial sequence
/// like `[0, 1, 1, 2]`).
pub const ACTION_FEEDBACK_PREFIX: &str = "Next number in sequence received:";

/// Action client terminal log prefix for the result
/// (`"Result received:"`, followed by the full sequence).
pub const ACTION_RESULT_PREFIX: &str = "Result received:";

/// The full-sequence suffix for a Fibonacci goal of order 10, as the action
/// client prints it (`"[0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]"`).
pub const FIBONACCI_ORDER_10_SEQUENCE: &str = "[0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]";

/// The exact listener log line for value `n`
/// (`"I heard: [Hello World: N]"`).
pub fn listener_line(n: impl std::fmt::Display) -> String {
    format!("{LISTENER_LOG_PREFIX} [Hello World: {n}]")
}

/// Extract the sequence number from a chatter payload, i.e. the `N` out of
/// `'Hello World: N'` (talker) or `[Hello World: N]` (listener). Returns
/// `None` when the payload doesn't have the official demo shape.
fn parse_hello_world_n(rest: &str) -> Option<i64> {
    let inner = rest
        .strip_prefix('\'')
        .and_then(|s| s.strip_suffix('\''))
        .or_else(|| rest.strip_prefix('[').and_then(|s| s.strip_suffix(']')))
        .unwrap_or(rest);
    inner
        .trim()
        .strip_prefix("Hello World:")?
        .trim()
        .parse()
        .ok()
}

/// Parsed talker (publisher) output.
#[derive(Debug)]
pub struct TalkerOutput {
    /// Number of [`TALKER_LOG_PREFIX`] lines found.
    pub published_count: usize,
    /// Sequence numbers extracted from `"Publishing: 'Hello World: N'"` lines.
    pub values: Vec<i64>,
}

/// Parsed listener (subscriber) output.
#[derive(Debug)]
pub struct ListenerOutput {
    /// Number of [`LISTENER_LOG_PREFIX`] lines found.
    pub received_count: usize,
    /// Sequence numbers extracted from `"I heard: [Hello World: N]"` lines.
    pub values: Vec<i64>,
}

/// Parsed action client output.
#[derive(Debug)]
pub struct ActionClientOutput {
    /// Whether the goal was accepted.
    pub goal_accepted: bool,
    /// Number of [`ACTION_FEEDBACK_PREFIX`] lines.
    pub feedback_count: usize,
    /// Whether the action completed.
    pub completed: bool,
}

/// Parse talker output, extracting `"Publishing: 'Hello World: N'"` lines.
pub fn parse_talker(output: &str) -> TalkerOutput {
    let mut values = Vec::new();
    let mut count = 0;
    for line in output.lines() {
        if let Some(rest) = extract_after(line, TALKER_LOG_PREFIX) {
            count += 1;
            if let Some(v) = parse_hello_world_n(rest) {
                values.push(v);
            }
        }
    }
    TalkerOutput {
        published_count: count,
        values,
    }
}

/// Parse listener output, extracting `"I heard: [Hello World: N]"` lines.
pub fn parse_listener(output: &str) -> ListenerOutput {
    let mut values = Vec::new();
    let mut count = 0;
    for line in output.lines() {
        if let Some(rest) = extract_after(line, LISTENER_LOG_PREFIX) {
            count += 1;
            if let Some(v) = parse_hello_world_n(rest) {
                values.push(v);
            }
        }
    }
    ListenerOutput {
        received_count: count,
        values,
    }
}

/// Parse action client output (the W5 fibonacci demo wording).
pub fn parse_action_client(output: &str) -> ActionClientOutput {
    // `"Goal accepted"` also matches the official rclpy/rclcpp client's
    // `"Goal accepted by server, waiting for result"` line.
    let goal_accepted = output.contains("Goal accepted");
    let feedback_count = output.matches(ACTION_FEEDBACK_PREFIX).count();
    let completed = output.contains(ACTION_RESULT_PREFIX);
    ActionClientOutput {
        goal_accepted,
        feedback_count,
        completed,
    }
}

/// Assert that the talker published at least `min_count` messages.
///
/// Panics with diagnostic output on failure.
pub fn assert_talker(output: &str, min_count: usize) -> TalkerOutput {
    let result = parse_talker(output);
    assert!(
        result.published_count >= min_count,
        "Talker: expected at least {} published messages, got {}.\nOutput:\n{}",
        min_count,
        result.published_count,
        output,
    );
    result
}

/// Assert that the listener received at least `min_count` messages.
///
/// Panics with diagnostic output on failure.
pub fn assert_listener(output: &str, min_count: usize) -> ListenerOutput {
    let result = parse_listener(output);
    assert!(
        result.received_count >= min_count,
        "Listener: expected at least {} received messages, got {}.\nOutput:\n{}",
        min_count,
        result.received_count,
        output,
    );
    result
}

/// Assert that the action client accepted a goal, received feedback, and completed.
///
/// Panics with diagnostic output on failure.
pub fn assert_action_client(output: &str) -> ActionClientOutput {
    let result = parse_action_client(output);
    assert!(
        result.goal_accepted && result.feedback_count > 0 && result.completed,
        "Action client: goal_accepted={}, feedback_count={}, completed={}.\nOutput:\n{}",
        result.goal_accepted,
        result.feedback_count,
        result.completed,
        output,
    );
    result
}

/// Assert that the values are monotonically non-decreasing.
pub fn assert_monotonic(values: &[i64]) {
    if values.len() < 2 {
        return;
    }
    for window in values.windows(2) {
        assert!(
            window[0] <= window[1],
            "Values are not monotonically increasing: {} > {} in {:?}",
            window[0],
            window[1],
            values,
        );
    }
}

// ---------------------------------------------------------------------------
// RFC-0052 / phase-296 W3b.4/.5 — contract-monitor parity fixture markers.
// The rule ids are the play_launch runtime-enforcement vocabulary (RFC-0050),
// so the same contract yields the same rule string on either runtime.
// ---------------------------------------------------------------------------

/// `contract-monitor-diagsink` per-status prefix (`"DIAG rule=<id> hw=<ep>"`).
pub const CONTRACT_MONITOR_DIAG_PREFIX: &str = "DIAG rule=";

/// Readiness marker of the `contract-monitor-diagsink` observer (its banner
/// contains "Listener", like the other sink fixtures).
pub const CONTRACT_MONITOR_DIAGSINK_READY_MARKER: &str = "diagsink Listener";

/// Publisher-rate-contract violation rule id (`min_rate_hz` guarantee).
pub const RULE_RATE_HIERARCHY_RUNTIME: &str = "rate-hierarchy-runtime";

/// Subscriber max-data-age violation rule id (`max_age_ms` assumption).
pub const RULE_MAX_AGE_RUNTIME: &str = "max-age-runtime";

/// Emitted by `nros-board-zephyr`'s `run_tiers` when a real-time tier's
/// kernel EDF deadline is applied (phase-296 W5.5). MIRRORS the literal
/// `::log::info!` prefix in `nros-board-zephyr/src/entry_tiers.rs`
/// (`apply_tier_deadline`) AND the C/C++ arm's `zephyr_apply_tier_deadline`
/// in `nros-board-zephyr/c/zephyr_run_tiers.c` — keep all three in lockstep
/// (the no_std board crate cannot depend on this crate).
pub const ZEPHYR_EDF_DEADLINE_MARKER: &str = "nros: EDF deadline set tier=";

/// Emitted by the NuttX board seam (`nuttx_run_tiers.c`,
/// `nros_nuttx_apply_current_sporadic` — shared by the C/C++ AND Rust tier
/// arms) when the kernel actually accepted SCHED_SPORADIC for a tier
/// (phase-296 W5.9). MIRRORS the printf literal there — keep in lockstep.
pub const NUTTX_SPORADIC_MARKER: &str = "nros: sporadic budget set tier=";

/// The honest-fallback sibling: printed when a tier DECLARED a sporadic
/// budget but the running kernel lacks `CONFIG_SCHED_SPORADIC` (the
/// executor's cooperative Sporadic SchedContext stays the enforcement).
/// MIRRORS the printf literal in `nuttx_run_tiers.c` — keep in lockstep.
pub const NUTTX_SPORADIC_FALLBACK_MARKER: &str = "nros: sporadic budget declared for tier=";

/// Emitted by the ThreadX board (`nros-board-threadx/src/entry.rs`, both the
/// boot reprioritize and the spawn path) when the kernel ACCEPTED a tier's
/// preemption threshold (phase-296 W5.10, the non_preempt_scope dim).
/// MIRRORS the `B::println` literal there — keep in lockstep.
pub const THREADX_PREEMPT_MARKER: &str = "nros: preempt threshold set tier=";

/// Emitted by the Zephyr board when the kernel ACCEPTED a tier's CPU pin
/// (phase-296 W5.11, the `placement` dim). MIRRORS the `::log::info!` prefix
/// in `nros-board-zephyr/src/entry_tiers.rs` (`apply_tier_core_pin`) AND the
/// `printk` literal in the C/C++ arm's `zephyr_apply_core_pin`
/// (`nros-board-zephyr/c/zephyr_run_tiers.c`) — keep all three in lockstep
/// (the no_std board crate cannot depend on this crate).
pub const ZEPHYR_CORE_PIN_MARKER: &str = "nros: core pin tier=";

/// The honest-fallback sibling: printed when a tier DECLARED a `core` but the
/// running image cannot honor the pin (`CONFIG_SCHED_CPU_MASK_PIN_ONLY` off /
/// no SMP / bad cpu) — the tier runs unpinned, loudly. MIRRORS the `FAILED`
/// literals in both Zephyr arms — keep in lockstep.
pub const ZEPHYR_CORE_PIN_FALLBACK_MARKER: &str = "nros: core pin FAILED tier=";

/// issue 0260 — the CPU a tier was OBSERVED running on, printed from the tier's
/// own thread.
///
/// Distinct from [`ZEPHYR_CORE_PIN_MARKER`], which reports only that the kernel
/// ACCEPTED the pin. On a uniprocessor image that acceptance is true and says
/// nothing: a pin to cpu 0 is accepted and cpu 0 is the only CPU. This marker
/// is what an SMP fixture must assert on; without it the SMP image proves
/// exactly what native_sim already proves.
///
/// ABSENT on an image that cannot answer (no `CONFIG_SMP` — the posix arch does
/// not provide `arch_proc_id`), rather than reporting a fabricated `cpu 0`.
pub const ZEPHYR_CORE_PIN_OBSERVED_MARKER: &str = "nros: core pin observed tier=";

/// issue 0260 — the SMP fixture's exact expected line.
///
/// The PREFIX above is not a sufficient assertion for the placement cell: it
/// matches `running_on=0` just as happily, and 0 is where an unpinned tier
/// lands anyway. The cell must pin the VALUE, so the value is part of the
/// constant — including the tier name, because `smp_bringup` declares the
/// `core` on `high` and a different tier reporting it would not be the same
/// claim.
/// issue 0750 — BOARD-NEUTRAL on purpose. The NuttX board
/// (`nros_nuttx_report_observed_cpu`) prints this identical line, so a
/// `ZEPHYR_`-prefixed name would be drift by construction: the first NuttX cell
/// would grep a Zephyr-named constant, and the next person to slim the Zephyr
/// banner would have no way to know a second board depended on it. One literal,
/// one name, both boards.
pub const CORE_PIN_OBSERVED_CPU1: &str = "nros: core pin observed tier=`high` running_on=1";

/// Deprecated alias kept so the rename lands without touching the Zephyr cell in
/// the same commit. Remove once the NuttX cell exists and both read the
/// board-neutral name.
pub const ZEPHYR_CORE_PIN_OBSERVED_CPU1: &str = CORE_PIN_OBSERVED_CPU1;

/// Emitted by the NuttX board seam (`nuttx_run_tiers.c`,
/// `nros_nuttx_apply_current_affinity` — shared by the C/C++ AND Rust tier
/// arms) when the kernel accepted a tier's SMP core pin (phase-296 W5.11, the
/// placement dim). The board-agnostic literal is identical to the Zephyr arm's;
/// kept board-scoped here for lockstep clarity. MIRRORS the printf literal in
/// `nuttx_run_tiers.c` — keep in lockstep.
pub const NUTTX_CORE_PIN_MARKER: &str = "nros: core pin tier=";

/// phase-302 W3 (issue 0263) — the nuttx Rust arm's tier-priority adopt
/// marker (`nros_nuttx_apply_current_priority`, nuttx_run_tiers.c).
pub const NUTTX_TIER_PRIORITY_MARKER: &str = "nros: tier priority set tier=";

/// The loud-failure sibling: printed when a tier DECLARED a
/// `[tiers.*.nuttx] priority` but `pthread_setschedparam` rejected it — the
/// tier runs at its INHERITED priority, loudly. MIRRORS the `FAILED` printf in
/// `nuttx_run_tiers.c` — keep in lockstep.
///
/// issue 0579 / phase-358 W4 — this const exists because the e2e cell spelled
/// the note as a bare literal, which is the thing that lets a marker be slimmed
/// out from under a test.
pub const NUTTX_TIER_PRIORITY_FAILED_MARKER: &str = "nros: tier priority FAILED tier=";

/// issue 0636 — the FreeRTOS seam's tier-priority markers
/// (`freertos_apply_tier_priority`, freertos_run_tiers.c). The literals are the
/// SAME strings the NuttX seam prints; kept board-scoped here for lockstep
/// clarity, exactly as the two core-pin markers are.
///
/// These exist because until 0636 FreeRTOS printed no tier-priority marker in
/// EITHER language, so #579's "every declaring tier adopts or says why" was
/// enforced on NuttX alone — and a boot task that adopted nothing at all had no
/// cell that could see it.
pub const FREERTOS_TIER_PRIORITY_MARKER: &str = "nros: tier priority set tier=";

/// The loud-failure sibling of [`FREERTOS_TIER_PRIORITY_MARKER`]: printed when
/// a tier DECLARED a priority this build cannot express (>= `configMAX_PRIORITIES`).
/// `xTaskCreate` would clamp such a value SILENTLY, which makes a mis-authored
/// table read as honored. MIRRORS the `FAILED` literal in
/// `freertos_run_tiers.c` — keep in lockstep.
pub const FREERTOS_TIER_PRIORITY_FAILED_MARKER: &str = "nros: tier priority FAILED tier=";

/// Render the per-tier form of a tier-priority marker, so a test can assert
/// that a SPECIFIC tier adopted a SPECIFIC declared priority rather than that
/// some tier adopted something.
///
/// Board-neutral on purpose: the NuttX and FreeRTOS seams print the identical
/// line, so they share ONE renderer. A second per-kernel spelling of this is
/// exactly how the two seams came to differ in the first place (0636).
///
/// issue 0579 / phase-358 W4 — the reason this exists. The tier-priority cell
/// asserted `log.contains(NUTTX_TIER_PRIORITY_MARKER)`, which one spawned tier
/// satisfies for the whole image; the boot tier dropped its declared priority
/// for the life of the bug with that assert green. The rule is "every DECLARING
/// tier adopts or says why", so the gate has to name the tiers (the issue-0196
/// class: gate coverage narrower than the rule it enforces).
pub fn tier_priority_line(marker: &str, tier: &str, priority: u32) -> String {
    format!("{marker}`{tier}` prio={priority}")
}

/// The honest-fallback sibling: printed when a tier DECLARED a `core` but the
/// NuttX image lacks `CONFIG_SMP` (or the kernel rejected the pin) — the tier
/// runs unpinned, loudly. MIRRORS the `FAILED` literals in both NuttX seams —
/// keep in lockstep.
pub const NUTTX_CORE_PIN_FALLBACK_MARKER: &str = "nros: core pin FAILED tier=";

/// Emitted by the FreeRTOS board seam (`freertos_run_tiers.c`) over the
/// semihosting console when a tier's SMP core pin is applied
/// (`vTaskCoreAffinitySet` on a `configUSE_CORE_AFFINITY` build) — phase-296
/// W5.11, the placement dim. Board-agnostic literal; kept board-scoped for
/// lockstep clarity. MIRRORS the `semihosting_write0` literal — keep in lockstep.
pub const FREERTOS_CORE_PIN_MARKER: &str = "nros: core pin tier=";

/// The honest-fallback sibling: printed when a tier DECLARED a `core` but the
/// FreeRTOS build is uniprocessor (no `configUSE_CORE_AFFINITY`) — the tier
/// runs unpinned, loudly (before W5.11 this was a SILENT `(void)task`). MIRRORS
/// the `FAILED` literal in `freertos_run_tiers.c` — keep in lockstep.
pub const FREERTOS_CORE_PIN_FALLBACK_MARKER: &str = "nros: core pin FAILED tier=";

/// Emitted by the ThreadX board (`nros-board-threadx/src/entry.rs`,
/// `apply_tier_core_exclude`, boot + spawned) when the kernel accepted a tier's
/// SMP core pin (`tx_thread_smp_core_exclude` — the placement dim's ThreadX
/// realization, RFC-0052's "SMP core exclude"), phase-296 W5.13. Board-agnostic
/// literal; MIRRORS the `B::println` literal there — keep in lockstep.
pub const THREADX_CORE_PIN_MARKER: &str = "nros: core pin tier=";

/// The honest-fallback sibling: printed when a tier DECLARED a `core` but the
/// ThreadX build lacks `TX_THREAD_SMP` (no core-affinity API) — the tier runs
/// unpinned, loudly. MIRRORS the `FAILED` literal in `entry.rs` — keep in
/// lockstep.
pub const THREADX_CORE_PIN_FALLBACK_MARKER: &str = "nros: core pin FAILED tier=";

/// Emitted by the ThreadX board (`nros-board-threadx/src/entry.rs`, boot +
/// spawned) when a tier's round-robin time slice was applied
/// (`tx_thread_create` slice param / `tx_thread_time_slice_change`), phase-296
/// issue #0266. ThreadX honors a per-thread slice unconditionally, so this is
/// accept-only (no fallback). MIRRORS the `B::println` literal there — keep in
/// lockstep.
pub const THREADX_TIME_SLICE_MARKER: &str = "nros: time slice set tier=";

/// Emitted by the LINUX board (`nros-board-linux/src/lib.rs`,
/// `apply_tier_affinity`, boot + spawned) when `sched_setaffinity` pinned a
/// tier thread to its declared `core` (phase-296 W5.13, the placement dim). A
/// Linux host is genuinely multi-core and the call needs no privilege, so this
/// is the FIRST RUNTIME accept-arm proof of the core-pin consumer (issue #260).
/// MIRRORS the `B::println` literal there — keep in lockstep.
pub const LINUX_CORE_PIN_MARKER: &str = "nros: core pin tier=";

/// The honest-fallback sibling: printed when `sched_setaffinity` rejects a
/// declared `core` (bad cpu id) — the tier runs unpinned, loudly. MIRRORS the
/// `FAILED` literal in `nros-board-linux/src/lib.rs` — keep in lockstep.
pub const LINUX_CORE_PIN_FALLBACK_MARKER: &str = "nros: core pin FAILED tier=";

/// issue 0680 — the `errno-isolation` fixture's verdict lines.
///
/// The fixture proves newlib's `errno` is PER-THREAD on threadx-riscv64: one
/// task is driven into a failing `write(-1, …)` while another observes its
/// own. Before issue 0680 both resolved through one `_impure_ptr`.
///
/// Three markers rather than one, because "no PASS" has three very different
/// causes and a test that cannot tell them apart is the kind this fixture
/// exists to replace: SETUP means the probe never fired (the image could not
/// spawn, or `write(-1, …)` did not fail) and proves NOTHING about errno;
/// FAIL means the observer saw the victim's value, which is the bug; PASS
/// means isolation held. MIRRORS the `printf` literals in
/// `examples/qemu-riscv64-threadx/c/errno-isolation/src/main.c` — keep in
/// lockstep.
/// Common prefix of ALL three verdicts, so a harness can wait for "the fixture
/// decided" rather than for one outcome — waiting on PASS alone turns a real
/// FAIL into a timeout, which reads as a hang and hides the finding.
pub const ERRNO_ISOLATION_VERDICT: &str = "errno-isolation: verdict";
pub const ERRNO_ISOLATION_PASS: &str = "errno-isolation: verdict PASS per-thread errno";

/// phase-381 — the graph probe's verdict line. Like `ERRNO_ISOLATION_PASS`,
/// this is not delivery: it is the fixture saying it saw the peer it was told
/// to expect, which is the only thing a discovery workload can assert.
/// phase-425 W5 — the sim-clock listener's per-second report:
/// `SIMCLOCK t=<n> ros=<n> wall=<n> ros_now_ms=<n> active=<bool> attached=<bool>`.
///
/// A prefix and not a whole line, because the test parses the FIELDS: the
/// numbers are the evidence, and a marker that only proves the line printed
/// would pass on a listener whose ROS timer never ticked.
pub const SIMCLOCK_REPORT_PREFIX: &str = "SIMCLOCK t=";

/// phase-425 W5 — the publisher has stopped sending `/clock` and is idling with
/// its session still open. The test waits for this before measuring the SILENT
/// half: a peer that exited would take its participant out of the graph, and a
/// listener that stops ticking because its peer vanished has demonstrated
/// nothing about simulated time.
pub const SIMCLOCK_PUB_STOPPED: &str = "SIMCLOCK_PUB_STOPPED";

pub const GRAPH_PROBE_SAW: &str = "GRAPH_PROBE_SAW";

/// phase-381 verification — the probe exercised ALL ELEVEN graph slots against
/// a live peer and every one answered.
///
/// Distinct from [`GRAPH_PROBE_SAW`], which reports only that the peer was
/// enumerated. `check-rmw-slot-producers` calls all eleven `produced`, but that
/// means something writes and reads the slot — NOT that either met a real ROS 2
/// node. Issue 0903 was nine slots that had never been called at all sitting
/// behind two that had.
pub const GRAPH_PROBE_ALL_SLOTS_OK: &str = "GRAPH_PROBE_ALL_SLOTS_OK";
pub const ERRNO_ISOLATION_FAIL: &str = "errno-isolation: verdict FAIL shared errno";
pub const ERRNO_ISOLATION_SETUP: &str = "errno-isolation: verdict SETUP failed";

/// Extract the trimmed text after a marker in a line.
///
/// Returns `None` if the marker is not found.
fn extract_after<'a>(line: &'a str, marker: &str) -> Option<&'a str> {
    let idx = line.find(marker)?;
    Some(line[idx + marker.len()..].trim())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_talker_line_and_listener_line() {
        assert_eq!(talker_line(4), "Publishing: 'Hello World: 4'");
        assert_eq!(listener_line(250), "I heard: [Hello World: 250]");
        // The helpers build on the same prefix constants `parse_talker` /
        // `parse_listener` use, so a line built by `talker_line`/`listener_line`
        // round-trips through the parser.
        let output = format!("{}\n", talker_line(7));
        assert_eq!(parse_talker(&output).values, vec![7]);
        let output = format!("{}\n", listener_line(7));
        assert_eq!(parse_listener(&output).values, vec![7]);
    }

    #[test]
    fn test_parse_talker() {
        let output = "[INFO talker] Publishing: 'Hello World: 1'\n\
                      [INFO talker] Publishing: 'Hello World: 2'\n\
                      [INFO talker] Publishing: 'Hello World: 3'\n";
        let result = parse_talker(output);
        assert_eq!(result.published_count, 3);
        assert_eq!(result.values, vec![1, 2, 3]);
    }

    #[test]
    fn test_parse_listener() {
        let output = "[INFO listener] I heard: [Hello World: 5]\n\
                      [INFO listener] I heard: [Hello World: 6]\n";
        let result = parse_listener(output);
        assert_eq!(result.received_count, 2);
        assert_eq!(result.values, vec![5, 6]);
    }

    #[test]
    fn test_parse_talker_with_noise() {
        let output = "Starting up...\nPublishing: 'Hello World: 1'\nsome noise\n\
                      Publishing: 'abc'\nPublishing: 'Hello World: 2'\n";
        let result = parse_talker(output);
        // "Publishing: 'abc'" counts as a published line but yields no N
        assert_eq!(result.published_count, 3);
        assert_eq!(result.values, vec![1, 2]);
    }

    #[test]
    fn test_parse_hello_world_n_shapes() {
        // Quoted (talker), bracketed (listener), and bare payloads all parse.
        assert_eq!(parse_hello_world_n("'Hello World: 12'"), Some(12));
        assert_eq!(parse_hello_world_n("[Hello World: 12]"), Some(12));
        assert_eq!(parse_hello_world_n("Hello World: 12"), Some(12));
        assert_eq!(parse_hello_world_n("'Hello World: x'"), None);
        assert_eq!(parse_hello_world_n("42"), None);
    }

    #[test]
    fn test_parse_action_client() {
        let output = "Goal accepted by server, waiting for result\n\
                      Next number in sequence received: [0]\n\
                      Next number in sequence received: [0, 1]\n\
                      Result received: [0, 1]\n";
        let result = parse_action_client(output);
        assert!(result.goal_accepted);
        assert_eq!(result.feedback_count, 2);
        assert!(result.completed);
    }

    #[test]
    fn test_assert_monotonic() {
        assert_monotonic(&[0, 1, 2, 3]);
        assert_monotonic(&[0, 0, 1, 1, 2]);
        assert_monotonic(&[]);
        assert_monotonic(&[42]);
    }

    #[test]
    #[should_panic(expected = "not monotonically increasing")]
    fn test_assert_monotonic_fails() {
        assert_monotonic(&[0, 2, 1, 3]);
    }

    #[test]
    fn test_extract_after() {
        assert_eq!(
            extract_after("[INFO] Published: 42", "Published:"),
            Some("42")
        );
        assert_eq!(extract_after("no match here", "Published:"), None);
        assert_eq!(extract_after("Received: hello", "Received:"), Some("hello"));
    }
}

#[cfg(test)]
mod silence_tests {
    use super::runtime_silence_note;

    /// The exact shape issue 0459 reported: a Zephyr image that booted and then
    /// said nothing. The old failure named a missing EDF marker, which reads as
    /// a scheduling problem and is not one.
    #[test]
    fn a_boot_banner_and_nothing_else_is_silence() {
        let log = "WARNING: Using a test - not safe - entropy source\n\
                   *** Booting Zephyr OS build v3.7.0 ***\n";
        let note = runtime_silence_note(log).expect("must classify as silent");
        assert!(note.contains("NO RUNTIME OUTPUT"), "{note}");
        assert!(note.contains("2 non-empty line(s)"), "{note}");
        assert!(
            note.contains("Booting Zephyr"),
            "tail must be shown: {note}"
        );
    }

    #[test]
    fn empty_output_is_silence() {
        assert!(runtime_silence_note("").is_some());
        assert!(runtime_silence_note("\n\n  \n").is_some());
    }

    /// The other half, and the one that keeps this from swallowing real
    /// failures: once the runtime has spoken, a missing marker IS about the
    /// marker and the caller's own message must stand alone.
    #[test]
    fn a_runtime_that_spoke_is_not_silence() {
        let log = "*** Booting Zephyr OS ***\n[nros] tier task entered\n";
        assert!(runtime_silence_note(log).is_none());
        assert!(runtime_silence_note("nros: session open\n").is_none());
    }
}

// =============================================================================
// Readiness, by ROLE — issue 0481 / phase-342
// =============================================================================

/// The role a fixture binary plays in the standard ROS demo (talker/listener,
/// service, action, sink).
///
/// # Why this exists
///
/// The native examples are three implementations of the SAME demo, and they do
/// not print the same readiness line:
///
/// ```text
/// rust/listener   "Subscriber created for topic: /chatter"
/// c/listener      "Waiting for messages (Ctrl+C to exit)..."
/// cpp/listener    "Node created: …" then "Waiting for messages (…)..."
/// ```
///
/// So "wait until the listener is ready" has three spellings, and every call
/// site used to pick one by hand. That is not a hypothetical: nine sites picked
/// wrong, each waiting out its FULL timeout in silence — 90 s across the suite,
/// found only by noticing one cell was 7x its siblings (issue 0481).
///
/// Naming the ROLE instead of the STRING moves the choice to one place. A test
/// says what it is waiting for; this module knows how that role spells it in
/// each language.
///
/// The deeper fix is for the demos to agree on one banner, which would collapse
/// this mapping to a single constant. That is an EXAMPLE-facing change — the
/// banners are load-bearing for other greps (phase-277 slimmed them and broke
/// ~10 tests) — so it wants its own change with its own sweep. Until then, the
/// divergence is real and belongs in exactly one table.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DemoRole {
    /// Publishes on `/chatter`.
    Talker,
    /// Subscribes to `/chatter` — the role whose spelling differs most.
    Listener,
    /// The CRC-validating safety listener, which deliberately does NOT share the
    /// plain listener's marker (see [`SAFETY_LISTENER_READY_MARKER`]).
    SafetyListener,
    /// `add_two_ints` server.
    ServiceServer,
    /// Fibonacci action server.
    ActionServer,
    /// The `int32-sink` observer.
    Int32Sink,
}

/// The readiness marker for `(role, lang)` — the ONE place that mapping lives.
///
/// `lang` is currently ignored by every arm — the native demos were converged on
/// one spelling per role (phase-342), which is what makes that possible. It stays
/// in the signature because the divergence is a property of the EXAMPLES, not of
/// this table: a future language whose demo cannot print the shared line gets an
/// arm here instead of a literal at the call site.
pub fn ready_marker(role: DemoRole, _lang: crate::matrix::Lang) -> &'static str {
    match role {
        DemoRole::Talker => TALKER_READY_MARKER,
        // phase-342 — ONE marker for every language. The three native listeners
        // used to spell readiness three ways (rust "Subscriber created…", C
        // "Subscription created…", C++ nothing but a "Waiting for messages"
        // banner), which is what made a hand-picked literal match two languages
        // out of three and silently time out on the third. They now all print
        // `LISTENER_READY_MARKER`, so this arm no longer branches on `lang`.
        DemoRole::Listener => LISTENER_READY_MARKER,
        DemoRole::SafetyListener => SAFETY_LISTENER_READY_MARKER,
        DemoRole::ServiceServer => SERVICE_SERVER_READY_MARKER,
        DemoRole::ActionServer => ACTION_SERVER_READY_MARKER,
        DemoRole::Int32Sink => INT32_SINK_READY_MARKER,
    }
}

// ── Guest-side failure signatures (issue 0557 / phase-358 W5) ───────────────
//
// A readiness wait that times out reports "didn't reach readiness within N s".
// That is a true statement and usually the WRONG headline: when the guest died
// at boot, the marker was never going to arrive, and the actual error is
// already sitting in the collected output a few lines up. The reader takes the
// timeout as the verdict and starts investigating latency.
//
// Issue 0445 named this shape at the fixture level — "a STALE verdict is
// ABSORBING … whatever it would have done at runtime is replaced by a message
// that explains itself" — and 0557 is the same thing one layer out: a
// self-explaining harness verdict standing in front of a specific guest error.
// 0557's own report shows six `tid … is in use!` lines and
// `run_components failed rc=-100` under a headline that says the server
// "didn't reach readiness within 60 s".
//
// So: when a readiness wait fails, LEAD with the guest's own error if there is
// one. The signatures below are all failures observed in this tree, not a
// speculative list:
//
//   `run_components failed rc=`  the entry's own failure code (0557)
//   `ZEPHYR FATAL ERROR`         Zephyr's fatal handler (0552 stack overflow)
//   `USAGE FAULT` / `MPU FAULT`  Cortex-M faults (0552: PC=0 from an overflow)
//   `BUS FAULT` / `HARD FAULT`   siblings of the above
//   `is in use!`                 Zephyr dynamic thread stack reuse (0557)
//   `<err> os:`                  any Zephyr kernel error log line
//   `panicked at`                a Rust panic inside the guest
//
// ORDER IS PRECEDENCE, not documentation — `first_guest_failure` scans this
// list in order and returns the first line matching the highest-ranked
// signature present (phase-358 W5). Put the most specific, most authoritative
// signatures first: an entry's own error code beats a kernel log line, and a
// kernel log line beats a generic `<err>`. `is in use!` is deliberately LOW:
// issue 0557 established it is benign (Zephyr's `pthread_attr_destroy` frees
// the stack of the thread it just created, logs `-EBUSY`, and returns 0), and
// it is printed BEFORE the real failure, so ranking it high made the verdict
// lead with a red herring.
pub const GUEST_FAILURE_SIGNATURES: &[&str] = &[
    "run_components failed rc=",
    "ZEPHYR FATAL ERROR",
    "USAGE FAULT",
    "MPU FAULT",
    "BUS FAULT",
    "HARD FAULT",
    "is in use!",
    "<err> os:",
    "panicked at",
];

/// The most SPECIFIC guest-side failure line in `output`, if any.
///
/// Scanning is rank-major: take [`GUEST_FAILURE_SIGNATURES`] in order and
/// return the first line matching the highest-ranked signature present, not the
/// earliest failure-ish line in the log.
///
/// phase-358 W5 — it used to be plain line order, on the reasoning that "the
/// earliest failure is usually the cause and the rest are its consequences".
/// Issue 0557 is the counter-example, and it was the very issue this function
/// was written for: the guest prints six `tid … is in use!` lines BEFORE
/// `run_components failed rc=-100`, so line order made the verdict lead with
/// them. Those lines are benign — Zephyr's `pthread_attr_destroy` calls
/// `k_thread_stack_free` on the stack of the thread it just created, logs
/// `-EBUSY`, and returns 0 anyway — so the headline named a red herring while
/// the entry's own error code sat four lines down. A diagnostic that names the
/// wrong line is a new hiding place, which is the thing this function exists to
/// remove.
///
/// Within one signature it is still the earliest matching line, so 0552's
/// fault-then-register-dump and 0557's six consecutive lines each collapse to
/// their first.
///
/// Returns the whole line, trimmed — callers put it in the headline so the
/// verdict names the failure instead of naming the wait that observed it.
pub fn first_guest_failure(output: &str) -> Option<&str> {
    GUEST_FAILURE_SIGNATURES.iter().find_map(|sig| {
        output
            .lines()
            .map(str::trim)
            .find(|line| line.contains(sig))
    })
}

#[cfg(test)]
mod guest_failure_tests {
    use super::*;

    #[test]
    fn the_0557_transcript_reports_its_own_error_not_the_timeout() {
        // Verbatim shape from issue 0557.
        let out = "*** Booting Zephyr OS build v3.7.0 ***\n\
                   <inf> cyclonedds: session_create: domain=29 entering\n\
                   <err> os: tid 0x581fa0 is in use!\n\
                   <inf> cyclonedds: dds_create_participant returned 49379019\n\
                   nros zephyr entry: run_components failed rc=-100\n";
        // phase-358 W5 — this used to expect the `tid … is in use!` line, on
        // the theory that the earliest failure is the cause. Running the guest
        // disproved it: those lines are benign (Zephyr's `pthread_attr_destroy`
        // frees the stack of the thread it just created, logs `-EBUSY`, returns
        // 0), and leading with them buried the entry's own error code four
        // lines down — a diagnostic pointing at a red herring, which is the
        // failure mode this helper exists to prevent.
        assert_eq!(
            first_guest_failure(out),
            Some("nros zephyr entry: run_components failed rc=-100")
        );
    }

    #[test]
    fn a_benign_line_alone_is_still_reported() {
        // Ranking `is in use!` low must not make it invisible: with nothing
        // better in the log it is still far more use than a bare timeout.
        let out = "*** Booting Zephyr OS build v3.7.0 ***\n\
                   <err> os: tid 0x581fa0 is in use!\n";
        assert_eq!(
            first_guest_failure(out),
            Some("<err> os: tid 0x581fa0 is in use!")
        );
    }

    #[test]
    fn precedence_beats_line_order_but_not_within_a_signature() {
        // A later, higher-ranked signature wins over an earlier, lower one …
        let out = "<err> os: tid 0x1 is in use!\n\
                   <err> os: ***** USAGE FAULT *****\n";
        assert_eq!(
            first_guest_failure(out),
            Some("<err> os: ***** USAGE FAULT *****")
        );
        // … while repeats of the SAME signature still collapse to the first,
        // which is what makes 0552's fault-then-register-dump read correctly.
        let dump = "<err> os: ***** USAGE FAULT *****\n\
                    <err> os: ***** USAGE FAULT ***** (second core)\n";
        assert_eq!(
            first_guest_failure(dump),
            Some("<err> os: ***** USAGE FAULT *****")
        );
    }

    #[test]
    fn a_healthy_boot_has_no_failure_line() {
        let out = "*** Booting Zephyr OS build v3.7.0 ***\n\
                   Subscriber created for topic: /chatter\n\
                   I heard: [Hello World: 1]\n";
        assert_eq!(first_guest_failure(out), None);
    }

    #[test]
    fn a_cortex_m_fault_is_recognised() {
        // Issue 0552's transcript: the fault line precedes the register dump.
        let out = "<err> os: ***** USAGE FAULT *****\n\
                   <err> os:   Illegal use of the EPSR\n";
        assert_eq!(
            first_guest_failure(out),
            Some("<err> os: ***** USAGE FAULT *****")
        );
    }
}
