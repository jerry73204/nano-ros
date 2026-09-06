//! Native C / C++ API integration tests.
//!
//! Exercises the prebuilt CMake C and C++ examples (talker, listener,
//! service-server/-client, action-server/-client) as a single parametrised
//! suite. `just native build-fixtures` stages these under isolated
//! `build-<rmw>/` dirs so tests do not compile or overwrite native
//! archives at runtime. Language-agnostic tests are generated as two cases
//! (`::case_1_C` and `::case_2_Cpp`) via `#[rstest]`; language-specific
//! tests (interop tests that always pair with Rust, the C++ goal-rejection
//! test, the C action blocking case) stay as named functions below.
//!
//! **Bucket (phase-329 W4):** genuine one-offs no matrix cell covers, and stays
//! — cross-LANGUAGE interop pairs (`*_interop_*`, `c_rust_*`), the RFC-0041
//! callback-shape variants (`*_callback`), the C++ goal-rejection path, the MIXED
//! cyclone pairs (rust talker ↔ c/cpp listener), and the threadx-linux ↔ native
//! cross-PLATFORM cyclone interop. The four same-language delivery cases that
//! duplicated the matrix consumers (`test_native_talker_listener_communication`
//! rust-zenoh pubsub; `test_native_service_communication` c/cpp-zenoh service;
//! `test_native_cyclonedds_{service,action}` c/cpp-cyclone) were FOLDED into
//! `native_example_pubsub_e2e` / `native_example_reqresp_e2e` (phase-329 W4).
//!
//! Consolidates what used to be `c_api.rs` + `cpp_api.rs` (Phase 85.1).

use nros_tests::{
    count_pattern,
    fixtures::{
        ManagedProcess, Rmw, ZenohRouter, build_c_action_client, build_c_action_server,
        build_c_listener, build_c_service_client_callback, build_c_service_server, build_c_talker,
        build_cpp_action_client, build_cpp_action_client_callback, build_cpp_action_server,
        build_cpp_listener, build_cpp_service_client_callback, build_cpp_service_server,
        build_cpp_talker, build_native_c_example_rmw, build_native_cpp_example_rmw,
        build_native_listener_rmw, build_native_service_client_callback, build_native_talker_rmw,
        require_cmake, require_zenohd, zenohd_unique,
    },
    output::{
        ACTION_EXECUTING_MARKER, ACTION_GOAL_ACCEPTED_PREFIX, ACTION_GOAL_NO_RESPONSE_PREFIX,
        ACTION_GOAL_REJECTED_PREFIX, ACTION_GOAL_REQUEST_PREFIX, ACTION_RESULT_PREFIX,
        ACTION_SERVER_READY_MARKER, SERVICE_RESULT_PREFIX, service_result_line,
    },
};
use rstest::rstest;
use std::{
    path::{Path, PathBuf},
    process::Command,
    time::Duration,
};

// =============================================================================
// Language selector
// =============================================================================

/// Which of the two native FFI surfaces the parametrised tests exercise.
#[derive(Clone, Copy, Debug)]
#[allow(clippy::upper_case_acronyms)]
enum Language {
    C,
    Cpp,
}

impl Language {
    /// Process-name prefix used in log lines (e.g. `"c-talker"`).
    fn tag(&self) -> &'static str {
        match self {
            Language::C => "c",
            Language::Cpp => "cpp",
        }
    }

    /// Human-readable label for `eprintln!` output.
    fn label(&self) -> &'static str {
        match self {
            Language::C => "C",
            Language::Cpp => "C++",
        }
    }

    /// Substring that every language-native binary prints after
    /// `nros::init` / `nros_support_init` succeeds. Used to assert that
    /// the binary started cleanly.
    fn init_marker(&self) -> &'static str {
        match self {
            // C binaries print `"Support initialized"` from `nros_support_init`.
            Language::C => "Support initialized",
            // C++ binaries print `"Node created: <name>"` from `nros::create_node`.
            Language::Cpp => "Node created",
        }
    }

    /// Substring proving the TALKER started and is publishing. Phase-277
    /// W5.C1 slimmed `native/c/talker` to match the official ROS 2 demo,
    /// which dropped the `"Support initialized"` print — the publish line
    /// (identical wording in both languages, demo parity) is the reliable
    /// started-marker for talkers; it also proves the timer fired.
    fn talker_ready_marker(&self) -> &'static str {
        nros_tests::output::TALKER_PAYLOAD_PREFIX
    }

    fn talker_binary(&self) -> PathBuf {
        match self {
            Language::C => build_c_talker(),
            Language::Cpp => build_cpp_talker(),
        }
        .unwrap_or_else(|e| skip_missing_fixture("native talker", e))
        .to_path_buf()
    }

    fn listener_binary(&self) -> PathBuf {
        match self {
            Language::C => build_c_listener(),
            Language::Cpp => build_cpp_listener(),
        }
        .unwrap_or_else(|e| skip_missing_fixture("native listener", e))
        .to_path_buf()
    }

    fn service_server_binary(&self) -> PathBuf {
        match self {
            Language::C => build_c_service_server(),
            Language::Cpp => build_cpp_service_server(),
        }
        .unwrap_or_else(|e| skip_missing_fixture("native service server", e))
        .to_path_buf()
    }

    fn action_server_binary(&self) -> PathBuf {
        match self {
            Language::C => build_c_action_server(),
            Language::Cpp => build_cpp_action_server(),
        }
        .unwrap_or_else(|e| skip_missing_fixture("native action server", e))
        .to_path_buf()
    }

    fn action_client_binary(&self) -> PathBuf {
        match self {
            Language::C => build_c_action_client(),
            Language::Cpp => build_cpp_action_client(),
        }
        .unwrap_or_else(|e| skip_missing_fixture("native action client", e))
        .to_path_buf()
    }
}

fn skip_missing_fixture(label: &str, err: nros_tests::TestError) -> ! {
    match err {
        nros_tests::TestError::BuildFailed(msg) if msg.contains("not prebuilt") => {
            nros_tests::skip!("{label} fixture not prebuilt: {msg}");
        }
        other => panic!("failed to resolve {label} fixture: {other:?}"),
    }
}

/// Wrap a native binary with `stdbuf -oL -eL` to force line-buffered
/// stdout/stderr. Both C's `printf` and C++'s `std::printf` fully-buffer
/// when the output is a pipe, which breaks `wait_for_output_pattern`.
fn stdbuf_command(binary: &Path) -> Command {
    let mut cmd = Command::new("stdbuf");
    cmd.args(["-oL", "-eL"]).arg(binary);
    cmd
}

/// Skip-guard shared by every test in this file.
/// Skip-or-proceed guard: returns normally ONLY when every precondition holds.
///
/// Issue 1135 — this returned `bool` and every one of its ~20 callers wrote
/// `if !require_native_env() { return; }`. Both arms already skip, so `false`
/// was unreachable and the guard branch was dead code — but it is the same
/// SHAPE as the live defect in `params.rs`, where a bool-returning `require_*`
/// let four call sites turn an unmet precondition into a green `#[test]`. The
/// dead branch was a standing invitation to add a third arm that returns
/// `false` instead of skipping. Returning `()` removes the invitation.
fn require_native_env() {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }
    if !require_cmake() {
        nros_tests::skip!("cmake not found");
    }
}

// =============================================================================
// (Phase 182.3) The per-role/-lang `*_builds` presence tests were removed —
// they only asserted a fixture binary built, coverage already provided by
// `build-all` (Phase 181) + the `_require-fixtures` preflight, and the
// startup / pub-sub / service / action e2e tests below build+run the same
// binaries. The `lang.*_binary()` resolvers live on; the e2e tests use them.
// =============================================================================

// =============================================================================
// Startup tests (parametrised)
// =============================================================================

fn spawn_native(binary: &Path, lang: Language, kind: &str, locator: &str) -> ManagedProcess {
    let mut cmd = stdbuf_command(binary);
    cmd.env("NROS_LOCATOR", locator);
    let name = format!("{}-{}", lang.tag(), kind);
    ManagedProcess::spawn_command(cmd, &name).unwrap_or_else(|_| panic!("Failed to start {name}"))
}

#[rstest]
fn test_native_talker_starts(
    zenohd_unique: ZenohRouter,
    #[values(Language::C, Language::Cpp)] lang: Language,
) {
    require_native_env();
    let locator = zenohd_unique.locator();
    let binary = lang.talker_binary();
    let mut talker = spawn_native(&binary, lang, "talker", &locator);

    // Wait for initialization — bounded by the init marker rather than
    // a fixed sleep. `wait_for_output_pattern` times out at 10s but
    // normally returns in <1s once `nros::init` completes.
    let output = talker.collect_until(lang.talker_ready_marker(), Duration::from_secs(30));

    eprintln!("{} talker output:\n{}", lang.label(), output);
    assert!(
        output.contains(lang.talker_ready_marker()),
        "{} talker failed to initialize.\nOutput:\n{}",
        lang.label(),
        output
    );
}

#[rstest]
fn test_native_listener_starts(
    zenohd_unique: ZenohRouter,
    #[values(Language::C, Language::Cpp)] lang: Language,
) {
    require_native_env();
    let locator = zenohd_unique.locator();
    let binary = lang.listener_binary();
    let mut listener = spawn_native(&binary, lang, "listener", &locator);

    let output = listener.collect_until(lang.init_marker(), Duration::from_secs(30));

    eprintln!("{} listener output:\n{}", lang.label(), output);
    assert!(
        output.contains(lang.init_marker()),
        "{} listener failed to initialize.\nOutput:\n{}",
        lang.label(),
        output
    );
}

#[rstest]
fn test_native_service_server_starts(
    zenohd_unique: ZenohRouter,
    #[values(Language::C, Language::Cpp)] lang: Language,
) {
    require_native_env();
    let locator = zenohd_unique.locator();
    let binary = lang.service_server_binary();
    let mut server = spawn_native(&binary, lang, "service-server", &locator);

    let output = server.collect_until(lang.init_marker(), Duration::from_secs(30));

    eprintln!("{} service server output:\n{}", lang.label(), output);
    assert!(
        output.contains(lang.init_marker()),
        "{} service server failed to initialize.\nOutput:\n{}",
        lang.label(),
        output
    );
}

#[rstest]
fn test_native_action_server_starts(
    zenohd_unique: ZenohRouter,
    #[values(Language::C, Language::Cpp)] lang: Language,
) {
    require_native_env();
    let locator = zenohd_unique.locator();
    let binary = lang.action_server_binary();
    let mut server = spawn_native(&binary, lang, "action-server", &locator);

    let output = server.collect_until(lang.init_marker(), Duration::from_secs(30));

    eprintln!("{} action server output:\n{}", lang.label(), output);
    assert!(
        output.contains(lang.init_marker()),
        "{} action server failed to initialize.\nOutput:\n{}",
        lang.label(),
        output
    );
}

// =============================================================================
// Service communication — callback receive (RFC-0041 / Phase 239)
// =============================================================================

/// Callback service client (C or C++) vs the stock same-language service
/// server. Proves the RFC-0041 callback receive path E2E: replies arrive
/// through the typed response handler dispatched at `spin_once`
/// (`nros_executor_spin_some` / `nros::spin_once`) — no Promise/Future poll.
/// Both `service-client-callback` variants log the demo result line
/// (`Result of add_two_ints: N`) from inside the registered callback, so its
/// presence proves the callback dispatched.
#[rstest]
fn test_native_service_communication_callback(
    zenohd_unique: ZenohRouter,
    #[values(Language::C, Language::Cpp)] lang: Language,
) {
    require_native_env();
    let locator = zenohd_unique.locator();
    let server_bin = lang.service_server_binary();
    let client_bin = match lang {
        Language::C => build_c_service_client_callback(),
        Language::Cpp => build_cpp_service_client_callback(),
    }
    .unwrap_or_else(|e| skip_missing_fixture("native service client (callback)", e))
    .to_path_buf();

    let mut server = spawn_native(&server_bin, lang, "service-server", &locator);
    server
        .wait_for_output_pattern(
            nros_tests::output::SERVICE_SERVER_READY_MARKER,
            Duration::from_secs(30),
        )
        .expect("service server did not become ready");

    let mut client = spawn_native(&client_bin, lang, "service-client-callback", &locator);

    // issue 1026 — every client wait in this file used to read
    // `wait_for_output_pattern(M, t).or_else(|_| wait_for_all_output(2s))
    //  .unwrap_or_default()`, which destroys its own evidence twice: the strict
    // wait had already collected the transcript into an `Err` the `or_else`
    // drops, and the fallback then KILLS the client to gather the two seconds
    // that remain — so a failing cell reports the tail of a run it truncated.
    // `collect_until` is the primitive for "assert on the content yourself":
    // it returns at the marker, returns everything printed when the marker
    // never comes, and kills nothing.
    //
    // Bound stated (all of these): a single request/goal, asserted on its
    // terminal line. Nothing here observes what the SUT does after that.
    // Swept with:
    //   rg -n 'or_else\(\|_\| client\.wait_for_all_output' packages/testing
    let client_output = client.collect_until(SERVICE_RESULT_PREFIX, Duration::from_secs(15));

    server.kill();

    eprintln!(
        "{} callback service client output:\n{}",
        lang.label(),
        client_output
    );

    // The reply must arrive via the callback, not a poll: the result line is
    // printed from inside the registered handler.
    assert!(
        client_output.contains(&service_result_line(5)),
        "Expected the callback-dispatched `{}` reply.\nOutput:\n{}",
        service_result_line(5),
        client_output
    );
}

// =============================================================================
// Cross-language service-callback interop (RFC-0041 / Phase 239.15)
// =============================================================================
//
// Pair each language's callback CLIENT against the *other* language's service
// server to prove the callback receive model is wire-compatible and
// backend-agnostic — the reply is framed by one language's RMW and dispatched
// to the other's spin-time callback. Reuses the existing same-language
// fixtures; only the pairing is new.

/// Run a callback-client (`client_bin`, language `client_lang`) against a
/// service server (`server_bin`) on a fresh router; assert the reply arrives
/// via the callback. Shared by the cross-language 239.15 cases.
fn service_callback_interop_body(
    locator: &str,
    server_bin: &Path,
    client_lang: Language,
    client_bin: &Path,
) {
    let mut server = spawn_native(server_bin, client_lang, "service-server", locator);
    server
        .wait_for_output_pattern(
            nros_tests::output::SERVICE_SERVER_READY_MARKER,
            Duration::from_secs(30),
        )
        .expect("service server did not become ready");

    let mut client = spawn_native(client_bin, client_lang, "service-client-callback", locator);
    let client_output = client.collect_until(SERVICE_RESULT_PREFIX, Duration::from_secs(15));
    server.kill();

    eprintln!("cross-lang callback client output:\n{}", client_output);
    assert!(
        client_output.contains(&service_result_line(5)),
        "Expected the callback-dispatched `{}` reply cross-language.\nOutput:\n{}",
        service_result_line(5),
        client_output
    );
}

/// C callback client ↔ C++ service server.
#[rstest]
fn test_service_callback_interop_c_client_cpp_server(zenohd_unique: ZenohRouter) {
    require_native_env();
    let server_bin = Language::Cpp.service_server_binary();
    let client_bin = build_c_service_client_callback()
        .unwrap_or_else(|e| skip_missing_fixture("C service client (callback)", e))
        .to_path_buf();
    service_callback_interop_body(
        &zenohd_unique.locator(),
        &server_bin,
        Language::C,
        &client_bin,
    );
}

/// C++ callback client ↔ C service server.
#[rstest]
fn test_service_callback_interop_cpp_client_c_server(zenohd_unique: ZenohRouter) {
    require_native_env();
    let server_bin = Language::C.service_server_binary();
    let client_bin = build_cpp_service_client_callback()
        .unwrap_or_else(|e| skip_missing_fixture("C++ service client (callback)", e))
        .to_path_buf();
    service_callback_interop_body(
        &zenohd_unique.locator(),
        &server_bin,
        Language::Cpp,
        &client_bin,
    );
}

/// Spawn the native Rust callback service client. Unlike the C / C++ binaries it
/// logs via `env_logger`, so `RUST_LOG=info` is required to surface the
/// `Result of add_two_ints:` marker.
fn spawn_rust_callback_client(binary: &Path, locator: &str) -> ManagedProcess {
    let mut cmd = stdbuf_command(binary);
    cmd.env("NROS_LOCATOR", locator);
    cmd.env("RUST_LOG", "info");
    ManagedProcess::spawn_command(cmd, "rust-service-client-callback")
        .expect("Failed to start rust-service-client-callback")
}

/// Rust callback client ↔ a C / C++ service server. Extends the 239.15 matrix to
/// the Rust FFI surface: the typed `create_client_with_callback` closure
/// dispatched at `spin_once` receives a reply framed by the other language's RMW.
fn rust_callback_interop_body(locator: &str, server_bin: &Path, server_lang: Language) {
    let client_bin = build_native_service_client_callback()
        .unwrap_or_else(|e| skip_missing_fixture("Rust service client (callback)", e))
        .to_path_buf();

    let mut server = spawn_native(server_bin, server_lang, "service-server", locator);
    server
        .wait_for_output_pattern(
            nros_tests::output::SERVICE_SERVER_READY_MARKER,
            Duration::from_secs(30),
        )
        .expect("service server did not become ready");

    let mut client = spawn_rust_callback_client(&client_bin, locator);
    let client_output = client.collect_until(SERVICE_RESULT_PREFIX, Duration::from_secs(20));
    server.kill();

    eprintln!(
        "rust callback client ↔ {} server output:\n{}",
        server_lang.label(),
        client_output
    );
    assert!(
        client_output.contains(&service_result_line(5)),
        "Expected the callback-dispatched `{}` reply (rust↔{}).\nOutput:\n{}",
        service_result_line(5),
        server_lang.label(),
        client_output
    );
}

/// Rust callback client ↔ C service server.
#[rstest]
fn test_service_callback_interop_rust_client_c_server(zenohd_unique: ZenohRouter) {
    require_native_env();
    let server_bin = Language::C.service_server_binary();
    rust_callback_interop_body(&zenohd_unique.locator(), &server_bin, Language::C);
}

/// Rust callback client ↔ C++ service server.
#[rstest]
fn test_service_callback_interop_rust_client_cpp_server(zenohd_unique: ZenohRouter) {
    require_native_env();
    let server_bin = Language::Cpp.service_server_binary();
    rust_callback_interop_body(&zenohd_unique.locator(), &server_bin, Language::Cpp);
}

// =============================================================================
// Action communication (one function per language)
// =============================================================================
//
// These stay as two small wrappers around a shared body so failures keep
// clear language-specific names.

fn native_action_communication_body(lang: Language, locator: &str) {
    let server_bin = lang.action_server_binary();
    let client_bin = lang.action_client_binary();

    let mut server = spawn_native(&server_bin, lang, "action-server", locator);
    server
        .wait_for_output_pattern(
            nros_tests::output::ACTION_SERVER_READY_MARKER,
            Duration::from_secs(30),
        )
        .expect("action server did not become ready");

    let mut client = spawn_native(&client_bin, lang, "action-client", locator);

    // Both clients end with the demo terminal line `Result received: [...]`.
    //
    // issue 1026 — the `.or_else(wait_for_all_output).unwrap_or_default()`
    // idiom this replaces destroyed its own evidence twice over: the strict
    // wait had already consumed the transcript into an `Err` the `or_else`
    // dropped, and the fallback then KILLED the client to collect the two
    // seconds that remained. `collect_until` returns at the marker, returns
    // everything printed when it does not appear, and kills nothing.
    let client_output = client.collect_until(ACTION_RESULT_PREFIX, Duration::from_secs(20));

    // The server is free-running (`spin = "forever"`), so there is no drain
    // that ends by itself. Wait for the marker the assertion below names — by
    // now the goal has already completed on the client side, so this normally
    // returns immediately from what is already buffered.
    //
    // Bound stated: this observes the server only up to the FIRST goal it
    // executed. A server that wedges after one goal, or whose session lapses
    // later, is outside what this cell can see — it drives exactly one goal.
    let server_output = server.collect_until(ACTION_EXECUTING_MARKER, Duration::from_secs(5));
    server.kill();

    eprintln!(
        "=== {} action server output ===\n{}",
        lang.label(),
        server_output
    );
    eprintln!(
        "=== {} action client output ===\n{}",
        lang.label(),
        client_output
    );

    assert!(
        client_output.contains("Goal accepted"),
        "{} action client failed to send goal or get acceptance.\nOutput:\n{}",
        lang.label(),
        client_output
    );
    assert!(
        client_output.contains(ACTION_RESULT_PREFIX),
        "{} action client did not receive a result.\nOutput:\n{}",
        lang.label(),
        client_output
    );
    assert!(
        server_output.contains(ACTION_EXECUTING_MARKER),
        "{} action server did not process the goal.\nServer output:\n{}",
        lang.label(),
        server_output
    );
}

#[rstest]
fn test_c_action_communication(zenohd_unique: ZenohRouter) {
    require_native_env();
    native_action_communication_body(Language::C, &zenohd_unique.locator());
}

#[rstest]
fn test_cpp_action_communication(zenohd_unique: ZenohRouter) {
    require_native_env();
    native_action_communication_body(Language::Cpp, &zenohd_unique.locator());
}

// =============================================================================
// Action communication — callback receive (RFC-0041 / Phase 239.14)
// =============================================================================

/// C++ callback action client vs the stock C++ action server. Proves the
/// RFC-0041 action callback path E2E: goal-response, feedback, and result all
/// arrive through `SendGoalOptions` callbacks dispatched by `ActionClient::poll()`
/// at each `spin_once` (no Future/stream poll). Asserts acceptance, ≥1 feedback
/// callback, and the full Fibonacci result sequence delivered via the callback.
#[rstest]
fn test_cpp_action_communication_callback(zenohd_unique: ZenohRouter) {
    require_native_env();
    let locator = zenohd_unique.locator();
    let server_bin = Language::Cpp.action_server_binary();
    let client_bin = build_cpp_action_client_callback()
        .unwrap_or_else(|e| skip_missing_fixture("native action client (callback)", e))
        .to_path_buf();

    let mut server = spawn_native(&server_bin, Language::Cpp, "action-server", &locator);
    server
        .wait_for_output_pattern(
            nros_tests::output::ACTION_SERVER_READY_MARKER,
            Duration::from_secs(30),
        )
        .expect("action server did not become ready");

    let mut client = spawn_native(
        &client_bin,
        Language::Cpp,
        "action-client-callback",
        &locator,
    );

    let client_output = client.collect_until(ACTION_RESULT_PREFIX, Duration::from_secs(25));

    server.kill();

    eprintln!("C++ callback action client output:\n{}", client_output);

    assert!(
        client_output.contains(nros_tests::output::ACTION_GOAL_ACCEPTED_MARKER),
        "Expected goal-response callback to fire with acceptance.\nOutput:\n{}",
        client_output
    );
    // Fibonacci(order=10) result delivered via the result callback.
    assert!(
        client_output.contains("Result received: [0, 1, 1, 2, 3, 5, 8"),
        "Expected the result callback to deliver the Fibonacci sequence.\nOutput:\n{}",
        client_output
    );
    // At least one feedback callback fired during execution.
    let fb_count = count_pattern(&client_output, nros_tests::output::ACTION_FEEDBACK_PREFIX);
    assert!(
        fb_count >= 1,
        "Expected >=1 feedback callback, got {} feedback.\nOutput:\n{}",
        fb_count,
        client_output
    );
}

// =============================================================================
// Cross-language action-callback interop (RFC-0041 / Phase 239.15)
// =============================================================================
//
// Pair each language's callback action client against the *other* language's
// action server. Proves the action callback receive model (goal-response /
// feedback / result dispatched at spin) is wire-compatible across the C / C++
// FFI surfaces over zenoh. Sequence length differs slightly by server
// (`c_action_server` emits one more Fibonacci term than `cpp_action_server`),
// so assert a stable prefix rather than the exact tail.

/// C++ callback action client ↔ C action server.
#[rstest]
fn test_action_callback_interop_cpp_client_c_server(zenohd_unique: ZenohRouter) {
    require_native_env();
    let locator = zenohd_unique.locator();
    let server_bin = build_c_action_server()
        .unwrap_or_else(|e| skip_missing_fixture("C action server", e))
        .to_path_buf();
    let client_bin = build_cpp_action_client_callback()
        .unwrap_or_else(|e| skip_missing_fixture("C++ action client (callback)", e))
        .to_path_buf();

    let mut server = spawn_native(&server_bin, Language::C, "action-server", &locator);
    server
        .wait_for_output_pattern(
            nros_tests::output::ACTION_SERVER_READY_MARKER,
            Duration::from_secs(30),
        )
        .expect("C action server did not become ready");
    let mut client = spawn_native(
        &client_bin,
        Language::Cpp,
        "action-client-callback",
        &locator,
    );

    let client_output = client.collect_until(ACTION_RESULT_PREFIX, Duration::from_secs(25));
    server.kill();

    eprintln!(
        "cross-lang action client (cpp↔c) output:\n{}",
        client_output
    );
    assert!(
        client_output.contains(nros_tests::output::ACTION_GOAL_ACCEPTED_MARKER),
        "Expected goal-response acceptance across languages.\nOutput:\n{}",
        client_output
    );
    assert!(
        client_output.contains("Result received: [0, 1, 1, 2"),
        "Expected the result callback to deliver the Fibonacci prefix.\nOutput:\n{}",
        client_output
    );
}

/// C action client ↔ C++ action server (the reverse of the pairing above).
///
/// Completes the action cross-language matrix. The C action client uses its
/// feedback/result callbacks; this asserts the C++ server returns the full
/// Fibonacci result for the C-framed goal (issue #43 — was a stale pre-233.6 C
/// fixture writing a now-removed GoalId sequence prefix; resolved by a fresh
/// build).
#[rstest]
fn test_action_callback_interop_c_client_cpp_server(zenohd_unique: ZenohRouter) {
    require_native_env();
    let locator = zenohd_unique.locator();
    let server_bin = build_cpp_action_server()
        .unwrap_or_else(|e| skip_missing_fixture("C++ action server", e))
        .to_path_buf();
    let client_bin = build_c_action_client()
        .unwrap_or_else(|e| skip_missing_fixture("C action client", e))
        .to_path_buf();

    let mut server = spawn_native(&server_bin, Language::Cpp, "action-server", &locator);
    server
        .wait_for_output_pattern(
            nros_tests::output::ACTION_SERVER_READY_MARKER,
            Duration::from_secs(30),
        )
        .expect("C++ action server did not become ready");
    let mut client = spawn_native(&client_bin, Language::C, "action-client", &locator);

    let client_output = client.collect_until(ACTION_RESULT_PREFIX, Duration::from_secs(25));
    server.kill();

    eprintln!(
        "cross-lang action client (c↔cpp) output:\n{}",
        client_output
    );
    assert!(
        client_output.contains("Goal accepted"),
        "Expected the C client to get goal acceptance from the C++ server.\nOutput:\n{}",
        client_output
    );
    assert!(
        client_output.contains(ACTION_RESULT_PREFIX) && client_output.contains("[0, 1, 1, 2"),
        "Expected the C++ server to return the full Fibonacci result.\nOutput:\n{}",
        client_output
    );
}

// =============================================================================
// C++ goal rejection (Phase 83.15) — C++ only; C action examples don't
// read NROS_TEST_GOAL_ORDER yet.
// =============================================================================

#[rstest]
fn test_cpp_action_goal_rejection(zenohd_unique: ZenohRouter) {
    require_native_env();
    let locator = zenohd_unique.locator();
    let server_bin = Language::Cpp.action_server_binary();
    let client_bin = Language::Cpp.action_client_binary();

    let mut server = spawn_native(&server_bin, Language::Cpp, "action-server", &locator);
    server
        .wait_for_output_pattern(
            nros_tests::output::ACTION_SERVER_READY_MARKER,
            Duration::from_secs(30),
        )
        .expect("cpp-action-server did not become ready");

    let mut client_cmd = stdbuf_command(&client_bin);
    client_cmd.env("NROS_LOCATOR", &locator);
    // Order 100 > 64 triggers the server's goal callback to return
    // `GoalResponse::Reject`.
    client_cmd.env("NROS_TEST_GOAL_ORDER", "100");
    let mut client = ManagedProcess::spawn_command(client_cmd, "cpp-action-client")
        .expect("Failed to start cpp-action-client");

    let client_output = client.collect_until(ACTION_GOAL_REJECTED_PREFIX, Duration::from_secs(20));

    server.kill();
    eprintln!("C++ action client output:\n{}", client_output);

    // Issue 0868 — this assertion used to be satisfied by ANY `send_goal`
    // failure. The example printed the rejection line for every non-OK return,
    // so a timeout (the server never answered) passed this test as if the
    // reject path had run. Now `send_goal` returns `Rejected` only for a
    // server decision and the example branches on it, so the marker is
    // evidence of the reject path rather than of some failure.
    assert!(
        client_output.contains(ACTION_GOAL_REJECTED_PREFIX),
        "Expected goal rejection marker in client output.\nOutput:\n{}",
        client_output
    );
    // The other half of the same guarantee, and the one that keeps this test
    // honest if the branches are ever re-merged: a timeout prints a DIFFERENT
    // line, and seeing it here would mean the goal never reached the server's
    // reject path at all.
    assert!(
        !client_output.contains(ACTION_GOAL_NO_RESPONSE_PREFIX),
        "Client timed out waiting for a goal response — the server's reject \
         path was never exercised, so a passing rejection assertion would be \
         reporting a transport failure as a server decision (issue 0868).\nOutput:\n{}",
        client_output
    );
    // Audit 2026-07-28 finding E7 — this used to be `!contains("[OK]")`, a
    // marker the C++ action client never prints, so the negative assertion
    // could not fail. The audit spotted it and DOWNGRADED it on the grounds
    // that "the positive assertion two lines above carries the test" — which
    // was the one assertion that could not carry it, since the client printed
    // the rejection line for every failure. Both halves are real now:
    // `ACTION_GOAL_ACCEPTED_PREFIX` is what the client actually prints when a
    // goal is accepted, and a rejected client exits before reaching it.
    assert!(
        !client_output.contains(ACTION_GOAL_ACCEPTED_PREFIX),
        "Client reported the goal ACCEPTED on a goal the server rejects.\nOutput:\n{}",
        client_output
    );
}

// =============================================================================
// Cross-language interop (C ↔ Rust, C++ ↔ Rust)
// =============================================================================
//
// These pair a native binary with a Rust binary, so the Language enum can
// only represent the native half. The Rust half is always the same.

fn native_rust_pubsub_interop(lang: Language, locator: &str) {
    let rust_listener = match nros_tests::fixtures::build_native_listener() {
        Ok(p) => p.to_path_buf(),
        Err(e) => {
            nros_tests::skip!("could not build Rust listener: {}", e);
        }
    };

    let mut listener_cmd = Command::new(&rust_listener);
    listener_cmd.env("NROS_LOCATOR", locator);
    listener_cmd.env("RUST_LOG", "info");
    let mut listener = ManagedProcess::spawn_command(listener_cmd, "rust-listener")
        .expect("Failed to start Rust listener");

    // Rust listener logs "Subscriber created" once
    // `create_subscription` succeeds.
    listener
        .wait_for_output_pattern(
            nros_tests::output::LISTENER_READY_MARKER,
            Duration::from_secs(30),
        )
        .expect("rust-listener did not become ready");

    let talker_bin = lang.talker_binary();
    let mut talker = spawn_native(&talker_bin, lang, "talker", locator);

    // phase-342 W8 — wait for the delivery this test ASSERTS, instead of
    // sleeping 6 s and hoping it happened. The condition is the assertion
    // below; a fixed sleep is that condition guessed at, and it costs the full
    // 6 s even when delivery landed in one. On timeout the error carries the
    // collected output, so a failure reads the same as before.
    let listener_output = listener
        .wait_for_output_count(
            nros_tests::output::LISTENER_LOG_PREFIX,
            2,
            Duration::from_secs(20),
        )
        .unwrap_or_else(|e| panic!("listener never received 2 messages: {e}"));
    talker.kill();
    eprintln!(
        "Rust listener output ({} talker):\n{}",
        lang.label(),
        listener_output
    );

    let received_count = count_pattern(&listener_output, nros_tests::output::LISTENER_LOG_PREFIX);
    eprintln!(
        "Rust listener received {} messages from {} talker",
        received_count,
        lang.label()
    );
    assert!(
        received_count >= 2,
        "Expected at least 2 cross-language messages, got {}.\nOutput:\n{}",
        received_count,
        listener_output
    );
}

#[rstest]
fn test_c_rust_pubsub_interop(zenohd_unique: ZenohRouter) {
    require_native_env();
    native_rust_pubsub_interop(Language::C, &zenohd_unique.locator());
}

#[rstest]
fn test_cpp_rust_pubsub_interop(zenohd_unique: ZenohRouter) {
    require_native_env();
    native_rust_pubsub_interop(Language::Cpp, &zenohd_unique.locator());
}

// =============================================================================
// Cyclone DDS cross-language interop (C/C++ ↔ Rust, brokerless RTPS)
// =============================================================================

fn next_cyclonedds_domain() -> String {
    // Cyclone is brokerless RTPS: every participant runs SPDP discovery on the
    // host's interfaces, so two test processes sharing a domain cross-talk —
    // a goal/result gets matched by the wrong server. nextest runs each test in
    // its OWN process, so a process-local atomic counter is NOT unique across
    // concurrent tests: every process restarts it at the same base and they all
    // collide on one domain (observed: a C action server `Executing goal [1]`
    // *and* `[2]` from a concurrent C++ client → the C client's result is stolen
    // and it never reaches `Final result`). Delegate to the PID-seeded allocator
    // that is unique across concurrent nextest processes (concurrently-spawned
    // tests have near-consecutive PIDs, so `(pid % 232) + 1` is collision-free).
    nros_tests::unique_ros_domain_id().to_string()
}

fn cyclone_talker_binary(lang: Language) -> PathBuf {
    match lang {
        Language::C => build_native_c_example_rmw("talker", "c_talker", Rmw::Cyclonedds),
        Language::Cpp => build_native_cpp_example_rmw("talker", "cpp_talker", Rmw::Cyclonedds),
    }
    .unwrap_or_else(|e| skip_missing_fixture("native cyclonedds talker", e))
}

fn cyclone_listener_binary(lang: Language) -> PathBuf {
    match lang {
        Language::C => build_native_c_example_rmw("listener", "c_listener", Rmw::Cyclonedds),
        Language::Cpp => build_native_cpp_example_rmw("listener", "cpp_listener", Rmw::Cyclonedds),
    }
    .unwrap_or_else(|e| skip_missing_fixture("native cyclonedds listener", e))
}

fn rust_cyclone_talker_binary() -> PathBuf {
    build_native_talker_rmw(Rmw::Cyclonedds)
        .unwrap_or_else(|e| skip_missing_fixture("native rust cyclonedds talker", e))
        .to_path_buf()
}

fn rust_cyclone_listener_binary() -> PathBuf {
    build_native_listener_rmw(Rmw::Cyclonedds)
        .unwrap_or_else(|e| skip_missing_fixture("native rust cyclonedds listener", e))
        .to_path_buf()
}

fn spawn_cyclone_binary(binary: &Path, name: &str, domain_id: &str) -> ManagedProcess {
    let mut cmd = stdbuf_command(binary);
    cmd.env("ROS_DOMAIN_ID", domain_id);
    cmd.env("RUST_LOG", "info");
    let cyclone_lib = nros_tests::project_root().join("build/install/lib");
    let ld_library_path = match std::env::var_os("LD_LIBRARY_PATH") {
        Some(existing) if !existing.is_empty() => {
            let mut paths = vec![cyclone_lib];
            paths.extend(std::env::split_paths(&existing));
            std::env::join_paths(paths).expect("valid LD_LIBRARY_PATH")
        }
        _ => cyclone_lib.into_os_string(),
    };
    cmd.env("LD_LIBRARY_PATH", ld_library_path);
    ManagedProcess::spawn_command(cmd, name).unwrap_or_else(|_| panic!("Failed to start {name}"))
}

#[rstest]
fn test_native_cyclonedds_talker_to_rust_listener(
    #[values(Language::C, Language::Cpp)] lang: Language,
) {
    if !require_cmake() {
        nros_tests::skip!("cmake not found");
    }
    let domain_id = next_cyclonedds_domain();
    let listener_bin = rust_cyclone_listener_binary();
    let talker_bin = cyclone_talker_binary(lang);

    let mut listener = spawn_cyclone_binary(&listener_bin, "rust-cyclonedds-listener", &domain_id);
    listener
        .wait_for_output_pattern(
            nros_tests::output::LISTENER_READY_MARKER,
            Duration::from_secs(30),
        )
        .expect("rust cyclonedds listener did not become ready");

    let mut talker = spawn_cyclone_binary(
        &talker_bin,
        &format!("{}-cyclonedds-talker", lang.tag()),
        &domain_id,
    );

    // phase-342 W8 — wait for the delivery this test ASSERTS, instead of
    // sleeping 6 s and hoping it happened. The condition is the assertion
    // below; a fixed sleep is that condition guessed at, and it costs the full
    // 6 s even when delivery landed in one. On timeout the error carries the
    // collected output, so a failure reads the same as before.
    let listener_output = listener
        .wait_for_output_count(
            nros_tests::output::LISTENER_LOG_PREFIX,
            2,
            Duration::from_secs(20),
        )
        .unwrap_or_else(|e| panic!("listener never received 2 messages: {e}"));
    talker.kill();
    eprintln!(
        "Rust Cyclone listener output ({} talker):\n{}",
        lang.label(),
        listener_output
    );

    let received_count = count_pattern(&listener_output, nros_tests::output::LISTENER_LOG_PREFIX);
    assert!(
        received_count >= 2,
        "Expected at least 2 CycloneDDS samples from {} talker, got {}.\nOutput:\n{}",
        lang.label(),
        received_count,
        listener_output
    );
}

#[rstest]
fn test_native_cyclonedds_rust_talker_to_listener(
    #[values(Language::C, Language::Cpp)] lang: Language,
) {
    if !require_cmake() {
        nros_tests::skip!("cmake not found");
    }
    let domain_id = next_cyclonedds_domain();
    let listener_bin = cyclone_listener_binary(lang);
    let talker_bin = rust_cyclone_talker_binary();

    let mut listener = spawn_cyclone_binary(
        &listener_bin,
        &format!("{}-cyclonedds-listener", lang.tag()),
        &domain_id,
    );
    let listener_boot_output = listener
        .wait_for_output_pattern(
            nros_tests::output::LISTENER_READY_MARKER,
            Duration::from_secs(30),
        )
        .expect("cyclonedds listener did not become ready");

    let mut talker = spawn_cyclone_binary(&talker_bin, "rust-cyclonedds-talker", &domain_id);

    // phase-342 W8 — wait for the delivery this test ASSERTS, instead of
    // sleeping 6 s and hoping it happened. The condition is the assertion
    // below; a fixed sleep is that condition guessed at, and it costs the full
    // 6 s even when delivery landed in one. On timeout the error carries the
    // collected output, so a failure reads the same as before.
    let listener_tail = listener
        .wait_for_output_count(
            nros_tests::output::LISTENER_LOG_PREFIX,
            2,
            Duration::from_secs(20),
        )
        .unwrap_or_else(|e| panic!("listener never received 2 messages: {e}"));
    talker.kill();
    let listener_output = listener_boot_output + &listener_tail;
    eprintln!(
        "{} Cyclone listener output (Rust talker):\n{}",
        lang.label(),
        listener_output
    );

    let received_count = count_pattern(&listener_output, nros_tests::output::LISTENER_LOG_PREFIX);
    assert!(
        received_count >= 2,
        "Expected at least 2 CycloneDDS samples from Rust talker, got {}.\nOutput:\n{}",
        received_count,
        listener_output
    );
}

/// Phase 177.26 — ThreadX↔native CycloneDDS interop. A **threadx-linux** node
/// (ThreadX kernel + NetX Duo over NSOS host sockets) talks to a **native**
/// POSIX node over the same Cyclone backend on loopback. Proves the
/// `nano-ros` Cyclone wire (SPDP discovery + `rt/`-prefixed RTPS data) is
/// platform-agnostic: an RTOS node and a native node interoperate with no
/// bridge.
///
/// The threadx-linux talker is fixed to domain 0 by its `config.toml` (it
/// ignores `ROS_DOMAIN_ID`), so the native listener runs on domain 0 too.
/// Domain 0 is free of the auto-allocated test domains (`next_cyclonedds_domain`
/// starts at 40).
///
/// Fixtures: `just cyclonedds setup` + `just threadx_linux build-fixture-extras`
/// (the cyclone fixtures are gated on the Cyclone install).
#[test]
fn test_threadx_linux_cyclonedds_talker_to_native_listener() {
    if !require_cmake() {
        nros_tests::skip!("cmake not found");
    }
    // Phase 186.6.4 — the build/install gate is obsolete: threadx-linux Cyclone
    // self-provisions from source (no host install), so the fixture binary's
    // presence is the real precondition.
    // 287-W6 renamed the output binary `threadx_c_talker` -> `c_talker` (ament
    // shape). Issue #215: this path kept the OLD name and an orphaned museum
    // binary in the never-wiped build dir satisfied the existence check while
    // silently broken — keep the name in sync with the CMake target.
    let talker_bin = nros_tests::fixtures::threadx_linux::build_threadx_cmake_example_rmw(
        "c",
        "talker",
        "c_talker",
        nros_tests::fixtures::Rmw::Cyclonedds,
    )
    .unwrap_or_else(|e| panic!("resolve threadx-linux c talker (cyclonedds): {e}"));
    // Native C Cyclone listener built via the standard fixture path.
    let listener_bin = cyclone_listener_binary(Language::C);

    // domain 107 — the allocator's (threadx-linux, c, pubsub) Cyclone domain
    // (`nros_tests::alloc::domain_of`; `just threadx_linux build-fixtures`
    // bakes the same number via -DNROS_DOMAIN_ID into the cyclonedds
    // fixture). The native listener reads the domain from the env at runtime
    // (host exception), so set it to 107 to pair with the talker.
    let mut listener = spawn_cyclone_binary(&listener_bin, "native-c-cyclonedds-listener", "107");
    listener
        .wait_for_output_pattern(
            nros_tests::output::LISTENER_READY_MARKER,
            Duration::from_secs(30),
        )
        .expect("native cyclonedds listener did not become ready");

    // The threadx-linux talker is embedded: it uses its baked domain (107)
    // and ignores this env arg; passed for parity with the listener.
    let mut talker = spawn_cyclone_binary(&talker_bin, "threadx-linux-cyclonedds-talker", "107");

    // phase-342 W8 — wait for the delivery this test ASSERTS, instead of
    // sleeping 8 s and hoping it happened. The condition is the assertion
    // below; a fixed sleep is that condition guessed at, and it costs the full
    // 8 s even when delivery landed in one. On timeout the error carries the
    // collected output, so a failure reads the same as before.
    let listener_output = listener
        .wait_for_output_count(
            nros_tests::output::LISTENER_LOG_PREFIX,
            2,
            Duration::from_secs(20),
        )
        .unwrap_or_else(|e| panic!("listener never received 2 messages: {e}"));
    talker.kill();
    eprintln!("Native listener output (threadx-linux talker):\n{listener_output}");

    let received_count = count_pattern(&listener_output, nros_tests::output::LISTENER_LOG_PREFIX);
    assert!(
        received_count >= 2,
        "Expected ≥2 CycloneDDS samples from the threadx-linux talker, got {received_count}.\n\
         Output:\n{listener_output}"
    );
}

/// issue #233 cell 4 — threadx-linux **C++** CycloneDDS pubsub: the embedded
/// ThreadX C++ talker feeds a native POSIX C++ listener over Cyclone (the C++
/// sibling of the #215 C pubsub lane; the C++ cyclone image linked BuildOnly).
#[test]
fn test_threadx_linux_cyclonedds_cpp_talker_to_native_listener() {
    if !require_cmake() {
        nros_tests::skip!("cmake not found");
    }
    let talker_bin = nros_tests::fixtures::threadx_linux::build_threadx_cmake_example_rmw(
        "cpp",
        "talker",
        "cpp_talker",
        nros_tests::fixtures::Rmw::Cyclonedds,
    )
    .unwrap_or_else(|e| panic!("resolve threadx-linux cpp talker (cyclonedds): {e}"));
    let listener_bin = cyclone_listener_binary(Language::Cpp);

    let mut listener = spawn_cyclone_binary(&listener_bin, "native-cpp-cyclonedds-listener", "107");
    listener
        .wait_for_output_pattern(
            nros_tests::output::LISTENER_READY_MARKER,
            Duration::from_secs(30),
        )
        .expect("native C++ cyclonedds listener did not become ready");

    let mut talker = spawn_cyclone_binary(&talker_bin, "threadx-cpp-cyclonedds-talker", "107");
    // phase-342 W8 — wait for the delivery this test ASSERTS, instead of
    // sleeping 8 s and hoping it happened. The condition is the assertion
    // below; a fixed sleep is that condition guessed at, and it costs the full
    // 8 s even when delivery landed in one. On timeout the error carries the
    // collected output, so a failure reads the same as before.
    let listener_output = listener
        .wait_for_output_count(
            nros_tests::output::LISTENER_LOG_PREFIX,
            2,
            Duration::from_secs(20),
        )
        .unwrap_or_else(|e| panic!("listener never received 2 messages: {e}"));
    talker.kill();
    eprintln!("Native C++ listener output (threadx-linux C++ talker):\n{listener_output}");

    let received = count_pattern(&listener_output, nros_tests::output::LISTENER_LOG_PREFIX);
    assert!(
        received >= 2,
        "Expected ≥2 CycloneDDS samples from the threadx-linux C++ talker, got {received}.\n\
         Output:\n{listener_output}"
    );
}

/// issue #233 cell 3 — threadx-linux C CycloneDDS **service**: an embedded
/// ThreadX server (NetX Duo over NSOS) answers a native POSIX client over the
/// same Cyclone backend, mirroring the #215 pubsub interop lane. The
/// service/action fixtures always built (BuildOnly in the matrix); this is
/// the runtime proof. Domain 107 matches the threadx cyclone fixture's baked
/// `-DNROS_DOMAIN_ID` (the native client reads it from the env — host
/// exception).
#[test]
fn test_threadx_linux_cyclonedds_service() {
    if !require_cmake() {
        nros_tests::skip!("cmake not found");
    }
    let server_bin = nros_tests::fixtures::threadx_linux::build_threadx_cmake_example_rmw(
        "c",
        "service-server",
        "c_service_server",
        nros_tests::fixtures::Rmw::Cyclonedds,
    )
    .unwrap_or_else(|e| panic!("resolve threadx-linux c service-server (cyclonedds): {e}"));
    let client_bin = cyclone_role_binary(Language::C, "service-client");

    let mut server = spawn_cyclone_binary(&server_bin, "threadx-cyclonedds-service-server", "107");
    let _ = server.wait_for_output_pattern(
        nros_tests::output::SERVICE_SERVER_READY_MARKER,
        Duration::from_secs(30),
    );
    let mut client = spawn_cyclone_binary(&client_bin, "native-cyclonedds-service-client", "107");

    let client_out = client.collect_until(SERVICE_RESULT_PREFIX, Duration::from_secs(30));
    std::thread::sleep(Duration::from_millis(500));
    let server_out = server.collect_until("Incoming request", Duration::from_secs(2));
    client.kill();
    server.kill();

    eprintln!("threadx→native Cyclone service client:\n{client_out}\n--- server ---\n{server_out}");
    let calls = count_pattern(&client_out, SERVICE_RESULT_PREFIX);
    let handled = count_pattern(&server_out, "Incoming request");
    assert!(
        calls >= 1 || handled >= 1,
        "threadx-linux C Cyclone service roundtrip produced no calls/requests.\n\
         client:\n{client_out}\nserver:\n{server_out}"
    );
}

/// issue #233 cell 3 — threadx-linux C CycloneDDS **action**: the embedded
/// ThreadX Fibonacci server drives a native POSIX action client to a full
/// order-10 result over Cyclone (unlike the pure-rust cyclone action path,
/// C carries the action-type descriptors via `descriptors.cpp`, so creation
/// succeeds).
#[test]
fn test_threadx_linux_cyclonedds_action() {
    if !require_cmake() {
        nros_tests::skip!("cmake not found");
    }
    let server_bin = nros_tests::fixtures::threadx_linux::build_threadx_cmake_example_rmw(
        "c",
        "action-server",
        "c_action_server",
        nros_tests::fixtures::Rmw::Cyclonedds,
    )
    .unwrap_or_else(|e| panic!("resolve threadx-linux c action-server (cyclonedds): {e}"));
    let client_bin = cyclone_role_binary(Language::C, "action-client");

    let mut server = spawn_cyclone_binary(&server_bin, "threadx-cyclonedds-action-server", "107");
    let _ = server.wait_for_output_pattern(ACTION_SERVER_READY_MARKER, Duration::from_secs(30));
    let mut client = spawn_cyclone_binary(&client_bin, "native-cyclonedds-action-client", "107");

    let client_out = client.collect_until(ACTION_RESULT_PREFIX, Duration::from_secs(40));
    client.kill();
    server.kill();

    eprintln!("threadx→native Cyclone action client:\n{client_out}");
    let results = count_pattern(&client_out, ACTION_RESULT_PREFIX);
    assert!(
        results >= 1,
        "threadx-linux C Cyclone action produced no result.\nclient:\n{client_out}"
    );
}

fn native_rust_service_interop(lang: Language, locator: &str) {
    let rust_client = match nros_tests::fixtures::build_native_service_client() {
        Ok(p) => p.to_path_buf(),
        Err(e) => {
            nros_tests::skip!("could not build Rust service client: {}", e);
        }
    };

    let server_bin = lang.service_server_binary();
    let mut server = spawn_native(&server_bin, lang, "service-server", locator);
    server
        .wait_for_output_pattern(
            nros_tests::output::SERVICE_SERVER_READY_MARKER,
            Duration::from_secs(30),
        )
        .expect("native service server did not become ready");
    assert!(
        server.is_running(),
        "{} service server died during startup",
        lang.label()
    );

    let mut client_cmd = Command::new(&rust_client);
    client_cmd.env("NROS_LOCATOR", locator);
    client_cmd.env("RUST_LOG", "info");
    let mut client = ManagedProcess::spawn_command(client_cmd, "rust-service-client")
        .expect("Failed to start Rust service client");

    let client_output = client.collect_until(SERVICE_RESULT_PREFIX, Duration::from_secs(30));

    // issue 1026 — a DIAGNOSTIC drain, and deliberately left as one: nothing
    // below asserts on `server_output`, it is only `eprintln!`ed beside the
    // client's, and by this point the client has already printed its result so
    // the server's side of the exchange is in the pipe. The 2 s is therefore a
    // bound on the PRINTOUT, not on anything observed. The kill it performs is
    // the teardown this test wanted anyway.
    let server_output = server
        .wait_for_all_output(Duration::from_secs(2))
        .unwrap_or_default();

    eprintln!("{} service server output:\n{}", lang.label(), server_output);
    eprintln!(
        "Rust client output ({} server):\n{}",
        lang.label(),
        client_output
    );

    assert!(
        client_output.contains(&service_result_line(5)),
        "Expected the Rust client's `{}` from the {} server.\nOutput:\n{}",
        service_result_line(5),
        lang.label(),
        client_output
    );
}

#[rstest]
fn test_c_rust_service_interop(zenohd_unique: ZenohRouter) {
    require_native_env();
    native_rust_service_interop(Language::C, &zenohd_unique.locator());
}

#[rstest]
fn test_cpp_rust_service_interop(zenohd_unique: ZenohRouter) {
    require_native_env();
    native_rust_service_interop(Language::Cpp, &zenohd_unique.locator());
}

// =============================================================================
// Native CycloneDDS service + action E2E (Phase 183.4)
//
// native/{c,cpp} cyclonedds ship 6 cases but only pub/sub had an e2e
// (test_native_cyclonedds_*_talker_to_listener). These add the service +
// action roundtrips for C and C++ — the CMake/Corrosion `build-cyclonedds/`
// binaries (Phase 175) on a per-test ROS_DOMAIN_ID over SPDP discovery,
// pinning the Cyclone C++ action get_result/feedback path (28e9e6502 +
// Phase 171.0.b). Skip cleanly when `just cyclonedds setup` hasn't built them.
// =============================================================================

/// Resolve a native Cyclone C/C++ example role binary (prebuilt), or skip.
fn cyclone_role_binary(lang: Language, case: &str) -> PathBuf {
    let snake = case.replace('-', "_");
    match lang {
        Language::C => {
            let bin = format!("c_{snake}");
            build_native_c_example_rmw(case, &bin, Rmw::Cyclonedds)
        }
        Language::Cpp => {
            let bin = format!("cpp_{snake}");
            build_native_cpp_example_rmw(case, &bin, Rmw::Cyclonedds)
        }
    }
    .unwrap_or_else(|e| {
        skip_missing_fixture(&format!("native {} cyclonedds {case}", lang.label()), e)
    })
}

/// issue #233 cell 1 — native **Rust** CycloneDDS service pair.
///
/// The Rust cyclone SERVICE path was flagged BuildOnly-unproven by the
/// phase-295 matrix (the C/Cpp cyclone-service rstest above never covered
/// Rust — the typed `create_service::<M>` over cyclone needs the
/// `nros/rmw-cyclonedds` marker, #67). This proves it delivers: server sees
/// the request, client prints `Result of add_two_ints:`. Separate from the
/// C/Cpp rstest because Rust resolves via `build_native_rust_example_rmw`
/// (target-cyclonedds), not the CMake `cyclone_role_binary`.
///
/// The Rust cyclone ACTION pair is deliberately NOT wired: it fails at
/// creation (`ActionCreationFailed`) — the typed-action-descriptor gap the
/// C/C++ `descriptors.cpp` fills but the pure-rust path does not. Tracked in
/// issue #233.
#[rstest]
fn test_native_cyclonedds_rust_service() {
    if !require_cmake() {
        nros_tests::skip!("cmake not found");
    }
    let domain = next_cyclonedds_domain();
    let server_bin = nros_tests::fixtures::build_native_rust_example_rmw(
        "service-server",
        "service-server",
        Rmw::Cyclonedds,
    )
    .unwrap_or_else(|e| skip_missing_fixture("native rust cyclonedds service-server", e));
    let client_bin = nros_tests::fixtures::build_native_rust_example_rmw(
        "service-client",
        "service-client",
        Rmw::Cyclonedds,
    )
    .unwrap_or_else(|e| skip_missing_fixture("native rust cyclonedds service-client", e));

    let mut server = spawn_cyclone_binary(&server_bin, "rust-cyclonedds-service-server", &domain);
    let _ = server.wait_for_output_pattern(
        nros_tests::output::SERVICE_SERVER_READY_MARKER,
        Duration::from_secs(30),
    );
    let mut client = spawn_cyclone_binary(&client_bin, "rust-cyclonedds-service-client", &domain);

    let client_out = client.collect_until(SERVICE_RESULT_PREFIX, Duration::from_secs(30));
    std::thread::sleep(Duration::from_millis(500));
    let server_out = server.collect_until("Incoming request", Duration::from_secs(2));
    client.kill();
    server.kill();

    eprintln!("Rust Cyclone service client:\n{client_out}\n--- server ---\n{server_out}");
    let calls = count_pattern(&client_out, SERVICE_RESULT_PREFIX);
    let handled = count_pattern(&server_out, "Incoming request");
    assert!(
        calls >= 1 || handled >= 1,
        "Rust Cyclone service roundtrip produced no calls/requests.\nclient:\n{client_out}\nserver:\n{server_out}"
    );
}

/// issue #234 — native **Rust** CycloneDDS action pair (Fibonacci).
///
/// The rust cyclone action pair previously failed at CREATION
/// (`ActionCreationFailed`), before any goal: `RosAction::register_protocol_types`
/// registered the `action_msgs` protocol descriptors (`CancelGoal_{Request,
/// Response}`, `GoalStatusArray`) through a `#[cfg(feature = "rmw-cyclonedds")]`-gated
/// `nros_rmw_cyclonedds::register::<M>()` call — but the example build never turned
/// that generated-crate feature on, so the block compiled out and the cancel-service
/// / status-publisher sub-creates found no descriptor. The codegen now routes those
/// three registrations through the generic `nros_rmw::register_type_descriptor` seam
/// (the same one the 8 action envelopes already use via `register_type::<M>()` in
/// nros-node), which forwards to Cyclone's runtime type registry when linked and is a
/// no-op otherwise. This proves the pair now delivers: the server sees the goal, the
/// client prints the order-10 Fibonacci `Result received: [...]`.
///
/// Mirrors `test_native_cyclonedds_rust_service`: Rust cyclone fixtures resolve via
/// `build_native_rust_example_rmw` (target-cyclonedds), not the CMake
/// `cyclone_role_binary`.
#[rstest]
fn test_native_cyclonedds_rust_action() {
    if !require_cmake() {
        nros_tests::skip!("cmake not found");
    }
    let domain = next_cyclonedds_domain();
    let server_bin = nros_tests::fixtures::build_native_rust_example_rmw(
        "action-server",
        "action-server",
        Rmw::Cyclonedds,
    )
    .unwrap_or_else(|e| skip_missing_fixture("native rust cyclonedds action-server", e));
    let client_bin = nros_tests::fixtures::build_native_rust_example_rmw(
        "action-client",
        "action-client",
        Rmw::Cyclonedds,
    )
    .unwrap_or_else(|e| skip_missing_fixture("native rust cyclonedds action-client", e));

    let mut server = spawn_cyclone_binary(&server_bin, "rust-cyclonedds-action-server", &domain);
    let _ = server.wait_for_output_pattern(ACTION_SERVER_READY_MARKER, Duration::from_secs(30));
    let mut client = spawn_cyclone_binary(&client_bin, "rust-cyclonedds-action-client", &domain);

    let client_out = client.collect_until(ACTION_RESULT_PREFIX, Duration::from_secs(40));
    std::thread::sleep(Duration::from_millis(500));
    let server_out = server.collect_until(ACTION_GOAL_REQUEST_PREFIX, Duration::from_secs(2));
    client.kill();
    server.kill();

    eprintln!("Rust Cyclone action client:\n{client_out}\n--- server ---\n{server_out}");
    assert!(
        client_out.contains(ACTION_RESULT_PREFIX),
        "Rust Cyclone action client did not reach a result (expected `{ACTION_RESULT_PREFIX}`).\nclient:\n{client_out}\nserver:\n{server_out}"
    );
}

/// Native CycloneDDS service server ↔ **callback** client (RFC-0041 / Phase 239.8).
///
/// Backend-parity check: the callback receive model is structurally
/// transport-agnostic (one `drive_io` per spin, non-blocking drain); this proves
/// it empirically on a non-zenoh backend — the reply is dispatched to the
/// spin-time callback over CycloneDDS, no `Promise::wait` budget-burn.
#[rstest]
fn test_native_cyclonedds_service_callback(#[values(Language::C, Language::Cpp)] lang: Language) {
    if !require_cmake() {
        nros_tests::skip!("cmake not found");
    }
    let domain = next_cyclonedds_domain();
    let server_bin = cyclone_role_binary(lang, "service-server");
    let client_bin = cyclone_role_binary(lang, "service-client-callback");

    let mut server = spawn_cyclone_binary(
        &server_bin,
        &format!("{}-cyclonedds-service-server", lang.tag()),
        &domain,
    );
    let _ = server.wait_for_output_pattern(
        nros_tests::output::SERVICE_SERVER_READY_MARKER,
        Duration::from_secs(30),
    );
    let mut client = spawn_cyclone_binary(
        &client_bin,
        &format!("{}-cyclonedds-service-client-callback", lang.tag()),
        &domain,
    );

    let client_out = client.collect_until(SERVICE_RESULT_PREFIX, Duration::from_secs(30));
    client.kill();
    server.kill();

    eprintln!(
        "{} Cyclone callback service client:\n{client_out}",
        lang.label()
    );
    // Replies dispatched via the spin-time callback over CycloneDDS.
    let cb = count_pattern(&client_out, SERVICE_RESULT_PREFIX);
    assert!(
        cb >= 1,
        "{} Cyclone callback service roundtrip produced no callback-dispatched replies.\nclient:\n{client_out}",
        lang.label()
    );
}
