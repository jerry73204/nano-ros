//! ESP32-C3 QEMU emulator tests
//!
//! **Bucket (phase-329 W4): KEEP — sole ESP32-C3 coverage, heavy prereqs, not
//! matrix-cell dups.** No cell-bound consumer runs the Espressif QEMU fork
//! (`qemu-system-riscv32 -M esp32c3` + espflash + the RISC-V zenoh-pico), so this
//! file is the only runtime coverage of the esp32 target — nothing duplicates it.
//! Kept as one-offs: the BSP boot banner test, the local talker→listener e2e, the
//! ESP32↔native INTEROP pair (`test_esp32_to_native` / `test_native_to_esp32`),
//! the workspace-entry e2e, and the toolchain/qemu/espflash detection probes. The
//! two interop tests are the fold candidates for `interop::CELLS` IF esp32 later
//! joins that matrix (its `PlatformId` + a peer/dir binding); until then they stay
//! here, bound to the esp32 fixtures the interop harness does not build.
//! No deletions: sole platform coverage, un-run-provable folds on a host without
//! the Espressif fork.
//!
//! Run with: `just test-qemu-esp32`
//!
//! ## Test Groups
//!
//! - **Build tests**: Verify `cargo +nightly build --release` succeeds (no QEMU needed)
//! - **Boot test**: Verify BSP banner appears on UART (QEMU needed, no networking)
//! - **Networked E2E**: Talker → listener via zenohd + slirp user-mode networking
//!
//! ## Prerequisites
//!
//! - Build tests: nightly toolchain + riscv32imc target + zenoh-pico RISC-V
//! - Boot test: + qemu-system-riscv32 (Espressif fork) + espflash
//! - Networked E2E: + zenohd (slirp networking, no TAP/bridge setup needed)

use nros_tests::{
    alloc::port_of,
    count_pattern,
    esp32::*,
    fixtures::{
        ManagedProcess, ZenohRouter, build_esp32_qemu_listener, build_esp32_qemu_talker,
        build_int32_sink, build_native_listener, build_native_talker,
        get_prebuilt_esp32_qemu_workspace_entry, require_zenohd,
    },
    output::LISTENER_READY_MARKER,
    platform, wait_for_port,
};
use std::{process::Command, time::Duration};

// =============================================================================
// (Phase 182.3) `test_esp32_qemu_{talker,listener}_builds` removed — they only
// asserted the riscv32 fixture compiled, covered by `build-all` (Phase 181.4.h)
// + the boot / talker-listener e2e tests below (which build the same binaries
// via the shared `build_esp32_qemu_*` resolvers).
// =============================================================================

// =============================================================================
// Boot Test (QEMU needed, no networking)
// =============================================================================

/// Verify ESP32-C3 boots and shows BSP banner on UART
#[test]
fn test_esp32_qemu_talker_boots() {
    if !require_riscv32_target() {
        nros_tests::skip!("riscv32 target not available");
    }

    if !require_qemu_riscv32() {
        nros_tests::skip!("qemu-system-riscv32 not available");
    }

    if !require_espflash() {
        nros_tests::skip!("espflash not available");
    }

    let elf = build_esp32_qemu_talker().expect("Failed to build esp32-qemu-talker");

    // Create flash image
    // issue 0535 — the packed image has no manifest row (it is a postprocess of
    // one), so the KIND is what this test and `just esp32 build-qemu` share.
    let flash_image =
        nros_tests::build_dir(nros_tests::kind::ESP32_QEMU, &[]).join("esp32-qemu-talker.bin");
    create_esp32_flash_image(elf, &flash_image).expect("Failed to create flash image");

    // Boot without networking
    let mut qemu = start_esp32_qemu(&flash_image, false).expect("Failed to start ESP32-C3 QEMU");

    let output = qemu
        .wait_for_output_pattern("nros ESP32-C3 QEMU Platform", Duration::from_secs(30))
        .expect("QEMU timed out waiting for platform banner");

    assert!(
        output.contains("nros ESP32-C3 QEMU Platform"),
        "Expected platform banner in output.\nOutput:\n{}",
        output
    );

    eprintln!("SUCCESS: ESP32-C3 QEMU boots and shows platform banner");
}

// =============================================================================
// Networked E2E Tests (QEMU + slirp + zenohd)
// =============================================================================
//
// These tests require:
// - qemu-system-riscv32, espflash, nightly toolchain
// - zenohd on the allocator's esp32 pubsub port (`platform::ESP32.zenohd_port`)
//
// Each QEMU instance has its own isolated 10.0.2.0/24 slirp network.
// Firmware connects to zenohd via slirp gateway 10.0.2.2:<port> → host loopback.
// No TAP bridge setup is needed.
//
// ESP32 examples use IPs 10.0.2.50 (talker) and 10.0.2.51 (listener).
//
// Ordering:
//   1. Start zenohd, verify reachable on the host loopback
//   2. Start listener, wait for subscription
//   3. Start talker, wait for publish completion
//   4. Verify listener received messages

/// Helper: build and create flash images for talker and listener
fn build_esp32_flash_images() -> (std::path::PathBuf, std::path::PathBuf) {
    let talker_elf = build_esp32_qemu_talker().expect("Failed to build esp32-qemu-talker");
    let listener_elf = build_esp32_qemu_listener().expect("Failed to build esp32-qemu-listener");

    let esp32_qemu = nros_tests::build_dir(nros_tests::kind::ESP32_QEMU, &[]);
    let talker_bin = esp32_qemu.join("esp32-qemu-talker.bin");
    let listener_bin = esp32_qemu.join("esp32-qemu-listener.bin");

    create_esp32_flash_image(talker_elf, &talker_bin).expect("Failed to create talker flash image");
    create_esp32_flash_image(listener_elf, &listener_bin)
        .expect("Failed to create listener flash image");

    (talker_bin, listener_bin)
}

/// Check all prerequisites for networked ESP32 tests
fn require_esp32_networked() -> bool {
    if !require_riscv32_target() {
        return false;
    }
    if !require_qemu_riscv32() {
        return false;
    }
    if !require_espflash() {
        return false;
    }
    if !require_zenohd() {
        return false;
    }
    true
}

/// Test ESP32 talker → ESP32 listener end-to-end
///
/// Each QEMU instance has its own isolated slirp network (10.0.2.0/24):
/// - Talker:   IP 10.0.2.50
/// - Listener: IP 10.0.2.51
///
/// Both connect to zenohd via the slirp gateway on the baked pubsub port.
///
/// KNOWN-RED (re-diagnosed 2026-06-15). The old "OpenETH smoltcp never emits
/// the final ACK / handshake stalls" note is STALE: the listener now reaches
/// `Waiting for messages...` (Executor::open + subscribe succeed), so the TCP
/// handshake + zenoh session open work. The real failure is a firmware CPU
/// exception during session init, decoded from the QEMU backtrace:
///
///   Exception 'Load access fault' mtval=0xffffffff
///     libc_stubs::strlen                       (a load off a 0xffffffff ptr)
///     <- zenoh-pico _z_str_size / _z_str_clone  (string.c:165/189)
///     <- _zp_config_insert                      (config.c:36)
///     <- zpico_init_with_config                 (zpico.c:833)
///     <- nros_rmw_zenoh::zpico::Context::with_config (zpico.rs:347)
///
/// i.e. a config-string `value` handed to zenoh-pico's config intmap is a
/// garbage pointer (0xffffffff). Intermittent (the node sometimes connects),
/// esp32-c3 only — the identical `with_config` path is green on
/// freertos/threadx/native — so it is memory corruption local to the
/// bare-metal session-init path, NOT a networking/OpenETH RX-TX issue.
///
/// Not yet root-caused (needs instrumented QEMU runs, ~250 s each). Ruled out
/// so far: stale global `g_config` (zpico.c re-runs `z_config_default` per
/// call); a non-NUL-terminated locator/property value (all are NUL-terminated
/// stack buffers in `SmoltcpSession::new`); a too-small main stack (~18 KB:
/// `_stack_start` 0x3fcce400 − `_stack_end` 0x3fcc9a4c — the ~4.2 KB
/// `SmoltcpSession::new` frame, key_bufs/val_bufs 2×256×8, is large but the
/// fault is a deref of an all-ones *pointer value*, not a run-off-into-guard).
/// Open leads: the `connect_with_retry` closure re-invoking
/// `zpico_init_with_config` (re-entrancy over shared globals), or esp32 heap
/// corruption from `z_malloc` in the config-clone path. Tracked as the esp32
/// embedded-harness residual.

#[test]
fn test_esp32_talker_listener_e2e() {
    if !require_esp32_networked() {
        nros_tests::skip!("require_esp32_networked check failed");
    }

    let (talker_bin, listener_bin) = build_esp32_flash_images();

    // Start zenohd on the baked pubsub port (kills any orphaned zenohd first)
    let _router =
        nros_tests::fixtures::or_skip(ZenohRouter::start_slirp(platform::ESP32.zenohd_port));

    // Verify zenohd is reachable on localhost
    assert!(
        wait_for_port(platform::ESP32.zenohd_port, Duration::from_secs(10)),
        "zenohd not reachable on localhost:{}",
        platform::ESP32.zenohd_port
    );

    // Step 1: Start listener
    let mut listener =
        start_esp32_qemu(&listener_bin, true).expect("Failed to start ESP32 listener");

    // Wait for the listener to connect and subscribe: the readiness marker
    // implies `Executor::open` + subscription declaration succeeded (the
    // examples abort on transport failure before this line).
    //
    // Issue 0489 — this greped the LITERAL "Waiting for messages...", which the
    // ESP32 listener does not print and, since phase-342 W7 converged the
    // examples, no listener prints. The image was booting fine and announcing
    // `Subscriber created for topic: /chatter`; the test burned its whole 60 s
    // and reported "failed to start". Use the shared constant, which
    // `example_output_conformance` gates every example against.
    let listener_startup = listener
        .wait_for_output_pattern(LISTENER_READY_MARKER, Duration::from_secs(60))
        .expect("ESP32 listener failed to start (check zenoh connection)");
    eprintln!("Listener connected and subscribed");

    // Step 2: Network stabilization — give listener time to register
    // subscription with zenohd before talker starts publishing
    std::thread::sleep(Duration::from_secs(5));

    // Step 3: Start talker
    let mut talker = start_esp32_qemu(&talker_bin, true).expect("Failed to start ESP32 talker");

    // Wait for talker to publish messages (examples now run forever).
    // Reaching the first nros_tests::output::INT32_TALKER_LOG_PREFIX implies session open + publisher
    // declaration succeeded.
    let talker_output = talker
        .wait_for_output_pattern(
            nros_tests::output::TALKER_LOG_PREFIX,
            Duration::from_secs(60),
        )
        .expect("ESP32 talker timed out waiting for publish");

    let published_count = count_pattern(&talker_output, nros_tests::output::TALKER_LOG_PREFIX);
    eprintln!("Talker published {} messages", published_count);

    // Step 4: Wait for listener to receive messages
    let listener_output = listener.collect_until(
        nros_tests::output::LISTENER_LOG_PREFIX,
        Duration::from_secs(30),
    );

    let all_output = format!("{}{}", listener_startup, listener_output);
    let received_count = count_pattern(&all_output, nros_tests::output::LISTENER_LOG_PREFIX);
    eprintln!("Listener received {} messages", received_count);

    assert!(
        received_count >= 1,
        "Expected listener to receive at least 1 message, got {}.\nOutput:\n{}",
        received_count,
        all_output
    );

    eprintln!("SUCCESS: ESP32 talker → ESP32 listener E2E works");
}

// =============================================================================
// Cross-Architecture Interop Tests (ESP32 QEMU ↔ native)
// =============================================================================
//
// These tests verify that ESP32 QEMU examples can communicate with native
// nros examples via CDR-encoded Int32 on the /chatter ROS 2 topic.
//
// Both ESP32 and native processes connect to the same zenohd on the baked
// pubsub port:
// - ESP32 via the slirp gateway at 10.0.2.2:<port> → host loopback
// - Native via 127.0.0.1:<port>

/// Helper: build ESP32 talker flash image only
fn build_esp32_talker_flash() -> std::path::PathBuf {
    let talker_elf = build_esp32_qemu_talker().expect("Failed to build esp32-qemu-talker");
    let talker_bin =
        nros_tests::build_dir(nros_tests::kind::ESP32_QEMU, &[]).join("esp32-qemu-talker.bin");
    create_esp32_flash_image(talker_elf, &talker_bin).expect("Failed to create talker flash image");
    talker_bin
}

/// Helper: build ESP32 listener flash image only
fn build_esp32_listener_flash() -> std::path::PathBuf {
    let listener_elf = build_esp32_qemu_listener().expect("Failed to build esp32-qemu-listener");
    let listener_bin =
        nros_tests::build_dir(nros_tests::kind::ESP32_QEMU, &[]).join("esp32-qemu-listener.bin");
    create_esp32_flash_image(listener_elf, &listener_bin)
        .expect("Failed to create listener flash image");
    listener_bin
}

/// Test ESP32 talker → native listener cross-architecture interop
///
/// ESP32 publishes CDR Int32 on /chatter via slirp network,
/// native listener receives on localhost.

#[test]
fn test_esp32_to_native() {
    if !require_esp32_networked() {
        nros_tests::skip!("require_esp32_networked check failed");
    }

    // Only need talker flash + native listener
    let talker_bin = build_esp32_talker_flash();
    let native_listener = build_native_listener().expect("Failed to build native listener");

    // Start zenohd on the baked pubsub port (kills any orphaned zenohd first)
    let _router =
        nros_tests::fixtures::or_skip(ZenohRouter::start_slirp(platform::ESP32.zenohd_port));

    // Verify zenohd is reachable on localhost
    assert!(
        wait_for_port(platform::ESP32.zenohd_port, Duration::from_secs(10)),
        "zenohd not reachable on localhost:{}",
        platform::ESP32.zenohd_port
    );

    // Start native listener on localhost (connects to same zenohd)
    let mut listener_cmd = Command::new(native_listener);
    listener_cmd
        .env(
            "NROS_LOCATOR",
            format!("tcp/127.0.0.1:{}", platform::ESP32.zenohd_port),
        )
        .env("RUST_LOG", "info");
    let mut native_proc = ManagedProcess::spawn_command(listener_cmd, "native-rs-listener")
        .expect("Failed to start native listener");

    // Wait for native listener to be ready
    native_proc
        .wait_for_output_pattern(
            nros_tests::output::LISTENER_READY_MARKER,
            Duration::from_secs(10),
        )
        .unwrap_or_else(|e| panic!("native listener never signalled readiness: {e}"));

    // Stabilization delay
    std::thread::sleep(Duration::from_secs(5));

    // Start ESP32 talker
    let mut talker = start_esp32_qemu(&talker_bin, true).expect("Failed to start ESP32 talker");

    // Wait for ESP32 talker to publish messages. The first nros_tests::output::INT32_TALKER_LOG_PREFIX
    // implies the session opened and the publisher was declared.
    let _talker_output = talker
        .wait_for_output_pattern(
            nros_tests::output::TALKER_LOG_PREFIX,
            Duration::from_secs(60),
        )
        .expect("ESP32 talker timed out");

    // Wait for native listener to receive messages
    let listener_output = native_proc.collect_until(
        nros_tests::output::LISTENER_LOG_PREFIX,
        Duration::from_secs(15),
    );

    let received_count = count_pattern(&listener_output, nros_tests::output::LISTENER_LOG_PREFIX);
    eprintln!(
        "Native listener received {} messages from ESP32 talker",
        received_count
    );

    assert!(
        received_count >= 1,
        "Expected native listener to receive at least 1 message from ESP32, got {}.\nListener output:\n{}",
        received_count,
        listener_output
    );

    eprintln!("SUCCESS: ESP32 talker → native listener interop works");
}

/// Test native talker → ESP32 listener cross-architecture interop
///
/// Native publishes CDR Int32 on /chatter via localhost,
/// ESP32 listener receives via slirp network.

#[test]
fn test_native_to_esp32() {
    if !require_esp32_networked() {
        nros_tests::skip!("require_esp32_networked check failed");
    }

    // Only need listener flash + native talker
    let listener_bin = build_esp32_listener_flash();
    let native_talker = build_native_talker().expect("Failed to build native talker");

    // Start zenohd on the baked pubsub port (kills any orphaned zenohd first)
    let _router =
        nros_tests::fixtures::or_skip(ZenohRouter::start_slirp(platform::ESP32.zenohd_port));

    // Verify zenohd is reachable on localhost
    assert!(
        wait_for_port(platform::ESP32.zenohd_port, Duration::from_secs(10)),
        "zenohd not reachable on localhost:{}",
        platform::ESP32.zenohd_port
    );

    // Start ESP32 listener
    let mut esp32_listener =
        start_esp32_qemu(&listener_bin, true).expect("Failed to start ESP32 listener");

    // Wait for the ESP32 listener to connect and subscribe — the readiness
    // marker implies the session opened and the subscription was declared.
    // Issue 0489: was the stale literal "Waiting for messages...". See above.
    let listener_startup = esp32_listener
        .wait_for_output_pattern(LISTENER_READY_MARKER, Duration::from_secs(60))
        .expect("ESP32 listener failed to start");

    // Stabilization delay
    std::thread::sleep(Duration::from_secs(5));

    // Start native talker on localhost (publishes every 1s)
    let mut talker_cmd = Command::new(native_talker);
    talker_cmd
        .env(
            "NROS_LOCATOR",
            format!("tcp/127.0.0.1:{}", platform::ESP32.zenohd_port),
        )
        .env("RUST_LOG", "info");
    let mut native_proc = ManagedProcess::spawn_command(talker_cmd, "native-rs-talker")
        .expect("Failed to start native talker");

    // Wait for native talker to start publishing
    native_proc
        .wait_for_output_pattern(
            nros_tests::output::TALKER_LOG_PREFIX,
            Duration::from_secs(10),
        )
        .unwrap_or_else(|e| panic!("native talker never started publishing: {e}"));

    // Wait for ESP32 listener to receive messages
    let listener_output = esp32_listener.collect_until(
        nros_tests::output::LISTENER_LOG_PREFIX,
        Duration::from_secs(30),
    );

    let all_output = format!("{}{}", listener_startup, listener_output);
    let received_count = count_pattern(&all_output, nros_tests::output::LISTENER_LOG_PREFIX);
    eprintln!(
        "ESP32 listener received {} messages from native talker",
        received_count
    );

    assert!(
        received_count >= 1,
        "Expected ESP32 listener to receive at least 1 message from native, got {}.\nESP32 output:\n{}",
        received_count,
        all_output
    );

    eprintln!("SUCCESS: native talker → ESP32 listener interop works");
}

// =============================================================================
// Workspace Entry E2E (Phase 225.O)
// =============================================================================
//
// The workspace Entry (`examples/workspaces/rust/src/esp32_entry`) is the
// ESP32-C3 QEMU sibling of the native / FreeRTOS / ThreadX / Zephyr
// workspace Entries: a SINGLE bare-metal image hosting the whole
// launch-defined node set — talker AND listener — in one process via
// `nros::main!(launch = "demo_bringup:system.launch.xml")`. Built by the
// 225.O workspace lane and resolved here through
// `get_prebuilt_esp32_qemu_workspace_entry()` (tests run prebuilt
// workspace fixtures, never build them in-body). Since the phase-295 W4
// re-bake the Entry bakes its OWN allocator slot
// (`alloc::port_of(Esp32Qemu, Rust, EntryPubsub)`), so this e2e no longer
// shares a router with the talker/listener pubsub lanes.
//
// Single-session caveat: zenoh does NOT loop a session's own publications
// back to a subscriber in that same session, so the Entry's in-process
// listener cannot observe the in-process talker. We therefore assert
// delivery to a SECOND, EXTERNAL native listener — the same shape as the
// Zephyr workspace Entry E2E — which is a real cross-process pub/sub
// observation of `std_msgs/Int32` on `/chatter`. The Entry's baked
// locator, the external listener's `NROS_LOCATOR`, and the zenohd router
// all use the allocator's esp32 EntryPubsub slot.

/// The allocator's (esp32, rust, EntryPubsub) slot — the ws Entry's baked
/// locator port (`examples/workspaces/rust/src/esp32_entry` deploy
/// metadata bakes the same number).
const ESP32_WS_ENTRY_PORT: u16 = port_of(
    nros_tests::matrix::PlatformId::Esp32Qemu,
    nros_tests::matrix::Lang::Rust,
    nros_tests::matrix::Workload::EntryPubsub,
);

/// ESP32-C3 QEMU workspace Entry boots, brings up its launch node set
/// (talker + listener in one image), and its `/chatter` publications are
/// delivered cross-process to an external native listener.
#[test]
fn test_esp32_workspace_entry_e2e() {
    if !require_esp32_networked() {
        nros_tests::skip!("require_esp32_networked check failed");
    }

    // Resolve the prebuilt workspace-Entry ELF + pack a flash image.
    let entry_elf = get_prebuilt_esp32_qemu_workspace_entry().expect(
        "Failed to resolve prebuilt ESP32 workspace Entry — run `just esp32 build-fixtures` first",
    );
    let entry_bin =
        nros_tests::build_dir(nros_tests::kind::ESP32_QEMU, &[]).join("esp32-ws-entry.bin");
    create_esp32_flash_image(&entry_elf, &entry_bin)
        .expect("Failed to create workspace Entry flash image");

    // zenohd on the Entry's own allocator slot; its baked locator points here.
    let _router = nros_tests::fixtures::or_skip(ZenohRouter::start_slirp(ESP32_WS_ENTRY_PORT));
    assert!(
        wait_for_port(ESP32_WS_ENTRY_PORT, Duration::from_secs(10)),
        "zenohd not reachable on localhost:{}",
        ESP32_WS_ENTRY_PORT
    );

    // External native listener — the observable delivery endpoint (the
    // Entry's own in-process listener sees nothing; no same-session
    // zenoh loopback).
    // phase-338 W3 — was the EXAMPLE (`native/rust/listener`) with a test-only
    // `NROS_SUB_TYPE=int32` switch. That switch moved to `bins/int32-sink`,
    // which subscribes Int32 natively, so the example can drop its branching.
    // Readiness is unchanged ("Waiting for"); the per-message marker is not —
    // the sink prints "Received:", the example printed "I heard:".
    // Issue 1112 — `.expect`, not `skip!`. An absent in-lane fixture is a HARD
    // FAILURE (issue 0584): a gated run has already asserted this lane's
    // fixtures are built, so "not prebuilt" here means the LANE is wrong, and a
    // skip reports that as coverage. This site was the laundering the
    // skip-budget check kept naming ("Something is still laundering the
    // resolver's Err into a [SKIPPED]") — its own two sibling tests twenty
    // lines up already `.expect()` their native peers.
    let native_listener = build_int32_sink()
        .expect("Failed to build int32-sink")
        .to_path_buf();
    let mut listener_cmd = Command::new(native_listener);
    listener_cmd
        .env(
            "NROS_LOCATOR",
            format!("tcp/127.0.0.1:{}", ESP32_WS_ENTRY_PORT),
        )
        // #190 — the workspace Entry's talker_pkg publishes std_msgs/Int32 on
        // /chatter; the message type is baked into the wire keyexpr, so a
        // String subscription matches NOTHING (pcap: the Entry declares
        // `0/chatter/std_msgs::msg::dds_::Int32_/…`). Hence the Int32 sink.
        .env("RUST_LOG", "info");
    let mut native_proc = ManagedProcess::spawn_command(listener_cmd, "native-rs-listener")
        .expect("Failed to start native listener");
    native_proc
        .wait_for_output_pattern(
            nros_tests::output::LISTENER_READY_MARKER,
            Duration::from_secs(10),
        )
        .unwrap_or_else(|e| panic!("native listener never signalled readiness: {e}"));
    std::thread::sleep(Duration::from_secs(5));

    // Boot the single-process Entry (talker + listener).
    let mut entry =
        start_esp32_qemu(&entry_bin, true).expect("Failed to start ESP32 workspace Entry");

    // "Application setup complete" is the board's post-`register()` banner
    // — reaching it proves the launch node set registered against a live
    // executor (talker + listener `register()` returned Ok).
    let entry_output = entry
        .wait_for_output_pattern("Application setup complete", Duration::from_secs(60))
        .expect("ESP32 workspace Entry did not finish node registration");
    eprintln!("Workspace Entry registered its launch node set");

    // The external listener must log at least one real `Received:` line.
    let listener_output = native_proc.collect_until(
        nros_tests::output::INT32_LISTENER_LOG_PREFIX,
        Duration::from_secs(30),
    );
    let received = count_pattern(
        &listener_output,
        nros_tests::output::INT32_LISTENER_LOG_PREFIX,
    );
    eprintln!("Native listener received {received} message(s) from the workspace Entry");

    assert!(
        received >= 1,
        "Workspace Entry talker delivered no messages to the external native listener \
         (0 `Received:` lines).\nEntry output:\n{entry_output}\nListener output:\n{listener_output}",
    );

    eprintln!(
        "SUCCESS: ESP32 workspace Entry delivered {received} message(s) to the external listener"
    );
}

// =============================================================================
// Detection Tests (removed)
// =============================================================================
//
// `test_esp32_{qemu_riscv32,riscv32_target,espflash}_detection` each read one
// `is_*_available()` boolean and printed it. They asserted nothing, so all
// three reported PASS on a host with no qemu-system-riscv32, no riscv32 target
// and no espflash — the precise "always run" that CLAUDE.md forbids ("Tests
// must fail on unmet preconditions"). The probes remain load-bearing as the
// `skip!` guards on the real ESP32 tests above, where a `false` stops the run
// instead of decorating it. Forbidden repo-wide by `check-no-vacuous-tests`.
