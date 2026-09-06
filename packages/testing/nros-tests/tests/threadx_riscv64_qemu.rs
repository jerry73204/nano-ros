//! ThreadX QEMU RISC-V 64-bit integration tests
//!
//! **Bucket (phase-329 W4): KEEP — not matrix cells.** The standard
//! (nano → host) pubsub/service/action DELIVERY e2e already lives in the
//! `rtos_e2e.rs` matrix consumer (see the note below). What remains here is
//! one-offs no cell covers: the detection probe and the CycloneDDS TWO-QEMU
//! pubsub tests (two embedded ThreadX-RV64 nodes talking over Cyclone DDS —
//! a distinct topology from rtos_e2e's nano → host zenoh shape), c/rust/cpp.
//!
//! Tests that verify ThreadX QEMU RISC-V examples build and run on QEMU virt
//! machine with virtio-net networking. Examples use `riscv64gc-unknown-none-elf`
//! target with `no_std` + NetX Duo networking over virtio-net.
//!
//! The E2E test bodies live in `tests/rtos_e2e.rs` (parametrised over
//! platform × language × variant).
//!
//! Prerequisites:
//! - `THREADX_DIR` env var pointing to ThreadX source (e.g., `third-party/threadx/kernel`)
//! - `NETX_DIR` env var pointing to NetX Duo source (e.g., `third-party/threadx/netxduo`)
//! - `riscv64-unknown-elf-gcc` cross-compiler installed
//! - `qemu-system-riscv64` with virt machine support
//! - zenohd: `just build-zenohd`
//!
//! Run with: `just test-threadx-riscv64`
//! Or: `cargo nextest run -p nros-tests --test threadx_riscv64_qemu`

use std::time::Duration;

use nros_tests::fixtures::{
    QemuProcess, Rmw, is_qemu_riscv64_available, qemu_riscv64_supports_dgram_unix,
    threadx_riscv64::{
        build_rv64_cmake_example_rmw, is_netx_available, is_riscv_gcc_available,
        is_threadx_available,
    },
};

// =============================================================================
// Prerequisite checks
// =============================================================================

/// Require the ThreadX RISC-V prerequisites; skip loudly (naming which one).
///
/// Issue 1135 — this returned `bool` and every caller wrote
/// `if !require_threadx_riscv64() { nros_tests::skip!("require_threadx_riscv64 check failed"); }`. That is the
/// CORRECT verdict spelled uninformatively: the real reason was an
/// `eprintln!` inside the helper, and `--failure-output never` (what the
/// `just` recipes pass) eats it, so the log said only "check failed". A guard
/// that skips where it KNOWS the reason names the reason, and cannot be
/// misused by a caller who writes a bare `return` instead.
fn require_threadx_riscv64() {
    if !is_threadx_available() {
        nros_tests::skip!(
            "THREADX_DIR not set or invalid — run `just setup-threadx` + `source .envrc`"
        );
    }
    if !is_netx_available() {
        nros_tests::skip!(
            "NETX_DIR not set or invalid — run `just setup-threadx` + `source .envrc`"
        );
    }
    if !is_riscv_gcc_available() {
        nros_tests::skip!(
            "riscv64-unknown-elf-gcc not found — install it              (`sudo apt install gcc-riscv64-unknown-elf`)"
        );
    }
}

// =============================================================================
// Prerequisite detection tests (always run)
// =============================================================================

// `test_threadx_riscv64_detection` removed: it read five `is_*_available()`
// booleans and printed them, asserting nothing, so it reported PASS on a host
// with no ThreadX, no NetX Duo, no riscv64 gcc, no QEMU and no zenohd. All five
// probes stay load-bearing as the `skip!` guards on the real tests, where a
// `false` stops the run. Forbidden repo-wide by `check-no-vacuous-tests`.

// =============================================================================
// (Phase 182.3) `test_threadx_riscv64_all_examples_build` removed — it rebuilt
// every ThreadX-RV64 example, which `build-all` / `build-test-fixtures` already
// do before `test-all` (the `_require-fixtures` preflight). The per-role
// binaries are consumed by the `rtos_e2e` Platform__ThreadxRiscv64 tests.
// =============================================================================

// =============================================================================
// CycloneDDS two-QEMU peer interop (Phase 177.26)
// =============================================================================

/// Two ThreadX RISC-V64 QEMU nodes running the CycloneDDS C talker and
/// listener, wired together over an AF_UNIX SOCK_DGRAM L2 tunnel (no slirp
/// isolation), exchange a `std_msgs/Int32` sample on `/chatter`.
///
/// This exercises the real cross-node RTPS path: SPDP multicast discovery
/// over NetX Duo + IGMP (Phase 177.26 flips the ThreadX Cyclone profile's
/// `AllowMulticast` from `false` to `spdp`), unicast RTPS data delivery,
/// and CDR decode on the subscriber.
///
/// The CycloneDDS fixtures are built by default (Phase 203 decision):
///   just threadx_riscv64 build-fixtures   # or `just build-all`
/// (opt out with NROS_THREADX_RV64_CYCLONEDDS_FIXTURES=0).
///
/// Phase 177.26 — ThreadX↔ThreadX Cyclone RTPS works end-to-end. Two fixes
/// landed it: the cyclonedds ThreadX ddsrt port joins SPDP multicast with an
/// `INADDR_ANY` interface (NetX BSD `IP_ADD_MEMBERSHIP` byte-order, fork
/// `nano-ros`@`12b4af2c`), and the nano-ros Cyclone subscriber allocates its
/// RX take buffer from the ddsrt heap rather than libc (`std::calloc` returns
/// NULL on the unwired ThreadX libc heap; 177.26.RX.2). The earlier
/// `register_subscription -> -1` symptom closed under 177.28.
///
/// Transport: prefers `-netdev dgram` (QEMU ≥ 7.2 — point-to-point AF_UNIX
/// pair, CI-isolated). On older QEMU it falls back to `-netdev socket,mcast`
/// (shared host L2). Both put the two nodes on one link; reliable RTPS
/// retransmission covers any cross-process loss on the mcast path.
#[test]
fn test_threadx_riscv64_cyclonedds_two_qemu_pubsub() {
    require_threadx_riscv64();
    if !is_qemu_riscv64_available() {
        nros_tests::skip!("qemu-system-riscv64 not found");
    }

    let root = nros_tests::project_root();
    // Issue 0786 — RESOLVE, never `root.join`. A hand-built path skips the lane
    // coordinate check and the staleness probe, and the tier-2 build lane is
    // 1-wise, so it need not rebuild this coordinate: the C++ sibling of this
    // test ran a five-day-old image for exactly that reason and read as a code
    // regression.
    let talker_bin = build_rv64_cmake_example_rmw("c", "talker", "c_talker", Rmw::Cyclonedds)
        .expect("resolve rv64 C talker (cyclonedds)");
    let listener_bin = build_rv64_cmake_example_rmw("c", "listener", "c_listener", Rmw::Cyclonedds)
        .expect("resolve rv64 C listener (cyclonedds)");

    // MACs match each node's config.toml (talker 10.0.2.40/:56,
    // listener 10.0.2.41/:57) so QEMU's device MAC equals the NetX-assigned
    // address.
    const TALKER_MAC: &str = "52:54:00:12:34:56";
    const LISTENER_MAC: &str = "52:54:00:12:34:57";

    // Subscriber first so it has joined the SPDP multicast group before the
    // talker announces (CLAUDE.md QEMU-test convention; SPDP re-announces
    // periodically regardless). Both transports place the pair on one L2 link.
    let (mut listener, _talker) = if qemu_riscv64_supports_dgram_unix() {
        // AF_UNIX dgram pair (kept short to stay under sun_path's 108-byte
        // limit). Each QEMU binds its `local` path, sends to the peer's.
        let sock_dir = root.join("tmp");
        std::fs::create_dir_all(&sock_dir).expect("create tmp dir");
        let sock_talker = sock_dir.join("tx_rv64_cyc_talker.sock");
        let sock_listener = sock_dir.join("tx_rv64_cyc_listener.sock");
        let _ = std::fs::remove_file(&sock_talker);
        let _ = std::fs::remove_file(&sock_listener);
        let sock_talker = sock_talker.to_str().expect("utf-8 socket path");
        let sock_listener = sock_listener.to_str().expect("utf-8 socket path");

        let mut listener = QemuProcess::start_riscv64_virt_dgram(
            &listener_bin,
            sock_listener,
            sock_talker,
            LISTENER_MAC,
        )
        .expect("start listener QEMU (dgram)");
        // phase-342 W8b — wait for the listener to SAY it is subscribed.
        // All three threadx listeners print `LISTENER_READY_MARKER` (W7
        // converged them; `example_output_conformance` gates that they keep
        // doing so), so this cannot rot into a silent timeout.
        listener
            .wait_for_output_pattern(
                nros_tests::output::LISTENER_READY_MARKER,
                Duration::from_secs(30),
            )
            .unwrap_or_else(|e| panic!("threadx riscv64 listener never subscribed: {e}"));
        let talker = QemuProcess::start_riscv64_virt_dgram(
            &talker_bin,
            sock_talker,
            sock_listener,
            TALKER_MAC,
        )
        .expect("start talker QEMU (dgram)");
        (listener, talker)
    } else {
        // QEMU < 7.2 fallback: shared `-netdev socket,mcast` segment. The
        // group is dedicated to this test so it can't cross-talk with other
        // platforms' mcast-socket harnesses (threadx tests run single-threaded
        // in their nextest group).
        const MCAST: &str = "230.0.0.7:11700";
        let mut listener =
            QemuProcess::start_riscv64_virt_mcast(&listener_bin, MCAST, LISTENER_MAC)
                .expect("start listener QEMU (socket,mcast)");
        // phase-342 W8b — wait for the listener to SAY it is subscribed.
        // All three threadx listeners print `LISTENER_READY_MARKER` (W7
        // converged them; `example_output_conformance` gates that they keep
        // doing so), so this cannot rot into a silent timeout.
        listener
            .wait_for_output_pattern(
                nros_tests::output::LISTENER_READY_MARKER,
                Duration::from_secs(30),
            )
            .unwrap_or_else(|e| panic!("threadx riscv64 listener never subscribed: {e}"));
        let talker = QemuProcess::start_riscv64_virt_mcast(&talker_bin, MCAST, TALKER_MAC)
            .expect("start talker QEMU (socket,mcast)");
        (listener, talker)
    };

    // Wait for the listener to decode at least one sample. Discovery +
    // first delivery completes in a few seconds when it works; the generous
    // window covers SPDP retry cadence. `_talker` is dropped (and killed) at
    // end of scope.
    let result = listener.wait_for_output_pattern(
        nros_tests::output::LISTENER_LOG_PREFIX,
        Duration::from_secs(90),
    );
    listener.kill();

    match result {
        Ok(output) => {
            nros_tests::output::assert_listener(&output, 1);
        }
        Err(e) => {
            panic!("listener never received a CycloneDDS sample from the peer ThreadX node: {e:?}")
        }
    }
}

// =============================================================================
// CycloneDDS two-QEMU peer interop — RUST examples (issue #214)
// =============================================================================

/// Two ThreadX RISC-V64 QEMU nodes running the RUST CycloneDDS talker and
/// listener exchange `std_msgs/String` on `/chatter` — the rust sibling of
/// `test_threadx_riscv64_cyclonedds_two_qemu_pubsub`. Closes the #214 gap:
/// the rust cyclone fixtures previously had NO test consumer, the deploy-less
/// `run_app_thread(Config::default())` boot ran every image with the same
/// MAC/IP/domain (identity collapse), and the domain never matched the
/// fixture bake. `Config::default()` now applies the build-env identity
/// (`NROS_APP_NET_{IP,MAC}_LAST`, `NROS_DOMAIN_ID` — set per-example by
/// `nros_threadx_rv64_rust_cyclone_app`), so the pair boots as
/// 192.0.3.10/:56 (talker) + 192.0.3.11/:57 (listener) on the fixture domain.
///
/// The QEMU device MACs mirror the firmware bake (same convention as the C
/// test). Descriptors register via the board `.init_array` walk (#195/#205).
#[test]
fn test_threadx_riscv64_cyclonedds_two_qemu_rust_pubsub() {
    use nros_tests::fixtures::{Rmw, build_threadx_rv64_rust_example_rmw};

    require_threadx_riscv64();
    if !is_qemu_riscv64_available() {
        nros_tests::skip!("qemu-system-riscv64 not found");
    }

    let talker_bin = build_threadx_rv64_rust_example_rmw(
        "talker",
        "riscv64_threadx_rust_talker",
        Rmw::Cyclonedds,
    )
    .unwrap_or_else(|e| {
        nros_tests::skip!(
            "rust cyclone talker fixture missing (just threadx_riscv64 build-fixtures): {e:?}"
        )
    });
    let listener_bin = build_threadx_rv64_rust_example_rmw(
        "listener",
        "riscv64_threadx_rust_listener",
        Rmw::Cyclonedds,
    )
    .unwrap_or_else(|e| {
        nros_tests::skip!(
            "rust cyclone listener fixture missing (just threadx_riscv64 build-fixtures): {e:?}"
        )
    });

    const TALKER_MAC: &str = "52:54:00:12:34:56";
    const LISTENER_MAC: &str = "52:54:00:12:34:57";

    // Subscriber first (SPDP re-announces cover the rest); both transports put
    // the two nodes on one L2 link — same shapes as the C sibling.
    let (mut listener, _talker) = if qemu_riscv64_supports_dgram_unix() {
        let root = nros_tests::project_root();
        let sock_dir = root.join("tmp");
        std::fs::create_dir_all(&sock_dir).expect("create tmp dir");
        let sock_talker = sock_dir.join("tx_rv64_cyc_rs_talker.sock");
        let sock_listener = sock_dir.join("tx_rv64_cyc_rs_listener.sock");
        let _ = std::fs::remove_file(&sock_talker);
        let _ = std::fs::remove_file(&sock_listener);
        let sock_talker = sock_talker.to_str().expect("utf-8 socket path");
        let sock_listener = sock_listener.to_str().expect("utf-8 socket path");

        let mut listener = QemuProcess::start_riscv64_virt_dgram(
            &listener_bin,
            sock_listener,
            sock_talker,
            LISTENER_MAC,
        )
        .expect("start listener QEMU (dgram)");
        // phase-342 W8b — wait for the listener to SAY it is subscribed.
        // All three threadx listeners print `LISTENER_READY_MARKER` (W7
        // converged them; `example_output_conformance` gates that they keep
        // doing so), so this cannot rot into a silent timeout.
        listener
            .wait_for_output_pattern(
                nros_tests::output::LISTENER_READY_MARKER,
                Duration::from_secs(30),
            )
            .unwrap_or_else(|e| panic!("threadx riscv64 listener never subscribed: {e}"));
        let talker = QemuProcess::start_riscv64_virt_dgram(
            &talker_bin,
            sock_talker,
            sock_listener,
            TALKER_MAC,
        )
        .expect("start talker QEMU (dgram)");
        (listener, talker)
    } else {
        // Dedicated mcast group — distinct from the C test's so the two can
        // run concurrently within the serialized qemu group.
        const MCAST: &str = "230.0.0.7:11701";
        let mut listener =
            QemuProcess::start_riscv64_virt_mcast(&listener_bin, MCAST, LISTENER_MAC)
                .expect("start listener QEMU (socket,mcast)");
        // phase-342 W8b — wait for the listener to SAY it is subscribed.
        // All three threadx listeners print `LISTENER_READY_MARKER` (W7
        // converged them; `example_output_conformance` gates that they keep
        // doing so), so this cannot rot into a silent timeout.
        listener
            .wait_for_output_pattern(
                nros_tests::output::LISTENER_READY_MARKER,
                Duration::from_secs(30),
            )
            .unwrap_or_else(|e| panic!("threadx riscv64 listener never subscribed: {e}"));
        let talker = QemuProcess::start_riscv64_virt_mcast(&talker_bin, MCAST, TALKER_MAC)
            .expect("start talker QEMU (socket,mcast)");
        (listener, talker)
    };

    let result = listener.wait_for_output_pattern(
        nros_tests::output::LISTENER_LOG_PREFIX,
        Duration::from_secs(90),
    );
    listener.kill();

    match result {
        Ok(output) => {
            nros_tests::output::assert_listener(&output, 1);
        }
        Err(e) => panic!(
            "rust listener never received a CycloneDDS sample from the peer ThreadX node: {e:?}"
        ),
    }
}

// =============================================================================
// CycloneDDS two-QEMU peer interop — C++ examples (issue #235)
// =============================================================================

/// Two ThreadX RISC-V64 QEMU nodes running the **C++** CycloneDDS talker and
/// listener exchange `std_msgs/String` on `/chatter` — the C++ sibling of
/// `test_threadx_riscv64_cyclonedds_two_qemu_pubsub` (C) and `..._rust_pubsub`
/// (#214). Closes issue #235: the C++ cyclone riscv64 fixtures already built
/// (talker .40/0x56, listener .41/0x57, domain 129 — distinct identity per
/// node) but had no runtime consumer, leaving the matrix cell BuildOnly. The
/// C++ cyclone code path itself is proven on threadx-linux
/// (`test_threadx_linux_cyclonedds_cpp_talker_to_native_listener`).
#[test]
fn test_threadx_riscv64_cyclonedds_two_qemu_cpp_pubsub() {
    require_threadx_riscv64();
    if !is_qemu_riscv64_available() {
        nros_tests::skip!("qemu-system-riscv64 not found");
    }

    let root = nros_tests::project_root();
    // Issue 0786 — RESOLVE, never `root.join`; see the note in the C sibling.
    // This is the test that ran the museum binary.
    let talker_bin = build_rv64_cmake_example_rmw("cpp", "talker", "cpp_talker", Rmw::Cyclonedds)
        .expect("resolve rv64 C++ talker (cyclonedds)");
    let listener_bin =
        build_rv64_cmake_example_rmw("cpp", "listener", "cpp_listener", Rmw::Cyclonedds)
            .expect("resolve rv64 C++ listener (cyclonedds)");

    // MACs match each C++ node's baked identity (talker 0x56, listener 0x57).
    const TALKER_MAC: &str = "52:54:00:12:34:56";
    const LISTENER_MAC: &str = "52:54:00:12:34:57";

    let (mut listener, _talker) = if qemu_riscv64_supports_dgram_unix() {
        let sock_dir = root.join("tmp");
        std::fs::create_dir_all(&sock_dir).expect("create tmp dir");
        let sock_talker = sock_dir.join("tx_rv64_cyc_cpp_talker.sock");
        let sock_listener = sock_dir.join("tx_rv64_cyc_cpp_listener.sock");
        let _ = std::fs::remove_file(&sock_talker);
        let _ = std::fs::remove_file(&sock_listener);
        let sock_talker = sock_talker.to_str().expect("utf-8 socket path");
        let sock_listener = sock_listener.to_str().expect("utf-8 socket path");

        let mut listener = QemuProcess::start_riscv64_virt_dgram(
            &listener_bin,
            sock_listener,
            sock_talker,
            LISTENER_MAC,
        )
        .expect("start C++ listener QEMU (dgram)");
        // phase-342 W8b — wait for the listener to SAY it is subscribed.
        // All three threadx listeners print `LISTENER_READY_MARKER` (W7
        // converged them; `example_output_conformance` gates that they keep
        // doing so), so this cannot rot into a silent timeout.
        listener
            .wait_for_output_pattern(
                nros_tests::output::LISTENER_READY_MARKER,
                Duration::from_secs(30),
            )
            .unwrap_or_else(|e| panic!("threadx riscv64 listener never subscribed: {e}"));
        let talker = QemuProcess::start_riscv64_virt_dgram(
            &talker_bin,
            sock_talker,
            sock_listener,
            TALKER_MAC,
        )
        .expect("start C++ talker QEMU (dgram)");
        (listener, talker)
    } else {
        // Distinct mcast group from the C (11700) / rust lanes so the segments
        // never cross-talk; threadx lanes are serialized in their nextest group
        // regardless.
        const MCAST: &str = "230.0.0.8:11701";
        let mut listener =
            QemuProcess::start_riscv64_virt_mcast(&listener_bin, MCAST, LISTENER_MAC)
                .expect("start C++ listener QEMU (socket,mcast)");
        // phase-342 W8b — wait for the listener to SAY it is subscribed.
        // All three threadx listeners print `LISTENER_READY_MARKER` (W7
        // converged them; `example_output_conformance` gates that they keep
        // doing so), so this cannot rot into a silent timeout.
        listener
            .wait_for_output_pattern(
                nros_tests::output::LISTENER_READY_MARKER,
                Duration::from_secs(30),
            )
            .unwrap_or_else(|e| panic!("threadx riscv64 listener never subscribed: {e}"));
        let talker = QemuProcess::start_riscv64_virt_mcast(&talker_bin, MCAST, TALKER_MAC)
            .expect("start C++ talker QEMU (socket,mcast)");
        (listener, talker)
    };

    let result = listener.wait_for_output_pattern(
        nros_tests::output::LISTENER_LOG_PREFIX,
        Duration::from_secs(90),
    );
    listener.kill();

    match result {
        Ok(output) => {
            nros_tests::output::assert_listener(&output, 1);
        }
        Err(e) => panic!(
            "C++ listener never received a CycloneDDS sample from the peer ThreadX node: {e:?}"
        ),
    }
}

// =============================================================================
// issue 0680 — per-thread `errno`
// =============================================================================

/// Two tasks spawned through `nros_platform_task_init` must not share `errno`.
///
/// **Why this is not a cell body in `rtos_e2e.rs`:** it is not a delivery test.
/// There is no peer, no zenohd and no message — one image spawns two tasks, one
/// provokes a real libc failure (`strtol` overflow -> `ERANGE`), and the other
/// checks that its own `errno` is untouched. The `matrix::CELLS` row exists so
/// the fixture has an owner that RUNS it; the body lives here with the other
/// one-off topologies.
///
/// This test is a discriminator, not a smoke test: on a board without the
/// `tx_thread_schedule.S` reent swap it reports `FAIL shared errno` with the
/// victim's `errno` visible from the observer. That was verified by running it
/// against a build with the swap removed, which is the only reason to trust a
/// pass here.
#[test]
fn test_threadx_riscv64_errno_is_per_thread() {
    require_threadx_riscv64();
    if !is_qemu_riscv64_available() {
        nros_tests::skip!("qemu-system-riscv64 not found");
    }

    let binary = match nros_tests::fixtures::threadx_riscv64::build_rv64_c_errno_isolation() {
        Ok(p) => p,
        Err(e) => nros_tests::skip!("errno-isolation fixture unavailable: {e}"),
    };

    // Self-contained image: the netdev exists only because the board brings up
    // NetX at boot, and nothing in this test talks to it.
    let mut qemu = QemuProcess::start_riscv64_virt(binary, 0).expect("start errno-isolation QEMU");

    // Wait for a VERDICT, not just the pass marker: waiting on PASS alone
    // would turn a real FAIL into a 60s timeout, reported as a hang -- which
    // hides the exact finding this fixture exists to produce.
    let out = qemu.collect_until(
        nros_tests::output::ERRNO_ISOLATION_VERDICT,
        Duration::from_secs(60),
    );

    assert!(
        out.contains(nros_tests::output::ERRNO_ISOLATION_PASS),
        "errno is SHARED between ThreadX tasks on threadx-riscv64, or the \
         fixture never reached a verdict (issue 0680).\nOutput:\n{out}"
    );
}
