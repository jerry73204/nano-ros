//! phase-295 W6.c — THE ROS 2 interop matrix consumer (RFC-0051 §2 + §W6.c).
//!
//! Consolidates the ROS 2 interop family — `rmw_interop.rs` (zenoh),
//! `cyclonedds_ros2_interop.rs` (cyclone), `demo_nodes_cpp_interop.rs`
//! (cross-vendor stock rclcpp talker), and `ros2_lifecycle_interop.rs`
//! (lifecycle) — into one parametrized test over the `Kind::Interop` cells
//! of the test matrix (`nros_tests::matrix`): a nano-ros node exchanges data
//! with a REAL ROS 2 peer, over the reduced workload set (pubsub + service,
//! plus lifecycle) × direction (nano-pub/ros-sub, ros-pub/nano-sub) × RMW
//! (rmw_zenoh_cpp, rmw_cyclonedds_cpp).
//!
//! **Behavioral interchangeability (RFC-0051 §2).** The shared
//! [`nros_tests::checker::assert_delivery`] reads process output and is
//! transport/peer-agnostic, so every nano-ros endpoint (and the stock
//! `demo_nodes_cpp` peer) is asserted through it — the same contract the
//! nano↔nano example cells pin. The raw `ros2 topic echo` / `ros2 service
//! call` sinks are NOT demo nodes; they dump the DDS/CLI wire fields
//! (`data:`, `sum`), so those directions count wire samples with the
//! DDS/CLI markers (which are NOT nano demo markers — the output-marker gate
//! only guards the nano demo wording in `output.rs`).
//!
//! Skip semantics are preserved EXACTLY from the per-cell files: the zenoh +
//! lifecycle cells gate on `require_ros2` (ROS 2 CLI + `rmw_zenoh_cpp`) and a
//! startable `zenohd`; the cyclone cells gate on `require_ros2_cyclonedds`
//! (ROS 2 + `rmw_cyclonedds_cpp`) + the native Cyclone fixtures. A missing
//! ROS 2 / RMW / fixture / peer-launch is a clean `skip!`, never a failure —
//! after the gate passes, ZERO delivery is a real failure (#133 fail-loud).
//!
//! Isolation: zenoh cells take an EPHEMERAL router
//! (`ZenohRouter::start_unique`, `NROS_LOCATOR`); cyclone cells take a
//! PID-seeded `unique_ros_domain_id()` `ROS_DOMAIN_ID` so no two concurrent
//! interop tests share a domain (across RMWs too). The whole binary runs in
//! the `ros2-interop` nextest group (singleton ros2 daemon = the exclusive
//! resource; max-threads = 1); the cyclone cases route to
//! `host-dds-ros2-interop` (shared 232-slot DDS domain space) via a
//! `test(cyclone)` filter.
//!
//! Bespoke interop lanes kept OUT of this consumer (their own binaries):
//! `xrce_ros2_interop.rs` (XRCE Agent lifecycle specifics) and
//! `qos_zephyr_ros2_interop_e2e.rs` (the on-target zephyr-image QoS interop,
//! `zephyr-qos-port` nextest group). See the phase-295 W6.c doc for the
//! introspection/benchmark lanes retired in the reduction.
//!
//! Run with: `cargo nextest run -p nros-tests --test interop_e2e`
//! (one RMW: `-E 'binary(interop_e2e) and test(cyclone)'`).

use nros_tests::{
    checker::assert_delivery,
    count_pattern,
    fixtures::{
        DEFAULT_ROS_DISTRO, ManagedProcess, Rmw as FixtureRmw, Ros2DdsProcess, Ros2Process,
        ZenohRouter, build_native_c_example_rmw, build_ros2_string_interop, lifecycle_node_binary,
        listener_binary, require_ros2, require_ros2_cyclonedds, ros2_env_setup_with_locator,
        service_client_binary, service_server_binary, talker_binary,
    },
    matrix::{Lang, PlatformId, Rmw, Workload},
    output, skip,
};
use rstest::rstest;
use std::{
    path::Path,
    process::Command,
    time::{Duration, Instant},
};

const TOPIC: &str = "/chatter";
const STRING_MSG: &str = "std_msgs/msg/String";
const SRV: &str = "/add_two_ints";
const SRV_TYPE: &str = "example_interfaces/srv/AddTwoInts";

// =============================================================================
// Cell table
// =============================================================================

/// The exact interop scenario the cell drives (RMW × workload × direction).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Scenario {
    /// zenoh: nano talker → `ros2 topic echo` (rmw_zenoh_cpp).
    ZenohPubsubNanoToRos2,
    /// zenoh: `ros2 topic pub` (rmw_zenoh_cpp) → nano listener.
    ZenohPubsubRos2ToNano,
    /// zenoh cross-vendor: a STOCK unmodified `demo_nodes_cpp talker`
    /// (rclcpp) → nano listener — the phase-211 "behaves like real ROS"
    /// proof (`demo_nodes_cpp_interop.rs`).
    ZenohPubsubStockDemoToNano,
    /// zenoh: nano service server ↔ `ros2 service call` (rmw_zenoh_cpp).
    ZenohServiceNanoServer,
    /// zenoh: `ros2` AddTwoInts server ↔ nano service client.
    ZenohServiceRos2Server,
    /// cyclone: nano talker → `ros2 topic echo` (rmw_cyclonedds_cpp).
    CyclonePubsubNanoToRos2,
    /// cyclone: `ros2 topic pub` (rmw_cyclonedds_cpp) → nano listener.
    CyclonePubsubRos2ToNano,
    /// cyclone: nano service server ↔ `ros2 service call` (rmw_cyclonedds_cpp).
    CycloneServiceNanoServer,
    /// zenoh: drive an nros lifecycle node through the REP-2002 service
    /// surface via `ros2 lifecycle …` (`ros2_lifecycle_interop.rs`).
    ZenohLifecycle,
}

impl Scenario {
    /// `true` for the cyclone cells (`test(cyclone)` routes them to the
    /// `host-dds-ros2-interop` nextest group).
    fn is_cyclone(self) -> bool {
        matches!(
            self,
            Scenario::CyclonePubsubNanoToRos2
                | Scenario::CyclonePubsubRos2ToNano
                | Scenario::CycloneServiceNanoServer
        )
    }
}

/// The `interop::CELLS` coordinate a scenario exercises — all native; the lang,
/// rmw and workload come from the scenario. Direction and the stock-demo variant
/// collapse onto the same coordinate (the binding is at coordinate level).
///
/// phase-433 W3 — the LANGUAGE is the one the case actually spawns, not a
/// constant. It was `Lang::Rust` unconditionally while every cyclone case runs
/// `nano_cyclone_c_binary(...)` — `c_talker` / `c_listener` /
/// `c_service_server` out of `examples/native/c/`. The per-case tripwire below
/// could not catch it: `interop::CELLS` declared the same wrong language, so
/// the coordinate agreed with itself and C/Cyclone ran while Rust/Cyclone was
/// the shape the matrix claimed. The zenoh + lifecycle cases really are Rust —
/// `talker_binary` / `listener_binary` / `service_{server,client}_binary` /
/// `lifecycle_node_binary` all resolve `examples/native/rust/*`, and the
/// stock-demo case's nano side is the Rust `bins/ros2-string-interop` (the C++
/// in it is the PEER, which no coordinate names).
fn scenario_coord(s: Scenario) -> (PlatformId, Lang, Rmw, Workload) {
    use Scenario::*;
    let (lang, rmw, workload) = match s {
        ZenohPubsubNanoToRos2 | ZenohPubsubRos2ToNano | ZenohPubsubStockDemoToNano => {
            (Lang::Rust, Rmw::Zenoh, Workload::Pubsub)
        }
        ZenohServiceNanoServer | ZenohServiceRos2Server => {
            (Lang::Rust, Rmw::Zenoh, Workload::Service)
        }
        CyclonePubsubNanoToRos2 | CyclonePubsubRos2ToNano => {
            (Lang::C, Rmw::Cyclonedds, Workload::Pubsub)
        }
        CycloneServiceNanoServer => (Lang::C, Rmw::Cyclonedds, Workload::Service),
        ZenohLifecycle => (Lang::Rust, Rmw::Zenoh, Workload::Lifecycle),
    };
    (PlatformId::Linux, lang, rmw, workload)
}

/// One interop matrix cell.
struct Cell {
    scenario: Scenario,
    /// Provenance / nuance — folded into failure messages so a red cell
    /// still names the seam it pins.
    note: &'static str,
}

// =============================================================================
// Shared helpers
// =============================================================================

/// Skip-precondition gate: cyclone cells need ROS 2 + `rmw_cyclonedds_cpp`;
/// zenoh + lifecycle cells need ROS 2 + `rmw_zenoh_cpp`. Identical semantics
/// to the pre-consolidation files.
fn require_cell_env(scenario: Scenario) {
    if scenario.is_cyclone() {
        if !require_ros2_cyclonedds() {
            skip!("ROS 2 + rmw_cyclonedds_cpp not available");
        }
    } else if !require_ros2() {
        skip!(
            "ROS 2 / rmw_zenoh_cpp not available — install it from apt \
             (`ros-$ROS_DISTRO-rmw-zenoh-cpp`, declared in nros-sdk-index.toml)."
        );
    }
}

/// Start an ephemeral zenohd for the zenoh cells; a missing/unstartable
/// zenohd is a clean skip (the SUT is the interop, not the router).
fn start_zenoh_router() -> ZenohRouter {
    ZenohRouter::start_unique().unwrap_or_else(|e| skip!("zenohd failed to start: {e}"))
}

/// Spawn a native nano-ros zenoh binary dialing `locator`.
fn spawn_nano_zenoh(bin: &Path, name: &str, locator: &str) -> ManagedProcess {
    let mut cmd = Command::new(bin);
    cmd.env("RUST_LOG", "info").env("NROS_LOCATOR", locator);
    ManagedProcess::spawn_command(cmd, name).unwrap_or_else(|e| panic!("spawn {name}: {e}"))
}

/// Resolve (building if needed) a native Cyclone C example binary, or skip
/// when the fixtures aren't set up (`just cyclonedds setup`).
fn nano_cyclone_c_binary(case: &str, binary: &str) -> std::path::PathBuf {
    build_native_c_example_rmw(case, binary, FixtureRmw::Cyclonedds).unwrap_or_else(|e| {
        skip!("native/c/{case} cyclonedds fixture not built (run `just cyclonedds setup`): {e:?}")
    })
}

/// Spawn a nano-ros Cyclone binary on `domain_id`, wiring `LD_LIBRARY_PATH`
/// to the in-tree `libddsc` (mirrors `native_api.rs::spawn_cyclone_binary`).
fn spawn_nano_cyclone(binary: &Path, name: &str, domain_id: u8) -> ManagedProcess {
    let mut cmd = Command::new(binary);
    cmd.env("ROS_DOMAIN_ID", domain_id.to_string())
        .env("RUST_LOG", "info");
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
    // Issue 1137 — the SAME bus as the `ros2` peer on the other side of every
    // cyclone cell. Those peers go through `ros2_env_setup_rmw_with_domain`,
    // which has exported a loopback-only `CYCLONEDDS_URI` since issue 1009;
    // this half got nothing, so the pair sat on two different interfaces and
    // discovered nothing at all. "Pin both sides or neither" is 1009's own
    // conclusion, and this is the other side.
    nros_tests::dds_isolation::apply_to_command(&mut cmd);
    ManagedProcess::spawn_command(cmd, name).unwrap_or_else(|_| panic!("Failed to start {name}"))
}

/// Run `ros2 <subcommand>` against `locator` (rmw_zenoh overlay); return
/// combined stdout+stderr. Used by the lifecycle cell — `--no-daemon` is
/// passed by the caller so the CLI uses this process' zenoh session.
fn run_ros2(locator: &str, subcommand: &str) -> String {
    let (env, _config_guard) = ros2_env_setup_with_locator(DEFAULT_ROS_DISTRO, locator);
    let script = format!("{env} && timeout 10 ros2 {subcommand} 2>&1");
    let out = Command::new("bash")
        .args(["-c", &script])
        .output()
        .expect("failed to spawn bash for ros2 invocation");
    String::from_utf8_lossy(&out.stdout).into_owned()
}

/// Poll `ros2 <subcommand>` until its output contains `marker`
/// (case-insensitive) or timeout.
fn poll_ros2_until(locator: &str, subcommand: &str, marker: &str, timeout: Duration) -> String {
    let deadline = Instant::now() + timeout;
    let marker = marker.to_lowercase();
    let mut last = String::new();
    while Instant::now() < deadline {
        last = run_ros2(locator, subcommand);
        if last.to_lowercase().contains(&marker) {
            return last;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    last
}

// =============================================================================
// The parametrized matrix consumer
// =============================================================================

/// One interop cell: exchange data between a nano-ros node and a real ROS 2
/// peer, asserting delivery per the scenario. Case names carry the
/// `<rmw>_<workload>_<direction>` shape so nextest `test(...)` filters can
/// slice by RMW (`test(cyclone)` routes to the host-DDS group).
#[rstest]
// ── zenoh (rmw_zenoh_cpp) — matrix (Linux, Rust, Zenoh, {Pubsub,Service}, Interop)
#[case::zenoh_pubsub_nano_to_ros2(Cell {
    scenario: Scenario::ZenohPubsubNanoToRos2,
    note: "#133 fail-loud: after require_ros2, the ros2 subscriber receiving 0 \
           `data:` samples is a real rmw_zenoh delivery failure, not timing",
})]
#[case::zenoh_pubsub_ros2_to_nano(Cell {
    scenario: Scenario::ZenohPubsubRos2ToNano,
    note: "#146: rmw_zenoh pub → zenoh-pico sub discovery is ~10 s (25 s window); \
           data integrity checked once delivery held",
})]
#[case::zenoh_pubsub_stock_demo_nodes_cpp(Cell {
    scenario: Scenario::ZenohPubsubStockDemoToNano,
    note: "phase-211: an UNMODIFIED stock demo_nodes_cpp talker (rclcpp) reaches a \
           nano-ros subscriber cross-vendor over a shared zenohd",
})]
#[case::zenoh_service_nano_server(Cell {
    scenario: Scenario::ZenohServiceNanoServer,
    note: "#133 fail-loud: the ros2 client must receive a `sum` reply (5 + 3 = 8)",
})]
#[case::zenoh_service_ros2_server(Cell {
    scenario: Scenario::ZenohServiceRos2Server,
    note: "#133 fail-loud: the nano client must receive the ros2 AddTwoInts reply",
})]
// ── cyclone (rmw_cyclonedds_cpp) — matrix (Linux, {C}, Cyclonedds, {Pubsub,Service}, Interop)
#[case::cyclone_pubsub_nano_to_ros2(Cell {
    scenario: Scenario::CyclonePubsubNanoToRos2,
    note: "phase-117/183.5: nano-ros Cyclone talker is wire-compatible with stock \
           rmw_cyclonedds_cpp over RTPS/SPDP",
})]
#[case::cyclone_pubsub_ros2_to_nano(Cell {
    scenario: Scenario::CyclonePubsubRos2ToNano,
    note: "phase-183.5: setvbuf(_IOLBF) in examples/native/c/listener made the \
           `I heard:` lines reach the harness (block-buffering, not a wire gap)",
})]
#[case::cyclone_service_nano_server(Cell {
    scenario: Scenario::CycloneServiceNanoServer,
    note: "phase-117.12.B.1: write the reply once the reply reader is DISCOVERED \
           (total_count > 0), not only on current_count > 0 (src/service.cpp)",
})]
// ── zenoh lifecycle — matrix (Linux, Rust, Zenoh, Lifecycle, Interop)
#[case::zenoh_lifecycle_full_cycle(Cell {
    scenario: Scenario::ZenohLifecycle,
    note: "the full REP-2002 cycle driven end-to-end via `ros2 lifecycle …` \
           (nodes/get/list + set configure→activate→deactivate→cleanup) against an \
           nros lifecycle node, asserting each transition, its on_* callback, and \
           the resulting state",
})]
fn interop(#[case] cell: Cell) {
    // Issue 0352 / phase-324 W4.d — per-case binding: the coordinate this case
    // runs must be declared for `interop_e2e` in `interop::CELLS`. Runs before
    // the fixture gate, so it is exercised for every case even when the runtime
    // dependency is absent (the case then skips). A case whose scenario drifts
    // from what the SSoT declares fails here, not silently.
    let (p, l, r, w) = scenario_coord(cell.scenario);
    assert!(
        nros_tests::interop::test_covers("interop_e2e", p, l, r, w),
        "interop_e2e case {:?} runs coordinate ({p:?}, {l:?}, {r:?}, {w:?}) that \
         interop::CELLS does not declare for `interop_e2e` — issue 0352 binding drift",
        cell.scenario
    );

    require_cell_env(cell.scenario);

    match cell.scenario {
        // ── zenoh pubsub: nano talker → ros2 topic echo ──────────────────
        Scenario::ZenohPubsubNanoToRos2 => {
            let router = start_zenoh_router();
            let locator = router.locator();
            // issue 1026 — the echo peer's LIFETIME and the wait are one
            // decision, made here. `ECHO_WINDOW` is the `timeout --foreground`
            // baked into the peer; the wait is shorter so a missed sample is
            // reported as "no delivery" rather than as the peer vanishing
            // mid-read. The wait itself is a CONDITION (one `data:` line), so
            // the common path returns as soon as delivery happens.
            const ECHO_WINDOW: Duration = Duration::from_secs(25);
            const ECHO_WAIT: Duration = Duration::from_secs(20);
            let mut ros2 = match Ros2Process::topic_echo_for(
                TOPIC,
                STRING_MSG,
                &locator,
                DEFAULT_ROS_DISTRO,
                ECHO_WINDOW,
            ) {
                Ok(p) => p,
                Err(e) => skip!("ROS 2 topic echo could not start: {e}"),
            };
            let mut talker = spawn_nano_zenoh(&talker_binary(), "native-rs-talker", &locator);
            // Bound stated: this cell asserts FIRST delivery only. It cannot
            // see a session that dies after the first sample — that is the
            // continuity assertion, and it lives on the pubsub cells that
            // count samples over a lease interval (issue 1013).
            // The asserted string and the diagnostic go on DIFFERENT channels
            // (issue 0670): the error text names the pattern it waited for, so
            // folding it into `out` would make `count_pattern(&out, "data:")`
            // match the complaint about the missing samples.
            let (out, why) = match ros2.wait_for_output_count("data:", 1, ECHO_WAIT) {
                Ok(o) => (o, String::new()),
                Err(e) => (String::new(), format!("\n[wait data:] {e}")),
            };
            talker.kill();

            let n = count_pattern(&out, "data:");
            assert!(
                n > 0,
                "nros → ROS 2 delivered nothing: the ros2 subscriber received 0 `data:` \
                 samples from the nano talker over rmw_zenoh ({}).\n\
                 Pairing: {}\n\
                 (phase-362 W2 — a zenoh CONVENTION break looks exactly like this, and \
                 the two versions above are the first thing to diff. `rmw_zenoh`'s \
                 conventions carry no version of their own, so the zenoh numbers are \
                 the closest available proxy; the ROS PACKAGE version is not — it is a \
                 wrapper version and says nothing about the zenoh inside it.)\n\
                 ROS 2 output:\n{out}{why}",
                cell.note,
                nros_tests::process::zenoh_pairing_versions()
            );
        }

        // ── zenoh pubsub: ros2 topic pub → nano listener ─────────────────
        Scenario::ZenohPubsubRos2ToNano => {
            let router = start_zenoh_router();
            let locator = router.locator();
            let mut listener = spawn_nano_zenoh(&listener_binary(), "native-rs-listener", &locator);
            listener
                .wait_for_output_pattern(
                    nros_tests::output::LISTENER_READY_MARKER,
                    Duration::from_secs(5),
                )
                .expect("nros listener did not become ready");

            let mut ros2 = match Ros2Process::topic_pub(
                TOPIC,
                STRING_MSG,
                "{data: 'Hello World: 42'}",
                1,
                &locator,
                DEFAULT_ROS_DISTRO,
            ) {
                Ok(p) => p,
                Err(e) => {
                    listener.kill();
                    skip!("ROS 2 publisher could not start: {e}");
                }
            };

            // #146 — rmw_zenoh pub → zenoh-pico sub discovery is ~10 s.
            let out = listener
                .wait_for_output_count(output::LISTENER_LOG_PREFIX, 1, Duration::from_secs(25))
                .unwrap_or_default();
            ros2.kill();
            listener.kill();

            assert_delivery(Workload::Pubsub, &out, 1);
            assert!(
                out.contains("Hello World: 42"),
                "ROS 2 → nros data integrity: payload 'Hello World: 42' missing ({}).\n\
                 nros output:\n{out}",
                cell.note
            );
        }

        // ── zenoh pubsub cross-vendor: stock demo_nodes_cpp talker → nano ─
        Scenario::ZenohPubsubStockDemoToNano => {
            let router = start_zenoh_router();
            let locator = router.locator();
            let sub_bin = build_ros2_string_interop()
                .map(|p| p.to_path_buf())
                .unwrap_or_else(|e| skip!("ros2-string-interop fixture not built: {e}"));

            // nano subscriber first, so its /chatter subscription is declared
            // before the stock talker publishes.
            let mut sub_cmd = Command::new(&sub_bin);
            sub_cmd
                .env("RUST_LOG", "info")
                .env("NROS_LOCATOR", &locator)
                .env("NROS_SESSION_MODE", "client");
            let mut sub = ManagedProcess::spawn_command(sub_cmd, "nros-string-sub")
                .expect("spawn nano-ros sub");
            sub.wait_for_output_pattern(
                nros_tests::output::LISTENER_READY_MARKER,
                Duration::from_secs(8),
            )
            .expect("nano-ros subscriber did not become ready");

            let mut talker = Ros2Process::demo_nodes_cpp_talker(&locator, DEFAULT_ROS_DISTRO)
                .expect("spawn demo_nodes_cpp talker");

            // This nano sub is the raw-Int32 `ros2-string-interop` bin
            // (prints `Received:`, not the demo `I heard:`), so it counts
            // the INT32 listener marker directly.
            let out = sub
                .wait_for_output_count(
                    output::INT32_LISTENER_LOG_PREFIX,
                    2,
                    Duration::from_secs(20),
                )
                .unwrap_or_default();
            sub.kill();
            talker.kill();

            let received = count_pattern(&out, output::INT32_LISTENER_LOG_PREFIX);
            assert!(
                received >= 2,
                "nano-ros must receive the stock demo_nodes_cpp talker cross-vendor \
                 (received = {received}) ({}).\n{out}",
                cell.note
            );
        }

        // ── zenoh service: nano server ↔ ros2 service call ───────────────
        Scenario::ZenohServiceNanoServer => {
            let router = start_zenoh_router();
            let locator = router.locator();
            let mut server = spawn_nano_zenoh(
                &service_server_binary(),
                "native-rs-service-server",
                &locator,
            );
            let _ = server.wait_for_output_pattern(
                output::SERVICE_SERVER_READY_MARKER,
                Duration::from_secs(5),
            );
            if !server.is_running() {
                panic!("native-rs-service-server (the nros SUT) exited before service-ready");
            }

            let mut client = match Ros2Process::service_call(
                SRV,
                SRV_TYPE,
                "{a: 5, b: 3}",
                &locator,
                DEFAULT_ROS_DISTRO,
            ) {
                Ok(p) => p,
                Err(e) => {
                    server.kill();
                    skip!("ROS 2 service call could not start: {e}");
                }
            };
            let out = client
                .wait_for_output(Duration::from_secs(10))
                .unwrap_or_default();
            server.kill();

            assert!(
                out.contains("sum"),
                "nros service server ↔ ROS 2 client got no `sum` response (5 + 3 = 8) \
                 ({}).\nROS 2 output:\n{out}",
                cell.note
            );
        }

        // ── zenoh service: ros2 server ↔ nano client ─────────────────────
        Scenario::ZenohServiceRos2Server => {
            let router = start_zenoh_router();
            let locator = router.locator();
            // phase-428 W13 / issue 1087 — the CLIENT starts first, with no
            // server anywhere, so the cell observes `wait_for_service`'s two
            // halves in order: it answers "no" while no server exists (the
            // client prints rclcpp's "service not available, waiting again..."
            // and sends nothing), and it answers "yes" once the ROS 2 server's
            // `SS` liveliness token lands in the zenoh matched-server set
            // (the client then calls and prints the result). Before W13 the
            // second half was a `z_liveliness_get` and the first half did not
            // exist — the zenoh client latched "seen once" forever, and every
            // other backend answered yes without asking.
            let mut client = spawn_nano_zenoh(
                &service_client_binary(),
                "native-rs-service-client",
                &locator,
            );
            let waiting = client.collect_until(
                output::SERVICE_NOT_AVAILABLE_MARKER,
                Duration::from_secs(15),
            );
            assert!(
                waiting.contains(output::SERVICE_NOT_AVAILABLE_MARKER),
                "with no server running, `service_is_ready` must answer NO and the \
                 client must say so (`{}`) rather than send into the void ({}).\n\
                 nano output:\n{waiting}",
                output::SERVICE_NOT_AVAILABLE_MARKER,
                cell.note
            );
            assert_eq!(
                count_pattern(&waiting, output::SERVICE_CALL_FAILED_MARKER),
                0,
                "the client sent a request while `service_is_ready` said no — the \
                 gate is not gating.\nnano output:\n{waiting}"
            );

            let mut ros2_server =
                match Ros2Process::add_two_ints_server(&locator, DEFAULT_ROS_DISTRO) {
                    Ok(p) => p,
                    Err(e) => skip!("ROS 2 service server could not start: {e}"),
                };
            // issue 1026 — was `wait_for_all_output(15s)`, which KILLS at the
            // deadline: the SUT's whole life was the wait window, so the cell
            // could only ever mean "did it answer within 15 s of birth".
            // `collect_until` waits on the CONDITION instead and leaves the
            // client running.
            //
            // Bound stated: the demo client is SINGLE-SHOT — `State::done`
            // latches on the first reply and it then idles forever — so one
            // result is everything it will ever print, and no count above 1 is
            // available to assert here. What this cell therefore cannot see is
            // whether the session survives past that first call; that is the
            // continuity question, and it belongs to a cell whose SUT keeps
            // issuing requests. Nor does it see the DELETE half of the set
            // (server goes away -> unavailable again): the client is done by
            // then. That half is pinned by `zpico-sys`'s
            // `graph_set_put_then_delete_round_trips` against the C set.
            let out = client.collect_until(output::SERVICE_RESULT_PREFIX, Duration::from_secs(20));
            ros2_server.kill();

            // The nano client prints the demo `Result of add_two_ints:` line.
            assert_delivery(Workload::Service, &out, 1);
            // And it got there through the gate, not around it: the result
            // follows the waiting line, and no attempt was made blind.
            assert_eq!(
                count_pattern(&out, output::SERVICE_CALL_FAILED_MARKER),
                0,
                "a request went out before the server was discoverable.\nnano output:\n{out}"
            );
        }

        // ── cyclone pubsub: nano talker → ros2 topic echo ────────────────
        Scenario::CyclonePubsubNanoToRos2 => {
            let domain = nros_tests::unique_ros_domain_id();
            let talker_bin = nano_cyclone_c_binary("talker", "c_talker");
            let mut ros2 = Ros2DdsProcess::topic_echo_cyclonedds_with_domain(
                TOPIC,
                STRING_MSG,
                DEFAULT_ROS_DISTRO,
                domain,
            )
            .expect("start ros2 cyclone echo");
            std::thread::sleep(Duration::from_secs(2));
            let mut talker = spawn_nano_cyclone(&talker_bin, "nano-cyclone-talker", domain);

            let out = ros2
                .wait_for_output(Duration::from_secs(10))
                .unwrap_or_default();
            talker.kill();

            let n = count_pattern(&out, "data:");
            assert!(
                n > 0,
                "ROS 2 cyclone subscriber received no `data:` samples from the nano \
                 talker ({}).\n{out}",
                cell.note
            );
        }

        // ── cyclone pubsub: ros2 topic pub → nano listener ───────────────
        Scenario::CyclonePubsubRos2ToNano => {
            let domain = nros_tests::unique_ros_domain_id();
            let listener_bin = nano_cyclone_c_binary("listener", "c_listener");
            let mut listener = spawn_nano_cyclone(&listener_bin, "nano-cyclone-listener", domain);
            std::thread::sleep(Duration::from_secs(3));
            let mut ros2 = Ros2DdsProcess::topic_pub_cyclonedds_with_domain(
                TOPIC,
                STRING_MSG,
                "{data: 'Hello World: 42'}",
                5,
                DEFAULT_ROS_DISTRO,
                domain,
            )
            .expect("start ros2 cyclone pub");

            let out = listener.collect_until(output::LISTENER_LOG_PREFIX, Duration::from_secs(10));
            ros2.kill();
            listener.kill();

            assert_delivery(Workload::Pubsub, &out, 1);
        }

        // ── cyclone service: nano server ↔ ros2 service call ─────────────
        Scenario::CycloneServiceNanoServer => {
            let domain = nros_tests::unique_ros_domain_id();
            let server_bin = nano_cyclone_c_binary("service-server", "c_service_server");
            let mut server = spawn_nano_cyclone(&server_bin, "nano-cyclone-service-server", domain);
            // Services need queryable/endpoint discovery before the client call.
            std::thread::sleep(Duration::from_secs(4));
            let mut client = Ros2DdsProcess::service_call_cyclonedds_with_domain(
                SRV,
                SRV_TYPE,
                "{a: 5, b: 3}",
                DEFAULT_ROS_DISTRO,
                domain,
            )
            .expect("start ros2 cyclone service call");

            let out = client
                .wait_for_output(Duration::from_secs(10))
                .unwrap_or_default();
            client.kill();
            server.kill();

            assert!(
                out.contains("sum=8") || out.contains("response"),
                "ROS 2 cyclone client did not get the AddTwoInts reply (expected sum=8) \
                 ({}).\n{out}",
                cell.note
            );
        }

        // ── zenoh lifecycle: drive the REP-2002 surface via ros2 lifecycle ─
        Scenario::ZenohLifecycle => {
            let router = start_zenoh_router();
            let locator = router.locator();

            let mut node = spawn_nano_zenoh(&lifecycle_node_binary(), "lifecycle-node", &locator);
            let boot_log = node
                .wait_for_output_pattern("Ready. Drive the lifecycle", Duration::from_secs(15))
                .expect("lifecycle-node never reached ready state");
            assert!(
                boot_log.contains("Lifecycle services registered"),
                "boot log missing service-registration marker: {boot_log}"
            );

            // A: `ros2 lifecycle nodes` discovers /lifecycle_demo.
            let nodes = poll_ros2_until(
                &locator,
                "lifecycle nodes --no-daemon --spin-time 0.1",
                "/lifecycle_demo",
                Duration::from_secs(10),
            );
            assert!(
                nodes.contains("/lifecycle_demo"),
                "ros2 lifecycle nodes did not list /lifecycle_demo ({}):\n{nodes}",
                cell.note
            );

            // B: initial get returns unconfigured.
            let before = run_ros2(
                &locator,
                "lifecycle get --no-daemon --spin-time 0.1 /lifecycle_demo",
            );
            assert!(
                before.to_lowercase().contains("unconfigured"),
                "expected Unconfigured before configure, got:\n{before}"
            );

            // C: set configure → inactive + fires on_configure.
            let configure = run_ros2(
                &locator,
                "lifecycle set --no-daemon --spin-time 0.1 /lifecycle_demo configure",
            );
            assert!(
                configure.contains("Transitioning successful"),
                "configure did not report success:\n{configure}"
            );
            node.wait_for_output_pattern("on_configure", Duration::from_secs(3))
                .expect("on_configure never logged");
            let after = poll_ros2_until(
                &locator,
                "lifecycle get --no-daemon --spin-time 0.1 /lifecycle_demo",
                "inactive",
                Duration::from_secs(5),
            );
            assert!(
                after.to_lowercase().contains("inactive"),
                "expected Inactive after configure, got:\n{after}"
            );

            // D: list shows reachable transitions from Inactive.
            let list = run_ros2(
                &locator,
                "lifecycle list --no-daemon --spin-time 0.1 /lifecycle_demo",
            );
            for marker in ["activate", "cleanup", "shutdown"] {
                assert!(
                    list.contains(marker),
                    "ros2 lifecycle list missing `{marker}`:\n{list}"
                );
            }

            // E: set activate → Active + fires on_activate.
            let activate = run_ros2(
                &locator,
                "lifecycle set --no-daemon --spin-time 0.1 /lifecycle_demo activate",
            );
            assert!(
                activate.contains("Transitioning successful"),
                "activate did not report success:\n{activate}"
            );
            node.wait_for_output_pattern("on_activate", Duration::from_secs(3))
                .expect("on_activate never logged");
            let active = poll_ros2_until(
                &locator,
                "lifecycle get --no-daemon --spin-time 0.1 /lifecycle_demo",
                "active",
                Duration::from_secs(5),
            );
            // `ros2 lifecycle get` prints just the current state (e.g. `active [3]`),
            // so Active is "active" with no "inactive" substring.
            assert!(
                active.to_lowercase().contains("active")
                    && !active.to_lowercase().contains("inactive"),
                "expected Active after activate, got:\n{active}"
            );

            // F: set deactivate → back to Inactive + fires on_deactivate.
            let deactivate = run_ros2(
                &locator,
                "lifecycle set --no-daemon --spin-time 0.1 /lifecycle_demo deactivate",
            );
            assert!(
                deactivate.contains("Transitioning successful"),
                "deactivate did not report success:\n{deactivate}"
            );
            node.wait_for_output_pattern("on_deactivate", Duration::from_secs(3))
                .expect("on_deactivate never logged");

            // G: set cleanup → back to Unconfigured + fires on_cleanup, closing the
            // REP-2002 cycle end to end.
            let cleanup = run_ros2(
                &locator,
                "lifecycle set --no-daemon --spin-time 0.1 /lifecycle_demo cleanup",
            );
            assert!(
                cleanup.contains("Transitioning successful"),
                "cleanup did not report success:\n{cleanup}"
            );
            node.wait_for_output_pattern("on_cleanup", Duration::from_secs(3))
                .expect("on_cleanup never logged");
            let final_state = poll_ros2_until(
                &locator,
                "lifecycle get --no-daemon --spin-time 0.1 /lifecycle_demo",
                "unconfigured",
                Duration::from_secs(5),
            );
            assert!(
                final_state.to_lowercase().contains("unconfigured"),
                "expected Unconfigured after cleanup, got:\n{final_state}"
            );

            node.kill();
        }
    }
}

// Issue 0352 / phase-324 — bind this test to `interop::CELLS`. The coordinates
// below must equal what the list declares for `interop_e2e`; adding/retiring an
// interop cell for this test, or drifting a cell's coordinate (issue 0341
// defect 2), turns this RED. Needs no fixtures — runs in tier 1.
#[test]
fn cases_bound_to_interop_cells() {
    #[allow(unused_imports)]
    use nros_tests::matrix::{Lang::*, PlatformId::*, Rmw::*, Workload::*};
    nros_tests::interop::assert_test_bound(
        "interop_e2e",
        &[
            (Linux, Rust, Zenoh, Pubsub),
            (Linux, Rust, Zenoh, Service),
            // The cyclone cases spawn the C examples, not the Rust ones
            // (phase-433 W3) — see `scenario_coord`.
            (Linux, C, Cyclonedds, Pubsub),
            (Linux, C, Cyclonedds, Service),
            (Linux, Rust, Zenoh, Lifecycle),
        ],
    );
}
