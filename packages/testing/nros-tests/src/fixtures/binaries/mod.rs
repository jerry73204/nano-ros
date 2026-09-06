//! Binary build helpers for integration tests
//!
//! Provides functions to build test binaries with caching support.

pub mod freertos;
pub mod nuttx;
pub mod threadx_linux;
pub mod threadx_riscv64;

use crate::{TestError, TestResult, build_dir, fixtures::staleness, project_root};
use duct::cmd;
use once_cell::sync::OnceCell;
use std::{
    env, fs,
    path::{Path, PathBuf},
    process::Command,
};

fn workspace_fixture_stamp_name(fixture_id: &str) -> String {
    format!(".nros-workspace-fixture.{fixture_id}.inputsig")
}

/// Cached path to the qemu-test binary
static QEMU_TEST_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the qemu-wcet-bench binary
static QEMU_WCET_BENCH_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the qemu-lan9118 binary
static QEMU_LAN9118_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native-rs-talker binary
static NATIVE_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native-rs-listener binary
static NATIVE_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Phase 115.F — cached path to the custom-transport-talker example.
static NATIVE_CT_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Phase 115.F — cached path to the custom-transport-listener example.
static NATIVE_CT_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Phase 211.I — cached path to the `tt-zenoh-to-xrce` bridge binary used by
/// the mixed-RMW bridge e2e (Phase 110.G.bridge example reused as fixture).
static NATIVE_BRIDGE_TT_ZENOH_XRCE_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Issue #53 — cached path to the `bridge-zenoh-to-cyclonedds-fwd` fixture
/// (the stock-cyclonedds sibling of `bridge-zenoh-to-xrce-fwd`).
static NATIVE_BRIDGE_ZENOH_CYCLONEDDS_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Phase 211.H — cached path to the `qos-override-pubsub` runtime-delivery
/// fixture (`packages/testing/nros-tests/bins/qos-override-pubsub`).
static NATIVE_QOS_OVERRIDE_PUBSUB_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Phase 250 Wave 5 — cached path to the `declarative-safety-listener` fixture
/// (`packages/testing/nros-tests/bins/declarative-safety-listener`).
static NATIVE_DECLARATIVE_SAFETY_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Phase 211 acceptance — cached path to the `ros2-string-interop` fixture
/// (`packages/testing/nros-tests/bins/ros2-string-interop`).
static NATIVE_ROS2_STRING_INTEROP_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native-rs-lifecycle-node binary
static NATIVE_LIFECYCLE_NODE_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the `safety-chatter-talker` fixture bin (phase-277 W3.a —
/// was the talker `safety-e2e` feature build).
static NATIVE_TALKER_SAFETY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_TALKER_HEADER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached paths to the three `contract-monitor` parity-fixture bins
/// (RFC-0052 / phase-296 W3b.4/.5): rate-contract pub, age-contract sub, and
/// the `/diagnostics` observer.
static CONTRACT_MONITOR_PUB_BINARY: OnceCell<PathBuf> = OnceCell::new();
static CONTRACT_MONITOR_SUB_BINARY: OnceCell<PathBuf> = OnceCell::new();
static CONTRACT_MONITOR_DIAGSINK_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the `safety-chatter-listener` fixture bin (phase-277 W3.a —
/// was the listener `safety-e2e`-gated second `main`).
static NATIVE_LISTENER_SAFETY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// issue 0441 — the receive-side MessageInfo observer, plain and zero-copy.
static MESSAGE_INFO_OBSERVER_BINARY: OnceCell<PathBuf> = OnceCell::new();
static MESSAGE_INFO_OBSERVER_ZERO_COPY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native-rs-talker binary with link-tls
static NATIVE_TALKER_TLS_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// issue 0711 — the peer-mode pair, built with the multicast transport ON.
static NATIVE_TALKER_PEER_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_LISTENER_PEER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native-rs-listener binary with link-tls
static NATIVE_LISTENER_TLS_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native-rs-action-server binary
static NATIVE_ACTION_SERVER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native-rs-action-client binary
static NATIVE_ACTION_CLIENT_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native-rs-service-server binary
static NATIVE_SERVICE_SERVER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native-rs-service-client binary
static NATIVE_SERVICE_CLIENT_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native-rs-service-client-callback binary (RFC-0041 / Phase 239)
static NATIVE_SERVICE_CLIENT_CALLBACK_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native-rs-custom-msg binary
static NATIVE_CUSTOM_MSG_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the qemu-bsp-talker binary
static QEMU_BSP_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the qemu-bsp-listener binary
static QEMU_BSP_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

// Phase 169.4 — bare-metal MPS2-AN385 DDS fixture statics removed
// (Phase 97.3.mps2-an385 lineage; deleted with the Rust DDS retirement).

/// Cached path to the qemu-serial-talker binary
static QEMU_SERIAL_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the qemu-serial-listener binary
static QEMU_SERIAL_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Phase 207 — cached path to the bare-metal XRCE talker binary.
static QEMU_TALKER_XRCE_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the esp32-qemu-talker binary (ELF)
static ESP32_QEMU_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the esp32-qemu-listener binary (ELF)
static ESP32_QEMU_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

// Phase 169.4b — ESP32-C3 QEMU DDS fixture statics removed alongside
// the Rust DDS retirement (Phase 169.2 deleted the example crates).

/// Cached path to the xrce-talker binary
static XRCE_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the xrce-listener binary
static XRCE_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the xrce-service-server binary
static XRCE_SERVICE_SERVER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the xrce-service-client binary
static XRCE_SERVICE_CLIENT_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the xrce-action-server binary
static XRCE_ACTION_SERVER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the xrce-action-client binary
static XRCE_ACTION_CLIENT_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the xrce-large-msg-test binary
static XRCE_LARGE_MSG_TEST_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the zenoh-stress-test binary
static ZENOH_STRESS_TEST_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the zenoh-stress-test binary built with large subscriber buffer
static ZENOH_STRESS_TEST_LARGE_BUF_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the xrce-stress-test binary
static XRCE_STRESS_TEST_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the xrce-stress-test binary built with a large receive ring
static XRCE_STRESS_TEST_LARGE_BUF_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the qemu-bsp-large-msg-test binary
static QEMU_LARGE_MSG_TEST_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the xrce-serial-talker binary
static XRCE_SERIAL_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the xrce-serial-listener binary
static XRCE_SERIAL_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the px4-stub binary (Phase 233.4 — PX4 XRCE companion).
static PX4_STUB_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the px4 offboard-companion binary (Phase 233.4).
static PX4_COMPANION_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the c-talker binary
static C_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the c-listener binary
static C_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the c-service-server binary
static C_SERVICE_SERVER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the c-service-client binary
static C_SERVICE_CLIENT_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the c-service-client-callback binary (RFC-0041 / Phase 239)
static C_SERVICE_CLIENT_CALLBACK_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the c-action-server binary
static C_ACTION_SERVER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the c-action-client binary
static C_ACTION_CLIENT_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the raw-goal wire probe (issue 0454 / phase-354 W3)
static ACTION_RAW_GOAL_PROBE_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the c-xrce-talker binary
static C_XRCE_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the c-xrce-listener binary
static C_XRCE_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native Rust workspace Entry pkg binary.
static NATIVE_WORKSPACE_RUST_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Phase 264 W4c — cached path to the parameterised workspace Entry pkg binary.
static NATIVE_WORKSPACE_RUST_PARAMS_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_RUST_QOS_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_RUST_SIZING_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_RUST_BRIDGE_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_RUST_BRIDGE_XRCE_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-263 A1 (Track D) — cached paths to the cross-process service entries.
static NATIVE_WORKSPACE_RUST_SERVICE_SERVER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_RUST_SERVICE_CLIENT_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// issue 0096 (regression guard) — cached path to the in-process service entry.
static NATIVE_WORKSPACE_RUST_SERVICE_INPROCESS_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-263 A4 (Track D) — cached paths to the cross-process Fibonacci action entries.
static NATIVE_WORKSPACE_RUST_ACTION_SERVER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_RUST_ACTION_CLIENT_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-263 B2 (Track D) — cached path to the real-time multi-tier workspace Entry.
static NATIVE_WORKSPACE_RUST_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-263 A3 (Track D) — cached path to the managed (lifecycle) workspace Entry.
static NATIVE_WORKSPACE_RUST_LIFECYCLE_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-263 B1 (Track D) — cached paths to the cross-process E2E-safety entries.
static NATIVE_WORKSPACE_RUST_REMAP_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_RUST_SAFETY_TALKER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_RUST_SAFETY_LISTENER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Phase 211.F — cached paths to the per-host workspace Entry pkg binaries.
static NATIVE_WORKSPACE_RUST_ENTRY_ROBOT1_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_RUST_ENTRY_ROBOT2_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native C workspace Entry pkg binary.
static NATIVE_WORKSPACE_C_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-263 Track C — cached paths to the per-host C multihost entries.
static NATIVE_WORKSPACE_C_ENTRY_ROBOT1_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_C_ENTRY_ROBOT2_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_C_SERVICE_SERVER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_C_SERVICE_CLIENT_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_C_ACTION_SERVER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_C_ACTION_CLIENT_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// phase-263 B6 — cached paths to the C custom-msg cross-process entries.
static NATIVE_WORKSPACE_C_CUSTOM_MSG_TALKER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_C_CUSTOM_MSG_LISTENER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// phase-263 B4 — cached paths to the C QoS-override cross-process entries.
static NATIVE_WORKSPACE_C_QOS_TALKER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_C_QOS_LISTENER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// Phase 269 W1 — cached paths to the parameterised C/C++ workspace entries.
static NATIVE_WORKSPACE_C_PARAMS_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_CPP_PARAMS_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// Phase 269 W2 — cached paths to the managed-node (lifecycle) C/C++ workspace entries.
static NATIVE_WORKSPACE_C_LIFECYCLE_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_CPP_LIFECYCLE_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// Phase 270 (#103) — cached path to the wrapper-managed C++ lifecycle entry.
static NATIVE_WORKSPACE_CPP_LIFECYCLE_MANAGED_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// Phase 269 W3 — cached paths to the E2E-safety C/C++ workspace entries (talker + listener).
static NATIVE_WORKSPACE_C_SAFETY_TALKER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_C_SAFETY_LISTENER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_CPP_SAFETY_TALKER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_CPP_SAFETY_LISTENER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// Phase 269 W4 — cached paths to the 2-tier sched-context C/C++ realtime workspace entries.
static NATIVE_WORKSPACE_C_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_CPP_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// Phase 272 W3 — cached path to the rclcpp-shape 2-tier realtime workspace entry.
static NATIVE_WORKSPACE_CPP_RCLCPP_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// Phase 273 W4 — cached path to the sub-node 2-group realtime workspace entry (RFC-0047 proof).
static NATIVE_WORKSPACE_CPP_SUBNODE_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// Phase 273 W4 — cached path to the sub-node portability workspace entry (RFC-0047 portability).
static NATIVE_WORKSPACE_CPP_SUBNODE_PORTABLE_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// phase-263 Track-B language matrix — cached paths to the C++ + MIXED projections of the
/// QoS-override and custom-message cross-process workspace entries.
static NATIVE_WORKSPACE_CPP_QOS_TALKER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_CPP_QOS_LISTENER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_MIXED_QOS_TALKER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_MIXED_QOS_LISTENER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_CPP_CUSTOM_MSG_TALKER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_CPP_CUSTOM_MSG_LISTENER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_MIXED_CUSTOM_MSG_TALKER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_MIXED_CUSTOM_MSG_LISTENER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_CPP_SERVICE_SERVER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_CPP_SERVICE_CLIENT_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_CPP_ACTION_SERVER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_CPP_ACTION_CLIENT_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_MIXED_SERVICE_SERVER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_MIXED_SERVICE_CLIENT_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_MIXED_ACTION_SERVER_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_MIXED_ACTION_CLIENT_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-370 W3 — cached paths to the FreeRTOS POSIX simulator workspace
/// EMBEDDED entries (`nano_ros_entry(BOARD freertos-posix …)`), C and C++.
/// Host processes: FreeRTOS tasks are pthreads and the sockets are the host's,
/// so these are the freertos family's first entries with no QEMU behind them.
static FREERTOS_POSIX_WORKSPACE_C_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static FREERTOS_POSIX_WORKSPACE_CPP_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-263 C2a — cached path to the threadx-linux C workspace EMBEDDED entry
/// (`nano_ros_entry(BOARD threadx-linux …)`, the first embedded LAUNCH entry).
static THREADX_LINUX_WORKSPACE_C_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-263 C2b — cached path to the FreeRTOS (QEMU MPS2-AN385) C workspace embedded
/// entry (`nano_ros_entry(BOARD mps2-an385-freertos …)`, the first QEMU-cross entry).
static FREERTOS_WORKSPACE_C_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NUTTX_WORKSPACE_C_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// phase-281 W3-nuttx — cached path to the 2-tier C++ realtime NuttX entry
/// (`realtime-cpp` nuttx_entry), run by `realtime_tiers_cpp_nuttx_e2e.rs`.
static NUTTX_WORKSPACE_CPP_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// phase-281 W3-nuttx — cached path to the 2-tier C realtime NuttX entry
/// (`realtime-c` nuttx_entry), run by `realtime_tiers_c_nuttx_e2e.rs`.
static NUTTX_WORKSPACE_C_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// phase-281 W3-nuttx — cached path to the 2-tier **Rust** realtime NuttX entry
/// (`realtime-rust` nuttx_entry), run by `realtime_tiers_rust_nuttx_e2e.rs`.
static NUTTX_WORKSPACE_RUST_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NUTTX_RISCV_WORKSPACE_RUST_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-263 C2c — cached paths to the C++ embedded workspace entries (threadx-linux host
/// sim + FreeRTOS QEMU), the C++ siblings of the C2a/C2b C entries.
static THREADX_LINUX_WORKSPACE_CPP_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
static FREERTOS_WORKSPACE_CPP_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
/// phase-274 W3 (#126) — cached path to the 2-tier C++ realtime FreeRTOS entry
/// (`realtime-cpp`), run by `realtime_tiers_cpp_freertos_e2e.rs`.
static FREERTOS_WORKSPACE_CPP_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-281 W2 — cached path to the 2-tier C realtime FreeRTOS entry
/// (`realtime-c`), run by `realtime_tiers_c_freertos_e2e.rs`.
static FREERTOS_WORKSPACE_C_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-263 C2c — cached path to the MIXED (C + C++ + Rust) threadx-linux embedded entry.
static THREADX_LINUX_WORKSPACE_MIXED_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-263 C2c — cached path to the MIXED FreeRTOS QEMU embedded entry (no_std Rust node).
static FREERTOS_WORKSPACE_MIXED_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native C++ workspace Entry pkg binary.
static NATIVE_WORKSPACE_CPP_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-263 Track C — cached paths to the per-host C++ multihost entries.
static NATIVE_WORKSPACE_CPP_ENTRY_ROBOT1_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_CPP_ENTRY_ROBOT2_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native mixed C/C++ workspace Entry pkg binary.
static NATIVE_WORKSPACE_MIXED_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// phase-263 Track C — cached paths to the per-host mixed multihost entries.
static NATIVE_WORKSPACE_MIXED_ENTRY_ROBOT1_BINARY: OnceCell<PathBuf> = OnceCell::new();
static NATIVE_WORKSPACE_MIXED_ENTRY_ROBOT2_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Build the qemu-test example and return its path
///
/// Uses OnceLock to cache the build, so subsequent calls are fast.
pub fn build_qemu_test() -> TestResult<&'static Path> {
    QEMU_TEST_BINARY
        .get_or_try_init(|| {
            let root = project_root();
            let example_dir = root.join("packages/testing/nros-tests/bins/cdr-roundtrip-qemu");

            eprintln!("Building qemu-test...");

            let mut args = cargo_build_args();
            args.push("--target".to_string());
            args.push("thumbv7m-none-eabi".to_string());

            let output = cmd("cargo", args)
                .dir(&example_dir)
                .stderr_to_stdout()
                .stdout_capture()
                .unchecked()
                .run()
                .map_err(|e| TestError::BuildFailed(e.to_string()))?;

            if !output.status.success() {
                return Err(TestError::BuildFailed(
                    String::from_utf8_lossy(&output.stdout).to_string(),
                ));
            }

            let binary_path = example_dir.join(format!(
                "target/thumbv7m-none-eabi/{}/qemu-rs-test",
                cargo_target_profile_dir()
            ));

            if !binary_path.exists() {
                return Err(TestError::BuildFailed(format!(
                    "Binary not found after build: {}",
                    binary_path.display()
                )));
            }

            Ok(binary_path)
        })
        .map(|p| p.as_path())
}

/// Build an example from the examples directory
///
/// # Arguments
/// * `name` - Example directory name (e.g., "native-rs-talker")
/// * `binary_name` - Actual binary name (e.g., "talker")
/// * `features` - Optional features to enable
/// * `target` - Optional target triple (e.g., "thumbv7m-none-eabi")
///
/// # Returns
/// Path to the built binary
/// Verify a test-fixture binary was prebuilt — the only contract.
/// Tests must not compile fixtures inside their bodies; the build phase
/// belongs to `just build-test-fixtures`, which sequences cargo/cmake/west
/// invocations cooperatively instead of letting them race with the host's
/// QEMU + zenohd test load. Builds inside test bodies historically
/// stretched a 14 s test to 125 s on a saturated host.
/// phase-319 W3 (issue 0351) — read the build stage's failure marker for a
/// fixture artifact, if one was dropped.
///
/// `compile-check-fixtures.sh` writes `.build-failed` into the fixture's stamp
/// directory when its build fails, recording which builder failed. The resolver
/// walks up from the artifact because an artifact can sit several levels below
/// the stamp dir (`<id>/target/debug/bin`, `<id>/src/entry/entry`); the search
/// stops at the two known roots so it can never wander into an unrelated tree.
fn build_failure_marker(binary_path: &Path) -> Option<String> {
    // RFC-0070 R3 — the same derivation the writer and the stale probe use.
    let roots = [
        build_dir(crate::kind::COMPILE_CHECK, &[]),
        build_dir(crate::kind::CMAKE_FIXTURES, &[]),
    ];
    let mut cur = binary_path.parent()?;
    loop {
        // Only consider dirs beneath a known fixture root.
        if roots.iter().any(|r| cur.starts_with(r)) {
            let marker = cur.join(".build-failed");
            if let Ok(body) = std::fs::read_to_string(&marker) {
                return Some(body);
            }
        }
        cur = cur.parent()?;
        if roots.iter().any(|r| cur == r) || cur.as_os_str().is_empty() {
            return None;
        }
    }
}

/// [`require_prebuilt_binary`] for a caller that already selected its manifest
/// row — issue 0517 step 1.
///
/// Takes the row instead of re-deriving it from a path. The lane check is by
/// COORDINATE and the redirect is by `shared`/`slug`, both read off the row, so
/// nothing here needs a per-variant leaf directory to exist. That is the whole
/// reason `target_dir` survived phase B: `attribute_path` needed the leaf path
/// to tell two rows of one leaf apart, and this is the same answer without the
/// path.
///
/// `rel` is what cargo writes BELOW the artifact root (`[<triple>/]<profile>/
/// <bin>`) — the redirect is a root rewrite and never touches those components.
/// Issue 0608 — rewrite `rel`'s PROFILE component to the one the row's platform
/// is actually built at.
///
/// Every caller builds `rel` from `cargo_target_profile_dir()`, the AMBIENT
/// profile, because that is all a caller knows. But NuttX and FreeRTOS-QEMU
/// cargo fixtures carry a profile carve-out (`nros-minsizerel`), so a
/// group-built row for those platforms lives under a directory the ambient name
/// never spells — the binary was right there, one directory over, reported as
/// MISSING.
///
/// Done here rather than at the nine call sites for the reason the comment in
/// `require_prebuilt_binary` gives about its own redirect: the funnels are not
/// the whole class, and fixing a subset of resolvers is the #328 shape. The
/// leaf resolver and the staleness probe already apply the carve-out; this is
/// the third consumer that has to, and the last one that did not.
fn rel_at_row_profile(row: &crate::fixtures::groups::GroupRow, rel: &Path) -> PathBuf {
    let (platform, _lang, _rmw) = &row.coord;
    let Some(profile) = nros_cargo_profile::platform_profile(platform) else {
        return rel.to_path_buf();
    };
    let want = nros_cargo_profile::target_dir(profile);
    let ambient = cargo_target_profile_dir();
    if want == ambient {
        return rel.to_path_buf();
    }
    // Replace only the component that IS the ambient profile dir. A blind
    // string replace would also rewrite a binary or triple that happened to
    // share the name.
    rel.iter()
        .map(|c| {
            if c == std::ffi::OsStr::new(&ambient) {
                std::ffi::OsString::from(&want)
            } else {
                c.to_os_string()
            }
        })
        .collect()
}

pub(crate) fn require_prebuilt_row_binary(
    row: &crate::fixtures::groups::GroupRow,
    rel: &Path,
) -> TestResult<PathBuf> {
    crate::fixtures::lane::require_coord_in_lane(&row.coord, &row.dir)?;
    let rel = rel_at_row_profile(row, rel);
    let binary_path = crate::fixtures::groups::row_resolved_dir(row).join(&rel);
    require_prebuilt_binary_checks(&binary_path)
}

pub(crate) fn require_prebuilt_binary(binary_path: &Path) -> TestResult<PathBuf> {
    // phase-340 W3 (issue 0482) — a coordinate-scoped RUN skips exactly the
    // manifest rows its lane's BUILD was told to omit. BEFORE the existence
    // check on purpose: the whole point is that an out-of-lane fixture reports
    // as skipped rather than as a missing binary. In-lane fixtures fall through
    // to every check below unchanged, so this can never launder "never built"
    // into "skipped" — see `fixtures::lane`.
    crate::fixtures::lane::require_in_lane(binary_path)?;
    // phase-340 B2 — and AFTER the lane check, also on purpose: lane narrowing
    // attributes a path to its manifest row via the LEAF artifact root, which
    // the redirect below replaces. Reordering these two silently disables
    // coordinate-scoped runs for every migrated platform.
    //
    // The redirect is at the chokepoint rather than at the two `build_example*`
    // funnels because the funnels are not the whole class: `bins/int32-sink`,
    // `bins/param-chatter-talker` and ~30 siblings spell their leaf `target/`
    // path inline, and each of those is a manifest row that the fixture build
    // will redirect the moment its platform is migrated. Fixing only the
    // funnels is the #328 shape (4 resolvers fixed, ~30 left).
    let binary_path = &crate::fixtures::groups::resolved(binary_path);
    require_prebuilt_binary_checks(binary_path)
}

/// Everything both chokepoints do once the path is final: existence, the
/// build-failure marker, and the tier-aware skip. Split out so the row-keyed
/// entry point cannot drift from the path-keyed one.
fn require_prebuilt_binary_checks(binary_path: &Path) -> TestResult<PathBuf> {
    if binary_path.exists() {
        return Ok(binary_path.to_path_buf());
    }
    // Tier-aware (#25): the LIGHT host-integration lane (`NROS_FIXTURES_OPTIONAL=1`)
    // does not build every native fixture variant (TLS / cyclonedds / zero-copy /
    // workspace-entry need extra system deps + tools). There an unstaged fixture
    // is an environment-conditional skip, not a failure — `skip!` ([SKIPPED]) so
    // the [SKIPPED]-aware recipe treats it as a skip. The FULL `test-all` tier
    // leaves the var unset and still hard-fails, surfacing any real fixture gap.
    // phase-319 W3 (issue 0351) — a BROKEN fixture is not a skip in any tier.
    //
    // The light tier's skip is right for "this machine lacks the toolchain" and
    // wrong for "the build stage ran and FAILED", and until now the resolver
    // could not tell them apart: both present as a missing artifact. That is how
    // issue 0350 stayed green — `compile-check-fixtures.sh` was failing wholesale
    // while the lane people run locally reported skips.
    //
    // The build stage now drops a `.build-failed` marker beside the artifact when
    // a fixture's build fails, so the two cases are distinguishable. A marker
    // means the machine COULD build it and did not, which is a hard error
    // everywhere.
    if let Some(reason) = build_failure_marker(binary_path) {
        return Err(TestError::BuildFailed(format!(
            "Test fixture FAILED to build (not merely absent): {}\n\
             {}\n\
             This is a hard error in every tier — the build stage ran and could \
             not produce it. Fix the build; do not re-run with the fixture missing.",
            binary_path.display(),
            reason.trim(),
        )));
    }
    if std::env::var_os("NROS_FIXTURES_OPTIONAL").is_some() {
        crate::skip!(
            "fixture binary not prebuilt: {} (light tier; run `just build-test-fixtures` for full coverage)",
            binary_path.display()
        );
    }
    // Issue 0584 part 2 — an ABSENT in-lane fixture is not a skip.
    //
    // The ~500 `skip!` call sites turn this `Err` into `[SKIPPED] … not
    // prebuilt`, so a lane whose fixtures never got built reports skips and
    // greens. That is the same silent-green shape as 0445 (an absorbing STALE
    // verdict) and 0196 (a gate narrower than the rule it enforces): the run
    // says nothing failed because nothing ran.
    //
    // A missing fixture is not an environment fact. It is a broken promise: a
    // gated run has already asserted, at `_require-fixtures` /
    // `check-fixtures-stale`, that this lane's fixtures exist and are fresh. If
    // one is missing anyway, either the gate's coverage is wrong or the build
    // stage failed quietly — both are bugs, and both must be loud.
    //
    // So when a gate context is present, PANIC rather than return: the panic
    // carries no `[SKIPPED]` marker, so no call site can launder it into a skip
    // and the junit rewrite counts it as the real failure it is. Ungated runs
    // (a bare `cargo nextest` on a developer box that built nothing) keep the
    // old `Err`, because there no one promised anything.
    if gate_promised_fixtures() {
        panic!(
            "Test fixture binary MISSING for an in-lane coordinate: {}\n\
             A gated run already asserted this lane's fixtures are built and \n\
             fresh, so this is a broken promise, not an environment skip.\n\
             THREE things it can be, in the order worth checking (issue 0588 \n\
             burned an issue cycle on the wrong one because this message \n\
             offered only the last two):\n\
             \n\
             1. the NAME is wrong. A test may be asking for a ROS NODE name \n\
                where the package's only `[[bin]]` is called something else \n\
                — `add_two_ints_server` is the node, `service-server` is the \n\
                binary. `find build examples -name <the name>` returning \n\
                nothing is evidence about the NAME, not about the builder.\n\
             2. the build failed quietly. Read the PER-STAGE log, \n\
                tmp/build-test-fixtures-*/<family>.log — the driver only \n\
                tails it on failure, so its own summary shows `== native ==` \n\
                and `== native == OK` one line apart, and a `build=0` line \n\
                from a DIFFERENT builder reads like this one's.\n\
             3. the staleness gate does not cover this row.\n\
             \n\
             Build it:   just build-test-fixtures\n\
             Ungate:     unset NROS_TEST_SCOPE / NROS_TEST_COORDS (then it \n\
             degrades to a skip again), or NROS_FIXTURES_OPTIONAL=1 for the \n\
             light tier.",
            binary_path.display()
        );
    }
    // Issue 1016 — an UNGATED run against a coordinate-scoped BUILD.
    //
    // We are here only when nothing promised these fixtures (the panic above
    // covers the gated case), i.e. a bare `cargo nextest` — which is what
    // CLAUDE.md's own triage advice tells you to type when re-running a red
    // cell SOLO. If the last fixture build recorded a narrow lane and this
    // artifact's coordinate is not in it, then "not prebuilt" is true and
    // useless: the fact is about the LANE, and the verdict a reader sees is
    // indistinguishable from a cell that ran and failed. `[SKIPPED:lane]` says
    // what actually happened, and is counted apart from `capability` (0584).
    //
    // Only on the failure path, and only against the build's OWN record — so
    // this cannot turn a present, fresh fixture into a skip, which is the
    // issue-0445 hazard.
    if let Some(reason) = crate::fixtures::lane::recorded_build_omits(binary_path) {
        crate::skip_class!(lane, "not built: {}\n  {reason}", binary_path.display());
    }
    Err(TestError::BuildFailed(format!(
        "Test fixture binary not prebuilt: {}\n\
         Run `just build-test-fixtures` first.",
        binary_path.display()
    )))
}

/// Did something already PROMISE that this lane's fixtures are present?
///
/// True when the run carries a lane scope — `NROS_TEST_SCOPE` (tier 1 narrows
/// by name) or `NROS_TEST_COORDS` (tier 2 / nightly narrow by coordinate).
/// Both are set by the `just` recipes that run `_require-fixtures` and
/// `check-fixtures-stale` first, so their presence marks "a gate ran".
///
/// `NROS_FIXTURES_OPTIONAL` is the explicit opt-out and is handled by the
/// caller above, before this is consulted.
fn gate_promised_fixtures() -> bool {
    std::env::var_os("NROS_TEST_SCOPE").is_some() || std::env::var_os("NROS_TEST_COORDS").is_some()
}

// The dep-info PARSE (`split_dep_info_line` / `dep_file_paths`) moved to
// `fixtures::staleness` in phase-395 W10, so the shadow cache key reads the
// compiler's recorded input set through the same reader this probe does rather
// than growing a second parser beside it. The mtime policy stays HERE, where it
// belongs — see `dep_file_newer_than_for`.

/// Detect-only staleness probe (issue #147 / phase-278). Reads cargo's
/// `<binary>.d` dep-info file — the make-style `TARGET: DEP DEP …` list of
/// every source input (incl. shared crates + generated msg crates) cargo
/// records for its own incrementality — and returns the first source whose
/// mtime is newer than the binary. This reads the toolchain's recorded
/// dependency graph and `stat()`s files; it never invokes the compiler, so it
/// cannot trigger a rebuild.
///
/// Returns `None` (treated as fresh) when the `.d` is absent (non-cargo
/// binary / older cargo) or unreadable — no regression for callers that only
/// need the existence contract. A dep listed in the `.d` that no longer exists
/// on disk is ignored (conservative: a GC'd registry/intermediate path must
/// not flag a false-stale; an edited source is present with a newer mtime and
/// is caught).
fn dep_info_newer_source(binary_path: &Path) -> Option<PathBuf> {
    let Ok(bin_mtime) = fs::metadata(binary_path).and_then(|m| m.modified()) else {
        return None;
    };
    // phase-353 W2 — pass the artifact so a newer MTIME is checked by CONTENT
    // before it is called stale, and record the baseline when the verdict is
    // fresh. Recording here rather than at the call site keeps the two halves
    // (which deps were walked, and the verdict) in one place; splitting them is
    // how the arms diverged in issue 0442.
    let (newer, walked) = dep_file_newer_than_for(&binary_path.with_extension("d"), bin_mtime);
    let Some(newer) = newer else {
        // Fresh by mtime. Record/refresh the baseline NOW, while the artifact is
        // known good, so a later byte-identical rewrite has something to be
        // compared against. Recording only when something already looks newer
        // would leave the FIRST treadmill event with no baseline, which is the
        // event this exists to forgive.
        let _ = staleness::candidates_changed_content_policy(binary_path, &walked, true);
        return None;
    };
    // An mtime moved. Only the BYTES decide, through the one shared helper
    // (#147 / phase-286 W2, now in `staleness`) that the zephyr arm has used
    // since it was written. Called ONCE with the whole dep set, the way zephyr
    // calls it — per-dep calls would each rewrite the baseline with a single
    // entry.
    match staleness::candidates_changed_content_policy(binary_path, &walked, false) {
        Some(true) => Some(newer), // genuinely edited
        Some(false) => None,       // identical bytes: a rewrite, not an edit
        None => Some(newer),       // cannot tell → keep the old, strict answer
    }
}

// The exemption rule (which candidate mtimes are edit events and which are
// build side-effects) lives in `fixtures::staleness`, ONE spelling shared by
// every probe arm below. It used to live here as two predicates each arm
// called à la carte, and the arms diverged: `newest_source_after` skipped the
// in-place headers but not `OUT_DIR` products, the ninja arm the same, the
// dep-info arm both. That divergence is issue 0442 — every freertos /
// threadx-linux C and C++ zenoh fixture read STALE against a cbindgen header
// one arm was already exempting, so those cells stopped running and issue 0444
// hid behind them for as long as it lasted.

/// phase-363 W5 — the CONFIGURE inputs the Zephyr build itself recorded.
///
/// The leaf's candidate list cannot reach what west/cmake read to CONFIGURE the
/// image, and that set is neither small nor inert: measured on
/// `build-c-talker-cyclonedds`, ninja records 3291 in-repo configure inputs,
/// among them
///
///   * `cmake/NanoRos*.cmake` — the shared modules; an edit changes every image;
///   * the `nros` CLI itself — since phase-432 W2.6 it GENERATES the entry TU
///     for `nano_ros_node_register` too (`nros codegen entry-node`), which is
///     why `nros_codegen_tool_reconfigure()` puts it in
///     `CMAKE_CONFIGURE_DEPENDS` (issue 1018);
///   * `cmake/zephyr/native-sim-line-3.7.conf`, `cmake/compat/stubs/*`.
///
/// A hand list cannot enumerate those without becoming a second copy of the
/// build graph, which is the guessing this phase exists to retire.
///
/// Returns PATHS for the content-aware check, deliberately NOT an mtime verdict.
/// The first cut compared mtimes here and reported every image stale, because at
/// 3291 files any `just format` or pull touches something — the treadmill #147
/// exists to avoid. A measured input set still needs a content comparison; the
/// two halves of "was this built from what is on disk" are independent.
///
/// Paths under the build root are skipped: they are this build's own output.
fn ninja_configure_deps(build_root: &Path) -> Vec<PathBuf> {
    let Ok(text) = fs::read_to_string(build_root.join("build.ninja")) else {
        return Vec::new();
    };
    // `$` continues a line in ninja syntax; the edge is one logical line.
    let unfolded = text.replace("$\n", " ");
    let Some(line) = unfolded
        .lines()
        .find(|l| l.starts_with("build build.ninja:"))
    else {
        return Vec::new();
    };
    let Some((_, deps)) = line.split_once('|') else {
        return Vec::new();
    };
    let root = crate::project_root();
    deps.split_whitespace()
        .map(Path::new)
        .filter(|p| p.is_absolute() && p.starts_with(&root) && !p.starts_with(build_root))
        .filter(|p| p.is_file())
        .map(|p| p.to_path_buf())
        .collect()
}

/// Return the first dependency listed in a make-style `.d` dep-info file whose
/// mtime is newer than `reference`. Shared by the direct-cargo probe
/// ([`dep_info_newer_source`], `.d` next to the binary) and the Zephyr probe
/// (the west staticlib's `.d` vs the linked `zephyr.exe`, which are different
/// artifacts). Missing/unreadable `.d` → `None` (treated fresh).
fn dep_file_newer_than(dep_file: &Path, reference: std::time::SystemTime) -> Option<PathBuf> {
    dep_file_newer_than_for(dep_file, reference).0
}

/// [`dep_file_newer_than`], plus the dep list it walked.
///
/// phase-353 W2 direction (3) — the dep list comes back so the caller can put
/// the whole set through the CONTENT check. `git pull --rebase`,
/// `git stash push/pop` and a branch switch all rewrite tracked files whose
/// bytes are identical, and every prebuilt fixture then reads STALE for no
/// reason; a cold Zephyr leaf costs ~28 s (issue 0509, issue 0466).
///
/// mtime remains the cheap FIRST gate, so a warm tree does no extra work: the
/// content check runs only once something already looks newer.
fn dep_file_newer_than_for(
    dep_file: &Path,
    reference: std::time::SystemTime,
) -> (Option<PathBuf>, Vec<PathBuf>) {
    let mut walked: Vec<PathBuf> = Vec::new();
    for dep_path in staleness::dep_file_paths(dep_file) {
        if staleness::note_candidate(&dep_path) {
            continue;
        }
        walked.push(dep_path.clone());
        if let Ok(dep_mtime) = fs::metadata(&dep_path).and_then(|m| m.modified())
            && dep_mtime > reference
        {
            return (Some(dep_path), walked);
        }
    }
    (None, walked)
}

/// Existence contract PLUS a detect-only staleness check (issue #147 /
/// phase-278): a fixture whose source is newer than its built binary
/// hard-fails "… is STALE" instead of silently running the old binary. Works
/// under ANY launcher (incl. a bare `cargo nextest run`), unlike the
/// `just test-all` preflight. Bypassable with `NROS_SKIP_FIXTURE_CHECK=1`
/// (same knob the preflight honours) for the "built them another way" case.
/// [`require_prebuilt_binary_fresh`] for a caller that already selected its row
/// (issue 0517 step 1). Same staleness probe, on the resolved path.
pub(crate) fn require_prebuilt_row_binary_fresh(
    row: &crate::fixtures::groups::GroupRow,
    rel: &Path,
) -> TestResult<PathBuf> {
    let resolved = require_prebuilt_row_binary(row, rel)?;
    if std::env::var_os("NROS_SKIP_FIXTURE_CHECK").is_some() {
        return Ok(resolved);
    }
    staleness::begin_probe();
    if let Some(newer) = dep_info_newer_source(&resolved) {
        return Err(staleness::stale_error(
            "Test fixture",
            &resolved,
            &newer,
            "Run `just build-test-fixtures` first \
             (or set NROS_SKIP_FIXTURE_CHECK=1 if you built it another way).",
        ));
    }
    staleness::record_fresh(&resolved).map_err(TestError::BuildFailed)?;
    // phase-395 W10 — shadow-mode cache observation, OFF unless
    // `NROS_FIXTURE_CACHE_SHADOW` is set. It records; it cannot skip anything,
    // it cannot serve anything, and it cannot fail this resolution. This arm
    // already HAS the row, so the coordinate comes from `row.coord` rather than
    // from re-attributing a path the shared group dir cannot attribute.
    crate::fixtures::cache_key::observe_row_and_record_if_enabled(row, &resolved);
    Ok(resolved)
}

pub(crate) fn require_prebuilt_binary_fresh(binary_path: &Path) -> TestResult<PathBuf> {
    let resolved = require_prebuilt_binary(binary_path)?;
    if std::env::var_os("NROS_SKIP_FIXTURE_CHECK").is_some() {
        return Ok(resolved);
    }
    staleness::begin_probe();
    // phase-340 B2 — probe the RESOLVED path, not the authored one.
    // `require_prebuilt_binary` may have redirected a leaf-local path onto its
    // shared cargo group dir, and the `.d` dep-info file lives beside the
    // artifact cargo actually wrote. Probing the leaf path instead would find
    // no `.d`, report `None`, and treat every redirected fixture as fresh
    // forever — a museum binary with a passing freshness check, which is issue
    // 0196 in the resolver.
    // The AUTHORED path is what the manifest can attribute to a row (a shared
    // group dir names a platform, not a row), so the shadow observer below
    // keeps it while the probe uses the resolved one.
    let authored = binary_path;
    let binary_path: &Path = &resolved;
    if let Some(newer) = dep_info_newer_source(binary_path) {
        return Err(staleness::stale_error(
            "Test fixture",
            binary_path,
            &newer,
            "Run `just build-test-fixtures` first \
             (or set NROS_SKIP_FIXTURE_CHECK=1 if you built it another way).",
        ));
    }
    staleness::record_fresh(binary_path).map_err(TestError::BuildFailed)?;
    // phase-395 W10 — shadow-mode cache observation, OFF unless
    // `NROS_FIXTURE_CACHE_SHADOW` is set. It records; it cannot skip anything,
    // it cannot serve anything, and it cannot fail this resolution.
    crate::fixtures::cache_key::observe_and_record_if_enabled(authored);
    Ok(resolved)
}

// phase-336 — the profile table is `nros-cargo-profile`, shared with the CLI
// verb the build scripts and cmake call. These three used to re-implement it,
// which is how a test could look for a fixture in a directory the builder never
// wrote to.
fn cargo_profile_name() -> String {
    // EMPTY counts as unset. The justfile exports `NROS_CARGO_PROFILE := ""` so
    // the table owns the default (phase-336 W3) — but `env::var` returns
    // `Ok("")` for a set-but-empty variable, so `unwrap_or_else` never fires and
    // the profile name became "". Every fixture path then resolved through
    // `target//<binary>` and 110 tests reported their binary as not prebuilt.
    nros_cargo_profile::profile_or_default(env::var("NROS_CARGO_PROFILE").ok().as_deref())
}

pub(crate) fn cargo_target_profile_dir() -> String {
    nros_cargo_profile::target_dir(&cargo_profile_name())
}

/// The `target/` profile DIRECTORY a row's platform builds its cargo fixtures
/// into — the platform's carve-out when it has one, the ambient profile
/// otherwise.
///
/// Issue 1027 / issue 0608. A resolver that already holds its manifest row has
/// no reason to name a profile constant: the row carries the coordinate, and
/// `nros_cargo_profile::platform_profile` is THE platform→profile derivation
/// (its shell twin is `nros_cargo_platform_profile`). Naming
/// `NUTTX_RUST_PROFILE` at a call site is the same defect one level down — it
/// hardcodes an answer that a table already gives, so a platform whose carve-out
/// changes leaves the resolver looking in a directory the builder stopped
/// writing.
///
/// This is deliberately the SAME derivation [`rel_at_row_profile`] applies to a
/// caller-built `rel`, exposed so a resolver can ask "which profile is on disk?"
/// BEFORE it hands a path over — which is what the NuttX carve-out/ambient
/// choice needs.
pub(crate) fn row_profile_dir(row: &crate::fixtures::groups::GroupRow) -> String {
    let (platform, _lang, _rmw) = &row.coord;
    nros_cargo_profile::platform_profile(platform)
        .map(nros_cargo_profile::target_dir)
        .unwrap_or_else(cargo_target_profile_dir)
}

fn cargo_build_args() -> Vec<String> {
    let mut args = vec!["build".to_string()];
    args.extend(nros_cargo_profile::build_args(&cargo_profile_name()));
    args
}

pub fn build_example(
    name: &str,
    binary_name: &str,
    _features: Option<&[&str]>,
    target: Option<&str>,
) -> TestResult<PathBuf> {
    let root = project_root();
    let example_dir = root.join(format!("examples/{}", name));

    if !example_dir.exists() {
        return Err(TestError::BuildFailed(format!(
            "Example directory not found: {}",
            example_dir.display()
        )));
    }

    let profile_dir = cargo_target_profile_dir();
    let rel = PathBuf::from(match target {
        Some(target) => format!("{target}/{profile_dir}/{binary_name}"),
        None => format!("{profile_dir}/{binary_name}"),
    });

    // issue 0517 step 3 — the PLAIN row of this leaf. `_features` has been an
    // ignored parameter since it was added, and this is what every caller of
    // this funnel actually means: the default-configuration build. Naming it
    // matters here more than anywhere else, because these are the multi-row
    // leaves — `examples/native/rust/talker` has five — so this is the funnel
    // that would otherwise still need `<dir>/target` to be distinct from
    // `<dir>/target-zenoh`.
    let leaf = format!("examples/{name}");
    if crate::fixtures::groups::leaf_has_rows(&leaf) {
        let row = crate::fixtures::groups::select_row(
            &leaf,
            &crate::fixtures::groups::FixtureVariant::plain(),
        )?;
        return require_prebuilt_row_binary_fresh(row, &rel);
    }
    let binary_path = example_dir.join("target").join(&rel);

    // phase-278 W1 — native rust single-node fixtures carry a cargo `.d`
    // dep-info file; guard against a source-newer-than-binary stale build.
    require_prebuilt_binary_fresh(&binary_path)
}

/// Phase 118 — RMW selector for the per-feature collapsed example dirs.
///
/// Mirror of the per-feature `rmw-{zenoh,dds,xrce}` Cargo features
/// exposed by every `examples/<plat>/<lang>/<case>/Cargo.toml` after
/// the collapse. Build harness picks one feature + the matching
/// `--target-dir target-<rmw>/` so each RMW's incremental state stays
/// isolated from the others (same isolation pattern Phase 88 zero-copy
/// / safety-e2e use).
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Rmw {
    Zenoh,
    Xrce,
    /// Phase 11W — Cyclone DDS. Today exercised by the Zephyr
    /// `prj-cyclonedds.conf` overlay path; native / FreeRTOS /
    /// ThreadX wiring follows once those platforms grow a
    /// cyclonedds backend. (Phase 171.A removed the dead `Rmw::Dds`
    /// dust-DDS variant — dust-DDS retired in Phase 169.)
    Cyclonedds,
}

impl Rmw {
    /// Cargo feature name (`rmw-zenoh` / `rmw-xrce` / `rmw-cyclonedds`).
    pub fn cargo_feature(self) -> &'static str {
        match self {
            Rmw::Zenoh => "rmw-zenoh",
            Rmw::Xrce => "rmw-xrce",
            Rmw::Cyclonedds => "rmw-cyclonedds",
        }
    }

    /// `--target-dir` suffix.
    pub fn target_dir(self) -> &'static str {
        match self {
            Rmw::Zenoh => "target-zenoh",
            Rmw::Xrce => "target-xrce",
            Rmw::Cyclonedds => "target-cyclonedds",
        }
    }

    /// `NROS_RMW` cmake cache value.
    pub fn cmake_value(self) -> &'static str {
        match self {
            Rmw::Zenoh => "zenoh",
            Rmw::Xrce => "xrce",
            Rmw::Cyclonedds => "cyclonedds",
        }
    }

    /// The `rmw` field's spelling in `examples/fixtures.toml` — the third
    /// element of a fixture COORDINATE.
    ///
    /// Delegates to [`cmake_value`](Self::cmake_value) rather than repeating the
    /// match: the manifest and `NROS_RMW` share one vocabulary, and that is a
    /// fact worth stating in code instead of leaving two identical tables to
    /// drift. A separate name because they are separate facts — if the manifest
    /// ever renames a backend, this is the one that moves.
    pub fn coord_token(self) -> &'static str {
        self.cmake_value()
    }

    /// Per-RMW C / C++ build dir name. Same isolation pattern as
    /// `target_dir()` but for cmake.
    pub fn build_dir(self) -> &'static str {
        match self {
            Rmw::Zenoh => "build-zenoh",
            Rmw::Xrce => "build-xrce",
            Rmw::Cyclonedds => "build-cyclonedds",
        }
    }
}

/// Phase 118 — resolve a prebuilt binary for a collapsed-shape example
/// built under a specific RMW feature.
///
/// `name` is the example dir under `examples/` (e.g. `"native/rust/talker"`,
/// without a `<rmw>` axis). `binary_name` is the Cargo `[[bin]] name`.
/// The build is expected to live at
/// `examples/<name>/<rmw.target_dir()>/<profile>/<binary_name>` — the
/// harness asserts the binary exists, mirroring `require_prebuilt_binary`'s
/// contract. The actual `cargo build --no-default-features --features <rmw>
/// --target-dir <rmw.target_dir()>` invocation belongs to
/// `just <plat> build-fixtures`.
pub fn build_example_rmw(name: &str, binary_name: &str, rmw: Rmw) -> TestResult<PathBuf> {
    let root = project_root();
    let example_dir = root.join(format!("examples/{}", name));

    if !example_dir.exists() {
        return Err(TestError::BuildFailed(format!(
            "Example directory not found: {}",
            example_dir.display()
        )));
    }

    // issue 0517 phase B — the artifact root comes from the manifest ROW,
    // selected by the configuration this call already names, instead of from
    // `rmw.target_dir()`. The literal was the row's identity spelt as a
    // directory: correct only while every RMW variant authors `target-<rmw>`,
    // which is the column phase-340 W2.d deletes.
    //
    // `FixtureVariant::rmw` is the feature-selected shape (`--no-default-features
    // --features rmw-<x>`), which is exactly what this function's own doc
    // comment says the build runs.
    let leaf = format!("examples/{name}");
    if crate::fixtures::groups::leaf_has_rows(&leaf) {
        let row = crate::fixtures::groups::select_row(
            &leaf,
            &crate::fixtures::groups::FixtureVariant::rmw(rmw),
        )?;
        let rel = PathBuf::from(format!("{}/{}", cargo_target_profile_dir(), binary_name));
        return require_prebuilt_row_binary_fresh(row, &rel);
    }
    let artifact_root = {
        // A leaf with no manifest row is not this manifest's to place. The live
        // case is `px4/rust/companion/*`, built by `just px4 build-fixtures` —
        // its own lane, its own SDK prerequisites, and no `[[fixture]]` row
        // anywhere. Refusing here would turn a working resolver into an error on
        // every host that has the px4 SDK; keeping the authored spelling is only
        // a fallback for builders the manifest does not describe.
        example_dir.join(rmw.target_dir())
    };
    let binary_path = artifact_root.join(format!("{}/{}", cargo_target_profile_dir(), binary_name));
    // phase-278 W1 — see build_example.
    require_prebuilt_binary_fresh(&binary_path)
}

/// Phase 118 — resolve a prebuilt binary for a collapsed-shape C / C++
/// example built under a specific RMW (cmake `-DNROS_RMW=<rmw>`).
///
/// `name` is the example dir under `examples/` (e.g. `"native/c/talker"`).
/// `binary_name` is the cmake `add_executable` target name. The build
/// is expected to land at
/// `examples/<name>/<rmw.build_dir()>/<binary_name>`. The actual
/// `cmake -B build-<rmw> -S . -DNROS_RMW=<rmw> && cmake --build
/// build-<rmw>` invocation belongs to `just <plat> build-fixtures`.
pub fn build_example_cmake_rmw(name: &str, binary_name: &str, rmw: Rmw) -> TestResult<PathBuf> {
    build_cmake_leaf_rmw(&format!("examples/{}", name), binary_name, rmw)
}

/// The same resolution for a CMake C/C++ leaf that is NOT under `examples/`.
///
/// `rel_dir` is repo-relative (e.g.
/// `"packages/testing/nros-tests/bins/action-raw-goal-probe"`). Regression
/// probes are not examples — examples are standalone copy-out user projects —
/// but they are built by the same `fixtures-build.sh <plat> <lang> <rmw>` group
/// off a `[[fixture]]` row, so they must resolve by the same rule.
/// [`build_example_cmake_rmw`] is a thin wrapper over this, deliberately: one
/// locator, not a second spelling that drifts from it.
pub fn build_cmake_leaf_rmw(rel_dir: &str, binary_name: &str, rmw: Rmw) -> TestResult<PathBuf> {
    let root = project_root();
    let leaf_dir = root.join(rel_dir);

    if !leaf_dir.exists() {
        return Err(TestError::BuildFailed(format!(
            "CMake leaf directory not found: {}",
            leaf_dir.display()
        )));
    }

    let binary_path = leaf_dir.join(format!("{}/{}", rmw.build_dir(), binary_name));
    // phase-278 W2 — the cmake build dir (`build-<rmw>/`) is the binary's parent.
    require_prebuilt_binary_fresh_cmake(&binary_path)
}

/// Detect-only staleness probe for a cmake/ninja cell (issue #147 /
/// phase-278). ninja folds the compiler's `-MD` dep lists into its binary
/// `.ninja_deps` log (no text `.d` files survive), so we read them back with
/// `ninja -t deps` — a pure QUERY that dumps the log, never a build. Returns
/// the first repo-local dependency whose mtime is newer than the binary.
///
/// `build_dir` is the binary's parent (`examples/<name>/build-<rmw>/`). Returns
/// `None` (fresh) when there is no `.ninja_deps` / `ninja` is unavailable /
/// the query fails — existence-only fallback, no regression. System headers
/// (`/usr/...`, outside the repo) are ignored: they never change and pulling
/// their mtime in would only add noise.
fn cmake_dep_info_newer_source(binary_path: &Path) -> Option<PathBuf> {
    let build_dir = binary_path.parent()?;
    if !build_dir.join(".ninja_deps").exists() {
        return None;
    }
    let bin_mtime = fs::metadata(binary_path).ok()?.modified().ok()?;
    // issue 0764 — gather the WHOLE candidate set before deciding, instead of
    // returning on the first newer mtime. The bytes get the last word (below),
    // and the content helper must be called ONCE with every input: called
    // per-dep it would rewrite the baseline with a single entry each time,
    // which is the divergence issue 0442 records.
    let mut candidates: Vec<PathBuf> = Vec::new();
    let mut newer: Option<PathBuf> = None;
    // The `ninja -t deps` QUERY and its repo-local filter live in
    // `staleness::ninja_dep_paths` since phase-395 W10 — one reader, so the
    // shadow cache key measures the same set this arm probes. The mtime policy
    // and the exemption accounting stay here.
    for dep_path in staleness::ninja_dep_paths(build_dir) {
        if staleness::note_candidate(&dep_path) {
            continue;
        }
        if newer.is_none()
            && let Ok(dep_mtime) = fs::metadata(&dep_path).and_then(|m| m.modified())
            && dep_mtime > bin_mtime
        {
            newer = Some(dep_path.clone());
        }
        candidates.push(dep_path);
    }
    // Sources compiled into the fixture by a Rust dep's `build.rs` (via the `cc`
    // crate) are INVISIBLE to ninja's dependency graph above: corrosion invokes
    // `cargo` as one opaque custom command, so `ninja -t deps` never lists e.g.
    // `zpico.c`. A committed edit to that C surface therefore slips past this
    // gate and the fixture runs as a museum binary showing the pre-edit
    // behaviour (issue 0391 — the issue-0196 "watch the same inputs" rule, one
    // level deeper). Close it for the zpico (zenoh-pico) C shim, the one
    // purely-cargo C surface these fixtures link.
    let (zpico_newer, zpico_candidates) = zpico_c_inputs(binary_path, bin_mtime);
    candidates.extend(zpico_candidates);
    let newer = newer.or(zpico_newer);

    // issue 1005 — the arm above asks the BUILD SCRIPT what it READ. That is a
    // different question from what the build script was COMPILED FROM, and the
    // second question has no `rerun-if-changed` answer at all: a build-script
    // DEPENDENCY crate is tracked by cargo through its unit graph, never as a
    // recorded path. `Z_TRANSPORT_LEASE_MS` lives in `nros-zpico-build`, which
    // appears in none of the 41 entries `zpico-sys` records, so every FreeRTOS
    // zenoh fixture read FRESH for ten days while baking a lease value issue
    // 0906 had measured as delivery-breaking (19 of 77 heard vs 77 of 77).
    //
    // Cargo already wrote the complete answer next to the artifact: the
    // corrosion staticlib's own dep-info (`<profile>/libnros_c.d`), which lists
    // EVERY rustc input for that unit — the target-side crates, the C shim the
    // `cc` crate compiled, AND the whole build-script closure
    // (`nros-zpico-build`, `nros-board-common`, `nros-cc-flags`,
    // `nros-build-paths`). So this is the same "ask the tool that owns the
    // graph" move phase-363 made for the `cc` inputs, one level up, read
    // through the SAME `.d` helper the pure-cargo and Zephyr arms already use.
    //
    // Derived, not authored: nothing here enumerates a crate, so a build script
    // that gains a dependency tomorrow is covered without anyone remembering.
    // That is what direction (2) in the issue could not promise, and it is why
    // this needs no generated list + gate the way issue 0627's CLI closure did
    // — that one had to be authored because `source_stamp.rs` cannot shell
    // `cargo metadata`; here cargo has already left its resolve on disk.
    let (cargo_newer, cargo_candidates) = cargo_rust_inputs(build_dir, bin_mtime);
    candidates.extend(cargo_candidates);
    let newer = newer.or(cargo_newer);

    // issue 0764 — the CONTENT decides, through the same shared helper the
    // cargo dep-info arm (`dep_info_newer_source`) and the zephyr arm already
    // use. Before this, THIS arm compared raw mtimes while the build compared
    // bytes: `copy_if_different` correctly skips a byte-identical archive, so
    // nothing relinks, so the binary's mtime stays older than a source whose
    // mtime moved — and the verdict was unclearable BY CONSTRUCTION. Rebuilding
    // could not fix it, which is what separated 0764 from the documented mtime
    // treadmill.
    let Some(newer) = newer else {
        // Fresh by mtime: record/refresh the baseline now, while the artifact is
        // known good, so the FIRST later byte-identical rewrite has something to
        // compare against. Same reasoning as the cargo arm.
        let _ = staleness::candidates_changed_content_policy(binary_path, &candidates, true);
        return None;
    };
    match staleness::candidates_changed_content_policy(binary_path, &candidates, false) {
        Some(true) => Some(newer), // genuinely edited
        Some(false) => None,       // identical bytes: a rewrite, not an edit
        None => Some(newer),       // cannot tell → keep the old, strict answer
    }
}

/// The zpico C shim (`zpico-sys/c/**`) is compiled by `zpico-sys`'s `build.rs`,
/// not by the fixture's cmake/ninja graph, so [`cmake_dep_info_newer_source`]
/// cannot see it. When the fixture is a zenoh-backed cmake build (cyclone
/// fixtures don't link zpico → gate on the `build-zenoh` marker to avoid false
/// stales), compare every `.c`/`.h` under `zpico-sys/c/` against the binary
/// mtime and report the first newer one. Detect-only; never builds.
/// issue 0764 — returns BOTH halves of one resolution: the first input whose
/// mtime is newer than the binary, and the full candidate set to content-hash.
///
/// One function rather than a `_newer` and a `_candidates` sibling, because two
/// functions resolving the same input set is how the probe arms diverged in the
/// first place (issue 0442): the answer and the evidence for it must come from
/// the same walk, or a later edit to one will silently not reach the other.
fn zpico_c_inputs(
    binary_path: &Path,
    bin_mtime: std::time::SystemTime,
) -> (Option<PathBuf>, Vec<PathBuf>) {
    let Some(build_dir) = binary_path.parent() else {
        return (None, Vec::new());
    };
    if !build_dir.to_string_lossy().contains("zenoh") {
        return (None, Vec::new());
    }

    // phase-363 — ASK the tool that owns the graph. `zpico-sys`'s `build.rs`
    // drives the `cc` crate, which emits `cargo:rerun-if-changed` for every path
    // it read; cargo stores that verbatim in the build script's `output`. That
    // set is what the build ACTUALLY consumed, and it is strictly broader than
    // the walk below ever was: it names `nros-platform-api/include` and
    // `config/*/nros-platform.toml`, neither under `zpico-sys/c` nor a `.c`/`.h`,
    // so a platform-config edit used to slip past this gate entirely.
    let recorded = zpico_recorded_inputs(build_dir);
    if !recorded.is_empty() {
        let newer = recorded
            .iter()
            .find_map(|p| newest_path_after(p, bin_mtime));
        // The recorded set may name DIRECTORIES as well as files; the content
        // helper expands them itself (`collect_source_files`), the same way the
        // zephyr arm passes its candidates.
        return (newer, recorded);
    }

    // Bootstrap only: no build script output found, so nothing has recorded an
    // answer for this tree yet. Fall back to the hand-authored walk, which is
    // over-broad and therefore fails SAFE — the same reasoning W4 records for
    // needing a pre-build answer to bootstrap a row that has never built.
    //
    // Measured 2026-08-17: all 76 zenoh fixture build dirs carry a
    // `zpico-sys-*/output`, each with >= 5 in-repo entries, so this arm is
    // unreachable for a built fixture. That is exactly why it announces itself
    // — an arm that never runs is also an arm nobody notices has started
    // running, and a corrosion layout change is all it would take.
    staleness::note_unmeasured_input_set();
    let c_root = zpico_manifest_dir().join("c");
    let newer = newest_source_after(&c_root, bin_mtime);
    (newer, vec![c_root])
}

/// `zpico-sys`'s manifest dir — the base a RELATIVE `rerun-if-changed` entry is
/// written against (cargo resolves them from `CARGO_MANIFEST_DIR`).
///
/// Spelled once and shared by both arms of the probe, so the recorded-input
/// resolution and the bootstrap walk cannot disagree about where that crate is.
fn zpico_manifest_dir() -> PathBuf {
    project_root().join("packages/rmw/zenoh/zpico-sys")
}

/// In-repo paths `zpico-sys`'s build script recorded as its inputs.
///
/// Cargo writes `cargo:rerun-if-changed=<path>` lines into
/// `<target>/<profile>/build/zpico-sys-<hash>/output`. Corrosion puts that tree
/// somewhere under the cmake build dir, and the hash and profile both vary, so
/// the file is located by shape rather than by a spelled path.
fn zpico_recorded_inputs(build_dir: &Path) -> Vec<PathBuf> {
    let root = project_root();
    let mut out = Vec::new();
    for output in find_build_script_outputs(build_dir, "zpico-sys-", 0) {
        let Ok(text) = fs::read_to_string(&output) else {
            continue;
        };
        for line in text.lines() {
            // `cargo::` is the modern spelling, `cargo:` the legacy one; the cc
            // crate still emits the legacy form, and a probe that knew only one
            // would silently record nothing.
            let Some(rest) = line
                .strip_prefix("cargo::rerun-if-changed=")
                .or_else(|| line.strip_prefix("cargo:rerun-if-changed="))
            else {
                continue;
            };
            // Issue 0696 — resolve a RELATIVE entry against the crate that
            // RECORDED it, never against the process CWD.
            //
            // `Path::canonicalize` resolves a relative path against the current
            // directory, and a nextest binary runs with CWD =
            // `packages/testing/nros-tests`. `zpico-sys`'s build script records
            // `rerun-if-changed=src/lib.rs` — its OWN lib.rs — so that entry
            // resolved to the TEST HARNESS's `src/lib.rs`, a file in no
            // fixture's dependency graph. It then passed the in-repo filter
            // below (it really is in-repo), joined the input set, and every
            // native C/C++ fixture read STALE against it after any pull that
            // rewrote that file. No build could clear it, because no build
            // compiles the harness into a C executable.
            //
            // Of the 18 distinct relative entries this output records, that was
            // the ONLY one that resolved from the test CWD — the rest
            // (`cbindgen.toml`, `c/zpico/zpico.c`, …) failed to canonicalize and
            // were silently skipped. One wrong file, always the same one, and
            // the real inputs missing.
            let raw = PathBuf::from(rest.trim());
            let candidate = if raw.is_absolute() {
                raw
            } else {
                zpico_manifest_dir().join(raw)
            };
            // Only in-repo inputs: a system header changing is not this tree's
            // staleness, and `..` segments are why this canonicalizes first.
            let Ok(path) = candidate.canonicalize() else {
                continue;
            };
            if path.starts_with(&root) && !out.contains(&path) {
                out.push(path);
            }
        }
    }
    out
}

/// `<dir>/**/build/<prefix><hash>/output`, bounded so a deep cmake tree cannot
/// turn a staleness probe into a full-tree walk.
fn find_build_script_outputs(dir: &Path, prefix: &str, depth: usize) -> Vec<PathBuf> {
    const MAX_DEPTH: usize = 8;
    let mut found = Vec::new();
    if depth > MAX_DEPTH {
        return found;
    }
    let Ok(entries) = fs::read_dir(dir) else {
        return found;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        // issue 1005 — `is_dir()` (which STATS, following symlinks), never
        // `entry.file_type()` (which LSTATS). Since the phase-340 shared cargo
        // group dir landed, a cross-platform leaf's `build-<rmw>/cargo` is a
        // SYMLINK into `build/corrosion-cargo/<platform>/<hash>/`, so an
        // lstat-based walk stopped at it and this returned EMPTY for every
        // FreeRTOS / NuttX / ThreadX fixture. The caller then fell through to
        // its hand-authored bootstrap walk — the arm whose doc comment says it
        // should be unreachable for a built fixture, announcing itself via
        // `note_unmeasured_input_set()`. It had been running for the whole
        // cross-compiled half of the tree.
        if !path.is_dir() {
            continue;
        }
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if name.starts_with(prefix) {
            let output = path.join("output");
            if output.is_file() {
                found.push(output);
            }
            continue;
        }
        found.extend(find_build_script_outputs(&path, prefix, depth + 1));
    }
    found
}

/// issue 1005 — the cargo-side input closure of a cmake/corrosion fixture,
/// read from the dep-info cargo wrote for the staticlib it links.
///
/// The Rust half of a C/C++ fixture is one opaque `cargo` custom command as far
/// as ninja is concerned, so [`cmake_dep_info_newer_source`]'s ninja arm never
/// sees a single `.rs`. `zpico_c_inputs` closes part of that by replaying what
/// `zpico-sys`'s build script RECORDED, but a recorded `rerun-if-changed` path
/// can only ever name what a build script READ — never the crates the build
/// script itself was COMPILED FROM. That is the class issue 1005 measured:
/// `nros-zpico-build` is a build-DEPENDENCY, tracked correctly by cargo's unit
/// graph and absent from every recorded path.
///
/// Cargo's own dep-info for the staticlib carries both halves. Measured on
/// `examples/qemu-arm-freertos/c/talker/build-zenoh` (2026-09-04),
/// `libnros_c.d` lists 236 in-repo inputs including `packages/core/**` (65),
/// `zpico-sys/c/**` (14) and the build-script closure
/// (`nros-zpico-build/src/lib.rs`, `nros-board-common/src/*`,
/// `nros-cc-flags/src/lib.rs`, `nros-build-paths/src/lib.rs`).
///
/// Returns BOTH halves of one resolution — first newer input, and the whole
/// candidate set to content-hash — for the same reason `zpico_c_inputs` does:
/// two functions resolving one input set is issue 0442.
///
/// Empty when no cargo profile dir is found (a fixture with no Rust half, or a
/// layout this cannot recognise): "nothing was measured", never "nothing
/// changed" — and the arms above still apply.
fn cargo_rust_inputs(
    build_dir: &Path,
    bin_mtime: std::time::SystemTime,
) -> (Option<PathBuf>, Vec<PathBuf>) {
    let mut candidates: Vec<PathBuf> = Vec::new();
    let mut newer: Option<PathBuf> = None;
    for dep_file in cargo_unit_dep_files(build_dir) {
        // issue 0764's rule, and the reason this does NOT call
        // `dep_file_newer_than_for`: that helper RETURNS at the first newer
        // entry, so its `walked` list is truncated there. The ninja arm above
        // deliberately gathers the WHOLE set before deciding, because the
        // content helper must be called once with every input — measured here
        // at 139 of 236 in-repo inputs surviving the early return, with
        // `nros-zpico-build/src/lib.rs` among the 97 that did not.
        for dep_path in staleness::dep_file_paths(&dep_file) {
            if staleness::note_candidate(&dep_path) {
                continue;
            }
            if candidates.contains(&dep_path) {
                continue; // `libnros_c.d` and `libnros_cpp.d` overlap heavily
            }
            if newer.is_none()
                && let Ok(dep_mtime) = fs::metadata(&dep_path).and_then(|m| m.modified())
                && dep_mtime > bin_mtime
            {
                newer = Some(dep_path.clone());
            }
            candidates.push(dep_path);
        }
    }
    (newer, candidates)
}

/// The per-unit `.d` files cargo left at the top level of each profile dir
/// under a cmake build dir (`<group>/<triple>/<profile>/libnros_c.d`).
///
/// The cargo target dir is a phase-340 group dir — usually a symlink to
/// `build/corrosion-cargo/<platform>/<hash>/<group>` shared by every leaf of
/// the family, which is correct here: those leaves link the SAME staticlib, so
/// they have the same Rust input closure.
///
/// Only the top level of a profile dir is read. `deps/*.d` holds one file per
/// intermediate unit, with paths written RELATIVE to the cargo invocation's
/// cwd — the shape issue 0696 records as resolving against the nextest CWD and
/// flagging the wrong file. The staticlib `.d` beside the `.a` is absolute and
/// is already the union of those units.
fn cargo_unit_dep_files(build_dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for profile in find_cargo_profile_dirs(build_dir, 0) {
        let Ok(entries) = fs::read_dir(&profile) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().and_then(|e| e.to_str()) == Some("d") && path.is_file() {
                out.push(path);
            }
        }
    }
    out
}

/// Directories that look like a cargo profile output dir — i.e. that contain a
/// `deps/` child. Bounded like [`find_build_script_outputs`], and it does not
/// descend into cargo's own bulk subtrees, so this stays a handful of
/// `read_dir`s rather than a walk of a cmake tree.
fn find_cargo_profile_dirs(dir: &Path, depth: usize) -> Vec<PathBuf> {
    const MAX_DEPTH: usize = 8;
    let mut found = Vec::new();
    if depth > MAX_DEPTH {
        return found;
    }
    let Ok(entries) = fs::read_dir(dir) else {
        return found;
    };
    let mut children: Vec<PathBuf> = Vec::new();
    let mut is_profile = false;
    for entry in entries.flatten() {
        let path = entry.path();
        // Stat, not lstat: `build-<rmw>/cargo` is a symlink to the phase-340
        // shared group dir on every cross-compiled leaf (see
        // `find_build_script_outputs`). MAX_DEPTH bounds a symlink cycle.
        if !path.is_dir() {
            continue;
        }
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if name == "deps" {
            is_profile = true;
            continue;
        }
        // cargo's bulk subtrees and cmake's own: nothing under them is a
        // profile dir, and `.fingerprint` alone is thousands of entries.
        if matches!(
            name.as_ref(),
            ".fingerprint" | "build" | "incremental" | "examples" | "CMakeFiles" | "_deps"
        ) {
            continue;
        }
        children.push(path);
    }
    if is_profile {
        found.push(dir.to_path_buf());
    }
    for child in children {
        found.extend(find_cargo_profile_dirs(&child, depth + 1));
    }
    found
}

/// First path at or under `p` whose mtime is newer than `bin_mtime`.
///
/// No extension filter, deliberately: cargo named this path an input, so what it
/// is matters less than that it changed. A `rerun-if-changed` entry may be a
/// FILE or a DIRECTORY — `nros-platform-api/include` is the latter — and cargo
/// treats a directory as "anything under it".
fn newest_path_after(p: &Path, bin_mtime: std::time::SystemTime) -> Option<PathBuf> {
    let meta = fs::metadata(p).ok()?;
    if meta.is_file() {
        if staleness::note_candidate(p) {
            return None;
        }
        return (meta.modified().ok()? > bin_mtime).then(|| p.to_path_buf());
    }
    if !meta.is_dir() {
        return None;
    }
    for entry in fs::read_dir(p).ok()?.flatten() {
        if let Some(found) = newest_path_after(&entry.path(), bin_mtime) {
            return Some(found);
        }
    }
    None
}

/// Recursively walk `dir` for `.c`/`.h`/`.cpp`/`.hpp` sources and return the
/// first one whose mtime is newer than `bin_mtime`. Bounded to the given tree;
/// reads directory entries + `stat`, never builds. Symlinks are not followed.
///
/// Exempts through [`staleness::note_candidate`], the rule every arm shares.
/// This walk is a SIBLING of `cmake_dep_info_newer_source`'s ninja-deps loop,
/// and the two carrying DIFFERENT exemption subsets is what made every freertos
/// / threadx-linux C and C++ zenoh fixture read STALE against a cbindgen header
/// the other arm was already exempting (issue 0442; issue #222's cross-family
/// false-stale reached through a new door). A guard narrower than the rule it
/// enforces — issue 0196 — which is why the rule is no longer per-arm.
fn newest_source_after(dir: &Path, bin_mtime: std::time::SystemTime) -> Option<PathBuf> {
    let entries = fs::read_dir(dir).ok()?;
    for entry in entries.flatten() {
        let path = entry.path();
        let file_type = entry.file_type().ok()?;
        if file_type.is_dir() {
            if let Some(found) = newest_source_after(&path, bin_mtime) {
                return Some(found);
            }
        } else if file_type.is_file() {
            let is_source = matches!(
                path.extension().and_then(|e| e.to_str()),
                Some("c" | "h" | "cpp" | "hpp" | "cc" | "hh")
            );
            if is_source
                && !staleness::note_candidate(&path)
                && let Ok(mtime) = fs::metadata(&path).and_then(|m| m.modified())
                && mtime > bin_mtime
            {
                return Some(path);
            }
        }
    }
    None
}

/// Existence contract PLUS the cmake/ninja detect-only staleness check
/// (issue #147 / phase-278). C/C++ analog of [`require_prebuilt_binary_fresh`].
pub(crate) fn require_prebuilt_binary_fresh_cmake(binary_path: &Path) -> TestResult<PathBuf> {
    let resolved = require_prebuilt_binary(binary_path)?;
    if std::env::var_os("NROS_SKIP_FIXTURE_CHECK").is_some() {
        return Ok(resolved);
    }
    staleness::begin_probe();
    if let Some(newer) = cmake_dep_info_newer_source(binary_path) {
        return Err(staleness::stale_error(
            "Test fixture",
            binary_path,
            &newer,
            "Run `just build-test-fixtures` first \
             (or set NROS_SKIP_FIXTURE_CHECK=1 if you built it another way).",
        ));
    }
    staleness::record_fresh(binary_path).map_err(TestError::BuildFailed)?;
    // phase-395 W10 — shadow-mode cache observation, OFF unless
    // `NROS_FIXTURE_CACHE_SHADOW` is set. It records; it cannot skip anything,
    // it cannot serve anything, and it cannot fail this resolution.
    crate::fixtures::cache_key::observe_and_record_if_enabled(binary_path);
    Ok(resolved)
}

/// Locate the west-built Rust app staticlib's `.d` dep-info under a Zephyr
/// build root (`<root>/rust/target/<triple>/<profile>/librustapp.d`). The
/// board triple varies (native_sim = `x86_64-unknown-none`, arm boards
/// differ), so scan the `rust/target/*/<profile>/` dirs rather than hardcode.
fn zephyr_staticlib_dep_file(build_root: &Path) -> Option<PathBuf> {
    let profile = cargo_target_profile_dir();
    let target_root = build_root.join("rust/target");
    for entry in fs::read_dir(&target_root).ok()?.flatten() {
        let cand = entry.path().join(&profile).join("librustapp.d");
        if cand.exists() {
            return Some(cand);
        }
    }
    None
}

/// Which leaf a Zephyr image was built FROM — issue 0466.
///
/// A west-built `zephyr.exe` carries no usable back-pointer to its sources: it
/// is linked from a staticlib plus kernel objects into a build root named by
/// COORDINATE (`build-ws-rs-qos-entry-zenoh`), not by path. So the resolver has
/// to say. Every resolver names its leaf, and
/// [`require_prebuilt_binary_fresh_zephyr`] rejects a dir that does not exist
/// rather than quietly watching nothing — a typo here would reinstate exactly
/// the hole this closes.
#[derive(Copy, Clone, Debug)]
pub(crate) struct ZephyrLeafSource<'a> {
    /// Repo-relative leaf dir, e.g. `examples/workspaces/features/src/zephyr_rust_qos_entry`.
    pub dir: &'a str,
    /// `"c"` / `"cpp"` / `"rust"`; `None` watches every core API crate (the
    /// mixed entry, which links all three).
    pub lang: Option<&'a str>,
    /// RMW token; `None` watches all three backend packages.
    pub rmw: Option<&'a str>,
    /// `;`-separated `-DCONF_FILE` value when the leaf uses one.
    pub conf_files: Option<&'a str>,
}

impl<'a> ZephyrLeafSource<'a> {
    /// The common case: a zenoh leaf of known language, west-default conf files
    /// (`prj.conf` is already in the candidate set).
    pub(crate) fn zenoh(dir: &'a str, lang: &'a str) -> Self {
        Self {
            dir,
            lang: Some(lang),
            rmw: Some("zenoh"),
            conf_files: None,
        }
    }
}

/// Existence contract PLUS a detect-only staleness check for a Zephyr
/// workspace-entry image (issue #147 / phase-278). `zephyr.exe` is LINKED by
/// west from the cargo staticlib `librustapp.a` + C/kernel objects, so its own
/// `.ninja_deps` does not track the Rust entry/node source; the Rust-source
/// staleness (the dominant drift for these entries — see the phase-276
/// stale-image reruns) lives in the staticlib's `.d`. Compare that `.d`'s deps
/// against the LINKED `zephyr.exe` mtime: a Rust source newer than the image
/// means west never relinked. Reads the staticlib `.d` + `stat()`; never
/// builds. Missing `.d` → existence-only fallback.
pub(crate) fn require_prebuilt_binary_fresh_zephyr(
    zephyr_exe: &Path,
    src: ZephyrLeafSource<'_>,
) -> TestResult<PathBuf> {
    // Issue 0713 — the LANE decides first, and it decides ONCE, here.
    //
    // West leaves have manifest rows but their images land in the Zephyr build
    // root rather than under `row_artifact_root`, so `fixtures::lane` cannot
    // attribute them by PATH and its fail-closed arm calls every one in-lane.
    // That arm's justification — "their build is not narrowed either, so
    // nothing is missing" — was true when it was written and stopped being true
    // when phase-350 W1.b narrowed the zephyr BUILD by coordinate. Since then a
    // tier-2 run resolved leaves the build had deliberately skipped and
    // reported a broken promise indistinguishable from a regression: seven of
    // its twelve failures.
    //
    // `get_prebuilt_zephyr_example` already took the coordinate route for its
    // own leaves (issue 0517). Putting the call HERE rather than at the ~14
    // resolvers is the difference between fixing the class and fixing the two
    // sites whose failures happened to be read — the shape CLAUDE.md names
    // (#282's second idiom, #328's ~30 unswept resolvers). It also means a new
    // zephyr resolver gets the narrowing without knowing it exists, the same
    // argument issue 0466 settled one block down for the source-candidate check.
    //
    // An IN-lane coordinate still fails exactly as hard when its image is
    // missing or stale: the skip is keyed on the coordinate, never on absence
    // (issue 0445).
    if let Some(build_name) = crate::fixtures::lane::west_build_name(zephyr_exe) {
        crate::fixtures::lane::require_west_leaf_in_lane(build_name, src.dir)?;
    }

    let resolved = require_prebuilt_binary(zephyr_exe)?;
    if std::env::var_os("NROS_SKIP_FIXTURE_CHECK").is_some() {
        return Ok(resolved);
    }
    staleness::begin_probe();

    // Half 1 — the cargo staticlib's `.d`. Watches the REAL dependency closure
    // of the Rust half (529 entries for a workspace leaf, generated msg crates
    // included), which no hand-written candidate list could enumerate. Absent
    // for a C-only image, which is precisely why it cannot be the only half.
    // `<build-root>/zephyr/zephyr.exe` → `<build-root>`. Every link in the chain
    // is genuinely optional — a C-only image has no staticlib `.d` at all — so a
    // miss anywhere just means half 1 has nothing to say, and half 2 answers.
    if let Some(build_root) = zephyr_exe.parent().and_then(Path::parent)
        && let Ok(exe_mtime) = fs::metadata(zephyr_exe).and_then(|m| m.modified())
        && let Some(dep_file) = zephyr_staticlib_dep_file(build_root)
        && let Some(newer) = dep_file_newer_than(&dep_file, exe_mtime)
    {
        return Err(staleness::stale_error(
            "Zephyr fixture",
            zephyr_exe,
            &newer,
            "Run `just zephyr build-fixtures` first \
             (or set NROS_SKIP_FIXTURE_CHECK=1 if you built it another way).",
        ));
    }

    // phase-363 W5 — the configure inputs the build recorded, handed to the
    // content-aware check below rather than judged on mtime here.
    let configure_inputs = zephyr_exe
        .parent()
        .and_then(Path::parent)
        .map(ninja_configure_deps)
        .unwrap_or_default();

    // Half 2 — issue 0466. The leaf's own AUTHORED sources: `prj.conf`, the
    // board overlays, `CMakeLists.txt`, `src/`, plus the shared core and RMW
    // crates. Half 1 sees none of them: measured on `build-ws-rs-qos-entry-zenoh`,
    // the leaf's `prj.conf` appears ZERO times in the `.d`, whose only `.conf` is
    // the build's own GENERATED one. That is how a `CONFIG_NROS_MAX_QUERYABLES`
    // bump left every image "fresh" with the old value compiled in, and the qos
    // and lifecycle cells failed with product-level assertions that sent two
    // sessions looking at the product instead of the image (0460, then 0466).
    // For a C or C++ entry there is no `.d` at all, so before this half those
    // images were existence-checked and nothing more.
    let src_dir = crate::project_root().join(src.dir);
    if !src_dir.is_dir() {
        // A resolver naming a dir that does not exist would watch NOTHING and
        // silently reinstate the hole. Fail loudly instead.
        return Err(TestError::BuildFailed(format!(
            "Zephyr fixture {} declares source dir `{}`, which does not exist.\n\
             A `ZephyrLeafSource` must name the leaf the image is built from \
             (issue 0466).",
            zephyr_exe.display(),
            src.dir
        )));
    }
    if crate::zephyr::source_dir_is_stale(
        &resolved,
        &src_dir,
        src.lang,
        src.rmw,
        src.conf_files,
        &configure_inputs,
    ) {
        return Err(staleness::stale_error(
            "Zephyr fixture",
            zephyr_exe,
            &src_dir,
            "Run `just zephyr build-fixtures` first \
             (or set NROS_SKIP_FIXTURE_CHECK=1 if you built it another way).",
        ));
    }

    staleness::record_fresh(zephyr_exe).map_err(TestError::BuildFailed)?;
    Ok(resolved)
}

fn workspace_example_dir(name: &str) -> TestResult<PathBuf> {
    let root = project_root();
    let example_dir = root.join(format!("examples/workspaces/{name}"));
    if !example_dir.exists() {
        return Err(TestError::BuildFailed(format!(
            "Workspace example directory not found: {}",
            example_dir.display()
        )));
    }
    Ok(example_dir)
}

/// One workspace row of `examples/fixtures.toml`, as the manifest reader
/// renders it: `\x1f`-separated fields, field 4 the hand-written `entry`
/// and field 13 the generated `image`.
///
/// `pub` so a GATE can read the manifest the same way the resolvers do
/// (`rmw_coordinate_truth`, issue 0831). Re-deriving a row by parsing
/// `fixtures.toml` in a second place is the split that issue 0393 spent a
/// wave collapsing.
pub fn current_workspace_fixture_record(fixture_id: &str) -> TestResult<String> {
    let root = project_root();
    // No `--platform` filter: workspace cmake fixtures span platforms (native +
    // the phase-263 C2a embedded threadx-linux row), and a cmake record is NOT
    // platform-prefixed (only rust cargo records are), so listing all platforms and
    // matching by the unique id prefix resolves any of them with a stable signature.
    let output = Command::new("python3")
        .arg(root.join("scripts/build/fixtures-manifest.py"))
        .arg("list-workspaces")
        .arg("--id")
        .arg(fixture_id)
        .current_dir(&root)
        .output()
        .map_err(|e| {
            TestError::BuildFailed(format!("Failed to run workspace fixture manifest: {e}"))
        })?;

    if !output.status.success() {
        return Err(TestError::BuildFailed(format!(
            "Failed to read workspace fixture manifest:\n{}",
            String::from_utf8_lossy(&output.stderr)
        )));
    }

    let prefix = format!("{fixture_id}\x1f");
    String::from_utf8_lossy(&output.stdout)
        .lines()
        .find(|line| line.starts_with(&prefix))
        .map(ToOwned::to_owned)
        .ok_or_else(|| {
            TestError::BuildFailed(format!(
                "Workspace fixture {fixture_id:?} is not declared in examples/fixtures.toml"
            ))
        })
}

fn current_workspace_fixture_signature(fixture_id: &str) -> TestResult<String> {
    let root = project_root();
    let record = current_workspace_fixture_record(fixture_id)?;
    let output = Command::new("bash")
        .arg(root.join("scripts/build/workspace-fixture-signature.sh"))
        .arg(&record)
        .current_dir(&root)
        .output()
        .map_err(|e| {
            TestError::BuildFailed(format!("Failed to run workspace fixture signature: {e}"))
        })?;

    if !output.status.success() {
        return Err(TestError::BuildFailed(format!(
            "Failed to compute workspace fixture signature:\n{}",
            String::from_utf8_lossy(&output.stderr)
        )));
    }

    Ok(String::from_utf8_lossy(&output.stdout)
        .trim_end()
        .to_owned())
}

fn require_prebuilt_workspace_binary(
    fixture_id: &str,
    binary_path: &Path,
    stamp_path: &Path,
) -> TestResult<PathBuf> {
    // phase-340 W3 — attributed by `id` rather than by path: both workspace
    // resolvers already take the manifest row's id, and several rows share one
    // `dir`, so `id` is the exact key where a path prefix would be ambiguous.
    crate::fixtures::lane::require_workspace_in_lane(fixture_id)?;
    if !binary_path.exists() {
        return Err(TestError::BuildFailed(format!(
            "Workspace fixture binary not prebuilt: {}\n\
             Run `just native build-workspace-fixtures` first.",
            binary_path.display()
        )));
    }

    let expected = current_workspace_fixture_signature(fixture_id)?;
    let actual = fs::read_to_string(stamp_path).map_err(|e| {
        TestError::BuildFailed(format!(
            "Workspace fixture stamp missing for {fixture_id}: {} ({e})\n\
             Run `just native build-workspace-fixtures` first.",
            stamp_path.display()
        ))
    })?;
    if actual.trim_end() != expected {
        return Err(TestError::BuildFailed(format!(
            "Workspace fixture {fixture_id} is stale: {}\n\
             Run `just native build-workspace-fixtures` first.",
            stamp_path.display()
        )));
    }

    Ok(binary_path.to_path_buf())
}

/// A GENERATED row's binary name is DERIVED — `<image>_entry`, the target
/// `nros build` emits — so a resolver that names anything else is wrong, and
/// wrong in the expensive direction: the binary is simply "not there", which is
/// indistinguishable from an absent toolchain, so the cell SKIPS and the lane
/// stays green over a coordinate that never ran (issue 0411).
///
/// The sibling check inside [`build_workspace_cmake_entry_in`] covers a
/// HAND-WRITTEN row by comparing against the manifest's `entry` field. That
/// field is empty by construction for a generated row, so this is the same
/// guard reading the field that does carry the answer — one helper on both
/// resolvers rather than a second spelling of the rule.
///
/// Written after getting it wrong: migrating `safety` (phase-383 W10.a) renamed
/// four resolvers to the bare image name. It built clean and would have skipped
/// all four cells forever.
fn assert_generated_entry_name(fixture_id: &str, binary_name: &str) -> TestResult<()> {
    let Ok(record) = current_workspace_fixture_record(fixture_id) else {
        return Ok(());
    };
    let fields: Vec<&str> = record.split('\x1f').collect();
    let Some(image) = fields.get(13).filter(|f| !f.is_empty()) else {
        return Ok(());
    };
    let want = format!("{image}_entry");
    if want == binary_name {
        return Ok(());
    }
    Err(TestError::BuildFailed(format!(
        "fixture {fixture_id:?} resolves entry {binary_name:?} but its row declares \
         image {image:?}, whose generated target is {want:?} — examples/fixtures.toml \
         is the SSoT, and a resolver naming a different binary reports the fixture as \
         'not built' forever (issue 0411)"
    )))
}

/// Resolve a prebuilt Rust workspace Entry pkg binary.
///
/// The workspace fixture build step owns `nros sync`,
/// `nros codegen-system`, and the Cargo build. Tests only require the
/// resulting binary from the deterministic fixture target dir.
pub fn build_workspace_rust_entry(
    fixture_id: &str,
    workspace: &str,
    binary_name: &str,
) -> TestResult<PathBuf> {
    assert_generated_entry_name(fixture_id, binary_name)?;
    // Kept for its existence check and its error message; the PATH now comes
    // from the row (issue 0517).
    workspace_example_dir(workspace)?;
    // issue 0517 — the row says where its artifacts land (`row_artifact_root`),
    // reached by the `id` this function already takes. `target-fixtures` was the
    // row's identity spelt as a directory.
    let target_dir = crate::fixtures::groups::workspace_artifact_dir(fixture_id)?;
    let binary_path = target_dir.join(format!("{}/{}", cargo_target_profile_dir(), binary_name));
    require_prebuilt_workspace_binary(
        fixture_id,
        &binary_path,
        &target_dir.join(workspace_fixture_stamp_name(fixture_id)),
    )
}

/// Resolve a prebuilt CMake workspace Entry pkg binary.
///
/// The workspace fixture build step owns `nros sync`,
/// `nros codegen-system`, and the CMake configure/build. Tests only
/// require the resulting binary from the deterministic fixture build dir.
pub fn build_workspace_cmake_entry(
    fixture_id: &str,
    workspace: &str,
    binary_name: &str,
) -> TestResult<PathBuf> {
    build_workspace_cmake_entry_in(
        fixture_id,
        workspace,
        "build-workspace-fixtures",
        binary_name,
    )
}

/// Like [`build_workspace_cmake_entry`] but for a fixture configured into a
/// NON-default `build_subdir`. A CMake workspace builds as ONE platform per
/// configure (one board per `CMakeCache.txt`), so an embedded fixture
/// (phase-263 C2a — threadx-linux) cannot share the posix
/// `build-workspace-fixtures` dir; its row sets a distinct `build_subdir` and
/// this resolver consumes the prebuilt binary from there.
pub fn build_workspace_cmake_entry_in(
    fixture_id: &str,
    workspace: &str,
    build_subdir: &str,
    binary_name: &str,
) -> TestResult<PathBuf> {
    // The manifest already says which entry this fixture builds, so a
    // `binary_name` that disagrees with it can only be wrong — and it fails in
    // the most expensive way: the binary is simply "not there", which is
    // indistinguishable from an absent toolchain, so the cell SKIPS and the
    // lane stays green over a coordinate that never ran (issue 0411, the 0350
    // class). phase-331 W2b renamed entries to the platform vocabulary and this
    // resolver kept `native_threadx_entry` for months.
    //
    // Checked here rather than at the one call site that drifted: the argument
    // exists in ~20 places and any of them can go stale the same way.
    if let Ok(record) = current_workspace_fixture_record(fixture_id) {
        let fields: Vec<&str> = record.split('\x1f').collect();
        if let Some(declared) = fields.get(4)
            && !declared.is_empty()
            && *declared != binary_name
        {
            return Err(TestError::BuildFailed(format!(
                "fixture {fixture_id:?} resolves entry {binary_name:?} but \
                 examples/fixtures.toml declares {declared:?} — the manifest is the SSoT; \
                 a resolver that names a different entry reports the fixture as \
                 'not built' forever (issue 0411)"
            )));
        }
    }
    assert_generated_entry_name(fixture_id, binary_name)?;
    // Kept for its existence check and its error message; the PATH now comes
    // from the row (issue 0517).
    workspace_example_dir(workspace)?;
    // issue 0517 — derived from the row, with the caller's `build_subdir` kept as
    // an ASSERTION rather than as the source of truth. Same reasoning as the
    // entry-name check above: the argument exists in ~20 places and any of them
    // can drift, and the failure mode is the expensive one — the binary is simply
    // "not there", which reads as an absent toolchain and SKIPS.
    let build_dir = crate::fixtures::groups::workspace_artifact_dir(fixture_id)?;
    // A MIGRATED row (phase-383 W10.a) builds through `nros build` into
    // `build/<coord>/cmake`, a path the caller has no business restating: the
    // coordinate is derived from the image's board and RMW, so the literal
    // would be a fourth place for one fact to drift. The manifest is the SSoT
    // this assertion exists to defend — for a generated row it IS the answer,
    // so there is nothing to cross-check it against.
    let generated = current_workspace_fixture_record(fixture_id)
        .ok()
        .and_then(|r| r.split('\x1f').nth(13).map(|f| !f.is_empty()))
        .unwrap_or(false);
    if !generated && !build_dir.ends_with(build_subdir) {
        return Err(TestError::BuildFailed(format!(
            "fixture {fixture_id:?} resolves build dir {build_dir:?} but the caller \
             passed build_subdir {build_subdir:?} — examples/fixtures.toml is the SSoT \
             for where a workspace fixture builds (issue 0517)"
        )));
    }
    // A GENERATED entry lands at the TOP of the cmake binary dir; a
    // hand-written one is a subdirectory package and lands under
    // `src/<entry>/`. Decided from the manifest row (its `image` field), not
    // from probing both, because "try one then the other" is how a fixture that
    // was never built reads as one that moved — and the failure is the
    // expensive one this function already guards against twice above: a binary
    // that is simply "not there" SKIPS the cell and leaves the lane green over
    // a coordinate that never ran (issue 0411).
    let binary_path = if generated {
        build_dir.join(binary_name)
    } else {
        build_dir.join(format!("src/{binary_name}/{binary_name}"))
    };
    require_prebuilt_workspace_binary(
        fixture_id,
        &binary_path,
        &build_dir.join(workspace_fixture_stamp_name(fixture_id)),
    )
}

/// Native Rust workspace Entry pkg fixture.
pub fn build_native_workspace_rust_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry("workspace-rust-native", "rust", "native_entry")
        })
        .map(|p| p.as_path())
}

/// Phase 264 W4c — the parameterised native Rust workspace Entry pkg fixture
/// (`features`, built via the pure-cargo `nros::main!` path).
pub fn build_native_workspace_rust_params_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_PARAMS_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-features-rust-params",
                "features",
                "native_rust_params_entry",
            )
        })
        .map(|p| p.as_path())
}

/// issue #52 / 0303 / 0304 — the QoS workspace Entry pkg on native
/// (`features`). Its committed model overrides the talker publisher's
/// reliability to `best_effort`, which the node's code never asks for.
pub fn build_native_workspace_rust_qos_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_QOS_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-features-rust-qos",
                "features",
                "native_rust_qos_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-307 W6 lane 2 (issue 0257) — the executor-sizing showcase Entry
/// (`sizing`). Its node registers six timers the SystemModel cannot
/// count; the entry only links if `nros::main!` read the source-metadata
/// sidecar and derived a table bigger than the four-slot default.
pub fn build_native_workspace_rust_sizing_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_SIZING_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry("workspace-rust-native-sizing", "sizing", "native_entry")
        })
        .map(|p| p.as_path())
}

/// phase-267 W-B — the declarative cross-RMW bridge Entry (`bridge-cyclonedds`,
/// cached). Config-driven pure-cargo `nros::main!` + `run_from_config_str`; the
/// entry links zenoh ingress + cyclonedds egress. Gated on the cyclonedds
/// submodule (the build compiles vendored C++ Cyclone), so the fixture is absent
/// when Cyclone isn't provisioned — callers skip cleanly.
pub fn build_native_workspace_rust_bridge_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_BRIDGE_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-rust-native-bridge",
                "bridge-cyclonedds",
                "native_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-267 (xrce variant) — the declarative `zenoh↔xrce` bridge Entry
/// (`bridge-xrce`, cached). Config-driven pure-cargo `nros::main!`; links
/// zenoh + xrce backends (no cyclonedds submodule gate). The runtime e2e needs a
/// Micro-XRCE-DDS Agent (`XrceAgent`).
pub fn build_native_workspace_rust_bridge_xrce_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_BRIDGE_XRCE_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-rust-native-bridge-xrce",
                "bridge-xrce",
                "native_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A1 (Track D) — the server half of the cross-process AddTwoInts
/// service demo (cached). Pure-cargo `nros::main!` booting `add_server` alone.
pub fn build_native_workspace_rust_service_server_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_SERVICE_SERVER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-rust-native-service-server",
                "rust",
                "native_service_server_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A1 (Track D) — the client half of the cross-process AddTwoInts
/// service demo (cached). `add_client` calls the server entry and republishes
/// the server-computed sum on /sum; the `roundtrip_xprocess_e2e` native_rust_service cell asserts it.
pub fn build_native_workspace_rust_service_client_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_SERVICE_CLIENT_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-rust-native-service-client",
                "rust",
                "native_service_client_entry",
            )
        })
        .map(|p| p.as_path())
}

/// issue 0096 (regression guard) — the in-process service entry (cached). Boots
/// `add_server` AND `add_client` in ONE process/session; `add_client` calls `add_server`
/// on the SAME executor and republishes the sum on `/sum`. Two nodes stay under the
/// default `MAX_CBS = 4`, so it shares `target-fixtures` via the standard builder.
/// `service_roundtrip_inprocess_e2e` asserts the same-session round-trip reaches `/sum`.
pub fn build_native_workspace_rust_service_inprocess_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_SERVICE_INPROCESS_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-rust-native-service-inprocess",
                "rust",
                "native_service_inprocess_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A4 (Track D) — the server half of the cross-process Fibonacci action
/// demo (cached). Pure-cargo `nros::main!` booting `fibonacci_server` alone.
pub fn build_native_workspace_rust_action_server_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_ACTION_SERVER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-rust-native-action-server",
                "rust",
                "native_action_server_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A4 (Track D) — the client half of the cross-process Fibonacci action
/// demo (cached). Sends a goal and republishes the result's last element on
/// `/fib_result`, which the `roundtrip_xprocess_e2e` native_rust_action cell asserts.
pub fn build_native_workspace_rust_action_client_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_ACTION_CLIENT_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-rust-native-action-client",
                "rust",
                "native_action_client_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A3 (Track D) — the managed (lifecycle) workspace Entry (cached). The
/// entry bakes `lifecycle-services` + declares `[lifecycle] autostart = "active"`, so
/// `nros::main!` registers the 5 REP-2002 services and drives Configure→Activate at
/// boot; `workspace_features_e2e` (native_rust_lifecycle) asserts the node reaches `active` via `ros2 lifecycle`.
pub fn build_native_workspace_rust_lifecycle_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_LIFECYCLE_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-features-rust-lifecycle",
                "features",
                "native_rust_lifecycle_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B2 (Track D) — the real-time 2-tier workspace Entry (cached). The
/// `run_tiers` multi-tier entry schedules a high-tier `/ctrl` node (10 ms) and a
/// low-tier `/telem` node (100 ms); `realtime_tiers_e2e` asserts both are scheduled.
pub fn build_native_workspace_rust_realtime_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-rust-native-realtime",
                "realtime-rust",
                "native_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-306 W4 (issue 0255) — the remap/private-name native Rust workspace
/// Entry pkg fixture (`features`, cached; pure-cargo
/// `nros::main!(model = …)`). The model namespaces the node under `/island`
/// and remaps its PRIVATE `~/out` to `/remapped_out` — the wire-name proof
/// lane consumed by `workspace_features_e2e` (native_rust_remap).
pub fn build_native_workspace_rust_remap_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_REMAP_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-features-rust-remap",
                "features",
                "native_rust_remap_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B1 (Track D) — the talker half of the cross-process E2E-safety demo
/// (cached). Bakes `safety-e2e`, so its /chatter publishes carry a backend CRC.
pub fn build_native_workspace_rust_safety_talker_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_SAFETY_TALKER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-safety-rust-talker",
                "safety",
                "native_rust_safety_talker_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B1 (Track D) — the listener half of the cross-process E2E-safety demo
/// (cached). Validates the CRC and republishes the validated count on `/safe_ok`,
/// which `workspace_features_e2e` (native_rust_safety) asserts.
pub fn build_native_workspace_rust_safety_listener_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_SAFETY_LISTENER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-safety-rust-listener",
                "safety",
                "native_rust_safety_listener_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Phase 211.F — `robot1` per-host entry (talker) from `multihost.launch.xml`.
pub fn build_native_workspace_rust_entry_robot1() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_ENTRY_ROBOT1_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-rust-native-robot1",
                "rust",
                // phase-383 W9.b — GENERATED entry: the row names `[image.native_robot1]`
                // and the package name is derived (`<image>_entry`), so this is
                // `native_robot1_entry`, not the pre-migration `native_entry_robot1`.
                "native_robot1_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Phase 211.F — `robot2` per-host entry (listener) from `multihost.launch.xml`.
pub fn build_native_workspace_rust_entry_robot2() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_RUST_ENTRY_ROBOT2_BINARY
        .get_or_try_init(|| {
            build_workspace_rust_entry(
                "workspace-rust-native-robot2",
                "rust",
                // phase-383 W9.b — see robot1 above.
                "native_robot2_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Native C workspace Entry pkg fixture.
pub fn build_native_workspace_c_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_ENTRY_BINARY
        .get_or_try_init(|| build_workspace_cmake_entry("workspace-c-native", "c", "native_entry"))
        .map(|p| p.as_path())
}

/// Phase 269 W1 — the parameterised C workspace Entry pkg fixture (`features`).
/// Reads live `publish_period_ms` via `nros_cpp_get_param_integer`; consumed by
/// tests/cpp_c_param_live_read_e2e.rs.
pub fn build_native_workspace_c_params_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_PARAMS_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-c-params",
                "features",
                "native_c_params_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Phase 269 W2 — the managed-node (lifecycle) C workspace Entry pkg fixture (`features`).
/// Boots the talker node to `active` via `nros_cpp_lifecycle_autostart`; consumed by
/// tests/workspace_features_e2e.rs.
pub fn build_native_workspace_c_lifecycle_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_LIFECYCLE_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-c-lifecycle",
                "features",
                "native_c_lifecycle_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Phase 269 W3 — the talker half of the cross-process E2E-safety C demo (`safety`).
/// Publishes CRC-annotated /chatter frames when built with NANO_ROS_SAFETY_E2E=ON.
/// Consumed by tests/workspace_features_e2e.rs.
pub fn build_native_workspace_c_safety_talker_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_SAFETY_TALKER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-safety-c-talker",
                "safety",
                "build/posix-zenoh-native/cmake",
                "native_c_safety_talker_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Phase 269 W3 — the listener half of the cross-process E2E-safety C demo (`safety`).
/// Uses `nros_cpp_subscription_register_validated`; counts CRC-valid frames.
/// Consumed by tests/workspace_features_e2e.rs.
pub fn build_native_workspace_c_safety_listener_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_SAFETY_LISTENER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-safety-c-listener",
                "safety",
                "build/posix-zenoh-native/cmake",
                "native_c_safety_listener_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 Track C — the robot1 (talker) per-host C multihost entry (cached).
pub fn build_native_workspace_c_entry_robot1() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_ENTRY_ROBOT1_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-c-native-robot1",
                "c",
                // phase-383 W10.a — GENERATED entry: the row names
                // `[image.native_robot1]` and the name is derived
                // (`<image>_entry`), not the pre-migration `native_entry_robot1`.
                "native_robot1_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A1 (services, C) — the AddTwoInts service SERVER single-node entry (cached).
/// Cross-process round-trip (issue 0096); consumed by tests/roundtrip_xprocess_e2e.rs (native_c_service cell).
pub fn build_native_workspace_c_service_server_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_SERVICE_SERVER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-c-native-service-server",
                "c",
                "native_service_server_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A1 (services, C) — the AddTwoInts service CLIENT single-node entry (cached). Calls
/// the server each tick + prints the server-computed sums it receives over the wire.
pub fn build_native_workspace_c_service_client_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_SERVICE_CLIENT_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-c-native-service-client",
                "c",
                "native_service_client_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A4 (actions, C) — the Fibonacci action SERVER single-node entry (cached).
/// Cross-process round-trip (issue 0096); consumed by tests/roundtrip_xprocess_e2e.rs (native_c_action cell).
pub fn build_native_workspace_c_action_server_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_ACTION_SERVER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-c-native-action-server",
                "c",
                "native_action_server_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A4 (actions, C) — the Fibonacci action CLIENT single-node entry (cached). Sends a
/// goal each tick + prints the server-computed result sequence's last element it receives.
pub fn build_native_workspace_c_action_client_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_ACTION_CLIENT_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-c-native-action-client",
                "c",
                "native_action_client_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B6 (custom-msg, C) — the workspace-local `custom_msgs/Reading` TALKER single-node
/// entry (cached). Cross-process (issue 0096); consumed by tests/workspace_features_e2e.rs.
pub fn build_native_workspace_c_custom_msg_talker_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_CUSTOM_MSG_TALKER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-c-custom-msg-talker",
                "features",
                "native_c_custom_msg_talker_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B6 (custom-msg, C) — the workspace-local `custom_msgs/Reading` LISTENER single-node
/// entry (cached). Subscribes + prints the decoded `sequence`/`temperature` fields it receives.
pub fn build_native_workspace_c_custom_msg_listener_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_CUSTOM_MSG_LISTENER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-c-custom-msg-listener",
                "features",
                "native_c_custom_msg_listener_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B4 (QoS, C) — the per-entity QoS-override TALKER single-node entry (cached).
/// Publishes `std_msgs/Int32` on /chatter with a NON-DEFAULT QoS profile (reliable +
/// transient-local + keep-last-10) set in code. Cross-process (issue 0096); consumed by
/// tests/workspace_features_e2e.rs.
pub fn build_native_workspace_c_qos_talker_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_QOS_TALKER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-c-qos-talker",
                "features",
                "native_c_qos_talker_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B4 (QoS, C) — the per-entity QoS-override LISTENER single-node entry (cached).
/// Subscribes /chatter with the SAME non-default QoS profile as the talker + prints `Received: N`.
pub fn build_native_workspace_c_qos_listener_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_QOS_LISTENER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-c-qos-listener",
                "features",
                "native_c_qos_listener_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B4 (QoS, C++) — the per-entity QoS-override TALKER single-node entry (cached).
/// A C++ talker publishes `std_msgs/Int32` on /chatter with a NON-DEFAULT QoS profile (reliable +
/// transient-local + keep-last-10) built via the `nros::QoS` builder. Cross-process (issue 0096);
/// consumed by tests/workspace_features_e2e.rs.
pub fn build_native_workspace_cpp_qos_talker_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_QOS_TALKER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-cpp-qos-talker",
                "features",
                "native_cpp_qos_talker_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B4 (QoS, C++) — the per-entity QoS-override LISTENER single-node entry (cached).
/// Subscribes /chatter with the SAME non-default QoS profile as the talker + prints `Received: N`.
pub fn build_native_workspace_cpp_qos_listener_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_QOS_LISTENER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-cpp-qos-listener",
                "features",
                "native_cpp_qos_listener_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B4 (QoS, MIXED) — the QoS-override TALKER single-node entry (cached). The C
/// `qos_talker_pkg` (non-default QoS in code) reused verbatim, driven by a C++ TYPED entry carrier.
/// Cross-process (issue 0096); consumed by tests/workspace_features_e2e.rs.
pub fn build_native_workspace_mixed_qos_talker_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_MIXED_QOS_TALKER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-mixed-qos-talker",
                "features",
                "native_mixed_qos_talker_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B4 (QoS, MIXED) — the QoS-override LISTENER single-node entry (cached). The C
/// `qos_listener_pkg` reused verbatim, driven by a C++ TYPED entry carrier + prints `Received: N`.
pub fn build_native_workspace_mixed_qos_listener_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_MIXED_QOS_LISTENER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-mixed-qos-listener",
                "features",
                "native_mixed_qos_listener_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B6 (custom-msg, C++) — the workspace-local `custom_msgs/Reading` TALKER single-node
/// entry (cached). A C++ talker hand-encodes the Reading CDR (raw-CDR idiom, no generated link).
/// Cross-process (issue 0096); consumed by tests/workspace_features_e2e.rs.
pub fn build_native_workspace_cpp_custom_msg_talker_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_CUSTOM_MSG_TALKER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-cpp-custom-msg-talker",
                "features",
                "native_cpp_custom_msg_talker_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B6 (custom-msg, C++) — the workspace-local `custom_msgs/Reading` LISTENER single-node
/// entry (cached). Subscribes + prints the decoded `sequence`/`temperature` fields it receives.
pub fn build_native_workspace_cpp_custom_msg_listener_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_CUSTOM_MSG_LISTENER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-cpp-custom-msg-listener",
                "features",
                "native_cpp_custom_msg_listener_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B6 (custom-msg, MIXED) — the workspace-local `custom_msgs/Reading` TALKER single-node
/// entry (cached). The C `reading_talker_pkg` reused verbatim, driven by a C++ TYPED entry carrier.
/// Cross-process (issue 0096); consumed by tests/workspace_features_e2e.rs.
pub fn build_native_workspace_mixed_custom_msg_talker_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_MIXED_CUSTOM_MSG_TALKER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-mixed-custom-msg-talker",
                "features",
                "native_mixed_custom_msg_talker_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 B6 (custom-msg, MIXED) — the workspace-local `custom_msgs/Reading` LISTENER single-node
/// entry (cached). The C `reading_listener_pkg` reused verbatim, driven by a C++ TYPED entry carrier.
pub fn build_native_workspace_mixed_custom_msg_listener_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_MIXED_CUSTOM_MSG_LISTENER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-mixed-custom-msg-listener",
                "features",
                "native_mixed_custom_msg_listener_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A1 (services, C++) — the AddTwoInts service SERVER single-node entry (cached).
pub fn build_native_workspace_cpp_service_server_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_SERVICE_SERVER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-cpp-native-service-server",
                "cpp",
                "native_service_server_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A1 (services, C++) — the AddTwoInts service CLIENT single-node entry (cached).
pub fn build_native_workspace_cpp_service_client_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_SERVICE_CLIENT_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-cpp-native-service-client",
                "cpp",
                "native_service_client_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A4 (actions, C++) — the Fibonacci action SERVER single-node entry (cached).
/// Cross-process round-trip (issue 0096); consumed by tests/roundtrip_xprocess_e2e.rs (native_cpp_action cell).
pub fn build_native_workspace_cpp_action_server_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_ACTION_SERVER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-cpp-native-action-server",
                "cpp",
                "native_action_server_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A4 (actions, C++) — the Fibonacci action CLIENT single-node entry (cached). Sends a
/// goal each tick + prints the server-computed result sequence's last element it receives.
pub fn build_native_workspace_cpp_action_client_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_ACTION_CLIENT_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-cpp-native-action-client",
                "cpp",
                "native_action_client_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A1 (services, MIXED) — the AddTwoInts service SERVER single-node entry (cached).
/// The mixed workspace runs a C server + a C client (the cross-language cpp-client variant is
/// blocked on the action_msgs cpp-codegen gap — see the phase doc).
pub fn build_native_workspace_mixed_service_server_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_MIXED_SERVICE_SERVER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-mixed-native-service-server",
                "mixed",
                "native_service_server_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A1 (services, MIXED) — the AddTwoInts service CLIENT single-node entry (cached).
pub fn build_native_workspace_mixed_service_client_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_MIXED_SERVICE_CLIENT_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-mixed-native-service-client",
                "mixed",
                "native_service_client_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A4 (actions, MIXED) — the Fibonacci action SERVER single-node entry (cached).
/// The mixed workspace runs a C server + a C client (the cross-language cpp variant is blocked
/// on the action_msgs cpp-codegen gap — see the phase doc).
pub fn build_native_workspace_mixed_action_server_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_MIXED_ACTION_SERVER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-mixed-native-action-server",
                "mixed",
                "native_action_server_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 A4 (actions, MIXED) — the Fibonacci action CLIENT single-node entry (cached).
pub fn build_native_workspace_mixed_action_client_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_MIXED_ACTION_CLIENT_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-mixed-native-action-client",
                "mixed",
                "native_action_client_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 Track C — the robot2 (listener) per-host C multihost entry (cached).
pub fn build_native_workspace_c_entry_robot2() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_ENTRY_ROBOT2_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-c-native-robot2",
                "c",
                // phase-383 W10.a — see robot1 above.
                "native_robot2_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 C2a — the threadx-linux C workspace EMBEDDED entry (cached). Built
/// into its own `build-workspace-fixtures-threadx` dir (one board per configure)
/// with a compile-time-baked `tcp/127.0.0.1:<port>` locator so the host-sim
/// connects over the nsos POSIX-`connect()` shim with NO veth bridge / root.
pub fn build_threadx_linux_workspace_c_entry() -> TestResult<&'static Path> {
    THREADX_LINUX_WORKSPACE_C_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-c-threadx-linux",
                "c",
                "build-workspace-fixtures-threadx",
                // `threadx_entry`, not `native_threadx_entry` — the manifest row
                // and the workspace agree on the former (issue 0411). phase-331
                // W2b unified entry names to the platform vocabulary and this
                // resolver kept the old spelling, so the cell reported
                // "[SKIPPED] not built" for a fixture that had been built all
                // along: a missing binary and an absent toolchain look identical
                // from here.
                "threadx_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-370 W3 — the FreeRTOS POSIX simulator C workspace embedded entry (cached).
///
/// Both languages share one `build-workspace-fixtures-freertos-posix` dir per
/// workspace, matching the manifest rows. No baked locator and no slirp net:
/// the RMW is host CycloneDDS over the host's own stack, so the entry needs
/// nothing the `linux` rows do not.
pub fn build_freertos_posix_workspace_c_entry() -> TestResult<&'static Path> {
    FREERTOS_POSIX_WORKSPACE_C_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-c-freertos-posix",
                "c",
                "build-workspace-fixtures-freertos-posix",
                "freertos_posix_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-370 W3 — the C++ projection of the entry above.
pub fn build_freertos_posix_workspace_cpp_entry() -> TestResult<&'static Path> {
    FREERTOS_POSIX_WORKSPACE_CPP_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-cpp-freertos-posix",
                "cpp",
                "build-workspace-fixtures-freertos-posix",
                "freertos_posix_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 C2b — the FreeRTOS (QEMU MPS2-AN385) C workspace embedded entry (cached).
/// Built into its own `build-workspace-fixtures-freertos` dir (one board per configure,
/// cross-compiled thumbv7m via the workspace-root toolchain map) with a baked
/// `tcp/192.0.3.1:<port>` locator the QEMU slirp guest dials.
pub fn build_freertos_workspace_c_entry() -> TestResult<&'static Path> {
    FREERTOS_WORKSPACE_C_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-c-freertos",
                "c",
                "build-workspace-fixtures-freertos",
                "freertos_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 C2b — the NuttX (QEMU arm-virt) C workspace embedded entry (cached). The
/// multi-node talker + listener are linked INTO the NuttX kernel by the cargo `nros-nuttx-ffi`
/// build (the bootable `armv7a-nuttx-eabihf` ELF); the connect locator (`tcp/10.0.2.2:<port>`,
/// dialed through the QEMU slirp gateway) is baked via the NanoRosEntry COMPILE_DEFINITIONS
/// ferried into the cc-rs entry-TU compile. Built into its own `build-workspace-fixtures-nuttx`
/// dir via the `<entry>_build` cargo target (the host `add_executable` is EXCLUDE_FROM_ALL).
pub fn build_nuttx_workspace_c_entry() -> TestResult<&'static Path> {
    NUTTX_WORKSPACE_C_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-c-nuttx",
                "c",
                "build-workspace-fixtures-nuttx",
                "nuttx_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-281 W3-nuttx — the 2-tier **C++** realtime NuttX (QEMU arm-virt) entry
/// (`realtime-cpp`): ctrl (high tier, 10 ms) + telem (low tier, 100 ms) C++
/// nodes over ONE shared session via `NuttxBoard::run_tiers` (RFC-0015 Model 1,
/// one pthread per tier). The FIRST full nuttx link + runtime proof of the
/// W3(nuttx) `nros_board_nuttx_run_tiers` seam. Kernel-linked via cargo
/// `nros-nuttx-ffi` into the bootable `armv7a-nuttx-eabihf` ELF; the connect
/// locator (`tcp/10.0.2.2:<port>`, dialed through the QEMU slirp gateway) is baked
/// via the NanoRosEntry COMPILE_DEFINITIONS ferried into the cc-rs entry-TU
/// compile. Built into its own `build-workspace-fixtures-nuttx` dir.
pub fn build_nuttx_workspace_cpp_realtime_entry() -> TestResult<&'static Path> {
    NUTTX_WORKSPACE_CPP_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-cpp-nuttx-realtime",
                "realtime-cpp",
                "build-workspace-fixtures-nuttx",
                "nuttx_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-281 W3-nuttx — the 2-tier **C** realtime NuttX (QEMU arm-virt) entry
/// (`realtime-c`): ctrl (high tier, 10 ms) + telem (low tier, 100 ms) C nodes
/// over ONE shared session via `NuttxBoard::run_tiers` (RFC-0015 Model 1, one
/// pthread per tier). The C sibling of the C++ `build_nuttx_workspace_cpp_realtime_entry`
/// and of the pure-C `build_nuttx_workspace_c_entry`. Kernel-linked via cargo
/// `nros-nuttx-ffi` into the bootable `armv7a-nuttx-eabihf` ELF; the connect
/// locator (`tcp/10.0.2.2:<port>`, dialed through the QEMU slirp gateway) is baked
/// via the NanoRosEntry COMPILE_DEFINITIONS ferried into the cc-rs entry-TU
/// compile. Built into its own `build-workspace-fixtures-nuttx` dir.
pub fn build_nuttx_workspace_c_realtime_entry() -> TestResult<&'static Path> {
    NUTTX_WORKSPACE_C_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-c-nuttx-realtime",
                "realtime-c",
                "build-workspace-fixtures-nuttx",
                "nuttx_entry",
            )
        })
        .map(|p| p.as_path())
}

/// #199 follow-up — the 2-tier **C++** realtime NuttX rv-virt (riscv32) entry
/// (`realtime-cpp/src/riscv_nuttx_entry`), the riscv sibling of the arm
/// `workspace-cpp-nuttx-realtime` entry. Built by `just nuttx
/// build-riscv-c-workspaces`; consumed by `realtime_tiers_cpp_riscv_nuttx_e2e`.
pub fn build_nuttx_riscv_workspace_cpp_realtime_entry() -> TestResult<&'static Path> {
    static NUTTX_RISCV_WORKSPACE_CPP_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
    NUTTX_RISCV_WORKSPACE_CPP_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-cpp-nuttx-riscv-realtime",
                "realtime-cpp",
                "build-workspace-fixtures-nuttx-riscv",
                "riscv_nuttx_entry",
            )
        })
        .map(|p| p.as_path())
}

/// #199 follow-up — the 2-tier **C** realtime NuttX rv-virt (riscv32) entry
/// (`realtime-c/src/riscv_nuttx_entry`), the riscv sibling of
/// [`build_nuttx_workspace_c_realtime_entry`]. Built by `just nuttx
/// build-riscv-c-workspaces`; consumed by `realtime_tiers_c_riscv_nuttx_e2e`.
pub fn build_nuttx_riscv_workspace_c_realtime_entry() -> TestResult<&'static Path> {
    static NUTTX_RISCV_WORKSPACE_C_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
    NUTTX_RISCV_WORKSPACE_C_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-c-nuttx-riscv-realtime",
                "realtime-c",
                "build-workspace-fixtures-nuttx-riscv",
                "riscv_nuttx_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-281 W3-nuttx — the 2-tier **Rust** realtime NuttX (QEMU arm-virt) entry
/// (`realtime-rust/src/nuttx_entry`): ctrl (high tier, 10 ms) + telem (low
/// tier, 100 ms) Rust nodes over ONE shared session via `<QemuArmVirt>::run_tiers`
/// (RFC-0015 Model 1, one `std::thread` per tier — NuttX is `std` + zenoh-pico
/// `Z_FEATURE_MULTI_THREAD = 1`). Closes the LAST cell (rust×nuttx) of the
/// convergence matrix. Unlike the C/C++ nuttx entries (kernel-linked via CMake),
/// this is a pure-cargo cross build (`--target armv7a-nuttx-eabihf`, build-std),
/// so it resolves the prebuilt binary directly from the row's `target_dir`
/// (`target-fixtures/nuttx/armv7a-nuttx-eabihf/<carve-out profile>/nuttx_entry`) at the
/// `release` profile — the 177.8.c CGU-miscompile dodge the NuttX cargo lane forces.
/// Built by `just nuttx build-examples` (→ `workspace-fixtures-build.sh nuttx rust`).
/// phase-285 W6 (issue #165) — the rv-virt (riscv32) sibling of the arm entry
/// above: `realtime-rust/src/riscv_nuttx_entry`, built by the
/// `workspace-rust-nuttx-riscv-realtime` fixture row (`just nuttx
/// build-riscv-rust`).
pub fn build_nuttx_riscv_workspace_rust_realtime_entry() -> TestResult<&'static Path> {
    NUTTX_RISCV_WORKSPACE_RUST_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            let fixture_id = "workspace-rust-nuttx-riscv-realtime";
            workspace_example_dir("realtime-rust")?;
            let target_dir = crate::fixtures::groups::workspace_artifact_dir(fixture_id)?;
            // phase-336 — the NuttX carve-out profile (see NUTTX_RUST_PROFILE);
            // `workspace-fixtures-build.sh` builds these rows at it.
            let binary_path = target_dir.join(format!(
                "riscv32imac-unknown-nuttx-elf/{}/riscv_nuttx_entry",
                nros_cargo_profile::target_dir(nros_cargo_profile::NUTTX_RUST_PROFILE)
            ));
            require_prebuilt_workspace_binary(
                fixture_id,
                &binary_path,
                &target_dir.join(workspace_fixture_stamp_name(fixture_id)),
            )
        })
        .map(|p| p.as_path())
}

/// Resolve the prebuilt riscv-nuttx (rv-virt) C talker kernel ELF (cached).
///
/// #199 follow-up — the first riscv-nuttx C-lane RUNTIME fixture: the
/// standalone `examples/qemu-riscv-nuttx/c/talker` cmake example, built by
/// `just nuttx build-riscv-c` with the baked `tcp/10.0.2.2:8700` locator
/// (see its `examples/fixtures.toml` row). Consumed by `c_riscv_nuttx_e2e`.
pub fn build_nuttx_riscv_c_talker() -> TestResult<&'static Path> {
    static NUTTX_RISCV_C_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();
    NUTTX_RISCV_C_TALKER_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw("qemu-riscv-nuttx/c/talker", "c_talker", Rmw::Zenoh)
        })
        .map(|p| p.as_path())
}

pub fn build_nuttx_workspace_rust_realtime_entry() -> TestResult<&'static Path> {
    NUTTX_WORKSPACE_RUST_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            let fixture_id = "workspace-rust-nuttx-realtime";
            workspace_example_dir("realtime-rust")?;
            let target_dir = crate::fixtures::groups::workspace_artifact_dir(fixture_id)?;
            let binary_path = target_dir.join(format!(
                "armv7a-nuttx-eabihf/{}/nuttx_entry",
                nros_cargo_profile::target_dir(nros_cargo_profile::NUTTX_RUST_PROFILE)
            ));
            require_prebuilt_workspace_binary(
                fixture_id,
                &binary_path,
                &target_dir.join(workspace_fixture_stamp_name(fixture_id)),
            )
        })
        .map(|p| p.as_path())
}

/// phase-297 W5 (RFC-0053) — the 2-tier **Rust** realtime ThreadX-Linux entry
/// (`realtime-rust/src/threadx_entry`): ctrl (high tier, 10 ms) + telem
/// (low tier, 100 ms) Rust nodes over ONE shared session via
/// `<ThreadxLinux>::run_tiers` (one ThreadX thread per tier, stacks from the
/// shared byte pool — `nros_threadx_create_task`). Hosted simulation: a plain
/// host cargo cross-dir build (ThreadX threads are
/// pthreads, NSOS host sockets — no QEMU), so it resolves the prebuilt binary
/// from the row's `target_dir` at the default fixture profile — FLAT, with no
/// triple segment: the row used to carry `target = "x86_64-unknown-linux-gnu"`,
/// which both spliced a triple in here and made the build fail on every non-x86
/// host. Omitting the row's `target` is what "host build" actually means. Built by
/// `workspace-fixtures-build.sh threadx-linux rust`
/// (`just threadx_linux build-fixtures`).
pub fn build_threadx_workspace_rust_realtime_entry() -> TestResult<&'static Path> {
    static THREADX_WORKSPACE_RUST_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
    THREADX_WORKSPACE_RUST_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            let fixture_id = "workspace-rust-threadx-linux-realtime";
            workspace_example_dir("realtime-rust")?;
            let target_dir = crate::fixtures::groups::workspace_artifact_dir(fixture_id)?;
            let binary_path =
                target_dir.join(format!("{}/threadx_entry", cargo_target_profile_dir()));
            require_prebuilt_workspace_binary(
                fixture_id,
                &binary_path,
                &target_dir.join(workspace_fixture_stamp_name(fixture_id)),
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 C2c — the threadx-linux C++ workspace embedded entry (cached).
pub fn build_threadx_linux_workspace_cpp_entry() -> TestResult<&'static Path> {
    THREADX_LINUX_WORKSPACE_CPP_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-cpp-threadx-linux",
                "cpp",
                "build-workspace-fixtures-threadx",
                "threadx_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 C2c — the FreeRTOS (QEMU MPS2-AN385) C++ workspace embedded entry (cached).
pub fn build_freertos_workspace_cpp_entry() -> TestResult<&'static Path> {
    FREERTOS_WORKSPACE_CPP_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-cpp-freertos",
                "cpp",
                "build-workspace-fixtures-freertos",
                "freertos_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-274 W3 (#126) — the 2-tier C++ realtime FreeRTOS/mps2 entry
/// (`realtime-cpp`): ctrl (high tier, 10 ms) + telem (low tier, 100 ms)
/// over one shared session via `FreertosBoard::run_tiers` (RFC-0015 Model 1).
/// issue 0636 gap 2 — the 2-tier **Rust** realtime FreeRTOS/mps2 entry
/// (`realtime-rust`): ctrl (high, 10 ms) + telem (low, 100 ms) over one shared
/// session via `Mps2An385Freertos::run_tiers` → `run_tiers_entry`.
///
/// That Rust path was exported from `nros-board-freertos` and reachable from
/// the `nros::main!` macro with NO consumer anywhere: every FreeRTOS realtime
/// fixture was C or C++, and the one Rust FreeRTOS entry (`workspaces/rust`) is
/// single-tier `run_entry`. So it is the arm #0636's boot-tier fix had to be
/// reasoned onto rather than measured — which is the gap this closes.
///
/// A CARGO row, unlike its C/C++ siblings' cmake ones: the entry is a
/// `#![no_std] #![no_main]` bin cross-built for `thumbv7m-none-eabi`.
pub fn build_freertos_workspace_rust_realtime_entry() -> TestResult<&'static Path> {
    static FREERTOS_WORKSPACE_RUST_REALTIME_ENTRY_BINARY: OnceCell<PathBuf> = OnceCell::new();
    FREERTOS_WORKSPACE_RUST_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            let fixture_id = "workspace-rust-freertos-realtime";
            workspace_example_dir("realtime-rust")?;
            let target_dir = crate::fixtures::groups::workspace_artifact_dir(fixture_id)?;
            let binary_path = target_dir.join(format!(
                "thumbv7m-none-eabi/{}/freertos_realtime_entry",
                cargo_target_profile_dir()
            ));
            require_prebuilt_workspace_binary(
                fixture_id,
                &binary_path,
                &target_dir.join(workspace_fixture_stamp_name(fixture_id)),
            )
        })
        .map(|p| p.as_path())
}

pub fn build_freertos_workspace_cpp_realtime_entry() -> TestResult<&'static Path> {
    FREERTOS_WORKSPACE_CPP_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-cpp-freertos-realtime",
                "realtime-cpp",
                "build-workspace-fixtures-freertos",
                "freertos_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-281 W2 — the 2-tier **C** realtime FreeRTOS/mps2 entry
/// (`realtime-c`): ctrl (high tier, 10 ms) + telem (low tier, 100 ms)
/// over one shared session via `FreertosBoard::run_tiers` → the shared C
/// `nros_board_freertos_run_tiers` glue (RFC-0015 Model 1). Proves that shared
/// C run_tiers impl drives a C *node*, not only a C++ one.
pub fn build_freertos_workspace_c_realtime_entry() -> TestResult<&'static Path> {
    FREERTOS_WORKSPACE_C_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-c-freertos-realtime",
                "realtime-c",
                "build-workspace-fixtures-freertos",
                "freertos_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 C2c — the MIXED (C + C++ + Rust) threadx-linux embedded entry (cached). The
/// Rust heartbeat node links via the `nros_ws_runtime` umbrella (host x86_64 triple).
pub fn build_threadx_linux_workspace_mixed_entry() -> TestResult<&'static Path> {
    THREADX_LINUX_WORKSPACE_MIXED_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-mixed-threadx-linux",
                "mixed",
                "build-workspace-fixtures-threadx",
                "threadx_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 C2c — the MIXED (C + C++ + no_std Rust) FreeRTOS QEMU embedded entry (cached).
/// The Rust heartbeat node compiles no_std (thumbv7m); the nros_ws_runtime umbrella
/// cross-compiles + re-points NanoRosCpp for the board.
pub fn build_freertos_workspace_mixed_entry() -> TestResult<&'static Path> {
    FREERTOS_WORKSPACE_MIXED_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-mixed-freertos",
                "mixed",
                "build-workspace-fixtures-freertos",
                "freertos_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Native C++ workspace Entry pkg fixture.
pub fn build_native_workspace_cpp_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry("workspace-cpp-native", "cpp", "native_entry")
        })
        .map(|p| p.as_path())
}

/// Phase 269 W1 — the parameterised C++ workspace Entry pkg fixture (`features`).
/// Reads live `publish_period_ms` via `nros_cpp_get_param_integer` (executor handle
/// saved from `node.executor_handle()` at configure time); consumed by
/// tests/cpp_c_param_live_read_e2e.rs.
pub fn build_native_workspace_cpp_params_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_PARAMS_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-cpp-params",
                "features",
                "native_cpp_params_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Phase 269 W2 — the managed-node (lifecycle) C++ workspace Entry pkg fixture (`managed`).
/// Boots the talker node to `active` via `nros_cpp_lifecycle_autostart`; consumed by
/// tests/workspace_features_e2e.rs.
pub fn build_native_workspace_cpp_lifecycle_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_LIFECYCLE_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-features-cpp-lifecycle",
                "features",
                "native_cpp_lifecycle_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Phase 270 (#103) — the wrapper-managed C++ lifecycle entry (`managed`,
/// `native_managed_entry`). Boots `ManagedTalker`, which drives REP-2002 itself via
/// `nros::LifecycleNode` (register_services + autostart(Active)); consumed by
/// tests/cpp_lifecycle_node_wrapper_e2e.rs.
pub fn build_native_workspace_cpp_lifecycle_managed_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_LIFECYCLE_MANAGED_ENTRY_BINARY
        .get_or_try_init(|| {
            // Issue 0257 — the managed bringup bakes a larger executor callback
            // arena (NROS_EXECUTOR_MAX_CBS=8), which probes a DIFFERENT executor
            // size than the sibling `workspace-cpp-native-lifecycle` row. They
            // must NOT share `build-workspace-fixtures` (the per-build-tree
            // `nros_config_generated.h` would disagree and trip the sizes-mirror
            // guard), so `fixtures.toml` gives this row its own
            // `build-workspace-fixtures-managed` — resolve the entry from there.
            build_workspace_cmake_entry_in(
                "workspace-managed-cpp-native",
                "managed",
                "build-workspace-fixtures",
                "native_managed_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Phase 269 W3 — the talker half of the cross-process E2E-safety C++ demo (`safety`).
/// Publishes CRC-annotated /chatter frames when built with NANO_ROS_SAFETY_E2E=ON.
/// Consumed by tests/workspace_features_e2e.rs.
pub fn build_native_workspace_cpp_safety_talker_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_SAFETY_TALKER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-safety-cpp-talker",
                "safety",
                "build/posix-zenoh-native/cmake",
                "native_cpp_safety_talker_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Phase 269 W3 — the listener half of the cross-process E2E-safety C++ demo (`safety`).
/// Uses `node.create_subscription_with_safety<M>()` typed API + IntegrityStatus callback.
/// Consumed by tests/workspace_features_e2e.rs.
pub fn build_native_workspace_cpp_safety_listener_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_SAFETY_LISTENER_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry_in(
                "workspace-safety-cpp-listener",
                "safety",
                "build/posix-zenoh-native/cmake",
                "native_cpp_safety_listener_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Phase 269 W4 — the 2-tier sched-context C realtime workspace entry (cached).
/// Schedules ctrl_node (10 ms, high-priority sched context) and telem_node
/// (100 ms, low-priority sched context) via nros_cpp_create_sched_context +
/// nros_cpp_node_create_ex. Consumed by tests/realtime_tiers_c_e2e.rs.
pub fn build_native_workspace_c_realtime_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_C_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry("workspace-c-native-realtime", "realtime-c", "native_entry")
        })
        .map(|p| p.as_path())
}

/// Phase 269 W4 — the 2-tier sched-context C++ realtime workspace entry (cached).
/// Schedules ctrl_node (10 ms, high-priority sched context) and telem_node
/// (100 ms, low-priority sched context) via nros_cpp_create_sched_context +
/// NodeBuilder::sched(). Consumed by tests/realtime_tiers_cpp_e2e.rs.
pub fn build_native_workspace_cpp_realtime_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-cpp-native-realtime",
                "realtime-cpp",
                "native_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Phase 272 W3 (RFC-0047, issue #124) — the rclcpp-shape 2-tier realtime workspace
/// entry (cached). Same tier config as realtime-cpp (ctrl_node 10 ms high,
/// telem_node 100 ms low) but components are IS-A-node `nros::ComponentNode`
/// subclasses (SHAPE rclcpp). Tier binding via the W2-seeded `node_name →
/// sched_context` table — the runtime proof that #124 is dissolved.
/// Consumed by tests/realtime_tiers_cpp_rclcpp_e2e.rs.
pub fn build_native_workspace_cpp_rclcpp_realtime_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_RCLCPP_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-cpp-native-realtime-rclcpp",
                "realtime-cpp",
                // The manifest row declares `native_rclcpp_entry` and the
                // manifest is the SSoT (issue 0411). This named the sibling
                // `native_entry` — a real package, but the CONFIGURE-shape one,
                // not the rclcpp shape this fixture exists to cover. The
                // mismatch makes the fixture read "not built" forever: the
                // builder produces what the manifest says and the resolver
                // looks for something else.
                "native_rclcpp_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Phase 273 W4 (RFC-0047) — the sub-node 2-group realtime workspace entry (cached).
/// ONE `subnode_pkg::SubNode` (IS-A ComponentNode) declares two callback groups
/// in code ("ctrl" 10 ms, "telem" 100 ms); `system.toml group_tiers` maps them to
/// the "high" and "low" tiers. The entry emits `bind_group_sched` for BOTH groups of
/// the SAME node before construction — proving per-group binding (the capability the
/// per-node-name table cannot express). Consumed by tests/realtime_subnode_cpp_e2e.rs.
pub fn build_native_workspace_cpp_subnode_realtime_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_SUBNODE_REALTIME_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-cpp-native-realtime-subnode",
                "realtime-cpp",
                // The manifest row declares `entry = "native_subnode_entry"`,
                // and the workspace builds three entries side by side
                // (`native_entry`, `native_rclcpp_entry`, `native_subnode_entry`)
                // — so naming the wrong one here resolves to a real binary's
                // NEIGHBOUR and the cell skips forever. issue 0411's class,
                // caught by the guard this call sits under.
                "native_subnode_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Phase 273 W4 (RFC-0047) — the sub-node portability workspace entry (cached).
/// The `subnode_pkg::SubNode` component is IDENTICAL to the sub-node fixture
/// but this workspace uses "fast"/"bulk" tier names instead of "high"/"low" — no
/// package change. Proves RFC-0047 portability: a group-using package can be
/// redeployed with different tier names by changing only `system.toml`.
/// Consumed by tests/realtime_subnode_cpp_portable_e2e.rs.
pub fn build_native_workspace_cpp_subnode_portable_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_SUBNODE_PORTABLE_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-cpp-native-realtime-subnode-portable",
                "realtime-cpp",
                "native_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 Track C — the robot1 (talker) per-host C++ multihost entry (cached).
pub fn build_native_workspace_cpp_entry_robot1() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_ENTRY_ROBOT1_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry("workspace-cpp-native-robot1", "cpp", "native_robot1_entry")
        })
        .map(|p| p.as_path())
}

/// phase-263 Track C — the robot2 (listener) per-host C++ multihost entry (cached).
pub fn build_native_workspace_cpp_entry_robot2() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_CPP_ENTRY_ROBOT2_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry("workspace-cpp-native-robot2", "cpp", "native_robot2_entry")
        })
        .map(|p| p.as_path())
}

/// Native mixed C/C++ workspace Entry pkg fixture.
pub fn build_native_workspace_mixed_entry() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_MIXED_ENTRY_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry("workspace-mixed-native", "mixed", "native_entry")
        })
        .map(|p| p.as_path())
}

/// phase-263 Track C — the robot1 (C talker + Rust heartbeat) per-host mixed entry.
pub fn build_native_workspace_mixed_entry_robot1() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_MIXED_ENTRY_ROBOT1_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-mixed-native-robot1",
                "mixed",
                "native_robot1_entry",
            )
        })
        .map(|p| p.as_path())
}

/// phase-263 Track C — the robot2 (C++ listener) per-host mixed entry.
pub fn build_native_workspace_mixed_entry_robot2() -> TestResult<&'static Path> {
    NATIVE_WORKSPACE_MIXED_ENTRY_ROBOT2_BINARY
        .get_or_try_init(|| {
            build_workspace_cmake_entry(
                "workspace-mixed-native-robot2",
                "mixed",
                "native_robot2_entry",
            )
        })
        .map(|p| p.as_path())
}

/// Phase 118 — collapsed-shape native C talker, RMW-parametrized.
///
/// Returns the prebuilt binary for the named RMW. The fixture build
/// chain (`just native build-fixtures`) configures + builds
/// `examples/native/c/talker/` once per RMW into separate
/// `build-{zenoh,dds,xrce}/` dirs.
pub fn build_native_c_talker_rmw(rmw: Rmw) -> TestResult<&'static Path> {
    static ZENOH_CELL: OnceCell<PathBuf> = OnceCell::new();
    static XRCE_CELL: OnceCell<PathBuf> = OnceCell::new();
    static CYCLONEDDS_CELL: OnceCell<PathBuf> = OnceCell::new();
    let cell = match rmw {
        Rmw::Zenoh => &ZENOH_CELL,
        Rmw::Xrce => &XRCE_CELL,
        Rmw::Cyclonedds => &CYCLONEDDS_CELL,
    };
    cell.get_or_try_init(|| build_example_cmake_rmw("native/c/talker", "c_talker", rmw))
        .map(|p| p.as_path())
}

/// Phase 131.B — resolve a prebuilt test-fixture / bench binary that lives
/// under `packages/testing/nros-{tests/bins,bench,smoke}/<crate>/`.
///
/// `crate_subpath` is the path *under* `packages/testing/` (e.g.
/// `"nros-tests/bins/cdr-roundtrip-qemu"`).
pub fn build_test_fixture(
    crate_subpath: &str,
    binary_name: &str,
    target: Option<&str>,
) -> TestResult<PathBuf> {
    // #156 — NuttX Rust fixtures are ALWAYS built at the carve-out profile (see
    // `nros_cargo_profile::NUTTX_RUST_PROFILE` for the miscompile it dodges), so
    // this resolver must look where the BUILDER wrote rather than where the
    // ambient profile would put it. Before the constant existed, the two
    // disagreed and a fresh, working image read as stale/absent.
    let profile = match target {
        Some("armv7a-nuttx-eabihf") => nros_cargo_profile::NUTTX_RUST_PROFILE,
        _ => return build_test_fixture_at_profile(crate_subpath, binary_name, target, None),
    };
    build_test_fixture_at_profile(crate_subpath, binary_name, target, Some(profile))
}

/// [`build_test_fixture`] with the builder's profile named EXPLICITLY.
///
/// phase-340 P2 — the FreeRTOS QEMU carve-out
/// (`nros_cargo_profile::FREERTOS_QEMU_PROFILE`) cannot be inferred from the
/// target triple the way the NuttX one can: `thumbv7m-none-eabi` is shared with
/// the bare-metal MPS2 bins, which build at the ambient profile. Rather than
/// guess from the crate path, the caller that knows the carve-out says so —
/// there is exactly one such caller today
/// ([`build_logging_smoke_freertos_mps2`]), and `just freertos build-fixtures`
/// passes the same constant to the builder via `NROS_CARGO_PROFILE`.
///
/// `profile = None` means the ambient profile.
pub fn build_test_fixture_at_profile(
    crate_subpath: &str,
    binary_name: &str,
    target: Option<&str>,
    profile: Option<&str>,
) -> TestResult<PathBuf> {
    let root = project_root();
    let crate_dir = root.join(format!("packages/testing/{}", crate_subpath));

    if !crate_dir.exists() {
        return Err(TestError::BuildFailed(format!(
            "Test fixture crate directory not found: {}",
            crate_dir.display()
        )));
    }

    let profile_dir = match profile {
        Some(p) => nros_cargo_profile::target_dir(p),
        None => cargo_target_profile_dir(),
    };
    let rel = PathBuf::from(match target {
        Some(target) => format!("{target}/{profile_dir}/{binary_name}"),
        None => format!("{profile_dir}/{binary_name}"),
    });

    // issue 0517 step 3, second funnel. `build_example` grew this guard; this —
    // its `packages/testing/` sibling — did not, and step 3's "0 multi-row
    // leaves left on the path route" was measured over the migrated funnel only.
    // `nros-bench/stress-zenoh` was reached through here and IS a two-row leaf
    // (plain + `ZPICO_SUBSCRIBER_BUFFER_SIZE=8192`), so with the `target_dir`
    // column deleted both rows share `<dir>/target`, `groups::attribute` calls
    // that ambiguous and declines to redirect, and this funnel resolved the
    // un-redirected leaf path — where a pre-group build had left a binary. All
    // five `large_msg` zenoh tests read that museum artifact and reported STALE
    // while the freshly built one sat in `build/cargo-fixtures/<slug>`.
    //
    // MULTI-row, not `leaf_has_rows`: see `groups::leaf_is_multi_row` for why
    // the wider predicate would break the sole-row leaves reached from here.
    let leaf = format!("packages/testing/{crate_subpath}");
    if crate::fixtures::groups::leaf_is_multi_row(&leaf) {
        let row = crate::fixtures::groups::select_row(
            &leaf,
            &crate::fixtures::groups::FixtureVariant::plain(),
        )?;
        return require_prebuilt_row_binary_fresh(row, &rel);
    }
    let binary_path = crate_dir.join("target").join(&rel);

    // phase-278 W2 — `bins/` fixtures are cargo binaries and carry a `.d`
    // dep-info file, so the same rust staleness probe applies.
    require_prebuilt_binary_fresh(&binary_path)
}

/// Phase 226.D — resolve a prebuilt standalone Rust fixture that builds into
/// the shared fixture target dir, for a caller that knows a PLATFORM and a
/// binary name but no manifest row.
///
/// `platform` selects the group, `triple` is the cross target the leaf's
/// `.cargo/config.toml [build] target` puts in the path, `binary_name` is the
/// Cargo `[[bin]]` name; the binary lands at
/// `build/cargo-fixtures/<group>/<triple>/<profile>/<binary_name>`.
///
/// phase-340 B2 — the group now comes from
/// [`crate::fixtures::groups::sole_group_dir`] instead of a hardcoded
/// `cargo-fixtures/<platform>`, which is what made this the DEFAULT-group-only
/// resolver `check-fixture-groups`'s A2 arm was blocking migrations on. Two
/// consequences, both deliberate:
///
/// * the Rust-side mirror of `NROS_FIXTURE_SHARED_PLATFORMS` is GONE.
///   Eligibility is decided once, by the shell, and arrives per row in the
///   export — a mirror of an eligibility list is the #393 class, and this one
///   had already been reduced from a hardcoded `match` to a duplicated env read
///   without removing the duplication.
/// * a platform with MORE than one group is an error here rather than a silent
///   answer for the default group. Such a platform's resolvers must route
///   through a leaf artifact path (the two `build_example*` funnels and their
///   ~30 inline siblings all do), because only the path names the row and hence
///   the variant. `linux` will have seven groups.
fn require_shared_fixture_binary(
    platform: &str,
    triple: &str,
    binary_name: &str,
) -> TestResult<PathBuf> {
    let target_dir = crate::fixtures::groups::sole_group_dir(platform)?;
    let binary_path = target_dir.join(format!(
        "{triple}/{}/{}",
        cargo_target_profile_dir(),
        binary_name
    ));
    require_prebuilt_binary_fresh(&binary_path)
}

/// Phase 226.D — qemu-arm-baremetal (`thumbv7m-none-eabi`) shared-fixture
/// binary resolver.
fn require_qemu_baremetal_fixture(binary_name: &str) -> TestResult<PathBuf> {
    require_shared_fixture_binary("qemu-arm-baremetal", "thumbv7m-none-eabi", binary_name)
}

/// Build native-rs-talker (cached). phase-277 W3.a: the default-target talker
/// fixture is now the PLAIN talker build (the parameterised variant moved to
/// the `param-chatter-talker` bin — see [`build_native_param_talker`]).
pub fn build_native_talker() -> TestResult<&'static Path> {
    NATIVE_TALKER_BINARY
        .get_or_try_init(|| build_example("native/rust/talker", "talker", None, None))
        .map(|p| p.as_path())
}

/// Resolve the prebuilt `param-chatter-talker` fixture (cached).
///
/// phase-277 W3.a: was `examples/native/rust/talker --features param-services`
/// (the default-target talker build); now a dedicated bin
/// (`packages/testing/nros-tests/bins/param-chatter-talker`) with the
/// parameter services baked. Same behavior: registers the REP-2002 parameter
/// services, declares `start_value`, logs `Counter start value: N`, publishes
/// Int32 on /chatter every 1 s. Consumed by tests/params.rs.
pub fn build_native_param_talker() -> TestResult<&'static Path> {
    static NATIVE_PARAM_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();
    NATIVE_PARAM_TALKER_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_sole_row(
                "packages/testing/nros-tests/bins/param-chatter-talker",
            )?;
            let profile = cargo_target_profile_dir();
            let rel = PathBuf::from(format!("{profile}/param-chatter-talker"));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// Resolve the prebuilt `int32-sink` fixture (cached).
///
/// phase-277 W4: was `examples/native/rust/listener` with its `NROS_SUB_TOPIC`
/// escape hatch; the example flipped to the official `std_msgs/String` chatter
/// (`I heard: [Hello World: N]`), so the generic Int32 side-topic observer
/// moved to a dedicated bin (`packages/testing/nros-tests/bins/int32-sink`).
/// Subscribes to `NROS_SUB_TOPIC` (default `/chatter`) typed `std_msgs/Int32`
/// and prints `Received: N`
/// ([`crate::output::INT32_LISTENER_LOG_PREFIX`]) per message.
/// phase-338 W3 — resolved PER RMW. The bin was hardcoded to zenoh, which is
/// why the zenoh->xrce declarative bridge e2e still drove the EXAMPLE with a
/// test-only `NROS_SUB_TYPE=int32` switch; it now carries the same
/// `rmw-{zenoh,xrce}` axis the examples do.
///
/// issue 0449 — cyclonedds joined them, which is what let
/// `examples/native/c/listener` drop its `NROS_SUB_TYPE` type switch: the
/// zenoh->cyclonedds bridge e2e was the last caller that needed an Int32
/// listener speaking a transport this bin could not.
/// phase-381 — resolve the prebuilt `graph-probe` fixture (cached).
///
/// Prints the ROS graph as the node sees it, so `graph_interop.rs` can compare
/// that against `ros2 node list`. Zenoh only for now: it is the backend whose
/// twelve graph slots are filled, and the one issue 0903 was measured against.
pub fn build_graph_probe() -> TestResult<&'static Path> {
    build_graph_probe_rmw(Rmw::Zenoh)
}

/// The graph probe built against `rmw` — phase-381 step 2.
///
/// Per-RMW because Cyclone's `ros_discovery_info` READER had never been run
/// against a live peer, and zenoh's path is the reason that matters: every unit
/// test passed while the feature did not work at all (issue 0903). One cell per
/// backend, one binary per cell.
pub fn build_graph_probe_rmw(rmw: Rmw) -> TestResult<&'static Path> {
    static ZENOH_BIN: OnceCell<PathBuf> = OnceCell::new();
    static CYCLONE_BIN: OnceCell<PathBuf> = OnceCell::new();
    static XRCE_ABSENT: OnceCell<PathBuf> = OnceCell::new();
    let cell = match rmw {
        Rmw::Zenoh => &ZENOH_BIN,
        Rmw::Cyclonedds => &CYCLONE_BIN,
        // XRCE has no graph, so it has no `[[fixture]]` row and no static
        // slot. `select_row` below reports the missing row precisely; W6 makes
        // "no graph" an `UNSUPPORTED` ANSWER at runtime rather than a fixture.
        Rmw::Xrce => &XRCE_ABSENT,
    };
    cell.get_or_try_init(|| {
        let row = crate::fixtures::groups::select_row(
            "packages/testing/nros-tests/bins/graph-probe",
            &crate::fixtures::groups::FixtureVariant::rmw(rmw),
        )?;
        let profile = cargo_target_profile_dir();
        let rel = PathBuf::from(format!("{profile}/graph-probe"));
        require_prebuilt_row_binary_fresh(row, &rel)
    })
    .map(|p| p.as_path())
}

/// phase-433 W6 — resolve the prebuilt `advertised-state-probe` fixture.
///
/// Holds a Cyclone publisher and subscription open and prints what the
/// matched-count, GID, actual-QoS and serialization-format slots say about
/// them, so `advertised_state_interop.rs` can put the same questions to a live
/// ROS 2 peer.
///
/// No `rmw` parameter, unlike [`build_graph_probe_rmw`]: Cyclone is the only
/// backend that fills any of those slots, so there is one row and one binary.
/// A zenoh build would assert against NULL pointers.
pub fn build_advertised_state_probe() -> TestResult<&'static Path> {
    static BIN: OnceCell<PathBuf> = OnceCell::new();
    BIN.get_or_try_init(|| {
        let row = crate::fixtures::groups::select_row(
            "packages/testing/nros-tests/bins/advertised-state-probe",
            &crate::fixtures::groups::FixtureVariant::rmw(Rmw::Cyclonedds),
        )?;
        let profile = cargo_target_profile_dir();
        let rel = PathBuf::from(format!("{profile}/advertised-state-probe"));
        require_prebuilt_row_binary_fresh(row, &rel)
    })
    .map(|p| p.as_path())
}

pub fn build_int32_sink_rmw(rmw: Rmw) -> TestResult<&'static Path> {
    static ZENOH_BIN: OnceCell<PathBuf> = OnceCell::new();
    static XRCE_BIN: OnceCell<PathBuf> = OnceCell::new();
    static CYCLONE_BIN: OnceCell<PathBuf> = OnceCell::new();
    let cell = match rmw {
        Rmw::Zenoh => &ZENOH_BIN,
        Rmw::Xrce => &XRCE_BIN,
        Rmw::Cyclonedds => &CYCLONE_BIN,
    };
    cell.get_or_try_init(|| {
        let row = crate::fixtures::groups::select_row(
            "packages/testing/nros-tests/bins/int32-sink",
            &crate::fixtures::groups::FixtureVariant::rmw(rmw),
        )?;
        let profile = cargo_target_profile_dir();
        let rel = PathBuf::from(format!("{profile}/int32-sink"));
        require_prebuilt_row_binary_fresh(row, &rel)
    })
    .map(|p| p.as_path())
}

/// phase-425 W5 — the `/clock` publisher, the simulator half of the sim-time
/// pair. Cyclone only: zenoh needs a router that ships with ROS, and this
/// fixture exists to DEMONSTRATE simulated time on a host that may have none.
pub fn build_sim_clock_publisher() -> TestResult<&'static Path> {
    static BIN: OnceCell<PathBuf> = OnceCell::new();
    BIN.get_or_try_init(|| {
        let row = crate::fixtures::groups::select_row(
            "packages/testing/nros-tests/bins/sim-clock-publisher",
            &crate::fixtures::groups::FixtureVariant::rmw(Rmw::Cyclonedds),
        )?;
        let profile = cargo_target_profile_dir();
        let rel = PathBuf::from(format!("{profile}/sim-clock-publisher"));
        require_prebuilt_row_binary_fresh(row, &rel)
    })
    .map(|p| p.as_path())
}

/// phase-425 W5 — the node half: `use_sim_time` true, one ROS-time timer and
/// one wall timer of the same period.
pub fn build_sim_clock_listener() -> TestResult<&'static Path> {
    static BIN: OnceCell<PathBuf> = OnceCell::new();
    BIN.get_or_try_init(|| {
        let row = crate::fixtures::groups::select_row(
            "packages/testing/nros-tests/bins/sim-clock-listener",
            &crate::fixtures::groups::FixtureVariant::rmw(Rmw::Cyclonedds),
        )?;
        let profile = cargo_target_profile_dir();
        let rel = PathBuf::from(format!("{profile}/sim-clock-listener"));
        require_prebuilt_row_binary_fresh(row, &rel)
    })
    .map(|p| p.as_path())
}

/// The zenoh build — the default for every caller that does not care.
pub fn build_int32_sink() -> TestResult<&'static Path> {
    build_int32_sink_rmw(Rmw::Zenoh)
}

/// Resolve the prebuilt concurrent Fibonacci action-server fixture for `rmw`
/// (cached per RMW).
///
/// phase-277 W5: was the example action-server's `NROS_ACTION_CONCURRENT`
/// escape hatch; the example slimmed to the official single-goal
/// `fibonacci_action_server` demo, so the concurrent-goals stress role moved
/// to a dedicated bin
/// (`packages/testing/nros-tests/bins/action-server-concurrent`, built
/// per-RMW into `target-zenoh/` / `target-xrce/`). Accepts + advances several
/// goals at once, draining get_result every spin.
pub fn build_action_server_concurrent(rmw: Rmw) -> TestResult<&'static Path> {
    static ZENOH_BIN: OnceCell<PathBuf> = OnceCell::new();
    static XRCE_BIN: OnceCell<PathBuf> = OnceCell::new();
    let cell = match rmw {
        Rmw::Zenoh => &ZENOH_BIN,
        Rmw::Xrce => &XRCE_BIN,
        other => {
            return Err(TestError::BuildFailed(format!(
                "action-server-concurrent has no {other:?} fixture build"
            )));
        }
    };
    cell.get_or_try_init(|| {
        let row = crate::fixtures::groups::select_row(
            "packages/testing/nros-tests/bins/action-server-concurrent",
            &crate::fixtures::groups::FixtureVariant::rmw(rmw),
        )?;
        let profile = cargo_target_profile_dir();
        let rel = PathBuf::from(format!("{profile}/action-server-concurrent"));
        require_prebuilt_row_binary_fresh(row, &rel)
    })
    .map(|p| p.as_path())
}

/// issue 0322 — the multi-goal client half of the `MAX_GOALS` stress pair.
/// Sends more concurrent goals than the server's `active_goals` table holds
/// and reports each acceptance verdict, so "full table rejects" can be
/// distinguished from "full table acknowledges and drops". Pair with
/// [`build_action_server_concurrent`], which holds goals across spins.
pub fn build_action_client_multigoal(rmw: Rmw) -> TestResult<&'static Path> {
    static ZENOH_BIN: OnceCell<PathBuf> = OnceCell::new();
    let cell = match rmw {
        Rmw::Zenoh => &ZENOH_BIN,
        other => {
            return Err(TestError::BuildFailed(format!(
                "action-client-multigoal has no {other:?} fixture build"
            )));
        }
    };
    cell.get_or_try_init(|| {
        let row = crate::fixtures::groups::select_row(
            "packages/testing/nros-tests/bins/action-client-multigoal",
            &crate::fixtures::groups::FixtureVariant::rmw(rmw),
        )?;
        let profile = cargo_target_profile_dir();
        let rel = PathBuf::from(format!("{profile}/action-client-multigoal"));
        require_prebuilt_row_binary_fresh(row, &rel)
    })
    .map(|p| p.as_path())
}

/// rstest fixture that provides the multi-goal action-client binary (Zenoh).
#[rstest::fixture]
pub fn action_client_multigoal_binary() -> PathBuf {
    build_action_client_multigoal(Rmw::Zenoh)
        .expect("Failed to build action-client-multigoal (zenoh)")
        .to_path_buf()
}

/// rstest fixture that provides the concurrent action-server fixture binary
/// (Zenoh build).
#[rstest::fixture]
pub fn action_server_concurrent_binary() -> PathBuf {
    build_action_server_concurrent(Rmw::Zenoh)
        .expect("Failed to build action-server-concurrent (zenoh)")
        .to_path_buf()
}

/// rstest fixture that provides the concurrent action-server fixture binary
/// (XRCE build).
#[rstest::fixture]
pub fn xrce_action_server_concurrent_binary() -> PathBuf {
    build_action_server_concurrent(Rmw::Xrce)
        .expect("Failed to build action-server-concurrent (xrce)")
        .to_path_buf()
}

/// Resolve a build-stage "compile-check" fixture's `.compile-ok` stamp (issue
/// 0034). `scripts/build/compile-check-fixtures.sh` (run by
/// `build-test-fixtures`) stages the template, rewrites placeholders, runs
/// `cargo check`, and writes the stamp on success — so a test asserts the stamp
/// instead of running `cargo check` at run time. Tier-aware via
/// `require_prebuilt_binary` (hard-fail in full tier → run `build-test-fixtures`;
/// `[SKIPPED]` under `NROS_FIXTURES_OPTIONAL=1`).
pub fn require_compile_check(id: &str) -> TestResult<PathBuf> {
    let stamp = build_dir(crate::kind::COMPILE_CHECK, &[id]).join(".compile-ok");
    require_prebuilt_binary_fresh(&stamp)
}

/// Resolve a build-stage build-fixture's binary (issue 0034). The build entries
/// in `compile-check-fixtures.sh` run `cargo build -p demo_entry`, producing a
/// runnable binary at `build/compile-check-fixtures/<id>/<rel>` (e.g.
/// `target/debug/demo_entry`) that a test executes. Tier-aware like
/// `require_compile_check`.
pub fn require_compile_check_bin(id: &str, rel: &str) -> TestResult<PathBuf> {
    let bin = build_dir(crate::kind::COMPILE_CHECK, &[id]).join(rel);
    require_prebuilt_binary_fresh(&bin)
}

/// Resolve a file inside a build-stage **cmake** fixture's persistent build dir
/// (issue 0034). `compile-check-fixtures.sh` cmake-configures + builds a C/C++
/// template into `build/cmake-fixtures/<id>/`, keeping generated TUs / link
/// sidecars / depfiles + the produced executable so a test can inspect / run /
/// `nm` them instead of running cmake at run time. Tier-aware (the cmake build
/// is skipped when cmake or a `codegen entry`-capable `nros` is absent → the
/// fixture file is missing → `[SKIPPED]` under `NROS_FIXTURES_OPTIONAL`, hard
/// fail in the full tier).
pub fn require_cmake_fixture(id: &str, rel: &str) -> TestResult<PathBuf> {
    let p = build_dir(crate::kind::CMAKE_FIXTURES, &[id]).join(rel);
    require_prebuilt_binary_fresh(&p)
}

/// Resolve a file inside a build-stage **esp-idf** fixture (issue 0041).
/// `scripts/build/idf-fixtures.sh` stages an esp-idf example/fixture and runs
/// `idf.py set-target && build` into `build/idf-fixtures/<id>/`, producing an ELF
/// the test resolves instead of running `idf.py` at run time. Tier-aware: the
/// idf build is skipped (no stamp / no ELF) when idf.py/IDF_PATH is absent →
/// `[SKIPPED]` under `NROS_FIXTURES_OPTIONAL`, hard fail in the full tier.
/// The coordinate the esp-idf bringup fixtures occupy — issue 0700.
///
/// They have no `[[fixture]]` row, so `attribute_path` cannot place them and
/// `skip_reason_for_path`'s `?` reads "cannot attribute" as "not out of lane",
/// i.e. RUN IT. That rule is right (CLAUDE.md: an unattributable path is never
/// skipped) and it assumes something in the lane built the artifact. Nothing
/// does: `build-test-fixtures` has zero references to esp32/idf/platformio at
/// any lane, `lane=all` included, so a gated tier-2 run selected these on any
/// host with the toolchain and then failed on a promise no build ever made.
///
/// Same shape, same remedy as the px4 companion above: state the coordinate
/// here, which is what `require_coord_in_lane` exists for.
fn idf_bringup_coord() -> crate::fixtures::lane::Coord {
    (
        "esp32".to_string(),
        "c".to_string(),
        Rmw::Xrce.coord_token().to_string(),
    )
}

pub fn require_idf_fixture(id: &str, rel: &str) -> TestResult<PathBuf> {
    // Toolchain-gated via the test-all env_exclude (deselect when idf.py absent);
    // resolves the prebuilt ELF here. Built by `just esp32 build-fixtures`.
    //
    // Issue 0700 — the LANE decides first. Selection keys on "can this host
    // build it" and the build lane keys on "is this coordinate in the lane";
    // without this, the two never met and every provisioned host saw a
    // "broken promise" indistinguishable from a real regression.
    crate::fixtures::lane::require_coord_in_lane(&idf_bringup_coord(), id)?;
    let p = build_dir(crate::kind::IDF_FIXTURES, &[id]).join(rel);
    require_prebuilt_binary_fresh(&p)
}

/// Resolve a file inside a build-stage **zephyr west** fixture (issue 0041).
/// `scripts/build/west-fixtures.sh` `west build`s a zephyr bringup fixture into
/// `build/west-fixtures/<id>/`, keeping baked artifacts / CMakeCache / zephyr.exe
/// the test inspects instead of running west at run time. Tier-aware: the west
/// build is skipped (no stamp) when west / a provisioned Zephyr workspace is
/// absent → `[SKIPPED]` under `NROS_FIXTURES_OPTIONAL`, hard fail in the full tier.
pub fn require_west_fixture(id: &str, rel: &str) -> TestResult<PathBuf> {
    // Toolchain-gated via the test-all env_exclude (deselect when west / Zephyr
    // SDK absent); resolves the prebuilt artifact here. Built by `just zephyr
    // build-fixtures`.
    let fixture_dir = build_dir(crate::kind::WEST_FIXTURES, &[id]);

    // #185 (the #182 guard, west edition) — the bake is a function of the
    // `nros` CLI (nros_system_generate / nros_generate_interfaces run it at
    // configure time), so a fixture built with a different CLI is a museum
    // bake even when every source file looks current. The build script stamps
    // `tool:nros=<sha256>` into `.compile-ok`; compare against the current
    // binary and fail loud instead of soft-passing. A date-only legacy stamp
    // reads as stale (one rebuild refreshes it).
    let stamp = fixture_dir.join(".compile-ok");
    if let Ok(stamp_text) = fs::read_to_string(&stamp) {
        let stamped_tool = stamp_text
            .lines()
            .find_map(|l| l.strip_prefix("tool:nros="))
            .map(str::trim);
        let cli = project_root().join("packages/cli/target/release/nros");
        // Same hasher the stamping script uses (sha256sum) — no new dep, and
        // byte-identical output by construction.
        let current = Command::new("sha256sum")
            .arg(&cli)
            .output()
            .ok()
            .and_then(|o| {
                o.status.success().then(|| {
                    String::from_utf8_lossy(&o.stdout)
                        .split_whitespace()
                        .next()
                        .unwrap_or_default()
                        .to_string()
                })
            });
        let fresh = match (stamped_tool, current.as_deref()) {
            (Some(stamped), Some(current)) => stamped == current,
            // CLI missing on disk: can't judge — let the artifact check decide.
            (Some(_), None) => true,
            // Legacy date-only stamp: pre-guard build → stale.
            (None, _) => false,
        };
        if !fresh {
            return Err(staleness::stale_error_custom(
                &fixture_dir.join(rel),
                format!(
                    "West fixture {id} is STALE — built with a different `nros` CLI \
                     than the current packages/cli/target/release/nros.\n  stamp: {}\n\
                     Run `bash scripts/build/west-fixtures.sh` (or `just zephyr \
                     build-fixtures`) to rebuild.",
                    stamp.display()
                ),
            ));
        }
    }

    require_prebuilt_binary_fresh(&fixture_dir.join(rel))
}

/// Resolve the prebuilt `entry-poc` fixture (cached). The
/// `packages/testing/nros-tests/bins/entry-poc` Entry pkg (`nros::main!()` →
/// native `BoardEntry::run`) is built by `just native build-fixtures` /
/// `build-test-fixtures` (examples/fixtures.toml). Tests consume the artifact
/// instead of running `cargo build` at run time (issue 0034).
pub fn build_entry_poc() -> TestResult<&'static Path> {
    static ENTRY_POC_BINARY: OnceCell<PathBuf> = OnceCell::new();
    ENTRY_POC_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_sole_row(
                "packages/testing/nros-tests/bins/entry-poc",
            )?;
            let profile = cargo_target_profile_dir();
            let rel = PathBuf::from(format!("{profile}/entry-poc"));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// Phase 118 — collapsed-shape native talker, RMW-parametrized.
///
/// Returns the prebuilt binary for the named RMW. Phase 220.C path B
/// retired the cmake/corrosion cyclonedds bridge; every RMW (incl.
/// Cyclone) now resolves to a pure-cargo `target-<rmw>/<profile>/talker`
/// binary produced by `just native build-fixtures`. Cached per RMW so
/// repeated lookups in a nextest run avoid filesystem-stat overhead.
pub fn build_native_talker_rmw(rmw: Rmw) -> TestResult<&'static Path> {
    static ZENOH_CELL: OnceCell<PathBuf> = OnceCell::new();
    static XRCE_CELL: OnceCell<PathBuf> = OnceCell::new();
    static CYCLONEDDS_CELL: OnceCell<PathBuf> = OnceCell::new();
    let cell = match rmw {
        Rmw::Zenoh => &ZENOH_CELL,
        Rmw::Xrce => &XRCE_CELL,
        Rmw::Cyclonedds => &CYCLONEDDS_CELL,
    };
    cell.get_or_try_init(|| build_example_rmw("native/rust/talker", "talker", rmw))
        .map(|p| p.as_path())
}

/// Phase 118 — collapsed-shape native listener, RMW-parametrized.
///
/// See `build_native_talker_rmw` — same pure-cargo path post-220.C.
pub fn build_native_listener_rmw(rmw: Rmw) -> TestResult<&'static Path> {
    static ZENOH_CELL: OnceCell<PathBuf> = OnceCell::new();
    static XRCE_CELL: OnceCell<PathBuf> = OnceCell::new();
    static CYCLONEDDS_CELL: OnceCell<PathBuf> = OnceCell::new();
    let cell = match rmw {
        Rmw::Zenoh => &ZENOH_CELL,
        Rmw::Xrce => &XRCE_CELL,
        Rmw::Cyclonedds => &CYCLONEDDS_CELL,
    };
    cell.get_or_try_init(|| build_example_rmw("native/rust/listener", "listener", rmw))
        .map(|p| p.as_path())
}

/// Phase 118 — generic native Rust example resolver. Cuts repetition
/// when the test only needs a single (case, rmw) tuple instead of the
/// pre-cached talker/listener wrappers.
pub fn build_native_rust_example_rmw(
    case: &str,
    binary_name: &str,
    rmw: Rmw,
) -> TestResult<PathBuf> {
    build_example_rmw(&format!("native/rust/{}", case), binary_name, rmw)
}

/// Phase 118 — generic native C example resolver. `case` is the
/// directory name under `examples/native/c/` (talker, listener,
/// service-server, …); `binary_name` is the cmake target (e.g.
/// `c_talker`, `c_service_server`, …).
pub fn build_native_c_example_rmw(case: &str, binary_name: &str, rmw: Rmw) -> TestResult<PathBuf> {
    build_example_cmake_rmw(&format!("native/c/{}", case), binary_name, rmw)
}

/// Phase 118 — generic native C++ example resolver. Mirror of the C
/// helper for `examples/native/cpp/<case>/`.
pub fn build_native_cpp_example_rmw(
    case: &str,
    binary_name: &str,
    rmw: Rmw,
) -> TestResult<PathBuf> {
    build_example_cmake_rmw(&format!("native/cpp/{}", case), binary_name, rmw)
}

/// Phase 118.C — collapsed-shape ThreadX-RV64 Rust example resolver.
/// Zenoh uses the pure-cargo target dir; CycloneDDS uses the
/// CMake/Corrosion staticlib path added in Phase 175.B.
pub fn build_threadx_rv64_rust_example_rmw(
    case: &str,
    binary_name: &str,
    rmw: Rmw,
) -> TestResult<PathBuf> {
    let root = project_root();
    let example_dir = root.join(format!("examples/qemu-riscv64-threadx/rust/{}", case));
    if !example_dir.exists() {
        return Err(TestError::BuildFailed(format!(
            "Example directory not found: {}",
            example_dir.display()
        )));
    }
    // phase-369 W4 — BOTH RMWs resolve through the cmake build dir.
    //
    // The zenoh arm used to fall through to `select_row()`, the CARGO row, because
    // the zenoh image was a cargo bin built from `src/main.rs`. W2/W3 moved it onto
    // the same cmake seam as cyclone and deleted that entry point, so the artifact
    // now lives at `<dir>/build-zenoh/<target>` exactly as cyclone's lives at
    // `<dir>/build-cyclonedds/<target>`. `Rmw::build_dir()` already spells both.
    // Both rows are `builder = "cmake"` now (cyclonedds since phase-344 W2, zenoh
    // since phase-369 W2), and `rmw.build_dir()` and `cmake_build_subdir`'s
    // default are the same expression, `build-<rmw>`.
    //
    // The cargo route that used to follow — `select_row()` plus a
    // `riscv64gc-unknown-none-elf/<profile>/<bin>` join — is DELETED rather than
    // left unreachable: after W3 there is no cargo bin to find, because
    // `src/main.rs` and the `[[bin]]` section are gone.
    let binary_path = example_dir.join(format!("{}/{}", rmw.build_dir(), binary_name));
    require_prebuilt_binary_fresh(&binary_path)
}

/// Phase 118.B.7 — collapsed-shape threadx-linux Rust example resolver.
pub fn build_threadx_linux_rust_example_rmw(
    case: &str,
    binary_name: &str,
    rmw: Rmw,
) -> TestResult<PathBuf> {
    build_example_rmw(&format!("threadx-linux/rust/{}", case), binary_name, rmw)
}

/// Phase 118.B.7 — collapsed-shape threadx-linux C / C++ example resolver.
pub fn build_threadx_linux_cmake_example_rmw(
    lang: &str,
    case: &str,
    binary_name: &str,
    rmw: Rmw,
) -> TestResult<PathBuf> {
    build_example_cmake_rmw(
        &format!("threadx-linux/{}/{}", lang, case),
        binary_name,
        rmw,
    )
}

/// Phase 168.1 — collapsed-shape Zephyr Rust example resolver.
///
/// Zephyr west builds drop the artifact at
/// `zephyr-workspace/build-rs-<case>-<rmw>/zephyr/zephyr.exe` (not
/// inside the example dir), so this helper resolves to that path
/// instead of using `build_example_rmw`. `case` is the directory
/// name under `examples/zephyr/rust/` (talker, listener, …).
fn zephyr_build_root() -> PathBuf {
    if let Some(path) = std::env::var_os("NROS_ZEPHYR_BUILD_ROOT") {
        return PathBuf::from(path);
    }
    let root = project_root();
    // Mirror just/zephyr.just's ZEPHYR_WORKSPACE selection: the in-tree
    // `zephyr-workspace` (canonical), else the legacy `../nano-ros-workspace`
    // sibling. The build stages fixtures into whichever it picks (when
    // writable), falling back to `build/zephyr-workspace-builds` only when no
    // writable workspace exists — so the resolver must look in the same order.
    let in_tree = root.join("zephyr-workspace");
    let workspace = if in_tree.is_dir() || in_tree.is_symlink() {
        in_tree
    } else {
        match root.parent().map(|p| p.join("nano-ros-workspace")) {
            Some(sibling) if sibling.is_dir() => sibling,
            _ => in_tree,
        }
    };
    if workspace
        .metadata()
        .map(|m| !m.permissions().readonly())
        .unwrap_or(false)
    {
        workspace
    } else {
        root.join("build/zephyr-workspace-builds")
    }
}

// issue 1016 — `build_zephyr_rust_example_rmw` stood here and named
// `build-rs-<case>-<rmw>`. That is the `rs` LANG TAG issue 0539 retired from
// both producers when the build side stopped spelling it: the west lane writes
// `build-rust-<case>-<rmw>` and `fixtures-manifest.py west-leaves` models only
// that. So this resolver could only ever name a build dir no lane builds — and
// `require_west_leaf_in_lane` fails OPEN on a name it cannot find, so no lane
// could skip it either. It had no callers; it was
// `get_prebuilt_zephyr_example("zephyr-rs-<case>", …)` spelt a second, wrong
// way. `check-west-leaf-vocabulary.py` now refuses the class.

/// phase-337 W2.f — the Zephyr Cortex-M witness (`mps2_an385`) leaf resolver.
///
/// Two things differ from every sibling above, and both are the point of the
/// board rather than incidental:
///
/// * `zephyr.elf`, not `zephyr.exe`. native_sim builds a HOST executable the
///   harness runs directly; this is a real image QEMU loads with `-kernel`.
/// * the build name carries a `cortex-m` segment, because the same
///   `(lang, role, rmw)` coordinate already names a native_sim leaf — the board
///   is the thing that distinguishes them, and it has to appear in the path or
///   the two builds collide in one directory.
///
/// `lang` is `"c"` or `"cpp"`. There is no rust arm: issue 0432 blocks the
/// `zephyr` crate on any board with gpio nodes.
pub fn build_zephyr_cortex_m_example(lang: &str, case: &str, rmw: Rmw) -> TestResult<PathBuf> {
    let root = project_root();
    let example_dir = root.join(format!("examples/zephyr/{}/{}", lang, case));
    if !example_dir.exists() {
        return Err(TestError::BuildFailed(format!(
            "Example directory not found: {}",
            example_dir.display()
        )));
    }
    let binary_path = zephyr_build_root().join(format!(
        "build-cortex-m-{}-{}-{}/zephyr/zephyr.elf",
        lang,
        case,
        rmw.cmake_value()
    ));
    let leaf = format!("examples/zephyr/{lang}/{case}");
    require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        ZephyrLeafSource {
            dir: &leaf,
            lang: Some(lang),
            rmw: Some(rmw.cmake_value()),
            conf_files: None,
        },
    )
}

/// Phase 168.4 — collapsed-shape Zephyr C / C++ example resolver.
/// `lang` is `"c"` or `"cpp"`. Mirrors the Rust resolver.
pub fn build_zephyr_cmake_example_rmw(lang: &str, case: &str, rmw: Rmw) -> TestResult<PathBuf> {
    let root = project_root();
    let example_dir = root.join(format!("examples/zephyr/{}/{}", lang, case));
    if !example_dir.exists() {
        return Err(TestError::BuildFailed(format!(
            "Example directory not found: {}",
            example_dir.display()
        )));
    }
    let binary_path = zephyr_build_root().join(format!(
        "build-{}-{}-{}/zephyr/zephyr.exe",
        lang,
        case,
        rmw.cmake_value()
    ));
    let leaf = format!("examples/zephyr/{lang}/{case}");
    require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        ZephyrLeafSource {
            dir: &leaf,
            lang: Some(lang),
            rmw: Some(rmw.cmake_value()),
            conf_files: None,
        },
    )
}

/// phase-263 C2d — the Zephyr (native_sim) C WORKSPACE embedded entry (talker + listener),
/// built by the west lane (`zephyr-fixture-leaves.sh --include-workspace-entry`) into
/// `<zephyr-build-root>/build-ws-c-entry-zenoh/zephyr/zephyr.exe`. The C/C++ sibling of the
/// Rust workspace zephyr entry; consumed by `tests/entry_e2e.rs` (zephyr_c cell).
pub fn build_zephyr_workspace_c_entry() -> TestResult<PathBuf> {
    let binary_path = zephyr_build_root().join("build-ws-c-entry-zenoh/zephyr/zephyr.exe");
    require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        ZephyrLeafSource::zenoh("examples/workspaces/c/src/zephyr_entry", "c"),
    )
}

/// phase-263 C2c — the Zephyr (native_sim) C++ WORKSPACE embedded entry (talker + listener,
/// typed `std_msgs::msg::Int32`), built by the west lane into
/// `<zephyr-build-root>/build-ws-cpp-entry-zenoh/zephyr/zephyr.exe`. Consumed by
/// `tests/entry_e2e.rs` (zephyr_cpp cell).
pub fn build_zephyr_workspace_cpp_entry() -> TestResult<PathBuf> {
    let binary_path = zephyr_build_root().join("build-ws-cpp-entry-zenoh/zephyr/zephyr.exe");
    require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        ZephyrLeafSource::zenoh("examples/workspaces/cpp/src/zephyr_entry", "cpp"),
    )
}

/// phase-263 C2c-zephyr — the Zephyr (native_sim) MIXED WORKSPACE embedded entry (C talker +
/// C++ listener + Rust heartbeat), built by the west lane into
/// `<zephyr-build-root>/build-ws-mixed-entry-zenoh/zephyr/zephyr.exe`. The Rust node is bundled
/// into the `nros_ws_runtime` umbrella staticlib (single Rust runtime). Consumed by
/// `tests/entry_e2e.rs` (zephyr_mixed cell).
pub fn build_zephyr_workspace_mixed_entry() -> TestResult<PathBuf> {
    let binary_path = zephyr_build_root().join("build-ws-mixed-entry-zenoh/zephyr/zephyr.exe");
    require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        // The mixed entry links C, C++ AND Rust nodes, so `lang: None` —
        // no language API crate may be dropped from the watch set.
        ZephyrLeafSource {
            dir: "examples/workspaces/mixed/src/zephyr_entry",
            lang: None,
            rmw: Some("zenoh"),
            conf_files: None,
        },
    )
}

/// phase-276 W1 (#128) — the Zephyr (native_sim) PARAMETERISED Rust workspace Entry
/// (`features/src/zephyr_rust_params_entry`): the param_talker node + the six ROS 2 parameter
/// services (the #128 fix made the `Framework::Zephyr` macro arm emit that registration).
/// Built by the west lane into `<zephyr-build-root>/build-ws-rs-params-entry-zenoh/zephyr/
/// zephyr.exe`; consumed by `tests/entry_e2e.rs` (zephyr_rust_params cell).
pub fn build_zephyr_workspace_rust_params_entry() -> TestResult<PathBuf> {
    let binary_path = zephyr_build_root().join("build-ws-rs-params-entry-zenoh/zephyr/zephyr.exe");
    require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        ZephyrLeafSource::zenoh(
            "examples/workspaces/features/src/zephyr_rust_params_entry",
            "rust",
        ),
    )
}

/// phase-276 W3 (#128) — the Zephyr (native_sim) MANAGED (lifecycle) Rust workspace Entry
/// (`features/src/zephyr_rust_lifecycle_entry`): the talker node + the five REP-2002 lifecycle
/// services with boot autostart (the #128 `Framework::Zephyr` `apply_lifecycle` emit).
/// Built by the west lane into `<zephyr-build-root>/build-ws-rs-lifecycle-entry-zenoh/
/// zephyr/zephyr.exe`; consumed by `tests/entry_e2e.rs` (zephyr_rust_lifecycle cell).
pub fn build_zephyr_workspace_rust_lifecycle_entry() -> TestResult<PathBuf> {
    let binary_path =
        zephyr_build_root().join("build-ws-rs-lifecycle-entry-zenoh/zephyr/zephyr.exe");
    require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        ZephyrLeafSource::zenoh(
            "examples/workspaces/features/src/zephyr_rust_lifecycle_entry",
            "rust",
        ),
    )
}

/// phase-276 W5 — the Zephyr (native_sim) QOS-OVERRIDE Rust workspace Entry
/// (`features/src/zephyr_rust_qos_entry`): reliable_talker publishes `/qos_chatter` with a
/// non-default profile (reliable + transient_local) and qos_listener subscribes with the
/// byte-identical profile, republishing the matched receive count on `/qos_ok`. Built by
/// the west lane into `<zephyr-build-root>/build-ws-rs-qos-entry-zenoh/zephyr/zephyr.exe`;
/// consumed by `tests/entry_e2e.rs` (zephyr_rust_qos cell).
pub fn build_zephyr_workspace_rust_qos_entry() -> TestResult<PathBuf> {
    let binary_path = zephyr_build_root().join("build-ws-rs-qos-entry-zenoh/zephyr/zephyr.exe");
    require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        ZephyrLeafSource::zenoh(
            "examples/workspaces/features/src/zephyr_rust_qos_entry",
            "rust",
        ),
    )
}

/// phase-276 W4 — the Zephyr (native_sim) E2E-SAFETY (CRC) Rust workspace Entry
/// (`safety/src/zephyr_rust_safety_entry`): the system declares `features = ["safety"]`,
/// so the zenoh backend attaches the E2E CRC + sequence number on publish and
/// validates on receive; safe_listener republishes its CRC-VALIDATED count on
/// `/safe_ok`. Built by the west lane into
/// `<zephyr-build-root>/build-ws-rs-safety-entry-zenoh/zephyr/zephyr.exe`;
/// consumed by `tests/entry_e2e.rs` (zephyr_rust_safety cell).
pub fn build_zephyr_workspace_rust_safety_entry() -> TestResult<PathBuf> {
    let binary_path = zephyr_build_root().join("build-ws-rs-safety-entry-zenoh/zephyr/zephyr.exe");
    require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        ZephyrLeafSource::zenoh(
            "examples/workspaces/safety/src/zephyr_rust_safety_entry",
            "rust",
        ),
    )
}

/// phase-276 W6 — the Zephyr (native_sim) MULTIHOST robot1 (talker) Rust workspace
/// Entry (`workspaces/rust/src/zephyr_entry_robot1`): `nros::main!(model =
/// "demo_bringup:config/multihost_robot1_model.yaml")` bakes only the robot1
/// slice of the multi-host launch — the talker (the per-host model is resolved
/// with `host:=robot1`; phase-326 / issue 0364). Paired with the NATIVE robot2
/// (listener) per-host entry so `/chatter` crosses hosts. Built by the west lane
/// into `<zephyr-build-root>/build-ws-rs-mh-robot1-entry-zenoh/zephyr/zephyr.exe`;
/// consumed by `tests/multihost_e2e.rs` (zephyr_rust cell).
pub fn build_zephyr_workspace_rust_multihost_robot1_entry() -> TestResult<PathBuf> {
    let binary_path =
        zephyr_build_root().join("build-ws-rs-mh-robot1-entry-zenoh/zephyr/zephyr.exe");
    require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        ZephyrLeafSource::zenoh("examples/workspaces/rust/src/zephyr_entry_robot1", "rust"),
    )
}

/// phase-276 W2 / issue #128 half 2 — the Zephyr (native_sim) RT-TIERS Rust
/// workspace Entry (`realtime-rust/src/zephyr_entry`): `system.toml`
/// declares two `[tiers.*]` with `[tiers.*.zephyr]` priorities, so the macro
/// emits `ZephyrBoard::run_tiers` — one k_thread per tier over ONE shared
/// session; ctrl (10 ms, high) publishes `/ctrl`, telem (100 ms, low)
/// publishes `/telem`. Built by the west lane into
/// `<zephyr-build-root>/build-ws-rs-realtime-entry-zenoh/zephyr/zephyr.exe`;
/// consumed by `tests/realtime_tiers_zephyr_entry_e2e.rs`.
pub fn build_zephyr_workspace_rust_realtime_entry() -> TestResult<PathBuf> {
    let binary_path =
        zephyr_build_root().join("build-ws-rs-realtime-entry-zenoh/zephyr/zephyr.exe");
    require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        ZephyrLeafSource::zenoh("examples/workspaces/realtime-rust/src/zephyr_entry", "rust"),
    )
}

/// phase-281 W3b — the Zephyr (native_sim) RT-TIERS C++ workspace Entry
/// (`realtime-cpp/src/zephyr_entry`): the FIRST full west link + runtime proof of
/// the W3a `ZephyrBoard::run_tiers` seam. `demo_bringup/system.toml` declares two
/// `[tiers.*]` with `[tiers.*.zephyr]` priorities, so the C++ codegen emits a plain
/// `int main(void)` calling `ZephyrBoard::run_tiers` (`nros_board_zephyr_run_tiers`) —
/// one k_thread per tier over ONE shared session; ctrl (10 ms, high) publishes `/ctrl`,
/// telem (100 ms, low) publishes `/telem`. Built by the west lane into
/// `<zephyr-build-root>/build-ws-cpp-realtime-entry-zenoh/zephyr/zephyr.exe`; consumed by
/// `tests/realtime_tiers_cpp_zephyr_e2e.rs`.
pub fn build_zephyr_workspace_cpp_realtime_entry() -> TestResult<PathBuf> {
    let binary_path =
        zephyr_build_root().join("build-ws-cpp-realtime-entry-zenoh/zephyr/zephyr.exe");
    require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        ZephyrLeafSource::zenoh("examples/workspaces/realtime-cpp/src/zephyr_entry", "cpp"),
    )
}

/// phase-281 W3c — the Zephyr (native_sim) RT-TIERS C workspace Entry
/// (`realtime-c/src/zephyr_entry`): the FIRST full west link + runtime proof of the
/// W3a `ZephyrBoard::run_tiers` seam for a C node (closes the c×zephyr cell).
/// `demo_bringup/system.toml` declares two `[tiers.*]` with `[tiers.*.zephyr]` priorities,
/// so the C codegen emits a plain `int main(void)` calling `ZephyrBoard::run_tiers`
/// (`nros_board_zephyr_run_tiers`) — one k_thread per tier over ONE shared session; ctrl
/// (10 ms, high) publishes `/ctrl`, telem (100 ms, low) publishes `/telem`. Built by the
/// west lane into `<zephyr-build-root>/build-ws-c-realtime-entry-zenoh/zephyr/zephyr.exe`;
/// consumed by `tests/realtime_tiers_c_zephyr_e2e.rs`.
pub fn build_zephyr_workspace_c_realtime_entry() -> TestResult<PathBuf> {
    let binary_path = zephyr_build_root().join("build-ws-c-realtime-entry-zenoh/zephyr/zephyr.exe");
    require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        ZephyrLeafSource::zenoh("examples/workspaces/realtime-c/src/zephyr_entry", "c"),
    )
}

/// issue 0260 / phase-356 W3 — the SMP realtime entry.
///
/// `zephyr.elf`, not `zephyr.exe`: this is a real aarch64 image for
/// `qemu_cortex_a53/.../smp`, the tree's only multi-core Zephyr target, where a
/// `core` dim can be verified as PLACEMENT rather than only as acceptance.
/// Same workspace and entry as the native_sim C row — the fixture axis is the
/// board, which selects `smp_bringup` (`core = 1`) via `CONFIG_SMP`.
pub fn build_zephyr_workspace_c_realtime_entry_smp() -> TestResult<PathBuf> {
    let binary_path = zephyr_build_root().join("build-ws-c-realtime-entry-smp/zephyr/zephyr.elf");
    require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        ZephyrLeafSource::zenoh("examples/workspaces/realtime-c/src/zephyr_entry", "c"),
    )
}

/// Phase 118.C — collapsed-shape ThreadX-RV64 C / C++ example resolver.
pub fn build_threadx_rv64_cmake_example_rmw(
    lang: &str,
    case: &str,
    binary_name: &str,
    rmw: Rmw,
) -> TestResult<PathBuf> {
    build_example_cmake_rmw(
        &format!("qemu-riscv64-threadx/{}/{}", lang, case),
        binary_name,
        rmw,
    )
}

/// Phase 118.B.5 — collapsed-shape NuttX C / C++ example resolver.
pub fn build_nuttx_cmake_example_rmw(
    lang: &str,
    case: &str,
    binary_name: &str,
    rmw: Rmw,
) -> TestResult<PathBuf> {
    build_example_cmake_rmw(
        &format!("qemu-arm-nuttx/{}/{}", lang, case),
        binary_name,
        rmw,
    )
}

/// Phase 118.D — collapsed-shape FreeRTOS C / C++ example resolver.
/// `lang` is `"c"` or `"cpp"`. Binary lands at
/// `examples/qemu-arm-freertos/<lang>/<case>/build-<rmw>/<binary>`.
pub fn build_freertos_cmake_example_rmw(
    lang: &str,
    case: &str,
    binary_name: &str,
    rmw: Rmw,
) -> TestResult<PathBuf> {
    build_example_cmake_rmw(
        &format!("qemu-arm-freertos/{}/{}", lang, case),
        binary_name,
        rmw,
    )
}

/// Phase 118.D — collapsed-shape FreeRTOS Rust example resolver.
///
/// Phase 220.C path B — the CycloneDDS Rust fixture is retired from the
/// cmake/corrosion bridge (`build-cyclonedds/`); a pure-cargo FreeRTOS
/// cyclonedds path is deferred behind Phase 214.S.5.b's BSP gate
/// (cyclonedds-sys vendored build against the ARM cross toolchain +
/// FreeRTOS POSIX shim). Until that lands the cyclonedds branch returns
/// a `BuildFailed` error so callers (`freertos_qemu.rs`) emit the
/// proper `nros_tests::skip!` rather than silently passing.
///
/// phase-340 P2 — the ZENOH arm used to spell
/// `target-zenoh/thumbv7m-none-eabi/<ambient profile>/<binary>`, an output the
/// manifest stopped producing when the six FreeRTOS rust rows were repointed at
/// the build the tests actually consume (the `target-zenoh` half had no live
/// reader: this function's only two callers are the `#[ignore]`d cyclonedds
/// tests, which never reach here). Leaving a resolver pointing at a directory
/// nothing writes is how issue 0475 and phase-340 item 7 both started, so the
/// arm now delegates to the one FreeRTOS entry resolver.
pub fn build_freertos_rust_example_rmw(
    case: &str,
    binary_name: &str,
    rmw: Rmw,
) -> TestResult<PathBuf> {
    let root = project_root();
    let example_dir = root.join(format!("examples/qemu-arm-freertos/rust/{}", case));
    if !example_dir.exists() {
        return Err(TestError::BuildFailed(format!(
            "Example directory not found: {}",
            example_dir.display()
        )));
    }
    if rmw != Rmw::Zenoh {
        return Err(TestError::BuildFailed(format!(
            "Phase 220.C path B: FreeRTOS rust {:?} fixture retired \
             (cmake-bridge removed; pure-cargo path blocked on Phase \
             214.S.5.b BSP gate). Requested: {}/{}",
            rmw, case, binary_name
        )));
    }
    freertos::require_entry_binary(case)
}

/// Build native-rs-listener (cached)
pub fn build_native_listener() -> TestResult<&'static Path> {
    NATIVE_LISTENER_BINARY
        .get_or_try_init(|| build_example("native/rust/listener", "listener", None, None))
        .map(|p| p.as_path())
}

static NATIVE_RS_LOGGING_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// #102 H3 — build the `native/rust/logging` example (cached). Distinct from the
/// `logging-smoke-*` bins: after firing every severity once it RAISES the runtime
/// threshold to `Warn` and fires them again, so the consuming e2e can assert the
/// runtime level filter (round-2 trace/debug/info suppressed) that the smoke bins
/// never exercise.
pub fn build_native_logging() -> TestResult<&'static Path> {
    NATIVE_RS_LOGGING_BINARY
        .get_or_try_init(|| build_example("native/rust/logging", "logging", None, None))
        .map(|p| p.as_path())
}

/// Phase 115.F — build the custom-transport talker example (cached).
pub fn build_native_custom_transport_talker() -> TestResult<&'static Path> {
    NATIVE_CT_TALKER_BINARY
        .get_or_try_init(|| {
            build_example(
                "native/rust/custom-transport-talker",
                // phase-340 W2 — was `talker`, which collided with
                // `native-rs-talker`'s binary inside the `linux` default
                // fixture group's one flat output namespace.
                "custom-transport-talker",
                None,
                None,
            )
        })
        .map(|p| p.as_path())
}

/// Phase 211.I — resolve the prebuilt mixed-RMW bridge fixture binary
/// (`packages/testing/nros-tests/bins/bridge-zenoh-to-xrce-fwd`). Used by
/// `tests/bridge_mixed_rmw.rs` to forward zenoh `/chatter` samples into an
/// XRCE-DDS session. A minimal sibling to the Phase 110.G
/// `tt-zenoh-to-xrce` example: same dual-session topology, but the type
/// name matches `std_msgs::msg::Int32` (the type the talker/listener
/// fixtures use) and no TT-window gating — the 211.I assertion is "a
/// sample crosses the RMW boundary", which the TT example's String-type
/// constants would block at keyexpr registration.
///
/// The fixture sits in its own Cargo workspace (`[workspace]` table); the
/// test skips cleanly when the binary is missing.
pub fn build_bridge_zenoh_to_xrce_fwd() -> TestResult<&'static Path> {
    NATIVE_BRIDGE_TT_ZENOH_XRCE_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_sole_row(
                "packages/testing/nros-tests/bins/bridge-zenoh-to-xrce-fwd",
            )?;
            let profile = cargo_target_profile_dir();
            let rel = PathBuf::from(format!("{profile}/bridge-zenoh-to-xrce-fwd"));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// Issue #53 — resolve the prebuilt `bridge-zenoh-to-cyclonedds-fwd` fixture
/// (`packages/testing/nros-tests/bins/bridge-zenoh-to-cyclonedds-fwd`). Used by
/// `tests/bridge_zenoh_to_cyclonedds.rs` to forward zenoh `/chatter` samples onto
/// a Cyclone DDS egress session. The stock-cyclonedds sibling of
/// [`build_bridge_zenoh_to_xrce_fwd`]; links the vendored CycloneDDS and stages
/// the `std_msgs/Int32` descriptor before raw publish. Its own Cargo workspace;
/// the test skips cleanly when the binary is missing.
pub fn build_bridge_zenoh_to_cyclonedds_fwd() -> TestResult<&'static Path> {
    NATIVE_BRIDGE_ZENOH_CYCLONEDDS_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_sole_row(
                "packages/testing/nros-tests/bins/bridge-zenoh-to-cyclonedds-fwd",
            )?;
            let profile = cargo_target_profile_dir();
            let rel = PathBuf::from(format!("{profile}/bridge-zenoh-to-cyclonedds-fwd"));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// Phase 211.H — resolve the prebuilt `qos-override-pubsub` fixture binary
/// (`packages/testing/nros-tests/bins/qos-override-pubsub`). Used by
/// `tests/qos_overrides_runtime_delivery.rs` to prove a per-topic QoS override
/// is honoured on a live entity + still delivers cross-process. Its own Cargo
/// workspace; the test skips cleanly when the binary is missing.
pub fn build_qos_override_pubsub() -> TestResult<&'static Path> {
    NATIVE_QOS_OVERRIDE_PUBSUB_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_sole_row(
                "packages/testing/nros-tests/bins/qos-override-pubsub",
            )?;
            let profile = cargo_target_profile_dir();
            let rel = PathBuf::from(format!("{profile}/qos-override-pubsub"));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// Phase 250 Wave 5 — resolve the prebuilt `declarative-safety-listener` fixture
/// (`packages/testing/nros-tests/bins/declarative-safety-listener`). A
/// declarative (`Node` + `.safety()`) subscriber that surfaces `ctx.integrity()`;
/// paired with the safety talker in `tests/safety_e2e.rs`. Own Cargo workspace;
/// the test skips cleanly when the binary is missing.
pub fn build_native_declarative_safety_listener() -> TestResult<&'static Path> {
    NATIVE_DECLARATIVE_SAFETY_LISTENER_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_sole_row(
                "packages/testing/nros-tests/bins/declarative-safety-listener",
            )?;
            let profile = cargo_target_profile_dir();
            let rel = PathBuf::from(format!("{profile}/declarative-safety-listener"));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// Phase 211 acceptance — resolve the prebuilt `ros2-string-interop` fixture
/// binary (`packages/testing/nros-tests/bins/ros2-string-interop`). A nano-ros
/// raw `std_msgs/String` subscriber on `/chatter`, paired with a stock
/// `demo_nodes_cpp talker` in `tests/demo_nodes_cpp_interop.rs`. Own Cargo
/// workspace; the test skips cleanly when the binary is missing.
pub fn build_ros2_string_interop() -> TestResult<&'static Path> {
    NATIVE_ROS2_STRING_INTEROP_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_sole_row(
                "packages/testing/nros-tests/bins/ros2-string-interop",
            )?;
            let profile = cargo_target_profile_dir();
            let rel = PathBuf::from(format!("{profile}/ros2-string-interop"));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// Phase 115.F — build the custom-transport listener example (cached).
pub fn build_native_custom_transport_listener() -> TestResult<&'static Path> {
    NATIVE_CT_LISTENER_BINARY
        .get_or_try_init(|| {
            build_example(
                "native/rust/custom-transport-listener",
                // phase-340 W2 — was `listener`; see the talker sibling.
                "custom-transport-listener",
                None,
                None,
            )
        })
        .map(|p| p.as_path())
}

/// Build native-rs-lifecycle-node (cached)
///
/// Enables `lifecycle-services` so the `ros2 lifecycle *` service surface
/// is exposed for interop tests.
pub fn build_native_lifecycle_node() -> TestResult<&'static Path> {
    NATIVE_LIFECYCLE_NODE_BINARY
        .get_or_try_init(|| {
            build_example(
                "native/rust/lifecycle-node",
                "lifecycle-node",
                Some(&["lifecycle-services"]),
                None,
            )
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the qemu-test binary path
#[rstest::fixture]
pub fn qemu_binary() -> PathBuf {
    build_qemu_test()
        .expect("Failed to build qemu-test")
        .to_path_buf()
}

/// Cached path to the Phase 88.15.a `logging-smoke-mps2-baremetal`
/// fixture binary (bare-metal MPS2-AN385 nros-log smoke).
static LOGGING_SMOKE_MPS2_BAREMETAL_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Resolve the prebuilt Phase 88.15.a logging smoke binary. The
/// fixture must already be built (`just qemu build-fixtures`).
pub fn build_logging_smoke_mps2_baremetal() -> TestResult<&'static Path> {
    LOGGING_SMOKE_MPS2_BAREMETAL_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("logging-smoke-mps2-baremetal"))
        .map(|p| p.as_path())
}

/// Cached path to the Phase 88.15.b `logging-smoke-freertos-mps2`
/// fixture binary (MPS2-AN385 + FreeRTOS + lwIP nros-log smoke).
static LOGGING_SMOKE_FREERTOS_MPS2_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Resolve the prebuilt Phase 88.15.b logging smoke binary.
pub fn build_logging_smoke_freertos_mps2() -> TestResult<&'static Path> {
    LOGGING_SMOKE_FREERTOS_MPS2_BINARY
        .get_or_try_init(|| {
            // phase-340 P2 — the whole `freertos rust` fixture lane is built at
            // the `freertos-qemu` carve-out (`just freertos build-fixtures`
            // exports it as `NROS_CARGO_PROFILE`), so this row lands there too.
            build_test_fixture_at_profile(
                "nros-tests/bins/logging-smoke-freertos-mps2",
                "logging-smoke-freertos-mps2",
                Some("thumbv7m-none-eabi"),
                Some(nros_cargo_profile::FREERTOS_QEMU_PROFILE),
            )
        })
        .map(|p| p.as_path())
}

/// Cached path to the Phase 88.15.d `logging-smoke-threadx-riscv64`
/// fixture binary (ThreadX + NetX Duo on QEMU `virt` RV64).
static LOGGING_SMOKE_THREADX_RISCV64_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Resolve the prebuilt Phase 88.15.d logging smoke binary.
pub fn build_logging_smoke_threadx_riscv64() -> TestResult<&'static Path> {
    LOGGING_SMOKE_THREADX_RISCV64_BINARY
        .get_or_try_init(|| {
            build_test_fixture(
                "nros-tests/bins/logging-smoke-threadx-riscv64",
                "logging-smoke-threadx-riscv64",
                Some("riscv64gc-unknown-none-elf"),
            )
        })
        .map(|p| p.as_path())
}

/// Cached path to the issue-0697 `pool-exhaustion-threadx-linux` fixture binary.
static POOL_EXHAUSTION_THREADX_LINUX_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Resolve the prebuilt ThreadX Linux zenoh session-pool exhaustion binary.
///
/// issue 0697 — the arm this image executes was hardened for `no_std` targets by
/// issue 0589 and, until this fixture, had never run on one.
pub fn build_pool_exhaustion_threadx_linux() -> TestResult<&'static Path> {
    POOL_EXHAUSTION_THREADX_LINUX_BINARY
        .get_or_try_init(|| {
            build_test_fixture(
                "nros-tests/bins/pool-exhaustion-threadx-linux",
                "pool-exhaustion-threadx-linux",
                None,
            )
        })
        .map(|p| p.as_path())
}

/// Cached path to the `logging-smoke-threadx-linux` fixture binary.
static LOGGING_SMOKE_THREADX_LINUX_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Resolve the prebuilt ThreadX Linux logging smoke binary.
pub fn build_logging_smoke_threadx_linux() -> TestResult<&'static Path> {
    LOGGING_SMOKE_THREADX_LINUX_BINARY
        .get_or_try_init(|| {
            build_test_fixture(
                "nros-tests/bins/logging-smoke-threadx-linux",
                "logging-smoke-threadx-linux",
                None,
            )
        })
        .map(|p| p.as_path())
}

/// Cached path to the Phase 88.15.f `logging-smoke-esp32-qemu`
/// flash image (ESP32-C3 binary under stock `qemu-system-riscv32 -M
/// esp32c3`).
static LOGGING_SMOKE_ESP32_QEMU_FLASH: OnceCell<PathBuf> = OnceCell::new();

/// Resolve the prebuilt Phase 88.15.f logging smoke flash image.
/// Built by `just esp32 build-logging-smoke` (or whichever recipe
/// invokes the espflash `save-image` step against the fixture's
/// ELF output).
pub fn build_logging_smoke_esp32_qemu_flash() -> TestResult<&'static Path> {
    LOGGING_SMOKE_ESP32_QEMU_FLASH
        .get_or_try_init(|| {
            build_test_fixture(
                "nros-tests/bins/logging-smoke-esp32-qemu",
                "logging-smoke-esp32-qemu.bin",
                Some("riscv32imc-unknown-none-elf"),
            )
        })
        .map(|p| p.as_path())
}

/// Cached path to the Phase 88.15.c `logging-smoke-nuttx-qemu-arm`
/// fixture binary (NuttX flat-build kernel image for QEMU ARM virt).
static LOGGING_SMOKE_NUTTX_QEMU_ARM_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Resolve the prebuilt Phase 88.15.c logging smoke binary. Built
/// by `just nuttx build-fixtures` (folded the fixture into the same
/// parallel sweep that builds the NuttX example tree).
pub fn build_logging_smoke_nuttx_qemu_arm() -> TestResult<&'static Path> {
    LOGGING_SMOKE_NUTTX_QEMU_ARM_BINARY
        .get_or_try_init(|| {
            build_test_fixture(
                "nros-tests/bins/logging-smoke-nuttx-qemu-arm",
                "logging-smoke-nuttx-qemu-arm",
                Some("armv7a-nuttx-eabihf"),
            )
        })
        .map(|p| p.as_path())
}

/// Cached path to the Phase 88.15.e `logging-smoke-zephyr-native-sim`
/// fixture binary (Zephyr `native_sim/native/64` running as a Linux
/// process).
static LOGGING_SMOKE_ZEPHYR_NATIVE_SIM_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Resolve the prebuilt Phase 88.15.e logging smoke binary. Built
/// by `just zephyr build-logging-smoke` (or whichever recipe wires
/// the fixture into `just zephyr build-fixtures`). The Zephyr
/// `native_sim` flow emits a Linux ELF under
/// `<zephyr-workspace>/build-logging-smoke/zephyr/zephyr.exe`.
pub fn build_logging_smoke_zephyr_native_sim() -> TestResult<&'static Path> {
    LOGGING_SMOKE_ZEPHYR_NATIVE_SIM_BINARY
        .get_or_try_init(|| {
            let binary = zephyr_build_root().join("build-logging-smoke/zephyr/zephyr.exe");
            // Issue 0713 — a west leaf like any other, but resolved through the
            // generic freshness helper rather than the zephyr one, so the lane
            // call cannot ride along and is named here. Its manifest rmw label
            // is `default`, which `west_leaves` resolves to the `zenoh`
            // coordinate exactly as `row_coord` does.
            crate::fixtures::lane::require_west_leaf_in_lane(
                "build-logging-smoke",
                "packages/testing/nros-tests/bins/logging-smoke-zephyr-native-sim",
            )?;
            require_prebuilt_binary_fresh(&binary)
        })
        .map(|p| p.as_path())
}

/// Build the qemu-wcet-bench example and return its path (cached)
pub fn build_qemu_wcet_bench() -> TestResult<&'static Path> {
    QEMU_WCET_BENCH_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-rs-wcet-bench"))
        .map(|p| p.as_path())
}

/// Build the qemu-lan9118 example and return its path (cached)
pub fn build_qemu_lan9118() -> TestResult<&'static Path> {
    QEMU_LAN9118_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-rs-lan9118"))
        .map(|p| p.as_path())
}

/// rstest fixture that provides the native-rs-talker binary path
#[rstest::fixture]
pub fn talker_binary() -> PathBuf {
    build_native_talker()
        .expect("Failed to build native-rs-talker")
        .to_path_buf()
}

/// rstest fixture that provides the native-rs-listener binary path
#[rstest::fixture]
pub fn listener_binary() -> PathBuf {
    build_native_listener()
        .expect("Failed to build native-rs-listener")
        .to_path_buf()
}

/// rstest fixture that provides the native-rs-lifecycle-node binary path
#[rstest::fixture]
pub fn lifecycle_node_binary() -> PathBuf {
    build_native_lifecycle_node()
        .expect("Failed to build native-rs-lifecycle-node")
        .to_path_buf()
}

/// The peer-mode talker — issue 0711.
///
/// `ZPICO_MULTICAST_TRANSPORT=1` compiles the multicast transport + scouting
/// that issue 0682 turned off for size, so `ZPICO_PEER_MODE_SUPPORTED` is true
/// and the session layer stops refusing peer mode up front. The default native
/// talker is built WITHOUT it, which is why `test_peer_mode_communication`
/// could only ever skip: the binaries it spawns are prebuilt, and no env
/// exported around the test crate can change what they were compiled with.
///
/// Row-selected rather than path-selected (issue 0517): "the talker row built
/// with this env", not "the talker's peer dir". The env is part of the group
/// signature, so the pair lands in its own artifact root and the default native
/// tree keeps its footprint.
pub fn build_native_talker_peer() -> TestResult<&'static Path> {
    NATIVE_TALKER_PEER_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_row(
                "examples/native/rust/talker",
                &crate::fixtures::groups::FixtureVariant::plain()
                    .with_env(&[("ZPICO_MULTICAST_TRANSPORT", "1")]),
            )?;
            let rel = PathBuf::from(format!("{}/talker", cargo_target_profile_dir()));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// The peer-mode listener — issue 0711. See [`build_native_talker_peer`].
pub fn build_native_listener_peer() -> TestResult<&'static Path> {
    NATIVE_LISTENER_PEER_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_row(
                "examples/native/rust/listener",
                &crate::fixtures::groups::FixtureVariant::plain()
                    .with_env(&[("ZPICO_MULTICAST_TRANSPORT", "1")]),
            )?;
            let rel = PathBuf::from(format!("{}/listener", cargo_target_profile_dir()));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// Build native-rs-talker with link-tls feature (cached)
///
/// Uses a separate `target-tls` directory to avoid overwriting the
/// standard talker binary that other parallel test processes use.
pub fn build_native_talker_tls() -> TestResult<&'static Path> {
    NATIVE_TALKER_TLS_BINARY
        .get_or_try_init(|| {
            // issue 0517 — "the talker row built with `link-tls`", not
            // "the talker's target-tls dir".
            let row = crate::fixtures::groups::select_row(
                "examples/native/rust/talker",
                &crate::fixtures::groups::FixtureVariant::features(&["link-tls"]),
            )?;
            let rel = PathBuf::from(format!("{}/talker", cargo_target_profile_dir()));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// Build native-rs-listener with link-tls feature (cached)
///
/// Uses a separate `target-tls` directory to avoid overwriting the
/// standard listener binary that other parallel test processes use.
pub fn build_native_listener_tls() -> TestResult<&'static Path> {
    NATIVE_LISTENER_TLS_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_row(
                "examples/native/rust/listener",
                &crate::fixtures::groups::FixtureVariant::features(&["link-tls"]),
            )?;
            let rel = PathBuf::from(format!("{}/listener", cargo_target_profile_dir()));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the native-rs-talker binary path (with link-tls)
#[rstest::fixture]
pub fn talker_tls_binary() -> PathBuf {
    build_native_talker_tls()
        .expect("Failed to build native-rs-talker with link-tls")
        .to_path_buf()
}

/// rstest fixture that provides the native-rs-listener binary path (with link-tls)
#[rstest::fixture]
pub fn listener_tls_binary() -> PathBuf {
    build_native_listener_tls()
        .expect("Failed to build native-rs-listener with link-tls")
        .to_path_buf()
}

/// Build native-rs-action-server (cached)
pub fn build_native_action_server() -> TestResult<&'static Path> {
    NATIVE_ACTION_SERVER_BINARY
        .get_or_try_init(|| {
            build_example_rmw("native/rust/action-server", "action-server", Rmw::Zenoh)
        })
        .map(|p| p.as_path())
}

/// Build native-rs-action-client (cached)
pub fn build_native_action_client() -> TestResult<&'static Path> {
    NATIVE_ACTION_CLIENT_BINARY
        .get_or_try_init(|| {
            build_example_rmw("native/rust/action-client", "action-client", Rmw::Zenoh)
        })
        .map(|p| p.as_path())
}

/// Resolve the prebuilt `safety-chatter-talker` fixture (cached).
///
/// phase-277 W3.a: was `examples/native/rust/talker --features safety-e2e`
/// in `target-safety/`; now a dedicated bin
/// (`packages/testing/nros-tests/bins/safety-chatter-talker`) with the
/// safety features baked, so the example manifest carries no test-only
/// features. Same behavior: Int32 on /chatter every 1 s + backend CRC attach.
pub fn build_native_talker_safety() -> TestResult<&'static Path> {
    NATIVE_TALKER_SAFETY_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_sole_row(
                "packages/testing/nros-tests/bins/safety-chatter-talker",
            )?;
            let profile = cargo_target_profile_dir();
            let rel = PathBuf::from(format!("{profile}/safety-chatter-talker"));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// Resolve the prebuilt `header-chatter-talker` fixture (cached): it also
/// publishes a NESTED `std_msgs/Header` on /header, for the declarative
/// bridge's non-flat forwarding e2e. phase-277 W3.a: was the talker `header`
/// feature build in `target-header/`; now a dedicated bin
/// (`packages/testing/nros-tests/bins/header-chatter-talker`).
pub fn build_native_talker_header() -> TestResult<&'static Path> {
    NATIVE_TALKER_HEADER_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_sole_row(
                "packages/testing/nros-tests/bins/header-chatter-talker",
            )?;
            let profile = cargo_target_profile_dir();
            let rel = PathBuf::from(format!("{profile}/header-chatter-talker"));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// Resolve one prebuilt `contract-monitor` parity-fixture bin (cached). All
/// three binaries come from the single
/// `packages/testing/nros-tests/bins/contract-monitor` crate, so building any
/// one builds all three.
fn build_contract_monitor_bin(
    cache: &'static OnceCell<PathBuf>,
    bin_name: &str,
) -> TestResult<&'static Path> {
    cache
        .get_or_try_init(|| {
            let root = project_root();
            let dir = root.join("packages/testing/nros-tests/bins/contract-monitor");
            let profile = cargo_target_profile_dir();
            let binary = dir.join(format!("target/{profile}/{bin_name}"));
            require_prebuilt_binary_fresh(&binary)
        })
        .map(|p| p.as_path())
}

/// RFC-0052 / phase-296 W3b.4 — the rate-contract publisher bin.
pub fn build_contract_monitor_pub() -> TestResult<&'static Path> {
    build_contract_monitor_bin(&CONTRACT_MONITOR_PUB_BINARY, "contract-monitor-pub")
}

/// RFC-0052 / phase-296 W3b.5 — the age-contract subscriber bin.
pub fn build_contract_monitor_sub() -> TestResult<&'static Path> {
    build_contract_monitor_bin(&CONTRACT_MONITOR_SUB_BINARY, "contract-monitor-sub")
}

/// RFC-0052 / phase-296 W3b.4/.5 — the `/diagnostics` observer bin.
pub fn build_contract_monitor_diagsink() -> TestResult<&'static Path> {
    build_contract_monitor_bin(
        &CONTRACT_MONITOR_DIAGSINK_BINARY,
        "contract-monitor-diagsink",
    )
}

/// Resolve the prebuilt `safety-chatter-listener` fixture (cached).
///
/// phase-277 W3.a: was the `safety-e2e`-gated second `main` of
/// `examples/native/rust/listener` (built into `target-safety/`); now a
/// dedicated bin (`packages/testing/nros-tests/bins/safety-chatter-listener`)
/// so the example keeps a single cfg-free `main`. Same behavior: `.safety()`
/// subscription on /chatter logging `[SAFETY] seq_gap=.. dup=.. crc=..`.
pub fn build_native_listener_safety() -> TestResult<&'static Path> {
    NATIVE_LISTENER_SAFETY_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_sole_row(
                "packages/testing/nros-tests/bins/safety-chatter-listener",
            )?;
            let profile = cargo_target_profile_dir();
            let rel = PathBuf::from(format!("{profile}/safety-chatter-listener"));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the native-rs-talker binary path (with safety-e2e)
#[rstest::fixture]
pub fn talker_safety_binary() -> PathBuf {
    build_native_talker_safety()
        .expect("Failed to build native-rs-talker with safety-e2e")
        .to_path_buf()
}

/// rstest fixture that provides the native-rs-listener binary path (with safety-e2e)
#[rstest::fixture]
pub fn listener_safety_binary() -> PathBuf {
    build_native_listener_safety()
        .expect("Failed to build native-rs-listener with safety-e2e")
        .to_path_buf()
}

/// Build native-rs-listener with unstable-zenoh-api feature (cached)
///
/// Uses a separate `target-zero-copy` directory to avoid overwriting the
/// standard/safety listener binaries that other parallel test processes use.
/// issue 0441 — the receive-side MessageInfo observer, built PLAIN.
///
/// Pairs with [`build_message_info_observer_zero_copy`]: the two differ in
/// exactly one feature, so "both print the same `seq=`/`gid=` line" is a
/// statement about the zero-copy trampoline rather than about two unrelated
/// binaries. Replaces the zero-copy build of the listener EXAMPLE, whose output
/// was byte-identical to the plain one — the example cannot reach `MessageInfo`
/// at all (`CallbackCtx` has no accessor; the `FnMut(&M, Option<&MessageInfo>)`
/// shape lives on the executor's `.message_info()` builder).
pub fn build_message_info_observer() -> TestResult<&'static Path> {
    MESSAGE_INFO_OBSERVER_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_row(
                "packages/testing/nros-tests/bins/message-info-observer",
                &crate::fixtures::groups::FixtureVariant::plain(),
            )?;
            let rel = PathBuf::from(format!(
                "{}/message-info-observer",
                cargo_target_profile_dir()
            ));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// issue 0441 — the same observer built with `unstable-zenoh-api` (the
/// zero-copy receive path).
pub fn build_message_info_observer_zero_copy() -> TestResult<&'static Path> {
    MESSAGE_INFO_OBSERVER_ZERO_COPY_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_row(
                "packages/testing/nros-tests/bins/message-info-observer",
                &crate::fixtures::groups::FixtureVariant::features(&["unstable-zenoh-api"]),
            )?;
            let rel = PathBuf::from(format!(
                "{}/message-info-observer",
                cargo_target_profile_dir()
            ));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the native-rs-action-server binary path
#[rstest::fixture]
pub fn action_server_binary() -> PathBuf {
    build_native_action_server()
        .expect("Failed to build native-rs-action-server")
        .to_path_buf()
}

/// rstest fixture that provides the native-rs-action-client binary path
#[rstest::fixture]
pub fn action_client_binary() -> PathBuf {
    build_native_action_client()
        .expect("Failed to build native-rs-action-client")
        .to_path_buf()
}

/// Build native-rs-service-server (cached)
pub fn build_native_service_server() -> TestResult<&'static Path> {
    NATIVE_SERVICE_SERVER_BINARY
        .get_or_try_init(|| {
            build_example_rmw("native/rust/service-server", "service-server", Rmw::Zenoh)
        })
        .map(|p| p.as_path())
}

static NATIVE_RS_ASYNC_SERVICE_CLIENT_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// #102 H3 — build the `native/rust/service-client-async` example (cached). The
/// async/tokio service-client variant (`spin_async` + `.await` on the response
/// Promise); paired with `build_native_service_server` in the roundtrip e2e.
pub fn build_native_async_service_client() -> TestResult<&'static Path> {
    NATIVE_RS_ASYNC_SERVICE_CLIENT_BINARY
        .get_or_try_init(|| {
            build_example(
                "native/rust/service-client-async",
                "native-rs-async-service-client",
                None,
                None,
            )
        })
        .map(|p| p.as_path())
}

static NATIVE_RS_ASYNC_ACTION_CLIENT_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// #102 H3 — build the `native/rust/action-client-async` example (cached). The
/// async/tokio action-client variant (background `spin_async` + `.await` on the
/// goal/result Promises + `StreamExt` feedback); paired with
/// `build_native_action_server` in the roundtrip e2e.
pub fn build_native_async_action_client() -> TestResult<&'static Path> {
    NATIVE_RS_ASYNC_ACTION_CLIENT_BINARY
        .get_or_try_init(|| {
            build_example(
                "native/rust/action-client-async",
                "native-rs-async-action-client",
                None,
                None,
            )
        })
        .map(|p| p.as_path())
}

/// Build native-rs-service-client (cached)
pub fn build_native_service_client() -> TestResult<&'static Path> {
    NATIVE_SERVICE_CLIENT_BINARY
        .get_or_try_init(|| {
            build_example_rmw("native/rust/service-client", "service-client", Rmw::Zenoh)
        })
        .map(|p| p.as_path())
}

/// Build native-rs-service-client-callback (cached, RFC-0041 / Phase 239)
pub fn build_native_service_client_callback() -> TestResult<&'static Path> {
    NATIVE_SERVICE_CLIENT_CALLBACK_BINARY
        .get_or_try_init(|| {
            build_example_rmw(
                "native/rust/service-client-callback",
                "service-client-callback",
                Rmw::Zenoh,
            )
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the native-rs-service-server binary path
#[rstest::fixture]
pub fn service_server_binary() -> PathBuf {
    build_native_service_server()
        .expect("Failed to build native-rs-service-server")
        .to_path_buf()
}

/// rstest fixture that provides the native-rs-service-client binary path
#[rstest::fixture]
pub fn service_client_binary() -> PathBuf {
    build_native_service_client()
        .expect("Failed to build native-rs-service-client")
        .to_path_buf()
}

/// Build native-rs-custom-msg (cached)
pub fn build_native_custom_msg() -> TestResult<&'static Path> {
    NATIVE_CUSTOM_MSG_BINARY
        .get_or_try_init(|| build_example("native/rust/custom-msg", "custom_msg", None, None))
        .map(|p| p.as_path())
}

/// Build native-rs-custom-msg (uncached, for serialization tests)
pub fn build_native_custom_msg_no_zenoh() -> TestResult<PathBuf> {
    build_example("native/rust/custom-msg", "custom_msg", None, None)
}

/// rstest fixture that provides the native-rs-custom-msg binary path (with zenoh)
#[rstest::fixture]
pub fn custom_msg_binary() -> PathBuf {
    build_native_custom_msg()
        .expect("Failed to build native-rs-custom-msg")
        .to_path_buf()
}

/// Build qemu-bsp-talker (cached)
pub fn build_qemu_bsp_talker() -> TestResult<&'static Path> {
    QEMU_BSP_TALKER_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-bsp-talker"))
        .map(|p| p.as_path())
}

/// Build qemu-bsp-listener (cached)
pub fn build_qemu_bsp_listener() -> TestResult<&'static Path> {
    QEMU_BSP_LISTENER_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-bsp-listener"))
        .map(|p| p.as_path())
}

/// Phase 244.D1 — resolve the prebuilt bare-metal `nros::main!()` BoardEntry
/// E2E fixture (`qemu-baremetal-main-e2e`, source at
/// `packages/testing/nros-tests/bins/qemu-baremetal-main-e2e/`) from the
/// shared qemu-arm-baremetal fixture target dir. No caching needed — it only stats a path. Absent →
/// `BuildFailed` (the test skips, prompting `just qemu-baremetal build-fixtures`).
pub fn qemu_baremetal_main_e2e_binary() -> TestResult<PathBuf> {
    require_qemu_baremetal_fixture("qemu-baremetal-main-e2e")
}

// Phase 169.4 — bare-metal DDS fixture builders deleted with the
// Rust DDS examples (Phase 169.2).
/// rstest fixture that provides the qemu-bsp-talker binary path
#[rstest::fixture]
pub fn qemu_bsp_talker_binary() -> PathBuf {
    build_qemu_bsp_talker()
        .expect("Failed to build qemu-bsp-talker")
        .to_path_buf()
}

/// rstest fixture that provides the qemu-bsp-listener binary path
#[rstest::fixture]
pub fn qemu_bsp_listener_binary() -> PathBuf {
    build_qemu_bsp_listener()
        .expect("Failed to build qemu-bsp-listener")
        .to_path_buf()
}

// ═══════════════════════════════════════════════════════════════════════════
// Serial Example Builders (QEMU bare-metal, cross-compiled)
// ═══════════════════════════════════════════════════════════════════════════

/// Build qemu-serial-talker (cached)
pub fn build_qemu_serial_talker() -> TestResult<&'static Path> {
    QEMU_SERIAL_TALKER_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-serial-talker"))
        .map(|p| p.as_path())
}

/// rstest fixture that provides the qemu-serial-talker binary path
#[rstest::fixture]
pub fn qemu_serial_talker_binary() -> PathBuf {
    build_qemu_serial_talker()
        .expect("Failed to build qemu-serial-talker")
        .to_path_buf()
}

/// Build qemu-serial-listener (cached)
pub fn build_qemu_serial_listener() -> TestResult<&'static Path> {
    QEMU_SERIAL_LISTENER_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-serial-listener"))
        .map(|p| p.as_path())
}

/// rstest fixture that provides the qemu-serial-listener binary path
#[rstest::fixture]
pub fn qemu_serial_listener_binary() -> PathBuf {
    build_qemu_serial_listener()
        .expect("Failed to build qemu-serial-listener")
        .to_path_buf()
}

/// Phase 207 — build the bare-metal XRCE talker (cached). Wraps the same
/// `build_example` path the serial-talker uses; the prebuilt at
/// `target/.../<profile>/qemu-talker-xrce` is checked, not rebuilt
/// (`just qemu build-fixtures` / `cargo build --profile <p>` is the build
/// step, this is the resolve step).
pub fn build_qemu_talker_xrce() -> TestResult<&'static Path> {
    QEMU_TALKER_XRCE_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-talker-xrce"))
        .map(|p| p.as_path())
}

/// rstest fixture that provides the qemu-talker-xrce binary path.
#[rstest::fixture]
pub fn qemu_talker_xrce_binary() -> PathBuf {
    build_qemu_talker_xrce()
        .expect("Failed to build qemu-talker-xrce")
        .to_path_buf()
}

// ═══════════════════════════════════════════════════════════════════════════
// RTIC Example Builders (native host)
//
// phase-337 W7.a — the cross-compiled STM32F4 half of this block went with the
// board. Its six accessors had ZERO callers once `stm32f4_rtic_main_macro.rs`
// was deleted, which is the same "resolver nobody calls" shape as #222/#328.
// ═══════════════════════════════════════════════════════════════════════════

/// Cached path to the native rtic-talker binary
static NATIVE_RTIC_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native rtic-listener binary
static NATIVE_RTIC_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native rtic-service-server binary
static NATIVE_RTIC_SERVICE_SERVER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native rtic-service-client binary
static NATIVE_RTIC_SERVICE_CLIENT_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native rtic-action-server binary
static NATIVE_RTIC_ACTION_SERVER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the native rtic-action-client binary
static NATIVE_RTIC_ACTION_CLIENT_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Build native rtic-talker (cached)
pub fn build_native_rtic_talker() -> TestResult<&'static Path> {
    NATIVE_RTIC_TALKER_BINARY
        .get_or_try_init(|| build_example("native/rust/talker-rtic", "rtic-talker", None, None))
        .map(|p| p.as_path())
}

/// Build native rtic-listener (cached)
pub fn build_native_rtic_listener() -> TestResult<&'static Path> {
    NATIVE_RTIC_LISTENER_BINARY
        .get_or_try_init(|| build_example("native/rust/listener-rtic", "rtic-listener", None, None))
        .map(|p| p.as_path())
}

/// Build native rtic-service-server (cached)
pub fn build_native_rtic_service_server() -> TestResult<&'static Path> {
    NATIVE_RTIC_SERVICE_SERVER_BINARY
        .get_or_try_init(|| {
            build_example(
                "native/rust/service-server-rtic",
                "rtic-service-server",
                None,
                None,
            )
        })
        .map(|p| p.as_path())
}

/// Build native rtic-service-client (cached)
pub fn build_native_rtic_service_client() -> TestResult<&'static Path> {
    NATIVE_RTIC_SERVICE_CLIENT_BINARY
        .get_or_try_init(|| {
            build_example(
                "native/rust/service-client-rtic",
                "rtic-service-client",
                None,
                None,
            )
        })
        .map(|p| p.as_path())
}

/// Build native rtic-action-server (cached)
pub fn build_native_rtic_action_server() -> TestResult<&'static Path> {
    NATIVE_RTIC_ACTION_SERVER_BINARY
        .get_or_try_init(|| {
            build_example(
                "native/rust/action-server-rtic",
                "rtic-action-server",
                None,
                None,
            )
        })
        .map(|p| p.as_path())
}

/// Build native rtic-action-client (cached)
pub fn build_native_rtic_action_client() -> TestResult<&'static Path> {
    NATIVE_RTIC_ACTION_CLIENT_BINARY
        .get_or_try_init(|| {
            build_example(
                "native/rust/action-client-rtic",
                "rtic-action-client",
                None,
                None,
            )
        })
        .map(|p| p.as_path())
}

// ═══════════════════════════════════════════════════════════════════════════
// XRCE-DDS Example Builders
// ═══════════════════════════════════════════════════════════════════════════

/// Build the xrce-talker example binary (cached).
pub fn build_xrce_talker() -> TestResult<&'static Path> {
    XRCE_TALKER_BINARY
        .get_or_try_init(|| build_example_rmw("native/rust/talker", "talker", Rmw::Xrce))
        .map(|p| p.as_path())
}

/// Build the xrce-listener example binary (cached).
pub fn build_xrce_listener() -> TestResult<&'static Path> {
    XRCE_LISTENER_BINARY
        .get_or_try_init(|| build_example_rmw("native/rust/listener", "listener", Rmw::Xrce))
        .map(|p| p.as_path())
}

/// rstest fixture that provides the xrce-talker binary path.
#[rstest::fixture]
pub fn xrce_talker_binary() -> PathBuf {
    build_xrce_talker()
        .expect("Failed to build xrce-talker")
        .to_path_buf()
}

/// rstest fixture that provides the xrce-listener binary path.
#[rstest::fixture]
pub fn xrce_listener_binary() -> PathBuf {
    build_xrce_listener()
        .expect("Failed to build xrce-listener")
        .to_path_buf()
}

/// Resolve the prebuilt px4-stub example binary (Phase 233.4). Built by
/// `just px4 build-fixtures` to `examples/px4/rust/companion/px4-stub/target-xrce/`.
pub fn build_px4_stub() -> TestResult<&'static Path> {
    PX4_STUB_BINARY
        .get_or_try_init(|| {
            let dir = "px4/rust/companion/px4-stub";
            crate::fixtures::lane::require_coord_in_lane(&px4_companion_coord(), dir)?;
            build_example_rmw(dir, "px4-stub", Rmw::Xrce)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the px4-stub binary path.
#[rstest::fixture]
pub fn px4_stub_binary() -> PathBuf {
    build_px4_stub()
        .expect("Failed to build px4-stub")
        .to_path_buf()
}

/// The coordinate of the px4 companion leaves — issue 0658 follow-up.
///
/// `examples/px4/rust/companion/*` has NO `[[fixture]]` row: it is built by
/// `just px4 build-fixtures`, in its own lane, behind its own SDK
/// prerequisites. `attribute_path` therefore cannot place it, and the lane
/// machinery's rule for an unattributable path is deliberately "never skip" —
/// so under a narrowed lane these leaves reported STALE (a hard failure)
/// instead of "not in this lane".
///
/// That is the right rule and the wrong outcome. `require_coord_in_lane` exists
/// for exactly this case — "a resolver that selected its row by configuration
/// knows the coordinate outright" — so state the coordinate here rather than
/// relaxing attribution or inventing a manifest row for a lane that does not
/// build from the manifest.
///
/// ONE spelling for both leaves: two accessors drifting apart is how the
/// `[SKIPPED]` literal reached five call sites (issue 0658).
fn px4_companion_coord() -> crate::fixtures::lane::Coord {
    (
        "px4".to_string(),
        "rust".to_string(),
        Rmw::Xrce.coord_token().to_string(),
    )
}

/// Resolve the prebuilt px4 offboard-companion example binary (Phase 233.4).
pub fn build_px4_companion() -> TestResult<&'static Path> {
    PX4_COMPANION_BINARY
        .get_or_try_init(|| {
            let dir = "px4/rust/companion/offboard-companion";
            crate::fixtures::lane::require_coord_in_lane(&px4_companion_coord(), dir)?;
            build_example_rmw(dir, "offboard-companion", Rmw::Xrce)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the px4 offboard-companion binary path.
#[rstest::fixture]
pub fn px4_companion_binary() -> PathBuf {
    build_px4_companion()
        .expect("Failed to build px4 offboard-companion")
        .to_path_buf()
}

/// Build the xrce-service-server example binary (cached).
pub fn build_xrce_service_server() -> TestResult<&'static Path> {
    XRCE_SERVICE_SERVER_BINARY
        .get_or_try_init(|| {
            build_example_rmw("native/rust/service-server", "service-server", Rmw::Xrce)
        })
        .map(|p| p.as_path())
}

/// Build the xrce-service-client example binary (cached).
pub fn build_xrce_service_client() -> TestResult<&'static Path> {
    XRCE_SERVICE_CLIENT_BINARY
        .get_or_try_init(|| {
            build_example_rmw("native/rust/service-client", "service-client", Rmw::Xrce)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the xrce-service-server binary path.
#[rstest::fixture]
pub fn xrce_service_server_binary() -> PathBuf {
    build_xrce_service_server()
        .expect("Failed to build xrce-service-server")
        .to_path_buf()
}

/// rstest fixture that provides the xrce-service-client binary path.
#[rstest::fixture]
pub fn xrce_service_client_binary() -> PathBuf {
    build_xrce_service_client()
        .expect("Failed to build xrce-service-client")
        .to_path_buf()
}

/// Build the xrce-action-server example binary (cached).
pub fn build_xrce_action_server() -> TestResult<&'static Path> {
    XRCE_ACTION_SERVER_BINARY
        .get_or_try_init(|| {
            build_example_rmw("native/rust/action-server", "action-server", Rmw::Xrce)
        })
        .map(|p| p.as_path())
}

/// Build the xrce-action-client example binary (cached).
pub fn build_xrce_action_client() -> TestResult<&'static Path> {
    XRCE_ACTION_CLIENT_BINARY
        .get_or_try_init(|| {
            build_example_rmw("native/rust/action-client", "action-client", Rmw::Xrce)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the xrce-action-server binary path.
#[rstest::fixture]
pub fn xrce_action_server_binary() -> PathBuf {
    build_xrce_action_server()
        .expect("Failed to build xrce-action-server")
        .to_path_buf()
}

/// rstest fixture that provides the xrce-action-client binary path.
#[rstest::fixture]
pub fn xrce_action_client_binary() -> PathBuf {
    build_xrce_action_client()
        .expect("Failed to build xrce-action-client")
        .to_path_buf()
}

/// Build the xrce-serial-talker example binary (cached).
pub fn build_xrce_serial_talker() -> TestResult<&'static Path> {
    XRCE_SERIAL_TALKER_BINARY
        .get_or_try_init(|| {
            build_example(
                "native/rust/serial-talker",
                "xrce-serial-talker",
                None,
                None,
            )
        })
        .map(|p| p.as_path())
}

/// Build the xrce-serial-listener example binary (cached).
pub fn build_xrce_serial_listener() -> TestResult<&'static Path> {
    XRCE_SERIAL_LISTENER_BINARY
        .get_or_try_init(|| {
            build_example(
                "native/rust/serial-listener",
                "xrce-serial-listener",
                None,
                None,
            )
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the xrce-serial-talker binary path.
#[rstest::fixture]
pub fn xrce_serial_talker_binary() -> PathBuf {
    build_xrce_serial_talker()
        .expect("Failed to build xrce-serial-talker")
        .to_path_buf()
}

/// rstest fixture that provides the xrce-serial-listener binary path.
#[rstest::fixture]
pub fn xrce_serial_listener_binary() -> PathBuf {
    build_xrce_serial_listener()
        .expect("Failed to build xrce-serial-listener")
        .to_path_buf()
}

/// Build the xrce-large-msg-test example binary (cached).
pub fn build_xrce_large_msg_test() -> TestResult<&'static Path> {
    XRCE_LARGE_MSG_TEST_BINARY
        .get_or_try_init(|| {
            build_test_fixture("nros-bench/large-msg-xrce", "xrce-large-msg-test", None)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the xrce-large-msg-test binary path.
#[cfg(test)]
mod fixture_absence_class_tests {
    use super::*;

    /// Issue 0584 part 2 — an absent fixture is a SKIP only when nothing
    /// promised it. The two arms are the whole point of the change, so both are
    /// asserted; testing only the panic would let the ungated arm rot into a
    /// hard failure and break every developer's bare `cargo nextest`.
    ///
    /// `require_prebuilt_binary_checks` is the shared resolver — the sibling
    /// `build_*` helpers all funnel through it, which is why the panic lives
    /// there and not at the ~500 call sites that launder its `Err` into
    /// `[SKIPPED] … not prebuilt`.
    #[test]
    fn ungated_absence_is_a_recoverable_error() {
        // SAFETY: nextest runs each test in its own process.
        unsafe {
            std::env::remove_var("NROS_TEST_SCOPE");
            std::env::remove_var("NROS_TEST_COORDS");
        }
        let missing = std::path::Path::new("/nonexistent/nros-fixture-absence-probe");
        let got = require_prebuilt_binary_checks(missing);
        assert!(
            matches!(&got, Err(crate::TestError::BuildFailed(m)) if m.contains("not prebuilt")),
            "ungated: expected a recoverable BuildFailed, got {got:?}"
        );
    }

    #[test]
    #[should_panic(expected = "MISSING for an in-lane coordinate")]
    fn gated_absence_is_a_hard_failure() {
        // SAFETY: nextest runs each test in its own process.
        unsafe { std::env::set_var("NROS_TEST_SCOPE", "native") }
        let missing = std::path::Path::new("/nonexistent/nros-fixture-absence-probe");
        let _ = require_prebuilt_binary_checks(missing);
    }
}

/// Phase 150.F — "binary not prebuilt" is an environment/setup
/// condition (user didn't run `just build-test-fixtures`), not a
/// test-logic failure. Surface it via `nros_tests::skip!` so
/// `_count-real-failures` filters it out and the ci summary
/// doesn't flag it as a real failure. Any OTHER build error (e.g.
/// the fixture crate genuinely failing to compile) panics
/// normally and counts as a real failure.
#[rstest::fixture]
pub fn xrce_large_msg_test_binary() -> PathBuf {
    match build_xrce_large_msg_test() {
        Ok(p) => p.to_path_buf(),
        Err(crate::TestError::BuildFailed(msg)) if msg.contains("not prebuilt") => {
            nros_tests_skip(msg)
        }
        Err(e) => panic!("Failed to build xrce-large-msg-test: {e:?}"),
    }
}

/// Helper that panics with the `[SKIPPED]` prefix recognised by
/// `justfile::_count-real-failures`. Kept local to this module
/// so the macro's lexical scope doesn't need to escape.
fn nros_tests_skip(msg: String) -> ! {
    panic!("[SKIPPED] {msg}")
}

// ═══════════════════════════════════════════════════════════════════════════
// Stress Test & Large Message Builders
// ═══════════════════════════════════════════════════════════════════════════

/// Build the zenoh-stress-test binary (cached).
///
/// The PLAIN row of a two-row leaf; `build_test_fixture` selects it by row
/// rather than by path (issue 0517 step 3 — see the guard there).
pub fn build_zenoh_stress_test() -> TestResult<&'static Path> {
    ZENOH_STRESS_TEST_BINARY
        .get_or_try_init(|| {
            build_test_fixture("nros-bench/stress-zenoh", "zenoh-stress-test", None)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the zenoh-stress-test binary path.
/// See `xrce_large_msg_test_binary` for the not-prebuilt → skip
/// rationale (Phase 150.F).
#[rstest::fixture]
pub fn zenoh_stress_test_binary() -> PathBuf {
    match build_zenoh_stress_test() {
        Ok(p) => p.to_path_buf(),
        Err(crate::TestError::BuildFailed(msg)) if msg.contains("not prebuilt") => {
            nros_tests_skip(msg)
        }
        Err(e) => panic!("Failed to build zenoh-stress-test: {e:?}"),
    }
}

/// Build the zenoh-stress-test binary with large subscriber buffer (8192B, cached).
///
/// Uses `ZPICO_SUBSCRIBER_BUFFER_SIZE=8192`. The separation from its plain
/// sibling is the row's own cargo group, not the `target-large-buf` directory
/// this comment used to name — issue 0517 step 3 deleted that column.
pub fn build_zenoh_stress_test_large_buf() -> TestResult<&'static Path> {
    ZENOH_STRESS_TEST_LARGE_BUF_BINARY
        .get_or_try_init(|| {
            // The one row the selector needs `env` for: it is otherwise
            // identical to its plain sibling (issue 0517).
            let row = crate::fixtures::groups::select_row(
                "packages/testing/nros-bench/stress-zenoh",
                &crate::fixtures::groups::FixtureVariant::plain()
                    .with_env(&[("ZPICO_SUBSCRIBER_BUFFER_SIZE", "8192")]),
            )?;
            let rel = PathBuf::from(format!("{}/zenoh-stress-test", cargo_target_profile_dir()));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the zenoh-stress-test binary path (large subscriber buffer).
#[rstest::fixture]
pub fn zenoh_stress_test_large_buf_binary() -> PathBuf {
    build_zenoh_stress_test_large_buf()
        .expect("Failed to build zenoh-stress-test (large-buf)")
        .to_path_buf()
}

/// Build the xrce-stress-test binary (cached).
pub fn build_xrce_stress_test() -> TestResult<&'static Path> {
    XRCE_STRESS_TEST_BINARY
        .get_or_try_init(|| build_test_fixture("nros-bench/stress-xrce", "xrce-stress-test", None))
        .map(|p| p.as_path())
}

/// rstest fixture that provides the xrce-stress-test binary path.
/// See `xrce_large_msg_test_binary` for the not-prebuilt → skip
/// rationale (Phase 150.F).
#[rstest::fixture]
pub fn xrce_stress_test_binary() -> PathBuf {
    match build_xrce_stress_test() {
        Ok(p) => p.to_path_buf(),
        Err(crate::TestError::BuildFailed(msg)) if msg.contains("not prebuilt") => {
            nros_tests_skip(msg)
        }
        Err(e) => panic!("Failed to build xrce-stress-test: {e:?}"),
    }
}

/// Build the xrce-stress-test binary with a large receive ring (8192B, cached).
///
/// phase-384 W2. Uses `NROS_XRCE_BUFFER_SIZE=8192`, mirroring the
/// `ZPICO_SUBSCRIBER_BUFFER_SIZE` row for stress-zenoh. The XRCE ring entry is
/// a fixed `uint8_t data[XRCE_BUFFER_SIZE]` (`nros-rmw-xrce/src/internal.h`), so
/// the caller's own `RX_BUF` cannot raise it — this env knob is the only lever,
/// and this row is what proves it works.
pub fn build_xrce_stress_test_large_buf() -> TestResult<&'static Path> {
    XRCE_STRESS_TEST_LARGE_BUF_BINARY
        .get_or_try_init(|| {
            let row = crate::fixtures::groups::select_row(
                "packages/testing/nros-bench/stress-xrce",
                &crate::fixtures::groups::FixtureVariant::plain()
                    .with_env(&[("NROS_XRCE_BUFFER_SIZE", "8192")]),
            )?;
            let rel = PathBuf::from(format!("{}/xrce-stress-test", cargo_target_profile_dir()));
            require_prebuilt_row_binary_fresh(row, &rel)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the xrce-stress-test binary path (large receive ring).
#[rstest::fixture]
pub fn xrce_stress_test_large_buf_binary() -> PathBuf {
    match build_xrce_stress_test_large_buf() {
        Ok(p) => p.to_path_buf(),
        Err(crate::TestError::BuildFailed(msg)) if msg.contains("not prebuilt") => {
            nros_tests_skip(msg)
        }
        Err(e) => panic!("Failed to build xrce-stress-test (large-buf): {e:?}"),
    }
}

/// Build qemu-bsp-large-msg-test (cached).
pub fn build_qemu_large_msg_test() -> TestResult<&'static Path> {
    QEMU_LARGE_MSG_TEST_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-bsp-large-msg-test"))
        .map(|p| p.as_path())
}

/// rstest fixture that provides the qemu-bsp-large-msg-test binary path.
#[rstest::fixture]
pub fn qemu_large_msg_test_binary() -> PathBuf {
    build_qemu_large_msg_test()
        .expect("Failed to build qemu-bsp-large-msg-test")
        .to_path_buf()
}

// ═══════════════════════════════════════════════════════════════════════════
// C Example Builders (CMake-based)
// ═══════════════════════════════════════════════════════════════════════════

/// Build c-talker example (cached)
pub fn build_c_talker() -> TestResult<&'static Path> {
    C_TALKER_BINARY
        .get_or_try_init(|| build_example_cmake_rmw("native/c/talker", "c_talker", Rmw::Zenoh))
        .map(|p| p.as_path())
}

/// Build c-listener example (cached)
pub fn build_c_listener() -> TestResult<&'static Path> {
    C_LISTENER_BINARY
        .get_or_try_init(|| build_example_cmake_rmw("native/c/listener", "c_listener", Rmw::Zenoh))
        .map(|p| p.as_path())
}

/// rstest fixture that provides the c-talker binary path
#[rstest::fixture]
pub fn c_talker_binary() -> PathBuf {
    build_c_talker()
        .expect("Failed to build c-talker")
        .to_path_buf()
}

/// rstest fixture that provides the c-listener binary path
#[rstest::fixture]
pub fn c_listener_binary() -> PathBuf {
    build_c_listener()
        .expect("Failed to build c-listener")
        .to_path_buf()
}

/// Build c-service-server example (cached)
pub fn build_c_service_server() -> TestResult<&'static Path> {
    C_SERVICE_SERVER_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw("native/c/service-server", "c_service_server", Rmw::Zenoh)
        })
        .map(|p| p.as_path())
}

/// Build c-service-client example (cached)
pub fn build_c_service_client() -> TestResult<&'static Path> {
    C_SERVICE_CLIENT_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw("native/c/service-client", "c_service_client", Rmw::Zenoh)
        })
        .map(|p| p.as_path())
}

/// Build c-service-client-callback example (cached, RFC-0041 / Phase 239)
pub fn build_c_service_client_callback() -> TestResult<&'static Path> {
    C_SERVICE_CLIENT_CALLBACK_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw(
                "native/c/service-client-callback",
                "c_service_client_callback",
                Rmw::Zenoh,
            )
        })
        .map(|p| p.as_path())
}

/// Build c-action-server example (cached)
pub fn build_c_action_server() -> TestResult<&'static Path> {
    C_ACTION_SERVER_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw("native/c/action-server", "c_action_server", Rmw::Zenoh)
        })
        .map(|p| p.as_path())
}

/// Build the raw-goal wire probe (cached).
///
/// Issue 0454 / phase-354 W3 — the only caller of
/// `nros_action_client_send_goal_raw`. Not an example: it lives under
/// `packages/testing/nros-tests/bins/`, so it resolves through
/// [`build_cmake_leaf_rmw`] rather than the `examples/`-rooted wrapper.
pub fn build_action_raw_goal_probe() -> TestResult<&'static Path> {
    ACTION_RAW_GOAL_PROBE_BINARY
        .get_or_try_init(|| {
            build_cmake_leaf_rmw(
                "packages/testing/nros-tests/bins/action-raw-goal-probe",
                "action_raw_goal_probe",
                Rmw::Zenoh,
            )
        })
        .map(|p| p.as_path())
}

/// Build c-action-client example (cached)
pub fn build_c_action_client() -> TestResult<&'static Path> {
    C_ACTION_CLIENT_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw("native/c/action-client", "c_action_client", Rmw::Zenoh)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the c-service-server binary path
#[rstest::fixture]
pub fn c_service_server_binary() -> PathBuf {
    build_c_service_server()
        .expect("Failed to build c-service-server")
        .to_path_buf()
}

/// rstest fixture that provides the c-service-client binary path
#[rstest::fixture]
pub fn c_service_client_binary() -> PathBuf {
    build_c_service_client()
        .expect("Failed to build c-service-client")
        .to_path_buf()
}

/// rstest fixture that provides the c-service-client-callback binary path
#[rstest::fixture]
pub fn c_service_client_callback_binary() -> PathBuf {
    build_c_service_client_callback()
        .expect("Failed to build c-service-client-callback")
        .to_path_buf()
}

/// rstest fixture that provides the c-action-server binary path
#[rstest::fixture]
pub fn c_action_server_binary() -> PathBuf {
    build_c_action_server()
        .expect("Failed to build c-action-server")
        .to_path_buf()
}

/// rstest fixture that provides the c-action-client binary path
#[rstest::fixture]
pub fn c_action_client_binary() -> PathBuf {
    build_c_action_client()
        .expect("Failed to build c-action-client")
        .to_path_buf()
}

// ═══════════════════════════════════════════════════════════════════════════
// C XRCE Example Builders (CMake-based, XRCE-DDS backend)
// ═══════════════════════════════════════════════════════════════════════════

/// Build c-xrce-talker example (cached)
pub fn build_c_xrce_talker() -> TestResult<&'static Path> {
    C_XRCE_TALKER_BINARY
        .get_or_try_init(|| build_example_cmake_rmw("native/c/talker", "c_talker", Rmw::Xrce))
        .map(|p| p.as_path())
}

/// Build c-xrce-listener example (cached)
pub fn build_c_xrce_listener() -> TestResult<&'static Path> {
    C_XRCE_LISTENER_BINARY
        .get_or_try_init(|| build_example_cmake_rmw("native/c/listener", "c_listener", Rmw::Xrce))
        .map(|p| p.as_path())
}

/// rstest fixture that provides the c-xrce-talker binary path
#[rstest::fixture]
pub fn c_xrce_talker_binary() -> PathBuf {
    build_c_xrce_talker()
        .expect("Failed to build c-xrce-talker")
        .to_path_buf()
}

/// rstest fixture that provides the c-xrce-listener binary path
#[rstest::fixture]
pub fn c_xrce_listener_binary() -> PathBuf {
    build_c_xrce_listener()
        .expect("Failed to build c-xrce-listener")
        .to_path_buf()
}

// ═══════════════════════════════════════════════════════════════════════════
// ESP32-C3 QEMU Example Builders (nightly toolchain)
// ═══════════════════════════════════════════════════════════════════════════

/// Build an ESP32-C3 QEMU example using the pinned nightly
///
/// ESP32 examples require a nightly toolchain with `-Zbuild-std`, so we
/// can't use the generic `build_example()` which uses stable `cargo build`.
/// The channel comes from `tools/rust-toolchain.toml` via [`pinned_nightly`].
fn build_esp32_qemu_example(name: &str, binary_name: &str) -> TestResult<PathBuf> {
    // Issue #181 — this used to `cargo build` IN-TEST (against the repo's
    // no-compilation-inside-tests rule) and then probe a HYPHENATED binary
    // name the crates stopped producing (their `[[bin]]` names are
    // underscored), so it failed "Binary not found after build" on every
    // host while the fixture sweep skipped the esp32 lane entirely. The
    // lane is now part of `build-test-fixtures` (`just esp32 build-fixtures`);
    // consume the prebuilt ELF like every other platform.
    let root = project_root();
    let example_dir = root.join(format!("examples/qemu-esp32-baremetal/rust/{}", name));

    if !example_dir.exists() {
        return Err(TestError::BuildFailed(format!(
            "ESP32 example directory not found: {}",
            example_dir.display()
        )));
    }

    let binary_path = example_dir.join(format!(
        "target/riscv32imc-unknown-none-elf/{}/{}",
        cargo_target_profile_dir(),
        binary_name
    ));
    require_prebuilt_binary_fresh(&binary_path)
}

/// Build esp32-qemu-talker (cached)
pub fn build_esp32_qemu_talker() -> TestResult<&'static Path> {
    ESP32_QEMU_TALKER_BINARY
        .get_or_try_init(|| build_esp32_qemu_example("talker", "esp32_qemu_talker"))
        .map(|p| p.as_path())
}

/// Build esp32-qemu-listener (cached)
pub fn build_esp32_qemu_listener() -> TestResult<&'static Path> {
    ESP32_QEMU_LISTENER_BINARY
        .get_or_try_init(|| build_esp32_qemu_example("listener", "esp32_qemu_listener"))
        .map(|p| p.as_path())
}

/// Resolve the PREBUILT ESP32-C3 QEMU workspace Entry ELF (Phase 225.O).
///
/// The workspace Entry (`examples/workspaces/rust/src/esp32_entry`) is
/// the ESP32 sibling of the native / FreeRTOS / ThreadX / Zephyr
/// workspace Entries: a SINGLE bare-metal binary that hosts the whole
/// launch-defined node set (talker + listener) in one image via
/// `nros::main!(launch = "demo_bringup:system.launch.xml")`. It is built
/// by the workspace-fixture lane
/// (`scripts/build/workspace-fixtures-build.sh esp32 rust`, run by
/// `just esp32 build-examples` / `build-fixtures`) into
/// `target-fixtures/esp32/riscv32imc-unknown-none-elf/<profile>/esp32_entry`,
/// NOT in-body — tests only run prebuilt workspace fixtures, mirroring
/// the Zephyr workspace Entry convention.
///
/// Fails fast with a `just esp32 build-fixtures` hint when the binary is
/// absent.
pub fn get_prebuilt_esp32_qemu_workspace_entry() -> TestResult<PathBuf> {
    // issue 0517 — the row's artifact root, not a hand-spelled one. The rest of
    // the path (triple / profile / bin) is what cargo writes BELOW the root and
    // is carried verbatim.
    let elf =
        crate::fixtures::groups::workspace_artifact_dir("workspace-rust-esp32")?.join(format!(
            "riscv32imc-unknown-none-elf/{}/esp32_entry",
            cargo_target_profile_dir()
        ));
    if !elf.exists() {
        return Err(TestError::BuildFailed(format!(
            "ESP32 workspace Entry binary not found: {}\n\
             Build the workspace fixtures first: `just esp32 build-fixtures` \
             (or `bash scripts/build/workspace-fixtures-build.sh esp32 rust`).",
            elf.display()
        )));
    }
    Ok(elf)
}

// ───────────────────────────────────────────────────────────────────────────
// Phase 169.4b — ESP32-C3 QEMU Rust DDS fixture builders deleted
// alongside the Rust DDS retirement (Phase 169.2 deleted the example
// crates).
// ───────────────────────────────────────────────────────────────────────────

// ═══════════════════════════════════════════════════════════════════════════
// RTIC QEMU Example Builders (MPS2-AN385, Cortex-M3)
// ═══════════════════════════════════════════════════════════════════════════

/// Cached path to the qemu-rtic-talker binary
static QEMU_RTIC_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the qemu-rtic-listener binary
static QEMU_RTIC_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Build qemu-rtic-talker (cached)
pub fn build_qemu_rtic_talker() -> TestResult<&'static Path> {
    QEMU_RTIC_TALKER_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-rtic-talker"))
        .map(|p| p.as_path())
}

/// Build qemu-rtic-listener (cached)
pub fn build_qemu_rtic_listener() -> TestResult<&'static Path> {
    QEMU_RTIC_LISTENER_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-rtic-listener"))
        .map(|p| p.as_path())
}

/// Cached path to the qemu-rtic-service-server binary
static QEMU_RTIC_SERVICE_SERVER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the qemu-rtic-service-client binary
static QEMU_RTIC_SERVICE_CLIENT_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Build qemu-rtic-service-server (cached)
pub fn build_qemu_rtic_service_server() -> TestResult<&'static Path> {
    QEMU_RTIC_SERVICE_SERVER_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-rtic-service-server"))
        .map(|p| p.as_path())
}

/// Build qemu-rtic-service-client (cached)
pub fn build_qemu_rtic_service_client() -> TestResult<&'static Path> {
    QEMU_RTIC_SERVICE_CLIENT_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-rtic-service-client"))
        .map(|p| p.as_path())
}

// ═══════════════════════════════════════════════════════════════════════════
// C++ Example Builders (CMake-based)
// ═══════════════════════════════════════════════════════════════════════════

/// Cached path to the cpp-talker binary
static CPP_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the cpp-listener binary
static CPP_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the cpp-service-server binary
static CPP_SERVICE_SERVER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the cpp-service-client binary
static CPP_SERVICE_CLIENT_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the cpp-service-client-callback binary (RFC-0041 / Phase 239)
static CPP_SERVICE_CLIENT_CALLBACK_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the cpp-action-server binary
static CPP_ACTION_SERVER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the cpp-action-client binary
static CPP_ACTION_CLIENT_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the cpp-action-client-callback binary (RFC-0041 / Phase 239)
static CPP_ACTION_CLIENT_CALLBACK_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the cpp-parameters binary
static CPP_PARAMETERS_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Build cpp-talker example (cached)
pub fn build_cpp_talker() -> TestResult<&'static Path> {
    CPP_TALKER_BINARY
        .get_or_try_init(|| build_example_cmake_rmw("native/cpp/talker", "cpp_talker", Rmw::Zenoh))
        .map(|p| p.as_path())
}

/// Build cpp-listener example (cached)
pub fn build_cpp_listener() -> TestResult<&'static Path> {
    CPP_LISTENER_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw("native/cpp/listener", "cpp_listener", Rmw::Zenoh)
        })
        .map(|p| p.as_path())
}

/// Build cpp-service-server example (cached)
pub fn build_cpp_service_server() -> TestResult<&'static Path> {
    CPP_SERVICE_SERVER_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw(
                "native/cpp/service-server",
                "cpp_service_server",
                Rmw::Zenoh,
            )
        })
        .map(|p| p.as_path())
}

/// Build cpp-service-client example (cached)
pub fn build_cpp_service_client() -> TestResult<&'static Path> {
    CPP_SERVICE_CLIENT_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw(
                "native/cpp/service-client",
                "cpp_service_client",
                Rmw::Zenoh,
            )
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the cpp-talker binary path
#[rstest::fixture]
pub fn cpp_talker_binary() -> PathBuf {
    build_cpp_talker()
        .expect("Failed to build cpp-talker")
        .to_path_buf()
}

/// rstest fixture that provides the cpp-listener binary path
#[rstest::fixture]
pub fn cpp_listener_binary() -> PathBuf {
    build_cpp_listener()
        .expect("Failed to build cpp-listener")
        .to_path_buf()
}

/// Build cpp-service-client-callback example (cached, RFC-0041 / Phase 239)
pub fn build_cpp_service_client_callback() -> TestResult<&'static Path> {
    CPP_SERVICE_CLIENT_CALLBACK_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw(
                "native/cpp/service-client-callback",
                "cpp_service_client_callback",
                Rmw::Zenoh,
            )
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the cpp-service-server binary path
#[rstest::fixture]
pub fn cpp_service_server_binary() -> PathBuf {
    build_cpp_service_server()
        .expect("Failed to build cpp-service-server")
        .to_path_buf()
}

/// rstest fixture that provides the cpp-service-client binary path
#[rstest::fixture]
pub fn cpp_service_client_binary() -> PathBuf {
    build_cpp_service_client()
        .expect("Failed to build cpp-service-client")
        .to_path_buf()
}

/// Build cpp-action-server example (cached)
pub fn build_cpp_action_server() -> TestResult<&'static Path> {
    CPP_ACTION_SERVER_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw("native/cpp/action-server", "cpp_action_server", Rmw::Zenoh)
        })
        .map(|p| p.as_path())
}

/// Build cpp-action-client example (cached)
pub fn build_cpp_action_client() -> TestResult<&'static Path> {
    CPP_ACTION_CLIENT_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw("native/cpp/action-client", "cpp_action_client", Rmw::Zenoh)
        })
        .map(|p| p.as_path())
}

/// Build cpp-action-client-callback example (cached, RFC-0041 / Phase 239)
pub fn build_cpp_action_client_callback() -> TestResult<&'static Path> {
    CPP_ACTION_CLIENT_CALLBACK_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw(
                "native/cpp/action-client-callback",
                "cpp_action_client_callback",
                Rmw::Zenoh,
            )
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the cpp-action-server binary path
#[rstest::fixture]
pub fn cpp_action_server_binary() -> PathBuf {
    build_cpp_action_server()
        .expect("Failed to build cpp-action-server")
        .to_path_buf()
}

/// rstest fixture that provides the cpp-action-client binary path
#[rstest::fixture]
pub fn cpp_action_client_binary() -> PathBuf {
    build_cpp_action_client()
        .expect("Failed to build cpp-action-client")
        .to_path_buf()
}

/// Build cpp-parameters example (cached)
pub fn build_cpp_parameters() -> TestResult<&'static Path> {
    CPP_PARAMETERS_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw("native/cpp/parameters", "cpp_parameters", Rmw::Zenoh)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the cpp-parameters binary path
#[rstest::fixture]
pub fn cpp_parameters_binary() -> PathBuf {
    build_cpp_parameters()
        .expect("Failed to build cpp-parameters")
        .to_path_buf()
}

/// Build c-parameters example (cached) — phase-277 W5, the C sibling of
/// cpp-parameters (extracted from the pre-W5 native/c/talker demo block).
pub fn build_c_parameters() -> TestResult<&'static Path> {
    static C_PARAMETERS_BINARY: OnceCell<PathBuf> = OnceCell::new();
    C_PARAMETERS_BINARY
        .get_or_try_init(|| {
            build_example_cmake_rmw("native/c/parameters", "c_parameters", Rmw::Zenoh)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that provides the c-parameters binary path
#[rstest::fixture]
pub fn c_parameters_binary() -> PathBuf {
    build_c_parameters()
        .expect("Failed to build c-parameters")
        .to_path_buf()
}

/// Cached path to the qemu-rtic-action-server binary
static QEMU_RTIC_ACTION_SERVER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the qemu-rtic-action-client binary
static QEMU_RTIC_ACTION_CLIENT_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Build qemu-rtic-action-server (cached)
pub fn build_qemu_rtic_action_server() -> TestResult<&'static Path> {
    QEMU_RTIC_ACTION_SERVER_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-rtic-action-server"))
        .map(|p| p.as_path())
}

/// Build qemu-rtic-action-client (cached)
pub fn build_qemu_rtic_action_client() -> TestResult<&'static Path> {
    QEMU_RTIC_ACTION_CLIENT_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-rtic-action-client"))
        .map(|p| p.as_path())
}

// ═══════════════════════════════════════════════════════════════════════════
// QEMU RTIC Mixed-Priority Example Builders (ffi-sync)
// ═══════════════════════════════════════════════════════════════════════════

/// Cached path to the qemu-rtic-mixed-talker binary
static QEMU_RTIC_MIXED_TALKER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Cached path to the qemu-rtic-mixed-listener binary
static QEMU_RTIC_MIXED_LISTENER_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Build qemu-rtic-mixed-talker (cached)
pub fn build_qemu_rtic_mixed_talker() -> TestResult<&'static Path> {
    QEMU_RTIC_MIXED_TALKER_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-rtic-mixed-talker"))
        .map(|p| p.as_path())
}

/// Build qemu-rtic-mixed-listener (cached)
pub fn build_qemu_rtic_mixed_listener() -> TestResult<&'static Path> {
    QEMU_RTIC_MIXED_LISTENER_BINARY
        // Phase 226.D — built into build/cargo-fixtures/qemu-arm-baremetal.
        .get_or_try_init(|| require_qemu_baremetal_fixture("qemu-rtic-mixed-listener"))
        .map(|p| p.as_path())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Issue 0608 — a group-built row must be looked up at its PLATFORM's
    /// profile, not the ambient one. The NuttX carve-out is the case that
    /// failed: the builder wrote `nros-minsizerel` and the resolver asked for
    /// `nros-relwithdebinfo`, so a binary that existed reported as MISSING.
    ///
    /// Asserts the AGREEMENT (what `platform_profile` says) rather than the
    /// constant, per #393 — restating `nros-minsizerel` here would keep passing
    /// if the carve-out moved.
    #[test]
    fn a_row_is_resolved_at_its_platforms_profile() {
        let ambient = cargo_target_profile_dir();

        for platform in ["nuttx", "nuttx-riscv", "freertos"] {
            let want = nros_cargo_profile::target_dir(
                nros_cargo_profile::platform_profile(platform)
                    .unwrap_or_else(|| panic!("{platform} lost its carve-out")),
            );
            let row = crate::fixtures::groups::GroupRow::for_test(platform);
            let rel = PathBuf::from(format!("armv7a-nuttx-eabihf/{ambient}/listener"));
            let got = rel_at_row_profile(&row, &rel);
            assert_eq!(
                got,
                PathBuf::from(format!("armv7a-nuttx-eabihf/{want}/listener")),
                "{platform}: the profile component must become the carve-out"
            );
        }

        // A platform with no carve-out is left exactly as it was — the rewrite
        // must not invent a redirect for the 190+ `linux` rows.
        let row = crate::fixtures::groups::GroupRow::for_test("linux");
        let rel = PathBuf::from(format!("{ambient}/talker"));
        assert_eq!(rel_at_row_profile(&row, &rel), rel);

        // Only the profile COMPONENT moves. A binary that happens to be named
        // like the ambient profile stays put.
        let row = crate::fixtures::groups::GroupRow::for_test("nuttx");
        let rel = PathBuf::from(format!("{ambient}/{ambient}"));
        let got = rel_at_row_profile(&row, &rel);
        let want = nros_cargo_profile::target_dir(nros_cargo_profile::NUTTX_RUST_PROFILE);
        assert_eq!(got, PathBuf::from(format!("{want}/{want}")));
    }

    /// Issue 0608 — and the RESOLVER must actually call it.
    ///
    /// The test above passes with the chokepoint bypassed, because it exercises
    /// `rel_at_row_profile` directly. That is the gap this codebase keeps
    /// finding in gates (issue 0196): the rule is right and the wiring is not
    /// checked. So drive `require_prebuilt_row_binary` for real and assert on
    /// the path it went looking for.
    #[test]
    fn the_row_resolver_uses_the_carve_out_profile() {
        // Neutralize any LANE narrowing first. This test drives the resolver
        // with a FABRICATED row (`GroupRow::for_test`, an artifact_root that
        // does not exist) to read back which profile the lookup asked for. Under
        // a narrowed lane — `just ci-matrix` exports `NROS_TEST_COORDS` — the
        // resolver's coordinate skip fires BEFORE it composes that path, so the
        // assertion reads `[SKIPPED:lane] out of lane: … nuttx,rust,zenoh`
        // instead of the profile dir, and the test can never pass in tier 2.
        //
        // A lane skip is ABSORBING (issue 0445): whatever the call would have
        // reported is replaced by a message explaining itself. A synthetic row
        // has no manifest identity to narrow ON, so the narrowing is meaningless
        // here rather than merely inconvenient.
        //
        // SAFETY: nextest runs each test in its own process, so this mutates no
        // other test's environment; it must precede the first `lane::run_coords`
        // call, which latches a `OnceLock`.
        unsafe { std::env::remove_var(crate::fixtures::lane::RUN_COORDS_ENV) };

        let ambient = cargo_target_profile_dir();
        let want = nros_cargo_profile::target_dir(nros_cargo_profile::NUTTX_RUST_PROFILE);
        assert_ne!(
            want, ambient,
            "the carve-out must differ or this proves nothing"
        );

        // `shared: false` routes `row_resolved_dir` through `artifact_root`,
        // repo-relative — so point it at a directory that does not exist and
        // read which profile the lookup asked for.
        let mut row = crate::fixtures::groups::GroupRow::for_test("nuttx");
        row.shared = false;
        row.artifact_root = "tmp/issue-0608-probe".to_string();

        let rel = PathBuf::from(format!("armv7a-nuttx-eabihf/{ambient}/listener"));

        // `catch_unwind`, not `expect_err`: a missing IN-LANE fixture is a
        // "broken promise" PANIC, not an `Err` — a gated run already asserted
        // the lane was built. Either way the diagnostic names the path it went
        // looking for, which is what this test reads.
        let prev = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));
        let outcome = std::panic::catch_unwind(|| require_prebuilt_row_binary(&row, &rel));
        std::panic::set_hook(prev);

        let msg = match outcome {
            Err(payload) => payload
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
                .unwrap_or_else(|| "<non-string panic>".to_string()),
            Ok(Err(e)) => format!("{e:?}"),
            Ok(Ok(p)) => panic!("no such fixture exists, yet it resolved: {}", p.display()),
        };

        assert!(
            msg.contains(&format!("armv7a-nuttx-eabihf/{want}/listener")),
            "resolver looked somewhere other than the carve-out profile: {msg}"
        );
        assert!(
            !msg.contains(&format!("armv7a-nuttx-eabihf/{ambient}/listener")),
            "resolver still used the AMBIENT profile — this is issue 0608: {msg}"
        );
    }

    /// phase-363 — the bootstrap fallback must be VISIBLE.
    ///
    /// Measured: every zenoh fixture build dir carries a build-script record, so
    /// this arm is unreachable today. That is the argument for the assertion,
    /// not against it — an arm that never runs is one nobody notices has
    /// started running, and CLAUDE.md tells the reader to believe the `probe:`
    /// line over the verdict.
    #[test]
    fn falling_back_to_a_hand_authored_input_set_is_reported() {
        staleness::begin_probe();
        assert!(
            !staleness::probe_accounting().contains("UNMEASURED"),
            "a measured probe must not claim otherwise"
        );

        staleness::note_unmeasured_input_set();
        assert!(
            staleness::probe_accounting().contains("UNMEASURED"),
            "the fallback must say so in the line the reader is told to trust"
        );

        // …and the next probe starts clean, or one degraded resolution would
        // taint every later verdict in the process.
        staleness::begin_probe();
        assert!(!staleness::probe_accounting().contains("UNMEASURED"));
    }

    /// phase-363 — the zpico probe reads the inputs cargo RECORDED, not a
    /// hand-authored walk. Hermetic: builds a fake build-script `output` in a
    /// temp tree, so it asserts the parser and the search, never a real fixture.
    /// Issue 0696 — a RELATIVE `rerun-if-changed` entry belongs to the crate that
    /// recorded it, never to whatever directory the test process happens to run in.
    ///
    /// `zpico-sys` records `rerun-if-changed=src/lib.rs` (its own). A nextest
    /// binary's CWD is `packages/testing/nros-tests`, so resolving that entry
    /// against the CWD produced the HARNESS's `src/lib.rs` — a file no C fixture
    /// depends on, in-repo enough to pass the filter, and unclearable by any
    /// build. This pins the base rather than the symptom: both paths exist, so a
    /// regression cannot hide behind a missing file.
    #[test]
    fn relative_recorded_input_resolves_against_the_recording_crate() {
        let harness_lib = project_root().join("packages/testing/nros-tests/src/lib.rs");
        let zpico_lib = zpico_manifest_dir().join("src/lib.rs");
        assert!(
            harness_lib.is_file() && zpico_lib.is_file(),
            "both files must exist or this test proves nothing"
        );

        let resolved = zpico_manifest_dir()
            .join("src/lib.rs")
            .canonicalize()
            .unwrap();
        assert_eq!(
            resolved,
            zpico_lib.canonicalize().unwrap(),
            "a relative entry must land in zpico-sys"
        );
        assert_ne!(
            resolved,
            harness_lib.canonicalize().unwrap(),
            "issue 0696: `src/lib.rs` resolved into the test harness, so every \
             native C/C++ fixture read STALE against a file it never depended on"
        );
    }

    #[test]
    fn zpico_inputs_come_from_the_build_script_output() {
        let root = project_root();
        let tmp = root.join("tmp/zpico-probe-test");
        let _ = fs::remove_dir_all(&tmp);
        // The REAL nesting corrosion produces, measured on
        // `examples/qemu-riscv64-threadx/cpp/action-client/build-zenoh`:
        //   cargo/<pkg>_<hash>/<triple>/<profile>/build/zpico-sys-<hash>/output
        // The `zpico-sys-*` dir is matched at recursion depth 5 from the build
        // dir — measured, not counted by eye: MAX_DEPTH 4 fails this test and 5
        // passes. The shipped 8 is that bound plus headroom for a deeper
        // corrosion layout, and it is a real bound rather than a round number:
        // without it a staleness probe walks the whole cmake tree.
        let bs = tmp.join(
            "cargo/nano-ros_0b88c/riscv64gc-unknown-none-elf/release/build/zpico-sys-deadbeef",
        );
        fs::create_dir_all(&bs).unwrap();

        // A real in-repo path, a real in-repo DIR, and two that must be dropped:
        // a system header (not in-repo) and a non-rerun line.
        let in_repo_file = root.join("packages/rmw/zenoh/zpico-sys/c/zpico/zpico.c");
        let in_repo_dir = root.join("packages/platform/nros-platform-api/include");
        fs::write(
            bs.join("output"),
            format!(
                "cargo:rerun-if-changed={}\n\
                 cargo:rerun-if-changed={}\n\
                 cargo:rerun-if-changed=/usr/include/stdio.h\n\
                 cargo:rustc-link-lib=static=zpico\n",
                in_repo_file.display(),
                in_repo_dir.display(),
            ),
        )
        .unwrap();

        let found = zpico_recorded_inputs(&tmp);
        assert!(
            found.contains(&in_repo_file.canonicalize().unwrap()),
            "the recorded C source must be an input: {found:?}"
        );
        assert!(
            found.contains(&in_repo_dir.canonicalize().unwrap()),
            "a recorded DIRECTORY must be an input — this is the entry the old \
             walk could not see: {found:?}"
        );
        assert!(
            !found.iter().any(|p| p.starts_with("/usr")),
            "a system header is not this tree's staleness: {found:?}"
        );

        // The measured set must be BROADER than the old walk: prove the
        // directory entry lies outside `zpico-sys/c`, which is all the walk
        // ever looked at.
        let old_walk_root = root.join("packages/rmw/zenoh/zpico-sys/c");
        assert!(
            !in_repo_dir.starts_with(&old_walk_root),
            "this assertion is vacuous if the include tree moved under the walk"
        );

        // A DIRECTORY input reports a newer file inside it — the case the
        // extension-filtered walk also could not express.
        let long_ago = std::time::SystemTime::UNIX_EPOCH;
        assert!(
            newest_path_after(&in_repo_dir, long_ago).is_some(),
            "a directory input must resolve to a file under it"
        );
        // …and nothing is newer than the far future, so this is not a probe
        // that simply always fires.
        let far_future = std::time::SystemTime::now() + std::time::Duration::from_secs(86_400);
        assert!(
            newest_path_after(&in_repo_dir, far_future).is_none(),
            "the probe must not report staleness against a future binary"
        );

        fs::remove_dir_all(&tmp).unwrap();
    }

    #[test]
    fn test_project_root_has_examples() {
        let root = project_root();
        assert!(root.join("examples").exists());
    }

    /// A staleness verdict must carry what the probe compared and how long the
    /// coordinate has not run (issue 0445) — otherwise it absorbs the runtime
    /// result nobody then sees, which is how issue 0444 hid behind 0442.
    #[test]
    fn a_stale_verdict_reports_its_own_reasoning_and_its_age() {
        // Unique per invocation: phase-353 W2 gave the probe on-disk memory
        // (`.nros-srcbaseline`, written beside the artifact), so a FIXED
        // directory is shared mutable state between parallel tests in this
        // binary. The path was safe only while the probe was stateless.
        let dir = project_root().join(format!(
            "target/nros-probe-selftest-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        let bin = dir.join("fixture-bin");
        let src = dir.join("edited.rs");
        // Binary first, then the source, so the source is unambiguously newer.
        fs::write(&bin, b"binary").unwrap();
        std::thread::sleep(std::time::Duration::from_millis(20));
        fs::write(&src, b"edit").unwrap();
        // Two deps: one exempt (regenerated in place), one real edit.
        let exempt = project_root().join("packages/rmw/zenoh/zpico-sys/c/include/zpico.h");
        fs::write(
            bin.with_extension("d"),
            format!(
                "{}: {} {}\n",
                bin.display(),
                exempt.display(),
                src.display()
            ),
        )
        .unwrap();

        staleness::record_fresh(&bin).expect("self-test probe is not degraded");
        let first = require_prebuilt_binary_fresh(&bin).unwrap_err().to_string();
        assert!(
            first.contains("probe:") && first.contains("examined 2"),
            "the verdict must account for what it compared: {first}"
        );
        assert!(
            first.contains("exempted 1 regenerated-in-place header"),
            "an exemption applied by one arm and not its sibling IS issue 0442, and \
             it is only visible if the verdict prints it: {first}"
        );
        assert!(
            !first.contains("NOT RUN"),
            "one stale verdict is the normal case: {first}"
        );

        let _ = require_prebuilt_binary_fresh(&bin);
        let third = require_prebuilt_binary_fresh(&bin).unwrap_err().to_string();
        assert!(
            third.contains("NOT RUN") && third.contains("3th consecutive"),
            "a coordinate stale run after run has produced no runtime result and \
             must say so: {third}"
        );

        // Resolving fresh ends the non-running run.
        fs::remove_file(bin.with_extension("d")).unwrap();
        require_prebuilt_binary_fresh(&bin).unwrap();
        fs::write(
            bin.with_extension("d"),
            format!("{}: {}\n", bin.display(), src.display()),
        )
        .unwrap();
        let after = require_prebuilt_binary_fresh(&bin).unwrap_err().to_string();
        assert!(
            !after.contains("NOT RUN"),
            "the count measures non-running, not age — a cell that ran must reset: {after}"
        );

        staleness::record_fresh(&bin).expect("self-test probe is not degraded");
        let _ = fs::remove_dir_all(&dir);
    }
    /// phase-353 W2 direction (3) — the DEP-INFO arm must be content-aware too.
    ///
    /// The zephyr arm has consulted `candidates_changed_content` since #147 /
    /// phase-286 W2; this arm compared raw mtimes, so `git pull --rebase`,
    /// `git stash push/pop` and a branch switch — all of which rewrite tracked
    /// files whose BYTES are identical — turned every prebuilt fixture cold
    /// (~28 s per cold Zephyr leaf, issue 0509 / issue 0466). One probe, two
    /// arms, one of them wrong: issue 0442's shape.
    ///
    /// The test drives `dep_info_newer_source` through a real `.d` file, so it
    /// fails if the arm stops consulting the shared helper.
    #[test]
    fn dep_info_arm_forgives_a_byte_identical_rewrite_but_not_an_edit() {
        use std::{thread::sleep, time::Duration};
        let dir = std::env::temp_dir().join(format!("nros-w2-depinfo-{}", std::process::id()));
        let _ = fs::create_dir_all(&dir);
        let bin = dir.join("fixture");
        let src = dir.join("watched.rs");
        fs::write(&src, b"fn main() {}").unwrap();
        fs::write(&bin, b"ARTIFACT").unwrap();
        // rustc's dep-info shape: `<target>: <deps>`
        fs::write(
            bin.with_extension("d"),
            format!("{}: {}\n", bin.display(), src.display()),
        )
        .unwrap();

        // Baseline: nothing newer than the binary.
        assert!(
            dep_info_newer_source(&bin).is_none(),
            "a freshly built fixture must not be stale"
        );

        // The rewrite a rebase performs: same bytes, newer mtime.
        sleep(Duration::from_millis(10));
        fs::write(&src, b"fn main() {}").unwrap();
        assert!(
            dep_info_newer_source(&bin).is_none(),
            "an mtime-only rewrite was called STALE — the treadmill is back"
        );

        // A real edit must still be caught, or the probe is worthless.
        sleep(Duration::from_millis(10));
        fs::write(&src, b"fn main() { edited() }").unwrap();
        assert!(
            dep_info_newer_source(&bin).is_some(),
            "an edited source was forgiven — the fixture would run against stale code"
        );

        let _ = fs::remove_dir_all(&dir);
    }

    /// issue 1005 — a build-script DEPENDENCY crate is an input of a cmake
    /// fixture, and the probe must be able to see it.
    ///
    /// `Z_TRANSPORT_LEASE_MS` lives in `nros-zpico-build`, which `zpico-sys`
    /// build-depends on. Cargo tracks that through its unit graph and never
    /// through a `rerun-if-changed` path, so the arm that replays recorded
    /// paths is blind to it BY CONSTRUCTION — measured on the FreeRTOS C
    /// talker, whose recorded set names 28 in-repo inputs and none of them
    /// there. Ten days of FreeRTOS zenoh fixtures therefore read FRESH while
    /// baking the 10 s lease issue 0906 measured at 19 heard of 77.
    ///
    /// Hermetic: a synthetic corrosion layout plus a `.d` in the shape cargo
    /// writes, so this asserts the search and the parse rather than the state
    /// of anybody's build tree.
    #[test]
    fn a_build_script_dependency_crate_is_a_staleness_input() {
        let root = project_root();
        let tmp = root.join("tmp/issue-1005-probe");
        let _ = fs::remove_dir_all(&tmp);

        // The layout corrosion produces, measured on
        // `examples/qemu-arm-freertos/c/talker/build-zenoh` (2026-09-04):
        //   cargo/<pkg>_<hash>/<triple>/<profile>/{deps/,libnros_c.d,libnros_c.a}
        let profile = tmp.join("cargo/nano-ros_0b88c/thumbv7m-none-eabi/release");
        fs::create_dir_all(profile.join("deps")).unwrap();

        // The build-script dependency crate that issue 1005 measured.
        let build_dep = root.join("packages/rmw/zenoh/nros-zpico-build/src/lib.rs");
        assert!(
            build_dep.is_file(),
            "the crate this probes moved; re-point the test, do not delete it"
        );
        fs::write(
            profile.join("libnros_c.d"),
            format!(
                "{}: {} /usr/include/stdio.h\n",
                profile.join("libnros_c.a").display(),
                build_dep.display(),
            ),
        )
        .unwrap();
        // `deps/*.d` must NOT be read: those name paths RELATIVE to the cargo
        // invocation's cwd, and a nextest process resolving one against ITS cwd
        // is issue 0696 — one wrong file, always the same one. The staticlib
        // `.d` beside the `.a` is absolute and is already their union.
        fs::write(
            profile.join("deps/nros_zpico_build-dead.d"),
            "x: src/lib.rs\n",
        )
        .unwrap();

        let long_ago = std::time::SystemTime::UNIX_EPOCH;
        let (newer, candidates) = cargo_rust_inputs(&tmp, long_ago);
        assert_eq!(
            newer.as_deref(),
            Some(build_dep.as_path()),
            "the build-script dependency must be reported: {candidates:?}"
        );
        assert!(
            !candidates
                .iter()
                .any(|p| p.ends_with("src/lib.rs") && p.is_relative()),
            "a relative `deps/*.d` entry must not enter the set (issue 0696): {candidates:?}"
        );

        // …and this is not a probe that simply always fires.
        let far_future = std::time::SystemTime::now() + std::time::Duration::from_secs(86_400);
        assert!(
            cargo_rust_inputs(&tmp, far_future).0.is_none(),
            "nothing is newer than a future artifact"
        );

        // The point of the arm: the RECORDED-path arm cannot express this. Its
        // set is what a build script READ, never what it was COMPILED FROM.
        let recorded = zpico_recorded_inputs(&tmp);
        assert!(
            !recorded.contains(&build_dep),
            "if the recorded set ever names it, this test is vacuous"
        );

        fs::remove_dir_all(&tmp).unwrap();
    }

    /// issue 1005 — …and the PROBE must actually consult that arm.
    ///
    /// The test above drives `cargo_rust_inputs` directly, so it passes with
    /// the arm unwired. That is the gap this codebase keeps finding in gates
    /// (issue 0196): the rule is right and the wiring is not checked. So drive
    /// `cmake_dep_info_newer_source` end to end over a synthetic build dir and
    /// assert it reports the cargo-side input.
    ///
    /// The synthetic input is given a FUTURE mtime and the artifact a present
    /// one, so every other arm is quiet by construction: `ninja -t deps` has no
    /// graph to answer from, and the zpico bootstrap walk sees a real
    /// `zpico-sys/c` tree that is older than the artifact. Whatever this
    /// reports therefore came from the cargo arm and nowhere else.
    #[test]
    fn the_cmake_probe_consults_the_cargo_input_arm() {
        let root = project_root();
        let tmp = root.join("tmp/issue-1005-wiring");
        let _ = fs::remove_dir_all(&tmp);
        let build_dir = tmp.join("build-zenoh");
        let profile = build_dir.join("cargo/nano-ros_0b88c/thumbv7m-none-eabi/release");
        fs::create_dir_all(profile.join("deps")).unwrap();
        // Present, but unanswerable: `ninja_dep_paths` requires the log to
        // exist and yields nothing when the query fails.
        fs::write(build_dir.join(".ninja_deps"), b"").unwrap();

        let input = tmp.join("synthetic_input.rs");
        fs::write(&input, "// issue 1005 wiring probe\n").unwrap();
        fs::write(
            profile.join("libnros_c.d"),
            format!(
                "{}: {}\n",
                profile.join("libnros_c.a").display(),
                input.display()
            ),
        )
        .unwrap();

        let binary = build_dir.join("c_talker");
        fs::write(&binary, b"\x7fELF not really\n").unwrap();
        let day = std::time::Duration::from_secs(86_400);
        fs::File::options()
            .write(true)
            .open(&input)
            .unwrap()
            .set_times(fs::FileTimes::new().set_modified(std::time::SystemTime::now() + day))
            .unwrap();

        let verdict = cmake_dep_info_newer_source(&binary);
        assert_eq!(
            verdict.as_deref(),
            Some(input.as_path()),
            "the cmake probe must reach the cargo dep-info arm; without it the \
             fixture reports FRESH against an input cargo itself recorded"
        );

        fs::remove_dir_all(&tmp).unwrap();
    }

    /// issue 1005, second half — the build-script record lives across a SYMLINK.
    ///
    /// Since the phase-340 shared cargo group dir, a cross-compiled leaf's
    /// `build-<rmw>/cargo` is a symlink into `build/corrosion-cargo/<platform>/
    /// <hash>/`. `DirEntry::file_type()` LSTATs, so it answers "not a
    /// directory" for that symlink and the walk stopped dead: measured
    /// 2026-09-04, `zpico_recorded_inputs` returned 0 entries for every
    /// FreeRTOS / NuttX / ThreadX fixture, and the probe silently ran the
    /// hand-authored bootstrap walk that
    /// `falling_back_to_a_hand_authored_input_set_is_reported` calls
    /// unreachable. It reported 28 entries once the walk STATted instead.
    ///
    /// The fallback announces itself only through `probe_accounting()`, which
    /// is rendered inside a STALE message — so on the FRESH path, the direction
    /// that matters, it said nothing at all.
    #[test]
    fn the_build_script_record_is_found_across_a_symlinked_cargo_dir() {
        let root = project_root();
        let tmp = root.join("tmp/issue-1005-symlink");
        let _ = fs::remove_dir_all(&tmp);

        let build_dir = tmp.join("build-zenoh");
        fs::create_dir_all(&build_dir).unwrap();
        let group = tmp.join("corrosion-cargo/freertos/2e024fb928e1");
        let bs = group.join("thumbv7m-none-eabi/release/build/zpico-sys-deadbeef");
        fs::create_dir_all(&bs).unwrap();
        let in_repo_file = root.join("packages/rmw/zenoh/zpico-sys/c/zpico/zpico.c");
        fs::write(
            bs.join("output"),
            format!("cargo:rerun-if-changed={}\n", in_repo_file.display()),
        )
        .unwrap();
        std::os::unix::fs::symlink(&group, build_dir.join("cargo")).unwrap();

        let found = zpico_recorded_inputs(&build_dir);
        assert!(
            found.contains(&in_repo_file.canonicalize().unwrap()),
            "the walk must cross the group-dir symlink, or every cross-compiled \
             fixture silently falls back to the hand-authored input set: {found:?}"
        );

        fs::remove_dir_all(&tmp).unwrap();
    }
    /// issue 1045 — the leaf-`target/` literals still in this file are correct
    /// only because their platforms have no profile CARVE-OUT.
    ///
    /// Issue 1027 fixed the NuttX and FreeRTOS locators, which spelled a leaf
    /// `target/<triple>/<profile>/` path and got the profile wrong: the artifact
    /// root redirect in `require_prebuilt_binary` had already worked, and only
    /// the profile component was stale. FreeRTOS was green purely because it
    /// happened to spell the carve-out directly.
    ///
    /// Three such literals remain here — `qemu-rs-test` (qemu-arm-baremetal),
    /// `contract-monitor-*` (linux) and the esp32 examples
    /// (qemu-esp32-baremetal) — and all three resolve today, VERIFIED on disk:
    /// each artifact sits under the shared group dir at the AMBIENT profile,
    /// which is what `cargo_target_profile_dir()` spells.
    ///
    /// They are one carve-out away from issue 1027. `platform_profile` returns
    /// `Some` for exactly `freertos`, `nuttx` and `nuttx-riscv`; the day someone
    /// adds a fourth for one of these three platforms, the literal silently
    /// names a directory nothing writes and the resolver reports `not prebuilt`
    /// on a freshly built tree. This test is the tripwire for that day, and it
    /// names the sites so the fix has somewhere to start.
    #[test]
    fn the_leaf_profile_literals_only_work_because_their_platforms_have_no_carve_out() {
        for (platform, site) in [
            (
                "qemu-arm-baremetal",
                "build_qemu_test — target/thumbv7m-none-eabi/<profile>/qemu-rs-test",
            ),
            (
                "linux",
                "build_contract_monitor_bin — target/<profile>/<bin>",
            ),
            (
                "qemu-esp32-baremetal",
                "esp32 examples — target/riscv32imc-unknown-none-elf/<profile>/<bin>",
            ),
        ] {
            assert!(
                nros_cargo_profile::platform_profile(platform).is_none(),
                "`{platform}` has gained a profile carve-out, so the leaf literal in \
                 `{site}` now names a directory nothing writes — this is issue 1027 at \
                 a new site. Resolve it through the manifest row instead: \
                 `groups::select_sole_row` for the artifact ROOT and `row_profile_dir` \
                 for the PROFILE, as `nuttx.rs` and `freertos.rs` already do."
            );
        }
    }

    /// The carve-out set itself, so the tripwire above cannot quietly stop
    /// covering anything (issue 0196's rule: check that the gate still covers
    /// the rule it enforces).
    #[test]
    fn only_freertos_and_nuttx_carve_out_a_profile() {
        let carved: Vec<&str> = [
            "freertos",
            "nuttx",
            "nuttx-riscv",
            "linux",
            "qemu-arm-baremetal",
            "qemu-esp32-baremetal",
            "threadx-linux",
            "threadx-riscv64",
            "zephyr",
        ]
        .into_iter()
        .filter(|p| nros_cargo_profile::platform_profile(p).is_some())
        .collect();
        assert_eq!(
            carved,
            vec!["freertos", "nuttx", "nuttx-riscv"],
            "the set of platforms with a profile carve-out changed. Every leaf \
             `target/<triple>/<profile>/` literal in this file assumes the AMBIENT \
             profile, so a new carve-out needs its resolver moved onto the row route \
             first (issues 1027, 1045)."
        );
    }
}
