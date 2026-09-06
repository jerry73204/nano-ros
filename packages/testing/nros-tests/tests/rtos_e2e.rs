//! Parametrised RTOS E2E integration tests.
//!
//! Collapses the four per-platform E2E clusters (FreeRTOS, NuttX, ThreadX
//! Linux, ThreadX QEMU RISC-V) into three parametrised `#[rstest]`
//! functions (pubsub / service / action), each fanned out over three
//! languages (Rust / C / C++). See Phase 85.4 in
//! `docs/roadmap/phase-85-test-suite-consolidation.md`.
//!
//! Per-platform build and detection smoke tests remain in the original
//! `freertos_qemu.rs` / `nuttx_qemu.rs` / `threadx_linux.rs` /
//! `threadx_riscv64_qemu.rs` files — only the E2E bodies moved here.

use nros_tests::{
    TestError, TestResult, count_pattern,
    fixtures::{
        QemuProcess, ZenohRouter, freertos, is_qemu_available, is_qemu_riscv64_available, nuttx,
        threadx_linux, threadx_riscv64, zenohd_unavailable_reason,
    },
    platform,
    process::{ManagedProcess, kill_process_group},
};
use rstest::rstest;
use std::{
    fmt,
    path::Path,
    time::{Duration, Instant},
};

// =============================================================================
// Parameter enums
// =============================================================================

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Platform {
    Freertos,
    Nuttx,
    ThreadxLinux,
    ThreadxRiscv64,
}

impl fmt::Display for Platform {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Platform::Freertos => "freertos",
            Platform::Nuttx => "nuttx",
            Platform::ThreadxLinux => "threadx_linux",
            Platform::ThreadxRiscv64 => "threadx_riscv64",
        };
        f.write_str(s)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Lang {
    Rust,
    C,
    Cpp,
}

impl fmt::Display for Lang {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Lang::Rust => "rust",
            Lang::C => "c",
            Lang::Cpp => "cpp",
        };
        f.write_str(s)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Variant {
    Pubsub,
    Service,
    Action,
}

// =============================================================================
// RtosProcess — wraps QemuProcess and ManagedProcess with a common API
// =============================================================================

enum RtosProcess {
    Qemu(QemuProcess),
    Managed(ManagedProcess),
}

impl RtosProcess {
    /// Collect output, stopping early once `pattern` appears (issue 0471).
    ///
    /// There is no strict counterpart here: every wait in this file asserts on
    /// the collected text itself, so the strict wrapper this replaced had no
    /// callers left once they were migrated.
    fn collect_until(&mut self, pattern: &str, timeout: Duration) -> String {
        match self {
            RtosProcess::Qemu(p) => p.collect_until(pattern, timeout),
            RtosProcess::Managed(p) => p.collect_until(pattern, timeout),
        }
    }

    /// Collect output until `pattern` has appeared `expected` times, or the
    /// deadline passes — the COUNTING sibling of [`Self::collect_until`], and
    /// lenient the same way: it returns what was printed either way, so the
    /// caller asserts on the count and reports the real transcript.
    ///
    /// Issue 1013 — this is what a wait on a FREE-RUNNING node has to be. The
    /// pub/sub cell used to end its talker with `wait_for_output`, a
    /// run-to-completion wait ("wait for QEMU to produce output *and exit*")
    /// aimed at a node that never exits; the timeout became the talker's
    /// lifetime instead.
    fn collect_until_count(&mut self, pattern: &str, expected: usize, timeout: Duration) -> String {
        match self {
            RtosProcess::Qemu(p) => p.collect_until_count(pattern, expected, timeout),
            // `ManagedProcess::collect_until_count` splits an unmet count into
            // an EMPTY string plus a diagnostic (issue 0670's rule: never let
            // an assertion match the complaint about its own pattern). Correct
            // there, wrong here — this call site asserts on the COUNT, so an
            // empty string would report 0 samples where the process printed 19,
            // and the number is the whole finding. So build the count wait out
            // of the single-match wait: each call returns at the next
            // occurrence, N calls cost N lines, and the accumulated text is the
            // honest transcript.
            RtosProcess::Managed(p) => {
                let deadline = Instant::now() + timeout;
                let mut out = String::new();
                while count_pattern(&out, pattern) < expected {
                    let remaining = deadline.saturating_duration_since(Instant::now());
                    if remaining.is_zero() {
                        break;
                    }
                    let chunk = p.collect_until(pattern, remaining);
                    if chunk.is_empty() {
                        // Nothing read before the deadline, or the process is
                        // gone. Either way another lap would only spin.
                        if !p.is_running() {
                            break;
                        }
                    }
                    out.push_str(&chunk);
                }
                out
            }
        }
    }

    fn wait_for_output(&mut self, timeout: Duration) -> TestResult<String> {
        match self {
            RtosProcess::Qemu(p) => p.wait_for_output(timeout),
            // ManagedProcess::wait_for_all_output captures both stdout and
            // stderr, matching the shape the existing ThreadX Linux tests
            // use. The ThreadX Linux binaries emit their readiness banner
            // on stderr via env_logger.
            RtosProcess::Managed(p) => p.wait_for_all_output(timeout),
        }
    }

    fn kill(&mut self) {
        match self {
            RtosProcess::Qemu(p) => p.kill(),
            RtosProcess::Managed(p) => kill_process_group(p.handle_mut()),
        }
    }

    /// How this node was actually started, for the failure message.
    ///
    /// Issue 0877 — the report that "the same two images DELIVER when run by
    /// hand … with the harness's own QEMU arguments" concluded from that
    /// experiment that the images and the transport were fine and the harness
    /// was at fault. The arguments were the harness's; the BINARY was not.
    /// `qemu_system_arm_path` prefers `build/qemu/bin/qemu-system-arm` — our
    /// LAN9118 flow-control patch (issues 0830 / 0917) — over the
    /// `qemu-system-arm` a shell finds on `$PATH`, and on the host that filed
    /// 0877 those were QEMU 11.0.0-patched and QEMU 9.0.2-stock. The hand run
    /// changed the emulated NIC's RX flow control, which is exactly the layer a
    /// "nothing was delivered" symptom lives in.
    ///
    /// These lanes run with `--failure-output never`, so this has to be IN the
    /// assertion text; an `eprintln!` at spawn time is discarded.
    fn command_line(&self) -> String {
        match self {
            RtosProcess::Qemu(p) => p.command_line().to_string(),
            RtosProcess::Managed(p) => p.command_line().to_string(),
        }
    }
}

// =============================================================================
// Platform dispatch
// =============================================================================

impl Platform {
    /// Per-platform base port. Phase 89.9/89.10 splits the router port
    /// further by test variant so `pubsub`, `service`, and `action` on the
    /// same platform can run concurrently — see [`zenohd_port_for`].
    /// Phase 89.13 (pilot: FreeRTOS) additionally splits by language so the
    /// Rust / C / C++ binaries within a single variant can run in parallel
    /// on migrated platforms; see `PlatformConfig::lang_stride`.
    fn zenohd_base(self) -> &'static platform::PlatformConfig {
        match self {
            Platform::Freertos => &platform::FREERTOS,
            Platform::Nuttx => &platform::NUTTX,
            Platform::ThreadxLinux => &platform::THREADX_LINUX,
            Platform::ThreadxRiscv64 => &platform::THREADX_RISCV,
        }
    }

    fn zenohd_port_for(self, variant: Variant, lang: Lang) -> u16 {
        let pv = match variant {
            Variant::Pubsub => platform::TestVariant::Pubsub,
            Variant::Service => platform::TestVariant::Service,
            Variant::Action => platform::TestVariant::Action,
        };
        let pl = match lang {
            Lang::Rust => platform::TestLang::Rust,
            Lang::C => platform::TestLang::C,
            Lang::Cpp => platform::TestLang::Cpp,
        };
        self.zenohd_base().zenohd_port_for(pv, pl)
    }

    fn zenoh_router_start(self, variant: Variant, lang: Lang) -> TestResult<ZenohRouter> {
        // ThreadX Linux is bridge-networked (veth pairs), so zenohd must
        // bind to 0.0.0.0 to be reachable from the bridged simulation
        // interface. The QEMU-based platforms use slirp and reach zenohd
        // via the slirp gateway (10.0.2.2) forwarded to host localhost.
        let port = self.zenohd_port_for(variant, lang);
        match self {
            Platform::ThreadxLinux => ZenohRouter::start_on("0.0.0.0", port),
            _ => ZenohRouter::start_slirp(port),
        }
    }

    fn stabilization_delay(self) -> Duration {
        match self {
            // QEMU cold-boot + zenoh connect — ~15s is typical; use 20s
            // margin to match the original per-platform tests.
            Platform::Freertos | Platform::Nuttx | Platform::ThreadxRiscv64 => {
                Duration::from_secs(20)
            }
            // ThreadX Linux is a native-process simulation. Keep this
            // below zenohd's 10 s lease: a full-lease head start lets
            // the first process expire just as the peer starts.
            Platform::ThreadxLinux => Duration::from_secs(1),
        }
    }

    /// `Ok(())` when this platform can run an e2e test here, else the reason.
    ///
    /// Issue 0982 — this returned `bool` and printed the reason with
    /// `eprintln!`. nextest runs these lanes with `--failure-output never`, so
    /// stderr is suppressed and the CI log said only `require_e2e check failed
    /// for freertos`: the gate, never the missing tool. Five nightly cells
    /// reported that for nine tests each. The messages were always here; they
    /// just could not reach the skip.
    fn require_e2e(self) -> Result<(), String> {
        match self {
            Platform::Freertos => {
                if !freertos::is_freertos_available() {
                    return Err("FREERTOS_DIR not set or invalid".to_string());
                }
                if !freertos::is_lwip_available() {
                    return Err("LWIP_DIR not set or invalid".to_string());
                }
                if !freertos::is_arm_gcc_available() {
                    return Err("arm-none-eabi-gcc not found".to_string());
                }
                if !is_qemu_available() {
                    return Err("qemu-system-arm not found".to_string());
                }
                zenohd_unavailable_reason().map_or(Ok(()), Err)
            }
            Platform::Nuttx => {
                if !nuttx::is_nuttx_available() {
                    return Err("NUTTX_DIR not set or invalid".to_string());
                }
                if !nuttx::is_nuttx_configured() {
                    return Err("NuttX not configured".to_string());
                }
                if !nuttx::is_arm_gcc_available() {
                    return Err("arm-none-eabi-gcc not found".to_string());
                }
                if !nuttx::is_nuttx_toolchain_available() {
                    return Err(
                        "nightly toolchain missing rust-src for armv7a-nuttx-eabihf".to_string()
                    );
                }
                // Issue 0743 — ask for the ARM kernel specifically. The old
                // `.is_none()` check passed on a riscv image, because the two
                // configurations share the one filename.
                if let Err(why) = nuttx::nuttx_kernel_path_for(nuttx::NuttxArch::Arm) {
                    return Err(why.to_string());
                }
                if !is_qemu_available() {
                    return Err("qemu-system-arm not found".to_string());
                }
                zenohd_unavailable_reason().map_or(Ok(()), Err)
            }
            Platform::ThreadxLinux => {
                if !threadx_linux::is_threadx_available() {
                    return Err("THREADX_DIR not set or invalid".to_string());
                }
                if !threadx_linux::is_nsos_netx_available() {
                    return Err(
                        "nsos-netx not found at packages/drivers/net/nsos-netx/".to_string()
                    );
                }
                zenohd_unavailable_reason().map_or(Ok(()), Err)
            }
            Platform::ThreadxRiscv64 => {
                if !threadx_riscv64::is_threadx_available() {
                    return Err("THREADX_DIR not set or invalid".to_string());
                }
                if !threadx_riscv64::is_netx_available() {
                    return Err("NETX_DIR not set or invalid".to_string());
                }
                if !threadx_riscv64::is_riscv_gcc_available() {
                    return Err("riscv64-unknown-elf-gcc not found".to_string());
                }
                if !is_qemu_riscv64_available() {
                    return Err("qemu-system-riscv64 not found".to_string());
                }
                zenohd_unavailable_reason().map_or(Ok(()), Err)
            }
        }
    }

    /// Spawn a platform-specific emulator / process for the given binary.
    ///
    /// `node_idx` is 0 for the "first" node (talker / server) and 1 for
    /// the "second" node (listener / client). It's only meaningful for
    /// ThreadX QEMU RISC-V, where the MAC address is derived from the
    /// index. Other platforms either use slirp (per-instance NAT) or
    /// pre-assigned static IPs in the firmware and ignore the index.
    fn start_process(
        self,
        binary: &Path,
        node_idx: u8,
        name: &str,
        lang: Lang,
    ) -> TestResult<RtosProcess> {
        match self {
            Platform::Freertos => {
                // phase-287 W6 — the C/C++ images run the board's STATIC lwIP
                // plan (192.0.3.10 / gw 192.0.3.1, no DHCP): default slirp
                // (10.0.2.0/24) never answers their gateway ARP, so they get
                // the board-net slirp launcher and bake `tcp/192.0.3.1:<port>`
                // locators (examples/fixtures.toml). The RUST *-entry images
                // keep the historical DEFAULT-slirp plan (guest 10.0.2.15,
                // host 10.0.2.2 — their Cargo `[package.metadata.nros.deploy]`
                // bakes `tcp/10.0.2.2:<port>`), so they need the plain
                // launcher (issue #181; unifying the plans is follow-up work).
                if matches!(lang, Lang::Rust) {
                    QemuProcess::start_mps2_an385_networked(binary).map(RtosProcess::Qemu)
                } else {
                    QemuProcess::start_mps2_an385_freertos_slirp(binary).map(RtosProcess::Qemu)
                }
            }
            Platform::Nuttx => QemuProcess::start_nuttx_virt(binary, true).map(RtosProcess::Qemu),
            Platform::ThreadxLinux => {
                ManagedProcess::spawn(binary, &[], name).map(RtosProcess::Managed)
            }
            Platform::ThreadxRiscv64 => {
                QemuProcess::start_riscv64_virt(binary, node_idx).map(RtosProcess::Qemu)
            }
        }
    }

    /// Per-(lang, variant) skip reason, or `None` if the combination is
    /// expected to run on this platform.
    ///
    /// Phase 140 — the pre-140 NuttX C/C++ heuristic looked for prebuilt
    /// variant libs under `build/install/lib/`. With install-local gone,
    /// NuttX C/C++ examples build in-tree via `add_subdirectory(nano-ros)`
    /// plus the Phase 139 NuttX integration shell; toolchain absence is
    /// the real skip signal and is surfaced where the QEMU /
    /// cross-compile build runs.
    fn skip_reason(self, _lang: Lang, _variant: Variant) -> Option<&'static str> {
        None
    }
}

// =============================================================================
// Binary dispatch
// =============================================================================

type BuildFn = fn() -> TestResult<&'static Path>;

struct BinaryPair {
    first_builder: BuildFn,
    second_builder: BuildFn,
}

fn binaries(platform: Platform, lang: Lang, variant: Variant) -> BinaryPair {
    // "first" = talker / server; "second" = listener / client.
    match (platform, lang, variant) {
        // FreeRTOS — Rust
        (Platform::Freertos, Lang::Rust, Variant::Pubsub) => BinaryPair {
            first_builder: freertos::build_freertos_talker,
            second_builder: freertos::build_freertos_listener,
        },
        (Platform::Freertos, Lang::Rust, Variant::Service) => BinaryPair {
            first_builder: freertos::build_freertos_service_server,
            second_builder: freertos::build_freertos_service_client,
        },
        (Platform::Freertos, Lang::Rust, Variant::Action) => BinaryPair {
            first_builder: freertos::build_freertos_action_server,
            second_builder: freertos::build_freertos_action_client,
        },
        // FreeRTOS — C
        (Platform::Freertos, Lang::C, Variant::Pubsub) => BinaryPair {
            first_builder: freertos::build_freertos_c_talker,
            second_builder: freertos::build_freertos_c_listener,
        },
        (Platform::Freertos, Lang::C, Variant::Service) => BinaryPair {
            first_builder: freertos::build_freertos_c_service_server,
            second_builder: freertos::build_freertos_c_service_client,
        },
        (Platform::Freertos, Lang::C, Variant::Action) => BinaryPair {
            first_builder: freertos::build_freertos_c_action_server,
            second_builder: freertos::build_freertos_c_action_client,
        },
        // FreeRTOS — C++
        (Platform::Freertos, Lang::Cpp, Variant::Pubsub) => BinaryPair {
            first_builder: freertos::build_freertos_cpp_talker,
            second_builder: freertos::build_freertos_cpp_listener,
        },
        (Platform::Freertos, Lang::Cpp, Variant::Service) => BinaryPair {
            first_builder: freertos::build_freertos_cpp_service_server,
            second_builder: freertos::build_freertos_cpp_service_client,
        },
        (Platform::Freertos, Lang::Cpp, Variant::Action) => BinaryPair {
            first_builder: freertos::build_freertos_cpp_action_server,
            second_builder: freertos::build_freertos_cpp_action_client,
        },

        // NuttX — Rust
        (Platform::Nuttx, Lang::Rust, Variant::Pubsub) => BinaryPair {
            first_builder: nuttx::build_nuttx_talker,
            second_builder: nuttx::build_nuttx_listener,
        },
        (Platform::Nuttx, Lang::Rust, Variant::Service) => BinaryPair {
            first_builder: nuttx::build_nuttx_service_server,
            second_builder: nuttx::build_nuttx_service_client,
        },
        (Platform::Nuttx, Lang::Rust, Variant::Action) => BinaryPair {
            first_builder: nuttx::build_nuttx_action_server,
            second_builder: nuttx::build_nuttx_action_client,
        },
        // NuttX — C
        (Platform::Nuttx, Lang::C, Variant::Pubsub) => BinaryPair {
            first_builder: nuttx::build_nuttx_c_talker,
            second_builder: nuttx::build_nuttx_c_listener,
        },
        (Platform::Nuttx, Lang::C, Variant::Service) => BinaryPair {
            first_builder: nuttx::build_nuttx_c_service_server,
            second_builder: nuttx::build_nuttx_c_service_client,
        },
        (Platform::Nuttx, Lang::C, Variant::Action) => BinaryPair {
            first_builder: nuttx::build_nuttx_c_action_server,
            second_builder: nuttx::build_nuttx_c_action_client,
        },
        // NuttX — C++ (all skipped, but wire up builders for completeness)
        (Platform::Nuttx, Lang::Cpp, Variant::Pubsub) => BinaryPair {
            first_builder: nuttx::build_nuttx_cpp_talker,
            second_builder: nuttx::build_nuttx_cpp_listener,
        },
        (Platform::Nuttx, Lang::Cpp, Variant::Service) => BinaryPair {
            first_builder: nuttx::build_nuttx_cpp_service_server,
            second_builder: nuttx::build_nuttx_cpp_service_client,
        },
        (Platform::Nuttx, Lang::Cpp, Variant::Action) => BinaryPair {
            first_builder: nuttx::build_nuttx_cpp_action_server,
            second_builder: nuttx::build_nuttx_cpp_action_client,
        },

        // ThreadX Linux — Rust
        (Platform::ThreadxLinux, Lang::Rust, Variant::Pubsub) => BinaryPair {
            first_builder: threadx_linux::build_threadx_talker,
            second_builder: threadx_linux::build_threadx_listener,
        },
        (Platform::ThreadxLinux, Lang::Rust, Variant::Service) => BinaryPair {
            first_builder: threadx_linux::build_threadx_service_server,
            second_builder: threadx_linux::build_threadx_service_client,
        },
        (Platform::ThreadxLinux, Lang::Rust, Variant::Action) => BinaryPair {
            first_builder: threadx_linux::build_threadx_action_server,
            second_builder: threadx_linux::build_threadx_action_client,
        },
        // ThreadX Linux — C
        (Platform::ThreadxLinux, Lang::C, Variant::Pubsub) => BinaryPair {
            first_builder: threadx_linux::build_threadx_c_talker,
            second_builder: threadx_linux::build_threadx_c_listener,
        },
        (Platform::ThreadxLinux, Lang::C, Variant::Service) => BinaryPair {
            first_builder: threadx_linux::build_threadx_c_service_server,
            second_builder: threadx_linux::build_threadx_c_service_client,
        },
        (Platform::ThreadxLinux, Lang::C, Variant::Action) => BinaryPair {
            first_builder: threadx_linux::build_threadx_c_action_server,
            second_builder: threadx_linux::build_threadx_c_action_client,
        },
        // ThreadX Linux — C++ (Action skipped, Pubsub/Service run)
        (Platform::ThreadxLinux, Lang::Cpp, Variant::Pubsub) => BinaryPair {
            first_builder: threadx_linux::build_threadx_cpp_talker,
            second_builder: threadx_linux::build_threadx_cpp_listener,
        },
        (Platform::ThreadxLinux, Lang::Cpp, Variant::Service) => BinaryPair {
            first_builder: threadx_linux::build_threadx_cpp_service_server,
            second_builder: threadx_linux::build_threadx_cpp_service_client,
        },
        (Platform::ThreadxLinux, Lang::Cpp, Variant::Action) => BinaryPair {
            first_builder: threadx_linux::build_threadx_cpp_action_server,
            second_builder: threadx_linux::build_threadx_cpp_action_client,
        },

        // ThreadX RISC-V — Rust
        (Platform::ThreadxRiscv64, Lang::Rust, Variant::Pubsub) => BinaryPair {
            first_builder: threadx_riscv64::build_threadx_rv64_talker,
            second_builder: threadx_riscv64::build_threadx_rv64_listener,
        },
        (Platform::ThreadxRiscv64, Lang::Rust, Variant::Service) => BinaryPair {
            first_builder: threadx_riscv64::build_threadx_rv64_service_server,
            second_builder: threadx_riscv64::build_threadx_rv64_service_client,
        },
        (Platform::ThreadxRiscv64, Lang::Rust, Variant::Action) => BinaryPair {
            first_builder: threadx_riscv64::build_threadx_rv64_action_server,
            second_builder: threadx_riscv64::build_threadx_rv64_action_client,
        },
        // ThreadX RISC-V — C
        (Platform::ThreadxRiscv64, Lang::C, Variant::Pubsub) => BinaryPair {
            first_builder: threadx_riscv64::build_rv64_c_talker,
            second_builder: threadx_riscv64::build_rv64_c_listener,
        },
        (Platform::ThreadxRiscv64, Lang::C, Variant::Service) => BinaryPair {
            first_builder: threadx_riscv64::build_rv64_c_service_server,
            second_builder: threadx_riscv64::build_rv64_c_service_client,
        },
        (Platform::ThreadxRiscv64, Lang::C, Variant::Action) => BinaryPair {
            first_builder: threadx_riscv64::build_rv64_c_action_server,
            second_builder: threadx_riscv64::build_rv64_c_action_client,
        },
        // ThreadX RISC-V — C++
        (Platform::ThreadxRiscv64, Lang::Cpp, Variant::Pubsub) => BinaryPair {
            first_builder: threadx_riscv64::build_rv64_cpp_talker,
            second_builder: threadx_riscv64::build_rv64_cpp_listener,
        },
        (Platform::ThreadxRiscv64, Lang::Cpp, Variant::Service) => BinaryPair {
            first_builder: threadx_riscv64::build_rv64_cpp_service_server,
            second_builder: threadx_riscv64::build_rv64_cpp_service_client,
        },
        (Platform::ThreadxRiscv64, Lang::Cpp, Variant::Action) => BinaryPair {
            first_builder: threadx_riscv64::build_rv64_cpp_action_server,
            second_builder: threadx_riscv64::build_rv64_cpp_action_client,
        },
    }
}

// =============================================================================
// Helpers shared by all three parametrised tests
// =============================================================================

/// Returns normally ONLY when this cell can actually run; otherwise it skips.
///
/// Two skip paths with different causes, one verdict:
/// - **Unsupported combination** (`skip_reason` returns `Some`): the example
///   for this (platform, lang, variant) tuple is not implemented upstream
///   (e.g., NuttX C++ blocked by libc, or Phase 69.7/77/69.8 follow-ups).
///   The intent has always been "the `#[ignore]` the original per-platform
///   tests used, expressed at runtime because rstest `#[values]` cannot
///   attach `#[ignore]` per case" — but the pre-1135 spelling was a printed
///   `[SKIP]` and a bare `return`, which reports PASS, and `#[ignore]` does
///   not. `skip_class!(capability, …)` is the spelling that actually matches
///   the intent.
/// - **Missing prerequisite** (`require_e2e` returns `Err`): SDK / env
///   var / toolchain missing. Per CLAUDE.md, this must not silently turn into
///   a false PASS either.
fn require_cell_runnable(platform: Platform, lang: Lang, variant: Variant) {
    // Issue 1135 — this was `maybe_skip(..) -> bool` and its three callers wrote
    // `if maybe_skip(..) { return; }`. `skip_reason` returns `None` for every
    // combination today, so the `true` arm was unreachable and the guard branch
    // dead — but the branch is the live defect's shape: the moment someone
    // declares a genuinely unsupported (platform, lang, variant), the printed
    // `[SKIP]` becomes a bare `return`, i.e. a PASS for a cell that never ran.
    // Skipping HERE, and returning nothing, means a future `Some(reason)` is
    // reported as a skip by construction.
    if let Some(reason) = platform.skip_reason(lang, variant) {
        nros_tests::skip_class!(
            capability,
            "{} {} {:?}: {}",
            platform,
            lang,
            variant,
            reason
        );
    }
    if let Err(why) = platform.require_e2e() {
        // The site's OWN text, unchanged — those messages already name the
        // remedy, and rewording here would put the remedy in two places.
        // `<platform>: <reason>` rather than `<platform> needs <reason>`: the
        // reasons are a mix of noun phrases ("arm-none-eabi-gcc not found") and
        // full sentences ("the NuttX kernel at … is a RiscV image, but this
        // lane needs Arm"), and only the colon reads correctly for both.
        nros_tests::skip!("{}: {}", platform, why);
    }
}

/// Build a (first, second) binary pair, panicking on build failure.
fn build_pair(platform: Platform, lang: Lang, variant: Variant) -> (&'static Path, &'static Path) {
    let pair = binaries(platform, lang, variant);
    let first = (pair.first_builder)().unwrap_or_else(|e| {
        panic!(
            "Failed to build first binary ({} {} {:?}): {:?}",
            platform, lang, variant, e
        )
    });
    let second = (pair.second_builder)().unwrap_or_else(|e| {
        panic!(
            "Failed to build second binary ({} {} {:?}): {:?}",
            platform, lang, variant, e
        )
    });
    (first, second)
}

/// Start a REQUEST/RESPONSE pair: the server first, the client only once the
/// server has actually announced itself.
///
/// The pub/sub shape starts its two instances in parallel on NuttX and MUST
/// keep doing so — the reason, from the `start_pair` helper this replaced: the
/// NuttX Rust binaries boot slowly, and giving the listener the usual 20 s
/// head-start expires its session before the talker finishes booting. That
/// reasoning is sound for PUB/SUB, where a subscriber joining late still
/// receives the next sample, so nothing is lost by racing. It was keyed on the
/// PLATFORM, though, so it also governed the request/response shapes.
///
/// A request/response client asks ONCE and gives up. Started alongside the
/// server, the NuttX C action client reaches `Sending goal` while the server's
/// queryable is not yet declared, and the deadline that expires is the app's
/// OWN — `Failed to send goal: -2` (`NROS_RET_TIMEOUT`), 3/3, solo, which no
/// test-side budget can move (issue 0867). Booted by hand with the client
/// started after the server's banner, the same two images complete
/// goal -> accept -> feedback -> result every time.
///
/// So the ordering is keyed on the SHAPE, which is what actually differs
/// between the two cases, and it waits for the banner instead of sleeping a
/// fixed guess at how long the banner takes.
fn start_server_then_client(
    platform: Platform,
    lang: Lang,
    server_bin: &Path,
    client_bin: &Path,
    server_name: &str,
    client_name: &str,
    ready_marker: &str,
) -> TestResult<(RtosProcess, String, RtosProcess)> {
    let mut server = platform.start_process(server_bin, 0, server_name, lang)?;
    let server_boot = server.collect_until(ready_marker, boot_budget(platform));
    ensure_ready(&server_boot, ready_marker, platform, &server.command_line());
    let client = platform.start_process(client_bin, 1, client_name, lang)?;
    Ok((server, server_boot, client))
}
/// Common readiness check: is the platform's "first" process (listener
/// or server) past its boot banner? Panics on every platform if the
/// banner is missing — boot failures are real regressions (either a
/// kernel/app integration issue or a zenoh / networking problem) and
/// must surface loudly.
/// How long a readiness banner may take to appear, by PLATFORM.
///
/// All three e2e shapes wait for a banner before doing anything else, and all
/// three waited a flat 30 s — a budget sized for a native process. On an
/// emulated platform that same wait has to cover a cold QEMU boot, the app's
/// own startup sleep and a zenoh session open. The talker-window comment below
/// already measured those at ">15 s before the first publish", so 30 s was
/// nearly no margin, and under host load it is this wait that expires.
///
/// It expires badly. `collect_until` returns `Err(Timeout)` only when NOTHING
/// was captured, so a slow boot is reported as `<no output collected>` and
/// `ensure_ready` then blames a missing banner — indistinguishable from a dead
/// image. The image is fine: run the same test serially and it passes.
///
/// Keyed on platform, because every term in that list — emulator, board, cold
/// boot — is a property of the platform and not of the language the node was
/// written in (issue 0891).
fn boot_budget(platform: Platform) -> Duration {
    match platform {
        // A native process: no emulator and no cold boot to absorb.
        Platform::ThreadxLinux => Duration::from_secs(30),
        _ => Duration::from_secs(90),
    }
}

/// How many delivered samples the pub/sub cell requires before it is satisfied.
///
/// Every talker example in the matrix runs a **1 Hz** timer (`TimerDuration::
/// from_millis(1000)` in the Rust ones, `nros_timer_init(..., 1000000000ULL,
/// ...)` in the C/C++ ones), so this number is also *how many seconds of
/// SESSION LIFE the cell observes* — the property issue 1013 says a 15 s
/// run-to-completion window did not have.
///
/// **60 is derived, not chosen.** zenoh-pico's client lease task
/// (`_zp_unicast_lease_task`, `src/transport/unicast/lease.c`) arms
/// `next_lease = lease` and, at the first expiry, consumes the `_received` flag
/// the handshake set; only the SECOND expiry with nothing received closes the
/// session. A session whose peer never speaks therefore dies at **2 x lease** —
/// issue 0906's 10 s lease lapsed at 19.5 s on the wire, measured.
///
/// The peer is `rmw_zenohd`, and its shipped
/// `DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5` announces `lease: 60000` with
/// `keep_alive: 2`, i.e. an idle router speaks every **30 s**. zenoh-pico takes
/// `min(peer_lease, Z_TRANSPORT_LEASE)`. So the lease values split cleanly:
///
/// * `L >= 30 s` — every `L`-wide check window contains a keep-alive; the
///   session holds indefinitely. The shipped 60 s is in this set.
/// * `2L < 30 s`, i.e. `L < 15 s` — the second window contains none and the
///   session closes at `2L`. Issue 0906's 10 s is in this set, and so is
///   zenoh-pico's own upstream default, which is what a regression here would
///   revert to.
///
/// **Corrected (issue 1056): the middle is not a clean split.** For
/// `15 s <= L < 30 s` the first close is neither `2L` nor never — it is the end
/// of the first `L`-window that no 30 s keep-alive lands in, which is a beat
/// between `L` and 30 and runs away as `L -> 30`: 45 s at `L = 15`, 80 s at 20,
/// **899 s at 29**, 9000 s at 29.9. The table at [`MAX_ROUTER_SESSIONS`] has the
/// numbers. So 60 samples is the frontier for the DETERMINISTIC half of the
/// broken set (`L <= 14 s`), not for all of it, and no affordable sample count
/// reaches the top of the band — 120 samples buys `L <= 18 s`, 180 buys
/// `L <= 22 s`. A shorter window would readmit part of what IS covered (a 40 s
/// window passes any `L >= 10 s`); a longer one buys leases nobody ships.
///
/// **The bound this does NOT cover, stated:** a defect whose first symptom is
/// beyond 60 s of session life stays invisible here — the shipped 60 s lease's
/// own lapse would be at 120 s, and a slow leak or a drift that needs minutes
/// is out of reach of any per-cell window. Nothing in the suite covers that
/// today; it wants a soak, not a bigger e2e budget.
const PUBSUB_MIN_SAMPLES: usize = 70;

/// How long the pub/sub cell will wait for [`PUBSUB_MIN_SAMPLES`] samples.
///
/// The wait starts when the talker is spawned, so it has to cover that image's
/// boot, its network wait and its session open BEFORE the first of the 60
/// seconds of publishing — which is what the old talker window measured at
/// ">15 s" on QEMU, and NuttX's cold arm-virt boot makes worse. Keyed on
/// platform for the same reason [`boot_budget`] is: every term in that list is
/// a property of the emulator and the board, not of the language.
fn pubsub_window(platform: Platform) -> Duration {
    // Every arm carries +10 s over its pre-1056 value, because this deadline and
    // `PUBSUB_MIN_SAMPLES` are COUPLED: the wait must cover the talker's boot
    // plus one second per sample, so raising the count to 70 without raising the
    // deadline would have spent 10 s of the headroom that absorbs a slow QEMU
    // boot, and paid for it in flakes rather than in wall clock. Headroom is
    // unchanged, which is the point.
    match platform {
        // Cold arm-virt boot + slirp; the pre-1013 cell already gave this
        // platform 3x the others.
        Platform::Nuttx => Duration::from_secs(160),
        // A native process — no emulator, no cold boot to absorb.
        Platform::ThreadxLinux => Duration::from_secs(100),
        _ => Duration::from_secs(130),
    }
}

/// The zenoh router's per-SESSION DEBUG line, and the only evidence a test on
/// this side of the link has that a peer re-dialled it.
///
/// Deliberately the transport line and not `"Accepted TCP connection"`: a
/// healthy FreeRTOS pair produces THREE accepts and two sessions, because the
/// first dial is abandoned before the handshake (measured: accept at t=0.0 with
/// no transport, then one transport each at t=2.0 and t=22.0). Counting accepts
/// would have to carry that boot artefact as slack, on every platform, forever.
/// Sessions are the thing being asserted anyway.
///
/// It is third-party text (RFC-0075: the router is whatever ROS ships), so
/// [`assert_no_session_churn`] treats "not one of these in the whole log" as a
/// FAILURE rather than as zero reconnects — a rename upstream must show up as a
/// red naming this constant, never as a cell that silently stops checking.
const ROUTER_SESSION_MARKER: &str = "New transport opened";

/// Router log filter for [`ROUTER_SESSION_MARKER`]. Scoped to the crate that
/// emits it — a whole-router `debug` writes ~46 KB per cell to say the same
/// thing.
const ROUTER_SESSION_LOG_FILTER: &str = "zenoh_transport=debug";

/// How many sessions the router may open during one pub/sub cell.
///
/// TWO is the invariant — one per node, held for the whole run. Measured on the
/// shipped 60 s lease: exactly 2, on all three FreeRTOS languages, every run.
/// The third slot is slack for ONE genuine re-open (a link that drops once, a
/// platform whose bring-up re-dials); a lapse on a TIMER cannot hide under it,
/// because a lease of `L` costs one re-open per node per `2L` and issue 0906's
/// 10 s lease measured FIVE in this cell's window.
///
/// # What this count actually covers, and what it does not (issues 1044, 1056)
///
/// This is a COUNT and the thing it stands in for is a RATE, so the band it
/// separates has to be derived rather than assumed. The derivation below
/// REPLACES the "each node re-dials every `2 x lease`" table that stood here:
/// that table predicted **8** sessions for issue 0906's 10 s lease where the
/// cell measured **5**, and the discrepancy was recorded without being chased.
/// Chasing it is what produced this.
///
/// **One node lapses, not two.** `_zp_unicast_lease_task`
/// (zenoh-pico `src/transport/unicast/lease.c`) wakes every `lease` ms and
/// closes only if `_received` was false for that whole window; any inbound frame
/// sets it. The LISTENER is fed a 1 Hz sample stream by the router, so its
/// windows always contain traffic and it never lapses. The TALKER hears nothing
/// back but keep-alives — our fork's own `config.h` says so: *"a pure publisher
/// hears nothing back and closes at 2 x Z_TRANSPORT_LEASE"*. So
/// `sessions = 2 + talker lapses`, and 0906's five is `2 + floor(60 / 20)`
/// exactly.
///
/// **The lapse period is `2L` only while `2L < 30 s`.** The check window is `L`
/// wide and `rmw_zenohd` speaks every 30 s (`lease: 60000, keep_alive: 2`), so
/// the session dies at the end of the first `L`-window that no keep-alive falls
/// into. That is a beat between `L` and 30, and it diverges as `L -> 30`:
///
/// | lease | first close | sessions in a 60 s talker life | verdict |
/// | ---: | ---: | ---: | --- |
/// | 10 s | 20 s | 5 (measured) | fails |
/// | 14 s | 28 s | 4 | fails |
/// | 15 s | 45 s | 3 | passes — inside the slack |
/// | 20 s | 80 s | 2 | passes |
/// | 29 s | 899 s | 2 | passes |
/// | 29.9 s | 8999 s | 2 | passes |
/// | >= 30 s | never | 2 | passes, correctly |
///
/// **So the covered band is `L <= 14 s`, and no AFFORDABLE window reaches 30 s.**
/// (Every `L < 30` closes eventually, so a long enough window would see it — at
/// `L = 29.9` that is two closes of 9000 s each, five hours per cell.)
/// Issue 1056 proposed doubling the cell to 120 s to cover all of `(0, 30_000)`;
/// the arithmetic does not support it. A 120 s talker life moves the band to
/// `L <= 18 s` and a 180 s one to `L <= 22 s` — 12 and 24 extra minutes across
/// the twelve cells, for leases nobody has ever shipped. What IS shipped
/// is 60 s (here) and 10 s (zenoh-pico's own default, and 0906's regression),
/// and the cell separates those with margin.
///
/// **Start skew is not the exposure.** Both nodes must participate for all
/// [`PUBSUB_MIN_SAMPLES`] samples, so each one's observed life is at least that
/// span whatever the skew; the skew lengthens the EARLIER node's life and leaves
/// the later one alone. Measured across nine stored router logs: opens 0.00 s to
/// 20.0 s apart, router span 60.6-82.2 s, later node alive ~60 s in every one,
/// and exactly 2 sessions in all nine.
///
/// **The slack of one is what costs the 15-18 s band.** Dropping
/// `MAX_ROUTER_SESSIONS` to 2 would buy `L <= 18 s` for no wall clock at all,
/// since it only needs ONE lapse. Not taken: the slack is there for a single
/// genuine re-dial, nine healthy runs is not enough evidence to retire it on all
/// four platforms, and turning this cell flaky is worse than the band it buys.
///
/// **Per-NODE counting would make the slack "one re-open each" and is still not
/// available**: the client zid is regenerated on every session open —
/// `zpico.c`'s `zpico_next_session_zid_counter()` mixes a monotonic counter and
/// the clock into it — so grouping the router log by zid reads as more NODES
/// rather than more sessions.
///
/// The gaps between consecutive opens are printed on both paths below, so a
/// human reading a run can see the period even where the count cannot assert on
/// it: evenly spaced opens are a lapse, one late outlier is a drop.
const MAX_ROUTER_SESSIONS: usize = 3;

/// The largest `Z_TRANSPORT_LEASE_MS` this cell can still fail, in seconds.
///
/// Derived at [`MAX_ROUTER_SESSIONS`] from the talker's ~60 s life, the router's
/// 30 s keep-alive cadence and the one-slot slack. Printed on every run so a
/// PASS says what it proved: leases above this are NOT covered here, and no
/// affordable window covers the top of the band.
const COVERED_LEASE_SECS: u32 = 14;

/// Turn on the router's accept log for this test process.
///
/// SAFETY / why an env var: `ZenohRouter` reads `ZENOHD_LOG` when it SPAWNS the
/// router, and there is no per-call switch. nextest runs each case as its own
/// process, so this write happens once, before this cell's router exists and
/// before any thread that could read the environment concurrently. An operator
/// value is left alone — `ZENOHD_LOG=trace` is someone debugging, and its log
/// is a superset of what this needs.
fn enable_router_session_log() {
    if std::env::var_os("ZENOHD_LOG").is_none() {
        // SAFETY: single-threaded, once, at the top of the test.
        unsafe { std::env::set_var("ZENOHD_LOG", ROUTER_SESSION_LOG_FILTER) };
    }
}

/// Seconds between consecutive `ROUTER_SESSION_MARKER` lines, rendered for a
/// human (issue 1044).
///
/// The count assertion below cannot separate "one node re-dialled on a timer"
/// from "a link dropped once" — see [`MAX_ROUTER_SESSIONS`] — but the SPACING
/// can: a lapse repeats every `2 x lease`, a drop happens once. This is a
/// DIAGNOSTIC and never an assertion: it parses `HH:MM:SS` out of the router's
/// third-party log lines, which is exactly the kind of text RFC-0075 says we do
/// not control, so a parse miss must cost information and not a verdict.
fn session_open_spacing(log: &str) -> String {
    let secs: Vec<f64> = log
        .lines()
        .filter(|l| l.contains(ROUTER_SESSION_MARKER))
        .filter_map(|l| {
            let t = l.split('T').nth(1)?;
            let mut parts = t.split(':');
            let h: f64 = parts.next()?.parse().ok()?;
            let m: f64 = parts.next()?.parse().ok()?;
            let rest = parts.next()?;
            let s: f64 = rest
                .get(..rest.find(|c: char| !c.is_ascii_digit() && c != '.')?)?
                .parse()
                .ok()?;
            Some(h * 3600.0 + m * 60.0 + s)
        })
        .collect();
    if secs.len() < 2 {
        return "n/a (fewer than two parsable timestamps)".to_string();
    }
    secs.windows(2)
        .map(|w| format!("{:.1}s", w[1] - w[0]))
        .collect::<Vec<_>>()
        .join(", ")
}

/// The pub/sub cell's SECOND question: did the two nodes hold ONE session each,
/// or did they re-dial the router on a timer?
///
/// Issue 1013 asked for a delivery window long enough to see issue 0906's 10 s
/// `Z_TRANSPORT_LEASE` fail. Measured on this tree, a 60 s delivery window does
/// NOT see it: rebuilt with the 10 s lease, FreeRTOS delivered **60 published /
/// 60 heard** on all three languages. The defects that turned a lapse into lost
/// messages (issues 0899, 0924, and the board's `LWIP_NETCONN_FULLDUPLEX`) have
/// since been fixed, so the reopen now completes in ~15 ms and a 1 Hz publisher
/// almost never has a sample in flight during one. "Almost never" is the whole
/// problem: a delivery assertion would catch that build a few runs in a
/// thousand, which is worse than not claiming to catch it.
///
/// What that build still does is re-handshake every `2 x lease`, exactly as 0906
/// measured on the wire — and the router logs each one. So this is 0906's own
/// acceptance ("ONE session for at least five lease periods, proven by a capture
/// showing no second handshake"), automated from the router's own log instead of
/// a packet capture. It is the half that actually fails on that build: **5
/// sessions against a limit of 3, 3 languages out of 3**, while delivery reads
/// perfect.
fn assert_no_session_churn(platform: Platform, lang: Lang, port: u16, window: Duration) {
    let log_path = nros_tests::fixtures::fixture_log_path(&format!("zenohd-{port}"));
    let log = std::fs::read_to_string(&log_path).unwrap_or_else(|e| {
        panic!(
            "{platform} {lang} pubsub E2E: cannot read the router log at {} ({e}).\n\
             This cell asserts on it, so an unreadable log is a failure, not a pass. \
             `enable_router_session_log` sets ZENOHD_LOG before the router starts; \
             if that no longer reaches `ZenohRouter`, this is where it shows.",
            log_path.display()
        )
    });
    let sessions = count_pattern(&log, ROUTER_SESSION_MARKER);
    // Say what this verdict covers, not just what it counted (issue 1056; the
    // same rule as issue 0445 one layer up — a check that does not report its
    // own scope cannot be caught having the wrong one). A PASS here means "no
    // lease at or below COVERED_LEASE_SECS", never "the lease is healthy": the
    // band above it is unreachable from a count, at any window length.
    eprintln!(
        "[{} {}] router sessions opened: {} (max {}); gaps between opens: {}; \
         this count can fail a lease of <= {} s, and nothing above it \
         (see MAX_ROUTER_SESSIONS)",
        platform,
        lang,
        sessions,
        MAX_ROUTER_SESSIONS,
        session_open_spacing(&log),
        COVERED_LEASE_SECS,
    );
    assert!(
        sessions >= 2,
        "{platform} {lang} pubsub E2E: the router log records fewer than two \
         `{ROUTER_SESSION_MARKER}` lines, yet both nodes demonstrably talked to it — so \
         this cell's session-churn check just measured nothing.\n\
         That line is third-party text (RFC-0075: the router is whatever ROS ships). \
         Either the filter `{ROUTER_SESSION_LOG_FILTER}` no longer selects it or this \
         zenoh renamed it; fix ROUTER_SESSION_MARKER / ROUTER_SESSION_LOG_FILTER in this \
         file. Log: {}",
        log_path.display(),
    );
    // Bound before the assertion rather than formatted inside it: a `format!`
    // among another format's arguments is `clippy::format_in_format_args`, and
    // the lane that catches it (`check test-targets`) runs on no merge-gating
    // event, so it landed on main and sat there.
    let session_lines = log
        .lines()
        .filter(|l| l.contains(ROUTER_SESSION_MARKER))
        .collect::<Vec<_>>()
        .join("\n");
    assert!(
        sessions <= MAX_ROUTER_SESSIONS,
        "{platform} {lang} pubsub E2E: the router opened {sessions} sessions in {window:?} \
         for TWO nodes — they are re-dialling on a timer, not holding one session each.\n\
         That is issue 0906's signature: zenoh-pico's lease task closes a session at the \
         end of the first `Z_TRANSPORT_LEASE`-wide window with nothing received, and \
         `rmw_zenohd` speaks only every 30 s (`lease: 60000, keep_alive: 2`), so a client \
         lease under 15 s lapses deterministically at 2 x lease (issue 1056 — above 15 s \
         it is a beat against the 30 s cadence and this count stops seeing it, which is \
         why a PASS here is scoped, not clean). Note delivery can look PERFECT while this \
         fails — the reopen \
         is fast now, which is exactly why the count is what this asserts on. Check \
         `Z_TRANSPORT_LEASE_MS` in `packages/rmw/zenoh/nros-zpico-build/src/lib.rs` \
         (60_000) and what the leaf actually BAKED — issue 1005: the staleness probe does \
         not watch that constant, so a stale image can carry an old value.\n\n\
         Sessions:\n{}\n\nGaps between opens: {} — evenly spaced means a lease \
         lapsing on a timer (the period is 2 x lease while 2 x lease < 30 s, and a \
         longer beat against the router's keep-alive above that); one outlier means \
         a single dropped link.",
        session_lines,
        session_open_spacing(&log),
    );
}

fn ensure_ready(output: &str, readiness_pattern: &str, platform: Platform, cmdline: &str) {
    if output.contains(readiness_pattern) {
        return;
    }
    // issue 0891 — say WHICH of the two happened. `collect_until` renders a
    // harness-level failure into the text it returns, and returns `Err` (hence
    // `<no output collected>`) only when the capture came back EMPTY. So an
    // empty capture means the wait expired with the image having printed
    // nothing yet — a slow boot and a dead image are the same string, and the
    // string reads like the dead one. Naming the difference here is what stops
    // the next reader diagnosing a runtime regression that is not there.
    let empty_capture = output.starts_with("<no output collected");
    let hint = if empty_capture {
        "\n\nThe capture is EMPTY, which means the WAIT expired before the image \
         printed anything — not that the image is dead. A cold QEMU boot under \
         host load reaches this. Confirm by running this test alone; if it \
         passes, the budget or the concurrency cap is what to look at \
         (boot_budget / [test-groups.qemu-nuttx]), not the image."
    } else {
        ""
    };
    panic!(
        "{} E2E failed — readiness pattern '{}' not observed.\nStarted with:\n  {}\nOutput so far (truncated):\n{}{}",
        platform,
        readiness_pattern,
        cmdline,
        &output[..output.len().min(2048)],
        hint
    );
}

// =============================================================================
// Parametrised E2E tests
// =============================================================================

/// End-to-end pub/sub across Rust, C, and C++ on all four RTOS
/// platforms. First node is the talker; second node is the listener.
///
/// We wait for the listener's readiness banner, then let BOTH nodes run until
/// the listener has heard [`PUBSUB_MIN_SAMPLES`] samples — 60 at 1 Hz, so 60
/// seconds of session life, the number derived at that constant. Neither node
/// is killed before the count is in.
///
/// The old shape asserted `received > 0` after killing the talker at 15 s
/// (issue 1013): it proved a sample could be delivered, never that the session
/// survived long enough to keep delivering them.
#[rstest]
fn test_rtos_pubsub_e2e(
    #[values(
        Platform::Freertos,
        Platform::Nuttx,
        Platform::ThreadxLinux,
        Platform::ThreadxRiscv64
    )]
    platform: Platform,
    #[values(Lang::Rust, Lang::C, Lang::Cpp)] lang: Lang,
) {
    require_cell_runnable(platform, lang, Variant::Pubsub);

    let (talker_bin, listener_bin) = build_pair(platform, lang, Variant::Pubsub);

    // Ask the router to record its TCP accepts BEFORE starting it — that log is
    // this cell's only window onto session CONTINUITY (see
    // `assert_no_session_churn`), and `ZenohRouter` decides whether to keep one
    // at spawn time.
    enable_router_session_log();
    let zenohd = platform
        .zenoh_router_start(Variant::Pubsub, lang)
        .expect("Failed to start zenohd");

    eprintln!(
        "[{} {}] pubsub: starting talker/listener...",
        platform, lang
    );
    // Subscriber (listener) before publisher (talker) — zenoh does not
    // buffer for unknown subscribers. On every platform except NuttX we
    // intentionally invert the naming: "first" = listener, "second" =
    // talker, matching the per-platform tests. NuttX boots the two in
    // parallel (see start_pair docstring).
    let (mut listener, mut talker) = match platform {
        Platform::Nuttx => {
            // Parallel launch.
            let listener = platform
                .start_process(listener_bin, 1, "nuttx-listener", lang)
                .expect("Failed to start listener");
            let talker = platform
                .start_process(talker_bin, 0, "nuttx-talker", lang)
                .expect("Failed to start talker");
            (listener, talker)
        }
        _ => {
            let listener = platform
                .start_process(listener_bin, 1, "rtos-listener", lang)
                .expect("Failed to start listener");
            std::thread::sleep(platform.stabilization_delay());
            let talker = platform
                .start_process(talker_bin, 0, "rtos-talker", lang)
                .expect("Failed to start talker");
            (listener, talker)
        }
    };

    // Listener boot check: NuttX needs a lenient readiness probe because
    // its Rust apps sometimes fail to register with NSH. Other platforms
    // want a hard fail to surface environment regressions.
    // #132 — the NuttX Rust listener is an `*_entry` image (the role crate is
    // lib-only since 212.L.1). Its board `run_entry` prints "nros entry ready"
    // after the session opens; the C examples print "Waiting for messages". Pick
    // the marker per lang so the readiness gate matches the actual image.
    let ready_marker = match (platform, lang) {
        // #131 / #132 — Rust `*_entry` images print "nros entry ready" from the
        // board `run_*` after the session opens; only the C examples print
        // "Waiting for messages". ThreadX-RV64 rust joins the nuttx case.
        (Platform::Nuttx, Lang::Rust) | (Platform::ThreadxRiscv64, Lang::Rust) => {
            "nros entry ready"
        }
        // issue #181 — the freertos rust `*-entry` images (the runnables since
        // 212.L made the role crates lib-only) print the entry-runtime banner,
        // not the C examples' "Waiting for messages". issue #194 — the
        // threadx-linux rust lane consumes the same `*-entry` image shape.
        (Platform::Freertos, Lang::Rust) | (Platform::ThreadxLinux, Lang::Rust) => {
            "Application setup complete"
        }
        _ => "Waiting for messages",
    };
    let listener_boot = listener.collect_until(ready_marker, boot_budget(platform));
    ensure_ready(
        &listener_boot,
        ready_marker,
        platform,
        &listener.command_line(),
    );

    // ⚠️ The talker is FREE-RUNNING and nothing here may cut it short. ⚠️
    //
    // Issue 1013 — what stood here was
    //
    //     let talker_out = talker.wait_for_output(15s).unwrap_or_default();
    //
    // and `wait_for_output` is a RUN-TO-COMPLETION wait: its own doc says "wait
    // for QEMU to produce output *and exit*", and at the deadline it
    // `kill_process_group`s the guest. Pointed at a 1 Hz publisher — a node
    // with no terminal state — the timeout silently became the talker's
    // LIFETIME. Measured: exactly 12 publishes, every run, all three languages,
    // whole cell in ~35 s. The assertion below was `received > 0`, so every
    // defect whose first symptom lands after the twelfth publish was invisible
    // AND the cell said PASS: a build carrying issue 0906's 10 s
    // `Z_TRANSPORT_LEASE` — a session that provably dies at ~20 s — passed it
    // 6 runs out of 6.
    //
    // The service and action cells never had this: they let the long-lived
    // server run and wait on the CLIENT's terminal line. Pub/sub has no
    // terminal line, so the wait is on a COUNT of delivered samples, and the
    // talker is killed only after the count is in.
    //
    // Both processes now run undrained for the length of the wait. That is
    // sound at these volumes and not by luck: an mps2 talker prints its boot
    // banner plus one ~40-byte line per second, so a 90 s wait leaves single-
    // digit kilobytes against a 64 KiB pipe buffer. A node that logs per-sample
    // at any real rate would need this drained in slices.
    let listener_window = pubsub_window(platform);
    let already_heard = count_pattern(&listener_boot, nros_tests::output::LISTENER_LOG_PREFIX);
    let final_out = listener.collect_until_count(
        nros_tests::output::LISTENER_LOG_PREFIX,
        PUBSUB_MIN_SAMPLES.saturating_sub(already_heard),
        listener_window,
    );
    let full_listener = format!("{}{}", listener_boot, final_out);

    // The talker has been publishing the whole time; scrape its transcript for
    // the log now that the listener's count is in. Bounded and NON-killing (the
    // kill is two lines down either way) — this is a read, not a wait.
    let talker_out = talker.collect_until_count(
        nros_tests::output::TALKER_PAYLOAD_PREFIX,
        PUBSUB_MIN_SAMPLES,
        Duration::from_secs(5),
    ); // Capture BEFORE the kills — `command_line` borrows the live process.
    let talker_cmd = talker.command_line();
    let listener_cmd = listener.command_line();

    talker.kill();
    listener.kill();

    eprintln!("Talker output:\n{}", talker_out);
    eprintln!("Listener output:\n{}", full_listener);

    // TALKER_PAYLOAD_PREFIX, not TALKER_LOG_PREFIX: only the opening payload
    // quote separates a real publish line from setup prose containing the word
    // "Publishing" (phase-295 W2).
    let published = count_pattern(&talker_out, nros_tests::output::TALKER_PAYLOAD_PREFIX);
    let received = count_pattern(&full_listener, nros_tests::output::LISTENER_LOG_PREFIX);
    // Published vs heard is the shape issue 0906 was measured in ("77
    // published, 19 heard"), so print both: a shortfall says the session
    // stopped delivering, not that the talker stopped publishing.
    eprintln!(
        "[{} {}] messages published: {}, received: {} (need {})",
        platform, lang, published, received, PUBSUB_MIN_SAMPLES
    );
    assert!(
        received >= PUBSUB_MIN_SAMPLES,
        "{} {} pubsub E2E failed — heard {} of the {} samples this cell requires \
         (talker printed {} publish lines) within {:?}.\n\
         {} samples at 1 Hz is {} SECONDS of session life: a shortfall means \
         delivery stopped part-way, which is what a session that expires looks \
         like from here (issue 0906/1013).\n\
         Router:   {} (listening on port {})\n\
         Talker:   {}\n\
         Listener: {}\n\
         issue 0877 — to reproduce by hand, paste those three lines VERBATIM. \
         The program in them is not necessarily the `qemu-system-arm` your \
         shell resolves: `qemu_system_arm_path` prefers the patched build under \
         `build/qemu/bin/`, whose LAN9118 model carries our RX flow-control \
         patch (issues 0830 / 0917). Substituting the system QEMU changes the \
         emulated NIC's receive path, which is the layer a no-delivery symptom \
         lives in.\n\
         Listener output:\n{}",
        platform,
        lang,
        received,
        PUBSUB_MIN_SAMPLES,
        published,
        listener_window,
        PUBSUB_MIN_SAMPLES,
        PUBSUB_MIN_SAMPLES,
        zenohd.launch_line(),
        zenohd.port(),
        talker_cmd,
        listener_cmd,
        full_listener,
    );
    // Delivery is only half of it — see this function's header for why a build
    // that lapses every 20 s can still deliver 60 of 60.
    assert_no_session_churn(platform, lang, zenohd.port(), listener_window);

    eprintln!(
        "[PASS] {} {} pubsub E2E: {} messages",
        platform, lang, received
    );
}

/// End-to-end service request/response. First node is the server;
/// second is the client. The client should receive at least three
/// responses (most examples issue four: 5+3, 10+20, 100+200, -5+10) and
/// the "All service calls completed" marker.
#[rstest]
fn test_rtos_service_e2e(
    #[values(
        Platform::Freertos,
        Platform::Nuttx,
        Platform::ThreadxLinux,
        Platform::ThreadxRiscv64
    )]
    platform: Platform,
    #[values(Lang::Rust, Lang::C, Lang::Cpp)] lang: Lang,
) {
    require_cell_runnable(platform, lang, Variant::Service);

    let (server_bin, client_bin) = build_pair(platform, lang, Variant::Service);

    let zenohd = platform
        .zenoh_router_start(Variant::Service, lang)
        .expect("Failed to start zenohd");

    eprintln!("[{} {}] service: starting server/client...", platform, lang);

    // Server boot check. #132 — the NuttX Rust server is an `*_entry` image
    // whose board `run_entry` prints "nros entry ready" (the role crate is
    // lib-only since 212.L.1, so there's no C-shaped "Waiting for service
    // requests" line). Pick the marker per lang.
    let server_ready_marker = match (platform, lang) {
        (Platform::Nuttx, Lang::Rust) | (Platform::ThreadxRiscv64, Lang::Rust) => {
            "nros entry ready"
        }
        // issue #181 — see the pubsub gate: freertos rust entries print the
        // entry-runtime banner. issue #194 — threadx-linux rust entries too.
        (Platform::Freertos, Lang::Rust) | (Platform::ThreadxLinux, Lang::Rust) => {
            "Application setup complete"
        }
        _ => nros_tests::output::SERVICE_SERVER_READY_MARKER,
    };
    // The comment that used to sit here said the client needs "the same boot
    // delay as the server so its first query doesn't race ahead of the server
    // queryable's declaration" — then excluded NuttX, which is exactly a
    // QEMU-cold-boot platform. `start_server_then_client` makes the ordering
    // real instead of approximating it with a sleep: the client is not spawned
    // until the server's banner has actually been seen (issue 0867).
    let (mut server, _server_boot, mut client) = start_server_then_client(
        platform,
        lang,
        server_bin,
        client_bin,
        "rtos-server",
        "rtos-client",
        server_ready_marker,
    )
    .expect("Failed to start server/client");

    // NuttX C service is slower: 4 nros_client_call round-trips over
    // QEMU slirp + zenoh-pico TCP are routinely in the 40–60 s range,
    // and the first call can time out if the server queryable isn't
    // fully registered yet (cold-host runs). Give extra headroom.
    let client_timeout = match platform {
        Platform::Nuttx => Duration::from_secs(180),
        _ => Duration::from_secs(60),
    };
    // Early-exit when the client logs its single `Result of add_two_ints: N`
    // line rather than blind-collecting the full timeout. On timeout
    // `wait_for_output_pattern` still returns whatever it read (it only errors
    // when nothing was captured), so this never collects less than a blind
    // wait — it just returns as soon as the run completes.
    let client_out =
        client.collect_until(nros_tests::output::SERVICE_RESULT_PREFIX, client_timeout);

    // issue 0877 — capture the launch lines while the processes are alive.
    let server_cmd = server.command_line();
    let client_cmd = client.command_line();

    server.kill();
    client.kill();

    eprintln!("Client output:\n{}", client_out);

    // W5: the client sends ONE request (2, 3) and logs one result line.
    let response_count = count_pattern(&client_out, nros_tests::output::SERVICE_RESULT_PREFIX);
    eprintln!("[{} {}] responses: {}", platform, lang, response_count);

    assert!(
        response_count >= 1,
        "{} {} service E2E failed — got {} responses (expected >= 1).\n\
         Router: {} (listening on port {})\n\
         Server: {}\n\
         Client: {}\n\
         issue 0877 — reproduce by pasting those lines VERBATIM; the QEMU in \
         them is the one this harness resolved, which is not necessarily the \
         one on your `$PATH`.",
        platform,
        lang,
        response_count,
        zenohd.launch_line(),
        zenohd.port(),
        server_cmd,
        client_cmd,
    );
    eprintln!(
        "[PASS] {} {} service E2E: {} responses",
        platform, lang, response_count
    );
}

/// End-to-end action (goal / feedback / result). First node is the
/// action server; second is the action client. We assert the client
/// saw "Goal accepted" plus the terminal `Result received: [...]` line
/// (phase-277 W5 demo wording).
// Phase 182.5 — action is the wall-clock critical path (90–270 s/cell ×
// retries). Keep **all three language bindings** (each exercises distinct
// goal/feedback/result serialization), but only on the platforms where action
// runs reliably and cheaply: ThreadX-Linux (host process / NSOS, ~seconds) and
// FreeRTOS-QEMU. The QEMU-heavy NuttX + ThreadX-RISCV64 action cells are
// dropped — they are QEMU-heavy (270 s/cell × retries) and add no unique
// binding coverage. (NuttX action itself is fixed: the C++ client's hang was a
// real `fflush(stdout)` deadlock against zenoh-pico's background threads on the
// libc stdout FILE* lock — found + fixed in 177.30; a manual 2-QEMU boot now
// runs goal→accept→feedback→result. It stays out of the CI matrix only because
// the QEMU round-trip is slow/expensive, not because it is broken.) pubsub +
// service keep all 4 platforms × 3 langs. Cpp/C action bindings remain covered
// on native (Cyclone) and zephyr (xrce/dds) e2e.
//
// STALE ABOVE, kept for the reasoning and corrected here: phase-240.5 /
// issue-0047 put the NuttX action cells BACK ("migrated + runtime-validated",
// see the `#[values]` note below), so the paragraph's "the QEMU-heavy NuttX +
// ThreadX-RISCV64 action cells are dropped" describes only ThreadX-RISCV64
// today. Left visible rather than deleted because its measurement — action is
// the wall-clock critical path — is still true and still worth knowing before
// anyone adds a platform here.
#[rstest]
fn test_rtos_action_e2e(
    // Phase 240.5 / issue-0047 — NuttX action transport (server + CALLBACK client)
    // is migrated + runtime-validated: the typed component server executes real
    // goals (accept → compute Fibonacci → complete_goal) and the callback client
    // (`bind_action_client` — set_callbacks + a poll-timer pump per RFC-0041)
    // receives goal-response + result.
    #[values(Platform::Freertos, Platform::Nuttx, Platform::ThreadxLinux)] platform: Platform,
    #[values(Lang::Rust, Lang::C, Lang::Cpp)] lang: Lang,
) {
    require_cell_runnable(platform, lang, Variant::Action);

    let (server_bin, client_bin) = build_pair(platform, lang, Variant::Action);

    let zenohd = platform
        .zenoh_router_start(Variant::Action, lang)
        .expect("Failed to start zenohd");

    eprintln!("[{} {}] action: starting server/client...", platform, lang);

    // #132 — NuttX Rust action server is an `*_entry` image; readiness = the
    // board's "nros entry ready" line (see the service e2e note).
    let action_ready_marker = match (platform, lang) {
        (Platform::Nuttx, Lang::Rust) | (Platform::ThreadxRiscv64, Lang::Rust) => {
            "nros entry ready"
        }
        // issue #181 — freertos rust entries print the entry-runtime banner.
        // issue #194 — threadx-linux rust entries too.
        (Platform::Freertos, Lang::Rust) | (Platform::ThreadxLinux, Lang::Rust) => {
            "Application setup complete"
        }
        _ => nros_tests::output::ACTION_SERVER_READY_MARKER,
    };
    let (mut server, server_boot, mut client) = start_server_then_client(
        platform,
        lang,
        server_bin,
        client_bin,
        "rtos-action-server",
        "rtos-action-client",
        action_ready_marker,
    )
    .expect("Failed to start server/client");

    // ⚠️ DO NOT SHRINK the FreeRTOS-C action window. ⚠️
    //
    // The full goal→accept→feedback→result chain is slow on FreeRTOS QEMU under
    // `-icount` + heavy `test-all` host load (two QEMU guests + zenohd competing
    // for the host CPU): boot + 5 s net wait + session open + goal round-trip +
    // feedback + result, far longer under a full parallel `test-all`. 90 s is
    // the empirical floor with margin. ThreadX-Linux is a host process (NSOS)
    // and finishes in seconds, so it uses the 60 s default.
    //
    // (NuttX/ThreadX-RISCV64 were dropped from the matrix in 182.5 — see the
    // header comment. The NuttX C++ client's hang was a real fflush(stdout)
    // deadlock, fixed in 177.30, not a timeout issue.)
    let client_timeout = match (platform, lang) {
        (Platform::Freertos, Lang::C) => Duration::from_secs(90),
        _ => Duration::from_secs(60),
    };
    // Early-exit on the terminal `Result received: [...]` line rather than
    // blind-collecting the full timeout. `wait_for_output_pattern` returns the
    // collected output on timeout too (errors only when nothing was read), so
    // a variant that never reaches the result falls back to exactly the old
    // blind-wait behaviour — never collecting less.
    let client_out = client.collect_until(nros_tests::output::ACTION_RESULT_PREFIX, client_timeout);

    let server_post = server
        .wait_for_output(Duration::from_secs(2))
        .unwrap_or_default();

    // issue 0877 — capture the launch lines while the processes are alive.
    let server_cmd = server.command_line();
    let client_cmd = client.command_line();

    server.kill();
    client.kill();

    eprintln!("Server boot:\n{}", server_boot);
    eprintln!("Server post-boot:\n{}", server_post);
    eprintln!("Client output:\n{}", client_out);

    let goal_accepted = client_out.contains("Goal accepted");
    let completed = client_out.contains(nros_tests::output::ACTION_RESULT_PREFIX);

    // Issue 0870 — a failure here is now ANSWERABLE from the output above, and
    // said so, because for weeks it was not: NuttX had no `printk` arm so every
    // shim diagnostic compiled away, and retries reported the cell FLAKY so
    // nobody read the one run that had anything to say.
    assert!(
        goal_accepted && completed,
        "{} {} action E2E failed: accepted={}, completed={}\n\
         \n\
         If the client never reached `Sending goal`, construction failed. Read \
         the client output above for `action client: <which> failed` and for \
         zenoh-pico's raw code (`zpico: z_declare_subscriber (ring) failed: <n>` \
         / `z_liveliness_declare_token failed: <n>`). THE QUESTION IT ANSWERS: \
         did ALL SIX network operations fail (5 liveliness declares + the \
         feedback subscriber -- the session's declare/TX path is dead), or ONLY \
         the subscriber (a subscriber-specific fault)? That distinction is \
         issue 0870's whole remaining unknown.\n\
         \n\
         Router: {} (listening on port {})\n\
         Server: {}\n\
         Client: {}\n\
         issue 0877 — reproduce by pasting those lines VERBATIM; the QEMU in \
         them is the one this harness resolved, which is not necessarily the \
         one on your `$PATH`.",
        platform,
        lang,
        goal_accepted,
        completed,
        zenohd.launch_line(),
        zenohd.port(),
        server_cmd,
        client_cmd
    );
    eprintln!(
        "[PASS] {} {} action E2E: accepted={}, completed={}",
        platform, lang, goal_accepted, completed
    );
}

// Keep the `TestError` import alive when no code path actually constructs
// one — the type is re-exported from `nros_tests::TestError` so future
// maintainers (and rust-analyzer) see where it comes from.
#[allow(dead_code)]
fn _unused_type_anchor() -> Option<TestError> {
    None
}
