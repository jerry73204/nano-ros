//! Zephyr process fixture for embedded testing
//!
//! Provides managed Zephyr processes for testing native_sim and QEMU targets.

use crate::{
    TestError, TestResult,
    process::{kill_process_group, set_new_process_group},
    project_root,
};
// phase-353 W2 — the content-aware staleness helpers moved to `fixtures::staleness`
// so the dep-info arm shares them; this module keeps using the same one.
use crate::fixtures::staleness::candidates_changed_content;
use std::{
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    time::{Duration, Instant},
};

/// Zephyr platform variants
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZephyrPlatform {
    /// Native simulator (x86_64, runs directly on host)
    NativeSim,
    /// QEMU ARM Cortex-M3 emulation (Stellaris LM3S6965)
    QemuArm,
    /// QEMU ARM Cortex-A9 emulation (Xilinx Zynq-7000) with the
    /// Xilinx GEM ethernet driver wired into the Zephyr native IP
    /// stack. Phase 92 — the platform real DDS-on-Zephyr deployments
    /// run on (Zynq, STM32-Eth, NXP-MAC all use the same Cortex-A9
    /// + Zephyr stack code path).
    QemuCortexA9,
    /// issue 0260 / phase-356 W3 — `qemu_cortex_a53/qemu_cortex_a53/smp`, the
    /// tree's only MULTI-CORE Zephyr target. It exists so a `core` dim can be
    /// verified as PLACEMENT (the tier ran on the cpu it asked for) rather than
    /// only as ACCEPTANCE (the kernel took the pin), which is all a
    /// uniprocessor image can show.
    QemuCortexA53Smp,
}

impl ZephyrPlatform {
    /// Get the west board specifier for this platform
    pub fn board_spec(&self) -> &'static str {
        match self {
            ZephyrPlatform::NativeSim => "native_sim/native/64",
            ZephyrPlatform::QemuArm => "qemu_cortex_m3",
            ZephyrPlatform::QemuCortexA9 => "qemu_cortex_a9",
            ZephyrPlatform::QemuCortexA53Smp => "qemu_cortex_a53/qemu_cortex_a53/smp",
        }
    }
}

/// phase-295 W5.a — parsed subset of a Zephyr `runners.yaml`.
///
/// Zephyr emits `<build_dir>/zephyr/runners.yaml` at the BUILD stage — it is
/// the framework's runner metadata, the same file `west flash` / `west build
/// -t run` consult to decide *how* an image is launched. Interpreting it here
/// (instead of hand-coding a launch line) keeps the RUN command the
/// framework's, per RFC-0051 §4. This is the generalization of the `west fvp
/// run` template (phase-215.D `scripts/west_commands/fvp.py`), which reads the
/// board's runner key from `CMakeCache.txt` and delegates accordingly.
///
/// The parse is deliberately a tiny hand-rolled reader (no YAML dep, matching
/// `fvp.py`'s flat-key `CMakeCache` parse) covering only the keys the harness
/// needs: the `flash-runner`, the `runners:` list, and the `config.exe_file`
/// / `config.elf_file` build outputs.
#[derive(Debug, Clone, Default)]
pub struct RunnersYaml {
    /// The `flash-runner:` value (`native`, `qemu`, `jlink`, `armfvp`, …).
    pub flash_runner: Option<String>,
    /// The `runners:` list (available runners the board declared).
    pub runners: Vec<String>,
    /// `config.exe_file` — the host executable a `native` runner runs
    /// directly (e.g. `zephyr.exe`).
    pub exe_file: Option<String>,
    /// `config.elf_file` — the ELF a `qemu`/hardware runner loads
    /// (e.g. `zephyr.elf`).
    pub elf_file: Option<String>,
    /// The directory the file lives in (`<build_dir>/zephyr`), used to
    /// resolve the relative `exe_file` / `elf_file` names.
    dir: PathBuf,
}

impl RunnersYaml {
    /// Parse `<zephyr_dir>/runners.yaml`. `zephyr_dir` is the `zephyr/`
    /// subdirectory of a Zephyr build directory (the dir the built
    /// `zephyr.exe` / `zephyr.elf` also live in). Returns `None` when the
    /// file is absent or unreadable so callers can fall back gracefully.
    pub fn from_zephyr_dir(zephyr_dir: &Path) -> Option<Self> {
        let text = std::fs::read_to_string(zephyr_dir.join("runners.yaml")).ok()?;
        let mut me = RunnersYaml {
            dir: zephyr_dir.to_path_buf(),
            ..Default::default()
        };
        // Section tracking: `runners:` list items are dash-prefixed and (in
        // Zephyr's emitter) sit at column 0; `config:` children are indented
        // `key: value`. Track the last top-level key as the active section.
        let mut section = String::new();
        for raw in text.lines() {
            let trimmed = raw.trim();
            if trimmed.is_empty() || trimmed.starts_with('#') {
                continue;
            }
            if let Some(item) = trimmed.strip_prefix("- ") {
                if section == "runners" {
                    me.runners.push(item.trim().to_string());
                }
                continue;
            }
            let indented = raw.starts_with(' ') || raw.starts_with('\t');
            let Some((key, val)) = trimmed.split_once(':') else {
                continue;
            };
            if !indented {
                section = key.trim().to_string();
                if key.trim() == "flash-runner" {
                    me.flash_runner = Some(val.trim().to_string());
                }
            } else if section == "config" {
                match key.trim() {
                    "exe_file" => me.exe_file = Some(val.trim().to_string()),
                    "elf_file" => me.elf_file = Some(val.trim().to_string()),
                    _ => {}
                }
            }
        }
        Some(me)
    }

    /// Convenience: parse `<build_dir>/zephyr/runners.yaml`.
    pub fn from_build_dir(build_dir: &Path) -> Option<Self> {
        Self::from_zephyr_dir(&build_dir.join("zephyr"))
    }

    /// Absolute path to the `native`-runner host executable, if declared.
    pub fn exe_path(&self) -> Option<PathBuf> {
        self.exe_file.as_ref().map(|f| self.dir.join(f))
    }

    /// Absolute path to the `qemu`/hardware-runner ELF, if declared.
    pub fn elf_path(&self) -> Option<PathBuf> {
        self.elf_file.as_ref().map(|f| self.dir.join(f))
    }
}

/// Managed Zephyr process for native_sim or QEMU
///
/// Starts a Zephyr application and captures output.
/// Automatically kills the process on drop.
///
/// # Example
///
/// ```ignore
/// use nros_tests::zephyr::{ZephyrProcess, ZephyrPlatform};
/// use std::path::Path;
/// use std::time::Duration;
///
/// let workspace = zephyr_workspace_path().unwrap();
/// let binary = workspace.join("build/zephyr/zephyr.exe");
/// let mut zephyr = ZephyrProcess::start(&binary, ZephyrPlatform::NativeSim).unwrap();
/// let output = zephyr.wait_for_output(Duration::from_secs(15)).unwrap();
/// ```
pub struct ZephyrProcess {
    handle: Child,
    platform: ZephyrPlatform,
    // Accumulated stdout+stderr, grown by the background readers spawned in
    // `start()`. `wait_for_pattern()` polls this buffer for a readiness
    // marker (e.g. "Waiting for messages"), replacing the old fixed
    // sleeps that couldn't keep up with parallel-load cold-boot
    // variance. `wait_for_output()` returns the final snapshot and
    // signals the reader to stop.
    output: std::sync::Arc<std::sync::Mutex<String>>,
    reader_done: std::sync::Arc<std::sync::atomic::AtomicBool>,
    // Joined via Drop when the process is killed; held here so the
    // threads outlive any `wait_for_pattern` / `wait_for_output` call.
    #[allow(dead_code)]
    reader_threads: Vec<std::thread::JoinHandle<()>>,
}

/// Atomic counter to ensure each Zephyr process gets a unique seed
static SEED_COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

fn spawn_output_readers(
    handle: &mut Child,
    output: std::sync::Arc<std::sync::Mutex<String>>,
    reader_done: std::sync::Arc<std::sync::atomic::AtomicBool>,
) -> TestResult<Vec<std::thread::JoinHandle<()>>> {
    let stdout = handle
        .stdout
        .take()
        .ok_or_else(|| TestError::ProcessFailed("No stdout on spawned process".to_string()))?;
    let stderr = handle
        .stderr
        .take()
        .ok_or_else(|| TestError::ProcessFailed("No stderr on spawned process".to_string()))?;

    let remaining = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(2));
    Ok(vec![
        spawn_stream_reader(
            stdout,
            output.clone(),
            reader_done.clone(),
            remaining.clone(),
        ),
        spawn_stream_reader(stderr, output, reader_done, remaining),
    ])
}

fn spawn_stream_reader<R>(
    mut stream: R,
    output: std::sync::Arc<std::sync::Mutex<String>>,
    reader_done: std::sync::Arc<std::sync::atomic::AtomicBool>,
    remaining: std::sync::Arc<std::sync::atomic::AtomicUsize>,
) -> std::thread::JoinHandle<()>
where
    R: std::io::Read + Send + 'static,
{
    std::thread::spawn(move || {
        let mut buf = [0u8; 4096];
        loop {
            match stream.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => {
                    let chunk = String::from_utf8_lossy(&buf[..n]);
                    if let Ok(mut guard) = output.lock() {
                        guard.push_str(&chunk);
                    }
                }
                Err(_) => break,
            }
        }
        if remaining.fetch_sub(1, std::sync::atomic::Ordering::AcqRel) == 1 {
            reader_done.store(true, std::sync::atomic::Ordering::Release);
        }
    })
}

impl ZephyrProcess {
    /// Start a Zephyr application
    ///
    /// # Arguments
    /// * `binary` - Path to the Zephyr executable (zephyr.exe for native_sim, zephyr.elf for QEMU)
    /// * `platform` - Target platform
    ///
    /// # Returns
    /// A managed Zephyr process
    pub fn start(binary: &Path, platform: ZephyrPlatform) -> TestResult<Self> {
        Self::start_inner(binary, platform, &[])
    }

    /// #166 / phase-286 W1 — start a native_sim image with a per-test router
    /// locator override, passed as `-testargs --nros-locator=<locator>`. The
    /// image's `nros_runtime_locator_override()` reads it (via
    /// `nsi_get_test_cmd_line_args`) and dials THIS locator instead of the
    /// build-time-baked port, so each test can run its own ephemeral zenohd and
    /// the zenoh e2e lanes need not serialize on a shared baked port.
    ///
    /// native_sim only: `-testargs` is a native-simulator feature; QEMU /
    /// hardware images ignore the override and keep their baked locator.
    pub fn start_with_locator(
        binary: &Path,
        platform: ZephyrPlatform,
        locator: &str,
    ) -> TestResult<Self> {
        Self::start_inner(binary, platform, &[format!("--nros-locator={locator}")])
    }

    fn start_inner(
        binary: &Path,
        platform: ZephyrPlatform,
        testargs: &[String],
    ) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Zephyr binary not found: {}",
                binary.display()
            )));
        }

        let mut handle = match platform {
            ZephyrPlatform::NativeSim => {
                // phase-295 W5.a — native_sim direct-exec IS the framework
                // convention: Zephyr's `native` runner (see the board's
                // `runners.yaml`, `flash-runner: native`) simply runs
                // `config.exe_file` as a host process — there is no emulator to
                // interpret. So "derive the launch from runner metadata" here
                // means honoring that file's `exe_file` under a confirmed
                // `native` runner, which we do below; running the built
                // `zephyr.exe` directly is the SANCTIONED form, not an
                // E1/E9 bypass. If `runners.yaml` is missing or names a
                // non-native runner we fall back to the passed binary so the
                // lane still runs (identical effect — `binary` already points
                // at `<build_dir>/zephyr/zephyr.exe`).
                let launch_bin = binary
                    .parent()
                    .and_then(RunnersYaml::from_zephyr_dir)
                    .filter(|r| r.flash_runner.as_deref() == Some("native"))
                    .and_then(|r| r.exe_path())
                    .filter(|p| p.exists())
                    .unwrap_or_else(|| binary.to_path_buf());

                // Each process needs a unique --seed to prevent ephemeral port conflicts
                // (the test entropy source produces identical random numbers without different seeds).
                // Use current time nanos as a random base; the atomic counter ensures
                // two processes spawned in the same test get different seeds.
                use std::time::{SystemTime, UNIX_EPOCH};
                let base = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .subsec_nanos();
                let offset = SEED_COUNTER.fetch_add(10000, std::sync::atomic::Ordering::Relaxed);
                let seed = base.wrapping_add(offset);
                let mut cmd = Command::new(&launch_bin);
                cmd.arg(format!("--seed={}", seed))
                    .stdout(Stdio::piped())
                    .stderr(Stdio::piped());
                // #166 — everything after `-testargs` goes to the native-sim
                // "test args" argv (bypassing native_sim's own option parser,
                // which would abort on an unregistered `--nros-locator`).
                if !testargs.is_empty() {
                    cmd.arg("-testargs");
                    cmd.args(testargs);
                }
                #[cfg(unix)]
                set_new_process_group(&mut cmd);
                cmd.spawn()?
            }
            ZephyrPlatform::QemuArm => {
                // phase-295 W5.a — the launch ELF is derived from the board's
                // `runners.yaml` (`config.elf_file` under a confirmed `qemu`
                // runner), mirroring the native branch above; falls back to the
                // passed binary when the metadata is absent.
                //
                // SANCTIONED BYPASS (E1/E9 exception, RFC-0051 §4): the
                // `-cpu/-machine` block below stays hand-rolled because Zephyr
                // does NOT record the QEMU machine flags in `runners.yaml`.
                // For an emulator board the flags live in
                // `boards/qemu/cortex_m3/board.cmake` (`QEMU_FLAGS_<ARCH> =
                // -cpu cortex-m3 -machine lm3s6965evb …`) and are assembled by
                // `cmake/emu/qemu.cmake` into the CMake `run` target — reachable
                // only via `west build -t run`, which triggers the build half
                // (an E1 violation, and a stale reconfigure can mask a museum
                // binary). So we read what `runners.yaml` DOES carry (the runner
                // kind + the ELF) and mirror the board.cmake machine line here.
                // A follow-up (phase doc W5.c) can relocate these flags into the
                // board crate's `NROS_BOARD_RUNNER`-adjacent metadata.
                let launch_bin = binary
                    .parent()
                    .and_then(RunnersYaml::from_zephyr_dir)
                    .filter(|r| r.runners.iter().any(|x| x == "qemu"))
                    .and_then(|r| r.elf_path())
                    .filter(|p| p.exists())
                    .unwrap_or_else(|| binary.to_path_buf());

                // QEMU ARM requires qemu-system-arm; Phase 143 routes
                // this through the patched build under `build/qemu/`
                // when present.
                let mut cmd = crate::qemu::qemu_system_arm_cmd();
                cmd.args([
                    "-cpu",
                    "cortex-m3",
                    "-machine",
                    "lm3s6965evb",
                    "-nographic",
                    "-kernel",
                ])
                .arg(&launch_bin)
                .stdout(Stdio::piped())
                .stderr(Stdio::piped());
                #[cfg(unix)]
                set_new_process_group(&mut cmd);
                cmd.spawn()?
            }
            ZephyrPlatform::QemuCortexA53Smp => {
                // Same shape as QemuArm above, and the same SANCTIONED BYPASS:
                // `runners.yaml` carries the runner kind and the ELF, never the
                // machine flags, so those are mirrored from
                // `boards/qemu/cortex_a53/board.cmake` here.
                //
                // ONE DELIBERATE DEVIATION from that board.cmake: it passes
                // `-nic tap,model=e1000,ifname=zeth`, which needs `/dev/net/tun`
                // and fails with "Operation not permitted" for an unprivileged
                // runner. User-mode networking gives the guest the same e1000
                // with no privileges; the guest is 10.0.2.15 and the host
                // 10.0.2.2, which is what the fixture's baked locator names.
                // `-smp cpus=2` is the whole point of this platform.
                let launch_bin = binary
                    .parent()
                    .and_then(RunnersYaml::from_zephyr_dir)
                    .filter(|r| r.runners.iter().any(|x| x == "qemu"))
                    .and_then(|r| r.elf_path())
                    .filter(|p| p.exists())
                    .unwrap_or_else(|| binary.to_path_buf());

                let mut cmd = Command::new("qemu-system-aarch64");
                cmd.args([
                    "-cpu",
                    "cortex-a53",
                    "-machine",
                    "virt,gic-version=3",
                    "-nographic",
                    "-smp",
                    "cpus=2",
                    "-nic",
                    "user,model=e1000",
                    "-kernel",
                ])
                .arg(&launch_bin)
                .stdout(Stdio::piped())
                .stderr(Stdio::piped());
                #[cfg(unix)]
                set_new_process_group(&mut cmd);
                cmd.spawn()?
            }
            ZephyrPlatform::QemuCortexA9 => {
                // For Cortex-A9 the test must call `start_qemu_a9_mcast`
                // instead so it can supply a virtual-L2 mcast group.
                // Calling the bare `start` API doesn't make sense here.
                return Err(TestError::ProcessFailed(String::from(
                    "ZephyrPlatform::QemuCortexA9 requires `start_qemu_a9_mcast` (Phase 92)",
                )));
            }
        };

        // Spawn background readers that accumulate stdout and stderr into
        // a shared buffer. Subsequent `wait_for_pattern()` calls poll
        // this buffer; `wait_for_output()` returns its final snapshot.
        let output = std::sync::Arc::new(std::sync::Mutex::new(String::new()));
        let reader_done = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let reader_threads =
            spawn_output_readers(&mut handle, output.clone(), reader_done.clone())?;

        Ok(Self {
            handle,
            platform,
            output,
            reader_done,
            reader_threads,
        })
    }

    /// Launch a Zephyr `qemu_cortex_a9` binary with QEMU's
    /// `-netdev socket,mcast=…` networking. Two instances launched
    /// with the same `mcast_addr:port` share a virtual L2 broadcast
    /// domain on the host (no `sudo`, no TAP), so SPDP/SEDP/ARP all
    /// flow between them. Phase 92.5 talker↔listener interop runs
    /// over this.
    ///
    /// `mac` must be unique per instance — Zephyr's GEM driver uses
    /// the DTS `local-mac-address`, but QEMU still wants its `-nic
    /// mac=` to match for ARP/IGMP plumbing on the host side.
    pub fn start_qemu_a9_mcast(
        binary: &Path,
        mcast_addr_port: &str,
        mac: &str,
    ) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Zephyr binary not found: {}",
                binary.display()
            )));
        }

        // Locate the SDK-bundled qemu-system-xilinx-aarch64 — same binary the
        // upstream zephyr-lang-rust samples/philosophers build uses via
        // `west build -t run`.
        //
        // issue 0334 — this used to hardcode one developer's absolute path,
        // pinning BOTH the SDK version (`zephyr-sdk-0.16.8`, owned by
        // scripts/zephyr/setup.sh) and the host tuple
        // (`x86_64-pokysdk-linux`, so it could not work on an aarch64 host
        // even with the SDK present). On any other machine it handed a
        // nonexistent path to `Command`, which fails with a bare ENOENT rather
        // than a diagnosable "SDK not found" — while the dtb lookup four lines
        // below already resolved properly.
        let qemu = match std::env::var("QEMU_BIN") {
            Ok(explicit) => explicit,
            Err(_) => sdk_qemu_xilinx_aarch64().ok_or_else(|| {
                TestError::BuildFailed(
                    "qemu-system-xilinx-aarch64 not found. Set QEMU_BIN, or install the \
                     Zephyr SDK (`just zephyr setup`) so it can be found under \
                     $ZEPHYR_SDK_INSTALL_DIR or scripts/zephyr/sdk/zephyr-sdk-*."
                        .to_string(),
                )
            })?,
        };
        let dtb = std::env::var("ZEPHYR_FDT_ZYNQ7000S").unwrap_or_else(|_| {
            let ws = zephyr_workspace_path()
                .map(|p| p.join("zephyr/boards/qemu/cortex_a9/fdt-zynq7000s.dtb"))
                .unwrap_or_default();
            ws.to_string_lossy().into_owned()
        });

        let mut cmd = Command::new(&qemu);
        cmd.args([
            "-nographic",
            "-machine",
            "arm-generic-fdt-7series",
            "-dtb",
            dtb.as_str(),
            "-nic",
            &format!("socket,model=cadence_gem,mcast={mcast_addr_port},mac={mac}"),
            "-chardev",
            "stdio,id=con,mux=on",
            "-serial",
            "chardev:con",
            "-mon",
            "chardev=con,mode=readline",
            "-device",
        ])
        .arg(format!(
            "loader,file={},cpu-num=0",
            binary.to_string_lossy()
        ))
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        let mut handle = cmd.spawn()?;

        let output = std::sync::Arc::new(std::sync::Mutex::new(String::new()));
        let reader_done = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let reader_threads =
            spawn_output_readers(&mut handle, output.clone(), reader_done.clone())?;

        Ok(Self {
            handle,
            platform: ZephyrPlatform::QemuCortexA9,
            output,
            reader_done,
            reader_threads,
        })
    }

    /// Wait until `pattern` appears in the process's accumulated stdout,
    /// or until `timeout` elapses.
    ///
    /// Returns the output seen so far (whether or not the pattern
    /// matched), so callers can inspect it either way.
    ///
    /// Unlike `wait_for_output`, this does NOT stop the reader thread —
    /// subsequent calls to `wait_for_pattern` or `wait_for_output` keep
    /// seeing new output as it arrives.
    pub fn wait_for_pattern(&self, pattern: &str, timeout: Duration) -> String {
        let deadline = Instant::now() + timeout;
        loop {
            {
                let guard = self.output.lock().expect("output mutex poisoned");
                if guard.contains(pattern) {
                    return guard.clone();
                }
                if self.reader_done.load(std::sync::atomic::Ordering::Acquire) {
                    // Process exited; no more output is coming.
                    return guard.clone();
                }
            }
            if Instant::now() >= deadline {
                return self.output.lock().expect("output mutex poisoned").clone();
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    /// Get the platform this process is running on
    pub fn platform(&self) -> ZephyrPlatform {
        self.platform
    }

    /// Wait for output with timeout, then KILL the image.
    ///
    /// Collects stdout from the process. Since Zephyr native_sim processes
    /// typically output everything quickly and then wait indefinitely,
    /// this uses a thread to avoid blocking on read().
    ///
    /// **A terminal drain, not a wait-for-readiness — issue 1026.** The only
    /// early-outs are four hard-coded terminal markers
    /// (`SUCCESS`/`COMPLETE`/`session error`/`Failed to create context`), so an
    /// image that keeps spinning always runs to the deadline, and the deadline
    /// kills it *unconditionally* — every return path here passes through
    /// `kill_process_group`. Aimed at a `spin_blocking` node the timeout is
    /// that node's LIFETIME, and whatever it would have printed afterwards is
    /// unobservable by construction (issue 1013 measured the cost of exactly
    /// this on the RTOS pubsub cell).
    ///
    /// Use [`Self::wait_for_pattern`] when the test is waiting for a
    /// CONDITION: it returns at the marker, returns the output either way, and
    /// leaves the image running for the next wait.
    ///
    /// # Arguments
    /// * `timeout` - Maximum time to wait
    ///
    /// # Returns
    /// The collected stdout as a string
    pub fn wait_for_output(&mut self, timeout: Duration) -> TestResult<String> {
        let start = Instant::now();
        let deadline = start + timeout;
        let mut markers_seen = false;

        while Instant::now() < deadline {
            // Check for completion/error markers in the accumulated
            // output. This short-circuits the wait when a Zephyr app
            // emits a known terminal string.
            {
                let guard = self.output.lock().expect("output mutex poisoned");
                if guard.contains("Failed to create context")
                    || guard.contains("session error")
                    || guard.contains("SUCCESS")
                    || guard.contains("COMPLETE")
                {
                    markers_seen = true;
                    break;
                }
            }

            // If the reader has signalled EOF the process ended; no
            // more output is coming.
            if self.reader_done.load(std::sync::atomic::Ordering::Acquire) {
                break;
            }
            if let Ok(Some(_)) = self.handle.try_wait() {
                // Process exited — give the reader a moment to drain
                // the last buffered bytes, then return.
                std::thread::sleep(Duration::from_millis(100));
                break;
            }

            std::thread::sleep(Duration::from_millis(50));
        }

        // Kill the process if still running. The reader thread exits
        // on stdout EOF, which the kill will cause.
        kill_process_group(&mut self.handle);
        let _ = markers_seen;

        let guard = self.output.lock().expect("output mutex poisoned");
        if guard.is_empty() {
            Err(TestError::Timeout)
        } else {
            Ok(guard.clone())
        }
    }

    /// Kill the Zephyr process
    pub fn kill(&mut self) {
        kill_process_group(&mut self.handle);
    }

    /// Check if process is still running
    pub fn is_running(&mut self) -> bool {
        matches!(self.handle.try_wait(), Ok(None))
    }
}

impl Drop for ZephyrProcess {
    fn drop(&mut self) {
        kill_process_group(&mut self.handle);
        for thread in std::mem::take(&mut self.reader_threads) {
            let _ = thread.join();
        }
    }
}

// =============================================================================
// Zephyr Availability Checks
// =============================================================================

/// Resolve the Zephyr-SDK-bundled `qemu-system-xilinx-aarch64` (issue 0334).
///
/// Nothing here is pinned: the SDK version and the host tuple are both GLOBBED,
/// because both were previously hardcoded and both are owned elsewhere — the
/// version by `scripts/zephyr/setup.sh`, the tuple by whatever host you are on.
///
/// Search order mirrors the other resolvers in this file:
/// 1. `ZEPHYR_SDK_INSTALL_DIR` (the SDK's own variable, set by `west`/`setup.sh`)
/// 2. `<project_root>/scripts/zephyr/sdk/zephyr-sdk-*`
///
/// Returns `None` when absent so the caller can report a diagnosable error
/// rather than handing a nonexistent path to `Command`.
fn sdk_qemu_xilinx_aarch64() -> Option<String> {
    const QEMU: &str = "qemu-system-xilinx-aarch64";

    // Inside one SDK root, find `sysroots/<host-tuple>/usr/bin/<QEMU>`.
    fn in_sdk_root(root: &Path) -> Option<String> {
        let entries = std::fs::read_dir(root.join("sysroots")).ok()?;
        for entry in entries.flatten() {
            let candidate = entry.path().join("usr/bin").join(QEMU);
            if candidate.is_file() {
                return Some(candidate.to_string_lossy().into_owned());
            }
        }
        None
    }

    if let Ok(dir) = std::env::var("ZEPHYR_SDK_INSTALL_DIR")
        && let Some(found) = in_sdk_root(Path::new(&dir))
    {
        return Some(found);
    }

    // Newest-first so a host carrying several SDKs picks the latest, matching
    // what `just zephyr setup` most recently provisioned.
    let sdk_parent = crate::project_root().join("scripts/zephyr/sdk");
    let mut roots: Vec<PathBuf> = std::fs::read_dir(&sdk_parent)
        .ok()?
        .flatten()
        .map(|e| e.path())
        .filter(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.starts_with("zephyr-sdk-"))
        })
        .collect();
    roots.sort();
    roots.iter().rev().find_map(|r| in_sdk_root(r))
}

/// Get the path to the Zephyr workspace
///
/// Checks in order:
/// 1. `ZEPHYR_NANO_ROS` environment variable
/// 2. `zephyr-workspace` symlink in project root
/// 3. Sibling workspace `../nano-ros-workspace/`
///
/// # Returns
/// Path to the workspace, or None if not found
pub fn zephyr_workspace_path() -> Option<PathBuf> {
    // 1. Environment variable
    if let Ok(path) = std::env::var("ZEPHYR_NANO_ROS") {
        let path = PathBuf::from(path);
        if path.exists() {
            return Some(path);
        }
    }

    let root = project_root();

    // 2. zephyr-workspace symlink
    let symlink = root.join("zephyr-workspace");
    if (symlink.is_symlink() || symlink.is_dir())
        && let Ok(resolved) = std::fs::canonicalize(&symlink)
        && resolved.exists()
    {
        return Some(resolved);
    }

    // 3. Sibling workspace
    let sibling = root.parent()?.join("nano-ros-workspace");
    if sibling.exists() {
        return Some(sibling);
    }

    None
}

fn zephyr_build_root(workspace: &Path) -> PathBuf {
    if let Some(path) = std::env::var_os("NROS_ZEPHYR_BUILD_ROOT") {
        return PathBuf::from(path);
    }
    if workspace_is_writable(workspace) {
        workspace.to_path_buf()
    } else {
        crate::build_dir(crate::kind::ZEPHYR_WORKSPACE_BUILDS, &[])
    }
}

fn workspace_is_writable(path: &Path) -> bool {
    path.metadata()
        .map(|m| !m.permissions().readonly())
        .unwrap_or(false)
}

/// Check if west command is available
pub fn is_west_available() -> bool {
    Command::new("west")
        .arg("--version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Check if Zephyr workspace is configured
pub fn is_zephyr_workspace_available() -> bool {
    zephyr_workspace_path()
        .map(|p| p.join("zephyr").exists())
        .unwrap_or(false)
}

/// Check if all Zephyr prerequisites are available
///
/// Checks:
/// - west command available
/// - Zephyr workspace configured
///
/// Networking on native_sim uses NSOS (host loopback), so no TAP/bridge setup
/// is required.
pub fn is_zephyr_available() -> bool {
    is_west_available() && is_zephyr_workspace_available()
}

/// Skip test if Zephyr is not available
///
/// Returns `false` if Zephyr prerequisites are not met, printing a skip message.
/// Returns `true` if Zephyr is available and the test should proceed.
pub fn require_zephyr() -> bool {
    if !is_west_available() {
        eprintln!("Skipping test: west not found");
        return false;
    }
    if !is_zephyr_workspace_available() {
        eprintln!("Skipping test: Zephyr workspace not found");
        eprintln!("  Run: ./scripts/zephyr/setup.sh");
        return false;
    }
    true
}

// =============================================================================
// Zephyr Fixture Helpers
// =============================================================================

/// Get the build directory name for an example
///
/// Returns a unique build directory to allow simultaneous builds of talker and listener.
/// Phase 168.6.B — alias → (lang, case, rmw) decoder.
///
/// Legacy alias names (kept for caller-source stability) are mapped
/// to the collapsed Phase 168 shape. The build directory and
/// example path both follow `build-<lang>-<case>-<rmw>`
/// / `examples/zephyr/<lang>/<case>` respectively.
///
/// issue 1016 — there was a fourth field, a free-form BOARD SUFFIX, and every
/// arm that used it (the eighteen `zephyr-dds-*-a9` aliases) is gone. It had to
/// go with them: a free-form suffix is a way to spell a build-dir name the
/// manifest cannot explain, and `require_west_leaf_in_lane` fails OPEN on
/// exactly those. A second board's leaves reintroduce it together with their
/// `[[fixture]] builder = "west"` rows, which is the pairing
/// `check-west-leaf-vocabulary.py` requires.
fn decode_alias(example_name: &str) -> Option<(&'static str, &'static str, &'static str)> {
    // (lang, case, rmw)
    Some(match example_name {
        // Rust zenoh
        "zephyr-rs-talker" | "rs-talker" => ("rust", "talker", "zenoh"),
        "zephyr-rs-listener" | "rs-listener" => ("rust", "listener", "zenoh"),
        "zephyr-rs-action-server" | "rs-action-server" => ("rust", "action-server", "zenoh"),
        "zephyr-rs-action-client" | "rs-action-client" => ("rust", "action-client", "zenoh"),
        "zephyr-rs-service-server" | "rs-service-server" => ("rust", "service-server", "zenoh"),
        "zephyr-rs-service-client" | "rs-service-client" => ("rust", "service-client", "zenoh"),
        // C++ zenoh
        "zephyr-cpp-talker" | "cpp-talker" => ("cpp", "talker", "zenoh"),
        "zephyr-cpp-listener" | "cpp-listener" => ("cpp", "listener", "zenoh"),
        "zephyr-cpp-service-server" | "cpp-service-server" => ("cpp", "service-server", "zenoh"),
        "zephyr-cpp-service-client" | "cpp-service-client" => ("cpp", "service-client", "zenoh"),
        "zephyr-cpp-action-server" | "cpp-action-server" => ("cpp", "action-server", "zenoh"),
        "zephyr-cpp-action-client" | "cpp-action-client" => ("cpp", "action-client", "zenoh"),
        // C zenoh
        "zephyr-c-talker" | "c-talker" => ("c", "talker", "zenoh"),
        "zephyr-c-listener" | "c-listener" => ("c", "listener", "zenoh"),
        "zephyr-c-service-server" | "c-service-server" => ("c", "service-server", "zenoh"),
        "zephyr-c-service-client" | "c-service-client" => ("c", "service-client", "zenoh"),
        "zephyr-c-action-server" | "c-action-server" => ("c", "action-server", "zenoh"),
        "zephyr-c-action-client" | "c-action-client" => ("c", "action-client", "zenoh"),
        // XRCE Rust
        "zephyr-xrce-rs-talker" | "xrce-rs-talker" => ("rust", "talker", "xrce"),
        "zephyr-xrce-rs-listener" | "xrce-rs-listener" => ("rust", "listener", "xrce"),
        "zephyr-xrce-rs-service-server" | "xrce-rs-service-server" => {
            ("rust", "service-server", "xrce")
        }
        "zephyr-xrce-rs-service-client" | "xrce-rs-service-client" => {
            ("rust", "service-client", "xrce")
        }
        "zephyr-xrce-rs-action-server" | "xrce-rs-action-server" => {
            ("rust", "action-server", "xrce")
        }
        "zephyr-xrce-rs-action-client" | "xrce-rs-action-client" => {
            ("rust", "action-client", "xrce")
        }
        // XRCE C
        "zephyr-xrce-c-talker" | "xrce-c-talker" => ("c", "talker", "xrce"),
        "zephyr-xrce-c-listener" | "xrce-c-listener" => ("c", "listener", "xrce"),
        "zephyr-xrce-c-service-server" | "xrce-c-service-server" => ("c", "service-server", "xrce"),
        "zephyr-xrce-c-service-client" | "xrce-c-service-client" => ("c", "service-client", "xrce"),
        "zephyr-xrce-c-action-server" | "xrce-c-action-server" => ("c", "action-server", "xrce"),
        "zephyr-xrce-c-action-client" | "xrce-c-action-client" => ("c", "action-client", "xrce"),
        // XRCE C++
        "zephyr-xrce-cpp-talker" | "xrce-cpp-talker" => ("cpp", "talker", "xrce"),
        "zephyr-xrce-cpp-listener" | "xrce-cpp-listener" => ("cpp", "listener", "xrce"),
        "zephyr-xrce-cpp-service-server" | "xrce-cpp-service-server" => {
            ("cpp", "service-server", "xrce")
        }
        "zephyr-xrce-cpp-service-client" | "xrce-cpp-service-client" => {
            ("cpp", "service-client", "xrce")
        }
        "zephyr-xrce-cpp-action-server" | "xrce-cpp-action-server" => {
            ("cpp", "action-server", "xrce")
        }
        "zephyr-xrce-cpp-action-client" | "xrce-cpp-action-client" => {
            ("cpp", "action-client", "xrce")
        }
        // Cyclone DDS — C / C++ today; Rust path lands once Phase 169.5
        // ships `nros-rmw-cyclonedds-sys`. Legacy `zephyr-dds-*` aliases
        // map to cyclonedds for source compatibility after Phase 169.4.
        "zephyr-dds-cpp-talker" => ("cpp", "talker", "cyclonedds"),
        "zephyr-dds-cpp-listener" => ("cpp", "listener", "cyclonedds"),
        "zephyr-dds-cpp-service-server" => ("cpp", "service-server", "cyclonedds"),
        "zephyr-dds-cpp-service-client" => ("cpp", "service-client", "cyclonedds"),
        "zephyr-dds-cpp-action-server" => ("cpp", "action-server", "cyclonedds"),
        "zephyr-dds-cpp-action-client" => ("cpp", "action-client", "cyclonedds"),
        "zephyr-dds-c-talker" => ("c", "talker", "cyclonedds"),
        "zephyr-dds-c-listener" => ("c", "listener", "cyclonedds"),
        "zephyr-dds-c-service-server" => ("c", "service-server", "cyclonedds"),
        "zephyr-dds-c-service-client" => ("c", "service-client", "cyclonedds"),
        "zephyr-dds-c-action-server" => ("c", "action-server", "cyclonedds"),
        "zephyr-dds-c-action-client" => ("c", "action-client", "cyclonedds"),
        // DDS Rust legacy aliases — Phase 169.4 retired the old Rust DDS
        // backend. These
        // map to cyclonedds for now; the build dir + example path
        // resolve correctly only once Phase 169.5's `nros-rmw-cyclonedds-sys`
        // lands. Tests that invoke these aliases without the shim get
        // a clean "example not found" failure.
        "zephyr-dds-rs-talker" | "dds-rs-talker" => ("rust", "talker", "cyclonedds"),
        "zephyr-dds-rs-listener" | "dds-rs-listener" => ("rust", "listener", "cyclonedds"),
        "zephyr-dds-rs-service-server" | "dds-rs-service-server" => {
            ("rust", "service-server", "cyclonedds")
        }
        "zephyr-dds-rs-service-client" | "dds-rs-service-client" => {
            ("rust", "service-client", "cyclonedds")
        }
        "zephyr-dds-rs-action-server" | "dds-rs-action-server" => {
            ("rust", "action-server", "cyclonedds")
        }
        "zephyr-dds-rs-action-client" | "dds-rs-action-client" => {
            ("rust", "action-client", "cyclonedds")
        }
        // `service-client-async` zephyr/rust example dropped 2026-06-02 per
        // Phase 212.M-F.5 — pending async-`Node` trait decision.
        //
        // issue 1016 — the eighteen `zephyr-dds-*-a9` arms are gone. No test
        // named one, no `[[fixture]]` row modelled one, and no recipe built
        // one, so each was a build-dir name (`build-<lang>-<case>-cyclonedds-a9`)
        // this resolver could produce and `west-leaves` could not explain.
        // `require_west_leaf_in_lane` fails OPEN on a name it cannot find —
        // "run it" — so an alias like that is by construction a leaf the run
        // demands at every lane and no lane builds, which is exactly the
        // STALE-verdict shape issue 1016 reported. The vocabulary is now a
        // subset of the manifest's, and `check-west-leaf-vocabulary.py` keeps
        // it one.
        _ => return None,
    })
}

/// Build-dir slot for the alias: `build-<lang>-<case>-<rmw>`.
///
/// Every name this can return must be one `fixtures-manifest.py west-leaves`
/// models, or `require_west_leaf_in_lane` cannot decide the leaf's lane and
/// falls open — see [`decode_alias`]'s closing note and
/// `scripts/check-west-leaf-vocabulary.py`.
///
/// issue 0539 — this used to map `rust` -> `rs` here, and
/// `fixtures-manifest.py::west_lang_tag` carried the same mapping on the BUILD
/// side. Two producers of one spelling; both retired together, or the build and
/// this resolver would name different directories.
fn build_dir_for_example(example_name: &str) -> String {
    if let Some((lang, case, rmw)) = decode_alias(example_name) {
        format!("build-{lang}-{case}-{rmw}")
    } else {
        // issue 1016 — this used to fall back to the bare `"build"`, a name
        // `west-leaves` does not model. A name the manifest does not model is
        // built by NO lane and skippable by NO lane, so the fallback could only
        // ever produce the missing-or-stale verdict this issue is about, with
        // the alias typo — the real fault — nowhere in it.
        panic!(
            "unknown Zephyr example alias {example_name:?} — `decode_alias` has no arm for \n\
             it, so there is no west build directory and no manifest row to place it in a \n\
             lane. Add the alias AND its `[[fixture]] builder = \"west\"` row together \n\
             (`check-west-leaf-vocabulary.py` requires the pair)."
        );
    }
}

/// Convert example name to the actual path under examples/
///
/// Handles both legacy names (zephyr-rs-talker) and new names (rs-talker).
/// Returns path relative to examples/ directory.
fn example_path_for_name(example_name: &str) -> String {
    if let Some((lang, case, _rmw)) = decode_alias(example_name) {
        return format!("zephyr/{lang}/{case}");
    }
    example_name.to_string()
}

/// Phase 168.6.B — `-DCONF_FILE="..."` argument value for a
/// collapsed alias. Returns `None` for non-collapsed names so
/// callers leave the west default (single `prj.conf`) alone.
fn conf_files_for_example(example_name: &str) -> Option<String> {
    decode_alias(example_name).map(|(_lang, _case, rmw)| format!("prj.conf;prj-{rmw}.conf"))
}

/// Get path to a prebuilt Zephyr fixture binary.
///
/// This function checks if a Zephyr binary already exists in the build directory
/// and returns it when it is fresh. Tests must not build fixtures in their
/// bodies; stale or missing fixtures report a setup error that points to
/// `just zephyr build-fixtures`.
///
/// # Arguments
/// * `example_name` - Name of the example directory (e.g., "zephyr-rs-talker")
/// * `platform` - Target platform
/// # Returns
/// Path to the binary
pub fn get_prebuilt_zephyr_example(
    example_name: &str,
    platform: ZephyrPlatform,
) -> TestResult<PathBuf> {
    let workspace = zephyr_workspace_path()
        .ok_or_else(|| TestError::BuildFailed("Zephyr workspace not found".to_string()))?;

    let build_root = zephyr_build_root(&workspace);
    let build_dir = build_dir_for_example(example_name);

    // phase-350 W1.b — a coordinate-scoped lane build now omits west leaves
    // outside its coordinates, so the run must recognise them as out-of-lane
    // instead of failing on a fixture the build deliberately skipped. West
    // leaves cannot be attributed by PATH (their artifacts live in the Zephyr
    // workspace, not under the row's dir), so they take the coordinate route —
    // the same one issue 0517's multi-row leaves take — keyed on the build-dir
    // name both halves already agree on.
    crate::fixtures::lane::require_west_leaf_in_lane(&build_dir, example_name)?;

    // Determine binary path based on platform
    let binary_path = match platform {
        ZephyrPlatform::NativeSim => build_root.join(format!("{}/zephyr/zephyr.exe", build_dir)),
        ZephyrPlatform::QemuArm
        | ZephyrPlatform::QemuCortexA9
        | ZephyrPlatform::QemuCortexA53Smp => {
            build_root.join(format!("{}/zephyr/zephyr.elf", build_dir))
        }
    };

    // Issue 0466 — the source-candidate check used to live HERE, as a second
    // block after the `.d` guard, and only here. That is why every workspace
    // entry resolved through `binaries/mod.rs` (params, lifecycle, qos, safety,
    // multihost, the three realtime ones, and the C/C++/mixed entries) got the
    // `.d` guard alone. Both halves now live inside
    // `require_prebuilt_binary_fresh_zephyr`, one spelling, so a new resolver
    // cannot pick up half the coverage by accident.
    let decoded = decode_alias(example_name);
    let leaf = format!("examples/{}", example_path_for_name(example_name));
    let conf_files = conf_files_for_example(example_name);
    let binary = crate::fixtures::require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        crate::fixtures::ZephyrLeafSource {
            dir: &leaf,
            lang: decoded.map(|(lang, _, _)| lang),
            rmw: decoded.map(|(_, _, rmw)| rmw),
            conf_files: conf_files.as_deref(),
        },
    )?;
    eprintln!("Using prebuilt Zephyr binary: {}", binary.display());
    Ok(binary)
}

/// Build directory the 225.P workspace-Entry leaf is emitted into by
/// `scripts/build/zephyr-fixture-leaves.sh` (Approach A — a single
/// post-matrix leaf, not a role/RMW-decoded alias). The Zephyr Entry
/// (`examples/workspaces/rust/src/zephyr_entry`) defaults to the zenoh
/// RMW (`prj.conf;prj-zenoh.conf`), so the native_sim ELF lands at
/// `<build_root>/build-ws-rs-entry-zenoh/zephyr/zephyr.exe`.
const ZEPHYR_WORKSPACE_ENTRY_BUILD_DIR: &str = "build-ws-rs-entry-zenoh";

/// Source-tree key handed to [`is_binary_stale`] for the workspace Entry.
/// It is not a `decode_alias` name (the Entry has no role/RMW alias), so
/// the decoder falls through to `None` — which makes staleness watch the
/// whole `examples/workspaces/rust` tree plus every shared core/rmw crate
/// (never under-watches).
const ZEPHYR_WORKSPACE_ENTRY_SRC_KEY: &str = "workspaces/rust";

/// Get path to the prebuilt Zephyr **workspace Entry** binary (Phase 225.P).
///
/// The workspace Entry is a single Zephyr application that hosts the whole
/// launch-defined node set — talker *and* listener — in one process
/// (`nros::main!(launch = "demo_bringup:system.launch.xml")`). It is the
/// Zephyr sibling of the native / FreeRTOS / ThreadX workspace Entries.
///
/// Mirrors [`get_prebuilt_zephyr_example`] but resolves the fixed
/// workspace-Entry build directory directly — there is no role/RMW alias to
/// decode. Tests must not build fixtures in their bodies; a missing or stale
/// binary surfaces a setup error pointing at `just zephyr build-fixtures`.
///
/// Only `native_sim` is resolved (the E2E lane runs the host build); other
/// boards flash the same Entry source but are out of scope for the host test
/// harness.
///
/// # Returns
/// Path to `build-ws-rs-entry-zenoh/zephyr/zephyr.exe`.
pub fn get_prebuilt_zephyr_workspace_entry() -> TestResult<PathBuf> {
    let workspace = zephyr_workspace_path()
        .ok_or_else(|| TestError::BuildFailed("Zephyr workspace not found".to_string()))?;

    let build_root = zephyr_build_root(&workspace);
    // phase-350 W1.b — same coordinate route as `get_prebuilt_zephyr_example`.
    crate::fixtures::lane::require_west_leaf_in_lane(
        ZEPHYR_WORKSPACE_ENTRY_BUILD_DIR,
        "workspaces/rust/src/zephyr_entry",
    )?;
    let binary_path = build_root.join(format!(
        "{ZEPHYR_WORKSPACE_ENTRY_BUILD_DIR}/zephyr/zephyr.exe"
    ));

    // Issue 0466 — one spelling; see `get_prebuilt_zephyr_example`. The leaf is
    // the WHOLE `workspaces/rust` tree rather than `src/zephyr_entry`, which is
    // what `ZEPHYR_WORKSPACE_ENTRY_SRC_KEY` has always meant: the entry pulls in
    // sibling node packages from that workspace, so watching only its own dir
    // would under-watch. Errors are no longer remapped — `stale_error` names the
    // input that moved, which "binary not found" threw away.
    let binary = crate::fixtures::require_prebuilt_binary_fresh_zephyr(
        &binary_path,
        crate::fixtures::ZephyrLeafSource::zenoh(
            &format!("examples/{ZEPHYR_WORKSPACE_ENTRY_SRC_KEY}"),
            "rust",
        ),
    )?;
    eprintln!(
        "Using prebuilt Zephyr workspace Entry binary: {}",
        binary.display()
    );
    Ok(binary)
}

/// Whether a `packages/core/<crate>` subdir should be watched for staleness
/// of a fixture whose language API crate is `lang_api_crate` (`Some("nros-c")`
/// for C, `Some("nros-cpp")` for C++, `Some("nros")` for Rust, `None` if the
/// language is unknown). Drops only the *other* languages' API crates; every
/// shared/platform/rmw crate stays watched (Phase 177.8). Unknown language →
/// watch everything (never under-watch).
fn core_crate_is_watched(crate_name: &str, lang_api_crate: Option<&str>) -> bool {
    let is_lang_api = matches!(crate_name, "nros" | "nros-c" | "nros-cpp");
    let is_other_lang_api = is_lang_api && lang_api_crate.is_some_and(|c| c != crate_name);
    !is_other_lang_api
}

/// The one implementation of "is this Zephyr image older than the sources it was
/// built from", keyed on a source DIRECTORY rather than an example alias.
///
/// Issue 0466 — this used to be reachable only through an `is_binary_stale`
/// wrapper keyed on a `decode_alias` NAME, so it covered the per-example leaves and
/// exactly ONE workspace entry. Every other entry (params, lifecycle, qos,
/// safety, multihost, the three realtime ones, and the C/C++/mixed entries) went
/// through `require_prebuilt_binary_fresh_zephyr` alone, which watches only the
/// cargo staticlib's `.d`. Measured on `build-ws-rs-qos-entry-zenoh`: that `.d`
/// lists 529 deps and the only `.conf` among them is the GENERATED
/// `<build_root>/zephyr/.conf` — the leaf's own `prj.conf`, the file an author
/// edits, appears ZERO times. So raising `CONFIG_NROS_MAX_QUERYABLES` left every
/// image "fresh" with the old value compiled in, and the qos and lifecycle cells
/// failed with product-level assertions that sent two sessions (0460, then this
/// one) looking at the product instead of the image.
///
/// Worse for C and C++: `zephyr_staticlib_dep_file` looks for `librustapp.d`, a
/// C-only image has none, and the helper then returns `Ok` — those entries had
/// NO freshness check at all, only an existence check.
///
/// `lang` is the alias language token (`"c"` / `"cpp"` / `"rust"`), used only to
/// drop the OTHER languages' API crates from the watch set; `None` watches every
/// core crate, which never under-watches. `rmw` likewise narrows to one backend
/// package, `None` watches all three. `conf_files` is the `;`-separated
/// `-DCONF_FILE` value when the leaf uses one.
pub(crate) fn source_dir_is_stale(
    binary_path: &Path,
    example_dir: &Path,
    lang: Option<&str>,
    rmw: Option<&str>,
    conf_files: Option<&str>,
    // phase-363 W5 — extra candidates the BUILD recorded (ninja's RERUN_CMAKE
    // edge), which no hand list can reach: the shared `cmake/NanoRos*.cmake`
    // modules and the entry template the generated TU comes from.
    extra: &[std::path::PathBuf],
) -> bool {
    let Ok(binary_mtime) = binary_path.metadata().and_then(|m| m.modified()) else {
        // Can't stat the binary — assume stale so we rebuild and get a
        // real error instead of reusing something mysterious.
        return true;
    };

    let root = project_root();

    // The example-local set catches app source, Kconfig overlays, and Rust
    // dependency changes. The package set catches shared nros backend/platform
    // edits; otherwise tests can report stale Zephyr runtime failures after a
    // library fix has already landed.
    let mut candidates = vec![
        example_dir.join("prj.conf"),
        example_dir.join("CMakeLists.txt"),
        example_dir.join("Cargo.toml"),
        example_dir.join("Cargo.lock"),
        example_dir.join("boards"),
        example_dir.join("src"),
        root.join("zephyr"),
    ];

    // Watch `packages/core`, but skip the *other* languages' API crates so a
    // single-language core edit (e.g. an `nros-cpp` change) doesn't falsely
    // mark unrelated C/Rust fixtures stale — cmake correctly leaves them
    // un-rebuilt (a C fixture links `nros-c`, not `nros-cpp`), yet the gate
    // would report a spurious runtime "is stale" failure (Phase 177.8). Every
    // shared/platform/rmw crate (nros-core, nros-node, nros-rmw, nros-serdes,
    // nros-platform-*, …) stays watched, and new crates are picked up
    // automatically; only the two non-matching language API crates are
    // dropped. `nros` = Rust API, `nros-c` = C API, `nros-cpp` = C++ API.
    let lang_api_crate = match lang {
        Some("c") => Some("nros-c"),
        Some("cpp") => Some("nros-cpp"),
        Some("rust") => Some("nros"),
        _ => None,
    };
    let core_dir = root.join("packages/core");
    match std::fs::read_dir(&core_dir) {
        Ok(entries) => {
            for entry in entries.flatten() {
                let name = entry.file_name();
                if core_crate_is_watched(&name.to_string_lossy(), lang_api_crate) {
                    candidates.push(entry.path());
                }
            }
        }
        // Can't enumerate — fall back to watching the whole tree (safe: never
        // under-watches, at worst keeps the old over-broad behaviour).
        Err(_) => candidates.push(core_dir),
    }
    match rmw {
        Some("cyclonedds") => candidates.push(root.join("packages/rmw/cyclonedds")),
        Some("xrce") => candidates.push(root.join("packages/rmw/xrce")),
        Some("zenoh") => candidates.push(root.join("packages/rmw/zenoh")),
        _ => {
            candidates.push(root.join("packages/rmw/cyclonedds"));
            candidates.push(root.join("packages/rmw/xrce"));
            candidates.push(root.join("packages/rmw/zenoh"));
        }
    }
    if let Some(conf_files) = conf_files {
        for conf_file in conf_files.split(';') {
            candidates.push(example_dir.join(conf_file));
        }
    }
    candidates.extend(extra.iter().cloned());
    // #147 / phase-286 W2 — content-aware staleness. A pure mtime compare
    // (`path_newer_than`) cannot tell a real edit from an mtime bump that left
    // the bytes identical (a rebase/checkout/pull "mtime treadmill", or an edit
    // reverted to the same content), and false-reports EVERY fixture stale after
    // such a bump even though the image is current. `candidates_changed_content`
    // uses the LINKED binary's own content hash as the "was rebuilt" signal and
    // only content-hashes sources whose (mtime,size) actually moved, so an
    // identical-content bump is not stale while a genuine edit still is. Falls
    // back to the mtime compare if the binary can't be hashed.
    let _ = binary_mtime;
    match candidates_changed_content(binary_path, &candidates) {
        Some(stale) => stale,
        None => candidates.iter().any(|p| path_newer_than(p, binary_mtime)),
    }
}

fn path_newer_than(path: &Path, cutoff: std::time::SystemTime) -> bool {
    let Ok(meta) = path.metadata() else {
        return false;
    };
    if !meta.is_dir() {
        // Files: a newer mtime is a real content change.
        return meta.modified().is_ok_and(|mtime| mtime > cutoff);
    }
    // Directories: do NOT trust the directory's OWN mtime — it bumps on any
    // transient entry add/remove (e.g. a codegen step writing then deleting a
    // temp file inside a watched `include/` dir), which is not a source change
    // and would falsely mark EVERY fixture stale right after a clean rebuild.
    // (Observed: `_test-c-codegen` churns `packages/core/nros-{c,cpp}/include/
    // nros/`, bumping the dir mtime while every header inside stays unchanged
    // — git-clean — yet the old `meta.modified() > cutoff` early-return tripped
    // here and reported all zephyr fixtures stale.) Recurse into entries
    // instead; only real file mtimes count. Pure deletions are not detected by
    // mtime anyway — the build-side content `.nros-zephyr-fixture.sig` is the
    // safety net for those.
    let Ok(entries) = std::fs::read_dir(path) else {
        return false;
    };
    for entry in entries.flatten() {
        let p = entry.path();
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if crate::treewalk::is_skipped_dir(name.as_ref()) {
            continue;
        }
        if path_newer_than(&p, cutoff) {
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_core_crate_is_watched_per_language() {
        // Shared crates: always watched, regardless of fixture language.
        for lang in [Some("nros-c"), Some("nros-cpp"), Some("nros"), None] {
            for shared in ["nros-core", "nros-node", "nros-rmw", "nros-platform-zephyr"] {
                assert!(
                    core_crate_is_watched(shared, lang),
                    "shared crate {shared} must stay watched for lang {lang:?}"
                );
            }
        }
        // C fixture: watches nros-c, drops the other two language API crates.
        assert!(core_crate_is_watched("nros-c", Some("nros-c")));
        assert!(!core_crate_is_watched("nros-cpp", Some("nros-c")));
        assert!(!core_crate_is_watched("nros", Some("nros-c")));
        // C++ fixture.
        assert!(core_crate_is_watched("nros-cpp", Some("nros-cpp")));
        assert!(!core_crate_is_watched("nros-c", Some("nros-cpp")));
        assert!(!core_crate_is_watched("nros", Some("nros-cpp")));
        // Rust fixture.
        assert!(core_crate_is_watched("nros", Some("nros")));
        assert!(!core_crate_is_watched("nros-c", Some("nros")));
        assert!(!core_crate_is_watched("nros-cpp", Some("nros")));
        // Unknown language: never under-watch — all language crates kept.
        assert!(core_crate_is_watched("nros-c", None));
        assert!(core_crate_is_watched("nros-cpp", None));
        assert!(core_crate_is_watched("nros", None));
    }

    #[test]
    fn content_aware_staleness_ignores_mtime_only_bumps() {
        // #147 / phase-286 W2 — the core guarantee: a watched source whose mtime
        // moves but whose BYTES are unchanged (rebase/checkout/inert-edit) must
        // NOT report the fixture stale, while a real content edit must.
        use std::{thread::sleep, time::Duration};
        let dir = tempfile::tempdir().unwrap();
        let bin = dir.path().join("zephyr.exe");
        let src = dir.path().join("watched.rs");
        std::fs::write(&bin, b"BINARY-V1").unwrap();
        std::fs::write(&src, b"fn main() {}").unwrap();
        let candidates = vec![src.clone()];

        // First sight: no baseline yet → records it, reports not-stale.
        assert_eq!(
            candidates_changed_content(&bin, &candidates),
            Some(false),
            "first sight must record a baseline and report fresh"
        );

        // mtime-only bump (identical bytes rewritten) → NOT stale.
        sleep(Duration::from_millis(10));
        std::fs::write(&src, b"fn main() {}").unwrap();
        assert_eq!(
            candidates_changed_content(&bin, &candidates),
            Some(false),
            "an mtime bump with identical content must not be stale"
        );

        // Real content edit (binary unchanged) → STALE.
        std::fs::write(&src, b"fn main() { /* edited */ }").unwrap();
        assert_eq!(
            candidates_changed_content(&bin, &candidates),
            Some(true),
            "a genuine content change must report stale"
        );

        // Rebuild (binary bytes change) → the image is the fresh truth again.
        std::fs::write(&bin, b"BINARY-V2").unwrap();
        assert_eq!(
            candidates_changed_content(&bin, &candidates),
            Some(false),
            "a rebuilt binary re-baselines and reports fresh"
        );
    }

    #[test]
    fn runners_yaml_parses_native_and_qemu() {
        // phase-295 W5.a — the harness must read the framework's runner
        // metadata to construct its launch. Lock the tiny parser against the
        // real emitter shape (native_sim + a qemu board).
        let dir = tempfile::tempdir().unwrap();
        let zdir = dir.path();

        // native_sim shape (as emitted for native_sim/native/64).
        std::fs::write(
            zdir.join("runners.yaml"),
            "runners:\n- native\nflash-runner: native\ndebug-runner: native\n\
             config:\n  board_dir: /x/boards/native/native_sim\n  \
             elf_file: zephyr.elf\n  exe_file: zephyr.exe\n  gdb: /usr/bin/gdb\n",
        )
        .unwrap();
        let r = RunnersYaml::from_zephyr_dir(zdir).unwrap();
        assert_eq!(r.flash_runner.as_deref(), Some("native"));
        assert_eq!(r.runners, vec!["native".to_string()]);
        assert_eq!(r.exe_file.as_deref(), Some("zephyr.exe"));
        assert_eq!(r.exe_path().unwrap(), zdir.join("zephyr.exe"));
        assert_eq!(r.elf_path().unwrap(), zdir.join("zephyr.elf"));

        // qemu board shape (multiple runners, flash-runner qemu).
        std::fs::write(
            zdir.join("runners.yaml"),
            "runners:\n- qemu\n- jlink\nflash-runner: qemu\n\
             config:\n  elf_file: zephyr.elf\n",
        )
        .unwrap();
        let r = RunnersYaml::from_zephyr_dir(zdir).unwrap();
        assert_eq!(r.flash_runner.as_deref(), Some("qemu"));
        assert!(r.runners.iter().any(|x| x == "qemu"));
        assert_eq!(r.elf_path().unwrap(), zdir.join("zephyr.elf"));
        assert!(r.exe_file.is_none());
    }

    #[test]
    fn runners_yaml_absent_is_none() {
        let dir = tempfile::tempdir().unwrap();
        assert!(RunnersYaml::from_zephyr_dir(dir.path()).is_none());
    }

    #[test]
    fn test_platform_board_spec() {
        assert_eq!(
            ZephyrPlatform::NativeSim.board_spec(),
            "native_sim/native/64"
        );
        assert_eq!(ZephyrPlatform::QemuArm.board_spec(), "qemu_cortex_m3");
    }

    #[test]
    fn test_west_detection() {
        let available = is_west_available();
        eprintln!("west available: {}", available);
    }

    #[test]
    fn test_workspace_detection() {
        if let Some(path) = zephyr_workspace_path() {
            eprintln!("Zephyr workspace: {}", path.display());
            assert!(path.exists());
        } else {
            eprintln!("Zephyr workspace not found");
        }
    }
}
