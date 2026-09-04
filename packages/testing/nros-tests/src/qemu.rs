//! QEMU process fixture for embedded testing
//!
//! Provides managed QEMU processes for testing ARM Cortex-M binaries.
//!
//! ## Launch convention — SANCTIONED hand-rolled builders (phase-295 W5.c)
//!
//! Every `QemuProcess::start_*` builder below assembles a `qemu-system-{arm,
//! riscv32,riscv64}` command line by hand. This is the **sanctioned form**
//! (RFC-0051 §4 / audit checklist E9 / the #222 E1-exception pattern), not a
//! bypass to fix, because the platforms this module launches — FreeRTOS,
//! NuttX, ThreadX, and pure bare-metal — have **no build-system runner
//! metadata** to interpret:
//!
//! - There is no Zephyr `runners.yaml` (that path is handled by the
//!   [`crate::zephyr::RunnersYaml`] interpreter) and no ESP-IDF
//!   `flasher_args.json` ([`crate::esp32::EspFlasherArgs`]) for these images.
//!   They are plain cross-compiled ELFs; the emulator machine/CPU/netdev plan
//!   lives only in each board crate's build glue and this file.
//! - So `qemu.rs` is itself "the interpreter" E9 refers to: the hand-rolled
//!   `-M/-cpu/-netdev` blocks here are the run convention, and consuming
//!   PREBUILT ELFs (never building at run time) keeps the E1 no-compile rule.
//!
//! FOLLOW-UP (phase doc W5.c): the per-machine argument blocks (the
//! `-M virt -cpu cortex-a7 …` specifics) can later move into each board
//! crate's `NROS_BOARD_RUNNER`-adjacent metadata so a board owns its launch
//! line (the phase-215 duty rule), leaving these functions as thin readers of
//! board-provided launch metadata. That relocation is deferred here to avoid
//! disturbing the many green QEMU lanes; the doc-comment route is the
//! sanctioned interim state.

use crate::{
    TestError, TestResult,
    process::{kill_process_group, set_new_process_group},
};
#[cfg(unix)]
use std::os::unix::io::AsRawFd;
use std::{
    io::Read,
    path::Path,
    process::{Child, Command, Stdio},
    time::{Duration, Instant},
};

/// Phase 127.D + Phase 143 — pick the `qemu-system-arm` binary to use.
///
/// Selection order:
///   1. `QEMU_SYSTEM_ARM` env var (developer override / CI pin).
///   2. Project-local patched build at
///      `<project_root>/build/qemu/bin/qemu-system-arm` (built by
///      `just qemu setup-qemu`, includes the LAN9118 RX-flush patch
///      and ships qemu ≥ 7.2 so `-netdev dgram,local.type=unix,…`
///      works for NuttX / ThreadX multi-instance tests).
///   3. System `qemu-system-arm` on `$PATH` — kept as fallback so
///      contributors who ran a minimal `just setup` still produce
///      the documented `[SKIPPED]` rather than an exec error.
pub fn qemu_system_arm_path() -> std::ffi::OsString {
    if let Some(env) = std::env::var_os("QEMU_SYSTEM_ARM") {
        return env;
    }
    // phase-334 W2.b step 2 — the Rust mirror of `nros_build_dir`. Unlike the
    // other families this one has no shell twin to move: `QEMU_PREFIX` in
    // just/qemu-baremetal.just is a parse-time `absolute_path()` and stays a
    // literal (a known, recorded exception), so the two spellings agree by the
    // derivation emitting today's path rather than by both being migrated.
    let patched = crate::build_dir(crate::kind::QEMU, &["bin"]).join("qemu-system-arm");
    if patched.exists() {
        return patched.into_os_string();
    }
    // `nros setup` store qemu (the patched `11.0.0-nros*` dist).
    if let Some(store) = crate::nros_store_bin("qemu", "qemu-system-arm") {
        return store.into_os_string();
    }
    std::ffi::OsString::from("qemu-system-arm")
}

/// Convenience wrapper around [`qemu_system_arm_path`] that returns
/// a fresh [`Command`] preconfigured to invoke the resolved binary.
/// Prefer this over `Command::new("qemu-system-arm")` so future
/// `build/qemu/` patches propagate automatically (Phase 143).
pub fn qemu_system_arm_cmd() -> Command {
    Command::new(qemu_system_arm_path())
}

/// Best-effort discovery of the cargo workspace root by walking
/// Phase 88.16.A — flip a piped stdio fd to non-blocking so the
/// drain loop below never wedges.
#[cfg(unix)]
fn set_nonblocking<F: AsRawFd>(fd: &F) {
    let raw = fd.as_raw_fd();
    // SAFETY: `raw` is a valid OS fd we own; fcntl is async-signal-safe.
    unsafe {
        let flags = libc::fcntl(raw, libc::F_GETFL);
        if flags >= 0 {
            libc::fcntl(raw, libc::F_SETFL, flags | libc::O_NONBLOCK);
        }
    }
}

/// Phase 88.16.A — drain whatever bytes are currently buffered on
/// `src` into `dst`. Returns `true` if any bytes were consumed.
/// Treats `WouldBlock`, `0`, and read errors as "no progress".
fn drain_into<R: Read>(src: &mut R, buffer: &mut [u8], dst: &mut String) -> bool {
    match src.read(buffer) {
        Ok(0) => false,
        Ok(n) => {
            dst.push_str(&String::from_utf8_lossy(&buffer[..n]));
            true
        }
        Err(_) => false,
    }
}

// phase-334 W2.b step 2 — the private `project_root()` walk-up that used to
// live here is GONE, not silenced. Its only caller was `qemu_system_arm_path`,
// and the root it recomputed is what `crate::build_dir` already derives. Two
// spellings of "where is the repo" is the R3 drift this step exists to remove;
// keeping it behind an `#[allow(dead_code)]` would have preserved the second
// spelling for the next caller to find.

/// Managed QEMU process for Cortex-M3 emulation
///
/// Starts QEMU with semihosting enabled and captures output.
/// Automatically kills the process on drop.
///
/// # Example
///
/// ```ignore
/// use nros_tests::fixtures::QemuProcess;
/// use std::path::Path;
///
/// let binary = Path::new("target/thumbv7m-none-eabi/release/my-test");
/// let mut qemu = QemuProcess::start_cortex_m3(binary).unwrap();
/// let output = qemu.wait_for_output(Duration::from_secs(30)).unwrap();
/// assert!(output.contains("[PASS]"));
/// ```
pub struct QemuProcess {
    handle: Child,
    /// The exact command this process was started with — see
    /// [`QemuProcess::command_line`].
    cmdline: String,
}

/// Render a spawned command as a line a human can paste into a shell.
///
/// Issue 0877 — "run it by hand with the harness's own QEMU arguments" is only
/// a valid experiment if the hand run uses the harness's own QEMU BINARY, and
/// nothing here ever said which one that is. [`qemu_system_arm_path`] prefers
/// `build/qemu/bin/qemu-system-arm` (our LAN9118 flow-control patch, issues
/// 0830 / 0917) over the `qemu-system-arm` a shell resolves from `$PATH`, so
/// copying the ARGUMENTS alone silently swaps the emulated NIC's RX flow
/// control for a different implementation — the one component a delivery
/// failure is most likely to be about. Recording the resolved program next to
/// its arguments makes the two runs comparable.
///
/// Quoting is the conservative kind: anything outside a known-safe set is
/// single-quoted, so the line is pasteable rather than merely readable.
pub(crate) fn render_command(cmd: &Command) -> String {
    fn quote(word: &str) -> String {
        let safe = |c: char| c.is_ascii_alphanumeric() || "_-./=:,@%+".contains(c);
        if !word.is_empty() && word.chars().all(safe) {
            word.to_string()
        } else {
            format!("'{}'", word.replace('\'', r"'\''"))
        }
    }
    let mut out = quote(&cmd.get_program().to_string_lossy());
    for arg in cmd.get_args() {
        out.push(' ');
        out.push_str(&quote(&arg.to_string_lossy()));
    }
    out
}

impl QemuProcess {
    /// Spawn `cmd`, recording the exact command line for failure messages.
    ///
    /// THE one spawn point for every `start_*` builder in this file, so a new
    /// builder cannot forget to record what it ran (issue 0877).
    fn spawn(mut cmd: Command) -> TestResult<Self> {
        let cmdline = render_command(&cmd);
        let handle = cmd.spawn()?;
        Ok(Self { handle, cmdline })
    }

    /// The command line this QEMU was started with, pasteable into a shell.
    ///
    /// Assertion messages that report "the guest did nothing" must carry this:
    /// the RTOS lanes run under `--failure-output never` (see the `just
    /// <platform> test` recipes), so anything printed with `eprintln!` is
    /// discarded and only the panic text survives. Issue 0877 is what that
    /// costs — a report whose central experiment was a hand run that could not
    /// have used the same emulator, with nothing in the failure output to say
    /// so.
    #[must_use]
    pub fn command_line(&self) -> &str {
        &self.cmdline
    }

    /// Start QEMU Cortex-M3 emulator with semihosting
    ///
    /// Uses the LM3S6965EVB machine which supports semihosting output.
    ///
    /// # Arguments
    /// * `binary` - Path to the ARM ELF binary to run
    ///
    /// # Returns
    /// A managed QEMU process
    pub fn start_cortex_m3(binary: &Path) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Binary not found: {}",
                binary.display()
            )));
        }

        let mut cmd = qemu_system_arm_cmd();
        cmd.args([
            "-cpu",
            "cortex-m3",
            "-machine",
            "lm3s6965evb",
            "-nographic",
            "-semihosting-config",
            "enable=on,target=native",
            "-kernel",
        ])
        .arg(binary)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        Self::spawn(cmd)
    }

    /// Start QEMU with MPS2-AN385 machine (Cortex-M3 + LAN9118 Ethernet)
    ///
    /// Uses the MPS2-AN385 machine which has a LAN9118 Ethernet controller.
    ///
    /// # Arguments
    /// * `binary` - Path to the ARM ELF binary to run
    ///
    /// # Returns
    /// A managed QEMU process
    pub fn start_mps2_an385(binary: &Path) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Binary not found: {}",
                binary.display()
            )));
        }

        let mut cmd = qemu_system_arm_cmd();
        cmd.args([
            "-cpu",
            "cortex-m3",
            "-machine",
            "mps2-an385",
            "-nographic",
            "-semihosting-config",
            "enable=on,target=native",
            "-kernel",
        ])
        .arg(binary)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        Self::spawn(cmd)
    }

    /// Start QEMU with MPS2-AN385 machine + LAN9118 slirp networking
    ///
    /// Uses the MPS2-AN385 machine with a LAN9118 Ethernet controller in QEMU's
    /// user-mode (slirp) networking. Each QEMU instance gets its own fully
    /// isolated NAT stack — no TAP devices, no bridge, no sudo needed.
    ///
    /// The firmware connects to the host via slirp gateway `10.0.2.2`. The
    /// firmware's `config.toml` must use the `10.0.2.0/24` subnet with gateway
    /// `10.0.2.2` and a unique guest IP per instance.
    ///
    /// Uses `-icount shift=auto` to synchronize QEMU's virtual clock with
    /// wall-clock time. Without this, hardware timers (CMSDK Timer0) race ahead
    /// during WFI, causing zenoh-pico timeouts to expire before network I/O
    /// completes. See `docs/reference/qemu-icount.md`.
    pub fn start_mps2_an385_networked(binary: &Path) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Binary not found: {}",
                binary.display()
            )));
        }

        let mut cmd = qemu_system_arm_cmd();
        cmd.args([
            "-cpu",
            "cortex-m3",
            "-machine",
            "mps2-an385",
            "-nographic",
            // Synchronize virtual clock with wall-clock time. With sleep=on
            // (default), WFI advances virtual time at wall-clock speed via
            // QEMU_CLOCK_VIRTUAL_RT instead of jumping instantly. This keeps
            // hardware timer-backed clocks (CMSDK Timer0) aligned with slirp
            // network I/O timing.
            "-icount",
            "shift=auto",
            "-semihosting-config",
            "enable=on,target=native",
            "-kernel",
        ])
        .arg(binary)
        .args(["-nic", "user,model=lan9118"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        Self::spawn(cmd)
    }

    /// phase-263 C2b — MPS2-AN385 + LAN9118 slirp, with the user-net configured to the
    /// FreeRTOS board's COMPILE-TIME static address plan (`192.0.3.0/24`, gateway
    /// `192.0.3.1`) instead of slirp's default `10.0.2.0/24`.
    ///
    /// The `nros-board-mps2-an385-freertos` firmware brings up lwIP with a STATIC IP
    /// `192.0.3.10` / gateway `192.0.3.1` (no DHCP — see `startup.c`), so the default
    /// slirp net is unroutable for it. Pointing slirp's `host=` at the board's gateway
    /// (`192.0.3.1`) makes the guest reach the host machine, which forwards to a zenohd
    /// bound on `0.0.0.0:<port>`. The entry's baked locator is therefore
    /// `tcp/192.0.3.1:<port>`. No TAP / bridge / sudo.
    pub fn start_mps2_an385_freertos_slirp(binary: &Path) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Binary not found: {}",
                binary.display()
            )));
        }

        let mut cmd = qemu_system_arm_cmd();
        cmd.args([
            "-cpu",
            "cortex-m3",
            "-machine",
            "mps2-an385",
            "-nographic",
            "-icount",
            "shift=auto",
            "-semihosting-config",
            "enable=on,target=native",
            "-kernel",
        ])
        .arg(binary)
        // net/host match the board's static lwIP config so the guest's gateway
        // (192.0.3.1) IS the slirp host → forwards to the host machine's zenohd.
        .args(["-nic", "user,model=lan9118,net=192.0.3.0/24,host=192.0.3.1"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        Self::spawn(cmd)
    }

    /// Start QEMU with MPS2-AN385 machine + LAN9118 mcast-socket
    /// networking (Phase 97.4.freertos).
    ///
    /// Two QEMU instances launched with the same `mcast_addr:port`
    /// share a virtual L2 broadcast domain on the host (no `sudo`,
    /// no TAP, no bridge needed). RTPS SPDP / SEDP / ARP all flow
    /// between them. Mirrors the Zephyr A9 mcast pattern Phase 92
    /// uses for cross-instance DDS.
    ///
    /// `mac` must be unique per instance so ARP behaves; the FreeRTOS
    /// board crate's `Config::mac` should match what's passed here.
    pub fn start_mps2_an385_mcast(
        binary: &Path,
        mcast_addr_port: &str,
        mac: &str,
    ) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Binary not found: {}",
                binary.display()
            )));
        }

        let mut cmd = qemu_system_arm_cmd();
        cmd.args([
            "-cpu",
            "cortex-m3",
            "-machine",
            "mps2-an385",
            "-nographic",
            "-icount",
            "shift=auto",
            "-semihosting-config",
            "enable=on,target=native",
            "-kernel",
        ])
        .arg(binary)
        .args([
            "-nic",
            &format!("socket,model=lan9118,mcast={mcast_addr_port},mac={mac}"),
        ])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        Self::spawn(cmd)
    }

    /// Start QEMU with MPS2-AN385 machine + external serial device
    ///
    /// Connects UART0 to the given serial device path (e.g., a socat PTY).
    /// Uses `-display none -monitor none` to avoid `-nographic`'s implicit
    /// `-serial mon:stdio` which would hijack UART0 for the monitor.
    /// Semihosting output goes to stdout via the ARM semihosting interface.
    ///
    /// # Arguments
    /// * `binary` - Path to the ARM ELF binary to run
    /// * `serial_path` - Host serial device path (e.g., `/tmp/serial-a`)
    pub fn start_mps2_an385_with_serial(binary: &Path, serial_path: &str) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Binary not found: {}",
                binary.display()
            )));
        }

        let chardev_arg = format!("serial,id=ser0,path={}", serial_path);
        let mut cmd = qemu_system_arm_cmd();
        cmd.args([
            "-cpu",
            "cortex-m3",
            "-machine",
            "mps2-an385",
            "-display",
            "none",
            "-monitor",
            "none",
            // Synchronize virtual clock with wall-clock time. The firmware
            // uses CMSDK Timer0 (hardware timer) for zenoh-pico timeouts.
            // Without icount, WFI advances virtual time instantly, causing
            // timeouts to fire before serial I/O through zenohd completes.
            "-icount",
            "shift=auto",
            "-semihosting-config",
            "enable=on,target=native",
            "-chardev",
        ])
        .arg(&chardev_arg)
        .args(["-serial", "chardev:ser0", "-kernel"])
        .arg(binary)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        Self::spawn(cmd)
    }

    /// Wait for QEMU to produce output and exit
    ///
    /// # Arguments
    /// * `timeout` - Maximum time to wait
    ///
    /// # Returns
    /// The combined stdout/stderr output
    pub fn wait_for_output(&mut self, timeout: Duration) -> TestResult<String> {
        let start = Instant::now();
        let mut output = String::new();

        // Phase 88.16.A — drain stderr alongside stdout so logging
        // records (which route through `hstderr()` on bare-metal
        // semihosting) reach the captured string. Examples that
        // emit via `log_info!` would otherwise look silent to the
        // harness.
        let mut stdout = self
            .handle
            .stdout
            .take()
            .ok_or_else(|| TestError::ProcessFailed("No stdout".to_string()))?;
        let mut stderr = self.handle.stderr.take();

        // Set non-blocking mode so read() doesn't block indefinitely when the
        // QEMU process pauses output (e.g., listener waiting for messages).
        #[cfg(unix)]
        {
            set_nonblocking(&stdout);
            if let Some(ref s) = stderr {
                set_nonblocking(s);
            }
        }

        let mut buffer = [0u8; 4096];
        loop {
            if start.elapsed() > timeout {
                kill_process_group(&mut self.handle);
                if output.is_empty() {
                    return Err(TestError::Timeout);
                }
                break;
            }

            match self.handle.try_wait() {
                Ok(Some(_status)) => {
                    let _ = stdout.read_to_string(&mut output);
                    if let Some(mut s) = stderr.take() {
                        let _ = s.read_to_string(&mut output);
                    }
                    break;
                }
                Ok(None) => {
                    let mut progressed = false;
                    progressed |= drain_into(&mut stdout, &mut buffer, &mut output);
                    if let Some(ref mut s) = stderr {
                        progressed |= drain_into(s, &mut buffer, &mut output);
                    }
                    if !progressed {
                        std::thread::sleep(Duration::from_millis(50));
                    }
                    // Check for test completion markers
                    if output.contains("All tests passed")
                        || output.contains("Benchmark complete")
                        || output.contains("TEST COMPLETE")
                        || output.contains("QEMU: Terminated")
                        || output.contains(crate::output::SERVICE_RESULT_PREFIX)
                        || output.contains(crate::output::ACTION_RESULT_PREFIX)
                    {
                        std::thread::sleep(Duration::from_millis(100));
                        kill_process_group(&mut self.handle);
                        break;
                    }
                }
                Err(_) => break,
            }
        }

        Ok(output)
    }

    /// Wait until the output contains `pattern` — the STRICT wait. Kills the
    /// process on timeout.
    ///
    /// `Ok(output)` means the pattern appeared. `Err` means it did not, whether
    /// because the timeout elapsed or because QEMU exited first, and quotes the
    /// output collected so far.
    ///
    /// Issue 0471 was filed against [`crate::process::ManagedProcess`]; this is
    /// the same defect in the same shape, found by fixing the class rather than
    /// the reported site. Both types had TWO lenient returns — timeout with
    /// non-empty output, and process exit — and on an emulator target the exit
    /// path is the common one, so this copy mattered more.
    ///
    /// When a missing pattern is not itself the failure, use
    /// [`Self::collect_until`].
    pub fn wait_for_output_pattern(
        &mut self,
        pattern: &str,
        timeout: Duration,
    ) -> TestResult<String> {
        let out = self.collect_until_inner(pattern, timeout)?;
        if out.contains(pattern) {
            Ok(out)
        } else {
            Err(TestError::ProcessFailed(format!(
                "qemu did not print `{pattern}` within {timeout:?}. Output:\n{out}"
            )))
        }
    }

    /// Wait until EVERY pattern in `patterns` has appeared, then return the
    /// accumulated output. Errors naming the ones still missing at the
    /// deadline.
    ///
    /// issue 0636 — the wait to use when the rule is "each of these must
    /// report". [`Self::wait_for_output_pattern`] returns at the first match,
    /// which for a per-tier rule means the assertion runs against a log that
    /// was cut mid-boot; the failure then reads as a dropped dim rather than as
    /// a short read, which is the most expensive kind of wrong.
    /// Each GROUP must be satisfied by ANY of its alternatives — the shape a
    /// per-tier rule actually has, where a tier may report either its accept
    /// marker or its loud-failure marker and both count as "it reported".
    /// A single-element group degenerates to "this must appear".
    pub fn wait_for_output_each(
        &mut self,
        groups: &[Vec<String>],
        timeout: Duration,
    ) -> TestResult<String> {
        let satisfied = |o: &str, g: &Vec<String>| g.iter().any(|p| o.contains(p.as_str()));
        let out = self.collect_until_pred(|o| groups.iter().all(|g| satisfied(o, g)), timeout)?;
        let missing: Vec<String> = groups
            .iter()
            .filter(|g| !satisfied(&out, g))
            .map(|g| g.join(" | "))
            .collect();
        if missing.is_empty() {
            Ok(out)
        } else {
            Err(TestError::ProcessFailed(format!(
                "qemu did not print {} within {timeout:?}: {}. Output:\n{out}",
                if missing.len() == 1 { "this" } else { "these" },
                missing.join(", ")
            )))
        }
    }

    /// Collect output, stopping early once `pattern` appears — the LENIENT wait.
    ///
    /// Returns whatever QEMU printed, pattern or not. Correct for readiness
    /// waits and for tests that assert on the content themselves (and want the
    /// output in their own failure message). See
    /// [`crate::process::ManagedProcess::collect_until`].
    ///
    /// Returns `String`, not `TestResult`, to match `ManagedProcess` — callers
    /// use the value in their own assertion message. A harness-level failure
    /// (stdout already taken, or QEMU silent until the deadline) is rendered
    /// INTO the returned text rather than dropped, so the assertion that
    /// follows prints the reason instead of an empty diff.
    pub fn collect_until(&mut self, pattern: &str, timeout: Duration) -> String {
        match self.collect_until_inner(pattern, timeout) {
            Ok(out) => out,
            Err(e) => format!("<no output collected: {e}>"),
        }
    }

    fn collect_until_inner(&mut self, pattern: &str, timeout: Duration) -> TestResult<String> {
        self.collect_until_pred(|out| out.contains(pattern), timeout)
    }

    /// Collect output, stopping early once `pattern` has appeared `expected`
    /// times — the COUNTING sibling of [`Self::collect_until`], lenient in
    /// exactly the same way.
    ///
    /// This is the wait for a FREE-RUNNING producer, which is the one shape
    /// [`Self::wait_for_output`] cannot serve: that call is run-to-completion
    /// (it kills the guest at the deadline), so pointing it at a node with no
    /// terminal state silently makes the timeout the node's LIFETIME —
    /// issue 1013, where a 15 s window ended a 1 Hz talker after twelve
    /// publishes and the cell still reported PASS. A count says what such a
    /// test actually means: "keep it alive until it has produced this much".
    ///
    /// Returns whatever was printed, count reached or not, so the caller can
    /// assert on the count itself and put the real transcript in its own
    /// failure message.
    pub fn collect_until_count(
        &mut self,
        pattern: &str,
        expected: usize,
        timeout: Duration,
    ) -> String {
        match self.collect_until_pred(|out| out.matches(pattern).count() >= expected, timeout) {
            Ok(out) => out,
            Err(e) => format!("<no output collected: {e}>"),
        }
    }

    /// The general form: collect until `done` accepts the accumulated output,
    /// or the deadline passes.
    ///
    /// issue 0636 — [`Self::wait_for_output_pattern`] stops at the FIRST match
    /// and then the caller kills QEMU, so a test whose rule needs SEVERAL
    /// distinct markers only sees the later ones if they happened to land in
    /// the same buffer flush. That is a coin flip, not a wait: the multi-tier
    /// sched-dim cells assert one marker PER TIER, and the third tier's line
    /// went missing in 2 of 8 runs purely because the reader had already
    /// returned. Waiting on a predicate lets the caller say "all of them".
    fn collect_until_pred(
        &mut self,
        done: impl Fn(&str) -> bool,
        timeout: Duration,
    ) -> TestResult<String> {
        let start = Instant::now();
        let mut output = String::new();

        let mut stdout = self
            .handle
            .stdout
            .take()
            .ok_or_else(|| TestError::ProcessFailed("No stdout".to_string()))?;
        let mut stderr = self.handle.stderr.take();

        #[cfg(unix)]
        {
            set_nonblocking(&stdout);
            if let Some(ref s) = stderr {
                set_nonblocking(s);
            }
        }

        let mut buffer = [0u8; 4096];
        loop {
            if start.elapsed() > timeout {
                kill_process_group(&mut self.handle);
                if output.is_empty() {
                    return Err(TestError::Timeout);
                }
                break;
            }

            match self.handle.try_wait() {
                Ok(Some(_)) => {
                    let _ = stdout.read_to_string(&mut output);
                    if let Some(mut s) = stderr.take() {
                        let _ = s.read_to_string(&mut output);
                    }
                    break;
                }
                Ok(None) => {
                    let mut progressed = false;
                    progressed |= drain_into(&mut stdout, &mut buffer, &mut output);
                    if let Some(ref mut s) = stderr {
                        progressed |= drain_into(s, &mut buffer, &mut output);
                    }
                    if done(&output) {
                        // Put streams back so follow-up `wait_for_output`
                        // / `kill` calls see them.
                        self.handle.stdout = Some(stdout);
                        self.handle.stderr = stderr;
                        return Ok(output);
                    }
                    if !progressed {
                        std::thread::sleep(Duration::from_millis(50));
                    }
                }
                Err(_) => break,
            }
        }

        Ok(output)
    }

    /// Kill the QEMU process
    pub fn kill(&mut self) {
        kill_process_group(&mut self.handle);
    }

    /// Check if QEMU is still running
    pub fn is_running(&mut self) -> bool {
        matches!(self.handle.try_wait(), Ok(None))
    }

    /// Start QEMU with ARM virt machine (Cortex-A7 + virtio-net + slirp networking)
    ///
    /// Used for NuttX QEMU tests. The virt machine provides a virtio-net interface
    /// using QEMU's user-mode (slirp) networking. Each instance gets its own
    /// isolated NAT stack — no TAP devices, no bridge, no sudo needed.
    ///
    /// # Arguments
    /// * `binary` - Path to the NuttX ELF binary (kernel + app)
    /// * `networking` - `true` for slirp networking, `false` for no NIC (boot tests)
    ///
    /// # Returns
    /// A managed QEMU process
    /// Phase 97.4.nuttx — same shape as `start_mps2_an385_mcast` but for
    /// NuttX's `qemu-system-arm -M virt -cpu cortex-a7` machine and
    /// virtio-net-device. Both sibling instances share the host
    /// `-netdev socket,mcast=…` segment so SPDP / SEDP / pubsub frames
    /// cross between them. Caller picks distinct MAC addrs.
    pub fn start_nuttx_virt_mcast(
        binary: &Path,
        mcast_addr_port: &str,
        mac: &str,
    ) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Binary not found: {}",
                binary.display()
            )));
        }

        let mut cmd = qemu_system_arm_cmd();
        cmd.args([
            "-M",
            "virt",
            "-cpu",
            "cortex-a7",
            "-nographic",
            "-icount",
            "shift=auto",
            "-kernel",
        ])
        .arg(binary)
        .args([
            "-netdev",
            // Phase 127.B.5 — QEMU 6.2's `net/socket.c` picks the
            // mcast egress interface from the host routing table when
            // `localaddr=` is omitted, which routes to a real LAN NIC
            // and prevents cross-process delivery on the same host.
            // Pin egress to lo so the host kernel loops mcast back
            // to the sibling QEMU. Requires
            // `sudo ip route add 230.10.0.0/16 dev lo` on the host;
            // `require_mcast_route` below prints a clear hint when
            // missing.
            &format!("socket,id=net0,mcast={mcast_addr_port},localaddr=127.0.0.1"),
            "-device",
            &format!("virtio-net-device,netdev=net0,mac={mac}"),
        ])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        Self::spawn(cmd)
    }

    /// Phase 127.B.5 — QEMU 7.2+ `-netdev dgram,local.type=unix,…`
    /// peer pair. Replaces the lossy `socket,mcast=…` cross-process
    /// path with an AF_UNIX SOCK_DGRAM tunnel between exactly two
    /// QEMU processes. No root, no routes, no IGMP — frames sent to
    /// `remote_unix_path` arrive at the peer's `local_unix_path` and
    /// vice versa.
    ///
    /// Caller is responsible for picking two distinct unique
    /// per-pair paths and deleting any stale files at those paths
    /// before launch (QEMU bind fails on EADDRINUSE).
    pub fn start_nuttx_virt_dgram(
        binary: &Path,
        local_unix_path: &str,
        remote_unix_path: &str,
        mac: &str,
    ) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Binary not found: {}",
                binary.display()
            )));
        }

        let mut cmd = qemu_system_arm_cmd();
        cmd.args([
            "-M",
            "virt",
            "-cpu",
            "cortex-a7",
            "-nographic",
            // Phase 160.K — NO `-icount shift=auto`. The MPS2 / slirp
            // sibling sets icount to keep CMSDK Timer0 aligned with
            // host slirp I/O during WFI. NuttX virt + AF_UNIX dgram
            // pair has no slirp dependency and runs under wall-clock
            // virtio-net. Icount tied virtual time to instruction
            // count, so heavy RTPS bursts (SPDP+SEDP+
            // reliability) advanced virtual time by milliseconds per
            // wall-second — the 1 Hz publish timer fired ~once per
            // 30 wall-seconds, breaking the test's 60 s budget. With
            // icount off, wall-clock virtio + NuttX kernel timing
            // align and the publish loop ticks at the configured
            // rate.
            "-kernel",
        ])
        .arg(binary)
        .args([
            "-netdev",
            &format!(
                "dgram,id=net0,\
                 local.type=unix,local.path={local_unix_path},\
                 remote.type=unix,remote.path={remote_unix_path}"
            ),
            "-device",
            &format!("virtio-net-device,netdev=net0,mac={mac}"),
        ])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        Self::spawn(cmd)
    }

    pub fn start_nuttx_virt(binary: &Path, networking: bool) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Binary not found: {}",
                binary.display()
            )));
        }

        let mut cmd = qemu_system_arm_cmd();
        cmd.args([
            "-M",
            "virt",
            "-cpu",
            "cortex-a7",
            "-nographic",
            // Sync virtual clock with wall-clock so sleep()/timeouts run
            // at real-time rates (matches the interactive `just nuttx
            // talker` recipe in just/nuttx.just).
            "-icount",
            "shift=auto",
            "-kernel",
        ])
        .arg(binary);

        if networking {
            // NuttX's defconfig enables CONFIG_DRIVERS_VIRTIO_MMIO, so the
            // NIC must be attached to the virtio-mmio transport — not
            // virtio-net-pci, which is what `-nic user` defaults to on the
            // ARM virt machine. Use explicit `-netdev user` + `-device
            // virtio-net-device` to force the MMIO path so NuttX's driver
            // actually probes and configures the interface.
            cmd.args([
                "-netdev",
                "user,id=net0",
                "-device",
                "virtio-net-device,netdev=net0",
            ]);
        } else {
            cmd.args(["-nic", "none"]);
        }

        cmd.stdout(Stdio::piped()).stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        Self::spawn(cmd)
    }

    /// #165 — boot a riscv32 rv-virt NuttX kernel image (the riscv sibling of
    /// [`Self::start_nuttx_virt`]). Command mirrors the canonical rv-virt export
    /// `scripts/nuttx/build-nuttx.sh` emits: `qemu-system-riscv32 -M virt -bios
    /// none` + a virtio-net-device on the MMIO transport (NuttX's rv-virt
    /// defconfig enables `CONFIG_DRIVERS_VIRTIO_MMIO`, so — as on arm-virt — the
    /// NIC must be `-device virtio-net-device`, not the PCI default `-nic user`).
    /// `-icount shift=auto` syncs the virtual clock to wall-clock so `sleep()` /
    /// timeouts run at real rates, matching `start_nuttx_virt`.
    pub fn start_nuttx_riscv(binary: &Path, networking: bool) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Binary not found: {}",
                binary.display()
            )));
        }

        let mut cmd = Command::new("qemu-system-riscv32");
        cmd.args([
            "-M",
            "virt",
            "-bios",
            "none",
            "-nographic",
            "-icount",
            "shift=auto",
            "-kernel",
        ])
        .arg(binary);

        if networking {
            cmd.args([
                "-netdev",
                "user,id=net0",
                "-device",
                "virtio-net-device,netdev=net0",
            ]);
        } else {
            cmd.args(["-nic", "none"]);
        }

        cmd.stdout(Stdio::piped()).stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        Self::spawn(cmd)
    }

    /// Start QEMU with RISC-V 64-bit virt machine + virtio-net slirp networking
    ///
    /// Used for ThreadX QEMU RISC-V tests. The virt machine provides a virtio-net
    /// MMIO interface using QEMU's user-mode (slirp) networking. Each instance
    /// gets its own isolated NAT stack — no TAP devices, no bridge, no sudo needed.
    ///
    /// The `peer_index` selects the MAC address:
    /// - 0: MAC 52:54:00:12:34:56 (talker/server)
    /// - 1: MAC 52:54:00:12:34:57 (listener/client)
    ///
    /// MAC addresses use the QEMU OUI range (52:54:00) with the last byte
    /// derived from the peer index (0x56 + index). These must match the
    /// firmware's `Config::default()` / `Config::listener()` presets in
    /// `nros-board-threadx-qemu-riscv64`.
    /// Phase 97.4.threadx-riscv64 — sibling-mcast variant. Two
    /// QEMU instances share `-netdev socket,mcast=…` so SPDP /
    /// SEDP / pubsub frames cross between them on the host's
    /// loopback / primary iface (no `localaddr` — same lesson as
    /// the FreeRTOS slice).
    pub fn start_riscv64_virt_mcast(
        binary: &Path,
        mcast_addr_port: &str,
        mac: &str,
    ) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Binary not found: {}",
                binary.display()
            )));
        }

        let mut cmd = Command::new("qemu-system-riscv64");
        cmd.args([
            "-M",
            "virt",
            "-m",
            "256M",
            "-bios",
            "none",
            "-nographic",
            "-global",
            "virtio-mmio.force-legacy=false",
            "-kernel",
        ])
        .arg(binary)
        .args([
            "-netdev",
            &format!("socket,id=net0,mcast={mcast_addr_port}"),
            "-device",
            &format!("virtio-net-device,netdev=net0,bus=virtio-mmio-bus.0,mac={mac}"),
        ])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        Self::spawn(cmd)
    }

    /// Phase 127.B.5 — QEMU 7.2+ `-netdev dgram,local.type=unix,…`
    /// peer pair for RV64 ThreadX. Same shape as
    /// `start_nuttx_virt_dgram` — avoids QEMU's broken cross-process
    /// `socket,mcast=` delivery by routing two QEMU processes at each
    /// other via an AF_UNIX SOCK_DGRAM pair.
    pub fn start_riscv64_virt_dgram(
        binary: &Path,
        local_unix_path: &str,
        remote_unix_path: &str,
        mac: &str,
    ) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Binary not found: {}",
                binary.display()
            )));
        }

        let mut cmd = Command::new("qemu-system-riscv64");
        cmd.args([
            "-M",
            "virt",
            "-m",
            "256M",
            "-bios",
            "none",
            "-nographic",
            "-global",
            "virtio-mmio.force-legacy=false",
            "-kernel",
        ])
        .arg(binary)
        .args([
            "-netdev",
            &format!(
                "dgram,id=net0,\
                 local.type=unix,local.path={local_unix_path},\
                 remote.type=unix,remote.path={remote_unix_path}"
            ),
            "-device",
            &format!("virtio-net-device,netdev=net0,bus=virtio-mmio-bus.0,mac={mac}"),
        ])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        Self::spawn(cmd)
    }

    pub fn start_riscv64_virt(binary: &Path, peer_index: u8) -> TestResult<Self> {
        if !binary.exists() {
            return Err(TestError::BuildFailed(format!(
                "Binary not found: {}",
                binary.display()
            )));
        }

        let mac = format!("52:54:00:12:34:{:02x}", 0x56u8 + peer_index);

        let mut cmd = Command::new("qemu-system-riscv64");
        cmd.args([
            "-M",
            "virt",
            "-m",
            "256M",
            "-bios",
            "none",
            "-nographic",
            "-global",
            "virtio-mmio.force-legacy=false",
            "-kernel",
        ])
        .arg(binary);

        cmd.args(["-netdev", "user,id=net0"]);
        let device_arg = format!(
            "virtio-net-device,netdev=net0,bus=virtio-mmio-bus.0,mac={}",
            mac
        );
        cmd.args(["-device", &device_arg]);

        cmd.stdout(Stdio::piped()).stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        Self::spawn(cmd)
    }
}

impl Drop for QemuProcess {
    fn drop(&mut self) {
        kill_process_group(&mut self.handle);
    }
}

/// Parse test results from QEMU semihosting output
///
/// Looks for `[PASS]` and `[FAIL]` markers.
///
/// # Returns
/// (passed_count, failed_count)
pub fn parse_test_results(output: &str) -> (usize, usize) {
    let passed = output.matches("[PASS]").count();
    let failed = output.matches("[FAIL]").count();
    (passed, failed)
}

/// Check if the veth bridge network is available for ThreadX Linux simulation.
///
/// Verifies that the `qemu-br` bridge and at least `veth-tx0` + `veth-tx1`
/// interfaces exist. These are created by `sudo ./scripts/qemu/setup-network.sh`.
///
/// ThreadX Linux uses veth pairs (not TAP devices) because the NetX Duo Linux
/// network driver uses AF_PACKET/SOCK_RAW, which doesn't work correctly on TAP
/// devices with a bridge. veth pairs are purely kernel-side and route traffic
/// through the bridge correctly.
pub fn is_veth_bridge_available() -> bool {
    let bridge_exists = std::path::Path::new("/sys/class/net/qemu-br").exists();
    let veth0_exists = std::path::Path::new("/sys/class/net/veth-tx0").exists();
    let veth1_exists = std::path::Path::new("/sys/class/net/veth-tx1").exists();
    bridge_exists && veth0_exists && veth1_exists
}

/// Skip test if veth bridge is not available for ThreadX Linux simulation
pub fn require_veth_bridge() -> bool {
    if !is_veth_bridge_available() {
        eprintln!("Skipping test: veth bridge not available for ThreadX Linux");
        eprintln!("Setup: sudo ./scripts/qemu/setup-network.sh");
        return false;
    }
    true
}

/// Phase 127.B.5 — check whether the host has a route for the given
/// IPv4 multicast group via the loopback interface.
///
/// QEMU's `-netdev socket,mcast=…,localaddr=127.0.0.1` puts the mcast
/// egress on `lo`, but the kernel only loops the frame back to a
/// sibling QEMU's joined socket if a route for the group is present
/// on `lo`. Without that route the kernel still picks the default
/// route (a real LAN NIC), and cross-process delivery silently fails.
///
/// Returns `true` if `ip route show <group>` reports `dev lo`.
pub fn is_mcast_loopback_route_present(group: &str) -> bool {
    let group_only = group.split(':').next().unwrap_or(group);
    /* `ip route show <addr>` only matches an exact-destination route
     * entry; use `ip route get <addr>` so a prefix route like
     * `230.10.0.0/16 dev lo` is recognised when the test picks
     * `230.10.0.137` from inside it. */
    let out = Command::new("ip")
        .args(["route", "get", group_only])
        .output();
    match out {
        Ok(o) if o.status.success() => {
            let s = String::from_utf8_lossy(&o.stdout);
            s.contains(" dev lo ") || s.contains(" dev lo\n") || s.trim_end().ends_with(" dev lo")
        }
        _ => false,
    }
}

/// Phase 127.B.5 — print a clear hint when the host lacks a loopback
/// route for the test's multicast group. Returns `true` if the route
/// is present (test may proceed), `false` otherwise. Tests should
/// `nros_tests::skip!` (or equivalent panic-skip) on `false`.
pub fn require_mcast_loopback_route(group: &str) -> bool {
    let group_only = group.split(':').next().unwrap_or(group);
    if is_mcast_loopback_route_present(group_only) {
        return true;
    }
    eprintln!(
        "Skipping test: host route for multicast group {group_only} not on lo.\n\
         QEMU `-netdev socket,mcast=…,localaddr=127.0.0.1` needs the kernel\n\
         to loop the egress back to the sibling QEMU; without a `dev lo`\n\
         route the default route picks a real LAN NIC and cross-process\n\
         delivery silently drops every peer's frame (Phase 127.B.5).\n\
         One-time host setup (root):\n\
         \n\
         sudo ip route add 230.10.0.0/16 dev lo\n\
         sudo ip route add 239.0.0.0/8   dev lo\n\
         \n\
         To make it survive a reboot, drop the same commands into a\n\
         `systemd-networkd` unit, NetworkManager dispatcher script, or\n\
         `/etc/network/if-up.d/`."
    );
    false
}

/// Check if QEMU RISC-V 64-bit is available
pub fn is_qemu_riscv64_available() -> bool {
    Command::new("qemu-system-riscv64")
        .arg("--version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Phase 127.B.5 — does the host's `qemu-system-arm` support
/// `-netdev dgram,local.type=unix,…` (the AF_UNIX peer-pair
/// netdev that replaces the lossy `socket,mcast=` cross-process
/// path)? `dgram` was added in QEMU 7.2.
pub fn qemu_supports_dgram_unix() -> bool {
    qemu_cmd_supports_dgram(qemu_system_arm_cmd())
}

/// `-netdev dgram` support for `qemu-system-riscv64` specifically. The
/// patched ARM binary (stable-11.0) always has it, but the RISC-V64 binary is
/// the unpatched system one — on this host QEMU 6.2, which predates `dgram`
/// (added in 7.2). Checking the ARM binary's help here would be wrong, so the
/// ThreadX-RV64 two-node test gates on this and falls back to `socket,mcast`.
pub fn qemu_riscv64_supports_dgram_unix() -> bool {
    qemu_cmd_supports_dgram(Command::new("qemu-system-riscv64"))
}

/// `-netdev help` lists backend types one per line; "dgram" appears iff
/// QEMU >= 7.2.
fn qemu_cmd_supports_dgram(mut cmd: Command) -> bool {
    match cmd.args(["-M", "virt", "-netdev", "help"]).output() {
        Ok(o) => {
            String::from_utf8_lossy(&o.stdout)
                .lines()
                .any(|l| l.trim() == "dgram")
                || String::from_utf8_lossy(&o.stderr)
                    .lines()
                    .any(|l| l.trim() == "dgram")
        }
        Err(_) => false,
    }
}

/// Check if QEMU ARM is available
pub fn is_qemu_available() -> bool {
    qemu_system_arm_cmd()
        .arg("--version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Check if ARM toolchain is available
pub fn is_arm_toolchain_available() -> bool {
    Command::new("rustup")
        .args(["target", "list", "--installed"])
        .output()
        .map(|o| String::from_utf8_lossy(&o.stdout).contains("thumbv7m-none-eabi"))
        .unwrap_or(false)
}

/// Check if zenoh-pico ARM library is available
///
/// The BSP examples require the zenoh-pico library to be cross-compiled for
/// ARM Cortex-M3.
///
/// Built by `just qemu build-zenoh-pico` (which runs
/// `scripts/qemu/build-zenoh-pico.sh`), or by `just qemu setup`, which calls it.
///
/// Issue 0483 — this used to say `just build-zenoh-pico-arm`, a recipe that has
/// never existed. All 16 `emulator.rs` tests skip when the library is absent, so
/// the suite reported a green that meant "did not run" and the printed remedy
/// could not be followed. `just qemu build-fixtures` does NOT build it, which is
/// how a developer reaches that state without doing anything wrong. Measured
/// after building it by hand: 0 passed/16 skipped in ~1 s became 16 passed/0
/// skipped in 116 s.
pub fn is_zenoh_pico_arm_available() -> bool {
    // Check if the library exists at the expected location relative to project root
    let lib_path = crate::build_dir(crate::kind::QEMU_ZENOH_PICO, &[]).join("libzenohpico.a");
    lib_path.exists()
}

/// Skip test if zenoh-pico ARM library is not available
pub fn require_zenoh_pico_arm() -> bool {
    if !is_zenoh_pico_arm_available() {
        eprintln!("Skipping test: libzenohpico.a not found");
        eprintln!("Build with: just qemu build-zenoh-pico   (or: just qemu setup)");
        eprintln!("Note: `just qemu build-fixtures` does NOT build it (issue 0483).");
        return false;
    }
    true
}

/// A socat-managed virtual serial pair (two linked PTYs).
///
/// Creates a bidirectional PTY pair via `socat`. Data written to one
/// end appears on the other. Both ends are exposed as symlinks for
/// stable paths.
///
/// Use this to wire QEMU's UART0 to zenohd's serial listener without
/// timing races: the PTY pair exists before either side starts, so
/// data is kernel-buffered.
///
/// # Example
///
/// ```ignore
/// let pair = SocatPtyPair::create("/tmp/serial-qemu", "/tmp/serial-zenohd")?;
/// // QEMU connects to /tmp/serial-qemu
/// // zenohd listens on /tmp/serial-zenohd
/// // pair is killed on drop
/// ```
pub struct SocatPtyPair {
    handle: Child,
    /// Path for the QEMU side
    pub qemu_path: String,
    /// Path for the zenohd side
    pub zenohd_path: String,
}

impl SocatPtyPair {
    /// Create a new PTY pair with symlinks at the given paths.
    ///
    /// Blocks until both symlinks exist (up to 5 seconds).
    pub fn create(qemu_link: &str, zenohd_link: &str) -> TestResult<Self> {
        // Remove stale symlinks from previous runs
        let _ = std::fs::remove_file(qemu_link);
        let _ = std::fs::remove_file(zenohd_link);

        let pty_a = format!("PTY,raw,echo=0,link={}", qemu_link);
        let pty_b = format!("PTY,raw,echo=0,link={}", zenohd_link);

        let mut cmd = Command::new("socat");
        cmd.args([&pty_a, &pty_b])
            .stdout(Stdio::null())
            .stderr(Stdio::null());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        let handle = cmd.spawn()?;

        // Wait for symlinks to appear
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(5) {
            if Path::new(qemu_link).exists() && Path::new(zenohd_link).exists() {
                // Small delay for socat to finish setting up the PTY
                std::thread::sleep(Duration::from_millis(100));
                return Ok(Self {
                    handle,
                    qemu_path: qemu_link.to_string(),
                    zenohd_path: zenohd_link.to_string(),
                });
            }
            std::thread::sleep(Duration::from_millis(50));
        }

        Err(TestError::ProcessFailed(
            "Timeout waiting for socat PTY symlinks".to_string(),
        ))
    }
}

impl Drop for SocatPtyPair {
    fn drop(&mut self) {
        kill_process_group(&mut self.handle);
        let _ = std::fs::remove_file(&self.qemu_path);
        let _ = std::fs::remove_file(&self.zenohd_path);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_results() {
        let output = "[PASS] test1\n[PASS] test2\n[FAIL] test3\n[PASS] test4";
        let (passed, failed) = parse_test_results(output);
        assert_eq!(passed, 3);
        assert_eq!(failed, 1);
    }

    #[test]
    fn test_parse_results_empty() {
        let (passed, failed) = parse_test_results("");
        assert_eq!(passed, 0);
        assert_eq!(failed, 0);
    }

    /// Issue 0877 — the recorded line must name the RESOLVED program, because
    /// that is the half a hand run gets wrong. Asserting only on the arguments
    /// would reproduce the mistake the record exists to prevent.
    #[test]
    fn recorded_command_names_the_resolved_program_not_the_bare_name() {
        let mut cmd = qemu_system_arm_cmd();
        cmd.args(["-machine", "mps2-an385"]);
        let rendered = render_command(&cmd);
        let resolved = qemu_system_arm_path();
        let resolved = resolved.to_string_lossy();
        assert!(
            rendered.starts_with(resolved.as_ref()),
            "recorded line must start with the resolved program `{resolved}`, got: {rendered}"
        );
        assert!(rendered.ends_with("-machine mps2-an385"), "{rendered}");
    }

    /// A pasteable line, not merely a readable one: anything a shell would
    /// re-interpret has to come back quoted.
    #[test]
    fn recorded_command_quotes_arguments_a_shell_would_split() {
        let mut cmd = Command::new("/usr/bin/qemu-system-arm");
        cmd.args([
            "-nic",
            "user,model=lan9118,net=192.0.3.0/24,host=192.0.3.1",
            "-semihosting-config",
            "enable=on,target=native",
            "-append",
            "two words",
        ]);
        let rendered = render_command(&cmd);
        // Commas, slashes and `=` are safe — quoting them would make the line
        // harder to read for no gain.
        assert!(
            rendered.contains("-nic user,model=lan9118,net=192.0.3.0/24,host=192.0.3.1"),
            "{rendered}"
        );
        assert!(rendered.contains("'two words'"), "{rendered}");
    }
}
