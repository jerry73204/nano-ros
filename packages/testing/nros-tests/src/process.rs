//! Managed process utilities for integration tests
//!
//! Provides RAII-based process management with automatic cleanup.
//! All child processes are spawned in their own process group so that
//! `kill_process_group()` can reap the entire tree (bash + children).

use crate::TestError;
use std::{
    net::TcpListener,
    process::{Child, Command, Stdio},
    time::Duration,
};

/// Wait for a file descriptor to become readable, or sleep on non-Unix.
///
/// Uses `poll(2)` on Unix to avoid busy-waiting.
#[cfg(unix)]
fn poll_or_sleep(fd: std::os::unix::io::RawFd, remaining: Duration) {
    let ms = remaining.as_millis().min(500) as i32;
    let mut fds = [libc::pollfd {
        fd,
        events: libc::POLLIN,
        revents: 0,
    }];
    unsafe {
        libc::poll(fds.as_mut_ptr(), 1, ms);
    }
}

#[cfg(not(unix))]
fn poll_or_sleep(remaining: Duration) {
    std::thread::sleep(remaining.min(Duration::from_millis(50)));
}

/// Configure a Command to spawn the child in its own process group.
///
/// This ensures that `kill_process_group()` can kill the child and all
/// its descendants (e.g., bash → timeout → ros2).
///
/// On Linux, also sets `PR_SET_PDEATHSIG(SIGKILL)` so the child is killed
/// when the parent dies — prevents orphans when nextest SIGKILL's the test binary.
#[cfg(unix)]
pub fn set_new_process_group(command: &mut Command) -> &mut Command {
    use std::os::unix::process::CommandExt;
    // SAFETY: setpgid and prctl are async-signal-safe and called before exec
    unsafe {
        command.pre_exec(|| {
            libc::setpgid(0, 0);
            #[cfg(target_os = "linux")]
            {
                libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGKILL);
            }
            Ok(())
        })
    }
}

/// Issue 0923 — spawn a child that takes its whole process group down with it
/// when the parent dies, instead of leaving the reaping to the next lane.
///
/// [`set_new_process_group`] sets `PR_SET_PDEATHSIG(SIGKILL)`, which reaps
/// exactly one level: `bash` dies and `timeout`/`ros2`/the node reparent to
/// init. That is not a fixable oversight — SIGKILL cannot be handled, so the
/// dying process cannot pass the news on — and it is why the ledger sweep
/// exists. But the sweep only runs at lane start, so between a SIGKILLed run
/// and the next lane the peers are alive, holding DDS discovery ports.
///
/// So this variant asks for **SIGTERM**, which `bash` CAN trap, and pairs with
/// [`group_suicide_wrapper`] on the command text: parent dies -> kernel SIGTERMs
/// `bash` -> the trap kills the whole group -> `ros2` and the node die too, at
/// that moment.
///
/// The trade is deliberate and bounded: a peer that IGNORES SIGTERM survives
/// where SIGKILL would have reaped it, which is why the wrapper follows up with
/// `SIGKILL` after a grace period and why the ledger sweep stays as the
/// backstop. Use this for `bash -c` peers; direct children keep
/// [`set_new_process_group`].
#[cfg(unix)]
pub fn set_orphan_group_suicide(command: &mut Command) -> &mut Command {
    use std::os::unix::process::CommandExt;
    // SAFETY: setpgid, prctl, getppid and _exit are async-signal-safe and are
    // called between fork and exec.
    unsafe {
        command.pre_exec(|| {
            libc::setpgid(0, 0);
            #[cfg(target_os = "linux")]
            {
                libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGTERM);
                // The race the flag cannot cover: if the parent died between
                // fork and the prctl above, the signal was already delivered to
                // nobody and never will be. Reparenting to init is the
                // observable, so check for it and leave rather than run a peer
                // nothing will ever reap.
                if libc::getppid() == 1 {
                    libc::_exit(0);
                }
            }
            Ok(())
        })
    }
}

/// Wrap a `bash -c` command so it kills its own process group on the way out.
///
/// The other half of [`set_orphan_group_suicide`]. `kill -TERM -- -0` addresses
/// the caller's whole process group, which `setpgid(0, 0)` has just made
/// exclusively ours — so this reaps `timeout`, `ros2` and the node, the
/// descendants `PR_SET_PDEATHSIG` cannot reach.
///
/// Traps EXIT as well as the signals: a peer whose command simply returns
/// should not leave a straggler either.
pub fn group_suicide_wrapper(cmd: &str) -> String {
    // `kill 0` is the whole group. The KILL follow-up bounds a peer that
    // ignores TERM; the `|| true` keeps a normal exit — where everything is
    // already gone — from failing the script.
    format!(
        "_nros_reap() {{ trap - EXIT INT TERM; kill -TERM 0 2>/dev/null || true; \
         sleep 0.3; kill -KILL 0 2>/dev/null || true; }}; \
         trap _nros_reap EXIT INT TERM; {cmd}"
    )
}

/// Issue 0659 — a durable record of the process groups this run spawned, so a
/// LATER run can reap what a SIGKILL left behind.
///
/// `PR_SET_PDEATHSIG` above covers exactly one level: it kills `bash`, and
/// `timeout`/`ros2`/the node reparent to init. Nothing inside the tree can do
/// better, because PDEATHSIG delivers SIGKILL and SIGKILL cannot be handled — so
/// the cleanup has to be performed by something that is not being killed. That
/// is this ledger plus [`sweep_orphaned_process_groups`], run at lane start.
///
/// Measured 2026-08-17: 59 orphaned `add_two_ints_server` on one host, oldest
/// 9.4 days, holding domain-5 discovery ports until an unrelated cyclone test
/// failed with `failed to bind to ANY:8650: address in use`.
///
/// **Linux, not unix.** Every question this module asks is asked of `/proc`:
/// `start_time` reads `/proc/<pid>/stat` field 22 and `members` enumerates
/// `/proc` to match field 3 against a pgid. procfs is a Linux filesystem —
/// FreeBSD does not mount one by default, and the one it can mount has no
/// `stat` file — so under the old `cfg(unix)` guard this compiled on the BSDs
/// and then recorded nothing and swept nothing, while its own test asserted on
/// `/proc/<pid>/comm`. There is no portable substitute: enumerating a process
/// group's members needs procfs or a kernel-specific interface (`kvm`/`sysctl`
/// on the BSDs), so the honest gate is the one that names the interface used.
/// A non-Linux unix therefore has no orphan sweep, which is what it effectively
/// had before.
#[cfg(target_os = "linux")]
pub mod group_ledger {
    use std::{fs, path::PathBuf};

    fn dir() -> PathBuf {
        // Under `build/`, which is gitignored and already the home for run
        // state. `NROS_PEER_LEDGER_DIR` exists so the tests for this can use a
        // scratch dir instead of the real one.
        std::env::var_os("NROS_PEER_LEDGER_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|| crate::project_root().join("build/test-peer-groups"))
    }

    /// Boot-relative start time of `pid` (field 22 of `/proc/<pid>/stat`), the
    /// only cheap thing that distinguishes a pid from a RECYCLED pid.
    ///
    /// Parsed from the last `)` rather than by splitting on whitespace: field 2
    /// is the comm, which is parenthesised and may itself contain spaces.
    pub fn start_time(pid: i32) -> Option<u64> {
        let stat = fs::read_to_string(format!("/proc/{pid}/stat")).ok()?;
        let tail = &stat[stat.rfind(')')? + 1..];
        tail.split_whitespace().nth(19)?.parse().ok()
    }

    /// Record a spawned group. `pgid` equals the child's pid because
    /// `set_new_process_group` made it its own leader.
    pub fn record(pgid: i32, label: &str) {
        record_in(&dir(), pgid, label)
    }

    /// Directory-explicit form. Tests use this rather than an env var: cargo
    /// runs a crate's tests as THREADS of one process, so two tests setting the
    /// same global would race, and the loser's sweep would look in the winner's
    /// directory, find nothing, and pass vacuously. That is not hypothetical —
    /// it happened here, and it made the recycled-pgid safety test green while
    /// the check it guards was mutated away.
    pub fn record_in(d: &std::path::Path, pgid: i32, label: &str) {
        let Some(started) = start_time(pgid) else {
            return;
        };
        if fs::create_dir_all(d).is_err() {
            return;
        }
        // The leader's start time is the FLOOR for its descendants: a process
        // cannot predate the group it belongs to. That is what makes a recycled
        // pgid safe — its members would predate this.
        let _ = fs::write(d.join(pgid.to_string()), format!("{started}\n{label}\n"));
    }

    /// Forget a group the orderly path already killed.
    pub fn forget(pgid: i32) {
        let _ = fs::remove_file(dir().join(pgid.to_string()));
    }

    /// Directory-explicit form, for the same reason as [`record_in`].
    pub fn sweep_in(d: &std::path::Path) -> usize {
        sweep_dir(d)
    }

    /// Members of `pgid` still alive, as `(pid, start_time)`.
    fn members(pgid: i32) -> Vec<(i32, u64)> {
        let mut out = Vec::new();
        let Ok(entries) = fs::read_dir("/proc") else {
            return out;
        };
        for e in entries.flatten() {
            let Ok(pid) = e.file_name().to_string_lossy().parse::<i32>() else {
                continue;
            };
            let Ok(stat) = fs::read_to_string(format!("/proc/{pid}/stat")) else {
                continue;
            };
            let Some(tail) = stat.rfind(')').map(|i| &stat[i + 1..]) else {
                continue;
            };
            let f: Vec<&str> = tail.split_whitespace().collect();
            // after the comm: state(0) ppid(1) pgrp(2) … starttime(19)
            let (Some(pg), Some(st)) = (
                f.get(2).and_then(|v| v.parse::<i32>().ok()),
                f.get(19).and_then(|v| v.parse::<u64>().ok()),
            ) else {
                continue;
            };
            if pg == pgid {
                out.push((pid, st));
            }
        }
        out
    }

    /// Kill every recorded group that still has members, and drop the record.
    ///
    /// Returns the number of groups killed. Verification before killing is the
    /// whole point: this issue also records a cleanup that matched on process
    /// NAME and killed 26 live Autoware components. A group is killed only when
    /// EVERY surviving member started at or after the recorded leader — a
    /// recycled pgid hosting an older process is therefore skipped, not killed.
    pub fn sweep() -> usize {
        sweep_dir(&dir())
    }

    fn sweep_dir(d: &std::path::Path) -> usize {
        let Ok(entries) = fs::read_dir(d) else {
            return 0;
        };
        let mut killed = 0;
        for e in entries.flatten() {
            let Ok(pgid) = e.file_name().to_string_lossy().parse::<i32>() else {
                continue;
            };
            let recorded = fs::read_to_string(e.path())
                .ok()
                .and_then(|t| t.lines().next()?.parse::<u64>().ok());
            let Some(floor) = recorded else {
                let _ = fs::remove_file(e.path());
                continue;
            };
            let live = members(pgid);
            if live.is_empty() {
                let _ = fs::remove_file(e.path());
                continue;
            }
            if live.iter().all(|(_, st)| *st >= floor) {
                // SAFETY: negative pid signals the process group.
                unsafe {
                    libc::kill(-pgid, libc::SIGKILL);
                }
                killed += 1;
                let _ = fs::remove_file(e.path());
            }
            // else: pgid was recycled — leave both the processes and the record
            // alone rather than guess.
        }
        killed
    }
}

/// Sweep process groups left behind by a previous, SIGKILLed run.
///
/// Linux-gated with [`group_ledger`], whose `/proc` dependency it inherits.
#[cfg(target_os = "linux")]
pub fn sweep_orphaned_process_groups() -> usize {
    group_ledger::sweep()
}

// `/proc/<pid>/comm` and `/proc/<pid>/stat` are read directly below, so this
// follows `group_ledger`'s gate rather than widening it back to `unix`.
#[cfg(all(test, target_os = "linux"))]
mod group_ledger_tests {
    use super::*;
    use std::process::{Command, Stdio};

    /// Issue 0659's own acceptance test: SIGKILL the supervisor, then assert no
    /// descendant survives. The previous shapes (a bash EXIT trap, `exec
    /// timeout`) both PASS a "clean teardown" test and both fail this one,
    /// which is why it is written this way round.
    #[test]
    fn a_sigkilled_supervisor_leaves_a_group_the_sweep_reaps() {
        let tmp = tempfile::tempdir().expect("tmpdir");

        // bash -> timeout -> bash -> sleep, mirroring the real chain
        // `bash -c "<env> && timeout N ros2 run <pkg> <node>"`, which is FOUR
        // deep: the python `ros2` launcher spawns the node.
        //
        // Both shortcuts the earlier shape took are load-bearing, and both were
        // measured wrong on this host (issue 0659 follow-up, bash 5.3.15 +
        // coreutils 9.11):
        //
        //   * "`true &&` stops bash EXECing away" — it does not. bash execs the
        //     last command of an `&&` list, and did so for `export X=1 && …`
        //     too, so the pgid leader was `timeout` and there was no supervisor
        //     to kill. The trailing `; :` is what keeps bash resident, and the
        //     assertion below states that rather than trusting it.
        //   * "`timeout N sleep` leaks its child" — it does not. coreutils
        //     `timeout` takes its child down with it, which is the OPPOSITE of
        //     the property the real leak needs. `ros2` (python) does not, so
        //     the middle process here must be one that also does not: an inner
        //     `bash` that backgrounds its child and waits.
        //
        // `--foreground` is not cosmetic and mirrors the peer spawn: without it
        // `timeout` puts ITSELF in a new process group, so every descendant
        // leaves the group the ledger recorded and the sweep — which matches
        // `/proc/<pid>/stat`'s pgrp against that pgid — cannot see the orphans
        // it exists to reap. Measured: leader pgid 1213064, orphaned
        // timeout/bash/sleep in 1212154. With `--foreground` all four share the
        // recorded group.
        //
        // A unique duration is the marker: passing a token as an extra argv word
        // makes `sleep` exit immediately with "invalid time interval", which
        // silently turns this into a test that always passes.
        let marker = format!("31.{}", std::process::id() % 900 + 100);
        let mut cmd = Command::new("bash");
        cmd.args([
            "-c",
            &format!("true && timeout --foreground 120 bash -c 'sleep {marker} & wait'; :"),
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null());
        set_new_process_group(&mut cmd);
        let mut child = cmd.spawn().expect("spawn");
        let pgid = child.id() as i32;
        group_ledger::record_in(tmp.path(), pgid, "test-peer");

        std::thread::sleep(std::time::Duration::from_millis(400));
        assert!(
            !group_ledger_members_for_test(pgid).is_empty(),
            "harness broken: nothing started, so this test would pass vacuously"
        );

        // The SHAPE this test depends on, asserted rather than assumed. If a
        // future bash execs the compound away, the leader stops being a
        // supervisor and the leak cannot reproduce — which previously surfaced
        // as "the leak did not reproduce" and sent the reader looking for a
        // regression in the fix instead of a change in the shell.
        let leader_comm = std::fs::read_to_string(format!("/proc/{pgid}/comm"))
            .unwrap_or_default()
            .trim()
            .to_string();
        assert_eq!(
            leader_comm, "bash",
            "the process-group leader is `{leader_comm}`, not bash: this shell \
             EXECed the compound away, so there is no supervisor to SIGKILL and \
             the leak this test exists to reproduce cannot happen. Fix the \
             command shape (a trailing `; :` kept bash resident on 5.3.15), do \
             not delete the assertion."
        );

        // SIGKILL the supervisor ONLY — what PDEATHSIG does to bash when the
        // test binary dies. No Drop runs, so nothing kills the group.
        // SAFETY: signalling a pid this test owns.
        unsafe { libc::kill(pgid, libc::SIGKILL) };
        let _ = child.wait();
        std::thread::sleep(std::time::Duration::from_millis(400));

        let orphans = group_ledger_members_for_test(pgid);
        assert!(
            !orphans.is_empty(),
            "the leak did not reproduce, so the sweep below proves nothing — \
             if PDEATHSIG ever grows subtree semantics, delete this test"
        );

        assert_eq!(
            group_ledger::sweep_in(tmp.path()),
            1,
            "sweep must reap the group"
        );
        std::thread::sleep(std::time::Duration::from_millis(400));
        assert!(
            group_ledger_members_for_test(pgid).is_empty(),
            "group survived the sweep: {:?}",
            group_ledger_members_for_test(pgid)
        );
    }

    /// A recycled pgid — members predating the record — must be LEFT ALONE.
    /// This is the property that separates the fix from the cleanup that killed
    /// 26 live Autoware components.
    #[test]
    fn a_group_older_than_its_record_is_not_killed() {
        let tmp = tempfile::tempdir().expect("tmpdir");

        let marker = format!("32.{}", std::process::id() % 900 + 100);
        let mut cmd = Command::new("bash");
        cmd.args(["-c", &format!("true && timeout 120 sleep {marker}")])
            .stdout(Stdio::null())
            .stderr(Stdio::null());
        set_new_process_group(&mut cmd);
        let mut child = cmd.spawn().expect("spawn");
        let pgid = child.id() as i32;

        // Record a floor in the FUTURE: every live member predates it, exactly
        // as they would if this pgid had been recycled.
        let future = group_ledger::start_time(pgid).expect("starttime") + 1_000_000;
        std::fs::write(
            tmp.path().join(pgid.to_string()),
            format!("{future}\nrecycled\n"),
        )
        .expect("write record");

        assert_eq!(
            group_ledger::sweep_in(tmp.path()),
            0,
            "must not kill a recycled pgid"
        );
        assert!(
            !group_ledger_members_for_test(pgid).is_empty(),
            "the group was killed despite predating its record"
        );

        // SAFETY: cleaning up a group this test owns.
        unsafe { libc::kill(-pgid, libc::SIGKILL) };
        let _ = child.wait();
    }

    fn group_ledger_members_for_test(pgid: i32) -> Vec<i32> {
        let out = Command::new("ps")
            .args(["-eo", "pid,pgid", "--no-headers"])
            .output()
            .expect("ps");
        String::from_utf8_lossy(&out.stdout)
            .lines()
            .filter_map(|l| {
                let mut f = l.split_whitespace();
                let pid: i32 = f.next()?.parse().ok()?;
                let pg: i32 = f.next()?.parse().ok()?;
                (pg == pgid).then_some(pid)
            })
            .collect()
    }
}

/// Kill an entire process group immediately with SIGKILL.
///
/// Use for processes that don't need graceful shutdown (e.g., QEMU emulators).
/// For processes with TCP state (zenohd), use [`graceful_kill_process_group`].
#[cfg(unix)]
pub fn kill_process_group(handle: &mut Child) {
    let pid = handle.id() as libc::pid_t;
    unsafe {
        libc::kill(-pid, libc::SIGKILL);
    }
    let _ = handle.wait();
    // issue 0659 — the orderly path just did the cleanup, so drop the record.
    // A ledger that only grows makes every later sweep examine more groups, and
    // each stale entry is one more chance to act on a recycled pgid.
    // (Linux-only, with the `/proc`-backed ledger itself.)
    #[cfg(target_os = "linux")]
    group_ledger::forget(pid);
}

/// Kill a process group gracefully: SIGTERM first, then SIGKILL after timeout.
///
/// Sends SIGTERM to allow graceful shutdown (TCP cleanup, etc.),
/// waits up to 2 seconds, then SIGKILL if still alive. This prevents
/// stale TCP TIME_WAIT entries from blocking subsequent test connections.
#[cfg(unix)]
pub fn graceful_kill_process_group(handle: &mut Child) {
    let pid = handle.id() as libc::pid_t;

    // Try graceful shutdown first (SIGTERM)
    unsafe {
        libc::kill(-pid, libc::SIGTERM);
    }

    // Wait up to 2s for graceful exit
    let start = std::time::Instant::now();
    loop {
        match handle.try_wait() {
            Ok(Some(_)) => return, // exited cleanly
            Ok(None) if start.elapsed() < Duration::from_secs(2) => {
                std::thread::sleep(Duration::from_millis(50));
            }
            _ => break, // timeout or error → force kill
        }
    }

    // Force kill if SIGTERM didn't work
    unsafe {
        libc::kill(-pid, libc::SIGKILL);
    }
    let _ = handle.wait();
    // issue 0659 — BOTH teardown paths drop the record, not just the immediate
    // one. Fixing only `kill_process_group` would leave every gracefully-killed
    // group in the ledger forever, which is this tree's recurring shape: the fix
    // that lands where the symptom was seen.
    // (Linux-only, with the `/proc`-backed ledger itself.)
    #[cfg(target_os = "linux")]
    group_ledger::forget(pid);
}

/// Fallback for non-unix: kill just the direct child.
#[cfg(not(unix))]
pub fn kill_process_group(handle: &mut Child) {
    let _ = handle.kill();
    let _ = handle.wait();
}

/// Fallback for non-unix: same as kill_process_group.
#[cfg(not(unix))]
pub fn graceful_kill_process_group(handle: &mut Child) {
    kill_process_group(handle);
}

/// Managed process with automatic cleanup
///
/// Wraps a child process and ensures it is killed on drop.
/// Used for running talker/listener binaries and other test processes.
///
/// # Example
///
/// ```ignore
/// let mut proc = ManagedProcess::spawn(&binary_path, &["--tcp", "127.0.0.1:7447"], "talker")?;
/// let output = proc.wait_for_output_count("Publishing:", 1, Duration::from_secs(5))?;
/// // Process is automatically killed on drop
/// ```
pub struct ManagedProcess {
    handle: Child,
    name: String,
    /// The exact command this process was started with — see
    /// [`ManagedProcess::command_line`].
    cmdline: String,
}

impl ManagedProcess {
    /// Spawn a new managed process
    ///
    /// # Arguments
    /// * `binary` - Path to the executable
    /// * `args` - Command line arguments
    /// * `name` - Human-readable name for error messages
    ///
    /// Delegates to [`Self::spawn_command`] rather than duplicating its four
    /// lines: two copies of the spawn sequence are two places to change when
    /// the sequence grows, and this file had exactly that shape.
    pub fn spawn(
        binary: &std::path::Path,
        args: &[&str],
        name: impl Into<String>,
    ) -> Result<Self, TestError> {
        let mut cmd = Command::new(binary);
        cmd.args(args);
        Self::spawn_command(cmd, name)
    }

    /// Spawn a process from a Command builder
    ///
    /// # Arguments
    /// * `command` - Pre-configured Command builder
    /// * `name` - Human-readable name for error messages
    ///
    /// # This does NOT pin the DDS bus — and that is deliberate (issue 1137)
    ///
    /// It is the obvious place to put [`crate::dds_isolation::apply_to_command`],
    /// and it would be wrong. The pin is only correct when the PEER is pinned
    /// too ("pin both sides or neither", issue 1009), and one family of peers
    /// cannot be: the `DockerRosEnv` editions lanes run their ROS 2 side inside
    /// a container whose mount namespace cannot reach the host profile path, so
    /// those pairs are symmetric-UNPINNED today and pinning our half here would
    /// break them exactly the way 1137 broke the Cyclone graph cell. So the pin
    /// is applied at the spawn sites whose peer is a host `ros2` process, and
    /// `check-dds-isolation-symmetry` is what keeps a new such site from
    /// forgetting.
    pub fn spawn_command(mut command: Command, name: impl Into<String>) -> Result<Self, TestError> {
        let name = name.into();
        command.stdout(Stdio::piped()).stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut command);
        let cmdline = crate::qemu::render_command(&command);
        let handle = command
            .spawn()
            .map_err(|e| TestError::ProcessFailed(format!("Failed to spawn {}: {}", name, e)))?;

        Ok(Self {
            handle,
            name,
            cmdline,
        })
    }

    /// Get the process name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The command line this process was started with, pasteable into a shell.
    ///
    /// Issue 0877 — the sibling of [`crate::qemu::QemuProcess::command_line`],
    /// and it exists for the same reason: an e2e failure message that says only
    /// "0 messages received" sends the next reader off to reconstruct the run
    /// by hand, and a reconstruction is only evidence if it starts the same
    /// programs. The e2e lanes run with `--failure-output never`, so the record
    /// has to be inside the assertion text.
    #[must_use]
    pub fn command_line(&self) -> &str {
        &self.cmdline
    }

    /// Check if process is still running
    pub fn is_running(&mut self) -> bool {
        matches!(self.handle.try_wait(), Ok(None))
    }

    /// Get mutable access to the underlying Child handle
    ///
    /// Use with caution - modifications may affect cleanup behavior.
    pub fn handle_mut(&mut self) -> &mut Child {
        &mut self.handle
    }

    /// Drain STDOUT until the process exits or `timeout` elapses, then KILL it.
    ///
    /// **A terminal drain, not a wait-for-readiness.** There is no stop
    /// pattern, so a process that keeps running always reaches the timeout, and
    /// reaching it kills the process group. To wait for a process you intend to
    /// KEEP, call [`Self::wait_for_output_pattern`].
    ///
    /// It also reads STDOUT ONLY, so a process logging to stderr produces
    /// nothing here and the early return is unreachable. Issue 0672 was that
    /// pair on the sibling type: a readiness wait that could observe nothing,
    /// which is worse than a sleep — a sleep leaves the process alive, and this
    /// killed the ROS 2 server the test went on to talk to.
    pub fn wait_for_output(&mut self, timeout: Duration) -> Result<String, TestError> {
        use std::io::Read;
        #[cfg(unix)]
        use std::os::unix::io::AsRawFd;

        let start = std::time::Instant::now();
        let mut output = String::new();

        let mut stdout = self
            .handle
            .stdout
            .take()
            .ok_or_else(|| TestError::ProcessFailed(format!("No stdout for {}", self.name)))?;

        // Set non-blocking mode on stdout
        #[cfg(unix)]
        {
            let fd = stdout.as_raw_fd();
            unsafe {
                let flags = libc::fcntl(fd, libc::F_GETFL);
                libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
            }
        }

        let mut buffer = [0u8; 4096];

        #[cfg(unix)]
        let fd = stdout.as_raw_fd();

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
                    // Process exited, read remaining output
                    let _ = stdout.read_to_string(&mut output);
                    break;
                }
                Ok(None) => match stdout.read(&mut buffer) {
                    Ok(0) => {
                        poll_or_sleep(fd, timeout.saturating_sub(start.elapsed()));
                    }
                    Ok(n) => {
                        output.push_str(&String::from_utf8_lossy(&buffer[..n]));
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        poll_or_sleep(fd, timeout.saturating_sub(start.elapsed()));
                    }
                    Err(_) => break,
                },
                Err(_) => break,
            }
        }

        Ok(output)
    }

    /// Wait until `pattern` appears in stdout+stderr — the STRICT wait.
    ///
    /// `Ok(output)` means the pattern appeared, and carries everything printed
    /// up to that point. `Err` means it did not, either because the timeout
    /// elapsed or because the process exited first; the error quotes the output
    /// so the failure explains itself.
    ///
    /// Issue 0471: this used to return `Ok` on BOTH of those failure paths as
    /// long as the process had printed anything at all, so
    /// `wait_for_output_pattern(MARKER, …)?` asserted only "was not completely
    /// silent". A test written against a knowingly-broken fixture passed,
    /// because the error explaining the breakage is itself non-empty output.
    ///
    /// When a missing pattern is NOT a failure — a readiness wait, or a test
    /// that asserts on the content itself — use [`Self::collect_until`], which
    /// says so in its name and hands back the output unconditionally.
    /// Wait until `role` signals readiness, or FAIL (issue 0481 / phase-342).
    ///
    /// The hardened form of [`Self::wait_for_output_pattern`], and the one tests
    /// should reach for. Two things it removes from the call site:
    ///
    /// 1. **Choosing the marker string.** It comes from
    ///    [`crate::output::ready_marker`], which knows that the same demo spells
    ///    readiness differently in rust and in C/C++. Nine sites picked wrong by
    ///    hand and each waited out its whole timeout in silence.
    /// 2. **Deciding whether a timeout matters.** It does. A process that never
    ///    signals readiness has not met the precondition the next line depends
    ///    on, so this panics rather than returning a `Result` the caller can
    ///    drop with `let _ =` — which is precisely how the silence survived.
    ///
    /// Use [`Self::wait_for_output_pattern`] directly only when waiting for
    /// something that is NOT a role's readiness — a payload line, a shutdown
    /// notice — where a timeout may legitimately be tolerable.
    pub fn expect_ready(
        &mut self,
        role: crate::output::DemoRole,
        lang: crate::matrix::Lang,
        timeout: Duration,
    ) -> String {
        let marker = crate::output::ready_marker(role, lang);
        match self.wait_until_pattern(marker, timeout) {
            (out, true) => out,
            (out, false) => panic!(
                "{} never signalled {:?} readiness: no `{}` within {:?}.\nOutput:\n{}",
                self.name, role, marker, timeout, out
            ),
        }
    }

    pub fn wait_for_output_pattern(
        &mut self,
        pattern: &str,
        timeout: Duration,
    ) -> Result<String, TestError> {
        match self.wait_until_pattern(pattern, timeout) {
            (out, true) => Ok(out),
            (out, false) => Err(TestError::ProcessFailed(format!(
                "{} did not print `{}` within {:?}. Output:\n{}",
                self.name, pattern, timeout, out
            ))),
        }
    }

    /// The shared engine behind [`Self::wait_for_output_pattern`] (strict) and
    /// [`Self::collect_until`] (lenient): collect output until `pattern` shows
    /// up, the process exits, or `timeout` elapses.
    ///
    /// Returns the output collected AND whether the pattern actually appeared.
    /// Keeping both facts in one return value is the point — issue 0471 existed
    /// because the single `Result` conflated them, so the only path that carried
    /// the output was also the path that claimed success.
    fn wait_until_pattern(&mut self, pattern: &str, timeout: Duration) -> (String, bool) {
        use std::io::Read;
        #[cfg(unix)]
        use std::os::unix::io::AsRawFd;

        let start = std::time::Instant::now();
        let mut output = String::new();

        let mut stdout = self.handle.stdout.take();
        let mut stderr = self.handle.stderr.take();

        // Set non-blocking mode
        #[cfg(unix)]
        {
            if let Some(ref out) = stdout {
                let fd = out.as_raw_fd();
                unsafe {
                    let flags = libc::fcntl(fd, libc::F_GETFL);
                    libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
                }
            }
            if let Some(ref err) = stderr {
                let fd = err.as_raw_fd();
                unsafe {
                    let flags = libc::fcntl(fd, libc::F_GETFL);
                    libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
                }
            }
        }

        let mut buf = [0u8; 4096];

        loop {
            if start.elapsed() > timeout {
                // Put handles back so drop can still kill the process
                self.handle.stdout = stdout;
                self.handle.stderr = stderr;
                // The loop below checks `contains` every iteration, so reaching
                // the deadline means the pattern is absent — but say it by
                // testing, not by assuming, so a future edit cannot silently
                // make this lie.
                let matched = output.contains(pattern);
                return (output, matched);
            }

            if let Ok(Some(_)) = self.handle.try_wait() {
                if let Some(ref mut out) = stdout {
                    let _ = out.read_to_string(&mut output);
                }
                if let Some(ref mut err) = stderr {
                    let _ = err.read_to_string(&mut output);
                }
                break;
            }

            let mut got_data = false;
            if let Some(ref mut out) = stdout
                && let Ok(n) = out.read(&mut buf)
                && n > 0
            {
                output.push_str(&String::from_utf8_lossy(&buf[..n]));
                got_data = true;
            }
            if let Some(ref mut err) = stderr
                && let Ok(n) = err.read(&mut buf)
                && n > 0
            {
                output.push_str(&String::from_utf8_lossy(&buf[..n]));
                got_data = true;
            }

            if output.contains(pattern) {
                // Put handles back for further use
                self.handle.stdout = stdout;
                self.handle.stderr = stderr;
                return (output, true);
            }

            if !got_data {
                #[cfg(unix)]
                {
                    let remaining = timeout.saturating_sub(start.elapsed());
                    let ms = remaining.as_millis().min(500) as i32;
                    let mut fds = Vec::new();
                    if let Some(ref out) = stdout {
                        fds.push(libc::pollfd {
                            fd: out.as_raw_fd(),
                            events: libc::POLLIN,
                            revents: 0,
                        });
                    }
                    if let Some(ref err) = stderr {
                        fds.push(libc::pollfd {
                            fd: err.as_raw_fd(),
                            events: libc::POLLIN,
                            revents: 0,
                        });
                    }
                    if !fds.is_empty() {
                        unsafe {
                            libc::poll(fds.as_mut_ptr(), fds.len() as libc::nfds_t, ms);
                        }
                    }
                }
                #[cfg(not(unix))]
                std::thread::sleep(Duration::from_millis(50));
            }
        }

        // Put handles back
        self.handle.stdout = stdout;
        self.handle.stderr = stderr;
        // Reached by the process EXITING. This was the second lenient return
        // (issue 0471 named only the timeout one): a process that died without
        // ever printing the marker also reported success.
        let matched = output.contains(pattern);
        (output, matched)
    }

    /// Collect output, stopping early once `pattern` appears — the LENIENT wait.
    ///
    /// Infallible by design: it returns whatever the process printed, whether or
    /// not the pattern showed up. That is the honest name for what
    /// [`Self::wait_for_output_pattern`] used to do on every path (issue 0471),
    /// and it is the right primitive for the two cases where leniency is
    /// correct:
    ///
    /// * a **readiness wait** — "give the server a moment to say `Spinning`",
    ///   where the real assertion comes later and a missing marker is not
    ///   itself the failure;
    /// * an **assert-on-content** test — `let out = p.collect_until(M, t);
    ///   assert!(out.contains(M), "…{out}")`, where the caller wants the output
    ///   in its own failure message.
    ///
    /// The second case is why this returns `String` rather than a `Result` the
    /// caller would `unwrap_or_default()`. Under the strict contract that
    /// idiom yields an EMPTY string on timeout, so the assertion that follows
    /// reports the failure with no output attached — the diagnostic is
    /// destroyed by the very call that was supposed to gather it.
    ///
    /// Use [`Self::wait_for_output_pattern`] when a missing pattern IS the
    /// failure; it says so with `Err`.
    pub fn collect_until(&mut self, pattern: &str, timeout: Duration) -> String {
        self.wait_until_pattern(pattern, timeout).0
    }

    /// [`Self::wait_for_output_count`] for an assert-on-content caller: the
    /// real output, and the diagnostic SEPARATELY.
    ///
    /// Issue 0670. The obvious way to keep a timed-out wait's evidence is
    /// `unwrap_or_else(|e| e.to_string())`, and it is a trap: that text NAMES
    /// the pattern it was waiting for (``did not print `max-age-runtime` ``),
    /// so an assertion of the common shape
    ///
    /// ```ignore
    /// assert!(seen.contains(RULE), "expected {RULE}, got:\n{seen}");
    /// ```
    ///
    /// then matches the COMPLAINT about the missing rule, and the test passes
    /// exactly when it should fail. That was tried on `contract_monitor_parity`
    /// and produced a green run against a pipeline emitting one DIAG line.
    ///
    /// The other obvious way, `unwrap_or_default()`, throws the evidence away
    /// instead — which is how the same test reported `got:` with nothing after
    /// it, and why the real cause (issue 0671: an unguarded `epoch_us_fn`
    /// clobber left the age monitor with no clock) took a separate
    /// investigation to find.
    ///
    /// So the two are returned on different channels and cannot be confused:
    /// put `.0` in what you assert on, `.1` in the panic MESSAGE.
    ///
    /// ```ignore
    /// let (seen, why) = p.collect_until_count(RULE, 1, Duration::from_secs(14));
    /// assert!(
    ///     seen.contains(RULE),
    ///     "expected {RULE} on /diagnostics, got:\n{seen}{}",
    ///     why.unwrap_or_default()
    /// );
    /// ```
    ///
    /// Sibling of [`Self::collect_until`], which serves the same purpose for
    /// the single-occurrence case and returns no diagnostic because its own
    /// `Err` carries none.
    pub fn collect_until_count(
        &mut self,
        pattern: &str,
        expected: usize,
        timeout: Duration,
    ) -> (String, Option<String>) {
        match self.wait_for_output_count(pattern, expected, timeout) {
            Ok(out) => (out, None),
            // The error carries the output too (see the timeout branch of
            // `wait_for_output_count`), but it is NOT returned as output: only
            // what the process really printed belongs in the asserted string.
            Err(e) => (String::new(), Some(format!("\n[wait {pattern}] {e}"))),
        }
    }

    /// Wait until a pattern appears at least `expected` times in stdout+stderr.
    pub fn wait_for_output_count(
        &mut self,
        pattern: &str,
        expected: usize,
        timeout: Duration,
    ) -> Result<String, TestError> {
        use std::io::Read;
        #[cfg(unix)]
        use std::os::unix::io::AsRawFd;

        let start = std::time::Instant::now();
        let mut output = String::new();
        let mut stdout = self.handle.stdout.take();
        let mut stderr = self.handle.stderr.take();
        let mut buf = [0u8; 4096];

        #[cfg(unix)]
        {
            if let Some(ref out) = stdout {
                let fd = out.as_raw_fd();
                unsafe {
                    let flags = libc::fcntl(fd, libc::F_GETFL);
                    libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
                }
            }
            if let Some(ref err) = stderr {
                let fd = err.as_raw_fd();
                unsafe {
                    let flags = libc::fcntl(fd, libc::F_GETFL);
                    libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
                }
            }
        }

        loop {
            if output.matches(pattern).count() >= expected {
                self.handle.stdout = stdout;
                self.handle.stderr = stderr;
                return Ok(output);
            }

            if start.elapsed() > timeout {
                self.handle.stdout = stdout;
                self.handle.stderr = stderr;
                // Carry the output. This is the ONE path where a caller most
                // needs to see what the process actually printed — it wanted
                // `pattern` and got something else — and it used to be the only
                // path that threw it away: `TestError::Timeout` is a unit
                // variant, so a `.unwrap_or_default()` caller ends up asserting
                // on `""` and reporting `got:` with nothing after it. The
                // process-exited branch just below already reports this way; so
                // do the siblings at `wait_for_output`/`wait_for_all_output`,
                // which return `Timeout` only when the output is genuinely
                // empty. `param_live_read_e2e` worked around this by waiting on
                // a broader pattern; the workaround is no longer needed.
                return Err(TestError::ProcessFailed(format!(
                    "{} did not print `{}` {} time(s) within {:?} (saw {}). Output:\n{}",
                    self.name,
                    pattern,
                    expected,
                    timeout,
                    output.matches(pattern).count(),
                    output
                )));
            }

            if let Ok(Some(_)) = self.handle.try_wait() {
                if let Some(ref mut out) = stdout {
                    let _ = out.read_to_string(&mut output);
                }
                if let Some(ref mut err) = stderr {
                    let _ = err.read_to_string(&mut output);
                }
                self.handle.stdout = stdout;
                self.handle.stderr = stderr;
                return if output.matches(pattern).count() >= expected {
                    Ok(output)
                } else {
                    Err(TestError::ProcessFailed(format!(
                        "{} exited before `{}` appeared {} times. Output:\n{}",
                        self.name, pattern, expected, output
                    )))
                };
            }

            let mut got_data = false;
            if let Some(ref mut out) = stdout
                && let Ok(n) = out.read(&mut buf)
                && n > 0
            {
                output.push_str(&String::from_utf8_lossy(&buf[..n]));
                got_data = true;
            }
            if let Some(ref mut err) = stderr
                && let Ok(n) = err.read(&mut buf)
                && n > 0
            {
                output.push_str(&String::from_utf8_lossy(&buf[..n]));
                got_data = true;
            }

            if !got_data {
                #[cfg(unix)]
                {
                    let remaining = timeout.saturating_sub(start.elapsed());
                    let ms = remaining.as_millis().min(500) as i32;
                    let mut fds = Vec::new();
                    if let Some(ref out) = stdout {
                        fds.push(libc::pollfd {
                            fd: out.as_raw_fd(),
                            events: libc::POLLIN,
                            revents: 0,
                        });
                    }
                    if let Some(ref err) = stderr {
                        fds.push(libc::pollfd {
                            fd: err.as_raw_fd(),
                            events: libc::POLLIN,
                            revents: 0,
                        });
                    }
                    if !fds.is_empty() {
                        unsafe {
                            libc::poll(fds.as_mut_ptr(), fds.len() as libc::nfds_t, ms);
                        }
                    }
                }
                #[cfg(not(unix))]
                std::thread::sleep(Duration::from_millis(50));
            }
        }
    }

    /// Kill the process group and wait for it to exit
    pub fn kill(&mut self) {
        kill_process_group(&mut self.handle);
    }

    /// Wait for output with timeout, capturing both stdout and stderr
    ///
    /// Similar to wait_for_output but also captures stderr (useful for env_logger output).
    /// The process is killed when the timeout is reached.
    pub fn wait_for_all_output(&mut self, timeout: Duration) -> Result<String, TestError> {
        use std::io::Read;
        #[cfg(unix)]
        use std::os::unix::io::AsRawFd;

        let start = std::time::Instant::now();
        let mut output = String::new();

        // Take both stdout and stderr
        let mut stdout = self.handle.stdout.take();
        let mut stderr = self.handle.stderr.take();

        // Set non-blocking mode on stdout and stderr
        #[cfg(unix)]
        {
            if let Some(ref out) = stdout {
                let fd = out.as_raw_fd();
                unsafe {
                    let flags = libc::fcntl(fd, libc::F_GETFL);
                    libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
                }
            }
            if let Some(ref err) = stderr {
                let fd = err.as_raw_fd();
                unsafe {
                    let flags = libc::fcntl(fd, libc::F_GETFL);
                    libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
                }
            }
        }

        let mut stdout_buf = [0u8; 4096];
        let mut stderr_buf = [0u8; 4096];

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
                    // Process exited, read remaining output
                    if let Some(ref mut out) = stdout {
                        let _ = out.read_to_string(&mut output);
                    }
                    if let Some(ref mut err) = stderr {
                        let _ = err.read_to_string(&mut output);
                    }
                    break;
                }
                Ok(None) => {
                    // Read from stdout
                    if let Some(ref mut out) = stdout {
                        match out.read(&mut stdout_buf) {
                            Ok(0) => {}
                            Ok(n) => {
                                output.push_str(&String::from_utf8_lossy(&stdout_buf[..n]));
                            }
                            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                            Err(_) => {}
                        }
                    }
                    // Read from stderr
                    if let Some(ref mut err) = stderr {
                        match err.read(&mut stderr_buf) {
                            Ok(0) => {}
                            Ok(n) => {
                                output.push_str(&String::from_utf8_lossy(&stderr_buf[..n]));
                            }
                            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                            Err(_) => {}
                        }
                    }
                    // Wait for data on either fd via poll(2)
                    #[cfg(unix)]
                    {
                        let remaining = timeout.saturating_sub(start.elapsed());
                        let ms = remaining.as_millis().min(500) as i32;
                        let mut fds = Vec::new();
                        if let Some(ref out) = stdout {
                            fds.push(libc::pollfd {
                                fd: out.as_raw_fd(),
                                events: libc::POLLIN,
                                revents: 0,
                            });
                        }
                        if let Some(ref err) = stderr {
                            fds.push(libc::pollfd {
                                fd: err.as_raw_fd(),
                                events: libc::POLLIN,
                                revents: 0,
                            });
                        }
                        if !fds.is_empty() {
                            unsafe {
                                libc::poll(fds.as_mut_ptr(), fds.len() as libc::nfds_t, ms);
                            }
                        }
                    }
                    #[cfg(not(unix))]
                    std::thread::sleep(Duration::from_millis(50));
                }
                Err(_) => break,
            }
        }

        Ok(output)
    }
}

impl Drop for ManagedProcess {
    fn drop(&mut self) {
        self.kill();
    }
}

// =============================================================================
// Zenoh Availability Check
// =============================================================================

/// phase-362 W1 — locate the ROS-shipped `rmw_zenohd`.
///
/// `rmw_zenohd` ships with `rmw_zenoh_cpp` and links the SAME `libzenohc.so`
/// the RMW does, so it cannot drift from it. Our vendored router could, and
/// did: issue 0609 measured `ros-humble-rmw-zenoh-cpp` 0.1.1 -> 0.1.9 moving
/// its vendored zenoh 1.2.0 -> 1.8.0 and interop going from zero samples to
/// 10/10, while our own pin took no part in either outcome (RFC-0075).
///
/// The argument that does not depend on a version number: in production a ROS 2
/// deployment runs `rmw_zenohd`. Testing against ours tested a configuration
/// nobody deploys.
///
/// Resolution order, most explicit first (issue 0653). Every step is something
/// the USER stated; nothing here searches for a router the user did not name:
///   1. `NROS_RMW_ZENOHD` — an explicit path, for a non-standard install.
///   2. `$AMENT_PREFIX_PATH` — every prefix the caller has SOURCED, in the
///      order ament lists them (most recently sourced first).
///   3. `$ROS_DISTRO` under `/opt/ros` — the distro the caller named.
///
/// Step 2 is what makes "source `setup.bash`, then run the tests" true, and
/// `/opt/ros` alone did not: a ROS install need not live there. This repo
/// documents building one on Arch/Fedora/NixOS
/// (`docs/development/ros2-on-non-ubuntu.md`), and a colcon overlay sits
/// wherever the user put it. `AMENT_PREFIX_PATH` is the sourced environment's
/// own answer to "which prefixes are active", so it covers both without this
/// code guessing a layout.
///
/// # `PATH` is deliberately NOT searched
///
/// It was, briefly, on the reasoning that a caller who put the router there
/// expects it found. That is the wrong trade for THIS binary, because the whole
/// point of RFC-0075 is that the router must be the one paired with the
/// `rmw_zenoh_cpp` a ROS node is using — and `PATH` is exactly where an
/// unrelated router accumulates. Measured on the machine this was written on:
///
/// ```text
/// /usr/bin/zenohd                            # a system install: zenohd v1.4.0
/// ~/.nros/sdk/zenohd/1.7.2-nros2/bin/zenohd  # nano-ros's own RETIRED entry: 1.7.2
/// #define ZENOH_C "1.8.0"                    # what ROS actually ships
/// ```
///
/// TWO unpaired routers, four and one minor versions behind, and one of them was
/// first on `PATH` because nano-ros itself put it there. Preferring `PATH` would
/// let either shadow the paired one and reintroduce precisely the drift issue
/// 0609 measured (`rmw-zenoh-cpp` 0.1.1 -> 0.1.9 moving vendored zenoh 1.2.0 ->
/// 1.8.0, interop going from zero samples to 10/10) with no version to point at.
/// `NROS_RMW_ZENOHD` remains for the deliberate case.
///
/// Note the search never looks for the name `zenohd` either — that is the
/// RETIRED vendored router, and both paths above carry one.
///
/// Returns `None` rather than a fallback path: a caller that cannot find the
/// ROS router must SKIP (issue 0599), and a `PathBuf` that does not exist would
/// turn that into a spawn failure several frames away from the cause.
///
/// Mirrored by `nros_zenohd_bin` in `scripts/dev/zenohd.sh`; the two are kept
/// in step by `check-zenohd-resolution-parity`, which parses both.
pub fn ros_zenohd_path() -> Option<std::path::PathBuf> {
    ros_zenohd_path_in(&ZenohdEnv::from_process())
}

/// The environment `ros_zenohd_path` reads, captured so the order can be tested.
///
/// Resolution is a function of four env vars and the filesystem. Taking them as
/// data is what lets the parity test drive both this and the shell over the same
/// synthetic prefixes — otherwise the only assertion available is "it found the
/// one router this host happens to have", which passes for every wrong order.
#[derive(Debug, Default, Clone)]
pub struct ZenohdEnv {
    pub explicit: Option<std::path::PathBuf>,
    pub ament_prefixes: Vec<std::path::PathBuf>,
    pub ros_distro: Option<String>,
    pub opt_ros: std::path::PathBuf,
}

impl ZenohdEnv {
    pub fn from_process() -> Self {
        let split = |k: &str| -> Vec<std::path::PathBuf> {
            std::env::var_os(k)
                .map(|v| std::env::split_paths(&v).collect())
                .unwrap_or_default()
        };
        Self {
            explicit: std::env::var_os("NROS_RMW_ZENOHD").map(Into::into),
            ament_prefixes: split("AMENT_PREFIX_PATH"),
            ros_distro: std::env::var("ROS_DISTRO").ok(),
            // Overridable ONLY for the parity gate's synthetic tree; see the
            // note on `opt_ros` in `scripts/dev/zenohd.sh`.
            opt_ros: std::env::var_os("NROS_ZENOHD_OPT_ROS")
                .map(std::path::PathBuf::from)
                .unwrap_or_else(|| std::path::PathBuf::from("/opt/ros")),
        }
    }
}

/// The router's path relative to an install prefix.
pub const ROS_ZENOHD_RELATIVE: &str = "lib/rmw_zenoh_cpp/rmw_zenohd";

/// [`ros_zenohd_path`] against an explicit environment. See [`ZenohdEnv`].
pub fn ros_zenohd_path_in(env: &ZenohdEnv) -> Option<std::path::PathBuf> {
    if let Some(explicit) = &env.explicit {
        return explicit.exists().then(|| explicit.clone());
    }
    // 2 — the sourced prefixes, in ament's own precedence order.
    for prefix in &env.ament_prefixes {
        let p = prefix.join(ROS_ZENOHD_RELATIVE);
        if p.exists() {
            return Some(p);
        }
    }
    // 3 — the named distro under the conventional prefix.
    if let Some(distro) = &env.ros_distro {
        let p = env.opt_ros.join(distro).join(ROS_ZENOHD_RELATIVE);
        if p.exists() {
            return Some(p);
        }
    }
    // There is deliberately no fourth step. Globbing `/opt/ros/*` and taking the
    // newest name resolves a router the user never asked for: on a host with
    // humble AND jazzy installed it picks jazzy by COLLATION, whatever the user
    // sourced or intended. That is the same defect as searching `PATH` — an
    // answer arrived at quickly and belonging to somebody else — and it is worse
    // here because the two are both plausible, so nothing about the result looks
    // wrong. If neither the environment nor an explicit path names a router,
    // saying so is the correct answer.
    None
}

/// Where a resolved router came from, and whether it is the ROS-paired one.
///
/// Issue 0653 — resolving *a* router is not the requirement; RFC-0075's whole
/// argument is that it must be the router paired with the `rmw_zenoh_cpp` in
/// use, because that pairing is what a version number could not express (issue
/// 0609: `rmw-zenoh-cpp` 0.1.1 -> 0.1.9 moved vendored zenoh 1.2.0 -> 1.8.0 with
/// no signal). Steps 2..4 of the search satisfy that by construction — they only
/// ever look inside an ament prefix — but `NROS_RMW_ZENOHD` is an escape hatch
/// and can point anywhere, including at a `zenohd` a user built from zenoh's own
/// instructions.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ZenohdProvenance {
    /// The install prefix, when the path has the ROS shape.
    pub prefix: Option<std::path::PathBuf>,
    /// `ZENOH_C` from that prefix's `zenoh_cpp_vendor` header, when present.
    pub zenoh_c: Option<String>,
}

impl ZenohdProvenance {
    /// True when the binary sits in an ament prefix that also ships the zenoh
    /// vendor header — i.e. it is `rmw_zenoh_cpp`'s own router, not a loose one.
    pub fn is_ros_shipped(&self) -> bool {
        self.prefix.is_some() && self.zenoh_c.is_some()
    }

    /// One line for a log or an error, naming what was actually resolved.
    pub fn describe(&self) -> String {
        match (&self.prefix, &self.zenoh_c) {
            (Some(prefix), Some(v)) => {
                format!("ROS-shipped, prefix {}, zenoh-c {v}", prefix.display())
            }
            (Some(prefix), None) => format!(
                "prefix {} has the ROS layout but no zenoh_cpp_vendor header — \
                 cannot confirm it is rmw_zenoh_cpp's own router",
                prefix.display()
            ),
            _ => "NOT a ROS-shipped router: the path is not <prefix>/lib/rmw_zenoh_cpp/\
                  rmw_zenohd, so it is not paired with any rmw_zenoh_cpp"
                .to_string(),
        }
    }
}

/// Describe where `path` came from. See [`ZenohdProvenance`].
pub fn ros_zenohd_provenance(path: &std::path::Path) -> ZenohdProvenance {
    // <prefix>/lib/rmw_zenoh_cpp/rmw_zenohd -> <prefix>
    let prefix = path
        .parent()
        .filter(|d| d.file_name().is_some_and(|n| n == "rmw_zenoh_cpp"))
        .and_then(|d| d.parent())
        .filter(|d| d.file_name().is_some_and(|n| n == "lib"))
        .and_then(|d| d.parent())
        .map(std::path::Path::to_path_buf);
    let zenoh_c = prefix.as_ref().and_then(|p| read_vendored_zenoh_c(p));
    ZenohdProvenance { prefix, zenoh_c }
}

/// `ZENOH_C` out of a prefix's `zenoh_cpp_vendor` header, if it has one.
fn read_vendored_zenoh_c(prefix: &std::path::Path) -> Option<String> {
    let header = prefix.join("opt/zenoh_cpp_vendor/include/zenoh_configure.h");
    let text = std::fs::read_to_string(header).ok()?;
    text.lines().find_map(|l| {
        // `#define ZENOH_C "1.6.2"` — the trailing space matters: `ZENOH_C_MAJOR`
        // sits right below it and a prefix match without it takes the wrong line.
        let rest = l.trim().strip_prefix("#define ZENOH_C ")?;
        Some(rest.trim().trim_matches('"').to_string())
    })
}

/// phase-362 W2 — the zenoh versions on BOTH sides of the seam.
///
/// A convention break currently surfaces as `delivered nothing: 0 data:
/// samples`, and both numbers that would explain it are one file read away.
///
/// Read the HEADER, never `dpkg -l`: the package version is the ROS wrapper's
/// (`0.1.9`) and says nothing about the zenoh inside it (`1.8.0`). Reading the
/// package version is exactly what produced a wrong version claim in issue
/// 0609's first filing, so this helper does not offer that route at all.
pub fn zenoh_pairing_versions() -> String {
    // One reader, shared with `ros_zenohd_provenance` — the header parse used to
    // be written out again here, and a second spelling of a parse is a second
    // thing to get wrong.
    let zenoh_c = ros_zenohd_path()
        .map(|p| ros_zenohd_provenance(&p))
        .and_then(|prov| prov.zenoh_c)
        .unwrap_or_else(|| "unknown".to_string());

    let pico = option_env!("CARGO_MANIFEST_DIR")
        .map(std::path::Path::new)
        .and_then(|d| d.ancestors().nth(3))
        .map(|root| root.join("packages/rmw/zenoh/zpico-sys/zenoh-pico/version.txt"))
        .and_then(|p| std::fs::read_to_string(p).ok())
        .map(|s| s.trim().to_string())
        .unwrap_or_else(|| "unknown".to_string());

    format!("zenoh-c (ROS) {zenoh_c}, zenoh-pico {pico}")
}

/// Is a zenoh router available on this host?
///
/// phase-362 — that means the ROS one. Existence is the whole check: the
/// vendored binary was probed with `--version`, which `rmw_zenohd` cannot
/// answer — it ignores argv and a probe would START A ROUTER and block.
pub fn is_zenohd_available() -> bool {
    ros_zenohd_path().is_some()
}

/// Check if the current environment allows local TCP listeners.
///
/// QEMU slirp itself is unprivileged, but test routers still need host TCP
/// sockets for zenohd. Some sandboxes deny `socket(AF_INET, ...)` outright.
pub fn is_local_tcp_listener_available() -> bool {
    TcpListener::bind("127.0.0.1:0").is_ok()
}

/// Skip test if zenohd is not available.
///
/// Returns `false` if zenohd is not available, printing a skip message.
/// Returns `true` if zenohd is available and the test should proceed.
///
/// # Example
///
/// ```ignore
/// #[test]
/// fn test_something() {
///     if !require_zenohd() {
///         return;
///     }
///     // ... test code
/// }
/// ```
pub fn require_zenohd() -> bool {
    match zenohd_unavailable_reason() {
        Some(why) => {
            eprintln!("Skipping test: {why}");
            false
        }
        None => true,
    }
}

/// Why the zenoh router is unusable here, or `None` if it is usable.
///
/// The reason-returning half of [`require_zenohd`], added for issue 0982: a
/// caller that turns "unavailable" into a SKIP must be able to put the cause in
/// the skip message. `require_zenohd`'s `eprintln!` goes to stderr, which
/// nextest suppresses under `--failure-output never` — so the CI log recorded
/// that a precondition was unmet and never which one.
///
/// `require_zenohd` keeps its `bool` signature: it has ~190 call sites, and the
/// ones that merely gate an early return do not need the text.
pub fn zenohd_unavailable_reason() -> Option<String> {
    if !is_local_tcp_listener_available() {
        return Some("local TCP listeners unavailable in this environment".to_string());
    }

    if !is_zenohd_available() {
        // Names SOURCING first (issue 0653): the common cause is not a missing
        // package but an unsourced one, and `rmw_zenohd` is invisible to
        // `command -v` even when present, so "is it installed?" is the question
        // a reader cannot answer from the shell.
        // Names SOURCING first (issue 0653) — see above.
        return Some(
            "no ROS zenoh router. Source your ROS setup \
             (`source /opt/ros/<distro>/setup.bash`) — that exports \
             AMENT_PREFIX_PATH, which is how the router is found; note \
             `rmw_zenohd` is NOT on PATH even when installed. Otherwise install \
             `ros-<distro>-rmw-zenoh-cpp`, or set NROS_RMW_ZENOHD to its path. \
             No ROS on this host? `--rmw cyclonedds` needs no router (phase-362)"
                .to_string(),
        );
    }
    None
}

/// Check if cmake is available in PATH
pub fn is_cmake_available() -> bool {
    Command::new("cmake")
        .arg("--version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Skip test if cmake is not available
///
/// Returns `false` if cmake is not found, printing a skip message.
/// Returns `true` if cmake is available and the test should proceed.
pub fn require_cmake() -> bool {
    if !is_cmake_available() {
        eprintln!("Skipping test: cmake not found");
        return false;
    }
    true
}

/// Resolve the `nros` CLI path: `$NROS_CLI` → `PATH` (incl in-tree
/// `packages/cli/target/release/` via `activate.sh`) → `~/.nros/bin/nros`
/// (transitional fallback).
///
/// Mirrors the shell `nros_cli_bin` helper in `scripts/build/cargo.sh`.
fn nros_cli_path() -> std::path::PathBuf {
    if let Ok(p) = std::env::var("NROS_CLI") {
        return std::path::PathBuf::from(p);
    }
    if let Ok(out) = Command::new("sh").args(["-c", "command -v nros"]).output()
        && out.status.success()
    {
        let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
        if !s.is_empty() {
            return std::path::PathBuf::from(s);
        }
    }
    let home = std::env::var("NROS_HOME")
        .ok()
        .or_else(|| std::env::var("HOME").ok().map(|h| format!("{h}/.nros")))
        .unwrap_or_else(|| "/root/.nros".to_string());
    std::path::PathBuf::from(format!("{home}/bin/nros"))
}

/// Check whether the installed `nros` CLI exposes the `nros sync` verb.
///
/// Added by Phase 210.D.1 / 210.E.3.d.native; the shipped 0.3.7 release
/// predates it. Tests that shell out to `nros sync …` (e.g. codegen-
/// preflight integration tests) should gate on this and skip cleanly
/// rather than burying the run in a clap "unrecognized subcommand 'ws'"
/// stack trace. Phase 214.I.2.
pub fn is_nros_ws_sync_available() -> bool {
    let bin = nros_cli_path();
    Command::new(bin)
        .args(["help", "ws"])
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .output()
        .map(|out| {
            // `nros help ws` exits non-zero on stock 0.3.7 (verb absent).
            // Check captured stdout for the `sync` subcommand row
            // independently of the exit status — newer releases that
            // succeed AND older that fail both flow through one match.
            String::from_utf8_lossy(&out.stdout).lines().any(|l| {
                let t = l.trim_start();
                t.starts_with("sync ") || t == "sync"
            })
        })
        .unwrap_or(false)
}

/// Skip the current test if the installed `nros` CLI lacks `nros sync`.
///
/// Returns `false` (test should bail / early-return) with a printed
/// skip line when unavailable; `true` when present. Phase 214.I.2.
///
/// # Example
///
/// ```ignore
/// #[test]
/// fn codegen_preflight() {
///     if !require_nros_ws_sync() {
///         return;
///     }
///     // ... shell out to `nros sync <dir>`
/// }
/// ```
pub fn require_nros_ws_sync() -> bool {
    if !is_nros_ws_sync_available() {
        eprintln!(
            "Skipping test: `nros sync` verb unavailable (installed nros lacks Phase 210.D.1; \
             rebuild via `just setup-cli` to pick up the in-tree CLI — Phase 218)"
        );
        return false;
    }
    true
}

/// Check if `docker compose` is available and the Docker daemon is running
pub fn is_docker_compose_available() -> bool {
    Command::new("docker")
        .args(["compose", "version"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
        && Command::new("docker")
            .args(["info"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .map(|s| s.success())
            .unwrap_or(false)
}

/// Skip test if Docker Compose is not available
///
/// Returns `false` if Docker Compose or the Docker daemon is unavailable.
/// Returns `true` if Docker is available and the test should proceed.
pub fn require_docker_compose() -> bool {
    if !is_docker_compose_available() {
        eprintln!("Skipping test: docker compose not available");
        return false;
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zenohd_detection() {
        let available = is_zenohd_available();
        eprintln!("zenohd available: {}", available);
    }

    #[test]
    fn test_cmake_detection() {
        let available = is_cmake_available();
        eprintln!("cmake available: {}", available);
    }

    /// Spawn a shell that prints `text` and then either exits or sleeps.
    fn echoing(text: &str, then_sleep: bool) -> ManagedProcess {
        let script = if then_sleep {
            format!("printf '%s\\n' '{text}'; sleep 30")
        } else {
            format!("printf '%s\\n' '{text}'", text = text)
        };
        let mut cmd = std::process::Command::new("sh");
        cmd.arg("-c").arg(script);
        ManagedProcess::spawn_command(cmd, "0471-contract").expect("spawn sh")
    }

    /// Issue 0471, the whole point: output that is not the pattern is NOT a
    /// match, on the TIMEOUT path.
    ///
    /// This is the case that made the defect invisible — a process printing the
    /// error explaining its own failure is non-empty output, so the old
    /// implementation returned `Ok`. A test written against a knowingly-broken
    /// fixture passed.
    #[test]
    fn wait_for_output_pattern_rejects_non_matching_output() {
        let mut p = echoing("Transport(InvalidConfig)", true);
        let r = p.wait_for_output_pattern("READY", Duration::from_secs(2));
        p.kill();
        let err = r.expect_err(
            "printing something OTHER than the pattern must not count as a match \
             — that is issue 0471 exactly",
        );
        let msg = format!("{err:?}");
        assert!(
            msg.contains("Transport(InvalidConfig)"),
            "the error must quote the output so a failure explains itself, got: {msg}"
        );
    }

    /// The second lenient path, which issue 0471 did not name: a process that
    /// EXITS without ever printing the pattern.
    #[test]
    fn wait_for_output_pattern_rejects_exit_without_pattern() {
        let mut p = echoing("goodbye", false);
        let r = p.wait_for_output_pattern("READY", Duration::from_secs(5));
        p.kill();
        assert!(
            r.is_err(),
            "a process that exited without printing the pattern did not match it"
        );
    }

    #[test]
    fn wait_for_output_pattern_accepts_a_real_match() {
        let mut p = echoing("READY", true);
        let out = p
            .wait_for_output_pattern("READY", Duration::from_secs(5))
            .expect("the pattern was printed");
        p.kill();
        assert!(out.contains("READY"), "Ok must carry the output: {out}");
    }

    /// The lenient counterpart keeps the old behavior under an honest name:
    /// output comes back whether or not the pattern appeared.
    #[test]
    fn collect_until_returns_output_even_without_a_match() {
        let mut p = echoing("Transport(InvalidConfig)", true);
        let out = p.collect_until("READY", Duration::from_secs(2));
        p.kill();
        assert!(
            out.contains("Transport(InvalidConfig)"),
            "collect_until must hand back what was printed so the caller's own \
             assertion can quote it, got: {out}"
        );
    }

    /// Issue 0653 — a router outside an ament prefix is not the paired one.
    ///
    /// The search cannot produce such a path (steps 2..4 look only inside ament
    /// prefixes), but `NROS_RMW_ZENOHD` can, and that is the door a user walks
    /// through with a `zenohd` built from zenoh's own instructions — whose
    /// version has no relation to the `rmw_zenoh_cpp` in use. The router fixture
    /// warns on exactly this predicate, so the predicate is worth pinning.
    #[test]
    fn provenance_distinguishes_the_paired_router_from_a_loose_one() {
        let root = std::env::temp_dir().join(format!("nros-zenohd-prov-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);

        let paired = root.join("ros").join(ROS_ZENOHD_RELATIVE);
        std::fs::create_dir_all(paired.parent().unwrap()).unwrap();
        std::fs::write(&paired, b"").unwrap();
        let header = root.join("ros/opt/zenoh_cpp_vendor/include/zenoh_configure.h");
        std::fs::create_dir_all(header.parent().unwrap()).unwrap();
        std::fs::write(
            &header,
            "#define ZENOH_C_MAJOR 1\n#define ZENOH_C \"1.8.0\"\n",
        )
        .unwrap();

        let prov = ros_zenohd_provenance(&paired);
        assert!(prov.is_ros_shipped(), "{}", prov.describe());
        assert_eq!(prov.zenoh_c.as_deref(), Some("1.8.0"));
        assert_eq!(prov.prefix.as_deref(), Some(root.join("ros").as_path()));

        // A loose router — the shape `command -v zenohd` finds. This host had a
        // retired `zenohd` 1.7.2 in `~/.nros/sdk` while ROS shipped 1.8.0, which
        // is why the search does not consult PATH at all.
        let loose = root.join("sdk/zenohd/1.7.2/bin/zenohd");
        std::fs::create_dir_all(loose.parent().unwrap()).unwrap();
        std::fs::write(&loose, b"").unwrap();
        let prov = ros_zenohd_provenance(&loose);
        assert!(!prov.is_ros_shipped());
        assert!(prov.prefix.is_none(), "a bin/ path is not an ament prefix");
        assert!(prov.describe().contains("NOT a ROS-shipped router"));

        // The ROS LAYOUT without the vendor header: right place, unconfirmable
        // pairing. Distinguished from the case above because the remedy differs
        // — this one is "your prefix is incomplete", not "wrong binary".
        let unmarked = root.join("nomarker").join(ROS_ZENOHD_RELATIVE);
        std::fs::create_dir_all(unmarked.parent().unwrap()).unwrap();
        std::fs::write(&unmarked, b"").unwrap();
        let prov = ros_zenohd_provenance(&unmarked);
        assert!(!prov.is_ros_shipped());
        assert!(prov.prefix.is_some());
        assert!(prov.describe().contains("no zenoh_cpp_vendor header"));

        let _ = std::fs::remove_dir_all(&root);
    }

    /// Issue 0653 — the RUST resolver answers the shared resolution table.
    ///
    /// The other consumer is `scripts/check-zenohd-resolution-parity.sh`, which
    /// runs `scripts/dev/zenohd.sh` over the same rows. Two resolvers exist
    /// because one is invoked from a justfile and one from the harness, and
    /// neither can call the other; what stops them drifting is that the
    /// EXPECTATIONS are written once. `zenohd.sh` carried "the two must agree"
    /// as a comment for a phase and they drifted regardless — both looking only
    /// under `/opt/ros`, so a sourced ROS built anywhere else resolved nothing.
    ///
    /// Reads the table rather than restating it: a copy here would be a second
    /// spelling of the thing the table exists to prevent.
    #[test]
    fn zenohd_resolution_matches_the_shared_table() {
        let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("repo root above packages/testing/nros-tests")
            .to_path_buf();
        let table_path = repo.join("scripts/dev/zenohd-resolution-cases.tsv");
        let table = std::fs::read_to_string(&table_path)
            .unwrap_or_else(|e| panic!("read {}: {e}", table_path.display()));

        let root = std::env::temp_dir().join(format!(
            "nros-zenohd-parity-{}-{}",
            std::process::id(),
            line!()
        ));
        let _ = std::fs::remove_dir_all(&root);
        let rel = std::path::Path::new(ROS_ZENOHD_RELATIVE);
        let touch_exec = |p: &std::path::Path| {
            std::fs::create_dir_all(p.parent().unwrap()).unwrap();
            std::fs::write(p, b"").unwrap();
            #[cfg(unix)]
            {
                use std::os::unix::fs::PermissionsExt;
                std::fs::set_permissions(p, std::fs::Permissions::from_mode(0o755)).unwrap();
            }
        };
        for prefix in ["overlay", "underlay", "optros/humble", "optros/jazzy"] {
            touch_exec(&root.join(prefix).join(rel));
        }
        touch_exec(&root.join("explicit/router"));
        std::fs::create_dir_all(root.join("nothing")).unwrap();

        let expand = |s: &str| -> String {
            s.replace('@', &root.to_string_lossy())
                .replace("$rel", ROS_ZENOHD_RELATIVE)
        };
        let split = |s: &str| -> Vec<std::path::PathBuf> {
            s.split(':')
                .filter(|c| !c.is_empty())
                .map(std::path::PathBuf::from)
                .collect()
        };

        let mut ran = 0usize;
        let mut failures: Vec<String> = Vec::new();
        for line in table.lines() {
            if line.trim_start().starts_with('#') || line.trim().is_empty() {
                continue;
            }
            let col: Vec<&str> = line.split('\t').collect();
            assert_eq!(
                col.len(),
                6,
                "row {:?} has {} columns, expected 6 — the table's shape is shared \
                 with the shell gate, so a ragged row breaks both",
                col[0],
                col.len()
            );
            ran += 1;
            let (name, explicit, ament, distro, optros, expected) = (
                col[0],
                expand(col[1]),
                expand(col[2]),
                col[3],
                expand(col[4]),
                expand(col[5]),
            );

            let env = ZenohdEnv {
                explicit: (!explicit.is_empty()).then(|| std::path::PathBuf::from(&explicit)),
                ament_prefixes: split(&ament),
                ros_distro: (!distro.is_empty()).then(|| distro.to_string()),
                opt_ros: std::path::PathBuf::from(&optros),
            };
            let got = ros_zenohd_path_in(&env)
                .map(|p| p.to_string_lossy().into_owned())
                .unwrap_or_default();
            if got != expected {
                failures.push(format!(
                    "  {name}: resolved {:?}, table says {:?}",
                    got, expected
                ));
            }
        }
        let _ = std::fs::remove_dir_all(&root);

        assert!(
            ran > 0,
            "the shared table has NO rows — this test watches nothing"
        );
        assert!(
            failures.is_empty(),
            "{} of {ran} shared-table case(s) disagree:\n{}",
            failures.len(),
            failures.join("\n")
        );
    }
}
