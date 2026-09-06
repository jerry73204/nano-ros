//! ROS 2 process fixtures for integration tests
//!
//! Provides helpers for running ROS 2 commands and processes.

use crate::{TestError, TestResult, process::kill_process_group};
use std::{
    path::PathBuf,
    process::{Child, Command, Stdio},
    time::Duration,
};

/// Default ROS 2 distro to use
pub const DEFAULT_ROS_DISTRO: &str = "humble";

/// How long a `ros2 topic echo` peer lives when the caller does not say
/// (issue 1026) — the historical baked `timeout --foreground 10`, now named so
/// a caller can compare its own wait against it instead of guessing.
pub const DEFAULT_ECHO_WINDOW: Duration = Duration::from_secs(10);

/// How long a `ros2 topic pub` peer lives when the caller does not say
/// (issue 1044) — the same horizon as [`DEFAULT_ECHO_WINDOW`] one role over.
///
/// A publisher's `timeout --foreground` is its LIFETIME too: it stops
/// publishing when this expires, so a subscriber-side wait longer than this
/// window is waiting on a peer that has already gone. Named for the same reason
/// the echo window is — so the two numbers can be compared instead of guessed.
///
/// [`Ros2Process::topic_pub`] deliberately uses a longer 45 s window and says so
/// at its own call site; this constant is the DDS/Cyclone family's default.
pub const DEFAULT_PUB_WINDOW: Duration = Duration::from_secs(10);

/// phase-304 W4 — is a specific ROS 2 distro installed under `/opt/ros/<distro>`?
/// Distro-parametric so an edition lane (RFC-0056) can require iron/jazzy/rolling
/// and `skip!` when absent, instead of everything assuming humble. Returns false
/// (never panics) when the setup script is missing or `ros2 --help` fails.
pub fn is_ros2_distro_available(distro: &str) -> bool {
    // Reject a distro name that isn't a bare identifier (defense against a
    // caller interpolating an untrusted string into the sourced path).
    if distro.is_empty()
        || !distro
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_')
    {
        return false;
    }
    Command::new("bash")
        .args([
            "-c",
            &format!("source /opt/ros/{distro}/setup.bash && ros2 --help"),
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Is `pkg` present in the ROS 2 install for `distro`?
///
/// Distro-parametric for the same reason [`is_ros2_distro_available`] is: a host
/// carries exactly ONE ROS 2 edition (RFC-0058 / phase-309 — humble on Ubuntu
/// 22.04, jazzy on 24.04, and the apt trees do not coexist). So a literal
/// `humble` in a probe does not mean "the usual one", it means "a prefix that
/// does not exist" on every other host — and the failure is silent: the source
/// fails, the probe returns false, and every test guarded by it reports SKIP
/// rather than red. A test that skips on exactly the host it was written to
/// exercise is worse than one that fails, because the sweep still goes green.
/// The three RMW probes below each carried that literal and took the whole
/// zenoh/fastrtps/cyclone interop surface out of a jazzy run with no signal.
///
/// One spelling, not one per RMW — the repeated `source … && ros2 pkg list |
/// grep -q …` shape is what let the distro drift in three places at once.
/// Returns false (never panics) when the setup script is missing or the command
/// fails.
pub fn is_ros2_package_available(distro: &str, pkg: &str) -> bool {
    // Both strings are interpolated into a shell command, so both must be bare
    // identifiers (same guard, same reason, as `is_ros2_distro_available`).
    let is_bare =
        |s: &str| !s.is_empty() && s.chars().all(|c| c.is_ascii_alphanumeric() || c == '_');
    if !is_bare(distro) || !is_bare(pkg) {
        return false;
    }
    Command::new("bash")
        .args([
            "-c",
            &format!("source /opt/ros/{distro}/setup.bash && ros2 pkg list | grep -q {pkg}"),
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Check if ROS 2 is available (the [`DEFAULT_ROS_DISTRO`] — humble today).
pub fn is_ros2_available() -> bool {
    is_ros2_distro_available(DEFAULT_ROS_DISTRO)
}

/// Require ROS 2 for a test (skips if not available)
///
/// Returns true if ROS 2 is available, false otherwise.
/// Prints a skip message when returning false.
pub fn require_ros2() -> bool {
    if !is_ros2_available() {
        eprintln!("Skipping test: ROS 2 not found");
        return false;
    }
    if !is_rmw_zenoh_available() {
        eprintln!("Skipping test: rmw_zenoh_cpp not found");
        return false;
    }
    true
}

/// Check if rmw_zenoh_cpp is available in the [`DEFAULT_ROS_DISTRO`] install.
///
/// The distro install is the only source. The opt-in `build/rmw_zenoh_ws`
/// source overlay it used to prefer is gone — nothing automated ever built it,
/// and its stated reason (wire-matching our zenoh-pico pin) was refuted by issue
/// 0291: zenoh's wire is proto-stable across 1.x.
pub fn is_rmw_zenoh_available() -> bool {
    is_ros2_package_available(DEFAULT_ROS_DISTRO, "rmw_zenoh_cpp")
}

/// Get ROS 2 environment setup command with default locator
pub fn ros2_env_setup(distro: &str) -> (String, tempfile::TempDir) {
    ros2_env_setup_with_locator(distro, "tcp/127.0.0.1:7447")
}

/// The `rmw_zenoh_cpp` session config we hand ROS 2, built by OVERLAYING our
/// settings onto the one it ships — not by replacing it.
///
/// `ZENOH_SESSION_CONFIG_URI` REPLACES `DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5`
/// wholesale; it does not merge. Issue 0609: this function used to write twenty
/// lines carrying `mode`, `connect` and `scouting`, so every other setting the
/// RMW relies on was silently dropped — `timestamping` among them. `/rosout` is
/// transient-local and routes through zenoh-ext's `PublicationCache`, which
/// REQUIRES timestamping, so under the old config **no ROS 2 node could start**:
///
/// ```text
/// zenohc::publication_cache: Failed requirement for PublicationCache on
///   0/rosout/…: the 'timestamping' setting must be enabled
/// [ERROR] [rmw_zenoh_cpp]: Unable to make PublisherData.
/// ```
///
/// Fifteen tests failed that way, and none of them was about `/rosout`.
///
/// Restating `timestamping` alone would fix today's symptom and leave the
/// mechanism that lost it: the next setting the RMW starts depending on would
/// vanish the same way, with a failure just as far from its cause. So we parse
/// what it ships and override only the three things we actually need — client
/// mode, our locator, and no multicast scouting.
///
/// Returns a [`tempfile::TempDir`] the caller MUST hold for the lifetime of the
/// process reading the config; dropping it deletes the file.
fn write_zenoh_session_config(locator: &str) -> tempfile::TempDir {
    let dir = tempfile::tempdir().expect("failed to create temp dir for zenoh session config");
    let config_path = dir.path().join("session_config.json5");

    let mut config = shipped_session_config().unwrap_or_else(|| {
        // No ROS 2 install to read from. The bare config still carries
        // `timestamping`, so this fallback is not a way to reintroduce 0609 —
        // and a caller with no rmw_zenoh_cpp has nothing to hand it to anyway.
        serde_json::json!({
            "timestamping": {
                "enabled": { "router": true, "peer": true, "client": true },
                "drop_future_timestamp": false,
            }
        })
    });

    let obj = config
        .as_object_mut()
        .expect("zenoh session config must be a JSON object");
    obj.insert("mode".into(), serde_json::json!("client"));
    obj.insert(
        "connect".into(),
        serde_json::json!({
            "endpoints": [locator],
            "exit_on_failure": { "client": true },
            "timeout_ms": { "client": 0 },
        }),
    );
    // Only the multicast switch — `scouting` carries gossip/autoconnect settings
    // the RMW sets deliberately, and replacing the whole table would be 0609
    // again at one level down.
    if let Some(scouting) = obj.get_mut("scouting").and_then(|s| s.as_object_mut()) {
        scouting.insert(
            "multicast".into(),
            match scouting.get("multicast").cloned() {
                Some(serde_json::Value::Object(mut m)) => {
                    m.insert("enabled".into(), serde_json::json!(false));
                    serde_json::Value::Object(m)
                }
                _ => serde_json::json!({ "enabled": false }),
            },
        );
    } else {
        obj.insert(
            "scouting".into(),
            serde_json::json!({ "multicast": { "enabled": false } }),
        );
    }

    // Serialized as JSON, which is a subset of JSON5 — no need to re-emit
    // comments or unquoted keys, and the RMW parses either.
    std::fs::write(
        &config_path,
        serde_json::to_string_pretty(&config).expect("serialize zenoh session config"),
    )
    .expect("failed to write zenoh session config");
    dir
}

/// `rmw_zenoh_cpp`'s own `DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5`, parsed.
///
/// Searched where the RMW itself would be found: the distro install. (A
/// source overlay used to be searched first; it is gone — RFC-0075, amended
/// 2026-08-19.) Returns `None` when it does not exist or the file will not
/// parse — the caller falls back rather than failing, because a host without
/// ROS 2 must still be able to call this.
///
/// The file is real JSON5 (comments, unquoted keys, trailing commas), so it
/// needs a JSON5 parser; `serde_json` cannot read it.
fn shipped_session_config() -> Option<serde_json::Value> {
    const REL: &str = "share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5";
    let mut candidates: Vec<PathBuf> = Vec::new();
    for distro in ["humble", "iron", "jazzy", "rolling"] {
        candidates.push(PathBuf::from(format!("/opt/ros/{distro}")).join(REL));
    }
    for path in candidates {
        let Ok(text) = std::fs::read_to_string(&path) else {
            continue;
        };
        match json5::from_str::<serde_json::Value>(&text) {
            Ok(v) if v.is_object() => return Some(v),
            _ => continue,
        }
    }
    None
}

/// Get ROS 2 environment setup command with custom locator.
///
/// Returns `(shell_snippet, _config_guard)`. The caller **must** hold the
/// returned [`tempfile::TempDir`] for the lifetime of the process that reads
/// the config — dropping it deletes the config file.
///
/// Stops the ROS 2 daemon first because it maintains its own zenoh session
/// connected to the default `tcp/localhost:7447`. If the daemon is running,
/// `ros2 topic list` queries the graph via XML-RPC through the daemon, which
/// ignores per-process zenoh config. Stopping the daemon forces the CLI to
/// create its own zenoh session using our custom locator.
///
/// Uses `ZENOH_SESSION_CONFIG_URI` to point rmw_zenoh_cpp at a JSON5 config
/// file with `mode: "client"` and the specified locator as the connect endpoint.
pub fn ros2_env_setup_with_locator(distro: &str, locator: &str) -> (String, tempfile::TempDir) {
    // Domain 0 is a CHOICE here, not an absence — see `ros2_env_setup_zenoh`.
    ros2_env_setup_zenoh(distro, locator, 0)
}

/// THE zenoh peer environment — issue 0763. Every other zenoh spelling routes
/// here, so there is one place that decides what a ROS 2 peer's environment is.
///
/// `ROS_DOMAIN_ID` is exported unconditionally, and that is the point. It was
/// absent from this family entirely, with two consequences that look unrelated
/// and are not:
///
/// * **Discovery.** The domain is the FIRST segment of an `rmw_zenoh` keyexpr,
///   so a peer and the nano-ros node must agree on it or they simply never see
///   each other. `Middleware::Zenoh` has carried a `domain_id` since phase-309
///   and says so in its own doc comment; the host backend destructured it away
///   with `{ locator, .. }` and every zenoh peer silently ran on domain 0. The
///   docker backend exported it. Same `Middleware` value, two environments.
/// * **The daemon.** ros2cli keys its daemon on `ROS_DOMAIN_ID` ALONE
///   (`get_ros_domain_id()` — no RMW in the key), so "no domain" means every
///   test in a parallel suite shares domain 0's singleton daemon. Distinct
///   domains give distinct daemons.
///
/// Note what is NOT here: `ros2 daemon stop`. It used to lead this command and
/// under a parallel suite it is a cross-test kill. The daemon caching a stale
/// `RMW_IMPLEMENTATION` is a real hazard, but the answer is not to consult a
/// shared daemon — every query helper in this module passes `--no-daemon`.
pub fn ros2_env_setup_zenoh(
    distro: &str,
    locator: &str,
    domain_id: u8,
) -> (String, tempfile::TempDir) {
    let config_dir = write_zenoh_session_config(locator);
    let config_path = config_dir.path().join("session_config.json5");
    // The distro's `rmw_zenoh_cpp`, and only that. A source overlay used to be
    // layered on top when present; it is gone (see `is_rmw_zenoh_available`).
    // No `ros2 daemon stop` here — issue 0763.
    //
    // It used to lead this command, and under a parallel suite that is a
    // CROSS-TEST KILL: the daemon is a singleton keyed on `ROS_DOMAIN_ID`
    // alone (ros2cli `get_ros_domain_id()`, no RMW in the key), nothing here
    // sets a domain, so every test shares domain 0's daemon and each one of
    // these setups stopped the daemon the others were mid-query against.
    //
    // What it was defending against is real: the daemon caches
    // `RMW_IMPLEMENTATION` and the rest of the environment from whoever
    // started it, and serves every later caller from that stale snapshot. But
    // the defence has to be "do not consult a shared daemon", not "restart the
    // shared daemon" — hence `--no-daemon` on every query helper below, which
    // makes a stale daemon unable to poison a result and a daemon stop
    // unnecessary.
    let cmd = format!(
        "source /opt/ros/{distro}/setup.bash && \
         export RMW_IMPLEMENTATION=rmw_zenoh_cpp && \
         export ROS_DOMAIN_ID={domain_id} && \
         export ZENOH_SESSION_CONFIG_URI={config_path}",
        config_path = config_path.display()
    );
    (cmd, config_dir)
}

/// Managed ROS 2 process
///
/// Wraps a ROS 2 command with proper environment setup.
/// Automatically kills the process on drop.
///
/// Holds a [`tempfile::TempDir`] to keep the zenoh session config file alive
/// for the lifetime of the process.
pub struct Ros2Process {
    handle: Child,
    name: String,
    _config_dir: Option<tempfile::TempDir>,
}

impl Ros2Process {
    /// Spawn a bash command in its own process group.
    fn spawn_bash(
        cmd: &str,
        name: impl Into<String>,
        config_dir: Option<tempfile::TempDir>,
    ) -> TestResult<Self> {
        let name = name.into();
        let mut command = Command::new("bash");
        command
            // issue 0923 — the peer takes its whole group down when this
            // process dies, rather than waiting for the next lane's sweep.
            .args(["-c", &crate::process::group_suicide_wrapper(cmd)[..]])
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        #[cfg(unix)]
        crate::process::set_orphan_group_suicide(&mut command);
        let handle = command
            .spawn()
            .map_err(|e| TestError::ProcessFailed(format!("Failed to start {name}: {e}")))?;
        // issue 0659 — record the group so a LATER run can reap it. The child is
        // its own group leader (`setpgid(0,0)`), so its pid IS the pgid. Nothing
        // in this process can clean up after its own SIGKILL, which is why the
        // record has to outlive it. Linux, not unix: the ledger is `/proc`-backed
        // (see `process::group_ledger`), so `cfg(unix)` promised a sweep on the
        // BSDs that the reader could never perform.
        #[cfg(target_os = "linux")]
        crate::process::group_ledger::record(handle.id() as i32, &name);
        Ok(Self {
            handle,
            name,
            _config_dir: config_dir,
        })
    }

    /// Start a ROS 2 topic echo subscriber that lives for
    /// [`DEFAULT_ECHO_WINDOW`].
    ///
    /// # Arguments
    /// * `topic` - Topic name (e.g., "/chatter")
    /// * `msg_type` - Message type (e.g., "std_msgs/msg/Int32")
    /// * `locator` - Zenoh locator (e.g., "tcp/127.0.0.1:7447")
    /// * `distro` - ROS distro (e.g., "humble")
    pub fn topic_echo(
        topic: &str,
        msg_type: &str,
        locator: &str,
        distro: &str,
    ) -> TestResult<Self> {
        Self::topic_echo_for(topic, msg_type, locator, distro, DEFAULT_ECHO_WINDOW)
    }

    /// [`Self::topic_echo`] with the subscriber's LIFETIME named by the caller
    /// (issue 1026).
    ///
    /// The `timeout --foreground` is a horizon, not an implementation detail:
    /// it is the hard cap on what the echo can ever observe, so a caller whose
    /// own wait is longer than it silently gets a truncated transcript and a
    /// failure that reads like "no delivery". Passing the window makes the two
    /// numbers one decision at one site.
    ///
    /// `PYTHONUNBUFFERED=1` is what makes a CONDITION wait possible here at
    /// all: `ros2 topic echo` is a Python entry point, so with stdout on a pipe
    /// its `print`s sit in a block buffer until the process exits — which is
    /// how the baked timeout came to double as the flush mechanism, and why
    /// every caller used to have to drain to completion.
    pub fn topic_echo_for(
        topic: &str,
        msg_type: &str,
        locator: &str,
        distro: &str,
        window: Duration,
    ) -> TestResult<Self> {
        let (env_setup, config_dir) = ros2_env_setup_with_locator(distro, locator);
        let secs = window.as_secs().max(1);
        let cmd = format!(
            "{env_setup} && PYTHONUNBUFFERED=1 timeout --foreground {secs} ros2 topic echo {topic} {msg_type} --qos-reliability best_effort"
        );

        Self::spawn_bash(&cmd, format!("ros2 topic echo {topic}"), Some(config_dir))
    }

    /// Start a ROS 2 action send_goal command
    ///
    /// # Arguments
    /// * `action_name` - Action name (e.g., "/demo/fibonacci")
    /// * `action_type` - Action type (e.g., "example_interfaces/action/Fibonacci")
    /// * `goal` - Goal data as YAML (e.g., "{order: 5}")
    /// * `locator` - Zenoh locator (e.g., "tcp/127.0.0.1:7447")
    /// * `distro` - ROS distro (e.g., "humble")
    pub fn action_send_goal(
        action_name: &str,
        action_type: &str,
        goal: &str,
        locator: &str,
        distro: &str,
    ) -> TestResult<Self> {
        let (env_setup, config_dir) = ros2_env_setup_with_locator(distro, locator);
        let cmd = format!(
            "{env_setup} && timeout --foreground 15 ros2 action send_goal --feedback {action_name} {action_type} \"{goal}\""
        );

        Self::spawn_bash(
            &cmd,
            format!("ros2 action send_goal {action_name}"),
            Some(config_dir),
        )
    }

    /// Start a ROS 2 Fibonacci action server
    ///
    /// Uses the example_interfaces Fibonacci action server.
    /// Requires ros-humble-example-interfaces package.
    ///
    /// # Arguments
    /// * `locator` - Zenoh locator (e.g., "tcp/127.0.0.1:7447")
    /// * `distro` - ROS distro (e.g., "humble")
    pub fn action_server_fibonacci(locator: &str, distro: &str) -> TestResult<Self> {
        let (env_setup, config_dir) = ros2_env_setup_with_locator(distro, locator);
        // Issue 0153 — inline rclpy server pinned to `example_interfaces`
        // (the type+name the nano-ros action client uses). The stock
        // `action_tutorials_py fibonacci_action_server` serves
        // `action_tutorials_interfaces/action/Fibonacci`, a DIFFERENT type,
        // so the client's send_goal queryable never matched and every goal
        // timed out. Same fix the DDS variant
        // (`action_server_fibonacci_with_domain`) already carries (233.6).
        let python_script = r#"
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class Server(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._srv = ActionServer(self, Fibonacci, '/fibonacci', self.execute)
        print('SERVER READY', flush=True)
    def execute(self, goal_handle):
        order = goal_handle.request.order
        print(f'SERVER GOAL order={order}', flush=True)
        seq = [0, 1]
        for i in range(1, order):
            seq.append(seq[i] + seq[i - 1])
            fb = Fibonacci.Feedback()
            fb.sequence = seq
            goal_handle.publish_feedback(fb)
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = seq
        print(f'SERVER DONE {seq}', flush=True)
        return result

rclpy.init()
node = Server()
rclpy.spin(node)
"#;
        let cmd = format!(
            "{env_setup} && timeout --foreground 60 python3 -u - 2>&1 <<'NROS_PYEOF'
{python_script}
NROS_PYEOF"
        );
        Self::spawn_bash(&cmd, "ros2 fibonacci_action_server", Some(config_dir))
    }

    /// Phase 211 acceptance — start the stock `demo_nodes_cpp talker` (an
    /// UNMODIFIED upstream rclcpp node publishing `std_msgs/String` "Hello
    /// World: N" on `/chatter` at 1 Hz), over `rmw_zenoh_cpp` pointed at
    /// `locator`. Pairs with a nano-ros raw-String subscriber to prove
    /// cross-vendor interop on the ROS graph.
    pub fn demo_nodes_cpp_talker(locator: &str, distro: &str) -> TestResult<Self> {
        let (env_setup, config_dir) = ros2_env_setup_with_locator(distro, locator);
        let cmd = format!("{env_setup} && timeout --foreground 30 ros2 run demo_nodes_cpp talker");
        Self::spawn_bash(&cmd, "ros2 demo_nodes_cpp talker", Some(config_dir))
    }

    /// Start a ROS 2 topic pub publisher
    ///
    /// # Arguments
    /// * `topic` - Topic name (e.g., "/chatter")
    /// * `msg_type` - Message type (e.g., "std_msgs/msg/Int32")
    /// * `data` - Message data as YAML (e.g., "{data: 42}")
    /// * `rate` - Publishing rate in Hz
    /// * `locator` - Zenoh locator (e.g., "tcp/127.0.0.1:7447")
    /// * `distro` - ROS distro (e.g., "humble")
    pub fn topic_pub(
        topic: &str,
        msg_type: &str,
        data: &str,
        rate: u32,
        locator: &str,
        distro: &str,
    ) -> TestResult<Self> {
        let (env_setup, config_dir) = ros2_env_setup_with_locator(distro, locator);
        // #146 — publish with the DEFAULT (reliable) profile: the nano
        // subscribers under test declare a default (reliable) subscription, and
        // a best_effort publisher is INCOMPATIBLE with a reliable subscriber by
        // ROS 2 QoS rules — rmw_zenoh delivers nothing, which read as a
        // "ros2→nano broken" false alarm. The nano→ros2 direction already relies
        // on this compatibility (reliable nano pub → `ros2 topic echo`'s
        // best_effort sensor_data sub). `timeout 45` (was 10): rmw_zenoh's
        // publisher-side discovery of a zenoh-pico subscriber takes ~10 s, so a
        // 10 s publisher would die right as the first sample would land.
        let cmd = format!(
            "{env_setup} && timeout --foreground 45 ros2 topic pub -r {rate} {topic} {msg_type} \"{data}\""
        );

        Self::spawn_bash(&cmd, format!("ros2 topic pub {topic}"), Some(config_dir))
    }

    /// Wait for output and return it
    /// Collect output until `stop` is satisfied, the process exits, or `timeout`
    /// elapses — the shared engine behind [`Self::wait_for_output`] (drain to the
    /// deadline) and [`Self::wait_for_output_count`] (stop early on a pattern).
    ///
    /// Returns the output AND whether `stop` was satisfied. Both facts in one
    /// return value on purpose: `ManagedProcess` had to learn this the hard way
    /// (issue 0471) when a single `Result` conflated them and the only path
    /// carrying the output was also the path claiming success.
    ///
    /// phase-342 W9 — this exists so a ROS 2 wait can be a CONDITION rather than
    /// a duration. Before it, the only way to "give ROS 2 time to receive" was
    /// `sleep`, because `wait_for_output` drains to its deadline and cannot be
    /// asked about a pattern.
    fn collect_until(
        &mut self,
        stop: Option<(&str, usize)>,
        timeout: Duration,
    ) -> TestResult<(String, bool)> {
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

        // Set non-blocking mode on stdout so read() doesn't block forever
        #[cfg(unix)]
        let fd = {
            let fd = stdout.as_raw_fd();
            unsafe {
                let flags = libc::fcntl(fd, libc::F_GETFL);
                libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
            }
            fd
        };

        let satisfied = |out: &str| match stop {
            Some((pat, n)) => out.matches(pat).count() >= n,
            None => false,
        };

        let mut buffer = [0u8; 4096];
        loop {
            if satisfied(&output) {
                // Put stdout back: a caller that stopped early may still want to
                // collect the rest, which the old drain-only shape made impossible.
                self.handle.stdout = Some(stdout);
                return Ok((output, true));
            }
            if start.elapsed() > timeout {
                kill_process_group(&mut self.handle);
                if output.is_empty() && stop.is_none() {
                    return Err(TestError::Timeout);
                }
                break;
            }

            match self.handle.try_wait() {
                Ok(Some(_)) => {
                    let _ = stdout.read_to_string(&mut output);
                    break;
                }
                Ok(None) => match stdout.read(&mut buffer) {
                    Ok(0) => {
                        Self::wait_for_data(
                            #[cfg(unix)]
                            fd,
                            timeout.saturating_sub(start.elapsed()),
                        );
                    }
                    Ok(n) => {
                        output.push_str(&String::from_utf8_lossy(&buffer[..n]));
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        Self::wait_for_data(
                            #[cfg(unix)]
                            fd,
                            timeout.saturating_sub(start.elapsed()),
                        );
                    }
                    Err(_) => break,
                },
                Err(_) => break,
            }
        }

        let matched = satisfied(&output);
        Ok((output, matched))
    }

    /// Drain STDOUT until the process exits or `timeout` elapses, then KILL it.
    ///
    /// **A terminal drain, not a wait-for-readiness.** `collect_until(None, …)`
    /// has no stop condition, so a process that keeps running always reaches
    /// the timeout — and reaching it calls `kill_process_group`. To wait for
    /// readiness and KEEP the process, use [`Self::wait_for_output_count`].
    ///
    /// Issue 0672 called the misuse "a 5 s sleep that never observed anything",
    /// which understates it in the direction that matters: a sleep leaves the
    /// server running, and this one killed it. The client was then started
    /// against a server that was reliably gone, not one that might not have
    /// been listening.
    pub fn wait_for_output(&mut self, timeout: Duration) -> TestResult<String> {
        self.collect_until(None, timeout).map(|(out, _)| out)
    }

    /// Wait until `pattern` has appeared `expected` times, returning as soon as
    /// it has (phase-342 W9).
    ///
    /// This is what lets a ROS 2 test wait for the delivery it asserts instead
    /// of sleeping a fixed duration and hoping — the W8 rule, previously
    /// unavailable here for want of this method.
    ///
    /// On timeout the error carries the collected output, so a failure reads
    /// like the assertion that would have followed.
    pub fn wait_for_output_count(
        &mut self,
        pattern: &str,
        expected: usize,
        timeout: Duration,
    ) -> TestResult<String> {
        match self.collect_until(Some((pattern, expected)), timeout)? {
            (out, true) => Ok(out),
            (out, false) => Err(TestError::ProcessFailed(format!(
                "{} printed `{}` fewer than {} time(s) within {:?}. Output:\n{}",
                self.name, pattern, expected, timeout, out
            ))),
        }
    }

    /// Wait for data on a file descriptor (or sleep on non-Unix).
    #[cfg(unix)]
    fn wait_for_data(fd: std::os::unix::io::RawFd, remaining: Duration) {
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
    fn wait_for_data(remaining: Duration) {
        std::thread::sleep(remaining.min(Duration::from_millis(50)));
    }

    /// Kill the process
    pub fn kill(&mut self) {
        kill_process_group(&mut self.handle);
    }

    /// Check if process is still running
    pub fn is_running(&mut self) -> bool {
        matches!(self.handle.try_wait(), Ok(None))
    }
}

impl Drop for Ros2Process {
    fn drop(&mut self) {
        self.kill();
    }
}

/// Read everything a spawned child prints on stdout within `timeout`, without
/// blocking forever. Shared by [`Ros2Process`] and [`crate::ros_env::RosPeer`]
/// (phase-309) so both peer wrappers use identical non-blocking drain logic.
///
/// **A terminal drain, not a wait-for-readiness (issue 1026).** There is no
/// stop condition, so a process that keeps running ALWAYS reaches the deadline
/// — and reaching it kills the process group. Pointed at a `spin = "forever"`
/// node the timeout is therefore the node's LIFETIME, and whatever the node
/// would have done afterwards is unobservable by construction. To wait on a
/// CONDITION and keep the peer alive, use [`collect_child_until`].
pub fn wait_child_output(handle: &mut Child, name: &str, timeout: Duration) -> TestResult<String> {
    collect_child_inner(handle, name, None, timeout)
}

/// Collect a child's stdout, returning as soon as `pattern` has appeared
/// `expected` times — the CONDITION sibling of [`wait_child_output`], and the
/// wait to use for a free-running node (issue 1026).
///
/// Never kills: the caller (or [`crate::ros_env::RosPeer`]'s `Drop`) decides
/// when the peer dies, so a test that wants to keep observing after the marker
/// still can. Returns whatever was printed whether or not the condition was
/// met — the caller asserts on the content and wants the transcript in its own
/// failure message, exactly as with
/// [`crate::process::ManagedProcess::collect_until`].
pub fn collect_child_until(
    handle: &mut Child,
    name: &str,
    pattern: &str,
    expected: usize,
    timeout: Duration,
) -> TestResult<String> {
    collect_child_inner(handle, name, Some((pattern, expected)), timeout)
}

/// The shared drain loop. `stop` is `None` for the terminal drain (run to the
/// deadline, then kill) and `Some((pattern, n))` for the condition wait (return
/// at the n-th occurrence, kill nothing).
fn collect_child_inner(
    handle: &mut Child,
    name: &str,
    stop: Option<(&str, usize)>,
    timeout: Duration,
) -> TestResult<String> {
    use std::io::Read;
    #[cfg(unix)]
    use std::os::unix::io::AsRawFd;

    let satisfied = |out: &str| match stop {
        Some((pattern, expected)) => out.matches(pattern).count() >= expected,
        None => false,
    };

    let start = std::time::Instant::now();
    let mut output = String::new();

    let mut stdout = handle
        .stdout
        .take()
        .ok_or_else(|| TestError::ProcessFailed(format!("No stdout for {name}")))?;

    #[cfg(unix)]
    let fd = {
        let fd = stdout.as_raw_fd();
        unsafe {
            let flags = libc::fcntl(fd, libc::F_GETFL);
            libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
        }
        fd
    };

    let mut buffer = [0u8; 4096];
    loop {
        if satisfied(&output) {
            // Condition met — hand the stream back so a later call keeps
            // reading where this one stopped, and leave the peer RUNNING.
            handle.stdout = Some(stdout);
            return Ok(output);
        }
        if start.elapsed() > timeout {
            // Only the terminal drain kills. A condition wait that misses its
            // deadline leaves the peer alive: the caller asserts on the output
            // and may still want to look at (or keep) the process.
            if stop.is_none() {
                kill_process_group(handle);
            } else {
                handle.stdout = Some(stdout);
                return Ok(output);
            }
            if output.is_empty() {
                return Err(TestError::Timeout);
            }
            break;
        }
        match handle.try_wait() {
            Ok(Some(_)) => {
                let _ = stdout.read_to_string(&mut output);
                break;
            }
            Ok(None) => match stdout.read(&mut buffer) {
                Ok(0) => wait_child_data(
                    #[cfg(unix)]
                    fd,
                    timeout.saturating_sub(start.elapsed()),
                ),
                Ok(n) => output.push_str(&String::from_utf8_lossy(&buffer[..n])),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => wait_child_data(
                    #[cfg(unix)]
                    fd,
                    timeout.saturating_sub(start.elapsed()),
                ),
                Err(_) => break,
            },
            Err(_) => break,
        }
    }
    if stop.is_some() {
        // Process exited (or the stream broke) before the condition was met.
        // Give the stream back regardless; the caller decides what the missing
        // marker means.
        handle.stdout = Some(stdout);
    }
    Ok(output)
}

/// Block up to `remaining` (capped) for data on `fd`, or sleep on non-Unix.
#[cfg(unix)]
fn wait_child_data(fd: std::os::unix::io::RawFd, remaining: Duration) {
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
fn wait_child_data(remaining: Duration) {
    std::thread::sleep(remaining.min(Duration::from_millis(50)));
}

// =============================================================================
// Discovery Helpers
// =============================================================================

/// Run `ros2 node list` and return the output
pub fn ros2_node_list(locator: &str, distro: &str) -> TestResult<String> {
    let (env_setup, _config_dir) = ros2_env_setup_with_locator(distro, locator);
    let cmd = format!("{env_setup} && timeout --foreground 10 ros2 node list --no-daemon 2>&1");

    let output = Command::new("bash")
        .args(["-c", &cmd])
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("Failed to run ros2 node list: {e}")))?;

    Ok(String::from_utf8_lossy(&output.stdout).to_string())
}

/// Run `ros2 topic list` and return the output
pub fn ros2_topic_list(locator: &str, distro: &str) -> TestResult<String> {
    let (env_setup, _config_dir) = ros2_env_setup_with_locator(distro, locator);
    let cmd = format!("{env_setup} && timeout --foreground 10 ros2 topic list --no-daemon 2>&1");

    let output = Command::new("bash")
        .args(["-c", &cmd])
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("Failed to run ros2 topic list: {e}")))?;

    Ok(String::from_utf8_lossy(&output.stdout).to_string())
}

/// Issue 0309 — run `ros2 topic info --verbose <topic>` and return the report.
///
/// The ADVERTISED QoS profile is the only QoS fact observable from outside a
/// nano-ros process: the zenoh backend encodes the profile into the liveliness
/// token (`nros-rmw-zenoh`'s `to_qos_string`) and implements no QoS SEMANTICS —
/// no history cache, no depth-driven drops — so there is no delivery difference
/// to measure. A stock `rmw_zenoh_cpp` peer reading the token is how a test
/// distinguishes "the profile I declared" from "whatever default happened to
/// connect".
pub fn ros2_topic_info_verbose(locator: &str, distro: &str, topic: &str) -> TestResult<String> {
    let (env_setup, _config_dir) = ros2_env_setup_with_locator(distro, locator);
    let cmd = format!(
        "{env_setup} && timeout --foreground 15 ros2 topic info --verbose --no-daemon {topic} 2>&1"
    );

    let output = Command::new("bash")
        .args(["-c", &cmd])
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("Failed to run ros2 topic info: {e}")))?;

    Ok(format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    ))
}

/// Issue 0309 — slice ONE `Endpoint type: <kind>` section out of a
/// [`ros2_topic_info_verbose`] report (`"PUBLISHER"` / `"SUBSCRIPTION"`).
///
/// Assert against this, never against the whole report: the report carries both
/// endpoints, so a whole-report `contains("TRANSIENT_LOCAL")` passes on the
/// WRONG endpoint's profile. That is not hypothetical — the first draft of
/// `qos_override_e2e` did exactly that and passed with the bug it was written
/// to catch (issue 0306) reverted.
pub fn topic_endpoint_block(report: &str, kind: &str) -> Option<String> {
    let marker = format!("Endpoint type: {kind}");
    let start = report.find(&marker)?;
    let rest = &report[start..];
    // An endpoint's QoS block ends where the next endpoint's "Node name:"
    // begins, or at the end of the report.
    let end = rest[marker.len()..]
        .find("Node name:")
        .map(|i| i + marker.len())
        .unwrap_or(rest.len());
    Some(rest[..end].to_string())
}

/// One endpoint record from a [`ros2_topic_info_verbose`] report.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TopicEndpoint {
    /// The `Node name:` value.
    pub node: String,
    /// The `Endpoint type:` value — `PUBLISHER` or `SUBSCRIPTION`.
    pub kind: String,
    /// The record's text, from its `Node name:` line to the next one.
    pub block: String,
}

/// Every endpoint in a `ros2 topic info -v` report, in the order listed.
///
/// Issue 0690 — [`topic_endpoint_block`] returns the FIRST block of a kind, and
/// a topic can carry more than one. `case_08_c_qos` failed in-sweep and passed
/// solo with the publisher advertising exactly `nros_c_qos_default()`, which is
/// what reading a FOREIGN endpoint's profile looks like; the same binaries
/// passed and failed, so it was never a product regression.
///
/// The report is a flat list of records delimited by `Node name:` — the field
/// order within one is `Node name:`, `Endpoint type:`, then the QoS block, and
/// nothing else is guaranteed (a `GID:`/`Node namespace:` line appears on some
/// distros and not others), so the parse keys only on those two.
pub fn topic_endpoints(report: &str) -> Vec<TopicEndpoint> {
    const NODE: &str = "Node name:";
    let mut out = Vec::new();
    let mut starts: Vec<usize> = report.match_indices(NODE).map(|(i, _)| i).collect();
    starts.push(report.len());
    for pair in starts.windows(2) {
        let block = &report[pair[0]..pair[1]];
        let field = |name: &str| -> Option<String> {
            let at = block.find(name)? + name.len();
            Some(block[at..].lines().next()?.trim().to_string())
        };
        let (Some(node), Some(kind)) = (field(NODE), field("Endpoint type:")) else {
            continue;
        };
        out.push(TopicEndpoint {
            node,
            kind,
            block: block.to_string(),
        });
    }
    out
}

/// The endpoints of `kind` belonging to `node`.
///
/// Issue 0690's fix direction: select the block by the node UNDER TEST rather
/// than by position, so a foreign publisher on the same topic cannot be
/// asserted against.
///
/// Returns a Vec deliberately. Node name alone does not always identify one
/// endpoint — the three `*_qos` workspace cells all name their node
/// `qos_talker` — so a caller that gets more than one is looking at a sibling
/// cell, not a foreign process, and those need different remedies. Collapsing
/// that to "the first match" here would rebuild the bug one level up.
pub fn topic_endpoints_for_node(report: &str, kind: &str, node: &str) -> Vec<TopicEndpoint> {
    topic_endpoints(report)
        .into_iter()
        .filter(|e| e.kind == kind && e.node == node)
        .collect()
}

/// Poll `ros2 topic info --verbose` until every wanted `(kind, node)` endpoint
/// has appeared, or the deadline passes — issues 0705, 0761.
///
/// **A single-shot query is a race by construction.** The graph is discovered
/// asynchronously by the ros2 daemon, so "not there yet" and "not there" print
/// the same thing, and under sweep load the first is far more likely: issue
/// 0761's `qos_override_e2e` slept a fixed 3 s, asked once, and failed with
/// `Unknown topic '/qos_chatter'` in a 1658-test sweep while passing solo in
/// 5.08 s on the same checkout and fixtures. Issue 0705 is the same failure one
/// file over, where it was worse than a flake: the single shot returned a
/// report naming some OTHER cell's `talker`, so it read as "the wrong graph"
/// rather than "not yet propagated".
///
/// This does not weaken any assertion. It returns the LAST report and the
/// matches found in it; the caller still asserts the full profile and still
/// fails on timeout, carrying that report.
///
/// Two things it deliberately does not wait out:
///
/// * **More than one match for a wanted endpoint.** That is the sibling-cell
///   case (issue 0690), and waiting only makes the report bigger. It returns so
///   the caller's assertion can name the ambiguity.
/// * **A `ros2` invocation that ERRORS.** Propagated, not retried — a broken
///   environment is not a slow one, and retrying it for 20 s only delays the
///   message that says so.
pub fn await_topic_endpoints(
    locator: &str,
    distro: &str,
    topic: &str,
    want: &[(&str, &str)],
    timeout: Duration,
) -> TestResult<(String, Vec<Vec<TopicEndpoint>>)> {
    let deadline = std::time::Instant::now() + timeout;
    loop {
        let report = ros2_topic_info_verbose(locator, distro, topic)?;
        let found: Vec<Vec<TopicEndpoint>> = want
            .iter()
            .map(|(kind, node)| topic_endpoints_for_node(&report, kind, node))
            .collect();
        let all_present = found.iter().all(|m| !m.is_empty());
        let ambiguous = found.iter().any(|m| m.len() > 1);
        if all_present || ambiguous || std::time::Instant::now() >= deadline {
            return Ok((report, found));
        }
        std::thread::sleep(Duration::from_millis(500));
    }
}

/// phase-433 W6 — `ros2 topic info --verbose` on a CYCLONE domain.
///
/// The sibling of [`ros2_topic_info_verbose`], and it needs its own body
/// rather than a parameter because the two bus vocabularies do not overlap:
/// the zenoh form is addressed by a router LOCATOR and the DDS form by a
/// `ROS_DOMAIN_ID`, and `ros2_env_setup_cyclonedds_with_domain` additionally
/// folds in [`crate::dds_isolation`]'s loopback profile, which the zenoh path
/// deliberately does not get (issue 1009 — a `CYCLONEDDS_URI` whitelist is how
/// a DDS test stops reading a foreign participant on the LAN, and half a pin
/// is no discovery at all).
pub fn ros2_topic_info_verbose_cyclonedds(
    distro: &str,
    domain_id: u8,
    topic: &str,
) -> TestResult<String> {
    let env_setup = ros2_env_setup_cyclonedds_with_domain(distro, domain_id);
    let cmd = format!(
        "{env_setup} && timeout --foreground 15 ros2 topic info --verbose --no-daemon {topic} 2>&1"
    );

    let output = Command::new("bash")
        .args(["-c", &cmd])
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("Failed to run ros2 topic info: {e}")))?;

    Ok(format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    ))
}

/// The Cyclone-domain twin of [`await_topic_endpoints`], with the same
/// contract — poll to a deadline, return the LAST report plus the per-want
/// matches, propagate an errored `ros2` rather than retrying it, and stop
/// early on an ambiguous match so the caller can name it.
///
/// The reason for polling is the reason there: a single-shot query is a race
/// by construction (issues 0705, 0761), and "not there yet" prints exactly
/// like "not there".
pub fn await_topic_endpoints_cyclonedds(
    distro: &str,
    domain_id: u8,
    topic: &str,
    want: &[(&str, &str)],
    timeout: Duration,
) -> TestResult<(String, Vec<Vec<TopicEndpoint>>)> {
    let deadline = std::time::Instant::now() + timeout;
    loop {
        let report = ros2_topic_info_verbose_cyclonedds(distro, domain_id, topic)?;
        let found: Vec<Vec<TopicEndpoint>> = want
            .iter()
            .map(|(kind, node)| topic_endpoints_for_node(&report, kind, node))
            .collect();
        let all_present = found.iter().all(|m| !m.is_empty());
        let ambiguous = found.iter().any(|m| m.len() > 1);
        if all_present || ambiguous || std::time::Instant::now() >= deadline {
            return Ok((report, found));
        }
        std::thread::sleep(Duration::from_millis(500));
    }
}

/// The `GID:` line of one `ros2 topic info --verbose` endpoint block, as bytes.
///
/// `None` when the block carries no `GID:` line — which is a real possibility,
/// not defensive coding: [`topic_endpoints`] keys only on `Node name:` and
/// `Endpoint type:` precisely because a `GID:`/`Node namespace:` line appears
/// on some distros and not others. A caller that wants to compare GIDs must
/// treat the absence as "this tooling cannot answer", never as a mismatch.
///
/// rclpy renders the gid as dot-separated lowercase hex bytes
/// (`01.0f.28.6b.…`); its WIDTH is distro-dependent (24 bytes through Humble,
/// 16 in later editions), so the bytes are returned as a `Vec` and the caller
/// decides how many of them are load-bearing.
pub fn endpoint_gid_bytes(block: &str) -> Option<Vec<u8>> {
    let line = block.lines().find_map(|l| l.trim().strip_prefix("GID:"))?;
    let bytes: Option<Vec<u8>> = line
        .trim()
        .split('.')
        .map(|b| u8::from_str_radix(b.trim(), 16).ok())
        .collect();
    bytes.filter(|b| !b.is_empty())
}

/// Run `ros2 service list` and return the output
pub fn ros2_service_list(locator: &str, distro: &str) -> TestResult<String> {
    let (env_setup, _config_dir) = ros2_env_setup_with_locator(distro, locator);
    let cmd = format!("{env_setup} && timeout --foreground 10 ros2 service list --no-daemon 2>&1");

    let output = Command::new("bash")
        .args(["-c", &cmd])
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("Failed to run ros2 service list: {e}")))?;

    Ok(String::from_utf8_lossy(&output.stdout).to_string())
}

/// Run `ros2 node info` for a specific node
pub fn ros2_node_info(node_name: &str, locator: &str, distro: &str) -> TestResult<String> {
    let (env_setup, _config_dir) = ros2_env_setup_with_locator(distro, locator);
    let cmd = format!(
        "{env_setup} && timeout --foreground 10 ros2 node info --no-daemon {node_name} 2>&1"
    );

    let output = Command::new("bash")
        .args(["-c", &cmd])
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("Failed to run ros2 node info: {e}")))?;

    Ok(String::from_utf8_lossy(&output.stdout).to_string())
}

/// Run `ros2 param list` for a specific node
pub fn ros2_param_list(node_name: &str, locator: &str, distro: &str) -> TestResult<String> {
    let (env_setup, _config_dir) = ros2_env_setup_with_locator(distro, locator);
    let cmd = format!(
        "{env_setup} && timeout --foreground 15 ros2 param list --no-daemon {node_name} 2>&1"
    );

    let output = Command::new("bash")
        .args(["-c", &cmd])
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("Failed to run ros2 param list: {e}")))?;

    Ok(String::from_utf8_lossy(&output.stdout).to_string())
}

/// Run `ros2 param get` for a specific parameter on a node
pub fn ros2_param_get(
    node_name: &str,
    param_name: &str,
    locator: &str,
    distro: &str,
) -> TestResult<String> {
    let (env_setup, _config_dir) = ros2_env_setup_with_locator(distro, locator);
    let cmd = format!(
        "{env_setup} && timeout --foreground 15 ros2 param get --no-daemon {node_name} {param_name} 2>&1"
    );

    let output = Command::new("bash")
        .args(["-c", &cmd])
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("Failed to run ros2 param get: {e}")))?;

    Ok(String::from_utf8_lossy(&output.stdout).to_string())
}

/// Run `ros2 param set` to set a parameter on a node
pub fn ros2_param_set(
    node_name: &str,
    param_name: &str,
    value: &str,
    locator: &str,
    distro: &str,
) -> TestResult<String> {
    let (env_setup, _config_dir) = ros2_env_setup_with_locator(distro, locator);
    let cmd = format!(
        "{env_setup} && timeout --foreground 15 ros2 param set --no-daemon {node_name} {param_name} {value} 2>&1"
    );

    let output = Command::new("bash")
        .args(["-c", &cmd])
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("Failed to run ros2 param set: {e}")))?;

    Ok(String::from_utf8_lossy(&output.stdout).to_string())
}

/// Run `ros2 param describe` for a specific parameter on a node
pub fn ros2_param_describe(
    node_name: &str,
    param_name: &str,
    locator: &str,
    distro: &str,
) -> TestResult<String> {
    let (env_setup, _config_dir) = ros2_env_setup_with_locator(distro, locator);
    let cmd = format!(
        "{env_setup} && timeout --foreground 15 ros2 param describe --no-daemon {node_name} {param_name} 2>&1"
    );

    let output = Command::new("bash")
        .args(["-c", &cmd])
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("Failed to run ros2 param describe: {e}")))?;

    Ok(String::from_utf8_lossy(&output.stdout).to_string())
}

/// Run `ros2 topic info` for a specific topic
pub fn ros2_topic_info(topic: &str, locator: &str, distro: &str) -> TestResult<String> {
    let (env_setup, _config_dir) = ros2_env_setup_with_locator(distro, locator);
    let cmd =
        format!("{env_setup} && timeout --foreground 10 ros2 topic info --no-daemon {topic} 2>&1");

    let output = Command::new("bash")
        .args(["-c", &cmd])
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("Failed to run ros2 topic info: {e}")))?;

    Ok(String::from_utf8_lossy(&output.stdout).to_string())
}

/// Run `ros2 topic hz <topic>` for a measurement window and return the captured
/// output. `ros2 topic hz` streams "average rate: X.YYY" lines roughly every
/// second; the helper times-out after `secs` seconds (caller picks 5–10 s for a
/// stable reading) and returns whatever the command printed so far.
///
/// Used by Phase 211.C to close the `<topic_list, topic_echo, topic_hz>` host
/// CLI interop trio — the first two were already covered, only `topic hz` was
/// missing.
pub fn ros2_topic_hz(topic: &str, secs: u64, locator: &str, distro: &str) -> TestResult<String> {
    let (env_setup, _config_dir) = ros2_env_setup_with_locator(distro, locator);
    // `ros2 topic hz` (Humble) only takes `--window / --filter / --wall-time /
    // --spin-time / -s`. `--no-daemon` + `--qos-reliability` are NOT valid here
    // (they belong on `lifecycle` and `topic echo` respectively); passing them
    // makes argparse hard-fail. `--spin-time` extends the discovery window so
    // the subscriber matches the rmw_zenoh talker before the timeout fires.
    let spin = (secs / 3).max(2);
    // Python's stdout is block-buffered when piped; `ros2 topic hz` prints
    // "average rate: …" lines that never reach our capture buffer before
    // `timeout` SIGTERMs the process. `stdbuf -oL` line-buffers stdout so each
    // averaged line is emitted as it's produced. `--wall-time` measures
    // against wall-clock (no /clock subscription needed for rmw_zenoh).
    let cmd = format!(
        "{env_setup} && timeout {secs} stdbuf -oL \
             ros2 topic hz --spin-time {spin} --wall-time {topic} 2>&1"
    );

    let output = Command::new("bash")
        .args(["-c", &cmd])
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("Failed to run ros2 topic hz: {e}")))?;

    Ok(String::from_utf8_lossy(&output.stdout).to_string())
}

// =============================================================================
// Service Helpers
// =============================================================================

impl Ros2Process {
    /// Start a ROS 2 service call
    ///
    /// # Arguments
    /// * `service_name` - Service name (e.g., "/add_two_ints")
    /// * `service_type` - Service type (e.g., "example_interfaces/srv/AddTwoInts")
    /// * `request` - Request data as YAML (e.g., "{a: 5, b: 3}")
    /// * `locator` - Zenoh locator (e.g., "tcp/127.0.0.1:7447")
    /// * `distro` - ROS distro (e.g., "humble")
    pub fn service_call(
        service_name: &str,
        service_type: &str,
        request: &str,
        locator: &str,
        distro: &str,
    ) -> TestResult<Self> {
        let (env_setup, config_dir) = ros2_env_setup_with_locator(distro, locator);
        let cmd = format!(
            "{env_setup} && timeout --foreground 10 ros2 service call {service_name} {service_type} \"{request}\""
        );

        Self::spawn_bash(
            &cmd,
            format!("ros2 service call {service_name}"),
            Some(config_dir),
        )
    }

    /// Start a ROS 2 service server (example_interfaces AddTwoInts)
    ///
    /// Uses a Python script to create a simple service server.
    /// The server responds with a + b for the AddTwoInts service.
    ///
    /// # Arguments
    /// * `locator` - Zenoh locator (e.g., "tcp/127.0.0.1:7447")
    /// * `distro` - ROS distro (e.g., "humble")
    pub fn add_two_ints_server(locator: &str, distro: &str) -> TestResult<Self> {
        let (env_setup, config_dir) = ros2_env_setup_with_locator(distro, locator);
        // Use a Python one-liner to create a simple service server
        let python_script = r#"
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Server(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, '/add_two_ints', self.callback)
        self.get_logger().info('Service server ready')
    def callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response

rclpy.init()
node = Server()
rclpy.spin(node)
"#;

        // Feed the script via a quoted heredoc so its real newlines reach
        // python. `python3 -c '<one line with \n literals>'` is a SyntaxError —
        // the `\n` is not a newline outside a string — so the server never
        // started and the reverse-direction interop tests timed out.
        let cmd = format!(
            "{env_setup} && timeout --foreground 60 python3 -u - 2>&1 <<'NROS_PYEOF'\n{python_script}\nNROS_PYEOF"
        );

        Self::spawn_bash(&cmd, "ros2 add_two_ints_server", Some(config_dir))
    }

    /// Start a ROS 2 topic echo subscriber with custom QoS
    ///
    /// # Arguments
    /// * `topic` - Topic name (e.g., "/chatter")
    /// * `msg_type` - Message type (e.g., "std_msgs/msg/Int32")
    /// * `reliability` - QoS reliability ("reliable" or "best_effort")
    /// * `locator` - Zenoh locator (e.g., "tcp/127.0.0.1:7447")
    /// * `distro` - ROS distro (e.g., "humble")
    ///
    /// Lives for [`DEFAULT_ECHO_WINDOW`] — the subscriber's whole lifetime, not
    /// a safety net (issue 1044), and buffered until exit like its
    /// `Ros2DdsProcess` siblings.
    pub fn topic_echo_with_qos(
        topic: &str,
        msg_type: &str,
        reliability: &str,
        locator: &str,
        distro: &str,
    ) -> TestResult<Self> {
        let (env_setup, config_dir) = ros2_env_setup_with_locator(distro, locator);
        let secs = DEFAULT_ECHO_WINDOW.as_secs().max(1);
        let cmd = format!(
            "{env_setup} && timeout --foreground {secs} ros2 topic echo {topic} {msg_type} --qos-reliability {reliability}"
        );

        Self::spawn_bash(
            &cmd,
            format!("ros2 topic echo {topic} ({reliability})"),
            Some(config_dir),
        )
    }

    /// Start a ROS 2 topic pub publisher with custom QoS
    ///
    /// # Arguments
    /// * `topic` - Topic name (e.g., "/chatter")
    /// * `msg_type` - Message type (e.g., "std_msgs/msg/Int32")
    /// * `data` - Message data as YAML (e.g., "{data: 42}")
    /// * `rate` - Publishing rate in Hz
    /// * `reliability` - QoS reliability ("reliable" or "best_effort")
    /// * `locator` - Zenoh locator (e.g., "tcp/127.0.0.1:7447")
    /// * `distro` - ROS distro (e.g., "humble")
    pub fn topic_pub_with_qos(
        topic: &str,
        msg_type: &str,
        data: &str,
        rate: u32,
        reliability: &str,
        locator: &str,
        distro: &str,
    ) -> TestResult<Self> {
        let (env_setup, config_dir) = ros2_env_setup_with_locator(distro, locator);
        let secs = DEFAULT_PUB_WINDOW.as_secs().max(1);
        let cmd = format!(
            "{env_setup} && timeout --foreground {secs} ros2 topic pub -r {rate} {topic} {msg_type} \"{data}\" --qos-reliability {reliability}"
        );

        Self::spawn_bash(
            &cmd,
            format!("ros2 topic pub {topic} ({reliability})"),
            Some(config_dir),
        )
    }
}

// =============================================================================
// DDS (rmw_fastrtps_cpp) Helpers — for XRCE-DDS ↔ ROS 2 interop tests
// =============================================================================

/// Check if rmw_fastrtps_cpp is available (the default RMW in humble and
/// jazzy alike) in the [`DEFAULT_ROS_DISTRO`] install.
pub fn is_rmw_fastrtps_available() -> bool {
    is_ros2_package_available(DEFAULT_ROS_DISTRO, "rmw_fastrtps_cpp")
}

/// Require ROS 2 with DDS (rmw_fastrtps_cpp) for a test.
///
/// Returns true if both ROS 2 and rmw_fastrtps_cpp are available.
/// Prints a skip message and returns false otherwise.
pub fn require_ros2_dds() -> bool {
    if !is_ros2_available() {
        eprintln!("Skipping test: ROS 2 not found");
        return false;
    }
    if !is_rmw_fastrtps_available() {
        eprintln!("Skipping test: rmw_fastrtps_cpp not found");
        return false;
    }
    true
}

/// Get ROS 2 environment setup command for DDS (rmw_fastrtps_cpp).
///
/// Unlike the zenoh variant, no locator or zenoh config is needed —
/// DDS uses multicast discovery on the local network.
pub fn ros2_env_setup_dds(distro: &str) -> String {
    ros2_env_setup_dds_with_domain(distro, 0)
}

/// Get ROS 2 environment setup command for DDS with an explicit domain
/// (defaults the middleware to `rmw_fastrtps_cpp`).
pub fn ros2_env_setup_dds_with_domain(distro: &str, domain_id: u8) -> String {
    ros2_env_setup_rmw_with_domain(distro, "rmw_fastrtps_cpp", domain_id)
}

/// Get ROS 2 environment setup for an explicit RMW + domain. No zenoh locator —
/// DDS RMWs use multicast discovery on the local network. The `rmw` string
/// selects the ROS 2 middleware (`rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp`, …).
pub fn ros2_env_setup_rmw_with_domain(distro: &str, rmw: &str, domain_id: u8) -> String {
    // Issue 1009 — pin the bus to loopback. Every DDS lane's env string funnels
    // through here, and every one of them pairs two processes on THIS host, so
    // there is nothing for a LAN peer to legitimately be. Opt out with
    // `NROS_DDS_ALLOW_LAN=1`.
    //
    // Deliberately NOT `ROS_LOCALHOST_ONLY=1`, which was measured at 0 of 15:
    // it isolates the ROS side and the XRCE Agent ignores it, so the pair stops
    // discovering each other. See `crate::dds_isolation`.
    let isolation = crate::dds_isolation::env_exports_for_rmw(rmw);
    format!(
        "source /opt/ros/{distro}/setup.bash && \
         export RMW_IMPLEMENTATION={rmw} && \
         export ROS_DOMAIN_ID={domain_id}{isolation}"
    )
}

/// Get ROS 2 environment setup for CycloneDDS (`rmw_cyclonedds_cpp`) + domain.
/// Used by the CycloneDDS ↔ ROS 2 interop suite (Phase 183.5) — nano-ros's
/// Cyclone backend and a stock `rmw_cyclonedds_cpp` ROS 2 node share a
/// `ROS_DOMAIN_ID` and discover over RTPS/SPDP.
pub fn ros2_env_setup_cyclonedds_with_domain(distro: &str, domain_id: u8) -> String {
    ros2_env_setup_rmw_with_domain(distro, "rmw_cyclonedds_cpp", domain_id)
}

/// Check if `rmw_cyclonedds_cpp` is available in the [`DEFAULT_ROS_DISTRO`]
/// ROS 2 install.
pub fn is_rmw_cyclonedds_available() -> bool {
    is_ros2_package_available(DEFAULT_ROS_DISTRO, "rmw_cyclonedds_cpp")
}

/// Require ROS 2 with CycloneDDS (`rmw_cyclonedds_cpp`) for a test.
pub fn require_ros2_cyclonedds() -> bool {
    if !is_ros2_available() {
        eprintln!("Skipping test: ROS 2 not found");
        return false;
    }
    if !is_rmw_cyclonedds_available() {
        eprintln!("Skipping test: rmw_cyclonedds_cpp not found");
        return false;
    }
    true
}

/// Managed ROS 2 process using DDS (rmw_fastrtps_cpp).
///
/// Same pattern as `Ros2Process` but uses DDS multicast discovery
/// instead of zenoh. No locator parameter needed.
pub struct Ros2DdsProcess {
    handle: Child,
    name: String,
}

impl Ros2DdsProcess {
    /// Spawn a bash command in its own process group.
    fn spawn_bash(cmd: &str, name: impl Into<String>) -> TestResult<Self> {
        let name = name.into();
        let mut command = Command::new("bash");
        command
            // issue 0923 — the peer takes its whole group down when this
            // process dies, rather than waiting for the next lane's sweep.
            .args(["-c", &crate::process::group_suicide_wrapper(cmd)[..]])
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        #[cfg(unix)]
        crate::process::set_orphan_group_suicide(&mut command);
        let handle = command
            .spawn()
            .map_err(|e| TestError::ProcessFailed(format!("Failed to start {name}: {e}")))?;
        Ok(Self { handle, name })
    }

    /// Start a ROS 2 DDS topic echo subscriber
    ///
    /// # Arguments
    /// * `topic` - Topic name (e.g., "/chatter")
    /// * `msg_type` - Message type (e.g., "std_msgs/msg/Int32")
    /// * `distro` - ROS distro (e.g., "humble")
    pub fn topic_echo(topic: &str, msg_type: &str, distro: &str) -> TestResult<Self> {
        Self::topic_echo_with_domain(topic, msg_type, distro, 0)
    }

    /// Start a ROS 2 DDS topic echo subscriber on a specific ROS domain.
    ///
    /// Lives for [`DEFAULT_ECHO_WINDOW`], and that window is this subscriber's
    /// whole LIFETIME rather than a safety net — issue 1044. A caller whose own
    /// wait is longer than it gets a truncated transcript and a failure that
    /// reads like "no delivery"; a caller whose wait is shorter is fine.
    ///
    /// Unlike [`Ros2Process::topic_echo_for`] this one does NOT set
    /// `PYTHONUNBUFFERED=1`, so the timeout is also the FLUSH: `ros2 topic echo`
    /// is a Python entry point and its `print`s sit in a block buffer until the
    /// process exits. Every caller therefore drains to completion, which is why
    /// the window is not a parameter here — making it one without unbuffering
    /// would let a caller ask for a wait that can never produce output.
    pub fn topic_echo_with_domain(
        topic: &str,
        msg_type: &str,
        distro: &str,
        domain_id: u8,
    ) -> TestResult<Self> {
        let env_setup = ros2_env_setup_dds_with_domain(distro, domain_id);
        let secs = DEFAULT_ECHO_WINDOW.as_secs().max(1);
        let cmd = format!(
            "{env_setup} && timeout --foreground {secs} ros2 topic echo {topic} {msg_type} --qos-reliability reliable"
        );
        Self::spawn_bash(&cmd, format!("ros2-dds topic echo {topic}"))
    }

    /// Start a ROS 2 DDS topic pub publisher
    ///
    /// # Arguments
    /// * `topic` - Topic name (e.g., "/chatter")
    /// * `msg_type` - Message type (e.g., "std_msgs/msg/Int32")
    /// * `data` - Message data as YAML (e.g., "{data: 42}")
    /// * `rate` - Publishing rate in Hz
    /// * `distro` - ROS distro (e.g., "humble")
    pub fn topic_pub(
        topic: &str,
        msg_type: &str,
        data: &str,
        rate: u32,
        distro: &str,
    ) -> TestResult<Self> {
        Self::topic_pub_with_domain(topic, msg_type, data, rate, distro, 0)
    }

    /// Start a ROS 2 DDS topic publisher on a specific ROS domain.
    pub fn topic_pub_with_domain(
        topic: &str,
        msg_type: &str,
        data: &str,
        rate: u32,
        distro: &str,
        domain_id: u8,
    ) -> TestResult<Self> {
        let env_setup = ros2_env_setup_dds_with_domain(distro, domain_id);
        let secs = DEFAULT_PUB_WINDOW.as_secs().max(1);
        let cmd = format!(
            "{env_setup} && timeout --foreground {secs} ros2 topic pub -r {rate} {topic} {msg_type} \"{data}\" --qos-reliability reliable"
        );
        Self::spawn_bash(&cmd, format!("ros2-dds topic pub {topic}"))
    }

    /// Start a ROS 2 DDS service call
    ///
    /// # Arguments
    /// * `service_name` - Service name (e.g., "/add_two_ints")
    /// * `service_type` - Service type (e.g., "example_interfaces/srv/AddTwoInts")
    /// * `request` - Request data as YAML (e.g., "{a: 5, b: 3}")
    /// * `distro` - ROS distro (e.g., "humble")
    pub fn service_call(
        service_name: &str,
        service_type: &str,
        request: &str,
        distro: &str,
    ) -> TestResult<Self> {
        Self::service_call_with_domain(service_name, service_type, request, distro, 0)
    }

    /// Start a ROS 2 DDS service call on a specific ROS domain.
    pub fn service_call_with_domain(
        service_name: &str,
        service_type: &str,
        request: &str,
        distro: &str,
        domain_id: u8,
    ) -> TestResult<Self> {
        let env_setup = ros2_env_setup_dds_with_domain(distro, domain_id);
        let cmd = format!(
            "{env_setup} && timeout --foreground 10 ros2 service call {service_name} {service_type} \"{request}\""
        );
        Self::spawn_bash(&cmd, format!("ros2-dds service call {service_name}"))
    }

    // --- CycloneDDS variants (Phase 183.5) — same `ros2` CLI, but with
    // RMW_IMPLEMENTATION=rmw_cyclonedds_cpp so the ROS 2 node speaks Cyclone
    // RTPS to nano-ros's CycloneDDS backend on a shared ROS_DOMAIN_ID. ---

    /// CycloneDDS topic echo subscriber on a specific ROS domain.
    ///
    /// Same [`DEFAULT_ECHO_WINDOW`] lifetime and the same buffered-until-exit
    /// behaviour as [`Self::topic_echo_with_domain`] — see its note (issue 1044).
    pub fn topic_echo_cyclonedds_with_domain(
        topic: &str,
        msg_type: &str,
        distro: &str,
        domain_id: u8,
    ) -> TestResult<Self> {
        let env_setup = ros2_env_setup_cyclonedds_with_domain(distro, domain_id);
        let secs = DEFAULT_ECHO_WINDOW.as_secs().max(1);
        let cmd = format!(
            "{env_setup} && timeout --foreground {secs} ros2 topic echo {topic} {msg_type} --qos-reliability reliable"
        );
        Self::spawn_bash(&cmd, format!("ros2-cyclone topic echo {topic}"))
    }

    /// CycloneDDS topic publisher on a specific ROS domain.
    pub fn topic_pub_cyclonedds_with_domain(
        topic: &str,
        msg_type: &str,
        data: &str,
        rate: u32,
        distro: &str,
        domain_id: u8,
    ) -> TestResult<Self> {
        let env_setup = ros2_env_setup_cyclonedds_with_domain(distro, domain_id);
        let secs = DEFAULT_PUB_WINDOW.as_secs().max(1);
        let cmd = format!(
            "{env_setup} && timeout --foreground {secs} ros2 topic pub -r {rate} {topic} {msg_type} \"{data}\" --qos-reliability reliable"
        );
        Self::spawn_bash(&cmd, format!("ros2-cyclone topic pub {topic}"))
    }

    /// A stock `demo_nodes_cpp talker` on CycloneDDS, on a specific domain.
    ///
    /// phase-381 step 2. The zenoh half of the graph acceptance runs the same
    /// node; Cyclone needed its own because the two backends discover through
    /// entirely different mechanisms — zenoh's `@ros2_lv` liveliness tokens
    /// versus Cyclone's `ros_discovery_info` topic — so proving one says
    /// nothing about the other. Issue 0903 is the evidence: zenoh's path passed
    /// every unit test and did not work at all.
    ///
    /// Longer than the 10 s the `topic pub` helpers use: the graph probe polls
    /// to convergence with a budget of its own, and a talker that exits first
    /// turns a real answer into an empty one.
    pub fn demo_nodes_cpp_talker_cyclonedds_with_domain(
        distro: &str,
        domain_id: u8,
    ) -> TestResult<Self> {
        let env_setup = ros2_env_setup_cyclonedds_with_domain(distro, domain_id);
        let cmd = format!("{env_setup} && timeout --foreground 40 ros2 run demo_nodes_cpp talker");
        Self::spawn_bash(&cmd, "ros2-cyclone demo_nodes_cpp talker")
    }

    /// CycloneDDS service call on a specific ROS domain.
    pub fn service_call_cyclonedds_with_domain(
        service_name: &str,
        service_type: &str,
        request: &str,
        distro: &str,
        domain_id: u8,
    ) -> TestResult<Self> {
        let env_setup = ros2_env_setup_cyclonedds_with_domain(distro, domain_id);
        let cmd = format!(
            "{env_setup} && timeout --foreground 10 ros2 service call {service_name} {service_type} \"{request}\""
        );
        Self::spawn_bash(&cmd, format!("ros2-cyclone service call {service_name}"))
    }

    /// CycloneDDS action `send_goal --feedback` on a specific ROS domain.
    pub fn action_send_goal_cyclonedds_with_domain(
        action_name: &str,
        action_type: &str,
        goal: &str,
        distro: &str,
        domain_id: u8,
    ) -> TestResult<Self> {
        let env_setup = ros2_env_setup_cyclonedds_with_domain(distro, domain_id);
        let cmd = format!(
            "{env_setup} && timeout --foreground 20 ros2 action send_goal --feedback {action_name} {action_type} \"{goal}\""
        );
        Self::spawn_bash(&cmd, format!("ros2-cyclone action send_goal {action_name}"))
    }

    // --- DDS server / action side (Phase 183.6) — the reverse interop
    // directions: a ROS 2 (rmw_fastrtps_cpp) service/action SERVER + an action
    // goal CLIENT, on an explicit ROS_DOMAIN_ID, for nano-XRCE ↔ ROS 2. ---

    /// ROS 2 DDS `add_two_ints` service server (rclpy one-liner) on a domain.
    pub fn add_two_ints_server_with_domain(distro: &str, domain_id: u8) -> TestResult<Self> {
        let env_setup = ros2_env_setup_dds_with_domain(distro, domain_id);
        let python_script = r#"
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Server(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, '/add_two_ints', self.callback)
        self.get_logger().info('Service server ready')
    def callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response

rclpy.init()
node = Server()
rclpy.spin(node)
"#;
        // Feed the script via a quoted heredoc so its real newlines reach
        // python. `python3 -c '<one line with \n literals>'` is a SyntaxError —
        // the `\n` is not a newline outside a string — so the server never
        // started and the reverse-direction interop tests timed out.
        let cmd = format!(
            "{env_setup} && timeout --foreground 60 python3 -u - 2>&1 <<'NROS_PYEOF'\n{python_script}\nNROS_PYEOF"
        );
        Self::spawn_bash(&cmd, "ros2-dds add_two_ints_server")
    }

    /// ROS 2 DDS Fibonacci action server on a domain.
    ///
    /// Serves `example_interfaces/action/Fibonacci` on `/fibonacci` — the SAME
    /// type+name the nano-ros action client/server examples use. The stock
    /// `action_tutorials_py fibonacci_action_server` serves
    /// `action_tutorials_interfaces/action/Fibonacci`, a DIFFERENT type, so DDS
    /// type matching never succeeds against our client → goal-acceptance timeout
    /// (233.6). A small rclpy server pinned to `example_interfaces` fixes the
    /// type alignment without depending on `action_tutorials_py` being present.
    pub fn action_server_fibonacci_with_domain(distro: &str, domain_id: u8) -> TestResult<Self> {
        let env_setup = ros2_env_setup_dds_with_domain(distro, domain_id);
        let python_script = r#"
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class Server(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._srv = ActionServer(self, Fibonacci, '/fibonacci', self.execute)
        print('SERVER READY', flush=True)
    def execute(self, goal_handle):
        order = goal_handle.request.order
        print(f'SERVER GOAL order={order}', flush=True)
        seq = [0, 1]
        for i in range(1, order):
            seq.append(seq[i] + seq[i - 1])
            fb = Fibonacci.Feedback()
            fb.sequence = seq
            goal_handle.publish_feedback(fb)
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = seq
        print(f'SERVER DONE {seq}', flush=True)
        return result

rclpy.init()
node = Server()
rclpy.spin(node)
"#;
        // Quoted heredoc so the script's real newlines reach python —
        // `python3 -c '<\n literals>'` is a SyntaxError (see add_two_ints_server).
        let cmd = format!(
            "{env_setup} && timeout --foreground 60 python3 -u - 2>&1 <<'NROS_PYEOF'\n{python_script}\nNROS_PYEOF"
        );
        Self::spawn_bash(&cmd, "ros2-dds fibonacci_action_server")
    }

    /// ROS 2 DDS `ros2 action send_goal --feedback` on a domain.
    pub fn action_send_goal_with_domain(
        action_name: &str,
        action_type: &str,
        goal: &str,
        distro: &str,
        domain_id: u8,
    ) -> TestResult<Self> {
        let env_setup = ros2_env_setup_dds_with_domain(distro, domain_id);
        let cmd = format!(
            "{env_setup} && timeout --foreground 20 ros2 action send_goal --feedback {action_name} {action_type} \"{goal}\""
        );
        Self::spawn_bash(&cmd, format!("ros2-dds action send_goal {action_name}"))
    }

    /// Wait for output and return it
    /// Collect output until `stop` is satisfied, the process exits, or `timeout`
    /// elapses — the shared engine behind [`Self::wait_for_output`] and
    /// [`Self::wait_for_output_count`] (phase-342 W9).
    ///
    /// Same shape as `Ros2Process::collect_until`, and deliberately a second
    /// copy rather than a trait: these two structs own different handles and
    /// differ in their exit handling, so the honest move is two small engines
    /// over one abstraction that fits neither. If a third appears, extract.
    fn collect_until(
        &mut self,
        stop: Option<(&str, usize)>,
        timeout: Duration,
    ) -> TestResult<(String, bool)> {
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

        // Set non-blocking mode on stdout so read() doesn't block forever
        #[cfg(unix)]
        let fd = {
            let fd = stdout.as_raw_fd();
            unsafe {
                let flags = libc::fcntl(fd, libc::F_GETFL);
                libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
            }
            fd
        };

        let satisfied = |out: &str| match stop {
            Some((pat, n)) => out.matches(pat).count() >= n,
            None => false,
        };

        let mut buffer = [0u8; 4096];
        loop {
            if satisfied(&output) {
                self.handle.stdout = Some(stdout);
                return Ok((output, true));
            }
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
                    break;
                }
                Ok(None) => match stdout.read(&mut buffer) {
                    Ok(0) => {
                        Self::wait_for_data(
                            #[cfg(unix)]
                            fd,
                            timeout.saturating_sub(start.elapsed()),
                        );
                    }
                    Ok(n) => {
                        output.push_str(&String::from_utf8_lossy(&buffer[..n]));
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        Self::wait_for_data(
                            #[cfg(unix)]
                            fd,
                            timeout.saturating_sub(start.elapsed()),
                        );
                    }
                    Err(_) => break,
                },
                Err(_) => break,
            }
        }

        let matched = satisfied(&output);
        Ok((output, matched))
    }

    /// Drain STDOUT until the process exits or `timeout` elapses, then KILL it.
    ///
    /// **A terminal drain, not a wait-for-readiness.** `collect_until(None, …)`
    /// has no stop condition, so a process that keeps running always reaches
    /// the timeout — and reaching it calls `kill_process_group`. To wait for
    /// readiness and KEEP the process, use [`Self::wait_for_output_count`].
    ///
    /// Issue 0672 called the misuse "a 5 s sleep that never observed anything",
    /// which understates it in the direction that matters: a sleep leaves the
    /// server running, and this one killed it. The client was then started
    /// against a server that was reliably gone, not one that might not have
    /// been listening.
    pub fn wait_for_output(&mut self, timeout: Duration) -> TestResult<String> {
        self.collect_until(None, timeout).map(|(out, _)| out)
    }

    /// Wait until `pattern` has appeared `expected` times, returning as soon as
    /// it has (phase-342 W9) — so a DDS-side test waits for the delivery it
    /// asserts rather than sleeping a fixed duration.
    pub fn wait_for_output_count(
        &mut self,
        pattern: &str,
        expected: usize,
        timeout: Duration,
    ) -> TestResult<String> {
        match self.collect_until(Some((pattern, expected)), timeout)? {
            (out, true) => Ok(out),
            (out, false) => Err(TestError::ProcessFailed(format!(
                "{} printed `{}` fewer than {} time(s) within {:?}. Output:\n{}",
                self.name, pattern, expected, timeout, out
            ))),
        }
    }

    /// Wait for data on a file descriptor (or sleep on non-Unix).
    #[cfg(unix)]
    fn wait_for_data(fd: std::os::unix::io::RawFd, remaining: Duration) {
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
    fn wait_for_data(remaining: Duration) {
        std::thread::sleep(remaining.min(Duration::from_millis(50)));
    }

    /// Kill the process
    pub fn kill(&mut self) {
        kill_process_group(&mut self.handle);
    }

    /// Check if process is still running
    pub fn is_running(&mut self) -> bool {
        matches!(self.handle.try_wait(), Ok(None))
    }
}

impl Drop for Ros2DdsProcess {
    fn drop(&mut self) {
        self.kill();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ros2_env_setup() {
        let (setup, _config_dir) = ros2_env_setup("humble");
        assert!(setup.contains("/opt/ros/humble"));
        assert!(setup.contains("rmw_zenoh_cpp"));
        assert!(setup.contains("ZENOH_SESSION_CONFIG_URI"));
    }

    #[test]
    fn test_ros2_env_setup_dds_format() {
        let setup = ros2_env_setup_dds("humble");
        assert!(setup.contains("/opt/ros/humble"));
        assert!(setup.contains("rmw_fastrtps_cpp"));
        // DDS setup should NOT contain zenoh config
        assert!(!setup.contains("ZENOH"));
    }

    #[test]
    fn test_ros2_detection() {
        // Just verify detection works, don't require ROS 2
        let available = is_ros2_available();
        eprintln!("ROS 2 available: {}", available);
    }

    #[test]
    fn test_rmw_zenoh_detection() {
        let available = is_rmw_zenoh_available();
        eprintln!("rmw_zenoh_cpp available: {}", available);
    }

    #[test]
    fn test_rmw_fastrtps_detection() {
        let available = is_rmw_fastrtps_available();
        eprintln!("rmw_fastrtps_cpp available: {}", available);
    }

    /// Issue 0690 — the report shape, taken verbatim from issue 0312's capture
    /// plus a second publisher, which is the situation the sweep produces.
    const TWO_PUBLISHERS: &str = "\
Type: std_msgs/msg/String

Publisher count: 2

Node name: qos_talker
Endpoint type: PUBLISHER
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (10)
  Durability: TRANSIENT_LOCAL

Node name: foreign_talker
Endpoint type: PUBLISHER
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (10)
  Durability: VOLATILE

Node name: qos_listener
Endpoint type: SUBSCRIPTION
QoS profile:
  Reliability: RELIABLE
  Durability: TRANSIENT_LOCAL
";

    #[test]
    fn endpoints_are_split_by_node_and_kind() {
        let eps = topic_endpoints(TWO_PUBLISHERS);
        assert_eq!(eps.len(), 3, "{eps:#?}");
        assert_eq!(eps[0].node, "qos_talker");
        assert_eq!(eps[0].kind, "PUBLISHER");
        assert_eq!(eps[2].kind, "SUBSCRIPTION");
        // Each record carries its OWN QoS and not the next one's.
        assert!(eps[0].block.contains("TRANSIENT_LOCAL"));
        assert!(eps[1].block.contains("Durability: VOLATILE"));
        assert!(!eps[1].block.contains("TRANSIENT_LOCAL"));
    }

    /// The regression this issue is about: positional selection reads whichever
    /// publisher ROS 2 listed first. Built explicitly with the FOREIGN endpoint
    /// first and carrying `nros_c_qos_default()` — which is the profile the
    /// in-sweep failure actually printed.
    #[test]
    fn selecting_by_node_ignores_a_foreign_publisher() {
        const FOREIGN_FIRST: &str = "\
Type: std_msgs/msg/String

Publisher count: 2

Node name: foreign_talker
Endpoint type: PUBLISHER
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (10)
  Durability: VOLATILE

Node name: qos_talker
Endpoint type: PUBLISHER
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (10)
  Durability: TRANSIENT_LOCAL
";

        // Positional reads the foreign block — the default profile, which is
        // what `case_08_c_qos` reported when it failed in-sweep.
        let positional = topic_endpoint_block(FOREIGN_FIRST, "PUBLISHER").unwrap();
        assert!(
            positional.contains("Durability: VOLATILE"),
            "positional selection should read the FIRST block:\n{positional}"
        );

        // Node-selected reads the endpoint under test, whatever the order.
        let mine = topic_endpoints_for_node(FOREIGN_FIRST, "PUBLISHER", "qos_talker");
        assert_eq!(mine.len(), 1, "{mine:#?}");
        assert!(
            mine[0].block.contains("Durability: TRANSIENT_LOCAL"),
            "node selection must return the node under test:\n{}",
            mine[0].block
        );
    }

    /// Node name does not always identify ONE endpoint — the three `*_qos`
    /// cells all name their node `qos_talker`. That case must stay visible to
    /// the caller rather than collapsing to "the first match", which is the bug
    /// one level up.
    #[test]
    fn two_endpoints_sharing_a_node_name_are_both_returned() {
        let siblings = TWO_PUBLISHERS.replace("foreign_talker", "qos_talker");
        let found = topic_endpoints_for_node(&siblings, "PUBLISHER", "qos_talker");
        assert_eq!(found.len(), 2, "{found:#?}");
    }
}

/// A one-shot fingerprint of the DDS/XRCE environment, for an interop failure.
///
/// Issue 0741 — three hosts ran the same command on the same tree: one failed
/// deterministically, two passed. Nobody could say what differed, because the
/// failure printed the processes' output and nothing about the stack under
/// them. Two more "does not reproduce" reports would not have settled it
/// either; what settles it is the FAILING run describing its own environment.
///
/// Deliberately cheap and total: every probe degrades to a note rather than an
/// error, because this runs on a path that is already failing and must not add
/// a second failure mode of its own.
pub fn interop_environment_fingerprint() -> String {
    use std::fmt::Write as _;
    let mut s = String::from("--- interop environment ---\n");

    for var in [
        "ROS_DISTRO",
        "RMW_IMPLEMENTATION",
        "ROS_DOMAIN_ID",
        "ZENOH_SESSION_CONFIG_URI",
    ] {
        let _ = writeln!(
            s,
            "  {var}={}",
            std::env::var(var).unwrap_or_else(|_| "<unset>".into())
        );
    }

    // The agent registers the DDS type that a ROS reader sizes its history
    // from, so its identity is the first thing to compare when a reader refuses
    // a correctly-sized sample.
    let agent = crate::fixtures::xrce_agent_binary_path();
    let _ = writeln!(s, "  xrce agent: {}", agent.display());
    if let Ok(md) = std::fs::metadata(&agent) {
        let _ = writeln!(s, "    size={} bytes", md.len());
    } else {
        let _ = writeln!(s, "    (not present at that path)");
    }

    // Fast-DDS / rmw_fastrtps versions, from the ament index rather than dpkg:
    // it works whatever installed ROS, and it names the prefix actually in use.
    // EVERY prefix, and both packages independently. Breaking at the first
    // prefix that yields anything under-reports: `fastrtps` and
    // `rmw_fastrtps_cpp` can live in different prefixes, and Fast-DDS's own
    // version is the field this issue most needs — reporting only the wrapper's
    // would be a fingerprint that omits the fingerprint.
    let prefixes = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default();
    for pkg in ["fastrtps", "rmw_fastrtps_cpp"] {
        let mut seen: Vec<String> = Vec::new();
        for prefix in prefixes.split(':').filter(|p| !p.is_empty()) {
            let share = std::path::Path::new(prefix).join("share").join(pkg);
            // TWO shapes, because Fast-DDS is not an ament package: ROS
            // packages declare `<version>` in `share/<pkg>/package.xml`, while
            // `fastrtps` ships only a cmake config
            // (`cmake/<pkg>-config-version.cmake`, `set(PACKAGE_VERSION "…")`).
            // Probing one shape reported the wrapper's version and a permanent
            // "not found" for the library underneath it — which is exactly the
            // field this issue turns on.
            let ver = std::fs::read_to_string(share.join("package.xml"))
                .ok()
                .and_then(|b| {
                    b.split("<version>")
                        .nth(1)
                        .and_then(|r| r.split("</version>").next())
                        .map(str::to_string)
                })
                .or_else(|| {
                    let cfg = share
                        .join("cmake")
                        .join(format!("{pkg}-config-version.cmake"));
                    std::fs::read_to_string(cfg).ok().and_then(|b| {
                        b.split("set(PACKAGE_VERSION")
                            .nth(1)
                            .and_then(|r| r.split('"').nth(1))
                            .map(str::to_string)
                    })
                });
            if let Some(ver) = ver
                && !seen.contains(&ver)
            {
                seen.push(ver);
            }
        }
        let _ = match seen.len() {
            0 => writeln!(s, "  {pkg}: not found on AMENT_PREFIX_PATH"),
            // More than one is worth seeing rather than collapsing: two
            // versions of the same package on the path is itself an answer.
            _ => writeln!(s, "  {pkg}: {}", seen.join(", ")),
        };
    }
    s
}

#[cfg(test)]
mod fingerprint_tests {
    /// Issue 0741 — the fingerprint runs on an ALREADY-FAILING path, so the one
    /// thing it must never do is add a second failure. Total, and it names the
    /// fields a reader needs even when every probe comes up empty.
    #[test]
    fn the_fingerprint_is_total_and_names_its_fields() {
        let s = super::interop_environment_fingerprint();
        // Visible under `--nocapture`: the point of the fingerprint is to be
        // READ, and a test that only asserts on it never shows what it says.
        eprintln!("{s}");
        for want in ["ROS_DISTRO", "RMW_IMPLEMENTATION", "xrce agent", "fastrtps"] {
            assert!(s.contains(want), "fingerprint omitted {want}:\n{s}");
        }
    }
}

/// Who else is on the DDS bus, captured WHILE the peers are still alive.
///
/// Issue 0741 — the failing host and two passing hosts have now matched on every
/// static axis, including the `libfastrtps` / `librmw_fastrtps_cpp` binaries
/// byte for byte. If the software is identical, the remaining difference is
/// what ELSE is on the bus, and that is not something a version comparison can
/// reach.
///
/// It matters because the failure's own mechanism allows it: the reply reader
/// sizes its history from a max-serialized-size learned AT DISCOVERY, and
/// nothing requires the endpoint it learned from to be the one under test. A
/// history of 15 bytes for a type whose correct wire size is 28 is what
/// matching something else looks like.
///
/// Must be called BEFORE the server and agent are torn down — after `kill()`
/// the bus is empty and the snapshot says nothing. That ordering is the whole
/// reason this is a separate function rather than another line in
/// [`interop_environment_fingerprint`], which runs inside the assert.
///
/// Total: every probe degrades to a note. This runs on an already-failing path.
pub fn dds_bus_snapshot(distro: &str, domain_id: u8) -> String {
    use std::fmt::Write as _;
    let env = ros2_env_setup_dds_with_domain(distro, domain_id);
    // The domain is stated because the test passes it per-invocation rather
    // than exporting it, so the fingerprint's `ROS_DOMAIN_ID=<unset>` is honest
    // about the process and says nothing about the bus this ran on.
    let mut s = format!("--- DDS bus, domain {domain_id}, peers still alive ---\n");
    // An EMPTY `nodes` list is the normal reading, not a failed probe: the XRCE
    // agent creates DDS participants on behalf of its clients, and those are not
    // ROS nodes. `services` and `topics` are where a foreign endpoint shows up —
    // a SECOND `/add_two_ints`, or a reply topic nobody in this test created.
    //
    // Issue 1009 — `--no-daemon` on every one, like the three sibling probes at
    // the top of this file. Without it each call STARTS a ros2 daemon on this
    // domain and leaves it running; `domain_discovery_port_busy` then reads the
    // port as busy and the next run steps to the NEXT domain. That is not a
    // tidiness point: it silently walks a failing test off the domain that made
    // it fail, so a real, reproducible failure presents as "one red then dozens
    // of greens" — which is exactly the pattern issue 0741 recorded and could
    // not explain, while a foreign CycloneDDS server sat on domains 1 and 5.
    //
    // Note this probe could not have caught 0741 even so: `ros2 service list`
    // collapses a service to one NAME however many servers offer it, so the
    // "SECOND /add_two_ints" the comment above hopes for never appears as a
    // second row. Fixing the daemon leak is necessary, not sufficient.
    for (label, sub) in [
        ("nodes", "node list --no-daemon"),
        ("services", "service list -t --no-daemon"),
        // Hidden topics included: a service's request/reply pair is hidden, and
        // hiding them is precisely what would keep a foreign endpoint invisible.
        (
            "topics",
            "topic list -t --include-hidden-topics --no-daemon",
        ),
    ] {
        let out = std::process::Command::new("bash")
            .args(["-c", &format!("{env} && timeout 10 ros2 {sub} 2>&1")])
            .output();
        match out {
            Ok(o) => {
                let body = String::from_utf8_lossy(&o.stdout);
                let body = body.trim();
                let _ = writeln!(
                    s,
                    "  [{label}]\n{}",
                    if body.is_empty() {
                        "    <empty>".to_string()
                    } else {
                        body.lines()
                            .map(|l| format!("    {l}"))
                            .collect::<Vec<_>>()
                            .join("\n")
                    }
                );
            }
            Err(e) => {
                let _ = writeln!(s, "  [{label}] could not run: {e}");
            }
        }
    }
    s
}

/// Is `service` ALREADY advertised on `domain_id`, before this test starts
/// anything?
///
/// Issue 0741 — the precondition five sessions of that investigation assumed
/// and nobody checked. A ROS 2 service is not owned by the node that names it:
/// any participant on the domain advertising the same name and type is a peer
/// the client may talk to instead, and on this bus that is not a fair coin —
/// whoever answers first wins.
///
/// The check is meaningful only BEFORE the node under test comes up, because
/// `ros2 service list` cannot say WHOSE endpoint it found (Humble has no
/// `ros2 service info`). Called at that moment the answer is unambiguous:
/// anything present is foreign.
///
/// `None` when the probe could not run at all — a probe that cannot see must
/// not invent, the same rule [`dds_bus_snapshot`] follows.
pub fn service_present_on_domain(distro: &str, domain_id: u8, service: &str) -> Option<bool> {
    let env = ros2_env_setup_dds_with_domain(distro, domain_id);
    let out = std::process::Command::new("bash")
        .args([
            "-c",
            &format!("{env} && timeout 10 ros2 service list 2>/dev/null"),
        ])
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let body = String::from_utf8_lossy(&out.stdout);
    Some(body.lines().any(|l| l.trim() == service))
}

/// Recognise the cross-RMW service-framing collision in a ROS 2 client's output,
/// and say what it is.
///
/// Issue 0741, root cause. `rmw_cyclonedds_cpp` and `rmw_fastrtps_cpp` do not
/// frame ROS 2 service replies the same way and are NOT interoperable for
/// services:
///
/// * `rmw_cyclonedds_cpp` inlines a 16-byte request header
///   (`[client_guid:8][seq:8]`) in the reply PAYLOAD — so an
///   `AddTwoInts_Response` is `4` encapsulation + `16` header + `8` sum = **28**
///   bytes. nano-ros's own Cyclone backend mirrors that framing deliberately
///   (`build_wire_with_header`, `packages/rmw/cyclonedds/nros-rmw-cyclonedds/src/service.cpp`).
/// * `rmw_fastrtps_cpp` carries the same correlation in the RTPS inline QoS
///   (`related_sample_identity`), so its reply payload is `4 + 8` = **12** bytes.
///   The type is BOUNDED, which makes rmw_fastrtps choose
///   `PREALLOCATED_MEMORY_MODE`, and Fast-DDS sizes the reader history at
///   `m_typeSize + 3` = **15**. Preallocated means strict: a larger sample is
///   refused rather than resized.
///
/// So a Fast-DDS service client that discovers a Cyclone reply writer refuses a
/// 28-byte change into a 15-byte history, and neither side is wrong. Verified as
/// a control with no nano-ros and no XRCE agent in the picture: a stock
/// `rmw_cyclonedds_cpp` `demo_nodes_cpp add_two_ints_server` plus a stock
/// `rmw_fastrtps_cpp` `ros2 service call` on one domain reproduces the message
/// byte for byte.
///
/// Which is why this must never be reported as an XRCE, encoding or type
/// registration defect: the numbers do not come from nano-ros's XRCE path at
/// all. Both were reasoned about for weeks.
pub fn cross_rmw_service_framing_note(client_output: &str) -> Option<String> {
    if !(client_output.contains("RTPS_READER_HISTORY")
        && client_output.contains("cannot be resized"))
    {
        return None;
    }
    Some(
        "--- issue 0741: this is a CROSS-RMW collision, not an XRCE defect ---\n\
         `RTPS_READER_HISTORY … cannot be resized` on a service reply means this\n\
         rmw_fastrtps client received a reply framed by rmw_cyclonedds_cpp:\n\
         cyclone inlines a 16-byte request header in the reply PAYLOAD (28 bytes\n\
         for AddTwoInts_Response), fastrtps carries it in inline QoS (12 bytes)\n\
         and preallocates a 15-byte history for the bounded type. The two RMWs\n\
         are not service-interoperable.\n\
         So a FOREIGN Cyclone `/add_two_ints` server is on this ROS domain — an\n\
         orphan from an earlier run (issue 0659/0707's class) or a concurrent\n\
         Cyclone fixture. Check `pgrep -a add_two_ints_server` and the bus\n\
         snapshot below; nothing in the XRCE path computes either number.\n"
            .to_string(),
    )
}

#[cfg(test)]
mod cross_rmw_note_tests {
    /// The signature must be recognised, and nothing else must be.
    #[test]
    fn recognises_only_the_history_refusal() {
        let red = "requester: making request: …\n[RTPS_READER_HISTORY Error] Change payload \
                   size of '28' bytes is larger than the history payload size of '15' bytes \
                   and cannot be resized.";
        let note = super::cross_rmw_service_framing_note(red)
            .expect("the 28-into-15 refusal must be recognised");
        assert!(
            note.contains("CROSS-RMW"),
            "note must name the cause: {note}"
        );
        assert!(
            super::cross_rmw_service_framing_note("response:\nAddTwoInts_Response(sum=8)")
                .is_none(),
            "a passing run must produce no note"
        );
        assert!(
            super::cross_rmw_service_framing_note("[RTPS_READER_HISTORY Error] something else")
                .is_none(),
            "the marker alone is not the signature — the refusal is"
        );
    }
}
