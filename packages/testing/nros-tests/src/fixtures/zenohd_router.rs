//! ZenohRouter fixture for managing zenohd process
//!
//! Provides automatic startup and cleanup of the zenoh router daemon.

use crate::{TestError, TestResult, process::graceful_kill_process_group};
use std::{
    io::Read,
    net::TcpStream,
    process::{Child, Stdio},
    time::{Duration, Instant},
};

/// Issue 0470 — lease a TCP port, exclusive until the lease drops.
///
/// This used to bind port 0, read the number and CLOSE the socket. The comment
/// justifying it ("safe for nextest where each test runs in a separate
/// process") drew the wrong conclusion from a true premise: separate processes
/// are exactly why an in-process scheme cannot work, and the closed socket left
/// the port free for the kernel to hand to a concurrent caller. See
/// `crate::port_lease` for the measurement and the XRCE failure it produced.
fn lease_ephemeral_port() -> std::io::Result<crate::port_lease::PortLease> {
    crate::port_lease::lease_port(crate::port_lease::Transport::Tcp)
}

/// Kill any process listening on the given TCP port.
///
/// Orphaned zenohd processes can survive across test runs when nextest
/// SIGKILL's a test process (preventing Drop from running). This function
/// detects and kills such orphans before starting a new router.
fn kill_listeners_on_port(port: u16) {
    if TcpStream::connect(format!("127.0.0.1:{}", port)).is_err() {
        return; // nothing listening
    }
    eprintln!(
        "WARNING: port {} already in use — killing orphaned process",
        port
    );
    // fuser -k sends SIGKILL to all processes using the port
    let _ = std::process::Command::new("fuser")
        .args(["-k", &format!("{}/tcp", port)])
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status();
    // Wait for the port to actually become free
    let start = std::time::Instant::now();
    while start.elapsed() < Duration::from_secs(5) {
        if TcpStream::connect(format!("127.0.0.1:{}", port)).is_err() {
            return; // port is now free
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    eprintln!("WARNING: port {} still in use after kill attempt", port);
}

/// phase-362 W1 — build the router command. The router comes from ROS.
///
/// `rmw_zenohd` takes NO command-line configuration: it ignores its argv (a
/// `--help` starts a router) and reads `ZENOH_ROUTER_CONFIG_URI` /
/// `ZENOH_CONFIG_OVERRIDE` instead. So every `--listen` / `--cfg` this fixture
/// used to pass becomes an override entry, `;`-separated, with `=` where the
/// CLI used `:`. Verified against the installed binary rather than inferred:
/// `listen/endpoints=["tcp/127.0.0.1:17447"]` binds 17447 and NOT the default
/// 7447.
///
/// `scouting/multicast/enabled=false` is stated even though the ROS default
/// config already sets it: the vendored router needed `--no-multicast-scouting`
/// explicitly, and a default is a thing that can change under us. This is the
/// one property these tests actually depend on.
/// The `libzenohc.so` the resolved router was BUILT against.
///
/// `rmw_zenohd` links `libzenohc.so` by SONAME, so which one it actually loads
/// is decided by the loader, not by which router we resolved. ROS ships its own
/// under `<prefix>/opt/zenoh_cpp_vendor/lib`, and that directory is on
/// `LD_LIBRARY_PATH` only if the caller sourced `setup.bash` (or this repo's
/// `activate.sh`, which adds the same entry). Without it, any other
/// `libzenohc.so` on the default search path wins — and a zenoh C library the
/// router was not built against does not fail to load, it SEGVs partway through
/// startup. On this host a stray `/lib/libzenohc.so` owned by no package did
/// exactly that, and every router-backed test failed with `signal: 11` and no
/// hint that the environment was the cause.
///
/// So the fixture pins the pairing itself instead of inheriting it. This is
/// RFC-0075's drift concern one layer down: the RFC keeps us from running a
/// router that disagrees with the ROS side, and this keeps that router from
/// loading a zenoh that disagrees with IT. Finding a binary and being able to
/// RUN it are different properties, and only the first was being checked.
///
/// Absent directory ⇒ `None` and the inherited environment stands: a layout
/// without a vendored zenoh is not ours to second-guess.
fn paired_zenoh_library_dir(router: &std::path::Path) -> Option<std::path::PathBuf> {
    // <prefix>/lib/rmw_zenoh_cpp/rmw_zenohd -> <prefix>/opt/zenoh_cpp_vendor/lib
    let prefix = router.parent()?.parent()?.parent()?;
    let dir = prefix.join("opt").join("zenoh_cpp_vendor").join("lib");
    dir.join("libzenohc.so").exists().then_some(dir)
}

fn router_command(overrides: &[String]) -> TestResult<std::process::Command> {
    let path = crate::process::ros_zenohd_path().ok_or_else(|| {
        TestError::RouterUnavailable(format!(
            "no `rmw_zenoh_cpp/rmw_zenohd` found. Source your ROS setup \
             (`source /opt/ros/<distro>/setup.bash`) — that exports \
             AMENT_PREFIX_PATH, which is how it is located; `rmw_zenohd` is NOT \
             on PATH even when installed. AMENT_PREFIX_PATH={}, ROS_DISTRO={}. \
             Otherwise install `ros-<distro>-rmw-zenoh-cpp`, or set \
             NROS_RMW_ZENOHD. Pairing: {}",
            std::env::var("AMENT_PREFIX_PATH").unwrap_or_else(|_| "unset".into()),
            std::env::var("ROS_DISTRO").unwrap_or_else(|_| "unset".into()),
            crate::process::zenoh_pairing_versions(),
        ))
    })?;

    // Issue 0653 — say which router this is, not just that one was found.
    //
    // Only `NROS_RMW_ZENOHD` can land here un-paired: the other search steps
    // look inside ament prefixes and cannot produce anything else. But that
    // escape hatch is exactly where a user points at a `zenohd` built from
    // zenoh's own instructions, whose version has no relationship to the
    // `rmw_zenoh_cpp` the ROS side is running — the drift RFC-0075 exists to
    // remove, arriving by the one door left open. A warning rather than an
    // error: the override is a deliberate act and the lane may be exactly what
    // the user is testing; what must not happen is it being SILENT.
    let provenance = crate::process::ros_zenohd_provenance(&path);
    if !provenance.is_ros_shipped() {
        nros_log::log_warn!(
            nros_log::get_logger("nros_tests"),
            "zenoh router {} — {}. Interop results from this run say nothing \
             about the paired configuration (RFC-0075).",
            path.display(),
            provenance.describe()
        );
    }

    let mut cmd = std::process::Command::new(&path);
    // Put the paired zenoh ahead of whatever the inherited environment would
    // resolve; see `paired_zenoh_library_dir`.
    if let Some(dir) = paired_zenoh_library_dir(&path) {
        let mut search = vec![dir];
        if let Some(inherited) = std::env::var_os("LD_LIBRARY_PATH") {
            search.extend(std::env::split_paths(&inherited));
        }
        if let Ok(joined) = std::env::join_paths(search) {
            cmd.env("LD_LIBRARY_PATH", joined);
        }
    }
    let mut all: Vec<String> = vec!["scouting/multicast/enabled=false".to_string()];
    all.extend_from_slice(overrides);
    cmd.env("ZENOH_CONFIG_OVERRIDE", all.join(";"));
    Ok(cmd)
}

/// The router binary and the config override it was handed, on one line.
///
/// `rmw_zenohd` takes no argv, so the argument list a `Command` carries says
/// nothing; the whole configuration is `ZENOH_CONFIG_OVERRIDE`. Rendering both
/// is what makes a hand-started router comparable to this one (issue 0877).
fn describe_router(cmd: &std::process::Command) -> String {
    let overrides = cmd
        .get_envs()
        .find(|(k, _)| *k == std::ffi::OsStr::new("ZENOH_CONFIG_OVERRIDE"))
        .and_then(|(_, v)| v)
        .map(|v| v.to_string_lossy().into_owned())
        .unwrap_or_default();
    format!(
        "ZENOH_CONFIG_OVERRIDE='{}' {}",
        overrides,
        cmd.get_program().to_string_lossy()
    )
}

fn wait_for_router_ready(handle: &mut Child, locator: &str, port: u16) -> TestResult<()> {
    let start = Instant::now();
    let timeout = Duration::from_secs(10);
    let addr = format!("127.0.0.1:{port}");

    while start.elapsed() < timeout {
        if TcpStream::connect(&addr).is_ok() {
            return Ok(());
        }

        if let Some(status) = handle.try_wait()? {
            let mut stderr = String::new();
            if let Some(mut pipe) = handle.stderr.take() {
                let _ = pipe.read_to_string(&mut stderr);
            }
            let stderr = stderr.trim();
            let detail = if stderr.is_empty() {
                String::new()
            } else {
                format!(": {stderr}")
            };
            return Err(TestError::ProcessFailed(format!(
                "zenohd exited before listening on {locator} with {status}{detail}"
            )));
        }

        std::thread::sleep(Duration::from_millis(100));
    }

    graceful_kill_process_group(handle);
    Err(TestError::Timeout)
}

/// Managed zenohd router process
///
/// Automatically starts zenohd on creation and kills it on drop.
/// Uses OS-assigned ephemeral ports to allow parallel test execution
/// across nextest's separate test processes.
///
/// Supports both TCP and TLS listeners.
///
/// # Example
///
/// ```ignore
/// use nros_tests::fixtures::{ZenohRouter, or_skip};
///
/// // `or_skip`, never `.unwrap()`: a host with no `rmw_zenohd` must SKIP the
/// // lane, and only `or_skip` tells that apart from a router that is present
/// // and refuses to start (issues 0599, 0804).
/// let router = or_skip(ZenohRouter::start_unique());
/// println!("Router at: {}", router.locator());
/// // Router is automatically stopped when dropped
/// ```
pub struct ZenohRouter {
    handle: Child,
    port: u16,
    tls: bool,
    /// The router binary + config override this fixture actually started —
    /// see [`ZenohRouter::launch_line`].
    launch: String,
    /// Issue 0470 — held for the router's lifetime so no other fixture is
    /// handed this port. `None` when the caller named the port itself.
    _lease: Option<crate::port_lease::PortLease>,
}

impl ZenohRouter {
    /// Start a new zenohd router on the specified port, bound to `127.0.0.1`.
    ///
    /// Kills any orphaned zenohd still listening on the port from a previous
    /// test run (e.g. if nextest SIGKILL'd the test process, preventing Drop).
    ///
    /// Binding to loopback prevents cross-platform interference and is
    /// sufficient for native tests — every peer is a host process reaching the
    /// router over `127.0.0.1`. "native" here is the ROLE (host build), not a
    /// reach claim; the platform beneath it happens to be `posix`.
    ///
    /// For bridge-networked tests (ThreadX Linux sim) that connect via
    /// a non-loopback IP, use [`start_on`](Self::start_on) with `"0.0.0.0"`.
    pub fn start(port: u16) -> TestResult<Self> {
        Self::start_on("127.0.0.1", port)
    }

    /// Start a router for QEMU user-mode networking guests.
    ///
    /// Slirp guests connect to the host through gateway `10.0.2.2`; binding
    /// only to loopback can leave those guest SYNs unreachable on some hosts.
    pub fn start_slirp(port: u16) -> TestResult<Self> {
        Self::start_on("0.0.0.0", port)
    }

    /// Start a new zenohd router on the specified bind address and port.
    ///
    /// # Arguments
    /// * `bind_addr` - IP address to bind to (`"127.0.0.1"` or `"0.0.0.0"`)
    /// * `port` - TCP port to listen on
    ///
    /// # Returns
    /// A managed router instance that will be stopped on drop
    pub fn start_on(bind_addr: &str, port: u16) -> TestResult<Self> {
        if !crate::process::is_local_tcp_listener_available() {
            return Err(TestError::ProcessFailed(
                "local TCP listeners unavailable in this environment".to_string(),
            ));
        }

        // Kill any orphaned zenohd from a previous test run
        kill_listeners_on_port(port);

        let locator = format!("tcp/{}:{}", bind_addr, port);

        let mut cmd = router_command(&[format!("listen/endpoints=[\"{locator}\"]")])?;
        // Diagnostic log capture per port — opt-in, unified dir. Enabled by
        // ZENOHD_LOG=trace|debug (also sets RUST_LOG level) or NROS_TEST_LOGS;
        // the file lands in test-logs/fixtures/ (see fixtures::fixture_log_path).
        // Defaults to null sinks so a normal run leaves nothing behind.
        let zenohd_log = std::env::var("ZENOHD_LOG").ok();
        if zenohd_log.is_some() || crate::fixtures::fixture_logs_enabled() {
            let log_path = crate::fixtures::fixture_log_path(&format!("zenohd-{port}"));
            let log = std::fs::File::create(&log_path).map_err(TestError::ProcessStart)?;
            let log_stdout = log.try_clone().map_err(TestError::ProcessStart)?;
            cmd.env("RUST_LOG", zenohd_log.as_deref().unwrap_or("info"))
                .stdout(Stdio::from(log_stdout))
                .stderr(Stdio::from(log));
        } else {
            cmd.stdout(Stdio::null()).stderr(Stdio::piped());
        }
        #[cfg(unix)]
        crate::process::set_new_process_group(&mut cmd);
        let launch = describe_router(&cmd);
        let mut handle = cmd.spawn()?;

        // Wait for zenohd to be ready (TCP port accepting connections)
        // 10s allows for slow startup under concurrent test load
        wait_for_router_ready(&mut handle, &locator, port)?;

        Ok(Self {
            handle,
            port,
            tls: false,
            launch,
            _lease: None,
        })
    }

    /// Start a zenohd router with serial listeners on the given PTY paths
    ///
    /// Each PTY path is added as a `serial/<path>#baudrate=115200` listener.
    /// No TCP listener is created — the router is only reachable via serial.
    ///
    /// # Arguments
    /// * `pty_paths` - Host PTY device paths (e.g., `/dev/pts/5`)
    pub fn start_serial(pty_paths: &[&str]) -> TestResult<Self> {
        // phase-362 W1 — one `listen/endpoints` LIST, not a repeated flag: the
        // CLI took `--listen` once per locator, the config takes an array.
        let endpoints = pty_paths
            .iter()
            .map(|pty| format!("\"serial/{pty}#baudrate=115200\""))
            .collect::<Vec<_>>()
            .join(",");
        let mut cmd = router_command(&[format!("listen/endpoints=[{endpoints}]")])?;

        let zenohd_log = std::env::var("ZENOHD_LOG").ok();
        if zenohd_log.is_some() || crate::fixtures::fixture_logs_enabled() {
            let log_path = crate::fixtures::fixture_log_path("zenohd-serial");
            let log = std::fs::File::create(&log_path).map_err(TestError::ProcessStart)?;
            let log_stdout = log.try_clone().map_err(TestError::ProcessStart)?;
            cmd.env("RUST_LOG", zenohd_log.as_deref().unwrap_or("info"))
                .stdout(Stdio::from(log_stdout))
                .stderr(Stdio::from(log));
        } else {
            cmd.stdout(std::process::Stdio::null())
                .stderr(std::process::Stdio::piped());
        }
        #[cfg(unix)]
        crate::process::set_new_process_group(&mut cmd);
        let launch = describe_router(&cmd);
        let mut handle = cmd.spawn()?;

        // Serial listeners don't have a TCP port to probe, so wait a bit
        // for zenohd to initialize and open the serial devices.
        std::thread::sleep(Duration::from_secs(2));

        // #189 — FAIL LOUD if the router already died. A zenohd built without
        // `zenoh/transport_serial` (the pre-nros2 SDK binaries) refuses the
        // serial listener ("Unicast not supported for serial protocol") and
        // exits within the sleep above; the old code returned the corpse and
        // every guest hung silently at its serial handshake until the test
        // timeout.
        if let Ok(Some(status)) = handle.try_wait() {
            return Err(TestError::ProcessFailed(format!(
                "zenohd exited ({status}) right after starting with serial \
                 listener(s) — the provisioned zenohd likely lacks the \
                 `zenoh/transport_serial` feature (needs the 1.7.2-nros2 \
                 build; re-run `just zenohd setup` after `git pull`)."
            )));
        }

        Ok(Self {
            handle,
            port: 0,
            tls: false,
            launch,
            _lease: None,
        })
    }

    /// Start a router with TLS listener on the specified port
    ///
    /// # Arguments
    /// * `port` - TCP port to listen on
    /// * `cert_path` - Path to PEM certificate file
    /// * `key_path` - Path to PEM private key file
    pub fn start_tls(
        port: u16,
        cert_path: &std::path::Path,
        key_path: &std::path::Path,
    ) -> TestResult<Self> {
        kill_listeners_on_port(port);

        // issue 0408 — listen DUAL-STACK, not IPv4-only. Clients reach this
        // router by NAME (`tls/localhost:<port>`, matching the CN=localhost in
        // the self-signed test cert), and on a dual-stack host `localhost`
        // resolves to `::1` FIRST. Against an IPv4-only listener that attempt is
        // refused, and zenoh-pico then fails the IPv4 fallback outright:
        //
        //     connect(::1:7922)       = ECONNREFUSED
        //     connect(127.0.0.1:7922) = EINVAL      <- same socket, not a fresh one
        //
        // so the session never opens and the test reports "Listener: expected at
        // least 1 received messages, got 0" — which reads as a delivery or
        // discovery fault, not an address-family one. `[::]` accepts both
        // families on Linux (bindv6only=0), so either resolution order works.
        //
        // The plain-TCP lanes never hit this because they connect to a literal
        // 127.0.0.1; only the TLS lane needs a hostname, for cert verification.
        //
        // The EINVAL retry is a zenoh-pico robustness bug in its own right — a
        // failed connect leaves the socket unusable and the next candidate
        // address needs a fresh one. Worth fixing in the fork; this makes the
        // harness stop provoking it.
        let listen = format!("tls/[::]:{}", port);
        // phase-362 W1 — `key=value` for the config override; the retired CLI
        // spelled the same keys `--cfg key:value`.
        let cert_cfg = format!(
            "transport/link/tls/listen_certificate=\"{}\"",
            cert_path.display()
        );
        let key_cfg = format!(
            "transport/link/tls/listen_private_key=\"{}\"",
            key_path.display()
        );

        let mut cmd = router_command(&[
            format!("listen/endpoints=[\"{listen}\"]"),
            cert_cfg,
            key_cfg,
        ])?;
        cmd.stdout(Stdio::null()).stderr(Stdio::piped());
        #[cfg(unix)]
        crate::process::set_new_process_group(&mut cmd);
        let launch = describe_router(&cmd);
        let mut handle = cmd.spawn()?;

        // Readiness probes the loopback IPv4 address: the dual-stack listener
        // accepts it, and it is what `kill_listeners_on_port` reasons about.
        let probe = format!("tls/127.0.0.1:{}", port);
        wait_for_router_ready(&mut handle, &probe, port)?;

        Ok(Self {
            handle,
            port,
            tls: true,
            launch,
            _lease: None,
        })
    }

    /// Start a TLS router on an OS-assigned ephemeral port (parallel-safe)
    pub fn start_tls_unique(
        cert_path: &std::path::Path,
        key_path: &std::path::Path,
    ) -> TestResult<Self> {
        let lease = lease_ephemeral_port()
            .map_err(|e| TestError::ProcessFailed(format!("Failed to lease port: {}", e)))?;
        let mut router = Self::start_tls(lease.port(), cert_path, key_path)?;
        router._lease = Some(lease);
        Ok(router)
    }

    /// Start a router on an OS-assigned ephemeral port (parallel-safe)
    pub fn start_unique() -> TestResult<Self> {
        let lease = lease_ephemeral_port()
            .map_err(|e| TestError::ProcessFailed(format!("Failed to lease port: {}", e)))?;
        let mut router = Self::start(lease.port())?;
        router._lease = Some(lease);
        Ok(router)
    }

    /// Get the locator string for connecting to this router
    ///
    /// TLS connections use `localhost` (not `127.0.0.1`) to match
    /// the CN=localhost in our self-signed test certificates, which
    /// avoids hostname verification failures.
    pub fn locator(&self) -> String {
        if self.tls {
            format!("tls/localhost:{}", self.port)
        } else {
            format!("tcp/127.0.0.1:{}", self.port)
        }
    }

    /// Get the port number
    pub fn port(&self) -> u16 {
        self.port
    }

    /// How this router was actually started — binary plus the
    /// `ZENOH_CONFIG_OVERRIDE` it was given.
    ///
    /// Issue 0877, same reason as [`crate::qemu::QemuProcess::command_line`]:
    /// a report whose reasoning rests on "I ran the same thing by hand" needs
    /// the harness to say what "the same thing" WAS. The router is the half
    /// nobody writes down — `rmw_zenohd` takes no argv, so a hand run looks
    /// identical while reading a different config, and which `libzenohc.so` it
    /// loads is the loader's decision, not the path's (issue 0774).
    #[must_use]
    pub fn launch_line(&self) -> &str {
        &self.launch
    }

    /// Check if the router is still running
    pub fn is_running(&mut self) -> bool {
        matches!(self.handle.try_wait(), Ok(None))
    }
}

impl Drop for ZenohRouter {
    fn drop(&mut self) {
        graceful_kill_process_group(&mut self.handle);
    }
}

/// rstest fixture for zenohd on port 7447 (native, loopback-only integration
/// tests).
///
/// QEMU slirp platform tests use `ZenohRouter::start_slirp(platform::*.zenohd_port)`
/// with per-platform ports (7450–7456) for parallel execution.
///
/// # Example
///
/// ```ignore
/// use nros_tests::fixtures::zenohd;
/// use rstest::rstest;
///
/// #[rstest]
/// fn my_test(zenohd: ZenohRouter) {
///     // zenohd is ready to use
/// }
/// ```
#[rstest::fixture]
pub fn zenohd() -> ZenohRouter {
    or_skip(ZenohRouter::start(7447))
}

/// rstest fixture for zenohd on an OS-assigned ephemeral port (parallel-safe)
#[rstest::fixture]
pub fn zenohd_unique() -> ZenohRouter {
    or_skip(ZenohRouter::start_unique())
}

/// phase-362 W3 — a host with no ROS router SKIPS, it does not fail.
///
/// The three outcomes this phase allows for a lane that used to spawn our
/// vendored router are: run routerless, run on `rmw_zenohd`, or skip with a
/// reason naming what is missing. This is the third — and the class is
/// `capability`, because a missing `rmw_zenoh_cpp` install is a property of the
/// HOST, like an absent cross toolchain or emulator.
///
/// The rule it serves is issue 0599's: a lane that cannot run must say so. The
/// alternative — falling back to a router no ROS 2 deployment runs — is how
/// these lanes came to test a configuration nobody deploys while reporting
/// green.
///
/// Every other error stays a failure. `RouterUnavailable` is the only one that
/// means "not runnable here"; a router that is present and will not start is a
/// real fault.
pub fn or_skip(res: TestResult<ZenohRouter>) -> ZenohRouter {
    match res {
        Ok(router) => router,
        Err(TestError::RouterUnavailable(why)) => {
            crate::skip_class!(capability, "{why}")
        }
        Err(e) => panic!("the ROS zenoh router failed to start: {e}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `paired_zenoh_library_dir` must derive the vendored zenoh dir from the
    /// router path alone, and must yield `None` when there is nothing to pair
    /// with — the two behaviours the SEGV fix rests on. Driven over a synthetic
    /// prefix rather than the host's ROS install, so it asserts the DERIVATION
    /// and not "this machine happens to have one".
    #[test]
    fn paired_zenoh_library_dir_derives_from_the_router_path() {
        let tmp = std::env::temp_dir().join(format!("nros-pairing-{}", std::process::id()));
        let router = tmp.join("lib/rmw_zenoh_cpp/rmw_zenohd");
        let libdir = tmp.join("opt/zenoh_cpp_vendor/lib");
        std::fs::create_dir_all(router.parent().unwrap()).unwrap();

        // No vendored zenoh next to the router -> inherit the environment.
        assert_eq!(
            paired_zenoh_library_dir(&router),
            None,
            "must not invent a directory that does not exist"
        );

        std::fs::create_dir_all(&libdir).unwrap();
        std::fs::write(libdir.join("libzenohc.so"), b"").unwrap();
        assert_eq!(
            paired_zenoh_library_dir(&router).as_deref(),
            Some(libdir.as_path()),
            "must pair the router with the zenoh shipped beside it"
        );

        std::fs::remove_dir_all(&tmp).ok();
    }

    #[test]
    fn test_zenoh_router_locator() {
        // Just test the locator format without starting a real router
        let port = 12345;
        let expected = "tcp/127.0.0.1:12345";
        assert_eq!(format!("tcp/127.0.0.1:{}", port), expected);
    }

    /// Issue 0470 — distinct while HELD. The previous version dropped each port
    /// before asking for the next and asserted they differed, which the racy
    /// allocator also satisfied: the kernel only re-hands a port once it has
    /// been released. A guard that cannot fail on its own defect is not a guard.
    #[test]
    fn leased_ports_are_distinct_while_held() {
        let held: Vec<_> = (0..16).map(|_| lease_ephemeral_port().unwrap()).collect();
        let mut ports: Vec<u16> = held.iter().map(|l| l.port()).collect();
        assert!(
            ports.iter().all(|p| *p > 1024),
            "port outside ephemeral range"
        );
        let total = ports.len();
        ports.sort_unstable();
        ports.dedup();
        assert_eq!(
            ports.len(),
            total,
            "two concurrently-held leases shared a port"
        );
    }
}
