//! Phase 212.L.5 — top-level init API.
//!
//! Three patterns are supported (per the Phase 212.L canonical pkg shape):
//!
//! 1. **Node pkg** — register via the [`nros::node!`](crate::node!)
//!    macro (Phase 172 W.3); the generated runtime owns the spin loop.
//! 2. **Application pkg + launch-aware** — call [`init_with_launch_auto`] (or
//!    [`init_with_launch`] for an explicit path). The returned [`Context`]
//!    carries launch-resolved fields (domain id, locator, RMW choice). User
//!    code drives its own spin via `Executor::open` +
//!    `Executor::spin_blocking`.
//! 3. **Application pkg + custom spin** — call [`init()`] (or [`init_with_args`]
//!    for argv-style overrides). Launch file is ignored; env vars +
//!    `ExecutorConfig::from_env()` semantics still apply.
//!
//! The [`Context`] struct is a thin holder of the resolved init knobs. To
//! actually open a session, materialise an [`crate::ExecutorConfig`] via
//! [`Context::config`] and pass it to `Executor::open`.
//!
//! ## Launch overlay (current limitation)
//!
//! `init_with_launch_auto` / `init_with_launch` currently consume the
//! launch-resolved knobs the parent `nros launch` process exports via env
//! vars (`ROS_DOMAIN_ID`, `NROS_LOCATOR`, `NROS_SESSION_MODE`,
//! `RMW_IMPLEMENTATION`, plus the placeholder `NROS_RUNTIME_OVERLAY` for
//! the future structured overlay path). The launch XML is NOT parsed
//! in-process; the runtime trusts the launcher to project the relevant
//! params / remaps / env into the child environment. A follow-up wave wires
//! the structured overlay (Option A — `nros launch --emit-runtime-overlay`
//! → JSON sidecar consumed here). See Phase 212.L.5 notes.

#[cfg(feature = "env")]
// phase-359 W10 — kept, and it is not a spelling that can be unwound.
// `init_with_launch` verifies a launch file EXISTS, which is a filesystem
// question; `AsRef<Path>` is also what a Rust caller expects to pass a
// `PathBuf`, a `&str` or a `Path` to. Narrowing the signature to `&str` would
// trade a real ergonomic for one census point, which is moving the number
// rather than the build. The whole module is behind `env`, which requires
// `std`, so nothing here is reachable without one.
use std::path::Path;

use nros_node::ExecutorConfig;
use nros_rmw::SessionMode;

/// Errors returned by the init API.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InitError {
    /// `init_with_launch(path)` was passed a path that does not exist or
    /// could not be read.
    LaunchFileNotFound,
    /// The launch file existed but could not be parsed.
    ///
    /// Phase 212.L.5 ships a stub — actual XML parsing arrives with the
    /// runtime-overlay wave. Until then this variant is unused.
    LaunchParseFailed,
    /// A launch-derived env var (`ROS_DOMAIN_ID`, etc.) failed to parse.
    EnvParseFailed,
}

impl core::fmt::Display for InitError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InitError::LaunchFileNotFound => f.write_str("launch file not found"),
            InitError::LaunchParseFailed => f.write_str("launch file parse failed"),
            InitError::EnvParseFailed => f.write_str("env var parse failed"),
        }
    }
}

// `core::error::Error` — `std::error::Error` is a re-export of it since Rust
// 1.81, so this is the same trait. The `cfg` stays: this whole module reads the
// environment (`std::env::var`), which has no no_std equivalent.
#[cfg(feature = "env")]
impl core::error::Error for InitError {}

/// Phase 212.L.5 — resolved init context.
///
/// Returned by every `init*` entry point. Carries the fields the user
/// needs to construct an [`ExecutorConfig`] and open a session.
///
/// Fields are owned (`String` on hosted builds) so the `Context` can
/// outlive transient parents (env caches, parsed launch files).
#[cfg(feature = "env")]
#[derive(Debug, Clone)]
pub struct Context {
    /// ROS 2 domain ID (`ROS_DOMAIN_ID`, default 0).
    pub domain_id: u32,
    /// Middleware locator (`NROS_LOCATOR` / legacy `ZENOH_LOCATOR`).
    pub locator: alloc::string::String,
    /// Session mode (`NROS_SESSION_MODE` / legacy `ZENOH_MODE`, default `Client`).
    pub mode: SessionMode,
    /// RMW implementation hint (`RMW_IMPLEMENTATION` /  `NROS_RMW`).
    ///
    /// Empty when neither var is set. The runtime uses this to pick a
    /// primary backend when multiple are linked; see
    /// `crate::internals::open_session`.
    pub rmw: alloc::string::String,
    /// Source of this context — useful for diagnostics + tests.
    pub source: ContextSource,
}

/// Where the [`Context`] came from. Diagnostics only.
#[cfg(feature = "env")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContextSource {
    /// Built from env vars by [`init`] / [`init_with_args`].
    Env,
    /// Built from a launch file (path supplied to [`init_with_launch`]) or
    /// auto-discovered via [`init_with_launch_auto`]. The launch XML itself
    /// is NOT yet parsed (see module docs); the launcher's projected env
    /// is the source of truth for now.
    Launch,
}

#[cfg(feature = "env")]
impl Context {
    /// Materialise an [`ExecutorConfig`] for a node with the given name.
    ///
    /// The returned config borrows from `self`, so callers usually do:
    ///
    /// ```ignore
    /// let ctx = nros::init()?;
    /// let cfg = ctx.config("talker");
    /// let mut executor = nros::Executor::open(&cfg)?;
    /// ```
    pub fn config<'a>(&'a self, node_name: &'a str) -> ExecutorConfig<'a> {
        ExecutorConfig::new(self.locator.as_str())
            .node_name(node_name)
            .domain_id(self.domain_id)
            .mode(self.mode)
    }
}

#[cfg(feature = "env")]
fn read_env_context(source: ContextSource) -> Result<Context, InitError> {
    // issue 0687 — through the ONE resolver, not a third parse of the same four
    // variables. This function had its own copy, and the copies had drifted:
    // it did not warn on the legacy `$ZENOH_LOCATOR`/`$ZENOH_MODE` spellings
    // the other reader deprecates, and it range-checked nothing, so
    // `ROS_DOMAIN_ID=300` reached a backend that could only fail later.
    //
    // Issue 0330 — no backend default for the locator: `nros` is RMW-agnostic,
    // so unset env leaves it EMPTY (= absent) and the linked backend
    // substitutes its own (zenoh: `nros_rmw_zenoh::DEFAULT_LOCATOR`; xrce: its
    // agent default; cyclonedds ignores the locator entirely). That is what
    // `resolve_hosted` does with an empty `BootConfig` too.
    let resolved = crate::env::try_resolve_hosted(nros_node::BootConfig::default())
        .map_err(|_| InitError::EnvParseFailed)?;
    let locator = alloc::string::String::from(resolved.locator);
    let domain_id = resolved.domain_id;
    let mode = resolved.mode;
    // issue 0687 — the `$NROS_RMW` half comes from the shared selector; the
    // `RMW_IMPLEMENTATION` fallback stays HERE and only here. `Context.rmw` is
    // a ROS-vocabulary HINT (`rmw_cyclonedds_cpp`), not the cffi registry
    // selector (`cyclonedds`) — folding the two together would hand a ROS name
    // to `resolve_backend`, which answers `Unknown` and fails the open.
    let rmw = crate::rmw_selector()
        .map(|s| alloc::string::String::from(s.as_str()))
        .or_else(|| std::env::var("RMW_IMPLEMENTATION").ok())
        .unwrap_or_default();
    Ok(Context {
        domain_id,
        locator,
        mode,
        rmw,
        source,
    })
}

/// Pattern 3 — raw init, launch file ignored.
///
/// Reads env vars (`ROS_DOMAIN_ID`, `NROS_LOCATOR`, `NROS_SESSION_MODE`,
/// `NROS_RMW` / `RMW_IMPLEMENTATION`) and returns a [`Context`]. The
/// caller owns the spin loop — typically `Executor::open(&ctx.config(name))`
/// followed by `spin_blocking` or a hand-rolled `spin_once` loop.
#[cfg(feature = "env")]
pub fn init() -> Result<Context, InitError> {
    read_env_context(ContextSource::Env)
}

/// The refusal `init_with_args` emits when it is handed `--ros-args`. The
/// same text `NROS_RCLCPP_REFUSE_INIT_ARGV` carries for the C++ twin
/// (`packages/api/nros-cpp/include/nros/log.hpp`), with the Rust spellings.
#[cfg(feature = "env")]
pub const REFUSE_INIT_ARGS: &str = "nros::init_with_args was given --ros-args, which nano-ros cannot honour \
(RFC-0089, phase-417 W3.b). Proceeding would DISCARD it, so `-r chatter:=/other` would \
silently become a wrong-topic bug at runtime -- the 'compiles and differs' the rule \
forbids. Nothing in this process parses --ros-args yet, and honouring them is remap \
resolution -- RFC-0020 violation class 4 -- so the parser belongs beside nros::resolve_name, \
not in this wrapper. Today remaps and parameter overrides come from the LAUNCHER, which \
projects them into the environment before exec. Call nros::init(), or \
nros::init_with_launch_auto() for the launch-aware entry point.";

/// The refusal's predicate, separately checkable (RFC-0089 §"where the
/// refusal fires": an abort inlined into `init` can only be observed by a
/// process that then dies, which is the shape of a check nothing runs).
///
/// Exact match only. `--ros-args-extra` is not the flag, and a prefix match
/// would refuse an argument nano-ros never drops — the mutation the C++
/// predicate's `static_assert`s pin, pinned here by a unit test.
#[cfg(feature = "env")]
pub fn args_have_ros_args<I, S>(args: I) -> bool
where
    I: IntoIterator<Item = S>,
    S: AsRef<str>,
{
    args.into_iter().any(|a| a.as_ref() == "--ros-args")
}

/// Pattern 3 — like [`init`] but accepts a `[--arg=value, ...]`-style argv
/// iterator.
///
/// **What it does with the arguments:** it does NOT parse them, and it does
/// not silently ignore them either. The structured argv parse (`--ros-args -p
/// foo:=42`, remaps) is remap resolution, which belongs beside
/// `nros::resolve_name` and lands with the runtime-overlay wave. Until it
/// does, an argument vector that carries `--ros-args` is REFUSED LOUDLY at the
/// call: the refusal is logged through `nros_log` (never `std::println!`,
/// issue 0589) and the process panics naming the flag — the same shape
/// `rclcpp::init(argc, argv)` has in `nros.hpp`. A vector with no ROS
/// arguments is unaffected, because nothing was dropped.
///
/// RFC-0089: only the VALUE carries the defect, so the call is the earliest
/// point it is knowable; silence there is the "compiles and differs" the
/// compile-or-conform rule forbids.
#[cfg(feature = "env")]
pub fn init_with_args<I, S>(args: I) -> Result<Context, InitError>
where
    I: IntoIterator<Item = S>,
    S: AsRef<str>,
{
    if args_have_ros_args(args) {
        nros_log::log_error!(nros_log::get_logger("nros"), "{}", REFUSE_INIT_ARGS);
        panic!("{}", REFUSE_INIT_ARGS);
    }
    init()
}

#[cfg(all(test, feature = "env"))]
mod ros_args_refusal_tests {
    use super::*;

    #[test]
    fn predicate_matches_the_flag_exactly() {
        assert!(args_have_ros_args(["node", "--ros-args", "-p", "x:=1"]));
        assert!(args_have_ros_args(["--ros-args"]));
        // A prefix match would refuse an argument nano-ros never drops.
        assert!(!args_have_ros_args(["--ros-args-extra"]));
        assert!(!args_have_ros_args(["--ros-arg"]));
        assert!(!args_have_ros_args(["node", "--verbose"]));
        assert!(!args_have_ros_args(core::iter::empty::<&str>()));
    }

    #[test]
    #[should_panic(expected = "--ros-args")]
    fn ros_args_are_refused_loudly() {
        let _ = init_with_args(["--ros-args"]);
    }

    #[test]
    #[should_panic(expected = "--ros-args")]
    fn ros_args_are_refused_loudly_anywhere_in_argv() {
        let _ = init_with_args(["/usr/bin/talker", "--ros-args", "-r", "chatter:=/other"]);
    }

    #[test]
    fn plain_args_pass_through_to_init() {
        // Same answer as `init()` — the arguments are not consulted, and no
        // refusal fires. Both read the same environment, so compare the
        // resolved knobs rather than asserting a particular value.
        let via_args = init_with_args(["/usr/bin/talker", "--verbose", "positional"]);
        let plain = init();
        match (via_args, plain) {
            (Ok(a), Ok(b)) => {
                assert_eq!(a.domain_id, b.domain_id);
                assert_eq!(a.locator, b.locator);
                assert_eq!(a.rmw, b.rmw);
            }
            (Err(a), Err(b)) => assert_eq!(a, b),
            (a, b) => panic!("init_with_args and init disagree: {a:?} vs {b:?}"),
        }
    }
}

/// Pattern 2 — launch-aware init.
///
/// Resolves the launch file via:
///
/// 1. `$NROS_RUNTIME_OVERLAY` — when set, the path points at a JSON sidecar
///    written by `nros launch --emit-runtime-overlay`. (NOT yet consumed;
///    placeholder for the follow-up wave.)
/// 2. `<CARGO_MANIFEST_DIR>/launch/<pkg>.launch.xml` or
///    `<CARGO_MANIFEST_DIR>/launch/system.launch.xml`. (NOT yet parsed;
///    placeholder.)
/// 3. The env vars described in [`init`] — the launcher projects launch
///    params into the child env before `exec()`, so the env path is the
///    de-facto launch overlay today.
///
/// Returns a [`Context`] whose `source = ContextSource::Launch` so callers
/// can introspect whether the run is launch-driven.
#[cfg(feature = "env")]
pub fn init_with_launch_auto() -> Result<Context, InitError> {
    // TODO (Phase 212.L.5 follow-up):
    //   1. If $NROS_RUNTIME_OVERLAY is set, read the JSON sidecar and fold
    //      its params/remaps/env into the Context.
    //   2. Else walk <CARGO_MANIFEST_DIR>/launch/* and parse the XML
    //      in-process (Option B — only if Option A overhead is rejected).
    // For now the env path is the only overlay channel.
    read_env_context(ContextSource::Launch)
}

/// Pattern 2 — explicit-path variant of [`init_with_launch_auto`].
///
/// Verifies the file exists (so misspelled paths fail fast at init time)
/// but does NOT yet parse the XML; the launcher's projected env is the
/// active overlay. See the module-level notes for the follow-up plan.
#[cfg(feature = "env")]
pub fn init_with_launch(path: impl AsRef<Path>) -> Result<Context, InitError> {
    let p = path.as_ref();
    if !p.exists() {
        return Err(InitError::LaunchFileNotFound);
    }
    // TODO (Phase 212.L.5 follow-up): parse the launch XML and fold params
    // / remaps / env into the returned Context. Today we only verify the
    // file exists and fall through to the env overlay path.
    read_env_context(ContextSource::Launch)
}
