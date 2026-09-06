//! Phase 212.L.5 — top-level init API; phase-427 W9 — on every target.
//!
//! [`Context`] is WHERE this image is connected — locator, domain, RMW,
//! session mode, and the source those came from (RFC-0089 "Context and
//! `init`, settled"). One per process or image; a resolved value, not an
//! entity on the graph. A `Node` is a named participant with its own
//! entities, and one image holds several of those on one `Context`
//! (RFC-0047).
//!
//! ## Where the values come from
//!
//! * **Hosted** (feature `env`): the process environment (`ROS_DOMAIN_ID`,
//!   `NROS_LOCATOR`, `NROS_SESSION_MODE`, `NROS_RMW` / `RMW_IMPLEMENTATION`)
//!   through the one hosted resolver, `crate::env::try_resolve_hosted`. A
//!   launcher projects per-node values into that environment before exec.
//! * **Freestanding** (no `env`): there is no process and no environment; the
//!   same values are BAKED at compile time — `option_env!("NROS_LOCATOR")` and
//!   `option_env!("NROS_DOMAIN_ID")`, exported into this crate's build by
//!   `nros_zephyr_build::bake_nros_config()` from Kconfig (`build.rs`), or
//!   present in the process environment of the build for the other RTOS lanes
//!   (the board crates read the same two names the same way). The semantics
//!   survive with the source moved from run time to build time: "the
//!   environment this image was built for or launched in".
//!
//! The `env` feature gates ONLY the process-environment reader and the
//! argv/launch entry points; `Context`, [`ContextSource`], [`InitError`],
//! [`InitOptions`] and [`Context::config`] compile on every target. The floor
//! for `Context` itself is `alloc`: `locator` and `rmw` are owned `String`s
//! so the value outlives whatever transient produced it (an env cache, a
//! parsed launch file, a harness-supplied override).
//!
//! ## The rclrs family
//!
//! [`Context::default_from_env`], [`Context::from_env`] and (hosted only)
//! [`Context::new`] are the `rclrs::Context` constructors under the same
//! names; [`InitOptions`] is rclrs's, with the one option it has. [`init()`] is
//! the C++-symmetric anchor (`rclcpp::init`) and equals `default_from_env()`.
//!
//! Three patterns are supported (per the Phase 212.L canonical pkg shape):
//!
//! 1. **Node pkg** — register via the [`nros::node!`](crate::node!)
//!    macro (Phase 172 W.3); the generated runtime owns the spin loop and
//!    builds its `Context` through [`Context::default_from_env`].
//! 2. **Application pkg + launch-aware** — call [`init_with_launch_auto`] (or
//!    [`init_with_launch`] for an explicit path). The returned [`Context`]
//!    carries launch-resolved fields (domain id, locator, RMW choice). User
//!    code drives its own spin via `Executor::open` +
//!    `Executor::spin_blocking`.
//! 3. **Application pkg + custom spin** — call [`init()`] /
//!    [`Context::default_from_env`] (or [`init_with_args`] / [`Context::new`]
//!    for argv-style entry). Launch file is ignored; env vars +
//!    `ExecutorConfig::from_env()` semantics still apply.
//!
//! To actually open a session, materialise an [`crate::ExecutorConfig`] via
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
// rather than the build. `init_with_launch` is behind `env`, which requires
// `std`, so nothing here is reachable without one.
use std::path::Path;

#[cfg(feature = "alloc")]
use nros_node::DOMAIN_ID_MAX;
#[cfg(feature = "alloc")]
use nros_node::ExecutorConfig;
#[cfg(feature = "alloc")]
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
    /// A launch-derived env var (`ROS_DOMAIN_ID`, etc.) failed to parse —
    /// or, freestanding, the baked `NROS_DOMAIN_ID` is not a decimal integer.
    EnvParseFailed,
    /// A domain id — from the environment, the bake, or an
    /// [`InitOptions::with_domain_id`] override — exceeds
    /// [`crate::DOMAIN_ID_MAX`]. Rejected here rather than
    /// handed to a backend that can only fail later (the #206 rule).
    DomainIdOutOfRange,
}

impl core::fmt::Display for InitError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InitError::LaunchFileNotFound => f.write_str("launch file not found"),
            InitError::LaunchParseFailed => f.write_str("launch file parse failed"),
            InitError::EnvParseFailed => f.write_str("env var parse failed"),
            InitError::DomainIdOutOfRange => f.write_str("domain id out of range"),
        }
    }
}

// `core::error::Error` — `std::error::Error` is a re-export of it since Rust
// 1.81, so this is the same trait, and it needs no feature: phase-427 W9 took
// this module out from behind `env`.
impl core::error::Error for InitError {}

/// Phase 212.L.5 — resolved init context.
///
/// Returned by every `init*` entry point and every `Context::*` constructor.
/// Carries the fields the user needs to construct an [`ExecutorConfig`] and
/// open a session.
///
/// Fields are owned (`String`) so the `Context` can outlive transient parents
/// (env caches, parsed launch files, a harness-supplied locator). That is why
/// the type's feature floor is `alloc`.
#[cfg(feature = "alloc")]
#[derive(Debug, Clone)]
pub struct Context {
    /// ROS 2 domain ID (`ROS_DOMAIN_ID`, default 0; baked `NROS_DOMAIN_ID`
    /// freestanding).
    pub domain_id: u32,
    /// Middleware locator (`NROS_LOCATOR` / legacy `ZENOH_LOCATOR`; baked
    /// `NROS_LOCATOR` freestanding). Empty means ABSENT: `nros` is
    /// RMW-agnostic, so the linked backend substitutes its own default
    /// (issue 0330).
    pub locator: alloc::string::String,
    /// Session mode (`NROS_SESSION_MODE` / legacy `ZENOH_MODE`, default `Client`).
    pub mode: SessionMode,
    /// RMW implementation hint (`RMW_IMPLEMENTATION` /  `NROS_RMW`).
    ///
    /// Empty when neither var is set, and always empty freestanding (an image
    /// links exactly one backend). The runtime uses this to pick a primary
    /// backend when multiple are linked; see `crate::internals::open_session`.
    pub rmw: alloc::string::String,
    /// Source of this context — useful for diagnostics + tests.
    pub source: ContextSource,
}

/// Where the [`Context`] came from. Diagnostics only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContextSource {
    /// Built from env vars by [`init()`] / [`init_with_args`] /
    /// [`Context::new`], or by [`Context::default_from_env`] /
    /// [`Context::from_env`] on a hosted build.
    Env,
    /// Built from a launch file (path supplied to [`init_with_launch`]) or
    /// auto-discovered via [`init_with_launch_auto`]. The launch XML itself
    /// is NOT yet parsed (see module docs); the launcher's projected env
    /// is the source of truth for now.
    Launch,
    /// phase-427 W9 — built from the constants baked into this crate at
    /// compile time (`NROS_LOCATOR`, `NROS_DOMAIN_ID`) by [`Context::baked`],
    /// which is what [`Context::default_from_env`] / [`Context::from_env`]
    /// resolve to on a freestanding build. The entry macros
    /// (`nros::main!`'s Zephyr arm, `nros::zephyr_component_main!`) produce
    /// this one.
    Baked,
}

/// phase-427 W9 — `rclrs::InitOptions`, with the one option it has.
///
/// Consumed by [`Context::from_env`] and [`Context::new`]; `None` for the
/// domain keeps whatever the environment (hosted) or the bake (freestanding)
/// resolved.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct InitOptions {
    domain_id: Option<u32>,
}

impl InitOptions {
    /// Default options: nothing overridden.
    pub const fn new() -> Self {
        Self { domain_id: None }
    }

    /// Override the domain id (`Some`) or keep the resolved one (`None`).
    ///
    /// `usize` because that is rclrs's signature. A value above `u32::MAX`
    /// cannot be a domain id at all; it saturates here and the constructor
    /// that consumes the options then rejects it as
    /// [`InitError::DomainIdOutOfRange`], which is the loud place — a builder
    /// method has no `Result` to carry the refusal in.
    pub const fn with_domain_id(mut self, domain_id: Option<usize>) -> Self {
        self.domain_id = match domain_id {
            Some(d) if d > u32::MAX as usize => Some(u32::MAX),
            Some(d) => Some(d as u32),
            None => None,
        };
        self
    }

    /// The in-place twin of [`with_domain_id`](Self::with_domain_id) — rclrs
    /// has both, and a ported `opts.set_domain_id(Some(7))` compiles as
    /// written. Same saturation.
    pub fn set_domain_id(&mut self, domain_id: Option<usize>) {
        *self = self.with_domain_id(domain_id);
    }

    /// The domain override, if any.
    pub const fn domain_id(&self) -> Option<u32> {
        self.domain_id
    }
}

#[cfg(feature = "alloc")]
/// The range check every constructor applies to a domain id, whatever its
/// source (env, bake, override): the same bound `try_resolve_hosted` enforces
/// ("INCLUDING a baked one"), so a freestanding image cannot boot on a domain
/// a hosted process would have refused.
fn check_domain_id(domain_id: u32) -> Result<u32, InitError> {
    if domain_id > DOMAIN_ID_MAX {
        Err(InitError::DomainIdOutOfRange)
    } else {
        Ok(domain_id)
    }
}

#[cfg(feature = "alloc")]
impl Context {
    /// Materialise an [`ExecutorConfig`] for a node with the given name.
    ///
    /// The returned config borrows from `self`, so callers usually do:
    ///
    /// ```ignore
    /// let ctx = nros::Context::default_from_env()?;
    /// let cfg = ctx.config("talker");
    /// let mut executor = nros::Executor::open(&cfg)?;
    /// ```
    pub fn config<'a>(&'a self, node_name: &'a str) -> ExecutorConfig<'a> {
        ExecutorConfig::new(self.locator.as_str())
            .node_name(node_name)
            .domain_id(self.domain_id)
            .mode(self.mode)
    }

    /// `rclrs::Context::default_from_env()` — the environment, default
    /// options.
    ///
    /// Hosted (feature `env`) this reads the process environment and is
    /// exactly [`init()`]; freestanding it is [`Context::baked`], the build
    /// environment this image was compiled for. RFC-0089 "Context and `init`,
    /// settled": the semantics survive with the source moved from run time to
    /// build time, which is the bounded half of this adoption.
    pub fn default_from_env() -> Result<Context, InitError> {
        #[cfg(feature = "env")]
        {
            init()
        }
        #[cfg(not(feature = "env"))]
        {
            Self::baked()
        }
    }

    /// `rclrs::Context::from_env(options)` — [`default_from_env`](Self::default_from_env)
    /// with the [`InitOptions`] applied on top: a
    /// `Some` domain replaces the resolved one, `None` keeps it.
    pub fn from_env(options: InitOptions) -> Result<Context, InitError> {
        let mut ctx = Self::default_from_env()?;
        if let Some(domain_id) = options.domain_id {
            ctx.domain_id = check_domain_id(domain_id)?;
        }
        Ok(ctx)
    }

    /// `rclrs::Context::new(args, options)` — the argv constructor. HOSTED
    /// ONLY: a freestanding image has no argv, so the name is absent there and
    /// the compiler says so.
    ///
    /// **What it does with the arguments:** the same as [`init_with_args`] —
    /// it does NOT parse them, and it does not silently ignore them either. A
    /// vector carrying `--ros-args` is REFUSED LOUDLY at the call (logged
    /// through `nros_log`, then a panic naming the flag —
    /// [`REFUSE_INIT_ARGS`]); any other vector passes through to
    /// [`Context::from_env`] untouched, because nothing was dropped. Parsing
    /// `--ros-args` is remap resolution (RFC-0020 class 4) and belongs beside
    /// `nros::resolve_name`, not in a constructor. RFC-0089: only the VALUE
    /// carries the defect, so the call is the earliest point it is knowable.
    ///
    /// `S: AsRef<str>` is a superset of rclrs's `Item = String`, so
    /// `Context::new(std::env::args(), InitOptions::new())` compiles as
    /// written.
    #[cfg(feature = "env")]
    pub fn new<I, S>(args: I, options: InitOptions) -> Result<Context, InitError>
    where
        I: IntoIterator<Item = S>,
        S: AsRef<str>,
    {
        refuse_ros_args(args);
        Self::from_env(options)
    }

    /// phase-427 W9 — the freestanding constructor: the constants baked into
    /// this crate at compile time.
    ///
    /// * `NROS_LOCATOR` — the middleware locator. Unset or empty ⇒ empty ⇒
    ///   the linked backend's own default (issue 0330; for zenoh-pico that is
    ///   multicast scouting, which native_sim NSOS cannot satisfy — hence the
    ///   bake).
    /// * `NROS_DOMAIN_ID` — the domain. Unset ⇒ 0; non-numeric ⇒
    ///   [`InitError::EnvParseFailed`]; above `DOMAIN_ID_MAX` ⇒
    ///   [`InitError::DomainIdOutOfRange`].
    ///
    /// Both names reach this crate's build the same way they reach the board
    /// crates: on Zephyr, `build.rs` re-exports `CONFIG_NROS_ZENOH_LOCATOR` /
    /// `CONFIG_NROS_DOMAIN_ID` from `$DOTCONFIG` (`nros_zephyr_build::
    /// bake_nros_config`, the issue-0460 channel); on the other RTOS lanes the
    /// fixture build exports them into the process environment. This is the
    /// ONE `option_env!` site for the pair — the entry macros used to carry
    /// their own copies (and `nros::main!`'s Zephyr arm had lost the domain
    /// half, issue 0161's class), which is what "one source of the baked
    /// shape" replaces.
    ///
    /// Mode is `Client` and `rmw` is empty: an image links exactly one
    /// backend and nothing bakes a session mode today.
    ///
    /// Hosted builds have this too — it answers "what was baked", which a
    /// hosted process rarely wants; [`Context::default_from_env`] is the
    /// constructor that picks the right source for the build.
    pub fn baked() -> Result<Context, InitError> {
        Self::from_baked(option_env!("NROS_LOCATOR"), option_env!("NROS_DOMAIN_ID"))
    }

    /// The parse behind [`Context::baked`], with the two constants injected
    /// so a unit test can exercise every arm without a build environment.
    /// Public because the macros in `nros-macros` expand in another crate;
    /// not API.
    #[doc(hidden)]
    pub fn from_baked(
        baked_locator: Option<&str>,
        baked_domain_id: Option<&str>,
    ) -> Result<Context, InitError> {
        let domain_id = match baked_domain_id {
            Some(d) => check_domain_id(d.trim().parse().map_err(|_| InitError::EnvParseFailed)?)?,
            None => 0,
        };
        Ok(Context {
            domain_id,
            locator: alloc::string::String::from(baked_locator.unwrap_or("")),
            mode: SessionMode::Client,
            rmw: alloc::string::String::new(),
            source: ContextSource::Baked,
        })
    }

    /// Replace the locator with one handed in at run time, when there is one.
    ///
    /// #166 / phase-286 W1 — native_sim test parallelism: the harness launches
    /// an image with `-testargs --nros-locator=<loc>` and a per-test router on
    /// that ephemeral port, and `nros_runtime_locator_override()`
    /// (`nros-platform-zephyr`, argv-backed) hands it back here. `None` or an
    /// empty string keeps the baked value, so on real hardware — where the hook
    /// returns NULL — the bake stands. The entry macros call this after
    /// [`Context::default_from_env`]; a hand-written entry may too.
    pub fn with_locator_override(mut self, locator: Option<&str>) -> Self {
        if let Some(loc) = locator {
            if !loc.is_empty() {
                self.locator = alloc::string::String::from(loc);
            }
        }
        self
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
    let resolved =
        crate::env::try_resolve_hosted(nros_node::BootConfig::default()).map_err(|e| match e {
            nros_node::BootConfigError::DomainIdRange => InitError::DomainIdOutOfRange,
            nros_node::BootConfigError::DomainIdParse => InitError::EnvParseFailed,
        })?;
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

/// Pattern 3 — raw init, launch file ignored. The C++-symmetric anchor:
/// `rclcpp::init()` in the Rust spelling, and EQUAL to
/// [`Context::default_from_env`] on a hosted build (RFC-0089 "Context and
/// `init`, settled" — each language follows its own upstream; rclrs spells
/// this one on the type, rclcpp as a free function, and both are here).
///
/// Reads env vars (`ROS_DOMAIN_ID`, `NROS_LOCATOR`, `NROS_SESSION_MODE`,
/// `NROS_RMW` / `RMW_IMPLEMENTATION`) and returns a [`Context`]. The
/// caller owns the spin loop — typically `Executor::open(&ctx.config(name))`
/// followed by `spin_blocking` or a hand-rolled `spin_once` loop.
#[cfg(feature = "env")]
pub fn init() -> Result<Context, InitError> {
    read_env_context(ContextSource::Env)
}

/// The refusal [`init_with_args`] and [`Context::new`] emit when handed
/// `--ros-args`. The same text `NROS_RCLCPP_REFUSE_INIT_ARGV` carries for the
/// C++ twin (`packages/api/nros-cpp/include/nros/log.hpp`), with the Rust
/// spellings.
#[cfg(feature = "env")]
pub const REFUSE_INIT_ARGS: &str = "nros::init_with_args / nros::Context::new was given --ros-args, which nano-ros cannot \
honour (RFC-0089, phase-417 W3.b). Proceeding would DISCARD it, so `-r chatter:=/other` would \
silently become a wrong-topic bug at runtime -- the 'compiles and differs' the rule \
forbids. Nothing in this process parses --ros-args yet, and honouring them is remap \
resolution -- RFC-0020 violation class 4 -- so the parser belongs beside nros::resolve_name, \
not in this wrapper. Today remaps and parameter overrides come from the LAUNCHER, which \
projects them into the environment before exec. Call nros::init() / \
nros::Context::default_from_env(), or nros::init_with_launch_auto() for the launch-aware \
entry point.";

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

/// The one refusal site, shared by [`init_with_args`] and [`Context::new`]:
/// logged through `nros_log` (never `std::println!`, issue 0589), then a
/// panic naming the flag — the same shape `rclcpp::init(argc, argv)` has in
/// `nros.hpp`.
#[cfg(feature = "env")]
fn refuse_ros_args<I, S>(args: I)
where
    I: IntoIterator<Item = S>,
    S: AsRef<str>,
{
    if args_have_ros_args(args) {
        nros_log::log_error!(nros_log::get_logger("nros"), "{}", REFUSE_INIT_ARGS);
        panic!("{}", REFUSE_INIT_ARGS);
    }
}

/// Pattern 3 — like [`init()`] but accepts a `[--arg=value, ...]`-style argv
/// iterator. The free-function twin of [`Context::new`] with default options.
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
    refuse_ros_args(args);
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

    // phase-427 W9 — the rclrs family on a hosted build.

    fn same_identity(a: &Context, b: &Context) {
        assert_eq!(a.domain_id, b.domain_id);
        assert_eq!(a.locator, b.locator);
        assert_eq!(a.mode, b.mode);
        assert_eq!(a.rmw, b.rmw);
        assert_eq!(a.source, b.source);
    }

    #[test]
    fn default_from_env_is_init() {
        match (Context::default_from_env(), init()) {
            (Ok(a), Ok(b)) => {
                same_identity(&a, &b);
                assert_eq!(a.source, ContextSource::Env);
            }
            (Err(a), Err(b)) => assert_eq!(a, b),
            (a, b) => panic!("default_from_env and init disagree: {a:?} vs {b:?}"),
        }
    }

    #[test]
    fn from_env_honours_the_domain_override() {
        let env = match init() {
            Ok(c) => c,
            Err(e) => panic!("init() failed on the test host: {e:?}"),
        };
        // A value the environment is not carrying, so the override is visible
        // whatever `ROS_DOMAIN_ID` says.
        let wanted = if env.domain_id == 7 { 9 } else { 7 };
        let ctx = Context::from_env(InitOptions::new().with_domain_id(Some(wanted as usize)))
            .expect("from_env with a valid override");
        assert_eq!(ctx.domain_id, wanted);
        // Everything else is untouched.
        assert_eq!(ctx.locator, env.locator);
        assert_eq!(ctx.rmw, env.rmw);
        assert_eq!(ctx.source, ContextSource::Env);
    }

    #[test]
    fn from_env_with_no_override_keeps_the_env_value() {
        match (
            Context::from_env(InitOptions::new().with_domain_id(None)),
            init(),
        ) {
            (Ok(a), Ok(b)) => same_identity(&a, &b),
            (Err(a), Err(b)) => assert_eq!(a, b),
            (a, b) => panic!("from_env(None) and init disagree: {a:?} vs {b:?}"),
        }
    }

    #[test]
    fn from_env_rejects_an_out_of_range_override() {
        if init().is_err() {
            // The environment itself is invalid; the override is never reached.
            return;
        }
        assert_eq!(
            Context::from_env(InitOptions::new().with_domain_id(Some(DOMAIN_ID_MAX as usize + 1)))
                .map(|c| c.domain_id),
            Err(InitError::DomainIdOutOfRange)
        );
        // A value no `u32` can hold saturates in the builder and is refused
        // here, not truncated into a plausible small number.
        assert_eq!(
            Context::from_env(InitOptions::new().with_domain_id(Some(usize::MAX)))
                .map(|c| c.domain_id),
            Err(InitError::DomainIdOutOfRange)
        );
    }

    #[test]
    #[should_panic(expected = "--ros-args")]
    fn context_new_refuses_ros_args() {
        let _ = Context::new(
            ["/usr/bin/talker", "--ros-args", "-r", "chatter:=/other"],
            InitOptions::new(),
        );
    }

    #[test]
    fn context_new_passes_plain_args_through() {
        let opts = InitOptions::new().with_domain_id(Some(3));
        match (
            Context::new(["/usr/bin/talker", "--verbose", "positional"], opts),
            Context::from_env(opts),
        ) {
            (Ok(a), Ok(b)) => {
                same_identity(&a, &b);
                assert_eq!(a.domain_id, 3);
            }
            (Err(a), Err(b)) => assert_eq!(a, b),
            (a, b) => panic!("Context::new and from_env disagree: {a:?} vs {b:?}"),
        }
    }

    #[test]
    fn context_new_accepts_rclrs_argv_shape() {
        // `impl IntoIterator<Item = String>` is what rclrs takes; ours must
        // accept the same call unchanged.
        let args: alloc::vec::Vec<alloc::string::String> =
            alloc::vec![alloc::string::String::from("talker")];
        let _ = Context::new(args, InitOptions::default());
    }
}

/// phase-427 W9 — the freestanding constructor's parse, with the constants
/// injected. Runs on the `alloc` floor (`cargo test -p nros --features alloc
/// --lib`); nothing here reads the environment.
#[cfg(all(test, feature = "alloc"))]
mod baked_tests {
    use super::*;

    #[test]
    fn nothing_baked_is_domain_zero_and_an_absent_locator() {
        let ctx = Context::from_baked(None, None).expect("nothing baked is valid");
        assert_eq!(ctx.domain_id, 0);
        assert_eq!(ctx.locator, "");
        assert_eq!(ctx.mode, SessionMode::Client);
        assert_eq!(ctx.rmw, "");
        assert_eq!(ctx.source, ContextSource::Baked);
    }

    #[test]
    fn baked_values_are_carried() {
        let ctx = Context::from_baked(Some("tcp/127.0.0.1:7456"), Some("42")).expect("valid bake");
        assert_eq!(ctx.locator, "tcp/127.0.0.1:7456");
        assert_eq!(ctx.domain_id, 42);
        assert_eq!(ctx.source, ContextSource::Baked);
        // The config it materialises is the one the macros used to assemble
        // by hand: locator, domain, node name, client mode.
        let cfg = ctx.config("talker");
        assert_eq!(cfg.locator, "tcp/127.0.0.1:7456");
        assert_eq!(cfg.domain_id, 42);
        assert_eq!(cfg.node_name, "talker");
        assert_eq!(cfg.mode, SessionMode::Client);
    }

    #[test]
    fn an_empty_baked_locator_is_absent() {
        let ctx = Context::from_baked(Some(""), None).expect("empty is absent, not an error");
        assert_eq!(ctx.locator, "");
    }

    #[test]
    fn a_non_numeric_baked_domain_is_refused() {
        assert_eq!(
            Context::from_baked(None, Some("seven")).map(|c| c.domain_id),
            Err(InitError::EnvParseFailed)
        );
        assert_eq!(
            Context::from_baked(None, Some("-1")).map(|c| c.domain_id),
            Err(InitError::EnvParseFailed)
        );
    }

    #[test]
    fn an_out_of_range_baked_domain_is_refused() {
        // The bound `try_resolve_hosted` applies, so an image cannot be built
        // onto a domain a hosted process would refuse to boot on.
        assert_eq!(
            Context::from_baked(None, Some("300")).map(|c| c.domain_id),
            Err(InitError::DomainIdOutOfRange)
        );
        assert_eq!(
            Context::from_baked(None, Some("232")).map(|c| c.domain_id),
            Ok(DOMAIN_ID_MAX)
        );
    }

    #[test]
    fn locator_override_applies_only_when_present_and_non_empty() {
        let baked = || Context::from_baked(Some("tcp/10.0.2.2:7447"), Some("1")).unwrap();
        assert_eq!(
            baked()
                .with_locator_override(Some("tcp/127.0.0.1:7456"))
                .locator,
            "tcp/127.0.0.1:7456"
        );
        assert_eq!(
            baked().with_locator_override(None).locator,
            "tcp/10.0.2.2:7447"
        );
        assert_eq!(
            baked().with_locator_override(Some("")).locator,
            "tcp/10.0.2.2:7447"
        );
        // The override touches nothing else.
        let ctx = baked().with_locator_override(Some("tcp/127.0.0.1:7456"));
        assert_eq!(ctx.domain_id, 1);
        assert_eq!(ctx.source, ContextSource::Baked);
    }

    #[test]
    fn baked_reads_this_crates_build_environment() {
        // The values depend on the build; the SHAPE does not. Whatever was
        // baked, the source is `Baked`, and a parse failure would have come
        // from a bake this test cannot control — report it rather than hide it.
        match Context::baked() {
            Ok(ctx) => assert_eq!(ctx.source, ContextSource::Baked),
            Err(e) => panic!(
                "NROS_DOMAIN_ID was baked into this test build with a value the constructor refuses: {e:?}"
            ),
        }
    }

    #[test]
    fn init_options_default_overrides_nothing() {
        assert_eq!(InitOptions::new(), InitOptions::default());
        assert_eq!(InitOptions::new().domain_id(), None);
        assert_eq!(
            InitOptions::new().with_domain_id(Some(5)).domain_id(),
            Some(5)
        );
        assert_eq!(
            InitOptions::new()
                .with_domain_id(Some(5))
                .with_domain_id(None)
                .domain_id(),
            None
        );
        // Saturates rather than truncates: `u32::MAX + 1` must not read as 0.
        assert_eq!(
            InitOptions::new()
                .with_domain_id(Some(u32::MAX as usize + 1))
                .domain_id(),
            Some(u32::MAX)
        );
        // The in-place spelling is the same operation.
        let mut opts = InitOptions::new();
        opts.set_domain_id(Some(9));
        assert_eq!(opts, InitOptions::new().with_domain_id(Some(9)));
        opts.set_domain_id(None);
        assert_eq!(opts, InitOptions::new());
    }

    #[cfg(not(feature = "env"))]
    #[test]
    fn freestanding_default_from_env_is_the_bake() {
        // Without `env` there is no process environment to read; the family
        // resolves to the bake, and the override still lands on top of it.
        let ctx = Context::default_from_env().expect("bake");
        assert_eq!(ctx.source, ContextSource::Baked);
        let ctx = Context::from_env(InitOptions::new().with_domain_id(Some(11))).expect("bake");
        assert_eq!(ctx.domain_id, 11);
        assert_eq!(ctx.source, ContextSource::Baked);
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
/// 3. The env vars described in [`init()`] — the launcher projects launch
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
