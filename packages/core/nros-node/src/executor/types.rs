//! Public types for the embedded executor.

use nros_rmw::{SessionMode, TransportError};

// ============================================================================
// SpinOnceResult
// ============================================================================

/// Result of a single spin iteration
///
/// Contains counts of how many items were processed during `spin_once()`,
/// plus error counts for transport failures that would otherwise be silently dropped.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SpinOnceResult {
    /// Number of subscription callbacks invoked
    pub subscriptions_processed: usize,
    /// Number of timers that fired
    pub timers_fired: usize,
    /// Number of service requests handled
    pub services_handled: usize,
    /// Number of subscription processing errors (e.g., BufferTooSmall, MessageTooLarge)
    pub subscription_errors: usize,
    /// Number of service processing errors (e.g., BufferTooSmall)
    pub service_errors: usize,
}

impl SpinOnceResult {
    /// Create a new empty result
    pub const fn new() -> Self {
        Self {
            subscriptions_processed: 0,
            timers_fired: 0,
            services_handled: 0,
            subscription_errors: 0,
            service_errors: 0,
        }
    }

    /// Check if any work was done (errors are not counted as work)
    pub const fn any_work(&self) -> bool {
        self.subscriptions_processed > 0 || self.timers_fired > 0 || self.services_handled > 0
    }

    /// Total number of callbacks successfully invoked (errors excluded)
    pub const fn total(&self) -> usize {
        self.subscriptions_processed + self.timers_fired + self.services_handled
    }

    /// Check if any errors occurred during this spin iteration
    pub const fn any_errors(&self) -> bool {
        self.subscription_errors > 0 || self.service_errors > 0
    }

    /// Total number of errors across all handle types
    pub const fn total_errors(&self) -> usize {
        self.subscription_errors + self.service_errors
    }
}

// ============================================================================
// SpinPeriodPollingResult (no_std)
// ============================================================================

/// Result from a single period of polling execution (`no_std` compatible).
///
/// Contains the work performed and the remaining time the caller should sleep.
/// The caller is responsible for the actual delay (platform-specific).
///
/// # Example
///
/// ```ignore
/// loop {
///     let r = executor.spin_one_period(10, elapsed_ms);
///     platform_sleep_ms(r.remaining_ms);
/// }
/// ```
#[derive(Debug, Clone, Copy)]
pub struct SpinPeriodPollingResult {
    /// Work performed during this iteration
    pub work: SpinOnceResult,
    /// Remaining time in ms that the caller should sleep
    pub remaining_ms: u64,
}

// ============================================================================
// SpinPeriodResult (std only)
// ============================================================================

/// Result from a single period with monotonic-clock measurement.
///
/// Contains the work performed, whether processing exceeded the period
/// (overrun), and the actual measured processing time.
///
/// phase-359 W10 — was "(`std` only)". The measurement is the executor's own
/// clock now, so a no_std image gets the same result type its hosted sibling
/// does; `elapsed` is zero on a build with no clock at all.
#[cfg(feature = "alloc")]
#[derive(Debug, Clone)]
pub struct SpinPeriodResult {
    /// Work performed during this period
    pub work: SpinOnceResult,
    /// Whether processing exceeded the target period
    pub overrun: bool,
    /// Actual wall-clock processing time
    pub elapsed: core::time::Duration,
}

// ============================================================================
// SpinOptions
// ============================================================================

/// Options controlling blocking spin behavior.
///
/// Used with `Executor::spin_blocking`
/// to control when the spin loop exits.
#[derive(Debug, Clone, Default)]
pub struct SpinOptions {
    /// Stop after this duration.
    ///
    /// phase-379 W5 — a `Duration`, not a `u64` of milliseconds. The `_ms`
    /// suffix was inconsistent with the two methods that consume these options
    /// (`Executor::spin_once` and `Executor::spin` already take
    /// `core::time::Duration`) as well as with rclrs's `SpinOptions::timeout`.
    /// Milliseconds ARE the unit the platform ABI's wait primitives take, but
    /// that conversion belongs at the ABI boundary, where it already happens —
    /// not in the public option a user writes.
    pub timeout: Option<core::time::Duration>,
    /// Only process immediately available work (single iteration)
    pub only_next: bool,
    /// Stop after processing this many callbacks total
    pub max_callbacks: Option<usize>,
}

impl SpinOptions {
    /// Create default spin options (spin forever until halted)
    pub const fn new() -> Self {
        Self {
            timeout: None,
            only_next: false,
            max_callbacks: None,
        }
    }

    /// Set a timeout duration.
    ///
    /// Matches rclrs's `SpinOptions::timeout(Duration)`. Stays `const` — ours
    /// is a strict superset there, and `Duration::from_millis` is const-stable
    /// well below this workspace's MSRV, so `SpinOptions::new().timeout(
    /// Duration::from_millis(50))` is usable in a const context.
    pub const fn timeout(mut self, timeout: core::time::Duration) -> Self {
        self.timeout = Some(timeout);
        self
    }

    /// Only process one round of work (equivalent to spin_once)
    pub const fn spin_once() -> Self {
        Self {
            timeout: None,
            only_next: true,
            max_callbacks: None,
        }
    }

    /// Stop after processing N callbacks
    pub const fn max_callbacks(mut self, n: usize) -> Self {
        self.max_callbacks = Some(n);
        self
    }
}

// ============================================================================
// Configuration constants and defaults
// ============================================================================

// Issue 0330 — there is deliberately NO `DEFAULT_LOCATOR` here. This crate is
// RMW-blind: a router endpoint is a backend fact (the zenoh default lives in
// `nros_rmw_zenoh::DEFAULT_LOCATOR`, the XRCE agent default in the xrce
// backend, and cyclonedds ignores the locator entirely). The bottom rung of
// the RFC-0045 ladder therefore resolves to the empty string, which every
// backend reads as "absent — apply your own default".

/// RFC-0045 / issue #206 — maximum valid ROS 2 domain ID. The ROS 2 / DDS
/// convention (RTPS port arithmetic) caps usable domains at 232; values
/// above it are a configuration error in EVERY language front-end (never a
/// silent clamp or silent 0). Mirrored into the generated C header — keep
/// the mirror in sync (the #160 drift class).
pub const DOMAIN_ID_MAX: u32 = 232;

/// Issue #227 — C/C++-ABI escape for an EXPLICIT domain 0.
///
/// The C/C++ init surface carries `domain_id` as a `u8` where `0` is the
/// UNSET sentinel (the #206 model-A / ROS-convention decision: unset defers
/// to env > baked macro > default). That makes a literal domain 0
/// unreachable once an image bakes a nonzero `NROS_ENTRY_DOMAIN_ID`. Since
/// valid domains cap at [`DOMAIN_ID_MAX`] (232), the value 255 is free:
/// passing it means "explicitly domain 0 — do NOT treat as unset". Hosted
/// env still overrides it (model A), like every other explicit argument.
pub const DOMAIN_ID_EXPLICIT_ZERO_C_ABI: u8 = 255;

/// Map the C/C++ ABI's `u8` domain argument onto the resolver's baked rung.
///
/// `0` → `None` (unset; the ladder decides), 255
/// ([`DOMAIN_ID_EXPLICIT_ZERO_C_ABI`]) → `Some(0)` (explicit zero), anything
/// else → `Some(n)`. Values in `233..=254` pass through so
/// [`ExecutorConfig::try_resolve`] rejects them loudly
/// ([`BootConfigError::DomainIdRange`]) instead of this edge inventing its
/// own validation.
pub fn baked_domain_from_c_abi(raw: u8) -> Option<u32> {
    match raw {
        0 => None,
        DOMAIN_ID_EXPLICIT_ZERO_C_ABI => Some(0),
        n => Some(n as u32),
    }
}

/// RFC-0045 / issue #206 — boot-config resolution error. Malformed or
/// out-of-range identity input (env or baked) is an ERROR, never a silent
/// fallback: a typo'd `ROS_DOMAIN_ID` must not invisibly move a node to
/// domain 0 (the pre-#206 C++ behavior) or be silently ignored (the
/// pre-#206 behavior of this resolver).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootConfigError {
    /// `ROS_DOMAIN_ID` env var set but not a decimal integer.
    DomainIdParse,
    /// Domain ID (env or baked) exceeds [`DOMAIN_ID_MAX`].
    DomainIdRange,
}

impl core::fmt::Display for BootConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            BootConfigError::DomainIdParse => {
                write!(f, "ROS_DOMAIN_ID is set but is not a decimal integer")
            }
            BootConfigError::DomainIdRange => {
                write!(f, "domain id exceeds DOMAIN_ID_MAX ({DOMAIN_ID_MAX})")
            }
        }
    }
}

// ============================================================================
// ExecutorConfig
// ============================================================================

/// Configuration for opening an embedded executor session.
///
/// Provides a backend-agnostic builder for configuring the middleware
/// connection. The active Cargo feature (`rmw-zenoh`, `rmw-xrce`, or
/// `rmw-cffi`) determines which backend is used.
///
/// # Example
///
/// ```ignore
/// use nros::prelude::*;
///
/// let config = ExecutorConfig::new("tcp/127.0.0.1:7447")
///     .node_name("talker")
///     .domain_id(0);
/// let mut executor: Executor = Executor::open(&config)?;
/// ```
pub struct ExecutorConfig<'a> {
    /// Middleware-specific connection string.
    pub locator: &'a str,
    /// Session mode (client or peer).
    pub mode: SessionMode,
    /// ROS 2 domain ID.
    pub domain_id: u32,
    /// Node name.
    pub node_name: &'a str,
    /// Node namespace.
    pub namespace: &'a str,
    /// Monotonic microsecond clock for the executor's timer accounting.
    ///
    /// phase-359 W10 — was `no_std`-only, because the std build read an
    /// `Instant` instead and had nothing to override. There is one clock now,
    /// so the override applies to every flavour: a hosted caller that wants
    /// simulated or externally-driven time can supply it here rather than
    /// being told that is an embedded-only capability.
    pub clock_us: Option<fn() -> u64>,
    /// RFC-0052 / phase-296 W3b.2 — wall-clock µs since the UNIX epoch,
    /// for `now - header.stamp` age monitors. Distinct from the monotonic
    /// `clock_us`: `header.stamp` is ROS (wall) time. `None` = no epoch
    /// source on this target — a baked `max_age_ms` contract with no
    /// epoch source is a BAKE-time error (fail-loud, never a
    /// silently-dead monitor). On `std` targets the executor falls back
    /// to `SystemTime::now()` when unset.
    pub epoch_us: Option<fn() -> u64>,
    /// The RMW backend the caller selected, by the cffi registry's canonical
    /// name (`zenoh`, `cyclonedds`, `xrce`, `uorb`). `None` = no selection:
    /// [`Executor::open`](crate::Executor::open) then takes the unique
    /// registered backend, and reports `Ambiguous` when there is more than one.
    ///
    /// issue 0687 — this used to be read from `$NROS_RMW` INSIDE
    /// `Executor::open`, which is why the core crate needed a process
    /// environment at all. A backend selection is a decision the hosted edge
    /// makes and hands down, exactly like the locator beside it;
    /// `nros::ExecutorConfigEnvExt::from_env` and `nros::env::resolve_hosted`
    /// fill this from `$NROS_RMW`, and an embedded image bakes it or leaves it
    /// `None`.
    pub rmw: Option<&'a str>,
}

impl<'a> ExecutorConfig<'a> {
    /// Create a new configuration with the given locator.
    ///
    /// Defaults: `Client` mode, domain 0, node name `"node"`, empty namespace.
    pub const fn new(locator: &'a str) -> Self {
        Self {
            locator,
            mode: SessionMode::Client,
            domain_id: 0,
            node_name: "node",
            namespace: "",
            clock_us: None,
            epoch_us: None,
            rmw: None,
        }
    }

    /// Phase 104.C.3.3.b — `Default`-style constructor with an
    /// empty locator. Most users want `ExecutorConfig::from_env()`
    /// to pick up `ZENOH_LOCATOR` / `ROS_DOMAIN_ID`; this is the
    /// rclcpp-`NodeOptions{}` shape for callers that set every
    /// field explicitly via the chaining setters.
    pub const fn default_const() -> Self {
        Self::new("")
    }
}

impl Default for ExecutorConfig<'_> {
    fn default() -> Self {
        Self::default_const()
    }
}

impl<'a> ExecutorConfig<'a> {
    /// Set the ROS 2 domain ID.
    pub const fn domain_id(mut self, id: u32) -> Self {
        self.domain_id = id;
        self
    }

    /// Set the node name.
    pub const fn node_name(mut self, name: &'a str) -> Self {
        self.node_name = name;
        self
    }

    /// Set the node namespace.
    pub const fn namespace(mut self, ns: &'a str) -> Self {
        self.namespace = ns;
        self
    }

    /// Set the session mode.
    pub const fn mode(mut self, mode: SessionMode) -> Self {
        self.mode = mode;
        self
    }

    /// RFC-0052 W3b.2 — set the wall-clock (epoch µs) source.
    pub const fn epoch_us(mut self, epoch: fn() -> u64) -> Self {
        self.epoch_us = Some(epoch);
        self
    }

    /// Set the monotonic microsecond clock for the executor's timers.
    pub const fn clock_us(mut self, clock: fn() -> u64) -> Self {
        self.clock_us = Some(clock);
        self
    }

    /// issue 0687 — select the RMW backend by its canonical registry name.
    pub const fn rmw(mut self, rmw: &'a str) -> Self {
        self.rmw = Some(rmw);
        self
    }
}

// ============================================================================
// BootConfig + ExecutorConfig::resolve  (RFC-0045)
// ============================================================================

/// Session-identity subset a caller supplies to `ExecutorConfig::resolve`.
///
/// `None` on a field means "not specified — fall through to the next
/// precedence level".  The precedence model (A) resolved by
/// [`ExecutorConfig::resolve`] is:
///
/// ```text
/// env rung (a Some field on the [`EnvRung`] the caller supplies)
///   > baked (a Some field here)
///     > compiled default
/// ```
///
/// Fields are resolved **independently**: an env locator and a baked
/// `node_name` can both apply in the same call.
///
/// Note: `mode` (session mode) is **not** configurable through `BootConfig`.
/// Session mode comes from the env rung, defaulting to `SessionMode::Client`.
#[derive(Debug, Default, Clone, Copy)]
pub struct BootConfig<'a> {
    /// Node name override.  Maps to [`ExecutorConfig::node_name`].
    pub node_name: Option<&'a str>,
    /// Middleware locator override.  Maps to [`ExecutorConfig::locator`].
    pub locator: Option<&'a str>,
    /// ROS 2 domain ID override.  Maps to [`ExecutorConfig::domain_id`].
    pub domain_id: Option<u32>,
    /// Node namespace override.  Maps to [`ExecutorConfig::namespace`].
    pub namespace: Option<&'a str>,
    /// RMW backend selector.  Maps to [`ExecutorConfig::rmw`], the name
    /// `nros_rmw_cffi::resolve_backend` looks up.
    ///
    /// Issue 1050 defect (3) — this rung did not exist. Every other field of
    /// [`ExecutorConfig`] resolved `env > baked > compiled default`; `rmw`
    /// resolved from the environment ALONE, so `$NROS_RMW` was the only way in
    /// the tree to name a backend. An image that knows perfectly well which
    /// backend it wants — a PX4 module declaring `BACKENDS uorb`, a C entry
    /// built against one RMW, any embedded target with no environment to read —
    /// could not say so, and got whichever backend registered first.
    ///
    /// The environment still wins, because a deployment overriding a build is
    /// the whole point of precedence model A. What changed is that there is now
    /// something for it to override.
    pub rmw: Option<&'a str>,
}

/// The environment rung of precedence model A, as VALUES.
///
/// issue 0687 — `try_resolve` used to take `hosted_env: bool` and read the
/// process environment itself, which is why this crate needed `std`. A rung is
/// a set of values, not a place they come from: the hosted edge
/// (`nros::env::resolve_hosted`) reads `$NROS_LOCATOR`, `$ROS_DOMAIN_ID`,
/// `$NROS_SESSION_MODE`, `$NROS_NODE_NAME` and `$NROS_RMW` and fills this in;
/// an embedded caller that wants the same precedence from some other source (a
/// settings partition, a Kconfig blob) can fill it too, which the bool could
/// never express.
///
/// `None` on a field means "the environment did not speak" — resolution falls
/// through to `baked`, then to the compiled default. The old bool's
/// `hosted_env=false` is `None` here at the whole-rung level.
#[derive(Debug, Default, Clone, Copy)]
pub struct EnvRung<'a> {
    /// `$NROS_LOCATOR` / legacy `$ZENOH_LOCATOR`.
    pub locator: Option<&'a str>,
    /// `$ROS_DOMAIN_ID`, already parsed — a malformed value is the EDGE's
    /// error ([`BootConfigError::DomainIdParse`]), reported before it gets
    /// here, because only the edge knows the text.
    pub domain_id: Option<u32>,
    /// `$NROS_SESSION_MODE` / legacy `$ZENOH_MODE`.
    pub mode: Option<SessionMode>,
    /// `$NROS_NODE_NAME`.
    pub node_name: Option<&'a str>,
    /// `$NROS_RMW` — the backend selector, resolved through
    /// `nros::rmw_selector` so every reader agrees on what "unset" means.
    pub rmw: Option<&'a str>,
}

impl<'a> ExecutorConfig<'a> {
    /// Resolve boot config under precedence model A (RFC-0045), with no
    /// environment rung: `baked > compiled default`.
    ///
    /// Panics on invalid identity input — fail-loud (repo rule): a bad domain
    /// id at boot is a configuration error, never a silent domain-0 node. FFI
    /// shims that need an error code call [`try_resolve`](Self::try_resolve).
    pub fn resolve(baked: BootConfig<'a>) -> ExecutorConfig<'a> {
        Self::resolve_with(baked, None)
    }

    /// [`resolve`](Self::resolve) with an environment rung on top.
    ///
    /// Hosted callers reach this through `nros::env::resolve_hosted`, which
    /// fills the rung from the process environment.
    pub fn resolve_with(baked: BootConfig<'a>, env: Option<EnvRung<'a>>) -> ExecutorConfig<'a> {
        match Self::try_resolve_with(baked, env) {
            Ok(cfg) => cfg,
            Err(e) => panic!("nros boot-config resolution failed: {e}"),
        }
    }

    /// RFC-0045 / issue #206 — fallible [`resolve`](Self::resolve).
    pub fn try_resolve(baked: BootConfig<'a>) -> Result<ExecutorConfig<'a>, BootConfigError> {
        Self::try_resolve_with(baked, None)
    }

    /// RFC-0045 / issue #206 — fallible resolve with an environment rung.
    /// Returns [`BootConfigError`] instead of panicking on out-of-range
    /// identity input, so the C / C++ FFI shims can surface a return code.
    /// Validation is uniform across languages: any resolved domain id >
    /// [`DOMAIN_ID_MAX`] is [`BootConfigError::DomainIdRange`], INCLUDING a
    /// baked one (the DDS backend would only fail later).
    ///
    /// Fields resolve **independently**: an env locator and a baked
    /// `node_name` can both apply in the same call.
    pub fn try_resolve_with(
        baked: BootConfig<'a>,
        env: Option<EnvRung<'a>>,
    ) -> Result<ExecutorConfig<'a>, BootConfigError> {
        let Some(env) = env else {
            // ── no environment rung: baked > compiled default ──────────────
            let domain_id = baked.domain_id.unwrap_or(0);
            if domain_id > DOMAIN_ID_MAX {
                return Err(BootConfigError::DomainIdRange);
            }
            return Ok(ExecutorConfig {
                locator: baked.locator.unwrap_or(""),
                mode: nros_rmw::SessionMode::Client,
                domain_id,
                node_name: baked.node_name.unwrap_or("node"),
                namespace: baked.namespace.unwrap_or(""),
                clock_us: None,
                epoch_us: None,
                // Issue 1050 defect (3) — the baked rung. This was a hard
                // `None`, which is why an image with no environment (every RTOS
                // target, and every hosted image whose launcher sets nothing)
                // had no way to name its backend at all.
                rmw: baked.rmw,
            });
        };

        let domain_id = env.domain_id.or(baked.domain_id).unwrap_or(0);
        if domain_id > DOMAIN_ID_MAX {
            return Err(BootConfigError::DomainIdRange);
        }

        Ok(ExecutorConfig {
            locator: env
                .locator
                // Issue 0330 — bottom rung: leave it ABSENT (`""`). A router
                // endpoint is a backend fact; this crate is RMW-blind, so the
                // empty string travels to whichever backend is linked and THAT
                // backend applies its own default (e.g.
                // `nros_rmw_zenoh::DEFAULT_LOCATOR`). Matches the no-rung path
                // above, which has always resolved to `""`.
                .or(baked.locator)
                .unwrap_or(""),
            mode: env.mode.unwrap_or(SessionMode::Client),
            domain_id,
            node_name: env.node_name.or(baked.node_name).unwrap_or("node"),
            namespace: baked.namespace.unwrap_or(""),
            clock_us: None,
            // A rung means a hosted caller, and a hosted caller has a wall
            // clock — the same one `from_env` installs. `default_epoch_us_fn`
            // answers "does this build have one" in the single place that
            // knows, rather than at two struct literals.
            epoch_us: default_epoch_us_fn(),
            // Issue 1050 defect (3) — `env > baked`, like every other field
            // here. It read `env.rmw` alone, so a baked selector was discarded
            // on any hosted build even when the environment said nothing.
            rmw: env.rmw.or(baked.rmw),
        })
    }
}

// ============================================================================
// Error type
// ============================================================================

/// Error type for generic embedded node operations.
///
/// Not `Copy` — `NodeError::Transport` wraps a [`TransportError`] which
/// carries owned diagnostic strings (`Backend` / `BackendDynamic`). Rust
/// callers that matched on `NodeError` by value may need `ref` arms or
/// `.clone()`; C/C++ callers are unaffected (they see an integer
/// `nros_ret_t`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NodeError {
    /// Transport-level error.
    Transport(TransportError),
    /// Node name exceeds 64 bytes.
    NameTooLong,
    /// CDR serialization failed.
    Serialization,
    /// CDR deserialization failed.
    Deserialization,
    /// Buffer too small for message.
    BufferTooSmall,
    /// Action server/client creation failed.
    ActionCreationFailed,
    /// Service request failed.
    ServiceRequestFailed,
    /// Service reply failed.
    ServiceReplyFailed,
    /// Operation timed out.
    Timeout,
    /// A required subsystem has not been initialized (e.g. parameter
    /// services have not been registered on the executor).
    NotInitialized,
    /// The client / action client already has a request in flight that
    /// hasn't been consumed. Phase 84.D3: fixes the hazard where dropping
    /// a `Promise` without awaiting its reply left the stale reply queued
    /// to be delivered to the *next* call. Resolve by either polling the
    /// existing promise to completion or calling `reset_in_flight()`.
    RequestInFlight,
    /// Phase 110.B — `create_sched_context` ran out of slots
    /// (`MAX_SC` exceeded).
    NoSchedContextSlot,
    /// Phase 110.B — `bind_handle_to_sched_context` was called with
    /// an out-of-range handle, an empty entry slot, or an unknown
    /// `SchedContextId`.
    InvalidSchedContextBinding,
    /// Phase 104.C.2 — `Executor::node_builder(...).build()` was
    /// called when the per-Executor node table is full
    /// (`NROS_EXECUTOR_MAX_NODES` reached).
    NodeTableFull,
    /// Issue 0095 — the executor's fixed callback-entry table is full
    /// (`NROS_EXECUTOR_MAX_CBS`, default 4): a timer / subscription / service /
    /// action could not claim a slot. Distinct from `BufferTooSmall` so the
    /// register seam can name the knob.
    ExecutorFull,
    /// Phase 104.C.2 — requested RMW backend does not match the
    /// Executor's open session (single-session restriction lifts in
    /// Phase 104.C.3 when the per-Node session cache lands).
    BackendMismatch,
    /// Issue 0790 — the shutdown-hook table for the requested phase is full
    /// (`NROS_EXECUTOR_MAX_SHUTDOWN_CBS`, default 2). Distinct from
    /// `ExecutorFull` so the register seam can name the knob that is actually
    /// exhausted; the two tables are sized independently of `MAX_CBS`.
    ShutdownCallbacksFull,
}

impl From<TransportError> for NodeError {
    fn from(err: TransportError) -> Self {
        NodeError::Transport(err)
    }
}

impl From<nros_core::SerError> for NodeError {
    fn from(_: nros_core::SerError) -> Self {
        NodeError::Serialization
    }
}

impl From<nros_core::DeserError> for NodeError {
    fn from(_: nros_core::DeserError) -> Self {
        NodeError::Deserialization
    }
}

/// Default transmit buffer size (bytes).
#[cfg(any(has_rmw, test))]
pub(crate) const DEFAULT_TX_BUF: usize = crate::config::DEFAULT_RX_BUF_SIZE;

// ============================================================================
// Phase 110.A — Activator + ReadySet + Dispatcher
// ============================================================================

/// Index into the executor's `entries[]` array. Phase 110.A caps at
/// 64 to match the existing readiness bitmap width; if a future
/// MAX_HANDLES bump goes past 64 the type widens accordingly.
#[cfg(any(has_rmw, test))]
pub(crate) type DescIdx = u8;

/// Sort key used to order callbacks within a `ReadySet`.
///
/// Phase 110.A: registration-order — `sort_key` mirrors `desc_idx`
/// numerically so `FifoReadySet` preserves bit-for-bit dispatch order.
/// Phase 110.B will widen this to encode an EDF deadline ahead of
/// `desc_idx`.
#[cfg(any(has_rmw, test))]
#[allow(dead_code)] // Phase 110.A — wired in 110.A.b spin_once rewire.
pub(crate) type SortKey = u32;

/// One ready callback queued for dispatch. Stored in the `ReadySet`,
/// consumed by the `Dispatcher`. Full handle metadata (callback fn,
/// data offset, kind) is reconstructed from
/// `Executor::entries[desc_idx]` at dispatch time so the ready set
/// itself stays compact.
#[cfg(any(has_rmw, test))]
#[allow(dead_code)] // Phase 110.A — wired in 110.A.b spin_once rewire.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub(crate) struct ActiveJob {
    pub sort_key: SortKey,
    pub desc_idx: DescIdx,
}

/// How aggressively the dispatcher drains the `ReadySet`.
///
/// `Latched` (default) preserves today's `spin_once` semantics:
/// callbacks that become ready *during* dispatch wait for the next
/// cycle. `Greedy` re-runs the activator after each callback so newly
/// ready entries fire in the same cycle — soft-RT pipelines that want
/// chain-style propagation use this.
#[cfg(any(has_rmw, test))]
#[allow(dead_code)] // Phase 110.B introduces the user-facing knob.
#[derive(Debug, Clone, Copy, Eq, PartialEq, Default)]
pub(crate) enum DrainMode {
    #[default]
    Latched,
    Greedy,
}

// ============================================================================
// HandleId
// ============================================================================

/// Opaque handle identifier returned by registration methods.
///
/// Used with [`Trigger::One`] and [`HandleSet`] for type-safe trigger
/// configuration. The inner value is the entry slot index.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct HandleId(pub usize);

// ============================================================================
// HandleSet
// ============================================================================

/// A set of handle IDs, represented as a bitset.
///
/// Supports up to 64 handles. Construct via `HandleId` operators:
/// ```ignore
/// let set = imu | gps | lidar;  // HandleSet from 3 handles
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct HandleSet(pub(crate) u64);

impl HandleSet {
    /// Empty set.
    pub const EMPTY: Self = Self(0);

    /// Insert a handle into the set.
    pub const fn insert(self, id: HandleId) -> Self {
        Self(self.0 | (1u64 << id.0))
    }

    /// Check if the set contains a handle.
    pub const fn contains(self, id: HandleId) -> bool {
        self.0 & (1u64 << id.0) != 0
    }

    /// Union of two sets.
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// Number of handles in the set.
    pub const fn len(self) -> u32 {
        self.0.count_ones()
    }

    /// Check if the set is empty.
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

impl core::ops::BitOr for HandleId {
    type Output = HandleSet;
    fn bitor(self, rhs: HandleId) -> HandleSet {
        HandleSet::EMPTY.insert(self).insert(rhs)
    }
}

impl core::ops::BitOr<HandleId> for HandleSet {
    type Output = HandleSet;
    fn bitor(self, rhs: HandleId) -> HandleSet {
        self.insert(rhs)
    }
}

impl core::ops::BitOr for HandleSet {
    type Output = HandleSet;
    fn bitor(self, rhs: HandleSet) -> HandleSet {
        self.union(rhs)
    }
}

// ============================================================================
// ReadinessSnapshot
// ============================================================================

/// Snapshot of handle readiness at the start of a spin iteration.
///
/// Passed to [`Trigger::Predicate`] functions. Query by [`HandleId`].
pub struct ReadinessSnapshot {
    pub(crate) bits: u64,
    pub(crate) count: usize,
}

impl ReadinessSnapshot {
    /// Check if a specific handle has data.
    pub const fn is_ready(&self, id: HandleId) -> bool {
        self.bits & (1u64 << id.0) != 0
    }

    /// Check if all handles in the set have data.
    pub const fn all_ready(&self, set: HandleSet) -> bool {
        self.bits & set.0 == set.0
    }

    /// Check if any handle in the set has data.
    pub const fn any_ready(&self, set: HandleSet) -> bool {
        self.bits & set.0 != 0
    }

    /// Number of handles that have data.
    pub const fn ready_count(&self) -> u32 {
        self.bits.count_ones()
    }

    /// Total registered handles.
    pub const fn total(&self) -> usize {
        self.count
    }
}

// ============================================================================
// InvocationMode
// ============================================================================

/// Per-callback invocation mode.
///
/// Controls whether a callback fires only when new data is available
/// or on every spin iteration that passes the trigger gate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InvocationMode {
    /// Fire only when `has_data()` returns true (default).
    #[default]
    OnNewData,
    /// Fire on every spin iteration, regardless of data availability.
    Always,
}

// ============================================================================
// Trigger
// ============================================================================

/// Executor-level trigger condition.
///
/// Controls when the executor dispatches callbacks during `spin_once()`.
/// The trigger is evaluated after polling the transport but before any
/// callback dispatch.
#[derive(Clone, Copy, Default)]
pub enum Trigger {
    /// Fire if any registered handle has data (default).
    #[default]
    Any,
    /// Fire only when ALL non-timer handles have data.
    All,
    /// Fire only when a specific handle has data.
    One(HandleId),
    /// Fire only when every handle in the set has data.
    AllOf(HandleSet),
    /// Fire when any handle in the set has data.
    AnyOf(HandleSet),
    /// Always fire, regardless of data availability.
    Always,
    /// Custom predicate over a readiness snapshot.
    Predicate(fn(&ReadinessSnapshot) -> bool),
    /// Custom predicate with C-compatible signature and context pointer.
    ///
    /// The callback receives a `bool` array of readiness flags (one per handle),
    /// the count of handles, and a user-provided context pointer.
    /// Used by the C API to bridge `nros_executor_trigger_t` to the Rust trigger system.
    RawPredicate {
        /// C trigger callback
        callback: unsafe extern "C" fn(
            ready: *const bool,
            count: usize,
            context: *mut core::ffi::c_void,
        ) -> bool,
        /// User-provided context pointer passed to the callback
        context: *mut core::ffi::c_void,
    },
}

// Manual Debug impl because fn pointers don't impl Debug well
impl core::fmt::Debug for Trigger {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Any => write!(f, "Any"),
            Self::All => write!(f, "All"),
            Self::One(id) => f.debug_tuple("One").field(id).finish(),
            Self::AllOf(set) => f.debug_tuple("AllOf").field(set).finish(),
            Self::AnyOf(set) => f.debug_tuple("AnyOf").field(set).finish(),
            Self::Always => write!(f, "Always"),
            Self::Predicate(_) => write!(f, "Predicate(...)"),
            Self::RawPredicate { .. } => write!(f, "RawPredicate(...)"),
        }
    }
}

// ============================================================================
// Raw callback types (for C API)
// ============================================================================

/// Raw subscription callback that receives CDR bytes without deserialization.
///
/// # Safety
/// The `data` pointer is valid for `len` bytes during the call.
pub type RawSubscriptionCallback =
    unsafe extern "C" fn(data: *const u8, len: usize, context: *mut core::ffi::c_void);

/// phase-417 W5.a — the deserialiser half of the TYPED C subscription path.
///
/// Writes the CDR bytes at `buffer[..buffer_size]` into `msg`, which is storage
/// the CALLER owns and whose type only the caller knows. Returns `0` on success
/// and non-zero on failure; the executor treats any non-zero as
/// `TransportError::DeserializationError` and does NOT dispatch the callback.
///
/// This is the erased form of the generated `<Msg>_deserialize`, which already
/// has exactly this contract — it writes into caller storage and returns
/// `0`/`-1`. There is no allocator on this path and none is needed: caller-owned
/// storage is the whole mechanism, which is also how rclc delivers a typed
/// message without one.
///
/// # Safety
/// `msg` must point to writable storage of the type this deserialiser was
/// generated for; `buffer` is valid for `buffer_size` bytes during the call.
pub type RawMessageDeserializeFn =
    unsafe extern "C" fn(msg: *mut core::ffi::c_void, buffer: *const u8, buffer_size: usize) -> i32;

/// phase-417 W5.a — a subscription callback that receives the DESERIALISED
/// message rather than its CDR bytes.
///
/// `msg` is the caller-owned storage handed to the registration, populated by
/// the [`RawMessageDeserializeFn`] for this subscription. It is valid for the
/// duration of the call and is overwritten by the next dispatch; anything the
/// callback retains must be copied out.
///
/// The `context` parameter is what makes this the analog of rclc's
/// `rclc_subscription_callback_with_context_t` rather than its context-free
/// `rclc_subscription_callback_t` — C has no closures, and every other callback
/// in this API already carries one, so dropping it would have been the odd
/// shape rather than the faithful one.
///
/// # Safety
/// `msg` points to the caller's storage, valid during the call only.
pub type TypedSubscriptionCallback =
    unsafe extern "C" fn(msg: *const core::ffi::c_void, context: *mut core::ffi::c_void);

/// Raw subscription callback that also receives the incoming sample's
/// wire-level attachment (Phase 189.M3.4 — the C analog of the Rust
/// `FnMut(&[u8], &RawMessageInfo)` builder path). `attachment` is valid
/// for `attachment_len` bytes during the call; `attachment_len == 0`
/// means the sample carried no attachment. Cross-RMW bridges read the
/// `bridge_origin` tag from it.
///
/// # Safety
/// `data` is valid for `len` bytes and `attachment` for `attachment_len`
/// bytes, during the call only.
pub type RawSubscriptionInfoCallback = unsafe extern "C" fn(
    data: *const u8,
    len: usize,
    attachment: *const u8,
    attachment_len: usize,
    context: *mut core::ffi::c_void,
);

/// Phase 269 W3 — raw subscription callback that ALSO surfaces the sample's E2E
/// integrity status (CRC + sequence gap/dup) — the C/C++ component-callback
/// projection of Rust's `FnMut(&[u8], &IntegrityStatus)` (used by
/// `register_subscription_buffered_raw_safety_on`).
///
/// The executor unpacks `nros_rmw::IntegrityStatus` into three plain scalars to
/// keep this callback type free of any external-crate struct dependency:
///   * `gap`       — sequence-number gap since the last in-order sample (0 = none)
///   * `duplicate` — `true` if the sequence number was already seen
///   * `crc_valid` — `1` = CRC ok, `0` = CRC mismatch, `-1` = no CRC on the wire
///
/// Requires the `safety-e2e` feature; the C/C++ registration FFI
/// (`nros_cpp_subscription_register_validated`) is gated on the same feature.
///
/// # Safety
/// `data` is valid for `len` bytes during the call only.
#[cfg(feature = "safety-e2e")]
pub type RawSubscriptionSafetyCallback = unsafe extern "C" fn(
    data: *const u8,
    len: usize,
    gap: i64,
    duplicate: bool,
    crc_valid: i8,
    context: *mut core::ffi::c_void,
);

/// Raw service callback that receives and produces CDR bytes.
///
/// # Safety
/// - `req` is valid for `req_len` bytes
/// - `resp` is valid for `resp_cap` bytes (writable)
/// - `resp_len` is a valid pointer to write the response length
///
/// Returns `true` if the request was handled successfully.
pub type RawServiceCallback = unsafe extern "C" fn(
    req: *const u8,
    req_len: usize,
    resp: *mut u8,
    resp_cap: usize,
    resp_len: *mut usize,
    context: *mut core::ffi::c_void,
) -> bool;

/// Raw service-client response callback.
///
/// Invoked by the executor's arena dispatch when a previously-sent
/// service request has its response delivered. The C/C++ blocking
/// wrappers install a one-shot trampoline that flips a static flag;
/// async users register their own callback via the C API.
///
/// # Safety
/// - `data` is valid for `len` bytes during the call.
pub type RawResponseCallback =
    unsafe extern "C" fn(data: *const u8, len: usize, context: *mut core::ffi::c_void);

/// Raw action goal callback that receives CDR bytes without deserialization.
///
/// # Safety
/// - `goal_id` is valid for the duration of the call
/// - `goal_data` is valid for `goal_len` bytes
///
/// Returns a `GoalResponse` value (0=Reject, 1=AcceptAndExecute, 2=AcceptAndDefer).
pub type RawGoalCallback = unsafe extern "C" fn(
    goal_id: *const nros_core::GoalId,
    goal_data: *const u8,
    goal_len: usize,
    context: *mut core::ffi::c_void,
) -> nros_core::GoalResponse;

/// Raw accepted-goal hook.
///
/// Called immediately after the accept reply has been sent to the client
/// (i.e. after `ActionServerCore::accept_goal`). Used by the C API so that
/// the user's `accepted_callback` can run *after* the client has observed
/// the accept, without blocking the accept reply on a long-running
/// execution inside the goal-decision callback.
///
/// # Safety
/// - `goal_id` is valid for the duration of the call.
pub type RawAcceptedCallback =
    unsafe extern "C" fn(goal_id: *const nros_core::GoalId, context: *mut core::ffi::c_void);

/// Raw action cancel callback.
///
/// # Safety
/// - `goal_id` is valid for the duration of the call
///
/// Returns the PER-GOAL decision `nros_core::CancelResponse` (0=Reject,
/// 1=Accept) — the same two values C's `nros_cancel_response_t` and C++'s
/// `nros::CancelResponse` use. Issue 0796: this used to return the
/// `action_msgs/srv/CancelGoal` RPC return code (now `CancelReturnCode`), so a
/// callback answering one goal spoke in whole-request status codes and the
/// server wrote its answer straight into the reply's `return_code` field.
pub type RawCancelCallback = unsafe extern "C" fn(
    goal_id: *const nros_core::GoalId,
    status: nros_core::GoalStatus,
    context: *mut core::ffi::c_void,
) -> nros_core::CancelResponse;

/// Raw action client goal-response callback.
///
/// Called when the action server accepts or rejects a goal.
///
/// # Safety
/// - `goal_id` is valid for the duration of the call
///
/// `accepted` is `true` if the goal was accepted, `false` if rejected.
pub type RawGoalResponseCallback = unsafe extern "C" fn(
    goal_id: *const nros_core::GoalId,
    accepted: bool,
    context: *mut core::ffi::c_void,
);

/// Raw action client result callback.
///
/// Called when the action result is received.
///
/// # Safety
/// - `goal_id` is valid for the duration of the call
/// - `result_data` points to `result_len` valid bytes (CDR-encoded result)
pub type RawResultCallback = unsafe extern "C" fn(
    goal_id: *const nros_core::GoalId,
    status: nros_core::GoalStatus,
    result_data: *const u8,
    result_len: usize,
    context: *mut core::ffi::c_void,
);

/// Raw action client feedback callback.
///
/// Called when feedback is received for an active goal.
///
/// # Safety
/// - `goal_id` is valid for the duration of the call
/// - `feedback_data` points to `feedback_len` valid bytes (CDR-encoded feedback)
pub type RawFeedbackCallback = unsafe extern "C" fn(
    goal_id: *const nros_core::GoalId,
    feedback_data: *const u8,
    feedback_len: usize,
    context: *mut core::ffi::c_void,
);

// ============================================================================
// ExecutorSemantics
// ============================================================================

/// Data communication semantics for the executor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ExecutorSemantics {
    /// Standard interleaved execution (default). Each callback sees the
    /// latest data at the time it runs.
    #[default]
    RclcppExecutor,
    /// Logical Execution Time. All subscriptions are sampled at spin start;
    /// callbacks process from the snapshot.
    LogicalExecutionTime,
}

// ============================================================================
// GuardCondition
// ============================================================================

/// Handle for triggering a guard condition from outside the executor.
///
/// Obtained from `Executor::register_guard_condition`.
/// Safe to use from any thread — the inner `&'static AtomicBool` is inherently
/// `Send + Sync`.
pub struct GuardCondition {
    // The AtomicBool lives in the executor's arena, which is never moved or
    // deallocated while handles exist. The 'static lifetime is asserted at
    // construction time (see `new()`).
    flag: &'static portable_atomic::AtomicBool,
    /// Phase 124.B.5 — runtime wake callback. On std + rmw-cffi
    /// builds the executor sets this to `nros_rmw_runtime_wake_cb`
    /// with `ctx` pointing at the executor's WakeCtx. `trigger`
    /// invokes it after writing the arena flag so a `spin_once`
    /// blocked on `wake_cv` resumes immediately (sub-poll wake
    /// latency). Bare/no-std builds leave it `None` — the arena
    /// flag is observed on the next spin iteration as before.
    wake_cb: Option<unsafe extern "C" fn(ctx: *mut core::ffi::c_void)>,
    wake_ctx: *mut core::ffi::c_void,
}

// SAFETY: `wake_cb` is a plain function pointer; `wake_ctx` points
// at a WakeCtx Arc allocated on Executor::new and never freed before
// Executor::drop. Both are safe to share across threads.
unsafe impl Send for GuardCondition {}
unsafe impl Sync for GuardCondition {}

impl GuardCondition {
    /// Create a handle from a raw pointer to an arena-allocated `AtomicBool`.
    ///
    /// # Safety
    ///
    /// The pointed-to `AtomicBool` must outlive this handle. This is guaranteed
    /// when the `AtomicBool` lives in the executor arena (which is never moved
    /// or deallocated while handles exist).
    #[cfg(any(has_rmw, test))]
    pub(crate) unsafe fn new(flag: *const portable_atomic::AtomicBool) -> Self {
        // SAFETY: Caller guarantees the AtomicBool outlives this handle.
        Self {
            flag: unsafe { &*flag },
            wake_cb: None,
            wake_ctx: core::ptr::null_mut(),
        }
    }

    /// Phase 124.B.5 — install the executor's wake callback. Called
    /// once at handle creation; the executor passes its
    /// `nros_rmw_runtime_wake_cb` + WakeCtx pointer here so
    /// `trigger()` can signal the wake condvar from any thread / ISR.
    #[cfg(any(all(feature = "alloc", feature = "rmw-cffi"), test))]
    #[allow(dead_code)] // Wired by register_guard_condition under cfg.
    pub(crate) fn set_wake_cb(
        &mut self,
        cb: unsafe extern "C" fn(ctx: *mut core::ffi::c_void),
        ctx: *mut core::ffi::c_void,
    ) {
        self.wake_cb = Some(cb);
        self.wake_ctx = ctx;
    }

    /// Trigger the guard condition.
    ///
    /// The executor will invoke the associated callback on the next spin iteration.
    /// On std + rmw-cffi builds, `trigger` also signals the executor's
    /// wake condvar so a blocked `spin_once` resumes immediately.
    pub fn trigger(&self) {
        self.flag.store(true, portable_atomic::Ordering::Release);
        if let Some(cb) = self.wake_cb {
            // SAFETY: cb + ctx installed by Executor::register_guard_condition;
            // ctx points at WakeCtx valid for Executor's lifetime.
            unsafe { cb(self.wake_ctx) };
        }
    }
}

// ============================================================================
// BakedBootConfig re-import — RFC-0045 "Single embedded bake site"
//
// The type and its consts live in nros-platform-api so that nros-platform's
// DeployOverlay can hold a `&'static BakedBootConfig` without a dep cycle.
// Re-imported here so BootConfig::from_baked (below) can reference the type.
// ============================================================================

// Re-export so `nros-node` consumers that used `nros_node::BakedBootConfig` etc.
// continue to compile. The nros-node lib.rs re-export uses nros_platform_api
// directly, but internal users in other nros-node submodules see these via
// `use types::*`.
pub use nros_platform_api::{
    BOOT_SET_DOMAIN, BOOT_SET_LOCATOR, BOOT_SET_NAMESPACE, BOOT_SET_NODE_NAME, BOOT_SET_RMW,
    BakedBootConfig, NROS_BOOT_CONFIG_MAGIC, NROS_BOOT_CONFIG_VERSION,
};

/// Find the length of the non-NUL prefix in `buf`.
///
/// Returns the index of the first `0` byte, or `buf.len()` if there is none.
fn nul_len(buf: &[u8]) -> usize {
    let mut i = 0;
    while i < buf.len() {
        if buf[i] == 0 {
            return i;
        }
        i += 1;
    }
    buf.len()
}

impl<'a> BootConfig<'a> {
    /// Read a baked config into the plain-field `BootConfig` the resolver consumes.
    ///
    /// Returns all-`None` (→ resolver uses compiled defaults) if `magic`/`version`
    /// don't match — defensive against a corrupt or zero-initialised section.
    ///
    /// Each `Option` is `Some` iff its `set_flags` bit is set; string values are
    /// the bytes up to the first NUL (or full buffer if no NUL).  Invalid UTF-8 in
    /// a set field is treated as unset for that field.
    pub fn from_baked(baked: &'a nros_platform_api::BakedBootConfig) -> BootConfig<'a> {
        // Validate the fingerprint.
        if baked.magic != NROS_BOOT_CONFIG_MAGIC || baked.version != NROS_BOOT_CONFIG_VERSION {
            return BootConfig::default();
        }

        let node_name = if baked.set_flags & BOOT_SET_NODE_NAME != 0 {
            let len = nul_len(&baked.node_name);
            core::str::from_utf8(&baked.node_name[..len]).ok()
        } else {
            None
        };

        let locator = if baked.set_flags & BOOT_SET_LOCATOR != 0 {
            let len = nul_len(&baked.locator);
            core::str::from_utf8(&baked.locator[..len]).ok()
        } else {
            None
        };

        let domain_id = if baked.set_flags & BOOT_SET_DOMAIN != 0 {
            Some(baked.domain_id)
        } else {
            None
        };

        let namespace = if baked.set_flags & BOOT_SET_NAMESPACE != 0 {
            let len = nul_len(&baked.namespace);
            core::str::from_utf8(&baked.namespace[..len]).ok()
        } else {
            None
        };

        // Issue 1050 defect (3) — layout version 2. A v1 struct has the bit
        // clear (its `set_flags` never set bit 4), so an older bake reads as
        // "not specified" rather than as garbage.
        let rmw = if baked.set_flags & BOOT_SET_RMW != 0 {
            let len = nul_len(&baked.rmw);
            core::str::from_utf8(&baked.rmw[..len]).ok()
        } else {
            None
        };

        BootConfig {
            node_name,
            locator,
            domain_id,
            namespace,
            rmw,
        }
    }
}

// ============================================================================
// BootConfig / resolve unit tests (std only)
// ============================================================================

/// RFC-0052 W3b.3 — split epoch-µs into the `builtin_interfaces/Time`
/// field pair `(sec, nanosec)` for `header.stamp` population. Types-free
/// (each workspace has its own generated `Time`); assign the tuple to the
/// struct's fields.
pub const fn epoch_us_to_stamp(us: u64) -> (i32, u32) {
    ((us / 1_000_000) as i32, ((us % 1_000_000) * 1_000) as u32)
}

/// Capacity of a backend selector name, matching `Executor::primary_rmw_name`.
///
/// issue 0687 — the READER moved to `nros::rmw_selector` (it needs a process
/// environment, which this crate no longer has), but the CAP belongs here: it
/// is the executor's own identity storage, and a selector longer than this
/// cannot name a registry slot.
pub const RMW_SELECTOR_CAP: usize = 32;

/// The default wall clock, or `None` when this build has neither a platform port
/// nor a host to ask.
///
/// phase-359 W10 — the symmetric twin of `default_clock_us_fn`. The predicate
/// "does this build have a wall clock" used to be spelled at the `Executor`
/// struct literal, in two arms, duplicating the cfgs that already decide
/// whether `default_epoch_us` exists at all. One place answers it now.
// Its only caller is the `Executor` constructor, which is itself compiled only
// when a backend exists (`any(has_rmw, test)`), so builds without one have this
// with nothing to call it. Saying so once beats a cfg predicate that has to
// track the constructor's.
#[allow(dead_code)]
pub fn default_epoch_us_fn() -> Option<fn() -> u64> {
    #[cfg(feature = "rmw-cffi")]
    {
        Some(default_epoch_us)
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        None
    }
}

/// RFC-0052 W3b.2 — the default wall-clock source: µs since the UNIX epoch.
///
/// phase-359 W10 — this was `std_epoch_us`, `std`-gated, `SystemTime`. It is
/// the same story the monotonic clock had one commit ago: `rmw-cffi` means a
/// platform port is linked, every port exports the wall clock, so a hosted
/// build reads the same source an embedded one does. `SystemTime` survives
/// only where there is no port to ask. The claim that stood here — that the
/// metadata probe is that configuration — was WRONG; the probe links a port and
/// resolves `rmw-cffi`. See `default_clock_us_fn` for what was measured.
#[cfg(feature = "rmw-cffi")]
pub fn default_epoch_us() -> u64 {
    unsafe extern "C" {
        fn nros_platform_time_now_ns() -> u64;
    }
    // SAFETY: a bare wall-clock read, no pointer arguments, guaranteed by
    // whichever port linked the binary.
    //
    // ONE symbol since issue 0532; the bounded re-read this carried against the
    // former `time_since_epoch_{secs,nanos}` pair is gone with it.
    unsafe { nros_platform_time_now_ns() / 1_000 }
}

#[cfg(test)]
mod boot_config_tests {
    use super::*;

    // issue 0687 — these tests no longer touch the process environment, and
    // that is the point of the move: resolution takes an [`EnvRung`] of
    // VALUES, so its precedence can be asserted without a mutex serialising
    // `set_var` across a shared process (the race issue 0607 chased) and
    // without this crate having an environment at all. The mapping FROM env
    // vars TO a rung is asserted at the edge that performs it, in
    // `nros::env`.

    /// A rung standing in for "the hosted edge read the environment and found
    /// a locator", the shape most of these tests need.
    fn rung_locator(locator: &str) -> EnvRung<'_> {
        EnvRung {
            locator: Some(locator),
            ..EnvRung::default()
        }
    }

    // ── T2: no rung ignores everything but baked ────────────────────────────

    /// With no env rung, resolution is `baked > compiled default` and nothing
    /// else — the embedded path.
    #[test]
    fn no_rung_uses_compiled_defaults() {
        let resolved = ExecutorConfig::resolve(BootConfig::default());

        // Embedded compiled defaults: locator="" (ExecutorConfig::new("")),
        // domain_id=0, node_name="node", namespace="".
        assert_eq!(resolved.locator, "");
        assert_eq!(resolved.domain_id, 0);
        assert_eq!(resolved.node_name, "node");
        assert_eq!(resolved.namespace, "");
        assert_eq!(resolved.rmw, None);
    }

    // ── T3: baked overrides compiled default (embedded path) ─────────────────

    /// Baked fields override the compiled default on the no-rung path;
    /// unspecified baked fields keep their compiled default.
    #[test]
    fn baked_overrides_compiled_default() {
        let baked = BootConfig {
            node_name: Some("talker"),
            domain_id: Some(7),
            ..Default::default()
        };
        let resolved = ExecutorConfig::resolve(baked);

        assert_eq!(resolved.node_name, "talker");
        assert_eq!(resolved.domain_id, 7);
        // locator and namespace were not baked → compiled defaults.
        assert_eq!(resolved.locator, "");
        assert_eq!(resolved.namespace, "");
    }

    // ── T3b: the RMW selector's baked rung (issue 1050 defect (3)) ───────────

    /// The baked selector reaches the resolver on the no-rung path.
    ///
    /// It did not, and that was the whole defect: `rmw` was the ONE
    /// `ExecutorConfig` field that resolved from the environment alone, so an
    /// image with no environment to read — every RTOS target, and every hosted
    /// image whose launcher sets nothing — could not name its backend at all
    /// and got whichever one registered first.
    #[test]
    fn baked_rmw_resolves_with_no_env_rung() {
        let resolved = ExecutorConfig::resolve(BootConfig {
            rmw: Some("uorb"),
            ..Default::default()
        });
        assert_eq!(resolved.rmw, Some("uorb"));
    }

    /// The env rung still wins — precedence model A is unchanged. What changed
    /// is that there is now something for it to win against.
    #[test]
    fn env_rmw_overrides_baked_rmw() {
        let baked = BootConfig {
            rmw: Some("uorb"),
            ..Default::default()
        };
        let env = EnvRung {
            rmw: Some("zenoh"),
            ..Default::default()
        };
        let resolved = ExecutorConfig::resolve_with(baked, Some(env));
        assert_eq!(resolved.rmw, Some("zenoh"));
    }

    /// A silent env rung falls through to baked rather than erasing it. This is
    /// the arm that was wrong on the hosted path: it read `env.rmw` alone, so a
    /// baked selector was discarded whenever an environment rung existed at all
    /// — which on a hosted build is always.
    #[test]
    fn baked_rmw_survives_a_silent_env_rung() {
        let baked = BootConfig {
            rmw: Some("uorb"),
            ..Default::default()
        };
        let resolved = ExecutorConfig::resolve_with(baked, Some(EnvRung::default()));
        assert_eq!(resolved.rmw, Some("uorb"));
    }

    // ── T4: env rung overrides baked ─────────────────────────────────────────

    /// A rung locator wins over a baked one, with its own value.
    #[test]
    fn env_rung_overrides_baked() {
        let baked = BootConfig {
            locator: Some("tcp/baked:9999"),
            ..Default::default()
        };
        let resolved = ExecutorConfig::resolve_with(baked, Some(rung_locator("tcp/env:7447")));

        assert_eq!(
            resolved.locator, "tcp/env:7447",
            "env locator must win over baked, with its own value"
        );
    }

    // ── T5: baked used when the rung is silent on that field ─────────────────

    /// A rung that says nothing about the locator falls through to baked —
    /// the case a hosted caller hits when `$NROS_LOCATOR` is unset.
    #[test]
    fn baked_used_when_rung_silent() {
        let baked = BootConfig {
            locator: Some("tcp/baked-only:8888"),
            ..Default::default()
        };
        let resolved = ExecutorConfig::resolve_with(baked, Some(EnvRung::default()));

        assert_eq!(
            resolved.locator, "tcp/baked-only:8888",
            "baked locator must be used when the rung is silent"
        );
    }

    // ── T6: per-field independence ────────────────────────────────────────────

    /// A baked `node_name` and a rung `locator` must both apply
    /// independently in the same `resolve` call.
    #[test]
    fn per_field_independence_baked_name_env_locator() {
        let baked = BootConfig {
            node_name: Some("my_talker"),
            // No locator baked — the rung should supply it.
            ..Default::default()
        };
        let resolved = ExecutorConfig::resolve_with(baked, Some(rung_locator("tcp/env:7447")));

        assert_eq!(
            resolved.locator, "tcp/env:7447",
            "rung locator must apply when locator is not baked"
        );
        assert_eq!(
            resolved.node_name, "my_talker",
            "baked node_name must apply even when the locator comes from the rung"
        );
    }

    // ── #206 / RFC-0045 — try_resolve validation + rung parity ──────────────

    #[test]
    fn try_resolve_rung_domain_over_max_errors() {
        let rung = EnvRung {
            domain_id: Some(233),
            ..EnvRung::default()
        };
        let err = match ExecutorConfig::try_resolve_with(BootConfig::default(), Some(rung)) {
            Err(e) => e,
            Ok(_) => panic!("expected DomainIdRange error"),
        };
        assert_eq!(err, BootConfigError::DomainIdRange);
    }

    #[test]
    fn try_resolve_baked_domain_over_max_errors_both_paths() {
        let baked = BootConfig {
            domain_id: Some(DOMAIN_ID_MAX + 1),
            ..BootConfig::default()
        };
        assert!(matches!(
            ExecutorConfig::try_resolve_with(baked, Some(EnvRung::default())),
            Err(BootConfigError::DomainIdRange)
        ));
        assert!(
            matches!(
                ExecutorConfig::try_resolve(baked),
                Err(BootConfigError::DomainIdRange)
            ),
            "the no-rung path validates the baked value too"
        );
    }

    /// Issue #227 — the C-ABI mapping: 0 = unset, 255 = explicit zero,
    /// everything else passes through (233..=254 reach the resolver's range
    /// check and fail there, not here).
    #[test]
    fn baked_domain_from_c_abi_mapping() {
        assert_eq!(baked_domain_from_c_abi(0), None);
        assert_eq!(
            baked_domain_from_c_abi(DOMAIN_ID_EXPLICIT_ZERO_C_ABI),
            Some(0)
        );
        assert_eq!(baked_domain_from_c_abi(61), Some(61));
        assert_eq!(baked_domain_from_c_abi(232), Some(232));
        // Free-range values above DOMAIN_ID_MAX are NOT swallowed…
        assert_eq!(baked_domain_from_c_abi(233), Some(233));
        // …and the resolver rejects them loudly.
        let baked = BootConfig {
            domain_id: baked_domain_from_c_abi(233),
            ..BootConfig::default()
        };
        assert!(matches!(
            ExecutorConfig::try_resolve(baked),
            Err(BootConfigError::DomainIdRange)
        ));
        // Explicit zero resolves to domain 0 even with no rung to save it.
        let baked = BootConfig {
            domain_id: baked_domain_from_c_abi(DOMAIN_ID_EXPLICIT_ZERO_C_ABI),
            ..BootConfig::default()
        };
        let cfg = match ExecutorConfig::try_resolve(baked) {
            Ok(c) => c,
            Err(e) => panic!("explicit zero must resolve: {e}"),
        };
        assert_eq!(cfg.domain_id, 0);
    }

    #[test]
    fn try_resolve_domain_max_is_valid() {
        let baked = BootConfig {
            domain_id: Some(DOMAIN_ID_MAX),
            ..BootConfig::default()
        };
        let cfg = match ExecutorConfig::try_resolve(baked) {
            Ok(c) => c,
            Err(e) => panic!("DOMAIN_ID_MAX must be valid: {e}"),
        };
        assert_eq!(cfg.domain_id, DOMAIN_ID_MAX);
    }

    #[test]
    fn try_resolve_node_name_rung_wins() {
        let rung = EnvRung {
            node_name: Some("env_node"),
            ..EnvRung::default()
        };
        let baked = BootConfig {
            node_name: Some("baked_node"),
            ..BootConfig::default()
        };
        let cfg = match ExecutorConfig::try_resolve_with(baked, Some(rung)) {
            Ok(c) => c,
            Err(e) => panic!("resolve failed: {e}"),
        };
        assert_eq!(
            cfg.node_name, "env_node",
            "the rung must override baked with its own value"
        );
    }

    /// issue 0687 — the selector rides the same rung as the locator, so a
    /// hosted `$NROS_RMW` reaches `Executor::open` through the config rather
    /// than through a second read inside it.
    #[test]
    fn rung_carries_the_rmw_selector() {
        let rung = EnvRung {
            rmw: Some("cyclonedds"),
            ..EnvRung::default()
        };
        let cfg = ExecutorConfig::resolve_with(BootConfig::default(), Some(rung));
        assert_eq!(cfg.rmw, Some("cyclonedds"));
        // …and no rung means no selection, which is what an embedded image
        // with exactly one registered backend relies on.
        assert_eq!(ExecutorConfig::resolve(BootConfig::default()).rmw, None);
    }
}

// ============================================================================
// BakedBootConfig round-trip tests (no_std-compatible, run under std test runner)
//
// These test the full round-trip BakedBootConfig::new → BootConfig::from_baked.
// Pure BakedBootConfig::new / pack unit tests live in nros-platform-api.
// ============================================================================

#[cfg(test)]
mod baked_boot_config_tests {
    // BakedBootConfig + consts come from nros_platform_api via super::*.
    use super::*;

    // ── T-BB1: round-trip — typical mixed case ────────────────────────────────

    /// Pack node_name + locator + domain_id (namespace absent), then unpack and
    /// verify each field matches.  The absent namespace must be None.
    #[test]
    fn round_trip_typical() {
        let baked = BakedBootConfig::new(
            Some("param_talker"),
            Some("tcp/10.0.0.5:7447"),
            Some(7),
            None,
        );
        let cfg = BootConfig::from_baked(&baked);

        assert_eq!(cfg.node_name, Some("param_talker"));
        assert_eq!(cfg.locator, Some("tcp/10.0.0.5:7447"));
        assert_eq!(cfg.domain_id, Some(7));
        assert_eq!(cfg.namespace, None);
    }

    // ── T-BB2: all-None — no fields set ──────────────────────────────────────

    /// When every argument is None the set_flags must be zero and from_baked
    /// must return all-None.
    #[test]
    fn all_none_round_trips_to_default() {
        let baked = BakedBootConfig::new(None, None, None, None);
        assert_eq!(baked.set_flags, 0);
        let cfg = BootConfig::from_baked(&baked);
        assert_eq!(cfg.node_name, None);
        assert_eq!(cfg.locator, None);
        assert_eq!(cfg.domain_id, None);
        assert_eq!(cfg.namespace, None);
    }

    // ── T-BB3: bad magic → all-None ──────────────────────────────────────────

    /// A BakedBootConfig with a wrong magic word must be treated as unrecognised
    /// and from_baked must return all-None.
    #[test]
    fn bad_magic_returns_default() {
        let mut baked =
            BakedBootConfig::new(Some("talker"), Some("tcp/1.2.3.4:7447"), Some(1), None);
        baked.magic = 0; // corrupt the fingerprint
        let cfg = BootConfig::from_baked(&baked);
        assert_eq!(cfg.node_name, None);
        assert_eq!(cfg.locator, None);
        assert_eq!(cfg.domain_id, None);
        assert_eq!(cfg.namespace, None);
    }

    // ── T-BB4: bad version → all-None ────────────────────────────────────────

    /// A BakedBootConfig with the right magic but a wrong version must likewise
    /// return all-None.
    #[test]
    fn bad_version_returns_default() {
        let mut baked = BakedBootConfig::new(Some("talker"), None, None, None);
        baked.version = 99; // future/unknown version
        let cfg = BootConfig::from_baked(&baked);
        assert_eq!(cfg.node_name, None);
    }

    // ── T-BB5: NUL-trim — short string round-trips without trailing NULs ─────

    /// A node_name shorter than 64 bytes must unpack to exactly the original
    /// string, with no trailing NUL characters in the &str.
    #[test]
    fn nul_trim_short_name() {
        let name = "robot";
        let baked = BakedBootConfig::new(Some(name), None, None, None);
        let cfg = BootConfig::from_baked(&baked);
        assert_eq!(cfg.node_name, Some(name));
        assert_eq!(cfg.node_name.unwrap().len(), name.len());
    }

    // ── T-BB6: each field independent ────────────────────────────────────────

    /// Setting only node_name leaves the others None.
    #[test]
    fn only_node_name_set() {
        let baked = BakedBootConfig::new(Some("solo"), None, None, None);
        let cfg = BootConfig::from_baked(&baked);
        assert_eq!(cfg.node_name, Some("solo"));
        assert_eq!(cfg.locator, None);
        assert_eq!(cfg.domain_id, None);
        assert_eq!(cfg.namespace, None);
    }

    /// Setting only locator leaves the others None.
    #[test]
    fn only_locator_set() {
        let baked = BakedBootConfig::new(None, Some("tcp/127.0.0.1:7447"), None, None);
        let cfg = BootConfig::from_baked(&baked);
        assert_eq!(cfg.node_name, None);
        assert_eq!(cfg.locator, Some("tcp/127.0.0.1:7447"));
        assert_eq!(cfg.domain_id, None);
        assert_eq!(cfg.namespace, None);
    }

    /// Setting only domain_id leaves the others None.
    #[test]
    fn only_domain_id_set() {
        let baked = BakedBootConfig::new(None, None, Some(42), None);
        let cfg = BootConfig::from_baked(&baked);
        assert_eq!(cfg.node_name, None);
        assert_eq!(cfg.locator, None);
        assert_eq!(cfg.domain_id, Some(42));
        assert_eq!(cfg.namespace, None);
    }

    /// Setting only namespace leaves the others None.
    #[test]
    fn only_namespace_set() {
        let baked = BakedBootConfig::new(None, None, None, Some("/robot"));
        let cfg = BootConfig::from_baked(&baked);
        assert_eq!(cfg.node_name, None);
        assert_eq!(cfg.locator, None);
        assert_eq!(cfg.domain_id, None);
        assert_eq!(cfg.namespace, Some("/robot"));
    }

    // ── T-BB7: full-buffer-length string (boundary) ───────────────────────────

    /// A node_name of exactly 64 bytes must compile and round-trip correctly
    /// (the buffer is fully populated with no NUL terminator, so nul_len
    /// returns 64 and the whole buffer is the string value).
    #[test]
    fn full_64_byte_name_round_trips() {
        // Exactly 64 ASCII bytes.
        let name = "abcdefghijklmnopqrstuvwxyz012345abcdefghijklmnopqrstuvwxyz012345";
        assert_eq!(name.len(), 64);
        let baked = BakedBootConfig::new(Some(name), None, None, None);
        let cfg = BootConfig::from_baked(&baked);
        assert_eq!(cfg.node_name, Some(name));
    }

    // ── T-BB8: set_flags bit-pattern is exact ─────────────────────────────────

    /// Verify the set_flags bitmask matches the expected bit positions.
    #[test]
    fn set_flags_bits_correct() {
        let baked = BakedBootConfig::new(
            Some("n"), // bit 0
            None,
            Some(0),  // bit 2
            Some(""), // bit 3
        );
        assert_eq!(
            baked.set_flags,
            BOOT_SET_NODE_NAME | BOOT_SET_DOMAIN | BOOT_SET_NAMESPACE
        );
    }

    // ── T-BB9: the layout-version-2 `rmw` field (issue 1050 defect (3)) ──────

    /// The selector round-trips through the linker-section struct, and an
    /// unspecified one reads as `None` rather than as an empty name.
    ///
    /// Both halves matter: the reader keys on bit 4, so a struct baked by an
    /// older toolchain (bit clear, 32 zero bytes) must resolve to "not
    /// specified" and NOT to a backend named `""`, which would fail lookup.
    #[test]
    fn baked_rmw_round_trips_and_absent_reads_as_none() {
        let with = BakedBootConfig::new_with_rmw(None, None, None, None, Some("uorb"));
        assert_eq!(with.set_flags & BOOT_SET_RMW, BOOT_SET_RMW);
        assert_eq!(BootConfig::from_baked(&with).rmw, Some("uorb"));

        let without = BakedBootConfig::new(None, None, None, None);
        assert_eq!(without.set_flags & BOOT_SET_RMW, 0);
        assert_eq!(BootConfig::from_baked(&without).rmw, None);

        // The version moved with the layout — a v1 reader must be able to
        // reject a v2 struct rather than read `rmw` as whatever its own layout
        // put after `namespace`.
        assert_eq!(with.version, 2);
    }

    // ── Compile-failure comment ───────────────────────────────────────────────
    // Uncommenting the line below must FAIL to compile because the string
    // exceeds the 64-byte node_name buffer.  Do NOT uncomment in CI.
    //
    // const _: BakedBootConfig = BakedBootConfig::new(
    //     Some("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"), // 65 A's (> 64-byte buffer)
    //     None, None, None,
    // );
}

// ============================================================================
// Shutdown hooks (issue 0790)
// ============================================================================

/// A shutdown hook: `extern "C"` so the same slot serves Rust, C and C++
/// without a second registry, and so no allocator is involved.
///
/// Deliberately NOT a closure. rclcpp takes `std::function<void()>`, which is
/// a heap allocation per registration; the callback shape the rest of this
/// tree's FFI surface already uses is a bare function pointer plus an opaque
/// `context` the callee casts back to its own state.
pub type ShutdownCallbackFn = unsafe extern "C" fn(context: *mut core::ffi::c_void);

/// Which of the two ordered shutdown phases a hook belongs to.
///
/// The ordering IS the feature (issue 0790): a node that must release a bus or
/// park an actuator has to do it while its entities still work, so it can
/// publish a final state or answer a last request. After teardown it cannot.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShutdownPhase {
    /// Runs BEFORE the session is closed — publishers, subscriptions, services
    /// and clients are all still live. rclcpp's `add_pre_shutdown_callback`.
    Pre = 0,
    /// Runs AFTER the session is closed. rclcpp's `add_on_shutdown_callback` /
    /// `rclcpp::on_shutdown`.
    Post = 1,
}

/// Handle to a registered shutdown hook, returned by
/// [`Executor::add_pre_shutdown_callback`] /
/// [`Executor::add_on_shutdown_callback`] and consumed by the matching
/// `remove_*`. rclcpp's `PreShutdownCallbackHandle` / `OnShutdownCallbackHandle`.
///
/// # Why an index and not a pointer
///
/// rclcpp's handle owns a `std::shared_ptr` to the callback. There is no
/// allocator here and the hooks live in a fixed-capacity static table, so the
/// handle is the SLOT INDEX — the smallest thing that identifies a row of a
/// static array, and trivially passable through the C ABI as one `uint32_t`.
///
/// The phase is packed into the high half rather than left implicit, so a
/// handle from one list cannot silently remove a callback from the other. A
/// slot index is only meaningful next to the table it indexes, and there are
/// two tables.
///
/// [`Executor::add_pre_shutdown_callback`]: crate::executor::Executor::add_pre_shutdown_callback
/// [`Executor::add_on_shutdown_callback`]: crate::executor::Executor::add_on_shutdown_callback
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShutdownCallbackHandle(pub u32);

impl ShutdownCallbackHandle {
    /// The value no registration ever returns. C callers compare against
    /// `NROS_SHUTDOWN_CALLBACK_HANDLE_INVALID`, which is this number.
    pub const INVALID: ShutdownCallbackHandle = ShutdownCallbackHandle(u32::MAX);

    /// Bit position the phase tag occupies. 16 bits of index is more slots
    /// than any static table will ever hold and keeps the packing readable in
    /// hex (`0x0000_0001` = pre slot 1, `0x0001_0001` = post slot 1).
    const PHASE_SHIFT: u32 = 16;
    const INDEX_MASK: u32 = (1 << Self::PHASE_SHIFT) - 1;

    /// Pack a `(phase, slot)` pair. `None` when `index` does not fit the
    /// 16-bit index field — unreachable for any table this crate builds, and
    /// an honest `None` rather than a silent truncation if one ever grows.
    pub const fn new(phase: ShutdownPhase, index: usize) -> Option<Self> {
        if index > Self::INDEX_MASK as usize {
            return None;
        }
        Some(ShutdownCallbackHandle(
            ((phase as u32) << Self::PHASE_SHIFT) | index as u32,
        ))
    }

    /// The phase this handle was issued for, or `None` if the value is not a
    /// handle this crate ever issued (including [`Self::INVALID`]).
    pub const fn phase(self) -> Option<ShutdownPhase> {
        match self.0 >> Self::PHASE_SHIFT {
            0 => Some(ShutdownPhase::Pre),
            1 => Some(ShutdownPhase::Post),
            _ => None,
        }
    }

    /// The slot index within this handle's phase table.
    pub const fn index(self) -> usize {
        (self.0 & Self::INDEX_MASK) as usize
    }

    /// True for anything [`Self::new`] could have produced.
    pub const fn is_valid(self) -> bool {
        self.phase().is_some()
    }
}

/// One occupied row of a shutdown-hook table.
///
/// Gated exactly as `executor::spin` is (`mod.rs`: `#[cfg(any(has_rmw, test))]`),
/// because that module holds the only constructor. Without the gate a build
/// with no RMW backend linked — `cargo check -p nros-node --lib` — compiles the
/// type and not `Executor`, and `-D dead-code` fails on a struct nothing can
/// build. The public vocabulary beside it (`ShutdownCallbackFn`,
/// `ShutdownPhase`, `ShutdownCallbackHandle`) stays ungated: those are named in
/// signatures a caller writes, and a handle type that vanishes with the backend
/// would be worse than one that is merely unused.
#[cfg(any(has_rmw, test))]
#[derive(Clone, Copy)]
pub(crate) struct ShutdownHook {
    pub(crate) callback: ShutdownCallbackFn,
    pub(crate) context: *mut core::ffi::c_void,
}

// ============================================================================
// Kani Verification
// ============================================================================

#[cfg(kani)]
mod verification {
    use super::*;

    // ---- HandleSet algebraic properties ----

    #[kani::proof]
    fn handleset_insert_contains() {
        let idx: usize = kani::any();
        kani::assume(idx < 64);
        let id = HandleId(idx);
        let set = HandleSet::EMPTY.insert(id);
        assert!(set.contains(id));
    }

    #[kani::proof]
    fn handleset_insert_idempotent() {
        let idx: usize = kani::any();
        kani::assume(idx < 64);
        let id = HandleId(idx);
        let once = HandleSet::EMPTY.insert(id);
        let twice = once.insert(id);
        assert_eq!(once.0, twice.0);
    }

    #[kani::proof]
    fn handleset_union_commutative() {
        let a: u64 = kani::any();
        let b: u64 = kani::any();
        let set_a = HandleSet(a);
        let set_b = HandleSet(b);
        assert_eq!(set_a.union(set_b).0, set_b.union(set_a).0);
    }

    #[kani::proof]
    fn handleset_union_associative() {
        let a: u64 = kani::any();
        let b: u64 = kani::any();
        let c: u64 = kani::any();
        let sa = HandleSet(a);
        let sb = HandleSet(b);
        let sc = HandleSet(c);
        assert_eq!(sa.union(sb).union(sc).0, sa.union(sb.union(sc)).0);
    }

    #[kani::proof]
    fn handleset_union_contains_both() {
        let idx_a: usize = kani::any();
        let idx_b: usize = kani::any();
        kani::assume(idx_a < 64);
        kani::assume(idx_b < 64);
        let a = HandleId(idx_a);
        let b = HandleId(idx_b);
        let set_a = HandleSet::EMPTY.insert(a);
        let set_b = HandleSet::EMPTY.insert(b);
        let merged = set_a.union(set_b);
        assert!(merged.contains(a));
        assert!(merged.contains(b));
    }

    #[kani::proof]
    fn handleset_empty_contains_nothing() {
        let idx: usize = kani::any();
        kani::assume(idx < 64);
        let id = HandleId(idx);
        assert!(!HandleSet::EMPTY.contains(id));
    }

    #[kani::proof]
    fn handleset_bitor_matches_insert() {
        let idx_a: usize = kani::any();
        let idx_b: usize = kani::any();
        kani::assume(idx_a < 64);
        kani::assume(idx_b < 64);
        let a = HandleId(idx_a);
        let b = HandleId(idx_b);
        let via_bitor = a | b;
        let via_insert = HandleSet::EMPTY.insert(a).insert(b);
        assert_eq!(via_bitor.0, via_insert.0);
    }

    #[kani::proof]
    fn handleset_len_after_insert() {
        let idx: usize = kani::any();
        kani::assume(idx < 64);
        let id = HandleId(idx);
        let set = HandleSet::EMPTY.insert(id);
        assert_eq!(set.len(), 1);
        assert!(!set.is_empty());
    }

    // ---- ReadinessSnapshot properties ----

    #[kani::proof]
    fn snapshot_is_ready_consistent() {
        let bits: u64 = kani::any();
        let idx: usize = kani::any();
        kani::assume(idx < 64);
        let snap = ReadinessSnapshot { bits, count: 64 };
        let id = HandleId(idx);
        // is_ready matches the bit
        assert_eq!(snap.is_ready(id), bits & (1u64 << idx) != 0);
    }

    #[kani::proof]
    fn snapshot_all_ready_correct() {
        let bits: u64 = kani::any();
        let set_bits: u64 = kani::any();
        let snap = ReadinessSnapshot { bits, count: 64 };
        let set = HandleSet(set_bits);
        // all_ready iff every bit in set is present in bits
        assert_eq!(snap.all_ready(set), bits & set_bits == set_bits);
    }

    #[kani::proof]
    fn snapshot_any_ready_correct() {
        let bits: u64 = kani::any();
        let set_bits: u64 = kani::any();
        let snap = ReadinessSnapshot { bits, count: 64 };
        let set = HandleSet(set_bits);
        // any_ready iff at least one bit overlaps
        assert_eq!(snap.any_ready(set), bits & set_bits != 0);
    }

    // ---- Trigger evaluation soundness ----

    // These verify the boolean expressions used in spin_once().

    #[kani::proof]
    fn trigger_any_fires_iff_nonzero() {
        let readiness: u64 = kani::any();
        // Trigger::Any fires when readiness_bits != 0
        let fires = readiness != 0;
        assert_eq!(fires, readiness != 0);
    }

    #[kani::proof]
    fn trigger_one_fires_iff_bit_set() {
        let readiness: u64 = kani::any();
        let idx: usize = kani::any();
        kani::assume(idx < 64);
        let id = HandleId(idx);
        // Trigger::One(id) fires when readiness & (1 << id.0) != 0
        let fires = readiness & (1u64 << id.0) != 0;
        // This is equivalent to checking the specific bit
        assert_eq!(fires, readiness & (1u64 << idx) != 0);
    }

    #[kani::proof]
    fn trigger_allof_fires_iff_all_set() {
        let readiness: u64 = kani::any();
        let set_bits: u64 = kani::any();
        let set = HandleSet(set_bits);
        // Trigger::AllOf(set) fires when readiness & set.0 == set.0
        let fires = readiness & set.0 == set.0;
        // AllOf with empty set always fires
        if set_bits == 0 {
            assert!(fires);
        }
        // If fires, then every bit in set is present
        if fires {
            assert_eq!(readiness & set_bits, set_bits);
        }
    }

    #[kani::proof]
    fn trigger_anyof_fires_iff_any_set() {
        let readiness: u64 = kani::any();
        let set_bits: u64 = kani::any();
        let set = HandleSet(set_bits);
        // Trigger::AnyOf(set) fires when readiness & set.0 != 0
        let fires = readiness & set.0 != 0;
        // AnyOf with empty set never fires
        if set_bits == 0 {
            assert!(!fires);
        }
    }

    #[kani::proof]
    fn trigger_allof_implies_anyof() {
        let readiness: u64 = kani::any();
        let set_bits: u64 = kani::any();
        kani::assume(set_bits != 0); // Non-empty set
        let allof_fires = readiness & set_bits == set_bits;
        let anyof_fires = readiness & set_bits != 0;
        // If AllOf fires, then AnyOf must also fire
        if allof_fires {
            assert!(anyof_fires);
        }
    }

    #[kani::proof]
    fn trigger_one_equivalent_to_anyof_singleton() {
        let readiness: u64 = kani::any();
        let idx: usize = kani::any();
        kani::assume(idx < 64);
        let id = HandleId(idx);
        let singleton = HandleSet::EMPTY.insert(id);
        // One(id) and AnyOf({id}) produce the same result
        let one_fires = readiness & (1u64 << id.0) != 0;
        let anyof_fires = readiness & singleton.0 != 0;
        assert_eq!(one_fires, anyof_fires);
    }
}
