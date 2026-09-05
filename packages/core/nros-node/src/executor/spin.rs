//! Executor struct and core spin methods.

use core::{marker::PhantomData, mem::MaybeUninit};

use nros_core::{RosMessage, RosService, ViewableMessage};
use nros_rmw::{QoSProfile, ServiceInfo, Session, TopicInfo, TransportError};

use crate::{session, timer::TimerDuration};

#[cfg(feature = "safety-e2e")]
use super::arena::{
    SubSafetyEntry, sub_safety_has_data, sub_safety_pre_sample, sub_safety_try_process,
};
#[cfg(feature = "rmw-cffi")]
use super::types::ExecutorConfig;
#[cfg(feature = "alloc")]
use super::types::SpinOptions;
use super::{
    arena::{
        BufferStrategy, CallbackMeta, EntryKind, GuardConditionEntry, ServiceClientCallbackEntry,
        ServiceClientRawArenaEntry, ServiceClientSendHeader, SrvEntry, SrvRawEntry,
        SubBufferedEntry, SubBufferedRawCEntry, SubBufferedRawEntry, SubBufferedRawInfoCEntry,
        SubBufferedRawInfoEntry, SubBufferedViewEntry, SubInfoEntry, SubInplaceEntry,
        TimerClockSource, TimerEntry, TimerHeader, TimerOverrunPolicy, TraceName, always_ready,
        buffered_region_size, drop_entry, guard_has_data, guard_try_process, no_pre_sample,
        service_client_callback_try_process, service_client_raw_try_process, srv_has_data,
        srv_raw_has_data, srv_raw_try_process, srv_try_process, sub_buffered_has_data,
        sub_buffered_raw_c_has_data, sub_buffered_raw_c_try_process, sub_buffered_raw_has_data,
        sub_buffered_raw_info_c_has_data, sub_buffered_raw_info_c_try_process,
        sub_buffered_raw_info_has_data, sub_buffered_raw_info_try_process,
        sub_buffered_raw_try_process, sub_buffered_try_process, sub_buffered_view_has_data,
        sub_buffered_view_try_process, sub_info_has_data, sub_info_pre_sample,
        sub_info_try_process, sub_inplace_has_data, sub_inplace_try_process, timer_try_process,
    },
    node::NodeHandle,
    spsc_ring::SpscRing,
    triple_buffer::TripleBuffer,
    types::{
        ExecutorSemantics, GuardCondition, HandleId, InvocationMode, NodeError,
        RawResponseCallback, RawServiceCallback, RawSubscriptionCallback,
        RawSubscriptionInfoCallback, ReadinessSnapshot, SpinOnceResult, SpinPeriodPollingResult,
        Trigger,
    },
};

// ============================================================================
// Phase 8 — callback registration event (paired stubs)
// ============================================================================
//
// `docs/design/callback_tracing.rst`. Paired stubs (the `entry_tiers.rs`
// idiom) so `emplace_entry` reads identically whether or not the feature is
// on, and the `#[cfg]` lives in exactly one place.

/// Emit `nros_callback_register(handle, kind, name)` for a newly installed
/// executor entry.
#[cfg(feature = "trace-callbacks")]
#[inline]
fn trace_register(slot: usize, kind: EntryKind, name: TraceName<'_>) {
    super::callback_trace::register(slot, kind, name);
}

#[cfg(not(feature = "trace-callbacks"))]
#[inline]
fn trace_register(_slot: usize, _kind: EntryKind, _name: TraceName<'_>) {}

// ============================================================================
// Executor::open() factory method
// ============================================================================

/// phase-271 — leak a default-sized (`ExecutorSizing::DEFAULT`) `u64` backing,
/// yielding the `'static` storage the `alloc` convenience constructors borrow.
/// One-time, executor-lifetime allocation (the executor lives for the program);
/// intentionally not freed. `alloc`-only — no_std-no-alloc entries supply their
/// own `static`/stack backing via `from_session_in` / the `nros::main!` macro.
#[cfg(feature = "alloc")]
fn leak_default_backing(sizing: super::storage::ExecutorSizing) -> &'static mut [MaybeUninit<u64>] {
    alloc::boxed::Box::leak(alloc::boxed::Box::new_uninit_slice(sizing.u64_len()))
}

#[cfg(feature = "rmw-cffi")]
impl<'s> Executor<'s> {
    /// phase-271 — open a new executor session over caller-supplied `backing`,
    /// sized by `sizing` (per-entry sizing). The core, non-generic sized entry
    /// point: the `alloc` [`open`](Self::open) convenience leaks a default
    /// backing and delegates here, and the `nros::main!` macro emits a backing
    /// sized to the entry's own entity count.
    ///
    /// Phase 115.M.4 — auto-registers the cffi vtable for whichever
    /// backend the build was configured for, mirroring the C++ side's
    /// `#ifdef NROS_RMW_<NAME>` fan-out in `<nros/node.hpp>`. The
    /// runtime's atomic vtable slot is idempotent: a re-call of any
    /// backend's `register()` is a no-op, so the fan-out below is safe
    /// to invoke on every `Executor::open` (cheaper than a `Once` and
    /// doesn't pull in `std::sync` for no_std targets).
    ///
    /// Connects to the middleware at the locator specified in `config`.
    ///
    /// # Safety
    /// `backing` must be ≥ `sizing.u64_len()` words, live for `'s`, and be
    /// otherwise untouched while the executor lives (see
    /// [`from_session_in`](Self::from_session_in)).
    pub unsafe fn open_in(
        config: &ExecutorConfig<'_>,
        backing: &'s mut [MaybeUninit<u64>],
        sizing: super::storage::ExecutorSizing,
    ) -> Result<Self, NodeError> {
        use nros_rmw::Rmw;

        // Phase 128.A.3 / 249 P4b.1 — manifest-driven backend selection.
        //
        // Every linked backend self-registered via its `.init_array`
        // ctor before `main` (RFC-0042 §D3.3), so the registry is
        // already populated — no runtime section walk.
        //
        // 1. Honour `config.rmw` — the caller's explicit backend selection,
        //    mirroring ROS 2's `RMW_IMPLEMENTATION`. On a hosted build
        //    `nros::ExecutorConfigEnvExt::from_env` / `nros::env::resolve_hosted`
        //    fill it from `$NROS_RMW`; issue 0687 moved that read to the edge,
        //    because reading a process environment is what kept `std` in this
        //    crate and an RTOS image has no environment to read.
        // 2. With no selector, pick the unique registered backend.
        //    Zero registered → `NoBackend`; more than one →
        //    `Ambiguous` (user must select one, or use
        //    `Executor::open_multi`).
        let selector = config.rmw;
        match nros_rmw_cffi::resolve_backend(selector.map(str::as_bytes)) {
            nros_rmw_cffi::BackendResolution::Single(_) => {}
            // Issue 0436 — these are SELECTION outcomes, not transport failures,
            // and calling them `ConnectionFailed` actively misleads: a PX4 bridge
            // that registered two backends reported "connection failed", which
            // reads as a router/network problem and was chased as one. Nothing is
            // connected at this point — the resolver has not chosen a backend yet.
            //
            // The variant still collapses into `NodeError::Transport` (the enum has
            // no selection arm, and adding one is an ABI change across the C/C++
            // seams), but `InvalidConfig` at least says "your configuration is
            // unresolvable", and the std-gated line below names WHICH outcome.
            other => {
                {
                    let why: &str = match other {
                        nros_rmw_cffi::BackendResolution::NoBackend => {
                            "no RMW backend is registered"
                        }
                        nros_rmw_cffi::BackendResolution::Ambiguous => {
                            "more than one RMW backend is registered and no \
                             $NROS_RMW selector was set — name one (e.g. \
                             NROS_RMW=uorb), or open per-backend sessions"
                        }
                        nros_rmw_cffi::BackendResolution::Unknown => {
                            "$NROS_RMW names a backend that is not registered"
                        }
                        nros_rmw_cffi::BackendResolution::Single(_) => unreachable!(),
                    };
                    nros_log::nros_error!(
                        nros_log::get_logger("nros"),
                        "cannot select an RMW backend — {why}"
                    );
                }
                let _ = other;
                return Err(NodeError::Transport(TransportError::InvalidConfig));
            }
        }

        let rmw_config = nros_rmw::RmwConfig {
            locator: config.locator,
            mode: config.mode,
            domain_id: config.domain_id,
            node_name: config.node_name,
            namespace: config.namespace,
            properties: &[],
        };
        let session = if let Some(name) = selector {
            // Selector path: route to the specific named backend so
            // the env-var-disambiguated outcome matches what the
            // resolver above identified.
            nros_rmw_cffi::CffiRmw::open_with_rmw(name, &rmw_config)
        } else {
            nros_rmw_cffi::CffiRmw.open(&rmw_config)
        }
        .map_err(|e| {
            // Issue 0465 — do not relabel. This used to discard the backend's
            // error and report `ConnectionFailed` for every open failure, so an
            // exhausted session pool (`InvalidConfig`) and a router that is not
            // there produced the same sentence. Same lesson as the selection
            // arm above: say which failure happened.
            nros_log::nros_error!(
                nros_log::get_logger("nros"),
                "RMW session open failed — {e:?}"
            );
            NodeError::Transport(e)
        })?;
        // SAFETY: forwarded from this fn's contract — `backing`/`sizing` sized
        // + alive for `'s`.
        let mut executor = unsafe { Self::from_session_in(session, backing, sizing) };
        {
            // `config.clock_us` overrides the constructor's platform default
            // (issue: assigning `None` here clobbered it and re-enabled the
            // credit-the-requested-timeout fallback).
            if let Some(clock) = config.clock_us {
                executor.clock_us_fn = Some(clock);
                executor.last_spin_end_us = Some(clock());
            }
            // issue 0671 — the SAME rule as `clock_us` directly above, which is
            // why that one is guarded: a config that does not SPECIFY an epoch
            // must not be read as "this target HAS no epoch". Assigning `None`
            // here clobbered the constructor's platform default
            // (`Some(default_epoch_us)`), and `ExecutorConfig::new` — the path
            // `nros::init_*` + `ctx.config()` takes — leaves `epoch_us: None`,
            // so EVERY hosted node built that way silently lost its wall clock.
            // With no epoch, `Node::subscription` never attaches the age cell
            // (it needs `(STAMP_OFFSET, epoch)` both `Some`), so a baked
            // `max_age_ms` contract becomes a silently-dead monitor — the exact
            // outcome RFC-0052 says must never happen. The rate monitor rides
            // the GUARDED `clock_us_fn` and kept working, which is why only the
            // age half went quiet.
            if let Some(epoch) = config.epoch_us {
                executor.epoch_us_fn = Some(epoch);
            }
        }
        executor.set_node_identity(config.node_name, config.namespace);
        // Issue 0656 — beside the identity, for the same reason: an entity needs
        // both to be addressable, and this half used to be dropped here.
        executor.domain_id = config.domain_id;
        #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
        executor.install_wake_signal_on_primary();
        // Phase 277 W2.c — readiness marker for E2E harnesses. This is the
        // single call-through `open()`/`open_sized()` share, so it fires on
        // every platform that reaches here (native, freertos, zephyr,
        // threadx, …) regardless of which RMW backend or board owns the
        // boot path. It replaces the per-example synthetic
        // `log::info!("Publishing messages")` markers W4 removes — those
        // only proved a callback had fired at least once; this line proves
        // the session itself is up, before any node/callback exists.
        //
        // STABILITY CONTRACT: the leading `"nros: session open"` text is
        // load-bearing — test harnesses grep for it verbatim. Keep it
        // stable even if the trailing `(rmw=...)` detail changes.
        #[cfg(feature = "log")]
        {
            if let Some(name) = selector {
                log::info!("nros: session open (rmw={name})");
            } else {
                log::info!("nros: session open");
            }
        }
        Ok(executor)
    }
}

#[cfg(all(feature = "rmw-cffi", feature = "alloc"))]
impl Executor<'static> {
    /// Open a new executor session using the active RMW backend, at the
    /// build-time default sizing. Convenience over
    /// [`open_in`](Self::open_in): leaks a default-sized backing (executor-
    /// lifetime) so existing callers keep the zero-storage-arg signature.
    /// Per-entry sizing goes through `open_in` / the `nros::main!` macro.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let config = ExecutorConfig::from_env().node_name("my_node");
    /// let mut executor = Executor::open(&config)?;
    /// ```
    pub fn open(config: &ExecutorConfig<'_>) -> Result<Self, NodeError> {
        Self::open_sized(config, super::storage::ExecutorSizing::DEFAULT)
    }

    /// phase-271 — like [`open`](Self::open) but sized to a caller-supplied
    /// `sizing` (its own declared topology) instead of the build-time default.
    /// The `alloc` entry point the `nros::main!` macro's native board path uses
    /// to size a fat entry (>default `MAX_CBS` callbacks) without a
    /// workspace-global `NROS_EXECUTOR_MAX_CBS`. Leaks a `sizing`-sized backing
    /// (executor-lifetime); no-alloc entries use `open_in` with their own static.
    pub fn open_sized(
        config: &ExecutorConfig<'_>,
        sizing: super::storage::ExecutorSizing,
    ) -> Result<Self, NodeError> {
        // SAFETY: leaked backing is exactly `sizing.u64_len()` words, `'static`,
        // uniquely owned by the returned executor.
        unsafe { Self::open_in(config, leak_default_backing(sizing), sizing) }
    }

    /// Phase 128.F.1 — explicit per-backend session declaration for
    /// bridge mode. `specs[0]` becomes the primary session; `specs[1..]`
    /// open as extras keyed by RMW name. After construction, every
    /// `create_node_on(name, rmw)` call dispatches to whichever
    /// session was opened under that RMW name (or, when the rmw name
    /// matches the primary, the primary session itself).
    ///
    /// Single-backend callers should keep using
    /// [`open`](Self::open) — this entry costs an extra
    /// `open_with_rmw` per spec and adds no value when only one
    /// backend is linked.
    ///
    /// `$NROS_RMW` env is ignored: bridge mode wants explicit names.
    ///
    /// Default-sized `alloc` convenience over
    /// [`open_multi_in`](Self::open_multi_in) (leaks a default backing).
    #[cfg(feature = "rmw-cffi")]
    pub fn open_multi(specs: &[SessionSpec<'_>]) -> Result<Self, NodeError> {
        let sizing = super::storage::ExecutorSizing::DEFAULT;
        // SAFETY: leaked backing is exactly `sizing.u64_len()` words, `'static`.
        unsafe { Self::open_multi_in(specs, leak_default_backing(sizing), sizing) }
    }

    /// Phase 104.C.1 — open the Executor against a specific RMW
    /// backend by name. Selects from the named registry (Phase
    /// 104.B.2). `rmw_name` must match one of the names a backend
    /// registered under (`"zenoh"`, `"cyclonedds"`, `"xrce"`, …).
    ///
    /// Equivalent to [`Executor::open`] when the registry has exactly
    /// one backend (the default-backend fast path). Use this entry
    /// point in multi-backend builds where `Executor::open` would
    /// pick the first-registered slot.
    ///
    /// Single-Executor multi-Node multi-RMW (the long-term Design X
    /// from `docs/roadmap/phase-104-multi-backend-bridges.md`) is
    /// follow-up work — Phase 104.C.2 + C.3.
    ///
    /// Default-sized `alloc` convenience over
    /// [`open_with_rmw_in`](Self::open_with_rmw_in) (leaks a default backing).
    #[cfg(feature = "rmw-cffi")]
    pub fn open_with_rmw(rmw_name: &str, config: &ExecutorConfig<'_>) -> Result<Self, NodeError> {
        let sizing = super::storage::ExecutorSizing::DEFAULT;
        // SAFETY: leaked backing is exactly `sizing.u64_len()` words, `'static`.
        unsafe { Self::open_with_rmw_in(rmw_name, config, leak_default_backing(sizing), sizing) }
    }
}

// phase-271 — no-alloc sized cores for the bridge/named open paths. In
// `impl<'s>` (not the `'static` alloc block) so they stay available in
// `rmw-cffi`-without-`alloc` builds (e.g. the `nros-bridge` no_std default),
// which is where `open_multi`/`open_with_rmw` lived before.
#[cfg(feature = "rmw-cffi")]
impl<'s> Executor<'s> {
    /// Per-entry-sized [`open_multi`](Self::open_multi): carves `backing` for
    /// the executor's tables instead of leaking a default one.
    ///
    /// # Safety
    /// `backing`/`sizing` as in [`from_session_in`](Self::from_session_in).
    pub unsafe fn open_multi_in(
        specs: &[SessionSpec<'_>],
        backing: &'s mut [MaybeUninit<u64>],
        sizing: super::storage::ExecutorSizing,
    ) -> Result<Self, NodeError> {
        // Phase 249 P4b.1 — backends self-registered via their
        // `.init_array` ctor before `main`; no runtime section walk.
        let primary = specs
            .first()
            .ok_or(NodeError::Transport(TransportError::ConnectionFailed))?;
        let primary_session =
            nros_rmw_cffi::CffiRmw::open_with_rmw(primary.rmw, &primary.to_rmw_config())
                .map_err(NodeError::Transport)?;
        // SAFETY: forwarded from this fn's contract.
        let mut executor = unsafe { Self::from_session_in(primary_session, backing, sizing) };
        executor.set_node_identity("", "/");
        // Phase 156 — see `Executor::open` for primary-identity
        // recording rationale.
        let _ = executor.primary_rmw_name.push_str(primary.rmw);
        let _ = executor.primary_locator.push_str(primary.locator);
        #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
        executor.install_wake_signal_on_primary();

        for spec in specs.iter().skip(1) {
            let session = nros_rmw_cffi::CffiRmw::open_with_rmw(spec.rmw, &spec.to_rmw_config())
                .map_err(NodeError::Transport)?;
            executor
                .extra_sessions
                .push(session)
                .map_err(|_| NodeError::NodeTableFull)?;
            // Issue 0436 — record WHICH backend/locator this extra is, so
            // `NodeBuilder::rmw(name)` can find it instead of opening a second
            // session against the same (singleton) backend.
            {
                let mut rmw_s = heapless::String::<32>::new();
                let _ = rmw_s.push_str(spec.rmw);
                let mut loc_s = heapless::String::<128>::new();
                let _ = loc_s.push_str(spec.locator);
                let _ = executor.extra_session_ids.push((rmw_s, loc_s));
            }
            #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
            {
                let idx = executor.extra_sessions.len() - 1;
                executor.install_wake_signal_on_extra(idx);
            }
        }

        Ok(executor)
    }

    /// Per-entry-sized [`open_with_rmw`](Self::open_with_rmw): carves `backing`
    /// for the executor's tables instead of leaking a default one.
    ///
    /// # Safety
    /// `backing`/`sizing` as in [`from_session_in`](Self::from_session_in).
    pub unsafe fn open_with_rmw_in(
        rmw_name: &str,
        config: &ExecutorConfig<'_>,
        backing: &'s mut [MaybeUninit<u64>],
        sizing: super::storage::ExecutorSizing,
    ) -> Result<Self, NodeError> {
        if !nros_rmw_cffi::backend_registered() {
            return Err(NodeError::Transport(TransportError::ConnectionFailed));
        }

        let rmw_config = nros_rmw::RmwConfig {
            locator: config.locator,
            mode: config.mode,
            domain_id: config.domain_id,
            node_name: config.node_name,
            namespace: config.namespace,
            properties: &[],
        };
        let session = nros_rmw_cffi::CffiRmw::open_with_rmw(rmw_name, &rmw_config)
            .map_err(|_| NodeError::Transport(TransportError::ConnectionFailed))?;
        // SAFETY: forwarded from this fn's contract.
        let mut executor = unsafe { Self::from_session_in(session, backing, sizing) };
        {
            // `config.clock_us` overrides the constructor's platform default
            // (issue: assigning `None` here clobbered it and re-enabled the
            // credit-the-requested-timeout fallback).
            if let Some(clock) = config.clock_us {
                executor.clock_us_fn = Some(clock);
                executor.last_spin_end_us = Some(clock());
            }
            // issue 0671 — the SAME rule as `clock_us` directly above, which is
            // why that one is guarded: a config that does not SPECIFY an epoch
            // must not be read as "this target HAS no epoch". Assigning `None`
            // here clobbered the constructor's platform default
            // (`Some(default_epoch_us)`), and `ExecutorConfig::new` — the path
            // `nros::init_*` + `ctx.config()` takes — leaves `epoch_us: None`,
            // so EVERY hosted node built that way silently lost its wall clock.
            // With no epoch, `Node::subscription` never attaches the age cell
            // (it needs `(STAMP_OFFSET, epoch)` both `Some`), so a baked
            // `max_age_ms` contract becomes a silently-dead monitor — the exact
            // outcome RFC-0052 says must never happen. The rate monitor rides
            // the GUARDED `clock_us_fn` and kept working, which is why only the
            // age half went quiet.
            if let Some(epoch) = config.epoch_us {
                executor.epoch_us_fn = Some(epoch);
            }
        }
        executor.set_node_identity(config.node_name, config.namespace);
        // Phase 156 — record primary identity for the session-
        // cache hit path. See `Executor::open` for the rationale.
        let _ = executor.primary_rmw_name.push_str(rmw_name);
        let _ = executor.primary_locator.push_str(config.locator);
        #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
        executor.install_wake_signal_on_primary();
        Ok(executor)
    }
}

/// Phase 128.F.1 — per-backend session declaration for
/// [`Executor::open_multi`]. Each spec names an RMW backend (must
/// match one a backend registered under via
/// `nros_rmw_cffi_register_named` / the `RMW_INIT_ENTRIES` linker
/// section) and the locator + domain id to open against it.
#[cfg(feature = "rmw-cffi")]
#[derive(Clone, Copy)]
pub struct SessionSpec<'cfg> {
    pub rmw: &'cfg str,
    pub locator: &'cfg str,
    pub domain_id: u32,
    pub node_name: &'cfg str,
    pub namespace: &'cfg str,
}

#[cfg(feature = "rmw-cffi")]
impl<'cfg> SessionSpec<'cfg> {
    /// Minimal spec — just RMW name + locator. Domain id defaults to
    /// 0; node name and namespace are empty.
    pub const fn new(rmw: &'cfg str, locator: &'cfg str) -> Self {
        Self {
            rmw,
            locator,
            domain_id: 0,
            node_name: "",
            namespace: "",
        }
    }

    pub const fn domain_id(mut self, domain_id: u32) -> Self {
        self.domain_id = domain_id;
        self
    }

    pub const fn node_name(mut self, name: &'cfg str) -> Self {
        self.node_name = name;
        self
    }

    pub const fn namespace(mut self, ns: &'cfg str) -> Self {
        self.namespace = ns;
        self
    }

    fn to_rmw_config(self) -> nros_rmw::RmwConfig<'cfg> {
        nros_rmw::RmwConfig {
            locator: self.locator,
            mode: nros_rmw::SessionMode::Client,
            domain_id: self.domain_id,
            node_name: self.node_name,
            namespace: self.namespace,
            properties: &[],
        }
    }
}

// Phase 128.A.3 — selector for the single-backend resolution path.
//
// On hosted (`std`) builds, read `$NROS_RMW`; mirrors ROS 2's
// `RMW_IMPLEMENTATION`. Returns the name as a byte vector so the
// caller can pass it to `nros_rmw_cffi::resolve_backend` and (when
// `Some`) to `CffiRmw::open_with_rmw`.
//
// On `no_std` / bare-metal builds, environment variables are not
// available; resolution always falls through to the single-backend
// or ambiguous path. Embedded users with multiple backends use the
// bridge surface `Executor::open_multi` instead.
// phase-359 W10 / issue 0687 — the private reader that stood here is gone, and
// so is the shared one that briefly replaced it. `ExecutorConfig::rmw` carries
// the selection instead: `nros::rmw_selector` reads the variable ONCE, at the
// hosted edge, and every consumer takes the value.

// ============================================================================
// SessionStore — owned or borrowed session
// ============================================================================

/// Session storage: owned or borrowed via raw pointer.
///
/// The C API creates a session in `nros_support_init()` before the
/// executor. `Borrowed` lets the executor use that session without owning it.
#[allow(clippy::large_enum_variant)]
pub(crate) enum SessionStore {
    Owned(session::ConcreteSession),
    Borrowed(*mut session::ConcreteSession),
}

impl core::ops::Deref for SessionStore {
    type Target = session::ConcreteSession;
    fn deref(&self) -> &session::ConcreteSession {
        match self {
            SessionStore::Owned(s) => s,
            SessionStore::Borrowed(ptr) => unsafe { &**ptr },
        }
    }
}

impl core::ops::DerefMut for SessionStore {
    fn deref_mut(&mut self) -> &mut session::ConcreteSession {
        match self {
            SessionStore::Owned(s) => s,
            SessionStore::Borrowed(ptr) => unsafe { &mut **ptr },
        }
    }
}

/// Phase 228.E — an opaque, `Send` handle to an [`Executor`]'s RMW session.
///
/// In the per-tier model the boot executor opens the one session and hands each
/// spawned tier task a handle (not a borrow) so the task opens its own
/// [`Executor`] over that *same* session across the RTOS task boundary. Wrapping
/// the `pub(crate)` session pointer lets board crates (`nros-board-linux`,
/// `nros-board-freertos`, …) name + move the handle without naming the session
/// type. Obtain via [`Executor::session_handle`]; consume via
/// [`Executor::open_with_session_handle`].
#[cfg(any(has_rmw, test))]
pub struct SessionHandle(*mut session::ConcreteSession);

// SAFETY: the per-tier model deliberately shares one session across RTOS tasks;
// concurrent access is serialized by the RMW backend's internal locks (the RTOS
// targets build zenoh-pico `Z_FEATURE_MULTI_THREAD=1` — RFC-0032 §5.0). The
// boot executor owns the session and outlives every tier task.
#[cfg(any(has_rmw, test))]
unsafe impl Send for SessionHandle {}

#[cfg(any(has_rmw, test))]
impl SessionHandle {
    /// Phase 274.W1 — convert to an opaque `*mut c_void` for C/C++ FFI.
    ///
    /// The returned pointer encodes the session address and is valid as long as
    /// the owning executor lives. Reconstruct via [`Self::from_raw`].
    pub fn into_raw(self) -> *mut core::ffi::c_void {
        self.0 as *mut core::ffi::c_void
    }

    /// Phase 274.W1 — reconstruct a `SessionHandle` from an opaque pointer
    /// returned by [`Self::into_raw`].
    ///
    /// # Safety
    /// `ptr` must be a pointer obtained from `into_raw()` on a `SessionHandle`
    /// whose underlying session is still live and owned by its original executor.
    pub unsafe fn from_raw(ptr: *mut core::ffi::c_void) -> Self {
        Self(ptr as *mut session::ConcreteSession)
    }
}

/// Phase 228.C — pure callback-group filter decision. `None` = wildcard (accept
/// every group); `Some` = accept only listed groups. Backs
/// [`Executor::group_active`]; split out so the logic is unit-testable without a
/// live session.
pub(crate) fn group_filter_accepts<const N: usize>(
    active: Option<&[heapless::String<N>]>,
    group: &str,
) -> bool {
    match active {
        None => true,
        Some(v) => v.iter().any(|g| g.as_str() == group),
    }
}

#[cfg(test)]
mod group_filter_tests {
    use super::group_filter_accepts;

    type Group = heapless::String<32>;

    fn group(s: &str) -> Group {
        let mut g = Group::new();
        g.push_str(s).unwrap();
        g
    }

    #[test]
    fn wildcard_accepts_all() {
        assert!(group_filter_accepts::<32>(None, "anything"));
    }

    #[test]
    fn set_accepts_only_listed_groups() {
        let active = [group("ctrl")];
        assert!(group_filter_accepts(Some(&active[..]), "ctrl"));
        assert!(!group_filter_accepts(Some(&active[..]), "telem"));
    }

    /// phase-409 — the wildcard and an EMPTY filter are different answers, and
    /// the carved table cannot tell them apart on its own (the old
    /// `Option<Vec>` could). `Executor::active_groups_filtering` is what keeps
    /// them apart; this pins the distinction at the decision itself.
    #[test]
    fn an_empty_filter_is_not_the_wildcard() {
        let empty: [Group; 0] = [];
        assert!(!group_filter_accepts(Some(&empty[..]), "anything"));
        assert!(group_filter_accepts::<32>(None, "anything"));
    }
}

// ============================================================================
// Executor
// ============================================================================

/// Backend-agnostic executor that owns a session.
///
/// Provides `create_node()` for entity creation and `drive_io()` for polling.
///
/// # Callback Mode
///
/// The executor supports arena-based callback registration via the
/// `node_mut(id).subscription(t)` builder and
/// [`register_service()`](Self::register_service), with dispatch via
/// [`spin_once()`](Self::spin_once). No heap allocation is needed.
///
/// The sizes are set via `NROS_EXECUTOR_MAX_CBS` (default 4) and
/// `NROS_EXECUTOR_ARENA_SIZE` (default 4096) environment variables at build time.
///
/// Phase 124.B.2 — opaque context handed to the runtime wake
/// callback. Backends store the raw pointer + invoke the callback;
/// the callback decodes back to `&WakeCtx`.
#[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
pub(crate) struct WakeCtx {
    pub(crate) flag: portable_atomic_util::Arc<portable_atomic::AtomicBool>,
    /// The wake primitive. Phase 130.3 added it beside a `std::sync::Condvar`
    /// pair and noted that "a future migration to a single primitive flips one
    /// branch instead of two"; phase-359 W10 is that migration, and this is the
    /// one that survived.
    pub(crate) node_wake: Option<portable_atomic_util::Arc<super::node_wake::NodeWake>>,
}

/// Phase 124.B.2 — runtime wake callback.
///
/// RT-context contract:
///
/// * **Thread-safe**: callable from any thread. The cb is lock-free
///   on the cv path — no mutex held during `notify_all`. Lost-wakeup
///   is prevented by the waiter checking `wake_flag` under
///   `wake_mu` via the `wait_timeout_while` predicate.
/// * **NOT async-signal-safe on POSIX**: `pthread_cond_signal`
///   isn't on the POSIX async-signal-safe function list. For POSIX
///   signal handler wake, use a `signalfd` + select pattern in a
///   thread that owns the wake duty — that pattern is Linux-only
///   (`signalfd`/`eventfd` are Linux syscalls, not POSIX), which is
///   why the `WakeSignalFd` worker is gated `target_os = "linux"`.
/// * **RTOS ISR**: per-RTOS platform layer wraps the cv with an
///   ISR-safe primitive (`xSemaphoreGiveFromISR`,
///   `tx_event_flags_set` from ISR, `k_sem_give` from ISR on
///   Zephyr). Backend's ISR caller routes through the platform's
///   `signal_from_isr` API instead of this cb directly.
/// * **Bounded execution time**: O(1) — atomic store + cv notify.
///   No allocation, no contended lock.
///
/// The cb is the symbol backends invoke from their async wake path
/// (datagram arrival, worker-thread enqueue, etc.). It does
/// flag-write + condvar-signal in that order, lock-free.
#[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
pub(crate) unsafe extern "C" fn nros_rmw_runtime_wake_cb(ctx: *mut core::ffi::c_void) {
    if ctx.is_null() {
        return;
    }
    // Phase 141.B.2 — capture T0 at cb entry. No-op when the
    // probe feature is off or no cycle reader is installed.
    #[cfg(feature = "wake-latency-probe")]
    super::wake_probe::on_wake();
    // SAFETY: ctx points at a `WakeCtx` owned by an Executor still
    // alive at the time of the call. Executor::drop must clear the
    // callback via `set_wake_callback(None, _)` on all sessions
    // before dropping wake_ctx; this happens in `install_wake_*`
    // teardown path.
    let wake = unsafe { &*(ctx as *const WakeCtx) };
    // Lock-free: the `flag` store is SeqCst and therefore happens-before any
    // subsequent acquire in the waiter, so a wake cannot be missed even though
    // nothing is held here.
    wake.flag.store(true, core::sync::atomic::Ordering::SeqCst);
    // phase-359 W10 — ONE primitive. This used to signal both a
    // `std::sync::Condvar` and (since phase 130.3) the `NodeWake`, "so the cb
    // keeps working whichever wait primitive spin_once is using". `spin_once`
    // now has only one, so this has only one to signal.
    if let Some(nw) = wake.node_wake.as_ref() {
        nw.signal();
    }
}

/// Phase 124.B.7.c — Linux signalfd worker.
///
/// Owns a Linux `eventfd` plus a worker task that `read()`s the
/// fd and forwards it as a wake. The eventfd
/// write side is async-signal-safe per the kernel contract
/// (`write(2)` to an eventfd is permitted from signal handlers),
/// closing the gap that `pthread_cond_signal` leaves open on POSIX.
///
/// Lifecycle:
///   * Constructed lazily in `Executor::signal_fd()` on first
///     caller request.
///   * `Drop` writes a shutdown sentinel + joins the worker.
///
/// Caller flow (signal handler):
///   1. Get fd via `Executor::signal_fd()` before installing the
///      handler.
///   2. Handler does `eventfd_write(fd, 1)` (equivalently,
///      `write(fd, &1u64, 8)`).
///   3. Worker thread reads the fd, signals wake_cv. spin_once
///      blocked in cv.wait_timeout_while sees flag=true and exits.
///
/// phase-359 W10 — the worker is a PLATFORM TASK, not a `std::thread`, and it
/// forwards through the same [`NodeWake`](super::node_wake::NodeWake) the
/// runtime wake callback uses rather than a `std::sync::Condvar`. The eventfd
/// itself is still Linux — that is what makes the write async-signal-safe — so
/// this stays `target_os = "linux"`; what it no longer is, is std-only.
#[cfg(all(feature = "signal-fd-wake", target_os = "linux"))]
pub struct WakeSignalFd {
    fd: core::ffi::c_int,
    ctx: portable_atomic_util::Arc<SignalFdCtx>,
    task: Option<nros_platform_api::task::PlatformTask>,
}

/// What the signalfd worker task needs, reachable through one pointer.
#[cfg(all(feature = "signal-fd-wake", target_os = "linux"))]
struct SignalFdCtx {
    fd: core::ffi::c_int,
    shutdown: portable_atomic::AtomicBool,
    /// The executor's wake state — the same `WakeCtx` the runtime callback
    /// decodes, reached as an address so the context is plainly `Send`.
    wake_ctx: usize,
}

/// Worker entry: read the eventfd, forward as a wake, until shut down.
///
/// # Safety
/// `arg` must point at a live [`SignalFdCtx`] whose `wake_ctx` addresses a
/// `WakeCtx` that outlives this task.
#[cfg(all(feature = "signal-fd-wake", target_os = "linux"))]
unsafe extern "C" fn signal_fd_worker(arg: *mut core::ffi::c_void) -> *mut core::ffi::c_void {
    // SAFETY: the spawn site passes `Arc::as_ptr` of a ctx it keeps alive until
    // after the join in `Drop`.
    let ctx = unsafe { &*(arg as *const SignalFdCtx) };
    loop {
        let mut buf = [0u8; 8];
        // SAFETY: reading 8 bytes from an eventfd into an 8-byte buffer.
        let n = unsafe { libc::read(ctx.fd, buf.as_mut_ptr() as *mut core::ffi::c_void, 8) };
        if ctx.shutdown.load(portable_atomic::Ordering::Acquire) {
            return core::ptr::null_mut();
        }
        if n <= 0 {
            // EINTR / EOF — shutdown was re-checked above, so loop.
            continue;
        }
        // Same effect as `nros_rmw_runtime_wake_cb`, reached directly because
        // the shutdown check above plus `Drop`'s join is what guarantees the
        // `WakeCtx` is still alive.
        // SAFETY: `wake_ctx` addresses the executor's `WakeCtx`, which outlives
        // this task by that same guarantee.
        unsafe {
            let w = &*(ctx.wake_ctx as *const WakeCtx);
            w.flag.store(true, portable_atomic::Ordering::SeqCst);
            if let Some(wake) = w.node_wake.as_ref() {
                wake.signal();
            }
        }
    }
}

#[cfg(all(feature = "signal-fd-wake", target_os = "linux"))]
impl WakeSignalFd {
    /// Spawn the worker. `wake_ctx_ptr` is the `*const WakeCtx`
    /// produced by `Executor::wake_ctx_ptr` — same value the
    /// runtime wake cb decodes.
    ///
    /// phase-359 W10 — returns `NodeError` rather than `std::io::Error`. The
    /// only caller is `Executor::signal_fd`, and the errno detail it used to
    /// carry had no consumer: the one failure a caller can act on is "this
    /// platform would not give me the worker", which is what
    /// `NotInitialized` says — the eventfd and the task are both subsystems
    /// this capability requires and neither is guaranteed.
    fn new(wake_ctx_ptr: *const WakeCtx) -> Result<Self, NodeError> {
        // SAFETY: `eventfd(2)` with a valid flag; returns -1 on failure.
        let fd = unsafe { libc::eventfd(0, libc::EFD_CLOEXEC) };
        if fd < 0 {
            return Err(NodeError::NotInitialized);
        }
        let ctx = portable_atomic_util::Arc::new(SignalFdCtx {
            fd,
            shutdown: portable_atomic::AtomicBool::new(false),
            wake_ctx: wake_ctx_ptr as usize,
        });
        let arg = portable_atomic_util::Arc::as_ptr(&ctx) as *mut core::ffi::c_void;
        // SAFETY: `arg` points at a ctx this struct owns and keeps alive until
        // after the join in `Drop`.
        let Some(task) = (unsafe {
            nros_platform_api::task::PlatformTask::spawn(
                signal_fd_worker,
                arg,
                // A read(2) loop and one atomic store — the smallest stack any
                // port will honour is plenty.
                8192,
                c"nros-wakefd".as_ptr(),
            )
        }) else {
            // SAFETY: nothing else holds the fd — the task never started.
            unsafe { libc::close(fd) };
            return Err(NodeError::NotInitialized);
        };
        Ok(Self {
            fd,
            ctx,
            task: Some(task),
        })
    }

    /// Returns the writable eventfd. The caller (typically a POSIX
    /// signal handler) writes any non-zero 8-byte value to trigger
    /// a wake. `write(2)` on an eventfd is async-signal-safe per
    /// `eventfd(2)` man page.
    pub fn fd(&self) -> core::ffi::c_int {
        self.fd
    }
}

#[cfg(all(feature = "signal-fd-wake", target_os = "linux"))]
impl Drop for WakeSignalFd {
    fn drop(&mut self) {
        self.ctx
            .shutdown
            .store(true, portable_atomic::Ordering::Release);
        // Wake the worker so it re-checks shutdown: it is blocked in `read`,
        // which only this write can release.
        let one: u64 = 1;
        // SAFETY: an 8-byte write to our own eventfd.
        unsafe {
            libc::write(self.fd, &one as *const u64 as *const core::ffi::c_void, 8);
        }
        if let Some(task) = self.task.take() {
            task.join();
        }
        // SAFETY: the worker has exited, so nothing else touches the fd.
        unsafe { libc::close(self.fd) };
    }
}

/// Phase 124.B.7.b — ISR / interrupt-context wake callback.
///
/// Same semantics as [`nros_rmw_runtime_wake_cb`] but constrained to
/// async-signal-safe / ISR-safe primitives.
///
/// Per-platform routing:
///
/// * **POSIX (std)**: `pthread_cond_signal` is NOT on the POSIX
///   async-signal-safe function list. Calling from a SIGUSR1
///   handler is technically UB. Real fix (Phase 124.B.7.c) routes
///   via `signalfd`/`eventfd` + a runtime worker thread — Linux
///   only, since neither syscall is POSIX; until that lands,
///   signal-handler callers MUST use
///   `nros_guard_condition_trigger` from a **separate thread** (not
///   from the handler itself), OR set the wake_flag and rely on
///   the next poll deadline. This cb currently aliases the regular
///   `wake_cb` and is safe only from non-signal-handler ISR-like
///   contexts (e.g. timer thread, kernel callback).
///
/// * **RTOS no_std (Zephyr/FreeRTOS/ThreadX)**: routes through the
///   platform-cffi `condvar_signal_from_isr` slot. Each backend
///   uses its ISR-safe variant — `xSemaphoreGiveFromISR`,
///   `tx_semaphore_put`, `k_condvar_signal`.
///
/// `ctx` semantics identical to [`nros_rmw_runtime_wake_cb`].
#[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
#[allow(dead_code)] // Public exposure pending B.7.c signalfd worker.
pub(crate) unsafe extern "C" fn nros_rmw_runtime_wake_cb_from_isr(ctx: *mut core::ffi::c_void) {
    // Today: alias regular wake_cb. POSIX signal-handler safety
    // pending B.7.c (signalfd worker-thread forward). Documented in
    // the contract above so callers know the boundary.
    unsafe { nros_rmw_runtime_wake_cb(ctx) };
}

/// Phase 216 follow-up — per-Node dispatch trampoline registered with
/// [`Executor::register_dispatch_slot`].
///
/// The board-side dispatch task (RTIC `__nros_run` / Embassy
/// `__nros_run_task`) dequeues a `nros_platform::SignaledCallback`
/// envelope and forwards `(cb_id, ctx_ptr)` into
/// [`Executor::dispatch_callback`]; that method linear-scans this
/// slot table and invokes every registered `on_callback` with the
/// owning Node's per-Node `state` blob. Each Node's
/// `__nros_node_<pkg>_on_callback` self-filters on its own
/// `CallbackId` tag set, so a slot whose Node doesn't own this
/// callback is a cheap no-op string compare.
///
/// The shape mirrors the per-pkg `__nros_node_<pkg>_on_callback`
/// extern "C" trampoline emitted by the `nros::node!()` macro
/// (see `packages/core/nros-macros/src/lib.rs` Phase 216.A.5).
///
/// # Why not `linkme`
///
/// `linkme::distributed_slice` hangs on bare-metal Cortex-M /
/// RISC-V because `cortex_m_rt`'s link script doesn't provide the
/// `__start_/__stop_` section anchors in a shape that lets the
/// iterator terminate (see
/// `packages/rmw/cffi/src/section.rs` Phase 142). Since
/// stm32f4 RTIC / Embassy boards are the whole point of Phase 216,
/// the registry uses the explicit `register()` pattern from Phase
/// 104.A.
#[derive(Clone, Copy)]
pub struct DispatchSlot {
    /// Owning Node's `State` blob — produced by the macro-emitted
    /// `i()` and round-tripped through
    /// `nros::__private_node_state_into_raw`. Opaque to the
    /// executor.
    pub state: *mut core::ffi::c_void,
    /// Per-Node `extern "C"` trampoline; signature matches the
    /// `__nros_node_<pkg>_on_callback` symbol the `nros::node!()`
    /// macro emits.
    pub on_callback: unsafe extern "C" fn(
        state: *mut core::ffi::c_void,
        cb_id_ptr: *const u8,
        cb_id_len: usize,
        ctx: *mut core::ffi::c_void,
    ),
}

// SAFETY: `DispatchSlot` carries two raw pointers (`state` + an
// extern "C" fn pointer). The fn pointer is `Send`/`Sync` by
// definition; the `state` pointer's `Send`/`Sync` story matches the
// owning `Executor` (which is `unsafe impl Send`). Treating the
// slot itself as `Send` keeps the existing `Executor` Send impl
// intact — see `unsafe impl Send for Executor {}` later in this
// file.
unsafe impl Send for DispatchSlot {}
unsafe impl Sync for DispatchSlot {}

/// Phase 258 (Track 2, 2a) — executor-owned component tick slot.
///
/// The layering-clean half of the W0-B `install` seam's tick fix
/// (phase-257 D2). A `nros`-layer `install`/`register_node_borrowed`
/// builds an `Arc<ComponentCell>` (the typed/poll-driven component
/// state) and enrolls it here via [`Executor::enroll_component`]; the
/// executor then drives `tick` on every enrolled slot at the tail of
/// each [`spin_once`](Executor::spin_once) — so `install`'d nodes
/// (C, C++, **and Rust owned-spin**) tick, closing the
/// service-client/action poll gap that the callback-`Arc`-only
/// lifetime left open.
///
/// Like [`DispatchSlot`] the executor only sees raw pointers + `extern
/// "C"` fn pointers (no `nros` dep — `nros-node` is the lower layer):
///
/// * `state` — a *leaked* `Arc<ComponentCell>` (via `Arc::into_raw`),
///   re-borrowed by the `nros`-side `tick`/`drop` fns. Unlike a
///   pub/sub/timer component (kept alive by the executor's per-entity
///   callback `Arc` clones), a poll-only component has no callbacks, so
///   the slot must own a clone of the cell — hence the paired `drop`.
/// * `tick` — `nros`-side `extern "C"` fn that casts `state` back to
///   `&ComponentCell`, casts `exec_ctx` back to `*mut Executor`, and
///   runs that one cell's tick (mirrors `ExecutorNodeRuntime::run_ticks`).
/// * `drop` — `nros`-side `extern "C"` fn run on `Executor::drop` that
///   reconstitutes + drops the leaked `Arc`, so the executor owns the
///   cell's lifetime.
///
/// Kept a SEPARATE registry from [`DispatchSlot`] on purpose: framework
/// dispatch (RTIC / Embassy) is interrupt-driven, name-keyed, and has no
/// tick/own concern — mixing the two risks that path.
#[derive(Clone, Copy)]
pub struct ComponentSlot {
    /// Leaked `Arc<ComponentCell>` (opaque to the executor). Owned by
    /// this slot — dropped via `drop` on `Executor::drop`.
    pub state: *mut core::ffi::c_void,
    /// `nros`-side tick trampoline: `(state, exec_ctx)` where `exec_ctx`
    /// is `*mut Executor`. Drives one component's `tick`.
    pub tick: unsafe extern "C" fn(state: *mut core::ffi::c_void, exec_ctx: *mut core::ffi::c_void),
    /// `nros`-side drop trampoline: reconstitutes + drops the leaked
    /// `Arc<ComponentCell>` at `state`. Run once on `Executor::drop`.
    pub drop: unsafe extern "C" fn(state: *mut core::ffi::c_void),
}

// SAFETY: same story as `DispatchSlot` — two raw pointers + two extern
// "C" fn pointers. The `state` pointer's Send/Sync matches the owning
// `Executor` (`unsafe impl Send for Executor`); the fn pointers are
// Send/Sync by definition.
unsafe impl Send for ComponentSlot {}
unsafe impl Sync for ComponentSlot {}

/// phase-271 — fixed capacity of the per-spin ready-sets (FIFO bitmap + EDF
/// heap) and the dispatch loop's upper bound. The executor's callback index is
/// carried in a `u64` active-mask (`1u64 << i`), so a callback table can hold at
/// most 64 entries regardless of per-entry sizing; the ready-sets are sized to
/// this ceiling (stack-transient) so any entry slice up to 64 dispatches
/// correctly. `EdfReadySet`'s presence bitmap independently asserts `N <= 64`.
pub(crate) const MAX_CALLBACK_SLOTS: usize = 64;

/// Phase 305 W3 (issue 0255) — upper bound on launch remap rules held by one
/// executor (across all its nodes). Launch files carry a handful per node;
/// raise here if a plan legitimately outgrows it.
pub const MAX_REMAPS: usize = 16;

/// Phase 305 W3 (issue 0255) — one launch `<remap from= to=/>` rule, scoped to
/// the node that declared it. `from`/`to` are stored RAW (as written); the
/// lookup in [`Executor::resolve_entity_name_for`] expands both sides against
/// the owning node's identity via `crate::names` (exact-FQN match, no
/// wildcards).
pub(crate) struct RemapRule {
    pub(crate) node_name: heapless::String<64>,
    pub(crate) namespace: heapless::String<64>,
    pub(crate) from: heapless::String<{ crate::names::MAX_RESOLVED_NAME_LEN }>,
    pub(crate) to: heapless::String<{ crate::names::MAX_RESOLVED_NAME_LEN }>,
}

pub struct Executor<'s> {
    /// Issue 0656 — the ROS domain this executor's entities belong to.
    ///
    /// Retained because it was NOT: `open_in` passed `config.domain_id` into
    /// `RmwConfig` and dropped it, so every entity built from the executor
    /// (rather than from a `Node`, which keeps its own) declared on domain 0
    /// whatever `ROS_DOMAIN_ID` said. The value was read, printed, and then not
    /// used where it counts — issue 0161's shape.
    pub(crate) domain_id: u32,
    pub(crate) session: SessionStore,
    /// phase-271 (issue 0110) — the six sized tables are no longer inline
    /// arrays baked to `nros-node`'s build-time consts; they borrow
    /// caller-owned, per-entry-sized storage (`&'s mut` slices carved from a
    /// raw `[MaybeUninit<u64>]` backing by [`super::storage::carve`]). Lets a
    /// fat native entry and a lean embedded entry in one shared-target
    /// workspace each size to its own topology. `Executor` stays non-generic
    /// (lifetime only) so the C/C++ FFI keeps wrapping one concrete type.
    pub(crate) arena: &'s mut [MaybeUninit<u8>],
    pub(crate) arena_used: usize,
    pub(crate) entries: &'s mut [Option<CallbackMeta>],
    /// Phase 110.B — registered scheduling contexts. Slot 0 is
    /// auto-populated with a `Fifo` SC at construction; every entry
    /// without an explicit binding maps to it via
    /// `sched_context_bindings`.
    pub(crate) sched_contexts: &'s mut [Option<super::sched_context::SchedContext>],
    /// Per-entry SC binding parallel to `entries`. Defaults to
    /// `SchedContextId(0)` (the auto-created Fifo SC).
    pub(crate) sched_context_bindings: &'s mut [super::sched_context::SchedContextId],
    /// Phase 110.E — user-space sporadic-server budget state per
    /// Sporadic-class SC. Slot indices match `sched_contexts`; non-
    /// Sporadic slots stay `None`.
    pub(crate) sporadic_states: &'s mut [Option<super::sched_context::SporadicState>],
    /// Phase 110.E.b — atomic sporadic state + opaque platform-timer
    /// handle for ISR-driven refill. Populated by
    /// `register_sporadic_timer`; dropped on Executor `Drop` via the
    /// stored `destroy_fn`.
    #[cfg(feature = "alloc")]
    pub(crate) sporadic_atomic_states: &'s mut [Option<(
        portable_atomic_util::Arc<super::sched_context::AtomicSporadicState>,
        OpaqueTimerHandle,
    )>],
    /// Phase 110.G — major-frame length for time-triggered dispatch.
    /// `0` (default) disables the TT gate entirely; non-zero enables
    /// gating per
    /// `SchedContext.tt_window_offset_us / tt_window_duration_us`.
    pub(crate) major_frame_us: u32,
    /// Phase 110.F — per-OS-priority worker pool. Lazily populated
    /// on first dispatch routing to a non-zero `os_pri`.
    ///
    /// phase-359 W10 — was `std::collections::HashMap<u8, OsPriorityWorker>`
    /// behind `feature = "std"`, because the workers were `std::thread` +
    /// `mpsc`. They are platform tasks now (`super::os_priority`), so the pool
    /// needs only `alloc` and this capability is no longer std-only.
    #[cfg(all(
        feature = "alloc",
        feature = "rmw-cffi",
        feature = "scheduler-os-priority"
    ))]
    pub(crate) os_priority_pool: super::os_priority::OsPriorityPool,
    /// Phase 110.F — caller-supplied `apply_policy` function pointer
    /// each worker invokes at startup to elevate its OS priority.
    /// `None` = the worker pool is disabled; entries bound to non-
    /// zero `os_pri` SCs fall back to the cooperative path.
    /// Mirrors `Executor::open_threaded`'s `apply_policy: fn(...)`
    /// shape — keeps Executor non-generic over Platform.
    // phase-359 W10 — the POLICY is separable from the POOL. Registering it
    // needs nothing but the feature; hosting workers needs a platform
    // (`rmw-cffi`, the same proxy `node_wake` uses). A build that can register
    // but not host falls back to cooperative dispatch, so the public
    // `register_os_priority_dispatcher` keeps its old availability.
    #[cfg(all(feature = "alloc", feature = "scheduler-os-priority"))]
    pub(crate) os_priority_apply_policy:
        Option<fn(nros_platform_api::SchedPolicy) -> Result<(), nros_platform_api::SchedError>>,
    pub(crate) trigger: Trigger,
    pub(crate) semantics: ExecutorSemantics,
    /// Node name for entities created via `register_subscription`/`register_service`.
    /// Empty means unset — no liveliness tokens will be declared.
    pub(crate) node_name: heapless::String<64>,
    /// Phase 228.C — per-tier callback-group filter. Wildcard (register every
    /// callback — the single-tier degenerate case + today's behaviour) until
    /// [`set_active_groups`](Self::set_active_groups) names a non-empty set;
    /// after that this tier's executor accepts only callbacks whose
    /// `.callback_group()` is in it, and skips the others at registration.
    ///
    /// phase-409 — CARVED, and the wildcard is a separate flag rather than an
    /// `Option` around the table, because the table itself no longer lives in
    /// the value. "Filtering with an empty set" and "not filtering" stay
    /// distinguishable, which is the whole content of the old `Option`.
    pub(crate) active_groups: super::storage::CarvedVec<'s, super::storage::GroupName>,
    /// Whether [`active_groups`](Self::active_groups) is a filter at all.
    /// `false` = wildcard (the old `None`).
    pub(crate) active_groups_filtering: bool,
    /// Node namespace (default: "/").
    pub(crate) namespace: heapless::String<64>,
    /// Phase 104.C.2 — rclcpp-style `add_node` table. Holds the
    /// per-Node metadata (name, namespace, rmw, locator, default
    /// SchedContext) for every Node attached to this Executor. The
    /// implicit "primary" Node (NodeId(0)) mirrors `node_name` +
    /// `namespace` above and is auto-populated on first use.
    ///
    /// phase-409 — CARVED. `NodeId` IS an index into this table, so the
    /// carved vector must keep push order and never gain a `swap_remove`.
    pub(crate) nodes: super::storage::CarvedVec<'s, super::node_record::NodeRecord>,
    /// Phase 272 (RFC-0047) — config-seeded node → sched-context bindings, keyed by the node's
    /// fully-qualified `(name, namespace)` pair. `NodeBuilder::build` consults this table to set a
    /// node's `default_sched` when no explicit `.sched()` was given. Empty ⇒ every node stays
    /// `SchedContextId(0)` (byte-identical to pre-272 behaviour). Sized by `MAX_NODES` (at most
    /// one tier per node — RFC-0047 OQ1).
    ///
    /// phase-409 — CARVED.
    pub(crate) node_sched_table: super::storage::CarvedVec<'s, super::storage::NodeSchedEntry>,
    /// Phase 273 (RFC-0047) — config-seeded per-callback-group sched bindings, keyed by the node's
    /// fully-qualified `(name, namespace)` pair PLUS the callback-group name. Overrides the node
    /// default for a callback created in that group. Empty ⇒ no per-group binding (node default
    /// stands). Sized by `MAX_CBS` — an upper bound on distinct callback-group bindings (you can
    /// never have more distinct group bindings than max callbacks).
    ///
    /// phase-409 (issue 0961) — CARVED, and it is the reason the phase exists:
    /// at ~168 B per slot this was `MAX_CBS` * 168 bytes INSIDE the value, so
    /// raising the handle limit from 14 to 36 — a fix for an unrelated failure —
    /// added ~3.7 KiB to the stack frame of every function that moves an
    /// `Executor`, and overflowed a 320 KiB part's main thread.
    pub(crate) group_sched_table: super::storage::CarvedVec<'s, super::storage::GroupSchedEntry>,
    /// Phase 305 W3 (issue 0255) — launch-baked per-node remap rules, keyed by
    /// the declaring node's `(name, namespace)` identity. Entries store the
    /// rule RAW (as written in launch); [`Self::resolve_entity_name`] expands
    /// both sides against the owning node's identity at lookup, via the shared
    /// `crate::names` seam (the same semantics the Rust `ExecutorSink` path
    /// applies). Declaration order is match order (first rule wins).
    /// Issue 0563 — CARVED, not inline. As a `heapless::Vec<RemapRule,
    /// MAX_REMAPS>` this one field was 6664 bytes of an 11632-byte `Executor`
    /// (57%), which is what made building an executor a ~9.3 KB stack
    /// temporary and overflowed the Zephyr Cortex-M main stack (issue 0552).
    /// It is the seventh sized table; phase-271 moved the other six here.
    /// `remap_len` is the fill cursor — occupied slots are `[0, remap_len)`.
    pub(crate) remap_table: &'s mut [Option<RemapRule>],
    pub(crate) remap_len: usize,
    /// Phase 216 follow-up — per-Node dispatch trampoline registry.
    ///
    /// Populated by [`Executor::register_dispatch_slot`]; walked by
    /// [`Executor::dispatch_callback`] each time the board-side
    /// dispatch task hands off a `SignaledCallback` envelope.
    /// Sized by `MAX_NODES` because the upper-bound is one slot per
    /// Node pkg deployed on this executor (the same upper bound used
    /// by `nodes` and `extra_sessions`). `MAX_NODES` is driven by the
    /// `NROS_EXECUTOR_MAX_NODES` build-script env var (default 4);
    /// boards that deploy more Node pkgs raise it at build time.
    ///
    /// Default is `heapless::Vec::new()` (empty) — Nodes register
    /// themselves explicitly via the `register_dispatch_slot` API.
    /// The fallback shape avoids the `linkme` hazard on bare-metal
    /// Cortex-M / RISC-V (see `DispatchSlot` doc).
    ///
    /// phase-409 — CARVED (sized by `ExecutorSizing::nodes`, which is what
    /// `MAX_NODES` now seeds).
    pub(crate) dispatch_slots: super::storage::CarvedVec<'s, DispatchSlot>,
    /// Phase 258 (Track 2, 2a) — executor-owned component tick registry.
    /// Enrolled by [`Executor::enroll_component`] (from `nros`'s
    /// `install`/`register_node_borrowed`); each slot's `tick` runs at the
    /// tail of [`spin_once`](Self::spin_once); each slot's `drop` runs on
    /// `Executor::drop`. Bounded `MAX_NODES` (matches `dispatch_slots` /
    /// `nodes`). See [`ComponentSlot`] for why it's separate from
    /// `dispatch_slots`.
    ///
    /// phase-409 — CARVED.
    pub(crate) component_slots: super::storage::CarvedVec<'s, ComponentSlot>,
    /// Phase 104.C.3 — extra sessions opened by `node_builder.rmw()`
    /// calls that named a backend different from the Executor's
    /// primary session. Indexed by `NodeRecord.session_idx`
    /// (1..=N maps to `extra_sessions[N-1]`; idx 0 is the primary
    /// `self.session`). Sized by `NROS_EXECUTOR_MAX_NODES` since one
    /// extra session per Node is the worst case.
    ///
    /// phase-409 — CARVED, and the biggest single win: `ConcreteSession` is
    /// 524 B on the island, so this table alone was ~3.1 KiB of the value.
    /// Declared AFTER `session` so field-order drop still closes the primary
    /// session before the extras (`CarvedVec` owns its elements' drop).
    pub(crate) extra_sessions: super::storage::CarvedVec<'s, session::ConcreteSession>,
    /// Issue 0436 — `(rmw_name, locator)` for each entry of `extra_sessions`,
    /// the extras' equivalent of `primary_rmw_name` / `primary_locator`.
    ///
    /// Without it an extra session is ANONYMOUS, and
    /// `NodeBuilder::resolve_session_slot` could only recognise one by finding a
    /// previously-registered `NodeRecord` bound to it. Sessions opened by
    /// `open_multi*` have no such Node yet, so the FIRST `.rmw("zenoh")` node fell
    /// through to "open a new session" — a SECOND zenoh session, with an empty
    /// locator, which fails (see `primary_rmw_name`'s note: zenoh-pico's global
    /// state is a process singleton). That is why the PX4 bridge could open both
    /// sessions and then fail to bind its outward Node.
    ///
    /// Written unconditionally by `open_multi*`, but the only READER is
    /// `NodeBuilder::resolve_session_slot`, which is `rmw-cffi`-gated — so
    /// without that feature the field is genuinely unread and the
    /// workspace's `-D dead_code` is right to say so. Allow it exactly
    /// there rather than blanket-allowing a field that must stay live in
    /// every configuration that can reach the reader.
    ///
    /// phase-409 — CARVED.
    #[cfg_attr(not(feature = "rmw-cffi"), allow(dead_code))]
    pub(crate) extra_session_ids: super::storage::CarvedVec<'s, super::storage::ExtraSessionId>,
    /// Phase 156 — primary session's rmw name + locator, captured
    /// at `open*` time so `NodeBuilder::resolve_session_slot`'s
    /// cache lookup can detect when a `.rmw(name).locator(loc)`
    /// matches the primary (slot 0) instead of falling through to
    /// `CffiRmw::open_with_rmw` and trying to open a SECOND
    /// session against the same backend. zenoh-pico's global state
    /// is a process singleton; opening twice fails. Empty when
    /// constructed via `from_session(_ptr)` without `open*`
    /// recording the metadata; in that case the cache check
    /// degrades to "always miss" (today's behaviour).
    pub(crate) primary_rmw_name: heapless::String<32>,
    pub(crate) primary_locator: heapless::String<128>,
    // phase-359 W3 — `portable_atomic_util::Arc`, not `std::sync::Arc`. Same
    // atomically-refcounted pointer, available without `std`, and it compiles on
    // std too. W3 left the GATE on `std` because the public `halt_flag()` getter
    // was std-gated; W10 split that impl, so the field joins `wake_flag` on
    // `alloc` — the allocator is the real requirement, and without this a no_std
    // executor had no halt flag at all and so could not be stopped.
    #[cfg(feature = "alloc")]
    pub(crate) halt_flag: portable_atomic_util::Arc<portable_atomic::AtomicBool>,
    /// Phase 104.C.6 — shared executor wake flag. Any source of work
    /// (foreign thread handing off a callback, signal handler, future
    /// per-session vtable wake hook) sets this; `spin_once` swaps it to
    /// `false` on entry and, if it was `true`, polls every session with
    /// a 0-ms timeout instead of blocking. Lets one notification wake
    /// the executor regardless of which session the user is currently
    /// blocked on (the multi-RMW bridge case).
    // phase-359 W3 — ONE wake flag, both flavours. W2 had to leave this pair
    // alone because `wake_handle()` hands it out as a `std::sync::Arc`;
    // converting that signature is what lets the two collapse. Gated on
    // `alloc` (which `std` implies) because the Arc needs an allocator.
    #[cfg(feature = "alloc")]
    pub(crate) wake_flag: portable_atomic_util::Arc<portable_atomic::AtomicBool>,
    /// Phase 124.B.2 — wake condvar paired with `wake_flag`. The
    /// runtime-supplied wake callback (`nros_rmw_runtime_wake_cb` in
    /// nros-rmw-cffi) writes `wake_flag = true` AND signals
    /// `wake_cv` atomically under `wake_mu`. `spin_once` blocks on
    /// the cv with a deadline instead of calling `drive_io` with the
    /// user's timeout — sub-poll-period wake latency.
    ///
    /// Poll-only backends (NULL `set_wake_callback` slot) leave the
    /// cb uninstalled; the cv wait still fires on its deadline,
    /// then drive_io(0) drains whatever the backend's internal
    /// poll has buffered.
    /// Phase 130.3 — Zephyr+std uses `nros_platform_wake_*` (k_sem)
    /// instead of `std::sync::Condvar` because Zephyr's libc
    /// `pthread_cond_timedwait` hangs past its deadline. `None`
    /// when the platform provider didn't link a wake primitive
    /// (e.g. test builds with `rmw-cffi` but no `platform-*`
    /// feature); spin_once falls back to driving the transport
    /// for the full timeout in that case.
    // phase-359 W2 — ONE field, not one per flavour. `portable_atomic_util::Arc`
    // compiles on std too, and `std` implies `alloc`, so the std and alloc arms
    // were two spellings of the same thing. The inner `NodeWake` was already
    // shared; only the Arc differed.
    #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
    pub(crate) node_wake: Option<portable_atomic_util::Arc<super::node_wake::NodeWake>>,
    /// Phase 130.4 — true when at least one session's backend
    /// installed the wake callback. Drives whether `spin_once`
    /// uses the wake-primitive wait (`NodeWake` / `Condvar`) or
    /// just `drive_io(timeout_ms)`. Poll-only backends
    /// (XRCE-DDS-Client, current Cyclone/dust-DDS shims) leave
    /// this `false`; the wait then becomes a no-op sleep that
    /// starves reliable retransmission (Phase 127.C.4 root
    /// cause: server's `send_response` flushes 100 ms once, then a
    /// blind `wait_ms(100)` sleeps with zero session activity, so
    /// the agent's ACK arrives into a stalled session and reliable
    /// redelivery never fires).
    #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
    pub(crate) has_async_wake: bool,
    /// Phase 124.B.2 — opaque context Arc handed to backends via
    /// `set_wake_callback`. Lazy-allocated on first install; stays
    /// alive for the Executor's lifetime so the raw pointer stored
    /// in backends remains valid.
    #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
    pub(crate) wake_ctx: Option<portable_atomic_util::Arc<WakeCtx>>,
    /// Phase 124.B.7.c — lazily-allocated Linux signalfd worker.
    /// Owned by the Executor; spawned on first `signal_fd()` call.
    /// Drop joins the worker thread and closes the fd.
    #[cfg(all(feature = "signal-fd-wake", target_os = "linux"))]
    pub(crate) signal_fd: Option<WakeSignalFd>,
    #[cfg(feature = "param-services")]
    pub(crate) params: Option<alloc::boxed::Box<crate::parameter_services::ParamState<'s>>>,
    /// phase-425 W3b — the `/clock` subscription this image installed, if any.
    /// `None` means no time source; the value is the handle so it can be
    /// cancelled when `use_sim_time` goes false.
    #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
    pub(crate) sim_time_source: Option<HandleId>,
    /// phase-425 W3b — the LAST REQUESTED state of `use_sim_time`, which is not
    /// the same thing as the installed state: a request can arrive before any
    /// node exists (`nros::main!` declares parameters BEFORE it registers
    /// components), and there is no node to hang a subscription on until then.
    /// `reconcile_ros_time_source` closes the gap on each spin.
    #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
    pub(crate) sim_time_requested: bool,
    /// Whether `use_sim_time` was ever stated to THIS executor.
    ///
    /// `sim_time_requested` alone cannot answer it: `false` is both "told to
    /// turn it off" and "never told anything", and the two license opposite
    /// actions on a PROCESS-GLOBAL gate. Without this, every executor that
    /// never heard of the parameter wrote `set_active(false)` on its first
    /// spin -- because the gate defaults to TRUE, so `false != is_active()`
    /// held -- and switched off a simulated clock somebody else installed.
    /// One `cargo test` process is exactly that situation: 344 sibling tests
    /// spin an executor each, and the sim-time test failed 3 runs of 3 while
    /// passing alone.
    #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
    pub(crate) sim_time_stated: bool,
    #[cfg(feature = "lifecycle-services")]
    pub(crate) lifecycle:
        Option<alloc::boxed::Box<crate::lifecycle_services::LifecycleRuntimeState>>,
    /// Wall-clock instant at which the previous `spin_once` exited. The
    /// timer delta on the next call is measured from this point so any
    /// time the caller spent between `spin_once` invocations (e.g. an
    /// explicit `thread::sleep`) counts toward timer accumulation just
    /// like time spent inside `drive_io`.

    /// Monotonic clock endpoint for no_std timer accounting.
    // phase-359 W4 — ONE field. The std twin held an `Instant`; both now hold
    // µs from the single `now_us()` read.
    pub(crate) last_spin_end_us: Option<u64>,
    /// The executor's monotonic µs clock: `ExecutorConfig::clock_us` when the
    /// caller supplied one, else [`default_clock_us_fn`].
    ///
    /// phase-359 W10 — no longer `no_std`-only. The std build read an
    /// `Instant` through a separate field instead, which is how "what time is
    /// it" had two answers in one crate.
    pub(crate) clock_us_fn: Option<fn() -> u64>,

    /// Consecutive `drive_io` failures on the PRIMARY session (issue 0324).
    ///
    /// Reset to 0 by any successful drive. A session that has died — router
    /// gone, lease expired, socket closed — used to keep returning `Ok(())`
    /// from `spin()` forever: the node looked alive, publishes went nowhere,
    /// and no callback fired. Nothing in the crate could report it; a
    /// `git grep` for a health surface found none, so every such
    /// investigation started from packet captures (issue 0268 burned days
    /// this way).
    ///
    /// A COUNTER rather than propagating the error out of `spin_once`,
    /// deliberately: `drive_io` returns `Err` for any non-OK backend code, and
    /// whether a benign poll timeout maps to one is backend-specific. Aborting
    /// the spin on a single failure would risk turning a transient into a dead
    /// node. A counter cannot regress behaviour and still makes the condition
    /// observable via [`Executor::session_io_failures`].
    pub(crate) consecutive_io_failures: u32,
    /// RFC-0052 W3b.2 — wall-clock (epoch µs) source for age monitors.
    pub(crate) epoch_us_fn: Option<fn() -> u64>,
    /// RFC-0052 W3b.4 — baked contract-monitor table (empty = uncontracted
    /// image; every monitor path below folds away).
    pub(crate) monitor_table: &'static [super::monitor::MonitorSpec],
    pub(crate) monitor_states: [super::monitor::MonitorState; super::monitor::MAX_MONITORS],
    /// W3b.5 — baked subscriber age-contract table (empty = none).
    pub(crate) age_table: &'static [super::monitor::AgeMonitorSpec],
    pub(crate) age_states: [super::monitor::AgeState; super::monitor::MAX_MONITORS],
    /// W3b.5 — hook invoked on `DeadlineAction::Fault` (panic when unset).
    pub(crate) fault_fn: Option<fn(&super::monitor::Violation)>,
    /// Issue #515 — the spin cadence audit runs once, on the first spin
    /// that carries a non-zero timeout (which is where the tier's
    /// declared spin period becomes visible to the executor).
    pub(crate) spin_quantization_checked: bool,
    /// Issue #514 — violations discarded because the ring was full.
    /// Saturating. Without this a never-drained (or slowly-drained)
    /// image silently reports a stale prefix of its faults.
    pub(crate) monitor_violations_dropped: u32,
    /// Release jitter: worst observed lateness of a `spin_period` wake
    /// against its own nominal schedule, in microseconds.
    ///
    /// Issue #515 added a STATIC audit for one cause of cadence error --
    /// a period that is not a multiple of the spin period. Nothing measured
    /// the actual release instants, and `audit_spin_quantization` says so in
    /// as many words: "the rate is preserved and no activation is dropped,
    /// so every runtime rule is (correctly) silent -- the jitter stays
    /// invisible until someone measures cadence on target."
    ///
    /// This is that measurement, and it is the one `cyclictest` exists to
    /// report: the deviation between a timer's programmed wake-up and the
    /// instant the task actually resumed, whose MAXIMUM is the figure of
    /// merit for a real-time system.
    ///
    /// `spin_period` already holds both numbers -- `next_us` is the nominal
    /// deadline and `now_us()` the actual -- and when the loop is late it
    /// skips the sleep and discards the difference. That difference is the
    /// jitter.
    pub(crate) max_release_jitter_us: u64,
    /// Clock reading at the previous `spin_once` entry, for the interval the
    /// jitter is measured over. `None` until the first spin, and reset by
    /// `clear_release_jitter_stats` so a window starts clean.
    pub(crate) last_spin_entry_us: Option<u64>,
    /// Jitter high-water at the previous `release-jitter-runtime` check, so
    /// the rule reports the DELTA. Same shape as `overruns_reported` on a
    /// timer header, and for the same reason: a maximum that has not moved
    /// is the fault already reported, not a new one.
    pub(crate) jitter_reported_us: u64,
    /// Declared minimum stack headroom in bytes for the thread this executor
    /// spins on; `0` (default) disables the rule.
    ///
    /// An executor-level install rather than a contract field, because the
    /// bound cannot be derived from anything already declared -- see
    /// `check_stack_headroom`. The entry that spawned the thread is the one
    /// that knows what it gave it, so that is where the number comes from.
    pub(crate) min_stack_headroom_bytes: usize,
    /// Lowest headroom already reported, so the rule fires on new lows only.
    /// `usize::MAX` = nothing reported yet.
    pub(crate) stack_headroom_reported: usize,
    /// The pacing quantum the spin loop was last driven at, in microseconds.
    /// This is the bound the jitter rule judges against -- the caller's own
    /// declared cadence, so nothing further has to be declared.
    pub(crate) spin_nominal_us: u64,
    /// Wakes that were already past their nominal deadline, and wakes total.
    ///
    /// The maximum alone cannot distinguish one bad wake from a loop that is
    /// late every single cycle, and those are different faults: the first is
    /// a glitch, the second means the period cannot be met at all.
    pub(crate) late_wakes: u32,
    pub(crate) total_wakes: u32,
    /// Issue #514 — log every violation as it is detected. On by
    /// default: each rule pushed verdicts into a ring that nothing
    /// consumed in a real image, so a violated contract and a met one
    /// produced identical target-side output (none). Logging at
    /// DETECTION rather than draining the ring keeps
    /// [`Executor::drain_violations`] working unchanged for
    /// applications that report violations themselves.
    pub(crate) report_violations: bool,
    /// phase-409 — CARVED, at the fixed `MAX_VIOLATIONS` count (the same
    /// reasoning issue 0563 used for `remap_table`: the capability is unchanged,
    /// so it needs no new `ExecutorSizing` knob).
    pub(crate) monitor_violations: super::storage::CarvedVec<'s, super::monitor::Violation>,
    /// Issue 0790 — hooks that run BEFORE the session is closed, while every
    /// entity still works. The load-bearing half: a device releasing a bus or
    /// parking an actuator has to publish its final state / answer its last
    /// request from HERE, because after teardown it cannot.
    ///
    /// `[Option<_>; N]` rather than a `heapless::Vec`, deliberately: the handle
    /// a caller holds IS the slot index, so a removal must leave every other
    /// index where it was. A `Vec`'s `swap_remove` would silently re-point one
    /// live handle at a different callback, and its `remove` would re-point all
    /// of them. Clearing a slot to `None` also lets the next registration reuse
    /// it, which a fill cursor could not.
    pub(crate) pre_shutdown_hooks:
        [Option<super::types::ShutdownHook>; crate::config::MAX_SHUTDOWN_CBS],
    /// Issue 0790 — hooks that run AFTER the session is closed. rclcpp's
    /// `add_on_shutdown_callback` / `rclcpp::on_shutdown`. See
    /// [`Self::pre_shutdown_hooks`] for why this is an array.
    pub(crate) on_shutdown_hooks:
        [Option<super::types::ShutdownHook>; crate::config::MAX_SHUTDOWN_CBS],
}

impl<'s> Executor<'s> {
    /// phase-271 — assemble an executor over already-carved, caller-owned
    /// storage (`slices`, from [`super::storage::carve`]). Fills every
    /// non-storage field and reserves SC slot 0 for the default Fifo SC (carve
    /// left it `None`). The single builder shared by every constructor path.
    fn assemble(session: SessionStore, slices: super::storage::ExecutorSlices<'s>) -> Self {
        let super::storage::ExecutorSlices {
            arena,
            entries,
            sched_contexts,
            sched_context_bindings,
            sporadic_states,
            #[cfg(feature = "alloc")]
            sporadic_atomic_states,
            remaps,
            nodes,
            extra_sessions,
            extra_session_ids,
            node_sched_table,
            dispatch_slots,
            component_slots,
            active_groups,
            group_sched_table,
            monitor_violations,
        } = slices;
        // Slot 0 = the auto-created default Fifo SC (see field doc). carve
        // initialised the whole table to `None`; populate the reserved slot.
        if let Some(slot0) = sched_contexts.first_mut() {
            *slot0 = Some(super::sched_context::SchedContext::default());
        }
        // phase-412 -- stamp the self-report before anything can fail. `init`
        // is idempotent, so an image with two executors need not decide which
        // one owns the record, and `note_arena_capacity` records the slice this
        // executor was actually handed rather than the compiled constant: the
        // arena's placement is the caller's choice (issue 0900), so the two can
        // legitimately differ and the difference is worth seeing.
        crate::boot_report::init();
        crate::boot_report::note_arena_capacity(arena.len());
        crate::boot_report::checkpoint(crate::boot_report::Stage::ExecutorReady);
        Self {
            // `assemble` is reached from session-only entry points that have no
            // config, so 0 is the floor rather than a choice; `open_in` and any
            // binding that knows better overwrite it via `set_domain_id`.
            domain_id: 0,
            session,
            arena,
            arena_used: 0,
            entries,
            sched_contexts,
            sched_context_bindings,
            sporadic_states,
            #[cfg(feature = "alloc")]
            sporadic_atomic_states,
            major_frame_us: 0,
            #[cfg(all(
                feature = "alloc",
                feature = "rmw-cffi",
                feature = "scheduler-os-priority"
            ))]
            os_priority_pool: super::os_priority::OsPriorityPool::new(),
            #[cfg(all(feature = "alloc", feature = "scheduler-os-priority"))]
            os_priority_apply_policy: None,
            trigger: Trigger::Any,
            semantics: ExecutorSemantics::RclcppExecutor,
            node_name: heapless::String::new(),
            active_groups,
            active_groups_filtering: false,
            nodes,
            node_sched_table,
            group_sched_table,
            remap_table: remaps,
            remap_len: 0,
            dispatch_slots,
            component_slots,
            extra_sessions,
            extra_session_ids,
            primary_rmw_name: heapless::String::new(),
            primary_locator: heapless::String::new(),
            namespace: {
                let mut ns = heapless::String::new();
                let _ = ns.push_str("/");
                ns
            },
            #[cfg(feature = "alloc")]
            halt_flag: portable_atomic_util::Arc::new(portable_atomic::AtomicBool::new(false)),
            #[cfg(feature = "alloc")]
            wake_flag: portable_atomic_util::Arc::new(portable_atomic::AtomicBool::new(false)),
            #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
            node_wake: super::node_wake::NodeWake::new().map(portable_atomic_util::Arc::new),
            #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
            wake_ctx: None,
            #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
            has_async_wake: false,
            // Phase 141.A.3 — alloc-mode wake state init. Constructed
            // eagerly (NodeWake allocation) so the runtime cb can be
            // installed lazily on first session without a fallible
            // alloc inside spin_once. `None` when the platform
            // provider reports the primitive unavailable (matches
            // the std-RTOS path's `node_wake: Option<...>`).
            #[cfg(all(feature = "signal-fd-wake", target_os = "linux"))]
            signal_fd: None,
            #[cfg(feature = "param-services")]
            params: None,
            #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
            sim_time_source: None,
            #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
            sim_time_requested: false,
            #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
            sim_time_stated: false,
            #[cfg(feature = "lifecycle-services")]
            lifecycle: None,
            // Initialise the spin endpoint to construction time so the
            // very first `spin_once` credits time the caller spent
            // *before* it (e.g. setup, an explicit pre-spin sleep) just
            // like time spent between later calls.
            // One field, one seed now (phase-359 W10): the clock is absolute
            // on every flavour, so it is READ here rather than assumed to be
            // zero at construction. The std arm used to seed `Some(0)` because
            // its epoch WAS construction; with one provider that special case
            // is gone.
            last_spin_end_us: default_clock_us_fn().map(|clock| clock()),
            clock_us_fn: default_clock_us_fn(),
            consecutive_io_failures: 0,
            // RFC-0052 W3b.5 — a build with a wall clock gets one by default so
            // age monitors activate without extra wiring; a build with neither a
            // platform port nor a host gets `None`, and its board installs
            // `config.epoch_us` in `from_session_in`/`open`.
            //
            // phase-359 W10 — the cfg pair that used to be here moved into
            // `default_epoch_us_fn`, beside the `default_clock_us_fn` it
            // mirrors. (The old comment pointed at "the `not(std)` blocks
            // above" for the board install; those blocks are no longer
            // cfg-gated — the config override applies on every flavour now.)
            epoch_us_fn: super::types::default_epoch_us_fn(),
            monitor_table: &[],
            monitor_states: [super::monitor::MonitorState::default(); super::monitor::MAX_MONITORS],
            age_table: &[],
            age_states: [super::monitor::AgeState::default(); super::monitor::MAX_MONITORS],
            fault_fn: None,
            spin_quantization_checked: false,
            monitor_violations_dropped: 0,
            max_release_jitter_us: 0,
            last_spin_entry_us: None,
            jitter_reported_us: 0,
            min_stack_headroom_bytes: 0,
            stack_headroom_reported: usize::MAX,
            spin_nominal_us: 0,
            late_wakes: 0,
            total_wakes: 0,
            report_violations: true,
            monitor_violations,
            // Issue 0790 — both phase tables start empty. An image that
            // registers nothing pays these `None`s and a two-slot scan at
            // teardown, and nothing else.
            pre_shutdown_hooks: [None; crate::config::MAX_SHUTDOWN_CBS],
            on_shutdown_hooks: [None; crate::config::MAX_SHUTDOWN_CBS],
        }
    }

    /// Create an owning executor over caller-supplied `backing`, sized by
    /// `sizing`. The core, non-generic, per-entry entry point (the `alloc`
    /// [`from_session`](Self::from_session) convenience leaks a default backing
    /// and calls this; the macro / C FFI pass an entry-sized backing).
    ///
    /// # Safety
    /// `backing` must be ≥ `sizing.u64_len()` words, stay alive for `'s`, and
    /// not be otherwise accessed while the executor lives (it aliases it).
    /// `sizing.cbs` must be ≤ 64 (the `u64` ready-set bitmask ceiling).
    pub unsafe fn from_session_in(
        session: session::ConcreteSession,
        backing: &'s mut [MaybeUninit<u64>],
        sizing: super::storage::ExecutorSizing,
    ) -> Self {
        let slices = unsafe { super::storage::carve(backing, sizing) };
        Self::assemble(SessionStore::Owned(session), slices)
    }

    /// Create a borrowing executor over caller-supplied `backing`, sized by
    /// `sizing`. Counterpart to [`from_session_in`](Self::from_session_in) for
    /// the per-tier / C model (the session is borrowed, not owned).
    ///
    /// # Safety
    /// - `session_ptr` must point to a valid session that outlives the executor
    ///   and is not moved/dropped while it exists.
    /// - `backing` obligations as in [`from_session_in`](Self::from_session_in).
    pub unsafe fn from_session_ptr_in(
        session_ptr: *mut session::ConcreteSession,
        backing: &'s mut [MaybeUninit<u64>],
        sizing: super::storage::ExecutorSizing,
    ) -> Self {
        let slices = unsafe { super::storage::carve(backing, sizing) };
        Self::assemble(SessionStore::Borrowed(session_ptr), slices)
    }
}

impl Executor<'static> {
    /// Create an executor from an already-opened session, using the build-time
    /// default sizing (`MAX_CBS`/`MAX_SC`/`ARENA_SIZE`). Convenience for
    /// std/alloc callers that don't size per-entry: it leaks a default-sized
    /// backing (executor-lifetime, one-time) and calls
    /// [`from_session_in`](Self::from_session_in). Per-entry sizing goes through
    /// the macro / `open_in` instead.
    #[cfg(feature = "alloc")]
    pub fn from_session(session: session::ConcreteSession) -> Self {
        let sizing = super::storage::ExecutorSizing::DEFAULT;
        // SAFETY: the leaked backing is exactly `sizing.u64_len()` words,
        // `'static`, and uniquely owned by this executor.
        unsafe { Self::from_session_in(session, leak_default_backing(sizing), sizing) }
    }

    /// [`from_session`](Self::from_session) with an [`ExecutorConfig`], so a
    /// caller that brings its own session can also bring its own clock.
    ///
    /// issue 0709 / issue 0687 — `from_session` takes no config, and that is
    /// the path the no-port population uses: it accepts any `Session`, so a
    /// consumer with a non-cffi backend reaches the executor through it and had
    /// NO way to install `clock_us`. phase-359 W10 argued the `std`-without-a-
    /// port clock fallbacks could go because "a caller with a clock installs it
    /// through `ExecutorConfig::clock_us`" — true for `open`, false here, which
    /// is half of why that deletion was reverted.
    ///
    /// Only the timing sources are read from `config`; identity (locator,
    /// domain, names) belongs to the session the caller already opened. As in
    /// [`open_in`](Self::open_in), a `None` field does NOT clobber the
    /// platform default — it means "not specified" (the bug issue 0671
    /// records).
    #[cfg(feature = "alloc")]
    pub fn from_session_with(
        session: session::ConcreteSession,
        config: &super::types::ExecutorConfig<'_>,
    ) -> Self {
        let mut executor = Self::from_session(session);
        if let Some(clock) = config.clock_us {
            executor.clock_us_fn = Some(clock);
            executor.last_spin_end_us = Some(clock());
        }
        if let Some(epoch) = config.epoch_us {
            executor.epoch_us_fn = Some(epoch);
        }
        executor
    }

    /// Create an executor from a borrowed session pointer, default-sized. The
    /// `alloc` convenience wrapper over
    /// [`from_session_ptr_in`](Self::from_session_ptr_in) — leaks a default
    /// backing so existing callers keep the zero-storage-arg signature.
    ///
    /// # Safety
    /// - `session_ptr` must point to a valid, initialized session that lives at
    ///   least as long as this executor.
    /// - The caller must not move or drop the session while the executor exists.
    #[cfg(feature = "alloc")]
    pub unsafe fn from_session_ptr(session_ptr: *mut session::ConcreteSession) -> Self {
        let sizing = super::storage::ExecutorSizing::DEFAULT;
        // SAFETY: leaked backing as in `from_session`; session_ptr contract
        // forwarded to `from_session_ptr_in`.
        unsafe { Self::from_session_ptr_in(session_ptr, leak_default_backing(sizing), sizing) }
    }
}

impl<'s> Executor<'s> {
    /// Phase 228.B (RFC-0015) — construct a tier task's executor that **shares**
    /// a session opened once by the orchestration `main()`.
    ///
    /// In the per-tier execution model `main()` opens one RMW session, then
    /// spawns one RTOS task per priority tier; each task calls this to get an
    /// [`Executor`] over the *same* session (the `Borrowed` session store — this
    /// executor neither owns nor closes it), registers its tier's callback
    /// groups, and spins. Thin alias over [`Executor::from_session_ptr`].
    ///
    /// # Safety
    /// `session` must outlive every executor/task built from it (the
    /// orchestration `main()` holds it and never returns / WFIs), and must not
    /// be mutated except through these executors' spin calls.
    #[cfg(feature = "alloc")]
    pub unsafe fn open_with_session(session: *mut session::ConcreteSession) -> Executor<'static> {
        unsafe { Executor::<'static>::from_session_ptr(session) }
    }

    /// phase-271 — per-tier borrowed-session constructor over caller-supplied,
    /// per-tier-sized `backing`. The sized counterpart to
    /// [`open_with_session`](Self::open_with_session): each RTOS tier task owns
    /// its own backing so tiers size independently.
    ///
    /// # Safety
    /// `session` obligations as in [`open_with_session`](Self::open_with_session);
    /// `backing`/`sizing` as in [`from_session_ptr_in`](Self::from_session_ptr_in).
    pub unsafe fn open_with_session_in(
        session: *mut session::ConcreteSession,
        backing: &'s mut [MaybeUninit<u64>],
        sizing: super::storage::ExecutorSizing,
    ) -> Self {
        unsafe { Self::from_session_ptr_in(session, backing, sizing) }
    }

    /// Raw pointer to this executor's RMW session, for the per-tier model:
    /// the boot task opens the one session via [`Executor::open`] (the RMW
    /// session is a process-wide singleton — opening twice fails), then hands
    /// this pointer to each spawned tier task's
    /// [`Executor::open_with_session`]. The boot task's executor owns the
    /// session and outlives every borrower, so the pointer stays valid for the
    /// program's life. Works for both `Owned` and `Borrowed` stores.
    ///
    /// # Safety
    /// The returned pointer aliases `self.session`. Callers must keep `self`
    /// alive (not moved/dropped) for as long as any tier executor uses the
    /// pointer, and must only touch the session through executor spin calls
    /// (the RMW backend serializes concurrent access through its own locks).
    pub fn session_ptr(&mut self) -> *mut session::ConcreteSession {
        &mut *self.session as *mut session::ConcreteSession
    }

    /// Opaque, `Send` form of [`session_ptr`](Self::session_ptr) — the per-tier
    /// model hands this to each spawned tier task (it can cross the RTOS task /
    /// thread boundary, which a bare `*mut` cannot). See [`SessionHandle`].
    ///
    /// # Safety
    /// Same contract as [`session_ptr`](Self::session_ptr): `self` (the session
    /// owner) must outlive every executor built from the handle.
    pub fn session_handle(&mut self) -> SessionHandle {
        SessionHandle(self.session_ptr())
    }

    /// Open an [`Executor`] over the session a [`SessionHandle`] refers to (the
    /// `Borrowed` store — neither owns nor closes it). The tier-task counterpart
    /// to [`session_handle`](Self::session_handle).
    ///
    /// # Safety
    /// The handle's session must still be alive (its owning executor not moved
    /// or dropped); access only through executor spin calls.
    #[cfg(feature = "alloc")]
    pub unsafe fn open_with_session_handle(handle: SessionHandle) -> Executor<'static> {
        unsafe { Executor::<'static>::open_with_session(handle.0) }
    }

    /// phase-271 — sized counterpart to
    /// [`open_with_session_handle`](Self::open_with_session_handle) (per-tier
    /// backing).
    ///
    /// # Safety
    /// As [`open_with_session_handle`](Self::open_with_session_handle) +
    /// [`from_session_ptr_in`](Self::from_session_ptr_in).
    pub unsafe fn open_with_session_handle_in(
        handle: SessionHandle,
        backing: &'s mut [MaybeUninit<u64>],
        sizing: super::storage::ExecutorSizing,
    ) -> Self {
        unsafe { Self::open_with_session_in(handle.0, backing, sizing) }
    }

    /// Phase 228.C — set this tier executor's active callback-group filter. The
    /// generated per-tier task calls this before registering nodes; afterwards
    /// only callbacks whose `.callback_group()` is in `groups` register here.
    /// An empty slice (or never calling it) leaves the wildcard — register all
    /// callbacks (the single-tier degenerate case + today's behaviour).
    pub fn set_active_groups(&mut self, groups: &[&str]) {
        // phase-409 — the table is CARVED and reused, so clear before refilling;
        // the old `Option<heapless::Vec>` got a fresh empty vector each call.
        self.active_groups.clear();
        if groups.is_empty() {
            self.active_groups_filtering = false;
            return;
        }
        for g in groups {
            let mut s = heapless::String::new();
            if s.push_str(g).is_ok() {
                let _ = self.active_groups.push(s);
            }
        }
        self.active_groups_filtering = true;
    }

    /// The current callback-group filter, or `None` for the wildcard.
    fn active_group_filter(&self) -> Option<&[super::storage::GroupName]> {
        self.active_groups_filtering
            .then(|| self.active_groups.as_slice())
    }

    /// Phase 228.C — whether a callback in `group` should register in this
    /// executor under the current filter. The wildcard accepts everything.
    pub fn group_active(&self, group: &str) -> bool {
        group_filter_accepts(self.active_group_filter(), group)
    }

    /// Set the node name and namespace used for liveliness tokens.
    ///
    /// Called by `open()` to propagate config values. When `register_subscription`
    /// or `register_service` creates entities, these values are attached to the
    /// Phase 156 — record the primary session's backend identity
    /// (rmw name + locator) so `NodeBuilder::resolve_session_slot`
    /// can detect when a `.rmw(name)` matches the primary instead
    /// of opening a SECOND backend session against the same
    /// singleton (zenoh-pico's `g_session` is process-wide;
    /// opening twice fails). `Executor::open*` calls this
    /// automatically; the C surface (`nros_executor_init`) calls
    /// it manually because it constructs via `from_session_ptr`
    /// which doesn't know the open metadata. Empty strings = "no
    /// primary identity tracked"; the cache check degrades to
    /// always-miss.
    pub fn set_primary_identity(&mut self, rmw_name: &str, locator: &str) {
        self.primary_rmw_name.clear();
        let _ = self.primary_rmw_name.push_str(rmw_name);
        self.primary_locator.clear();
        let _ = self.primary_locator.push_str(locator);
    }

    /// `TopicInfo`/`ServiceInfo` so the zenoh backend can declare liveliness.
    /// Issue 0656 — set the ROS domain for entities this executor declares.
    ///
    /// For bindings that build an executor from an existing session
    /// (`from_session_ptr_in`), where no `ExecutorConfig` is available and the
    /// domain would otherwise stay at its 0 floor.
    pub fn set_domain_id(&mut self, domain_id: u32) {
        self.domain_id = domain_id;
    }

    /// The ROS domain this executor declares entities on.
    pub fn domain_id(&self) -> u32 {
        self.domain_id
    }

    pub fn set_node_identity(&mut self, node_name: &str, namespace: &str) {
        self.node_name.clear();
        let _ = self.node_name.push_str(node_name);
        if !namespace.is_empty() {
            self.namespace.clear();
            let _ = self.namespace.push_str(namespace);
        }
    }

    // =========================================================================
    // Phase 305 W3 (issue 0255) — per-node launch remap table
    // =========================================================================

    /// Record one launch `<remap from= to=/>` rule for the node identified by
    /// `(node_name, namespace)`. Rules are matched in declaration order (first
    /// wins) by [`Self::resolve_entity_name_for`]. Errors when a string
    /// overflows its slot or the table is at [`MAX_REMAPS`] — callers surface
    /// this rather than silently dropping a routing rule.
    #[allow(clippy::result_unit_err)]
    pub fn declare_remap(
        &mut self,
        node_name: &str,
        namespace: &str,
        from: &str,
        to: &str,
    ) -> Result<(), ()> {
        let mut rule = RemapRule {
            node_name: heapless::String::new(),
            namespace: heapless::String::new(),
            from: heapless::String::new(),
            to: heapless::String::new(),
        };
        rule.node_name.push_str(node_name)?;
        let ns = if namespace.is_empty() { "/" } else { namespace };
        rule.namespace.push_str(ns)?;
        rule.from.push_str(from)?;
        rule.to.push_str(to)?;
        // Same contract as the `heapless::Vec::push` this replaces: full table
        // is an error the caller surfaces, never a silently dropped rule.
        let slot = self.remap_table.get_mut(self.remap_len).ok_or(())?;
        *slot = Some(rule);
        self.remap_len += 1;
        Ok(())
    }

    /// Resolve a source-level entity name for the node identified by
    /// `(node_name, namespace)`: ROS 2 name expansion (`~`/relative → FQN)
    /// plus this node's declared remap rules (exact-FQN match, first rule
    /// wins). Nodes with no rules still get expansion. Errors on an
    /// unexpandable name (see `crate::names::expand_name`).
    #[allow(clippy::result_unit_err)]
    pub fn resolve_entity_name_for(
        &self,
        node_name: &str,
        namespace: &str,
        source: &str,
    ) -> Result<crate::names::ResolvedName, ()> {
        let ns = if namespace.is_empty() { "/" } else { namespace };
        let rules = self.remap_table[..self.remap_len]
            .iter()
            .flatten()
            .filter(|r| r.node_name.as_str() == node_name && r.namespace.as_str() == ns)
            .map(|r| (r.from.as_str(), r.to.as_str()));
        crate::names::resolve_name(source, node_name, ns, rules)
    }

    /// [`Self::resolve_entity_name_for`] against the executor's CURRENT node
    /// identity (`set_node_identity`) — the nros-c registration sites set that
    /// identity per node immediately before registering each entity.
    #[allow(clippy::result_unit_err)]
    pub fn resolve_entity_name(&self, source: &str) -> Result<crate::names::ResolvedName, ()> {
        self.resolve_entity_name_for(self.node_name.as_str(), self.namespace.as_str(), source)
    }

    // =========================================================================
    // Phase 272 (RFC-0047) — node-name → sched-context table
    // =========================================================================

    /// Seed a config-resolved tier binding by `(name, namespace)` before the
    /// node is built. `NodeBuilder::build` consults this table when no
    /// explicit `.sched()` override is given — the table entry then wins over
    /// the `SchedContextId(0)` default (precedence: explicit > table > 0).
    ///
    /// Call BEFORE `node_builder(name).build()`. An existing entry for the
    /// same `(name, namespace)` key is overwritten (last-write wins). Overflow
    /// past `MAX_NODES` is silently ignored. An empty `namespace` is normalised
    /// to `"/"` to match what `NodeBuilder::build` computes for a root-NS node.
    pub fn bind_node_name_sched(
        &mut self,
        name: &str,
        namespace: &str,
        sc: super::sched_context::SchedContextId,
    ) {
        let norm_ns = if namespace.is_empty() { "/" } else { namespace };
        // Overwrite if there is already an entry for this (name, ns) pair.
        for entry in self.node_sched_table.iter_mut() {
            if entry.0.as_str() == name && entry.1.as_str() == norm_ns {
                entry.2 = sc;
                return;
            }
        }
        // New entry — build the heapless strings and push. Silently ignore
        // if the name/ns is too long or the table is at capacity.
        let mut name_s = heapless::String::<64>::new();
        let mut ns_s = heapless::String::<64>::new();
        if name_s.push_str(name).is_err() || ns_s.push_str(norm_ns).is_err() {
            return;
        }
        let _ = self.node_sched_table.push((name_s, ns_s, sc));
    }

    /// Look up the seeded sched-context for `(name, namespace)`. Returns
    /// `None` when the table has no entry for this pair (unseed → default 0).
    /// `pub(super)` — visible only within the `executor` module (sibling
    /// `node_record` calls it from `NodeBuilder::build`).
    pub(super) fn lookup_node_sched(
        &self,
        name: &str,
        namespace: &str,
    ) -> Option<super::sched_context::SchedContextId> {
        for entry in self.node_sched_table.iter() {
            if entry.0.as_str() == name && entry.1.as_str() == namespace {
                return Some(entry.2);
            }
        }
        None
    }

    // =========================================================================
    // Phase 273 (RFC-0047) — per-callback-group → sched-context table
    // =========================================================================

    /// Seed a config-resolved tier binding by `(name, namespace, group)` before
    /// entities are registered. `apply_node_default_sched` consults this table
    /// first (group table > node default > `SchedContextId(0)`).
    ///
    /// Call BEFORE entity creation. An existing entry for the same
    /// `(name, namespace, group)` key is overwritten (last-write wins). Overflow
    /// past `MAX_CBS` is silently ignored. An empty `namespace` is normalised to
    /// `"/"` to match `NodeBuilder::build`. Mirror of `bind_node_name_sched`.
    pub fn bind_group_sched(
        &mut self,
        name: &str,
        namespace: &str,
        group: &str,
        sc: super::sched_context::SchedContextId,
    ) {
        let norm_ns = if namespace.is_empty() { "/" } else { namespace };
        // Overwrite if there is already an entry for this (name, ns, group).
        for entry in self.group_sched_table.iter_mut() {
            if entry.0.as_str() == name && entry.1.as_str() == norm_ns && entry.2.as_str() == group
            {
                entry.3 = sc;
                return;
            }
        }
        // New entry — build the heapless strings and push. Silently ignore
        // if name/ns/group is too long or the table is at capacity.
        let mut name_s = heapless::String::<64>::new();
        let mut ns_s = heapless::String::<64>::new();
        let mut grp_s = heapless::String::<32>::new();
        if name_s.push_str(name).is_err()
            || ns_s.push_str(norm_ns).is_err()
            || grp_s.push_str(group).is_err()
        {
            return;
        }
        let _ = self.group_sched_table.push((name_s, ns_s, grp_s, sc));
    }

    /// Look up the seeded sched-context for `(name, namespace, group)`. Returns
    /// `None` when the table has no entry for this triple.
    fn lookup_group_sched(
        &self,
        name: &str,
        namespace: &str,
        group: &str,
    ) -> Option<super::sched_context::SchedContextId> {
        for entry in self.group_sched_table.iter() {
            if entry.0.as_str() == name
                && entry.1.as_str() == namespace
                && entry.2.as_str() == group
            {
                return Some(entry.3);
            }
        }
        None
    }

    // =========================================================================
    // Phase 110.B — SchedContext API
    // =========================================================================

    /// Identifier of the auto-created default `Fifo`-class scheduling
    /// context. Every callback registered without an explicit
    /// [`bind_handle_to_sched_context`] binds to this SC.
    pub fn default_sched_context_id(&self) -> super::sched_context::SchedContextId {
        super::sched_context::SchedContextId(0)
    }

    /// Register a new scheduling context. Returns a [`SchedContextId`]
    /// callers pass to [`bind_handle_to_sched_context`] to attach
    /// callbacks. Phase 110.B.
    pub fn create_sched_context(
        &mut self,
        sc: super::sched_context::SchedContext,
    ) -> Result<super::sched_context::SchedContextId, NodeError> {
        // Slot 0 is reserved for the default Fifo SC; search 1..MAX_SC.
        for (i, slot) in self.sched_contexts.iter_mut().enumerate().skip(1) {
            if slot.is_none() {
                *slot = Some(sc);
                // Phase 110.E — Sporadic-class SCs get a sibling
                // `SporadicState` entry that the spin_once dispatch
                // path consults each cycle to refill the budget at
                // period boundaries and skip dispatch when budget
                // is exhausted.
                if matches!(sc.class, super::sched_context::SchedClass::Sporadic) {
                    let budget = sc.budget_us.get().map(|nz| nz.get()).unwrap_or(u32::MAX);
                    let period = sc.period_us.get().map(|nz| nz.get()).unwrap_or(u32::MAX);
                    self.sporadic_states[i] =
                        Some(super::sched_context::SporadicState::new(budget, period));
                }
                return Ok(super::sched_context::SchedContextId(i as u8));
            }
        }
        Err(NodeError::NoSchedContextSlot)
    }

    /// RFC-0052 W3b.2 — wall-clock µs since the UNIX epoch, when this
    /// target has an epoch source (config `epoch_us`, defaulted from
    /// `SystemTime` on hosted configs). `None` = no wall clock; age
    /// monitors must not have been baked (the emitter refuses).
    pub fn epoch_now_us(&self) -> Option<u64> {
        self.epoch_us_fn.map(|f| f())
    }

    /// RFC-0052 W3b.4 — install the baked contract-monitor table. Call
    /// BEFORE entity creation so `create_publisher` can attach each
    /// contracted endpoint's counter cell. Mirrors `set_qos_overrides`:
    /// `&'static`, codegen-baked, empty by default.
    pub fn set_monitor_table(&mut self, table: &'static [super::monitor::MonitorSpec]) {
        self.monitor_table = table;
    }

    /// The installed monitor table (empty unless the entry set one).
    #[must_use]
    pub fn monitor_table(&self) -> &'static [super::monitor::MonitorSpec] {
        self.monitor_table
    }

    /// W3b.5 — install the baked subscriber age-contract table. Call
    /// BEFORE entity creation so `create_subscription` can attach each
    /// contracted endpoint's age cell (needs an epoch source — see
    /// `ExecutorConfig::epoch_us`; without one the take path records
    /// nothing and age monitors stay silent).
    pub fn set_age_table(&mut self, table: &'static [super::monitor::AgeMonitorSpec]) {
        self.age_table = table;
    }

    /// The installed age table (empty unless the entry set one).
    #[must_use]
    pub fn age_table(&self) -> &'static [super::monitor::AgeMonitorSpec] {
        self.age_table
    }

    /// W3b.5 — override the wall-clock (epoch µs) source age monitors take
    /// message stamps against. Hosted builds default to `SystemTime`; a
    /// board with a synced RTC installs its own here (or via
    /// `ExecutorConfig::epoch_us`). Call BEFORE entity creation — the age
    /// hook captures this at `create_subscription` time.
    pub fn set_epoch_clock(&mut self, epoch_us: fn() -> u64) {
        self.epoch_us_fn = Some(epoch_us);
    }

    /// W3b.5 — resolve a subscription's age hook at registration time:
    /// exact topic match against the baked age table, only for stamped
    /// types (`M::STAMP_OFFSET`) and only when an epoch source exists.
    fn age_lookup<M: RosMessage>(&self, topic: &str) -> Option<super::arena::AgeMon> {
        M::STAMP_OFFSET?;
        let epoch = self.epoch_us_fn?;
        self.age_table
            .iter()
            .find(|a| a.topic == topic)
            .map(|a| (a.cell, epoch))
    }

    /// W3b.5 — install the `DeadlineAction::Fault` hook. Without one a
    /// fault-class deadline miss panics (watchdog-visible stop on
    /// embedded targets).
    /// Record one `spin_once` entry against the caller's intended cadence.
    ///
    /// Late is `(now - last_entry) - timeout`, clamped at zero: arriving early
    /// is not jitter, it is a poll that had nothing to wait for. A zero
    /// timeout claims no cadence and is skipped entirely, which keeps
    /// `Future::wait`-style busy spins out of the statistic.
    fn record_release_jitter(&mut self, timeout: core::time::Duration) {
        let nominal_us = timeout.as_micros().min(u64::MAX as u128) as u64;
        if nominal_us == 0 {
            return;
        }
        let Some(now) = self.now_us() else {
            return;
        };
        self.spin_nominal_us = nominal_us;
        if let Some(last) = self.last_spin_entry_us {
            let interval = now.saturating_sub(last);
            self.total_wakes = self.total_wakes.saturating_add(1);
            if let Some(late) = interval.checked_sub(nominal_us)
                && late > 0
            {
                self.late_wakes = self.late_wakes.saturating_add(1);
                if late > self.max_release_jitter_us {
                    self.max_release_jitter_us = late;
                }
            }
        }
        self.last_spin_entry_us = Some(now);
    }

    /// Release-jitter statistics from the spin loop: worst lateness in
    /// microseconds, the number of wakes that were already late, and the
    /// number of wakes total.
    ///
    /// The maximum is the figure of merit, and the ratio is what tells the
    /// two failures apart: one late wake in ten thousand is a glitch, ten
    /// thousand in ten thousand means the period cannot be met at all.
    ///
    /// Zero on a build with no clock -- `spin_period` refuses to run at all
    /// there (issue 0709), so there is nothing to have measured.
    pub fn release_jitter(&self) -> (u64, u32, u32) {
        (
            self.max_release_jitter_us,
            self.late_wakes,
            self.total_wakes,
        )
    }

    /// Reset the release-jitter statistics. For monitoring code that logs
    /// and clears per window, so a single early outlier does not pin the
    /// maximum for the life of the process.
    pub fn clear_release_jitter_stats(&mut self) {
        self.max_release_jitter_us = 0;
        self.late_wakes = 0;
        self.total_wakes = 0;
        self.last_spin_entry_us = None;
        self.jitter_reported_us = 0;
    }

    pub fn set_fault_handler(&mut self, f: fn(&super::monitor::Violation)) {
        self.fault_fn = Some(f);
    }

    /// Issue #515 — warn about periods the spin cadence cannot express.
    ///
    /// A timer fires on the first spin at or after its period elapses, so
    /// a period that is not an integer multiple of the executor's spin
    /// period lands on the spin grid instead of the declared cadence: a
    /// 33 ms timer on a 5 ms spin alternates 35 ms / 30 ms, mean 33.0.
    /// The rate is preserved and no activation is dropped, so every
    /// runtime rule is (correctly) silent — the jitter stays invisible
    /// until someone measures cadence on target.
    ///
    /// Runs ONCE, on the first spin carrying a non-zero timeout, because
    /// that timeout is where the tier's declared spin period becomes
    /// visible down here. A resolve-time diagnostic would be better
    /// still — the toolchain holds both numbers before the image is
    /// built — but this backstop needs no board or codegen change and
    /// catches hand-written spin loops too.
    fn audit_spin_quantization(&mut self, spin_us: u64) {
        if spin_us == 0 {
            return;
        }
        let arena_ptr = self.arena.as_ptr() as *const u8;
        for i in 0..self.entries.len() {
            let Some(meta) = self.entries[i].as_ref() else {
                continue;
            };
            if !matches!(meta.kind, EntryKind::Timer) {
                continue;
            }
            // SAFETY: a Timer entry's arena slot holds a `TimerEntry<F>`,
            // whose leading layout is `TimerHeader`.
            let header = unsafe { &*(arena_ptr.add(meta.offset) as *const TimerHeader) };
            let period_us = header.period_us;
            if period_us == 0 || period_us % spin_us == 0 {
                continue;
            }
            // The two periods the timer will actually alternate between.
            let early_us = (period_us / spin_us) * spin_us;
            let late_us = early_us.saturating_add(spin_us);
            nros_log::nros_warn!(
                nros_log::get_logger("nros"),
                "timer period {} us is not a multiple of the {} us spin period: activations will alternate between {} us and {} us (mean cadence preserved)",
                period_us,
                spin_us,
                early_us,
                late_us
            );
        }
    }

    /// Issue #514 — whether the executor logs each violation as it is
    /// detected (the default).
    ///
    /// Turn this off in an application that reports violations its own
    /// way via [`Self::drain_violations`]; the ring is unaffected
    /// either way.
    pub fn set_report_violations(&mut self, enabled: bool) {
        self.report_violations = enabled;
    }

    /// Issue #514 — violations discarded because the ring was full.
    ///
    /// Non-zero means the image produced faults faster than they were
    /// reported, so the reported set is a prefix, not the whole story.
    pub fn violations_dropped(&self) -> u32 {
        self.monitor_violations_dropped
    }

    /// RFC-0052 W3b.4 — drain pending contract violations (rate rule for
    /// now; age/latency land with W3b.5). The entry glue calls this after
    /// `spin_once` and feeds each entry to the `nros-diagnostics`
    /// reporter. Draining clears the ring.
    pub fn drain_violations(&mut self, mut f: impl FnMut(&super::monitor::Violation)) {
        for v in self.monitor_violations.iter() {
            f(v);
        }
        self.monitor_violations.clear();
    }

    /// THE monotonic-µs read. phase-359 W4 — every consumer goes through here.
    ///
    /// Before this there were FIVE spellings of "what time is it": this one,
    /// `last_spin_end: Instant`, and two ad-hoc `static EPOCH: OnceLock<Instant>`
    /// blocks — plus `PlatformClock`, which nothing called because `Executor` is
    /// deliberately non-generic and the trait's methods are associated fns.
    ///
    /// `None` means no clock is available (no_std with no injected hook).
    /// Callers must degrade, not guess: a missing clock is why the sporadic
    /// refill and the major-frame phase behaved differently on no_std.
    fn now_us(&mut self) -> Option<u64> {
        self.clock_us_fn.map(|clock| clock())
    }

    /// Run the rate/latency/age checks over the baked tables (single
    /// branch each when empty).
    fn run_contract_monitors(&mut self) {
        if !self.monitor_table.is_empty()
            && let Some(now_us) = self.now_us()
        {
            {
                for (i, spec) in self
                    .monitor_table
                    .iter()
                    .take(super::monitor::MAX_MONITORS)
                    .enumerate()
                {
                    if let Some(v) =
                        super::monitor::check_rate(spec, &mut self.monitor_states[i], now_us)
                    {
                        if self.report_violations {
                            super::monitor::log_violation(&v);
                        }
                        if self.monitor_violations.push(v).is_err() {
                            self.monitor_violations_dropped =
                                self.monitor_violations_dropped.saturating_add(1);
                        }
                    }
                    if let Some(v) =
                        super::monitor::check_latency(spec, &mut self.monitor_states[i])
                    {
                        if self.report_violations {
                            super::monitor::log_violation(&v);
                        }
                        if self.monitor_violations.push(v).is_err() {
                            self.monitor_violations_dropped =
                                self.monitor_violations_dropped.saturating_add(1);
                        }
                    }
                }
            }
        }
        if !self.age_table.is_empty() {
            for (i, spec) in self
                .age_table
                .iter()
                .take(super::monitor::MAX_MONITORS)
                .enumerate()
            {
                if let Some(v) = super::monitor::check_age(spec, &mut self.age_states[i]) {
                    if self.report_violations {
                        super::monitor::log_violation(&v);
                    }
                    if self.monitor_violations.push(v).is_err() {
                        self.monitor_violations_dropped =
                            self.monitor_violations_dropped.saturating_add(1);
                    }
                }
            }
        }
    }

    /// Issue #505 — report activations dropped by
    /// [`TimerOverrunPolicy::Skip`](super::arena::TimerOverrunPolicy)
    /// since the last check.
    ///
    /// Unlike the rate/age/latency rules this needs no baked spec table:
    /// every periodic timer counts its own overruns, and a dropped
    /// activation is a contract failure for any declared period. It runs
    /// on the same tick so violations land in the same ring the entry
    /// glue drains.
    /// Declare the minimum stack headroom this executor's thread must keep,
    /// in bytes. `0` (the default) disables the `stack-headroom-runtime`
    /// rule.
    ///
    /// Set by the entry that spawned the thread, because it is the only
    /// party that knows what stack it handed over: the executor never sees
    /// `stack_bytes`, and no portable query returns a task's total stack, so
    /// neither an absolute floor nor a percentage can be inferred here.
    pub fn set_min_stack_headroom_bytes(&mut self, bytes: usize) {
        self.min_stack_headroom_bytes = bytes;
    }

    /// Report a spin thread that has come closer to the end of its stack
    /// than `set_min_stack_headroom_bytes` allows.
    ///
    /// Runs on the same tick as the other rules and feeds the same ring.
    /// Costs one platform query per tick, and nothing at all when no minimum
    /// was declared.
    fn check_stack_headroom_rule(&mut self) {
        if self.min_stack_headroom_bytes == 0 {
            return;
        }
        let unused = nros_platform_api::stack_unused_bytes();
        // 0 means the port does not instrument stacks, not that the stack is
        // full. Reporting a violation there would be a fault invented from an
        // absence of data.
        if unused == 0 {
            return;
        }
        if let Some(v) = super::monitor::check_stack_headroom(
            unused,
            self.min_stack_headroom_bytes,
            &mut self.stack_headroom_reported,
        ) {
            if self.report_violations {
                super::monitor::log_violation(&v);
            }
            if self.monitor_violations.push(v).is_err() {
                self.monitor_violations_dropped = self.monitor_violations_dropped.saturating_add(1);
            }
        }
    }

    /// Issue #515 — report a spin wake a whole period late.
    ///
    /// Runs on the same tick as `check_timer_overruns` and feeds the same
    /// ring, so the entry glue drains it with the other rules and no caller
    /// needs to know this one exists. Like that rule it needs no spec table:
    /// the bound is the spin period the caller already passes in.
    fn check_release_jitter_rule(&mut self) {
        let (max_us, _late, _total) = self.release_jitter();
        let period_us = self.spin_nominal_us;
        if let Some(v) =
            super::monitor::check_release_jitter(max_us, &mut self.jitter_reported_us, period_us)
        {
            if self.report_violations {
                super::monitor::log_violation(&v);
            }
            if self.monitor_violations.push(v).is_err() {
                self.monitor_violations_dropped = self.monitor_violations_dropped.saturating_add(1);
            }
        }
    }

    fn check_timer_overruns(&mut self) {
        let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
        for i in 0..self.entries.len() {
            let Some(meta) = self.entries[i].as_ref() else {
                continue;
            };
            if !matches!(meta.kind, EntryKind::Timer) {
                continue;
            }
            // SAFETY: a Timer entry's arena slot holds a `TimerEntry<F>`,
            // which shares its leading layout with `TimerHeader`. The
            // baseline lives in the header too, so this needs no state
            // parallel to `entries` (whose capacity is a runtime slice
            // length, not a const).
            let header = unsafe { &mut *(arena_ptr.add(meta.offset) as *mut TimerHeader) };
            if let Some(v) = super::monitor::check_timer_overrun(
                header.overruns,
                &mut header.overruns_reported,
                0,
            ) {
                if self.report_violations {
                    super::monitor::log_violation(&v);
                }
                if self.monitor_violations.push(v).is_err() {
                    self.monitor_violations_dropped =
                        self.monitor_violations_dropped.saturating_add(1);
                }
            }
        }
    }

    /// RFC-0052 / phase-296 W3a — replace the DEFAULT scheduling context
    /// (slot 0, the SC every unbound callback dispatches through).
    ///
    /// The run_tiers model runs one Executor per tier, so a tier-wide
    /// scheduling policy (`[tiers.<t>] class/budget_us/period_us` and the
    /// TT window) is exactly "this executor's default SC". Boards call
    /// this once, before entity creation; explicit per-handle/per-group
    /// bindings still take precedence (they never resolve to slot 0).
    ///
    /// Sporadic-class SCs get the same sibling `SporadicState` the
    /// `create_sched_context` path builds, so budget refill/exhaustion
    /// applies to the default queue too.
    pub fn set_default_sched_context(&mut self, sc: super::sched_context::SchedContext) {
        self.sched_contexts[0] = Some(sc);
        if matches!(sc.class, super::sched_context::SchedClass::Sporadic) {
            let budget = sc.budget_us.get().map(|nz| nz.get()).unwrap_or(u32::MAX);
            let period = sc.period_us.get().map(|nz| nz.get()).unwrap_or(u32::MAX);
            self.sporadic_states[0] =
                Some(super::sched_context::SporadicState::new(budget, period));
        } else {
            self.sporadic_states[0] = None;
        }
    }

    /// Bind a registered callback to a scheduling context. The next
    /// `spin_once` cycle dispatches the callback through that SC's
    /// queue (FIFO bitmap or EDF heap). Phase 110.B.
    pub fn bind_handle_to_sched_context(
        &mut self,
        handle: HandleId,
        sc_id: super::sched_context::SchedContextId,
    ) -> Result<(), NodeError> {
        let i = handle.0;
        if i >= self.entries.len() {
            return Err(NodeError::InvalidSchedContextBinding);
        }
        if self.entries[i].is_none() {
            return Err(NodeError::InvalidSchedContextBinding);
        }
        let sc_idx = sc_id.0 as usize;
        if sc_idx >= self.sched_contexts.len() || self.sched_contexts[sc_idx].is_none() {
            return Err(NodeError::InvalidSchedContextBinding);
        }
        self.sched_context_bindings[i] = sc_id;
        Ok(())
    }

    /// Phase 110.F — opt in to per-callback OS-priority dispatch.
    /// Once registered, every `spin_once` cycle routes ready entries
    /// whose bound SC has `os_pri > 0` onto a worker thread the OS
    /// scheduler has elevated to that numeric priority. Workers are
    /// spawned lazily on first use and self-halt when the Executor
    /// drops.
    ///
    /// `apply_policy` is the same `fn(SchedPolicy) -> Result<(),
    /// SchedError>` shape `open_threaded` takes — keeps the
    /// Executor non-generic over Platform.
    ///
    /// Calling this with `apply_policy = noop` is fine for testing
    /// (workers spawn but don't actually elevate priority); real
    /// hard-RT use needs `CAP_SYS_NICE` on Linux or the equivalent
    /// kernel config on RTOSes.
    #[cfg(all(feature = "alloc", feature = "scheduler-os-priority"))]
    pub fn register_os_priority_dispatcher(
        &mut self,
        apply_policy: fn(
            nros_platform_api::SchedPolicy,
        ) -> Result<(), nros_platform_api::SchedError>,
    ) {
        self.os_priority_apply_policy = Some(apply_policy);
    }

    /// Phase 110.G — enable time-triggered dispatch by setting the
    /// executor's major-frame length. Once set, every `spin_once`
    /// cycle gates dispatch through each entry's bound SC's
    /// `tt_window_offset_us` / `tt_window_duration_us` fields:
    /// dispatch only fires when the current monotonic time falls
    /// inside the window `[off, off + duration) mod major_frame`.
    ///
    /// `major_frame_us = 0` disables the TT gate (default state).
    /// Setting a non-zero major frame after callbacks are already
    /// registered is allowed — TT gates take effect on the next
    /// `spin_once` cycle.
    pub fn register_time_triggered_dispatcher(&mut self, major_frame_us: u32) {
        self.major_frame_us = major_frame_us;
    }

    /// Phase 110.G — apply a declarative cyclic schedule.
    ///
    /// One-shot helper that wraps the underlying primitives:
    /// validates the schedule (`major_frame > 0`, no overlapping
    /// windows, every window fits inside the major frame), sets the
    /// executor's major-frame length, then materialises one
    /// `SchedContext` per window with `class = TimeTriggered` +
    /// the window's offset / duration. Returns the per-window
    /// [`SchedContextId`] array so callers can immediately
    /// `bind_handle_to_sched_context(handle, sc_id)` for their
    /// subscription / timer handles.
    ///
    /// `N` is the schedule's *declared* maximum window count;
    /// `schedule.window_count` gates how many SCs are actually
    /// created. Unused trailing slots return
    /// `SchedContextId::default()` (sentinel — callers must respect
    /// `window_count`).
    pub fn apply_time_triggered_schedule<const N: usize>(
        &mut self,
        schedule: &super::sched_context::TimeTriggeredSchedule<N>,
    ) -> Result<
        [super::sched_context::SchedContextId; N],
        super::sched_context::TimeTriggeredScheduleError,
    > {
        schedule.validate()?;
        self.major_frame_us = schedule.major_frame_us;
        // SC slot 0 is the auto-created default; reusing it as a
        // sentinel for unused trailing slots is safe because the
        // caller respects `schedule.window_count`.
        let mut ids: [super::sched_context::SchedContextId; N] =
            [super::sched_context::SchedContextId(0); N];
        for (i, window) in schedule.windows[..schedule.window_count].iter().enumerate() {
            // Deprecation note on `SchedClass::TimeTriggered`: TT
            // is implemented as a per-SC *window gate* on top of
            // the existing class-based dispatch (Fifo here keeps
            // the EDF / Sporadic budgets out of the picture for
            // pure cyclic schedules). The window-gate fields set
            // below are what `spin_once`'s 110.G runtime gate
            // actually reads.
            let sc = super::sched_context::SchedContext {
                tt_window_offset_us: super::sched_context::OptUs::from_us(window.offset_us),
                tt_window_duration_us: super::sched_context::OptUs::from_us(window.duration_us),
                ..super::sched_context::SchedContext::new_fifo()
            };
            ids[i] = self.create_sched_context(sc).map_err(|_| {
                super::sched_context::TimeTriggeredScheduleError::WindowCountOverflow
            })?;
        }
        Ok(ids)
    }

    /// Phase 110.E.b — register an ISR-driven refill timer for an
    /// already-created Sporadic SC. The caller invokes their
    /// platform's `PlatformTimer::create_periodic` with the returned
    /// `Arc<AtomicSporadicState>` as `user_data` and the
    /// `atomic_sporadic_refill_thunk` as the callback, then hands
    /// the resulting platform handle to this method via
    /// `OpaqueTimerHandle::new(handle, destroy_fn)`.
    ///
    /// The Executor stores both the Arc and the handle so Drop can
    /// clean them up. Calling this on a non-Sporadic SC returns
    /// `Err(InvalidSchedContextBinding)`.
    #[cfg(feature = "alloc")]
    pub fn register_sporadic_timer(
        &mut self,
        sc_id: super::sched_context::SchedContextId,
        timer: OpaqueTimerHandle,
    ) -> Result<portable_atomic_util::Arc<super::sched_context::AtomicSporadicState>, NodeError>
    {
        let i = sc_id.0 as usize;
        if i >= self.sched_contexts.len() {
            return Err(NodeError::InvalidSchedContextBinding);
        }
        let sc = self.sched_contexts[i]
            .as_ref()
            .ok_or(NodeError::InvalidSchedContextBinding)?;
        if !matches!(sc.class, super::sched_context::SchedClass::Sporadic) {
            return Err(NodeError::InvalidSchedContextBinding);
        }
        let budget = sc.budget_us.get().map(|nz| nz.get()).unwrap_or(u32::MAX);
        let period = sc.period_us.get().map(|nz| nz.get()).unwrap_or(u32::MAX);
        let state = portable_atomic_util::Arc::new(super::sched_context::AtomicSporadicState::new(
            budget, period,
        ));
        self.sporadic_atomic_states[i] = Some((portable_atomic_util::Arc::clone(&state), timer));
        Ok(state)
    }

    /// Inspect a registered scheduling context. Phase 110.B.
    pub fn sched_context(
        &self,
        sc_id: super::sched_context::SchedContextId,
    ) -> Option<&super::sched_context::SchedContext> {
        self.sched_contexts.get(sc_id.0 as usize)?.as_ref()
    }

    /// Phase 104.C.2 — start a rclcpp-style Node builder for this
    /// Executor. The returned [`NodeBuilder`](super::node_record::NodeBuilder)
    /// is chainable:
    ///
    /// ```ignore
    /// let id = exec.node_builder("ingress")
    ///     .rmw("zenoh")
    ///     .locator("tcp/127.0.0.1:7447")
    ///     .sched(my_sc_id)
    ///     .build()?;
    /// ```
    ///
    /// In Phase 104.C.2 the Node table is storage-only — all
    /// registered Nodes share the Executor's primary session. Per-
    /// Node session binding (the bridge feature) lands in Phase
    /// 104.C.3 when the session cache is wired.
    pub fn node_builder<'a, 'cfg>(
        &'a mut self,
        name: &'cfg str,
    ) -> super::node_record::NodeBuilder<'a, 'cfg, 's> {
        super::node_record::NodeBuilder {
            executor: self,
            name,
            namespace: None,
            rmw_name: None,
            locator: None,
            domain_id: None,
            sched: None,
            session_idx: None,
        }
    }

    /// Return the Node table — Phase 104.C.2 read accessor.
    pub fn nodes(&self) -> &[super::node_record::NodeRecord] {
        &self.nodes
    }

    /// Borrow a Node's metadata by id, returning `None` if the id
    /// is out of range.
    pub fn node(&self, id: super::node_record::NodeId) -> Option<&super::node_record::NodeRecord> {
        self.nodes.get(id.index())
    }

    /// Phase 189.M1 — an executor-borrowing node handle for the entity builders
    /// (`exec.node_mut(id).subscription(t)...` / `.create_subscription(...)`).
    /// A short-lived `&mut Executor` borrow — use one at a time; entity handles
    /// are owned and outlive it (see `NodeCtx`).
    pub fn node_mut(&mut self, id: super::node_record::NodeId) -> super::node::NodeCtx<'_, 's> {
        super::node::NodeCtx::new(self, id)
    }

    /// Issue #52 — install the baked QoS-override table on one node. Every
    /// entity created on it AFTERWARDS folds the matching `(topic, role)`
    /// entries into its QoS, before the backend-compat check — so an override
    /// the active RMW cannot honour still errors loudly rather than silently
    /// downgrading.
    ///
    /// Called by the generated entry (`nros::main!` → the register seam) right
    /// after the node is created and before the component declares entities.
    /// Unknown `node_id` is a no-op.
    pub fn set_node_qos_overrides(
        &mut self,
        node_id: super::node_record::NodeId,
        overrides: &'static [super::node_record::QoSOverrideCode],
    ) {
        if let Some(r) = self.nodes.get_mut(node_id.index()) {
            r.qos_overrides = overrides;
        }
    }

    /// Phase 104.C.3 — resolve a session-slot index to a mutable
    /// session reference. Slot 0 = the Executor's primary session;
    /// slots 1..=N = the `extra_sessions` vec opened by
    /// `node_builder.rmw(name)` calls that named a backend
    /// different from the primary.
    pub(crate) fn session_at_mut(&mut self, idx: u8) -> Option<&mut session::ConcreteSession> {
        if idx == 0 {
            Some(&mut *self.session)
        } else {
            self.extra_sessions.get_mut((idx - 1) as usize)
        }
    }

    /// Phase 104.C.9.b — resolve the per-Node session for direct
    /// entity creation paths (C++ FFI publisher / subscription /
    /// service that bypass the `register_*_on` arena dispatch).
    /// Returns `None` when `node_id` is out of range or the Node's
    /// `session_idx` lands outside the executor's session table.
    pub fn node_session_mut(
        &mut self,
        node_id: super::node_record::NodeId,
    ) -> Option<&mut session::ConcreteSession> {
        let session_idx = self.nodes.get(node_id.index())?.session_idx;
        self.session_at_mut(session_idx)
    }

    /// Phase 189.M1 — create a typed publisher bound to a node's session.
    /// Backs `node.publisher(t).typed::<M>().build()` on the
    /// executor-borrowing [`NodeCtx`](super::node::NodeCtx); the returned
    /// handle is owned and outlives the `NodeCtx`.
    pub fn create_publisher_on<M: crate::rmw_type_registry::MessageForRmw>(
        &mut self,
        node_id: super::node_record::NodeId,
        topic_name: &str,
        qos: QoSProfile,
    ) -> Result<crate::executor::handles::EmbeddedPublisher<M>, NodeError> {
        // RFC-0088 / phase-421 W1 — the message's declared format must be the
        // one the linked backend speaks. Universal: the const lives on
        // `RosMessage`, which `MessageForRmw` requires under every backend.
        crate::format_check::assert_message_format::<M>();
        // Phase 212.K.7.6.b — register `M`'s cyclonedds descriptor before
        // creating the underlying publisher handle. No-op for other RMWs.
        crate::rmw_type_registry::register_type::<M>()?;
        let handle = self.create_raw_publisher_handle_on(
            node_id,
            topic_name,
            <M as RosMessage>::TYPE_NAME,
            <M as RosMessage>::TYPE_HASH,
            qos,
        )?;
        // RFC-0052 W3b.4 — attach the contracted endpoint's counter cell.
        let monitor = self
            .monitor_table
            .iter()
            .find(|m| m.topic == topic_name)
            .map(|m| m.cell);
        Ok(crate::executor::handles::EmbeddedPublisher {
            handle,
            event_regs: crate::executor::handles::empty_event_regs(),
            monitor,
            epoch: self.epoch_us_fn,
            _phantom: PhantomData,
        })
    }

    /// Phase 189.M1 — create a generic (type-erased) publisher bound to a
    /// node's session. Backs `node.publisher(t).generic(ty, hash).build()`;
    /// the bridge re-publishes through this handle on the dest session.
    pub fn create_publisher_raw_on(
        &mut self,
        node_id: super::node_record::NodeId,
        topic_name: &str,
        type_name: &str,
        type_hash: &str,
        qos: QoSProfile,
    ) -> Result<crate::executor::handles::EmbeddedRawPublisher, NodeError> {
        let handle =
            self.create_raw_publisher_handle_on(node_id, topic_name, type_name, type_hash, qos)?;
        Ok(crate::executor::handles::EmbeddedRawPublisher {
            handle,
            arena: crate::executor::handles::TxArena::new(),
            event_regs: crate::executor::handles::empty_event_regs(),
        })
    }

    /// Shared prelude for the publisher-on-node paths: resolve the node's
    /// identity + session slot, build the [`TopicInfo`], validate QoS, and
    /// create the backend publisher handle. Mirrors
    /// `register_subscription_buffered_raw_on`'s session resolution so a
    /// bridge's source sub + dest pub agree on topic construction.
    fn create_raw_publisher_handle_on(
        &mut self,
        node_id: super::node_record::NodeId,
        topic_name: &str,
        type_name: &str,
        type_hash: &str,
        qos: QoSProfile,
    ) -> Result<session::RmwPublisher, NodeError> {
        let (node_name, ns, session_idx, overrides) = {
            let r = self
                .nodes
                .get(node_id.index())
                .ok_or(NodeError::InvalidSchedContextBinding)?;
            (
                r.name.clone(),
                r.namespace.clone(),
                r.session_idx,
                r.qos_overrides,
            )
        };
        // Issue #52 — fold the node's baked overrides for this topic BEFORE
        // `validate_against`, so an override the backend cannot honour errors
        // loudly instead of being silently dropped.
        let qos = super::node_record::apply_qos_override_codes(
            qos,
            topic_name,
            nros_rmw::QoSOverrideRole::Publisher,
            overrides,
        );
        let mut topic = TopicInfo::new(topic_name, type_name, type_hash)
            .with_domain(self.domain_id)
            .with_namespace(&ns);
        if !node_name.is_empty() {
            topic = topic.with_node_name(&node_name);
        }
        let session = self
            .session_at_mut(session_idx)
            .ok_or(NodeError::BackendMismatch)?;
        qos.validate_against(Session::supported_qos_policies(session))
            .map_err(NodeError::Transport)?;
        session
            .create_publisher(&topic, qos)
            .map_err(|_| NodeError::Transport(TransportError::PublisherCreationFailed))
    }

    /// Phase 124.B.1 — install the executor's wake callback onto the
    /// primary session. Best-effort: backends that don't override
    /// `Session::set_wake_callback` (poll-only XRCE, bare-metal)
    /// ignore the call and continue to be drained on the executor's
    /// deadline-bound cv-wait boundary.
    #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
    fn install_wake_signal_on_primary(&mut self) {
        use nros_rmw::Session as _;
        let ctx = self.wake_ctx_ptr();
        // SAFETY: `ctx` points at executor-owned wake state that outlives
        // the session callback installation and is cleared on executor drop.
        unsafe {
            self.session
                .set_wake_callback(Some(nros_rmw_runtime_wake_cb), ctx);
        }
        if self.session.supports_wake_callback() {
            self.has_async_wake = true;
        }
    }

    /// Phase 124.B.1 — install the wake callback onto an extra
    /// session opened by `node_builder.rmw(...)`. Called from
    /// `NodeBuilder::build()` right after `extra_sessions.push(...)`.
    #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
    pub(crate) fn install_wake_signal_on_extra(&mut self, idx: usize) {
        use nros_rmw::Session as _;
        let ctx = self.wake_ctx_ptr();
        if let Some(s) = self.extra_sessions.get_mut(idx) {
            // SAFETY: same executor-owned wake state as the primary session;
            // the extra session is owned by this executor.
            unsafe {
                s.set_wake_callback(Some(nros_rmw_runtime_wake_cb), ctx);
            }
            if s.supports_wake_callback() {
                self.has_async_wake = true;
            }
        }
    }

    /// Phase 124.B.2 — opaque context pointer the runtime wake
    /// callback receives. Encodes `(flag, mu, cv)` as a borrowed
    /// `&WakeCtx` reference; the callback decodes via
    /// `*const WakeCtx`.
    ///
    /// Lifetime: tied to the Executor instance. WakeCtx storage
    /// lives inside Executor (lazy-allocated on first install), so
    /// the pointer stays valid as long as the Executor is.
    /// Phase 124.B.7.c — POSIX signal-handler-safe wake fd.
    ///
    /// Returns a Linux `eventfd` that callers (typically POSIX
    /// signal handlers) can `write(fd, &1u64, 8)` to from any
    /// context, including signal handlers. A runtime-owned worker
    /// thread reads the fd and signals `wake_cv`, unblocking
    /// `spin_once`.
    ///
    /// The worker thread is spawned lazily on first call and
    /// joined on Executor drop. Linux-only and gated behind
    /// `feature = "signal-fd-wake"`; binaries that don't install
    /// signal handlers shouldn't enable it.
    ///
    /// Returns the raw fd. The Executor retains ownership; do not
    /// `close()` it from the caller.
    #[cfg(all(feature = "signal-fd-wake", feature = "rmw-cffi", target_os = "linux"))]
    /// phase-359 W10 — was `std::io::Result`. Same values, an error type that
    /// does not require `std`.
    pub fn signal_fd(&mut self) -> Result<core::ffi::c_int, NodeError> {
        let ctx_ptr = self.wake_ctx_ptr() as *const WakeCtx;
        if self.signal_fd.is_none() {
            self.signal_fd = Some(WakeSignalFd::new(ctx_ptr)?);
        }
        Ok(self.signal_fd.as_ref().expect("just set").fd())
    }

    #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
    fn wake_ctx_ptr(&mut self) -> *mut core::ffi::c_void {
        if self.wake_ctx.is_none() {
            self.wake_ctx = Some(portable_atomic_util::Arc::new(WakeCtx {
                flag: self.wake_flag.clone(),
                node_wake: self.node_wake.clone(),
            }));
        }
        let arc = self.wake_ctx.as_ref().expect("just set");
        portable_atomic_util::Arc::as_ptr(arc) as *mut core::ffi::c_void
    }

    /// Phase 104.C.4 — apply a Node's default SchedContext to a
    /// freshly-registered handle. Called from every `_inner`
    /// register variant after the entry slot is committed. No-op
    /// when `node_id` is None (legacy path), when the Node is
    /// out of range, or when the Node's `default_sched` is the
    /// auto-created Fifo slot (0) which matches the executor's
    /// default binding already.
    ///
    /// Phase 273 (RFC-0047) — extends with an optional `group` name.
    /// Precedence: **group table > node default > no binding** (SC 0).
    /// When `group` is `Some(g)`, consults `group_sched_table` first;
    /// if no entry exists for `(name, namespace, g)` falls back to the
    /// node's `default_sched`. When `group` is `None` the group table
    /// is not consulted (unchanged phase-272 path).
    ///
    /// Handles can still override per-call via
    /// `bind_handle_to_sched_context(handle, sc_id)` post-register.
    pub(crate) fn apply_node_default_sched(
        &mut self,
        slot: usize,
        node_id: Option<super::node_record::NodeId>,
        group: Option<&str>,
    ) {
        let Some(id) = node_id else { return };
        // Copy name, namespace, and default_sched out so the borrow on
        // `self.nodes` is released before the immutable `lookup_group_sched`
        // borrow and the mutable `sched_context_bindings` write below.
        let (name, namespace, node_sc) = {
            let Some(rec) = self.nodes.get(id.index()) else {
                return;
            };
            (rec.name.clone(), rec.namespace.clone(), rec.default_sched)
        };
        // Phase 273: group table > node default.
        let sc = match group {
            Some(g) => self
                .lookup_group_sched(name.as_str(), namespace.as_str(), g)
                .unwrap_or(node_sc),
            None => node_sc,
        };
        if sc.0 == 0 {
            return;
        }
        if slot >= self.entries.len() {
            return;
        }
        let sc_idx = sc.0 as usize;
        if sc_idx >= self.sched_contexts.len() || self.sched_contexts[sc_idx].is_none() {
            return;
        }
        self.sched_context_bindings[slot] = sc;
    }

    /// Phase 104.C.3.2 — scoped Node-handle access. The closure
    /// receives a [`Node`] bound to the requested [`NodeId`]'s
    /// session + identity. Use the standard `Node::create_publisher`,
    /// `create_subscription`, etc. APIs inside.
    ///
    /// rclcpp-aligned bridge pattern:
    ///
    /// ```ignore
    /// let node_in = exec.node_builder("ingress").rmw("zenoh").build()?;
    /// let node_out = exec.node_builder("egress").rmw("xrce").build()?;
    ///
    /// let pub_out = exec.with_node(node_out, |n| {
    ///     n.create_publisher::<Int32>("/fwd")
    /// })??;
    ///
    /// exec.with_node(node_in, |n| {
    ///     n.create_subscription_buffered::<Int32, _, 1024>(
    ///         "/src", qos(), move |m| { let _ = pub_out.publish(m); }
    ///     )
    /// })??;
    /// ```
    ///
    /// The closure can return any type; double-`?` unwraps the
    /// outer `Result<R, NodeError>` from `with_node` and the inner
    /// result returned by the closure.
    /// Phase 104.C.3.3.d — flat-Result variant of
    /// [`with_node`](Self::with_node). When the closure already
    /// returns `Result<R, NodeError>`, this avoids the double-`?`:
    ///
    /// ```ignore
    /// // Without `with_node_try`:
    /// let pub_ = exec.with_node(id, |n| n.create_publisher(...))??;
    ///
    /// // With `with_node_try`:
    /// let pub_ = exec.with_node_try(id, |n| n.create_publisher(...))?;
    /// ```
    pub fn with_node_try<R>(
        &mut self,
        id: super::node_record::NodeId,
        f: impl FnOnce(&mut NodeHandle<'_>) -> Result<R, NodeError>,
    ) -> Result<R, NodeError> {
        self.with_node(id, f)?
    }

    pub fn with_node<R>(
        &mut self,
        id: super::node_record::NodeId,
        f: impl FnOnce(&mut NodeHandle<'_>) -> R,
    ) -> Result<R, NodeError> {
        let (name, ns, session_idx) = {
            let r = self
                .nodes
                .get(id.index())
                .ok_or(NodeError::InvalidSchedContextBinding)?;
            (r.name.clone(), r.namespace.clone(), r.session_idx)
        };
        let monitors = self.monitor_table;
        let age_monitors = self.age_table;
        let epoch = self.epoch_us_fn;
        let domain_id = self.domain_id;
        let session = self
            .session_at_mut(session_idx)
            .ok_or(NodeError::BackendMismatch)?;
        // SAFETY: short-lived scoped reference. `Node::new` takes
        // `&mut ConcreteSession`; lifetime is bound to this fn's
        // body via the closure's borrow of `node`.
        // issue 0801 (second half) — the executor's domain, NOT a literal 0.
        // 429d5a581 fixed the ELEVEN arena `TopicInfo`s and left the THREE
        // `NodeHandle::new` sites, so an entity created through a node HANDLE
        // (`with_node`, `node`, `node_on`) still declared on domain 0 while the
        // arena path declared on the configured one. Same split the issue is
        // about, one constructor over: `loan_e2e` publishes through a handle and
        // subscribes through the arena, so it delivered on domain 0 and on
        // nothing else.
        let mut node = NodeHandle::new(name, ns, session, domain_id);
        // RFC-0052 W3b.4/.5 — seed the baked monitor tables so contracted
        // publishers/subscribers attach their cells without entry glue.
        node.set_monitors(monitors);
        node.set_age_monitors(age_monitors, epoch);
        Ok(f(&mut node))
    }

    /// Find a registered executor node by final name and namespace.
    pub fn node_id_by_name(
        &self,
        name: &str,
        namespace: &str,
    ) -> Option<super::node_record::NodeId> {
        self.nodes
            .iter()
            .enumerate()
            .find(|(_, node)| node.name.as_str() == name && node.namespace.as_str() == namespace)
            .map(|(index, _)| super::node_record::NodeId::from_raw(index as u8))
    }

    /// Create a node on this executor.
    ///
    /// Registers the node in the executor's table, deduplicating on
    /// `(name, namespace)` — phase-376 W5/B1.
    ///
    /// Until 2026-08-24 this path registered NOTHING. It built a `NodeHandle`
    /// and returned it, so `node_id_by_name` could not find a node the caller
    /// had just created, and two calls with one name handed out two handles the
    /// executor had never heard of. `create_node_on_with_domain` had the dedup
    /// (phase-267 added it there when N bridge endpoints overflowed the table);
    /// the plain path never got it.
    ///
    /// That is a prerequisite for the `create_node` vtable slot, whose contract
    /// is that the runtime calls it ONCE per distinct `(name, namespace)`:
    /// without a registry to check, the runtime would call it once per
    /// `create_node` and every backend would need its own dedup — which is the
    /// registry the slot exists to delete (zenoh's `ensure_node_liveliness`
    /// linear-scans `per_node_liveliness` for exactly this reason).
    ///
    /// The table is bounded by `MAX_NODES` (`NROS_EXECUTOR_MAX_NODES`, default
    /// 4), so a caller creating a fifth DISTINCT node now gets
    /// `NodeError::NodeTableFull` where it previously got a handle. That is the
    /// bound doing its job: a node the executor does not know about cannot
    /// carry a sched context, a QoS override, or a graph identity. Repeated
    /// calls with the SAME name are free.
    pub fn create_node(&mut self, name: &str) -> Result<NodeHandle<'_>, NodeError> {
        if name.len() > 64 {
            return Err(NodeError::NameTooLong);
        }

        let mut node_name = heapless::String::<64>::new();
        node_name
            .push_str(name)
            .map_err(|_| NodeError::NameTooLong)?;

        // Dedup against the executor's own namespace — the one this handle
        // will carry. `node_builder` resolves the session slot to 0 (primary)
        // when no rmw name is given, which is what this path has always used.
        if self
            .node_id_by_name(node_name.as_str(), self.namespace.as_str())
            .is_none()
        {
            self.node_builder(name).build()?;
        }

        // issue 0801 (second half) — the executor's domain, NOT a literal 0.
        // 429d5a581 fixed the ELEVEN arena `TopicInfo`s and left the THREE
        // `NodeHandle::new` sites, so an entity created through a node HANDLE
        // (`with_node`, `node`, `node_on`) still declared on domain 0 while the
        // arena path declared on the configured one. Same split the issue is
        // about, one constructor over: `loan_e2e` publishes through a handle and
        // subscribes through the arena, so it delivered on domain 0 and on
        // nothing else.
        let domain_id = self.domain_id;
        let mut node = NodeHandle::new(
            node_name,
            self.namespace.clone(),
            &mut self.session,
            domain_id,
        );
        node.set_monitors(self.monitor_table);
        node.set_age_monitors(self.age_table, self.epoch_us_fn);
        Ok(node)
    }

    /// Phase 128.F.2 — bridge-mode node factory. Registers a Node
    /// bound to the named RMW backend by opening (or reusing) an
    /// extra session via `node_builder().rmw(rmw).build()`, then
    /// returns a [`Node`] borrowing that session. Use when the
    /// binary intentionally links more than one backend and a Node
    /// must speak a specific one.
    ///
    /// The single-backend common case should keep using
    /// [`create_node`](Self::create_node) — this entry costs an
    /// extra session lookup and serves no purpose when only one
    /// backend is registered.
    #[cfg(feature = "rmw-cffi")]
    pub fn create_node_on(&mut self, name: &str, rmw: &str) -> Result<NodeHandle<'_>, NodeError> {
        self.create_node_on_with_domain(name, rmw, None, None)
    }

    /// Like [`create_node_on`](Self::create_node_on) but pins the extra
    /// session's domain id. Required for a multi-domain config-driven bridge:
    /// an extra RMW session's participant domain follows the **node builder's**
    /// `domain_id` (`resolve_session_slot` → `domain_id.unwrap_or(0)`), NOT the
    /// `SessionSpec`'s — so without this an egress on a non-zero domain silently
    /// opens on domain 0 and never matches its receiver (phase-267 issue 0109).
    /// `None` domain preserves the legacy domain-0 default. `locator` pins the
    /// extra session's address — REQUIRED for an agent-based backend (xrce: the
    /// Micro-XRCE-DDS Agent addr) whose session can't be opened locator-less;
    /// `None` keeps the rmw-default (cyclonedds is domain-discovered, no locator).
    pub fn create_node_on_with_domain(
        &mut self,
        name: &str,
        rmw: &str,
        domain_id: Option<u32>,
        locator: Option<&str>,
    ) -> Result<NodeHandle<'_>, NodeError> {
        if name.len() > 64 {
            return Err(NodeError::NameTooLong);
        }
        // Reuse an existing Node of this name rather than growing the node table
        // (phase-267 non-flat): a config-driven bridge calls this once per bridge
        // ENDPOINT, and the same session node (`s0`/`s1`) recurs across every
        // `[[bridge]]`. Without dedup, N bridges push 2N records and overflow
        // `MAX_NODES`. Names are unique per session in a generated bridge config,
        // so matching by name is unambiguous.
        let session_idx = if let Some(rec) = self.nodes.iter().find(|n| n.name.as_str() == name) {
            rec.session_idx
        } else {
            // Register the Node (opens an extra session under `rmw` if
            // none exists yet for that backend).
            let mut builder = self.node_builder(name).rmw(rmw);
            if let Some(d) = domain_id {
                builder = builder.domain_id(d);
            }
            if let Some(loc) = locator {
                builder = builder.locator(loc);
            }
            let id = builder.build()?;
            self.node(id).ok_or(NodeError::NodeTableFull)?.session_idx
        };

        let mut node_name = heapless::String::<64>::new();
        node_name
            .push_str(name)
            .map_err(|_| NodeError::NameTooLong)?;
        let namespace = self.namespace.clone();
        let monitors = self.monitor_table;
        let age_monitors = self.age_table;
        let epoch = self.epoch_us_fn;
        let domain_id = self.domain_id;
        let session = self
            .session_at_mut(session_idx)
            .ok_or(NodeError::NodeTableFull)?;
        // issue 0801 (second half) — the executor's domain, NOT a literal 0.
        // 429d5a581 fixed the ELEVEN arena `TopicInfo`s and left the THREE
        // `NodeHandle::new` sites, so an entity created through a node HANDLE
        // (`with_node`, `node`, `node_on`) still declared on domain 0 while the
        // arena path declared on the configured one. Same split the issue is
        // about, one constructor over: `loan_e2e` publishes through a handle and
        // subscribes through the arena, so it delivered on domain 0 and on
        // nothing else.
        let mut node = NodeHandle::new(node_name, namespace, session, domain_id);
        node.set_monitors(monitors);
        node.set_age_monitors(age_monitors, epoch);
        Ok(node)
    }

    /// Drive transport I/O (poll network, dispatch callbacks).
    #[allow(dead_code)]
    pub(crate) fn drive_io(&mut self, timeout_ms: i32) -> Result<(), NodeError> {
        self.session
            .drive_io(timeout_ms)
            .map_err(|_| NodeError::Transport(TransportError::PollFailed))
    }

    /// Close the underlying session, running the shutdown hooks around it.
    ///
    /// Issue 0790. The order is the feature:
    ///
    /// 1. every registered PRE-shutdown hook, while the session is still open
    ///    and every entity still works — this is where a node publishes a final
    ///    state, answers a last request, parks an actuator or releases a bus;
    /// 2. the session close;
    /// 3. every registered ON-shutdown hook.
    ///
    /// A hook runs EXACTLY ONCE: each phase table is emptied before its first
    /// hook is invoked, so a second `close()` — or the [`Drop`] sweep after one
    /// — finds nothing left to run. Step 2 runs even
    /// if a pre-shutdown hook was registered and step 3 even if the close
    /// failed — a hook cannot strand the session, and a dead session must not
    /// strand the hooks.
    ///
    /// # This is a CLEAN-STOP facility and nothing more
    ///
    /// A watchdog reset, a hard fault or a panic does not come through here, so
    /// these hooks do not run then. Nothing in a fixed static table can promise
    /// otherwise, and an API that implied it would be worse than none: hardware
    /// that must be safe across an abnormal stop needs a hardware answer (a
    /// pull-down, a watchdog-driven output disable), not a callback.
    pub fn close(&mut self) -> Result<(), NodeError> {
        self.run_shutdown_hooks(super::types::ShutdownPhase::Pre);
        let result = self
            .session
            .close()
            .map_err(|_| NodeError::Transport(TransportError::ConnectionFailed));
        self.run_shutdown_hooks(super::types::ShutdownPhase::Post);
        result
    }

    /// Register a hook to run BEFORE the session is closed (issue 0790).
    ///
    /// rclcpp's `Context::add_pre_shutdown_callback`. Returns the handle
    /// [`Self::remove_pre_shutdown_callback`] takes, or
    /// [`NodeError::ShutdownCallbacksFull`] when the phase table is full —
    /// raise `NROS_EXECUTOR_MAX_SHUTDOWN_CBS` (default 2) at build time.
    ///
    /// # Safety
    /// `callback` must be safe to invoke exactly once with `context`, and
    /// `context` must stay valid until the hook runs or is removed. The hook
    /// runs on whichever task calls [`Self::close`] (or drops the executor),
    /// which is not necessarily the task that registered it.
    pub unsafe fn add_pre_shutdown_callback(
        &mut self,
        callback: super::types::ShutdownCallbackFn,
        context: *mut core::ffi::c_void,
    ) -> Result<super::types::ShutdownCallbackHandle, NodeError> {
        Self::claim_shutdown_slot(
            &mut self.pre_shutdown_hooks,
            super::types::ShutdownPhase::Pre,
            callback,
            context,
        )
    }

    /// Register a hook to run AFTER the session is closed (issue 0790).
    ///
    /// rclcpp's `Context::add_on_shutdown_callback` / `rclcpp::on_shutdown`.
    /// Entities are gone by the time it runs, so anything that needs the wire
    /// belongs in [`Self::add_pre_shutdown_callback`] instead.
    ///
    /// # Safety
    /// Same contract as [`Self::add_pre_shutdown_callback`].
    pub unsafe fn add_on_shutdown_callback(
        &mut self,
        callback: super::types::ShutdownCallbackFn,
        context: *mut core::ffi::c_void,
    ) -> Result<super::types::ShutdownCallbackHandle, NodeError> {
        Self::claim_shutdown_slot(
            &mut self.on_shutdown_hooks,
            super::types::ShutdownPhase::Post,
            callback,
            context,
        )
    }

    /// Remove a pre-shutdown hook. `true` if `handle` named a live one.
    ///
    /// rclcpp's `Context::remove_pre_shutdown_callback`, and `bool` for the
    /// same reason: "it was not there" is an ordinary answer (the hook may
    /// already have run), not an error. A handle issued for the OTHER phase
    /// returns `false` and removes nothing — that is what the phase tag in
    /// [`ShutdownCallbackHandle`] buys.
    ///
    /// [`ShutdownCallbackHandle`]: super::types::ShutdownCallbackHandle
    pub fn remove_pre_shutdown_callback(
        &mut self,
        handle: super::types::ShutdownCallbackHandle,
    ) -> bool {
        Self::release_shutdown_slot(
            &mut self.pre_shutdown_hooks,
            super::types::ShutdownPhase::Pre,
            handle,
        )
    }

    /// Remove an on-shutdown hook. See [`Self::remove_pre_shutdown_callback`].
    pub fn remove_on_shutdown_callback(
        &mut self,
        handle: super::types::ShutdownCallbackHandle,
    ) -> bool {
        Self::release_shutdown_slot(
            &mut self.on_shutdown_hooks,
            super::types::ShutdownPhase::Post,
            handle,
        )
    }

    /// How many hooks are currently registered for `phase`. Diagnostic /
    /// test surface, and what a "did my registration land?" assertion reads.
    pub fn shutdown_callback_count(&self, phase: super::types::ShutdownPhase) -> usize {
        let table = match phase {
            super::types::ShutdownPhase::Pre => &self.pre_shutdown_hooks,
            super::types::ShutdownPhase::Post => &self.on_shutdown_hooks,
        };
        table.iter().flatten().count()
    }

    /// Claim the first free slot of a phase table.
    fn claim_shutdown_slot(
        table: &mut [Option<super::types::ShutdownHook>],
        phase: super::types::ShutdownPhase,
        callback: super::types::ShutdownCallbackFn,
        context: *mut core::ffi::c_void,
    ) -> Result<super::types::ShutdownCallbackHandle, NodeError> {
        for (index, slot) in table.iter_mut().enumerate() {
            if slot.is_some() {
                continue;
            }
            let handle = super::types::ShutdownCallbackHandle::new(phase, index)
                .ok_or(NodeError::ShutdownCallbacksFull)?;
            *slot = Some(super::types::ShutdownHook { callback, context });
            return Ok(handle);
        }
        Err(NodeError::ShutdownCallbacksFull)
    }

    /// Clear a slot of a phase table, rejecting a handle from the other phase.
    fn release_shutdown_slot(
        table: &mut [Option<super::types::ShutdownHook>],
        phase: super::types::ShutdownPhase,
        handle: super::types::ShutdownCallbackHandle,
    ) -> bool {
        if handle.phase() != Some(phase) {
            return false;
        }
        match table.get_mut(handle.index()) {
            Some(slot) => slot.take().is_some(),
            None => false,
        }
    }

    /// Run — and consume — every hook registered for `phase`, in registration
    /// order.
    ///
    /// The table is EMPTIED BEFORE the first callback runs, not slot by slot as
    /// the loop walks it, and both halves of that matter:
    ///
    /// * "exactly once" becomes a property of the table rather than of the
    ///   caller — a second `close()`, or the `Drop` sweep after one, finds
    ///   nothing left to run;
    /// * the `&mut self` borrow ENDS before any foreign code is invoked. A hook
    ///   is an `extern "C" fn` that may hold a raw pointer back to this
    ///   executor (the C and C++ shims hand out exactly that), so a hook that
    ///   registers or removes another one must not be running inside a live
    ///   `&mut` into the table it is touching.
    pub(crate) fn run_shutdown_hooks(&mut self, phase: super::types::ShutdownPhase) {
        let table = match phase {
            super::types::ShutdownPhase::Pre => &mut self.pre_shutdown_hooks,
            super::types::ShutdownPhase::Post => &mut self.on_shutdown_hooks,
        };
        // `ShutdownHook` is `Copy` (a fn pointer and a raw pointer), so this is
        // a register-width move of a table whose default size is two slots —
        // not a reason to keep the borrow open across the calls.
        let hooks = *table;
        *table = [None; crate::config::MAX_SHUTDOWN_CBS];
        for hook in hooks.iter().flatten() {
            // SAFETY: the `add_*_shutdown_callback` caller promised `callback`
            // is safe to invoke once with `context`, and that `context` stays
            // valid until the hook runs or is removed. This is that one call,
            // and the table is already cleared so it cannot happen again.
            unsafe {
                (hook.callback)(hook.context);
            }
        }
    }

    /// Phase 216 follow-up — register a per-Node dispatch trampoline.
    ///
    /// The board-side Entry pkg (or the macro-emitted
    /// `register_dispatch(executor)` wrapper, once wired) calls this
    /// once per deployed Node pkg, handing in the
    /// `__nros_node_<pkg>_on_callback` symbol + the Node's per-pkg
    /// `state` blob. [`Executor::dispatch_callback`] then linear-scans
    /// the registered slots when the dispatch task hands off a
    /// `SignaledCallback`.
    ///
    /// Returns `Err(())` when the registry is full (`MAX_NODES`
    /// entries — raise via `NROS_EXECUTOR_MAX_NODES` at build time).
    ///
    /// # Safety
    ///
    /// `state` must outlive the executor (the typical shape is a
    /// `*mut State` produced by
    /// `nros::__private_node_state_into_raw` from the
    /// macro-emitted `i()`; that pointer's lifetime IS the
    /// `Executor`'s by construction). `on_callback` must be safe to
    /// invoke with `(state, cb_id_ptr, cb_id_len, ctx)` matching the
    /// per-Node `__nros_node_<pkg>_on_callback` ABI emitted by the
    /// `nros::node!()` macro (Phase 216.A.5).
    #[allow(clippy::result_unit_err)]
    pub fn register_dispatch_slot(
        &mut self,
        state: *mut core::ffi::c_void,
        on_callback: unsafe extern "C" fn(
            *mut core::ffi::c_void,
            *const u8,
            usize,
            *mut core::ffi::c_void,
        ),
    ) -> Result<(), ()> {
        self.dispatch_slots
            .push(DispatchSlot { state, on_callback })
            .map_err(|_| ())
    }

    /// Phase 216 follow-up — current registered dispatch-slot count.
    /// Diagnostic / test surface.
    pub fn dispatch_slot_count(&self) -> usize {
        self.dispatch_slots.len()
    }

    /// Phase 258 (Track 2, 2a) — enroll a component into the executor-owned
    /// tick registry. Called by `nros`'s `install`/`register_node_borrowed`
    /// after it builds the `Arc<ComponentCell>`: `state` is the leaked
    /// `Arc<ComponentCell>` (the slot takes ownership), `tick`/`drop` are the
    /// `nros`-side trampolines (see [`ComponentSlot`]). The slot's `tick`
    /// runs at the tail of every [`spin_once`](Self::spin_once); its `drop`
    /// runs once on `Executor::drop`.
    ///
    /// Returns `Err(())` when the registry is full (`MAX_NODES` — raise via
    /// `NROS_EXECUTOR_MAX_NODES` at build time). On error the caller still
    /// owns `state` (the slot was not stored) and must drop it.
    ///
    /// # Safety
    /// `state` must be a `*mut` produced by leaking the component cell the
    /// `tick`/`drop` trampolines expect (an `Arc<ComponentCell>` via
    /// `Arc::into_raw` in the canonical `nros` caller), and must remain valid
    /// until the matching `drop` runs. `tick` must be safe to invoke with
    /// `(state, exec_ctx = *mut Executor)` each spin; `drop` must be safe to
    /// invoke exactly once with `state`.
    #[allow(clippy::result_unit_err)]
    pub unsafe fn enroll_component(
        &mut self,
        state: *mut core::ffi::c_void,
        tick: unsafe extern "C" fn(*mut core::ffi::c_void, *mut core::ffi::c_void),
        drop: unsafe extern "C" fn(*mut core::ffi::c_void),
    ) -> Result<(), ()> {
        self.component_slots
            .push(ComponentSlot { state, tick, drop })
            .map_err(|_| ())
    }

    /// Phase 258 (Track 2, 2a) — current enrolled component-slot count.
    /// Diagnostic / test surface.
    pub fn component_slot_count(&self) -> usize {
        self.component_slots.len()
    }

    /// issue #140 — the enrolled components' opaque `state` pointers, in enroll
    /// order. Each is the *leaked* `Arc<ComponentCell>` `enroll_component` was
    /// handed (see [`ComponentSlot::state`]); the `nros` layer re-borrows them
    /// to fold per-component dispatch counters into
    /// `observed_callback_counts` — install-seam components
    /// (`register_node_borrowed`) live ONLY here, not in
    /// `ExecutorNodeRuntime::components`, so the hosted-spin counts read zero
    /// without this surface.
    pub fn enrolled_component_states(&self) -> impl Iterator<Item = *mut core::ffi::c_void> + '_ {
        self.component_slots.iter().map(|slot| slot.state)
    }

    /// Phase 216 final dispatch hook — stable entry point the
    /// framework's dispatch task (RTIC `__nros_run` /
    /// Embassy `__nros_run_task`) calls for each `SignaledCallback`
    /// envelope it dequeues from the board-side SPSC / Embassy
    /// channel.
    ///
    /// ## Signature shape
    ///
    /// `nros-node` sits below `nros` in the dep graph, so the typed
    /// `nros::CallbackId<'_>` / `nros::CallbackCtx<'_>` types
    /// referenced in the Phase 216 design notes cannot appear in the
    /// signature here. The macro emit translates the dequeued
    /// envelope to the layer-clean `(cb_id: &str, ctx: *mut c_void)`
    /// pair before calling this method; the per-Node `on_callback`
    /// trampoline ABI (Phase 216.A.5,
    /// `__nros_node_<pkg>_on_callback(state, cb_id_ptr, cb_id_len,
    /// ctx)`) uses the same untyped shape on the other side of the
    /// fence, so the round-trip stays type-consistent.
    ///
    /// ## Body — linear scan of the dispatch registry
    ///
    /// Each registered [`DispatchSlot`] holds an
    /// `__nros_node_<pkg>_on_callback` fn pointer + the owning Node's
    /// `state` blob. The macro-emitted trampoline body
    /// `match`es on `CallbackId` tags the Node declared and is a
    /// no-op for non-matching `cb_id`s — at most one Node per
    /// `cb_id` actually acts, the rest are cheap string-compare
    /// no-ops. This mirrors the strategy
    /// `ExecutorNodeRuntime::dispatch_callback` uses in
    /// `packages/api/nros/src/node_runtime.rs:470`.
    ///
    /// ## What's NOT auto-wired today
    ///
    /// The `nros::node!()` macro doesn't yet emit a
    /// `register_dispatch(executor)` wrapper that pushes the per-pkg
    /// `(state, on_callback)` into this registry. Until that wiring
    /// lands (Phase 216 follow-up — see commit msg), downstream
    /// consumers (board's `init_hardware`, or the codegen-emitted
    /// `run_plan`) must call
    /// [`Executor::register_dispatch_slot`] explicitly with the
    /// `__nros_node_<pkg>_on_callback` symbol + a `state` blob from
    /// the macro-emitted `i()`.
    //
    // `ctx` is an opaque FFI cookie forwarded verbatim to each slot's
    // `on_callback`; this fn never dereferences it (the registered callback
    // does, under the `register_dispatch_slot` safety contract), so it is sound
    // to call from safe code.
    #[allow(clippy::not_unsafe_ptr_arg_deref)]
    pub fn dispatch_callback(&mut self, cb_id: &str, ctx: *mut core::ffi::c_void) {
        let cb_id_ptr = cb_id.as_ptr();
        let cb_id_len = cb_id.len();
        // Snapshot pointer + length to avoid an outstanding borrow
        // across the unsafe fn calls below; each `DispatchSlot` is
        // `Copy`, so iterating by value sidesteps any aliasing
        // worry the borrow checker would flag if a slot's
        // `on_callback` re-entered the executor.
        for slot in self.dispatch_slots.iter().copied() {
            // SAFETY: caller of `register_dispatch_slot` guaranteed
            // `state` outlives the executor + `on_callback` matches
            // the per-Node `__nros_node_<pkg>_on_callback` ABI;
            // `cb_id_ptr`/`cb_id_len` describe the live `&str` the
            // caller passed in.
            unsafe {
                (slot.on_callback)(slot.state, cb_id_ptr, cb_id_len, ctx);
            }
        }
    }

    /// Get a reference to the underlying session.
    pub fn session(&self) -> &session::ConcreteSession {
        &self.session
    }

    /// Get a mutable reference to the underlying session.
    pub fn session_mut(&mut self) -> &mut session::ConcreteSession {
        &mut self.session
    }

    /// Phase 124.F.3 — session-level connectivity probe. Wire-level
    /// round-trip "is the peer / agent / router still reachable?"
    /// — cheaper than the service-availability probe (no discovery
    /// state required).
    ///
    /// Returns `Ok(())` on reply within `timeout_ms`,
    /// `Err(NodeError::Transport(Timeout))` on no reply,
    /// `Err(NodeError::Transport(Unsupported))` when the active
    /// backend can't probe.
    ///
    /// Mirrors micro-ROS's `rmw_uros_ping_agent`. Useful for
    /// reconnect-on-link-loss patterns: bare-metal code can call
    /// `ping(100)` periodically and tear down / re-open the session
    /// on timeout.
    pub fn ping(&mut self, timeout_ms: i32) -> Result<(), NodeError> {
        use nros_rmw::Session;
        self.session
            .ping_session(timeout_ms)
            .map_err(NodeError::Transport)
    }

    /// phase-381 W4 — every node on the graph, with its namespace.
    ///
    /// `visit(name, namespace, enclave)` is called once per node and returns
    /// `false` to stop early. A VISITOR rather than a returned collection
    /// because there is no allocator at this layer and the graph has no bound
    /// the caller can know; peak extra memory is one entry. Every string is
    /// BORROWED for the duration of the call.
    ///
    /// `enclave` is `None` where the backend does not track one, which is what
    /// lets one call answer both `rmw_get_node_names` and
    /// `rmw_get_node_names_with_enclaves`.
    ///
    /// **This reports what has already been DISCOVERED, and never blocks.** The
    /// first call after startup legitimately sees a partial graph — the backend
    /// keeps a standing query fed by `spin`, so the view fills in over
    /// successive calls. Code that waits for a peer should poll, not call once
    /// and conclude. An empty result is "nobody seen yet", never "nobody
    /// exists".
    ///
    /// `Err(Transport(Unsupported))` from a backend with no graph — distinct
    /// from an empty graph, deliberately.
    pub fn get_node_names(
        &mut self,
        visit: &mut dyn FnMut(&str, &str, Option<&str>) -> bool,
    ) -> Result<(), NodeError> {
        use nros_rmw::Session;
        self.session
            .get_node_names(visit)
            .map_err(NodeError::Transport)
    }

    /// phase-381 W4 — every topic on the graph, with the types on it.
    ///
    /// `visit(topic_name, types)` is called once per distinct TOPIC — a topic
    /// carrying two types is one call with two entries, not two calls. `types`
    /// may legitimately be empty on a partially discovered graph: reporting the
    /// name without a type beats dropping it.
    ///
    /// Same discovery caveat as [`Self::get_node_names`].
    pub fn get_topic_names_and_types(
        &mut self,
        visit: &mut dyn FnMut(&str, &[&str]) -> bool,
    ) -> Result<(), NodeError> {
        use nros_rmw::Session;
        self.session
            .get_topic_names_and_types(visit)
            .map_err(NodeError::Transport)
    }

    /// phase-381 W4 — every service on the graph, with its types.
    /// As [`Self::get_topic_names_and_types`], over servers and clients.
    pub fn get_service_names_and_types(
        &mut self,
        visit: &mut dyn FnMut(&str, &[&str]) -> bool,
    ) -> Result<(), NodeError> {
        use nros_rmw::Session;
        self.session
            .get_service_names_and_types(visit)
            .map_err(NodeError::Transport)
    }

    /// phase-381 W4 — how many publishers are visible on `topic_name`.
    ///
    /// `topic_name` is a ROS name (`"/chatter"`). A count reflects what has
    /// been DISCOVERED, so it can be low right after startup and is never a
    /// proof of absence — see [`Self::get_node_names`].
    pub fn count_publishers(&mut self, topic_name: &str) -> Result<usize, NodeError> {
        use nros_rmw::Session;
        self.session
            .count_publishers(topic_name)
            .map_err(NodeError::Transport)
    }

    /// phase-381 W4 — how many subscribers are visible on `topic_name`.
    /// See [`Self::count_publishers`] for the caveats.
    pub fn count_subscribers(&mut self, topic_name: &str) -> Result<usize, NodeError> {
        use nros_rmw::Session;
        self.session
            .count_subscribers(topic_name)
            .map_err(NodeError::Transport)
    }

    /// phase-381 W4 — what one named node PUBLISHES, with the types.
    ///
    /// `visit(topic_name, types)` per distinct topic. A node the graph has not
    /// discovered yields no visits, which is not an error — see
    /// [`Self::get_node_names`] for why an empty answer means "not seen yet".
    pub fn get_publisher_names_and_types_by_node(
        &mut self,
        node_name: &str,
        node_namespace: &str,
        visit: &mut dyn FnMut(&str, &[&str]) -> bool,
    ) -> Result<(), NodeError> {
        use nros_rmw::{GraphEntityKind, Session};
        self.session
            .get_names_and_types_by_node(
                GraphEntityKind::Publisher,
                node_name,
                node_namespace,
                visit,
            )
            .map_err(NodeError::Transport)
    }

    /// phase-381 W4 — what one named node SUBSCRIBES to, with the types.
    ///
    /// **`subscription`, not `subscriber`** — this is rclrs's spelling
    /// (`get_subscription_names_and_types_by_node`), and the Rust surface takes
    /// its vocabulary from rclrs so a user porting Rust ROS 2 code types what
    /// they already know. The C surface says `subscriber` because rcl does, and
    /// the vtable slot says `subscriber` because upstream rmw does. Three
    /// layers, three upstreams, one word each — not drift. Issue 0788 owns the
    /// wider verb sweep.
    pub fn get_subscription_names_and_types_by_node(
        &mut self,
        node_name: &str,
        node_namespace: &str,
        visit: &mut dyn FnMut(&str, &[&str]) -> bool,
    ) -> Result<(), NodeError> {
        use nros_rmw::{GraphEntityKind, Session};
        self.session
            .get_names_and_types_by_node(
                GraphEntityKind::Subscriber,
                node_name,
                node_namespace,
                visit,
            )
            .map_err(NodeError::Transport)
    }

    /// phase-381 W4 — what services one named node SERVES, with the types.
    pub fn get_service_names_and_types_by_node(
        &mut self,
        node_name: &str,
        node_namespace: &str,
        visit: &mut dyn FnMut(&str, &[&str]) -> bool,
    ) -> Result<(), NodeError> {
        use nros_rmw::{GraphEntityKind, Session};
        self.session
            .get_names_and_types_by_node(GraphEntityKind::Service, node_name, node_namespace, visit)
            .map_err(NodeError::Transport)
    }

    /// phase-381 W4 — what services one named node CALLS, with the types.
    pub fn get_client_names_and_types_by_node(
        &mut self,
        node_name: &str,
        node_namespace: &str,
        visit: &mut dyn FnMut(&str, &[&str]) -> bool,
    ) -> Result<(), NodeError> {
        use nros_rmw::{GraphEntityKind, Session};
        self.session
            .get_names_and_types_by_node(GraphEntityKind::Client, node_name, node_namespace, visit)
            .map_err(NodeError::Transport)
    }

    /// phase-381 W4 — the publishers on `topic_name`, one visit each.
    ///
    /// Each `GraphEndpointInfo` BORROWS its strings for the duration of the
    /// visit; copy anything kept.
    ///
    /// It carries no QoS. The granted profile is what would answer "why is
    /// nothing arriving", and no backend can read one back yet — reporting the
    /// remote's DECLARED profile instead would be a confident wrong answer, so
    /// the field is absent rather than misleading.
    pub fn get_publishers_info_by_topic(
        &mut self,
        topic_name: &str,
        visit: &mut dyn FnMut(&nros_rmw::GraphEndpointInfo<'_>) -> bool,
    ) -> Result<(), NodeError> {
        use nros_rmw::Session;
        self.session
            .get_endpoint_info_by_topic(true, topic_name, visit)
            .map_err(NodeError::Transport)
    }

    /// phase-381 W4 — the subscriptions on `topic_name`, one visit each.
    /// See [`Self::get_publishers_info_by_topic`].
    pub fn get_subscriptions_info_by_topic(
        &mut self,
        topic_name: &str,
        visit: &mut dyn FnMut(&nros_rmw::GraphEndpointInfo<'_>) -> bool,
    ) -> Result<(), NodeError> {
        use nros_rmw::Session;
        self.session
            .get_endpoint_info_by_topic(false, topic_name, visit)
            .map_err(NodeError::Transport)
    }

    /// Get a mutable reference to an action client core in the arena by entry index.
    ///
    /// # Safety
    /// The caller must ensure that `entry_index` refers to an `ActionClientRawArenaEntry`.
    pub unsafe fn action_client_core_mut(
        &mut self,
        entry_index: usize,
    ) -> Option<&mut super::action_core::ActionClientCore> {
        let meta = self.entries.get(entry_index)?.as_ref()?;
        if !matches!(meta.kind, EntryKind::ActionClient) {
            return None;
        }
        let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
        unsafe {
            let entry_ptr = arena_ptr.add(meta.offset)
                as *mut super::arena::ActionClientRawArenaEntry<
                    { crate::config::DEFAULT_RX_BUF_SIZE },
                    { crate::config::DEFAULT_RX_BUF_SIZE },
                    { crate::config::DEFAULT_RX_BUF_SIZE },
                >;
            Some(&mut (*entry_ptr).core)
        }
    }

    /// Get a mutable reference to a service-client arena entry (Phase 82).
    ///
    /// Returns `None` if `entry_index` doesn't refer to a service client
    /// entry. The default reply buffer size is assumed because the C API
    /// always uses the default — the entry was registered via
    /// `register_service_client_raw_sized::<DEFAULT_RX_BUF_SIZE>`.
    ///
    /// # Safety
    /// `entry_index` must refer to a `ServiceClientRawArenaEntry`.
    pub unsafe fn service_client_entry_mut(
        &mut self,
        entry_index: usize,
    ) -> Option<&mut super::arena::ServiceClientRawArenaEntry<{ crate::config::DEFAULT_RX_BUF_SIZE }>>
    {
        let meta = self.entries.get(entry_index)?.as_ref()?;
        if !matches!(meta.kind, EntryKind::ServiceClient) {
            return None;
        }
        let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
        unsafe {
            let entry_ptr = arena_ptr.add(meta.offset)
                as *mut super::arena::ServiceClientRawArenaEntry<
                    { crate::config::DEFAULT_RX_BUF_SIZE },
                >;
            Some(&mut *entry_ptr)
        }
    }

    /// Set the executor-level trigger condition.
    ///
    /// Controls which handles must be ready before `spin_once` dispatches
    /// callbacks. Defaults to [`Trigger::AnyReady`](crate::Trigger).
    pub fn set_trigger(&mut self, trigger: Trigger) {
        self.trigger = trigger;
    }

    /// Set the executor data communication semantics.
    ///
    /// Choose between `Direct` (process in place) and `LET`
    /// (snapshot-then-process) semantics. See [`ExecutorSemantics`].
    pub fn set_semantics(&mut self, semantics: ExecutorSemantics) {
        self.semantics = semantics;
    }

    /// Set the invocation mode for a specific handle.
    ///
    /// Controls whether the callback fires on every spin
    /// ([`Always`](InvocationMode::Always)) or only when new data
    /// arrives ([`OnNewData`](InvocationMode::OnNewData), the default).
    pub fn set_invocation(&mut self, id: HandleId, mode: InvocationMode) {
        if let Some(Some(meta)) = self.entries.get_mut(id.0) {
            meta.invocation = mode;
        }
    }

    // ========================================================================
    // Arena-based callback registration
    // ========================================================================

    /// Arena bytes claimed by registered entities so far.
    ///
    /// EXACT, not a worst case: the arena is a bump allocator and every
    /// `arena_alloc` charges `size_of::<T>()`, so nothing is reserved per slot.
    /// `ARENA_SIZE` itself is derived the other way — every slot budgeted at
    /// the ActionClient worst case — which is issue 0900, and this accessor is
    /// how an image can find out what it actually needs.
    pub fn arena_used(&self) -> usize {
        self.arena_used
    }

    /// Total arena bytes this executor was given.
    ///
    /// The arena is a borrowed slice; whether its storage is stack or `.bss` is
    /// the caller's choice, not a property of this type (see
    /// `report_arena_headroom`).
    pub fn arena_capacity(&self) -> usize {
        self.arena.len()
    }

    /// Bump-allocate space for `T` in the arena. Returns the byte offset.
    pub(crate) fn arena_alloc<T>(&mut self) -> Result<usize, NodeError> {
        let align = core::mem::align_of::<T>();
        let size = core::mem::size_of::<T>();
        let aligned_offset = (self.arena_used + align - 1) & !(align - 1);
        let new_used = aligned_offset + size;
        if new_used > self.arena.len() {
            // issue 0900 — `BufferTooSmall` is returned by a dozen other paths,
            // so on a target where a return code is all you get, arena
            // exhaustion is indistinguishable from a message that did not fit.
            super::arena::report_arena_exhausted(
                new_used - self.arena.len(),
                self.arena_used,
                self.arena.len(),
            );
            crate::boot_report::note_alloc_failed(size, new_used - self.arena.len());
            return Err(NodeError::BufferTooSmall);
        }
        self.arena_used = new_used;
        crate::boot_report::note_alloc(size, new_used);
        Ok(aligned_offset)
    }

    /// Bump-allocate space for `T` plus `trailing_bytes` extra bytes.
    ///
    /// Returns `(entry_offset, trailing_offset)`. The trailing region starts
    /// immediately after `T` (aligned to 8 bytes).
    pub(crate) fn arena_alloc_with_trailing<T>(
        &mut self,
        trailing_bytes: usize,
    ) -> Result<(usize, usize), NodeError> {
        let align = core::mem::align_of::<T>();
        let entry_size = core::mem::size_of::<T>();
        let entry_offset = self.arena_used.next_multiple_of(align);
        // Trailing region starts on an 8-byte (u64) boundary after the entry.
        let trailing_offset =
            (entry_offset + entry_size).next_multiple_of(core::mem::align_of::<u64>());
        let new_used = trailing_offset + trailing_bytes;
        if new_used > self.arena.len() {
            // This path reported NOTHING until phase-412's self-report went in,
            // while its sibling `arena_alloc` has named the knob since issue
            // 0900. Half of arena exhaustion was therefore silent -- and it is
            // the half carrying buffered subscriptions and action entries,
            // which is what an island image actually allocates.
            super::arena::report_arena_exhausted(
                new_used - self.arena.len(),
                self.arena_used,
                self.arena.len(),
            );
            crate::boot_report::note_alloc_failed(
                new_used - entry_offset,
                new_used - self.arena.len(),
            );
            return Err(NodeError::BufferTooSmall);
        }
        self.arena_used = new_used;
        crate::boot_report::note_alloc(new_used - entry_offset, new_used);
        Ok((entry_offset, trailing_offset))
    }

    /// Find the next free entry slot index.
    pub(crate) fn next_entry_slot(&self) -> Result<usize, NodeError> {
        self.entries
            .iter()
            .position(|e| e.is_none())
            // Issue 0095 — the callback-entry table (`NROS_EXECUTOR_MAX_CBS`,
            // default 4) is full. Distinct from `BufferTooSmall` so the register
            // seam can tell the user to raise the knob.
            .ok_or(NodeError::ExecutorFull)
    }

    /// Install a finished [`CallbackMeta`] into its slot.
    ///
    /// phase-8 (`docs/design/callback_tracing.rst`) — the ONE choke point
    /// every registration site funnels through, so the
    /// `nros_callback_register(handle, kind, name)` event is emitted once,
    /// here, rather than at the 25 sites that build a `CallbackMeta`. The
    /// design's registration half is deliberately EXHAUSTIVE across every
    /// `EntryKind` even though the leaf hooks are staged: a callback that was
    /// registered but never observed then prints as "registered, not
    /// instrumented" instead of being silently absent, so an incomplete hook
    /// set announces itself in the output rather than looking like a
    /// measurement.
    ///
    /// It is the assignment — not [`next_entry_slot`](Self::next_entry_slot)
    /// — because a slot is claimed BEFORE the fallible work (session lookup,
    /// arena allocation, handle creation) that can still return `Err`.
    /// Emitting at slot-claim time would announce callbacks that do not
    /// exist.
    pub(crate) fn emplace_entry(&mut self, slot: usize, meta: CallbackMeta, name: TraceName<'_>) {
        trace_register(slot, meta.kind, name);
        self.entries[slot] = Some(meta);
    }

    /// Typed buffered subscription core (the `node_mut(id).subscription(t)
    /// .typed::<M>()` builder lowers here). Routes the typed subscription
    /// through the [`NodeId`]'s session + identity (rclcpp `add_node` pattern).
    ///
    /// Phase 403 W2 -- `rx_bytes` overrides how many bytes each buffer slot
    /// claims from the arena. `None` means `RX_BUF`, which is what every caller
    /// did before the knob existed and is what every caller that does not opt in
    /// still does: the default derivation is byte-for-byte unchanged.
    ///
    /// It is a RUNTIME `usize` and not a second const generic because on THIS
    /// path `RX_BUF` was never an array length -- it reaches
    /// `buffered_region_size`, `TripleBuffer::init` and `SpscRing::init`, all of
    /// which take a plain `usize`. That is the only reason W2's per-type sizing
    /// is expressible here at all: `Sub<{ M::BOUND }>` is
    /// `error: generic parameters may not be used in const operations` on stable
    /// (rustc 1.97.1), so the sibling `register_subscription_with_info_sized_inner`
    /// / `..._with_safety_sized_inner`, whose entries really do hold
    /// `[u8; RX_BUF]`, still need `nros::rx_buffer_for!` at the call site.
    pub(crate) fn register_subscription_buffered_on<M, F, const RX_BUF: usize>(
        &mut self,
        node_id: super::node_record::NodeId,
        topic_name: &str,
        qos: QoSProfile,
        callback: F,
        group: Option<&str>,
        rx_bytes: Option<usize>,
    ) -> Result<HandleId, NodeError>
    where
        M: crate::rmw_type_registry::MessageForRmw + 'static,
        F: FnMut(&M) + 'static,
    {
        type Entry<M, F> = SubBufferedEntry<M, F>;

        // RFC-0088 / phase-421 W1 — the message's declared format must be the
        // one the linked backend speaks. Universal: the const lives on
        // `RosMessage`, which `MessageForRmw` requires under every backend.
        crate::format_check::assert_message_format::<M>();
        // Phase 212.K.7.6.b — see `create_publisher_on`.
        crate::rmw_type_registry::register_type::<M>()?;

        let slot = self.next_entry_slot()?;
        let (node_name, ns, session_idx) = {
            let r = self
                .nodes
                .get(node_id.index())
                .ok_or(NodeError::InvalidSchedContextBinding)?;
            (r.name.clone(), r.namespace.clone(), r.session_idx)
        };
        let mut topic = TopicInfo::new(
            topic_name,
            <M as RosMessage>::TYPE_NAME,
            <M as RosMessage>::TYPE_HASH,
        )
        .with_domain(self.domain_id)
        .with_namespace(&ns)
        // Phase 231 (RFC-0038) — hand the backend a receive-buffer size so it
        // can size-class its receive storage (zenoh-pico: small vs large).
        //
        // Phase 392 W3a — the TYPE's bound, not `RX_BUF`. The arena slot size
        // says nothing about the message: a 64-byte type and a 4 KiB type both
        // hinted the same number, so the class was chosen from a value unrelated
        // to what arrives. Falls back to `RX_BUF` for an unbounded type, where
        // no bound exists to state (phase 380).
        .with_rx_buffer_hint(crate::rmw_type_registry::subscription_rx_hint::<M>(RX_BUF));
        if !node_name.is_empty() {
            topic = topic.with_node_name(&node_name);
        }
        // W3b.5 — contracted-endpoint age hook (None = free).
        let age_mon = self.age_lookup::<M>(topic_name);
        let handle = {
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            session
                .create_subscription(&topic, qos)
                .map_err(NodeError::Transport)?
        };

        // Phase 231 Wave 0.2 (RFC-0038) — in-place dispatch when the backend
        // advertises it: deserialize straight from the borrowed receive slot,
        // no arena buffer (copy #1 removed). Else the buffered path below.
        {
            use nros_rmw::Subscription as _;
            if handle.supports_process_in_place() {
                let entry_offset = self.arena_alloc::<SubInplaceEntry<M, F>>()?;
                unsafe {
                    let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
                    let entry_ptr = arena_ptr.add(entry_offset) as *mut SubInplaceEntry<M, F>;
                    core::ptr::write(
                        entry_ptr,
                        SubInplaceEntry {
                            handle,
                            callback,
                            age_mon,
                            _phantom: PhantomData,
                        },
                    );
                }
                let meta = CallbackMeta {
                    offset: entry_offset,
                    kind: EntryKind::Subscription,
                    try_process: sub_inplace_try_process::<M, F>,
                    has_data: sub_inplace_has_data::<M, F>,
                    pre_sample: no_pre_sample,
                    invocation: InvocationMode::OnNewData,
                    drop_fn: drop_entry::<SubInplaceEntry<M, F>>,
                };
                self.emplace_entry(slot, meta, TraceName::Text(topic_name));
                self.apply_node_default_sched(slot, Some(node_id), group);
                return Ok(HandleId(slot));
            }
        }

        // Phase 403 W2 -- one number for the whole allocation. The region size,
        // the strategy's slot size and the pointer arithmetic must agree, so
        // they read the same local rather than each spelling `RX_BUF`.
        let slot_size = rx_bytes.unwrap_or(RX_BUF);

        let (_slot_count, trailing_bytes) = buffered_region_size(qos.depth, slot_size);

        let (entry_offset, trailing_offset) =
            self.arena_alloc_with_trailing::<Entry<M, F>>(trailing_bytes)?;

        let buf_ptr = unsafe { (self.arena.as_mut_ptr() as *mut u8).add(trailing_offset) };

        let buffer = if qos.depth <= 1 {
            BufferStrategy::Triple(unsafe { TripleBuffer::init(buf_ptr, slot_size) })
        } else {
            BufferStrategy::Ring(unsafe { SpscRing::init(buf_ptr, slot_size, qos.depth as usize) })
        };

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(entry_offset) as *mut Entry<M, F>;
            core::ptr::write(
                entry_ptr,
                Entry {
                    handle,
                    buffer,
                    callback,
                    age_mon,
                    _phantom: PhantomData,
                },
            );
        }

        let meta = CallbackMeta {
            offset: entry_offset,
            kind: EntryKind::Subscription,
            try_process: sub_buffered_try_process::<M, F>,
            has_data: sub_buffered_has_data::<M, F>,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::OnNewData,
            drop_fn: drop_entry::<Entry<M, F>>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(topic_name));
        // Phase 104.C.4 — apply Node's default SchedContext.
        self.apply_node_default_sched(slot, Some(node_id), group);
        Ok(HandleId(slot))
    }

    /// Generic (type-erased) buffered subscription core (the
    /// `node_mut(id).subscription(t).generic(ty, hash)` builder lowers here).
    /// Routes the subscriber creation through the [`NodeId`]'s
    /// session + identity (rclcpp `add_node` pattern).
    ///
    /// Use this in bridge code where two Nodes bind to different RMW
    /// backends:
    ///
    /// ```ignore
    /// let node_in = exec.node_builder("ingress").rmw("zenoh").build()?;
    /// let pub_out = exec.with_node(node_out, |n| {
    ///     n.create_publisher_raw("/fwd", TYPE, HASH)
    /// })??;
    /// exec.register_subscription_buffered_raw_on::<_, 1024>(
    ///     node_in, "/src", TYPE, HASH, qos(),
    ///     move |bytes: &[u8]| { let _ = pub_out.publish_raw(bytes); },
    /// )?;
    /// ```
    pub(crate) fn register_subscription_buffered_raw_on<F, const RX_BUF: usize>(
        &mut self,
        node_id: super::node_record::NodeId,
        topic_name: &str,
        type_name: &str,
        type_hash: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<HandleId, NodeError>
    where
        F: FnMut(&[u8]) + 'static,
    {
        // Pull the Node's identity + session slot out first so the
        // mutable session borrow doesn't conflict with the arena
        // alloc inside `add_arena_subscription_callback`.
        let (node_name, ns, session_idx, overrides) = {
            let r = self
                .nodes
                .get(node_id.index())
                .ok_or(NodeError::InvalidSchedContextBinding)?;
            (
                r.name.clone(),
                r.namespace.clone(),
                r.session_idx,
                r.qos_overrides,
            )
        };
        // Issue #52 — fold the node's baked overrides for this topic before the
        // backend-compat check runs inside `create_subscription`.
        let qos = super::node_record::apply_qos_override_codes(
            qos,
            topic_name,
            nros_rmw::QoSOverrideRole::Subscription,
            overrides,
        );
        let mut topic = TopicInfo::new(topic_name, type_name, type_hash)
            .with_domain(self.domain_id)
            .with_namespace(&ns);
        if !node_name.is_empty() {
            topic = topic.with_node_name(&node_name);
        }
        let handle = {
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            session
                .create_subscription(&topic, qos)
                .map_err(NodeError::Transport)?
        };
        let handle_id = self.add_arena_subscription_callback::<F, RX_BUF>(handle, qos, callback)?;
        // Phase 104.C.4 — apply Node's default SchedContext.
        self.apply_node_default_sched(handle_id.0, Some(node_id), None);
        Ok(handle_id)
    }

    /// Register a borrowed (zero-copy) buffered subscription (Phase 229.6,
    /// issue 0007 / RFC-0033 `borrowed` mode).
    ///
    /// `B` is the code-generated borrowed-message marker (e.g. `ImageViewable`)
    /// implementing [`ViewableMessage`](nros_core::ViewableMessage); the
    /// callback receives `&B::View<'a>` — a lifetime-carrying message whose
    /// unbounded sequence/string fields borrow directly from the receive buffer
    /// (no `heapless::Vec` copy). The view is valid only for the callback's
    /// duration.
    ///
    /// **Triple-buffer only.** A borrowed view must reference exactly one
    /// well-defined buffer slot for the callback's duration; an SPSC ring
    /// (`qos.depth > 1`) keeps several samples in flight with no single such
    /// slot. `qos.depth > 1` is therefore rejected with
    /// [`TransportError::Unsupported`].
    pub(crate) fn register_subscription_buffered_borrowed_on<B, F, const RX_BUF: usize>(
        &mut self,
        node_id: super::node_record::NodeId,
        topic_name: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<HandleId, NodeError>
    where
        B: nros_core::ViewableMessage + 'static,
        F: for<'a> FnMut(&B::View<'a>) + 'static,
    {
        type Entry<B, F> = SubBufferedViewEntry<B, F>;

        // Borrowed views require a single well-defined slot (triple buffer).
        if qos.depth > 1 {
            return Err(NodeError::Transport(TransportError::Unsupported));
        }

        let slot = self.next_entry_slot()?;
        let (node_name, ns, session_idx) = {
            let r = self
                .nodes
                .get(node_id.index())
                .ok_or(NodeError::InvalidSchedContextBinding)?;
            (r.name.clone(), r.namespace.clone(), r.session_idx)
        };
        let mut topic = TopicInfo::new(
            topic_name,
            <B as ViewableMessage>::TYPE_NAME,
            <B as ViewableMessage>::TYPE_HASH,
        )
        .with_domain(self.domain_id)
        .with_namespace(&ns);
        if !node_name.is_empty() {
            topic = topic.with_node_name(&node_name);
        }
        let handle = {
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            session
                .create_subscription(&topic, qos)
                .map_err(NodeError::Transport)?
        };

        let (_slot_count, trailing_bytes) = buffered_region_size(qos.depth, RX_BUF);
        let (entry_offset, trailing_offset) =
            self.arena_alloc_with_trailing::<Entry<B, F>>(trailing_bytes)?;
        let buf_ptr = unsafe { (self.arena.as_mut_ptr() as *mut u8).add(trailing_offset) };

        // depth <= 1 guaranteed above → always triple buffer.
        let buffer = BufferStrategy::Triple(unsafe { TripleBuffer::init(buf_ptr, RX_BUF) });

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(entry_offset) as *mut Entry<B, F>;
            core::ptr::write(
                entry_ptr,
                Entry {
                    handle,
                    buffer,
                    callback,
                    _phantom: PhantomData,
                },
            );
        }

        let meta = CallbackMeta {
            offset: entry_offset,
            kind: EntryKind::Subscription,
            try_process: sub_buffered_view_try_process::<B, F>,
            has_data: sub_buffered_view_has_data::<B, F>,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::OnNewData,
            drop_fn: drop_entry::<Entry<B, F>>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(topic_name));
        self.apply_node_default_sched(slot, Some(node_id), None);
        Ok(HandleId(slot))
    }

    /// Register a raw (type-erased) buffered subscription whose callback
    /// also receives a [`RawMessageInfo`](nros_core::RawMessageInfo)
    /// carrying the sample's wire **attachment** (Phase 189.M1).
    ///
    /// Backs the `node.subscription(t).generic(..).message_info().build(cb)`
    /// builder — the cross-RMW bridge reads the `bridge_origin` tag from
    /// `info.attachment()` for echo suppression. One sample per
    /// `spin_once`; the attachment is staged in a flat per-entry buffer
    /// (cap [`RAW_INFO_ATT_CAP`](super::arena::RAW_INFO_ATT_CAP)).
    pub fn register_subscription_buffered_raw_info_on<F, const RX_BUF: usize>(
        &mut self,
        node_id: super::node_record::NodeId,
        topic_name: &str,
        type_name: &str,
        type_hash: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<HandleId, NodeError>
    where
        F: FnMut(&[u8], &nros_core::RawMessageInfo) + 'static,
    {
        type Entry<F, const N: usize> = SubBufferedRawInfoEntry<F, N>;

        let slot = self.next_entry_slot()?;
        let (node_name, ns, session_idx) = {
            let r = self
                .nodes
                .get(node_id.index())
                .ok_or(NodeError::InvalidSchedContextBinding)?;
            (r.name.clone(), r.namespace.clone(), r.session_idx)
        };
        let mut topic = TopicInfo::new(topic_name, type_name, type_hash)
            .with_domain(self.domain_id)
            .with_namespace(&ns);
        if !node_name.is_empty() {
            topic = topic.with_node_name(&node_name);
        }
        let handle = {
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            session
                .create_subscription(&topic, qos)
                .map_err(NodeError::Transport)?
        };

        let offset = self.arena_alloc::<Entry<F, RX_BUF>>()?;
        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut Entry<F, RX_BUF>;
            core::ptr::write(
                entry_ptr,
                Entry {
                    handle,
                    buffer: [0u8; RX_BUF],
                    att: [0u8; super::arena::RAW_INFO_ATT_CAP],
                    callback,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::Subscription,
            try_process: sub_buffered_raw_info_try_process::<F, RX_BUF>,
            has_data: sub_buffered_raw_info_has_data::<F, RX_BUF>,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::OnNewData,
            drop_fn: drop_entry::<Entry<F, RX_BUF>>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(topic_name));
        self.apply_node_default_sched(slot, Some(node_id), None);
        Ok(HandleId(slot))
    }

    /// Phase 250 (Wave 2) — register a generic (type-erased) raw subscription
    /// that surfaces E2E [`IntegrityStatus`](nros_rmw::IntegrityStatus) (CRC +
    /// sequence gap/dup) alongside the raw CDR bytes
    /// (`FnMut(&[u8], &IntegrityStatus)`). The type-erased analog of
    /// [`register_subscription_with_safety_sized_inner`]: the validator lives in
    /// the `RmwSubscriber` (`take_validated`), so the subscriber is created
    /// plainly and no `register_type::<M>()` is needed (the declarative `Node`
    /// path is generic). Used by the declarative runtime's `.safety()` opt-in.
    #[cfg(feature = "safety-e2e")]
    pub fn register_subscription_buffered_raw_safety_on<F, const RX_BUF: usize>(
        &mut self,
        node_id: super::node_record::NodeId,
        topic_name: &str,
        type_name: &str,
        type_hash: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<HandleId, NodeError>
    where
        F: FnMut(&[u8], &nros_rmw::IntegrityStatus) + 'static,
    {
        use super::arena::{
            SubBufferedRawSafetyEntry, sub_buffered_raw_safety_has_data,
            sub_buffered_raw_safety_try_process,
        };
        type Entry<F, const N: usize> = SubBufferedRawSafetyEntry<F, N>;

        let slot = self.next_entry_slot()?;
        let (node_name, ns, session_idx) = {
            let r = self
                .nodes
                .get(node_id.index())
                .ok_or(NodeError::InvalidSchedContextBinding)?;
            (r.name.clone(), r.namespace.clone(), r.session_idx)
        };
        let mut topic = TopicInfo::new(topic_name, type_name, type_hash)
            .with_domain(self.domain_id)
            .with_namespace(&ns);
        if !node_name.is_empty() {
            topic = topic.with_node_name(&node_name);
        }
        let handle = {
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            session
                .create_subscription(&topic, qos)
                .map_err(NodeError::Transport)?
        };

        let offset = self.arena_alloc::<Entry<F, RX_BUF>>()?;
        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut Entry<F, RX_BUF>;
            core::ptr::write(
                entry_ptr,
                Entry {
                    handle,
                    buffer: [0u8; RX_BUF],
                    callback,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::Subscription,
            try_process: sub_buffered_raw_safety_try_process::<F, RX_BUF>,
            has_data: sub_buffered_raw_safety_has_data::<F, RX_BUF>,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::OnNewData,
            drop_fn: drop_entry::<Entry<F, RX_BUF>>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(topic_name));
        self.apply_node_default_sched(slot, Some(node_id), None);
        Ok(HandleId(slot))
    }

    /// Register a raw byte-shaped callback against a pre-built
    /// `RmwSubscriber` handle.
    ///
    /// Backend-agnostic primitive — the caller is responsible for
    /// obtaining the handle by whatever route the active backend
    /// supports:
    ///
    /// - **Generic ROS-typed flow**: call `Session::create_subscription`
    ///   on `self.session_mut()` with a [`TopicInfo`]. The
    ///   `node_mut(id).subscription(t).generic(ty, hash)` builder is the
    ///   convenience wrapper for this path.
    /// - **Backend-specific flow** (e.g. uORB needs `&'static orb_metadata`):
    ///   reach into the concrete session via [`Self::session_mut`] and
    ///   call its backend-specific create method, then hand the handle
    ///   here. `nros-px4::uorb::create_subscription_with_callback` is
    ///   the example.
    ///
    /// The arena-store + vtable wiring is identical to
    /// `register_subscription_buffered_raw`; the only thing that varies is
    /// where the handle came from. Callback fires on every message
    /// delivery during [`spin_once`](Self::spin_once); bytes are
    /// passed as `&[u8]`.
    pub fn add_arena_subscription_callback<F, const RX_BUF: usize>(
        &mut self,
        handle: session::RmwSubscriber,
        qos: QoSProfile,
        callback: F,
    ) -> Result<HandleId, NodeError>
    where
        F: FnMut(&[u8]) + 'static,
    {
        type Entry<F> = SubBufferedRawEntry<F>;

        let slot = self.next_entry_slot()?;
        let (_slot_count, trailing_bytes) = buffered_region_size(qos.depth, RX_BUF);

        let (entry_offset, trailing_offset) =
            self.arena_alloc_with_trailing::<Entry<F>>(trailing_bytes)?;

        let buf_ptr = unsafe { (self.arena.as_mut_ptr() as *mut u8).add(trailing_offset) };

        let buffer = if qos.depth <= 1 {
            BufferStrategy::Triple(unsafe { TripleBuffer::init(buf_ptr, RX_BUF) })
        } else {
            BufferStrategy::Ring(unsafe { SpscRing::init(buf_ptr, RX_BUF, qos.depth as usize) })
        };

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(entry_offset) as *mut Entry<F>;
            core::ptr::write(
                entry_ptr,
                Entry {
                    handle,
                    buffer,
                    callback,
                },
            );
        }

        let meta = CallbackMeta {
            offset: entry_offset,
            kind: EntryKind::Subscription,
            try_process: sub_buffered_raw_try_process::<F>,
            has_data: sub_buffered_raw_has_data::<F>,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::OnNewData,
            drop_fn: drop_entry::<Entry<F>>,
        };
        self.emplace_entry(slot, meta, TraceName::Slot("sub", slot));
        Ok(HandleId(slot))
    }

    pub(crate) fn register_subscription_with_info_sized_inner<M, F, const RX_BUF: usize>(
        &mut self,
        node_id: Option<super::node_record::NodeId>,
        topic_name: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<HandleId, NodeError>
    where
        M: crate::rmw_type_registry::MessageForRmw + 'static,
        F: FnMut(&M, Option<&nros_core::MessageInfo>) + 'static,
    {
        type Entry<M, F, const N: usize> = SubInfoEntry<M, F, N>;

        // Phase 212.K.7.6.b — see `create_publisher_on`.
        crate::rmw_type_registry::register_type::<M>()?;

        let slot = self.next_entry_slot()?;
        let (node_name, ns, session_idx) = match node_id {
            Some(id) => {
                let r = self
                    .nodes
                    .get(id.index())
                    .ok_or(NodeError::InvalidSchedContextBinding)?;
                (r.name.clone(), r.namespace.clone(), r.session_idx)
            }
            None => (self.node_name.clone(), self.namespace.clone(), 0u8),
        };
        let mut topic = TopicInfo::new(
            topic_name,
            <M as RosMessage>::TYPE_NAME,
            <M as RosMessage>::TYPE_HASH,
        )
        .with_domain(self.domain_id)
        .with_namespace(&ns);
        if !node_name.is_empty() {
            topic = topic.with_node_name(&node_name);
        }
        let handle = {
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            session
                .create_subscription(&topic, qos)
                .map_err(NodeError::Transport)?
        };

        let offset = self.arena_alloc::<Entry<M, F, RX_BUF>>()?;

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut Entry<M, F, RX_BUF>;
            core::ptr::write(
                entry_ptr,
                Entry {
                    handle,
                    buffer: [0u8; RX_BUF],
                    sampled_len: 0,
                    callback,
                    _phantom: PhantomData,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::Subscription,
            try_process: sub_info_try_process::<M, F, RX_BUF>,
            has_data: sub_info_has_data::<M, F, RX_BUF>,
            pre_sample: sub_info_pre_sample::<M, F, RX_BUF>,
            invocation: InvocationMode::OnNewData,
            drop_fn: drop_entry::<Entry<M, F, RX_BUF>>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(topic_name));
        self.apply_node_default_sched(slot, node_id, None);
        Ok(HandleId(slot))
    }

    #[cfg(feature = "safety-e2e")]
    pub(crate) fn register_subscription_with_safety_sized_inner<M, F, const RX_BUF: usize>(
        &mut self,
        node_id: Option<super::node_record::NodeId>,
        topic_name: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<HandleId, NodeError>
    where
        M: crate::rmw_type_registry::MessageForRmw + 'static,
        F: FnMut(&M, &nros_rmw::IntegrityStatus) + 'static,
    {
        type Entry<M, F, const N: usize> = SubSafetyEntry<M, F, N>;

        // Phase 212.K.7.6.b — see `create_publisher_on`.
        crate::rmw_type_registry::register_type::<M>()?;

        let slot = self.next_entry_slot()?;
        let (node_name, ns, session_idx) = match node_id {
            Some(id) => {
                let r = self
                    .nodes
                    .get(id.index())
                    .ok_or(NodeError::InvalidSchedContextBinding)?;
                (r.name.clone(), r.namespace.clone(), r.session_idx)
            }
            None => (self.node_name.clone(), self.namespace.clone(), 0u8),
        };
        let mut topic = TopicInfo::new(
            topic_name,
            <M as RosMessage>::TYPE_NAME,
            <M as RosMessage>::TYPE_HASH,
        )
        .with_domain(self.domain_id)
        .with_namespace(&ns);
        if !node_name.is_empty() {
            topic = topic.with_node_name(&node_name);
        }
        let handle = {
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            session
                .create_subscription(&topic, qos)
                .map_err(NodeError::Transport)?
        };

        let offset = self.arena_alloc::<Entry<M, F, RX_BUF>>()?;

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut Entry<M, F, RX_BUF>;
            core::ptr::write(
                entry_ptr,
                Entry {
                    handle,
                    buffer: [0u8; RX_BUF],
                    sampled_len: 0,
                    callback,
                    _phantom: PhantomData,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::Subscription,
            try_process: sub_safety_try_process::<M, F, RX_BUF>,
            has_data: sub_safety_has_data::<M, F, RX_BUF>,
            pre_sample: sub_safety_pre_sample::<M, F, RX_BUF>,
            invocation: InvocationMode::OnNewData,
            drop_fn: drop_entry::<Entry<M, F, RX_BUF>>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(topic_name));
        self.apply_node_default_sched(slot, node_id, None);
        Ok(HandleId(slot))
    }

    /// Register a service callback with the default buffer size.
    ///
    /// The callback is stored in the arena and invoked during [`spin_once()`](Self::spin_once).
    pub fn register_service<Svc, F>(
        &mut self,
        service_name: &str,
        callback: F,
    ) -> Result<HandleId, NodeError>
    where
        Svc: RosService + 'static,
        Svc::Request: crate::rmw_type_registry::MessageForRmw,
        Svc::Reply: crate::rmw_type_registry::MessageForRmw,
        F: FnMut(&Svc::Request) -> Svc::Reply + 'static,
    {
        self.register_service_sized::<Svc, F, { crate::config::DEFAULT_RX_BUF_SIZE }, { crate::config::DEFAULT_RX_BUF_SIZE }>(service_name, callback)
    }

    /// Register a service callback with custom request/reply buffer sizes.
    pub fn register_service_sized<Svc, F, const REQ_BUF: usize, const REPLY_BUF: usize>(
        &mut self,
        service_name: &str,
        callback: F,
    ) -> Result<HandleId, NodeError>
    where
        Svc: RosService + 'static,
        Svc::Request: crate::rmw_type_registry::MessageForRmw,
        Svc::Reply: crate::rmw_type_registry::MessageForRmw,
        F: FnMut(&Svc::Request) -> Svc::Reply + 'static,
    {
        type Entry<Svc, F, const RQ: usize, const RP: usize> = SrvEntry<Svc, F, RQ, RP>;

        // Phase 212.K.7.7.b — register both halves of the service round-trip
        // under cyclonedds. No-op for other RMWs. Mirrors the K.7.6.b hook
        // on `Node::create_service_sized`.
        crate::rmw_type_registry::register_type::<Svc::Request>()?;
        crate::rmw_type_registry::register_type::<Svc::Reply>()?;

        let slot = self.next_entry_slot()?;
        let node_name: heapless::String<64> = self.node_name.clone();
        let ns: heapless::String<64> = self.namespace.clone();
        let mut info = ServiceInfo::new(service_name, Svc::SERVICE_NAME, Svc::SERVICE_HASH)
            .with_domain(self.domain_id)
            .with_namespace(&ns);
        if !node_name.is_empty() {
            info = info.with_node_name(&node_name);
        }
        let handle = self
            .session
            .create_service(&info, QoSProfile::services_default())
            .map_err(NodeError::Transport)?;

        let offset = self.arena_alloc::<Entry<Svc, F, REQ_BUF, REPLY_BUF>>()?;

        // SAFETY: same guarantees as register_subscription_sized.
        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut Entry<Svc, F, REQ_BUF, REPLY_BUF>;
            core::ptr::write(
                entry_ptr,
                Entry {
                    handle,
                    req_buffer: [0u8; REQ_BUF],
                    reply_buffer: [0u8; REPLY_BUF],
                    callback,
                    _phantom: PhantomData,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::Service,
            try_process: srv_try_process::<Svc, F, REQ_BUF, REPLY_BUF>,
            has_data: srv_has_data::<Svc, F, REQ_BUF, REPLY_BUF>,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::OnNewData,
            drop_fn: drop_entry::<Entry<Svc, F, REQ_BUF, REPLY_BUF>>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(service_name));
        Ok(HandleId(slot))
    }

    /// Phase 104.C.3.3.a — Node-aware variant of
    /// [`register_service_sized`](Self::register_service_sized).
    pub fn register_service_sized_on<Svc, F, const REQ_BUF: usize, const REPLY_BUF: usize>(
        &mut self,
        node_id: super::node_record::NodeId,
        service_name: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<HandleId, NodeError>
    where
        Svc: RosService + 'static,
        Svc::Request: crate::rmw_type_registry::MessageForRmw,
        Svc::Reply: crate::rmw_type_registry::MessageForRmw,
        F: FnMut(&Svc::Request) -> Svc::Reply + 'static,
    {
        type Entry<Svc, F, const RQ: usize, const RP: usize> = SrvEntry<Svc, F, RQ, RP>;

        // Phase 212.K.7.7.b — see `register_service_sized`.
        crate::rmw_type_registry::register_type::<Svc::Request>()?;
        crate::rmw_type_registry::register_type::<Svc::Reply>()?;

        let slot = self.next_entry_slot()?;
        let (node_name, ns, session_idx) = {
            let r = self
                .nodes
                .get(node_id.index())
                .ok_or(NodeError::InvalidSchedContextBinding)?;
            (r.name.clone(), r.namespace.clone(), r.session_idx)
        };
        let mut info = ServiceInfo::new(service_name, Svc::SERVICE_NAME, Svc::SERVICE_HASH)
            .with_domain(self.domain_id)
            .with_namespace(&ns);
        if !node_name.is_empty() {
            info = info.with_node_name(&node_name);
        }
        let handle = {
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            // Phase 193.5 — validate against the backend's supported policies
            // (no silent downgrade); request/reply effectively requires RELIABLE.
            qos.validate_against(session.supported_qos_policies())
                .map_err(NodeError::Transport)?;
            session
                .create_service(&info, qos)
                .map_err(NodeError::Transport)?
        };

        let offset = self.arena_alloc::<Entry<Svc, F, REQ_BUF, REPLY_BUF>>()?;
        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut Entry<Svc, F, REQ_BUF, REPLY_BUF>;
            core::ptr::write(
                entry_ptr,
                Entry {
                    handle,
                    req_buffer: [0u8; REQ_BUF],
                    reply_buffer: [0u8; REPLY_BUF],
                    callback,
                    _phantom: PhantomData,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::Service,
            try_process: srv_try_process::<Svc, F, REQ_BUF, REPLY_BUF>,
            has_data: srv_has_data::<Svc, F, REQ_BUF, REPLY_BUF>,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::OnNewData,
            drop_fn: drop_entry::<Entry<Svc, F, REQ_BUF, REPLY_BUF>>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(service_name));
        self.apply_node_default_sched(slot, Some(node_id), None);
        Ok(HandleId(slot))
    }

    /// Phase 104.C.3.3.a — Node-aware variant of
    /// [`register_service`](Self::register_service).
    pub fn register_service_on<Svc, F>(
        &mut self,
        node_id: super::node_record::NodeId,
        service_name: &str,
        callback: F,
    ) -> Result<HandleId, NodeError>
    where
        Svc: RosService + 'static,
        Svc::Request: crate::rmw_type_registry::MessageForRmw,
        Svc::Reply: crate::rmw_type_registry::MessageForRmw,
        F: FnMut(&Svc::Request) -> Svc::Reply + 'static,
    {
        self.register_service_sized_on::<
            Svc,
            F,
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
        >(node_id, service_name, QoSProfile::services_default(), callback)
    }

    // ========================================================================
    // Timer registration
    // ========================================================================

    /// Register a repeating timer callback.
    ///
    /// The callback fires every `period` milliseconds during [`spin_once()`](Self::spin_once).
    /// The timer delta is approximated by the `timeout_ms` argument to `spin_once`.
    pub fn register_timer<F>(
        &mut self,
        period: TimerDuration,
        callback: F,
    ) -> Result<HandleId, NodeError>
    where
        F: FnMut() + 'static,
    {
        let slot = self.next_entry_slot()?;
        let offset = self.arena_alloc::<TimerEntry<F>>()?;

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut TimerEntry<F>;
            core::ptr::write(
                entry_ptr,
                TimerEntry {
                    period_us: period.as_micros(),
                    elapsed_us: 0,
                    overruns: 0,
                    overruns_reported: 0,
                    oneshot: false,
                    fired: false,
                    cancelled: false,
                    overrun_policy: TimerOverrunPolicy::default(),
                    clock_source: TimerClockSource::Steady,
                    last_clock_ns: 0,
                    callback,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::Timer,
            try_process: timer_try_process::<F>,
            has_data: always_ready,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::Always,
            drop_fn: drop_entry::<TimerEntry<F>>,
        };
        self.emplace_entry(slot, meta, TraceName::TimerPeriod(period.as_micros()));
        Ok(HandleId(slot))
    }

    /// Register a repeating timer driven by a CLOCK rather than by the spin
    /// delta — phase-425 W4, the shape rclcpp spells
    /// `create_timer(node, clock, period, cb)`.
    ///
    /// [`register_timer`](Self::register_timer) is the wall timer: it consumes
    /// the executor's monotonic spin delta and no simulator can slow it down.
    /// This one reads `source` on every poll and advances by the difference, so
    /// a [`TimerClockSource::Ros`] timer follows `/clock` — it stops while the
    /// simulator is paused, halves with a bag replayed at 0.5x, and restarts
    /// its period on a backwards jump instead of stalling for the length of it.
    ///
    /// With no `/clock` source installed, a `Ros` timer reads system time and
    /// behaves like a wall timer with NTP steps, which is the same fallback
    /// `rclcpp::Clock` has: a node written for simulation still runs standalone.
    pub fn register_timer_on_clock<F>(
        &mut self,
        period: TimerDuration,
        source: TimerClockSource,
        callback: F,
    ) -> Result<HandleId, NodeError>
    where
        F: FnMut() + 'static,
    {
        let slot = self.next_entry_slot()?;
        let offset = self.arena_alloc::<TimerEntry<F>>()?;

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut TimerEntry<F>;
            core::ptr::write(
                entry_ptr,
                TimerEntry {
                    period_us: period.as_micros(),
                    elapsed_us: 0,
                    overruns: 0,
                    overruns_reported: 0,
                    oneshot: false,
                    fired: false,
                    cancelled: false,
                    overrun_policy: TimerOverrunPolicy::default(),
                    clock_source: source,
                    // Seeded HERE rather than on the first poll: a zero would
                    // make the first delta the whole epoch, which `Skip` would
                    // then coalesce into one immediate activation and `CatchUp`
                    // into a replay burst of ~10^9 periods.
                    last_clock_ns: source.now_ns(),
                    callback,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::Timer,
            try_process: timer_try_process::<F>,
            has_data: always_ready,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::Always,
            drop_fn: drop_entry::<TimerEntry<F>>,
        };
        self.emplace_entry(slot, meta, TraceName::TimerPeriod(period.as_micros()));
        Ok(HandleId(slot))
    }

    /// Register a one-shot timer callback.
    ///
    /// The callback fires once after `delay` milliseconds, then becomes inert.
    pub fn register_timer_oneshot<F>(
        &mut self,
        delay: TimerDuration,
        callback: F,
    ) -> Result<HandleId, NodeError>
    where
        F: FnMut() + 'static,
    {
        let slot = self.next_entry_slot()?;
        let offset = self.arena_alloc::<TimerEntry<F>>()?;

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut TimerEntry<F>;
            core::ptr::write(
                entry_ptr,
                TimerEntry {
                    period_us: delay.as_micros(),
                    elapsed_us: 0,
                    overruns: 0,
                    overruns_reported: 0,
                    oneshot: true,
                    fired: false,
                    cancelled: false,
                    overrun_policy: TimerOverrunPolicy::default(),
                    clock_source: TimerClockSource::Steady,
                    last_clock_ns: 0,
                    callback,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::Timer,
            try_process: timer_try_process::<F>,
            has_data: always_ready,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::Always,
            drop_fn: drop_entry::<TimerEntry<F>>,
        };
        self.emplace_entry(slot, meta, TraceName::TimerPeriod(delay.as_micros()));
        Ok(HandleId(slot))
    }

    /// Phase 273 (RFC-0047) — register a repeating timer callback bound to a
    /// specific node and optional callback group. The group name is threaded to
    /// `apply_node_default_sched` so the seeded `group_sched_table` assigns
    /// the timer's callback to the group's `SchedContext`. When `group` is
    /// `None` the node's `default_sched` applies (phase-272 behavior).
    ///
    /// This is the executor-level primitive called by the Rust `_in` API
    /// (`NodeCtx::create_timer_in`) and the C/C++ group-aware timer FFI.
    pub fn register_timer_on<F>(
        &mut self,
        node_id: Option<super::node_record::NodeId>,
        period: TimerDuration,
        callback: F,
        group: Option<&str>,
    ) -> Result<HandleId, NodeError>
    where
        F: FnMut() + 'static,
    {
        let slot = self.next_entry_slot()?;
        let offset = self.arena_alloc::<TimerEntry<F>>()?;

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut TimerEntry<F>;
            core::ptr::write(
                entry_ptr,
                TimerEntry {
                    period_us: period.as_micros(),
                    elapsed_us: 0,
                    overruns: 0,
                    overruns_reported: 0,
                    oneshot: false,
                    fired: false,
                    cancelled: false,
                    overrun_policy: TimerOverrunPolicy::default(),
                    clock_source: TimerClockSource::Steady,
                    last_clock_ns: 0,
                    callback,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::Timer,
            try_process: timer_try_process::<F>,
            has_data: always_ready,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::Always,
            drop_fn: drop_entry::<TimerEntry<F>>,
        };
        self.emplace_entry(slot, meta, TraceName::TimerPeriod(period.as_micros()));
        // Phase 273 — apply group sched binding (group > node default > SC 0).
        self.apply_node_default_sched(slot, node_id, group);
        Ok(HandleId(slot))
    }

    // ========================================================================
    // Raw callback registration (for C API)
    // ========================================================================

    /// The kept C-FFI subscription core (Phase 189.M2.b): registers a
    /// raw `RawSubscriptionCallback` fn-ptr + `context` against an
    /// optional node's session. The Rust ergonomic surface is the
    /// `node.subscription(t)` builder (closures); this is the single
    /// primitive the `nros-c` thin wrapper lowers to. `node_id == None`
    /// is the legacy single-node path.
    #[allow(clippy::too_many_arguments)]
    pub fn add_arena_subscription_c_callback<const RX_BUF: usize>(
        &mut self,
        node_id: Option<super::node_record::NodeId>,
        topic_name: &str,
        type_name: &str,
        type_hash: &str,
        qos: QoSProfile,
        callback: RawSubscriptionCallback,
        context: *mut core::ffi::c_void,
        group: Option<&str>,
        // phase-402 W2 / issue 0896 — bytes the caller expects to receive; 0 =
        // no opinion. A size-classing backend (zenoh-pico) routes on this, and
        // a subscription that states nothing takes the SMALL class whatever its
        // message type. The C path had no way to say it until the options
        // struct existed.
        rx_buffer_hint: usize,
    ) -> Result<HandleId, NodeError> {
        let slot = self.next_entry_slot()?;
        let (node_name, ns, session_idx) = match node_id {
            Some(id) => {
                let r = self
                    .nodes
                    .get(id.index())
                    .ok_or(NodeError::InvalidSchedContextBinding)?;
                (r.name.clone(), r.namespace.clone(), r.session_idx)
            }
            None => (self.node_name.clone(), self.namespace.clone(), 0u8),
        };
        let mut topic = TopicInfo::new(topic_name, type_name, type_hash)
            .with_domain(self.domain_id)
            .with_namespace(&ns);
        if !node_name.is_empty() {
            topic = topic.with_node_name(&node_name);
        }
        // phase-402 W2 — only when the caller actually stated one:
        // `with_rx_buffer_hint(0)` would be a claim of "zero bytes", not
        // "no opinion".
        if rx_buffer_hint != 0 {
            topic = topic.with_rx_buffer_hint(rx_buffer_hint);
        }
        let handle = {
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            session
                .create_subscription(&topic, qos)
                .map_err(NodeError::Transport)?
        };

        // phase-403 W3/W5 -- size the arena slot from the TYPE, not from the
        // image-wide default. `rx_buffer_hint` already arrives here (phase-402
        // routed it to the backend's payload class and stopped); spending it on
        // the allocation as well is what makes the buffer per-type.
        //
        // This is the whole saving on the raw path. `RX_BUF` is
        // DEFAULT_RX_BUF_SIZE, so every slot -- publishers and timers included --
        // was charged the largest subscription's buffer. Measured on
        // mr-canhubk344: 36 handles at the correct 2052-byte bound wanted a
        // 242096-byte arena of a 77968-byte region; only 13 of those handles
        // receive anything.
        //
        // 0 keeps the old behaviour, and it means "this CALLER stated nothing"
        // rather than "this type is unbounded" -- an unbounded type is a build
        // error now, so a zero here is a caller that did not ask, never a type
        // that could not answer.
        let rx_bytes = if rx_buffer_hint != 0 {
            rx_buffer_hint
        } else {
            RX_BUF
        };

        let (_slot_count, trailing_bytes) = buffered_region_size(qos.depth, rx_bytes);

        let (entry_offset, trailing_offset) =
            self.arena_alloc_with_trailing::<SubBufferedRawCEntry>(trailing_bytes)?;

        let buf_ptr = unsafe { (self.arena.as_mut_ptr() as *mut u8).add(trailing_offset) };

        let buffer = if qos.depth <= 1 {
            BufferStrategy::Triple(unsafe { TripleBuffer::init(buf_ptr, rx_bytes) })
        } else {
            BufferStrategy::Ring(unsafe { SpscRing::init(buf_ptr, rx_bytes, qos.depth as usize) })
        };

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(entry_offset) as *mut SubBufferedRawCEntry;
            core::ptr::write(
                entry_ptr,
                SubBufferedRawCEntry {
                    handle,
                    buffer,
                    callback,
                    context,
                },
            );
        }

        let meta = CallbackMeta {
            offset: entry_offset,
            kind: EntryKind::Subscription,
            try_process: sub_buffered_raw_c_try_process,
            has_data: sub_buffered_raw_c_has_data,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::OnNewData,
            drop_fn: drop_entry::<SubBufferedRawCEntry>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(topic_name));
        self.apply_node_default_sched(slot, node_id, group);
        Ok(HandleId(slot))
    }

    /// Phase 189.M3.4 — register a raw C-fn-ptr subscription whose callback
    /// also receives the sample's wire **attachment**
    /// ([`RawSubscriptionInfoCallback`]: `(data, len, attachment, att_len,
    /// context)`) — the C analog of the Rust
    /// `node.subscription(t).generic(..).message_info()` builder. Backs the C
    /// FFI `nros_executor_add_subscription_raw_with_info`. Flat per-entry
    /// payload + attachment buffers (cap [`RAW_INFO_ATT_CAP`](super::arena::RAW_INFO_ATT_CAP));
    /// one sample per `spin_once`.
    #[allow(clippy::too_many_arguments)]
    pub fn add_arena_subscription_c_info_callback<const RX_BUF: usize>(
        &mut self,
        node_id: Option<super::node_record::NodeId>,
        topic_name: &str,
        type_name: &str,
        type_hash: &str,
        qos: QoSProfile,
        callback: RawSubscriptionInfoCallback,
        context: *mut core::ffi::c_void,
        // phase-408 W5a/W5b — bytes the caller expects to receive; 0 = no
        // opinion. It now buys BOTH halves: the BACKEND's payload size-class
        // routing (W5a, below) and the ARENA slot (W5b, further down). `RX_BUF`
        // survives only as the fallback for a caller that states nothing, which
        // is exactly its role on the plain C path.
        rx_buffer_hint: usize,
    ) -> Result<HandleId, NodeError> {
        type Entry = SubBufferedRawInfoCEntry;

        let slot = self.next_entry_slot()?;
        let (node_name, ns, session_idx) = match node_id {
            Some(id) => {
                let r = self
                    .nodes
                    .get(id.index())
                    .ok_or(NodeError::InvalidSchedContextBinding)?;
                (r.name.clone(), r.namespace.clone(), r.session_idx)
            }
            None => (self.node_name.clone(), self.namespace.clone(), 0u8),
        };
        let mut topic = TopicInfo::new(topic_name, type_name, type_hash)
            .with_domain(self.domain_id)
            .with_namespace(&ns);
        if !node_name.is_empty() {
            topic = topic.with_node_name(&node_name);
        }
        // phase-408 W5a — only when the caller actually stated one:
        // `with_rx_buffer_hint(0)` would be a claim of "zero bytes", not
        // "no opinion".
        if rx_buffer_hint != 0 {
            topic = topic.with_rx_buffer_hint(rx_buffer_hint);
        }
        let handle = {
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            session
                .create_subscription(&topic, qos)
                .map_err(NodeError::Transport)?
        };

        // phase-408 W5b — the hint sizes the ARENA too, not just the backend's
        // size class. ONE slot, not `buffered_region_size`: this entry hands the
        // sample's attachment to the callback alongside the payload, so it
        // dispatches exactly one sample per spin and has no queue to size. 0
        // still means "this caller stated nothing", so `RX_BUF` — which is
        // `DEFAULT_RX_BUF_SIZE` at every call site — is what it falls back to,
        // and an unhinted registration claims the same bytes it always did.
        let rx_bytes = if rx_buffer_hint != 0 {
            rx_buffer_hint
        } else {
            RX_BUF
        };

        let (offset, trailing_offset) = self.arena_alloc_with_trailing::<Entry>(rx_bytes)?;
        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let buf = super::arena::TrailingBuf::init(arena_ptr.add(trailing_offset), rx_bytes);
            let entry_ptr = arena_ptr.add(offset) as *mut Entry;
            core::ptr::write(
                entry_ptr,
                Entry {
                    handle,
                    buffer: buf,
                    att: [0u8; super::arena::RAW_INFO_ATT_CAP],
                    callback,
                    context,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::Subscription,
            try_process: sub_buffered_raw_info_c_try_process,
            has_data: sub_buffered_raw_info_c_has_data,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::OnNewData,
            drop_fn: drop_entry::<Entry>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(topic_name));
        self.apply_node_default_sched(slot, node_id, None);
        Ok(HandleId(slot))
    }

    /// Phase 269 W3 — register a raw C-fn-ptr subscription whose callback
    /// ALSO surfaces the sample's E2E integrity status (CRC + sequence gap/dup)
    /// alongside the CDR bytes — the C/C++ component-callback analog of Rust's
    /// `register_subscription_buffered_raw_safety_on` (`FnMut(&[u8], &IntegrityStatus)`).
    ///
    /// The executor validates the sample via `take_validated` and unpacks the
    /// [`nros_rmw::IntegrityStatus`] into three plain scalars (gap, duplicate,
    /// crc_valid) before calling `callback`. This avoids introducing a
    /// cbindgen-visible struct at the executor layer; the C/C++ headers pack them
    /// back into their local integrity-status typedef.
    ///
    /// Requires the `safety-e2e` feature. Backed by [`SubBufferedRawSafetyCEntry`].
    #[cfg(feature = "safety-e2e")]
    #[allow(clippy::too_many_arguments)]
    pub fn add_arena_subscription_c_validated_callback<const RX_BUF: usize>(
        &mut self,
        node_id: Option<super::node_record::NodeId>,
        topic_name: &str,
        type_name: &str,
        type_hash: &str,
        qos: QoSProfile,
        callback: super::types::RawSubscriptionSafetyCallback,
        context: *mut core::ffi::c_void,
        // phase-408 W5a/W5b — bytes the caller expects to receive; 0 = no
        // opinion. It now buys BOTH halves: the BACKEND's payload size-class
        // routing (W5a, below) and the ARENA slot (W5b, further down). `RX_BUF`
        // survives only as the fallback for a caller that states nothing, which
        // is exactly its role on the plain C path.
        rx_buffer_hint: usize,
    ) -> Result<HandleId, NodeError> {
        use super::arena::{
            SubBufferedRawSafetyCEntry, TrailingBuf, sub_buffered_raw_safety_c_has_data,
            sub_buffered_raw_safety_c_try_process,
        };
        type Entry = SubBufferedRawSafetyCEntry;

        let slot = self.next_entry_slot()?;
        let (node_name, ns, session_idx) = match node_id {
            Some(id) => {
                let r = self
                    .nodes
                    .get(id.index())
                    .ok_or(NodeError::InvalidSchedContextBinding)?;
                (r.name.clone(), r.namespace.clone(), r.session_idx)
            }
            None => (self.node_name.clone(), self.namespace.clone(), 0u8),
        };
        let mut topic = TopicInfo::new(topic_name, type_name, type_hash)
            .with_domain(self.domain_id)
            .with_namespace(&ns);
        if !node_name.is_empty() {
            topic = topic.with_node_name(&node_name);
        }
        // phase-408 W5a — only when the caller actually stated one:
        // `with_rx_buffer_hint(0)` would be a claim of "zero bytes", not
        // "no opinion".
        if rx_buffer_hint != 0 {
            topic = topic.with_rx_buffer_hint(rx_buffer_hint);
        }
        let handle = {
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            session
                .create_subscription(&topic, qos)
                .map_err(NodeError::Transport)?
        };

        // phase-408 W5b — same as the info sibling: one flat slot in the
        // trailing region, sized from the hint, `RX_BUF` only as the
        // stated-nothing fallback. The integrity status is per-sample side
        // data, so this path also dispatches one sample per spin and wants no
        // queue.
        let rx_bytes = if rx_buffer_hint != 0 {
            rx_buffer_hint
        } else {
            RX_BUF
        };

        let (offset, trailing_offset) = self.arena_alloc_with_trailing::<Entry>(rx_bytes)?;
        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let buf = TrailingBuf::init(arena_ptr.add(trailing_offset), rx_bytes);
            let entry_ptr = arena_ptr.add(offset) as *mut Entry;
            core::ptr::write(
                entry_ptr,
                Entry {
                    handle,
                    buffer: buf,
                    callback,
                    context,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::Subscription,
            try_process: sub_buffered_raw_safety_c_try_process,
            has_data: sub_buffered_raw_safety_c_has_data,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::OnNewData,
            drop_fn: drop_entry::<Entry>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(topic_name));
        self.apply_node_default_sched(slot, node_id, None);
        Ok(HandleId(slot))
    }

    /// Register a raw (untyped) service callback.
    ///
    /// Register a raw (untyped) service callback with the default buffer size.
    ///
    /// The callback receives and produces CDR bytes without typed
    /// deserialization/serialization. Used by the C API wrapper.
    pub fn register_service_raw(
        &mut self,
        service_name: &str,
        service_type: &str,
        service_hash: &str,
        callback: RawServiceCallback,
        context: *mut core::ffi::c_void,
    ) -> Result<HandleId, NodeError> {
        self.register_service_raw_sized::<{ crate::config::DEFAULT_RX_BUF_SIZE }, { crate::config::DEFAULT_RX_BUF_SIZE }>(
            service_name,
            service_type,
            service_hash,
            QoSProfile::services_default(),
            callback,
            context,
        )
    }

    /// Register a raw (untyped) service callback with custom buffer sizes + QoS.
    ///
    /// `REQ_BUF` and `REPLY_BUF` set the stack-allocated CDR buffers
    /// for the request and reply respectively. Increase for services
    /// with large payloads (e.g., parameter services). `qos` applies to both
    /// the request + reply endpoints (Phase 193.2c).
    #[allow(clippy::too_many_arguments)]
    pub fn register_service_raw_sized<const REQ_BUF: usize, const REPLY_BUF: usize>(
        &mut self,
        service_name: &str,
        service_type: &str,
        service_hash: &str,
        qos: QoSProfile,
        callback: RawServiceCallback,
        context: *mut core::ffi::c_void,
    ) -> Result<HandleId, NodeError> {
        self.register_service_raw_sized_inner::<REQ_BUF, REPLY_BUF>(
            None,
            service_name,
            service_type,
            service_hash,
            qos,
            callback,
            context,
        )
    }

    /// Phase 104.C.3.3.a — Node-aware variant of
    /// [`register_service_raw_sized`]. C-FFI path.
    #[allow(clippy::too_many_arguments)]
    pub fn register_service_raw_sized_on<const REQ_BUF: usize, const REPLY_BUF: usize>(
        &mut self,
        node_id: super::node_record::NodeId,
        service_name: &str,
        service_type: &str,
        service_hash: &str,
        qos: QoSProfile,
        callback: RawServiceCallback,
        context: *mut core::ffi::c_void,
    ) -> Result<HandleId, NodeError> {
        self.register_service_raw_sized_inner::<REQ_BUF, REPLY_BUF>(
            Some(node_id),
            service_name,
            service_type,
            service_hash,
            qos,
            callback,
            context,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn register_service_raw_sized_inner<const REQ_BUF: usize, const REPLY_BUF: usize>(
        &mut self,
        node_id: Option<super::node_record::NodeId>,
        service_name: &str,
        service_type: &str,
        service_hash: &str,
        qos: QoSProfile,
        callback: RawServiceCallback,
        context: *mut core::ffi::c_void,
    ) -> Result<HandleId, NodeError> {
        let slot = self.next_entry_slot()?;
        let (node_name, ns, session_idx) = match node_id {
            Some(id) => {
                let r = self
                    .nodes
                    .get(id.index())
                    .ok_or(NodeError::InvalidSchedContextBinding)?;
                (r.name.clone(), r.namespace.clone(), r.session_idx)
            }
            None => (self.node_name.clone(), self.namespace.clone(), 0u8),
        };
        let mut info = ServiceInfo::new(service_name, service_type, service_hash)
            .with_domain(self.domain_id)
            .with_namespace(&ns);
        if !node_name.is_empty() {
            info = info.with_node_name(&node_name);
        }
        let handle = {
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            // Phase 193.5 — validate against the backend's supported policies
            // (no silent downgrade); request/reply effectively requires RELIABLE.
            qos.validate_against(session.supported_qos_policies())
                .map_err(NodeError::Transport)?;
            session
                .create_service(&info, qos)
                .map_err(NodeError::Transport)?
        };

        let offset = self.arena_alloc::<SrvRawEntry<REQ_BUF, REPLY_BUF>>()?;

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut SrvRawEntry<REQ_BUF, REPLY_BUF>;
            core::ptr::write(
                entry_ptr,
                SrvRawEntry {
                    handle,
                    req_buffer: [0u8; REQ_BUF],
                    reply_buffer: [0u8; REPLY_BUF],
                    callback,
                    context,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::Service,
            try_process: srv_raw_try_process::<REQ_BUF, REPLY_BUF>,
            has_data: srv_raw_has_data::<REQ_BUF, REPLY_BUF>,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::OnNewData,
            drop_fn: drop_entry::<SrvRawEntry<REQ_BUF, REPLY_BUF>>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(service_name));
        self.apply_node_default_sched(slot, node_id, None);
        Ok(HandleId(slot))
    }

    // ========================================================================
    // Raw service client registration (Phase 82)
    // ========================================================================

    /// Register a raw (untyped) service client with the default reply
    /// buffer size.
    ///
    /// The client is owned by the executor's arena. Each `spin_once`
    /// dispatch polls the in-flight reply slot via `take_response_raw`
    /// and fires the registered callback when the response arrives.
    /// Used by the C API thin wrapper — see Phase 82.
    pub fn register_service_client_raw(
        &mut self,
        service_name: &str,
        service_type: &str,
        service_hash: &str,
        callback: Option<RawResponseCallback>,
        context: *mut core::ffi::c_void,
    ) -> Result<HandleId, NodeError> {
        self.register_service_client_raw_sized::<{ crate::config::DEFAULT_RX_BUF_SIZE }>(
            service_name,
            service_type,
            service_hash,
            QoSProfile::services_default(),
            callback,
            context,
        )
    }

    /// Register a raw service client with a custom reply buffer size + QoS.
    ///
    /// `qos` applies to the client's request + reply endpoints (Phase 193.3b);
    /// defaults to [`QoSProfile::services_default`] via the convenience
    /// wrapper.
    #[allow(clippy::too_many_arguments)]
    pub fn register_service_client_raw_sized<const REPLY_BUF: usize>(
        &mut self,
        service_name: &str,
        service_type: &str,
        service_hash: &str,
        qos: QoSProfile,
        callback: Option<RawResponseCallback>,
        context: *mut core::ffi::c_void,
    ) -> Result<HandleId, NodeError> {
        self.register_service_client_raw_sized_inner::<REPLY_BUF>(
            None,
            service_name,
            service_type,
            service_hash,
            qos,
            callback,
            context,
        )
    }

    /// Phase 104.C.3.3.a — Node-aware variant of
    /// [`register_service_client_raw_sized`]. Routes the client
    /// creation through the named Node's session.
    #[allow(clippy::too_many_arguments)]
    pub fn register_service_client_raw_sized_on<const REPLY_BUF: usize>(
        &mut self,
        node_id: super::node_record::NodeId,
        service_name: &str,
        service_type: &str,
        service_hash: &str,
        qos: QoSProfile,
        callback: Option<RawResponseCallback>,
        context: *mut core::ffi::c_void,
    ) -> Result<HandleId, NodeError> {
        self.register_service_client_raw_sized_inner::<REPLY_BUF>(
            Some(node_id),
            service_name,
            service_type,
            service_hash,
            qos,
            callback,
            context,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn register_service_client_raw_sized_inner<const REPLY_BUF: usize>(
        &mut self,
        node_id: Option<super::node_record::NodeId>,
        service_name: &str,
        service_type: &str,
        service_hash: &str,
        qos: QoSProfile,
        callback: Option<RawResponseCallback>,
        context: *mut core::ffi::c_void,
    ) -> Result<HandleId, NodeError> {
        let slot = self.next_entry_slot()?;
        let (node_name, ns, session_idx) = match node_id {
            Some(id) => {
                let r = self
                    .nodes
                    .get(id.index())
                    .ok_or(NodeError::InvalidSchedContextBinding)?;
                (r.name.clone(), r.namespace.clone(), r.session_idx)
            }
            None => (self.node_name.clone(), self.namespace.clone(), 0u8),
        };
        let mut info = ServiceInfo::new(service_name, service_type, service_hash)
            .with_domain(self.domain_id)
            .with_namespace(&ns);
        if !node_name.is_empty() {
            info = info.with_node_name(&node_name);
        }
        let handle = {
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            // Phase 193.5 — validate against the backend's supported policies
            // (no silent downgrade); request/reply effectively requires RELIABLE.
            qos.validate_against(session.supported_qos_policies())
                .map_err(NodeError::Transport)?;
            session
                .create_client(&info, qos)
                .map_err(|_| NodeError::Transport(TransportError::ServiceClientCreationFailed))?
        };

        let offset = self.arena_alloc::<ServiceClientRawArenaEntry<REPLY_BUF>>()?;
        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut ServiceClientRawArenaEntry<REPLY_BUF>;
            core::ptr::write(
                entry_ptr,
                ServiceClientRawArenaEntry {
                    handle,
                    reply_buffer: [0u8; REPLY_BUF],
                    pending: false,
                    reply_ready: core::sync::atomic::AtomicBool::new(false),
                    callback,
                    context,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::ServiceClient,
            try_process: service_client_raw_try_process::<REPLY_BUF>,
            has_data: always_ready,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::Always,
            drop_fn: drop_entry::<ServiceClientRawArenaEntry<REPLY_BUF>>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(service_name));
        self.apply_node_default_sched(slot, node_id, None);
        Ok(HandleId(slot))
    }

    /// RFC-0041 / Phase 239.1 — register a **typed callback** service client.
    /// The reply is eager-drained at `spin_once` and dispatched to `callback` as
    /// a deserialized `Svc::Reply`. Returns the scheduling [`HandleId`] and a
    /// `*mut` to the arena entry's send header (used to build the typed
    /// [`ServiceClientCallback`](super::handles::ServiceClientCallback)).
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn register_service_client_callback<Svc, F, const REPLY_BUF: usize>(
        &mut self,
        node_id: Option<super::node_record::NodeId>,
        service_name: &str,
        service_type: &str,
        service_hash: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<(HandleId, *mut ServiceClientSendHeader<REPLY_BUF>), NodeError>
    where
        Svc: nros_core::RosService + 'static,
        F: FnMut(&Svc::Reply) + 'static,
    {
        let slot = self.next_entry_slot()?;
        let (node_name, ns, session_idx) = match node_id {
            Some(id) => {
                let r = self
                    .nodes
                    .get(id.index())
                    .ok_or(NodeError::InvalidSchedContextBinding)?;
                (r.name.clone(), r.namespace.clone(), r.session_idx)
            }
            None => (self.node_name.clone(), self.namespace.clone(), 0u8),
        };
        let mut info = ServiceInfo::new(service_name, service_type, service_hash)
            .with_domain(self.domain_id)
            .with_namespace(&ns);
        if !node_name.is_empty() {
            info = info.with_node_name(&node_name);
        }
        let handle = {
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            qos.validate_against(session.supported_qos_policies())
                .map_err(NodeError::Transport)?;
            session
                .create_client(&info, qos)
                .map_err(|_| NodeError::Transport(TransportError::ServiceClientCreationFailed))?
        };

        let offset = self.arena_alloc::<ServiceClientCallbackEntry<Svc, F, REPLY_BUF>>()?;
        let hdr_ptr = unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr =
                arena_ptr.add(offset) as *mut ServiceClientCallbackEntry<Svc, F, REPLY_BUF>;
            core::ptr::write(
                entry_ptr,
                ServiceClientCallbackEntry {
                    hdr: ServiceClientSendHeader {
                        handle,
                        reply_buffer: [0u8; REPLY_BUF],
                        pending: false,
                        reply_ready: core::sync::atomic::AtomicBool::new(false),
                    },
                    callback,
                    _phantom: core::marker::PhantomData,
                },
            );
            &mut (*entry_ptr).hdr as *mut ServiceClientSendHeader<REPLY_BUF>
        };

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::ServiceClient,
            try_process: service_client_callback_try_process::<Svc, F, REPLY_BUF>,
            has_data: always_ready,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::Always,
            drop_fn: drop_entry::<ServiceClientCallbackEntry<Svc, F, REPLY_BUF>>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(service_name));
        self.apply_node_default_sched(slot, node_id, None);
        Ok((HandleId(slot), hdr_ptr))
    }

    // ========================================================================
    // Guard condition registration
    // ========================================================================

    /// Register a guard condition with a callback.
    ///
    /// Returns both the [`HandleId`] for trigger configuration and a
    /// [`GuardCondition`] for triggering from other threads.
    pub fn register_guard_condition<F>(
        &mut self,
        callback: F,
    ) -> Result<(HandleId, GuardCondition), NodeError>
    where
        F: FnMut() + 'static,
    {
        let slot = self.next_entry_slot()?;
        let offset = self.arena_alloc::<GuardConditionEntry<F>>()?;

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut GuardConditionEntry<F>;
            core::ptr::write(
                entry_ptr,
                GuardConditionEntry {
                    flag: portable_atomic::AtomicBool::new(false),
                    callback,
                },
            );

            // Create a handle pointing to the flag in the arena
            let flag_ptr = &(*entry_ptr).flag as *const portable_atomic::AtomicBool;
            #[allow(unused_mut)]
            let mut guard_handle = GuardCondition::new(flag_ptr);
            // Phase 124.B.5 — wire the wake callback so trigger()
            // also signals the executor's wake_cv.
            #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
            {
                let ctx = self.wake_ctx_ptr();
                guard_handle.set_wake_cb(nros_rmw_runtime_wake_cb, ctx);
            }

            let meta = CallbackMeta {
                offset,
                kind: EntryKind::GuardCondition,
                try_process: guard_try_process::<F>,
                has_data: guard_has_data::<F>,
                pre_sample: no_pre_sample,
                invocation: InvocationMode::OnNewData,
                drop_fn: drop_entry::<GuardConditionEntry<F>>,
            };
            self.emplace_entry(slot, meta, TraceName::Slot("guard", slot));

            Ok((HandleId(slot), guard_handle))
        }
    }

    // ========================================================================
    // Timer control methods
    // ========================================================================

    /// Cancel a timer. A cancelled timer will not fire but still accumulates
    /// elapsed time. The timer can be restarted with [`reset_timer()`](Self::reset_timer).
    pub fn cancel_timer(&mut self, id: HandleId) -> Result<(), NodeError> {
        let meta = self
            .entries
            .get(id.0)
            .and_then(|e| e.as_ref())
            .ok_or(NodeError::BufferTooSmall)?;
        if !matches!(meta.kind, EntryKind::Timer) {
            return Err(NodeError::BufferTooSmall);
        }
        let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
        // SAFETY: meta.offset points to a valid TimerEntry<F> which shares
        // layout with TimerHeader for its initial fields (both #[repr(C)]).
        let header = unsafe { &mut *(arena_ptr.add(meta.offset) as *mut TimerHeader) };
        header.cancelled = true;
        Ok(())
    }

    /// Reset a timer. Clears the cancelled state and resets the elapsed time
    /// to zero, so the timer starts a fresh period.
    pub fn reset_timer(&mut self, id: HandleId) -> Result<(), NodeError> {
        let meta = self
            .entries
            .get(id.0)
            .and_then(|e| e.as_ref())
            .ok_or(NodeError::BufferTooSmall)?;
        if !matches!(meta.kind, EntryKind::Timer) {
            return Err(NodeError::BufferTooSmall);
        }
        let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
        let header = unsafe { &mut *(arena_ptr.add(meta.offset) as *mut TimerHeader) };
        header.cancelled = false;
        header.elapsed_us = 0;
        Ok(())
    }

    /// Check if a timer is cancelled.
    pub fn timer_is_canceled(&self, id: HandleId) -> bool {
        let meta = match self.entries.get(id.0).and_then(|e| e.as_ref()) {
            Some(m) if matches!(m.kind, EntryKind::Timer) => m,
            _ => return false,
        };
        let arena_ptr = self.arena.as_ptr() as *const u8;
        let header = unsafe { &*(arena_ptr.add(meta.offset) as *const TimerHeader) };
        header.cancelled
    }

    /// Get the period of a timer in milliseconds (truncated — see
    /// [`Self::timer_period_us`]), or `None` if the handle is not a
    /// valid timer.
    pub fn timer_period_ms(&self, id: HandleId) -> Option<u64> {
        let meta = self
            .entries
            .get(id.0)
            .and_then(|e| e.as_ref())
            .filter(|m| matches!(m.kind, EntryKind::Timer))?;
        let arena_ptr = self.arena.as_ptr() as *const u8;
        let header = unsafe { &*(arena_ptr.add(meta.offset) as *const TimerHeader) };
        Some(header.period_us / 1000)
    }

    /// Get the period of a timer in microseconds, or `None` if the
    /// handle is not a valid timer.
    pub fn timer_period_us(&self, id: HandleId) -> Option<u64> {
        let meta = self
            .entries
            .get(id.0)
            .and_then(|e| e.as_ref())
            .filter(|m| matches!(m.kind, EntryKind::Timer))?;
        let arena_ptr = self.arena.as_ptr() as *const u8;
        // SAFETY: same layout invariant as `timer_period_ms`.
        let header = unsafe { &*(arena_ptr.add(meta.offset) as *const TimerHeader) };
        Some(header.period_us)
    }

    /// Set a timer's overrun policy (issue #505). Timers default to
    /// [`TimerOverrunPolicy::Skip`]; switch to
    /// [`TimerOverrunPolicy::CatchUp`] for timers whose every activation
    /// is a unit of work that must not be lost.
    pub fn set_timer_overrun_policy(
        &mut self,
        id: HandleId,
        policy: TimerOverrunPolicy,
    ) -> Result<(), NodeError> {
        let meta = self
            .entries
            .get(id.0)
            .and_then(|e| e.as_ref())
            .ok_or(NodeError::BufferTooSmall)?;
        if !matches!(meta.kind, EntryKind::Timer) {
            return Err(NodeError::BufferTooSmall);
        }
        let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
        // SAFETY: same layout invariant as `cancel_timer`.
        let header = unsafe { &mut *(arena_ptr.add(meta.offset) as *mut TimerHeader) };
        header.overrun_policy = policy;
        Ok(())
    }

    /// Periods this timer missed and dropped under
    /// [`TimerOverrunPolicy::Skip`] (issue #505), or `None` if the handle
    /// is not a valid timer.
    ///
    /// Monotonic and saturating. A growing count is the on-target signal
    /// that a tier is not keeping up with its declared cadence — the
    /// symptom an external observer would otherwise have to infer by
    /// differencing timestamps.
    pub fn timer_overruns(&self, id: HandleId) -> Option<u32> {
        let meta = self
            .entries
            .get(id.0)
            .and_then(|e| e.as_ref())
            .filter(|m| matches!(m.kind, EntryKind::Timer))?;
        let arena_ptr = self.arena.as_ptr() as *const u8;
        let header = unsafe { &*(arena_ptr.add(meta.offset) as *const TimerHeader) };
        Some(header.overruns)
    }

    // ========================================================================
    // spin_once (three-phase: readiness -> trigger -> dispatch)
    // ========================================================================

    /// Drive I/O and dispatch registered callbacks once.
    ///
    /// Three-phase execution:
    /// 1. **Readiness scan** — query each handle's `has_data()`.
    /// 2. **Trigger evaluation** — check if the executor-level trigger passes.
    /// 3. **Dispatch** — invoke callbacks according to their `InvocationMode`.
    ///
    /// Returns a [`SpinOnceResult`] with counts of processed items and errors.
    ///
    /// # Arguments
    /// * `timeout` — upper bound on the I/O wait. Saturated at
    ///   `i32::MAX` ms (~24 days) for the underlying transport call.
    ///
    /// Phase 84.D7: unified on `core::time::Duration`. The previous
    /// `timeout_ms: i32` signature had a latent footgun where
    /// `spin_once(-1)` silently froze timers while still polling I/O;
    /// `Duration` has no negative sentinel.
    /// Consecutive `drive_io` failures on the primary session (issue 0324).
    ///
    /// `0` means the last drive succeeded. A value that keeps climbing across
    /// spins is a session that is no longer doing I/O — router gone, lease
    /// expired, socket closed — which otherwise presents as a node that spins
    /// `Ok(())` forever while publishing nowhere and firing no callbacks.
    ///
    /// Transient failures happen, so a single non-zero reading is not a fault;
    /// a threshold (say, "more than a few consecutive spins") is the useful
    /// signal. Extra (bridge / multi-domain) sessions are best-effort and are
    /// deliberately NOT counted here.
    pub fn session_io_failures(&self) -> u32 {
        self.consecutive_io_failures
    }

    /// Whether the primary session drove I/O successfully on the last spin.
    ///
    /// Convenience over [`Self::session_io_failures`] for the common
    /// "is my transport alive?" check.
    pub fn session_io_healthy(&self) -> bool {
        self.consecutive_io_failures == 0
    }

    pub fn spin_once(&mut self, timeout: core::time::Duration) -> SpinOnceResult {
        let timeout_ms = timeout.as_millis().min(i32::MAX as u128) as i32;

        // phase-425 W3b — bring the `/clock` subscription in line with
        // `use_sim_time`. One bool comparison in the settled case; the work only
        // happens on a transition, or while a request is waiting for its first
        // node to exist.
        #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
        self.reconcile_ros_time_source();

        // Release jitter, measured HERE rather than in `spin_period`, because
        // this is the function every driver goes through.
        //
        // The first version instrumented `spin_period` only, and that was the
        // same mistake issue 0736 records one layer down: a measurement placed
        // on a path no shipped image takes. `nros-cpp`'s tier trampoline paces
        // itself with a `spin_once` loop at `spin_period_us` (see its
        // `run_components` docs, step 4), so `spin_period` is not what the C++
        // entry -- and therefore not what the Zephyr lane -- actually calls.
        // The field recorded zero forever while claiming to measure cadence.
        //
        // `timeout` is the caller's intended pacing quantum: `spin_period`
        // passes its period, and the tier loop passes `spin_period_us`. A wake
        // that arrives later than that is late by the difference, which is the
        // `cyclictest` quantity. A zero timeout means "poll, no cadence
        // claimed", so it is not judged.
        self.record_release_jitter(timeout);

        // issue 0900 — one-shot advisory when the arena is far larger than what
        // registered. First spin rather than "end of registration", because
        // there is no such point: an app may register lazily.
        super::arena::maybe_report_arena_headroom(self.arena_used, self.arena.len());

        // phase-412 -- the same moment, recorded where a board with no log sink
        // can still be asked. Reaching this stage is the positive result the
        // advisory above cannot deliver on the island: registration completed.
        crate::boot_report::checkpoint(crate::boot_report::Stage::FirstSpin);

        // Phase 110.0 — cap against the backend's next internal-event
        // deadline (lease keepalive, heartbeat, ACK-NACK timeout, ...).
        // Default backend impl returns `None`, so this is a no-op
        // unless the active backend opts in.
        #[allow(unused_variables)]
        let timeout_ms = match self.session.next_deadline_ms() {
            Some(next) => timeout_ms.min(next.min(i32::MAX as u32) as i32),
            None => timeout_ms,
        };

        // Wall-clock-accurate timer accumulation. Measure real time
        // since the previous `spin_once` exited (or, on the first call,
        // since `drive_io` started). Two failure modes the requested
        // `timeout_ms` doesn't capture:
        //  1. `drive_io` returns early — e.g. zenoh-pico's condvar wakes
        //     on data arrival, well under 1 ms.
        //  2. The caller spends time outside `spin_once` (explicit sleep,
        //     ROS-2 cooperative scheduling, etc.) and that time should
        //     still count toward timers.
        // Crediting the requested timeout to timers in either case ticks
        // them faster than wall-clock — observed as a 30 Hz control loop
        // overshooting to >200 Hz under sustained traffic. Carry the
        // sub-ms remainder across calls so precision is preserved.
        // phase-359 W4 — the spin-ENTRY read, one spelling. Kept distinct from
        // the post-IO read below because the first spin's delta is measured from
        // entry; collapsing them would make that delta 0 on the first call only.
        let spin_start_us = self.now_us();

        // RFC-0052 W3b.4 — contract monitors tick once per spin (window
        // logic inside; single branch when the baked table is empty).
        self.run_contract_monitors();

        // Phase 104.C.6 — shared executor wake. Swap-and-clear the
        // wake flag; if it was set before this `spin_once` entered,
        // skip the blocking wait on the primary session and poll
        // every session non-blockingly. Lets a wake signal from any
        // thread (or, post-104.C.6.b, any backend's vtable hook)
        // pre-empt whichever session the executor would otherwise
        // sleep on. Cost on the no-wake path is one atomic swap.
        // phase-359 W10 — ONE swap, at `alloc` scope.
        //
        // This was `#[cfg(feature = "std")]` while the alloc wait arm did its
        // own swap further down. Merging the two arms made that two swaps of
        // one flag per spin, the first consuming what the second tested, so the
        // arm would have read `was_woken == false` forever. Deleting the outer
        // one instead broke a different thing: `wake_flag` is `alloc`-gated and
        // "spin_once consumes the wake flag" is an invariant of every alloc
        // build, but the merged arm only runs with `rmw-cffi` — so a
        // `std`-without-`rmw-cffi` build stopped clearing it, which
        // `test_wake_cleared_each_spin` catches. It belongs here, at the scope
        // of the field it reads.
        #[cfg(feature = "alloc")]
        #[allow(unused_variables)]
        let was_woken = self
            .wake_flag
            .swap(false, portable_atomic::Ordering::SeqCst);

        // Phase 124.B.4 — condvar-blocked wait.
        //
        // RT contract:
        //  * cv.wait_timeout_while: bounded by `timeout_ms`.
        //    Predicate is O(1) — one atomic swap + Instant::now.
        //    No allocation. PI-mutex consideration: wake_mu held
        //    only during predicate check (microseconds);
        //    contended worst-case = notify_all execution time
        //    (~10s of µs).
        //  * Backend's `set_wake_callback`-installed cb is called
        //    on async data arrival from its transport-notify path
        //    (worker thread, ISR-safe variant via 124.B.7). The
        //    runtime cb writes wake_flag + signals wake_cv,
        //    unblocking this loop sub-poll-period.
        //  * Poll-only backends (XRCE, bare-metal) leave the slot
        //    NULL; the cv wait still fires on its deadline, then
        //    drive_io(0) drains whatever the backend's internal
        //    poll has buffered. Equivalent to their pre-124
        //    behaviour minus the blocking wait inside drive_io.
        //
        // Lost-wakeup safe: SeqCst flag write happens-before
        // notify, and the waiter checks the flag under wake_mu in
        // the predicate. If wake fires between drain and cv.wait
        // entry, the predicate sees flag=true on first eval and
        // exits immediately.
        // Phase 130.4 — only sleep in the wake wait when a backend actually
        // installed `set_wake_callback`. Poll-only backends (XRCE, current
        // Cyclone) leave the vtable slot NULL → `has_async_wake == false` →
        // drive_io for the caller's full timeout instead of sleeping in a
        // never-signaled wait that starves reliable retransmission (Phase
        // 127.C.4 root cause: the server's `send_response` flushes 100 ms once,
        // then a blind `wait_ms(100)` sleeps with zero session activity, so the
        // agent's ACK arrives into a stalled session and reliable redelivery
        // never fires).
        //
        // Phase 248 (C2) — which primitive is a RUNTIME choice from the
        // platform vtable's wake probe, not a compile-time per-RTOS `cfg`:
        // `node_wake.is_some()` means `nros_platform_wake_*` is linked and
        // `nros_rmw_runtime_wake_cb` signals it on transport arrival.
        // phase-359 W10 — ONE arm. This was a `std` arm and an `alloc` arm with
        // the same body: block on the platform wake primitive when a backend
        // installed the callback and nothing has woken us yet, else drive the
        // transport for the caller's full timeout. They stopped differing when
        // the condvar fallback was deleted — the std arm's `else` branch had
        // already become "drive for the full timeout", which is what the alloc
        // arm always did — so keeping two was keeping a fork that agreed.
        #[cfg(all(feature = "alloc", feature = "rmw-cffi"))]
        let primary_drive_timeout_ms = {
            if !was_woken
                && self.has_async_wake
                && let Some(wake) = self.node_wake.as_ref()
            {
                let _ = wake.wait_ms(timeout_ms as u32);
                // Drain any flag the cb set while we were waiting.
                let _ = self
                    .wake_flag
                    .swap(false, portable_atomic::Ordering::SeqCst);
                0
            } else {
                // No wake primitive linked, or a poll-only backend: drive the
                // transport for the full timeout. It still BLOCKS — in the
                // transport's own recv — which is where every flavour ended up
                // anyway.
                timeout_ms
            }
        };

        // std builds without rmw-cffi (mock-session tests, future alternative
        // backends) keep the original "drive_io is non-blocking" assumption.
        // `std` implies `alloc`, so this is the only std configuration the arm
        // above does not cover.
        #[cfg(all(feature = "std", not(feature = "rmw-cffi")))]
        let primary_drive_timeout_ms = 0;

        // no_std without (alloc + rmw-cffi) keeps the legacy
        // full-timeout drive_io call.
        #[cfg(all(
            not(feature = "std"),
            not(all(feature = "alloc", feature = "rmw-cffi"))
        ))]
        let primary_drive_timeout_ms = timeout_ms;

        // issue 0324 — the primary session's I/O result is TRACKED, not
        // discarded. See `consecutive_io_failures` for why this is a counter
        // rather than an early return.
        match self.session.drive_io(primary_drive_timeout_ms) {
            Ok(()) => self.consecutive_io_failures = 0,
            Err(_) => self.consecutive_io_failures = self.consecutive_io_failures.saturating_add(1),
        }
        for extra in self.extra_sessions.iter_mut() {
            // Best-effort BY DESIGN: extra sessions are bridge / multi-domain
            // attachments, and one of them failing must not stall the primary
            // spin. The old code expressed this the same way it expressed the
            // primary's discarded error — identically — so the intent was
            // invisible. It is now the only `let _ =` of the two.
            let _ = extra.drive_io(0);
        }

        // phase-359 W4 — ONE clock read per spin, shared by the delta below and
        // every consumer further down. Replaces the std/no_std delta pair AND
        // two ad-hoc `static EPOCH: OnceLock<Instant>` blocks that each kept
        // their own std-only epoch.
        let now_us_this_spin = self.now_us();

        // Same rule on both flavours: measure elapsed when a clock exists, else
        // credit the REQUESTED timeout. That fallback was previously reachable
        // only on no_std; on std `Instant` always answered. It stays unreachable
        // on std for the same reason — `now_us()` is infallible there — so this
        // is one expression, not a behaviour change.
        let delta_us = match now_us_this_spin {
            Some(now) => {
                let prev = self
                    .last_spin_end_us
                    .unwrap_or_else(|| spin_start_us.unwrap_or(now));
                self.last_spin_end_us = Some(now);
                now.saturating_sub(prev)
            }
            None => (timeout_ms as u64).saturating_mul(1000),
        };

        if !self.spin_quantization_checked && timeout_ms > 0 {
            self.spin_quantization_checked = true;
            self.audit_spin_quantization((timeout_ms as u64).saturating_mul(1000));
        }

        let arena_ptr = self.arena.as_mut_ptr() as *mut u8;

        // Phase 1: Readiness scan (Phase 110.A.b — backed by FifoReadySet).
        //
        // `bits` carries data-readiness only (used by trigger eval +
        // by `InvocationMode::OnNewData`). `always_mask` carries the
        // `InvocationMode::Always` entries that fire regardless of
        // data presence. The dispatcher drains
        // `FifoReadySet(bits | always_mask)` after the trigger
        // passes; `pop_next` yields registration order (lowest bit
        // first) so behavior is bit-identical to the pre-refactor
        // `for (i, meta) in entries.iter().enumerate()` loop.
        let mut bits: u64 = 0;
        let mut count: usize = 0;
        let mut non_timer_mask: u64 = 0;
        let mut always_mask: u64 = 0;

        for (i, meta) in self.entries.iter().enumerate() {
            if let Some(meta) = meta {
                let data_ptr = unsafe { arena_ptr.add(meta.offset) as *const u8 };
                if unsafe { (meta.has_data)(data_ptr) } {
                    bits |= 1u64 << i;
                }
                if !matches!(meta.kind, EntryKind::Timer | EntryKind::GuardCondition) {
                    non_timer_mask |= 1u64 << i;
                }
                if matches!(meta.invocation, InvocationMode::Always) {
                    always_mask |= 1u64 << i;
                }
                count += 1;
            }
        }

        let snapshot = ReadinessSnapshot { bits, count };

        // Phase 2: Trigger evaluation
        let trigger_passes = match &self.trigger {
            Trigger::Any => bits & non_timer_mask != 0 || non_timer_mask == 0,
            Trigger::All => bits & non_timer_mask == non_timer_mask,
            Trigger::One(id) => snapshot.is_ready(*id),
            Trigger::AllOf(set) => snapshot.all_ready(*set),
            Trigger::AnyOf(set) => snapshot.any_ready(*set),
            Trigger::Always => true,
            Trigger::Predicate(f) => f(&snapshot),
            Trigger::RawPredicate { callback, context } => {
                // Convert ReadinessSnapshot bitmask to a bool array for the C callback
                let mut ready_array = [false; 64];
                for (i, slot) in ready_array
                    .iter_mut()
                    .enumerate()
                    .take(snapshot.count.min(64))
                {
                    *slot = snapshot.bits & (1u64 << i) != 0;
                }
                // SAFETY: The callback and context are provided by the C API caller.
                // The ready_array is valid for snapshot.count elements.
                unsafe { callback(ready_array.as_ptr(), snapshot.count, *context) }
            }
        };

        if !trigger_passes {
            // Timers still need delta accumulation even when trigger doesn't pass
            //
            // phase-8 — `.enumerate()` (rather than a bare `.flatten()`) because
            // `try_process` now carries the entry's slot index for the callback
            // trace hooks. This sweep FIRES timer callbacks, so it is a real
            // dispatch path and must attribute them like the drain below does.
            for (i, meta) in self
                .entries
                .iter()
                .enumerate()
                .filter_map(|(i, e)| e.as_ref().map(|m| (i, m)))
            {
                if matches!(meta.kind, EntryKind::Timer) {
                    let data_ptr = unsafe { arena_ptr.add(meta.offset) };
                    let _ = unsafe { (meta.try_process)(data_ptr, delta_us, i as u8) };
                }
            }

            // Parameter services live outside the arena and must be processed
            // regardless of trigger state, otherwise ROS 2 param queries time out.
            #[cfg(feature = "param-services")]
            {
                let mut handled = 0usize;
                if let Some(params) = &mut self.params {
                    {
                        let crate::parameter_services::ParamState {
                            server, services, ..
                        } = &mut **params;
                        if let Some(services) = services {
                            handled = services.process_services(server).unwrap_or(0);
                        }
                    }
                }
                // phase-425 W3b — a `ros2 param set … use_sim_time true` arrives
                // through this path.
                self.note_param_services_ran(handled);
            }

            // Same treatment for lifecycle services — `ros2 lifecycle get`
            // must succeed even when no callbacks fired this tick.
            // SAFETY: see the matching invariant on the later call site.
            #[cfg(feature = "lifecycle-services")]
            if let Some(lc) = &mut self.lifecycle {
                let crate::lifecycle_services::LifecycleRuntimeState {
                    state_machine,
                    services,
                } = &mut **lc;
                let _ = unsafe { services.process_services(state_machine) };
            }

            return SpinOnceResult::new();
        }

        // Phase 2.5: LET pre-sample (only when LogicalExecutionTime)
        //
        // Sample all subscription data into entry buffers BEFORE dispatching
        // any callbacks. This ensures all callbacks in this cycle see a
        // consistent snapshot of data from the same point in time.
        // Services are NOT pre-sampled (request-reply is sequential).
        if matches!(self.semantics, ExecutorSemantics::LogicalExecutionTime) {
            for meta in self.entries.iter().flatten() {
                if matches!(meta.kind, EntryKind::Subscription) {
                    let data_ptr = unsafe { arena_ptr.add(meta.offset) };
                    unsafe { (meta.pre_sample)(data_ptr) };
                }
            }
        }

        // Phase 3: Dispatch (Phase 110.C — bucketed by SC.priority).
        //
        // Two ready-set families, each split across `Priority::COUNT`
        // buckets (Critical / Normal / BestEffort). Per-entry SC
        // `class` selects FIFO bitmap vs EDF heap; SC `priority`
        // selects the bucket within. Drain order:
        //   for each bucket in priority order (Critical first):
        //     drain EDF heap (deadline-priority), then FIFO bitmap
        //     (registration-order)
        // Default workloads — every entry on the auto-default Fifo SC
        // (Normal priority) — populate only `fifo[Normal]`, so
        // dispatch order is bit-identical to 110.B.b for those.
        const NB: usize = super::sched_context::Priority::COUNT;
        let mut result = SpinOnceResult::new();
        let mut fifo: super::ready_set::BucketedFifoSet<NB, { MAX_CALLBACK_SLOTS }> =
            super::ready_set::BucketedFifoSet::new();
        let mut edf: super::ready_set::BucketedEdfSet<NB, { MAX_CALLBACK_SLOTS }> =
            super::ready_set::BucketedEdfSet::new();
        let active_mask = bits | always_mask;

        // Phase 110.E — refill any Sporadic SC budgets at period
        // boundaries before deciding what to dispatch this cycle.
        // Refill is polled (not ISR-driven) — coarse but correct
        // upper-bound bandwidth limiter.
        // phase-359 W4 — was `#[cfg(feature = "std")]` with NO no_std arm, so
        // polled Sporadic budgets never refilled on embedded: they exhausted
        // once and stayed exhausted. That was a consequence of "no clock on
        // no_std", which is false whenever a `clock_us` hook is injected. Now it
        // runs on either flavour when a clock exists, and is skipped when none
        // does — identical to today's behaviour in that case.
        if let Some(now_us) = now_us_this_spin {
            let now_ms = now_us / 1000;
            // issue 0736 — REFILL only. This used to also charge the cycle's
            // `delta_us` as the per-SC consumption estimate ("worst-case
            // attribution", pending a higher-precision clock hook). That hook
            // landed, and the attribution was not merely coarse: `delta_us` is
            // the wall-clock gap between spins, not CPU the callbacks spent, so
            // wherever the gap exceeds the budget the SC is exhausted on every
            // spin regardless of what it ran. Consumption is charged from
            // measured callback runtime in `consume_dispatch_runtime_us`.
            for slot in self.sporadic_states.iter_mut().flatten() {
                let _ = slot.refill(now_ms);
            }
        }

        for i in 0..self.entries.len() {
            if active_mask & (1u64 << i) == 0 {
                continue;
            }
            let sc_idx = self.sched_context_bindings[i].0 as usize;
            let sc_class_priority_deadline = self
                .sched_contexts
                .get(sc_idx)
                .and_then(|s| s.as_ref())
                .map(|sc| {
                    (
                        sc.class,
                        sc.priority.index(),
                        sc.deadline_us.get().map(|nz| nz.get()).unwrap_or(u32::MAX),
                    )
                });
            let (sc_class, bucket, deadline_us) = sc_class_priority_deadline.unwrap_or((
                super::sched_context::SchedClass::Fifo,
                super::sched_context::Priority::Normal.index(),
                u32::MAX,
            ));
            // Phase 110.E — Sporadic SC dispatch is suppressed when
            // its budget is exhausted. Atomic path (110.E.b PlatformTimer
            // refill) takes precedence when registered; polled path
            // (cycle-level delta_us attribution) handles the unregistered
            // case. Either way, exhausted budget skips dispatch.
            if matches!(sc_class, super::sched_context::SchedClass::Sporadic) {
                #[cfg(feature = "alloc")]
                let atomic_has_budget = self
                    .sporadic_atomic_states
                    .get(sc_idx)
                    .and_then(|s| s.as_ref())
                    .map(|(state, _)| state.has_budget());
                #[cfg(not(feature = "alloc"))]
                let atomic_has_budget: Option<bool> = None;
                let has_budget = match atomic_has_budget {
                    Some(b) => b,
                    None => self
                        .sporadic_states
                        .get(sc_idx)
                        .and_then(|s| s.as_ref())
                        .map(|s| s.budget_remaining_us > 0)
                        .unwrap_or(true),
                };
                if !has_budget {
                    // issue 0736 — keep the TIMER'S CLOCK honest even while the
                    // budget gates its dispatch.
                    //
                    // A timer's `elapsed_us` only advances inside
                    // `timer_try_process`, which is reached only when the entry
                    // is DISPATCHED, and each dispatch is handed just THIS
                    // cycle's `delta_us`. So every cycle skipped here was
                    // dropped from the timer's sense of time: it did not fire,
                    // it did not learn that its period had passed, and — because
                    // the overrun counter is driven by that same `elapsed_us` —
                    // it did not count the activation it missed.
                    //
                    // That is why the throttle was invisible. Measured on
                    // nuttx-arm/rust: ~238 activations lost against ~40
                    // reported, so five in six missed firings were unaccounted
                    // ANYWHERE. The tier ran at a quarter of its declared rate
                    // and the only honest signal was the delivery count itself.
                    //
                    // Advancing the clock here does not dispatch anything and
                    // does not defeat the budget. It makes the entry's own
                    // accounting independent of whether the executor CHOSE to
                    // run it, which is the property every "did this meet its
                    // declaration?" question needs, and it lets the existing
                    // `Skip` policy count the backlog and the existing
                    // `timer-overrun-runtime` rule report it.
                    if let Some(meta) = self.entries[i].as_ref()
                        && matches!(meta.kind, EntryKind::Timer)
                    {
                        // SAFETY: a Timer entry's arena slot holds a
                        // `TimerEntry<F>`, whose leading layout IS
                        // `TimerHeader` — the same cast the overrun reporter
                        // one screen up already relies on.
                        let header =
                            unsafe { &mut *(arena_ptr.add(meta.offset) as *mut TimerHeader) };
                        header.elapsed_us = header.elapsed_us.saturating_add(delta_us);
                    }
                    // issue 0736 — a budget skip used to be a bare `continue`,
                    // which is the silent-drop shape 0737 gated one layer out:
                    // the entry is simply not dispatched and nothing anywhere
                    // says so, so a permanently starved tier is
                    // indistinguishable from an idle one. It took a hand-run
                    // 45 s image and a per-executor probe to learn that this
                    // line was skipping 96 % of one tier's spins.
                    // The rolling window catches a budget that THROTTLES; the
                    // streak below catches one that starves outright. issue
                    // 0736 needed both: the measured case skipped ~3 spins in
                    // 4 without ever reaching a streak, so the tier ran at a
                    // quarter of its declared rate in silence.
                    if let Some((skips, total)) = self
                        .sporadic_states
                        .get_mut(sc_idx)
                        .and_then(|st| st.as_mut())
                        .and_then(|st| st.take_budget_window())
                    {
                        nros_log::nros_warn!(
                            nros_log::get_logger("nros"),
                            "sporadic budget throttled sched context {}: {} of the last {} dispatch opportunities were skipped for want of budget. The declared budget_us/period_us cannot sustain this tier's callbacks on this target.",
                            sc_idx,
                            skips,
                            total
                        );
                    }
                    if let Some(streak) = self
                        .sporadic_states
                        .get_mut(sc_idx)
                        .and_then(|st| st.as_mut())
                        .and_then(|st| st.note_budget_skip())
                    {
                        nros_log::nros_warn!(
                            nros_log::get_logger("nros"),
                            "sporadic budget exhausted for {} consecutive spins (sched context {}): its callbacks are not being dispatched. Either the callback runtime exceeds the declared budget_us per period_us, or the declaration is unsatisfiable on this target.",
                            streak,
                            sc_idx
                        );
                    }
                    continue;
                }
                // Phase 110.E.b follow-up — per-callback runtime
                // accounting (replaces this cycle-level attribution)
                // is applied at dispatch time below via
                // `consume_dispatch_runtime_us`. We only update the
                // polled-path `SporadicState` (no_std fallback) here
                // because the atomic path now records actual
                // wall-clock per-callback runtime. The
                // `delta_us` over-attribution that previously hit the
                // atomic state was a worst-case bandwidth limiter;
                // per-callback measurement is strictly tighter.
                #[cfg(not(feature = "alloc"))]
                {
                    let _ = sc_idx; // polled-state path lives in
                    // `sporadic_states`; this branch is a no-op when
                    // the atomic path is enabled.
                }
            }
            // Phase 110.F — per-callback OS priority routing. Entries
            // bound to an SC with `os_pri > 0` dispatch onto a worker
            // thread the OS has elevated to that priority; the
            // cooperative path is skipped for those entries. Workers
            // are spawned lazily.
            #[cfg(all(
                feature = "alloc",
                feature = "rmw-cffi",
                feature = "scheduler-os-priority"
            ))]
            {
                let os_pri = self
                    .sched_contexts
                    .get(sc_idx)
                    .and_then(|s| s.as_ref())
                    .map(|sc| sc.os_pri)
                    .unwrap_or(0);
                if os_pri > 0
                    && let Some(apply_policy) = self.os_priority_apply_policy
                    && let Some(meta) = self.entries[i].as_ref()
                {
                    let item = super::os_priority::WorkItem {
                        arena_base: arena_ptr as usize,
                        arena_offset: meta.offset,
                        try_process: meta.try_process,
                        delta_us,
                        // phase-8 — the worker runs the leaf on a DIFFERENT
                        // thread, so it has to carry the slot index with it;
                        // nothing on the far side can recover it from the
                        // arena address alone.
                        desc_idx: i as u8,
                    };
                    // phase-359 W10 — `try_dispatch` now reports whether the
                    // entry was actually handed to a worker. It can decline:
                    // the mailbox is bounded, the pool is capacity-limited, and
                    // a platform without a wake primitive hosts no workers at
                    // all. Previously the spawn was infallible (`.expect`) and
                    // an unbounded `mpsc` never refused, so this branch always
                    // `continue`d. Falling through to the cooperative path is
                    // the honest answer to a decline — the callback still runs,
                    // just without the OS priority guarantee, which is exactly
                    // what an entry got before this feature existed.
                    if self
                        .os_priority_pool
                        .try_dispatch(os_pri, apply_policy, item)
                    {
                        continue;
                    }
                }
            }
            // Phase 110.G — TT window gate, orthogonal to class.
            // Skips dispatch when the SC has a TT window AND the
            // current monotonic time is outside it. Both gates apply
            // independently — a Sporadic SC with a TT window must
            // pass both.
            if self.major_frame_us > 0 {
                let sc_opt = self.sched_contexts.get(sc_idx).and_then(|s| s.as_ref());
                if let Some(sc) = sc_opt {
                    let off = sc.tt_window_offset_us.get().map(|nz| nz.get()).unwrap_or(0);
                    let dur = sc
                        .tt_window_duration_us
                        .get()
                        .map(|nz| nz.get())
                        .unwrap_or(0);
                    if dur > 0 {
                        // Compute current phase within the major
                        // frame using the accumulated `delta_us` clock
                        // (std-only precise; no_std uses `delta_us`
                        // approximation from spin cadence).
                        // phase-359 W4 — the shared read. The no_std arm used
                        // to be `now_us = delta_us`, i.e. a per-spin INTERVAL
                        // used as an absolute phase clock, which the comment
                        // above called an approximation. It is now the real
                        // clock when one is injected, and falls back to the old
                        // approximation only when none is.
                        let now_us = now_us_this_spin.unwrap_or(delta_us);
                        let phase = (now_us % self.major_frame_us as u64) as u32;
                        let in_window = if off + dur <= self.major_frame_us {
                            phase >= off && phase < off + dur
                        } else {
                            // Window wraps the major frame boundary.
                            let end = (off as u64 + dur as u64) % self.major_frame_us as u64;
                            phase >= off || (phase as u64) < end
                        };
                        if !in_window {
                            continue;
                        }
                    }
                }
            }
            let is_edf = matches!(sc_class, super::sched_context::SchedClass::Edf);
            let job = super::types::ActiveJob {
                sort_key: if is_edf { deadline_us } else { i as u32 },
                desc_idx: i as super::types::DescIdx,
            };
            if is_edf {
                let _ = edf.insert_into(bucket, job);
            } else {
                let _ = fifo.insert_into(bucket, job);
            }
        }

        // SAFETY: each `desc_idx` we pop was set above only when the
        // corresponding `entries[i]` slot was `Some`; no Executor
        // mutation happens between that scan and this dispatch.
        let dispatch_one = |meta: &CallbackMeta,
                            desc_idx: usize,
                            arena_ptr: *mut u8,
                            delta_us: u64,
                            result: &mut SpinOnceResult| {
            // Phase 141.B.2 — capture T1 at subscription dispatch
            // entry. Probe pairs it with the most recent T0 from
            // `nros_rmw_runtime_wake_cb` (std + alloc variants)
            // and pushes `T1 - T0` onto the ring buffer 141.C
            // drains. No-op when the probe feature is off or
            // no cycle reader is installed. Other entry kinds
            // (Service / Timer / GuardCondition) skip the probe
            // because the 141 acceptance is specifically
            // wake-to-subscription-dispatch latency.
            #[cfg(feature = "wake-latency-probe")]
            if matches!(meta.kind, EntryKind::Subscription) {
                super::wake_probe::on_dispatch();
            }
            let data_ptr = unsafe { arena_ptr.add(meta.offset) };
            // phase-8 — `desc_idx` is the entry slot index, threaded through so
            // the leaf hooks in `arena.rs` can name the callback they bracket.
            // `MAX_CALLBACK_SLOTS` is 64 (enforced by the `u64` ready-set
            // bitmask), so the `as u8` cannot truncate a live slot.
            match unsafe { (meta.try_process)(data_ptr, delta_us, desc_idx as u8) } {
                Ok(true) => match meta.kind {
                    EntryKind::Subscription => result.subscriptions_processed += 1,
                    EntryKind::Service
                    | EntryKind::ServiceClient
                    | EntryKind::ActionServer
                    | EntryKind::ActionClient => result.services_handled += 1,
                    EntryKind::Timer => result.timers_fired += 1,
                    EntryKind::GuardCondition => {}
                },
                Ok(false) => {}
                Err(_) => match meta.kind {
                    EntryKind::Subscription => result.subscription_errors += 1,
                    EntryKind::Service
                    | EntryKind::ServiceClient
                    | EntryKind::ActionServer
                    | EntryKind::ActionClient => result.service_errors += 1,
                    EntryKind::Timer | EntryKind::GuardCondition => {}
                },
            }
        };

        // Phase 110.E.b follow-up — per-callback runtime accounting
        // for Sporadic SCs. Wall-clock-measure each dispatch and
        // consume the elapsed microseconds from the bound SC's
        // atomic budget. This replaces the cycle-level over-
        // attribution that previously charged the FULL `delta_us`
        // against every Sporadic SC regardless of which entries
        // actually fired — accurate per-callback measurement is the
        // shape the design doc's per-callback runtime acceptance
        // calls out.
        //
        // phase-359 W10 — this was `feature = "std"`-gated, because it "needs a
        // `core::time::Instant`-equivalent monotonic clock ... until a
        // board-side monotonic-microsecond accessor lands". That accessor
        // landed: `now_us()` reads the platform monotonic counter on every
        // flavour. So per-callback sporadic accounting is no longer something
        // only a hosted build gets, and the no_std fallback to polled
        // `SporadicState` cycle deltas is no longer the only option on target.
        let consume_dispatch_runtime_us =
            |desc_idx: usize,
             elapsed_us: u32,
             sched_context_bindings: &[super::sched_context::SchedContextId],
             sched_contexts: &[Option<super::sched_context::SchedContext>],
             sporadic_states: &mut [Option<super::sched_context::SporadicState>],
             #[cfg(feature = "alloc")] sporadic_atomic_states: &[Option<(
                portable_atomic_util::Arc<super::sched_context::AtomicSporadicState>,
                OpaqueTimerHandle,
            )>]| {
                let sc_idx = sched_context_bindings[desc_idx].0 as usize;
                let sc_class = sched_contexts
                    .get(sc_idx)
                    .and_then(|s| s.as_ref())
                    .map(|sc| sc.class)
                    .unwrap_or(super::sched_context::SchedClass::Fifo);
                if !matches!(sc_class, super::sched_context::SchedClass::Sporadic) {
                    return;
                }
                #[cfg(feature = "alloc")]
                if let Some((state, _)) =
                    sporadic_atomic_states.get(sc_idx).and_then(|s| s.as_ref())
                {
                    state.consume(elapsed_us);
                    // Unconditional, unlike the overrun record below: the
                    // budget is sized FROM this number, so it has to exist
                    // before the budget is right rather than only after it
                    // is wrong.
                    state.record_exec(elapsed_us);
                    // Phase 110.E.b — overrun detection. Cooperative
                    // single-thread can't preempt a runaway callback,
                    // so post-dispatch wall-clock comparison delivers
                    // the same observable signal as the design's
                    // oneshot-IRQ-and-cancel pattern, without needing
                    // a separate timer per SC. `budget_capacity_us` is
                    // the per-period budget the SC was sized against;
                    // any callback exceeding that has run past its
                    // bandwidth allotment.
                    if elapsed_us > state.budget_capacity_us {
                        state.record_overrun(elapsed_us - state.budget_capacity_us);
                    }
                }
                // issue 0736 — the POLLED state is charged unconditionally,
                // and it is the one that matters: `has_budget` prefers the
                // atomic state only when a refill timer was registered, which
                // no board or entry does. Before this, the alloc arm charged a
                // state nothing populated and the no_std arm discarded the
                // measurement outright (`let _ = (sc_idx, elapsed_us)`), so no
                // shipped image recorded per-callback runtime anywhere — while
                // the comment above claimed it did.
                if let Some(state) = sporadic_states.get_mut(sc_idx).and_then(|s| s.as_mut()) {
                    state.consume(elapsed_us);
                }
            };

        // W3b.5 — post-dispatch contract checks. `lat_active` gates the
        // per-dispatch publish-count snapshot (attribution of dispatch
        // elapsed time to monitored publishers whose counter advanced);
        // `dl_active` gates elapsed measurement for deadline actions. The third
        // term used to be `cfg!(feature = "std")` — "std measures anyway for
        // the sporadic path" — which made a COMPILE-TIME flavour stand in for
        // "is there a clock". It is now the runtime question it always was.
        let mon_table = self.monitor_table;
        let lat_active = mon_table.iter().any(|m| m.max_latency_ms > 0);
        let dl_active = self.sched_contexts.iter().flatten().any(|sc| {
            sc.deadline_us.is_some()
                && !matches!(
                    sc.deadline_action,
                    super::sched_context::DeadlineAction::Ignore
                )
        });
        let mon_clock = self.clock_us_fn;
        // phase-359 W4 — one predicate, no cfg block. `cfg!` is an expression,
        // so the flavour difference stays a value instead of a branch: std
        // measures unconditionally because the sporadic runtime accounting
        // below (itself std-only) consumes the result, which is exactly what
        // the comment above described and what the four-arm sites encoded.
        let measure_us = lat_active || dl_active || mon_clock.is_some();
        // phase-359 W4 — ONE hoisted µs reader for the per-dispatch latency
        // measurement below. Hoisted because `now_us()` takes `&mut self` and
        // the dispatch loop already holds borrows; the std arm copies the epoch
        // (a `Copy` `Instant`) so reading inside the loop touches no `self`.
        // This is the last cfg pair in the timing path: the two call sites it
        // serves each had FOUR arms (start + elapsed, per flavour).
        let read_us = move || mon_clock.map(|c| c()).unwrap_or(0);
        // Deferred deadline-miss violations (the loop body holds an
        // immutable borrow of `self.entries`, so the ring is fed after).
        let mut deadline_misses: heapless::Vec<
            super::monitor::Violation,
            { super::monitor::MAX_VIOLATIONS },
        > = heapless::Vec::new();
        // SCs whose remaining callbacks this cycle are skipped
        // (`DeadlineAction::Skip`) — bitmask over SC slots.
        let mut skipped_scs: u64 = 0;

        // For each priority bucket (Critical → Normal → BestEffort),
        // drain EDF first then FIFO so an EDF callback in this bucket
        // beats a FIFO peer at the same priority, but no lower-priority
        // entry runs while a higher-priority bucket has work pending.
        // Strict static priority across buckets; non-preemptive within
        // an in-flight callback (see Phase 110.D).
        for bucket in 0..NB {
            while let Some(job) = edf.pop_from(bucket) {
                let i = job.desc_idx as usize;
                let sc_idx = self.sched_context_bindings[i].0 as usize;
                if sc_idx < 64 && skipped_scs & (1u64 << sc_idx) != 0 {
                    continue; // W3b.5 DeadlineAction::Skip containment
                }
                if let Some(meta) = self.entries[i].as_ref() {
                    let counts_before = snapshot_pub_counts(mon_table, lat_active);
                    // Measured only when a consumer wants it, matching the
                    // no_std arm this replaces — the std arm used to measure
                    // unconditionally.
                    let start_us = measure_us.then(&read_us);
                    dispatch_one(meta, i, arena_ptr, delta_us, &mut result);
                    let elapsed_us: Option<u32> = start_us
                        .map(|t0| read_us().saturating_sub(t0))
                        .map(|d| d.min(u32::MAX as u64) as u32);
                    if let Some(elapsed_us) = elapsed_us {
                        consume_dispatch_runtime_us(
                            i,
                            elapsed_us,
                            &self.sched_context_bindings[..],
                            &self.sched_contexts[..],
                            &mut self.sporadic_states[..],
                            #[cfg(feature = "alloc")]
                            &self.sporadic_atomic_states[..],
                        );
                    }
                    if let Some(elapsed_us) = elapsed_us {
                        attribute_latency(mon_table, lat_active, &counts_before, elapsed_us);
                        check_deadline_miss(
                            self.sched_contexts.get(sc_idx).and_then(|s| s.as_ref()),
                            sc_idx,
                            elapsed_us,
                            &mut deadline_misses,
                            &mut skipped_scs,
                            self.fault_fn,
                        );
                    }
                }
            }
            while let Some(job) = fifo.pop_from(bucket) {
                let i = job.desc_idx as usize;
                let sc_idx = self.sched_context_bindings[i].0 as usize;
                if sc_idx < 64 && skipped_scs & (1u64 << sc_idx) != 0 {
                    continue; // W3b.5 DeadlineAction::Skip containment
                }
                if let Some(meta) = self.entries[i].as_ref() {
                    let counts_before = snapshot_pub_counts(mon_table, lat_active);
                    // Measured only when a consumer wants it, matching the
                    // no_std arm this replaces — the std arm used to measure
                    // unconditionally.
                    let start_us = measure_us.then(&read_us);
                    dispatch_one(meta, i, arena_ptr, delta_us, &mut result);
                    let elapsed_us: Option<u32> = start_us
                        .map(|t0| read_us().saturating_sub(t0))
                        .map(|d| d.min(u32::MAX as u64) as u32);
                    if let Some(elapsed_us) = elapsed_us {
                        consume_dispatch_runtime_us(
                            i,
                            elapsed_us,
                            &self.sched_context_bindings[..],
                            &self.sched_contexts[..],
                            &mut self.sporadic_states[..],
                            #[cfg(feature = "alloc")]
                            &self.sporadic_atomic_states[..],
                        );
                    }
                    if let Some(elapsed_us) = elapsed_us {
                        attribute_latency(mon_table, lat_active, &counts_before, elapsed_us);
                        check_deadline_miss(
                            self.sched_contexts.get(sc_idx).and_then(|s| s.as_ref()),
                            sc_idx,
                            elapsed_us,
                            &mut deadline_misses,
                            &mut skipped_scs,
                            self.fault_fn,
                        );
                    }
                }
            }
        }

        // W3b.5 — feed deferred deadline misses into the violation ring.
        for v in deadline_misses {
            if self.report_violations {
                super::monitor::log_violation(&v);
            }
            if self.monitor_violations.push(v).is_err() {
                self.monitor_violations_dropped = self.monitor_violations_dropped.saturating_add(1);
            }
        }

        // Issue #505 — same ring, same cycle. This runs AFTER dispatch,
        // unlike the windowed rate/age/latency rules at the top of the
        // spin: the activations it reports were dropped by the dispatch
        // that just happened, and a tier that is stalling should not have
        // to wait for the next spin to say so.
        self.check_timer_overruns();
        self.check_release_jitter_rule();
        self.check_stack_headroom_rule();

        // Process parameter services (outside the arena)
        #[cfg(feature = "param-services")]
        let mut handled = 0usize;
        #[cfg(feature = "param-services")]
        if let Some(params) = &mut self.params {
            {
                let crate::parameter_services::ParamState {
                    server, services, ..
                } = &mut **params;
                if let Some(services) = services
                    && let Ok(n) = services.process_services(server)
                {
                    result.services_handled += n;
                    handled = n;
                }
            }
        }
        #[cfg(feature = "param-services")]
        self.note_param_services_ran(handled);

        // Process lifecycle services (outside the arena).
        //
        // SAFETY: `change_state` dispatches a user-supplied C callback through a
        // raw function pointer stored in `LifecyclePollingNodeCtx`. The caller
        // of `register_lifecycle_services` guarantees the callback/context pair
        // stays live for as long as the executor (see that method's docs).
        #[cfg(feature = "lifecycle-services")]
        if let Some(lc) = &mut self.lifecycle {
            let crate::lifecycle_services::LifecycleRuntimeState {
                state_machine,
                services,
            } = &mut **lc;
            if let Ok(n) = unsafe { services.process_services(state_machine) } {
                result.services_handled += n;
            }
        }

        // Phase 258 (Track 2, 2a) — executor-owned component tick pass.
        // Mirrors `ExecutorNodeRuntime::run_ticks`: after the transport +
        // callbacks have been pumped, drive each enrolled component's `tick`
        // (service-client/action poll, etc.). `exec_ctx` hands the component
        // the whole executor as a raw `*mut Executor` so its tick can
        // reborrow it (the same disjoint-field raw-ptr pattern run_ticks
        // uses). Index-iterate over `Copy` slots so no borrow of
        // `self.component_slots` is held while `tick` runs (which aliases
        // `self` through `exec_ctx`).
        let exec_ctx = self as *mut Executor as *mut core::ffi::c_void;
        let slot_count = self.component_slots.len();
        for i in 0..slot_count {
            let slot = self.component_slots[i];
            // SAFETY: `slot.state` is the leaked cell the matching `tick`
            // expects (enrolled via `enroll_component`); `exec_ctx` is a live
            // `*mut Executor` for `self`. The slot was copied out, so no
            // borrow of `component_slots` is outstanding during the call.
            unsafe {
                (slot.tick)(slot.state, exec_ctx);
            }
        }

        result
    }

    /// Drive I/O and dispatch callbacks in an infinite loop.
    ///
    /// Each iteration calls [`spin_once(timeout_ms)`](Self::spin_once),
    /// which pumps the transport and dispatches all registered callbacks.
    ///
    /// This is the primary run loop for embedded applications:
    ///
    /// ```ignore
    /// let mut executor = Executor::open(&config)?;
    /// executor.register_subscription::<Int32, _>("/topic", |msg| { /* ... */ })?;
    /// executor.spin(10); // never returns
    /// ```
    pub fn spin(&mut self, timeout: core::time::Duration) -> ! {
        loop {
            self.spin_once(timeout);
        }
    }

    /// Phase 104.C.3.3.c — rclcpp-`spin()`-shape no-arg variant.
    /// Defaults the per-iteration timeout to 50 ms, which keeps
    /// idle binaries from busy-spinning while staying responsive
    /// enough for default-QoS messaging.
    pub fn spin_default(&mut self) -> ! {
        self.spin(core::time::Duration::from_millis(50))
    }

    /// Drive I/O and dispatch callbacks asynchronously.
    ///
    /// Runs forever, yielding between poll cycles so that other async tasks
    /// (e.g., [`Promise`](super::handles::Promise)) can make progress.
    ///
    /// Uses only `core::future` — no external async runtime dependency.
    ///
    /// # Usage patterns
    ///
    /// ```ignore
    /// // Pattern 1: select with a promise (embassy-futures)
    /// use embassy_futures::select::{select, Either};
    /// let promise = client.call(&req)?;
    /// let Either::Second(reply) = select(executor.spin_async(), promise).await
    ///     else { unreachable!() };
    ///
    /// // Pattern 2: manual polling (no async runtime)
    /// let mut promise = client.call(&req)?;
    /// loop {
    ///     executor.spin_once(core::time::Duration::from_millis(10));
    ///     if let Ok(Some(r)) = promise.take() { break r; }
    /// }
    /// ```
    pub async fn spin_async(&mut self) -> ! {
        loop {
            self.spin_once(core::time::Duration::from_millis(1));
            core::future::poll_fn::<(), _>(|cx| {
                cx.waker().wake_by_ref();
                core::task::Poll::Pending
            })
            .await;
        }
    }

    // ========================================================================
    // spin_one_period (no_std)
    // ========================================================================

    /// Process one iteration and return remaining sleep time.
    ///
    /// This is `no_std` compatible — the caller is responsible for the actual
    /// delay using platform-specific sleep.
    ///
    /// # Arguments
    /// * `period_ms` - Target period in milliseconds
    /// * `elapsed_ms` - Time elapsed since last call (used for timer ticking)
    ///
    /// # Example
    ///
    /// ```ignore
    /// loop {
    ///     let r = executor.spin_one_period(10, elapsed_ms);
    ///     platform_sleep_ms(r.remaining_ms);
    /// }
    /// ```
    pub fn spin_one_period(&mut self, period_ms: u64, elapsed_ms: u64) -> SpinPeriodPollingResult {
        let result = self.spin_once(core::time::Duration::from_millis(elapsed_ms));
        SpinPeriodPollingResult {
            work: result,
            remaining_ms: period_ms.saturating_sub(elapsed_ms),
        }
    }
}

// ============================================================================
// Parameter services (cfg param-services)
// ============================================================================

#[cfg(feature = "param-services")]
impl<'s> Executor<'s> {
    /// Register the 6 ROS 2 parameter services for this node.
    ///
    /// Creates service servers for `get_parameters`, `set_parameters`,
    /// `set_parameters_atomically`, `list_parameters`, `describe_parameters`,
    /// and `get_parameter_types`.
    ///
    /// The service names follow the ROS 2 convention: `/{namespace}/{node_name}/{suffix}`.
    /// For the default namespace `/`, this becomes `/{node_name}/{suffix}` (e.g.
    /// `/sentinel/list_parameters`).
    ///
    /// Parameter services are stored outside the arena and don't consume
    /// callback slots.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let config = ExecutorConfig::from_env().node_name("talker");
    /// let mut executor = Executor::open(&config)?;
    /// executor.register_parameter_services()?;
    /// executor.declare_parameter("start_value", ParameterValue::Integer(0));
    /// ```
    pub fn register_parameter_services(&mut self) -> Result<(), NodeError> {
        use crate::parameter_services::{
            DescribeParameters, GetParameterTypes, GetParameters, ListParameters,
            PARAM_SERVICE_BUFFER_SIZE, ParameterServiceServers, SetParameters,
            SetParametersAtomically,
        };
        use nros_core::RosService;

        type PSrv<Svc> = super::handles::EmbeddedServiceServer<
            Svc,
            PARAM_SERVICE_BUFFER_SIZE,
            PARAM_SERVICE_BUFFER_SIZE,
        >;

        // Build the node FQN from namespace + node_name, following ROS 2 convention.
        // Default namespace "/" → "/{node_name}"; otherwise "/{namespace}/{node_name}".
        let mut node_fqn = heapless::String::<256>::new();
        let ns: &str = &self.namespace;
        let nn: &str = &self.node_name;
        if ns.is_empty() || ns == "/" {
            node_fqn.push_str("/").map_err(|_| NodeError::NameTooLong)?;
            node_fqn.push_str(nn).map_err(|_| NodeError::NameTooLong)?;
        } else {
            node_fqn.push_str("/").map_err(|_| NodeError::NameTooLong)?;
            node_fqn
                .push_str(ns.trim_matches('/'))
                .map_err(|_| NodeError::NameTooLong)?;
            node_fqn.push_str("/").map_err(|_| NodeError::NameTooLong)?;
            node_fqn.push_str(nn).map_err(|_| NodeError::NameTooLong)?;
        }

        /// Build a service name like `{node_fqn}/{suffix}` and create the server handle.
        fn create_param_srv<Svc: RosService>(
            session: &mut session::ConcreteSession,
            domain_id: u32,
            node_fqn: &str,
            namespace: &str,
            node_name: &str,
            suffix: &str,
        ) -> Result<session::RmwServiceServer, NodeError> {
            let mut name = heapless::String::<256>::new();
            name.push_str(node_fqn)
                .map_err(|_| NodeError::NameTooLong)?;
            name.push_str("/").map_err(|_| NodeError::NameTooLong)?;
            name.push_str(suffix).map_err(|_| NodeError::NameTooLong)?;
            let mut info = ServiceInfo::new(&name, Svc::SERVICE_NAME, Svc::SERVICE_HASH)
                // issue 0824 follow-up — `domain_id` is a PARAMETER, not `self.domain_id`:
                // this is a nested `fn`, not a method, so `self` is not in scope and
                // the original spelling was E0434. It only fails under feature sets
                // that compile this arm, which is why `--all-features` caught it and
                // the narrower lanes did not.
                // issue 0824 follow-up — `domain_id` is a PARAMETER, not `self.domain_id`:
                // this is a nested `fn`, not a method, so `self` is not in scope and
                // the original spelling was E0434. It only fails under feature sets
                // that compile this arm, which is why `--all-features` caught it and
                // the narrower lanes did not.
                .with_domain(domain_id)
                .with_namespace(namespace);
            if !node_name.is_empty() {
                info = info.with_node_name(node_name);
            }
            // issue 0793 — the parameter services get the PARAMETER profile
            // (`rmw_qos_profile_parameters`: KEEP_LAST(1000), reliable,
            // volatile), not the generic services one. `QOS_PROFILE_PARAMETERS`
            // existed with the right depth and had no caller, so every parameter
            // server ran on a depth-10 queue while ROS 2 gives them 1000 — which
            // matters exactly when a tool sets many parameters at once, the case
            // the deep queue is for.
            session
                .create_service(&info, QoSProfile::parameters_default())
                .map_err(NodeError::Transport)
        }

        let get_handle = create_param_srv::<GetParameters>(
            &mut self.session,
            self.domain_id,
            &node_fqn,
            ns,
            nn,
            "get_parameters",
        )?;
        let set_handle = create_param_srv::<SetParameters>(
            &mut self.session,
            self.domain_id,
            &node_fqn,
            ns,
            nn,
            "set_parameters",
        )?;
        let set_atomic_handle = create_param_srv::<SetParametersAtomically>(
            &mut self.session,
            self.domain_id,
            &node_fqn,
            ns,
            nn,
            "set_parameters_atomically",
        )?;
        let list_handle = create_param_srv::<ListParameters>(
            &mut self.session,
            self.domain_id,
            &node_fqn,
            ns,
            nn,
            "list_parameters",
        )?;
        let desc_handle = create_param_srv::<DescribeParameters>(
            &mut self.session,
            self.domain_id,
            &node_fqn,
            ns,
            nn,
            "describe_parameters",
        )?;
        let types_handle = create_param_srv::<GetParameterTypes>(
            &mut self.session,
            self.domain_id,
            &node_fqn,
            ns,
            nn,
            "get_parameter_types",
        )?;

        let servers = ParameterServiceServers::new(
            PSrv::<GetParameters> {
                handle: get_handle,
                req_buffer: [0u8; PARAM_SERVICE_BUFFER_SIZE],
                reply_buffer: [0u8; PARAM_SERVICE_BUFFER_SIZE],
                _phantom: core::marker::PhantomData,
            },
            PSrv::<SetParameters> {
                handle: set_handle,
                req_buffer: [0u8; PARAM_SERVICE_BUFFER_SIZE],
                reply_buffer: [0u8; PARAM_SERVICE_BUFFER_SIZE],
                _phantom: core::marker::PhantomData,
            },
            PSrv::<SetParametersAtomically> {
                handle: set_atomic_handle,
                req_buffer: [0u8; PARAM_SERVICE_BUFFER_SIZE],
                reply_buffer: [0u8; PARAM_SERVICE_BUFFER_SIZE],
                _phantom: core::marker::PhantomData,
            },
            PSrv::<ListParameters> {
                handle: list_handle,
                req_buffer: [0u8; PARAM_SERVICE_BUFFER_SIZE],
                reply_buffer: [0u8; PARAM_SERVICE_BUFFER_SIZE],
                _phantom: core::marker::PhantomData,
            },
            PSrv::<DescribeParameters> {
                handle: desc_handle,
                req_buffer: [0u8; PARAM_SERVICE_BUFFER_SIZE],
                reply_buffer: [0u8; PARAM_SERVICE_BUFFER_SIZE],
                _phantom: core::marker::PhantomData,
            },
            PSrv::<GetParameterTypes> {
                handle: types_handle,
                req_buffer: [0u8; PARAM_SERVICE_BUFFER_SIZE],
                reply_buffer: [0u8; PARAM_SERVICE_BUFFER_SIZE],
                _phantom: core::marker::PhantomData,
            },
        );

        // Issue 0745 — PRESERVE an already-initialized store: launch-param
        // seeding may have run before any node existed (services can't be
        // registered that early). Overwriting here would wipe the seeds.
        let services_box: alloc::boxed::Box<dyn crate::parameter_services::ParamServiceProcessor> =
            alloc::boxed::Box::new(servers);
        match &mut self.params {
            Some(params) => params.services = Some(services_box),
            None => {
                // Issue 0756 — see new_param_state: never build a ParamState
                // by value, it does not fit an embedded thread stack.
                self.params = Some(Self::new_param_state(Some(services_box)));
            }
        }

        Ok(())
    }

    // phase-359 W10 / issue 0080 — `enable_parameter_persistence{,_with}` are
    // GONE with the seam they attached. Issue 0080 ruled on-device parameter
    // persistence a non-goal in July and listed both by name; nothing in the
    // tree ever called them, and the only backend that could be passed
    // (`FileParamStore`) is deleted. Runtime get/set/describe stay — it is the
    // PERSISTENCE half that was dropped.
}

// ============================================================================
// Lifecycle services (cfg lifecycle-services)
// ============================================================================

#[cfg(feature = "lifecycle-services")]
impl<'s> Executor<'s> {
    /// Register the five REP-2002 lifecycle services on this executor.
    ///
    /// After this call, `ros2 lifecycle set|get|list|nodes` can drive the
    /// stored [`LifecyclePollingNodeCtx`](crate::lifecycle::LifecyclePollingNodeCtx)
    /// through the node's lifecycle. The state machine is created fresh
    /// (starting in `Unconfigured`); callers register their transition
    /// callbacks via [`Executor::lifecycle_state_machine_mut`].
    ///
    /// # Safety
    /// Registered callbacks on the state machine are C FFI function pointers.
    /// The caller must keep the callback code and any context it captures
    /// valid for as long as the executor processes services.
    pub fn register_lifecycle_services(&mut self) -> Result<(), NodeError> {
        use crate::{
            lifecycle::LifecyclePollingNodeCtx,
            lifecycle_services::{
                ChangeState, GetAvailableStates, GetAvailableTransitions, GetState,
                LIFECYCLE_SERVICE_BUFFER_SIZE, LifecycleRuntimeState, LifecycleServiceServers,
            },
        };
        use nros_core::RosService;

        type LcSrv<Svc> = super::handles::EmbeddedServiceServer<
            Svc,
            LIFECYCLE_SERVICE_BUFFER_SIZE,
            LIFECYCLE_SERVICE_BUFFER_SIZE,
        >;

        // Build the node FQN from namespace + node_name (same convention as
        // register_parameter_services).
        let mut node_fqn = heapless::String::<256>::new();
        let ns: &str = &self.namespace;
        let nn: &str = &self.node_name;
        if ns.is_empty() || ns == "/" {
            node_fqn.push_str("/").map_err(|_| NodeError::NameTooLong)?;
            node_fqn.push_str(nn).map_err(|_| NodeError::NameTooLong)?;
        } else {
            node_fqn.push_str("/").map_err(|_| NodeError::NameTooLong)?;
            node_fqn
                .push_str(ns.trim_matches('/'))
                .map_err(|_| NodeError::NameTooLong)?;
            node_fqn.push_str("/").map_err(|_| NodeError::NameTooLong)?;
            node_fqn.push_str(nn).map_err(|_| NodeError::NameTooLong)?;
        }

        fn create_lc_srv<Svc: RosService>(
            session: &mut session::ConcreteSession,
            domain_id: u32,
            node_fqn: &str,
            namespace: &str,
            node_name: &str,
            suffix: &str,
        ) -> Result<session::RmwServiceServer, NodeError> {
            let mut name = heapless::String::<256>::new();
            name.push_str(node_fqn)
                .map_err(|_| NodeError::NameTooLong)?;
            name.push_str("/").map_err(|_| NodeError::NameTooLong)?;
            name.push_str(suffix).map_err(|_| NodeError::NameTooLong)?;
            let mut info = ServiceInfo::new(&name, Svc::SERVICE_NAME, Svc::SERVICE_HASH)
                .with_domain(domain_id)
                .with_namespace(namespace);
            if !node_name.is_empty() {
                info = info.with_node_name(node_name);
            }
            session
                .create_service(&info, QoSProfile::services_default())
                .map_err(NodeError::Transport)
        }

        let cs_handle = create_lc_srv::<ChangeState>(
            &mut self.session,
            self.domain_id,
            &node_fqn,
            ns,
            nn,
            "change_state",
        )?;
        let gs_handle = create_lc_srv::<GetState>(
            &mut self.session,
            self.domain_id,
            &node_fqn,
            ns,
            nn,
            "get_state",
        )?;
        let gas_handle = create_lc_srv::<GetAvailableStates>(
            &mut self.session,
            self.domain_id,
            &node_fqn,
            ns,
            nn,
            "get_available_states",
        )?;
        let gat_handle = create_lc_srv::<GetAvailableTransitions>(
            &mut self.session,
            self.domain_id,
            &node_fqn,
            ns,
            nn,
            "get_available_transitions",
        )?;
        let gtg_handle = create_lc_srv::<GetAvailableTransitions>(
            &mut self.session,
            self.domain_id,
            &node_fqn,
            ns,
            nn,
            "get_transition_graph",
        )?;

        let servers = LifecycleServiceServers::new(
            LcSrv::<ChangeState> {
                handle: cs_handle,
                req_buffer: [0u8; LIFECYCLE_SERVICE_BUFFER_SIZE],
                reply_buffer: [0u8; LIFECYCLE_SERVICE_BUFFER_SIZE],
                _phantom: core::marker::PhantomData,
            },
            LcSrv::<GetState> {
                handle: gs_handle,
                req_buffer: [0u8; LIFECYCLE_SERVICE_BUFFER_SIZE],
                reply_buffer: [0u8; LIFECYCLE_SERVICE_BUFFER_SIZE],
                _phantom: core::marker::PhantomData,
            },
            LcSrv::<GetAvailableStates> {
                handle: gas_handle,
                req_buffer: [0u8; LIFECYCLE_SERVICE_BUFFER_SIZE],
                reply_buffer: [0u8; LIFECYCLE_SERVICE_BUFFER_SIZE],
                _phantom: core::marker::PhantomData,
            },
            LcSrv::<GetAvailableTransitions> {
                handle: gat_handle,
                req_buffer: [0u8; LIFECYCLE_SERVICE_BUFFER_SIZE],
                reply_buffer: [0u8; LIFECYCLE_SERVICE_BUFFER_SIZE],
                _phantom: core::marker::PhantomData,
            },
            LcSrv::<GetAvailableTransitions> {
                handle: gtg_handle,
                req_buffer: [0u8; LIFECYCLE_SERVICE_BUFFER_SIZE],
                reply_buffer: [0u8; LIFECYCLE_SERVICE_BUFFER_SIZE],
                _phantom: core::marker::PhantomData,
            },
        );

        self.lifecycle = Some(alloc::boxed::Box::new(LifecycleRuntimeState {
            state_machine: LifecyclePollingNodeCtx::new(),
            services: alloc::boxed::Box::new(servers),
        }));

        Ok(())
    }

    /// Mutable access to the lifecycle state machine, if registered.
    ///
    /// Used to register transition callbacks before spinning and to read the
    /// current state from application code.
    pub fn lifecycle_state_machine_mut(
        &mut self,
    ) -> Option<&mut crate::lifecycle::LifecyclePollingNodeCtx> {
        self.lifecycle.as_mut().map(|lc| &mut lc.state_machine)
    }

    /// Immutable access to the lifecycle state machine, if registered.
    pub fn lifecycle_state_machine(&self) -> Option<&crate::lifecycle::LifecyclePollingNodeCtx> {
        self.lifecycle.as_ref().map(|lc| &lc.state_machine)
    }

    /// Register a safe lifecycle node (issue 0335 / phase-317).
    ///
    /// Binds the five REP-2002 lifecycle services and wires each transition to
    /// the [`LifecycleCallbacks`](crate::lifecycle::LifecycleCallbacks) trait
    /// impl — the safe Rust counterpart to the C++ `nros::LifecycleNode`. No
    /// `unsafe` in user code; alloc-free (monomorphized trampolines).
    ///
    /// `node` must outlive this executor's lifecycle registration: the context
    /// pointer stored here is dereferenced on every transition until the
    /// executor is finalized. The executor spins single-threaded, so the
    /// `&mut node` reconstituted inside a callback is never aliased.
    pub fn register_lifecycle_node<T: crate::lifecycle::LifecycleCallbacks>(
        &mut self,
        node: &mut T,
    ) -> Result<(), NodeError> {
        use crate::lifecycle::{LifecycleCallbackSlot as Slot, trampolines};

        self.register_lifecycle_services()?;
        let sm = self
            .lifecycle_state_machine_mut()
            .ok_or(NodeError::NotInitialized)?;
        sm.set_context(node as *mut T as *mut core::ffi::c_void);
        sm.register(Slot::Configure, Some(trampolines::on_configure::<T>));
        sm.register(Slot::Activate, Some(trampolines::on_activate::<T>));
        sm.register(Slot::Deactivate, Some(trampolines::on_deactivate::<T>));
        sm.register(Slot::Cleanup, Some(trampolines::on_cleanup::<T>));
        sm.register(Slot::Shutdown, Some(trampolines::on_shutdown::<T>));
        sm.register(Slot::Error, Some(trampolines::on_error::<T>));
        Ok(())
    }
}

// ============================================================================
// Parameter declaration API (cfg param-services)
// ============================================================================

#[cfg(feature = "param-services")]
impl<'s> Executor<'s> {
    /// Issue 0745 — lazily create the parameter STORE (no services yet).
    /// Launch-param seeding runs before any node is constructed; the six
    /// service servers attach later in `register_parameter_services`,
    /// which preserves this store.
    fn ensure_parameter_store(&mut self) {
        if self.params.is_none() {
            self.params = Some(Self::new_param_state(None));
        }
    }

    /// Produce the parameter slot table the executor's store will borrow.
    ///
    /// phase-382 W2' — `ParameterServer` no longer OWNS its slots; it borrows
    /// a caller-placed [`nros_params::ParameterTable`]. W3' carves that table
    /// out of the caller's executor backing, at which point this function goes
    /// away. Until then the executor has no caller-supplied home to borrow
    /// from, so it makes one: a single heap allocation, leaked, one per
    /// executor that touches parameters. The leak is what buys the `'static`
    /// the borrow needs without making `ParamState` self-referential (a
    /// `ParamState` that owned the storage AND a server borrowing it is not
    /// expressible), and it is bounded — `ensure_parameter_store` runs this
    /// at most once per executor.
    ///
    /// Issue 0756 — the placement dance is still here because the SIZE is
    /// still here; it only moved off `ParameterServer` and onto
    /// `ParameterStorage`. That storage is 285,184 bytes at the default
    /// `MAX_PARAMETERS=32` and 2,281,472 at 256 (`ParameterValue` is sized by
    /// its `StringArray` variant, so every slot costs ~8.5 KiB regardless of
    /// what it holds). `Box::new(ParameterStorage::new())` materialises all of
    /// that on the caller's stack before copying it into the allocation,
    /// because Rust has no placement-new. On the Zephyr lane that silently
    /// overran the thread stack: an image built with 256 slots boots to
    /// `dds_create_participant` and hangs with no fault and no output, while
    /// 32 — which fits the 512 KiB main stack the cyclonedds snippet asks for
    /// — runs clean. Initialising through the allocation bounds the largest
    /// stack temporary at one slot, so the knob no longer decides whether boot
    /// survives.
    fn leak_parameter_storage() -> nros_params::ParameterTable<'static> {
        let mut uninit = alloc::boxed::Box::<nros_params::ParameterStorage>::new_uninit();
        // Safety: `new_uninit` gives a correctly-sized, correctly-aligned
        // allocation for exactly this type, and `init_in_place` writes every
        // slot exactly once before `assume_init` observes it.
        unsafe {
            nros_params::ParameterStorage::init_in_place(uninit.as_mut_ptr());
            alloc::boxed::Box::leak(uninit.assume_init()).as_table()
        }
    }

    /// Build the executor's `ParamState`.
    ///
    /// phase-382 W2' — this is a plain `Box::new` again: `ParameterServer` is
    /// now a table borrow plus a count, so `ParamState` is tens of bytes and
    /// nothing about constructing one depends on `MAX_PARAMETERS`. The bulk
    /// (and issue 0756's placement requirement with it) lives in
    /// [`leak_parameter_storage`](Self::leak_parameter_storage).
    fn new_param_state(
        services: Option<alloc::boxed::Box<dyn crate::parameter_services::ParamServiceProcessor>>,
    ) -> alloc::boxed::Box<crate::parameter_services::ParamState<'s>> {
        alloc::boxed::Box::new(crate::parameter_services::ParamState {
            server: nros_params::ParameterServer::new_in(Self::leak_parameter_storage()),
            services,
        })
    }

    /// Declare a parameter with a value. Returns `true` if successful.
    pub fn declare_parameter(&mut self, name: &str, value: nros_params::ParameterValue) -> bool {
        self.ensure_parameter_store();
        // phase-425 W3b — `use_sim_time` is RESERVED, exactly as in ROS 2: its
        // value is not a value the app reads, it is the switch that attaches the
        // time source. This is the one seam every language funnels through
        // (`nros::main!`'s launch bakes via `apply_param_services`, nros-c's
        // `nros_parameter_declare_*`, nros-cpp's `params_shim`), so hooking it
        // here covers all of them instead of once per entry path.
        self.note_reserved_parameter(name, &value);
        if let Some(params) = &mut self.params {
            params.server.declare(name, value)
        } else {
            false
        }
    }

    /// phase-425 W3b — record a reserved parameter's effect. Today that is
    /// `use_sim_time` and nothing else.
    ///
    /// A non-bool `use_sim_time` is IGNORED rather than rejected: parameter
    /// declaration has no channel to report a complaint on (it returns "did the
    /// store take it"), and refusing the declaration outright would fail a node
    /// for a parameter ROS 2 lets it declare. The time source simply does not
    /// attach, which is the same outcome as `false`.
    #[cfg_attr(
        not(all(feature = "sim-time", any(has_rmw, test))),
        allow(unused_variables)
    )]
    fn note_reserved_parameter(&mut self, name: &str, value: &nros_params::ParameterValue) {
        #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
        if name == crate::time_source::USE_SIM_TIME_PARAM
            && let nros_params::ParameterValue::Bool(enable) = value
        {
            self.sim_time_requested = *enable;
            self.sim_time_stated = true;
        }
    }

    /// phase-425 W3b — a parameter service handled `n` requests this spin.
    ///
    /// The hook exists so `use_sim_time` can be re-read after a runtime
    /// `ros2 param set`, WITHOUT a per-spin scan of the store. It takes the
    /// count unconditionally and ignores it when `sim-time` is off, rather than
    /// the call sites carrying the feature test: `handled` was then assigned and
    /// never read in the `param-services`-without-`sim-time` combo, which is a
    /// `-D warnings` error `check-build` catches and the narrower per-crate
    /// clippy runs do not.
    #[inline]
    fn note_param_services_ran(&mut self, handled: usize) {
        #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
        if handled > 0 {
            self.refresh_use_sim_time_from_store();
        }
        let _ = handled;
    }

    /// Declare a parameter with a value and descriptor. Returns `true` if successful.
    pub fn declare_parameter_with_descriptor(
        &mut self,
        name: &str,
        value: nros_params::ParameterValue,
        descriptor: nros_params::ParameterDescriptor,
    ) -> bool {
        self.ensure_parameter_store();
        if let Some(params) = &mut self.params {
            params
                .server
                .declare_with_descriptor(name, value, Some(descriptor))
        } else {
            false
        }
    }

    /// Get a parameter value by name.
    pub fn get_parameter(&self, name: &str) -> Option<&nros_params::ParameterValue> {
        self.params.as_ref()?.server.get(name)
    }

    /// Get an integer parameter value by name (convenience).
    pub fn get_parameter_integer(&self, name: &str) -> Option<i64> {
        self.params.as_ref()?.server.get_integer(name)
    }

    /// Get a reference to the parameter server (if registered).
    pub fn params(&self) -> Option<&nros_params::ParameterServer<'s>> {
        self.params.as_ref().map(|p| &p.server)
    }

    /// Get a mutable reference to the parameter server (if registered).
    pub fn params_mut(&mut self) -> Option<&mut nros_params::ParameterServer<'s>> {
        self.params.as_mut().map(|p| &mut p.server)
    }

    /// Create a typed parameter builder (rclrs-compatible API).
    ///
    /// Returns a [`ParameterBuilder`] for fluent parameter declaration with
    /// `.default()`, `.description()`, `.range()`, and terminal methods
    /// `.mandatory()`, `.optional()`, or `.read_only()`.
    ///
    /// Returns [`NodeError::NotInitialized`] if parameter services have
    /// not been registered yet — call [`register_parameter_services`]
    /// first.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let max_speed = executor.parameter::<f64>("max_speed")?
    ///     .default(25.0)
    ///     .description("Maximum velocity (m/s)")
    ///     .read_only()?;
    /// ```
    ///
    /// [`ParameterBuilder`]: nros_params::ParameterBuilder
    /// [`register_parameter_services`]: Self::register_parameter_services
    pub fn parameter<'a, T: nros_params::ParameterVariant>(
        &'a mut self,
        name: &'a str,
    ) -> Result<nros_params::ParameterBuilder<'a, 's, T>, NodeError> {
        let server = self
            .params
            .as_mut()
            .map(|p| &mut p.server)
            .ok_or(NodeError::NotInitialized)?;
        Ok(nros_params::ParameterBuilder::new(server, name))
    }
}

// ============================================================================
// std-gated spin and halt methods
// ============================================================================

// phase-359 W10 — this impl used to be ONE `#[cfg(feature = "std")]` block.
// Only the wall-clock spin loops in it need `std` (`Instant`, `thread::sleep`,
// `thread::spawn`); `halt` / `is_halted` / `wake` are plain atomic stores on
// flags the struct already owns, and `halt_flag` / `wake_handle` only need
// `Arc`. Gating the BLOCK rather than the items made a no_std image unable to
// stop its own executor — `ExecutorNodeRuntime::spin` carried a matching `std`
// gate for no reason but this one.
// phase-359 W10 — `alloc`, not `std`. The wall-clock spin loops were the
// reason this block was std-gated: `Instant` for the deadline and
// `thread::sleep` for the pacing. Both now go through the platform — the
// executor's own `now_us()` and `nros_platform_sleep_us` — so a no_std
// image can run the same blocking loops a hosted one does instead of
// hand-rolling them in its BSP.
#[cfg(feature = "alloc")]
impl<'s> Executor<'s> {
    /// Blocking spin loop with configurable exit conditions.
    ///
    /// Runs until one of:
    /// - [`halt()`](Self::halt) is called (from another thread or signal handler)
    /// - Timeout expires (if set in options)
    /// - Max callbacks reached (if set in options)
    /// - `only_next` is true (single iteration)
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Spin forever until halted
    /// executor.spin_blocking(SpinOptions::default())?;
    ///
    /// // Spin with 5-second timeout
    /// executor.spin_blocking(SpinOptions::new().timeout(core::time::Duration::from_secs(5)))?;
    ///
    /// // Single iteration
    /// executor.spin_blocking(SpinOptions::spin_once())?;
    /// ```
    pub fn spin_blocking(&mut self, opts: SpinOptions) -> Result<(), NodeError> {
        const POLL_INTERVAL: core::time::Duration = core::time::Duration::from_millis(10);

        // phase-359 W10 — the executor's own clock. `now_us()` reads the
        // PLATFORM's monotonic counter, and nothing else: the platform API is
        // the clock, so a build with a port has one and a build without has
        // none. There is no `std::time::Instant` third answer any more, which
        // is what used to make "what time is it" depend on whether some crate
        // in the graph happened to name `std`.
        //
        // `None` means this build has NO clock at all — and issue 0709 is what
        // the previous answer here cost. It read: "a timeout cannot be honoured
        // then, and pretending otherwise would exit immediately or never; the
        // honest reading is no deadline". It then picked NEVER. A caller that
        // asked for 50 ms got an infinite loop, silently, in the one API whose
        // whole contract is "returns after N ms" — ten hours of a CI lane
        // before anyone read it as stuck rather than slow.
        //
        // Neither of the two silent readings is honest. An unmet precondition
        // is an ERROR (repo rule, fail-loud): the caller supplied a time
        // quantity this build cannot measure, and only they can decide what to
        // do about it. An UNTIMED `spin_blocking` still runs until halt, which
        // is a promise this build can keep.
        let start_us = self.now_us();
        if opts.timeout.is_some() && start_us.is_none() {
            nros_log::nros_error!(
                nros_log::get_logger("nros"),
                "spin_blocking: a timeout was requested but this build has no clock \
                 — install one with `ExecutorConfig::clock_us` (issue 0709)"
            );
            return Err(NodeError::NotInitialized);
        }
        // `as_micros()` rather than a hand-rolled `ms * 1_000`: the multiply
        // was unchecked, and the unit conversion is `Duration`'s job.
        let timeout_us = opts.timeout.map(|d| d.as_micros() as u64);
        let mut total_callbacks = 0usize;

        self.halt_flag
            .store(false, core::sync::atomic::Ordering::SeqCst);

        loop {
            if self.halt_flag.load(core::sync::atomic::Ordering::SeqCst) {
                break;
            }

            if let (Some(start), Some(limit)) = (start_us, timeout_us)
                && self
                    .now_us()
                    .is_some_and(|now| now.saturating_sub(start) >= limit)
            {
                break;
            }

            let result = self.spin_once(POLL_INTERVAL);
            total_callbacks += result.total();

            if opts.max_callbacks.is_some_and(|max| total_callbacks >= max) {
                break;
            }

            if opts.only_next {
                break;
            }
        }

        Ok(())
    }

    /// Execute one period with wall-clock overrun detection.
    ///
    /// Calls [`spin_once()`](Self::spin_once), measures wall-clock time, sleeps
    /// for the remainder if under budget.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let period = core::time::Duration::from_millis(10);
    /// let result = executor.spin_one_period_timed(period);
    /// if result.overrun {
    ///     log::warn!("Period overrun: {:?}", result.elapsed);
    /// }
    /// ```
    pub fn spin_one_period_timed(
        &mut self,
        period: core::time::Duration,
    ) -> super::types::SpinPeriodResult {
        let start_us = self.now_us();
        let result = self.spin_once(period);
        // With no clock there is nothing to measure and nothing to sleep off;
        // report zero elapsed and no overrun, which is what an unmeasured
        // period is.
        let elapsed = start_us
            .zip(self.now_us())
            .map(|(s, e)| core::time::Duration::from_micros(e.saturating_sub(s)))
            .unwrap_or_default();
        let overrun = elapsed > period;
        if !overrun && start_us.is_some() {
            platform_sleep(period - elapsed);
        }
        super::types::SpinPeriodResult {
            work: result,
            overrun,
            elapsed,
        }
    }

    /// Spin at a fixed rate with drift compensation. Blocks until halted.
    ///
    /// Uses wall-clock time to maintain the target rate. The next invocation
    /// time is accumulated (not reset to `now + period`) to prevent cumulative
    /// drift.
    ///
    /// # Example
    ///
    /// ```ignore
    /// // 100Hz control loop — blocks until halt() is called
    /// executor.spin_period(core::time::Duration::from_millis(10))?;
    /// ```
    pub fn spin_period(&mut self, period: core::time::Duration) -> Result<(), NodeError> {
        self.halt_flag
            .store(false, core::sync::atomic::Ordering::SeqCst);
        let period_us = period.as_micros().min(u64::MAX as u128) as u64;
        // Absolute next-deadline in the executor's own clock. issue 0709 — the
        // sibling of `spin_blocking`'s guard above: with no clock there is no
        // pacing, and the loop would run as fast as `spin_once` returns while
        // the caller believes it is running at `period`. A requested period
        // this build cannot honour is a configuration error, not a silent
        // busy-loop.
        let Some(start_us) = self.now_us() else {
            nros_log::nros_error!(
                nros_log::get_logger("nros"),
                "spin_period: a period was requested but this build has no clock \
                 — install one with `ExecutorConfig::clock_us` (issue 0709)"
            );
            return Err(NodeError::NotInitialized);
        };
        let mut next_us = Some(start_us + period_us);

        loop {
            if self.halt_flag.load(core::sync::atomic::Ordering::SeqCst) {
                break;
            }

            self.spin_once(period);

            if let Some(next) = next_us {
                // Jitter is recorded in `spin_once` above, which this loop
                // calls every iteration -- counting it here as well would
                // double every wake on this one driver.
                if let Some(now) = self.now_us()
                    && now < next
                {
                    platform_sleep(core::time::Duration::from_micros(next - now));
                }
                // Accumulate to prevent drift (not = now + period)
                next_us = Some(next + period_us);
            }
        }
        Ok(())
    }
}

#[cfg(feature = "alloc")]
impl<'s> Executor<'s> {
    /// Phase 110.D.b — move this Executor onto a fresh OS thread,
    /// apply a per-thread scheduling policy via the caller-supplied
    /// `apply_policy` function, and run the spin loop until
    /// [`ThreadHandle::halt`] fires.
    ///
    /// The function-pointer indirection on `apply_policy` lets the
    /// caller pass any platform's `PlatformScheduler::set_current_thread_policy`
    /// without forcing `Executor` to be generic over the platform —
    /// keeps the existing `Executor` type stable.
    ///
    /// Multi-executor preemption (the actual hard-RT win) comes from
    /// the OS scheduler — call `open_threaded` once per criticality
    /// tier, each with its own policy / priority. The kernel handles
    /// preemption across executors; within a single executor,
    /// dispatch remains non-preemptive (110.A–C bucketed sets).
    ///
    /// # Safety
    ///
    /// Moves `self` across thread boundaries. `Executor` contains a
    /// raw `*mut session::ConcreteSession` when constructed via
    /// `from_session_ptr`; the caller must ensure that pointer's
    /// referent stays valid across the lifetime of the spawned thread
    /// and that no other thread mutates the session concurrently.
    /// `from_session` (Owned) is safer — `ConcreteSession` ownership
    /// transfers cleanly into the thread.
    #[cfg(feature = "alloc")]
    pub unsafe fn open_threaded(
        self,
        policy: nros_platform_api::SchedPolicy,
        apply_policy: fn(
            nros_platform_api::SchedPolicy,
        ) -> Result<(), nros_platform_api::SchedError>,
        spin_period: core::time::Duration,
    ) -> ThreadHandle
    where
        // phase-271 — the spawned thread owns `self` for an unbounded lifetime,
        // so its borrowed storage must be `'static` (a leaked/`static` backing —
        // the `alloc` convenience constructors or a program-lifetime region).
        's: 'static,
    {
        let halt = self.halt_flag.clone();
        // phase-359 W10 — a platform task, not `std::thread`. The two other
        // executor-owned workers moved earlier; this is the third and last, and
        // it is the one the ABI fits best: `open_threaded` exists to give a
        // spin loop an OS SCHEDULING POLICY, which is not something a build
        // without an OS was ever going to get from `std`.
        //
        // The closure becomes a heap context because the entry point crosses C
        // as `*mut c_void`. It is reclaimed by the trampoline on exit rather
        // than leaked: unlike a tier task, this loop RETURNS — that is what
        // `halt` is for.
        let ctx = alloc::boxed::Box::into_raw(alloc::boxed::Box::new(ThreadedCtx {
            executor: self,
            policy,
            apply_policy,
            spin_period,
        }));
        // SAFETY: `ctx` stays live until the trampoline reclaims it, which
        // happens on the spawned task and only after the loop exits.
        let task = unsafe {
            nros_platform_api::task::PlatformTask::spawn(
                threaded_spin_trampoline,
                ctx as *mut core::ffi::c_void,
                OPEN_THREADED_STACK_BYTES,
                c"nros-exec".as_ptr(),
            )
        };
        let Some(task) = task else {
            // The platform refused (or cannot size) a task. Reclaim the
            // context and hand back a handle that owns nothing — `halt` and
            // `join` stay callable, which keeps the caller's shape identical to
            // the success path.
            //
            // SAFETY: nothing was spawned, so this pointer has no other owner.
            drop(unsafe { alloc::boxed::Box::from_raw(ctx) });
            return ThreadHandle { task: None, halt };
        };
        ThreadHandle {
            task: Some(task),
            halt,
        }
    }

    /// Request the executor to stop spinning.
    ///
    /// Sets a flag that causes [`spin_blocking()`](Self::spin_blocking) or
    /// [`spin_period()`](Self::spin_period) to exit on the next iteration.
    /// Safe to call from another thread or signal handler.
    ///
    /// Also raises the Phase 104.C.6 wake flag so a `spin_once` already
    /// blocked inside a backend's `drive_io` falls through to the halt
    /// check on its next loop iteration instead of waiting out its full
    /// `timeout_ms` first.
    pub fn halt(&self) {
        self.halt_flag
            .store(true, core::sync::atomic::Ordering::SeqCst);
        self.wake_flag
            .store(true, core::sync::atomic::Ordering::SeqCst);
    }

    /// Check if halt has been requested.
    pub fn is_halted(&self) -> bool {
        self.halt_flag.load(core::sync::atomic::Ordering::SeqCst)
    }

    /// Phase 104.C.6 — wake the executor from another thread / ISR /
    /// signal handler.
    ///
    /// Sets the shared `wake_flag`. The next `spin_once` swap-clears the
    /// flag, skips the blocking wait on the primary session, and polls
    /// every session non-blockingly so whatever queued the wake is
    /// observed in a single iteration. Idempotent — multiple `wake()`
    /// calls collapse into one observed wake per `spin_once`.
    pub fn wake(&self) {
        self.wake_flag
            .store(true, core::sync::atomic::Ordering::SeqCst);
    }
}

#[cfg(feature = "alloc")]
impl<'s> Executor<'s> {
    /// Get a clone of the halt flag for use in signal handlers or other threads.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let halt = executor.halt_flag();
    /// std::thread::spawn(move || {
    ///     std::thread::sleep(Duration::from_secs(5));
    ///     halt.store(true, Ordering::SeqCst);
    /// });
    /// executor.spin_blocking(SpinOptions::default())?;
    /// ```
    pub fn halt_flag(&self) -> portable_atomic_util::Arc<portable_atomic::AtomicBool> {
        self.halt_flag.clone()
    }

    /// Phase 104.C.6 — clone of the shared wake flag for cross-thread
    /// use (signal handlers, foreign threads, future per-backend vtable
    /// wake hooks).
    ///
    /// # Example
    ///
    /// ```ignore
    /// let wake = executor.wake_handle();
    /// std::thread::spawn(move || {
    ///     // ... compute something ...
    ///     // hand off to executor by setting the flag.
    ///     wake.store(true, Ordering::SeqCst);
    /// });
    /// loop { executor.spin_once(Duration::from_millis(100)); }
    /// ```
    pub fn wake_handle(&self) -> portable_atomic_util::Arc<portable_atomic::AtomicBool> {
        self.wake_flag.clone()
    }
}

/// Phase 110.E.b — opaque per-platform timer handle. Stores the
/// raw platform handle (POSIX `timer_t` boxed via `PosixTimerHandle`,
/// FreeRTOS `TimerHandle_t`, etc.) plus a destroy thunk so the
/// Executor can clean up without being generic over the platform.
///
/// Caller of `register_sporadic_timer` builds this via
/// `OpaqueTimerHandle::new(handle, destroy_fn)` after their
/// `PlatformTimer::create_periodic` call returns.
#[cfg(feature = "alloc")]
pub struct OpaqueTimerHandle {
    handle: *mut core::ffi::c_void,
    destroy_fn: extern "C" fn(*mut core::ffi::c_void),
}

#[cfg(feature = "alloc")]
unsafe impl Send for OpaqueTimerHandle {}
#[cfg(feature = "alloc")]
unsafe impl Sync for OpaqueTimerHandle {}

#[cfg(feature = "alloc")]
impl OpaqueTimerHandle {
    /// # Safety
    /// `handle` must be a live platform-specific timer handle that
    /// `destroy_fn` knows how to drop. Caller surrenders ownership
    /// of the underlying handle to the Executor.
    pub unsafe fn new(
        handle: *mut core::ffi::c_void,
        destroy_fn: extern "C" fn(*mut core::ffi::c_void),
    ) -> Self {
        Self { handle, destroy_fn }
    }
}

#[cfg(feature = "alloc")]
impl Drop for OpaqueTimerHandle {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            (self.destroy_fn)(self.handle);
            self.handle = core::ptr::null_mut();
        }
    }
}

/// Handle returned from [`Executor::open_threaded`]. Holds the
/// spawned thread's join handle and a clone of the executor's halt
/// flag. Drop runs `halt() + join()` so the thread can't outlive the
/// handle.
#[cfg(feature = "alloc")]
pub struct ThreadHandle {
    task: Option<nros_platform_api::task::PlatformTask>,
    halt: portable_atomic_util::Arc<portable_atomic::AtomicBool>,
}

/// Default stack for an `open_threaded` spin loop. `std::thread`'s default was
/// 2 MiB; the executor's own storage is caller-carved and lives elsewhere, so
/// this carries call frames only — the same reasoning (and size) as the NuttX
/// tier spawn, which measured this against a real RTOS default.
#[cfg(feature = "alloc")]
const OPEN_THREADED_STACK_BYTES: usize = 65536;

/// What [`Executor::open_threaded`] hands its task, in place of a closure's
/// captures. Owned by the trampoline, which reclaims it when the loop exits.
#[cfg(feature = "alloc")]
struct ThreadedCtx {
    executor: Executor<'static>,
    policy: nros_platform_api::SchedPolicy,
    apply_policy: fn(nros_platform_api::SchedPolicy) -> Result<(), nros_platform_api::SchedError>,
    spin_period: core::time::Duration,
}

/// The spawned spin loop.
///
/// # Safety
/// `arg` must be the `Box<ThreadedCtx>` raw pointer `open_threaded` created,
/// passed exactly once.
#[cfg(feature = "alloc")]
unsafe extern "C" fn threaded_spin_trampoline(
    arg: *mut core::ffi::c_void,
) -> *mut core::ffi::c_void {
    // SAFETY: the caller's contract — this is the pointer `open_threaded`
    // leaked, and the task is the only consumer of it.
    let mut ctx = unsafe { alloc::boxed::Box::from_raw(arg as *mut ThreadedCtx) };
    // Apply the requested OS scheduling policy to this fresh task. Failure is
    // reported but not propagated — a runtime that fails to lift to SCHED_FIFO
    // still spins correctly at SCHED_OTHER (just without RT guarantees).
    let _ = (ctx.apply_policy)(ctx.policy);
    while !ctx.executor.is_halted() {
        ctx.executor.spin_once(ctx.spin_period);
    }
    core::ptr::null_mut()
}

#[cfg(feature = "alloc")]
impl ThreadHandle {
    /// Signal the spawned executor thread to stop. The thread exits
    /// on its next `spin_once` iteration.
    pub fn halt(&self) {
        self.halt.store(true, core::sync::atomic::Ordering::SeqCst);
    }

    /// Wait for the spawned task to exit. After `join`, calling it again is a
    /// no-op.
    ///
    /// phase-359 W10 — was `std::thread::Result<()>`, which carried a panic
    /// payload this can no longer produce: a platform task has no unwinding
    /// join. Same two outcomes, an error type that does not require `std` —
    /// the trade `signal_fd` already made in this campaign. `NotInitialized`
    /// means the platform refused to host the task at spawn time — nothing was
    /// started, so there is nothing to wait for. (An existing variant rather
    /// than a new one: `NodeError` is mapped across the C and C++ FFI, so a new
    /// variant is a gate-checked ABI change and this needs no new meaning.)
    pub fn join(mut self) -> Result<(), NodeError> {
        self.halt();
        match self.task.take() {
            Some(t) => {
                t.join();
                Ok(())
            }
            None => Err(NodeError::NotInitialized),
        }
    }
}

#[cfg(feature = "alloc")]
impl Drop for ThreadHandle {
    fn drop(&mut self) {
        self.halt.store(true, core::sync::atomic::Ordering::SeqCst);
        if let Some(t) = self.task.take() {
            t.join();
        }
    }
}

// SAFETY: Phase 110.D.b — `Executor` contains a raw `*mut
// session::ConcreteSession` only on the `from_session_ptr` (Borrowed)
// path; the `from_session` (Owned) path is plain Send-able. The
// `unsafe fn open_threaded` entry point documents the safety
// contract for Borrowed sessions; for Owned sessions the Send claim
// is unconditional.
// phase-359 W10 — `alloc`, not `std`: this assertion exists for
// `open_threaded`, which now hands the executor to a PLATFORM task. The
// crossing is the same one; only the thing doing the crossing changed.
#[cfg(feature = "alloc")]
unsafe impl<'s> Send for Executor<'s> {}

// =============================================================================
// Phase 110.F — `OsPriorityWorker` + `WorkItem`
// =============================================================================

// phase-359 W10 — `OsPriorityWorker` / `WorkItem` moved to
// `super::os_priority` and were rewritten off `std::thread` + `std::sync::mpsc`
// + `HashMap` onto the platform task ABI, a bounded `heapless` mailbox and a
// `NodeWake` doorbell. The capability is no longer std-only; see that module
// for what changed semantically (bounded mailbox, capacity-limited pool, and
// no pool on a platform without a wake primitive).

impl<'s> Drop for Executor<'s> {
    fn drop(&mut self) {
        // Issue 0790 — a teardown that never went through `close()` still gets
        // its ordered shutdown hooks, and gets them in the SAME order. The C
        // API's `nros_executor_fini` is exactly that path: it drops the
        // executor in place and leaves the session to `nros_support_fini`, so
        // without this the whole facility would be silently inert for every C
        // entry. Entities are still live at the top of `drop` — the component
        // cells and the arena entries below are what tears them down — so this
        // is the last moment a pre-shutdown hook can do what it exists to do.
        //
        // After a normal `close()` both tables are already empty and these two
        // calls are a pair of `None` scans.
        self.run_shutdown_hooks(super::types::ShutdownPhase::Pre);
        // Phase 258 (Track 2, 2a) — release executor-owned component cells
        // first (before the arena entries), so a component's `drop`
        // trampoline can still touch its own (cell-owned) state. Each slot
        // owns a leaked `Arc<ComponentCell>`; its `drop` reconstitutes +
        // drops that Arc exactly once.
        for slot in self.component_slots.iter() {
            // SAFETY: `slot.state` is the leaked cell enrolled via
            // `enroll_component`; `slot.drop` is its matching trampoline, run
            // exactly once here (slots are not removed before Drop).
            unsafe {
                (slot.drop)(slot.state);
            }
        }
        let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
        for meta in self.entries.iter().flatten() {
            // SAFETY: each entry was written by `ptr::write` in `add_*` and
            // has not been dropped yet. `drop_fn` matches the concrete type.
            unsafe {
                let data_ptr = arena_ptr.add(meta.offset);
                (meta.drop_fn)(data_ptr);
            }
        }
        // Issue 0790 — the post-teardown half, after the entities are gone.
        // Same "already empty after `close()`" note as the pre pass above.
        self.run_shutdown_hooks(super::types::ShutdownPhase::Post);
    }
}

#[cfg(all(test, feature = "std", not(feature = "rmw-cffi")))]
mod dispatch_registry_tests {
    //! Phase 216 follow-up — `Executor::register_dispatch_slot` +
    //! `Executor::dispatch_callback` round-trip.
    //!
    //! Uses `MockSession` (same pattern as
    //! `lifecycle_services::tests::mock_integration`) so the test
    //! doesn't need a live RMW backend. Gated `not(feature =
    //! "rmw-cffi")` because under `rmw-cffi` the `ConcreteSession`
    //! type alias resolves to the cffi session, which `MockSession`
    //! can't impersonate.

    extern crate alloc;

    use super::Executor;
    use crate::mock::MockSession;
    use std::sync::Mutex;

    static CAPTURED: Mutex<alloc::vec::Vec<(usize, alloc::vec::Vec<u8>, usize)>> =
        Mutex::new(alloc::vec::Vec::new());

    /// Test trampoline matching the per-Node
    /// `__nros_node_<pkg>_on_callback` ABI shape (Phase 216.A.5).
    unsafe extern "C" fn recording_on_callback(
        state: *mut core::ffi::c_void,
        cb_id_ptr: *const u8,
        cb_id_len: usize,
        ctx: *mut core::ffi::c_void,
    ) {
        // SAFETY: caller (test body below) holds storage live;
        // `cb_id_ptr..len` points into a `&str` literal.
        let cb_id_bytes = unsafe { core::slice::from_raw_parts(cb_id_ptr, cb_id_len).to_vec() };
        let mut guard = CAPTURED.lock().expect("CAPTURED poisoned");
        guard.push((state as usize, cb_id_bytes, ctx as usize));
    }

    #[test]
    fn register_dispatch_slot_round_trip() {
        let session = MockSession::new();
        let mut executor: Executor = Executor::from_session(session);

        // Pre-condition: empty registry.
        assert_eq!(executor.dispatch_slot_count(), 0);

        // Two distinct "states" so we prove every slot gets called
        // with its OWN state.
        let mut state_blob_a: u32 = 0xABCD_0001;
        let mut state_blob_b: u32 = 0xABCD_0002;
        let state_a_ptr = &mut state_blob_a as *mut u32 as *mut core::ffi::c_void;
        let state_b_ptr = &mut state_blob_b as *mut u32 as *mut core::ffi::c_void;

        executor
            .register_dispatch_slot(state_a_ptr, recording_on_callback)
            .expect("register slot A");
        executor
            .register_dispatch_slot(state_b_ptr, recording_on_callback)
            .expect("register slot B");
        assert_eq!(executor.dispatch_slot_count(), 2);

        let mut ctx_blob: u32 = 0xFEED_BEEF;
        let ctx_ptr = &mut ctx_blob as *mut u32 as *mut core::ffi::c_void;
        let cb_id = "/talker/timer/publish";

        CAPTURED.lock().expect("CAPTURED poisoned").clear();
        executor.dispatch_callback(cb_id, ctx_ptr);

        let captured = CAPTURED.lock().expect("CAPTURED poisoned").clone();
        assert_eq!(
            captured.len(),
            2,
            "every registered slot must be invoked — linear scan, \
             no self-filter at the registry layer"
        );
        // The carved dispatch table iterates in insertion order.
        assert_eq!(captured[0].0, state_a_ptr as usize, "slot A's state");
        assert_eq!(captured[1].0, state_b_ptr as usize, "slot B's state");
        for (idx, capture) in captured.iter().enumerate() {
            assert_eq!(
                capture.1.as_slice(),
                cb_id.as_bytes(),
                "slot {idx} cb_id bytes round-trip"
            );
            assert_eq!(
                capture.2, ctx_ptr as usize,
                "slot {idx} ctx pointer round-trip"
            );
        }
    }

    #[test]
    fn register_dispatch_slot_capacity_full() {
        let session = MockSession::new();
        let mut executor: Executor = Executor::from_session(session);

        let mut state_blob: u32 = 0;
        let state_ptr = &mut state_blob as *mut u32 as *mut core::ffi::c_void;

        // `MAX_NODES` slots fit; the next one must error.
        for _ in 0..crate::config::MAX_NODES {
            executor
                .register_dispatch_slot(state_ptr, recording_on_callback)
                .expect("under-capacity push must succeed");
        }
        assert_eq!(executor.dispatch_slot_count(), crate::config::MAX_NODES);
        let overflow = executor.register_dispatch_slot(state_ptr, recording_on_callback);
        assert!(
            overflow.is_err(),
            "over-capacity push must return Err(()) — raise \
             NROS_EXECUTOR_MAX_NODES at build time to grow the registry"
        );
    }
}

/// Phase 274.W1 — borrowed-executor session sharing + active-groups gating.
///
/// Validates three primitives introduced for RFC-0015 Model 1:
/// - `session_handle` / `open_with_session_handle` (Borrowed session store —
///   the borrowed executor does not own or close the session on drop).
/// - `set_active_groups` + `group_active` (callback-group filter gating).
///
/// Uses `MockSession` (same pattern as `dispatch_registry_tests`); gated
/// `not(feature = "rmw-cffi")` for the same reason.
#[cfg(all(test, feature = "std", not(feature = "rmw-cffi")))]
mod p274_w1_tier_executor_tests {
    use super::Executor;
    use crate::mock::MockSession;

    #[test]
    fn session_handle_borrowed_executor_shares_session_ptr() {
        // Open the primary executor (session owner).
        let session = MockSession::new();
        let mut primary = Executor::from_session(session);

        // Record the primary's session pointer for later comparison.
        let primary_session_ptr = primary.session_ptr();

        // Get the opaque session handle (into_raw for C FFI; here we keep it raw).
        let handle = primary.session_handle();

        // Open a second executor over the SAME session (Borrowed — does not own it).
        // SAFETY: `primary` (the session owner) outlives `borrowed` in this scope.
        let mut borrowed = unsafe { Executor::open_with_session_handle(handle) };

        // Both executors must expose the same session pointer.
        assert_eq!(
            primary.session_ptr(),
            borrowed.session_ptr(),
            "borrowed executor must share the primary's session pointer"
        );
        assert_eq!(
            borrowed.session_ptr(),
            primary_session_ptr,
            "session pointer must be stable across session_handle / open_with_session_handle"
        );

        // Drop the borrowed executor — the primary's session must remain valid.
        drop(borrowed);

        // Primary still exposes the same session pointer (session was NOT closed by drop).
        assert_eq!(
            primary.session_ptr(),
            primary_session_ptr,
            "primary session pointer must be unchanged after dropping the borrowed executor"
        );
    }

    #[test]
    fn set_active_groups_gates_group_active() {
        let session = MockSession::new();
        let mut primary = Executor::from_session(session);
        let handle = primary.session_handle();

        // SAFETY: primary outlives borrowed.
        let mut borrowed = unsafe { Executor::open_with_session_handle(handle) };

        // Before gating: wildcard — every group is accepted.
        assert!(
            borrowed.group_active("ctrl"),
            "default (wildcard) must accept every group"
        );
        assert!(
            borrowed.group_active("telem"),
            "default (wildcard) must accept every group"
        );

        // Gate borrowed to only the "ctrl" group (one-tier filter).
        borrowed.set_active_groups(&["ctrl"]);

        assert!(
            borrowed.group_active("ctrl"),
            "\"ctrl\" must be active after set_active_groups([\"ctrl\"])"
        );
        assert!(
            !borrowed.group_active("telem"),
            "\"telem\" must NOT be active when only \"ctrl\" is gated"
        );
        assert!(
            !borrowed.group_active("planning"),
            "\"planning\" must NOT be active when only \"ctrl\" is gated"
        );

        // Primary is unaffected (it still uses the wildcard).
        assert!(
            primary.group_active("telem"),
            "primary executor must remain unaffected (still wildcard)"
        );

        // Clear the filter on borrowed — back to wildcard.
        borrowed.set_active_groups(&[]);
        assert!(
            borrowed.group_active("telem"),
            "after clearing, borrowed must accept all groups again"
        );
    }

    #[test]
    fn session_handle_into_raw_from_raw_round_trip() {
        let session = MockSession::new();
        let mut primary = Executor::from_session(session);
        let session_ptr = primary.session_ptr();

        let handle = primary.session_handle();
        let raw = handle.into_raw();

        // into_raw must return a non-null pointer matching the session address.
        assert!(!raw.is_null());
        assert_eq!(raw as *mut _, session_ptr);

        // from_raw must reconstruct a handle that opens the same borrowed executor.
        // SAFETY: primary still owns the session, raw is its address.
        let handle2 = unsafe { crate::executor::SessionHandle::from_raw(raw) };
        let mut borrowed = unsafe { Executor::open_with_session_handle(handle2) };
        assert_eq!(
            borrowed.session_ptr(),
            session_ptr,
            "from_raw reconstructed handle must open executor on the same session"
        );
    }
}

/// W3b.5 — snapshot the monitored publishers' counters before a dispatch
/// (only when a latency contract exists; otherwise a zeroed array that
/// `attribute_latency` never reads).
fn snapshot_pub_counts(
    table: &'static [super::monitor::MonitorSpec],
    active: bool,
) -> [u32; super::monitor::MAX_MONITORS] {
    let mut counts = [0u32; super::monitor::MAX_MONITORS];
    if active {
        for (k, spec) in table.iter().take(super::monitor::MAX_MONITORS).enumerate() {
            counts[k] = spec.cell.count.load(core::sync::atomic::Ordering::Relaxed);
        }
    }
    counts
}

/// W3b.5 — attribute one dispatch's elapsed time to every monitored
/// publisher whose counter advanced during it (an upper bound on the
/// node-path take → publish latency: the callback deserialized, ran, and
/// published within `elapsed_us`).
fn attribute_latency(
    table: &'static [super::monitor::MonitorSpec],
    active: bool,
    counts_before: &[u32; super::monitor::MAX_MONITORS],
    elapsed_us: u32,
) {
    if !active {
        return;
    }
    for (k, spec) in table.iter().take(super::monitor::MAX_MONITORS).enumerate() {
        if spec.max_latency_ms == 0 {
            continue;
        }
        let now = spec.cell.count.load(core::sync::atomic::Ordering::Relaxed);
        if now != counts_before[k] {
            spec.cell
                .max_latency_us
                .fetch_max(elapsed_us, core::sync::atomic::Ordering::Relaxed);
        }
    }
}

/// W3b.5 — enforce the bound SC's deadline after a dispatch. A miss maps
/// through [`DeadlineAction`](super::sched_context::DeadlineAction):
/// `Warn`/`Skip`/`Fault` all report; `Skip` additionally masks the SC's
/// remaining callbacks for this cycle; `Fault` invokes the fault hook
/// (panic when none is registered).
fn check_deadline_miss(
    sc: Option<&super::sched_context::SchedContext>,
    sc_idx: usize,
    elapsed_us: u32,
    misses: &mut heapless::Vec<super::monitor::Violation, { super::monitor::MAX_VIOLATIONS }>,
    skipped_scs: &mut u64,
    fault_fn: Option<fn(&super::monitor::Violation)>,
) {
    use super::sched_context::DeadlineAction;
    let Some(sc) = sc else { return };
    let Some(deadline_us) = sc.deadline_us.get().map(|nz| nz.get()) else {
        return;
    };
    if matches!(sc.deadline_action, DeadlineAction::Ignore) || elapsed_us <= deadline_us {
        return;
    }
    let v = super::monitor::Violation {
        rule: "deadline-miss-runtime",
        // Entries carry no name at this altitude; the SC slot stands in.
        fqn: "sched-context",
        measured: elapsed_us,
        declared: deadline_us,
    };
    match sc.deadline_action {
        DeadlineAction::Ignore => {}
        DeadlineAction::Warn => {
            let _ = misses.push(v);
        }
        DeadlineAction::Skip => {
            if sc_idx < 64 {
                *skipped_scs |= 1u64 << sc_idx;
            }
            let _ = misses.push(v);
        }
        DeadlineAction::Fault => {
            if let Some(f) = fault_fn {
                f(&v);
                let _ = misses.push(v);
            } else {
                panic!(
                    "nros: deadline fault — dispatch ran {elapsed_us} us past a {deadline_us} us deadline"
                );
            }
        }
    }
}

// Timer-accounting clock default (issue: no_std tiers with no `clock_us`
// credited each spin the REQUESTED timeout — spin.rs `delta_us` fallback —
// so shared-session wakes made low-rate timers fire early (a 100 ms timer
// at ~67 ms on Zephyr native_sim) and tick rounding made fast tiers fire
// late. Measured on-target; mechanism documented at the fallback site.)
//
// Every platform port exports `nros_platform_clock_ns` through the same
// linkage contract the wake primitives rely on (`nros_platform_export_clock!`
// in nros-platform-cffi), so an rmw-cffi no_std build can safely default the
// executor clock to it; `ExecutorConfig::clock_us` stays as an override.
#[cfg(feature = "rmw-cffi")]
unsafe extern "C" {
    fn nros_platform_clock_ns() -> u64;
}

#[cfg(feature = "rmw-cffi")]
fn default_platform_clock_us() -> u64 {
    // SAFETY: bare query of the platform's monotonic ns counter; the symbol
    // is guaranteed by whichever platform port linked the binary (the same
    // contract `nros_platform_wake_*` already depends on).
    //
    // RFC-0073 made the ABI nanoseconds; the executor's own accounting is
    // still microseconds (`clock_us_fn`, `delta_us`), so the division lives
    // here rather than being pushed into every port.
    unsafe { nros_platform_clock_ns() / 1_000 }
}

/// Sleep, through the platform ABI.
///
/// phase-359 W10 — the spin loops used `std::thread::sleep`. Every port already
/// exports `nros_platform_sleep_us` (the ABI's own pacing primitive, the one an
/// RTOS build has always used), so a hosted loop and an embedded one now pace
/// the same way. The µs entry point is used rather than `_ms` because
/// `spin_one_period_timed` sleeps off a remainder, which rounds badly at
/// millisecond granularity on short periods.
#[cfg(feature = "alloc")]
pub(crate) fn platform_sleep(d: core::time::Duration) {
    unsafe extern "C" {
        fn nros_platform_sleep_us(us: usize);
    }
    let us = d.as_micros().min(usize::MAX as u128) as usize;
    if us == 0 {
        return;
    }
    // SAFETY: a bare pacing call with no pointer arguments, guaranteed by
    // whichever port linked the binary — the same contract the clock and wake
    // symbols in this file already rely on.
    unsafe { nros_platform_sleep_us(us) }
}

/// A monotonic µs reader, or `None` when this build has no clock at all.
///
/// phase-359 W10 — one provider, chosen by what is LINKED rather than by which
/// flavour the crate was built in. W4 unified the executor's clock ACCESSOR and
/// said the provider was the last piece; this is it.
///
/// `rmw-cffi` means a platform port is linked, and every port exports
/// `nros_platform_clock_ns` under the same contract the wake primitives use —
/// so a hosted build reads the same counter an embedded one does, instead of
/// `Instant` on one and the platform on the other. `Instant` survives only
/// where there is no port to ask: `std` without `rmw-cffi`.
///
/// phase-359 W10 follow-up — this comment claimed the metadata PROBE was that
/// configuration, and **it is not**. The probe's generated manifest deps
/// `nros-platform-cffi` with `posix-c-port` (issue 0288 layer 5, so it links),
/// and its `nros` resolves with `rmw-cffi` ON — feature-unified from the
/// component's board crate. Measured on a real probe tree: `alloc default
/// macros rmw-cffi ros-humble std`. It takes the PORT arm.
///
/// What is real: `nros-rmw-metadata` and `nros-tests` do resolve `nros` with
/// `std` and no `rmw-cffi`, and have no port crate in their graphs. But nothing
/// in either uses this arm — deleting it and building the workspace
/// `--all-targets` is clean, and there is no in-tree caller of
/// `nros::time::now` at all. So the arm serves an OUT-OF-TREE contract only: a
/// consumer with `std`, no `rmw-cffi`, and its own `Session`.
pub(crate) fn default_clock_us_fn() -> Option<fn() -> u64> {
    #[cfg(feature = "rmw-cffi")]
    {
        Some(default_platform_clock_us)
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        None
    }
}

// =============================================================================
// phase-425 W3b — the `/clock` time source's executor half.
//
// Its OWN impl block, gated on `sim-time`, because the methods were first
// written next to `declare_parameter` — which lives in a
// `#[cfg(feature = "param-services")]` block, so `reconcile_ros_time_source`
// silently disappeared in every combo without parameter services, including the
// one `spin_once` calls it from.
// =============================================================================
#[cfg(all(feature = "sim-time", any(has_rmw, test)))]
impl<'s> Executor<'s> {
    /// phase-425 W3b — bring the `/clock` subscription in line with the last
    /// requested `use_sim_time`.
    ///
    /// Called at the head of every spin, and cheap when there is nothing to do:
    /// the common case is one bool comparison. It is a RECONCILE rather than an
    /// action at the request site because the request routinely arrives before
    /// there is a node to hang the subscription on — `nros::main!` emits
    /// `apply_param_services` before its per-node `register` calls, by design,
    /// so the store exists when each cell is created.
    ///
    /// Turning it off stops SAMPLES from being installed; the subscription
    /// itself stays, because the executor has no entity removal and inventing
    /// one for this would be a much larger change than the switch is worth. The
    /// gate is `time_source::set_active`, which the subscription callback reads.
    ///
    /// Turning it off also does NOT clear the override: a node that stops
    /// listening keeps the last simulated time rather than jumping back to the
    /// wall clock, which every ROS-time timer would otherwise absorb as a
    /// backwards jump.
    #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
    pub(crate) fn reconcile_ros_time_source(&mut self) {
        // An executor nobody told about `use_sim_time` has NO opinion, and the
        // gate is process-global, so writing its default here is not a no-op --
        // it is one executor overruling another. Say nothing until asked.
        if !self.sim_time_stated {
            return;
        }
        if self.sim_time_requested == crate::time_source::is_active()
            && (!self.sim_time_requested || self.sim_time_source.is_some())
        {
            return;
        }
        crate::time_source::set_active(self.sim_time_requested);
        if self.sim_time_requested && self.sim_time_source.is_none() {
            // No node yet — stay pending and try again next spin. Not an error:
            // it is the ordinary order of a generated entry, which declares
            // parameters before it registers components.
            if self.nodes.is_empty() {
                return;
            }
            if let Ok(handle) = self.install_ros_time_source(
                super::node_record::NodeId::PRIMARY,
                crate::time_source::CLOCK_TOPIC,
            ) {
                self.sim_time_source = Some(handle);
            }
        }
    }

    /// phase-425 W3b — re-read `use_sim_time` from the parameter store.
    ///
    /// Called after a parameter service actually handled something, which is
    /// what makes a runtime `ros2 param set <node> use_sim_time true` work
    /// without a per-spin scan of the store. The declaration path does not need
    /// it — `declare_parameter` records the value directly.
    #[cfg(all(feature = "sim-time", feature = "param-services", any(has_rmw, test)))]
    fn refresh_use_sim_time_from_store(&mut self) {
        if let Some(params) = self.params.as_ref()
            && let Some(enable) = params
                .server
                .get_bool(crate::time_source::USE_SIM_TIME_PARAM)
        {
            self.sim_time_requested = enable;
            self.sim_time_stated = true;
        }
    }

    /// phase-425 W3b — the installed `/clock` subscription, if any.
    ///
    /// The HANDLE rather than a bool, because "did we subscribe twice" is the
    /// question a reconciliation loop has to be able to answer: a second
    /// registration takes a new slot, so a stable handle across spins is the
    /// evidence that the loop is idempotent.
    #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
    pub fn ros_time_source_handle(&self) -> Option<HandleId> {
        self.sim_time_source
    }

    /// phase-425 W3b — whether a `/clock` subscription is currently installed.
    #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
    pub fn ros_time_source_installed(&self) -> bool {
        self.sim_time_source.is_some()
    }

    /// phase-425 W3 — subscribe `topic` and install every sample as this
    /// image's ROS time. The registration behind
    /// [`NodeCtx::install_ros_time_source`](super::node::NodeCtx::install_ros_time_source)
    /// and behind the `use_sim_time` reconciliation, so both spell the QoS and
    /// the conversion exactly once.
    #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
    pub(crate) fn install_ros_time_source(
        &mut self,
        node_id: super::node_record::NodeId,
        topic: &str,
    ) -> Result<HandleId, NodeError> {
        self.register_subscription_buffered_on::<
            nros_rosgraph_msgs::msg::Clock,
            _,
            { crate::config::DEFAULT_RX_BUF_SIZE },
        >(
            node_id,
            topic,
            QoSProfile::clock_default(),
            |msg: &nros_rosgraph_msgs::msg::Clock| {
                // The `use_sim_time` gate is read HERE rather than by removing
                // the subscription, because there is no entity removal. An
                // image that never touches `use_sim_time` and installs the
                // source explicitly is active by default.
                if !crate::time_source::is_active() {
                    return;
                }
                if let Some(nanos) =
                    crate::time_source::override_nanos(msg.clock.sec, msg.clock.nanosec)
                {
                    nros_core::clock::Clock::set_ros_time_override(nanos);
                }
            },
            None, // no group — node default
            None, // the configured default RX buffer, unchanged
        )
    }
}
