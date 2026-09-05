//! nros C++ API — header-only C++14 library + Rust FFI staticlib.
//!
//! This crate provides `extern "C"` functions designed for the nros-cpp
//! C++ headers. Unlike `nros-c` (which erases types into opaque handles),
//! `nros-cpp` preserves type information through the FFI boundary — each
//! message/service/action type gets its own FFI function.
//!
//! # Architecture
//!
//! ```text
//! C++ (nros-cpp headers)  →  extern "C"  →  nros-cpp (Rust)  →  nros-node
//! ```
//!
//! The C++ side provides inline opaque storage for all entity handles
//! (publisher, subscription, service, guard condition, executor, action).
//! No heap allocation required — fully alloc-free.
//!
//! All serialization/deserialization happens on the runtime.
//!
//! Issue 0436 — every exported entry point that takes a user-supplied `void*`
//! executor handle validates it with `cpp_ctx_checked` (tag check) instead of
//! blind-casting to `*mut CppContext`. Construction (`nros_cpp_init` /
//! `_open_over_session`) and destruction (`nros_cpp_fini`) sites stamp or tear
//! down the tag and are necessarily unchecked.

#![no_std]
#![allow(non_camel_case_types)]
#![allow(dead_code)]

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "panic-halt")]
use panic_halt as _;

// issue 0619 — a lib TEST is a final artifact, so it must link a platform port
// like any real consumer; without one, `nros_platform_clock_ns` and friends are
// declared and never defined. The `posix-c-port` dev-dependency supplies them,
// but a dev-dep nothing REFERENCES is dropped by rustc before its build script's
// `-l` reaches the link line — the `-L …/nros-platform-cffi-*/out` was already
// on the command and the archive still was not. So the reference has to be
// explicit. Deliberately not a weak/no-op sink: issue 0420 was exactly that, a
// log facade that silently did nothing on threadx/nuttx.
#[cfg(test)]
use nros_platform_cffi as _;

// Opt-in RTOS heap-usage tracking (issue #6). A single shared `HeapStats`
// counter instruments whichever RTOS global allocator is active (exactly one
// platform feature is on at a time). `STATS` sees the Rust global allocator's
// footprint only — zenoh-pico's direct C-side z_malloc/pvPortMalloc traffic is
// not counted, so it under-reports true heap pressure.
//
// Phase 230 1b.3 / RFC-0034 D7 — the platform ABI exposes the TRUE *unified*
// heap figures (`nros_platform_heap_used_bytes` / `_total_bytes`), where the
// platform owns one kernel heap shared by the C side and the Rust
// `#[global_allocator]`. Design: keep `nros_heap_used_bytes()` /
// `nros_heap_peak_bytes()` as the Rust-footprint view (unchanged semantics, so
// callers tracking only the Rust allocator keep their meaning) and add
// `nros_heap_platform_used_bytes()` + `nros_heap_total_bytes()` that forward to
// the platform query for the unified figure. Both return `0` on ports that
// don't instrument their heap.
//
// phase-361 W8.c / issue 0594 — this crate defines NONE of it. The four
// `#[no_mangle]` names above are the same symbols `nros-c` exports, and
// `nros-c` is a hard dependency below, so enabling `alloc-stats` on both
// crates produced duplicate definitions of `nros_heap_used_bytes` and friends.
// That is the single-runtime rule already stated for `#[global_allocator]` /
// `#[panic_handler]` / `critical_section::set_impl!` — exactly ONE of each
// across the crate graph — applied to the heap counter too:
// `nros-cpp/alloc-stats` forwards to `nros-c/alloc-stats`.

// Phase 241.D3-rev (W12) × phase-248 — nros-cpp bundles `nros-c` as a HARD dependency,
// and nros-c (behind the platform vtable) owns the no_std `#[global_allocator]`
// (`platform_alloc`), the `#[panic_handler]`, AND the `critical_section::set_impl!`
// (`platform_critical_section`). The single-runtime umbrella permits exactly ONE of each
// across the whole crate graph, so nros-cpp defines NONE of them here — nros-c's serve
// the entire `libnros_cpp.a` (all route through the same `nros_platform_*` vtable). This
// crate's `platform-*` features forward to `nros-c/platform-*` (which enable nros-c's
// `global-allocator` / `critical-section`) and `panic-halt` → `nros-c/panic-halt`.

use core::ffi::{c_char, c_int, c_void};

// Phase 241.D3-rev — force-link the selected backend into `libnros_cpp.a` (the C++
// umbrella's staticlib root) + auto-register it before `main`. nros-c's twin anchor
// is DCE'd as a dependency, so the root carries its own. See `rmw_backend`.
#[cfg(any(feature = "rmw-zenoh-cffi", feature = "rmw-xrce-cffi"))]
mod rmw_backend;

// Phase 241.D3-rev — pull nros-c's FULL `#[no_mangle]` C surface into libnros_cpp.a.
// nros-cpp bundles nros-c as an rlib and links only libnros_cpp.a, so rustc DCEs any
// C entry point the C++ FFI itself never references (e.g. nros_parameter_server_fini) —
// yet a C++ binary may call it via the C ABI. nros-c's own `#[used]` anchor is DCE'd
// as a dependency; referencing it from THIS staticlib root keeps it + the entry
// points it names.
#[used]
static _KEEP_C_SURFACE: &[unsafe extern "C" fn()] = &nros_c::c_surface_anchor::C_SURFACE_ANCHOR;

// Issue 0436 — same DCE class, for the multi-RMW bridge ABI. `nros-bridge` is an
// rlib dep whose `nros_init_multi` / `nros_pubsub_bridge_*` exports nothing in
// this crate calls, so rustc drops them from `libnros_cpp.a` and a C++ binary that
// includes `<nros/bridge.hpp>` fails to link. Reference them from the staticlib
// ROOT to keep them (the FORCE_LINK pattern `rmw_backend` uses for backends).
// Must be a RUST path into the crate (not an `extern "C"` redeclaration): the
// dependency edge is what pulls the rlib's objects into this staticlib. Declaring
// the symbols `extern "C"` only creates UNDEFINED references — verified: the
// archive then carried `U nros_init_multi` and nothing to satisfy it.
#[cfg(feature = "bridge")]
#[used]
static _KEEP_BRIDGE_SURFACE: &[unsafe extern "C" fn(*mut core::ffi::c_void)] =
    &nros_bridge::cffi_surface_anchor::CFFI_SURFACE_ANCHOR;

// Phase 241 W11 (Option D) — `pub mod cpp_surface_anchor { … CPP_SURFACE_ANCHOR }`,
// generated by build.rs (this crate's own ungated `nros_cpp_*` no_mangle surface).
include!(concat!(env!("OUT_DIR"), "/cpp_surface_anchor.rs"));

// Issue 0360 — archive half of the variant stamp; see nros-c/src/lib.rs.
include!(concat!(env!("OUT_DIR"), "/variant_symbol.rs"));

// W11 backend anchor — the selected cffi backend's auto-register fn, so a downstream
// staticlib root pulls its register closure into the archive. Empty with no cffi backend.
#[cfg(any(feature = "rmw-zenoh-cffi", feature = "rmw-xrce-cffi"))]
const _BACKEND_ANCHOR: &[unsafe extern "C" fn()] = &[rmw_backend::auto_register];
#[cfg(all(
    feature = "rmw-cffi",
    not(any(feature = "rmw-zenoh-cffi", feature = "rmw-xrce-cffi"))
))]
const _BACKEND_ANCHOR: &[unsafe extern "C" fn()] = &[];

/// Phase 241 W11 (Option D) — combined force-link anchor for a downstream staticlib root
/// (the per-entry `<entry>_runtime` crate). When nros-cpp is bundled as a dependency
/// rlib, its own `#[used]` anchors are DCE'd before the runtime staticlib is emitted; the
/// runtime root references THIS with its own `#[used]` to re-pull the full ABI surface —
/// nros-c's C API + nros-cpp's C++ FFI + the selected backend's register closure.
#[cfg(feature = "rmw-cffi")]
pub static FORCE_LINK_ANCHOR: &[&[unsafe extern "C" fn()]] = &[
    &nros_c::c_surface_anchor::C_SURFACE_ANCHOR,
    &cpp_surface_anchor::CPP_SURFACE_ANCHOR,
    _BACKEND_ANCHOR,
];

// W11 — re-export the backend auto-register so the runtime root can install its OWN
// `.init_array` ctor (this crate's ctor, in a dep rlib, is DCE'd). Pull `register()`
// before `main` on hosted targets that honor `.init_array`.
#[cfg(any(feature = "rmw-zenoh-cffi", feature = "rmw-xrce-cffi"))]
pub use rmw_backend::auto_register as nros_cpp_auto_register_backend;

// ── Core entity modules (alloc-free — caller provides inline storage) ──
// phase-308 — the three executor-side recording hooks the metadata RMW backend
// cannot observe (node identity, timers, guard conditions). No-ops unless
// `metadata-mode` is on.
#[cfg(feature = "rmw-cffi")]
mod guard_condition;
#[cfg(feature = "rmw-cffi")]
mod metadata_hooks;
#[cfg(feature = "rmw-cffi")]
mod publisher;
#[cfg(feature = "rmw-cffi")]
mod service;
#[cfg(feature = "rmw-cffi")]
mod subscription;
#[cfg(feature = "rmw-cffi")]
mod timer;

// ── Action module (alloc-free — caller provides inline storage) ──
#[cfg(feature = "rmw-cffi")]
mod action;

// Phase 115.D — runtime-pluggable custom transport. Always-on (no
// rmw-* gate) because the registration is platform-side, not RMW-side.
mod transport;

// Phase 269 (W0) — executor-shim: lifecycle + parameter FFI over the CppContext handle.
mod lifecycle_shim;
mod params_shim;

// Issue 0790 — ordered shutdown hooks over the CppContext handle. Not
// `rmw-cffi`-gated at the module level: the typedefs and the invalid-handle
// constant are part of the header whether or not a backend is linked, and the
// four entry points carry the gate individually (they need `ctx.executor`).
mod shutdown;

// ── Tick-time client dispatch (Phase 212.M-F.4.c) ──
//
// Mirror of the Rust substrate's `TickCtx::call_raw` /
// `TickCtx::send_goal_raw` seams added in Phase 212.M-F.4 (`d15565efe`).
// Always-on (no rmw-cffi gate) because the stub error path is independent
// of any RMW backend — the symbols exist + return `NROS_CPP_RET_ERROR`
// until the codegen-side `GenClientDispatch` impl lands (M-F.4.a).
mod tick_ctx;

// ============================================================================
// Error codes (mirror nros-c for consistency)
// ============================================================================

/// Return type for nros C++ FFI functions.
pub type nros_cpp_ret_t = c_int;

// Issue #229 — ONE return-code numbering across all three spaces: these
// constants are value-identical to nros-c's `NROS_RET_*` (and to C++
// `nros::ErrorCode`); `Result(any C code)` is correct by identity. The
// static_assert pin tables in nros-cpp's result.hpp / parameter.hpp fail
// the build on any re-divergence. (Pre-#229 this space numbered -4..-8
// differently from nros_ret_t — a raw -5 read as "Full" when the C side
// meant ALREADY_EXISTS.)
/// Success.
pub const NROS_CPP_RET_OK: nros_cpp_ret_t = 0;
/// Generic error.
pub const NROS_CPP_RET_ERROR: nros_cpp_ret_t = -1;
/// Timeout.
pub const NROS_CPP_RET_TIMEOUT: nros_cpp_ret_t = -2;
/// Invalid argument.
pub const NROS_CPP_RET_INVALID_ARGUMENT: nros_cpp_ret_t = -3;
/// Entity not found (topic, parameter, service…).
pub const NROS_CPP_RET_NOT_FOUND: nros_cpp_ret_t = -4;
/// Already exists (duplicate declare/register).
pub const NROS_CPP_RET_ALREADY_EXISTS: nros_cpp_ret_t = -5;
/// Resource limit reached.
pub const NROS_CPP_RET_FULL: nros_cpp_ret_t = -6;
/// Not initialized.
pub const NROS_CPP_RET_NOT_INIT: nros_cpp_ret_t = -7;
/// Operation invalid in the current state (bad call sequence).
pub const NROS_CPP_RET_BAD_SEQUENCE: nros_cpp_ret_t = -8;
/// Service request/reply failed.
pub const NROS_CPP_RET_SERVICE_FAILED: nros_cpp_ret_t = -9;
/// Publish failed.
pub const NROS_CPP_RET_PUBLISH_FAILED: nros_cpp_ret_t = -10;
/// Subscription create/take failed.
pub const NROS_CPP_RET_SUBSCRIPTION_FAILED: nros_cpp_ret_t = -11;
/// Operation not allowed for this entity/backend.
pub const NROS_CPP_RET_NOT_ALLOWED: nros_cpp_ret_t = -12;
/// Request was rejected — the peer considered it and declined.
/// A goal rejected by an action server, or a QoS/ABI incompatibility.
/// Distinct from `Error`, which means the request never got that far
/// (issue 0868).
pub const NROS_CPP_RET_REJECTED: nros_cpp_ret_t = -13;
/// Try again — operation not ready yet.
pub const NROS_CPP_RET_TRY_AGAIN: nros_cpp_ret_t = -14;
/// Reentrant call detected — executor is already spinning.
pub const NROS_CPP_RET_REENTRANT: nros_cpp_ret_t = -15;
/// Phase 108 — operation not implemented by the active backend.
pub const NROS_CPP_RET_UNSUPPORTED: nros_cpp_ret_t = -16;
/// Transport / connection error (C++-space extension; nros_ret_t stops at -16).
pub const NROS_CPP_RET_TRANSPORT_ERROR: nros_cpp_ret_t = -100;

// ============================================================================
// Inline opaque storage sizes (in u64 units)
// ============================================================================
//
// These constants define the inline storage for internal C++ FFI wrapper
// structs (CppPublisher, CppSubscription, etc.). The C++ side allocates
// buffers of this size; the runtime writes directly into them.
// Compile-time assertions in each module verify the storage is large enough.

// Opaque storage sizes computed from size_of at compile time — always exact.
// When no RMW backend is enabled (workspace-level check), placeholder values
// are used. The placeholders are never used at runtime.

const fn u64s_for<T>() -> usize {
    core::mem::size_of::<T>().div_ceil(8)
}

// With RMW backend: exact sizes from actual types.
// Phase 87.6: `CppPublisher` removed — the FFI stores an `RmwPublisher`
// handle directly, sized via `NROS_PUBLISHER_SIZE` from the `nros` probe
// (see packages/api/nros-cpp/build.rs).
// Phase 87.6: `CppSubscription` removed — the FFI stores an
// `RmwSubscriber` handle directly, sized via `NROS_SUBSCRIBER_SIZE` from
// the `nros` probe.
// Phase 87.6: `CppServiceServer` and `CppServiceClient` removed — the FFI
// stores `RmwServiceServer` / `RmwServiceClient` handles directly, sized
// via `NROS_SERVICE_SERVER_SIZE` / `NROS_SERVICE_CLIENT_SIZE` from the
// `nros` probe.
// Phase 87.11: `CPP_ACTION_SERVER_OPAQUE_U64S` and
// `CPP_ACTION_CLIENT_OPAQUE_U64S` removed. ActionServer/ActionClient
// storage sizes are now sourced from `nros::sizes::CppActionServerLayout`
// / `CppActionClientLayout` via the probe; see action.rs for the
// layout-mirror equality asserts.

// Phase 87.6: `CPP_GUARD_HANDLE_OPAQUE_U64S` removed — the C++
// `nros::GuardCondition` class sizes its `storage_` from
// `NROS_GUARD_CONDITION_SIZE` (`size_of::<GuardCondition>()`
// probed from the nros rlib).

// ============================================================================
// QoS types (passed from C++ to Rust by value)
// ============================================================================

/// QoS reliability policy.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_cpp_qos_reliability_t {
    NROS_CPP_QOS_RELIABLE = 0,
    NROS_CPP_QOS_BEST_EFFORT = 1,
}

/// QoS durability policy.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_cpp_qos_durability_t {
    NROS_CPP_QOS_VOLATILE = 0,
    NROS_CPP_QOS_TRANSIENT_LOCAL = 1,
}

/// QoS history policy.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_cpp_qos_history_t {
    NROS_CPP_QOS_KEEP_LAST = 0,
    NROS_CPP_QOS_KEEP_ALL = 1,
}

/// QoS liveliness policy. Phase 108.B.7 — matches DDS `LIVELINESS`.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_cpp_qos_liveliness_t {
    NROS_CPP_QOS_LIVELINESS_NONE = 0,
    NROS_CPP_QOS_LIVELINESS_AUTOMATIC = 1,
    NROS_CPP_QOS_LIVELINESS_MANUAL_BY_TOPIC = 2,
    NROS_CPP_QOS_LIVELINESS_MANUAL_BY_NODE = 3,
}

/// QoS settings (passed by value from C++).
///
/// Phase 108.B.7 — full DDS-shaped QoS surface. The four core fields
/// (`reliability`, `durability`, `history`, `depth`) plus extended
/// policies (`liveliness_kind`, `deadline_ms`, `lifespan_ms`,
/// `liveliness_lease_ms`, `avoid_ros_namespace_conventions`) match
/// `nros_qos_t` (C API) and `QoSProfile` (Rust API).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct nros_cpp_qos_t {
    pub reliability: nros_cpp_qos_reliability_t,
    pub durability: nros_cpp_qos_durability_t,
    pub history: nros_cpp_qos_history_t,
    pub liveliness_kind: nros_cpp_qos_liveliness_t,
    pub depth: c_int,
    /// Subscriber max-inter-arrival / publisher offered-rate, ms.
    /// `0` = infinite (no deadline check).
    pub deadline_ms: u32,
    /// Sample expiry, ms. `0` = infinite.
    pub lifespan_ms: u32,
    /// Liveliness lease, ms. `0` = infinite.
    pub liveliness_lease_ms: u32,
    /// If non-zero, topic-name encoding skips the `/rt/` ROS prefix.
    pub avoid_ros_namespace_conventions: u8,
    /// Phase 282 (#145) — publisher-side "express" hint: if non-zero, this
    /// publisher's samples bypass transport tx batching. A transport hint,
    /// not a DDS policy; ignored on subscriptions.
    pub tx_express: u8,
}

impl nros_cpp_qos_t {
    pub(crate) fn to_qos_settings(self) -> nros_rmw::QoSProfile {
        use nros_rmw::{
            QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy,
        };

        nros_rmw::QoSProfile {
            reliability: match self.reliability {
                nros_cpp_qos_reliability_t::NROS_CPP_QOS_RELIABLE => QoSReliabilityPolicy::Reliable,
                nros_cpp_qos_reliability_t::NROS_CPP_QOS_BEST_EFFORT => {
                    QoSReliabilityPolicy::BestEffort
                }
            },
            durability: match self.durability {
                nros_cpp_qos_durability_t::NROS_CPP_QOS_VOLATILE => QoSDurabilityPolicy::Volatile,
                nros_cpp_qos_durability_t::NROS_CPP_QOS_TRANSIENT_LOCAL => {
                    QoSDurabilityPolicy::TransientLocal
                }
            },
            history: match self.history {
                nros_cpp_qos_history_t::NROS_CPP_QOS_KEEP_LAST => QoSHistoryPolicy::KeepLast,
                nros_cpp_qos_history_t::NROS_CPP_QOS_KEEP_ALL => QoSHistoryPolicy::KeepAll,
            },
            liveliness_kind: match self.liveliness_kind {
                nros_cpp_qos_liveliness_t::NROS_CPP_QOS_LIVELINESS_NONE => {
                    QoSLivelinessPolicy::None
                }
                nros_cpp_qos_liveliness_t::NROS_CPP_QOS_LIVELINESS_AUTOMATIC => {
                    QoSLivelinessPolicy::Automatic
                }
                nros_cpp_qos_liveliness_t::NROS_CPP_QOS_LIVELINESS_MANUAL_BY_TOPIC => {
                    QoSLivelinessPolicy::ManualByTopic
                }
                nros_cpp_qos_liveliness_t::NROS_CPP_QOS_LIVELINESS_MANUAL_BY_NODE => {
                    QoSLivelinessPolicy::ManualByNode
                }
            },
            depth: self.depth as u32,
            deadline_ms: self.deadline_ms,
            lifespan_ms: self.lifespan_ms,
            liveliness_lease_ms: self.liveliness_lease_ms,
            avoid_ros_namespace_conventions: self.avoid_ros_namespace_conventions != 0,
            tx_express: self.tx_express != 0,
        }
    }
}

// ============================================================================
// Build-time configuration
// ============================================================================

mod executor_config {
    include!(concat!(env!("OUT_DIR"), "/nros_cpp_ffi_config.rs"));
}
pub use executor_config::CPP_EXECUTOR_OPAQUE_U64S;

// Compile-time asserts that the auto-generated C-side STORAGE macros
// are large enough for their Rust counterparts. If a Rust type grows
// past the estimate emitted by build.rs, compilation fails with a
// clear error instead of silently overflowing caller-provided storage.
#[cfg(feature = "rmw-cffi")]
const _: () = {
    // Phase 87.6: `CppPublisher`, `CppSubscription`, `CppServiceServer`,
    // and `CppServiceClient` assertions removed — all four now use
    // thin-wrapper storage sized from the Rust SSoT (`NROS_*_SIZE`
    // probes in the generated header).
    // Phase 87.6: `GuardCondition` assertion removed — storage
    // sized from `NROS_GUARD_CONDITION_SIZE` (probed).
};

// ============================================================================
// Executor handle (alloc-free — caller provides inline storage)
// ============================================================================

/// The concrete nros-node executor type used by the C++ FFI.
///
/// phase-271 — the executor now borrows its per-entry storage
/// (`Executor<'static>`); the C++ API keeps it heap-free by carving that backing
/// from the [`backing`](CppContext::backing) tail of the SAME pinned `CppContext`
/// buffer (see there).
#[cfg(feature = "rmw-cffi")]
pub(crate) type CppExecutor = nros_node::Executor<'static>;

/// `u64` words of per-entry backing the inline executor carves from its
/// `CppContext` buffer (default sizing).
#[cfg(feature = "rmw-cffi")]
pub(crate) const CPP_EXECUTOR_BACKING_U64S: usize = nros_node::ExecutorSizing::DEFAULT.u64_len();

/// Context wrapping the executor and the domain ID.
///
/// The executor doesn't store domain_id itself — it's consumed during
/// session open. We keep it here so publisher/subscription creation
/// can pass the correct value to `TopicInfo::with_domain()`.
///
/// phase-271 — `backing` is the executor's per-entry storage, carved in place by
/// `nros_cpp_init`. `executor` borrows it (a self-borrow within this struct);
/// sound because the buffer is pinned (caller-owned, initialised in place, only
/// reached through a stable `*mut CppContext`, never moved). Keep `backing` last
/// so `executor` stays at offset 0 for the `*mut c_void as *mut CppContext` casts.
#[cfg(feature = "rmw-cffi")]
pub(crate) struct CppContext {
    /// Issue 0436 — handle type tag, FIRST so it can be read before the struct is
    /// trusted. `nros_cpp_init`/`_init_multi` stamp it; every entry point that
    /// takes a `void*` executor handle checks it.
    ///
    /// Two different structs used to travel as `void*`: this one and
    /// `nros-bridge`'s `ExecutorBox` (from `nros_init_multi`). Both begin with the
    /// SAME `Executor<'static>`, so a mixed-up handle READ correctly and then wrote
    /// this struct's later fields over the other's `Vec` — memory corruption, seen
    /// as PX4 dumping core. A tag turns that into a clean error.
    pub(crate) tag: u64,
    pub(crate) executor: CppExecutor,
    pub(crate) domain_id: u32,
    /// Reentrancy guard — `true` while a spin is dispatching callbacks.
    ///
    /// The C twin of this flag is `nros_executor_t.in_dispatch` (nros-c
    /// `executor.rs`), checked by `nros_client_call` / `nros_action_send_goal`
    /// so a blocking helper invoked FROM a callback returns
    /// `NROS_RET_REENTRANT` instead of re-entering the executor. nros-cpp had
    /// no such flag (issue 0290): every blocking C++ helper reached
    /// `&mut *(handle as *mut CppContext)` unconditionally, so calling
    /// `Client::call()` inside a callback aliased `&mut Executor` — silently,
    /// with no error returned.
    ///
    /// Guarding here rather than in each helper covers the whole family at
    /// once: `Future::wait` / `Client::call` (via `nros_cpp_spin_once`) and
    /// the action helpers, which spin `ctx.executor` directly.
    pub(crate) in_dispatch: bool,
    pub(crate) backing: [core::mem::MaybeUninit<u64>; CPP_EXECUTOR_BACKING_U64S],
}

/// RAII guard: marks the context as dispatching for the duration of a spin and
/// clears the flag on drop, so an early return can't leave it stuck.
///
/// Borrows only the FLAG, not the whole context — callers split-borrow the
/// executor alongside it (`let CppContext { executor, in_dispatch, .. } = ctx`).
/// That keeps the guard independently testable: it needs a `&mut bool`, not a
/// live `Executor`.
// NOT gated on `rmw-cffi`: this is pure flag logic with no FFI dependency, and
// the `rmw-cffi` lib-test target does not link on the host (it needs a platform
// impl for `nros_platform_sleep_ms`). Keeping the guard feature-independent is
// what lets `dispatch_guard_tests` actually RUN in the default test lane
// instead of being silently skipped.
#[allow(dead_code)]
pub(crate) struct DispatchGuard<'a> {
    flag: &'a mut bool,
}

impl<'a> DispatchGuard<'a> {
    /// `None` when a spin is already in progress — the caller must NOT spin.
    #[allow(dead_code)]
    pub(crate) fn enter(flag: &'a mut bool) -> Option<Self> {
        if *flag {
            return None;
        }
        *flag = true;
        Some(Self { flag })
    }
}

impl Drop for DispatchGuard<'_> {
    fn drop(&mut self) {
        *self.flag = false;
    }
}

/// Issue 0436 — marks a buffer as an nros-cpp executor context. Value is
/// arbitrary but must not be a plausible pointer or small integer.
#[cfg(feature = "rmw-cffi")]
pub(crate) const CPP_CONTEXT_TAG: u64 = 0x6E52_4F53_4350_5001; // "nROSCP\x50\x01"

/// Issue 0436 — validate a caller-supplied executor handle before treating it as a
/// [`CppContext`]. Returns `None` for null or for a buffer this crate did not
/// stamp (e.g. an `nros_init_multi` bridge handle).
///
/// # Safety
/// `handle` must either be null or point to a readable 8-byte-aligned allocation.
#[cfg(feature = "rmw-cffi")]
pub(crate) unsafe fn cpp_ctx_checked<'a>(handle: *mut c_void) -> Option<&'a mut CppContext> {
    if handle.is_null() {
        return None;
    }
    let ctx = handle as *mut CppContext;
    // Read ONLY the tag until it is proven to be ours.
    if unsafe { core::ptr::read(core::ptr::addr_of!((*ctx).tag)) } != CPP_CONTEXT_TAG {
        return None;
    }
    Some(unsafe { &mut *ctx })
}

// Compile-time assertion: inline storage must fit CppContext (executor + domain
// + carved backing).
#[cfg(feature = "rmw-cffi")]
const _: () = assert!(
    core::mem::size_of::<CppContext>() <= CPP_EXECUTOR_OPAQUE_U64S * core::mem::size_of::<u64>(),
    "CPP_EXECUTOR_OPAQUE_U64S too small for CppContext — this is a MEASUREMENT, not a budget. The stated size is \
     `size_of` taken while building the `nros` facade in the sizes PROBE, and \
     this compares it against `size_of` in the unit that LINKS. They differ only \
     when the probe built under a different feature set than this crate resolves \
     — issue 0665: `std` here forwards `nros/env`, the probe forwarded only the \
     shared name `std`, and one fat pointer of difference made the number 16 \
     bytes short. Check the forwarded set \
     (`nros_sizes_build::resolved_features_for`) before touching any knob. \
     NROS_EXECUTOR_ARENA_SIZE / NROS_EXECUTOR_MAX_CBS move BOTH sides equally \
     and cannot close a feature-set gap."
);

// ============================================================================
// Init / Fini
// ============================================================================

/// Initialize an nros executor session.
///
/// Opens a middleware connection and writes the executor context directly
/// into caller-provided storage (no heap allocation).
///
/// # Parameters
/// * `locator` — Middleware locator (e.g., `"tcp/127.0.0.1:7447"`), or NULL for default.
/// * `domain_id` — ROS domain ID (0–232).
/// * `node_name` — Node name (null-terminated string). Must not be NULL.
/// * `namespace` — Node namespace (null-terminated string), or NULL for `"/"`.
/// * `storage` — Pointer to caller-provided storage (at least `CPP_EXECUTOR_OPAQUE_U64S * 8` bytes,
///   aligned to 8 bytes). The executor is written directly into this buffer.
///
/// # Safety
/// * `node_name` must be a valid null-terminated string.
/// * `locator` and `namespace` must be valid null-terminated strings or NULL.
/// * `storage` must be a valid pointer to appropriately sized and aligned storage.
///
/// # Returns
/// `NROS_CPP_RET_OK` on success, error code otherwise.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_init(
    locator: *const c_char,
    domain_id: u8,
    node_name: *const c_char,
    namespace: *const c_char,
    storage: *mut c_void,
) -> nros_cpp_ret_t {
    // Issue 1050 defect (3) — NULL selector, i.e. "this image names no
    // backend"; the registry's single entry resolves, and an ambiguous registry
    // is now refused instead of taking whichever backend registered first.
    unsafe {
        nros_cpp_init_rmw(
            core::ptr::null(),
            locator,
            domain_id,
            node_name,
            namespace,
            storage,
        )
    }
}

/// Issue 1050 defect (3) — [`nros_cpp_init`] with an explicit RMW selector.
///
/// `rmw` is the BAKED rung of precedence model A (RFC-0045): a hosted
/// `$NROS_RMW` still wins, and a NULL selector leaves the choice to the
/// registry, which resolves only when exactly one backend is registered.
///
/// It exists because `rmw` was the ONE field of `ExecutorConfig` with no baked
/// rung — resolvable from the process environment and from nowhere else. An
/// image that knows which backend it wants (a PX4 module declaring
/// `BACKENDS uorb`, any C++ entry built against one RMW, any RTOS target with
/// no environment to read) could not say so, and got whichever backend's
/// `.init_array` ctor ran first. On hosted POSIX that is the archive's
/// backend, not the image's.
///
/// Additive rather than a sixth parameter on `nros_cpp_init`: that symbol is
/// called by every generated C++ entry and by user code, and widening it is an
/// ABI break for all of them. `<nros/node.hpp>`'s `nros::init` reaches this
/// through the `NROS_ENTRY_RMW` bake macro.
///
/// # Safety
/// As [`nros_cpp_init`], plus: `rmw` must be a valid NUL-terminated string or
/// NULL.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_init_rmw(
    rmw: *const c_char,
    locator: *const c_char,
    domain_id: u8,
    node_name: *const c_char,
    namespace: *const c_char,
    storage: *mut c_void,
) -> nros_cpp_ret_t {
    // phase-412 -- stamp the self-report BEFORE anything can reject an
    // argument. This is the first nano-ros code an image runs, so a record that
    // is still zero after this point means the image never got here at all.
    //
    // Version 1 stamped it inside the executor constructor instead. On the
    // first board run that made "never entered nano-ros" and "entered and died
    // before the executor" the same observation -- both read magic 0 -- and
    // telling them apart took a walk through the disassembly to prove the call
    // chain existed. That is the manual work the record exists to remove.
    nros_node::boot_report::init();

    if node_name.is_null() || storage.is_null() {
        nros_node::boot_report::note_cpp_init_ret(NROS_CPP_RET_INVALID_ARGUMENT);
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    unsafe extern "C" {
        fn nros_app_register_backends();
    }
    unsafe {
        nros_app_register_backends();
    }
    // Phase 241.D3-rev — the selected backend is auto-registered before `main` by
    // the `nros-c` umbrella's `rmw_backend` `.init_array` ctor (bundled into
    // `libnros_cpp.a`). `nros_app_register_backends()` above stays as the weak
    // board-override hook; the explicit per-backend `register()` calls are gone
    // (idempotent re-register, and they referenced backend crates nros-cpp no
    // longer deps directly).
    let node_name_str = match unsafe { cstr_to_str(node_name) } {
        Some(s) => s,
        None => {
            nros_node::boot_report::note_cpp_init_ret(NROS_CPP_RET_INVALID_ARGUMENT);
            return NROS_CPP_RET_INVALID_ARGUMENT;
        }
    };

    let ns_str = if namespace.is_null() {
        "/"
    } else {
        match unsafe { cstr_to_str(namespace) } {
            Some(s) => s,
            None => {
                nros_node::boot_report::note_cpp_init_ret(NROS_CPP_RET_INVALID_ARGUMENT);
                return NROS_CPP_RET_INVALID_ARGUMENT;
            }
        }
    };

    let locator_str = if locator.is_null() {
        None
    } else {
        match unsafe { cstr_to_str(locator) } {
            Some(s) => Some(s),
            None => {
                nros_node::boot_report::note_cpp_init_ret(NROS_CPP_RET_INVALID_ARGUMENT);
                return NROS_CPP_RET_INVALID_ARGUMENT;
            }
        }
    };

    // Issue 1050 defect (3) — an EMPTY selector is "unset", not a backend named
    // "". The bake macro expands to a string literal, and a cmake variable that
    // did not resolve produces `""` rather than nothing at all; treating that
    // as a name would turn a missing bake into `Unknown backend` instead of the
    // registry default.
    let rmw_str = if rmw.is_null() {
        None
    } else {
        match unsafe { cstr_to_str(rmw) } {
            Some("") => None,
            Some(s) => Some(s),
            None => return NROS_CPP_RET_INVALID_ARGUMENT,
        }
    };

    // RFC-0045 / issue #206 — route through the ONE boot-config resolver
    // (precedence model A: hosted env > baked overlay > compiled default).
    // The header's arg/NROS_ENTRY_* chain arrives as the BAKED rung here;
    // the env rung (NROS_LOCATOR/ROS_DOMAIN_ID/NROS_NODE_NAME, hosted only)
    // and the domain validation (malformed / > DOMAIN_ID_MAX = error, never
    // silent 0) now live in nros-node, identical for Rust, C, and C++.
    // domain_id 0 = the unset sentinel (ROS convention; an explicit
    // "domain 0" and "unset" are indistinguishable at this ABI edge).
    let baked = nros_node::BootConfig {
        node_name: Some(node_name_str),
        locator: locator_str,
        // Issue #227 — 255 (NROS_DOMAIN_ID_EXPLICIT_ZERO) = explicit domain 0;
        // 0 stays the unset sentinel; 233..=254 flow to the resolver's range
        // check and fail loudly.
        domain_id: nros_node::baked_domain_from_c_abi(domain_id),
        namespace: Some(ns_str),
        rmw: rmw_str,
    };
    // issue 0687 — see the same call in `nros-c`: the env rung is the hosted
    // edge's to supply, so the capability cfg sits at the call site.
    let config = match resolve_boot(baked) {
        Ok(cfg) => cfg,
        Err(_) => {
            nros_node::boot_report::note_cpp_init_ret(NROS_CPP_RET_INVALID_ARGUMENT);
            return NROS_CPP_RET_INVALID_ARGUMENT;
        }
    };
    // Everything above this line is argument validation; everything below is
    // the executor. A record stopping here says the inputs were rejected, and
    // `cpp_init_ret` says by which check.
    nros_node::boot_report::checkpoint(nros_node::boot_report::Stage::BootConfigResolved);
    let domain_id = config.domain_id as u8;

    // phase-271 — construct in place: carve the executor's per-entry backing from
    // the tail of the caller's `CppContext` buffer, then open the executor over
    // it and write it (offset 0) + domain_id. Heap-free; the executor's slices
    // point into the final (pinned) buffer location, so nothing is moved after.
    let ctx_ptr = storage as *mut CppContext;
    let backing: &'static mut [core::mem::MaybeUninit<u64>] = unsafe {
        core::slice::from_raw_parts_mut(
            core::ptr::addr_of_mut!((*ctx_ptr).backing) as *mut core::mem::MaybeUninit<u64>,
            CPP_EXECUTOR_BACKING_U64S,
        )
    };
    // SAFETY: `backing` is sized/aligned per `ExecutorSizing::DEFAULT` and lives
    // for the program (caller owns the buffer); `open_in`'s contract is met.
    match unsafe { CppExecutor::open_in(&config, backing, nros_node::ExecutorSizing::DEFAULT) } {
        Ok(executor) => {
            // Write directly into caller-provided storage — no heap allocation.
            unsafe {
                core::ptr::write(core::ptr::addr_of_mut!((*ctx_ptr).executor), executor);
                core::ptr::write(
                    core::ptr::addr_of_mut!((*ctx_ptr).domain_id),
                    domain_id as u32,
                );
                core::ptr::write(core::ptr::addr_of_mut!((*ctx_ptr).in_dispatch), false);
                // Stamp LAST: the tag means "fully initialised", so a half-built
                // buffer never validates (issue 0436).
                core::ptr::write(core::ptr::addr_of_mut!((*ctx_ptr).tag), CPP_CONTEXT_TAG);
            }
            NROS_CPP_RET_OK
        }
        // Phase 155.C — surface the inner `NodeError` variant as a
        // specific `NROS_CPP_RET_*` code instead of collapsing every
        // backend failure to TRANSPORT_ERROR. Mirrors the C-side
        // `transport_error_to_ret` mapping from Phase 155.B so the
        // next `nros::init -> -X` log line in the FreeRTOS / RV64
        // C++ tests identifies which precondition the backend
        // rejected.
        Err(e) => {
            let ret = node_error_to_cpp_ret(e);
            nros_node::boot_report::note_cpp_init_ret(ret);
            ret
        }
    }
}

/// Issue 0589 — the ONE way this crate emits a diagnostic.
///
/// `std::eprintln!` is FATAL on Zephyr `native_sim`. Rust std stdio goes through
/// the POSIX device-io fdtable, and fds 0/1/2 all carry
/// `stdinout_fd_op_vtable`, whose write method under
/// `CONFIG_BOARD_NATIVE_POSIX` is
///
/// ```c
/// return zvfs_write(1, buffer, count);   /* called FROM zvfs_write(1, …) */
/// ```
///
/// — unbounded self-recursion. `k_mutex` is recursive so it never deadlocks; it
/// exhausts the stack and SIGSEGVs the image (observed `lock_count = 104756`).
/// C/C++ `printf` misses this entirely because picolibc uses the console hook,
/// which is why it stayed latent until a Rust diagnostic was added to an error
/// path that actually ran (issue 0557).
///
/// So: `eprintln!` where it is safe, and the `nros_log` dispatcher on Zephyr —
/// which reaches the console when a sink is wired and is silently dropped when
/// one is not. Never fatal either way. A diagnostic that can kill the image it
/// is diagnosing is worse than no diagnostic.
// phase-359 W10 — ONE arm, through `nros_log`.
//
// There were three: `std::eprintln!` on hosted, `nros_log` on Zephyr, and — on
// no_std everywhere else — a no-op that discarded the message, because
// "no-std, non-Zephyr: nothing to write to". There is something to write to.
// `nros_log`'s default sink forwards to `nros_platform_log_write`, which the
// POSIX, FreeRTOS, ThreadX, ESP-IDF and Zephyr ports all implement, and which
// `nros-c` installs for every C/C++ image. So a FreeRTOS or ThreadX C++ image
// was throwing away exactly the diagnostics ("tier X setup FAILED", "spin_once
// returned rc=") that explain why it is not working, on the flavours where
// they are hardest to obtain by other means.
//
// Hosted output does not move: the POSIX port's `log_write` writes to stderr,
// the same stream `eprintln!` used. It gains an `[ERROR] nros_cpp:` prefix,
// which no test matches against — nothing greps these lines today, and the
// harness matchers are substring checks rather than line anchors.
//
// It also retires this file's std-stdio allowlist marker (the one
// `check-no-std-stdio` reads): the reason std stdio appeared in a no_std crate
// was this macro, and it no longer appears. The marker is not written out here
// on purpose — spelling it in prose is how a gate that greps for it gets told
// a site is exempt when none is.
#[allow(unused_macros)]
macro_rules! cpp_diag {
    ($($arg:tt)+) => {{
        let logger: &'static nros_log::Logger = nros_log::get_logger("nros_cpp");
        nros_log::nros_error!(logger, $($arg)+);
    }};
}

#[allow(unused_imports)]
pub(crate) use cpp_diag;

/// issue 0586 — map `TransportError` to the closest `NROS_CPP_RET_*` code.
///
/// The sibling of [`node_error_to_cpp_ret`], and the reason that one no longer
/// carries an inline `match` with a `_` arm. Ten call sites across
/// `publisher.rs` / `subscription.rs` / `service.rs` used to write
/// `Err(_) => NROS_CPP_RET_TRANSPORT_ERROR`, so a C++ caller was told "transport
/// error" for an unsupported operation, a bad argument, an incompatible QoS or a
/// failed allocation. `-100` is documented as the catch-all for UNMAPPED
/// variants; it should not be the answer for variants that map perfectly well.
///
/// NO wildcard, deliberately — see [`node_error_to_cpp_ret`]. Adding a
/// `TransportError` variant fails to compile here until someone decides what the
/// C++ caller should see.
///
/// `TRANSPORT_ERROR` is kept for the cases that genuinely ARE the transport:
/// the connection, the peer, the wire, and a backend-specific string this layer
/// cannot interpret.
#[cfg(feature = "rmw-cffi")]
pub(crate) fn transport_error_to_cpp_ret(err: nros_rmw::TransportError) -> nros_cpp_ret_t {
    use nros_rmw::TransportError as T;
    match err {
        // Genuinely transport.
        T::ConnectionFailed | T::Disconnected | T::PublishFailed => NROS_CPP_RET_TRANSPORT_ERROR,
        T::KeepaliveFailed | T::PollFailed | T::JoinFailed => NROS_CPP_RET_TRANSPORT_ERROR,
        T::IncompatibleAbi | T::Backend(_) => NROS_CPP_RET_TRANSPORT_ERROR,
        // NOT `#[cfg(feature = "alloc")]`, deliberately. The VARIANT is gated on
        // `nros-rmw/alloc`; this arm used to be gated on `nros-cpp/alloc`, and
        // that implication only runs one way. Cargo unifies features across the
        // graph, so anything else enabling `nros-rmw/alloc` made the variant
        // exist while this arm was compiled out:
        //
        //   error[E0004]: non-exhaustive patterns:
        //     `TransportError::BackendDynamic(_)` not covered
        //
        // which took out the whole zephyr fixture family via the cortex-m C++
        // leaf (thumbv7m-none-eabi) once `nros-cpp`'s `default = []` (phase-361
        // W3 / issue 0591) made that combination reachable.
        //
        // Un-gating is safe because no buildable configuration lacks the
        // variant: `nros-cpp --no-default-features --features rmw-cffi` fails
        // to link at all ("no global memory allocator found but one is
        // required") since `nros-c` pulls the alloc machinery in regardless. If
        // a genuinely alloc-free build is ever made to work, this line fails to
        // compile with "no variant named BackendDynamic" — a loud, local error
        // pointing straight here, which is the right way to find out.
        //
        // A `_` arm is NOT the fix: issue 0586's gate states its guarantee as
        // "the mappers themselves are exhaustive (no `_` arm), so rustc already
        // refuses a new variant until someone maps it".
        T::BackendDynamic(_) => NROS_CPP_RET_TRANSPORT_ERROR,
        // Entity creation — the C++ surface has codes for these.
        T::PublisherCreationFailed => NROS_CPP_RET_PUBLISH_FAILED,
        T::SubscriberCreationFailed => NROS_CPP_RET_SUBSCRIPTION_FAILED,
        T::ServiceServerCreationFailed | T::ServiceClientCreationFailed => {
            NROS_CPP_RET_SERVICE_FAILED
        }
        T::ServiceRequestFailed | T::ServiceReplyFailed => NROS_CPP_RET_SERVICE_FAILED,

        // Payload / capacity.
        T::SerializationError | T::DeserializationError => NROS_CPP_RET_ERROR,
        T::BufferTooSmall | T::MessageTooLarge | T::TooLarge | T::BadAlloc => NROS_CPP_RET_FULL,

        // Timing.
        T::Timeout => NROS_CPP_RET_TIMEOUT,
        T::WouldBlock | T::NoData => NROS_CPP_RET_TRY_AGAIN,

        // Caller / configuration.
        T::InvalidConfig | T::InvalidArgument => NROS_CPP_RET_INVALID_ARGUMENT,
        T::TopicNameInvalid => NROS_CPP_RET_INVALID_ARGUMENT,
        T::NodeNameNonExistent => NROS_CPP_RET_NOT_FOUND,
        T::IncompatibleQos => NROS_CPP_RET_NOT_ALLOWED,
        T::Unsupported | T::LoanNotSupported => NROS_CPP_RET_UNSUPPORTED,
        T::TaskStartFailed => NROS_CPP_RET_ERROR,

        // `BackendDynamic` is gated on NROS-RMW's `alloc`, which is NOT this
        // crate's `alloc`: a cortex-m Zephyr image has the former without the
        // latter, so the variant EXISTS while the arm above is compiled out and
        // the match is non-exhaustive (E0004). Covered here, last, where a
        // wildcard belongs — an earlier attempt put it beside the arm it
        // complements, which silently shadowed every named arm after it.
        //
        // Exhaustiveness (issue 0586) still holds wherever the two gates agree;
        // this only fills the gap where they do not. `unreachable_patterns` is
        // allowed because when NEITHER crate has alloc the variant is absent and
        // this arm is genuinely dead.
        #[cfg(not(feature = "alloc"))]
        #[allow(unreachable_patterns)]
        _ => NROS_CPP_RET_TRANSPORT_ERROR,
    }
}

/// Phase 155.C — map `NodeError` to the closest `NROS_CPP_RET_*` code.
/// Unknown variants stay TRANSPORT_ERROR (-100) — the legacy catch-all.
#[cfg(feature = "rmw-cffi")]
/// phase-412 -- stable codes for the boot self-report.
///
/// EXHAUSTIVE and no `_` arm, for the reason issue 0586's gate states about the
/// ret mappers beside it: rustc must refuse a new variant until someone numbers
/// it. A `_` arm here would silently file every future error under "unknown",
/// which is the shape this whole record exists to remove.
///
/// The numbering is assigned HERE rather than taken from the Rust discriminant,
/// because a discriminant shifts when a variant is inserted and a dump decoded
/// against the wrong numbering names the wrong error, confidently. Append only.
#[cfg(feature = "rmw-cffi")]
fn node_error_class(err: &nros_node::NodeError) -> u32 {
    use nros_node::NodeError as E;
    match err {
        E::Transport(_) => 1,
        E::NameTooLong => 2,
        E::Serialization => 3,
        E::Deserialization => 4,
        E::BufferTooSmall => 5,
        E::ActionCreationFailed => 6,
        E::ServiceRequestFailed => 7,
        E::ServiceReplyFailed => 8,
        E::Timeout => 9,
        E::NotInitialized => 10,
        E::RequestInFlight => 11,
        E::NoSchedContextSlot => 12,
        E::InvalidSchedContextBinding => 13,
        E::NodeTableFull => 14,
        E::ExecutorFull => 15,
        E::BackendMismatch => 16,
        E::ShutdownCallbacksFull => 17,
    }
}

/// Which `TransportError`, for the report. Append only; see [`node_error_class`].
///
/// This is the field that pays for itself: the C++ ABI collapses eight distinct
/// transport variants onto `-100`, so a board that fails to create a
/// subscription reports "transport error" and nothing says which of the eight.
#[cfg(feature = "rmw-cffi")]
fn transport_error_class(err: &nros_rmw::TransportError) -> u32 {
    use nros_rmw::TransportError as T;
    match err {
        T::ConnectionFailed => 1,
        T::Disconnected => 2,
        T::PublisherCreationFailed => 3,
        T::SubscriberCreationFailed => 4,
        T::ServiceServerCreationFailed => 5,
        T::ServiceClientCreationFailed => 6,
        T::PublishFailed => 7,
        T::ServiceRequestFailed => 8,
        T::ServiceReplyFailed => 9,
        T::SerializationError => 10,
        T::DeserializationError => 11,
        T::BufferTooSmall => 12,
        T::MessageTooLarge => 13,
        T::Timeout => 14,
        T::InvalidConfig => 15,
        T::WouldBlock => 16,
        T::TooLarge => 17,
        T::TaskStartFailed => 18,
        T::PollFailed => 19,
        T::KeepaliveFailed => 20,
        T::JoinFailed => 21,
        T::InvalidArgument => 22,
        T::Unsupported => 23,
        T::BadAlloc => 24,
        T::IncompatibleQos => 25,
        T::TopicNameInvalid => 26,
        T::NodeNameNonExistent => 27,
        T::LoanNotSupported => 28,
        T::NoData => 29,
        T::IncompatibleAbi => 30,
        T::Backend(_) => 31,
        // Un-gated for the reason the arm below in `transport_error_to_cpp_ret`
        // is: cargo unifies features across the graph, so the variant can exist
        // while a `#[cfg(feature = "alloc")]` arm here is compiled out.
        T::BackendDynamic(_) => 32,
    }
}

/// Put the error in the boot self-report, then let the caller map it.
///
/// Called from the ONE funnel every FFI failure already passes through, so a
/// path that starts returning errors tomorrow is recorded without anyone
/// remembering to add a call.
#[cfg(feature = "rmw-cffi")]
fn record_node_error(err: &nros_node::NodeError) {
    let (transport, ptr, len) = match err {
        nros_node::NodeError::Transport(t) => {
            let (ptr, len) = match t {
                // The message is a static string already in the image, so the
                // record carries where it is rather than a copy of it.
                nros_rmw::TransportError::Backend(s) => (s.as_ptr() as u32, s.len() as u32),
                _ => (0, 0),
            };
            (transport_error_class(t), ptr, len)
        }
        _ => (0, 0, 0),
    };
    nros_node::boot_report::note_error(node_error_class(err), transport, ptr, len);
}

/// Phase 155.C -- map `NodeError` to the closest `NROS_CPP_RET_*` code, and
/// record it in the boot self-report on the way past.
///
/// Gated with its two callees: without `rmw-cffi` there is no
/// `nros_rmw::TransportError` to map, and nothing in this crate calls it.
#[cfg(feature = "rmw-cffi")]
pub(crate) fn node_error_to_cpp_ret(err: nros_node::NodeError) -> nros_cpp_ret_t {
    use nros_node::NodeError as E;
    record_node_error(&err);
    // Issue 0436 — `-100` is documented as the catch-all for unmapped variants, so
    // a C++ caller sees "TransportError" for causes that are not transport at all.
    // That collapse is what made the PX4 bridge's init failure undiagnosable (the
    // #0428 class, one layer out). Name the real variant on the error path — it
    // only runs when something already failed.
    //
    // NOT on Zephyr (issue 0557 / phase-358 W5). Rust std stdio there goes
    // through the POSIX device-io fdtable, and on a `CONFIG_BOARD_NATIVE_POSIX`
    // build Zephyr's own `stdinout_write_vmeth` is
    //
    //     return zvfs_write(1, buffer, count);
    //
    // called FROM `zvfs_write(1, …)` — unbounded self-recursion, observed as a
    // `k_mutex` with `lock_count = 104756` and a SIGSEGV from stack exhaustion.
    // So this diagnostic is fatal on exactly the platform whose return code is
    // often the only thing that reaches the console. The mapping below is the
    // part that carries the information; the print is a hosted convenience.
    // issue 0589 — the `cfg` that used to sit here is GONE, and that is the fix
    // rather than a tidy-up. It suppressed this diagnostic on Zephyr because
    // `std::eprintln!` is fatal there; `cpp_diag!` now has a Zephyr arm that
    // routes through `nros_log`, so gating the CALL SITE only threw the
    // information away on the one platform whose return code is often all that
    // reaches the console. The macro picks the sink; callers do not.
    crate::cpp_diag!("nros: NodeError::{err:?}");
    match err {
        E::NameTooLong => NROS_CPP_RET_INVALID_ARGUMENT,
        E::Serialization | E::Deserialization => NROS_CPP_RET_ERROR,
        E::BufferTooSmall => NROS_CPP_RET_FULL,
        E::Timeout => NROS_CPP_RET_TIMEOUT,
        E::NotInitialized => NROS_CPP_RET_NOT_INIT,
        E::RequestInFlight => NROS_CPP_RET_REENTRANT,
        E::Transport(t) => transport_error_to_cpp_ret(t),
        // issue 0557 / phase-358 W5 — these eight used to fall into the `_` arm
        // below and come out as `-100 TRANSPORT_ERROR`, which is how a Zephyr
        // Cyclone C action image reported "transport error" for a failure that
        // never touched the transport. On an embedded guest the return code is
        // frequently the ONLY thing that reaches the console (issue 0589 makes
        // the print itself fatal there), so an unmapped variant is not a cosmetic
        // problem — it is the whole diagnosis.
        E::ActionCreationFailed => NROS_CPP_RET_ERROR,
        E::ServiceRequestFailed | E::ServiceReplyFailed => NROS_CPP_RET_SERVICE_FAILED,
        E::NoSchedContextSlot => NROS_CPP_RET_FULL,
        E::InvalidSchedContextBinding => NROS_CPP_RET_INVALID_ARGUMENT,
        E::NodeTableFull | E::ExecutorFull | E::ShutdownCallbacksFull => NROS_CPP_RET_FULL,
        E::BackendMismatch => NROS_CPP_RET_UNSUPPORTED,
        // NO wildcard. Every `NodeError` variant is named above, and rustc
        // rejects a `_` arm here as unreachable — which is the property worth
        // keeping: adding a variant to `NodeError` now fails to compile until
        // someone decides what the C++ caller should see, instead of silently
        // joining the `-100` pile this arm used to be.
    }
}

/// Issue 0436 — multi-RMW init for the C++ surface: opens one session per spec
/// into the CALLER'S storage, producing a real [`CppContext`].
///
/// # Why this exists next to `nros_init_multi`
///
/// `nros_init_multi` (the `nros-bridge` C ABI behind `<nros/bridge.hpp>`'s
/// `MultiExecutor`) returns a heap `ExecutorBox { executor, _spec_strings }`. The
/// C++ Node surface casts its handle to `*mut CppContext`
/// (`{ executor, domain_id, in_dispatch, backing }`). Both begin with the SAME
/// `Executor<'static>` at offset 0, so the cast reads correctly and then writes
/// `domain_id` / `in_dispatch` over the `Vec<String>` that follows in the bridge's
/// box — memory corruption, observed as PX4 dumping core during construction.
///
/// The two surfaces need ONE handle type. This is that: same storage contract as
/// [`nros_cpp_init`] (caller owns the buffer, executor carved in place), so every
/// existing C++ path — `nros::Node`, `NodeBuilder().rmw(name)`, publishers,
/// subscriptions — works against a multi-session executor unchanged.
///
/// `specs[0]` is the primary session; the rest become extras, each findable by its
/// `rmw` name (see `Executor::extra_session_ids`).
///
/// # Safety
/// `specs` must point to `specs_len` valid [`NrosCppSessionSpec`] whose string
/// fields are NUL-terminated and outlive the call; `storage` must be a
/// `NROS_CPP_EXECUTOR_STORAGE_SIZE`-byte, `u64`-aligned buffer that outlives the
/// executor (identical to `nros_cpp_init`).
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_init_multi(
    specs: *const NrosCppSessionSpec,
    specs_len: usize,
    storage: *mut c_void,
) -> nros_cpp_ret_t {
    if specs.is_null() || specs_len == 0 || storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    unsafe extern "C" {
        fn nros_app_register_backends();
    }
    // Same as `nros_cpp_init`: the generated strong def registers the linked
    // backends. Without this the registry is empty and every spec's lookup misses
    // (issue 0436 — `nros_init_multi` does NOT do it, which is its own trap).
    unsafe {
        nros_app_register_backends();
    }

    // Borrow the caller's C strings for the duration of the call. `open_multi_in`
    // copies what it needs into the executor + `extra_session_ids`.
    // Fixed table — no allocator dependency (this crate is no_std-capable) and a
    // bridge names a handful of backends, not dozens.
    const MAX_SPECS: usize = 8;
    if specs_len > MAX_SPECS {
        return NROS_CPP_RET_FULL;
    }
    let mut owned: [Option<nros_node::executor::SessionSpec<'_>>; MAX_SPECS] = Default::default();
    for (i, slot) in owned.iter_mut().take(specs_len).enumerate() {
        let spec = unsafe { &*specs.add(i) };
        let rmw = match unsafe { cstr_to_str(spec.rmw) } {
            Some(s) => s,
            None => return NROS_CPP_RET_INVALID_ARGUMENT,
        };
        let locator = if spec.locator.is_null() {
            ""
        } else {
            match unsafe { cstr_to_str(spec.locator) } {
                Some(s) => s,
                None => return NROS_CPP_RET_INVALID_ARGUMENT,
            }
        };
        let node_name = if spec.node_name.is_null() {
            ""
        } else {
            unsafe { cstr_to_str(spec.node_name) }.unwrap_or("")
        };
        let namespace = if spec.namespace_.is_null() {
            "/"
        } else {
            unsafe { cstr_to_str(spec.namespace_) }.unwrap_or("/")
        };
        *slot = Some(nros_node::executor::SessionSpec {
            rmw,
            locator,
            domain_id: spec.domain_id,
            node_name,
            namespace,
        });
    }
    let mut flat: [nros_node::executor::SessionSpec<'_>; MAX_SPECS] =
        [owned[0].expect("specs_len >= 1"); MAX_SPECS];
    for (dst, slot) in flat.iter_mut().zip(owned.iter().take(specs_len)) {
        *dst = slot.expect("filled above");
    }
    let owned = &flat[..specs_len];

    let ctx_ptr = storage as *mut CppContext;
    let backing: &'static mut [core::mem::MaybeUninit<u64>] = unsafe {
        core::slice::from_raw_parts_mut(
            core::ptr::addr_of_mut!((*ctx_ptr).backing) as *mut core::mem::MaybeUninit<u64>,
            CPP_EXECUTOR_BACKING_U64S,
        )
    };
    // SAFETY: as `nros_cpp_init` — caller-owned, correctly sized/aligned buffer.
    match unsafe { CppExecutor::open_multi_in(owned, backing, nros_node::ExecutorSizing::DEFAULT) }
    {
        Ok(executor) => {
            unsafe {
                core::ptr::write(core::ptr::addr_of_mut!((*ctx_ptr).executor), executor);
                core::ptr::write(
                    core::ptr::addr_of_mut!((*ctx_ptr).domain_id),
                    owned.first().map(|s| s.domain_id).unwrap_or(0),
                );
                core::ptr::write(core::ptr::addr_of_mut!((*ctx_ptr).in_dispatch), false);
                core::ptr::write(core::ptr::addr_of_mut!((*ctx_ptr).tag), CPP_CONTEXT_TAG);
            }
            NROS_CPP_RET_OK
        }
        Err(e) => node_error_to_cpp_ret(e),
    }
}

/// Issue 0436 — C mirror of `nros_node::executor::SessionSpec`, matching
/// `nros_session_spec_t` in `<nros/bridge.h>` field for field so a caller can use
/// either entry point with one struct.
#[cfg(feature = "rmw-cffi")]
#[repr(C)]
pub struct NrosCppSessionSpec {
    /// Canonical backend name, e.g. `"zenoh"`. Must be registered.
    pub rmw: *const c_char,
    /// Backend-specific locator. NULL = empty (uORB ignores it entirely).
    pub locator: *const c_char,
    /// ROS domain id.
    pub domain_id: u32,
    /// Session-default node name. NULL = empty.
    pub node_name: *const c_char,
    /// Session-default namespace. NULL = "/".
    pub namespace_: *const c_char,
}

/// Shut down an nros executor session.
///
/// Drops the executor in-place within the caller's storage.
///
/// # Safety
/// `storage` must point to a live `CppContext` written by `nros_cpp_init()`, or NULL (no-op).
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_fini(storage: *mut c_void) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_OK;
    }

    unsafe {
        let ctx = &mut *(storage as *mut CppContext);
        let _ = ctx.executor.close();
        core::ptr::drop_in_place(storage as *mut CppContext);
    }

    NROS_CPP_RET_OK
}

/// Phase 266 (W5b/W6) — named variant of [`nros_board_native_run_components`].
///
/// `session_name` sets the primary session / node name visible via `ros2 node list`
/// (the #98 fix for C entries). NULL or empty → falls back to `"node"` (the
/// unified compiled default — same as the Rust `nros::main!` resolver compiled
/// default and the C++ 2-arg `nros::init` default after this phase).
///
/// The generated typed C entry (`nros codegen entry --lang c --typed`) calls this
/// from `main`, passing `nros_boot_config_node_name(&NROS_BOOT_CONFIG)` which
/// resolves to the launch node name for single-node entries (or NULL for multi-node,
/// where the "node" default applies).
///
/// # Safety
/// `session_name` must be NULL or a valid null-terminated string.
/// `setup` must be a valid function pointer; it is invoked once with the executor
/// handle (a `*mut CppContext`) before the spin loop.
// phase-359 W10 — `env`, not `std`: this entry resolves its locator, domain
// and spin bound from the PROCESS ENVIRONMENT, which is the capability, and
// `CString` is `alloc`. Nothing here needs the flavour.
#[cfg(all(feature = "rmw-cffi", feature = "env"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_board_native_run_components_named(
    session_name: *const c_char,
    setup: Option<unsafe extern "C" fn(executor: *mut c_void) -> i32>,
) -> i32 {
    let setup = match setup {
        Some(f) => f,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };

    // Resolve session name: null / empty → "node" (unified default, phase 266).
    let name_resolved: &core::ffi::CStr = if session_name.is_null() {
        c"node"
    } else {
        let s = unsafe { core::ffi::CStr::from_ptr(session_name) };
        if s.is_empty() { c"node" } else { s }
    };

    // issue 0687 — this entry USED to read `$NROS_LOCATOR` / `$ROS_DOMAIN_ID`
    // here and pass them down as the BAKED rung. `nros_cpp_init` already
    // applies the env rung through `try_resolve_hosted`, so the overlay was a
    // second reader of the same two variables — and it disagreed with the
    // first: it did not accept the legacy `$ZENOH_LOCATOR`, printed no
    // deprecation warning, and turned a malformed or out-of-range
    // `$ROS_DOMAIN_ID` into a SILENT domain 0, which is the failure mode #206
    // fixed for every other language. Passing NULL/0 (the unset sentinels)
    // hands the whole question to the one resolver.
    let mut storage = core::mem::MaybeUninit::<CppContext>::uninit();
    let sptr = storage.as_mut_ptr() as *mut c_void;
    let rc = unsafe {
        nros_cpp_init(
            core::ptr::null(),
            0,
            name_resolved.as_ptr(),
            core::ptr::null(),
            sptr,
        )
    };
    if rc != NROS_CPP_RET_OK {
        return rc as i32;
    }

    let setup_rc = unsafe { setup(sptr) };
    if setup_rc != 0 {
        unsafe { nros_cpp_fini(sptr) };
        return setup_rc;
    }

    // Issue 0329 — the bounded (`NROS_ENTRY_SPIN_MS`) external-observer path is
    // the shared wall-clock budgeted spin; reuse `nros_cpp_spin_for` instead of a
    // second hand-rolled budget loop. Unbounded, a native entry runs until the
    // process is signalled, so a plain `spin_once` loop suffices (no per-tick
    // yield — native is preemptively scheduled).
    let bound_ms: u32 = entry_spin_ms().min(u32::MAX as u64) as u32;
    let ret = if bound_ms != 0 {
        unsafe { nros_cpp_spin_for(sptr, bound_ms, 10) as i32 }
    } else {
        loop {
            let last = unsafe { nros_cpp_spin_once(sptr, 10) };
            if last != NROS_CPP_RET_OK {
                break last as i32;
            }
        }
    };

    unsafe { nros_cpp_fini(sptr) };
    ret
}

/// Phase 257 (W0-A, RFC-0043) — typed C Entry lifecycle (unnamed variant).
///
/// Delegates to [`nros_board_native_run_components_named`] with a NULL session
/// name, which resolves to the unified default `"node"` (phase 266 default
/// change). Kept for ABI back-compatibility with callers that do not pass a
/// name; new generated entries call `nros_board_native_run_components_named`
/// directly with the baked boot-config node name.
///
/// # Safety
/// `setup` must be a valid function pointer.
// phase-359 W10 — `env`, not `std`: this entry resolves its locator, domain
// and spin bound from the PROCESS ENVIRONMENT, which is the capability, and
// `CString` is `alloc`. Nothing here needs the flavour.
#[cfg(all(feature = "rmw-cffi", feature = "env"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_board_native_run_components(
    setup: Option<unsafe extern "C" fn(executor: *mut c_void) -> i32>,
) -> i32 {
    unsafe { nros_board_native_run_components_named(core::ptr::null(), setup) }
}

// ============================================================================
// Node
// ============================================================================

/// Opaque node handle.
///
/// A node is a lightweight view into the executor: it borrows the
/// executor for its lifetime. The C++ FFI stores the executor pointer
/// plus the node name/namespace and re-creates the borrow when needed.
#[repr(C)]
pub struct nros_cpp_node_t {
    /// Pointer to the parent executor handle (not owned).
    pub executor: *mut c_void,
    /// Node name (null-terminated, max 64 bytes including null).
    pub name: [u8; NROS_CPP_NAME_LEN],
    /// Node namespace (null-terminated, max 64 bytes including null).
    pub namespace: [u8; NROS_CPP_NAMESPACE_LEN],
    /// Phase 104.C.9.b — opaque NodeId returned by
    /// `Executor::node_builder(...).build()`. `0` = primary Node
    /// (legacy single-Session creation path); non-zero values route
    /// publisher / subscription / service creation through the
    /// per-Node session resolved via
    /// `Executor::node_session_mut(NodeId)`.
    /// Issue 0312 — the executor `NodeId`, stored BIASED BY ONE: `0` means "no
    /// node registered", `n` means `NodeId(n - 1)`.
    ///
    /// The bias exists because `NodeId` is an INDEX and the executor's node
    /// table starts empty, so the first node a C/C++ entry creates is
    /// `NodeId(0)` — indistinguishable from the "unset" sentinel this field used
    /// to carry. Every `node_id != 0` check then treated a single-node entry's
    /// only node as absent. Use [`store_node_id`] / [`node_id_opt`]; never read
    /// this field raw.
    pub node_id: u8,
    /// Reserved for future use; pad to next u64 boundary.
    pub _reserved: [u8; NROS_CPP_NODE_RESERVED],

    // Phase 211.H (issue #52) — per-topic QoS overrides the deploy plan lowered
    // from `qos_overrides.<topic>.<role>.<policy>` launch params. Set by
    // `nros_cpp_node_set_qos_overrides`; folded into each entity's QoS at
    // publisher/subscription create time. Appended at the END so existing field
    // offsets (the C++ ABI) are unchanged; null/0 = no overrides (legacy).
    /// Pointer to a `&'static`-lifetime array of [`nros_cpp_qos_override_t`], or
    /// null. The generated/hand-written entry owns the storage for the node's
    /// lifetime.
    pub qos_overrides: *const nros_cpp_qos_override_t,
    /// Number of entries in `qos_overrides`. 0 = none.
    pub qos_overrides_len: usize,
}

/// Phase 211.H (issue #52) — one per-topic QoS override, the C++-FFI mirror of
/// Rust's `nros_rmw::QoSOverride` (and nros-c's `nros_qos_override_t`). The
/// deploy plan lowers a `qos_overrides.<topic>.<role>.<policy>` launch param
/// into a `&'static` array of these, which the entry installs on the node via
/// [`nros_cpp_node_set_qos_overrides`]; the node folds matching `(topic, role)`
/// entries into each entity's QoS at create time, before the backend-compat
/// check. Plain scalar fields only (no `#[repr(C)]` enums) → trivially stable
/// cbindgen output.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct nros_cpp_qos_override_t {
    /// Resolved (remapped) topic, NUL-terminated UTF-8 (e.g. `"/chatter"`).
    pub topic: *const c_char,
    /// `0` = publisher, `1` = subscription.
    pub role: u8,
    /// `0` = reliability, `1` = durability, `2` = history, `3` = depth,
    /// `4` = deadline, `5` = lifespan, `6` = liveliness,
    /// `7` = liveliness_lease_duration. Append-only — these numbers are baked
    /// into shipped images; `nros_rmw::qos_override_policy` is the SSoT.
    pub policy: u8,
    /// Policy-specific value: reliability `0`=best_effort/`1`=reliable;
    /// durability `0`=volatile/`1`=transient_local; history
    /// `0`=keep_last/`1`=keep_all; depth = the KeepLast depth; deadline /
    /// lifespan / liveliness_lease_duration = milliseconds; liveliness =
    /// the `QoSLivelinessPolicy` discriminant
    /// (`0`=none/`1`=automatic/`2`=manual_by_topic/`3`=manual_by_node).
    pub value: u32,
}

pub(crate) const NROS_CPP_QOS_OVERRIDE_ROLE_PUBLISHER: u8 = 0;
pub(crate) const NROS_CPP_QOS_OVERRIDE_ROLE_SUBSCRIPTION: u8 = 1;

/// Phase 305 W3 (issue 0255) — resolve an entity source name (`~`/relative
/// expansion + launch remap rules) against the identity carried by the C
/// node handle, via the executor-side remap table. Every nros-cpp entity
/// registration funnels its topic/service/action name through this before
/// it reaches the wire.
#[cfg(feature = "rmw-cffi")]
pub(crate) fn resolve_node_entity_name(
    ctx: &CppContext,
    node_ref: &nros_cpp_node_t,
    source: &str,
) -> Result<nros_node::names::ResolvedName, ()> {
    let name = core::str::from_utf8(&node_ref.name)
        .ok()
        .and_then(|s| s.split('\0').next())
        .unwrap_or("");
    let ns = core::str::from_utf8(&node_ref.namespace)
        .ok()
        .and_then(|s| s.split('\0').next())
        .unwrap_or("/");
    ctx.executor.resolve_entity_name_for(name, ns, source)
}

/// Fold any overrides matching `(topic, role)` into `qos`. Mirrors
/// `nros_rmw::QoSProfile::apply_overrides`: single linear scan,
/// last-write-wins, no alloc. `overrides` may be null (`len == 0` ⇒ no-op).
///
/// # Safety
/// `overrides` must be null or point to `len` valid `nros_cpp_qos_override_t`,
/// each `topic` null or a valid NUL-terminated UTF-8 C string for the call.
pub(crate) unsafe fn apply_qos_overrides(
    mut qos: nros_rmw::QoSProfile,
    overrides: *const nros_cpp_qos_override_t,
    len: usize,
    topic: &str,
    role: u8,
) -> nros_rmw::QoSProfile {
    if overrides.is_null() || len == 0 {
        return qos;
    }
    let table = unsafe { core::slice::from_raw_parts(overrides, len) };
    for ovr in table {
        if ovr.role != role || ovr.topic.is_null() {
            continue;
        }
        let Ok(ovr_topic) = (unsafe { core::ffi::CStr::from_ptr(ovr.topic) }).to_str() else {
            continue;
        };
        if ovr_topic != topic {
            continue;
        }
        // Issue 0303 — ONE decoder lives in `nros-rmw`. This match used to be
        // spelled here with a silent catch-all, so a policy added to the enum
        // reached three languages and not this one.
        if let Some(value) = nros_rmw::decode_qos_override_value(ovr.policy, ovr.value) {
            qos.apply_override_value(value);
        }
    }
    qos
}

/// Install the per-topic QoS override table on `node` (issue #52). Every entity
/// created afterwards folds the matching `(topic, role)` entries into its QoS —
/// the C++ mirror of Rust's `NodeHandle::set_qos_overrides`. `overrides` must
/// outlive the node. `len == 0` (or null) clears.
///
/// # Safety
/// `node` must point to an initialised `nros_cpp_node_t`; `overrides` null or
/// `len` valid entries living at least as long as the node.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_node_set_qos_overrides(
    node: *mut nros_cpp_node_t,
    overrides: *const nros_cpp_qos_override_t,
    len: usize,
) -> nros_cpp_ret_t {
    let Some(node) = (unsafe { node.as_mut() }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    node.qos_overrides = overrides;
    node.qos_overrides_len = len;
    NROS_CPP_RET_OK
}

/// Maximum RMW backend name length for `nros_cpp_node_options_t`.
/// Mirrors `BACKEND_NAME_MAX` in `nros-rmw-cffi`.
pub const NROS_CPP_RMW_NAME_LEN: usize = 32;

/// Maximum per-Node locator override length for `nros_cpp_node_options_t`.
pub const NROS_CPP_LOCATOR_LEN: usize = 128;

/// Maximum namespace length for `nros_cpp_node_options_t`. Matches the
/// inline buffer in `nros_cpp_node_t`.
pub const NROS_CPP_NAMESPACE_LEN: usize = 64;

/// Maximum node name length for the inline buffer in `nros_cpp_node_t`. Phase
/// 192.5 — single source for what was an inlined `64` at every name-buffer site.
/// NOTE: `nros_node::limits` uses a larger namespace/name bound; this C++ ABI is
/// the deliberately-fixed 64-byte embedded inline — reconcile in a follow-up if
/// they are required to match (changing it is a `#[repr(C)]` ABI change).
pub const NROS_CPP_NAME_LEN: usize = 64;

/// Reserved padding bytes in `nros_cpp_node_t` (pad `node_id` to the next u64
/// boundary). Phase 192.5 — names the struct-layout `7`.
pub const NROS_CPP_NODE_RESERVED: usize = 7;

/// Sentinel value for `domain_id_override`. When set, the executor's
/// existing domain_id is used.
pub const NROS_CPP_DOMAIN_ID_INHERIT: u32 = u32::MAX;

/// Phase 104.C.9 — extended node-creation options (C++ FFI).
///
/// Mirrors `nros_node_options_t` in nros-c (Phase 104.C.8) — same field
/// shape, separate FFI surface so the two language wrappers can evolve
/// independently. Used by `nros_cpp_node_create_ex` and the C++
/// `NodeBuilder` wrapper in `nros/node.hpp`.
#[repr(C)]
pub struct nros_cpp_node_options_t {
    /// Namespace storage (UTF-8, NUL-terminated within `namespace_len`).
    pub namespace: [u8; NROS_CPP_NAMESPACE_LEN],
    /// Length of `namespace` in bytes (excluding NUL).
    pub namespace_len: usize,
    /// RMW backend name (e.g. "zenoh", "cyclonedds"). Empty selects first-registered.
    pub rmw_name: [u8; NROS_CPP_RMW_NAME_LEN],
    /// Length of `rmw_name`.
    pub rmw_name_len: usize,
    /// Optional per-Node locator override. Empty inherits the executor's.
    pub locator: [u8; NROS_CPP_LOCATOR_LEN],
    /// Length of `locator`.
    pub locator_len: usize,
    /// Per-Node domain ID. `NROS_CPP_DOMAIN_ID_INHERIT` = inherit.
    pub domain_id_override: u32,
    /// SchedContext slot to bind on handles created via this Node.
    /// 0 = executor default.
    pub sched_context_id: u8,
    /// Reserved for future use; must be zero.
    pub _reserved: [u8; 3],
}

impl Default for nros_cpp_node_options_t {
    fn default() -> Self {
        Self {
            namespace: [0u8; NROS_CPP_NAMESPACE_LEN],
            namespace_len: 0,
            rmw_name: [0u8; NROS_CPP_RMW_NAME_LEN],
            rmw_name_len: 0,
            locator: [0u8; NROS_CPP_LOCATOR_LEN],
            locator_len: 0,
            domain_id_override: NROS_CPP_DOMAIN_ID_INHERIT,
            sched_context_id: 0,
            _reserved: [0u8; 3],
        }
    }
}

/// Phase 104.C.9 — zero-initialised `nros_cpp_node_options_t`.
///
/// All length fields default to 0 ("inherit"); `domain_id_override` is
/// `NROS_CPP_DOMAIN_ID_INHERIT`. The C++ `NodeOptions` wrapper consumes
/// this via `Executor::node_builder(name)`.
#[unsafe(no_mangle)]
pub extern "C" fn nros_cpp_node_get_default_options() -> nros_cpp_node_options_t {
    nros_cpp_node_options_t::default()
}

/// Create a node on an executor.
///
/// Equivalent to populating an [`nros_cpp_node_options_t`] with the
/// supplied namespace + zero defaults and calling
/// `nros_cpp_node_create_ex`. Kept for source compatibility with
/// pre-Phase-104.C.9 callers.
///
/// # Parameters
/// * `executor_handle` — Opaque executor handle from `nros_cpp_init()`.
/// * `name` — Node name (null-terminated). Must not be NULL.
/// * `namespace` — Node namespace (null-terminated), or NULL for `"/"`.
/// * `out_node` — Receives the node handle on success.
///
/// # Safety
/// * `executor_handle` must be a valid handle from `nros_cpp_init()`.
/// * `name` must be a valid null-terminated string.
/// * `namespace` must be a valid null-terminated string or NULL.
/// * `out_node` must be a valid pointer to an `nros_cpp_node_t`.
///
/// # Returns
/// `NROS_CPP_RET_OK` on success, error code otherwise.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_node_create(
    executor_handle: *mut c_void,
    name: *const c_char,
    namespace: *const c_char,
    out_node: *mut nros_cpp_node_t,
) -> nros_cpp_ret_t {
    if name.is_null() || out_node.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let name_str = match unsafe { cstr_to_str(name) } {
        Some(s) if !s.is_empty() && s.len() < 64 => s,
        _ => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let ns_str = if namespace.is_null() {
        "/"
    } else {
        match unsafe { cstr_to_str(namespace) } {
            Some(s) if s.len() < NROS_CPP_NAMESPACE_LEN => s,
            _ => return NROS_CPP_RET_INVALID_ARGUMENT,
        }
    };

    // Phase 268 (RFC-0046) — register the node through `Executor::node_builder`
    // (the one shared site both languages funnel through) so it gets a distinct
    // NodeId + NodeRecord carrying this name. Previously this left `node_id = 0`
    // (unregistered), so node-id-keyed entity paths — notably the raw arena
    // subscription register — fell back to the session's name. A multi-node
    // entry therefore collapsed every such entity onto the single session name
    // (`/node`) instead of its component (`/talker`, `/listener`). With a real
    // NodeId, `node_session_mut(node_id)` still resolves the shared primary
    // session (no rmw override → slot 0), so routing is unchanged — only the
    // node identity is now correct per component, matching the Rust + `_ex`
    // paths.
    let Some(ctx) = (unsafe { cpp_ctx_checked(executor_handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let node_id = match ctx
        .executor
        .node_builder(name_str)
        .namespace(ns_str)
        .build()
    {
        Ok(id) => id,
        Err(_) => return NROS_CPP_RET_ERROR,
    };

    let out = unsafe { &mut *out_node };
    out.executor = executor_handle;
    out.name = [0u8; NROS_CPP_NAME_LEN];
    out.name[..name_str.len()].copy_from_slice(name_str.as_bytes());
    out.namespace = [0u8; NROS_CPP_NAMESPACE_LEN];
    if !ns_str.is_empty() {
        out.namespace[..ns_str.len()].copy_from_slice(ns_str.as_bytes());
    }
    store_node_id(out, node_id);
    out._reserved = [0u8; NROS_CPP_NODE_RESERVED];

    // phase-308 — open this node in the metadata recorder so the entities
    // declared next attribute to it (the RMW seam carries no node). No-op
    // unless `metadata-mode` is on.
    crate::metadata_hooks::on_node_create(name_str, ns_str, ctx.domain_id);

    NROS_CPP_RET_OK
}

/// Issue 0312 — the ROS 2 type-hash spelling used when a caller supplies none.
///
/// Upstream uses `RIHS01_<hex>` (Iron+) or the literal `TypeHashNotSupported`
/// (Humble). An EMPTY string is neither, and it is what most in-tree C examples
/// and the user-facing templates pass. That mattered invisibly: the field is a
/// SEGMENT of the ROS 2 liveliness keyexpr, so an empty hash produced a token
/// `rmw_zenoh_cpp` does not count — the entity delivered data normally while
/// being absent from `ros2 topic info` / `ros2 node info`. Delivery does not use
/// the hash, so nothing failed loudly.
///
/// Normalising here rather than rejecting keeps every existing caller working
/// AND discoverable; a caller that knows its hash should still pass it, since
/// the sentinel means "unknown", not "any".
pub(crate) const TYPE_HASH_UNSPECIFIED: &str = "TypeHashNotSupported";

/// Issue 0312 — map an empty type hash to [`TYPE_HASH_UNSPECIFIED`].
pub(crate) fn normalize_type_hash(hash: &str) -> &str {
    if hash.is_empty() {
        TYPE_HASH_UNSPECIFIED
    } else {
        hash
    }
}

/// Issue 0312 — encode an executor node index into the handle field, biased by
/// one so `0` can keep meaning "unset". Pure `u8` (not `NodeId`) so it holds
/// without the `rmw-cffi` feature and stays unit-testable.
pub(crate) const fn encode_node_id(raw: u8) -> u8 {
    raw.saturating_add(1)
}

/// Issue 0312 — decode the handle field. `None` when no node is registered.
pub(crate) const fn decode_node_id(field: u8) -> Option<u8> {
    if field == 0 { None } else { Some(field - 1) }
}

/// Store an executor [`NodeId`] on a handle.
#[cfg(feature = "rmw-cffi")]
pub(crate) fn store_node_id(out: &mut nros_cpp_node_t, id: nros_node::executor::NodeId) {
    out.node_id = encode_node_id(id.raw());
}

/// Issue 0312 — decode [`nros_cpp_node_t::node_id`]. `None` when the handle
/// carries no registered node (zero-initialised by a caller that never called
/// `nros_cpp_node_create*`).
///
/// Before the bias, this was spelled `if node.node_id != 0` at eight call sites,
/// and a single-node entry's `NodeId(0)` read as "no node" at every one of them.
/// The visible symptom was a listener that received fine yet advertised no
/// subscription to ROS 2 discovery: the arena registration fell back to the
/// executor's own (empty) node name, so `create_subscription` skipped the
/// liveliness token that `ros2 topic info` counts.
#[cfg(feature = "rmw-cffi")]
pub(crate) fn node_id_opt(node: &nros_cpp_node_t) -> Option<nros_node::executor::NodeId> {
    decode_node_id(node.node_id).map(nros_node::executor::NodeId::from_raw)
}

/// Phase 104.C.9 — create a node with extended options.
///
/// Thin C++ FFI wrapper over the Rust `Executor::node_builder(name)
/// .rmw(...).locator(...).domain_id(...).namespace(...).sched(...)
/// .build()` chain. The `options.rmw_name` selector binds the Node to
/// a registered RMW backend; subsequent handle creations on the Node
/// route through that backend's session.
///
/// Currently the per-Node SchedContext field and the multi-Session
/// `extra_sessions` plumbing land via a follow-up (Phase 104.C.9.b)
/// once the C++ executor surfaces `Executor::node_builder` directly.
/// The options struct round-trips into `nros_cpp_node_t` storage today
/// so users can write code against the final API surface.
///
/// # Parameters
/// * `executor_handle` — Opaque executor handle.
/// * `name` — Node name (null-terminated). Must not be NULL.
/// * `options` — Pointer to a populated `nros_cpp_node_options_t`. NULL
///   is rejected; use `nros_cpp_node_get_default_options()` to get a
///   zero-initialised instance.
/// * `out_node` — Receives the node handle on success.
///
/// # Safety
/// All pointer arguments must satisfy their per-parameter rules. The
/// options struct's length fields must not overrun their buffers.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_node_create_ex(
    executor_handle: *mut c_void,
    name: *const c_char,
    options: *const nros_cpp_node_options_t,
    out_node: *mut nros_cpp_node_t,
) -> nros_cpp_ret_t {
    if executor_handle.is_null() || name.is_null() || options.is_null() || out_node.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let name_str = match unsafe { cstr_to_str(name) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    if name_str.is_empty() || name_str.len() >= 64 {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let opts = unsafe { &*options };
    if opts.namespace_len > NROS_CPP_NAMESPACE_LEN
        || opts.rmw_name_len > NROS_CPP_RMW_NAME_LEN
        || opts.locator_len > NROS_CPP_LOCATOR_LEN
    {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    // Phase 104.C.9.b — drive Rust's `Executor::node_builder(name)
    // .rmw(...).locator(...).domain_id(...).namespace(...).sched(...).
    // build()` and store the returned NodeId on the C++ node handle.
    // Subsequent `nros_cpp_publisher_create` / `_subscription_create`
    // / `_service_*_create` calls observe `node_id != 0` and route
    // through `Executor::node_session_mut(NodeId)` instead of the
    // primary session.
    // Issue 0436 — VALIDATE the handle. This is the site where a `MultiExecutor` /
    // `nros_init_multi` handle used to be cast to `CppContext`: both start with the
    // same `Executor`, so the cast read fine and the later field writes corrupted
    // the other struct's `Vec` (PX4 dumped core). Now it is a clean error.
    let Some(ctx) = (unsafe { cpp_ctx_checked(executor_handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let mut builder = ctx.executor.node_builder(name_str);
    if opts.rmw_name_len > 0 {
        let rmw = unsafe { core::str::from_utf8_unchecked(&opts.rmw_name[..opts.rmw_name_len]) };
        builder = builder.rmw(rmw);
    }
    if opts.locator_len > 0 {
        let loc = unsafe { core::str::from_utf8_unchecked(&opts.locator[..opts.locator_len]) };
        builder = builder.locator(loc);
    }
    if opts.domain_id_override != NROS_CPP_DOMAIN_ID_INHERIT {
        builder = builder.domain_id(opts.domain_id_override);
    }
    if opts.namespace_len > 0 {
        let ns = unsafe { core::str::from_utf8_unchecked(&opts.namespace[..opts.namespace_len]) };
        builder = builder.namespace(ns);
    }
    if opts.sched_context_id != 0 {
        builder = builder.sched(nros_node::executor::sched_context::SchedContextId(
            opts.sched_context_id,
        ));
    }
    let node_id = match builder.build() {
        Ok(id) => id,
        Err(_) => return NROS_CPP_RET_ERROR,
    };

    let out = unsafe { &mut *out_node };
    out.executor = executor_handle;
    out.name = [0u8; NROS_CPP_NAME_LEN];
    out.name[..name_str.len()].copy_from_slice(name_str.as_bytes());

    out.namespace = [0u8; NROS_CPP_NAMESPACE_LEN];
    if opts.namespace_len > 0 {
        out.namespace[..opts.namespace_len].copy_from_slice(&opts.namespace[..opts.namespace_len]);
    } else {
        out.namespace[..1].copy_from_slice(b"/");
    }
    store_node_id(out, node_id);
    out._reserved = [0u8; NROS_CPP_NODE_RESERVED];

    // phase-308 — open this node in the metadata recorder so the entities
    // declared next attribute to it (the RMW seam carries no node). The `_ex`
    // path takes its namespace from the options struct, so read it back out of
    // the handle just written rather than re-deriving it. No-op unless
    // `metadata-mode` is on.
    let ns_str = core::str::from_utf8(&out.namespace)
        .ok()
        .and_then(|s| s.split('\0').next())
        .unwrap_or("/");
    crate::metadata_hooks::on_node_create(name_str, ns_str, ctx.domain_id);

    NROS_CPP_RET_OK
}

/// Destroy a node.
///
/// Currently a no-op since the node is just metadata referencing the executor.
/// The executor owns all resources.
#[unsafe(no_mangle)]
pub extern "C" fn nros_cpp_node_destroy(_node: *mut nros_cpp_node_t) -> nros_cpp_ret_t {
    // Node is a lightweight view — nothing to free.
    NROS_CPP_RET_OK
}

/// Get the node name.
///
/// Returns a pointer to the null-terminated name string stored in the node handle.
/// The pointer is valid as long as the `nros_cpp_node_t` is alive.
///
/// # Safety
/// `node` must be a valid pointer to an initialized `nros_cpp_node_t`, or NULL.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_node_get_name(node: *const nros_cpp_node_t) -> *const c_char {
    if node.is_null() {
        return core::ptr::null();
    }
    unsafe { (*node).name.as_ptr() as *const c_char }
}

/// Get the node namespace.
///
/// Returns a pointer to the null-terminated namespace string stored in the node handle.
/// The pointer is valid as long as the `nros_cpp_node_t` is alive.
///
/// # Safety
/// `node` must be a valid pointer to an initialized `nros_cpp_node_t`, or NULL.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_node_get_namespace(
    node: *const nros_cpp_node_t,
) -> *const c_char {
    if node.is_null() {
        return core::ptr::null();
    }
    unsafe { (*node).namespace.as_ptr() as *const c_char }
}

/// RFC-0088 D4 / phase-421 W2 — the serialization format the backend behind
/// THIS node speaks, as its cross-image identity string (`"cdr"`, `"uorb"`).
///
/// Resolved through `Executor::node_session_mut(node_id)`, i.e. the same
/// session a publisher created on this node would be built against. A node
/// created with `nros_cpp_node_init_ex(..., rmw = "xrce")` sits on its own
/// session in the executor's table, so an image with two such nodes gets two
/// answers from this one function — which is the case a compile-time constant
/// (`nros_node::IMAGE_SERIALIZATION_FORMAT`, the generated
/// `NROS_SERIALIZATION_FORMAT` macro) cannot describe at all.
///
/// Returns a static, null-terminated string the caller must not free, or NULL
/// if `node` is NULL, its executor handle is not an nros-cpp context, its
/// session cannot be resolved, or the backend does not declare a format. NULL
/// is not a synonym for `"cdr"`.
///
/// # Safety
/// `node` must be a valid pointer to an initialized `nros_cpp_node_t`, or NULL.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_node_get_serialization_format(
    node: *const nros_cpp_node_t,
) -> *const c_char {
    if node.is_null() {
        return core::ptr::null();
    }

    #[cfg(feature = "rmw-cffi")]
    {
        let node = unsafe { &*node };
        let Some(ctx) = (unsafe { cpp_ctx_checked(node.executor) }) else {
            return core::ptr::null();
        };
        // No NodeId (a node handle built before registration, or one whose
        // record is gone) means no per-node session to ask, and the primary
        // session is a GUESS in exactly the image this function exists for.
        let Some(node_id) = node_id_opt(node) else {
            return core::ptr::null();
        };
        match ctx.executor.node_session_mut(node_id) {
            Some(session) => session.serialization_format_cstr(),
            None => core::ptr::null(),
        }
    }

    #[cfg(not(feature = "rmw-cffi"))]
    {
        core::ptr::null()
    }
}

/// Phase 88.12 — return the `nros_log::Logger` keyed on this node's
/// name. Opaque handle on the C++ side; pass to `NROS_LOG_*` macros.
///
/// # Safety
/// `node` must be a valid pointer to an initialized `nros_cpp_node_t`,
/// or NULL (in which case NULL is returned).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_node_get_logger(
    node: *const nros_cpp_node_t,
) -> *const core::ffi::c_void {
    if node.is_null() {
        return core::ptr::null();
    }
    let name_ptr = unsafe { (*node).name.as_ptr() };
    // Find the NUL terminator in the fixed-size `name` array to
    // build a `&str` for the intern table lookup.
    let name_bytes = unsafe { core::slice::from_raw_parts(name_ptr, (*node).name.len()) };
    let nul = name_bytes.iter().position(|&b| b == 0).unwrap_or(0);
    let name = core::str::from_utf8(&name_bytes[..nul]).unwrap_or("");
    let logger: &'static nros_log::Logger = nros_log::get_logger(name);
    (logger as *const nros_log::Logger).cast()
}

// ============================================================================
// Spin
// ============================================================================

/// Drive transport I/O and dispatch any registered callbacks.
///
/// Call this periodically so subscriptions can receive data.
///
/// # Parameters
/// * `handle` — Opaque executor handle from `nros_cpp_init()`.
/// * `timeout_ms` — Maximum time to block waiting for I/O (milliseconds).
///
/// # Safety
/// `handle` must be a valid handle returned by `nros_cpp_init()`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_spin_once(
    handle: *mut c_void,
    timeout_ms: i32,
) -> nros_cpp_ret_t {
    let Some(ctx) = (unsafe { cpp_ctx_checked(handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let ms = timeout_ms.max(0) as u64;
    // Issue 0290 — refuse to re-enter from inside a callback. Without this a
    // blocking helper called during dispatch (`Client::call`, `Future::wait`,
    // which loop on this fn) would create a second `&mut Executor` while the
    // outer dispatch still holds one. `Future::wait` treats REENTRANT as
    // non-transient and propagates it, so the caller gets
    // `ErrorCode::Reentrant` rather than corruption.
    let CppContext {
        executor,
        in_dispatch,
        ..
    } = ctx;
    let Some(_guard) = DispatchGuard::enter(in_dispatch) else {
        return NROS_CPP_RET_REENTRANT;
    };
    // Phase 127.C.4 — the prior Zephyr+std bypass (drive_io(0) + msleep)
    // starved reliable XRCE retransmission on the server side and
    // skipped arena dispatch on the client side; the underlying
    // condvar hang it worked around is gated off in
    // `Executor::spin_once` for Zephyr+std, so a normal spin runs the
    // transport for the full timeout via UDP recv and fires the arena
    // trampolines.
    let _ = executor.spin_once(core::time::Duration::from_millis(ms));
    NROS_CPP_RET_OK
}

/// Spin the executor for `duration_ms`, budgeted by WALL-CLOCK time.
///
/// Issue 0329 — the single budgeted-spin entry point. `nros::spin()` and
/// `Executor::spin()` were hand-rolled loops in the C++ headers; one of them
/// (`nros.hpp`) still budgeted by ITERATION count (`elapsed += poll_ms`), the
/// exact defect `Executor::spin` documents as fixed in Phase 118.C — an early
/// `spin_once` return on a signaled wake collapsed the loop into milliseconds.
/// Consolidating the loop here (identical to `Future::wait`'s wall-clock budget)
/// gives every caller the correct behavior with one implementation.
///
/// Runs at least one `spin_once(poll_ms)`; returns the first non-OK code, else
/// the last. `duration_ms == 0` spins exactly once.
///
/// # Safety
/// `handle` must be a valid handle returned by `nros_cpp_init()`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_spin_for(
    handle: *mut c_void,
    duration_ms: u32,
    poll_ms: i32,
) -> nros_cpp_ret_t {
    if handle.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let start_ns = nros_cpp_time_ns();
    let budget_ns = (duration_ms as u64) * 1_000_000;
    loop {
        let last = unsafe { nros_cpp_spin_once(handle, poll_ms) };
        if last != NROS_CPP_RET_OK {
            return last;
        }
        if nros_cpp_time_ns() - start_ns >= budget_ns {
            return last;
        }
    }
}

/// Phase 124.F.3 — session-level connectivity probe.
///
/// Wire-level round-trip ("is the peer / agent / router reachable?")
/// with `timeout_ms` budget. Returns `NROS_CPP_RET_OK` on reply,
/// `NROS_CPP_RET_TIMEOUT` on no reply, `NROS_CPP_RET_UNSUPPORTED`
/// when the active backend can't probe. Mirrors micro-ROS's
/// `rmw_uros_ping_agent`.
///
/// # Safety
/// `handle` must be a valid `CppContext` from `nros_cpp_init()`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_ping(
    handle: *mut c_void,
    timeout_ms: i32,
) -> nros_cpp_ret_t {
    let Some(ctx) = (unsafe { cpp_ctx_checked(handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    match ctx.executor.ping(timeout_ms) {
        Ok(()) => NROS_CPP_RET_OK,
        Err(nros_node::NodeError::Transport(nros_rmw::TransportError::Timeout)) => {
            NROS_CPP_RET_TIMEOUT
        }
        Err(nros_node::NodeError::Transport(nros_rmw::TransportError::Unsupported)) => {
            NROS_CPP_RET_UNSUPPORTED
        }
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// phase-381 W4 — visit one node on the graph. `enclave` is NULL where the
/// backend tracks none. Strings are BORROWED for the call. Return `false` to
/// stop.
pub type nros_cpp_node_visit_fn = Option<
    unsafe extern "C" fn(
        ctx: *mut c_void,
        node_name: *const core::ffi::c_char,
        node_namespace: *const core::ffi::c_char,
        enclave: *const core::ffi::c_char,
    ) -> bool,
>;

/// phase-381 W4 — visit one name and the types on it. `types_count` may be 0 on
/// a partially discovered graph. Strings are BORROWED. Return `false` to stop.
pub type nros_cpp_names_and_types_visit_fn = Option<
    unsafe extern "C" fn(
        ctx: *mut c_void,
        name: *const core::ffi::c_char,
        types: *const *const core::ffi::c_char,
        types_count: usize,
    ) -> bool,
>;

/// phase-381 W4 — every node on the graph, with its namespace.
///
/// Reports what has been DISCOVERED and never blocks: an empty enumeration
/// means "nobody seen yet", not "nobody exists". `NROS_CPP_RET_UNSUPPORTED`
/// from a backend with no graph, which stays distinct from an empty one.
///
/// # Safety
/// `handle` must be a valid `CppContext` from `nros_cpp_init()`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_get_node_names(
    handle: *mut c_void,
    visit: nros_cpp_node_visit_fn,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    let Some(cpp) = (unsafe { cpp_ctx_checked(handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let Some(visit) = visit else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    const NAME_MAX: usize = 256;
    let mut cb = |name: &str, ns: &str, enclave: Option<&str>| -> bool {
        // NUL-terminated on the stack; the contract borrows for the call only.
        // A name that does not fit is SKIPPED, never truncated — a truncated
        // node name is a different, plausible node.
        let mut name_buf = [0u8; NAME_MAX];
        let mut ns_buf = [0u8; NAME_MAX];
        let mut enc_buf = [0u8; NAME_MAX];
        if name.len() >= NAME_MAX || ns.len() >= NAME_MAX {
            return true;
        }
        name_buf[..name.len()].copy_from_slice(name.as_bytes());
        ns_buf[..ns.len()].copy_from_slice(ns.as_bytes());
        let enc_ptr = match enclave {
            Some(e) if e.len() < NAME_MAX => {
                enc_buf[..e.len()].copy_from_slice(e.as_bytes());
                enc_buf.as_ptr() as *const core::ffi::c_char
            }
            _ => core::ptr::null(),
        };
        unsafe {
            visit(
                ctx,
                name_buf.as_ptr() as *const core::ffi::c_char,
                ns_buf.as_ptr() as *const core::ffi::c_char,
                enc_ptr,
            )
        }
    };
    match cpp.executor.get_node_names(&mut cb) {
        Ok(()) => NROS_CPP_RET_OK,
        Err(nros_node::NodeError::Transport(nros_rmw::TransportError::Unsupported)) => {
            NROS_CPP_RET_UNSUPPORTED
        }
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// The shared body for the two names-and-types entry points below.
///
/// A private helper rather than a macro: cbindgen does not expand macros, so a
/// macro-generated `#[no_mangle]` gets no header declaration and is uncallable
/// from C++ while every Rust signal stays green. Measured on the C side in this
/// same phase — see `nros-c/tests/compile/graph_query_entry_points.c`.
///
/// # Safety
/// Same contract as its callers.
#[cfg(feature = "rmw-cffi")]
unsafe fn cpp_names_and_types(
    handle: *mut c_void,
    visit: nros_cpp_names_and_types_visit_fn,
    ctx: *mut c_void,
    services: bool,
) -> nros_cpp_ret_t {
    let Some(cpp) = (unsafe { cpp_ctx_checked(handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let Some(visit) = visit else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    const NAME_MAX: usize = 256;
    const TYPES_MAX: usize = 8;
    let mut cb = |name: &str, types: &[&str]| -> bool {
        let mut name_buf = [0u8; NAME_MAX];
        if name.len() >= NAME_MAX {
            return true;
        }
        name_buf[..name.len()].copy_from_slice(name.as_bytes());
        let mut type_bufs = [[0u8; NAME_MAX]; TYPES_MAX];
        let mut ptrs: [*const core::ffi::c_char; TYPES_MAX] = [core::ptr::null(); TYPES_MAX];
        let mut n = 0usize;
        for t in types.iter().take(TYPES_MAX) {
            if t.len() >= NAME_MAX {
                continue;
            }
            type_bufs[n][..t.len()].copy_from_slice(t.as_bytes());
            ptrs[n] = type_bufs[n].as_ptr() as *const core::ffi::c_char;
            n += 1;
        }
        unsafe {
            visit(
                ctx,
                name_buf.as_ptr() as *const core::ffi::c_char,
                ptrs.as_ptr(),
                n,
            )
        }
    };
    let r = if services {
        cpp.executor.get_service_names_and_types(&mut cb)
    } else {
        cpp.executor.get_topic_names_and_types(&mut cb)
    };
    match r {
        Ok(()) => NROS_CPP_RET_OK,
        Err(nros_node::NodeError::Transport(nros_rmw::TransportError::Unsupported)) => {
            NROS_CPP_RET_UNSUPPORTED
        }
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// phase-381 W4 — every topic on the graph, with the types on it. One call per
/// distinct TOPIC.
///
/// # Safety
/// `handle` must be a valid `CppContext` from `nros_cpp_init()`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_get_topic_names_and_types(
    handle: *mut c_void,
    visit: nros_cpp_names_and_types_visit_fn,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    unsafe { cpp_names_and_types(handle, visit, ctx, false) }
}

/// phase-381 W4 — every service on the graph, with its types.
///
/// # Safety
/// `handle` must be a valid `CppContext` from `nros_cpp_init()`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_get_service_names_and_types(
    handle: *mut c_void,
    visit: nros_cpp_names_and_types_visit_fn,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    unsafe { cpp_names_and_types(handle, visit, ctx, true) }
}

/// The shared body for the two counts.
///
/// # Safety
/// Same contract as its callers.
#[cfg(feature = "rmw-cffi")]
unsafe fn cpp_count_on_topic(
    handle: *mut c_void,
    topic_name: *const core::ffi::c_char,
    out_count: *mut usize,
    subscribers: bool,
) -> nros_cpp_ret_t {
    let Some(cpp) = (unsafe { cpp_ctx_checked(handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    if topic_name.is_null() || out_count.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let Ok(topic) = (unsafe { core::ffi::CStr::from_ptr(topic_name) }).to_str() else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let r = if subscribers {
        cpp.executor.count_subscribers(topic)
    } else {
        cpp.executor.count_publishers(topic)
    };
    match r {
        Ok(n) => {
            unsafe { *out_count = n };
            NROS_CPP_RET_OK
        }
        Err(nros_node::NodeError::Transport(nros_rmw::TransportError::Unsupported)) => {
            NROS_CPP_RET_UNSUPPORTED
        }
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// phase-381 W4 — how many publishers are visible on `topic_name`. The count
/// reflects what has been DISCOVERED and is never a proof of absence.
///
/// # Safety
/// `handle` must be a valid `CppContext`; `topic_name` NUL-terminated;
/// `out_count` writable.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_count_publishers(
    handle: *mut c_void,
    topic_name: *const core::ffi::c_char,
    out_count: *mut usize,
) -> nros_cpp_ret_t {
    unsafe { cpp_count_on_topic(handle, topic_name, out_count, false) }
}

/// phase-381 W4 — how many subscribers are visible on `topic_name`.
///
/// # Safety
/// `handle` must be a valid `CppContext`; `topic_name` NUL-terminated;
/// `out_count` writable.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_count_subscribers(
    handle: *mut c_void,
    topic_name: *const core::ffi::c_char,
    out_count: *mut usize,
) -> nros_cpp_ret_t {
    unsafe { cpp_count_on_topic(handle, topic_name, out_count, true) }
}

/// phase-381 W4 — one discovered endpoint on a topic. Strings BORROWED for the
/// call. No QoS: the GRANTED profile is what would answer "why is nothing
/// arriving", no backend can read one back yet, and reporting the remote's
/// DECLARED profile would be a confident wrong answer.
#[repr(C)]
pub struct nros_cpp_endpoint_info_t {
    pub node_name: *const core::ffi::c_char,
    pub node_namespace: *const core::ffi::c_char,
    pub topic_type: *const core::ffi::c_char,
    pub is_publisher: bool,
    pub endpoint_gid: [u8; 24],
}

/// phase-381 W4 — visit one endpoint. Return `false` to stop.
pub type nros_cpp_endpoint_info_visit_fn =
    Option<unsafe extern "C" fn(ctx: *mut c_void, info: *const nros_cpp_endpoint_info_t) -> bool>;

/// Shared body for the four `*_by_node` entry points.
///
/// # Safety
/// Same contract as its callers.
#[cfg(feature = "rmw-cffi")]
unsafe fn cpp_by_node(
    handle: *mut c_void,
    node_name: *const core::ffi::c_char,
    node_namespace: *const core::ffi::c_char,
    visit: nros_cpp_names_and_types_visit_fn,
    ctx: *mut c_void,
    kind: u8,
) -> nros_cpp_ret_t {
    let Some(cpp) = (unsafe { cpp_ctx_checked(handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    if node_name.is_null() || node_namespace.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let Some(visit) = visit else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let (Ok(name), Ok(ns)) = (
        unsafe { core::ffi::CStr::from_ptr(node_name) }.to_str(),
        unsafe { core::ffi::CStr::from_ptr(node_namespace) }.to_str(),
    ) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    const NAME_MAX: usize = 256;
    const TYPES_MAX: usize = 8;
    let mut cb = |n: &str, types: &[&str]| -> bool {
        let mut name_buf = [0u8; NAME_MAX];
        if n.len() >= NAME_MAX {
            return true;
        }
        name_buf[..n.len()].copy_from_slice(n.as_bytes());
        let mut type_bufs = [[0u8; NAME_MAX]; TYPES_MAX];
        let mut ptrs: [*const core::ffi::c_char; TYPES_MAX] = [core::ptr::null(); TYPES_MAX];
        let mut count = 0usize;
        for t in types.iter().take(TYPES_MAX) {
            if t.len() >= NAME_MAX {
                continue;
            }
            type_bufs[count][..t.len()].copy_from_slice(t.as_bytes());
            ptrs[count] = type_bufs[count].as_ptr() as *const core::ffi::c_char;
            count += 1;
        }
        unsafe {
            visit(
                ctx,
                name_buf.as_ptr() as *const core::ffi::c_char,
                ptrs.as_ptr(),
                count,
            )
        }
    };
    let r = match kind {
        0 => cpp
            .executor
            .get_publisher_names_and_types_by_node(name, ns, &mut cb),
        1 => cpp
            .executor
            .get_subscription_names_and_types_by_node(name, ns, &mut cb),
        2 => cpp
            .executor
            .get_service_names_and_types_by_node(name, ns, &mut cb),
        _ => cpp
            .executor
            .get_client_names_and_types_by_node(name, ns, &mut cb),
    };
    match r {
        Ok(()) => NROS_CPP_RET_OK,
        Err(nros_node::NodeError::Transport(nros_rmw::TransportError::Unsupported)) => {
            NROS_CPP_RET_UNSUPPORTED
        }
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// phase-381 W4 — what one named node PUBLISHES.
///
/// # Safety
/// `handle` must be a valid `CppContext` from `nros_cpp_init()`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_get_publisher_names_and_types_by_node(
    handle: *mut c_void,
    node_name: *const core::ffi::c_char,
    node_namespace: *const core::ffi::c_char,
    visit: nros_cpp_names_and_types_visit_fn,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    unsafe { cpp_by_node(handle, node_name, node_namespace, visit, ctx, 0) }
}

/// phase-381 W4 — what one named node SUBSCRIBES to.
///
/// `subscription`, not `subscriber`: the C++ surface takes rclcpp's vocabulary
/// (`create_subscription`, `Subscription<T>`, `get_subscriptions_info_by_topic`).
/// rclcpp has no `*_by_node` form for subscriptions at all, so the WORD comes
/// from its vocabulary rather than from a method it lacks. C says `subscriber`
/// because rcl does.
///
/// # Safety
/// `handle` must be a valid `CppContext` from `nros_cpp_init()`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_get_subscription_names_and_types_by_node(
    handle: *mut c_void,
    node_name: *const core::ffi::c_char,
    node_namespace: *const core::ffi::c_char,
    visit: nros_cpp_names_and_types_visit_fn,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    unsafe { cpp_by_node(handle, node_name, node_namespace, visit, ctx, 1) }
}

/// phase-381 W4 — what services one named node SERVES.
///
/// # Safety
/// `handle` must be a valid `CppContext` from `nros_cpp_init()`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_get_service_names_and_types_by_node(
    handle: *mut c_void,
    node_name: *const core::ffi::c_char,
    node_namespace: *const core::ffi::c_char,
    visit: nros_cpp_names_and_types_visit_fn,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    unsafe { cpp_by_node(handle, node_name, node_namespace, visit, ctx, 2) }
}

/// phase-381 W4 — what services one named node CALLS.
///
/// # Safety
/// `handle` must be a valid `CppContext` from `nros_cpp_init()`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_get_client_names_and_types_by_node(
    handle: *mut c_void,
    node_name: *const core::ffi::c_char,
    node_namespace: *const core::ffi::c_char,
    visit: nros_cpp_names_and_types_visit_fn,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    unsafe { cpp_by_node(handle, node_name, node_namespace, visit, ctx, 3) }
}

/// Shared body for the two `*_info_by_topic` entry points.
///
/// # Safety
/// Same contract as its callers.
#[cfg(feature = "rmw-cffi")]
unsafe fn cpp_endpoint_info(
    handle: *mut c_void,
    topic_name: *const core::ffi::c_char,
    visit: nros_cpp_endpoint_info_visit_fn,
    ctx: *mut c_void,
    publishers: bool,
) -> nros_cpp_ret_t {
    let Some(cpp) = (unsafe { cpp_ctx_checked(handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    if topic_name.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let Some(visit) = visit else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let Ok(topic) = (unsafe { core::ffi::CStr::from_ptr(topic_name) }).to_str() else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    const NAME_MAX: usize = 256;
    let mut cb = |info: &nros_rmw::GraphEndpointInfo<'_>| -> bool {
        let mut name_buf = [0u8; NAME_MAX];
        let mut ns_buf = [0u8; NAME_MAX];
        let mut ty_buf = [0u8; NAME_MAX];
        if info.node_name.len() >= NAME_MAX
            || info.node_namespace.len() >= NAME_MAX
            || info.topic_type.len() >= NAME_MAX
        {
            return true;
        }
        name_buf[..info.node_name.len()].copy_from_slice(info.node_name.as_bytes());
        ns_buf[..info.node_namespace.len()].copy_from_slice(info.node_namespace.as_bytes());
        ty_buf[..info.topic_type.len()].copy_from_slice(info.topic_type.as_bytes());
        let c_info = nros_cpp_endpoint_info_t {
            node_name: name_buf.as_ptr() as *const core::ffi::c_char,
            node_namespace: ns_buf.as_ptr() as *const core::ffi::c_char,
            topic_type: ty_buf.as_ptr() as *const core::ffi::c_char,
            is_publisher: info.is_publisher,
            endpoint_gid: info.endpoint_gid,
        };
        unsafe { visit(ctx, &c_info as *const nros_cpp_endpoint_info_t) }
    };
    let r = if publishers {
        cpp.executor.get_publishers_info_by_topic(topic, &mut cb)
    } else {
        cpp.executor.get_subscriptions_info_by_topic(topic, &mut cb)
    };
    match r {
        Ok(()) => NROS_CPP_RET_OK,
        Err(nros_node::NodeError::Transport(nros_rmw::TransportError::Unsupported)) => {
            NROS_CPP_RET_UNSUPPORTED
        }
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// phase-381 W4 — the publishers on `topic_name`.
///
/// # Safety
/// `handle` must be a valid `CppContext` from `nros_cpp_init()`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_get_publishers_info_by_topic(
    handle: *mut c_void,
    topic_name: *const core::ffi::c_char,
    visit: nros_cpp_endpoint_info_visit_fn,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    unsafe { cpp_endpoint_info(handle, topic_name, visit, ctx, true) }
}

/// phase-381 W4 — the subscriptions on `topic_name`.
///
/// # Safety
/// `handle` must be a valid `CppContext` from `nros_cpp_init()`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_get_subscriptions_info_by_topic(
    handle: *mut c_void,
    topic_name: *const core::ffi::c_char,
    visit: nros_cpp_endpoint_info_visit_fn,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    unsafe { cpp_endpoint_info(handle, topic_name, visit, ctx, false) }
}

// =============================================================================
// Phase 110.B / 110.C — SchedContext FFI for the C++ wrapper
// =============================================================================

/// `nros::SchedClass` mirror. Phase 110.B.
#[cfg(feature = "rmw-cffi")]
#[repr(u8)]
pub enum nros_cpp_sched_class_t {
    Fifo = 0,
    Edf = 1,
    Sporadic = 2,
    BestEffort = 3,
    TimeTriggered = 4,
}

/// `nros::Priority` mirror. Phase 110.C.
#[cfg(feature = "rmw-cffi")]
#[repr(u8)]
pub enum nros_cpp_priority_t {
    Critical = 0,
    Normal = 1,
    BestEffort = 2,
}

/// `nros::DeadlinePolicy` mirror. Phase 110.B.
#[cfg(feature = "rmw-cffi")]
#[repr(u8)]
pub enum nros_cpp_deadline_policy_t {
    Released = 0,
    Activated = 1,
    Inherited = 2,
}

/// `nros::SchedContext` mirror passed to
/// [`nros_cpp_create_sched_context`]. Time fields use `0` as
/// "absent" sentinel (mirrors the Rust `OptUs` newtype).
#[cfg(feature = "rmw-cffi")]
#[repr(C)]
pub struct nros_cpp_sched_context_t {
    pub class: nros_cpp_sched_class_t,
    pub priority: nros_cpp_priority_t,
    pub deadline_policy: nros_cpp_deadline_policy_t,
    pub period_us: u32,
    pub budget_us: u32,
    pub deadline_us: u32,
    /// Phase 110.F — opt-in OS-level priority for per-callback dispatch.
    pub os_pri: u8,
    /// Phase 110.G — TT-window offset within the executor's major frame.
    pub tt_window_offset_us: u32,
    /// Phase 110.G — TT-window length in microseconds.
    pub tt_window_duration_us: u32,
}

/// Identifier of the auto-created default `Fifo` SC. Phase 110.B.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub extern "C" fn nros_cpp_default_sched_context_id() -> u8 {
    0
}

/// Register a new scheduling context. Phase 110.B.
///
/// On success writes the new SC id through `out_sc_id` and returns
/// `NROS_CPP_RET_OK`. Returns `NROS_CPP_RET_INVALID_ARGUMENT` for null
/// pointers, `NROS_CPP_RET_ERROR` if `MAX_SC` is exhausted.
///
/// # Safety
/// All pointers must be valid; `handle` must be a context returned by
/// `nros_cpp_init`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_create_sched_context(
    handle: *mut c_void,
    cfg: *const nros_cpp_sched_context_t,
    out_sc_id: *mut u8,
) -> nros_cpp_ret_t {
    if cfg.is_null() || out_sc_id.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    use nros_node::executor::sched_context::{
        DeadlinePolicy, OptUs, Priority, SchedClass, SchedContext,
    };
    let Some(ctx) = (unsafe { cpp_ctx_checked(handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let cfg = unsafe { &*cfg };
    #[allow(deprecated)]
    let sc = SchedContext {
        class: match cfg.class {
            nros_cpp_sched_class_t::Fifo => SchedClass::Fifo,
            nros_cpp_sched_class_t::Edf => SchedClass::Edf,
            nros_cpp_sched_class_t::Sporadic => SchedClass::Sporadic,
            nros_cpp_sched_class_t::BestEffort => SchedClass::BestEffort,
            // Phase 110.G refactor — TimeTriggered is now an
            // orthogonal slot annotation; route to Fifo + populate
            // tt_window_*.
            nros_cpp_sched_class_t::TimeTriggered => SchedClass::Fifo,
        },
        priority: match cfg.priority {
            nros_cpp_priority_t::Critical => Priority::Critical,
            nros_cpp_priority_t::Normal => Priority::Normal,
            nros_cpp_priority_t::BestEffort => Priority::BestEffort,
        },
        deadline_policy: match cfg.deadline_policy {
            nros_cpp_deadline_policy_t::Released => DeadlinePolicy::Released,
            nros_cpp_deadline_policy_t::Activated => DeadlinePolicy::Activated,
            nros_cpp_deadline_policy_t::Inherited => DeadlinePolicy::Inherited,
        },
        period_us: OptUs::from_us(cfg.period_us),
        budget_us: OptUs::from_us(cfg.budget_us),
        deadline_us: OptUs::from_us(cfg.deadline_us),
        os_pri: cfg.os_pri,
        // W3b.5 — no C++ surface for the deadline-miss action yet
        // (Rust-only until a consumer exists; phase-296 W3b.5 note).
        deadline_action: Default::default(),
        tt_window_offset_us: OptUs::from_us(cfg.tt_window_offset_us),
        tt_window_duration_us: OptUs::from_us(cfg.tt_window_duration_us),
    };
    match ctx.executor.create_sched_context(sc) {
        Ok(id) => {
            unsafe { *out_sc_id = id.0 };
            NROS_CPP_RET_OK
        }
        Err(_) => NROS_CPP_RET_FULL,
    }
}

/// RFC-0052 — create a scheduling context from a RAW tier policy, routing
/// through the **common backend** [`nros_node::executor::sched_context::SchedContext::from_tier_policy`]
/// so the C++ entry codegen never re-derives the class/budget/period/deadline
/// mapping. This is the single-source-of-truth shared with the Rust runtime
/// (`ExecutorNodeRuntime::apply_tier_sched_policy`) — the C++ single-executor
/// entry path emits a call to THIS instead of building an
/// `nros_cpp_sched_context_t` field-by-field, so a `real_time` tier lowers to
/// the same Sporadic SC on every language.
///
/// `class` / `deadline_policy` are NUL-terminated UTF-8 or null (= absent);
/// `period_us` / `budget_us` / `deadline_us` use `0` as the absent sentinel.
/// A tier with no real-time policy yields a `Fifo` SC carrying only `os_pri`
/// (byte-identical to the pre-policy single-executor SC). When the policy is
/// `time_triggered`, the major-frame dispatcher is registered on the executor
/// as a side effect (same as the Rust path).
///
/// On success writes the new SC id through `out_sc_id` and returns
/// `NROS_CPP_RET_OK`.
///
/// # Safety
/// `handle` must be a context returned by `nros_cpp_init`; `class` and
/// `deadline_policy` must each be null or a NUL-terminated C string;
/// `out_sc_id` must be a valid `*mut u8`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_create_sched_context_from_policy(
    handle: *mut c_void,
    class: *const c_char,
    period_us: u64,
    budget_us: u64,
    deadline_us: u64,
    deadline_policy: *const c_char,
    os_pri: u8,
    out_sc_id: *mut u8,
) -> nros_cpp_ret_t {
    if out_sc_id.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    use nros_node::executor::sched_context::SchedContext;
    let Some(ctx) = (unsafe { cpp_ctx_checked(handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    // null → None; a non-UTF-8 string is treated as absent (the bake already
    // validated the tier vocabulary, so this only guards a caller ABI bug).
    let class_str = if class.is_null() {
        None
    } else {
        unsafe { core::ffi::CStr::from_ptr(class) }.to_str().ok()
    };
    let deadline_policy_str = if deadline_policy.is_null() {
        None
    } else {
        unsafe { core::ffi::CStr::from_ptr(deadline_policy) }
            .to_str()
            .ok()
    };
    let opt_u64 = |v: u64| if v == 0 { None } else { Some(v) };
    let (mut sc, tt_frame) = SchedContext::from_tier_policy(
        class_str,
        opt_u64(period_us),
        opt_u64(budget_us),
        opt_u64(deadline_us),
        deadline_policy_str,
    )
    .unwrap_or((SchedContext::new_fifo(), None));
    sc.os_pri = os_pri;
    if let Some(frame_us) = tt_frame {
        ctx.executor.register_time_triggered_dispatcher(frame_us);
    }
    match ctx.executor.create_sched_context(sc) {
        Ok(id) => {
            unsafe { *out_sc_id = id.0 };
            NROS_CPP_RET_OK
        }
        Err(_) => NROS_CPP_RET_FULL,
    }
}

/// Bind a registered callback to a scheduling context. Phase 110.B.
///
/// `handle` is the executor context; `callback_handle` is the index
/// returned from a previous `add_*` call; `sc_id` is from
/// [`nros_cpp_create_sched_context`].
///
/// # Safety
/// `handle` must be a context returned by `nros_cpp_init`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_bind_handle_to_sched_context(
    handle: *mut c_void,
    callback_handle: usize,
    sc_id: u8,
) -> nros_cpp_ret_t {
    let Some(ctx) = (unsafe { cpp_ctx_checked(handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let h = nros_node::executor::HandleId(callback_handle);
    let id = nros_node::executor::sched_context::SchedContextId(sc_id);
    match ctx.executor.bind_handle_to_sched_context(h, id) {
        Ok(()) => NROS_CPP_RET_OK,
        Err(_) => NROS_CPP_RET_INVALID_ARGUMENT,
    }
}

/// Phase 272 (W2) — seed the `node_name → sched_context` table before the node
/// is built. Mirrors the W1 `Executor::bind_node_name_sched` via the C++ executor
/// handle; called by the emitted entry setup AFTER creating sched-context slots
/// and BEFORE constructing/configuring components (RFC-0047: seed before build).
///
/// Covers every component shape (configure-shape C/C++ and rclcpp IS-A-node) since
/// every node funnels through `Executor::node_builder(name).build()` (RFC-0046) and
/// the builder looks up the table there. This dissolves issue #124 at the emit level:
/// rclcpp-shape nodes are seeded here and pick up their tier in the builder.
///
/// # Safety
/// `handle` must be a context returned by `nros_cpp_init`.
/// `name` must be a valid null-terminated UTF-8 string.
/// `namespace_` may be NULL (defaults to `"/"`), otherwise must be a valid
/// null-terminated UTF-8 string.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_bind_node_name_sched(
    handle: *mut c_void,
    name: *const c_char,
    namespace_: *const c_char,
    sc_id: u8,
) -> nros_cpp_ret_t {
    if name.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let Some(ctx) = (unsafe { cpp_ctx_checked(handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let name_str = match unsafe { cstr_to_str(name) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let ns_str = if namespace_.is_null() {
        "/"
    } else {
        match unsafe { cstr_to_str(namespace_) } {
            Some(s) => s,
            None => return NROS_CPP_RET_INVALID_ARGUMENT,
        }
    };
    ctx.executor.bind_node_name_sched(
        name_str,
        ns_str,
        nros_node::executor::sched_context::SchedContextId(sc_id),
    );
    NROS_CPP_RET_OK
}

/// Phase 273 (W2) — seed the group → sched-context table for a specific
/// callback group of a named node. Call BEFORE the node is constructed (before
/// `nros_cpp_node_create`) so that the group's entities pick up the binding at
/// register time. Layering: group table > node-name table > default (RFC-0047
/// Precedence). Mirror of `nros_cpp_bind_node_name_sched` at finer granularity.
///
/// # Safety
/// `handle` must be a context returned by `nros_cpp_init`.
/// `name` must be a valid null-terminated UTF-8 string.
/// `namespace_` may be NULL (defaults to `"/"`), otherwise must be a valid
/// null-terminated UTF-8 string.
/// `group` must be a valid null-terminated UTF-8 string.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_bind_group_sched(
    handle: *mut c_void,
    name: *const c_char,
    namespace_: *const c_char,
    group: *const c_char,
    sc_id: u8,
) -> nros_cpp_ret_t {
    if name.is_null() || group.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let Some(ctx) = (unsafe { cpp_ctx_checked(handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let name_str = match unsafe { cstr_to_str(name) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let ns_str = if namespace_.is_null() {
        "/"
    } else {
        match unsafe { cstr_to_str(namespace_) } {
            Some(s) => s,
            None => return NROS_CPP_RET_INVALID_ARGUMENT,
        }
    };
    let group_str = match unsafe { cstr_to_str(group) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    ctx.executor.bind_group_sched(
        name_str,
        ns_str,
        group_str,
        nros_node::executor::sched_context::SchedContextId(sc_id),
    );
    NROS_CPP_RET_OK
}

/// Phase 305 W3 (issue 0255) — declare one launch `<remap from= to=/>` rule for
/// the node identified by `(node_name, node_namespace)`. Call BEFORE the node's
/// component registers its entities (the entry codegen emits these right after
/// `nros_cpp_node_create`); every subsequent entity registration resolves its
/// source name through the executor-side remap table (`~`/relative expansion +
/// exact-FQN match, first rule wins). Errors on a full table / oversized string
/// so a dropped routing rule is never silent.
///
/// # Safety
/// `handle` must be a context returned by `nros_cpp_init`.
/// `node_name`, `from`, `to` must be valid null-terminated UTF-8 strings.
/// `node_namespace` may be NULL (defaults to `"/"`), otherwise must be a valid
/// null-terminated UTF-8 string.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_declare_remap(
    handle: *mut c_void,
    node_name: *const c_char,
    node_namespace: *const c_char,
    from: *const c_char,
    to: *const c_char,
) -> nros_cpp_ret_t {
    if node_name.is_null() || from.is_null() || to.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let Some(ctx) = (unsafe { cpp_ctx_checked(handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let name_str = match unsafe { cstr_to_str(node_name) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let ns_str = if node_namespace.is_null() {
        "/"
    } else {
        match unsafe { cstr_to_str(node_namespace) } {
            Some(s) => s,
            None => return NROS_CPP_RET_INVALID_ARGUMENT,
        }
    };
    let from_str = match unsafe { cstr_to_str(from) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let to_str = match unsafe { cstr_to_str(to) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    match ctx
        .executor
        .declare_remap(name_str, ns_str, from_str, to_str)
    {
        Ok(()) => NROS_CPP_RET_OK,
        Err(()) => NROS_CPP_RET_FULL,
    }
}

// ============================================================================
// Phase 274.W1 — RFC-0015 Model 1 primitives (session ⊥ executor + gating FFI)
// ============================================================================

/// Phase 274.W1 (RFC-0015 Model 1) — get the session handle from an opened executor.
///
/// Returns an opaque pointer to the underlying RMW session. Pass this to
/// [`nros_cpp_executor_open_over_session`] to open additional executors that
/// share the same session (the per-tier borrowed-executor model).
///
/// The returned pointer is valid as long as the primary executor's storage
/// (`nros_cpp_init` / `out_storage`) lives. NULL is returned on null input.
///
/// # Safety
/// `executor` must be a valid pointer to a `CppContext` written by `nros_cpp_init()`.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_session_handle(executor: *mut c_void) -> *mut c_void {
    let Some(ctx) = (unsafe { cpp_ctx_checked(executor) }) else {
        return core::ptr::null_mut();
    };
    ctx.executor.session_handle().into_raw()
}

/// Phase 274.W1 (RFC-0015 Model 1) — open a new `Borrowed` executor over a shared session.
///
/// Opens an executor that **does not own or close** the session on drop (the
/// `Borrowed` session store). This is the per-tier task primitive: the primary
/// executor opened the session once via `nros_cpp_init`; each tier task calls this
/// with the primary's session handle to get its own executor over the same session.
///
/// `node_name` sets the borrowed executor's node identity for graph naming; NULL
/// leaves it unnamed. `domain_id` is stored in the new context (it is consumed
/// during session open for the primary — the borrowed executor inherits the same
/// transport config from the shared session).
///
/// # Safety
/// - `session_handle` must be a valid non-null pointer from
///   [`nros_cpp_executor_session_handle`] on a live primary executor.
/// - `out_storage` must be valid caller-provided storage of at least
///   `CPP_EXECUTOR_OPAQUE_U64S * 8` bytes, 8-byte aligned, uninitialised.
/// - The primary executor's storage MUST outlive every borrowed executor built from it.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_open_over_session(
    session_handle: *mut c_void,
    node_name: *const c_char,
    domain_id: u32,
    out_storage: *mut c_void,
) -> nros_cpp_ret_t {
    if session_handle.is_null() || out_storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    // Reconstruct the SessionHandle from the opaque pointer.
    // SAFETY: caller guarantees this came from nros_cpp_executor_session_handle
    // on a still-live primary executor.
    let handle = unsafe { nros_node::SessionHandle::from_raw(session_handle) };

    // phase-271 × 274.W1 — construct in place, heap-free: carve the borrowed
    // executor's per-entry backing from the tail of the caller's `CppContext`
    // buffer (same self-referential-pin model as `nros_cpp_init`'s `open_in`),
    // then open the borrowed executor over it and write it (offset 0) +
    // domain_id. The executor Borrows the session — does NOT open a new RMW
    // session and does NOT close it on drop (the Rust tier-task pattern).
    let ctx_ptr = out_storage as *mut CppContext;
    let backing: &'static mut [core::mem::MaybeUninit<u64>] = unsafe {
        core::slice::from_raw_parts_mut(
            core::ptr::addr_of_mut!((*ctx_ptr).backing) as *mut core::mem::MaybeUninit<u64>,
            CPP_EXECUTOR_BACKING_U64S,
        )
    };
    // SAFETY: the handle's session is alive (caller's contract); `backing` is
    // sized/aligned per `ExecutorSizing::DEFAULT` and lives for the caller's
    // buffer lifetime, meeting `open_with_session_handle_in`'s contract.
    let mut executor = unsafe {
        CppExecutor::open_with_session_handle_in(
            handle,
            backing,
            nros_node::ExecutorSizing::DEFAULT,
        )
    };

    // Set node identity for graph naming (liveliness key expressions etc.).
    if let Some(name_str) = unsafe { cstr_to_str(node_name) } {
        executor.set_node_identity(name_str, "/");
    }

    // Write directly into caller-provided storage — no heap allocation. The
    // executor's slices already point into `(*ctx_ptr).backing` (its final,
    // pinned location), so moving the `Executor` struct itself is sound.
    unsafe {
        // Issue 0458 — stamp the handle tag. `CppContext` is `MaybeUninit`, and
        // `cpp_ctx_checked` reads `tag` BEFORE trusting the struct, so leaving it
        // uninitialized makes every entry point that takes this handle read
        // garbage and reject it with `INVALID_ARGUMENT` (-3). That is exactly
        // what killed the C/C++ multi-tier low tier: `nros_cpp_node_create`
        // returned -3, so `tier 'low' setup FAILED (rc=-3) — tier will not run`
        // and `/telem` never published.
        //
        // This is the SAME defect as the `in_dispatch` note below, one field
        // over: a new `CppContext` field gets stamped in `nros_cpp_init` +
        // `_init_multi` and this THIRD constructor is missed. `tag` came from
        // #0436, `in_dispatch` from #0290 (fixed as #0387). If you add a field
        // here, initialize it in all three.
        core::ptr::write(core::ptr::addr_of_mut!((*ctx_ptr).tag), CPP_CONTEXT_TAG);
        core::ptr::write(core::ptr::addr_of_mut!((*ctx_ptr).executor), executor);
        core::ptr::write(core::ptr::addr_of_mut!((*ctx_ptr).domain_id), domain_id);
        // Issue 0387 — the CppContext is `MaybeUninit`; the reentrancy guard
        // `in_dispatch` (added by #0290) MUST be initialized here or `spin_once`
        // reads uninitialized garbage as "already dispatching" and returns
        // REENTRANT on the first call. That silently killed every borrowed-tier
        // executor (C/C++ multi-tier entries: `nros_board_native_run_tiers`),
        // e.g. a low-tier `/telem` publisher that never ran. Mirror
        // `nros_cpp_init`'s init.
        core::ptr::write(core::ptr::addr_of_mut!((*ctx_ptr).in_dispatch), false);
        // Issue 0436 — stamp the handle tag LAST, exactly as `nros_cpp_init` and
        // `nros_cpp_init_multi` do: the tag means "fully initialised", so no
        // partially-built context can ever pass `cpp_ctx_checked`. This is the
        // THIRD construction site; missing it here made every borrowed-tier
        // context (C/C++ multi-tier entries) fail validation at the checked
        // accessors — the same blast radius as the 0387 `in_dispatch` bug above,
        // and for the same reason: a field this struct's readers require was
        // initialised at two of the three places that build one.
        core::ptr::write(core::ptr::addr_of_mut!((*ctx_ptr).tag), CPP_CONTEXT_TAG);
    }
    NROS_CPP_RET_OK
}

/// Phase 274.W1 (RFC-0015 Model 1) — gate this executor to a set of named callback groups.
///
/// After this call, only callbacks whose `.callback_group()` is in `groups` will
/// register on this executor. Pass `n == 0` or `groups == NULL` to clear the
/// filter (wildcard — accept all groups, which is the default).
///
/// In the per-tier model: call this on a borrowed executor BEFORE registering
/// callbacks so only the tier's groups land here. Mirrors
/// `Executor::set_active_groups`.
///
/// # Safety
/// - `executor` must be a valid pointer to a `CppContext`.
/// - `groups` must be NULL or point to `n` valid null-terminated UTF-8 C strings.
#[cfg(feature = "rmw-cffi")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_executor_set_active_groups(
    executor: *mut c_void,
    groups: *const *const c_char,
    n: usize,
) -> nros_cpp_ret_t {
    let Some(ctx) = (unsafe { cpp_ctx_checked(executor) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };

    if n == 0 || groups.is_null() {
        // Empty / NULL ⇒ wildcard (clear filter, accept all groups).
        ctx.executor.set_active_groups(&[]);
        return NROS_CPP_RET_OK;
    }

    // Collect group names from the C string pointer array onto the stack.
    // Bounded at 16 entries (generous for tier gating; silently truncates extras).
    const MAX_GROUPS_FFI: usize = 16;
    let mut group_strs = [""; MAX_GROUPS_FFI];
    let mut count = 0usize;

    let ptr_slice = unsafe { core::slice::from_raw_parts(groups, n.min(MAX_GROUPS_FFI)) };
    for &raw_ptr in ptr_slice {
        if let Some(s) = unsafe { cstr_to_str(raw_ptr) }
            && !s.is_empty()
        {
            group_strs[count] = s;
            count += 1;
        }
    }

    ctx.executor.set_active_groups(&group_strs[..count]);
    NROS_CPP_RET_OK
}

// ============================================================================
// Phase 274.W2 — RFC-0015 Model 1: native multi-tier entry (C-ABI seam)
// ============================================================================

/// Per-tier specification for [`nros_board_native_run_tiers`].
///
/// Mirrors `nros_platform::TierSpec` in C-ABI form. `groups` must point to
/// an array of `n_groups` null-terminated UTF-8 strings; NULL / 0 means
/// "accept all groups" (wildcard — degenerate single-tier).
///
/// `setup` is called once on the tier's thread, with the tier's borrowed
/// executor handle, AFTER `set_active_groups` — so only the tier's groups'
/// callbacks register. The boot tier (index 0) uses the owning executor.
///
/// `priority` is a raw POSIX nice-level adjustment (advisory on Linux
/// without elevated privileges). `stack_bytes` is informational on native
/// (`std::thread` manages the stack). `spin_period_us` is the sleep between
/// `spin_once` calls; 0 uses a 1 ms floor.
///
/// # Safety
///
/// `name` must be NULL or a valid null-terminated string.
/// `groups` must be NULL or point to `n_groups` valid null-terminated strings.
/// `setup` must be a valid function pointer or NULL (NULL skips setup — only
/// useful for tiers that register no nodes of their own).
// phase-359 W10 — `env`, not `std`. The tier runtime below spawns PLATFORM
// TASKS now, not `std::thread`s; what it still needs from the host is the
// environment it resolves its locator, domain and spin bound from.
#[cfg(all(feature = "rmw-cffi", feature = "env"))]
#[repr(C)]
pub struct NativeTierSpecC {
    pub name: *const c_char,
    pub groups: *const *const c_char,
    pub n_groups: usize,
    pub priority: i64,
    pub stack_bytes: usize,
    pub spin_period_us: u64,
    pub setup: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    /// RFC-0052 W2 — CPU pin + 1; 0 = unpinned (advisory on native v1:
    /// recorded for parity, pinning lands with the W3 executor work).
    pub core_plus1: u32,
    /// ThreadX-only (bake-validated); -1 = unset. Never consumed on native.
    pub preempt_threshold: i64,
    /// phase-296 W5.7 append — RTOS-agnostic scheduling class
    /// (`"best_effort"`|`"real_time"`|`"time_triggered"`); NULL = unset.
    /// ABI append-only: keep main.h/main.hpp/board mirrors/emitters in sync.
    pub tier_class: *const c_char,
    /// Sporadic replenishment period (µs); 0 = unset.
    pub period_us: u64,
    /// Execution budget (µs); 0 = unset.
    pub budget_us: u64,
    /// Relative deadline (µs); 0 = unset. Kernel-native consumers apply it
    /// where the RTOS offers the feature (Zephyr EDF); never consumed on
    /// native (the cooperative lowering is codegen-emitted per tier).
    pub deadline_us: u64,
    /// On-miss action (`"ignore"`|`"warn"`|`"skip"`|`"fault"`); NULL = unset.
    pub deadline_policy: *const c_char,
}

/// Phase 274.W2 (RFC-0015 Model 1) — run a native multi-tier entry over one
/// shared RMW session.
///
/// Opens ONE session on the calling (boot) thread; spawns `n_tiers - 1`
/// threads each opening a **borrowed** executor (no second RMW session, no
/// double-close). Each thread:
///   1. `nros_cpp_executor_open_over_session` — open borrowed executor.
///   2. `nros_cpp_executor_set_active_groups` — gate to the tier's groups.
///   3. `setup(executor)` — create + configure nodes (only the tier's
///      groups' callbacks register).
///   4. `spin_once` loop at `spin_period_us` until shutdown flag.
///
/// The boot thread runs the first (highest-priority) tier on the owning
/// executor; it respects the `$NROS_ENTRY_SPIN_MS` bound for test/CI use.
/// When the boot thread exits its spin loop it signals the other tiers (via
/// `Arc<AtomicBool>`) and joins them before closing the session.
///
/// # Safety
/// `tiers` must be a valid pointer to `n_tiers` [`NativeTierSpecC`] entries,
/// valid for the duration of the call. `session_name` is NULL or a valid
/// null-terminated string.
#[cfg(all(feature = "rmw-cffi", feature = "env"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_board_native_run_tiers(
    session_name: *const c_char,
    tiers: *const NativeTierSpecC,
    n_tiers: usize,
) -> i32 {
    // phase-359 W10 — `alloc`/`core`, not `std`: every one of these has a home
    // outside the standard library, and naming them through `std` made this
    // function look like it needed the flavour when what it needs is the
    // process ENVIRONMENT (see the `env` gate above).
    use alloc::{string::String, sync::Arc, vec::Vec};
    use core::sync::atomic::{AtomicBool, Ordering};

    if tiers.is_null() || n_tiers == 0 {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let tier_slice = unsafe { core::slice::from_raw_parts(tiers, n_tiers) };

    // Resolve session name: null / empty → "node".
    let name_resolved: &core::ffi::CStr = if session_name.is_null() {
        c"node"
    } else {
        let s = unsafe { core::ffi::CStr::from_ptr(session_name) };
        if s.is_empty() { c"node" } else { s }
    };

    // issue 0687 — the env overlay that stood here is gone; see the twin in
    // `nros_board_native_run_components_named`. `nros_cpp_init` resolves the
    // locator and domain through the one hosted resolver, so NULL/0 (the unset
    // sentinels) is how this entry says "whatever the environment asked for".
    let mut boot_storage = core::mem::MaybeUninit::<CppContext>::uninit();
    let sptr = boot_storage.as_mut_ptr() as *mut c_void;
    let rc = unsafe {
        nros_cpp_init(
            core::ptr::null(),
            0,
            name_resolved.as_ptr(),
            core::ptr::null(),
            sptr,
        )
    };
    if rc != NROS_CPP_RET_OK {
        return rc as i32;
    }

    // The RESOLVED domain, for the tiers that open over the shared session.
    // Reading it back beats re-reading `$ROS_DOMAIN_ID`: this is the value the
    // resolver actually chose, so a baked overlay or a validation failure can
    // never leave the tiers on a different domain from the boot executor —
    // which is issue 0656's defect one entry over.
    // SAFETY: `nros_cpp_init` returned OK, so `sptr` is an initialised
    // `CppContext`.
    let domain_id = unsafe { (*(sptr as *const CppContext)).domain_id };

    // Get the shared session handle for borrowed-executor tier threads.
    let session_handle: usize = unsafe { nros_cpp_executor_session_handle(sptr) } as usize;

    // Boot tier — apply active_groups + run setup on the owning executor.
    let boot_tier = &tier_slice[0];
    if !boot_tier.groups.is_null() && boot_tier.n_groups > 0 {
        unsafe {
            nros_cpp_executor_set_active_groups(sptr, boot_tier.groups, boot_tier.n_groups);
        }
    }
    if let Some(setup_fn) = boot_tier.setup {
        let setup_rc = unsafe { setup_fn(sptr) };
        if setup_rc != 0 {
            unsafe { nros_cpp_fini(sptr) };
            return setup_rc;
        }
    }

    crate::cpp_diag!(
        "nros: multi-tier run — {} tier(s) over one session",
        n_tiers
    );

    // Shared shutdown flag — boot thread sets it; tier tasks poll it.
    let shutdown = Arc::new(AtomicBool::new(false));

    // phase-359 W10 — one PLATFORM TASK per non-boot tier, not one
    // `std::thread`.
    //
    // Two fields this API documented as ignored now take effect, because the
    // ABI's task attribute carries them and `std::thread` did not: `priority`
    // (documented "advisory ... raw POSIX nice-level adjustment", applied by
    // nobody) and `stack_bytes` ("informational on native — `std::thread`
    // manages the stack"). A C++ author declaring a tier priority was writing a
    // number the native runtime dropped on the floor.
    //
    // Application stays BEST-EFFORT: the POSIX port sets the policy after
    // create and treats a refusal — the usual case on Linux without
    // `CAP_SYS_NICE` — as success, so an unprivileged run behaves exactly as
    // it did before.
    let mut tier_tasks: Vec<nros_platform::task::PlatformTask> = Vec::with_capacity(n_tiers - 1);
    for tier in &tier_slice[1..] {
        let tier_name = if tier.name.is_null() {
            String::new()
        } else {
            unsafe { core::ffi::CStr::from_ptr(tier.name) }
                .to_string_lossy()
                .into_owned()
        };
        let ctx = alloc::boxed::Box::into_raw(alloc::boxed::Box::new(NativeTierCtx {
            shutdown: Arc::clone(&shutdown),
            period_us: tier.spin_period_us,
            n_groups: tier.n_groups,
            groups: tier.groups as usize,
            session: session_handle,
            setup: tier.setup,
            domain_id,
            name: tier_name.clone(),
        }));
        // The port copies what it keeps, but the pointer must be valid FOR the
        // call, so the CString outlives it here.
        let task_name = alloc::ffi::CString::new(alloc::format!("nros-tier-{tier_name}"))
            .unwrap_or_else(|_| c"nros-tier".into());
        // SAFETY: `ctx` stays live until the trampoline reclaims it — on the
        // spawned task, after its loop exits.
        let spawned = unsafe {
            nros_platform::task::PlatformTask::spawn_with(
                native_tier_trampoline,
                ctx as *mut c_void,
                task_name.as_ptr(),
                tier.stack_bytes,
                tier.priority,
            )
        };
        match spawned {
            Some(task) => tier_tasks.push(task),
            None => {
                crate::cpp_diag!("nros: failed to spawn tier '{tier_name}' — tier will not run");
                // SAFETY: nothing was spawned, so nothing else owns this.
                drop(unsafe { alloc::boxed::Box::from_raw(ctx) });
            }
        }
    }

    // Boot thread spin loop (tier[0] on the owning executor).
    let bound_ms: u64 = entry_spin_ms();
    let start_ns = nros_cpp_time_ns();
    let boot_period_us = boot_tier.spin_period_us.max(1_000);
    let mut ret = 0i32;
    loop {
        let last = unsafe { nros_cpp_spin_once(sptr, 10) };
        if last != NROS_CPP_RET_OK {
            ret = last as i32;
            break;
        }
        if bound_ms != 0 {
            let elapsed_ms = (nros_cpp_time_ns() - start_ns) / 1_000_000;
            if elapsed_ms >= bound_ms {
                break;
            }
        }
        platform_sleep_us(boot_period_us);
    }

    // Signal all tier tasks to exit and wait for them.
    shutdown.store(true, Ordering::Relaxed);
    for t in tier_tasks {
        t.join();
    }

    // Close the primary (session-owning) executor.
    unsafe { nros_cpp_fini(sptr) };
    ret
}

/// What one spawned tier receives, in place of a closure's captures — the entry
/// crosses C as `*mut c_void`.
///
/// `groups` and `session` are held as `usize` for the same reason the previous
/// `SendUsize` wrapper existed: raw pointers are not `Send`, and this value
/// moves to another task.
#[cfg(all(feature = "rmw-cffi", feature = "env"))]
struct NativeTierCtx {
    shutdown: alloc::sync::Arc<core::sync::atomic::AtomicBool>,
    period_us: u64,
    n_groups: usize,
    groups: usize,
    session: usize,
    setup: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    domain_id: u32,
    name: alloc::string::String,
}

/// One tier's loop, as a platform-task entry.
///
/// # Safety
/// `arg` must be the `Box<NativeTierCtx>` raw pointer the spawn site created,
/// passed exactly once.
#[cfg(all(feature = "rmw-cffi", feature = "env"))]
unsafe extern "C" fn native_tier_trampoline(arg: *mut c_void) -> *mut c_void {
    use core::sync::atomic::Ordering;

    // SAFETY: the caller's contract — this task is the only consumer of the
    // pointer, and reclaiming it here is what frees the context when the loop
    // exits.
    let ctx = unsafe { alloc::boxed::Box::from_raw(arg as *mut NativeTierCtx) };
    let tier_name = &ctx.name;

    // Open borrowed executor (shares the session — does NOT open a new RMW
    // session and does NOT close it on drop).
    let sh = ctx.session as *mut c_void;
    let groups_ptr = ctx.groups as *const *const c_char;

    let mut tier_storage = core::mem::MaybeUninit::<CppContext>::uninit();
    let tptr = tier_storage.as_mut_ptr() as *mut c_void;
    let rc =
        unsafe { nros_cpp_executor_open_over_session(sh, core::ptr::null(), ctx.domain_id, tptr) };
    if rc != NROS_CPP_RET_OK {
        crate::cpp_diag!(
            "nros: tier '{tier_name}' FAILED to open its borrowed executor (rc={rc}) — tier will not run"
        );
        return core::ptr::null_mut();
    }

    // Gate to this tier's callback groups.
    if !groups_ptr.is_null() && ctx.n_groups > 0 {
        unsafe { nros_cpp_executor_set_active_groups(tptr, groups_ptr, ctx.n_groups) };
    }

    // Run setup (creates + configures nodes for this tier).
    if let Some(setup) = ctx.setup {
        let setup_rc = unsafe { setup(tptr) };
        if setup_rc != 0 {
            crate::cpp_diag!(
                "nros: tier '{tier_name}' setup FAILED (rc={setup_rc}) — tier will not run"
            );
            // Drop borrowed executor (no session close).
            unsafe { core::ptr::drop_in_place(tptr as *mut CppContext) };
            return core::ptr::null_mut();
        }
    }

    // Spin at the tier's period until the shutdown flag is set.
    let period_us = ctx.period_us.max(1_000);
    while !ctx.shutdown.load(Ordering::Relaxed) {
        let rc = unsafe { nros_cpp_spin_once(tptr, 10) };
        if rc != NROS_CPP_RET_OK {
            crate::cpp_diag!(
                "nros: tier '{tier_name}' spin_once returned rc={rc} — tier loop EXITING"
            );
            break;
        }
        platform_sleep_us(period_us);
    }

    // Drop borrowed executor in place (does NOT close the shared session).
    unsafe { core::ptr::drop_in_place(tptr as *mut CppContext) };
    core::ptr::null_mut()
}

/// Sleep through the platform ABI, like every other pacing site in the tree
/// since phase-359 W10.
#[cfg(all(feature = "rmw-cffi", feature = "env"))]
fn platform_sleep_us(us: u64) {
    unsafe extern "C" {
        fn nros_platform_sleep_us(us: usize);
    }
    // SAFETY: a bare pacing call with no pointer arguments, guaranteed by
    // whichever port linked the image.
    unsafe { nros_platform_sleep_us(us as usize) }
}

/// Get current monotonic time in nanoseconds.
///
/// Used by `nros::Future::wait()` (header-side) to budget its spin loop by
/// wall-clock rather than iteration count, so that an early-returning
/// `spin_once` on a signaled condvar doesn't collapse the nominal timeout
/// into microseconds. Phase 89.2.
#[unsafe(no_mangle)]
pub extern "C" fn nros_cpp_time_ns() -> u64 {
    // RFC-0073 / phase-352: the platform clock IS nanoseconds now, so this
    // stopped scaling microseconds up (phase-243 had to, and the extra zeros
    // were never real precision).
    //
    // phase-359 W10 — ONE implementation. The `std` arm kept its own epoch in a
    // `OnceLock<Instant>`, so a hosted image and a target image answered "what
    // time is it" from different clocks, and `nros::Future::wait()` — the
    // caller this exists for — budgeted its spin against whichever one the
    // build happened to have.
    //
    // Reached through the C symbol rather than
    // `<nros_platform::ConcretePlatform as PlatformClock>`: `ConcretePlatform`
    // exists only when a `platform-*` feature is selected, which made this
    // crate uncompilable in its own default configuration once phase-361 W3
    // stopped defaulting to `std` — a bare `cargo check -p nros-cpp` died on a
    // type that is really a LINK-time fact, not a compile-time one. Every port
    // exports this symbol through the same contract the wake primitives rely
    // on, which is how `nros-node` and `nros-c` reach the same clock.
    unsafe extern "C" {
        fn nros_platform_clock_ns() -> u64;
    }
    // SAFETY: bare query of the platform's monotonic ns counter; the symbol is
    // guaranteed by whichever platform port linked the binary.
    unsafe { nros_platform_clock_ns() }
}

// ============================================================================
// Helpers
// ============================================================================

/// Convert a C null-terminated string to a `&str`.
///
/// Returns `None` if the pointer is null or the bytes are not valid UTF-8.
pub(crate) unsafe fn cstr_to_str<'a>(ptr: *const c_char) -> Option<&'a str> {
    if ptr.is_null() {
        return None;
    }
    // Find null terminator
    let mut len = 0usize;
    unsafe {
        while *ptr.add(len) != 0 {
            len += 1;
            if len > 4096 {
                return None; // safety bound
            }
        }
    }
    let bytes = unsafe { core::slice::from_raw_parts(ptr.cast::<u8>(), len) };
    core::str::from_utf8(bytes).ok()
}

#[cfg(test)]
mod dispatch_guard_tests {
    use super::*;

    /// Issue 0290 — the guard is what makes a blocking C++ helper refuse to
    /// re-enter the executor from inside a callback. First entry succeeds;
    /// a nested entry while the first is live returns `None`, which the
    /// callers translate into `NROS_CPP_RET_REENTRANT`.
    #[test]
    fn entering_marks_dispatching_and_dropping_clears_it() {
        let mut flag = false;
        {
            let _outer = DispatchGuard::enter(&mut flag).expect("first entry must succeed");
            // `flag` is mutably borrowed by the guard here, so the "is it set?"
            // assertion lives in `refused_entry_leaves_the_outer_dispatch_intact`
            // below, which observes the same state the real nested call sees
            // (it reaches the context through a raw pointer, not a borrow).
        }
        assert!(!flag, "dropping the guard must clear the flag");
    }

    /// A nested attempt against an already-dispatching flag is refused, and
    /// the refusal does NOT clear the flag — the outer spin is still running.
    #[test]
    fn refused_entry_leaves_the_outer_dispatch_intact() {
        let mut flag = true; // simulates "outer spin_once is dispatching"
        assert!(
            DispatchGuard::enter(&mut flag).is_none(),
            "must refuse to re-enter while a spin is in progress"
        );
        assert!(
            flag,
            "a refused entry must not clear the outer dispatch flag — \
             the outer spin is still live"
        );
    }

    /// The flag is not sticky: sequential (non-nested) spins each succeed.
    /// A guard that leaked its flag would deadlock every later blocking call.
    #[test]
    fn sequential_entries_each_succeed() {
        let mut flag = false;
        for i in 0..3 {
            let g = DispatchGuard::enter(&mut flag);
            assert!(g.is_some(), "sequential entry {i} must succeed");
            drop(g);
            assert!(!flag, "flag must be clear after entry {i}");
        }
    }
}

#[cfg(test)]
mod qos_override_tests {
    use super::*;
    use nros_rmw::{QoSDurabilityPolicy, QoSReliabilityPolicy};

    #[test]
    fn apply_qos_overrides_matches_topic_and_role() {
        let ovr = [nros_cpp_qos_override_t {
            topic: c"/chatter".as_ptr(),
            role: NROS_CPP_QOS_OVERRIDE_ROLE_PUBLISHER,
            policy: 0, // reliability
            value: 0,  // best_effort
        }];
        let base = nros_rmw::QoSProfile::default(); // Reliable

        let got = unsafe {
            apply_qos_overrides(
                base,
                ovr.as_ptr(),
                ovr.len(),
                "/chatter",
                NROS_CPP_QOS_OVERRIDE_ROLE_PUBLISHER,
            )
        };
        assert_eq!(got.reliability, QoSReliabilityPolicy::BestEffort);

        // Wrong role / topic / empty → untouched.
        let got = unsafe {
            apply_qos_overrides(
                base,
                ovr.as_ptr(),
                ovr.len(),
                "/chatter",
                NROS_CPP_QOS_OVERRIDE_ROLE_SUBSCRIPTION,
            )
        };
        assert_eq!(got.reliability, QoSReliabilityPolicy::Reliable);
        let got = unsafe {
            apply_qos_overrides(
                base,
                ovr.as_ptr(),
                ovr.len(),
                "/other",
                NROS_CPP_QOS_OVERRIDE_ROLE_PUBLISHER,
            )
        };
        assert_eq!(got.reliability, QoSReliabilityPolicy::Reliable);
        let got = unsafe {
            apply_qos_overrides(
                base,
                core::ptr::null(),
                0,
                "/chatter",
                NROS_CPP_QOS_OVERRIDE_ROLE_PUBLISHER,
            )
        };
        assert_eq!(got.reliability, QoSReliabilityPolicy::Reliable);
    }

    #[test]
    fn apply_qos_overrides_durability_and_depth() {
        let ovr = [
            nros_cpp_qos_override_t {
                topic: c"/t".as_ptr(),
                role: NROS_CPP_QOS_OVERRIDE_ROLE_SUBSCRIPTION,
                policy: 1,
                value: 1,
            },
            nros_cpp_qos_override_t {
                topic: c"/t".as_ptr(),
                role: NROS_CPP_QOS_OVERRIDE_ROLE_SUBSCRIPTION,
                policy: 3,
                value: 42,
            },
        ];
        let got = unsafe {
            apply_qos_overrides(
                nros_rmw::QoSProfile::default(),
                ovr.as_ptr(),
                ovr.len(),
                "/t",
                NROS_CPP_QOS_OVERRIDE_ROLE_SUBSCRIPTION,
            )
        };
        assert_eq!(got.durability, QoSDurabilityPolicy::TransientLocal);
        assert_eq!(got.depth, 42);
    }

    /// Issue 0312-adjacent — `NodeId` is an INDEX, so the first node a C/C++
    /// entry creates is `NodeId(0)`. The handle field stores it BIASED BY ONE
    /// so that value stays distinguishable from "no node registered"; before
    /// the bias, eight `node_id != 0` checks read a single-node entry's only
    /// node as absent.
    /// Issue 0312 — an empty type hash becomes the documented "unknown"
    /// spelling, because the raw empty string produces a liveliness keyexpr
    /// ROS 2 discovery silently ignores.
    #[test]
    fn an_empty_type_hash_normalizes_to_the_unspecified_sentinel() {
        assert_eq!(normalize_type_hash(""), TYPE_HASH_UNSPECIFIED);
        // A real hash is passed through untouched.
        assert_eq!(normalize_type_hash("RIHS01_abc"), "RIHS01_abc");
        assert_eq!(
            normalize_type_hash(TYPE_HASH_UNSPECIFIED),
            TYPE_HASH_UNSPECIFIED
        );
    }

    #[test]
    fn node_id_zero_survives_the_handle_round_trip() {
        // Zero-initialised handle = no node registered.
        assert_eq!(decode_node_id(0), None);
        // The FIRST node is index 0 — the case that used to vanish.
        assert_ne!(
            encode_node_id(0),
            0,
            "stored form must not collide with the sentinel"
        );
        assert_eq!(decode_node_id(encode_node_id(0)), Some(0));
        // Later nodes round-trip unchanged.
        for raw in [1u8, 3, 7, 254] {
            assert_eq!(decode_node_id(encode_node_id(raw)), Some(raw), "raw={raw}");
        }
    }
}

// ---------------------------------------------------------------------------
// phase-361 W8.e / issue 0594 — capabilities REQUIRE the heap / the standard
// library, they do not enable it. Turning `alloc` or `std` on for the user
// silently changes what their firmware image is; naming the feature they must
// add does not.
// ---------------------------------------------------------------------------
#[cfg(all(feature = "bridge", not(feature = "alloc")))]
compile_error!("`bridge` boxes every entity handle: add \"alloc\" to this crate's features");

// phase-359 W10 follow-up — these two were FREE-RIDING on `nros`'s guards, and
// one of them stopped being covered the moment `nros`'s was corrected.
//
// `nros/metadata-mode` used to require `std`, which covered this crate's file
// write by accident. It requires `alloc` now (its own code needed only a heap
// and a lock — the file write is HERE), so a `metadata-mode` build without
// `std` reached `std::fs::write` and produced a raw "cannot find crate `std`"
// four frames from anything a user could act on. Relaxing a guard in one crate
// exposed a missing guard in another that had never named its own requirement.
//
// The sweep is bounded by `check-std-census`, not by the feature list: a
// capability can only require a flavour if its gated code names it, and the
// census enumerates every non-test `std::` site in the tree. Of the 25, these
// two are the only ones gated by a CAPABILITY of this crate rather than by
// `std` itself.
#[cfg(all(feature = "metadata-mode", not(feature = "std")))]
compile_error!(
    "`metadata-mode` writes the sidecar file (`nros_cpp_metadata_dump`): add \"std\" to this crate's features"
);
#[cfg(all(feature = "env", not(feature = "std")))]
compile_error!(
    "`env` reads the process environment (`$NROS_ENTRY_SPIN_MS`): add \"std\" to this crate's features"
);

/// Resolve boot config, with the environment rung when this build has the
/// `env` capability.
///
/// issue 0687 — `ExecutorConfig::try_resolve` used to take `hosted_env: bool`
/// and read the process environment inside the core crate. The core takes
/// values now, so the cfg that decides whether there IS an environment lives at
/// the edge that has one.
fn resolve_boot(
    baked: nros_node::BootConfig<'_>,
) -> Result<nros_node::ExecutorConfig<'_>, nros_node::BootConfigError> {
    #[cfg(feature = "env")]
    {
        nros::env::try_resolve_hosted(baked)
    }
    #[cfg(not(feature = "env"))]
    {
        nros_node::ExecutorConfig::try_resolve(baked)
    }
}

/// `$NROS_ENTRY_SPIN_MS` — the bounded-spin budget for a native entry, `0` for
/// "run until the process is signalled".
///
/// issue 0687 — one reader. The two native entries each parsed this variable
/// themselves, at different widths (`u32` and `u64`), which is how the same
/// name comes to mean two things: the `u32` site silently fell back to
/// "unbounded" for any value above 4.29e9 ms while the `u64` site accepted it.
/// Both take the same answer now, and the entry that wants 32 bits saturates
/// rather than wrapping to a value nobody asked for.
#[cfg(feature = "env")]
fn entry_spin_ms() -> u64 {
    std::env::var("NROS_ENTRY_SPIN_MS")
        .ok()
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or(0)
}

// ── Serialization format (RFC-0088 D5) ──────────────────────────────────
//
// The discriminant of the format the linked backend speaks, lowered by
// `cbindgen` into `nros_cpp_ffi.h` as
// `#define NROS_CPP_SERIALIZATION_FORMAT_ID`. `nros/serialization_format.hpp`
// lifts it into `nros::SerializationFormat` and asserts every typed entity's
// message type against it.
//
// It is emitted HERE, into the C++ crate's own FFI header, rather than reused
// from `nros-c`'s `NROS_SERIALIZATION_FORMAT_ID`, so that no C++ header has to
// include a C API header to get it. Reaching `nros/nros_generated.h` from a
// generated message header would put it BEFORE `nros_cpp_ffi.h` in include
// order, and that order is one-way (issue 0160: the FFI struct mirrors).
//
// The literal is a MIRROR — `cbindgen` runs with `parse_deps = false` and
// evaluates no expressions — and the `const _` below is what makes it true:
// it compares against `nros_node::session::IMAGE_SERIALIZATION_FORMAT_ID`, the
// same constant `nros_node::format_check` asserts on. A backend whose format is
// not CDR fails this crate's build naming the drift, instead of shipping a
// header that quietly disagrees with the backend the image links.
//
// **Only meaningful in a single-backend image** (RFC-0088 D5). A bridge image
// links two backends and has no single answer; it asks per session, with
// `nros::Node::serialization_format()`. `scripts/check-format-macro-scope.py`
// refuses a bridge-linked translation unit that references the macro.

/// Image-local discriminant of the linked backend's serialization format
/// (`nros_serdes::format::SerializationFormatId`). RFC-0088 D2 — image-local:
/// never persist it, never compare it across images.
pub const NROS_CPP_SERIALIZATION_FORMAT_ID: u8 = 1;

/// Cross-image identity of the linked backend's serialization format.
/// Not lowered to C++ (cbindgen maps no Rust `&str` to a C constant);
/// `nros::linked_format_name()` derives the name from the discriminant.
pub const NROS_CPP_SERIALIZATION_FORMAT: &str = "cdr";

// GATED on `rmw-cffi`, for the same reason and by the same rule as the sibling
// block in `nros-c/src/constants.rs` — this is the SECOND site of that defect,
// and the first fix landed only on the one the error message named.
// `nros_node::session` is `#[cfg(any(has_rmw, test))]`, `has_rmw` is exactly
// nros-node's `rmw-cffi` feature, and nros-cpp's own `rmw-cffi` forwards to it
// (`nros/rmw-cffi` + `nros-c/rmw-cffi`). Ungated, the block named a module that
// exists in no RMW-less combo, so the crate failed to COMPILE there.
#[cfg(feature = "rmw-cffi")]
const _: () = {
    assert!(
        NROS_CPP_SERIALIZATION_FORMAT_ID
            == nros_node::session::IMAGE_SERIALIZATION_FORMAT_ID.as_u8(),
        "RFC-0088: NROS_CPP_SERIALIZATION_FORMAT_ID no longer matches the linked \
         backend — update the mirror (and the C++ headers that assert on it)"
    );
    let mirrored = NROS_CPP_SERIALIZATION_FORMAT.as_bytes();
    let linked = nros_node::session::IMAGE_SERIALIZATION_FORMAT.as_bytes();
    assert!(
        mirrored.len() == linked.len(),
        "RFC-0088: NROS_CPP_SERIALIZATION_FORMAT no longer matches the linked backend"
    );
    let mut i = 0;
    while i < mirrored.len() {
        assert!(
            mirrored[i] == linked[i],
            "RFC-0088: NROS_CPP_SERIALIZATION_FORMAT no longer matches the linked backend"
        );
        i += 1;
    }
};
