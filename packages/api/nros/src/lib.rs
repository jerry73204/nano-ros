//! # nros
//!
//! A lightweight ROS 2 client library for embedded systems.
//!
//! This crate provides a unified API for building ROS 2 nodes in Rust,
//! with support for `no_std` environments and embedded targets.
//!
//! ## Features
//!
//! - **no_std compatible**: Works on bare-metal and RTOS targets
//! - **Zero-copy where possible**: Minimizes memory allocations
//! - **Type-safe**: Compile-time verification of message types
//! - **ROS 2 compatible**: Interoperates with standard ROS 2 nodes via rmw_zenoh
//!
//! ## Quick Start
//!
//! ```ignore
//! use nros::prelude::*;
//! use std_msgs::msg::Int32;
//!
//! let config = ExecutorConfig::from_env().node_name("my_node");
//! let mut executor = Executor::open(&config)?;
//!
//! let node = executor.node_builder("my_node").build()?;
//! let publisher = executor.node_mut(node).create_publisher::<Int32>("/my_topic")?;
//! publisher.publish(&Int32 { data: 42 })?;
//!
//! executor.node_mut(node).create_subscription::<Int32, _>("/topic", |msg: &Int32| {
//!     println!("Received: {}", msg.data);
//! })?;
//!
//! executor.spin_blocking(SpinOptions::default());
//! ```
//!
//! ## Executor Sizing
//!
//! The executor's static memory layout is controlled via environment variables
//! at build time:
//!
//! - **`NROS_EXECUTOR_MAX_CBS`** (default 4) — maximum number of registered
//!   callbacks (subscriptions + timers + services + guard conditions).
//! - **`NROS_EXECUTOR_ARENA_SIZE`** (default 4096) — byte budget for storing
//!   callback closures inline.
//!
//! For messages larger than the default 1024-byte receive buffer, size the
//! subscription via the builder's `.rx_buffer::<N>()` knob (e.g.
//! `node_mut(id).subscription(t).typed::<M>().rx_buffer::<4096>().build(cb)`).
//!
//! ## Transport Backends
//!
//! Phase 248 C5c — `nros` is RMW- and platform-AGNOSTIC. It carries only the
//! `rmw-cffi` vtable; the concrete backend (zenoh / xrce / cyclonedds) enters
//! the link graph via the board crate (embedded), the board-less app's own
//! `nros-rmw-*` dep (native), or the `nros-c`/`nros-cpp` staticlib root (D3),
//! and self-registers through the `RMW_INIT_ENTRIES` walker at `Executor::open`.
//! The concrete session type is resolved automatically; advanced users can
//! access it via `nros::internals::RmwSession`.
//!
//! ## Crate Features
//!
//! `nros` exposes only FUNCTIONAL features — `std`/`alloc`, the `rmw-cffi`
//! vtable, `lending`, `bridge`/`config`, `param-services`,
//! `lifecycle-services`, `safety-e2e`, `stream`, `ffi-sync`, and the ROS
//! edition (`ros-humble`/`ros-iron`). There are NO `platform-*` or concrete
//! `rmw-*` selector features (Phase 248 C7). Platform + RMW are selected by the
//! board / staticlib root via dependencies, not `nros` features. The
//! `zephyr_component_main!` entry macro is gated only on `rmw-cffi` (it's
//! framework entry codegen, like `nros::main!`), not a platform feature.
//!
//! **ROS edition** (select one; RFC-0056 — the per-distro interop profile):
//! - `ros-humble` - ROS 2 Humble (default; `TypeHashNotSupported`, XCDR1)
//! - `ros-iron` - ROS 2 Iron (RIHS01 type hash)
//! - `ros-jazzy` - ROS 2 Jazzy (RIHS01; XCDR2/appendable is phase-303)
//!
//! **Other**:
//! - `std` (default) - Enable standard library support
//! - `alloc` - Enable heap allocation without full std
//!
//! ## Further Reading
//!
//! - [`guide`] — tutorials: getting started, services, configuration,
//!   ROS 2 interop, and troubleshooting
//! - [Message Generation](https://github.com/jerry73204/nano-ros/blob/main/docs/guides/message-generation.md)
//!   — codegen reference (all options, output structure, bundled interfaces)
//! - [Environment Variables](https://github.com/jerry73204/nano-ros/blob/main/docs/reference/environment-variables.md)
//!   — complete buffer tuning reference
//! - [ROS 2 Interop](https://github.com/jerry73204/nano-ros/blob/main/docs/reference/rmw_zenoh_interop.md)
//!   — protocol details (key expressions, liveliness, attachments)
//! - [Examples](https://github.com/jerry73204/nano-ros/tree/main/examples)
//!   — working examples by platform (native, QEMU, ESP32, Zephyr)

#![no_std]

// ── Feature validation (mutual exclusivity) ─────────────────────────────
// Phase 248 C5c/C7 — `nros` carries NO `platform-*` selector features, so the
// platform mutual-exclusion `compile_error!` is gone. The platform is selected
// by the board / staticlib root via an `nros-platform` dep, and nros-node picks
// the kernel primitive at runtime (C2 wake-probe).
// Only `rmw-cffi` is exposed at this layer; the cffi shim selects the
// concrete backend at the C ABI level via the `RMW_INIT_ENTRIES` walker.

// At most one ROS edition (RFC-0056 — the axis is compile-time exclusive).
#[cfg(any(
    all(
        feature = "ros-humble",
        any(feature = "ros-iron", feature = "ros-jazzy")
    ),
    all(feature = "ros-iron", feature = "ros-jazzy"),
))]
compile_error!("`ros-{humble,iron,jazzy}` are mutually exclusive — select one ROS edition.");

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

// Phase 216.A.5 — the `nros::node!()` proc-macro emits absolute paths
// under `::nros::*` (so downstream Node pkgs only need a single `nros`
// dep). For the in-crate macro-expansion test in `node.rs`, alias the
// `nros` crate name to itself so those absolute paths resolve. Gated on
// `cfg(test)` to keep the alias out of normal builds.
#[cfg(test)]
extern crate self as nros;

// Phase 248 C5c — the umbrella's force-link statics
// (`__FORCE_LINK_{PLATFORM_CFFI,ZENOH,XRCE,CYCLONEDDS_SYS}`) are REMOVED along
// with `nros`'s concrete-backend deps. `nros` no longer references any concrete
// RMW or platform crate, so it has nothing to force-link. Registration + the
// `nros_platform_*` link anchor now live with whoever owns the concrete crate:
//   * embedded   — the BOARD crate force-links its backend + calls
//     `<backend>::register()` in its boot path (C5a);
//   * board-less native — the APP owns `nros-rmw-*` + a `#[used]` force-link in
//     its `main.rs`, and `nros-platform-cffi[posix-c-port]` anchors the C symbols;
//   * C/C++ staticlib — `nros-c`/`nros-cpp` bundle one backend (D3) and anchor
//     `nros-platform` themselves.

// Phase 249 P1 — `__register_linked_rmw()` (a Phase 248 C5c no-op kept only so the
// `nros::main!` framework's call sites compiled) is REMOVED along with those call
// sites. Backend registration never routed through the backend-agnostic `nros` crate:
// hosted auto-registers via the `RMW_INIT_ENTRIES` walk at `Executor::open`; embedded
// boards perform the explicit `<backend>::register()` in their boot path (C5a). One
// Rust trigger = the board/app explicit register (phase-249).

// phase-391 W5 — build-time knobs (`MAX_COMPONENTS`, `COMPONENT_SLOT_BYTES`).
// Ungated: they are plain consts, useful to size caller-supplied storage
// whether or not the runtime module that consumes them is compiled in.
pub mod config;

// phase-391 W5 — caller-supplied component-pool storage sizing. Ungated for the
// same reason `config` is: plain arithmetic, useful to size a `static` whether
// or not `node_runtime` is compiled in.
pub mod dispatch_tag;
pub mod guide;
#[cfg(feature = "metadata-mode")]
pub mod metadata_mode;
pub mod node;
pub mod node_metadata;
/// Phase 212.M.5.a.2 — executor-backed component runtime.
///
/// Binds [`Node`] / [`ExecutableNode`] to a live
/// [`Executor`] so a Node pkg can actually run (versus
/// [`MetadataRecorder`](node_metadata::MetadataRecorder) which
/// is the planner-side metadata sink).
///
/// Gated on `rmw-cffi`; the underlying [`Executor`] is only present
/// when an RMW backend is linked. W5-endgame (issue 0843): the MACRO install
/// path (per-class static storage, placed cells, slabbed ctxs) is alloc-free,
/// so the module no longer demands `alloc` — only the dynamic
/// `ExecutorNodeRuntime` half does, item-gated inside.
#[cfg(feature = "rmw-cffi")]
pub mod node_runtime;
pub mod runtime_storage;

/// Phase 212.L.5 — top-level init API.
///
/// Re-exported flat at the crate root: `nros::init()`,
/// `nros::init_with_launch_auto()`, `nros::init_with_launch(path)`,
/// `nros::init_with_args(args)`, `nros::Context`, `nros::InitError`.
#[cfg(feature = "env")]
pub mod init;

/// issue 0687 — the hosted edge of configuration: every env var nano-ros
/// honours is read here, and the core takes values.
#[cfg(feature = "env")]
pub mod env;

#[cfg(feature = "env")]
pub use env::{ExecutorConfigEnvExt, rmw_selector};

#[cfg(feature = "env")]
pub use init::{
    Context, ContextSource, InitError, init, init_with_args, init_with_launch,
    init_with_launch_auto,
};

/// Compile-time opaque storage sizes for FFI consumers.
///
/// See [`sizes`] for the `export_size!` pattern used to expose these values
/// to `nros-c` / `nros-cpp` at build time.
pub mod sizes;

/// Monotonic time for portable node code (issue #504).
///
/// phase-359 W10 — `rmw-cffi`, not `any(std, rmw-cffi)`: the clock is the
/// platform port's, and a build with no port has none to offer.
#[cfg(feature = "rmw-cffi")]
pub mod time;

/// CDR encapsulation constants and helpers for FFI layers that handle raw
/// CDR bytes (e.g. nros-c, nros-cpp action and service paths).
pub mod cdr {
    pub use nros_serdes::{
        CDR_BE_HEADER, CDR_HEADER_LEN, CDR_LE_HEADER, strip_cdr_header, write_cdr_le_header,
    };
}

// Re-export core types
pub use nros_core::{
    CdrReader, CdrWriter, Clock, ClockType, DeserError, Deserialize, Duration, MessageInfo,
    PUBLISHER_GID_SIZE, RawMessageInfo, RosMessage, RosService, SerError, Serialize, Time,
};

// -----------------------------------------------------------------------------
// Logging (phase-417 W4.d, issue 0589).
//
// `nros_log` was in this crate's dependency graph and reachable from NONE of
// its surface: no `pub use`, nothing in the prelude. So from the façade the
// shortest path to "print something" was `std::println!` — which is FATAL on
// Zephyr `native_sim`: `zvfs_write(1, …)` dispatches to
// `stdinout_write_vmeth`, which calls `zvfs_write(1, …)` again, and because
// `k_mutex` is recursive it exhausts the stack instead of deadlocking. The
// image dies with no message. A logging facade nobody can reach from the
// umbrella is not a facade.
//
// `nros::Logger` IS RE-BOUND, and that is the point rather than a side effect.
// It used to name `nros_core::logger::Logger`, which formats through the `log`
// crate: on a target with no `log` backend installed — every embedded one —
// the record is constructed and dropped, and it NEVER reaches
// `nros_platform_log_write`. The type the C and C++ FFI hands out, the type
// the `nros_*!` macros dispatch through, and the type the platform sink
// receives is `nros_log::Logger`. One name, one meaning.
//
// Migration: the two types share `new` and `name` and NOTHING else — the old
// one has inherent `.info(&str)` / `.warn(&str)` / `*_once` / `*_throttle`
// methods, the new one is driven by the `nros_*!` macros. So every call site
// that used the old surface FAILS TO COMPILE against the new binding rather
// than silently changing where its records go. That is the RFC-0089 rule
// applied to our own rename, and it is why this is a re-binding with a named
// escape hatch instead of a quiet swap. The escape hatch is
// [`LogCrateLogger`], deprecated on arrival.
pub use nros_log::{
    DEFAULT_LOGGER, LogSink, Logger, MAX_LOGGERS, Record as LogRecord, Severity, ThrottleState,
    get_logger, get_or_create_logger, register_logger,
};
// The emission macros, at the crate root, because a user who has
// `use nros::prelude::*` should not have to learn a second crate name to log a
// line. `#[macro_export]` puts them at `nros_log`'s root; this puts them at
// ours.
//
// NAMES (phase-417, RFC-0089 "each language follows ITS OWN upstream",
// settled 2026-09-04). The three ROS 2 client libraries disagree about how to
// spell a log line — `rclrs::log_info!`, `RCLCPP_INFO`,
// `RCUTILS_LOG_INFO_NAMED` — so there is no one "ROS 2 spelling" to match.
// Rust follows rclrs, because a node ported from rclrs is read beside rclrs.
// Hence the five severity macros below carry rclrs's names.
//
// Two families keep OUR prefix, and that is a decision rather than an
// oversight: a name takes rclrs's spelling exactly when rclrs HAS that name.
//
//   * `nros_trace!` — rclrs stops at `debug`. TRACE has no upstream twin, so
//     giving it one would claim a correspondence that does not exist.
//   * `nros_*_throttle!` / `nros_*_throttle_at!` — rclrs throttles with a
//     MODIFIER on one macro (`log_info!(logger.throttle(d), "…")` via
//     `LogParams` / `ToLogParams`), not a macro per severity. Adopting that is
//     a shape change, not a rename; see `nros-log/src/macros.rs`'s throttle
//     note for the three ledger rows it would have to reverse.
pub use nros_log::{
    log_debug, log_error, log_fatal, log_info, log_warn, nros_debug_throttle,
    nros_debug_throttle_at, nros_error_throttle, nros_error_throttle_at, nros_fatal_throttle,
    nros_fatal_throttle_at, nros_info_throttle, nros_info_throttle_at, nros_trace,
    nros_trace_throttle, nros_trace_throttle_at, nros_warn_throttle, nros_warn_throttle_at,
};
/// The whole logging facade — sinks, the early-record ring, the throttle
/// primitives, `init`/`flush`/`add_sink`.
///
/// The root re-exports above are the surface a node author needs; this is
/// everything, for a board or a bridge composing its own delivery.
pub mod logging {
    pub use nros_log::*;
}

/// The pre-phase-417 `nros::Logger`: `nros_core::logger::Logger`, which
/// forwards to the `log` crate.
///
/// Deprecated on arrival. It exists so a downstream call site pinned to the old
/// method surface has a one-word fix and a deadline, not so anyone reaches for
/// it: on any target without a `log` backend installed — which is every
/// embedded one — its records are formatted and dropped before reaching
/// `nros_platform_log_write`. Move to [`Logger`] and the `nros_*!` macros.
#[deprecated(
    since = "0.5.0",
    note = "`nros::Logger` now means `nros_log::Logger`, the logger that reaches the platform \
            sink. This alias is the `log`-crate-backed type it used to mean; its records are \
            dropped on any target with no `log` backend. Use `nros::Logger` + the `log_info!` \
            family."
)]
pub type LogCrateLogger<'a> = nros_core::Logger<'a>;

/// Companion to [`LogCrateLogger`] — see its note.
#[deprecated(
    since = "0.5.0",
    note = "the once-flag belongs to the deprecated `log`-crate logger; `nros_log` throttling is \
            `nros::ThrottleState` + the `nros_*_throttle!` macros."
)]
pub type LogCrateOnceFlag = nros_core::OnceFlag;

// Re-export heapless for generated message types and examples
pub use nros_core::heapless;

// Re-export component-mode API
#[cfg(feature = "rmw-cffi")]
pub use node::NodeExecutorRuntime;
// Phase 212.M.5.a.2 — executor-backed runtime entry points.
// (`component_register_symbol` retired in the Phase 212.N.7 closing
// sweep — the helper had no live callers after the BSP baker + macro
// extern emit were deleted.)
pub use node::{
    ActionExecutor, Callback, CallbackCtx, CallbackEffects, ClientDispatch, DeclaredNode,
    DeclaredNodeRuntime, EntityBounds, ExecutableNode, MISSING_NODE_EXPORT_ERROR, Node,
    NodeActionClient, NodeActionServer, NodeContext, NodeDeclError, NodeOptions, NodeParameter,
    NodePublisher, NodeResult, NodeRuntime, NodeRuntimeAdapter, NodeServiceClient,
    NodeServiceServer, NodeSubscription, NodeTimer, PublisherResolver, RuntimeNodeRecord, TickCtx,
    record_node_metadata, register_node,
};
// Phase 212.M.5.a.4 — internal helper consumed by `nros::node!()`
// for the BSP dispatch path. Public-but-doc-hidden so the macro expand
// resolves it as `::nros::__private_node_state_into_raw`.
#[cfg(feature = "alloc")]
#[doc(hidden)]
pub use node::__private_node_state_into_raw;
// phase-359 W8 — follows `node_metadata`'s re-gate: the type needs `alloc`,
// not `std`.
#[cfg(feature = "alloc")]
pub use node_metadata::SourceMetadataExport;
pub use node_metadata::{
    CallbackEffectKind, CallbackEffectMetadata, EntityKind, EntityMetadata, MetadataRecorder,
    MetadataString, NodeMetadata, NodeMetadataError, ParameterDefault, SourceLocationMetadata,
    SourceNameKind,
};
#[doc(hidden)]
pub use node_metadata::{CallbackId, EntityId, NodeId};
// Phase 216.A.4 — opaque tag types Node authors hold on `Self::State`
// and match against the `Callback<'_>` delivered to
// `ExecutableNode::on_callback`.
pub use dispatch_tag::{ActionTag, ServiceTag, SubscriptionTag};
// W5-endgame (issue 0843) — the alloc-free half of the seam: per-class static
// storage + the `_in` installs the macro emits. Available on `rmw-cffi` alone.
#[cfg(feature = "rmw-cffi")]
pub use node_runtime::{
    ComponentSlotStorage,
    // Phase 257 (W0-B) — the uniform cross-language component-install seam backing
    // `__nros_component_<pkg>_install` (nros::node!): register an ExecutableNode on the
    // shared executor a foreign typed entry hands in. (`register_node_borrowed` stays
    // crate-internal — it returns the private `ComponentCell`.)
    install_node_typed_in,
    // Phase 305 W3 (issue 0255) — same seam plus launch `<remap>` rules; the variant
    // `nros::node!()` emits.
    install_node_typed_with_launch_in,
};
// The dynamic runtime + the leak-per-call conveniences still need `alloc`.
#[cfg(all(feature = "rmw-cffi", feature = "alloc"))]
pub use node_runtime::{
    ExecutorError,
    ExecutorNodeRuntime,
    RegisteredNode,
    install_node_typed,
    install_node_typed_with_launch,
    // Phase 268 W1 — same seam with both `<param>` initials AND `<node name= namespace=>`
    // identity injection (RFC-0046).
    install_node_typed_with_node_identity,
    // W4a — same seam, seeding the node's NodeContext with launch-baked `<param>` initials.
    install_node_typed_with_params,
};

/// Phase 257 (W0-B) — `install_node_typed` stub for builds without the cffi runtime.
/// The typed-entry install seam needs the `rmw-cffi` executor; a `nros::node!()` pkg
/// compiled without `rmw-cffi` still emits `__nros_component_<pkg>_install` (the macro
/// can't see the umbrella's feature), so this stub keeps it linkable — it returns `-1`
/// (no real executor to install on). The real impl is `node_runtime::install_node_typed`.
///
/// # Safety
/// Signature parity with the real impl; the stub dereferences nothing.
#[cfg(not(feature = "rmw-cffi"))]
#[doc(hidden)]
pub unsafe fn install_node_typed<C: node::ExecutableNode + 'static>(
    _executor: *mut core::ffi::c_void,
) -> i32
where
    C::State: 'static,
{
    -1
}

/// W4a — `install_node_typed_with_params` stub for builds without the cffi runtime.
/// Signature parity with `node_runtime::install_node_typed_with_params`; returns `-1`.
///
/// # Safety
/// The stub dereferences nothing.
#[cfg(not(feature = "rmw-cffi"))]
#[doc(hidden)]
pub unsafe fn install_node_typed_with_params<C: node::ExecutableNode + 'static>(
    _executor: *mut core::ffi::c_void,
    _params: &[(&str, &str)],
) -> i32
where
    C::State: 'static,
{
    -1
}

/// Phase 268 W1 — `install_node_typed_with_node_identity` stub for builds without the
/// cffi runtime. Signature parity with the real impl; returns `-1`.
///
/// # Safety
/// The stub dereferences nothing.
#[cfg(not(feature = "rmw-cffi"))]
#[doc(hidden)]
pub unsafe fn install_node_typed_with_node_identity<C: node::ExecutableNode + 'static>(
    _executor: *mut core::ffi::c_void,
    _params: &[(&str, &str)],
    _node_identity: Option<(&'static str, &'static str)>,
) -> i32
where
    C::State: 'static,
{
    -1
}

/// phase-391 W5.3b — `ComponentSlotStorage` stub for builds without the cffi
/// runtime (or without `alloc`): the macro emits a per-class
/// `static ... = ComponentSlotStorage::new()` unconditionally, so the name must
/// exist and be const-constructible + `Sync` in every cfg. Zero-sized.
#[cfg(not(feature = "rmw-cffi"))]
#[doc(hidden)]
pub struct ComponentSlotStorage<
    C,
    const N: usize = { crate::config::MAX_CLASS_INSTANCES },
    const PUBS: usize = { crate::config::MAX_CELL_ENTITIES },
    const SVCS: usize = { crate::config::MAX_CELL_ENTITIES },
    const ACTC: usize = { crate::config::MAX_CELL_ENTITIES },
    const ACTS: usize = { crate::config::MAX_CELL_ENTITIES },
    const SSRV: usize = { crate::config::MAX_CELL_ENTITIES },
> {
    _p: core::marker::PhantomData<fn() -> C>,
}

#[cfg(not(feature = "rmw-cffi"))]
impl<C, const N: usize, const PUBS: usize, const SVCS: usize, const ACTC: usize, const ACTS: usize>
    ComponentSlotStorage<C, N, PUBS, SVCS, ACTC, ACTS>
{
    #[doc(hidden)]
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            _p: core::marker::PhantomData,
        }
    }
}

/// phase-391 W5.3b — `install_node_typed_in` stub; returns `-1`.
///
/// # Safety
/// Signature parity with the real impl; the stub dereferences nothing.
#[cfg(not(feature = "rmw-cffi"))]
#[doc(hidden)]
pub unsafe fn install_node_typed_in<
    C: node::ExecutableNode + 'static,
    const N: usize,
    const PUBS: usize,
    const SVCS: usize,
    const ACTC: usize,
    const ACTS: usize,
    const SSRV: usize,
>(
    _executor: *mut core::ffi::c_void,
    _store: &'static ComponentSlotStorage<C, N, PUBS, SVCS, ACTC, ACTS, SSRV>,
) -> i32
where
    C::State: 'static,
{
    -1
}

/// phase-391 W5.3b — `install_node_typed_with_launch_in` stub; returns `-1`.
///
/// # Safety
/// Signature parity with the real impl; the stub dereferences nothing.
#[cfg(not(feature = "rmw-cffi"))]
#[doc(hidden)]
pub unsafe fn install_node_typed_with_launch_in<
    C: node::ExecutableNode + 'static,
    const N: usize,
    const PUBS: usize,
    const SVCS: usize,
    const ACTC: usize,
    const ACTS: usize,
    const SSRV: usize,
>(
    _executor: *mut core::ffi::c_void,
    _store: &'static ComponentSlotStorage<C, N, PUBS, SVCS, ACTC, ACTS, SSRV>,
    _params: &[(&str, &str)],
    _node_identity: Option<(&'static str, &'static str)>,
    _remaps: &[(&str, &str)],
    // Spelled as the plain tuple rather than `QoSOverrideCode`: that alias
    // lives behind nros-node's cffi gate and is unnameable in this cfg, but
    // its definition is this tuple — the same spelling `RuntimeCtx` uses, so
    // emitted calls typecheck identically against stub and real. (The OLDER
    // with_launch stub above simply dropped this parameter, which is 4-vs-5
    // signature drift against its real impl — not repeated here.)
    _qos_overrides: &'static [(&'static str, u8, u8, u32)],
) -> i32
where
    C::State: 'static,
{
    -1
}

/// Phase 305 W3 (issue 0255) — `install_node_typed_with_launch` stub for builds
/// without the cffi runtime. Signature parity with the real impl; returns `-1`.
///
/// # Safety
/// The stub dereferences nothing.
#[cfg(not(feature = "rmw-cffi"))]
#[doc(hidden)]
pub unsafe fn install_node_typed_with_launch<C: node::ExecutableNode + 'static>(
    _executor: *mut core::ffi::c_void,
    _params: &[(&str, &str)],
    _node_identity: Option<(&'static str, &'static str)>,
    _remaps: &[(&str, &str)],
) -> i32
where
    C::State: 'static,
{
    -1
}
// Phase 212.N.12 — canonical `nros::node!()` macro. Replaces the legacy
// `nros::node!()` macro (retired in the N.12 hard rename — both the
// proc-macro forwarder and the Cargo metadata key are gone).
#[cfg(feature = "macros")]
pub use nros_macros::node;
// Phase 212.N.9 — `nros::main!()` proc-macro family. One-line Entry-pkg
// `main.rs` (replaces the legacy `build.rs + include!()` shape). See
// `docs/design/0024-multi-node-workspace-layout.md` §11.6.
#[cfg(feature = "macros")]
pub use nros_macros::main;

/// Route this image's panics to its platform — phase-366 W5.c / RFC-0077.
///
/// Emits the `#[panic_handler]` for an embedded image, forwarding the message
/// to `nros_platform_panic` so a Rust panic ends the same way a C precondition
/// failure or a C++ terminate does. What that ending IS belongs to the port:
/// `k_panic()` on Zephyr, `esp_system_abort()` on ESP-IDF, UART-then-exit-QEMU
/// on the ThreadX RV64 board.
///
/// # Why this is a macro you INVOKE, not something `nros::main!` emits
///
/// `#[panic_handler]` is a singleton of the final artifact, and the image owns
/// it. Emitting one silently from `nros::main!()` would collide with every image
/// that already declares its own — `examples/qemu-esp32-baremetal` writes
/// `use esp_backtrace as _;`, and `logging-smoke-freertos-mps2` uses
/// `panic-semihosting` with `features = ["exit"]` so a panic exits QEMU instead
/// of hanging the test harness. Those images are RIGHT, and an invisible default
/// would fight them.
///
/// So the line is written in the entry, where it can be read, swapped for
/// `use panic_halt as _;`, or replaced by a hand-written handler that logs to
/// NVM and reboots. A default you cannot see is a constraint, not a default.
///
/// # Use — `main!(panic = …)` is the normal way; this is the escape hatch
///
/// phase-366 R3. An entry that goes through `nros::main!()` should say
/// `panic = "platform"` (or nothing — that is the default since M5) and let the
/// macro emit this body. One line, and the build can check it.
///
/// ```ignore
/// #![no_std]
/// nros::main!();            // ends through `nros_platform_panic`
/// ```
///
/// Invoke this macro directly only where no `main!()` expansion can carry the
/// item — a hand-rolled `no_std` binary, or the lib side of a crate whose
/// `crate-type` includes `staticlib` and whose entry macro is
/// `zephyr_component_main!` / a board `app_main!`. It is kept for exactly those:
/// deleting it would strand the images that cannot use the macro replacing it.
///
/// ```ignore
/// // src/app_main.rs — the .a is a final artifact to rustc, and `main!()` in
/// // the bin target never reaches it.
/// nros::panic_to_platform!();
/// ```
///
/// Do NOT invoke it in a `std` image: libstd supplies the lang item there and a
/// second one does not compile. Do not invoke it alongside `use panic_halt as _`
/// or any other provider, for the same reason — that is the duplicate
/// `check-archive-lang-items` exists to catch.
/// Park the core on panic — the `halt` value of `nros::main!(panic = …)`.
///
/// For an image that must not print: no formatting, no allocation, no call out
/// to the platform. Interrupts are masked first so the parked core cannot be
/// woken back into a half-dead system by a timer or a driver ISR still armed
/// from before the panic.
///
/// This is a body rather than a re-export of the `panic-halt` crate so that
/// choosing it costs the entry no new dependency — `main!(panic = "halt")` is a
/// word in a macro the crate already calls, which is the point of the surface.
/// The behaviour is the same: mask, then spin forever.
///
/// Prefer `panic = "platform"`. Halting discards the diagnosis, and every port
/// implements `nros_platform_panic` precisely so a dying image can say why.
/// Phase 392 W3b — the receive-buffer size for a message type, as a constant
/// that cannot drift from the type.
///
/// ```ignore
/// node.subscription::<PointCloud2>("points")
///     .rx_buffer::<{ nros::rx_buffer_for!(PointCloud2) }>()
///     .build(on_cloud)?;
/// ```
///
/// `.rx_buffer::<N>()` has always accepted a number. The problem is where the
/// number comes from: a literal is correct until someone appends a field to the
/// message, and then it is silently too small — the sample is received, ACKed
/// and dropped at the transport, which needs a packet capture to attribute
/// (`report_dropped_take`, and the 13.4 KiB Autoware trajectory case). This
/// expands to the type's own bound, computed from its schema by phase 380, so
/// appending a field moves the buffer with it.
///
/// **Why a macro and not a method.** The builder cannot do this for you:
/// inside `impl<M> TypedSubscriptionBuilder<M>` the type is a generic
/// parameter, and on stable Rust a generic parameter may not appear in a const
/// operation — `error: generic parameters may not be used in const operations`.
/// At a call site the type is CONCRETE, which is legal, so the size has to be
/// named where the type is. That constraint is also why phase-392's size
/// classes were "decoupled from codegen" in the first place.
///
/// **Unbounded types are a BUILD ERROR** (phase-403 W0). A type with an
/// unbounded `string`/`wstring`/`sequence` has no bound, and this refuses to
/// invent one. It used to expand to `DEFAULT_RX_BUF_SIZE`; it now fails to
/// compile, naming both remedies.
///
/// Every message type is REQUIRED to carry a derived upper bound, stated in the
/// `.msg` (`string<=64`) or capped in `nros-codegen.toml`. Phase 380 is explicit
/// that `None` means "no bound EXISTS", never "unknown", and that a buffer must
/// not be sized from a fallback. Substituting the configured default was the
/// violation of that rule; refusing is what it licenses. `report_dropped_take`
/// is a backstop for a buffer that is too small, not a licence to pick one.
///
/// ```compile_fail
/// use nros_serdes::schema::{Field, FieldType, Message};
/// struct Unbounded;
/// impl Message for Unbounded {
///     const TYPE_NAME: &'static str = "test/msg/Unbounded";
///     const FIELDS: &'static [Field] = &[Field {
///         name: "s",
///         ty: FieldType::String,
///         offset: 0,
///     }];
/// }
/// let _n: usize = nros::rx_buffer_for!(Unbounded);
/// ```
///
/// The positive control for that `compile_fail`, so it cannot pass because the
/// fixture stopped compiling for an unrelated reason:
///
/// ```
/// use nros_serdes::schema::{Field, FieldType, Message};
/// struct Bounded;
/// impl Message for Bounded {
///     const TYPE_NAME: &'static str = "test/msg/Bounded";
///     const FIELDS: &'static [Field] = &[Field {
///         name: "a",
///         ty: FieldType::Uint64,
///         offset: 0,
///     }];
/// }
/// let n: usize = nros::rx_buffer_for!(Bounded);
/// assert!(n > 0);
/// ```
///
/// **Why the `const` block.** The macro is used in two positions: as a
/// const-generic argument (`.rx_buffer::<{ rx_buffer_for!(M) }>()`) and as a
/// plain expression (`let n = rx_buffer_for!(M);`). A bare `panic!` is a build
/// error in the first and a RUNTIME panic in the second, which would make the
/// rule depend on where the macro appears. Wrapping the whole match in an inline
/// `const` block forces compile-time evaluation in BOTH, so an unbounded type
/// can never reach a running image.
///
/// **What the error names.** rustc points at this macro invocation, where the
/// type is written literally, so the TYPE is named. The MEMBER that costs the
/// bound cannot be in the message: const evaluation does not format, so a
/// `panic!` there takes a literal only. The member is named by the codegen
/// diagnostic for the same type -- `unbounded_reason` in the generated C header
/// (`packs/c/message.h.jinja`), which names EVERY member that costs the bound in
/// one build (phase-403 W0), or `nros_serdes::size::visit_unbounded` over its
/// `FIELDS`.
#[macro_export]
macro_rules! rx_buffer_for {
    ($msg:ty) => {
        // The `const` block is load-bearing; see "Why the `const` block" above.
        const {
            match $crate::__rx_bound::<$msg>() {
                ::core::option::Option::Some(n) => n,
                ::core::option::Option::None => ::core::panic!(
                    "nros: this message type has NO serialized-size bound, so no \
                     receive buffer can be sized from it.\n\
                     Every message type must carry a derived upper bound. Bound the \
                     member that costs it, either:\n\
                     \x20 - in the `.msg`: `string<=64`, `wstring<=64`, \
                     `sequence<T, N>`, `T[<=N]`; or\n\
                     \x20 - as an INLINE `cap` in `nros-codegen.toml`: under \
                     `[fields]`, `\"pkg/Msg.field\" = 64`. A `heap` or `view` cap \
                     is a sizing hint that nothing enforces (RFC-0033), so it \
                     deliberately does NOT bound.\n\
                     WHICH members cost the bound -- all of them, not just the \
                     first -- are named by the codegen diagnostic for this same \
                     type: `unbounded_reason` in the generated C header, or \
                     `nros_serdes::size::visit_unbounded` over its `FIELDS`. The type itself is named by the \
                     `rx_buffer_for!` invocation rustc points at.\n\
                     Phase 380: `None` means no bound EXISTS, never \"unknown\", and \
                     a buffer sized from a fallback is the failure that rule was \
                     written to prevent. Erroring honours it; defaulting to \
                     `DEFAULT_RX_BUF_SIZE` did not."
                ),
            }
        }
    };
}

#[macro_export]
macro_rules! panic_halt {
    () => {
        #[panic_handler]
        fn __nros_panic_halt(_info: &::core::panic::PanicInfo) -> ! {
            // Mask interrupts through the platform's critical section, which is
            // the one IRQ primitive that is portable across the ports (the
            // `cortex_m`/`riscv` intrinsics are not). Entering and never
            // leaving is deliberate.
            unsafe extern "C" {
                fn nros_platform_critical_section_acquire() -> u32;
            }
            // SAFETY: the ABI's acquire takes no argument and returns a restore
            // token we deliberately drop — nothing after this point runs.
            unsafe {
                let _ = nros_platform_critical_section_acquire();
            }
            loop {
                ::core::hint::spin_loop();
            }
        }
    };
}

#[macro_export]
macro_rules! panic_to_platform {
    () => {
        #[panic_handler]
        fn __nros_panic(info: &::core::panic::PanicInfo) -> ! {
            use ::core::fmt::Write as _;

            // Fixed buffer, never the heap: this runs when the allocator may be
            // exactly what failed. Truncation is deliberate — a short panic line
            // still diagnoses; a missing one does not.
            struct Buf {
                bytes: [u8; 192],
                used: usize,
            }
            impl ::core::fmt::Write for Buf {
                fn write_str(&mut self, s: &str) -> ::core::fmt::Result {
                    let room = self.bytes.len() - self.used;
                    let n = s.len().min(room);
                    self.bytes[self.used..self.used + n].copy_from_slice(&s.as_bytes()[..n]);
                    self.used += n;
                    Ok(())
                }
            }

            let mut buf = Buf {
                bytes: [0u8; 192],
                used: 0,
            };
            let _ = write!(buf, "{info}");

            unsafe extern "C" {
                fn nros_platform_panic(msg: *const u8, len: usize) -> !;
            }
            // SAFETY: `buf.bytes[..used]` is initialised and outlives the
            // diverging call; the ABI takes a length-delimited diagnostic, not
            // a C string.
            unsafe { nros_platform_panic(buf.bytes.as_ptr(), buf.used) }
        }
    };
}

/// Define Zephyr's `rust_main` for a self-bringup Rust component package.
///
/// The macro is intended for `rust_cargo_application()` apps whose crate
/// already invokes `nros::node!()`. It opens a Zephyr executor, registers
/// the supplied component through [`ExecutorNodeRuntime`], and spins forever.
/// Issue 0330 — force-link an RMW backend crate into a pure-Rust staticlib.
///
/// On a Rust-only image (Zephyr and friends) the Zephyr module emits a weak
/// `nros_rmw_<name>_register` and calls it only if it resolves. The strong
/// definition is the backend crate's `#[no_mangle]` export — and rustc's
/// staticlib DCE drops it unless something in the crate being compiled into the
/// staticlib references that crate. The symbol is then present in the rlib and
/// absent from the `.a`, the weak call sees NULL, and the image comes up with no
/// backend registered (issues 0155 / 0163).
///
/// This emits the reference, without naming any backend in nano-ros' own
/// RMW-agnostic layers — the app crate names it, because the app crate is what
/// selects an RMW:
///
/// ```ignore
/// #[cfg(feature = "rmw-zenoh")]
/// nros::force_link_backend!(nros_rmw_zenoh);
/// #[cfg(feature = "rmw-xrce")]
/// nros::force_link_backend!(nros_rmw_xrce_cffi);
/// ```
///
/// It is an ANCHOR, not a registration call — the static is never executed
/// (same class as `nros-c`'s `FORCE_LINK` and `nros-rmw-cffi`'s section anchor).
/// Registration happens through `nros_app_register_backends`. Backends whose
/// register entry lives in a C/C++ library the image already links (cyclonedds
/// on Zephyr) need no anchor at all.
///
/// Invoke at module scope. Multiple invocations in one crate are fine — each
/// expands inside its own anonymous const, so the static names cannot collide.
#[macro_export]
macro_rules! force_link_backend {
    // `ident`, not `path`: a `path` fragment may not be followed by `::`, so
    // `$backend::register()` fails to parse at the CALL site with a misleading
    // "expected an operator". Backend crate names are single idents anyway.
    ($backend:ident) => {
        const _: () = {
            #[used]
            static __NROS_FORCE_LINK_BACKEND: fn() = || {
                let _ = $backend::register();
            };
        };
    };
}

// Phase 248 C7 (Method A) — gated on `rmw-cffi` only (needs `Executor`), NOT a
// `platform-*` feature. This is a framework ENTRY macro (same category as
// `nros::main!`'s zephyr `rust_main` codegen) — `#[macro_export]` so it emits
// nothing unless a Zephyr example invokes it; the body's `::zephyr::*` /
// `::nros_platform::zephyr::wait_network` resolve only in that zephyr-build
// context (the example deps the `zephyr` crate + `nros-platform[platform-zephyr]`).
#[cfg(feature = "rmw-cffi")]
#[macro_export]
macro_rules! zephyr_component_main {
    ($node:ty) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn rust_main() {
            unsafe {
                zephyr::set_logger().ok();
            }
            // Phase 248 C7 step 1 — relocated helper (was `$crate::platform::zephyr`).
            let _ = ::nros_platform::zephyr::wait_network(2000);
            // Phase 249 P1 — RMW register is board/platform-owned (Phase 248 C5a);
            // the backend-agnostic `nros` crate cannot register (no backend dep).
            // Issue 0155 — the "board/platform boot path" that was supposed to
            // register never fired for pure-Rust Zephyr images: the zephyr
            // module emits a STRONG `nros_app_register_backends` stub for the
            // Kconfig-selected RMW (zephyr/CMakeLists.txt Phase 160.A), but
            // only the C/C++ `nros_cpp_init` path ever CALLED it — a Rust-only
            // image reached `Executor::open` with no backend registered and
            // died with Transport(ConnectionFailed) (silently, pre-0155).
            // Call the hook explicitly, exactly like the C++ init path.
            unsafe extern "C" {
                fn nros_app_register_backends();
            }
            unsafe { nros_app_register_backends() };
            // Issue 0163 — a pure-Rust image has no `libnros_c.a`, so the
            // backend must ride in THIS staticlib and be referenced from the
            // app crate, or rustc's staticlib DCE drops the whole backend
            // closure (the `#[no_mangle]` C export included), leaving the
            // module's weak `nros_rmw_<x>_register` resolving to NULL and the
            // hook above registering nothing.
            //
            // Issue 0330 — that reference used to be a pair of hardcoded
            // `::nros_rmw_zenoh::register()` / `::nros_rmw_xrce_cffi::register()`
            // calls emitted RIGHT HERE, which named two concrete backends in
            // the RMW-agnostic facade (and left cyclonedds handled asymmetrically
            // through the C hook). It also forced every consumer to carry
            // `rmw-zenoh` / `rmw-xrce` feature rows purely so these `cfg`s would
            // resolve — the cyclonedds-only example carried both as inert
            // placeholders. The anchor now lives in the app crate, which is the
            // layer that legitimately selects an RMW: see
            // [`nros::force_link_backend!`]. Registration itself is unchanged —
            // the `nros_app_register_backends` hook above does it.
            // Locator: `default_const()` = EMPTY locator → zenoh-pico
            // multicast scouting, which native_sim NSOS can't satisfy.
            // Bake `NROS_LOCATOR` at compile time (the example `build.rs`
            // re-exports `CONFIG_NROS_ZENOH_LOCATOR` from Kconfig into that
            // env). No baked value → falls back to the empty locator.
            const BAKED_LOCATOR: ::core::option::Option<&str> = ::core::option_env!("NROS_LOCATOR");
            // Domain: the example `build.rs` bakes `CONFIG_NROS_DOMAIN_ID`
            // into `NROS_DOMAIN_ID` the same way (its comment has promised
            // this consumption since phase-225; the phase-277 macro rework
            // dropped it — issue 0161: every Rust cyclonedds image silently
            // ran domain 0 regardless of the Kconfig bake).
            const BAKED_DOMAIN: ::core::option::Option<&str> =
                ::core::option_env!("NROS_DOMAIN_ID");
            let domain_id: u32 = match BAKED_DOMAIN {
                ::core::option::Option::Some(d) => match d.parse() {
                    ::core::result::Result::Ok(v) => v,
                    ::core::result::Result::Err(_) => {
                        panic!("nros zephyr entry: NROS_DOMAIN_ID baked non-numeric: {d:?}")
                    }
                },
                ::core::option::Option::None => 0,
            };
            // #166 / phase-286 W1 — native_sim test parallelism. The test
            // harness launches the image with `-testargs --nros-locator=<loc>`
            // and starts a per-test zenohd on that (ephemeral) port; preferring
            // it over the build-time bake lets every test dial a DISTINCT router,
            // retiring the shared-baked-port serialization of the zenoh e2e
            // lanes. Provided by `nros-platform-zephyr` (argv-backed, process
            // lifetime); returns NULL on real embedded → the bake stands.
            unsafe extern "C" {
                fn nros_runtime_locator_override() -> *const ::core::ffi::c_char;
            }
            let runtime_locator: ::core::option::Option<&str> = {
                let p = unsafe { nros_runtime_locator_override() };
                if p.is_null() {
                    ::core::option::Option::None
                } else {
                    match unsafe { ::core::ffi::CStr::from_ptr(p) }.to_str() {
                        ::core::result::Result::Ok(s) if !s.is_empty() => {
                            ::core::option::Option::Some(s)
                        }
                        _ => ::core::option::Option::None,
                    }
                }
            };
            let effective_locator = runtime_locator.or(match BAKED_LOCATOR {
                ::core::option::Option::Some(loc) if !loc.is_empty() => {
                    ::core::option::Option::Some(loc)
                }
                _ => ::core::option::Option::None,
            });
            let config = match effective_locator {
                ::core::option::Option::Some(loc) => {
                    $crate::ExecutorConfig::new(loc).node_name(<$node as $crate::Node>::NAME)
                }
                ::core::option::Option::None => {
                    $crate::ExecutorConfig::default_const().node_name(<$node as $crate::Node>::NAME)
                }
            }
            .domain_id(domain_id);
            // Issue 0155 — fail LOUD (repo rule: panic, not silent
            // early-return). A silent `return` here idles the image with zero
            // output; the zephyr-cyclonedds rust lane was undiagnosable until
            // this printed the real error.
            let executor = match $crate::Executor::open(&config) {
                Ok(executor) => executor,
                Err(e) => {
                    panic!("nros zephyr entry: Executor::open failed: {e:?}");
                }
            };
            let mut runtime = $crate::ExecutorNodeRuntime::from_executor(executor);
            if let Err(e) = runtime.register_node::<$node>() {
                panic!("nros zephyr entry: register_node failed: {e:?}");
            }
            // Readiness marker. The C/C++ Zephyr listeners print
            // "Waiting for messages..." from their `main()` before the spin
            // loop; the e2e harness polls for that substring to know the
            // subscriber has declared before starting the talker (Phase 89.12).
            // The Rust path's spin loop lives in this macro (the node only owns
            // callbacks), so emit the same canonical marker here — without it a
            // fully-working Rust listener never signals readiness and the e2e
            // times out at 30 s (issue #35: the zenoh native_sim rust pubsub /
            // service / action failures were this missing marker, not a
            // transport fault — `Executor::open` + `register_node` had already
            // succeeded).
            ::log::info!("Waiting for messages");
            loop {
                let _ = runtime.spin_once(::core::time::Duration::from_millis(10));
            }
        }
    };
}

// Re-export node types
pub use nros_node::{NodeConfig, PublisherHandle, StandaloneNode, SubscriptionHandle};

// Re-export publisher/subscriber options (topic + QoS; always available).
pub use nros_node::{PublisherOptions, SubscriptionOptions};

// Re-export timer types
pub use nros_node::{TimerCallbackFn, TimerDuration, TimerHandle, TimerMode, TimerState};
// phase-425 W4/W5 — which clock advances a timer. Re-exported here for the
// reason `sim-time` is a feature here: an application deps `nros`, not
// `nros-node`, so a capability that stops at the core crate is one no
// application can name. W4 added the type and the registrar and stopped at the
// core, which the `/clock` fixture found the moment it tried to use them.
// Gated on `rmw-cffi`, the spelling every other executor re-export here uses:
// `nros-node`'s own `has_rmw` IS its `rmw-cffi` feature, and `has_rmw` is a
// build-script cfg that does not exist in THIS crate.
#[cfg(feature = "rmw-cffi")]
pub use nros_node::executor::TimerClockSource;

// Re-export transport types (middleware-agnostic)
pub use nros_rmw::{
    ClientTrait, Publisher, QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy,
    QoSOverride, QoSOverrideRole, QoSOverrideValue, QoSPolicyMask, QoSProfile,
    QoSReliabilityPolicy, Rmw, RmwConfig, ServiceInfo, ServiceRequest, ServiceTrait, Session,
    SessionMode, Subscription as SubscriptionTrait, TopicInfo, Transport, TransportConfig,
};

/// Phase 108.B — standard ROS-2-equivalent QoS profiles. Match
/// upstream `rmw_qos_profile_default` etc. field-by-field. Backends
/// validate against these synchronously at create time; no silent
/// downgrade.
// phase-379 W5 — rclrs exports the eight QoS presets as CRATE-LEVEL consts
// (`rclrs::QOS_PROFILE_DEFAULT`); ours were associated consts on `QoSProfile`
// and nothing re-exported them, so `use rclrs::QOS_PROFILE_DEFAULT` had no
// counterpart to port to. The NAMES already matched exactly — only the path
// did not, which is why the ledger filed these as a re-export and not a
// rename.
//
// These ALIAS the associated consts rather than restating them. `nros::qos`
// below is a second, hand-written copy of the same presets that predates them;
// `qos_presets_agree` in the test module asserts the two never drift, which is
// the hand-mirror class (issues 0088/0160/0245) one layer up.
/// `rmw_qos_profile_default` — reliable, volatile, keep-last(10).
pub const QOS_PROFILE_DEFAULT: QoSProfile = QoSProfile::QOS_PROFILE_DEFAULT;
/// `rmw_qos_profile_system_default` — the RMW implementation's own defaults.
pub const QOS_PROFILE_SYSTEM_DEFAULT: QoSProfile = QoSProfile::QOS_PROFILE_SYSTEM_DEFAULT;
/// `rmw_qos_profile_sensor_data` — best-effort, keep-last(5).
pub const QOS_PROFILE_SENSOR_DATA: QoSProfile = QoSProfile::QOS_PROFILE_SENSOR_DATA;
/// `rmw_qos_profile_services_default`.
pub const QOS_PROFILE_SERVICES_DEFAULT: QoSProfile = QoSProfile::QOS_PROFILE_SERVICES_DEFAULT;
/// `rmw_qos_profile_parameters` — reliable, depth 1000.
pub const QOS_PROFILE_PARAMETERS: QoSProfile = QoSProfile::QOS_PROFILE_PARAMETERS;
/// `rmw_qos_profile_parameter_events`.
pub const QOS_PROFILE_PARAMETER_EVENTS: QoSProfile = QoSProfile::QOS_PROFILE_PARAMETER_EVENTS;
/// The clock preset — sensor-data shaped with depth 1.
pub const QOS_PROFILE_CLOCK: QoSProfile = QoSProfile::QOS_PROFILE_CLOCK;
/// The action-status preset — reliable + transient-local, depth 1.
pub const QOS_PROFILE_ACTION_STATUS_DEFAULT: QoSProfile =
    QoSProfile::QOS_PROFILE_ACTION_STATUS_DEFAULT;

pub mod qos {
    use crate::{
        QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy, QoSProfile,
        QoSReliabilityPolicy,
    };

    /// `rmw_qos_profile_default`-equivalent: reliable + volatile +
    /// keep-last(10), automatic liveliness, no deadline / lifespan.
    pub const DEFAULT: QoSProfile = QoSProfile {
        reliability: QoSReliabilityPolicy::Reliable,
        durability: QoSDurabilityPolicy::Volatile,
        history: QoSHistoryPolicy::KeepLast,
        liveliness_kind: QoSLivelinessPolicy::Automatic,
        depth: 10,
        deadline_ms: 0,
        lifespan_ms: 0,
        liveliness_lease_ms: 0,
        avoid_ros_namespace_conventions: false,
        tx_express: false,
    };

    /// `rmw_qos_profile_sensor_data`-equivalent: best-effort +
    /// volatile + keep-last(5).
    pub const SENSOR_DATA: QoSProfile = QoSProfile {
        reliability: QoSReliabilityPolicy::BestEffort,
        depth: 5,
        ..DEFAULT
    };

    /// `rmw_qos_profile_services_default`-equivalent.
    pub const SERVICES_DEFAULT: QoSProfile = DEFAULT;

    /// `rmw_qos_profile_parameters`-equivalent: depth = 1000.
    pub const PARAMETERS: QoSProfile = QoSProfile {
        depth: 1000,
        ..DEFAULT
    };

    /// `rmw_qos_profile_system_default`-equivalent — **an absence, not a
    /// profile**: every policy is the SYSTEM_DEFAULT sentinel and the depth is
    /// 0, resolved by whichever backend is linked.
    ///
    /// issue 0829 — this said `= DEFAULT` (depth 10) while
    /// `QoSProfile::QOS_PROFILE_SYSTEM_DEFAULT` said depth 1, so one name
    /// shipped two queue depths depending on which spelling a caller reached.
    /// It is now an ALIAS of the associated const, like the other four, and
    /// neither number survives: see `QoSProfile::QOS_PROFILE_SYSTEM_DEFAULT`.
    pub const SYSTEM_DEFAULT: QoSProfile = QoSProfile::QOS_PROFILE_SYSTEM_DEFAULT;
}

// Re-export safety types when feature is enabled
#[cfg(feature = "safety-e2e")]
pub use nros_rmw::{IntegrityStatus, SafetyValidator, crc32};

// Phase 248 C7 step 1 — the `nros::platform::zephyr` module (the
// `wait_for_network` FFI wrapper) RELOCATED to `nros-platform`
// (`nros_platform::zephyr::wait_network`); callers reference it via
// `::nros_platform::zephyr::wait_network`. nros no longer hosts a platform
// helper module. (The `zephyr_component_main!` macro relocation is C7 step 2.)
//
/// Backend-specific internal types.
///
/// These types are implementation details of the transport backends.
/// Most users should use the high-level APIs (`Executor`, etc.)
/// instead of these types directly.
///
/// The `Rmw*` type aliases resolve to whichever backend is active at compile time,
/// providing a backend-agnostic way to reference concrete transport types.
pub mod internals {
    // ── Backend-agnostic type aliases ────────────────────────────────────
    // These resolve to the concrete types of the active RMW backend.
    // Today the only exposed backend at this layer is the cffi shim.

    #[cfg(feature = "rmw-cffi")]
    pub type RmwSession = nros_rmw_cffi::CffiSession;
    #[cfg(feature = "rmw-cffi")]
    pub type RmwPublisher = nros_rmw_cffi::CffiPublisher;
    #[cfg(feature = "rmw-cffi")]
    pub type RmwSubscriber = nros_rmw_cffi::CffiSubscription;
    #[cfg(feature = "rmw-cffi")]
    pub type RmwServiceServer = nros_rmw_cffi::CffiService;
    #[cfg(feature = "rmw-cffi")]
    pub type RmwServiceClient = nros_rmw_cffi::CffiClient;

    /// Phase 124.A — zero-copy publisher slot type. Lives in the
    /// `internals` module so `nros-c` can construct + transmute the
    /// lifetime when boxing the slot for the C-side `_loan` /
    /// `_commit` / `_discard` token plumbing.
    #[cfg(all(feature = "rmw-cffi", feature = "lending"))]
    pub type RmwSlot<'a> = nros_rmw_cffi::CffiSlot<'a>;

    /// Phase 124.A — zero-copy subscriber view type.
    #[cfg(all(feature = "rmw-cffi", feature = "lending"))]
    pub type RmwView<'a> = nros_rmw_cffi::CffiView<'a>;

    /// Open a new middleware session.
    ///
    /// Wraps the backend-specific session constructor behind a common signature.
    /// Used by the C API (`nros-c`); Rust users should prefer `Executor::open()`.
    ///
    /// Phase 156 — takes an explicit primary backend by name, mirroring what
    /// `Executor::open` does for Rust callers. Without it, C bridges built with
    /// two linked backends (e.g. xrce + dds) get whichever ctor fires first —
    /// non-deterministic across link orderings, and often the wrong backend for
    /// the bridge's intended primary.
    ///
    /// Issue 1050 defect (3) — `rmw` is the RESOLVED selector, not a second
    /// reading of the environment. This function used to consult
    /// `rmw_selector()` itself, which made it the tree's second answer to "which
    /// backend?" and blind to every rung but the environment: a C image with a
    /// BAKED selector had it discarded here. The caller resolves the whole
    /// ladder (`env > baked > none`) through `ExecutorConfig` and passes the
    /// result; `None` means the registry must hold exactly one backend, which
    /// `nros_rmw_cffi::get_vtable` now enforces rather than assumes.
    #[cfg(feature = "rmw-cffi")]
    pub fn open_session(
        locator: &str,
        mode: nros_rmw::SessionMode,
        domain_id: u32,
        node_name: &str,
        rmw: Option<&str>,
    ) -> Result<RmwSession, nros_rmw::TransportError> {
        use nros_rmw::Rmw;

        // Phase 249 P4b.1 — every linked backend self-registered via
        // its `.init_array` ctor before `main` (RFC-0042 §D3.3); no
        // runtime section walk.

        let config = nros_rmw::RmwConfig {
            locator,
            mode,
            domain_id,
            node_name,
            namespace: "",
            properties: &[],
        };
        // Phase 155.B — propagate the real `TransportError` instead of
        // collapsing every backend failure to `ConnectionFailed`. The
        // C-side `nros_support_init` decodes the variant into a
        // specific `NROS_RET_*` code so "init -> -X" tells the user
        // which precondition the backend rejected.
        // issue 1050 defect (3) — the selector ARRIVES here now. It used to be
        // read from `$NROS_RMW` on the spot, which is why a baked one could not
        // reach this path: the reader knew about one rung and the resolver knew
        // about two. The environment still wins; it wins in `ExecutorConfig`,
        // where every other field's precedence is decided, instead of here.
        if let Some(name) = rmw {
            return nros_rmw_cffi::CffiRmw::open_with_rmw(name, &config);
        }
        nros_rmw_cffi::CffiRmw.open(&config)
    }

    /// Drive middleware I/O for pull-based backends.
    ///
    /// Delegates to [`Session::drive_io()`](nros_rmw::Session::drive_io),
    /// which each backend implements appropriately (no-op for push-based,
    /// poll for pull-based).
    ///
    /// Used by the C API executor before polling handles.
    #[cfg(feature = "rmw-cffi")]
    pub fn drive_session_io(session: &mut RmwSession, timeout_ms: i32) {
        use nros_rmw::Session;
        let _ = session.drive_io(timeout_ms);
    }
}

// Re-export types that don't depend on RMW (always available)
pub use nros_node::{
    BOOT_SET_DOMAIN, BOOT_SET_LOCATOR, BOOT_SET_NAMESPACE, BOOT_SET_NODE_NAME, BakedBootConfig,
    BootConfig, BootConfigError, DOMAIN_ID_EXPLICIT_ZERO_C_ABI, DOMAIN_ID_MAX, ExecutorConfig,
    ExecutorSemantics, GuardCondition, HandleId, HandleSet, InvocationMode, NROS_BOOT_CONFIG_MAGIC,
    NROS_BOOT_CONFIG_VERSION, RawCancelCallback, RawGoalCallback, RawServiceCallback,
    RawSubscriptionCallback, ReadinessSnapshot, ShutdownCallbackFn, ShutdownCallbackHandle,
    ShutdownPhase, SpinOnceResult, SpinOptions, SpinPeriodPollingResult, Trigger,
    baked_domain_from_c_abi,
};

// ---------------------------------------------------------------------------
// The error vocabulary (issue 0783). Both re-exports are unconditional.
//
// Every fallible call in the Rust user API returns `NodeError`, and
// `NodeError::Transport(TransportError)` is its most common variant — so
// handling one means naming both. They used to be exported from two unrelated
// blocks (`NodeError` with the boot/executor types, `TransportError` with the
// transport traits), which is a discoverability nit rather than a capability
// gap: both were always reachable, and `prelude` already listed them together.
// Nothing here changes what `nros::` exports.
//
// There is deliberately NO numeric code type beside them. A Rust caller matches
// a flat enum; the numeric vocabulary is the C ABI's `nros_ret_t` /
// `NROS_RET_*` (0, -1..-16), which is its own space and not `rcl_ret_t`'s.
// nros-core carried an `rcl_ret_t` mirror (`RclReturnCode`) and an error that
// wrapped it (`NanoRosError`); neither was ever reachable from this facade and
// neither had a producer, so issue 0783 deleted them rather than exporting a
// type a user could name and never receive. RFC-0036's Errors row now describes
// what these two are.
// ---------------------------------------------------------------------------
pub use nros_node::NodeError;
pub use nros_rmw::TransportError;

// RFC-0052 / phase-296 W3b — on-target contract-monitor types. Baked
// `system_monitors.rs` uses the fully-qualified `::nros_node::executor::
// monitor::*` path; this re-export lets hand-written entries and fixtures
// reach the same types through the `nros` umbrella (they install the
// tables via `Executor::set_monitor_table` / `set_age_table` and drain
// with `drain_violations`). The monitor module is `has_rmw`-gated in
// nros-node (it names entity types), so mirror that with `rmw-cffi`.
#[cfg(feature = "rmw-cffi")]
pub mod monitor {
    pub use nros_node::executor::monitor::{
        AgeMonitorSpec, MonitorSpec, PubMonitorCell, SubMonitorCell, Violation,
    };
}

// Re-export RMW-dependent types (require an active transport backend)
#[cfg(feature = "rmw-cffi")]
pub use nros_node::{
    ActionClient, ActionClientCore, ActionServer, ActionServerCore, ActionServerHandle,
    ActionServerRawHandle, ActiveGoal, CompletedGoal, EmbeddedPublisher, EmbeddedRawPublisher,
    EmbeddedServiceClient, EmbeddedServiceServer, Executor, ExecutorSizing, FeedbackStream,
    GoalFeedbackStream, LoanError, NodeHandle, Promise, PublishLoan, RawActionClientSpec,
    RawActionServerSpec, RawActiveGoal, RawSubscription, RecvView, SessionHandle, SessionSpec,
    Subscription, action_channel_type,
};

// phase-271 (issue #110) — per-entry executor sizing helper: the orchestration
// codegen's `build_executor` sizes its backing to the system's callback count
// via `nros::arena_size_for(CALLBACK_COUNT)` + `ExecutorSizing`, replacing the
// workspace-global `NROS_EXECUTOR_MAX_CBS`.
#[cfg(feature = "rmw-cffi")]
pub use nros_node::config::arena_size_for;

/// The configured default receive-buffer size (`NROS_SUBSCRIPTION_BUFFER_SIZE`).
///
/// NOT a fallback for a missing bound any more (phase-403 W0): [`rx_buffer_for!`]
/// used to expand to this for an unbounded type and now refuses to compile
/// instead. It stays public, and stays the default, for the paths that have no
/// `M` to ask rather than a bound they declined to use -- `create_subscription_raw`
/// and the RFC-0043 type-name-string subscriptions, the service / action / TX
/// buffer defaults, the `pubsub_entry` term of the arena derivation in
/// `nros-node/build.rs`, and its weld to the C API's `MESSAGE_BUFFER_SIZE`.
/// Whether the arena derivation should stop leaning on it is a phase-403 W5
/// question.
///
/// Re-exported here (rather than reached through `nros_node`) because a macro
/// expands at the CALLER, where `nros_node` may not be a dependency at all.
pub use nros_node::config::DEFAULT_RX_BUF_SIZE;

/// Phase 392 W3b — the bound behind [`rx_buffer_for!`]. Not part of the stable
/// surface; call the macro.
///
/// Public only because a `macro_rules!` body is expanded in the caller's crate
/// and can reach nothing private. `#[doc(hidden)]` and `__`-prefixed for the
/// same reason every other macro-support item in this crate is.
#[doc(hidden)]
pub const fn __rx_bound<M: nros_serdes::schema::Message>() -> Option<usize> {
    nros_serdes::size::max_serialized_bound::<M>()
}

// Phase 173.5 — board config traits. `BoardConfig` (read locator /
// domain). `BoardTransportConfig` was removed with its dead
// setters (issue 1064); the live path is the deploy overlay.
pub use nros_platform::BoardConfig;

// Phase 216.A.1 — `DispatchStrategy` enum. User-visible at
// `nros::DispatchStrategy`; the canonical home is `nros_platform::
// board::dispatch` so the C ABI symbol the `nros::node!()` macro emits
// (`__nros_node_<pkg>_dispatch_strategy() -> u8`) lives next to the
// other board-side trampolines.
pub use nros_platform::DispatchStrategy;

/// Implementation detail — used by `nros::node!()` macro expansion.
///
/// Re-exports `nros_platform` so the macro's emitted trampoline can
/// reference `RuntimeCtx` / `RuntimeError` / the `Node*Fn`
/// fn-pointer aliases without forcing every consumer Node pkg's
/// `Cargo.toml` to carry an explicit `nros-platform` dep on top of
/// `nros`. Phase 212.M-F.13 path (b).
///
/// Not part of the public API — paths under this module may change at
/// any time. End users should depend on `nros` alone and invoke
/// `nros::node!()`; the macro routes through here automatically.
#[doc(hidden)]
pub mod __macro_support {
    pub use ::nros_platform;

    /// phase-314 — whether THIS `nros` build carries the parameter services.
    ///
    /// The `nros::main!` expansion const-asserts it when the system declares
    /// `[param_services]`. A cfg in the entry crate cannot see this: the
    /// feature belongs to `nros`, and the entry enables it through its
    /// dependency, so only `nros` itself can report the answer.
    ///
    /// Without the assert the mismatch is SILENT — `apply_param_services` is a
    /// no-op, the build succeeds, the image boots, and `ros2 param list`
    /// returns nothing.
    pub const PARAM_SERVICES_ENABLED: bool = cfg!(feature = "param-services");

    /// Issue 0257 — the build-time executor callback-table size
    /// (`NROS_EXECUTOR_MAX_CBS`, default 4). Re-exported so the `nros::main!`
    /// expansion can `const`-assert the model's entity count against the
    /// capacity that ACTUALLY compiles in, instead of letting the image boot
    /// and die on `create_timer (code=-6 Full)`.
    pub use ::nros_node::config::MAX_CBS as EXECUTOR_MAX_CBS;
}

// Phase 110.B / 110.G — scheduling-context API surface. Consumers
// of the Phase 110 cyclic / TT scheduler need these types to
// describe schedules and bind handles; re-exporting them here
// keeps user code free of `nros_node::executor::sched_context`
// path noise. Gated on `rmw-cffi`: the source module is
// `#[cfg(any(has_rmw, test))]` in nros-node, so it only exists once
// an RMW backend is linked (matches the re-export block above).
#[cfg(feature = "rmw-cffi")]
pub use nros_node::executor::sched_context::{
    DeadlineAction, DeadlinePolicy, OptUs, Priority, SchedClass, SchedContext, SchedContextId,
    TimeTriggeredSchedule, TimeTriggeredScheduleError, TimeTriggeredWindow,
};

#[cfg(all(feature = "std", feature = "rmw-cffi"))]
pub use nros_node::SpinPeriodResult;

// Re-export service types
pub use nros_core::{ServiceClient, ServiceServer};

// Re-export action types.
//
// issue 0796 — `CancelResponse` (the per-goal Reject/Accept decision) and
// `CancelReturnCode` (the `action_msgs/srv/CancelGoal` RPC status) are two
// concepts that shared one name until the split. Both are exported: without
// `CancelReturnCode` here, `ActionClient::cancel_goal`'s `Promise<CancelReturnCode>`
// could not be NAMED from `nros::` alone.
pub use nros_core::{
    CancelResponse, CancelReturnCode, GoalId, GoalInfo, GoalResponse, GoalStatus,
    GoalStatusStamped, RosAction,
};

// Re-export lifecycle types (always available, no_std compatible)
pub use nros_core::{LifecycleState, LifecycleTransition, TransitionResult};
pub use nros_node::{LifecycleCallbackFn, LifecycleError, LifecyclePollingNode};

/// Re-export of the full lifecycle module so examples can reach
/// `LifecycleCallbackSlot`, `LifecyclePollingNodeCtx`, etc.
pub mod lifecycle {
    pub use nros_core::lifecycle::{LifecycleState, LifecycleTransition, TransitionResult};
    pub use nros_node::lifecycle::*;
}

// Phase 128.G — bridge surface re-exports. Gated behind the
// `bridge` / `config` umbrella features so single-backend builds
// don't pull in `nros-bridge` (or, for `config`, the TOML stack).
#[cfg(feature = "bridge")]
pub use nros_bridge as bridge;

#[cfg(feature = "config")]
pub use nros_bridge::run_from_config;

// Re-export parameter types.
//
// phase-382 W2' — `ParameterStorage` / `ParameterTable` are here because the
// store's slots are CALLER-OWNED: anyone constructing a `ParameterServer`
// outside an executor has to place the storage and lend it.
pub use nros_params::{
    MandatoryParameter, OptionalParameter, Parameter, ParameterBuilder, ParameterDescriptor,
    ParameterError, ParameterServer, ParameterStorage, ParameterTable, ParameterType,
    ParameterValue, ParameterVariant, ReadOnlyParameter, SetParameterResult,
};
/// Prelude module for convenient imports
///
/// Import everything you need with a single statement:
/// ```
/// use nros::prelude::*;
/// ```
/// phase-379 W5 — the RTOS machinery, named explicitly.
///
/// The second tier of the two-tier surface. `nros::prelude` is the API a ported
/// ROS 2 node uses; everything here exists because the target is an RTOS, has no
/// correspondent in rclrs/rclcpp/rclc, and a ROS 2 developer reading a node
/// should not have to step over it.
///
/// Nothing moved out of `nros::` — these are re-exports, and every name is still
/// reachable at its old path. What changed is that they are no longer dragged in
/// by `use nros::prelude::*`.
///
/// The membership rule is mechanical rather than taste: a name belongs in the
/// PRELUDE iff the parity ledger gives it a non-`extension` verdict — i.e. it
/// corresponds to something in rclrs, rclcpp or rclc. Names with no
/// correspondent belong here, unless they are load-bearing for startup
/// (`ExecutorConfig`, `SpinOptions`), which is an argued allow-list rather than
/// an exception anyone may grow.
pub mod embedded {
    // Wire encoding. A node publishes typed messages; these are for code that
    // handles bytes, which upstream hides entirely.
    pub use crate::{CdrReader, CdrWriter};

    // Handle bookkeeping — the static tables that replace an allocator.
    #[cfg(feature = "rmw-cffi")]
    pub use crate::{HandleId, HandleSet, InvocationMode};

    // Component/runtime plumbing, and the source-metadata capture the
    // orchestration layer records. None of it appears in a ported node.
    pub use crate::{MetadataRecorder, NodeRuntimeAdapter, RuntimeNodeRecord};

    // Entity REGISTRATION vocabulary. A ported node writes
    // `create_publisher(...)`; these are what the declaration macros and the
    // orchestration layer use to describe what was created.
    pub use crate::{
        ActionTag, Callback, CallbackEffectKind, CallbackEffects, DeclaredNode,
        DeclaredNodeRuntime, EntityKind, NodeRuntime, ServiceTag, SourceLocationMetadata,
        SourceNameKind, SubscriptionTag, record_node_metadata, register_node,
    };
    // Gated where the root gates it — the export follows the capability, not
    // the tier.
    #[cfg(feature = "alloc")]
    pub use crate::SourceMetadataExport;
}

pub mod prelude {
    // phase-379 W5 — the RTOS machinery moved to `nros::embedded`. Removed
    // here rather than re-exported from both: a two-tier surface that still
    // drags tier two in through the glob is one tier with extra words.

    pub use crate::{
        Deserialize, Logger, MessageInfo, NodeConfig, PublisherHandle, QoSDurabilityPolicy,
        QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, RosMessage, RosService, Serialize,
        StandaloneNode, SubscriptionHandle, TopicInfo,
    };

    // phase-417 / issue 0589 — logging, in the glob a node author already
    // writes. `Logger` above now means `nros_log::Logger`; these are what
    // drives it. Without them the shortest path from `use nros::prelude::*` to
    // a printed line was `std::println!`, which kills a Zephyr `native_sim`
    // image outright.
    //
    // The five severity macros now carry rclrs's OWN names (RFC-0089, settled
    // 2026-09-04), so a ROS 2 developer meeting one through the glob meets the
    // exact spelling their client library has, not a near-miss of it. That is
    // the property `check-prelude-tiers` enforces mechanically: a ledger
    // `extension` may not sit in the glob.
    //
    // `Severity`, `get_logger`, `get_or_create_logger`, `register_logger` and
    // `nros_trace` are ledger `extension`s -- rclrs has no correspondent for a
    // free logger lookup, a registry, or a TRACE severity (it stops at debug).
    // They stay behind `nros::logging::` / the crate root, where reaching for
    // one is a decision. phase-379 W5's rule, applied to this phase's own work.
    //
    // The throttle macros ARE here: they are ledger `divergence`s, not
    // extensions -- rclrs has the capability under a different decomposition
    // (`log_info!(logger.throttle(d), "…")`), so the glob is not introducing a
    // name with no upstream counterpart.
    pub use crate::{
        log_debug, log_error, log_fatal, log_info, log_warn, nros_debug_throttle,
        nros_debug_throttle_at, nros_error_throttle, nros_error_throttle_at, nros_fatal_throttle,
        nros_fatal_throttle_at, nros_info_throttle, nros_info_throttle_at, nros_warn_throttle,
        nros_warn_throttle_at,
    };
    // Re-export component-mode API.
    #[cfg(feature = "rmw-cffi")]
    pub use crate::NodeExecutorRuntime;
    #[cfg(feature = "alloc")]
    pub use crate::{
        Node, NodeActionClient, NodeActionServer, NodeContext, NodeDeclError, NodeOptions,
        NodeParameter, NodePublisher, NodeResult, NodeServiceClient, NodeServiceServer,
        NodeSubscription, NodeTimer, ParameterDefault, node,
    };

    // Re-export lifecycle types
    pub use crate::{
        LifecycleCallbackFn, LifecycleError, LifecyclePollingNode, LifecycleState,
        LifecycleTransition, TransitionResult,
    };

    // Re-export executor config + handle types (always available)
    pub use crate::{
        ExecutorConfig, GuardCondition, NodeError, SessionMode, SpinOnceResult, SpinOptions,
        SpinPeriodPollingResult, TransportError, Trigger,
    };

    // issue 0687 — `ExecutorConfig::from_env()` is an extension trait now (the
    // environment is read at this crate's edge, not in the core), so the
    // spelling only works where the trait is in scope. It is in the prelude
    // precisely so that the consumers written against the inherent method —
    // the native examples, the benches — keep compiling unchanged.
    #[cfg(feature = "env")]
    pub use crate::ExecutorConfigEnvExt;

    // Re-export RMW-dependent executor + handle types
    #[cfg(feature = "rmw-cffi")]
    pub use crate::{
        EmbeddedPublisher, EmbeddedServiceClient, Executor, FeedbackStream, NodeHandle, Promise,
        Subscription,
    };

    // Publisher/Subscriber options (topic + QoS).
    pub use crate::{PublisherOptions, SubscriptionOptions};

    #[cfg(all(feature = "std", feature = "rmw-cffi"))]
    pub use crate::SpinPeriodResult;

    // Re-export parameter types
    pub use crate::{ParameterServer, ParameterStorage, ParameterType, ParameterValue};

    // Re-export typed parameter API (rclrs-compatible builder pattern)
    pub use crate::{
        MandatoryParameter, OptionalParameter, ParameterBuilder, ParameterError, ParameterVariant,
        ReadOnlyParameter,
    };

    // Re-export action types
    pub use crate::{GoalId, GoalInfo, GoalResponse, GoalStatus, GoalStatusStamped, RosAction};

    // Re-export Time, Duration, Clock from core
    pub use nros_core::{Clock, ClockType, Duration, Time};

    // Re-export timer types
    pub use crate::{TimerCallbackFn, TimerDuration, TimerHandle, TimerMode};
}

/// Derive macros for message types
///
/// Use these macros to generate message serialization code.
/// These macros help you create custom message types that are compatible
/// with ROS 2's CDR serialization format.
pub mod derive {
    #[cfg(feature = "macros")]
    pub use nros_macros::RosMessage;
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_prelude_imports() {
        // This test just verifies that the prelude compiles
        use crate::prelude::*;

        let _ = NodeConfig::new("test_node", "/");
        let _ = QoSProfile::BEST_EFFORT;
    }

    /// Verify the Node* canonical trait + context + result types
    /// resolve after the Component→Node hard rename. The Component*
    /// aliases were dropped in the same phase; their absence is
    /// enforced by the workspace audit (no live `Component*` ident
    /// remains in core / examples / tests).
    #[test]
    fn node_context_types_resolve() {
        // Canonical "Node*" trait + context names (post-rename).
        fn _take_node_ctx<N: crate::Node>(_: &mut crate::NodeContext<'_, dyn crate::NodeRuntime>) {}
        // Result type resolves.
        let _: crate::NodeResult<()> = Ok(());
    }
}

// ---------------------------------------------------------------------------
// phase-361 W8.e / issue 0594 — capabilities REQUIRE the heap / the standard
// library, they do not enable it. Turning `alloc` or `std` on for the user
// silently changes what their firmware image is; naming the feature they must
// add does not.
// ---------------------------------------------------------------------------
// issue 0687 follow-up — `alloc`, not `std`. The reason recorded here ("writes
// a file and exits") was never this crate's: the file write is `nros-cpp`'s
// `metadata_hooks`. What `metadata_mode.rs` actually needed was a `Sync`
// global, and it named `std::sync::Mutex` for it; on the portable mutex the
// capability is `String` + `format!` + a lock, i.e. the heap and nothing more.
#[cfg(all(feature = "metadata-mode", not(feature = "alloc")))]
compile_error!(
    "`metadata-mode` records into a heap-allocated global: add \"alloc\" to this crate's features"
);
// Emitted here as well as in `nros-node`, and NOT for the reason it first
// looks like. `nros = { features = ["env"] }` alone does not reach this line:
// `env` forwards to `nros-node/env`, nros-node is compiled first, and its own
// guard aborts the build there — measured, not assumed. What this covers is
// the case feature unification creates, where some other crate in the graph
// turns `nros-node/std` on so that guard stays quiet while THIS crate's `std`
// is still off. Rare, and exactly the shape that would otherwise compile a
// hosted capability into a build that never named the standard library.
#[cfg(all(feature = "env", not(feature = "std")))]
compile_error!(
    "`env` reads the process environment, which needs the standard library: add \"std\" to this crate's features"
);

// phase-379 W5 — the crate-level `QOS_PROFILE_*` presets (rclrs parity) and the
// older hand-written `nros::qos::*` module are two spellings of the same eight
// profiles. The first ALIASES `QoSProfile`'s associated consts; the second
// restates them as struct literals and predates them.
//
// They agree today. Nothing made them agree tomorrow, and a hand-mirrored
// constant drifting silently is the class issues 0088 / 0160 / 0245 all record.
// So assert it, at compile time where possible.
#[cfg(test)]
mod qos_preset_parity {
    use super::*;

    #[test]
    fn crate_level_presets_alias_the_associated_consts() {
        assert_eq!(QOS_PROFILE_DEFAULT, QoSProfile::QOS_PROFILE_DEFAULT);
        assert_eq!(QOS_PROFILE_SENSOR_DATA, QoSProfile::QOS_PROFILE_SENSOR_DATA);
        assert_eq!(QOS_PROFILE_PARAMETERS, QoSProfile::QOS_PROFILE_PARAMETERS);
        assert_eq!(
            QOS_PROFILE_ACTION_STATUS_DEFAULT,
            QoSProfile::QOS_PROFILE_ACTION_STATUS_DEFAULT
        );
    }

    /// The one that can actually rot: `qos::*` is a SEPARATE hand-written copy.
    #[test]
    fn qos_module_agrees_with_the_presets() {
        // issue 0829 FIXED — SYSTEM_DEFAULT joins the four that always agreed.
        // It is here rather than in its own pinned test because the two copies
        // no longer say anything a copy could get wrong: both alias the one
        // associated const, which is all sentinel.
        assert_eq!(
            qos::SYSTEM_DEFAULT,
            QOS_PROFILE_SYSTEM_DEFAULT,
            "qos::SYSTEM_DEFAULT drifted"
        );
        assert_eq!(qos::DEFAULT, QOS_PROFILE_DEFAULT, "qos::DEFAULT drifted");
        assert_eq!(
            qos::SENSOR_DATA,
            QOS_PROFILE_SENSOR_DATA,
            "qos::SENSOR_DATA drifted"
        );
        assert_eq!(
            qos::SERVICES_DEFAULT,
            QOS_PROFILE_SERVICES_DEFAULT,
            "qos::SERVICES_DEFAULT drifted"
        );
        assert_eq!(
            qos::PARAMETERS,
            QOS_PROFILE_PARAMETERS,
            "qos::PARAMETERS drifted"
        );
    }

    /// issue 0829, RESOLVED — this test used to PIN the divergence, asserting
    /// depth 10 on the façade side and depth 1 on the `nros-rmw` side because
    /// neither was obviously the live one. Both numbers are gone, and the
    /// replacement is not "we picked one": no concrete depth can be right,
    /// because the two reference RMWs resolve the same sentinel differently
    /// (`rmw_cyclonedds_cpp` → `KEEP_LAST, 1`; `rmw_zenoh_cpp` →
    /// `RMW_ZENOH_DEFAULT_HISTORY_DEPTH`, 42).
    ///
    /// So what is asserted now is the SHAPE: `SYSTEM_DEFAULT` states nothing,
    /// on every field. That is what makes it different from `DEFAULT` — the
    /// two being byte-identical was the older defect, and asserting they
    /// DIFFER is what keeps anyone from quietly aliasing them again.
    #[test]
    fn system_default_states_nothing_on_every_field() {
        use nros_rmw::{
            DEPTH_SYSTEM_DEFAULT, QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy,
            QoSReliabilityPolicy,
        };
        let sd = QOS_PROFILE_SYSTEM_DEFAULT;
        assert_eq!(sd.reliability, QoSReliabilityPolicy::SystemDefault);
        assert_eq!(sd.durability, QoSDurabilityPolicy::SystemDefault);
        assert_eq!(sd.history, QoSHistoryPolicy::SystemDefault);
        assert_eq!(sd.depth, DEPTH_SYSTEM_DEFAULT);
        // `None` IS the liveliness sentinel — it lowers to
        // `NROS_RMW_LIVELINESS_SYSTEM_DEFAULT` (0), the two having collapsed
        // onto one value in phase-376 W5/B2.
        assert_eq!(sd.liveliness_kind, QoSLivelinessPolicy::None);
        assert!(sd.has_unresolved_system_default());

        // The whole point of the name: it is NOT a synonym for DEFAULT.
        assert_ne!(
            sd, QOS_PROFILE_DEFAULT,
            "SYSTEM_DEFAULT aliased DEFAULT again — see issue 0829"
        );
        assert!(!QOS_PROFILE_DEFAULT.has_unresolved_system_default());
    }
}
