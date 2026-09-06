//! Node abstraction for nros
//!
//! This crate provides the high-level Node API for creating ROS 2 compatible
//! publishers and subscribers on embedded systems.
//!
//! # Executor-Based API
//!
//! The executor-based API provides a unified interface that works on both
//! std (desktop) and no_std (embedded) targets.
//!
//! ## Desktop Example
//!
//! ```ignore
//! use nros::prelude::*;
//! use std_msgs::msg::Int32;
//!
//! let config = ExecutorConfig::from_env().node_name("my_node");
//! let mut executor: Executor = Executor::open(&config)?;
//!
//! // Register subscription callback
//! let node = executor.node_builder("my_node").build()?;
//! executor.node_mut(node).create_subscription::<Int32, _>("/topic", |msg: &Int32| {
//!     println!("Received: {}", msg.data);
//! })?;
//!
//! // Spin (processes callbacks)
//! executor.spin_blocking(SpinOptions::default());
//! ```
//!
//! ## Embedded Example
//!
//! ```ignore
//! use nros::prelude::*;
//! use std_msgs::msg::Int32;
//!
//! let config = ExecutorConfig { locator: "tcp/192.168.1.1:7447", ..Default::default() };
//! let mut executor: Executor = Executor::open(&config)?;
//!
//! // Register subscription callback
//! let node = executor.node_builder("my_node").build()?;
//! executor.node_mut(node).create_subscription::<Int32, _>("/cmd", |msg: &Int32| {
//!     // process message...
//! })?;
//!
//! // In your main loop:
//! loop {
//!     executor.spin_once(core::time::Duration::from_millis(10));
//!     // platform delay...
//! }
//! ```
//!
//! # Features
//!
//! - `std` - Enable standard library support (spin_blocking)
//! - `alloc` - Enable heap allocation (parameter service boxed replies)

#![no_std]

// phase-359 W10 — force the POSIX platform port into the UNIT-TEST link.
//
// `nros-platform-cffi` (with `posix-c-port`) is a host dev-dependency, but a
// dev-dep is only LINKED if something references the crate, and nothing in this
// crate's `src/` does — it reaches the platform through bare `extern "C"`
// declarations. So the C objects were never pulled in and every
// `nros_platform_*` symbol was undefined at test-link time, which is why
// `open_threaded`'s tests could not be run against a platform task until now.
// Issue 0612 records the same shape for `tests/signal_fd_wake.rs`, which this
// does NOT fix: that is a separate integration binary with its own link.
#[cfg(test)]
extern crate nros_platform_cffi as _;

// Phase 248 (C2) — `nros-rmw-cyclonedds[-sys]` deps removed (issue #60,
// Tier 1). Per-type descriptor registration is now the generic
// `nros_rmw::register_type_descriptor` seam (see `rmw_type_registry`);
// the Cyclone backend installs its registrar from its own crate. The
// `needs-type-descriptors` capability feature (no dep edge) still emits
// `cfg(rmw_needs_type_descriptors)` to compile the schema-passing body +
// `M: Message` bound for builds where a descriptor-needing backend is
// linked by the umbrella.

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

/// phase-412 -- the boot self-report, for boards with no reachable log sink.
pub mod boot_report;
pub mod c_waker;
pub mod config;
/// RFC-0088 / phase-421 W1 — the compile-time message-format check.
///
/// Gated exactly like [`session`], whose `IMAGE_SERIALIZATION_FORMAT_ID` it
/// compares against: with no RMW seam compiled in there is no backend, so
/// there is no format for a message to disagree with.
#[cfg(any(has_rmw, test))]
pub mod format_check;
/// Phase 212.K.7.6.b — runtime cyclonedds type-descriptor registry hook.
pub mod rmw_type_registry;

pub mod executor;
pub mod lifecycle;
pub mod limits;
pub mod names;
mod node;
mod publisher;
#[cfg(any(has_rmw, test))]
pub mod session;
mod subscriber;
/// phase-425 W3 — the `/clock` time source. Feature-gated: an image that will
/// never see a simulator should not carry the subscription or the message crate.
/// Also gated on `has_rmw`, like `session` and the entity API it uses: without a
/// backend there is no subscription to install, so the module would be a
/// conversion helper with no caller.
#[cfg(all(feature = "sim-time", any(has_rmw, test)))]
pub mod time_source;
pub mod timer;

// MockSession only matters when neither a real RMW backend feature
// nor lifecycle-services is enabled — the same gate as
// `session::ConcreteSession = MockSession` and the executor tests in
// `executor/mod.rs:42`. Compiling mock.rs unconditionally under
// `cfg(test)` produced "never constructed / never used" warnings on
// `cargo build --tests` when feature-unification activated a real
// RMW backend (e.g. workspace builds with `rmw-uorb` on).
#[cfg(all(test, not(feature = "rmw-cffi")))]
pub(crate) mod mock;

// Issue 0092 — the service servers these modules build (`executor::
// EmbeddedServiceServer`) only exist when an RMW backend is present
// (`#[cfg(any(has_rmw, test))]` on `executor::handles`). Gate the modules on
// `has_rmw` too — `--features {lifecycle,param}-services` with no RMW otherwise
// fails to resolve `EmbeddedServiceServer`. Service servers are meaningless
// without a backend; every shipping app/entry selects an RMW (→ has_rmw). The
// `test` arm keeps the modules in test builds.
#[cfg(all(feature = "param-services", any(has_rmw, test)))]
pub mod parameter_services;

/// phase-303 W4 (#0267) — construct the outbound CDR writer. Every tx path routes
/// through here so the wire encoding is chosen in ONE place.
///
/// **DEFAULT XCDR1 — corrected 2026-07-26 after live verification.** An earlier
/// version selected XCDR2 (DELIMITED_CDR2) for iron/jazzy+. That is WRONG: a
/// default Jazzy peer serializes its types FINAL/XCDR1 on the wire (verified
/// live + by `nros_serdes::cdr::tests::xcdr1_header_matches_live_jazzy_wire_bytes`),
/// and an APPENDABLE/XCDR2 writer is DDS-incompatible with its FINAL readers. So
/// nano-ros emits XCDR1 — byte-identical to a default Jazzy node. The XCDR2 path
/// (`new_with_header_xcdr2` + the generated DHEADER wrap) stays built for a
/// future PER-TYPE `@appendable` opt-in, not an edition blanket. See #0267.
#[inline]
pub(crate) fn tx_writer(buf: &mut [u8]) -> Result<nros_core::CdrWriter<'_>, nros_core::SerError> {
    nros_core::CdrWriter::new_with_header(buf)
}

// Re-export parameter types when param-services is enabled
#[cfg(feature = "param-services")]
pub use nros_params::{
    ParameterDescriptor, ParameterServer, ParameterType, ParameterValue, SetParameterResult,
};

#[cfg(all(feature = "lifecycle-services", any(has_rmw, test)))]
pub mod lifecycle_services;

// Export standalone node (without transport)
pub use node::{Node as StandaloneNode, NodeConfig, NodeError as StandaloneNodeError};

pub use publisher::PublisherHandle;
pub use subscriber::SubscriptionHandle;

// Re-export transport types for convenience
pub use nros_rmw::{
    ActionInfo, QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy, QoSPolicyMask,
    QoSProfile, QoSReliabilityPolicy, ServiceInfo, TopicInfo, TransportConfig, TransportError,
};

// Re-export RMW protocol traits so thin wrappers (nros-c, nros-cpp) can
// pull them through nros-node instead of going around it. Phase 91.B.
pub use nros_rmw::{
    ClientTrait, Publisher, ServiceTrait, Session, Subscription as SubscriptionTrait,
};

// Re-export action protocol types from nros-core. Same motivation as the
// RMW trait re-exports above — keeps thin wrappers off the
// nros-core::* path. Phase 91.B5.
pub use nros_core::{CancelResponse, CancelReturnCode, GoalId, GoalResponse, GoalStatus};

// Re-export lifecycle protocol types. Phase 91.B2.
pub use nros_core::lifecycle::{LifecycleState, LifecycleTransition, TransitionResult};

// Re-export CDR ser/de types so the C-side serialization helpers in
// nros-c/src/cdr.rs don't have to reach past nros-node either. These
// are themselves re-exports from nros-serdes via nros-core; collecting
// them here keeps the import boundary uniform. Phase 91.B6.
pub use nros_core::{
    CdrReader, CdrWriter, DHeaderMark, DHeaderScope, DeserError, EncodingVersion, SerError,
};

// Re-export safety types when feature is enabled
#[cfg(feature = "safety-e2e")]
pub use nros_rmw::{IntegrityStatus, SafetyValidator};

// Re-export publisher/subscriber options (topic + QoS; backend-agnostic).
pub use node::{PublisherOptions, SubscriptionOptions};

// Re-export session mode (used by ExecutorConfig)
pub use nros_rmw::SessionMode;

// Re-export timer types
pub use timer::{TimerCallbackFn, TimerDuration, TimerHandle, TimerMode, TimerState};

// Re-export lifecycle types
pub use lifecycle::{LifecycleCallbackFn, LifecycleError, LifecyclePollingNode};

// Re-export types that don't depend on RMW (always available)
pub use executor::{
    BOOT_SET_DOMAIN, BOOT_SET_LOCATOR, BOOT_SET_NAMESPACE, BOOT_SET_NODE_NAME, BOOT_SET_RMW,
    BakedBootConfig, BootConfig, BootConfigError, DOMAIN_ID_EXPLICIT_ZERO_C_ABI, DOMAIN_ID_MAX,
    EnvRung, ExecutorConfig, ExecutorSemantics, GuardCondition, HandleId, HandleSet,
    InvocationMode, NROS_BOOT_CONFIG_MAGIC, NROS_BOOT_CONFIG_VERSION, NodeError,
    RawAcceptedCallback, RawCancelCallback, RawGoalCallback, RawResponseCallback,
    RawServiceCallback, RawSubscriptionCallback, ReadinessSnapshot, ShutdownCallbackFn,
    ShutdownCallbackHandle, ShutdownPhase, SpinOnceResult, SpinOptions, SpinPeriodPollingResult,
    Trigger, baked_domain_from_c_abi,
};

// Re-export RMW-dependent executor types
#[cfg(any(has_rmw, test))]
pub use executor::{
    ActionClient, ActionClientCore, ActionServer, ActionServerCore, ActionServerHandle,
    ActionServerRawHandle, ActiveGoal, CallbackGroup, CompletedGoal, EmbeddedPublisher,
    EmbeddedRawPublisher, EmbeddedServiceClient, EmbeddedServiceServer, Executor, FeedbackStream,
    GoalFeedbackStream, LoanError, NodeHandle, Promise, PublishLoan, RawActionClientSpec,
    RawActionServerSpec, RawActiveGoal, RawServiceClient, RawServiceServer, RawSubscription,
    RecvView, SessionHandle, Subscription, action_channel_type, executor_storage_layout,
    executor_storage_u64_len,
};
#[cfg(any(has_rmw, test))]
pub use executor::{ExecutorInlineStorage, ExecutorSizing};

// issue 0687 — the selector's CAP (its reader lives at the hosted edge, in
// `nros::env`) and the hosted wall clock the edge installs on a resolved
// config. Both are consumed by `nros`, which builds what this crate takes.
pub use executor::{RMW_SELECTOR_CAP, default_epoch_us_fn};

// Phase 173.5 — bridge multi-session spec (consumed by the generated
// orchestration package's `Executor::open_multi`). Gated to match
// `executor::SessionSpec` (needs the cffi vtable surface).
#[cfg(all(any(has_rmw, test), feature = "rmw-cffi"))]
pub use executor::SessionSpec;

#[cfg(all(feature = "alloc", any(has_rmw, test)))]
pub use executor::SpinPeriodResult;

// ---------------------------------------------------------------------------
// phase-361 W8.e / issue 0594 — capabilities REQUIRE the heap / the standard
// library, they do not enable it. Turning `alloc` or `std` on for the user
// silently changes what their firmware image is; naming the feature they must
// add does not.
// ---------------------------------------------------------------------------
#[cfg(all(feature = "param-services", not(feature = "alloc")))]
compile_error!("`param-services` allocates: add \"alloc\" to this crate's features");
#[cfg(all(feature = "lifecycle-services", not(feature = "alloc")))]
compile_error!("`lifecycle-services` allocates: add \"alloc\" to this crate's features");
// phase-359 W10 — the forwarder's WORKER is a platform task now, not a
// `std::thread`, and it signals a `NodeWake` rather than a `Condvar`. Two of
// the three requirements are therefore no longer `std`: it needs the heap for
// its context (`alloc`) and a linked platform for `nros_platform_task_*` /
// `nros_platform_wake_*` (`rmw-cffi`, the same proxy `node_wake` uses).
//
// `std` REMAINS required, for one reason with a known end: the wake state it
// forwards into is still `WakeCtx`, the condvar-carrying type gated on `std`.
// Deleting that type — the campaign's next W10 step, and the one this port
// unblocked — is what removes this line.
#[cfg(all(feature = "signal-fd-wake", not(feature = "alloc")))]
compile_error!("`signal-fd-wake` allocates: add \"alloc\" to this crate's features");
#[cfg(all(feature = "signal-fd-wake", not(feature = "rmw-cffi")))]
compile_error!(
    "`signal-fd-wake` needs the platform task + wake ABI: add \"rmw-cffi\" to this crate's features"
);
// phase-359 W10 — this guard said `signal-fd-wake` "still reaches the std-gated
// `WakeCtx`: add \"std\" (phase-359 W10 removes this)". It did remove it: the
// wake context is `alloc`-gated, its worker is a platform task, and the only
// `std::` left in `WakeSignalFd` is a comment about what it stopped returning.
// What the feature actually needs is the allocator the context is built on.
#[cfg(all(feature = "signal-fd-wake", not(feature = "alloc")))]
compile_error!("`signal-fd-wake` builds on the `alloc`-gated wake context: add \"alloc\"");
// issue 0687 — `env` used to be declared here, and to require `std` for the
// same reason every capability in this block does. It is gone: reading the
// process environment moved to the hosted edge (`nros::env`), so the guard has
// nothing left to guard. What replaced it is a VALUE — `ExecutorConfig::
// resolve_with` takes an `EnvRung` — which needs no capability at all.
