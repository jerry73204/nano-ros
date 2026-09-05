//! Embedded executor with build-time configured arena.
//!
//! Provides `Executor` and `Node` that work with the compile-time
//! selected RMW backend (zenoh, XRCE-DDS, or C FFI).
//!
//! # Example
//!
//! ```ignore
//! use nros_node::executor::*;
//! use std_msgs::msg::Int32;
//!
//! let config = ExecutorConfig::from_env().node_name("my_node");
//! let mut executor = Executor::open(&config)?;
//! let mut node = executor.create_node("my_node")?;
//!
//! let publisher = node.create_publisher::<Int32>("/chatter")?;
//! publisher.publish(&Int32 { data: 42 })?;
//!
//! loop {
//!     executor.spin_once(core::time::Duration::from_millis(10));
//! }
//! ```

#[cfg(any(has_rmw, test))]
pub mod action_core;
#[cfg(any(has_rmw, test))]
pub(crate) mod activator;
#[cfg(any(has_rmw, test))]
mod arena;
// Phase 8 (autoware-safety-island `docs/design/callback_tracing.rst`) —
// callback-level dispatch tracing. Same gating shape as `wake_probe`, for the
// same reason: hot-path hooks that must vanish in production. The module ALSO
// self-gates (`#![cfg(feature = "trace-callbacks")]`), so the call sites use
// paired stubs rather than naming it directly.
#[cfg(all(any(has_rmw, test), feature = "trace-callbacks"))]
pub mod callback_trace;
#[cfg(any(has_rmw, test))]
pub(crate) mod dispatcher;
#[cfg(any(has_rmw, test))]
mod handles;
// issue 0669 / phase-359 W10 — `pub mod handoff` stood here. `Handoff<M, N>`
// was optional sugar over `Arc<Mutex<heapless::Vec<M, N>>>` for the
// cross-priority bridge pattern (phase-104.E.3), it was `std`-gated on
// `std::sync::Mutex` alone, and it had no consumer anywhere in the tree — not
// one call site in `packages/` or `examples/` since it landed. Deleted rather
// than ported: the only portable mutex reachable from here costs a `spin`
// dependency edge on every build of this crate (measured: `spin` is in NO
// board's graph today), and an API with no consumer has no evidence its shape
// is the right one. The pattern it wrapped is six lines and is written out in
// issue 0669.
#[cfg(any(has_rmw, test))]
pub mod monitor;
#[cfg(any(has_rmw, test))]
mod node;
#[cfg(any(has_rmw, test))]
pub mod node_record;
#[cfg(any(has_rmw, test))]
mod node_wake;
// phase-359 W10 — the per-OS-priority worker pool, ported off `std::thread`
// onto the platform task ABI so it is reachable on every platform.
//
// Deliberately the SAME predicate as `node_wake` above, which it imports:
// spelling a second, hand-matched predicate here is how the two drift, and they
// did — an `all(feature = …, any(has_rmw, test))` copy resolved true in a build
// where `node_wake` resolved false. The feature half lives inside the file as
// an inner `#![cfg]`, so there is one condition per fact and no pair to keep in
// step.
#[cfg(any(has_rmw, test))]
pub(crate) mod os_priority;
// phase-359 W10 — the allocate/spawn/join helper moved to
// `nros_platform_api::task`, beside the ABI it wraps, once `nros-cpp` became a
// third caller. Reached through a `use` below rather than a module here.
#[cfg(any(has_rmw, test))]
pub(crate) mod ready_set;
pub mod sched_context;
#[cfg(any(has_rmw, test))]
mod spin;
#[cfg(any(has_rmw, test))]
pub(crate) mod spsc_ring;
#[cfg(any(has_rmw, test))]
mod storage;
#[cfg(any(has_rmw, test))]
pub(crate) mod triple_buffer;
mod types;
#[cfg(all(any(has_rmw, test), feature = "wake-latency-probe"))]
pub mod wake_probe;

#[cfg(any(has_rmw, test))]
pub mod action;

// MockSession-based tests. Disabled when any rmw-* feature is active because
// feature unification under `cargo test --workspace` flips `ConcreteSession`
// to a real backend handle (e.g. UorbSession when rmw-uorb is on transitively
// via the workspace), breaking the type signatures the tests expect.
// `feature = "std"` is part of the gate, not decoration. The crate is
// `#![no_std]`; `tests.rs` uses `std::` in 155 places and calls
// `from_session`, which is itself `#[cfg(feature = "alloc")]`. Without the
// feature in the gate, a plain `cargo test -p nros-node` compiles this module
// into a no_std crate and produces 252 errors, 192 of them "cannot find module
// or crate `std`" — an incomprehensible wall for anyone running the obvious
// command, and unrelated to whatever they were changing.
//
// These tests genuinely need a host: they spawn threads, sleep, and read the
// wall clock. Gating them is not hiding coverage, it is declaring what they
// already required. `cargo test -p nros-node --features std` is unchanged and
// remains the way to run them.
#[cfg(all(test, feature = "alloc", not(feature = "rmw-cffi")))]
mod tests;

// Flat re-exports so users write `executor::Executor` etc.
#[cfg(any(has_rmw, test))]
pub use action::{
    ActionClientRawHandle, ActionServerHandle, ActionServerRawHandle, RawActionClientSpec,
    RawActionServerSpec,
};
#[cfg(any(has_rmw, test))]
pub use action_core::{ActionClientCore, ActionServerCore, RawActiveGoal, action_channel_type};
#[cfg(any(has_rmw, test))]
pub use arena::TimerClockSource;
#[cfg(any(has_rmw, test))]
pub use arena::TimerOverrunPolicy;
#[cfg(any(has_rmw, test))]
pub use handles::*;
#[cfg(any(has_rmw, test))]
pub use node::{CallbackGroup, NodeHandle};
#[cfg(any(has_rmw, test))]
pub use node_record::{NodeBuilder, NodeId, NodeRecord};
#[cfg(any(has_rmw, test))]
pub use spin::Executor;
#[cfg(any(has_rmw, test))]
pub use spin::SessionHandle;
#[cfg(all(any(has_rmw, test), feature = "rmw-cffi"))]
pub use spin::SessionSpec;
#[cfg(any(has_rmw, test))]
pub use storage::{
    ExecutorInlineStorage, ExecutorSizing, executor_storage_layout, executor_storage_u64_len,
};
pub use types::*;
