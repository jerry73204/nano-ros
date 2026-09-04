//! Executor API for nros C API.
//!
//! Thin wrapper over `nros_node::Executor`. All dispatch logic, trigger
//! evaluation, LET semantics, and I/O driving are delegated to the Rust
//! executor — this module only handles C FFI translation.

use core::{
    ffi::{c_char, c_int},
    ptr,
};

use crate::{
    action::{
        ActionServerInternal, cancel_callback_trampoline, goal_callback_trampoline,
        nros_action_client_state_t, nros_action_client_t, nros_action_server_state_t,
        nros_action_server_t, nros_goal_status_t, nros_goal_uuid_t,
    },
    error::*,
    guard_condition::{nros_guard_condition_state_t, nros_guard_condition_t},
    node::nros_node_t,
    service::{
        client_response_trampoline, nros_client_state_t, nros_client_t, nros_service_state_t,
        nros_service_t,
    },
    subscription::{nros_subscription_state_t, nros_subscription_t},
    support::{nros_support_state_t, nros_support_t},
    timer::{nros_timer_state_t, nros_timer_t},
};

pub use crate::config::*;
use crate::constants::NROS_MAX_CONCURRENT_GOALS;

// ============================================================================
// Internal executor type
// ============================================================================

/// The concrete nros-node executor type used by the C API.
///
/// Sizes are configured via `NROS_EXECUTOR_MAX_CBS` and `NROS_EXECUTOR_ARENA_SIZE`
/// environment variables at build time (matching nros-node's build.rs).
// phase-271 — the executor borrows its per-entry storage (`Executor<'static>`);
// the C API keeps it heap-free by carving that backing from the SAME pinned
// `_opaque` buffer, laid out as [`nros_node::ExecutorInlineStorage`] (executor
// header at offset 0, backing tail). The executor still lives at offset 0, so
// [`get_executor`] / drop are unchanged.
pub(crate) type CExecutor = nros_node::Executor<'static>;

/// `u64` words of per-entry backing the inline executor carves from the tail of
/// its `_opaque` buffer (default sizing — same as the Rust `alloc` convenience).
#[cfg(feature = "rmw-cffi")]
pub(crate) const EXECUTOR_BACKING_U64S: usize = nros_node::ExecutorSizing::DEFAULT.u64_len();

// Compile-time assertion: inline opaque storage must fit the executor header
// PLUS its carved backing (the `ExecutorInlineStorage` layout).
#[cfg(feature = "rmw-cffi")]
const _: () = assert!(
    core::mem::size_of::<nros_node::ExecutorInlineStorage>()
        <= EXECUTOR_OPAQUE_U64S * core::mem::size_of::<u64>(),
    "EXECUTOR_OPAQUE_U64S too small for Executor + backing — this is a MEASUREMENT, not a budget. The stated size is \
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

/// Get a mutable reference to the internal executor from opaque storage.
///
/// # Safety
/// The opaque storage must contain a live, initialized `CExecutor`.
#[inline]
pub(crate) unsafe fn get_executor(opaque: &mut [u64; EXECUTOR_OPAQUE_U64S]) -> &mut CExecutor {
    &mut *(opaque.as_mut_ptr() as *mut CExecutor)
}

/// Get a mutable reference to the internal executor from a raw pointer.
///
/// Used by the action server module which stores a raw pointer to the
/// executor's opaque storage.
///
/// # Safety
/// `ptr` must point to the `_opaque` field of a live, initialized
/// `nros_executor_t`.
#[inline]
pub(crate) unsafe fn get_executor_from_ptr(ptr: *mut core::ffi::c_void) -> &'static mut CExecutor {
    &mut *(ptr as *mut CExecutor)
}

/// Propagate node identity from a C node into the executor before
/// registering an entity. The `add_*_raw_*` methods read
/// `self.node_name` / `self.namespace` to build the liveliness keyexpr;
/// without identity, no liveliness token is declared and rmw_zenoh
/// subscribers won't discover the entity.
///
/// # Safety
/// `node` must be NULL or point to an initialized `nros_node_t` with
/// valid `name_len` / `namespace_len`.
unsafe fn set_executor_node_identity(
    rust_exec: &mut CExecutor,
    node: crate::node::nros_node_ref_t,
) {
    // phase-379 W4 — resolve the IDENTITY against the executor's own
    // `NodeRecord`, rather than dereferencing a `*const nros_node_t` the entity
    // captured at init.
    //
    // This is the call that made the stored pointer load-bearing: it reads the
    // node's name and namespace, long after the entity was created, from
    // caller-owned storage the runtime does not control. On an RTOS that
    // storage can be a task stack `vTaskDelete` has already freed. The record
    // holds the same two strings and outlives the caller's struct.
    if !crate::node::node_ref_is_live(node) {
        return;
    }
    let id = nros_node::executor::NodeId::from_raw(node.node_id);
    let Some(record) = rust_exec.node(id) else {
        return;
    };
    let name = record.name.clone();
    let namespace = record.namespace.clone();
    rust_exec.set_node_identity(name.as_str(), namespace.as_str());
}

// ============================================================================
// C types (kept for API compatibility)
// ============================================================================

/// Trigger function type for executor.
///
/// A trigger function receives a boolean array indicating which handles have
/// data ready, along with the count of handles. It returns true if the executor
/// should process callbacks.
///
/// # Parameters
/// * `ready` - Pointer to boolean array (one per handle)
/// * `count` - Number of elements in the array
/// * `context` - User-provided context pointer
///
/// # Returns
/// * `true` if executor should process callbacks
/// * `false` if executor should skip processing
pub type nros_executor_trigger_t = Option<
    unsafe extern "C" fn(ready: *const bool, count: usize, context: *mut core::ffi::c_void) -> bool,
>;

/// Callback invocation mode
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_executor_handle_invocation_t {
    /// Only invoke callback when new data is available
    NROS_EXECUTOR_ON_NEW_DATA = 0,
    /// Always invoke callback (even with NULL data)
    NROS_EXECUTOR_ALWAYS = 1,
}

/// Executor data communication semantics
///
/// Defines when data is taken from DDS during spin operations.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_executor_semantics_t {
    /// RCLCPP executor semantics: Data is taken from DDS just before
    /// the corresponding callback is called.
    NROS_SEMANTICS_RCLCPP_EXECUTOR = 0,
    /// Logical Execution Time (LET) semantics: At one sampling point,
    /// new data of all ready subscriptions are taken from DDS.
    /// During sequential processing, the data from that sampling point
    /// is used. New data arriving after the sampling point is not
    /// considered until the next spin iteration.
    NROS_SEMANTICS_LOGICAL_EXECUTION_TIME = 1,
}

/// Executor state
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_executor_state_t {
    /// Not initialized
    NROS_EXECUTOR_STATE_UNINITIALIZED = 0,
    /// Initialized and ready
    NROS_EXECUTOR_STATE_INITIALIZED = 1,
    /// Currently spinning
    NROS_EXECUTOR_STATE_SPINNING = 2,
    /// Shutdown
    NROS_EXECUTOR_STATE_SHUTDOWN = 3,
}

/// Executor structure.
///
/// The executor delegates all dispatch logic to an internal
/// executor. The C struct retains state, timeout, and
/// per-type counters for API compatibility.
///
/// The internal executor is stored inline in `_opaque` — no heap
/// allocation is needed. The storage size is computed at build time
/// from `NROS_EXECUTOR_MAX_CBS` and `NROS_EXECUTOR_ARENA_SIZE`.
#[repr(C)]
pub struct nros_executor_t {
    /// Current state
    pub state: nros_executor_state_t,
    /// Timeout in nanoseconds for spin_some
    pub timeout_ns: u64,
    /// Data communication semantics
    pub semantics: nros_executor_semantics_t,
    /// Pointer to support context
    pub support: *const nros_support_t,
    /// Trigger function (NULL = default "any" trigger)
    pub trigger: nros_executor_trigger_t,
    /// User context for trigger function
    pub trigger_context: *mut core::ffi::c_void,
    /// Number of handles registered
    pub handle_count: usize,
    /// Maximum handles (configured at init)
    pub max_handles: usize,
    /// Number of subscription handles
    pub subscription_count: usize,
    /// Number of timer handles
    pub timer_count: usize,
    /// Number of service handles
    pub service_count: usize,
    /// Next invocation time in nanoseconds for drift-compensated spin_period
    pub invocation_time_ns: u64,
    /// Reentrancy guard: set to `true` while `spin_once` is dispatching
    /// callbacks. Blocking helpers (`nros_client_call`, `nros_action_send_goal`,
    /// etc.) check this flag and return `NROS_RET_REENTRANT` if set.
    pub in_dispatch: bool,
    /// Inline opaque storage for the executor.
    /// Managed by nros_executor_init/fini — no heap allocation needed.
    pub _opaque: [u64; EXECUTOR_OPAQUE_U64S],
}

impl Default for nros_executor_t {
    fn default() -> Self {
        Self {
            state: nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED,
            timeout_ns: 100_000_000, // 100ms default
            semantics: nros_executor_semantics_t::NROS_SEMANTICS_RCLCPP_EXECUTOR,
            support: ptr::null(),
            trigger: None,
            trigger_context: ptr::null_mut(),
            handle_count: 0,
            max_handles: NROS_EXECUTOR_MAX_HANDLES,
            subscription_count: 0,
            timer_count: 0,
            service_count: 0,
            invocation_time_ns: 0,
            in_dispatch: false,
            #[allow(clippy::large_stack_arrays)] // Intentional: inline opaque storage avoids heap
            _opaque: [0u64; EXECUTOR_OPAQUE_U64S],
        }
    }
}

/// Get a zero-initialized executor.
#[unsafe(no_mangle)]
pub extern "C" fn rclc_executor_get_zero_initialized_executor() -> nros_executor_t {
    nros_executor_t::default()
}

/// Initialize an executor.
///
/// # Parameters
/// * `executor` - Pointer to a zero-initialized executor
/// * `support` - Pointer to an initialized support context
/// * `max_handles` - Maximum number of handles (capped at NROS_EXECUTOR_MAX_HANDLES)
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if any pointer is NULL or max_handles is 0
/// * `NROS_RET_NOT_INIT` if support is not initialized
///
/// # Safety
/// * All pointers must be valid
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_init(
    executor: *mut nros_executor_t,
    support: *const nros_support_t,
    max_handles: usize,
) -> nros_ret_t {
    validate_not_null!(executor, support);

    if max_handles == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let executor = &mut *executor;
    let support_ref = &*support;

    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED,
        NROS_RET_BAD_SEQUENCE
    );
    validate_state!(
        support_ref,
        nros_support_state_t::NROS_SUPPORT_STATE_INITIALIZED
    );

    // Create the internal nros-node executor using a borrowed session pointer.
    // Written directly into inline opaque storage — no heap allocation.
    let session_ptr = support_ref.get_session_ptr();
    if session_ptr.is_null() {
        return NROS_RET_NOT_INIT;
    }

    // `mut` only needed under `feature = "env"` where the
    // env-var-driven primary-identity block below mutates it;
    // on `no_std` (FreeRTOS / NuttX / ThreadX) the mutation
    // path compiles out and `-D unused-mut` would otherwise
    // hard-fail every cmake build of the C examples.
    // phase-271 — carve the per-entry backing from the tail of this same
    // (pinned, caller-owned, never-moved) `_opaque` buffer, then write the
    // executor header at offset 0. No heap. SAFETY: `_opaque` is sized for
    // `ExecutorInlineStorage` (asserted above); the buffer outlives the executor
    // (C owns it for the program) so treating the backing as `&'static mut` is
    // sound, and the header/backing sub-regions are disjoint.
    let inline = executor._opaque.as_mut_ptr() as *mut nros_node::ExecutorInlineStorage;
    let backing: &'static mut [core::mem::MaybeUninit<u64>] =
        core::slice::from_raw_parts_mut((*inline).backing.as_mut_ptr(), EXECUTOR_BACKING_U64S);
    #[allow(unused_mut)]
    let mut rust_exec =
        CExecutor::from_session_ptr_in(session_ptr, backing, nros_node::ExecutorSizing::DEFAULT);
    // Phase 156 — populate executor's primary identity fields
    // so `NodeBuilder::resolve_session_slot` can return slot 0
    // when a C-side `nros_executor_node_init(rmw_name, ...)`
    // names the same backend the support session opened
    // against. Mirror env-var resolution `open_session` uses so
    // primary picks line up.
    // phase-359 W10 — `env`, not `std`: reading `$NROS_RMW` is the process
    // environment CAPABILITY, the same one `ExecutorConfig::from_env`,
    // `nros::init*` and `nros-node`'s selector already moved onto.
    #[cfg(feature = "env")]
    {
        // issue 0687 — the shared selector. This used to read `$NROS_RMW`
        // directly AND pass an empty string through as the identity when it was
        // unset; `rmw_selector` reports unset as `None`, and the
        // `unwrap_or_default` below preserves the empty-identity behaviour the
        // C path expects.
        let name = nros::rmw_selector().unwrap_or_default();
        let name = name.as_str();
        let support_locator =
            core::str::from_utf8_unchecked(&support_ref.locator[..support_ref.locator_len]);
        rust_exec.set_primary_identity(name, support_locator);
        // Issue 0656 — the domain, beside the identity, for the same reason.
        // `from_session_ptr_in` takes a session and no config, so the executor
        // floors its domain to 0; every entity the C binding declares through
        // an executor path then lands on `0/…` however `ROS_DOMAIN_ID` was set.
        // MEASURED, not reasoned: with the executor fix alone, a rebuilt
        // `c_action_server` under `ROS_DOMAIN_ID=42` still declared
        // `0/fibonacci/_action/…` on a live `rmw_zenohd`.
        rust_exec.set_domain_id(u32::from(support_ref.domain_id));
    }
    ptr::write(executor._opaque.as_mut_ptr() as *mut CExecutor, rust_exec);

    executor.max_handles = max_handles.min(NROS_EXECUTOR_MAX_HANDLES);
    executor.handle_count = 0;
    executor.support = support;
    executor.timeout_ns = 100_000_000; // 100ms default
    executor.state = nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED;

    NROS_RET_OK
}

/// Set the executor timeout.
///
/// # Safety
/// * `executor` must be a valid pointer to an initialized executor
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_executor_set_timeout(
    executor: *mut nros_executor_t,
    timeout_ns: u64,
) -> nros_ret_t {
    validate_not_null!(executor);

    let executor = &mut *executor;

    if executor.state == nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED
        || executor.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SHUTDOWN
    {
        return NROS_RET_NOT_INIT;
    }

    executor.timeout_ns = timeout_ns;
    NROS_RET_OK
}

/// Phase 124.F.3 — session-level connectivity probe.
///
/// Sends a wire-level round-trip ("is the peer / agent / router
/// reachable?") and waits up to `timeout_ms`. Mirrors micro-ROS's
/// `rmw_uros_ping_agent`. Useful for reconnect-on-link-loss
/// patterns: bare-metal code calls `ping(100)` periodically and
/// tears down / re-opens the session on timeout.
///
/// # Returns
/// * `NROS_RET_OK` — peer responded within budget.
/// * `NROS_RET_TIMEOUT` — no reply before `timeout_ms`.
/// * `NROS_RET_UNSUPPORTED` — active backend can't probe.
/// * `NROS_RET_NOT_INIT` — executor not initialised.
/// * `NROS_RET_INVALID_ARGUMENT` — `executor` is NULL.
///
/// # Safety
/// * `executor` must be a valid pointer to an initialized executor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_ping(
    executor: *mut nros_executor_t,
    timeout_ms: i32,
) -> nros_ret_t {
    validate_not_null!(executor);
    let exec_t = &mut *executor;
    if exec_t.state == nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED
        || exec_t.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SHUTDOWN
    {
        return NROS_RET_NOT_INIT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let exec = get_executor(&mut exec_t._opaque);
        match exec.ping(timeout_ms) {
            Ok(()) => NROS_RET_OK,
            Err(nros_node::NodeError::Transport(nros_rmw::TransportError::Timeout)) => {
                NROS_RET_TIMEOUT
            }
            Err(nros_node::NodeError::Transport(nros_rmw::TransportError::Unsupported)) => {
                NROS_RET_UNSUPPORTED
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = timeout_ms;
        NROS_RET_UNSUPPORTED
    }
}

/// phase-381 W4 — visit one node on the graph.
///
/// `node_namespace` is a ROS namespace (`"/"`, `"/demo"`). `enclave` is NULL
/// where the backend does not track one, which is what lets one call answer
/// both `rmw_get_node_names` and `rmw_get_node_names_with_enclaves`.
///
/// Every string is BORROWED for the duration of the call — copy anything you
/// keep. Return `false` to stop the enumeration early.
pub type nros_node_visit_fn = Option<
    unsafe extern "C" fn(
        ctx: *mut core::ffi::c_void,
        node_name: *const core::ffi::c_char,
        node_namespace: *const core::ffi::c_char,
        enclave: *const core::ffi::c_char,
    ) -> bool,
>;

/// phase-381 W4 — visit one name and the types on it.
///
/// `types_count` may legitimately be 0 on a partially discovered graph:
/// reporting the name without a type beats dropping it. Strings are BORROWED.
/// Return `false` to stop.
pub type nros_names_and_types_visit_fn = Option<
    unsafe extern "C" fn(
        ctx: *mut core::ffi::c_void,
        name: *const core::ffi::c_char,
        types: *const *const core::ffi::c_char,
        types_count: usize,
    ) -> bool,
>;

/// phase-381 W4 — every node on the graph, with its namespace.
///
/// A VISITOR rather than an out-array: upstream's `rcutils_string_array_t`
/// allocates two levels deep, there is no allocator here, and a
/// caller-provided buffer needs a bound the caller cannot know. Peak extra
/// memory is one entry, and a caller with its own limit stops by returning
/// `false`.
///
/// **Reports what has been DISCOVERED; never blocks.** The first call after
/// startup legitimately sees a partial graph — the backend keeps a standing
/// query fed by the spin loop. Poll rather than calling once and concluding: an
/// empty enumeration means "nobody seen yet", never "nobody exists".
///
/// # Returns
/// * `NROS_RET_OK` — enumeration ran (possibly visiting nothing).
/// * `NROS_RET_UNSUPPORTED` — this backend has no graph. DISTINCT from an
///   empty graph, deliberately.
/// * `NROS_RET_NOT_INIT` — executor not initialised.
/// * `NROS_RET_INVALID_ARGUMENT` — `executor` or `visit` is NULL.
///
/// # Safety
/// * `executor` must point to an initialised executor.
/// * `visit` must be callable for the duration of the call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_get_node_names(
    executor: *mut nros_executor_t,
    visit: nros_node_visit_fn,
    ctx: *mut core::ffi::c_void,
) -> nros_ret_t {
    validate_not_null!(executor);
    let Some(visit) = visit else {
        return NROS_RET_INVALID_ARGUMENT;
    };
    let exec_t = &mut *executor;
    if exec_t.state == nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED
        || exec_t.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SHUTDOWN
    {
        return NROS_RET_NOT_INIT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        const NAME_MAX: usize = 256;
        let exec = get_executor(&mut exec_t._opaque);
        let mut cb = |name: &str, ns: &str, enclave: Option<&str>| -> bool {
            // NUL-terminated on the stack: the C contract borrows for the call
            // only, so nothing is allocated on either side of the seam. A name
            // that does not fit is SKIPPED rather than truncated — a truncated
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
            visit(
                ctx,
                name_buf.as_ptr() as *const core::ffi::c_char,
                ns_buf.as_ptr() as *const core::ffi::c_char,
                enc_ptr,
            )
        };
        match exec.get_node_names(&mut cb) {
            Ok(()) => NROS_RET_OK,
            Err(nros_node::NodeError::Transport(nros_rmw::TransportError::Unsupported)) => {
                NROS_RET_UNSUPPORTED
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (visit, ctx);
        NROS_RET_UNSUPPORTED
    }
}

/// phase-381 W4 — every topic on the graph, with the types on it.
///
/// Called once per distinct TOPIC: a topic carrying two types is one call with
/// two entries, not two calls. `types_count` may legitimately be 0 on a
/// partially discovered graph — reporting the name without a type beats
/// dropping it.
///
/// Same discovery caveat as `nros_executor_get_node_names`: an empty
/// enumeration means "nobody seen yet", not "nobody exists".
///
/// # Returns
/// * `NROS_RET_OK` — enumeration ran (possibly visiting nothing).
/// * `NROS_RET_UNSUPPORTED` — this backend has no graph.
/// * `NROS_RET_NOT_INIT` — executor not initialised.
/// * `NROS_RET_INVALID_ARGUMENT` — `executor` or `visit` is NULL.
///
/// # Safety
/// * `executor` must point to an initialised executor.
/// * `visit` must be callable for the duration of the call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_get_topic_names_and_types(
    executor: *mut nros_executor_t,
    visit: nros_names_and_types_visit_fn,
    ctx: *mut core::ffi::c_void,
) -> nros_ret_t {
    nros_executor_names_and_types_impl(executor, visit, ctx, NamesAndTypesKind::Topics)
}

/// phase-381 W4 — every service on the graph, with its types. As
/// `nros_executor_get_topic_names_and_types`, over servers and clients.
///
/// # Safety
/// * `executor` must point to an initialised executor.
/// * `visit` must be callable for the duration of the call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_get_service_names_and_types(
    executor: *mut nros_executor_t,
    visit: nros_names_and_types_visit_fn,
    ctx: *mut core::ffi::c_void,
) -> nros_ret_t {
    nros_executor_names_and_types_impl(executor, visit, ctx, NamesAndTypesKind::Services)
}

/// phase-381 W4 — how many publishers are visible on `topic_name`.
///
/// `topic_name` is a ROS name (`"/chatter"`). The count reflects what has been
/// DISCOVERED, so it can be low right after startup and is never a proof of
/// absence.
///
/// # Safety
/// * `executor` must point to an initialised executor.
/// * `topic_name` must be a valid NUL-terminated string.
/// * `out_count` must be writable.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_count_publishers(
    executor: *mut nros_executor_t,
    topic_name: *const core::ffi::c_char,
    out_count: *mut usize,
) -> nros_ret_t {
    nros_executor_count_impl(executor, topic_name, out_count, CountKind::Publishers)
}

/// phase-381 W4 — how many subscribers are visible on `topic_name`. See
/// `nros_executor_count_publishers` for the caveats.
///
/// # Safety
/// * `executor` must point to an initialised executor.
/// * `topic_name` must be a valid NUL-terminated string.
/// * `out_count` must be writable.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_count_subscribers(
    executor: *mut nros_executor_t,
    topic_name: *const core::ffi::c_char,
    out_count: *mut usize,
) -> nros_ret_t {
    nros_executor_count_impl(executor, topic_name, out_count, CountKind::Subscribers)
}

/// Which enumeration `nros_executor_names_and_types_impl` runs.
///
/// The four entry points above are written LONGHAND, not generated by a macro,
/// and that is deliberate: cbindgen does not expand macros, so a
/// macro-generated `#[no_mangle]` function gets no header declaration and is
/// uncallable from C — which is the entire point of these. Measured, not
/// assumed: the macro version compiled, passed `just check c`, and produced
/// zero lines in `nros_generated.h`. The shared body lives in a private helper
/// instead, so there is still one implementation.
#[cfg(feature = "rmw-cffi")]
enum NamesAndTypesKind {
    Topics,
    Services,
}

#[cfg(feature = "rmw-cffi")]
enum CountKind {
    Publishers,
    Subscribers,
}

/// The shared body for the two names-and-types entry points.
///
/// # Safety
/// Same contract as its callers.
unsafe fn nros_executor_names_and_types_impl(
    executor: *mut nros_executor_t,
    visit: nros_names_and_types_visit_fn,
    ctx: *mut core::ffi::c_void,
    #[allow(unused_variables)] kind: NamesAndTypesKind,
) -> nros_ret_t {
    validate_not_null!(executor);
    let Some(visit) = visit else {
        return NROS_RET_INVALID_ARGUMENT;
    };
    let exec_t = &mut *executor;
    if exec_t.state == nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED
        || exec_t.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SHUTDOWN
    {
        return NROS_RET_NOT_INIT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        const NAME_MAX: usize = 256;
        const TYPES_MAX: usize = 8;
        let exec = get_executor(&mut exec_t._opaque);
        let mut cb = |name: &str, types: &[&str]| -> bool {
            // NUL-terminated on the stack; the C contract borrows for the call
            // only. A name or type that does not fit is SKIPPED rather than
            // truncated — a truncated type name is a different, plausible type.
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
            visit(
                ctx,
                name_buf.as_ptr() as *const core::ffi::c_char,
                ptrs.as_ptr(),
                n,
            )
        };
        let r = match kind {
            NamesAndTypesKind::Topics => exec.get_topic_names_and_types(&mut cb),
            NamesAndTypesKind::Services => exec.get_service_names_and_types(&mut cb),
        };
        match r {
            Ok(()) => NROS_RET_OK,
            Err(nros_node::NodeError::Transport(nros_rmw::TransportError::Unsupported)) => {
                NROS_RET_UNSUPPORTED
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (visit, ctx);
        NROS_RET_UNSUPPORTED
    }
}

/// The shared body for the two count entry points.
///
/// # Safety
/// Same contract as its callers.
unsafe fn nros_executor_count_impl(
    executor: *mut nros_executor_t,
    topic_name: *const core::ffi::c_char,
    out_count: *mut usize,
    #[allow(unused_variables)] kind: CountKind,
) -> nros_ret_t {
    validate_not_null!(executor);
    if topic_name.is_null() || out_count.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let exec_t = &mut *executor;
    if exec_t.state == nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED
        || exec_t.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SHUTDOWN
    {
        return NROS_RET_NOT_INIT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let Ok(topic) = core::ffi::CStr::from_ptr(topic_name).to_str() else {
            return NROS_RET_INVALID_ARGUMENT;
        };
        let exec = get_executor(&mut exec_t._opaque);
        let r = match kind {
            CountKind::Publishers => exec.count_publishers(topic),
            CountKind::Subscribers => exec.count_subscribers(topic),
        };
        match r {
            Ok(n) => {
                *out_count = n;
                NROS_RET_OK
            }
            Err(nros_node::NodeError::Transport(nros_rmw::TransportError::Unsupported)) => {
                NROS_RET_UNSUPPORTED
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (topic_name, out_count);
        NROS_RET_UNSUPPORTED
    }
}

/// phase-381 W4 — visit one discovered endpoint on a topic.
///
/// Strings are BORROWED for the duration of the call. Return `false` to stop.
///
/// No QoS: the GRANTED profile is what would answer "why is nothing arriving",
/// no backend can read one back yet, and reporting the remote's DECLARED
/// profile instead would be a confident wrong answer.
#[repr(C)]
pub struct nros_endpoint_info_t {
    /// Node that owns the endpoint.
    pub node_name: *const core::ffi::c_char,
    /// That node's namespace.
    pub node_namespace: *const core::ffi::c_char,
    /// Fully-qualified type on the wire, e.g. `"std_msgs/msg/Int32"`.
    pub topic_type: *const core::ffi::c_char,
    /// `true` for a publisher, `false` for a subscription.
    pub is_publisher: bool,
    /// 24-byte identity; all-zero when the backend has none.
    pub endpoint_gid: [u8; 24],
}

/// phase-381 W4 — visit one endpoint. Return `false` to stop.
pub type nros_endpoint_info_visit_fn = Option<
    unsafe extern "C" fn(ctx: *mut core::ffi::c_void, info: *const nros_endpoint_info_t) -> bool,
>;

/// The shared body for the four `*_by_node` entry points.
///
/// # Safety
/// Same contract as its callers.
unsafe fn nros_executor_by_node_impl(
    executor: *mut nros_executor_t,
    node_name: *const core::ffi::c_char,
    node_namespace: *const core::ffi::c_char,
    visit: nros_names_and_types_visit_fn,
    ctx: *mut core::ffi::c_void,
    #[allow(unused_variables)] kind: u8,
) -> nros_ret_t {
    validate_not_null!(executor);
    if node_name.is_null() || node_namespace.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let Some(visit) = visit else {
        return NROS_RET_INVALID_ARGUMENT;
    };
    let exec_t = &mut *executor;
    if exec_t.state == nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED
        || exec_t.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SHUTDOWN
    {
        return NROS_RET_NOT_INIT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let (Ok(name), Ok(ns)) = (
            core::ffi::CStr::from_ptr(node_name).to_str(),
            core::ffi::CStr::from_ptr(node_namespace).to_str(),
        ) else {
            return NROS_RET_INVALID_ARGUMENT;
        };
        const NAME_MAX: usize = 256;
        const TYPES_MAX: usize = 8;
        let exec = get_executor(&mut exec_t._opaque);
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
            visit(
                ctx,
                name_buf.as_ptr() as *const core::ffi::c_char,
                ptrs.as_ptr(),
                count,
            )
        };
        let r = match kind {
            0 => exec.get_publisher_names_and_types_by_node(name, ns, &mut cb),
            1 => exec.get_subscription_names_and_types_by_node(name, ns, &mut cb),
            2 => exec.get_service_names_and_types_by_node(name, ns, &mut cb),
            _ => exec.get_client_names_and_types_by_node(name, ns, &mut cb),
        };
        match r {
            Ok(()) => NROS_RET_OK,
            Err(nros_node::NodeError::Transport(nros_rmw::TransportError::Unsupported)) => {
                NROS_RET_UNSUPPORTED
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (node_name, node_namespace, visit, ctx);
        NROS_RET_UNSUPPORTED
    }
}

/// phase-381 W4 — what one named node PUBLISHES, with the types.
///
/// # Safety
/// * `executor` must point to an initialised executor.
/// * `node_name` / `node_namespace` must be valid NUL-terminated strings.
/// * `visit` must be callable for the duration of the call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_get_publisher_names_and_types_by_node(
    executor: *mut nros_executor_t,
    node_name: *const core::ffi::c_char,
    node_namespace: *const core::ffi::c_char,
    visit: nros_names_and_types_visit_fn,
    ctx: *mut core::ffi::c_void,
) -> nros_ret_t {
    nros_executor_by_node_impl(executor, node_name, node_namespace, visit, ctx, 0)
}

/// phase-381 W4 — what one named node SUBSCRIBES to, with the types.
///
/// **`subscriber`, not `subscription`** — rcl spells it
/// `rcl_get_subscriber_names_and_types_by_node`, and the C surface takes its
/// vocabulary from rcl so a user porting C ROS 2 code types what they already
/// know. The C++ and Rust surfaces say `subscription` because rclcpp and rclrs
/// do. Three layers, three upstreams, one word each — not drift. Issue 0788
/// owns the wider verb sweep.
///
/// # Safety
/// * `executor` must point to an initialised executor.
/// * `node_name` / `node_namespace` must be valid NUL-terminated strings.
/// * `visit` must be callable for the duration of the call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_get_subscriber_names_and_types_by_node(
    executor: *mut nros_executor_t,
    node_name: *const core::ffi::c_char,
    node_namespace: *const core::ffi::c_char,
    visit: nros_names_and_types_visit_fn,
    ctx: *mut core::ffi::c_void,
) -> nros_ret_t {
    nros_executor_by_node_impl(executor, node_name, node_namespace, visit, ctx, 1)
}

/// phase-381 W4 — what services one named node SERVES, with the types.
///
/// # Safety
/// As `nros_executor_get_publisher_names_and_types_by_node`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_get_service_names_and_types_by_node(
    executor: *mut nros_executor_t,
    node_name: *const core::ffi::c_char,
    node_namespace: *const core::ffi::c_char,
    visit: nros_names_and_types_visit_fn,
    ctx: *mut core::ffi::c_void,
) -> nros_ret_t {
    nros_executor_by_node_impl(executor, node_name, node_namespace, visit, ctx, 2)
}

/// phase-381 W4 — what services one named node CALLS, with the types.
///
/// # Safety
/// As `nros_executor_get_publisher_names_and_types_by_node`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_get_client_names_and_types_by_node(
    executor: *mut nros_executor_t,
    node_name: *const core::ffi::c_char,
    node_namespace: *const core::ffi::c_char,
    visit: nros_names_and_types_visit_fn,
    ctx: *mut core::ffi::c_void,
) -> nros_ret_t {
    nros_executor_by_node_impl(executor, node_name, node_namespace, visit, ctx, 3)
}

/// The shared body for the two `*_info_by_topic` entry points.
///
/// # Safety
/// Same contract as its callers.
unsafe fn nros_executor_endpoint_info_impl(
    executor: *mut nros_executor_t,
    topic_name: *const core::ffi::c_char,
    visit: nros_endpoint_info_visit_fn,
    ctx: *mut core::ffi::c_void,
    #[allow(unused_variables)] publishers: bool,
) -> nros_ret_t {
    validate_not_null!(executor);
    if topic_name.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let Some(visit) = visit else {
        return NROS_RET_INVALID_ARGUMENT;
    };
    let exec_t = &mut *executor;
    if exec_t.state == nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED
        || exec_t.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SHUTDOWN
    {
        return NROS_RET_NOT_INIT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let Ok(topic) = core::ffi::CStr::from_ptr(topic_name).to_str() else {
            return NROS_RET_INVALID_ARGUMENT;
        };
        const NAME_MAX: usize = 256;
        let exec = get_executor(&mut exec_t._opaque);
        let mut cb = |info: &nros_rmw::GraphEndpointInfo<'_>| -> bool {
            let mut name_buf = [0u8; NAME_MAX];
            let mut ns_buf = [0u8; NAME_MAX];
            let mut ty_buf = [0u8; NAME_MAX];
            if info.node_name.len() >= NAME_MAX
                || info.node_namespace.len() >= NAME_MAX
                || info.topic_type.len() >= NAME_MAX
            {
                return true; // skip, never truncate
            }
            name_buf[..info.node_name.len()].copy_from_slice(info.node_name.as_bytes());
            ns_buf[..info.node_namespace.len()].copy_from_slice(info.node_namespace.as_bytes());
            ty_buf[..info.topic_type.len()].copy_from_slice(info.topic_type.as_bytes());
            let c_info = nros_endpoint_info_t {
                node_name: name_buf.as_ptr() as *const core::ffi::c_char,
                node_namespace: ns_buf.as_ptr() as *const core::ffi::c_char,
                topic_type: ty_buf.as_ptr() as *const core::ffi::c_char,
                is_publisher: info.is_publisher,
                endpoint_gid: info.endpoint_gid,
            };
            visit(ctx, &c_info as *const nros_endpoint_info_t)
        };
        let r = if publishers {
            exec.get_publishers_info_by_topic(topic, &mut cb)
        } else {
            exec.get_subscriptions_info_by_topic(topic, &mut cb)
        };
        match r {
            Ok(()) => NROS_RET_OK,
            Err(nros_node::NodeError::Transport(nros_rmw::TransportError::Unsupported)) => {
                NROS_RET_UNSUPPORTED
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (topic_name, visit, ctx);
        NROS_RET_UNSUPPORTED
    }
}

/// phase-381 W4 — the publishers on `topic_name`, one visit each.
///
/// # Safety
/// * `executor` must point to an initialised executor.
/// * `topic_name` must be a valid NUL-terminated string.
/// * `visit` must be callable for the duration of the call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_get_publishers_info_by_topic(
    executor: *mut nros_executor_t,
    topic_name: *const core::ffi::c_char,
    visit: nros_endpoint_info_visit_fn,
    ctx: *mut core::ffi::c_void,
) -> nros_ret_t {
    nros_executor_endpoint_info_impl(executor, topic_name, visit, ctx, true)
}

/// phase-381 W4 — the subscriptions on `topic_name`, one visit each.
///
/// # Safety
/// As `nros_executor_get_publishers_info_by_topic`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_get_subscriptions_info_by_topic(
    executor: *mut nros_executor_t,
    topic_name: *const core::ffi::c_char,
    visit: nros_endpoint_info_visit_fn,
    ctx: *mut core::ffi::c_void,
) -> nros_ret_t {
    nros_executor_endpoint_info_impl(executor, topic_name, visit, ctx, false)
}

/// Set data communication semantics.
///
/// # Safety
/// * `executor` must be a valid pointer to an initialized executor
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_executor_set_semantics(
    executor: *mut nros_executor_t,
    semantics: nros_executor_semantics_t,
) -> nros_ret_t {
    validate_not_null!(executor);

    let executor = &mut *executor;

    if executor.state == nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED
        || executor.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SHUTDOWN
    {
        return NROS_RET_NOT_INIT;
    }

    executor.semantics = semantics;

    // Forward to the internal executor
    if executor.state == nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
        || executor.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SPINNING
    {
        let rust_exec = get_executor(&mut executor._opaque);
        rust_exec.set_semantics(match semantics {
            nros_executor_semantics_t::NROS_SEMANTICS_RCLCPP_EXECUTOR => {
                nros_node::ExecutorSemantics::RclcppExecutor
            }
            nros_executor_semantics_t::NROS_SEMANTICS_LOGICAL_EXECUTION_TIME => {
                nros_node::ExecutorSemantics::LogicalExecutionTime
            }
        });
    }

    NROS_RET_OK
}

/// Phase 104.C.8.b — initialize a Node via the executor's
/// [`node_builder`](nros_node::Executor::node_builder) chain.
///
/// Thin wrapper over Rust's
/// `executor.node_builder(name).rmw(...).locator(...).domain_id(...).
/// namespace(...).sched(...).build()`. Materialises a Node inside the
/// executor's node table and stores the returned NodeId in
/// `node.node_id` so subsequent
/// [`nros_executor_add_subscription`] / `_service` / `_client` /
/// `_action_*` calls route through `register_*_on(NodeId, ...)`
/// instead of the legacy single-Node path.
///
/// Replaces the pre-104.C ordering of `support_init → node_init →
/// executor_init` with the rclcpp-aligned `support_init → executor_init →
/// executor_node_init`. The old `rclc_node_init_default` / `nros_node_init_ex`
/// entry points are preserved for source compatibility — they still
/// drive the single-Node legacy path and leave `node.node_id = 0`.
///
/// # Parameters
/// * `executor` — Pointer to an initialised executor.
/// * `node` — Pointer to a zero-initialised node. Populated on success.
/// * `name` — Node name (null-terminated). Must not be NULL.
/// * `options` — Pointer to populated `nros_node_options_t`. NULL =
///   default options (no rmw override, inherits executor's locator
///   + domain, executor-default SchedContext).
///
/// # Returns
/// * `NROS_RET_OK` on success.
/// * `NROS_RET_INVALID_ARGUMENT` on NULL pointers / bad strings.
/// * `NROS_RET_BAD_SEQUENCE` if node is already initialised.
/// * `NROS_RET_NOT_INIT` if executor isn't initialised.
/// * `NROS_RET_ERROR` if the executor's node table is full
///   (`NROS_EXECUTOR_MAX_NODES`) or the backend session open failed.
///
/// # Safety
/// All pointer arguments must satisfy their per-parameter rules. `options`
/// length fields must not overrun their buffers.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_node_init(
    executor: *mut nros_executor_t,
    node: *mut nros_node_t,
    name: *const c_char,
    options: *const crate::node::nros_node_options_t,
) -> nros_ret_t {
    use crate::{
        constants::{MAX_LOCATOR_LEN, MAX_NAMESPACE_LEN, MAX_RMW_NAME_LEN},
        node::{NROS_DOMAIN_ID_INHERIT, nros_node_options_t, nros_node_state_t},
    };

    validate_not_null!(executor, node, name);

    let executor = &mut *executor;
    let node_ref = &mut *node;

    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    if node_ref.state != nros_node_state_t::NROS_NODE_STATE_UNINITIALIZED {
        return NROS_RET_BAD_SEQUENCE;
    }

    // Length-bound + copy node name into struct.
    node_ref.name_len = crate::util::copy_cstr_into(name, &mut node_ref.name);
    if node_ref.name_len == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }

    // Stack-copy a defaulted options struct when caller passed NULL,
    // so the rest of the function reads a uniform shape.
    let default_opts = nros_node_options_t::default();
    let opts = if options.is_null() {
        &default_opts
    } else {
        let opts_ref = &*options;
        if opts_ref.namespace_len > MAX_NAMESPACE_LEN
            || opts_ref.rmw_name_len > MAX_RMW_NAME_LEN
            || opts_ref.locator_len > MAX_LOCATOR_LEN
        {
            return NROS_RET_INVALID_ARGUMENT;
        }
        opts_ref
    };

    // Mirror options into node struct so subsequent helpers can read
    // namespace / rmw / domain_id without consulting `options` again.
    node_ref.namespace[..opts.namespace_len].copy_from_slice(&opts.namespace[..opts.namespace_len]);
    node_ref.namespace_len = opts.namespace_len;
    node_ref.rmw_name[..opts.rmw_name_len].copy_from_slice(&opts.rmw_name[..opts.rmw_name_len]);
    node_ref.rmw_name_len = opts.rmw_name_len;
    node_ref.domain_id_override = opts.domain_id_override;
    node_ref.sched_context_id = opts.sched_context_id;

    // Drive the Rust executor's NodeBuilder.
    let rust_exec = get_executor(&mut executor._opaque);
    let name_str = core::str::from_utf8_unchecked(&node_ref.name[..node_ref.name_len]);
    let mut builder = rust_exec.node_builder(name_str);
    if opts.rmw_name_len > 0 {
        builder = builder.rmw(core::str::from_utf8_unchecked(
            &opts.rmw_name[..opts.rmw_name_len],
        ));
    }
    if opts.locator_len > 0 {
        builder = builder.locator(core::str::from_utf8_unchecked(
            &opts.locator[..opts.locator_len],
        ));
    }
    if opts.domain_id_override != NROS_DOMAIN_ID_INHERIT {
        builder = builder.domain_id(opts.domain_id_override);
    }
    if opts.namespace_len > 0 {
        builder = builder.namespace(core::str::from_utf8_unchecked(
            &opts.namespace[..opts.namespace_len],
        ));
    }
    if opts.sched_context_id != 0 {
        builder = builder.sched(nros_node::executor::sched_context::SchedContextId(
            opts.sched_context_id,
        ));
    }
    let node_id = match builder.build() {
        Ok(id) => id,
        Err(_) => return NROS_RET_ERROR,
    };

    // Persist NodeId so handle-creation paths can hit the `_on()`
    // multi-Session variants. Support pointer stays NULL on this path —
    // legacy single-Node paths key off support, multi-Node paths key
    // off node_id + executor pointer (Phase 156 Sub-bug D).
    node_ref.node_id = node_id.raw();
    node_ref.support = core::ptr::null();
    node_ref.executor = executor as *const nros_executor_t;
    node_ref.state = nros_node_state_t::NROS_NODE_STATE_INITIALIZED;

    NROS_RET_OK
}

/// Set the trigger condition for the executor.
///
/// # Safety
/// * `executor` must be a valid pointer to an initialized executor
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_executor_set_trigger(
    executor: *mut nros_executor_t,
    trigger: nros_executor_trigger_t,
    context: *mut core::ffi::c_void,
) -> nros_ret_t {
    validate_not_null!(executor);

    let executor = &mut *executor;

    if executor.state == nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED
        || executor.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SHUTDOWN
    {
        return NROS_RET_NOT_INIT;
    }

    executor.trigger = trigger;
    executor.trigger_context = context;

    // Forward to the internal executor
    let rust_exec = get_executor(&mut executor._opaque);
    match trigger {
        Some(cb) => {
            rust_exec.set_trigger(nros_node::Trigger::RawPredicate {
                callback: cb,
                context,
            });
        }
        None => {
            rust_exec.set_trigger(nros_node::Trigger::Any);
        }
    }

    NROS_RET_OK
}

// ============================================================================
// Built-in trigger functions (kept as C-exported convenience wrappers)
// ============================================================================

/// Built-in trigger: fire when ANY handle has data ready.
///
/// # Safety
/// * `ready` must point to a valid array of at least `count` booleans
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_executor_trigger_any(
    ready: *const bool,
    count: usize,
    context: *mut core::ffi::c_void,
) -> bool {
    let _ = context;
    for i in 0..count {
        if *ready.add(i) {
            return true;
        }
    }
    false
}

/// Built-in trigger: fire when ALL handles have data ready.
///
/// # Safety
/// * `ready` must point to a valid array of at least `count` booleans
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_executor_trigger_all(
    ready: *const bool,
    count: usize,
    context: *mut core::ffi::c_void,
) -> bool {
    let _ = context;
    if count == 0 {
        return false;
    }
    for i in 0..count {
        if !*ready.add(i) {
            return false;
        }
    }
    true
}

/// Built-in trigger: always fire (unconditionally).
///
/// # Safety
/// * `ready` and `count` are unused
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_executor_trigger_always(
    ready: *const bool,
    count: usize,
    context: *mut core::ffi::c_void,
) -> bool {
    let _ = (ready, count, context);
    true
}

/// Built-in trigger: fire when the handle at the index stored in context has data.
///
/// `context` must point to a caller-owned `size_t` holding the handle
/// index. Passing `(void*)(size_t)idx` directly is NOT supported — that
/// pattern is UB on strict-alignment targets and CHERI, and the function
/// will dereference the pointer.
///
/// Recommended usage:
/// ```c
/// static size_t my_trigger_index = 2;
/// rclc_executor_set_trigger(&exec, rclc_executor_trigger_one, &my_trigger_index);
/// ```
///
/// # Safety
/// * `ready` must point to a valid array of at least `count` booleans.
/// * `context` must point to a valid `size_t` alive for the trigger's lifetime.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_executor_trigger_one(
    ready: *const bool,
    count: usize,
    context: *mut core::ffi::c_void,
) -> bool {
    if context.is_null() {
        return false;
    }
    let index = *(context as *const usize);
    if index < count {
        *ready.add(index)
    } else {
        false
    }
}

// ============================================================================
// Handle registration — delegated to nros-node Executor
// ============================================================================

/// Add a subscription to the executor.
///
/// Extracts metadata from the subscription struct and registers a raw-bytes
/// callback with the internal nros-node executor. The RMW subscriber handle
/// is created here (moved from subscription init).
///
/// # Safety
/// * All pointers must be valid and point to initialized objects
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_subscription(
    executor: *mut nros_executor_t,
    subscription: *mut nros_subscription_t,
    invocation: nros_executor_handle_invocation_t,
) -> nros_ret_t {
    validate_not_null!(executor, subscription);

    let executor = &mut *executor;
    let subscription_ref = &*subscription;

    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    validate_state!(
        subscription_ref,
        nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_INITIALIZED
    );

    // Check capacity
    if executor.handle_count >= executor.max_handles {
        return NROS_RET_FULL;
    }

    {
        let rust_exec = get_executor(&mut executor._opaque);

        // Extract metadata from subscription struct
        let topic_str = core::str::from_utf8_unchecked(
            &subscription_ref.topic_name[..subscription_ref.topic_name_len],
        );
        let type_str = core::str::from_utf8_unchecked(
            &subscription_ref.type_name[..subscription_ref.type_name_len],
        );
        let type_hash_str = core::str::from_utf8_unchecked(
            &subscription_ref.type_hash[..subscription_ref.type_hash_len],
        );

        // Get QoS settings from the subscription
        let qos = subscription_ref.get_qos_settings();

        // Get callback and context
        let callback = match subscription_ref.get_callback() {
            Some(cb) => cb,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        let context = subscription_ref.get_context();

        // Propagate node identity into the executor so the underlying
        // create_subscription call gets liveliness keyexpr metadata.
        set_executor_node_identity(rust_exec, subscription_ref.node);
        // Phase 305 W3 (issue 0255) — resolve `~`/relative names + launch remaps
        // against the identity just set (executor-side remap table).
        let __resolved_name = match rust_exec.resolve_entity_name(topic_str) {
            Ok(r) => r,
            Err(()) => return NROS_RET_INVALID_ARGUMENT,
        };
        let topic_str = __resolved_name.as_str();

        // Phase 104.C.8.b — when the Node was created via
        // `nros_executor_node_init`, route through `_on(NodeId, ...)`
        // so multi-RMW bridges land on the right session. Legacy
        // `rclc_node_init_default`-style Nodes carry `node_id == 0` and fall
        // through to the single-Node entry point.
        let node_raw_id = if !subscription_ref.node.is_bound() {
            0
        } else {
            subscription_ref.node.node_id
        };
        // Phase 189.M2.b — the single kept C-FFI subscription core.
        let node_id =
            (node_raw_id != 0).then(|| nros_node::executor::NodeId::from_raw(node_raw_id));
        let result = rust_exec.add_arena_subscription_c_callback::<MESSAGE_BUFFER_SIZE>(
            node_id,
            topic_str,
            type_str,
            type_hash_str,
            qos,
            callback,
            context,
            None, // Phase 273 W3: group threading is via nros_executor_add_subscription_in_group
            // phase-402 W2 — this path states no hint; 0 = no opinion.
            0,
        );

        match result {
            Ok(handle_id) => {
                // Store the handle ID in the subscription for later reference
                let sub_mut = &mut *subscription;
                sub_mut.set_handle_id(handle_id);

                // Set invocation mode
                if invocation == nros_executor_handle_invocation_t::NROS_EXECUTOR_ALWAYS {
                    rust_exec.set_invocation(handle_id, nros_node::InvocationMode::Always);
                }

                // Phase 189.M3 — apply a scheduling-context binding
                // requested via `nros_subscription_init_with_options`.
                // `0` = inherit the default (no-op). A non-zero slot
                // must be a valid id from
                // `nros_executor_create_sched_context`; an unknown id
                // fails the registration so the caller learns the
                // binding was rejected rather than silently dropped.
                let requested_sc = sub_mut.sched_context_id;
                if requested_sc != 0 {
                    let sc_id = nros_node::executor::sched_context::SchedContextId(requested_sc);
                    if rust_exec
                        .bind_handle_to_sched_context(handle_id, sc_id)
                        .is_err()
                    {
                        return NROS_RET_INVALID_ARGUMENT;
                    }
                }

                executor.handle_count += 1;
                executor.subscription_count += 1;
                NROS_RET_OK
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
}

/// phase-417 W5.a — the deserialiser handed to
/// [`nros_executor_add_subscription_typed`].
///
/// Writes the CDR bytes at `buffer[..buffer_size]` into `msg`, which is storage
/// the CALLER owns. Returns `0` on success, non-zero on failure. This is the
/// type-erased form of the generated `<Msg>_deserialize`, which already has
/// exactly this contract; generated headers emit a `<Msg>_deserialize_erased`
/// with this signature so no call goes through a cast function pointer.
pub type nros_message_deserialize_fn_t = Option<
    unsafe extern "C" fn(msg: *mut core::ffi::c_void, buffer: *const u8, buffer_size: usize) -> i32,
>;

/// phase-417 W5.a — a subscription callback that receives the DESERIALISED
/// message, the shape rclc delivers
/// (`rclc_subscription_callback_with_context_t`).
///
/// `msg` is the caller's own storage, populated by this subscription's
/// [`nros_message_deserialize_fn_t`] immediately before the call. It is valid
/// for the duration of the call and is overwritten by the next dispatch — copy
/// out anything retained.
///
/// The `context` parameter is why this is the analog of rclc's
/// *with-context* callback rather than its bare `void (*)(const void *)`: C has
/// no closures and every other callback in this API already carries one.
pub type nros_typed_subscription_callback_t =
    Option<unsafe extern "C" fn(msg: *const core::ffi::c_void, context: *mut core::ffi::c_void)>;

/// phase-417 W5.a (RFC-0087 stage 5) — add a subscription that delivers a
/// **deserialised message** into storage the CALLER owns.
///
/// The rclc-shaped registration:
///
/// ```c
/// rclc_executor_add_subscription(&exec, &sub, &msg, &cb, ON_NEW_DATA);
/// rclc_executor_add_subscription_with_context(&exec, &sub, &msg, &cb, &ctx, ON_NEW_DATA);
/// nros_executor_add_subscription_typed(&exec, &sub, &msg, MyMsg_deserialize_erased,
///                                      &cb, &ctx, NROS_EXECUTOR_ON_NEW_DATA);
/// ```
///
/// One argument more than rclc's, and it is the only one: `nros_message_type_t`
/// carries a type NAME and a type HASH and no deserialiser, so the function
/// that writes this type into `msg` has to arrive from somewhere. Generated
/// headers collapse it away, so a ported line is rclc's six arguments in rclc's
/// order:
/// `MyMsg_executor_add_subscription(&exec, &sub, &msg, &cb, &ctx, NROS_EXECUTOR_ON_NEW_DATA)`.
///
/// The CONTEXT-LESS `rclc_executor_add_subscription` (a `void (*)(const void*)`
/// callback) is deliberately absent: adapting it needs a trampoline that
/// remembers the original function pointer, and state in the wrapper is
/// RFC-0019/0020's violation rather than an ergonomic. `nros/rcl_compat.h`
/// records the same refusal.
///
/// **No allocator is involved, and none is needed.** The caller owning the
/// storage IS the mechanism — the same one rclc uses. The C ledger recorded our
/// byte-oriented delivery as forced by "no allocator"; the compared surface
/// operates under the same constraint, so the stated reason was refuted by the
/// thing it was compared against (issue 1022, W-C3).
///
/// The byte-oriented [`nros_executor_add_subscription`] is untouched and stays.
/// This is additive; a raw subscriber is a legitimate thing to want.
///
/// # What the callback sees when decoding fails
/// **Nothing — it is not called.** A non-zero return from `deserialize` drops
/// the sample, leaves `msg` as it was, counts a `subscription_errors` in the
/// spin result, and emits a rate-limited `nros_log` error naming the sample
/// length and the return code. Dispatching anyway would hand the callback the
/// PREVIOUS message under the impression it was the new one, which is exactly
/// the "compile and differ" RFC-0087 forbids.
///
/// A message too large for the caller's storage arrives here as that same
/// failure and never as a truncation: a bounded string whose wire length
/// exceeds its declared capacity fails in `nros_cdr_read_string`
/// (`str_len > max_len`), and a bounded sequence fails on
/// `len > capacity` — both return `-1` rather than writing a short value.
///
/// # The subscription's own callback and context are not used on this path
/// `subscription->callback` and `subscription->context` (given to
/// `nros_subscription_init`) belong to the RAW registration and are **not read
/// here**: the typed callback and its `context` are supplied at registration,
/// exactly as rclc's `rclc_executor_add_subscription_with_context` does. Taking
/// the context from two places would be two sources for one value and therefore
/// a place for them to disagree.
///
/// What IS read from the subscription is what describes the ENTITY — topic,
/// type name, type hash, QoS, node binding, and the scheduling context
/// requested via `nros_subscription_init_with_options`.
///
/// `nros_subscription_init` currently rejects a NULL callback, so a typed-only
/// user must still pass one that never fires. That belongs to
/// `nros/subscription.h`'s owner, not here; see the W5.a report.
///
/// # Safety
/// * `executor` and `subscription` must be valid, initialised objects.
/// * `msg` must point to writable storage of the type `deserialize` writes, and
///   must stay valid for as long as the subscription is registered. Nothing
///   here allocates it, copies it, or frees it.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_subscription_typed(
    executor: *mut nros_executor_t,
    subscription: *mut nros_subscription_t,
    msg: *mut core::ffi::c_void,
    deserialize: nros_message_deserialize_fn_t,
    callback: nros_typed_subscription_callback_t,
    context: *mut core::ffi::c_void,
    invocation: nros_executor_handle_invocation_t,
) -> nros_ret_t {
    nros_executor_add_subscription_typed_sized(
        executor,
        subscription,
        msg,
        deserialize,
        callback,
        context,
        invocation,
        0,
    )
}

/// phase-417 W5.a — [`nros_executor_add_subscription_typed`] with the
/// receive-buffer hint stated by the caller.
///
/// `rx_bytes` is the same hint the raw path takes (issue 0896 / phase-403 W5):
/// the bytes this subscription expects to receive, sizing BOTH the backend's
/// payload size class and the executor's arena slot. `0` means "this caller
/// states nothing" and falls back to the image default — never a claim of zero
/// bytes. A typed caller normally HAS the number, because its message type
/// computes one (`<Msg>_RX_MAX_SERIALIZED_SIZE`), which is why the generated
/// macro passes it and the plain form above exists only for a type with no
/// bound.
///
/// # Safety
/// See [`nros_executor_add_subscription_typed`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_subscription_typed_sized(
    executor: *mut nros_executor_t,
    subscription: *mut nros_subscription_t,
    msg: *mut core::ffi::c_void,
    deserialize: nros_message_deserialize_fn_t,
    callback: nros_typed_subscription_callback_t,
    context: *mut core::ffi::c_void,
    invocation: nros_executor_handle_invocation_t,
    rx_bytes: u32,
) -> nros_ret_t {
    validate_not_null!(executor, subscription, msg);

    let (Some(deserialize_fn), Some(callback_fn)) = (deserialize, callback) else {
        return NROS_RET_INVALID_ARGUMENT;
    };
    let Some(msg_nn) = core::ptr::NonNull::new(msg) else {
        return NROS_RET_INVALID_ARGUMENT;
    };

    let executor = &mut *executor;
    let subscription_ref = &*subscription;

    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    validate_state!(
        subscription_ref,
        nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_INITIALIZED
    );

    if executor.handle_count >= executor.max_handles {
        return NROS_RET_FULL;
    }

    {
        let rust_exec = get_executor(&mut executor._opaque);

        let topic_str = core::str::from_utf8_unchecked(
            &subscription_ref.topic_name[..subscription_ref.topic_name_len],
        );
        let type_str = core::str::from_utf8_unchecked(
            &subscription_ref.type_name[..subscription_ref.type_name_len],
        );
        let type_hash_str = core::str::from_utf8_unchecked(
            &subscription_ref.type_hash[..subscription_ref.type_hash_len],
        );

        let qos = subscription_ref.get_qos_settings();

        set_executor_node_identity(rust_exec, subscription_ref.node);
        // Phase 305 W3 (issue 0255) — `~`/relative names + launch remaps, the
        // same resolution the raw path does. Doing it here rather than letting
        // the typed path skip it is what keeps the two registrations from being
        // two different name-resolution answers.
        let __resolved_name = match rust_exec.resolve_entity_name(topic_str) {
            Ok(r) => r,
            Err(()) => return NROS_RET_INVALID_ARGUMENT,
        };
        let topic_str = __resolved_name.as_str();

        let node_raw_id = if !subscription_ref.node.is_bound() {
            0
        } else {
            subscription_ref.node.node_id
        };
        let node_id =
            (node_raw_id != 0).then(|| nros_node::executor::NodeId::from_raw(node_raw_id));

        let result = rust_exec.add_arena_subscription_c_typed_callback::<MESSAGE_BUFFER_SIZE>(
            node_id,
            topic_str,
            type_str,
            type_hash_str,
            qos,
            msg_nn,
            deserialize_fn,
            callback_fn,
            context,
            None,
            rx_bytes as usize,
        );

        match result {
            Ok(handle_id) => {
                let sub_mut = &mut *subscription;
                sub_mut.set_handle_id(handle_id);

                if invocation == nros_executor_handle_invocation_t::NROS_EXECUTOR_ALWAYS {
                    rust_exec.set_invocation(handle_id, nros_node::InvocationMode::Always);
                }

                let requested_sc = sub_mut.sched_context_id;
                if requested_sc != 0 {
                    let sc_id = nros_node::executor::sched_context::SchedContextId(requested_sc);
                    if rust_exec
                        .bind_handle_to_sched_context(handle_id, sc_id)
                        .is_err()
                    {
                        return NROS_RET_INVALID_ARGUMENT;
                    }
                }

                executor.handle_count += 1;
                executor.subscription_count += 1;
                NROS_RET_OK
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
}

/// phase-417 stage 6 — register a BYTE-oriented subscription, supplying its
/// callback HERE rather than at `*_init`.
///
/// This is the raw-path sibling of [`nros_executor_add_subscription_typed`].
/// `rclc_subscription_init_default` takes no callback (rclc's arity), so the
/// byte callback that used to ride on `nros_subscription_init` is supplied at
/// registration, which is where rclc supplies it
/// (`rclc_executor_add_subscription`). The name keeps the `nros_` prefix
/// deliberately: rclc's `add_subscription` delivers a DESERIALIZED message into
/// caller storage, so the byte path has no upstream counterpart and must not
/// wear its name (RFC-0087's compile-or-conform rule — a plausible name over an
/// opposite data contract is the defect the RFC exists to prevent).
///
/// `callback` may be NULL, which registers the subscription with nothing to
/// dispatch — the same state `nros_executor_add_subscription` sees for a
/// subscription initialised by `rclc_subscription_init_default` alone.
///
/// # Safety
/// * `executor` and `subscription` must be valid, initialised objects.
/// * `context` is passed through untouched and may be NULL.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_subscription_raw(
    executor: *mut nros_executor_t,
    subscription: *mut nros_subscription_t,
    callback: crate::subscription::nros_subscription_callback_t,
    context: *mut core::ffi::c_void,
    invocation: nros_executor_handle_invocation_t,
) -> nros_ret_t {
    validate_not_null!(executor, subscription);
    (*subscription).callback = callback;
    (*subscription).context = context;
    nros_executor_add_subscription(executor, subscription, invocation)
}

/// Phase 189.M3.4 — register a raw subscription whose callback also receives
/// the sample's wire **attachment** (the C analog of the Rust
/// `node.subscription(t).generic(..).message_info()` builder; rclc's
/// generic-with-info subscription). Direct-arg form (no `nros_subscription_t`
/// struct): the callback signature differs from the plain
/// [`nros_subscription_callback_t`], so this is its own entry point rather than
/// a flag on `nros_executor_add_subscription`.
///
/// `node` may be NULL (legacy single-Node path) or a Node created via
/// `nros_executor_node_init` (routes to that Node's session). `qos` may be NULL
/// (defaults). Cross-RMW bridges read the `bridge_origin` tag from the
/// attachment for echo suppression.
///
/// # Safety
/// All non-NULL pointers must be valid; the C strings must be NUL-terminated
/// UTF-8 valid for the duration of the call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_subscription_raw_with_info(
    executor: *mut nros_executor_t,
    node: *const nros_node_t,
    topic_name: *const c_char,
    type_name: *const c_char,
    type_hash: *const c_char,
    qos: *const crate::qos::nros_qos_t,
    callback: crate::subscription::nros_subscription_info_callback_t,
    context: *mut core::ffi::c_void,
) -> nros_ret_t {
    validate_not_null!(executor, topic_name, type_name, type_hash);
    let Some(cb) = callback else {
        return NROS_RET_INVALID_ARGUMENT;
    };

    let executor = &mut *executor;
    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    if executor.handle_count >= executor.max_handles {
        return NROS_RET_FULL;
    }

    let (Ok(topic_str), Ok(type_str), Ok(type_hash_str)) = (
        core::ffi::CStr::from_ptr(topic_name).to_str(),
        core::ffi::CStr::from_ptr(type_name).to_str(),
        core::ffi::CStr::from_ptr(type_hash).to_str(),
    ) else {
        return NROS_RET_INVALID_ARGUMENT;
    };

    let qos_settings = if qos.is_null() {
        nros_node::QoSProfile::default()
    } else {
        (*qos).to_qos_settings()
    };

    {
        let rust_exec = get_executor(&mut executor._opaque);
        set_executor_node_identity(rust_exec, unsafe { crate::node::node_ref_of(node) });
        // Phase 305 W3 (issue 0255) — resolve `~`/relative names + launch remaps
        // against the identity just set (executor-side remap table).
        let __resolved_name = match rust_exec.resolve_entity_name(topic_str) {
            Ok(r) => r,
            Err(()) => return NROS_RET_INVALID_ARGUMENT,
        };
        let topic_str = __resolved_name.as_str();
        let node_raw_id = if node.is_null() { 0 } else { (*node).node_id };
        let node_id =
            (node_raw_id != 0).then(|| nros_node::executor::NodeId::from_raw(node_raw_id));
        let result = rust_exec.add_arena_subscription_c_info_callback::<MESSAGE_BUFFER_SIZE>(
            node_id,
            topic_str,
            type_str,
            type_hash_str,
            qos_settings,
            cb,
            context,
            // phase-408 W5a — this entry point predates the options struct and
            // has no hint to forward, so it states none. 0 is "no opinion", not
            // a claim of zero bytes: the subscription keeps the small payload
            // class it has always taken. A C caller that wants the derived size
            // goes through the options-carrying registration.
            0,
        );
        match result {
            Ok(_handle_id) => {
                executor.handle_count += 1;
                executor.subscription_count += 1;
                NROS_RET_OK
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
}

/// Add a timer to the executor.
///
/// Wraps the C timer callback in a closure and registers it with the
/// internal nros-node executor.
///
/// # Safety
/// * All pointers must be valid and point to initialized objects
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_executor_add_timer(
    executor: *mut nros_executor_t,
    timer: *mut nros_timer_t,
) -> nros_ret_t {
    validate_not_null!(executor, timer);

    let executor = &mut *executor;
    let timer_ref = &*timer;

    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    validate_state!(timer_ref, nros_timer_state_t::NROS_TIMER_STATE_RUNNING);

    // Check capacity
    if executor.handle_count >= executor.max_handles {
        return NROS_RET_FULL;
    }

    {
        let rust_exec = get_executor(&mut executor._opaque);

        // Get the C callback and context from the timer
        let c_callback = match timer_ref.get_callback() {
            Some(cb) => cb,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        let c_context = timer_ref.get_context();
        let timer_ptr = timer;

        // Wrap the C callback in a Rust closure
        let wrapper = move || {
            // SAFETY: The C callback and timer/context pointers remain valid
            // for the lifetime of the executor (same guarantee as rclc).
            c_callback(timer_ptr, c_context);
        };

        // Issue #505 — this used to truncate ns to MILLISECONDS, so the
        // ABI promised nanoseconds and rejected every sub-millisecond
        // period with INVALID_ARGUMENT (while the Rust API silently
        // turned the same request into a fire-every-spin timer). The
        // executor's timer accounting is microsecond-based, so convert
        // to microseconds and reject only a genuinely sub-microsecond
        // request.
        let period_us = timer_ref.period_ns / 1_000;
        if period_us == 0 {
            return NROS_RET_INVALID_ARGUMENT;
        }

        // Register with the nros-node executor
        let period = nros_node::TimerDuration::from_micros(period_us);
        match rust_exec.register_timer(period, wrapper) {
            Ok(handle_id) => {
                // Store handle ID and executor pointer for cancel/reset operations
                let timer_mut = &mut *timer;
                timer_mut.set_handle_id(handle_id);
                timer_mut.set_executor_ptr(executor._opaque.as_mut_ptr() as *mut core::ffi::c_void);

                executor.handle_count += 1;
                executor.timer_count += 1;
                NROS_RET_OK
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
}

/// Phase 273 (RFC-0047) — register a subscription in a named callback group.
///
/// Identical to `nros_executor_add_subscription` but additionally passes
/// the group name to the executor so the seeded `group_sched_table` can bind
/// the callback to the group's `SchedContext`. `callback_group` may be NULL or
/// an empty string — both behave identically to `nros_executor_add_subscription`.
///
/// # Safety
/// All non-NULL pointers must be valid and point to initialized objects.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_subscription_in_group(
    executor: *mut nros_executor_t,
    subscription: *mut nros_subscription_t,
    invocation: nros_executor_handle_invocation_t,
    callback_group: *const c_char,
) -> nros_ret_t {
    validate_not_null!(executor, subscription);

    let executor = &mut *executor;
    let subscription_ref = &*subscription;

    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    validate_state!(
        subscription_ref,
        nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_INITIALIZED
    );

    if executor.handle_count >= executor.max_handles {
        return NROS_RET_FULL;
    }

    {
        let rust_exec = get_executor(&mut executor._opaque);

        let topic_str = core::str::from_utf8_unchecked(
            &subscription_ref.topic_name[..subscription_ref.topic_name_len],
        );
        let type_str = core::str::from_utf8_unchecked(
            &subscription_ref.type_name[..subscription_ref.type_name_len],
        );
        let type_hash_str = core::str::from_utf8_unchecked(
            &subscription_ref.type_hash[..subscription_ref.type_hash_len],
        );
        let qos = subscription_ref.get_qos_settings();
        let callback = match subscription_ref.get_callback() {
            Some(cb) => cb,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        let context = subscription_ref.get_context();

        set_executor_node_identity(rust_exec, subscription_ref.node);
        // Phase 305 W3 (issue 0255) — resolve `~`/relative names + launch remaps
        // against the identity just set (executor-side remap table).
        let __resolved_name = match rust_exec.resolve_entity_name(topic_str) {
            Ok(r) => r,
            Err(()) => return NROS_RET_INVALID_ARGUMENT,
        };
        let topic_str = __resolved_name.as_str();

        let node_raw_id = if !subscription_ref.node.is_bound() {
            0
        } else {
            subscription_ref.node.node_id
        };
        let node_id =
            (node_raw_id != 0).then(|| nros_node::executor::NodeId::from_raw(node_raw_id));

        // Extract the group name from the C string (NULL or empty ⇒ None).
        let group_str = if callback_group.is_null() {
            None
        } else {
            let s = core::ffi::CStr::from_ptr(callback_group)
                .to_str()
                .unwrap_or("");
            if s.is_empty() { None } else { Some(s) }
        };

        let result = rust_exec.add_arena_subscription_c_callback::<MESSAGE_BUFFER_SIZE>(
            node_id,
            topic_str,
            type_str,
            type_hash_str,
            qos,
            callback,
            context,
            group_str,
            // phase-402 W2 — this path states no hint; 0 = no opinion.
            0,
        );

        match result {
            Ok(handle_id) => {
                let sub_mut = &mut *subscription;
                sub_mut.set_handle_id(handle_id);

                // Apply invocation override if not the default (on-new-data = 0).
                if invocation == nros_executor_handle_invocation_t::NROS_EXECUTOR_ALWAYS {
                    rust_exec.set_invocation(handle_id, nros_node::InvocationMode::Always);
                }

                executor.handle_count += 1;
                NROS_RET_OK
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
}

/// Phase 273 (RFC-0047) — register a timer in a named callback group.
///
/// Identical to `rclc_executor_add_timer` but additionally passes the
/// group name so the seeded `group_sched_table` can bind the callback to the
/// group's `SchedContext`. `callback_group` may be NULL or empty — both behave
/// identically to `rclc_executor_add_timer`.
///
/// # Safety
/// All non-NULL pointers must be valid and point to initialized objects.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_timer_in_group(
    executor: *mut nros_executor_t,
    timer: *mut nros_timer_t,
    callback_group: *const c_char,
) -> nros_ret_t {
    validate_not_null!(executor, timer);

    let executor = &mut *executor;
    let timer_ref = &*timer;

    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    validate_state!(timer_ref, nros_timer_state_t::NROS_TIMER_STATE_RUNNING);

    if executor.handle_count >= executor.max_handles {
        return NROS_RET_FULL;
    }

    {
        let rust_exec = get_executor(&mut executor._opaque);

        let c_callback = match timer_ref.get_callback() {
            Some(cb) => cb,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        let c_context = timer_ref.get_context();
        let timer_ptr = timer;

        let wrapper = move || {
            c_callback(timer_ptr, c_context);
        };

        // Issue #505 — microseconds, as in `rclc_executor_add_timer`.
        let period_us = timer_ref.period_ns / 1_000;
        if period_us == 0 {
            return NROS_RET_INVALID_ARGUMENT;
        }

        // Extract the group name (NULL or empty ⇒ None).
        let group_str = if callback_group.is_null() {
            None
        } else {
            let s = core::ffi::CStr::from_ptr(callback_group)
                .to_str()
                .unwrap_or("");
            if s.is_empty() { None } else { Some(s) }
        };

        // Node identity: the C timer struct doesn't carry a node_id today.
        // Group lookup falls back to the executor's primary node when no
        // node_id is threaded — sufficient for the single-node C use case.
        let period = nros_node::TimerDuration::from_micros(period_us);
        match rust_exec.register_timer_on(None, period, wrapper, group_str) {
            Ok(handle_id) => {
                let timer_mut = &mut *timer;
                timer_mut.set_handle_id(handle_id);
                timer_mut.set_executor_ptr(executor._opaque.as_mut_ptr() as *mut core::ffi::c_void);

                executor.handle_count += 1;
                executor.timer_count += 1;
                NROS_RET_OK
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
}

/// Add a service to the executor.
///
/// Extracts metadata from the service struct and registers a raw-bytes
/// service callback with the internal nros-node executor.
///
/// # Safety
/// * All pointers must be valid and point to initialized objects
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_service(
    executor: *mut nros_executor_t,
    service: *mut nros_service_t,
) -> nros_ret_t {
    validate_not_null!(executor, service);

    let executor = &mut *executor;
    let service_ref = &*service;

    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    validate_state!(
        service_ref,
        nros_service_state_t::NROS_SERVICE_STATE_INITIALIZED
    );

    // Check capacity
    if executor.handle_count >= executor.max_handles {
        return NROS_RET_FULL;
    }

    {
        let rust_exec = get_executor(&mut executor._opaque);

        // Extract metadata from service struct
        let service_name = core::str::from_utf8_unchecked(
            &service_ref.service_name[..service_ref.service_name_len],
        );
        let type_str =
            core::str::from_utf8_unchecked(&service_ref.type_name[..service_ref.type_name_len]);
        let type_hash_str =
            core::str::from_utf8_unchecked(&service_ref.type_hash[..service_ref.type_hash_len]);

        // Get callback and context
        let callback = match service_ref.get_callback() {
            Some(cb) => cb,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        let context = service_ref.get_context();

        // Propagate node identity for liveliness key expression.
        set_executor_node_identity(rust_exec, service_ref.node);
        // Phase 305 W3 (issue 0255) — resolve `~`/relative names + launch remaps
        // against the identity just set (executor-side remap table).
        let __resolved_name = match rust_exec.resolve_entity_name(service_name) {
            Ok(r) => r,
            Err(()) => return NROS_RET_INVALID_ARGUMENT,
        };
        let service_name = __resolved_name.as_str();

        // Phase 104.C.8.b — route multi-Node services through the
        // `_on(NodeId, ...)` variant when the Node was created via
        // `nros_executor_node_init`.
        let node_raw_id = if !service_ref.node.is_bound() {
            0
        } else {
            service_ref.node.node_id
        };
        // Phase 193.4 — the service's QoS (set via nros_service_init_with_qos;
        // defaults to services_default via nros_service_init).
        let svc_qos = service_ref.get_qos_settings();
        // Phase 189.M3.3.a — capture the requested sched-context slot before the
        // `&mut *service` reborrow in the Ok arm (avoids an aliasing borrow).
        let requested_sc = service_ref.sched_context_id;
        let result = if node_raw_id != 0 {
            rust_exec.register_service_raw_sized_on::<MESSAGE_BUFFER_SIZE, MESSAGE_BUFFER_SIZE>(
                nros_node::executor::NodeId::from_raw(node_raw_id),
                service_name,
                type_str,
                type_hash_str,
                svc_qos,
                callback,
                context,
            )
        } else {
            rust_exec.register_service_raw_sized::<MESSAGE_BUFFER_SIZE, MESSAGE_BUFFER_SIZE>(
                service_name,
                type_str,
                type_hash_str,
                svc_qos,
                callback,
                context,
            )
        };

        match result {
            Ok(handle_id) => {
                // Phase 189.M3.3.a — apply a scheduling-context binding requested
                // via `nros_service_init_with_options`. Done *before* the
                // `executor as *mut _` store below so `rust_exec`'s borrow of
                // `executor._opaque` ends here (no overlap with the whole-executor
                // reborrow). `0` = inherit the default (no-op). An unknown slot
                // fails the registration so the caller learns the binding was
                // rejected rather than silently dropped (mirrors subscriptions).
                if requested_sc != 0 {
                    let sc_id = nros_node::executor::sched_context::SchedContextId(requested_sc);
                    if rust_exec
                        .bind_handle_to_sched_context(handle_id, sc_id)
                        .is_err()
                    {
                        return NROS_RET_INVALID_ARGUMENT;
                    }
                }

                let service_mut = &mut *service;
                service_mut._internal.arena_entry_index = handle_id.0 as i32;
                service_mut._internal.executor_ptr = executor as *mut _ as *mut core::ffi::c_void;

                executor.handle_count += 1;
                executor.service_count += 1;
                NROS_RET_OK
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
}

/// phase-417 stage 6 — register a service server, supplying its request
/// callback HERE rather than at `*_init`.
///
/// `rclc_service_init_default` takes no callback (rclc's arity), so the handler
/// that used to ride on `nros_service_init` is supplied at registration — the
/// call rclc calls `rclc_executor_add_service`. The `nros_` prefix is kept for
/// the same reason as `nros_executor_add_subscription_raw`: rclc's
/// `rclc_executor_add_service` carries caller-owned `request_msg` /
/// `response_msg` storage and a `void (*)(const void *, void *)` handler, while
/// ours is a CDR-byte handler that RETURNS a value. Same capability, different
/// signature, and only a signature can carry a name.
///
/// # Safety
/// * Both pointers must reference initialized objects.
/// * `context` is passed through untouched and may be NULL.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_service_raw(
    executor: *mut nros_executor_t,
    service: *mut nros_service_t,
    callback: crate::service::nros_service_callback_t,
    context: *mut core::ffi::c_void,
) -> nros_ret_t {
    validate_not_null!(executor, service);
    (*service).callback = callback;
    (*service).context = context;
    nros_executor_add_service(executor, service)
}

/// Add a service client to the executor (Phase 82).
///
/// Creates the underlying `RmwServiceClient` inside the executor's arena
/// and stashes the executor pointer + arena entry index into the
/// client's `_internal` blob so subsequent calls to `nros_client_call`,
/// `nros_client_send_request_async`, and friends can drive the executor
/// without taking it as an explicit argument.
///
/// Must be called exactly once after `rclc_client_init_default` and before any
/// send/call. Calling it twice on the same client returns
/// `NROS_RET_BAD_SEQUENCE`.
///
/// # Safety
/// * Both pointers must reference initialized objects.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_client(
    executor: *mut nros_executor_t,
    client: *mut nros_client_t,
) -> nros_ret_t {
    validate_not_null!(executor, client);

    let executor = &mut *executor;
    let client_ref = &*client;

    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    validate_state!(
        client_ref,
        nros_client_state_t::NROS_CLIENT_STATE_INITIALIZED
    );

    if executor.handle_count >= executor.max_handles {
        return NROS_RET_FULL;
    }

    {
        let opaque_ptr = executor._opaque.as_mut_ptr() as *mut core::ffi::c_void;
        let rust_exec = get_executor_from_ptr(opaque_ptr);

        let service_name =
            core::str::from_utf8_unchecked(&client_ref.service_name[..client_ref.service_name_len]);
        let type_str =
            core::str::from_utf8_unchecked(&client_ref.type_name[..client_ref.type_name_len]);
        let type_hash_str =
            core::str::from_utf8_unchecked(&client_ref.type_hash[..client_ref.type_hash_len]);

        // Trampoline always installed — checks the C struct's
        // response_callback at invocation time, so blocking wrappers
        // (nros_client_call) can install one-shot callbacks AFTER
        // registration without re-registering with the arena.
        let cb: Option<nros_node::RawResponseCallback> = Some(client_response_trampoline);
        let client_ctx = client as *mut core::ffi::c_void;

        // Propagate node identity for liveliness key expression.
        set_executor_node_identity(rust_exec, client_ref.node);
        // Phase 305 W3 (issue 0255) — resolve `~`/relative names + launch remaps
        // against the identity just set (executor-side remap table).
        let __resolved_name = match rust_exec.resolve_entity_name(service_name) {
            Ok(r) => r,
            Err(()) => return NROS_RET_INVALID_ARGUMENT,
        };
        let service_name = __resolved_name.as_str();

        // Phase 104.C.8.b — service-client multi-Node dispatch.
        let node_raw_id = if !client_ref.node.is_bound() {
            0
        } else {
            client_ref.node.node_id
        };
        // Phase 193.4b — the client's QoS (set via nros_client_init_with_qos;
        // defaults to services_default via rclc_client_init_default).
        let client_qos = client_ref.get_qos_settings();
        // Phase 189.M3.3.a — capture the requested sched-context slot before the
        // `&mut *client` reborrow in the Ok arm (avoids an aliasing borrow).
        let requested_sc = client_ref.sched_context_id;
        let result = if node_raw_id != 0 {
            rust_exec.register_service_client_raw_sized_on::<MESSAGE_BUFFER_SIZE>(
                nros_node::executor::NodeId::from_raw(node_raw_id),
                service_name,
                type_str,
                type_hash_str,
                client_qos,
                cb,
                client_ctx,
            )
        } else {
            rust_exec.register_service_client_raw_sized::<MESSAGE_BUFFER_SIZE>(
                service_name,
                type_str,
                type_hash_str,
                client_qos,
                cb,
                client_ctx,
            )
        };

        match result {
            Ok(handle_id) => {
                // Phase 189.M3.3.a — apply a sched-context binding requested via
                // `nros_client_init_with_options`, *before* the `executor as
                // *mut _` store below so `rust_exec`'s `executor._opaque` borrow
                // ends here (no overlap with the whole-executor reborrow). `0` =
                // inherit (no-op); an unknown slot fails registration.
                if requested_sc != 0 {
                    let sc_id = nros_node::executor::sched_context::SchedContextId(requested_sc);
                    if rust_exec
                        .bind_handle_to_sched_context(handle_id, sc_id)
                        .is_err()
                    {
                        return NROS_RET_INVALID_ARGUMENT;
                    }
                }

                let client_mut = &mut *client;
                client_mut._internal.arena_entry_index = handle_id.0 as i32;
                client_mut._internal.executor_ptr = executor as *mut _ as *mut core::ffi::c_void;
                client_mut.state = nros_client_state_t::NROS_CLIENT_STATE_REGISTERED;

                executor.handle_count += 1;
                NROS_RET_OK
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
}

/// Add a guard condition to the executor.
///
/// # Safety
/// * All pointers must be valid and point to initialized objects
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_guard_condition(
    executor: *mut nros_executor_t,
    guard: *mut nros_guard_condition_t,
) -> nros_ret_t {
    validate_not_null!(executor, guard);

    let executor = &mut *executor;
    let guard_ref = &*guard;

    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    validate_state!(
        guard_ref,
        nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_INITIALIZED
    );

    // Check capacity
    if executor.handle_count >= executor.max_handles {
        return NROS_RET_FULL;
    }

    {
        let rust_exec = get_executor(&mut executor._opaque);

        // Get the C callback and context from the guard condition
        let c_callback = guard_ref.get_callback();
        let c_context = guard_ref.get_context();

        // Wrap the C callback in a Rust closure
        let wrapper = move || {
            if let Some(cb) = c_callback {
                // SAFETY: The C callback and context remain valid for the
                // lifetime of the executor.
                cb(c_context);
            }
        };

        match rust_exec.register_guard_condition(wrapper) {
            Ok((handle_id, guard_handle)) => {
                let guard_mut = &mut *guard;
                guard_mut.set_handle_id(handle_id);
                guard_mut.set_guard_handle(guard_handle);

                executor.handle_count += 1;
                NROS_RET_OK
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
}

/// Add an action server to the executor.
///
/// Extracts metadata from the action server struct, creates callback
/// trampolines, and registers with the internal executor.
///
/// # Safety
/// * All pointers must be valid and point to initialized objects
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_action_server(
    executor: *mut nros_executor_t,
    server: *mut nros_action_server_t,
) -> nros_ret_t {
    validate_not_null!(executor, server);

    let executor = &mut *executor;
    let server_ref = &*server;

    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    validate_state!(
        server_ref,
        nros_action_server_state_t::NROS_ACTION_SERVER_STATE_INITIALIZED
    );

    // Check capacity
    if executor.handle_count >= executor.max_handles {
        return NROS_RET_FULL;
    }

    {
        // Grab the opaque pointer before borrowing for get_executor to avoid double borrow.
        let opaque_ptr = executor._opaque.as_mut_ptr() as *mut core::ffi::c_void;
        let rust_exec = get_executor_from_ptr(opaque_ptr);

        // Extract metadata from action server struct
        let action_name =
            core::str::from_utf8_unchecked(&server_ref.action_name[..server_ref.action_name_len]);
        let type_str =
            core::str::from_utf8_unchecked(&server_ref.type_name[..server_ref.type_name_len]);
        let type_hash_str =
            core::str::from_utf8_unchecked(&server_ref.type_hash[..server_ref.type_hash_len]);

        // Get the goal callback (required — validated during init)
        let c_goal_callback = match server_ref.goal_callback {
            Some(cb) => cb,
            None => return NROS_RET_INVALID_ARGUMENT,
        };

        // Create the internal struct (handle filled after registration).
        // Phase 87.5: ActionServerInternal is now a typed `#[repr(C)]` field,
        // not an opaque blob — assign by value.
        let server_mut = &mut *server;
        server_mut._internal = ActionServerInternal {
            handle: nros_node::ActionServerRawHandle::invalid(),
            executor_ptr: opaque_ptr,
            c_goal_callback,
            c_cancel_callback: server_ref.cancel_callback,
            c_accepted_callback: server_ref.accepted_callback,
            c_context: server_ref.context,
            server_ptr: server,
        };
        let context =
            (&mut server_mut._internal) as *mut ActionServerInternal as *mut core::ffi::c_void;

        // Propagate node identity for liveliness key expression.
        set_executor_node_identity(rust_exec, server_ref.node);
        // Phase 305 W3 (issue 0255) — resolve `~`/relative names + launch remaps
        // against the identity just set (executor-side remap table).
        let __resolved_name = match rust_exec.resolve_entity_name(action_name) {
            Ok(r) => r,
            Err(()) => return NROS_RET_INVALID_ARGUMENT,
        };
        let action_name = __resolved_name.as_str();

        // Phase 104.C.8.b — action-server multi-Node dispatch.
        let node_raw_id = if !server_ref.node.is_bound() {
            0
        } else {
            server_ref.node.node_id
        };

        // Register with the nros-node executor using trampolines. The
        // accepted_callback_trampoline is invoked by the arena *after* the
        // accept reply is sent, so the user's long-running execution does
        // not delay the reply the client is blocking on.
        // Phase 193.4b — the action server's QoS (set via
        // nros_action_server_init_with_qos; defaults to services_default via
        // nros_action_server_init). Applies to the three underlying service
        // servers.
        let server_qos = server_ref.get_qos_settings();
        // Phase 189.M3.3.b — requested sched-context slot (bind applied at the
        // Ok arm using the action server's goal-service handle).
        let requested_sc = server_ref.sched_context_id;
        let node_id = if node_raw_id != 0 {
            Some(nros_node::executor::NodeId::from_raw(node_raw_id))
        } else {
            None
        };
        let result = rust_exec
            .register_action_server_raw_sized::<MESSAGE_BUFFER_SIZE, MESSAGE_BUFFER_SIZE, MESSAGE_BUFFER_SIZE, NROS_MAX_CONCURRENT_GOALS>(
                nros_node::RawActionServerSpec {
                    node_id,
                    action_name,
                    type_name: type_str,
                    type_hash: type_hash_str,
                    qos: server_qos,
                    goal_callback: goal_callback_trampoline,
                    cancel_callback: cancel_callback_trampoline,
                    accepted_callback: Some(crate::action::accepted_callback_trampoline),
                    context,
                },
            );

        match result {
            Ok(handle) => {
                // Phase 189.M3.3.b — bind the action's goal-service handle to the
                // requested sched context (governs the action's callback
                // dispatch). `0` = inherit (no-op); an unknown slot fails
                // registration (mirrors subscriptions/services).
                if requested_sc != 0 {
                    let sc_id = nros_node::executor::sched_context::SchedContextId(requested_sc);
                    if rust_exec
                        .bind_handle_to_sched_context(handle.handle_id(), sc_id)
                        .is_err()
                    {
                        return NROS_RET_INVALID_ARGUMENT;
                    }
                }
                // Fill in the handle now that registration succeeded
                server_mut._internal.handle = handle;
                executor.handle_count += 1;
                NROS_RET_OK
            }
            Err(_) => {
                // Registration failed — reset the internal back to invalid.
                server_mut._internal = ActionServerInternal::invalid_default();
                NROS_RET_ERROR
            }
        }
    }
}

/// Register an action client with the executor for async (non-blocking) operation.
///
/// After registration, `rclc_executor_spin_some` polls the action client's
/// pending requests (goal response, feedback, result) and invokes the
/// registered callbacks.
///
/// The action client must already be initialized via `nros_action_client_init`.
/// Callbacks should be set via `nros_action_client_set_goal_response_callback`,
/// `nros_action_client_set_feedback_callback`, and `nros_action_client_set_result_callback`
/// before or after this call.
///
/// # Safety
/// * `executor` and `client` must be valid pointers to initialized structs.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_action_client(
    executor: *mut nros_executor_t,
    client: *mut nros_action_client_t,
) -> nros_ret_t {
    validate_not_null!(executor, client);

    let executor = &mut *executor;
    let client_ref = &*client;

    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    validate_state!(
        client_ref,
        nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_INITIALIZED
    );

    if executor.handle_count >= executor.max_handles {
        return NROS_RET_FULL;
    }

    {
        let opaque_ptr = executor._opaque.as_mut_ptr() as *mut core::ffi::c_void;
        let rust_exec = get_executor_from_ptr(opaque_ptr);

        // Always register trampolines — they check the C struct's callback
        // pointer at invocation time, so they handle None gracefully. This is
        // critical: the blocking wrappers (nros_action_send_goal, etc.) install
        // temporary callbacks on the C struct AFTER registration. If we only
        // register trampolines when callbacks are non-None at registration time,
        // the arena will consume replies without invoking the trampoline,
        // causing the blocking wrapper's flag to never be set (→ timeout).
        let goal_response_cb: Option<nros_node::executor::RawGoalResponseCallback> =
            Some(goal_response_trampoline as nros_node::executor::RawGoalResponseCallback);

        let feedback_cb: Option<nros_node::executor::RawFeedbackCallback> =
            Some(feedback_trampoline as nros_node::executor::RawFeedbackCallback);

        let result_cb: Option<nros_node::executor::RawResultCallback> =
            Some(result_trampoline as nros_node::executor::RawResultCallback);

        let client_ctx = client as *mut core::ffi::c_void;

        let action_name =
            core::str::from_utf8_unchecked(&client_ref.action_name[..client_ref.action_name_len]);
        let type_str =
            core::str::from_utf8_unchecked(&client_ref.type_name[..client_ref.type_name_len]);
        let type_hash_str =
            core::str::from_utf8_unchecked(&client_ref.type_hash[..client_ref.type_hash_len]);

        // Propagate node identity for liveliness key expression.
        set_executor_node_identity(rust_exec, client_ref.node);
        // Phase 305 W3 (issue 0255) — resolve `~`/relative names + launch remaps
        // against the identity just set (executor-side remap table).
        let __resolved_name = match rust_exec.resolve_entity_name(action_name) {
            Ok(r) => r,
            Err(()) => return NROS_RET_INVALID_ARGUMENT,
        };
        let action_name = __resolved_name.as_str();

        // Phase 104.C.8.b — action-client multi-Node dispatch.
        // `spec.node_id` selects the target Node's session (or the
        // executor's own node when `None`).
        let node_raw_id = if !client_ref.node.is_bound() {
            0
        } else {
            client_ref.node.node_id
        };
        let node_id = if node_raw_id != 0 {
            Some(nros_node::executor::NodeId::from_raw(node_raw_id))
        } else {
            None
        };
        // Phase 189.M3.3.b — requested sched-context slot (bind applied at the
        // Ok arm; the client handle's entry_index is its callback slot).
        let requested_sc = client_ref.sched_context_id;

        // Create a NEW ActionClientCore in the arena via register_action_client_raw.
        // The async send functions will use this core (not the client's original).
        // Both share the same global zenoh session, so the arena core's service
        // clients can communicate with the server independently.
        let result = rust_exec.register_action_client_raw(nros_node::RawActionClientSpec {
            node_id,
            action_name,
            type_name: type_str,
            type_hash: type_hash_str,
            goal_response_callback: goal_response_cb,
            feedback_callback: feedback_cb,
            result_callback: result_cb,
            context: client_ctx,
        });

        match result {
            Ok(handle) => {
                let client_mut = &mut *client;
                client_mut._internal.arena_entry_index = handle.entry_index() as i32;
                client_mut._internal.executor_ptr = opaque_ptr;

                // Phase 189.M3.3.b — bind the client's callback slot to the
                // requested sched context (the handle's entry_index is the
                // entries[] slot). `0` = inherit (no-op); unknown slot fails.
                if requested_sc != 0 {
                    let sc_id = nros_node::executor::sched_context::SchedContextId(requested_sc);
                    let handle_id = nros_node::executor::HandleId(handle.entry_index());
                    if rust_exec
                        .bind_handle_to_sched_context(handle_id, sc_id)
                        .is_err()
                    {
                        return NROS_RET_INVALID_ARGUMENT;
                    }
                }

                executor.handle_count += 1;
                NROS_RET_OK
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
}

/// Goal response trampoline — adapts nros-node callback to C API callback.
///
/// # Safety
/// `context` must point to a valid `nros_action_client_t`.
unsafe extern "C" fn goal_response_trampoline(
    goal_id: *const nros_node::GoalId,
    accepted: bool,
    context: *mut core::ffi::c_void,
) {
    let client = &*(context as *const nros_action_client_t);
    if let Some(cb) = client.goal_response_callback {
        let uuid = nros_goal_uuid_t {
            uuid: (*goal_id).uuid,
        };
        cb(&uuid, accepted, client.context);
    }
}

/// Feedback trampoline — adapts nros-node callback to C API callback.
///
/// # Safety
/// `context` must point to a valid `nros_action_client_t`.
unsafe extern "C" fn feedback_trampoline(
    goal_id: *const nros_node::GoalId,
    feedback_data: *const u8,
    feedback_len: usize,
    context: *mut core::ffi::c_void,
) {
    let client = &*(context as *const nros_action_client_t);
    if let Some(cb) = client.feedback_callback {
        let uuid = nros_goal_uuid_t {
            uuid: (*goal_id).uuid,
        };
        cb(&uuid, feedback_data, feedback_len, client.context);
    }
}

/// Result trampoline — adapts nros-node callback to C API callback.
///
/// # Safety
/// `context` must point to a valid `nros_action_client_t`.
unsafe extern "C" fn result_trampoline(
    goal_id: *const nros_node::GoalId,
    status: nros_node::GoalStatus,
    result_data: *const u8,
    result_len: usize,
    context: *mut core::ffi::c_void,
) {
    let client = &*(context as *const nros_action_client_t);
    if let Some(cb) = client.result_callback {
        let uuid = nros_goal_uuid_t {
            uuid: (*goal_id).uuid,
        };
        let c_status = match status {
            nros_node::GoalStatus::Succeeded => nros_goal_status_t::NROS_GOAL_STATUS_SUCCEEDED,
            nros_node::GoalStatus::Canceled => nros_goal_status_t::NROS_GOAL_STATUS_CANCELED,
            nros_node::GoalStatus::Aborted => nros_goal_status_t::NROS_GOAL_STATUS_ABORTED,
            _ => nros_goal_status_t::NROS_GOAL_STATUS_UNKNOWN,
        };
        cb(&uuid, c_status, result_data, result_len, client.context);
    }
}

// ============================================================================
// Spin functions — delegated to nros-node executor
// ============================================================================

/// Spin the executor once.
///
/// Drives middleware I/O, then dispatches ready callbacks.
///
/// # Safety
/// * `executor` must be a valid pointer to an initialized executor
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_executor_spin_some(
    executor: *mut nros_executor_t,
    timeout_ns: u64,
) -> nros_ret_t {
    validate_not_null!(executor);

    let executor = &mut *executor;

    // Accept both INITIALIZED and SPINNING states
    if executor.state != nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
        && executor.state != nros_executor_state_t::NROS_EXECUTOR_STATE_SPINNING
    {
        return NROS_RET_NOT_INIT;
    }

    {
        let rust_exec = get_executor(&mut executor._opaque);

        // Convert timeout from nanoseconds to milliseconds (i32) for nros-node
        let timeout_ms: u64 = if timeout_ns > 0 {
            (timeout_ns / 1_000_000).max(1)
        } else {
            0
        };

        // spin_once drives I/O internally and handles trigger evaluation,
        // LET semantics, and dispatch. Guard against reentrancy so
        // blocking helpers called from inside a callback are detected.
        executor.in_dispatch = true;
        let result = rust_exec.spin_once(core::time::Duration::from_millis(timeout_ms));
        executor.in_dispatch = false;

        if result.any_work() {
            NROS_RET_OK
        } else {
            NROS_RET_TIMEOUT
        }
    }
}

/// Spin the executor forever.
///
/// # Safety
/// * `executor` must be a valid pointer to an initialized executor
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_executor_spin(executor: *mut nros_executor_t) -> nros_ret_t {
    validate_not_null!(executor);

    let executor_ref = &mut *executor;

    validate_state!(
        executor_ref,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );

    executor_ref.state = nros_executor_state_t::NROS_EXECUTOR_STATE_SPINNING;
    // phase-417 W4.c — this loop owns its own iteration, so it must tell the
    // executor: `enter_spin_loop` clears any stale cancel and raises the flag
    // `nros_executor_is_spinning` reads. RFC-0019 — the loop keeps no bit of its
    // own. Paired with `exit_spin_loop` on BOTH exits below.
    get_executor(&mut executor_ref._opaque).enter_spin_loop();

    // Spin until shutdown, or until the SESSION dies persistently.
    //
    // issue 0324 — this used `let _ = rclc_executor_spin_some(...)`, so a
    // transport that had died kept the loop running forever and the blocking
    // spin still returned OK on shutdown: the C caller could not distinguish
    // "ran until you stopped me" from "spun uselessly against a dead session".
    //
    // issue 0355 — dead-session detection reads the REAL `drive_io` health
    // counter, NOT `spin_some`'s return. `spin_some` returns `NROS_RET_TIMEOUT`
    // on every idle tick (`!any_work()`) against a perfectly LIVE transport, so
    // counting those returns killed a healthy listener that simply idled before
    // its publisher was discovered — normal DDS timing (the old
    // `SPIN_ERROR_TOLERANCE * timeout` ≈ 1.6 s is shorter than SPDP). The
    // executor's `session_io_failures()` counts only genuine `drive_io` errors
    // (dead router, closed socket, expired lease) and resets on any successful
    // drive, so a benign idle leaves it at 0. Same threshold, correct signal.
    // The cancel flag is the second exit, and it is the one `nros_executor_cancel`
    // raises from a signal handler: `state` is only readable by whoever holds the
    // struct, while the flag is what a `drive_io` blocked mid-poll is woken on.
    while executor_ref.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SPINNING
        && !get_executor(&mut executor_ref._opaque).is_halted()
    {
        let ret = rclc_executor_spin_some(executor, executor_ref.timeout_ns);
        if get_executor(&mut executor_ref._opaque).session_io_failures() >= SPIN_ERROR_TOLERANCE {
            get_executor(&mut executor_ref._opaque).exit_spin_loop();
            if executor_ref.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SPINNING {
                executor_ref.state = nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED;
            }
            return if ret != NROS_RET_OK {
                ret
            } else {
                NROS_RET_ERROR
            };
        }
    }

    get_executor(&mut executor_ref._opaque).exit_spin_loop();
    if executor_ref.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SPINNING {
        executor_ref.state = nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED;
    }
    NROS_RET_OK
}

/// How many CONSECUTIVE `drive_io` failures (the executor's
/// [`session_io_failures`](nros_node::Executor::session_io_failures) counter) a
/// blocking spin tolerates before giving up and returning an error (issue 0324,
/// corrected in issue 0355).
///
/// Not 1: a drive can fail transiently (a momentary transport hiccup), and
/// aborting on the first would turn a blip into a dead node. Not unbounded
/// either — that was the 0324 bug: a `let _ =` let a dead session spin forever
/// while the blocking call still returned OK on shutdown.
///
/// It gates the REAL health counter (which increments only on a genuine
/// `drive_io` error and resets on any successful drive), NOT `spin_some`'s
/// return — issue 0355: a live-but-idle transport returns `NROS_RET_TIMEOUT`
/// every tick, so counting THOSE killed a healthy listener that idled before its
/// publisher was discovered.
const SPIN_ERROR_TOLERANCE: u32 = 16;

/// Spin the executor with a fixed period.
///
/// # Safety
/// * `executor` must be a valid pointer to an initialized executor
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_executor_spin_period(
    executor: *mut nros_executor_t,
    period_ns: u64,
) -> nros_ret_t {
    validate_not_null!(executor);

    if period_ns == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let executor_ref = &mut *executor;

    validate_state!(
        executor_ref,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );

    executor_ref.state = nros_executor_state_t::NROS_EXECUTOR_STATE_SPINNING;
    executor_ref.invocation_time_ns = crate::platform::get_time_ns();
    // phase-417 W4.c — see `rclc_executor_spin`: one flag, owned by Rust.
    get_executor(&mut executor_ref._opaque).enter_spin_loop();

    while executor_ref.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SPINNING
        && !get_executor(&mut executor_ref._opaque).is_halted()
    {
        // `period_ns` is an upper bound on how long `drive_io` will block.
        // The timer delta credited to spin_once is the *real* wall-clock
        // elapsed inside drive_io (measured via std::time::Instant when
        // available), not `period_ns` itself — transports like zenoh-pico's
        // condvar wake early on data arrival, and treating the requested
        // timeout as the delta would tick timers faster than wall-clock.
        let ret = rclc_executor_spin_some(executor, period_ns);
        // issue 0355 — bail only on genuine SESSION death, read from the real
        // `drive_io` health counter, NOT from `spin_some`'s idle
        // `NROS_RET_TIMEOUT`. See `rclc_executor_spin` for the full rationale:
        // an idle tick against a live transport is expected while a publisher
        // is still discovering, and must not count toward the tolerance.
        if get_executor(&mut executor_ref._opaque).session_io_failures() >= SPIN_ERROR_TOLERANCE {
            get_executor(&mut executor_ref._opaque).exit_spin_loop();
            if executor_ref.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SPINNING {
                executor_ref.state = nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED;
            }
            return if ret != NROS_RET_OK {
                ret
            } else {
                NROS_RET_ERROR
            };
        }

        // Accumulate next invocation time to prevent drift
        executor_ref.invocation_time_ns += period_ns;
        let now = crate::platform::get_time_ns();
        if executor_ref.invocation_time_ns > now {
            crate::platform::sleep_ns(executor_ref.invocation_time_ns - now);
        }
    }

    get_executor(&mut executor_ref._opaque).exit_spin_loop();
    if executor_ref.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SPINNING {
        executor_ref.state = nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED;
    }
    NROS_RET_OK
}

/// Spin the executor for one period.
///
/// # Safety
/// * `executor` must be a valid pointer to an initialized executor
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_executor_spin_one_period(
    executor: *mut nros_executor_t,
    period_ns: u64,
) -> nros_ret_t {
    validate_not_null!(executor);

    if period_ns == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let executor_ref = &mut *executor;

    if executor_ref.state != nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
        && executor_ref.state != nros_executor_state_t::NROS_EXECUTOR_STATE_SPINNING
    {
        return NROS_RET_NOT_INIT;
    }

    let start = crate::platform::get_time_ns();

    // `period_ns` bounds how long `drive_io` may block. spin_once
    // measures the actual elapsed wall-clock and credits that — not
    // `period_ns` — to timers. See `rclc_executor_spin_period` above.
    let _ = rclc_executor_spin_some(executor, period_ns);

    // Sleep for remaining time in period
    let elapsed = crate::platform::get_time_ns().saturating_sub(start);
    if elapsed < period_ns {
        crate::platform::sleep_ns(period_ns - elapsed);
    }

    NROS_RET_OK
}

/// Ask a spinning executor to stop — `rclcpp::Executor::cancel` /
/// `rcl`'s context shutdown, phase-417 W4.c.
///
/// Renamed from `nros_executor_stop`, which survives as a deprecated
/// `static inline` forwarder in `<nros/executor.h>`. Three of our own languages
/// had three answers to "stop spinning"; `cancel` is ROS 2's.
///
/// # ADOPT-BOUNDED (RFC-0087)
///
/// `cancel` sets a flag the spin loop observes at the NEXT POLL BOUNDARY, so it
/// returns BEFORE spinning has actually stopped;
/// [`nros_executor_is_spinning`] is the observable that tells you when it has.
/// The boundary is one `spin_once` timeout wide (`rclc_executor_set_timeout`).
///
/// It does NOT tear down the session: the executor stays initialised, keeps its
/// entities, and can be spun again. `rclc_executor_fini` is the other verb.
///
/// Safe to call from a signal handler or another thread.
///
/// # Safety
/// * `executor` must be a valid pointer
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_cancel(executor: *mut nros_executor_t) -> nros_ret_t {
    validate_not_null!(executor);

    let executor = &mut *executor;

    // RFC-0019 — the state lives in Rust. This raises the ONE cancel flag
    // `nros_node::Executor` owns (which also wakes a `drive_io` blocked on its
    // full timeout); the `state` transition below is the C ABI's own
    // observable, not a second copy of the decision.
    if executor.state != nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED
        && executor.state != nros_executor_state_t::NROS_EXECUTOR_STATE_SHUTDOWN
    {
        get_executor(&mut executor._opaque).cancel();
    }

    if executor.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SPINNING {
        executor.state = nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED;
    }

    NROS_RET_OK
}

/// Is a blocking spin (`rclc_executor_spin` / `_spin_period`) running on this
/// executor right now? phase-417 W4.c — `rclcpp::Executor::is_spinning`.
///
/// The C API has always MODELLED this (`NROS_EXECUTOR_STATE_SPINNING`) and
/// never exported a way to ask, so a C caller could stop a spin but not observe
/// that it had stopped — the second half of [`nros_executor_cancel`]'s envelope
/// had no reader.
///
/// A DIFFERENT question from "was cancel requested": between `cancel()` and the
/// loop's next poll boundary the request is in and the spin is still running.
/// This answers the second, which is the one a caller waits on.
///
/// `false` for a null pointer — "no executor" is not spinning, and this is a
/// predicate with no error channel.
///
/// # Safety
/// * `executor` must be null or a valid pointer
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_is_spinning(executor: *const nros_executor_t) -> bool {
    if executor.is_null() {
        return false;
    }

    let executor = &*executor;

    if executor.state == nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED
        || executor.state == nros_executor_state_t::NROS_EXECUTOR_STATE_SHUTDOWN
    {
        return false;
    }

    // Read the Rust flag, not `state`. `state` is set to SPINNING before the
    // loop is entered and cleared by `cancel` before the loop has observed it,
    // so it answers "was a spin ASKED FOR"; the executor's own flag answers
    // "is one RUNNING", which is what the envelope promises.
    //
    // The cast drops `const`: `get_executor` hands out `&mut CExecutor` because
    // every other caller mutates through it. Nothing is mutated here.
    let opaque = &raw const executor._opaque;
    get_executor(&mut *opaque.cast_mut()).is_spinning()
}

/// Finalize an executor.
///
/// # Safety
/// * `executor` must be a valid pointer
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_executor_fini(executor: *mut nros_executor_t) -> nros_ret_t {
    validate_not_null!(executor);

    let executor = &mut *executor;

    if executor.state == nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED {
        return NROS_RET_NOT_INIT;
    }

    // Drop the internal executor in-place — arena entries are cleaned up
    core::ptr::drop_in_place(executor._opaque.as_mut_ptr() as *mut CExecutor);
    #[allow(clippy::large_stack_arrays)] // Intentional: zero-fill inline opaque storage
    {
        executor._opaque = [0u64; EXECUTOR_OPAQUE_U64S];
    }
    executor.handle_count = 0;
    executor.subscription_count = 0;
    executor.timer_count = 0;
    executor.service_count = 0;
    executor.support = ptr::null();
    executor.state = nros_executor_state_t::NROS_EXECUTOR_STATE_SHUTDOWN;

    NROS_RET_OK
}

// ============================================================================
// Query functions
// ============================================================================

/// Get the number of handles in the executor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_get_handle_count(executor: *const nros_executor_t) -> c_int {
    if executor.is_null() {
        return 0;
    }

    let executor = &*executor;
    executor.handle_count as c_int
}

/// Check if executor is valid (initialized).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_is_valid(executor: *const nros_executor_t) -> bool {
    if executor.is_null() {
        return false;
    }

    let executor = &*executor;
    matches!(
        executor.state,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
            | nros_executor_state_t::NROS_EXECUTOR_STATE_SPINNING
    )
}

/// Get remaining total handle capacity.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_get_remaining_handles(
    executor: *const nros_executor_t,
) -> c_int {
    if executor.is_null() {
        return -1;
    }

    let executor = &*executor;
    (executor.max_handles - executor.handle_count) as c_int
}

// ============================================================================
// Kani verification
// ============================================================================

#[cfg(kani)]
mod verification {
    use super::*;
    use crate::error::*;

    #[kani::proof]
    #[kani::unwind(5)]
    fn executor_init_null_ptrs() {
        // NULL executor → INVALID_ARGUMENT
        let support = crate::support::nros_support_get_zero_initialized();
        assert_eq!(
            unsafe { nros_executor_init(core::ptr::null_mut(), &support, 4) },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL support → INVALID_ARGUMENT
        let mut executor = rclc_executor_get_zero_initialized_executor();
        assert_eq!(
            unsafe { nros_executor_init(&mut executor, core::ptr::null(), 4) },
            NROS_RET_INVALID_ARGUMENT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn executor_zero_initialized_state() {
        let executor = rclc_executor_get_zero_initialized_executor();
        assert_eq!(
            executor.state,
            nros_executor_state_t::NROS_EXECUTOR_STATE_UNINITIALIZED,
        );
        assert!(executor.support.is_null());
        assert_eq!(executor.handle_count, 0);
    }
}

// =============================================================================
// Phase 110.B / 110.C — SchedContext C-API surface
// =============================================================================

/// Scheduling class — picks the runtime queue + selection policy.
/// Mirrors `nros_node::executor::sched_context::SchedClass`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum nros_sched_class_t {
    NROS_SCHED_CLASS_FIFO = 0,
    NROS_SCHED_CLASS_EDF = 1,
    NROS_SCHED_CLASS_SPORADIC = 2,
    NROS_SCHED_CLASS_BEST_EFFORT = 3,
    NROS_SCHED_CLASS_TIME_TRIGGERED = 4,
}

/// Criticality bucket. Lower numeric value = higher priority.
/// Mirrors `nros_node::executor::sched_context::Priority`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum nros_sched_priority_t {
    NROS_SCHED_PRIORITY_CRITICAL = 0,
    NROS_SCHED_PRIORITY_NORMAL = 1,
    NROS_SCHED_PRIORITY_BEST_EFFORT = 2,
}

/// Deadline interpretation policy.
/// Mirrors `nros_node::executor::sched_context::DeadlinePolicy`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum nros_deadline_policy_t {
    NROS_DEADLINE_POLICY_RELEASED = 0,
    NROS_DEADLINE_POLICY_ACTIVATED = 1,
    NROS_DEADLINE_POLICY_INHERITED = 2,
}

/// Scheduling-context descriptor passed to
/// [`nros_executor_create_sched_context`].
///
/// Time fields use a `0` sentinel for "absent" (mirrors the Rust
/// `OptUs` newtype). Cbindgen emits these as plain `uint32_t`.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
#[allow(non_camel_case_types)]
pub struct nros_sched_context_t {
    pub class: nros_sched_class_t,
    pub priority: nros_sched_priority_t,
    pub deadline_policy: nros_deadline_policy_t,
    /// Period in microseconds (0 = absent).
    pub period_us: u32,
    /// Budget in microseconds (0 = absent).
    pub budget_us: u32,
    /// Deadline in microseconds (0 = absent).
    pub deadline_us: u32,
    /// Phase 110.F — opt-in OS-level priority for per-callback
    /// dispatch. `0` = no per-callback OS priority (default cooperative
    /// path runs every callback). Numeric meaning is platform-defined.
    pub os_pri: u8,
    /// Phase 110.G — TT-window offset within the executor's major
    /// frame, microseconds. `0` (with `tt_window_duration_us = 0`) =
    /// no TT gate.
    pub tt_window_offset_us: u32,
    /// Phase 110.G — TT-window length in microseconds. `0` disables
    /// the TT gate for this SC.
    pub tt_window_duration_us: u32,
}

/// Identifier of a registered scheduling context. `0` is the
/// auto-created default `Fifo` SC. Mirrors
/// `nros_node::executor::sched_context::SchedContextId`.
#[allow(non_camel_case_types)]
pub type nros_sched_context_id_t = u8;

/// Identifier of the auto-created default `Fifo`-class SC. Every
/// callback registered without an explicit binding maps to it.
/// Phase 110.B.
#[unsafe(no_mangle)]
pub extern "C" fn nros_executor_default_sched_context_id() -> nros_sched_context_id_t {
    0
}

fn convert_sched_context(
    cfg: &nros_sched_context_t,
) -> nros_node::executor::sched_context::SchedContext {
    use nros_node::executor::sched_context::{
        DeadlinePolicy, OptUs, Priority, SchedClass, SchedContext,
    };
    #[allow(deprecated)]
    SchedContext {
        class: match cfg.class {
            nros_sched_class_t::NROS_SCHED_CLASS_FIFO => SchedClass::Fifo,
            nros_sched_class_t::NROS_SCHED_CLASS_EDF => SchedClass::Edf,
            nros_sched_class_t::NROS_SCHED_CLASS_SPORADIC => SchedClass::Sporadic,
            nros_sched_class_t::NROS_SCHED_CLASS_BEST_EFFORT => SchedClass::BestEffort,
            // Phase 110.G refactor: TimeTriggered class is deprecated;
            // accept the C-side enum value but route to Fifo. Callers
            // should switch to populating tt_window_offset_us /
            // tt_window_duration_us for the gate semantics.
            nros_sched_class_t::NROS_SCHED_CLASS_TIME_TRIGGERED => SchedClass::Fifo,
        },
        priority: match cfg.priority {
            nros_sched_priority_t::NROS_SCHED_PRIORITY_CRITICAL => Priority::Critical,
            nros_sched_priority_t::NROS_SCHED_PRIORITY_NORMAL => Priority::Normal,
            nros_sched_priority_t::NROS_SCHED_PRIORITY_BEST_EFFORT => Priority::BestEffort,
        },
        deadline_policy: match cfg.deadline_policy {
            nros_deadline_policy_t::NROS_DEADLINE_POLICY_RELEASED => DeadlinePolicy::Released,
            nros_deadline_policy_t::NROS_DEADLINE_POLICY_ACTIVATED => DeadlinePolicy::Activated,
            nros_deadline_policy_t::NROS_DEADLINE_POLICY_INHERITED => DeadlinePolicy::Inherited,
        },
        period_us: OptUs::from_us(cfg.period_us),
        budget_us: OptUs::from_us(cfg.budget_us),
        deadline_us: OptUs::from_us(cfg.deadline_us),
        os_pri: cfg.os_pri,
        // W3b.5 — no C surface for the deadline-miss action yet (Rust-only
        // until a C consumer exists; see phase-296 W3b.5 deferral note).
        deadline_action: Default::default(),
        tt_window_offset_us: OptUs::from_us(cfg.tt_window_offset_us),
        tt_window_duration_us: OptUs::from_us(cfg.tt_window_duration_us),
    }
}

/// Phase 110.G — enable TT dispatch on this executor by setting the
/// major-frame length in microseconds. `0` disables the gate.
///
/// # Safety
/// `executor` must be a valid pointer to an initialized executor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_time_triggered_dispatcher(
    executor: *mut nros_executor_t,
    major_frame_us: u32,
) -> nros_ret_t {
    validate_not_null!(executor);
    let executor = &mut *executor;
    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    let rust_exec = get_executor(&mut executor._opaque);
    rust_exec.register_time_triggered_dispatcher(major_frame_us);
    NROS_RET_OK
}

/// Register a new scheduling context with the executor. Phase 110.B.
///
/// On success writes the new `SchedContextId` through `out_sc_id` and
/// returns `NROS_RET_OK`. Returns `NROS_RET_FULL` when no slot is
/// available (build-time `NROS_EXECUTOR_MAX_SC` exhausted).
///
/// # Safety
/// All pointers must be valid and the executor initialized.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_create_sched_context(
    executor: *mut nros_executor_t,
    cfg: *const nros_sched_context_t,
    out_sc_id: *mut nros_sched_context_id_t,
) -> nros_ret_t {
    validate_not_null!(executor, cfg, out_sc_id);
    let executor = &mut *executor;
    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    let rust_exec = get_executor(&mut executor._opaque);
    let sc = convert_sched_context(&*cfg);
    match rust_exec.create_sched_context(sc) {
        Ok(id) => {
            *out_sc_id = id.0;
            NROS_RET_OK
        }
        Err(_) => NROS_RET_FULL,
    }
}

/// Bind a registered callback to a scheduling context. The next
/// `spin_once` cycle dispatches that callback through the SC's queue
/// (FIFO bitmap or EDF heap, in the SC's priority bucket).
/// Phase 110.B.
///
/// `handle` is the index returned by the corresponding
/// `nros_executor_add_*` call. `sc_id` must be a value previously
/// returned from [`nros_executor_create_sched_context`] (or 0 for the
/// auto-created default Fifo SC).
///
/// Returns `NROS_RET_INVALID_ARGUMENT` for an out-of-range handle, an
/// empty entry slot, or an unknown `sc_id`.
///
/// # Safety
/// `executor` must be a valid pointer to an initialized executor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_bind_handle_to_sched_context(
    executor: *mut nros_executor_t,
    handle: usize,
    sc_id: nros_sched_context_id_t,
) -> nros_ret_t {
    validate_not_null!(executor);
    let executor = &mut *executor;
    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    let rust_exec = get_executor(&mut executor._opaque);
    let h = nros_node::executor::HandleId(handle);
    let id = nros_node::executor::sched_context::SchedContextId(sc_id);
    match rust_exec.bind_handle_to_sched_context(h, id) {
        Ok(()) => NROS_RET_OK,
        Err(_) => NROS_RET_INVALID_ARGUMENT,
    }
}

// ============================================================================
// Shutdown hooks (issue 0790)
// ============================================================================

/// Shutdown-hook callback type.
///
/// # Parameters
/// * `context` - User-provided context pointer, handed back unchanged.
//
// A LOCAL alias, not `nros_node::ShutdownCallbackFn`. cbindgen runs with
// `parse_deps = false`, so a type owned by another crate in an `extern "C"`
// signature degrades to an opaque struct that C cannot call through — the same
// reason `nros_timer_callback_t` is spelled here rather than imported.
pub type nros_shutdown_callback_t = Option<unsafe extern "C" fn(context: *mut core::ffi::c_void)>;

/// Handle to a registered shutdown hook.
///
/// Returned by `nros_executor_add_{pre,on}_shutdown_callback` and consumed by
/// the matching `remove`. Opaque: the only operations are "pass it back" and
/// "compare against `NROS_SHUTDOWN_CALLBACK_HANDLE_INVALID`".
pub type nros_shutdown_callback_handle_t = u32;

/// The value no successful registration ever produces.
//
// A LITERAL, not `u32::MAX` and not `Something as u32`: cbindgen silently DROPS
// a constant whose initializer it cannot evaluate, and a constant that is
// missing from the header is worse than one that is wrong — C compiles until
// someone uses it. The `const _` below is what keeps the literal honest.
pub const NROS_SHUTDOWN_CALLBACK_HANDLE_INVALID: nros_shutdown_callback_handle_t = 0xFFFF_FFFF;

const _: () = assert!(
    NROS_SHUTDOWN_CALLBACK_HANDLE_INVALID == nros_node::ShutdownCallbackHandle::INVALID.0,
    "the C-visible invalid-handle literal drifted from nros_node's ShutdownCallbackHandle::INVALID"
);

/// Register a callback to run BEFORE the executor's session is closed.
///
/// Issue 0790. This is the phase with no workaround: the callback runs while
/// every entity still works, so a node can publish a final state, answer a last
/// request, park an actuator or release a bus. After teardown it cannot.
///
/// On success writes the handle through `out_handle` and returns
/// `NROS_RET_OK`. Returns `NROS_RET_FULL` when the phase's fixed table is
/// exhausted (build-time `NROS_EXECUTOR_MAX_SHUTDOWN_CBS`, default 2).
///
/// The hooks run when the executor is finalized — `rclc_executor_fini()`, or
/// whatever tears the executor down. They are a CLEAN-STOP facility: a watchdog
/// reset, a hard fault or an abort does not come through here.
///
/// # Safety
/// * `executor` must be a valid pointer to an initialized executor.
/// * `out_handle` must be a valid pointer.
/// * `callback` must be safe to invoke once with `context`, and `context` must
///   stay valid until the hook runs or is removed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_pre_shutdown_callback(
    executor: *mut nros_executor_t,
    callback: nros_shutdown_callback_t,
    context: *mut core::ffi::c_void,
    out_handle: *mut nros_shutdown_callback_handle_t,
) -> nros_ret_t {
    validate_not_null!(executor, out_handle);
    let Some(callback) = callback else {
        return NROS_RET_INVALID_ARGUMENT;
    };
    let executor = &mut *executor;
    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    let rust_exec = get_executor(&mut executor._opaque);
    match rust_exec.add_pre_shutdown_callback(callback, context) {
        Ok(handle) => {
            *out_handle = handle.0;
            NROS_RET_OK
        }
        Err(_) => {
            *out_handle = NROS_SHUTDOWN_CALLBACK_HANDLE_INVALID;
            NROS_RET_FULL
        }
    }
}

/// Register a callback to run AFTER the executor's session is closed.
///
/// Issue 0790 — rclcpp's `add_on_shutdown_callback` / `rclcpp::on_shutdown`.
/// The entities are gone by the time it runs, so anything that needs the wire
/// belongs in [`nros_executor_add_pre_shutdown_callback`] instead.
///
/// # Safety
/// Same contract as [`nros_executor_add_pre_shutdown_callback`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_add_on_shutdown_callback(
    executor: *mut nros_executor_t,
    callback: nros_shutdown_callback_t,
    context: *mut core::ffi::c_void,
    out_handle: *mut nros_shutdown_callback_handle_t,
) -> nros_ret_t {
    validate_not_null!(executor, out_handle);
    let Some(callback) = callback else {
        return NROS_RET_INVALID_ARGUMENT;
    };
    let executor = &mut *executor;
    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    let rust_exec = get_executor(&mut executor._opaque);
    match rust_exec.add_on_shutdown_callback(callback, context) {
        Ok(handle) => {
            *out_handle = handle.0;
            NROS_RET_OK
        }
        Err(_) => {
            *out_handle = NROS_SHUTDOWN_CALLBACK_HANDLE_INVALID;
            NROS_RET_FULL
        }
    }
}

/// Remove a previously registered pre-shutdown callback.
///
/// Returns `NROS_RET_OK` when `handle` named a live hook, `NROS_RET_NOT_FOUND`
/// when it did not — including a handle that was already removed, and a handle
/// issued by `nros_executor_add_on_shutdown_callback` (the phase is part of the
/// handle, so it cannot cross over and remove the wrong hook).
///
/// # Safety
/// `executor` must be a valid pointer to an initialized executor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_remove_pre_shutdown_callback(
    executor: *mut nros_executor_t,
    handle: nros_shutdown_callback_handle_t,
) -> nros_ret_t {
    validate_not_null!(executor);
    let executor = &mut *executor;
    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    let rust_exec = get_executor(&mut executor._opaque);
    if rust_exec.remove_pre_shutdown_callback(nros_node::ShutdownCallbackHandle(handle)) {
        NROS_RET_OK
    } else {
        NROS_RET_NOT_FOUND
    }
}

/// Remove a previously registered on-shutdown callback.
/// See [`nros_executor_remove_pre_shutdown_callback`].
///
/// # Safety
/// `executor` must be a valid pointer to an initialized executor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_executor_remove_on_shutdown_callback(
    executor: *mut nros_executor_t,
    handle: nros_shutdown_callback_handle_t,
) -> nros_ret_t {
    validate_not_null!(executor);
    let executor = &mut *executor;
    validate_state!(
        executor,
        nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED
    );
    let rust_exec = get_executor(&mut executor._opaque);
    if rust_exec.remove_on_shutdown_callback(nros_node::ShutdownCallbackHandle(handle)) {
        NROS_RET_OK
    } else {
        NROS_RET_NOT_FOUND
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trigger_any_matches_behavior() {
        unsafe {
            let ready = [true, false, true];
            assert!(rclc_executor_trigger_any(
                ready.as_ptr(),
                ready.len(),
                ptr::null_mut()
            ));

            let ready = [false, false, false];
            assert!(!rclc_executor_trigger_any(
                ready.as_ptr(),
                ready.len(),
                ptr::null_mut()
            ));

            assert!(!rclc_executor_trigger_any([].as_ptr(), 0, ptr::null_mut()));
        }
    }

    #[test]
    fn test_trigger_all_matches_behavior() {
        unsafe {
            let ready = [true, true, true];
            assert!(rclc_executor_trigger_all(
                ready.as_ptr(),
                ready.len(),
                ptr::null_mut()
            ));

            let ready = [true, false, true];
            assert!(!rclc_executor_trigger_all(
                ready.as_ptr(),
                ready.len(),
                ptr::null_mut()
            ));

            let ready = [false, false, false];
            assert!(!rclc_executor_trigger_all(
                ready.as_ptr(),
                ready.len(),
                ptr::null_mut()
            ));

            assert!(!rclc_executor_trigger_all([].as_ptr(), 0, ptr::null_mut()));
        }
    }

    #[test]
    fn test_trigger_always_matches_behavior() {
        unsafe {
            assert!(rclc_executor_trigger_always(
                [].as_ptr(),
                0,
                ptr::null_mut()
            ));

            let ready = [false, false];
            assert!(rclc_executor_trigger_always(
                ready.as_ptr(),
                ready.len(),
                ptr::null_mut()
            ));
        }
    }

    #[test]
    fn test_trigger_one_matches_behavior() {
        unsafe {
            let ready = [false, true, false];
            let mut idx: usize = 1;
            assert!(rclc_executor_trigger_one(
                ready.as_ptr(),
                ready.len(),
                &mut idx as *mut usize as *mut core::ffi::c_void,
            ));

            idx = 0;
            assert!(!rclc_executor_trigger_one(
                ready.as_ptr(),
                ready.len(),
                &mut idx as *mut usize as *mut core::ffi::c_void,
            ));

            idx = 10;
            assert!(!rclc_executor_trigger_one(
                ready.as_ptr(),
                ready.len(),
                &mut idx as *mut usize as *mut core::ffi::c_void,
            ));

            // NULL context returns false (no dereference).
            assert!(!rclc_executor_trigger_one(
                ready.as_ptr(),
                ready.len(),
                core::ptr::null_mut(),
            ));
        }
    }

    #[test]
    fn test_trigger_all_matches_rust_behavior() {
        let test_cases: &[(&[bool], bool)] = &[
            (&[true, true, true], true),
            (&[true, false, true], false),
            (&[false, false, false], false),
            (&[true], true),
            (&[false], false),
            (&[], false),
        ];

        for (case, expected) in test_cases {
            let c_result =
                unsafe { rclc_executor_trigger_all(case.as_ptr(), case.len(), ptr::null_mut()) };
            assert_eq!(
                c_result, *expected,
                "trigger_all mismatch for {:?}: got {}, expected {}",
                case, c_result, expected
            );
        }
    }

    #[test]
    fn test_set_trigger_requires_init() {
        unsafe {
            let mut executor = rclc_executor_get_zero_initialized_executor();

            let ret = rclc_executor_set_trigger(
                &mut executor,
                Some(rclc_executor_trigger_all),
                ptr::null_mut(),
            );
            assert_eq!(ret, NROS_RET_NOT_INIT);
        }
    }

    #[test]
    fn test_set_trigger_null_executor() {
        unsafe {
            let ret = rclc_executor_set_trigger(
                ptr::null_mut(),
                Some(rclc_executor_trigger_all),
                ptr::null_mut(),
            );
            assert_eq!(ret, NROS_RET_INVALID_ARGUMENT);
        }
    }

    #[test]
    fn test_set_semantics_rclcpp() {
        unsafe {
            // Manually initialize (no real session needed for semantics test)
            let mut executor = rclc_executor_get_zero_initialized_executor();
            executor.state = nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED;
            executor.max_handles = 4;

            assert_eq!(
                executor.semantics,
                nros_executor_semantics_t::NROS_SEMANTICS_RCLCPP_EXECUTOR
            );

            let ret = rclc_executor_set_semantics(
                &mut executor,
                nros_executor_semantics_t::NROS_SEMANTICS_RCLCPP_EXECUTOR,
            );
            assert_eq!(ret, NROS_RET_OK);
            assert_eq!(
                executor.semantics,
                nros_executor_semantics_t::NROS_SEMANTICS_RCLCPP_EXECUTOR
            );
        }
    }

    #[test]
    fn test_set_semantics_let() {
        unsafe {
            // Manually initialize (no real session needed for semantics test)
            let mut executor = rclc_executor_get_zero_initialized_executor();
            executor.state = nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED;
            executor.max_handles = 4;

            let ret = rclc_executor_set_semantics(
                &mut executor,
                nros_executor_semantics_t::NROS_SEMANTICS_LOGICAL_EXECUTION_TIME,
            );
            assert_eq!(ret, NROS_RET_OK);
            assert_eq!(
                executor.semantics,
                nros_executor_semantics_t::NROS_SEMANTICS_LOGICAL_EXECUTION_TIME
            );
        }
    }

    #[test]
    fn test_set_semantics_requires_init() {
        unsafe {
            let mut executor = rclc_executor_get_zero_initialized_executor();

            let ret = rclc_executor_set_semantics(
                &mut executor,
                nros_executor_semantics_t::NROS_SEMANTICS_LOGICAL_EXECUTION_TIME,
            );
            assert_eq!(ret, NROS_RET_NOT_INIT);
        }
    }

    #[test]
    fn test_spin_one_period_null() {
        unsafe {
            let ret = rclc_executor_spin_one_period(ptr::null_mut(), 10_000_000);
            assert_eq!(ret, NROS_RET_INVALID_ARGUMENT);
        }
    }

    #[test]
    fn test_spin_one_period_zero_period() {
        unsafe {
            let mut executor = rclc_executor_get_zero_initialized_executor();
            let ret = rclc_executor_spin_one_period(&mut executor, 0);
            assert_eq!(ret, NROS_RET_INVALID_ARGUMENT);
        }
    }

    #[test]
    fn test_spin_one_period_not_init() {
        unsafe {
            let mut executor = rclc_executor_get_zero_initialized_executor();
            let ret = rclc_executor_spin_one_period(&mut executor, 10_000_000);
            assert_eq!(ret, NROS_RET_NOT_INIT);
        }
    }

    #[test]
    fn test_spin_period_null() {
        unsafe {
            let ret = rclc_executor_spin_period(ptr::null_mut(), 10_000_000);
            assert_eq!(ret, NROS_RET_INVALID_ARGUMENT);
        }
    }

    #[test]
    fn test_spin_period_zero_period() {
        unsafe {
            let mut executor = rclc_executor_get_zero_initialized_executor();
            let ret = rclc_executor_spin_period(&mut executor, 0);
            assert_eq!(ret, NROS_RET_INVALID_ARGUMENT);
        }
    }

    #[test]
    fn test_invocation_time_ns_initialized() {
        let executor = rclc_executor_get_zero_initialized_executor();
        assert_eq!(executor.invocation_time_ns, 0);
    }

    #[test]
    fn test_per_type_counters_initialized_to_zero() {
        let executor = rclc_executor_get_zero_initialized_executor();
        assert_eq!(executor.subscription_count, 0);
        assert_eq!(executor.timer_count, 0);
        assert_eq!(executor.service_count, 0);
    }

    #[test]
    fn test_remaining_handles_null() {
        unsafe {
            assert_eq!(nros_executor_get_remaining_handles(ptr::null()), -1);
        }
    }

    #[test]
    fn test_remaining_capacity_initial() {
        unsafe {
            let mut executor = rclc_executor_get_zero_initialized_executor();
            executor.state = nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED;
            executor.max_handles = NROS_EXECUTOR_MAX_HANDLES;

            assert_eq!(
                nros_executor_get_remaining_handles(&executor),
                NROS_EXECUTOR_MAX_HANDLES as c_int
            );
        }
    }

    #[test]
    fn test_max_handles_equals_max_cbs() {
        assert_eq!(NROS_EXECUTOR_MAX_HANDLES, nros_node::config::MAX_CBS);
    }
}
