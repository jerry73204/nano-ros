//! Action client implementation.

use core::{ffi::c_void, ptr};

use nros::{
    GoalId,
    cdr::{CDR_HEADER_LEN, strip_cdr_header, write_cdr_le_header},
};

use super::common::*;
use crate::{
    constants::{MAX_ACTION_NAME_LEN, MAX_TYPE_HASH_LEN, MAX_TYPE_NAME_LEN},
    error::*,
    node::{nros_node_state_t, nros_node_t},
};

/// CDR sequence<uint8, 16> length prefix (4 bytes) in front of the UUID bytes.
/// Mirrors the constant of the same name in `server.rs`.
const GOAL_ID_SEQ_PREFIX_LEN: usize = GoalId::SEQ_PREFIX_LEN;

/// Bytes of CDR framing that precede feedback fields in a feedback message:
/// CDR encapsulation header + GoalId sequence length prefix + UUID.
const FEEDBACK_FRAMING_LEN: usize = CDR_HEADER_LEN + GOAL_ID_SEQ_PREFIX_LEN + GoalId::UUID_LEN;

// ============================================================================
// Internal implementation
// ============================================================================

/// Internal state for the action client.
///
/// Lightweight — stores only the arena entry index and executor pointer.
/// The `ActionClientCore` (transport handles) lives in the executor's arena,
/// created by `nros_executor_add_action_client`.
#[repr(C)]
pub struct ActionClientInternal {
    /// Arena entry index (set by nros_executor_add_action_client).
    /// -1 means not registered with executor.
    pub arena_entry_index: i32,
    /// Pointer to the executor (set by nros_executor_add_action_client).
    pub executor_ptr: *mut core::ffi::c_void,
}

impl ActionClientInternal {
    pub const fn new() -> Self {
        Self {
            arena_entry_index: -1,
            executor_ptr: core::ptr::null_mut(),
        }
    }

    /// Get a mutable reference to the ActionClientCore in the executor arena.
    ///
    /// Returns `None` if not yet registered with the executor.
    ///
    /// # Safety
    /// `executor_ptr` must point to a valid `CExecutor`.
    unsafe fn arena_core_mut(&mut self) -> Option<&mut nros_node::ActionClientCore> {
        if self.arena_entry_index < 0 || self.executor_ptr.is_null() {
            return None;
        }
        let exec = &mut *(self.executor_ptr as *mut crate::executor::CExecutor);
        unsafe { exec.action_client_core_mut(self.arena_entry_index as usize) }
    }
}

impl Default for ActionClientInternal {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Action Client
// ============================================================================

/// Action client state.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_action_client_state_t {
    /// Not initialized
    NROS_ACTION_CLIENT_STATE_UNINITIALIZED = 0,
    /// L2 callback-mode: initialized via `nros_action_client_init`;
    /// transport creation deferred to
    /// `nros_executor_add_action_client`.
    NROS_ACTION_CLIENT_STATE_INITIALIZED = 1,
    /// Shutdown
    NROS_ACTION_CLIENT_STATE_SHUTDOWN = 2,
    /// Phase 122.3.c.6.b — L1 polling-mode: `ActionClientCore` lives
    /// inline in `_opaque`; caller drives via the
    /// `nros_action_client_*_raw` family. No executor registration.
    NROS_ACTION_CLIENT_STATE_POLLING = 3,
}

/// Action client structure.
#[repr(C)]
pub struct nros_action_client_t {
    /// Current state
    pub state: nros_action_client_state_t,
    /// Action name storage
    pub action_name: [u8; MAX_ACTION_NAME_LEN],
    /// Action name length
    pub action_name_len: usize,
    /// Type name storage
    pub type_name: [u8; MAX_TYPE_NAME_LEN],
    /// Type name length
    pub type_name_len: usize,
    /// Type hash storage
    pub type_hash: [u8; MAX_TYPE_HASH_LEN],
    /// Type hash length
    pub type_hash_len: usize,
    /// Goal response callback (for async send_goal)
    pub goal_response_callback: nros_goal_response_callback_t,
    /// Feedback callback
    pub feedback_callback: nros_feedback_callback_t,
    /// Result callback
    pub result_callback: nros_result_callback_t,
    /// User context pointer
    pub context: *mut c_void,
    /// Pointer to parent node
    pub node: crate::node::nros_node_ref_t,
    /// Phase 189.M3.3.b — scheduling-context slot to bind the action client's
    /// executor handle to. `0` = inherit the executor / Node default; set via
    /// `nros_action_client_init_with_options`. When non-zero,
    /// `nros_executor_add_action_client` binds the handle after
    /// registration. No effect on the L1 polling path.
    pub sched_context_id: crate::executor::nros_sched_context_id_t,
    /// Internal state (arena entry index + executor pointer). Phase 87.5:
    /// Typed C-ABI handle field.
    pub _internal: ActionClientInternal,
    /// Phase 122.3.c.6.b — inline opaque storage for the L1
    /// polling-mode `ActionClientCore`. Zeroed in L2 mode; populated
    /// by `nros_action_client_init_polling`.
    pub _opaque: [u64; crate::opaque_sizes::ACTION_CLIENT_OPAQUE_U64S],
}

impl Default for nros_action_client_t {
    // The default `_opaque` array sizes to the probed ActionClient size,
    // which exceeds clippy's 16 KB stack-array threshold. It is RVO'd
    // into its destination by the optimiser.
    #[allow(clippy::large_stack_arrays)]
    fn default() -> Self {
        Self {
            state: nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_UNINITIALIZED,
            action_name: [0u8; MAX_ACTION_NAME_LEN],
            action_name_len: 0,
            type_name: [0u8; MAX_TYPE_NAME_LEN],
            type_name_len: 0,
            type_hash: [0u8; MAX_TYPE_HASH_LEN],
            type_hash_len: 0,
            goal_response_callback: None,
            feedback_callback: None,
            result_callback: None,
            context: ptr::null_mut(),
            node: crate::node::nros_node_ref_t::none(),
            sched_context_id: 0,
            _internal: ActionClientInternal::new(),
            _opaque: [0u64; crate::opaque_sizes::ACTION_CLIENT_OPAQUE_U64S],
        }
    }
}

// ============================================================================
// Action Client Functions
// ============================================================================

/// Get a zero-initialized action client.
#[unsafe(no_mangle)]
pub extern "C" fn rcl_action_get_zero_initialized_client() -> nros_action_client_t {
    nros_action_client_t::default()
}

/// Initialize an action client.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_init(
    client: *mut nros_action_client_t,
    node: *const nros_node_t,
    action_name: *const core::ffi::c_char,
    type_info: *const nros_action_type_t,
) -> nros_ret_t {
    validate_not_null!(client, node, action_name, type_info);

    let client = &mut *client;
    let node_ref = &*node;
    let type_info = &*type_info;

    validate_state!(
        client,
        nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_UNINITIALIZED,
        NROS_RET_BAD_SEQUENCE
    );
    validate_state!(node_ref, nros_node_state_t::NROS_NODE_STATE_INITIALIZED);

    // Copy action name (required — empty rejected)
    client.action_name_len = crate::util::copy_cstr_into(action_name, &mut client.action_name);
    if client.action_name_len == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }

    // Copy type name + hash (both optional — null sources leave dst untouched)
    client.type_name_len = crate::util::copy_cstr_into(type_info.type_name, &mut client.type_name);
    client.type_hash_len = crate::util::copy_cstr_into(type_info.type_hash, &mut client.type_hash);

    // Store node pointer
    client.node = unsafe { crate::node::node_ref_of(node) };

    // Metadata only — no transport handles created here.
    // Transport handles are created in nros_executor_add_action_client,
    // which places the ActionClientCore in the executor's arena.
    client._internal = ActionClientInternal::new();

    client.state = nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_INITIALIZED;

    NROS_RET_OK
}

/// Phase 189.M3.3.b — rclc-style named action-client options (action clients
/// carry no QoS field, so this is options-only). Zero-init = default behaviour.
#[repr(C)]
#[derive(Default)]
pub struct nros_action_client_options_t {
    /// Scheduling-context slot to bind the action client's executor handle to.
    /// `0` = inherit the executor / Node default. A non-zero value must be an id
    /// from `nros_executor_create_sched_context`; the bind is applied by
    /// `nros_executor_add_action_client` once the handle exists. No effect
    /// on the L1 polling path.
    pub sched_context: crate::executor::nros_sched_context_id_t,
    /// Reserved for future use; must be zero. Pads for ABI stability.
    pub _reserved: [u8; 3],
}

/// Get a zero-initialised [`nros_action_client_options_t`] (`sched_context = 0`).
#[unsafe(no_mangle)]
pub extern "C" fn rcl_action_client_get_default_options() -> nros_action_client_options_t {
    nros_action_client_options_t::default()
}

/// Phase 189.M3.3.b — initialize an action client with named options. Like
/// [`nros_action_client_init`] except a non-zero `options->sched_context` is
/// stashed so [`nros_executor_add_action_client`] binds the resulting
/// executor handle to that scheduling context once known.
///
/// # Safety
/// All non-NULL pointers must be valid + the node initialized; `options` may be
/// NULL.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_init_with_options(
    client: *mut nros_action_client_t,
    node: *const nros_node_t,
    action_name: *const core::ffi::c_char,
    type_info: *const nros_action_type_t,
    options: *const nros_action_client_options_t,
) -> nros_ret_t {
    let ret = nros_action_client_init(client, node, action_name, type_info);
    if ret != NROS_RET_OK {
        return ret;
    }
    if !options.is_null() {
        (*client).sched_context_id = (*options).sched_context;
    }
    NROS_RET_OK
}

/// Set feedback callback.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_set_feedback_callback(
    client: *mut nros_action_client_t,
    callback: nros_feedback_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    validate_not_null!(client);

    let client = &mut *client;
    client.feedback_callback = callback;
    client.context = context;

    NROS_RET_OK
}

/// Set result callback.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_set_result_callback(
    client: *mut nros_action_client_t,
    callback: nros_result_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    validate_not_null!(client);

    let client = &mut *client;
    client.result_callback = callback;
    client.context = context;

    NROS_RET_OK
}

/// Block until the action server's send-goal queryable is discoverable
/// on the network, or `timeout_ms` elapses.
///
/// Mirrors `rclcpp_action::Client::wait_for_action_server` and the
/// the underlying `ActionClient::wait_for_action_server`. Internally
/// probes the action's `send_goal` service-server liveliness keyexpr
/// (the goal queryable is the load-bearing entity for the first
/// `nros_action_send_goal` call) via the same primitive as the
/// service-client equivalent. See
/// `packages/api/nros-c/src/service.rs::nros_client_wait_for_service`
/// for the re-probe rationale.
///
/// # Returns
/// * `NROS_RET_OK` — server visible.
/// * `NROS_RET_TIMEOUT` — `timeout_ms` elapsed without seeing a token.
/// * `NROS_RET_NOT_INIT` — client not registered with an executor.
/// * `NROS_RET_ERROR` — transport-level failure.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_wait_for_action_server(
    client: *mut nros_action_client_t,
    executor: *mut crate::executor::nros_executor_t,
    timeout_ms: u32,
) -> nros_ret_t {
    validate_not_null!(client, executor);

    #[cfg(feature = "rmw-cffi")]
    {
        let client_ref = &mut *client;
        if client_ref.state != nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_INITIALIZED {
            return NROS_RET_NOT_INIT;
        }
        let internal = &mut client_ref._internal;
        if internal.executor_ptr.is_null() || internal.arena_entry_index < 0 {
            return NROS_RET_NOT_INIT;
        }
        // Note: action clients store an opaque pointer into
        // `executor._opaque` (see `nros_executor_add_action_client`),
        // not to the outer `nros_executor_t`, so we can't recover the
        // wrapper from `internal.executor_ptr`. Take `executor` as a
        // separate argument — same convention as
        // `nros_action_send_goal`.
        let exec_t = &mut *executor;
        if exec_t.in_dispatch {
            return NROS_RET_REENTRANT;
        }

        // Latched fast-path.
        {
            let exec = crate::executor::get_executor(&mut exec_t._opaque);
            let core = match exec.action_client_core_mut(internal.arena_entry_index as usize) {
                Some(c) => c,
                None => return NROS_RET_NOT_INIT,
            };
            if core.is_server_ready() {
                return NROS_RET_OK;
            }
        }

        const PROBE_TIMEOUT_MS: u32 = nros_node::SERVER_DISCOVERY_PROBE_TIMEOUT_MS; // issue #224
        let start_ns = crate::platform::get_time_ns();
        let timeout_ns: u64 = (timeout_ms as u64).saturating_mul(1_000_000);
        loop {
            {
                let exec = crate::executor::get_executor(&mut exec_t._opaque);
                let core = match exec.action_client_core_mut(internal.arena_entry_index as usize) {
                    Some(c) => c,
                    None => return NROS_RET_NOT_INIT,
                };
                if core.start_server_discovery(PROBE_TIMEOUT_MS).is_err() {
                    return NROS_RET_ERROR;
                }
            }

            loop {
                crate::executor::rclc_executor_spin_some(executor, 10_000_000);

                let exec = crate::executor::get_executor(&mut exec_t._opaque);
                let core = match exec.action_client_core_mut(internal.arena_entry_index as usize) {
                    Some(c) => c,
                    None => return NROS_RET_NOT_INIT,
                };
                match core.poll_server_discovery() {
                    Ok(Some(true)) => return NROS_RET_OK,
                    Ok(Some(false)) => break,
                    Ok(None) => {}
                    Err(_) => return NROS_RET_ERROR,
                }

                let elapsed_ns = crate::platform::get_time_ns().saturating_sub(start_ns);
                if elapsed_ns >= timeout_ns {
                    return NROS_RET_TIMEOUT;
                }
            }

            let elapsed_ns = crate::platform::get_time_ns().saturating_sub(start_ns);
            if elapsed_ns >= timeout_ns {
                return NROS_RET_TIMEOUT;
            }
        }
    }

    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (client, executor, timeout_ms);
        NROS_RET_OK
    }
}

/// Non-blocking snapshot of action-server visibility. Mirrors
/// `rclcpp_action::Client::action_server_is_ready`. Takes `executor`
/// for the same reason as
/// [`nros_action_client_wait_for_action_server`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_action_server_is_ready(
    client: *const nros_action_client_t,
    executor: *mut crate::executor::nros_executor_t,
) -> bool {
    if client.is_null() || executor.is_null() {
        return false;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let client_ref = &*client;
        if client_ref.state != nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_INITIALIZED {
            return false;
        }
        let internal = &client_ref._internal;
        if internal.executor_ptr.is_null() || internal.arena_entry_index < 0 {
            return false;
        }
        let exec_t = &mut *executor;
        let exec = crate::executor::get_executor(&mut exec_t._opaque);
        match exec.action_client_core_mut(internal.arena_entry_index as usize) {
            Some(core) => core.is_server_ready(),
            None => false,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (client, executor);
        true
    }
}

/// Send a goal request.
#[unsafe(no_mangle)]
/// Send a goal (blocking convenience).
///
/// Calls `nros_action_send_goal_async` then spins the executor until the
/// goal is accepted/rejected or timeout. Never calls `zpico_get` directly —
/// all I/O is driven by the executor's `spin_once`.
///
/// Like the runtime's `Promise::wait`, this is syntactic sugar over async + spin.
///
/// # Returns
///
/// Three outcomes, three codes — they were two before issue 0868 (see the C++
/// twin `nros_cpp_action_client_send_goal`):
///
/// * `NROS_RET_OK` — the server ACCEPTED; `goal_uuid` is filled.
/// * `NROS_RET_REJECTED` — the server RECEIVED the goal and declined it.
/// * `NROS_RET_TIMEOUT` — no goal response within the blocking budget. The
///   server may never have received the goal at all.
/// * `NROS_RET_ERROR` — the goal could not be SENT.
/// * `NROS_RET_REENTRANT` — called from inside a dispatch callback.
#[allow(static_mut_refs)]
pub unsafe extern "C" fn nros_action_send_goal(
    client: *mut nros_action_client_t,
    executor: *mut crate::executor::nros_executor_t,
    goal: *const u8,
    goal_len: usize,
    goal_uuid: *mut nros_goal_uuid_t,
) -> nros_ret_t {
    validate_not_null!(client, goal, goal_uuid, executor);

    // Reentrancy guard: this function spins the executor internally,
    // so it must not be called from inside a dispatch callback.
    let exec_ref = &*executor;
    if exec_ref.in_dispatch {
        return NROS_RET_REENTRANT;
    }

    // Send async
    let ret = nros_action_send_goal_async(client, goal, goal_len, goal_uuid);
    if ret != NROS_RET_OK {
        return ret;
    }

    // Install a temporary goal_response callback that sets a flag.
    // The arena's action_client_raw_try_process fires the trampoline
    // during spin_once, which reads client.goal_response_callback.
    //
    // Phase 214.G.1 — replaced `static mut BLOCKING_ACCEPTED: i32` with
    // an `AtomicI32` so the callback + spin loop have an explicit
    // happens-before via Release/Acquire ordering. Same single-call
    // contract (this blocking API expects one concurrent invocation)
    // but no longer relies on the absent fence between the callback's
    // store and the loop's load.
    let client_ref = &mut *client;
    use core::sync::atomic::{AtomicI32, Ordering};
    static BLOCKING_ACCEPTED: AtomicI32 = AtomicI32::new(-1); // -1=pending, 0=rejected, 1=accepted
    BLOCKING_ACCEPTED.store(-1, Ordering::Relaxed);

    let orig_cb = client_ref.goal_response_callback;
    let orig_ctx = client_ref.context;
    unsafe extern "C" fn blocking_goal_cb(
        _uuid: *const nros_goal_uuid_t,
        accepted: bool,
        _ctx: *mut core::ffi::c_void,
    ) {
        BLOCKING_ACCEPTED.store(if accepted { 1 } else { 0 }, Ordering::Release);
    }
    client_ref.goal_response_callback = Some(blocking_goal_cb);

    // Phase 89.12: wall-clock budget — same fix as 89.2 for the service
    // client. The old `for _ in 0..1000` loop assumed each
    // `spin_some(10ms)` actually spent ~10 ms, but on multi-threaded
    // zpico backends the inner condvar can wake on any incoming frame
    // (keep-alives, discovery gossip) and 1000 iterations exhaust in
    // milliseconds. On NuttX QEMU cold-boot the server-side goal
    // response easily slides past that window — budget by the clock.
    const ACTION_BLOCKING_TIMEOUT_MS: u64 = 15_000;
    let start_ns = crate::platform::get_time_ns();
    let timeout_ns: u64 = ACTION_BLOCKING_TIMEOUT_MS.saturating_mul(1_000_000);
    loop {
        crate::executor::rclc_executor_spin_some(executor, 10_000_000);
        let flag = BLOCKING_ACCEPTED.load(Ordering::Acquire);
        if flag >= 0 {
            client_ref.goal_response_callback = orig_cb;
            client_ref.context = orig_ctx;
            // issue 0868 — a server REJECTION is `REJECTED`, not `ERROR`.
            // Same split as the C++ twin `nros_cpp_action_client_send_goal`:
            // `ERROR` is what the async send below returns when the goal
            // could not be sent at all, so reusing it here left the caller
            // unable to tell "the server said no" from "it never left".
            return if flag == 1 {
                NROS_RET_OK
            } else {
                NROS_RET_REJECTED
            };
        }
        let elapsed_ns = crate::platform::get_time_ns().saturating_sub(start_ns);
        if elapsed_ns >= timeout_ns {
            break;
        }
    }
    client_ref.goal_response_callback = orig_cb;
    client_ref.context = orig_ctx;
    NROS_RET_TIMEOUT
}

/// Request cancellation of a goal.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_cancel_goal(
    client: *mut nros_action_client_t,
    goal_uuid: *const nros_goal_uuid_t,
) -> nros_ret_t {
    validate_not_null!(client, goal_uuid);

    let client = &mut *client;

    validate_state!(
        client,
        nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_INITIALIZED
    );

    let internal = &mut client._internal;
    let uuid = &*goal_uuid;
    let goal_id = nros_node::GoalId { uuid: uuid.uuid };

    let core = match unsafe { internal.arena_core_mut() } {
        Some(c) => c,
        None => return NROS_RET_NOT_INIT,
    };
    match core.send_cancel_request(&goal_id) {
        Ok(()) => NROS_RET_OK,
        Err(_) => NROS_RET_ERROR,
    }
}

/// Request result of a goal (blocking convenience).
///
/// Calls `nros_action_get_result_async` then spins the executor until the
/// result arrives or timeout. Never calls `zpico_get` directly.
#[unsafe(no_mangle)]
#[allow(static_mut_refs)]
pub unsafe extern "C" fn nros_action_get_result(
    client: *mut nros_action_client_t,
    executor: *mut crate::executor::nros_executor_t,
    goal_uuid: *const nros_goal_uuid_t,
    status: *mut nros_goal_status_t,
    result: *mut u8,
    result_capacity: usize,
    result_len: *mut usize,
) -> nros_ret_t {
    validate_not_null!(client, executor, goal_uuid, status, result, result_len);

    // Reentrancy guard: this function spins the executor internally,
    // so it must not be called from inside a dispatch callback.
    let exec_ref = &*executor;
    if exec_ref.in_dispatch {
        return NROS_RET_REENTRANT;
    }

    // Send get_result request async
    let ret = nros_action_get_result_async(client, goal_uuid);
    if ret != NROS_RET_OK {
        return ret;
    }

    // Install temporary result callback that captures the result into static buffers.
    //
    // Phase 214.G.1 — replaced `static mut BLK_RESULT_{LEN,STATUS}: i32`/`u8`
    // with `AtomicI32`/`AtomicU8`, and wrapped the byte buffer in a
    // `Sync`-asserting `UnsafeCell`. The callback writes the buffer +
    // STATUS then publishes via a `Release` store to LEN; the loop
    // observes LEN with `Acquire` and that fences the buffer read.
    // Single-call contract preserved.
    let client_ref = &mut *client;
    const BLK_RESULT_BUF_LEN: usize = 1024; // max captured action-result CDR
    use core::{
        cell::UnsafeCell,
        sync::atomic::{AtomicI32, AtomicU8, Ordering},
    };
    struct BlkResultBuf(UnsafeCell<[u8; BLK_RESULT_BUF_LEN]>);
    // SAFETY: cross-thread access is gated by BLK_RESULT_LEN's
    // Release/Acquire pair — the callback's store(Release) happens-
    // before the loop's load(Acquire) ≥ 0, which fences the buffer
    // write.
    unsafe impl Sync for BlkResultBuf {}
    static BLK_RESULT_LEN: AtomicI32 = AtomicI32::new(-1);
    static BLK_RESULT_STATUS: AtomicU8 = AtomicU8::new(0);
    static BLK_RESULT_BUF: BlkResultBuf = BlkResultBuf(UnsafeCell::new([0u8; BLK_RESULT_BUF_LEN]));
    BLK_RESULT_LEN.store(-1, Ordering::Relaxed);

    let orig_cb = client_ref.result_callback;
    let orig_ctx = client_ref.context;
    unsafe extern "C" fn blk_result_cb(
        _uuid: *const nros_goal_uuid_t,
        st: nros_goal_status_t,
        data: *const u8,
        len: usize,
        _ctx: *mut core::ffi::c_void,
    ) {
        BLK_RESULT_STATUS.store(st as u8, Ordering::Relaxed);
        let copy_len = len.min(1024);
        // SAFETY: BLK_RESULT_BUF lives forever; this fn runs in the
        // executor spin context that called the blocking API and only
        // one such call is in flight by contract.
        unsafe {
            core::ptr::copy_nonoverlapping(data, (*BLK_RESULT_BUF.0.get()).as_mut_ptr(), copy_len);
        }
        // Publish: any loop observing LEN ≥ 0 with Acquire sees the
        // buffer write above.
        BLK_RESULT_LEN.store(copy_len as i32, Ordering::Release);
    }
    client_ref.result_callback = Some(blk_result_cb);

    // Phase 89.12: wall-clock budget (same fix as the send_goal side).
    // Actions can legitimately run for several seconds; 30 s gives
    // room for a Fibonacci-10 feedback stream + result over QEMU slirp.
    const ACTION_RESULT_TIMEOUT_MS: u64 = 30_000;
    let start_ns = crate::platform::get_time_ns();
    let timeout_ns: u64 = ACTION_RESULT_TIMEOUT_MS.saturating_mul(1_000_000);
    loop {
        crate::executor::rclc_executor_spin_some(executor, 10_000_000);
        let rlen = BLK_RESULT_LEN.load(Ordering::Acquire);
        if rlen >= 0 {
            client_ref.result_callback = orig_cb;
            client_ref.context = orig_ctx;
            let data_len = rlen as usize;

            *status = match BLK_RESULT_STATUS.load(Ordering::Relaxed) {
                1 => nros_goal_status_t::NROS_GOAL_STATUS_ACCEPTED,
                2 => nros_goal_status_t::NROS_GOAL_STATUS_EXECUTING,
                3 => nros_goal_status_t::NROS_GOAL_STATUS_CANCELING,
                4 => nros_goal_status_t::NROS_GOAL_STATUS_SUCCEEDED,
                5 => nros_goal_status_t::NROS_GOAL_STATUS_CANCELED,
                6 => nros_goal_status_t::NROS_GOAL_STATUS_ABORTED,
                _ => nros_goal_status_t::NROS_GOAL_STATUS_UNKNOWN,
            };

            // Issue #179 — the dispatch paths now deliver the result WITH its
            // CDR encapsulation header (arena + poll both splice one in when a
            // C server's reply arrives raw), so copy verbatim. The old
            // unconditional re-header here double-framed a typed server's
            // reply and, combined with the poll path's off-by-alignment slice,
            // broke every zenoh C/C++ action client.
            if data_len > result_capacity {
                return NROS_RET_ERROR;
            }
            let out = core::slice::from_raw_parts_mut(result, result_capacity);
            // SAFETY: BLK_RESULT_LEN's Acquire load above fences the
            // buffer write in `blk_result_cb`.
            let buf = unsafe { &*BLK_RESULT_BUF.0.get() };
            out[..data_len].copy_from_slice(&buf[..data_len]);
            *result_len = data_len;
            return NROS_RET_OK;
        }
        let elapsed_ns = crate::platform::get_time_ns().saturating_sub(start_ns);
        if elapsed_ns >= timeout_ns {
            break;
        }
    }
    client_ref.result_callback = orig_cb;
    client_ref.context = orig_ctx;
    NROS_RET_TIMEOUT
}

/// Try to receive feedback for an active goal (non-blocking).
///
/// If feedback is available, invokes the feedback callback (if set).
/// Returns `NROS_RET_OK` if feedback was received and dispatched,
/// `NROS_RET_TIMEOUT` if no feedback is available.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_try_recv_feedback(
    client: *mut nros_action_client_t,
) -> nros_ret_t {
    validate_not_null!(client);

    let client = &mut *client;

    validate_state!(
        client,
        nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_INITIALIZED
    );

    let internal = &mut client._internal;

    let core = match unsafe { internal.arena_core_mut() } {
        Some(c) => c,
        None => return NROS_RET_NOT_INIT,
    };

    match core.try_recv_feedback_raw() {
        Ok(Some((goal_id, len))) => {
            if let Some(cb) = client.feedback_callback {
                let uuid = nros_goal_uuid_t { uuid: goal_id.uuid };

                // Feedback wire layout: [CDR_HEADER][GoalId seq prefix][UUID][feedback fields].
                // The C deserializer expects [CDR_HEADER][fields], so we
                // prepend a CDR header in a stack buffer.
                let fb_fields_len = len.saturating_sub(FEEDBACK_FRAMING_LEN);

                if fb_fields_len > 0 {
                    // Re-framed feedback: [CDR_HEADER][feedback_fields].
                    const FB_USER_BUF: usize = 512;
                    let mut fb_buf = [0u8; FB_USER_BUF];
                    let payload =
                        write_cdr_le_header(&mut fb_buf).expect("fb_buf >= CDR_HEADER_LEN");
                    let copy_len = fb_fields_len.min(payload.len());
                    payload[..copy_len].copy_from_slice(
                        &core.feedback_buffer_ref()
                            [FEEDBACK_FRAMING_LEN..FEEDBACK_FRAMING_LEN + copy_len],
                    );
                    cb(
                        &uuid,
                        fb_buf.as_ptr(),
                        CDR_HEADER_LEN + copy_len,
                        client.context,
                    );
                } else {
                    cb(&uuid, ptr::null(), 0, client.context);
                }
            }

            NROS_RET_OK
        }
        Ok(None) => NROS_RET_TIMEOUT,
        Err(_) => NROS_RET_ERROR,
    }
}

// ============================================================================
// Async (non-blocking) action client functions
// ============================================================================

/// Send a goal asynchronously (non-blocking).
///
/// Sends the goal request and returns immediately. The goal response
/// (accepted/rejected) arrives via the goal_response_callback registered
/// with `nros_action_client_set_goal_response_callback`, invoked during
/// `rclc_executor_spin_some`.
///
/// The `goal_uuid` output is filled with the generated goal UUID on success.
///
/// # Safety
/// All pointers must be valid. `goal` must point to `goal_len` bytes of
/// CDR-serialized goal data (with CDR header).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_send_goal_async(
    client: *mut nros_action_client_t,
    goal: *const u8,
    goal_len: usize,
    goal_uuid: *mut nros_goal_uuid_t,
) -> nros_ret_t {
    validate_not_null!(client, goal, goal_uuid);

    let client = &mut *client;

    validate_state!(
        client,
        nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_INITIALIZED
    );

    let internal = &mut client._internal;
    let goal_data = core::slice::from_raw_parts(goal, goal_len);

    // C serialize produces [CDR_HEADER][fields] — strip the header.
    let goal_fields = strip_cdr_header(goal_data);

    let core = match unsafe { internal.arena_core_mut() } {
        Some(c) => c,
        None => return NROS_RET_NOT_INIT,
    };

    // Non-blocking: uses zpico_get_start internally (not zpico_get).
    match core.send_goal_raw(goal_fields) {
        Ok(goal_id) => {
            let uuid = &mut *goal_uuid;
            uuid.uuid = goal_id.uuid;
            NROS_RET_OK
        }
        Err(_) => NROS_RET_ERROR,
    }
}

/// Request a goal result asynchronously (non-blocking).
///
/// Sends the get_result request and returns immediately. The result
/// arrives via the result_callback registered with
/// `nros_action_client_set_result_callback`, invoked during
/// `rclc_executor_spin_some`.
///
/// # Safety
/// All pointers must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_get_result_async(
    client: *mut nros_action_client_t,
    goal_uuid: *const nros_goal_uuid_t,
) -> nros_ret_t {
    validate_not_null!(client, goal_uuid);

    let client = &mut *client;

    validate_state!(
        client,
        nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_INITIALIZED
    );

    let internal = &mut client._internal;
    let uuid = &*goal_uuid;
    let goal_id = nros_node::GoalId { uuid: uuid.uuid };

    let core = match unsafe { internal.arena_core_mut() } {
        Some(c) => c,
        None => return NROS_RET_NOT_INIT,
    };

    // Non-blocking: uses zpico_get_start internally.
    match core.send_get_result_request(&goal_id) {
        Ok(()) => NROS_RET_OK,
        Err(_) => NROS_RET_ERROR,
    }
}

/// Set the goal response callback for async goal sending.
///
/// Called during `rclc_executor_spin_some` when the server accepts or
/// rejects a goal sent via `nros_action_send_goal_async`.
///
/// # Safety
/// `client` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_set_goal_response_callback(
    client: *mut nros_action_client_t,
    callback: nros_goal_response_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    validate_not_null!(client);

    let client = &mut *client;
    client.goal_response_callback = callback;
    if !context.is_null() {
        client.context = context;
    }
    NROS_RET_OK
}

/// Poll the action client for pending async replies (non-blocking).
///
/// **Note**: In the unified design (77.6+), `rclc_executor_spin_some` already
/// dispatches `action_client_raw_try_process` which invokes callbacks. This
/// function is provided for manual polling outside the executor loop.
///
/// # Safety
/// `client` must be a valid pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_poll(client: *mut nros_action_client_t) -> nros_ret_t {
    validate_not_null!(client);

    let client_ref = &mut *client;

    validate_state!(
        client_ref,
        nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_INITIALIZED
    );

    let internal = &mut client_ref._internal;
    let ctx = client_ref.context;

    let core = match unsafe { internal.arena_core_mut() } {
        Some(c) => c,
        None => return NROS_RET_NOT_INIT,
    };

    // Poll goal acceptance reply
    if let Ok(Some(total_len)) = core.try_recv_send_goal_reply()
        && let Some(cb) = client_ref.goal_response_callback
    {
        let buf = core.result_buffer_ref();
        let accepted = total_len >= 5 && buf[4] != 0;
        let uuid = nros_goal_uuid_t {
            uuid: {
                let mut u = [0u8; 16];
                u[..8].copy_from_slice(&core.goal_counter().to_le_bytes());
                u
            },
        };
        cb(&uuid, accepted, ctx);
    }

    // Poll feedback
    if let Ok(Some((goal_id, total_len))) = core.try_recv_feedback_raw()
        && let Some(cb) = client_ref.feedback_callback
    {
        let buf = core.feedback_buffer_ref();
        // NOTE: this polling path uses CDR_HEADER + UUID (20 bytes) whereas
        // `nros_action_try_recv_feedback` above uses CDR_HEADER + seq_prefix
        // + UUID (24). The discrepancy is tracked as a Phase 83 audit finding
        // — preserve current semantics until the feedback framing is unified
        // inside `ActionClientCore`.
        let offset = CDR_HEADER_LEN + GoalId::UUID_LEN;
        if total_len > offset {
            let uuid = nros_goal_uuid_t { uuid: goal_id.uuid };
            cb(
                &uuid,
                buf[offset..total_len].as_ptr(),
                total_len - offset,
                ctx,
            );
        }
    }

    // Poll result reply
    if let Ok(Some(total_len)) = core.try_recv_get_result_reply()
        && let Some(cb) = client_ref.result_callback
    {
        let buf = core.result_buffer_ref();
        if total_len >= 5 {
            let status_byte = buf[4];
            let c_status = match status_byte {
                4 => nros_goal_status_t::NROS_GOAL_STATUS_SUCCEEDED,
                5 => nros_goal_status_t::NROS_GOAL_STATUS_CANCELED,
                6 => nros_goal_status_t::NROS_GOAL_STATUS_ABORTED,
                _ => nros_goal_status_t::NROS_GOAL_STATUS_UNKNOWN,
            };
            // Issue #179 — the reply is `[CDR hdr 4][status i8][align(4)]
            // [result payload]`: the payload starts at stream-pos 4 = byte 8,
            // NOT byte 5. Slicing at 5 handed the three alignment-pad bytes
            // to the deserializer as the start of the result body → every
            // zenoh C/C++ action client failed `Failed to deserialize result`
            // on all platforms (feedback worked: its 4+16 framing lands
            // aligned by luck). Mirror the arena dispatch (executor/arena.rs
            // `RESULT_PAYLOAD_OFFSET`): payload@8, delivered WITH a CDR
            // encapsulation header — a typed/Rust server serialised one
            // in-line; a C server's `strip_cdr_header`→slab reply arrives
            // raw, so splice the reply's own (always-valid) top-level encap
            // in front. Callback contract: bytes = encap-headered result.
            let result_offset = (CDR_HEADER_LEN + 4).min(total_len);
            let uuid = nros_goal_uuid_t {
                uuid: {
                    let mut u = [0u8; 16];
                    u[..8].copy_from_slice(&core.goal_counter().to_le_bytes());
                    u
                },
            };
            let raw = &buf[result_offset..total_len];
            let has_encap =
                raw.len() >= 2 && raw[0] == 0 && matches!(raw[1], 0 | 1 | 6 | 7 | 0x0a | 0x0b);
            if has_encap {
                cb(&uuid, c_status, raw.as_ptr(), raw.len(), ctx);
            } else {
                const SPLICE_BUF: usize = 1024;
                let mut spliced = [0u8; SPLICE_BUF];
                let n = raw.len().min(SPLICE_BUF - CDR_HEADER_LEN);
                spliced[..CDR_HEADER_LEN].copy_from_slice(&buf[..CDR_HEADER_LEN]);
                spliced[CDR_HEADER_LEN..CDR_HEADER_LEN + n].copy_from_slice(&raw[..n]);
                cb(&uuid, c_status, spliced.as_ptr(), CDR_HEADER_LEN + n, ctx);
            }
        }
    }

    NROS_RET_OK
}

/// Finalize an action client.
// The reset path zeroes the probed-size `_opaque` slot, which exceeds
// clippy's 16 KB stack-array threshold. The assignment writes directly
// through the existing `*mut` — no transient stack copy in release.
#[allow(clippy::large_stack_arrays)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_fini(client: *mut nros_action_client_t) -> nros_ret_t {
    validate_not_null!(client);

    let client = &mut *client;

    match client.state {
        nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_INITIALIZED => {
            // L2: client lives in executor arena (if registered) —
            // reset metadata only.
        }
        nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_POLLING => {
            // L1: drop the inline ActionClientCore so its channel
            // handles' Drops run.
            #[cfg(feature = "rmw-cffi")]
            {
                core::ptr::drop_in_place(client._opaque.as_mut_ptr()
                    as *mut nros_node::ActionClientCore<
                        { crate::config::MESSAGE_BUFFER_SIZE },
                        { crate::config::MESSAGE_BUFFER_SIZE },
                        { crate::config::MESSAGE_BUFFER_SIZE },
                    >);
                client._opaque = [0u64; crate::opaque_sizes::ACTION_CLIENT_OPAQUE_U64S];
            }
        }
        _ => return NROS_RET_NOT_INIT,
    }

    client._internal = ActionClientInternal::new();
    client.feedback_callback = None;
    client.result_callback = None;
    client.context = ptr::null_mut();
    client.node = crate::node::nros_node_ref_t::none();
    client.state = nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_SHUTDOWN;

    NROS_RET_OK
}

// ============================================================================
// Phase 122.3.c.6.b — Layer-1 primitive entry points (caller polls)
// ============================================================================

/// Phase 122.3.c.6.b — initialize an L1 polling-mode action client.
///
/// Creates the 4 transport channels (3 service clients + feedback
/// subscriber) immediately and stores the `ActionClientCore` inline
/// in `_opaque`. The caller drives the goal lifecycle via:
/// * `nros_action_client_send_goal_raw` — start a new goal; returns
///   the generated 16-byte UUID.
/// * `nros_action_client_try_recv_goal_response_raw` — poll for the
///   send_goal RPC reply (accepted / rejected).
/// * `nros_action_client_send_get_result_request_raw` +
///   `_try_recv_result_raw` — fetch the terminal result.
/// * `nros_action_client_send_cancel_request_raw` +
///   `_try_recv_cancel_response_raw` — cancel a goal.
/// * `nros_action_client_try_recv_feedback_raw` — drain feedback.
///
/// # Safety
/// All pointers must be valid; `action_name` must be a valid
/// null-terminated string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_init_polling(
    client: *mut nros_action_client_t,
    node: *const nros_node_t,
    type_info: *const super::common::nros_action_type_t,
    action_name: *const core::ffi::c_char,
) -> nros_ret_t {
    validate_not_null!(client, node, type_info, action_name);

    let client_mut = &mut *client;
    let node_ref = &*node;
    let type_info_ref = &*type_info;

    validate_state!(
        client_mut,
        nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_UNINITIALIZED,
        NROS_RET_BAD_SEQUENCE
    );
    validate_state!(node_ref, nros_node_state_t::NROS_NODE_STATE_INITIALIZED);

    client_mut.action_name_len =
        crate::util::copy_cstr_into(action_name, &mut client_mut.action_name);
    if client_mut.action_name_len == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }
    client_mut.type_name_len =
        crate::util::copy_cstr_into(type_info_ref.type_name, &mut client_mut.type_name);
    client_mut.type_hash_len =
        crate::util::copy_cstr_into(type_info_ref.type_hash, &mut client_mut.type_hash);

    client_mut.node = unsafe { crate::node::node_ref_of(node) };

    #[cfg(feature = "rmw-cffi")]
    {
        // Phase 156 Sub-bug D — multi-Session dispatch (see
        // `rclc_publisher_init_default`).
        let (session, domain_id) = match crate::node::resolve_session_and_domain(node_ref) {
            Some(t) => t,
            None => return NROS_RET_NOT_INIT,
        };

        let action_str =
            core::str::from_utf8_unchecked(&client_mut.action_name[..client_mut.action_name_len]);
        let type_str =
            core::str::from_utf8_unchecked(&client_mut.type_name[..client_mut.type_name_len]);
        let type_hash_str =
            core::str::from_utf8_unchecked(&client_mut.type_hash[..client_mut.type_hash_len]);
        let node_name_str = core::str::from_utf8_unchecked(&node_ref.name[..node_ref.name_len]);
        let namespace_str =
            core::str::from_utf8_unchecked(&node_ref.namespace[..node_ref.namespace_len]);

        use nros_node::{ActionInfo, QoSProfile, ServiceInfo, Session, TopicInfo};
        let action_info =
            ActionInfo::new(action_str, type_str, type_hash_str).with_domain(domain_id);

        // Issue 0454 / phase-354 W3 — the per-CHANNEL DDS type, not the bare
        // action type. The type name is baked into the keyexpr, so advertising
        // `…Fibonacci_` where ROS 2 expects `…Fibonacci_SendGoal_` makes the
        // client's query and the server's queryable different keys: every goal
        // times out. phase-338 W3 fixed this on the raw REGISTER path
        // (`action_channel_type`'s own doc records it); these polling arms are
        // the same defect, unfixed because nothing called them until the
        // raw-goal probe did.
        let send_goal_keyexpr: nros_core::heapless::String<256> = action_info.send_goal_key();
        let send_goal_type: nros_core::heapless::String<256> =
            nros_node::action_channel_type(type_str, "SendGoal");
        let send_goal_info = ServiceInfo::new(&send_goal_keyexpr, &send_goal_type, type_hash_str)
            .with_domain(domain_id)
            .with_node_name(node_name_str)
            .with_namespace(namespace_str);
        let send_goal_client =
            match session.create_client(&send_goal_info, QoSProfile::services_default()) {
                Ok(h) => h,
                Err(_) => return NROS_RET_ERROR,
            };

        let cancel_goal_keyexpr: nros_core::heapless::String<256> = action_info.cancel_goal_key();
        let cancel_goal_info = ServiceInfo::new(
            &cancel_goal_keyexpr,
            "action_msgs::srv::dds_::CancelGoal_",
            type_hash_str,
        )
        .with_domain(domain_id)
        .with_node_name(node_name_str)
        .with_namespace(namespace_str);
        let cancel_goal_client =
            match session.create_client(&cancel_goal_info, QoSProfile::services_default()) {
                Ok(h) => h,
                Err(_) => return NROS_RET_ERROR,
            };

        let get_result_keyexpr: nros_core::heapless::String<256> = action_info.get_result_key();
        let get_result_type: nros_core::heapless::String<256> =
            nros_node::action_channel_type(type_str, "GetResult");
        let get_result_info =
            ServiceInfo::new(&get_result_keyexpr, &get_result_type, type_hash_str)
                .with_domain(domain_id)
                .with_node_name(node_name_str)
                .with_namespace(namespace_str);
        let get_result_client =
            match session.create_client(&get_result_info, QoSProfile::services_default()) {
                Ok(h) => h,
                Err(_) => return NROS_RET_ERROR,
            };

        let feedback_keyexpr: nros_core::heapless::String<256> = action_info.feedback_key();
        let feedback_type: nros_core::heapless::String<256> =
            nros_node::action_channel_type(type_str, "FeedbackMessage");
        let feedback_topic = TopicInfo::new(&feedback_keyexpr, &feedback_type, type_hash_str)
            .with_domain(domain_id)
            .with_node_name(node_name_str)
            .with_namespace(namespace_str);
        let feedback_subscriber =
            match session.create_subscription(&feedback_topic, QoSProfile::BEST_EFFORT) {
                Ok(h) => h,
                Err(_) => return NROS_RET_ERROR,
            };

        type Core = nros_node::ActionClientCore<
            { crate::config::MESSAGE_BUFFER_SIZE },
            { crate::config::MESSAGE_BUFFER_SIZE },
            { crate::config::MESSAGE_BUFFER_SIZE },
        >;
        let core = Core::new(
            send_goal_client,
            cancel_goal_client,
            get_result_client,
            feedback_subscriber,
        );
        core::ptr::write(client_mut._opaque.as_mut_ptr() as *mut Core, core);
    }

    client_mut.state = nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_POLLING;
    NROS_RET_OK
}

#[cfg(feature = "rmw-cffi")]
type PollingClientCore = nros_node::ActionClientCore<
    { crate::config::MESSAGE_BUFFER_SIZE },
    { crate::config::MESSAGE_BUFFER_SIZE },
    { crate::config::MESSAGE_BUFFER_SIZE },
>;

#[cfg(feature = "rmw-cffi")]
#[inline]
unsafe fn polling_client_core(
    client: *mut nros_action_client_t,
) -> Option<&'static mut PollingClientCore> {
    if client.is_null() {
        return None;
    }
    let client_mut = &mut *client;
    if client_mut.state != nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_POLLING {
        return None;
    }
    Some(&mut *(client_mut._opaque.as_mut_ptr() as *mut PollingClientCore))
}

/// Phase 122.3.c.6.b — L1 polling: send a goal. Writes the generated
/// 16-byte UUID into `goal_id_out`. Poll for the accept/reject reply
/// via `nros_action_client_try_recv_goal_response_raw`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_send_goal_raw(
    client: *mut nros_action_client_t,
    goal_cdr: *const u8,
    goal_len: usize,
    goal_id_out: *mut [u8; 16],
) -> nros_ret_t {
    if client.is_null() || goal_id_out.is_null() || (goal_cdr.is_null() && goal_len != 0) {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_client_core(client) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        // Issue 0454 / phase-354 W3 — STRIP the encapsulation header.
        //
        // The parameter is `goal_cdr`: a C caller supplies CDR, which begins
        // with the 4-byte encapsulation header. `PollingActionClientCore::
        // send_goal_raw` wants goal FIELDS — its own doc says "without GoalId
        // framing", and it appends the bytes after a header `tx_writer`
        // (`CdrWriter::new_with_header`) has already written. Passing CDR
        // through unstripped therefore put TWO headers on the wire, which is
        // issue 0448's defect reaching a peer.
        //
        // The two INTERNAL callers already did this (`strip_cdr_header(goal_data)`
        // -> `goal_fields`); only the FFI arms did not, so the parameter name
        // stated one contract and the code implemented another.
        let slice = core::slice::from_raw_parts(goal_cdr, goal_len);
        match core.send_goal_raw(strip_cdr_header(slice)) {
            Ok(id) => {
                (*goal_id_out).copy_from_slice(&id.uuid);
                NROS_RET_OK
            }
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (goal_cdr, goal_len, goal_id_out);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.b — L1 polling: try to receive a send_goal reply
/// (the accept/reject response). Returns `0` when no reply yet, `>0`
/// bytes when one was copied into `buf`, negative on error.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_try_recv_goal_response_raw(
    client: *mut nros_action_client_t,
    buf: *mut u8,
    buf_len: usize,
) -> i32 {
    if client.is_null() || (buf.is_null() && buf_len != 0) {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_client_core(client) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        match core.try_recv_send_goal_reply() {
            Ok(Some(len)) => {
                let src = core.result_buffer_ref();
                let copy_len = len.min(buf_len);
                core::ptr::copy_nonoverlapping(src.as_ptr(), buf, copy_len);
                copy_len as i32
            }
            Ok(None) => 0,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (buf, buf_len);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.b — L1 polling: send a get_result request for the
/// given goal. Reply lands via `_try_recv_result_raw`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_send_get_result_request_raw(
    client: *mut nros_action_client_t,
    goal_id: *const [u8; 16],
) -> nros_ret_t {
    if client.is_null() || goal_id.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_client_core(client) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        let id = nros::GoalId { uuid: *goal_id };
        match core.send_get_result_request(&id) {
            Ok(_) => NROS_RET_OK,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = goal_id;
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.b — L1 polling: try to receive a get_result
/// reply. Returns `0` when no reply yet, `>0` bytes copied, negative
/// on error.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_try_recv_result_raw(
    client: *mut nros_action_client_t,
    buf: *mut u8,
    buf_len: usize,
) -> i32 {
    if client.is_null() || (buf.is_null() && buf_len != 0) {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_client_core(client) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        match core.try_recv_get_result_reply() {
            Ok(Some(len)) => {
                let src = core.result_buffer_ref();
                let copy_len = len.min(buf_len);
                core::ptr::copy_nonoverlapping(src.as_ptr(), buf, copy_len);
                copy_len as i32
            }
            Ok(None) => 0,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (buf, buf_len);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.b — L1 polling: send a cancel request.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_send_cancel_request_raw(
    client: *mut nros_action_client_t,
    goal_id: *const [u8; 16],
) -> nros_ret_t {
    if client.is_null() || goal_id.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_client_core(client) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        let id = nros::GoalId { uuid: *goal_id };
        match core.send_cancel_request(&id) {
            Ok(_) => NROS_RET_OK,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = goal_id;
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.c — L1 polling: try to receive the cancel-RPC
/// reply. Returns `0` when no reply yet, `>0` bytes copied into
/// `buf`, negative on error.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_try_recv_cancel_response_raw(
    client: *mut nros_action_client_t,
    buf: *mut u8,
    buf_len: usize,
) -> i32 {
    if client.is_null() || (buf.is_null() && buf_len != 0) {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_client_core(client) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        match core.try_recv_cancel_reply() {
            Ok(Some(len)) => {
                let src = core.result_buffer_ref();
                let copy_len = len.min(buf_len);
                core::ptr::copy_nonoverlapping(src.as_ptr(), buf, copy_len);
                copy_len as i32
            }
            Ok(None) => 0,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (buf, buf_len);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.e — register a C wake callback on the
/// send_goal-reply channel of an L1 polling-mode action client.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_set_goal_response_wake_callback(
    client: *mut nros_action_client_t,
    state: *mut crate::service::nros_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_ret_t {
    set_action_client_wake_callback(client, state, cb, ctx, ClientChannel::GoalResponse)
}

/// Phase 122.3.c.6.e — wake on the cancel-reply channel.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_set_cancel_response_wake_callback(
    client: *mut nros_action_client_t,
    state: *mut crate::service::nros_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_ret_t {
    set_action_client_wake_callback(client, state, cb, ctx, ClientChannel::CancelResponse)
}

/// Phase 122.3.c.6.e — wake on the get_result-reply channel.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_set_result_wake_callback(
    client: *mut nros_action_client_t,
    state: *mut crate::service::nros_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_ret_t {
    set_action_client_wake_callback(client, state, cb, ctx, ClientChannel::Result)
}

/// Phase 122.3.c.6.e — wake on the feedback channel.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_set_feedback_wake_callback(
    client: *mut nros_action_client_t,
    state: *mut crate::service::nros_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_ret_t {
    set_action_client_wake_callback(client, state, cb, ctx, ClientChannel::Feedback)
}

enum ClientChannel {
    GoalResponse,
    CancelResponse,
    Result,
    Feedback,
}

unsafe fn set_action_client_wake_callback(
    client: *mut nros_action_client_t,
    state: *mut crate::service::nros_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
    channel: ClientChannel,
) -> nros_ret_t {
    if client.is_null() || state.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_client_core(client) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        let state_ptr = state as *mut nros_node::c_waker::CWakeState;
        core::ptr::write(
            state_ptr,
            nros_node::c_waker::CWakeState { fn_ptr: cb, ctx },
        );
        let waker = nros_node::c_waker::make_waker(state_ptr);
        match channel {
            ClientChannel::GoalResponse => core.register_goal_response_waker(&waker),
            ClientChannel::CancelResponse => core.register_cancel_response_waker(&waker),
            ClientChannel::Result => core.register_result_waker(&waker),
            ClientChannel::Feedback => core.register_feedback_waker(&waker),
        }
        NROS_RET_OK
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (state, cb, ctx, channel);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.b — L1 polling: try to receive feedback for any
/// goal. Returns `0` when no feedback yet, `>0` bytes copied (with
/// `goal_id_out` filled), negative on error.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_client_try_recv_feedback_raw(
    client: *mut nros_action_client_t,
    buf: *mut u8,
    buf_len: usize,
    goal_id_out: *mut [u8; 16],
) -> i32 {
    if client.is_null() || (buf.is_null() && buf_len != 0) || goal_id_out.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_client_core(client) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        match core.try_recv_feedback_raw() {
            Ok(Some((id, len))) => {
                let src = core.feedback_buffer_ref();
                let copy_len = len.min(buf_len);
                core::ptr::copy_nonoverlapping(src.as_ptr(), buf, copy_len);
                (*goal_id_out).copy_from_slice(&id.uuid);
                copy_len as i32
            }
            Ok(None) => 0,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (buf, buf_len, goal_id_out);
        NROS_RET_NOT_INIT
    }
}

// ============================================================================
// Kani Verification
// ============================================================================

#[cfg(kani)]
mod verification {
    use super::*;
    use crate::error::*;
    use core::ptr;

    // Helper to create a dummy action type info
    fn dummy_action_type() -> nros_action_type_t {
        let type_name = b"example_interfaces::action::dds_::Fibonacci_\0";
        let type_hash = b"RIHS01_test\0";
        nros_action_type_t {
            type_name: type_name.as_ptr() as *const core::ffi::c_char,
            type_hash: type_hash.as_ptr() as *const core::ffi::c_char,
            goal_serialized_size_max: 8,
            result_serialized_size_max: 264,
            feedback_serialized_size_max: 264,
        }
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn action_client_init_null_ptrs() {
        let action_name = b"/fibonacci\0";
        let type_info = dummy_action_type();
        let node = crate::node::rcl_get_zero_initialized_node();

        // NULL client
        assert_eq!(
            unsafe {
                nros_action_client_init(
                    ptr::null_mut(),
                    &node,
                    action_name.as_ptr() as *const core::ffi::c_char,
                    &type_info,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL node
        let mut cli = rcl_action_get_zero_initialized_client();
        assert_eq!(
            unsafe {
                nros_action_client_init(
                    &mut cli,
                    ptr::null(),
                    action_name.as_ptr() as *const core::ffi::c_char,
                    &type_info,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL action_name
        let mut cli = rcl_action_get_zero_initialized_client();
        assert_eq!(
            unsafe { nros_action_client_init(&mut cli, &node, ptr::null(), &type_info) },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL type_info
        let mut cli = rcl_action_get_zero_initialized_client();
        assert_eq!(
            unsafe {
                nros_action_client_init(
                    &mut cli,
                    &node,
                    action_name.as_ptr() as *const core::ffi::c_char,
                    ptr::null(),
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn action_client_init_uninit_node() {
        let action_name = b"/fibonacci\0";
        let type_info = dummy_action_type();
        let node = crate::node::rcl_get_zero_initialized_node();

        let mut cli = rcl_action_get_zero_initialized_client();
        assert_eq!(
            unsafe {
                nros_action_client_init(
                    &mut cli,
                    &node,
                    action_name.as_ptr() as *const core::ffi::c_char,
                    &type_info,
                )
            },
            NROS_RET_NOT_INIT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn action_client_zero_initialized_state() {
        let cli = rcl_action_get_zero_initialized_client();
        assert_eq!(
            cli.state,
            nros_action_client_state_t::NROS_ACTION_CLIENT_STATE_UNINITIALIZED,
        );
        assert!(cli.node.is_null());
        assert_eq!(cli._internal.arena_entry_index, -1);
        assert!(cli._internal.executor_ptr.is_null());
        assert!(cli.feedback_callback.is_none());
        assert!(cli.result_callback.is_none());
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn action_client_fini_null_safety() {
        // NULL → INVALID_ARGUMENT
        assert_eq!(
            unsafe { nros_action_client_fini(ptr::null_mut()) },
            NROS_RET_INVALID_ARGUMENT,
        );

        // UNINITIALIZED → NOT_INIT
        let mut cli = rcl_action_get_zero_initialized_client();
        assert_eq!(
            unsafe { nros_action_client_fini(&mut cli) },
            NROS_RET_NOT_INIT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn action_send_goal_null_ptrs() {
        let goal_data = [0u8; 8];
        let mut uuid = nros_goal_uuid_t::default();

        // NULL client
        assert_eq!(
            unsafe { nros_action_send_goal(ptr::null_mut(), goal_data.as_ptr(), 8, &mut uuid) },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL goal
        let mut cli = rcl_action_get_zero_initialized_client();
        assert_eq!(
            unsafe { nros_action_send_goal(&mut cli, ptr::null(), 0, &mut uuid) },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL uuid
        assert_eq!(
            unsafe { nros_action_send_goal(&mut cli, goal_data.as_ptr(), 8, ptr::null_mut()) },
            NROS_RET_INVALID_ARGUMENT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn action_get_result_null_ptrs() {
        let uuid = nros_goal_uuid_t::default();
        let mut status = nros_goal_status_t::NROS_GOAL_STATUS_UNKNOWN;
        let mut result_buf = [0u8; 8];
        let mut result_len: usize = 0;

        // NULL client
        assert_eq!(
            unsafe {
                nros_action_get_result(
                    ptr::null_mut(),
                    &uuid,
                    &mut status,
                    result_buf.as_mut_ptr(),
                    8,
                    &mut result_len,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL uuid
        let mut cli = rcl_action_get_zero_initialized_client();
        assert_eq!(
            unsafe {
                nros_action_get_result(
                    &mut cli,
                    ptr::null(),
                    &mut status,
                    result_buf.as_mut_ptr(),
                    8,
                    &mut result_len,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL status
        assert_eq!(
            unsafe {
                nros_action_get_result(
                    &mut cli,
                    &uuid,
                    ptr::null_mut(),
                    result_buf.as_mut_ptr(),
                    8,
                    &mut result_len,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL result
        assert_eq!(
            unsafe {
                nros_action_get_result(
                    &mut cli,
                    &uuid,
                    &mut status,
                    ptr::null_mut(),
                    8,
                    &mut result_len,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL result_len
        assert_eq!(
            unsafe {
                nros_action_get_result(
                    &mut cli,
                    &uuid,
                    &mut status,
                    result_buf.as_mut_ptr(),
                    8,
                    ptr::null_mut(),
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );
    }
}
