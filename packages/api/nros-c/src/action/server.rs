//! Action server implementation.

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
/// See [`nros_node::GoalId`] encoding in `CLAUDE.md`.
const GOAL_ID_SEQ_PREFIX_LEN: usize = GoalId::SEQ_PREFIX_LEN;

/// Bytes of CDR framing that precede the goal payload in a send_goal request:
/// CDR encapsulation header + GoalId sequence length prefix + UUID.
const GOAL_REQUEST_FRAMING_LEN: usize = CDR_HEADER_LEN + GOAL_ID_SEQ_PREFIX_LEN + GoalId::UUID_LEN;

// ============================================================================
// Action Server
// ============================================================================

/// Action server state.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_action_server_state_t {
    /// Not initialized
    NROS_ACTION_SERVER_STATE_UNINITIALIZED = 0,
    /// L2 callback-mode: initialized via `nros_action_server_init`;
    /// transport creation deferred to
    /// `nros_executor_add_action_server`.
    NROS_ACTION_SERVER_STATE_INITIALIZED = 1,
    /// Shutdown
    NROS_ACTION_SERVER_STATE_SHUTDOWN = 2,
    /// Phase 122.3.c.6.b — L1 polling-mode: `ActionServerCore` lives
    /// inline in `_opaque`; caller drives via the
    /// `nros_action_server_*_raw` family. No executor registration.
    NROS_ACTION_SERVER_STATE_POLLING = 3,
}

/// Action server structure.
#[repr(C)]
pub struct nros_action_server_t {
    /// Current state
    pub state: nros_action_server_state_t,
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
    /// Goal callback
    pub goal_callback: nros_goal_callback_t,
    /// Cancel callback
    pub cancel_callback: nros_cancel_callback_t,
    /// Accepted callback
    pub accepted_callback: nros_accepted_callback_t,
    /// User context pointer
    pub context: *mut c_void,
    /// Pointer to parent node
    pub node: crate::node::nros_node_ref_t,
    /// Phase 193.4b — action-server QoS, applied to the three underlying
    /// service servers (send_goal / cancel_goal / get_result). The feedback +
    /// status publishers keep their own profiles. Defaults to the services
    /// profile (RELIABLE+VOLATILE+KEEP_LAST(10)); set via
    /// `nros_action_server_init_with_qos`.
    pub qos: crate::qos::nros_qos_t,
    /// Phase 189.M3.3.b — scheduling-context slot to bind the action server's
    /// executor handle to (the goal-service slot; governs the action's callback
    /// dispatch). `0` = inherit the executor / Node default; set via
    /// `nros_action_server_init_with_options`. When non-zero,
    /// `nros_executor_add_action_server` binds the handle after
    /// registration. No effect on the L1 polling path.
    pub sched_context_id: crate::executor::nros_sched_context_id_t,
    /// Internal state — set by `nros_executor_add_action_server`.
    /// Typed C-ABI handle field (was an opaque blob in earlier versions).
    pub _internal: ActionServerInternal,
    /// Phase 122.3.c.6.b — inline opaque storage for the L1
    /// polling-mode `ActionServerCore`. Zeroed in L2 mode; populated
    /// by `nros_action_server_init_polling`.
    pub _opaque: [u64; crate::opaque_sizes::ACTION_SERVER_OPAQUE_U64S],
}

impl Default for nros_action_server_t {
    fn default() -> Self {
        Self {
            state: nros_action_server_state_t::NROS_ACTION_SERVER_STATE_UNINITIALIZED,
            action_name: [0u8; MAX_ACTION_NAME_LEN],
            action_name_len: 0,
            type_name: [0u8; MAX_TYPE_NAME_LEN],
            type_name_len: 0,
            type_hash: [0u8; MAX_TYPE_HASH_LEN],
            type_hash_len: 0,
            goal_callback: None,
            cancel_callback: None,
            accepted_callback: None,
            context: ptr::null_mut(),
            node: crate::node::nros_node_ref_t::none(),
            qos: crate::qos::nros_qos_t::default(),
            sched_context_id: 0,
            _internal: ActionServerInternal::invalid_default(),
            _opaque: [0u64; crate::opaque_sizes::ACTION_SERVER_OPAQUE_U64S],
        }
    }
}

impl nros_action_server_t {
    /// Phase 193.4b — the server's QoS as `nros_node` settings.
    pub(crate) fn get_qos_settings(&self) -> nros_rmw::QoSProfile {
        self.qos.to_qos_settings()
    }
}

// ============================================================================
// Internal implementation (delegates to nros-node executor)
// ============================================================================

/// Internal state created during executor registration.
///
/// Holds the action server handle and C callback pointers needed by
/// the goal/cancel trampolines.
///
/// `handle` was `Option<ActionServerRawHandle>` previously; now it is
/// always present, with the sentinel `INVALID_ENTRY_INDEX` indicating
/// "not registered yet". Use `is_handle_set()` to check.
#[repr(C)]
pub struct ActionServerInternal {
    /// Handle returned by executor registration. `entry_index ==
    /// INVALID_ENTRY_INDEX` until registration completes.
    pub handle: nros_node::ActionServerRawHandle,
    /// Pointer to the internal executor (`CExecutor`).
    pub executor_ptr: *mut c_void,
    /// C goal callback from init. Required.
    pub c_goal_callback: unsafe extern "C" fn(
        *mut nros_action_server_t,
        *const nros_goal_handle_t,
        *const u8,
        usize,
        *mut c_void,
    ) -> nros_goal_response_t,
    /// C cancel callback from init (may be None).
    pub c_cancel_callback: nros_cancel_callback_t,
    /// C accepted callback from init (may be None).
    pub c_accepted_callback: nros_accepted_callback_t,
    /// C user context from init.
    pub c_context: *mut c_void,
    /// Pointer back to the C action server struct.
    pub server_ptr: *mut nros_action_server_t,
}

impl ActionServerInternal {
    /// `true` once `handle` has been populated by executor registration.
    #[inline]
    pub fn is_handle_set(&self) -> bool {
        !self.handle.is_invalid()
    }

    /// Construct a zero-initialised internal — sentinel handle, null pointers,
    /// no callbacks. Used for the default state of `nros_action_server_t._internal`.
    pub fn invalid_default() -> Self {
        Self {
            handle: nros_node::ActionServerRawHandle::invalid(),
            executor_ptr: ptr::null_mut(),
            c_goal_callback: dummy_goal_callback,
            c_cancel_callback: None,
            c_accepted_callback: None,
            c_context: ptr::null_mut(),
            server_ptr: ptr::null_mut(),
        }
    }
}

impl Default for ActionServerInternal {
    fn default() -> Self {
        Self::invalid_default()
    }
}

/// Stub for the required `c_goal_callback` field when the internal is in
/// its uninitialised state. Never called — the dispatch path checks
/// `is_handle_set()` first.
unsafe extern "C" fn dummy_goal_callback(
    _server: *mut nros_action_server_t,
    _goal: *const nros_goal_handle_t,
    _data: *const u8,
    _len: usize,
    _ctx: *mut c_void,
) -> nros_goal_response_t {
    nros_goal_response_t::NROS_GOAL_REJECT
}

/// Build a stack-local `nros_goal_handle_t` from an arena-supplied UUID.
///
/// The handle is a pure ID card (`{uuid}`), so the trampolines can hand
/// user callbacks a `*const nros_goal_handle_t` that's valid for the
/// duration of the callback. Users copy the handle by value if they need
/// the UUID beyond the callback's lifetime.
unsafe fn handle_from_goal_id(goal_id: *const nros_node::GoalId) -> nros_goal_handle_t {
    nros_goal_handle_t {
        uuid: nros_goal_uuid_t {
            uuid: (*goal_id).uuid,
        },
    }
}

/// Goal callback trampoline for nros-node.
///
/// Builds a stack-local `nros_goal_handle_t` from the incoming goal_id
/// and forwards to the user's `c_goal_callback` with `(server, goal,
/// request, len, ctx)`. The handle lives on this stack frame; users that
/// need to reference the goal later copy it by value.
pub(crate) unsafe extern "C" fn goal_callback_trampoline(
    goal_id: *const nros_node::GoalId,
    goal_data: *const u8,
    goal_len: usize,
    context: *mut c_void,
) -> nros_node::GoalResponse {
    let internal = &*(context as *const ActionServerInternal);
    let goal_handle = handle_from_goal_id(goal_id);

    // goal_data contains the full CDR request: [CDR_HDR][GoalId seq][UUID][goal_fields].
    // The C callback expects CDR-encoded goal data: [CDR_HDR][goal_fields].
    // Extract goal fields (after CDR header + GoalId) and prepend a CDR header.
    let goal_slice = core::slice::from_raw_parts(goal_data, goal_len);

    // Build [CDR_HEADER][goal_fields] on the stack (must outlive the callback)
    const GOAL_CB_BUF: usize = 512;
    let mut cb_buf = [0u8; GOAL_CB_BUF];
    let (cb_ptr, cb_len) = if goal_len > GOAL_REQUEST_FRAMING_LEN {
        let fields = &goal_slice[GOAL_REQUEST_FRAMING_LEN..];
        let payload = write_cdr_le_header(&mut cb_buf).expect("cb_buf >= CDR_HEADER_LEN");
        let copy_len = fields.len().min(payload.len());
        payload[..copy_len].copy_from_slice(&fields[..copy_len]);
        (cb_buf.as_ptr(), CDR_HEADER_LEN + copy_len)
    } else {
        (goal_data, goal_len)
    };

    let c_response = (internal.c_goal_callback)(
        internal.server_ptr,
        &goal_handle,
        cb_ptr,
        cb_len,
        internal.c_context,
    );

    match c_response {
        nros_goal_response_t::NROS_GOAL_REJECT => nros_node::GoalResponse::Reject,
        nros_goal_response_t::NROS_GOAL_ACCEPT_AND_EXECUTE => {
            nros_node::GoalResponse::AcceptAndExecute
        }
        nros_goal_response_t::NROS_GOAL_ACCEPT_AND_DEFER => nros_node::GoalResponse::AcceptAndDefer,
    }
}

/// Post-accept trampoline for nros-node.
///
/// Invoked by the arena *after* `ActionServerCore::accept_goal` has sent
/// the accept reply to the client. Forwards to the user's
/// `c_accepted_callback` with a stack-local handle.
pub(crate) unsafe extern "C" fn accepted_callback_trampoline(
    goal_id: *const nros_node::GoalId,
    context: *mut c_void,
) {
    let internal = &*(context as *const ActionServerInternal);
    if let Some(cb) = internal.c_accepted_callback {
        let goal_handle = handle_from_goal_id(goal_id);
        cb(internal.server_ptr, &goal_handle, internal.c_context);
    }
}

/// Cancel callback trampoline for nros-node.
///
/// Forwards to the user's `c_cancel_callback` with a stack-local handle.
/// Status transitions (CANCELING, etc.) are performed by the arena in
/// `ActionServerCore`; this trampoline only translates the ABI.
pub(crate) unsafe extern "C" fn cancel_callback_trampoline(
    goal_id: *const nros_node::GoalId,
    _status: nros_node::GoalStatus,
    context: *mut c_void,
) -> nros_node::CancelResponse {
    let internal = &*(context as *const ActionServerInternal);

    let Some(cb) = internal.c_cancel_callback else {
        // No cancel callback — accept by default.
        return nros_node::CancelResponse::Accept;
    };

    let goal_handle = handle_from_goal_id(goal_id);
    let c_response = cb(internal.server_ptr, &goal_handle, internal.c_context);
    // issue 0796 — one-to-one now: C's REJECT=0/ACCEPT=1 and Rust's
    // `CancelResponse::{Reject,Accept}` are the same per-goal decision with the
    // same discriminants. This used to translate into the RPC-level return
    // code (`Ok`/`Rejected`), which is why it needed a comment explaining that
    // "Ok" meant "accepted".
    match c_response {
        nros_cancel_response_t::NROS_CANCEL_ACCEPT => nros_node::CancelResponse::Accept,
        nros_cancel_response_t::NROS_CANCEL_REJECT => nros_node::CancelResponse::Reject,
    }
}

/// Get the `ActionServerInternal` from a server's inline `_internal` storage.
///
/// # Safety
/// The server must have been registered with the executor (state = INITIALIZED
/// and `_internal` written via `ptr::write`).
unsafe fn get_internal(server: *const nros_action_server_t) -> &'static ActionServerInternal {
    &(*server)._internal
}

// ============================================================================
// Action Server Functions
// ============================================================================

/// Get a zero-initialized action server.
///
/// `improper_ctypes_definitions` is silenced because the inline opaque
/// handle stores a function-pointer field whose parameter signature
/// references a runtime-internal type. The pointer itself is C-ABI
/// safe; the C API never lets callers invoke through it directly.
#[unsafe(no_mangle)]
#[allow(improper_ctypes_definitions)]
pub extern "C" fn rcl_action_get_zero_initialized_server() -> nros_action_server_t {
    nros_action_server_t::default()
}

/// Initialize an action server.
///
/// Stores metadata (name, type, callbacks). RMW entity creation is deferred
/// to `nros_executor_add_action_server()`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_init(
    server: *mut nros_action_server_t,
    node: *const nros_node_t,
    action_name: *const core::ffi::c_char,
    type_info: *const nros_action_type_t,
    goal_callback: nros_goal_callback_t,
    cancel_callback: nros_cancel_callback_t,
    accepted_callback: nros_accepted_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    validate_not_null!(server, node, action_name, type_info);

    if goal_callback.is_none() {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let server = &mut *server;
    let node_ref = &*node;
    let type_info = &*type_info;

    validate_state!(
        server,
        nros_action_server_state_t::NROS_ACTION_SERVER_STATE_UNINITIALIZED,
        NROS_RET_BAD_SEQUENCE
    );
    validate_state!(node_ref, nros_node_state_t::NROS_NODE_STATE_INITIALIZED);

    // Copy action name (required — empty rejected)
    server.action_name_len = crate::util::copy_cstr_into(action_name, &mut server.action_name);
    if server.action_name_len == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }

    // Copy type name + hash (both optional — null sources leave dst untouched)
    server.type_name_len = crate::util::copy_cstr_into(type_info.type_name, &mut server.type_name);
    server.type_hash_len = crate::util::copy_cstr_into(type_info.type_hash, &mut server.type_hash);

    // Store callbacks and context
    server.goal_callback = goal_callback;
    server.cancel_callback = cancel_callback;
    server.accepted_callback = accepted_callback;
    server.context = context;
    server.node = unsafe { crate::node::node_ref_of(node) };

    // Phase 193.4b — default to the services profile;
    // nros_action_server_init_with_qos overrides. Read at registration time by
    // nros_executor_add_action_server.
    server.qos = crate::qos::nros_qos_t::default();

    // RMW entity creation is deferred to nros_executor_add_action_server()
    server._internal = ActionServerInternal::invalid_default();
    server.state = nros_action_server_state_t::NROS_ACTION_SERVER_STATE_INITIALIZED;

    NROS_RET_OK
}

/// Phase 193.4b — initialize an action server with an explicit QoS profile
/// (rclc's `_with_options`). The profile applies to the three underlying
/// service servers (send_goal / cancel_goal / get_result); the feedback +
/// status publishers keep their own profiles. `qos` NULL ⇒ the services
/// default. Same as [`nros_action_server_init`] otherwise.
///
/// # Safety
/// All non-NULL pointers must be valid + the node initialized.
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn nros_action_server_init_with_qos(
    server: *mut nros_action_server_t,
    node: *const nros_node_t,
    action_name: *const core::ffi::c_char,
    type_info: *const nros_action_type_t,
    goal_callback: nros_goal_callback_t,
    cancel_callback: nros_cancel_callback_t,
    accepted_callback: nros_accepted_callback_t,
    context: *mut c_void,
    qos: *const crate::qos::nros_qos_t,
) -> nros_ret_t {
    let ret = nros_action_server_init(
        server,
        node,
        action_name,
        type_info,
        goal_callback,
        cancel_callback,
        accepted_callback,
        context,
    );
    if ret == NROS_RET_OK && !qos.is_null() {
        (*server).qos = *qos;
    }
    ret
}

/// Phase 189.M3.3.b — rclc-style named action-server options. QoS is passed
/// separately; this carries the non-QoS axes. Zero-init = default behaviour.
#[repr(C)]
#[derive(Default)]
pub struct nros_action_server_options_t {
    /// Scheduling-context slot to bind the action server's executor handle to.
    /// `0` = inherit the executor / Node default. A non-zero value must be an id
    /// from `nros_executor_create_sched_context`; the bind is applied by
    /// `nros_executor_add_action_server` once the handle exists. No effect
    /// on the L1 polling path.
    pub sched_context: crate::executor::nros_sched_context_id_t,
    /// Reserved for future use; must be zero. Pads for ABI stability.
    pub _reserved: [u8; 3],
}

/// Get a zero-initialised [`nros_action_server_options_t`] (`sched_context = 0`).
#[unsafe(no_mangle)]
pub extern "C" fn rcl_action_server_get_default_options() -> nros_action_server_options_t {
    nros_action_server_options_t::default()
}

/// Phase 189.M3.3.b — initialize an action server with custom QoS + named
/// options. Like [`nros_action_server_init_with_qos`] except a non-zero
/// `options->sched_context` is stashed so [`nros_executor_add_action_server`]
/// binds the resulting executor handle to that scheduling context once known.
///
/// # Safety
/// All non-NULL pointers must be valid + the node initialized; `qos` / `options`
/// may be NULL.
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn nros_action_server_init_with_options(
    server: *mut nros_action_server_t,
    node: *const nros_node_t,
    action_name: *const core::ffi::c_char,
    type_info: *const nros_action_type_t,
    goal_callback: nros_goal_callback_t,
    cancel_callback: nros_cancel_callback_t,
    accepted_callback: nros_accepted_callback_t,
    context: *mut c_void,
    qos: *const crate::qos::nros_qos_t,
    options: *const nros_action_server_options_t,
) -> nros_ret_t {
    let ret = nros_action_server_init_with_qos(
        server,
        node,
        action_name,
        type_info,
        goal_callback,
        cancel_callback,
        accepted_callback,
        context,
        qos,
    );
    if ret != NROS_RET_OK {
        return ret;
    }
    if !options.is_null() {
        (*server).sched_context_id = (*options).sched_context;
    }
    NROS_RET_OK
}

/// Publish feedback for an active goal.
///
/// Delegates to `ActionServerRawHandle::publish_feedback_raw`; the arena
/// enforces goal liveness and is the sole source of truth for lifecycle
/// state.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_publish_feedback(
    server: *mut nros_action_server_t,
    goal: *const nros_goal_handle_t,
    feedback: *const u8,
    feedback_len: usize,
) -> nros_ret_t {
    validate_not_null!(server, goal, feedback);

    let internal = get_internal(server);
    if !internal.is_handle_set() {
        return NROS_RET_NOT_INIT;
    }
    let handle = internal.handle;

    let executor = crate::executor::get_executor_from_ptr(internal.executor_ptr);
    let goal_id = nros_node::GoalId {
        uuid: (*goal).uuid.uuid,
    };
    let data = core::slice::from_raw_parts(feedback, feedback_len);

    // C serialize produces [CDR_HEADER][fields], but publish_feedback_raw
    // expects raw fields only (it adds its own CDR header + GoalId framing).
    let fields = strip_cdr_header(data);

    match handle.publish_feedback_raw(executor, &goal_id, fields) {
        Ok(()) => NROS_RET_OK,
        Err(_) => NROS_RET_ERROR,
    }
}

/// Mark a goal as succeeded with a result.
///
/// Delegates to `ActionServerRawHandle::complete_goal_raw`; the arena
/// owns the active-goals vector and retires the goal there.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_succeed(
    server: *mut nros_action_server_t,
    goal: *const nros_goal_handle_t,
    result: *const u8,
    result_len: usize,
) -> nros_ret_t {
    validate_not_null!(server, goal);

    let internal = get_internal(server);
    if !internal.is_handle_set() {
        return NROS_RET_NOT_INIT;
    }
    let handle = internal.handle;

    let executor = crate::executor::get_executor_from_ptr(internal.executor_ptr);
    let goal_id = nros_node::GoalId {
        uuid: (*goal).uuid.uuid,
    };
    let result_data = if !result.is_null() {
        core::slice::from_raw_parts(result, result_len)
    } else {
        &[]
    };

    // C serialize produces [CDR_HEADER][fields], but complete_goal_raw
    // expects raw fields only (it stores them in the slab and adds its own
    // CDR header when replying to get_result requests).
    let result_fields = strip_cdr_header(result_data);

    // issue 0796 — a result the server cannot retain (larger than the action
    // server's RESULT_BUF) is now reported instead of silently dropped.
    match handle.complete_goal_raw(
        executor,
        &goal_id,
        nros_node::GoalStatus::Succeeded,
        result_fields,
    ) {
        Ok(()) => NROS_RET_OK,
        Err(_) => NROS_RET_ERROR,
    }
}

/// Mark a goal as aborted with an optional result.
///
/// Delegates to `ActionServerRawHandle::complete_goal_raw`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_abort(
    server: *mut nros_action_server_t,
    goal: *const nros_goal_handle_t,
    result: *const u8,
    result_len: usize,
) -> nros_ret_t {
    validate_not_null!(server, goal);

    let internal = get_internal(server);
    if !internal.is_handle_set() {
        return NROS_RET_NOT_INIT;
    }
    let handle = internal.handle;

    let executor = crate::executor::get_executor_from_ptr(internal.executor_ptr);
    let goal_id = nros_node::GoalId {
        uuid: (*goal).uuid.uuid,
    };
    let result_data = if !result.is_null() {
        core::slice::from_raw_parts(result, result_len)
    } else {
        &[]
    };

    // C serialize produces [CDR_HEADER][fields] — strip the header.
    let result_fields = strip_cdr_header(result_data);

    // issue 0796 — a result the server cannot retain (larger than the action
    // server's RESULT_BUF) is now reported instead of silently dropped.
    match handle.complete_goal_raw(
        executor,
        &goal_id,
        nros_node::GoalStatus::Aborted,
        result_fields,
    ) {
        Ok(()) => NROS_RET_OK,
        Err(_) => NROS_RET_ERROR,
    }
}

/// Mark a goal as canceled with an optional result.
///
/// Delegates to `ActionServerRawHandle::complete_goal_raw`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_canceled(
    server: *mut nros_action_server_t,
    goal: *const nros_goal_handle_t,
    result: *const u8,
    result_len: usize,
) -> nros_ret_t {
    validate_not_null!(server, goal);

    let internal = get_internal(server);
    if !internal.is_handle_set() {
        return NROS_RET_NOT_INIT;
    }
    let handle = internal.handle;

    let executor = crate::executor::get_executor_from_ptr(internal.executor_ptr);
    let goal_id = nros_node::GoalId {
        uuid: (*goal).uuid.uuid,
    };
    let result_data = if !result.is_null() {
        core::slice::from_raw_parts(result, result_len)
    } else {
        &[]
    };

    // C serialize produces [CDR_HEADER][fields] — strip the header.
    let result_fields = strip_cdr_header(result_data);

    // issue 0796 — a result the server cannot retain (larger than the action
    // server's RESULT_BUF) is now reported instead of silently dropped.
    match handle.complete_goal_raw(
        executor,
        &goal_id,
        nros_node::GoalStatus::Canceled,
        result_fields,
    ) {
        Ok(()) => NROS_RET_OK,
        Err(_) => NROS_RET_ERROR,
    }
}

/// Execute a goal (transition to `Executing`).
///
/// Idempotent: delegates to `ActionServerRawHandle::set_goal_status` which
/// is a no-op if the goal isn't in the active-goals vector. Returns
/// `NROS_RET_OK` on success, `NROS_RET_NOT_INIT` if the server isn't
/// registered.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_execute(
    server: *mut nros_action_server_t,
    goal: *const nros_goal_handle_t,
) -> nros_ret_t {
    validate_not_null!(server, goal);

    let internal = get_internal(server);
    if !internal.is_handle_set() {
        return NROS_RET_NOT_INIT;
    }
    let handle = internal.handle;

    let executor = crate::executor::get_executor_from_ptr(internal.executor_ptr);
    let goal_id = nros_node::GoalId {
        uuid: (*goal).uuid.uuid,
    };
    handle.set_goal_status(executor, &goal_id, nros_node::GoalStatus::Executing);
    NROS_RET_OK
}

/// Get the number of currently active goals.
///
/// Phase-379 W6 decision 3: ONE function in rcl's shape, serving both
/// tiers. Upstream ships no count function; its nearest neighbour is
/// `rcl_action_server_get_goal_handles(server, ***handles, size_t
/// *num_goals) -> rcl_ret_t`, which reports the count through an
/// out-param and keeps the return code for "did the query work". This
/// replaces the pair `nros_action_server_get_active_goal_count(server)
/// -> size_t` (answered 0 on every error) and
/// `nros_action_server_active_goal_count_raw(server) -> int32_t`
/// (negative = error code), each of which collapsed the two channels
/// into one value.
///
/// The tier is selected by `server->state`, and the two tiers read
/// DIFFERENT STORAGE — this was never one field behind two names:
///
/// * `NROS_ACTION_SERVER_STATE_INITIALIZED` (L2, callback tier) reads
///   the executor arena through `ActionServerRawHandle::active_goal_count`.
/// * `NROS_ACTION_SERVER_STATE_POLLING` (L1, polling tier) reads the
///   `ActionServerCore` living inline in `_opaque`, with no executor and
///   no arena indirection, behind `#[cfg(feature = "rmw-cffi")]`.
///
/// Returns:
///
/// * `NROS_RET_OK` — `*out` holds the count, which may legitimately be 0.
/// * `NROS_RET_INVALID_ARGUMENT` — `server` or `out` is NULL.
/// * `NROS_RET_NOT_INIT` — the server is UNINITIALIZED or SHUTDOWN; or it
///   is INITIALIZED but was never registered with an executor (or has
///   been finalised); or it is POLLING in a build without `rmw-cffi`,
///   where the inline core was never constructed.
///
/// `*out` is left untouched on every error path, so a caller that ignores
/// the return code reads back its own initialiser rather than a
/// fabricated `0` — which is precisely the ambiguity the old `size_t`
/// spelling could not escape.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_get_active_goal_count(
    server: *mut nros_action_server_t,
    out: *mut usize,
) -> nros_ret_t {
    validate_not_null!(server, out);

    match (*server).state {
        nros_action_server_state_t::NROS_ACTION_SERVER_STATE_INITIALIZED => {
            let internal = get_internal(server);
            if !internal.is_handle_set() {
                return NROS_RET_NOT_INIT;
            }
            let handle = internal.handle;
            let executor = crate::executor::get_executor_from_ptr(internal.executor_ptr);
            *out = handle.active_goal_count(executor);
            NROS_RET_OK
        }
        nros_action_server_state_t::NROS_ACTION_SERVER_STATE_POLLING => {
            #[cfg(feature = "rmw-cffi")]
            {
                match polling_server_core(server) {
                    Some(core) => {
                        *out = core.active_goal_count();
                        NROS_RET_OK
                    }
                    // Unreachable: `polling_server_core` rejects only NULL
                    // (excluded above) and a non-POLLING state (this arm).
                    // Mapped rather than `unreachable!()` so a future
                    // rejection reason cannot turn into a panic across the
                    // FFI boundary.
                    None => NROS_RET_NOT_INIT,
                }
            }
            // `nros_action_server_init_polling` sets `state = POLLING`
            // outside the `rmw-cffi` gate, so this arm is reachable: the
            // state says polling and `_opaque` holds no core.
            #[cfg(not(feature = "rmw-cffi"))]
            {
                NROS_RET_NOT_INIT
            }
        }
        _ => NROS_RET_NOT_INIT,
    }
}

/// Look up a goal's current status in the arena by UUID.
///
/// Returns `NROS_RET_OK` and writes the arena-sourced status on success.
/// Returns `NROS_RET_NOT_FOUND` if the arena has already retired the goal
/// (completed + result delivered, or cancelled + acknowledged).
///
/// This is the authoritative successor to the removed `goal->status`
/// field: lifecycle transitions are driven by
/// `ActionServerArenaEntry::active_goals` and read back through this
/// function.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_get_goal_status(
    server: *const nros_action_server_t,
    goal: *const nros_goal_handle_t,
    status: *mut nros_goal_status_t,
) -> nros_ret_t {
    validate_not_null!(server, goal, status);

    let internal = get_internal(server);
    if !internal.is_handle_set() {
        return NROS_RET_NOT_INIT;
    }
    let handle = internal.handle;
    let executor = crate::executor::get_executor_from_ptr(internal.executor_ptr);
    let goal_id = nros_node::GoalId {
        uuid: (*goal).uuid.uuid,
    };
    match handle.goal_status(executor, &goal_id) {
        Some(s) => {
            *status = goal_status_from_core(s);
            NROS_RET_OK
        }
        None => NROS_RET_NOT_FOUND,
    }
}

/// Map a `nros_node::GoalStatus` to its `nros_goal_status_t` equivalent.
fn goal_status_from_core(status: nros_node::GoalStatus) -> nros_goal_status_t {
    use nros_node::GoalStatus;
    match status {
        GoalStatus::Unknown => nros_goal_status_t::NROS_GOAL_STATUS_UNKNOWN,
        GoalStatus::Accepted => nros_goal_status_t::NROS_GOAL_STATUS_ACCEPTED,
        GoalStatus::Executing => nros_goal_status_t::NROS_GOAL_STATUS_EXECUTING,
        GoalStatus::Canceling => nros_goal_status_t::NROS_GOAL_STATUS_CANCELING,
        GoalStatus::Succeeded => nros_goal_status_t::NROS_GOAL_STATUS_SUCCEEDED,
        GoalStatus::Canceled => nros_goal_status_t::NROS_GOAL_STATUS_CANCELED,
        GoalStatus::Aborted => nros_goal_status_t::NROS_GOAL_STATUS_ABORTED,
    }
}

/// Phase 122.3.c.6.b — reverse of `goal_status_from_core`.
fn c_status_to_rust(status: nros_goal_status_t) -> nros_node::GoalStatus {
    use nros_node::GoalStatus;
    match status {
        nros_goal_status_t::NROS_GOAL_STATUS_UNKNOWN => GoalStatus::Unknown,
        nros_goal_status_t::NROS_GOAL_STATUS_ACCEPTED => GoalStatus::Accepted,
        nros_goal_status_t::NROS_GOAL_STATUS_EXECUTING => GoalStatus::Executing,
        nros_goal_status_t::NROS_GOAL_STATUS_CANCELING => GoalStatus::Canceling,
        nros_goal_status_t::NROS_GOAL_STATUS_SUCCEEDED => GoalStatus::Succeeded,
        nros_goal_status_t::NROS_GOAL_STATUS_CANCELED => GoalStatus::Canceled,
        nros_goal_status_t::NROS_GOAL_STATUS_ABORTED => GoalStatus::Aborted,
    }
}

/// Finalize an action server.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_fini(server: *mut nros_action_server_t) -> nros_ret_t {
    validate_not_null!(server);

    let server = &mut *server;

    match server.state {
        nros_action_server_state_t::NROS_ACTION_SERVER_STATE_INITIALIZED => {
            // L2: action server lives in executor arena (if registered) —
            // reset metadata only.
        }
        nros_action_server_state_t::NROS_ACTION_SERVER_STATE_POLLING => {
            // L1: drop the inline ActionServerCore so its 5 channel
            // handles' Drops run.
            #[cfg(feature = "rmw-cffi")]
            {
                core::ptr::drop_in_place(server._opaque.as_mut_ptr()
                    as *mut nros_node::ActionServerCore<
                        { crate::config::MESSAGE_BUFFER_SIZE },
                        { crate::config::MESSAGE_BUFFER_SIZE },
                        { crate::config::MESSAGE_BUFFER_SIZE },
                        4,
                    >);
                server._opaque = [0u64; crate::opaque_sizes::ACTION_SERVER_OPAQUE_U64S];
            }
        }
        _ => return NROS_RET_NOT_INIT,
    }

    server._internal = ActionServerInternal::invalid_default();
    server.goal_callback = None;
    server.cancel_callback = None;
    server.accepted_callback = None;
    server.context = ptr::null_mut();
    server.node = crate::node::nros_node_ref_t::none();
    server.state = nros_action_server_state_t::NROS_ACTION_SERVER_STATE_SHUTDOWN;

    NROS_RET_OK
}

// ============================================================================
// Phase 122.3.c.6.b — Layer-1 primitive entry points (caller polls)
// ============================================================================

/// Phase 122.3.c.6.b — initialize an L1 polling-mode action server.
///
/// Creates the 5 transport channels immediately and stores the
/// `ActionServerCore` inline in `_opaque`. The caller drives the
/// goal lifecycle via:
/// * `nros_action_server_try_recv_goal_request_raw` — poll for new
///   goal requests (returns goal_id + sequence + payload).
/// * `nros_action_server_accept_goal_raw` / `_reject_goal_raw` —
///   reply to the send_goal RPC.
/// * `nros_action_server_publish_feedback_raw` — push feedback.
/// * `nros_action_server_complete_goal_raw` — terminate (SUCCEEDED /
///   ABORTED / CANCELED).
/// * `nros_action_server_try_handle_cancel_raw` /
///   `_try_handle_get_result_raw` — service cancel / get_result RPCs.
///
/// # Safety
/// All pointers must be valid; `action_name` must be a valid
/// null-terminated string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_init_polling(
    server: *mut nros_action_server_t,
    node: *const nros_node_t,
    type_info: *const super::common::nros_action_type_t,
    action_name: *const core::ffi::c_char,
) -> nros_ret_t {
    validate_not_null!(server, node, type_info, action_name);

    let server_mut = &mut *server;
    let node_ref = &*node;
    let type_info_ref = &*type_info;

    validate_state!(
        server_mut,
        nros_action_server_state_t::NROS_ACTION_SERVER_STATE_UNINITIALIZED,
        NROS_RET_BAD_SEQUENCE
    );
    validate_state!(node_ref, nros_node_state_t::NROS_NODE_STATE_INITIALIZED);

    server_mut.action_name_len =
        crate::util::copy_cstr_into(action_name, &mut server_mut.action_name);
    if server_mut.action_name_len == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }
    server_mut.type_name_len =
        crate::util::copy_cstr_into(type_info_ref.type_name, &mut server_mut.type_name);
    server_mut.type_hash_len =
        crate::util::copy_cstr_into(type_info_ref.type_hash, &mut server_mut.type_hash);

    server_mut.node = unsafe { crate::node::node_ref_of(node) };

    #[cfg(feature = "rmw-cffi")]
    {
        // Phase 156 Sub-bug D — multi-Session dispatch (see
        // `rclc_publisher_init_default`).
        let (session, domain_id) = match crate::node::resolve_session_and_domain(node_ref) {
            Some(t) => t,
            None => return NROS_RET_NOT_INIT,
        };

        let action_str =
            core::str::from_utf8_unchecked(&server_mut.action_name[..server_mut.action_name_len]);
        let type_str =
            core::str::from_utf8_unchecked(&server_mut.type_name[..server_mut.type_name_len]);
        let type_hash_str =
            core::str::from_utf8_unchecked(&server_mut.type_hash[..server_mut.type_hash_len]);
        let node_name_str = core::str::from_utf8_unchecked(&node_ref.name[..node_ref.name_len]);
        let namespace_str =
            core::str::from_utf8_unchecked(&node_ref.namespace[..node_ref.namespace_len]);

        // Inline the 5-channel build (mirror of
        // `Node::create_action_server_raw_sized` but operating on
        // a raw `&mut Session` to avoid creating a temporary
        // `Node`).
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
        let send_goal_server =
            match session.create_service(&send_goal_info, QoSProfile::services_default()) {
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
        let cancel_goal_server =
            match session.create_service(&cancel_goal_info, QoSProfile::services_default()) {
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
        let get_result_server =
            match session.create_service(&get_result_info, QoSProfile::services_default()) {
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
        let feedback_publisher =
            match session.create_publisher(&feedback_topic, QoSProfile::BEST_EFFORT) {
                Ok(h) => h,
                Err(_) => return NROS_RET_ERROR,
            };

        let status_keyexpr: nros_core::heapless::String<256> = action_info.status_key();
        let status_topic = TopicInfo::new(
            &status_keyexpr,
            "action_msgs::msg::dds_::GoalStatusArray_",
            type_hash_str,
        )
        .with_domain(domain_id)
        .with_node_name(node_name_str)
        .with_namespace(namespace_str);
        let status_publisher =
            match session.create_publisher(&status_topic, QoSProfile::BEST_EFFORT) {
                Ok(h) => h,
                Err(_) => return NROS_RET_ERROR,
            };

        type Core = nros_node::ActionServerCore<
            { crate::config::MESSAGE_BUFFER_SIZE },
            { crate::config::MESSAGE_BUFFER_SIZE },
            { crate::config::MESSAGE_BUFFER_SIZE },
            4,
        >;
        let core = Core::from_channels(
            send_goal_server,
            cancel_goal_server,
            get_result_server,
            feedback_publisher,
            status_publisher,
        );
        core::ptr::write(server_mut._opaque.as_mut_ptr() as *mut Core, core);
    }

    server_mut.state = nros_action_server_state_t::NROS_ACTION_SERVER_STATE_POLLING;
    NROS_RET_OK
}

#[cfg(feature = "rmw-cffi")]
type PollingServerCore = nros_node::ActionServerCore<
    { crate::config::MESSAGE_BUFFER_SIZE },
    { crate::config::MESSAGE_BUFFER_SIZE },
    { crate::config::MESSAGE_BUFFER_SIZE },
    4,
>;

#[cfg(feature = "rmw-cffi")]
#[inline]
unsafe fn polling_server_core(
    server: *mut nros_action_server_t,
) -> Option<&'static mut PollingServerCore> {
    if server.is_null() {
        return None;
    }
    let server_mut = &mut *server;
    if server_mut.state != nros_action_server_state_t::NROS_ACTION_SERVER_STATE_POLLING {
        return None;
    }
    Some(&mut *(server_mut._opaque.as_mut_ptr() as *mut PollingServerCore))
}

/// Phase 122.3.c.6.b — L1 polling: try to receive a goal request.
///
/// On success writes the goal payload bytes (already stripped of CDR
/// framing) into `buf`, returns the number of bytes copied (>= 0),
/// and fills `goal_id_out` (16 bytes) + `sequence_number_out`. Use
/// the sequence number with `nros_action_server_accept_goal_raw` /
/// `_reject_goal_raw`.
///
/// Returns `0` when no request is pending; negative `nros_ret_t` on
/// error.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_try_recv_goal_request_raw(
    server: *mut nros_action_server_t,
    buf: *mut u8,
    buf_len: usize,
    goal_id_out: *mut [u8; 16],
    sequence_number_out: *mut i64,
) -> i32 {
    if server.is_null()
        || (buf.is_null() && buf_len != 0)
        || goal_id_out.is_null()
        || sequence_number_out.is_null()
    {
        return NROS_RET_INVALID_ARGUMENT;
    }

    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_server_core(server) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        match core.try_recv_goal_request() {
            Ok(Some(req)) => {
                // Goal payload sits in core.goal_buffer() at
                // [GOAL_REQUEST_FRAMING_LEN..GOAL_REQUEST_FRAMING_LEN+req.data_len].
                let goal_buf = core.goal_buffer();
                let payload_offset = GOAL_REQUEST_FRAMING_LEN;
                if payload_offset + req.data_len > goal_buf.len() {
                    return NROS_RET_ERROR;
                }
                let copy_len = req.data_len.min(buf_len);
                core::ptr::copy_nonoverlapping(
                    goal_buf.as_ptr().add(payload_offset),
                    buf,
                    copy_len,
                );
                (*goal_id_out).copy_from_slice(&req.goal_id.uuid);
                *sequence_number_out = req.sequence_number;
                copy_len as i32
            }
            Ok(None) => 0,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (buf, buf_len, goal_id_out, sequence_number_out);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.b — L1 polling: accept a goal received via
/// `try_recv_goal_request_raw`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_accept_goal_raw(
    server: *mut nros_action_server_t,
    goal_id: *const [u8; 16],
    sequence_number: i64,
) -> nros_ret_t {
    if server.is_null() || goal_id.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_server_core(server) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        let id = nros::GoalId { uuid: *goal_id };
        core.accept_goal(id, sequence_number)
            .map(|_| NROS_RET_OK)
            .unwrap_or(NROS_RET_ERROR)
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (goal_id, sequence_number);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.b — L1 polling: reject a goal received via
/// `try_recv_goal_request_raw`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_reject_goal_raw(
    server: *mut nros_action_server_t,
    sequence_number: i64,
) -> nros_ret_t {
    if server.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_server_core(server) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        core.reject_goal(sequence_number)
            .map(|_| NROS_RET_OK)
            .unwrap_or(NROS_RET_ERROR)
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = sequence_number;
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.b — L1 polling: publish a feedback message.
///
/// `feedback_cdr` is the CDR-encoded `<Action>_Feedback_` payload
/// (without the goal_id prefix — the core wraps it).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_publish_feedback_raw(
    server: *mut nros_action_server_t,
    goal_id: *const [u8; 16],
    feedback_cdr: *const u8,
    feedback_len: usize,
) -> nros_ret_t {
    if server.is_null() || goal_id.is_null() || (feedback_cdr.is_null() && feedback_len != 0) {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_server_core(server) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        let id = nros::GoalId { uuid: *goal_id };
        let slice = core::slice::from_raw_parts(feedback_cdr, feedback_len);
        core.publish_feedback_raw(&id, slice)
            .map(|_| NROS_RET_OK)
            .unwrap_or(NROS_RET_ERROR)
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (goal_id, feedback_cdr, feedback_len);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.b — L1 polling: mark a goal terminal.
///
/// `status` must be one of SUCCEEDED / ABORTED / CANCELED.
/// `result_cdr` is the CDR-encoded `<Action>_Result_` payload (the
/// core handles the status wrapper before storing in the slab).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_complete_goal_raw(
    server: *mut nros_action_server_t,
    goal_id: *const [u8; 16],
    status: nros_goal_status_t,
    result_cdr: *const u8,
    result_len: usize,
) -> nros_ret_t {
    if server.is_null() || goal_id.is_null() || (result_cdr.is_null() && result_len != 0) {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_server_core(server) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        let id = nros::GoalId { uuid: *goal_id };
        let rust_status = c_status_to_rust(status);
        let slice = core::slice::from_raw_parts(result_cdr, result_len);
        // issue 0796 — see `nros_action_succeed`.
        match core.complete_goal_raw(&id, rust_status, slice) {
            Ok(()) => NROS_RET_OK,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (goal_id, status, result_cdr, result_len);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.d — L1 polling: peek a pending cancel-goal
/// request. Writes the named goal_id, the matching service
/// sequence number, and the goal's current status (matches the
/// `nros_goal_status_t` discriminants — pass it back unchanged).
///
/// Returns `1` when a request was peeked, `0` when none pending,
/// negative `nros_ret_t` on error.
///
/// After a successful peek, call
/// `nros_action_server_send_cancel_reply_raw` with the recorded
/// `sequence_number` and the list of goals to accept-cancel.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_try_recv_cancel_request_raw(
    server: *mut nros_action_server_t,
    goal_id_out: *mut [u8; 16],
    sequence_number_out: *mut i64,
    current_status_out: *mut nros_goal_status_t,
) -> i32 {
    if server.is_null()
        || goal_id_out.is_null()
        || sequence_number_out.is_null()
        || current_status_out.is_null()
    {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_server_core(server) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        match core.try_recv_cancel_request() {
            Ok(Some(req)) => {
                (*goal_id_out).copy_from_slice(&req.goal_id.uuid);
                *sequence_number_out = req.sequence_number;
                *current_status_out = goal_status_from_core(req.current_status);
                1
            }
            Ok(None) => 0,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (goal_id_out, sequence_number_out, current_status_out);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.d — overall cancel-RPC return code. Distinct from
/// the per-goal `nros_cancel_response_t` (ACCEPT/REJECT) used by the
/// L2 callback path. These four discriminants mirror
/// `nros_core::CancelReturnCode` and the `action_msgs/srv/CancelGoal`
/// wire-CDR `return_code` field. (Issue 0796 — the Rust type was called
/// `CancelResponse` until it was split; this pair of C names is what the
/// Rust names now follow.)
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_cancel_return_code_t {
    NROS_CANCEL_RC_OK = 0,
    NROS_CANCEL_RC_REJECTED = 1,
    NROS_CANCEL_RC_UNKNOWN_GOAL = 2,
    NROS_CANCEL_RC_GOAL_TERMINATED = 3,
}

/// Phase 122.3.c.6.d — L1 polling: reply to a cancel-goal request
/// previously peeked via `_try_recv_cancel_request_raw`. `accepted`
/// points to `accepted_count` goal-ID byte arrays that will
/// transition to `CANCELING`. Pass an empty list (`accepted=NULL,
/// accepted_count=0`) with
/// `return_code=NROS_CANCEL_RC_REJECTED` to refuse the request
/// entirely.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_send_cancel_reply_raw(
    server: *mut nros_action_server_t,
    sequence_number: i64,
    return_code: nros_cancel_return_code_t,
    accepted: *const [u8; 16],
    accepted_count: usize,
) -> nros_ret_t {
    if server.is_null() || (accepted.is_null() && accepted_count != 0) {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_server_core(server) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        let cancel_resp = match return_code {
            nros_cancel_return_code_t::NROS_CANCEL_RC_OK => nros_node::CancelReturnCode::Ok,
            nros_cancel_return_code_t::NROS_CANCEL_RC_REJECTED => {
                nros_node::CancelReturnCode::Rejected
            }
            nros_cancel_return_code_t::NROS_CANCEL_RC_UNKNOWN_GOAL => {
                nros_node::CancelReturnCode::UnknownGoal
            }
            nros_cancel_return_code_t::NROS_CANCEL_RC_GOAL_TERMINATED => {
                nros_node::CancelReturnCode::GoalTerminated
            }
        };
        // Build a stack-resident slice of GoalIds from the caller's
        // contiguous byte array. Cap at 8 — the wire format allows
        // more but no current backend handles more than a few
        // simultaneous cancels.
        let mut ids: nros_core::heapless::Vec<nros::GoalId, 8> = nros_core::heapless::Vec::new();
        for i in 0..accepted_count {
            if i >= 8 {
                return NROS_RET_BAD_SEQUENCE;
            }
            let uuid = *accepted.add(i);
            if ids.push(nros::GoalId { uuid }).is_err() {
                return NROS_RET_ERROR;
            }
        }
        match core.send_cancel_reply(sequence_number, cancel_resp, &ids) {
            Ok(()) => NROS_RET_OK,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (sequence_number, return_code, accepted, accepted_count);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.b — L1 polling: serve a pending get_result query.
///
/// `default_result_cdr` is the default serialized result (without
/// status byte / CDR header) returned to the client when no
/// `complete_goal_raw` has been called for the queried goal yet.
///
/// Returns `0` when no query is pending; `1` when one was served;
/// negative `nros_ret_t` on error.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_try_handle_get_result_raw(
    server: *mut nros_action_server_t,
    default_result_cdr: *const u8,
    default_result_len: usize,
) -> i32 {
    if server.is_null() || (default_result_cdr.is_null() && default_result_len != 0) {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_server_core(server) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        let slice = core::slice::from_raw_parts(default_result_cdr, default_result_len);
        match core.try_handle_get_result_raw(slice) {
            Ok(Some(_)) => 1,
            Ok(None) => 0,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (default_result_cdr, default_result_len);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.e — register a C wake callback on the
/// send_goal channel of an L1 polling-mode action server. `state`
/// is a caller-owned `nros_wake_state_t` that must outlive the
/// server and not move.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_set_goal_wake_callback(
    server: *mut nros_action_server_t,
    state: *mut crate::service::nros_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_ret_t {
    set_action_server_wake_callback(server, state, cb, ctx, ChannelKind::Goal)
}

/// Phase 122.3.c.6.e — register a C wake callback on the
/// cancel_goal channel of an L1 polling-mode action server. The
/// primary use case from the .c.6.e design discussion — RTOS /
/// event-driven cancel handling.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_set_cancel_wake_callback(
    server: *mut nros_action_server_t,
    state: *mut crate::service::nros_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_ret_t {
    set_action_server_wake_callback(server, state, cb, ctx, ChannelKind::Cancel)
}

/// Phase 122.3.c.6.e — register a C wake callback on the
/// get_result channel of an L1 polling-mode action server.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_action_server_set_get_result_wake_callback(
    server: *mut nros_action_server_t,
    state: *mut crate::service::nros_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_ret_t {
    set_action_server_wake_callback(server, state, cb, ctx, ChannelKind::GetResult)
}

enum ChannelKind {
    Goal,
    Cancel,
    GetResult,
}

unsafe fn set_action_server_wake_callback(
    server: *mut nros_action_server_t,
    state: *mut crate::service::nros_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
    kind: ChannelKind,
) -> nros_ret_t {
    if server.is_null() || state.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        let core = match polling_server_core(server) {
            Some(c) => c,
            None => return NROS_RET_INVALID_ARGUMENT,
        };
        let state_ptr = state as *mut nros_node::c_waker::CWakeState;
        core::ptr::write(
            state_ptr,
            nros_node::c_waker::CWakeState { fn_ptr: cb, ctx },
        );
        let waker = nros_node::c_waker::make_waker(state_ptr);
        match kind {
            ChannelKind::Goal => core.register_goal_waker(&waker),
            ChannelKind::Cancel => core.register_cancel_waker(&waker),
            ChannelKind::GetResult => core.register_get_result_waker(&waker),
        }
        NROS_RET_OK
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (state, cb, ctx, kind);
        NROS_RET_NOT_INIT
    }
}

// `nros_action_server_active_goal_count_raw` lived here until
// phase-379 W6 decision 3 merged it into
// `nros_action_server_get_active_goal_count`, which now branches on
// `state` to serve the polling tier too. The old spelling survived one
// release as a deprecated `static inline` forwarder in `nros/action.h` and was
// retired by phase-417 stage 6 step B; either way it exports no symbol from
// here.

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

    // Helper goal callback
    unsafe extern "C" fn dummy_goal_callback(
        _server: *mut nros_action_server_t,
        _goal: *const nros_goal_handle_t,
        _req: *const u8,
        _len: usize,
        _ctx: *mut core::ffi::c_void,
    ) -> nros_goal_response_t {
        nros_goal_response_t::NROS_GOAL_ACCEPT_AND_EXECUTE
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn action_server_init_null_ptrs() {
        let action_name = b"/fibonacci\0";
        let type_info = dummy_action_type();
        let node = crate::node::rcl_get_zero_initialized_node();

        // NULL server
        assert_eq!(
            unsafe {
                nros_action_server_init(
                    ptr::null_mut(),
                    &node,
                    action_name.as_ptr() as *const core::ffi::c_char,
                    &type_info,
                    Some(dummy_goal_callback),
                    None,
                    None,
                    ptr::null_mut(),
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL node
        let mut srv = rcl_action_get_zero_initialized_server();
        assert_eq!(
            unsafe {
                nros_action_server_init(
                    &mut srv,
                    ptr::null(),
                    action_name.as_ptr() as *const core::ffi::c_char,
                    &type_info,
                    Some(dummy_goal_callback),
                    None,
                    None,
                    ptr::null_mut(),
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL action_name
        let mut srv = rcl_action_get_zero_initialized_server();
        assert_eq!(
            unsafe {
                nros_action_server_init(
                    &mut srv,
                    &node,
                    ptr::null(),
                    &type_info,
                    Some(dummy_goal_callback),
                    None,
                    None,
                    ptr::null_mut(),
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL type_info
        let mut srv = rcl_action_get_zero_initialized_server();
        assert_eq!(
            unsafe {
                nros_action_server_init(
                    &mut srv,
                    &node,
                    action_name.as_ptr() as *const core::ffi::c_char,
                    ptr::null(),
                    Some(dummy_goal_callback),
                    None,
                    None,
                    ptr::null_mut(),
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn action_server_init_none_goal_callback() {
        let action_name = b"/fibonacci\0";
        let type_info = dummy_action_type();
        let node = crate::node::rcl_get_zero_initialized_node();

        let mut srv = rcl_action_get_zero_initialized_server();
        assert_eq!(
            unsafe {
                nros_action_server_init(
                    &mut srv,
                    &node,
                    action_name.as_ptr() as *const core::ffi::c_char,
                    &type_info,
                    None, // goal_callback is required
                    None,
                    None,
                    ptr::null_mut(),
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn action_server_init_uninit_node() {
        let action_name = b"/fibonacci\0";
        let type_info = dummy_action_type();
        let node = crate::node::rcl_get_zero_initialized_node();

        let mut srv = rcl_action_get_zero_initialized_server();
        assert_eq!(
            unsafe {
                nros_action_server_init(
                    &mut srv,
                    &node,
                    action_name.as_ptr() as *const core::ffi::c_char,
                    &type_info,
                    Some(dummy_goal_callback),
                    None,
                    None,
                    ptr::null_mut(),
                )
            },
            NROS_RET_NOT_INIT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn action_server_zero_initialized_state() {
        let srv = rcl_action_get_zero_initialized_server();
        assert_eq!(
            srv.state,
            nros_action_server_state_t::NROS_ACTION_SERVER_STATE_UNINITIALIZED,
        );
        assert!(srv._internal.handle.is_invalid());
        assert!(srv._internal.executor_ptr.is_null());
        assert!(srv.node.is_null());
        assert!(srv.goal_callback.is_none());
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn action_server_fini_null_safety() {
        // NULL → INVALID_ARGUMENT
        assert_eq!(
            unsafe { nros_action_server_fini(ptr::null_mut()) },
            NROS_RET_INVALID_ARGUMENT,
        );

        // UNINITIALIZED → NOT_INIT
        let mut srv = rcl_action_get_zero_initialized_server();
        assert_eq!(
            unsafe { nros_action_server_fini(&mut srv) },
            NROS_RET_NOT_INIT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn action_server_double_init_rejected() {
        let action_name = b"/fibonacci\0";
        let type_info = dummy_action_type();
        let mut node = crate::node::rcl_get_zero_initialized_node();
        node.state = crate::node::nros_node_state_t::NROS_NODE_STATE_INITIALIZED;

        let mut srv = rcl_action_get_zero_initialized_server();
        let ret = unsafe {
            nros_action_server_init(
                &mut srv,
                &node,
                action_name.as_ptr() as *const core::ffi::c_char,
                &type_info,
                Some(dummy_goal_callback),
                None,
                None,
                ptr::null_mut(),
            )
        };
        assert_eq!(ret, NROS_RET_OK);

        // Second init → BAD_SEQUENCE
        assert_eq!(
            unsafe {
                nros_action_server_init(
                    &mut srv,
                    &node,
                    action_name.as_ptr() as *const core::ffi::c_char,
                    &type_info,
                    Some(dummy_goal_callback),
                    None,
                    None,
                    ptr::null_mut(),
                )
            },
            NROS_RET_BAD_SEQUENCE,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn action_server_active_goal_count_null() {
        let mut count: usize = 7;

        // NULL server.
        assert_eq!(
            unsafe { nros_action_server_get_active_goal_count(ptr::null_mut(), &mut count) },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL out-param.
        let mut srv = rcl_action_get_zero_initialized_server();
        assert_eq!(
            unsafe { nros_action_server_get_active_goal_count(&mut srv, ptr::null_mut()) },
            NROS_RET_INVALID_ARGUMENT,
        );

        // Zero-initialised server is UNINITIALIZED: neither tier claims it.
        assert_eq!(
            unsafe { nros_action_server_get_active_goal_count(&mut srv, &mut count) },
            NROS_RET_NOT_INIT,
        );

        // `*out` is untouched on every error path.
        assert_eq!(count, 7);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn action_publish_feedback_null_ptrs() {
        let feedback = [0u8; 8];
        let goal = nros_goal_handle_t::default();

        // NULL server
        assert_eq!(
            unsafe { nros_action_publish_feedback(ptr::null_mut(), &goal, feedback.as_ptr(), 8) },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL goal
        let mut srv = rcl_action_get_zero_initialized_server();
        assert_eq!(
            unsafe { nros_action_publish_feedback(&mut srv, ptr::null(), feedback.as_ptr(), 8) },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL feedback
        assert_eq!(
            unsafe { nros_action_publish_feedback(&mut srv, &goal, ptr::null(), 0) },
            NROS_RET_INVALID_ARGUMENT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn goal_succeed_null_ptr() {
        assert_eq!(
            unsafe { nros_action_succeed(ptr::null_mut(), ptr::null(), ptr::null(), 0) },
            NROS_RET_INVALID_ARGUMENT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn goal_abort_null_ptr() {
        assert_eq!(
            unsafe { nros_action_abort(ptr::null_mut(), ptr::null(), ptr::null(), 0) },
            NROS_RET_INVALID_ARGUMENT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn goal_canceled_null_ptr() {
        assert_eq!(
            unsafe { nros_action_canceled(ptr::null_mut(), ptr::null(), ptr::null(), 0) },
            NROS_RET_INVALID_ARGUMENT,
        );
    }
}
