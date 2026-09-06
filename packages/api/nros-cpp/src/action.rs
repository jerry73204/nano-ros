//! Action server and client FFI functions for the C++ API.
//!
//! Alloc-free: all internal state is written into caller-provided inline storage.
//!
//! Issue 0436 — user-supplied executor handles are tag-validated via
//! `cpp_ctx_checked`. Handles reached through an already-validated node/client
//! (`node_ref.executor`, `client.executor_ptr`) inherit that provenance and stay
//! direct casts.

use core::ffi::{c_char, c_void};

use nros::{
    GoalId,
    cdr::{CDR_HEADER_LEN, strip_cdr_header},
};
use nros_node::config::DEFAULT_RX_BUF_SIZE;

use crate::{
    CppContext, DispatchGuard, NROS_CPP_RET_ERROR, NROS_CPP_RET_INVALID_ARGUMENT, NROS_CPP_RET_OK,
    NROS_CPP_RET_REENTRANT, NROS_CPP_RET_REJECTED, NROS_CPP_RET_TIMEOUT,
    NROS_CPP_RET_TRANSPORT_ERROR, NROS_CPP_RET_TRY_AGAIN, cpp_ctx_checked, cstr_to_str,
    nros_cpp_node_t, nros_cpp_qos_t, nros_cpp_ret_t,
};

/// Scratch buffer for re-framing an incoming goal payload before handing
/// it to the user goal callback: strip the action framing
/// (`[CDR_HDR][seq_prefix][UUID]`) and re-prepend a plain CDR header so the
/// callback sees a normal `[CDR_HDR][goal_fields]` message.
const GOAL_USER_BUF: usize = 512;

// Phase 87.11: opaque storage sizes are now driven by layout-mirror
// structs in `nros::sizes` (see `CppActionServerLayout` /
// `CppActionClientLayout`). The asserts below ensure the real
// `CppActionServer` / `CppActionClient` layouts stay byte-equivalent
// to the mirrors — any field-shape change here must be paired with an
// update in `packages/api/nros/src/sizes.rs`.

// ============================================================================
// Action Server
// ============================================================================

/// Goal callback type invoked by the arena goal trampoline.
///
/// Receives raw CDR goal bytes — the C++ header layer deserializes them
/// into the typed `A::Goal` before forwarding to the user's callback.
/// Returns `1` for `AcceptAndExecute`, `2` for `AcceptAndDefer`, `0` for `Reject`.
pub type CppGoalCallback = unsafe extern "C" fn(
    goal_id: *const [u8; 16],
    data: *const u8,
    len: usize,
    ctx: *mut c_void,
) -> i32;

/// Cancel callback type invoked by the arena cancel trampoline.
///
/// Returns `1` for `Accept`, `0` for `Reject`.
pub type CppCancelCallback =
    unsafe extern "C" fn(goal_id: *const [u8; 16], ctx: *mut c_void) -> i32;

/// Accepted-goal callback type invoked by the arena's post-accept hook.
///
/// Issue 0796 — the C++ callback tier had no way to register one, so a user
/// who returned `AcceptAndDefer` from the goal callback was never told the
/// goal had been accepted: the exact point at which a deferred goal is
/// supposed to start executing. C takes `nros_accepted_callback_t` at
/// `nros_action_server_init` and Rust takes one in
/// `create_action_server_with_callbacks(goal, cancel, accepted)`; this is the
/// C++ spelling of the same hook.
///
/// Fires AFTER the accept reply has reached the client (see
/// [`nros_node::RawAcceptedCallback`]), for both `AcceptAndExecute` and
/// `AcceptAndDefer`, so a long-running body here does not delay the accept.
pub type CppAcceptedCallback = unsafe extern "C" fn(goal_id: *const [u8; 16], ctx: *mut c_void);

/// Internal state for the action server.
///
/// Holds the arena handle and the user-registered callbacks. Phase 87.6
/// thin-wrapper refactor: the `action_name` / `type_name` / `type_hash`
/// buffers moved to the C++ `nros::ActionServer<A>` class (passed to
/// `nros_cpp_action_server_register` at registration time). No C++-side
/// goal queue — the arena in `nros-node` owns all lifecycle state.
///
/// `#[repr(C)]` is load-bearing: the layout-mirror assert below compares this
/// against the `#[repr(C)]` `nros::sizes::CppActionServerLayout`. Without it
/// the compiler may reorder/repack fields — the sizes coincide on 64-bit hosts
/// but diverge on 32-bit targets (e.g. armv7a-nuttx-eabihf), tripping the
/// assert. repr(C) pins both to identical declaration-order layout everywhere.
#[repr(C)]
pub(crate) struct CppActionServer {
    handle: Option<nros_node::ActionServerRawHandle>,
    goal_cb: Option<CppGoalCallback>,
    cancel_cb: Option<CppCancelCallback>,
    /// issue 0796 — post-accept hook, `None` until
    /// `nros_cpp_action_server_set_accepted_callback` installs one.
    accepted_cb: Option<CppAcceptedCallback>,
    cb_ctx: *mut c_void,
    /// Phase 104.C.9.b — NodeId captured at create time, consumed by
    /// `nros_cpp_action_server_register` to pick the `_on(NodeId, ...)`
    /// multi-Node dispatch variant. Issue 0312 — BIASED BY ONE, mirroring
    /// `nros_cpp_node_t::node_id`: `0` = no node, `n` = `NodeId(n - 1)`.
    node_id: u8,
    _reserved: [u8; 7],
    /// Phase 193.4b — create-time QoS, applied to the three underlying
    /// service servers (send_goal / cancel_goal / get_result) at register
    /// time. Defaults to the services profile via `nros_cpp_qos_t`.
    qos: nros_cpp_qos_t,
}

// Layout-mirror equivalence (Phase 87.11): the real `CppActionServer`
// must have the same byte size as `nros::sizes::CppActionServerLayout`,
// which is what `NROS_CPP_ACTION_SERVER_SIZE` is sized to. Any field
// shape change here requires a paired update in
// `packages/api/nros/src/sizes.rs::CppActionServerLayout`.
const _: () = assert!(
    core::mem::size_of::<CppActionServer>()
        == core::mem::size_of::<nros::sizes::CppActionServerLayout>(),
    "CppActionServer size diverges from nros::sizes::CppActionServerLayout — \
     update the layout mirror to track any field-shape change"
);

/// Goal callback trampoline — forwards to the user's registered callback
/// (if any) and returns `Reject` otherwise.
///
/// The request bytes arrive as the full CDR payload
/// `[CDR_HDR][seq_prefix][UUID][fields]`; we strip the 24-byte framing so
/// the user callback receives just `[CDR_HDR][fields]` (re-prepended).
///
/// # Safety
/// `context` must point to a valid `CppActionServer`.
unsafe extern "C" fn goal_callback_trampoline(
    goal_id: *const nros::GoalId,
    goal_data: *const u8,
    goal_len: usize,
    context: *mut c_void,
) -> nros::GoalResponse {
    let server = unsafe { &*(context as *const CppActionServer) };
    let Some(cb) = server.goal_cb else {
        return nros::GoalResponse::Reject;
    };

    // Incoming framing: [CDR_HDR][seq_prefix(4)][UUID(16)][goal_fields].
    // User-facing framing: [CDR_HDR][goal_fields].
    let framing_len = CDR_HEADER_LEN + GoalId::SEQ_PREFIX_LEN + GoalId::UUID_LEN;
    let slice = unsafe { core::slice::from_raw_parts(goal_data, goal_len) };
    let mut user_buf = [0u8; GOAL_USER_BUF];
    let (ptr, len) = if goal_len > framing_len {
        let fields = &slice[framing_len..];
        user_buf[..CDR_HEADER_LEN].copy_from_slice(&nros::cdr::CDR_LE_HEADER);
        let copy_len = fields.len().min(user_buf.len() - CDR_HEADER_LEN);
        user_buf[CDR_HEADER_LEN..CDR_HEADER_LEN + copy_len].copy_from_slice(&fields[..copy_len]);
        (user_buf.as_ptr(), CDR_HEADER_LEN + copy_len)
    } else {
        (goal_data, goal_len)
    };

    let uuid_ptr = goal_id as *const [u8; 16];
    let resp = unsafe { cb(uuid_ptr, ptr, len, server.cb_ctx) };
    match resp {
        1 => nros::GoalResponse::AcceptAndExecute,
        2 => nros::GoalResponse::AcceptAndDefer,
        _ => nros::GoalResponse::Reject,
    }
}

/// Cancel callback trampoline — forwards to the user's registered
/// callback, defaulting to `Accept` when none is set.
///
/// # Safety
/// `context` must point to a valid `CppActionServer`.
unsafe extern "C" fn cancel_callback_trampoline(
    goal_id: *const nros::GoalId,
    _status: nros::GoalStatus,
    context: *mut c_void,
) -> nros::CancelResponse {
    let server = unsafe { &*(context as *const CppActionServer) };
    let Some(cb) = server.cancel_cb else {
        return nros::CancelResponse::Accept;
    };
    let uuid_ptr = goal_id as *const [u8; 16];
    // issue 0796 — `nros::CancelResponse` is now the PER-GOAL decision, with
    // the same 0/1 discriminants as C++'s `nros::CancelResponse` enum class,
    // so this is a straight translation rather than a hop through the
    // `CancelGoal` RPC return code.
    match unsafe { cb(uuid_ptr, server.cb_ctx) } {
        0 => nros::CancelResponse::Reject,
        _ => nros::CancelResponse::Accept,
    }
}

/// Post-accept trampoline — forwards to the user's registered accepted
/// callback, if any.
///
/// Registered unconditionally at `register_action_server_raw` time (the arena
/// captures the hook when the entry is built, before the user has had a chance
/// to call `set_accepted_callback`), so the `None` case is the normal one and
/// must be free.
///
/// # Safety
/// `context` must point to a valid `CppActionServer`.
unsafe extern "C" fn accepted_callback_trampoline(
    goal_id: *const nros::GoalId,
    context: *mut c_void,
) {
    let server = unsafe { &*(context as *const CppActionServer) };
    let Some(cb) = server.accepted_cb else {
        return;
    };
    unsafe { cb(goal_id as *const [u8; 16], server.cb_ctx) };
}

/// Create an action server on a node.
///
/// Phase 87.6: this call only zero-initialises the storage. Names and
/// executor registration happen in `nros_cpp_action_server_register`
/// below, same as before — the split exists to avoid a FreeRTOS QEMU
/// deadlock where eagerly declaring the five underlying entities blocks
/// the session mutex.
///
/// # Safety
/// `storage` must point to an 8-byte-aligned buffer of at least
/// `NROS_CPP_ACTION_SERVER_STORAGE_SIZE` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_create(
    node: *const nros_cpp_node_t,
    _action_name: *const c_char,
    _type_name: *const c_char,
    _type_hash: *const c_char,
    qos: nros_cpp_qos_t,
    storage: *mut c_void,
) -> nros_cpp_ret_t {
    if node.is_null() || storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let node_ref = unsafe { &*node };
    if node_ref.executor.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let server = CppActionServer {
        handle: None,
        goal_cb: None,
        cancel_cb: None,
        accepted_cb: None,
        cb_ctx: core::ptr::null_mut(),
        // Phase 104.C.9.b — capture the Node's id so register can
        // pick the multi-Node `_on(NodeId, ...)` dispatch variant.
        node_id: node_ref.node_id,
        _reserved: [0u8; 7],
        // Phase 193.4b — stash the create-time QoS for the three underlying
        // service servers (applied at register time).
        qos,
    };
    unsafe {
        core::ptr::write(storage as *mut CppActionServer, server);
    }
    NROS_CPP_RET_OK
}

/// Register an action server with the executor (creates transport handles).
///
/// Must be called after `nros_cpp_action_server_create`. Creates the
/// 3 queryables + 2 publishers in the executor context. Separated from
/// create to avoid deadlocks on FreeRTOS QEMU where declaring 5 entities
/// eagerly blocks the session mutex.
///
/// # Safety
/// `storage` must point to a valid `CppActionServer` from create.
/// `executor_handle` must point to a valid `CppContext`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_register(
    storage: *mut c_void,
    executor_handle: *mut c_void,
    action_name: *const c_char,
    type_name: *const c_char,
    type_hash: *const c_char,
    sched_context: u8,
) -> nros_cpp_ret_t {
    if storage.is_null() || action_name.is_null() || type_name.is_null() || type_hash.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let server = unsafe { &mut *(storage as *mut CppActionServer) };
    let Some(ctx) = (unsafe { cpp_ctx_checked(executor_handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };

    let act_str = match unsafe { cstr_to_str(action_name) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let type_str = match unsafe { cstr_to_str(type_name) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    // Issue 0312 — an empty hash would land in the liveliness keyexpr as an
    // empty segment, making the entity invisible to ROS 2 discovery.
    let hash_str = match unsafe { cstr_to_str(type_hash) } {
        Some(s) => crate::normalize_type_hash(s),
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };

    // Phase 305 W3 (issue 0255) — resolve `~`/relative names + launch remaps.
    // No node handle crosses this seam (only the executor), so the executor's
    // CURRENT identity scopes the lookup.
    let resolved_act = match ctx.executor.resolve_entity_name(act_str) {
        Ok(r) => r,
        Err(()) => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let act_str = resolved_act.as_str();

    // Phase 104.C.9.b — `spec.node_id` routes registration through the
    // named multi-RMW Node's session (so the underlying
    // queryables/publishers land there); `None` uses the default Node.
    // Phase 193.4b — `spec.qos` applies the create-time QoS to the three
    // underlying service servers (rclcpp `create_action_server(name, qos)`).
    let qos = server.qos.to_qos_settings();
    // Issue 0312 — `CppActionServer` mirrors the handle's biased id, so decode
    // it the same way (a raw `!= 0` here would drop a single-node entry's node).
    let node_id =
        (server.node_id != 0).then(|| nros_node::executor::NodeId::from_raw(server.node_id - 1));
    let result = ctx
        .executor
        .register_action_server_raw(nros_node::RawActionServerSpec {
            node_id,
            action_name: act_str,
            type_name: type_str,
            type_hash: hash_str,
            qos,
            goal_callback: goal_callback_trampoline,
            cancel_callback: cancel_callback_trampoline,
            // issue 0796 — always registered; the trampoline is a no-op until
            // the user installs a callback. The arena captures this pointer
            // when the entry is built, so it cannot be added later.
            accepted_callback: Some(accepted_callback_trampoline),
            context: storage,
        });
    match result {
        Ok(handle) => {
            // Phase 189.M3.3.c — bind the action's goal-service handle to the
            // requested sched context. The C++ action server is arena-registered
            // (unlike poll-style C++ services), so its handle is real + its
            // goal/cancel callbacks are executor-dispatched — the bind is
            // functional. `0` = inherit (no-op); an unknown slot fails.
            if sched_context != 0 {
                let sc_id = nros_node::executor::sched_context::SchedContextId(sched_context);
                if ctx
                    .executor
                    .bind_handle_to_sched_context(handle.handle_id(), sc_id)
                    .is_err()
                {
                    return NROS_CPP_RET_INVALID_ARGUMENT;
                }
            }
            server.handle = Some(handle);
            NROS_CPP_RET_OK
        }
        // issue 0557 / phase-358 W5 — was `Err(_) => TRANSPORT_ERROR`, which
        // discarded the `NodeError` and reported `-100` for causes that are not
        // transport at all. That is the collapse issue 0436 already fixed for
        // `nros_cpp_init` (via this same mapper, which also names the variant on
        // the error path) — applied there and not here.
        //
        // It cost exactly what that kind of collapse costs: the Zephyr Cyclone C
        // action image failed with `run_components failed rc=-100` and no way to
        // tell which precondition the backend rejected, while the Rust action,
        // the C service and the C pubsub cells on the same backend all passed.
        Err(e) => crate::node_error_to_cpp_ret(e),
    }
}

/// Register callbacks on the action server.
///
/// The goal callback receives raw CDR goal bytes and returns `1`
/// (AcceptAndExecute), `2` (AcceptAndDefer), or `0` (Reject). The cancel
/// callback returns `1` (Accept) or `0` (Reject). Either callback may be
/// null — a null goal callback causes every request to be rejected; a
/// null cancel callback causes every cancel to be accepted. The C++
/// template header translates typed callables into this raw-bytes form.
///
/// # Safety
/// `handle` must be a valid initialized action server storage.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_set_callbacks(
    handle: *mut c_void,
    goal_cb: Option<CppGoalCallback>,
    cancel_cb: Option<CppCancelCallback>,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    if handle.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let server = unsafe { &mut *(handle as *mut CppActionServer) };
    server.goal_cb = goal_cb;
    server.cancel_cb = cancel_cb;
    server.cb_ctx = ctx;
    NROS_CPP_RET_OK
}

/// Register the accepted-goal callback on the action server (issue 0796).
///
/// Separate from `nros_cpp_action_server_set_callbacks` so that adding the
/// third callback did not change an existing FFI signature; `ctx` is the SAME
/// shared context slot those two use, and passing a different pointer here
/// re-points all three. The C++ `ActionServer<A>` passes `this` from
/// `install_callbacks()` in both calls.
///
/// The callback fires once per accepted goal, after the accept reply has been
/// sent — for `AcceptAndExecute` and `AcceptAndDefer` alike. Pass a null
/// `accepted_cb` to clear it.
///
/// # Safety
/// `handle` must be a valid initialized action server storage.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_set_accepted_callback(
    handle: *mut c_void,
    accepted_cb: Option<CppAcceptedCallback>,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    if handle.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let server = unsafe { &mut *(handle as *mut CppActionServer) };
    server.accepted_cb = accepted_cb;
    server.cb_ctx = ctx;
    NROS_CPP_RET_OK
}

/// Publish feedback for an active goal.
///
/// # Parameters
/// * `handle` — Action server handle.
/// * `executor_handle` — Executor handle from `nros_cpp_init()`.
/// * `goal_id` — 16-byte goal UUID.
/// * `feedback_buf` — CDR-serialized feedback data.
/// * `feedback_len` — Length of feedback data.
///
/// # Safety
/// All pointers must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_publish_feedback(
    handle: *mut c_void,
    executor_handle: *mut c_void,
    goal_id: *const [u8; 16],
    feedback_buf: *const u8,
    feedback_len: usize,
) -> nros_cpp_ret_t {
    if handle.is_null() || goal_id.is_null() || feedback_buf.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let server = unsafe { &*(handle as *const CppActionServer) };
    let Some(ctx) = (unsafe { cpp_ctx_checked(executor_handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let id = nros::GoalId {
        uuid: unsafe { *goal_id },
    };
    let data = unsafe { core::slice::from_raw_parts(feedback_buf, feedback_len) };
    // Phase 177.9.F — strip the C++ serializer's CDR header before handing
    // feedback to the core, exactly as `nros_cpp_action_server_complete_goal`
    // does for results. The core action storage keeps FIELDS ONLY; the
    // client-side `cpp_feedback_trampoline` re-adds a CDR header before C++
    // deserialize. Without this strip the feedback travelled with its header,
    // the trampoline added a second one, and the C++ deserializer read the
    // inner header's bytes as the sequence length → garbage → `feedback=0`
    // on the cpp/xrce action E2E.
    let feedback_fields = strip_cdr_header(data);

    let h = match &server.handle {
        Some(h) => h,
        None => return NROS_CPP_RET_ERROR,
    };
    match h.publish_feedback_raw(&mut ctx.executor, &id, feedback_fields) {
        Ok(()) => NROS_CPP_RET_OK,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Complete a goal with a result.
///
/// # Parameters
/// * `handle` — Action server handle.
/// * `executor_handle` — Executor handle from `nros_cpp_init()`.
/// * `goal_id` — 16-byte goal UUID.
/// * `result_buf` — CDR-serialized result data.
/// * `result_len` — Length of result data.
///
/// # Safety
/// All pointers must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_complete_goal(
    handle: *mut c_void,
    executor_handle: *mut c_void,
    goal_id: *const [u8; 16],
    status: i32,
    result_buf: *const u8,
    result_len: usize,
) -> nros_cpp_ret_t {
    if handle.is_null() || goal_id.is_null() || result_buf.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let server = unsafe { &*(handle as *const CppActionServer) };
    let Some(ctx) = (unsafe { cpp_ctx_checked(executor_handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let id = nros::GoalId {
        uuid: unsafe { *goal_id },
    };
    let data = unsafe { core::slice::from_raw_parts(result_buf, result_len) };
    // Core action storage keeps result fields only; C++ serializers include
    // the CDR header for user-facing deserialize calls.
    let result_fields = strip_cdr_header(data);

    let h = match &server.handle {
        Some(h) => h,
        None => return NROS_CPP_RET_ERROR,
    };
    // issue 0796 — this used to pass `Succeeded` unconditionally, so a C++
    // callback-tier server that aborted a goal reported it to the client as
    // succeeded. Every other surface (C, the C++ polling tier, both Rust
    // servers) takes a status; this one now does too.
    let Some(status) = nros::GoalStatus::from_i8(status as i8) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    // issue 0796 — a result the server cannot retain (larger than the action
    // server's RESULT_BUF) is now reported instead of silently dropped.
    match h.complete_goal_raw(&mut ctx.executor, &id, status, result_fields) {
        Ok(()) => NROS_CPP_RET_OK,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Iterate over every goal currently live in the arena.
///
/// Calls `visitor(uuid, status, ctx)` for each entry in the
/// arena's `active_goals`. Status is the raw i8 discriminant of
/// `nros_core::GoalStatus` (0 = Unknown, 1 = Accepted, 2 = Executing,
/// 3 = Canceling, 4 = Succeeded, 5 = Canceled, 6 = Aborted). The arena
/// never stores the original goal CDR payload, so only identity + status
/// are forwarded; users needing the goal bytes should stash them in
/// their own `{uuid → state}` table keyed from `set_goal_callback`.
///
/// # Safety
/// `handle` must be a valid `CppActionServer` storage pointer.
/// `executor_handle` must point to a valid `CppContext`.
/// `visitor` must be a valid function pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_for_each_active_goal(
    handle: *mut c_void,
    executor_handle: *mut c_void,
    visitor: Option<unsafe extern "C" fn(goal_id: *const [u8; 16], status: i8, ctx: *mut c_void)>,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    if handle.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let Some(visitor) = visitor else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let server = unsafe { &*(handle as *const CppActionServer) };
    let Some(executor_ctx) = (unsafe { cpp_ctx_checked(executor_handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let executor_ctx = &*executor_ctx;
    let arena_handle = match &server.handle {
        Some(h) => *h,
        None => return NROS_CPP_RET_ERROR,
    };
    arena_handle.for_each_active_goal(&executor_ctx.executor, |g| unsafe {
        let uuid_ptr: *const [u8; 16] = &g.goal_id.uuid;
        visitor(uuid_ptr, g.status as i8, ctx);
    });
    NROS_CPP_RET_OK
}

/// Destroy an action server (drop in place, no free).
///
/// # Safety
/// `storage` must be a valid initialized action server storage, or NULL (no-op).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_destroy(storage: *mut c_void) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_OK;
    }
    unsafe {
        core::ptr::drop_in_place(storage as *mut CppActionServer);
    }
    NROS_CPP_RET_OK
}

/// Relocate a `CppActionServer` from `old_storage` to `new_storage`.
///
/// Performs the bitwise move. The C++ `ActionServer<A>` move ctor /
/// move assignment must still call `install_callbacks()` afterwards,
/// because the callback trampolines were registered with the previous
/// `this` as their context and need to be re-registered with the new
/// `this`. This FFI only transfers the runtime state; the re-install
/// step is intentionally left to the C++ side so this function stays
/// free of C++-specific pointer semantics.
///
/// # Safety
/// See `nros_cpp_publisher_relocate`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_relocate(
    old_storage: *mut c_void,
    new_storage: *mut c_void,
) -> nros_cpp_ret_t {
    if old_storage.is_null() || new_storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    unsafe {
        let value = core::ptr::read(old_storage as *mut CppActionServer);
        core::ptr::write(new_storage as *mut CppActionServer, value);
    }
    NROS_CPP_RET_OK
}

// ============================================================================
// Action Client
// ============================================================================

/// Internal state for the action client.
/// C++ action client callback function pointers (freestanding C++14).
#[repr(C)]
pub(crate) struct CppActionClientCallbacks {
    pub goal_response:
        Option<unsafe extern "C" fn(accepted: bool, goal_id: *const [u8; 16], ctx: *mut c_void)>,
    pub feedback: Option<
        unsafe extern "C" fn(
            goal_id: *const [u8; 16],
            data: *const u8,
            len: usize,
            ctx: *mut c_void,
        ),
    >,
    pub result: Option<
        unsafe extern "C" fn(
            goal_id: *const [u8; 16],
            status: i32,
            data: *const u8,
            len: usize,
            ctx: *mut c_void,
        ),
    >,
    pub context: *mut c_void,
}

impl Default for CppActionClientCallbacks {
    fn default() -> Self {
        Self {
            goal_response: None,
            feedback: None,
            result: None,
            context: core::ptr::null_mut(),
        }
    }
}

/// Internal state for the C++ action client.
///
/// Lightweight — the `ActionClientCore` lives in the executor's arena.
/// This struct stores the arena entry index, executor pointer, and callbacks.
/// Phase 87.6: the action_name buffer moved to the C++
/// `nros::ActionClient<A>` class.
///
/// `#[repr(C)]` is load-bearing (see `CppActionServer`): pins the layout to
/// match `#[repr(C)]` `nros::sizes::CppActionClientLayout` on 32-bit targets
/// too, where repr(Rust) packing would otherwise diverge.
#[repr(C)]
pub(crate) struct CppActionClient {
    callbacks: CppActionClientCallbacks,
    arena_entry_index: i32,
    executor_ptr: *mut c_void,
}

/// Get a mutable reference to an action client's core in the executor arena.
///
/// # Safety
/// `executor_ptr` must point to a valid `CppContext`.
unsafe fn cpp_arena_core_mut<'a>(
    arena_entry_index: i32,
    executor_ptr: *mut c_void,
) -> Option<&'a mut nros_node::ActionClientCore> {
    if arena_entry_index < 0 || executor_ptr.is_null() {
        return None;
    }
    unsafe {
        let ctx = &mut *(executor_ptr as *mut CppContext);
        ctx.executor
            .action_client_core_mut(arena_entry_index as usize)
    }
}

// Layout-mirror equivalence (Phase 87.11). See the matching assert
// above for `CppActionServer`.
const _: () = assert!(
    core::mem::size_of::<CppActionClient>()
        == core::mem::size_of::<nros::sizes::CppActionClientLayout>(),
    "CppActionClient size diverges from nros::sizes::CppActionClientLayout — \
     update the layout mirror to track any field-shape change"
);

// C++ action client callback trampolines for the arena entry.
// `context` is the CppActionClient storage pointer.
unsafe extern "C" fn cpp_goal_response_trampoline(
    goal_id: *const nros::GoalId,
    accepted: bool,
    context: *mut c_void,
) {
    let client = unsafe { &*(context as *const CppActionClient) };
    if let Some(cb) = client.callbacks.goal_response {
        unsafe { cb(accepted, &(*goal_id).uuid, client.callbacks.context) };
    }
}

/// Single-slot stash for feedback messages consumed by the trampoline.
///
/// The arena's `action_client_raw_try_process` consumes feedback from
/// the core during `spin_once`, then fires the trampoline. If the
/// user registered a `feedback` callback via
/// `nros_cpp_action_client_set_callbacks`, the trampoline forwards
/// to it. Otherwise — the polling case used by the example —
/// without this stash the message would simply be dropped, leaving
/// `feedback_stream().try_next()` and `try_recv_feedback()` empty
/// even though the server published. Stashing the latest feedback
/// here lets both APIs coexist.
///
/// Single slot: a second feedback arriving before the first is
/// drained overwrites it. Real-world action workflows rarely need
/// more than the latest sample, and a heapless ring would add
/// per-slot capacity tradeoffs without test pressure.
static mut FEEDBACK_STASH_LEN: i32 = -1;
static mut FEEDBACK_STASH: [u8; DEFAULT_RX_BUF_SIZE] = [0u8; DEFAULT_RX_BUF_SIZE];
static mut FEEDBACK_STASH_GOAL_ID: nros::GoalId = nros::GoalId { uuid: [0u8; 16] };

/// Issue #179 — a raw payload never starts with a CDR encapsulation
/// identifier (`00 {00|01|06|07|0a|0b}`). The upstream dispatch (arena /
/// poll) now delivers action payloads WITH a single encap header (splicing
/// one in when a C server's stripped-fields reply arrives raw), so the
/// trampolines must prepend ONLY when the header is genuinely absent —
/// the old unconditional prepend double-framed every zenoh delivery.
fn payload_has_cdr_encap(buf: &[u8]) -> bool {
    buf.len() >= 2 && buf[0] == 0 && matches!(buf[1], 0 | 1 | 6 | 7 | 0x0a | 0x0b)
}

unsafe extern "C" fn cpp_feedback_trampoline(
    goal_id: *const nros::GoalId,
    feedback_data: *const u8,
    feedback_len: usize,
    context: *mut c_void,
) {
    let data = unsafe { core::slice::from_raw_parts(feedback_data, feedback_len) };
    let mut framed = [0u8; DEFAULT_RX_BUF_SIZE];
    let framed_len = if payload_has_cdr_encap(data) {
        let n = feedback_len.min(DEFAULT_RX_BUF_SIZE);
        framed[..n].copy_from_slice(&data[..n]);
        n
    } else {
        framed[..CDR_HEADER_LEN].copy_from_slice(&nros::cdr::CDR_LE_HEADER);
        let n = feedback_len.min(DEFAULT_RX_BUF_SIZE - CDR_HEADER_LEN);
        framed[CDR_HEADER_LEN..CDR_HEADER_LEN + n].copy_from_slice(&data[..n]);
        CDR_HEADER_LEN + n
    };

    // Always stash the latest feedback for `try_recv_feedback` /
    // `feedback_stream().try_next()` polling.
    unsafe {
        core::ptr::copy_nonoverlapping(
            framed.as_ptr(),
            core::ptr::addr_of_mut!(FEEDBACK_STASH) as *mut u8,
            framed_len,
        );
        core::ptr::write(
            core::ptr::addr_of_mut!(FEEDBACK_STASH_LEN),
            framed_len as i32,
        );
        core::ptr::write(core::ptr::addr_of_mut!(FEEDBACK_STASH_GOAL_ID), *goal_id);
    }

    // Also forward to user callback if set.
    let client = unsafe { &*(context as *const CppActionClient) };
    if let Some(cb) = client.callbacks.feedback {
        unsafe {
            cb(
                &(*goal_id).uuid,
                framed.as_ptr(),
                framed_len,
                client.callbacks.context,
            )
        };
    }
}

/// Stash for result data captured by the trampoline.
/// Used by `nros_cpp_action_client_try_recv_result` to retrieve
/// results that were consumed by the executor's auto-dispatch.
static mut RESULT_STASH_LEN: i32 = -1; // -1 = empty, >= 0 = data length
static mut RESULT_STASH: [u8; DEFAULT_RX_BUF_SIZE] = [0u8; DEFAULT_RX_BUF_SIZE];

unsafe extern "C" fn cpp_result_trampoline(
    goal_id: *const nros::GoalId,
    status: nros::GoalStatus,
    result_data: *const u8,
    result_len: usize,
    context: *mut c_void,
) {
    // User callbacks expect normal CDR bytes. The upstream dispatch delivers
    // WITH a single encap header (issue #179); prepend only when it is
    // genuinely absent.
    let data = unsafe { core::slice::from_raw_parts(result_data, result_len) };
    let mut framed = [0u8; DEFAULT_RX_BUF_SIZE];
    let framed_len = if payload_has_cdr_encap(data) {
        let n = result_len.min(DEFAULT_RX_BUF_SIZE);
        framed[..n].copy_from_slice(&data[..n]);
        n
    } else {
        framed[..CDR_HEADER_LEN].copy_from_slice(&nros::cdr::CDR_LE_HEADER);
        let n = result_len.min(DEFAULT_RX_BUF_SIZE - CDR_HEADER_LEN);
        framed[CDR_HEADER_LEN..CDR_HEADER_LEN + n].copy_from_slice(&data[..n]);
        CDR_HEADER_LEN + n
    };

    // Always stash the result for Future::wait polling
    unsafe {
        core::ptr::copy_nonoverlapping(
            framed.as_ptr(),
            core::ptr::addr_of_mut!(RESULT_STASH) as *mut u8,
            framed_len,
        );
        core::ptr::write(core::ptr::addr_of_mut!(RESULT_STASH_LEN), framed_len as i32);
    }

    // Also forward to user callback if set
    let client = unsafe { &*(context as *const CppActionClient) };
    if let Some(cb) = client.callbacks.result {
        let s = match status {
            nros::GoalStatus::Succeeded => 4i32,
            nros::GoalStatus::Canceled => 5,
            nros::GoalStatus::Aborted => 6,
            _ => 0,
        };
        unsafe {
            cb(
                &(*goal_id).uuid,
                s,
                framed.as_ptr(),
                framed_len,
                client.callbacks.context,
            )
        };
    }
}

/// Create an action client on a node.
///
/// # Safety
/// All pointer parameters must be valid. `storage` must point to an
/// 8-byte-aligned buffer of at least `CPP_ACTION_CLIENT_OPAQUE_U64S * 8` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_create(
    node: *const nros_cpp_node_t,
    action_name: *const c_char,
    type_name: *const c_char,
    type_hash: *const c_char,
    _qos: nros_cpp_qos_t,
    storage: *mut c_void,
) -> nros_cpp_ret_t {
    if node.is_null()
        || action_name.is_null()
        || type_name.is_null()
        || type_hash.is_null()
        || storage.is_null()
    {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let node_ref = unsafe { &*node };
    if node_ref.executor.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let act_str = match unsafe { cstr_to_str(action_name) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let type_str = match unsafe { cstr_to_str(type_name) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    // Issue 0312 — an empty hash would land in the liveliness keyexpr as an
    // empty segment, making the entity invisible to ROS 2 discovery.
    let hash_str = match unsafe { cstr_to_str(type_hash) } {
        Some(s) => crate::normalize_type_hash(s),
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };

    let ctx = unsafe { &mut *(node_ref.executor as *mut CppContext) };

    // Phase 305 W3 (issue 0255) — resolve `~`/relative names + launch remaps.
    let resolved_act = match crate::resolve_node_entity_name(ctx, node_ref, act_str) {
        Ok(r) => r,
        Err(()) => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let act_str = resolved_act.as_str();

    // Register with executor — creates the ONLY ActionClientCore in the arena.
    // Trampolines read from CppActionClient.callbacks (set later via set_callbacks).
    // Phase 104.C.9.b — `spec.node_id` routes multi-Node action clients
    // through the named Node's session; `None` uses the default Node.
    let node_id = crate::node_id_opt(node_ref);
    let handle = match ctx
        .executor
        .register_action_client_raw(nros_node::RawActionClientSpec {
            node_id,
            action_name: act_str,
            type_name: type_str,
            type_hash: hash_str,
            goal_response_callback: Some(cpp_goal_response_trampoline),
            feedback_callback: Some(cpp_feedback_trampoline),
            result_callback: Some(cpp_result_trampoline),
            context: storage, // context = CppActionClient pointer
        }) {
        Ok(h) => h,
        // issue 0870 — was `NROS_CPP_RET_TRANSPORT_ERROR`, discarding a typed
        // `NodeError`. Nothing here has touched the transport yet: this is the
        // executor's arena claim, so the variant that actually fires is
        // `ExecutorFull` (NROS_EXECUTOR_MAX_CBS) — which exists, per its own
        // doc, "so the register seam can name the knob". Reporting it as a TX
        // failure sent the diagnosis at zenoh-pico's send path instead of at a
        // capacity knob. `node_error_to_cpp_ret` already maps every variant and
        // prints it; this seam simply was not calling it. Same collapse issue
        // 0557 fixed inside the mapper, one layer out.
        Err(e) => return crate::node_error_to_cpp_ret(e),
    };

    let client = CppActionClient {
        callbacks: CppActionClientCallbacks::default(),
        arena_entry_index: handle.entry_index() as i32,
        executor_ptr: node_ref.executor,
    };

    // Write directly into caller-provided storage — no heap allocation.
    unsafe {
        core::ptr::write(storage as *mut CppActionClient, client);
    }
    NROS_CPP_RET_OK
}

/// Send a goal and receive the generated goal UUID.
///
/// # Parameters
/// * `handle` — Action client handle.
/// * `goal_buf` — CDR-serialized goal data.
/// * `goal_len` — Length of goal data.
/// * `goal_id_out` — Receives the 16-byte goal UUID.
///
/// # Safety
/// All pointers must be valid.
/// phase-338 W8 — block until the action server's send-goal queryable is
/// discoverable.
///
/// C++ sibling of `nros_action_client_wait_for_action_server`, which the C API
/// has had since phase-124 while C++ bound neither it nor a readiness probe —
/// so the C++ action-client examples hand-rolled a 3-attempt retry around
/// `send_goal`. Mirrors `rclcpp_action::Client::wait_for_action_server`.
///
/// Spins on the `send_goal` service client's readiness, which is the
/// load-bearing entity for the first `send_goal`; the same shape as
/// `ActionClient::wait_for_action_server` in
/// `nros-node/src/executor/handles.rs` (phase-428 W13).
///
/// # Safety
/// `handle` must be a valid initialized `CppActionClient`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_wait_for_action_server(
    handle: *mut c_void,
    timeout_ms: u32,
) -> nros_cpp_ret_t {
    if handle.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let client = unsafe { &mut *(handle as *mut CppActionClient) };
    let executor_ptr = client.executor_ptr;
    if executor_ptr.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    const SPIN_MS: u64 = 10;
    let deadline_ns = crate::nros_cpp_time_ns() + (timeout_ms as u64) * 1_000_000;

    // phase-428 W13 — spin, then ask again, re-borrowing the arena each time
    // so the executor borrow does not overlap.
    loop {
        {
            let Some(core) =
                (unsafe { cpp_arena_core_mut(client.arena_entry_index, executor_ptr) })
            else {
                return NROS_CPP_RET_INVALID_ARGUMENT;
            };
            if core.is_server_ready() {
                return NROS_CPP_RET_OK;
            }
        }
        if crate::nros_cpp_time_ns() >= deadline_ns {
            return NROS_CPP_RET_TIMEOUT;
        }
        let ctx = unsafe { &mut *(executor_ptr as *mut CppContext) };
        let _ = ctx
            .executor
            .spin_once(core::time::Duration::from_millis(SPIN_MS));
    }
}

/// Send a goal and block until the server accepts or rejects it.
///
/// # Returns
///
/// Three outcomes, three codes — they were two before issue 0868, and callers
/// could not tell a server decision from a failed send:
///
/// * `NROS_CPP_RET_OK` — the server ACCEPTED; `goal_id_out` is filled.
/// * `NROS_CPP_RET_REJECTED` — the server RECEIVED the goal and declined it.
///   This is a decision, not a fault: the server's goal callback said no.
/// * `NROS_CPP_RET_TIMEOUT` — no goal response within 30 s. The server may
///   never have received the goal at all; nothing here can tell.
/// * `NROS_CPP_RET_ERROR` — the goal could not be SENT (no arena entry, or
///   the backend refused the request).
/// * `NROS_CPP_RET_REENTRANT` — called from inside a callback. This spins the
///   executor, so it cannot run under one.
///
/// # Safety
/// All pointers must be valid; `handle` must be an initialized `CppActionClient`.
#[unsafe(no_mangle)]
#[allow(static_mut_refs)]
pub unsafe extern "C" fn nros_cpp_action_client_send_goal(
    handle: *mut c_void,
    goal_buf: *const u8,
    goal_len: usize,
    goal_id_out: *mut [u8; 16],
) -> nros_cpp_ret_t {
    if handle.is_null() || goal_buf.is_null() || goal_id_out.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let client = unsafe { &mut *(handle as *mut CppActionClient) };
    let goal_data = unsafe { core::slice::from_raw_parts(goal_buf, goal_len) };

    // C++ ffi_serialize produces [CDR_HEADER][fields], but send_goal_blocking
    // expects raw fields only (it adds its own CDR header + GoalId).
    let goal_fields = strip_cdr_header(goal_data);

    // Send goal via arena core (non-blocking)
    let core = match unsafe { cpp_arena_core_mut(client.arena_entry_index, client.executor_ptr) } {
        Some(c) => c,
        None => return NROS_CPP_RET_ERROR,
    };
    let goal_id = match core.send_goal_raw(goal_fields) {
        Ok(id) => id,
        Err(_) => return NROS_CPP_RET_ERROR,
    };
    unsafe {
        *goal_id_out = goal_id.uuid;
    }

    // Use a flag-based approach: install a temporary goal_response callback
    // that sets a local flag. The arena's action_client_raw_try_process fires
    // the trampoline during spin_once, which reads client.callbacks and
    // dispatches to the user's callback (or our temporary one).
    static mut BLOCKING_ACCEPTED: i32 = -1; // -1=pending, 0=rejected, 1=accepted
    unsafe {
        core::ptr::write(core::ptr::addr_of_mut!(BLOCKING_ACCEPTED), -1i32);
    }

    // Save original callback and install temporary one
    let orig_cb = client.callbacks.goal_response;
    let orig_ctx = client.callbacks.context;
    unsafe extern "C" fn blocking_goal_cb(
        _accepted: bool,
        _goal_id: *const [u8; 16],
        _ctx: *mut c_void,
    ) {
        unsafe {
            core::ptr::write(
                core::ptr::addr_of_mut!(BLOCKING_ACCEPTED),
                if _accepted { 1i32 } else { 0i32 },
            );
        }
    }
    client.callbacks.goal_response = Some(blocking_goal_cb);
    client.callbacks.context = core::ptr::null_mut();

    // Spin executor until flag set or timeout (30 s wall-clock).
    //
    // Phase 89.3: wall-clock budgeting instead of `for _ in 0..1000`
    // (same class of bug fixed for service clients in 89.2). On
    // multi-threaded zpico backends (POSIX/Zephyr), `spin_once(10)`
    // returns early on any incoming frame (keep-alives, discovery
    // gossip, interest messages) — a 1000-iteration budget collapses
    // to milliseconds of real time and the goal-response callback
    // never has a chance to fire before we return Error.
    //
    // Phase 160.C.2 — bumped 10 s → 30 s. Zephyr zenoh-pico's
    // declare-cascade slowness extends to the get-query path too;
    // server's queryable callback may not run for ~12 s after the
    // client sends the goal (lease/keepalive serialization). The
    // 10 s budget was racing this on test_zephyr_cpp_action_server_to_client_e2e.
    let ctx = unsafe { &mut *(client.executor_ptr as *mut CppContext) };
    // Issue 0290 — this helper BLOCKS by spinning the executor, so it must not
    // run from inside a callback (the C twin `nros_action_send_goal` returns
    // NROS_RET_REENTRANT for exactly this). Restore the caller's callback
    // before bailing so the guard can't strand the client on the blocking one.
    let CppContext {
        executor,
        in_dispatch,
        ..
    } = ctx;
    let Some(_guard) = DispatchGuard::enter(in_dispatch) else {
        client.callbacks.goal_response = orig_cb;
        client.callbacks.context = orig_ctx;
        return NROS_CPP_RET_REENTRANT;
    };
    let start_ns = crate::nros_cpp_time_ns();
    let timeout_ns: u64 = 30_000_000_000; // 30 s
    loop {
        let _ = executor.spin_once(core::time::Duration::from_millis(10));
        let flag = unsafe { core::ptr::read(core::ptr::addr_of!(BLOCKING_ACCEPTED)) };
        if flag >= 0 {
            // Restore original callback
            client.callbacks.goal_response = orig_cb;
            client.callbacks.context = orig_ctx;
            // issue 0868 — a server REJECTION is `REJECTED`, not `ERROR`.
            //
            // This used to return `ERROR` (-1), which is also what the two
            // failure paths above return when the goal could not be SENT at
            // all (no arena core, `send_goal_raw` failed). One code for
            // "the server considered this goal and said no" and "the goal
            // never left" makes the distinction unrecoverable by the caller,
            // so every C++ action-client example printed "Goal was rejected
            // by server" for all three outcomes including `TIMEOUT`.
            //
            // With this split the three are distinguishable: `REJECTED` = the
            // server decided, `ERROR` = we could not send, `TIMEOUT` = no
            // answer within the budget.
            return if flag == 1 {
                NROS_CPP_RET_OK
            } else {
                NROS_CPP_RET_REJECTED
            };
        }
        let elapsed_ns = crate::nros_cpp_time_ns().saturating_sub(start_ns);
        if elapsed_ns >= timeout_ns {
            break;
        }
    }
    // Restore original callback on timeout
    client.callbacks.goal_response = orig_cb;
    client.callbacks.context = orig_ctx;
    NROS_CPP_RET_TIMEOUT
}

/// Get the result for a goal (blocking with timeout).
///
/// Sends a get_result request and polls for the reply.
///
/// # Parameters
/// * `handle` — Action client handle.
/// * `executor_handle` — Executor handle for spin_once during polling.
/// * `goal_id` — 16-byte goal UUID.
/// * `result_buf` — Buffer for CDR-serialized result data.
/// * `result_buf_len` — Size of result buffer.
/// * `result_len` — Receives actual result data length.
///
/// # Safety
/// All pointers must be valid.
#[unsafe(no_mangle)]
#[allow(static_mut_refs)]
pub unsafe extern "C" fn nros_cpp_action_client_get_result(
    handle: *mut c_void,
    executor_handle: *mut c_void,
    goal_id: *const [u8; 16],
    result_buf: *mut u8,
    result_buf_len: usize,
    result_len: *mut usize,
) -> nros_cpp_ret_t {
    if handle.is_null() || goal_id.is_null() || result_buf.is_null() || result_len.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let client = unsafe { &mut *(handle as *mut CppActionClient) };
    let Some(_ctx) = (unsafe { cpp_ctx_checked(executor_handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let id = nros::GoalId {
        uuid: unsafe { *goal_id },
    };

    // Send get_result request via arena core (non-blocking)
    {
        let core =
            match unsafe { cpp_arena_core_mut(client.arena_entry_index, client.executor_ptr) } {
                Some(c) => c,
                None => return NROS_CPP_RET_ERROR,
            };
        if core.send_get_result_request(&id).is_err() {
            return NROS_CPP_RET_ERROR;
        }
    }

    // Flag-based: install temporary result callback, spin until flag set.
    static mut BLOCKING_RESULT_LEN: i32 = -1; // -1=pending, >=0=length
    static mut BLOCKING_RESULT_STATUS: i32 = 0;
    static mut BLOCKING_RESULT_BUF: [u8; DEFAULT_RX_BUF_SIZE] = [0u8; DEFAULT_RX_BUF_SIZE];
    unsafe {
        core::ptr::write(core::ptr::addr_of_mut!(BLOCKING_RESULT_LEN), -1i32);
        core::ptr::write(core::ptr::addr_of_mut!(BLOCKING_RESULT_STATUS), 0i32);
    }

    let orig_cb = client.callbacks.result;
    let orig_ctx = client.callbacks.context;
    unsafe extern "C" fn blocking_result_cb(
        _goal_id: *const [u8; 16],
        status: i32,
        data: *const u8,
        len: usize,
        _ctx: *mut c_void,
    ) {
        unsafe {
            core::ptr::write(core::ptr::addr_of_mut!(BLOCKING_RESULT_STATUS), status);
            let copy_len = len.min(DEFAULT_RX_BUF_SIZE);
            core::ptr::copy_nonoverlapping(
                data,
                core::ptr::addr_of_mut!(BLOCKING_RESULT_BUF) as *mut u8,
                copy_len,
            );
            core::ptr::write(
                core::ptr::addr_of_mut!(BLOCKING_RESULT_LEN),
                copy_len as i32,
            );
        }
    }
    client.callbacks.result = Some(blocking_result_cb);
    client.callbacks.context = core::ptr::null_mut();

    // Spin executor until flag set or timeout (10 s wall-clock).
    //
    // Phase 89.3: wall-clock budgeting — see the explanation on
    // `send_goal` above for why `for _ in 0..1000` is insufficient on
    // multi-threaded zpico backends.
    let ctx = unsafe { &mut *(client.executor_ptr as *mut CppContext) };
    // Issue 0290 — blocking helper; refuse to re-enter from a callback.
    let CppContext {
        executor,
        in_dispatch,
        ..
    } = ctx;
    let Some(_guard) = DispatchGuard::enter(in_dispatch) else {
        client.callbacks.result = orig_cb;
        client.callbacks.context = orig_ctx;
        return NROS_CPP_RET_REENTRANT;
    };
    let start_ns = crate::nros_cpp_time_ns();
    let timeout_ns: u64 = 10_000_000_000; // 10 s
    loop {
        let _ = executor.spin_once(core::time::Duration::from_millis(10));
        let rlen = unsafe { core::ptr::read(core::ptr::addr_of!(BLOCKING_RESULT_LEN)) };
        if rlen >= 0 {
            client.callbacks.result = orig_cb;
            client.callbacks.context = orig_ctx;
            let data_len = rlen as usize;
            if data_len <= result_buf_len {
                unsafe {
                    core::ptr::copy_nonoverlapping(
                        core::ptr::addr_of!(BLOCKING_RESULT_BUF) as *const u8,
                        result_buf,
                        data_len,
                    );
                    *result_len = data_len;
                }
                return NROS_CPP_RET_OK;
            }
            return NROS_CPP_RET_ERROR;
        }
        let elapsed_ns = crate::nros_cpp_time_ns().saturating_sub(start_ns);
        if elapsed_ns >= timeout_ns {
            break;
        }
    }
    client.callbacks.result = orig_cb;
    client.callbacks.context = orig_ctx;
    NROS_CPP_RET_TIMEOUT
}

/// Try to receive feedback (non-blocking).
///
/// # Parameters
/// * `handle` — Action client handle.
/// * `feedback_buf` — Buffer for CDR-serialized feedback data.
/// * `buf_len` — Size of feedback buffer.
/// * `feedback_len` — Receives actual feedback data length (0 if none available).
///
/// # Returns
/// `NROS_CPP_RET_OK` on success (check `*feedback_len > 0` for data).
///
/// # Safety
/// All pointers must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_try_recv_feedback(
    handle: *mut c_void,
    feedback_buf: *mut u8,
    buf_len: usize,
    feedback_len: *mut usize,
) -> nros_cpp_ret_t {
    if handle.is_null() || feedback_buf.is_null() || feedback_len.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    // First check the trampoline's stash. Arena's
    // `action_client_raw_try_process` consumes feedback from the
    // core during `spin_once`; without checking the stash here we'd
    // see no feedback even though the trampoline got it.
    let stash_len = unsafe { core::ptr::read(core::ptr::addr_of!(FEEDBACK_STASH_LEN)) };
    if stash_len >= 0 {
        let data_len = stash_len as usize;
        unsafe { core::ptr::write(core::ptr::addr_of_mut!(FEEDBACK_STASH_LEN), -1i32) };
        if data_len > buf_len {
            unsafe {
                *feedback_len = data_len;
            }
            return NROS_CPP_RET_ERROR;
        }
        unsafe {
            core::ptr::copy_nonoverlapping(
                core::ptr::addr_of!(FEEDBACK_STASH) as *const u8,
                feedback_buf,
                data_len,
            );
            *feedback_len = data_len;
        }
        return NROS_CPP_RET_OK;
    }

    let client = unsafe { &mut *(handle as *mut CppActionClient) };

    let core = match unsafe { cpp_arena_core_mut(client.arena_entry_index, client.executor_ptr) } {
        Some(c) => c,
        None => {
            unsafe {
                *feedback_len = 0;
            }
            return NROS_CPP_RET_OK;
        }
    };

    match core.try_recv_feedback_raw() {
        Ok(Some((_goal_id, total_len))) => {
            // Feedback buffer layout (see arena.rs
            // `FEEDBACK_PAYLOAD_OFFSET` comment): outer CDR(4) +
            // GoalId.length(u32 = 4) + GoalId.uuid(16) + payload.
            // Skip 24 bytes to land on the payload — the prior
            // `CDR_HEADER_LEN + GoalId::UUID_LEN` (= 20) missed
            // the GoalId length-prefix u32.
            const FEEDBACK_PAYLOAD_OFFSET: usize =
                CDR_HEADER_LEN + GoalId::SEQ_PREFIX_LEN + GoalId::UUID_LEN;
            let buf = core.feedback_buffer_ref();
            if total_len > FEEDBACK_PAYLOAD_OFFSET {
                let data = &buf[FEEDBACK_PAYLOAD_OFFSET..total_len];
                let framed_len = CDR_HEADER_LEN + data.len();
                if framed_len <= buf_len {
                    unsafe {
                        core::ptr::copy_nonoverlapping(
                            nros::cdr::CDR_LE_HEADER.as_ptr(),
                            feedback_buf,
                            CDR_HEADER_LEN,
                        );
                        core::ptr::copy_nonoverlapping(
                            data.as_ptr(),
                            feedback_buf.add(CDR_HEADER_LEN),
                            data.len(),
                        );
                        *feedback_len = framed_len;
                    }
                    return NROS_CPP_RET_OK;
                }
            }
            unsafe {
                *feedback_len = 0;
            }
            NROS_CPP_RET_OK
        }
        Ok(None) => {
            unsafe {
                *feedback_len = 0;
            }
            NROS_CPP_RET_OK
        }
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Try to receive the goal acceptance response (non-blocking).
///
/// Returns `NROS_CPP_RET_OK` with serialized `GoalAccept` data if ready,
/// `NROS_CPP_RET_TRY_AGAIN` if not yet available.
///
/// Output layout: `[goal_id: 16 bytes][accepted: 1 byte]` (17 bytes total).
///
/// Used by C++ `Future<GoalAccept>` via the `TryRecvFn` interface.
///
/// # Safety
/// All pointers must be valid. `out_data` must point to `out_capacity` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_try_recv_goal_response(
    handle: *mut c_void,
    out_data: *mut u8,
    out_capacity: usize,
    out_len: *mut usize,
) -> nros_cpp_ret_t {
    if handle.is_null() || out_data.is_null() || out_len.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let client = unsafe { &mut *(handle as *mut CppActionClient) };
    let core = match unsafe { cpp_arena_core_mut(client.arena_entry_index, client.executor_ptr) } {
        Some(c) => c,
        None => {
            unsafe { *out_len = 0 };
            return NROS_CPP_RET_TRY_AGAIN;
        }
    };

    match core.try_recv_send_goal_reply() {
        Ok(Some(total_len)) => {
            // The reply buffer contains: CDR header (4) + accepted byte (1) + ...
            // We produce: goal_id (16) + accepted (1) = 17 bytes
            let needed = 17usize;
            if needed > out_capacity {
                unsafe { *out_len = needed };
                return NROS_CPP_RET_ERROR;
            }
            let buf = core.result_buffer_ref();
            let accepted: u8 = if total_len >= 5 && buf[4] != 0 { 1 } else { 0 };
            // Fill goal_id from the counter (same logic as poll())
            let counter = core.goal_counter();
            let mut uuid = [0u8; 16];
            uuid[..8].copy_from_slice(&counter.to_le_bytes());
            unsafe {
                core::ptr::copy_nonoverlapping(uuid.as_ptr(), out_data, 16);
                *out_data.add(16) = accepted;
                *out_len = needed;
            }
            NROS_CPP_RET_OK
        }
        Ok(None) => {
            unsafe { *out_len = 0 };
            NROS_CPP_RET_TRY_AGAIN
        }
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Decode the goal-acceptance payload into split out-params.
///
/// Issue 0329 — the goal-accept wire layout (16-byte goal_id, then one
/// `accepted` byte) is produced by
/// `nros_cpp_action_client_try_recv_goal_response` above. It was ALSO decoded by
/// hand in the C++ `action_client.hpp` header (`GoalAccept::ffi_deserialize`) —
/// the same 17-byte layout in two places, a drift no ABI assert covers. This
/// owns the read Rust-side so the header only adapts types.
///
/// # Safety
/// `data` must point to at least `len` readable bytes; `out_goal_id` /
/// `out_accepted` must be valid writable pointers.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_goal_accept_decode(
    data: *const u8,
    len: usize,
    out_goal_id: *mut [u8; 16],
    out_accepted: *mut bool,
) -> nros_cpp_ret_t {
    if data.is_null() || out_goal_id.is_null() || out_accepted.is_null() || len < 17 {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    unsafe {
        core::ptr::copy_nonoverlapping(data, (*out_goal_id).as_mut_ptr(), 16);
        *out_accepted = *data.add(16) != 0;
    }
    NROS_CPP_RET_OK
}

/// Try to receive the result for a pending get_result request (non-blocking).
///
/// Returns `NROS_CPP_RET_OK` with result data if ready,
/// `NROS_CPP_RET_TRY_AGAIN` if not yet available.
///
/// Output layout: CDR-serialized result fields (same as `get_result` output).
///
/// Used by C++ `Future<ResultType>` via the `TryRecvFn` interface.
///
/// # Safety
/// All pointers must be valid. `out_data` must point to `out_capacity` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_try_recv_result(
    handle: *mut c_void,
    out_data: *mut u8,
    out_capacity: usize,
    out_len: *mut usize,
) -> nros_cpp_ret_t {
    if handle.is_null() || out_data.is_null() || out_len.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    // Check the result stash (filled by cpp_result_trampoline during spin_once).
    // The executor's action_client_raw_try_process consumes the reply from the
    // core and fires the trampoline, which stashes the data here. We can't call
    // core.try_recv_get_result_reply() because the data is already consumed.
    //
    // The stash contains the payload as written by the action server
    // (the bytes `complete_goal_raw` was given) — for cpp callers that
    // means a CDR-encoded buffer with header included, since
    // `ffi_serialize` writes one. Forward it verbatim; `ffi_deserialize`
    // re-reads the CDR header.
    let stash_len = unsafe { core::ptr::read(core::ptr::addr_of!(RESULT_STASH_LEN)) };
    if stash_len >= 0 {
        let data_len = stash_len as usize;
        unsafe { core::ptr::write(core::ptr::addr_of_mut!(RESULT_STASH_LEN), -1i32) };

        if data_len > out_capacity {
            unsafe { *out_len = data_len };
            return NROS_CPP_RET_ERROR;
        }
        unsafe {
            core::ptr::copy_nonoverlapping(
                core::ptr::addr_of!(RESULT_STASH) as *const u8,
                out_data,
                data_len,
            );
            *out_len = data_len;
        }
        return NROS_CPP_RET_OK;
    }

    // Fallback: check the core directly (in case the trampoline wasn't fired yet)
    let _client = unsafe { &mut *(handle as *mut CppActionClient) };
    let core = match unsafe { cpp_arena_core_mut(_client.arena_entry_index, _client.executor_ptr) }
    {
        Some(c) => c,
        None => {
            unsafe { *out_len = 0 };
            return NROS_CPP_RET_TRY_AGAIN;
        }
    };

    match core.try_recv_get_result_reply() {
        Ok(Some(total_len)) => {
            // Reply layout (see arena.rs `RESULT_PAYLOAD_OFFSET`
            // comment): outer CDR(4) + status(1) + align(4 → pad 3)
            // + payload. Skip 8 bytes to land on the payload.
            const RESULT_PAYLOAD_OFFSET: usize = 8;
            if total_len <= RESULT_PAYLOAD_OFFSET {
                unsafe { *out_len = 0 };
                return NROS_CPP_RET_OK;
            }
            let data_len = total_len - RESULT_PAYLOAD_OFFSET;
            if data_len > out_capacity {
                unsafe { *out_len = data_len };
                return NROS_CPP_RET_ERROR;
            }
            let buf = core.result_buffer_ref();
            unsafe {
                core::ptr::copy_nonoverlapping(
                    buf[RESULT_PAYLOAD_OFFSET..total_len].as_ptr(),
                    out_data,
                    data_len,
                );
                *out_len = data_len;
            }
            NROS_CPP_RET_OK
        }
        Ok(None) => {
            unsafe { *out_len = 0 };
            NROS_CPP_RET_TRY_AGAIN
        }
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Destroy an action client (drop in place, no free).
///
/// # Safety
/// `storage` must be a valid initialized action client storage, or NULL (no-op).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_destroy(storage: *mut c_void) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_OK;
    }
    unsafe {
        core::ptr::drop_in_place(storage as *mut CppActionClient);
    }
    NROS_CPP_RET_OK
}

/// Relocate a `CppActionClient` from `old_storage` to `new_storage`.
///
/// The action client's async callback context (`options.context`) is
/// user-provided, so it stays valid across the move. Relocation is a
/// straight `ptr::read` + `ptr::write`.
///
/// # Safety
/// See `nros_cpp_publisher_relocate`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_relocate(
    old_storage: *mut c_void,
    new_storage: *mut c_void,
) -> nros_cpp_ret_t {
    if old_storage.is_null() || new_storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    unsafe {
        let value = core::ptr::read(old_storage as *mut CppActionClient);
        core::ptr::write(new_storage as *mut CppActionClient, value);
    }
    NROS_CPP_RET_OK
}

// ============================================================================
// Async (non-blocking) Action Client FFI
// ============================================================================

/// Send a goal asynchronously (non-blocking).
///
/// Uses `send_goal_raw` (zpico_get_start) instead of `send_goal_blocking`.
/// The goal response arrives via the callback registered with
/// `nros_cpp_action_client_register_async`, invoked during `spin_once`.
///
/// # Safety
/// All pointers must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_send_goal_async(
    handle: *mut c_void,
    goal_buf: *const u8,
    goal_len: usize,
    goal_id_out: *mut [u8; 16],
) -> nros_cpp_ret_t {
    if handle.is_null() || goal_buf.is_null() || goal_id_out.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let client = unsafe { &mut *(handle as *mut CppActionClient) };
    let goal_data = unsafe { core::slice::from_raw_parts(goal_buf, goal_len) };

    // Strip CDR header (same as blocking variant)
    let goal_fields = strip_cdr_header(goal_data);

    let core = match unsafe { cpp_arena_core_mut(client.arena_entry_index, client.executor_ptr) } {
        Some(c) => c,
        None => return NROS_CPP_RET_ERROR,
    };

    // Non-blocking: uses zpico_get_start internally
    match core.send_goal_raw(goal_fields) {
        Ok(goal_id) => {
            unsafe {
                *goal_id_out = goal_id.uuid;
            }
            NROS_CPP_RET_OK
        }
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Request a goal result asynchronously (non-blocking).
///
/// Uses `send_get_result_request` (zpico_get_start) instead of
/// `get_result_blocking`. The result arrives via the result callback
/// registered with `nros_cpp_action_client_register_async`.
///
/// # Safety
/// All pointers must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_get_result_async(
    handle: *mut c_void,
    goal_id: *const [u8; 16],
) -> nros_cpp_ret_t {
    if handle.is_null() || goal_id.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let client = unsafe { &mut *(handle as *mut CppActionClient) };
    let id = nros::GoalId {
        uuid: unsafe { *goal_id },
    };

    // Reset stashes before sending (so take_*  starts clean)
    unsafe { core::ptr::write(core::ptr::addr_of_mut!(RESULT_STASH_LEN), -1i32) };
    unsafe { core::ptr::write(core::ptr::addr_of_mut!(FEEDBACK_STASH_LEN), -1i32) };

    let core = match unsafe { cpp_arena_core_mut(client.arena_entry_index, client.executor_ptr) } {
        Some(c) => c,
        None => return NROS_CPP_RET_ERROR,
    };

    match core.send_get_result_request(&id) {
        Ok(()) => NROS_CPP_RET_OK,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Cancel a goal (non-blocking) — issue 0796.
///
/// Sends the `action_msgs/srv/CancelGoal` request for `goal_id` and returns.
/// The RPC's return code arrives later; drain it with
/// [`nros_cpp_action_client_try_recv_cancel_response`].
///
/// The callback tier had no cancel at all: C has `nros_action_cancel_goal`,
/// Rust has `ActionClient::cancel_goal`, and C++ had only the L1 polling
/// tier's `PollingActionClient::send_cancel_request` — so a C++ application
/// written against the callback tier could start a goal it could not stop.
/// This is the same shape as C's: fire the request, read the outcome later.
///
/// # Safety
/// All pointers must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_cancel_goal(
    handle: *mut c_void,
    goal_id: *const [u8; 16],
) -> nros_cpp_ret_t {
    if handle.is_null() || goal_id.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let client = unsafe { &mut *(handle as *mut CppActionClient) };
    let id = nros::GoalId {
        uuid: unsafe { *goal_id },
    };
    let core = match unsafe { cpp_arena_core_mut(client.arena_entry_index, client.executor_ptr) } {
        Some(c) => c,
        None => return NROS_CPP_RET_ERROR,
    };
    match core.send_cancel_request(&id) {
        Ok(()) => NROS_CPP_RET_OK,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Drain the reply to a `nros_cpp_action_client_cancel_goal` (non-blocking).
///
/// Writes the `action_msgs/srv/CancelGoal` RETURN CODE (0 = Ok, 1 = Rejected,
/// 2 = UnknownGoal, 3 = GoalTerminated — `nros::CancelReturnCode`, not the
/// per-goal `nros::CancelResponse`; issue 0796) to `out_return_code`.
///
/// Returns `NROS_CPP_RET_OK` when a reply was consumed,
/// `NROS_CPP_RET_TRY_AGAIN` when none has arrived yet.
///
/// # Safety
/// All pointers must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_try_recv_cancel_response(
    handle: *mut c_void,
    out_return_code: *mut i8,
) -> nros_cpp_ret_t {
    if handle.is_null() || out_return_code.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let client = unsafe { &mut *(handle as *mut CppActionClient) };
    let core = match unsafe { cpp_arena_core_mut(client.arena_entry_index, client.executor_ptr) } {
        Some(c) => c,
        None => return NROS_CPP_RET_ERROR,
    };
    match core.try_recv_cancel_reply() {
        // [CDR_HDR(4)][return_code i8][goals_canceling…]
        Ok(Some(len)) if len > CDR_HEADER_LEN => {
            unsafe { *out_return_code = core.result_buffer_ref()[CDR_HEADER_LEN] as i8 };
            NROS_CPP_RET_OK
        }
        // A reply that carries no return_code byte is a malformed frame, not
        // "no reply yet" — issue 0223's rule: do not collapse a truncation
        // onto a plausible default.
        Ok(Some(_)) => NROS_CPP_RET_ERROR,
        Ok(None) => NROS_CPP_RET_TRY_AGAIN,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Register async callbacks on the action client.
///
/// # Safety
/// `handle` must be a valid action client storage. Function pointers
/// may be null (no callback for that event).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_set_callbacks(
    handle: *mut c_void,
    goal_response: Option<unsafe extern "C" fn(bool, *const [u8; 16], *mut c_void)>,
    feedback: Option<unsafe extern "C" fn(*const [u8; 16], *const u8, usize, *mut c_void)>,
    result: Option<unsafe extern "C" fn(*const [u8; 16], i32, *const u8, usize, *mut c_void)>,
    context: *mut c_void,
) -> nros_cpp_ret_t {
    if handle.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let client = unsafe { &mut *(handle as *mut CppActionClient) };
    client.callbacks.goal_response = goal_response;
    client.callbacks.feedback = feedback;
    client.callbacks.result = result;
    client.callbacks.context = context;
    NROS_CPP_RET_OK
}

/// Poll action client for pending replies (non-blocking).
///
/// Checks for goal acceptance reply, feedback, and result reply.
/// Invokes the corresponding callbacks registered via
/// `nros_cpp_action_client_set_callbacks`.
///
/// Call this in the spin loop after `nros_cpp_spin_once`.
///
/// # Safety
/// `handle` must be a valid action client storage.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_poll(handle: *mut c_void) -> nros_cpp_ret_t {
    if handle.is_null() {
        return NROS_CPP_RET_OK;
    }

    let client = unsafe { &mut *(handle as *mut CppActionClient) };

    // Read callbacks before borrowing the arena core (avoids borrow conflict)
    let goal_response_cb = client.callbacks.goal_response;
    let feedback_cb = client.callbacks.feedback;
    let result_cb = client.callbacks.result;
    let ctx = client.callbacks.context;
    let idx = client.arena_entry_index;
    let eptr = client.executor_ptr;

    let make_uuid = |counter: u64| -> [u8; 16] {
        let mut u = [0u8; 16];
        u[..8].copy_from_slice(&counter.to_le_bytes());
        u
    };

    let core = match unsafe { cpp_arena_core_mut(idx, eptr) } {
        Some(c) => c,
        None => return NROS_CPP_RET_OK,
    };

    // Poll goal acceptance reply
    if let Ok(Some(total_len)) = core.try_recv_send_goal_reply()
        && let Some(cb) = goal_response_cb
    {
        let buf = core.result_buffer_ref();
        let accepted = total_len >= 5 && buf[4] != 0;
        let uuid = make_uuid(core.goal_counter());
        unsafe { cb(accepted, &uuid, ctx) };
    }

    // Poll feedback
    if let Ok(Some((goal_id, total_len))) = core.try_recv_feedback_raw()
        && let Some(cb) = feedback_cb
    {
        // [CDR_HEADER(4)][GoalId.uuid(16)][feedback payload] — SEQ_PREFIX_LEN
        // is 0 since 233.6 (kept in the sum so the framing reads uniformly).
        // Issue #179 — deliver WITH a single encap header, mirroring the
        // arena dispatch: a typed server's payload already carries one
        // (`ffi_serialize`), a stripped-fields payload gets the reply's own
        // top-level encap spliced in front. The old verbatim pass-through
        // handed raw fields to `ffi_deserialize`, which ate the first data
        // word as the encap.
        const FEEDBACK_PAYLOAD_OFFSET: usize =
            CDR_HEADER_LEN + GoalId::SEQ_PREFIX_LEN + GoalId::UUID_LEN;
        if total_len > FEEDBACK_PAYLOAD_OFFSET {
            let buf = core.feedback_buffer_ref();
            let raw = &buf[FEEDBACK_PAYLOAD_OFFSET..total_len];
            if payload_has_cdr_encap(raw) {
                unsafe { cb(&goal_id.uuid, raw.as_ptr(), raw.len(), ctx) };
            } else {
                let mut spliced = [0u8; DEFAULT_RX_BUF_SIZE];
                let n = raw.len().min(DEFAULT_RX_BUF_SIZE - CDR_HEADER_LEN);
                spliced[..CDR_HEADER_LEN].copy_from_slice(&buf[..CDR_HEADER_LEN]);
                spliced[CDR_HEADER_LEN..CDR_HEADER_LEN + n].copy_from_slice(&raw[..n]);
                unsafe { cb(&goal_id.uuid, spliced.as_ptr(), CDR_HEADER_LEN + n, ctx) };
            }
        }
    }

    // Poll result reply
    //
    // Layout (`reply_get_result_from_slab`): [CDR_HEADER(4)][status(i8)]
    // [align(4) → byte 8][result payload]. Issue #179 — the old offset-5
    // slice handed the three alignment-pad bytes to the deserializer as the
    // start of the body (its comment cited the C wrapper's identical bug as
    // proof of correctness — two bugs cross-validating; phase-239's revert
    // of a prior 8-offset was measuring against that same broken pair).
    // Deliver WITH a single encap header, arena-style.
    const RESULT_PAYLOAD_OFFSET: usize = CDR_HEADER_LEN + 4;
    if let Ok(Some(total_len)) = core.try_recv_get_result_reply()
        && let Some(cb) = result_cb
        && total_len >= 5
    {
        let buf = core.result_buffer_ref();
        let status = buf[4] as i32;
        let uuid = make_uuid(core.goal_counter());
        let raw = &buf[RESULT_PAYLOAD_OFFSET.min(total_len)..total_len];
        if payload_has_cdr_encap(raw) {
            unsafe { cb(&uuid, status, raw.as_ptr(), raw.len(), ctx) };
        } else {
            let mut spliced = [0u8; DEFAULT_RX_BUF_SIZE];
            let n = raw.len().min(DEFAULT_RX_BUF_SIZE - CDR_HEADER_LEN);
            spliced[..CDR_HEADER_LEN].copy_from_slice(&buf[..CDR_HEADER_LEN]);
            spliced[CDR_HEADER_LEN..CDR_HEADER_LEN + n].copy_from_slice(&raw[..n]);
            unsafe { cb(&uuid, status, spliced.as_ptr(), CDR_HEADER_LEN + n, ctx) };
        }
    }

    NROS_CPP_RET_OK
}

// ============================================================================
// Phase 122.3.c.6.e — C-ABI wake-state slot. Same layout as
// `nros_c::service::nros_wake_state_t` (16 bytes / 2 u64). Caller
// declares one per (entity, channel) pair.
// ============================================================================

#[repr(C)]
pub struct nros_cpp_wake_state_t {
    pub _opaque: [u64; 2],
}

#[unsafe(no_mangle)]
pub extern "C" fn nros_cpp_wake_state_get_zero_initialized() -> nros_cpp_wake_state_t {
    nros_cpp_wake_state_t { _opaque: [0u64; 2] }
}

unsafe fn install_waker_on(
    state: *mut nros_cpp_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
    register: impl FnOnce(&core::task::Waker),
) -> nros_cpp_ret_t {
    if state.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let state_ptr = state as *mut nros_node::c_waker::CWakeState;
    unsafe {
        core::ptr::write(
            state_ptr,
            nros_node::c_waker::CWakeState { fn_ptr: cb, ctx },
        );
        let waker = nros_node::c_waker::make_waker(state_ptr);
        register(&waker);
    }
    NROS_CPP_RET_OK
}

/// Phase 122.3.c.6.e — wake on subscription rx.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_subscription_set_wake_callback(
    storage: *mut c_void,
    state: *mut nros_cpp_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    use nros_rmw::Subscription;
    let sub = unsafe { &*(storage as *const nros::internals::RmwSubscriber) };
    unsafe { install_waker_on(state, cb, ctx, |w| sub.register_waker(w)) }
}

/// Phase 122.3.c.6.e — wake on service-server rx.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_server_set_wake_callback(
    storage: *mut c_void,
    state: *mut nros_cpp_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    use nros_rmw::ServiceTrait;
    let srv = unsafe { &*(storage as *const nros::internals::RmwServiceServer) };
    unsafe { install_waker_on(state, cb, ctx, |w| srv.register_waker(w)) }
}

/// Phase 122.3.c.6.e — wake on service-client reply.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_client_set_wake_callback(
    storage: *mut c_void,
    state: *mut nros_cpp_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    use nros_rmw::ClientTrait;
    let cli = unsafe { &*(storage as *const nros::internals::RmwServiceClient) };
    unsafe { install_waker_on(state, cb, ctx, |w| cli.register_waker(w)) }
}

// Per-channel wake callbacks for L1 polling action server / client.
// `storage` is the POLLING `ActionServerCore` / `ActionClientCore`
// inline storage (i.e. what `_init_polling` wrote).

#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_set_goal_wake_callback(
    storage: *mut c_void,
    state: *mut nros_cpp_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &*(storage as *const PollingActionServerCore) };
    unsafe { install_waker_on(state, cb, ctx, |w| core.register_goal_waker(w)) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_set_cancel_wake_callback(
    storage: *mut c_void,
    state: *mut nros_cpp_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &*(storage as *const PollingActionServerCore) };
    unsafe { install_waker_on(state, cb, ctx, |w| core.register_cancel_waker(w)) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_set_get_result_wake_callback(
    storage: *mut c_void,
    state: *mut nros_cpp_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &*(storage as *const PollingActionServerCore) };
    unsafe { install_waker_on(state, cb, ctx, |w| core.register_get_result_waker(w)) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_set_goal_response_wake_callback(
    storage: *mut c_void,
    state: *mut nros_cpp_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &*(storage as *const PollingActionClientCore) };
    unsafe { install_waker_on(state, cb, ctx, |w| core.register_goal_response_waker(w)) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_set_cancel_response_wake_callback(
    storage: *mut c_void,
    state: *mut nros_cpp_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &*(storage as *const PollingActionClientCore) };
    unsafe { install_waker_on(state, cb, ctx, |w| core.register_cancel_response_waker(w)) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_set_result_wake_callback(
    storage: *mut c_void,
    state: *mut nros_cpp_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &*(storage as *const PollingActionClientCore) };
    unsafe { install_waker_on(state, cb, ctx, |w| core.register_result_waker(w)) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_set_feedback_wake_callback(
    storage: *mut c_void,
    state: *mut nros_cpp_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &*(storage as *const PollingActionClientCore) };
    unsafe { install_waker_on(state, cb, ctx, |w| core.register_feedback_waker(w)) }
}

// ============================================================================
// Phase 122.3.d — Layer-1 polling-mode FFI for action server / client
// ============================================================================
//
// Mirrors the nros-c L1 polling surface (122.3.c.6.b) for callers
// that drive their own scheduler. `_init_polling` writes the raw
// `ActionServerCore` / `ActionClientCore` into caller-provided
// inline storage; raw methods delegate to the core. Storage size:
// `NROS_CPP_RAW_ACTION_{SERVER,CLIENT}_OPAQUE_U64S` (× 8 bytes).
//
// The existing L2 callback path (`nros_cpp_action_{server,client}_create`
// + executor registration) stays — callers pick at construction time.

type PollingActionServerCore =
    nros_node::ActionServerCore<DEFAULT_RX_BUF_SIZE, DEFAULT_RX_BUF_SIZE, DEFAULT_RX_BUF_SIZE, 4>;

type PollingActionClientCore =
    nros_node::ActionClientCore<DEFAULT_RX_BUF_SIZE, DEFAULT_RX_BUF_SIZE, DEFAULT_RX_BUF_SIZE>;

// CDR framing bytes prefixing the goal payload in send_goal requests
// (CDR encapsulation header + GoalId sequence length prefix + UUID).
const POLLING_GOAL_REQUEST_FRAMING_LEN: usize =
    CDR_HEADER_LEN + GoalId::SEQ_PREFIX_LEN + GoalId::UUID_LEN;

/// Phase 122.3.d — initialize an L1 polling-mode action server.
///
/// Builds the 5 channels via the node's session and writes the
/// `ActionServerCore` into `storage` (must be at least
/// `NROS_CPP_RAW_ACTION_SERVER_OPAQUE_U64S × 8` bytes,
/// 8-byte-aligned).
///
/// # Safety
/// All pointers valid; `action_name` / `type_name` / `type_hash`
/// are valid null-terminated strings.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_init_polling(
    node: *const nros_cpp_node_t,
    action_name: *const c_char,
    type_name: *const c_char,
    type_hash: *const c_char,
    storage: *mut c_void,
) -> nros_cpp_ret_t {
    if node.is_null()
        || action_name.is_null()
        || type_name.is_null()
        || type_hash.is_null()
        || storage.is_null()
    {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let node_ref = unsafe { &*node };
    if node_ref.executor.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let action_str = match unsafe { cstr_to_str(action_name) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let type_str = match unsafe { cstr_to_str(type_name) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    // Issue 0312 — an empty hash would land in the liveliness keyexpr as an
    // empty segment, making the entity invisible to ROS 2 discovery.
    let hash_str = match unsafe { cstr_to_str(type_hash) } {
        Some(s) => crate::normalize_type_hash(s),
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let node_name_str = core::str::from_utf8(&node_ref.name)
        .ok()
        .and_then(|s| s.split('\0').next());
    let ns_str = core::str::from_utf8(&node_ref.namespace)
        .ok()
        .and_then(|s| s.split('\0').next())
        .unwrap_or("/");
    let ctx = unsafe { &mut *(node_ref.executor as *mut CppContext) };

    use nros_node::{ActionInfo, QoSProfile, ServiceInfo, Session, TopicInfo};
    let action_info = ActionInfo::new(action_str, type_str, hash_str).with_domain(ctx.domain_id);
    let session = ctx.executor.session_mut();

    fn with_node<'a>(info: ServiceInfo<'a>, node_name: Option<&'a str>) -> ServiceInfo<'a> {
        match node_name {
            Some(n) if !n.is_empty() => info.with_node_name(n),
            _ => info,
        }
    }

    // Issue 0454 / phase-354 W3 — per-CHANNEL DDS type, not the bare action
    // type. The type name is baked into the keyexpr, so `…Fibonacci_` where ROS
    // 2 expects `…Fibonacci_SendGoal_` makes query and queryable different keys
    // and every goal times out. Same defect phase-338 W3 fixed on the raw
    // register path; these polling arms had no caller to expose it.
    let send_goal_keyexpr: nros::heapless::String<256> = action_info.send_goal_key();
    let send_goal_type: nros::heapless::String<256> =
        nros::action_channel_type(type_str, "SendGoal");
    let send_goal_info = with_node(
        ServiceInfo::new(&send_goal_keyexpr, &send_goal_type, hash_str)
            .with_domain(ctx.domain_id)
            .with_namespace(ns_str),
        node_name_str,
    );
    let send_goal_server =
        match session.create_service(&send_goal_info, QoSProfile::services_default()) {
            Ok(h) => h,
            Err(_) => return NROS_CPP_RET_TRANSPORT_ERROR,
        };

    let cancel_goal_keyexpr: nros::heapless::String<256> = action_info.cancel_goal_key();
    let cancel_goal_info = with_node(
        ServiceInfo::new(
            &cancel_goal_keyexpr,
            "action_msgs::srv::dds_::CancelGoal_",
            hash_str,
        )
        .with_domain(ctx.domain_id)
        .with_namespace(ns_str),
        node_name_str,
    );
    let cancel_goal_server =
        match session.create_service(&cancel_goal_info, QoSProfile::services_default()) {
            Ok(h) => h,
            Err(_) => return NROS_CPP_RET_TRANSPORT_ERROR,
        };

    let get_result_keyexpr: nros::heapless::String<256> = action_info.get_result_key();
    let get_result_type: nros::heapless::String<256> =
        nros::action_channel_type(type_str, "GetResult");
    let get_result_info = with_node(
        ServiceInfo::new(&get_result_keyexpr, &get_result_type, hash_str)
            .with_domain(ctx.domain_id)
            .with_namespace(ns_str),
        node_name_str,
    );
    let get_result_server =
        match session.create_service(&get_result_info, QoSProfile::services_default()) {
            Ok(h) => h,
            Err(_) => return NROS_CPP_RET_TRANSPORT_ERROR,
        };

    let feedback_keyexpr: nros::heapless::String<256> = action_info.feedback_key();
    let feedback_type: nros::heapless::String<256> =
        nros::action_channel_type(type_str, "FeedbackMessage");
    let mut feedback_topic = TopicInfo::new(&feedback_keyexpr, &feedback_type, hash_str)
        .with_domain(ctx.domain_id)
        .with_namespace(ns_str);
    if let Some(n) = node_name_str
        && !n.is_empty()
    {
        feedback_topic = feedback_topic.with_node_name(n);
    }
    let feedback_publisher =
        match session.create_publisher(&feedback_topic, QoSProfile::BEST_EFFORT) {
            Ok(h) => h,
            Err(_) => return NROS_CPP_RET_TRANSPORT_ERROR,
        };

    let status_keyexpr: nros::heapless::String<256> = action_info.status_key();
    let mut status_topic = TopicInfo::new(
        &status_keyexpr,
        "action_msgs::msg::dds_::GoalStatusArray_",
        hash_str,
    )
    .with_domain(ctx.domain_id)
    .with_namespace(ns_str);
    if let Some(n) = node_name_str
        && !n.is_empty()
    {
        status_topic = status_topic.with_node_name(n);
    }
    let status_publisher = match session.create_publisher(&status_topic, QoSProfile::BEST_EFFORT) {
        Ok(h) => h,
        Err(_) => return NROS_CPP_RET_TRANSPORT_ERROR,
    };

    let core = PollingActionServerCore::from_channels(
        send_goal_server,
        cancel_goal_server,
        get_result_server,
        feedback_publisher,
        status_publisher,
    );
    unsafe {
        core::ptr::write(storage as *mut PollingActionServerCore, core);
    }
    NROS_CPP_RET_OK
}

/// Phase 122.3.d — L1 polling: try to receive a goal request.
///
/// On success copies the goal payload (CDR-framing-stripped) into
/// `buf`, fills `goal_id_out` (16 bytes) + `sequence_number_out`,
/// returns bytes copied. `0` means no request pending; negative on
/// error.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_try_recv_goal_request_raw(
    storage: *mut c_void,
    buf: *mut u8,
    buf_len: usize,
    goal_id_out: *mut [u8; 16],
    sequence_number_out: *mut i64,
) -> i32 {
    if storage.is_null()
        || (buf.is_null() && buf_len != 0)
        || goal_id_out.is_null()
        || sequence_number_out.is_null()
    {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionServerCore) };
    match core.try_recv_goal_request() {
        Ok(Some(req)) => {
            let goal_buf = core.goal_buffer();
            let payload_offset = POLLING_GOAL_REQUEST_FRAMING_LEN;
            if payload_offset + req.data_len > goal_buf.len() {
                return NROS_CPP_RET_ERROR;
            }
            let copy_len = req.data_len.min(buf_len);
            unsafe {
                core::ptr::copy_nonoverlapping(
                    goal_buf.as_ptr().add(payload_offset),
                    buf,
                    copy_len,
                );
                (*goal_id_out).copy_from_slice(&req.goal_id.uuid);
                *sequence_number_out = req.sequence_number;
            }
            copy_len as i32
        }
        Ok(None) => 0,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.d — L1 polling: accept a goal received via
/// `try_recv_goal_request_raw`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_accept_goal_raw(
    storage: *mut c_void,
    goal_id: *const [u8; 16],
    sequence_number: i64,
) -> nros_cpp_ret_t {
    if storage.is_null() || goal_id.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionServerCore) };
    let id = GoalId {
        uuid: unsafe { *goal_id },
    };
    match core.accept_goal(id, sequence_number) {
        Ok(_) => NROS_CPP_RET_OK,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.d — L1 polling: reject a goal.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_reject_goal_raw(
    storage: *mut c_void,
    sequence_number: i64,
) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionServerCore) };
    match core.reject_goal(sequence_number) {
        Ok(_) => NROS_CPP_RET_OK,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.d — L1 polling: publish feedback.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_publish_feedback_raw(
    storage: *mut c_void,
    goal_id: *const [u8; 16],
    feedback_cdr: *const u8,
    feedback_len: usize,
) -> nros_cpp_ret_t {
    if storage.is_null() || goal_id.is_null() || (feedback_cdr.is_null() && feedback_len != 0) {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionServerCore) };
    let id = GoalId {
        uuid: unsafe { *goal_id },
    };
    let slice = unsafe { core::slice::from_raw_parts(feedback_cdr, feedback_len) };
    match core.publish_feedback_raw(&id, slice) {
        Ok(_) => NROS_CPP_RET_OK,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.d — L1 polling: mark a goal terminal.
/// `status_code`: 4 = Succeeded, 5 = Canceled, 6 = Aborted (matches
/// the `nros_core::GoalStatus` discriminants).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_complete_goal_raw(
    storage: *mut c_void,
    goal_id: *const [u8; 16],
    status_code: i32,
    result_cdr: *const u8,
    result_len: usize,
) -> nros_cpp_ret_t {
    if storage.is_null() || goal_id.is_null() || (result_cdr.is_null() && result_len != 0) {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionServerCore) };
    let id = GoalId {
        uuid: unsafe { *goal_id },
    };
    let slice = unsafe { core::slice::from_raw_parts(result_cdr, result_len) };
    let status = match status_code {
        4 => nros::GoalStatus::Succeeded,
        5 => nros::GoalStatus::Canceled,
        6 => nros::GoalStatus::Aborted,
        _ => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    // issue 0796 — see `nros_cpp_action_server_complete_goal`.
    match core.complete_goal_raw(&id, status, slice) {
        Ok(()) => NROS_CPP_RET_OK,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.c.6.d / .d — L1 polling: peek a pending
/// cancel-goal request. Writes goal_id + sequence_number +
/// current_status (matches `nros_cpp_goal_status_t` discriminants).
/// Returns `1` on peek, `0` if none pending, negative on error.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_try_recv_cancel_request_raw(
    storage: *mut c_void,
    goal_id_out: *mut [u8; 16],
    sequence_number_out: *mut i64,
    current_status_out: *mut i8,
) -> i32 {
    if storage.is_null()
        || goal_id_out.is_null()
        || sequence_number_out.is_null()
        || current_status_out.is_null()
    {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionServerCore) };
    match core.try_recv_cancel_request() {
        Ok(Some(req)) => {
            unsafe {
                (*goal_id_out).copy_from_slice(&req.goal_id.uuid);
                *sequence_number_out = req.sequence_number;
                *current_status_out = req.current_status as i8;
            }
            1
        }
        Ok(None) => 0,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.c.6.d / .d — L1 polling: reply to a cancel-goal
/// request. `return_code` matches `nros_core::CancelReturnCode`
/// (0 = Ok, 1 = Rejected, 2 = UnknownGoal, 3 = GoalTerminated) — the
/// `action_msgs/srv/CancelGoal` RPC status, NOT the per-goal
/// `nros::CancelResponse` accept/reject a cancel callback returns (issue
/// 0796; the two used to share a name and their discriminants overlap with
/// opposite meanings).
/// `accepted` points to `accepted_count` 16-byte goal-id arrays.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_send_cancel_reply_raw(
    storage: *mut c_void,
    sequence_number: i64,
    return_code: i8,
    accepted: *const [u8; 16],
    accepted_count: usize,
) -> nros_cpp_ret_t {
    if storage.is_null() || (accepted.is_null() && accepted_count != 0) {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionServerCore) };
    let resp = match return_code {
        0 => nros_node::CancelReturnCode::Ok,
        1 => nros_node::CancelReturnCode::Rejected,
        2 => nros_node::CancelReturnCode::UnknownGoal,
        3 => nros_node::CancelReturnCode::GoalTerminated,
        _ => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let mut ids: nros::heapless::Vec<nros::GoalId, 8> = nros::heapless::Vec::new();
    for i in 0..accepted_count {
        if i >= 8 {
            return NROS_CPP_RET_INVALID_ARGUMENT;
        }
        let uuid = unsafe { *accepted.add(i) };
        if ids.push(nros::GoalId { uuid }).is_err() {
            return NROS_CPP_RET_ERROR;
        }
    }
    match core.send_cancel_reply(sequence_number, resp, &ids) {
        Ok(()) => NROS_CPP_RET_OK,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.d — L1 polling: serve a pending get_result query.
/// Returns `1` if served, `0` if none pending, negative on error.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_try_handle_get_result_raw(
    storage: *mut c_void,
    default_result_cdr: *const u8,
    default_result_len: usize,
) -> i32 {
    if storage.is_null() || (default_result_cdr.is_null() && default_result_len != 0) {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionServerCore) };
    let data = unsafe { core::slice::from_raw_parts(default_result_cdr, default_result_len) };
    let slice = strip_cdr_header(data);
    match core.try_handle_get_result_raw(slice) {
        Ok(Some(_)) => 1,
        Ok(None) => 0,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.d — L1 polling: drop the inline `ActionServerCore`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_server_destroy_polling(
    storage: *mut c_void,
) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_OK;
    }
    unsafe {
        core::ptr::drop_in_place(storage as *mut PollingActionServerCore);
    }
    NROS_CPP_RET_OK
}

// ----------------------------------------------------------------------------
// Action client L1 polling
// ----------------------------------------------------------------------------

/// Phase 122.3.d — initialize an L1 polling-mode action client.
///
/// Builds the 3 service clients + feedback subscriber via the
/// session and writes the `ActionClientCore` into `storage`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_init_polling(
    node: *const nros_cpp_node_t,
    action_name: *const c_char,
    type_name: *const c_char,
    type_hash: *const c_char,
    storage: *mut c_void,
) -> nros_cpp_ret_t {
    if node.is_null()
        || action_name.is_null()
        || type_name.is_null()
        || type_hash.is_null()
        || storage.is_null()
    {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let node_ref = unsafe { &*node };
    if node_ref.executor.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let action_str = match unsafe { cstr_to_str(action_name) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let type_str = match unsafe { cstr_to_str(type_name) } {
        Some(s) => s,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    // Issue 0312 — an empty hash would land in the liveliness keyexpr as an
    // empty segment, making the entity invisible to ROS 2 discovery.
    let hash_str = match unsafe { cstr_to_str(type_hash) } {
        Some(s) => crate::normalize_type_hash(s),
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let node_name_str = core::str::from_utf8(&node_ref.name)
        .ok()
        .and_then(|s| s.split('\0').next());
    let ns_str = core::str::from_utf8(&node_ref.namespace)
        .ok()
        .and_then(|s| s.split('\0').next())
        .unwrap_or("/");
    let ctx = unsafe { &mut *(node_ref.executor as *mut CppContext) };

    use nros_node::{ActionInfo, QoSProfile, ServiceInfo, Session, TopicInfo};
    let action_info = ActionInfo::new(action_str, type_str, hash_str).with_domain(ctx.domain_id);
    let session = ctx.executor.session_mut();

    fn with_node<'a>(info: ServiceInfo<'a>, node_name: Option<&'a str>) -> ServiceInfo<'a> {
        match node_name {
            Some(n) if !n.is_empty() => info.with_node_name(n),
            _ => info,
        }
    }

    // Issue 0454 / phase-354 W3 — per-CHANNEL DDS type, not the bare action
    // type. The type name is baked into the keyexpr, so `…Fibonacci_` where ROS
    // 2 expects `…Fibonacci_SendGoal_` makes query and queryable different keys
    // and every goal times out. Same defect phase-338 W3 fixed on the raw
    // register path; these polling arms had no caller to expose it.
    let send_goal_keyexpr: nros::heapless::String<256> = action_info.send_goal_key();
    let send_goal_type: nros::heapless::String<256> =
        nros::action_channel_type(type_str, "SendGoal");
    let send_goal_info = with_node(
        ServiceInfo::new(&send_goal_keyexpr, &send_goal_type, hash_str)
            .with_domain(ctx.domain_id)
            .with_namespace(ns_str),
        node_name_str,
    );
    let send_goal_client =
        match session.create_client(&send_goal_info, QoSProfile::services_default()) {
            Ok(h) => h,
            Err(_) => return NROS_CPP_RET_TRANSPORT_ERROR,
        };

    let cancel_goal_keyexpr: nros::heapless::String<256> = action_info.cancel_goal_key();
    let cancel_goal_info = with_node(
        ServiceInfo::new(
            &cancel_goal_keyexpr,
            "action_msgs::srv::dds_::CancelGoal_",
            hash_str,
        )
        .with_domain(ctx.domain_id)
        .with_namespace(ns_str),
        node_name_str,
    );
    let cancel_goal_client =
        match session.create_client(&cancel_goal_info, QoSProfile::services_default()) {
            Ok(h) => h,
            Err(_) => return NROS_CPP_RET_TRANSPORT_ERROR,
        };

    let get_result_keyexpr: nros::heapless::String<256> = action_info.get_result_key();
    let get_result_type: nros::heapless::String<256> =
        nros::action_channel_type(type_str, "GetResult");
    let get_result_info = with_node(
        ServiceInfo::new(&get_result_keyexpr, &get_result_type, hash_str)
            .with_domain(ctx.domain_id)
            .with_namespace(ns_str),
        node_name_str,
    );
    let get_result_client =
        match session.create_client(&get_result_info, QoSProfile::services_default()) {
            Ok(h) => h,
            Err(_) => return NROS_CPP_RET_TRANSPORT_ERROR,
        };

    let feedback_keyexpr: nros::heapless::String<256> = action_info.feedback_key();
    let feedback_type: nros::heapless::String<256> =
        nros::action_channel_type(type_str, "FeedbackMessage");
    let mut feedback_topic = TopicInfo::new(&feedback_keyexpr, &feedback_type, hash_str)
        .with_domain(ctx.domain_id)
        .with_namespace(ns_str);
    if let Some(n) = node_name_str
        && !n.is_empty()
    {
        feedback_topic = feedback_topic.with_node_name(n);
    }
    let feedback_subscriber =
        match session.create_subscription(&feedback_topic, QoSProfile::BEST_EFFORT) {
            Ok(h) => h,
            Err(_) => return NROS_CPP_RET_TRANSPORT_ERROR,
        };

    let core = PollingActionClientCore::new(
        send_goal_client,
        cancel_goal_client,
        get_result_client,
        feedback_subscriber,
    );
    unsafe {
        core::ptr::write(storage as *mut PollingActionClientCore, core);
    }
    NROS_CPP_RET_OK
}

/// Phase 122.3.d — L1 polling: send a goal. Writes 16-byte UUID
/// into `goal_id_out`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_send_goal_raw(
    storage: *mut c_void,
    goal_cdr: *const u8,
    goal_len: usize,
    goal_id_out: *mut [u8; 16],
) -> nros_cpp_ret_t {
    if storage.is_null() || goal_id_out.is_null() || (goal_cdr.is_null() && goal_len != 0) {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionClientCore) };
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
    let slice = unsafe { core::slice::from_raw_parts(goal_cdr, goal_len) };
    match core.send_goal_raw(strip_cdr_header(slice)) {
        Ok(id) => {
            unsafe {
                (*goal_id_out).copy_from_slice(&id.uuid);
            }
            NROS_CPP_RET_OK
        }
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.d — L1 polling: try receiving the send_goal RPC reply.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_try_recv_goal_response_raw(
    storage: *mut c_void,
    buf: *mut u8,
    buf_len: usize,
) -> i32 {
    if storage.is_null() || (buf.is_null() && buf_len != 0) {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionClientCore) };
    match core.try_recv_send_goal_reply() {
        Ok(Some(len)) => {
            let copy_len = len.min(buf_len);
            unsafe {
                core::ptr::copy_nonoverlapping(core.result_buffer_ref().as_ptr(), buf, copy_len);
            }
            copy_len as i32
        }
        Ok(None) => 0,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.d — L1 polling: send a get_result request.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_send_get_result_request_raw(
    storage: *mut c_void,
    goal_id: *const [u8; 16],
) -> nros_cpp_ret_t {
    if storage.is_null() || goal_id.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionClientCore) };
    let id = GoalId {
        uuid: unsafe { *goal_id },
    };
    match core.send_get_result_request(&id) {
        Ok(_) => NROS_CPP_RET_OK,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.d — L1 polling: try receiving the get_result reply.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_try_recv_result_raw(
    storage: *mut c_void,
    buf: *mut u8,
    buf_len: usize,
) -> i32 {
    if storage.is_null() || (buf.is_null() && buf_len != 0) {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionClientCore) };
    match core.try_recv_get_result_reply() {
        Ok(Some(len)) => {
            let copy_len = len.min(buf_len);
            unsafe {
                core::ptr::copy_nonoverlapping(core.result_buffer_ref().as_ptr(), buf, copy_len);
            }
            copy_len as i32
        }
        Ok(None) => 0,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.d — L1 polling: send a cancel request.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_send_cancel_request_raw(
    storage: *mut c_void,
    goal_id: *const [u8; 16],
) -> nros_cpp_ret_t {
    if storage.is_null() || goal_id.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionClientCore) };
    let id = GoalId {
        uuid: unsafe { *goal_id },
    };
    match core.send_cancel_request(&id) {
        Ok(_) => NROS_CPP_RET_OK,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.d / .c.6.c — L1 polling: try receiving the cancel
/// RPC reply.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_try_recv_cancel_response_raw(
    storage: *mut c_void,
    buf: *mut u8,
    buf_len: usize,
) -> i32 {
    if storage.is_null() || (buf.is_null() && buf_len != 0) {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionClientCore) };
    match core.try_recv_cancel_reply() {
        Ok(Some(len)) => {
            let copy_len = len.min(buf_len);
            unsafe {
                core::ptr::copy_nonoverlapping(core.result_buffer_ref().as_ptr(), buf, copy_len);
            }
            copy_len as i32
        }
        Ok(None) => 0,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.d — L1 polling: try receiving feedback. Writes
/// goal_id_out + bytes copied.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_try_recv_feedback_raw(
    storage: *mut c_void,
    buf: *mut u8,
    buf_len: usize,
    goal_id_out: *mut [u8; 16],
) -> i32 {
    if storage.is_null() || (buf.is_null() && buf_len != 0) || goal_id_out.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let core = unsafe { &mut *(storage as *mut PollingActionClientCore) };
    match core.try_recv_feedback_raw() {
        Ok(Some((id, len))) => {
            let copy_len = len.min(buf_len);
            unsafe {
                core::ptr::copy_nonoverlapping(core.feedback_buffer_ref().as_ptr(), buf, copy_len);
                (*goal_id_out).copy_from_slice(&id.uuid);
            }
            copy_len as i32
        }
        Ok(None) => 0,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Phase 122.3.d — L1 polling: drop the inline `ActionClientCore`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_action_client_destroy_polling(
    storage: *mut c_void,
) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_OK;
    }
    unsafe {
        core::ptr::drop_in_place(storage as *mut PollingActionClientCore);
    }
    NROS_CPP_RET_OK
}
