//! Service server and client FFI functions for the C++ API.
//!
//! Phase 87.6 (thin-wrapper refactor): caller's opaque storage holds a bare
//! `RmwServiceServer` / `RmwServiceClient` handle. Service-name buffers live
//! on the C++ `nros::Service<S>` / `nros::Client<S>` classes. Received CDR
//! bytes are copied directly into caller-provided output buffers — no
//! runtime scratch.
//!
//! Issue 0436 — user-supplied executor handles are tag-validated via
//! `cpp_ctx_checked`. Handles reached through an already-validated node/client
//! (`node_ref.executor`) inherit that provenance and stay direct casts.

use core::ffi::{c_char, c_void};

use nros_rmw::{ClientTrait, ServiceInfo, ServiceTrait, Session};

use crate::{
    CppContext, NROS_CPP_RET_ERROR, NROS_CPP_RET_INVALID_ARGUMENT, NROS_CPP_RET_OK,
    NROS_CPP_RET_TIMEOUT, NROS_CPP_RET_TRY_AGAIN, cpp_ctx_checked, cstr_to_str, nros_cpp_node_t,
    nros_cpp_qos_t, nros_cpp_ret_t,
};

use core::{
    sync::atomic::AtomicBool,
    task::{RawWaker, RawWakerVTable, Waker},
};

/// Build a `Waker` that sets `flag` to `true` when woken — the transport
/// signals reply-arrival into the arena entry's `reply_ready` slot. Mirrors
/// `nros_c::service::atomic_bool_waker`; see that function for the full
/// # Safety contract (lifetime / unregister-before-free / wake races). The
/// only callsite here borrows `&entry.reply_ready` from the executor's
/// service-client arena, which is heap-allocated at session-open and never
/// recycled until the executor is destroyed; `register_waker` is a
/// single-slot overwrite. Both contracts hold by construction.
unsafe fn atomic_bool_waker(flag: &AtomicBool) -> Waker {
    static VTABLE: RawWakerVTable = RawWakerVTable::new(
        |data| RawWaker::new(data, &VTABLE),
        |data| unsafe {
            (*(data as *const AtomicBool)).store(true, core::sync::atomic::Ordering::Release);
        },
        |data| unsafe {
            (*(data as *const AtomicBool)).store(true, core::sync::atomic::Ordering::Release);
        },
        |_data| {},
    );
    let raw = RawWaker::new(flag as *const AtomicBool as *const (), &VTABLE);
    // SAFETY: the vtable is valid; the caller asserts `flag` outlives the
    // returned Waker + clones (see the doc-comment).
    unsafe { Waker::from_raw(raw) }
}

// ============================================================================
// Service Server
// ============================================================================

/// Create a service server on a node.
///
/// The caller provides `storage` — a pointer to a buffer of at least
/// `size_of::<RmwServiceServer>()` bytes (exposed via `NROS_SERVICE_SERVER_SIZE`).
///
/// # Safety
/// All pointer parameters must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_server_create(
    node: *const nros_cpp_node_t,
    service_name: *const c_char,
    type_name: *const c_char,
    type_hash: *const c_char,
    qos: nros_cpp_qos_t,
    storage: *mut c_void,
) -> nros_cpp_ret_t {
    if node.is_null()
        || service_name.is_null()
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

    let svc_str = match unsafe { cstr_to_str(service_name) } {
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

    // Phase 305 W3 (issue 0255) — resolve `~`/relative names + launch remaps.
    let resolved_svc = match crate::resolve_node_entity_name(ctx, node_ref, svc_str) {
        Ok(r) => r,
        Err(()) => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let svc_str = resolved_svc.as_str();

    let mut svc_info = ServiceInfo::new(svc_str, type_str, hash_str)
        .with_domain(ctx.domain_id)
        .with_namespace(ns_str);
    if let Some(name) = node_name_str
        && !name.is_empty()
    {
        svc_info = svc_info.with_node_name(name);
    }

    // Phase 104.C.9.b — route through the Node's session when bound
    // to a non-primary RMW backend.
    let session = match crate::node_id_opt(node_ref) {
        Some(id) => match ctx.executor.node_session_mut(id) {
            Some(s) => s,
            None => return NROS_CPP_RET_INVALID_ARGUMENT,
        },
        None => ctx.executor.session_mut(),
    };

    // Phase 193.3 — apply the caller's QoS (rclcpp `create_service(name, qos)`).
    match session.create_service(&svc_info, qos.to_qos_settings()) {
        Ok(handle) => {
            unsafe {
                core::ptr::write(storage as *mut nros::internals::RmwServiceServer, handle);
            }
            NROS_CPP_RET_OK
        }
        Err(e) => crate::transport_error_to_cpp_ret(e),
    }
}

/// Phase 189.M3.3.e — register a **callback-style** service server in the
/// executor arena (rclcpp dispatch model), as opposed to the poll-style
/// `nros_cpp_service_server_create` above. The arena owns the server; spin
/// dispatches `callback(request_cdr) → reply_cdr`. Returns the executor
/// `HandleId` via `out_handle_id` (for cancel / introspection) and, when
/// `sched_context != 0`, binds that handle to the scheduling context — the
/// payoff this path unlocks (poll-style services have no dispatched callback to
/// schedule). `callback` is the C++ template's raw trampoline; `context` is the
/// `nros::Service<S>` object (`this`).
///
/// # Safety
/// All non-NULL pointers must be valid; `callback` must be a valid trampoline;
/// `context` outlives the executor (no move after register).
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn nros_cpp_service_server_register(
    node: *const nros_cpp_node_t,
    service_name: *const c_char,
    type_name: *const c_char,
    type_hash: *const c_char,
    qos: nros_cpp_qos_t,
    callback: nros_node::RawServiceCallback,
    context: *mut c_void,
    sched_context: u8,
    out_handle_id: *mut usize,
) -> nros_cpp_ret_t {
    if node.is_null()
        || service_name.is_null()
        || type_name.is_null()
        || type_hash.is_null()
        || out_handle_id.is_null()
    {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let node_ref = unsafe { &*node };
    if node_ref.executor.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let svc_str = match unsafe { cstr_to_str(service_name) } {
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

    // The arena registration derives namespace + node name from the node
    // record (register_service_raw_sized_on), so only the bare service name is
    // passed here — unlike the poll-style create which builds ServiceInfo itself.
    let ctx = unsafe { &mut *(node_ref.executor as *mut CppContext) };

    // Phase 305 W3 (issue 0255) — resolve `~`/relative names + launch remaps.
    let resolved_svc = match crate::resolve_node_entity_name(ctx, node_ref, svc_str) {
        Ok(r) => r,
        Err(()) => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let svc_str = resolved_svc.as_str();

    use nros_node::config::DEFAULT_RX_BUF_SIZE as BUF;
    let result = if crate::node_id_opt(node_ref).is_some() {
        ctx.executor.register_service_raw_sized_on::<BUF, BUF>(
            crate::node_id_opt(node_ref).expect("guarded above"),
            svc_str,
            type_str,
            hash_str,
            qos.to_qos_settings(),
            callback,
            context,
        )
    } else {
        ctx.executor.register_service_raw_sized::<BUF, BUF>(
            svc_str,
            type_str,
            hash_str,
            qos.to_qos_settings(),
            callback,
            context,
        )
    };

    match result {
        Ok(handle_id) => {
            // Phase 189.M3.3.e — bind the (now-real) arena handle to the
            // requested sched context. `0` = inherit (no-op); unknown slot fails.
            if sched_context != 0 {
                let sc_id = nros_node::executor::sched_context::SchedContextId(sched_context);
                if ctx
                    .executor
                    .bind_handle_to_sched_context(handle_id, sc_id)
                    .is_err()
                {
                    return NROS_CPP_RET_INVALID_ARGUMENT;
                }
            }
            unsafe {
                *out_handle_id = handle_id.0;
            }
            NROS_CPP_RET_OK
        }
        Err(e) => crate::node_error_to_cpp_ret(e),
    }
}

/// Try to receive a raw service request (non-blocking).
///
/// Writes the received CDR bytes directly into the caller's output buffer —
/// no runtime scratch.
///
/// # Safety
/// All pointers must be valid. `out_data` must point to `out_capacity` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_server_take_request_raw(
    storage: *mut c_void,
    out_data: *mut u8,
    out_capacity: usize,
    out_len: *mut usize,
    out_sequence: *mut i64,
) -> nros_cpp_ret_t {
    if storage.is_null() || out_data.is_null() || out_len.is_null() || out_sequence.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let server = unsafe { &mut *(storage as *mut nros::internals::RmwServiceServer) };
    let out_slice = unsafe { core::slice::from_raw_parts_mut(out_data, out_capacity) };

    match server.take_request(out_slice) {
        Ok(Some(request)) => {
            unsafe {
                *out_len = request.data.len();
                *out_sequence = request.sequence_number;
            }
            NROS_CPP_RET_OK
        }
        Ok(None) => {
            unsafe {
                *out_len = 0;
            }
            NROS_CPP_RET_OK
        }
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Send a raw reply to a service request.
///
/// # Safety
/// `storage` must be valid. `data` must point to `len` readable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_server_send_response_raw(
    storage: *mut c_void,
    sequence_number: i64,
    data: *const u8,
    len: usize,
) -> nros_cpp_ret_t {
    if storage.is_null() || data.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let server = unsafe { &mut *(storage as *mut nros::internals::RmwServiceServer) };
    let data_slice = unsafe { core::slice::from_raw_parts(data, len) };

    match server.send_response(sequence_number, data_slice) {
        Ok(_seq) => NROS_CPP_RET_OK,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Destroy a service server (drop in place, no free).
///
/// # Safety
/// `storage` must be a valid initialized service server storage, or NULL (no-op).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_server_destroy(storage: *mut c_void) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_OK;
    }
    unsafe {
        core::ptr::drop_in_place(storage as *mut nros::internals::RmwServiceServer);
    }
    NROS_CPP_RET_OK
}

/// Relocate an `RmwServiceServer` from `old_storage` to `new_storage`.
///
/// # Safety
/// See `nros_cpp_publisher_relocate`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_server_relocate(
    old_storage: *mut c_void,
    new_storage: *mut c_void,
) -> nros_cpp_ret_t {
    if old_storage.is_null() || new_storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    unsafe {
        let value = core::ptr::read(old_storage as *mut nros::internals::RmwServiceServer);
        core::ptr::write(new_storage as *mut nros::internals::RmwServiceServer, value);
    }
    NROS_CPP_RET_OK
}

// ============================================================================
// Service Client
// ============================================================================

/// Create a service client on a node.
///
/// The caller provides `storage` — a pointer to a buffer of at least
/// `size_of::<RmwServiceClient>()` bytes (exposed via `NROS_SERVICE_CLIENT_SIZE`).
///
/// # Safety
/// All pointer parameters must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_client_create(
    node: *const nros_cpp_node_t,
    service_name: *const c_char,
    type_name: *const c_char,
    type_hash: *const c_char,
    qos: nros_cpp_qos_t,
    storage: *mut c_void,
) -> nros_cpp_ret_t {
    if node.is_null()
        || service_name.is_null()
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

    let svc_str = match unsafe { cstr_to_str(service_name) } {
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

    // Phase 305 W3 (issue 0255) — resolve `~`/relative names + launch remaps.
    let resolved_svc = match crate::resolve_node_entity_name(ctx, node_ref, svc_str) {
        Ok(r) => r,
        Err(()) => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let svc_str = resolved_svc.as_str();

    let mut svc_info = ServiceInfo::new(svc_str, type_str, hash_str)
        .with_domain(ctx.domain_id)
        .with_namespace(ns_str);
    if let Some(name) = node_name_str
        && !name.is_empty()
    {
        svc_info = svc_info.with_node_name(name);
    }

    // Phase 104.C.9.b — route through the Node's session.
    let session = match crate::node_id_opt(node_ref) {
        Some(id) => match ctx.executor.node_session_mut(id) {
            Some(s) => s,
            None => return NROS_CPP_RET_INVALID_ARGUMENT,
        },
        None => ctx.executor.session_mut(),
    };

    // Phase 193.3 — apply the caller's QoS.
    match session.create_client(&svc_info, qos.to_qos_settings()) {
        Ok(handle) => {
            unsafe {
                core::ptr::write(storage as *mut nros::internals::RmwServiceClient, handle);
            }
            NROS_CPP_RET_OK
        }
        Err(e) => crate::transport_error_to_cpp_ret(e),
    }
}

/// Send a service request and block for reply (raw CDR).
///
/// # Safety
/// All pointers must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_client_call_raw(
    storage: *mut c_void,
    req_data: *const u8,
    req_len: usize,
    resp_data: *mut u8,
    resp_capacity: usize,
    resp_len: *mut usize,
    timeout_ms: u32,
) -> nros_cpp_ret_t {
    if storage.is_null() || req_data.is_null() || resp_data.is_null() || resp_len.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let client = unsafe { &mut *(storage as *mut nros::internals::RmwServiceClient) };
    let req_slice = unsafe { core::slice::from_raw_parts(req_data, req_len) };
    let resp_slice = unsafe { core::slice::from_raw_parts_mut(resp_data, resp_capacity) };

    // Bounded blocking call that NEVER drives the executor (issue 0278 Half B):
    // send, then poll `take_response_raw` sleeping `nros_platform_sleep_ms`
    // between attempts until `timeout_ms` elapses. Because it does not call
    // `spin_once`, it is safe to invoke from inside a subscription/timer
    // callback (which runs while the executor is exclusively borrowed) — on a
    // MULTI-THREADED backend the reply is delivered into the client's queue by
    // the backend's own read task, so this loop only reads that queue and
    // yields. On a single-threaded/polled backend the reply can only arrive via
    // `spin_once` (which the callback is blocking), so the call times out there
    // — use the executor-driven `Client::call` from the main loop instead.
    // phase-301 deleted the RMW call_raw slot, so this composes the async pair.
    use nros_rmw::ClientTrait as _;
    unsafe extern "C" {
        // Portable platform sleep (nros-platform-api platform.h) — linked
        // into every image via the active platform impl.
        fn nros_platform_sleep_ms(ms: usize);
    }
    if client.send_request_raw(req_slice).is_err() {
        return NROS_CPP_RET_TIMEOUT;
    }
    const STEP_MS: u32 = 5;
    let mut waited_ms: u32 = 0;
    loop {
        match client.take_response_raw(resp_slice) {
            // Issue 0778 — one call in flight on this blocking path.
            Ok(Some((len, _seq))) => {
                unsafe {
                    *resp_len = len;
                }
                return NROS_CPP_RET_OK;
            }
            Ok(None) => {
                if waited_ms >= timeout_ms {
                    return NROS_CPP_RET_TIMEOUT;
                }
                unsafe { nros_platform_sleep_ms(STEP_MS as usize) };
                waited_ms += STEP_MS;
            }
            Err(_) => return NROS_CPP_RET_TIMEOUT,
        }
    }
}

/// Phase 189.M3.3.f — register a **callback-style** service client in the
/// executor arena (rclcpp async dispatch), as opposed to the poll/future-style
/// `nros_cpp_service_client_create`. The arena owns the client; spin dispatches
/// `callback(response_cdr)` when a reply arrives. Requests are sent via
/// `nros_cpp_service_client_send_on_handle` using the returned `out_handle_id`.
/// Binds the handle to `sched_context` when non-zero (the payoff). `callback` is
/// the C++ template's raw response trampoline; `context` is the `Client<S>`
/// object (`this`).
///
/// # Safety
/// All non-NULL pointers valid; `callback` a valid trampoline; `context`
/// outlives the executor (no move after register).
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn nros_cpp_service_client_register(
    node: *const nros_cpp_node_t,
    service_name: *const c_char,
    type_name: *const c_char,
    type_hash: *const c_char,
    qos: nros_cpp_qos_t,
    callback: nros_node::RawResponseCallback,
    context: *mut c_void,
    sched_context: u8,
    out_handle_id: *mut usize,
) -> nros_cpp_ret_t {
    if node.is_null()
        || service_name.is_null()
        || type_name.is_null()
        || type_hash.is_null()
        || out_handle_id.is_null()
    {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let node_ref = unsafe { &*node };
    if node_ref.executor.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }

    let svc_str = match unsafe { cstr_to_str(service_name) } {
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
    let resolved_svc = match crate::resolve_node_entity_name(ctx, node_ref, svc_str) {
        Ok(r) => r,
        Err(()) => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    let svc_str = resolved_svc.as_str();

    use nros_node::config::DEFAULT_RX_BUF_SIZE as BUF;
    let result = if crate::node_id_opt(node_ref).is_some() {
        ctx.executor.register_service_client_raw_sized_on::<BUF>(
            crate::node_id_opt(node_ref).expect("guarded above"),
            svc_str,
            type_str,
            hash_str,
            qos.to_qos_settings(),
            Some(callback),
            context,
        )
    } else {
        ctx.executor.register_service_client_raw_sized::<BUF>(
            svc_str,
            type_str,
            hash_str,
            qos.to_qos_settings(),
            Some(callback),
            context,
        )
    };

    match result {
        Ok(handle_id) => {
            if sched_context != 0 {
                let sc_id = nros_node::executor::sched_context::SchedContextId(sched_context);
                if ctx
                    .executor
                    .bind_handle_to_sched_context(handle_id, sc_id)
                    .is_err()
                {
                    return NROS_CPP_RET_INVALID_ARGUMENT;
                }
            }
            unsafe {
                *out_handle_id = handle_id.0;
            }
            NROS_CPP_RET_OK
        }
        Err(e) => crate::node_error_to_cpp_ret(e),
    }
}

/// Phase 189.M3.3.f — send a request on a callback-style (arena-registered)
/// service client identified by `handle_id` (from
/// `nros_cpp_service_client_register`). The reply is delivered to the client's
/// registered response callback during spin. Mirrors the C arena send path.
///
/// # Safety
/// `executor_handle` must point to a valid `CppContext`; `req_data` to `req_len`
/// readable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_client_send_on_handle(
    executor_handle: *mut c_void,
    handle_id: usize,
    req_data: *const u8,
    req_len: usize,
) -> nros_cpp_ret_t {
    if req_data.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let Some(ctx) = (unsafe { cpp_ctx_checked(executor_handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let entry = match unsafe { ctx.executor.service_client_entry_mut(handle_id) } {
        Some(e) => e,
        None => return NROS_CPP_RET_INVALID_ARGUMENT,
    };
    // Single in-flight request per entry: refuse a second send while a reply
    // is still outstanding (the dispatcher gates on `pending`).
    if entry.pending {
        return NROS_CPP_RET_ERROR;
    }
    let request = unsafe { core::slice::from_raw_parts(req_data, req_len) };
    // Clear the ready flag before sending so we don't pick up a stale wake
    // from a previous request.
    entry
        .reply_ready
        .store(false, core::sync::atomic::Ordering::Release);
    match entry.handle.send_request_raw(request) {
        Ok(_seq) => {
            // CRITICAL (RFC-0041 / Phase 239): `service_client_raw_try_process`
            // early-returns unless `pending` is set, so the reply would never be
            // dispatched to the response trampoline without this. Mirror the C
            // wrapper's `nros_client_send_request_async` exactly.
            entry.pending = true;
            // Register a waker that flips `reply_ready` when the transport
            // delivers the reply, so the executor wakes instead of blind-polling.
            let waker = unsafe { atomic_bool_waker(&entry.reply_ready) };
            entry.handle.register_waker(&waker);
            NROS_CPP_RET_OK
        }
        Err(e) => crate::transport_error_to_cpp_ret(e),
    }
}

/// Send a service request asynchronously (non-blocking).
///
/// # Safety
/// `storage` must be a valid initialized service client. `req_data` must point
/// to `req_len` readable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_client_send_request(
    storage: *mut c_void,
    req_data: *const u8,
    req_len: usize,
) -> nros_cpp_ret_t {
    if storage.is_null() || req_data.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let client = unsafe { &mut *(storage as *mut nros::internals::RmwServiceClient) };
    let req_slice = unsafe { core::slice::from_raw_parts(req_data, req_len) };
    match client.send_request_raw(req_slice) {
        Ok(_seq) => NROS_CPP_RET_OK,
        Err(e) => crate::transport_error_to_cpp_ret(e),
    }
}

/// Try to receive a reply (non-blocking).
///
/// Returns `NROS_CPP_RET_OK` and fills `resp_data`/`resp_len` on success,
/// `NROS_CPP_RET_TRY_AGAIN` if no reply is available yet.
///
/// # Safety
/// All pointers must be valid. `resp_data` must point to `resp_capacity` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_client_take_response(
    storage: *mut c_void,
    resp_data: *mut u8,
    resp_capacity: usize,
    resp_len: *mut usize,
) -> nros_cpp_ret_t {
    if storage.is_null() || resp_data.is_null() || resp_len.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let client = unsafe { &mut *(storage as *mut nros::internals::RmwServiceClient) };
    let resp_slice = unsafe { core::slice::from_raw_parts_mut(resp_data, resp_capacity) };

    match client.take_response_raw(resp_slice) {
        Ok(Some((len, _seq))) => {
            unsafe {
                *resp_len = len;
            }
            NROS_CPP_RET_OK
        }
        Ok(None) => {
            unsafe {
                *resp_len = 0;
            }
            NROS_CPP_RET_TRY_AGAIN
        }
        Err(e) => crate::transport_error_to_cpp_ret(e),
    }
}

/// Phase 124.C.3 — graph-aware "is the matching server up?" probe.
///
/// Writes `1` to `*out` if the backend has discovered ≥ 1 matching
/// server, `0` if not yet, or `-1` if the backend cannot answer
/// (e.g. XRCE). Never spins the executor — callers that want a
/// blocking wait should use the higher-level Promise / Future API.
///
/// # Safety
/// `storage` must be a valid initialized service client. `out` must
/// be a writable `i32` pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_client_server_available(
    storage: *mut c_void,
    out: *mut i32,
) -> nros_cpp_ret_t {
    use nros_node::ClientTrait;

    if storage.is_null() || out.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let client = unsafe { &*(storage as *const nros::internals::RmwServiceClient) };
    match client.service_is_ready() {
        Ok(true) => {
            unsafe { *out = 1 };
            NROS_CPP_RET_OK
        }
        Ok(false) => {
            unsafe { *out = 0 };
            NROS_CPP_RET_OK
        }
        Err(_) => {
            // Backend can't answer (Unsupported). Surface the sentinel
            // so C++ callers can distinguish "no" (0) from "don't
            // know" (-1) without losing the OK status.
            unsafe { *out = -1 };
            NROS_CPP_RET_OK
        }
    }
}

/// phase-338 W8 — block until a matching service server is discoverable.
///
/// The C API has had `nros_client_wait_for_service` since phase-124; C++ never
/// bound it, so every C++ client example hand-rolled a retry loop around the
/// first request instead (three attempts, spin in between) to paper over slow
/// discovery. This is the primitive those loops were approximating.
///
/// Mirrors `rclcpp::ClientBase::wait_for_service`: a spin on
/// `service_is_ready`, the same shape as `Client::wait_for_service`
/// (`nros-node/src/executor/handles.rs`) — each check is a synchronous read
/// of the discovery state the backend maintains, nothing is latched, and a
/// backend that cannot answer waits out the budget (phase-428 W13, issue 1087).
///
/// # Returns
/// * `NROS_CPP_RET_OK` — server visible.
/// * `NROS_CPP_RET_TIMEOUT` — budget elapsed without seeing a token.
/// * `NROS_CPP_RET_INVALID_ARGUMENT` — null storage / executor.
/// * `NROS_CPP_RET_TRANSPORT_ERROR` — transport-level failure.
///
/// # Safety
/// `storage` must be a valid initialized future-style service client;
/// `executor_handle` a valid `CppContext`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_client_wait_for_service(
    storage: *mut c_void,
    executor_handle: *mut c_void,
    timeout_ms: u32,
) -> nros_cpp_ret_t {
    use nros_rmw::ClientTrait as _;

    if storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    let Some(ctx) = (unsafe { cpp_ctx_checked(executor_handle) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let client = unsafe { &mut *(storage as *mut nros::internals::RmwServiceClient) };

    // phase-428 W13 — spin, then ask again. `Ok(true)` ONLY (issue 1008):
    // `Err` (the backend cannot answer) and `Ok(false)` both keep waiting.
    const SPIN_MS: u64 = 10;
    let deadline_ns = crate::nros_cpp_time_ns() + (timeout_ms as u64) * 1_000_000;
    loop {
        if matches!(client.service_is_ready(), Ok(true)) {
            return NROS_CPP_RET_OK;
        }
        if crate::nros_cpp_time_ns() >= deadline_ns {
            return NROS_CPP_RET_TIMEOUT;
        }
        let _ = ctx
            .executor
            .spin_once(core::time::Duration::from_millis(SPIN_MS));
    }
}

/// Destroy a service client (drop in place, no free).
///
/// # Safety
/// `storage` must be a valid initialized service client storage, or NULL (no-op).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_client_destroy(storage: *mut c_void) -> nros_cpp_ret_t {
    if storage.is_null() {
        return NROS_CPP_RET_OK;
    }
    unsafe {
        core::ptr::drop_in_place(storage as *mut nros::internals::RmwServiceClient);
    }
    NROS_CPP_RET_OK
}

/// Relocate an `RmwServiceClient` from `old_storage` to `new_storage`.
///
/// # Safety
/// See `nros_cpp_publisher_relocate`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_service_client_relocate(
    old_storage: *mut c_void,
    new_storage: *mut c_void,
) -> nros_cpp_ret_t {
    if old_storage.is_null() || new_storage.is_null() {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    }
    unsafe {
        let value = core::ptr::read(old_storage as *mut nros::internals::RmwServiceClient);
        core::ptr::write(new_storage as *mut nros::internals::RmwServiceClient, value);
    }
    NROS_CPP_RET_OK
}
