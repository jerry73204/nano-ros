//! Service API for nros C API.
//!
//! Services provide request-reply communication patterns.
//! This module implements both service servers and clients.

use core::{
    ffi::{c_char, c_void},
    ptr,
    sync::atomic::AtomicBool,
    task::{RawWaker, RawWakerVTable, Waker},
};

use crate::{
    constants::{MAX_SERVICE_NAME_LEN, MAX_TYPE_HASH_LEN, MAX_TYPE_NAME_LEN},
    error::*,
    executor::nros_executor_t,
    node::{nros_node_state_t, nros_node_t},
    opaque_sizes::{SERVICE_CLIENT_OPAQUE_U64S, SERVICE_SERVER_OPAQUE_U64S},
    publisher::nros_service_type_t,
};

// ============================================================================
// Waker helper — creates a Waker that sets an AtomicBool
// ============================================================================

/// Create a [`Waker`] whose `wake()` stores `true` into `flag`.
///
/// Used to bridge `transport.register_waker(&Waker)` to the arena
/// entry's `reply_ready` `AtomicBool`. Alloc-free — the returned
/// `Waker` holds the raw `*const AtomicBool` derived from `flag`,
/// not an owned Arc.
///
/// # Safety
///
/// The caller must uphold all three invariants below; the function
/// signature returns an effectively `'static` `Waker` that cannot
/// carry the lifetime, so the borrow checker CANNOT enforce them.
/// Phase 214 audit (G/J review) made the contract explicit.
///
/// 1. **Lifetime**: `flag` and every byte of the pointed-to
///    `AtomicBool` must remain valid for the entire reachable
///    lifetime of the returned `Waker` AND of every `Waker` cloned
///    from it. The vtable does not refcount; the underlying
///    `AtomicBool` must outlive the longest-living clone.
///
/// 2. **Unregister-before-free**: if `flag` lives in a recyclable
///    arena slot, the caller MUST unregister the `Waker` from any
///    holder (e.g. `transport.register_waker(&noop)`) before
///    dropping / repositioning / freeing the slot. The `Waker`'s
///    own `Drop` impl is a no-op and does NOT detach the borrow.
///
/// 3. **Concurrent wake races**: `Waker: Send + Sync`. A transport
///    that delivers wakes from a background thread MAY race with
///    the spin loop. The caller MUST ensure that
///    `transport.register_waker(&noop)` (or equivalent unregister)
///    completes — including draining in-flight clones — before the
///    `AtomicBool`'s storage is invalidated.
///
/// Callsites: today's only callsite at `service.rs::nros_client_send_request_async`
/// borrows `&entry.reply_ready` where `entry` lives in the executor's
/// service-client arena. The arena is heap-allocated via `core::ptr::write`
/// at session-open and is never resized or recycled until the executor is
/// destroyed; the transport's `register_waker` slot is single-Waker and
/// gets overwritten on each subsequent call. Both contracts above hold by
/// construction.
unsafe fn atomic_bool_waker(flag: &AtomicBool) -> Waker {
    static VTABLE: RawWakerVTable = RawWakerVTable::new(
        // clone: return a new RawWaker pointing to the same flag.
        // NB: no refcount — clones share the borrow per the # Safety
        // invariant 1.
        |data| RawWaker::new(data, &VTABLE),
        // wake: set the flag (by value — consumes the waker)
        |data| unsafe {
            (*(data as *const AtomicBool)).store(true, core::sync::atomic::Ordering::Release);
        },
        // wake_by_ref: set the flag (by reference — waker stays alive)
        |data| unsafe {
            (*(data as *const AtomicBool)).store(true, core::sync::atomic::Ordering::Release);
        },
        // drop: no-op — the flag is borrowed, not owned. Unregister
        // happens via the transport-side `register_waker` overwrite
        // contract (# Safety invariant 2).
        |_data| {},
    );
    let raw = RawWaker::new(flag as *const AtomicBool as *const (), &VTABLE);
    // SAFETY: the vtable is valid; per the function's # Safety contract,
    // the caller asserts `flag` outlives the returned Waker + clones.
    unsafe { Waker::from_raw(raw) }
}

// ============================================================================
// Service Server
// ============================================================================

/// Service server callback function type.
///
/// # Parameters
/// * `request_data` - Pointer to CDR-serialized request data
/// * `request_len` - Length of request data in bytes
/// * `response_data` - Pointer to buffer for CDR-serialized response
/// * `response_capacity` - Capacity of response buffer
/// * `response_len` - Output: actual length of response data written
/// * `context` - User-provided context pointer
///
/// # Returns
/// * `true` if the request was handled successfully
/// * `false` if there was an error handling the request
pub type nros_service_callback_t = Option<
    unsafe extern "C" fn(
        request_data: *const u8,
        request_len: usize,
        response_data: *mut u8,
        response_capacity: usize,
        response_len: *mut usize,
        context: *mut c_void,
    ) -> bool,
>;

/// Service server state
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_service_state_t {
    /// Not initialized
    NROS_SERVICE_STATE_UNINITIALIZED = 0,
    /// L2 callback-mode: initialized via `nros_service_init`, transport
    /// creation deferred to `nros_executor_add_service`.
    NROS_SERVICE_STATE_INITIALIZED = 1,
    /// Shutdown
    NROS_SERVICE_STATE_SHUTDOWN = 2,
    /// Phase 122.3.c.4 — L1 polling-mode: transport entity lives inline
    /// in `_opaque`; caller drains via
    /// `nros_service_take_request_raw` and replies via
    /// `nros_service_send_response_raw`. No executor registration.
    NROS_SERVICE_STATE_POLLING = 3,
}

/// Service server structure.
#[repr(C)]
pub struct nros_service_t {
    /// Current state
    pub state: nros_service_state_t,
    /// Service name storage
    pub service_name: [u8; MAX_SERVICE_NAME_LEN],
    /// Service name length
    pub service_name_len: usize,
    /// Type name storage
    pub type_name: [u8; MAX_TYPE_NAME_LEN],
    /// Type name length
    pub type_name_len: usize,
    /// Type hash storage
    pub type_hash: [u8; MAX_TYPE_HASH_LEN],
    /// Type hash length
    pub type_hash_len: usize,
    /// User callback function
    pub callback: nros_service_callback_t,
    /// User context pointer
    pub context: *mut c_void,
    /// Pointer to parent node
    pub node: crate::node::nros_node_ref_t,
    /// Phase 193.4 — service QoS (applied to both request + reply endpoints).
    /// Defaults to the services profile (RELIABLE+VOLATILE+KEEP_LAST(10));
    /// set via `nros_service_init_with_qos`.
    pub qos: crate::qos::nros_qos_t,
    /// Phase 189.M3.3.a — scheduling-context slot to bind the service's
    /// executor handle to. `0` = inherit the executor / Node default (no
    /// explicit bind); set via `nros_service_init_with_options`. When non-zero,
    /// `nros_executor_add_service` binds the freshly-created handle to this
    /// SC after registration. No effect on the L1 polling path.
    pub sched_context_id: crate::executor::nros_sched_context_id_t,
    /// Internal state (arena entry index + executor pointer). Phase 87.5:
    /// Typed C-ABI handle field (was an opaque blob in earlier versions).
    pub _internal: ServiceServerInternal,
    /// Phase 122.3.c.4 — inline opaque storage for the L1 polling-mode
    /// `RawServiceServer<MESSAGE_BUFFER_SIZE, MESSAGE_BUFFER_SIZE>`.
    /// Zeroed in L2 (callback + executor arena) mode; populated by
    /// `nros_service_init_polling`.
    pub _opaque: [u64; SERVICE_SERVER_OPAQUE_U64S],
}

impl Default for nros_service_t {
    fn default() -> Self {
        Self {
            state: nros_service_state_t::NROS_SERVICE_STATE_UNINITIALIZED,
            service_name: [0u8; MAX_SERVICE_NAME_LEN],
            service_name_len: 0,
            type_name: [0u8; MAX_TYPE_NAME_LEN],
            type_name_len: 0,
            type_hash: [0u8; MAX_TYPE_HASH_LEN],
            type_hash_len: 0,
            callback: None,
            context: ptr::null_mut(),
            node: crate::node::nros_node_ref_t::none(),
            qos: crate::qos::nros_qos_t::default(),
            sched_context_id: 0,
            _internal: ServiceServerInternal::new(),
            _opaque: [0u64; SERVICE_SERVER_OPAQUE_U64S],
        }
    }
}

impl nros_service_t {
    /// Get the callback function
    pub(crate) fn get_callback(&self) -> nros_service_callback_t {
        self.callback
    }

    /// Get the context pointer
    pub(crate) fn get_context(&self) -> *mut c_void {
        self.context
    }

    /// Phase 193.4 — the service's QoS as `nros_node` settings.
    pub(crate) fn get_qos_settings(&self) -> nros_rmw::QoSProfile {
        self.qos.to_qos_settings()
    }
}

/// Get a zero-initialized service server.
#[unsafe(no_mangle)]
pub extern "C" fn nros_service_get_zero_initialized() -> nros_service_t {
    nros_service_t::default()
}

/// Initialize a service server.
///
/// # Parameters
/// * `service` - Pointer to a zero-initialized service
/// * `node` - Pointer to an initialized node
/// * `type_info` - Pointer to service type information
/// * `service_name` - Service name (null-terminated string)
/// * `callback` - Callback function to invoke when requests arrive
/// * `context` - User context pointer passed to callback (can be NULL)
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if any required pointer is NULL
/// * `NROS_RET_NOT_INIT` if node is not initialized
/// * `NROS_RET_ERROR` on initialization failure
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_service_init(
    service: *mut nros_service_t,
    node: *const nros_node_t,
    type_info: *const nros_service_type_t,
    service_name: *const c_char,
    callback: nros_service_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    validate_not_null!(service, node, type_info, service_name);

    if callback.is_none() {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let service = &mut *service;
    let node_ref = &*node;
    let type_info = &*type_info;

    validate_state!(
        service,
        nros_service_state_t::NROS_SERVICE_STATE_UNINITIALIZED,
        NROS_RET_BAD_SEQUENCE
    );
    validate_state!(node_ref, nros_node_state_t::NROS_NODE_STATE_INITIALIZED);

    // Copy service name (required — empty rejected)
    service.service_name_len = crate::util::copy_cstr_into(service_name, &mut service.service_name);
    if service.service_name_len == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }

    // Copy type name + hash (both optional — null sources leave dst untouched)
    service.type_name_len =
        crate::util::copy_cstr_into(type_info.type_name, &mut service.type_name);
    service.type_hash_len =
        crate::util::copy_cstr_into(type_info.type_hash, &mut service.type_hash);

    // Store callback and context
    service.callback = callback;
    service.context = context;
    service.node = unsafe { crate::node::node_ref_of(node) };
    // Phase 193.4 — default to the services profile; nros_service_init_with_qos
    // overrides. (`register_service` reads this at registration time.)
    service.qos = crate::qos::nros_qos_t::default();

    // Service server creation is deferred to nros_executor_add_service(),
    // which calls nros_node::Executor::register_service_raw_sized().
    // Initialise the internal state (executor_ptr null until registration).
    service._internal = ServiceServerInternal::new();
    service.state = nros_service_state_t::NROS_SERVICE_STATE_INITIALIZED;

    NROS_RET_OK
}

/// Phase 193.4 — initialize a service server with an explicit QoS profile
/// (rclc's `_with_options`; the profile applies to both the request + reply
/// endpoints). `qos` NULL ⇒ the services default. Same as
/// [`nros_service_init`] otherwise.
///
/// # Safety
/// All non-NULL pointers must be valid + the node initialized.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_service_init_with_qos(
    service: *mut nros_service_t,
    node: *const nros_node_t,
    type_info: *const nros_service_type_t,
    service_name: *const c_char,
    callback: nros_service_callback_t,
    context: *mut c_void,
    qos: *const crate::qos::nros_qos_t,
) -> nros_ret_t {
    let ret = nros_service_init(service, node, type_info, service_name, callback, context);
    if ret == NROS_RET_OK && !qos.is_null() {
        (*service).qos = *qos;
    }
    ret
}

/// Phase 189.M3.3.a — rclc-style named service options.
///
/// Sits ALONGSIDE the QoS profile (same convention as
/// `nros_subscription_options_t`): QoS is passed separately, this struct
/// carries the non-QoS service-creation axes. Plain scalar fields only — safe
/// to stack-allocate, memcpy, and pass across the FFI. Zero-init selects the
/// default behaviour, identical to `nros_service_init_with_qos`.
#[repr(C)]
#[derive(Default)]
pub struct nros_service_options_t {
    /// Scheduling-context slot to bind the service's executor handle to.
    /// `0` = inherit the executor / Node default (no explicit bind). A non-zero
    /// value must be an id previously returned from
    /// `nros_executor_create_sched_context`; the bind is applied by
    /// `nros_executor_add_service` once the handle exists. No effect on the
    /// L1 polling path.
    pub sched_context: crate::executor::nros_sched_context_id_t,
    /// Reserved for future use; must be zero. Pads the struct for ABI stability.
    pub _reserved: [u8; 3],
}

/// Get a zero-initialised [`nros_service_options_t`] (`sched_context = 0`).
#[unsafe(no_mangle)]
pub extern "C" fn nros_service_get_default_options() -> nros_service_options_t {
    nros_service_options_t::default()
}

/// Phase 189.M3.3.a — initialize a service server with custom QoS + named
/// options. Behaves like [`nros_service_init_with_qos`] except a non-zero
/// `options->sched_context` is stashed on the service so that
/// [`nros_executor_add_service`] binds the resulting executor handle to
/// that scheduling context once the handle is known (server creation is
/// deferred to registration, so the handle does not exist at init time).
///
/// # Safety
/// All non-NULL pointers must be valid + the node initialized; `qos` / `options`
/// may be NULL.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_service_init_with_options(
    service: *mut nros_service_t,
    node: *const nros_node_t,
    type_info: *const nros_service_type_t,
    service_name: *const c_char,
    callback: nros_service_callback_t,
    context: *mut c_void,
    qos: *const crate::qos::nros_qos_t,
    options: *const nros_service_options_t,
) -> nros_ret_t {
    let ret = nros_service_init_with_qos(
        service,
        node,
        type_info,
        service_name,
        callback,
        context,
        qos,
    );
    if ret != NROS_RET_OK {
        return ret;
    }
    if !options.is_null() {
        (*service).sched_context_id = (*options).sched_context;
    }
    NROS_RET_OK
}

/// Finalize a service server.
///
/// # Parameters
/// * `service` - Pointer to an initialized service
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if service is NULL
/// * `NROS_RET_NOT_INIT` if not initialized
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_service_fini(service: *mut nros_service_t) -> nros_ret_t {
    validate_not_null!(service);

    let service = &mut *service;

    match service.state {
        nros_service_state_t::NROS_SERVICE_STATE_INITIALIZED => {
            // L2: server lives in the executor arena (if registered) —
            // reset metadata. The arena entry's Drop runs when the
            // executor is destroyed.
        }
        nros_service_state_t::NROS_SERVICE_STATE_POLLING => {
            // L1: drop the inline RawServiceServer so its Drop runs
            // (closes the underlying RMW server).
            #[cfg(feature = "rmw-cffi")]
            {
                core::ptr::drop_in_place(service._opaque.as_mut_ptr()
                    as *mut nros_node::RawServiceServer<
                        { crate::config::MESSAGE_BUFFER_SIZE },
                        { crate::config::MESSAGE_BUFFER_SIZE },
                    >);
                service._opaque = [0u64; SERVICE_SERVER_OPAQUE_U64S];
            }
        }
        _ => return NROS_RET_NOT_INIT,
    }

    service._internal = ServiceServerInternal::new();
    service.callback = None;
    service.context = ptr::null_mut();
    service.node = crate::node::nros_node_ref_t::none();
    service.state = nros_service_state_t::NROS_SERVICE_STATE_SHUTDOWN;

    NROS_RET_OK
}

// ============================================================================
// Phase 122.3.c.4 — Layer-1 primitive entry points (caller polls)
// ============================================================================
//
// L1 path: caller owns scheduling. The transport server is created
// during `nros_service_init_polling` and stored inline in
// `service._opaque`; caller drains requests via
// `nros_service_take_request_raw` and replies via
// `nros_service_send_response_raw`. No executor registration. Used by
// RTIC / embassy / FreeRTOS-task-per-entity patterns and the C/C++
// FFI shims for callers that drive their own poll loops.

/// Phase 122.3.c.4 — initialize an L1 polling-mode service server.
///
/// Creates the underlying RMW server immediately and stores it inline
/// in the service's `_opaque` field. The caller drains received
/// requests via `nros_service_take_request_raw` and sends replies
/// via `nros_service_send_response_raw`.
///
/// # Parameters
/// * `service` - Pointer to a zero-initialized service
/// * `node` - Pointer to an initialized node
/// * `type_info` - Pointer to service type information
/// * `service_name` - Service name (null-terminated)
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if any pointer is NULL or name empty
/// * `NROS_RET_NOT_INIT` if node / support not initialized
/// * `NROS_RET_ERROR` if server creation failed
///
/// # Safety
/// All pointers must be valid; `service_name` must be a valid
/// null-terminated string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_service_init_polling(
    service: *mut nros_service_t,
    node: *const nros_node_t,
    type_info: *const nros_service_type_t,
    service_name: *const c_char,
) -> nros_ret_t {
    validate_not_null!(service, node, type_info, service_name);

    let service_mut = &mut *service;
    let node_ref = &*node;
    let type_info_ref = &*type_info;

    validate_state!(
        service_mut,
        nros_service_state_t::NROS_SERVICE_STATE_UNINITIALIZED,
        NROS_RET_BAD_SEQUENCE
    );
    validate_state!(node_ref, nros_node_state_t::NROS_NODE_STATE_INITIALIZED);

    service_mut.service_name_len =
        crate::util::copy_cstr_into(service_name, &mut service_mut.service_name);
    if service_mut.service_name_len == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }
    service_mut.type_name_len =
        crate::util::copy_cstr_into(type_info_ref.type_name, &mut service_mut.type_name);
    service_mut.type_hash_len =
        crate::util::copy_cstr_into(type_info_ref.type_hash, &mut service_mut.type_hash);

    service_mut.node = unsafe { crate::node::node_ref_of(node) };
    service_mut.callback = None;
    service_mut.context = ptr::null_mut();

    #[cfg(feature = "rmw-cffi")]
    {
        use nros_node::{ServiceInfo, Session};

        // Phase 156 Sub-bug D — multi-Session dispatch (see
        // `nros_publisher_init`).
        let (session, domain_id) = match crate::node::resolve_session_and_domain(node_ref) {
            Some(t) => t,
            None => return NROS_RET_NOT_INIT,
        };

        let service_str = core::str::from_utf8_unchecked(
            &service_mut.service_name[..service_mut.service_name_len],
        );
        let type_str =
            core::str::from_utf8_unchecked(&service_mut.type_name[..service_mut.type_name_len]);
        let type_hash_str =
            core::str::from_utf8_unchecked(&service_mut.type_hash[..service_mut.type_hash_len]);
        let node_name_str = core::str::from_utf8_unchecked(&node_ref.name[..node_ref.name_len]);
        let namespace_str =
            core::str::from_utf8_unchecked(&node_ref.namespace[..node_ref.namespace_len]);

        let info = ServiceInfo::new(service_str, type_str, type_hash_str)
            .with_domain(domain_id)
            .with_node_name(node_name_str)
            .with_namespace(namespace_str);

        match session.create_service(&info, nros_node::QoSProfile::services_default()) {
            Ok(handle) => {
                let raw = nros_node::RawServiceServer::<
                    { crate::config::MESSAGE_BUFFER_SIZE },
                    { crate::config::MESSAGE_BUFFER_SIZE },
                >::new(handle);
                core::ptr::write(
                    service_mut._opaque.as_mut_ptr()
                        as *mut nros_node::RawServiceServer<
                            { crate::config::MESSAGE_BUFFER_SIZE },
                            { crate::config::MESSAGE_BUFFER_SIZE },
                        >,
                    raw,
                );
            }
            Err(_) => return NROS_RET_ERROR,
        }
    }

    service_mut.state = nros_service_state_t::NROS_SERVICE_STATE_POLLING;
    NROS_RET_OK
}

/// Phase 122.3.c.6.e — C-ABI wake-state slot. Caller declares one
/// per (entity, channel) pair next to the entity, passes a pointer
/// to it into the matching `nros_*_set_wake_callback` and keeps it
/// alive at the same address for as long as the entity is in
/// POLLING state. Two `uint64_t` slots are enough to hold the
/// `(fn_ptr, ctx)` pair plus 8-byte alignment.
#[repr(C)]
#[derive(Default)]
pub struct nros_wake_state_t {
    pub _opaque: [u64; 2],
}

/// Phase 122.3.c.6.e — zero-initialise a wake-state slot.
#[unsafe(no_mangle)]
pub extern "C" fn nros_wake_state_get_zero_initialized() -> nros_wake_state_t {
    nros_wake_state_t::default()
}

/// Phase 122.3.c.6.e — register a C wake callback on an L1
/// polling-mode service server. `state` must point to a
/// `nros_wake_state_t` allocated by the caller; it must outlive
/// the service (typically declared inline next to the
/// `nros_service_t`) and must not move after this call.
/// Pass `cb = NULL` to disable. The backend wakes the callback
/// when a new request arrives.
///
/// # Safety
/// All pointers valid; `state` storage stable for the entity's
/// lifetime.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_service_set_wake_callback(
    service: *mut nros_service_t,
    state: *mut nros_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_ret_t {
    if service.is_null() || state.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let service_mut = &mut *service;
    if service_mut.state != nros_service_state_t::NROS_SERVICE_STATE_POLLING {
        return NROS_RET_INVALID_ARGUMENT;
    }

    #[cfg(feature = "rmw-cffi")]
    {
        let state_ptr = state as *mut nros_node::c_waker::CWakeState;
        // Re-initialise via volatile write — caller may pass a
        // zero-initialised state (first call) or a state already
        // holding an older callback (re-registration). Either way,
        // overwriting both fields is safe before the new Waker is
        // built.
        core::ptr::write(
            state_ptr,
            nros_node::c_waker::CWakeState { fn_ptr: cb, ctx },
        );
        let waker = nros_node::c_waker::make_waker(state_ptr);
        let raw = &*(service_mut._opaque.as_ptr()
            as *const nros_node::RawServiceServer<
                { crate::config::MESSAGE_BUFFER_SIZE },
                { crate::config::MESSAGE_BUFFER_SIZE },
            >);
        raw.register_waker(&waker);
        NROS_RET_OK
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (state, cb, ctx);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.6.e — register a C wake callback on an L1
/// polling-mode service client. Wakes when a reply lands. See
/// `nros_service_set_wake_callback` for the lifetime contract.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_set_wake_callback(
    client: *mut nros_client_t,
    state: *mut nros_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_ret_t {
    if client.is_null() || state.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let client_mut = &mut *client;
    if client_mut.state != nros_client_state_t::NROS_CLIENT_STATE_POLLING {
        return NROS_RET_INVALID_ARGUMENT;
    }

    #[cfg(feature = "rmw-cffi")]
    {
        let state_ptr = state as *mut nros_node::c_waker::CWakeState;
        core::ptr::write(
            state_ptr,
            nros_node::c_waker::CWakeState { fn_ptr: cb, ctx },
        );
        let waker = nros_node::c_waker::make_waker(state_ptr);
        let raw = &*(client_mut._opaque.as_ptr()
            as *const nros_node::RawServiceClient<
                { crate::config::MESSAGE_BUFFER_SIZE },
                { crate::config::MESSAGE_BUFFER_SIZE },
            >);
        raw.register_waker(&waker);
        NROS_RET_OK
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (state, cb, ctx);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.4 — non-blocking poll for a pending request on an L1
/// polling-mode service. Writes the request bytes into the caller's
/// `buf` and the matching `sequence_number` (required for reply).
///
/// # Returns
/// * `>= 0` — number of bytes written to `buf` (0 = no request)
/// * `NROS_RET_INVALID_ARGUMENT` if pointers / state wrong
/// * `NROS_RET_ERROR` on transport failure
///
/// # Safety
/// `service` must be in `POLLING` state. `buf` writable for `buf_len`
/// bytes. `sequence_number` writable for `i64`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_service_take_request_raw(
    service: *mut nros_service_t,
    buf: *mut u8,
    buf_len: usize,
    sequence_number: *mut i64,
) -> i32 {
    if service.is_null() || (buf.is_null() && buf_len != 0) || sequence_number.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let service_mut = &mut *service;
    if service_mut.state != nros_service_state_t::NROS_SERVICE_STATE_POLLING {
        return NROS_RET_INVALID_ARGUMENT;
    }

    #[cfg(feature = "rmw-cffi")]
    {
        let raw = &mut *(service_mut._opaque.as_mut_ptr()
            as *mut nros_node::RawServiceServer<
                { crate::config::MESSAGE_BUFFER_SIZE },
                { crate::config::MESSAGE_BUFFER_SIZE },
            >);
        match raw.take_request_raw() {
            Ok(Some((len, seq))) => {
                let copy_len = len.min(buf_len);
                core::ptr::copy_nonoverlapping(raw.req_buffer().as_ptr(), buf, copy_len);
                *sequence_number = seq;
                copy_len as i32
            }
            Ok(None) => 0,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (buf, buf_len, sequence_number);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.4 — send a reply on an L1 polling-mode service.
///
/// `sequence_number` must equal the value returned by the most recent
/// `nros_service_take_request_raw` for the request being replied
/// to.
///
/// # Safety
/// `service` must be in `POLLING` state. `data` readable for `len`
/// bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_service_send_response_raw(
    service: *mut nros_service_t,
    sequence_number: i64,
    data: *const u8,
    len: usize,
) -> nros_ret_t {
    if service.is_null() || (data.is_null() && len != 0) {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let service_mut = &mut *service;
    if service_mut.state != nros_service_state_t::NROS_SERVICE_STATE_POLLING {
        return NROS_RET_INVALID_ARGUMENT;
    }

    #[cfg(feature = "rmw-cffi")]
    {
        let raw = &mut *(service_mut._opaque.as_mut_ptr()
            as *mut nros_node::RawServiceServer<
                { crate::config::MESSAGE_BUFFER_SIZE },
                { crate::config::MESSAGE_BUFFER_SIZE },
            >);
        let slice = core::slice::from_raw_parts(data, len);
        match raw.send_response_raw(sequence_number, slice) {
            Ok(()) => NROS_RET_OK,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (sequence_number, data, len);
        NROS_RET_NOT_INIT
    }
}

// ============================================================================
// Typed service / client path — the reporting half (phase-417 W5.e)
// ============================================================================
//
// The generated `<Srv>_service_handle_request` / `<Srv>_client_handle_response`
// trampolines in `packs/c/service.h.jinja` are `static inline` glue over the
// per-type `_deserialize` / `_serialize` the same pack emits. Everything a
// trampoline needs is in `<nros/types.h>` EXCEPT one thing: a way to be LOUD.
//
// RFC-0087 requires a deserialise failure or an oversized payload to be loud
// rather than a silent truncation, and the executor's own dispatch cannot
// supply that — `srv_raw_try_process`
// (`nros-node/src/executor/arena.rs:2559`) reads the callback's `false` as
// "send nothing" and returns `Ok(true)`, so a refusal is indistinguishable
// from a reply that has not arrived yet. The peer eventually times out with
// no statement of why.
//
// So the diagnostic lands HERE, in Rust, for the reason RFC-0087 §"Who
// implements an adopted name" gives: the logger is Rust's, and a second
// formatting path in a generated header would be a second implementation.
// Pulling `<nros/log.h>` into every generated service header is not an option
// either — it includes `<stdio.h>`, and these headers are freestanding.
//
// The generated glue therefore calls ONE exported symbol with an error code
// and the type's own name, and this function turns that into an `ERROR`
// record on the same sink chain every other C log record uses.

/// Typed glue succeeded.
pub const NROS_SERVICE_TYPED_OK: i32 = 0;
/// `<Req>_deserialize` rejected the request bytes — the request was malformed,
/// truncated, or encoded by a peer whose schema disagrees. No reply is sent.
pub const NROS_SERVICE_TYPED_ERR_REQUEST_DESERIALIZE: i32 = -1;
/// `<Resp>_serialize` did not fit the executor's reply buffer. No reply is
/// sent — a truncated CDR payload is a wrong answer, not a partial one.
pub const NROS_SERVICE_TYPED_ERR_RESPONSE_SERIALIZE: i32 = -2;
/// `<Req>_serialize` did not fit the scratch buffer the caller supplied to
/// `<Srv>_client_send_request`. Nothing is sent.
pub const NROS_SERVICE_TYPED_ERR_REQUEST_SERIALIZE: i32 = -3;
/// `<Resp>_deserialize` rejected the reply bytes. The response struct is left
/// zero-initialised rather than half-filled.
pub const NROS_SERVICE_TYPED_ERR_RESPONSE_DESERIALIZE: i32 = -4;
/// The handler struct reached the trampoline with no user callback installed —
/// `<Srv>_service_handler_init` was skipped or the struct was re-zeroed.
pub const NROS_SERVICE_TYPED_ERR_NO_CALLBACK: i32 = -5;

/// Append as much of `s` as fits, silently stopping at the end of `buf`.
///
/// Truncating a DIAGNOSTIC is not the truncation RFC-0087 forbids: the
/// payload is refused either way, and this only bounds how much of the type
/// name the message can carry.
fn append_bytes(buf: &mut [u8], n: &mut usize, s: &[u8]) {
    let room = buf.len().saturating_sub(*n);
    let take = if s.len() < room { s.len() } else { room };
    buf[*n..*n + take].copy_from_slice(&s[..take]);
    *n += take;
}

/// Borrow a NUL-terminated C string as bytes, bounded so a missing NUL cannot
/// walk off the end.
///
/// # Safety
/// `ptr` is NULL or points to at least one readable byte, NUL-terminated
/// within `max` bytes or not at all.
unsafe fn bounded_cstr(ptr: *const c_char, max: usize) -> &'static [u8] {
    if ptr.is_null() {
        return b"<unknown>";
    }
    let mut len = 0usize;
    while len < max && *ptr.add(len) != 0 {
        len += 1;
    }
    core::slice::from_raw_parts(ptr, len)
}

/// Report a typed service/client glue failure as an `ERROR` log record.
///
/// Called by the generated `static inline` trampolines. `error` is one of the
/// `NROS_SERVICE_TYPED_ERR_*` codes above; `type_name` is the service type's
/// own `<Srv>_get_type_name()`, so the record names the type without the
/// generated header having to duplicate any string.
///
/// Deliberately returns nothing and cannot fail: it is the loudness path, and
/// a loudness path that itself has an error channel just moves the silence.
///
/// # Safety
/// `type_name` is NULL or a NUL-terminated string readable for its length.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_service_typed_report_error(error: i32, type_name: *const c_char) {
    let reason: &[u8] = match error {
        NROS_SERVICE_TYPED_ERR_REQUEST_DESERIALIZE => {
            b"request CDR did not decode; no reply sent" as &[u8]
        }
        NROS_SERVICE_TYPED_ERR_RESPONSE_SERIALIZE => {
            b"response did not fit the reply buffer; no reply sent" as &[u8]
        }
        NROS_SERVICE_TYPED_ERR_REQUEST_SERIALIZE => {
            b"request did not fit the caller's scratch buffer; nothing sent" as &[u8]
        }
        NROS_SERVICE_TYPED_ERR_RESPONSE_DESERIALIZE => {
            b"response CDR did not decode; response left zeroed" as &[u8]
        }
        NROS_SERVICE_TYPED_ERR_NO_CALLBACK => {
            b"no typed callback installed on the handler" as &[u8]
        }
        _ => b"unknown typed service failure" as &[u8],
    };

    let mut buf = [0u8; 256];
    let mut n = 0usize;
    append_bytes(&mut buf, &mut n, b"typed service glue refused a payload: ");
    append_bytes(&mut buf, &mut n, reason);
    append_bytes(&mut buf, &mut n, b" (type=");
    append_bytes(&mut buf, &mut n, bounded_cstr(type_name, 128));
    append_bytes(&mut buf, &mut n, b")");

    crate::log::nros_log_emit_at(
        crate::log::nros_log_default_logger(),
        crate::log::nros_log_severity_t::NROS_LOG_SEVERITY_ERROR,
        buf.as_ptr() as *const c_char,
        n,
        c"<nros-c typed service>".as_ptr(),
        0,
    );
}

/// Get the service name.
///
/// # Parameters
/// * `service` - Pointer to a service
///
/// # Returns
/// * Pointer to service name (null-terminated), or NULL if invalid
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_service_get_service_name(
    service: *const nros_service_t,
) -> *const c_char {
    if service.is_null() {
        return ptr::null();
    }

    let service = &*service;
    if service.state != nros_service_state_t::NROS_SERVICE_STATE_INITIALIZED {
        return ptr::null();
    }

    service.service_name.as_ptr() as *const c_char
}

/// Check if service is valid (initialized).
///
/// # Parameters
/// * `service` - Pointer to a service
///
/// # Returns
/// * `true` if valid, `false` if invalid or NULL
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_service_is_valid(service: *const nros_service_t) -> bool {
    if service.is_null() {
        return false;
    }

    let service = &*service;
    service.state == nros_service_state_t::NROS_SERVICE_STATE_INITIALIZED
}

// ============================================================================
// Service Server Internal
// ============================================================================

/// Internal state for the service server (Phase 82.7).
///
/// Lightweight — stores only the arena entry index and the executor
/// pointer where the actual transport handle lives. Mirrors
/// `ServiceClientInternal` and `ActionClientInternal`.
///
/// struct definition directly — no hand-math or `u64s_for::<T>()` probe
/// required.
#[repr(C)]
pub struct ServiceServerInternal {
    /// Arena entry index. -1 means not registered with any executor yet.
    pub arena_entry_index: i32,
    /// Pointer to the outer `nros_executor_t` that owns the arena entry.
    pub executor_ptr: *mut c_void,
}

impl ServiceServerInternal {
    pub const fn new() -> Self {
        Self {
            arena_entry_index: -1,
            executor_ptr: core::ptr::null_mut(),
        }
    }
}

impl Default for ServiceServerInternal {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Service Client
// ============================================================================

/// Default service-client RPC timeout in milliseconds (Phase 82).
///
/// `nros_client_call` reads this from `ServiceClientInternal.timeout_ms`,
/// which is initialised to this value by `nros_client_init` and can be
/// changed at any time via `nros_client_set_timeout`.
///
/// Phase 192.4 — baked from `NROS_SERVICE_TIMEOUT_MS` at build time (default
/// 30000), the same env var and default the zenoh backend uses, so the two
/// RMW paths no longer disagree.
const NROS_DEFAULT_SERVICE_TIMEOUT_MS: u32 = crate::config::SERVICE_DEFAULT_TIMEOUT_MS;

/// Service-client response callback type (Phase 82).
///
/// Invoked by the executor's `spin_some` dispatch when a previously-sent
/// request has its response delivered. The CDR bytes are owned by the
/// arena entry's reply buffer for the duration of the call — copy if you
/// need to keep them.
pub type nros_response_callback_t =
    Option<unsafe extern "C" fn(response: *const u8, response_len: usize, context: *mut c_void)>;

/// Internal state for the service client (Phase 82).
///
/// Lightweight — stores only the arena entry index and the executor
/// pointer where the actual transport handle lives. Mirrors
/// `ActionClientInternal`.
#[repr(C)]
pub struct ServiceClientInternal {
    /// Arena entry index. -1 means not registered with any executor yet.
    pub arena_entry_index: i32,
    /// Pointer to the executor that owns the arena entry.
    pub executor_ptr: *mut c_void,
    /// Default timeout used by `nros_client_call`.
    pub timeout_ms: u32,
}

impl ServiceClientInternal {
    pub const fn new() -> Self {
        Self {
            arena_entry_index: -1,
            executor_ptr: ptr::null_mut(),
            timeout_ms: NROS_DEFAULT_SERVICE_TIMEOUT_MS,
        }
    }
}

impl Default for ServiceClientInternal {
    fn default() -> Self {
        Self::new()
    }
}

/// Client state
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_client_state_t {
    /// Not initialized
    NROS_CLIENT_STATE_UNINITIALIZED = 0,
    /// Initialized — metadata stored, *not yet registered with an executor*.
    NROS_CLIENT_STATE_INITIALIZED = 1,
    /// Registered with an executor and ready for `send_request_async` / `call`.
    NROS_CLIENT_STATE_REGISTERED = 2,
    /// Shutdown
    NROS_CLIENT_STATE_SHUTDOWN = 3,
    /// Phase 122.3.c.5 — L1 polling-mode: transport entity lives inline
    /// in `_opaque`; caller drives via `nros_client_send_request_raw`
    /// and `nros_client_take_response_raw`. No executor registration.
    NROS_CLIENT_STATE_POLLING = 4,
}

/// Service client structure.
#[repr(C)]
pub struct nros_client_t {
    /// Current state
    pub state: nros_client_state_t,
    /// Service name storage
    pub service_name: [u8; MAX_SERVICE_NAME_LEN],
    /// Service name length
    pub service_name_len: usize,
    /// Type name storage
    pub type_name: [u8; MAX_TYPE_NAME_LEN],
    /// Type name length
    pub type_name_len: usize,
    /// Type hash storage
    pub type_hash: [u8; MAX_TYPE_HASH_LEN],
    /// Type hash length
    pub type_hash_len: usize,
    /// User response callback, fired from `nros_executor_spin_some` when
    /// a response to a previously-sent async request arrives.
    pub response_callback: nros_response_callback_t,
    /// User context pointer passed to `response_callback`.
    pub context: *mut c_void,
    /// Pointer to parent node
    pub node: crate::node::nros_node_ref_t,
    /// Phase 193.4b — service-client QoS (applied to both request + reply
    /// endpoints). Defaults to the services profile
    /// (RELIABLE+VOLATILE+KEEP_LAST(10)); set via `nros_client_init_with_qos`.
    pub qos: crate::qos::nros_qos_t,
    /// Phase 189.M3.3.a — scheduling-context slot to bind the client's executor
    /// handle to. `0` = inherit the executor / Node default; set via
    /// `nros_client_init_with_options`. When non-zero,
    /// `nros_executor_register_client` binds the freshly-created handle to this
    /// SC after registration. No effect on the L1 polling path.
    pub sched_context_id: crate::executor::nros_sched_context_id_t,
    /// Internal state (arena entry index + executor pointer + timeout).
    /// Typed C-ABI handle field.
    pub _internal: ServiceClientInternal,
    /// Phase 122.3.c.5 — inline opaque storage for the L1 polling-mode
    /// `RawServiceClient<MESSAGE_BUFFER_SIZE, MESSAGE_BUFFER_SIZE>`.
    /// Zeroed in L2 (callback + executor arena) mode; populated by
    /// `nros_client_init_polling`.
    pub _opaque: [u64; SERVICE_CLIENT_OPAQUE_U64S],
}

impl Default for nros_client_t {
    fn default() -> Self {
        Self {
            state: nros_client_state_t::NROS_CLIENT_STATE_UNINITIALIZED,
            service_name: [0u8; MAX_SERVICE_NAME_LEN],
            service_name_len: 0,
            type_name: [0u8; MAX_TYPE_NAME_LEN],
            type_name_len: 0,
            type_hash: [0u8; MAX_TYPE_HASH_LEN],
            type_hash_len: 0,
            response_callback: None,
            context: ptr::null_mut(),
            node: crate::node::nros_node_ref_t::none(),
            qos: crate::qos::nros_qos_t::default(),
            sched_context_id: 0,
            _internal: ServiceClientInternal::new(),
            _opaque: [0u64; SERVICE_CLIENT_OPAQUE_U64S],
        }
    }
}

impl nros_client_t {
    /// Phase 193.4b — the client's QoS as `nros_node` settings.
    pub(crate) fn get_qos_settings(&self) -> nros_rmw::QoSProfile {
        self.qos.to_qos_settings()
    }
}

/// Get a zero-initialized client.
#[unsafe(no_mangle)]
pub extern "C" fn nros_client_get_zero_initialized() -> nros_client_t {
    nros_client_t::default()
}

/// Initialize a service client (Phase 82: metadata-only).
///
/// Stores the service name/type metadata and a `ServiceClientInternal`
/// blob; the actual transport handle (`RmwServiceClient`) is created
/// later by `nros_executor_add_client`. This deferred lifecycle matches
/// `nros_service_init` (server side) and the action client.
///
/// # Parameters
/// * `client` - Pointer to a zero-initialized client
/// * `node` - Pointer to an initialized node
/// * `type_info` - Pointer to service type information
/// * `service_name` - Service name (null-terminated string)
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if any required pointer is NULL
/// * `NROS_RET_NOT_INIT` if node is not initialized
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_init(
    client: *mut nros_client_t,
    node: *const nros_node_t,
    type_info: *const nros_service_type_t,
    service_name: *const c_char,
) -> nros_ret_t {
    validate_not_null!(client, node, type_info, service_name);

    let client = &mut *client;
    let node_ref = &*node;
    let type_info = &*type_info;

    validate_state!(
        client,
        nros_client_state_t::NROS_CLIENT_STATE_UNINITIALIZED,
        NROS_RET_BAD_SEQUENCE
    );
    validate_state!(node_ref, nros_node_state_t::NROS_NODE_STATE_INITIALIZED);

    // Copy service name (required — empty rejected)
    client.service_name_len = crate::util::copy_cstr_into(service_name, &mut client.service_name);
    if client.service_name_len == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }

    // Copy type name + hash (both optional — null sources leave dst untouched)
    client.type_name_len = crate::util::copy_cstr_into(type_info.type_name, &mut client.type_name);
    client.type_hash_len = crate::util::copy_cstr_into(type_info.type_hash, &mut client.type_hash);

    // Store node pointer + zero callback fields
    client.node = unsafe { crate::node::node_ref_of(node) };
    client.response_callback = None;
    client.context = ptr::null_mut();

    // Phase 193.4b — default to the services profile; nros_client_init_with_qos
    // overrides. (`nros_executor_register_client` reads this at registration.)
    client.qos = crate::qos::nros_qos_t::default();

    // Initialise the internal state (executor_ptr null until registration).
    client._internal = ServiceClientInternal::new();

    client.state = nros_client_state_t::NROS_CLIENT_STATE_INITIALIZED;
    NROS_RET_OK
}

/// Phase 193.4b — initialize a service client with an explicit QoS profile
/// (rclc's `_with_options`; the profile applies to both the request + reply
/// endpoints). `qos` NULL ⇒ the services default. Same as
/// [`nros_client_init`] otherwise.
///
/// # Safety
/// All non-NULL pointers must be valid + the node initialized.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_init_with_qos(
    client: *mut nros_client_t,
    node: *const nros_node_t,
    type_info: *const nros_service_type_t,
    service_name: *const c_char,
    qos: *const crate::qos::nros_qos_t,
) -> nros_ret_t {
    let ret = nros_client_init(client, node, type_info, service_name);
    if ret == NROS_RET_OK && !qos.is_null() {
        (*client).qos = *qos;
    }
    ret
}

/// Phase 189.M3.3.a — rclc-style named service-client options (mirrors
/// `nros_service_options_t`). QoS is passed separately; this carries the
/// non-QoS axes. Zero-init selects the default behaviour.
#[repr(C)]
#[derive(Default)]
pub struct nros_client_options_t {
    /// Scheduling-context slot to bind the client's executor handle to.
    /// `0` = inherit the executor / Node default. A non-zero value must be an id
    /// from `nros_executor_create_sched_context`; the bind is applied by
    /// `nros_executor_register_client` once the handle exists. No effect on L1.
    pub sched_context: crate::executor::nros_sched_context_id_t,
    /// Reserved for future use; must be zero. Pads for ABI stability.
    pub _reserved: [u8; 3],
}

/// Get a zero-initialised [`nros_client_options_t`] (`sched_context = 0`).
#[unsafe(no_mangle)]
pub extern "C" fn nros_client_get_default_options() -> nros_client_options_t {
    nros_client_options_t::default()
}

/// Phase 189.M3.3.a — initialize a service client with custom QoS + named
/// options. Like [`nros_client_init_with_qos`] except a non-zero
/// `options->sched_context` is stashed so [`nros_executor_register_client`]
/// binds the resulting executor handle to that scheduling context once known.
///
/// # Safety
/// All non-NULL pointers must be valid + the node initialized; `qos` / `options`
/// may be NULL.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_init_with_options(
    client: *mut nros_client_t,
    node: *const nros_node_t,
    type_info: *const nros_service_type_t,
    service_name: *const c_char,
    qos: *const crate::qos::nros_qos_t,
    options: *const nros_client_options_t,
) -> nros_ret_t {
    let ret = nros_client_init_with_qos(client, node, type_info, service_name, qos);
    if ret != NROS_RET_OK {
        return ret;
    }
    if !options.is_null() {
        (*client).sched_context_id = (*options).sched_context;
    }
    NROS_RET_OK
}

/// Finalize a service client.
///
/// Phase 82: the underlying transport handle lives in the executor's
/// arena and is dropped automatically when the executor is finalised.
/// This function only resets the C-side metadata + internal blob.
///
/// # Parameters
/// * `client` - Pointer to an initialized client
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if client is NULL
/// * `NROS_RET_NOT_INIT` if not initialized
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_fini(client: *mut nros_client_t) -> nros_ret_t {
    validate_not_null!(client);

    let client = &mut *client;

    if client.state == nros_client_state_t::NROS_CLIENT_STATE_UNINITIALIZED
        || client.state == nros_client_state_t::NROS_CLIENT_STATE_SHUTDOWN
    {
        return NROS_RET_NOT_INIT;
    }

    if client.state == nros_client_state_t::NROS_CLIENT_STATE_POLLING {
        // L1: drop the inline RawServiceClient so its Drop runs
        // (closes the underlying RMW client).
        #[cfg(feature = "rmw-cffi")]
        {
            core::ptr::drop_in_place(client._opaque.as_mut_ptr()
                as *mut nros_node::RawServiceClient<
                    { crate::config::MESSAGE_BUFFER_SIZE },
                    { crate::config::MESSAGE_BUFFER_SIZE },
                >);
            client._opaque = [0u64; SERVICE_CLIENT_OPAQUE_U64S];
        }
    }

    // Reset the inline ServiceClientInternal. The RmwServiceClient lives
    // in the executor's arena and is freed when the executor is destroyed.
    client._internal = ServiceClientInternal::new();
    client.response_callback = None;
    client.context = ptr::null_mut();
    client.node = crate::node::nros_node_ref_t::none();
    client.state = nros_client_state_t::NROS_CLIENT_STATE_SHUTDOWN;

    NROS_RET_OK
}

// ============================================================================
// Phase 122.3.c.5 — Layer-1 primitive entry points for service client
// ============================================================================

/// Phase 122.3.c.5 — initialize an L1 polling-mode service client.
///
/// Creates the underlying RMW client immediately and stores it inline
/// in the client's `_opaque` field. The caller drives the
/// request/reply cycle via `nros_client_send_request_raw` +
/// `nros_client_take_response_raw`.
///
/// # Parameters
/// * `client` - Pointer to a zero-initialized client
/// * `node` - Pointer to an initialized node
/// * `type_info` - Pointer to service type information
/// * `service_name` - Service name (null-terminated)
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if any pointer is NULL or name empty
/// * `NROS_RET_NOT_INIT` if node / support not initialized
/// * `NROS_RET_ERROR` if client creation failed
///
/// # Safety
/// All pointers must be valid; `service_name` must be a valid
/// null-terminated string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_init_polling(
    client: *mut nros_client_t,
    node: *const nros_node_t,
    type_info: *const nros_service_type_t,
    service_name: *const c_char,
) -> nros_ret_t {
    validate_not_null!(client, node, type_info, service_name);

    let client_mut = &mut *client;
    let node_ref = &*node;
    let type_info_ref = &*type_info;

    validate_state!(
        client_mut,
        nros_client_state_t::NROS_CLIENT_STATE_UNINITIALIZED,
        NROS_RET_BAD_SEQUENCE
    );
    validate_state!(node_ref, nros_node_state_t::NROS_NODE_STATE_INITIALIZED);

    client_mut.service_name_len =
        crate::util::copy_cstr_into(service_name, &mut client_mut.service_name);
    if client_mut.service_name_len == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }
    client_mut.type_name_len =
        crate::util::copy_cstr_into(type_info_ref.type_name, &mut client_mut.type_name);
    client_mut.type_hash_len =
        crate::util::copy_cstr_into(type_info_ref.type_hash, &mut client_mut.type_hash);

    client_mut.node = unsafe { crate::node::node_ref_of(node) };
    client_mut.response_callback = None;
    client_mut.context = ptr::null_mut();

    #[cfg(feature = "rmw-cffi")]
    {
        use nros_node::{ServiceInfo, Session};

        // Phase 156 Sub-bug D — multi-Session dispatch (see
        // `nros_publisher_init`).
        let (session, domain_id) = match crate::node::resolve_session_and_domain(node_ref) {
            Some(t) => t,
            None => return NROS_RET_NOT_INIT,
        };

        let service_str =
            core::str::from_utf8_unchecked(&client_mut.service_name[..client_mut.service_name_len]);
        let type_str =
            core::str::from_utf8_unchecked(&client_mut.type_name[..client_mut.type_name_len]);
        let type_hash_str =
            core::str::from_utf8_unchecked(&client_mut.type_hash[..client_mut.type_hash_len]);
        let node_name_str = core::str::from_utf8_unchecked(&node_ref.name[..node_ref.name_len]);
        let namespace_str =
            core::str::from_utf8_unchecked(&node_ref.namespace[..node_ref.namespace_len]);

        let info = ServiceInfo::new(service_str, type_str, type_hash_str)
            .with_domain(domain_id)
            .with_node_name(node_name_str)
            .with_namespace(namespace_str);

        match session.create_client(&info, nros_node::QoSProfile::services_default()) {
            Ok(handle) => {
                let raw = nros_node::RawServiceClient::<
                    { crate::config::MESSAGE_BUFFER_SIZE },
                    { crate::config::MESSAGE_BUFFER_SIZE },
                >::new(handle);
                core::ptr::write(
                    client_mut._opaque.as_mut_ptr()
                        as *mut nros_node::RawServiceClient<
                            { crate::config::MESSAGE_BUFFER_SIZE },
                            { crate::config::MESSAGE_BUFFER_SIZE },
                        >,
                    raw,
                );
            }
            Err(_) => return NROS_RET_ERROR,
        }
    }

    client_mut.state = nros_client_state_t::NROS_CLIENT_STATE_POLLING;
    NROS_RET_OK
}

/// Phase 122.3.c.5 — send a raw request on an L1 polling-mode client.
/// Non-blocking. Poll for the reply via
/// `nros_client_take_response_raw`.
///
/// # Safety
/// `client` must be in `POLLING` state. `data` readable for `len`
/// bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_send_request_raw(
    client: *mut nros_client_t,
    data: *const u8,
    len: usize,
) -> nros_ret_t {
    if client.is_null() || (data.is_null() && len != 0) {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let client_mut = &mut *client;
    if client_mut.state != nros_client_state_t::NROS_CLIENT_STATE_POLLING {
        return NROS_RET_INVALID_ARGUMENT;
    }

    #[cfg(feature = "rmw-cffi")]
    {
        let raw = &mut *(client_mut._opaque.as_mut_ptr()
            as *mut nros_node::RawServiceClient<
                { crate::config::MESSAGE_BUFFER_SIZE },
                { crate::config::MESSAGE_BUFFER_SIZE },
            >);
        let slice = core::slice::from_raw_parts(data, len);
        match raw.send_request_raw(slice) {
            Ok(()) => NROS_RET_OK,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (data, len);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.c.5 — non-blocking poll for a reply on an L1
/// polling-mode client. Writes reply bytes into the caller's `buf`.
///
/// # Returns
/// * `>= 0` — number of bytes written to `buf` (0 = no reply yet)
/// * `NROS_RET_INVALID_ARGUMENT` if pointers / state wrong
/// * `NROS_RET_ERROR` on transport failure
///
/// # Safety
/// `client` must be in `POLLING` state. `buf` writable for `buf_len`
/// bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_take_response_raw(
    client: *mut nros_client_t,
    buf: *mut u8,
    buf_len: usize,
) -> i32 {
    if client.is_null() || (buf.is_null() && buf_len != 0) {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let client_mut = &mut *client;
    if client_mut.state != nros_client_state_t::NROS_CLIENT_STATE_POLLING {
        return NROS_RET_INVALID_ARGUMENT;
    }

    #[cfg(feature = "rmw-cffi")]
    {
        let raw = &mut *(client_mut._opaque.as_mut_ptr()
            as *mut nros_node::RawServiceClient<
                { crate::config::MESSAGE_BUFFER_SIZE },
                { crate::config::MESSAGE_BUFFER_SIZE },
            >);
        match raw.take_response_raw() {
            Ok(Some(len)) => {
                let copy_len = len.min(buf_len);
                core::ptr::copy_nonoverlapping(raw.reply_buffer().as_ptr(), buf, copy_len);
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

// ============================================================================
// Service Client async pair + setters (Phase 82)
// ============================================================================

/// Response trampoline registered with the executor's arena entry.
///
/// Reads the user's `response_callback` from the `nros_client_t` struct
/// at invocation time so the blocking wrapper (`nros_client_call`) can
/// install a one-shot callback after registration.
///
/// # Safety
/// `context` must point to a live `nros_client_t`.
pub(crate) unsafe extern "C" fn client_response_trampoline(
    response: *const u8,
    response_len: usize,
    context: *mut c_void,
) {
    let client = &*(context as *const nros_client_t);
    if let Some(cb) = client.response_callback {
        cb(response, response_len, client.context);
    }
}

/// Set the response callback fired by `nros_executor_spin_some` when an
/// async request has its reply delivered.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_set_response_callback(
    client: *mut nros_client_t,
    callback: nros_response_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    validate_not_null!(client);
    let client = &mut *client;
    client.response_callback = callback;
    client.context = context;
    NROS_RET_OK
}

/// Set the default timeout used by `nros_client_call`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_set_timeout(
    client: *mut nros_client_t,
    timeout_ms: u32,
) -> nros_ret_t {
    validate_not_null!(client);
    let client = &mut *client;
    client._internal.timeout_ms = timeout_ms;
    NROS_RET_OK
}

/// Phase 124.C.3 — synchronous, graph-aware "is the matching server up?"
/// probe. Unlike [`nros_client_wait_for_service`] this never spins the
/// executor: it asks the active RMW backend whether at least one
/// matching server has been observed on the RMW graph and returns
/// immediately.
///
/// `out` receives `1` if a server is visible, `0` if none is yet,
/// `-1` if the backend cannot answer (e.g. XRCE without participant
/// enumeration). Callers that want the same answer rounded to a
/// hard yes/no should treat `-1` as "unknown — assume yes" or use
/// [`nros_client_wait_for_service`] instead.
///
/// # Returns
/// * `NROS_RET_OK` — `*out` was written (`0`, `1`, or `-1`).
/// * `NROS_RET_INVALID_ARGUMENT` — `client` or `out` is null.
/// * `NROS_RET_NOT_INIT` — client not registered with an executor.
/// * `NROS_RET_ERROR` — transport-level failure.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_server_available(
    client: *mut nros_client_t,
    out: *mut i32,
) -> nros_ret_t {
    validate_not_null!(client);
    if out.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }

    #[cfg(feature = "rmw-cffi")]
    {
        use nros_node::ClientTrait;

        let client_ref = &mut *client;
        if client_ref.state != nros_client_state_t::NROS_CLIENT_STATE_REGISTERED {
            return NROS_RET_NOT_INIT;
        }
        let internal = &mut client_ref._internal;
        if internal.executor_ptr.is_null() || internal.arena_entry_index < 0 {
            return NROS_RET_NOT_INIT;
        }
        let exec_t = &mut *(internal.executor_ptr as *mut nros_executor_t);
        let exec = crate::executor::get_executor(&mut exec_t._opaque);
        let entry = match exec.service_client_entry_mut(internal.arena_entry_index as usize) {
            Some(e) => e,
            None => return NROS_RET_NOT_INIT,
        };
        match entry.handle.service_is_ready() {
            Ok(true) => {
                *out = 1;
                NROS_RET_OK
            }
            Ok(false) => {
                *out = 0;
                NROS_RET_OK
            }
            Err(_) => {
                // Backend can't answer (Unsupported). Surface a sentinel
                // so callers can distinguish "no" (0) from "don't know"
                // (-1) without losing the OK status.
                *out = -1;
                NROS_RET_OK
            }
        }
    }

    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = client;
        *out = -1;
        NROS_RET_OK
    }
}

/// Block until a matching service server is discoverable, or `timeout_ms`
/// elapses. Mirrors `rclcpp::ClientBase::wait_for_service` and the
/// the underlying `Client::wait_for_service`.
///
/// The client must already have been registered with the executor via
/// `nros_executor_add_client`. Internally fires liveliness queries
/// against the matching service-server's wildcard liveliness keyexpr
/// and spins the executor cooperatively while the probe is in flight.
/// 1-second per-probe timeout, looped until either a token reply lands
/// or the outer wall-clock budget expires — see the runtime API for the
/// rationale (a single liveliness_get samples the router's current
/// token list and terminates, so a server that comes up after we
/// start waiting needs to be re-probed).
///
/// # Returns
/// * `NROS_RET_OK` — server is visible (proceed with `nros_client_call`).
/// * `NROS_RET_TIMEOUT` — `timeout_ms` elapsed without seeing a token.
/// * `NROS_RET_NOT_INIT` — client not registered with an executor.
/// * `NROS_RET_ERROR` — transport-level failure.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_wait_for_service(
    client: *mut nros_client_t,
    timeout_ms: u32,
) -> nros_ret_t {
    validate_not_null!(client);

    #[cfg(feature = "rmw-cffi")]
    {
        use nros_node::ClientTrait;

        let client_ref = &mut *client;
        if client_ref.state != nros_client_state_t::NROS_CLIENT_STATE_REGISTERED {
            return NROS_RET_NOT_INIT;
        }
        let internal = &mut client_ref._internal;
        if internal.executor_ptr.is_null() || internal.arena_entry_index < 0 {
            return NROS_RET_NOT_INIT;
        }

        let exec_t = &mut *(internal.executor_ptr as *mut nros_executor_t);
        if exec_t.in_dispatch {
            return NROS_RET_REENTRANT;
        }
        let executor = exec_t as *mut nros_executor_t;

        // Latched fast-path: if a previous wait already proved the
        // server is reachable, don't re-probe.
        {
            let exec = crate::executor::get_executor(&mut exec_t._opaque);
            let entry = match exec.service_client_entry_mut(internal.arena_entry_index as usize) {
                Some(e) => e,
                None => return NROS_RET_NOT_INIT,
            };
            // issue 1008 — `Ok(true)` only. `is_server_ready` defaulted to
            // `true`, so this returned OK without waiting on every backend.
            if matches!(entry.handle.service_is_ready(), Ok(true)) {
                return NROS_RET_OK;
            }
        }

        // Per-probe / outer budget. Mirrors `Client::wait_for_service` in
        // packages/core/nros-node/src/executor/handles.rs.
        const PROBE_TIMEOUT_MS: u32 = nros_node::SERVER_DISCOVERY_PROBE_TIMEOUT_MS; // issue #224
        let start_ns = crate::platform::get_time_ns();
        let timeout_ns: u64 = (timeout_ms as u64).saturating_mul(1_000_000);
        loop {
            // Re-borrow each iteration to avoid holding `entry` across
            // the executor spin (which itself touches the arena).
            {
                let exec = crate::executor::get_executor(&mut exec_t._opaque);
                let entry = match exec.service_client_entry_mut(internal.arena_entry_index as usize)
                {
                    Some(e) => e,
                    None => return NROS_RET_NOT_INIT,
                };
                if entry
                    .handle
                    .start_server_discovery(PROBE_TIMEOUT_MS)
                    .is_err()
                {
                    return NROS_RET_ERROR;
                }
            }

            // Drain this probe to completion (token reply or empty FINAL).
            loop {
                crate::executor::nros_executor_spin_some(executor, 10_000_000);

                let exec = crate::executor::get_executor(&mut exec_t._opaque);
                let entry = match exec.service_client_entry_mut(internal.arena_entry_index as usize)
                {
                    Some(e) => e,
                    None => return NROS_RET_NOT_INIT,
                };
                match entry.handle.poll_server_discovery() {
                    Ok(Some(true)) => return NROS_RET_OK,
                    Ok(Some(false)) => break, // probe finished empty — re-issue
                    Ok(None) => {}            // still in flight
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
        let _ = (client, timeout_ms);
        NROS_RET_OK
    }
}

/// Non-blocking snapshot of whether a matching service server is
/// currently visible. Mirrors `rclcpp::ClientBase::service_is_ready`
/// and rcl's `rcl_service_server_is_available`. Returns `false` when
/// the client isn't registered with an executor or the backend lacks
/// liveliness discovery (in which case use `nros_client_wait_for_service`
/// instead, which handles those cases conservatively).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_service_is_ready(client: *const nros_client_t) -> bool {
    if client.is_null() {
        return false;
    }
    #[cfg(feature = "rmw-cffi")]
    {
        use nros_node::ClientTrait;

        let client_ref = &*client;
        if client_ref.state != nros_client_state_t::NROS_CLIENT_STATE_REGISTERED {
            return false;
        }
        let internal = &client_ref._internal;
        if internal.executor_ptr.is_null() || internal.arena_entry_index < 0 {
            return false;
        }
        let exec_t = &mut *(internal.executor_ptr as *mut nros_executor_t);
        let exec = crate::executor::get_executor(&mut exec_t._opaque);
        match exec.service_client_entry_mut(internal.arena_entry_index as usize) {
            // issue 1008 — a backend that cannot answer reports NOT ready.
            Some(entry) => matches!(entry.handle.service_is_ready(), Ok(true)),
            None => false,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = client;
        true
    }
}

/// Send a service request asynchronously (Phase 82).
///
/// Non-blocking. The reply is delivered via the registered
/// `response_callback` during `nros_executor_spin_some`. The user must
/// have previously registered the client with `nros_executor_add_client`.
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_NOT_INIT` if the client is not registered with an executor
/// * `NROS_RET_BAD_SEQUENCE` if a previous request is still pending
/// * `NROS_RET_ERROR` on transport failure
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_send_request_async(
    client: *mut nros_client_t,
    request_data: *const u8,
    request_len: usize,
) -> nros_ret_t {
    validate_not_null!(client, request_data);

    #[cfg(feature = "rmw-cffi")]
    {
        use nros_node::ClientTrait;

        let client_ref = &mut *client;
        if client_ref.state != nros_client_state_t::NROS_CLIENT_STATE_REGISTERED {
            return NROS_RET_NOT_INIT;
        }

        let internal = &mut client_ref._internal;
        if internal.executor_ptr.is_null() || internal.arena_entry_index < 0 {
            return NROS_RET_NOT_INIT;
        }

        let exec_t = &mut *(internal.executor_ptr as *mut nros_executor_t);
        let exec = crate::executor::get_executor(&mut exec_t._opaque);
        let entry = match exec.service_client_entry_mut(internal.arena_entry_index as usize) {
            Some(e) => e,
            None => return NROS_RET_NOT_INIT,
        };
        if entry.pending {
            return NROS_RET_BAD_SEQUENCE;
        }

        let request = core::slice::from_raw_parts(request_data, request_len);
        // Clear the ready flag before sending so we don't pick up a
        // stale wake from a previous request.
        entry
            .reply_ready
            .store(false, core::sync::atomic::Ordering::Release);
        // Issue 0778 — the C client API is one-call-in-flight; the sequence
        // id is dropped here deliberately, not lost.
        match entry.handle.send_request_raw(request) {
            Ok(_seq) => {
                entry.pending = true;
                // Register a waker that sets reply_ready when the
                // transport delivers the reply. This replaces blind
                // polling of get_check on every spin tick.
                //
                // SAFETY (Phase 214.J.1 audit): `atomic_bool_waker`
                // requires that `&entry.reply_ready` outlives every
                // reachable clone of the returned Waker. `entry`
                // lives in the executor's service-client arena, which
                // is heap-allocated at session-open and only freed
                // when the executor is destroyed; the transport's
                // single-Waker slot is overwritten on each subsequent
                // `register_waker(...)`, bounding the clone lifetime
                // to the next call. Both # Safety invariants hold.
                let waker = unsafe { atomic_bool_waker(&entry.reply_ready) };
                entry.handle.register_waker(&waker);
                NROS_RET_OK
            }
            Err(_) => NROS_RET_ERROR,
        }
    }

    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (client, request_data, request_len);
        NROS_RET_ERROR
    }
}

/// Poll for the reply to the most recently sent async request.
///
/// # Returns
/// * `NROS_RET_OK` if the reply was filled into `response_data`
/// * `NROS_RET_TRY_AGAIN` if no reply yet (caller should spin and retry)
/// * `NROS_RET_NOT_INIT` if the client isn't registered or has no pending request
/// * `NROS_RET_ERROR` on transport failure
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_take_response(
    client: *mut nros_client_t,
    response_data: *mut u8,
    response_capacity: usize,
    response_len: *mut usize,
) -> nros_ret_t {
    validate_not_null!(client, response_data, response_len);

    #[cfg(feature = "rmw-cffi")]
    {
        use nros_node::ClientTrait;

        let client_ref = &mut *client;
        if client_ref.state != nros_client_state_t::NROS_CLIENT_STATE_REGISTERED {
            return NROS_RET_NOT_INIT;
        }

        let internal = &mut client_ref._internal;
        if internal.executor_ptr.is_null() || internal.arena_entry_index < 0 {
            return NROS_RET_NOT_INIT;
        }

        let exec_t = &mut *(internal.executor_ptr as *mut nros_executor_t);
        let exec = crate::executor::get_executor(&mut exec_t._opaque);
        let entry = match exec.service_client_entry_mut(internal.arena_entry_index as usize) {
            Some(e) => e,
            None => return NROS_RET_NOT_INIT,
        };
        if !entry.pending {
            return NROS_RET_NOT_INIT;
        }

        let buf = core::slice::from_raw_parts_mut(response_data, response_capacity);
        match entry.handle.take_response_raw(buf) {
            Ok(Some((len, _seq))) => {
                entry.pending = false;
                *response_len = len;
                NROS_RET_OK
            }
            Ok(None) => NROS_RET_TRY_AGAIN,
            Err(_) => {
                entry.pending = false;
                NROS_RET_ERROR
            }
        }
    }

    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (client, response_data, response_capacity, response_len);
        NROS_RET_ERROR
    }
}

/// Call a service (blocking convenience over the async pair).
///
/// Phase 82: signature unchanged, but no longer blocks at the transport
/// layer. Internally calls `nros_client_send_request_async` and spins
/// the registered executor via `nros_executor_spin_some` until the
/// response arrives or the client's `timeout_ms` elapses. The client
/// must have been registered with `nros_executor_add_client`.
///
/// # Parameters
/// * `client` - Pointer to a registered client
/// * `request_data` - CDR-serialized request data
/// * `request_len` - Length of request data
/// * `response_data` - Buffer to receive CDR-serialized response
/// * `response_capacity` - Capacity of response buffer
/// * `response_len` - Output: actual length of response data
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if any pointer is NULL
/// * `NROS_RET_NOT_INIT` if the client isn't registered with an executor
/// * `NROS_RET_TIMEOUT` if no response within `timeout_ms`
/// * `NROS_RET_ERROR` on call failure
#[unsafe(no_mangle)]
#[allow(static_mut_refs)]
pub unsafe extern "C" fn nros_client_call(
    client: *mut nros_client_t,
    request_data: *const u8,
    request_len: usize,
    response_data: *mut u8,
    response_capacity: usize,
    response_len: *mut usize,
) -> nros_ret_t {
    validate_not_null!(client, request_data, response_data, response_len);

    let client_ref = &mut *client;
    if client_ref.state != nros_client_state_t::NROS_CLIENT_STATE_REGISTERED {
        return NROS_RET_NOT_INIT;
    }

    let executor_ptr = client_ref._internal.executor_ptr;
    let timeout_ms = client_ref._internal.timeout_ms;
    if executor_ptr.is_null() {
        return NROS_RET_NOT_INIT;
    }

    // Reentrancy guard: nros_client_call spins the executor internally,
    // so it must not be called from inside a dispatch callback.
    let exec_t = &*(executor_ptr as *const nros_executor_t);
    if exec_t.in_dispatch {
        return NROS_RET_REENTRANT;
    }

    // One-shot blocking response capture. Static is fine because
    // nros_client_call is non-reentrant by design (callable only from
    // outside spin_some; the reentrancy guard in 82.8 will enforce it).
    //
    // Phase 214.G.1 — replaced `static mut BLK_DONE/BLK_LEN/BLK_BUF`
    // with atomics + `Sync`-asserting `UnsafeCell` so the callback +
    // spin loop have an explicit happens-before: the callback's
    // Release store on `BLK_DONE` publishes the buffer write, and
    // the loop's Acquire load fences the buffer read.
    const BLK_BUF_LEN: usize = 4096; // max captured service-response CDR
    use core::{
        cell::UnsafeCell,
        sync::atomic::{AtomicI32, AtomicUsize, Ordering},
    };
    struct BlkBuf(UnsafeCell<[u8; BLK_BUF_LEN]>);
    // SAFETY: cross-thread access gated by BLK_DONE's Release/Acquire
    // pair; non-reentrancy guarantees a single in-flight caller.
    unsafe impl Sync for BlkBuf {}
    static BLK_DONE: AtomicI32 = AtomicI32::new(-1);
    static BLK_LEN: AtomicUsize = AtomicUsize::new(0);
    static BLK_BUF: BlkBuf = BlkBuf(UnsafeCell::new([0u8; BLK_BUF_LEN]));
    BLK_DONE.store(-1, Ordering::Relaxed);
    BLK_LEN.store(0, Ordering::Relaxed);

    let orig_cb = client_ref.response_callback;
    let orig_ctx = client_ref.context;

    unsafe extern "C" fn blocking_response_cb(data: *const u8, len: usize, _ctx: *mut c_void) {
        let copy = len.min(BLK_BUF_LEN);
        // SAFETY: see BlkBuf Sync assertion above.
        unsafe {
            core::ptr::copy_nonoverlapping(data, (*BLK_BUF.0.get()).as_mut_ptr(), copy);
        }
        BLK_LEN.store(copy, Ordering::Relaxed);
        // Release: publish buffer + LEN to any loop observing DONE ≥ 0.
        BLK_DONE.store(1, Ordering::Release);
    }
    client_ref.response_callback = Some(blocking_response_cb);

    let send = nros_client_send_request_async(client, request_data, request_len);
    if send != NROS_RET_OK {
        client_ref.response_callback = orig_cb;
        client_ref.context = orig_ctx;
        return send;
    }

    // Spin: drive I/O then dispatch arena entries. On single-threaded
    // transports (smoltcp/NuttX), drive_io reads from the socket. On
    // multi-threaded (POSIX), the background thread handles I/O and
    // the waker signals reply_ready when the response arrives.
    //
    // Phase 89.2: wall-clock budgeting instead of `max_spins = timeout_ms/10`.
    // On multi-threaded zpico backends (POSIX/Zephyr) the condvar-wait in
    // `zpico_spin_once(10)` can return early on any incoming frame
    // (keep-alives, discovery gossip, …), so a pure iteration count can
    // burn through the nominal timeout in milliseconds and return TIMEOUT
    // before the reply actually has a chance to arrive. Budget by the clock.
    let executor = executor_ptr as *mut nros_executor_t;
    let start_ns = crate::platform::get_time_ns();
    let timeout_ns: u64 = (timeout_ms as u64).saturating_mul(1_000_000);
    // Resend the request periodically until the reply arrives. The first
    // request can be published before the backend's request writer is
    // matched to the server's reader — the RTPS discovery race on XRCE
    // (agent bridges each session to its own DDS participant; reliable +
    // volatile drops a request published pre-match) and the cold-boot
    // server case on NuttX both lose the first request. Resending lets a
    // later attempt land once discovery completes, within the timeout.
    let resend_interval_ns: u64 = 500_000_000;
    let mut last_send_ns = start_ns;
    loop {
        crate::executor::nros_executor_spin_some(executor, 10_000_000);
        if BLK_DONE.load(Ordering::Acquire) >= 0 {
            client_ref.response_callback = orig_cb;
            client_ref.context = orig_ctx;
            let blk_len = BLK_LEN.load(Ordering::Relaxed);
            if blk_len > response_capacity {
                return NROS_RET_ERROR;
            }
            // SAFETY: BLK_DONE's Acquire load above fences the buffer
            // write in `blocking_response_cb`.
            let buf_ptr = unsafe { (*BLK_BUF.0.get()).as_ptr() };
            core::ptr::copy_nonoverlapping(buf_ptr, response_data, blk_len);
            *response_len = blk_len;
            return NROS_RET_OK;
        }
        let now_ns = crate::platform::get_time_ns();
        if now_ns.saturating_sub(start_ns) >= timeout_ns {
            break;
        }
        if now_ns.saturating_sub(last_send_ns) >= resend_interval_ns {
            // Clear `pending` so the resend isn't rejected with
            // BAD_SEQUENCE, then re-issue the same request.
            {
                let internal = &mut client_ref._internal;
                if !internal.executor_ptr.is_null() && internal.arena_entry_index >= 0 {
                    let exec_t2 = &mut *(internal.executor_ptr as *mut nros_executor_t);
                    let exec2 = crate::executor::get_executor(&mut exec_t2._opaque);
                    if let Some(entry) =
                        exec2.service_client_entry_mut(internal.arena_entry_index as usize)
                    {
                        entry.pending = false;
                    }
                }
            }
            let _ = nros_client_send_request_async(client, request_data, request_len);
            last_send_ns = now_ns;
        }
    }
    client_ref.response_callback = orig_cb;
    client_ref.context = orig_ctx;

    // Phase 89.12: clear `entry.pending` (set by `nros_client_send_request_async`
    // at line ~763) so the next `nros_client_call` doesn't bounce off
    // NROS_RET_BAD_SEQUENCE. Without this, a single slow-first-RPC
    // timeout cascades every subsequent blocking call on the same
    // client — which is exactly what NuttX `lang_2_C` rtos_e2e flakes
    // hit on QEMU cold-boot: call [1] times out because the server
    // queryable isn't ready, calls [2–4] all return BAD_SEQUENCE, the
    // test sees 0 responses and fails even though the server came up
    // fine. Symmetrical to the RAII-style reset on Rust's `Promise`
    // drop path (handles.rs::Promise::take clears `in_flight` on
    // successful reception) — we reset here on the timeout path.
    //
    // Semantic note: if the late reply for the timed-out call arrives
    // before the caller fires another request, it will be picked up
    // by the next `nros_client_take_response` / spin dispatch.
    // That's a known "stale reply" quirk of single-slot clients; the
    // caller either tolerates it (match on returned seq) or resets
    // the slot explicitly. The previous behaviour — silently jamming
    // every subsequent call — was strictly worse.
    let internal = &mut client_ref._internal;
    if !internal.executor_ptr.is_null() && internal.arena_entry_index >= 0 {
        let exec_t = &mut *(internal.executor_ptr as *mut nros_executor_t);
        let exec = crate::executor::get_executor(&mut exec_t._opaque);
        if let Some(entry) = exec.service_client_entry_mut(internal.arena_entry_index as usize) {
            entry.pending = false;
            entry
                .reply_ready
                .store(false, core::sync::atomic::Ordering::Release);
        }
    }

    NROS_RET_TIMEOUT
}

/// Get the service name of a client.
///
/// # Parameters
/// * `client` - Pointer to a client
///
/// # Returns
/// * Pointer to service name (null-terminated), or NULL if invalid
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_get_service_name(
    client: *const nros_client_t,
) -> *const c_char {
    if client.is_null() {
        return ptr::null();
    }

    let client = &*client;
    if client.state != nros_client_state_t::NROS_CLIENT_STATE_INITIALIZED {
        return ptr::null();
    }

    client.service_name.as_ptr() as *const c_char
}

/// Check if client is valid (initialized).
///
/// # Parameters
/// * `client` - Pointer to a client
///
/// # Returns
/// * `true` if valid, `false` if invalid or NULL
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_client_is_valid(client: *const nros_client_t) -> bool {
    if client.is_null() {
        return false;
    }

    let client = &*client;
    client.state == nros_client_state_t::NROS_CLIENT_STATE_INITIALIZED
}

// ============================================================================
// Kani Verification
// ============================================================================

#[cfg(kani)]
mod verification {
    use super::*;
    use crate::error::*;
    use core::ptr;

    // Helper to create a dummy type_info
    fn dummy_service_type() -> nros_service_type_t {
        let type_name = b"example_interfaces::srv::dds_::AddTwoInts_\0";
        let type_hash = b"RIHS01_test\0";
        nros_service_type_t {
            type_name: type_name.as_ptr() as *const core::ffi::c_char,
            type_hash: type_hash.as_ptr() as *const core::ffi::c_char,
        }
    }

    // Helper callback for service init
    unsafe extern "C" fn dummy_callback(
        _req: *const u8,
        _req_len: usize,
        _resp: *mut u8,
        _resp_cap: usize,
        _resp_len: *mut usize,
        _ctx: *mut core::ffi::c_void,
    ) -> bool {
        true
    }

    // -- Service Server Harnesses --

    #[kani::proof]
    #[kani::unwind(5)]
    fn service_init_null_ptrs() {
        let svc_name = b"/add_two_ints\0";
        let type_info = dummy_service_type();
        let mut node = crate::node::nros_node_get_zero_initialized();

        // NULL service
        assert_eq!(
            unsafe {
                nros_service_init(
                    ptr::null_mut(),
                    &node,
                    &type_info,
                    svc_name.as_ptr() as *const core::ffi::c_char,
                    Some(dummy_callback),
                    ptr::null_mut(),
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL node
        let mut svc = nros_service_get_zero_initialized();
        assert_eq!(
            unsafe {
                nros_service_init(
                    &mut svc,
                    ptr::null(),
                    &type_info,
                    svc_name.as_ptr() as *const core::ffi::c_char,
                    Some(dummy_callback),
                    ptr::null_mut(),
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL type_info
        let mut svc = nros_service_get_zero_initialized();
        assert_eq!(
            unsafe {
                nros_service_init(
                    &mut svc,
                    &node,
                    ptr::null(),
                    svc_name.as_ptr() as *const core::ffi::c_char,
                    Some(dummy_callback),
                    ptr::null_mut(),
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL service_name
        let mut svc = nros_service_get_zero_initialized();
        assert_eq!(
            unsafe {
                nros_service_init(
                    &mut svc,
                    &node,
                    &type_info,
                    ptr::null(),
                    Some(dummy_callback),
                    ptr::null_mut(),
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );
    }

    // Phase 189.M3.3.a — `init_with_options` stashes the requested sched-context
    // slot on the service (the executor applies the bind at register time). A
    // NULL options pointer leaves the default (0 = inherit).
    #[kani::proof]
    #[kani::unwind(5)]
    fn service_init_with_options_stashes_sched_context() {
        let svc_name = b"/add_two_ints\0";
        let type_info = dummy_service_type();
        let mut node = crate::node::nros_node_get_zero_initialized();
        node.state = crate::node::nros_node_state_t::NROS_NODE_STATE_INITIALIZED;

        let requested: crate::executor::nros_sched_context_id_t = 7;
        let options = nros_service_options_t {
            sched_context: requested,
            _reserved: [0; 3],
        };
        let mut svc = nros_service_get_zero_initialized();
        let ret = unsafe {
            nros_service_init_with_options(
                &mut svc,
                &node,
                &type_info,
                svc_name.as_ptr() as *const core::ffi::c_char,
                Some(dummy_callback),
                ptr::null_mut(),
                ptr::null(),
                &options,
            )
        };
        assert_eq!(ret, NROS_RET_OK);
        assert_eq!(svc.sched_context_id, requested);

        // NULL options ⇒ default (inherit).
        let mut svc2 = nros_service_get_zero_initialized();
        let ret2 = unsafe {
            nros_service_init_with_options(
                &mut svc2,
                &node,
                &type_info,
                svc_name.as_ptr() as *const core::ffi::c_char,
                Some(dummy_callback),
                ptr::null_mut(),
                ptr::null(),
                ptr::null(),
            )
        };
        assert_eq!(ret2, NROS_RET_OK);
        assert_eq!(svc2.sched_context_id, 0);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn service_init_none_callback() {
        let svc_name = b"/add_two_ints\0";
        let type_info = dummy_service_type();
        let node = crate::node::nros_node_get_zero_initialized();

        let mut svc = nros_service_get_zero_initialized();
        assert_eq!(
            unsafe {
                nros_service_init(
                    &mut svc,
                    &node,
                    &type_info,
                    svc_name.as_ptr() as *const core::ffi::c_char,
                    None,
                    ptr::null_mut(),
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn service_init_uninit_node() {
        let svc_name = b"/add_two_ints\0";
        let type_info = dummy_service_type();
        let node = crate::node::nros_node_get_zero_initialized();

        // Node is UNINITIALIZED → NOT_INIT
        let mut svc = nros_service_get_zero_initialized();
        assert_eq!(
            unsafe {
                nros_service_init(
                    &mut svc,
                    &node,
                    &type_info,
                    svc_name.as_ptr() as *const core::ffi::c_char,
                    Some(dummy_callback),
                    ptr::null_mut(),
                )
            },
            NROS_RET_NOT_INIT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn service_zero_initialized_state() {
        let svc = nros_service_get_zero_initialized();
        assert_eq!(
            svc.state,
            nros_service_state_t::NROS_SERVICE_STATE_UNINITIALIZED,
        );
        assert!(!svc.node.is_bound(), "fini must unbind the node reference");
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn service_fini_null_safety() {
        // NULL → INVALID_ARGUMENT
        assert_eq!(
            unsafe { nros_service_fini(ptr::null_mut()) },
            NROS_RET_INVALID_ARGUMENT,
        );

        // UNINITIALIZED → NOT_INIT
        let mut svc = nros_service_get_zero_initialized();
        assert_eq!(unsafe { nros_service_fini(&mut svc) }, NROS_RET_NOT_INIT,);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn service_double_init_rejected() {
        let svc_name = b"/add_two_ints\0";
        let type_info = dummy_service_type();
        let mut node = crate::node::nros_node_get_zero_initialized();
        // Manually set node to initialized state for this test
        node.state = crate::node::nros_node_state_t::NROS_NODE_STATE_INITIALIZED;

        let mut svc = nros_service_get_zero_initialized();
        // First init succeeds (metadata only)
        let ret = unsafe {
            nros_service_init(
                &mut svc,
                &node,
                &type_info,
                svc_name.as_ptr() as *const core::ffi::c_char,
                Some(dummy_callback),
                ptr::null_mut(),
            )
        };
        assert_eq!(ret, NROS_RET_OK);

        // Second init → BAD_SEQUENCE
        assert_eq!(
            unsafe {
                nros_service_init(
                    &mut svc,
                    &node,
                    &type_info,
                    svc_name.as_ptr() as *const core::ffi::c_char,
                    Some(dummy_callback),
                    ptr::null_mut(),
                )
            },
            NROS_RET_BAD_SEQUENCE,
        );
    }

    // -- Service Client Harnesses --

    // Phase 189.M3.3.a — client `init_with_options` mirrors the service: stash
    // the requested sched-context slot (bind applied at register time).
    #[kani::proof]
    #[kani::unwind(5)]
    fn client_init_with_options_stashes_sched_context() {
        let svc_name = b"/add_two_ints\0";
        let type_info = dummy_service_type();
        let mut node = crate::node::nros_node_get_zero_initialized();
        node.state = crate::node::nros_node_state_t::NROS_NODE_STATE_INITIALIZED;

        let options = nros_client_options_t {
            sched_context: 5,
            _reserved: [0; 3],
        };
        let mut client = nros_client_get_zero_initialized();
        let ret = unsafe {
            nros_client_init_with_options(
                &mut client,
                &node,
                &type_info,
                svc_name.as_ptr() as *const core::ffi::c_char,
                ptr::null(),
                &options,
            )
        };
        assert_eq!(ret, NROS_RET_OK);
        assert_eq!(client.sched_context_id, 5);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn client_init_null_ptrs() {
        let svc_name = b"/add_two_ints\0";
        let type_info = dummy_service_type();
        let node = crate::node::nros_node_get_zero_initialized();

        // NULL client
        assert_eq!(
            unsafe {
                nros_client_init(
                    ptr::null_mut(),
                    &node,
                    &type_info,
                    svc_name.as_ptr() as *const core::ffi::c_char,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL node
        let mut client = nros_client_get_zero_initialized();
        assert_eq!(
            unsafe {
                nros_client_init(
                    &mut client,
                    ptr::null(),
                    &type_info,
                    svc_name.as_ptr() as *const core::ffi::c_char,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL type_info
        let mut client = nros_client_get_zero_initialized();
        assert_eq!(
            unsafe {
                nros_client_init(
                    &mut client,
                    &node,
                    ptr::null(),
                    svc_name.as_ptr() as *const core::ffi::c_char,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL service_name
        let mut client = nros_client_get_zero_initialized();
        assert_eq!(
            unsafe { nros_client_init(&mut client, &node, &type_info, ptr::null()) },
            NROS_RET_INVALID_ARGUMENT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn client_init_uninit_node() {
        let svc_name = b"/add_two_ints\0";
        let type_info = dummy_service_type();
        let node = crate::node::nros_node_get_zero_initialized();

        let mut client = nros_client_get_zero_initialized();
        assert_eq!(
            unsafe {
                nros_client_init(
                    &mut client,
                    &node,
                    &type_info,
                    svc_name.as_ptr() as *const core::ffi::c_char,
                )
            },
            NROS_RET_NOT_INIT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn client_zero_initialized_state() {
        let client = nros_client_get_zero_initialized();
        assert_eq!(
            client.state,
            nros_client_state_t::NROS_CLIENT_STATE_UNINITIALIZED,
        );
        assert!(
            !client.node.is_bound(),
            "fini must unbind the node reference"
        );
        assert!(client._internal.iter().all(|&v| v == 0));
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn client_fini_null_safety() {
        // NULL → INVALID_ARGUMENT
        assert_eq!(
            unsafe { nros_client_fini(ptr::null_mut()) },
            NROS_RET_INVALID_ARGUMENT,
        );

        // UNINITIALIZED → NOT_INIT
        let mut client = nros_client_get_zero_initialized();
        assert_eq!(unsafe { nros_client_fini(&mut client) }, NROS_RET_NOT_INIT,);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn client_call_null_safety() {
        let req = [0u8; 8];
        let mut resp = [0u8; 8];
        let mut resp_len: usize = 0;

        // NULL client
        assert_eq!(
            unsafe {
                nros_client_call(
                    ptr::null_mut(),
                    req.as_ptr(),
                    req.len(),
                    resp.as_mut_ptr(),
                    resp.len(),
                    &mut resp_len,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL request_data
        let mut client = nros_client_get_zero_initialized();
        assert_eq!(
            unsafe {
                nros_client_call(
                    &mut client,
                    ptr::null(),
                    0,
                    resp.as_mut_ptr(),
                    resp.len(),
                    &mut resp_len,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL response_data
        assert_eq!(
            unsafe {
                nros_client_call(
                    &mut client,
                    req.as_ptr(),
                    req.len(),
                    ptr::null_mut(),
                    0,
                    &mut resp_len,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL response_len
        assert_eq!(
            unsafe {
                nros_client_call(
                    &mut client,
                    req.as_ptr(),
                    req.len(),
                    resp.as_mut_ptr(),
                    resp.len(),
                    ptr::null_mut(),
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn client_call_reentrant_rejected() {
        let svc_name = b"/add_two_ints\0";
        let type_info = dummy_service_type();
        let mut node = crate::node::nros_node_get_zero_initialized();
        node.state = crate::node::nros_node_state_t::NROS_NODE_STATE_INITIALIZED;

        let mut client = nros_client_get_zero_initialized();
        let ret = unsafe {
            nros_client_init(
                &mut client,
                &node,
                &type_info,
                svc_name.as_ptr() as *const core::ffi::c_char,
            )
        };
        assert_eq!(ret, NROS_RET_OK);

        // Simulate registration: set state to REGISTERED and stash
        // a pointer to a fake executor with in_dispatch = true.
        let mut executor = crate::executor::nros_executor_get_zero_initialized();
        executor.state = crate::executor::nros_executor_state_t::NROS_EXECUTOR_STATE_INITIALIZED;
        executor.in_dispatch = true;

        client.state = nros_client_state_t::NROS_CLIENT_STATE_REGISTERED;
        client._internal.executor_ptr = &mut executor as *mut _ as *mut core::ffi::c_void;
        client._internal.timeout_ms = NROS_DEFAULT_SERVICE_TIMEOUT_MS;

        let req = [0u8; 8];
        let mut resp = [0u8; 8];
        let mut resp_len: usize = 0;

        assert_eq!(
            unsafe {
                nros_client_call(
                    &mut client,
                    req.as_ptr(),
                    req.len(),
                    resp.as_mut_ptr(),
                    resp.len(),
                    &mut resp_len,
                )
            },
            NROS_RET_REENTRANT,
        );
    }

    // -- Name Getter Harnesses --

    #[kani::proof]
    #[kani::unwind(5)]
    fn service_name_getter_null() {
        let result = unsafe { nros_service_get_service_name(ptr::null()) };
        assert!(result.is_null());
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn client_name_getter_null() {
        let result = unsafe { nros_client_get_service_name(ptr::null()) };
        assert!(result.is_null());
    }
}
