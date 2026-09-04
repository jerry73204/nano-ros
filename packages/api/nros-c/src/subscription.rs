//! Subscription API for nros C API.
//!
//! Subscriptions receive messages from topics that publishers send to.

use core::{
    ffi::{c_char, c_void},
    ptr,
};

use crate::{
    constants::{MAX_TOPIC_LEN, MAX_TYPE_HASH_LEN, MAX_TYPE_NAME_LEN},
    error::*,
    node::{nros_node_state_t, nros_node_t},
    opaque_sizes::SUBSCRIPTION_OPAQUE_U64S,
    publisher::nros_message_type_t,
    qos::nros_qos_t,
};

/// Subscription callback function type.
///
/// # Parameters
/// * `data` - Pointer to received CDR-serialized message data
/// * `len` - Length of data in bytes
/// * `context` - User-provided context pointer
pub type nros_subscription_callback_t =
    Option<unsafe extern "C" fn(data: *const u8, len: usize, context: *mut c_void)>;

/// Subscription callback that also receives the sample's wire **attachment**
/// (Phase 189.M3.4 — the C analog of the Rust
/// `node.subscription(t).generic(..).message_info()` builder path). Used by
/// [`nros_executor_add_subscription_raw_with_info`].
///
/// # Parameters
/// * `data` / `len` — received CDR bytes.
/// * `attachment` / `attachment_len` — the sample's wire attachment
///   (`attachment_len == 0` ⇒ none). Cross-RMW bridges read the
///   `bridge_origin` tag from it. Valid only during the call.
/// * `context` — user-provided context pointer.
pub type nros_subscription_info_callback_t = Option<
    unsafe extern "C" fn(
        data: *const u8,
        len: usize,
        attachment: *const u8,
        attachment_len: usize,
        context: *mut c_void,
    ),
>;

/// Subscription state
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_subscription_state_t {
    /// Not initialized
    NROS_SUBSCRIPTION_STATE_UNINITIALIZED = 0,
    /// Initialized for callback dispatch (Layer 2). Subscriber entity
    /// creation is deferred to `nros_executor_add_subscription`.
    NROS_SUBSCRIPTION_STATE_INITIALIZED = 1,
    /// Shutdown
    NROS_SUBSCRIPTION_STATE_SHUTDOWN = 2,
    /// Phase 122.3.b — initialized for primitive-mode polling (Layer 1).
    /// Subscriber entity created at init time and stored inline in
    /// `_opaque`; caller drains via `nros_subscription_take_serialized`.
    /// No executor registration.
    NROS_SUBSCRIPTION_STATE_POLLING = 3,
}

/// Subscription structure.
#[repr(C)]
pub struct nros_subscription_t {
    /// Current state
    pub state: nros_subscription_state_t,
    /// Topic name storage
    pub topic_name: [u8; MAX_TOPIC_LEN],
    /// Topic name length
    pub topic_name_len: usize,
    /// Type name storage
    pub type_name: [u8; MAX_TYPE_NAME_LEN],
    /// Type name length
    pub type_name_len: usize,
    /// Type hash storage
    pub type_hash: [u8; MAX_TYPE_HASH_LEN],
    /// Type hash length
    pub type_hash_len: usize,
    /// User callback function
    pub callback: nros_subscription_callback_t,
    /// User context pointer
    pub context: *mut c_void,
    /// Pointer to parent node
    pub node: crate::node::nros_node_ref_t,
    /// QoS settings (stored during init, used by executor registration)
    pub qos: crate::qos::nros_qos_t,
    /// Handle ID from executor registration (SIZE_MAX = not registered)
    pub handle_id: usize,
    /// Phase 189.M3 — scheduling-context slot requested via
    /// [`nros_subscription_init_with_options`]. `0` = inherit the
    /// executor / Node default (no explicit bind), matching the
    /// `nros_node_options_t::sched_context_id` convention. When non-zero,
    /// `nros_executor_add_subscription` binds the freshly-created
    /// handle to this SC after registration. Has no effect on the L1
    /// polling path (no executor handle to bind).
    pub sched_context_id: crate::executor::nros_sched_context_id_t,
    /// Phase 122.3.b — inline opaque storage for the L1 polling-mode
    /// `RawSubscription<MESSAGE_BUFFER_SIZE>`. Zeroed in callback (L2)
    /// mode; populated by `nros_subscription_init_polling`.
    pub _opaque: [u64; SUBSCRIPTION_OPAQUE_U64S],
}

impl Default for nros_subscription_t {
    fn default() -> Self {
        Self {
            state: nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_UNINITIALIZED,
            topic_name: [0u8; MAX_TOPIC_LEN],
            topic_name_len: 0,
            type_name: [0u8; MAX_TYPE_NAME_LEN],
            type_name_len: 0,
            type_hash: [0u8; MAX_TYPE_HASH_LEN],
            type_hash_len: 0,
            callback: None,
            context: ptr::null_mut(),
            node: crate::node::nros_node_ref_t::none(),
            qos: crate::qos::nros_qos_t::default(),
            handle_id: usize::MAX,
            sched_context_id: 0,
            _opaque: [0u64; SUBSCRIPTION_OPAQUE_U64S],
        }
    }
}

/// Get a zero-initialized subscription.
#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_subscription() -> nros_subscription_t {
    nros_subscription_t::default()
}

/// Phase 189.M3 — rclc-style named subscription options.
///
/// Sits ALONGSIDE the QoS profile (rclc convention): QoS is passed
/// separately, this struct carries the non-QoS subscription-creation
/// axes. The struct holds only plain scalar fields — no pointers — so it
/// is safe to stack-allocate, memcpy, and pass across the FFI. Zero-init
/// (all fields 0) selects the default behaviour, identical to
/// `nros_subscription_init_with_qos`.
#[repr(C)]
#[derive(Default)]
pub struct nros_subscription_options_t {
    /// Scheduling-context slot to bind the subscription's executor
    /// handle to. `0` = inherit the executor / Node default (no explicit
    /// bind), matching the `nros_node_options_t::sched_context_id`
    /// convention. A non-zero value must be an id previously returned
    /// from `nros_executor_create_sched_context`; the bind is applied by
    /// `nros_executor_add_subscription` once the handle exists. Has
    /// no effect on the L1 polling path.
    pub sched_context: crate::executor::nros_sched_context_id_t,
    /// Reserved — needs a with-info arena path (Phase 189.M3.4), not yet
    /// wired. When a future M3.4 lands this will request the
    /// message-info delivery variant (sample identity + reception
    /// timestamp alongside the payload). Setting it to a non-zero value
    /// today is accepted but ignored. Treat as bool (0 = off).
    pub message_info: u8,
    /// Reserved for future use; must be zero. Pads the struct for ABI
    /// stability so later axes can be added without a layout break.
    pub _reserved: [u8; 2],
}

/// Get a zero-initialised [`nros_subscription_options_t`].
///
/// All fields default to "inherit"/"off": `sched_context = 0` (executor
/// default), `message_info = 0` (reserved, off). Callers populate only
/// the fields they want before passing the struct to
/// [`nros_subscription_init_with_options`].
#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_get_default_options() -> nros_subscription_options_t {
    nros_subscription_options_t::default()
}

/// Initialize a subscription with default QoS (RELIABLE, KEEP_LAST(10)).
///
/// rclc's `rclc_subscription_init_default`, in rclc's argument order and at
/// rclc's ARITY (phase-417 stage 6). The old `nros_subscription_init` carried
/// two extra arguments — `callback` and `context` — and RFC-0087 records why
/// they are gone from here:
///
/// * the binding site was never mandated. RFC-0041 is about the DISPATCH
///   MODEL, not about which call first supplies a callback, and
///   `executor-owns-no-entity-storage` — cited by name in ten ledger rows as
///   the reason — is defined nowhere in `docs/design/`;
/// * the only real constraint found (`c:timer_exchange_callback`) forbids
///   SWAPPING a callback on a live entity, which registration-time binding
///   does not do.
///
/// So the callback is supplied where rclc supplies it, at registration:
/// [`nros_executor_add_subscription_typed`] for the typed path (rclc's
/// `rclc_executor_add_subscription_with_context` shape) and
/// `nros_executor_add_subscription_raw` for the byte path, which has no rclc
/// counterpart and therefore keeps an `nros_` name.
///
/// The typesupport parameter stays `const nros_message_type_t *` rather than
/// taking `rosidl_message_type_support_t`'s name: a rosidl typesupport's
/// MEMBERS are its contract, including the `func` dispatcher we do not have,
/// so adopting the name would claim a structure we lack (RFC-0087, settled
/// 2026-09-04). It costs a ported call site nothing — the argument comes from
/// our codegen either way.
///
/// The deprecated six-argument `nros_subscription_init` survives as an
/// `NROS_DEPRECATED_MSG` `static inline` in `<nros/subscription.h>`; it
/// forwards to [`nros_subscription_init_with_qos`], which still carries the
/// callback, so the old behaviour is preserved exactly.
///
/// # Parameters
/// * `subscription` - Pointer to a zero-initialized subscription
/// * `node` - Pointer to an initialized node
/// * `type_info` - Pointer to message type information
/// * `topic_name` - Topic name (null-terminated string)
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if any required pointer is NULL
/// * `NROS_RET_NOT_INIT` if node is not initialized
/// * `NROS_RET_ERROR` on initialization failure
///
/// # Safety
/// * All required pointers must be valid
/// * `topic_name` must be a valid null-terminated string
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_subscription_init_default(
    subscription: *mut nros_subscription_t,
    node: *const nros_node_t,
    type_info: *const nros_message_type_t,
    topic_name: *const c_char,
) -> nros_ret_t {
    nros_subscription_init_with_qos(
        subscription,
        node,
        type_info,
        topic_name,
        None,
        ptr::null_mut(),
        ptr::null(),
    )
}

/// Initialize a subscription with custom QoS.
///
/// # Safety
/// See `nros_subscription_init` for safety requirements.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_subscription_init_with_qos(
    subscription: *mut nros_subscription_t,
    node: *const nros_node_t,
    type_info: *const nros_message_type_t,
    topic_name: *const c_char,
    callback: nros_subscription_callback_t,
    context: *mut c_void,
    qos: *const nros_qos_t,
) -> nros_ret_t {
    validate_not_null!(subscription, node, type_info, topic_name);

    // phase-417 W5.a — a NULL callback is LEGAL, and it is the typed path's
    // shape. `nros_executor_add_subscription_typed` reads neither
    // `subscription->callback` nor `->context`: it carries its own deserialiser,
    // the caller's message storage and the typed callback. Rejecting NULL forced
    // a typed-only user to pass a raw callback that can never fire, which is a
    // required argument with no meaning -- and it is also why
    // `rclc_subscription_init_default` (4 args, no callback) could not be
    // aliased onto ours (6 args) in `<nros/rcl_compat.h>`.
    //
    // The raw path is unchanged: it is `nros_executor_add_subscription` that
    // dispatches through this field, and that entry point already treats an
    // absent callback as nothing to dispatch.
    let _ = &callback;

    let subscription = &mut *subscription;
    let node_ref = &*node;
    let type_info = &*type_info;

    validate_state!(
        subscription,
        nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_UNINITIALIZED,
        NROS_RET_BAD_SEQUENCE
    );

    validate_state!(node_ref, nros_node_state_t::NROS_NODE_STATE_INITIALIZED);

    // Copy topic name (required — empty rejected)
    subscription.topic_name_len =
        crate::util::copy_cstr_into(topic_name, &mut subscription.topic_name);
    if subscription.topic_name_len == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }

    // Copy type name + hash (both optional — null sources leave dst untouched)
    subscription.type_name_len =
        crate::util::copy_cstr_into(type_info.type_name, &mut subscription.type_name);
    subscription.type_hash_len =
        crate::util::copy_cstr_into(type_info.type_hash, &mut subscription.type_hash);

    // Store callback and context
    subscription.callback = callback;
    subscription.context = context;
    subscription.node = unsafe { crate::node::node_ref_of(node) };

    // Store QoS settings for later use by executor registration
    subscription.qos = if qos.is_null() {
        crate::qos::NROS_QOS_DEFAULT
    } else {
        *qos
    };

    // Subscriber creation is deferred to nros_executor_add_subscription(),
    // which calls nros_node::Executor::add_arena_subscription_c_callback().
    subscription.handle_id = usize::MAX;
    subscription.state = nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_INITIALIZED;

    NROS_RET_OK
}

/// Phase 189.M3 — initialize a subscription with custom QoS + named options.
///
/// rclc-style entry point: QoS is passed separately (`qos`, NULL =
/// default) and the non-QoS axes ride in `options` (NULL = defaults).
/// Behaves exactly like [`nros_subscription_init_with_qos`] except that
/// a non-zero `options->sched_context` is stashed on the subscription so
/// that [`nros_executor_add_subscription`] binds the resulting
/// executor handle to that scheduling context once the handle is known
/// (entity creation is deferred to registration, so the handle does not
/// exist at init time). `options->message_info` is RESERVED and ignored
/// (Phase 189.M3.4).
///
/// # Parameters
/// * `subscription` - Pointer to a zero-initialized subscription
/// * `node` - Pointer to an initialized node
/// * `type_info` - Pointer to message type information
/// * `topic_name` - Topic name (null-terminated string)
/// * `callback` - Callback function to invoke when messages arrive
/// * `context` - User context pointer passed to callback (can be NULL)
/// * `qos` - Pointer to QoS settings (NULL for default)
/// * `options` - Pointer to subscription options (NULL for defaults)
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if any required pointer is NULL
/// * `NROS_RET_NOT_INIT` if node is not initialized
/// * `NROS_RET_ERROR` on initialization failure
///
/// # Safety
/// * All required pointers must be valid
/// * `topic_name` must be a valid null-terminated string
/// * `qos` / `options` may be NULL or point to valid structs
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_subscription_init_with_options(
    subscription: *mut nros_subscription_t,
    node: *const nros_node_t,
    type_info: *const nros_message_type_t,
    topic_name: *const c_char,
    callback: nros_subscription_callback_t,
    context: *mut c_void,
    qos: *const nros_qos_t,
    options: *const nros_subscription_options_t,
) -> nros_ret_t {
    let ret = nros_subscription_init_with_qos(
        subscription,
        node,
        type_info,
        topic_name,
        callback,
        context,
        qos,
    );
    if ret != NROS_RET_OK {
        return ret;
    }

    // Stash the requested scheduling-context slot. The handle does not
    // exist yet (entity creation + handle assignment happen in
    // `nros_executor_add_subscription`), so the actual
    // `bind_handle_to_sched_context` call is deferred to there. `0`
    // means "inherit the default" and is a no-op. `message_info` is
    // reserved (Phase 189.M3.4) and intentionally ignored.
    if !options.is_null() {
        let opts = &*options;
        // SAFETY: `subscription` was validated non-NULL by the
        // `_with_qos` call above (it returned OK).
        (*subscription).sched_context_id = opts.sched_context;
    }

    NROS_RET_OK
}

// ============================================================================
// Phase 122.3.b — Layer-1 primitive entry points (caller polls)
// ============================================================================
//
// L1 is for callers that own their own scheduler (RTIC, embassy,
// FreeRTOS-native task-per-entity). The subscriber entity is created
// at init time and stored inline in `_opaque`; the caller polls it
// directly via `nros_subscription_take_serialized`, never registering
// with an `nros_executor_t`.
//
// L1 ops are mutually exclusive with L2 (callback / executor). A
// subscription in `POLLING` state cannot be registered with an
// executor; one in `INITIALIZED` (L2) state cannot be polled
// directly. State transitions:
//
//   UNINITIALIZED → POLLING (via nros_subscription_init_polling)
//                 → INITIALIZED (via nros_subscription_init,
//                                deferred subscriber creation)
//   POLLING / INITIALIZED → SHUTDOWN (via nros_subscription_fini)

/// Phase 122.3.b — initialize an L1 polling-mode subscription.
///
/// Creates the underlying RMW subscriber immediately and stores it
/// inline in the subscription's `_opaque` field. The caller drains
/// received messages via `nros_subscription_take_serialized`.
///
/// Uses default QoS (RELIABLE, KEEP_LAST(10)). For custom QoS, use
/// `nros_subscription_init_polling_with_qos`.
///
/// # Parameters
/// * `subscription` - Pointer to a zero-initialized subscription
/// * `node` - Pointer to an initialized node
/// * `type_info` - Pointer to message type information
/// * `topic_name` - Topic name (null-terminated)
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if any pointer is NULL or topic empty
/// * `NROS_RET_NOT_INIT` if node / support not initialized
/// * `NROS_RET_ERROR` if subscriber creation failed
///
/// # Safety
/// * All pointers must be valid
/// * `topic_name` must be a valid null-terminated string
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_subscription_init_polling(
    subscription: *mut nros_subscription_t,
    node: *const nros_node_t,
    type_info: *const nros_message_type_t,
    topic_name: *const c_char,
) -> nros_ret_t {
    nros_subscription_init_polling_with_qos(subscription, node, type_info, topic_name, ptr::null())
}

/// Phase 122.3.b — initialize an L1 polling-mode subscription with custom QoS.
///
/// See `nros_subscription_init_polling` for the threading + lifecycle
/// contract.
///
/// # Safety
/// See `nros_subscription_init_polling`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_subscription_init_polling_with_qos(
    subscription: *mut nros_subscription_t,
    node: *const nros_node_t,
    type_info: *const nros_message_type_t,
    topic_name: *const c_char,
    qos: *const nros_qos_t,
) -> nros_ret_t {
    validate_not_null!(subscription, node, type_info, topic_name);

    let subscription_mut = &mut *subscription;
    let node_ref = &*node;
    let type_info_ref = &*type_info;

    validate_state!(
        subscription_mut,
        nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_UNINITIALIZED,
        NROS_RET_BAD_SEQUENCE
    );
    validate_state!(node_ref, nros_node_state_t::NROS_NODE_STATE_INITIALIZED);

    // Copy topic + type metadata into the subscription struct so
    // getters keep working post-init.
    subscription_mut.topic_name_len =
        crate::util::copy_cstr_into(topic_name, &mut subscription_mut.topic_name);
    if subscription_mut.topic_name_len == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }
    subscription_mut.type_name_len =
        crate::util::copy_cstr_into(type_info_ref.type_name, &mut subscription_mut.type_name);
    subscription_mut.type_hash_len =
        crate::util::copy_cstr_into(type_info_ref.type_hash, &mut subscription_mut.type_hash);

    subscription_mut.node = unsafe { crate::node::node_ref_of(node) };
    subscription_mut.qos = if qos.is_null() {
        crate::qos::NROS_QOS_DEFAULT
    } else {
        *qos
    };
    subscription_mut.callback = None;
    subscription_mut.context = ptr::null_mut();
    subscription_mut.handle_id = usize::MAX;

    // Create the subscriber NOW (vs deferred for L2). The L1 path
    // owns the entity inline; no executor arena involved.
    #[cfg(feature = "rmw-cffi")]
    {
        use nros_node::{Session, TopicInfo};

        // Phase 156 Sub-bug D — multi-Session dispatch (see
        // `rclc_publisher_init_default` for the long form).
        let (session, domain_id) = match crate::node::resolve_session_and_domain(node_ref) {
            Some(t) => t,
            None => return NROS_RET_NOT_INIT,
        };

        let topic_str = core::str::from_utf8_unchecked(
            &subscription_mut.topic_name[..subscription_mut.topic_name_len],
        );
        let type_str = core::str::from_utf8_unchecked(
            &subscription_mut.type_name[..subscription_mut.type_name_len],
        );
        let type_hash_str = core::str::from_utf8_unchecked(
            &subscription_mut.type_hash[..subscription_mut.type_hash_len],
        );
        let node_name_str = core::str::from_utf8_unchecked(&node_ref.name[..node_ref.name_len]);
        let namespace_str =
            core::str::from_utf8_unchecked(&node_ref.namespace[..node_ref.namespace_len]);

        let topic_info = TopicInfo::new(topic_str, type_str, type_hash_str)
            .with_domain(domain_id)
            .with_node_name(node_name_str)
            .with_namespace(namespace_str);

        // Phase 211.H (issue #52) — fold any plan qos_overrides for this
        // topic + subscription role into the profile before create, mirroring
        // Rust's `NodeHandle::create_subscription_with_qos`.
        let qos_settings = crate::qos::apply_qos_overrides(
            subscription_mut.qos.to_qos_settings(),
            node_ref.qos_overrides,
            node_ref.qos_overrides_len,
            topic_str,
            crate::qos::QOS_OVERRIDE_ROLE_SUBSCRIPTION,
        );
        match session.create_subscription(&topic_info, qos_settings) {
            Ok(handle) => {
                let raw = nros_node::RawSubscription::<{ crate::config::MESSAGE_BUFFER_SIZE }>::new(
                    handle,
                );
                core::ptr::write(
                    subscription_mut._opaque.as_mut_ptr()
                        as *mut nros_node::RawSubscription<{ crate::config::MESSAGE_BUFFER_SIZE }>,
                    raw,
                );
            }
            Err(_) => return NROS_RET_ERROR,
        }
    }

    subscription_mut.state = nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_POLLING;
    NROS_RET_OK
}

/// Phase 122.3.c.6.e — register a C wake callback on an L1
/// polling-mode subscription. `state` is a caller-owned
/// `nros_wake_state_t` (declared next to the subscription) that
/// must outlive the subscription and not move. Pass `cb = NULL`
/// to disable. The backend wakes the callback when a new message
/// arrives.
///
/// # Safety
/// All pointers valid; `state` storage stable for the
/// subscription's lifetime.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_subscription_set_wake_callback(
    subscription: *mut nros_subscription_t,
    state: *mut crate::service::nros_wake_state_t,
    cb: Option<unsafe extern "C" fn(*mut c_void)>,
    ctx: *mut c_void,
) -> nros_ret_t {
    if subscription.is_null() || state.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let subscription_mut = &mut *subscription;
    if subscription_mut.state != nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_POLLING {
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
        let raw = &*(subscription_mut._opaque.as_ptr()
            as *const nros_node::RawSubscription<{ crate::config::MESSAGE_BUFFER_SIZE }>);
        raw.register_waker(&waker);
        NROS_RET_OK
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (state, cb, ctx);
        NROS_RET_NOT_INIT
    }
}

/// Phase 122.3.b — non-blocking poll on an L1 polling-mode
/// subscription. Returns the number of bytes received on success
/// (may be 0 if no data available), or a negative `nros_ret_t` on
/// error.
///
/// # Parameters
/// * `subscription` - Pointer to a POLLING-state subscription
/// * `buf` - Caller-supplied buffer to receive the message bytes
/// * `buf_len` - Capacity of `buf`
///
/// # Returns
/// * `>= 0` — number of bytes copied into `buf`
/// * `NROS_RET_INVALID_ARGUMENT` if any pointer is NULL or state is wrong
/// * `NROS_RET_ERROR` on transport failure
///
/// # Safety
/// * `subscription` must be in `POLLING` state
/// * `buf` must point to writable memory of at least `buf_len` bytes
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_subscription_take_serialized(
    subscription: *mut nros_subscription_t,
    buf: *mut u8,
    buf_len: usize,
) -> i32 {
    if subscription.is_null() || (buf.is_null() && buf_len != 0) {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let subscription_mut = &mut *subscription;
    if subscription_mut.state != nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_POLLING {
        return NROS_RET_BAD_SEQUENCE;
    }

    #[cfg(feature = "rmw-cffi")]
    {
        let raw = &mut *(subscription_mut._opaque.as_mut_ptr()
            as *mut nros_node::RawSubscription<{ crate::config::MESSAGE_BUFFER_SIZE }>);
        match raw.take_serialized() {
            Ok(Some(len)) => {
                let to_copy = len.min(buf_len);
                core::ptr::copy_nonoverlapping(raw.buffer().as_ptr(), buf, to_copy);
                to_copy as i32
            }
            Ok(None) => 0,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (subscription_mut, buf, buf_len);
        NROS_RET_ERROR
    }
}

/// Phase 252 / issue 0073 — E2E message-integrity status surfaced to the C/C++
/// receive path ([`nros_subscription_take_validated`]). The C analog of the
/// Rust `IntegrityStatus` / `CallbackCtx::integrity()`.
#[cfg(feature = "safety-e2e")]
#[repr(C)]
pub struct nros_integrity_status_t {
    /// Sequence-number gap since the previous in-order message (0 = none).
    pub gap: i64,
    /// `true` if this sample's sequence number was already seen (a duplicate).
    pub duplicate: bool,
    /// CRC verdict: `1` = valid, `0` = mismatch (corruption), `-1` = no CRC on the
    /// wire (the publisher was built without `safety-e2e`).
    pub crc_valid: i8,
}

/// Phase 252 / issue 0073 — non-blocking poll that ALSO returns the E2E
/// integrity status (CRC + sequence gap/dup) of the received sample. The
/// safety-e2e analog of [`nros_subscription_take_serialized`]: it requests
/// validation on the backend (the zenoh shim recomputes + compares the CRC
/// attachment and tracks the sequence) and writes the verdict to `*out_status`.
///
/// Requires the build to enable `safety-e2e` on both ends (the zenoh backend's
/// own feature, lowered from a declared `[safety]` axis); otherwise `crc_valid`
/// reports `-1` (no CRC on the wire).
///
/// # Returns
/// * `>= 0` — number of bytes copied into `buf` (0 = no data available)
/// * `NROS_RET_INVALID_ARGUMENT` if `subscription`/`buf` is NULL
/// * `NROS_RET_BAD_SEQUENCE` if the subscription is not in POLLING state
/// * `NROS_RET_ERROR` on transport failure
///
/// # Safety
/// * `subscription` must be a valid POLLING subscription
/// * `buf` must point to writable memory of at least `buf_len` bytes
/// * `out_status`, if non-NULL, must point to a writable `nros_integrity_status_t`
#[cfg(feature = "safety-e2e")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_subscription_take_validated(
    subscription: *mut nros_subscription_t,
    buf: *mut u8,
    buf_len: usize,
    out_status: *mut nros_integrity_status_t,
) -> i32 {
    if subscription.is_null() || (buf.is_null() && buf_len != 0) {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let subscription_mut = &mut *subscription;
    if subscription_mut.state != nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_POLLING {
        return NROS_RET_BAD_SEQUENCE;
    }

    #[cfg(feature = "rmw-cffi")]
    {
        let raw = &mut *(subscription_mut._opaque.as_mut_ptr()
            as *mut nros_node::RawSubscription<{ crate::config::MESSAGE_BUFFER_SIZE }>);
        match raw.take_validated() {
            Ok(Some((len, status))) => {
                let to_copy = len.min(buf_len);
                core::ptr::copy_nonoverlapping(raw.buffer().as_ptr(), buf, to_copy);
                if !out_status.is_null() {
                    *out_status = nros_integrity_status_t {
                        gap: status.gap,
                        duplicate: status.duplicate,
                        crc_valid: match status.crc_valid {
                            Some(true) => 1,
                            Some(false) => 0,
                            None => -1,
                        },
                    };
                }
                to_copy as i32
            }
            Ok(None) => 0,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (subscription_mut, buf, buf_len, out_status);
        NROS_RET_ERROR
    }
}

// ============================================================================
// Phase 124.A.6 — zero-copy subscription borrow / release
// ============================================================================

/// Phase 124.A.6 — borrow a read-only view of the next available message
/// in place (zero-copy receive path).
///
/// **Availability:** requires the `lending` AND `alloc` cargo features,
/// and no shipped build enables `lending` — see `nros_publisher_loan`
/// for what that means for this header. Issue 0814.
///
/// Bypasses the subscription's staging buffer. On success, `*out_buf`
/// points at `*out_len` bytes the caller can read directly. Caller MUST
/// pass `*out_token` back to [`nros_subscription_release`] before
/// requesting another borrow on the same subscription — only one
/// outstanding view per subscription at a time.
///
/// Falls back to a `take_serialized` copy into the staging buffer when the
/// active backend's vtable doesn't expose a native borrow slot.
///
/// **Requires `alloc` as well as `lending`.** Unlike the publish half —
/// which issue 0812 made allocation-free by passing the BACKEND's own
/// token through — the receive half still boxes a `RecvView` per borrow
/// to manufacture a stable FFI token, so it cannot exist in a heap-free
/// image. That was 0812's own "what this does NOT yet buy" item, and
/// leaving the `cfg` at `lending` alone did not make it work: it made
/// `nros-c` FAIL TO COMPILE under `--features lending` without `alloc`
/// (four `alloc::boxed::Box` uses with no `extern crate alloc` in scope).
/// A capability that cannot build is worse than one that is absent, so
/// the `cfg` now states the real requirement (issue 0814).
///
/// # Returns
/// * `> 0` — message length written into `*out_len`; view is ready.
/// * `0` — no message ready right now.
/// * negative — error (see `nros_ret_t`).
///
/// # Safety
/// * `subscription` must be a valid polling subscription.
/// * `out_buf` / `out_len` / `out_token` must be valid pointers.
#[cfg(all(feature = "lending", feature = "alloc"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_subscription_borrow(
    subscription: *mut nros_subscription_t,
    out_buf: *mut *const u8,
    out_len: *mut usize,
    out_token: *mut *mut core::ffi::c_void,
) -> i32 {
    if subscription.is_null() || out_buf.is_null() || out_len.is_null() || out_token.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let subscription_mut = &mut *subscription;
    if subscription_mut.state != nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_POLLING {
        return NROS_RET_BAD_SEQUENCE;
    }

    let raw = &mut *(subscription_mut._opaque.as_mut_ptr()
        as *mut nros_node::RawSubscription<{ crate::config::MESSAGE_BUFFER_SIZE }>);
    // RawSubscription::try_borrow returns a RecvView bound to &mut raw.
    // For the C-side token plumbing, we erase the lifetime and Box it
    // — the caller's release/destroy contract restores correctness.
    match raw.try_borrow() {
        Ok(Some(view)) => {
            // RecvView impls Deref<Target=[u8]> on both paths, so the
            // slice methods are reached by auto-deref (issue 0814: the
            // explicit `(&*view)` reborrow this replaces tripped clippy's
            // `needless_borrow` / `explicit_auto_deref` under `-D warnings`.
            // It had never been LINTED, because until now this file did not
            // compile under `lending` at all).
            let buf_ptr = view.as_ptr();
            let len = view.len();
            // SAFETY: erase the lifetime — caller must release before
            // dropping the subscription or requesting another borrow.
            let view_static: nros_node::RecvView<'static> = core::mem::transmute(view);
            let boxed = alloc::boxed::Box::new(view_static);
            *out_buf = buf_ptr;
            *out_len = len;
            *out_token = alloc::boxed::Box::into_raw(boxed) as *mut core::ffi::c_void;
            len as i32
        }
        Ok(None) => 0,
        Err(_) => NROS_RET_ERROR,
    }
}

/// Phase 124.A.6 — release a previously borrowed view.
///
/// **Availability:** requires `lending` AND `alloc`, and no shipped build
/// enables `lending` — see `nros_publisher_loan`. Issue 0814.
///
/// `token` MUST come from a prior `nros_subscription_borrow` on the
/// SAME subscription; consuming it is mandatory before the
/// subscription's next borrow / destroy.
///
/// **Requires `alloc` as well as `lending`** — it unboxes the `RecvView`
/// its paired `nros_subscription_borrow` boxed. See that function.
///
/// # Safety
/// * `subscription` must be the subscription the token was borrowed from.
/// * `token` must not be NULL and must not be reused after this call.
#[cfg(all(feature = "lending", feature = "alloc"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_subscription_release(
    subscription: *mut nros_subscription_t,
    token: *mut core::ffi::c_void,
) -> nros_ret_t {
    if subscription.is_null() || token.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let subscription_mut = &mut *subscription;
    if subscription_mut.state != nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_POLLING {
        return NROS_RET_BAD_SEQUENCE;
    }
    // SAFETY: token came from `Box::into_raw(Box<RecvView<'static>>)`
    // inside `nros_subscription_borrow`. Reconstitute and drop —
    // RecvView::drop fires the backend's sub_release (or no-op for
    // the staging-buffer fallback path).
    let _view: alloc::boxed::Box<nros_node::RecvView<'static>> =
        alloc::boxed::Box::from_raw(token as *mut nros_node::RecvView<'static>);
    NROS_RET_OK
}

/// Phase 124.D.1 — burst-take. Drain up to `max_msgs` queued samples
/// into the contiguous `buf` block in a single call, with the i-th
/// sample at `buf + i * per_msg_cap` and length `out_lens[i]`.
///
/// `buf` is a caller-owned contiguous region of at least
/// `max_msgs * per_msg_cap` bytes. `out_lens` is a writable array
/// of at least `max_msgs` `size_t` slots.
///
/// Returns the number of messages delivered (`>= 0`) on success, or
/// a negative `nros_ret_t` error code. Partial drains are reported
/// as the count, never as an error.
///
/// # Safety
/// * `subscription` must be in `POLLING` state.
/// * `buf` must point to writable memory of at least
///   `max_msgs * per_msg_cap` bytes.
/// * `out_lens` must point to a writable array of at least
///   `max_msgs` `size_t` slots.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_subscription_take_sequence(
    subscription: *mut nros_subscription_t,
    buf: *mut u8,
    per_msg_cap: usize,
    max_msgs: usize,
    out_lens: *mut usize,
) -> i32 {
    if subscription.is_null() || buf.is_null() || out_lens.is_null() || per_msg_cap == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let subscription_mut = &mut *subscription;
    if subscription_mut.state != nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_POLLING {
        return NROS_RET_BAD_SEQUENCE;
    }

    #[cfg(feature = "rmw-cffi")]
    {
        let raw = &mut *(subscription_mut._opaque.as_mut_ptr()
            as *mut nros_node::RawSubscription<{ crate::config::MESSAGE_BUFFER_SIZE }>);
        // `RawSubscription`'s `handle` is the cffi subscriber that
        // drives the vtable slot (or its loop fallback). Borrow the
        // caller buffer as a flat slice and dispatch.
        let buf_slice = core::slice::from_raw_parts_mut(buf, max_msgs.saturating_mul(per_msg_cap));
        let lens_slice = core::slice::from_raw_parts_mut(out_lens, max_msgs);
        match raw.take_sequence(buf_slice, per_msg_cap, max_msgs, lens_slice) {
            Ok(count) => count as i32,
            Err(_) => NROS_RET_ERROR,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = (subscription_mut, buf, per_msg_cap, max_msgs, out_lens);
        NROS_RET_ERROR
    }
}

/// # Safety
/// * `subscription` must be a valid pointer
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_subscription_fini(
    subscription: *mut nros_subscription_t,
) -> nros_ret_t {
    validate_not_null!(subscription);

    let subscription = &mut *subscription;

    match subscription.state {
        nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_INITIALIZED => {
            // L2: subscriber lives in the executor arena (if registered) —
            // just reset metadata.
        }
        nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_POLLING => {
            // L1: drop the inline RawSubscription so its Drop runs
            // (closes the underlying RMW subscriber).
            // phase-379 W4 — the node this entity was created on is gone, so its
            // teardown ORDER was wrong. Reported rather than performed: before W4 the
            // entity held a pointer nothing dereferenced and this case succeeded
            // silently, which is what made the ordering obligation unenforceable.
            if subscription.node.is_bound() && !crate::node::node_ref_is_live(subscription.node) {
                return crate::error::NROS_RET_STALE_NODE;
            }

            #[cfg(feature = "rmw-cffi")]
            {
                core::ptr::drop_in_place(subscription._opaque.as_mut_ptr()
                    as *mut nros_node::RawSubscription<{ crate::config::MESSAGE_BUFFER_SIZE }>);
                subscription._opaque = [0u64; SUBSCRIPTION_OPAQUE_U64S];
            }
        }
        _ => return NROS_RET_NOT_INIT,
    }

    subscription.handle_id = usize::MAX;
    subscription.callback = None;
    subscription.context = ptr::null_mut();
    subscription.node = crate::node::nros_node_ref_t::none();
    subscription.state = nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_SHUTDOWN;

    NROS_RET_OK
}

/// Get the topic name of a subscription.
///
/// # Parameters
/// * `subscription` - Pointer to a subscription
///
/// # Returns
/// * Pointer to topic name (null-terminated), or NULL if invalid
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_subscription_get_topic_name(
    subscription: *const nros_subscription_t,
) -> *const c_char {
    if subscription.is_null() {
        return ptr::null();
    }

    let subscription = &*subscription;
    if subscription.state != nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_INITIALIZED {
        return ptr::null();
    }

    subscription.topic_name.as_ptr() as *const c_char
}

/// Check if subscription is valid (initialized).
///
/// # Parameters
/// * `subscription` - Pointer to a subscription
///
/// # Returns
/// * `true` if valid, `false` if invalid or NULL
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_subscription_is_valid(
    subscription: *const nros_subscription_t,
) -> bool {
    if subscription.is_null() {
        return false;
    }

    let subscription = &*subscription;
    subscription.state == nros_subscription_state_t::NROS_SUBSCRIPTION_STATE_INITIALIZED
}

// Internal helper methods for executor
impl nros_subscription_t {
    /// Get the callback function
    pub(crate) fn get_callback(&self) -> nros_subscription_callback_t {
        self.callback
    }

    /// Get the user context
    pub(crate) fn get_context(&self) -> *mut c_void {
        self.context
    }

    /// Get the stored QoS as `nros_rmw::QoSProfile`
    pub(crate) fn get_qos_settings(&self) -> nros_rmw::QoSProfile {
        self.qos.to_qos_settings()
    }

    /// Set the handle ID from executor registration
    pub(crate) fn set_handle_id(&mut self, id: nros_node::HandleId) {
        self.handle_id = id.0;
    }
}
