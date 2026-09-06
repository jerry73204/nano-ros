//! Lifecycle node API for the C FFI.
//!
//! Thin wrapper around [`nros_node::lifecycle::LifecyclePollingNodeCtx`]. The
//! `#[repr(C)]` struct holds an opaque u64 array sized to fit the real Rust
//! state machine; all transition / callback logic lives in `nros-node` and is
//! tested there.

use core::{ffi::c_void, mem::MaybeUninit};

use nros_node::{
    LifecycleState, LifecycleTransition, TransitionResult,
    lifecycle::{LifecycleCallbackSlot, LifecycleError, LifecyclePollingNodeCtx},
};

use crate::{constants::NROS_LIFECYCLE_CTX_OPAQUE_U64S, error::*};

// ============================================================================
// Constants — exported to C via cbindgen
// ============================================================================
//
// Issue 0792: these are spelled as LITERALS, not as `LifecycleState::X as u8`.
// cbindgen runs with `parse_deps = false`, so it cannot see the `nros_core`
// enums and cannot evaluate a cast through one — it silently DROPS such a
// constant rather than failing. All fifteen were missing from
// `nros_generated.h`, so a C caller had no name for a transition id or for a
// callback's return value: the same cross-crate blind spot that made
// `Option<LifecycleCallbackFnCtx>` an opaque struct, one item over. The
// `const _` block below is what keeps the literals honest — a discriminant
// change in `nros_core::lifecycle` is a compile error here, not a wrong number
// in a header.

/// Lifecycle state: Unconfigured
pub const NROS_LIFECYCLE_STATE_UNCONFIGURED: u8 = 1;
/// Lifecycle state: Inactive
pub const NROS_LIFECYCLE_STATE_INACTIVE: u8 = 2;
/// Lifecycle state: Active
pub const NROS_LIFECYCLE_STATE_ACTIVE: u8 = 3;
/// Lifecycle state: Finalized
pub const NROS_LIFECYCLE_STATE_FINALIZED: u8 = 4;
/// Lifecycle state: ErrorProcessing
pub const NROS_LIFECYCLE_STATE_ERROR_PROCESSING: u8 = 5;

// Issue 1099 — the transition ids below ARE `lifecycle_msgs/msg/Transition`.
// They are the ids `nros_lifecycle_change_state` and
// `nros::LifecycleNode::trigger_transition` accept and the ids a `ChangeState`
// request carries, so a caller who writes the number instead of the name must
// still get the transition ROS 2 would give them. Four of these used to
// disagree with upstream (2/3/4 were Activate/Deactivate/Cleanup, 8 was Error
// Recovery), which made `trigger_transition(2)` ACTIVATE an Inactive node
// where ROS 2 cleans it up.

/// Lifecycle transition: Configure (`lifecycle_msgs` `TRANSITION_CONFIGURE`)
pub const NROS_LIFECYCLE_TRANSITION_CONFIGURE: u8 = 1;
/// Lifecycle transition: Cleanup (`lifecycle_msgs` `TRANSITION_CLEANUP`)
pub const NROS_LIFECYCLE_TRANSITION_CLEANUP: u8 = 2;
/// Lifecycle transition: Activate (`lifecycle_msgs` `TRANSITION_ACTIVATE`)
pub const NROS_LIFECYCLE_TRANSITION_ACTIVATE: u8 = 3;
/// Lifecycle transition: Deactivate (`lifecycle_msgs` `TRANSITION_DEACTIVATE`)
pub const NROS_LIFECYCLE_TRANSITION_DEACTIVATE: u8 = 4;
/// Lifecycle transition: Shutdown (from Unconfigured)
/// (`lifecycle_msgs` `TRANSITION_UNCONFIGURED_SHUTDOWN`)
pub const NROS_LIFECYCLE_TRANSITION_SHUTDOWN_UNCONFIGURED: u8 = 5;
/// Lifecycle transition: Shutdown (from Inactive)
/// (`lifecycle_msgs` `TRANSITION_INACTIVE_SHUTDOWN`)
pub const NROS_LIFECYCLE_TRANSITION_SHUTDOWN_INACTIVE: u8 = 6;
/// Lifecycle transition: Shutdown (from Active)
/// (`lifecycle_msgs` `TRANSITION_ACTIVE_SHUTDOWN`)
pub const NROS_LIFECYCLE_TRANSITION_SHUTDOWN_ACTIVE: u8 = 7;
/// Lifecycle transition: Error Recovery
/// (`lifecycle_msgs` `TRANSITION_ON_ERROR_SUCCESS`). Upstream `8` is
/// `TRANSITION_DESTROY`, which nano-ros does not implement, and upstream models
/// error recovery as an implicit transition — `60` is upstream's id for its
/// success edge and is what nano-ros already put on the wire for it.
pub const NROS_LIFECYCLE_TRANSITION_ERROR_RECOVERY: u8 = 60;

/// Transition result: Success
pub const NROS_LIFECYCLE_RET_OK: u8 = 0;
/// Transition result: Failure
pub const NROS_LIFECYCLE_RET_FAILURE: u8 = 1;
/// Transition result: Error
pub const NROS_LIFECYCLE_RET_ERROR: u8 = 2;

/// Issue 0792 — the literals above must equal the `nros_core::lifecycle`
/// discriminants they mirror. This is the check cbindgen cannot do.
const _: () = {
    assert!(NROS_LIFECYCLE_STATE_UNCONFIGURED == LifecycleState::Unconfigured as u8);
    assert!(NROS_LIFECYCLE_STATE_INACTIVE == LifecycleState::Inactive as u8);
    assert!(NROS_LIFECYCLE_STATE_ACTIVE == LifecycleState::Active as u8);
    assert!(NROS_LIFECYCLE_STATE_FINALIZED == LifecycleState::Finalized as u8);
    assert!(NROS_LIFECYCLE_STATE_ERROR_PROCESSING == LifecycleState::ErrorProcessing as u8);

    assert!(NROS_LIFECYCLE_TRANSITION_CONFIGURE == LifecycleTransition::Configure as u8);
    assert!(NROS_LIFECYCLE_TRANSITION_CLEANUP == LifecycleTransition::Cleanup as u8);
    assert!(NROS_LIFECYCLE_TRANSITION_ACTIVATE == LifecycleTransition::Activate as u8);
    assert!(NROS_LIFECYCLE_TRANSITION_DEACTIVATE == LifecycleTransition::Deactivate as u8);
    assert!(
        NROS_LIFECYCLE_TRANSITION_SHUTDOWN_UNCONFIGURED
            == LifecycleTransition::ShutdownUnconfigured as u8
    );
    assert!(
        NROS_LIFECYCLE_TRANSITION_SHUTDOWN_INACTIVE == LifecycleTransition::ShutdownInactive as u8
    );
    assert!(NROS_LIFECYCLE_TRANSITION_SHUTDOWN_ACTIVE == LifecycleTransition::ShutdownActive as u8);
    assert!(NROS_LIFECYCLE_TRANSITION_ERROR_RECOVERY == LifecycleTransition::ErrorRecovery as u8);

    assert!(NROS_LIFECYCLE_RET_OK == TransitionResult::Success as u8);
    assert!(NROS_LIFECYCLE_RET_FAILURE == TransitionResult::Failure as u8);
    assert!(NROS_LIFECYCLE_RET_ERROR == TransitionResult::Error as u8);
};

/// Issue 1099 — the 0792 block above proves the header agrees with our enum;
/// it cannot catch the two being wrong TOGETHER, which is exactly what
/// happened. These literals are transcribed from
/// `lifecycle_msgs/msg/Transition.msg` and are what a ported ROS 2 caller
/// means by the number.
const _: () = {
    assert!(NROS_LIFECYCLE_TRANSITION_CONFIGURE == 1); // TRANSITION_CONFIGURE
    assert!(NROS_LIFECYCLE_TRANSITION_CLEANUP == 2); // TRANSITION_CLEANUP
    assert!(NROS_LIFECYCLE_TRANSITION_ACTIVATE == 3); // TRANSITION_ACTIVATE
    assert!(NROS_LIFECYCLE_TRANSITION_DEACTIVATE == 4); // TRANSITION_DEACTIVATE
    assert!(NROS_LIFECYCLE_TRANSITION_SHUTDOWN_UNCONFIGURED == 5); // ..._UNCONFIGURED_SHUTDOWN
    assert!(NROS_LIFECYCLE_TRANSITION_SHUTDOWN_INACTIVE == 6); // ..._INACTIVE_SHUTDOWN
    assert!(NROS_LIFECYCLE_TRANSITION_SHUTDOWN_ACTIVE == 7); // ..._ACTIVE_SHUTDOWN
    assert!(NROS_LIFECYCLE_TRANSITION_ERROR_RECOVERY == 60); // TRANSITION_ON_ERROR_SUCCESS

    // The primary states already match `lifecycle_msgs/msg/State.msg`.
    // `NROS_LIFECYCLE_STATE_ERROR_PROCESSING` deliberately does NOT: upstream
    // spells it `TRANSITION_STATE_ERRORPROCESSING = 15`. `5` is UNASSIGNED
    // upstream, so unlike the transition ids it can never silently mean a
    // different state; `state_wire()` in `nros-node` maps it to 15 on the wire.
    assert!(NROS_LIFECYCLE_STATE_UNCONFIGURED == 1);
    assert!(NROS_LIFECYCLE_STATE_INACTIVE == 2);
    assert!(NROS_LIFECYCLE_STATE_ACTIVE == 3);
    assert!(NROS_LIFECYCLE_STATE_FINALIZED == 4);
};

// ============================================================================
// Types
// ============================================================================

/// C callback type for a lifecycle transition: `uint8_t callback(void *context)`.
///
/// The return value is a REP-2002 `TransitionResult`
/// ([`NROS_LIFECYCLE_RET_OK`] = 0, [`NROS_LIFECYCLE_RET_FAILURE`] = 1,
/// [`NROS_LIFECYCLE_RET_ERROR`] = 2). `context` is the pointer most recently
/// handed to a `nros_lifecycle_register_on_*` /
/// `nros_executor_lifecycle_register_on_*` call — all six slots on one state
/// machine share a single context.
///
/// `NULL` clears the slot.
///
/// Issue 0792: this alias is declared HERE rather than reusing
/// `nros_node::lifecycle::LifecycleCallbackFnCtx` because cbindgen runs with
/// `parse_deps = false`. A cross-crate `Option<LifecycleCallbackFnCtx>` in an
/// `extern "C"` signature is unresolvable to it, so it emitted an OPAQUE
/// `struct Option_LifecycleCallbackFnCtx` and passed it BY VALUE — a C
/// translation unit could not form the argument, and all twelve
/// `*_register_on_*` entry points were uncallable from C. A local
/// `Option<fn>` alias renders as a plain nullable function pointer (the same
/// shape `nros_guard_condition_callback_t` and `nros_timer_callback_t`
/// already use, and the same one the C++ shim's
/// `nros_cpp_lifecycle_callback_t` uses). The ABI is unchanged: a nullable
/// `extern "C" fn` is one pointer either way.
pub type nros_lifecycle_callback_t = Option<unsafe extern "C" fn(context: *mut c_void) -> u8>;

/// Opaque lifecycle state machine storage.
///
/// The `storage` field holds a [`LifecyclePollingNodeCtx`] placed into a
/// `u64` array to keep C-ABI struct layout predictable for C callers.
/// Treat the struct as opaque — use [`nros_lifecycle_get_current_state`] and the
/// `nros_lifecycle_register_on_*` functions to interact with it.
#[repr(C)]
pub struct nros_lifecycle_state_machine_t {
    /// Whether the state machine has been initialised.
    pub initialized: bool,
    /// Padding for 8-byte alignment of `storage`.
    _pad: [u8; 7],
    /// Opaque storage for the underlying internal state machine.
    storage: [u64; NROS_LIFECYCLE_CTX_OPAQUE_U64S],
}

impl Default for nros_lifecycle_state_machine_t {
    fn default() -> Self {
        Self {
            initialized: false,
            _pad: [0; 7],
            storage: [0; NROS_LIFECYCLE_CTX_OPAQUE_U64S],
        }
    }
}

// ============================================================================
// Opaque-storage access helpers
// ============================================================================

#[inline]
unsafe fn inner_mut(
    sm: *mut nros_lifecycle_state_machine_t,
) -> &'static mut LifecyclePollingNodeCtx {
    let ptr = (*sm).storage.as_mut_ptr() as *mut LifecyclePollingNodeCtx;
    &mut *ptr
}

#[inline]
unsafe fn inner_ref(sm: *const nros_lifecycle_state_machine_t) -> &'static LifecyclePollingNodeCtx {
    let ptr = (*sm).storage.as_ptr() as *const LifecyclePollingNodeCtx;
    &*ptr
}

// ============================================================================
// Functions
// ============================================================================

/// Get a zero-initialized lifecycle state machine.
#[unsafe(no_mangle)]
pub extern "C" fn nros_lifecycle_get_zero_initialized() -> nros_lifecycle_state_machine_t {
    nros_lifecycle_state_machine_t::default()
}

/// Initialize a standalone lifecycle state machine.
///
/// The machine starts in `Unconfigured` with no callbacks registered. It is
/// self-contained: it drives the REP-2002 transition table and the six
/// transition callbacks, and it is NOT bound to a node and registers no ROS 2
/// services. A node that should answer `ros2 lifecycle set|get|list` uses the
/// executor-scoped family instead — `nros_executor_register_lifecycle_services`
/// plus `nros_executor_lifecycle_*`.
///
/// Issue 0792: this took a `const nros_node_t *node` until it was noticed that
/// the pointer was only NULL-checked and never stored. The parameter is gone
/// rather than stored, because storing it would not have helped: the
/// executor-scoped family does not reach its node through an `nros_node_t` at
/// all. `Executor::register_lifecycle_services` builds the five servers on the
/// executor's own session, keeps them in the executor's `LifecycleRuntimeState`,
/// and relies on the executor's spin loop to poll them — none of which a
/// caller-held `nros_lifecycle_state_machine_t` has access to or can be given
/// by a node pointer. The removed `nros_make_node_a_lifecycle_node` alias went
/// with it for the same reason: it could not do what its name said.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_lifecycle_init(
    sm: *mut nros_lifecycle_state_machine_t,
) -> nros_ret_t {
    if sm.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    if (*sm).initialized {
        return NROS_RET_BAD_SEQUENCE;
    }

    // Place a fresh LifecyclePollingNodeCtx into the opaque storage.
    let slot = (*sm).storage.as_mut_ptr() as *mut MaybeUninit<LifecyclePollingNodeCtx>;
    (*slot).write(LifecyclePollingNodeCtx::new());
    (*sm).initialized = true;

    NROS_RET_OK
}

/// Finalize a lifecycle state machine.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_lifecycle_fini(
    sm: *mut nros_lifecycle_state_machine_t,
) -> nros_ret_t {
    if sm.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    if !(*sm).initialized {
        return NROS_RET_NOT_INIT;
    }

    let inner = inner_mut(sm);
    inner.finalize();
    inner.clear_callbacks();
    (*sm).initialized = false;

    NROS_RET_OK
}

/// Trigger a lifecycle state transition.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_lifecycle_change_state(
    sm: *mut nros_lifecycle_state_machine_t,
    transition_id: u8,
) -> nros_ret_t {
    if sm.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    if !(*sm).initialized {
        return NROS_RET_NOT_INIT;
    }
    let Some(transition) = LifecycleTransition::from_u8(transition_id) else {
        return NROS_RET_INVALID_ARGUMENT;
    };

    match inner_mut(sm).trigger_transition(transition) {
        Ok(_) => NROS_RET_OK,
        Err(LifecycleError::InvalidTransition { .. }) => NROS_RET_BAD_SEQUENCE,
        Err(LifecycleError::CallbackFailed { .. }) => NROS_RET_ERROR,
        Err(LifecycleError::NodeFinalized) => NROS_RET_BAD_SEQUENCE,
    }
}

/// Get the current lifecycle state.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_lifecycle_get_current_state(
    sm: *const nros_lifecycle_state_machine_t,
) -> u8 {
    if sm.is_null() || !(*sm).initialized {
        return 0;
    }
    inner_ref(sm).state() as u8
}

/// Register a callback for the `configure` transition.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_lifecycle_register_on_configure(
    sm: *mut nros_lifecycle_state_machine_t,
    cb: nros_lifecycle_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    register(sm, LifecycleCallbackSlot::Configure, cb, context)
}

/// Register a callback for the `activate` transition.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_lifecycle_register_on_activate(
    sm: *mut nros_lifecycle_state_machine_t,
    cb: nros_lifecycle_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    register(sm, LifecycleCallbackSlot::Activate, cb, context)
}

/// Register a callback for the `deactivate` transition.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_lifecycle_register_on_deactivate(
    sm: *mut nros_lifecycle_state_machine_t,
    cb: nros_lifecycle_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    register(sm, LifecycleCallbackSlot::Deactivate, cb, context)
}

/// Register a callback for the `cleanup` transition.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_lifecycle_register_on_cleanup(
    sm: *mut nros_lifecycle_state_machine_t,
    cb: nros_lifecycle_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    register(sm, LifecycleCallbackSlot::Cleanup, cb, context)
}

/// Register a callback for the `shutdown` transition.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_lifecycle_register_on_shutdown(
    sm: *mut nros_lifecycle_state_machine_t,
    cb: nros_lifecycle_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    register(sm, LifecycleCallbackSlot::Shutdown, cb, context)
}

/// Register a callback for the `error` transition (error recovery).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_lifecycle_register_on_error(
    sm: *mut nros_lifecycle_state_machine_t,
    cb: nros_lifecycle_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    register(sm, LifecycleCallbackSlot::Error, cb, context)
}

// `nros_make_node_a_lifecycle_node` used to live here as a documented alias for
// `nros_lifecycle_init`, matching rclc's `rclc_make_node_a_lifecycle_node`.
// Issue 0792 removed it: it forwarded the node pointer to a function that only
// NULL-checked it, so a function named "make node a lifecycle node" did not make
// a node a lifecycle node. The capability it advertised is
// `nros_executor_register_lifecycle_services` (below), which is where the node's
// session and spin loop actually are.

// ============================================================================
// Internal helper
// ============================================================================

#[inline]
unsafe fn register(
    sm: *mut nros_lifecycle_state_machine_t,
    slot: LifecycleCallbackSlot,
    cb: nros_lifecycle_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    if sm.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    if !(*sm).initialized {
        return NROS_RET_NOT_INIT;
    }
    let inner = inner_mut(sm);
    inner.register(slot, cb);
    inner.set_context(context);
    NROS_RET_OK
}

// ============================================================================
// Executor-integrated lifecycle services (ROS 2 tooling surface)
// ============================================================================
//
// These functions are gated on `lifecycle-services` + an active RMW backend
// because the Executor type is only defined when an RMW is compiled in.
// They expose the state machine owned *by the Executor* (created during
// `nros_executor_register_lifecycle_services`) — distinct from the
// standalone `nros_lifecycle_state_machine_t` used by drivers that don't
// want ROS 2 tooling integration.

#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
mod service_backed {
    use super::*;
    use crate::executor::{get_executor, nros_executor_t};

    /// Register the five REP-2002 lifecycle services on the executor's node.
    ///
    /// After this call, `ros2 lifecycle set|get|list|nodes` can drive the
    /// executor-owned state machine. Register transition callbacks via
    /// `nros_executor_lifecycle_register_on_*` and inspect the state via
    /// `nros_executor_lifecycle_get_current_state`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn nros_executor_register_lifecycle_services(
        executor: *mut nros_executor_t,
    ) -> nros_ret_t {
        if executor.is_null() {
            return NROS_RET_INVALID_ARGUMENT;
        }
        let exec = unsafe { get_executor(&mut (*executor)._opaque) };
        match exec.register_lifecycle_services() {
            Ok(()) => NROS_RET_OK,
            Err(_) => NROS_RET_ERROR,
        }
    }

    /// Get the current lifecycle state of the executor's state machine.
    ///
    /// Returns `NROS_LIFECYCLE_STATE_UNCONFIGURED` if services are not
    /// registered yet.
    ///
    /// Takes `*mut` rather than `*const` because `get_executor` returns
    /// `&mut CExecutor` — reading the state is logically read-only but the
    /// executor accessor shares storage with the services loop that needs
    /// `&mut` during spin.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn nros_executor_lifecycle_get_current_state(
        executor: *mut nros_executor_t,
    ) -> u8 {
        if executor.is_null() {
            return NROS_LIFECYCLE_STATE_UNCONFIGURED;
        }
        let exec = unsafe { get_executor(&mut (*executor)._opaque) };
        match exec.lifecycle_state_machine() {
            Some(sm) => sm.state() as u8,
            None => NROS_LIFECYCLE_STATE_UNCONFIGURED,
        }
    }

    /// Trigger a lifecycle transition on the executor's state machine.
    ///
    /// # Safety
    /// Invokes the user's registered C callback through a raw function
    /// pointer. The caller must ensure the callback and any context it
    /// captures are live.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn nros_executor_lifecycle_change_state(
        executor: *mut nros_executor_t,
        transition_id: u8,
    ) -> nros_ret_t {
        if executor.is_null() {
            return NROS_RET_INVALID_ARGUMENT;
        }
        let exec = unsafe { get_executor(&mut (*executor)._opaque) };
        let Some(sm) = exec.lifecycle_state_machine_mut() else {
            return NROS_RET_NOT_INIT;
        };
        let Some(t) = LifecycleTransition::from_u8(transition_id) else {
            return NROS_RET_INVALID_ARGUMENT;
        };
        // SAFETY: forwarded through this function's unsafe contract.
        match unsafe { sm.trigger_transition(t) } {
            Ok(_) => NROS_RET_OK,
            Err(LifecycleError::NodeFinalized) => NROS_RET_BAD_SEQUENCE,
            Err(LifecycleError::InvalidTransition { .. }) => NROS_RET_INVALID_ARGUMENT,
            Err(LifecycleError::CallbackFailed { .. }) => NROS_RET_ERROR,
        }
    }

    #[inline]
    unsafe fn register_exec(
        executor: *mut nros_executor_t,
        slot: LifecycleCallbackSlot,
        cb: nros_lifecycle_callback_t,
        context: *mut c_void,
    ) -> nros_ret_t {
        if executor.is_null() {
            return NROS_RET_INVALID_ARGUMENT;
        }
        let exec = unsafe { get_executor(&mut (*executor)._opaque) };
        let Some(sm) = exec.lifecycle_state_machine_mut() else {
            return NROS_RET_NOT_INIT;
        };
        sm.register(slot, cb);
        sm.set_context(context);
        NROS_RET_OK
    }

    /// Register the on-configure callback on the executor's state machine.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn nros_executor_lifecycle_register_on_configure(
        executor: *mut nros_executor_t,
        cb: nros_lifecycle_callback_t,
        context: *mut c_void,
    ) -> nros_ret_t {
        unsafe { register_exec(executor, LifecycleCallbackSlot::Configure, cb, context) }
    }

    /// Register the on-activate callback on the executor's state machine.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn nros_executor_lifecycle_register_on_activate(
        executor: *mut nros_executor_t,
        cb: nros_lifecycle_callback_t,
        context: *mut c_void,
    ) -> nros_ret_t {
        unsafe { register_exec(executor, LifecycleCallbackSlot::Activate, cb, context) }
    }

    /// Register the on-deactivate callback on the executor's state machine.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn nros_executor_lifecycle_register_on_deactivate(
        executor: *mut nros_executor_t,
        cb: nros_lifecycle_callback_t,
        context: *mut c_void,
    ) -> nros_ret_t {
        unsafe { register_exec(executor, LifecycleCallbackSlot::Deactivate, cb, context) }
    }

    /// Register the on-cleanup callback on the executor's state machine.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn nros_executor_lifecycle_register_on_cleanup(
        executor: *mut nros_executor_t,
        cb: nros_lifecycle_callback_t,
        context: *mut c_void,
    ) -> nros_ret_t {
        unsafe { register_exec(executor, LifecycleCallbackSlot::Cleanup, cb, context) }
    }

    /// Register the on-shutdown callback on the executor's state machine.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn nros_executor_lifecycle_register_on_shutdown(
        executor: *mut nros_executor_t,
        cb: nros_lifecycle_callback_t,
        context: *mut c_void,
    ) -> nros_ret_t {
        unsafe { register_exec(executor, LifecycleCallbackSlot::Shutdown, cb, context) }
    }

    /// Register the on-error callback on the executor's state machine.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn nros_executor_lifecycle_register_on_error(
        executor: *mut nros_executor_t,
        cb: nros_lifecycle_callback_t,
        context: *mut c_void,
    ) -> nros_ret_t {
        unsafe { register_exec(executor, LifecycleCallbackSlot::Error, cb, context) }
    }
}

// ============================================================================
// Tests — focused on the FFI bridge; the state machine itself is tested in
// `nros_node::lifecycle::tests`.
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zero_initialized_and_init_fini() {
        unsafe {
            let mut sm = nros_lifecycle_get_zero_initialized();
            assert!(!sm.initialized);

            assert_eq!(nros_lifecycle_init(&mut sm), NROS_RET_OK);
            assert!(sm.initialized);
            assert_eq!(
                nros_lifecycle_get_current_state(&sm),
                NROS_LIFECYCLE_STATE_UNCONFIGURED
            );

            // Double-init rejected
            assert_eq!(nros_lifecycle_init(&mut sm), NROS_RET_BAD_SEQUENCE);

            assert_eq!(nros_lifecycle_fini(&mut sm), NROS_RET_OK);
            assert!(!sm.initialized);
            assert_eq!(nros_lifecycle_fini(&mut sm), NROS_RET_NOT_INIT);
        }
    }

    #[test]
    fn test_null_checks() {
        unsafe {
            assert_eq!(
                nros_lifecycle_init(core::ptr::null_mut()),
                NROS_RET_INVALID_ARGUMENT
            );
            assert_eq!(
                nros_lifecycle_change_state(
                    core::ptr::null_mut(),
                    NROS_LIFECYCLE_TRANSITION_CONFIGURE
                ),
                NROS_RET_INVALID_ARGUMENT
            );
            assert_eq!(nros_lifecycle_get_current_state(core::ptr::null()), 0);
        }
    }

    unsafe extern "C" fn cb_success(_ctx: *mut c_void) -> u8 {
        NROS_LIFECYCLE_RET_OK
    }

    unsafe extern "C" fn cb_failure(_ctx: *mut c_void) -> u8 {
        NROS_LIFECYCLE_RET_FAILURE
    }

    #[test]
    fn test_happy_path_through_ffi() {
        unsafe {
            let mut sm = nros_lifecycle_get_zero_initialized();
            nros_lifecycle_init(&mut sm);

            nros_lifecycle_register_on_configure(&mut sm, Some(cb_success), core::ptr::null_mut());
            nros_lifecycle_register_on_activate(&mut sm, Some(cb_success), core::ptr::null_mut());

            assert_eq!(
                nros_lifecycle_change_state(&mut sm, NROS_LIFECYCLE_TRANSITION_CONFIGURE),
                NROS_RET_OK
            );
            assert_eq!(
                nros_lifecycle_get_current_state(&sm),
                NROS_LIFECYCLE_STATE_INACTIVE
            );
            assert_eq!(
                nros_lifecycle_change_state(&mut sm, NROS_LIFECYCLE_TRANSITION_ACTIVATE),
                NROS_RET_OK
            );
            assert_eq!(
                nros_lifecycle_get_current_state(&sm),
                NROS_LIFECYCLE_STATE_ACTIVE
            );
        }
    }

    #[test]
    fn test_callback_failure_rolls_back() {
        unsafe {
            let mut sm = nros_lifecycle_get_zero_initialized();
            nros_lifecycle_init(&mut sm);

            nros_lifecycle_register_on_configure(&mut sm, Some(cb_failure), core::ptr::null_mut());
            assert_eq!(
                nros_lifecycle_change_state(&mut sm, NROS_LIFECYCLE_TRANSITION_CONFIGURE),
                NROS_RET_ERROR
            );
            assert_eq!(
                nros_lifecycle_get_current_state(&sm),
                NROS_LIFECYCLE_STATE_UNCONFIGURED
            );
        }
    }

    #[test]
    fn test_invalid_transition_id() {
        unsafe {
            let mut sm = nros_lifecycle_get_zero_initialized();
            nros_lifecycle_init(&mut sm);

            assert_eq!(
                nros_lifecycle_change_state(&mut sm, 99),
                NROS_RET_INVALID_ARGUMENT
            );
        }
    }

    #[test]
    fn test_context_passed_through() {
        use core::sync::atomic::{AtomicUsize, Ordering};
        static SEEN: AtomicUsize = AtomicUsize::new(0);
        unsafe extern "C" fn cb(ctx: *mut c_void) -> u8 {
            SEEN.store(ctx as usize, Ordering::Relaxed);
            NROS_LIFECYCLE_RET_OK
        }

        unsafe {
            let mut sm = nros_lifecycle_get_zero_initialized();
            nros_lifecycle_init(&mut sm);

            nros_lifecycle_register_on_configure(&mut sm, Some(cb), 0xDEAD as *mut c_void);
            nros_lifecycle_change_state(&mut sm, NROS_LIFECYCLE_TRANSITION_CONFIGURE);
            assert_eq!(SEEN.load(Ordering::Relaxed), 0xDEAD);
        }
    }

    /// Issue 0792 — a NULL callback clears the slot rather than being rejected,
    /// which is what makes `nros_lifecycle_callback_t` a *nullable* pointer
    /// rather than a plain one.
    #[test]
    fn test_null_callback_clears_slot() {
        unsafe {
            let mut sm = nros_lifecycle_get_zero_initialized();
            nros_lifecycle_init(&mut sm);

            nros_lifecycle_register_on_configure(&mut sm, Some(cb_failure), core::ptr::null_mut());
            assert_eq!(
                nros_lifecycle_change_state(&mut sm, NROS_LIFECYCLE_TRANSITION_CONFIGURE),
                NROS_RET_ERROR
            );

            // NULL == no callback == the transition succeeds by default.
            nros_lifecycle_register_on_configure(&mut sm, None, core::ptr::null_mut());
            assert_eq!(
                nros_lifecycle_change_state(&mut sm, NROS_LIFECYCLE_TRANSITION_CONFIGURE),
                NROS_RET_OK
            );
            assert_eq!(
                nros_lifecycle_get_current_state(&sm),
                NROS_LIFECYCLE_STATE_INACTIVE
            );
        }
    }
}
