//! Phase 269 (W0) — executor-shim: lifecycle FFI over the CppContext handle.
//!
//! Mirrors `nros-c/src/lifecycle.rs`'s service-backed module but recovers the
//! executor from `CppContext*` instead of `nros_executor_t*`. W1/W2 emitters
//! call these; no emitter wires them yet this wave.
//!
//! Issue 0436 — user-supplied executor handles are tag-validated via
//! `cpp_ctx_checked` instead of being blind-cast to `*mut CppContext`.

#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
use core::ffi::c_void;

#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
use nros_node::LifecycleTransition;

#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
use nros_node::lifecycle::{LifecycleCallbackSlot, LifecycleError};

/// C callback type for a lifecycle transition: `uint8_t callback(void* context)`.
///
/// The return value is a REP-2002 `TransitionResult` (`Success = 0`, `Failure = 1`,
/// `Error = 2`). Registered per transition via `nros_cpp_lifecycle_register_on_*`;
/// all six callbacks on one node share the single `ctx` set by the last register call
/// (the C++ `LifecycleNode` wrapper always passes `this`).
#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
pub type nros_cpp_lifecycle_callback_t = unsafe extern "C" fn(*mut c_void) -> u8;

#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
use crate::{
    NROS_CPP_RET_ERROR, NROS_CPP_RET_INVALID_ARGUMENT, NROS_CPP_RET_NOT_INIT, NROS_CPP_RET_OK,
    cpp_ctx_checked, nros_cpp_ret_t,
};

/// Register the five REP-2002 lifecycle services on the C++ executor's node.
///
/// # Safety
/// `executor` must be a valid, live `CppContext*` produced by `nros_cpp_init`.
#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_register_lifecycle_services(
    executor: *mut c_void,
) -> nros_cpp_ret_t {
    let Some(ctx) = (unsafe { cpp_ctx_checked(executor) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    match ctx.executor.register_lifecycle_services() {
        Ok(()) => NROS_CPP_RET_OK,
        Err(_) => NROS_CPP_RET_ERROR,
    }
}

/// Trigger a lifecycle transition on the C++ executor's state machine.
///
/// `transition_id` is a `lifecycle_msgs/msg/Transition` id — the SAME numbering
/// rclcpp uses (issue 1099): Configure=1, Cleanup=2, Activate=3, Deactivate=4,
/// UnconfiguredShutdown=5, InactiveShutdown=6, ActiveShutdown=7,
/// ErrorRecovery=60. `0` (CREATE) and `8` (DESTROY) are not implemented and
/// return `INVALID_ARGUMENT`.
///
/// (This comment used to read "Configure=1, Activate=2, Deactivate=3,
/// Cleanup=4, Shutdown=5, ErrorProcessed=6" — three of those were nano-ros's
/// own numbering and `ErrorProcessed=6` was never a transition at all.)
///
/// # Safety
/// `executor` must be a valid, live `CppContext*`. Any registered transition
/// callbacks are invoked through raw function pointers; the caller must ensure
/// they and their captured context remain live.
#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_lifecycle_change_state(
    executor: *mut c_void,
    // `transition_id` is a `lifecycle_msgs/msg/Transition` id (issue 1099) —
    // the same number rclcpp means. Unimplemented ids (`0` CREATE, `8`
    // DESTROY) fall out of `from_u8` as `INVALID_ARGUMENT`.
    transition_id: u8,
) -> nros_cpp_ret_t {
    let Some(ctx) = (unsafe { cpp_ctx_checked(executor) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let Some(sm) = ctx.executor.lifecycle_state_machine_mut() else {
        return NROS_CPP_RET_NOT_INIT;
    };
    let Some(t) = LifecycleTransition::from_u8(transition_id) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    // SAFETY: forwarded through this function's unsafe contract.
    match unsafe { sm.trigger_transition(t) } {
        Ok(_) => NROS_CPP_RET_OK,
        Err(LifecycleError::NodeFinalized) => NROS_CPP_RET_ERROR,
        Err(LifecycleError::InvalidTransition { .. }) => NROS_CPP_RET_INVALID_ARGUMENT,
        Err(LifecycleError::CallbackFailed { .. }) => NROS_CPP_RET_ERROR,
    }
}

/// Get the current REP-2002 lifecycle state of the C++ executor's state machine.
///
/// Returns `0` if the executor is null or lifecycle services are not registered
/// yet — that is the `Unknown` sentinel (`nros::LifecycleState::Unknown`), NOT
/// `Unconfigured`. State numbering is `lifecycle_msgs/msg/State`'s:
/// `Unconfigured = 1`, `Inactive = 2`, `Active = 3`, `Finalized = 4`, plus
/// `ErrorProcessing = 5` (upstream numbers that one 15; see
/// `nros_core::lifecycle`). This comment previously documented a 0-based
/// numbering that has never been what the function returns.
///
/// # Safety
/// `executor` must be a valid, live `CppContext*` produced by `nros_cpp_init`.
#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_lifecycle_get_current_state(executor: *mut c_void) -> u8 {
    let Some(ctx) = (unsafe { cpp_ctx_checked(executor) }) else {
        return 0;
    };
    match ctx.executor.lifecycle_state_machine() {
        Some(sm) => sm.state() as u8,
        None => 0,
    }
}

/// Shared body for the six `nros_cpp_lifecycle_register_on_*` shims: recover the
/// `CppContext`'s executor state machine, install `cb` in `slot`, and point the
/// machine's single callback context at `ctx`.
///
/// # Safety
/// `executor` must be a valid, live `CppContext*`; `cb` and `ctx` must remain live
/// for as long as the callback can fire.
#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
unsafe fn register_cpp(
    executor: *mut c_void,
    slot: LifecycleCallbackSlot,
    cb: nros_cpp_lifecycle_callback_t,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    let Some(context) = (unsafe { cpp_ctx_checked(executor) }) else {
        return NROS_CPP_RET_INVALID_ARGUMENT;
    };
    let Some(sm) = context.executor.lifecycle_state_machine_mut() else {
        return NROS_CPP_RET_NOT_INIT;
    };
    sm.register(slot, Some(cb));
    sm.set_context(ctx);
    NROS_CPP_RET_OK
}

/// Register the `on_configure` transition callback on the C++ executor's state machine.
///
/// # Safety
/// See [`register_cpp`].
#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_lifecycle_register_on_configure(
    executor: *mut c_void,
    cb: nros_cpp_lifecycle_callback_t,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    unsafe { register_cpp(executor, LifecycleCallbackSlot::Configure, cb, ctx) }
}

/// Register the `on_activate` transition callback on the C++ executor's state machine.
///
/// # Safety
/// See [`register_cpp`].
#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_lifecycle_register_on_activate(
    executor: *mut c_void,
    cb: nros_cpp_lifecycle_callback_t,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    unsafe { register_cpp(executor, LifecycleCallbackSlot::Activate, cb, ctx) }
}

/// Register the `on_deactivate` transition callback on the C++ executor's state machine.
///
/// # Safety
/// See [`register_cpp`].
#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_lifecycle_register_on_deactivate(
    executor: *mut c_void,
    cb: nros_cpp_lifecycle_callback_t,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    unsafe { register_cpp(executor, LifecycleCallbackSlot::Deactivate, cb, ctx) }
}

/// Register the `on_cleanup` transition callback on the C++ executor's state machine.
///
/// # Safety
/// See [`register_cpp`].
#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_lifecycle_register_on_cleanup(
    executor: *mut c_void,
    cb: nros_cpp_lifecycle_callback_t,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    unsafe { register_cpp(executor, LifecycleCallbackSlot::Cleanup, cb, ctx) }
}

/// Register the `on_shutdown` transition callback on the C++ executor's state machine.
///
/// # Safety
/// See [`register_cpp`].
#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_lifecycle_register_on_shutdown(
    executor: *mut c_void,
    cb: nros_cpp_lifecycle_callback_t,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    unsafe { register_cpp(executor, LifecycleCallbackSlot::Shutdown, cb, ctx) }
}

/// Register the `on_error` transition callback on the C++ executor's state machine.
///
/// # Safety
/// See [`register_cpp`].
#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_lifecycle_register_on_error(
    executor: *mut c_void,
    cb: nros_cpp_lifecycle_callback_t,
    ctx: *mut c_void,
) -> nros_cpp_ret_t {
    unsafe { register_cpp(executor, LifecycleCallbackSlot::Error, cb, ctx) }
}

/// Register lifecycle services and optionally drive the node to a higher
/// autostart state.
///
/// `autostart_code`: 0 = services only (none), 1 = configure, 2 = active.
///
/// # Safety
/// Same as [`nros_cpp_register_lifecycle_services`] and
/// [`nros_cpp_lifecycle_change_state`].
#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_lifecycle_autostart(
    executor: *mut c_void,
    autostart_code: u8,
) -> nros_cpp_ret_t {
    let ret = unsafe { nros_cpp_register_lifecycle_services(executor) };
    if ret != NROS_CPP_RET_OK {
        return ret;
    }
    // autostart_code: 1 = configure, 2 = configure + activate.
    //
    // Issue 1099 — spelled through the enum, not as literals. These WERE `1`
    // and `2`; `2` is `lifecycle_msgs`' CLEANUP, so once the ids became
    // upstream's, an autostart-to-Active would have configured the node and
    // then immediately cleaned it back up.
    if autostart_code >= 1 {
        let ret = unsafe {
            nros_cpp_lifecycle_change_state(executor, LifecycleTransition::Configure as u8)
        };
        if ret != NROS_CPP_RET_OK {
            return ret;
        }
    }
    if autostart_code >= 2 {
        let ret = unsafe {
            nros_cpp_lifecycle_change_state(executor, LifecycleTransition::Activate as u8)
        };
        if ret != NROS_CPP_RET_OK {
            return ret;
        }
    }
    NROS_CPP_RET_OK
}

#[cfg(test)]
#[cfg(all(feature = "lifecycle-services", feature = "rmw-cffi"))]
mod tests {
    use core::ptr;

    use super::*;

    unsafe extern "C" fn dummy_cb(_ctx: *mut core::ffi::c_void) -> u8 {
        0
    }

    /// Null-pointer guard: every shim fn returns INVALID_ARGUMENT for a null executor.
    #[test]
    fn null_executor_returns_invalid_argument() {
        let ret = unsafe { nros_cpp_register_lifecycle_services(ptr::null_mut()) };
        assert_eq!(ret, NROS_CPP_RET_INVALID_ARGUMENT);
        let ret = unsafe { nros_cpp_lifecycle_change_state(ptr::null_mut(), 1) };
        assert_eq!(ret, NROS_CPP_RET_INVALID_ARGUMENT);
        let ret = unsafe { nros_cpp_lifecycle_autostart(ptr::null_mut(), 0) };
        assert_eq!(ret, NROS_CPP_RET_INVALID_ARGUMENT);
        for reg in [
            nros_cpp_lifecycle_register_on_configure as unsafe extern "C" fn(_, _, _) -> _,
            nros_cpp_lifecycle_register_on_activate,
            nros_cpp_lifecycle_register_on_deactivate,
            nros_cpp_lifecycle_register_on_cleanup,
            nros_cpp_lifecycle_register_on_shutdown,
            nros_cpp_lifecycle_register_on_error,
        ] {
            let ret = unsafe { reg(ptr::null_mut(), dummy_cb, ptr::null_mut()) };
            assert_eq!(ret, NROS_CPP_RET_INVALID_ARGUMENT);
        }
    }

    /// Null executor reads back as `Unconfigured` (0) rather than trapping.
    #[test]
    fn null_executor_get_current_state_is_unconfigured() {
        assert_eq!(
            unsafe { nros_cpp_lifecycle_get_current_state(ptr::null_mut()) },
            0
        );
    }
}
