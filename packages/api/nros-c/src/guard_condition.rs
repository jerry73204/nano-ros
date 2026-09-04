//! Guard condition API for nros C API.
//!
//! Guard conditions provide a mechanism for signaling the executor from
//! another thread. They are used for shutdown requests, custom triggers,
//! and inter-thread communication.

use core::{ffi::c_void, ptr};

use crate::{
    constants::GUARD_HANDLE_OPAQUE_U64S,
    error::*,
    support::{nros_support_state_t, nros_support_t},
};

// ============================================================================
// Guard Condition Types
// ============================================================================

/// Guard condition state.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_guard_condition_state_t {
    /// Not initialized
    NROS_GUARD_CONDITION_STATE_UNINITIALIZED = 0,
    /// Initialized and ready
    NROS_GUARD_CONDITION_STATE_INITIALIZED = 1,
    /// Shutdown
    NROS_GUARD_CONDITION_STATE_SHUTDOWN = 2,
}

/// Guard condition callback type.
pub type nros_guard_condition_callback_t = Option<unsafe extern "C" fn(context: *mut c_void)>;

/// Guard condition structure.
#[repr(C)]
pub struct nros_guard_condition_t {
    /// Current state
    pub state: nros_guard_condition_state_t,
    /// Triggered flag (volatile for cross-thread visibility)
    pub triggered: bool,
    /// Callback function
    pub callback: nros_guard_condition_callback_t,
    /// User context pointer
    pub context: *mut c_void,
    /// Pointer to parent support context
    pub _support: *const nros_support_t,
    /// Handle ID from executor registration (SIZE_MAX = not registered)
    pub handle_id: usize,
    /// Whether the guard handle has been initialized
    pub _guard_valid: bool,
    /// Inline opaque storage for the guard condition handle (set by executor).
    /// Avoids heap allocation — managed by executor registration / guard_condition_fini.
    pub _guard_opaque: [u64; GUARD_HANDLE_OPAQUE_U64S],
}

// GUARD_HANDLE_OPAQUE_U64S is computed from size_of::<GuardCondition>() in
// opaque_sizes.rs — always large enough by construction.

// Safety: The triggered flag is designed for cross-thread access.
// The callback and context are only accessed from the executor thread.
unsafe impl Send for nros_guard_condition_t {}
unsafe impl Sync for nros_guard_condition_t {}

impl Default for nros_guard_condition_t {
    fn default() -> Self {
        Self {
            state: nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_UNINITIALIZED,
            triggered: false,
            callback: None,
            context: ptr::null_mut(),
            _support: ptr::null(),
            handle_id: usize::MAX,
            _guard_valid: false,
            _guard_opaque: [0u64; GUARD_HANDLE_OPAQUE_U64S],
        }
    }
}

impl nros_guard_condition_t {
    /// Get the callback function.
    pub(crate) fn get_callback(&self) -> nros_guard_condition_callback_t {
        self.callback
    }

    /// Get the context pointer.
    pub(crate) fn get_context(&self) -> *mut c_void {
        self.context
    }

    /// Set the handle ID from executor registration.
    pub(crate) fn set_handle_id(&mut self, id: nros_node::HandleId) {
        self.handle_id = id.0;
    }

    /// Set the guard handle for external triggering.
    pub(crate) fn set_guard_handle(&mut self, handle: nros_node::GuardCondition) {
        unsafe {
            core::ptr::write(
                self._guard_opaque.as_mut_ptr() as *mut nros_node::GuardCondition,
                handle,
            );
        }
        self._guard_valid = true;
    }

    /// Get the guard handle for triggering.
    pub(crate) fn get_guard_handle(&self) -> Option<&nros_node::GuardCondition> {
        if !self._guard_valid {
            None
        } else {
            Some(unsafe { &*(self._guard_opaque.as_ptr() as *const nros_node::GuardCondition) })
        }
    }
}

// ============================================================================
// Guard Condition Functions
// ============================================================================

/// Get a zero-initialized guard condition.
#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_guard_condition() -> nros_guard_condition_t {
    nros_guard_condition_t::default()
}

/// Initialize a guard condition.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_guard_condition_init(
    guard: *mut nros_guard_condition_t,
    support: *const nros_support_t,
) -> nros_ret_t {
    validate_not_null!(guard, support);

    let guard = &mut *guard;
    let support_ref = &*support;

    validate_state!(
        guard,
        nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_UNINITIALIZED,
        NROS_RET_BAD_SEQUENCE
    );
    validate_state!(
        support_ref,
        nros_support_state_t::NROS_SUPPORT_STATE_INITIALIZED
    );

    guard._support = support;
    guard.triggered = false;
    guard.callback = None;
    guard.context = ptr::null_mut();
    guard.state = nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_INITIALIZED;

    NROS_RET_OK
}

/// Set the guard condition callback.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_guard_condition_set_callback(
    guard: *mut nros_guard_condition_t,
    callback: nros_guard_condition_callback_t,
    context: *mut c_void,
) -> nros_ret_t {
    validate_not_null!(guard);

    let guard = &mut *guard;

    validate_state!(
        guard,
        nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_INITIALIZED
    );

    guard.callback = callback;
    guard.context = context;

    NROS_RET_OK
}

/// Trigger a guard condition.
///
/// This function is designed to be thread-safe. When registered with an
/// executor, it triggers via the executor's guard handle (atomic flag in
/// the arena). Otherwise falls back to the local triggered flag.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_trigger_guard_condition(
    guard: *mut nros_guard_condition_t,
) -> nros_ret_t {
    validate_not_null!(guard);

    let guard = &mut *guard;

    validate_state!(
        guard,
        nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_INITIALIZED
    );

    // If registered with an executor, trigger via the executor's guard handle
    if let Some(handle) = guard.get_guard_handle() {
        handle.trigger();
        return NROS_RET_OK;
    }

    // Fallback: use platform atomic operation for thread-safety
    crate::platform::atomic_store_bool(&mut guard.triggered as *mut bool, true);

    NROS_RET_OK
}

/// Check if the guard condition is triggered.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_guard_condition_is_triggered(
    guard: *const nros_guard_condition_t,
) -> bool {
    if guard.is_null() {
        return false;
    }

    let guard = &*guard;

    if guard.state != nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_INITIALIZED {
        return false;
    }

    // Use platform atomic operation for thread-safety
    crate::platform::atomic_load_bool(&guard.triggered as *const bool)
}

/// Clear the triggered flag.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_guard_condition_clear(
    guard: *mut nros_guard_condition_t,
) -> nros_ret_t {
    validate_not_null!(guard);

    let guard = &mut *guard;

    // Use platform atomic operation for thread-safety
    crate::platform::atomic_store_bool(&mut guard.triggered as *mut bool, false);

    NROS_RET_OK
}

/// Check if guard condition is valid (initialized).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_guard_condition_is_valid(
    guard: *const nros_guard_condition_t,
) -> bool {
    if guard.is_null() {
        return false;
    }

    let guard = &*guard;
    guard.state == nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_INITIALIZED
}

/// Finalize a guard condition.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_guard_condition_fini(
    guard: *mut nros_guard_condition_t,
) -> nros_ret_t {
    validate_not_null!(guard);

    let guard = &mut *guard;

    validate_state!(
        guard,
        nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_INITIALIZED
    );

    // Drop the inline guard handle if initialized
    if guard._guard_valid {
        core::ptr::drop_in_place(guard._guard_opaque.as_mut_ptr() as *mut nros_node::GuardCondition);
    }

    guard.triggered = false;
    guard.callback = None;
    guard.context = ptr::null_mut();
    guard._support = ptr::null();
    guard.handle_id = usize::MAX;
    guard._guard_valid = false;
    guard._guard_opaque = [0u64; GUARD_HANDLE_OPAQUE_U64S];
    guard.state = nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_SHUTDOWN;

    NROS_RET_OK
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_guard_condition_default() {
        let guard = rcl_get_zero_initialized_guard_condition();
        assert_eq!(
            guard.state,
            nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_UNINITIALIZED
        );
        assert!(!guard.triggered);
        assert!(guard.callback.is_none());
        assert!(guard.context.is_null());
    }

    #[test]
    fn test_guard_condition_init_null_guard() {
        unsafe {
            let support = crate::support::nros_support_get_zero_initialized();
            let ret = nros_guard_condition_init(ptr::null_mut(), &support);
            assert_eq!(ret, NROS_RET_INVALID_ARGUMENT);
        }
    }

    #[test]
    fn test_guard_condition_init_null_support() {
        unsafe {
            let mut guard = rcl_get_zero_initialized_guard_condition();
            let ret = nros_guard_condition_init(&mut guard, ptr::null());
            assert_eq!(ret, NROS_RET_INVALID_ARGUMENT);
        }
    }

    #[test]
    fn test_guard_condition_trigger_not_init() {
        unsafe {
            let mut guard = rcl_get_zero_initialized_guard_condition();
            let ret = rcl_trigger_guard_condition(&mut guard);
            assert_eq!(ret, NROS_RET_NOT_INIT);
        }
    }

    #[test]
    fn test_guard_condition_trigger_null() {
        unsafe {
            let ret = rcl_trigger_guard_condition(ptr::null_mut());
            assert_eq!(ret, NROS_RET_INVALID_ARGUMENT);
        }
    }

    #[test]
    fn test_guard_condition_is_triggered_null() {
        unsafe {
            let result = nros_guard_condition_is_triggered(ptr::null());
            assert!(!result);
        }
    }

    #[test]
    fn test_guard_condition_is_triggered_not_init() {
        unsafe {
            let guard = rcl_get_zero_initialized_guard_condition();
            let result = nros_guard_condition_is_triggered(&guard);
            assert!(!result);
        }
    }

    #[test]
    fn test_guard_condition_clear_null() {
        unsafe {
            let ret = nros_guard_condition_clear(ptr::null_mut());
            assert_eq!(ret, NROS_RET_INVALID_ARGUMENT);
        }
    }

    #[test]
    fn test_guard_condition_is_valid_null() {
        unsafe {
            let result = nros_guard_condition_is_valid(ptr::null());
            assert!(!result);
        }
    }

    #[test]
    fn test_guard_condition_is_valid_not_init() {
        unsafe {
            let guard = rcl_get_zero_initialized_guard_condition();
            let result = nros_guard_condition_is_valid(&guard);
            assert!(!result);
        }
    }

    #[test]
    fn test_guard_condition_fini_null() {
        unsafe {
            let ret = rcl_guard_condition_fini(ptr::null_mut());
            assert_eq!(ret, NROS_RET_INVALID_ARGUMENT);
        }
    }

    #[test]
    fn test_guard_condition_fini_not_init() {
        unsafe {
            let mut guard = rcl_get_zero_initialized_guard_condition();
            let ret = rcl_guard_condition_fini(&mut guard);
            assert_eq!(ret, NROS_RET_NOT_INIT);
        }
    }

    #[test]
    fn test_guard_condition_set_callback_null() {
        unsafe {
            let ret = nros_guard_condition_set_callback(ptr::null_mut(), None, ptr::null_mut());
            assert_eq!(ret, NROS_RET_INVALID_ARGUMENT);
        }
    }

    #[test]
    fn test_guard_condition_set_callback_not_init() {
        unsafe {
            let mut guard = rcl_get_zero_initialized_guard_condition();
            let ret = nros_guard_condition_set_callback(&mut guard, None, ptr::null_mut());
            assert_eq!(ret, NROS_RET_NOT_INIT);
        }
    }

    // Test with a mock initialized guard condition
    #[test]
    fn test_guard_condition_trigger_and_clear() {
        unsafe {
            // Manually set up an initialized guard condition (bypassing support check)
            let mut guard = rcl_get_zero_initialized_guard_condition();
            guard.state = nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_INITIALIZED;

            // Initially not triggered
            assert!(!nros_guard_condition_is_triggered(&guard));

            // Trigger it
            let ret = rcl_trigger_guard_condition(&mut guard);
            assert_eq!(ret, NROS_RET_OK);
            assert!(nros_guard_condition_is_triggered(&guard));

            // Clear it
            let ret = nros_guard_condition_clear(&mut guard);
            assert_eq!(ret, NROS_RET_OK);
            assert!(!nros_guard_condition_is_triggered(&guard));
        }
    }

    #[test]
    fn test_guard_condition_is_valid_initialized() {
        unsafe {
            let mut guard = rcl_get_zero_initialized_guard_condition();
            guard.state = nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_INITIALIZED;

            let result = nros_guard_condition_is_valid(&guard);
            assert!(result);
        }
    }

    #[test]
    fn test_guard_condition_fini_initialized() {
        unsafe {
            let mut guard = rcl_get_zero_initialized_guard_condition();
            guard.state = nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_INITIALIZED;
            guard.triggered = true;

            let ret = rcl_guard_condition_fini(&mut guard);
            assert_eq!(ret, NROS_RET_OK);
            assert_eq!(
                guard.state,
                nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_SHUTDOWN
            );
            assert!(!guard.triggered);
        }
    }

    // Test callback storage
    unsafe extern "C" fn test_callback(_context: *mut c_void) {}

    #[test]
    fn test_guard_condition_set_callback_initialized() {
        unsafe {
            let mut guard = rcl_get_zero_initialized_guard_condition();
            guard.state = nros_guard_condition_state_t::NROS_GUARD_CONDITION_STATE_INITIALIZED;

            let context_value: i32 = 42;
            let ret = nros_guard_condition_set_callback(
                &mut guard,
                Some(test_callback),
                &context_value as *const i32 as *mut c_void,
            );
            assert_eq!(ret, NROS_RET_OK);
            assert!(guard.get_callback().is_some());
            assert!(!guard.get_context().is_null());
        }
    }
}
