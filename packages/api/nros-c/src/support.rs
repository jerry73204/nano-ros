//! Support context for nros C API.
//!
//! The support context manages the underlying middleware session and provides
//! shared resources for nodes, publishers, and subscribers.

use core::ffi::c_char;

use crate::{
    constants::{MAX_LOCATOR_LEN, SESSION_OPAQUE_U64S},
    error::*,
};

/// Phase 155.B — collapse `nros_rmw::TransportError` into the closest
/// matching `NROS_RET_*` so the C-side `nros_support_init` error log
/// (`init -> -X`) tells the user which precondition the backend
/// rejected. Anything not directly classifiable falls back to
/// `NROS_RET_ERROR` (-1) — the legacy value, preserving the existing
/// "something went wrong, dig into the backend log" semantics for
/// callers that branch on `== NROS_RET_ERROR`.
#[cfg(feature = "rmw-cffi")]
pub(crate) fn transport_error_to_ret(err: nros_rmw::TransportError) -> nros_ret_t {
    use nros_rmw::TransportError as E;
    match err {
        E::ConnectionFailed | E::Disconnected => NROS_RET_NOT_FOUND,
        E::Timeout => NROS_RET_TIMEOUT,
        E::WouldBlock | E::NoData => NROS_RET_TRY_AGAIN,
        E::InvalidConfig | E::InvalidArgument | E::TopicNameInvalid | E::NodeNameNonExistent => {
            NROS_RET_INVALID_ARGUMENT
        }
        E::BufferTooSmall | E::MessageTooLarge | E::TooLarge | E::BadAlloc => NROS_RET_FULL,
        E::PublishFailed => NROS_RET_PUBLISH_FAILED,
        E::ServiceRequestFailed | E::ServiceReplyFailed => NROS_RET_SERVICE_FAILED,
        E::Unsupported | E::LoanNotSupported => NROS_RET_NOT_ALLOWED,
        E::IncompatibleQos | E::IncompatibleAbi => NROS_RET_REJECTED,
        E::PublisherCreationFailed
        | E::SubscriberCreationFailed
        | E::ServiceServerCreationFailed
        | E::ServiceClientCreationFailed
        | E::SerializationError
        | E::DeserializationError
        | E::TaskStartFailed
        | E::PollFailed
        | E::KeepaliveFailed
        | E::JoinFailed => NROS_RET_ERROR,
        // Backend / BackendDynamic carry a backend-defined string that
        // can't ride through a `nros_ret_t`. Keep them at the generic
        // catch-all so callers branching on `== NROS_RET_ERROR` still
        // catch backend faults; the string is lost (TODO: surface via
        // a side-channel debug-log macro for diagnostic builds).
        _ => NROS_RET_ERROR,
    }
}

/// Issue #227 — pass this as `domain_id` to `nros_support_init[_named]` to
/// request an EXPLICIT domain 0. Plain `0` is the UNSET sentinel (defers to
/// `ROS_DOMAIN_ID` env on hosted, then the baked/default rungs — the #206
/// model-A ladder). Valid domains cap at 232, so 255 is unambiguous. Hosted
/// env still overrides it, like every explicit argument under model A.
pub const NROS_DOMAIN_ID_EXPLICIT_ZERO: u8 = 255;

/// Support context state
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_support_state_t {
    /// Not initialized
    NROS_SUPPORT_STATE_UNINITIALIZED = 0,
    /// Initialized and ready
    NROS_SUPPORT_STATE_INITIALIZED = 1,
    /// Shutdown
    NROS_SUPPORT_STATE_SHUTDOWN = 2,
}

/// Support context structure.
///
/// This is the main context for nros, similar to rclc_support_t.
/// It manages the middleware session and provides shared resources.
#[repr(C)]
pub struct nros_support_t {
    /// Current state
    pub state: nros_support_state_t,
    /// Domain ID (ROS_DOMAIN_ID)
    pub domain_id: u8,
    /// Locator string storage
    pub locator: [u8; MAX_LOCATOR_LEN],
    /// Locator string length
    pub locator_len: usize,
    /// Inline opaque storage for the RMW session.
    /// Avoids heap allocation — managed by nros_support_init/fini.
    pub _opaque: [u64; SESSION_OPAQUE_U64S],
}

// SESSION_OPAQUE_U64S is computed from size_of::<RmwSession>() in opaque_sizes.rs —
// always large enough by construction.

impl Default for nros_support_t {
    fn default() -> Self {
        Self {
            state: nros_support_state_t::NROS_SUPPORT_STATE_UNINITIALIZED,
            domain_id: 0,
            locator: [0u8; MAX_LOCATOR_LEN],
            locator_len: 0,
            _opaque: [0u64; SESSION_OPAQUE_U64S],
        }
    }
}

/// Get a zero-initialized support context.
///
/// # Safety
/// Returns a stack-allocated struct that must be initialized before use.
#[unsafe(no_mangle)]
pub extern "C" fn nros_support_get_zero_initialized() -> nros_support_t {
    nros_support_t::default()
}

/// Initialize the support context.
///
/// This function initializes the middleware session and prepares the context
/// for creating nodes, publishers, and subscribers.
///
/// # Parameters
/// * `support` - Pointer to a zero-initialized support context
/// * `locator` - Middleware locator string (e.g., "tcp/127.0.0.1:7447"), or NULL for default
/// * `domain_id` - ROS domain ID (0-232)
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if support is NULL
/// * `NROS_RET_ERROR` on initialization failure
///
/// # Safety
/// * `support` must be a valid pointer to a zero-initialized nros_support_t
/// * `locator` must be a valid null-terminated string or NULL
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_support_init(
    support: *mut nros_support_t,
    locator: *const c_char,
    domain_id: u8,
) -> nros_ret_t {
    unsafe { nros_support_init_named(support, locator, domain_id, core::ptr::null()) }
}

/// Initialize the support context with a session name.
///
/// Like `nros_support_init`, but allows specifying a session name for
/// XRCE-DDS. Different XRCE clients on the same agent MUST use different
/// session names; otherwise the agent treats them as the same client and
/// won't relay data between them.
///
/// # Parameters
/// * `support` - Pointer to a zero-initialized support context
/// * `locator` - Middleware locator string, or NULL for default
/// * `domain_id` - ROS domain ID (0-232)
/// * `session_name` - Session name for XRCE key derivation, or NULL for default
///
/// # Safety
/// * `support` must be a valid pointer to a zero-initialized nros_support_t
/// * `locator` and `session_name` must be valid null-terminated strings or NULL
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_support_init_named(
    support: *mut nros_support_t,
    locator: *const c_char,
    domain_id: u8,
    session_name: *const c_char,
) -> nros_ret_t {
    // Issue 1050 defect (3) — NULL selector: this image names no backend, so
    // the registry must hold exactly one.
    unsafe { nros_support_init_rmw(support, locator, domain_id, session_name, core::ptr::null()) }
}

/// Issue 1050 defect (3) — [`nros_support_init_named`] with an explicit RMW
/// selector.
///
/// `rmw` is the BAKED rung of precedence model A (RFC-0045): a hosted
/// `$NROS_RMW` still wins, NULL or `""` means "no selector", and a selector
/// that names no registered backend is an error rather than a fallback.
///
/// The C surface needed this most. Its open path
/// (`nros::internals::open_session`) never consulted `resolve_backend` at all —
/// it took registry slot 0 whenever `$NROS_RMW` was unset, so a second backend
/// registering from an `.init_array` ctor before `main` silently won. That is
/// fixed at the source (`nros_rmw_cffi::get_vtable` resolves once, for every
/// language), and this is how a C image says which one it meant.
///
/// # Safety
/// As [`nros_support_init_named`], plus: `rmw` must be a valid NUL-terminated
/// string or NULL.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_support_init_rmw(
    support: *mut nros_support_t,
    locator: *const c_char,
    domain_id: u8,
    session_name: *const c_char,
    rmw: *const c_char,
) -> nros_ret_t {
    if support.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let support = &mut *support;

    // Check if already initialized
    if support.state != nros_support_state_t::NROS_SUPPORT_STATE_UNINITIALIZED {
        return NROS_RET_BAD_SEQUENCE;
    }

    // RFC-0045 / issue #206 — route locator + domain through the ONE
    // boot-config resolver (precedence model A: hosted env > baked >
    // default). This gives the C API the same NROS_LOCATOR/ROS_DOMAIN_ID
    // env overlay Rust and C++ have (previously: none), with the same
    // validation (malformed / > DOMAIN_ID_MAX ROS_DOMAIN_ID = error, never
    // silent 0). The BAKED rung = the caller's explicit args, falling back
    // to the historical per-backend default locator; domain_id 0 = the
    // unset sentinel (ROS convention).
    let baked_locator: Option<&str> = if !locator.is_null() {
        match core::ffi::CStr::from_ptr(locator).to_str() {
            Ok(s) => Some(s),
            Err(_) => return NROS_RET_INVALID_ARGUMENT,
        }
    } else {
        // Issue 0330 — NO backend default here. This layer is RMW-blind, and
        // the substitution that used to live here (`127.0.0.1:2019`) is
        // exactly why: it was an XRCE fact whose port had DRIFTED away from
        // the xrce backend's own `XRCE_DEFAULT_AGENT_PORT` (2018), and it was
        // applied to zenoh / dds / cyclonedds builds too. The locator stays
        // absent and the linked backend applies its own default
        // (`nros-rmw-xrce/session.c` for xrce, `nros_rmw_zenoh::DEFAULT_LOCATOR`
        // for zenoh; cyclonedds ignores the locator).
        None
    };
    let baked = nros_node::BootConfig {
        node_name: None,
        locator: baked_locator,
        // Issue #227 — NROS_DOMAIN_ID_EXPLICIT_ZERO (255) = explicit domain 0;
        // 0 stays the unset sentinel (model A / ROS convention); 233..=254
        // flow to the resolver's range check and fail loudly.
        domain_id: nros_node::baked_domain_from_c_abi(domain_id),
        namespace: None,
        // Issue 1050 defect (3) — an empty selector is "unset", not a backend
        // named "": the bake macro expands to a string literal, and an
        // unresolved cmake variable produces `""` rather than nothing.
        rmw: if rmw.is_null() {
            None
        } else {
            match core::ffi::CStr::from_ptr(rmw).to_str() {
                Ok("") => None,
                Ok(s) => Some(s),
                Err(_) => return NROS_RET_INVALID_ARGUMENT,
            }
        },
    };
    // issue 0687 — the environment rung comes from the hosted edge now. This
    // call passed `hosted_env: true` unconditionally and relied on the core's
    // `env` cfg to make it inert on a target build; the cfg is HERE now, where
    // the capability is, so a no-`env` C image says so instead of asking for a
    // rung that cannot exist.
    let resolved = match resolve_boot(baked) {
        Ok(cfg) => cfg,
        Err(_) => return NROS_RET_INVALID_ARGUMENT,
    };

    // Store the RESOLVED domain ID (fits u8: try_resolve enforces
    // DOMAIN_ID_MAX = 232).
    support.domain_id = resolved.domain_id as u8;

    // Store the RESOLVED locator.
    {
        let bytes = resolved.locator.as_bytes();
        let cap = support.locator.len() - 1;
        if bytes.len() > cap {
            return NROS_RET_INVALID_ARGUMENT;
        }
        support.locator[..bytes.len()].copy_from_slice(bytes);
        support.locator[bytes.len()] = 0;
        support.locator_len = bytes.len();
    }

    // Phase 128.C.2 — RMW-blind support init. Every linked backend
    // self-registers via the `RMW_INIT_ENTRIES` linker section
    // (linkme on the Rust side, `NROS_RMW_REGISTER_BACKEND` macro on
    // the C/C++ side). The walker fires inside the matching
    // `nros::internals::open_session` / `Executor::open` call below;
    // the prior `nros_app_register_backends()` weak/strong dance and
    // the legacy `cffi-xrce-c` hook are no longer needed.
    //
    // Phase 155.B.4 — Phase 128.C.2's "no longer needed" assumption
    // doesn't hold on RTOS targets `linkme` doesn't recognise:
    // FreeRTOS, NuttX, Zephyr, ESP-IDF all silently skip the
    // distributed-slice entry, so `walk_init_section` finds zero
    // backends and `get_vtable()` returns
    // `TransportError::InvalidArgument` (registry empty). The
    // sibling `nros_cpp_init` still calls
    // `nros_app_register_backends()` explicitly for this exact
    // reason. Mirror that here so the C-API path works on the
    // same RTOSes.
    #[cfg(feature = "rmw-cffi")]
    {
        unsafe extern "C" {
            fn nros_app_register_backends();
        }
        unsafe {
            nros_app_register_backends();
        }
    }

    // Initialize the middleware session
    #[cfg(feature = "rmw-cffi")]
    {
        use nros_node::SessionMode;

        let locator_str = core::str::from_utf8_unchecked(&support.locator[..support.locator_len]);

        // Derive session name. XRCE uses this to generate a unique session key —
        // two processes with the same name on the same agent will collide.
        let name: nros_core::heapless::String<32> = if !session_name.is_null() {
            // Use caller-provided name
            let name_cstr = core::ffi::CStr::from_ptr(session_name);
            nros_core::heapless::String::try_from(name_cstr.to_str().unwrap_or("nros"))
                .unwrap_or_else(|_| nros_core::heapless::String::try_from("nros").unwrap())
        } else {
            // Phase 266 (W5b) — null/empty name defaults to "node" (unified
            // compiled default across C, C++, and Rust, matching the Rust
            // resolver compiled default). The old `nros_{pid}` / "nros"
            // fallback is replaced for consistent `ros2 node list` output.
            nros_core::heapless::String::try_from("node").unwrap()
        };

        match nros::internals::open_session(
            locator_str,
            SessionMode::Client,
            support.domain_id as u32,
            &name,
            // Issue 1050 defect (3) — the RESOLVED selector (env > baked). The
            // borrow is of `resolved`, which outlives this call.
            resolved.rmw,
        ) {
            Ok(session) => {
                // Write session directly into inline opaque storage
                core::ptr::write(
                    support._opaque.as_mut_ptr() as *mut nros::internals::RmwSession,
                    session,
                );
                support.state = nros_support_state_t::NROS_SUPPORT_STATE_INITIALIZED;
                NROS_RET_OK
            }
            // Phase 155.B — surface the inner `TransportError` variant
            // as a specific `NROS_RET_*` code so a fresh
            // `nros_support_init -> -X` log line tells the user which
            // precondition the backend rejected, instead of every
            // failure mode collapsing to NROS_RET_ERROR (-1).
            Err(e) => transport_error_to_ret(e),
        }
    }

    #[cfg(not(feature = "rmw-cffi"))]
    {
        NROS_RET_ERROR
    }
}

/// Finalize the support context.
///
/// This function closes the middleware session and releases all resources.
///
/// # Parameters
/// * `support` - Pointer to an initialized support context
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if support is NULL
/// * `NROS_RET_NOT_INIT` if not initialized
///
/// # Safety
/// * `support` must be a valid pointer to an initialized nros_support_t
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rclc_support_fini(support: *mut nros_support_t) -> nros_ret_t {
    if support.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let support = &mut *support;

    if support.state != nros_support_state_t::NROS_SUPPORT_STATE_INITIALIZED {
        return NROS_RET_NOT_INIT;
    }

    // Drop the inline RMW session
    #[cfg(feature = "rmw-cffi")]
    {
        core::ptr::drop_in_place(support._opaque.as_mut_ptr() as *mut nros::internals::RmwSession);
    }

    support._opaque = [0u64; SESSION_OPAQUE_U64S];
    support.state = nros_support_state_t::NROS_SUPPORT_STATE_SHUTDOWN;

    NROS_RET_OK
}

/// Check if support context is valid (initialized).
///
/// # Parameters
/// * `support` - Pointer to a support context
///
/// # Returns
/// * `true` if valid, `false` if invalid or NULL
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_support_is_valid(support: *const nros_support_t) -> bool {
    if support.is_null() {
        return false;
    }

    let support = &*support;
    support.state == nros_support_state_t::NROS_SUPPORT_STATE_INITIALIZED
}

#[cfg(kani)]
mod verification {
    use super::*;
    use crate::error::*;

    #[kani::proof]
    #[kani::unwind(5)]
    fn support_init_null_ptr() {
        // NULL support pointer → INVALID_ARGUMENT
        let ret = unsafe { nros_support_init(core::ptr::null_mut(), core::ptr::null(), 0) };
        assert_eq!(ret, NROS_RET_INVALID_ARGUMENT);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn support_zero_initialized_state() {
        let support = nros_support_get_zero_initialized();
        assert_eq!(
            support.state,
            nros_support_state_t::NROS_SUPPORT_STATE_UNINITIALIZED
        );
        assert_eq!(support.domain_id, 0);
        assert!(support._opaque.iter().all(|&v| v == 0));
    }
}

impl nros_support_t {
    /// Get the raw session pointer (for executor initialization).
    pub(crate) fn get_session_ptr(&self) -> *mut nros::internals::RmwSession {
        self._opaque.as_ptr() as *mut nros::internals::RmwSession
    }

    /// Get the internal session reference (for internal use)
    pub(crate) unsafe fn get_session(&self) -> Option<&nros::internals::RmwSession> {
        if self.state != nros_support_state_t::NROS_SUPPORT_STATE_INITIALIZED {
            None
        } else {
            Some(&*(self._opaque.as_ptr() as *const nros::internals::RmwSession))
        }
    }

    /// Get the internal session reference mutably (for internal use)
    pub(crate) unsafe fn get_session_mut(&mut self) -> Option<&mut nros::internals::RmwSession> {
        if self.state != nros_support_state_t::NROS_SUPPORT_STATE_INITIALIZED {
            None
        } else {
            Some(&mut *(self._opaque.as_mut_ptr() as *mut nros::internals::RmwSession))
        }
    }

    /// Get the locator string
    pub(crate) fn get_locator(&self) -> &[u8] {
        &self.locator[..self.locator_len]
    }
}

/// Resolve boot config, with the environment rung when this build has the
/// `env` capability.
///
/// issue 0687 — `ExecutorConfig::try_resolve` used to take `hosted_env: bool`
/// and read the process environment inside the core crate. The core takes
/// values now, so the cfg that decides whether there IS an environment lives at
/// the edge that has one.
fn resolve_boot(
    baked: nros_node::BootConfig<'_>,
) -> Result<nros_node::ExecutorConfig<'_>, nros_node::BootConfigError> {
    #[cfg(feature = "env")]
    {
        nros::env::try_resolve_hosted(baked)
    }
    #[cfg(not(feature = "env"))]
    {
        nros_node::ExecutorConfig::try_resolve(baked)
    }
}
