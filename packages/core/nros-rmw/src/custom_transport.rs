//! Phase 115.A — runtime-pluggable custom transport vtable.
//!
//! Defines the platform-side hook that lets users plug a custom
//! transport (USB-CDC, BLE, RS-485, semihosting bridge, ring-buffer
//! loopback) at runtime without changing board crate, Cargo features,
//! or rebuilding.
//!
//! The shape mirrors micro-ROS's
//! `rmw_uros_set_custom_transport(framing, params, open, close, write, read)`
//! and the C ABI exposed by `nros-c` / `nros-cpp` as
//! `nros_transport_ops_t`.
//!
//! ## Why a fn-pointer vtable, not a Rust trait
//!
//! 1. **alloc-free.** A `Box<dyn CustomTransport>` would force the
//!    `alloc` crate on every no_std backend that wants to use the
//!    runtime hook. nano-ros's bare-metal / FreeRTOS / NuttX /
//!    ThreadX targets ship without a global allocator on the default
//!    feature flags, so `dyn` is a non-starter.
//! 2. **C ABI parity.** The user-facing surface is `nros_transport_ops_t`
//!    (a `#[repr(C)]` struct of fn pointers + a `void *`). A
//!    Rust-side fn-ptr vtable means the `set_custom_transport` C
//!    entry just memcpys the incoming struct into the static — no
//!    glue, no shims, no trampolines.
//! 3. **Matches XRCE's existing shape.** `uxr_set_custom_transport_callbacks`
//!    already takes 4 raw fn pointers; the Rust wrapper at
//!    `nros-rmw-xrce::init_transport` likewise. A trait would just
//!    be an extra layer that has to be type-erased into fn pointers
//!    anyway.
//!
//! See `docs/roadmap/phase-115-runtime-transport-vtable.md` § A.1
//! for the full discussion.
//!
//! ## Threading contract (v1)
//!
//! - `read` and `write` may NOT be called concurrently from
//!   different threads. The active backend serialises them through
//!   the `drive_io` / spin-once path. Custom transports written
//!   against this contract can use a single-buffer state machine
//!   without internal locking.
//! - Callbacks must NOT be invoked from interrupt context. Wrap
//!   ISR-driven hardware in a queue + `read` poller.
//! - `user_data` is opaque to the runtime — its `Send` / `Sync`
//!   discipline is the caller's responsibility. The vtable struct
//!   itself is `Send + Sync` because the four fn pointers always
//!   are.

use core::ffi::c_void;

use crate::sync::Mutex;

/// Phase 115.A.2 — current ABI version of [`NrosTransportOps`].
///
/// Embedded as the first field of the struct (see § *Versioning*
/// below). Consumers fill in this exact value before passing the
/// struct to [`set_custom_transport`]; runtime entry points reject
/// any other value with [`crate::TransportError::IncompatibleAbi`]
/// (or `NROS_RMW_RET_INCOMPATIBLE_ABI` at the C boundary).
///
/// The version bumps under two rules (per the portable-ABI design
/// note R5 in `docs/design/0006-portable-rmw-platform-interface.md`):
///
/// - **Major** (e.g. `V1` → `V2`): existing fields removed or
///   reordered. Old consumers fail cleanly via the version check.
/// - **Minor** (e.g. struct gains an appended fn pointer): version
///   stays the same. New consumers detect the new fn via the size
///   of the trailing `_reserved` region. Today there is none — v1 is
///   the inaugural version.
pub const NROS_TRANSPORT_OPS_ABI_VERSION_V1: u32 = 1;

/// Phase 115.A — runtime-pluggable custom transport. Caller fills in
/// the four fn pointers, hands the struct to [`set_custom_transport`],
/// and the active backend treats it as the read / write surface for
/// every wire frame.
///
/// `#[repr(C)]` so this is the SAME struct that
/// `nros_transport_ops_t` (C) and `nros::TransportOps` (C++) point at
/// — single layout, no parallel definitions to drift.
///
/// # Return-code conventions
///
/// `open` / `write` return `NROS_RMW_RET_OK` (== 0) on success and
/// a negative `nros_ret_t` (see `nros-rmw-cffi`) on failure. `read`
/// returns the non-negative byte count on success or a negative
/// `nros_ret_t` on error / timeout.
///
/// # Safety contract for the four fn pointers
///
/// - All callbacks receive `user_data` as their first argument. The
///   pointer is whatever the caller stored at registration time; the
///   runtime never dereferences it.
/// - `buf` / `len` describe a contiguous byte region the callback
///   may read (`write`) or write (`read`). The callback must NOT
///   retain pointers across the call.
/// - `params` (in `open`) is opaque per-transport metadata
///   threaded through from `set_custom_transport`. May be `NULL`.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct NrosTransportOps {
    /// Phase 115.A.2 — ABI version. Consumers MUST fill in
    /// [`NROS_TRANSPORT_OPS_ABI_VERSION_V1`]; mismatched values are
    /// rejected at registration time with
    /// `TransportError::IncompatibleAbi` (`NROS_RMW_RET_INCOMPATIBLE_ABI`
    /// at the C boundary). Reserved for future minor-version
    /// detection — see the const's doc-comment.
    pub abi_version: u32,
    /// Phase 115.A.2 — reserved padding to keep the struct
    /// alignment-stable across appends. Must be zero.
    pub _reserved: u32,
    /// Opaque caller context, threaded back into every callback as
    /// the first argument. Lifetime: must outlive the transport's
    /// active period (i.e. until `close` returns).
    pub user_data: *mut c_void,
    /// Open the underlying medium. `params` is opaque per-transport
    /// metadata (e.g. UART baud rate, USB-CDC endpoint id) supplied
    /// at registration time.
    pub open: unsafe extern "C" fn(user_data: *mut c_void, params: *const c_void) -> i32,
    /// Tear the transport down. Complement of `open`. After `close`
    /// returns, the runtime will not invoke `read` or `write` on this
    /// transport unless `set_custom_transport` is called again.
    pub close: unsafe extern "C" fn(user_data: *mut c_void),
    /// Send `len` bytes from `buf`. Returns 0 on success, negative
    /// `nros_ret_t` on failure. Must NOT block beyond a brief
    /// hardware retry; long blocking should surface as
    /// `NROS_RMW_RET_TIMEOUT` (-2).
    pub write: unsafe extern "C" fn(user_data: *mut c_void, buf: *const u8, len: usize) -> i32,
    /// Receive up to `len` bytes into `buf` within `timeout_ms`.
    /// Returns the non-negative byte count on success (may be less
    /// than `len`), or a negative `nros_ret_t` on error / timeout.
    pub read: unsafe extern "C" fn(
        user_data: *mut c_void,
        buf: *mut u8,
        len: usize,
        timeout_ms: u32,
    ) -> i32,
}

// SAFETY: the struct is just four fn pointers + a *mut. Send / Sync
// are sound because (a) fn pointers are always Send+Sync, and (b) the
// caller owns synchronisation of `user_data` per the threading contract
// (no concurrent read/write, no ISR invocation). Cross-thread
// observability of the registered struct is guarded by the surrounding
// Mutex, not by any property of this struct.
unsafe impl Send for NrosTransportOps {}
unsafe impl Sync for NrosTransportOps {}

/// Phase 115.A — single-slot storage for the registered transport.
///
/// `set_custom_transport` writes the struct in; backends read it via
/// [`take_custom_transport`] (during `Rmw::open`) or
/// [`peek_custom_transport`] (during steady-state for liveliness /
/// reconnect logic). One transport per process; a second
/// `set_custom_transport` call before `take` overwrites the slot
/// (documented as "register early, register once").
static SLOT: Mutex<Option<NrosTransportOps>> = Mutex::new(None);

/// Phase 115.A — register a custom transport vtable. Must be called
/// **before** the first `Rmw::open` (or
/// `nros_support_init` from the C surface). v1 leaves enforcement of
/// "before init" to backend code, and NO backend enforces it: a call
/// after `Rmw::open` is implementation-defined, which is also what the
/// C entry point `nros_set_custom_transport` documents. This line
/// used to say backends "reject re-registration with
/// `ALREADY_INIT`" — `rmw_ret.h` defines no such code
/// (issue 1126), so the sentence promised both a behaviour and a
/// constant that do not exist.
///
/// Pass `None` to clear a previously-registered vtable (e.g. for
/// teardown in tests).
///
/// Returns `Err(TransportError::IncompatibleAbi)` when `ops.is_some()`
/// but `abi_version != NROS_TRANSPORT_OPS_ABI_VERSION_V1`. C / C++
/// wrappers map this to `NROS_RMW_RET_INCOMPATIBLE_ABI`.
///
/// # Safety
///
/// The four fn pointers in `ops` must follow the threading contract
/// documented on [`NrosTransportOps`] — no concurrent read/write, no
/// ISR invocation, `user_data` outlives the transport's active period.
pub unsafe fn set_custom_transport(
    ops: Option<NrosTransportOps>,
) -> Result<(), crate::TransportError> {
    if let Some(o) = ops.as_ref()
        && o.abi_version != NROS_TRANSPORT_OPS_ABI_VERSION_V1
    {
        return Err(crate::TransportError::IncompatibleAbi);
    }
    SLOT.with(|slot| *slot = ops);
    Ok(())
}

/// Phase 115.A — peek at the currently-registered transport without
/// consuming it. Used by backends that need to re-attach on session
/// reconnect, or by tests that want to verify a registration landed.
/// Returns `None` if nothing was registered.
pub fn peek_custom_transport() -> Option<NrosTransportOps> {
    SLOT.with(|slot| *slot)
}

/// Phase 115.A — drain the registered transport. Returns the
/// previously-registered vtable (`None` if nothing was registered)
/// and clears the slot. Backends call this from `Rmw::open` when
/// `platform-custom` is the active platform; the vtable then lives
/// inside the session for the rest of the process lifetime.
pub fn take_custom_transport() -> Option<NrosTransportOps> {
    SLOT.with(|slot| slot.take())
}

#[cfg(test)]
mod tests {
    use super::*;

    unsafe extern "C" fn stub_open(_ud: *mut c_void, _params: *const c_void) -> i32 {
        0
    }
    unsafe extern "C" fn stub_close(_ud: *mut c_void) {}
    unsafe extern "C" fn stub_write(_ud: *mut c_void, _buf: *const u8, _len: usize) -> i32 {
        0
    }
    unsafe extern "C" fn stub_read(
        _ud: *mut c_void,
        _buf: *mut u8,
        _len: usize,
        _timeout_ms: u32,
    ) -> i32 {
        0
    }

    fn make_ops() -> NrosTransportOps {
        NrosTransportOps {
            abi_version: NROS_TRANSPORT_OPS_ABI_VERSION_V1,
            _reserved: 0,
            user_data: core::ptr::null_mut(),
            open: stub_open,
            close: stub_close,
            write: stub_write,
            read: stub_read,
        }
    }

    /// Lifecycle: register, peek, take, peek-after-take.
    #[test]
    fn lifecycle() {
        // Drain anything a previous test left behind so this test
        // is order-independent under shared SLOT.
        let _ = take_custom_transport();

        assert!(peek_custom_transport().is_none());

        unsafe {
            set_custom_transport(Some(make_ops())).expect("set");
        }

        let peeked = peek_custom_transport().expect("peek after set");
        assert!(peeked.user_data.is_null());
        assert_eq!(peeked.abi_version, NROS_TRANSPORT_OPS_ABI_VERSION_V1);

        // Peek again — slot still occupied.
        assert!(peek_custom_transport().is_some());

        let taken = take_custom_transport().expect("take");
        assert!(taken.user_data.is_null());

        // Slot is now empty.
        assert!(peek_custom_transport().is_none());
        assert!(take_custom_transport().is_none());
    }

    /// `set_custom_transport(None)` clears the slot.
    #[test]
    fn explicit_clear() {
        let _ = take_custom_transport();
        unsafe {
            set_custom_transport(Some(make_ops())).expect("set");
            set_custom_transport(None).expect("clear");
        }
        assert!(peek_custom_transport().is_none());
    }

    /// Phase 115.A.2 — abi_version mismatch is rejected with
    /// `TransportError::IncompatibleAbi`. Slot stays whatever it was
    /// before the bad call.
    #[test]
    fn rejects_unknown_abi_version() {
        let _ = take_custom_transport();
        let mut ops = make_ops();
        ops.abi_version = 0xBAD0_BAD0; // not V1.
        let err = unsafe { set_custom_transport(Some(ops)) };
        assert!(matches!(err, Err(crate::TransportError::IncompatibleAbi)));
        // Bad call did NOT install — slot stays empty.
        assert!(peek_custom_transport().is_none());
    }

    /// Struct stays `Copy + Send + Sync` — the static-slot pattern
    /// relies on these bounds at compile time.
    #[test]
    fn ops_is_copy_send_sync() {
        fn assert_copy_send_sync<T: Copy + Send + Sync>() {}
        assert_copy_send_sync::<NrosTransportOps>();
    }
}
