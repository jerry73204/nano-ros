//! nros-rmw-zenoh: Zenoh-pico RMW backend for nros
//!
//! This crate provides the zenoh-pico transport implementation,
//! combining the safe Rust API over zenoh-pico FFI with the
//! transport layer that implements nros-rmw traits.
//!
//! # Platform Backends
//!
//! Select one backend via feature flags:
//! - `platform-posix` - Uses POSIX threads, for desktop testing
//! - `platform-zephyr` - Uses Zephyr RTOS threads
//! - `platform-bare-metal` - Uses polling (bare-metal platforms)
//! - `platform-freertos` - Uses FreeRTOS threads + lwIP sockets
//! - `platform-threadx` - Uses ThreadX threads + NetX Duo sockets

#![no_std]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

pub(crate) mod config;
pub mod keyexpr;
pub mod zpico;

/// Issue 0330 — **the** default zenoh router endpoint, owned by the zenoh
/// backend.
///
/// This is the single source of truth for the value. RMW-agnostic layers
/// (`nros-node`'s boot-config resolver, `nros`'s env ladder, the C
/// `nros_support_init` edge, the C++ `nros::init` header, the
/// `NROS_ENTRY_LOCATOR` macro) must NOT restate it: they hand the backend an
/// *absent* locator (`None`, or the empty string — see
/// [`shim::session::effective_client_locator`]) and the backend substitutes
/// this const. That keeps a cyclonedds- or xrce-only build free of a zenoh
/// fact it never uses, and leaves exactly one place to change the value.
pub const DEFAULT_LOCATOR: &str = "tcp/127.0.0.1:7447";

pub mod shim;

// Re-export zpico types (always available)
pub use zpico::{ZenohId, ZpicoError};

// Phase 214.G — link-graph anchor for POSIX.
//
// `zpico-sys`'s C alias TU (`c/zpico/platform_aliases.c`) forwards
// every `_z_*` zenoh-pico symbol to the canonical `nros_platform_*`
// ABI. On POSIX hosts, those symbols live in the C library compiled
// from `nros-platform-cffi`'s `posix-c-port` feature (forwarded by
// our `platform-posix` feature → `nros-platform/platform-posix` →
// `nros-platform-cffi/posix-c-port`).
//
// `nros-platform`'s `lib.rs:81` provides `__FORCE_LINK_CFFI` as a
// `#[used] pub static` so `rust-lld` is forced to pull the
// `nros-platform-cffi` rlib (and its `libnros_platform_posix.a`
// native lib) into the final binary. The downstream contract is
// that any consumer of `nros-platform/platform-posix` that needs
// those symbols re-anchors the `#[used]` chain locally. Without
// this re-anchor, `nros-rmw-zenoh` test binaries (which don't
// reference any `nros_platform` Rust symbol — every callsite goes
// through the C ABI inside `zpico-sys`) leave the cffi rlib
// untouched and `rust-lld` errors with `undefined symbol:
// nros_platform_mutex_*`. See Track G in
// `docs/roadmap/phase-214-antipattern-audit-findings.md`.
//
// Phase 227.3(B) — `test` added to the gate: the Rust shim is now
// platform-agnostic and compiles into the `--lib` unit-test binary
// unconditionally, so that binary references the zpico C-port symbols
// and must link the posix C provider. The `[dev-dependencies]` entry
// pins `nros-platform` to `platform-posix`, so `__FORCE_LINK_CFFI`
// exists under `cfg(test)` and re-anchors the cffi rlib for the test
// binary even when this crate's own `platform-posix` feature is off.
#[cfg(any(feature = "platform-posix", test))]
#[doc(hidden)]
#[used]
pub static __FORCE_LINK_PLATFORM_CFFI: extern "C" fn() = nros_platform::__FORCE_LINK_CFFI;

// Re-export platform-gated zpico types
pub use zpico::{
    Context, LivelinessToken, Publisher as ZpicoPublisher, Queryable, Subscriber as ZpicoSubscriber,
};

// Re-export shim types when platform feature is enabled
pub use shim::{
    MessageInfo, RMW_GID_SIZE, RmwAttachment, Ros2Liveliness, SERVICE_BUFFER_SIZE,
    SUBSCRIBER_BUFFER_SIZE, ZenohPublisher, ZenohRmw, ZenohServiceClient, ZenohServiceServer,
    ZenohSession, ZenohSubscriber, ZenohTransport, effective_client_locator, normalize_locator,
    overflow_drops_total,
};

// Re-export std-only executor wake functions
#[cfg(feature = "std")]
pub use shim::{signal_executor_wake, wait_for_executor_wake};

// Re-export extension traits
pub use keyexpr::{QosKeyExpr, ServiceKeyExpr, TopicKeyExpr};

// Re-export safety types when feature is enabled
#[cfg(feature = "safety-e2e")]
pub use nros_rmw::{IntegrityStatus, SafetyValidator, crc32};

// ============================================================================
// Phase 115.M.3 — C-vtable register entry (folded in from the
// retired `nros-rmw-zenoh-cffi` crate).
// ============================================================================
//
// The vtable IS the cross-language boundary. Once registered, runtime
// dispatch goes Rust→vtable→… directly; backends never `use` each
// other's trait surface. So the register fn lives next to the trait
// impl, and the legacy `*-cffi` two-crate split goes away.

/// Phase 403 W4 — the slots zenoh-pico answers that the generic
/// `RustBackendAdapter` cannot.
///
/// Unconditional, unlike [`loan_trampolines`]: before W4 the only zenoh-owned
/// vtable was the `lending` one, so a build without that feature registered the
/// generic adapter table verbatim and had nowhere to put a backend-specific
/// slot. `ZENOH_VTABLE_BASE` is that place, and both register paths go through
/// it.
///
/// Only zenoh gets `required_rx_bytes`. Putting it on `RustBackend` as a
/// defaulted trait method would hand every Rust backend a non-NULL slot whose
/// body is "no opinion" — the relocation `rmw_vtable.h` argues against for
/// making the slot mandatory, arrived at from the other direction. cyclonedds
/// and XRCE have one receive buffer and nothing to say beyond the hint, so they
/// stay NULL and the runtime's NULL-slot fallback is their (correct) answer.
mod rx_sizing_trampolines {
    use nros_rmw_cffi::{
        NROS_RMW_RET_INVALID_ARGUMENT, NROS_RMW_RET_OK, NROS_RMW_RET_UNSUPPORTED, NrosRmwRet,
        NrosRmwVtable, RustBackendAdapter,
    };

    use crate::ZenohRmw;

    /// `nros_rmw_vtable_t::required_rx_bytes` for zenoh-pico.
    ///
    /// `type_name` / `type_hash` are unread, and that is the finding rather
    /// than an omission: across this ABI a type is a STRING and the backend
    /// holds no schema, so the only thing zenoh-pico can size from is the
    /// `hint` the runtime computed from the type's own bound. A backend that
    /// carried a type registry would use them; this one has nothing to look up.
    ///
    /// The arithmetic, and why the answer is minimal rather than the class
    /// stride, is [`crate::shim::required_rx_bytes`].
    unsafe extern "C" fn zenoh_required_rx_bytes(
        _type_name: *const core::ffi::c_char,
        _type_hash: *const core::ffi::c_char,
        hint: usize,
        out_bytes: *mut usize,
    ) -> NrosRmwRet {
        if out_bytes.is_null() {
            return NROS_RMW_RET_INVALID_ARGUMENT;
        }
        match crate::shim::required_rx_bytes(hint) {
            Some(bytes) => {
                // SAFETY: checked non-NULL above; the caller owns a `size_t`.
                unsafe { *out_bytes = bytes };
                NROS_RMW_RET_OK
            }
            // Per-type inability, which NULLing the whole slot cannot express:
            // no size class in this image can hold a sample this big, so there
            // is no take-buffer length that makes the subscription work.
            // `*out_bytes` is left untouched, and the runtime falls back to the
            // hint exactly as for a NULL slot.
            None => NROS_RMW_RET_UNSUPPORTED,
        }
    }

    /// The generic adapter table plus the slots zenoh-pico fills itself.
    /// A `const`, not a `static`, because a `static` initializer may not read
    /// another `static` and the `lending` table spreads this one.
    pub(super) const ZENOH_VTABLE_BASE: NrosRmwVtable = NrosRmwVtable {
        required_rx_bytes: Some(zenoh_required_rx_bytes),
        ..RustBackendAdapter::<ZenohRmw>::VTABLE
    };

    /// The table a build without `lending` registers.
    #[cfg(not(feature = "lending"))]
    pub(super) static ZENOH_VTABLE: NrosRmwVtable = ZENOH_VTABLE_BASE;
}

mod cffi_register {
    use core::ffi::c_int;

    use nros_rmw_cffi::{NROS_RMW_RET_OK, NrosRmwRet};

    /// C entry — installs the zenoh-pico vtable into the cffi
    /// runtime under the canonical name `"zenoh"`. Returns
    /// `NROS_RMW_RET_OK` (0) on success. Idempotent — duplicate
    /// `("zenoh", vtable)` registrations are in-place overwrites.
    ///
    /// Phase 124.A.4.b — when the `lending` feature is on, install
    /// a vtable that overrides `pub_loan/_commit/_discard` with
    /// zenoh-pico-specific trampolines (zero-copy aliased publish).
    /// Without `lending`, the loan slots stay NULL and the runtime
    /// arena fallback applies.
    ///
    /// Phase 403 W4 — neither path registers the generic adapter table
    /// verbatim any more. Both go through `rx_sizing_trampolines`, which
    /// fills `required_rx_bytes`; that slot is answerable on every zenoh
    /// build, `lending` or not.
    #[cfg(not(feature = "lending"))]
    #[unsafe(no_mangle)]
    pub extern "C" fn nros_rmw_zenoh_register() -> NrosRmwRet {
        unsafe {
            nros_rmw_cffi::nros_rmw_cffi_register_named(
                c"zenoh".as_ptr(),
                &super::rx_sizing_trampolines::ZENOH_VTABLE,
            )
        }
    }

    #[cfg(feature = "lending")]
    #[unsafe(no_mangle)]
    pub extern "C" fn nros_rmw_zenoh_register() -> NrosRmwRet {
        unsafe {
            nros_rmw_cffi::nros_rmw_cffi_register_named(
                c"zenoh".as_ptr(),
                &super::loan_trampolines::ZENOH_VTABLE,
            )
        }
    }

    /// Failure mode for the safe Rust wrapper.
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    pub struct RegisterError(pub c_int);

    /// Safe Rust wrapper around [`nros_rmw_zenoh_register`]. Returns
    /// `Err(RegisterError(rc))` when the runtime rejects the vtable.
    pub fn register() -> Result<(), RegisterError> {
        let rc = nros_rmw_zenoh_register();
        if rc == NROS_RMW_RET_OK {
            Ok(())
        } else {
            Err(RegisterError(rc))
        }
    }

    // Phase 249 P4b — hosted self-registration via the
    // `nros_rmw_register_backend!` macro. The macro expands to a
    // `#[used]` `.init_array` ctor on `not(target_os = "none")` and to
    // nothing on `target_os = "none"` (bare-metal, and the RTOS ports
    // that build against a bare-metal triple: Zephyr, FreeRTOS,
    // ThreadX, ESP32-C3). Issue 1028 — NuttX is NOT in that second set:
    // `armv7a-nuttx-eabihf` reports `target_os = "nuttx"`, so a NuttX
    // image DOES get the ctor. That is deliberate and stated in
    // `nros_rmw_cffi::section` — the board still calls `register()`
    // explicitly and `register()` is idempotent — but this comment used
    // to claim NuttX had `target_os = "none"`, which is the same false
    // premise 1028 was filed against.
    nros_rmw_cffi::nros_rmw_register_backend! {
        fn() {
            let _ = nros_rmw_zenoh_register();
        }
    }
}

pub use cffi_register::{RegisterError, nros_rmw_zenoh_register, register};

// ============================================================================
// Phase 124.A.4.b — zenoh-pico cffi loan trampolines
// ============================================================================
//
// When the `lending` feature is on, the cffi register installs a
// vtable whose `pub_loan/_commit/_discard` slots call into
// `ZenohPublisher`'s native single-slot arena + aliased-publish path
// (Phase 99.F). C/C++ callers get the same zero-copy semantics Rust
// callers have through the `SlotLending` trait — no staging-buffer
// memcpy in the cffi fallback.
//
// Storage discipline (mirrors `RustBackendAdapter`):
//   - `NrosRmwPublisher::backend_data` was set by `create_publisher`
//     to `Box::into_raw(Box<ZenohPublisher>)`. Trampolines cast back
//     to `&ZenohPublisher`.
//   - Issue 0812 — the loan token is the loan's LENGTH, tagged, NOT a
//     heap pointer. It used to be `Box::into_raw` of a lifetime-erased
//     `ZenohSlot<'static>`, which put a malloc + free on the zero-copy
//     publish path (and made the whole `lending` surface depend on a
//     global allocator). The arena is single-slot per publisher and the
//     publisher comes back as a parameter on both commit and discard, so
//     the length is the only thing the token has to carry;
//     `ZenohSlot::from_outstanding_loan` rebuilds the slot from it.
#[cfg(feature = "lending")]
mod loan_trampolines {

    use nros_rmw::SlotLending;
    use nros_rmw_cffi::{
        NROS_RMW_RET_ERROR, NROS_RMW_RET_OK, NROS_RMW_RET_WOULD_BLOCK, NrosRmwPublisher,
        NrosRmwRet, NrosRmwVtable,
    };

    use crate::{ZenohRmw, shim::publisher::ZenohSlot};

    type ZenohPublisher =
        <<ZenohRmw as nros_rmw::Rmw>::Session as nros_rmw::Session>::PublisherHandle;

    /// Top bit of the loan token. Two jobs: it keeps the token non-NULL
    /// for a zero-length loan (the cffi runtime rejects a NULL token), and
    /// it makes a token that came from anywhere else — a stale pointer, a
    /// token minted by a different backend — fail the decode instead of
    /// being read as a length.
    const TOKEN_TAG: usize = 1usize << (usize::BITS - 1);

    /// Encode a granted loan length as the opaque token the cffi runtime
    /// hands back at commit / discard. Never dereferenced — phase-406 W3
    /// gave it a type so it cannot be crossed with another backend's token
    /// or with a message pointer, but it is still a tagged integer.
    fn encode_token(len: usize) -> *mut nros_rmw_cffi::generated::rmw_loan_token_t {
        (len | TOKEN_TAG) as *mut nros_rmw_cffi::generated::rmw_loan_token_t
    }

    /// Recover the granted length, or `None` when `token` did not come
    /// from [`encode_token`].
    fn decode_token(token: *mut nros_rmw_cffi::generated::rmw_loan_token_t) -> Option<usize> {
        let raw = token as usize;
        if (raw & TOKEN_TAG) == 0 {
            return None;
        }
        Some(raw & !TOKEN_TAG)
    }

    /// Resolve the `ZenohPublisher` behind a vtable publisher view.
    ///
    /// # Safety
    /// `publisher`, when non-NULL, must point at a live `NrosRmwPublisher`
    /// whose `backend_data` this backend created.
    unsafe fn publisher_handle<'a>(
        publisher: *const NrosRmwPublisher,
    ) -> Option<&'a ZenohPublisher> {
        if publisher.is_null() {
            return None;
        }
        let backend_data = unsafe { (*publisher).backend_data };
        if backend_data.is_null() {
            return None;
        }
        Some(unsafe { &*(backend_data as *const ZenohPublisher) })
    }

    unsafe extern "C" fn zenoh_pub_loan(
        publisher: *const NrosRmwPublisher,
        requested_len: usize,
        out_slot: *mut nros_rmw_cffi::generated::rmw_mut_byte_span_t,
        out_token: *mut *mut nros_rmw_cffi::generated::rmw_loan_token_t,
    ) -> NrosRmwRet {
        if out_slot.is_null() || out_token.is_null() || requested_len == 0 {
            return nros_rmw_cffi::NROS_RMW_RET_INVALID_ARGUMENT;
        }
        // SAFETY: cffi-runtime contract — the view describes a live
        // publisher this backend created.
        let Some(pub_handle) = (unsafe { publisher_handle(publisher) }) else {
            return nros_rmw_cffi::NROS_RMW_RET_INVALID_ARGUMENT;
        };
        match pub_handle.try_lend_slot(requested_len) {
            Ok(Some(mut slot)) => {
                let buf_ptr = slot.as_mut().as_mut_ptr();
                let cap = slot.as_mut().len();
                // Issue 0812 — no box. Forgetting the slot is what keeps
                // the loan OUTSTANDING (dropping it would release the
                // arena immediately); `zenoh_pub_commit` /
                // `zenoh_pub_discard` rebuild it from `cap`, which is the
                // whole content of the token.
                core::mem::forget(slot);
                unsafe {
                    (*out_slot).data = buf_ptr;
                    (*out_slot).capacity = cap;
                    (*out_slot).len = 0;
                    *out_token = encode_token(cap);
                }
                NROS_RMW_RET_OK
            }
            Ok(None) => NROS_RMW_RET_WOULD_BLOCK,
            Err(_) => NROS_RMW_RET_ERROR,
        }
    }

    unsafe extern "C" fn zenoh_pub_commit(
        publisher: *const NrosRmwPublisher,
        token: *mut nros_rmw_cffi::generated::rmw_loan_token_t,
        actual_len: usize,
    ) -> NrosRmwRet {
        let Some(len) = decode_token(token) else {
            // A NULL or foreign token is a caller bug, not a no-op: the
            // loan it names was never granted here.
            return nros_rmw_cffi::NROS_RMW_RET_INVALID_ARGUMENT;
        };
        // SAFETY: cffi-runtime contract — the view describes a live
        // publisher this backend created.
        let Some(pub_handle) = (unsafe { publisher_handle(publisher) }) else {
            return nros_rmw_cffi::NROS_RMW_RET_INVALID_ARGUMENT;
        };
        // SAFETY: the runtime consumes a token exactly once, so the loan
        // `token` names is still outstanding and this is its only holder.
        let mut slot = unsafe { ZenohSlot::from_outstanding_loan(pub_handle, len) };
        slot.truncate(actual_len);
        match pub_handle.commit_slot(slot) {
            Ok(()) => NROS_RMW_RET_OK,
            Err(_) => NROS_RMW_RET_ERROR,
        }
    }

    unsafe extern "C" fn zenoh_pub_discard(
        publisher: *const NrosRmwPublisher,
        token: *mut nros_rmw_cffi::generated::rmw_loan_token_t,
    ) -> NrosRmwRet {
        let Some(len) = decode_token(token) else {
            // A NULL or foreign token is a caller bug, not a no-op: the
            // loan it names was never granted here.
            return nros_rmw_cffi::NROS_RMW_RET_INVALID_ARGUMENT;
        };
        // The publisher is load-bearing now, where the boxed token made it
        // ignorable: releasing the arena is the publisher's own operation.
        // SAFETY: cffi-runtime contract, as above.
        let Some(pub_handle) = (unsafe { publisher_handle(publisher) }) else {
            return nros_rmw_cffi::NROS_RMW_RET_INVALID_ARGUMENT;
        };
        // SAFETY: as in commit. Dropping the rebuilt slot is what releases
        // the arena — `ZenohSlot::drop` does it.
        drop(unsafe { ZenohSlot::from_outstanding_loan(pub_handle, len) });
        NROS_RMW_RET_OK
    }

    /// Customised zenoh vtable: base = generic `RustBackendAdapter`
    /// trampolines for all standard slots; loan slots overridden to
    /// route through zenoh-pico's aliased-publish path.
    /// Phase 403 W4 — the base is `rx_sizing_trampolines::ZENOH_VTABLE_BASE`,
    /// not the bare adapter table, so a `lending` build answers
    /// `required_rx_bytes` too. Spreading the adapter table here instead would
    /// have made the slot's presence depend on a publisher-side feature.
    pub(super) static ZENOH_VTABLE: NrosRmwVtable = NrosRmwVtable {
        borrow_loaned_message: Some(zenoh_pub_loan),
        publish_loaned_message: Some(zenoh_pub_commit),
        return_loaned_message_from_publisher: Some(zenoh_pub_discard),
        ..super::rx_sizing_trampolines::ZENOH_VTABLE_BASE
    };
}
