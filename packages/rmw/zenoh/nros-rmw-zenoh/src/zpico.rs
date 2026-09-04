//! High-level Rust API for zenoh-pico
//!
//! This module provides a safe Rust wrapper around the zenoh-pico C shim,
//! enabling embedded applications to use zenoh for communication.

use core::{ffi::c_void, marker::PhantomData};

// ============================================================================
// FFI Reentrancy Guard
// ============================================================================

/// Execute a closure with FFI reentrancy protection.
///
/// When the `ffi-sync` feature is enabled, wraps the closure in
/// `critical_section::with()` to prevent concurrent access to zpico global
/// state from mixed-priority tasks or ISRs.
///
/// When the feature is disabled, this is a zero-cost passthrough.
#[allow(dead_code)] // used only when a platform feature is enabled
#[inline(always)]
pub(crate) fn ffi_guard<R>(f: impl FnOnce() -> R) -> R {
    #[cfg(feature = "ffi-sync")]
    {
        return critical_section::with(|_cs| f());
    }
    #[cfg(not(feature = "ffi-sync"))]
    f()
}

// Re-export FFI types and constants from sys crate
pub use zpico_sys::{
    ZPICO_ERR_CONFIG, ZPICO_ERR_FULL, ZPICO_ERR_GENERIC, ZPICO_ERR_INVALID, ZPICO_ERR_KEYEXPR,
    ZPICO_ERR_PUBLISH, ZPICO_ERR_SESSION, ZPICO_ERR_TASK, ZPICO_ERR_TIMEOUT, ZPICO_MAX_LIVELINESS,
    ZPICO_MAX_PENDING_GETS, ZPICO_MAX_PUBLISHERS, ZPICO_MAX_QUERYABLES, ZPICO_MAX_SESSIONS,
    ZPICO_MAX_SUBSCRIBERS, ZPICO_OK, ZPICO_PEER_MODE_SUPPORTED, ZPICO_QUERYABLE_TABLE_DECLARED,
    ZPICO_RMW_GID_SIZE, ZPICO_ZID_SIZE, ZpicoCallback, ZpicoCallbackWithAttachment,
    ZpicoNotifyCallback, ZpicoQueryCallback, zpico_property_t, zpico_ring_desc_t,
};

// Import FFI functions from sys crate
use zpico_sys::{
    zpico_close, zpico_declare_liveliness, zpico_declare_publisher_ex, zpico_declare_queryable,
    zpico_declare_subscriber, zpico_declare_subscriber_direct_write, zpico_declare_subscriber_ring,
    zpico_declare_subscriber_with_attachment, zpico_get_zid, zpico_init, zpico_init_with_config,
    zpico_is_open, zpico_open, zpico_publish, zpico_publish_with_attachment,
    zpico_publish_with_attachment_aliased, zpico_query_reply, zpico_queryable_take_reply_seq,
    zpico_session_acquire, zpico_session_release, zpico_session_t, zpico_spin_once,
    zpico_undeclare_liveliness, zpico_undeclare_publisher, zpico_undeclare_queryable,
    zpico_undeclare_subscriber, zpico_uses_polling,
};

// ============================================================================
// Error Types
// ============================================================================

/// Error type for shim operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZpicoError {
    /// Generic error
    Generic,
    /// Configuration error
    Config,
    /// Session error
    Session,
    /// Task creation error
    Task,
    /// Invalid key expression
    KeyExpr,
    /// Resource limit reached
    Full,
    /// Invalid handle
    Invalid,
    /// Publish error
    Publish,
    /// Session not open
    NotOpen,
    /// Query timeout (no reply received)
    Timeout,
}

impl ZpicoError {
    fn from_code(code: i32) -> Self {
        match code {
            -1 => ZpicoError::Generic,
            -2 => ZpicoError::Config,
            -3 => ZpicoError::Session,
            -4 => ZpicoError::Task,
            -5 => ZpicoError::KeyExpr,
            -6 => ZpicoError::Full,
            -7 => ZpicoError::Invalid,
            -8 => ZpicoError::Publish,
            // ZPICO_ERR_TIMEOUT. Added to zpico.h without extending this
            // mirror, so -9 fell into the Generic catch-all — an idle
            // spin_once (the ThreadX poll branch's normal quiet tick)
            // became ConnectionFailed and fed the executor's session-death
            // counter (issue 0387: quiet C talkers died at
            // SPIN_ERROR_TOLERANCE × period).
            -9 => ZpicoError::Timeout,
            _ => ZpicoError::Generic,
        }
    }
}

impl core::fmt::Display for ZpicoError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ZpicoError::Generic => write!(f, "generic error"),
            ZpicoError::Config => write!(f, "configuration error"),
            ZpicoError::Session => write!(f, "session error"),
            ZpicoError::Task => write!(f, "task creation error"),
            ZpicoError::KeyExpr => write!(f, "invalid key expression"),
            ZpicoError::Full => write!(f, "resource limit reached"),
            ZpicoError::Invalid => write!(f, "invalid handle"),
            ZpicoError::Publish => write!(f, "publish error"),
            ZpicoError::NotOpen => write!(f, "session not open"),
            ZpicoError::Timeout => write!(f, "query timeout"),
        }
    }
}

/// Result type for shim operations
pub type Result<T> = core::result::Result<T, ZpicoError>;

// ============================================================================
// ZenohId
// ============================================================================

/// A 16-byte Zenoh session ID
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ZenohId {
    /// The raw 16-byte ID
    pub id: [u8; 16],
}

impl ZenohId {
    /// Create a new ZenohId from raw bytes
    pub fn from_bytes(bytes: [u8; 16]) -> Self {
        Self { id: bytes }
    }

    // `to_hex_string()` removed — was dead code requiring `alloc`.
    // Use `to_hex_bytes()` instead (alloc-free, writes into caller-provided buffer).

    /// Format the ID into a fixed-size buffer (for no_std)
    ///
    /// Returns the number of bytes written (always 32).
    pub fn to_hex_bytes(&self, buf: &mut [u8; 32]) {
        const HEX_CHARS: &[u8; 16] = b"0123456789abcdef";
        // LSB-first order
        for i in 0..16 {
            let byte = self.id[15 - i];
            buf[i * 2] = HEX_CHARS[(byte >> 4) as usize];
            buf[i * 2 + 1] = HEX_CHARS[(byte & 0xf) as usize];
        }
    }
}

// ============================================================================
// LivelinessToken
// ============================================================================

/// A liveliness token for ROS 2 discovery
///
/// When a liveliness token is declared, subscribers on intersecting key expressions
/// will receive a PUT sample when connectivity is achieved, and a DELETE sample
/// if it's lost.
///
/// Liveliness tokens are automatically undeclared when dropped.
///
/// Note: The C shim manages tokens via static storage with integer handles,
/// so the token does not need a lifetime parameter.
pub struct LivelinessToken {
    session: *mut zpico_session_t,
    handle: i32,
}

impl LivelinessToken {
    /// Get the liveliness handle
    pub fn handle(&self) -> i32 {
        self.handle
    }
}

impl Drop for LivelinessToken {
    fn drop(&mut self) {
        ffi_guard(|| unsafe {
            zpico_undeclare_liveliness(self.session, self.handle);
        });
    }
}

// ============================================================================
// Queryable
// ============================================================================

/// A queryable for receiving service requests
///
/// Queryables receive queries and can send replies. This is used to implement
/// ROS 2 service servers.
///
/// Note: The C shim manages queryables via static storage with integer handles,
/// so the queryable does not need a lifetime parameter.
pub struct Queryable {
    session: *mut zpico_session_t,
    handle: i32,
}

impl Queryable {
    /// Get the queryable handle
    pub fn handle(&self) -> i32 {
        self.handle
    }
}

impl Drop for Queryable {
    fn drop(&mut self) {
        ffi_guard(|| unsafe {
            zpico_undeclare_queryable(self.session, self.handle);
        });
    }
}

// ============================================================================
// Context
// ============================================================================

/// Context for managing zenoh-pico shim session
///
/// The context manages the zenoh session lifecycle and provides methods
/// for creating publishers and subscribers.
///
/// # Note
///
/// Each `Context` owns one slot of the C shim's session pool
/// (`ZPICO_MAX_SESSIONS`, default 1) — issue 0348 / phase-328. The raw
/// `handle` pointer also makes `Context` `!Send`/`!Sync`, matching the
/// old `PhantomData<*const ()>` marker.
pub struct Context {
    handle: *mut zpico_session_t,
}

/// The message an exhausted session pool prints, and the marker a test greps.
///
/// Named because it is asserted from OUTSIDE this crate, and CLAUDE.md's rule is
/// that test greps use constants rather than literals — a slimmed banner has
/// broken ~10 tests before.
pub const SESSION_POOL_EXHAUSTED_MARKER: &str = "zenoh session pool exhausted";

/// Take a pool slot, or report the one failure whose reason exists only here.
///
/// Issue 0465 — this is the ONLY frame that knows why. Downstream the C ABI
/// collapses it to an int and the reason is gone, which is how an exhausted
/// session pool spent two months looking like `Transport(ConnectionFailed)` —
/// a router/network problem, and chased as one.
///
/// issue 0589 — `nros_log`, not `std::eprintln!`: std stdio SIGSEGVs a Zephyr
/// native_sim image, which would replace the one explanation this failure has
/// with a bare core dump. It also reaches `no_std` targets, where the old
/// `cfg(feature = "std")` arm left the pool exhaustion mute — and firmware is
/// where a fixed-size pool actually fills.
///
/// issue 0697 — ONE function, because this block was duplicated BYTE FOR BYTE in
/// `new` and `with_config`, 25 lines each with the comments. Two copies of a
/// message a test greps is two things to keep in step, and the test would have
/// pinned only one of them.
fn acquire_session_slot() -> Result<*mut zpico_sys::zpico_session_t> {
    // SAFETY: no arguments; returns a pool slot or null.
    let handle = unsafe { zpico_session_acquire() };
    if handle.is_null() {
        nros_log::log_error!(
            nros_log::get_logger("nros_rmw_zenoh"),
            "{} — this build allows ZPICO_MAX_SESSIONS={} and one is already \
             open. A non-bridge application opens exactly ONE session; two \
             usually means something opened a second executor instead of \
             reusing the global one. Rebuild with ZPICO_MAX_SESSIONS=<n> only if \
             the extra session is genuinely wanted (a bridge).",
            SESSION_POOL_EXHAUSTED_MARKER,
            crate::zpico::ZPICO_MAX_SESSIONS
        );
        return Err(ZpicoError::Full);
    }
    Ok(handle)
}

impl Context {
    /// Create a new shim context with the given locator
    ///
    /// The locator should be a null-terminated string like `b"tcp/127.0.0.1:7447\0"`.
    ///
    /// # Errors
    ///
    /// Returns an error if initialization or session opening fails.
    /// Backoff between connect attempts. Mirrors the service-client retry's
    /// platform split: a real `z_sleep_ms` everywhere except ThreadX, whose
    /// sleep is driven through the cooperative spin path.
    #[inline]
    fn connect_backoff_ms(session: *mut zpico_session_t, ms: usize) {
        #[cfg(not(feature = "platform-threadx"))]
        {
            let _ = session;
            unsafe extern "C" {
                fn z_sleep_ms(time: usize) -> i8;
            }
            unsafe {
                let _ = z_sleep_ms(ms);
            }
        }
        #[cfg(feature = "platform-threadx")]
        {
            unsafe {
                let _ = zpico_sys::zpico_spin_once(session, ms as u32);
            }
        }
    }

    /// Run an `init + open` attempt with a bounded retry on a transient connect
    /// failure.
    ///
    /// `attempt` performs `zpico_init*` + `zpico_open` and returns `Ok(())` on
    /// success or `Err((error, retryable))` — `retryable` is `true` only for the
    /// `zpico_open` (connect) step, `false` for deterministic init/config errors.
    ///
    /// Why: `zpico_open`'s `z_open` TCP connect to the router can flake under
    /// rapid QEMU churn (NuttX cold boot / a fully parallel `test-all`) — a
    /// single connect races the router's accept and returns `ZPICO_ERR_SESSION`
    /// → `ConnectionFailed`, aborting node startup ("readiness pattern never
    /// observed"). Re-running the whole `init + open` (so `zpico_init` rebuilds
    /// the config that `z_open` consumes via `z_config_move`) with a short
    /// backoff recovers the transient failure, matching the robustness the
    /// blocking C client path has. Bounded (~3 s worst case) so a genuinely
    /// wrong locator still fails promptly. See phase-177 G4 connect-churn note.
    fn connect_with_retry(
        session: *mut zpico_session_t,
        mut attempt: impl FnMut() -> core::result::Result<(), (ZpicoError, bool)>,
    ) -> Result<()> {
        const MAX_ATTEMPTS: u32 = 10;
        const BACKOFF_MS: usize = 300;
        let mut last_err = ZpicoError::Session;
        for i in 0..MAX_ATTEMPTS {
            match attempt() {
                Ok(()) => return Ok(()),
                Err((err, retryable)) => {
                    last_err = err;
                    if !retryable || i + 1 == MAX_ATTEMPTS {
                        return Err(last_err);
                    }
                    Self::connect_backoff_ms(session, BACKOFF_MS);
                }
            }
        }
        Err(last_err)
    }

    pub fn new(locator: &[u8]) -> Result<Self> {
        ffi_guard(|| {
            // Acquire a pool slot up front; release it on any init/open failure.
            let handle = acquire_session_slot()?;
            let connect = Self::connect_with_retry(handle, || {
                // Safety: locator is a valid byte slice, cast to c_char for C string
                let ret = unsafe { zpico_init(handle, locator.as_ptr().cast()) };
                if ret < 0 {
                    return Err((ZpicoError::from_code(ret), false));
                }
                let ret = unsafe { zpico_open(handle) };
                if ret < 0 {
                    return Err((ZpicoError::from_code(ret), true));
                }
                Ok(())
            });
            if let Err(e) = connect {
                unsafe { zpico_session_release(handle) };
                return Err(e);
            }

            Ok(Context { handle })
        })
    }

    /// Create a new shim context with mode, locator, and properties
    ///
    /// Byte slices for locator and mode must be null-terminated C strings.
    ///
    /// # Arguments
    ///
    /// * `locator` - Null-terminated locator (e.g., `b"tcp/127.0.0.1:7447\0"`), or `None` for peer mode
    /// * `mode` - Null-terminated mode string (`b"client\0"` or `b"peer\0"`)
    /// * `properties` - Array of C-compatible key-value properties
    ///
    /// # Errors
    ///
    /// Returns an error if initialization or session opening fails.
    pub fn with_config(
        locator: Option<&[u8]>,
        mode: &[u8],
        properties: &[zpico_sys::zpico_property_t],
    ) -> Result<Self> {
        // Issue #64 — stage the locator into a fixed-address static instead of
        // passing a captured `&[u8]`. On esp32-c3 a failed first `zpico_open()`
        // is retryable; during the `connect_backoff_ms` → `z_sleep_ms` network
        // poll, a wild write on the OpenETH poll/DMA path overwrites the retry
        // closure's *captured* locator pointer field with `0xffffffff`, so the
        // 2nd `zpico_init_with_config` faults in `_z_str_clone(0xffffffff)`. The
        // `.bss` static lives outside the clobbered frame and its address is a
        // link constant (recomputed each attempt, never stored in a corruptible
        // stack slot). zpico is single-session/global already, so a static
        // connect buffer matches the existing design. `mode` is a `'static`
        // flash pointer and `properties` is empty on embedded, so both survive
        // as-is; only the DRAM-backed locator needs staging.
        const LOC_CAP: usize = 256;
        static mut LOC_BUF: [u8; LOC_CAP] = [0; LOC_CAP];
        static mut LOC_VALID: bool = false;
        match locator {
            Some(loc) if loc.len() <= LOC_CAP => unsafe {
                core::ptr::copy_nonoverlapping(
                    loc.as_ptr(),
                    (&raw mut LOC_BUF) as *mut u8,
                    loc.len(),
                );
                LOC_VALID = true;
            },
            Some(_) => return Err(ZpicoError::Config),
            None => unsafe { LOC_VALID = false },
        }
        ffi_guard(|| {
            let handle = acquire_session_slot()?;
            let connect = Self::connect_with_retry(handle, || {
                // Read the locator from its constant static address (not a
                // captured pointer) so the retry survives the backoff clobber.
                let locator_ptr: *const core::ffi::c_char = if unsafe { LOC_VALID } {
                    (&raw const LOC_BUF) as *const core::ffi::c_char
                } else {
                    core::ptr::null()
                };
                let props_ptr = if properties.is_empty() {
                    core::ptr::null()
                } else {
                    properties.as_ptr()
                };
                let ret = unsafe {
                    zpico_init_with_config(
                        handle,
                        locator_ptr,
                        mode.as_ptr().cast(),
                        props_ptr,
                        properties.len(),
                    )
                };
                if ret < 0 {
                    return Err((ZpicoError::from_code(ret), false));
                }
                let ret = unsafe { zpico_open(handle) };
                if ret < 0 {
                    return Err((ZpicoError::from_code(ret), true));
                }
                Ok(())
            });
            if let Err(e) = connect {
                unsafe { zpico_session_release(handle) };
                return Err(e);
            }

            Ok(Context { handle })
        })
    }

    /// The raw session handle (for callers that reach `zpico_sys` directly,
    /// e.g. the shim's queryable/reply-waker callbacks and `ping_session`).
    pub(crate) fn handle(&self) -> *mut zpico_session_t {
        self.handle
    }

    /// Check if the session is open
    pub fn is_open(&self) -> bool {
        ffi_guard(|| unsafe { zpico_is_open(self.handle) != 0 })
    }

    /// Check if this backend uses polling
    ///
    /// If true, you must call `poll()` or `spin_once()` regularly to
    /// process network data and dispatch callbacks.
    pub fn uses_polling(&self) -> bool {
        ffi_guard(|| unsafe { zpico_uses_polling() })
    }

    /// Declare a publisher for the given key expression
    ///
    /// The key expression should be a null-terminated string like `b"demo/topic\0"`.
    ///
    /// # Errors
    ///
    /// Returns an error if the session is not open, the key expression is invalid,
    /// or the maximum number of publishers has been reached.
    pub fn declare_publisher(&self, keyexpr: &[u8], tx_express: bool) -> Result<Publisher<'_>> {
        let handle = ffi_guard(|| unsafe {
            zpico_declare_publisher_ex(self.handle, keyexpr.as_ptr().cast(), tx_express as i32)
        });
        if handle < 0 {
            return Err(ZpicoError::from_code(handle));
        }

        Ok(Publisher {
            session: self.handle,
            handle,
            _ctx: PhantomData,
        })
    }

    /// Declare a subscriber for the given key expression
    ///
    /// The key expression should be a null-terminated string like `b"demo/topic\0"`.
    /// The callback will be invoked when samples arrive.
    ///
    /// # Safety
    ///
    /// The callback and context must remain valid for the lifetime of the subscriber.
    /// The context pointer must be valid for the callback to dereference.
    ///
    /// # Errors
    ///
    /// Returns an error if the session is not open, the key expression is invalid,
    /// or the maximum number of subscribers has been reached.
    pub unsafe fn declare_subscriber_raw<'a>(
        &'a self,
        keyexpr: &[u8],
        callback: ZpicoCallback,
        ctx: *mut c_void,
    ) -> Result<Subscriber<'a>> {
        let handle = ffi_guard(|| unsafe {
            zpico_declare_subscriber(self.handle, keyexpr.as_ptr().cast(), callback, ctx)
        });
        if handle < 0 {
            return Err(ZpicoError::from_code(handle));
        }

        Ok(Subscriber {
            session: self.handle,
            handle,
            _ctx: PhantomData,
        })
    }

    /// Declare a subscriber with attachment support for RMW compatibility
    ///
    /// The key expression should be a null-terminated string like `b"demo/topic\0"`.
    /// The callback will be invoked when samples arrive, with attachment data if present.
    ///
    /// # Safety
    ///
    /// The callback and context must remain valid for the lifetime of the subscriber.
    /// The context pointer must be valid for the callback to dereference.
    ///
    /// # Errors
    ///
    /// Returns an error if the session is not open, the key expression is invalid,
    /// or the maximum number of subscribers has been reached.
    pub unsafe fn declare_subscriber_with_attachment_raw<'a>(
        &'a self,
        keyexpr: &[u8],
        callback: ZpicoCallbackWithAttachment,
        ctx: *mut c_void,
    ) -> Result<Subscriber<'a>> {
        let handle = ffi_guard(|| unsafe {
            zpico_declare_subscriber_with_attachment(
                self.handle,
                keyexpr.as_ptr().cast(),
                callback,
                ctx,
            )
        });
        if handle < 0 {
            return Err(ZpicoError::from_code(handle));
        }

        Ok(Subscriber {
            session: self.handle,
            handle,
            _ctx: PhantomData,
        })
    }

    /// Declare a subscriber with direct-write to a Rust buffer.
    ///
    /// The C shim reads the payload directly into `buf_ptr` using
    /// `z_bytes_reader_read()`, avoiding a malloc. The notify callback
    /// is called after the write, providing only the length and attachment.
    ///
    /// # Safety
    ///
    /// `buf_ptr` must point to valid memory for `buf_capacity` bytes that
    /// outlives the subscriber. `locked_ptr` must point to a valid `AtomicBool`.
    pub unsafe fn declare_subscriber_direct_write_raw<'a>(
        &'a self,
        keyexpr: &[u8],
        buf_ptr: *mut u8,
        buf_capacity: usize,
        locked_ptr: *const bool,
        callback: ZpicoNotifyCallback,
        ctx: *mut c_void,
    ) -> Result<Subscriber<'a>> {
        let handle = ffi_guard(|| unsafe {
            zpico_declare_subscriber_direct_write(
                self.handle,
                keyexpr.as_ptr().cast(),
                buf_ptr,
                buf_capacity,
                locked_ptr,
                callback,
                ctx,
            )
        });
        if handle < 0 {
            return Err(ZpicoError::from_code(handle));
        }

        Ok(Subscriber {
            session: self.handle,
            handle,
            _ctx: PhantomData,
        })
    }

    /// Phase 124.D.3.c — declare a burst-tolerant direct-write
    /// subscriber backed by an SPSC ring.
    ///
    /// # Safety
    ///
    /// `desc` must point at a `zpico_ring_desc_t` whose backing
    /// storage (payload / attachment / len arrays / head / tail)
    /// outlives the returned `Subscriber`. The C shim is the sole
    /// writer of `tail`; the caller must be the sole writer of
    /// `head`.
    pub unsafe fn declare_subscriber_ring_raw<'a>(
        &'a self,
        keyexpr: &[u8],
        desc: *mut zpico_ring_desc_t,
        callback: ZpicoNotifyCallback,
        ctx: *mut c_void,
    ) -> Result<Subscriber<'a>> {
        let handle = ffi_guard(|| unsafe {
            zpico_declare_subscriber_ring(self.handle, keyexpr.as_ptr().cast(), desc, callback, ctx)
        });
        if handle < 0 {
            return Err(ZpicoError::from_code(handle));
        }
        Ok(Subscriber {
            session: self.handle,
            handle,
            _ctx: PhantomData,
        })
    }

    /// Combined poll and keepalive operation
    ///
    /// This is equivalent to calling `poll()` and performing any necessary
    /// keepalive operations.
    ///
    /// # Arguments
    ///
    /// * `timeout_ms` - Maximum time to wait (0 = non-blocking)
    ///
    /// # Returns
    ///
    /// Number of events processed, or error
    pub fn spin_once(&self, timeout_ms: u32) -> Result<i32> {
        // When FFI guard is enabled, each `zpico_spin_once` call must
        // run inside a `critical_section::with()` to serialise against
        // mixed-priority tasks that also touch zenoh-pico globals.
        //
        // Phase 77.19 — the inner call is now `zpico_spin_once(remaining_ms)`
        // instead of a busy `zpico_spin_once(0)` tight-loop. On platforms
        // that have event-driven wake primitives (POSIX condvar, Zephyr
        // condvar, FreeRTOS binary semaphore, NuttX `sem_timedwait` — all
        // landed in 77.16 / 77.17), this returns immediately on data
        // arrival instead of burning CPU until the deadline. On
        // polled-smoltcp / polled-serial bare-metal the inner call is
        // still a tight loop but the outer loop no longer adds a second
        // layer of busy-waiting on top.
        //
        // The critical section is held for up to `remaining_ms` per
        // iteration, which is fine today because the only active
        // `ffi-sync` consumer is bare-metal RTIC mixed-priority, and
        // those examples always call with `timeout_ms == 0` (single pass,
        // never enters the loop). If a future consumer calls with
        // `timeout_ms > 0` on a cortex-m RTIC target, this will hold
        // IRQs off for that duration and should be revisited (split the
        // wait and the zpico-state touch into separate CS regions).
        #[cfg(feature = "ffi-sync")]
        {
            let ret = ffi_guard(|| unsafe { zpico_spin_once(self.handle, 0) });
            if ret < 0 {
                return Err(ZpicoError::from_code(ret));
            }
            if ret > 0 || timeout_ms == 0 {
                return Ok(ret);
            }
            // Loop with guarded blocking spin_once calls until timeout
            let mut clock = [0u8; 16];
            unsafe { zpico_sys::zpico_clock_start(clock.as_mut_ptr()) };
            loop {
                let elapsed =
                    unsafe { zpico_sys::zpico_clock_elapsed_ms_since(clock.as_mut_ptr()) };
                let elapsed_u32 = elapsed as u32;
                if elapsed_u32 >= timeout_ms {
                    return Ok(0);
                }
                let remaining_ms = timeout_ms - elapsed_u32;
                let ret = ffi_guard(|| unsafe { zpico_spin_once(self.handle, remaining_ms) });
                if ret < 0 {
                    return Err(ZpicoError::from_code(ret));
                }
                if ret > 0 {
                    return Ok(ret);
                }
            }
        }
        #[cfg(not(feature = "ffi-sync"))]
        {
            let ret = unsafe { zpico_spin_once(self.handle, timeout_ms) };
            if ret < 0 {
                return Err(ZpicoError::from_code(ret));
            }
            Ok(ret)
        }
    }

    /// Get the session's Zenoh ID
    ///
    /// The Zenoh ID uniquely identifies this session in the Zenoh network.
    /// It is used in liveliness token key expressions for ROS 2 discovery.
    pub fn zid(&self) -> Result<ZenohId> {
        let mut id = [0u8; 16];
        let ret = ffi_guard(|| unsafe { zpico_get_zid(self.handle, id.as_mut_ptr()) });
        if ret < 0 {
            return Err(ZpicoError::from_code(ret));
        }
        Ok(ZenohId::from_bytes(id))
    }

    /// Declare a liveliness token for ROS 2 discovery
    ///
    /// The key expression should be a null-terminated string.
    ///
    /// # Errors
    ///
    /// Returns an error if the session is not open, the key expression is invalid,
    /// or the maximum number of liveliness tokens has been reached.
    pub fn declare_liveliness(&self, keyexpr: &[u8]) -> Result<LivelinessToken> {
        let handle =
            ffi_guard(|| unsafe { zpico_declare_liveliness(self.handle, keyexpr.as_ptr().cast()) });
        if handle < 0 {
            return Err(ZpicoError::from_code(handle));
        }

        Ok(LivelinessToken {
            session: self.handle,
            handle,
        })
    }

    /// Declare a queryable for receiving service requests
    ///
    /// The key expression should be a null-terminated string.
    /// The callback will be invoked when queries arrive.
    ///
    /// # Safety
    ///
    /// The callback and context must remain valid for the lifetime of the queryable.
    /// The context pointer must be valid for the callback to dereference.
    ///
    /// # Errors
    ///
    /// Returns an error if the session is not open, the key expression is invalid,
    /// or the maximum number of queryables has been reached.
    pub unsafe fn declare_queryable_raw(
        &self,
        keyexpr: &[u8],
        callback: ZpicoQueryCallback,
        ctx: *mut c_void,
    ) -> Result<Queryable> {
        let handle = ffi_guard(|| unsafe {
            zpico_declare_queryable(self.handle, keyexpr.as_ptr().cast(), callback, ctx)
        });
        if handle < 0 {
            return Err(ZpicoError::from_code(handle));
        }

        Ok(Queryable {
            session: self.handle,
            handle,
        })
    }

    /// Reply to a query (must be called within query callback)
    ///
    /// This sends a reply to the current query being processed.
    /// Must only be called from within a queryable callback.
    ///
    /// # Parameters
    ///
    /// * `queryable_handle` - Handle of the queryable that received the query
    /// * `keyexpr` - Reply key expression (null-terminated)
    /// * `data` - Reply payload
    /// * `attachment` - Optional attachment data
    ///
    /// # Errors
    ///
    /// Returns an error if the queryable handle is invalid, no stored query exists,
    /// or if the reply operation fails.
    pub fn query_reply(
        &self,
        queryable_handle: i32,
        reply_seq: i64,
        keyexpr: &[u8],
        data: &[u8],
        attachment: Option<&[u8]>,
    ) -> Result<()> {
        let (att_ptr, att_len) = match attachment {
            Some(att) => (att.as_ptr(), att.len()),
            None => (core::ptr::null(), 0),
        };

        let ret = ffi_guard(|| unsafe {
            zpico_query_reply(
                self.handle,
                queryable_handle,
                reply_seq,
                keyexpr.as_ptr().cast(),
                data.as_ptr(),
                data.len(),
                att_ptr,
                att_len,
            )
        });
        if ret < 0 {
            return Err(ZpicoError::from_code(ret));
        }
        Ok(())
    }

    /// Phase 237 — reply-slot index allocated by the most recent query callback
    /// for `queryable_handle` (the deferred-reply seq). Call from inside the
    /// synchronous query callback; -1 if the reply table was full.
    pub fn queryable_take_reply_seq(&self, queryable_handle: i32) -> i64 {
        unsafe { zpico_queryable_take_reply_seq(self.handle, queryable_handle) }
    }

    /// Start a non-blocking query (for async service client).
    ///
    /// Returns a slot handle on success that can be polled with [`get_check()`](Self::get_check).
    pub fn get_start(&self, keyexpr: &[u8], payload: &[u8], timeout_ms: u32) -> Result<i32> {
        self.get_start_with_attachment(keyexpr, payload, &[], timeout_ms)
    }

    /// Issue 0153 — query start carrying an rmw attachment (sequence_number +
    /// source_timestamp + gid). rmw_zenoh_cpp service servers REQUIRE it on
    /// the query; an empty `attachment` omits it (nano↔nano tolerates both).
    pub fn get_start_with_attachment(
        &self,
        keyexpr: &[u8],
        payload: &[u8],
        attachment: &[u8],
        timeout_ms: u32,
    ) -> Result<i32> {
        let (payload_ptr, payload_len) = if payload.is_empty() {
            (core::ptr::null(), 0)
        } else {
            (payload.as_ptr(), payload.len())
        };
        let (att_ptr, att_len) = if attachment.is_empty() {
            (core::ptr::null(), 0)
        } else {
            (attachment.as_ptr(), attachment.len())
        };

        let ret = ffi_guard(|| unsafe {
            zpico_sys::zpico_get_start_with_attachment(
                self.handle,
                keyexpr.as_ptr().cast(),
                payload_ptr,
                payload_len,
                att_ptr,
                att_len,
                timeout_ms,
            )
        });

        if ret < 0 {
            return Err(ZpicoError::from_code(ret));
        }

        Ok(ret)
    }

    /// Check for a reply to a pending non-blocking query.
    ///
    /// Returns `Ok(Some(len))` when a reply has arrived, `Ok(None)` if still
    /// pending, or `Err` on failure/timeout.
    pub fn get_check(&self, handle: i32, reply_buf: &mut [u8]) -> Result<Option<usize>> {
        let ret = ffi_guard(|| unsafe {
            zpico_sys::zpico_get_check(self.handle, handle, reply_buf.as_mut_ptr(), reply_buf.len())
        });

        if ret > 0 {
            Ok(Some(ret as usize))
        } else if ret == 0 {
            Ok(None)
        } else {
            if ret == -9 {
                return Err(ZpicoError::Timeout);
            }
            Err(ZpicoError::from_code(ret))
        }
    }

    /// Start a non-blocking liveliness query.
    ///
    /// Returns a slot handle on success that can be polled with
    /// [`liveliness_get_check()`](Self::liveliness_get_check).
    /// `keyexpr` must be a null-terminated byte slice.
    pub fn liveliness_get_start(&self, keyexpr: &[u8], timeout_ms: u32) -> Result<i32> {
        let ret = ffi_guard(|| unsafe {
            zpico_sys::zpico_liveliness_get_start(self.handle, keyexpr.as_ptr().cast(), timeout_ms)
        });

        if ret < 0 {
            return Err(ZpicoError::from_code(ret));
        }
        Ok(ret)
    }

    /// Poll a pending liveliness query.
    ///
    /// Returns `Ok(true)` once at least one matching liveliness token has
    /// reported back, `Ok(false)` while the query is still in flight with
    /// no replies yet. Returns `Err(ZpicoError::Timeout)` when the query
    /// dropper fired without seeing any matching token (no server visible
    /// within the timeout).
    pub fn liveliness_get_check(&self, handle: i32) -> Result<bool> {
        let ret =
            ffi_guard(|| unsafe { zpico_sys::zpico_liveliness_get_check(self.handle, handle) });

        if ret == 1 {
            Ok(true)
        } else if ret == 0 {
            Ok(false)
        } else if ret == -9 {
            Err(ZpicoError::Timeout)
        } else {
            Err(ZpicoError::from_code(ret))
        }
    }

    /// Phase 108.C.zenoh.4-followup — count of liveliness-token replies
    /// received on this slot. Used by the subscriber-side
    /// `LivelinessChanged` bridge to surface `alive_count > 1` when more
    /// than one publisher matches the wildcard liveliness keyexpr.
    /// Returns `0` while the query is in flight; the count is final
    /// once `liveliness_get_check` returns `Ok(true)` / `Err(Timeout)`.
    pub fn liveliness_get_count(&self, handle: i32) -> Result<u32> {
        let ret =
            ffi_guard(|| unsafe { zpico_sys::zpico_liveliness_get_count(self.handle, handle) });
        if ret < 0 {
            Err(ZpicoError::from_code(ret))
        } else {
            Ok(ret as u32)
        }
    }
}

impl Drop for Context {
    fn drop(&mut self) {
        ffi_guard(|| unsafe {
            zpico_close(self.handle);
            zpico_session_release(self.handle);
        });
    }
}

// ============================================================================
// Publisher
// ============================================================================

/// Publisher handle for sending data
///
/// Created via `Context::declare_publisher()`.
pub struct Publisher<'a> {
    session: *mut zpico_session_t,
    handle: i32,
    _ctx: PhantomData<&'a Context>,
}

impl<'a> Publisher<'a> {
    /// Publish data
    ///
    /// # Errors
    ///
    /// Returns an error if the publish operation fails.
    pub fn publish(&self, data: &[u8]) -> Result<()> {
        let ret = ffi_guard(|| unsafe {
            zpico_publish(self.session, self.handle, data.as_ptr(), data.len())
        });
        if ret < 0 {
            return Err(ZpicoError::from_code(ret));
        }
        Ok(())
    }

    /// Publish data with an attachment
    ///
    /// This is used for RMW compatibility, where an attachment contains
    /// metadata like sequence number, timestamp, and GID.
    ///
    /// # Parameters
    ///
    /// * `data` - The message payload
    /// * `attachment` - Optional attachment data (for RMW compatibility)
    ///
    /// # Errors
    ///
    /// Returns an error if the publish operation fails.
    pub fn publish_with_attachment(&self, data: &[u8], attachment: Option<&[u8]>) -> Result<()> {
        let (att_ptr, att_len) = match attachment {
            Some(att) => (att.as_ptr(), att.len()),
            None => (core::ptr::null(), 0),
        };

        let ret = ffi_guard(|| unsafe {
            zpico_publish_with_attachment(
                self.session,
                self.handle,
                data.as_ptr(),
                data.len(),
                att_ptr,
                att_len,
            )
        });
        if ret < 0 {
            return Err(ZpicoError::from_code(ret));
        }
        Ok(())
    }

    /// Phase 99.F — zero-copy publish via z_bytes_from_static_buf.
    ///
    /// Identical to [`Self::publish_with_attachment`] but ALIASES the
    /// payload pointer instead of copying. Caller MUST guarantee
    /// `data` outlives the call (zenoh-pico's posix/embedded
    /// transports consume the alias synchronously before
    /// `z_publisher_put` returns).
    ///
    /// Used by the SlotLending impl on ZenohPublisher to back the
    /// EmbeddedRawPublisher::try_loan zero-copy path.
    pub fn publish_with_attachment_aliased(
        &self,
        data: &[u8],
        attachment: Option<&[u8]>,
    ) -> Result<()> {
        let (att_ptr, att_len) = match attachment {
            Some(att) => (att.as_ptr(), att.len()),
            None => (core::ptr::null(), 0),
        };

        let ret = ffi_guard(|| unsafe {
            zpico_publish_with_attachment_aliased(
                self.session,
                self.handle,
                data.as_ptr(),
                data.len(),
                att_ptr,
                att_len,
            )
        });
        if ret < 0 {
            return Err(ZpicoError::from_code(ret));
        }
        Ok(())
    }

    /// Get the publisher handle
    pub fn handle(&self) -> i32 {
        self.handle
    }

    /// The owning session handle (for the shim's direct `publish_streamed`
    /// call, which bypasses this wrapper). Same cfg as that sole caller —
    /// `safety-e2e` builds take the staging-buffer path and would otherwise
    /// trip `-D dead_code`.
    #[cfg(not(feature = "safety-e2e"))]
    pub(crate) fn session(&self) -> *mut zpico_session_t {
        self.session
    }
}

impl<'a> Drop for Publisher<'a> {
    fn drop(&mut self) {
        ffi_guard(|| unsafe {
            zpico_undeclare_publisher(self.session, self.handle);
        });
    }
}

// ============================================================================
// Subscriber
// ============================================================================

/// Subscriber handle for receiving data
///
/// Created via `Context::declare_subscriber_raw()`.
pub struct Subscriber<'a> {
    session: *mut zpico_session_t,
    handle: i32,
    _ctx: PhantomData<&'a Context>,
}

impl<'a> Subscriber<'a> {
    /// Get the subscriber handle
    pub fn handle(&self) -> i32 {
        self.handle
    }
}

impl<'a> Drop for Subscriber<'a> {
    fn drop(&mut self) {
        ffi_guard(|| unsafe {
            zpico_undeclare_subscriber(self.session, self.handle);
        });
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use std::format;

    #[test]
    fn test_error_from_code() {
        assert_eq!(ZpicoError::from_code(-1), ZpicoError::Generic);
        assert_eq!(ZpicoError::from_code(-2), ZpicoError::Config);
        assert_eq!(ZpicoError::from_code(-3), ZpicoError::Session);
        assert_eq!(ZpicoError::from_code(-4), ZpicoError::Task);
        assert_eq!(ZpicoError::from_code(-5), ZpicoError::KeyExpr);
        assert_eq!(ZpicoError::from_code(-6), ZpicoError::Full);
        assert_eq!(ZpicoError::from_code(-7), ZpicoError::Invalid);
        assert_eq!(ZpicoError::from_code(-8), ZpicoError::Publish);
        assert_eq!(ZpicoError::from_code(-99), ZpicoError::Generic);
    }

    #[test]
    fn test_error_display() {
        assert_eq!(format!("{}", ZpicoError::Generic), "generic error");
        assert_eq!(format!("{}", ZpicoError::Session), "session error");
    }
}
