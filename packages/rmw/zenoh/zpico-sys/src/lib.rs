//! zpico-sys: C wrapper library for zenoh-pico with FFI bindings
//!
//! This crate provides:
//! - The compiled zpico C library (zpico.c)
//! - FFI constants and types
//! - zenoh-pico library (compiled from submodule)
//!
//! # Platform Backends
//!
//! Select one backend via feature flags:
//! - `posix` - Uses POSIX threads, for desktop testing
//! - `zephyr` - Uses Zephyr RTOS threads
//! - `bare-metal` - Uses polling (bare-metal platforms)
//! - `freertos` - Uses FreeRTOS threads + lwIP sockets

#![no_std]

#[cfg(any(feature = "std", test))]
extern crate std;

// Force-link the platform shim crate so its extern "C" symbols (z_clock_now,
// z_malloc, _z_mutex_lock, etc.) are available to the C objects in this crate.
// On POSIX/RTOS, `extern crate` suffices. On bare-metal, the board crate must
// also directly depend on zpico-platform-shim for the embedded linker to
// include the symbols (see board crate Cargo.toml).
// Force-link: POSIX/NuttX/FreeRTOS/ThreadX use the shim for platform symbols.
// Bare-metal board crates have their own extern crate for the embedded linker.
// Phase 129.D — `zpico-platform-shim` retired. The C alias TU
// (`c/zpico/platform_aliases.c`, default-on via `platform-aliases`)
// emits every `z_*` / `_z_*` symbol the shim used to provide;
// no `extern crate` keep-alive needed any more. IVC link-layer
// forwarders moved to the carved-out `zpico-link-ivc` crate,
// keep-alived below.
#[cfg(feature = "link-ivc")]
extern crate zpico_link_ivc;

// Phase 115.B — force-link the Rust-side `nros_zpico_custom_take`
// symbol that the C custom-link factory calls. Same pattern as
// `extern crate zpico_platform_shim` above.
#[cfg(feature = "link-custom")]
extern crate zpico_platform_custom;

// Note: The smoltcp platform uses a custom bump allocator for C FFI (zenoh-pico),
// not Rust's global allocator. The `alloc` crate is NOT needed.

// Phase 227.3(B) — the C-ABI import declarations below are
// platform-agnostic (identical signatures on every target); the
// per-platform feature gate was vestigial. Imports are resolved at
// link time against the C lib (`c/zpico/zpico.c`), so declaring them
// unconditionally is harmless on a platform-feature-free `cargo
// build`. Only the `cbindgen` guard is genuine (cbindgen must not
// re-emit a Rust import of a C function into the generated header).
#[cfg(not(cbindgen))]
use core::ffi::c_void;

// ============================================================================
// Configuration Constants
// ============================================================================

pub mod config;
pub use config::*;

// ============================================================================
// FFI Declarations
// ============================================================================

mod ffi;
pub use ffi::*;

// ============================================================================
// Platform-specific Modules
// ============================================================================

// Note: The C platform layer (`c/platform/`) provides bare-metal
// headers and optional C shims. Platform crates (`zpico-platform-*`)
// provide system primitives (clock, memory, RNG) and the transport
// crate provides TCP symbols directly in Rust.

// ============================================================================
// Extern C Functions from the Shim
// ============================================================================

/// A key-value property for transport configuration (C-compatible)
#[repr(C)]
pub struct zpico_property_t {
    /// Property key (null-terminated C string)
    pub key: *const core::ffi::c_char,
    /// Property value (null-terminated C string)
    pub value: *const core::ffi::c_char,
}

/// Phase 124.D.3.c — SPSC ring descriptor mirroring
/// `zpico_ring_desc_t` in `c/include/zpico.h`. Field order, names,
/// and types track the C struct byte-for-byte. The Rust shim owns
/// the backing storage (a `SubscriberBuffer`) and fills this
/// descriptor; the C shim reads it from `sample_handler`.
///
/// `head` / `tail` are monotonic counters accessed with atomics on
/// both sides — the slot index is `counter % slot_count`.
#[repr(C)]
pub struct zpico_ring_desc_t {
    /// `slot_count * payload_stride` bytes of payload storage.
    pub payload_base: *mut u8,
    /// Bytes between payload slot starts.
    pub payload_stride: usize,
    /// `slot_count * att_stride` bytes of attachment storage.
    pub att_base: *mut u8,
    /// Bytes between attachment slot starts.
    pub att_stride: usize,
    /// Number of ring slots N.
    pub slot_count: usize,
    /// `slot_count` entries — per-slot payload length.
    pub payload_len: *mut usize,
    /// `slot_count` entries — per-slot attachment length.
    pub att_len: *mut usize,
    /// Consumer counter — written only by the Rust shim.
    pub head: *mut usize,
    /// Producer counter — written only by the C shim.
    pub tail: *mut usize,
}

// These extern declarations import the zpico C functions.
// The actual implementations are in c/zpico/zpico.c
//
// Note: Excluded from cbindgen - these are Rust imports of C functions,
// not declarations for the header file.
// Phase 227.3(B) — un-gated from the per-platform umbrella (vestigial;
// see the `use core::ffi::c_void` note above). The `not(cbindgen)`
// guard stays — these are Rust imports of C functions, not header
// declarations.
/// Opaque per-session handle (issue 0348 / phase-328). A `zpico_session_t*`
/// points into the C shim's compile-time session pool
/// (`ZPICO_MAX_SESSIONS`, default 1). Every `zpico_*` entry point that
/// operates on a session takes one as its leading argument. Rust never
/// dereferences it — it is a bare pointer forwarded to the C shim.
///
/// EXCEPTIONS to the handle rule (process-wide, no session arg):
/// `zpico_set_task_config` / `zpico_set_flush_task_config` are process-wide
/// task-spawn DEFAULTS applied at `zpico_open` (they are called from board
/// boot before any session exists), and `zpico_get_diag_counters` /
/// `zpico_uses_polling` / the clock helpers are process-global.
#[repr(C)]
pub struct zpico_session_t {
    _private: [u8; 0],
}

#[cfg(not(cbindgen))]
#[allow(improper_ctypes)]
unsafe extern "C" {
    // Session lifecycle
    /// Acquire a free slot from the C shim's session pool. Returns NULL when
    /// the pool (`ZPICO_MAX_SESSIONS`) is exhausted. Pair with
    /// `zpico_session_release` after `zpico_close`.
    pub fn zpico_session_acquire() -> *mut zpico_session_t;
    /// Return a slot to the pool. Call after `zpico_close`; the pointer is
    /// invalid afterwards.
    pub fn zpico_session_release(session: *mut zpico_session_t);
    pub fn zpico_init(session: *mut zpico_session_t, locator: *const core::ffi::c_char) -> i32;
    pub fn zpico_init_with_config(
        session: *mut zpico_session_t,
        locator: *const core::ffi::c_char,
        mode: *const core::ffi::c_char,
        properties: *const zpico_property_t,
        num_properties: usize,
    ) -> i32;
    pub fn zpico_open(session: *mut zpico_session_t) -> i32;
    pub fn zpico_is_open(session: *mut zpico_session_t) -> i32;
    /// Phase 124.F.2 — wire-level connectivity probe. Returns 0
    /// on success, `ZPICO_ERR_*` on failure.
    pub fn zpico_send_keep_alive(session: *mut zpico_session_t) -> i32;
    pub fn zpico_close(session: *mut zpico_session_t);

    // Task scheduling configuration (process-wide default; call before
    // zpico_open — see the zpico_session_t exception note above).
    pub fn zpico_set_task_config(
        read_priority: u32,
        read_stack_bytes: u32,
        lease_priority: u32,
        lease_stack_bytes: u32,
    );

    // ZenohId
    pub fn zpico_get_zid(session: *mut zpico_session_t, zid_out: *mut u8) -> i32;

    // Publishers
    pub fn zpico_declare_publisher(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
    ) -> i32;
    pub fn zpico_declare_publisher_ex(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
        is_express: i32,
    ) -> i32;
    pub fn zpico_publish(
        session: *mut zpico_session_t,
        handle: i32,
        data: *const u8,
        len: usize,
    ) -> i32;
    /// Phase 124.E.3 — streamed publish via zenoh-pico's
    /// `z_bytes_writer` API. `chunk_cb` is invoked repeatedly with
    /// up to 1 KiB buffers until `total_len` bytes have landed.
    /// `attachment` carries the ROS-interop metadata (seq + source
    /// timestamp + GID); pass NULL / 0 for a bare publish.
    pub fn zpico_publish_streamed(
        session: *mut zpico_session_t,
        handle: i32,
        total_len: usize,
        chunk_cb: Option<
            unsafe extern "C" fn(
                out_buf: *mut u8,
                cap: usize,
                out_written: *mut usize,
                user_ctx: *mut core::ffi::c_void,
            ),
        >,
        user_ctx: *mut core::ffi::c_void,
        attachment: *const u8,
        attachment_len: usize,
    ) -> i32;
    pub fn zpico_publish_with_attachment(
        session: *mut zpico_session_t,
        handle: i32,
        data: *const u8,
        len: usize,
        attachment: *const u8,
        attachment_len: usize,
    ) -> i32;
    /// Phase 99.F: zero-copy publish via z_bytes_from_static_buf.
    /// Caller guarantees `data` outlives the call.
    pub fn zpico_publish_with_attachment_aliased(
        session: *mut zpico_session_t,
        handle: i32,
        data: *const u8,
        len: usize,
        attachment: *const u8,
        attachment_len: usize,
    ) -> i32;
    pub fn zpico_undeclare_publisher(session: *mut zpico_session_t, handle: i32) -> i32;

    // Subscribers
    pub fn zpico_declare_subscriber(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
        callback: ZpicoCallback,
        ctx: *mut c_void,
    ) -> i32;
    pub fn zpico_declare_subscriber_with_attachment(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
        callback: ZpicoCallbackWithAttachment,
        ctx: *mut c_void,
    ) -> i32;
    pub fn zpico_declare_subscriber_direct_write(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
        buf_ptr: *mut u8,
        buf_capacity: usize,
        locked_ptr: *const bool,
        callback: ZpicoNotifyCallback,
        ctx: *mut c_void,
    ) -> i32;
    /// phase-412 -- which exit `zpico_declare_subscriber_ring` took (1..=6,
    /// 0 = entered and stamped nothing), and the raw zenoh-pico code when the
    /// declare itself failed. Written by the C shim on every call; read by the
    /// caller immediately after a failure. See the comment beside their
    /// definition in `c/zpico/zpico.c`.
    pub static zpico_last_sub_declare_exit: i32;
    /// Raw `z_declare_subscriber` return, valid when the exit above is 5.
    pub static zpico_last_sub_declare_ret: i32;

    pub fn zpico_declare_subscriber_ring(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
        desc: *mut zpico_ring_desc_t,
        callback: ZpicoNotifyCallback,
        ctx: *mut c_void,
    ) -> i32;
    pub fn zpico_subscribe_zero_copy(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
        callback: ZpicoZeroCopyCallback,
        ctx: *mut c_void,
    ) -> i32;
    pub fn zpico_undeclare_subscriber(session: *mut zpico_session_t, handle: i32) -> i32;

    // Liveliness
    pub fn zpico_declare_liveliness(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
    ) -> i32;
    pub fn zpico_undeclare_liveliness(session: *mut zpico_session_t, handle: i32) -> i32;

    // Queryables (for services)
    pub fn zpico_declare_queryable(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
        callback: ZpicoQueryCallback,
        ctx: *mut c_void,
    ) -> i32;
    pub fn zpico_undeclare_queryable(session: *mut zpico_session_t, handle: i32) -> i32;
    pub fn zpico_query_reply(
        session: *mut zpico_session_t,
        queryable_handle: i32,
        reply_seq: i64,
        keyexpr: *const core::ffi::c_char,
        data: *const u8,
        len: usize,
        attachment: *const u8,
        attachment_len: usize,
    ) -> i32;
    /// Phase 237 — reply-slot index from the most recent query callback (the
    /// deferred-reply seq); call from inside the synchronous query callback.
    pub fn zpico_queryable_take_reply_seq(
        session: *mut zpico_session_t,
        queryable_handle: i32,
    ) -> i64;

    // Service client (queries)
    pub fn zpico_get(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
        payload: *const u8,
        payload_len: usize,
        reply_buf: *mut u8,
        reply_buf_size: usize,
        timeout_ms: u32,
    ) -> i32;

    // Non-blocking service client (async queries)
    pub fn zpico_get_start(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
        payload: *const u8,
        payload_len: usize,
        timeout_ms: u32,
    ) -> i32;
    pub fn zpico_get_start_with_attachment(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
        payload: *const u8,
        payload_len: usize,
        attachment: *const u8,
        attachment_len: usize,
        timeout_ms: u32,
    ) -> i32;
    pub fn zpico_get_check(
        session: *mut zpico_session_t,
        handle: i32,
        reply_buf: *mut u8,
        reply_buf_size: usize,
    ) -> i32;

    // Non-blocking liveliness query (for wait_for_service / wait_for_action_server).
    pub fn zpico_liveliness_get_start(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
        timeout_ms: u32,
    ) -> i32;
    pub fn zpico_liveliness_get_check(session: *mut zpico_session_t, handle: i32) -> i32;
    /// Phase 108.C.zenoh.4-followup — count of liveliness-token
    /// replies on this slot. Used by the subscriber-side
    /// `LivelinessChanged` bridge to surface `alive_count > 1`.
    pub fn zpico_liveliness_get_count(session: *mut zpico_session_t, handle: i32) -> i32;

    /// phase-381 W1 — a liveliness query that KEEPS its replies' keyexprs.
    /// Poll with `zpico_liveliness_get_check`, then read with the two below.
    pub fn zpico_liveliness_collect_start(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
        timeout_ms: u32,
    ) -> i32;

    /// How many keyexprs were STORED — compare against
    /// `zpico_liveliness_get_count` (how many ARRIVED) to detect truncation.
    /// Has the collecting sweep FINISHED? `zpico_liveliness_get_check` returns
    /// 1 on the FIRST reply, so restarting on that truncates enumeration.
    pub fn zpico_liveliness_collect_done(session: *mut zpico_session_t, handle: i32) -> i32;

    pub fn zpico_liveliness_entry_count(session: *mut zpico_session_t, handle: i32) -> i32;

    /// Copy stored keyexpr `index` into `out`. Returns bytes written excluding
    /// the NUL, or a negative `ZPICO_ERR_*`.
    pub fn zpico_liveliness_entry(
        session: *mut zpico_session_t,
        handle: i32,
        index: u32,
        out: *mut core::ffi::c_char,
        cap: usize,
    ) -> i32;

    /// The PURE half of `zpico_liveliness_entry`: index into a NUL-separated
    /// run. Reachable without a live session, which is why the graph cache
    /// reuses it to walk its snapshot instead of growing a second walk.
    pub fn zpico_entry_at(
        buf: *const u8,
        len: usize,
        count: u32,
        index: u32,
        out: *mut core::ffi::c_char,
        cap: usize,
    ) -> i32;

    /// phase-381 / issue 0903 — the GRAPH CACHE, which replaces the per-question
    /// liveliness sweep.
    ///
    /// A liveliness get is an INTEREST under the hood, and a token only reaches
    /// a get's callback when the router tags its declaration with that interest
    /// id. Measured against a live `rmw_zenoh_cpp` talker, two sweeps in flight
    /// did not both receive replies, and a single sweep saw 2-4 of the dozen
    /// tokens the talker declares — a different subset each run. A SUBSCRIBER
    /// with `history` is what `rmw_zenoh_cpp` uses for its own graph cache and
    /// removes the class: one standing declaration, current tokens delivered
    /// once, later changes pushed.
    ///
    /// Idempotent, so every graph entry point may call it without coordinating.
    pub fn zpico_graph_cache_start(
        session: *mut zpico_session_t,
        keyexpr: *const core::ffi::c_char,
    ) -> i32;

    /// Stop the cache and release the subscriber. Idempotent.
    pub fn zpico_graph_cache_stop(session: *mut zpico_session_t) -> i32;

    /// How many tokens the cache holds, and how many did not fit.
    pub fn zpico_graph_entry_count(session: *mut zpico_session_t, out_dropped: *mut u32) -> i32;

    /// Copy cached keyexpr `index` into `out`, NUL-terminated. ONE entry per
    /// call, under the C side's lock — the cache is sized for a real ROS graph
    /// (tens of KB), which cannot live on an embedded caller's stack, and this
    /// keeps that capacity a fact about the C side alone.
    pub fn zpico_graph_entry_at(
        session: *mut zpico_session_t,
        index: u32,
        out: *mut core::ffi::c_char,
        cap: usize,
    ) -> i32;

    /// The session's pool index (0..ZPICO_MAX_SESSIONS), or -1 if the handle is
    /// not a valid pool slot. Used to scope the Rust shim's process-global
    /// service-buffer / reply-waker tables per session (issue 0376).
    pub fn zpico_session_index(session: *mut zpico_session_t) -> i32;

    // Reply waker callback (for async service client) — per-session. The
    // callback receives (session_index, slot) so the Rust waker table is
    // session-scoped (issue 0376).
    pub fn zpico_set_reply_waker(
        session: *mut zpico_session_t,
        func: Option<unsafe extern "C" fn(i32, i32)>,
    );

    // Phase 127.D — get/get_check/reply-handler/dropper diagnostic counters
    // (process-global — see the zpico_session_t exception note above).
    // out fills with [get_start, get_check, get_check_returns_data,
    // reply_handler_calls, reply_dropper_calls].
    pub fn zpico_get_diag_counters(out: *mut u32);

    // Polling
    pub fn zpico_spin_once(session: *mut zpico_session_t, timeout_ms: u32) -> i32;
    pub fn zpico_uses_polling() -> bool;

    // Clock helpers (for FFI reentrancy guard timeout decomposition) — process-global.
    pub fn zpico_clock_start(clock_buf: *mut u8);
    pub fn zpico_clock_elapsed_ms_since(clock_buf: *mut u8) -> core::ffi::c_ulong;
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// phase-381 W1 — the NUL-separated entry walk, against the REAL C
    /// function rather than a Rust copy of it.
    ///
    /// `zpico_entry_at` is the pure half of `zpico_liveliness_entry`, split out
    /// precisely so it can be reached without a live zenoh session. This is
    /// where an off-by-one costs a WRONG keyexpr rather than a missing one, and
    /// a wrong one names a real, different node — so the last entry (which has
    /// no trailing separator to lean on) and the short-buffer refusal both get
    /// their own case.
    #[test]
    fn entry_walk_indexes_a_nul_separated_run() {
        unsafe extern "C" {
            fn zpico_entry_at(
                buf: *const u8,
                len: usize,
                count: u32,
                index: u32,
                out: *mut core::ffi::c_char,
                cap: usize,
            ) -> i32;
        }

        // Exactly what the reply handler writes: each keyexpr NUL-terminated.
        let raw: &[u8] =
            b"@ros2_lv/0/aa/1/2/talker\0@ros2_lv/0/bb/1/3/listener\0@ros2_lv/0/cc/1/4/x\0";
        let mut out = [0i8; 128];

        let read = |index: u32, cap: usize, out: &mut [i8; 128]| -> i32 {
            unsafe {
                zpico_entry_at(
                    raw.as_ptr(),
                    raw.len(),
                    3,
                    index,
                    out.as_mut_ptr().cast::<core::ffi::c_char>(),
                    cap,
                )
            }
        };
        // `no_std`: compare the written bytes directly rather than building a
        // String.
        fn written(out: &[i8; 128], n: i32) -> &[u8] {
            // SAFETY: `out` is a plain byte array reinterpreted as unsigned;
            // `n` is what the C function reported it wrote.
            unsafe { core::slice::from_raw_parts(out.as_ptr().cast::<u8>(), n as usize) }
        }

        let n = read(0, out.len(), &mut out);
        assert_eq!(written(&out, n), b"@ros2_lv/0/aa/1/2/talker", "first entry");

        let n = read(1, out.len(), &mut out);
        assert_eq!(
            written(&out, n),
            b"@ros2_lv/0/bb/1/3/listener",
            "middle entry"
        );

        let n = read(2, out.len(), &mut out);
        assert_eq!(
            written(&out, n),
            b"@ros2_lv/0/cc/1/4/x",
            "LAST entry — no trailing separator to lean on"
        );

        assert_eq!(
            read(3, out.len(), &mut out),
            ZPICO_ERR_INVALID,
            "an index past the stored count must be refused, not read off the end"
        );

        // The reason a short buffer must never truncate: the prefix of a
        // keyexpr is itself a VALID keyexpr, naming a different entity.
        assert_eq!(
            read(0, 10, &mut out),
            ZPICO_ERR_BUFFER,
            "a short buffer must be refused rather than truncated"
        );
    }

    #[test]
    fn test_constants() {
        assert_eq!(ZPICO_OK, 0);
        assert_eq!(ZPICO_ERR_GENERIC, -1);
        const { assert!(ZPICO_MAX_PUBLISHERS > 0) };
        const { assert!(ZPICO_MAX_SUBSCRIBERS > 0) };
    }
}
