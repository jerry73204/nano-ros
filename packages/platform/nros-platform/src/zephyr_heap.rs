//! phase-391 W3 — the Zephyr rlsf arena behind `nros_platform_alloc`.
//!
//! On Zephyr the allocation funnel is C
//! (`nros-platform-zephyr/src/platform.c`): `z_malloc` and the compiler's
//! `__rust_alloc` both tail-call `nros_platform_alloc`, which historically was
//! three `k_malloc`/`k_free` lines against `CONFIG_HEAP_MEM_POOL_SIZE`'s
//! `sys_heap`. This module gives that funnel an rlsf-backed arena instead:
//! O(1) alloc/free (the property a safety image needs), and once an image's
//! conf sets `CONFIG_HEAP_MEM_POOL_SIZE=0`, `k_malloc`/`sys_heap_*`
//! garbage-collect out of the link — which is simultaneously the wave's link
//! test that no enabled Zephyr subsystem still needed them.
//!
//! # Concurrency
//!
//! `FreeListHeap` is single-threaded by contract and Zephyr is not: the
//! zenoh-pico read/lease tasks allocate concurrently with the app. The C
//! funnel wraps every call to these exports in a `k_spinlock`
//! (`platform.c`), which is correct because rlsf is O(1) — the critical
//! section is short and bounded. These exports MUST NOT be called from any
//! other path.
//!
//! # Sizing
//!
//! `NROS_ZEPHYR_HEAP_SIZE` (compile-time env, decimal bytes), default 64 KiB —
//! matching the zenoh images' previous `CONFIG_HEAP_MEM_POOL_SIZE=65536`, so a
//! converted image's RAM is net-neutral before any tuning. An image that has
//! NOT set its pool to 0 temporarily carries both arenas; convert the conf in
//! the same change.

use zpico_alloc::FreeListHeap;

const DEFAULT_HEAP_SIZE: usize = 64 * 1024;

const HEAP_SIZE: usize = match option_env!("NROS_ZEPHYR_HEAP_SIZE") {
    Some(s) => parse_usize(s),
    None => DEFAULT_HEAP_SIZE,
};

/// `const`-evaluable decimal parse — same pattern as the bare-metal ports'
/// `memory.rs` (`NROS_HEAP_SIZE`).
const fn parse_usize(s: &str) -> usize {
    let b = s.as_bytes();
    let mut v = 0usize;
    let mut i = 0;
    while i < b.len() {
        assert!(
            b[i] >= b'0' && b[i] <= b'9',
            "NROS_ZEPHYR_HEAP_SIZE must be decimal bytes"
        );
        v = v * 10 + (b[i] - b'0') as usize;
        i += 1;
    }
    v
}

static HEAP: FreeListHeap<HEAP_SIZE> = FreeListHeap::new();

/// The C funnel's backing. Callers hold the funnel's `k_spinlock`.
#[unsafe(no_mangle)]
pub extern "C" fn nros_zephyr_heap_alloc(size: usize) -> *mut core::ffi::c_void {
    HEAP.alloc(size)
}

/// See [`nros_zephyr_heap_alloc`].
#[unsafe(no_mangle)]
pub extern "C" fn nros_zephyr_heap_realloc(
    ptr: *mut core::ffi::c_void,
    size: usize,
) -> *mut core::ffi::c_void {
    HEAP.realloc(ptr, size)
}

/// See [`nros_zephyr_heap_alloc`].
#[unsafe(no_mangle)]
pub extern "C" fn nros_zephyr_heap_free(ptr: *mut core::ffi::c_void) {
    HEAP.free(ptr)
}

/// Managed capacity, for the platform's unified-heap figures.
#[unsafe(no_mangle)]
pub extern "C" fn nros_zephyr_heap_capacity() -> usize {
    HEAP.capacity()
}

/// phase-412 -- bytes this arena currently has handed out.
///
/// `FreeListHeap` has tracked this all along; nothing exported it, so the
/// platform's `nros_platform_heap_used_bytes()` answered from Zephyr's
/// `_system_heap` instead -- the heap the application STOPPED allocating from
/// when the funnel moved here (phase-391 W3). The figure that would size
/// `NROS_ZEPHYR_HEAP_SIZE` was measuring a different arena.
#[cfg(feature = "heap-stats")]
#[unsafe(no_mangle)]
pub extern "C" fn nros_zephyr_heap_used() -> usize {
    HEAP.used()
}

/// 0 = "unknown", the convention `nros_platform_heap_used_bytes` already uses
/// when its stats are not compiled in.
#[cfg(not(feature = "heap-stats"))]
#[unsafe(no_mangle)]
pub extern "C" fn nros_zephyr_heap_used() -> usize {
    0
}

/// The most this arena has ever had handed out at once.
///
/// This is the number `NROS_ZEPHYR_HEAP_SIZE` should be set from, plus
/// headroom. `used` sampled at an arbitrary instant reports whatever happened
/// to be live at the moment of the read, which is not what the knob bounds.
#[cfg(feature = "heap-stats")]
#[unsafe(no_mangle)]
pub extern "C" fn nros_zephyr_heap_peak() -> usize {
    HEAP.peak()
}

/// See [`nros_zephyr_heap_used`] for the 0 convention.
#[cfg(not(feature = "heap-stats"))]
#[unsafe(no_mangle)]
pub extern "C" fn nros_zephyr_heap_peak() -> usize {
    0
}
