//! Free-list allocator for zenoh-pico `z_malloc`/`z_free` on bare-metal.
//!
//! Provides a generic [`FreeListHeap`] that platform crates instantiate as a
//! static with their desired heap size. The allocator uses first-fit search
//! with address-ordered free list and two-pass coalescing on free.
//!
//! ## Slab fast-path
//!
//! Allocations ≤ `SLAB_SLOT_SIZE` (64) bytes are served from a small slab cache
//! (8 slots × 64 bytes = 512 bytes) with O(1) bitmap-based alloc/free. This
//! targets zenoh-pico's per-message string field allocations (short-lived
//! `z_malloc` + `z_free` pairs during CDR parsing). Larger allocations fall
//! through to the free-list.
//!
//! # Usage
//!
//! ```rust,ignore
//! use zpico_alloc::FreeListHeap;
//!
//! static HEAP: FreeListHeap<{64 * 1024}> = FreeListHeap::new();
//!
//! #[unsafe(no_mangle)]
//! pub extern "C" fn z_malloc(size: usize) -> *mut core::ffi::c_void {
//!     HEAP.alloc(size)
//! }
//! #[unsafe(no_mangle)]
//! pub extern "C" fn z_free(ptr: *mut core::ffi::c_void) {
//!     HEAP.free(ptr)
//! }
//! #[unsafe(no_mangle)]
//! pub extern "C" fn z_realloc(ptr: *mut core::ffi::c_void, size: usize) -> *mut core::ffi::c_void {
//!     HEAP.realloc(ptr, size)
//! }
//! ```
//!
//! ## Unified heap — this crate is the ARENA, not the allocator
//!
//! A bare-metal board uses the *same* free-list heap for zenoh-pico's
//! `z_malloc` and for Rust's `alloc` (`Box`/`Vec`/`String`) — one heap, not
//! two. The Rust half reaches it through the platform's `PlatformAlloc` impl,
//! and `nros-platform` installs the single `#[global_allocator]` in the tree.
//!
//! phase-361 W8.c / issue 0594 — [`FreeListHeap`] no longer implements
//! [`core::alloc::GlobalAlloc`] itself. Doing so made this crate a second
//! allocator provider whose users bypassed `nros_platform_alloc`, the sole
//! allocation funnel of RFC-0034 D6. The C side still allocates from the arena
//! directly (the `z_malloc` shim above); the Rust side arrives via
//! `PlatformAlloc for <YourPlatform>`, which reaches the same static.
//!
//! All allocations are 8-byte aligned. Requested alignments greater than 8 are
//! not supported and yield a null pointer (the caller's `handle_alloc_error`
//! fires) — no nros runtime type needs more than 8-byte alignment.

#![no_std]

use core::{alloc::Layout, cell::UnsafeCell, ptr, ptr::NonNull};
// `portable_atomic`, not `core::sync::atomic`: the counters use `fetch_add`,
// and a target without atomic RMW (esp32-c3 is `riscv32imc`) has no such method
// on the core types. See the note in Cargo.toml.
use portable_atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering};
use rlsf::Tlsf;

/// phase-391 W2 — the second-level list length. 16 bounds internal
/// fragmentation at 1/SLLEN = 6.25%.
const SLLEN: usize = 16;

/// rlsf's `GRANULARITY.trailing_zeros()`: `GRANULARITY = size_of::<usize>() * 4`,
/// so 4 on a 32-bit target and 5 on 64-bit. Used only by the compile-time pool
/// bound below; rlsf computes its own internally.
const GRANULARITY_LOG2: u32 = (core::mem::size_of::<usize>() * 4).trailing_zeros();

/// First-level bitmap. Fixed at `u32` rather than tracked to `FLLEN`: it must
/// hold at least `FLLEN` bits, and letting the two drift is a foot-gun that
/// costs 2 bytes to remove. `u32` covers every `FLLEN` up to 32.
type FlBitmap = u32;

const ALIGN: usize = 8;
const HEADER_SIZE: usize = core::mem::size_of::<BlockHeader>();

/// Slab slot size in bytes. Allocations ≤ this size use the O(1) slab cache.
///
/// 64 bytes covers zenoh-pico's common string field allocations (topic names,
/// key expressions, type hashes) which are typically 20–50 bytes.
const SLAB_SLOT_SIZE: usize = 64;

/// Number of slab slots. 8 slots = 512 bytes total slab region.
///
/// zenoh-pico parses at most a few string fields per message, so 8 slots
/// provides ample headroom for concurrent short-lived allocations.
const SLAB_SLOT_COUNT: usize = 8;

/// Total slab region size in bytes.
const SLAB_REGION_SIZE: usize = SLAB_SLOT_SIZE * SLAB_SLOT_COUNT;

/// 8-byte-aligned byte-array wrapper for the heap / slab backing storage.
///
/// A bare `[u8; N]` has alignment 1, so the free-list base could land on an
/// odd address. Forcing the storage to 8-byte alignment makes every returned
/// pointer 8-aligned (headers and aligned sizes are 8-byte multiples), which
/// is what lets the platform's `#[global_allocator]` hand this memory to Rust
/// on strict-alignment targets. Strictly stronger than a bare `[u8; N]`.
#[repr(C, align(8))]
struct Aligned<const M: usize>([u8; M]);

/// Block header stored immediately before each allocation.
#[repr(C)]
struct BlockHeader {
    /// Usable region size (excludes this header).
    size: usize,
    /// Next free block (null when allocated or last in free list).
    next_free: *mut BlockHeader,
}

/// First-fit free-list allocator backed by a static `[u8; N]` heap,
/// with an O(1) slab fast-path for small allocations.
///
/// Single-threaded bare-metal only — uses `Relaxed` atomics for the free-list
/// head pointer and initialization flag. Not safe for multi-threaded use.
pub struct FreeListHeap<const N: usize, const FLLEN: usize = 18> {
    heap: UnsafeCell<Aligned<N>>,
    /// phase-391 W2 — the O(1) allocator. Was an `AtomicUsize` head of an
    /// address-ordered first-fit free list, whose walk had no worst-case bound.
    tlsf: UnsafeCell<Tlsf<'static, FlBitmap, u16, FLLEN, SLLEN>>,
    initialized: AtomicBool,
    /// Slab region: 8 slots × 64 bytes, separate from the main heap.
    slab: UnsafeCell<Aligned<SLAB_REGION_SIZE>>,
    /// Bitmap of USED slab slots (bit set = occupied). Starts 0 (all free).
    ///
    /// phase-391 W3 — the sense is inverted deliberately: with "set = free"
    /// this initialised to 0xFF, and ONE nonzero byte in an otherwise-zero
    /// struct places the ENTIRE static — arena included — in `.data` instead
    /// of `.bss`, costing the arena's full size in FLASH (measured: 67 KB on
    /// the Zephyr image, and every bare-metal port paid its arena size too).
    /// All-zero init keeps the static in `.bss`, where a heap belongs.
    slab_used_bitmap: AtomicU8,
    /// #190 — count of foreign-pointer `free`/`realloc` calls refused.
    foreign_frees: AtomicUsize,
    #[cfg(feature = "stats")]
    used_bytes: AtomicUsize,
    #[cfg(feature = "stats")]
    peak_bytes: AtomicUsize,
}

// Safety: bare-metal single-threaded. The AtomicUsize/AtomicBool provide
// interior mutability without `static mut`.
unsafe impl<const N: usize, const FLLEN: usize> Sync for FreeListHeap<N, FLLEN> {}

impl<const N: usize, const FLLEN: usize> Default for FreeListHeap<N, FLLEN> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize, const FLLEN: usize> FreeListHeap<N, FLLEN> {
    /// Create a new heap. Use in a `static`:
    /// ```rust,ignore
    /// static HEAP: FreeListHeap<{64 * 1024}> = FreeListHeap::new();
    /// ```
    /// The largest pool rlsf can hold at this `FLLEN`:
    /// `MAX_POOL_SIZE = 1 << (GRANULARITY_LOG2 + FLLEN)` (rlsf `tlsf.rs`).
    pub const MAX_POOL: usize = 1usize << (GRANULARITY_LOG2 + FLLEN as u32);

    pub const fn new() -> Self {
        // phase-391 W2 — a compile-time bound, not a comment. An arena larger
        // than rlsf's max pool would otherwise fail at the first insert, at
        // runtime, on the target. Raise FLLEN (each +1 doubles the bound and
        // costs ~66 B of .bss at SLLEN=16): 12 -> 64 KiB, 14 -> 256 KiB,
        // 18 -> 4 MiB.
        assert!(
            N <= Self::MAX_POOL,
            "zpico-alloc: heap size exceeds rlsf MAX_POOL_SIZE for this FLLEN — raise FLLEN"
        );
        Self {
            heap: UnsafeCell::new(Aligned([0u8; N])),
            tlsf: UnsafeCell::new(Tlsf::new()),
            initialized: AtomicBool::new(false),
            slab: UnsafeCell::new(Aligned([0u8; SLAB_REGION_SIZE])),
            slab_used_bitmap: AtomicU8::new(0), // all 8 slots free (set = used)
            foreign_frees: AtomicUsize::new(0),
            #[cfg(feature = "stats")]
            used_bytes: AtomicUsize::new(0),
            #[cfg(feature = "stats")]
            peak_bytes: AtomicUsize::new(0),
        }
    }

    /// Lazily initialize the free list with one block spanning the whole heap.
    #[inline]
    unsafe fn ensure_init(&self) {
        if !self.initialized.load(Ordering::Relaxed) {
            // phase-391 W2 — hand the whole arena to rlsf as one pool. The
            // arena itself is unchanged: same `Aligned<N>` storage, same
            // 8-byte alignment guarantee. Only who manages it changed.
            //
            // `insert_free_block_ptr` wants a `NonNull<[u8]>` over memory this
            // heap owns for `'static`; the arena is a field of a `static`, so
            // that holds. It returns the usable size, or `None` if the region
            // was too small to hold even one block — which `N` cannot be,
            // given the constructor's `MAX_POOL` bound and rlsf's own
            // granularity minimum.
            let base = self.heap.get() as *mut u8;
            let region = ptr::slice_from_raw_parts_mut(base, N);
            unsafe {
                let tlsf = &mut *self.tlsf.get();
                tlsf.insert_free_block_ptr(NonNull::new_unchecked(region));
            }
            self.initialized.store(true, Ordering::Relaxed);
        }
    }

    // ── Slab fast-path ─────────────────────────────────────────────────

    /// Base pointer of the slab region.
    #[inline]
    fn slab_base(&self) -> *mut u8 {
        self.slab.get() as *mut u8
    }

    /// Check if `ptr` points into the slab region.
    #[inline]
    fn is_in_slab(&self, ptr: *mut u8) -> bool {
        let base = self.slab_base() as usize;
        let addr = ptr as usize;
        addr >= base && addr < base + SLAB_REGION_SIZE
    }

    /// Check if `ptr` points into the main heap region (#190 guard).
    #[inline]
    fn is_in_heap(&self, ptr: *mut u8) -> bool {
        let base = self.heap.get() as usize;
        let addr = ptr as usize;
        addr >= base + HEADER_SIZE && addr < base + N
    }

    /// #190 — count of `free`/`realloc` calls that handed us a pointer
    /// OUTSIDE this heap's arena (a cross-allocator free). Diagnostic.
    pub fn foreign_free_count(&self) -> usize {
        self.foreign_frees.load(Ordering::Relaxed)
    }

    /// Try to allocate from the slab. Returns null if no free slot or size too large.
    #[inline]
    fn slab_alloc(&self, size: usize) -> *mut core::ffi::c_void {
        if size > SLAB_SLOT_SIZE {
            return ptr::null_mut();
        }

        let bitmap = self.slab_used_bitmap.load(Ordering::Relaxed);
        if bitmap == 0xFF {
            return ptr::null_mut(); // all slots occupied
        }

        // Find first CLEAR bit (first free slot) — O(1)
        let slot = (!bitmap).trailing_zeros() as usize;

        // Set the bit (mark as occupied)
        self.slab_used_bitmap
            .store(bitmap | (1 << slot), Ordering::Relaxed);

        #[cfg(feature = "stats")]
        {
            let used =
                self.used_bytes.fetch_add(SLAB_SLOT_SIZE, Ordering::Relaxed) + SLAB_SLOT_SIZE;
            let _ = self.peak_bytes.fetch_max(used, Ordering::Relaxed);
        }

        unsafe { self.slab_base().add(slot * SLAB_SLOT_SIZE) as *mut core::ffi::c_void }
    }

    /// Return a slab slot. Caller must verify `is_in_slab(ptr)` first.
    #[inline]
    fn slab_free(&self, ptr: *mut u8) {
        let offset = ptr as usize - self.slab_base() as usize;
        let slot = offset / SLAB_SLOT_SIZE;

        // Clear the bit (mark as free)
        let bitmap = self.slab_used_bitmap.load(Ordering::Relaxed);
        self.slab_used_bitmap
            .store(bitmap & !(1 << slot), Ordering::Relaxed);

        #[cfg(feature = "stats")]
        self.used_bytes.fetch_sub(SLAB_SLOT_SIZE, Ordering::Relaxed);
    }

    // ── Public API ─────────────────────────────────────────────────────

    /// Allocate `size` bytes (8-byte aligned). Returns null on failure.
    ///
    /// Allocations ≤ 64 bytes try the slab cache first (O(1)). Larger
    /// allocations use first-fit free-list search. Splits blocks when the
    /// remainder is large enough for another header + minimum allocation.
    pub fn alloc(&self, size: usize) -> *mut core::ffi::c_void {
        if size == 0 {
            return ptr::null_mut();
        }

        // Slab fast-path for small allocations. Kept: it is already O(1), so
        // it is not what phase-391 W2 set out to replace, and keeping it means
        // `capacity()`/`used()` keep their existing basis.
        if size <= SLAB_SLOT_SIZE {
            let ptr = self.slab_alloc(size);
            if !ptr.is_null() {
                return ptr;
            }
            // Slab full — fall through to the general allocator.
        }

        // phase-391 W2 — was a first-fit walk of an address-ordered free list,
        // O(n) in the number of free blocks and therefore unbounded in the
        // worst case. rlsf is O(1) for allocate and free regardless of heap
        // state (two-level segregated fit + bitmaps + a CLZ).
        unsafe {
            self.ensure_init();
            let layout = match Layout::from_size_align(size, ALIGN) {
                Ok(l) => l,
                Err(_) => return ptr::null_mut(),
            };
            let tlsf = &mut *self.tlsf.get();
            match tlsf.allocate(layout) {
                Some(p) => {
                    #[cfg(feature = "stats")]
                    {
                        // phase-412 -- charge the USABLE size, not the requested
                        // one. `free` has only the pointer, so the only size it
                        // can credit is what rlsf reports for the block; if alloc
                        // charged `size` and free credited the usable size (>= it,
                        // rounded to GRANULARITY) the counter would drift toward
                        // zero and stick there.
                        //
                        // Usable is also the honest figure: it is what the block
                        // costs the arena, which is what a heap-size knob covers.
                        let charged =
                            Tlsf::<'static, FlBitmap, u16, FLLEN, SLLEN>::allocation_usable_size(p);
                        let used = self.used_bytes.fetch_add(charged, Ordering::Relaxed) + charged;
                        let _ = self.peak_bytes.fetch_max(used, Ordering::Relaxed);
                    }
                    p.as_ptr() as *mut core::ffi::c_void
                }
                None => ptr::null_mut(),
            }
        }
    }

    /// Reallocate: alloc new block, copy old data, free old block.
    pub fn realloc(&self, old_ptr: *mut core::ffi::c_void, size: usize) -> *mut core::ffi::c_void {
        if old_ptr.is_null() {
            return self.alloc(size);
        }
        if size == 0 {
            self.free(old_ptr);
            return ptr::null_mut();
        }

        let new_ptr = self.alloc(size);
        if new_ptr.is_null() {
            return ptr::null_mut();
        }

        // #190 — foreign old pointer (not from this heap): there is no header
        // to read a size from, and `free` would corrupt live memory. Copy the
        // requested size from the foreign block (its true length is unknown
        // but ≥ what the caller is reallocating around) and leak it.
        if !self.is_in_slab(old_ptr as *mut u8) && !self.is_in_heap(old_ptr as *mut u8) {
            let n = self.foreign_frees.load(Ordering::Relaxed);
            self.foreign_frees.store(n + 1, Ordering::Relaxed);
            unsafe {
                ptr::copy_nonoverlapping(old_ptr as *const u8, new_ptr as *mut u8, size);
            }
            return new_ptr;
        }

        // phase-391 W2 follow-up — the heap branch used to read the OLD
        // free-list `BlockHeader` at `old_ptr - 8` for the size. Heap pointers
        // now come from rlsf, whose used-block header at that offset packs
        // size WITH flag bits in a different layout, so that read returned a
        // corrupted size and the copy ran out of bounds. Slab pointers are
        // still fixed-size and safe; heap pointers now go through rlsf's own
        // `reallocate`, which knows its own headers.
        if self.is_in_slab(old_ptr as *mut u8) {
            unsafe {
                let copy_size = if SLAB_SLOT_SIZE < size {
                    SLAB_SLOT_SIZE
                } else {
                    size
                };
                ptr::copy_nonoverlapping(old_ptr as *const u8, new_ptr as *mut u8, copy_size);
            }
            self.free(old_ptr);
            return new_ptr;
        }
        // Heap pointer: undo the speculative alloc above and let rlsf move the
        // block itself (it preserves the old block on failure).
        self.free(new_ptr);
        unsafe {
            self.ensure_init();
            let layout = match Layout::from_size_align(size, ALIGN) {
                Ok(l) => l,
                Err(_) => return ptr::null_mut(),
            };
            let tlsf = &mut *self.tlsf.get();
            let Some(nn) = core::ptr::NonNull::new(old_ptr as *mut u8) else {
                return ptr::null_mut();
            };
            match tlsf.reallocate(nn, layout) {
                Some(p) => p.as_ptr() as *mut core::ffi::c_void,
                None => ptr::null_mut(),
            }
        }
    }

    /// Return a block to the allocator.
    ///
    /// Slab pointers are returned to the slab bitmap (O(1)). Free-list
    /// pointers are inserted in address order with coalescing.
    pub fn free(&self, ptr: *mut core::ffi::c_void) {
        if ptr.is_null() {
            return;
        }

        // Slab fast-path (unchanged).
        if self.is_in_slab(ptr as *mut u8) {
            self.slab_free(ptr as *mut u8);
            return;
        }

        // #190 hardening, preserved verbatim in intent: refuse pointers
        // outside this heap's arena. A cross-allocator free (a Rust-global-heap
        // or static pointer handed to `z_free`) must never reach the allocator
        // — under the old free list it would have been spliced in as if
        // `ptr - 8` were a block header; under rlsf it would corrupt a block
        // header just the same. Dropping a foreign pointer leaks at worst.
        if !self.is_in_heap(ptr as *mut u8) {
            // issue 0851 — load+store, not `fetch_add`. This counter is
            // DIAGNOSTIC (`foreign_frees()` at line 233 is its only reader), and
            // the ESP32-C3 fixture target `riscv32imc-unknown-none-elf` has no
            // atomic CAS, so `fetch_add` does not exist there. The sibling
            // increment in `realloc` already uses exactly this idiom; this site
            // was the one that did not, and it broke the whole esp32 fixture
            // lane. A lost increment under contention costs an inaccurate
            // diagnostic, which is the right trade for a counter nothing
            // allocates against.
            let n = self.foreign_frees.load(Ordering::Relaxed);
            self.foreign_frees.store(n + 1, Ordering::Relaxed);
            return;
        }

        // phase-391 W2 — O(1) free. `deallocate` needs the alignment the block
        // was allocated with; this heap allocates everything at ALIGN.
        unsafe {
            self.ensure_init();
            let tlsf = &mut *self.tlsf.get();
            if let Some(nn) = NonNull::new(ptr as *mut u8) {
                // phase-412 -- BEFORE `deallocate`, which invalidates the block
                // header the size is read from. Until this existed the rlsf path
                // credited nothing, so `used()` was cumulative-allocated rather
                // than live and `peak()` merely tracked it: the island reported
                // 395,132 used against a 94,720-byte arena.
                #[cfg(feature = "stats")]
                {
                    let freed =
                        Tlsf::<'static, FlBitmap, u16, FLLEN, SLLEN>::allocation_usable_size(nn);
                    // Saturating, not `fetch_sub`: an unmatched free would wrap,
                    // and a counter stuck at zero is wrong in a way a reader can
                    // see, where an enormous one looks like a measurement.
                    let _ =
                        self.used_bytes
                            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |u| {
                                Some(u.saturating_sub(freed))
                            });
                }
                tlsf.deallocate(nn, ALIGN);
            }
        }
    }

    /// Total managed heap capacity in bytes (main free-list region +
    /// slab region). The denominator that pairs with [`used`](Self::used)
    /// for a "used / total" footprint figure — same basis as
    /// [`free_bytes`](Self::free_bytes). Always available (no atomics).
    pub const fn capacity(&self) -> usize {
        N + SLAB_REGION_SIZE
    }

    /// Current heap usage in bytes (slab + free-list, including headers).
    ///
    /// Only available with the `stats` feature.
    #[cfg(feature = "stats")]
    pub fn used(&self) -> usize {
        self.used_bytes.load(Ordering::Relaxed)
    }

    /// Peak heap usage in bytes since boot.
    ///
    /// Only available with the `stats` feature.
    #[cfg(feature = "stats")]
    pub fn peak(&self) -> usize {
        self.peak_bytes.load(Ordering::Relaxed)
    }

    /// Free bytes remaining (approximate — does not account for fragmentation).
    ///
    /// Only available with the `stats` feature.
    #[cfg(feature = "stats")]
    pub fn free_bytes(&self) -> usize {
        (N + SLAB_REGION_SIZE).saturating_sub(self.used_bytes.load(Ordering::Relaxed))
    }
}

// ============================================================================
// No `GlobalAlloc` impl here — phase-361 W8.c / issue 0594
// ============================================================================
//
// `FreeListHeap` used to implement `GlobalAlloc` behind a `global-alloc`
// feature so a bare-metal board could install it as the Rust
// `#[global_allocator]` directly. That made this crate a second allocator
// provider, and its one consumer reached the heap without passing through
// `nros_platform_alloc` — the sole allocation funnel RFC-0034 D6 defines.
//
// The heap is still the board's ARENA. The Rust side reaches it through the
// platform's `PlatformAlloc` impl, and `nros-platform` owns the single
// `#[global_allocator]` in the tree. The alignment contract that lived on the
// deleted impl now lives there: allocations are 8-byte aligned, and a request
// for more is answered with null so the caller's `handle_alloc_error` fires
// rather than receiving misaligned memory.

// ============================================================================
// Reusable heap-usage counter (stats feature)
// ============================================================================

/// Lightweight `used`/`peak` byte counter for instrumenting an allocator.
///
/// Mirrors the `used_bytes`/`peak_bytes` tracking [`FreeListHeap`] keeps under
/// the `stats` feature, but as a standalone unit so the RTOS allocator
/// wrappers (FreeRTOS `pvPortMalloc`, ThreadX `tx_byte_allocate`, Zephyr
/// `k_malloc`) in `nros-c` / `nros-cpp` can expose the same visibility into the
/// Rust global allocator's footprint.
///
/// Single-threaded bare-metal / RTOS use: `Relaxed` atomics. `AtomicUsize` is
/// already `Sync`, so this can live in a `static`.
#[cfg(feature = "stats")]
pub struct HeapStats {
    used_bytes: AtomicUsize,
    peak_bytes: AtomicUsize,
}

#[cfg(feature = "stats")]
impl Default for HeapStats {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "stats")]
impl HeapStats {
    /// Create a zeroed counter. `const` so it can be a `static`.
    pub const fn new() -> Self {
        Self {
            used_bytes: AtomicUsize::new(0),
            peak_bytes: AtomicUsize::new(0),
        }
    }

    /// Record a successful allocation of `size` bytes and update the peak.
    #[inline]
    pub fn on_alloc(&self, size: usize) {
        let used = self.used_bytes.fetch_add(size, Ordering::Relaxed) + size;
        let _ = self.peak_bytes.fetch_max(used, Ordering::Relaxed);
    }

    /// Record a deallocation of `size` bytes.
    #[inline]
    pub fn on_dealloc(&self, size: usize) {
        self.used_bytes.fetch_sub(size, Ordering::Relaxed);
    }

    /// Bytes currently outstanding.
    #[inline]
    pub fn used(&self) -> usize {
        self.used_bytes.load(Ordering::Relaxed)
    }

    /// Peak outstanding bytes since boot.
    #[inline]
    pub fn peak(&self) -> usize {
        self.peak_bytes.load(Ordering::Relaxed)
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slab_alloc_and_free() {
        let heap: FreeListHeap<1024> = FreeListHeap::new();

        // Allocate a small buffer — should hit slab
        let p1 = heap.alloc(32);
        assert!(!p1.is_null());
        assert!(heap.is_in_slab(p1 as *mut u8));

        // Free and reallocate — should reuse same slot
        heap.free(p1);
        let p2 = heap.alloc(32);
        assert!(!p2.is_null());
        assert!(heap.is_in_slab(p2 as *mut u8));
        assert_eq!(p1, p2); // same slot reused (first free bit)
        heap.free(p2);
    }

    #[test]
    fn slab_exhaustion_falls_through() {
        let heap: FreeListHeap<4096> = FreeListHeap::new();

        // Fill all 8 slab slots
        let mut ptrs = [ptr::null_mut(); SLAB_SLOT_COUNT];
        for p in &mut ptrs {
            *p = heap.alloc(16);
            assert!(!p.is_null());
            assert!(heap.is_in_slab(*p as *mut u8));
        }

        // 9th small alloc falls through to free-list
        let overflow = heap.alloc(16);
        assert!(!overflow.is_null());
        assert!(!heap.is_in_slab(overflow as *mut u8));

        // Free all
        for p in &ptrs {
            heap.free(*p);
        }
        heap.free(overflow);
    }

    #[test]
    fn large_alloc_skips_slab() {
        let heap: FreeListHeap<4096> = FreeListHeap::new();

        // > SLAB_SLOT_SIZE goes directly to free-list
        let p = heap.alloc(128);
        assert!(!p.is_null());
        assert!(!heap.is_in_slab(p as *mut u8));
        heap.free(p);
    }

    #[test]
    fn realloc_slab_to_freelist() {
        let heap: FreeListHeap<4096> = FreeListHeap::new();

        // Small alloc → slab
        let p1 = heap.alloc(32);
        assert!(heap.is_in_slab(p1 as *mut u8));

        // Write data
        unsafe {
            ptr::write_bytes(p1 as *mut u8, 0xAB, 32);
        }

        // Realloc to larger → moves to free-list, copies data
        let p2 = heap.realloc(p1, 128);
        assert!(!p2.is_null());
        assert!(!heap.is_in_slab(p2 as *mut u8));

        // Verify data was copied
        unsafe {
            let slice = core::slice::from_raw_parts(p2 as *const u8, 32);
            assert!(slice.iter().all(|&b| b == 0xAB));
        }

        heap.free(p2);
    }

    #[test]
    fn zero_size_returns_null() {
        let heap: FreeListHeap<1024> = FreeListHeap::new();
        assert!(heap.alloc(0).is_null());
    }

    #[test]
    fn free_null_is_noop() {
        let heap: FreeListHeap<1024> = FreeListHeap::new();
        heap.free(ptr::null_mut()); // should not panic
    }

    #[test]
    fn freelist_coalescing() {
        let heap: FreeListHeap<4096> = FreeListHeap::new();

        // Allocate three adjacent blocks (> SLAB_SLOT_SIZE to skip slab)
        let p1 = heap.alloc(128);
        let p2 = heap.alloc(128);
        let p3 = heap.alloc(128);
        assert!(!p1.is_null());
        assert!(!p2.is_null());
        assert!(!p3.is_null());

        // Free middle, then neighbours — should coalesce
        heap.free(p2);
        heap.free(p1);
        heap.free(p3);

        // Should be able to allocate a large block from coalesced region
        let big = heap.alloc(384);
        assert!(!big.is_null());
        heap.free(big);
    }

    #[test]
    fn allocations_are_8_byte_aligned() {
        let heap: FreeListHeap<4096> = FreeListHeap::new();
        // Slab path
        let p1 = heap.alloc(16);
        assert_eq!(p1 as usize % ALIGN, 0);
        // Free-list path
        let p2 = heap.alloc(200);
        assert_eq!(p2 as usize % ALIGN, 0);
        heap.free(p1);
        heap.free(p2);
    }

    #[cfg(feature = "stats")]
    #[test]
    fn heap_stats_counter() {
        let stats = HeapStats::new();
        assert_eq!(stats.used(), 0);
        assert_eq!(stats.peak(), 0);

        stats.on_alloc(100);
        stats.on_alloc(50);
        assert_eq!(stats.used(), 150);
        assert_eq!(stats.peak(), 150);

        stats.on_dealloc(100);
        assert_eq!(stats.used(), 50);
        assert_eq!(stats.peak(), 150); // peak is sticky

        stats.on_alloc(200);
        assert_eq!(stats.used(), 250);
        assert_eq!(stats.peak(), 250);
    }

    #[cfg(feature = "stats")]
    #[test]
    fn stats_track_slab_and_freelist() {
        let heap: FreeListHeap<4096> = FreeListHeap::new();

        assert_eq!(heap.used(), 0);

        // Slab alloc
        let p1 = heap.alloc(32);
        assert_eq!(heap.used(), SLAB_SLOT_SIZE); // slab charges full slot

        // Free-list alloc
        let p2 = heap.alloc(128);
        let used_after_both = heap.used();
        assert!(used_after_both > SLAB_SLOT_SIZE);

        // Free both
        heap.free(p1);
        heap.free(p2);
        assert_eq!(heap.used(), 0);

        // Peak should reflect the maximum
        assert!(heap.peak() >= used_after_both);
    }

    // phase-391 W2 follow-up — realloc used to read the retired free-list
    // BlockHeader under rlsf-owned pointers (an OOB read; not deterministically
    // assertable without miri, so these pin the REPLACEMENT's contract).
    #[test]
    fn heap_realloc_grow_preserves_content() {
        static H: FreeListHeap<{ 32 * 1024 }> = FreeListHeap::new();
        let p = H.alloc(200) as *mut u8; // > SLAB_SLOT_SIZE -> heap
        assert!(!p.is_null());
        for i in 0..200u32 {
            unsafe { *p.add(i as usize) = (i % 251) as u8 };
        }
        let q = H.realloc(p as *mut core::ffi::c_void, 1000) as *mut u8;
        assert!(!q.is_null());
        for i in 0..200u32 {
            assert_eq!(unsafe { *q.add(i as usize) }, (i % 251) as u8, "byte {i}");
        }
        H.free(q as *mut core::ffi::c_void);
    }

    #[test]
    fn heap_realloc_shrink_then_reuse() {
        static H: FreeListHeap<{ 32 * 1024 }> = FreeListHeap::new();
        let p = H.alloc(1000);
        assert!(!p.is_null());
        let q = H.realloc(p, 100);
        assert!(!q.is_null());
        // The allocator must still be coherent afterwards: a fresh large
        // alloc succeeds and does not alias the live block.
        let r = H.alloc(2000);
        assert!(!r.is_null());
        assert_ne!(q, r);
        H.free(q);
        H.free(r);
    }

    #[test]
    fn slab_realloc_promotes_to_heap_with_content() {
        static H: FreeListHeap<{ 32 * 1024 }> = FreeListHeap::new();
        let p = H.alloc(32) as *mut u8; // slab
        for i in 0..32u8 {
            unsafe { *p.add(i as usize) = i };
        }
        let q = H.realloc(p as *mut core::ffi::c_void, 500) as *mut u8; // heap
        assert!(!q.is_null());
        for i in 0..32u8 {
            assert_eq!(unsafe { *q.add(i as usize) }, i);
        }
        H.free(q as *mut core::ffi::c_void);
    }
}

#[cfg(all(test, feature = "stats"))]
mod stats_accounting_tests {
    use super::*;

    /// phase-412 -- `used()` must come back DOWN.
    ///
    /// The rlsf free path credited nothing, so `used()` was
    /// cumulative-allocated: the island reported 395,132 used against a
    /// 94,720-byte arena, and `peak()` merely tracked it. A counter that only
    /// grows cannot size a heap knob, which is the whole reason it exists.
    #[test]
    fn freeing_returns_the_bytes_to_the_counter() {
        static H: FreeListHeap<8192> = FreeListHeap::new();
        let base = H.used();

        // Larger than SLAB_SLOT_SIZE, so this is the rlsf path rather than the
        // slab fast-path, which was already symmetric.
        let p = H.alloc(1024);
        assert!(!p.is_null());
        let after_alloc = H.used();
        assert!(
            after_alloc >= base + 1024,
            "alloc charged {} for a 1024-byte request",
            after_alloc - base
        );

        H.free(p);
        assert_eq!(H.used(), base, "free did not credit what alloc charged");
    }

    /// Churn must not drift the counter in either direction.
    #[test]
    fn repeated_churn_does_not_drift() {
        static H: FreeListHeap<8192> = FreeListHeap::new();
        let base = H.used();
        for _ in 0..64 {
            let p = H.alloc(512);
            assert!(!p.is_null());
            H.free(p);
        }
        assert_eq!(H.used(), base, "64 alloc/free pairs drifted the counter");
    }

    /// Sizes that are not multiples of GRANULARITY are where an asymmetric
    /// charge/credit shows up fastest.
    #[test]
    fn odd_sizes_round_trip() {
        static H: FreeListHeap<16384> = FreeListHeap::new();
        let base = H.used();
        let mut ps = [core::ptr::null_mut(); 8];
        for (i, p) in ps.iter_mut().enumerate() {
            *p = H.alloc(65 + i * 37);
            assert!(!p.is_null());
        }
        for p in ps {
            H.free(p);
        }
        assert_eq!(H.used(), base, "odd-sized blocks drifted the counter");
    }
}
