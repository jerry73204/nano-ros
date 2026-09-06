//! phase-392 W6 — the executor's storage as a NAMED STATIC (RFC-0002 § 4.4b).
//!
//! # What this fixes, and what it does not
//!
//! Phase 392 prices an image's static RAM by reading its symbol table
//! (`just mem-report`). Every later wave of that campaign is defined as a saving
//! against those numbers, so a consumer the symbol table cannot see is a
//! consumer the campaign cannot price — and the executor's storage was exactly
//! that on every Rust image.
//!
//! **Not because the arena is inline.** It stopped being an inline
//! `[MaybeUninit<u8>; ARENA_SIZE]` field in phase-271 (issue 0110), and
//! [`Executor`](super::spin::Executor) has held `arena: &'s mut
//! [MaybeUninit<u8>]` — a slice carved out of caller-supplied backing — ever
//! since. Four documents, a build-script comment and a runtime advisory string
//! went on asserting the inline shape for five phases; phase-403 caught the
//! claim and phase-392 W6 corrected the sites.
//!
//! **Where that backing lives is the caller's choice**, and before this module
//! the tree answered it three different ways:
//!
//! * **C** — all 34 in-tree `nros_executor_t` objects are file-scope `static`,
//!   so the backing is carved from `_opaque` in `.bss`. Visible.
//! * **C++** — `nros::Node::GlobalStorageHolder<0>::storage`, a template static
//!   member, so also `.bss`. Visible.
//! * **Rust** — every board entry (`linux`, `zephyr`, `freertos`, `nuttx`,
//!   `threadx`, `esp32-qemu`, `mps2-an385`) reaches an `alloc` convenience
//!   constructor, which leaked a `Box`. **Invisible**: a heap allocation has no
//!   symbol. Measured on the native zenoh talker before this module: the largest
//!   `nros_node` RAM symbol in the whole image was **1 byte**.
//!
//! This module closes the Rust arm — one change, every Rust board.
//!
//! # It is a MOVE, not a saving, and the second half is the caller's
//!
//! The same bytes are reserved either way; what changes is *which budget pays*.
//! A leaked `Box` is drawn from the image's allocator arena. On a hosted target
//! that arena is the OS heap and has no fixed reservation, so the static costs
//! nothing that was not already spent. **On an RTOS it is itself a fixed
//! static** — `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE` on Zephyr,
//! `configTOTAL_HEAP_SIZE` on FreeRTOS — already sized to hold this backing, so
//! turning the reservation on there without lowering that knob reserves the same
//! bytes twice.
//!
//! That pairing is per-image and only the image's author can measure it, so the
//! opt-out is a knob rather than a guess: `NROS_EXECUTOR_BACKING_U64S=0` emits
//! no static at all and restores the leak. A non-zero value overrides the
//! reservation's SIZE, which is also how a fat entry — one declaring
//! `max_callbacks` above `NROS_EXECUTOR_MAX_CBS` — keeps a static instead of
//! falling back to the heap.
//!
//! **The Zephyr `CONFIG_` spelling is NOT wired yet.** `nros-node`'s build
//! script resolves this through the same env → `$DOTCONFIG` reader every other
//! executor knob uses, so `CONFIG_NROS_EXECUTOR_BACKING_U64S` would be honoured
//! the moment `zephyr/Kconfig` declares it — but it does not, and a symbol no
//! Kconfig declares never reaches the dotconfig. Declaring it belongs with the
//! measurement that makes it necessary (issue 1145), not ahead of it: the
//! natural `int … default 0` would mean "no static" on every Zephyr image the
//! day it landed, which is the opposite of the default this chose. Until then
//! the env knob is the whole interface.
//!
//! # Why there is no `// nros-pool:` annotation
//!
//! `scripts/gen-pool-inventory.py` evaluates a pool as a PRODUCT of knobs at
//! their literal defaults. This one is a SUM — the executor's carved tables plus
//! the arena, laid out with alignment padding between them — and its largest
//! term, `ARENA_SIZE`, is itself derived
//! (`env_usize("NROS_EXECUTOR_ARENA_SIZE", derived_arena)` in
//! `nros-node/build.rs`), so the inventory would record it as a computed default
//! even if the sum were expressible. A formula that is right for one build and
//! wrong for the rest is the drift class this tree gates against, so this
//! follows the two documented deliberate non-annotations
//! (`nros_rmw_zenoh::shim::publisher`, `nros_rmw_cffi`): **the size is known to
//! the compiler, so read it from the compiler's output.** `mem-report` prices
//! the symbol from the ELF, exactly, with no formula to drift.
//!
//! # Placement
//!
//! `NROS_EXECUTOR_BACKING_SECTION` (build-time env, read by `nros-node`'s build
//! script) puts the static in a named section, for the amendment-A case where a
//! part has tightly-coupled memory the linker script can target — issue 0880
//! does the same for `nros_thread_stacks` with `DTCM`. Two constraints, both
//! load-bearing: **the section must be `NOLOAD`**, because this static is
//! uninitialised and a loadable section would add its whole size to the image's
//! flash footprint; and it must be reachable by every bus master that touches
//! the executor's buffers, which on Cortex-M7 the TCMs typically are not.

use core::mem::MaybeUninit;

#[cfg(nros_executor_backing_static)]
use portable_atomic::{AtomicBool, Ordering};

use super::storage::ExecutorSizing;

/// The reservation's size when nothing overrides it: one default-sized
/// executor's worth.
///
/// One executor, not a multiple, because an entry opens ONE: the tiered boot
/// paths open a second per tier, and those correctly fall through to the heap
/// (see [`take`]).
///
/// Named by the GENERATED file when nothing overrides the size, so it is unused
/// under `NROS_EXECUTOR_BACKING_U64S=0` (no static) and under an explicit
/// override (a literal). Kept rather than `cfg`'d: it is what the override is
/// judged against, and a constant that disappears with its consumer cannot be
/// compared to anything.
#[allow(dead_code)]
pub(crate) const EXECUTOR_BACKING_DEFAULT_U64S: usize = ExecutorSizing::DEFAULT.u64_len();

// `EXECUTOR_BACKING`, its size const, and the optional
// `#[unsafe(link_section = …)]` on it are emitted by `build.rs`: `link_section`
// takes a string LITERAL and the section name is a build-time input, and the
// size may be overridden or the whole item suppressed. The file is EMPTY when
// `NROS_EXECUTOR_BACKING_U64S=0`, which is why the include is unconditional
// while everything that reads it is `cfg`-gated.
include!(concat!(env!("OUT_DIR"), "/nros_executor_backing.rs"));

/// The reservation must cover the sizing every `alloc` convenience constructor
/// passes, or it is dead weight every image carries and no executor ever uses.
///
/// A `const` assertion rather than a test, deliberately: the only way to make it
/// false is to set `NROS_EXECUTOR_BACKING_U64S` too low, and the person doing
/// that wants to hear about it from the build they just ran, not from a test
/// suite they may not run at all. `0` is the documented opt-out and is not
/// reached here — it emits no static, so this whole arm is `cfg`'d away.
#[cfg(nros_executor_backing_static)]
const _: () = assert!(
    EXECUTOR_BACKING_U64S >= EXECUTOR_BACKING_DEFAULT_U64S,
    "NROS_EXECUTOR_BACKING_U64S is below the default executor sizing, so the \
     reservation can never be taken and is pure dead weight; use 0 to decline \
     the static entirely"
);

/// Has [`EXECUTOR_BACKING`] been handed out?
///
/// A latch, not a free list: the backing is handed out as `&'static mut` and
/// never returned, exactly like the `Box::leak` it replaces. An executor that
/// drops does not give it back — reclaiming it would need the executor's own
/// tables to be provably dead first, which `Drop` cannot show for a slice it has
/// already lent to `Dispatcher`.
#[cfg(nros_executor_backing_static)]
static TAKEN: AtomicBool = AtomicBool::new(false);

/// Hand out [`EXECUTOR_BACKING`], once, if it can hold `words`.
///
/// `None` means "use the heap", and every path to it is legitimate rather than
/// degraded — it is the caller's behaviour from before this module existed:
///
/// * the reservation is switched off (`NROS_EXECUTOR_BACKING_U64S=0`);
/// * another executor already took it (a tiered boot opens one per tier);
/// * this executor is sized past the reservation (a fat entry).
///
/// The `swap` is what makes the returned `&'static mut` sound: exactly one
/// caller observes `false`, so exactly one reference to the static ever exists.
#[cfg(nros_executor_backing_static)]
pub(crate) fn take(words: usize) -> Option<&'static mut [MaybeUninit<u64>]> {
    if words > EXECUTOR_BACKING_U64S {
        return None;
    }
    if TAKEN.swap(true, Ordering::AcqRel) {
        return None;
    }
    // SAFETY: the swap above succeeds exactly once for the life of the process,
    // so this is the only reference ever created to `EXECUTOR_BACKING`, and it
    // is `'static` because the static is. `words <= EXECUTOR_BACKING_U64S` was
    // checked, so the slice is in bounds.
    // The raw ref is bound first rather than dereferenced in place: `&mut
    // *(&raw mut X)` reads as `deref_addrof` to clippy, and the obvious
    // rewrite it suggests (`&mut X`) is the `static_mut_refs` hazard this
    // spelling exists to avoid.
    let ptr = &raw mut EXECUTOR_BACKING;
    let all: &'static mut [MaybeUninit<u64>; EXECUTOR_BACKING_U64S] = unsafe { &mut *ptr };
    Some(&mut all[..words])
}

/// The reservation is switched off, so there is nothing to hand out.
///
/// A stub rather than a `cfg` at the call site: `default_backing` reads the same
/// either way, which is the paired-stub idiom `trace_register` uses one file
/// over, and it keeps the `#[cfg]` in exactly one place.
#[cfg(not(nros_executor_backing_static))]
pub(crate) fn take(_words: usize) -> Option<&'static mut [MaybeUninit<u64>]> {
    None
}

/// Has the static been claimed? Tests only.
#[cfg(all(test, nros_executor_backing_static))]
pub(crate) fn is_taken() -> bool {
    TAKEN.load(Ordering::Acquire)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The POSITIVE CONTROL, and the reason it and the two `None` cases share
    /// one test: `TAKEN` is process-scoped by design, so a second test would
    /// observe an already-consumed latch and assert nothing — the vacuous shape
    /// `check-no-vacuous-tests` exists to catch.
    ///
    /// A test that only ever saw `take` return `None` would pass against a `take`
    /// that can never succeed, which is exactly the bug that would make this
    /// whole module inert while reading as if it worked.
    #[cfg(nros_executor_backing_static)]
    #[test]
    fn the_static_is_handed_out_once_and_only_once() {
        // Too large for the reservation: refused BEFORE the latch is touched, so
        // this case cannot consume the one handout.
        assert!(
            take(EXECUTOR_BACKING_U64S + 1).is_none(),
            "a request past the reservation must fall back to the heap"
        );
        assert!(
            !is_taken(),
            "an over-large request must not claim the latch"
        );

        let first = take(EXECUTOR_BACKING_U64S).expect("the first taker gets the static");
        assert_eq!(
            first.len(),
            EXECUTOR_BACKING_U64S,
            "the handout is the requested length, not the whole reservation"
        );
        assert!(is_taken());

        assert!(
            take(1).is_none(),
            "a second taker must fall back to the heap — two `&'static mut` to \
             one static is UB, which is the whole reason for the latch"
        );
    }

    /// With the reservation off there is no static, so `take` is inert and no
    /// image carries the bytes.
    #[cfg(not(nros_executor_backing_static))]
    #[test]
    fn the_opt_out_removes_the_reservation_entirely() {
        assert!(
            take(1).is_none(),
            "NROS_EXECUTOR_BACKING_U64S=0 must hand out nothing"
        );
    }
}
