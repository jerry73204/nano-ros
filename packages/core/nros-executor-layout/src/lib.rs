//! The executor backing's placement arithmetic — issue 1197.
//!
//! # Why this is its own crate
//!
//! phase-392 W6 moved the executor's per-entry storage into a `.bss` static. On
//! an RTOS the allocator reservation it used to come out of is itself fixed, so
//! whatever sizes that reservation has to know how big the backing is.
//!
//! The size is `arena + tables`. The arena is plain arithmetic
//! `nros-node/build.rs` already computes. The tables are `repr(Rust)` layout —
//! field order, padding and niche packing are the compiler's to choose and to
//! CHANGE between versions — so they can never be re-derived outside it. That is
//! the sizes-header mirror class (0088 -> 0114 -> 0122 -> 0123 -> 0245 -> 0268)
//! this tree has re-fixed six times.
//!
//! So the arithmetic takes the per-type facts as DATA. `nros-node` supplies what
//! its compiler says while building for the target; a build script supplies
//! units recovered for a target it is not itself compiling (the way `nros-c`
//! recovers `EXECUTOR_OPAQUE_U64S` through `nros-sizes-build`). One
//! implementation, two sources of the inputs.
//!
//! It is a separate crate rather than a `pub fn` in `nros-node` for one reason:
//! a build script runs on the HOST, and a build-dependency on `nros-node` would
//! give it a host-compiled `nros-node` — the wrong layout, silently. This crate
//! is `no_std` and dependency-free so both sides can reach it.
//!
//! # What it does NOT decide
//!
//! The counts. `cbs`/`sc`/`nodes`/`arena` come from `nros-node/build.rs` and,
//! on a synced leaf, from the derived env `nros sync` writes (issues 0827,
//! 1061). This crate only places regions.

#![no_std]
#![forbid(unsafe_code)]

/// Size and alignment of one region's element type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RegionUnit {
    /// `size_of::<T>()`.
    pub size: usize,
    /// `align_of::<T>()`. Must be a power of two; zero is rejected as 1.
    pub align: usize,
}

/// Every element type the backing carves a table of, in declaration order.
///
/// The arena is absent on purpose: it is `[MaybeUninit<u8>]`, size 1 align 1 by
/// definition, and its LENGTH is a count rather than a unit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RegionUnits {
    pub callback_meta: RegionUnit,
    pub sched_context: RegionUnit,
    pub sched_context_id: RegionUnit,
    pub sporadic_state: RegionUnit,
    /// Placed only when [`Counts::alloc`] is set.
    pub sporadic_atomic: RegionUnit,
    pub remap_rule: RegionUnit,
    pub node_record: RegionUnit,
    pub concrete_session: RegionUnit,
    pub extra_session_id: RegionUnit,
    pub node_sched_entry: RegionUnit,
    pub dispatch_slot: RegionUnit,
    pub component_slot: RegionUnit,
    pub group_name: RegionUnit,
    pub group_sched_entry: RegionUnit,
    pub violation: RegionUnit,
}

/// How many of each region the image needs.
///
/// `remaps` and `violations` are fixed counts in `nros-node` rather than knobs;
/// they are parameters here so this crate states no policy.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Counts {
    pub cbs: usize,
    pub sc: usize,
    pub nodes: usize,
    pub arena: usize,
    pub remaps: usize,
    pub violations: usize,
    /// `nros-node`'s `alloc` feature — it adds the sporadic-atomic table, so it
    /// changes the SIZE and cannot be inferred from the counts.
    pub alloc: bool,
}

/// Byte offset of each region, plus the total size and alignment.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Offsets {
    pub arena: usize,
    pub entries: usize,
    pub sched_contexts: usize,
    pub sched_context_bindings: usize,
    pub sporadic_states: usize,
    pub sporadic_atomic_states: usize,
    pub remaps: usize,
    pub nodes: usize,
    pub extra_sessions: usize,
    pub extra_session_ids: usize,
    pub node_sched_table: usize,
    pub dispatch_slots: usize,
    pub component_slots: usize,
    pub active_groups: usize,
    pub group_sched_table: usize,
    pub monitor_violations: usize,
    pub size: usize,
    pub align: usize,
}

const fn align_up(off: usize, align: usize) -> usize {
    off.div_ceil(align) * align
}

/// Place every region and report the offsets, total size and alignment.
///
/// Reproduces `#[repr(C)]` declaration-order layout: the arena first at offset
/// 0, then each table aligned up to its own element alignment, and the total
/// rounded up to the maximum alignment seen.
pub const fn offsets(counts: Counts, units: RegionUnits) -> Offsets {
    let Counts {
        cbs,
        sc,
        nodes: node_slots,
        arena,
        remaps: remap_slots,
        violations: violation_slots,
        alloc,
    } = counts;

    let mut off = 0usize;
    let mut max_align = 1usize;

    // arena: [MaybeUninit<u8>; arena] — align 1, at offset 0.
    let arena_off = 0usize;
    off += arena;

    macro_rules! place {
        ($n:expr, $u:expr) => {{
            let RegionUnit { size, align } = $u;
            // A zero alignment is not a layout any compiler produces; treating
            // it as 1 keeps this total rather than panicking in a `const fn`,
            // and a caller that supplies one gets an answer it can compare.
            let a = if align == 0 { 1 } else { align };
            if a > max_align {
                max_align = a;
            }
            off = align_up(off, a);
            let at = off;
            off += $n * size;
            at
        }};
    }

    let entries = place!(cbs, units.callback_meta);
    let sched_contexts = place!(sc, units.sched_context);
    let sched_context_bindings = place!(cbs, units.sched_context_id);
    let sporadic_states = place!(sc, units.sporadic_state);
    // Placed only under `alloc`, and the `else` must not move `off` — the
    // no-alloc layout is the one every bare-metal image links.
    let sporadic_atomic_states = if alloc {
        place!(sc, units.sporadic_atomic)
    } else {
        off
    };
    let remaps = place!(remap_slots, units.remap_rule);
    let nodes = place!(node_slots, units.node_record);
    let extra_sessions = place!(node_slots, units.concrete_session);
    let extra_session_ids = place!(node_slots, units.extra_session_id);
    let node_sched_table = place!(node_slots, units.node_sched_entry);
    let dispatch_slots = place!(node_slots, units.dispatch_slot);
    let component_slots = place!(node_slots, units.component_slot);
    let active_groups = place!(node_slots, units.group_name);
    let group_sched_table = place!(cbs, units.group_sched_entry);
    let monitor_violations = place!(violation_slots, units.violation);

    Offsets {
        arena: arena_off,
        entries,
        sched_contexts,
        sched_context_bindings,
        sporadic_states,
        sporadic_atomic_states,
        remaps,
        nodes,
        extra_sessions,
        extra_session_ids,
        node_sched_table,
        dispatch_slots,
        component_slots,
        active_groups,
        group_sched_table,
        monitor_violations,
        size: align_up(off, max_align),
        align: max_align,
    }
}

/// The backing's total size in bytes — what a reservation has to hold.
pub const fn size_of_backing(counts: Counts, units: RegionUnits) -> usize {
    offsets(counts, units).size
}
