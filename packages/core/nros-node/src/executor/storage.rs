//! phase-271 / phase-409 — per-entry [`Executor`](super::spin::Executor)
//! storage (issues 0110, 0563, 0936).
//!
//! The executor's sized tables (callback table + arena + scheduling-context
//! tables + the node/session/dispatch tables) used to be inline fields sized by
//! build-time consts baked into `nros-node` — one size for every entry sharing a
//! compiled `nros-node`, and, worse, a size that every function MOVING an
//! `Executor` had to make room for on its stack. Here the ENTRY supplies its own
//! storage, sized to its topology, so a fat native entry and a lean embedded
//! entry in one workspace each get the right size with no workspace-global env,
//! and the `Executor` VALUE stays a small fixed header.
//!
//! Per the "C/C++ is a thin wrapper of Rust" principle the PUBLIC API stays
//! generic-free: the entry hands a raw, 8-aligned `&mut [MaybeUninit<u64>]` backing
//! (sized via [`executor_storage_u64_len`]); `nros-node` carves it privately into
//! the typed sub-slices ([`carve`]). The only `unsafe` is that carve, validated
//! against the `#[repr(C)]` reference [`ExecutorStorage`] layout by unit test.

use core::{
    alloc::Layout,
    mem::{MaybeUninit, align_of, needs_drop, size_of},
    ops::{Deref, DerefMut},
};

use super::{
    arena::CallbackMeta,
    monitor::{MAX_VIOLATIONS, Violation},
    node_record::NodeRecord,
    sched_context::{SchedContext, SchedContextId, SporadicState},
    spin::{ComponentSlot, DispatchSlot, MAX_REMAPS, RemapRule},
};
use crate::session::ConcreteSession;

#[cfg(feature = "alloc")]
type SporadicAtomic = (
    portable_atomic_util::Arc<super::sched_context::AtomicSporadicState>,
    super::spin::OpaqueTimerHandle,
);

/// phase-272 — `(node name, namespace, sched context)`, keyed by the node's
/// fully-qualified identity.
pub(crate) type NodeSchedEntry = (heapless::String<64>, heapless::String<64>, SchedContextId);
/// phase-273 — [`NodeSchedEntry`] plus the callback-group name that narrows it.
pub(crate) type GroupSchedEntry = (
    heapless::String<64>,
    heapless::String<64>,
    heapless::String<32>,
    SchedContextId,
);
/// Issue 0436 — `(rmw name, locator)` for one extra session.
pub(crate) type ExtraSessionId = (heapless::String<32>, heapless::String<128>);
/// Phase 228.C — one callback-group name in a tier executor's filter.
pub(crate) type GroupName = heapless::String<32>;

// ============================================================================
// CarvedVec — a `heapless::Vec` whose capacity lives in the caller's backing
// ============================================================================

/// A push-only vector over CARVED storage: the elements live in the caller's
/// backing, the `Executor` holds only a fat pointer and a fill cursor.
///
/// phase-409 (issue 0961) — this is the shape phase-271 gave the six sized
/// tables, generalised so the other nine can use it too. It exists because
/// `heapless::Vec<T, N>` puts `N * size_of::<T>()` bytes INSIDE the struct, so
/// every knob that sets an `N` also sets the stack frame of every function that
/// moves an `Executor`: `MAX_CBS` 14 -> 36 (a handle-count fix) added ~3.7 KiB
/// to `Executor::open_in`'s prologue on a part whose main thread has 32 KiB in
/// total.
///
/// Deliberately `MaybeUninit<T>` rather than `Option<T>`, for three reasons:
/// [`as_slice`](Self::as_slice) can hand back the `&[T]` the public
/// `Executor::nodes()` accessor already returns; `carve` writes NOTHING at open
/// (the `memclr` in issue 0961's fault was the executor's tables being zeroed);
/// and the layout matches `heapless::Vec`'s, so the storage cost is the same
/// bytes in a different place.
///
/// Occupied slots are `[0, len)` and IN PUSH ORDER — several callers index by
/// insertion position (`NodeId` IS an index into `nodes`; `session_idx` is an
/// index into `extra_sessions`), so this must never gain a `swap_remove`.
pub(crate) struct CarvedVec<'s, T> {
    slots: &'s mut [MaybeUninit<T>],
    len: usize,
}

impl<'s, T> CarvedVec<'s, T> {
    /// Wrap carved, UNINITIALISED storage as an empty vector.
    fn new(slots: &'s mut [MaybeUninit<T>]) -> Self {
        Self { slots, len: 0 }
    }

    /// Slots this vector can hold — the entry's sizing, not a build-time const.
    pub(crate) fn capacity(&self) -> usize {
        self.slots.len()
    }

    /// Append `value`, returning it in `Err` when full (`heapless::Vec::push`).
    pub(crate) fn push(&mut self, value: T) -> Result<(), T> {
        if self.len == self.capacity() {
            return Err(value);
        }
        self.slots[self.len].write(value);
        self.len += 1;
        Ok(())
    }

    /// Drop every element and reset the cursor.
    pub(crate) fn clear(&mut self) {
        let len = core::mem::replace(&mut self.len, 0);
        if needs_drop::<T>() {
            for slot in &mut self.slots[..len] {
                // SAFETY: `[0, len)` was initialised by `push` and is dropped
                // exactly once — `len` is reset above before this loop runs, so
                // a panicking `T::drop` cannot make a second pass see it.
                unsafe { slot.assume_init_drop() };
            }
        }
    }

    /// The initialised prefix.
    pub(crate) fn as_slice(&self) -> &[T] {
        // SAFETY: `push` initialises `[0, len)` and nothing shortens `len`
        // without dropping (see `clear`).
        unsafe { &*(&self.slots[..self.len] as *const [MaybeUninit<T>] as *const [T]) }
    }

    /// The initialised prefix, mutably.
    pub(crate) fn as_mut_slice(&mut self) -> &mut [T] {
        // SAFETY: as [`as_slice`](Self::as_slice).
        unsafe { &mut *(&mut self.slots[..self.len] as *mut [MaybeUninit<T>] as *mut [T]) }
    }
}

impl<T> Deref for CarvedVec<'_, T> {
    type Target = [T];
    fn deref(&self) -> &[T] {
        self.as_slice()
    }
}

impl<T> DerefMut for CarvedVec<'_, T> {
    fn deref_mut(&mut self) -> &mut [T] {
        self.as_mut_slice()
    }
}

impl<T> Drop for CarvedVec<'_, T> {
    fn drop(&mut self) {
        // The backing is the CALLER's memory, so nothing drops these elements
        // for us — and one of these tables is `extra_sessions`, whose elements
        // close RMW sessions. A `Drop` impl on the vector (rather than a pass in
        // `Executor::drop`) keeps the ORDER the inline `heapless::Vec`s had:
        // `Executor`'s fields drop in declaration order, so the primary
        // `session` still closes before the extras.
        self.clear();
    }
}

// ============================================================================
// Layout
// ============================================================================

/// The typed reference layout the [`carve`] mirrors. `#[repr(C)]` so its field
/// offsets are the deterministic declaration-order layout the const-fn below
/// reproduces; a unit test asserts they agree. Only referenced by tests.
#[cfg(test)]
#[repr(C)]
pub(crate) struct ExecutorStorage<
    const CBS: usize,
    const SC: usize,
    const ARENA: usize,
    const NODES: usize,
> {
    arena: [MaybeUninit<u8>; ARENA],
    entries: [Option<CallbackMeta>; CBS],
    sched_contexts: [Option<SchedContext>; SC],
    sched_context_bindings: [SchedContextId; CBS],
    sporadic_states: [Option<SporadicState>; SC],
    #[cfg(feature = "alloc")]
    sporadic_atomic_states: [Option<SporadicAtomic>; SC],
    remaps: [Option<RemapRule>; MAX_REMAPS],
    // phase-409 — the nine that phase-271 left inline.
    nodes: [MaybeUninit<NodeRecord>; NODES],
    extra_sessions: [MaybeUninit<ConcreteSession>; NODES],
    extra_session_ids: [MaybeUninit<ExtraSessionId>; NODES],
    node_sched_table: [MaybeUninit<NodeSchedEntry>; NODES],
    dispatch_slots: [MaybeUninit<DispatchSlot>; NODES],
    component_slots: [MaybeUninit<ComponentSlot>; NODES],
    active_groups: [MaybeUninit<GroupName>; NODES],
    group_sched_table: [MaybeUninit<GroupSchedEntry>; CBS],
    monitor_violations: [MaybeUninit<Violation>; MAX_VIOLATIONS],
}

/// The typed, mutable sub-slices an [`Executor`](super::spin::Executor) borrows
/// from a carved backing. Element memory is initialised by [`carve`] for the
/// `Option`/`SchedContextId` tables; the [`CarvedVec`] tables are handed over
/// UNINITIALISED and fill on push.
pub(crate) struct ExecutorSlices<'s> {
    pub(crate) arena: &'s mut [MaybeUninit<u8>],
    pub(crate) entries: &'s mut [Option<CallbackMeta>],
    pub(crate) sched_contexts: &'s mut [Option<SchedContext>],
    pub(crate) sched_context_bindings: &'s mut [SchedContextId],
    pub(crate) sporadic_states: &'s mut [Option<SporadicState>],
    #[cfg(feature = "alloc")]
    pub(crate) sporadic_atomic_states: &'s mut [Option<SporadicAtomic>],
    /// Issue 0563 — the SEVENTH sized table. phase-271 moved six of these out
    /// of `Executor` and into caller-owned storage; the remap table was left
    /// inline and grew to 6664 bytes of an 11632-byte struct (57%), which is
    /// what made constructing an executor a ~9.3 KB STACK temporary and
    /// overflowed the Zephyr Cortex-M main stack in issue 0552.
    ///
    /// Fixed count (`MAX_REMAPS`) rather than a new `ExecutorSizing` knob:
    /// the capability is unchanged, so this needs no public API change and no
    /// regeneration of every entry's sizing. The required backing grows by the
    /// same bytes, but that lands in the caller's STATIC buffer instead of on
    /// the stack, which is the entire point.
    pub(crate) remaps: &'s mut [Option<RemapRule>],
    // phase-409 (issue 0961) — the remaining nine, same reasoning one campaign
    // later. `MAX_CBS` 14 -> 36 put ~3.7 KiB of `group_sched_table` on the stack
    // of every function that moves an `Executor`.
    pub(crate) nodes: CarvedVec<'s, NodeRecord>,
    pub(crate) extra_sessions: CarvedVec<'s, ConcreteSession>,
    pub(crate) extra_session_ids: CarvedVec<'s, ExtraSessionId>,
    pub(crate) node_sched_table: CarvedVec<'s, NodeSchedEntry>,
    pub(crate) dispatch_slots: CarvedVec<'s, DispatchSlot>,
    pub(crate) component_slots: CarvedVec<'s, ComponentSlot>,
    pub(crate) active_groups: CarvedVec<'s, GroupName>,
    pub(crate) group_sched_table: CarvedVec<'s, GroupSchedEntry>,
    pub(crate) monitor_violations: CarvedVec<'s, Violation>,
}

/// Byte offsets of each field within the backing + total size/align. Computed
/// identically by [`executor_storage_layout`] and [`carve`] (single source of
/// truth), reproducing `#[repr(C)]` declaration-order layout.
struct FieldOffsets {
    arena: usize,
    entries: usize,
    sched_contexts: usize,
    sched_context_bindings: usize,
    sporadic_states: usize,
    #[cfg(feature = "alloc")]
    sporadic_atomic_states: usize,
    remaps: usize,
    nodes: usize,
    extra_sessions: usize,
    extra_session_ids: usize,
    node_sched_table: usize,
    dispatch_slots: usize,
    component_slots: usize,
    active_groups: usize,
    group_sched_table: usize,
    monitor_violations: usize,
    size: usize,
    align: usize,
}

const fn align_up(off: usize, align: usize) -> usize {
    off.div_ceil(align) * align
}

/// Size and alignment of ONE region's element type.
///
/// issue 1197 — the backing size is `arena + tables`, and the table half is a
/// function of these facts plus the counts. The facts are `repr(Rust)` layout,
/// which the compiler owns and may change between versions, so they can never be
/// re-derived outside it (that is the sizes-header mirror class, 0088 -> 0268).
/// Making them a PARAMETER means the one arithmetic below can be evaluated
/// against layouts obtained from the compiler for a target that is not this one
/// — a build script probing an rlib, the way `nros-c` already recovers
/// `EXECUTOR_OPAQUE_U64S` — without a second implementation of the arithmetic.
#[derive(Clone, Copy)]
pub struct RegionUnit {
    /// `size_of::<T>()`.
    pub size: usize,
    /// `align_of::<T>()`.
    pub align: usize,
}

impl RegionUnit {
    /// The compiler's answer for `T`, on the target being compiled.
    pub const fn of<T>() -> Self {
        Self {
            size: size_of::<T>(),
            align: align_of::<T>(),
        }
    }
}

/// Every element type the backing carves a table of, in declaration order.
///
/// `arena` is absent on purpose: it is `[MaybeUninit<u8>]`, size 1 align 1 by
/// definition, and its LENGTH is a count rather than a unit.
#[derive(Clone, Copy)]
pub struct RegionUnits {
    pub callback_meta: RegionUnit,
    pub sched_context: RegionUnit,
    pub sched_context_id: RegionUnit,
    pub sporadic_state: RegionUnit,
    /// `alloc` only; ignored when [`ExecutorSizing`] is evaluated without it.
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

impl RegionUnits {
    /// What this compiler says, for the target it is compiling.
    pub const NATIVE: Self = Self {
        callback_meta: RegionUnit::of::<Option<CallbackMeta>>(),
        sched_context: RegionUnit::of::<Option<SchedContext>>(),
        sched_context_id: RegionUnit::of::<SchedContextId>(),
        sporadic_state: RegionUnit::of::<Option<SporadicState>>(),
        #[cfg(feature = "alloc")]
        sporadic_atomic: RegionUnit::of::<Option<SporadicAtomic>>(),
        #[cfg(not(feature = "alloc"))]
        sporadic_atomic: RegionUnit { size: 0, align: 1 },
        remap_rule: RegionUnit::of::<Option<RemapRule>>(),
        node_record: RegionUnit::of::<NodeRecord>(),
        concrete_session: RegionUnit::of::<ConcreteSession>(),
        extra_session_id: RegionUnit::of::<ExtraSessionId>(),
        node_sched_entry: RegionUnit::of::<NodeSchedEntry>(),
        dispatch_slot: RegionUnit::of::<DispatchSlot>(),
        component_slot: RegionUnit::of::<ComponentSlot>(),
        group_name: RegionUnit::of::<GroupName>(),
        group_sched_entry: RegionUnit::of::<GroupSchedEntry>(),
        violation: RegionUnit::of::<Violation>(),
    };
}

const fn compute_offsets(sizing: ExecutorSizing) -> FieldOffsets {
    compute_offsets_with(sizing, RegionUnits::NATIVE)
}

/// The ONE placement arithmetic, evaluated against a supplied unit table.
///
/// `compute_offsets` is this with [`RegionUnits::NATIVE`]. Keeping one function
/// is the point: an external consumer supplies probed units and gets the same
/// answer by construction rather than by a second implementation agreeing.
const fn compute_offsets_with(sizing: ExecutorSizing, units: RegionUnits) -> FieldOffsets {
    let ExecutorSizing {
        cbs,
        sc,
        arena,
        nodes: node_slots,
    } = sizing;
    let mut off = 0usize;
    let mut max_align = 1usize;

    // arena: [MaybeUninit<u8>; arena] — align 1, at offset 0.
    let arena_off = 0usize;
    off += arena;

    // Reads its facts from `units` rather than `size_of::<T>()` so the SAME
    // arithmetic can be evaluated for a target this compiler is not building
    // (issue 1197). `RegionUnits::NATIVE` restores the old behaviour exactly.
    macro_rules! place {
        ($n:expr, $u:expr) => {{
            let RegionUnit { size, align: a } = $u;
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
    #[cfg(feature = "alloc")]
    let sporadic_atomic_states = place!(sc, units.sporadic_atomic);
    let remaps = place!(MAX_REMAPS, units.remap_rule);
    let nodes = place!(node_slots, units.node_record);
    let extra_sessions = place!(node_slots, units.concrete_session);
    let extra_session_ids = place!(node_slots, units.extra_session_id);
    let node_sched_table = place!(node_slots, units.node_sched_entry);
    let dispatch_slots = place!(node_slots, units.dispatch_slot);
    let component_slots = place!(node_slots, units.component_slot);
    let active_groups = place!(node_slots, units.group_name);
    let group_sched_table = place!(cbs, units.group_sched_entry);
    let monitor_violations = place!(MAX_VIOLATIONS, units.violation);

    let size = align_up(off, max_align);
    FieldOffsets {
        arena: arena_off,
        entries,
        sched_contexts,
        sched_context_bindings,
        sporadic_states,
        #[cfg(feature = "alloc")]
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
        size,
        align: max_align,
    }
}

/// Byte [`Layout`] of the backing, evaluated against a SUPPLIED unit table.
///
/// issue 1197 — the public half of the split. `executor_storage_layout` is this
/// with [`RegionUnits::NATIVE`]; an external consumer that has recovered the
/// units for another target (a build script probing an rlib, the way `nros-c`
/// recovers `EXECUTOR_OPAQUE_U64S`) gets the same arithmetic rather than a
/// second implementation of it.
///
/// The OFFSETS stay private: only `carve` needs them, and it runs in the crate
/// that owns the types. A consumer sizing a reservation needs the total.
pub const fn executor_storage_layout_with(sizing: ExecutorSizing, units: RegionUnits) -> Layout {
    let o = compute_offsets_with(sizing, units);
    // SAFETY: as `executor_storage_layout` — `align` is a max of alignments and
    // `size` is rounded up to it.
    unsafe { Layout::from_size_align_unchecked(o.size, o.align) }
}

/// Byte [`Layout`] of the backing a `sizing`-sized executor needs.
/// Public + non-generic so the macro / FFI can size a raw backing.
pub const fn executor_storage_layout(sizing: ExecutorSizing) -> Layout {
    let o = compute_offsets(sizing);
    // SAFETY: `align` is a power of two (a `max` of `align_of` results) and `size`
    // is rounded up to it; both are non-zero.
    unsafe { Layout::from_size_align_unchecked(o.size, o.align) }
}

/// Number of `u64` words a backing must hold for a `sizing`-sized executor.
/// `u64` backing is 8-aligned, which covers every field (all `align_of ≤ 8`;
/// asserted in tests), so the entry never hand-aligns. The macro emits
/// `[MaybeUninit<u64>; executor_storage_u64_len(sizing)]`.
pub const fn executor_storage_u64_len(sizing: ExecutorSizing) -> usize {
    executor_storage_layout(sizing).size().div_ceil(8)
}

/// Per-entry executor sizing — the entity counts an [`Executor`](super::spin::Executor)
/// is built to hold. **Public + non-generic** (the "C/C++ is a thin wrapper"
/// principle): the entry / macro / FFI supplies these as plain `usize`s rather
/// than as type/const generics C can't name. Used to size + carve the backing.
///
/// `cbs` is capped at 64 by the executor's `u64` ready-set bitmask (asserted in
/// [`carve`]-time / `open_in`).
#[derive(Clone, Copy)]
pub struct ExecutorSizing {
    /// Callback-table slots (`entries`, the per-entry SC bindings, and the
    /// per-callback-group sched table). ≤ 64.
    pub cbs: usize,
    /// Scheduling-context slots (`sched_contexts` + sporadic state tables).
    pub sc: usize,
    /// Bump-allocator arena size in bytes.
    pub arena: usize,
    /// phase-409 — Node slots. One worst-case extra session, extra-session id,
    /// node-sched binding, dispatch slot, component slot and callback-group
    /// filter entry per Node, so ONE count covers all seven tables (that is the
    /// upper bound each of them already assumed under `MAX_NODES`).
    pub nodes: usize,
}

impl ExecutorSizing {
    /// The build-time default (`MAX_CBS`/`MAX_SC`/`ARENA_SIZE`/`MAX_NODES`
    /// consts) — the backward-compatible size the `alloc` convenience
    /// constructors leak.
    pub const DEFAULT: Self = Self {
        cbs: crate::config::MAX_CBS,
        sc: crate::config::MAX_SC,
        arena: crate::config::ARENA_SIZE,
        nodes: crate::config::MAX_NODES,
    };

    /// `u64` words a backing must hold for this sizing (see
    /// [`executor_storage_u64_len`]).
    pub const fn u64_len(&self) -> usize {
        executor_storage_u64_len(*self)
    }
}

/// The exact `#[repr(C)]` byte layout the C/C++ FFI's inline executor buffer must
/// hold: an [`Executor`](super::spin::Executor)`<'static>` header immediately
/// followed by a default-sized ([`ExecutorSizing::DEFAULT`]) storage backing.
///
/// The FFI keeps the executor inline (heap-free — matching the Rust no-alloc
/// requirement) and carves its per-entry tables from the SAME buffer's
/// [`backing`](Self::backing) tail. Because that buffer is **pinned** — the C
/// caller allocates it, it is initialised in place, and it is only ever reached
/// through a stable `nros_executor_t*` (never moved after init) — the resulting
/// self-borrow (the header's slices pointing into the same struct's tail) is
/// sound. The FFI probes `size_of` of THIS type (not bare `Executor`) to size
/// its `_opaque` array, and reinterprets `_opaque` as `*mut ExecutorInlineStorage`
/// (the executor stays at offset 0, so existing offset-0 accessors are unchanged).
///
/// phase-409 — moving the last nine tables into the backing moves bytes from
/// `exec` to `backing` and leaves this total roughly where it was; what changes
/// is the STACK, because the header is what `open_in` builds and returns by
/// value.
#[repr(C)]
pub struct ExecutorInlineStorage {
    /// The executor, written in place (offset 0) by `from_session_ptr_in`.
    pub exec: MaybeUninit<super::spin::Executor<'static>>,
    /// The carved backing the executor's slices borrow (the buffer's tail).
    pub backing: [MaybeUninit<u64>; ExecutorSizing::DEFAULT.u64_len()],
}

/// Carve an 8-aligned `u64` backing into the typed executor slices.
///
/// # Safety
/// - `backing.len() * 8` must be ≥ `executor_storage_layout(sizing).size()`.
/// - The returned slices alias `backing`; it must outlive them (the `'s` bound)
///   and not be otherwise accessed while they live.
///
/// The `Option`/binding tables are initialised here (`entries`/SC tables →
/// `None`, bindings → `SchedContextId(0)`), so the returned `&mut [T]` reference
/// validly-init memory. The [`CarvedVec`] tables are handed over empty and
/// UNINITIALISED — nothing reads past their fill cursor.
pub(crate) unsafe fn carve<'s>(
    backing: &'s mut [MaybeUninit<u64>],
    sizing: ExecutorSizing,
) -> ExecutorSlices<'s> {
    let ExecutorSizing {
        cbs,
        sc,
        arena,
        nodes: node_slots,
    } = sizing;
    let o = compute_offsets(sizing);
    // Fail-loud on EVERY profile (not `debug_assert!`): embedded release builds
    // strip debug-assertions, and a backing that is too small is silent memory
    // corruption — the carved `entries`/`sched_contexts` tables run past the end
    // of `backing` into whatever .bss follows (e.g. a C carrier's `__nros_c_inst`),
    // leaving a NULL `drop_fn` that faults in `Executor::drop`. This is exactly
    // how a STALE config-header mirror (C buffer sized from an out-of-date
    // `NROS_*_STORAGE_SIZE`) manifested as a `jalr -> 0` on threadx-riscv64 (#131).
    // Panic here instead, at open, with the two sizes named.
    assert!(
        backing.len() * 8 >= o.size,
        "executor backing too small: {} bytes < {} required — the storage buffer \
         (NROS_*_STORAGE_SIZE) disagrees with the executor layout; rebuild clean so \
         the generated config header matches",
        backing.len() * 8,
        o.size
    );
    let base = backing.as_mut_ptr() as *mut u8;

    // One `CarvedVec` over `$n` slots at byte offset `$at`. No element writes —
    // the vector starts empty. Expanded inside the `unsafe` block below;
    // SAFETY there: `compute_offsets` placed `$n` `$ty`s at `$at`, aligned and
    // inside the asserted `o.size`, and each region is placed exactly once, so
    // the slices do not alias.
    macro_rules! carved {
        ($at:expr, $n:expr, $ty:ty) => {{
            let p = base.add($at) as *mut MaybeUninit<$ty>;
            CarvedVec::new(core::slice::from_raw_parts_mut(p, $n))
        }};
    }

    unsafe {
        // arena — no init needed (MaybeUninit).
        let arena_s =
            core::slice::from_raw_parts_mut(base.add(o.arena) as *mut MaybeUninit<u8>, arena);

        let entries_p = base.add(o.entries) as *mut Option<CallbackMeta>;
        let mut i = 0;
        while i < cbs {
            entries_p.add(i).write(None);
            i += 1;
        }
        let entries_s = core::slice::from_raw_parts_mut(entries_p, cbs);

        let sc_p = base.add(o.sched_contexts) as *mut Option<SchedContext>;
        let mut i = 0;
        while i < sc {
            sc_p.add(i).write(None);
            i += 1;
        }
        let sched_contexts_s = core::slice::from_raw_parts_mut(sc_p, sc);

        let bind_p = base.add(o.sched_context_bindings) as *mut SchedContextId;
        let mut i = 0;
        while i < cbs {
            bind_p.add(i).write(SchedContextId(0));
            i += 1;
        }
        let bindings_s = core::slice::from_raw_parts_mut(bind_p, cbs);

        let sp_p = base.add(o.sporadic_states) as *mut Option<SporadicState>;
        let mut i = 0;
        while i < sc {
            sp_p.add(i).write(None);
            i += 1;
        }
        let sporadic_s = core::slice::from_raw_parts_mut(sp_p, sc);

        #[cfg(feature = "alloc")]
        let atomic_s = {
            let ap = base.add(o.sporadic_atomic_states) as *mut Option<SporadicAtomic>;
            let mut i = 0;
            while i < sc {
                ap.add(i).write(None);
                i += 1;
            }
            core::slice::from_raw_parts_mut(ap, sc)
        };

        let remaps_p = base.add(o.remaps) as *mut Option<RemapRule>;
        let mut i = 0;
        while i < MAX_REMAPS {
            remaps_p.add(i).write(None);
            i += 1;
        }
        let remaps_s = core::slice::from_raw_parts_mut(remaps_p, MAX_REMAPS);

        ExecutorSlices {
            arena: arena_s,
            entries: entries_s,
            sched_contexts: sched_contexts_s,
            sched_context_bindings: bindings_s,
            sporadic_states: sporadic_s,
            #[cfg(feature = "alloc")]
            sporadic_atomic_states: atomic_s,
            remaps: remaps_s,
            nodes: carved!(o.nodes, node_slots, NodeRecord),
            extra_sessions: carved!(o.extra_sessions, node_slots, ConcreteSession),
            extra_session_ids: carved!(o.extra_session_ids, node_slots, ExtraSessionId),
            node_sched_table: carved!(o.node_sched_table, node_slots, NodeSchedEntry),
            dispatch_slots: carved!(o.dispatch_slots, node_slots, DispatchSlot),
            component_slots: carved!(o.component_slots, node_slots, ComponentSlot),
            active_groups: carved!(o.active_groups, node_slots, GroupName),
            group_sched_table: carved!(o.group_sched_table, cbs, GroupSchedEntry),
            monitor_violations: carved!(o.monitor_violations, MAX_VIOLATIONS, Violation),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// issue 1197 — the arithmetic must actually READ `units`, not quietly keep
    /// calling `size_of::<T>()` behind a parameter it ignores.
    ///
    /// A parameter that is accepted and unused is exactly how a "data-driven"
    /// refactor passes its own tests while delivering nothing: `NATIVE` would
    /// still be right, every existing assertion would still hold, and the
    /// external consumer this exists for would get the HOST's layout back.
    /// So this feeds a table that could not have come from this compiler and
    /// checks the answer moves by the amount that table implies.
    #[test]
    fn the_placement_reads_the_unit_table_it_is_given() {
        let sizing = ExecutorSizing {
            cbs: 1,
            sc: 0,
            arena: 0,
            nodes: 0,
        };
        let native = executor_storage_layout_with(sizing, RegionUnits::NATIVE);

        // One region is scaled by `cbs`: `callback_meta`. Grow ONLY it, by a
        // multiple of its own alignment so no padding shifts, and the total must
        // grow by exactly one element's worth.
        let mut fatter = RegionUnits::NATIVE;
        let grown_by = fatter.callback_meta.align * 4;
        fatter.callback_meta.size += grown_by;
        let widened = executor_storage_layout_with(sizing, fatter);

        assert_eq!(
            widened.size(),
            native.size() + grown_by,
            "growing one cbs-scaled unit by {grown_by} must grow the backing by \
             the same, or the table is being ignored"
        );

        // And a table of ZEROES must collapse the tables entirely, leaving only
        // the arena. This is the direction that catches a `size_of` still being
        // read for some region the parameter was threaded past.
        let zeroed = RegionUnits {
            callback_meta: RegionUnit { size: 0, align: 1 },
            sched_context: RegionUnit { size: 0, align: 1 },
            sched_context_id: RegionUnit { size: 0, align: 1 },
            sporadic_state: RegionUnit { size: 0, align: 1 },
            sporadic_atomic: RegionUnit { size: 0, align: 1 },
            remap_rule: RegionUnit { size: 0, align: 1 },
            node_record: RegionUnit { size: 0, align: 1 },
            concrete_session: RegionUnit { size: 0, align: 1 },
            extra_session_id: RegionUnit { size: 0, align: 1 },
            node_sched_entry: RegionUnit { size: 0, align: 1 },
            dispatch_slot: RegionUnit { size: 0, align: 1 },
            component_slot: RegionUnit { size: 0, align: 1 },
            group_name: RegionUnit { size: 0, align: 1 },
            group_sched_entry: RegionUnit { size: 0, align: 1 },
            violation: RegionUnit { size: 0, align: 1 },
        };
        let arena_only = executor_storage_layout_with(
            ExecutorSizing {
                cbs: CBS,
                sc: SC,
                arena: 4096,
                nodes: 4,
            },
            zeroed,
        );
        assert_eq!(
            arena_only.size(),
            4096,
            "with every unit zero-sized the backing is the arena and nothing else"
        );
    }

    const CBS: usize = crate::config::MAX_CBS;
    const SC: usize = crate::config::MAX_SC;
    const ARENA: usize = crate::config::ARENA_SIZE;
    const NODES: usize = crate::config::MAX_NODES;
    const DEFAULT: ExecutorSizing = ExecutorSizing::DEFAULT;

    #[test]
    fn layout_matches_typed_repr_c() {
        // The manual const-fn layout must equal the compiler's `#[repr(C)]` layout
        // of the typed storage — proof the carve offsets are the real field offsets.
        let got = executor_storage_layout(DEFAULT);
        let want = Layout::new::<ExecutorStorage<CBS, SC, ARENA, NODES>>();
        assert_eq!(got.size(), want.size(), "size");
        assert_eq!(got.align(), want.align(), "align");
    }

    /// phase-409 — and the FIELD offsets, one by one. The size/align check above
    /// is the one phase-271 shipped, and it does not distinguish a REORDER that
    /// happens to pad the same: with sixteen regions instead of seven, "the
    /// totals agree" stopped being adequate evidence that `carve` hands out the
    /// region it names. A mismatch here is `nodes` being handed
    /// `extra_sessions`' memory, which is not a size bug and no total would show.
    #[test]
    fn every_carved_region_starts_where_repr_c_puts_it() {
        type Ref = ExecutorStorage<CBS, SC, ARENA, NODES>;
        let o = compute_offsets(DEFAULT);
        macro_rules! same {
            ($f:ident) => {
                assert_eq!(
                    o.$f,
                    core::mem::offset_of!(Ref, $f),
                    concat!("offset of `", stringify!($f), "`")
                );
            };
        }
        same!(arena);
        same!(entries);
        same!(sched_contexts);
        same!(sched_context_bindings);
        same!(sporadic_states);
        #[cfg(feature = "alloc")]
        same!(sporadic_atomic_states);
        same!(remaps);
        same!(nodes);
        same!(extra_sessions);
        same!(extra_session_ids);
        same!(node_sched_table);
        same!(dispatch_slots);
        same!(component_slots);
        same!(active_groups);
        same!(group_sched_table);
        same!(monitor_violations);
    }

    #[test]
    fn u64_backing_covers_all_field_aligns() {
        assert!(align_of::<Option<CallbackMeta>>() <= 8);
        assert!(align_of::<Option<SchedContext>>() <= 8);
        assert!(align_of::<SchedContextId>() <= 8);
        assert!(align_of::<Option<SporadicState>>() <= 8);
        // phase-409 — the nine that moved. A type whose alignment exceeds 8
        // would make the `u64` backing insufficient for the whole carve, not
        // just for its own table.
        assert!(align_of::<NodeRecord>() <= 8);
        assert!(align_of::<ConcreteSession>() <= 8);
        assert!(align_of::<ExtraSessionId>() <= 8);
        assert!(align_of::<NodeSchedEntry>() <= 8);
        assert!(align_of::<DispatchSlot>() <= 8);
        assert!(align_of::<ComponentSlot>() <= 8);
        assert!(align_of::<GroupName>() <= 8);
        assert!(align_of::<GroupSchedEntry>() <= 8);
        assert!(align_of::<Violation>() <= 8);
        assert!(executor_storage_layout(DEFAULT).align() <= 8);
    }

    /// phase-409 (issue 0961) — every knob-scaled table's per-slot cost is
    /// charged to the BACKING, which is where the caller put it, and not to the
    /// `Executor` value, which is what `open_in` builds on the stack and
    /// returns by value.
    ///
    /// Deltas rather than absolute sizes, the shape
    /// `the_default_subscription_buffer_is_unchanged` uses: a total says
    /// nothing about which knob moved it, while the difference between two
    /// sizings isolates exactly one knob's per-slot width.
    #[test]
    fn every_knob_scaled_table_is_charged_to_the_backing() {
        let base = executor_storage_layout(DEFAULT).size();

        let one_more_node = executor_storage_layout(ExecutorSizing {
            nodes: DEFAULT.nodes + 1,
            ..DEFAULT
        })
        .size();
        let per_node = size_of::<NodeRecord>()
            + size_of::<ConcreteSession>()
            + size_of::<ExtraSessionId>()
            + size_of::<NodeSchedEntry>()
            + size_of::<DispatchSlot>()
            + size_of::<ComponentSlot>()
            + size_of::<GroupName>();
        assert!(
            one_more_node - base >= per_node,
            "one more Node slot must cost its seven tables ({per_node} B) in the \
             backing; it cost {}",
            one_more_node - base
        );

        // `arena` is its own field, so `..DEFAULT` holds it while `cbs` moves —
        // which isolates the callback-indexed tables. `group_sched_table` is the
        // one that coupled `NROS_EXECUTOR_MAX_CBS` to the main thread's stack.
        let one_more_cb = executor_storage_layout(ExecutorSizing {
            cbs: DEFAULT.cbs + 1,
            ..DEFAULT
        })
        .size();
        assert!(
            one_more_cb - base >= size_of::<GroupSchedEntry>(),
            "one more callback slot must cost a `group_sched_table` entry \
             ({} B) in the backing; it cost {}",
            size_of::<GroupSchedEntry>(),
            one_more_cb - base
        );
    }

    /// The other half of the same claim, and the one issue 0961 is about: the
    /// VALUE does not move.
    ///
    /// A CEILING rather than an equality, because the header's exact size is a
    /// target + feature detail — measured 1016 B on the `std` lane and 2048 B
    /// under `--all-features`, and the difference is almost entirely the primary
    /// session (`SessionStore` is 16 B over `MockSession`, 536 B over
    /// `CffiSession`) plus `scheduler-os-priority`'s worker pool. Both of those
    /// are allowed for BY NAME below, so what is left is a tight budget on the
    /// header proper.
    ///
    /// It is a ceiling on the same number at every knob value, which is the
    /// property under test: 1016 B at BOTH the shipped defaults and the island's
    /// `MAX_CBS=36` / `MAX_NODES=6`. Before this phase it was 5072 B and
    /// 12768 B respectively.
    ///
    /// If this fires for a field you MEANT to add, the fix is almost always to
    /// carve it rather than to raise the ceiling: `Executor::open_in` builds
    /// this value on the stack and returns it by value, and `nros_cpp_init`
    /// holds the returned value, so every byte is charged to two frames of every
    /// image's boot path.
    #[test]
    fn the_executor_value_does_not_scale_with_the_knobs() {
        let value = size_of::<super::super::spin::Executor<'static>>();
        // Two things in the header are legitimately large and are NOT scaled by
        // `MAX_CBS` / `MAX_NODES`, so they get named allowances rather than a
        // looser ceiling for every build:
        //   * the primary session, held by value (that is what owning a session
        //     means), whose size is the backend's, not a knob's;
        //   * `scheduler-os-priority`'s worker pool, a pair of `FnvIndexMap`s
        //     sized by `MAX_PRIORITY_LEVELS` — an opt-in capability.
        #[allow(unused_mut)]
        let mut ceiling = 1280 + size_of::<super::super::spin::SessionStore>();
        #[cfg(all(
            feature = "alloc",
            feature = "rmw-cffi",
            feature = "scheduler-os-priority"
        ))]
        {
            ceiling += size_of::<super::super::os_priority::OsPriorityPool>();
        }
        assert!(
            value <= ceiling,
            "`Executor` is {value} B at MAX_CBS={CBS} / MAX_NODES={NODES}, over \
             the {ceiling} B this value is budgeted; a table that scales with a \
             knob has come back inline (issue 0961)."
        );
    }

    // phase-361 W8.e / issue 0594 — this test heap-allocates its backing array,
    // so it needs `alloc`. It compiled under `--no-default-features` only
    // because feature unification from another workspace member happened to
    // turn `nros-node/alloc` on; nothing does that now.
    #[cfg(feature = "alloc")]
    #[test]
    fn carve_yields_right_lengths_and_inits() {
        // Heap-allocate: the default test config (MAX_CBS/MAX_SC/ARENA_SIZE from
        // build.rs) makes this backing array tens of KB, well past
        // `clippy::large_stack_arrays`'s threshold — and the size here is
        // incidental (mirrors production config), not the point under test, so
        // boxing is the right fix rather than an allow.
        let mut backing =
            alloc::vec![const { MaybeUninit::<u64>::uninit() }; executor_storage_u64_len(DEFAULT)]
                .into_boxed_slice();
        let s = unsafe { carve(&mut backing, DEFAULT) };
        assert_eq!(s.arena.len(), ARENA);
        assert_eq!(s.entries.len(), CBS);
        assert_eq!(s.sched_contexts.len(), SC);
        assert_eq!(s.sched_context_bindings.len(), CBS);
        assert_eq!(s.sporadic_states.len(), SC);
        assert!(s.entries.iter().all(|e| e.is_none()));
        assert!(s.sched_context_bindings.iter().all(|b| b.0 == 0));
        // phase-409 — the CarvedVecs come back empty at their carved capacity.
        assert_eq!(s.nodes.capacity(), NODES);
        assert_eq!(s.extra_sessions.capacity(), NODES);
        assert_eq!(s.extra_session_ids.capacity(), NODES);
        assert_eq!(s.node_sched_table.capacity(), NODES);
        assert_eq!(s.dispatch_slots.capacity(), NODES);
        assert_eq!(s.component_slots.capacity(), NODES);
        assert_eq!(s.active_groups.capacity(), NODES);
        assert_eq!(s.group_sched_table.capacity(), CBS);
        assert_eq!(s.monitor_violations.capacity(), MAX_VIOLATIONS);
        assert_eq!(s.nodes.len(), 0);
        assert_eq!(s.group_sched_table.len(), 0);
    }

    #[test]
    fn carved_vec_pushes_in_order_and_refuses_when_full() {
        let mut slots = [const { MaybeUninit::<u32>::uninit() }; 3];
        let mut v = CarvedVec::new(&mut slots);
        assert_eq!(v.capacity(), 3);
        assert!(v.push(10).is_ok());
        assert!(v.push(20).is_ok());
        assert!(v.push(30).is_ok());
        // Order is load-bearing: `NodeId` IS an index into `nodes`.
        assert_eq!(v.as_slice(), &[10, 20, 30]);
        // Full is a REFUSAL that hands the value back, like `heapless::Vec`.
        assert_eq!(v.push(40), Err(40));
        v.clear();
        assert!(v.is_empty());
        assert!(v.push(50).is_ok());
        assert_eq!(v.as_slice(), &[50]);
    }

    /// The elements live in the CALLER's backing, so nothing drops them unless
    /// the vector does — and `extra_sessions` holds RMW sessions. Without this
    /// a bridge executor would leak every extra session it opened.
    #[test]
    fn carved_vec_drops_its_elements() {
        use core::sync::atomic::{AtomicUsize, Ordering};
        static DROPS: AtomicUsize = AtomicUsize::new(0);
        struct Counted;
        impl Drop for Counted {
            fn drop(&mut self) {
                DROPS.fetch_add(1, Ordering::SeqCst);
            }
        }

        let mut slots = [const { MaybeUninit::<Counted>::uninit() }; 4];
        {
            let mut v = CarvedVec::new(&mut slots);
            assert!(v.push(Counted).is_ok());
            assert!(v.push(Counted).is_ok());
        }
        assert_eq!(
            DROPS.load(Ordering::SeqCst),
            2,
            "both pushed elements dropped — and only the pushed ones, not the \
             uninitialised tail"
        );
    }
}
