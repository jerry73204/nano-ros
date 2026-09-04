//! Phase 212.M.5.a.2 — Executor-backed `NodeRuntime` /
//! `DeclaredNodeRuntime` for nano-ros.
//!
//! [`MetadataRecorder`](crate::node_metadata::MetadataRecorder)
//! (the planner sink) binds the
//! [`Node`](crate::node::Node) /
//! [`ExecutableNode`](crate::node::ExecutableNode)
//! traits to a pure metadata target. This module is the missing twin:
//! it binds the same traits to a live [`Executor`](crate::Executor) so
//! a Node pkg can actually run — nodes, publishers,
//! subscriptions, timers materialise as real executor handles, and
//! every fired callback dispatches into
//! [`ExecutableNode::on_callback`] with the right
//! [`CallbackId`].
//!
//! Shape:
//!
//! ```ignore
//! use nros::{Executor, ExecutorConfig};
//! use nros::node_runtime::ExecutorNodeRuntime;
//!
//! let cfg = ExecutorConfig::from_env().node_name("talker_main");
//! let executor = Executor::open(&cfg).unwrap();
//! let mut runtime = ExecutorNodeRuntime::from_executor(executor);
//! let _handle = runtime.register_node::<Talker>().unwrap();
//! runtime.spin().unwrap();
//! ```
//!
//! Owned-spin / board consumer: the macro-emitted `<pkg>::register(runtime)`
//! wrapper (Phase 258, Track 2) installs each Node onto the runtime's executor
//! through the uniform `install_node_typed` seam — the same seam the C/C++
//! typed entries use. The codegen `run_plan(runtime)` body + `nros::main!`
//! owned-spin loop drive one `register(runtime)?` per launch-XML `<node>`,
//! then `runtime.spin()`. (The retired Phase 212.M.5.a four-fn-ptr BSP-baker
//! ABI — `register_dispatch_slot` / `nros_run_components` — is gone.)
//!
//! ## Coverage today (Phase 212.M.5.a.2)
//!
//! Publishers, subscriptions, and repeating timers wire end-to-end:
//! the live executor delivers callbacks; the bound
//! [`ExecutableNode::on_callback`] body runs with a
//! [`CallbackCtx`] backed by the per-component publisher resolver.
//! Service servers / clients and action servers / clients wire
//! end-to-end too (Phase 212.M-F.23): `create_entity` registers them on
//! the executor with C-ABI trampolines that route inbound requests /
//! goals into the component's `on_callback`, and the tick-time client /
//! action surface ([`TickCtx`]) is backed by `RuntimeClientDispatch` /
//! `RuntimeActions` over the live executor. Parameters are still a
//! follow-up (registration succeeds; param callbacks don't fire yet).

#![cfg(feature = "rmw-cffi")]

// W5-endgame (issue 0843): the DECLARATION is gated too — a bare
// `extern crate alloc` links the alloc crate into every image and rustc then
// demands a `#[global_allocator]` even when nothing here allocates. This line
// is what kept the first heap-free-tier image from linking.
#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "alloc")]
use alloc::{boxed::Box, vec::Vec};
#[cfg(feature = "alloc")]
use core::time::Duration;
use core::{
    cell::{Ref, RefCell, UnsafeCell},
    marker::PhantomData,
    mem::MaybeUninit,
};

// Via nros-core's re-export rather than a new direct dependency — every graph
// containing `nros` would otherwise move (16 leaf lockfiles, measured on W5.1).
use nros_core::heapless;
use portable_atomic::{AtomicUsize, Ordering};
#[cfg(feature = "alloc")]
use portable_atomic_util::Arc;

/// phase-391 W5 — entity/callback identifier, fixed-capacity.
///
/// Same bound as `nros_node::names::ResolvedName` (`MAX_RESOLVED_NAME_LEN` =
/// 128): every string these registries hold is a resolved entity id, callback
/// id, node name or namespace, i.e. name-shaped. An id that does not fit is a
/// REGISTRATION error, never a truncation.
type IdStr = heapless::String<{ nros_node::names::MAX_RESOLVED_NAME_LEN }>;

/// W5-endgame alloc-off — nodes one component's `register()` may create.
/// Components create one node in every in-tree class; 4 leaves room without
/// costing anything that outlives registration (the sink is a stack local).
const MAX_SINK_NODES: usize = 4;

// phase-391 W5 (amended by W5-endgame step 2a): the per-cell registry bound,
// PER KIND, is the `MAX_CELL_ENTITIES` knob — now spelled as `ComponentCell`'s
// const-parameter DEFAULTS rather than a shared `CELL_REG_CAP` const, so the
// macro can name tighter per-class bounds while every existing spelling keeps
// the knob-capped layout. Was `DEFAULT_MAX_METADATA_ENTITIES` (32), borrowed
// from the metadata twin — but that figure is per-PLAN-shaped and made every
// cell ~20 KB up front. 8 is per-component-shaped; a component declaring more
// gets a loud registration error naming `NROS_RUNTIME_MAX_CELL_ENTITIES`.

/// Owned copy of a name-shaped `&str`, or the registration error that says
/// which knob-less bound it burst.
fn id_str(s: &str) -> Result<IdStr, NodeDeclError> {
    IdStr::try_from(s).map_err(|_| NodeDeclError::Runtime)
}

use crate::{
    EmbeddedRawPublisher, Executor, GoalId, GoalStatus,
    node::{
        ActionExecutor, Callback, CallbackCtx, ClientDispatch, ExecutableNode, NodeContext,
        NodeDeclError, NodeOptions, NodeResult, NodeRuntime, PublisherResolver, TickCtx,
    },
    node_metadata::{
        CallbackEffectKind, CallbackId, EntityId, EntityKind, EntityMetadata, NodeId as MetaNodeId,
    },
};

// Phase 212.N.7 closing sweep — `component_register_symbol` retired
// (no live callers after the BSP baker + macro extern emit were
// removed). The former re-export here is gone.

// =============================================================================
// Public types
// =============================================================================

/// Opaque handle returned by
/// [`ExecutorNodeRuntime::register_node`].
///
/// `C` distinguishes handles at the type level so a caller who keeps
/// the handle can later (post-M.5.a.3) recover a typed mut-state
/// borrow. For today the handle is purely a witness that registration
/// succeeded.
pub struct RegisteredNode<C: ExecutableNode> {
    component_idx: usize,
    _phantom: PhantomData<fn() -> C>,
}

impl<C: ExecutableNode> RegisteredNode<C> {
    /// Slot index of this component inside the runtime.
    pub fn slot(&self) -> usize {
        self.component_idx
    }
}

/// Errors returned by the runtime entry points.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExecutorError {
    /// One of the components' register / lifecycle calls failed.
    Node(NodeDeclError),
    /// The executor's spin loop returned an unexpected error.
    SpinFailed,
}

impl From<NodeDeclError> for ExecutorError {
    fn from(e: NodeDeclError) -> Self {
        Self::Node(e)
    }
}

// =============================================================================
// Internal slot — type-erases the component's `State` so the runtime
// can hold a heterogeneous vec.
// =============================================================================

trait ComponentSlot {
    fn dispatch(&mut self, cb_id: &str, ctx: &mut CallbackCtx<'_>);
    fn tick(&mut self, ctx: &mut TickCtx<'_>);
}

struct TypedSlot<C: ExecutableNode> {
    state: C::State,
    _phantom: PhantomData<fn() -> C>,
}

impl<C: ExecutableNode> ComponentSlot for TypedSlot<C> {
    fn dispatch(&mut self, cb_id: &str, ctx: &mut CallbackCtx<'_>) {
        C::on_callback(
            &mut self.state,
            Callback::__from_id(CallbackId::new(cb_id)),
            ctx,
        );
    }
    fn tick(&mut self, ctx: &mut TickCtx<'_>) {
        C::tick(&mut self.state, ctx);
    }
}

// Phase 258 (Track 2, w5) — `BspDispatchSlot` (the type-erased
// four-fn-ptr BSP dispatch slot) is gone with the retired
// `register_dispatch_slot` / `nros_run_components` BSP-baker path. The
// only remaining `ComponentSlot` impl is `TypedSlot<C>` above (used by
// `register_node` / `register_node_borrowed` / `install_node_typed`).

/// phase-391 W5.3b — per-class slot storage the `nros::node!` macro emits.
///
/// One `static` of this type per macro expansion, so `C` is CONCRETE at the
/// emission site and the storage is sized `size_of::<TypedSlot<C>>()` exactly —
/// no byte-budget guessing on the FFI install path. The type is public API for
/// the MACRO EMIT only; it never crosses the C ABI (the trampoline keeps its
/// `(ptr, ptr, ptr) -> i32` shape), which is what keeps the const-generic
/// parameter legal under the "no const generics on FFI-visible types" rule.
///
/// `N` is the per-class INSTANCE cap: the launch path bakes one identity per
/// plan node and can name the same class twice, so this is an array, not one
/// slot. Taking past `N` is a registration error (`take` returns `None`), the
/// same Full shape as the executor's node table. Slots are one-shot — `next`
/// never rewinds — so storage is never re-initialised under a stale reference.
pub struct ComponentSlotStorage<
    C: ExecutableNode,
    const N: usize = { crate::config::MAX_CLASS_INSTANCES },
    const PUBS: usize = { crate::config::MAX_CELL_ENTITIES },
    const SVCS: usize = { crate::config::MAX_CELL_ENTITIES },
    const ACTC: usize = { crate::config::MAX_CELL_ENTITIES },
    const ACTS: usize = { crate::config::MAX_CELL_ENTITIES },
    const SSRV: usize = { crate::config::MAX_CELL_ENTITIES },
> {
    slots: [UnsafeCell<MaybeUninit<TypedSlot<C>>>; N],
    /// W5-endgame step 2b (issue 0857) — the per-instance CELL lives here too,
    /// so the whole component (state + registries) is `.bss`, not heap. Handed
    /// out pairwise with its slot by `take`; the cell's `slot` field then
    /// borrows the sibling region, which is sound because the two arrays are
    /// distinct `UnsafeCell`s claimed by the same monotonic index.
    cells: [UnsafeCell<MaybeUninit<ComponentCell<PUBS, SVCS, ACTC, ACTS, SSRV>>>; N],
    next: AtomicUsize,
}

// SAFETY: `take` hands each slot out at most once (monotonic `fetch_add`), so
// no two references to the same `UnsafeCell` contents ever coexist.
unsafe impl<
    C: ExecutableNode,
    const N: usize,
    const PUBS: usize,
    const SVCS: usize,
    const ACTC: usize,
    const ACTS: usize,
    const SSRV: usize,
> Sync for ComponentSlotStorage<C, N, PUBS, SVCS, ACTC, ACTS, SSRV>
{
}

impl<
    C: ExecutableNode,
    const N: usize,
    const PUBS: usize,
    const SVCS: usize,
    const ACTC: usize,
    const ACTS: usize,
    const SSRV: usize,
> ComponentSlotStorage<C, N, PUBS, SVCS, ACTC, ACTS, SSRV>
{
    /// Const constructor — the macro emits `static STORE: ... = ...::new();`.
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            slots: [const { UnsafeCell::new(MaybeUninit::uninit()) }; N],
            cells: [const { UnsafeCell::new(MaybeUninit::uninit()) }; N],
            next: AtomicUsize::new(0),
        }
    }

    /// Hand out the next uninitialised slot, or `None` past the instance cap.
    ///
    /// `clippy::mut_from_ref` fires because a `&mut` derived from a `&self` is
    /// usually unsound. It is not here, and the reason is already written down
    /// on the `unsafe impl Sync` above: the monotonic `fetch_add` hands each
    /// index out at most once, so no two `&mut` to one `UnsafeCell` can coexist,
    /// and `i >= N` bounds it. Allowed rather than restructured — the lint
    /// cannot see the atomic, and the invariant is the point of the type.
    #[allow(clippy::mut_from_ref)]
    fn take(
        &'static self,
    ) -> Option<(
        &'static mut MaybeUninit<TypedSlot<C>>,
        &'static mut MaybeUninit<ComponentCell<PUBS, SVCS, ACTC, ACTS, SSRV>>,
    )> {
        let i = self.next.fetch_add(1, Ordering::Relaxed);
        if i >= N {
            return None;
        }
        // SAFETY: `i` was claimed exactly once by the fetch_add above, so these
        // are the only references that will ever exist to `slots[i]`'s /
        // `cells[i]`'s contents; the storage is a `static`, so `'static` is
        // honest.
        Some(unsafe { (&mut *self.slots[i].get(), &mut *self.cells[i].get()) })
    }
}

/// Placement-init a [`TypedSlot<C>`] into `slot` and erase it.
///
/// Both registration paths end here: the FFI path with a slot from the
/// macro-emitted [`ComponentSlotStorage`], the dynamic path with one carved
/// from the runtime's caller-supplied backing.
fn place_slot<C: ExecutableNode>(
    slot: &'static mut MaybeUninit<TypedSlot<C>>,
) -> &'static mut dyn ComponentSlot
where
    C::State: 'static,
{
    slot.write(TypedSlot::<C> {
        state: C::init(),
        _phantom: PhantomData,
    })
}

/// phase-391 W5-endgame step 2b (issue 0857) — the NON-GENERIC head of every
/// `ComponentCell<..>`, guaranteed FIRST FIELD by the cell's `#[repr(C)]`.
///
/// The executor's enrolled-component state is a thin pointer to a cell whose
/// const parameters only the enrolling monomorphization knows. The tick/drop
/// trampolines are monomorphized alongside it and cast back to the concrete
/// type; the one consumer that iterates ALL enrolled states regardless of
/// class — the dispatch-stats fold — casts to this header instead. Both casts
/// are sound because the header is the first field of a `repr(C)` struct, so
/// a cell pointer IS a header pointer.
struct CellHeader {
    callback_dispatches: AtomicUsize,
    message_dispatches: AtomicUsize,
}

/// Shared per-component cell. Subscription / timer closures registered
/// against the executor hold a [`CellHandle`] so they can dispatch +
/// publish back through the resolver.
///
/// phase-391 W5-endgame step 2a (issue 0857) — generic over the four registry
/// capacities, defaulted to the `MAX_CELL_ENTITIES` knob so every existing
/// spelling (`ComponentCell`) keeps meaning the pool-capped layout. The
/// per-class macro emission names tighter bounds; everything downstream of
/// construction consumes the cell through the non-generic [`CellView`], so
/// the const params never cross an FFI or public signature.
#[repr(C)]
struct ComponentCell<
    const PUBS: usize = { crate::config::MAX_CELL_ENTITIES },
    const SVCS: usize = { crate::config::MAX_CELL_ENTITIES },
    const ACTC: usize = { crate::config::MAX_CELL_ENTITIES },
    const ACTS: usize = { crate::config::MAX_CELL_ENTITIES },
    const SSRV: usize = { crate::config::MAX_CELL_ENTITIES },
> {
    /// W5-endgame step 2b — MUST stay first; see [`CellHeader`].
    header: CellHeader,
    /// phase-391 W5.3b — BORROWED from per-class or pool storage, not boxed.
    /// The storage is `'static` (macro-emitted static / caller backing), so the
    /// reference outlives every closure that dispatches through it.
    slot: RefCell<&'static mut dyn ComponentSlot>,
    publishers: RefCell<heapless::Vec<(IdStr, EmbeddedRawPublisher), PUBS>>,
    // Phase 212.M-F.23 — declarative service/action CLIENT + action-SERVER
    // handles, keyed by stable entity id, resolved during tick dispatch.
    // Mirror of the orchestration `GenClientDispatch`/`GenActionExec` arrays,
    // but built at registration time on the single-node runtime. Service- and
    // action-SERVER request/goal dispatch is owned by the executor (the
    // trampolines registered in `create_entity`); only the action-server
    // handle is kept here so the tick can complete goals / publish feedback.
    service_clients: RefCell<heapless::Vec<(IdStr, crate::HandleId), SVCS>>,
    action_clients: RefCell<heapless::Vec<(IdStr, usize), ACTC>>,
    action_servers: RefCell<heapless::Vec<(IdStr, crate::ActionServerRawHandle), ACTS>>,
    // W5-endgame ctx slabs (issue 0857) — the leaked trampoline contexts the
    // sink used to `Box::into_raw` live INSIDE the cell, sized by the class's
    // declared bounds, so the macro path allocates nothing. Monotonic
    // counters; a full slab is a loud registration error. The entries die
    // with the cell (executor drop), after which no trampoline can fire.
    svc_ctxs: [UnsafeCell<MaybeUninit<ServiceServerCtx>>; SSRV],
    svc_ctxs_used: core::cell::Cell<usize>,
    act_srv_ctxs: [UnsafeCell<MaybeUninit<ActionServerCtx>>; ACTS],
    act_srv_ctxs_used: core::cell::Cell<usize>,
    act_cli_ctxs: [UnsafeCell<MaybeUninit<ActionClientCtx>>; ACTC],
    act_cli_ctxs_used: core::cell::Cell<usize>,
    // Phase 264 W4c — raw pointer to the executor's volatile parameter store, so a
    // subscription/timer/service/action callback can read `ctx.parameter::<T>(name)`.
    // The callback closures + leaked trampolines hold only a `CellHandle` (the
    // executor is unreachable when they fire), so the store address is threaded HERE by
    // `apply_param_services`' post-pass (mirrors the `run_ticks` disjoint borrow). Null
    // until param services are registered. Stable for the executor's life: the server
    // lives in a `Box<ParamState>`, and since phase-382 W2' its SLOTS live in
    // caller-owned storage it merely borrows — a fixed-length table either way, so a
    // declare never moves anything this pointer or a stored `&ParameterValue` names.
    #[cfg(feature = "param-services")]
    param_server: core::cell::Cell<*const nros_params::ParameterServer<'static>>,
}

impl<const PUBS: usize, const SVCS: usize, const ACTC: usize, const ACTS: usize, const SSRV: usize>
    Drop for ComponentCell<PUBS, SVCS, ACTC, ACTS, SSRV>
{
    fn drop(&mut self) {
        // phase-391 W5.3b — the slot is BORROWED from one-shot storage, so the
        // component state's destructor no longer rides a `Box` drop. Run it
        // here instead: the cell drops exactly once (the executor's drop
        // trampoline runs it in place on the enrolled path; the dynamic
        // path's `Arc` drops it on runtime drop), and the
        // storage slot is never handed out again (`take`/`next_slot` counters
        // are monotonic), so nothing can observe the dropped bytes.
        // W5-endgame ctx slabs — run the placed ctxs' destructors (their
        // `CellHandle` may hold an `Arc` on the pooled arm). Monotonic
        // counters bound the initialized prefix exactly.
        for i in 0..self.svc_ctxs_used.get() {
            // SAFETY: entries [0, used) were initialized by `place_service_ctx`
            // and are dropped exactly once, here.
            unsafe { (*self.svc_ctxs[i].get()).assume_init_drop() };
        }
        for i in 0..self.act_srv_ctxs_used.get() {
            // SAFETY: as above, for `place_action_server_ctx`.
            unsafe { (*self.act_srv_ctxs[i].get()).assume_init_drop() };
        }
        for i in 0..self.act_cli_ctxs_used.get() {
            // SAFETY: as above, for `place_action_client_ctx`.
            unsafe { (*self.act_cli_ctxs[i].get()).assume_init_drop() };
        }
        let slot: &mut &'static mut dyn ComponentSlot = self.slot.get_mut();
        let p: *mut dyn ComponentSlot = &raw mut **slot;
        // SAFETY: `p` targets storage uniquely owned by this cell (handed out
        // at most once), reachable only through the reference being dropped
        // with us; it is dropped at most once because the cell is.
        unsafe { core::ptr::drop_in_place(p) };
    }
}

impl<const PUBS: usize, const SVCS: usize, const ACTC: usize, const ACTS: usize, const SSRV: usize>
    ComponentCell<PUBS, SVCS, ACTC, ACTS, SSRV>
{
    /// Phase 264 W4c — the executor's parameter store, or `None` until
    /// `apply_param_services` threads it in. The deref is sound: single-threaded
    /// executor, param services mutate the server only outside callback dispatch.
    #[cfg(feature = "param-services")]
    fn param_server(&self) -> Option<&nros_params::ParameterServer<'static>> {
        let ptr = self.param_server.get();
        if ptr.is_null() {
            None
        } else {
            // SAFETY: `ptr` is the address of the executor's boxed `ParameterServer`
            // (stable for the executor's life); param services mutate it before/after
            // dispatch (`spin.rs` pre/post), never during, so no aliasing `&mut` is live.
            Some(unsafe { &*ptr })
        }
    }
}

/// phase-391 W5-endgame step 1 (issue 0857) — the NON-GENERIC view every
/// dispatch/tick path consumes instead of the concrete [`ComponentCell`].
///
/// Why a trait and not the cell: the endgame emits a per-class cell whose
/// registries are sized to the class's DECLARED entity counts (const generics,
/// which the "never public to other languages" rule confines to the macro
/// emission), while the dynamic `register_node` path keeps pool-backed cells at
/// the knob caps. Two cell layouts, ONE dispatch implementation — this trait is
/// the seam that makes the split affordable. `Ref<'_, [T]>` erases the
/// `heapless::Vec` capacity; `try_with_slot_mut` erases the slot's storage
/// shape (borrowed `&'static mut dyn` today, fused inline in the per-class
/// static tomorrow).
///
/// Object-safe on purpose: the FFI trampolines carry a THIN `*mut c_void`, so
/// each concrete cell type casts back to itself and only then widens to
/// `&dyn CellView`.
trait CellView {
    /// Bump the dispatch counters (`message` = payload was non-empty).
    fn note_dispatch(&self, message: bool);
    /// The publisher registered under `entity_id`, if this component declared
    /// one. Holds the registry's shared borrow for the returned `Ref`'s life.
    fn publisher(&self, entity_id: &str) -> Option<Ref<'_, EmbeddedRawPublisher>>;
    fn service_clients(&self) -> Ref<'_, [(IdStr, crate::HandleId)]>;
    fn action_clients(&self) -> Ref<'_, [(IdStr, usize)]>;
    fn action_servers(&self) -> Ref<'_, [(IdStr, crate::ActionServerRawHandle)]>;
    /// Run `f` over the component slot unless it is already mutably borrowed
    /// (a re-entrant dispatch on the same cell) — in which case the dispatch
    /// is dropped, exactly as the pre-trait `try_borrow_mut` spelled it.
    fn try_with_slot_mut(&self, f: &mut dyn FnMut(&mut dyn ComponentSlot));
    /// The executor's parameter store, or `None` until `apply_param_services`
    /// threads it in.
    #[cfg(feature = "param-services")]
    fn view_param_server(&self) -> Option<&nros_params::ParameterServer<'static>>;
    /// The `(callbacks, messages)` dispatch counters. Only the dynamic
    /// runtime's stats fold reads them through the trait; the macro path's
    /// fold reads the `CellHeader` directly.
    #[cfg(feature = "alloc")]
    fn dispatch_counts(&self) -> (usize, usize);
    /// Registration-time registry appends. `Err(())` = registry full — the
    /// component declared more entities of that kind than its cell's bound,
    /// which the caller reports loudly (never a silent drop).
    fn push_publisher(&self, id: IdStr, handle: EmbeddedRawPublisher) -> Result<(), ()>;
    fn push_service_client(&self, id: IdStr, handle: crate::HandleId) -> Result<(), ()>;
    fn push_action_client(&self, id: IdStr, entry_index: usize) -> Result<(), ()>;
    fn push_action_server(&self, id: IdStr, handle: crate::ActionServerRawHandle)
    -> Result<(), ()>;
    /// W5-endgame ctx slabs — place a trampoline context into the cell's slab
    /// and return its stable address for the executor's C-ABI registration.
    /// `Err(())` = slab full (the class declared fewer of that kind than
    /// `register()` creates). The entry lives — and is dropped — with the cell.
    fn place_service_ctx(&self, ctx: ServiceServerCtx) -> Result<*mut core::ffi::c_void, ()>;
    fn place_action_server_ctx(&self, ctx: ActionServerCtx) -> Result<*mut core::ffi::c_void, ()>;
    fn place_action_client_ctx(&self, ctx: ActionClientCtx) -> Result<*mut core::ffi::c_void, ()>;
}

impl<const PUBS: usize, const SVCS: usize, const ACTC: usize, const ACTS: usize, const SSRV: usize>
    CellView for ComponentCell<PUBS, SVCS, ACTC, ACTS, SSRV>
{
    fn note_dispatch(&self, message: bool) {
        self.header
            .callback_dispatches
            .fetch_add(1, Ordering::Relaxed);
        if message {
            self.header
                .message_dispatches
                .fetch_add(1, Ordering::Relaxed);
        }
    }

    #[cfg(feature = "alloc")]
    fn dispatch_counts(&self) -> (usize, usize) {
        (
            self.header.callback_dispatches.load(Ordering::Relaxed),
            self.header.message_dispatches.load(Ordering::Relaxed),
        )
    }

    fn push_publisher(&self, id: IdStr, handle: EmbeddedRawPublisher) -> Result<(), ()> {
        self.publishers
            .borrow_mut()
            .push((id, handle))
            .map_err(|_| ())
    }

    fn push_service_client(&self, id: IdStr, handle: crate::HandleId) -> Result<(), ()> {
        self.service_clients
            .borrow_mut()
            .push((id, handle))
            .map_err(|_| ())
    }

    fn push_action_client(&self, id: IdStr, entry_index: usize) -> Result<(), ()> {
        self.action_clients
            .borrow_mut()
            .push((id, entry_index))
            .map_err(|_| ())
    }

    fn push_action_server(
        &self,
        id: IdStr,
        handle: crate::ActionServerRawHandle,
    ) -> Result<(), ()> {
        self.action_servers
            .borrow_mut()
            .push((id, handle))
            .map_err(|_| ())
    }

    fn place_service_ctx(&self, ctx: ServiceServerCtx) -> Result<*mut core::ffi::c_void, ()> {
        let i = self.svc_ctxs_used.get();
        if i >= SSRV {
            return Err(());
        }
        self.svc_ctxs_used.set(i + 1);
        // SAFETY: `i` is claimed exactly once (monotonic counter, single-
        // threaded executor contract), so no other reference to this entry
        // exists; the address is stable because the cell never moves after
        // placement.
        Ok(
            unsafe { (*self.svc_ctxs[i].get()).write(ctx) as *mut ServiceServerCtx }
                as *mut core::ffi::c_void,
        )
    }

    fn place_action_server_ctx(&self, ctx: ActionServerCtx) -> Result<*mut core::ffi::c_void, ()> {
        let i = self.act_srv_ctxs_used.get();
        if i >= ACTS {
            return Err(());
        }
        self.act_srv_ctxs_used.set(i + 1);
        // SAFETY: as `place_service_ctx`.
        Ok(
            unsafe { (*self.act_srv_ctxs[i].get()).write(ctx) as *mut ActionServerCtx }
                as *mut core::ffi::c_void,
        )
    }

    fn place_action_client_ctx(&self, ctx: ActionClientCtx) -> Result<*mut core::ffi::c_void, ()> {
        let i = self.act_cli_ctxs_used.get();
        if i >= ACTC {
            return Err(());
        }
        self.act_cli_ctxs_used.set(i + 1);
        // SAFETY: as `place_service_ctx`.
        Ok(
            unsafe { (*self.act_cli_ctxs[i].get()).write(ctx) as *mut ActionClientCtx }
                as *mut core::ffi::c_void,
        )
    }

    fn publisher(&self, entity_id: &str) -> Option<Ref<'_, EmbeddedRawPublisher>> {
        Ref::filter_map(self.publishers.borrow(), |pubs| {
            pubs.iter().find(|(id, _)| id == entity_id).map(|(_, p)| p)
        })
        .ok()
    }

    fn service_clients(&self) -> Ref<'_, [(IdStr, crate::HandleId)]> {
        Ref::map(self.service_clients.borrow(), |v| v.as_slice())
    }

    fn action_clients(&self) -> Ref<'_, [(IdStr, usize)]> {
        Ref::map(self.action_clients.borrow(), |v| v.as_slice())
    }

    fn action_servers(&self) -> Ref<'_, [(IdStr, crate::ActionServerRawHandle)]> {
        Ref::map(self.action_servers.borrow(), |v| v.as_slice())
    }

    fn try_with_slot_mut(&self, f: &mut dyn FnMut(&mut dyn ComponentSlot)) {
        if let Ok(mut slot) = self.slot.try_borrow_mut() {
            f(*slot);
        }
    }

    #[cfg(feature = "param-services")]
    fn view_param_server(&self) -> Option<&nros_params::ParameterServer<'static>> {
        self.param_server()
    }
}

/// phase-391 W5-endgame step 2b — the two ways a cell is held, one spelling.
///
/// The macro/FFI install path places its cell in per-class static storage and
/// holds `&'static`; the dynamic `register_node` path (alloc-gated, hosted)
/// keeps the `Arc` it always had — its cells are not 0857's embedded heap
/// cost, and `Arc` gives them the drop-on-runtime-drop lifetime the dynamic
/// path needs. Closures and leaked ctxs clone the HANDLE (a ref copy or an
/// `Arc` bump) and dispatch through [`CellHandle::view`].
#[derive(Clone)]
enum CellHandle {
    Static(&'static dyn CellView),
    #[cfg(feature = "alloc")]
    Pooled(Arc<ComponentCell>),
}

impl CellHandle {
    fn view(&self) -> &dyn CellView {
        match self {
            CellHandle::Static(v) => *v,
            #[cfg(feature = "alloc")]
            CellHandle::Pooled(cell) => cell.as_ref(),
        }
    }
}

/// `PublisherResolver` implementation backed by a [`CellView`].
struct CellResolver<'a> {
    cell: &'a dyn CellView,
}

impl PublisherResolver for CellResolver<'_> {
    fn publish_raw(&self, entity_id: &str, data: &[u8]) -> NodeResult<()> {
        // issue 0736 — the two arms report DIFFERENT failures. A miss on
        // `lookup_publisher` means this component declared no publisher for the
        // entity, so nothing ever reached the transport; a `publish_raw` error
        // means the transport had the sample and refused it. Both used to
        // return `Runtime`, which is why a console full of "publish FAILED"
        // could not distinguish a wiring bug from a congested link.
        match self.cell.publisher(entity_id) {
            Some(p) => p.publish_raw(data).map_err(|_| NodeDeclError::Runtime),
            None => Err(NodeDeclError::UnknownPublisher),
        }
    }
}

// Phase 212.M-F.23 — the `UnsupportedActions` / `UnsupportedClients` tick-side
// stubs are retired. Real service/action client + action-server dispatch on the
// single-node runtime lives in `RuntimeClientDispatch` / `RuntimeActions`
// (below), wired into `run_ticks`.

// =============================================================================
// ExecutorNodeRuntime
// =============================================================================

/// Executor-backed component runtime.
///
/// Owns the [`Executor`] and one slot per registered component. The
/// register / spin lifecycle:
///
/// 1. [`from_executor`](Self::from_executor) wraps an open
///    [`Executor`].
/// 2. [`register_node`](Self::register_node) builds the
///    component's `State`, runs [`Node::register`](crate::node::Node::register) over an
///    internal [`NodeRuntime`] adapter that materialises nodes /
///    pubs / subs / timers on the real executor, and wires each
///    subscription + timer callback to dispatch into
///    [`ExecutableNode::on_callback`] with the right
///    [`CallbackId`].
/// 3. [`spin`](Self::spin) / [`spin_once`](Self::spin_once) drive the
///    executor; between iterations every registered component's
///    [`ExecutableNode::tick`] runs.
#[cfg(feature = "alloc")]
pub struct ExecutorNodeRuntime {
    executor: Executor<'static>,
    components: Vec<Arc<ComponentCell>>,
    /// phase-391 W5.3b — slot storage for [`register_node`](Self::register_node).
    /// Always present: the leaking constructor allocates it once, `new_in`
    /// carves the caller's backing.
    pool: ComponentPool,
}

/// Bump view over the runtime's slot backing. Raw pointer, not a slice, so the
/// runtime stays movable — the SLOTS never move; they live in the backing.
#[cfg(feature = "alloc")]
struct ComponentPool {
    base: *mut MaybeUninit<u8>,
    len_bytes: usize,
    per_slot: usize,
    capacity: usize,
    next: usize,
}

// SAFETY: the pool is confined to the runtime (`&mut self` on every use); the
// raw pointer targets `'static` backing handed over at construction.
#[cfg(feature = "alloc")]
unsafe impl Send for ComponentPool {}

#[cfg(feature = "alloc")]
impl ComponentPool {
    /// Next disjoint slot window, or `None` when the pool is full.
    fn next_slot(&mut self) -> Option<&'static mut [MaybeUninit<u8>]> {
        if self.next >= self.capacity {
            return None;
        }
        let off = self.next * self.per_slot;
        debug_assert!(off + self.per_slot <= self.len_bytes);
        self.next += 1;
        // SAFETY: `base` points at `'static` backing of `len_bytes`; the
        // window is in-bounds by the checks above and DISJOINT from every
        // earlier one because `next` only increases.
        Some(unsafe { core::slice::from_raw_parts_mut(self.base.add(off), self.per_slot) })
    }
}

#[cfg(feature = "alloc")]
impl ExecutorNodeRuntime {
    /// Wrap an already-built [`Executor`], LEAKING the slot backing.
    ///
    /// The convenience constructor — allocates `RuntimeSizing::DEFAULT`'s
    /// backing once and never frees it, exactly the relationship
    /// `Executor::from_session` has to `Executor::open_in`. Use
    /// [`new_in`](Self::new_in) on an image that must not allocate.
    pub fn from_executor(executor: Executor<'static>) -> Self {
        let sizing = crate::runtime_storage::RuntimeSizing::DEFAULT;
        let backing: &'static mut [MaybeUninit<u64>] =
            Vec::leak(alloc::vec![MaybeUninit::uninit(); sizing.u64_len()]);
        // SAFETY: freshly leaked — `'static`, uniquely owned, exactly
        // `u64_len()` words.
        unsafe { Self::new_in(executor, backing, sizing) }
    }

    /// Wrap an already-built [`Executor`] over CALLER-SUPPLIED slot storage.
    ///
    /// Size `backing` with [`crate::runtime_storage::RuntimeSizing::u64_len`]; a short one panics,
    /// naming both sizes (fail-loud on every profile — a short backing is
    /// silent corruption, the `executor::storage::carve` / issue #131 lesson).
    ///
    /// # Safety
    /// `backing` must be uniquely owned by this runtime for its whole life:
    /// slots carved from it are handed out as `&'static mut`, so aliasing it
    /// anywhere else is undefined behaviour.
    pub unsafe fn new_in(
        executor: Executor<'static>,
        backing: &'static mut [MaybeUninit<u64>],
        sizing: crate::runtime_storage::RuntimeSizing,
    ) -> Self {
        let need = sizing.u64_len();
        assert!(
            backing.len() >= need,
            "component pool backing too small: {} u64 words < {} required for {} slot(s) \
             of {} bytes — size it with RuntimeSizing::u64_len()",
            backing.len(),
            need,
            sizing.components,
            sizing.slot_bytes,
        );
        Self {
            executor,
            components: Vec::new(),
            pool: ComponentPool {
                base: backing.as_mut_ptr() as *mut MaybeUninit<u8>,
                len_bytes: backing.len() * 8,
                per_slot: (sizing.slot_bytes.div_ceil(8) * 8).max(1),
                capacity: sizing.components,
                next: 0,
            },
        }
    }

    /// Borrow the underlying executor.
    pub fn executor(&self) -> &Executor<'static> {
        &self.executor
    }

    /// Mutably borrow the underlying executor — for advanced wiring
    /// (parameter services, custom guard conditions). Don't use during
    /// [`spin`](Self::spin) from another thread; the runtime is
    /// single-threaded.
    pub fn executor_mut(&mut self) -> &mut Executor<'static> {
        &mut self.executor
    }

    /// RFC-0052 / phase-296 W5.4 — lower a tier's RTOS-agnostic scheduling
    /// policy onto this executor's DEFAULT scheduling context. One `Executor`
    /// per tier means "the tier's policy" == "this executor's default SC";
    /// per-group/per-handle bindings still take precedence.
    ///
    /// **Portable** across every board — call from each board's `run_tiers`
    /// after building the runtime. Takes the tier fields as primitives (not a
    /// `TierSpec`) so `nros` needs no board/platform dependency; a board passes
    /// `tier.class`, `tier.period_us`, … straight through.
    ///
    /// `real_time` + `budget_us` + `period_us` → [`SchedClass::Sporadic`](crate::SchedClass::Sporadic);
    /// `best_effort` → [`SchedClass::BestEffort`](crate::SchedClass::BestEffort); `time_triggered` +
    /// `period_us` → the cyclic dispatcher (major frame = period, window =
    /// `budget_us` or the whole frame); `deadline_us` sets the SC deadline and
    /// `deadline_policy` its action. A tier with no class/budget/deadline
    /// leaves the default `Fifo` SC untouched (byte-identical pre-W3 behavior).
    pub fn apply_tier_sched_policy(
        &mut self,
        class: Option<&str>,
        period_us: Option<u64>,
        budget_us: Option<u64>,
        deadline_us: Option<u64>,
        deadline_policy: Option<&str>,
    ) {
        use crate::SchedContext;
        // Common backend (RFC-0052): the tier→SchedContext lowering lives ONCE
        // in `SchedContext::from_tier_policy`, shared with the C / C++ entries
        // (`nros_{c,cpp}_create_sched_context_from_policy`) so the mapping never
        // drifts between languages. `None` → keep the default `Fifo` SC.
        let Some((sc, tt_frame)) = SchedContext::from_tier_policy(
            class,
            period_us,
            budget_us,
            deadline_us,
            deadline_policy,
        ) else {
            return;
        };
        if let Some(frame_us) = tt_frame {
            self.executor_mut()
                .register_time_triggered_dispatcher(frame_us);
        }
        self.executor_mut().set_default_sched_context(sc);
    }

    /// Number of registered components.
    pub fn component_count(&self) -> usize {
        self.components.len()
    }

    /// Register a [`Node`](crate::node::Node) (which must also be
    /// [`ExecutableNode`]) into this runtime. Builds the
    /// component's `State` (via [`ExecutableNode::init`]) and
    /// walks [`Node::register`](crate::node::Node::register) over the live executor — every
    /// declared node / pub / sub / timer materialises as a real
    /// executor handle, and subscription + timer callbacks are wired
    /// to dispatch into [`ExecutableNode::on_callback`].
    pub fn register_node<C: ExecutableNode + 'static>(&mut self) -> NodeResult<RegisteredNode<C>>
    where
        C::State: 'static,
    {
        // phase-391 W5.3b — draw slot storage from the runtime's pool instead
        // of boxing. Full → the executor-table-Full class (raise
        // NROS_RUNTIME_MAX_COMPONENTS); too big for `slot_bytes` → likewise a
        // registration error (raise NROS_RUNTIME_COMPONENT_SLOT_BYTES), never
        // a truncation.
        let raw = self.pool.next_slot().ok_or(NodeDeclError::ExecutorFull)?;
        if raw.len() < core::mem::size_of::<TypedSlot<C>>()
            || !(raw.as_ptr() as usize).is_multiple_of(core::mem::align_of::<TypedSlot<C>>())
        {
            return Err(NodeDeclError::Runtime);
        }
        // SAFETY: sized + aligned for `TypedSlot<C>` by the check above;
        // uniquely owned (`next_slot` windows are disjoint); `'static` backing.
        let slot_mu: &'static mut MaybeUninit<TypedSlot<C>> =
            unsafe { &mut *(raw.as_mut_ptr() as *mut MaybeUninit<TypedSlot<C>>) };
        let cell = Arc::new(ComponentCell {
            slot: RefCell::new(place_slot::<C>(slot_mu)),
            publishers: RefCell::new(heapless::Vec::new()),
            service_clients: RefCell::new(heapless::Vec::new()),
            action_clients: RefCell::new(heapless::Vec::new()),
            action_servers: RefCell::new(heapless::Vec::new()),
            svc_ctxs: [const { UnsafeCell::new(MaybeUninit::uninit()) }; _],
            svc_ctxs_used: core::cell::Cell::new(0),
            act_srv_ctxs: [const { UnsafeCell::new(MaybeUninit::uninit()) }; _],
            act_srv_ctxs_used: core::cell::Cell::new(0),
            act_cli_ctxs: [const { UnsafeCell::new(MaybeUninit::uninit()) }; _],
            act_cli_ctxs_used: core::cell::Cell::new(0),
            header: CellHeader {
                callback_dispatches: AtomicUsize::new(0),
                message_dispatches: AtomicUsize::new(0),
            },
            // W4c — set by `apply_param_services` once the store exists.
            #[cfg(feature = "param-services")]
            param_server: core::cell::Cell::new(core::ptr::null()),
        });
        let component_idx = self.components.len();
        self.components.push(cell.clone());

        let mut sink = ExecutorSink {
            executor: &mut self.executor,
            cell: CellHandle::Pooled(cell.clone()),
            nodes: heapless::Vec::new(),
            node_identity: None, // direct API — no launch injection
            remaps: &[],         // direct API — no launch remaps
            qos_overrides: &[],  // direct API — no plan overrides
        };
        let sink_dyn: &mut dyn NodeRuntime = &mut sink;
        let mut context = NodeContext::new(C::NAME, sink_dyn);
        let result = C::register(&mut context);
        if result.is_err() {
            // Roll back the slot push so `component_count` stays
            // consistent with what users observe.
            self.components.pop();
        }
        result?;

        // W4c — capture the executor's volatile param store on the cell (if param
        // services were registered before this call) so the node's callbacks can read
        // `ctx.parameter::<T>(name)`. Mirrors the install-seam path.
        #[cfg(feature = "param-services")]
        if let Some(server) = self.executor.params() {
            cell.param_server.set(server as *const _);
        }

        Ok(RegisteredNode {
            component_idx,
            _phantom: PhantomData,
        })
    }

    // Phase 258 (Track 2, w5) — `register_dispatch_slot` (the four-fn-ptr
    // BSP registration) is gone with the retired `register_dispatch_slot_dyn`
    // bridge + `nros_run_components`. Owned-spin / BSP entries now register
    // through `install_node_typed` (the uniform install seam).

    /// Drive one executor iteration + a `tick` per registered
    /// component.
    pub fn spin_once(&mut self, timeout: Duration) -> Result<(), ExecutorError> {
        let _result = self.executor.spin_once(timeout);
        self.run_ticks();
        Ok(())
    }

    /// [`Self::spin_once`], returning the executor's own per-iteration counts
    /// instead of discarding them — issue 0572.
    ///
    /// `spin_once` throws away a `SpinOnceResult` that already carries exactly
    /// what a stalled tier needs to be diagnosed: how many timers fired, how
    /// many subscription callbacks ran, and how many errored. Without it, "the
    /// tier's timer never fires" and "the tier's callback runs and its publish
    /// fails" are the same observation from outside the guest — a silent topic.
    pub fn spin_once_counted(
        &mut self,
        timeout: Duration,
    ) -> Result<nros_node::SpinOnceResult, ExecutorError> {
        let result = self.executor.spin_once(timeout);
        self.run_ticks();
        Ok(result)
    }

    /// Phase 216.B.3 / C.3 follow-up — route a signaled callback to
    /// every registered component slot.
    ///
    /// The RTIC (`nros-board-rtic-stm32f4`) and Embassy
    /// (`nros-board-embassy-stm32f4`) dispatch tasks dequeue a
    /// [`nros_platform::SignaledCallback`] envelope from their SPSC
    /// queue / Embassy channel and need a routing entry point that
    /// hands the callback off to the right Node's `on_callback`
    /// trampoline. This method is that entry point.
    ///
    /// # Strategy — linear scan
    ///
    /// Each registered slot's `dispatch_fn` is the codegen-emitted
    /// `d()` trampoline from `nros::node!()` (see
    /// `packages/core/nros-macros/src/lib.rs`). That trampoline calls
    /// `<NodeTy as ExecutableNode>::on_callback`, whose body
    /// `match`es on the callback's own tag set
    /// (`Subscription` / `Timer` / `Service` / `Action` ids) and is a
    /// no-op for non-matching `cb_id`s. So a linear scan across every
    /// slot is correct — each slot self-filters and at most one
    /// component actually acts on a given `cb_id`. A focused
    /// `cb_id → slot` index is a separate follow-up; the trampoline's
    /// tag dispatch already gates the real work cheaply (string
    /// compare on statically known literals), so the linear scan is
    /// the minimum-viable wiring that closes the conceptual gap left
    /// by the B.3 / C.3 skeleton emits.
    ///
    /// # Borrow semantics
    ///
    /// Each `ComponentCell`'s slot lives behind a [`RefCell`]; the
    /// per-slot dispatch takes `try_borrow_mut` and is a no-op on
    /// re-entrancy. The runtime is single-threaded by construction
    /// (the dispatch task owns it via `&mut self`), so the borrow
    /// always succeeds in normal flow.
    pub fn dispatch_callback(&mut self, cb_id: &str, ctx: &mut CallbackCtx<'_>) {
        for cell in &self.components {
            if let Ok(mut slot) = cell.slot.try_borrow_mut() {
                slot.dispatch(cb_id, ctx);
            }
        }
    }

    /// Spin until the executor's halt flag is raised.
    ///
    /// phase-359 W10 — this used to be `#[cfg(feature = "std")]` and described
    /// itself as hosted-only, but the body is `Duration` + `spin_once` +
    /// `is_halted`, none of which need an OS. The gate was describing a
    /// CONVENTION (a BSP usually wants its own loop so it can interleave
    /// board work) as if it were a requirement, and a bare-metal image that
    /// wants exactly this loop had to hand-roll it to get it.
    ///
    /// It does need `alloc`: the halt flag is an `Arc`, so a core-only image
    /// has no flag to poll.
    #[cfg(feature = "alloc")]
    pub fn spin(&mut self) -> Result<(), ExecutorError> {
        // 10 ms tick cadence — matches the existing executor spin
        // budgeting (see `Executor::spin_default`); short enough that
        // component `tick` hooks observe latency under one cycle.
        let tick = Duration::from_millis(10);
        while !self.executor.is_halted() {
            let _ = self.executor.spin_once(tick);
            self.run_ticks();
        }
        Ok(())
    }

    /// Halt a running [`spin`](Self::spin). Idempotent.
    ///
    /// phase-417 W4.c — forwards to `Executor::cancel()`, the rclcpp spelling.
    /// `Executor::halt` is now a deprecated forwarder onto it, so naming the old
    /// spelling here would be this crate warning at itself.
    #[cfg(feature = "alloc")]
    pub fn halt(&self) {
        self.executor.cancel();
    }

    fn run_ticks(&mut self) {
        // Per-component tick — each component's resolver is its own cell.
        // Phase 212.M-F.23: the tick reaches the executor (service-client
        // call_raw poll, action-server complete/feedback) through a raw
        // pointer so `&self.components` and `&mut self.executor` (disjoint
        // fields) can be live at once.
        let exec_ptr: *mut Executor<'static> = &mut self.executor;
        for cell in &self.components {
            tick_one_cell(cell.as_ref(), exec_ptr);
        }
    }
}

/// Phase 258 (Track 2, 2a) — drive one component cell's `tick` against the
/// executor. The single source of truth for the per-component tick body,
/// shared by [`ExecutorNodeRuntime::run_ticks`] (the owned-runtime path) and
/// the executor-enrolled [`component_tick_trampoline`] (the `install` path).
///
/// `exec_ptr` is reached through a raw `*mut Executor<'static>` so the caller can hold
/// the component (`&ComponentCell`) and the executor live at once — they are
/// disjoint, and `RuntimeActions` / `RuntimeClientDispatch` reborrow `&mut`
/// per call (see their docs).
fn tick_one_cell(cell: &dyn CellView, exec_ptr: *mut Executor<'static>) {
    let resolver = CellResolver { cell };
    let service_clients = cell.service_clients();
    let action_clients = cell.action_clients();
    let action_servers = cell.action_servers();
    let mut actions = RuntimeActions {
        executor: exec_ptr,
        handles: &action_servers,
    };
    let mut clients = RuntimeClientDispatch {
        executor: exec_ptr,
        services: &service_clients,
        actions: &action_clients,
    };
    let mut ctx = TickCtx::new(&resolver, &mut actions, &mut clients);
    // W4c — `tick` reads `ctx.parameter::<T>(name)` from the store via the cell pointer
    // (the store is a separate `Box<ParamState>` allocation, so this does NOT alias the
    // `&mut Executor<'static>` the action/client tick calls reborrow through `exec_ptr`).
    #[cfg(feature = "param-services")]
    ctx.set_param_server(cell.view_param_server());
    cell.try_with_slot_mut(&mut |slot| slot.tick(&mut ctx));
}

/// Phase 258 (Track 2, 2a) — executor `ComponentSlot.tick` trampoline. Casts
/// the enrolled state back to the component cell + `exec_ctx` back to the
/// executor and drives one tick. The layering-clean `extern "C"` shim the
/// `nros-node` [`Executor`] calls each `spin_once` (it can't name `nros`'s
/// [`ComponentCell`] — see [`register_node_borrowed`]'s enroll).
///
/// # Safety
/// `state` must be the placed `ComponentCell` enrolled via
/// [`Executor::enroll_component`] from [`register_node_borrowed`] (per-class
/// static storage or the alloc convenience's leaked box — live until
/// `component_drop_trampoline`); `exec_ctx` must be the live
/// `*mut Executor<'static>` the executor passes itself.
unsafe extern "C" fn component_tick_trampoline<
    const PUBS: usize,
    const SVCS: usize,
    const ACTC: usize,
    const ACTS: usize,
    const SSRV: usize,
>(
    state: *mut core::ffi::c_void,
    exec_ctx: *mut core::ffi::c_void,
) {
    // SAFETY: `state` is a live placed `ComponentCell` (kept in place until
    // the drop trampoline); this monomorphization was enrolled alongside it,
    // so the cast target is the cell's true type.
    let cell = unsafe { &*(state as *const ComponentCell<PUBS, SVCS, ACTC, ACTS, SSRV>) };
    tick_one_cell(cell, exec_ctx as *mut Executor<'static>);
}

/// Phase 258 (Track 2, 2a; W5-endgame step 2b) — executor `ComponentSlot.drop`
/// trampoline. Drops the placed `ComponentCell` (which drops the component
/// state through its borrowed slot) IN PLACE — the storage itself is a
/// `static` (or the alloc convenience's leaked box, whose bytes are never
/// freed, matching its slot's documented leak). Run exactly once on
/// `Executor::drop`; the storage's monotonic `take` never re-hands the
/// region out, so nothing can observe the dropped bytes.
///
/// # Safety
/// `state` must be the placed `ComponentCell` enrolled via
/// [`Executor::enroll_component`], not yet dropped.
unsafe extern "C" fn component_drop_trampoline<
    const PUBS: usize,
    const SVCS: usize,
    const ACTC: usize,
    const ACTS: usize,
    const SSRV: usize,
>(
    state: *mut core::ffi::c_void,
) {
    // SAFETY: enroll's contract above; dropped exactly once by the executor,
    // through the monomorphization enrolled with the cell.
    unsafe { core::ptr::drop_in_place(state as *mut ComponentCell<PUBS, SVCS, ACTC, ACTS, SSRV>) };
}

// =============================================================================
// Phase 212.N.7 step-3.3 — bridge to platform-side `NodeDispatchRuntime`.
// =============================================================================
//
// `nros_platform::NodeDispatchRuntime` is the board-side sink: object-safe +
// `no_std`. `BoardEntry::run` installs this `ExecutorNodeRuntime` impl on the
// per-boot `RuntimeCtx::runtime` slot. The owned-spin entry reaches the live
// executor through `executor_handle()` (a raw pointer crosses the layering wall
// cleanly) and installs via `nros::install_node_typed`. Phase 258 (w5) retired
// the old `register_dispatch_slot_dyn` four-fn-ptr bridge.

#[cfg(feature = "alloc")]
impl ::nros_platform::NodeDispatchRuntime for ExecutorNodeRuntime {
    fn spin_once(&mut self, timeout_ms: u32) -> Result<(), ()> {
        Self::spin_once(self, Duration::from_millis(timeout_ms.into())).map_err(|_| ())
    }

    fn executor_handle(&mut self) -> *mut core::ffi::c_void {
        // Phase 258 (Track 2, 2a) — hand the owned-spin entry a raw pointer to
        // the executor this runtime owns, so a Node pkg's `register(runtime)`
        // can install through `nros::install_node_typed` (same seam as the
        // C/C++ typed entries). The pointer is valid for the runtime's life
        // (the executor is an inline field); the install call uses it only
        // during registration, before any concurrent spin.
        &mut self.executor as *mut Executor<'static> as *mut core::ffi::c_void
    }

    // Phase 264 W2 — register the REP-2002 lifecycle services + drive boot
    // autostart on the owned executor (mirrors `generate.rs::render_lifecycle_fn`).
    // Only compiled with `lifecycle-services`; without it the trait default no-op
    // applies, so a `[lifecycle]` block is silently inert (the Entry opts in by
    // enabling `nros/lifecycle-services`). `nros::main!` calls this when
    // `system.toml` declares `[lifecycle]`.
    #[cfg(feature = "lifecycle-services")]
    fn apply_lifecycle(&mut self, autostart: u8) -> Result<(), &'static str> {
        // issue 0460 — carry WHY. The caller can only say which capability
        // failed; without this the whole diagnostic was
        // `NodeRegister("lifecycle")`.
        self.executor
            .register_lifecycle_services()
            .map_err(|e| capability_reason(&e))?;
        if autostart >= 1
            && let Some(sm) = self.executor.lifecycle_state_machine_mut()
        {
            // No transition callbacks registered → each transition takes the
            // default-success path (REP-2002 skeleton), as the bake does.
            unsafe {
                let _ = sm.trigger_transition(crate::LifecycleTransition::Configure);
                if autostart >= 2 {
                    let _ = sm.trigger_transition(crate::LifecycleTransition::Activate);
                }
            }
        }
        Ok(())
    }

    // Phase 264 W4b — register the 6 ROS 2 parameter services on the owned executor
    // + seed the volatile param store with the launch-baked `<param>` initials
    // (mirrors `generate.rs::render_param_persistence_fn`, minus persistence). Only
    // compiled with `param-services`; without it the trait default no-op applies, so a
    // `[param_services]` block is silently inert (the Entry opts in by enabling
    // `nros/param-services`). `nros::main!` calls this when `system.toml` declares
    // `[param_services]`. Reconfigured values (via `ros2 param set`) live in RAM until
    // the next boot — persistence is out of scope (issue 0080).
    // W4c note: `nros::main!` emits this BEFORE the per-node `register` calls, so the
    // store exists when each cell is created — `register_node_borrowed` / `register_node`
    // then capture the (stable, boxed) `ParameterServer` address on the cell, letting a
    // callback read `ctx.parameter::<T>(name)`. (The macro-path cells live in the
    // executor's tick registry, not `self.components`, so a post-pass here wouldn't reach
    // them — capture-at-registration does.)
    #[cfg(feature = "param-services")]
    fn apply_param_services(&mut self, params: &[(&str, &str)]) -> Result<(), &'static str> {
        self.executor
            .register_parameter_services()
            .map_err(|e| capability_reason(&e))?;
        for (name, raw) in params {
            self.executor
                .declare_parameter(name, infer_param_value(raw));
        }
        Ok(())
    }

    fn observed_callback_counts(&self) -> (usize, usize) {
        let direct = self
            .components
            .iter()
            .fold((0, 0), |(callbacks, messages), cell| {
                let (c, m) = cell.dispatch_counts();
                (callbacks + c, messages + m)
            });
        // issue #140 — install-seam components (`nros::node!` →
        // `install_node_typed*` → `register_node_borrowed`) never enter
        // `self.components`; their cells live only as the executor's enrolled
        // component slots (leaked `Arc<ComponentCell>`s). Without folding them
        // the hosted spin reported callbacks=0 for every macro-baked entry
        // (multihost robot2 et al.) while dispatch demonstrably ran. The two
        // populations are disjoint by construction: `register_node` pushes to
        // `components` and does not enroll; `register_node_borrowed` enrolls
        // and does not push.
        self.executor
            .enrolled_component_states()
            .fold(direct, |(callbacks, messages), state| {
                // SAFETY: every enrolled state is a leaked `Arc<ComponentCell>`
                // from `register_node_borrowed` (the only enroll site); the
                // executor keeps it alive until `Executor::drop`, and we only
                // read its atomic counters here.
                let header = unsafe { &*(state as *const CellHeader) };
                (
                    callbacks + header.callback_dispatches.load(Ordering::Relaxed),
                    messages + header.message_dispatches.load(Ordering::Relaxed),
                )
            })
    }
}

// =============================================================================
// Internal sink — bridges `NodeRuntime` declarations onto the
// live executor.
// =============================================================================

struct ExecutorSink<'a> {
    executor: &'a mut Executor<'static>,
    cell: CellHandle,
    /// Per-registration node mapping: stable id → executor `NodeId` plus the
    /// EFFECTIVE `(name, namespace)` the node was created with (launch identity
    /// or the `NodeOptions` default) — phase-306 W3 needs it to expand
    /// `~`/relative entity names per node.
    /// W5-endgame alloc-off — heapless: a component class declares a small,
    /// statically bounded number of NODES (nearly always one). Full = loud
    /// registration error, the registries' rule.
    nodes: heapless::Vec<SinkNode, MAX_SINK_NODES>,
    /// Phase 268 W1 — launch-injected node identity `(name, namespace)` baked by
    /// `nros::main!` per component. When `Some`, `create_node` uses this identity
    /// instead of the `NodeOptions` default; `None` → default stands (backward-compat).
    node_identity: Option<(&'static str, &'static str)>,
    /// Phase 305 W3 (issue 0255) — launch `<remap from= to=/>` rules baked by
    /// `nros::main!` for this component. Applied in `create_entity` via the
    /// shared `node_metadata::resolve_name` seam (exact-FQN match, first rule
    /// wins). Empty → names still get ROS 2 expansion, no substitution.
    remaps: &'a [(&'a str, &'a str)],
    /// Issue #52 — the component's baked QoS-override codes, installed on each
    /// node this sink creates (`Executor::set_node_qos_overrides`) BEFORE the
    /// component declares entities, so `create_publisher`/`create_subscription`
    /// fold the matching ones in. Empty → nothing installed, zero cost.
    qos_overrides: &'static [nros_node::executor::node_record::QoSOverrideCode],
}

struct SinkNode {
    stable_id: IdStr,
    node_id: nros_node::executor::NodeId,
    name: IdStr,
    namespace: IdStr,
}

impl ExecutorSink<'_> {
    fn lookup_node(&self, stable_id: &str) -> Option<&SinkNode> {
        self.nodes.iter().find(|n| n.stable_id == stable_id)
    }
}

impl NodeRuntime for ExecutorSink<'_> {
    fn create_node(&mut self, id: MetaNodeId<'_>, options: NodeOptions<'_>) -> NodeResult<()> {
        if self.nodes.iter().any(|n| n.stable_id == id.as_str()) {
            return Err(NodeDeclError::Runtime);
        }
        // Phase 268 W1 — launch wins over the NodeOptions default (RFC-0046).
        // When `nros::main!` injected an identity for this component, use it;
        // otherwise fall back to what the Node declared in its `create_node` call.
        let (name, ns) = match self.node_identity {
            Some((n, s)) => (n, s),
            None => (options.name, options.namespace),
        };
        let node_id = self
            .executor
            .node_builder(name)
            .namespace(ns)
            .domain_id(options.domain_id)
            .build()
            .map_err(decl_err_from_node)?;
        // Issue #52 — install the bake BEFORE the component declares any
        // entity on this node; entities created earlier could not be folded.
        if !self.qos_overrides.is_empty() {
            self.executor
                .set_node_qos_overrides(node_id, self.qos_overrides);
        }
        self.nodes
            .push(SinkNode {
                stable_id: id_str(id.as_str())?,
                node_id,
                name: id_str(name)?,
                namespace: id_str(ns)?,
            })
            .map_err(|_| NodeDeclError::Runtime)?;
        Ok(())
    }

    fn create_entity(&mut self, metadata: EntityMetadata) -> NodeResult<()> {
        // Phase 228.C tier gate: when this executor runs a specific tier
        // (`active_groups` set by codegen), an entity whose callback group
        // is not active on this tier is a no-op — no RMW handle, no slot.
        // An unlabeled entity (`callback_group == None`) is wildcard-eligible
        // and always registers; the degenerate single-tier executor leaves
        // `active_groups == None`, so every entity registers (byte-identical
        // to pre-228 output).
        if let Some(group) = metadata.callback_group.as_ref()
            && !self.executor.group_active(group.as_str())
        {
            return Ok(());
        }
        let (node, node_name, node_ns) = {
            let entry = self
                .lookup_node(metadata.node_id.as_str())
                .ok_or(NodeDeclError::Runtime)?;
            (entry.node_id, entry.name.clone(), entry.namespace.clone())
        };
        // Phase 305 W3 (issue 0255) — expand `~`/relative source names against
        // the owning node's identity and apply the launch remap rules (shared
        // `node_metadata::resolve_name` seam) before any name reaches the wire.
        // Timers have no wire name; parameter names are NOT remapped (matches
        // ROS 2 basic name remapping — param remaps are a separate rule class).
        let resolved_name = match metadata.kind {
            EntityKind::Timer | EntityKind::Parameter => None,
            _ => Some(
                crate::node_metadata::resolve_name(
                    metadata.source_name.as_str(),
                    &node_name,
                    &node_ns,
                    self.remaps.iter().copied(),
                )
                .map_err(|_| NodeDeclError::Runtime)?,
            ),
        };
        let entity_name = resolved_name.as_ref().map(|r| r.as_str()).unwrap_or("");
        match metadata.kind {
            EntityKind::Publisher => {
                // Issue 0306 — the node's DECLARED profile
                // (`create_publisher_for_topic_with_qos`) rides
                // `metadata.qos`; this used to call the default-QoS
                // constructor, so every declarative entity was created
                // `QoSProfile::default()` and a node's own QoS was silently
                // discarded. Plan overrides still win: they fold in below this,
                // inside the executor's create path.
                let handle = self
                    .executor
                    .node_mut(node)
                    .create_generic_publisher_with_qos(
                        entity_name,
                        metadata.type_name,
                        metadata.type_hash,
                        metadata.qos,
                    )
                    .map_err(decl_err_from_node)?;
                let id_owned = id_str(metadata.id.as_str())?;
                // Registry full = this component declared more entities than
                // CELL_REG_CAP — a registration error, never a silent drop.
                self.cell
                    .view()
                    .push_publisher(id_owned, handle)
                    .map_err(|_| NodeDeclError::Runtime)?;
                Ok(())
            }
            EntityKind::Subscription => {
                let cb_id = metadata
                    .callback_id
                    .as_ref()
                    .ok_or(NodeDeclError::Runtime)?;
                let cb_id_owned = id_str(cb_id.as_str())?;
                let cell = self.cell.clone();
                // Phase 250 (Wave 2b) — a `.safety()` subscription registers via
                // the integrity-aware generic path so `CallbackCtx::integrity()`
                // surfaces CRC + sequence gap/dup. Gated: when `safety-e2e` is off
                // the flag is ignored and the basic path below runs.
                #[cfg(feature = "safety-e2e")]
                if metadata.safety {
                    let cell_s = self.cell.clone();
                    let cb_s = cb_id_owned.clone();
                    self.executor
                        .node_mut(node)
                        .create_generic_subscription_with_integrity(
                            entity_name,
                            metadata.type_name,
                            metadata.type_hash,
                            move |payload: &[u8], status: &nros_node::IntegrityStatus| {
                                dispatch_into_cell_with_integrity(
                                    cell_s.view(),
                                    &cb_s,
                                    payload,
                                    status,
                                );
                            },
                        )
                        .map_err(decl_err_from_node)?;
                    return Ok(());
                }
                // Issue 0306 — same as the publisher branch: honour the
                // node's declared profile instead of defaulting it.
                self.executor
                    .node_mut(node)
                    .create_generic_subscription_with_qos(
                        entity_name,
                        metadata.type_name,
                        metadata.type_hash,
                        metadata.qos,
                        move |payload: &[u8]| {
                            dispatch_into_cell(cell.view(), &cb_id_owned, payload);
                        },
                    )
                    .map_err(decl_err_from_node)?;
                Ok(())
            }
            EntityKind::Timer => {
                let cb_id = metadata
                    .callback_id
                    .as_ref()
                    .ok_or(NodeDeclError::Runtime)?;
                let cb_id_owned = id_str(cb_id.as_str())?;
                // Issue #505 — prefer the microsecond field; fall back to
                // the millisecond one only for metadata that predates it.
                let period = match metadata.period_us {
                    Some(us) => nros_node::TimerDuration::from_micros(us),
                    None => nros_node::TimerDuration::from_millis(
                        metadata.period_ms.ok_or(NodeDeclError::Runtime)?,
                    ),
                };
                let cell = self.cell.clone();
                self.executor
                    .register_timer(period, move || {
                        dispatch_into_cell(cell.view(), &cb_id_owned, &[]);
                    })
                    .map_err(decl_err_from_node)?;
                Ok(())
            }
            // Phase 212.M-F.23 — service / action client + server dispatch on
            // the single-node runtime. The executor-level `register_*_on`
            // calls add an arena dispatch entry, so inbound requests / goals
            // are serviced inside `spin_once`; the leaked `*Ctx` trampoline
            // contexts bridge back into the component's `on_callback`. Client
            // handles are stashed in the cell for the tick-side dispatch
            // (`RuntimeClientDispatch` / `RuntimeActions` in `run_ticks`).
            EntityKind::ServiceServer => {
                let cb_id = metadata
                    .callback_id
                    .as_ref()
                    .ok_or(NodeDeclError::Runtime)?;
                let ctx = self
                    .cell
                    .view()
                    .place_service_ctx(ServiceServerCtx {
                        cell: self.cell.clone(),
                        callback_id: id_str(cb_id.as_str())?,
                    })
                    .map_err(|_| NodeDeclError::Runtime)?;
                self.executor
                    .register_service_raw_sized_on::<1024, 1024>(
                        node,
                        entity_name,
                        metadata.type_name,
                        metadata.type_hash,
                        crate::QoSProfile::services_default(),
                        service_server_trampoline,
                        ctx,
                    )
                    .map_err(decl_err_from_node)?;
                Ok(())
            }
            EntityKind::ServiceClient => {
                let hid = self
                    .executor
                    .register_service_client_raw_sized_on::<1024>(
                        node,
                        entity_name,
                        metadata.type_name,
                        metadata.type_hash,
                        crate::QoSProfile::services_default(),
                        None,
                        core::ptr::null_mut(),
                    )
                    .map_err(decl_err_from_node)?;
                self.cell
                    .view()
                    .push_service_client(id_str(metadata.id.as_str())?, hid)
                    .map_err(|_| NodeDeclError::Runtime)?;
                Ok(())
            }
            EntityKind::ActionServer => {
                let goal_cb = metadata
                    .callback_id
                    .as_ref()
                    .ok_or(NodeDeclError::Runtime)?;
                let cancel_cb = metadata
                    .action_cancel_callback_id
                    .as_ref()
                    .ok_or(NodeDeclError::Runtime)?;
                let accepted_cb = metadata
                    .action_accepted_callback_id
                    .as_ref()
                    .map(|c| id_str(c.as_str()))
                    .transpose()?;
                let ctx = self
                    .cell
                    .view()
                    .place_action_server_ctx(ActionServerCtx {
                        cell: self.cell.clone(),
                        goal_callback_id: id_str(goal_cb.as_str())?,
                        cancel_callback_id: id_str(cancel_cb.as_str())?,
                        accepted_callback_id: accepted_cb,
                    })
                    .map_err(|_| NodeDeclError::Runtime)?;
                let handle = self
                    .executor
                    .register_action_server_raw_sized::<1024, 1024, 1024, 4>(
                        crate::RawActionServerSpec {
                            node_id: Some(node),
                            action_name: entity_name,
                            type_name: metadata.type_name,
                            type_hash: metadata.type_hash,
                            qos: crate::QoSProfile::services_default(),
                            goal_callback: action_goal_trampoline,
                            cancel_callback: action_cancel_trampoline,
                            accepted_callback: Some(action_accepted_trampoline),
                            context: ctx,
                        },
                    )
                    .map_err(decl_err_from_node)?;
                self.cell
                    .view()
                    .push_action_server(id_str(metadata.id.as_str())?, handle)
                    .map_err(|_| NodeDeclError::Runtime)?;
                Ok(())
            }
            EntityKind::ActionClient => {
                // A bound `callback_id` (set by
                // `create_action_client_with_callbacks_for_name`) delivers the
                // terminal goal result to the component via `on_callback`; the
                // optional `action_accepted_callback_id` slot carries the
                // feedback callback (reused — unused on a client). The executor
                // auto-drives accept → feedback → result during spin and invokes
                // these trampolines. No callbacks → send-goal only.
                let (result_callback, feedback_callback, ctx) = match metadata.callback_id.as_ref()
                {
                    Some(result_cb) => {
                        let feedback_cb = metadata
                            .action_accepted_callback_id
                            .as_ref()
                            .map(|c| id_str(c.as_str()))
                            .transpose()?;
                        let ctx = self
                            .cell
                            .view()
                            .place_action_client_ctx(ActionClientCtx {
                                cell: self.cell.clone(),
                                result_callback_id: id_str(result_cb.as_str())?,
                                feedback_callback_id: feedback_cb.clone(),
                            })
                            .map_err(|_| NodeDeclError::Runtime)?;
                        let fb = feedback_cb.map(|_| action_feedback_trampoline as _);
                        (Some(action_result_trampoline as _), fb, ctx)
                    }
                    None => (None, None, core::ptr::null_mut()),
                };
                let handle = self
                    .executor
                    .register_action_client_raw_sized::<1024, 1024, 1024>(
                        crate::RawActionClientSpec {
                            node_id: Some(node),
                            action_name: entity_name,
                            type_name: metadata.type_name,
                            type_hash: metadata.type_hash,
                            goal_response_callback: None,
                            feedback_callback,
                            result_callback,
                            context: ctx,
                        },
                    )
                    .map_err(decl_err_from_node)?;
                self.cell
                    .view()
                    .push_action_client(id_str(metadata.id.as_str())?, handle.entry_index())
                    .map_err(|_| NodeDeclError::Runtime)?;
                Ok(())
            }
            EntityKind::Parameter => {
                // Phase 212.M-F.23 Wave 2 — declarative parameter dispatch on
                // the single-node runtime. The first declared parameter lazily
                // stands up the 6 ROS 2 parameter services for this executor's
                // node; `spin_once` drives those service servers thereafter
                // (`#[cfg(param-services)]` block at spin.rs). The declared
                // source default seeds the value. With `param-services` off the
                // arm is a no-op (entity declared, no RMW handle) — byte-
                // identical to the pre-Wave-2 behavior.
                #[cfg(feature = "param-services")]
                {
                    if self.executor.params().is_none() {
                        self.executor
                            .register_parameter_services()
                            .map_err(decl_err_from_node)?;
                    }
                    let value = param_default_to_value(metadata.parameter_default.as_ref());
                    self.executor
                        .declare_parameter(metadata.source_name.as_str(), value);
                }
                Ok(())
            }
        }
    }

    fn record_callback_effect(
        &mut self,
        _callback_id: CallbackId<'_>,
        _kind: CallbackEffectKind,
        _entity_id: EntityId<'_>,
    ) -> NodeResult<()> {
        // Planner concern only — the live runtime doesn't need the
        // effect graph at spin time.
        Ok(())
    }
}

/// Lower a source-recorded [`ParameterDefault`] into the executor-facing
/// [`nros_params::ParameterValue`] used to seed a declared parameter. Scalar
/// defaults carry their value directly; the array variants record only the
/// declared type (no element data at the source layer) so they seed as
/// `NotSet` — the parameter is still declared, just without a concrete array
/// default. A `Double` default is stored as a string at the metadata layer and
/// parsed here (unparseable → `0.0`).
#[cfg(feature = "param-services")]
fn param_default_to_value(
    default: Option<&crate::node_metadata::ParameterDefault>,
) -> nros_params::ParameterValue {
    use crate::node_metadata::ParameterDefault;
    use nros_params::ParameterValue;
    match default {
        None => ParameterValue::NotSet,
        Some(ParameterDefault::Bool(b)) => ParameterValue::Bool(*b),
        Some(ParameterDefault::Integer(i)) => ParameterValue::Integer(*i),
        Some(ParameterDefault::Double(s)) => {
            ParameterValue::Double(s.as_str().parse::<f64>().unwrap_or(0.0))
        }
        Some(ParameterDefault::String(s)) => {
            ParameterValue::from_string(s.as_str()).unwrap_or(ParameterValue::NotSet)
        }
        Some(
            ParameterDefault::BoolArray
            | ParameterDefault::IntegerArray
            | ParameterDefault::DoubleArray
            | ParameterDefault::StringArray,
        ) => ParameterValue::NotSet,
    }
}

/// Phase 264 W4b — infer a [`nros_params::ParameterValue`] from a raw launch
/// `<param value=…/>` string. ROS 2 launch `<param>` values are untyped strings; the
/// macro path has no per-param type attribute, so infer: `true`/`false` → `Bool`, an
/// `i64` literal → `Integer`, an `f64` literal → `Double`, otherwise `String`. This
/// mirrors the type a `ros2 param set` of the same literal would land on, so the baked
/// initial and a CLI override agree on type.
#[cfg(feature = "param-services")]
fn infer_param_value(raw: &str) -> nros_params::ParameterValue {
    use nros_params::ParameterValue;
    match raw {
        "true" => return ParameterValue::from_bool(true),
        "false" => return ParameterValue::from_bool(false),
        _ => {}
    }
    if let Ok(i) = raw.parse::<i64>() {
        return ParameterValue::from_integer(i);
    }
    if let Ok(f) = raw.parse::<f64>() {
        return ParameterValue::from_double(f);
    }
    ParameterValue::from_string(raw).unwrap_or(ParameterValue::NotSet)
}

fn dispatch_into_cell(cell: &dyn CellView, cb_id: &str, payload: &[u8]) {
    cell.note_dispatch(!payload.is_empty());
    let resolver = CellResolver { cell };
    let mut ctx = CallbackCtx::new(payload, &resolver);
    // W4c — let the callback read `ctx.parameter::<T>(name)` from the executor's store
    // (threaded onto the cell by `apply_param_services`; `None` until then).
    #[cfg(feature = "param-services")]
    ctx.set_param_server(cell.view_param_server());
    // If the slot is already borrowed (a re-entrant publish from a
    // tick hook on the same cell, etc.) the view drops this dispatch. In
    // practice the borrow succeeds because subscription / timer
    // callbacks run sequentially under the single-threaded executor.
    cell.try_with_slot_mut(&mut |slot| slot.dispatch(cb_id, &mut ctx));
}

/// Phase 250 (Wave 2b) — dispatch a `.safety()` subscription message into the
/// component's `on_callback` with its E2E [`IntegrityStatus`] attached, read via
/// `CallbackCtx::integrity()`. The integrity-aware twin of [`dispatch_into_cell`].
#[cfg(feature = "safety-e2e")]
fn dispatch_into_cell_with_integrity(
    cell: &dyn CellView,
    cb_id: &str,
    payload: &[u8],
    status: &nros_node::IntegrityStatus,
) {
    cell.note_dispatch(!payload.is_empty());
    let resolver = CellResolver { cell };
    let mut ctx = CallbackCtx::new_with_integrity(payload, &resolver, status);
    // W4c — param store for a `.safety()` subscription callback too.
    #[cfg(feature = "param-services")]
    ctx.set_param_server(cell.view_param_server());
    cell.try_with_slot_mut(&mut |slot| slot.dispatch(cb_id, &mut ctx));
}

// =============================================================================
// Phase 212.M-F.23 — service / action SERVER trampolines + tick-side client /
// action dispatch.
//
// The executor's raw service/action-server registration takes C-ABI fn
// pointers, so the runtime leaks a `*Ctx` (lives for the runtime's lifetime,
// like the executor) holding the owning `ComponentCell` + the declared
// callback ids. Each trampoline rebuilds a `CallbackCtx` and routes into the
// component's `on_callback`, exactly as the orchestration codegen's
// `svc_tramp_*` / `goal_tramp_*` do for the Entry path.
// =============================================================================

/// Leaked context for a service-server arena callback.
struct ServiceServerCtx {
    cell: CellHandle,
    callback_id: IdStr,
}

/// Leaked context for an action-server arena callback (goal + cancel + the
/// optional accepted hook all share one).
struct ActionServerCtx {
    cell: CellHandle,
    goal_callback_id: IdStr,
    cancel_callback_id: IdStr,
    accepted_callback_id: Option<IdStr>,
}

/// Leaked context for an action-CLIENT result + feedback callbacks.
struct ActionClientCtx {
    cell: CellHandle,
    result_callback_id: IdStr,
    feedback_callback_id: Option<IdStr>,
}

/// Action-client result callback: the executor's spin auto-drives the goal to
/// completion and hands the terminal result CDR here; route it into the
/// component's `on_callback` (read with `CallbackCtx::message`).
unsafe extern "C" fn action_result_trampoline(
    _goal_id: *const GoalId,
    _status: GoalStatus,
    result_data: *const u8,
    result_len: usize,
    ctx: *mut core::ffi::c_void,
) {
    let actx = unsafe { &*(ctx as *const ActionClientCtx) };
    let result_slice = unsafe { core::slice::from_raw_parts(result_data, result_len) };
    dispatch_into_cell(actx.cell.view(), &actx.result_callback_id, result_slice);
}

/// Action-client feedback callback: route each feedback CDR into the
/// component's `on_callback` under the bound feedback callback id.
unsafe extern "C" fn action_feedback_trampoline(
    _goal_id: *const GoalId,
    feedback_data: *const u8,
    feedback_len: usize,
    ctx: *mut core::ffi::c_void,
) {
    let actx = unsafe { &*(ctx as *const ActionClientCtx) };
    let Some(cb_id) = actx.feedback_callback_id.as_ref() else {
        return;
    };
    let feedback_slice = unsafe { core::slice::from_raw_parts(feedback_data, feedback_len) };
    dispatch_into_cell(actx.cell.view(), cb_id, feedback_slice);
}

/// Service-server request callback: deserialize-side runs in the component's
/// `on_callback` via `CallbackCtx::with_reply`; the executor sends the reply
/// from the bytes written into `resp`.
unsafe extern "C" fn service_server_trampoline(
    req: *const u8,
    req_len: usize,
    resp: *mut u8,
    resp_cap: usize,
    resp_len: *mut usize,
    ctx: *mut core::ffi::c_void,
) -> bool {
    let sctx = unsafe { &*(ctx as *const ServiceServerCtx) };
    let req_slice = unsafe { core::slice::from_raw_parts(req, req_len) };
    let resp_slice = unsafe { core::slice::from_raw_parts_mut(resp, resp_cap) };
    let mut written = 0usize;
    let view = sctx.cell.view();
    let resolver = CellResolver { cell: view };
    let mut cb = CallbackCtx::with_reply(req_slice, &resolver, resp_slice, &mut written);
    // W4c — service-server callback can read `ctx.parameter::<T>(name)` too.
    #[cfg(feature = "param-services")]
    cb.set_param_server(view.view_param_server());
    view.try_with_slot_mut(&mut |slot| slot.dispatch(&sctx.callback_id, &mut cb));
    unsafe { *resp_len = written };
    true
}

/// Action-server goal callback → component `on_callback` with a goal decision.
unsafe extern "C" fn action_goal_trampoline(
    _goal_id: *const GoalId,
    goal_data: *const u8,
    goal_len: usize,
    ctx: *mut core::ffi::c_void,
) -> crate::GoalResponse {
    let actx = unsafe { &*(ctx as *const ActionServerCtx) };
    let goal_slice = unsafe { core::slice::from_raw_parts(goal_data, goal_len) };
    let mut resp = crate::GoalResponse::Reject;
    let view = actx.cell.view();
    let resolver = CellResolver { cell: view };
    let mut cb = CallbackCtx::with_goal_decision(goal_slice, &resolver, &mut resp);
    // W4c — action goal callback can read `ctx.parameter::<T>(name)` too.
    #[cfg(feature = "param-services")]
    cb.set_param_server(view.view_param_server());
    view.try_with_slot_mut(&mut |slot| slot.dispatch(&actx.goal_callback_id, &mut cb));
    resp
}

/// Action-server cancel callback → component `on_callback` with a cancel
/// decision. The cancel callback has no goal payload.
unsafe extern "C" fn action_cancel_trampoline(
    _goal_id: *const GoalId,
    _status: GoalStatus,
    ctx: *mut core::ffi::c_void,
) -> crate::CancelResponse {
    let actx = unsafe { &*(ctx as *const ActionServerCtx) };
    let mut resp = crate::CancelResponse::Reject;
    let view = actx.cell.view();
    let resolver = CellResolver { cell: view };
    let mut cb = CallbackCtx::with_cancel_decision(&[], &resolver, &mut resp);
    // W4c — action cancel callback can read `ctx.parameter::<T>(name)` too.
    #[cfg(feature = "param-services")]
    cb.set_param_server(view.view_param_server());
    view.try_with_slot_mut(&mut |slot| slot.dispatch(&actx.cancel_callback_id, &mut cb));
    resp
}

/// Action-server accepted hook → component `on_callback` (no decision, no
/// payload). No-op when the component didn't declare an accepted callback.
unsafe extern "C" fn action_accepted_trampoline(
    _goal_id: *const GoalId,
    ctx: *mut core::ffi::c_void,
) {
    let actx = unsafe { &*(ctx as *const ActionServerCtx) };
    let Some(cb_id) = actx.accepted_callback_id.as_ref() else {
        return;
    };
    dispatch_into_cell(actx.cell.view(), cb_id, &[]);
}

/// Tick-side service/action CLIENT dispatch — the single-node runtime's mirror
/// of the orchestration `GenClientDispatch`. Resolves the per-component client
/// handle arrays + a `*mut Executor<'static>` (the tick borrows `&components` while
/// needing `&mut executor`, so the executor is reached through a raw pointer,
/// reborrowed `&mut` per call; no aliasing — `executor` and `components` are
/// disjoint fields).
struct RuntimeClientDispatch<'a> {
    executor: *mut Executor<'static>,
    services: &'a [(IdStr, crate::HandleId)],
    actions: &'a [(IdStr, usize)],
}

impl RuntimeClientDispatch<'_> {
    fn service(&self, entity: &str) -> NodeResult<crate::HandleId> {
        self.services
            .iter()
            .find(|(e, _)| e == entity)
            .map(|(_, h)| *h)
            .ok_or(NodeDeclError::Runtime)
    }

    fn action_entry(&self, entity: &str) -> NodeResult<usize> {
        self.actions
            .iter()
            .find(|(e, _)| e == entity)
            .map(|(_, i)| *i)
            .ok_or(NodeDeclError::Runtime)
    }
}

impl ClientDispatch for RuntimeClientDispatch<'_> {
    fn call_raw(
        &mut self,
        service_entity: &str,
        request_cdr: &[u8],
        response_buf: &mut [u8],
    ) -> NodeResult<usize> {
        use crate::ClientTrait;
        let hid = self.service(service_entity)?;
        {
            let executor = unsafe { &mut *self.executor };
            let entry = unsafe { executor.service_client_entry_mut(hid.0) }
                .ok_or(NodeDeclError::Runtime)?;
            entry
                .handle
                .send_request_raw(request_cdr)
                .map_err(|_| NodeDeclError::Runtime)?;
        }
        // Bounded wait — caps total time so the tick loop stays responsive.
        for _ in 0..200 {
            let executor = unsafe { &mut *self.executor };
            executor.spin_once(core::time::Duration::from_millis(10));
            let entry = unsafe { executor.service_client_entry_mut(hid.0) }
                .ok_or(NodeDeclError::Runtime)?;
            match entry.handle.take_response_raw(response_buf) {
                // Issue 0778 — one call in flight on this blocking path, so
                // the sequence id is dropped deliberately.
                Ok(Some((len, _seq))) => return Ok(len),
                Ok(None) => continue,
                Err(_) => return Err(NodeDeclError::Runtime),
            }
        }
        Err(NodeDeclError::Runtime)
    }

    fn send_goal_raw(&mut self, action_entity: &str, goal_cdr: &[u8]) -> NodeResult<GoalId> {
        let entry_index = self.action_entry(action_entity)?;
        let executor = unsafe { &mut *self.executor };
        let core = unsafe { executor.action_client_core_mut(entry_index) }
            .ok_or(NodeDeclError::Runtime)?;
        let goal_id = core.send_goal_raw(goal_cdr).map_err(decl_err_from_node)?;
        // rclcpp-style: request the result immediately. The server queues the
        // get_result request until the goal terminates, then replies — the
        // executor's spin auto-delivers it to the bound result callback (the
        // executor never auto-sends this request, so the declarative client
        // must). Best-effort: a transport hiccup just means no result callback.
        let _ = core.send_get_result_request(&goal_id);
        Ok(goal_id)
    }
}

/// Tick-side action-SERVER execution — mirror of `GenActionExec`. Lets a
/// component complete goals / publish feedback / enumerate active goals from
/// its `tick` via `TickCtx`.
struct RuntimeActions<'a> {
    executor: *mut Executor<'static>,
    handles: &'a [(IdStr, crate::ActionServerRawHandle)],
}

impl RuntimeActions<'_> {
    fn handle(&self, entity: &str) -> NodeResult<crate::ActionServerRawHandle> {
        self.handles
            .iter()
            .find(|(e, _)| e == entity)
            .map(|(_, h)| *h)
            .ok_or(NodeDeclError::Runtime)
    }
}

impl ActionExecutor for RuntimeActions<'_> {
    fn complete_goal_raw(
        &mut self,
        action_entity: &str,
        goal_id: &GoalId,
        status: GoalStatus,
        result: &[u8],
    ) -> NodeResult<()> {
        let handle = self.handle(action_entity)?;
        let executor = unsafe { &mut *self.executor };
        // issue 0796 — a result too large for the server's RESULT_BUF is
        // reported rather than silently dropped.
        handle
            .complete_goal_raw(executor, goal_id, status, result)
            .map_err(|_| NodeDeclError::Runtime)
    }

    fn publish_feedback_raw(
        &mut self,
        action_entity: &str,
        goal_id: &GoalId,
        feedback: &[u8],
    ) -> NodeResult<()> {
        let handle = self.handle(action_entity)?;
        let executor = unsafe { &mut *self.executor };
        handle
            .publish_feedback_raw(executor, goal_id, feedback)
            .map_err(|_| NodeDeclError::Runtime)
    }

    fn for_each_active_goal(
        &self,
        action_entity: &str,
        visit: &mut dyn FnMut(&GoalId, GoalStatus),
    ) {
        if let Ok(handle) = self.handle(action_entity) {
            let executor = unsafe { &*self.executor };
            handle.for_each_active_goal(executor, |g| visit(&g.goal_id, g.status));
        }
    }
}

// Phase 258 (Track 2, w5) — the typed BSP fn-ptr aliases (`NodeRegisterFn` /
// `NodeInitFn` / `NodeDispatchFn` / `NodeTickFn`) are gone with the retired
// `register_dispatch_slot` / `nros_run_components` BSP-baker path. The
// macro-emitted `register(runtime)` wrapper now installs via the
// `install_node_typed` seam (Track 2 w4).

/// Phase 257 (W0-B) — register an [`ExecutableNode`] `C` against a **borrowed**
/// executor (the shared cffi `Executor` a foreign-language typed entry hands in via
/// its `nros::global_handle()` / `Node::executor_handle()`), returning the live
/// [`ComponentCell`].
///
/// Unlike [`ExecutorNodeRuntime::register_node`] this owns neither the executor nor a
/// components list: the executor's per-entity callbacks hold `CellHandle`s
/// clones (see [`ExecutorSink`]), so the cell stays alive for the executor's lifetime
/// via dispatch alone — the caller may drop the returned cell (pub/sub/timer nodes, the
/// W0-B target) or stash it to drive `tick` (service-client/action nodes; phase-257 D2).
/// The node self-creates its node (its `Node::NAME`) on the shared executor (phase-257
/// D7 Option C — Rust nodes in a foreign entry self-name, no entry-side qos-override).
/// Issue 0095 — preserve executor callback-table exhaustion through the
/// `NodeError → NodeDeclError` collapse so the register seam (and ultimately
/// the `nros::main!` entry) can name `NROS_EXECUTOR_MAX_CBS` instead of an
/// opaque `NodeRegister`. Every other `NodeError` stays `Runtime`.
fn decl_err_from_node(e: nros_node::NodeError) -> NodeDeclError {
    // Issue 0428 — `NodeDeclError::Runtime` collapses every non-`ExecutorFull`
    // cause into one opaque `NodeRegister("<pkg>")`, "four collapses away from the
    // cause" (node.rs). That is how a Cyclone descriptor/transport failure was
    // mis-diagnosed twice. Surface the real variant on the error path (rare — this
    // only runs when a declaration is already failing), so the register seam names
    // WHAT failed, not just WHERE.
    // issue 0589 — `nros_log`, not `std::eprintln!`: std stdio is fatal on Zephyr
    // native_sim, and this crate cannot gate on the platform. Routing it here
    // also means a `no_std` image gets the diagnostic, which the old
    // `cfg(feature = "std")` arm never delivered.
    if !matches!(e, nros_node::NodeError::ExecutorFull) {
        nros_log::nros_error!(
            nros_log::get_logger("nros"),
            "node declaration failed — NodeError::{e:?}"
        );
    }
    // no_std targets get the same diagnostic through the `log` facade (every
    // RTOS board bridges it); without this the collapse left only an opaque
    // `NodeRegister("<pkg>")` on embedded — e.g. a static subscriber-pool
    // exhaustion (NROS_RMW_SUBSCRIBER_SLOTS) surfaced with no cause at all.
    #[cfg(not(feature = "std"))]
    if !matches!(e, nros_node::NodeError::ExecutorFull) {
        log::error!("nros: node declaration failed — NodeError::{e:?}");
    }
    match e {
        nros_node::NodeError::ExecutorFull => NodeDeclError::ExecutorFull,
        _ => NodeDeclError::Runtime,
    }
}

fn register_node_borrowed<
    'p,
    C: ExecutableNode + 'static,
    const PUBS: usize,
    const SVCS: usize,
    const ACTC: usize,
    const ACTS: usize,
    const SSRV: usize,
>(
    executor: &mut Executor<'static>,
    params: &'p [(&'p str, &'p str)],
    node_identity: Option<(&'static str, &'static str)>,
    remaps: &'p [(&'p str, &'p str)],
    qos_overrides: &'static [nros_node::executor::node_record::QoSOverrideCode],
    slot_mu: &'static mut MaybeUninit<TypedSlot<C>>,
    cell_mu: &'static mut MaybeUninit<ComponentCell<PUBS, SVCS, ACTC, ACTS, SSRV>>,
) -> NodeResult<&'static ComponentCell<PUBS, SVCS, ACTC, ACTS, SSRV>>
where
    C::State: 'static,
{
    // W5-endgame step 2b (issue 0857) — the cell is PLACED into caller
    // storage (the per-class static's paired region, or the alloc
    // convenience's leaked box), never `Arc`'d: 0857 measured the Arc'd cell
    // at ~17.5 KiB of heap per component. Closures and leaked ctxs hold a
    // `CellHandle::Static` copy; the executor's drop trampoline runs the
    // cell's destructor in place.
    let cell: &'static ComponentCell<PUBS, SVCS, ACTC, ACTS, SSRV> = cell_mu.write(ComponentCell {
        slot: RefCell::new(place_slot::<C>(slot_mu)),
        publishers: RefCell::new(heapless::Vec::new()),
        service_clients: RefCell::new(heapless::Vec::new()),
        action_clients: RefCell::new(heapless::Vec::new()),
        action_servers: RefCell::new(heapless::Vec::new()),
        svc_ctxs: [const { UnsafeCell::new(MaybeUninit::uninit()) }; _],
        svc_ctxs_used: core::cell::Cell::new(0),
        act_srv_ctxs: [const { UnsafeCell::new(MaybeUninit::uninit()) }; _],
        act_srv_ctxs_used: core::cell::Cell::new(0),
        act_cli_ctxs: [const { UnsafeCell::new(MaybeUninit::uninit()) }; _],
        act_cli_ctxs_used: core::cell::Cell::new(0),
        header: CellHeader {
            callback_dispatches: AtomicUsize::new(0),
            message_dispatches: AtomicUsize::new(0),
        },
        // W4c — set by `apply_param_services` once the store exists (it runs after this).
        #[cfg(feature = "param-services")]
        param_server: core::cell::Cell::new(core::ptr::null()),
    });
    let mut sink = ExecutorSink {
        // Reborrow so `executor` stays usable for `enroll_component` after the
        // sink (which holds `&mut Executor<'static>`) is dropped below.
        executor: &mut *executor,
        cell: CellHandle::Static(cell),
        nodes: heapless::Vec::new(),
        // Phase 268 W1 — thread the per-component identity bake (RFC-0046).
        node_identity,
        // Phase 305 W3 (issue 0255) — thread the per-component remap bake.
        remaps,
        // Issue #52 — thread the per-component QoS-override bake.
        qos_overrides,
    };
    let sink_dyn: &mut dyn NodeRuntime = &mut sink;
    let mut context = NodeContext::new(C::NAME, sink_dyn);
    // Phase 264 W4a — seed the baked launch-param initials so `register()` can read
    // them via `NodeContext::param`.
    context.set_params(params);
    C::register(&mut context)?;

    // Phase 258 (Track 2, 2a) — enroll the cell in the executor's component
    // tick registry so `install`'d nodes tick (closes phase-257 D2: poll-only
    // service-client/action nodes have no callbacks keeping the cell alive AND
    // never ran `tick`). The executor runs `tick` each `spin_once` and runs
    // the cell's destructor in place on `Executor::drop`. Harmless for
    // pub/sub/timer-only nodes — their tick body is a no-op. Registry-full
    // (`MAX_NODES`) proceeds without a tick slot; the placed cell then simply
    // lives (and leaks its component state's destructor) with its storage,
    // which a monotonic `take` never re-hands out.
    let raw = cell as *const ComponentCell<PUBS, SVCS, ACTC, ACTS, SSRV> as *mut core::ffi::c_void;
    // SAFETY: `raw` is the freshly placed cell; the enrolled trampoline
    // monomorphizations match its exact type (borrow on tick, drop_in_place
    // on executor drop).
    let _ = unsafe {
        executor.enroll_component(
            raw,
            component_tick_trampoline::<PUBS, SVCS, ACTC, ACTS, SSRV>,
            component_drop_trampoline::<PUBS, SVCS, ACTC, ACTS, SSRV>,
        )
    };

    // W4c — capture the executor's volatile param store on the cell so this node's
    // callbacks can read `ctx.parameter::<T>(name)`. Non-null only when `nros::main!`
    // emitted `apply_param_services` BEFORE this register call (`[param_services]`
    // declared); otherwise the store is absent and the field stays null.
    #[cfg(feature = "param-services")]
    if let Some(server) = executor.params() {
        cell.param_server.set(server as *const _);
    }

    Ok(cell)
}

/// Phase 257 (W0-B) — C-ABI typed component install. Recovers the shared `Executor`
/// from the foreign typed entry's handle (`global_handle()` / `Node::executor_handle()`
/// = the `_opaque` `*mut Executor<'static>`; cf. nros-c `get_executor_from_ptr`) and registers
/// `C` on it via `register_node_borrowed` (private helper). The component's `ComponentCell`
/// is PLACED in caller/leaked storage (W5-endgame step 2b) and dropped in place by the
/// executor on `Executor::drop`. Returns `0` on success, `-1` on a null handle or a
/// registration error.
///
/// This backs the `__nros_component_<pkg>_install(node, executor, self)` symbol
/// `nros::node!()` emits — the uniform cross-language install seam (phase-257 D6).
///
/// # Safety
/// `executor` must be the live `*mut Executor<'static>` handle a typed entry passes (its
/// `nros::global_handle()` / a node's `executor_handle()`), valid for the call.
#[cfg(feature = "alloc")]
pub unsafe fn install_node_typed<C: ExecutableNode + 'static>(
    executor: *mut core::ffi::c_void,
) -> i32
where
    C::State: 'static,
{
    // SAFETY: forwarded per this fn's contract; no baked params.
    unsafe { install_node_typed_with_params::<C>(executor, &[]) }
}

/// W4a — same as [`install_node_typed`] but seeds the node's [`NodeContext`] with the
/// launch-baked `<param>` initial values, so a `register`/`init`-time `ctx.param(name)`
/// observes the compile-time launch value (RFC-0004 §10). `params` must outlive the call.
///
/// # Safety
/// `executor` must be the live `*mut Executor<'static>` handle a typed entry passes, valid for the call.
#[cfg(feature = "alloc")]
pub unsafe fn install_node_typed_with_params<C: ExecutableNode + 'static>(
    executor: *mut core::ffi::c_void,
    params: &[(&str, &str)],
) -> i32
where
    C::State: 'static,
{
    // SAFETY: forwarded per this fn's contract; no identity injection.
    unsafe { install_node_typed_with_node_identity::<C>(executor, params, None) }
}

/// Phase 268 W1 — same as [`install_node_typed_with_params`] but also injects the
/// launch `<node name= namespace=>` identity so `ExecutorSink::create_node` uses it
/// instead of the `NodeOptions` default (RFC-0046). `None` → backward-compatible
/// (NodeOptions stands). `nros::node!()` calls this variant to carry the identity from
/// `RuntimeCtx::node_identity` set by `nros::main!` per component. `params` and
/// `node_identity` strings must outlive the call (both are `'static` in the macro emit).
///
/// # Safety
/// `executor` must be the live `*mut Executor<'static>` handle a typed entry passes, valid for the call.
#[cfg(feature = "alloc")]
pub unsafe fn install_node_typed_with_node_identity<C: ExecutableNode + 'static>(
    executor: *mut core::ffi::c_void,
    params: &[(&str, &str)],
    node_identity: Option<(&'static str, &'static str)>,
) -> i32
where
    C::State: 'static,
{
    // SAFETY: forwarded per this fn's contract; no remap rules.
    unsafe { install_node_typed_with_launch::<C>(executor, params, node_identity, &[], &[]) }
}

/// Phase 305 W3 (issue 0255) — same as [`install_node_typed_with_node_identity`]
/// but also carries the launch `<remap from= to=/>` rules `nros::main!` baked for
/// this component (`RuntimeCtx::remaps`). `ExecutorSink::create_entity` applies
/// them (plus `~`/relative expansion) before any entity name reaches the wire.
/// All slices must outlive the call (`'static` promoted in the macro emit).
///
/// # Safety
/// `executor` must be the live `*mut Executor<'static>` handle a typed entry passes, valid for the call.
#[cfg(feature = "alloc")]
pub unsafe fn install_node_typed_with_launch<C: ExecutableNode + 'static>(
    executor: *mut core::ffi::c_void,
    params: &[(&str, &str)],
    node_identity: Option<(&'static str, &'static str)>,
    remaps: &[(&str, &str)],
    qos_overrides: &'static [nros_node::executor::node_record::QoSOverrideCode],
) -> i32
where
    C::State: 'static,
{
    if executor.is_null() {
        return -1;
    }
    // SAFETY: per the fn contract, `executor` is the live `*mut Executor<'static>` handle.
    let exec: &mut Executor<'static> = unsafe { &mut *(executor as *mut Executor<'static>) };
    // phase-391 W5.3b — the alloc convenience: leak exactly-sized slot storage
    // per call, the same relationship `from_executor` has to `new_in`. The
    // macro-emitted entries call the `_in` twin with their per-class static
    // instead, so GENERATED images do not take this branch.
    let slot_mu: &'static mut MaybeUninit<TypedSlot<C>> =
        Box::leak(Box::new(MaybeUninit::uninit()));
    // W5-endgame step 2b — the cell leaks the same way; its destructor still
    // runs (executor drop trampoline, in place), only the bytes stay.
    let cell_mu: &'static mut MaybeUninit<ComponentCell> =
        Box::leak(Box::new(MaybeUninit::uninit()));
    match register_node_borrowed::<C, _, _, _, _, _>(
        exec,
        params,
        node_identity,
        remaps,
        qos_overrides,
        slot_mu,
        cell_mu,
    ) {
        Ok(_cell) => 0,
        // Issue 0095 — distinct code for executor-table exhaustion so the macro
        // register seam can name `NROS_EXECUTOR_MAX_CBS` instead of an opaque
        // `NodeRegister`. Every other failure stays the generic `-1`.
        Err(NodeDeclError::ExecutorFull) => -2,
        Err(_) => -1,
    }
}

/// phase-391 W5.3b — [`install_node_typed_with_launch`] over the CALLER's
/// per-class slot storage: the heap-free FFI install path. The `nros::node!`
/// macro emits one `static ComponentSlotStorage<C>` per expansion and calls
/// this twin; the C ABI trampoline above it keeps its `(ptr, ptr, ptr) -> i32`
/// shape, so nothing foreign changes.
///
/// Returns `-3` when the class's instance cap is exhausted — raise
/// `NROS_RUNTIME_MAX_CLASS_INSTANCES`. (Distinct from `-2`, the executor
/// callback-table Full.)
///
/// # Safety
/// `executor` must be the live `*mut Executor<'static>` handle a typed entry
/// passes, valid for the call.
pub unsafe fn install_node_typed_with_launch_in<
    C: ExecutableNode + 'static,
    const N: usize,
    const PUBS: usize,
    const SVCS: usize,
    const ACTC: usize,
    const ACTS: usize,
    const SSRV: usize,
>(
    executor: *mut core::ffi::c_void,
    store: &'static ComponentSlotStorage<C, N, PUBS, SVCS, ACTC, ACTS, SSRV>,
    params: &[(&str, &str)],
    node_identity: Option<(&'static str, &'static str)>,
    remaps: &[(&str, &str)],
    qos_overrides: &'static [nros_node::executor::node_record::QoSOverrideCode],
) -> i32
where
    C::State: 'static,
{
    if executor.is_null() {
        return -1;
    }
    let Some((slot_mu, cell_mu)) = store.take() else {
        return -3;
    };
    // SAFETY: per the fn contract, `executor` is the live handle.
    let exec: &mut Executor<'static> = unsafe { &mut *(executor as *mut Executor<'static>) };
    match register_node_borrowed::<C, _, _, _, _, _>(
        exec,
        params,
        node_identity,
        remaps,
        qos_overrides,
        slot_mu,
        cell_mu,
    ) {
        Ok(_cell) => 0,
        Err(NodeDeclError::ExecutorFull) => -2,
        Err(_) => -1,
    }
}

/// phase-391 W5.3b — [`install_node_typed`] over caller storage; see
/// [`install_node_typed_with_launch_in`].
///
/// # Safety
/// As [`install_node_typed_with_launch_in`].
pub unsafe fn install_node_typed_in<
    C: ExecutableNode + 'static,
    const N: usize,
    const PUBS: usize,
    const SVCS: usize,
    const ACTC: usize,
    const ACTS: usize,
    const SSRV: usize,
>(
    executor: *mut core::ffi::c_void,
    store: &'static ComponentSlotStorage<C, N, PUBS, SVCS, ACTC, ACTS, SSRV>,
) -> i32
where
    C::State: 'static,
{
    // SAFETY: forwarded per this fn's contract.
    unsafe { install_node_typed_with_launch_in(executor, store, &[], None, &[], &[]) }
}

// Phase 258 (Track 2, w5) — `nros_run_components` (the BSP shim that registered
// every component via the four-fn-ptr `register_dispatch_slot` then spun) is
// gone. It had no callers; owned-spin / BSP entries register through the
// `install_node_typed` seam + drive `ExecutorNodeRuntime::spin`.

// =============================================================================
// Tests
// =============================================================================
//
// Concrete `Executor` construction needs a real RMW backend session
// (with `rmw-cffi` on, `Executor::from_session` takes the cffi
// session). MockSession only exists when `rmw-cffi` is off — so the
// unit tests that exercise live timer firing live in
// `packages/testing/nros-tests/tests/phase212_m5a2_component_runtime.rs`
// gated behind the `component-runtime-test` feature (pulls
// `nros-rmw-zenoh`). The compile-only smoke here verifies the public
// types are reachable through the umbrella surface.

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node::Node;

    #[test]
    fn handle_slot_is_observable() {
        // Trivial smoke — the handle type carries the slot index.
        let h = RegisteredNode::<DummyComp> {
            component_idx: 7,
            _phantom: PhantomData,
        };
        assert_eq!(h.slot(), 7);
    }

    struct DummyComp;
    impl Node for DummyComp {
        const NAME: &'static str = "dummy";
        fn register(_ctx: &mut NodeContext<'_>) -> NodeResult<()> {
            Ok(())
        }
    }
    impl ExecutableNode for DummyComp {
        type State = ();
        fn init() -> Self::State {}
        fn on_callback(_s: &mut (), _cb: Callback<'_>, _ctx: &mut CallbackCtx<'_>) {}
    }
}

/// issue 0460 — map a `NodeError` to a static reason a capability failure can
/// carry across the `nros-platform` trait boundary (which cannot name
/// `NodeError`). Static strings, so this works on `no_std` targets with no
/// allocator and no `log` dependency in this crate.
///
/// Both matches are EXHAUSTIVE on purpose. A wildcard arm is how this issue got
/// its diagnostic — `NodeRegister("lifecycle")` named the capability and
/// nothing about the cause — and a `_ =>` here would reintroduce exactly that
/// for whichever variant is added next. The compile error is the point.
#[cfg(any(feature = "lifecycle-services", feature = "param-services"))]
fn capability_reason(e: &nros_node::NodeError) -> &'static str {
    use nros_node::NodeError;
    match e {
        NodeError::Transport(t) => capability_transport_reason(t),
        NodeError::NameTooLong => {
            "NameTooLong (the node FQN or a service name overflowed its fixed buffer)"
        }
        NodeError::ExecutorFull => {
            "ExecutorFull (no callback slot left — raise CONFIG_NROS_EXECUTOR_MAX_CBS; \
             a capability's services are not counted by the model)"
        }
        NodeError::NodeTableFull => "NodeTableFull (NROS_EXECUTOR_MAX_NODES reached)",
        NodeError::NotInitialized => "NotInitialized (a required subsystem was never registered)",
        NodeError::BackendMismatch => "BackendMismatch (the RMW is not the executor's session)",
        NodeError::BufferTooSmall => "BufferTooSmall",
        NodeError::Serialization => "Serialization",
        NodeError::Deserialization => "Deserialization",
        NodeError::ActionCreationFailed => "ActionCreationFailed",
        NodeError::ServiceRequestFailed => "ServiceRequestFailed",
        NodeError::ServiceReplyFailed => "ServiceReplyFailed",
        NodeError::Timeout => "Timeout",
        NodeError::RequestInFlight => "RequestInFlight",
        NodeError::NoSchedContextSlot => "NoSchedContextSlot",
        NodeError::InvalidSchedContextBinding => "InvalidSchedContextBinding",
        // issue 0790 added this variant and did not add the arm; the exhaustive
        // match above did exactly what its doc comment says it is for, and the
        // lanes that were run could not see it (`--all-targets` enables `test`,
        // which pulls in a different cfg path, and `cargo doc` does no
        // exhaustiveness checking, so the parity lane stayed green over it).
        NodeError::ShutdownCallbacksFull => {
            "ShutdownCallbacksFull (the executor's pre/on-shutdown callback table is \
             full — raise NROS_EXECUTOR_MAX_SHUTDOWN_CBS)"
        }
    }
}

/// The `NodeError::Transport` payload, which is the half that actually names
/// what the RMW refused. `Backend(&'static str)` passes its own diagnostic
/// straight through — it is already the string the backend wanted to say.
///
/// Unlike `capability_reason` above this one keeps a wildcard, and only because
/// it must: `TransportError::BackendDynamic` is gated on `nros-rmw/alloc`,
/// which feature unification can switch on from another crate in the graph
/// without `nros/alloc` — so no spelling of the arm is correct in both builds
/// (`nros-c`'s `nros_ret_t` mapping carries the same wildcard for the same
/// reason). Every other variant is named, so the wildcard's text is what an
/// unmapped one looks like.
#[cfg(any(feature = "lifecycle-services", feature = "param-services"))]
fn capability_transport_reason(t: &nros_rmw::TransportError) -> &'static str {
    use nros_rmw::TransportError as T;
    match t {
        T::Backend(s) => s,
        T::ServiceServerCreationFailed => {
            "Transport::ServiceServerCreationFailed (the RMW refused to declare the queryable)"
        }
        T::ServiceClientCreationFailed => "Transport::ServiceClientCreationFailed",
        T::PublisherCreationFailed => "Transport::PublisherCreationFailed",
        T::SubscriberCreationFailed => "Transport::SubscriberCreationFailed",
        T::ConnectionFailed => "Transport::ConnectionFailed (no session to the router)",
        T::Disconnected => "Transport::Disconnected",
        T::TopicNameInvalid => "Transport::TopicNameInvalid",
        T::NodeNameNonExistent => "Transport::NodeNameNonExistent",
        T::IncompatibleQos => "Transport::IncompatibleQos",
        T::IncompatibleAbi => "Transport::IncompatibleAbi",
        T::InvalidConfig => "Transport::InvalidConfig",
        T::InvalidArgument => "Transport::InvalidArgument",
        T::Unsupported => "Transport::Unsupported (the backend does not implement this)",
        T::BadAlloc => "Transport::BadAlloc",
        T::PublishFailed => "Transport::PublishFailed",
        T::ServiceRequestFailed => "Transport::ServiceRequestFailed",
        T::ServiceReplyFailed => "Transport::ServiceReplyFailed",
        T::SerializationError => "Transport::SerializationError",
        T::DeserializationError => "Transport::DeserializationError",
        T::BufferTooSmall => "Transport::BufferTooSmall",
        T::MessageTooLarge => "Transport::MessageTooLarge",
        T::Timeout => "Transport::Timeout",
        T::WouldBlock => "Transport::WouldBlock",
        T::TooLarge => "Transport::TooLarge",
        T::TaskStartFailed => "Transport::TaskStartFailed",
        T::PollFailed => "Transport::PollFailed",
        T::KeepaliveFailed => "Transport::KeepaliveFailed",
        T::JoinFailed => "Transport::JoinFailed",
        T::LoanNotSupported => "Transport::LoanNotSupported",
        T::NoData => "Transport::NoData",
        _ => "Transport::<variant with no reason mapping — add it here>",
    }
}
