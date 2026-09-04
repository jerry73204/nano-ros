//! Node API for nros C API.
//!
//! A node is the main entity in ROS 2 that can have publishers, subscribers,
//! services, and other communication primitives.

use core::{ffi::c_char, ptr};

use crate::{
    constants::{MAX_LOCATOR_LEN, MAX_NAME_LEN, MAX_NAMESPACE_LEN, MAX_RMW_NAME_LEN},
    error::*,
    support::{nros_support_state_t, nros_support_t},
};

/// issue 0972 — decode the C ABI `domain_id` byte, ONCE, for every caller.
///
/// `nros_support_t.domain_id` stores the byte exactly as the C caller passed
/// it, and that byte is not a domain: it is an encoding.
/// `nros_support_init` already decodes it correctly for the session
/// (`support.rs`, via `baked_domain_from_c_abi`), but the two entity-resolution
/// sites below read the RAW field and compared it against 0, which got both
/// ends wrong:
///
/// * `NROS_DOMAIN_ID_EXPLICIT_ZERO` (255, issue #227's "I really do mean domain
///   0") is `!= 0`, so it was used AS the domain — 255, which is not even a
///   legal ROS domain (they cap at 232). A caller asking for the default domain
///   got an out-of-range one.
/// * plain `0` means UNSET, and falling back to the session is right — that
///   part was correct and is preserved.
///
/// Keeping the decode in one place is the point: this was the same mistake at
/// two call sites, and a third would have made it three.
fn resolve_domain_from_c_abi(raw: u8, session_domain: u32) -> Option<u32> {
    match nros_node::baked_domain_from_c_abi(raw) {
        None => Some(session_domain),
        // The ROS domain space caps at 232 (`DOMAIN_ID_MAX`). `baked_domain_
        // from_c_abi` deliberately does NOT range-check — its docstring defers
        // that to `ExecutorConfig::try_resolve`, which is right for the boot
        // path but not for here: this resolver feeds entity keyexprs directly
        // and never goes through `try_resolve`, so 233..=254 would be ASSIGNED
        // as a domain. An out-of-spec domain is not a smaller problem than a
        // wrong one — it is unroutable, and silent, because a domain is just
        // the first element of the discovery key (issue 0801).
        Some(d) if d > nros_node::DOMAIN_ID_MAX => {
            nros_log::nros_error!(
                nros_log::get_logger("nros-c"),
                "domain id {} exceeds the ROS maximum of {}; refusing to create \
                 the entity rather than keying it on an unroutable domain \
                 (issue 0972)",
                d,
                nros_node::DOMAIN_ID_MAX
            );
            None
        }
        Some(explicit) => Some(explicit),
    }
}

/// Sentinel value for `domain_id_override`. When set, the support context's
/// domain_id is used instead of the per-Node override.
pub const NROS_DOMAIN_ID_INHERIT: u32 = u32::MAX;

/// Node state
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_node_state_t {
    /// Not initialized
    NROS_NODE_STATE_UNINITIALIZED = 0,
    /// Initialized and ready
    NROS_NODE_STATE_INITIALIZED = 1,
    /// Shutdown
    NROS_NODE_STATE_SHUTDOWN = 2,
}

/// phase-379 W4 — a checkable reference to the node an entity was created on.
///
/// Replaces the `*const nros_node_t` each entity used to store. That pointer
/// was never dereferenced anywhere in the C API — it was assigned, nulled, and
/// asserted on — so it bought provenance nobody could use while leaving a raw
/// pointer that invites a dereference the platform cannot support:
///
/// * bare metal is safe (no MMU, no relocation, handles in `.bss`), but
/// * an RTOS node declared on a task stack dies with `vTaskDelete`, and
/// * `nros_node_t a = b;` is legal and silent — C has no move semantics, and we
///   have no allocator to hide an indirection behind.
///
/// rcl's alternative (`rcl_publisher_fini(&pub, &node)`) does not remove that
/// assumption; it relocates it to a caller who cannot check it either.
///
/// So the entity stores an IDENTITY instead: the executor slot the node is
/// bound to, plus the generation that slot carried when the entity was created.
/// `nros_node_fini` bumps the generation, which makes "finalise an entity after
/// its node" a return code rather than a silent success.
///
/// `generation == 0` is reserved and never issued, so a zeroed struct — the
/// common C mistake — can never resolve.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct nros_node_ref_t {
    /// Executor node slot (`nros_node_t::node_id`). 0 = the primary node.
    pub node_id: u8,
    /// Slot generation when the entity was created. 0 = no node bound.
    pub generation: u32,
}

impl nros_node_ref_t {
    /// The reference an uninitialised entity carries: bound to nothing.
    pub const fn none() -> Self {
        Self {
            node_id: 0,
            generation: 0,
        }
    }

    /// Is this reference bound to a node at all?
    pub const fn is_bound(&self) -> bool {
        self.generation != 0
    }
}

/// Per-slot generation counters, bumped when a node is finalised.
///
/// A plain static rather than executor state because the C `nros_node_t` is
/// CALLER-allocated: there is no runtime-owned place to hang it that every
/// entity can reach without the pointer this type exists to remove.
static NODE_GENERATIONS: [core::sync::atomic::AtomicU32; nros_node::config::MAX_NODES] =
    [const { core::sync::atomic::AtomicU32::new(1) }; nros_node::config::MAX_NODES];

/// Mint a reference to `node`'s current binding.
///
/// # Safety
/// `node` must point at an initialised `nros_node_t` for the duration of the
/// call. The reference outlives the pointer deliberately — that is the point.
pub(crate) unsafe fn node_ref_of(node: *const nros_node_t) -> nros_node_ref_t {
    if node.is_null() {
        return nros_node_ref_t::none();
    }
    let id = unsafe { (*node).node_id };
    nros_node_ref_t {
        node_id: id,
        generation: current_generation(id),
    }
}

/// The generation slot `id` carries now.
pub(crate) fn current_generation(id: u8) -> u32 {
    NODE_GENERATIONS
        .get(id as usize)
        .map(|g| g.load(core::sync::atomic::Ordering::Acquire))
        .unwrap_or(0)
}

/// Retire slot `id`, so every reference minted against it stops resolving.
///
/// Wraps past `u32::MAX` back to 1, never 0 — 0 stays reserved for "unbound".
pub(crate) fn retire_generation(id: u8) {
    if let Some(g) = NODE_GENERATIONS.get(id as usize) {
        let next = g
            .load(core::sync::atomic::Ordering::Acquire)
            .wrapping_add(1);
        g.store(
            if next == 0 { 1 } else { next },
            core::sync::atomic::Ordering::Release,
        );
    }
}

/// Does `r` still name a live binding?
pub(crate) fn node_ref_is_live(r: nros_node_ref_t) -> bool {
    r.is_bound() && current_generation(r.node_id) == r.generation
}

/// Node structure.
///
/// Represents a ROS 2 node with a name and namespace.
#[repr(C)]
pub struct nros_node_t {
    /// Current state
    pub state: nros_node_state_t,
    /// Node name storage
    pub name: [u8; MAX_NAME_LEN],
    /// Node name length
    pub name_len: usize,
    /// Namespace storage
    pub namespace: [u8; MAX_NAMESPACE_LEN],
    /// Namespace length
    pub namespace_len: usize,
    /// Pointer to parent support context
    pub support: *const nros_support_t,

    // Phase 104.C.8 — multi-RMW + per-Node SchedContext fields. Populated
    // by `nros_node_init_ex` from `nros_node_options_t`. Zero values mean
    // "inherit from the support context / executor default" so the legacy
    // `nros_node_init(node, support, name, ns)` entry point keeps its old
    // single-Node behaviour through `nros_node_init_ex` with default
    // options.
    /// RMW backend name (UTF-8, NUL-terminated within `rmw_name_len`).
    /// Empty (`rmw_name_len == 0`) selects the first-registered backend.
    pub rmw_name: [u8; MAX_RMW_NAME_LEN],
    /// Length of `rmw_name` in bytes (excluding NUL). 0 = inherit.
    pub rmw_name_len: usize,
    /// Per-Node domain ID. `NROS_DOMAIN_ID_INHERIT` (== u32::MAX) means
    /// "use the support context's domain_id".
    pub domain_id_override: u32,
    /// SchedContext slot to inherit on every handle created by this Node
    /// (Phase 104.C.4). 0 = inherit the executor's default Fifo context.
    pub sched_context_id: u8,
    /// Reserved for future use (alignment + ABI stability).
    pub _reserved: [u8; 3],
    /// Opaque NodeId slot returned by `Executor::node_builder(...).build()`
    /// when this Node is bound to an Executor. 0 = primary Node (legacy
    /// single-Node path). Internal use only — readers should treat as
    /// opaque.
    pub node_id: u8,
    /// Phase 156 / 104.C.8.b — executor pointer for the multi-Session
    /// dispatch path. `nros_executor_node_init` populates this when
    /// the Node is bound; per-entity `nros_*_init` paths
    /// (`nros_publisher_init`, `nros_subscription_init`, etc.) branch
    /// on `node_id != 0 && !executor.is_null()` to route through
    /// `Executor::node_session_mut(NodeId)` instead of the legacy
    /// support-based dispatch. NULL = legacy single-Node path
    /// (`nros_node_init` / `nros_node_init_ex`).
    pub executor: *const crate::executor::nros_executor_t,

    // Phase 211.H (issue #52) — per-topic QoS overrides the deploy plan
    // lowered from `qos_overrides.<topic>.<role>.<policy>` launch params.
    // Set by `nros_node_set_qos_overrides`; folded into each entity's QoS at
    // `create_publisher` / `create_subscription` time. Appended at the END of
    // the struct so existing field offsets (hence the C ABI) are unchanged;
    // `null` / `0` means "no overrides" (the legacy behaviour).
    /// Pointer to a `&'static`-lifetime array of [`nros_qos_override_t`], or
    /// null. The caller (a generated entry / a hand-written app) owns the
    /// storage for the node's lifetime.
    pub qos_overrides: *const crate::qos::nros_qos_override_t,
    /// Number of entries in `qos_overrides`. 0 = none.
    pub qos_overrides_len: usize,
}

impl Default for nros_node_t {
    fn default() -> Self {
        Self {
            state: nros_node_state_t::NROS_NODE_STATE_UNINITIALIZED,
            name: [0u8; MAX_NAME_LEN],
            name_len: 0,
            namespace: [0u8; MAX_NAMESPACE_LEN],
            namespace_len: 0,
            support: ptr::null(),
            rmw_name: [0u8; MAX_RMW_NAME_LEN],
            rmw_name_len: 0,
            domain_id_override: NROS_DOMAIN_ID_INHERIT,
            sched_context_id: 0,
            _reserved: [0u8; 3],
            node_id: 0,
            executor: ptr::null(),
            qos_overrides: ptr::null(),
            qos_overrides_len: 0,
        }
    }
}

/// Install the per-topic QoS override table the deploy plan lowered from
/// `qos_overrides.<topic>.<role>.<policy>` launch params (issue #52). Every
/// entity created on `node` afterwards folds the matching `(topic, role)`
/// entries into its QoS before the backend-compat check — the C/C++ mirror of
/// Rust's `NodeHandle::set_qos_overrides`. Call once, after `nros_node_init*`
/// and before creating publishers/subscriptions (a generated entry does this
/// before `configure(node)`).
///
/// `overrides` must outlive the node (a `static` array in the generated entry).
/// Pass `len == 0` (or a null `overrides`) to clear.
///
/// # Safety
/// * `node` must point to an initialised `nros_node_t`.
/// * `overrides` must be null or point to `len` valid `nros_qos_override_t`
///   (each `topic` a valid NUL-terminated UTF-8 C string), living at least as
///   long as the node.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_node_set_qos_overrides(
    node: *mut nros_node_t,
    overrides: *const crate::qos::nros_qos_override_t,
    len: usize,
) -> nros_ret_t {
    let Some(node) = (unsafe { node.as_mut() }) else {
        return NROS_RET_INVALID_ARGUMENT;
    };
    node.qos_overrides = overrides;
    node.qos_overrides_len = len;
    NROS_RET_OK
}

/// Phase 104.C.8 — extended node-creation options.
///
/// Mirrors the Rust `Executor::node_builder(name).rmw(rmw_name).
/// locator(...).domain_id(...).namespace(...).sched(...)` chain. Pass an
/// instance to [`nros_node_init_ex`] to bind a Node to a specific RMW
/// backend, locator, domain, and default SchedContext. Zero fields keep
/// the legacy single-Node single-backend behaviour for back-compat
/// callers.
///
/// The struct contains plain inline buffers — no pointer fields — so it
/// is safe to stack-allocate, memcpy, and pass across the FFI.
#[repr(C)]
pub struct nros_node_options_t {
    /// Namespace storage (UTF-8, NUL-terminated within `namespace_len`).
    pub namespace: [u8; MAX_NAMESPACE_LEN],
    /// Length of `namespace` in bytes (excluding NUL).
    pub namespace_len: usize,
    /// RMW backend name (e.g. "zenoh", "cyclonedds"). Empty selects first-
    /// registered (single-backend convenience).
    pub rmw_name: [u8; MAX_RMW_NAME_LEN],
    /// Length of `rmw_name`.
    pub rmw_name_len: usize,
    /// Optional per-Node locator override (`tcp/...`, `udp/...`, …).
    /// Empty inherits the support context's locator.
    pub locator: [u8; MAX_LOCATOR_LEN],
    /// Length of `locator`.
    pub locator_len: usize,
    /// Per-Node domain ID. `NROS_DOMAIN_ID_INHERIT` = inherit support's.
    pub domain_id_override: u32,
    /// SchedContext slot for handle inheritance. 0 = executor default.
    pub sched_context_id: u8,
    /// Reserved for future use; must be zero.
    pub _reserved: [u8; 3],
}

impl Default for nros_node_options_t {
    fn default() -> Self {
        Self {
            namespace: [0u8; MAX_NAMESPACE_LEN],
            namespace_len: 0,
            rmw_name: [0u8; MAX_RMW_NAME_LEN],
            rmw_name_len: 0,
            locator: [0u8; MAX_LOCATOR_LEN],
            locator_len: 0,
            domain_id_override: NROS_DOMAIN_ID_INHERIT,
            sched_context_id: 0,
            _reserved: [0u8; 3],
        }
    }
}

/// Get a zero-initialised `nros_node_options_t`.
///
/// All fields default to "inherit" — `rmw_name_len = 0`, `locator_len = 0`,
/// `domain_id_override = NROS_DOMAIN_ID_INHERIT`, `sched_context_id = 0`.
/// Callers populate only the fields they want to override.
#[unsafe(no_mangle)]
pub extern "C" fn nros_node_get_default_options() -> nros_node_options_t {
    nros_node_options_t::default()
}

/// Get a zero-initialized node.
///
/// # Safety
/// Returns a stack-allocated struct that must be initialized before use.
#[unsafe(no_mangle)]
pub extern "C" fn nros_node_get_zero_initialized() -> nros_node_t {
    nros_node_t::default()
}

/// Initialize a node with default options.
///
/// Equivalent to building a [`nros_node_options_t`] via
/// [`nros_node_get_default_options`], copying `namespace_` into its
/// `namespace` field, and calling [`nros_node_init_ex`]. The shim is
/// kept for source-compatibility with rclc-style callers that pre-date
/// Phase 104.C.8.
///
/// # Parameters
/// * `node` - Pointer to a zero-initialized node
/// * `support` - Pointer to an initialized support context
/// * `name` - Node name (null-terminated string)
/// * `namespace_` - Node namespace (null-terminated string, use "/" for root)
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if any pointer is NULL or strings are invalid
/// * `NROS_RET_NOT_INIT` if support is not initialized
/// * `NROS_RET_ERROR` on initialization failure
///
/// # Safety
/// * All pointers must be valid
/// * `name` and `namespace_` must be valid null-terminated strings
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_node_init(
    node: *mut nros_node_t,
    support: *const nros_support_t,
    name: *const c_char,
    namespace_: *const c_char,
) -> nros_ret_t {
    if namespace_.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let mut options = nros_node_options_t::default();
    options.namespace_len = crate::util::copy_cstr_into(namespace_, &mut options.namespace);
    unsafe { nros_node_init_ex(node, support, name, &options) }
}

/// Phase 104.C.8 — initialize a Node with extended options.
///
/// Thin C wrapper over the Rust `Executor::node_builder(name).rmw(...)
/// .locator(...).domain_id(...).namespace(...).sched(...).build()`
/// chain. Options fields with `*_len == 0` (or `domain_id_override ==
/// NROS_DOMAIN_ID_INHERIT`) inherit from the support context, matching
/// the legacy single-Node behaviour `nros_node_init` provides.
///
/// The `rmw_name` selector drives Phase 104 multi-RMW Node binding: a
/// bridge node can be initialised with `options.rmw_name = "cyclonedds"` while
/// the support context's primary backend is `"zenoh"`, and subsequent
/// publishers/subscribers created via this Node route through the named
/// backend's session. (Internal multi-Session dispatch piggy-backs on
/// the executor's `extra_sessions` cache; see Phase 104.C.3.)
///
/// Currently the inline `node_id` slot stays 0; per-Node multi-RMW
/// dispatch in C lands once the C executor surfaces
/// `Executor::node_builder` (Phase 104.C.8 follow-up). Options fields
/// round-trip into the node struct today so users can write code
/// against the final API surface without waiting for that follow-up.
///
/// # Parameters
/// * `node` - Pointer to a zero-initialized node
/// * `support` - Pointer to an initialized support context
/// * `name` - Node name (null-terminated string)
/// * `options` - Pointer to an [`nros_node_options_t`] (must be non-NULL)
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` on NULL / invalid strings / overrun buffers
/// * `NROS_RET_BAD_SEQUENCE` if the node is already initialized
/// * `NROS_RET_NOT_INIT` if support is not initialized
///
/// # Safety
/// * All pointers must be valid
/// * `name` must be a valid NUL-terminated UTF-8 string
/// * `options` fields must satisfy their declared length invariants
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_node_init_ex(
    node: *mut nros_node_t,
    support: *const nros_support_t,
    name: *const c_char,
    options: *const nros_node_options_t,
) -> nros_ret_t {
    if node.is_null() || support.is_null() || name.is_null() || options.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let node = &mut *node;
    let support_ref = &*support;
    let opts = &*options;

    if node.state != nros_node_state_t::NROS_NODE_STATE_UNINITIALIZED {
        return NROS_RET_BAD_SEQUENCE;
    }
    if support_ref.state != nros_support_state_t::NROS_SUPPORT_STATE_INITIALIZED {
        return NROS_RET_NOT_INIT;
    }

    // Copy name (required — empty rejected).
    node.name_len = crate::util::copy_cstr_into(name, &mut node.name);
    if node.name_len == 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }

    // Validate options length fields against their buffer caps.
    if opts.namespace_len > MAX_NAMESPACE_LEN
        || opts.rmw_name_len > MAX_RMW_NAME_LEN
        || opts.locator_len > MAX_LOCATOR_LEN
    {
        return NROS_RET_INVALID_ARGUMENT;
    }

    // Mirror namespace from options into the node.
    node.namespace[..opts.namespace_len].copy_from_slice(&opts.namespace[..opts.namespace_len]);
    node.namespace_len = opts.namespace_len;

    // Mirror multi-RMW + SchedContext metadata.
    node.rmw_name[..opts.rmw_name_len].copy_from_slice(&opts.rmw_name[..opts.rmw_name_len]);
    node.rmw_name_len = opts.rmw_name_len;
    node.domain_id_override = opts.domain_id_override;
    node.sched_context_id = opts.sched_context_id;

    // `node_id` stays 0 for the legacy single-Node path. Future
    // follow-up (Phase 104.C.8.b) will call into the Executor's
    // `node_builder(...).build()` and store the returned NodeId
    // here when the C executor exposes a stable factory entry.

    node.support = support;
    node.state = nros_node_state_t::NROS_NODE_STATE_INITIALIZED;

    NROS_RET_OK
}

/// Finalize a node.
///
/// # Parameters
/// * `node` - Pointer to an initialized node
///
/// # Returns
/// * `NROS_RET_OK` on success
/// * `NROS_RET_INVALID_ARGUMENT` if node is NULL
/// * `NROS_RET_NOT_INIT` if not initialized
///
/// # Safety
/// * `node` must be a valid pointer to an initialized nros_node_t
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_node_fini(node: *mut nros_node_t) -> nros_ret_t {
    if node.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let node = &mut *node;

    if node.state != nros_node_state_t::NROS_NODE_STATE_INITIALIZED {
        return NROS_RET_NOT_INIT;
    }

    // phase-379 W4 — retire the slot BEFORE marking shutdown, so any entity
    // still holding a reference to this binding fails its own `_fini` with a
    // stale-node error instead of succeeding silently against a dead node.
    retire_generation(node.node_id);

    node.support = ptr::null();
    node.state = nros_node_state_t::NROS_NODE_STATE_SHUTDOWN;

    NROS_RET_OK
}

/// Get the node name.
///
/// # Parameters
/// * `node` - Pointer to an initialized node
///
/// # Returns
/// * Pointer to the node name (null-terminated), or NULL if invalid
///
/// # Safety
/// * `node` must be a valid pointer
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_node_get_name(node: *const nros_node_t) -> *const c_char {
    if node.is_null() {
        return ptr::null();
    }

    let node = &*node;
    if node.state != nros_node_state_t::NROS_NODE_STATE_INITIALIZED {
        return ptr::null();
    }

    node.name.as_ptr() as *const c_char
}

/// Get the node namespace.
///
/// # Parameters
/// * `node` - Pointer to an initialized node
///
/// # Returns
/// * Pointer to the node namespace (null-terminated), or NULL if invalid
///
/// # Safety
/// * `node` must be a valid pointer
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_node_get_namespace(node: *const nros_node_t) -> *const c_char {
    if node.is_null() {
        return ptr::null();
    }

    let node = &*node;
    if node.state != nros_node_state_t::NROS_NODE_STATE_INITIALIZED {
        return ptr::null();
    }

    node.namespace.as_ptr() as *const c_char
}

/// RFC-0088 D4 / phase-421 W2 — the serialization format the backend behind
/// THIS node speaks, as its cross-image identity string (`"cdr"`, `"uorb"`).
///
/// Resolved through the node's own session, not from an image-wide constant.
/// That distinction is the whole point: a single-backend image could answer
/// from the generated `NROS_SERIALIZATION_FORMAT` macro, but a bridge image
/// links two backends and has no single answer — its two nodes sit on two
/// sessions and this function reports each one's format separately. The macro
/// is the fast path for the common case; this is the correct one always.
///
/// # Parameters
/// * `node` - Pointer to an initialized node
///
/// # Returns
/// * Pointer to a static, null-terminated format name; the caller must not
///   free it and it stays valid for the life of the image.
/// * NULL if `node` is NULL / uninitialised, if its session cannot be
///   resolved, or if the backend does not declare a format. NULL is not a
///   synonym for `"cdr"` — a backend that has not said what it speaks has not
///   said CDR, and guessing on its behalf is exactly the fallback the vtable's
///   sibling identity slot spent two phases wrongly promising.
///
/// # Safety
/// * `node` must be a valid pointer
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_node_get_serialization_format(
    node: *const nros_node_t,
) -> *const c_char {
    if node.is_null() {
        return ptr::null();
    }

    let _node = &*node;
    if _node.state != nros_node_state_t::NROS_NODE_STATE_INITIALIZED {
        return ptr::null();
    }

    #[cfg(feature = "rmw-cffi")]
    {
        // The same resolver every entity-creation path uses, so the format
        // reported here is the format of the session a publisher created on
        // this node would actually be built against — both the multi-session
        // (`nros_executor_node_init`) and legacy single-session shapes.
        match resolve_session_and_domain(_node) {
            Some((session, _domain)) => session.serialization_format_cstr(),
            None => ptr::null(),
        }
    }

    #[cfg(not(feature = "rmw-cffi"))]
    {
        ptr::null()
    }
}

/// Phase 88.12 — return the `nros::Logger` keyed on this node's name.
///
/// The returned handle is opaque from the C side; pass it to
/// `nros_log_info(...)` / `nros_log_warn(...)` / etc. (declared in
/// `<nros/log.h>`). The lifetime is `'static` — loggers live for the
/// process; callers must NOT free the returned pointer.
///
/// # Parameters
/// * `node` - Pointer to an initialized node.
///
/// # Returns
/// * Opaque `nros_logger_t *` (= `&'static nros_log::Logger`), or NULL
///   if `node` is NULL / uninitialised.
///
/// # Safety
/// * `node` must be a valid pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_node_get_logger(
    node: *const nros_node_t,
) -> *const core::ffi::c_void {
    if node.is_null() {
        return core::ptr::null();
    }

    let node = &*node;
    if node.state != nros_node_state_t::NROS_NODE_STATE_INITIALIZED {
        return core::ptr::null();
    }

    // Name lives in `node.name: [u8; N]` as a NUL-terminated C string.
    // Find the NUL + slice to a `&str` before handing to nros-log.
    let name_bytes = &node.name[..];
    let nul = name_bytes.iter().position(|&b| b == 0).unwrap_or(0);
    let name = core::str::from_utf8(&name_bytes[..nul]).unwrap_or("");
    let logger: &'static nros_log::Logger = nros_log::get_logger(name);
    (logger as *const nros_log::Logger).cast()
}

// ============================================================================
// phase-417 W5.c — the node accessors filed as `gap`
//
// Every one of these is a FORWARDER. The four rows
// (`c:node_is_valid`, `c:node_get_domain_id`,
// `c:node_get_fully_qualified_name`, `c:node_resolve_name`) were filed as
// gaps because the answer lives somewhere the C node does not: on the
// executor, or behind the ONE name-resolution seam. Reaching through
// `node.executor` for it is a forward, not a second source of truth
// (RFC-0002 — one executor per image; RFC-0019 — Rust implements, C
// delegates). Nothing here adds a field, a cache, or a policy.
// ============================================================================

/// Read the NUL-terminated C string out of one of `nros_node_t`'s inline
/// buffers as a `&str`, empty on any surprise.
///
/// The three accessors below all need this and the logger accessor above
/// open-codes it; a fourth spelling would be the "second idiom instead of a
/// shared helper" the class rule warns about.
fn inline_str(buf: &[u8], len: usize) -> &str {
    // `*_len` excludes the NUL, but a caller-built struct can disagree with
    // itself, so clamp on the buffer AND stop at the first NUL.
    let end = len.min(buf.len());
    let bytes = &buf[..end];
    let nul = bytes.iter().position(|&b| b == 0).unwrap_or(end);
    core::str::from_utf8(&bytes[..nul]).unwrap_or("")
}

/// Copy `s` (plus a NUL) into a caller-owned buffer.
///
/// `NROS_RET_FULL` when it does not fit — never a truncated name. A truncated
/// FQN is a name that routes somewhere else, which is the failure mode
/// `Node::new`'s bounded push already refuses one layer down.
///
/// # Safety
/// `out` must be writable for `out_size` bytes.
unsafe fn write_cstr_out(s: &str, out: *mut c_char, out_size: usize) -> nros_ret_t {
    if out.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    if s.len() + 1 > out_size {
        return NROS_RET_FULL;
    }
    let dst = unsafe { core::slice::from_raw_parts_mut(out, s.len() + 1) };
    dst[..s.len()].copy_from_slice(s.as_bytes());
    dst[s.len()] = 0;
    NROS_RET_OK
}

/// Is this node handle usable?
///
/// rcl's `rcl_node_is_valid`. We already had `nros_node_state_t` and
/// `nros_support_is_valid` for the support object; the node had no predicate
/// that read its own state, which is the whole of gap `c:node_is_valid`.
///
/// Two questions, both answered, because either one alone is a lie:
///
/// * the handle's own state is `INITIALIZED` — `nros_node_fini` sets
///   `SHUTDOWN`, so a finalised node reports false; and
/// * the executor slot it is bound to still carries the generation it was
///   bound at (phase-379 W4). C has no move semantics, so
///   `nros_node_t copy = original;` is legal and silent — the copy keeps
///   `state == INITIALIZED` after the original is finalised, and only the
///   generation catches that.
///
/// A legacy (`nros_node_init`) node is not executor-bound, so only the first
/// question applies to it.
///
/// # Safety
/// * `node` must be NULL or point to a valid `nros_node_t`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_node_is_valid(node: *const nros_node_t) -> bool {
    if node.is_null() {
        return false;
    }
    let node_ref = &*node;
    if node_ref.state != nros_node_state_t::NROS_NODE_STATE_INITIALIZED {
        return false;
    }
    if node_ref.is_multi_session() {
        return node_ref_is_live(node_ref_of(node));
    }
    true
}

/// The ROS domain this node's entities are declared on.
///
/// rcl's `rcl_node_get_domain_id(node, size_t *domain_id)`. Gap
/// `c:node_get_domain_id` records why it was missing: the domain is an INPUT
/// to `nros_support_init` and could not be read back, while on a device the
/// value that actually won came from the boot ladder.
///
/// So this forwards to [`resolve_session_and_domain`] — the one place the
/// ladder is decoded (per-node override → C-ABI byte → the session's own
/// domain). Re-deriving it here is precisely issue 0972's defect: the same
/// decode at a third call site, where `NROS_DOMAIN_ID_EXPLICIT_ZERO` (255)
/// reads as an out-of-range domain and plain `0` reads as domain 0 rather
/// than "unset".
///
/// Out-param + status rather than a bare return, because this genuinely can
/// fail to answer — an uninitialised support context, a retired node slot, or
/// a domain byte above `DOMAIN_ID_MAX` all have no domain to report, and
/// `0` is a legal domain that must not stand in for any of them.
///
/// Returns `NROS_RET_UNSUPPORTED` in a build with no RMW (`rmw-cffi` off):
/// there is no session, so there is no resolved domain to read.
///
/// # Safety
/// * `node` must be NULL or point to a valid `nros_node_t`.
/// * `domain_id` must be NULL or writable.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_node_get_domain_id(
    node: *const nros_node_t,
    domain_id: *mut u32,
) -> nros_ret_t {
    validate_not_null!(node, domain_id);
    let node_ref = &*node;
    validate_state!(node_ref, nros_node_state_t::NROS_NODE_STATE_INITIALIZED);

    #[cfg(feature = "rmw-cffi")]
    {
        match resolve_session_and_domain(node_ref) {
            Some((_session, resolved)) => {
                *domain_id = resolved;
                NROS_RET_OK
            }
            None => NROS_RET_NOT_INIT,
        }
    }
    #[cfg(not(feature = "rmw-cffi"))]
    {
        let _ = node_ref;
        NROS_RET_UNSUPPORTED
    }
}

/// The node's namespace and name as one string — `/ns/name`, the form that
/// appears on the wire.
///
/// rcl's `rcl_node_get_fully_qualified_name`. Gap
/// `c:node_get_fully_qualified_name` records that we exposed
/// `nros_node_get_name` and `nros_node_get_namespace` separately and never
/// their composition.
///
/// The composition is NOT written here. It is `nros_node::names::expand_name`
/// with the private-name source `~`, which is by definition
/// `/<ns>/<node>` — the same seam every entity name on this node goes
/// through (`Executor::resolve_entity_name_for`), so a node's FQN and its
/// entities' FQNs can never disagree about namespace normalisation. A second
/// `push(namespace); push('/'); push(name)` in the C layer is exactly
/// RFC-0020's violation class 4 (name construction in a wrapper), and it is
/// what the four sibling implementations of this already spelled differently.
///
/// **Divergence from rcl, deliberate:** rcl returns `const char *` into
/// node-owned storage. We have no allocator and the node struct holds no
/// composed buffer, so the caller supplies one. `NROS_RET_FULL` when it is
/// too small — a truncated FQN names a different node.
///
/// # Safety
/// * `node` must be NULL or point to a valid `nros_node_t`.
/// * `output_name` must be NULL or writable for `output_size` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_node_get_fully_qualified_name(
    node: *const nros_node_t,
    output_name: *mut c_char,
    output_size: usize,
) -> nros_ret_t {
    validate_not_null!(node, output_name);
    let node_ref = &*node;
    validate_state!(node_ref, nros_node_state_t::NROS_NODE_STATE_INITIALIZED);

    let name = inline_str(&node_ref.name, node_ref.name_len);
    if name.is_empty() {
        return NROS_RET_NOT_INIT;
    }
    let namespace = inline_str(&node_ref.namespace, node_ref.namespace_len);

    match nros_node::names::expand_name("~", name, namespace) {
        Ok(fqn) => write_cstr_out(fqn.as_str(), output_name, output_size),
        Err(()) => NROS_RET_FULL,
    }
}

/// Expand `input_name` against this node's namespace and apply its remap
/// rules — what a topic or service name a caller builds at runtime will
/// actually become on the wire.
///
/// rcl's `rcl_node_resolve_name`. Gap `c:node_resolve_name` records the
/// reason it was missing: our names are resolved at codegen/launch time
/// (RFC-0046), so a C caller constructing one dynamically had no way to ask.
///
/// Forwards to `Executor::resolve_entity_name_for` — the identical call the
/// registration paths in `executor.rs` make for every publisher,
/// subscription, service and action. That is the point: this answers what
/// creating the entity WOULD do, not what a parallel implementation thinks
/// it would do.
///
/// `only_expand` is rcl's own parameter and carries rcl's meaning: `true`
/// applies ROS 2 name expansion (`~`, relative → FQN) and ignores remap
/// rules; `false` also applies them.
///
/// **Remaps live on the executor, so `only_expand == false` needs an
/// executor-bound node** (`nros_executor_node_init`). On the legacy
/// `nros_node_init` path this returns `NROS_RET_NOT_INIT` rather than
/// quietly expanding without the rules — silently dropping a routing rule is
/// the failure this whole campaign exists to stop, and the caller who wants
/// expansion alone can ask for it by passing `true`.
///
/// **Divergences from rcl, both forced:** no `rcl_allocator_t` (we have no
/// allocator, so the caller owns the buffer and `NROS_RET_FULL` reports a
/// short one), and no `is_service` (it selects between rcl's topic- and
/// service-name VALIDATORS, which we do not ship; taking the argument and
/// ignoring it would be the inert-parameter defect RFC-0087 §"The hazard"
/// names).
///
/// # Safety
/// * `node` must be NULL or point to a valid `nros_node_t`.
/// * `input_name` must be NULL or a valid NUL-terminated C string.
/// * `output_name` must be NULL or writable for `output_size` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_node_resolve_name(
    node: *const nros_node_t,
    input_name: *const c_char,
    only_expand: bool,
    output_name: *mut c_char,
    output_size: usize,
) -> nros_ret_t {
    validate_not_null!(node, input_name, output_name);
    let node_ref = &*node;
    validate_state!(node_ref, nros_node_state_t::NROS_NODE_STATE_INITIALIZED);

    let Ok(source) = core::ffi::CStr::from_ptr(input_name).to_str() else {
        return NROS_RET_INVALID_ARGUMENT;
    };
    if source.is_empty() {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let name = inline_str(&node_ref.name, node_ref.name_len);
    if name.is_empty() {
        return NROS_RET_NOT_INIT;
    }
    let namespace = inline_str(&node_ref.namespace, node_ref.namespace_len);

    if only_expand {
        return match nros_node::names::expand_name(source, name, namespace) {
            Ok(expanded) => write_cstr_out(expanded.as_str(), output_name, output_size),
            Err(()) => NROS_RET_INVALID_ARGUMENT,
        };
    }

    // Remap rules live in the executor's table. No executor reachable means
    // the rules are unreadable, NOT absent — say so.
    if !node_ref.is_multi_session() {
        return NROS_RET_NOT_INIT;
    }
    if !node_ref_is_live(node_ref_of(node)) {
        return NROS_RET_STALE_NODE;
    }
    let exec_mut = &mut *(node_ref.executor as *mut crate::executor::nros_executor_t);
    let rust_exec = crate::executor::get_executor(&mut exec_mut._opaque);
    match rust_exec.resolve_entity_name_for(name, namespace, source) {
        Ok(resolved) => write_cstr_out(resolved.as_str(), output_name, output_size),
        Err(()) => NROS_RET_INVALID_ARGUMENT,
    }
}

#[cfg(kani)]
mod verification {
    use super::*;
    use crate::error::*;

    #[kani::proof]
    #[kani::unwind(5)]
    fn node_init_null_ptrs() {
        let name = b"test\0";
        let ns = b"/\0";

        // NULL node → INVALID_ARGUMENT
        let mut support = crate::support::nros_support_get_zero_initialized();
        assert_eq!(
            unsafe {
                nros_node_init(
                    core::ptr::null_mut(),
                    &support,
                    name.as_ptr() as *const core::ffi::c_char,
                    ns.as_ptr() as *const core::ffi::c_char,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL support → INVALID_ARGUMENT
        let mut node = nros_node_get_zero_initialized();
        assert_eq!(
            unsafe {
                nros_node_init(
                    &mut node,
                    core::ptr::null(),
                    name.as_ptr() as *const core::ffi::c_char,
                    ns.as_ptr() as *const core::ffi::c_char,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL name → INVALID_ARGUMENT
        assert_eq!(
            unsafe {
                nros_node_init(
                    &mut node,
                    &support,
                    core::ptr::null(),
                    ns.as_ptr() as *const core::ffi::c_char,
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );

        // NULL namespace → INVALID_ARGUMENT
        assert_eq!(
            unsafe {
                nros_node_init(
                    &mut node,
                    &support,
                    name.as_ptr() as *const core::ffi::c_char,
                    core::ptr::null(),
                )
            },
            NROS_RET_INVALID_ARGUMENT,
        );
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn node_zero_initialized_state() {
        let node = nros_node_get_zero_initialized();
        assert_eq!(node.state, nros_node_state_t::NROS_NODE_STATE_UNINITIALIZED);
        assert!(node.support.is_null());
    }
}

impl nros_node_t {
    /// Get the node name as a string slice
    pub(crate) fn get_name_str(&self) -> &str {
        unsafe { core::str::from_utf8_unchecked(&self.name[..self.name_len]) }
    }

    /// Get the namespace as a string slice
    pub(crate) fn get_namespace_str(&self) -> &str {
        unsafe { core::str::from_utf8_unchecked(&self.namespace[..self.namespace_len]) }
    }

    /// Get the support context
    pub(crate) unsafe fn get_support(&self) -> Option<&nros_support_t> {
        if self.support.is_null() {
            None
        } else {
            Some(&*self.support)
        }
    }

    /// Get the support context mutably
    ///
    /// This returns a mutable reference from an immutable pointer, which is
    /// intentional for C FFI where the node stores a const pointer but the
    /// support may need to be mutated.
    #[allow(clippy::mut_from_ref)]
    pub(crate) unsafe fn get_support_mut(&self) -> Option<&mut nros_support_t> {
        if self.support.is_null() {
            None
        } else {
            Some(&mut *(self.support as *mut nros_support_t))
        }
    }

    /// Phase 156 Sub-bug D — true on Nodes bound via
    /// `nros_executor_node_init` (multi-Session bridge path). False on
    /// nodes initialised via `nros_node_init` / `nros_node_init_ex`
    /// (legacy single-Session path).
    #[inline]
    pub(crate) fn is_multi_session(&self) -> bool {
        self.node_id != 0 && !self.executor.is_null()
    }
}

/// Phase 156 Sub-bug D — resolve the per-Node session + effective
/// domain id for entity-init paths. Branches on `is_multi_session`:
///   * Multi-session: dereferences `node.executor`, walks the
///     NodeRecord table via [`Executor::node_session_mut`], pulls the
///     domain id from `node.domain_id_override` (or the executor's
///     support when the Node opted to inherit).
///   * Single-session: falls back to `node.get_support_mut` +
///     `support.get_session_mut`, mirrors the pre-Phase-156 dispatch.
///
/// Returns `None` when any lookup fails so callers can map to
/// `NROS_RET_NOT_INIT`.
#[cfg(feature = "rmw-cffi")]
#[allow(clippy::mut_from_ref)]
pub(crate) unsafe fn resolve_session_and_domain(
    node: &nros_node_t,
) -> Option<(&mut nros::internals::RmwSession, u32)> {
    if node.is_multi_session() {
        let exec_mut = &mut *(node.executor as *mut crate::executor::nros_executor_t);
        let support_ptr = exec_mut.support;
        let rust_exec = crate::executor::get_executor(&mut exec_mut._opaque);
        let node_id = nros_node::executor::node_record::NodeId::from_raw(node.node_id);
        let session = rust_exec.node_session_mut(node_id)?;
        // Inherit from the SESSION when there is no support context. The old
        // `else { 0 }` silently put every entity on domain 0 while the session
        // itself was open on another domain -- the node liveliness token used
        // the configured domain and every publisher, subscriber and their
        // tokens used 0. Since the domain is the first element of every key
        // rmw_zenoh matches on, discovery simply never matched, with no error
        // anywhere (issue 0801).
        let domain_id = if node.domain_id_override != NROS_DOMAIN_ID_INHERIT {
            node.domain_id_override
        } else if !support_ptr.is_null() {
            // issue 0972 — decode, do not read the raw byte.
            resolve_domain_from_c_abi((*support_ptr).domain_id, session.domain_id())?
        } else {
            session.domain_id()
        };
        Some((session, domain_id))
    } else {
        let support_mut = node.get_support_mut()?;
        if support_mut.state != crate::support::nros_support_state_t::NROS_SUPPORT_STATE_INITIALIZED
        {
            return None;
        }
        // Prefer the SESSION's domain. `support.domain_id` is resolved from the
        // C ABI argument, where 0 means "unset" and resolves to 0 -- but the
        // session was opened from RmwConfig, which carries CONFIG_NROS_DOMAIN_ID.
        // When a caller leaves the C argument unset the two disagree, and every
        // entity lands on domain 0 while the node liveliness token uses the
        // configured domain. Discovery then never matches and nothing reports an
        // error, because the domain is just the first element of the key
        // (issue 0801).
        let raw_domain = support_mut.domain_id;
        let session = support_mut.get_session_mut()?;
        // issue 0972 — decode, do not read the raw byte.
        let domain_id = resolve_domain_from_c_abi(raw_domain, session.domain_id())?;
        Some((session, domain_id))
    }
}

#[cfg(test)]
mod node_ref_tests {
    use super::*;

    /// issue 0972 — the C ABI `domain_id` byte is an ENCODING, not a domain.
    ///
    /// Both entity-resolution sites used to read it raw and compare against 0.
    /// The regression that matters is `NROS_DOMAIN_ID_EXPLICIT_ZERO`: it is 255,
    /// so `!= 0` was true and 255 was used AS the domain — not merely the wrong
    /// domain, but one outside the legal ROS range (0..=232). A caller asking
    /// for the default domain got an impossible one, silently, because a domain
    /// is just the first element of the discovery key (issue 0801).
    #[test]
    fn explicit_zero_resolves_to_domain_zero_not_255() {
        // The session is on a different domain, so a fallback would be visible.
        assert_eq!(
            resolve_domain_from_c_abi(crate::support::NROS_DOMAIN_ID_EXPLICIT_ZERO, 7),
            Some(0),
            "explicit-zero must mean domain 0, not the sentinel's own value"
        );
    }

    /// Plain 0 is UNSET and must defer to the session. This half was already
    /// correct; it is pinned so a future simplification cannot collapse the two
    /// meanings back together.
    #[test]
    fn unset_defers_to_the_session_domain() {
        assert_eq!(resolve_domain_from_c_abi(0, 7), Some(7));
        assert_eq!(resolve_domain_from_c_abi(0, 0), Some(0));
    }

    /// An ordinary domain passes through unchanged.
    #[test]
    fn an_explicit_domain_wins_over_the_session() {
        assert_eq!(resolve_domain_from_c_abi(5, 7), Some(5));
        assert_eq!(resolve_domain_from_c_abi(232, 7), Some(232));
    }

    /// issue 0972 follow-up — the ROS domain space caps at 232, so a byte above
    /// it is not a domain and must not become one.
    ///
    /// `baked_domain_from_c_abi` deliberately does not range-check (its
    /// docstring defers that to `ExecutorConfig::try_resolve`), and this
    /// resolver does not go through `try_resolve` — it feeds entity keyexprs
    /// directly. Without this guard 233..=254 were ASSIGNED as domains:
    /// unroutable, and silent, because the domain is just the first element of
    /// the discovery key (issue 0801). Refusing is the loud option available
    /// here; the caller turns `None` into a failed entity creation.
    #[test]
    fn a_domain_above_the_ros_maximum_is_refused_not_assigned() {
        for raw in [233u8, 240, 254] {
            assert_eq!(
                resolve_domain_from_c_abi(raw, 7),
                None,
                "domain {raw} exceeds the ROS maximum and must not be assigned"
            );
        }
        // The boundary itself is legal and must still pass.
        assert_eq!(resolve_domain_from_c_abi(232, 7), Some(232));
        // 255 is the explicit-zero SENTINEL, not an out-of-range domain — it
        // must keep decoding to 0 rather than being caught by the range guard.
        assert_eq!(
            resolve_domain_from_c_abi(crate::support::NROS_DOMAIN_ID_EXPLICIT_ZERO, 7),
            Some(0)
        );
    }

    /// The two meanings of "zero" are distinguishable — which is the whole
    /// point of the sentinel, and what reading the raw byte threw away.
    #[test]
    fn explicit_zero_and_unset_are_not_the_same_input() {
        let session = 9;
        assert_ne!(
            resolve_domain_from_c_abi(crate::support::NROS_DOMAIN_ID_EXPLICIT_ZERO, session),
            resolve_domain_from_c_abi(0, session),
        );
    }

    /// phase-379 W4 — a reference minted before `nros_node_fini` stops
    /// resolving after it.
    ///
    /// This is the whole behavioural change. Before W4 the entity held a
    /// `*const nros_node_t` that nothing dereferenced, so "finalise the node,
    /// then finalise its publisher" succeeded silently and the teardown-order
    /// obligation the C API places on the caller was unenforceable.
    #[test]
    fn retiring_a_slot_invalidates_references_minted_against_it() {
        // Slot 3 rather than 0: the primary node is the legacy path and the
        // interesting case is a bound one.
        let id = 3u8;
        let before = nros_node_ref_t {
            node_id: id,
            generation: current_generation(id),
        };
        assert!(node_ref_is_live(before), "a fresh reference must resolve");

        retire_generation(id);
        assert!(
            !node_ref_is_live(before),
            "a reference minted before the slot was retired must stop resolving"
        );

        let after = nros_node_ref_t {
            node_id: id,
            generation: current_generation(id),
        };
        assert!(node_ref_is_live(after), "the rebound slot issues live refs");
        assert_ne!(
            before.generation, after.generation,
            "retiring must change the generation, or nothing is detectable"
        );
    }

    /// `generation == 0` is reserved, so a zeroed struct never resolves.
    ///
    /// A zeroed handle is the commonest C mistake, and it is exactly the shape
    /// `calloc`, a `static`, and `memset` all produce.
    #[test]
    fn a_zeroed_reference_is_never_live() {
        let zeroed = nros_node_ref_t {
            node_id: 0,
            generation: 0,
        };
        assert!(!zeroed.is_bound());
        assert!(!node_ref_is_live(zeroed));
        assert!(!node_ref_is_live(nros_node_ref_t::none()));
    }

    /// The wrap never lands on 0, because 0 means "unbound".
    #[test]
    fn generation_wrap_skips_the_reserved_value() {
        let id = (nros_node::config::MAX_NODES - 1) as u8;
        NODE_GENERATIONS[id as usize].store(u32::MAX, core::sync::atomic::Ordering::Release);
        retire_generation(id);
        assert_ne!(
            current_generation(id),
            0,
            "wrapping past u32::MAX must skip 0, or a zeroed struct would resolve"
        );
    }
}

// ============================================================================
// phase-417 W5.c tests
// ============================================================================

#[cfg(test)]
mod accessor_tests {
    use super::*;

    /// A node on the legacy (`nros_node_init`) path: initialised, not bound
    /// to an executor.
    fn unbound_node(name: &str, namespace: &str) -> nros_node_t {
        let mut node = nros_node_t::default();
        node.name[..name.len()].copy_from_slice(name.as_bytes());
        node.name_len = name.len();
        node.namespace[..namespace.len()].copy_from_slice(namespace.as_bytes());
        node.namespace_len = namespace.len();
        node.state = nros_node_state_t::NROS_NODE_STATE_INITIALIZED;
        node
    }

    fn read_out(buf: &[u8]) -> &str {
        let nul = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
        core::str::from_utf8(&buf[..nul]).unwrap()
    }

    #[test]
    fn is_valid_tracks_the_node_state() {
        let mut node = unbound_node("talker", "/");
        assert!(unsafe { nros_node_is_valid(&node) });

        assert_eq!(unsafe { nros_node_fini(&mut node) }, NROS_RET_OK);
        assert!(
            !unsafe { nros_node_is_valid(&node) },
            "a finalised node is not usable"
        );

        assert!(!unsafe { nros_node_is_valid(core::ptr::null()) });
        assert!(!unsafe { nros_node_is_valid(&nros_node_t::default()) });
    }

    /// The FQN is namespace + name, normalised by the SAME seam entity names
    /// go through. The root-namespace collapse is the case a hand-written
    /// `push(ns); push('/'); push(name)` gets wrong: it yields `//talker`.
    #[test]
    fn fully_qualified_name_composes_namespace_and_name() {
        for (ns, expected) in [
            ("/", "/talker"),
            ("", "/talker"),
            ("/sensing", "/sensing/talker"),
            // A namespace written without its leading slash normalises, the
            // same way it does for every topic on this node.
            ("sensing", "/sensing/talker"),
            ("/sensing/", "/sensing/talker"),
        ] {
            let node = unbound_node("talker", ns);
            let mut buf = [0u8; 64];
            assert_eq!(
                unsafe {
                    nros_node_get_fully_qualified_name(
                        &node,
                        buf.as_mut_ptr() as *mut c_char,
                        buf.len(),
                    )
                },
                NROS_RET_OK,
                "namespace {ns:?}"
            );
            assert_eq!(read_out(&buf), expected, "namespace {ns:?}");
        }
    }

    /// A short buffer is an error, never a truncated name. A truncated FQN
    /// names a different node.
    #[test]
    fn fully_qualified_name_refuses_a_short_buffer() {
        let node = unbound_node("talker", "/sensing");
        // "/sensing/talker" is 15 bytes + NUL.
        let mut exact = [0u8; 16];
        assert_eq!(
            unsafe {
                nros_node_get_fully_qualified_name(
                    &node,
                    exact.as_mut_ptr() as *mut c_char,
                    exact.len(),
                )
            },
            NROS_RET_OK
        );
        assert_eq!(read_out(&exact), "/sensing/talker");

        let mut short = [0xAAu8; 15];
        assert_eq!(
            unsafe {
                nros_node_get_fully_qualified_name(
                    &node,
                    short.as_mut_ptr() as *mut c_char,
                    short.len(),
                )
            },
            NROS_RET_FULL
        );
        assert!(
            short.iter().all(|&b| b == 0xAA),
            "a refused write must leave the caller's buffer untouched"
        );
    }

    /// `only_expand` applies ROS 2 name expansion and nothing else, which is
    /// answerable without an executor.
    #[test]
    fn resolve_name_expands_against_the_nodes_namespace() {
        let node = unbound_node("filter", "/sensing");
        for (input, expected) in [
            ("/scan", "/scan"),
            ("scan", "/sensing/scan"),
            ("~/points", "/sensing/filter/points"),
        ] {
            let mut buf = [0u8; 64];
            let input_c = alloc_cstr(input);
            assert_eq!(
                unsafe {
                    nros_node_resolve_name(
                        &node,
                        input_c.as_ptr() as *const c_char,
                        true,
                        buf.as_mut_ptr() as *mut c_char,
                        buf.len(),
                    )
                },
                NROS_RET_OK,
                "input {input:?}"
            );
            assert_eq!(read_out(&buf), expected, "input {input:?}");
        }
    }

    /// Remap rules live on the executor. Asking for them on a node that is
    /// not executor-bound must FAIL rather than quietly returning the
    /// expansion — a silently dropped routing rule is the failure class this
    /// campaign exists to stop.
    #[test]
    fn resolve_name_refuses_remaps_it_cannot_read() {
        let node = unbound_node("filter", "/sensing");
        let input_c = alloc_cstr("scan");
        let mut buf = [0xAAu8; 64];
        assert_eq!(
            unsafe {
                nros_node_resolve_name(
                    &node,
                    input_c.as_ptr() as *const c_char,
                    false,
                    buf.as_mut_ptr() as *mut c_char,
                    buf.len(),
                )
            },
            NROS_RET_NOT_INIT
        );
        assert!(
            buf.iter().all(|&b| b == 0xAA),
            "the refusal must not look like a successful expansion"
        );
    }

    #[test]
    fn resolve_name_refuses_null_and_empty_input() {
        let node = unbound_node("filter", "/sensing");
        let mut buf = [0u8; 64];
        let empty = alloc_cstr("");
        assert_eq!(
            unsafe {
                nros_node_resolve_name(
                    &node,
                    core::ptr::null(),
                    true,
                    buf.as_mut_ptr() as *mut c_char,
                    buf.len(),
                )
            },
            NROS_RET_INVALID_ARGUMENT
        );
        assert_eq!(
            unsafe {
                nros_node_resolve_name(
                    &node,
                    empty.as_ptr() as *const c_char,
                    true,
                    buf.as_mut_ptr() as *mut c_char,
                    buf.len(),
                )
            },
            NROS_RET_INVALID_ARGUMENT
        );
    }

    /// The domain is not readable from a node with no session behind it.
    /// `0` is a legal domain, so it must not stand in for "unknown".
    #[test]
    fn domain_id_on_a_session_less_node_is_reported_not_guessed() {
        let node = unbound_node("talker", "/");
        let mut domain: u32 = 0xFFFF;
        let rc = unsafe { nros_node_get_domain_id(&node, &mut domain) };
        assert_ne!(rc, NROS_RET_OK);
        assert_eq!(domain, 0xFFFF, "a failed read must not write the out-param");

        assert_eq!(
            unsafe { nros_node_get_domain_id(&node, core::ptr::null_mut()) },
            NROS_RET_INVALID_ARGUMENT
        );
        assert_eq!(
            unsafe { nros_node_get_domain_id(core::ptr::null(), &mut domain) },
            NROS_RET_INVALID_ARGUMENT
        );
    }

    /// `inline_str` stops at the first NUL AND at the buffer bound, because a
    /// caller-built struct can disagree with itself.
    #[test]
    fn inline_str_is_bounded_by_both_the_length_and_the_nul() {
        let mut buf = [0u8; 8];
        buf[..3].copy_from_slice(b"abc");
        assert_eq!(inline_str(&buf, 3), "abc");
        // A length past the NUL still stops at the NUL.
        assert_eq!(inline_str(&buf, 8), "abc");
        // A length past the buffer is clamped rather than read out of bounds.
        assert_eq!(inline_str(&buf, 999), "abc");
    }

    /// Test-only NUL-terminated byte buffer. The crate is `no_std` but its
    /// test profile is hosted, so a `Vec` is available here and nowhere in
    /// the shipped code.
    fn alloc_cstr(s: &str) -> std::vec::Vec<u8> {
        let mut v = std::vec::Vec::with_capacity(s.len() + 1);
        v.extend_from_slice(s.as_bytes());
        v.push(0);
        v
    }
}
