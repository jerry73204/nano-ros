//! ZenohSession implementation

use nros_rmw::{
    QoSProfile, ServiceInfo, Session, SessionMode, TopicInfo, TransportConfig, TransportError,
};

use super::{
    CONFIG_PROPERTY_SIZE, Context, EntityKind, LOCATOR_BUFFER_SIZE,
    LivelinessEntity as Ros2LivelinessEntity, LivelinessToken, MAX_SESSION_PROPERTIES,
    Ros2Liveliness, ZenohId,
    publisher::ZenohPublisher,
    service::{ZenohServiceClient, ZenohServiceServer},
    subscriber::ZenohSubscriber,
};

/// Issue 0330 — normalize an incoming locator to "supplied" vs "absent".
///
/// RMW-agnostic layers no longer carry a zenoh default; they hand this
/// backend either `None` or the empty string when the user supplied
/// nothing (the C/C++ ABI edges cannot express `None`, so `""` is the
/// wire form of "absent" — `NROS_ENTRY_LOCATOR` defaults to `""`, and
/// `ExecutorConfig::try_resolve`'s bottom rung yields `""`).
///
/// Both spellings collapse to `None` here. That matters: `zpico_init_with_config`
/// (`zpico-sys/c/zpico/zpico.c`) inserts whatever non-NULL locator it is given
/// as the zenoh CONNECT endpoint, so a bare `""` would be configured as a real
/// (broken) endpoint rather than meaning "no endpoint".
pub fn normalize_locator(locator: Option<&str>) -> Option<&str> {
    match locator {
        Some(l) if !l.is_empty() => Some(l),
        _ => None,
    }
}

/// Issue 0330 — the client-mode locator this backend will actually dial:
/// the caller's value when they supplied one, else [`crate::DEFAULT_LOCATOR`]
/// on HOSTED builds, else nothing.
///
/// This is the ONLY place the zenoh default is applied. Agnostic layers must
/// not pre-substitute it.
///
/// The default is deliberately HOSTED-only. Every layer it replaced was hosted
/// too — `nros-node`'s old `DEFAULT_LOCATOR` was `#[cfg(feature = "std")]`,
/// and `node.hpp`'s rung sat behind `NROS_CPP_STD || __STDC_HOSTED__`. On an
/// embedded image `tcp/127.0.0.1:7447` is the board's own loopback and dialling
/// it is strictly worse than the established no-locator behaviour: no connect
/// endpoint, so zenoh-pico falls back to multicast scouting. Returning `None`
/// here is what preserves that.
///
/// The gate is `target_os = "none"`, matching `nros-rmw-cffi`'s registration
/// seam — NOT `feature = "std"`, which this crate leaves off even on hosted
/// builds (`--features platform-posix` alone), so a `std` gate would have
/// silently disabled the default everywhere.
pub fn effective_client_locator(locator: Option<&str>) -> Option<&str> {
    match normalize_locator(locator) {
        Some(l) => Some(l),
        #[cfg(not(target_os = "none"))]
        None => Some(crate::DEFAULT_LOCATOR),
        #[cfg(target_os = "none")]
        None => None,
    }
}

#[cfg(feature = "std")]
fn append_locator_param(
    buf: &mut [u8; LOCATOR_BUFFER_SIZE],
    len: &mut usize,
    first_param: &mut bool,
    key: &str,
    value: &str,
) -> Result<(), TransportError> {
    let separator = if *first_param { b'#' } else { b';' };
    let needed = 1 + key.len() + 1 + value.len();
    if *len + needed >= buf.len() {
        return Err(TransportError::InvalidArgument);
    }
    buf[*len] = separator;
    *len += 1;
    buf[*len..*len + key.len()].copy_from_slice(key.as_bytes());
    *len += key.len();
    buf[*len] = b'=';
    *len += 1;
    buf[*len..*len + value.len()].copy_from_slice(value.as_bytes());
    *len += value.len();
    *first_param = false;
    Ok(())
}

#[cfg(feature = "std")]
fn append_tls_env_to_locator(
    loc: &str,
    buf: &mut [u8; LOCATOR_BUFFER_SIZE],
    len: &mut usize,
) -> Result<(), TransportError> {
    if !loc.starts_with("tls/") {
        return Ok(());
    }

    let mut first_param = !loc.contains('#');
    if !loc.contains("root_ca_certificate=")
        && let Ok(value) = std::env::var("ZENOH_TLS_ROOT_CA_CERTIFICATE")
    {
        append_locator_param(buf, len, &mut first_param, "root_ca_certificate", &value)?;
    }
    if !loc.contains("root_ca_certificate_base64=")
        && let Ok(value) = std::env::var("ZENOH_TLS_ROOT_CA_CERTIFICATE_BASE64")
    {
        append_locator_param(
            buf,
            len,
            &mut first_param,
            "root_ca_certificate_base64",
            &value,
        )?;
    }
    if !loc.contains("verify_name_on_connect=")
        && let Ok(value) = std::env::var("ZENOH_TLS_VERIFY_NAME_ON_CONNECT")
    {
        append_locator_param(buf, len, &mut first_param, "verify_name_on_connect", &value)?;
    }
    Ok(())
}

// ============================================================================
// ZenohSession
// ============================================================================

/// Maximum number of distinct per-node NN liveliness tokens the session will
/// hold.  Derived from `NROS_EXECUTOR_MAX_NODES` (build.rs) so it tracks the
/// executor's per-process node cap — one session hosts at most that many graph
/// nodes, so a token is never silently dropped on overflow at the default cap.
use crate::config::MAX_PER_NODE_LIVELINESS;

/// Zenoh session wrapping nros-rmw-zenoh Context
///
/// This session requires manual polling via `spin_once()` or `poll()`.
/// There are no background threads.
pub struct ZenohSession {
    context: Context,
    /// Fix #104 — node liveliness token declared at session open so the
    /// primary node is visible in `ros2 node list`.  A dropped
    /// `LivelinessToken` is immediately undeclared, so we MUST hold it
    /// for the entire session lifetime.  `None` when `config.node_name`
    /// is empty or `should_declare_liveliness()` returns `false`.
    node_liveliness: Option<LivelinessToken>,
    /// The node name used for the #104 primary token (from
    /// `TransportConfig::node_name` at open).  Stored so that
    /// `drop_primary_node_liveliness_if_superseded` can compare it against
    /// per-node names from W2 entity creation and decide whether to drop the
    /// primary (multi-node case) or keep it (single-node case).
    primary_node_name: heapless::String<64>,
    /// Phase 268 W2 — one NN liveliness token per distinct node name seen
    /// via entity creation, so each launch component appears as its own node
    /// in `ros2 node list`.  Bounded; held for the session lifetime (dropping
    /// a token undeclares it).
    per_node_liveliness:
        heapless::Vec<(heapless::String<64>, LivelinessToken), MAX_PER_NODE_LIVELINESS>,
    /// Phase 124.B.3 — executor wake callback. Installed by
    /// `set_wake_callback`; non-null after the runtime has wired
    /// the executor through the cffi vtable. Invoked by
    /// `drive_io` after work was observed; the runtime cb does
    /// flag-write + condvar-signal atomically so the executor
    /// wakes from `wake_cv` instead of polling on a deadline.
    wake_cb: core::sync::atomic::AtomicPtr<core::ffi::c_void>,
    wake_ctx: core::sync::atomic::AtomicPtr<core::ffi::c_void>,
    /// Issue #0292 — monotonic per-session entity id for the `<node_id>/<id>`
    /// field of an entity liveliness token. rmw_zenoh keys graph entities by
    /// `(zid, node_id, entity_id)`; a hardcoded id made every entity collide, so
    /// an action server's five entities (3 services + feedback/status pubs)
    /// deduped to ONE and the action never assembled (`ros2 action list` empty).
    /// Starts at 1 (0 is the node's own id, `PROTO_VERSION_NODE = "0/0"`).
    ///
    /// `portable_atomic::AtomicU32`, not `core`'s — riscv32imc (esp32-c3) has no
    /// atomic-CAS ISA, so `core::sync::atomic::AtomicU32` lacks `fetch_add`;
    /// portable-atomic supplies it (single-core fallback on the esp32 build,
    /// native atomics everywhere else).
    entity_counter: portable_atomic::AtomicU32,
    /// phase-381 W3 — the domain, kept so a graph query can be built later.
    /// Construction used `config.domain_id` and dropped it; enumeration needs
    /// it every call.
    domain_id: u32,
    /// phase-381 / issue 0903 — the ONE standing query, and which shape it is
    /// currently asking for.
    ///
    /// **At most one liveliness get may be in flight.** Measured against a live
    /// `rmw_zenoh_cpp` talker: with a node query standing, the entity query got
    /// `arrived=0` across 99 sweeps; skipping the node phase made the same
    /// entity query work immediately. Symmetric — once the entity query also
    /// ran, node enumeration went to zero. Two concurrent
    /// `z_liveliness_get`s do not both receive replies.
    ///
    /// Collapsing them into one `**` query was tried and is WORSE: `**`
    /// delivered two tokens and no node token at all.
    ///
    /// So they are SERIALIZED. A caller wanting the shape that is not currently
    /// in flight gets an empty answer and starts its own once this sweep
    /// finishes — which is exactly the warm-up the whole family already
    /// documents, so it costs no new contract.
    /// phase-381 W3 — the standing ENTITY-enumeration query.
    ///
    /// Separate from `graph_query` because node tokens and entity tokens are
    /// different SHAPES (9 chunks vs 13), so one wildcard cannot match both.
    /// One query serves all four entity kinds — the kind chunk is wildcarded —
    /// rather than one query per slot, which would put four sweeps on the wire
    /// to answer four questions about the same graph.
    /// Whether the standing liveliness subscriber has been declared.
    graph_cache_started: bool,
}

/// Which standing query a drain is reading — phase-381 W3.
#[derive(Copy, Clone, PartialEq, Eq)]
enum GraphQuery {
    /// `NN` tokens: the nodes themselves.
    Nodes,
    /// `MP` / `MS` / `SS` / `SC` tokens: everything a node declares.
    Entities,
}

/// Which standing query a diagnostic line came from (issue 0903).
#[cfg(feature = "std")]
fn which_label(which: GraphQuery) -> &'static str {
    match which {
        GraphQuery::Nodes => "nodes",
        GraphQuery::Entities => "entities",
    }
}

/// Does this entity belong to the named node? `None` means "any node".
///
/// The namespace arrives MANGLED on both sides — the caller's was mangled once
/// before the sweep — so this is a string compare and not a demangle per token.
fn entity_on_node(e: &Ros2LivelinessEntity<'_>, node: Option<(&str, &str)>) -> bool {
    match node {
        None => true,
        Some((name, ns_mangled)) => e.node_name == name && e.namespace == ns_mangled,
    }
}

/// phase-381 W3 — how long a standing graph query waits for replies.
///
/// Not a caller budget: `get_node_names` never blocks. This is the zenoh
/// dropper's window, i.e. how long the slot keeps accepting replies before it
/// is finished and restarted.
/// Bytes of the graph-cache snapshot a drain copies out of the C side.
///
/// Longest liveliness keyexpr this enumeration will read.
///
/// A stack buffer, so it is bounded by construction. An entry longer than this
/// is SKIPPED, never truncated — a partial keyexpr parses as a different,
/// plausible node.
const GRAPH_KEYEXPR_MAX: usize = 512;

/// Longest topic / service / type name a graph query reports.
const GRAPH_NAME_MAX: usize = 256;

/// Most distinct names one enumeration reports. A graph with more reports the
/// first this many — bounded by construction, and stated rather than silent.
const GRAPH_NAMES_MAX: usize = 64;

/// Most types reported for ONE name. Overflow reports the types that fit
/// rather than dropping the name: the contract allows a short `types_count` on
/// a partial graph, and a topic with one of its two types is still true.
const GRAPH_TYPES_MAX: usize = 8;

impl ZenohSession {
    /// Create a new shim session with the given configuration
    ///
    /// # Arguments
    ///
    /// * `config` - Transport configuration with locator and mode
    ///
    /// # Returns
    ///
    /// A new session or error if connection fails
    pub fn new(config: &TransportConfig) -> Result<Self, TransportError> {
        // Issue 0682 — peer mode rests on zenoh-pico's multicast transport and
        // scouting, and this shim is compiled WITHOUT them (a size decision, see
        // `nros_zpico_build::MULTICAST_TRANSPORT`). Refuse here, where the reason
        // is still known.
        //
        // Without this the request went all the way to `z_open`, which had no
        // multicast link to bring up and returned the same `ConnectionFailed`
        // that a wrong locator or a dead router produces — so the one error the
        // build could have explained exactly was the one it explained least.
        // `NROS_SESSION_MODE=peer` is a documented value; a documented value
        // that cannot work has to say so rather than look like a network fault.
        if matches!(config.mode, SessionMode::Peer) && !crate::zpico::ZPICO_PEER_MODE_SUPPORTED {
            // Kept under nros-log's 256-byte buffer: a message that overflows is
            // replaced by "…", and a diagnostic nobody can read is the failure
            // this guard exists to prevent.
            nros_log::log_error!(
                nros_log::get_logger("nros_rmw_zenoh"),
                "peer mode unsupported: shim built without multicast transport/scouting \
                 (issue 0682). Use client mode with a router, or rebuild with \
                 ZPICO_MULTICAST_TRANSPORT=1 at build time (issue 0711)."
            );
            return Err(TransportError::Unsupported);
        }

        // Issue 0330 — `None` and `""` both mean "caller supplied nothing";
        // the backend (not any agnostic layer) owns the default.
        let supplied_locator = normalize_locator(config.locator);

        // Build the locator string with null terminator
        // `None` here means "dial nothing" — peer mode, or a no_std client with
        // no locator supplied (multicast scouting).
        let dial: Option<&str> = match &config.mode {
            SessionMode::Client => effective_client_locator(supplied_locator),
            SessionMode::Peer => None,
        };

        let locator = match dial {
            Some(loc) => {
                // Create null-terminated locator
                let mut buf = [0u8; LOCATOR_BUFFER_SIZE];
                let bytes = loc.as_bytes();
                if bytes.len() >= buf.len() {
                    return Err(TransportError::InvalidArgument);
                }
                buf[..bytes.len()].copy_from_slice(bytes);
                #[cfg(feature = "std")]
                let len = {
                    let mut len = bytes.len();
                    append_tls_env_to_locator(loc, &mut buf, &mut len)?;
                    len
                };
                #[cfg(not(feature = "std"))]
                let len = bytes.len();
                buf[len] = 0; // Null terminator
                buf
            }
            None => [0u8; LOCATOR_BUFFER_SIZE],
        };

        // Build mode string
        let mode: &[u8] = match config.mode {
            SessionMode::Client => b"client\0",
            SessionMode::Peer => b"peer\0",
        };

        // Build null-terminated property strings on the stack
        // Each key/value is at most 64 bytes
        let mut key_bufs = [[0u8; CONFIG_PROPERTY_SIZE]; MAX_SESSION_PROPERTIES];
        let mut val_bufs = [[0u8; CONFIG_PROPERTY_SIZE]; MAX_SESSION_PROPERTIES];
        let mut c_props: [crate::zpico::zpico_property_t; MAX_SESSION_PROPERTIES] =
            unsafe { core::mem::zeroed() };

        let mut prop_count = 0usize;

        // Copy explicit properties from config.
        //
        // phase-206 W3 — both of the drops that used to live here are errors
        // now. The loop bound was `.min(MAX_SESSION_PROPERTIES)`, so a ninth
        // property vanished, and an over-long key or value `continue`d. Either
        // way the session opened and reported success while running a
        // configuration the caller never asked for — which is the whole class
        // this work item is about: a configuration line that is silently
        // dropped is indistinguishable from one that took effect, so the next
        // person debugs the transport rather than the input.
        if config.properties.len() > MAX_SESSION_PROPERTIES {
            nros_log::log_error!(
                nros_log::get_logger("nros_rmw_zenoh"),
                "session config: {} properties supplied, this build carries at most {} \
                 (raise MAX_SESSION_PROPERTIES in nros-rmw-zenoh's shim; it is a stack \
                 array, so the embedded cap is deliberately small)",
                config.properties.len(),
                MAX_SESSION_PROPERTIES
            );
            return Err(TransportError::InvalidArgument);
        }
        for i in 0..config.properties.len() {
            let (key, value) = config.properties[i];
            let key_bytes = key.as_bytes();
            let val_bytes = value.as_bytes();
            if key_bytes.len() >= CONFIG_PROPERTY_SIZE || val_bytes.len() >= CONFIG_PROPERTY_SIZE {
                nros_log::log_error!(
                    nros_log::get_logger("nros_rmw_zenoh"),
                    "session config: property '{}' does not fit this build's {}-byte \
                     key/value buffer",
                    key,
                    CONFIG_PROPERTY_SIZE
                );
                return Err(TransportError::InvalidArgument);
            }
            key_bufs[prop_count][..key_bytes.len()].copy_from_slice(key_bytes);
            key_bufs[prop_count][key_bytes.len()] = 0;
            val_bufs[prop_count][..val_bytes.len()].copy_from_slice(val_bytes);
            val_bufs[prop_count][val_bytes.len()] = 0;
            c_props[prop_count] = crate::zpico::zpico_property_t {
                key: key_bufs[prop_count].as_ptr().cast(),
                value: val_bufs[prop_count].as_ptr().cast(),
            };
            prop_count += 1;
        }

        // Read ZENOH_* env vars as defaults (explicit properties take precedence)
        #[cfg(feature = "std")]
        {
            let env_mappings: &[(&str, &str)] = &[
                ("ZENOH_MULTICAST_SCOUTING", "multicast_scouting"),
                ("ZENOH_SCOUTING_TIMEOUT", "scouting_timeout_ms"),
                ("ZENOH_LISTEN", "listen"),
                // TLS configuration
                ("ZENOH_TLS_ROOT_CA_CERTIFICATE", "root_ca_certificate"),
                (
                    "ZENOH_TLS_ROOT_CA_CERTIFICATE_BASE64",
                    "root_ca_certificate_base64",
                ),
                ("ZENOH_TLS_VERIFY_NAME_ON_CONNECT", "verify_name_on_connect"),
            ];
            for &(env_name, prop_key) in env_mappings {
                if let Ok(val) = std::env::var(env_name) {
                    let already_set = config.properties.iter().any(|(k, _)| *k == prop_key);
                    if already_set {
                        continue;
                    }
                    // phase-206 W3 — an env var the caller set and this build
                    // cannot carry is an error, not a shrug. It used to fall
                    // off the end of the array (or off the end of the buffer)
                    // in silence, which reads exactly like "the variable had
                    // no effect on this backend".
                    let key_bytes = prop_key.as_bytes();
                    let val_bytes = val.as_bytes();
                    if prop_count >= MAX_SESSION_PROPERTIES
                        || key_bytes.len() >= CONFIG_PROPERTY_SIZE
                        || val_bytes.len() >= CONFIG_PROPERTY_SIZE
                    {
                        nros_log::log_error!(
                            nros_log::get_logger("nros_rmw_zenoh"),
                            "session config: {} could not be applied — {} properties already \
                             set (max {}) or the value exceeds {} bytes",
                            env_name,
                            prop_count,
                            MAX_SESSION_PROPERTIES,
                            CONFIG_PROPERTY_SIZE
                        );
                        return Err(TransportError::InvalidArgument);
                    }
                    key_bufs[prop_count][..key_bytes.len()].copy_from_slice(key_bytes);
                    key_bufs[prop_count][key_bytes.len()] = 0;
                    val_bufs[prop_count][..val_bytes.len()].copy_from_slice(val_bytes);
                    val_bufs[prop_count][val_bytes.len()] = 0;
                    c_props[prop_count] = crate::zpico::zpico_property_t {
                        key: key_bufs[prop_count].as_ptr().cast(),
                        value: val_bufs[prop_count].as_ptr().cast(),
                    };
                    prop_count += 1;
                }
            }
        }

        // Issue 0330 — nothing to dial (peer mode, or a no_std client that
        // supplied no locator) must pass NO endpoint, never a bare `""`, which
        // zpico would install as a real, broken CONNECT endpoint.
        let locator_opt = dial.map(|_| locator.as_slice());

        let context = Context::with_config(locator_opt, mode, &c_props[..prop_count])
            .map_err(TransportError::from)?;

        // Register the reply waker callback for async service client support
        super::service::register_reply_waker(context.handle());

        // Fix #104 — declare the node liveliness token so the primary node
        // appears in `ros2 node list`.  Build the session first (token is
        // None), then immediately declare it while the session is live.
        let mut primary_node_name: heapless::String<64> = heapless::String::new();
        let _ = primary_node_name.push_str(config.node_name);
        let mut session = Self {
            context,
            node_liveliness: None,
            primary_node_name,
            per_node_liveliness: heapless::Vec::new(),
            wake_cb: core::sync::atomic::AtomicPtr::new(core::ptr::null_mut()),
            wake_ctx: core::sync::atomic::AtomicPtr::new(core::ptr::null_mut()),
            entity_counter: portable_atomic::AtomicU32::new(1),
            domain_id: config.domain_id,
            graph_cache_started: false,
        };

        if !config.node_name.is_empty() {
            // Treat empty namespace as root "/" — that is what the keyexpr
            // builder expects for a top-level node.
            let ns = if config.namespace.is_empty() {
                "/"
            } else {
                config.namespace
            };
            session.node_liveliness =
                session.declare_node_liveliness(config.domain_id, ns, config.node_name);
        }

        Ok(session)
    }

    /// Check if the session is open
    pub fn is_open(&self) -> bool {
        self.context.is_open()
    }

    /// Check if this backend requires polling
    ///
    /// For shim transport, this always returns true - manual polling is required.
    pub fn uses_polling(&self) -> bool {
        self.context.uses_polling()
    }

    /// Combined poll and keepalive operation
    ///
    /// This is the recommended way to drive the session. Call this
    /// periodically (e.g., every 10ms) from your main loop or RTIC task.
    ///
    /// # Arguments
    ///
    /// * `timeout_ms` - Maximum time to wait (0 = non-blocking)
    ///
    /// # Returns
    ///
    /// Number of events processed, or error
    pub fn spin_once(&self, timeout_ms: u32) -> Result<i32, TransportError> {
        self.context
            .spin_once(timeout_ms)
            .map_err(TransportError::from)
    }

    /// Get a reference to the underlying Context
    pub fn inner(&self) -> &Context {
        &self.context
    }

    /// Get the session's Zenoh ID
    ///
    /// The Zenoh ID uniquely identifies this session in the Zenoh network.
    /// It is used in liveliness token key expressions for ROS 2 discovery.
    pub fn zid(&self) -> Result<ZenohId, TransportError> {
        self.context.zid().map_err(TransportError::from)
    }

    /// Declare a liveliness token for ROS 2 discovery
    ///
    /// This creates a liveliness token at the given key expression,
    /// allowing ROS 2 nodes using rmw_zenoh to discover this entity.
    ///
    /// The key expression should be null-terminated.
    pub fn declare_liveliness(&self, keyexpr: &[u8]) -> Result<LivelinessToken, TransportError> {
        self.context
            .declare_liveliness(keyexpr)
            .map_err(TransportError::from)
    }

    /// Declare a node liveliness token for ROS 2 participant discovery
    ///
    /// Creates an NN liveliness token so ROS 2 tools (`ros2 node list`)
    /// can discover this node. The token is kept alive for the session lifetime.
    pub fn declare_node_liveliness(
        &self,
        domain_id: u32,
        namespace: &str,
        node_name: &str,
    ) -> Option<LivelinessToken> {
        if !self.should_declare_liveliness() {
            return None;
        }

        self.declare_entity_liveliness(|zid, _entity_id| {
            Ros2Liveliness::node_keyexpr::<256>(domain_id, zid, namespace, node_name)
        })
    }

    #[inline]
    fn should_declare_liveliness(&self) -> bool {
        // issue 0283 — liveliness is ON everywhere. It used to be disabled
        // wholesale on FreeRTOS (phase-127.B, 6866903ab) to dodge a
        // declaration that could block once a second peer joined; that
        // blocking path was the pre-0269 declare stack (slot exhaustion +
        // the short-write `_z_send_tcp`), both fixed. A ROS 2 graph that
        // no tool can see is not an acceptable steady state for hardware —
        // `ros2 node list` is how an integrator confirms a safety MCU
        // joined the system.
        //
        // Deliberate opt-out for peer-to-peer fixtures that want the wire
        // quiet: the `no-liveliness` cargo feature.
        !cfg!(feature = "no-liveliness")
    }

    /// Helper: build a liveliness keyexpr using a closure and declare it. The
    /// closure receives a freshly-allocated per-session entity id (issue #0292)
    /// for the `<node_id>/<id>` token field; entity builders embed it so each
    /// entity is graph-distinct, while the node builder ignores it (nodes use the
    /// fixed `0/0`).
    fn declare_entity_liveliness(
        &self,
        build_keyexpr: impl FnOnce(&ZenohId, u32) -> heapless::String<256>,
    ) -> Option<LivelinessToken> {
        let zid = self.context.zid().ok()?;
        let entity_id = self
            .entity_counter
            .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        let keyexpr = build_keyexpr(&zid, entity_id);

        #[cfg(feature = "std")]
        log::debug!("liveliness keyexpr: {}", keyexpr.as_str());

        let mut buf = [0u8; 257];
        let bytes = keyexpr.as_bytes();
        if bytes.len() < buf.len() {
            buf[..bytes.len()].copy_from_slice(bytes);
            buf[bytes.len()] = 0;
            self.context.declare_liveliness(&buf[..=bytes.len()]).ok()
        } else {
            None
        }
    }

    /// Phase 268 W2 — lazily declare a per-node NN liveliness token the first
    /// time an entity for `node_name` is created.
    ///
    /// Subsequent calls for the same name are no-ops (dedup).  The token is
    /// held in `per_node_liveliness` for the session lifetime; dropping it
    /// would undeclare the node in `ros2 node list`.
    // Issue #143 — the #129-era Zephyr gate here is LIFTED: the "deadlock"
    // this declare hit was the #139 socket-timeout starvation (5 s recv
    // window serializing every tx), fixed at the root. Per-node tokens are
    // back on every platform, restoring per-component `ros2 node list`
    // fidelity on Zephyr.
    /// phase-381 W3 — drain a standing liveliness query, parsed.
    ///
    /// The one place the graph is read. Ten vtable slots differ only in which
    /// entities they keep and what they report, so the drain, the parse, the
    /// skip-never-truncate rule and the warm-up policy live here rather than in
    /// ten copies — the second spelling is what this codebase keeps paying for.
    ///
    /// `f` returns `false` to stop early, which is the visitor contract's way
    /// for a caller with a bound to stop paying for the rest.
    ///
    /// **Never blocks.** The slot contract forbids it and takes no timeout, so
    /// this reports what has ALREADY arrived: it drains the standing query and
    /// starts the next once this one has finished. A first call therefore sees
    /// a partial graph and later calls see more — the warm-up Design note 3
    /// describes. Blocking here would stall the executor's only thread inside
    /// an introspection call.
    fn for_each_entity(
        &mut self,
        which: GraphQuery,
        f: &mut dyn FnMut(&Ros2LivelinessEntity<'_>) -> bool,
    ) -> Result<(), TransportError> {
        // Used only to label the diagnostics below; the cache is shape-agnostic.
        let _ = which;
        // One standing cache serves every caller and every shape (issue 0903).
        // `which` no longer selects a query — a node token and an entity token
        // arrive through the same subscriber, and callers already filter by
        // `EntityKind`. It survives only to label the diagnostics.
        self.ensure_graph_cache()?;

        let mut dropped: u32 = 0;
        let count =
            unsafe { zpico_sys::zpico_graph_entry_count(self.context.handle(), &mut dropped) };
        if count < 0 {
            return Err(TransportError::Unsupported);
        }
        let count = count as u32;

        #[cfg(feature = "std")]
        if std::env::var_os("NROS_GRAPH_DUMP").is_some() {
            log::warn!(
                "GRAPH_COUNTS[{:?}] cached={} dropped={}",
                which_label(which),
                count,
                dropped
            );
        }

        for index in 0..count {
            let mut key = [0i8; GRAPH_KEYEXPR_MAX];
            let n = unsafe {
                zpico_sys::zpico_graph_entry_at(
                    self.context.handle(),
                    index,
                    key.as_mut_ptr() as *mut core::ffi::c_char,
                    key.len(),
                )
            };
            if n <= 0 {
                // ZPICO_ERR_BUFFER for an entry longer than this stack buffer.
                // Skipped rather than truncated: a partial keyexpr names a
                // different, plausible entity.
                continue;
            }
            // SAFETY: the C side wrote `n` bytes plus a NUL.
            let bytes =
                unsafe { core::slice::from_raw_parts(key.as_ptr() as *const u8, n as usize) };
            let Ok(text) = core::str::from_utf8(bytes) else {
                continue;
            };
            // Opt-in wire diagnostic (issue 0903). Committed rather than
            // patched in when needed: the first time this was wanted it lived
            // in the distrobox tree and a re-sync destroyed it, so the next
            // person re-derived it. `NROS_GRAPH_DUMP=1` prints what the wire
            // ACTUALLY carries and what the parser made of it — the two
            // questions any "the graph is empty" report has to separate.
            #[cfg(feature = "std")]
            let dump = std::env::var_os("NROS_GRAPH_DUMP").is_some();
            #[cfg(not(feature = "std"))]
            let dump = false;
            if dump {
                #[cfg(feature = "std")]
                log::warn!("GRAPH_RAW[{:?}] {}", which_label(which), text);
            }
            // `parse` refuses anything it does not recognise, so an unknown
            // shape is dropped here rather than reported as a plausible entity.
            let Some(entity) = Ros2Liveliness::parse(text) else {
                if dump {
                    #[cfg(feature = "std")]
                    log::warn!("GRAPH_REFUSED[{:?}] {}", which_label(which), text);
                }
                continue;
            };
            if !f(&entity) {
                break;
            }
        }
        Ok(())
    }

    /// Declare the standing liveliness subscriber, once.
    ///
    /// `**` covers both token shapes — a node token is 9 chunks and an entity
    /// token 13 — which a single-`*` pattern cannot. That width failed for the
    /// GET form, where the router tags replies per interest; it is the natural
    /// fit for a subscriber, which simply matches.
    fn ensure_graph_cache(&mut self) -> Result<(), TransportError> {
        if self.graph_cache_started {
            return Ok(());
        }
        let key = Ros2Liveliness::graph_keyexpr_wildcard::<256>(self.domain_id);
        #[cfg(feature = "std")]
        if std::env::var_os("NROS_GRAPH_DUMP").is_some() {
            log::warn!("GRAPH_CACHE_START {}", key.as_str());
        }
        let mut buf = [0u8; 257];
        let bytes = key.as_bytes();
        if bytes.len() >= buf.len() {
            return Err(TransportError::InvalidArgument);
        }
        buf[..bytes.len()].copy_from_slice(bytes);
        buf[bytes.len()] = 0;
        let rc = unsafe {
            zpico_sys::zpico_graph_cache_start(
                self.context.handle(),
                buf.as_ptr() as *const core::ffi::c_char,
            )
        };
        if rc < 0 {
            return Err(TransportError::Unsupported);
        }
        self.graph_cache_started = true;
        Ok(())
    }

    /// Retire a FINISHED sweep so the next public call starts a fresh one.
    ///
    /// Called once per public entry point, after every drain that call needs.
    /// Refresh the graph view before a public entry point reads it.
    ///
    /// With the sweep gone there is nothing to retire: the subscriber keeps the
    /// cache current on its own, so this only guarantees the subscriber EXISTS
    /// before the first read. Kept as a named step because the call sites read
    /// better for it, and because the ordering it used to protect — refresh once
    /// per entry point, never inside a drain — is still the rule.
    fn refresh_query(&mut self, _which: GraphQuery) {
        let _ = self.ensure_graph_cache();
    }

    /// Count the entities of one kind on a topic — phase-381 W3.
    ///
    /// `count_publishers` / `count_subscribers` differ only by kind.
    fn count_entities_on_topic(
        &mut self,
        kind: EntityKind,
        topic_name: &str,
    ) -> Result<usize, TransportError> {
        let mangled = Ros2Liveliness::mangle_topic_name_pub::<256>(topic_name);
        let mut n = 0usize;
        self.for_each_entity(GraphQuery::Entities, &mut |e| {
            if e.kind == kind && e.topic == Some(mangled.as_str()) {
                n += 1;
            }
            true
        })?;
        Ok(n)
    }

    /// phase-381 W3 — group entities by NAME and report each name once.
    ///
    /// The visitor contract is "a name and the types on it", but the wire is one
    /// token per entity, so the same topic arrives once per publisher. Grouping
    /// needs dedup, and there is no allocator here.
    ///
    /// So: one pass to find the next name not yet reported, a second to collect
    /// that name's types. Quadratic in the number of distinct names, which is
    /// the right trade at this size — the alternative is a heap map, and the
    /// bound is the reply buffer, not the graph. Everything is BORROWED; the
    /// only owned storage is a fixed array of type pointers.
    ///
    /// A name whose type list overflows `GRAPH_TYPES_MAX` is reported with the
    /// types that fit rather than dropped: the contract says `types_count` may
    /// be short on a partial graph, and a topic reported with one of its two
    /// types is still true, where dropping the topic is not.
    fn names_and_types(
        &mut self,
        kinds: &[EntityKind],
        visit: &mut dyn FnMut(&str, &[&str]) -> bool,
    ) -> Result<(), TransportError> {
        self.names_and_types_filtered(kinds, None, visit)
    }

    /// phase-381 W3 — the grouping, optionally narrowed to ONE node.
    ///
    /// `node` is `(name, MANGLED namespace)`. The whole-graph forms pass `None`
    /// and the `*_by_node` forms pass `Some`, so the dedup exists once: four
    /// per-node slots plus two whole-graph ones would otherwise be six copies
    /// of one two-pass algorithm.
    fn names_and_types_filtered(
        &mut self,
        kinds: &[EntityKind],
        node: Option<(&str, &str)>,
        visit: &mut dyn FnMut(&str, &[&str]) -> bool,
    ) -> Result<(), TransportError> {
        // Names already reported, so the outer pass makes progress. Bounded by
        // construction; a graph with more distinct names than this reports the
        // first `GRAPH_NAMES_MAX` of them.
        let mut seen: heapless::Vec<heapless::String<GRAPH_NAME_MAX>, GRAPH_NAMES_MAX> =
            heapless::Vec::new();

        loop {
            // Pass 1 — the next name we have not reported yet.
            let mut next: Option<heapless::String<GRAPH_NAME_MAX>> = None;
            self.for_each_entity(GraphQuery::Entities, &mut |e| {
                if !kinds.contains(&e.kind) || !entity_on_node(e, node) {
                    return true;
                }
                let Some(topic) = e.topic else { return true };
                let Ok(name) = heapless::String::try_from(topic) else {
                    return true;
                };
                if seen.contains(&name) {
                    return true;
                }
                next = Some(name);
                false // stop: one new name per outer iteration
            })?;

            let Some(name) = next else { break };
            if seen.push(name.clone()).is_err() {
                // Name table full — stop rather than loop forever on a name we
                // cannot record as seen.
                break;
            }

            // Pass 2 — that name's types. Pointers into the parse buffers are
            // not valid past `for_each_entity`, so the strings are COPIED here;
            // this is the one place that costs storage, and it is bounded.
            let mut types: heapless::Vec<heapless::String<GRAPH_NAME_MAX>, GRAPH_TYPES_MAX> =
                heapless::Vec::new();
            self.for_each_entity(GraphQuery::Entities, &mut |e| {
                if !kinds.contains(&e.kind)
                    || e.topic != Some(name.as_str())
                    || !entity_on_node(e, node)
                {
                    return true;
                }
                if let Some(t) = e.type_name
                    && let Ok(ty) = heapless::String::try_from(t)
                    && !types.contains(&ty)
                {
                    let _ = types.push(ty);
                }
                true
            })?;

            let demangled = Ros2Liveliness::demangle_topic_name::<GRAPH_NAME_MAX>(name.as_str());
            let refs: heapless::Vec<&str, GRAPH_TYPES_MAX> =
                types.iter().map(|t| t.as_str()).collect();
            if !visit(demangled.as_str(), refs.as_slice()) {
                break;
            }
        }
        // One refresh for the WHOLE grouping, after every pass it needed.
        self.refresh_query(GraphQuery::Entities);
        Ok(())
    }

    fn ensure_node_liveliness(&mut self, domain_id: u32, namespace: &str, node_name: &str) {
        if node_name.is_empty() {
            return;
        }
        // Dedup: already declared for this name.
        if self
            .per_node_liveliness
            .iter()
            .any(|(n, _)| n.as_str() == node_name)
        {
            return;
        }
        // Treat empty namespace as root "/" — same as the #104 primary path.
        let ns = if namespace.is_empty() { "/" } else { namespace };
        if let Some(tok) = self.declare_node_liveliness(domain_id, ns, node_name) {
            let mut key: heapless::String<64> = heapless::String::new();
            let _ = key.push_str(node_name);
            let _ = self.per_node_liveliness.push((key, tok)); // silent overflow past MAX
            // Gate: a per-node token with a DIFFERENT name supersedes the
            // primary `/node` phantom — drop it so multi-node launches show
            // only their components, not a spurious "node" entry.
            self.drop_primary_node_liveliness_if_superseded(node_name);
        }
    }

    /// Phase 268 W2 — drop the #104 primary node liveliness token when a
    /// per-node token with a DIFFERENT name has been declared.
    ///
    /// **Single-node case**: the one component's name matches the primary name
    /// (e.g. primary `"talker"` + per-node `"talker"`) → keep primary.
    ///
    /// **Multi-node case**: primary generic name (e.g. `"node"`) differs from
    /// the per-node name (e.g. `"talker"`) → drop primary → `ros2 node list`
    /// shows only `/talker` + `/listener`, not a spurious `/node`.
    fn drop_primary_node_liveliness_if_superseded(&mut self, new_name: &str) {
        if self.node_liveliness.is_some()
            && !self.primary_node_name.is_empty()
            && self.primary_node_name.as_str() != new_name
        {
            self.node_liveliness = None; // Drop → immediately undeclares the NN token.
        }
    }
}

/// issue 0829 — what THIS backend resolves `rmw_qos_profile_system_default`
/// to, applied at every create entry via `QoSProfile::resolve_system_default`.
///
/// The three policies mirror `rmw_zenoh_cpp`'s `QoS::QoS()`
/// (`RMW_ZENOH_DEFAULT_RELIABILITY` RELIABLE, `_DURABILITY` VOLATILE,
/// `_HISTORY` KEEP_LAST), which is also what `rmw_cyclonedds_cpp` folds the
/// same sentinels to — the two reference RMWs agree on everything except the
/// depth.
///
/// The depth deliberately does NOT mirror upstream's 42. That number is
/// `rmw_zenoh_cpp`'s figure for its own buffers; ours is
/// `SUBSCRIBER_RING_DEPTH`, the statically-allocated per-subscriber ring the
/// shim actually enforces (`shim/subscriber.rs:51-57`, overflow drops at
/// `:1538`; build-time `ZPICO_SUBSCRIBER_RING_DEPTH`, default 4). Advertising
/// a depth we cannot honour is the lie issue 0829 is about, and the depth
/// reaches a peer only through the liveliness-token keyexpr, where history and
/// depth are not RxO policies and so do not gate matching.
const ZENOH_SYSTEM_DEFAULTS: nros_rmw::QoSSystemDefaults = nros_rmw::QoSSystemDefaults {
    reliability: nros_rmw::QoSReliabilityPolicy::Reliable,
    durability: nros_rmw::QoSDurabilityPolicy::Volatile,
    history: nros_rmw::QoSHistoryPolicy::KeepLast,
    depth: crate::config::SUBSCRIBER_RING_DEPTH as u32,
};

impl Session for ZenohSession {
    type Error = TransportError;
    type PublisherHandle = ZenohPublisher;
    type SubscriptionHandle = ZenohSubscriber;
    type ServiceHandle = ZenohServiceServer;
    type ClientHandle = ZenohServiceClient;

    fn create_publisher(
        &mut self,
        topic: &TopicInfo,
        qos: QoSProfile,
    ) -> Result<Self::PublisherHandle, Self::Error> {
        // issue 0829 — resolve the SYSTEM_DEFAULT sentinel FIRST, before
        // anything is derived from the profile. The liveliness-token keyexpr
        // below serialises this QoS in upstream's numbering for a ROS
        // `rmw_zenoh_cpp` peer to parse out of the graph; upstream resolves in
        // `best_available_qos` before the token exists, so its tokens never
        // carry a sentinel and neither may ours.
        let qos = qos.resolve_system_default(&ZENOH_SYSTEM_DEFAULTS);
        let mut publisher = ZenohPublisher::new(&self.context, topic, None, &qos)?;
        // Phase 268 W2 — ensure a per-node NN token for this publisher's node.
        if let Some(node_name) = topic.node_name {
            self.ensure_node_liveliness(topic.domain_id, topic.namespace, node_name);
        }
        let liveliness_token = self
            .should_declare_liveliness()
            .then_some(())
            .and_then(|_| {
                topic.node_name.and_then(|node_name| {
                    self.declare_entity_liveliness(|zid, entity_id| {
                        Ros2Liveliness::publisher_keyexpr::<256>(
                            topic.domain_id,
                            zid,
                            entity_id,
                            topic.namespace,
                            node_name,
                            topic,
                            &qos,
                        )
                    })
                })
            });
        publisher.set_liveliness(liveliness_token);
        Ok(publisher)
    }

    fn create_subscription(
        &mut self,
        topic: &TopicInfo,
        qos: QoSProfile,
    ) -> Result<Self::SubscriptionHandle, Self::Error> {
        // issue 0829 — see `create_publisher`: resolve before the keyexpr.
        let qos = qos.resolve_system_default(&ZENOH_SYSTEM_DEFAULTS);
        let mut subscriber = ZenohSubscriber::new(&self.context, topic, None, &qos)?;
        // Phase 268 W2 — ensure a per-node NN token for this subscriber's node.
        if let Some(node_name) = topic.node_name {
            self.ensure_node_liveliness(topic.domain_id, topic.namespace, node_name);
        }
        let liveliness_token = self
            .should_declare_liveliness()
            .then_some(())
            .and_then(|_| {
                topic.node_name.and_then(|node_name| {
                    self.declare_entity_liveliness(|zid, entity_id| {
                        Ros2Liveliness::subscriber_keyexpr::<256>(
                            topic.domain_id,
                            zid,
                            entity_id,
                            topic.namespace,
                            node_name,
                            topic,
                            &qos,
                        )
                    })
                })
            });
        subscriber.set_liveliness(liveliness_token);
        Ok(subscriber)
    }

    fn create_service(
        &mut self,
        service: &ServiceInfo,
        qos: QoSProfile,
    ) -> Result<Self::ServiceHandle, Self::Error> {
        // TODO(193.1b): zenoh-pico services have no endpoint-level QoS
        // slot (the `None` below is the liveliness token, not QoS) — the
        // requested service QoS cannot be applied to the queryable yet.
        // Thread it through once zenoh-pico exposes per-endpoint QoS.
        let _ = qos;
        let mut server = ZenohServiceServer::new(&self.context, service, None)?;
        // Phase 268 W2 — ensure a per-node NN token for this server's node.
        if let Some(node_name) = service.node_name {
            self.ensure_node_liveliness(service.domain_id, service.namespace, node_name);
        }
        let liveliness_token = self
            .should_declare_liveliness()
            .then_some(())
            .and_then(|_| {
                service.node_name.and_then(|node_name| {
                    self.declare_entity_liveliness(|zid, entity_id| {
                        Ros2Liveliness::service_server_keyexpr::<256>(
                            service.domain_id,
                            zid,
                            entity_id,
                            service.namespace,
                            node_name,
                            service,
                            &QoSProfile::services_default(),
                        )
                    })
                })
            });
        server.set_liveliness(liveliness_token);
        Ok(server)
    }

    fn create_client(
        &mut self,
        service: &ServiceInfo,
        qos: QoSProfile,
    ) -> Result<Self::ClientHandle, Self::Error> {
        // TODO(193.1b): zenoh-pico services have no endpoint-level QoS
        // slot — the requested service QoS cannot be applied to the
        // querier yet. Thread it once zenoh-pico exposes per-endpoint QoS.
        let _ = qos;
        // Phase 268 W2 — ensure a per-node NN token for this client's node.
        if let Some(node_name) = service.node_name {
            self.ensure_node_liveliness(service.domain_id, service.namespace, node_name);
        }
        let liveliness_token = self
            .should_declare_liveliness()
            .then_some(())
            .and_then(|_| {
                service.node_name.and_then(|node_name| {
                    self.declare_entity_liveliness(|zid, entity_id| {
                        Ros2Liveliness::service_client_keyexpr::<256>(
                            service.domain_id,
                            zid,
                            entity_id,
                            service.namespace,
                            node_name,
                            service,
                            &QoSProfile::services_default(),
                        )
                    })
                })
            });
        ZenohServiceClient::new(&self.context, service, liveliness_token)
    }

    fn close(&mut self) -> Result<(), Self::Error> {
        // Context is closed on drop
        Ok(())
    }

    fn drive_io(&mut self, timeout_ms: i32) -> Result<(), Self::Error> {
        // Idle deadline expiry is the transport's normal quiet state, not an
        // I/O failure. The polled shim branches (ThreadX, single-threaded
        // select) report "no inbound data before the timeout" as
        // ZPICO_ERR_TIMEOUT, while the condvar/semaphore branches return 0 —
        // without this mapping the executor's session-death counter (issue
        // 0355) grows on every quiet tick, and the C spin wrappers bail at
        // SPIN_ERROR_TOLERANCE: a quiet ThreadX C talker died after
        // 16×period (issue 0387; the Rust entry loops never read the
        // counter, which is why only the C arms fell over).
        let res = match self.spin_once(timeout_ms as u32) {
            Err(TransportError::Timeout) => Ok(0),
            r => r,
        };
        // issue 0589 — was a bare `std::eprintln!("DBG …")`, which is a
        // leftover debug print on the executor's hot path AND fatal on Zephyr
        // native_sim (std stdio recurses in `zvfs_write` until the stack is
        // gone). `trace` is the level a per-spin line belongs at; the sink
        // drops it unless someone asked.
        if let Err(ref e) = res {
            nros_log::nros_trace!(
                nros_log::get_logger("nros_rmw_zenoh"),
                "drive_io error: {e:?}"
            );
        }
        // Phase 124.B.3 — when zenoh's spin_once observed any work
        // (n > 0), call the runtime-supplied wake callback so the
        // executor's `wake_cv` is signalled (flag-write +
        // condvar-signal happen atomically inside the cb).
        // Best-effort: if the cb hasn't been installed yet the
        // executor still drains via its deadline-bound cv-wait.
        if matches!(res, Ok(n) if n > 0) {
            let cb = self.wake_cb.load(core::sync::atomic::Ordering::Acquire);
            if !cb.is_null() {
                let ctx = self.wake_ctx.load(core::sync::atomic::Ordering::Acquire);
                // SAFETY: cb was installed via `set_wake_callback`
                // and points at a runtime-owned function; ctx
                // points at WakeCtx allocated for the executor's
                // lifetime.
                let f: unsafe extern "C" fn(*mut core::ffi::c_void) =
                    unsafe { core::mem::transmute(cb) };
                unsafe { f(ctx) };
            }
        }
        res.map(|_| ())
    }

    unsafe fn set_wake_callback(
        &mut self,
        cb: Option<unsafe extern "C" fn(ctx: *mut core::ffi::c_void)>,
        ctx: *mut core::ffi::c_void,
    ) {
        let cb_ptr = cb
            .map(|f| f as *mut core::ffi::c_void)
            .unwrap_or(core::ptr::null_mut());
        self.wake_cb
            .store(cb_ptr, core::sync::atomic::Ordering::Release);
        self.wake_ctx
            .store(ctx, core::sync::atomic::Ordering::Release);
        // Issue #0317 — also mirror into the process-global so the async read-task
        // arrival hook (`subscriber_notify_callback`) can fire the wake-cb on the
        // multi-threaded backend, where `drive_io`'s poll path never sees the work.
        super::set_runtime_wake_cb(cb, ctx);
    }

    /// Phase 110.0 — bound the executor's `drive_io` wait against
    /// zenoh-pico's transport keepalive interval.
    ///
    /// zenoh-pico does not expose its internal "next keepalive
    /// timestamp" through FFI, so the shim returns a conservative
    /// upper-bound: `Z_TRANSPORT_LEASE / Z_TRANSPORT_LEASE_EXPIRE_FACTOR`.
    /// With the upstream defaults (lease = 10 000 ms, factor = 3) that
    /// caps wake-late to ~3.3 s on a quiet link — the runtime never
    /// blocks longer than one keepalive interval before returning
    /// control to the executor. Tracking the precise per-call
    /// timestamp would need a hook in `_z_send_keep_alive`; out of
    /// scope for the v1 surface.
    fn next_deadline_ms(&self) -> Option<u32> {
        // Z_TRANSPORT_LEASE = 10000 ms, Z_TRANSPORT_LEASE_EXPIRE_FACTOR = 3
        const ZENOH_KEEPALIVE_INTERVAL_MS: u32 = 10_000 / 3;
        Some(ZENOH_KEEPALIVE_INTERVAL_MS)
    }

    fn ping_session(&mut self, _timeout_ms: i32) -> Result<(), Self::Error> {
        // Phase 124.F.2 — zenoh-pico's closest match to a true ping
        // is `zp_send_keep_alive`. Fire one frame; success means the
        // transport accepted it (TCP / serial / shm send returned OK,
        // i.e. the link is still alive from the local side). Failure
        // surfaces as `Timeout` per the 124.F.1 semantics so callers
        // can tear down + re-open the session on a dead link.
        //
        // The `timeout_ms` argument is ignored — the underlying call
        // is synchronous and non-blocking. We don't honour the budget
        // because the call returns within microseconds either way.
        // True round-trip ping would need a `z_send_ping` API that
        // zenoh-pico hasn't yet exposed; deferred to a follow-up
        // when upstream lands one.
        let rc = unsafe { zpico_sys::zpico_send_keep_alive(self.context.handle()) };
        if rc == 0 {
            Ok(())
        } else {
            Err(TransportError::Timeout)
        }
    }

    /// phase-381 W3 — enumerate nodes from the standing liveliness query.
    ///
    /// Thin over `for_each_entity` (private); the drain, the parse and the
    /// warm-up policy live there because ten slots share them.
    fn get_node_names(
        &mut self,
        visit: &mut dyn FnMut(&str, &str, Option<&str>) -> bool,
    ) -> Result<(), Self::Error> {
        let r = self.for_each_entity(GraphQuery::Nodes, &mut |e| {
            if e.kind != EntityKind::Node {
                return true;
            }
            let ns = Ros2Liveliness::demangle_topic_name::<128>(e.namespace);
            // `enclave` is None: the token carries one, but ours is always the
            // root, and reporting a value we do not track would be worse than
            // saying we do not.
            visit(e.node_name, ns.as_str(), None)
        });
        self.refresh_query(GraphQuery::Nodes);
        r
    }

    fn count_publishers(&mut self, topic_name: &str) -> Result<usize, Self::Error> {
        self.count_entities_on_topic(EntityKind::Publisher, topic_name)
    }

    fn count_subscribers(&mut self, topic_name: &str) -> Result<usize, Self::Error> {
        self.count_entities_on_topic(EntityKind::Subscriber, topic_name)
    }

    fn get_topic_names_and_types(
        &mut self,
        visit: &mut dyn FnMut(&str, &[&str]) -> bool,
    ) -> Result<(), Self::Error> {
        self.names_and_types(&[EntityKind::Publisher, EntityKind::Subscriber], visit)
    }

    fn get_service_names_and_types(
        &mut self,
        visit: &mut dyn FnMut(&str, &[&str]) -> bool,
    ) -> Result<(), Self::Error> {
        self.names_and_types(
            &[EntityKind::ServiceServer, EntityKind::ServiceClient],
            visit,
        )
    }

    fn get_names_and_types_by_node(
        &mut self,
        kind: nros_rmw::GraphEntityKind,
        node_name: &str,
        node_namespace: &str,
        visit: &mut dyn FnMut(&str, &[&str]) -> bool,
    ) -> Result<(), Self::Error> {
        let want = match kind {
            nros_rmw::GraphEntityKind::Publisher => EntityKind::Publisher,
            nros_rmw::GraphEntityKind::Subscriber => EntityKind::Subscriber,
            nros_rmw::GraphEntityKind::Service => EntityKind::ServiceServer,
            nros_rmw::GraphEntityKind::Client => EntityKind::ServiceClient,
        };
        // The wire carries the namespace MANGLED (`/demo` -> `%demo`), so the
        // caller's ROS name is mangled once here rather than demangling every
        // token — the comparison happens per entity and the mangle does not.
        let ns_mangled = Ros2Liveliness::mangle_topic_name_pub::<GRAPH_NAME_MAX>(node_namespace);
        self.names_and_types_filtered(&[want], Some((node_name, ns_mangled.as_str())), visit)
    }

    fn get_endpoint_info_by_topic(
        &mut self,
        publishers: bool,
        topic_name: &str,
        visit: &mut dyn FnMut(&nros_rmw::GraphEndpointInfo<'_>) -> bool,
    ) -> Result<(), Self::Error> {
        let want = if publishers {
            EntityKind::Publisher
        } else {
            EntityKind::Subscriber
        };
        let mangled = Ros2Liveliness::mangle_topic_name_pub::<GRAPH_NAME_MAX>(topic_name);
        let r = self.for_each_entity(GraphQuery::Entities, &mut |e| {
            if e.kind != want || e.topic != Some(mangled.as_str()) {
                return true;
            }
            let ns = Ros2Liveliness::demangle_topic_name::<GRAPH_NAME_MAX>(e.namespace);
            let info = nros_rmw::GraphEndpointInfo {
                node_name: e.node_name,
                node_namespace: ns.as_str(),
                topic_type: e.type_name.unwrap_or(""),
                is_publisher: publishers,
                // The liveliness token carries no GID — it identifies the
                // entity by keyexpr, not by a 24-byte id. All-zero is the
                // ABI's "this backend has none", which is honest; synthesising
                // one from the zid would invent an identity a peer cannot match.
                endpoint_gid: [0u8; 24],
            };
            visit(&info)
        });
        self.refresh_query(GraphQuery::Entities);
        r
    }

    fn supported_qos_policies(&self) -> nros_rmw::QoSPolicyMask {
        // Phase 108.B/C — zenoh-pico's wire protocol has no native
        // DDS QoS, so the shim emulates everything:
        // - Reliability maps to zenoh congestion-control (CORE).
        // - Durability VOLATILE / History / Depth honoured at the
        //   subscriber buffer level (CORE).
        // - DEADLINE: clock-based check at take_serialized (sub) /
        //   publish_raw (pub). 108.C.zenoh.2.
        // - LIFESPAN: subscriber filters samples whose attachment
        //   timestamp is older than `now - lifespan_ms`. 108.C.zenoh.3.
        // - LIVELINESS_AUTOMATIC: zenoh runtime declares the token
        //   automatically when the publisher is created
        //   (`Ros2Liveliness::publisher_keyexpr`); subscribers track
        //   alive-state via a periodic poll of the wildcard liveliness
        //   keyexpr. Per-publisher count surfaced via
        //   `zpico_liveliness_get_count` (108.C.zenoh.4-followup).
        // - LIVELINESS_MANUAL_BY_TOPIC / MANUAL_BY_NODE: shim-side
        //   keepalive timer. `Publisher::assert_liveliness()`
        //   refreshes the lease; `publish_raw` checks for expiry and
        //   fires `LivelinessLost` rate-limited to ≤ 1 per lease.
        //   (108.C.zenoh.4-followup).
        // - LIVELINESS_LEASE: caller-supplied lease duration honoured
        //   for all liveliness kinds.
        use nros_rmw::QoSPolicyMask;
        QoSPolicyMask::CORE
            | QoSPolicyMask::DEADLINE
            | QoSPolicyMask::LIFESPAN
            | QoSPolicyMask::LIVELINESS_AUTOMATIC
            | QoSPolicyMask::LIVELINESS_MANUAL_BY_TOPIC
            | QoSPolicyMask::LIVELINESS_MANUAL_BY_NODE
            | QoSPolicyMask::LIVELINESS_LEASE
    }
}
