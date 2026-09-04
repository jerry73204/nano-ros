//! Transport abstraction traits.
//!
//! Defines the backend-agnostic interface that transport implementations
//! (zenoh-pico, XRCE-DDS) must satisfy. The core trait hierarchy is:
//!
//! - [`Session`] — connection lifecycle and handle creation
//! - [`Publisher`] / [`Subscription`] — pub/sub data transport
//! - [`ServiceTrait`] / [`ClientTrait`] — request/reply
//! - [`Rmw`] — top-level factory that creates sessions

use nros_core::{Deserialize, RosMessage, RosService, Serialize};

/// Topic information for pub/sub
#[derive(Debug, Clone)]
pub struct TopicInfo<'a> {
    /// Topic name (e.g., "/chatter")
    pub name: &'a str,
    /// ROS type name (e.g., "std_msgs::msg::dds_::String_")
    pub type_name: &'a str,
    /// Type hash for compatibility checking
    pub type_hash: &'a str,
    /// Domain ID (default: 0)
    pub domain_id: u32,
    /// Node name for liveliness token generation.
    /// `None` means no node association — no liveliness token will be declared.
    pub node_name: Option<&'a str>,
    /// Node namespace for liveliness token generation (default: "/").
    /// In ROS 2, "/" is the root namespace and the standard default.
    pub namespace: &'a str,
    /// Phase 231 (RFC-0038) — receive-buffer size hint, bytes. The executor sets
    /// this from the subscription's `RX_BUF`; a backend may use it to route the
    /// subscription to a size-class receive buffer (zenoh-pico: small vs large).
    /// `0` = unset (backend picks its default). Ignored by backends that don't
    /// size-class their receive storage.
    pub rx_buffer_hint: usize,
    /// phase-279 (#145) — publisher-side "express" hint: this topic's samples
    /// bypass transport tx batching (zenoh: the wire EXPRESS flag; a batching
    /// zenoh-pico session sends them immediately instead of queueing them for
    /// the next flush). For control-tier / latency-sensitive topics. Ignored by
    /// subscriptions and by backends without a batching concept.
    pub tx_express: bool,
}

impl<'a> TopicInfo<'a> {
    /// Create new topic info
    pub const fn new(name: &'a str, type_name: &'a str, type_hash: &'a str) -> Self {
        Self {
            name,
            type_name,
            type_hash,
            domain_id: 0,
            node_name: None,
            namespace: "/",
            rx_buffer_hint: 0,
            tx_express: false,
        }
    }

    /// Create topic info with custom domain ID
    pub const fn with_domain(mut self, domain_id: u32) -> Self {
        self.domain_id = domain_id;
        self
    }

    /// Phase 231 (RFC-0038) — set the receive-buffer size hint (bytes) used by
    /// size-classing backends (zenoh-pico) to route the subscription's receive
    /// buffer to the small or large class.
    pub const fn with_rx_buffer_hint(mut self, hint: usize) -> Self {
        self.rx_buffer_hint = hint;
        self
    }

    /// phase-279 (#145) — mark this publisher's samples express: they bypass
    /// transport tx batching (sent immediately even when `ZPICO_TX_BATCH` is
    /// on). For control-tier / latency-sensitive topics.
    pub const fn with_tx_express(mut self, express: bool) -> Self {
        self.tx_express = express;
        self
    }

    /// Set the node name for liveliness token generation
    pub const fn with_node_name(mut self, node_name: &'a str) -> Self {
        self.node_name = Some(node_name);
        self
    }

    /// Set the node namespace for liveliness token generation
    pub const fn with_namespace(mut self, namespace: &'a str) -> Self {
        self.namespace = namespace;
        self
    }
}

/// Service information for service client/server
#[derive(Debug, Clone)]
pub struct ServiceInfo<'a> {
    /// Service name (e.g., "/add_two_ints")
    pub name: &'a str,
    /// ROS service type name (e.g., "example_interfaces::srv::dds_::AddTwoInts_")
    pub type_name: &'a str,
    /// Type hash for compatibility checking
    pub type_hash: &'a str,
    /// Domain ID (default: 0)
    pub domain_id: u32,
    /// Node name for liveliness token generation.
    /// `None` means no node association — no liveliness token will be declared.
    pub node_name: Option<&'a str>,
    /// Node namespace for liveliness token generation (default: "/").
    /// In ROS 2, "/" is the root namespace and the standard default.
    pub namespace: &'a str,
}

/// Action information for action client/server
///
/// Actions in ROS 2 use 5 communication channels:
/// - `send_goal` service: `<action_name>/_action/send_goal`
/// - `cancel_goal` service: `<action_name>/_action/cancel_goal`
/// - `get_result` service: `<action_name>/_action/get_result`
/// - `feedback` topic: `<action_name>/_action/feedback`
/// - `status` topic: `<action_name>/_action/status`
#[derive(Debug, Clone)]
pub struct ActionInfo<'a> {
    /// Action name (e.g., "/fibonacci")
    pub name: &'a str,
    /// ROS action type name (e.g., "example_interfaces::action::dds_::Fibonacci_")
    pub type_name: &'a str,
    /// Type hash for compatibility checking
    pub type_hash: &'a str,
    /// Domain ID (default: 0)
    pub domain_id: u32,
}

impl<'a> ActionInfo<'a> {
    /// Create new action info
    pub const fn new(name: &'a str, type_name: &'a str, type_hash: &'a str) -> Self {
        Self {
            name,
            type_name,
            type_hash,
            domain_id: 0,
        }
    }

    /// Create action info with custom domain ID
    pub const fn with_domain(mut self, domain_id: u32) -> Self {
        self.domain_id = domain_id;
        self
    }

    /// Generate the send_goal service name
    /// Returns: `<action>/_action/send_goal`
    pub fn send_goal_key<const N: usize>(&self) -> heapless::String<N> {
        self.sub_name::<N>("send_goal")
    }

    /// Generate the cancel_goal service name
    /// Returns: `<action>/_action/cancel_goal`
    pub fn cancel_goal_key<const N: usize>(&self) -> heapless::String<N> {
        self.sub_name::<N>("cancel_goal")
    }

    /// Generate the get_result service name
    /// Returns: `<action>/_action/get_result`
    pub fn get_result_key<const N: usize>(&self) -> heapless::String<N> {
        self.sub_name::<N>("get_result")
    }

    /// Generate the feedback topic name
    /// Returns: `<action>/_action/feedback`
    pub fn feedback_key<const N: usize>(&self) -> heapless::String<N> {
        self.sub_name::<N>("feedback")
    }

    /// Generate the status topic name
    /// Returns: `<action>/_action/status`
    pub fn status_key<const N: usize>(&self) -> heapless::String<N> {
        self.sub_name::<N>("status")
    }

    /// Generate a sub-entity name for an action component
    /// Returns: `<action>/_action/<suffix>` (e.g., `fibonacci/_action/send_goal`)
    ///
    /// The caller is responsible for constructing the full key expression
    /// by wrapping this name in a `ServiceInfo` or `TopicInfo` with the
    /// correct sub-service/sub-topic type name.
    fn sub_name<const N: usize>(&self, suffix: &str) -> heapless::String<N> {
        let mut name = heapless::String::new();
        let action_stripped = self.name.trim_matches('/');
        let _ = core::fmt::write(
            &mut name,
            format_args!("/{}/_action/{}", action_stripped, suffix),
        );
        name
    }
}

impl<'a> ServiceInfo<'a> {
    /// Create new service info
    pub const fn new(name: &'a str, type_name: &'a str, type_hash: &'a str) -> Self {
        Self {
            name,
            type_name,
            type_hash,
            domain_id: 0,
            node_name: None,
            namespace: "/",
        }
    }

    /// Create service info with custom domain ID
    pub const fn with_domain(mut self, domain_id: u32) -> Self {
        self.domain_id = domain_id;
        self
    }

    /// Set the node name for liveliness token generation
    pub const fn with_node_name(mut self, node_name: &'a str) -> Self {
        self.node_name = Some(node_name);
        self
    }

    /// Set the node namespace for liveliness token generation
    pub const fn with_namespace(mut self, namespace: &'a str) -> Self {
        self.namespace = namespace;
        self
    }
}

/// Transport error types.
///
/// No longer `Copy` — the `Backend` / `BackendDynamic` variants carry a
/// string diagnostic, which can't be `Copy`. Rust callers that used to
/// copy a `TransportError` value repeatedly now need `.clone()` or
/// `ref` in match arms. C/C++ callers are unaffected — both map
/// `TransportError` to integer codes (`nros_ret_t` / `ErrorCode`)
/// before crossing the FFI boundary.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TransportError {
    /// Failed to connect to transport
    ConnectionFailed,
    /// Connection was closed
    Disconnected,
    /// Failed to create publisher
    PublisherCreationFailed,
    /// Failed to create subscriber
    SubscriberCreationFailed,
    /// Failed to create service server
    ServiceServerCreationFailed,
    /// Failed to create service client
    ServiceClientCreationFailed,
    /// Failed to publish message
    PublishFailed,
    /// Failed to send service request
    ServiceRequestFailed,
    /// Failed to send service reply
    ServiceReplyFailed,
    /// Serialization error
    SerializationError,
    /// Deserialization error
    DeserializationError,
    /// Buffer too small
    BufferTooSmall,
    /// Incoming message exceeded the static buffer capacity
    MessageTooLarge,
    /// Timeout waiting for message
    Timeout,
    /// Invalid configuration
    InvalidConfig,
    /// Resource (slot, buffer, queue) momentarily unavailable. Retry.
    /// Phase 99: returned by `try_loan` when arena slots are full and
    /// by `try_borrow` when no message is ready (alternative to
    /// `Ok(None)` for backends that prefer the error variant).
    WouldBlock,
    /// Requested allocation exceeds backend capacity. Phase 99:
    /// `try_loan(len)` returns this when `len` > arena slot size.
    TooLarge,
    /// Failed to start background tasks
    TaskStartFailed,
    /// Failed to poll for incoming messages
    PollFailed,
    /// Failed to send keepalive
    KeepaliveFailed,
    /// Failed to send join message
    JoinFailed,
    /// Caller supplied a NULL pointer, an out-of-range value, or an
    /// inconsistent argument combination. Phase 102.1.
    InvalidArgument,
    /// The backend does not implement this operation. Optional
    /// callbacks return this; the runtime then falls back to a
    /// default path. Phase 102.1.
    Unsupported,
    /// Memory allocation failed. Returned by backends on `std` /
    /// `alloc`-equipped platforms when heap allocation fails.
    /// Bare-metal backends generally do not produce this — they
    /// preallocate at session-open time. Phase 102.1.
    BadAlloc,
    /// Publisher and subscriber QoS profiles do not match in a way
    /// the backend cannot reconcile. Phase 102.1.
    IncompatibleQos,
    /// Topic, service, or action name failed validation. Phase 102.1.
    TopicNameInvalid,
    /// A request referenced a node that does not exist in this
    /// session. Phase 102.1.
    NodeNameNonExistent,
    /// The backend does not support loaned messages on this entity,
    /// or the loan slot is currently in use. Phase 102.1.
    LoanNotSupported,
    /// No data was available on a non-blocking receive. Distinct
    /// from `Timeout`: fires immediately, not after a bounded wait.
    /// Phase 102.1.
    NoData,
    /// Phase 115.A.2 — caller passed a versioned vtable struct
    /// (e.g. `NrosTransportOps`) with an `abi_version` the runtime
    /// doesn't know. Maps to `NROS_RMW_RET_INCOMPATIBLE_ABI` at
    /// the C boundary.
    IncompatibleAbi,
    /// Backend-specific error with a `'static` diagnostic string.
    ///
    /// Useful for zenoh-pico / XRCE-DDS return codes that map to a
    /// fixed set of known messages. `no_std`-compatible.
    Backend(&'static str),
    /// Backend-specific error with an owned diagnostic string.
    ///
    /// Available only with the `alloc` feature. Use this when the
    /// diagnostic is formatted at runtime (e.g. from a C error code
    /// plus a socket address).
    #[cfg(feature = "alloc")]
    BackendDynamic(alloc::string::String),
}

/// QoS history policy
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum QoSHistoryPolicy {
    /// **The caller did not state a history policy** — the backend picks.
    /// Upstream's `RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT`; lowers to
    /// `NROS_RMW_HISTORY_SYSTEM_DEFAULT` (0) across the C ABI.
    ///
    /// See [`QoSSystemDefaults`] for how a backend resolves it. Listed
    /// first to match rclrs's enumerator order.
    SystemDefault,
    /// Keep last N messages (where N is defined in QoSProfile)
    #[default]
    KeepLast,
    /// Keep all messages (up to resource limits)
    KeepAll,
}

/// QoS reliability policy
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum QoSReliabilityPolicy {
    /// **The caller did not state a reliability policy** — the backend picks.
    /// Upstream's `RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT`; lowers to
    /// `NROS_RMW_RELIABILITY_SYSTEM_DEFAULT` (0) across the C ABI.
    ///
    /// See [`QoSSystemDefaults`].
    SystemDefault,
    /// Reliable delivery (retransmit if needed).
    ///
    /// Default — matches ROS 2 `rmw_qos_profile_default` and the
    /// `QoSProfile::default()` / `QOS_PROFILE_DEFAULT` aggregates.
    #[default]
    Reliable,
    /// Best-effort delivery (no retransmits)
    BestEffort,
}

/// QoS durability policy
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum QoSDurabilityPolicy {
    /// **The caller did not state a durability policy** — the backend picks.
    /// Upstream's `RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT`; lowers to
    /// `NROS_RMW_DURABILITY_SYSTEM_DEFAULT` (0) across the C ABI.
    ///
    /// See [`QoSSystemDefaults`].
    SystemDefault,
    /// Messages are discarded when subscriber disconnects
    #[default]
    Volatile,
    /// Messages are persisted for late-joining subscribers
    TransientLocal,
}

/// QoS liveliness policy. Matches DDS `LIVELINESS` semantics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum QoSLivelinessPolicy {
    /// No liveliness assertion or tracking. Default for entities
    /// that don't care about liveliness.
    #[default]
    None = 0,
    /// Backend's keepalive task asserts liveliness automatically.
    Automatic = 1,
    /// Application calls `assert_liveliness()` at the node level.
    ///
    /// Phase 376 W5/B2 — this was 3 and `ManualByTopic` was 2, the opposite of
    /// upstream. The discriminant IS the C ABI value (`liveliness_kind` is
    /// written with `as u8`), and the cyclonedds backend turns it into a real
    /// DDS liveliness kind that a ROS peer matches on, so the swap was visible
    /// on the wire and nowhere else.
    ManualByNode = 2,
    /// Application calls `assert_liveliness()` per topic explicitly.
    ManualByTopic = 3,
}

/// Phase 211.H — which side of a topic a [`QoSOverride`] targets.
/// Mirrors the `<role>` segment of a ROS 2
/// `qos_overrides.<topic>.<role>.<policy>` launch parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QoSOverrideRole {
    /// `qos_overrides.<topic>.publisher.*`
    Publisher,
    /// `qos_overrides.<topic>.subscription.*`
    Subscription,
}

/// Phase 211.H — a single policy value a [`QoSOverride`] sets. A typed enum
/// (not a string) so the codegen that bakes these from the plan catches an
/// unknown policy / mistyped value at generation time rather than silently
/// no-op-ing at runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QoSOverrideValue {
    /// `.reliability` → Reliable / BestEffort.
    Reliability(QoSReliabilityPolicy),
    /// `.durability` → Volatile / TransientLocal.
    Durability(QoSDurabilityPolicy),
    /// `.history` → KeepLast / KeepAll.
    History(QoSHistoryPolicy),
    /// `.depth` → KeepLast depth.
    Depth(u32),
    /// Issue 0303 — `.deadline` → [`QoSProfile::deadline_ms`]. `0` /
    /// [`DURATION_INFINITE_MS`] = no deadline check.
    Deadline(u32),
    /// Issue 0303 — `.lifespan` → [`QoSProfile::lifespan_ms`].
    Lifespan(u32),
    /// Issue 0303 — `.liveliness` → [`QoSProfile::liveliness_kind`].
    Liveliness(QoSLivelinessPolicy),
    /// Issue 0303 — `.liveliness_lease_duration` →
    /// [`QoSProfile::liveliness_lease_ms`].
    LivelinessLease(u32),
}

/// Issue 0303 — the wire form a baked QoS override travels in:
/// `(topic, role, policy, value)`, all primitives.
///
/// One numbering for every language: `nros_qos_override_t` (C),
/// `nros_cpp_qos_override_t` (C++), `RuntimeCtx::qos_overrides` (the Rust
/// entry bake) and `NodeRecord::qos_overrides` are all THIS tuple. Codes rather
/// than [`QoSOverride`] because the table crosses the C ABI and rides
/// `nros-platform`, which sits below this crate in the layer graph.
///
/// `topic` is `&'static` because every producer bakes a literal.
pub type QoSOverrideCode = (&'static str, u8, u8, u32);

/// `role` codes for [`QoSOverrideCode`].
pub mod qos_override_role {
    /// The override targets publishers on the topic.
    pub const PUBLISHER: u8 = 0;
    /// The override targets subscriptions on the topic.
    pub const SUBSCRIPTION: u8 = 1;
}

/// `policy` codes for [`QoSOverrideCode`].
///
/// Append-only: these numbers are baked into shipped images and mirrored in
/// two C headers. Never renumber — add.
pub mod qos_override_policy {
    /// value `0` = best_effort, `1` = reliable.
    pub const RELIABILITY: u8 = 0;
    /// value `0` = volatile, `1` = transient_local.
    pub const DURABILITY: u8 = 1;
    /// value `0` = keep_last, `1` = keep_all.
    pub const HISTORY: u8 = 2;
    /// value = the KeepLast depth.
    pub const DEPTH: u8 = 3;
    /// value = milliseconds (issue 0303).
    pub const DEADLINE: u8 = 4;
    /// value = milliseconds (issue 0303).
    pub const LIFESPAN: u8 = 5;
    /// value = [`super::QoSLivelinessPolicy`] discriminant (issue 0303).
    pub const LIVELINESS: u8 = 6;
    /// value = milliseconds (issue 0303).
    pub const LIVELINESS_LEASE: u8 = 7;
}

/// Decode one [`QoSOverrideCode`] into a typed [`QoSOverride`].
///
/// `None` for an unrecognised role or policy code. THE one decoder: before
/// issue 0303 this match existed four times (nros-node, nros-c, nros-cpp, and
/// the executor's node record), each with a silent catch-all, so adding a
/// policy meant finding all four — and the two FFI copies had already been
/// forgotten once.
pub fn decode_qos_override(code: &QoSOverrideCode) -> Option<QoSOverride> {
    let (topic, role, policy, value) = *code;
    decode_qos_override_parts(topic, role, policy, value)
}

/// [`decode_qos_override`] over loose parts — for the FFI paths, which read the
/// fields out of a `#[repr(C)]` struct rather than a tuple.
pub fn decode_qos_override_parts(
    topic: &'static str,
    role: u8,
    policy: u8,
    value: u32,
) -> Option<QoSOverride> {
    Some(QoSOverride {
        topic,
        role: decode_qos_override_role(role)?,
        value: decode_qos_override_value(policy, value)?,
    })
}

/// Decode a `role` code. `None` for an unrecognised one.
pub fn decode_qos_override_role(role: u8) -> Option<QoSOverrideRole> {
    match role {
        qos_override_role::PUBLISHER => Some(QoSOverrideRole::Publisher),
        qos_override_role::SUBSCRIPTION => Some(QoSOverrideRole::Subscription),
        _ => None,
    }
}

/// Decode a `(policy, value)` code pair. `None` for an unrecognised policy or
/// an out-of-range enum value.
///
/// Split out from [`decode_qos_override`] for the FFI paths: they have already
/// matched the topic against a `*const c_char`, so they need the VALUE without
/// a `&'static str` to build a whole [`QoSOverride`] around.
pub fn decode_qos_override_value(policy: u8, value: u32) -> Option<QoSOverrideValue> {
    let out = match policy {
        qos_override_policy::RELIABILITY => QoSOverrideValue::Reliability(if value == 0 {
            QoSReliabilityPolicy::BestEffort
        } else {
            QoSReliabilityPolicy::Reliable
        }),
        qos_override_policy::DURABILITY => QoSOverrideValue::Durability(if value == 1 {
            QoSDurabilityPolicy::TransientLocal
        } else {
            QoSDurabilityPolicy::Volatile
        }),
        qos_override_policy::HISTORY => QoSOverrideValue::History(if value == 1 {
            QoSHistoryPolicy::KeepAll
        } else {
            QoSHistoryPolicy::KeepLast
        }),
        qos_override_policy::DEPTH => QoSOverrideValue::Depth(value),
        qos_override_policy::DEADLINE => QoSOverrideValue::Deadline(value),
        qos_override_policy::LIFESPAN => QoSOverrideValue::Lifespan(value),
        // The encoder is `nros_orchestration_ir::qos_override`; both ends name
        // the variant rather than its number, so W5/B2's renumbering moved them
        // together instead of silently swapping two policies.
        qos_override_policy::LIVELINESS => QoSOverrideValue::Liveliness(match value {
            v if v == QoSLivelinessPolicy::None as u32 => QoSLivelinessPolicy::None,
            v if v == QoSLivelinessPolicy::Automatic as u32 => QoSLivelinessPolicy::Automatic,
            v if v == QoSLivelinessPolicy::ManualByNode as u32 => QoSLivelinessPolicy::ManualByNode,
            v if v == QoSLivelinessPolicy::ManualByTopic as u32 => {
                QoSLivelinessPolicy::ManualByTopic
            }
            _ => return None,
        }),
        qos_override_policy::LIVELINESS_LEASE => QoSOverrideValue::LivelinessLease(value),
        _ => return None,
    };
    Some(out)
}

/// Phase 211.H — one per-topic QoS override, lowered from a ROS 2
/// `qos_overrides.<topic>.<role>.<policy>` launch parameter by the planner and
/// baked into a `&'static [QoSOverride]` table by the entry codegen. The node
/// folds the matching entries into the entity's [`QoSProfile`] at
/// `create_publisher` / `create_subscription` time (setup-time, single
/// linear scan, no alloc), *before* the backend-compat `validate_against` —
/// so an override the active RMW can't honour still errors loudly, never a
/// silent downgrade.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QoSOverride {
    /// The resolved (remapped) topic name the override targets, e.g.
    /// `"/chatter"`. Matched exactly against the entity's topic.
    pub topic: &'static str,
    /// Publisher or subscription side.
    pub role: QoSOverrideRole,
    /// The policy + value to set.
    pub value: QoSOverrideValue,
}

/// Phase-301 (issue 0241) — explicit "infinite" spelling for the u32
/// millisecond QoS duration fields (`deadline_ms`, `lifespan_ms`,
/// `liveliness_lease_ms`). Semantically identical to `0` (unset /
/// no-check) at every check site; exists so a caller can distinguish
/// "default" from "deliberately infinite". Mirrors the C header's
/// `NROS_RMW_DURATION_INFINITE_MS`.
pub const DURATION_INFINITE_MS: u32 = u32::MAX;

/// Phase-301 (issue 0241) — lower a [`core::time::Duration`] into a u32
/// millisecond QoS field. Boundary contract:
///
/// - zero stays `0` (unset / no-check);
/// - sub-millisecond remainders CEIL to the next ms (rounding down
///   would silently turn a real deadline into "no deadline");
/// - values at or past [`DURATION_INFINITE_MS`] ms are a create-time
///   error, never a clamp (infinite is requested via the sentinel or
///   `0`, not by a huge finite duration).
pub fn duration_to_qos_ms(d: core::time::Duration) -> Result<u32, TransportError> {
    if d.is_zero() {
        return Ok(0);
    }
    let mut ms = d.as_millis();
    if !d.subsec_nanos().is_multiple_of(1_000_000) {
        ms += 1;
    }
    if ms >= DURATION_INFINITE_MS as u128 {
        return Err(TransportError::InvalidArgument);
    }
    Ok(ms as u32)
}

/// Full DDS-shaped QoS profile. Matches the field set of upstream
/// `rmw_qos_profile_t`.
///
/// Backends advertise per-policy support via
/// [`Session::supported_qos_policies`]; entities created with a
/// profile the active backend can't honour return
/// [`TransportError::IncompatibleQos`] synchronously at create time
/// — no silent downgrade.
///
/// Zero-valued time-window fields ("off") mean infinite — the policy
/// is effectively disabled for the entity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QoSProfile {
    /// History policy
    pub history: QoSHistoryPolicy,
    /// Reliability policy
    pub reliability: QoSReliabilityPolicy,
    /// Durability policy
    pub durability: QoSDurabilityPolicy,
    /// Liveliness policy
    pub liveliness_kind: QoSLivelinessPolicy,
    /// History depth (only used if history is KeepLast)
    pub depth: u32,
    /// Subscription max-inter-arrival / publisher offered-rate, ms.
    /// `0` = infinite (no deadline check).
    pub deadline_ms: u32,
    /// Sample expiry, ms. Subscribers filter samples older than this.
    /// `0` = infinite (no expiry).
    pub lifespan_ms: u32,
    /// Liveliness lease, ms. `0` = infinite.
    pub liveliness_lease_ms: u32,
    /// If `true`, topic-name encoding skips the `/rt/` ROS prefix.
    pub avoid_ros_namespace_conventions: bool,
    /// Phase 282 (#145) — publisher-side "express" hint: this publisher's
    /// samples bypass transport tx batching (zenoh: the wire EXPRESS flag; a
    /// batching zenoh-pico session sends them immediately instead of queueing
    /// them for the next flush). A transport hint, not a DDS policy — no RxO
    /// matching, no backend-compat validation; ignored by subscriptions and by
    /// backends without a batching concept.
    pub tx_express: bool,
}

impl Default for QoSProfile {
    fn default() -> Self {
        Self::QOS_PROFILE_DEFAULT
    }
}

/// The depth sentinel — "the caller did not state a queue depth".
///
/// issue 0829. Upstream spells it `RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT = 0`
/// (`rmw/include/rmw/types.h`); depth is `size_t` there and `uint16_t` in our
/// C ABI, but 0 is 0 in both. The value was already free on every backend:
/// Cyclone REJECTS `KEEP_LAST` with depth 0 outright
/// (`validate_history_qospolicy`, `ddsi_plist.c:2603-2604`), the XRCE client
/// already reads it as "unstated" and drops the field from the wire
/// (`create_entities_bin.c:148`), and both `read_entity_qos`
/// (`nros-rmw-cyclonedds/src/qos.cpp:138`) and `report_qos_downgrade`
/// (`nros-rmw-cffi/src/lib.rs:1903`) already treat a 0 read-back as "no
/// answer" rather than as an answer.
pub const DEPTH_SYSTEM_DEFAULT: u32 = 0;

/// What ONE backend resolves the `SYSTEM_DEFAULT` sentinel to.
///
/// issue 0829. `rmw_qos_profile_system_default` is an absence, and the RMW
/// fills it — which means the answer is per backend and there is no constant
/// this crate could bake. Each backend declares its own `QoSSystemDefaults`
/// and applies it with [`QoSProfile::resolve_system_default`] at its create
/// entry, **before anything is derived from the QoS**.
///
/// That ordering is load-bearing on the zenoh path: the profile is serialised
/// into the liveliness-token keyexpr that a ROS `rmw_zenoh_cpp` peer parses
/// out of the graph (`nros-rmw-zenoh/src/keyexpr.rs`). Upstream resolves in
/// `best_available_qos` before the entity and its token exist, so its tokens
/// never carry a sentinel; ours must not either, or we advertise `0:0:0,0` to
/// peers as though it were a policy.
///
/// The values a backend picks should mirror **the corresponding upstream
/// RMW**, not the raw middleware default — interop with a ROS peer is the
/// requirement, and the two differ. Leaving `dds_qset_reliability` unset gives
/// Cyclone's own reader default of BEST_EFFORT (`ddsi_plist.c:3470`), where
/// `rmw_cyclonedds_cpp` deliberately picks RELIABLE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QoSSystemDefaults {
    /// Concrete reliability. Must not itself be `SystemDefault`.
    pub reliability: QoSReliabilityPolicy,
    /// Concrete durability. Must not itself be `SystemDefault`.
    pub durability: QoSDurabilityPolicy,
    /// Concrete history. Must not itself be `SystemDefault`.
    pub history: QoSHistoryPolicy,
    /// Concrete queue depth. `0` means the backend genuinely has no answer and
    /// defers further down the stack — the XRCE case, where the client encodes
    /// exactly that (`optional_history_depth = false`) and the Agent's DDS
    /// layer resolves it.
    pub depth: u32,
}

impl QoSProfile {
    /// Phase 211.H — fold the plan's `qos_overrides` matching `topic` + `role`
    /// into this profile, returning the overridden profile. Setup-time only
    /// (called from `create_publisher`/`create_subscription`): a single linear
    /// scan over the baked `&'static` table, no alloc, RT-safe. Later entries
    /// win on a duplicate `(topic, role, policy)` (last-write), matching the
    /// planner's sorted, de-conflicted emit. Non-matching entries are ignored,
    /// so passing the whole node table to every entity is cheap + correct.
    #[must_use]
    pub fn apply_overrides(
        mut self,
        topic: &str,
        role: QoSOverrideRole,
        overrides: &[QoSOverride],
    ) -> Self {
        for ovr in overrides {
            if ovr.topic == topic && ovr.role == role {
                self.apply_override_value(ovr.value);
            }
        }
        self
    }

    /// Issue 0303 — apply ONE decoded override value. The single place a
    /// policy maps to the field it sets; `apply_overrides` and the FFI folds
    /// both go through it, so a new policy cannot reach some paths only.
    pub fn apply_override_value(&mut self, value: QoSOverrideValue) {
        match value {
            QoSOverrideValue::Reliability(r) => self.reliability = r,
            QoSOverrideValue::Durability(d) => self.durability = d,
            QoSOverrideValue::History(h) => self.history = h,
            QoSOverrideValue::Depth(d) => self.depth = d,
            QoSOverrideValue::Deadline(ms) => self.deadline_ms = ms,
            QoSOverrideValue::Lifespan(ms) => self.lifespan_ms = ms,
            QoSOverrideValue::Liveliness(k) => self.liveliness_kind = k,
            QoSOverrideValue::LivelinessLease(ms) => self.liveliness_lease_ms = ms,
        }
    }

    /// Issue 0303 — fold baked [`QoSOverrideCode`]s for one `(topic, role)`.
    /// Unrecognised codes are skipped; the producers reject them at BAKE time
    /// (`nros_orchestration_ir::qos_override`), so a code reaching here that
    /// this build does not know is an older image running newer data, not a
    /// user error to diagnose at runtime.
    pub fn apply_override_codes(
        self,
        topic: &str,
        role: QoSOverrideRole,
        codes: &[QoSOverrideCode],
    ) -> Self {
        let mut qos = self;
        for code in codes {
            if let Some(ovr) = decode_qos_override(code) {
                qos = qos.apply_overrides(topic, role, core::slice::from_ref(&ovr));
            }
        }
        qos
    }
}

impl QoSProfile {
    /// Internal const builder. Extended-policy fields default to
    /// "off" (zero) and `liveliness_kind = Automatic` (the upstream
    /// `rmw_qos_profile_default` choice).
    const fn build(
        reliability: QoSReliabilityPolicy,
        durability: QoSDurabilityPolicy,
        history: QoSHistoryPolicy,
        depth: u32,
    ) -> Self {
        Self {
            history,
            reliability,
            durability,
            liveliness_kind: QoSLivelinessPolicy::Automatic,
            depth,
            deadline_ms: 0,
            lifespan_ms: 0,
            liveliness_lease_ms: 0,
            avoid_ros_namespace_conventions: false,
            tx_express: false,
        }
    }

    /// Create new QoS settings with defaults (matches `QOS_PROFILE_DEFAULT`:
    /// Reliable, Volatile, KeepLast(10)).
    pub const fn new() -> Self {
        Self::QOS_PROFILE_DEFAULT
    }

    /// Best-effort QoS (for real-time)
    pub const BEST_EFFORT: Self = Self::build(
        QoSReliabilityPolicy::BestEffort,
        QoSDurabilityPolicy::Volatile,
        QoSHistoryPolicy::KeepLast,
        1,
    );

    /// Reliable QoS
    pub const RELIABLE: Self = Self::build(
        QoSReliabilityPolicy::Reliable,
        QoSDurabilityPolicy::Volatile,
        QoSHistoryPolicy::KeepLast,
        10,
    );

    /// `rmw_qos_profile_system_default` — **an absence, not a profile.**
    ///
    /// issue 0829. Every field is the sentinel: upstream's constant names no
    /// concrete policy at all, and the two reference RMWs fill the absence with
    /// different numbers — `rmw_cyclonedds_cpp`'s `create_readwrite_qos` folds
    /// `RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT` to `KEEP_LAST(1)`, while
    /// `rmw_zenoh_cpp`'s `QoS::QoS()` fills it from
    /// `RMW_ZENOH_DEFAULT_HISTORY_DEPTH`, which is 42, over a comment stating
    /// the contract outright: *"If the depth field in the qos profile is set to
    /// 0, the RMW implementation has the liberty to assign a default depth."*
    ///
    /// So no baked number can be right. This carried a concrete
    /// `Reliable / Volatile / KeepLast(1)` until 2026-09-03, and
    /// `nros::qos::SYSTEM_DEFAULT` carried a concrete depth **10**, which is how
    /// one name shipped two queue depths. Both are gone; the backend resolves
    /// this at its create entry via [`QoSProfile::resolve_system_default`].
    ///
    /// `liveliness_kind` is [`QoSLivelinessPolicy::None`], which IS the
    /// sentinel on that policy — it lowers to
    /// `NROS_RMW_LIVELINESS_SYSTEM_DEFAULT` (0), the two having collapsed onto
    /// one value in phase-376 W5/B2.
    pub const QOS_PROFILE_SYSTEM_DEFAULT: Self = Self {
        history: QoSHistoryPolicy::SystemDefault,
        reliability: QoSReliabilityPolicy::SystemDefault,
        durability: QoSDurabilityPolicy::SystemDefault,
        liveliness_kind: QoSLivelinessPolicy::None,
        depth: DEPTH_SYSTEM_DEFAULT,
        deadline_ms: 0,
        lifespan_ms: 0,
        liveliness_lease_ms: 0,
        avoid_ros_namespace_conventions: false,
        tx_express: false,
    };

    /// Default QoS profile (matches rmw_qos_profile_default)
    pub const QOS_PROFILE_DEFAULT: Self = Self::build(
        QoSReliabilityPolicy::Reliable,
        QoSDurabilityPolicy::Volatile,
        QoSHistoryPolicy::KeepLast,
        10,
    );

    /// Sensor data QoS profile (matches rmw_qos_profile_sensor_data)
    pub const QOS_PROFILE_SENSOR_DATA: Self = Self::build(
        QoSReliabilityPolicy::BestEffort,
        QoSDurabilityPolicy::Volatile,
        QoSHistoryPolicy::KeepLast,
        5,
    );

    /// Services default QoS profile (matches rmw_qos_profile_services_default)
    pub const QOS_PROFILE_SERVICES_DEFAULT: Self = Self::build(
        QoSReliabilityPolicy::Reliable,
        QoSDurabilityPolicy::Volatile,
        QoSHistoryPolicy::KeepLast,
        10,
    );

    /// Parameters QoS profile (matches rmw_qos_profile_parameters)
    /// Mirrors `rmw_qos_profile_parameters`: KEEP_LAST(1000), RELIABLE,
    /// **VOLATILE**.
    ///
    /// issue 0793 — this said `TransientLocal` until 2026-08-25, disagreeing
    /// both with upstream (`/opt/ros/<distro>/include/rmw/rmw/qos_profiles.h`)
    /// and with our own second copy of the same profile, `nros::qos::PARAMETERS`,
    /// which was already correct. Two copies of one profile that disagree is the
    /// defect; the wrong one being the one named after the upstream constant is
    /// what made it hard to see.
    pub const QOS_PROFILE_PARAMETERS: Self = Self::build(
        QoSReliabilityPolicy::Reliable,
        QoSDurabilityPolicy::Volatile,
        QoSHistoryPolicy::KeepLast,
        1000,
    );

    /// Clock QoS profile - same as sensor data but with depth 1
    pub const QOS_PROFILE_CLOCK: Self = Self::build(
        QoSReliabilityPolicy::BestEffort,
        QoSDurabilityPolicy::Volatile,
        QoSHistoryPolicy::KeepLast,
        1,
    );

    /// Parameter events QoS profile (matches rmw_qos_profile_parameter_events)
    pub const QOS_PROFILE_PARAMETER_EVENTS: Self = Self::build(
        QoSReliabilityPolicy::Reliable,
        QoSDurabilityPolicy::Volatile,
        QoSHistoryPolicy::KeepAll,
        0, // Not used with KeepAll
    );

    /// Action status default QoS profile (matches rcl_action_qos_profile_status_default)
    pub const QOS_PROFILE_ACTION_STATUS_DEFAULT: Self = Self::build(
        QoSReliabilityPolicy::Reliable,
        QoSDurabilityPolicy::TransientLocal,
        QoSHistoryPolicy::KeepLast,
        1,
    );

    /// PX4 companion QoS profile (Phase 233 / RFC-0039 Track B). Matches the
    /// QoS PX4's `uxrce_dds_client` uses on `/fmu/out/*` and `/fmu/in/*` —
    /// `BEST_EFFORT` + `VOLATILE` + `KEEP_LAST(1)`. A nano-ros node talking to
    /// the same `MicroXRCEAgent` must use this (a reliable or
    /// `TRANSIENT_LOCAL` reader will not match PX4's volatile best-effort
    /// writers). Verified against real PX4 SITL (`nros-px4-sitl-test`):
    /// `TRANSIENT_LOCAL` durability silently fails to match `/fmu/out/*`.
    /// Adjust depth via `.keep_last(n)` for higher-rate streams.
    pub const QOS_PROFILE_PX4: Self = Self::build(
        QoSReliabilityPolicy::BestEffort,
        QoSDurabilityPolicy::Volatile,
        QoSHistoryPolicy::KeepLast,
        1,
    );

    // --- Static constructor methods (matching rclrs API) ---

    /// Get the default QoS profile for ordinary topics
    pub const fn topics_default() -> Self {
        Self::QOS_PROFILE_DEFAULT
    }

    /// The PX4 companion QoS profile ([`QOS_PROFILE_PX4`](Self::QOS_PROFILE_PX4))
    /// — use for `/fmu/out/*` subscriptions and `/fmu/in/*` publications against
    /// a `MicroXRCEAgent`.
    pub const fn px4() -> Self {
        Self::QOS_PROFILE_PX4
    }

    /// Get the default QoS profile for sensor data topics
    pub const fn sensor_data_default() -> Self {
        Self::QOS_PROFILE_SENSOR_DATA
    }

    /// Get the default QoS profile for services
    pub const fn services_default() -> Self {
        Self::QOS_PROFILE_SERVICES_DEFAULT
    }

    /// Get the default QoS profile for parameter services
    pub const fn parameters_default() -> Self {
        Self::QOS_PROFILE_PARAMETERS
    }

    /// Get the default QoS profile for parameter events
    pub const fn parameter_events_default() -> Self {
        Self::QOS_PROFILE_PARAMETER_EVENTS
    }

    /// Get the system default QoS profile
    pub const fn system_default() -> Self {
        Self::QOS_PROFILE_SYSTEM_DEFAULT
    }

    /// Get the default QoS profile for action status topics
    pub const fn action_status_default() -> Self {
        Self::QOS_PROFILE_ACTION_STATUS_DEFAULT
    }

    /// Get the default QoS profile for clock topics
    pub const fn clock_default() -> Self {
        Self::QOS_PROFILE_CLOCK
    }

    // --- Builder methods ---

    /// Set history to keep last N messages
    pub const fn keep_last(mut self, depth: u32) -> Self {
        self.history = QoSHistoryPolicy::KeepLast;
        self.depth = depth;
        self
    }

    /// Set history to keep all messages
    pub const fn keep_all(mut self) -> Self {
        self.history = QoSHistoryPolicy::KeepAll;
        self
    }

    /// Set reliability to reliable
    pub const fn reliable(mut self) -> Self {
        self.reliability = QoSReliabilityPolicy::Reliable;
        self
    }

    /// Set reliability to best-effort
    pub const fn best_effort(mut self) -> Self {
        self.reliability = QoSReliabilityPolicy::BestEffort;
        self
    }

    /// Set durability to volatile
    pub const fn volatile(mut self) -> Self {
        self.durability = QoSDurabilityPolicy::Volatile;
        self
    }

    /// Set durability to transient local
    pub const fn transient_local(mut self) -> Self {
        self.durability = QoSDurabilityPolicy::TransientLocal;
        self
    }

    /// Set reliability policy explicitly
    pub const fn reliability(mut self, policy: QoSReliabilityPolicy) -> Self {
        self.reliability = policy;
        self
    }

    /// Set durability policy explicitly
    pub const fn durability(mut self, policy: QoSDurabilityPolicy) -> Self {
        self.durability = policy;
        self
    }

    /// Set history policy explicitly
    pub const fn history(mut self, policy: QoSHistoryPolicy) -> Self {
        self.history = policy;
        self
    }

    /// Set history depth explicitly
    pub const fn depth(mut self, depth: u32) -> Self {
        self.depth = depth;
        self
    }

    /// Phase 282 (#145) — mark this publisher's samples "express": they
    /// bypass transport tx batching (sent immediately even when the batching
    /// knob is on). A transport hint for control-tier / latency-sensitive
    /// topics; ignored on subscriptions and by backends without batching.
    pub const fn tx_express(mut self, express: bool) -> Self {
        self.tx_express = express;
        self
    }

    /// Get history depth (for backwards compatibility)
    pub const fn history_depth(&self) -> u8 {
        if self.depth > 255 {
            255
        } else {
            self.depth as u8
        }
    }
}

/// Transport session configuration
#[derive(Debug, Clone)]
pub struct TransportConfig<'a> {
    /// Peer locator (e.g., "tcp/192.168.1.1:7447" or "serial//dev/ttyUSB0#baudrate=115200")
    pub locator: Option<&'a str>,
    /// Session mode: client, peer, or router
    pub mode: SessionMode,
    /// Additional transport properties (key-value pairs)
    ///
    /// These are passed through to the underlying transport backend.
    /// For zenoh-pico, recognized keys include:
    /// - `"multicast_scouting"` - Enable/disable multicast scouting (`"true"` or `"false"`)
    /// - `"scouting_timeout_ms"` - Scouting timeout in milliseconds
    /// - `"multicast_locator"` - Multicast group address
    /// - `"listen"` - Listen endpoint (e.g., `"tcp/0.0.0.0:0"`)
    /// - `"add_timestamp"` - Add timestamps to messages (`"true"` or `"false"`)
    pub properties: &'a [(&'a str, &'a str)],
    /// Node name for ROS 2 graph discovery liveliness token.
    ///
    /// Empty string (`""`) means no node-liveliness token is declared (preserves
    /// the pre-#104 behaviour). Non-empty causes the session to declare a
    /// `@ros2_lv/<domain>/<zid>/0/0/NN/%/<ns>/<node>` token on open.
    pub node_name: &'a str,
    /// Node namespace for the liveliness token (e.g., `""` or `"/ns1"`).
    ///
    /// Empty string is treated as root `"/"` by the keyexpr builder.
    pub namespace: &'a str,
    /// ROS 2 domain ID used in the liveliness token key expression.
    pub domain_id: u32,
}

impl Default for TransportConfig<'_> {
    fn default() -> Self {
        Self {
            locator: None,
            mode: SessionMode::Client,
            properties: &[],
            node_name: "",
            namespace: "",
            domain_id: 0,
        }
    }
}

/// Middleware-agnostic session configuration.
///
/// `RmwConfig` provides a uniform interface that any RMW backend can
/// interpret. Backends map the universal fields to their own connection
/// parameters and interpret `properties` for anything backend-specific.
///
/// # Examples
///
/// ```
/// use nros_rmw::{RmwConfig, SessionMode};
///
/// let config = RmwConfig {
///     locator: "tcp/192.168.1.1:7447",
///     mode: SessionMode::Client,
///     domain_id: 0,
///     node_name: "talker",
///     namespace: "",
///     properties: &[],
/// };
/// ```
#[derive(Debug, Clone, Copy)]
pub struct RmwConfig<'a> {
    /// Middleware-specific connection string.
    ///
    /// - zenoh: `"tcp/192.168.1.1:7447"` or `"udp/224.0.0.224:7447"`
    /// - XRCE-DDS: `"udp/192.168.1.1:2019"`
    pub locator: &'a str,
    /// Session mode (zenoh: client/peer; XRCE-DDS: always client)
    pub mode: SessionMode,
    /// ROS 2 domain ID (maps to DDS domain or zenoh key prefix)
    pub domain_id: u32,
    /// Node name (e.g., `"talker"`)
    pub node_name: &'a str,
    /// Node namespace (e.g., `""` or `"/ns1"`)
    pub namespace: &'a str,
    /// Backend-specific key/value properties.
    ///
    /// Uniform escape hatch for backend-specific tuning that doesn't fit
    /// the universal fields above. Each backend documents the keys it
    /// understands; unknown keys are ignored. Passing `&[]` is always
    /// valid.
    ///
    /// Examples:
    /// - zenoh: `"tls.root_ca"`, `"scouting.multicast.enabled"`
    /// - XRCE-DDS: `"agent_port"`, `"client_key"`
    pub properties: &'a [(&'a str, &'a str)],
}

impl Default for RmwConfig<'_> {
    fn default() -> Self {
        Self {
            locator: "tcp/127.0.0.1:7447",
            mode: SessionMode::Client,
            domain_id: 0,
            node_name: "node",
            namespace: "",
            properties: &[],
        }
    }
}

/// Locator transport protocol
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocatorProtocol {
    /// TCP transport (e.g., "tcp/127.0.0.1:7447")
    Tcp,
    /// UDP transport (e.g., "udp/192.168.1.50:2019" — common for XRCE-DDS)
    Udp,
    /// Serial/UART transport (e.g., "serial//dev/ttyUSB0#baudrate=115200")
    Serial,
    /// Unknown protocol
    Unknown,
}

/// Parse the protocol from a locator string
pub fn locator_protocol(locator: &str) -> LocatorProtocol {
    if locator.starts_with("tcp/") {
        LocatorProtocol::Tcp
    } else if locator.starts_with("udp/") {
        LocatorProtocol::Udp
    } else if locator.starts_with("serial/") {
        LocatorProtocol::Serial
    } else {
        LocatorProtocol::Unknown
    }
}

/// Validate a locator string format.
///
/// Returns `Ok(())` if the locator is well-formed, or an error message describing
/// the problem. This provides early feedback before zenoh-pico or XRCE-DDS rejects
/// a bad locator.
///
/// Supported formats:
/// - TCP: `tcp/<host>:<port>` (e.g., `tcp/127.0.0.1:7447`)
/// - UDP: `udp/<host>:<port>` (e.g., `udp/192.168.1.50:2019`)
/// - Serial: `serial/<device>#baudrate=<rate>` (e.g., `serial//dev/ttyUSB0#baudrate=115200`)
pub fn validate_locator(locator: &str) -> Result<(), &'static str> {
    match locator_protocol(locator) {
        LocatorProtocol::Tcp => {
            let rest = &locator[4..]; // skip "tcp/"
            if !rest.contains(':') {
                return Err("TCP locator must contain host:port (e.g., tcp/127.0.0.1:7447)");
            }
            Ok(())
        }
        LocatorProtocol::Udp => {
            let rest = &locator[4..]; // skip "udp/"
            if !rest.contains(':') {
                return Err("UDP locator must contain host:port (e.g., udp/192.168.1.50:2019)");
            }
            Ok(())
        }
        LocatorProtocol::Serial => {
            let rest = &locator[7..]; // skip "serial/"
            if rest.is_empty() {
                return Err(
                    "serial locator must specify device (e.g., serial//dev/ttyUSB0#baudrate=115200)",
                );
            }
            if !rest.contains("#baudrate=") {
                return Err(
                    "serial locator must include #baudrate=RATE (e.g., serial//dev/ttyUSB0#baudrate=115200)",
                );
            }
            // Validate baudrate is numeric
            if let Some(baud_str) = rest.split("#baudrate=").nth(1) {
                let baud_str = baud_str.split('#').next().unwrap_or(baud_str);
                if baud_str.parse::<u32>().is_err() {
                    return Err("serial baudrate must be a number");
                }
            }
            Ok(())
        }
        LocatorProtocol::Unknown => {
            Err("unknown locator protocol (expected tcp/, udp/, or serial/)")
        }
    }
}

/// Session mode
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum SessionMode {
    /// Connect as client to a router
    #[default]
    Client,
    /// Connect as peer for peer-to-peer communication
    Peer,
}

/// Transport session trait — the per-process anchor an RMW backend
/// gives to the executor.
///
/// # Threading
///
/// `&mut self` on every method means the executor serialises all
/// session calls onto a single thread. A backend may rely on this
/// — no internal locking is required for `create_*` / `close` /
/// `drive_io`. **Publisher / subscriber / service handles created
/// from the session, however, are typically used from worker
/// threads** and must carry their own synchronisation (see the
/// [`Publisher`] / [`Subscription`] trait docs).
///
/// # Calling pattern
///
/// 1. Open the session (backend-specific factory; not on this trait).
/// 2. `create_*` for every entity at startup. Creating entities mid-
///    flight after `drive_io` has run is allowed but not common.
/// 3. The executor calls `drive_io` periodically. Worker threads
///    publish / receive in parallel.
/// 4. `close` once at shutdown. Entities must be dropped first.
pub trait Session {
    /// RFC-0088 — the serialization format this backend speaks, as ROS 2's
    /// `rmw_get_serialization_format()` reports it ("One middleware can only
    /// have one encoding").
    ///
    /// Defaulted to CDR because every backend in tree except uORB speaks it,
    /// and a backend that speaks something else says so by overriding these
    /// two. They travel together: `SERIALIZATION_FORMAT` is the identity that
    /// crosses images (bridge config, tooling, the vtable slot) and
    /// `SERIALIZATION_FORMAT_ID` is the image-local discriminant used for the
    /// one-byte comparison a bridge makes at construction.
    const SERIALIZATION_FORMAT: &'static str =
        nros_serdes::format::SerializationFormatId::Cdr.as_str();

    /// Image-local discriminant for [`Self::SERIALIZATION_FORMAT`]. Never
    /// persisted, never compared across images — see `nros_serdes::format`.
    const SERIALIZATION_FORMAT_ID: nros_serdes::format::SerializationFormatId =
        nros_serdes::format::SerializationFormatId::Cdr;

    /// RFC-0088 — this session's serialization format, as ROS 2's
    /// `rmw_get_serialization_format()` reports it.
    ///
    /// **Per session, not per process.** ROS 2's function takes no handle
    /// because one process links one middleware; an `Executor::open_multi`
    /// image links two, so the answer must be asked of the session.
    ///
    /// The default answers from [`Self::SERIALIZATION_FORMAT`], which is right
    /// for any backend whose format is a compile-time fact. A session that
    /// dispatches to a backend chosen at run time — the C-ABI adapter, whose
    /// vtable carries the answer — overrides this to ask the backend.
    fn serialization_format(&self) -> &'static str {
        Self::SERIALIZATION_FORMAT
    }

    /// Error type for this session
    type Error;
    /// Publisher handle type
    type PublisherHandle;
    /// Subscription handle type
    type SubscriptionHandle;
    /// Service server handle type
    type ServiceHandle;
    /// Service client handle type
    type ClientHandle;

    /// Create a publisher bound to this session.
    ///
    /// May allocate transport resources (zenoh declarations, DDS
    /// writers). Returns a handle that outlives the call but not the
    /// session — drop the handle before `close()`.
    fn create_publisher(
        &mut self,
        topic: &TopicInfo,
        qos: QoSProfile,
    ) -> Result<Self::PublisherHandle, Self::Error>;

    /// Create a subscriber bound to this session.
    ///
    /// Subscribers may start receiving immediately after creation if
    /// the transport supports late-joining publishers. Late messages
    /// are buffered up to the QoS depth.
    fn create_subscription(
        &mut self,
        topic: &TopicInfo,
        qos: QoSProfile,
    ) -> Result<Self::SubscriptionHandle, Self::Error>;

    /// Create a service server bound to this session. Replies are
    /// matched to requests by the sequence number returned from
    /// [`ServiceTrait::take_request`].
    ///
    /// `qos` is applied to both the request and reply endpoints (a
    /// service is two DDS topics; rmw uses one profile for both). The
    /// default is [`QoSProfile::services_default`]
    /// (RELIABLE+VOLATILE+KEEP_LAST(10)).
    fn create_service(
        &mut self,
        service: &ServiceInfo,
        qos: QoSProfile,
    ) -> Result<Self::ServiceHandle, Self::Error>;

    /// Create a service client bound to this session.
    ///
    /// `qos` is applied to both the request and reply endpoints (a
    /// service is two DDS topics; rmw uses one profile for both). The
    /// default is [`QoSProfile::services_default`]
    /// (RELIABLE+VOLATILE+KEEP_LAST(10)).
    fn create_client(
        &mut self,
        service: &ServiceInfo,
        qos: QoSProfile,
    ) -> Result<Self::ClientHandle, Self::Error>;

    /// Close the session, releasing transport resources. All entity
    /// handles created from this session must already be dropped.
    fn close(&mut self) -> Result<(), Self::Error>;

    /// Drive transport I/O (poll network, dispatch callbacks).
    ///
    /// Both zenoh-pico and XRCE-DDS are pull-based: they require the
    /// application to periodically call this method to read from the
    /// network socket and dispatch incoming messages to subscriber
    /// buffers.
    ///
    /// `timeout_ms` is the maximum time to wait for data (0 = non-blocking;
    /// negative values mean "block indefinitely" — see Phase 84.D7 for the
    /// planned migration to `core::time::Duration`).
    ///
    /// **Required**. There is no default body — both shipped backends
    /// (zenoh and XRCE) must drive I/O, and a silent no-op default was a
    /// trap for third-party implementers. If your backend genuinely
    /// receives data via OS callbacks (push-based) and has nothing to do
    /// here, return `Ok(())` explicitly.
    fn drive_io(&mut self, timeout_ms: i32) -> Result<(), Self::Error>;

    /// Phase 109 — report which QoS policies the active backend
    /// honours. The runtime validates requested QoS against this mask
    /// at entity-create time and returns
    /// [`TransportError::IncompatibleQos`] if the requested profile
    /// includes a policy the backend can't enforce. **No silent
    /// downgrade.**
    ///
    /// Default returns [`QoSPolicyMask::CORE`] — reliability +
    /// durability VOLATILE + history + depth. Backends override per
    /// supported policy.
    fn supported_qos_policies(&self) -> QoSPolicyMask {
        QoSPolicyMask::CORE
    }

    /// Phase 110.0 — backend's next internal-event deadline in
    /// milliseconds from now (lease keepalive, heartbeat, reader
    /// ACK-NACK timeout, etc.).
    ///
    /// The executor caps its `drive_io` timeout against
    /// `min(user_timeout, timer_deadline, this)` so quiet links don't
    /// wake early, see no user-visible work, and round-trip back into
    /// `drive_io`. Returns `None` when the backend has no internal
    /// deadlines or chooses not to expose them.
    ///
    /// Default `None` keeps existing backends working unchanged; opt-in
    /// per backend.
    fn next_deadline_ms(&self) -> Option<u32> {
        None
    }

    /// Phase 124.B.1 — install (or clear, when `cb.is_none()`) the
    /// executor wake callback. The runtime calls this once per
    /// session after `open` with `cb` pointing at a runtime-owned
    /// function and `ctx` pointing at the executor's wake state.
    /// The backend stores `(cb, ctx)` in its per-session state and
    /// calls `cb(ctx)` whenever its transport notification path
    /// fires (datagram arrival, condvar wake, etc.) — the runtime
    /// cb does flag-write + condvar-signal atomically, so a
    /// `spin_once` blocked on the wake condvar resumes immediately
    /// instead of waiting for the next poll iteration.
    ///
    /// # Safety
    ///
    /// When `cb` is `Some`, `ctx` must remain valid until the callback is
    /// cleared or the session is closed. The backend may invoke `cb(ctx)` from
    /// its transport notification path.
    ///
    /// Default body: ignore the call. Poll-only backends (XRCE,
    /// bare-metal) leave the default in place; the executor still
    /// drains them on its deadline-bound cv-wait boundary.
    unsafe fn set_wake_callback(
        &mut self,
        cb: Option<unsafe extern "C" fn(ctx: *mut core::ffi::c_void)>,
        ctx: *mut core::ffi::c_void,
    ) {
        let _ = (cb, ctx);
    }

    /// Phase 130.4 — does this backend actually honour
    /// [`set_wake_callback`]?
    ///
    /// `true` means the backend installs the callback and will
    /// fire it from its async notify path (worker thread, ISR,
    /// signalfd, …). `false` (the default) means
    /// `set_wake_callback` was a no-op — the executor must drive
    /// I/O for the caller's full timeout because no async wake
    /// will pre-empt it.
    ///
    /// The executor uses this to choose between the wake-primitive
    /// wait (`NodeWake::wait_ms` / `std::Condvar::wait_timeout_while`)
    /// and a direct `drive_io(timeout_ms)`. Poll-only backends
    /// (XRCE-DDS-Client, bare-metal smoltcp) return `false`;
    /// event-driven backends (zenoh-pico with an RX task that
    /// invokes the callback on packet arrival) return `true`.
    ///
    /// [`set_wake_callback`]: Self::set_wake_callback
    fn supports_wake_callback(&self) -> bool {
        false
    }

    /// Phase 124.F.1 — session-level connectivity probe.
    ///
    /// Sends a wire-level round-trip probe and waits up to
    /// `timeout_ms`. `Ok(())` on reply, `Err(TransportError::Timeout)`
    /// on no reply, `Err(TransportError::Unsupported)` when the
    /// backend can't probe (DDS without participant introspection).
    /// Lesson from micro-ROS's `rmw_uros_ping_agent`.
    ///
    /// Default body: `Err(Unsupported)`. Backends with a native
    /// ping API (zenoh: `z_send_ping`; XRCE:
    /// `uxr_ping_agent_session_until_timeout`) opt in by overriding.
    fn ping_session(&mut self, timeout_ms: i32) -> Result<(), Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        let _ = timeout_ms;
        Err(TransportError::Unsupported.into())
    }

    /// phase-381 W3 — enumerate the nodes this session can see.
    ///
    /// A VISITOR, not a returned collection, because upstream's
    /// `rcutils_string_array_t` allocates two levels deep and there is no
    /// allocator at this seam. A caller-provided buffer is worse than it looks:
    /// the graph has no bound the CALLER can know. So the backend streams from
    /// state it already holds, peak extra RAM is one entry, and a caller with a
    /// bound stops early by returning `false`.
    ///
    /// `namespace` and `name` are ROS names. `enclave` is `None` where the
    /// backend does not track one — which is what lets this one method answer
    /// both `rmw_get_node_names` and `rmw_get_node_names_with_enclaves`.
    ///
    /// Every string is BORROWED for the duration of the call.
    ///
    /// **Must not block on the wire, and takes no timeout.** It reports what has
    /// ALREADY arrived, so the first call after startup legitimately returns a
    /// partial graph — a backend feeds its view from `drive_io`. Letting this
    /// block was considered and rejected: it would stall the executor's only
    /// thread inside an introspection call, on a runtime whose premise is that
    /// there is no other thread to do the work.
    ///
    /// Default body: `Err(Unsupported)` — a backend with no graph (XRCE) says
    /// so, and the runtime can tell that from an empty graph.
    fn get_node_names(
        &mut self,
        visit: &mut dyn FnMut(&str, &str, Option<&str>) -> bool,
    ) -> Result<(), Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        let _ = visit;
        Err(TransportError::Unsupported.into())
    }

    /// phase-381 W3 — how many publishers this session can see on `topic_name`.
    ///
    /// `topic_name` is a ROS name (`/chatter`); the backend mangles as needed.
    /// Same warm-up caveat as [`Self::get_node_names`]: a count reflects what
    /// has already been discovered, so it can be low right after startup and is
    /// never a proof of absence.
    ///
    /// Default body: `Err(Unsupported)` — distinct from `Ok(0)`, which claims
    /// there are none.
    fn count_publishers(&mut self, topic_name: &str) -> Result<usize, Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        let _ = topic_name;
        Err(TransportError::Unsupported.into())
    }

    /// phase-381 W3 — how many subscribers this session can see on `topic_name`.
    /// See [`Self::count_publishers`] for the caveats.
    fn count_subscribers(&mut self, topic_name: &str) -> Result<usize, Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        let _ = topic_name;
        Err(TransportError::Unsupported.into())
    }

    /// phase-381 W3 — every topic, with the types published or subscribed on it.
    ///
    /// A visitor for the same reason as [`Self::get_node_names`], and one call
    /// per distinct NAME: the contract hands over a name and the types on it,
    /// so a topic carrying two types is one visit with two entries, not two
    /// visits.
    ///
    /// `types_count` may legitimately be 0 on a partially discovered graph —
    /// reporting the name without a type beats dropping it.
    ///
    /// Default body: `Err(Unsupported)`.
    fn get_topic_names_and_types(
        &mut self,
        visit: &mut dyn FnMut(&str, &[&str]) -> bool,
    ) -> Result<(), Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        let _ = visit;
        Err(TransportError::Unsupported.into())
    }

    /// phase-381 W3 — every service, with its types. As
    /// [`Self::get_topic_names_and_types`], over servers and clients.
    fn get_service_names_and_types(
        &mut self,
        visit: &mut dyn FnMut(&str, &[&str]) -> bool,
    ) -> Result<(), Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        let _ = visit;
        Err(TransportError::Unsupported.into())
    }

    /// phase-381 W3 — what ONE named node publishes / subscribes / serves /
    /// calls.
    ///
    /// `node_name` and `node_namespace` are ROS names; a node the graph has not
    /// discovered yields no visits, which is NOT an error — see
    /// [`Self::get_node_names`] for why an empty answer is "not seen yet".
    ///
    /// `kind` selects which of the four upstream `*_by_node` questions this
    /// answers. One method rather than four because the four differ ONLY by
    /// which entity kind they keep, and four trait methods would be four copies
    /// of one filter.
    ///
    /// The trait keeps the ABI's `subscriber` vocabulary because
    /// `rmw_get_subscriber_names_and_types_by_node` is what upstream rmw calls
    /// it and RFC-0054 makes the C headers the SSoT. The USER-facing spelling
    /// is per language and settled at that layer: rcl says `subscriber`, rclcpp
    /// and rclrs say `subscription`.
    ///
    /// Default body: `Err(Unsupported)`.
    fn get_names_and_types_by_node(
        &mut self,
        kind: GraphEntityKind,
        node_name: &str,
        node_namespace: &str,
        visit: &mut dyn FnMut(&str, &[&str]) -> bool,
    ) -> Result<(), Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        let _ = (kind, node_name, node_namespace, visit);
        Err(TransportError::Unsupported.into())
    }

    /// phase-381 W3 — the endpoints on one topic, with the QoS each GRANTED.
    ///
    /// `publishers` selects `rmw_get_publishers_info_by_topic` (`true`) or
    /// `rmw_get_subscriptions_info_by_topic` (`false`).
    ///
    /// The granted profile is the whole reason a consumer asks: "why is nothing
    /// arriving" is usually a QoS incompatibility, and the REQUESTED profile
    /// cannot answer it. A backend that cannot read a remote's granted QoS says
    /// so per field rather than echoing the request back — see
    /// `rmw_topic_endpoint_info_t`.
    ///
    /// Default body: `Err(Unsupported)`.
    fn get_endpoint_info_by_topic(
        &mut self,
        publishers: bool,
        topic_name: &str,
        visit: &mut dyn FnMut(&GraphEndpointInfo<'_>) -> bool,
    ) -> Result<(), Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        let _ = (publishers, topic_name, visit);
        Err(TransportError::Unsupported.into())
    }
}

/// Which entity kind a `*_by_node` graph query keeps — phase-381 W3.
///
/// Named for upstream rmw's four `*_by_node` slots. `Subscriber` carries rmw's
/// word, not rclcpp's `subscription`; the user-facing spelling is chosen per
/// language one layer up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GraphEntityKind {
    Publisher,
    Subscriber,
    Service,
    Client,
}

/// One discovered endpoint on a topic — phase-381 W3, the Rust view of
/// `rmw_topic_endpoint_info_t`.
///
/// Every string BORROWS from the backend's own state for the duration of the
/// visit. A caller that needs one afterwards copies it; that is what lets the
/// graph stream without an allocator.
#[derive(Debug, Clone)]
pub struct GraphEndpointInfo<'a> {
    /// Node that owns the endpoint.
    pub node_name: &'a str,
    /// That node's namespace.
    pub node_namespace: &'a str,
    /// Fully-qualified type on the wire, e.g. `"std_msgs/msg/Int32"`.
    pub topic_type: &'a str,
    /// `true` for a publisher, `false` for a subscription.
    pub is_publisher: bool,
    /// The endpoint's 24-byte identity; all-zero when the backend has none.
    pub endpoint_gid: [u8; 24],
}

// No `qos` field, deliberately. The C seam carries one and fills it with the
// ABI's `*_UNKNOWN` sentinels, which is the contract for a policy a backend
// cannot determine (see `publisher_get_actual_qos`: write UNKNOWN and return
// OK; `UNSUPPORTED` means no read-back AT ALL).
//
// No backend can fill it today. zenoh's liveliness token carries the DECLARING
// side's own profile, not a negotiated grant, and this seam promises the
// granted one — reporting the declaration would be the plausible wrong answer
// the slot exists to avoid, since "why is nothing arriving" is usually a QoS
// mismatch and the requested profile cannot show it. Cyclone can read a real
// grant and will need this; adding a field to a Rust struct then is additive
// and not an ABI change, which is why carrying an always-`None` field now
// buys nothing.

/// Bitmask of QoS policies a backend can honour. See
/// [`Session::supported_qos_policies`].
///
/// `CORE` covers the policies every nano-ros backend implements:
/// reliability, durability=VOLATILE, history, depth. Backends opt
/// into additional policies by OR-ing the relevant flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QoSPolicyMask(pub u32);

impl QoSPolicyMask {
    pub const RELIABILITY: Self = Self(1 << 0);
    pub const DURABILITY_VOLATILE: Self = Self(1 << 1);
    pub const DURABILITY_TRANSIENT_LOCAL: Self = Self(1 << 2);
    pub const HISTORY: Self = Self(1 << 3);
    pub const DEPTH: Self = Self(1 << 4);
    pub const DEADLINE: Self = Self(1 << 5);
    pub const LIFESPAN: Self = Self(1 << 6);
    pub const LIVELINESS_AUTOMATIC: Self = Self(1 << 7);
    pub const LIVELINESS_MANUAL_BY_TOPIC: Self = Self(1 << 8);
    pub const LIVELINESS_MANUAL_BY_NODE: Self = Self(1 << 9);
    pub const LIVELINESS_LEASE: Self = Self(1 << 10);
    pub const AVOID_ROS_NAMESPACE_CONVENTIONS: Self = Self(1 << 11);

    /// Policies every nano-ros backend implements.
    pub const CORE: Self =
        Self(Self::RELIABILITY.0 | Self::DURABILITY_VOLATILE.0 | Self::HISTORY.0 | Self::DEPTH.0);

    /// `true` if `self` contains every policy in `other`.
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// Bitwise OR of two masks.
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

impl core::ops::BitOr for QoSPolicyMask {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self {
        self.union(rhs)
    }
}

impl core::ops::BitOrAssign for QoSPolicyMask {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

impl QoSProfile {
    /// Compute the set of QoS policies actually requested by this profile.
    ///
    /// Zero-valued time fields and `LivelinessKind::None` count as "not
    /// requesting" the corresponding policy — the cheap default.
    ///
    /// issue 0829 — the four `CORE` bits (reliability, durability, history,
    /// depth) used to be added UNCONDITIONALLY, on the reasoning that every
    /// nano-ros backend honours them. That is true and still not the right
    /// test: this function answers *what did the caller ASK FOR*, and a
    /// `SYSTEM_DEFAULT` policy asks for nothing. Starting from `CORE` made
    /// `QOS_PROFILE_SYSTEM_DEFAULT` demand all four, so a backend that could
    /// not honour one would reject the profile with `IncompatibleQos` — for
    /// requesting nothing. The pattern was already here for the extended
    /// policies three lines down (a zero `deadline_ms` declines its bit); it
    /// just never reached the four CORE ones.
    ///
    /// This RELAXES the mask, so it can only turn a rejection into an
    /// acceptance, never the reverse — and it flips no verdict today: every
    /// `supported_qos_policies` impl in the tree returns at least `CORE`
    /// (`traits.rs` default impl, `nros-node/src/mock.rs:259`,
    /// `nros-rmw-cffi/src/lib.rs:2580`, `nros-rmw-zenoh/src/shim/session.rs:1245`),
    /// so a CORE bit has never been the reason for a failure.
    pub fn required_policies(&self) -> QoSPolicyMask {
        let mut mask = QoSPolicyMask(0);
        if self.reliability != QoSReliabilityPolicy::SystemDefault {
            mask |= QoSPolicyMask::RELIABILITY;
        }
        if self.history != QoSHistoryPolicy::SystemDefault {
            mask |= QoSPolicyMask::HISTORY;
        }
        // Depth's sentinel is a VALUE, not a variant: `DEPTH_SYSTEM_DEFAULT`
        // is 0, the same 0 that `KEEP_ALL` profiles carry to mean "depth is
        // not used here" (`QOS_PROFILE_PARAMETER_EVENTS`). Both mean the
        // caller did not ask for a depth, so both decline the bit.
        if self.depth != DEPTH_SYSTEM_DEFAULT {
            mask |= QoSPolicyMask::DEPTH;
        }
        match self.durability {
            QoSDurabilityPolicy::SystemDefault => {}
            QoSDurabilityPolicy::Volatile => mask |= QoSPolicyMask::DURABILITY_VOLATILE,
            QoSDurabilityPolicy::TransientLocal => {
                mask |= QoSPolicyMask::DURABILITY_TRANSIENT_LOCAL
            }
        }
        // Phase-301 (issue 0241): DURATION_INFINITE_MS reads the same as 0
        // (infinite = no check) at every duration check site.
        if self.deadline_ms != 0 && self.deadline_ms != DURATION_INFINITE_MS {
            mask |= QoSPolicyMask::DEADLINE;
        }
        if self.lifespan_ms != 0 && self.lifespan_ms != DURATION_INFINITE_MS {
            mask |= QoSPolicyMask::LIFESPAN;
        }
        match self.liveliness_kind {
            QoSLivelinessPolicy::None => {}
            QoSLivelinessPolicy::Automatic => mask |= QoSPolicyMask::LIVELINESS_AUTOMATIC,
            QoSLivelinessPolicy::ManualByTopic => mask |= QoSPolicyMask::LIVELINESS_MANUAL_BY_TOPIC,
            QoSLivelinessPolicy::ManualByNode => mask |= QoSPolicyMask::LIVELINESS_MANUAL_BY_NODE,
        }
        if self.liveliness_lease_ms != 0 && self.liveliness_lease_ms != DURATION_INFINITE_MS {
            mask |= QoSPolicyMask::LIVELINESS_LEASE;
        }
        if self.avoid_ros_namespace_conventions {
            mask |= QoSPolicyMask::AVOID_ROS_NAMESPACE_CONVENTIONS;
        }
        mask
    }

    /// Returns `Err(TransportError::IncompatibleQos)` if any policy this
    /// profile requires is missing from the backend's `supported` mask.
    /// Used at entity-create time to enforce the **no silent
    /// degradation** contract.
    pub fn validate_against(&self, supported: QoSPolicyMask) -> Result<(), TransportError> {
        if supported.contains(self.required_policies()) {
            Ok(())
        } else {
            Err(TransportError::IncompatibleQos)
        }
    }

    /// Replace every `SYSTEM_DEFAULT` field with the backend's own answer.
    ///
    /// issue 0829 — a backend calls this at its create entry, **before**
    /// anything is derived from the profile (see [`QoSSystemDefaults`] for why
    /// the ordering matters on the zenoh path). Fields the caller DID state
    /// are left exactly as they are: this resolves an absence, it never
    /// overrides a request.
    ///
    /// Idempotent, and safe to call on a fully concrete profile — a profile
    /// with no sentinel in it is returned unchanged.
    #[must_use]
    pub const fn resolve_system_default(mut self, defaults: &QoSSystemDefaults) -> Self {
        if matches!(self.reliability, QoSReliabilityPolicy::SystemDefault) {
            self.reliability = defaults.reliability;
        }
        if matches!(self.durability, QoSDurabilityPolicy::SystemDefault) {
            self.durability = defaults.durability;
        }
        if matches!(self.history, QoSHistoryPolicy::SystemDefault) {
            self.history = defaults.history;
        }
        if self.depth == DEPTH_SYSTEM_DEFAULT {
            self.depth = defaults.depth;
        }
        self
    }

    /// `true` if any field is still the `SYSTEM_DEFAULT` sentinel.
    ///
    /// Depth is deliberately NOT part of this test: a `KEEP_ALL` profile
    /// legitimately carries depth 0 forever (`QOS_PROFILE_PARAMETER_EVENTS`),
    /// and a backend that resolves the sentinel depth to 0 — XRCE does, on
    /// purpose — leaves a resolved profile reading 0 here. This asks about the
    /// three POLICY fields, where the sentinel is a distinct variant and so
    /// cannot be confused with a stated value.
    #[must_use]
    pub const fn has_unresolved_system_default(&self) -> bool {
        matches!(self.reliability, QoSReliabilityPolicy::SystemDefault)
            || matches!(self.durability, QoSDurabilityPolicy::SystemDefault)
            || matches!(self.history, QoSHistoryPolicy::SystemDefault)
    }
}

/// Publisher trait for sending messages.
///
/// # Threading
///
/// `&self` on `publish_raw` — implementors must allow concurrent
/// publishes from multiple threads. Internal locking (or lock-free
/// queues) is the backend's responsibility.
///
/// # Buffer ownership
///
/// `data` in `publish_raw` is borrowed for the duration of the call.
/// The backend must either send it inline or copy into its own
/// buffer before returning — the slice is invalid after the call.
///
/// # Blocking
///
/// `publish_raw` is expected to be non-blocking on best-effort QoS
/// and bounded-blocking on reliable QoS (waiting for outbound queue
/// space). Backends should *not* block waiting for ack from a
/// matched subscriber.
pub trait Publisher {
    /// Error type for publish operations
    type Error;

    /// Publish a CDR-serialised message.
    ///
    /// Returns once the message has been handed to the transport
    /// (queued or fired-and-forgotten depending on QoS). Does **not**
    /// wait for delivery.
    fn publish_raw(&self, data: &[u8]) -> Result<(), Self::Error>;

    /// Phase 128.F.4 — publish with an opaque attachment block.
    ///
    /// `attachment` rides alongside the payload at the wire layer.
    /// Receivers can read it back via
    /// [`Subscription::take_serialized_with_attachment`].
    ///
    /// Primary use case: cross-RMW bridges stamp a `bridge_origin`
    /// tag (the source backend's RMW name) so a paired return
    /// bridge can deterministically drop echoed frames.
    ///
    /// Default body delegates to [`publish_raw`](Self::publish_raw)
    /// and discards the attachment — backends that do not natively
    /// carry attachments (XRCE today, DDS without a user-data hook)
    /// see no change. Backends with native attachment support
    /// (zenoh-pico's `z_publisher_put_options::attachment`,
    /// Cyclone DDS user-data) override to write the bytes onto the
    /// wire.
    fn publish_raw_with_attachment(
        &self,
        data: &[u8],
        _attachment: &[u8],
    ) -> Result<(), Self::Error> {
        self.publish_raw(data)
    }

    /// Phase 124.E.1 — streamed publish.
    ///
    /// `size_cb` reports the total payload length once; `chunk_cb`
    /// fills the slot in chunks. Saves the per-publisher staging
    /// buffer when the message is large enough to dominate the
    /// device's `.bss`.
    ///
    /// Default body — the **staging-buffer fallback** (124.E.2).
    /// Asks `size_cb` for the total length, fills a stack-allocated
    /// `[u8; NROS_MAX_STREAM_CHUNK]` via `chunk_cb`, then forwards
    /// to `publish_raw`. Returns `Err(BufferTooSmall)` if the total
    /// exceeds the stack cap so the caller can drop back to a
    /// regular `publish_raw` with a heap-sized buffer.
    ///
    /// Concrete backends opt in by overriding to stream straight
    /// into the network buffer (zenoh: write into the zenoh-pico
    /// outbound buffer; XRCE: micro-CDR streaming APIs).
    ///
    /// # Safety
    ///
    /// The caller must ensure `user_ctx` is valid for every invocation of
    /// `size_cb` and `chunk_cb` during this call, and that both callbacks obey
    /// their out-pointer contracts.
    ///
    /// `size_cb` and `chunk_cb` may be called from the same thread
    /// that called `publish_streamed`. Backends MUST NOT defer the
    /// calls past the function return; the caller's `user_ctx`
    /// pointer is only guaranteed valid for the duration of the
    /// call.
    unsafe fn publish_streamed(
        &self,
        size_cb: unsafe extern "C" fn(out_total_len: *mut usize, user_ctx: *mut core::ffi::c_void),
        chunk_cb: unsafe extern "C" fn(
            out_buf: *mut u8,
            cap: usize,
            out_written: *mut usize,
            user_ctx: *mut core::ffi::c_void,
        ),
        user_ctx: *mut core::ffi::c_void,
    ) -> Result<(), Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        /// Default staging-buffer cap. Stack-allocated, so embedded
        /// callers don't pay for it unless they actually invoke this
        /// fallback. 4 KiB matches typical RTPS frag-size +
        /// micro-XRCE message ceilings.
        const STAGE_CAP: usize = 4096;

        let mut total = 0usize;
        // SAFETY: caller's contract on `size_cb` matches our trait
        // doc — fire once with a writable `*mut usize` slot.
        unsafe { size_cb(&mut total as *mut usize, user_ctx) };
        if total > STAGE_CAP {
            return Err(TransportError::BufferTooSmall.into());
        }
        let mut stage = [0u8; STAGE_CAP];
        let mut written_so_far = 0usize;
        while written_so_far < total {
            let mut chunk_written = 0usize;
            let remaining = total - written_so_far;
            // SAFETY: `chunk_cb` writes ≤ `cap` bytes to
            // `out_buf` and reports the count via `out_written`.
            unsafe {
                chunk_cb(
                    stage.as_mut_ptr().add(written_so_far),
                    remaining,
                    &mut chunk_written as *mut usize,
                    user_ctx,
                );
            }
            if chunk_written == 0 {
                // Caller signalled EOF early — treat the partial
                // write as a malformed sequence; reporting it as
                // BufferTooSmall keeps the surface tight without
                // adding a new variant.
                return Err(TransportError::BufferTooSmall.into());
            }
            written_so_far += chunk_written;
        }
        self.publish_raw(&stage[..total])
    }

    /// Publish a typed message (serializes automatically)
    fn publish<M: RosMessage>(&self, msg: &M, buf: &mut [u8]) -> Result<(), Self::Error> {
        use nros_core::CdrWriter;

        let mut writer = CdrWriter::new_with_header(buf).map_err(|_| self.buffer_error())?;
        msg.serialize(&mut writer)
            .map_err(|_| self.serialization_error())?;
        let len = writer.position();
        self.publish_raw(&buf[..len])
    }

    /// Return a buffer-too-small error (implementation specific)
    fn buffer_error(&self) -> Self::Error;

    /// Return a serialization error (implementation specific)
    fn serialization_error(&self) -> Self::Error;

    /// Phase 108 — `true` if the backend can generate this event for
    /// this publisher. Default returns `false`; backends override per
    /// supported event kind.
    ///
    /// Only [`EventKind::LivelinessLost`](crate::event::EventKind::LivelinessLost) and
    /// [`EventKind::OfferedDeadlineMissed`](crate::event::EventKind::OfferedDeadlineMissed) are publisher-side events;
    /// other kinds always return `false` here.
    fn supports_event(&self, _kind: crate::event::EventKind) -> bool {
        false
    }

    /// Phase 108 — register a callback fired when the named status
    /// event occurs. `deadline_ms` applies to
    /// [`EventKind::OfferedDeadlineMissed`](crate::event::EventKind::OfferedDeadlineMissed) only; ignored otherwise.
    /// Default impl returns the backend's "unsupported"-shaped error.
    ///
    /// # Safety
    ///
    /// `cb` and `user_ctx` must remain valid for the entity's
    /// lifetime. Caller (typically `nros-node`'s typed wrapper) is
    /// responsible for keeping the closure / context arena alive.
    unsafe fn register_event_callback(
        &mut self,
        _kind: crate::event::EventKind,
        _deadline_ms: u32,
        _cb: crate::event::EventCallback,
        _user_ctx: *mut core::ffi::c_void,
    ) -> Result<(), Self::Error> {
        Err(self.unsupported_event_error())
    }

    /// Phase 108 — backend's error variant for "this event kind is
    /// not supported." Default impl reuses `serialization_error()`
    /// since most backends share an `Unsupported` variant; backends
    /// override if they have a distinct `Unsupported` mapping.
    fn unsupported_event_error(&self) -> Self::Error {
        self.serialization_error()
    }

    /// Phase 109 — assert this publisher's liveliness manually.
    /// Required for publishers configured with
    /// `QoSLivelinessPolicy::ManualByTopic`. No-op for other
    /// liveliness kinds. Default impl returns `Ok(())` (no-op);
    /// backends override when they implement manual liveliness.
    fn assert_liveliness(&self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Subscription trait for receiving messages.
///
/// # Threading
///
/// `&mut self` on `take_serialized` — the executor takes exclusive
/// ownership of the subscriber for the duration of a receive. A
/// backend that wants to allow concurrent receives must split into
/// per-thread sub-handles internally.
///
/// # Buffer ownership
///
/// `buf` is caller-owned. The implementation copies the next ready
/// message into `buf` and returns the byte count. The caller may
/// re-use or drop `buf` immediately after the call.
///
/// # Blocking
///
/// `take_serialized` is **non-blocking**: returns `Ok(None)` (or
/// equivalent for backends that map empty into a zero-length read)
/// when no message is ready. Use [`Session::drive_io`] to wait for
/// data; never sleep inside `take_serialized`.
pub trait Subscription {
    /// Error type for receive operations
    type Error;

    /// Check if data is available without consuming it.
    ///
    /// Non-destructive — does not advance the receive cursor.
    /// Conservative default returns `true` (always assume data may
    /// be available); backends should override with a real check
    /// to avoid spurious receive attempts.
    fn has_data(&self) -> bool {
        true
    }

    /// Try to receive one message into `buf`.
    ///
    /// Non-blocking. On success returns `Ok(Some(len))` where `len`
    /// is the byte count written into `buf[..len]`. Returns
    /// `Ok(None)` if no message is ready. If `buf` is too small the
    /// backend may either truncate (and document it) or return an
    /// error (preferred).
    fn take_serialized(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Self::Error>;

    /// Phase 128.F.4 — receive with attachment bytes alongside the
    /// payload.
    ///
    /// On success returns `Ok(Some((payload_len, attachment_len)))`
    /// with the payload written into `buf[..payload_len]` and the
    /// attachment (if any) written into
    /// `att_buf[..attachment_len]`. `attachment_len == 0` means the
    /// incoming sample carried no attachment.
    ///
    /// Default body falls back to [`take_serialized`](Self::take_serialized)
    /// and reports a 0-length attachment. Backends with native
    /// attachment support override to populate `att_buf`. Cross-RMW
    /// bridges use the attachment to read the `bridge_origin` tag
    /// stamped by the sending side.
    fn take_serialized_with_attachment(
        &mut self,
        buf: &mut [u8],
        _att_buf: &mut [u8],
    ) -> Result<Option<(usize, usize)>, Self::Error> {
        match self.take_serialized(buf)? {
            Some(len) => Ok(Some((len, 0))),
            None => Ok(None),
        }
    }

    /// Phase 124.D.1 — burst-take.
    ///
    /// Drain up to `max_msgs` queued samples into the contiguous
    /// `buf` block in one call, with the i-th sample at
    /// `buf[i * per_msg_cap .. i * per_msg_cap + out_lens[i]]`.
    /// Returns the number of messages actually delivered. Partial
    /// drains MUST report the count, not error out.
    ///
    /// Default body loop-drives `take_serialized` so callers can
    /// commit to the batched API regardless of backend support.
    /// Concrete backends opt in by overriding with a native batch
    /// take (zenoh queue drain, `dds_take(max_samples)`).
    fn take_sequence(
        &mut self,
        buf: &mut [u8],
        per_msg_cap: usize,
        max_msgs: usize,
        out_lens: &mut [usize],
    ) -> Result<usize, Self::Error> {
        if per_msg_cap == 0 || max_msgs == 0 {
            return Ok(0);
        }
        let limit = max_msgs.min(out_lens.len());
        let mut count = 0;
        for i in 0..limit {
            let slot = &mut buf[i * per_msg_cap..(i + 1) * per_msg_cap];
            // Issue 0971 — NOT `take_serialized(slot)?`. The `?` propagates the
            // error and DISCARDS `count`, which is precisely what the doc
            // comment above forbids ("Partial drains MUST report the count, not
            // error out") and what `c3af8c1d1` removed from the two concrete
            // implementations. A backend that does not override this body got
            // the original defect back through the default.
            //
            // The shape is the one that fix established: a drain that has
            // already taken messages reports the COUNT, and the error is
            // delivered by the NEXT call — the caller sees every message it was
            // handed, then the reason the drain stopped. With nothing taken
            // there is no count to protect, so the error goes out immediately.
            //
            // Parking is per-implementation state, which a default body has no
            // place to keep. So it does the half it can do correctly: it
            // returns the partial count and lets the error surface on the
            // caller's next `take_serialized`, which is where an unconsumed
            // backend error still sits.
            match self.take_serialized(slot) {
                Ok(Some(len)) => {
                    out_lens[i] = len;
                    count += 1;
                }
                Ok(None) => break,
                Err(e) => {
                    if count == 0 {
                        return Err(e);
                    }
                    break;
                }
            }
        }
        Ok(count)
    }

    /// Try to receive a typed message (non-blocking)
    fn take<M: RosMessage>(&mut self, buf: &mut [u8]) -> Result<Option<M>, Self::Error> {
        use nros_core::CdrReader;

        match self.take_serialized(buf)? {
            Some(len) => {
                let mut reader = CdrReader::new_with_header(&buf[..len])
                    .map_err(|_| self.deserialization_error())?;
                let msg = M::deserialize(&mut reader).map_err(|_| self.deserialization_error())?;
                Ok(Some(msg))
            }
            None => Ok(None),
        }
    }

    /// Process the received message in-place without copying.
    ///
    /// Calls `f` with a reference to the raw CDR bytes in the subscriber's
    /// internal receive buffer, avoiding a copy into a caller-provided buffer.
    /// While `f` executes the buffer is exclusively borrowed — any messages
    /// arriving from the transport during that time are dropped to prevent
    /// data races.
    ///
    /// Returns `Ok(true)` if a message was available and `f` was called,
    /// `Ok(false)` if no message was available.
    ///
    /// **Default body**: returns `Err(MessageTooLarge)` — the old default
    /// silently truncated anything larger than 1 KB into a stack buffer,
    /// which broke large messages with no diagnostic. Backends must
    /// override this with a real zero-copy path if they advertise support
    /// for `process_raw_in_place`; callers that hit the default should
    /// use `take_serialized` with a caller-sized buffer instead.
    fn process_raw_in_place(&mut self, f: impl FnOnce(&[u8])) -> Result<bool, Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        let _ = f;
        Err(TransportError::MessageTooLarge.into())
    }

    /// Whether this backend implements the in-place dispatch methods
    /// ([`process_raw_in_place`](Subscription::process_raw_in_place) /
    /// [`process_raw_in_place_with_info`](Subscription::process_raw_in_place_with_info))
    /// with a real zero-copy borrow.
    ///
    /// The executor consults this at subscription registration to choose the
    /// **in-place** arena dispatch (borrow + deserialize from the backend slot, no
    /// arena buffer) over the **buffered** dispatch (copy into an arena buffer
    /// first). Backends that leave the in-place methods at their unsupported
    /// default return `false` (the default) and keep the buffered path. (RFC-0038,
    /// Phase 231 Wave 0.2.)
    fn supports_process_in_place(&self) -> bool {
        false
    }

    /// In-place processing variant that also surfaces publisher metadata.
    ///
    /// Same borrow contract as
    /// [`process_raw_in_place`](Subscription::process_raw_in_place): `f` receives
    /// the raw CDR bytes plus the parsed [`MessageInfo`](nros_core::MessageInfo)
    /// — the co-located attachment (publisher GID / sequence / source timestamp),
    /// or `None` when no attachment was present — for the duration of the call;
    /// the slot is released after `f` returns. `Ok(true)` = a message was
    /// available and `f` was called; `Ok(false)` = none ready.
    ///
    /// **Default body**: returns the unsupported error (mirrors
    /// `process_raw_in_place`). Backends that advertise in-place support override
    /// this with a real zero-copy path; callers that hit the default should use
    /// the buffered [`take_serialized_with_info`](Subscription::take_serialized_with_info)
    /// path instead. (RFC-0038, Phase 231 Wave 0.1.)
    fn process_raw_in_place_with_info(
        &mut self,
        f: impl FnOnce(&[u8], Option<nros_core::MessageInfo>),
    ) -> Result<bool, Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        let _ = f;
        Err(TransportError::MessageTooLarge.into())
    }

    /// Try to receive raw data along with publisher metadata.
    ///
    /// When available, [`MessageInfo`](nros_core::MessageInfo) contains
    /// the publisher's GID (Global Identifier) and source timestamp,
    /// extracted from a transport-level attachment on the incoming message.
    ///
    /// Returns `Ok(Some((len, info)))` if data is available, where:
    /// - `len` is the number of bytes written to the buffer
    /// - `info` is the parsed publisher metadata (if attachment was present)
    ///
    /// Default: delegates to [`take_serialized`](Subscription::take_serialized) with no info.
    fn take_serialized_with_info(
        &mut self,
        buf: &mut [u8],
    ) -> Result<Option<(usize, Option<nros_core::MessageInfo>)>, Self::Error> {
        self.take_serialized(buf)
            .map(|opt| opt.map(|len| (len, None)))
    }

    /// Try to receive raw data with E2E safety validation (CRC + sequence tracking).
    ///
    /// Returns `Ok(Some((len, status)))` if data is available, where:
    /// - `len` is the number of bytes written to the buffer
    /// - `status` is the integrity validation result
    ///
    /// Default: delegates to `take_serialized` with no CRC info.
    #[cfg(feature = "safety-e2e")]
    fn take_validated(
        &mut self,
        buf: &mut [u8],
    ) -> Result<Option<(usize, crate::IntegrityStatus)>, Self::Error> {
        self.take_serialized(buf).map(|opt| {
            opt.map(|len| {
                (
                    len,
                    crate::IntegrityStatus {
                        gap: 0,
                        duplicate: false,
                        crc_valid: None,
                    },
                )
            })
        })
    }

    /// Register an async waker to be notified when data arrives.
    ///
    /// Called from `Future::poll()` implementations to store the waker.
    /// The transport backend calls `waker.wake()` from its receive callback
    /// when new data is available, enabling event-driven async without
    /// busy-polling.
    ///
    /// Default: no-op (backends that don't support waking simply ignore this).
    fn register_waker(&self, _waker: &core::task::Waker) {}

    /// Return a deserialization error (implementation specific)
    fn deserialization_error(&self) -> Self::Error;

    /// Phase 108 — `true` if the backend can generate this event for
    /// this subscriber. Default returns `false`; backends override per
    /// supported event kind.
    ///
    /// Subscription-side event kinds:
    /// [`EventKind::LivelinessChanged`](crate::event::EventKind::LivelinessChanged),
    /// [`EventKind::RequestedDeadlineMissed`](crate::event::EventKind::RequestedDeadlineMissed),
    /// [`EventKind::MessageLost`](crate::event::EventKind::MessageLost).
    /// Publisher kinds always return `false` here.
    fn supports_event(&self, _kind: crate::event::EventKind) -> bool {
        false
    }

    /// Phase 108 — register a callback fired when the named status
    /// event occurs. `deadline_ms` applies to
    /// [`EventKind::RequestedDeadlineMissed`](crate::event::EventKind::RequestedDeadlineMissed) only; ignored otherwise.
    /// Default impl returns the backend's "unsupported"-shaped error.
    ///
    /// # Safety
    ///
    /// `cb` and `user_ctx` must remain valid for the entity's
    /// lifetime. Caller (typically `nros-node`'s typed wrapper) is
    /// responsible for keeping the closure / context arena alive.
    unsafe fn register_event_callback(
        &mut self,
        _kind: crate::event::EventKind,
        _deadline_ms: u32,
        _cb: crate::event::EventCallback,
        _user_ctx: *mut core::ffi::c_void,
    ) -> Result<(), Self::Error> {
        Err(self.unsupported_event_error())
    }

    /// Phase 108 — backend's error variant for "this event kind is
    /// not supported." Default reuses `deserialization_error()` for
    /// backends that don't have a distinct `Unsupported` mapping.
    fn unsupported_event_error(&self) -> Self::Error {
        self.deserialization_error()
    }
}

/// Service request from a client
pub struct ServiceRequest<'a> {
    /// Raw request data (CDR encoded)
    pub data: &'a [u8],
    /// Sequence number for request/response matching
    pub sequence_number: i64,
}

// ============================================================================
// Phase 99 — zero-copy raw API: SlotLending / SlotBorrowing
// ============================================================================
//
// Backends that can lend a slot directly into their outbound buffer
// (zenoh-pico w/ unstable-zenoh-api, XRCE-DDS via uxr_prepare_output_stream,
// full DDS w/ SHM transport) implement these traits. Backends that cannot
// (uORB, default zenoh-pico) do NOT impl them — `EmbeddedRawPublisher` then
// falls back to its per-publisher arena and memcpys at commit time. Both
// paths land at `Publisher::publish_raw` for the actual wire write; only
// the user-side copy is eliminated when lending is available.
//
// Selection is **compile-time** via the `lending` Cargo feature. Each
// backend crate forwards its own `lending` feature to `nros-rmw/lending`
// when it can satisfy the trait. nros-node's `rmw-lending` aggregates.
// User opting `nros/rmw-lending` w/ a non-lending backend (e.g. uORB)
// gets a clear compile error from the unsatisfied trait bound on the
// concrete `RmwPublisher`.

/// Backend can lend a writable slot into its outbound buffer.
///
/// The returned slot's lifetime is tied to `&self`; user fills it in
/// place, then calls [`commit_slot`](Self::commit_slot) to publish.
/// Dropping the slot without commit is a no-op (slot returned to free
/// pool); concurrent loan attempts that would exceed backend capacity
/// return [`TransportError::WouldBlock`] — never block.
#[cfg(feature = "lending")]
pub trait SlotLending: Publisher {
    /// Backend-owned writable slot. Holds a `&'a mut [u8]` and any
    /// state needed for commit_slot.
    type Slot<'a>: AsMut<[u8]> + 'a
    where
        Self: 'a;

    /// Reserve a writable slot of `len` bytes from the backend's
    /// outbound buffer. Returns `Ok(None)` if the backend has no slot
    /// available (full); never blocks.
    fn try_lend_slot(&self, len: usize) -> Result<Option<Self::Slot<'_>>, Self::Error>;

    /// Commit a previously-lent slot. Consumes the slot and triggers
    /// the actual wire write. Returns `Err` on backend send failure;
    /// the slot's bytes are lost in that case (caller must re-lend +
    /// re-fill to retry).
    fn commit_slot(&self, slot: Self::Slot<'_>) -> Result<(), Self::Error>;
}

/// Backend can lend a read-only view into its receive buffer.
///
/// The returned view's lifetime is tied to `&mut self` (subscriber-
/// exclusive); dropping the view releases any backend lock and lets
/// the next message advance into the buffer.
#[cfg(feature = "lending")]
pub trait SlotBorrowing: Subscription {
    /// Backend-owned read-only view. Holds a `&'a [u8]` and any state
    /// needed to release the borrow on Drop.
    type View<'a>: AsRef<[u8]> + 'a
    where
        Self: 'a;

    /// Try to borrow the next available message in place. Returns
    /// `Ok(None)` if no message is ready; never blocks.
    fn try_borrow(&mut self) -> Result<Option<Self::View<'_>>, Self::Error>;
}

/// Service server trait for handling requests.
///
/// # Threading
///
/// `&mut self` on `take_request` and `send_response` — the executor
/// owns the server while a request is being handled. Handler bodies
/// run synchronously on the executor thread; long handlers should
/// dispatch work to a worker queue and reply later via the recorded
/// `sequence_number`.
///
/// # Calling pattern
///
/// 1. Executor calls `take_request(buf)`.
/// 2. If `Some(req)` returned, decode, run handler, encode reply.
/// 3. `send_response(req.sequence_number, &reply_buf)`.
///
/// `sequence_number` is the canonical request → reply correlation
/// token; backends derive it from the wire-level metadata (zenoh
/// query id, DDS sample identity).
pub trait ServiceTrait {
    /// Error type for service operations
    type Error;

    /// Check if a request is available without consuming it.
    ///
    /// Non-destructive. Default returns `true` (always assume one
    /// may be available); backends should override with a real
    /// check.
    fn has_request(&self) -> bool {
        true
    }

    /// Phase 122.3.c.6.e — register a `Waker` for event-driven
    /// service servers. Mirrors the matching method on
    /// `SubscriberTrait` / `ClientTrait`. Backends that
    /// surface incoming-request notifications wake `waker` when
    /// `has_request()` flips true. Default: no-op (backends without
    /// wake support ignore — caller falls back to polling).
    fn register_waker(&self, _waker: &core::task::Waker) {}

    /// Try to receive a service request into `buf` (non-blocking).
    ///
    /// On success returns a `ServiceRequest` that borrows from
    /// `buf`. The borrow is released when the returned struct is
    /// dropped — typically before `send_response` is called, since
    /// `send_response` takes `&mut self`.
    fn take_request<'a>(
        &mut self,
        buf: &'a mut [u8],
    ) -> Result<Option<ServiceRequest<'a>>, Self::Error>;

    /// Send a reply for the given sequence number. Non-blocking
    /// from the application's perspective; the backend may queue
    /// the reply for transport-level transmission.
    fn send_response(&mut self, sequence_number: i64, data: &[u8]) -> Result<(), Self::Error>;

    /// Handle a service request with typed messages
    fn handle_request<S: RosService>(
        &mut self,
        req_buf: &mut [u8],
        reply_buf: &mut [u8],
        handler: impl FnOnce(&S::Request) -> S::Reply,
    ) -> Result<bool, Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        use nros_core::{CdrReader, CdrWriter};

        // First, try to receive a request and extract necessary data.
        // Capture the data slice's offset within `req_buf` so we can
        // re-borrow it after the `ServiceRequest` (which holds a
        // borrow into `req_buf`) is dropped. Some backends prepend a
        // header (DDS: 8-byte sequence number) and place the CDR
        // payload at a non-zero offset in the buffer; others (zenoh)
        // put it at offset 0. Reading from offset 0 unconditionally
        // would feed the prefix bytes to the CDR deserializer and
        // silently corrupt the request.
        let buf_start = req_buf.as_ptr() as usize;
        let (data_offset, data_len, sequence_number) = match self.take_request(req_buf)? {
            Some(request) => {
                let offset = (request.data.as_ptr() as usize).saturating_sub(buf_start);
                (offset, request.data.len(), request.sequence_number)
            }
            None => return Ok(false),
        };

        // Deserialize request from the captured offset.
        let mut reader = CdrReader::new_with_header(&req_buf[data_offset..data_offset + data_len])
            .map_err(|_| TransportError::DeserializationError)?;
        let req = S::Request::deserialize(&mut reader)
            .map_err(|_| TransportError::DeserializationError)?;

        // Call handler
        let reply = handler(&req);

        // Serialize reply
        let mut writer =
            CdrWriter::new_with_header(reply_buf).map_err(|_| TransportError::BufferTooSmall)?;
        reply
            .serialize(&mut writer)
            .map_err(|_| TransportError::SerializationError)?;
        let len = writer.position();

        // Send reply (now we can borrow self mutably again)
        self.send_response(sequence_number, &reply_buf[..len])?;
        Ok(true)
    }

    /// Handle a service request where the handler returns `Box<S::Reply>`
    ///
    /// Identical to `handle_request` but the handler returns a heap-allocated reply.
    /// This is needed for services with large response types (e.g., parameter services
    /// where `Vec<ParameterValue, 64>` is ~1MB+) that would overflow the stack.
    #[cfg(feature = "alloc")]
    fn handle_request_boxed<S: RosService>(
        &mut self,
        req_buf: &mut [u8],
        reply_buf: &mut [u8],
        handler: impl FnOnce(&S::Request) -> alloc::boxed::Box<S::Reply>,
    ) -> Result<bool, Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        use nros_core::{CdrReader, CdrWriter};

        let buf_start = req_buf.as_ptr() as usize;
        let (data_offset, data_len, sequence_number) = match self.take_request(req_buf)? {
            Some(request) => {
                let offset = (request.data.as_ptr() as usize).saturating_sub(buf_start);
                (offset, request.data.len(), request.sequence_number)
            }
            None => return Ok(false),
        };

        let mut reader = CdrReader::new_with_header(&req_buf[data_offset..data_offset + data_len])
            .map_err(|_| TransportError::DeserializationError)?;
        let req = S::Request::deserialize(&mut reader)
            .map_err(|_| TransportError::DeserializationError)?;

        let reply = handler(&req);

        let mut writer =
            CdrWriter::new_with_header(reply_buf).map_err(|_| TransportError::BufferTooSmall)?;
        reply
            .serialize(&mut writer)
            .map_err(|_| TransportError::SerializationError)?;
        let len = writer.position();

        self.send_response(sequence_number, &reply_buf[..len])?;
        Ok(true)
    }

    /// Handle one request by STREAMING: the handler reads fields off the wire
    /// and writes the reply's fields straight back, so neither the request nor
    /// the reply is ever materialised as a value.
    ///
    /// phase-382 W1'. `handle_request_boxed` above boxes the REPLY because the
    /// parameter responses are enormous — `GetParametersResponse` measures
    /// 1,176,072 bytes. It does not box the REQUEST, which is deserialized by
    /// value into a stack local one line above the handler, and
    /// `SetParametersRequest` measures **1,192,968 bytes**. So every
    /// `ros2 param set` against a node put a 1.19 MB local on the calling
    /// task's stack — larger than the reply the boxing exists for, on every
    /// platform, with `param-services` live on Zephyr.
    ///
    /// Streaming removes both, and removes the `alloc` requirement with them:
    /// the whole value is never needed, because serialisation happens on the
    /// line after construction. Three things make it safe rather than clever:
    ///
    /// * **No `rcl_interfaces` message uses a DHEADER** — every `Serialize` impl
    ///   is plain sequential CDR, so hand-written field writes are byte-identical
    ///   to the generated ones. (If a future message gains XCDR2 extensibility
    ///   this stops being true for THAT message; see RFC-0055.)
    /// * `req_buf` and `reply_buf` are disjoint, so a handler can hold the
    ///   reader and the writer at once.
    /// * `CdrReader::read_string` borrows out of `req_buf` rather than copying,
    ///   so a handler can look a name up without a buffer of its own.
    ///
    /// The cost is that the hand-written writes can drift from the generated
    /// `Serialize`. Guard it with a round-trip test that deserialises the
    /// streamed bytes back into the generated type — the by-value handler makes
    /// a good test-only oracle.
    ///
    /// No `alloc`, deliberately: this is the seam that lets `param-services` and
    /// `lifecycle-services` build without it.
    fn handle_request_raw(
        &mut self,
        req_buf: &mut [u8],
        reply_buf: &mut [u8],
        handler: impl FnOnce(
            &mut nros_core::CdrReader<'_>,
            &mut nros_core::CdrWriter<'_>,
        ) -> Result<(), TransportError>,
    ) -> Result<bool, Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        use nros_core::{CdrReader, CdrWriter};

        let buf_start = req_buf.as_ptr() as usize;
        let (data_offset, data_len, sequence_number) = match self.take_request(req_buf)? {
            Some(request) => {
                let offset = (request.data.as_ptr() as usize).saturating_sub(buf_start);
                (offset, request.data.len(), request.sequence_number)
            }
            None => return Ok(false),
        };

        // Split the borrow so the reader (over `req_buf`) and the writer (over
        // `reply_buf`) can be live at the same time. They are separate
        // parameters, so this is disjoint by construction.
        let mut reader = CdrReader::new_with_header(&req_buf[data_offset..data_offset + data_len])
            .map_err(|_| TransportError::DeserializationError)?;
        let mut writer =
            CdrWriter::new_with_header(reply_buf).map_err(|_| TransportError::BufferTooSmall)?;

        handler(&mut reader, &mut writer)?;
        let len = writer.position();

        self.send_response(sequence_number, &reply_buf[..len])?;
        Ok(true)
    }
}

/// Service client trait for sending requests.
///
/// # Threading
///
/// `&mut self` on every method — the client is single-owner. For
/// fan-out request patterns, create one client per worker thread.
///
/// # Calling pattern
///
/// All in-tree backends route blocking waits through the executor:
///
/// 1. `send_request_raw(buf)` — non-blocking; returns once the
///    request is queued for transmission.
/// 2. The executor's `drive_io` runs.
/// 3. `take_response_raw(buf)` — non-blocking; returns
///    `Ok(Some(len))` when the reply is back.
///
/// Phase-301 (issue 0240): the deprecated blocking `call_raw` path is
/// DELETED — `send_request_raw` + `take_response_raw` is the one
/// request/reply path, and both are required for service-capable
/// backends.
pub trait ClientTrait {
    /// Error type for service operations
    type Error;

    /// Send a service request without waiting for a reply (non-blocking).
    ///
    /// Returns the SEQUENCE ID the backend assigned. The caller must
    /// subsequently poll [`take_response_raw`](Self::take_response_raw) and
    /// match that id against the one the reply carries.
    ///
    /// Issue 0778 — this returned `()` until 2026-08-25, and every backend
    /// computed an id and discarded it. With nothing to correlate by, a client
    /// with two calls outstanding could not tell the replies apart, so each
    /// backend picked a policy: cyclonedds abandoned the older request, zenoh
    /// took the first reply. Both are wrong for `send_goal` and
    /// `SetParameters`, which travel this path and are not idempotent.
    fn send_request_raw(&mut self, request: &[u8]) -> Result<i64, Self::Error>;

    /// Poll for a reply (non-blocking).
    ///
    /// Returns `Ok(Some((len, sequence_id)))` when a reply has arrived,
    /// `Ok(None)` if not yet available, or `Err` on failure. The
    /// `sequence_id` is the one [`send_request_raw`](Self::send_request_raw)
    /// returned for the request this answers.
    ///
    /// It used to say "a reply to the MOST RECENTLY sent request", which was
    /// the single-outstanding-call assumption written into the contract.
    fn take_response_raw(
        &mut self,
        reply_buf: &mut [u8],
    ) -> Result<Option<(usize, i64)>, Self::Error>;

    /// Send a typed service request without waiting for a reply (non-blocking).
    ///
    /// Serializes the request into `req_buf` and calls [`send_request_raw`](Self::send_request_raw).
    fn send_request<S: RosService>(
        &mut self,
        request: &S::Request,
        req_buf: &mut [u8],
    ) -> Result<(), Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        use nros_core::CdrWriter;

        let mut writer =
            CdrWriter::new_with_header(req_buf).map_err(|_| TransportError::BufferTooSmall)?;
        request
            .serialize(&mut writer)
            .map_err(|_| TransportError::SerializationError)?;
        let req_len = writer.position();

        self.send_request_raw(&req_buf[..req_len]).map(|_seq| ())
    }

    /// Poll for a typed reply to the most recently sent request (non-blocking).
    ///
    /// Calls [`take_response_raw`](Self::take_response_raw) and deserializes if available.
    fn take_response<S: RosService>(
        &mut self,
        reply_buf: &mut [u8],
    ) -> Result<Option<S::Reply>, Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        use nros_core::CdrReader;

        match self.take_response_raw(reply_buf)? {
            Some((len, _seq)) => {
                let mut reader = CdrReader::new_with_header(&reply_buf[..len])
                    .map_err(|_| TransportError::DeserializationError)?;
                let reply = S::Reply::deserialize(&mut reader)
                    .map_err(|_| TransportError::DeserializationError)?;
                Ok(Some(reply))
            }
            None => Ok(None),
        }
    }

    /// Register an async waker to be notified when a reply arrives.
    ///
    /// Called from `Future::poll()` implementations to store the waker.
    /// The transport backend calls `waker.wake()` from its reply callback
    /// when a response is available, enabling event-driven async without
    /// busy-polling.
    ///
    /// Default: no-op (backends that don't support waking simply ignore this).
    fn register_waker(&self, _waker: &core::task::Waker) {}

    /// Begin a server-discovery query on this client (non-blocking).
    ///
    /// Models `rclcpp::ClientBase::wait_for_service` machinery: the backend
    /// fires off a discovery probe (typically a Zenoh liveliness query
    /// against the matching server's wildcarded liveliness keyexpr) and
    /// the caller polls [`poll_server_discovery`](Self::poll_server_discovery)
    /// to collect the result.
    ///
    /// Default impl: no-op success. Backends without a discovery channel
    /// (or those that always assume the server is reachable) can leave
    /// this default and have `poll_server_discovery` return
    /// `Ok(Some(true))` immediately.
    fn start_server_discovery(&mut self, _timeout_ms: u32) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Poll an in-flight server-discovery query.
    ///
    /// - `Ok(Some(true))` — at least one matching server has reported
    ///   back; safe to send the first request.
    /// - `Ok(Some(false))` — discovery query finished without finding
    ///   any matching server (timeout / no-replies).
    /// - `Ok(None)` — query still in flight.
    /// - `Err(_)` — transport-level failure unrelated to server presence.
    ///
    /// Default impl: returns `Ok(Some(true))` (i.e., "server is always
    /// assumed reachable"). The Zenoh backend overrides this with a
    /// liveliness-token check.
    fn poll_server_discovery(&mut self) -> Result<Option<bool>, Self::Error> {
        Ok(Some(true))
    }

    /// Whether a matching service server is currently discoverable.
    ///
    /// Mirrors `rclcpp::ClientBase::service_is_ready` — the NAME is upstream's.
    /// The SHAPE is `rcl`'s: `rcl_service_server_is_available(node, client,
    /// bool *is_available)` returns `RCL_RET_OK` "if the check was made
    /// successfully (regardless of the service readiness)", i.e. the return
    /// code says whether the CHECK worked and the out-param carries the ANSWER.
    /// rclcpp collapses that to a bare `bool` and moves the error to
    /// exceptions; RFC-0018 forbids exceptions, so `Result<bool, _>` is how the
    /// same contract is expressed here (phase-379 W6, RFC-0036).
    ///
    /// Returns `Ok(true)` if at least one matching server has been
    /// discovered, `Ok(false)` if none yet, or `Err(_)` if the
    /// backend cannot answer (e.g. XRCE — micro-XRCE-DDS-Client has
    /// no participant enumeration).
    ///
    /// This USED to be contrasted with an `is_server_ready` that collapsed
    /// "don't know" and "no server" into one `false`. That method is gone
    /// (issue 1008, W6 decision 2): its trait DEFAULT was `true`, only zenoh
    /// overrode it, and `wait_for_service`'s fast path read it — so on every
    /// cffi-reached backend the wait returned `Ok(true)` immediately, without
    /// probing, whether or not any server existed. The two-channel form here is
    /// what replaced it; there is no longer a collapsing sibling to contrast
    /// with. (The rustdoc link to the deleted method is what kept `just book`
    /// red from 2026-09-03 until phase-417 W-B4 tripped over it.)
    ///
    /// User-facing surface: `Client<S>::server_available()` in Rust,
    /// `nros_client_server_available()` in C/C++. Clients use this
    /// to gate the first request so a startup-ordering race
    /// (client opens before server's discovery announcement lands)
    /// doesn't surface as a request-side timeout.
    ///
    /// Default impl: `Err(TransportError::Unsupported)` — backends
    /// that support graph introspection (zenoh queryable interest,
    /// DDS built-in topic readers) opt in by overriding.
    fn service_is_ready(&self) -> Result<bool, Self::Error>
    where
        Self::Error: From<TransportError>,
    {
        Err(TransportError::Unsupported.into())
    }
}

/// Transport backend trait (legacy).
///
/// Use [`Rmw`] for new code. This trait is retained for backward compatibility
/// with existing code that uses [`TransportConfig`] directly.
pub trait Transport {
    /// Error type for this transport
    type Error;
    /// Session type for this transport
    type Session: Session;

    /// Open a new session with the given configuration
    fn open(config: &TransportConfig) -> Result<Self::Session, Self::Error>;
}

/// Factory trait for compile-time middleware selection.
///
/// Embedded crates select a backend via feature flag:
/// ```rust,ignore
/// #[cfg(feature = "rmw-cffi")]
/// type DefaultRmw = nros_rmw_cffi::CffiRmw;
/// ```
///
/// Each backend provides its own `Rmw` implementation that bridges
/// from the middleware-agnostic [`RmwConfig`] to backend-specific
/// initialization.
///
/// Phase 84.E2: `open` consumes `self`. Backends carry their own
/// configuration (agent addresses, serial ports, TLS CA slots)
/// inside the factory value and hand that over to the session at
/// `open` time. All in-repo backends also implement
/// [`Default`]; most callers spell this as
/// `BackendRmw::default().open(&config)`.
pub trait Rmw {
    /// Session type returned by [`open`](Rmw::open)
    type Session: Session;
    /// Error type for session creation
    type Error: core::fmt::Debug;

    /// Open a new middleware session with the given configuration.
    ///
    /// The backend maps [`RmwConfig`] fields to its own connection
    /// parameters (e.g., zenoh locator and session mode, XRCE-DDS
    /// agent address). Any backend-specific pre-open state stored
    /// on `self` (e.g. configured agent IP / port) is moved into the
    /// returned `Session`.
    fn open(self, config: &RmwConfig) -> Result<Self::Session, Self::Error>;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Issue 0971 — the DEFAULT `take_sequence` body must report a partial
    /// count rather than discard it, which is what its own doc comment says and
    /// what `?` did not do.
    ///
    /// This is the default's negative control: a backend that takes two
    /// messages and then errors must hand the caller those two. Before the fix
    /// the `?` threw them away and the caller could not tell two messages had
    /// been consumed — they were gone from the queue and absent from the
    /// result.
    mod take_sequence_default {
        use super::*;

        #[derive(Debug, PartialEq)]
        struct Boom;

        /// Yields `n` one-byte messages, then errors forever.
        struct ThenErrors {
            left: usize,
        }

        impl Subscription for ThenErrors {
            type Error = Boom;

            fn take_serialized(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Self::Error> {
                if self.left == 0 {
                    return Err(Boom);
                }
                self.left -= 1;
                buf[0] = 0xAB;
                Ok(Some(1))
            }

            fn deserialization_error(&self) -> Self::Error {
                Boom
            }
        }

        #[test]
        fn a_partial_drain_reports_its_count_instead_of_the_error() {
            let mut sub = ThenErrors { left: 2 };
            let mut buf = [0u8; 4 * 8];
            let mut lens = [0usize; 4];

            // Two messages are available, the third call errors.
            let got = sub.take_sequence(&mut buf, 8, 4, &mut lens);

            assert_eq!(
                got,
                Ok(2),
                "a drain that took messages must report them; `?` discarded the \
                 count and the caller lost two consumed messages"
            );
            assert_eq!(&lens[..2], &[1, 1]);
        }

        #[test]
        fn an_error_with_nothing_taken_is_returned_immediately() {
            let mut sub = ThenErrors { left: 0 };
            let mut buf = [0u8; 4 * 8];
            let mut lens = [0usize; 4];

            // No count to protect, so the caller should see the error itself
            // rather than an ambiguous `Ok(0)`.
            assert_eq!(sub.take_sequence(&mut buf, 8, 4, &mut lens), Err(Boom));
        }
    }

    #[test]
    fn test_topic_info() {
        let topic = TopicInfo::new("/chatter", "std_msgs::msg::dds_::String_", "abc123");
        assert_eq!(topic.name, "/chatter");
        assert_eq!(topic.domain_id, 0);
    }

    #[test]
    fn qos_apply_overrides_matches_topic_and_role() {
        // Default is Reliable / Volatile / KeepLast(10).
        static OVERRIDES: &[QoSOverride] = &[
            QoSOverride {
                topic: "/chatter",
                role: QoSOverrideRole::Publisher,
                value: QoSOverrideValue::Reliability(QoSReliabilityPolicy::BestEffort),
            },
            QoSOverride {
                topic: "/chatter",
                role: QoSOverrideRole::Publisher,
                value: QoSOverrideValue::Depth(5),
            },
            QoSOverride {
                topic: "/scan",
                role: QoSOverrideRole::Subscription,
                value: QoSOverrideValue::Durability(QoSDurabilityPolicy::TransientLocal),
            },
        ];

        // Matching topic + publisher role → reliability + depth applied.
        let pub_qos = QoSProfile::default().apply_overrides(
            "/chatter",
            QoSOverrideRole::Publisher,
            OVERRIDES,
        );
        assert_eq!(pub_qos.reliability, QoSReliabilityPolicy::BestEffort);
        assert_eq!(pub_qos.depth, 5);
        assert_eq!(pub_qos.durability, QoSDurabilityPolicy::Volatile); // untouched

        // Same topic but subscription role → publisher overrides DON'T apply;
        // the /scan override is for a different topic → also no change.
        let sub_qos = QoSProfile::default().apply_overrides(
            "/chatter",
            QoSOverrideRole::Subscription,
            OVERRIDES,
        );
        assert_eq!(sub_qos, QoSProfile::default());

        // The /scan subscription override applies only to /scan+subscription.
        let scan_qos = QoSProfile::default().apply_overrides(
            "/scan",
            QoSOverrideRole::Subscription,
            OVERRIDES,
        );
        assert_eq!(scan_qos.durability, QoSDurabilityPolicy::TransientLocal);

        // Empty table → identity (the zero-override fast path).
        assert_eq!(
            QoSProfile::default().apply_overrides("/x", QoSOverrideRole::Publisher, &[]),
            QoSProfile::default()
        );
    }

    #[test]
    fn test_qos_defaults() {
        let qos = QoSProfile::default();
        assert_eq!(qos.reliability, QoSReliabilityPolicy::Reliable);
    }

    #[test]
    fn test_action_info() {
        let action = ActionInfo::new(
            "/fibonacci",
            "example_interfaces::action::dds_::Fibonacci_",
            "abc123",
        );
        assert_eq!(action.name, "/fibonacci");
        assert_eq!(action.domain_id, 0);
    }

    #[test]
    fn test_action_info_with_domain() {
        let action = ActionInfo::new(
            "/fibonacci",
            "example_interfaces::action::dds_::Fibonacci_",
            "abc123",
        )
        .with_domain(42);
        assert_eq!(action.domain_id, 42);
    }

    #[test]
    fn test_action_send_goal_key() {
        let action = ActionInfo::new(
            "/fibonacci",
            "example_interfaces::action::dds_::Fibonacci_",
            "abc123",
        )
        .with_domain(0);

        let key: heapless::String<256> = action.send_goal_key();
        // ActionInfo returns the sub-entity name with leading slash for ROS 2 compatibility
        assert_eq!(key.as_str(), "/fibonacci/_action/send_goal");
    }

    #[test]
    fn test_action_feedback_key() {
        let action = ActionInfo::new(
            "/fibonacci",
            "example_interfaces::action::dds_::Fibonacci_",
            "abc123",
        )
        .with_domain(0);

        let key: heapless::String<256> = action.feedback_key();
        assert_eq!(key.as_str(), "/fibonacci/_action/feedback");
    }

    #[test]
    fn test_action_all_sub_names() {
        let action = ActionInfo::new(
            "/fibonacci",
            "example_interfaces::action::dds_::Fibonacci_",
            "abc123",
        )
        .with_domain(0);

        let cancel: heapless::String<256> = action.cancel_goal_key();
        assert_eq!(cancel.as_str(), "/fibonacci/_action/cancel_goal");

        let result: heapless::String<256> = action.get_result_key();
        assert_eq!(result.as_str(), "/fibonacci/_action/get_result");

        let status: heapless::String<256> = action.status_key();
        assert_eq!(status.as_str(), "/fibonacci/_action/status");
    }

    // --- QoS Profile Tests ---

    #[test]
    fn test_qos_profile_sensor_data() {
        let qos = QoSProfile::QOS_PROFILE_SENSOR_DATA;
        assert_eq!(qos.reliability, QoSReliabilityPolicy::BestEffort);
        assert_eq!(qos.durability, QoSDurabilityPolicy::Volatile);
        assert_eq!(qos.history, QoSHistoryPolicy::KeepLast);
        assert_eq!(qos.depth, 5);
    }

    #[test]
    fn test_qos_profile_default() {
        let qos = QoSProfile::QOS_PROFILE_DEFAULT;
        assert_eq!(qos.reliability, QoSReliabilityPolicy::Reliable);
        assert_eq!(qos.durability, QoSDurabilityPolicy::Volatile);
        assert_eq!(qos.depth, 10);
    }

    #[test]
    fn test_qos_profile_services_default() {
        let qos = QoSProfile::QOS_PROFILE_SERVICES_DEFAULT;
        assert_eq!(qos.reliability, QoSReliabilityPolicy::Reliable);
        assert_eq!(qos.durability, QoSDurabilityPolicy::Volatile);
    }

    /// issue 0829 — `SYSTEM_DEFAULT` asks for NOTHING, so it demands no policy.
    ///
    /// `required_policies` started from `QoSPolicyMask::CORE` unconditionally,
    /// which made the sentinel profile demand reliability, durability, history
    /// AND depth — so a backend that could not honour one would answer
    /// `IncompatibleQos` to a profile that requested none of them.
    #[test]
    fn system_default_profile_requires_no_policy() {
        let required = QoSProfile::QOS_PROFILE_SYSTEM_DEFAULT.required_policies();
        assert_eq!(required.0, 0, "SYSTEM_DEFAULT demanded {required:?}");
        // Therefore it is admissible against a backend that advertises nothing.
        assert!(
            QoSProfile::QOS_PROFILE_SYSTEM_DEFAULT
                .validate_against(QoSPolicyMask(0))
                .is_ok()
        );
    }

    /// The relaxation is per FIELD, not all-or-nothing: a profile that states
    /// SOME policies still demands exactly those.
    #[test]
    fn a_partly_stated_profile_demands_only_what_it_states() {
        let qos = QoSProfile {
            reliability: QoSReliabilityPolicy::BestEffort,
            ..QoSProfile::QOS_PROFILE_SYSTEM_DEFAULT
        };
        let required = qos.required_policies();
        assert!(required.contains(QoSPolicyMask::RELIABILITY));
        assert!(!required.contains(QoSPolicyMask::HISTORY));
        assert!(!required.contains(QoSPolicyMask::DEPTH));
        assert!(!required.contains(QoSPolicyMask::DURABILITY_VOLATILE));
        assert!(!required.contains(QoSPolicyMask::DURABILITY_TRANSIENT_LOCAL));
    }

    /// A fully concrete profile is unaffected by the relaxation — this is the
    /// guard on "do not silently stop demanding what the caller asked for".
    #[test]
    fn concrete_profiles_still_demand_their_core_policies() {
        let required = QoSProfile::QOS_PROFILE_DEFAULT.required_policies();
        assert!(required.contains(QoSPolicyMask::RELIABILITY));
        assert!(required.contains(QoSPolicyMask::DURABILITY_VOLATILE));
        assert!(required.contains(QoSPolicyMask::HISTORY));
        assert!(required.contains(QoSPolicyMask::DEPTH));
        // And a backend missing one still rejects it.
        let missing = QoSPolicyMask(required.0 & !QoSPolicyMask::DEPTH.0);
        assert_eq!(
            QoSProfile::QOS_PROFILE_DEFAULT.validate_against(missing),
            Err(TransportError::IncompatibleQos)
        );
    }

    /// issue 0829 — resolution replaces ONLY the sentinel fields, and the
    /// answer is the backend's, not a constant this crate bakes.
    #[test]
    fn resolve_system_default_fills_absences_and_touches_nothing_else() {
        // Two backends, two different answers to the SAME sentinel — the whole
        // reason no concrete `QOS_PROFILE_SYSTEM_DEFAULT` could be right.
        const CYCLONE: QoSSystemDefaults = QoSSystemDefaults {
            reliability: QoSReliabilityPolicy::Reliable,
            durability: QoSDurabilityPolicy::Volatile,
            history: QoSHistoryPolicy::KeepLast,
            depth: 1,
        };
        const ZENOH: QoSSystemDefaults = QoSSystemDefaults {
            depth: 4,
            ..CYCLONE
        };

        let a = QoSProfile::QOS_PROFILE_SYSTEM_DEFAULT.resolve_system_default(&CYCLONE);
        let b = QoSProfile::QOS_PROFILE_SYSTEM_DEFAULT.resolve_system_default(&ZENOH);
        assert_eq!(a.reliability, QoSReliabilityPolicy::Reliable);
        assert_eq!(a.history, QoSHistoryPolicy::KeepLast);
        assert_eq!(a.durability, QoSDurabilityPolicy::Volatile);
        assert_eq!(a.depth, 1);
        assert_eq!(b.depth, 4);
        assert!(!a.has_unresolved_system_default());
        assert!(!b.has_unresolved_system_default());

        // A STATED policy is never overridden — resolution fills an absence.
        let stated = QoSProfile {
            reliability: QoSReliabilityPolicy::BestEffort,
            history: QoSHistoryPolicy::KeepAll,
            durability: QoSDurabilityPolicy::TransientLocal,
            depth: 7,
            ..QoSProfile::QOS_PROFILE_SYSTEM_DEFAULT
        };
        let resolved = stated.resolve_system_default(&CYCLONE);
        assert_eq!(resolved, stated, "resolution overrode a stated policy");

        // Idempotent.
        assert_eq!(a.resolve_system_default(&ZENOH), a);
    }

    #[test]
    fn services_default_validates_and_rejects_missing_policy() {
        // Phase 193.5 — the service-create path (node `create_service*_sized` +
        // the typed-arena `register_service*_on`) runs `validate_against` on the
        // caller's profile, exactly like pub/sub. A backend advertising the
        // profile's required policies admits it; dropping any required bit (here
        // RELIABILITY) rejects it with `IncompatibleQos` — no silent downgrade.
        let qos = QoSProfile::services_default();
        let required = qos.required_policies();
        assert!(qos.validate_against(required).is_ok());
        assert!(qos.validate_against(QoSPolicyMask(u32::MAX)).is_ok());
        let missing = QoSPolicyMask(required.0 & !QoSPolicyMask::RELIABILITY.0);
        assert_eq!(
            qos.validate_against(missing),
            Err(TransportError::IncompatibleQos)
        );
    }

    /// issue 0793 — this asserted `TransientLocal`, which PINNED THE DEFECT:
    /// upstream `rmw_qos_profile_parameters` is KEEP_LAST(1000) + RELIABLE +
    /// **VOLATILE** (`/opt/ros/<distro>/include/rmw/rmw/qos_profiles.h`), and
    /// our own second copy of the profile, `nros::qos::PARAMETERS`, was already
    /// correct. The test agreed with the wrong copy, so the disagreement between
    /// the two survived every run.
    #[test]
    fn test_qos_profile_parameters() {
        let qos = QoSProfile::QOS_PROFILE_PARAMETERS;
        assert_eq!(qos.reliability, QoSReliabilityPolicy::Reliable);
        assert_eq!(
            qos.durability,
            QoSDurabilityPolicy::Volatile,
            "rmw_qos_profile_parameters is VOLATILE; transient-local would make \
             every parameter server retain for late joiners, which ROS 2 does not do"
        );
        assert_eq!(qos.depth, 1000);
    }

    #[test]
    fn test_qos_profile_clock() {
        let qos = QoSProfile::QOS_PROFILE_CLOCK;
        assert_eq!(qos.reliability, QoSReliabilityPolicy::BestEffort);
        assert_eq!(qos.depth, 1);
    }

    #[test]
    fn test_qos_profile_parameter_events() {
        let qos = QoSProfile::QOS_PROFILE_PARAMETER_EVENTS;
        assert_eq!(qos.reliability, QoSReliabilityPolicy::Reliable);
        assert_eq!(qos.history, QoSHistoryPolicy::KeepAll);
    }

    #[test]
    fn test_qos_profile_action_status() {
        let qos = QoSProfile::QOS_PROFILE_ACTION_STATUS_DEFAULT;
        assert_eq!(qos.reliability, QoSReliabilityPolicy::Reliable);
        assert_eq!(qos.durability, QoSDurabilityPolicy::TransientLocal);
        assert_eq!(qos.depth, 1);
    }

    #[test]
    fn test_qos_static_constructors() {
        assert_eq!(
            QoSProfile::topics_default(),
            QoSProfile::QOS_PROFILE_DEFAULT
        );
        assert_eq!(
            QoSProfile::sensor_data_default(),
            QoSProfile::QOS_PROFILE_SENSOR_DATA
        );
        assert_eq!(
            QoSProfile::services_default(),
            QoSProfile::QOS_PROFILE_SERVICES_DEFAULT
        );
        assert_eq!(
            QoSProfile::parameters_default(),
            QoSProfile::QOS_PROFILE_PARAMETERS
        );
        assert_eq!(
            QoSProfile::action_status_default(),
            QoSProfile::QOS_PROFILE_ACTION_STATUS_DEFAULT
        );
    }

    #[test]
    fn test_qos_builder_explicit_setters() {
        let qos = QoSProfile::new()
            .reliability(QoSReliabilityPolicy::Reliable)
            .durability(QoSDurabilityPolicy::TransientLocal)
            .history(QoSHistoryPolicy::KeepAll)
            .depth(100);

        assert_eq!(qos.reliability, QoSReliabilityPolicy::Reliable);
        assert_eq!(qos.durability, QoSDurabilityPolicy::TransientLocal);
        assert_eq!(qos.history, QoSHistoryPolicy::KeepAll);
        assert_eq!(qos.depth, 100);
    }

    #[test]
    fn test_qos_builder_chaining() {
        // Test that builder methods can be chained in any order
        let qos = QoSProfile::sensor_data_default()
            .reliable()
            .transient_local()
            .keep_last(20);

        assert_eq!(qos.reliability, QoSReliabilityPolicy::Reliable);
        assert_eq!(qos.durability, QoSDurabilityPolicy::TransientLocal);
        assert_eq!(qos.history, QoSHistoryPolicy::KeepLast);
        assert_eq!(qos.depth, 20);
    }

    #[test]
    fn test_qos_eq_impl() {
        // Verify that PartialEq works correctly via derive on QoSProfile
        let qos1 = QoSProfile::QOS_PROFILE_DEFAULT;
        let qos2 = QoSProfile::topics_default();
        // Both should have same values - verify field by field
        assert_eq!(qos1.reliability, qos2.reliability);
        assert_eq!(qos1.durability, qos2.durability);
        assert_eq!(qos1.history, qos2.history);
        assert_eq!(qos1.depth, qos2.depth);
    }

    // --- Locator validation tests ---

    #[test]
    fn test_locator_protocol_tcp() {
        assert_eq!(locator_protocol("tcp/127.0.0.1:7447"), LocatorProtocol::Tcp);
    }

    #[test]
    fn test_locator_protocol_serial() {
        assert_eq!(
            locator_protocol("serial//dev/ttyUSB0#baudrate=115200"),
            LocatorProtocol::Serial
        );
    }

    #[test]
    fn test_locator_protocol_unknown() {
        assert_eq!(locator_protocol(""), LocatorProtocol::Unknown);
        assert_eq!(locator_protocol("http://foo"), LocatorProtocol::Unknown);
        assert_eq!(locator_protocol("tls/host:port"), LocatorProtocol::Unknown);
    }

    #[test]
    fn test_locator_protocol_udp() {
        assert_eq!(locator_protocol("udp/127.0.0.1:7447"), LocatorProtocol::Udp);
        assert_eq!(
            locator_protocol("udp/192.168.1.50:2019"),
            LocatorProtocol::Udp
        );
    }

    #[test]
    fn test_validate_tcp_locator_ok() {
        assert!(validate_locator("tcp/127.0.0.1:7447").is_ok());
        assert!(validate_locator("tcp/192.168.1.1:7447").is_ok());
    }

    #[test]
    fn test_validate_tcp_locator_missing_port() {
        assert!(validate_locator("tcp/127.0.0.1").is_err());
    }

    #[test]
    fn test_validate_serial_locator_ok() {
        assert!(validate_locator("serial//dev/ttyUSB0#baudrate=115200").is_ok());
        assert!(validate_locator("serial//dev/ttyACM0#baudrate=9600").is_ok());
        assert!(validate_locator("serial/uart1#baudrate=921600").is_ok());
    }

    #[test]
    fn test_validate_serial_locator_empty_device() {
        assert!(validate_locator("serial/").is_err());
    }

    #[test]
    fn test_validate_serial_locator_missing_baudrate() {
        assert!(validate_locator("serial//dev/ttyUSB0").is_err());
    }

    #[test]
    fn test_validate_serial_locator_invalid_baudrate() {
        assert!(validate_locator("serial//dev/ttyUSB0#baudrate=abc").is_err());
    }

    #[test]
    fn test_validate_unknown_protocol() {
        assert!(validate_locator("http://foo").is_err());
        assert!(validate_locator("tls/host:port").is_err());
    }

    #[test]
    fn test_validate_udp_locator_ok() {
        assert!(validate_locator("udp/127.0.0.1:7447").is_ok());
        assert!(validate_locator("udp/192.168.1.50:2019").is_ok());
    }

    #[test]
    fn test_validate_udp_locator_missing_port() {
        assert!(validate_locator("udp/127.0.0.1").is_err());
    }

    // Phase 233.2 — the PX4 companion QoS profile must be BEST_EFFORT +
    // TRANSIENT_LOCAL + KEEP_LAST so it matches PX4's uxrce_dds_client endpoints.
    #[test]
    fn px4_qos_profile_matches_uxrce_dds_client() {
        let q = QoSProfile::px4();
        assert_eq!(q.reliability, QoSReliabilityPolicy::BestEffort);
        // VOLATILE — PX4's /fmu/out writers are volatile; a TRANSIENT_LOCAL
        // reader silently fails to match (verified against real PX4 SITL).
        assert_eq!(q.durability, QoSDurabilityPolicy::Volatile);
        assert_eq!(q.history, QoSHistoryPolicy::KeepLast);
        assert_eq!(q, QoSProfile::QOS_PROFILE_PX4);
        // Depth is tunable via the builder without losing the PX4 policies.
        let deep = QoSProfile::px4().keep_last(5);
        assert_eq!(deep.depth, 5);
        assert_eq!(deep.reliability, QoSReliabilityPolicy::BestEffort);
        assert_eq!(deep.durability, QoSDurabilityPolicy::Volatile);
    }

    // --- RmwConfig Tests ---

    #[test]
    fn test_rmw_config_default() {
        let config = RmwConfig::default();
        assert_eq!(config.locator, "tcp/127.0.0.1:7447");
        assert_eq!(config.mode, SessionMode::Client);
        assert_eq!(config.domain_id, 0);
        assert_eq!(config.node_name, "node");
        assert_eq!(config.namespace, "");
    }

    #[test]
    fn test_rmw_config_custom() {
        let config = RmwConfig {
            locator: "tcp/192.168.1.1:7447",
            mode: SessionMode::Peer,
            domain_id: 42,
            node_name: "talker",
            namespace: "/ns1",
            properties: &[("agent_port", "2019")],
        };
        assert_eq!(config.locator, "tcp/192.168.1.1:7447");
        assert_eq!(config.mode, SessionMode::Peer);
        assert_eq!(config.domain_id, 42);
        assert_eq!(config.node_name, "talker");
        assert_eq!(config.namespace, "/ns1");
        assert_eq!(config.properties.len(), 1);
        assert_eq!(config.properties[0].0, "agent_port");
    }

    #[test]
    fn test_rmw_config_is_copy() {
        let config = RmwConfig::default();
        let config2 = config; // Copy
        assert_eq!(config.locator, config2.locator);
        assert_eq!(config.domain_id, config2.domain_id);
    }

    #[test]
    fn test_rmw_config_clone() {
        let config = RmwConfig::default();
        let cloned = RmwConfig { ..config };
        assert_eq!(cloned.locator, config.locator);
        assert_eq!(cloned.node_name, config.node_name);
    }
}

#[cfg(test)]
mod rx_buffer_hint_tests {
    use super::*;

    /// issue 0896 / phase-402 — the hint must SURVIVE being set on `TopicInfo`.
    ///
    /// This is the seam every path funnels through: the C register writes it
    /// here, `nros-rmw-cffi` reads `topic.rx_buffer_hint` back out when
    /// building `rmw_subscription_options_t`, and a size-classing backend
    /// routes on it. A default that silently stayed 0 is the original defect,
    /// so the round trip is worth an assertion rather than an assumption.
    #[test]
    fn a_hint_set_on_topic_info_is_readable_back() {
        let t = TopicInfo::new("/chatter", "std_msgs/msg/Int32", "").with_rx_buffer_hint(4096);
        assert_eq!(t.rx_buffer_hint, 4096);
    }

    /// Zero is "no opinion", not "zero bytes". The C options struct uses 0 as
    /// its unset sentinel and the executor only calls the setter when the
    /// caller stated something, so the default must stay 0 for the
    /// backend-default path to remain reachable.
    #[test]
    fn the_default_hint_is_zero_meaning_no_opinion() {
        let t = TopicInfo::new("/chatter", "std_msgs/msg/Int32", "");
        assert_eq!(t.rx_buffer_hint, 0);
    }

    /// The builder must not disturb the rest of the descriptor — it is applied
    /// AFTER domain/namespace/node in the C path, so a setter that reset a
    /// field would drop identity that discovery depends on.
    #[test]
    fn setting_the_hint_preserves_the_rest_of_the_descriptor() {
        let make = || {
            TopicInfo::new("/chatter", "std_msgs/msg/Int32", "hash")
                .with_domain(7)
                .with_namespace("/ns")
        };
        let base = make();
        let hinted = make().with_rx_buffer_hint(1234);
        assert_eq!(hinted.name, base.name);
        assert_eq!(hinted.type_name, base.type_name);
        assert_eq!(hinted.type_hash, base.type_hash);
        assert_eq!(hinted.domain_id, base.domain_id);
        assert_eq!(hinted.namespace, base.namespace);
        assert_eq!(hinted.rx_buffer_hint, 1234);
    }
}
