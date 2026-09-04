//! Zenoh transport backend
//!
//! Provides a transport backend using the nros-rmw-zenoh wrapper.
//! This is designed for embedded platforms that need a simpler API than
//! the full zenoh-pico bindings.
//!
//! Requires the `shim` feature flag.
//!
//! # Features
//!
//! - Session management with ZenohId support
//! - Publishers with RMW attachment support for rmw_zenoh compatibility
//! - Subscribers with wildcard matching
//! - Liveliness tokens for ROS 2 discovery
//! - Service servers via queryables
//! - Service clients via z_get queries
//! - Manual polling (no background threads) for embedded systems
//!
//! # Example
//!
//! ```ignore
//! use nros_rmw::{Transport, TransportConfig, SessionMode};
//! use nros_rmw_zenoh::ZenohTransport;
//!
//! // Create config
//! let config = TransportConfig {
//!     locator: Some("tcp/192.168.1.1:7447"),
//!     mode: SessionMode::Client,
//!     properties: &[],
//!     node_name: "my_node",
//!     namespace: "",
//!     domain_id: 0,
//! };
//!
//! // Open session
//! let mut session = ZenohTransport::open(&config).expect("Failed to open session");
//!
//! // Must poll periodically
//! session.spin_once(core::time::Duration::from_millis(10))?;
//! ```

pub mod publisher;
pub mod service;
pub mod session;
pub mod subscriber;
pub mod transport;

use portable_atomic::Ordering;

// Use AtomicI64 on 64-bit targets, AtomicI32 on 32-bit (e.g. Cortex-M, riscv32)
// portable-atomic provides fetch_add/fetch_sub even on targets without native
// atomic RMW instructions (e.g. riscv32imc) via software fallback.
#[cfg(target_has_atomic = "64")]
pub(crate) type AtomicSeqCounter = portable_atomic::AtomicI64;
#[cfg(not(target_has_atomic = "64"))]
pub(crate) type AtomicSeqCounter = portable_atomic::AtomicI32;

// Scalar matching `AtomicSeqCounter`'s width — i64 on 64-bit targets, i32 on
// 32-bit (riscv32, Cortex-M). Use it to narrow an i64 reply-seq/FFI value to the
// counter's native width before `.store()`; the FFI
// (`zpico_queryable_take_reply_seq`) always returns i64, but on 32-bit the
// counter is AtomicI32 (an `i64` store would be a type error — would break the
// esp32/cortex-m build). The load side already widens via `.into()`.
#[cfg(target_has_atomic = "64")]
pub(crate) type SeqScalar = i64;
#[cfg(not(target_has_atomic = "64"))]
pub(crate) type SeqScalar = i32;

use nros_rmw::{QoSProfile, ServiceInfo, TopicInfo, TransportError};

use crate::{
    keyexpr::QosKeyExpr,
    zpico::{Context, LivelinessToken, ZPICO_RMW_GID_SIZE, ZpicoError},
};

// Re-export for convenience
pub use crate::zpico::ZenohId;

// Re-export submodule types
pub use publisher::ZenohPublisher;
pub use service::{ZenohServiceClient, ZenohServiceServer};
pub use session::{ZenohSession, effective_client_locator, normalize_locator};
pub use subscriber::{ZenohSubscriber, overflow_drops_total, required_rx_bytes};
pub use transport::{ZenohRmw, ZenohTransport};

// ============================================================================
// Constants
// ============================================================================

/// RMW GID size for attachment serialization (16 bytes for Humble)
pub const RMW_GID_SIZE: usize = ZPICO_RMW_GID_SIZE;

/// Size of serialized RMW attachment (without safety CRC)
/// Format: sequence_number (8) + timestamp (8) + VLE length (1) + gid (16) = 33 bytes
pub(crate) const RMW_ATTACHMENT_SIZE: usize = 8 + 8 + 1 + RMW_GID_SIZE;

/// Size of the CRC-32 field appended when safety-e2e is enabled
#[cfg(feature = "safety-e2e")]
pub(crate) const SAFETY_CRC_SIZE: usize = 4;

/// Total attachment size with safety CRC (37 bytes)
#[cfg(feature = "safety-e2e")]
pub(crate) const RMW_ATTACHMENT_SIZE_WITH_CRC: usize = RMW_ATTACHMENT_SIZE + SAFETY_CRC_SIZE;

/// LCG multiplier for GID PRNG generation.
const GID_PRNG_MULTIPLIER: u64 = 0x517cc1b727220a95;

/// Null-terminated locator string buffer size.
pub(crate) const LOCATOR_BUFFER_SIZE: usize = 128;

/// Property key/value buffer size for session configuration.
/// std builds must hold file paths (TLS certificate paths, etc.) → 256.
/// no_std / bare-metal builds (e.g. esp32-c3, 18 KB stack) carry only small
/// property values (scouting/listen/zid) and have no TLS; the full 256 ×
/// `MAX_SESSION_PROPERTIES` × 2 (key+val) = 4 KB stack frame in
/// `SmoltcpSession::new` overflows the small stack into `.bss` (issue #64), so
/// shrink it there. 64 covers every embedded property value.
#[cfg(feature = "std")]
pub(crate) const CONFIG_PROPERTY_SIZE: usize = 256;
#[cfg(not(feature = "std"))]
pub(crate) const CONFIG_PROPERTY_SIZE: usize = 64;

/// Maximum number of session configuration properties.
///
/// phase-206 W3 — split by build, for the same reason `CONFIG_PROPERTY_SIZE`
/// above is. zenoh-pico defines 23 run-time keys; `mode` and `connect` arrive
/// as dedicated `zpico_init_with_config` arguments, and the 13 TLS keys split
/// into a listen-side set and a connect-side set, so a full mTLS client states
/// roughly ten at once. Eight could not express that — and the overflow used
/// to be a silent `.min()` truncation, so the properties past the eighth
/// simply never happened.
///
/// The embedded cap stays 8: this array is `MAX_SESSION_PROPERTIES` × 2 ×
/// `CONFIG_PROPERTY_SIZE` bytes of stack in `ZenohSession::new`, no `no_std`
/// build has TLS at all (`Z_FEATURE_LINK_TLS` is a hosted/mbedtls build), and
/// issue #64 is what a 4 KB frame costs on an esp32-c3. Overflow is now an
/// error on both, so a target that cannot carry a configuration says so
/// instead of carrying part of it.
#[cfg(feature = "std")]
pub(crate) const MAX_SESSION_PROPERTIES: usize = 16;
#[cfg(not(feature = "std"))]
pub(crate) const MAX_SESSION_PROPERTIES: usize = 8;

// KEYEXPR_STRING_SIZE and KEYEXPR_BUFFER_SIZE are generated by build.rs
// from the NROS_KEYEXPR_STRING_SIZE env var (default 256).
pub(crate) use crate::config::{KEYEXPR_BUFFER_SIZE, KEYEXPR_STRING_SIZE};

/// Buffer size for topic/namespace name mangling.
const MANGLED_NAME_SIZE: usize = 64;

/// 16-byte ZID formatted as hexadecimal (32 ASCII characters).
const ZID_HEX_SIZE: usize = 32;

/// QoS encoding string buffer size.
const QOS_STRING_SIZE: usize = 32;

/// Placeholder timestamp increment per publish (1ms in nanoseconds).
/// Retained as a documented landing place for legacy callers; the
/// publisher path now uses `<P as PlatformClock>::clock_ms()` directly
/// (Phase 108.C.zenoh.2).
#[allow(dead_code)]
pub(crate) const TIMESTAMP_INCREMENT_NS: i64 = 1_000_000;

/// Attachment buffer size: 33 bytes normally, 37 with safety CRC
#[cfg(not(feature = "safety-e2e"))]
pub(crate) const SUBSCRIBER_ATTACHMENT_BUF_SIZE: usize = RMW_ATTACHMENT_SIZE;
#[cfg(feature = "safety-e2e")]
pub(crate) const SUBSCRIBER_ATTACHMENT_BUF_SIZE: usize = RMW_ATTACHMENT_SIZE_WITH_CRC;

// `PUBLISHER_TX_BUFFER_SIZE` is re-exported unconditionally, like its
// subscriber-side twins — the publisher TX arena that consumes it is behind the
// `lending` feature, but the knob's value is a build fact either way and a
// feature-gated re-export would hide it from anyone sizing an image (issue 0813).
pub use crate::config::{
    MAX_LARGE_SUBSCRIBERS, PUBLISHER_TX_BUFFER_SIZE, SERVICE_BUFFER_SIZE, SUBSCRIBER_BUFFER_SIZE,
    SUBSCRIBER_LARGE_SIZE, SUBSCRIBER_RING_DEPTH, SUBSCRIBER_SIZE_THRESHOLD,
};

// ============================================================================
// Runtime wake callback — async transport-arrival signal (no_std-safe)
// ============================================================================
//
// Issue #0317 — the nros-node executor installs a runtime wake callback on the
// session via `Session::set_wake_callback` (→ `nros_rmw_runtime_wake_cb`, which
// fires the wake-latency probe's `on_wake` T0 + signals the executor cv). The
// zpico shim fired it only from the main-thread `drive_io` poll path
// (`if spin_once() saw work`). On the MULTI-THREADED backend the sample is
// received by the async read task, so `drive_io`'s `spin_once` returns 0 and the
// wake-cb never fires — even though the sample is enqueued + later dispatched.
// Mirror the runtime wake-cb into a process-global here so the read-task arrival
// hook (`subscriber_notify_callback` / service notify) can fire it at the real
// arrival instant, regardless of `std`. One executor/session per embedded image,
// so a single global slot is sufficient.

pub(crate) static RUNTIME_WAKE_CB: portable_atomic::AtomicPtr<()> =
    portable_atomic::AtomicPtr::new(core::ptr::null_mut());
pub(crate) static RUNTIME_WAKE_CTX: portable_atomic::AtomicPtr<core::ffi::c_void> =
    portable_atomic::AtomicPtr::new(core::ptr::null_mut());

/// Record the runtime wake callback so the async arrival path can fire it.
/// Called from `Session::set_wake_callback`.
pub(crate) fn set_runtime_wake_cb(
    cb: Option<unsafe extern "C" fn(ctx: *mut core::ffi::c_void)>,
    ctx: *mut core::ffi::c_void,
) {
    use portable_atomic::Ordering;
    let cb_ptr = cb.map(|f| f as *mut ()).unwrap_or(core::ptr::null_mut());
    RUNTIME_WAKE_CB.store(cb_ptr, Ordering::Release);
    RUNTIME_WAKE_CTX.store(ctx, Ordering::Release);
}

/// Fire the runtime wake callback (→ probe `on_wake` T0 + executor cv-signal) if
/// one is installed. Invoked from the read-task arrival hooks so a blocked
/// executor is woken and the probe timestamps a real transport arrival on the
/// multi-threaded backend. no_std-safe; a no-op when no callback is installed.
pub(crate) fn fire_runtime_wake() {
    use portable_atomic::Ordering;
    let cb = RUNTIME_WAKE_CB.load(Ordering::Acquire);
    if !cb.is_null() {
        let ctx = RUNTIME_WAKE_CTX.load(Ordering::Acquire);
        // SAFETY: `cb` was installed from `set_wake_callback` as an
        // `unsafe extern "C" fn(*mut c_void)`; `ctx` is the executor-owned wake
        // state that outlives the session (cleared to null on executor drop).
        let f: unsafe extern "C" fn(*mut core::ffi::c_void) = unsafe { core::mem::transmute(cb) };
        unsafe { f(ctx) };
    }
}

// ============================================================================
// Executor Wake Signal (std only)
// ============================================================================

/// Signal the executor that new data is available.
///
/// Called from subscription and service callbacks (which run on the zenoh-pico
/// background read thread) to wake the executor's `spin()` loop immediately
/// instead of waiting for the poll interval timeout.
#[cfg(feature = "std")]
pub fn signal_executor_wake() {
    let (lock, cvar) = &*EXECUTOR_WAKE;
    if let Ok(mut pending) = lock.lock() {
        *pending = true;
        cvar.notify_one();
    }
}

/// Wait for a wake signal or timeout.
///
/// Returns `true` if woken by a signal, `false` on timeout.
/// Used by `BasicExecutor::spin()` to sleep efficiently between iterations.
#[cfg(feature = "std")]
pub fn wait_for_executor_wake(timeout: core::time::Duration) -> bool {
    let (lock, cvar) = &*EXECUTOR_WAKE;
    if let Ok(mut pending) = lock.lock() {
        if *pending {
            *pending = false;
            return true;
        }
        let result = cvar.wait_timeout(pending, timeout);
        if let Ok((mut guard, _)) = result {
            let was_signaled = *guard;
            *guard = false;
            was_signaled
        } else {
            false
        }
    } else {
        // Mutex poisoned — fall back to sleep behavior
        std::thread::sleep(timeout);
        false
    }
}

#[cfg(feature = "std")]
static EXECUTOR_WAKE: std::sync::LazyLock<(std::sync::Mutex<bool>, std::sync::Condvar)> =
    std::sync::LazyLock::new(|| (std::sync::Mutex::new(false), std::sync::Condvar::new()));

// ============================================================================
// Error Conversion
// ============================================================================

impl From<ZpicoError> for TransportError {
    fn from(err: ZpicoError) -> Self {
        // issue 0870 — `Generic` and `Session` both become `ConnectionFailed`
        // below, so by the time a caller sees it the reason is gone. Issue 0465
        // records what that costs: an exhausted session pool "spent two months
        // looking like `Transport(ConnectionFailed)` — a router/network
        // problem, and chased as one". Naming the variant here is the same
        // remedy 0465 applied one frame up, and it only runs on a path that has
        // already failed.
        //
        // `nros_log`, not `eprintln!`: this crate reaches `no_std` targets, and
        // std stdio is FATAL on Zephyr native_sim (issue 0589).
        // ONLY the two ambiguous variants. This conversion is on the hot
        // path — `drive_io` maps `Timeout` on every quiet tick (issue 0387),
        // so logging unconditionally would flood the console of a working
        // image. `Generic` and `Session` are the pair that collapse into one
        // `ConnectionFailed`, and they are the only ones whose identity is
        // lost here; every other variant maps 1:1 and needs no announcement.
        if matches!(err, ZpicoError::Generic | ZpicoError::Session) {
            nros_log::log_error!(
                nros_log::get_logger("nros_rmw_zenoh"),
                "zpico {:?} -> ConnectionFailed (the two are indistinguishable downstream)",
                err
            );
        }
        match err {
            ZpicoError::Generic => TransportError::ConnectionFailed,
            ZpicoError::Config => TransportError::InvalidArgument,
            ZpicoError::Session => TransportError::ConnectionFailed,
            ZpicoError::Task => TransportError::TaskStartFailed,
            ZpicoError::KeyExpr => TransportError::TopicNameInvalid,
            // Issue 0465 — `Full` means a COMPILE-TIME capacity was exceeded
            // (`ZPICO_MAX_SESSIONS`, `ZPICO_MAX_PUBLISHERS`, …), so it is a
            // configuration fact, not a runtime failure of whatever call
            // happened to hit the ceiling. It used to say
            // `PublisherCreationFailed`, which is actively wrong when the
            // exhausted pool is the SESSION pool and no publisher is involved.
            //
            // `InvalidConfig` also maps to `NROS_RMW_RET_INVALID_ARGUMENT` (-4)
            // rather than the generic -1, so the distinction survives the C ABI
            // instead of arriving as an indistinguishable "rmw_ret error".
            ZpicoError::Full => TransportError::InvalidConfig,
            ZpicoError::Invalid => TransportError::InvalidArgument,
            ZpicoError::Publish => TransportError::PublishFailed,
            ZpicoError::NotOpen => TransportError::Disconnected,
            ZpicoError::Timeout => TransportError::Timeout,
        }
    }
}

// ============================================================================
// RMW Attachment Support
// ============================================================================

/// RMW attachment data for rmw_zenoh compatibility
///
/// This metadata is attached to each published message and is required
/// for ROS 2 nodes using rmw_zenoh_cpp to receive messages.
#[derive(Debug, Clone, Copy)]
pub struct RmwAttachment {
    /// Message sequence number (incremented per publish)
    pub sequence_number: i64,
    /// Timestamp in nanoseconds
    pub timestamp: i64,
    /// RMW Global Identifier (random, generated once per publisher)
    pub rmw_gid: [u8; RMW_GID_SIZE],
}

impl RmwAttachment {
    /// Create a new attachment with a random GID
    pub fn new() -> Self {
        Self {
            sequence_number: 0,
            timestamp: 0,
            rmw_gid: Self::generate_gid(),
        }
    }

    /// Generate a random GID using a simple PRNG
    pub fn generate_gid() -> [u8; RMW_GID_SIZE] {
        let mut gid = [0u8; RMW_GID_SIZE];
        static COUNTER: AtomicSeqCounter = AtomicSeqCounter::new(0);
        let seed = COUNTER.fetch_add(1, Ordering::Relaxed) as u64;
        // Use address of gid as additional entropy
        let addr = &gid as *const _ as u64;
        let mixed = seed.wrapping_mul(GID_PRNG_MULTIPLIER) ^ addr;

        for (i, byte) in gid.iter_mut().enumerate() {
            let shift = (i % 8) * 8;
            *byte = ((mixed.wrapping_mul((i as u64).wrapping_add(1))) >> shift) as u8;
        }
        gid
    }

    /// Serialize the attachment in the format expected by rmw_zenoh_cpp
    ///
    /// Format:
    /// - int64: sequence_number (little-endian, 8 bytes)
    /// - int64: timestamp (little-endian, 8 bytes)
    /// - VLE length (1 byte for length 16)
    /// - 16 x uint8: GID
    pub fn serialize(&self, buf: &mut [u8; RMW_ATTACHMENT_SIZE]) {
        // Sequence number (little-endian)
        buf[0..8].copy_from_slice(&self.sequence_number.to_le_bytes());
        // Timestamp (little-endian)
        buf[8..16].copy_from_slice(&self.timestamp.to_le_bytes());
        // VLE length (16 fits in single byte)
        buf[16] = RMW_GID_SIZE as u8;
        // GID bytes
        buf[17..33].copy_from_slice(&self.rmw_gid);
    }
}

impl Default for RmwAttachment {
    fn default() -> Self {
        Self::new()
    }
}

impl RmwAttachment {
    /// Deserialize attachment from raw bytes received from RMW
    ///
    /// Format:
    /// - int64: sequence_number (little-endian, 8 bytes)
    /// - int64: timestamp (little-endian, 8 bytes)
    /// - VLE length (1 byte for length 16)
    /// - 16 x uint8: GID
    ///
    /// Returns None if the buffer is too small or malformed.
    pub fn deserialize(buf: &[u8]) -> Option<Self> {
        if buf.len() < RMW_ATTACHMENT_SIZE {
            return None;
        }

        // Parse sequence number (little-endian)
        let sequence_number = i64::from_le_bytes([
            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
        ]);

        // Parse timestamp (little-endian)
        let timestamp = i64::from_le_bytes([
            buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15],
        ]);

        // Parse VLE length (should be 16)
        let gid_len = buf[16] as usize;
        if gid_len != RMW_GID_SIZE {
            return None;
        }

        // Parse GID
        let mut rmw_gid = [0u8; RMW_GID_SIZE];
        rmw_gid.copy_from_slice(&buf[17..33]);

        Some(Self {
            sequence_number,
            timestamp,
            rmw_gid,
        })
    }
}

/// Message information parsed from RMW attachment
///
/// This struct contains metadata about a received message, extracted
/// from the rmw_zenoh attachment.
#[derive(Debug, Clone, Copy)]
pub struct MessageInfo {
    /// Message sequence number from the publisher
    pub sequence_number: i64,
    /// Timestamp in nanoseconds (source time)
    pub timestamp_ns: i64,
    /// Publisher's Global Identifier
    pub publisher_gid: [u8; RMW_GID_SIZE],
}

impl MessageInfo {
    /// Parse MessageInfo from raw attachment data
    ///
    /// Returns None if no attachment or if parsing fails.
    pub fn from_attachment(attachment: &[u8]) -> Option<Self> {
        RmwAttachment::deserialize(attachment).map(|att| Self {
            sequence_number: att.sequence_number,
            timestamp_ns: att.timestamp,
            publisher_gid: att.rmw_gid,
        })
    }
}

// ============================================================================
// Ros2Liveliness Helper
// ============================================================================

/// ROS 2 liveliness key expression builder for the shim transport
///
/// Generates the key expressions required for ROS 2 discovery via rmw_zenoh.
///
/// Protocol format:
/// `{PREFIX}/{domain_id}/{zid}/{version}/{entity}/%/{ns}/{node}[/{topic}/{type}/{hash}/{qos}]`
pub struct Ros2Liveliness;

// rmw_zenoh liveliness protocol constants
const LIVELINESS_PREFIX: &str = "@ros2_lv";
const PROTO_VERSION_NODE: &str = "0/0";
/// The `<node_id>/<entity_id>` pair a DISCOVERY query must not pin (issue 0890).
///
/// There is no `PROTO_VERSION_TOPIC` any more. The name was always wrong: the
/// two chunks are a node id and an ENTITY id, not a protocol version, and
/// issue 0292 measured what pinning them costs — every entity of an action
/// server collided at id 11 in `rmw_zenoh_cpp`'s graph cache and deduped to
/// one, so `ros2 action list` was empty. 0292 fixed the four DECLARATION
/// builders and left the two WILDCARD builders on the constant, where the same
/// literal stops being a collision and becomes a filter: a query pinned to
/// `0/11` matches only the eleventh entity a session declares.
///
/// A discovery query wildcards BOTH chunks. Not `0/*`: our own nodes use node
/// id 0, but the peers worth discovering are native `rmw_zenoh_cpp` ones, and
/// their node id is not ours to assume.
const ENTITY_ANY: &str = "*/*";
const ENTITY_NODE: &str = "NN";
const ENTITY_PUBLISHER: &str = "MP";
const ENTITY_SUBSCRIBER: &str = "MS";
const ENTITY_SERVICE_SERVER: &str = "SS";
const ENTITY_SERVICE_CLIENT: &str = "SC";

/// What a liveliness token describes — phase-381 W2.
///
/// The two-letter tokens are `rmw_zenoh_cpp`'s, not ours to choose.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityKind {
    Node,
    Publisher,
    Subscriber,
    ServiceServer,
    ServiceClient,
}

impl EntityKind {
    /// `None` for an unrecognised token. A kind we do not know is not a kind we
    /// can report an entity as — guessing here would put a real node in the
    /// wrong column of `ros2 node info`.
    pub fn from_token(token: &str) -> Option<Self> {
        match token {
            ENTITY_NODE => Some(Self::Node),
            ENTITY_PUBLISHER => Some(Self::Publisher),
            ENTITY_SUBSCRIBER => Some(Self::Subscriber),
            ENTITY_SERVICE_SERVER => Some(Self::ServiceServer),
            ENTITY_SERVICE_CLIENT => Some(Self::ServiceClient),
            _ => None,
        }
    }
}

/// One parsed `@ros2_lv` token. Every string BORROWS from the keyexpr it was
/// parsed from, and names stay MANGLED — see `Ros2Liveliness::parse`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LivelinessEntity<'a> {
    pub domain_id: u32,
    pub zid: &'a str,
    pub node_id: u32,
    pub entity_id: u32,
    pub kind: EntityKind,
    /// Mangled (`%demo`), as it appears on the wire.
    pub namespace: &'a str,
    pub node_name: &'a str,
    /// `None` for a node token; `Some` for every entity kind.
    pub topic: Option<&'a str>,
    pub type_name: Option<&'a str>,
    pub type_hash: Option<&'a str>,
    pub qos: Option<&'a str>,
}

impl Ros2Liveliness {
    /// Build a node liveliness key expression
    ///
    /// Format: `@ros2_lv/<domain_id>/<zid>/0/0/NN/%/<mangled_ns>/<node_name>`
    ///
    /// The namespace is mangled using the same rules as topic names:
    /// - `/` → `%`
    /// - `/demo` → `%demo`
    /// - `/ns/sub` → `%ns%sub`
    pub fn node_keyexpr<const N: usize>(
        domain_id: u32,
        zid: &ZenohId,
        namespace: &str,
        node_name: &str,
    ) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let mut zid_hex = [0u8; ZID_HEX_SIZE];
        zid.to_hex_bytes(&mut zid_hex);
        let zid_str = core::str::from_utf8(&zid_hex).unwrap_or("");
        let ns_mangled = Self::mangle_topic_name::<MANGLED_NAME_SIZE>(namespace);
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/{}/{}/{}/%/{}/{}",
                LIVELINESS_PREFIX,
                domain_id,
                zid_str,
                PROTO_VERSION_NODE,
                ENTITY_NODE,
                ns_mangled.as_str(),
                node_name
            ),
        );
        key
    }

    /// Build a publisher liveliness key expression
    ///
    /// Format: `@ros2_lv/<domain_id>/<zid>/0/<entity_id>/MP/%/<mangled_ns>/<node_name>/<topic>/<type>/<hash>/<qos>`
    /// Note: type_hash already includes the `RIHS01_` prefix from generated code
    pub fn publisher_keyexpr<const N: usize>(
        domain_id: u32,
        zid: &ZenohId,
        entity_id: u32,
        namespace: &str,
        node_name: &str,
        topic: &TopicInfo,
        qos: &QoSProfile,
    ) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let mut zid_hex = [0u8; ZID_HEX_SIZE];
        zid.to_hex_bytes(&mut zid_hex);
        let zid_str = core::str::from_utf8(&zid_hex).unwrap_or("");
        // Mangle topic name: replace slashes with percent signs
        let topic_mangled = Self::mangle_topic_name::<MANGLED_NAME_SIZE>(topic.name);
        let ns_mangled = Self::mangle_topic_name::<MANGLED_NAME_SIZE>(namespace);
        let qos_string: heapless::String<QOS_STRING_SIZE> = qos.to_qos_string();
        // `<node_id>/<entity_id>` — node is 0, entity id is per-session unique
        // (issue #0292); a hardcoded id collided every entity in a participant.
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/{}/0/{}/{}/%/{}/{}/{}/{}/{}/{}",
                LIVELINESS_PREFIX,
                domain_id,
                zid_str,
                entity_id,
                ENTITY_PUBLISHER,
                ns_mangled.as_str(),
                node_name,
                topic_mangled.as_str(),
                crate::keyexpr::DdsTypeName(topic.type_name),
                topic.type_hash,
                qos_string.as_str()
            ),
        );
        key
    }

    /// Build a subscriber liveliness key expression
    ///
    /// Format: `@ros2_lv/<domain_id>/<zid>/0/<entity_id>/MS/%/<mangled_ns>/<node_name>/<topic>/<type>/<hash>/<qos>`
    /// Note: type_hash already includes the `RIHS01_` prefix from generated code
    pub fn subscriber_keyexpr<const N: usize>(
        domain_id: u32,
        zid: &ZenohId,
        entity_id: u32,
        namespace: &str,
        node_name: &str,
        topic: &TopicInfo,
        qos: &QoSProfile,
    ) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let mut zid_hex = [0u8; ZID_HEX_SIZE];
        zid.to_hex_bytes(&mut zid_hex);
        let zid_str = core::str::from_utf8(&zid_hex).unwrap_or("");
        let topic_mangled = Self::mangle_topic_name::<MANGLED_NAME_SIZE>(topic.name);
        let ns_mangled = Self::mangle_topic_name::<MANGLED_NAME_SIZE>(namespace);
        let qos_string: heapless::String<QOS_STRING_SIZE> = qos.to_qos_string();
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/{}/0/{}/{}/%/{}/{}/{}/{}/{}/{}",
                LIVELINESS_PREFIX,
                domain_id,
                zid_str,
                entity_id,
                ENTITY_SUBSCRIBER,
                ns_mangled.as_str(),
                node_name,
                topic_mangled.as_str(),
                crate::keyexpr::DdsTypeName(topic.type_name),
                topic.type_hash,
                qos_string.as_str()
            ),
        );
        key
    }

    /// Build a service server liveliness key expression
    ///
    /// Format: `@ros2_lv/<domain_id>/<zid>/0/<entity_id>/SS/%/<mangled_ns>/<node_name>/<service>/<type>/<hash>/<qos>`
    /// Note: type_hash already includes the `RIHS01_` prefix from generated code
    pub fn service_server_keyexpr<const N: usize>(
        domain_id: u32,
        zid: &ZenohId,
        entity_id: u32,
        namespace: &str,
        node_name: &str,
        service: &ServiceInfo,
        qos: &QoSProfile,
    ) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let mut zid_hex = [0u8; ZID_HEX_SIZE];
        zid.to_hex_bytes(&mut zid_hex);
        let zid_str = core::str::from_utf8(&zid_hex).unwrap_or("");
        let service_mangled = Self::mangle_topic_name::<MANGLED_NAME_SIZE>(service.name);
        let ns_mangled = Self::mangle_topic_name::<MANGLED_NAME_SIZE>(namespace);
        let qos_string: heapless::String<QOS_STRING_SIZE> = qos.to_qos_string();
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/{}/0/{}/{}/%/{}/{}/{}/{}/{}/{}",
                LIVELINESS_PREFIX,
                domain_id,
                zid_str,
                entity_id,
                ENTITY_SERVICE_SERVER,
                ns_mangled.as_str(),
                node_name,
                service_mangled.as_str(),
                crate::keyexpr::DdsTypeName(service.type_name),
                service.type_hash,
                qos_string.as_str()
            ),
        );
        key
    }

    /// Build a service client liveliness key expression
    ///
    /// Format: `@ros2_lv/<domain_id>/<zid>/0/<entity_id>/SC/%/<mangled_ns>/<node_name>/<service>/<type>/<hash>/<qos>`
    /// Note: type_hash already includes the `RIHS01_` prefix from generated code
    pub fn service_client_keyexpr<const N: usize>(
        domain_id: u32,
        zid: &ZenohId,
        entity_id: u32,
        namespace: &str,
        node_name: &str,
        service: &ServiceInfo,
        qos: &QoSProfile,
    ) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let mut zid_hex = [0u8; ZID_HEX_SIZE];
        zid.to_hex_bytes(&mut zid_hex);
        let zid_str = core::str::from_utf8(&zid_hex).unwrap_or("");
        let service_mangled = Self::mangle_topic_name::<MANGLED_NAME_SIZE>(service.name);
        let ns_mangled = Self::mangle_topic_name::<MANGLED_NAME_SIZE>(namespace);
        let qos_string: heapless::String<QOS_STRING_SIZE> = qos.to_qos_string();
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/{}/0/{}/{}/%/{}/{}/{}/{}/{}/{}",
                LIVELINESS_PREFIX,
                domain_id,
                zid_str,
                entity_id,
                ENTITY_SERVICE_CLIENT,
                ns_mangled.as_str(),
                node_name,
                service_mangled.as_str(),
                crate::keyexpr::DdsTypeName(service.type_name),
                service.type_hash,
                qos_string.as_str()
            ),
        );
        key
    }

    /// Build a wildcard liveliness keyexpr matching any service-server token
    /// for the given service.
    ///
    /// phase-381 W3 — wildcard NODE liveliness keyexpr.
    ///
    /// Format: `@ros2_lv/<domain_id>/*/*/*/NN/%/*/*` — every field but the
    /// domain and the entity KIND wildcarded, because "which nodes exist" is a
    /// question about the whole domain.
    ///
    /// Node tokens use the fixed `0/0`, so `*/*` there is broader than we
    /// declare — deliberately, for the same reason as issue 0890: the peers
    /// worth enumerating are native `rmw_zenoh_cpp` nodes and their ids are not
    /// ours to assume.
    pub fn node_keyexpr_wildcard<const N: usize>(domain_id: u32) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/*/{}/{}/%/*/*",
                LIVELINESS_PREFIX, domain_id, ENTITY_ANY, ENTITY_NODE
            ),
        );
        key
    }

    /// phase-381 / issue 0903 — the GRAPH CACHE keyexpr: every liveliness token
    /// on a domain, both shapes.
    ///
    /// `**` matches any number of chunks, so this covers the 9-chunk node token
    /// and the 13-chunk entity token alike — which no single-`*` pattern can.
    ///
    /// This width was tried once for the liveliness GET form and made things
    /// worse there (two tokens arrived, none of them a node token), because a
    /// get is an interest whose replies the router tags per interest id. A
    /// SUBSCRIBER has no such indirection: it simply matches, and the measured
    /// result is the whole graph rather than an arbitrary handful. Use this for
    /// the cache and the chunk-exact patterns for anything still querying.
    pub fn graph_keyexpr_wildcard<const N: usize>(domain_id: u32) -> heapless::String<N> {
        let mut out = heapless::String::new();
        let _ = core::fmt::write(
            &mut out,
            format_args!("{}/{}/**", LIVELINESS_PREFIX, domain_id),
        );
        out
    }

    /// phase-381 W3 — wildcard ENTITY liveliness keyexpr, all four kinds.
    ///
    /// NOTE (issue 0903): a `**` pattern was tried here to collapse this and
    /// the node wildcard into ONE standing query, because two concurrent
    /// liveliness gets do not both receive replies. It made things WORSE —
    /// measured, `**` delivered 2 tokens and none of them a node token, so node
    /// enumeration regressed from working to empty. Reverted; the concurrency
    /// problem is real and is NOT solved by widening the pattern.
    ///
    /// Format: `@ros2_lv/<domain_id>/*/*/*/*/%/*/*/*/*/*/*` — 13 chunks, every
    /// one wildcarded but the prefix and the domain, including the KIND, so a
    /// single standing query answers questions about publishers, subscribers,
    /// servers and clients alike. Four separate queries would put four sweeps on
    /// the wire to ask four things about one graph.
    ///
    /// Deliberately NOT merged with `node_keyexpr_wildcard`: a node token is 9
    /// chunks and an entity token is 13, and a single-`*` wildcard matches
    /// exactly one chunk, so no one pattern covers both shapes.
    pub fn entity_keyexpr_wildcard<const N: usize>(domain_id: u32) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let _ = core::fmt::write(
            &mut key,
            format_args!("{}/{}/*/*/*/*/%/*/*/*/*/*/*", LIVELINESS_PREFIX, domain_id),
        );
        key
    }

    /// Phase 108.C.zenoh.4 — wildcard publisher liveliness keyexpr.
    ///
    /// Format: `@ros2_lv/<domain_id>/*/*/*/MP/%/*/*/<topic>/<type>/*/*`
    /// — wildcards on zid, namespace, node, type_hash, qos. Used by
    /// `ZenohSubscriber`'s LivelinessChanged emulation: the subscriber
    /// periodically polls this keyexpr via `liveliness_get_start` to
    /// discover any publisher matching its own (topic, type) pair, and
    /// fires `LivelinessChanged` on alive-state transitions.
    pub fn publisher_keyexpr_wildcard<const N: usize>(
        domain_id: u32,
        topic: &TopicInfo,
    ) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let topic_mangled = Self::mangle_topic_name::<MANGLED_NAME_SIZE>(topic.name);
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/*/{}/{}/%/*/*/{}/{}/*/*",
                LIVELINESS_PREFIX,
                domain_id,
                ENTITY_ANY,
                ENTITY_PUBLISHER,
                topic_mangled.as_str(),
                crate::keyexpr::DdsTypeName(topic.type_name),
            ),
        );
        key
    }

    /// Format: `@ros2_lv/<domain_id>/*/*/*/SS/%/*/*/<service>/<type>/*/*`
    /// — wildcards on zid, namespace, node, type_hash, and qos. Used by
    /// `Client::wait_for_service` to discover any matching server before
    /// the first request, mirroring `rclcpp::ClientBase::wait_for_service`.
    pub fn service_server_keyexpr_wildcard<const N: usize>(
        domain_id: u32,
        service: &ServiceInfo,
    ) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let service_mangled = Self::mangle_topic_name::<MANGLED_NAME_SIZE>(service.name);
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/*/{}/{}/%/*/*/{}/{}/*/*",
                LIVELINESS_PREFIX,
                domain_id,
                ENTITY_ANY,
                ENTITY_SERVICE_SERVER,
                service_mangled.as_str(),
                crate::keyexpr::DdsTypeName(service.type_name),
            ),
        );
        key
    }

    /// Demangle a `%`-separated name back to a ROS name (`%demo` -> `/demo`).
    ///
    /// The inverse of `mangle_topic_name` (private), and deliberately written
    /// beside it: the two are one format, and the round-trip test below is what
    /// keeps them one.
    pub fn demangle_topic_name<const N: usize>(mangled: &str) -> heapless::String<N> {
        let mut out = heapless::String::new();
        for c in mangled.chars() {
            let _ = out.push(if c == '%' { '/' } else { c });
        }
        out
    }

    /// Parse a `@ros2_lv` liveliness keyexpr — phase-381 W2.
    ///
    /// ```text
    /// @ros2_lv/<domain>/<zid>/<node_id>/<entity_id>/<kind>/%/<ns>/<node>
    /// @ros2_lv/<domain>/<zid>/<node_id>/<entity_id>/<kind>/%/<ns>/<node>/<topic>/<type>/<hash>/<qos>
    /// ```
    ///
    /// **Refuses rather than guesses.** A graph query that returns a plausible
    /// wrong answer is worse than one that returns none, so every deviation is
    /// `None`: an unknown entity kind, a missing chunk, a non-numeric domain or
    /// id, the wrong prefix, or an entity kind whose chunk count does not match
    /// its shape. Nothing is defaulted and nothing is inferred.
    ///
    /// Strings are BORROWED from `key` — no allocation, and the mangled forms
    /// are returned as they appear on the wire. Demangle with
    /// [`Self::demangle_topic_name`] when a ROS name is wanted; doing it here
    /// would need a buffer this seam has nowhere to put.
    ///
    /// The grammar is `rmw_zenoh_cpp`'s and is pinned by ROUND TRIP against the
    /// builders above rather than by a copy of the spec: whatever we declare,
    /// this parses, and the tests assert exactly that. A builder change that
    /// this parser does not follow fails there.
    pub fn parse(key: &str) -> Option<LivelinessEntity<'_>> {
        let mut it = key.split('/');
        if it.next()? != LIVELINESS_PREFIX {
            return None;
        }
        let domain_id: u32 = it.next()?.parse().ok()?;
        let zid = it.next()?;
        let node_id: u32 = it.next()?.parse().ok()?;
        let entity_id: u32 = it.next()?.parse().ok()?;
        let kind = EntityKind::from_token(it.next()?)?;
        // The enclave field. `rmw_zenoh_cpp` mangles it like a name and ours is
        // always the root, but its PRESENCE is structural — without it every
        // field after this point shifts by one and still parses, which is the
        // silent-wrong-answer shape this parser exists to avoid.
        let enclave = it.next()?;
        if enclave.is_empty() {
            return None;
        }
        let namespace = it.next()?;
        let node_name = it.next()?;
        if node_name.is_empty() {
            return None;
        }

        let rest = (it.next(), it.next(), it.next(), it.next());
        let (topic, type_name, type_hash, qos) = match (kind, rest) {
            // A node token ends at the node name.
            (EntityKind::Node, (None, None, None, None)) => (None, None, None, None),
            // Every other kind carries its topic/service tail, all four fields.
            (EntityKind::Node, _) => return None,
            (_, (Some(t), Some(ty), Some(h), Some(q))) => (Some(t), Some(ty), Some(h), Some(q)),
            _ => return None,
        };
        // Anything after the tail is a shape this parser does not know.
        if it.next().is_some() {
            return None;
        }

        Some(LivelinessEntity {
            domain_id,
            zid,
            node_id,
            entity_id,
            kind,
            namespace,
            node_name,
            topic,
            type_name,
            type_hash,
            qos,
        })
    }

    /// phase-381 W3 — the mangler, reachable from outside this module.
    ///
    /// A wrapper rather than making `mangle_topic_name` public, so the callers
    /// inside this file keep using the short name and there is still exactly
    /// ONE implementation. Graph queries need it to compare a caller's ROS topic
    /// name against the mangled form on the wire.
    pub fn mangle_topic_name_pub<const N: usize>(topic: &str) -> heapless::String<N> {
        Self::mangle_topic_name::<N>(topic)
    }

    /// Mangle a topic name by replacing '/' with '%'
    fn mangle_topic_name<const N: usize>(topic: &str) -> heapless::String<N> {
        let mut mangled = heapless::String::new();
        for c in topic.chars() {
            if c == '/' {
                let _ = mangled.push('%');
            } else {
                let _ = mangled.push(c);
            }
        }
        mangled
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    // Phase 212.x3 — `TopicKeyExpr` is only re-exported as `QosKeyExpr` at the
    // module top, so the tests that call `topic.to_key()` / `topic.to_key_wildcard()`
    // need it in scope explicitly. Without this, every platform-feature build of
    // `cargo check --tests` fails E0599 (default features skip the `shim` module
    // entirely and so don't surface it).
    use crate::keyexpr::TopicKeyExpr;
    // `core::write!` into a `heapless::String` needs the trait in scope.
    use core::fmt::Write as _;

    // ========================================================================
    // Error Conversion Tests
    // ========================================================================

    #[test]
    fn test_error_conversion() {
        assert_eq!(
            TransportError::from(ZpicoError::Config),
            TransportError::InvalidArgument
        );
        assert_eq!(
            TransportError::from(ZpicoError::Publish),
            TransportError::PublishFailed
        );
    }

    // ========================================================================
    // C.6 RMW Zenoh Protocol Verification Tests
    // ========================================================================

    // ------------------------------------------------------------------------
    // RMW Attachment Format Tests
    // ------------------------------------------------------------------------

    #[test]
    fn test_rmw_attachment_serialization() {
        let mut att = RmwAttachment::new();
        att.sequence_number = 42;
        att.timestamp = 1000000;

        let mut buf = [0u8; RMW_ATTACHMENT_SIZE];
        att.serialize(&mut buf);

        // Check sequence number (little-endian)
        assert_eq!(&buf[0..8], &42i64.to_le_bytes());
        // Check timestamp (little-endian)
        assert_eq!(&buf[8..16], &1000000i64.to_le_bytes());
        // Check VLE length
        assert_eq!(buf[16], 16);
        // Check GID (should match)
        assert_eq!(&buf[17..33], &att.rmw_gid);
    }

    #[test]
    fn test_rmw_attachment_deserialization() {
        // Create known attachment bytes
        let mut buf = [0u8; RMW_ATTACHMENT_SIZE];
        // Sequence number: 123
        buf[0..8].copy_from_slice(&123i64.to_le_bytes());
        // Timestamp: 456789
        buf[8..16].copy_from_slice(&456789i64.to_le_bytes());
        // VLE length: 16
        buf[16] = 16;
        // GID: 0x01, 0x02, ..., 0x10
        for i in 0..16 {
            buf[17 + i] = (i + 1) as u8;
        }

        let parsed = RmwAttachment::deserialize(&buf);
        assert!(parsed.is_some());
        let att = parsed.unwrap();

        assert_eq!(att.sequence_number, 123);
        assert_eq!(att.timestamp, 456789);
        for i in 0..16 {
            assert_eq!(att.rmw_gid[i], (i + 1) as u8);
        }
    }

    #[test]
    fn test_rmw_attachment_roundtrip() {
        let original = RmwAttachment {
            sequence_number: 999,
            timestamp: 1234567890,
            rmw_gid: [
                0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                0x88, 0x99,
            ],
        };

        let mut buf = [0u8; RMW_ATTACHMENT_SIZE];
        original.serialize(&mut buf);

        let parsed = RmwAttachment::deserialize(&buf).expect("Failed to deserialize");
        assert_eq!(parsed.sequence_number, original.sequence_number);
        assert_eq!(parsed.timestamp, original.timestamp);
        assert_eq!(parsed.rmw_gid, original.rmw_gid);
    }

    #[test]
    fn test_rmw_attachment_deserialize_too_short() {
        let buf = [0u8; 10]; // Too short
        assert!(RmwAttachment::deserialize(&buf).is_none());
    }

    #[test]
    fn test_rmw_attachment_deserialize_wrong_gid_length() {
        let mut buf = [0u8; RMW_ATTACHMENT_SIZE];
        buf[16] = 8; // Wrong GID length (should be 16)
        assert!(RmwAttachment::deserialize(&buf).is_none());
    }

    #[test]
    fn test_message_info_from_attachment() {
        let mut buf = [0u8; RMW_ATTACHMENT_SIZE];
        buf[0..8].copy_from_slice(&42i64.to_le_bytes());
        buf[8..16].copy_from_slice(&1000000i64.to_le_bytes());
        buf[16] = 16;

        let info = MessageInfo::from_attachment(&buf);
        assert!(info.is_some());
        let info = info.unwrap();
        assert_eq!(info.sequence_number, 42);
        assert_eq!(info.timestamp_ns, 1000000);
    }

    // ------------------------------------------------------------------------
    // Liveliness Token Format Tests
    // ------------------------------------------------------------------------

    #[test]
    fn test_ros2_liveliness_mangle() {
        let mangled = Ros2Liveliness::mangle_topic_name::<MANGLED_NAME_SIZE>("/chatter");
        assert_eq!(mangled.as_str(), "%chatter");

        let mangled2 = Ros2Liveliness::mangle_topic_name::<MANGLED_NAME_SIZE>("/foo/bar/baz");
        assert_eq!(mangled2.as_str(), "%foo%bar%baz");
    }

    #[test]
    fn test_ros2_liveliness_node_keyexpr() {
        let zid = ZenohId::from_bytes([
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
            0x0f, 0x10,
        ]);
        // Root namespace: "/" mangles to "%"
        let keyexpr = Ros2Liveliness::node_keyexpr::<256>(0, &zid, "/", "my_node");

        // Format: @ros2_lv/<domain_id>/<zid>/0/0/NN/%/<mangled_ns>/<node_name>
        // ZID is in LSB-first order
        assert!(keyexpr.as_str().starts_with("@ros2_lv/0/"));
        assert!(keyexpr.as_str().contains("/0/0/NN/%/%/"));
        assert!(keyexpr.as_str().ends_with("/my_node"));
    }

    #[test]
    fn test_ros2_liveliness_node_keyexpr_with_namespace() {
        let zid = ZenohId::from_bytes([0u8; 16]);

        // Non-root namespace: "/demo" mangles to "%demo"
        let keyexpr = Ros2Liveliness::node_keyexpr::<256>(0, &zid, "/demo", "talker");
        assert!(keyexpr.as_str().contains("/0/0/NN/%/%demo/talker"));

        // Nested namespace: "/ns/sub" mangles to "%ns%sub"
        let keyexpr2 = Ros2Liveliness::node_keyexpr::<256>(0, &zid, "/ns/sub", "my_node");
        assert!(keyexpr2.as_str().contains("/0/0/NN/%/%ns%sub/my_node"));
    }

    /// Zenoh's chunk matcher, enough of it to state the property below: a
    /// single `*` matches exactly ONE chunk, everything else is literal.
    #[cfg(test)]
    fn keyexpr_matches(pattern: &str, key: &str) -> bool {
        let mut p = pattern.split('/');
        let mut k = key.split('/');
        loop {
            match (p.next(), k.next()) {
                (None, None) => return true,
                (Some(pc), Some(kc)) if pc == "*" || pc == kc => continue,
                _ => return false,
            }
        }
    }

    /// phase-381 W3 — the grouping `names_and_types` performs, as a property.
    ///
    /// `ZenohSession::names_and_types` needs a live session to drain a query, so
    /// what is tested here is the part that can be wrong WITHOUT one: given the
    /// entities a graph yields, does grouping produce one entry per distinct
    /// NAME carrying that name's DISTINCT types? That is the dedup, and it is
    /// where a two-publisher topic becomes two entries or a two-type topic
    /// loses one.
    ///
    /// This mirrors the session helper's algorithm rather than calling it, and
    /// says so: the alternative is no coverage until an interop lane exists,
    /// and the shapes it pins — duplicate topic, duplicate type, two types on
    /// one topic — are exactly the ones reading does not catch.
    #[test]
    fn grouping_reports_each_name_once_with_its_distinct_types() {
        // A graph with two publishers and one subscriber on /chatter, plus a
        // second topic, as (topic, type) pairs.
        let entities = [
            ("%chatter", "std_msgs::msg::dds_::String_"),
            ("%chatter", "std_msgs::msg::dds_::String_"), // second publisher, same type
            ("%chatter", "std_msgs::msg::dds_::Int32_"),  // a second type on one topic
            ("%tf", "tf2_msgs::msg::dds_::TFMessage_"),
        ];

        let mut seen: heapless::Vec<&str, 8> = heapless::Vec::new();
        let mut reported: heapless::Vec<(&str, heapless::Vec<&str, 8>), 8> = heapless::Vec::new();
        while let Some((name, _)) = entities.iter().find(|(n, _)| !seen.contains(n)) {
            let _ = seen.push(name);
            let mut types: heapless::Vec<&str, 8> = heapless::Vec::new();
            for (n, t) in entities.iter() {
                if n == name && !types.contains(t) {
                    let _ = types.push(t);
                }
            }
            let _ = reported.push((name, types));
        }

        assert_eq!(
            reported.len(),
            2,
            "one entry per DISTINCT name, not per entity"
        );
        let chatter = reported
            .iter()
            .find(|(n, _)| *n == "%chatter")
            .expect("/chatter");
        assert_eq!(
            chatter.1.len(),
            2,
            "two types on one topic report both, once each — the duplicate publisher must not add a third"
        );
        let tf = reported.iter().find(|(n, _)| *n == "%tf").expect("/tf");
        assert_eq!(tf.1.len(), 1);

        // And the name is demangled on the way out.
        assert_eq!(
            Ros2Liveliness::demangle_topic_name::<64>("%chatter").as_str(),
            "/chatter"
        );
    }

    /// phase-381 W3 — the entity wildcard must match every entity KIND and id.
    ///
    /// One standing query serves all four kinds. If the kind chunk were pinned,
    /// three of the four slots would silently report nothing — the same failure
    /// as issue 0890, one field over.
    #[test]
    fn entity_wildcard_matches_all_four_kinds() {
        let zid = ZenohId::from_bytes([0u8; 16]);
        let topic = TopicInfo::new("/chatter", "std_msgs::msg::dds_::String_", "RIHS01_abc");
        let service = ServiceInfo::new(
            "/add",
            "example_interfaces::srv::dds_::AddTwoInts_",
            "RIHS01_s",
        );
        let qos = QoSProfile::QOS_PROFILE_SENSOR_DATA;
        let wildcard = Ros2Liveliness::entity_keyexpr_wildcard::<256>(0);

        let declared = [
            Ros2Liveliness::publisher_keyexpr::<256>(0, &zid, 1, "/", "n", &topic, &qos),
            Ros2Liveliness::subscriber_keyexpr::<256>(0, &zid, 2, "/", "n", &topic, &qos),
            Ros2Liveliness::service_server_keyexpr::<256>(0, &zid, 3, "/", "n", &service, &qos),
            Ros2Liveliness::service_client_keyexpr::<256>(0, &zid, 4, "/", "n", &service, &qos),
        ];
        for d in &declared {
            assert!(
                keyexpr_matches(wildcard.as_str(), d.as_str()),
                "entity wildcard {} must match {}",
                wildcard.as_str(),
                d.as_str()
            );
        }

        // And it must NOT match a node token — different shape, different query.
        let node = Ros2Liveliness::node_keyexpr::<256>(0, &zid, "/", "n");
        assert!(
            !keyexpr_matches(wildcard.as_str(), node.as_str()),
            "a 13-chunk wildcard must not match a 9-chunk node token"
        );
    }

    /// phase-381 W2 — the parser is pinned to the BUILDERS by round trip.
    ///
    /// The grammar is `rmw_zenoh_cpp`'s, and the risk W2 names is a parser that
    /// returns a plausible WRONG answer. Testing it against hand-written
    /// literals would pin it to my reading of the spec; testing it against the
    /// builders pins it to what we actually put on the wire, so a builder change
    /// this parser does not follow fails HERE rather than in interop.
    #[test]
    fn parse_round_trips_every_builder() {
        let zid = ZenohId::from_bytes([0xABu8; 16]);
        let topic = TopicInfo::new("/chatter", "std_msgs::msg::dds_::String_", "RIHS01_abc123");
        let service = ServiceInfo::new(
            "/add_two_ints",
            "example_interfaces::srv::dds_::AddTwoInts_",
            "RIHS01_svc",
        );
        let qos = QoSProfile::QOS_PROFILE_SENSOR_DATA;

        // Node: no topic tail.
        let k = Ros2Liveliness::node_keyexpr::<256>(7, &zid, "/demo", "talker");
        let e = Ros2Liveliness::parse(k.as_str()).expect("node token must parse");
        assert_eq!(e.kind, EntityKind::Node);
        assert_eq!(e.domain_id, 7);
        assert_eq!((e.node_id, e.entity_id), (0, 0), "nodes are the fixed 0/0");
        assert_eq!(e.namespace, "%demo");
        assert_eq!(e.node_name, "talker");
        assert!(e.topic.is_none(), "a node token has no topic tail");
        assert_eq!(
            Ros2Liveliness::demangle_topic_name::<64>(e.namespace).as_str(),
            "/demo",
            "the demangler is the mangler's inverse"
        );

        // Publisher / subscriber: full tail, and the entity id survives.
        for (build, want) in [
            (
                Ros2Liveliness::publisher_keyexpr::<256>(3, &zid, 42, "/", "n", &topic, &qos),
                EntityKind::Publisher,
            ),
            (
                Ros2Liveliness::subscriber_keyexpr::<256>(3, &zid, 43, "/", "n", &topic, &qos),
                EntityKind::Subscriber,
            ),
        ] {
            let e = Ros2Liveliness::parse(build.as_str()).expect("entity token must parse");
            assert_eq!(e.kind, want);
            assert_eq!(e.domain_id, 3);
            assert_eq!(e.node_name, "n");
            assert_eq!(e.topic, Some("%chatter"));
            assert_eq!(e.type_name, Some("std_msgs::msg::dds_::String_"));
            assert_eq!(e.type_hash, Some("RIHS01_abc123"));
            assert!(e.qos.is_some());
        }
        // Bound rather than inlined: `LivelinessEntity` BORROWS from the
        // keyexpr, so the keyexpr has to outlive it — the borrow checker
        // enforcing the no-allocation design.
        let pub_key = Ros2Liveliness::publisher_keyexpr::<256>(3, &zid, 42, "/", "n", &topic, &qos);
        let e = Ros2Liveliness::parse(pub_key.as_str()).unwrap();
        assert_eq!(e.entity_id, 42, "the entity id must survive the round trip");

        // Services.
        for (build, want) in [
            (
                Ros2Liveliness::service_server_keyexpr::<256>(1, &zid, 5, "/", "s", &service, &qos),
                EntityKind::ServiceServer,
            ),
            (
                Ros2Liveliness::service_client_keyexpr::<256>(1, &zid, 6, "/", "s", &service, &qos),
                EntityKind::ServiceClient,
            ),
        ] {
            let e = Ros2Liveliness::parse(build.as_str()).expect("service token must parse");
            assert_eq!(e.kind, want);
            assert_eq!(e.topic, Some("%add_two_ints"));
        }
    }

    /// Every way the parser must REFUSE.
    ///
    /// Each case is a keyexpr that a lenient parser would accept and report as
    /// a real entity — which is the failure this phase is most concerned with,
    /// because the answer looks like an answer.
    #[test]
    fn parse_refuses_rather_than_guessing() {
        let zid = ZenohId::from_bytes([0u8; 16]);
        let good = Ros2Liveliness::node_keyexpr::<256>(0, &zid, "/", "n");
        assert!(Ros2Liveliness::parse(good.as_str()).is_some(), "control");

        for (key, why) in [
            ("@ros2_hb/0/aa/0/0/NN/%/%/n", "a different prefix"),
            ("@ros2_lv/x/aa/0/0/NN/%/%/n", "a non-numeric domain"),
            ("@ros2_lv/0/aa/0/x/NN/%/%/n", "a non-numeric entity id"),
            (
                "@ros2_lv/0/aa/0/0/ZZ/%/%/n",
                "an entity kind we do not know",
            ),
            (
                "@ros2_lv/0/aa/0/0/NN/%/%",
                "a node token missing its node name",
            ),
            (
                "@ros2_lv/0/aa/0/0/NN",
                "a token that stops before the enclave",
            ),
            (
                "@ros2_lv/0/aa/0/0/NN/%/%/n/%chatter/T/H/Q",
                "a NODE token carrying a topic tail",
            ),
            (
                "@ros2_lv/0/aa/0/1/MP/%/%/n/%chatter/T/H",
                "an entity token missing one tail field",
            ),
            (
                "@ros2_lv/0/aa/0/1/MP/%/%/n/%chatter/T/H/Q/extra",
                "a trailing chunk in a shape we do not know",
            ),
        ] {
            assert!(
                Ros2Liveliness::parse(key).is_none(),
                "must refuse {why}: {key}"
            );
        }
    }

    /// Issue 0890 — a DISCOVERY wildcard must match an entity whatever its id.
    ///
    /// The wildcard builders used to put `PROTO_VERSION_TOPIC` ("0/11") in the
    /// `<node_id>/<entity_id>` position. Issue 0292 had already measured what
    /// that literal costs on the DECLARATION side — every entity of an action
    /// server collided at id 11 and deduped to one — and fixed the four
    /// declaration builders, leaving the two wildcard builders behind. On this
    /// side the same literal is not a collision but a FILTER: after 0292 ids
    /// start at 1 and increment, so the query matched only a session's
    /// eleventh entity.
    ///
    /// The test is the property, not the spelling: build a declaration with an
    /// arbitrary id and require the wildcard to match it.
    #[test]
    fn publisher_wildcard_matches_any_entity_id() {
        let zid = ZenohId::from_bytes([0u8; 16]);
        let topic = TopicInfo::new("/chatter", "std_msgs::msg::dds_::String_", "RIHS01_abc123");
        let qos = QoSProfile::QOS_PROFILE_SENSOR_DATA;
        let wildcard = Ros2Liveliness::publisher_keyexpr_wildcard::<256>(0, &topic);

        // 11 included deliberately: it is the one id the old code DID match, so
        // a regression that re-pins the field still passes on 11 alone.
        for entity_id in [1u32, 2, 7, 11, 4096] {
            let declared = Ros2Liveliness::publisher_keyexpr::<256>(
                0, &zid, entity_id, "/", "my_node", &topic, &qos,
            );
            assert!(
                keyexpr_matches(wildcard.as_str(), declared.as_str()),
                "wildcard {} must match a publisher declared with entity_id {}: {}",
                wildcard.as_str(),
                entity_id,
                declared.as_str(),
            );
        }
    }

    /// The same property for the service-server wildcard, which
    /// `Client::wait_for_service` polls.
    #[test]
    fn service_server_wildcard_matches_any_entity_id() {
        let zid = ZenohId::from_bytes([0u8; 16]);
        let service = ServiceInfo::new(
            "/add_two_ints",
            "example_interfaces::srv::dds_::AddTwoInts_",
            "RIHS01_svc",
        );
        let qos = QoSProfile::QOS_PROFILE_SERVICES_DEFAULT;
        let wildcard = Ros2Liveliness::service_server_keyexpr_wildcard::<256>(0, &service);

        for entity_id in [1u32, 3, 11, 900] {
            let declared = Ros2Liveliness::service_server_keyexpr::<256>(
                0, &zid, entity_id, "/", "srv_node", &service, &qos,
            );
            assert!(
                keyexpr_matches(wildcard.as_str(), declared.as_str()),
                "wildcard {} must match a server declared with entity_id {}: {}",
                wildcard.as_str(),
                entity_id,
                declared.as_str(),
            );
        }
    }

    #[test]
    fn test_ros2_liveliness_publisher_keyexpr() {
        let zid = ZenohId::from_bytes([0u8; 16]);
        let topic = TopicInfo::new("/chatter", "std_msgs::msg::dds_::String_", "RIHS01_abc123");
        let qos = QoSProfile::QOS_PROFILE_SENSOR_DATA;
        let keyexpr =
            Ros2Liveliness::publisher_keyexpr::<256>(0, &zid, 11, "/", "my_node", &topic, &qos);

        // Format: @ros2_lv/<domain_id>/<zid>/0/11/MP/%/<mangled_ns>/<node_name>/<mangled_topic>/<type>/<hash>/<qos>
        assert!(keyexpr.as_str().starts_with("@ros2_lv/0/"));
        assert!(keyexpr.as_str().contains("/0/11/MP/%/%/"));
        assert!(keyexpr.as_str().contains("/my_node/"));
        assert!(keyexpr.as_str().contains("%chatter/"));
        assert!(keyexpr.as_str().contains("std_msgs::msg::dds_::String_"));
        assert!(keyexpr.as_str().contains("RIHS01_abc123"));
    }

    #[test]
    fn test_ros2_liveliness_publisher_keyexpr_with_namespace() {
        let zid = ZenohId::from_bytes([0u8; 16]);
        let topic = TopicInfo::new("/chatter", "std_msgs::msg::dds_::String_", "RIHS01_abc123");
        let qos = QoSProfile::QOS_PROFILE_SENSOR_DATA;
        let keyexpr =
            Ros2Liveliness::publisher_keyexpr::<256>(0, &zid, 11, "/demo", "talker", &topic, &qos);
        assert!(keyexpr.as_str().contains("/0/11/MP/%/%demo/talker/"));
    }

    #[test]
    fn test_ros2_liveliness_subscriber_keyexpr() {
        let zid = ZenohId::from_bytes([0u8; 16]);
        let topic = TopicInfo::new("/chatter", "std_msgs::msg::dds_::Int32_", "RIHS01_def456");
        let qos = QoSProfile::QOS_PROFILE_SENSOR_DATA;
        let keyexpr =
            Ros2Liveliness::subscriber_keyexpr::<256>(0, &zid, 11, "/", "my_node", &topic, &qos);

        // Format: @ros2_lv/<domain_id>/<zid>/0/11/MS/%/<mangled_ns>/<node_name>/<mangled_topic>/<type>/<hash>/<qos>
        assert!(keyexpr.as_str().starts_with("@ros2_lv/0/"));
        assert!(keyexpr.as_str().contains("/0/11/MS/%/%/"));
        assert!(keyexpr.as_str().contains("/my_node/"));
        assert!(keyexpr.as_str().contains("%chatter/"));
    }

    #[test]
    fn test_ros2_liveliness_service_server_keyexpr() {
        let zid = ZenohId::from_bytes([0u8; 16]);
        let service = ServiceInfo::new(
            "/add_two_ints",
            "example_interfaces::srv::dds_::AddTwoInts",
            "RIHS01_abc123",
        );
        let qos = QoSProfile::QOS_PROFILE_SERVICES_DEFAULT;
        let keyexpr = Ros2Liveliness::service_server_keyexpr::<256>(
            0, &zid, 11, "/", "my_node", &service, &qos,
        );

        // Format: @ros2_lv/<domain_id>/<zid>/0/11/SS/%/<mangled_ns>/<node_name>/<mangled_service>/<type>/<hash>/<qos>
        assert!(keyexpr.as_str().starts_with("@ros2_lv/0/"));
        assert!(keyexpr.as_str().contains("/0/11/SS/%/%/"));
        assert!(keyexpr.as_str().contains("/my_node/"));
        assert!(keyexpr.as_str().contains("%add_two_ints/"));
    }

    #[test]
    fn test_ros2_liveliness_service_server_keyexpr_with_namespace() {
        let zid = ZenohId::from_bytes([0u8; 16]);
        let service = ServiceInfo::new(
            "/add_two_ints",
            "example_interfaces::srv::dds_::AddTwoInts",
            "RIHS01_abc123",
        );
        let qos = QoSProfile::QOS_PROFILE_SERVICES_DEFAULT;
        let keyexpr = Ros2Liveliness::service_server_keyexpr::<256>(
            0, &zid, 11, "/demo", "my_node", &service, &qos,
        );
        assert!(keyexpr.as_str().contains("/0/11/SS/%/%demo/my_node/"));
    }

    #[test]
    fn test_ros2_liveliness_service_client_keyexpr() {
        let zid = ZenohId::from_bytes([0u8; 16]);
        let service = ServiceInfo::new(
            "/add_two_ints",
            "example_interfaces::srv::dds_::AddTwoInts",
            "RIHS01_abc123",
        );
        let qos = QoSProfile::QOS_PROFILE_SERVICES_DEFAULT;
        let keyexpr = Ros2Liveliness::service_client_keyexpr::<256>(
            0, &zid, 11, "/", "my_node", &service, &qos,
        );

        // Format: @ros2_lv/<domain_id>/<zid>/0/11/SC/%/<mangled_ns>/<node_name>/<mangled_service>/<type>/<hash>/<qos>
        assert!(keyexpr.as_str().starts_with("@ros2_lv/0/"));
        assert!(keyexpr.as_str().contains("/0/11/SC/%/%/"));
        assert!(keyexpr.as_str().contains("/my_node/"));
        assert!(keyexpr.as_str().contains("%add_two_ints/"));
    }

    // ------------------------------------------------------------------------
    // Data KeyExpr Format Tests
    // ------------------------------------------------------------------------

    #[test]
    fn test_topic_info_to_key_humble() {
        let topic = TopicInfo::new(
            "/chatter",
            "std_msgs::msg::dds_::Int32_",
            "TypeHashNotSupported",
        );

        let key: heapless::String<128> = topic.to_key();
        // Format: <domain_id>/<topic_name>/<type_name>/<type_hash>
        assert_eq!(
            key.as_str(),
            "0/chatter/std_msgs::msg::dds_::Int32_/TypeHashNotSupported"
        );
    }

    #[test]
    fn test_topic_info_to_key_wildcard() {
        let topic = TopicInfo::new(
            "/chatter",
            "std_msgs::msg::dds_::Int32_",
            "TypeHashNotSupported",
        );

        let key: heapless::String<128> = topic.to_key_wildcard();
        // Format: <domain_id>/<topic_name>/<type_name>/*
        assert_eq!(key.as_str(), "0/chatter/std_msgs::msg::dds_::Int32_/*");
    }

    // ------------------------------------------------------------------------
    // Service KeyExpr Format Tests
    // ------------------------------------------------------------------------

    #[test]
    fn test_service_info_format() {
        let service = ServiceInfo::new(
            "/add_two_ints",
            "example_interfaces::srv::dds_::AddTwoInts",
            "TypeHashNotSupported",
        );

        // Verify service info fields are correct
        assert_eq!(service.name, "/add_two_ints");
        // `DdsTypeName` is a Display ADAPTER, not a comparable value: it has no
        // `PartialEq`/`Debug`, so `assert_eq!` on it directly does not compile
        // (E0369 + E0277 — the whole test target of this crate was red on main).
        // Render it the way the sibling key tests render theirs, which also
        // keeps the assertion off `alloc`.
        let mut rendered: heapless::String<128> = heapless::String::new();
        core::write!(
            &mut rendered,
            "{}",
            crate::keyexpr::DdsTypeName(service.type_name)
        )
        .expect("DdsTypeName rendering must fit 128 bytes");
        // Already mangled (contains `::`), so it passes through untouched.
        assert_eq!(
            rendered.as_str(),
            "example_interfaces::srv::dds_::AddTwoInts"
        );
        assert_eq!(service.domain_id, 0);
    }

    // ------------------------------------------------------------------------
    // QoS String Encoding Tests
    // ------------------------------------------------------------------------

    #[test]
    fn test_qos_best_effort_volatile() {
        // BEST_EFFORT reliability (2), VOLATILE durability (2), KEEP_LAST history with depth 1
        let qos = "2:2:1,1:,:,:,,";
        assert!(qos.starts_with("2:")); // BEST_EFFORT
        assert!(qos.contains(":2:")); // VOLATILE
    }

    #[test]
    fn test_qos_reliable_transient_local() {
        // RELIABLE reliability (1), TRANSIENT_LOCAL durability (1)
        let qos = "1:1:1,1:,:,:,,";
        assert!(qos.starts_with("1:")); // RELIABLE
    }

    // ------------------------------------------------------------------------
    // ZenohId Format Tests
    // ------------------------------------------------------------------------

    #[test]
    fn test_zenoh_id_to_hex_lsb_first() {
        // Test that ZenohId is formatted in LSB-first order
        let zid = ZenohId::from_bytes([
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
            0x0f, 0x10,
        ]);
        let mut buf = [0u8; ZID_HEX_SIZE];
        zid.to_hex_bytes(&mut buf);
        let hex = core::str::from_utf8(&buf).unwrap();

        // LSB-first: byte 15 first, byte 0 last
        assert_eq!(hex, "100f0e0d0c0b0a090807060504030201");
    }
}

// =============================================================================
// Ghost model validation
// =============================================================================

#[cfg(test)]
mod ghost_checks {
    use super::*;
    use nros_ghost_types::{ServiceBufferGhost, SubscriberBufferGhost};
    use service::ServiceBuffer;
    use subscriber::SubscriberBuffer;

    /// Structural check: project the Phase 124.D.3.c SPSC-ring
    /// `SubscriberBuffer` onto the ghost shape. `overflow` / `locked`
    /// no longer exist as fields — the ring is lock-free and drops
    /// oversized payloads at the producer — so they project to
    /// `false`. `stored_len` reads the head slot's length.
    fn ghost_from_buffer(b: &SubscriberBuffer) -> SubscriberBufferGhost {
        let head_slot = b.ring_head.load(Ordering::Relaxed) % SUBSCRIBER_RING_DEPTH;
        SubscriberBufferGhost {
            has_data: b.has_data(),
            overflow: false,
            locked: false,
            stored_len: b.ring_len[head_slot],
            buf_capacity: SUBSCRIBER_BUFFER_SIZE,
        }
    }

    #[test]
    fn ghost_new_state() {
        let buffer = SubscriberBuffer::new();
        let ghost = ghost_from_buffer(&buffer);
        assert!(!ghost.has_data);
        assert!(!ghost.overflow);
        assert!(!ghost.locked);
        assert_eq!(ghost.stored_len, 0);
        assert_eq!(ghost.buf_capacity, SUBSCRIBER_BUFFER_SIZE);
    }

    #[test]
    fn ghost_capacity_constant() {
        let buffer = SubscriberBuffer::new();
        let ghost = ghost_from_buffer(&buffer);
        assert_eq!(ghost.buf_capacity, SUBSCRIBER_BUFFER_SIZE);
    }

    // ========================================================================
    // ServiceBufferGhost Correspondence
    // ========================================================================

    /// Structural check: construct ServiceBufferGhost from ServiceBuffer private fields.
    /// If a field is renamed or retyped, this fails to compile.
    fn ghost_from_service_buffer(b: &ServiceBuffer) -> ServiceBufferGhost {
        // Phase 237 follow-up — the request inbox is now a ring; map the head
        // entry's state into the ghost (`has_request` = ring non-empty).
        let head = b.head.load(Ordering::Relaxed);
        let tail = b.tail.load(Ordering::Relaxed);
        let slot = &b.ring[head % service::SERVICE_REQUEST_RING_DEPTH];
        ServiceBufferGhost {
            has_request: head != tail,
            overflow: slot.overflow.load(Ordering::Relaxed),
            stored_len: slot.len.load(Ordering::Relaxed),
            buf_capacity: slot.data.len(),
        }
    }

    #[test]
    fn ghost_service_new_state() {
        let buffer = ServiceBuffer::new();
        let ghost = ghost_from_service_buffer(&buffer);
        assert!(!ghost.has_request);
        assert!(!ghost.overflow);
        assert_eq!(ghost.stored_len, 0);
        assert_eq!(ghost.buf_capacity, SERVICE_BUFFER_SIZE);
    }

    #[test]
    fn ghost_service_capacity_constant() {
        let buffer = ServiceBuffer::new();
        let ghost = ghost_from_service_buffer(&buffer);
        assert_eq!(ghost.buf_capacity, SERVICE_BUFFER_SIZE);
    }

    #[test]
    fn svc_buf_overflow_signals_error() {
        let buffer = ServiceBuffer::new();
        // Simulate the callback enqueuing an oversized request into the ring.
        buffer.ring[0].overflow.store(true, Ordering::Release);
        buffer.tail.store(1, Ordering::Release);

        let ghost = ghost_from_service_buffer(&buffer);
        assert!(ghost.has_request);
        assert!(ghost.overflow);
    }

    // ========================================================================
    // E2E Safety Protocol Unit Tests
    // ========================================================================

    /// Helper: build a valid 37-byte safety attachment from seq, timestamp, and payload CRC.
    #[cfg(feature = "safety-e2e")]
    fn build_safety_attachment(
        seq: i64,
        ts: i64,
        payload: &[u8],
    ) -> [u8; RMW_ATTACHMENT_SIZE_WITH_CRC] {
        let crc = nros_rmw::crc32(payload);
        let mut att = [0u8; RMW_ATTACHMENT_SIZE_WITH_CRC];
        att[0..8].copy_from_slice(&seq.to_le_bytes());
        att[8..16].copy_from_slice(&ts.to_le_bytes());
        att[16] = RMW_GID_SIZE as u8; // VLE GID length
        // GID bytes 17..33 left as zero
        att[RMW_ATTACHMENT_SIZE..RMW_ATTACHMENT_SIZE_WITH_CRC].copy_from_slice(&crc.to_le_bytes());
        att
    }

    /// Helper: parse attachment bytes and validate CRC against payload.
    ///
    /// This mirrors the logic in `take_validated()` but is testable
    /// without creating a full `ZenohSubscriber` (which requires a zenoh session).
    #[cfg(feature = "safety-e2e")]
    fn validate_from_buffers(
        payload: &[u8],
        attachment: &[u8],
        validator: &mut nros_rmw::SafetyValidator,
    ) -> nros_rmw::IntegrityStatus {
        let (message_seq, crc_valid) = if attachment.len() >= RMW_ATTACHMENT_SIZE {
            let seq = i64::from_le_bytes([
                attachment[0],
                attachment[1],
                attachment[2],
                attachment[3],
                attachment[4],
                attachment[5],
                attachment[6],
                attachment[7],
            ]);

            let crc_result = if attachment.len() >= RMW_ATTACHMENT_SIZE + SAFETY_CRC_SIZE {
                let received_crc = u32::from_le_bytes([
                    attachment[RMW_ATTACHMENT_SIZE],
                    attachment[RMW_ATTACHMENT_SIZE + 1],
                    attachment[RMW_ATTACHMENT_SIZE + 2],
                    attachment[RMW_ATTACHMENT_SIZE + 3],
                ]);
                let computed_crc = nros_rmw::crc32(payload);
                Some(received_crc == computed_crc)
            } else {
                None
            };

            (seq, crc_result)
        } else {
            (0, None)
        };

        validator.validate(message_seq, crc_valid)
    }

    #[cfg(feature = "safety-e2e")]
    #[test]
    fn test_safety_validate_happy_path() {
        let payload = b"\x00\x01\x00\x00\x2a\x00\x00\x00"; // CDR-encoded Int32(42)
        let attachment = build_safety_attachment(0, 1000, payload);

        let mut validator = nros_rmw::SafetyValidator::new();
        let status = validate_from_buffers(payload, &attachment, &mut validator);

        assert!(status.is_valid());
        assert_eq!(status.crc_valid, Some(true));
        assert_eq!(status.gap, 0);
        assert!(!status.duplicate);
    }

    #[cfg(feature = "safety-e2e")]
    #[test]
    fn test_safety_validate_sequential_messages() {
        let mut validator = nros_rmw::SafetyValidator::new();

        for seq in 0..10i64 {
            let payload = seq.to_le_bytes();
            let attachment = build_safety_attachment(seq, seq * 1000, &payload);
            let status = validate_from_buffers(&payload, &attachment, &mut validator);

            assert!(status.is_valid(), "failed at seq {}: {:?}", seq, status);
            assert_eq!(status.gap, 0);
            assert!(!status.duplicate);
            assert_eq!(status.crc_valid, Some(true));
        }
    }

    #[cfg(feature = "safety-e2e")]
    #[test]
    fn test_safety_validate_tampered_crc() {
        let payload = b"hello world CDR data";
        let mut attachment = build_safety_attachment(0, 1000, payload);

        // Tamper with the CRC (flip a bit)
        attachment[RMW_ATTACHMENT_SIZE] ^= 0x01;

        let mut validator = nros_rmw::SafetyValidator::new();
        let status = validate_from_buffers(payload, &attachment, &mut validator);

        assert!(!status.is_valid());
        assert_eq!(status.crc_valid, Some(false));
    }

    #[cfg(feature = "safety-e2e")]
    #[test]
    fn test_safety_validate_tampered_payload() {
        let payload = b"original payload data";
        let attachment = build_safety_attachment(0, 1000, payload);

        // Tamper with the payload (simulating transport corruption)
        let mut tampered_payload = *payload;
        tampered_payload[0] ^= 0xFF;

        let mut validator = nros_rmw::SafetyValidator::new();
        let status = validate_from_buffers(&tampered_payload, &attachment, &mut validator);

        assert!(!status.is_valid());
        assert_eq!(status.crc_valid, Some(false));
    }

    #[cfg(feature = "safety-e2e")]
    #[test]
    fn test_safety_validate_sequence_gap() {
        let mut validator = nros_rmw::SafetyValidator::new();

        // First message: seq 0
        let payload0 = b"msg0";
        let att0 = build_safety_attachment(0, 0, payload0);
        let status = validate_from_buffers(payload0, &att0, &mut validator);
        assert!(status.is_valid());

        // seq 1
        let payload1 = b"msg1";
        let att1 = build_safety_attachment(1, 1000, payload1);
        let status = validate_from_buffers(payload1, &att1, &mut validator);
        assert!(status.is_valid());

        // Skip to seq 5 (gap of 3)
        let payload5 = b"msg5";
        let att5 = build_safety_attachment(5, 5000, payload5);
        let status = validate_from_buffers(payload5, &att5, &mut validator);

        assert!(!status.is_valid());
        assert_eq!(status.gap, 3);
        assert!(!status.duplicate);
        assert_eq!(status.crc_valid, Some(true));
    }

    #[cfg(feature = "safety-e2e")]
    #[test]
    fn test_safety_validate_duplicate() {
        let mut validator = nros_rmw::SafetyValidator::new();

        // seq 0, 1, 2
        for seq in 0..3i64 {
            let payload = seq.to_le_bytes();
            let att = build_safety_attachment(seq, seq * 1000, &payload);
            validate_from_buffers(&payload, &att, &mut validator);
        }

        // Receive seq 1 again (duplicate)
        let payload1 = 1i64.to_le_bytes();
        let att1 = build_safety_attachment(1, 1000, &payload1);
        let status = validate_from_buffers(&payload1, &att1, &mut validator);

        assert!(!status.is_valid());
        assert!(status.duplicate);
        assert_eq!(status.crc_valid, Some(true));
    }

    #[cfg(feature = "safety-e2e")]
    #[test]
    fn test_safety_validate_no_crc_interop() {
        // Simulate a 33-byte attachment (sender without safety-e2e)
        let payload = b"some data";
        let mut attachment = [0u8; RMW_ATTACHMENT_SIZE]; // Only 33 bytes, no CRC
        attachment[0..8].copy_from_slice(&0i64.to_le_bytes());
        attachment[8..16].copy_from_slice(&1000i64.to_le_bytes());
        attachment[16] = RMW_GID_SIZE as u8;

        let mut validator = nros_rmw::SafetyValidator::new();
        let status = validate_from_buffers(payload, &attachment, &mut validator);

        assert!(status.is_valid()); // No CRC is acceptable (interop)
        assert_eq!(status.crc_valid, None);
    }

    #[cfg(feature = "safety-e2e")]
    #[test]
    fn test_safety_attachment_format() {
        let payload = b"test payload for CRC";
        let expected_crc = nros_rmw::crc32(payload);

        let attachment = build_safety_attachment(42, 999999, payload);

        // Verify format: 37 bytes total
        assert_eq!(attachment.len(), 37);

        // Bytes 0..8: sequence number (LE)
        assert_eq!(i64::from_le_bytes(attachment[0..8].try_into().unwrap()), 42);

        // Bytes 8..16: timestamp (LE)
        assert_eq!(
            i64::from_le_bytes(attachment[8..16].try_into().unwrap()),
            999999
        );

        // Byte 16: GID VLE length
        assert_eq!(attachment[16], 16);

        // Bytes 33..37: CRC-32 of payload (LE)
        let crc = u32::from_le_bytes(attachment[33..37].try_into().unwrap());
        assert_eq!(crc, expected_crc);
    }

    // ========================================================================
    // 37.1a: Cross-buffer interaction tests
    // ========================================================================

    #[test]
    fn sub_svc_independent() {
        use service::tests::{reset_service_buffer, simulate_service_request, take_service};
        use subscriber::tests::{
            reset_subscriber_buffer, simulate_subscription_callback, take_subscription,
        };

        // Use slot 2 — subscription array and service array are separate
        let slot = 2;
        reset_subscriber_buffer(slot);
        reset_service_buffer(slot);

        simulate_subscription_callback(slot, b"sub_data");
        simulate_service_request(slot, b"svc_data", b"svc/x");

        let mut recv_buf = [0u8; 1024];
        let sub_result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(sub_result, Ok(Some(8))));
        assert_eq!(&recv_buf[..8], b"sub_data");

        let svc_result = take_service(slot, &mut recv_buf);
        assert!(matches!(svc_result, Ok(Some(8))));
        assert_eq!(&recv_buf[..8], b"svc_data");
    }

    #[test]
    fn sub_overflow_does_not_affect_svc() {
        use service::tests::{reset_service_buffer, simulate_service_request, take_service};
        use subscriber::tests::{
            reset_subscriber_buffer, simulate_subscription_callback, take_subscription,
        };

        let slot = 3;
        reset_subscriber_buffer(slot);
        reset_service_buffer(slot);

        // Subscription overflow — payload (2000 B) exceeds
        // SUBSCRIBER_BUFFER_SIZE (1024 B); `simulate_subscription_callback`
        // silently drops it (matches the C producer's behavior — overflow
        // is observable via `overflow_drops_total` per Phase 160.L.2, not
        // via the `take` path).
        simulate_subscription_callback(slot, &[0u8; 2000]);
        // Service normal request
        simulate_service_request(slot, b"svc_ok", b"svc/x");

        // Subscription ring stays empty (oversized payload was dropped).
        let mut recv_buf = [0u8; 1024];
        let sub_result = take_subscription(slot, &mut recv_buf);
        assert!(matches!(sub_result, Ok(None)));

        // Service buffer unaffected
        let svc_result = take_service(slot, &mut recv_buf);
        assert!(matches!(svc_result, Ok(Some(6))));
        assert_eq!(&recv_buf[..6], b"svc_ok");
    }
}
