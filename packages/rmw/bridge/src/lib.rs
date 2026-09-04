//! Cross-RMW bridge primitives for nano-ros (Phase 128.F).
//!
//! Lets a binary that intentionally links more than one RMW backend
//! forward raw serialized payloads between Nodes bound to different
//! backends. Common single-backend code does not need this crate.
//!
//! # Serialization format (RFC-0088 D3, phase-421 W3)
//!
//! `Executor::open_multi` is the one place where RFC-0088's compile-time
//! answer stops existing — one image, two backends, potentially two formats —
//! so it is the one place the format becomes a runtime value.
//! [`PubSubBridge::new`] compares the two sides' formats once, at
//! construction, and returns [`BridgeError::FormatMismatch`] naming both when
//! they differ. [`PubSubBridge::with_converter`] is the deliberate
//! cross-format case; it takes a [`SerializationFormatConverter`] and checks
//! its declared endpoints, also once. Nothing about a format is evaluated per
//! sample.
//!
//! # Pattern
//!
//! 1. Open the executor with [`Executor::open_multi`].
//! 2. Build per-backend Nodes via [`Executor::create_node_on`].
//! 3. Create a raw subscription on the source Node and a raw publisher
//!    on the destination Node (existing
//!    [`create_subscription_raw`] / [`create_publisher_raw`] APIs).
//! 4. Hand them to [`PubSubBridge::new`] and call
//!    [`PubSubBridge::pump`] inside the executor's spin loop.
//!
//! ```ignore
//! use nros::prelude::*;
//! use nros_node::executor::SessionSpec;
//! use nros_bridge::PubSubBridge;
//!
//! fn main() -> Result<(), NodeError> {
//!     let mut exec = Executor::open_multi(&[
//!         SessionSpec::new("zenoh", "tcp/10.0.0.1:7447"),
//!         SessionSpec::new("dds",   "domain=0"),
//!     ])?;
//!     let mut field   = exec.create_node_on("field",   "zenoh")?;
//!     let mut control = exec.create_node_on("control", "dds")?;
//!     // user wires sub on `field`, pub on `control`, then:
//!     // let mut bridge = PubSubBridge::new(sub, pubr, "zenoh")?;
//!     // bridge.pump()?;
//!     Ok(())
//! }
//! ```
//!
//! # Loop protection (phase 128.F.4)
//!
//! Wire-level loop protection would prefer the ROS 2 attachment
//! mechanism (`bridge_origin` field), but the per-backend
//! `publish_raw_with_attachment` ABI is not yet on the public Rust
//! surface — that lands in phase 129 alongside the platform / link
//! feature cleanup. Until then, [`PubSubBridge`] uses a
//! best-effort payload-hash dedup window: each forwarded sample's
//! FNV-1a-64 hash goes into a small ring; samples whose hash matches
//! one already in the ring within the last [`DEDUP_WINDOW`] entries
//! are skipped on the way out. Handles the common bidirectional
//! echo pattern (forward → backend B → forward back → drop) without
//! requiring backend changes. Distinct messages that happen to
//! collide on hash within the window are silently dropped — collision
//! probability is ~`N/2^64`, negligible in practice for the small
//! windows in use.
//!
//! Set the origin string to `""` to disable dedup (single-direction
//! bridges don't need it).

#![cfg_attr(not(feature = "std"), no_std)]

use nros_node::{
    NodeError,
    executor::{EmbeddedRawPublisher, RawSubscription},
};

mod convert;
pub use convert::{BridgeError, ConvertError, SerializationFormatConverter};

#[cfg(feature = "config")]
mod config;
#[cfg(feature = "config")]
pub use config::{ConfigError, run_from_config, run_from_config_str};

#[cfg(feature = "cffi")]
mod cffi;

/// Issue 0436 — force-link anchor for the C/C++ bridge ABI.
///
/// When this crate is bundled as an rlib DEPENDENCY of a staticlib root (that is
/// how `libnros_cpp.a` ships it), rustc drops every `#[no_mangle]` export the root
/// never references — so `<nros/bridge.hpp>`'s `MultiExecutor` compiled but failed
/// to link, and a PX4 module could not reach the bridge API at all. The root
/// references THIS from a `#[used]` static, which keeps the objects (and with them
/// the whole `nros_init_multi` / `nros_pubsub_bridge_*` surface) in the archive.
///
/// Mirrors `nros_c::c_surface_anchor::C_SURFACE_ANCHOR`.
#[cfg(feature = "cffi")]
pub mod cffi_surface_anchor {
    /// Fn pointers into the C ABI. Never called through — an anchor only.
    pub static CFFI_SURFACE_ANCHOR: [unsafe extern "C" fn(*mut core::ffi::c_void); 1] =
        [super::cffi::nros_fini_multi];
}

/// Size of the per-bridge payload-hash dedup ring. Forwarded sample
/// hashes are written here in a circular buffer; receive-side
/// matches against any slot within the window cause the sample to be
/// dropped. 16 entries cover a one-direction-of-flight burst comfortably
/// while keeping the bridge's per-instance footprint at 128 bytes.
pub const DEDUP_WINDOW: usize = 16;

/// Bridge a raw subscription on one Node to a raw publisher on
/// another. Each [`pump`](Self::pump) call drains every queued sample
/// from the subscription and forwards the bytes to the publisher.
///
/// # Serialization format (RFC-0088 D3, phase-421 W3)
///
/// Backend bytes pass through untouched, so the two sides must agree on what
/// those bytes mean. That used to be a sentence in this doc comment ("both
/// sides must use ROS-CDR … Cross-encoding bridges would need an explicit
/// translator and are out of scope") checked by nothing; it is now a return
/// value. [`new`](Self::new) compares
/// [`RawSubscription::format`] against [`EmbeddedRawPublisher::format`] and
/// refuses a mismatch with [`BridgeError::FormatMismatch`], which names both
/// formats.
///
/// The comparison is **one byte, once, at construction**. [`pump`](Self::pump)
/// never looks at a format: a bridge that exists has already been proven
/// consistent, so forwarding stays a memcpy.
///
/// For the deliberate cross-format case, [`with_converter`](Self::with_converter)
/// takes a [`SerializationFormatConverter`] and validates its declared
/// endpoints against the two sides, also once, at construction.
///
/// # Const generics
///
/// `RX_BUF` / `TX_BUF` size the receive and transmit staging buffers.
/// `CONV_BUF` sizes the conversion output buffer and defaults to `0` — a
/// zero-length array is a ZST, so a bridge that does not convert costs exactly
/// what it did before converters existed. A converting bridge names its own
/// size: `PubSubBridge::<RX, TX, 1024>::with_converter(…)`.
pub struct PubSubBridge<
    const RX_BUF: usize = { nros_node::config::DEFAULT_RX_BUF_SIZE },
    const TX_BUF: usize = { nros_node::executor::DEFAULT_LOAN_BUF },
    const CONV_BUF: usize = 0,
> {
    sub: RawSubscription<RX_BUF>,
    pubr: EmbeddedRawPublisher<TX_BUF>,
    /// Phase 128.F.4 — name of the backend the source Node is bound
    /// to. Empty string disables dedup (single-direction bridges).
    origin: &'static str,
    /// Phase 128.F.4 — payload-hash dedup ring. Each forwarded
    /// sample's FNV-1a-64 hash is written here in a circular buffer.
    /// On receive, the bridge checks whether the incoming sample's
    /// hash is in the ring and skips when matched. Pair this bridge
    /// with the return-direction bridge using a paired
    /// [`LoopGuard`] (or hand the same allocation in by sharing a
    /// mutable reference if the bridges live in the same scope).
    dedup: LoopGuard,
    /// RFC-0088 D3 — `Some` only for a bridge built by
    /// [`with_converter`](Self::with_converter). `None` is the whole
    /// single-format world, and the field is one nullable fat pointer, checked
    /// once per drained sample at the publish step rather than compared as a
    /// format.
    conv: Option<&'static dyn SerializationFormatConverter>,
    /// RFC-0088 D3 — staging for [`SerializationFormatConverter::convert`]
    /// output. `[u8; 0]` (a ZST) unless the caller sized `CONV_BUF`.
    conv_buf: [u8; CONV_BUF],
}

/// Standalone payload-hash dedup ring. Used internally by
/// [`PubSubBridge`] and exposed so two bridges in a bidirectional
/// pair can optionally share state.
#[derive(Default)]
pub struct LoopGuard {
    ring: [u64; DEDUP_WINDOW],
    head: usize,
}

impl LoopGuard {
    /// Construct an empty guard. Both rings start as zeros; the
    /// "all-zero hash" collision is harmless because every realistic
    /// payload's FNV-1a hash is non-zero (FNV-1a starts at offset
    /// basis `0xcbf29ce484222325`).
    pub const fn new() -> Self {
        Self {
            ring: [0u64; DEDUP_WINDOW],
            head: 0,
        }
    }

    /// Insert `hash` into the ring at the current head, advancing.
    pub fn record(&mut self, hash: u64) {
        self.ring[self.head] = hash;
        self.head = (self.head + 1) % DEDUP_WINDOW;
    }

    /// Return true when `hash` appears anywhere in the current
    /// window. O(DEDUP_WINDOW) — 16 cmp instructions in the default
    /// configuration.
    pub fn contains(&self, hash: u64) -> bool {
        self.ring.contains(&hash)
    }
}

/// Phase 128.F.4 — wire-level `bridge_origin` attachment tag.
///
/// Layout written by [`encode_bridge_origin`]:
///
/// ```text
///   offset 0 ..  8  : ASCII magic b"NROSBRDG"
///   offset 8        : version byte (current = 1)
///   offset 9        : origin length (u8, 1..=54)
///   offset 10 .. n  : origin bytes (utf-8)
/// ```
///
/// Total header = 10 bytes; max tag size = 64 bytes (10 + 54 origin
/// bytes). Backends that don't carry attachments see no impact;
/// backends that do carry attachments propagate the tag to the
/// receive side, where [`parse_bridge_origin`] recovers the origin
/// string for the dedup compare. Unknown / malformed bytes return
/// `None` and let the fallback FNV hash dedup handle the case.
pub const BRIDGE_ORIGIN_MAGIC: &[u8; 8] = b"NROSBRDG";
const BRIDGE_ORIGIN_VERSION: u8 = 1;
const BRIDGE_ORIGIN_HEADER_LEN: usize = 10;

/// Encode `origin` into `out` and return the number of bytes
/// written. Caller must provide a buffer of at least
/// `BRIDGE_ORIGIN_HEADER_LEN + origin.len()` bytes. Origin longer
/// than 54 bytes is silently truncated (the receiver still recovers
/// a valid prefix; collision risk is the caller's problem).
pub fn encode_bridge_origin(origin: &[u8], out: &mut [u8]) -> usize {
    if origin.is_empty() {
        return 0;
    }
    let max = (out.len().saturating_sub(BRIDGE_ORIGIN_HEADER_LEN)).min(0xFF);
    let copy = origin.len().min(max);
    if out.len() < BRIDGE_ORIGIN_HEADER_LEN + copy {
        return 0;
    }
    out[..8].copy_from_slice(BRIDGE_ORIGIN_MAGIC);
    out[8] = BRIDGE_ORIGIN_VERSION;
    out[9] = copy as u8;
    out[BRIDGE_ORIGIN_HEADER_LEN..BRIDGE_ORIGIN_HEADER_LEN + copy].copy_from_slice(&origin[..copy]);
    BRIDGE_ORIGIN_HEADER_LEN + copy
}

/// Parse a `bridge_origin` attachment block. Returns the origin
/// bytes when the input matches our magic + version; otherwise
/// `None` (caller treats as "no bridge origin present" — the
/// attachment may belong to ROS safety / seq-num / other consumers).
pub fn parse_bridge_origin(att: &[u8]) -> Option<&[u8]> {
    if att.len() < BRIDGE_ORIGIN_HEADER_LEN {
        return None;
    }
    if &att[..8] != BRIDGE_ORIGIN_MAGIC {
        return None;
    }
    if att[8] != BRIDGE_ORIGIN_VERSION {
        return None;
    }
    let n = att[9] as usize;
    if att.len() < BRIDGE_ORIGIN_HEADER_LEN + n {
        return None;
    }
    Some(&att[BRIDGE_ORIGIN_HEADER_LEN..BRIDGE_ORIGIN_HEADER_LEN + n])
}

/// FNV-1a 64-bit. Public so `[LoopGuard]` users that need to share
/// hash values between bridges agree on the function.
pub fn payload_hash(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

/// Same-format bridges — the overwhelming majority — are `CONV_BUF = 0`, and
/// [`new`](PubSubBridge::new) lives here rather than in the general impl so
/// that inference can pick that `0` on its own. A const-generic DEFAULT is not
/// an inference fallback: with `new` in the general impl every existing
/// `PubSubBridge::new(sub, pubr, origin)` becomes "type annotations needed".
impl<const RX_BUF: usize, const TX_BUF: usize> PubSubBridge<RX_BUF, TX_BUF, 0> {
    /// Build a one-direction bridge. `origin` is the RMW name of the
    /// session the source subscription is bound to (e.g. `"zenoh"`).
    /// Pass `""` when the bridge is single-direction and loop
    /// protection is not needed.
    ///
    /// # Errors
    ///
    /// [`BridgeError::FormatMismatch`] when the subscription's
    /// [`format`](RawSubscription::format) and the publisher's
    /// [`format`](EmbeddedRawPublisher::format) differ. The bridge forwards
    /// bytes untouched, so a mismatch would put one format's bytes on a wire
    /// the other side reads as the other's — RFC-0088 D3 makes that a
    /// construction failure rather than a doc comment. Cross-format on purpose
    /// goes through [`with_converter`](Self::with_converter).
    ///
    /// The comparison is one byte, evaluated exactly here. `pump` gains
    /// nothing.
    pub fn new(
        sub: RawSubscription<RX_BUF>,
        pubr: EmbeddedRawPublisher<TX_BUF>,
        origin: &'static str,
    ) -> Result<Self, BridgeError> {
        let ingress = sub.format();
        let egress = pubr.format();
        if ingress != egress {
            return Err(BridgeError::FormatMismatch { ingress, egress });
        }
        Ok(Self {
            sub,
            pubr,
            origin,
            dedup: LoopGuard::new(),
            conv: None,
            conv_buf: [],
        })
    }
}

impl<const RX_BUF: usize, const TX_BUF: usize, const CONV_BUF: usize>
    PubSubBridge<RX_BUF, TX_BUF, CONV_BUF>
{
    /// Build a bridge that translates between two serialization formats
    /// (RFC-0088 D3).
    ///
    /// This is the deliberate cross-format case: the caller names a
    /// [`SerializationFormatConverter`], and the bridge checks — once, here —
    /// that the converter's declared endpoints are the bridge's own. A
    /// converter wired backwards is then a `Result` at wiring time rather than
    /// nonsense on the wire.
    ///
    /// Size the conversion output buffer on the type:
    /// `PubSubBridge::<RX, TX, 1024>::with_converter(sub, pubr, origin, conv)`.
    ///
    /// # Errors
    ///
    /// * [`BridgeError::NoConversionBuffer`] when `CONV_BUF == 0` — the
    ///   converter would have nowhere to write.
    /// * [`BridgeError::ConverterMismatch`] when the converter's `from`/`to`
    ///   are not the subscription's and the publisher's formats. The message
    ///   names all four.
    pub fn with_converter(
        sub: RawSubscription<RX_BUF>,
        pubr: EmbeddedRawPublisher<TX_BUF>,
        origin: &'static str,
        conv: &'static dyn SerializationFormatConverter,
    ) -> Result<Self, BridgeError> {
        if CONV_BUF == 0 {
            return Err(BridgeError::NoConversionBuffer);
        }
        let ingress = sub.format();
        let egress = pubr.format();
        let (conv_from, conv_to) = (conv.from(), conv.to());
        if conv_from != ingress || conv_to != egress {
            return Err(BridgeError::ConverterMismatch {
                conv_from,
                conv_to,
                ingress,
                egress,
            });
        }
        Ok(Self {
            sub,
            pubr,
            origin,
            dedup: LoopGuard::new(),
            conv: Some(conv),
            conv_buf: [0u8; CONV_BUF],
        })
    }

    /// The serialization format of the bytes entering this bridge.
    pub fn ingress_format(&self) -> nros_serdes::format::SerializationFormatId {
        self.sub.format()
    }

    /// The serialization format of the bytes leaving this bridge. Equal to
    /// [`ingress_format`](Self::ingress_format) unless the bridge was built
    /// with a converter.
    pub fn egress_format(&self) -> nros_serdes::format::SerializationFormatId {
        self.pubr.format()
    }

    /// Drain every queued sample and forward to the destination
    /// publisher. Returns the number of samples *actually* forwarded
    /// on this call. Samples that hashed into the dedup window are
    /// counted as dropped — see [`pump_with_stats`](Self::pump_with_stats)
    /// for the breakdown.
    pub fn pump(&mut self) -> Result<usize, NodeError> {
        Ok(self.pump_with_stats()?.forwarded)
    }

    /// Per-pump statistics — useful for diagnostics and for tests
    /// that need to assert dedup actually fired.
    pub fn pump_with_stats(&mut self) -> Result<PumpStats, NodeError> {
        let mut stats = PumpStats::default();
        let origin = self.origin;
        // RFC-0088 D3 — hoisted out of the drain loop and copied off `self`, so
        // the loop body borrows only the fields it writes. `None` is the
        // single-format world and stays one null test at the publish step; no
        // FORMAT is compared here or anywhere else in `pump` — that happened
        // once, in `new` / `with_converter`.
        let conv = self.conv;
        let origin_bytes = origin.as_bytes();
        // Phase 128.F.4 — wire-level attachment scratch buffers for
        // `bridge_origin` reads / writes. 64 bytes covers any
        // backend name we ship (`zenoh`/`dds`/`xrce`/`cyclonedds`)
        // with room for the tag header.
        let mut att_in = [0u8; 64];
        loop {
            let recv = self.sub.take_serialized_with_attachment(&mut att_in)?;
            let (payload_len, att_len) = match recv {
                Some(t) => t,
                None => break,
            };
            // Wire-level filter — when the backend natively carries
            // attachments AND a paired bridge stamped
            // `bridge_origin=<our origin>`, drop here. Empty origin
            // disables (single-direction bridges).
            if !origin.is_empty()
                && att_len > 0
                && parse_bridge_origin(&att_in[..att_len]) == Some(origin_bytes)
            {
                stats.dropped_echo += 1;
                continue;
            }
            // FNV hash fallback — catches echoes on backends that
            // don't yet carry attachments (xrce, dds default). Same
            // window-record discipline as before; harmless on
            // backends with native attachment because the wire-level
            // check above already fires first.
            let bytes = &self.sub.buffer()[..payload_len];
            let hash = payload_hash(bytes);
            if !origin.is_empty() && self.dedup.contains(hash) {
                stats.dropped_echo += 1;
                continue;
            }
            // RFC-0088 D3 — translate only when the caller asked for it. The
            // `None` arm is the original memcpy path, byte for byte.
            let out: &[u8] = match conv {
                None => bytes,
                Some(c) => match c.convert(bytes, &mut self.conv_buf) {
                    Ok(n) => {
                        let n = n.min(CONV_BUF);
                        &self.conv_buf[..n]
                    }
                    Err(e) => {
                        // Dropping is the only local recovery — the sample
                        // cannot be represented on the far side. Counted AND
                        // logged, never silent: a converter that fails every
                        // sample must not read as a quiet bridge.
                        nros_log::log_error!(
                            nros_log::get_logger("nros_bridge"),
                            "bridge conversion failed, sample dropped: {}",
                            e
                        );
                        stats.dropped_convert += 1;
                        continue;
                    }
                },
            };
            // Stamp our origin on the way out so the receiving
            // bridge can wire-level-filter it.
            let mut att_out = [0u8; 64];
            let att_out_len = encode_bridge_origin(origin_bytes, &mut att_out);
            self.pubr
                .publish_raw_with_attachment(out, &att_out[..att_out_len])?;
            if !origin.is_empty() {
                self.dedup.record(hash);
            }
            stats.forwarded += 1;
        }
        Ok(stats)
    }

    /// RMW backend name the source session is bound to.
    pub fn origin(&self) -> &'static str {
        self.origin
    }

    /// Borrow the dedup ring — bidirectional setups can call
    /// [`LoopGuard::record`] on the *other* bridge's guard when this
    /// one publishes, so the return-direction bridge sees its own
    /// echoes too.
    pub fn guard_mut(&mut self) -> &mut LoopGuard {
        &mut self.dedup
    }

    /// Decompose the bridge back into its source subscription and
    /// destination publisher.
    pub fn into_parts(self) -> (RawSubscription<RX_BUF>, EmbeddedRawPublisher<TX_BUF>) {
        (self.sub, self.pubr)
    }
}

/// Per-pump counters returned by [`PubSubBridge::pump_with_stats`].
#[derive(Debug, Default, Clone, Copy)]
pub struct PumpStats {
    /// Samples that crossed to the destination publisher.
    pub forwarded: usize,
    /// Samples that matched a recently-forwarded hash and were
    /// dropped to break a bidirectional echo loop.
    pub dropped_echo: usize,
    /// RFC-0088 D3 — samples a [`SerializationFormatConverter`] refused, and
    /// which therefore never reached the destination. Always `0` on a bridge
    /// built with [`PubSubBridge::new`], which has no converter. Not mirrored
    /// into the C `nros_pump_stats_t` — that struct's layout is a published
    /// ABI (`<nros/bridge.h>`) and appending to it is exactly the drift class
    /// `check-ffi-struct-mirrors` exists for.
    pub dropped_convert: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fnv1a_basis_non_zero() {
        // The empty payload's FNV-1a is the offset basis; non-zero so
        // the all-zeros initial ring doesn't accidentally suppress it.
        assert_ne!(payload_hash(&[]), 0);
    }

    #[test]
    fn bridge_origin_roundtrip() {
        let mut buf = [0u8; 64];
        let n = encode_bridge_origin(b"zenoh", &mut buf);
        assert_eq!(n, BRIDGE_ORIGIN_HEADER_LEN + 5);
        let parsed = parse_bridge_origin(&buf[..n]).expect("parse");
        assert_eq!(parsed, b"zenoh");
    }

    #[test]
    fn bridge_origin_rejects_garbage() {
        assert!(parse_bridge_origin(&[]).is_none());
        assert!(parse_bridge_origin(b"not a tag").is_none());
        // Right magic, wrong version.
        let mut buf = [0u8; 32];
        buf[..8].copy_from_slice(BRIDGE_ORIGIN_MAGIC);
        buf[8] = 99;
        buf[9] = 4;
        buf[10..14].copy_from_slice(b"junk");
        assert!(parse_bridge_origin(&buf[..14]).is_none());
    }

    #[test]
    fn bridge_origin_empty_skips_encode() {
        let mut buf = [0u8; 32];
        assert_eq!(encode_bridge_origin(b"", &mut buf), 0);
    }

    /// RFC-0088 D3 — the converter must be free for the bridges that do not
    /// use one. `CONV_BUF` defaults to `0`, whose array is a ZST, so a
    /// non-converting bridge pays only for the nullable converter pointer.
    #[test]
    fn zero_conv_buf_is_a_zst() {
        assert_eq!(core::mem::size_of::<[u8; 0]>(), 0);
        let plain = core::mem::size_of::<PubSubBridge<64, 64, 0>>();
        let converting = core::mem::size_of::<PubSubBridge<64, 64, 1024>>();
        assert!(
            converting >= plain + 1024,
            "a 1024-byte conversion buffer must actually be there: \
             plain={plain} converting={converting}"
        );
        // And the default spelling is the free one.
        assert_eq!(core::mem::size_of::<PubSubBridge<64, 64>>(), plain);
    }

    #[test]
    fn loop_guard_window() {
        let mut g = LoopGuard::new();
        for i in 0..(DEDUP_WINDOW as u64) {
            g.record(i + 1);
        }
        // All inserted values are still in the ring.
        for i in 0..(DEDUP_WINDOW as u64) {
            assert!(g.contains(i + 1));
        }
        // One more insert evicts the oldest.
        g.record(999);
        assert!(g.contains(999));
        assert!(!g.contains(1));
    }
}

// ---------------------------------------------------------------------------
// phase-361 W8.e / issue 0594 — capabilities REQUIRE the heap / the standard
// library, they do not enable it. Turning `alloc` or `std` on for the user
// silently changes what their firmware image is; naming the feature they must
// add does not.
// ---------------------------------------------------------------------------
#[cfg(all(feature = "config", not(feature = "std")))]
compile_error!("`config` parses TOML via serde: add \"std\" to this crate's features");
#[cfg(all(feature = "cffi", not(feature = "alloc")))]
compile_error!("`cffi` boxes every entity handle: add \"alloc\" to this crate's features");
