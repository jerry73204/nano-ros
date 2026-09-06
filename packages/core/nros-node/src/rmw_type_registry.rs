//! Per-type descriptor registration — generic seam (Phase 248 C2).
//!
//! (issue #225 — formerly `cyclonedds_register`: the seam was already
//! backend-agnostic in behavior; the module/cfg names now match. Cyclone DDS
//! is simply the first descriptor-needing tenant, named only in prose.)
//!
//! Some RMW backends (Cyclone DDS) resolve topic-type descriptors via a
//! runtime registry instead of a static-init table. Each `nros-node`
//! typed creator (`create_publisher`, `create_subscription`,
//! `create_client`, `create_service`, `create_action_*`) routes through
//! [`register_type::<M>`] *before* asking the cffi vtable to create the
//! entity so the descriptor exists when the backend's `dds_create_topic`
//! (or equivalent) runs.
//!
//! # No named-backend dependency (issue #60, Tier 1)
//!
//! Previously this module depended directly on `nros-rmw-cyclonedds` and
//! called `nros_rmw_cyclonedds::register::<M>()`, baking a concrete-RMW
//! crate into the platform/RMW-agnostic core executor. It now forwards
//! the message's flattened schema through the generic
//! [`nros_rmw::register_type_descriptor`] seam. The Cyclone backend
//! installs its registrar from its own crate
//! (`nros-rmw-cyclonedds::install_descriptor_registrar`, driven by the
//! `-sys` shim's `RMW_INIT_ENTRIES` self-registration); zenoh / xrce
//! install nothing, so the seam is a no-op there.
//!
//! # cfg gating (auto-detected, not feature-gated)
//!
//! This module's schema-passing body compiles to a no-op unless
//! `cfg(rmw_needs_type_descriptors)` is on. The cfg is emitted by
//! `nros-node/build.rs` from the `needs-type-descriptors` capability
//! marker feature (no dep edge), which the umbrella `nros/rmw-cyclonedds`
//! activates alongside its own `dep:nros-rmw-cyclonedds-sys`. Callers
//! depend on `nros = { features = ["rmw-cyclonedds"] }`; the hook lights
//! up automatically — no user-facing feature flag on `nros-node`. Each
//! typed creator calls [`register_type::<M>`] unconditionally; the body
//! is empty when the cfg is off so zenoh/xrce paths pay nothing. With the
//! cfg on, the caller pays one mutex acquisition + one lookup per creator
//! invocation (idempotent; the backend caches the descriptor on first
//! hit).
//!
//! # Trait bound — [`MessageForRmw`]
//!
//! A descriptor-needing backend needs [`nros_serdes::schema::Message`]
//! for the static field schema, but `nros-node`'s typed creators
//! historically only constrain `M: nros_core::RosMessage`. Adding
//! `Message` as a super-bound on `RosMessage` breaks every existing
//! codegen-emitted msg crate (they impl `RosMessage` but not yet
//! `Message`). Adding it as a per-method bound on every typed creator
//! touches 30+ sites.
//!
//! Compromise: introduce a helper trait [`MessageForRmw`] that is **the
//! bound the typed creators use** in place of bare `M: RosMessage`. It is
//! a blanket impl over `RosMessage` whose extra requirement is `Message`
//! when `cfg(rmw_needs_type_descriptors)` is on, and just `RosMessage` when
//! off.
//!
//! Net effect: a msg crate that impls `RosMessage` works as-is for zenoh
//! + xrce builds; for cyclonedds builds it must additionally impl
//! `Message`. The codegen template (`nros-cli` — separate repo) emits
//! both impls for every generated msg crate.
//!
//! # Error mapping
//!
//! A registrar failure flattens onto [`crate::NodeError::Transport`] with
//! [`nros_rmw::TransportError::PublisherCreationFailed`]. The choice not
//! to add a dedicated `NodeError` variant is deliberate: the C/C++ FFI
//! shim widens to a single `nros_ret_t`, and the failure mode
//! (out-of-capacity registry, descriptor build error, etc.) is a "topic
//! could not be created" from the caller's perspective.

use nros_core::RosMessage;

/// Bound used in place of bare `RosMessage` on typed creators.
///
/// Equivalent to `RosMessage` without `cfg(rmw_needs_type_descriptors)`;
/// equal to `RosMessage + nros_serdes::schema::Message` with it.
///
/// See module-level docs for the rationale.
#[cfg(rmw_needs_type_descriptors)]
pub trait MessageForRmw: RosMessage + nros_serdes::schema::Message {}

#[cfg(rmw_needs_type_descriptors)]
impl<T> MessageForRmw for T where T: RosMessage + nros_serdes::schema::Message {}

// phase-380 W4 — this arm deliberately does NOT require `schema::Message`.
//
// I tightened it to match the descriptor arm so the subscription size assertion
// could be universal, and it broke a documented user-facing pattern: a
// hand-written message type that implements `RosMessage` + `Serialize` +
// `Deserialize` and no schema. `examples/native/rust/custom-msg` exists to
// demonstrate exactly that, and its `SensorReading` stopped compiling. Requiring
// a schema here would make codegen mandatory for anyone subscribing to their own
// type, which is a much larger decision than a build assertion is worth.
#[cfg(not(rmw_needs_type_descriptors))]
pub trait MessageForRmw: RosMessage {}

#[cfg(not(rmw_needs_type_descriptors))]
impl<T> MessageForRmw for T where T: RosMessage {}

// ============================================================================
// register_type::<M>() — the K.7.6.b hook
// ============================================================================

/// Register `M`'s topic-type descriptor with whichever RMW backend
/// installed the generic descriptor seam (`nros_rmw::register_type_descriptor`).
///
/// No-op when `cfg(rmw_needs_type_descriptors)` is off (zenoh / xrce builds
/// never compile the schema-passing body). With the cfg on, flattens
/// `M`'s static schema (`TYPE_NAME` + `FIELDS`) and forwards it to the
/// installed [`nros_rmw::TypeDescriptorRegistrar`] — Cyclone DDS installs
/// one from its own crate (`nros-rmw-cyclonedds::install_descriptor_registrar`),
/// so the core executor no longer needs a named dependency on the Cyclone
/// shim. The first call for a given type builds + caches the descriptor;
/// subsequent calls are O(1) lookups inside the backend.
///
/// Returns `Ok(())` on success (including the "no descriptor-needing
/// backend installed" no-op), or
/// `NodeError::Transport(TransportError::PublisherCreationFailed)` on a
/// backend-side build/registry failure.
#[allow(unused_variables)] // M unused without the cfg
#[inline]
pub fn register_type<M: MessageForRmw>() -> Result<(), crate::NodeError> {
    #[cfg(rmw_needs_type_descriptors)]
    {
        // SAFETY-OF-INPUT: `M: Message` is enforced by the `MessageForRmw`
        // bound under `rmw_needs_type_descriptors`, so `TYPE_NAME` / `FIELDS`
        // are available and the registrar receives the schema it expects.
        nros_rmw::register_type_descriptor(
            <M as nros_serdes::schema::Message>::TYPE_NAME,
            <M as nros_serdes::schema::Message>::FIELDS,
        )
        .map_err(|err| {
            #[cfg(feature = "log")]
            log::error!(
                "nros_rmw::register_type_descriptor::<{}>() failed: {:?}",
                <M as nros_serdes::schema::Message>::TYPE_NAME,
                err
            );
            let _ = err; // silence unused without log
            crate::NodeError::Transport(err)
        })?;
    }
    Ok(())
}

/// Phase 392 W3a — the receive-buffer hint to hand the backend for `M`.
///
/// The hint describes THE MESSAGE, not the buffer the executor happens to have.
/// Until now the registration passed `RX_BUF` — the arena slot size — so a
/// 64-byte type and a 4 KiB type produced the same hint, and the backend's size
/// class was chosen from a number that says nothing about either.
///
/// Returns the type's own bound when there is one, and `default` (the arena
/// size) otherwise. `None` from `max_serialized_bound` means "no bound EXISTS",
/// which phase 380 is explicit must not be turned into a guess — for an
/// unbounded type there is nothing better to say than what we say today.
///
/// Same two arms as [`subscription_buffer_ok`], for the same reason: a backend
/// whose `MessageForRmw` carries no schema cannot answer the question, and
/// requiring one would make codegen mandatory for a hand-written message type.
///
/// This does NOT by itself let a large type through — `create_subscription`
/// still gates on the arena knob, and sizing the arena per type needs a
/// const-generic argument only codegen can supply (phase-392 W3b). What it does
/// is make the number the backend routes on describe the message.
#[cfg(rmw_needs_type_descriptors)]
pub const fn subscription_rx_hint<M: MessageForRmw>(default: usize) -> usize {
    match nros_serdes::size::max_serialized_bound::<M>() {
        Some(bound) => bound,
        None => default,
    }
}

/// See the documented sibling — no schema on this backend, so no better answer
/// than the caller's own default.
#[cfg(not(rmw_needs_type_descriptors))]
pub const fn subscription_rx_hint<M: MessageForRmw>(default: usize) -> usize {
    default
}

/// Phase 403 W2 -- the number of BYTES a buffered subscription's arena slot
/// needs for `M`, given the caller's own ceiling `rx_buf`, or `None` when `M`
/// has NO BOUND.
///
/// Distinct from [`subscription_rx_hint`] on purpose, and the difference is the
/// whole point: the hint describes the MESSAGE and is handed to the backend, so
/// it is the bound alone. This describes OUR STORAGE, so it is clamped by what
/// the caller asked for. A type whose bound EXCEEDS `rx_buf` keeps `rx_buf` --
/// growing it here would silently spend arena the caller budgeted elsewhere and
/// can exhaust it (`report_arena_exhausted`), and the too-small case is already
/// diagnosed by `report_dropped_take` and refused at build time on the
/// declarative path by `subscription_buffer_ok`.
///
/// `max_serialized_bound` takes the LARGER of XCDR1 and XCDR2, which is the
/// rule a RECEIVE buffer needs: this stack WRITES XCDR1, but a peer chooses the
/// encoding, and XCDR2 adds a 4-byte DHEADER and aligns differently. The C
/// constants say the same thing (`{PREFIX}_RX_MAX_SERIALIZED_SIZE` in
/// `packs/c/message.h.jinja`); sizing from XCDR1 alone is a trap.
///
/// # Why `Option` and not a fallback
///
/// Every message type is REQUIRED to carry a bound -- stated in the `.msg`
/// (`string<=64`) or as a `cap` in `nros-codegen.toml` -- so a type without one
/// is an error to be reported, not a case to be absorbed. Returning `rx_buf`
/// here would be exactly the substitution phase 380 forbids: `None` from
/// `max_serialized_bound` means "no bound EXISTS", never "unknown", and a buffer
/// sized from a fallback is the failure that rule was written to prevent.
/// Propagating the `None` makes the fallback impossible to write by accident and
/// leaves the caller to raise a diagnostic naming the type -- the same shape the
/// C header takes, where naming an unbounded type's size constant is a
/// deliberate compile error (`unbounded_token` / `unbounded_reason`).
///
/// Unlike its two neighbours this takes `nros_serdes::schema::Message` DIRECTLY
/// rather than `MessageForRmw`, so it needs no `rmw_needs_type_descriptors`
/// split: the caller that opts in supplies the schema bound itself. That is
/// deliberate -- the bound is a property of the TYPE, not of the backend, and
/// the non-descriptor arm of `MessageForRmw` deliberately does not require a
/// schema (a hand-written message type must keep working, phase-380 W4). Under
/// that arm the two neighbours can only answer "no opinion"; this one is simply
/// not reachable without a schema, which is the honest shape.
pub const fn subscription_rx_bytes<M: nros_serdes::schema::Message>(
    rx_buf: usize,
) -> Option<usize> {
    match nros_serdes::size::max_serialized_bound::<M>() {
        Some(bound) => {
            let framed = transport_framed(bound);
            Some(if framed < rx_buf { framed } else { rx_buf })
        }
        None => None,
    }
}

/// Round a serialized size up to the 4-byte multiple a receive buffer must be
/// able to hold.
///
/// The message is `bound` bytes. What a transport DELIVERS can be up to three
/// bytes more, and a buffer sized to `bound` exactly refuses those bytes rather
/// than truncating them — correctly, and with the message lost.
///
/// Measured, not assumed. A 25-byte `std_msgs/String` published by ROS 2 Humble
/// over stock `rmw_cyclonedds` arrives at the Cyclone backend as
/// `len:28 hdr:00010000 cdr:25` — three bytes of RTPS submessage alignment added
/// by the SENDER, with the encapsulation options reading `0000` rather than
/// `0003`, so the pad is not discoverable from the header either. The backend
/// hands back exactly what the wire gave it (issues 0969 / 0970), which is why
/// the pad reaches buffer sizing now instead of being absorbed by a re-encode.
///
/// Deliberately NOT folded into `max_serialized_bound`. That answers "how big is
/// this message", and issue 0964 exists because that number had been fudged
/// before — it stays exact. Framing is the transport's, so it is applied where a
/// RECEIVE BUFFER is sized and nowhere else.
///
/// Four is not a Cyclone number: RTPS aligns submessages to 4, so it is the
/// alignment any DDS peer can impose. A transport that framed more coarsely
/// would need its own allowance, which is why this is a named function and not
/// a `+ 3`.
///
/// `rosidl_codegen::bounds::transport_framed` is the same rule for the C and C++
/// constants; the two cite each other so they cannot drift apart.
const fn transport_framed(bound: usize) -> usize {
    bound.next_multiple_of(4)
}

/// Phase 380 W4 — can a default-sized receive buffer hold every message of `M`?
///
/// `true` when it can, when `M` is unbounded (nothing to prove), AND on backends
/// whose `MessageForRmw` does not carry a schema — there the question cannot be
/// asked at all, so the honest answer for a build assertion is "no objection".
///
/// The cfg lives here because this is where it is defined; `create_subscription`
/// calls this unconditionally and gets a compile-time `true` where the schema is
/// absent. Keeping the branch here rather than at the call site is what stops a
/// second spelling of "does this backend have schemas" growing in the API crate.
#[cfg(rmw_needs_type_descriptors)]
pub const fn subscription_buffer_ok<M: MessageForRmw>() -> bool {
    nros_serdes::size::bound_fits::<M>(crate::config::DEFAULT_RX_BUF_SIZE)
}

/// See the documented sibling — no schema on this backend, so no check.
#[cfg(not(rmw_needs_type_descriptors))]
pub const fn subscription_buffer_ok<M: MessageForRmw>() -> bool {
    true
}

/// phase-392 W3c — the receive-buffer size a subscription to `M` gets WITHOUT
/// asking, or `None` when there is nothing to derive from.
///
/// This is the inversion the wave is: before it, a subscription that named no
/// number was charged `DEFAULT_RX_BUF_SIZE` per slot and the type's own bound
/// reached the arena only if the consumer wrote
/// [`rx_buffer_from_type`](crate::executor::node::TypedSubscriptionBuilder::rx_buffer_from_type)
/// or `nros::rx_buffer_for!`. A consumer who did not know those exist got the
/// global, silently — the same failure the derivation was written to prevent.
/// Now the derivation is the default and `.rx_buffer::<N>()` is the opt-out.
///
/// # `None` is never a fallback, it is "no answer exists"
///
/// Two DIFFERENT facts both produce `None`, and neither may be turned into a
/// number here (phase 380):
///
/// * **`M` has no bound.** An unbounded member means no bound EXISTS, and
///   inventing one is the substitution that rule forbids. The opt-in spelling
///   refuses to compile in this case ([`subscription_rx_bytes`] returns `None`
///   and `rx_buffer_from_type`'s `const` assert reads it); the DEFAULT path
///   cannot refuse, because that would make a bound mandatory for every message
///   type in every image at once, so it keeps the caller's `RX_BUF` — which is
///   exactly what it was charged before this wave, so no image moves.
/// * **This backend's `MessageForRmw` carries no schema.** Only a backend that
///   declares `type-descriptors` in its `nros-rmw.toml` (today: Cyclone) makes
///   `MessageForRmw` require `nros_serdes::schema::Message`; the other arm
///   deliberately does not, so a hand-written message type keeps working
///   (phase-380 W4, `examples/native/rust/custom-msg`). Under that arm there is
///   no bound to read at a type-erased registration site at ALL, so this returns
///   `None` for every type and the default is unchanged. A consumer on such a
///   backend reaches the derivation by NAMING the schema itself, which is what
///   `.rx_buffer_from_type()` and `rx_buffer_for!` do.
///
/// Same two-arm shape, and the same reason for it, as [`subscription_rx_hint`]
/// and [`subscription_buffer_ok`]: the branch on "does this backend have
/// schemas" is spelled once, here, so no second spelling of it grows at a call
/// site.
#[cfg(rmw_needs_type_descriptors)]
pub const fn default_subscription_rx_bytes<M: MessageForRmw>(rx_buf: usize) -> Option<usize> {
    subscription_rx_bytes::<M>(rx_buf)
}

/// See the documented sibling — no schema on this backend, so no bound is
/// reachable from a `MessageForRmw` bound alone and the default is unchanged.
#[cfg(not(rmw_needs_type_descriptors))]
pub const fn default_subscription_rx_bytes<M: MessageForRmw>(_rx_buf: usize) -> Option<usize> {
    None
}
