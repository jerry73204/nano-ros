---
id: 1179
title: "Derived-by-default receive buffers reach only backends declaring `type-descriptors` — `MessageForRmw` carries no schema on zenoh or XRCE"
status: open
type: enhancement
area: core
related: [phase-392, phase-403, issue-0896]
---

## Problem

phase-392 W3c made the arena receive buffer derive from the message type by
default: `register_subscription_buffered_on` is the one choke point every
default registration funnels through, and a `None` receive size there now means
DERIVE rather than "the global". Measured saving on a 12-byte bounded type at
`QoSProfile::default().depth` (10): the registration's arena claim falls
13,632 -> 2,500 bytes, and at depth 2 it falls 5,376 -> 2,340.

**It is live on one backend of three.** The derivation reads
`nros_serdes::size::max_serialized_bound::<M>()`, which needs
`M: nros_serdes::schema::Message`. At the registration site the only bound in
scope is `M: MessageForRmw`, and that trait has two arms:

```rust
#[cfg(rmw_needs_type_descriptors)]
pub trait MessageForRmw: RosMessage + nros_serdes::schema::Message {}

#[cfg(not(rmw_needs_type_descriptors))]
pub trait MessageForRmw: RosMessage {}
```

`rmw_needs_type_descriptors` is emitted from the `needs-type-descriptors`
capability feature, which a backend requests by declaring `type-descriptors` in
its `nros-rmw.toml`. Measured across the four descriptors in the tree
(`packages/rmw/{zenoh,xrce,uorb,cyclonedds}/*/nros-rmw.toml`): **only Cyclone
declares it.** So on a zenoh or XRCE image there is no bound to read at a
type-erased registration site at all, and
`rmw_type_registry::default_subscription_rx_bytes::<M>` returns `None` for
every type — the default is unchanged, correctly and by construction.

This is the same reason phase-392 W3a's `subscription_rx_hint::<M>` is inert
there. Worth stating plainly, because the phase doc's W3a entry reads as
tree-wide: *"A Rust subscription to a 4 KiB type now routes large"* is true on
Cyclone and false on zenoh — and zenoh is the backend that HAS the small/large
payload classes the sentence is about.

The consumer-visible workaround is not a workaround so much as the honest
shape: `.rx_buffer_from_type()` and `nros::rx_buffer_for!(M)` NAME
`nros_serdes::schema::Message` themselves, so they derive on every backend.
They are opt-in, which is exactly what W3c set out to stop being the only
route.

## Why the obvious fix was refused, and what is left

Tightening `MessageForRmw` to require `schema::Message` on both arms is the
one-line version, and phase-380 W4 already refused it, in a comment on the arm
itself: it broke `examples/native/rust/custom-msg`, whose `SensorReading` is
hand-written (`RosMessage` + `Serialize` + `Deserialize`, no schema) precisely
to demonstrate that codegen is not mandatory for a user's own message type.
Requiring a schema there makes codegen mandatory for anyone subscribing to a
type they wrote, which is a much larger decision than a buffer default.

Two candidate designs, neither obviously right:

* **(a) Carry the bound on `RosMessage`.** Add
  `const MAX_SERIALIZED_SIZE: Option<usize> = None;` to `RosMessage` — which
  `MessageForRmw` requires under *both* arms — and have `emit_rust.rs` emit the
  override in every generated `impl RosMessage`, computed as
  `nros_serdes::size::max_serialized_bound::<Self>()`. That is the SAME
  computation, not a mirror of it, so it does not reopen the sizes-header drift
  class (0088 -> 0114 -> 0122 -> 0123 -> 0245 -> 0268) that issue 0896 warns
  about. Universal across backends; a hand-written type keeps the `None`
  default and behaves exactly as today. Cost: it touches the message ABI trait
  and needs the 88 committed files under `packages/interfaces/*/generated/`
  regenerated.
* **(b) Split the typed entry point.** `.typed::<M>()` requires
  `M: schema::Message` and derives; a second spelling takes schema-less types
  at the global. Cheap, and it forces a one-line source edit on every consumer
  with a hand-written type — loud rather than silent, but a breaking change to
  a documented pattern.

(a) preserves every consumer and costs regeneration; (b) preserves the build
and costs a documented API. **Choose deliberately, not by whichever is
implemented first** — the same instruction phase-392's own W3 remainder gives
for the C/C++ fork one lane over.

## Reproduce

```
cargo test -p nros-node --features std,alloc                          --lib -- derived_from_the_type
cargo test -p nros-node --features std,alloc,needs-type-descriptors   --lib -- derived_from_the_type
```

`the_default_subscription_buffer_is_derived_from_the_type` asserts both arms —
that naming nothing costs what `.rx_buffer_from_type()` costs where a schema is
present, and what `.rx_buffer::<GLOBAL>()` costs where it is not — so the test
is green either way and says which arm it ran in when it is not.

## Coverage gap this leaves

`test-unit` runs `cargo nextest --workspace` with no features, so the
descriptor arm — the arm on which W3c does anything — is exercised by no lane.
The assertion above is real on both arms, but only the second invocation above
can catch a regression in the derivation itself. Verified by reverting the
choke-point change and re-running: the descriptor arm fails with
`left: 5376, right: 2340`, the default arm stays green.
