---
rfc: 0022
title: "Entity API tiers — convenient (`fork`) + customizable (`clone`)"
status: Stable
since: 2026-05
last-reviewed: 2026-05
implements-tracked-by: []
supersedes: []
superseded-by: null
---

# Entity API tiers — convenient (`fork`) + customizable (`clone`)

**Problem.** The executor grew a combinatorial zoo of entity constructors —
`register_subscription`, `register_subscription_raw`,
`register_subscription_buffered_raw_on`,
`register_subscription_raw_with_qos_sized_on`, the proposed
`…_with_info_on`, plus the `create_publisher` / `create_publisher_raw`
pair, `create_subscription_raw_sized`, etc. Every new axis (raw vs typed, QoS,
`rx_buf` size, `_on`-a-node, MessageInfo, session binding) multiplies the
surface. New needs (the bridge wants raw + MessageInfo + session) tempt yet
another `_with_x_y_z` function.

**Goal.** Two tiers, like Linux `fork` vs `clone`:

- **`fork` — convenient, matches upstream ROS.** The 90% case reads exactly like
  rclcpp / rclrs. No knobs to learn.
- **`clone` — one fully-customizable primitive** that *every* convenient
  constructor delegates to, exposing every axis as an optional knob with a
  default. Adding an axis = a builder method, never a new top-level function.

## Naming policy

**No lengthy `verb_noun_axis_axis_axis` names** (`register_subscription_raw_with_qos_sized_on`
and friends). Names stay short; *axes are builder methods, not name suffixes*:

- **Convenient ctors:** `create_publisher` / `create_subscription`
  (+ `create_generic_publisher` / `create_generic_subscription`). One word per
  concept, matching upstream. That's the whole convenient vocabulary.
- **Builder entry:** `node.publisher(topic)` / `node.subscription(topic)`.
- **Knobs:** short single-word methods — `.generic()`, `.typed()`, `.qos()`,
  `.rx_buffer::<N>()`, `.session()`, `.message_info()`, `.sched_context()`,
  `.build()`. A new axis adds *one short method*, never a new function nor a
  longer name.

The long `register_*_*_*` names are **removed** (a brief deprecation shim, then
gone — §Migration), not carried forever. Generated code and applications use
only the short convenient ctors + the builder.

This is also how upstream is shaped: rclcpp has `create_subscription<M>(topic,
qos, cb, options)` + `create_generic_subscription(topic, type, qos, cb,
options)` over one `SubscriptionOptions`; rclrs has `create_subscription` +
fluent QoS-on-topic (`IntoPrimitiveOptions`) + `SubscriptionOptions`. We mirror
that, plus nano-ros-only knobs (raw `rx_buf` size, session binding) on the same
options.

## Tier 1 — convenient (`fork`), mirrors rclcpp / rclrs

```rust
// typed (the common case) — identical shape to rclrs/rclcpp:
let publisher = node.create_publisher::<Int32>("/chatter")?;
let _sub      = node.create_subscription::<Int32>("/chatter", |m: &Int32| { … })?;

// QoS via the fluent topic option (rclrs IntoPrimitiveOptions):
let _sub = node.create_subscription::<Int32>("/chatter".keep_last(10).reliable(), cb)?;

// generic / type-erased (rclcpp create_generic_*), for relays & tools:
let gp = node.create_generic_publisher("/chatter", "std_msgs/msg/Int32", hash)?;
let gs = node.create_generic_subscription("/chatter", ty, hash,
            |bytes: &[u8], info: &MessageInfo| { … })?;
```

These are thin wrappers — each is the builder below with defaults.

## Tier 2 — the `clone`: one builder, every knob

```rust
let sub = node.subscription("/chatter")          // SubscriptionBuilder
    .generic(type_name, type_hash)               // XOR .typed::<Int32>()
    .qos(QoSProfile::default().keep_last(10))
    .rx_buffer::<2048>()                          // const-generic staging size
    .message_info()                              // callback gets (&[u8], &MessageInfo)
    .session(slot)                               // bind to an open_multi session (172.K.5)
    .sched_context(sc)                           // scheduling tier
    .build(callback)?;

let publisher = node.publisher("/chatter")
    .typed::<Int32>()
    .qos(q)
    .session(slot)
    .build()?;
```

## Borrow model — node-centric without `Arc`

rclrs's `node.create_subscription(…)` works because its `Node` is `Arc`-shared.
nano-ros is **`&mut Executor` + no_std (no `Arc`)** — yet the node-centric shape
still works, under one rule: **a node handle is a *short-lived* `&mut Executor`
borrow — create entities on it, then drop it before acquiring the next.** Entity
handles (`Publisher`, the registered subscription/`HandleId`) are **owned** and
outlive the node handle, so two node handles are never needed at once.

`exec.node(id)` (or `create_node`'s return) yields a `NodeCtx<'_>` borrowing
`&mut Executor`; the `publisher` / `subscription` builders register through it.
Holding two `NodeCtx` simultaneously is a borrow error *by construction* — which
is exactly right, and the bridge fits it: build the dest publisher on one
node-ctx (dropped; the handle is owned), then register the source subscription
on another:

```rust
let dpub = exec.node(nb).publisher(topic)          // NodeCtx dropped after build
              .generic(ty, hash).build()?;         // dpub: owned handle, lives on
exec.node(na).subscription(topic)                  // re-borrow exec
    .generic(ty, hash).qos(q).message_info()
    .build(move |bytes, info| {                    // closure owns dpub
        if parse_bridge_origin(info.attachment()) == Some(ORIGIN) { return; }
        let _ = dpub.publish_raw_with_attachment(bytes, &ORIGIN_ATT);
    })?;                                            // NodeCtx(na) registers, then drops
```

This is the deliberate embedded trade vs rclrs's `Arc`: zero allocation / no
shared-ownership runtime cost, at the price of "one node-ctx live at a time" —
re-acquire with `exec.node(id)` whenever you need the node again. (The existing
session-borrowing `Node<'a>` from `create_node_on` is the lower-level form;
`NodeCtx` adds the `&mut Executor` reach the callback-registering subscription
builder needs.)

## Mapping the existing zoo onto the builder

Nothing is lost — everything collapses:

| today's flat fn | builder form |
|---|---|
| `register_subscription::<M>(t, cb)` | `subscription(t).typed::<M>().build(cb)` |
| `…_raw(t, ty, hash, cb)` | `.generic(ty, hash)` |
| `…_raw_sized::<N>` | `.rx_buffer::<N>()` |
| `…_with_qos…` | `.qos(q)` |
| `…_on(node, …)` | `node.subscription(...)` (node-scoped) + `.session(slot)` |
| `…_with_info_on` (proposed) | `.message_info()` |
| `create_publisher` / `…_raw` | `publisher(t).typed::<M>()` / `.generic(ty, hash)` |

## Why a builder (not a big options struct)

A `SubscriptionOptions { … }` struct works in C++/Python (named fields,
defaults). In Rust the **builder** is more ergonomic here because some knobs are
**const-generic** (`rx_buffer::<N>()` sizes the staging array at the type
level — can't be a runtime struct field) and the typed-vs-generic choice changes
the callback's argument type. A builder threads the const param + the
callback-type through `build`. `into`-style fluent topic QoS
(`"t".keep_last(10)`) stays available for the convenient tier.

The C / C++ surfaces (rclc / rclcpp mirrors) keep their named-options structs;
the builder is the Rust ergonomic front, all three lowering to the same core.

### C / C++ options shape (Phase 189.M3)

Both bindings already pass `QoS` (`nros::QoS` / `nros_qos_t`) as a separate
argument — rclcpp's convention — so the `Options` struct sits *alongside* qos
and carries the **non-QoS** axes, not qos itself. Axis realities (M3 survey):

- **`sched_context`** — a plain runtime field; lowers to create-then-bind via
  the existing `nros_{,cpp_}executor_bind_handle_to_sched_context`. Cheap.
- **`message_info`** — a bool selecting a with-`MessageInfo` subscription. No
  C/C++ with-info path exists yet; it needs a `SubBufferedRawInfoCEntry`
  C-fn-ptr arena path (the C analog of the Rust `SubBufferedRawInfoEntry`).
- **`rx_buffer`** — compile-time const in C/C++ (`MESSAGE_BUFFER_SIZE` / opaque
  storage), so it is *not* a runtime options field (the one knob that stays
  Rust-builder-only, since C can't size inline storage at runtime).
- **`PublisherOptions`** is thin (no callback ⇒ no sched/info); kept for
  rclcpp symmetry + future intra-process / loaned-message knobs.

C services/actions take no QoS today — M3 adds `_with_qos` / `_with_options`
parity there. See [Phase 189 M3](../roadmap/archived/phase-189-entity-api-tiers.md) for
the slice breakdown.

## Phasing

Tracked as **[Phase 189](../roadmap/archived/phase-189-entity-api-tiers.md)** (split from
Phase 172 — this is a runtime client-API refactor, not orchestration). M1 (the
Rust builder + convenient surface, incl. `.message_info()` + `.session()`)
unblocks the Phase 172 `[[bridge]]` topic-forwarding runtime half; M2 retires the
`register_*` zoo + points the generator at the builder; M3 adds the C/C++
named-options parity; M4 sweeps call sites + deletes the shims.

## Migration (additive, no break)

1. Land the builder (`node.subscription(t)` / `node.publisher(t)`) over the
   existing core — each `register_*` becomes a one-line delegate.
2. Keep the convenient `create_publisher`/`create_subscription` (+ generic)
   stable — they're the upstream-matching surface; re-point them at the builder.
3. **Remove the `register_subscription_*_*_*` zoo** — deleted outright, callers
   migrated in the same change (**no deprecation window** — decided 2026-05-28).
   It is an internal, unpublished surface (only the generator + tests/examples +
   the C FFI call it), so a deprecation release buys nothing. Three closure
   cores (`register_subscription_buffered_on`/`_raw_on`/`_raw_info_on`) survive
   as `pub(crate)` builder-lowering targets, plus one clean-named C-FFI fn-ptr
   core for `nros-c`; everything else goes. Per the naming policy, no
   `_raw_with_qos_sized_on`-style public identifier remains. See
   [Phase 189 M2](../roadmap/archived/phase-189-entity-api-tiers.md).
4. **The generator emits builder calls**, so generated code reads like
   hand-written application code (the orchestration ⇄ application symmetry the
   bridge design relies on).

## How the bridge uses this

No new flat function. The bridge relay (see
[`0009-bridge-topic-forwarding.md`](0009-bridge-topic-forwarding.md)) is just the
convenient generic subscription with two knobs set:

```rust
node_src.subscription(topic)
    .generic(type_name, type_hash).qos(qos)
    .message_info()                 // for the bridge_origin echo check
    .session(src_slot)              // 172.K.5 selector
    .build(move |payload, info| {
        if parse_bridge_origin(info.attachment()) == Some(ORIGIN) { return; }
        let _ = dst_pub.publish_raw_with_attachment(payload, &ORIGIN_ATT);
    })?;
```

`message_info` + `session` are pre-existing axes finally exposed as knobs rather
than as a new `register_subscription_raw_with_info_on`.
