---
rfc: 0019
title: "nros-c Thin-Wrapper Discipline (Phase 122.3.a)"
status: Stable
since: 2026-05
last-reviewed: 2026-05
implements-tracked-by: []
supersedes: []
superseded-by: null
---

# nros-c Thin-Wrapper Discipline (Phase 122.3.a)

**Status:** Audit complete. Refactor plan TBD as 122.3.b/c/d.
**Date:** 2026-05-13.
**Owner:** Phase 122 (Unify C / C++ / Rust API code paths).

> **Build/link contract.** The C ABI nros-c wraps resolves through one canonical
> `<nros/platform.h>` (`nros-platform-api`); board capabilities + the RMW backend
> link are config/manifest-driven — see
> [RFC-0042](0042-platform-build-determinism.md) +
> [c-api-cmake.md](../reference/c-api-cmake.md) "Board capabilities & deterministic
> build".

## Why this exists

Phase 122 establishes the rule: **the Rust API is the source of truth.
C / C++ wrappers are thin shims that delegate.** Today's nros-c
violates this unevenly — some entities are thin wrappers (publisher,
executor, support), others duplicate Rust-side bookkeeping in
C-visible struct fields (subscription, service, action, timer,
guard).

This doc captures the audit + lays down the per-entity refactor target.

## Audit (current state — 2026-05-13)

For each entity, three categories:

- **Opaque-thin (✅).** Public C struct holds the Rust entity inline
  via `_opaque: [u64; <ENTITY>_OPAQUE_U64S]` (size probed by
  `nros-sizes-build`). All ops cast `_opaque` to the Rust type +
  delegate. The C struct IS the Rust value.

- **Field-mirror (❌).** Public C struct exposes parallel storage for
  what Rust already tracks: `topic_name: [u8; N]`, `type_name`,
  `callback`, `context`, `qos`, `handle_id`, etc. C-side validation
  duplicates Rust-side validation. The real entity lives elsewhere
  (executor arena); C struct is a "registration packet".

- **Mixed (⚠️).** Combines `_opaque` for the Rust entity with
  field-mirror metadata.

| Entity | Pattern | C struct fields beyond `state` + `_opaque` |
|---|---|---|
| `nros_publisher_t` | ✅ opaque-thin | topic_name + type_name + type_hash + node + `_opaque` |
| `nros_support_t` (RmwSession) | ✅ opaque-thin | `_opaque` only |
| `nros_executor_t` | ✅ opaque-thin | `_opaque` only |
| `nros_node_t` | ✅ opaque-thin-ish | name + namespace + support* |
| `nros_subscription_t` | ❌ field-mirror | topic_name + type_name + type_hash + callback + context + node + qos + handle_id |
| `nros_service_t` (server) | ❌ field-mirror | service_name + type_name + type_hash + callback + context + node + `_internal` |
| `nros_client_t` | ❌ field-mirror | service_name + type_name + type_hash + response_callback + context + node + `_internal` |
| `nros_action_server_t` | ❌ field-mirror | action_name + type_name + type_hash + goal_callback + cancel_callback + accepted_callback + context + node + `_internal` |
| `nros_action_client_t` | ❌ field-mirror | action_name + type_name + type_hash + 3 callbacks + context + node + `_internal` |
| `nros_timer_t` | ❌ field-mirror | period_ns + last_call_time_ns + callback + context + `_internal` |
| `nros_guard_condition_t` | ⚠️ mixed | callback + context + `_guard_opaque` for `GuardCondition` |

Score: 4 of 11 entities follow the discipline.

## Why the split exists historically

The pattern divergence reflects when each entity was added:

- **Opaque-thin entities** (publisher, support, executor) are
  *self-contained Rust values*. The C struct allocates inline
  storage; the Rust value lives there for its lifetime.
- **Field-mirror entities** (subscription, service, action, timer,
  guard) are *executor-registered handlers*. The actual entity (the
  Rust `RawSubscription` / `SrvEntry` / etc.) lives in the
  executor's arena. The C struct only carries the *registration
  packet* — topic name, type info, callback, etc. — consumed at
  `nros_executor_register_*` time.

The split made sense when only the L2 (callback) path existed —
field-mirror was the cheapest way to ship a callback registration
through the C ABI. Phase 122 introduces L1 (primitive polling),
which puts the entity inside the C struct. Field-mirror doesn't
extend to L1 cleanly.

## Target shape (post-122.3.b)

### Universal rules

1. **Every public C struct = `state` + `_opaque: [u64; N]` only.**
   No bookkeeping fields visible to C. `_opaque` is sized by
   `nros-sizes-build` to fit the Rust entity (L1) OR the
   registration-packet Rust value (L2).

2. **All bookkeeping lives in Rust.** Topic name validation, type
   info copying, QoS settings, callback storage, handle ID — all
   private to Rust. The C ABI exposes only entry points.

3. **C entry points do boundary conversion + delegate.**
   - `*const c_char` → `&str` via `CStr::from_ptr` + `to_str`.
   - Null-pointer checks at the boundary.
   - Result mapping (`Result<T, NodeError>` → `nros_ret_t`).
   - No business logic in C.

4. **No C-side duplicated validation.** `MAX_TOPIC_LEN` check,
   `validate_state!` macro, etc. — these belong inside the Rust
   impl, not the C wrapper. The C wrapper trusts Rust to validate.

5. **Getters delegate via opaque cast.** `rcl_subscription_get_topic_name`
   reads the topic name from the Rust value inside `_opaque`, doesn't
   read a C-side mirror byte array.

### Per-layer adaptation

**L1 (primitive, caller polls):**
- `_opaque` holds the actual entity — `RawSubscription<RX_BUF>`,
  `RawService<...>`, `RawActionClient<...>`, etc.
- Entity created in `nros_<entity>_init` via Rust's `Node::create_*`.
- Entity destroyed in `nros_<entity>_fini` via `Drop`.
- Polling ops (`take_serialized`, `take_request`, `send_response`,
  …) delegate to the in-`_opaque` Rust value.

**L2 (callback, executor-managed):**
- `_opaque` holds a registration-packet Rust value — basically
  the inputs `register_<entity>_*` will consume. After
  `nros_executor_register_<entity>` runs, the entity itself lives
  in the executor's arena; the C struct's `_opaque` holds a handle
  ID pointing at that arena slot.
- Callback fn pointer goes into the registration-packet Rust value
  as a `unsafe extern "C" fn(...)` field — no longer a public C
  struct field. Set via `nros_<entity>_init_handler(..., cb, ctx)`.

### Why opaque-for-L2 too

Even L2 benefits from the opaque shape:

- ABI stability: growing the registration packet (e.g. new QoS
  fields) doesn't break consumer struct layout — `_opaque` size is
  probed.
- Field-mirror today already changes shape across releases
  (`_internal` field added in Phase 87.5 broke ABI). Opaque
  prevents this.
- Consistency: one shape across all entities. Easier for ABI
  validation, docgen, and example reading.

## Refactor plan (122.3.b/c/d)

### 122.3.b — subscription (template)

Steps:
1. Define Rust-side `SubscriptionInner<RX_BUF>` enum:
   - `Polling(RawSubscription<RX_BUF>)` — L1.
   - `Registration { topic, type_name, type_hash, callback, context, qos, handle_id: Option<HandleId> }` — L2.
2. Probe size: `SUBSCRIPTION_OPAQUE_U64S = u64s_for::<SubscriptionInner<DEFAULT_RX_BUF>>()`.
3. New `nros_subscription_t` shape:
   ```rust
   #[repr(C)]
   pub struct nros_subscription_t {
       pub state: nros_subscription_state_t,
       pub _opaque: [u64; SUBSCRIPTION_OPAQUE_U64S],
   }
   ```
4. New entry points:
   - `nros_subscription_init` (L2, with callback) → builds `Registration` variant.
   - `nros_subscription_init_polling` (L1, no callback) → builds `Polling` variant by calling `Node::create_subscription_raw_sized`.
   - `nros_subscription_take_serialized` (L1 op) → match on inner; if `Polling`, delegate.
   - `nros_subscription_fini` → drop in place.
   - `nros_executor_register_subscription` (L2 op) → match on inner; if `Registration`, call `executor.register_subscription_raw_*` with the stored callback.
5. Delete the old field-mirror fields from the public ABI.
6. Update C/C++ examples that reach into those fields (rare —
   most code goes through entry points).
7. Smoke test: L1 polling round-trip + L2 callback round-trip.

### 122.3.c — service, service_client, action_server, action_client

Same shape per entity. Each gets:
- Internal enum: `Polling(<RawForm>)` + `Registration { ..., callback, ... }`.
- `_opaque` sized to whichever variant is larger.
- L1 init + L1 op + L2 init_handler + L2 register entry points.

Per-entity ops to ship in L1:

| Entity | L1 op |
|---|---|
| service (server) | `take_request` + `send_response` |
| service_client | `send_request` + `take_response` |
| action_server | `try_recv_goal` + `accept/reject` + `publish_feedback` + `send_result` |
| action_client | `send_goal` + `try_recv_feedback` + `try_recv_result` |

(Match the existing Rust manual-poll API on `EmbeddedServiceClient`,
`ActionServer`, etc.)

### 122.3.d — C++ surface

`nros-cpp` headers wrap the C struct. With the unified opaque
shape on the C side, the C++ wrappers become:

```cpp
class Subscription {
public:
    // L1 — primitive ctor.
    Subscription(Node& node, const TopicInfo& topic);
    // L2 — handler ctor.
    template<typename F>
    Subscription(Node& node, const TopicInfo& topic, F&& callback);

    // L1 op (returns std::nullopt when no data).
    template<typename M>
    Result<std::optional<M>> take();
};

class Executor {
public:
    Result<HandleId> register_(Subscription& sub);
    // ... and so on for each entity.
};
```

The C++ class holds the C struct by value or via the
`SubscriptionStorage`-style inline-storage pattern (already used
for Executor / Node).

## What changes for consumers

- **Rust users:** no change (Rust API is the source of truth).
- **C users:** stop reading `sub.topic_name` / `sub.callback`
  fields directly. Use getters (e.g. `rcl_subscription_get_topic_name`).
  These already exist for most entities; missing ones get added.
- **C++ users:** mostly transparent — wrappers hide the change.
  C++ consumers that reach into `.handle.field_x` need to switch
  to method calls. Rare.

## ABI compatibility

Refactor IS an ABI break — struct sizes + field offsets change.
Consumers must recompile. No silent breakage (compiler catches at
build time). Phase 115's RMW backend rename already established
"breaking changes welcome" until first stable release.

## Validation

After 122.3.b/c/d land:

1. `cargo check --workspace`: clean.
2. `just check`: green.
3. `just test-unit`: 391/391 + N new (L1 polling round-trip tests).
4. `just native check`: all C/C++ examples build.
5. Audit re-run: 11/11 entities marked ✅ opaque-thin.

## Out of scope (deferred)

- **Header-only C API.** Some entities could become header-only via
  small `#define` shims if the C struct is just `_opaque`. Not a
  priority — keep cbindgen-emitted decls.
- **Removing public state field.** `state` could move into `_opaque`
  too. Keeping it visible is useful for debug visibility and
  zero-init detection. No change.
- **C-side `take` typed variants.** Today's C API has only
  raw-bytes ops. Typed `take` (with CDR decode in the
  consumer's language) is a future addition — `nros_<entity>_take_serialized`
  is enough for 122.3.
