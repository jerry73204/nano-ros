---
rfc: 0011
title: "PX4 RMW (`nros-rmw-uorb`) — design notes"
status: Stable
since: 2026-04
last-reviewed: 2026-04
implements-tracked-by: []
supersedes: []
superseded-by: null
---

# PX4 RMW (`nros-rmw-uorb`) — design notes

**Status:** v1 landed (Phase 90.1–90.5; 90.6/90.7 remaining). See
Phase 90 roadmap.

This document focuses on the nano-ros-specific design choices. For the
underlying async / WorkQueue / uORB story, see
[`px4-rs/docs/architecture.md`](https://github.com/aeon/px4-rs/blob/main/docs/architecture.md).

---

## 1. Why a separate RMW

uORB is fundamentally different from zenoh-pico and XRCE-DDS:

| Property            | zenoh-pico / XRCE-DDS                     | uORB                                  |
| ------------------- | ----------------------------------------- | ------------------------------------- |
| Wire format         | CDR-encoded                               | Raw `#[repr(C)]` memcpy               |
| Transport           | TCP/UDP/serial                            | In-process ringbuffer (kernel side)   |
| Discovery           | Dynamic (peer scouting / agent)           | Static (topics known at link time)    |
| QoS                 | Reliability, durability, depth            | Queue depth + interval throttle only  |
| Services / actions  | Native                                    | Not modelled (publish-only)           |
| Session model       | One per process; multiplexed across nodes | One per process; topics keyed by name |
| Thread / wake model | Background thread / poll loop             | PX4 WorkQueue wakes on publish        |

Folding uORB into `nros-rmw-zenoh` or `nros-rmw-xrce` would force the
network-shaped abstractions onto something that doesn't need them and
loses critical perf properties (zero-copy in-process delivery). Same
directory split as `packages/rmw/zenoh/` vs. `packages/rmw/xrce/`.

## 2. Style B vs Style C entry points

The phase plan distinguishes two ways users wire a nano-ros module under
PX4:

- **Style B** — board-crate `run(config, |executor| { … })` closure.
  Mirrors `nros-mps2-an385::run`, `nros-nuttx-qemu-arm::run`. The user
  closure registers nodes/publishers/subscribers/timers; `run()` parks
  the executor.
- **Style C** — `#[px4_workqueue::task(wq = "rate_ctrl")] async fn …`
  with topic recv inside an `async` body. Style C is enabled by
  `nros-rmw-uorb` + `px4-workqueue` directly — no extra nano-ros code
  is needed because `px4-workqueue` is style-agnostic.

v1 ships Style B via `nros_px4::run`.
Style C works today as long as the user is comfortable bypassing
`nros-node`'s typed Node abstraction.

## 3. Topic-name mapping

ROS 2 topic strings (e.g. `/fmu/out/sensor_gyro`) → uORB topic
identifiers (e.g. `sensor_gyro`) via a compile-time `phf::Map` generated
from `topics.toml` by
`build.rs`. The initial subset mirrors PX4-Autopilot's
`src/modules/uxrce_dds_client/dds_topics.yaml`.

Lookup is O(1) hash + string compare; misses return
`TransportError::InvalidConfig` from `Session::create_publisher` /
`create_subscription`.

## 4. Two API layers

### 4a. Direct typed pub/sub (primary)

Users generate PX4 message types via `px4-msg-codegen` and call
`nros_rmw_uorb::publication::<T>(ros_name, instance)` /
`subscription::<T>(...)` to obtain a typed `px4_uorb::Publication<T>` /
`Subscription<T>`. The function performs two validations:

1. `ros_name` must be present in `topics.toml`.
2. The mapped uORB name must match `T::metadata().o_name`.

On success the returned handle behaves exactly like a hand-constructed
`Publication<T>::new()` — same lazy-advertise, same waker chain, same
zero-copy semantics. The validation step catches "wrong type for this
topic name" mistakes at module init rather than first publish.

This is the **recommended path** for new code. Zero overhead beyond
`px4-uorb`'s own implementation.

### 4b. Type-erased trampoline registry (nros-node compat)

For users who want nros-node's typed `Node`/`Publisher<M>`/`Subscription<M>`
abstractions, [`crate::register`] populates a HashMap-backed registry
that bridges `nros_rmw::Publisher::publish_raw(&[u8])` onto typed
`Publication<T>`/`Subscription<T>` instances.

The trampoline registry is the bridge:

```
register::<T>("/ros/name", instance)
        │
        ▼
HashMap<&str, Box<dyn TopicHandle>>
        │
        ▼ at publish_raw / take_serialized time
Handle<T> { Publication<T>, Subscription<T> }
        │
        ▼ ptr::read_unaligned(&[u8] → T::Msg) / ptr::copy_nonoverlapping(T::Msg → &[u8])
px4_uorb broker
```

Trade-off: HashMap lookup + mutex acquire per publish. For high-rate
topics (>1 kHz control loops) prefer the direct API in §4a.
Trampoline-registered topics also require **explicit user registration**
at module init via [`crate::register`]; the direct API does not (it
goes straight through the typed `Publication<T>`).

Why not generate a per-topic Cargo build-script trampoline? Because
trampolines must own typed `Publication<T>`/`Subscription<T>` instances
that aren't const-constructible across language boundaries. A runtime
registry is simpler and lets us add topics without re-running
`cargo nano-ros generate`.

Alternative considered and rejected: bypass `px4-uorb` entirely and call
`px4-sys::orb_advertise_multi` + `orb_publish` directly with the
metadata pointer. Works on target but breaks the host-mock test path
(mocks are private to `px4-uorb`).

## 5. Memory model — no CDR

PX4 uORB uses raw memcpy. Bytes flowing through `publish_raw` are
interpreted as the PX4 message struct via
`core::ptr::read_unaligned(buf as *const T::Msg)`. Length must match
`size_of::<T::Msg>()`; mismatches return
`TransportError::BufferTooSmall`.

This means **users must use the same `.msg` definition on both ends** —
typically by feeding the same PX4 `.msg` file to `px4-msg-codegen` in
each crate that touches the topic. ROS 2 message types like
`sensor_msgs/msg/Imu` have entirely different layouts from PX4's
`sensor_combined`; bridging requires a schema-mapping shim that v1
does not generate.

For service-heavy workloads or where wire-compatible ROS 2 messages
matter, recommend XRCE-DDS over uORB.

## 6. Zero-copy: actually serialization-free

The phase doc and earlier drafts loosely called this "zero-copy". More
precise: **serialization-free**. `orb_copy` still memcpys into the
subscriber's caller-supplied buffer, and `orb_publish` memcpys from the
caller's struct into the broker's ring. There is no CDR encode/decode
overhead; that's the win.

## 7. Service / action gap — Phase 90.4 follow-up

uORB has no native request/response. The two viable mappings:

1. **Paired-topic + correlation ID.** Service `/foo` becomes uORB
   topics `foo_request` + `foo_reply`, each carrying a `seq: u32`
   correlation field. Client publishes request, polls reply topic
   for matching seq. Simple but doubles topic count.
2. **Recommend XRCE for service workloads.** uORB users get pub/sub;
   anyone who needs services uses `nros-rmw-xrce` (which can run on
   the same PX4 module via the uxrce-dds-agent bridge).

Decision deferred. v1 returns
`TransportError::Backend("uORB: services not yet supported (Phase 90.4)")`.

## 8. Spin loop vs. WorkQueue waker — Phase 90.5b follow-up

v1 `nros_px4::run` parks the executor in a 10 ms `spin_once` loop.
This is functionally correct but burns CPU when no topics are active.

90.5b will:

1. Allocate a `WorkItemCell` for the executor task.
2. Register a wake callback into `nros-rmw-uorb`'s subscription path
   so each uORB callback calls `ScheduleNow()` on the executor's
   WorkItem.
3. Replace `spin_once + Duration::from_millis(10)` with
   `spin_once(0); yield_to_workqueue()`.

This eliminates polling cost on quiescent topics and matches the
async model documented in `px4-rs/docs/async-model.md`.

## 9. Cross-cutting

- **`Z_FEATURE_MULTI_THREAD` from zenoh-pico is not relevant.** uORB
  has its own concurrency story; the registry's `Mutex<HashMap>` is
  the only shared state we add.
- **`ffi-sync` is not enabled** on uORB. The relevant FFI is
  px4-sys, and px4-uorb already serialises access to `g_session`-
  equivalent state internally.
- **Discovery vs liveliness** — neither concept exists in uORB.
  `Session::create_publisher` validates the topic is in `topics.toml`
  and returns immediately; the underlying `Publication<T>` lazy-
  advertises on first publish.

## 10. Risks

- **uORB metadata ABI drift.** If PX4 changes `orb_metadata` layout,
  `px4-sys` regenerates bindings and we recompile. Low risk; the
  struct has been stable for years.
- **Symbol collision in std mock.** `px4_uorb`'s mock broker is
  process-global. Tests must call `px4_uorb::_reset_broker()` +
  `nros_rmw_uorb::_reset()` between cases or they see stale state.
  Both helpers are gated behind `feature = "test-helpers"`.
- **PX4 build integration not yet exercised.** Phase 90.6 will write
  the first example module; until then we only know the wrapper crate
  compiles, not that it links into PX4 successfully.
