# QoS, Status Events, and Discovery

This page covers runtime event behavior that application authors need
after choosing an RMW backend: liveliness, deadline misses, lost
messages, and the places where nano-ros deliberately differs from
standard ROS 2 waitset-style event handling.

Transport-level status events let an application observe conditions
the underlying middleware detects: a remote node going silent, a
publisher missing its rate, a subscriber dropping samples. nano-ros
exposes a Tier-1 subset useful for embedded RTOS deployments —
liveliness changes, deadline misses, and message loss.

This page describes the event surface, the dispatch model, and the
patterns that motivate having events at all on RTOS targets.

## The Tier-1 events

Five event kinds. Three on subscribers, two on publishers.

### Subscriber events

| Kind | Fires when | Use |
|------|-----------|-----|
| `LivelinessChanged` | A tracked publisher's liveliness state changes (publisher started asserting / stopped) | Safety-island fail-over: trigger MRM when remote control node goes silent |
| `RequestedDeadlineMissed` | An expected sample didn't arrive within the configured deadline | Periodic-sensor pattern: 100 Hz topic with 15 ms deadline; fire alarm when late |
| `MessageLost` | The backend dropped one or more samples (typically: ring-buffer overflow, slow consumer) | Diagnostic + adaptive: log, drop more aggressively, coalesce |

### Publisher events

| Kind | Fires when | Use |
|------|-----------|-----|
| `LivelinessLost` | This publisher missed its own liveliness assertion deadline | Self-monitoring: detect own task starvation |
| `OfferedDeadlineMissed` | This publisher promised X Hz, fell behind | Self-monitoring: detect overrunning compute |

## Registering callbacks

Same shape as message callbacks. Register at construction time;
callback fires from inside `spin_once` when the backend detects the
event.

### Rust

```rust
use core::time::Duration;

let mut sub = node.create_subscription::<SensorReading>("/sensor")?;

// Liveliness — fires when the publisher comes / goes
sub.on_liveliness_changed(|status| {
    if status.alive_count == 0 {
        log::warn!("sensor publisher went silent");
        trigger_failover();
    }
})?;

// Deadline — fires when no sample within 15 ms
sub.on_requested_deadline_missed(Duration::from_millis(15), |status| {
    metric_inc(&LATE_SAMPLE_COUNT, status.total_count_change);
})?;

// Message lost — fires when backend drops a sample
sub.on_message_lost(|status| {
    log::warn!("dropped {} samples", status.total_count_change);
})?;
```

Async equivalents (`spin_async` / Embassy / tokio):

```rust
let status = sub.next_liveliness_change().await?;
let status = sub.next_deadline_miss().await?;
```

### C

```c
static void on_liveliness_changed(
        nros_subscription_t *sub,
        nros_liveliness_changed_status_t status,
        void *user_context) {
    if (status.alive_count == 0) {
        trigger_failover();
    }
}

nros_subscription_set_liveliness_changed_callback(
    sub, on_liveliness_changed, NULL);
```

Same shape on the four other event kinds. Each returns
`NROS_RMW_RET_UNSUPPORTED` if the active backend doesn't generate
that event.

### C++

```cpp
sub.on_liveliness_changed([&](nros::LivelinessChangedStatus status) {
    if (status.alive_count == 0) trigger_failover();
});

sub.on_requested_deadline_missed(
    std::chrono::milliseconds(15),
    [&](nros::DeadlineMissedStatus status) {
        late_count_ += status.total_count_change;
    });
```

`std::function` overloads available with `NROS_CPP_STD`; freestanding
mode uses C function pointers + user-context.

## How dispatch works

Events fire from inside `drive_io` on the executor thread, the same
way message callbacks do:

```text
spin_once(timeout)
 └─ session.drive_io(...)
       ├─ backend RX worker detects message → fires message callback
       ├─ backend RX worker detects liveliness change → fires liveliness callback
       ├─ backend timer expires (deadline) → fires deadline callback
       └─ return
```

Same execution context, same priority, same constraints as message
callbacks. The `max_callbacks_per_spin` knob (see
[RTOS Cooperation](rtos-cooperation.md)) covers events too — an
event callback counts as one against the cap.

This means events get the same scheduling treatment as messages.
`max_callbacks_per_spin = 1` will fire either one message OR one
event per `spin_once`, whichever the backend has ready first.

## Why a callback model, not a waitset

Upstream `rmw.h` exposes events via a waitset — register an
`rmw_event_t` handle in a waitset, call `rmw_wait`, then `rmw_take_event`
to pull payload. nano-ros doesn't have a waitset (see
[RMW vs upstream](../design/rmw-vs-upstream.md) Section 4).

Reusing the existing message-callback path avoids introducing the
waitset. Trade-off: applications can't bulk-poll for events. For the
Tier-1 set this isn't load-bearing — events are rare, callbacks
are cheap.

## Backend support is uneven

Not every backend can generate every event. Apps must handle
`Unsupported` errors when registering:

| Backend | `LivelinessChanged` / `Lost` | `DeadlineMissed` | `MessageLost` |
|---------|------------------------------|------------------|---------------|
| Cyclone DDS | ✗ Not wired through the nano-ros event API yet | ✗ Not wired yet | ✗ Not wired yet |
| XRCE-DDS | ✗ Not exposed (xrce-dds-client API limitation) | ✅ Shim-side clock check (sub: `has_data` / `take_serialized`; pub: `publish_raw`) | ✗ Not exposed |
| zenoh-pico | ✅ Poll-based via zenoh tokens (alive_count ∈ {0,1}) | ✅ Shim-side clock check (`<P as PlatformClock>::clock_ms`) | ✅ Seq-gap detection from RMW attachment |
| uORB | ✗ No wire-level liveliness | ✗ No rate concept | ✅ Native (host mock + real PX4 via `RustSubscriptionCallback` publish-counter) |

✓ = wired and tested. 🟡 = surface API works (returns Err while pending), wiring planned. ✗ = not feasible at this layer.

No current backend exposes the full Tier-1 event set through the
nano-ros event API. Apps should call `Subscriber::supports_event(kind)`
first or design for graceful fallback.

The `Subscriber::supports_event(kind)` query lets applications check
support before registering:

```rust
if sub.supports_event(EventKind::RequestedDeadlineMissed) {
    sub.on_requested_deadline_missed(...)?;
} else {
    // fallback: app-side timeout monitoring
}
```

Apps that need cross-backend portability code defensively. Apps
pinned to one backend can call register-and-unwrap.

## What's deliberately skipped

Three upstream event types are intentionally absent from the API:

### `*_MATCHED` (publication / subscription matched)

Fires when a remote endpoint appears or disappears. Useful for
discovery-tracking dashboards and dynamic-topology apps. Static-
topology embedded apps don't benefit; if a use case appears, the
event kind is additive.

### `*_QOS_INCOMPATIBLE`

Fires when publisher and subscriber QoS profiles can't be
reconciled. nano-ros surfaces this synchronously at `create_publisher`
/ `create_subscription` time as `NROS_RMW_RET_INCOMPATIBLE_QOS`. No
runtime event needed; the failure is visible at startup.

### `*_INCOMPATIBLE_TYPE`

Type-hash mismatch. Same: surfaced synchronously as
`NROS_RMW_RET_TOPIC_NAME_INVALID` (or a future `INCOMPATIBLE_TYPE` code,
unprefixed here because `rmw_ret.h` does not define one yet — issue 1126)
at creation. No runtime event.

## Patterns

### Drone-bridge fail-over on liveliness loss

```rust
let mut sub = node.create_subscription::<VehicleAttitude>("/vehicle_attitude")?;
sub.on_liveliness_changed(|status| {
    if status.alive_count == 0 {
        // PX4 commander.cpp went silent — trigger MRM
        request_minimum_risk_manoeuvre();
    } else if status.alive_count_change > 0 {
        // commander came back online; clear MRM if appropriate
        clear_failover();
    }
})?;
```

This pairs with the cross-backend bridge example pattern.

### 100 Hz sensor with deadline alarm

```rust
let mut sub = node.create_subscription::<SensorReading>("/imu")?;
sub.on_requested_deadline_missed(
    Duration::from_millis(15),    // expected 100 Hz, allow 15 ms deadline
    |status| {
        if status.total_count_change > 0 {
            log::error!("IMU late: {} missed deadlines", status.total_count_change);
            // Optional: enter degraded-mode controller
        }
    },
)?;
```

### Slow-consumer logging

```rust
let mut sub = node.create_subscription::<Pointcloud>("/lidar")?;
sub.on_message_lost(|status| {
    log::warn!("dropped {} pointcloud frames (total: {})",
               status.total_count_change, status.total_count);
    // Tighten downstream filter, reduce work, coalesce, etc.
})?;
```

## See also

- [RMW API: Differences from upstream `rmw.h`](../design/rmw-vs-upstream.md)
  Section 8 — the design comparison this page expands on.
- [RTOS Cooperation](rtos-cooperation.md) — `max_callbacks_per_spin`
  treatment of event callbacks.
- [Differences from ROS 2](ros2-comparison.md) — broader surface
  comparison.
