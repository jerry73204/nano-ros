---
id: 1164
title: "a Cyclone application cannot observe a status event through either half of the surface — `*_event_init` is NULL and `*_take_event` has no caller"
status: open
type: bug
area: rmw
severity: medium
found: 2026-09-06
related: [0800, 1137]
---

# Both halves are missing, and each one alone would be enough

`packages/rmw/cyclonedds/*/src/vtable.cpp:258`:

```cpp
// Phase 108 event hooks left NULL until a follow-up phase wires
// Cyclone listeners through to the runtime's status-event surface.
constexpr rmw_ret_t (*kRegisterSubscriptionEvent)(...) = nullptr;
constexpr rmw_ret_t (*kRegisterPublisherEvent)(...)    = nullptr;
```

Both are installed into `kVtable` as those NULL constants. So on Cyclone there
is **no way to register** a status-event callback.

The other half is `*_take_event`, which Cyclone *does* implement — it reads real
`dds_get_liveliness_changed_status` counters. But it has **no caller anywhere in
the tree** outside cyclonedds' own `tests/status_events.cpp`: the Rust adapter
hardcodes both slots to `None`, and `nros-node` exposes no poll API. So the
implemented half is unreachable too.

Either gap alone would make status events unobservable on Cyclone. Both are
present.

## Why `check-rmw-slot-producers` still says `produced`

Because zenoh fills the slots. `produced` is a statement about the SLOT, not
about a backend — which is the overstatement issue 0800 measured and the reason
the gate now prints the split. This issue is the per-backend version of that
question, and it argues the split should go one level further: a slot filled by
one backend and NULL in another is not the same thing as a slot that works, and
today nothing distinguishes them.

Note the same shape one family over: issue 1137 found Cyclone's graph reader
answering `Unsupported` on nine of twelve `produced` slots, live.

## Not a live-peer finding, deliberately

phase-433 W6 set out to make a QoS event fire against a stock ROS 2 peer and
succeeded on zenoh (`qos_event_interop.rs`). Cyclone is recorded there as
`native-qos-event-rust-cyclone-r2n-CARVED` rather than as a failing cell,
because **a live peer changes nothing about this**: the missing piece is a
registration path and a runtime poll path on our side. A test would only ever
report the absence we already know from reading the source.

## Fix

Two independent pieces, and the first is cheap:

1. Wire `*_take_event` to a poll path — the implementation exists and reads real
   counters; what is missing is a caller and an `nros-node` surface.
2. Implement `*_event_init` over Cyclone listeners, which is what the phase-108
   comment deferred.

Either makes Cyclone status events observable for the first time. Doing both
makes the surface match zenoh's.
