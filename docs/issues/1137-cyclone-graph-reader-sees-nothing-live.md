---
id: 1137
title: "Cyclone's graph reader sees only itself against a live stock talker, and 9 of its 12 graph slots answer Unsupported — measured, first live run"
status: open
type: bug
area: rmw, testing
severity: high
found: 2026-09-06
related: [0791, 0903, 1127]
---

# The first live run of `graph_interop`, and Cyclone fails it

phase-433 W1 ran `graph_interop` against a real ROS 2 Humble peer for the first
time since it was written. Inside the `ros2` distrobox, box tree at `45e6ced2d`,
fixtures built minutes before the run.

```
Starting 2 tests across 1 binary
    PASS [  18.762s] (1/2) graph_interop nano_ros_enumerates_a_stock_ros2_node
    FAIL [  37.468s] (2/2) graph_interop cyclone_enumerates_a_stock_ros2_node
Summary [  37.470s] 2 tests run: 1 passed, 1 failed, 0 skipped
```

**zenoh passes.** That is the first live proof of the twelve graph slots
phase-381 shipped and issue 0903 repaired — the mechanism (a standing liveliness
subscriber with history, rather than repeated `z_liveliness_get` interests)
works against a stock `rmw_zenoh_cpp` peer.

**Cyclone does not.** The probe's own output:

```
GRAPH_PROBE_DOMAIN 1
GRAPH_NODE /|graph_probe
GRAPH_PROBE_NODE_COUNT 1
GRAPH_PROBE_TOPICS_ERR Transport(Unsupported)
GRAPH_PROBE_TOPIC_COUNT 0
GRAPH_SERVICE_COUNT 0
GRAPH_SUB_BY_NODE_COUNT 0
GRAPH_CLIENT_BY_NODE_COUNT 0
GRAPH_SUB_INFO_COUNT 0
GRAPH_SLOT_UNSUPPORTED get_service_names_and_types
GRAPH_SLOT_UNSUPPORTED count_publishers
GRAPH_SLOT_UNSUPPORTED count_subscribers
GRAPH_SLOT_UNSUPPORTED get_publisher_names_and_types_by_node
GRAPH_SLOT_UNSUPPORTED get_subscription_names_and_types_by_node
GRAPH_SLOT_UNSUPPORTED get_service_names_and_types_by_node
GRAPH_SLOT_UNSUPPORTED get_client_names_and_types_by_node
GRAPH_SLOT_UNSUPPORTED get_publishers_info_by_topic
GRAPH_SLOT_UNSUPPORTED get_subscriptions_info_by_topic
GRAPH_PROBE_UNSUPPORTED_COUNT 9
GRAPH_PROBE_SLOTS_PARTIAL
GRAPH_PROBE_FAIL: expected a node matching "talker", saw ["/|graph_probe"] after 20000 ms
```

So on Cyclone: **`get_node_names` returns one entry, the probe itself.
`get_topic_names_and_types` errors `Transport(Unsupported)`. Nine of the twelve
slots answer Unsupported outright.** The reader phase-381 W5 added never sees a
participant.

## The peer is not the problem — control run

Before blaming the reader (issues 0859–0862: four ghost issues filed from
failures whose cause was the harness, two with confident wrong root causes), the
same box was asked whether a stock Cyclone talker is discoverable at all:

```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=77 ros2 run demo_nodes_cpp talker &
$ ros2 node list --no-daemon
/talker
$ ros2 topic list --no-daemon
/chatter
/parameter_events
/rosout
```

Discovery on Cyclone works in this container. The test's own setup is also
sound: `unique_ros_domain_id()`, a stock
`demo_nodes_cpp_talker_cyclonedds_with_domain` on that domain, and the probe
given the same `ROS_DOMAIN_ID`/`NROS_DOMAIN_ID`.

## Why nothing caught it earlier

This is phase-393's closing warning, demonstrated a second time. All twelve
Cyclone graph slots are `produced` by `check-rmw-slot-producers`, the parity
gate reports 0 gap, and every unit test passes — because each tests our code
against our own builders and our own parser. `interop::CELLS` even carries the
prediction, written when the reader landed:

> Cyclone's half. `graph.cpp` PUBLISHED `ros_discovery_info` since phase-177.36
> and only gained a reader in W5, **which has never been run against a live
> participant.**

It had been correct and unmeasured for as long as it had existed. Issue 1127 is
why: the lane that would have run this has not completed in 30 runs (issue
1136), so the prediction never became a verdict.

## What needs deciding, before fixing

The nine `Unsupported` slots may be a deliberate partial from W5 rather than
breakage — the phase shipped a reader for `ros_discovery_info`, which carries
node/topic participation, not every by-node and by-topic query. **Establish
which of the twelve W5 intended to answer on Cyclone before treating nine
Unsupported as nine bugs.** If the partial is intended, the honest fix is that
`check-rmw-slot-producers` should not count a slot as `produced` for a backend
that answers `Unsupported` at runtime — a distinction the gate does not
currently draw, and one that would change the "34 of 88 working" headline.

The remaining three are unambiguous: `get_node_names` returning only self while
a live participant is on the domain is a defect, and
`get_topic_names_and_types` answering `Transport(Unsupported)` contradicts its
`produced` classification.

## Reproduce

Inside the `ros2` box, on the box tree (issue 0759), sourcing `activate.sh`
FIRST and `ros2-box-env.sh` SECOND:

```bash
bash scripts/build/fixtures-build.sh linux rust cyclonedds
cargo nextest run -p nros-tests --test graph_interop -E 'test(cyclone)'
```
