---
id: 1137
title: "Cyclone's graph reader saw only itself against a live stock talker — the
  harness pinned the PEER's bus to loopback and not ours, and nine of the twelve
  `Unsupported` slots were never a defect"
status: resolved
type: bug
area: rmw, testing
severity: high
found: 2026-09-06
related: [0791, 0903, 0927, 1009, 1127]
---

> **State: RESOLVED 2026-09-07 by the measurement this issue asked for.**
> `graph_interop`'s cyclone case now PASSES live. Controlled before/after in
> the `ros2` distrobox, same machine, same fixtures rebuilt each side, only the
> tree differing:
>
> | tree | result |
> | --- | --- |
> | `91f3ce7c9` (pre-fix) | FAIL after 20 s, `GRAPH_PROBE_NODE_COUNT 1` |
> | `605d667eb` (with PR #659) | **PASS in 4.6 s** |
>
> Three other families moved with it, all previously unmeasured since the pin
> landed on 2026-09-04: `interop_e2e`'s `case_7_cyclone_pubsub_ros2_to_nano`
> and `case_8_cyclone_service_nano_server` (FAIL after 3 retries each → PASS),
> `declarative_bridge_zenoh_to_cyclonedds` (2/3 → 3/3) and
> `bridge_zenoh_to_cyclonedds` (3/4 → 4/4). The zenoh-only control
> `rust_multi_node_per_node_graph` stayed 2/2, which is what makes this a
> measurement rather than a coincidence.
>
> Recorded in `.config/interop-verdicts.toml` as
> `native-graph-rust-cyclone-r2n = pass`, written by `--record` from the junit.

# What this issue got wrong, and it was filed by the same person who fixed it

The original filing asserted three things. **All three were false**, and they
are kept here rather than edited away because the pattern is the point.

1. **"9 of 12 graph slots answer `Unsupported`" — intended, not a defect.**
   `vtable.cpp` fills `get_node_names` and leaves eleven graph slots explicitly
   `nullptr`. Phase-381 W5's scope was one slot, and three places in the tree
   said so already: the phase status line, `bins/graph-probe/src/main.rs:194`,
   and `graph_interop.rs:155`.
2. **"`get_node_names` returning only self is a reader defect" — it was the
   bus.** Issue 1009 pinned the ROS peer to loopback via a `source setup.bash
   && export …` string, which reaches a host `ros2` process; our side is a bare
   `std::process::Command` and got nothing. A loopback/no-multicast participant
   and a default-interface/multicast one cannot discover each other.
3. **"`Transport(Unsupported)` contradicts its `produced` classification" — it
   does not.** `check-rmw-slot-producers` answers "does ANY backend fill this
   slot", says so in its docstring, and never made a per-backend claim.

The dating was available before any of that was written: the cell PASSED live
on 2026-08-30 (issue 0927), `graph.cpp` has not changed since, and issue 1009
landed on 09-04. A regression bracketed by two dates was diagnosed as a
long-standing defect in code that had not moved.

**The one thing the original filing did right** was refuse to treat the nine
`Unsupported` as nine bugs without first establishing intent — that hedge is
why no wrong fix was written.

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

## The nine `Unsupported` slots are not a defect, and never were

Answered first, because nine of the eleven reported lines are a correct answer
and treating them as bugs is how a day gets spent in `graph.cpp`.

**Cyclone's vtable fills exactly ONE of the twelve graph slots.**
`nros-rmw-cyclonedds/src/vtable.cpp:380-391` is eleven consecutive `nullptr`s
under `/*get_node_names*/ cyclone_get_node_names,`, and a NULL slot is
`Transport(Unsupported)` by the ABI's own contract. Three independent sources
say that is the intent, all written when W5 landed:

* phase-381's status paragraph — *"zenoh answers all twelve, Cyclone answers
  `get_node_names`"*;
* `bins/graph-probe/src/main.rs:194` — *"Cyclone's W5 reader serves
  `get_node_names` and nothing else"*, which is why the probe RECORDS an
  `Unsupported` rather than failing on it;
* `graph_interop.rs:155` — *"Cyclone answers FEWER slots than zenoh, and that is
  the point of W6 rather than a defect."*

So the correct count is **10 of 12 `Unsupported` on Cyclone, all intended**: the
nine the probe classifies, plus `get_topic_names_and_types` (reported separately
as `GRAPH_PROBE_TOPICS_ERR`, and `nullptr` in the same block). The twelfth,
`node_get_graph_guard_condition`, is `nullptr` on every backend and is a
declared inert family in `check-rmw-slot-producers`. `Transport(Unsupported)` on
topics is an **honest answer**, not a contradiction of a `produced`
classification — see the classification section below.

That leaves exactly one real symptom: `get_node_names` returning only self.

## Root cause: the harness pinned the peer's bus and not ours

**Not the reader.** `graph.cpp` is unchanged since 2026-08-31 (a `std::fprintf`
spelling fix for threadx-riscv64) and this exact cell **passed live on
2026-08-30** — issue 0927 records `PASS cyclone_enumerates_a_stock_ros2_node`
after fixing the probe's `ROS_DOMAIN_ID`. The regression is dated:

| date | event |
| --- | --- |
| 2026-08-30 | issue 0927 fixed; both graph cells PASS live |
| 2026-09-04 | issue 1009 lands: DDS interop peers pinned to loopback |
| 2026-09-06 | this cell fails, "sees only itself", identical signature |

Issue 1009 confined every DDS interop peer to loopback, because a participant
elsewhere on the LAN is otherwise a peer (issue 0741 lost fifteen sections to
one). It did that in `ros2_env_setup_rmw_with_domain`, which builds a
`source setup.bash && export …` STRING — so the pin reaches exactly one kind of
process: **a host `ros2` peer.** Our own side of every pair is a bare
`std::process::Command`, and it got nothing.

For the Cyclone graph cell that means:

* the talker, via `demo_nodes_cpp_talker_cyclonedds_with_domain`, runs under
  `CYCLONEDDS_URI` = a profile with `NetworkInterface address="127.0.0.1"`,
  `multicast="false"`, `AllowMulticast=false` and `<Peer address="localhost"/>`
  — unicast SPDP on loopback, no multicast at all;
* `graph-probe` runs with Cyclone's DEFAULT interface pick, which on a
  distrobox (host networking) is a real ethernet interface, announcing SPDP to
  `239.255.0.1`.

Neither side's discovery traffic reaches the other. The probe's participant
still matches its OWN `ros_discovery_info` writer, so it enumerates one node,
itself — which is bit-for-bit the symptom issue 0927 already produced from a
different cause, and which reads as a broken reader.

0927's own cause is excluded by the run's first line: `GRAPH_PROBE_DOMAIN 1`
is `unique_ros_domain_id()`'s answer for a filtered/solo nextest run
(`domain_in_slot(0, 0)`), and `graph_interop.rs:188` hands the talker the same
value. The two sides agree about the domain and disagree about the bus.

**This is the failure mode issue 1009 measured and then half-applied.** Its own
conclusion: *pin both sides or neither; half is no discovery* — batch F, 0 of 15
with EMPTY output, because `ROS_LOCALHOST_ONLY` reached the ROS side and not the
XRCE Agent. The same asymmetry, one lane over, two days later.

### Why the issue's control run did not catch it

```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=77 ros2 run demo_nodes_cpp talker
ros2 node list --no-daemon   # -> /talker
```

Neither of those processes has `CYCLONEDDS_URI` set. The control is a
**both-unpinned** pair, which works; the harness is a **half-pinned** pair,
which does not. The control was right that discovery works in that container —
it just could not see the variable that the harness adds to one side only.

## The fix

One helper, applied at every spawn site whose peer is a pinned host `ros2`
process, plus a gate.

* `dds_isolation::apply_cyclone_config` / `apply_to_command` — the
  bare-`Command` twin of `env_exports_for_rmw`, and the sibling
  `apply_fastdds_profile` had been on its own since 1009.
* Applied at the five sites that pair a nano-ros DDS participant with a pinned
  host peer: `graph_interop.rs` (the probe), `interop_e2e.rs`
  (`spawn_nano_cyclone`, three cyclone scenarios), `ros2_action_e2e.rs` (the
  action server — the same defect, same day, never noticed),
  `bridge_zenoh_to_cyclonedds.rs` and
  `declarative_bridge_zenoh_to_cyclonedds.rs` (the Cyclone egress and the
  listener it feeds).
* Gate: **`check-dds-isolation-symmetry`** (fast line, buildless). A tracked
  test source that starts a pinned host DDS peer must also apply the pin to
  what it spawns. Mutation-tested: removing one call turns it red.

### What is deliberately NOT pinned

`ManagedProcess::spawn_command` is the obvious chokepoint and would be wrong.
The `DockerRosEnv` editions lanes run their ROS 2 peer inside a container whose
mount namespace cannot reach a host profile path, so those pairs are
**symmetric-unpinned** today; pinning our half of one would create this bug
rather than fix it. The gate therefore keys on `HostRosEnv` and the host
`Ros2*Process` helpers, never on the `Middleware` enum both backends share, and
`spawn_command` carries the reasoning in a comment so the next reader does not
have to re-derive it.

## The classification question: is `produced` lying?

The issue was filed saying *"all twelve Cyclone graph slots are `produced` by
`check-rmw-slot-producers`"*. That is a misreading, and worth recording because
it aimed at the wrong tool.

`check-rmw-slot-producers` answers **"does ANY backend's vtable fill this
slot"** — it says so in its own docstring, and its output is one number for the
whole ABI (`produced 58`). It has never made a per-backend claim, so it is not
lying about Cyclone; it simply never had the answer. Neither did anything else:
"zenoh 12, Cyclone 1, XRCE 0" lived only in prose, in three comments.

Fixed by measuring it. The report now carries a per-backend table:

```
## filled per backend (of 68 slots)

  cyclonedds                        38   graph 1/12  [get_node_names]
  rust-adapter (every Rust backend) 42   graph 11/12 [...]
  uorb                              18   graph 0/12
  xrce                              24   graph 0/12
  zenoh                              4   graph 0/12
```

**And that table still cannot answer the question for a Rust backend**, which
the report now says out loud. The adapter's graph slots are trampolines,
non-NULL for every `R: RustBackend`, forwarding to a `Session` trait method
whose default returns `Unsupported` — so zenoh reads as filled whether or not it
implements anything. No static tool closes that: the answer is a live call, and
`graph_interop` is the only thing that makes it.

So the recommendation is **not** to fail `--check` on a backend that answers
`Unsupported`. A slot a backend declines is RFC-0035's designed behaviour and
W6's whole point; the gap was visibility, and visibility is what was added.

## What remains unverified

The fix could not be run against a live peer from the authoring worktree — this
host has no ROS (`/opt/ros` absent), and the `ros2` distrobox tree was being
re-synced by another process (issue 0759: every job in the box, on its own
tree). What is proven locally: the gate, its mutation, the symmetry unit tests,
and that a child given `CYCLONEDDS_URI` can read the profile it names.

What needs the box to confirm:

```bash
bash scripts/build/fixtures-build.sh linux rust cyclonedds
cargo nextest run -p nros-tests --test graph_interop -E 'test(cyclone)'
```

The prediction is a PASS with `GRAPH_PROBE_NODE_COUNT 2` and
`GRAPH_PROBE_SLOTS_PARTIAL` (ten declared `Unsupported`, which the cell already
tolerates). If it still reports one node, the next measurement is
`NROS_GRAPH_DUMP=1`: `matched_publications=1` means the two participants still
do not see each other (look at the bus, not at `graph.cpp`),
`matched_publications>=2` with an empty enumeration means the reader really is
at fault this time.

Four other cells are predicted to change with it, all currently half-pinned and
all unmeasured since 2026-09-04: `interop_e2e`'s three cyclone scenarios and
`ros2_action_e2e`.

## The class

Third instance of "a variable that isolates one half of a pair", after issue
0741's foreign peer and issue 1009's own batch F. And the second time
`get_node_names` returning only itself has been filed as a Cyclone reader
defect when the reader was fine (issue 0927 is the first, and it says so in its
own title). A cyclone peer's BUS is load-bearing and silent when wrong: it does
not error, it reports an empty graph.
