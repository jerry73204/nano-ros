# Phase 433 — the RMW contract is closed; proving it works is a different job

**Status (2026-09-06). Not started. Analysis complete, jobs proposed below.**
This phase exists because phase-393 finished the RMW *contract* and said so,
then wrote its own warning at the bottom: `produced` means something writes a
slot and something reads it, **not** that either was ever exercised against a
real peer. This phase is that exercise.

**Implements** nothing new. Continues phase-393 (contract, complete) and
phase-381 (the graph slots, shipped and unproven). Owns issues 1127 and 0791;
touches 0902, 1008, 0814, 0823.

**Not to be confused with** phase-379 and phase-417, which are the USER API one
layer up. Nothing in this phase changes a signature.

## Where the contract stands, measured today

`just check rmw-api-parity`, 2026-09-06:

| bucket | n |
| --- | --- |
| vtable | 63 |
| global | 2 |
| layer | 3 |
| declined | 20 |
| **gap** | **0** |

`just check rmw-slot-producers`, same day: 68 slots — 58 `produced`, 4
`default`, 6 `inert`, 0 `unimplemented`. Seven of the 63 contract symbols in
the `vtable` column are answered by an inert slot.

So there is nothing left to *add*. Every remaining question is "does it work
against something we did not write".

## Why `produced` is not evidence

Phase-381 is the demonstration, and it is worth restating because it is the
whole argument for this phase. Twelve `rmw` graph slots shipped: produced,
reachable from Rust, C and C++, mutation-tested, `check-api-parity` clean.
**The feature did not work at all.** `z_liveliness_get` is an *interest*, and a
token reaches a get's callback only when the router tags its declaration with
that interest id — so a sweep saw an arbitrary handful of the domain's tokens.
Issue 0903 was several stacked defects sitting on top of a mechanism that could
not work. No unit test could see any of it.

Every check that passed tested our code against our own builders, our own
parser and our own vtable.

## The blocker: nothing runs the live-peer tests (issue 1127)

`nros_tests::interop::CELLS` is the intent list for live-peer work — 18 rows,
17 `Runtime`. Of the eleven distinct test binaries they name, **two are
reachable from a recipe** (`interop_e2e` via `just test-ros2` and
`just test-ros2-lifecycle`; `xrce_ros2_interop` via `just/xrce.just`). The
other nine are named by no recipe and no workflow. `just test-all` does not
close the hole — it excludes `group(=ros2-interop)` outright, correctly, and
nothing picks the group up elsewhere.

`matrix_fixture_coverage.rs` G1 is the gate for exactly this and its doc
comment claims exactly this ("A Runtime cell nothing runs … fails here"). What
it asserts is `tests_dir.join(format!("{}.rs", c.test)).is_file()` — that the
FILE exists. Five gates surround a cell; none asks whether anything invokes it.
The repo's recurring shape, one level up from where phase-393 named it: *a
cell's existence reads as a lane*.

**And this host has no ROS.** `/opt/ros` does not exist, `ros2` is not on PATH,
`ROS_DISTRO` is empty. ROS 2 Humble lives in the `ros2` distrobox, and issue
0759's rule stands: a box in play means every job runs in the box on its own
tree. The mirror at `/mnt/wd/data/projects/nano-ros-box` is 323 commits behind
`main`. So "wire the tests into a lane" is not a one-line change; it is a
box-resident lane, which is W1 below.

## Coverage map — what a live peer has actually seen

Derived from the two reachable binaries' cases, not from the cell list.

**Proven live** (a stock `rmw_zenoh_cpp` / `rmw_cyclonedds_cpp` / Fast-DDS-via-
XRCE-Agent peer has exchanged with us):

| family | slots | evidence |
| --- | --- | --- |
| session lifecycle | `create_session`, `destroy_session`, `ping_session` | every case |
| publish / take | `create_publisher`, `publish`, `create_subscription`, `take`, `has_data` | zenoh + cyclone + xrce, both directions, plus an unmodified stock `demo_nodes_cpp` talker |
| service / client | `create_service`, `send_response`, `take_request`, `has_request`, `create_client`, `send_request`, `take_response` | zenoh both roles, cyclone nano-server only, xrce both |
| lifecycle | (executor layer, not a slot) | the full REP-2002 cycle via `ros2 lifecycle` |
| wake / IO | `set_wake_callback`, `drive_io`, `next_deadline_ms` | implicitly, by every case that delivers |

**Never seen a peer.** Each of these is `produced` and mutation-tested against
our own code:

| family | slots | why it matters |
| --- | --- | --- |
| **graph** | 12 (`get_node_names`, `get_topic_names_and_types`, `get_service_names_and_types`, the four `*_by_node`, `get_publishers_info_by_topic`, `get_subscriptions_info_by_topic`, `count_publishers`, `count_subscribers`) | `graph_interop.rs` is written and has never executed. Cyclone's reader (phase-381 W5) has never run against a live participant. This is the family that already shipped broken once. |
| **matched counts** | `publisher_count_matched_subscriptions`, `subscription_count_matched_publishers` | landed phase-393 W2. The number is only meaningful about a peer, so a self-test cannot check it at all. |
| **GID** | `get_gid_for_publisher` | landed phase-393 W2. A GID's whole purpose is that another participant recognises it. |
| **actual QoS read-back** | 6 (`publisher_get_actual_qos`, `subscription_get_actual_qos`, the four service/client halves) | issue 0823 found these asserting the REQUESTED profile as GRANTED. The fix is unproven against a peer that negotiates. |
| **events** | `publisher_event_init`, `publisher_take_event`, `subscription_event_init`, `subscription_take_event` | QoS-incompatible, liveliness-lost, deadline-missed. These fire *only* when a peer disagrees with us, so a self-contained test is structurally incapable of raising one. |
| **liveliness** | `publisher_assert_liveliness` | same. |
| **serialized / raw path** | `get_serialization_format`, `take_sequence`, `process_raw_in_place`, `subscription_supports_in_place`, `required_rx_bytes`, `publish_streamed` | `get_serialization_format` is what a peer's rmw reads to decide CDR compatibility. |
| **loans** | `borrow_loaned_message`, `publish_loaned_message`, `take_loaned_message`, `return_loaned_message_from_{publisher,subscription}` | issue 0814 — the surface is behind `feature = "lending"`. `loan_e2e` / `zero_copy` / `borrowed_e2e` are host-only. |
| **logging** | `set_log_severity` | no live test. |
| **visitors** | `rmw_node_visit_fn`, `rmw_names_and_types_visit_fn`, `rmw_topic_endpoint_info_visit_fn` | reachable only through graph, so they inherit graph's status exactly. |

**Actions have no interop cell at all.** `ros2_action_e2e.rs` exists, faces a
live ROS 2, calls no `interop::assert_test_bound`, and is named by no recipe.
Issue 0902 reports action goals completing between 20 % and 90 % of the time on
the same build — the shape of a defect only a peer can show.

## Three axes with no live coverage

* **Language.** Sixteen of the eighteen cells declare `Lang::Rust`. One
  declares C++ (`cpp_multi_node_entry`, unreachable). **None declares C — and
  the declaration is wrong**: `interop_e2e::scenario_coord` returns
  `Lang::Rust` unconditionally, while all three cyclone cases spawn C example
  binaries through `nano_cyclone_c_binary` (`c_talker`, `c_listener`,
  `c_service_server`). The per-case coordinate tripwire cannot catch this,
  because it reads the same constant the cell was written from. So the true
  state is the inverse of the declared one: C/cyclone is proven live and
  Rust/cyclone is not, and the matrix says the opposite. Fixing the constant
  will make G-gates fail; that failure is the finding.
* **Platform.** One on-target cell exists (`zephyr-qos-rust-zenoh`) and no
  recipe runs it. FreeRTOS, NuttX, ThreadX and esp32 have **zero** live-peer
  cells. Every claim that an RTOS image interoperates rests on host testing plus
  the wire being the same, which is an argument, not a measurement.
* **Direction, for cyclone services.** zenoh has both `nano_server` and
  `ros2_server`; cyclone has only `nano_server`. A nano-ros *client* against a
  stock cyclone service is untested.

## Recommended jobs

Ordered so that each one's output is usable before the next starts. The
principle throughout, from CLAUDE.md: **a uniformly-red lane has no signal
capacity** — do not wire tests into a lane before knowing which pass.

### W1 — one cell, end to end, in the box

Refresh the box mirror (`scripts/dev/ros2-box-sync.sh`, currently 323 commits
behind) and run **`graph_interop` alone**, by hand, inside the box. Nothing
else. Output is a verdict on the single most valuable unproven family, and a
written record of what it took to get a live-peer test to run at all — which is
the input every later job needs.

Also resolve the `nano-ros-box-box` mirror-of-a-mirror at that path: confirm it
holds nothing unique and remove it.

*Acceptance:* `graph_interop` produces a pass or a real failure (not a skip),
and the box procedure is written down in `docs/development/`.

### W2 — run the other nine unreachable cells once each, by hand

`qos_override_e2e`, `params`, `rust_multi_node_per_node_graph`,
`cpp_multi_node_entry`, `qos_zephyr_ros2_interop_e2e`, the two declarative
bridges, the imperative bridge, and the Cyclone half of `graph_interop`.

Record a verdict per cell in a table in this doc. **A failing cell is a
finding, not a blocker** — file it and move to the next. Expect several: none
of these has run since it was written, and the fixture-mtime rules mean a red
must be re-checked against artifact timestamps before it is believed
(issues 0859–0862, four retracted ghost issues from exactly this mistake).

*Acceptance:* every Runtime cell has a dated verdict; every failure has an
issue.

### W3 — the language-axis correction

Fix `scenario_coord`'s unconditional `Lang::Rust`, let the G-gates fail, and
follow the failures. Then decide, per workload, whether the missing language
gets a cell or a recorded carve-out. This is cheap and it changes what the
matrix *claims*, which is the point.

*Acceptance:* the declared coordinate of every interop case matches the binary
it spawns; `matrix_fixture_coverage` green again.

### W4 — the gate G1's doc comment already promises

A check that every `Runtime` cell's test binary is named by at least one `just`
recipe or workflow. It is a grep over `just/` and `.github/workflows/` against
`interop::CELLS`, and it is the structural fix for issue 1127 — without it,
W1–W3's work decays the moment someone adds a cell.

*Acceptance:* the gate exists, is on a lane, and fails when a cell's test is
unreferenced.

### W5 — a lane, over the cells that passed

Box-resident, scheduled (not on the PR path — it needs ROS, a router and a
serialised daemon). Membership is exactly the cells W2 recorded as passing.
Reds then mean something.

*Acceptance:* the lane runs on a schedule and is green on its first run.

### W6 — the families a peer has never touched, in value order

Each is a new cell plus a test, and each needs a peer that disagrees with us,
which is the part that does not exist yet:

1. **actions** — bind `ros2_action_e2e` to a cell; it is the only family with
   a known flake (0902) and no live coverage.
2. **events** — a stock peer declaring an incompatible QoS, so
   `subscription_take_event` has something to report. Nothing today can raise
   one of these.
3. **matched counts and GID** — a peer appearing and disappearing while we
   read the count.
4. **actual QoS** — read our advertised profile from the peer's side and
   compare it to what `*_get_actual_qos` reports, which is issue 0823's
   acceptance and was never run.
5. **`get_serialization_format`** — cheap, and it gates CDR compatibility.

### W7 — one on-target cell that is not QoS

An RTOS image against a stock peer, on a platform that is not Zephyr. This is
the largest job here and the one with the weakest current argument, so it goes
last and may reasonably become its own phase.

## What this phase does NOT promise

Not the inert slots. Six slots are declared reservations with recorded reasons
(`INERT_FAMILIES`), and verifying a reservation is a category error — if one
should work, that is a contract job for phase-393's successor, not this one.

Not the declined twenty. Each carries an RTOS reason; a live peer does not
change a decision.

Not "the RMW works". At the end of every job above, the honest claim is still
per-family and per-coordinate. The number that matters is how many of the 58
`produced` slots a peer has touched, and today it is roughly twenty.
