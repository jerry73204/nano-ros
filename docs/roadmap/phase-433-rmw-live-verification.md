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

## The blocker: the live-peer tests run, skip, and read as green (issue 1127)

`nros_tests::interop::CELLS` is the intent list for live-peer work — 18 rows,
17 `Runtime`, across 11 test binaries.

**They are invoked on every push to `main`.** The root `just test-all`
(`justfile:2172`) runs `cargo nextest --workspace` with **no exclude filter**;
`just ci tier1` is preconditions + check + `rust-rtos-link-check` + `test-all`;
`host-tests.yml` runs `just ci tier1` on push to `main`.

**And every one of them skips.** That runner has no ROS — the workflow installs
none, and its own `on:` comment says "This lane needs ROS on the runner and its
two jobs failed on every pull request". So each cell resolves to
`nros_tests::skip!`, the junit rewrite converts it to a skip, and the lane is
green.

**A skip is not a verdict, and nothing tells one apart from a pass.** No
mechanism distinguishes "skipped on every host since it was written" from
"covered". Same absorbing-verdict class as issue 0445, where a STALE fixture
replaces whatever the runtime would have done with a message explaining itself,
and issue 0444 hid behind it for exactly that long.

`matrix_fixture_coverage.rs` G1 is the gate closest to this, and its doc
comment claims exactly this ("A Runtime cell nothing runs … fails here"). What
it asserts is `tests_dir.join(format!("{}.rs", c.test)).is_file()` — the FILE
exists. Five gates surround a cell (G1–G5) and **not one asks whether the cell
has ever produced a result.** They are all statements about declarations.

**A second, smaller problem: no focused runner.** `--workspace` reaches a
binary but cannot run *one cell against a live peer*, which is what verifying
any of this requires. Three of the 11 binaries have a focused recipe —
`interop_e2e` (`just native test-ros2`, `just native test-ros2-lifecycle`),
`params` (`just native test-ros2-params`), `xrce_ros2_interop` (`just xrce
test-ros2`). The other eight have none. (`just native test-all` aggregates the
three and is itself called by nothing. `just native test` is a different lane
and does exclude the ros2 groups — correctly; misreading that exclusion as the
root sweep's produced the first version of this analysis.)

**And this host has no ROS either**, so the work is box-resident by
construction (issue 0759: a box in play means every job in the box, on its own
tree). Verified 2026-09-06: the `ros2` box is fully provisioned — 290
`ros-humble-*` packages, `rmw_zenoh_cpp` 0.1.9, `rmw_cyclonedds_cpp` 1.3.4,
`rmw_fastrtps_cpp` 6.2.10, and box-owned `just` / `cargo-nextest` / `nros` /
`bindgen` under `~/.local-box/bin`. **Nothing about the environment is
missing.** The mirror at `/mnt/wd/data/projects/nano-ros-box` was 323 commits
behind and is being refreshed; a stray `nano-ros-box-box` beside it is debris
from a sync started inside the box tree.

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

* **Language.** Was the inverse of what the matrix declared; **corrected in
  W3 (PR #587)**. C/cyclone is what runs; Rust/cyclone is carved. What remains
  after the correction: C++ has exactly one cell (`cpp_multi_node_entry`) and
  it has no focused runner.
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

### W2 — run every remaining cell once, by hand

`qos_override_e2e`, `rust_multi_node_per_node_graph`, `cpp_multi_node_entry`,
`qos_zephyr_ros2_interop_e2e`, the two declarative bridges, the imperative
bridge, and the Cyclone half of `graph_interop`. Then the three that a recipe
does reach but no sweep does — `interop_e2e`, `params`, `xrce_ros2_interop` —
since "a recipe exists" is not a verdict either.

Record a verdict per cell in a table in this doc. **A failing cell is a
finding, not a blocker** — file it and move to the next. Expect several: none
of these has run since it was written, and the fixture-mtime rules mean a red
must be re-checked against artifact timestamps before it is believed
(issues 0859–0862, four retracted ghost issues from exactly this mistake).

*Acceptance:* every Runtime cell has a dated verdict; every failure has an
issue.

**The pre-flight is done** (PR #592, which adds the W2 pre-flight doc to this
series) and it changes the order. All 11 binaries compile. Ten of eleven are expected
to produce a real verdict given a Humble box; `graph_interop` has the strongest
assertions in the set and should go first. Two exceptions:

* **`params` cannot produce a verdict at all** — four of its tests guard with a
  bare `return`, which is a PASS. Issue **1135**. Fix that before running it,
  or its green means nothing.
* **`qos_zephyr_ros2_interop_e2e` is blocked** — the only `ZephyrWestLeaves`
  cell, needing a second build channel. Schedule it last or split it out.

Fixture builds W2 needs: `just build-test-fixtures lane=native` covers 10 of 11
— and it must be that recipe rather than `just native build-fixtures`, because
only the former also runs `build-compile-check-fixtures`, which produces
`cpp_multi_node_entry`'s cmake fixture. Plus `just zephyr build-fixtures` for
the eleventh, and `just cyclonedds setup` + `just xrce setup` in the box.

**Box trap worth stating once:** source `./activate.sh`, not only
`ros2-box-env.sh`. The router is resolved through `NROS_RMW_ZENOHD` →
`AMENT_PREFIX_PATH` → `$ROS_DISTRO`, and box-env sets neither of the last two,
so nine binaries would skip on "zenohd not found" with a perfectly good router
sitting in `/opt/ros/humble`.

### W3 — the language-axis correction — **DONE 2026-09-06 (PR #587)**

`scenario_coord` derives the language now. Only the cyclone half was wrong:
all three cyclone cases spawn C binaries, every zenoh case genuinely is Rust
(verified by reading what each spawns, including the stock-`demo_nodes_cpp`
case, where the C++ is the *peer* and no coordinate names it). The cells are
`native-pubsub-c-cyclone-n2r` and `native-service-c-cyclone-r2n`.

**The prediction in this doc was wrong and the correction is the useful part.**
It said "correcting it will make the G-gates fail, which is the finding". No
gate failed. G5 was satisfied without inventing anything — `examples/
fixtures.toml` already produces `linux|c|cyclonedds` rows for exactly the
binaries the cases spawn. So the declaration was wrong *in isolation*: nothing
downstream had ever depended on the value, which is its own finding about how
much the coordinate was load-bearing.

The vacated Rust/Cyclonedds coordinate is CARVED for both workloads rather than
deleted, because `examples/native/rust/*` do carry `linux|rust|cyclonedds`
fixture rows — the gap is a missing lane, not a missing build, and deleting the
rows would leave the fixture list as the only evidence. Note for whoever takes
that lane: Rust↔Cyclone against a live peer is not wholly unproven —
`native-graph-rust-cyclone-r2n` is that pairing. It is *delivery* over
Rust/Cyclone that has never run.

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
