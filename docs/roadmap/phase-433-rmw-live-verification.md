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

## The blocker, measured twice and wrong twice before this (issues 1136, 1127)

`interop::CELLS` is 18 rows, 17 `Runtime`, across 11 test binaries. They are
**wired into a lane**: root `just test-all` (`justfile:2172`) is `cargo nextest
--workspace` with no exclude filter, `just ci tier1` includes it, and
`host-tests.yml:266` runs `just ci tier1` on every push to `main`. That lane
**has ROS** — both jobs run in `container:
ghcr.io/newslabntu/nano-ros-ci:humble`, which is why grepping the workflow for
`ros-humble` or `setup-ros` finds nothing.

**And it never reaches the tests.** Last 30 runs, measured 2026-09-06: **0
success, 10 failure, 18 cancelled.** The last five failures die at the same
step, `Build workspace fixtures`:

```
CMake Error at cmake/NanoRosEntry.cmake:426 (add_executable):
  Cannot find source file:

    LAUNCH_ARGS
```

Phase-405 W1 removed `LAUNCH_ARGS` from `nano_ros_entry`'s parse list; the CLI
still emits it (`cmake_root.rs:347`, with a unit test asserting the emission).
That is **issue 1136**, it is high severity, and it is the reason nothing in
this phase can be observed in CI today. **It is now W0.**

The removal's own comment names the mistake it made: the keywords had "zero
authored users in the tree (*generated CMakeLists excluded, since those are
tool output rather than a caller's choice*)". The generator is the only caller
that runs in CI, so the sweep was scoped past the users that existed.

**The second failure outlives 1136's fix**, and it is issue 1127's: when the
lane does reach the cells, each reports a pass, a failure or a `skip!`, and
**nothing distinguishes a cell that has skipped on every host since it was
written from one that is covered.** Same absorbing-verdict class as issue 0445.
Five gates surround an interop cell (G1–G5) and not one asks whether it has
ever produced a result — they are all statements about declarations.

**Third: no focused runner.** `--workspace` reaches a binary but cannot run
*one cell against a live peer*, which is what verifying any of this requires.
Three of 11 binaries have a focused recipe (`interop_e2e`, `params`,
`xrce_ros2_interop`); eight have none. W4 gates that.

**And this host has no ROS**, so hand-verification is box-resident (issue
0759). Verified 2026-09-06: the `ros2` box is fully provisioned — 290
`ros-humble-*` packages, `rmw_zenoh_cpp` 0.1.9, `rmw_cyclonedds_cpp` 1.3.4,
`rmw_fastrtps_cpp` 6.2.10, box-owned `just`/`cargo-nextest`/`nros`/`bindgen`
under `~/.local-box/bin`. **Nothing about the environment is missing.** Driving
it correctly is the part with traps — see W1 below and issues 1144, 0759.

### W0 LANDED — the host-tests job could not build its fixtures (issue 1136)

Nothing in this phase is observable while `host-tests.yml`'s `nros-tests
integration (host)` job dies at **Build workspace fixtures**: measured
2026-09-06 over its last 30 runs, 0 success / 10 failure / 18 cancelled, the
last five failures all at that step.

phase-405 W1 removed `LAUNCH_ARGS` from `nano_ros_entry`'s
`cmake_parse_arguments` on a survey of "authored users in the tree (**generated
CMakeLists excluded, since those are tool output rather than a caller's
choice**)". The generator is the only caller that runs in CI, and
`builder/cmake_root.rs` still emitted it — so the keyword fell into
`UNPARSED_ARGUMENTS`, was spliced into `SOURCES`, and every workspace configure
died on `Cannot find source file: LAUNCH_ARGS`. Six days, unseen, because
host-tests runs on no gating event.

**The capability was LIVE, not dead — measured, because the phase-405 note
suggested otherwise.** Two frames, both checked:

* `nros model-path --launch multihost.launch.xml` → `config/multihost_model.yaml`;
  with `--arg host=robot1` / `robot2` → `config/multihost_robot1_model.yaml` /
  `..._robot2_model.yaml`. Three distinct models; `--arg` was never removed.
* The cmake side reached it by ACCIDENT and still worked. Replaying both
  parses (`cmake -P`) on a generated call: pre-W1 the verb left
  `LAUNCH_ARGS;host=robot1` in `UNPARSED_ARGUMENTS`, `_srcs` carried it into
  `SOURCES`, and `nano_ros_entry` re-tokenized it back into
  `_NRA_LAUNCH_ARGS=[host=robot1]` — which the `--arg` loop forwarded.
  Post-W1 the same call yields `SOURCES=[LAUNCH_ARGS;host=robot1]`.

So the per-host images were never silently identical: the loss was a hard
configure error, which is the good failure. What was silent is the sibling
found on the way — the verb kept forwarding `MODEL` after phase-405 W4 stopped
parsing it, so a `MODEL <path>` reached `UNPARSED_ARGUMENTS` and was DROPPED.
Zero callers passed it, which is why nobody saw it.

Landed: `LAUNCH_ARGS` restored in `nano_ros_entry` with its `--arg` forward;
`LAUNCH_ARGS` and `PANIC` parsed and forwarded EXPLICITLY by
`nano_ros_add_executable` instead of riding the `UNPARSED_ARGUMENTS`
re-tokenization phase-405 W1 itself named as a hazard; `MODEL` now a
`FATAL_ERROR` there. Gate `check-generated-cmake-keywords` (fast lane) compares
what `cmake_root.rs` emits against every cmake frame the call traverses — the
complement of `check-retired-cmake-keywords`, which scans tracked cmake and
therefore cannot see a generated root. Tests:
`model_location::per_host_images_resolve_to_distinct_models` and
`cmake_root::two_images_differing_only_in_args_render_differently`.

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
| **graph (zenoh only)** | 12 | **`graph_interop` PASS, 2026-09-06** — a nano-ros node enumerates a stock `rmw_zenoh_cpp` peer |
| wake / IO | `set_wake_callback`, `drive_io`, `next_deadline_ms` | implicitly, by every case that delivers |

**Never seen a peer.** Each of these is `produced` and mutation-tested against
our own code:

| family | slots | why it matters |
| --- | --- | --- |
| ~~**graph**~~ | 12 | **MEASURED 2026-09-06 (W1).** zenoh PASSES live — moved to the proven table. Cyclone FAILS: one node (itself), `Transport(Unsupported)` on topics, nine of twelve slots Unsupported. Issue 1137. |
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

### W1 — one cell, end to end, in the box — **DONE 2026-09-06**

`graph_interop` has now run against a live ROS 2 Humble peer. First time since
it was written.

```
Starting 2 tests across 1 binary
    PASS [  18.762s] (1/2) graph_interop nano_ros_enumerates_a_stock_ros2_node
    FAIL [  37.468s] (2/2) graph_interop cyclone_enumerates_a_stock_ros2_node
```

**zenoh PASSES.** The twelve graph slots phase-381 shipped and issue 0903
repaired work against a stock `rmw_zenoh_cpp` peer. Ten of those twelve were
listed in phase-393 as "still unproven live"; they are proven now.

**Cyclone FAILS**, and the cell's own comment had predicted exactly this
("has never been run against a live participant"). `get_node_names` returns one
entry — the probe itself — while a stock talker is on the domain;
`get_topic_names_and_types` errors `Transport(Unsupported)`; **nine of twelve
slots answer Unsupported.** Filed as **issue 1137**.

The peer was ruled out first, per issues 0859–0862: a stock Cyclone talker on
domain 77 in the same container is visible to `ros2 node list --no-daemon` and
`ros2 topic list`. Discovery works; our reader does not use it.

**What the run cost, and the procedure it produced.** The environment needed no
provisioning at all — the blocker was a poll loop from an 18-day-old session
holding the podman runtime lock, and one hung `podman exec` behind it. Once
cleared:

1. `bash scripts/dev/ros2-box-sync.sh` — takes over an hour on this disk; do not
   wrap it in a timeout. It now excludes `/.claude/worktrees/`, which it was
   copying once per live agent session.
2. **Enter as the user.** `podman exec` without `-u aeon` runs as ROOT, and
   git then refuses the box tree as dubiously-owned. `git ls-files` fails,
   `nros_source_manifest` cannot compute a workspace-fixture signature, and the
   test emits `[SKIPPED] … fixture not built` — which nextest renders as
   **FAIL**, because `skip!` panics and only `test-all`'s junit rewrite converts
   it. Five cells reported red for a git-config reason, naming the fixture
   rather than the ownership. It also stamps the box CLI inconsistently, so the
   next run as `aeon` refuses everything with "in-tree nros CLI is STALE".
   `distrobox enter` does not have this problem; `podman exec` is the shortcut
   that does.
3. **PATH: assert, do not assume** (issue 1144). In a NON-login shell
   `~/.cargo/bin` is absent, so `activate.sh`'s conditional prepend fires and
   lands the host's glibc-2.39 `just` ahead of the box's. Sourcing
   `activate.sh` before `ros2-box-env.sh` makes that guard a no-op and works,
   but it is a workaround — box-env sources `activate.sh` itself, and the
   ordering belongs there. **FIXED** (issue 1144, now archived): box-env seeds
   `$HOME/.cargo/bin` at the tail before sourcing, so the guard is a no-op in
   both shell kinds, and then asserts the outcome (`nros_box_check_path` — no
   box-installed tool may resolve into `$HOME/.cargo/bin`). Join the source to
   the command with `&&`, or a refusal does not stop the job.
4. `just setup-cli`, then `bash scripts/build/fixtures-build.sh linux rust
   <rmw>` — the positional filter narrows to a coordinate without needing a
   lane. 54 rows for zenoh, 11 for cyclonedds; roughly an hour together.
   **`[[fixture]]` rows are not workspace fixtures**: cells whose nano side is a
   workspace entry (the bridges, `qos_override_e2e`,
   `rust_multi_node_per_node_graph`) additionally need `just native
   build-workspace-fixtures`, a different builder.
5. `cargo nextest run -p nros-tests --test graph_interop`.

**Not done:** the `nano-ros-box-box` mirror-of-a-mirror still sits beside the
box tree, and this procedure is recorded here rather than in
`docs/development/`.

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

### W5.a — the durable half: a verdict is RECORDED, or it never happened

Issue 1127's Direction item 5, and the only one of the five that stops the class
recurring. W1–W4 make a cell *runnable and aimable*; none of them makes the
difference between "ran and passed" and "skipped on every host since the day it
was written" visible to anybody.

**What is being closed.** A `skip!` is not a verdict, and after
`_rewrite-skipped-junit` it is a `<skipped>` that every consumer counts as
not-a-failure. So a Runtime interop cell that has never met a peer looks
exactly like one that passes, on every lane, forever. Five gates surround such
a cell (`matrix_fixture_coverage.rs` G1–G5) plus W4's runner gate, and all six
are statements about DECLARATIONS. That is issue 0445's absorbing-verdict class
one level up: there a STALE fixture replaces the runtime's answer with a
message explaining itself; here a green tick does the same job. W1 measured the
cost — `graph_interop` had been "green" since it was written, and the first
time it was aimed at a live peer, zenoh passed and Cyclone failed nine of
twelve slots (issue 1137). The cell's own comment had predicted it.

**The mechanism.** `.config/interop-verdicts.toml` records, per cell, that a
real run produced a real result: the date, the case names, pass or fail, the
command, where it ran, and the observed output. **Absence of an entry means
NEVER RUN, and that is the default** — a new cell needs no ledger line and
starts life honestly unproven, which is the property a mechanism for this must
have. `scripts/check-interop-verdicts.py` is three things over that one file:

* `--check` — the gate (`just check interop-verdict-ledger`, fast lane).
  Integrity only, so it is green on a laptop with no ROS: every entry names a
  real `Tier::Runtime` cell in `interop::CELLS`, carries every required field,
  names test functions that still EXIST in that cell's declared test binary,
  and — the load-bearing rule — **names at least one case that is not a
  binding-only test.** `cases_bound_to_interop_cells` and its siblings call
  `interop::assert_test_bound` and nothing else; they pass on any host, with no
  peer, forever. Letting one of those count as evidence would re-create the
  exact defect inside the mechanism built to catch it. A `fail` verdict must
  name an OPEN issue (1137 does).
* `--report` — `just interop-verdicts`. Every Runtime cell, one line each:
  `PASS`/`FAIL` with its date and age, or `NEVER` with the focused runner W4's
  gate proved exists. This is the human-facing half.
* `--record CELL --junit F` — writes an entry FROM a run. It refuses when every
  case of that cell's binary in that junit skipped: *"no verdict — that is the
  absence this ledger exists to make visible."* So the honest path is
  mechanical, and hand-authoring an entry is a visibly different act.

And one line at the tail of every sweep (`_test-summary`, so every `test`,
`test-all` and tier): `--after-run` reports how many cells have ever produced a
verdict, how many live-peer binaries this run reached, and how many of them
skipped wholesale. It stays silent when the run contained no interop binary,
and it NAMES a cell whose binary produced a result the ledger has not recorded
— which is the ratchet tightening itself.

**Seeded with the two verdicts that exist.** W1's run is in the ledger:
`native-graph-rust-zenoh-r2n` PASS, `native-graph-rust-cyclone-r2n` FAIL
(issue 1137). 2 of 17. W2 fills in the rest by running the cells; the ledger
is where its "record a verdict per cell" lands, instead of a prose table in
this document that nothing can check.

**What was rejected, and why.**

* **A generated per-cell last-verdict file, updated by CI.** This is the
  0883/0884 shape exactly: a committed generated file that many PRs touch
  serialises the merge queue, and a merge driver cannot fix it because GitHub
  rebases queue entries server-side. The ledger dodges it by holding only
  POSITIVE claims, written by a human at most once per cell — 17 edits, ever,
  not one per push.
* **Extending `check-skip-budget` (issue 0584).** It is the right neighbour and
  the wrong shape. Its two assertions are deliberately DERIVED from a single
  run, with no declaration file, so it cannot express "has never happened on
  any host" — a property of all history everywhere. 0584 itself named the gap
  it could not close: *"a `capability` skip on a machine that HAS the
  capability … is a bug that is currently invisible."* Closing that needs to
  know what the host has, which the junit does not say.
* **Deriving the streak from junit alone.** Attractive — CI already produces
  the XML — and unsound at the granularity that matters. Junit gives (binary,
  case), and **every one of the eleven live-peer binaries also carries a
  binding test that passes with no peer**, so "did this binary produce a
  non-skip result" is TRUE for all eleven on a host with no ROS. A mechanism
  that reads that as coverage is the bug wearing the fix's clothes. Per-CELL
  attribution needs someone to say which case exercises which cell, and that
  is what an entry is. (`--record` and `--after-run` do consume junit, with
  the binding-only cases excluded by the rule above.)
* **A count ratchet ("no more than N unverified cells").** One integer in a
  shared file, edited by every PR that adds a cell — 0883/0884 again, for one
  line of value. Absence-means-never already makes a new cell unproven without
  anyone editing anything.
* **A gate that fails while a cell is unverified.** Uniformly red on every
  contributor's host, since most have no ROS; a lane with no signal capacity
  teaches people to ignore it (CLAUDE.md). Visibility is the lever here, not
  refusal.
* **A hard expiry on a verified entry**, the way `.config/flake-quarantine.toml`
  expires a quarantine. Right there, wrong here: quarantine holds a handful of
  deliberate suppressions, this holds up to 17 cells that only a ROS-carrying
  lane can refresh, so expiry would go red for everyone and stay red. The age
  is REPORTED instead, in days, beside every entry.

**What this does not do.** It cannot verify that a hand-written entry is true —
only that it is well-formed, names a real cell, and cites cases that exist and
are not binding-only. It cannot see an entry DELETED either — that is a
coverage regression only review catches, and a ratchet over the count would be
the shared committed line this design exists to avoid. Nor does it know when a
recorded verdict goes stale: a
cell verified today can break tomorrow, and only W5's scheduled lane will say
so. The ledger's job is to stop *absence* reading as *success*; keeping a
verdict fresh is what a lane is for.

*Acceptance:* the ledger and gate exist, the gate is on the fast lane and fails
on a stale entry, on a binding-only case cited as evidence and on an
unattributed `fail`; the report distinguishes a recorded verdict from a cell
that has never produced one; `--record` refuses a junit in which the cell's
binary only skipped.

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
