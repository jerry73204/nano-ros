# Phase 428 — sweep the C, C++ and Rust APIs against the porting principle

**Status (2026-09-05). RMW sweep DONE; findings below. User-API sweep planned.** Audits all three user APIs against RFC-0089's
governing principle. Not a rename pass — a conformance review that produces
findings, each of which becomes a work item somewhere else.

## The question being asked

For every public item in `nros-c`, `nros-cpp` and the `nros` Rust facade:

1. **Does an upstream counterpart exist** in rclc / rclcpp / rclrs? (The
   recorded surfaces already answer this — `docs/reference/api-surface/`.)
2. **If yes, do we use its name?** If not, why not — a constraint (clause 1), or
   a preference? RFC-0036 forbids recording a preference as a divergence, so a
   preference is a finding.
3. **If we use its name, is the difference MECHANICAL?** Does the compiler point
   at every difference, and is the fix local? A changed signature the compiler
   does not force — a widened return type at a discarding call site, behaviour
   behind an identical signature — is a finding.
4. **If no counterpart exists, is the invention necessary,** and does it carry a
   ledger row plus collision-gate coverage?

## Why it is worth doing as a sweep

The three known instances of question 3 were each found by accident, not by
looking: `rclcpp::init` widening `void` to `Result` at a discarding call site;
`ParametersQoS()` returning `QoS(10)` where upstream is `KEEP_LAST, 1000`; a
`NodeOptions` setter that stores its argument and is never read. All three
correlate `same` in the parity report, because correlation compares names and
shapes and neither differs. **The instrument cannot see this class**, which is
what makes it a sweep rather than a gate.

## Work items

* **W1 [tooling] — enumerate the candidates.** Every row where our name matches
  upstream's AND the shapes correlate `same`. That set is where a silent
  behavioural difference can hide; it is also the set the parity report is
  least likely to flag.
* **W2 [c] / W3 [cpp] / W4 [rust]** — one pass per language over its
  candidates, answering the four questions. Findings, not fixes.
* **W5 [ledger]** — every finding gets a row or amends one, with a disposition.
* **W6 [gate]** — where a finding is mechanically checkable, add the check
  rather than the note. The `[[nodiscard]]`-on-`Result` item from phase-427 is
  the model: a class of "the compiler does not force the edit" becomes a lint.

## Scope notes

* The C API's shape follows rcl/rclc, the C++ follows rclcpp, Rust follows
  rclrs — settled, each language takes its own upstream.
* This sweep does NOT rename. A finding that says "we should use upstream's
  name here" becomes a work item in the phase that owns that surface.
* Expect the Rust facade to have the most findings and the fewest of them
  actionable: rclrs is the youngest of the three upstreams and moves fastest, so
  "no counterpart" there often means "not yet" rather than "never".

## Findings — the RMW sweep (2026-09-05)

Three parallel read-only reviews: ABI surface, runtime semantics, and the
instruments themselves. Load-bearing claims re-verified by hand before filing.
Filed so far: **#1087**, **#1088**, **#1092**.

### What is sound, stated because a clean result is also a finding

The drift CLAUDE.md records is **closed**, re-derived independently rather than
read off an exit code: three `librmw_*_cpp.so` export byte-identical 88-symbol
sets and `diff` against `docs/reference/rmw-implementation-contract.txt` is
empty; an independent brace-matching parser of `rmw_vtable.h` finds the same 68
slots the tool does; the map partitions the contract exactly (63 vtable / 20
declined / 3 layer / 2 global / 0 gap) with no orphan slot and no missing
symbol. `check_against_vtable` fires in both directions.

All four QoS policy enums, `rmw_gid_t`, `rmw_message_info_t` and
`rmw_network_flow_endpoint_t` are byte-identical to upstream including layout.
`rmw_ret_t` values match. `NROS_RMW_QOS_PROFILE_PARAMETERS` is `KEEP_LAST, 1000`
— the hundredfold inversion RFC-0089 is written about does **not** exist at this
layer.

### F1 — capability claimed by a name, absent behind it

The class RFC-0089 exists for, and a parity report correlates every one of these
`same`, because correlation compares names and shapes.

* **zenoh ignores the QoS REQUEST — which is not the same as implementing
  nothing, and the first draft of this finding got that wrong.** The grep is
  real: `grep -rn "qos\.depth\|qos\.reliability\|qos\.durability\|qos\.history"
  packages/rmw/zenoh/nros-rmw-zenoh/src` → **0 matches**. But it proves the
  request is never READ, not that the policy is absent. Corrected, per policy:

  | policy | delivered? |
  | --- | --- |
  | reliability | **yes, unconditionally RELIABLE.** `Z_RELIABILITY_DEFAULT = Z_RELIABILITY_RELIABLE` (`zenoh-pico/api/constants.h:203`); the shim never sets the field, so every put takes that default, over a TCP transport |
  | durability | **yes, VOLATILE** — and `TRANSIENT_LOCAL` is correctly refused |
  | history | `KEEP_LAST` only |
  | depth | **no** — build-time `SUBSCRIBER_RING_DEPTH` (default 4), overflow drops at `subscriber.rs:1538` |

  So a caller asking BEST_EFFORT gets RELIABLE: *stronger* than requested,
  which is the safe direction under RxO (a BEST_EFFORT reader accepts a
  RELIABLE writer). It costs bandwidth and latency, not correctness, and
  advertising `RELIABILITY` is defensible.

  **The real over-claim is `DEPTH`.** A caller asking depth 100 silently gets
  4, and under `KEEP_ALL` that is a violation rather than a downgrade. The code
  already knows: `session.rs:900-907` says the reported depth deliberately does
  not mirror upstream's 42 because *"advertising a depth we cannot honour is
  the lie issue 0829 is about"* — so `get_actual_qos` tells the truth while the
  SUPPORTED MASK still claims `DEPTH`. The honesty is in one place and not the
  other.
* **zenoh `create_service` / `create_client` discard the caller's QoS** —
  literally `let _ = qos;` (`session.rs:1005`, `:1041`, verified) — and announce
  a hardcoded `services_default()` to the graph.
* **XRCE silently drops deadline / lifespan / liveliness**
  (`xrce/src/session.c:199-235`), and all six `*_get_actual_qos` slots are NULL,
  so the downgrade report is skipped too.
* **`*_get_actual_qos` echoes the REQUEST** where upstream requires `UNKNOWN`
  (`cyclonedds/src/qos.cpp:136-139` vs `rmw.h:669-671`). Because the one
  consumer compares requested against granted, echoing makes an unreportable
  field read as *granted* — the inverse of upstream's meaning.
  **CORRECTED 2026-09-05 by W1:** an earlier draft said
  `NROS_RMW_RELIABILITY_UNKNOWN` "exists and is unused". False — it is written
  at `rmw/cffi/src/rust_adapter.rs:1797` on every Rust backend and read by
  `qos_has_unknown` (`cffi/src/lib.rs:4307`) to produce upstream's
  `RMW_QOS_COMPATIBILITY_WARNING`, with tests either side. The true statement
  is narrower and worse: **no `*_get_actual_qos` implementation anywhere in the
  tree writes any `*_UNKNOWN` value** — the slots whose own documentation
  defines the contract are exactly the ones that do not honour it. The
  constant with genuinely zero uses is `NROS_RMW_QOS_PROFILE_UNKNOWN`, added by
  W10, whose only occurrence is its own definition.

### F2 — a fix landed where the symptom was seen

* **#1087** — 1008 fixed `is_server_ready`, which HAS a vtable slot;
  `poll_server_discovery` defaults to `Ok(Some(true))` and has NO slot, so
  `CffiClient` inherits it and a Rust backend loses its override crossing the C
  ABI. Six wait paths read it. Both comments justifying it claim rclcpp latches
  its answer; `ClientBase::service_is_ready()` calls rcl on every invocation.
* **`QOS_PROFILE_PARAMETER_EVENTS` is `KeepAll, 0`** (`traits.rs:902`) where
  upstream is `KEEP_LAST, 1000` — with the comment "matches
  rmw_qos_profile_parameter_events" directly above it. 0793 fixed `PARAMETERS`
  and stopped. Not cosmetic: a RELIABLE writer with KEEP_ALL **blocks** when
  history fills where KEEP_LAST(1000) overwrites, and a test at `traits.rs:3284`
  locks it in.
* **`report_qos_downgrade` has one call site**, in `create_publisher`;
  `create_subscription` has none — and the helper's warning text describes the
  reader case it is never called for.

### F3 — data loss reported as success

* **#1088** — cyclone `take_request` destroys a request before checking for a
  free correlation slot, then reports `taken = false`. The XRCE sibling gets it
  right.
* **bridge dedup drops every repeat of an identical payload permanently**
  (`bridge/src/lib.rs:436`, `:471`) — recorded only on the forward path, nothing
  evicts, so absent other traffic exactly one message ever crosses.
* **cyclone `publish` blocks 2 s on a type-name suffix then returns OK having
  published nothing** (`publisher.cpp:276-281`).
* **zenoh graph enumeration truncates silently** — the C cache returns a
  `dropped` count the Rust caller discards outside a debug log, while our own
  header says "never a truncated success".

### F4 — the instruments

* **#1092** — only **15 of 68** slots have arguments exactly enforced; 39 sit in
  `ARG_DEVIATIONS`, whose 42 values are all reason strings against a membership
  test; 14 are never signature-compared. **7 of 24 mutations left every RMW gate
  green**, including an `ADDED` slot gutted to a `void` return and the same slot
  deleted outright — and a reintroduction of the `void`-return defect W5 was
  created to find.
* **`check-rmw-slot-producers` tests `if s in produced` first**, so a slot with
  a backend body counts as covered whether or not anything reads it. It reports
  6 inert; **17 of 68 are unreachable from any application**, cyclone
  implementing and unit-testing 11 of them.
* **`check-rmw-ret-sign`'s `STATUS_ONLY` names 7 slots that do not exist** and
  watches 23 of 65 status-returning slots.
* **The gap-exception mechanism is unguarded** — a reason naming a nonexistent
  issue, or a resolved one, passes. Exposure is zero today (0 gaps); the
  exemplar in the script's own docstring is resolved issue 0776.

### F5 — prose in the SSoT

`rmw_ret.h:12-27` states the retired sign contract as current and is wrong four
ways at once; two vtable slots carry a complete pre-W3.d doc block above their
current one, which bindgen concatenates into `generated.rs`; `rmw_vtable.h`
documents NULL-slot fallbacks that do not exist in code — the same fiction
corrected 60 lines above for `get_implementation_identifier`.

`rmw_vtable.h:66-75` names this class itself: *"Prose in the SSoT header is not
covered by any of the shape gates."* The correction was applied to one
paragraph.

### The generalisation

**Every mechanism this campaign added validates EXISTENCE and SHAPE. Nothing
validates a REASON.** Three authored tables have now drifted exactly as the
parity map once did — `ARG_DEVIATIONS`' reasons, `check-rmw-ret-sign`'s
`STATUS_ONLY`, and the header prose. That is the work item behind most of the
rest.

## Work items — REVISED after the sweep

W1–W6 stand for the user-facing C/C++/Rust APIs. The RMW layer adds:

* **W7 [rmw] — pin every deviation.** `ARG_DEVIATIONS`/`RET_DEVIATIONS` values
  carry the expected signature, not a reason alone; `ADDED` keys are asserted to
  be live slots with a pinned return type; grouped-only targets are compared
  against the symbol grouped onto them. *Acceptance:* all seven surviving
  mutations from #1092 fail.
* **W8 [rmw] — a slot is covered when something READS it.** Reorder
  `check-rmw-slot-producers` so a backend body does not satisfy the check, and
  decide per slot whether the 17 unreachable ones get a consumer or are
  withdrawn. *Acceptance:* the tool's inert count matches the hand count.
* **W9 [rmw] — the supported mask is DERIVED, not authored.** Today
  `supported_qos_policies()` is a hand-written constant, so a backend can
  declare a capability it does not have. Derive it from what the backend
  actually honours.

  Measured for zenoh, the derived mask is
  `RELIABILITY | DURABILITY_VOLATILE | HISTORY` — **`DEPTH` drops out**, and
  nothing that works today breaks, because dropping `DEPTH` refuses a request we
  were already silently downgrading. RELIABLE keeps working because we genuinely
  deliver it.

  *Acceptance:* a backend whose code never reads a policy cannot advertise it;
  `create_service`/`create_client`'s `let _ = qos;` either applies the profile
  or refuses loudly; a request for a depth the ring cannot hold is refused or
  reported, never silently clamped.
* **W10 [rmw] — one QoS table.** Upstream's 7 profiles are hand-transcribed in
  four places (26 constants) with no gate binding them; `PARAMETER_EVENTS` is
  the proof. *Acceptance:* one SSoT, generated or gated, and the
  `PARAMETER_EVENTS` test asserts upstream's values.
* **W11 [rmw] — resolve every issue reference.** Every `issue NNNN` in a gap
  reason and every authored `issue =` resolves to a file with `status: open`.
* **W12 [rmw] — the prose.** Delete the retired sign contract, the two stale
  slot doc blocks and the fictional NULL-slot fallbacks; then gate what can be
  gated (a doc naming a slot that does not exist is mechanically checkable).


## W10 outcome (2026-09-05) — one gate, and the sources it does not yet reach

Landed: the SSoT is a `qos_profiles!` fence in `nros-rmw/src/traits.rs` stating
all ten fields per row with the upstream symbol each mirrors;
`check-qos-profile-ssot.py` binds it to a recorded upstream table and
re-extracts that table from `qos_profiles.h` when a ROS install is resolvable.

**Two gates were written and one was deleted.** The C-header agent and the Rust
agent independently produced `check-qos-profile-table.py` and
`check-qos-profile-ssot.py`, both parsing the same fence — the duplication this
item exists to remove, reproduced in the fix for it. Kept the one with a
working negative control (63 live mutations across 7 presets x 9 fields, self-
test on the normal path); deleted the other rather than ship two parsers of one
table, one of which was green by accident after its `QoSProfile::build` parser
lost its target.

**Not yet bound, carried forward rather than claimed:**

* `packages/core/nros-rmw-abi/include/nros/rmw_entity.h` — corrected and
  restructured to one line per profile, but the gate does not read it. The
  deleted gate did; that coverage is the work item.
* `packages/rmw/cffi/src/lib.rs` — the FIFTH transcription site, which the
  original survey missed entirely. Corrected by hand.
* `packages/api/nros-cpp/include/nros/qos.hpp` — now one in-file table
  (`nros::detail::qos_table`) with all six `rclcpp::*QoS` classes delegating,
  so it is parseable; it cannot consume the C macros, measured: they are C99
  compound literals with designated initializers and the lane is `-std=c++14`.
* `packages/api/nros/src/lib.rs` — `nros::qos` now declares no values at all;
  all five are aliases, so the SSoT reaches it with no second edit.

**The field that moved.** Upstream's `rmw_qos_profile_default` and its four
siblings carry `RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT`; every one of our
five sites carried `AUTOMATIC`. Issue 0829 corrected `SYSTEM_DEFAULT` alone and
left the siblings — F2 for the third time in this sweep.

Rust has no `SystemDefault` variant: `QoSLivelinessPolicy::None` occupies
discriminant 0, which is the value the C ABI spells `SYSTEM_DEFAULT`. So the
NAME diverges and the VALUE does not, and only the value crosses. The
mirroring rows now carry `None`, with the divergence recorded at the enum.
Renaming the variant is a separate question.

Wire effect: none. Cyclone calls `dds_qset_liveliness` only for a non-sentinel
kind and its own default is AUTOMATIC; XRCE lowers no liveliness field; zenoh
emits empty liveliness and lease positions for every profile. A backend that
starts honouring liveliness later now sees "the middleware's choice" instead of
an unrequested AUTOMATIC.


## W13 [rmw] — discovery becomes a maintained SET, not a per-call query

The structural fix behind issue 1087. Two wrong answers preceded it, both mine,
so the reasoning is recorded and not just the conclusion.

### Why our shape diverged from upstream's

`rmw_service_server_is_available(node, client, bool *out)` answers immediately
upstream **because DDS maintains a discovery cache** — the middleware already
knows which endpoints are matched, and no query is issued at call time.

Ours answers with a QUERY: `z_liveliness_get`, which is start-then-poll. That
is why the `start_server_discovery` / `poll_server_discovery` pair exists, why
it is asynchronous, and why a synchronous `service_is_ready` had nothing to
return but a latch. **The shape mismatch is downstream of not keeping the state
upstream keeps.**

Both earlier proposals are wrong:

* *"Delete the pair in favour of `service_server_is_available`"* — written into
  issue 1087 by me. It removes the only mechanism in the tree that issues a
  real discovery query, leaving a synchronous method that can never answer
  `true` on the one backend implementing discovery honestly.
  `wait_for_service` would go from "wrong answer instantly" to "correct answer
  never".
* *"Give the pair two vtable slots"* — better, but it standardises OUR
  invention into the ABI. Clause 3 where clause 2 is available.

### The fix

Zenoh liveliness subscriptions deliver **PUT** (declared) and **DELETE**
(dropped). The shim already runs a session-side
`z_liveliness_declare_subscriber` — `shim/subscriber.rs:1203` consumes it for
`LivelinessChanged` — so the stream exists and is used for one purpose only.

Feed a **matched-queryable set** from that same subscription: declared →
insert, dropped → remove, `service_is_ready()` → synchronous set lookup.

| today | after |
| --- | --- |
| `server_seen` latch, never cleared | a set with removal; the DELETE sample IS the invalidation |
| async pair, no vtable slot, C-ABI backends inherit a default | **deleted** — nothing needs it |
| `service_is_ready` returns a stale bool or `Unsupported` | the current answer, synchronously |
| our invented pair (clause 3) | upstream's shape (clause 2) |

Deleting the pair is correct AFTER this and a regression before it. The blocker
was never the deletion; it was that nothing else could answer the question.

### Work

* **W13.a [zenoh] — DONE (2026-09-06).** The matched set IS the graph cache
  (phase-381 / issue 0903): the standing `z_liveliness_declare_subscriber` on
  `@ros2_lv/<domain>/**` with `history = true`, PUT inserts and DELETE removes.
  It was already the right structure; what it lacked was a reader on the
  service path and a start before the first client. `ZenohSession::create_client`
  now starts it (once per session, idempotent). The set mutation is the pure
  `zpico_graph_set_apply`, split out of the sample handler like
  `zpico_entry_at`, and `zpico-sys`'s `graph_set_put_then_delete_round_trips`
  drives PUT / duplicate PUT / DELETE / absent DELETE / overflow through the
  real C. Not a per-service arena: the domain-wide cache was resident (64 KiB,
  `ZPICO_GRAPH_CACHE_SIZE`) in every session already, so the static RAM delta
  is zero and a second, narrower subscriber would have been a second spelling.
  *Acceptance met at the set level* (a DELETE removes the entry, so the answer
  can go back to false); the stop-and-restart of a live server is not observed
  by any cell yet — the interop client is single-shot.
* **W13.b [zenoh] — DONE.** `ZenohServiceClient::service_is_ready` walks the
  cache through the ONE walk, `session::graph_cache_for_each` (shared with the
  ten graph slots' `for_each_entity`), and matches an `SS` token on
  `(mangled service name, DDS type name)` — the pair `rmw_zenoh_cpp` keys a
  service on (`liveliness_utils.cpp`, `Entity::Entity`, humble 0.1.9; the
  parser is now pinned to a REAL token of each shape in
  `parse_reads_real_rmw_zenoh_tokens`). `Ok(true)` / `Ok(false)` /
  `Err(Unsupported)` when the cache is not running OR has dropped tokens. There
  was no `platform-bare-metal` gate to decide — `Z_FEATURE_LIVELINESS 1` is
  unconditional in the generated zenoh config on every platform (the gate in
  `subscriber.rs` is on the publisher-side `LivelinessChanged` poll, not the
  subscriber); a platform whose C shim stubs the cache gets `Err` from the same
  `count < 0` check.
* **W13.c [cyclone, xrce]** — OPEN. Fill the `service_server_is_available`
  vtable slot. It EXISTS and both leave it NULL, which is why they answered
  without asking. Cyclone is cheap: DDS has the cache natively. Until it lands,
  `wait_for_service` on those backends waits out its budget and reports
  `false` — honest, and slower than the cache could answer.
* **W13.d [core] — DONE**, ahead of W13.c and deliberately: with a–b in, the
  pair had no backend left that could answer through it better than
  `service_is_ready` does, so keeping it for cyclone/XRCE would have kept a
  "cannot say" spelled two ways. Deleted from `ClientTrait`, the zenoh shim
  (`server_seen`, `discovery_handle`, `discovery_keyexpr`,
  `service_server_keyexpr_wildcard`), `ActionClientCore`, and all six wait
  loops, which are now one shape: spin, ask, `Ok(true)` returns, else wait to
  the deadline. `SERVER_DISCOVERY_PROBE_TIMEOUT_MS` went with it.
  *Acceptance:* `grep -rn "poll_server_discovery" packages/` returns nothing
  (met). "A cyclone client answers from the DDS cache" is W13.c's.

Also in this wave: the native Rust `service-client` example gates its first
request on a new `TickCtx::service_is_ready_for_name` (rclcpp's
`while (!wait_for_service(1s)) "service not available, waiting again..."`, one
check per timer tick, `Err` = call anyway), and the `ZenohServiceRos2Server`
interop cell starts the client BEFORE the ROS 2 server so it observes both
halves: the waiting line with no request sent, then the result once the `SS`
token lands. `services.rs`'s two no-server tests assert the waiting marker on
this binary. **Unrun** in the landing session (no fixture build); compiled.

Issue 1087 is resolved and archived by this wave; 1008 stays as filed.

### What already landed on `fix/1087-1088-server-availability`

Carried into the W13 PR as its first two commits (the branch had no PR): the
optimistic default is `Ok(None)`; the two comments claiming an rclcpp
"snapshot semantic" are corrected (there is none —
`ClientBase::service_is_ready()` calls rcl on every invocation); the zenoh latch
returns `Err(Unsupported)` instead of a permanent yes. W13 then deleted both
the default and the latch.

## W5 — the findings, recorded (2026-09-05)

W1–W4 produced ~40 confirmed findings that existed only in agent transcripts.
Recording them here first, because a finding nobody can find is the campaign's
own failure shape one level up: not a fix landing where the symptom was seen,
but a finding landing where nobody will look.

**Verification status is stated per row and it is not uniform.** Six were
re-verified by hand against upstream headers (marked ✔); the rest rest on the
audits' evidence, which was well-cited but is not the same as checked.

### Ranked — silently wrong (no compile error, no runtime error, wrong result)

| # | finding | file:line | lane |
| --- | --- | --- | --- |
| 1 ✔ | `LifecycleNode::trigger_transition(uint8_t)` — four of eight ids differ from `lifecycle_msgs`; `2` ACTIVATES where ROS 2 cleans up. Correct mapping exists in-tree, wire-path only. **Issue 1099** | `nros-cpp/lifecycle.hpp:173` | W3 |
| 2 ✔ | `poll_server_discovery` defaults `Ok(Some(true))`, no vtable slot, so the 1008 fix is neutralised three lines below itself. **Issue 1087** | `nros-rmw/traits.rs:3006` | W4 |
| 3 | `rclc_publisher_init_default` applies neither namespace nor remap; every other entity kind applies both, via a seam publishers never reach | `nros-c/publisher.rs:245` | W2 |
| 4 | `rclc_executor_trigger_one`'s `obj` changed from entity pointer to pointer-to-index; ported call compiles with zero warnings. A unit test asserts the divergence | `nros-c/executor.rs:1349` | W2 |
| 5 | `rclc_executor_spin_some` returns TIMEOUT on every idle tick; upstream explicitly discards it | `nros-c/executor.rs:2860` | W2 |
| 6 | `rclc_executor_spin_period` uses `period` as the wait timeout; `set_timeout` loses its only reader | `nros-c/executor.rs:2989` | W2 |
| 7 | `Publisher::publish()` widened `void`→`Result` — the hottest line in a ported node — and is the only method with no `initialized_` guard | `nros-cpp/publisher.hpp:95` | W3 |
| 8 | `Client::service_is_ready()` returns `Expected<bool>` whose `operator bool` is the PROBE-SUCCEEDED flag; the rclcpp `while (!ready)` idiom exits on the first successful "no" | `nros-cpp/client.hpp:222` | W3 |
| 9 | seven `create_*` verbs discard `Result` and return a non-null pointer to a dead entity; `spin` then returns silently on an uninitialised node | `nros-cpp/nros.hpp:607…896` | W3 |
| 10 | `PublisherOptions::qos` / `SubscriptionOptions::qos` written, read by nothing; `#[allow(dead_code)]` suppressed the compiler's own report | `nros-node/node.rs:159,184` | W4 |
| 11 | `init_with_args` names the parameter `_args`; `init_with_launch{,_auto}` verify a path and never parse it, returning `ContextSource::Launch` | `api/nros/init.rs:186,212,228` | W4 |
| 12 | the attachment capability is claimed by four doc comments, implemented by nobody, and has no vtable slot — shares a root with the bridge dedup bug | `nros-rmw/traits.rs:2105,2320` | W4 |
| 13 | `take_validated` synthesises `gap: 0, duplicate: false` — an all-clear the safety layer never established | `nros-rmw/traits.rs:2500` | W4 |
| 14 ✔ | six `*_get_actual_qos` echo the request; `_UNKNOWN` is written nowhere in any implementation | `cyclonedds/qos.cpp:140` | W1 |
| 15 | `assert_liveliness` — one question, four answers, no wire traffic on zenoh; the header documents a fifth | `zenoh/shim/publisher.rs:413` | W1 |
| 16 | `get_serialization_format` gives two NULL-slot answers on one type, one of them the `"cdr"` guess the header forbids | `cffi/lib.rs:1712,2001` | W1 |
| 17 | `SubscriptionOptions::sched_context` is structurally unreachable on the poll overload; `message_info` stored and never read; `create_publisher_in` drops its callback group | `nros-cpp/subscription.hpp:674` | W3 |
| 18 | `Client::send_request` hardcodes slot 0 and discards the sequence number — two outstanding requests share one unkeyed slot | `nros-cpp/client.hpp:132` | W3 |
| 19 | `create_action_client`'s QoS is marshalled and named `_qos`; the server side honours the same struct | `nros-cpp/src/action.rs:867` | W3 |
| 20 | every C node's logger is the catch-all `"nros"` — `get_logger` is lookup-only and C has no registration entry point | `nros-c/node.rs:685` | W2 |
| 21 | `RCLCPP_FATAL` expands to `NROS_ERROR`; `Severity::Fatal` exists | `nros-cpp/log.hpp:331` | W3 |
| 22 | `rcl_*_is_valid` is false and `_get_*_name` NULL for entities in their working state | `nros-c/service.rs:2171` | W2 |
| 23 | `rcl_node_is_valid` never consults the context; true after `rclc_support_fini` | `nros-c/node.rs:760` | W2 |
| 24 | name/value truncation returns OK on a DIFFERENT name; a >63-byte parameter declares OK and is then permanently unreachable | `nros-c/util.rs:18` | W2 |
| 25 | `wait_for_service` defaults to 5 s where upstream blocks forever | `nros-cpp/client.hpp:261` | W3 |
| 26 | `Executor::spin_once` — 10 ms default, ms not ns, drains the ready set, and `-1` polls where upstream blocks | `nros-cpp/executor.hpp:167` | W3 |
| 27 | destroying or moving a callback-mode entity leaves the arena dispatching into freed memory; the guard is a comment | `nros-cpp/subscription.hpp:451` | W3 |

### Loud but wrong-shaped, and preferences recorded as constraints

`rcl_timer_fini`/`rcl_guard_condition_fini` reject the idempotent call upstream
accepts (W2 F11) · `nros_log_severity_t` 0–5 against rcutils 0/10/…/50, with
`TRACE = UNSET = 0` and no catch-all over a `#[repr(u8)]` enum (W2 F13) ·
`typedef const void* nros_logger_t` silently accepts the rcutils port (W2 F14) ·
`SpinOptions::only_next`, `ServiceServer`/`ServiceClient`, `GoalId`,
`CancelReturnCode`, `Severity`, the 12 `nros_*_throttle!` macros (W4 F14) ·
`get_subscription_names_and_types_by_node`, the inverted `create_subscription`
argument order, `rclcpp::Result` as a fabricated name in a namespace we do not
own (W3 F16) · `rclcpp::TimerBase` adopts the name and none of the methods, so
`timer_->cancel()` has no local fix (W3 F15) — and RFC-0089 decided today that
this class should NOT exist.

### The instruments — every one validates less than its green implies

* **`correlate.py` never compares return types or struct field values.** When
  arities intersect and no rule fires it assigns `same` unconditionally
  (`:348`). 135 cpp rows sit in `same`; 89 are UNLEDGERED and the ledger wants
  no row for them, because correspondence is decided without looking. **Both
  classes RFC-0089 was written about are outside the gate by construction.**
* `signature_rules.explain()` reconciles by ARITY ALONE — `clock_init`'s
  swapped argument order and `timer_init`'s dropped clock both print
  `no-allocator` as the whole explanation. 3 of 12 rules are dead, including
  one that can never fire because another reaches the arity first.
* `executor-owns-no-entity-storage` still fires 6 times and is cited in 10
  ledger rows after being withdrawn.
* 43 ledger rows cite `rclcpp_compat.hpp`, deleted in step A.
* The ledger validates existence in ONE direction: no row is ever checked for
  still having a subject. Six log rows describe a rename that shipped; `rust:init`
  is a mis-filed row about a different function.
* `check-rmw-slot-producers` tests `if s in produced` first, so 17 of 68 slots
  are unreachable from any application while reported as 6 inert.
* A comment claims a gate — `qos_presets_agree` — that does not exist.

### Corrections to this campaign's own records

* ✔ RFC-0089 §"get_logger follows ROS 2" argues the C++ sentinel is broken
  because `nros::Node::get_logger()` returns the real handle. **Both are the
  sentinel** — the Rust path falls back to `DEFAULT_LOGGER` too. The asymmetry
  argument does not hold.
* ✔ phase-428 F1 said `NROS_RMW_RELIABILITY_UNKNOWN` "exists and is unused" —
  false, it is live. Corrected in place.
* ✔ W10 left `QOS_PROFILE_CLOCK` on `Automatic` — F2's fourth instance, inside
  the fix for F2's third. Corrected.
* Issue 1041 diagnoses a path that has been dead since 2026-08-11; the C++
  timer never reaches the code it cites. Retractable.
* The `polling_subscription` inversion did NOT recur — stated because a clean
  result is also a finding.

### W5 — DONE (2026-09-06)

The authoring half landed: 82 ledger rows across 13 shards (32 new, 50
amended), each with a verdict and a disposition or an explicit statement of why
none applies. Six of the recorded findings above did not survive
re-verification and their rows say so. W6's gates landed on top:
`check-ledger-orphan-refs`, the correlator's return-type comparison (promoted
to verdict `same:ret`, 20 rows authored), `[[nodiscard]]` on `Result` (a signal
in this tree, not an enforced gate — no C++ lane uses bare `-Werror`), and
`#[must_use]` on the three `bool` returns that report failure.

Remainder-sweep findings filed as issues 1099, 1148, 1149, 1150, 1151, 1152.

## CORRECTION 2026-09-06 — this document overstated CI coverage

Notes above assert that `node-std-tests` became merge-gating in PR #438, and
reason from it that a class of reds can no longer reach `main`. **The step was
written, verified locally, described in three places as landed, and merged
nowhere** — `grep -c "node-std-tests" .github/workflows/gate.yml` returns 0 on
`origin/main` and 0 at #438's merge commit. Exactly one commit in the repo ever
contained it, on a branch that was still open.

So the fourth red did not slip past a gate; it reached `main` through the
absence of one. Every "that class is now gated" conclusion downstream is
unsupported until **PR #557** (`ci/land-node-std-gate`) merges — that PR
is this finding's other half, and it lands the step whose absence is the
subject here. Cited by PR NUMBER and not by commit: the hash this line
first carried (`00e8f3087`) stopped naming anything the moment that
branch was rebased, which is the same "believed because it was AUTHORED
rather than OBSERVED" shape the paragraph above is about.
`check-api-parity` remains run by no workflow on any event (issue 1066), which
was correctly stated and is unchanged.

The diagnosis is kept because the wrong turn is the instructive part. The red
IS a descendant of #438's merge, so the tested tree would have carried the step
had it existed — which looked like a hole, and produced two elaborate
explanations (an in-flight queue batch, an environment-dependent test) before
the simplest check. One `git merge-base --is-ancestor` and one `grep -c`
settled it.

**A guarantee asserted from having WRITTEN the code rather than from having
OBSERVED it in the tree that ships** is the mechanism this phase catalogues,
turned on its author.


## W6 remainder (2026-09-06) — the note that had never fired, and a `#[must_use]` sweep that measured smaller than it looked

Two items were outstanding after W6: widen `#[must_use]` on the Rust public
surface, and decide whether the correlator's return-type NOTE could become a
VERDICT now that W5 has authored 82 ledger rows. Both were sized before they
were applied, and both came out differently from the expectation.

### The note had fired ZERO times, in any language

`correlate.py`'s `ret_differs` was landed by W6 as "compares return types, as a
note". It did not. `_ret_of` read `item["ret"]`; `flatten` writes the return
type one level down, per OVERLOAD, in `item["overloads"][i]["ret"]`. So the
function answered `None` for every row on every surface and the branch it feeds
was unreachable — **0 notes across 323 `same` rows** in c/cpp/rust. W6's own
comment names `Publisher::publish`, `Client::service_is_ready` and
`Executor::spin_once` as the rows it would catch; none of the three was ever
marked.

This is the fourth instance in this phase of the same mechanism, and the
CORRECTION section above names it: **a guarantee asserted from having written
the code rather than from having observed it in the tree.** The note was
described as landed in a commit message and in this document; one measurement
would have said otherwise, and none was taken. The gate now carries a live
negative control (four mutations, all caught) precisely so this cannot recur.

### Sizing it, after the fix

Two false-positive classes had to go first, both pure SPELLING:

* `bool` vs `_Bool` — the same type in C before C23, spelled differently by the
  two extractions. **10 of the 11 C rows were this.**
* `rcl_{time_point,duration}_value_t` vs `int64_t` — a two-hop typedef
  (`rcl/time.h:46,48` -> `rcutils/time.h:48,50`). Two cpp rows.

Both are normalised in `_TYPE_NOISE` rather than ledgered, because a row
asserting a divergence that is only a spelling is a FALSE ledger claim, which is
what W5 complained about in nineteen existing rows. That table is authored, so
it is the drift class this phase catalogues; it is kept to three entries and a
fourth alias would surface as a new gate row, not as silence.

Measured after that, on the NATIVE surface the gate runs on:

| lane | `same` rows | of those, return type differs | already ledgered | residue |
| --- | --- | --- | --- | --- |
| cpp | 135 | 17 | 6 | 11 |
| c | 101 | 1 | 0 | 1 |
| rust | 88 | 15 | 9 | 6 |

**PROMOTED.** 33 rows carry the marker; 20 had no ledger entry once the gate ran
on the native surface (which adds `cpp:Node::get_logger` and `cpp:shutdown`, and
answers `LifecycleNode::trigger_transition` from an existing row). Twenty rows
is affordable, so all twenty were authored rather than left as a note. The
verdict is `gate_rows`'s only `same` exception and reports as bucket `same:ret`.

The sharpest of the twenty is `c:timer_get_time_until_next_call`: upstream is
`rcl_ret_t rcl_timer_get_time_until_next_call(const rcl_timer_t *, int64_t *
out)` and ours is `uint64_t nros_timer_get_time_until_next_call(const struct
nros_timer_t *, uint64_t now)`. The error channel is gone — our own doc says the
result is "0 if ready now **or invalid**", so a due timer and a dead one share a
value — and `unsigned` cannot express upstream's documented NEGATIVE "overdue by
that amount". The correlator called it `same`.

Next in line is `cpp:Node::get_logger`, `rclcpp::Logger` upstream and `const
void*` here, which is the row W5's CORRECTION section was already circling.

### The `#[must_use]` sweep is smaller than "sweep the public surface" suggests

**`Result` needs no attribute.** `core::result::Result` is itself `#[must_use]`,
so every `pub fn -> Result<..>` in the tree already warns on a discarded
statement; putting the attribute on the function would be redundant. The gap is
`bool`, which carries no such lint. That halves the stated scope before any code
is read.

Measured across `nros-node`, `nros-rmw` and the `nros` facade: **47 public
`bool`-returning functions, 45 without the attribute.** Marking all 45 is not
the ask — a `bool` that answers a QUESTION (`is_valid`, `has_data`,
`supports_event`, `any_ready`) is not one that reports a FAILURE. Reading all 45
found **three** in the second class, plus three more one crate down in
`nros-params` that the first three delegate to:

* `Executor::declare_parameter_with_descriptor` — the sibling three lines below
  the one method W4 named and W6 marked. Identical `else { false }` arm.
* `metadata_mode::begin_node` / `record_entity` — both doc comments already
  state the consequence of ignoring the answer.
* `ParameterServer::declare` / `declare_with_descriptor` and
  `ParameterBuilder::declare` — the actual implementations, marked because
  fixing only the wrapper is the half-fix rule this repo has a section about.

**Deliberately NOT marked:** the 39 question-answering predicates, and
`remove_pre_shutdown_callback` / `remove_on_shutdown_callback`, whose doc says
in as many words that "it was not there" is an ordinary answer and not an error.
`refill` returns whether budget remains, not whether the refill worked.

Cost, measured with `cargo check --workspace --all-targets` under the repo's own
`HOST_UNCHECKABLE` exclusions: the three in-scope crates produced **zero** new
warnings. The `nros-params` extension produced **nine**, all in that crate's own
`#[cfg(test)] mod tests`, all setup calls whose success the test depends on —
now `assert!(...)`, which is what "tests must fail on unmet preconditions" wants
anyway.

### Pre-existing, not introduced here

`api-parity.py --check` was ALREADY RED on this branch before any of this: six
rows (`c:client_init`, `c:node_init`, `c:service_init`, `c:subscription_init`,
`c:node_get_serialization_format`, `cpp:Node::serialization_format`) carry no
ledger entry. Verified by running the gate from this branch's own HEAD scripts
with the promotion absent — same six, and only those six. They are not in the
`same:ret` class and are outside this work item.

## W6 remainder, second half (2026-09-06) — `--require-disposition` now reaches the silent class, and the estimate was wrong

> **Written independently of the section above, against the same base commit,
> by an author who had not seen it.** Both halves found and fixed the SAME
> `_ret_of` bug (it read `item["ret"]`; `flatten` stores return types one level
> down under `overloads`). The section above landed first — the W6 commit
> that promotes the correlator's return-type check to a verdict — and ITS
> fix is the one that ships — it is the superset: it applies `canon_type`
> inside `_ret_of` and additionally normalises `rcl_{time_point,duration}_value_t`
> to `int64_t`, which the version here does not. What survives from this half is
> what it was actually for: `same_shaped_divergences`, the `--require-disposition`
> extension, its self-test controls, and 18 authored ledger rows disjoint from
> the other half's 20 (zero key overlap, verified row-wise). The numbers below
> were measured BEFORE the merge; the post-merge re-measurement follows the
> table.

`--require-disposition` failed only on `declined` rows. That is the SAFE half:
the name is absent, so a ported file gets `undeclared identifier` and the
compiler has already said everything a disposition would. It skipped every
`divergence` row — the half where the name EXISTS — which is the class RFC-0089
was written about, outside the gate by construction.

Now gated: a `divergence` row whose SUBJECT CORRELATES `same`. The compiler is
silent there by definition, not by luck.

**Measured first, and it did not match the estimate.** The work item said ~33
rows; the measurement is **18**, and it is 18 for a reason the estimate could
not have seen.

| set | count |
| --- | --- |
| ledger rows | 2666 (876 divergence, 854 extension, 697 declined, 174 gap, 65 rename) |
| `divergence` rows whose subject correlates `same`, PORTED surface | **18** |
| the same, NATIVE surface | 8 |
| `divergence` rows whose subject correlates `same:ret` | 2, both by inheritance |
| `divergence` rows in every other bucket — NOT gated | 819 |

**Re-measured after the merge** — the W6 commit that merges the second,
independent `_ret_of` fix in, plus this half — because the other half
authored 20 `divergence` rows on subjects that correlate `same` and they
land in exactly this set — the two halves compound, they do not overlap:

| set | before merge | after merge |
| --- | --- | --- |
| ledger rows | 2666 | 2686 (890 divergence, 854 extension, 697 declined, 177 gap, 68 rename) |
| `divergence` rows correlating `same`, PORTED surface | 18 | **33** |
| the same, NATIVE surface | 8 | 23 |
| `divergence` rows in every other bucket — NOT gated | 819 | 857 |

The merge surfaced exactly ONE row of this class that neither half had:
`cpp:Duration::from_nanoseconds`. The other half's
`rcl_duration_value_t` → `int64_t` normalisation moved it out of `arity-only`
and into `same`, which is what brings it under this gate; its `why` had said in
so many words that it "reads `arity-only` rather than `same`", so the merge
falsified a written claim rather than merely adding a row. Disposition `adopt`
(the two typedefs ARE the same 64-bit signed nanosecond count) and the stale
sentence corrected.

Keyed on the PORTED bucket, deliberately: a disposition answers what a porting
user GETS, and a porting user includes `<rclcpp/rclcpp.hpp>`. On the native
surface the set is 8, and the ten it drops are exactly the shim-side rows —
`SensorDataQoS` and `ServicesQoS` (with their constructors), `Server`,
`Logger`, `Node::Node`, `NodeOptions`, `NodeOptions::start_parameter_services`
and `init` — which is the half RFC-0089's "shipping" table is made of.

### `same:ret` was inert — `_ret_of` never fired anywhere

*(An independent rediscovery of what the section above records. Kept because
two authors reaching the same measurement from different starting points is
itself evidence about the claim; the FIX that ships is the other half's.)*

W6's return-type comparison was recorded as landed. It was not reaching
anything: **0 `same:ret` rows across all three languages**, against 323 `same`
rows. `_ret_of` was written to take "a record set" and CALLED with the flattened
ITEM, whose return types live one level down under `overloads`, so
`item.get("ret")` was always absent and the function always answered `None`.
Nothing consumed `ret_differs` either — not the report, not the self-test — so
the branch could not have been observed from any output.

Fixed at the function (it unwraps `overloads` itself now) and compared through
`canon_type`, the same normaliser the parameters use, so `nros::Result` and
`Result` are not reported as a return-type divergence. `_Bool` joins the type
noise: clang prints it for C's `bool`, and without the mapping nine C
predicates (`*_is_valid`, `executor_trigger_*`) read as diverging in a return
type where only the spelling differs.

After the fix: **45 `same:ret` rows** (11 C, 19 C++, 15 Rust). They include
`Publisher::assert_liveliness` (`bool` -> `Result`), `Executor::spin`/`cancel`
(`void` -> `Result`), the six `LifecycleNode` transitions
(`rclcpp_lifecycle::State&` -> `Result`), `QoS::depth` (`size_t` -> `int`) and
`Node::name`/`namespace` (`String` -> `&str`). Most are UNLEDGERED, so they are
not in this gate — the ledger's contract is one row per non-correspondence and
a `same` row never needed one. **That is the next measurement, not a claim that
they are fine.**

### The 18, and what they were classified as

* **refuse-loud (3)** — `cpp:NodeOptions`, `cpp:NodeOptions::start_parameter_services`
  (every setter and getter is a dependent `static_assert` carrying
  `NROS_RCLCPP_REFUSE_NODE_OPTIONS`), and `rust:Node`.
* **adopt (4)** — `cpp:SensorDataQoS`, `cpp:ServicesQoS` and their constructors:
  name, base class and all ten policy values are upstream's, pinned by
  `static_assert` against the W10 table.
* **adopt-bounded (11)** — `cpp:init`, `cpp:Node::Node`, `cpp:Logger`,
  `cpp:Server`, `cpp:GoalResponse`, `cpp:CancelResponse`, `rust:NodeOptions`,
  `rust:Publisher`, `rust:Subscription`, `rust:Timer`, `rust:MessageInfo`.

Three of the eighteen are recorded with the classification stated as WEAKER than
the disposition's definition, rather than glossed:

* `rust:Node` is refuse-loud by the TYPE SYSTEM — the name resolves to a trait,
  so every rclrs-shaped use is a type error — but the diagnostic is rustc's
  "expected struct, found trait", which names neither the constraint nor the
  nano-ros alternative. RFC-0089's refuse-loud promises the second half. Rust
  has no `static_assert` reaching a trait-vs-struct collision.
* `rust:MessageInfo` is adopt-bounded on two envelopes — backend-dependent
  POPULATION (a zenoh attachment fills the fields; a backend without one leaves
  the `Default`, and a reader cannot tell that from a real zero) and the 16-byte
  `PUBLISHER_GID_SIZE` against the ABI's 24-byte `rmw_gid_t` — **neither of
  which is in the type's doc comment today.** An adopt-bounded envelope is
  supposed to be part of the API; here half of it is only in the ledger.
* `cpp:Logger`'s row carried a stale fact: it said ours is "an OPAQUE
  `const void*`". That is the C spelling. The C++ one has been a real
  `class rclcpp::Logger` with a name and `get_name()` since stage 6 step A. The
  divergence that survives is that the NAME does not reach the output.

### Negative control

Four planted rows in `api-parity.py`'s `self_test()`, which
`just check api-parity-ledger` runs on the normal path — a control behind a flag
is prose:

| planted | expected |
| --- | --- |
| `divergence` + `same`, no disposition | REPORTED |
| `divergence` + `differs`, no disposition | not reported |
| `divergence` + `same`, WITH a disposition | not reported |
| `gap` + `same`, no disposition | not reported |

Each is also planted ALONE, so a pass cannot come from another row's presence,
and an inheritance pair pins that a member reports its TYPE's ledger key. Three
code mutations were run and all three go red: returning nothing (3 checks),
widening the scope past `same` (3 checks), and letting an authored disposition
stop satisfying it (2 checks). End-to-end against the real ledger and the real
extracted surfaces, stripping the disposition from `cpp:NodeOptions`,
`rust:Node` or `cpp:init` turns the gate red naming exactly that row, while the
857 non-`same` divergence rows keep it green.

**What this gate still does not reach**, stated because a green is narrower than
it looks: `--check --require-disposition` needs clang and nightly rustdoc, and
`check-api-parity` is run by no workflow on any event (issue 1066). It is a
local gate on `just ci l1`, not a merge-gating one.

## Q1 follow-through (2026-09-06) — the `refuse-loud` field was wrong in both directions

A read-only audit classified every `"disposition": "refuse-loud"` row against
the code. Of 144 rows carrying the label, three named a refusal that exists.
The rest split two ways, and the ledger was also wrong in the opposite
direction: the refusals that *do* exist mostly had no row.

### What was measured

Row set: `disposition == "refuse-loud"` over `docs/reference/api-parity-ledger/*.json`.
Presence: `scripts/api-parity.py --lang {c,cpp,rust} --show same --show all`
(the `same` bucket is hidden unless named, which is why an earlier join
reported four `same` rows as absent from the report), joined on key. Every
`theirs-only` hit was cross-checked with a non-comment declaration grep on the
key's leaf name over `nros-c/{include,src}`, `nros-cpp/include` and
`packages/**/*.rs`; the 15 keys that produced a hit were all the same leaf name
on a different receiver (`std::make_shared`, `Node::create_client` for the
`LifecycleNode::` member, `QoSProfile`'s builders for `IntoPrimitiveOptions::*`).

| class | count | what a porting user actually gets |
| --- | ---: | --- |
| absent, labelled refuse-loud | 130 | `undeclared identifier` / `no member named` — honest, names nothing |
| exists and differs, labelled refuse-loud | 8 | a compile error the compiler chooses, or nothing at all |
| implemented refusal with a row | 3 | the diagnostic the label promises |
| implemented refusal with NO refuse-loud row | ~23 sites, 20 keys | the diagnostic, unrecorded |

The structural cause of the last row is RFC-0089's own observation that a
refusal is a declaration: once a name refuses it correlates `same` or
`ours-only`, `--check` stops demanding a row, and nobody writes one.

### What changed, per commit

1. **130 rows → `absent`.** 129 correlate `theirs-only`; `cpp:SubscriptionContentFilterOptions`
   is on neither surface. `why` kept everywhere; the seven `NodeOptions::{allocator,
   context, append_parameter_override, parameter_event_qos, parameter_overrides,
   parameter_event_publisher_options, rosout_qos}` rows shared a sentence claiming a
   `= delete` overload that `options.hpp` never declared, replaced with what is there.
   Edited row-wise through a loader with a duplicate-key-raising `object_pairs_hook`;
   every shard round-trips byte-for-byte, so the diff is 130 disposition lines and 7 whys.
2. **The eight exists-and-differs rows.** `c:get_zero_initialized_{service,client}` →
   `adopt` (exported under rcl's own name, correlate `same`: verdict bugs).
   `c:{service,client}_init_default`, `cpp:spin_until_future_complete`,
   `cpp:Subscription::Subscription<M>`, `rust:Executor::spin_async` → `adopt-bounded`,
   each naming what the compiler points at (argument 3's typesupport type; the
   `int32_t` timeout against a `chrono::duration`; the constructor set; arity with no
   `SpinOptions`) and the envelope it does not. `spin_async` was argued rather than
   defaulted: it is a working entry point the `Promise` pattern relies on, so
   `refuse-loud` would claim a `compile_error!` that does not exist.
   **`rust:init_with_args` is now loud in code** (`packages/api/nros/src/init.rs`):
   `args_have_ros_args` (exact match, unit-tested against the prefix mutation) →
   `nros_log::log_error!` → panic carrying `REFUSE_INIT_ARGS`, the C++
   `NROS_RCLCPP_REFUSE_INIT_ARGV` text with Rust spellings. Two `#[should_panic]` tests,
   one pass-through, `cargo test -p nros --features env,std`.
3. **Every live refusal site has a row.** Eight `NodeOptions` members that only
   inherited the type's disposition get their own rows citing both `static_assert`
   lines; `enable_logger_service` and `start_parameter_services` re-cited;
   `SystemDefaultsQoS` cites `qos.hpp:767`; `Node::create_service` / `create_client`
   become `divergence` + `adopt-bounded` citing the `shared_ptr`-handler overload at
   `nros.hpp:955` / `:990` (and note the W5 discarded-`Result` finding is closed by
   `require_created`); five `cpp:RCLCPP_*_THROTTLE` macro rows in `other.json`, which no
   report lists because the extractor emits no macros; `cpp:init`'s citations refreshed
   to `:415` / `:427`.
4. **Six pre-existing unledgered rows** that made `just check api-parity` red before
   this branch touched anything: `c:{node,subscription,service,client}_init` are
   `divergence` + `absent` (rcl's five-argument spelling has no symbol here; the old
   forwarders were retired at phase-417 stage 6 step B, and the rclc-layer
   `*_init_default` is where the bounded adoption lives — `absent`, not the
   `adopt-bounded` first guessed, because that label promises the name exists);
   `c:node_get_serialization_format` / `cpp:Node::serialization_format` are
   `extension` with no disposition, like every other ours-only row.
5. **Two new rejections on `--check --require-disposition`.**
   `misdeclared_refusals`: a `refuse-loud` row whose key correlates `theirs-only`
   (looks through inheritance, reports the member). `unreferenced_refusals`: a
   `#define NROS_RCLCPP_REFUSE_*` in `nros-cpp/include/nros/*.hpp` cited by no row's
   `why`. The full "refusing declaration whose row is not refuse-loud" rule needs a C++
   parse — the `shared_ptr` refusal is the third overload of a verb whose row is
   legitimately `adopt-bounded`, and the throttle refusal is a `detail::` template five
   macros expand to — so it was skipped and the docstring says so. Self-tests on the
   normal path: four planted rows (each also alone), an inheritance pair, both directions
   of the macro check, the `#define` reader, and a guard that the reader finds something
   under the real include dir. Mutation controls: widening the bucket test → 4 red;
   returning `[]` from the macro check → 1 red; end-to-end, flipping
   `cpp:Clock::wait_until_started` to refuse-loud and appending a planted `#define` to
   `log.hpp` each turn `--check --require-disposition` red naming the row / macro.

### Census

| disposition | before (pre-Q1 branch state) | after |
| --- | ---: | ---: |
| refuse-loud | 144 | 20 |
| absent | 518 | 652 |
| adopt | 96 | 96 |
| adopt-bounded | 20 | 27 |
| (none) | 1908 | 1910 |
| rows | 2686 | 2705 |

The 20 refuse-loud rows now: `cpp:NodeOptions` and its ten refused members,
`cpp:SystemDefaultsQoS` and its constructor, the five `RCLCPP_*_THROTTLE` macros,
`rust:init_with_args`, and `rust:Node`. The last is the one this pass did not touch
and does not fully believe: its diagnostic is rustc's "expected struct, found trait",
which names neither the constraint nor the alternative, and the row says so.

### Leftovers, closed (2026-09-06, follow-through PR)

Three things the pass above left as found. Each is now closed, and what was
measured is stated apart from what was reasoned.

1. **The `env` surface is measured.** `env` joined `extract_rust.NROS_FEATURES`
   (it requires `std`, already in the set). Measured with
   `scripts/api-parity.py --lang rust --show all` before and after, on
   the branch state at the time: `same 88 → 89`, `theirs-only 503 → 502`,
   `ours-only 1225 → 1238`.
   `rust:Context` moved `theirs-only` → `same`, as predicted. What appeared was
   **19 report lines, not the 12 the earlier estimate said**: 16 UNLEDGERED (the
   six rclrs `Context::` members, which stop inheriting once their type is
   `same` — a member gap inside a shipped type is argued on its own, per
   `lookup`'s rule — plus `Context::config`, `ContextSource`, `InitError`,
   `REFUSE_INIT_ARGS`, `args_have_ros_args`, `ExecutorConfigEnvExt`,
   `ExecutorConfigEnvExt::from_env`, `resolve_hosted`, `try_resolve_hosted`,
   `rmw_selector`) and 3 already ledgered (`init_with_args` — now printing
   `<refuse-loud>` on a surface the gate can see, so `misdeclared_refusals`
   vouches for it — `init_with_launch`, `init_with_launch_auto`). All 16 have
   rows: `Context::new` / `default_from_env` / `domain_id` are `gap` + `absent`
   (`default_from_env` is `nros::init()` under rclrs's spelling, and the row says
   so rather than opening a rename on the entry-point family here);
   `Context::from_env` / `ok` / `create_executor` are `declined` + `absent`
   (options object, shutdown state, allocating factory — each names its
   constraint); the rest are `extension`. `rust:Context` itself is
   `divergence` + `adopt-bounded` (same identity, construction by the free
   `init*` family, executor opened into caller storage), and its stale
   `"bucket": "theirs-only"` field is gone.
2. **Stale `why` texts.** `rust:init` now describes `nros::init()` — the
   `nros::logging::init` text it carried was a mis-filing. The
   `cpp:Node::create_{publisher,subscription,wall_timer}` rows describe
   `detail::require_created` (`nros.hpp:384`) and keep the discarded-`Result`
   state as one dated sentence; `cpp:Node::Node` and the
   `create_wall_timer` rename resolution had the same predecessor and got the
   same treatment. **The class was `rclcpp_compat.hpp` cited as if it
   existed**: 42 rows across nine shards (one, `cpp:Result`, already said the
   header was retired and stands) — 19 carrying the identical
   "visible only because `rclcpp_compat.hpp` is now a translation unit"
   sentence (rewritten once, mechanically), the QoS-profile family, `Rate` /
   `WallRate`, `NodeOptions`, the parameter trio, `FutureReturnCode`, and two
   rows claiming `NodeOptions::enable_rosout` is an inert bool when it is a
   `static_assert` refusal (`options.hpp:250`). Every one now names the header
   and line that owns the declaration and keeps the shim as history. Sweep:
   `grep -o '.\{100\}rclcpp_compat.hpp' docs/reference/api-parity-ledger/*.json`
   — every remaining hit sits in a sentence that says the file was deleted.
   The orphan-refs gate was green throughout: it matches `dir/file.ext` and a
   bare `rclcpp_compat.hpp` has no directory, which is why this needed a
   reader and not a gate.
3. **Gates.** `just check api-parity-ledger`, `ledger-orphan-refs`,
   `gate-selftests` and `api-parity` (ending "every divergence carries a
   ledger entry") all green; every shard re-parsed through a
   duplicate-key-raising `object_pairs_hook` and round-trips byte-for-byte.
   `api-parity`'s C extractor needed the duplicate
   `nros_node_get_fully_qualified_name` fixed. That was cherry-picked as a
   temporary aid when this was written, and attributed to PR #612 — which never
   carried it. The fix is `add92bfcf` (PR #630, issue #1162), it is on `main`,
   and this branch is rebased onto it, so no aid is needed any more.

| disposition | before (the Q1 leftovers baseline) | after |
| --- | ---: | ---: |
| refuse-loud | 20 | 20 |
| absent | 650 | 656 |
| adopt | 96 | 96 |
| adopt-bounded | 27 | 28 |
| (none) | 1925 | 1934 |
| rows | 2718 | 2734 |

(Counted over every non-`_` key in the 17 shards; the census above was taken
on an earlier tip, which is why its "after" column and this "before" column
differ by the rows the merge with `main` added.)

## Closure (2026-09-07) — two items the follow-through left open

### 1. `check-ledger-orphan-refs` now sees a bare filename

The gate matched `dir/file.ext` only, which is how 42 rows citing the deleted
`rclcpp_compat.hpp` by bare name stayed green until a reader found them (item 2
above; PR #634 fixed the rows and said so). Widened, and the rule stated once
in the script's docstring:

* **Two spellings.** A repo-relative path (`packages/`, `docs/`, ...) must exist
  on disk, as before. A **bare `name.ext`** must match at least one tracked
  file of that basename (`git ls-files`), for `ext` in `hpp h rs py cpp c toml
  just md json jinja sh`. Both accept a `:NNN` line or a `:NNN-NNN` range,
  stripped and not checked.
* **The allow-list is three words**, sentence-scoped: `deleted`, `removed`,
  `retired`. "`x.hpp` holds it. The shim was deleted." is still an orphan —
  the first sentence claims the file is there. Nothing else excuses a bare
  name.
* **A directory-qualified path under a non-repo root is upstream and out of
  scope** (`rclcpp/qos.hpp`, `rmw/rmw.h`). That was already the rule for
  paths; it is now the convention the bare-name rule relies on: cite an
  upstream file WITH its directory, and a bare name is always ours.
* A work-item label (`W4.c`, `Q1.b`) has the shape of a bare name and is
  skipped by its stem (one or two letters and digits). Two ledger rows cite
  `W4.c` / `W5.c`; both would have been false findings.
* Nested objects (`rename`, `their_rename`) are scanned too. The first version
  read top-level strings only, and 14 of the 29 findings below sat in a
  `rename.residue`.

Self-test grew from three controls to ten: a planted bare orphan (red), the
same name with a `:120-131` range (red), the same name in a sentence saying it
was deleted (green), an existing basename with a line (green), a label
(green), a nested `rename.resolution` orphan (red), and the "deleted" word in
the NEXT sentence (red — the scoping control).

**Run on the ledger it found 29 rows, all real, fixed in the same change that
widened the gate to bare filenames:**

| what | rows | fix |
| --- | ---: | --- |
| `param_deprecation_probe.c` / `param_name_aliases.c` in the `c:param_*` rows' `residue` | 14 | W-B5 (commit `4dc5caee9`) retired the forwarders and the four typedef aliases and deleted the probes; the rows said the forwarders "survive" and the probes prove they warn. Now: survived one release, retired, `residue: NONE` dated |
| `service_deprecation_probe.c` in `c:service_send_reply_raw` | 1 | same class, same fix |
| `action_deprecation_probe.c` in the two `c:action_server_*goal_count*` rows | 2 | same class, same fix |
| `rmw.h` bare in the eight `*_get_actual_qos` rows | 8 | `rmw/rmw.h` |
| `graph.h`, `network_flow_endpoint.hpp`, `range.rs`, `create_timer.hpp` bare | 4 | `rcl/graph.h`, `rclcpp/network_flow_endpoint.hpp`, `rclrs/src/parameter/range.rs`, `rclcpp/create_timer.hpp` |

The 17 forwarder rows are the same class the gate's own docstring lists as its
second bullet ("describing forwarders removed in phase-417 W-B5") — the
filename was the only handle a filesystem check had on them, and it was
enough. What the gate still cannot see, and says so: the `c:param_*` rows are
`bucket: ours-only` for names no surface carries; that is the extractor's
question, not a stat's.

Gates: `just check ledger-orphan-refs`, `gate-selftests` (142/254 with a
selftest), `grep-q-error-conflation` (the script greps nothing; 87 baselined
sites, none grew), `gate-lists`, `api-parity-ledger` — all green.

### 2. The `init*` entry points — one `rename`, not a family decision

The Q1 follow-through filed `rust:Context::default_from_env` as `gap` +
`absent` with a note that it is byte-for-byte `nros::init()` under rclrs's
spelling, and deferred the verdict because "the entry-point spelling is one
decision for the four of them". Surveyed here, against the recorded rclrs
0.7.0 surface (`docs/reference/api-surface/rclrs.json`, `rclrs/src/context.rs`)
and `packages/api/nros/src/init.rs`. rclrs has no free `init`; its entry
points are the `Context` constructors. The RFC-0089 question is whether the
porting edit is MECHANICAL — directed by the compiler, one local edit.

| rclrs | ours | 1:1? | verdict | what the compiler says, and what to type |
| --- | --- | --- | --- | --- |
| `Context::default_from_env() -> Result<Self, RclrsError>` | `nros::init() -> Result<Context, InitError>` | **yes** — no args either side, same inputs (`ROS_DOMAIN_ID`, env rung), same output type, same fallibility | **`rename`** (was `gap`); disposition stays `absent` | `no function or associated item named default_from_env`; write `nros::init()?` — `rename.resolution` |
| `Context::new(args, options)` | `nros::init_with_args(args)` | no — upstream PARSES `--ros-args`, ours REFUSES it at the call; upstream takes `InitOptions`, ours has none | `gap` + `absent` (unchanged) | maps only when argv has no `--ros-args` AND options were default: two facts a reader checks, not an edit the compiler directs |
| `Context::from_env(options)` | `nros::init()` when `options == InitOptions::new()`; nothing otherwise | no — the argument has no counterpart | `declined` + `absent` (unchanged) | with default options the caller wanted `default_from_env`, whose row is the rename; with a domain id, it goes through the boot-config ladder (`rust:InitOptions`) |
| — | `init_with_launch`, `init_with_launch_auto` | no upstream candidate | `extension` (unchanged) | nothing to port |
| — | `nros::init()` (ours-only half) | pair of row 1 | **`rename`** (was `extension`) | `extension` says ROS 2 has none; rclrs has it under another name. Both halves carry the verdict (SCHEMA's pair rule) |

So the family argument was wrong in its premise: only one member maps 1:1,
and the decision is about one alias, not four spellings. Both `rename` rows
are `status: blocked-needs-decision`, and `rust:Context` now points at the
member that is a rename. **No code alias was added** — this is a ledger
decision.

**For the maintainer — would an alias make the port mechanical?** Yes, for
this one member and cheaply: `impl Context { pub fn default_from_env() ->
Result<Context, InitError> { init() } }` in `init.rs` behind `env`, two lines,
`init()` untouched (it anchors `init_with_args` and the launch forms). RFC-0089
clause 2 permits it and no constraint forbids it; the cost is a second spelling
of one function on the Rust surface, and this phase's scope note says the
sweep does not rename. It would NOT make `Context::new` or `from_env`
mechanical — those differ in what they accept, which no alias fixes — and the
ported program still changes the line after it (`Executor::open(&ctx.config(name))`
for `ctx.create_executor(..)`, `rust:Context::create_executor`).

Gates: `just check api-parity-ledger`, `api-parity` (ends "every divergence
carries a ledger entry"; the report prints `Context::default_from_env  rename
theirs-only <absent>` and `init  rename  ours-only`), `prelude-tiers` (a
`rename` is prelude-eligible where an `extension` was not; 92 names, unchanged),
`ledger-orphan-refs`. `cargo +nightly fmt --all -- --check` clean, no Rust
edits. Every shard re-parsed with a duplicate-key-raising `object_pairs_hook`
and round-trips byte-for-byte.

### Decided (2026-09-07): the alias is taken, as a family, in phase-427

Maintainer's call, after the discussion recorded in RFC-0089 "Context and
`init`, settled": `Context::default_from_env()` is added beside `nros::init()`,
together with `InitOptions` and `from_env`/`new`, and — the part the ledger
could not see — `Context` compiles on every target with the baked build
environment as its freestanding source. That is phase-427 W9; the executor/
node split it exposes (`create_executor`, `create_node`) is W10. The two
`rename` rows move from `blocked-needs-decision` to `decided` with W9 named as
the resolution; their dispositions flip when the code lands, not before.

