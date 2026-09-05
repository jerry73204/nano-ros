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

* **W13.a [zenoh]** — the matched set, fed by the existing liveliness
  subscriber. Bounded (one entry per matched service server), so it fits a
  fixed arena and needs no new zenoh resource.
  *Acceptance:* a server that stops and restarts reads unavailable in between;
  today it reads available forever.
* **W13.b [zenoh]** — `service_is_ready` reads the set. The
  `#[cfg(not(feature = "platform-bare-metal"))]` gate on the liveliness
  subscriber decides bare-metal's answer: if the subscriber cannot run there,
  bare-metal returns `Err(Unsupported)` and the caller waits, which is honest.
* **W13.c [cyclone, xrce]** — fill the `service_server_is_available` vtable
  slot. It EXISTS and both leave it NULL, which is why they answered without
  asking. Cyclone is cheap: DDS has the cache natively.
* **W13.d [core]** — delete the pair from the trait and its six call sites,
  once a–c land. *Acceptance:* `grep -rn "poll_server_discovery"` returns
  nothing, and a cyclone client's `wait_for_service` answers from the DDS cache
  rather than timing out.

### What already landed on `fix/1087-1088-server-availability`

The optimistic default is `Ok(None)`; the two comments claiming an rclcpp
"snapshot semantic" are corrected (there is none —
`ClientBase::service_is_ready()` calls rcl on every invocation); the zenoh latch
returns `Err(Unsupported)` instead of a permanent yes. W13 is what closes 1087.

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

### What W5 still owes

Each row above needs a ledger entry with a `disposition`. That is the authoring
step, and it is deliberately separate from this record: recording the finding
protects it from loss, ledgering it is what makes `--check` enforce it.
