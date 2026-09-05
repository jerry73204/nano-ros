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

* **zenoh implements none of reliability / durability / history / depth.**
  `grep -rn "qos\.depth\|qos\.reliability\|qos\.durability\|qos\.history"
  packages/rmw/zenoh/nros-rmw-zenoh/src` → **0 matches** (verified). Yet
  `supported_qos_policies()` returns `CORE`, whose definition
  (`traits.rs:1719`) is `RELIABILITY | DURABILITY_VOLATILE | HISTORY | DEPTH`,
  so `validate_against` accepts RELIABLE. The only consumers of
  `.reliability`/`.durability` are `keyexpr.rs:196,202`, which serialise them
  into the liveliness token **a `rmw_zenoh_cpp` peer reads**. Ring depth is the
  build-time `SUBSCRIBER_RING_DEPTH`, never `qos.depth`. `TRANSIENT_LOCAL` *is*
  correctly refused, so the loud path exists and reliability is simply not on
  it.
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
  `NROS_RMW_RELIABILITY_UNKNOWN` exists and is unused.

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
* **W9 [rmw] — QoS claimed must be QoS applied.** `supported_qos_policies()`
  must be derived from what the backend implements, not authored. *Acceptance:*
  a backend that ignores `qos.reliability` cannot advertise `RELIABILITY`; the
  zenoh service path either applies QoS or refuses loudly.
* **W10 [rmw] — one QoS table.** Upstream's 7 profiles are hand-transcribed in
  four places (26 constants) with no gate binding them; `PARAMETER_EVENTS` is
  the proof. *Acceptance:* one SSoT, generated or gated, and the
  `PARAMETER_EVENTS` test asserts upstream's values.
* **W11 [rmw] — resolve every issue reference.** Every `issue NNNN` in a gap
  reason and every authored `issue =` resolves to a file with `status: open`.
* **W12 [rmw] — the prose.** Delete the retired sign contract, the two stale
  slot doc blocks and the fictional NULL-slot fallbacks; then gate what can be
  gated (a doc naming a slot that does not exist is mechanically checkable).
