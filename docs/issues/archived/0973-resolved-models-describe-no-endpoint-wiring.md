---
id: 973
title: "No resolved SystemModel describes endpoint wiring — 0 of 119 carry topics, services or actions"
status: resolved
area: orchestration
severity: medium
found: 2026-09-01
resolved: 2026-09-06
related: [0900, 1120, 1121, 1140, RFC-0060, RFC-0063, phase-392, phase-412]
---

# `structure` carries placement, never endpoints

Measured across every resolved model in the tree:

```
resolved models        : 119
describe ANY wiring    :   0
have an actions block  :   0
have an action CLIENT  :   0
```

Every model's `structure` holds `scopes` and `nodes` and nothing else. The
schema has room for more — `ros_launch_manifest_model`'s `Structure` declares
`topics: BTreeMap<String, TopicWiring>`, `services` and `actions` (both
`ServiceWiring`, which carries `server` AND `client` lists) — and all three are
empty everywhere.

The sharp case: `examples/workspaces/rust/build/nros/models/demo_bringup/
action_client_model.yaml` resolves `action_client.launch.xml`, whose single node
`/fibonacci_client` IS an action client. Its `structure` names the scope and the
node, and has no `actions` block at all.

## Why this is not obviously a bug

`entity_facts.rs` already anticipates it. `describes_wiring()` tests
`!structure.topics.is_empty() || !services.is_empty() || !actions.is_empty()`,
and `a_model_that_describes_no_wiring_abstains_on_the_app_count` asserts that a
model without wiring makes the consumer ABSTAIN rather than report a zero. So
the code is correct for this state; the question is whether the state is
intended.

Two readings, and the difference matters:

* **Intended** — a launch file declares nodes, and endpoint wiring is a
  different layer that only some resolves produce. Then the abstain path is the
  permanent normal case, and anything wanting endpoint counts must get them
  elsewhere (the per-component sidecar, which issue 0900 extended for exactly
  this reason).
* **A gap** — the resolver can derive wiring and does not, or does and drops it
  somewhere between layer 2 and the written model. Then `describes_wiring` is
  guarding a bug rather than a design.

Nothing in the tree distinguishes these, which is itself the problem: a
consumer cannot tell "this system has no services" from "this model does not
say".

## What it currently costs

Issue 0900 wanted the action-client count to size the executor arena — 74,240
bytes against 16,384, ~56.5 KiB of TASK STACK per image. The model is the
natural source and abstains on all 119, so that path delivers nothing today. The
fallbacks are a per-component sidecar (which cannot cover cross-compiled
components — they are unprobeable by construction) and a post-link `nm` gate
(landed, and reporting 181 over-budgeted images).

More generally, any future sizing that wants "how many X does this image have"
meets the same wall.

## ANSWERED 2026-09-03 — reading 1. Wiring is AUTHORED, and nobody authors it

Traced through the resolver rather than inferred from the models.

`model_builder.rs` DOES populate `structure.topics` and `structure.services`
(and, since R1-P2, `structure.actions`). It fills them from `ManifestIndex`,
which `manifest_loader` builds by reading a **`<stem>.contract.yaml` sitting
beside each launch file**. Endpoint wiring is a separate authored artifact, not
something derived from the launch XML.

Counted in this tree:

```
*.launch.xml            : 93
*.contract.yaml         :  0
```

So the models are empty because **nothing declares wiring**, not because the
resolver drops it. `describes_wiring()` is guarding a design, and the abstain
path is the permanent normal case until someone authors contracts.

**Reading 2 was true once and is already fixed.** `manifest_loader.rs:255`
records it: "Previously the loader silently dropped `actions:` — the model's
`structure.actions` was always empty." That is the exact bug this issue
suspected, found and repaired by R1-P2 before this issue was filed. Looking for
it again would have been a second search for a closed defect — which is why this
was traced to the loader instead of stopping at the empty models.

### What that means for the consumers

Any sizing that wants "how many X does this image have" from the MODEL needs
contract files to exist first. That is a product decision — 93 of them, hand
written, describing endpoints that the component sidecar already records — and
issue 0900 took the other route for exactly this reason: the per-component
sidecar now carries action and service CLIENTS, and it needs no contract.

So the recommended action is item 3 below (write the design down), NOT item 2.
Item 2 is struck.

## Work

1. ~~Decide which reading is right.~~ **Done: reading 1, evidenced above.**
2. ~~If wiring IS meant to be produced: find where it is lost.~~ **Struck.**
   Nothing is lost; the input does not exist. The one real instance of this
   (dropped `actions:`) was fixed by R1-P2.
3. **This is the action.** Say so in RFC-0060 and in `describes_wiring`'s doc
   comment, so the abstain path reads as the designed outcome rather than as a
   symptom, and consumers stop being written against it. Name the input a user
   would have to author (`<stem>.contract.yaml`) so "the model does not say" is
   actionable rather than merely true.

Found while implementing issue 0900. Filed rather than pursued: the answer
changes the resolver's contract, and 0900 had already been mis-scoped three
times by inferring one level too shallow.

## RESOLVED 2026-09-06 — reading 1 stands; both counts moved under it

### Item 3 had already landed, and nobody closed the issue

`da0344af1` (2026-09-03) added the RFC-0060 section and the `describes_wiring`
doc comment this issue asked for. It is on `main`. The issue stayed `open`
because the commit that answered it never flipped the status — the same failure
mode the same commit message describes for the issue file itself, one field over.
So this resolution is mostly RE-VERIFICATION plus the corrections the
re-verification forced.

### The counts moved. The conclusion did not

Re-measured by RESOLVING every launch file in the tree with a freshly built
`nros-launch-resolve`. The 135 resolved models already sitting in the checkout
were NOT used: the newest was written 2026-09-05 16:09 (+0800) and phase-412's
commit reached `main` at 2026-09-06 05:59 (+0800), so every one of them predates
the first contract file by ~14 hours — museum artifacts, exactly the class the
repo keeps filing ghost issues from.

| | 2026-09-03 | 2026-09-06 |
| --- | ---: | ---: |
| `*.launch.xml` | 93 | **122** |
| `*.contract.yaml` | 0 | **5** |
| models that resolve standalone | 119 | 114 (8 need a sourced workspace) |
| …that DESCRIBE wiring | **0** | **5** |

The 5 are exactly the 5 with a contract beside them, all in
`examples/workspaces/cpp/src/demo_bringup/launch/`, authored by phase-412 on
2026-09-05 — the day after this issue was answered.

**Probe control, because a probe that can only return zero is worthless.**
`examples/workspaces/cpp/.../system.launch.xml` resolves to `topics=1
services=0 actions=0` with the provider-contract channel on, and to
`topics=0 services=0 actions=0` with `--no-provider-contracts`. Same file, same
resolver, both directions observed. At the CONSUMER level too:

```
$ nros ws entity-facts --model <service_server model>
NROS_DECLARED_INFRA_QUERYABLES=none
NROS_DECLARED_SERVICE_SERVERS=1
$ nros ws entity-facts --model <action_server model>
NROS_DECLARED_SERVICE_SERVERS=3            # an action server is 3 queryables
```

So `describes_wiring()` is not a function that always returns false, and the
"application half" phase-392 W5 recorded as permanently absent now reaches a
build. Reading 1 is confirmed and STRENGTHENED — phase-412 states the underlying
rule outright: *emitted evidence can VERIFY a count; it can never SUPPLY one.
Somebody has to state it.*

### What was wrong in the answer, and is fixed here

The 09-03 text told a consumer wanting per-image entity counts to use "the
component SIDECAR, which needs no contract file — the route issue 0900 took."
**That route no longer exists.** phase-412 retired
`nano_ros_node_register(... ENTITIES ...)` into a `FATAL_ERROR` naming
`<stem>.contract.yaml`, because a per-component list is hand-maintained beside
the code with nothing comparing the two, and on the safety island it drifted
(six subscriptions declared for a node creating seven; every derived pool short
by one; days on real silicon). So RFC-0060 and the doc comment were, for three
days, pointing readers at a retired mechanism — the precise failure this issue
exists to prevent.

### Changed

* **RFC-0060** — the section is rewritten: both contract channels named
  (provider sidecar and `--contracts` overlay), the measurement dated rather
  than quoted as a constant, the invariant stated as the durable claim
  (*wiring ⟺ an authored contract*), and a new subsection recording that
  phase-412 made the sidecar the ONLY route rather than an alternative.
* **`entity_facts::describes_wiring`** — doc comment rewritten to lead with the
  designed-abstention, name the file a user authors as a code block, and carry
  the dated measurement with the "quote the invariant, not the number"
  instruction.
* **The stale universals swept as a class**, not just at the reported site —
  every "all 115 resolved models" / "every in-tree example today" claim:
  `cmd/entity_inventory.rs`, `nros-zpico-build/src/runner.rs`,
  `cmake/NanoRosMessageBounds.cmake`, `cmake/NanoRosEntry.cmake`,
  `nros-macros/src/main_macro.rs` (×2), the `entity_facts` test comment, and
  phase-392's W5 status section. Sweep:
  `grep -rn "115 \|no wiring\|describes no wiring"` over `packages/ cmake/`.
* **The diagnostic (item 4)** — the existing phase-392 W5 configure line already
  printed `application count undeclared (no model here describes wiring)`, which
  is true and unactionable. It is EXTENDED, not duplicated, to name the input:
  `…; to declare it, author <bringup>/launch/<stem>.contract.yaml beside
  <stem>.launch.xml (RFC-0060)`. Rendered output verified by running the three
  statements under `cmake -P`. A second diagnostic beside the first was
  deliberately not added.

### Deliberately NOT done

93 (now 122) contract files were not written. The issue rejected that as a
product decision and it still is; phase-412 authored 5 where a real image needed
them, which is the right granularity.

### Spun out

**Issue 1140** — `describes_wiring` tests `structure.{topics,services,actions}`
while `EntityInventory::from_model` tests those PLUS `contracts.node_paths`,
where a contract's timer paths land. Measured: a timer-only contract makes
`entity-inventory` derive `NROS_EXECUTOR_MAX_CBS=1` and `entity-facts` abstain,
on one model. Two predicates for "was a contract authored", disagreeing on
exactly one case. No such contract exists in the tree yet, so it is a trap for
the next author rather than a live defect — filed rather than fixed here,
because that change alters what a build derives and this one does not.
