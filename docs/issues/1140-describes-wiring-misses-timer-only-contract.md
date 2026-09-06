---
id: 1140
title: "Two predicates ask whether a contract was authored, and they disagree on a timer-only contract"
status: open
area: orchestration
severity: low
found: 2026-09-06
related: [0973, 0900, 1120, 1121, RFC-0060]
---

# `describes_wiring` reads `structure` only; `from_model` also reads `contracts.node_paths`

Issue 0973 established that endpoint wiring in a resolved `SystemModel` is
AUTHORED — it appears only when a `<stem>.contract.yaml` resolves for the scope
— and that every consumer must therefore be able to tell "no contract was
authored" from "the contract says zero". Two consumers implement that test, and
they do not implement the same test.

| site | predicate |
| --- | --- |
| `nros_cli_core::cmd::entity_facts::describes_wiring` | `structure.{topics,services,actions}` non-empty |
| `nros_cli_core::entity_inventory::EntityInventory::from_model` | the same three **plus `contracts.node_paths`** |

`from_model` says why it includes the fourth, and the reason is right: the
contract expresses a timer as `paths: … trigger: { timer: { rate_hz: N } }`,
which the resolver flattens into a `contracts.node_paths` entry with an empty
`input`, NOT into `structure`. "A component whose only callback is a timer
describes real wiring."

`describes_wiring` does not, so a system whose contract declares only timers
reads to it as if nobody authored anything — and it abstains on a service-server
count the contract does state (as zero).

## Measured 2026-09-06

A one-node contract with a single timer path and no topics, services or actions,
beside `examples/workspaces/cpp/src/demo_bringup/launch/system.launch.xml`:

```
$ nros-launch-resolve tmp/timeronly/launch/system.launch.xml -o model.yaml
structure: topics 0  services 0  actions 0
contracts.node_paths 1

$ nros ws entity-inventory --metadata … --model model.yaml
NROS_EXECUTOR_MAX_CBS=1            <- derived; the timer is seen
NROS_EXECUTOR_ACTION_CLIENTS=0

$ nros ws entity-facts --model model.yaml
NROS_DECLARED_INFRA_QUERYABLES=none
                                   <- NROS_DECLARED_SERVICE_SERVERS is ABSENT
```

Same model, same authored contract, two different answers to "did anyone state
this system's wiring?".

## Why it is low severity today

No such contract exists in the tree: all 5 `*.contract.yaml` (the cpp workspace)
declare topics, services or actions alongside their timer paths, so both
predicates agree on all 5. This is a trap for the next contract author, not a
live defect.

## Cost when it fires — READ from the cmake, not measured

No timer-only contract exists to measure, so this half is traced rather than
observed. `entity_facts` abstaining is not free — `NanoRosEntityFacts.cmake` sets
`NROS_ENTITY_SERVERS_UNKNOWN` for the WHOLE configure the moment one entry
abstains (the shared staticlib holds the largest, and an unknown is not smaller
than anything), so `nros-zpico-build` falls back to `UNDECLARED_HEADROOM`
instead of the number the contract stated. One timer-only system in a workspace
therefore costs every image in it the queryable headroom phase-392 W5 exists to
delete.

## Work

1. Give the two consumers ONE predicate. `from_model`'s is the correct one — a
   contract that states only timers still states the system — so lift it to a
   shared helper (`ros_launch_manifest_model` side or `nros_cli_core`) and have
   `describes_wiring` call it, rather than adding a second spelling of the
   `node_paths` test. Second spellings of this exact test are how the repo got
   here.
2. Reconcile the stale prose while there: `EntityInventory::merged_per_kind_max`'s
   doc comment still says "The MODEL has no timer entity", which
   `from_model`'s own comment 160 lines above records as read-once and untrue.
3. A regression test with a timer-only model, asserting both consumers answer.

Found while resolving issue 0973 (2026-09-06). Filed rather than fixed there:
0973's change is documentation, and this one changes what a build derives.
