---
id: 1022
title: "Parity-ledger rows state FALSE facts about our own code, and several justify
  a divergence with a constraint the compared surface refutes"
status: resolved
type: bug
area: docs, api
related: [phase-379, phase-417, issue-1012, issue-1019, issue-1020, rfc-0036, rfc-0087]
---

## Problem

Issue 1012 covers rows naming a symbol that no longer exists — a dead spelling,
fixed by renaming the prose. This is the worse class: the symbol exists and the
sentence about it is **wrong**. A dead spelling makes a reader look something
up and fail. A false claim makes them act.

Found by three independent audits of the C, C++ and cross-language surfaces
(2026-09-04). **The counts in the table below were the audits' estimates and
several were too high** — the implementing pass measured them row by row and
the corrected figures are in the last column. That pass also refuted one claim
outright; see "Corrections to this issue" at the end.

| # | claim | reality | est. | actual |
| --- | --- | --- | ---: | ---: |
| 1 | 1 | "there is no runtime options struct" | seven ship — `nros_{node,publisher,subscription,service,client,action_client,action_server}_options_t` — plus seven `*_get_default_options`, and the correlator buckets all fourteen `same`. `nros_node_options_t` carries a runtime `domain_id_override` | 27 | **15** |
| 2 | the C zero-copy types are `nros_borrowed_str_t` / `nros_borrowed_bytes_t` | neither exists; they are `nros_view_str_t` / `nros_view_bytes_t` (`view.h:39,45`). Same class as 1012, found outside its scan | 28 | 28 |
| 3 | `get_actual_qos` "would return its own input" because QoS is compile-time | false. `Node::set_qos_overrides` (`node.hpp:292`) installs a launch-lowered table that `apply_qos_overrides` folds into every entity created afterwards. Actual ≠ requested exactly when a deploy plan says so — the accessor's whole use case. Should be `gap`, not `declined` | ~10 | **13** |
| 4 | `cpp:` rows answering in C spellings | `nros_count_status_t` / `nros_liveliness_changed_status_t` are the C types; C++ has three distinct ones (`nros_cpp_{pub_count,count,liveliness_changed}_status_t`, `nros_cpp_ffi.h:441,513,524`), so both the names and the count are wrong | 15 | **5** |
| 5 | verdict inflation — `declined` on rows whose own `why` describes a capability we HAVE under another spelling ("ours is `nros::spin_once(0)`") | those are `rename` / `remapped` by the ledger's own definitions | ~14 | **5** |
| 6 | `cpp:ServerGoalHandle` maps onto `nros_action_succeed`/`_abort`/`_canceled` | the row answers a C++ question with C functions and never names the C++ answer, `ActionServer::complete_goal` (`action_server.hpp:257`) + `for_each_active_goal` (`:284`) | 1 |
| 7 | `cpp:Node::set_parameter` — "no setter at all, not even a typed one" | `ParameterServer::set_parameter<T>` exists (`parameter.hpp:363`). The true, narrower claim is: no setter on the NODE facade | 1 |
| 8 | `c:log_severity_t` — "`<nros/nros.h>` does not `#include "nros/log.h"`" | it does, unconditionally, at `nros.h:40` | 1 |

## The one that is a defect in shipped documentation, not the ledger

`nros_generated.h:840` says `nros_ret_t` is *"Compatible with `rcl_ret_t` for
familiarity."* Only `OK` agrees:

| | ours | rcl |
| --- | ---: | ---: |
| OK | 0 | 0 |
| ERROR | −1 | 1 |
| TIMEOUT | −2 | 2 |
| INVALID_ARGUMENT | −3 | 11 |
| NOT_INIT | −7 | 101 |

Ported code writing `if (ret == RCL_RET_TIMEOUT)` compiles and never matches.
This is RFC-0089's compile-and-differ shape, in a comment rather than a
signature.

## The systematic one, and the reason this is worth a separate issue

Rows 1 and 3 share a shape with a larger family: **a divergence justified by a
constraint that the compared surface refutes.**

The canonical case is byte-oriented delivery. Many C rows justify it with "no
allocator". rclc has no allocator on that path either and still delivers a
deserialised message, by making the caller own the storage:

```c
rclc_executor_add_subscription(executor, subscription, void *msg,
                               rclc_subscription_callback_t callback, invocation);
typedef void (*rclc_subscription_callback_t)(const void *);
```

Ours has no `msg` slot and delivers `(const uint8_t *, size_t)`. The generated
`<Msg>_deserialize` already writes into caller storage and the typed *publish*
half already shipped — only the receive half was never built. So the divergence
is real and the stated reason is not, which is worse than an unexplained row:
it forecloses the fix.

RFC-0036 already forbids recording a preference as a divergence. This is the
adjacent failure — recording a real divergence under a false cause — and the
catalog has no rule against it.

## Fix

Correct the rows, and prefer deleting a justification to keeping a wrong one.
Where a verdict changes (rows 3 and 5), that is a ledger edit plus a
regenerated comparison, not code.

Then close the class: RFC-0036 should require that a `divergence`'s stated
constraint be one the COMPARED surface does not also operate under. That is
checkable by hand at review time and would have caught rows 1, 3 and the
allocator family.

## Corrections to this issue (2026-09-04, from the implementing pass)

Filed fast from three audit reports; measuring row by row refuted part of it.
Recorded here rather than silently fixed, because a wrong bug report aims the
next person at a dead end — the same failure this issue is about.

* **Row 6's reachability claim was FALSE.** It said "no nros-cpp header
  includes `nros_generated.h`", from grepping the C++ headers for a direct
  include and finding only comments. The path is TRANSITIVE:
  `nros.hpp` → `clock.hpp` → `nros/clock.h` → `nros/types.h:24` →
  `nros_generated.h`. Verified by compiling `#include <nros/nros.hpp>` plus
  `(void)&nros_action_succeed` — rc=0. The row's real defect stands: it answers
  a C++ question with C functions and never names `complete_goal`.
* **Row 1 is 15 ledger rows, not 27.** The remaining occurrences are not in the
  ledger at all — the same false sentence is the constraint text of the
  `compile-time-options` rule in
  `scripts/api_parity/signature_rules.py:54-62`, which generates rows rather
  than storing them. Fixing the ledger there would have been whack-a-mole.
* **Row 4 is 5 rows, not ~15.** The seven `*CallbackType` / `*EventCallbacks`
  rows were already corrected on 2026-09-04 by phase-379 W6 and now name both
  the C and C++ spellings.
* **Row 5 is 5 rows, not ~14.** The other ~60 `declined` rows carrying a
  `provides` decline a TYPE or a SHAPE while providing the capability, which is
  a legitimate `declined` — not verdict inflation.

Two dangling keys also found: `rust:ClientTrait::server_available` is
referenced three times (`graph.json`, `action.json`) and **has never existed**.

RESOLVED 2026-09-04 (phase-417 correction track). 92 rows corrected across 14
shards; `--check` rc=0. The `compile-time-options` constraint text in
`scripts/api_parity/signature_rules.py:54` was the one that could not be fixed
by a ledger edit — it GENERATES row text — and is corrected in place.
