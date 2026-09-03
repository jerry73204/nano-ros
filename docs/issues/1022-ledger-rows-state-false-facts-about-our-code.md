---
id: 1022
title: "~95 parity-ledger rows state something FALSE about our own code, and several
  justify a divergence with a constraint the compared surface refutes"
status: open
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
(2026-09-04), each verified against the headers before filing.

| # | claim | reality | rows |
| --- | --- | --- | ---: |
| 1 | "there is no runtime options struct" | seven ship — `nros_{node,publisher,subscription,service,client,action_client,action_server}_options_t` — plus seven `*_get_default_options`, and the correlator buckets all fourteen `same`. `nros_node_options_t` carries a runtime `domain_id_override` | 27 |
| 2 | the C zero-copy types are `nros_borrowed_str_t` / `nros_borrowed_bytes_t` | neither exists; they are `nros_view_str_t` / `nros_view_bytes_t` (`view.h:39,45`). Same class as 1012, found outside its scan | 28 |
| 3 | `get_actual_qos` "would return its own input" because QoS is compile-time | false. `Node::set_qos_overrides` (`node.hpp:292`) installs a launch-lowered table that `apply_qos_overrides` folds into every entity created afterwards. Actual ≠ requested exactly when a deploy plan says so — the accessor's whole use case. Should be `gap`, not `declined` | ~10 |
| 4 | `cpp:` rows answering in C spellings | `nros_count_status_t` / `nros_liveliness_changed_status_t` are the C types; C++ has three distinct ones (`nros_cpp_{pub_count,count,liveliness_changed}_status_t`, `nros_cpp_ffi.h:441,513,524`), so both the names and the count are wrong | 15 |
| 5 | verdict inflation — `declined` on rows whose own `why` describes a capability we HAVE under another spelling ("ours is `nros::spin_once(0)`") | those are `rename` / `remapped` by the ledger's own definitions | ~14 |
| 6 | `cpp:ServerGoalHandle` maps onto `nros_action_succeed`/`_abort`/`_canceled` | declared only in `nros-c/include/nros/nros_generated.h:3896`; **no nros-cpp header includes it**. The C++ answer, `ActionServer::complete_goal` (`action_server.hpp:257`), is never named | 1 |
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
This is RFC-0087's compile-and-differ shape, in a comment rather than a
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

Homed in phase-417 (correction track).
