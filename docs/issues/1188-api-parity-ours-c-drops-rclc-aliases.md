---
id: 1188
title: "`check-api-parity` filtered OUR C surface to `nros_` only, so the rclc-compatible entry points we ship under upstream's exact spelling read as theirs-only gaps"
status: open
type: bug
area: api, tooling, ci
severity: medium
found: 2026-09-07
related: [1040, 1042, 1091, 0417]
---

# 14 symbols whose names match upstream character for character, reported as gaps

`check-api-parity` (`api-parity.py --check --require-disposition`) is RED on a
clean `origin/main` worktree — 50 rows with no ledger disposition. It is on
`ci-l1`, which no merge-gating event runs (issue 1040), so it drifted unseen.

Of those 50, **14 are not gaps at all**. `ours_c()` extracted our C surface with

```python
prefixes={"nros_", "NROS_"},
```

and `correlate.LIB_PREFIXES["c"]` stripped only those on our side. But `nros-c`
deliberately ships rclc-COMPATIBLE entry points under upstream's exact spelling
— the whole `rclc_executor_*` family, `rclc_support_fini` — 25 of them in
`nros_generated.h`. A `nros_`-only filter drops every one from our surface, and
the upstream symbol then has nothing to meet:

| | `rclc_executor_spin` |
| --- | --- |
| theirs, normalized | `executor_spin` (prefix stripped) |
| ours, normalized | `rclc_executor_spin` (prefix NOT stripped, and not extracted at all) |

So a symbol we implement with an identical name was counted as something we do
not have.

## Fixed

`rclc_`/`RCLC_` added to the ours-side extraction prefixes and to the ours half
of `LIB_PREFIXES`, so both sides normalize the same way.

| | before | after |
| --- | --- | --- |
| C lane `same` | 39 | **59** |
| C lane theirs-only | 416 | **396** |
| rows with no disposition | 50 | **36** |

Negative control: reverting only the extraction widening puts it back to 50.
`api-parity.py --self-test` and `gen-api-comparison.py --self-test` both pass.

## Still open — 36 rows, and why they are not fixed here

Two ours-only (`LifecycleTransition`, `shutdown_transition_for`, from
`0747a4c34 fix(#1099)`) and 34 theirs-only, in clean families:

| family | members | what we ship instead |
| --- | --- | --- |
| `*_get_default_options` | action_client, action_server, client, node, publisher, service, subscription | `nros_*_options_t` + `*_init_with_options`; no getter, our options carry no allocator |
| `*_is_valid` | client, publisher, service, subscription | `nros_*_state_t` enums (we DO ship `nros_{clock,executor,guard_condition,support,timer}_is_valid`) |
| `*_init` / `*_fini` | client, node, publisher, service, subscription; node_fini, clock_fini, guard_condition_fini, timer_fini | mixed — `nros_{node,service,subscription}_init` exist, `nros_{publisher,client}_init` do not |
| accessors | node_get_name/namespace, publisher/subscription_get_topic_name, client/service_get_service_name | `nros_node_get_fully_qualified_name`, `nros_node_resolve_name` |
| ROS time override | enable/disable/set/is_enabled, clock_time_started | only `nros_get_ros_time_override` |
| misc | publisher_assert_liveliness, timer_cancel, timer_reset | — |

Not ledgered here for two reasons. First, at least one is unexplained rather
than merely undecided: `nros_node_init` IS in the header and `node_init` is
still reported theirs-only with no matching row printed, so the classification
does not follow from the surfaces as I understand them — and a confidently wrong
disposition aims the next reader at a dead end, which is the lesson issues
0859-0862 paid for. Second, these are statements of intent about the C API, and
`LifecycleTransition` / `shutdown_transition_for` belong to phase-417's
in-flight campaign; authoring dispositions underneath it would conflict with the
people who know the answer.

The ledger's own drift is already tracked by 1042 and 1091, and the reason it
went unnoticed by 1040.
