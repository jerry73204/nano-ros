---
id: 1188
title: "`check-api-parity` filtered OUR C surface to `nros_` only, so the 71 rcl/rclc-compatible entry points we ship under upstream's exact spelling read as theirs-only gaps"
status: resolved
type: bug
area: api, tooling, ci
severity: medium
found: 2026-09-07
resolved: 2026-09-07
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

`rclc_`/`RCLC_` **and `rcl_`/`RCL_`** added to the ours-side extraction prefixes
and to the ours half of `LIB_PREFIXES`, so both sides normalize the same way.

The `rcl_` half was found by finishing the sweep rather than stopping at the
first family: `nros-c` also defines 42 entry points under rcl's own spelling
(`rcl_node_get_name`, `rcl_timer_cancel`, `rcl_publisher_is_valid`,
`rcl_set_ros_time_override`, the `rcl_get_zero_initialized_*` family), and they
were invisible for exactly the same reason. Fixing only `rclc_` would have left
29 of the 34 remaining rows reading as gaps for symbols we ship character for
character.

| | before | after `rclc_` | after `rcl_` too |
| --- | --- | --- | --- |
| C lane `same` | 39 | 59 | **101** |
| C lane theirs-only | 416 | 396 | **354** |
| rows with no verdict | 50 | 36 | **7** |

Negative control: reverting only the extraction widening puts it back to 50.

## The 7 real rows, ledgered

What survived the tool fix was genuinely undispositioned, and small enough to
decide with evidence rather than by category:

**`c:{node,publisher,subscription,service,client}_init`** — `declined` /
`adopt`. rcl's `_init` IS its options-taking constructor; ours spells that pair
`_init_default` / `_init_with_options` across all five entities, and rclc's
default form ships under rclc's exact name. Node is the one where the absence was
a deliberate act: `nros_node_init` existed as a `static inline` forwarder and
phase-417 stage 6 step B retired it, because the rename landed together with a
parameter PERMUTATION and in C an incompatible pointer argument is a WARNING,
not an error — so deleting the identifier is what makes a stale call fail on the
NAME, which C does diagnose fatally.

**`cpp:LifecycleTransition`** — `extension`, mirroring `rust:LifecycleTransition`.
rclcpp takes a raw `uint8_t`; the constants a porting user reaches for live in
`lifecycle_msgs`, an interface package, so there is no rclcpp type to correspond
to. Half of what `cpp:Transition` records as OUR LANGUAGES DISAGREE — and not
rclcpp's `Transition` class, which also carries `start_state`/`goal_state`/`label`
and is still that row's gap.

**`cpp:shutdown_transition_for`** — `extension`. REP-2002 gives shutdown three
ids, one per legal source state; rclcpp resolves that inside `rcl_lifecycle` and
exposes no name for it. Ours is public and `constexpr` because issue 1099 found
`shutdown()` sending `5` unconditionally and therefore unable to shut down an
Inactive or Active node.

Rows were inserted in alphabetical position as text, never by re-serialising the
shard: `json.dumps(..., sort_keys=True)` rewrote 336 lines for a one-key change
earlier in this campaign, and issue 1091 records this ledger as the largest
PR-conflict cluster left. The four shards move 33 lines total.

## Acceptance

```
just check api-parity          rc=0 — "every divergence carries a ledger entry"
just check api-parity-ledger   self-test 0 failed (shard placement included)
just check fast                256 ran, 1 ledger skip, 0 failed
```

The ledger's own drift is already tracked by 1042 and 1091, and the reason it
went unnoticed by 1040.
