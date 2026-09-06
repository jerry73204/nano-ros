---
id: 1135
title: "four `params.rs` ROS 2 tests report PASS when the nano-ros node is not discoverable — a bare `return`, not `skip!`"
status: open
type: bug
area: testing, rmw
severity: high
found: 2026-09-06
related: [1127, 0445]
---

# The precondition they exist to check is the one they swallow

`packages/testing/nros-tests/tests/params.rs` — `test_ros2_param_list`,
`test_ros2_param_get`, `test_ros2_param_set`, `test_ros2_param_describe`, at
lines 189, 232, 276 and 323 — each open with:

```rust
if !require_node_discoverable(&locator) {
    return;
}
```

A bare `return` from a `#[test]` is a **PASS**. `require_node_discoverable`
polls `ros2 node list` three times for `/demo/talker` and prints
`"Skipping test: nros node /demo/talker not discoverable…"` on failure — but
printing is all it does, so nextest records four green tests.

CLAUDE.md's rule is explicit: *tests must fail on unmet preconditions;* bare
`eprintln!` + `return` reports PASS — never. The correct spelling
(`nros_tests::skip!`) is already used in this same file, at line 514.

## Why this one is worse than the general class

The swallowed condition is not an absent toolchain. It is **"ROS 2 is up, the
router is up, the fixture is built, and our node is not in the graph"** — which
is a *delivery failure*, the exact thing these four tests exist to detect. On
any host where the feature is broken but the environment is fine, all four go
green.

Under `--failure-output never` (what `just native test-ros2-params` passes) the
`eprintln!` is not even shown, so the run is silent as well as green.

`check-no-vacuous-tests` cannot see it: that gate keys on a test body whose only
effects are PRINTS, and these bodies do plenty on the passing path. The gate is
correct; the shape is different.

## Fix

One line per site — `return;` becomes
`nros_tests::skip!("nros node /demo/talker not discoverable via ros2 node list")`,
or `require_node_discoverable` itself skips instead of returning `bool`, which
removes the possibility at the source. Prefer the second: a helper that returns
`bool` invites this at every future call site, and it currently has four.

## The class question

Worth a sweep before closing: `grep -n 'return;'` across
`packages/testing/nros-tests/tests/` for other guard-shaped bare returns. This
one was found by reading, not by a gate, and the gate that would catch it
(`no-vacuous-tests` extended to "a `#[test]` that returns early on a
probe result") does not exist. Whether to write it is the durable half of this
issue.

## How it was found

The phase-433 W2 pre-flight (PR #592), which asked of every live-peer test
binary "would a skip be visible?" before spending
box time running them. `params` is the one binary in the set that cannot
produce a verdict today.
