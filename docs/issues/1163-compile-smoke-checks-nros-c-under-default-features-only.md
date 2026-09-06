---
id: 1163
title: "`compile-smoke` checks `nros-c` under its DEFAULT features, so a duplicate `#[no_mangle]` in the shipped feature set reached main green — and `api-parity`, which refuses the header, reports only post-submit"
status: open
type: bug
area: ci, c
severity: high
related: [1040, 1059, 1080, 1102, 0417, RFC-0061]
found: 2026-09-06
---

# The PR gate compiled the C crate in a shape nothing ships, and the shape that ships was red on main for a morning

## Measured

PR #329 (phase-417, rebase-merged 2026-09-06 07:44Z) landed
`packages/api/nros-c/src/node.rs` with `nros_node_get_fully_qualified_name`
defined TWICE — main's #567 four-argument form and the branch's stage-5
three-argument form, both `#[unsafe(no_mangle)]`, both surviving the restack.
On that main (`2bcdfa5d0`, still carrying it):

```
cargo check -p nros-c                                                      # green (default = panic-platform)
cargo check -p nros-c --no-default-features --features std                 # green
cargo check -p nros-c --no-default-features --features rmw-cffi            # E0428
cargo check -p nros-c --no-default-features --features std,rmw-zenoh       # E0428
cargo check -p nros-c --no-default-features \
    --features std,rmw-cffi,platform-posix,ros-humble                      # E0428 — the shape `check-c` builds
```

The second definition sits in a region only an RMW feature compiles. The
required `CI` context on a pull request runs `compile-smoke` =
`cargo check --workspace --all-targets` under default features
(`just/check/lanes.just:336`); its own comment already says "no feature
combinations". The merge queue's `test-unit` compiles the same default shape.
So neither could see it, and the first thing that did was a human running the
C build locally.

`check-api-parity` did refuse it, and said so at the right layer — its C
extractor parses the committed `nros_generated.h` with clang, and that header
carries both prototypes (`conflicting declaration` at `nros_generated.h:5859`).
But since issue 1040 the gate lives on `post-submit.yml`, after the merge:
its job went `failure` on every main run since `4ad88ef86` (07:49Z), which is
a report, not a gate. #557 adds `clang` to the CI image and leaves a
placeholder in `gate.yml` (line 789) saying the step is "NOT here yet".

Fix for the symptom: PR #612 (`7b3bab8e9`). This issue is the gate.

## Why no other lane covers it

- `check-fast` is buildless by contract (RFC-0061).
- `compile-smoke` is PR-only and default-features-only, by its own comment.
- `check-build` (which runs `check-c`, the lane that compiles the shipped
  shape at `lanes.just:71` and `:364`) is `schedule`/`workflow_dispatch` only
  since phase-396 — it needs artifacts no merge-gating job builds.
- `test-unit` on `merge_group` compiles the workspace under default features;
  same blind spot one lane later.
- Issue 1080 / #1102 closed the `--no-default-features` half of this class on
  the batch lane for `nros-node`. This is the other direction — a crate whose
  default (`panic-platform`) compiles almost nothing of its surface, and
  whose shipped shape is a specific positive feature set that every consumer
  selects with `default-features = false`.

## What would close this

1. `compile-smoke` adds one `cargo check -p nros-c --no-default-features
   --features std,rmw-cffi,platform-posix,ros-humble` (and the same for
   `nros-cpp`, which `check-cpp` builds in the same set at `lanes.just:364`).
   Seconds warm. The feature list must be read from the ONE place `check-c`
   reads it, not copied — a second spelling drifts.
2. `check-api-parity` becomes a step on `pull_request` and `merge_group` in
   `gate.yml` once #557's image lands. It is offline, 28.6 s, no fixture, no
   SDK — it satisfies `check-lane-contracts` as written. Its failure mode
   here (the header will not parse) is a stronger signal than the ledger
   verdicts it was written for, and it is the one that fired.
3. `check-lane-contracts` gains the rule: a crate whose consumers all take it
   with `default-features = false` is compiled in its documented shipped
   feature set by at least one merge-gating lane. Without the rule, (1) is
   the next fix that lands only at the reported site.

## Not covered

The merge queue was NOT blocked by this — the batch that merged over the
duplicate passed, because nothing in it compiles the shipped shape. That is
the point: green with no signal capacity, the same class the pitfall index
describes for uniformly-red lanes, seen from the other side.
