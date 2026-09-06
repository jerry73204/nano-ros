---
id: 1163
title: "`compile-smoke` checks `nros-c` under its DEFAULT features, so a duplicate `#[no_mangle]` in the shipped feature set reached main green — and `api-parity`, which refuses the header, reports only post-submit"
status: open
type: bug
area: ci, c
severity: high
related: [1040, 1059, 1080, 1102, 1162, 0417, RFC-0061]
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

Fix for the symptom: PR #630 (issue 1162, which also asks for a static
`no_mangle`-uniqueness gate — complementary to this one, which is about the
feature shape the lanes compile). This issue is the CI gate.

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

## Progress

**2026-09-07** — items 1 and 3 landed; item 2 stays open.

- (1) `compile-smoke` now runs `cargo check -p nros-c -p nros-cpp
  --no-default-features --features "{{C_API_SHIPPED_FEATURES}}"` (25 s warm).
  The feature list has ONE spelling, `C_API_SHIPPED_FEATURES` in
  `just/check.just`, read by `check-c`, `check-cpp` (both sites), the nros-c
  clippy combo and `compile-smoke` — the four literal copies in
  `just/check/lanes.just` are gone. Negative control: a second
  `#[no_mangle] nros_node_get_fully_qualified_name` inside an `rmw-cffi`
  region makes `just check compile-smoke` fail with E0428 (in the commit).
- (3) `check-lane-contracts` gains the rule. Subjects are DERIVED: every
  crate under `packages/` with a non-empty default feature set that every
  consumer takes `default-features = false` (measured: `nros-c` — 3
  consumers, its `path = "."` dev-dep excluded — `nros-cpp` — 2 — and
  `nros-board-nuttx`, exempt with a reason as cross-only). The shipped shape
  is authored once, as the `just` variable the lanes read (`SHIPPED_SHAPES`
  in the script), and some recipe reachable from a merge-gating lane
  (`pull_request`/`merge_group`, derived from the workflows) must compile the
  crate `--no-default-features --features` that variable (or a literal equal
  to it; a different literal is drift, and two spellings of the variable is a
  violation). Removing the compile-smoke line on the real tree reports both
  crates; the selftest carries that control plus the schedule-only,
  no-`--no-default-features`, undeclared-subject and two-spellings cases.
  Along the way: the gate's justfile parser did not follow `import`, so since
  phase-399 every gate in `just/check/*.just` was invisible to it —
  `check::compile-smoke` was not a recipe and the closure of `check::fast`
  was two forwarders. It follows imports now; 0 new findings on the real
  tree, lane invocations resolved 13 → 15.
- The `nros-node` `int_plus_one` lint at `executor/tests.rs:3438` that kept
  `check-test-targets` red for an unrelated reason is fixed (separate
  commit).
- (2) — `check-api-parity` on `pull_request`/`merge_group` — waits on #557's
  CI image; not attempted here.

**2026-09-07** — item 2 attempted; BLOCKED twice over, both prerequisites
measured rather than assumed. Neither is "add the step".

- The image #557 waited on was never going to arrive. `images.yml` had
  failed on every push to `main` since 2026-09-04 (`436035894` made the
  Dockerfile COPY two repo-relative files while the workflow's build context
  was still the Dockerfile's own directory), so `ghcr.io/newslabntu/
  nano-ros-ci:humble` was 2026-08-29 content with no `clang` — issue 1201,
  fixed and archived alongside this note; the fixed workflow was dispatched
  from the fix branch and the published image probed for `clang` before the
  PR was opened.
- `just check api-parity` is RED on `main`, and identically red in the
  post-submit lane (51 `UNLEDGERED` rows, local run and CI run `diff`
  IDENTICAL): 48 `theirs-only` rows in `c vs rclc+rcl` (`node_init`,
  `executor_spin`, `timer_cancel`, `publisher_init`, every
  `*_get_default_options`, the ROS-time-override family, …), plus
  `cpp:LifecycleTransition`, `cpp:shutdown_transition_for` and
  `rust:throttle_decide`. Red since `8dd7b3bc4` (2026-09-06 07:04) put
  `--require-disposition` on the gate. PR #629 touches all 17 shards and
  ledgers NONE of these 51. Per the rule gate.yml already states — a lane
  cannot start gating pull requests while it is red — the step is NOT added
  until those rows carry dispositions. The exact list is one `just check
  api-parity` away; it needs an author who can write the verdicts, not a
  CI edit.

Once both hold, the step is the `node-std-tests` shape in `gate.yml`'s
`check` job on `pull_request` + `merge_group`; the post-submit `api-parity`
job then duplicates it and goes (its apt-install of `clang` is redundant the
moment the image carries it).

