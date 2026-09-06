---
id: 1168
title: "`run-gates-parallel.sh` fans out to `nproc` gates and each starts a 25-thread nextest, so a wall-clock assertion measures the load instead of the code"
status: resolved
type: bug
area: testing, ci
severity: medium
found: 2026-09-06
resolved: 2026-09-06
related: [0726, 0838, 1159]
---

# 300 test threads for 12 cores, and nothing knew

`scripts/build/run-gates-parallel.sh` runs `NROS_GATE_JOBS` (default `nproc`)
gates concurrently. Each gate is an independent `just` recipe, and the ones that
run tests start their own `cargo nextest`, which takes its thread count from
`.config/nextest.toml` — **`test-threads = 25`** — and knows nothing about its
eleven siblings.

On this 12-core host that is up to **12 x 25 = 300 test threads for 12 cores**,
plus a `cargo` per gate. Nothing anywhere bounds the total. The runner's own
header records it was measured on a 32-core machine, where the ratio hurt less
and the problem never surfaced.

The 25 is not a performance knob and must not be read as one: it is the Cyclone
DOMAIN partition's slot count (issue 0838 — slot `s` owns domains `[s*4,
s*4+3]`), a correctness bound. It was never a statement about CPUs.

## What it cost

`check-build` went red on `check-required-features-tests`, whose last step is a
bare `cargo nextest run` over `nros-platform-cffi`:

```
FAIL [0.093s] (25/38) nros-platform-cffi::c_port_posix_timer periodic_timer_fires_repeatedly
  expected at least 4 fires over 40 ms, got 0
```

That test creates a 5 ms periodic timer, sleeps 40 ms, and asserts `count >= 4`.
It does not assert that a periodic timer is periodic; it asserts how much work
the machine did in 40 ms.

Reproduced deterministically, 12-core host, 24 busy loops (2x oversubscription):

| condition | result |
| --- | --- |
| solo, idle | 3 of 3 pass |
| 24 busy loops | **2 of 3 fail** (`got 2`) |

Its sibling `rust_trait_periodic_fires` passed in the same in-lane run — same
mechanism, same hazard, it just got scheduled.

## Fixed, in two independent places

**1. The budget, because the fan-out needed one.**
`run-gates-parallel.sh` now derives `NROS_GATE_CPU_SHARE = max(1, nproc / jobs)`
and exports it; `nros_nextest_cpu_budget_args` in `scripts/build/cargo.sh` turns
it into `--test-threads`, spliced through `nros_cargo_nextest_args`, which is
the single seam every nextest caller in the tree already goes through.

Three properties, each deliberate:

* **It only ever LOWERS.** The share is clamped by the config's own value, READ
  from `.config/nextest.toml` rather than copied, because that number is a
  correctness bound on domain isolation. `share=99` yields `--test-threads=25`,
  never 99.
* **It applies only INSIDE a fan-out.** The variable is exported by the parallel
  runner and by nothing else, so `just test-unit`, `just test-all` and a bare
  `cargo nextest` are byte-identically unchanged — they own the machine.
* **A caller's own `--test-threads` still wins.** These args are spliced BEFORE
  the caller's and nextest takes the last occurrence, so
  `just test-select ... --test-threads=1` is unaffected.

Build jobs are deliberately NOT throttled: concurrent cargos sharing a
target-dir already serialize on cargo's build lock, a compile is not
wall-clock-sensitive, and `check-build` is ~587 s serial — capping `-j` would
trade a real slowdown for nothing.

The runner now PRINTS the split, so it is readable rather than inferred:

```
check-fast (parallel): 245 gate(s) OK at -P12 (1 test-thread(s)/gate); slowest rmw-ret-sign 39843ms
```

**2. The assertions, because a budget nobody sets is a budget every future
timing test rediscovers.** All six tests in `c_port_posix_timer.rs` were
`thread::sleep(fixed)` then assert. They now poll to a generous deadline
(`TIMER_DEADLINE`, 2 s — two orders of magnitude above the periods under test,
so it is the point past which the timer is not late but ABSENT).

The lower bounds are kept, not traded away, because a deadline alone would pass
a "timer" that fires in a hot loop:

* the periodic tests additionally assert `elapsed >= 15 ms` for 4 fires of a
  5 ms period. Mutation-verified: period `1 us` reports *"4 fires ... arrived in
  1.059975ms — faster than the period allows"*.
* the two CANCEL tests have the OPPOSITE hazard — the test thread must reach the
  cancel before the timer's own 100 ms deadline — so they measure the window and
  fail naming starvation rather than reporting `cancel must report prevent-fire`
  about the scheduler.
* a missed deadline is a `panic!`, never a skip. A timer that never fired is
  what these tests exist to catch.

## Acceptance

| | before | after |
| --- | --- | --- |
| `c_port_posix_timer`, idle | 6 pass | 6 pass, 0.13 s |
| `c_port_posix_timer`, 24 busy loops | 2 of 3 runs red | **5 of 5 runs green** |
| `just check fast` | 245 OK | 245 OK, `-P12 (1 test-thread(s)/gate)` |
| hot-loop mutation (`period = 1 us`) | passed | **rejected** |

## Left open

`check-required-features-tests` cannot be run to green on this branch for an
unrelated reason: `packages/api/nros-c/src/node.rs` on `origin/main` defines
`nros_node_get_fully_qualified_name` twice (lines 672 and 980), so `nros-c`
fails `E0428` under `--features rmw-cffi`. The fix exists as `7b3bab8e9` on
`origin/fix/phase-417-parent-three-fixes`, which has no open pull request. Not
this issue's mechanism and not fixed here.
