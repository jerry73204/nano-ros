---
id: 1161
title: "The skip budget forbids a missing FIXTURE and permits a missing CAPABILITY, so `check-required-features-tests` reports pass having run 7 of 20"
status: open
type: bug
area: testing, ci
severity: medium
found: 2026-09-06
related: [0584, 0673, 1168]
---

> **Filed premise was wrong, corrected 2026-09-06.** This was filed as "a
> `skip!` is a failure in one lane and a skip in another, and `just ci gate`
> runs both", from reading a raw nextest `Summary` line as the lane's verdict.
> It is not the verdict, and CLAUDE.md says so in as many words: *"Bare
> `cargo nextest` counts `nros_tests::skip!` panics as FAILURES ... `Real
> failures: N` from the junit rewrite counts what the RUN saw."* The lane does
> not use a bare run — it goes through `_nextest-tolerant`, which rewrote the
> 13 and printed `All failures were [SKIPPED] preconditions — treating as
> pass`. The red I attributed to this was issue 1168's timer flake, four steps
> later. What survives is a different and real problem, below.

# A lane that ran 7 of its 20 tests, and said pass

```
check-skip-budget: 7 ran, 0 deselected (out of lane), 13 skipped for an unmet
                   precondition — capability=13
All failures were [SKIPPED] preconditions — treating as pass.
```

Thirteen of twenty tests never executed, on a host with no
`ros-<distro>-rmw-zenoh-cpp`, and `check-required-features-tests` is green. That
is the shape issue 0584 exists to prevent — "a lane greens over a coverage
hole" — and the guard was built and then applied to one of the two classes.

## The rule is already written; it names fixtures only

`scripts/test/check-skip-budget.py` asserts exactly two properties, and the
second is:

> **No skip whose reason is a missing fixture.** Since 0584 part 2 an absent
> in-lane fixture is a hard failure, not a skip.

So a missing FIXTURE fails. A missing CAPABILITY — here `rmw_zenoh_cpp/rmw_zenohd`,
resolved through `AMENT_PREFIX_PATH` — is counted, printed, and tolerated. The
difference is not principled: both are "the thing this test needs is not here",
and both make the run do less than it claims. It is historical, because 0673
introduced the capability tolerance to stop tier 1 going red for an environment
fact, before 0584 established that an unmet precondition should fail.

The cost is the one already recorded for uniformly-red lanes, in the other
direction: **these 13 have no signal capacity.** A regression in any of them
looks exactly like today's skip, and they are worth signal — issues
0652/0612/0667 found four targets broken and one capability non-functional the
first time this lane's targets actually ran.

## What "set the scope precisely" means here

The choice is between two honest states, and the current one is neither:

1. **The lane provides the capability.** `rmw_zenohd` is resolvable through
   RFC-0075's documented order (`NROS_RMW_ZENOHD` -> `AMENT_PREFIX_PATH` ->
   `$ROS_DISTRO` under `/opt/ros`), so a lane that declares it can also
   provision it, and then a red is a red.
2. **The lane does not claim them.** Move the router-dependent targets to a
   lane whose contract states the requirement, and let this one run only what it
   can. Seven tests honestly reported beats twenty with thirteen absent.

Either way the tolerance stops being ambient: a capability skip becomes a
FAILURE by default, and any remaining one needs a declared, checked exemption
the way `check-submodule-pins`' NOT VERIFIED does — a reported skip in a ledger,
not a silent subtraction.

`check-lane-contracts` already encodes the same principle one layer over: *a
gate in an affordability tier may only resolve artifacts the JOB ITSELF builds.*
This is that rule for capabilities rather than artifacts.

## Measured

| | |
| --- | --- |
| host | no `ros-humble-rmw-zenoh-cpp` installed (`dpkg -l` = 0 matches, `/opt/ros/humble/lib/rmw_zenoh_cpp` absent) |
| lane | `check-required-features-tests`, first step |
| ran | 7 |
| skipped `capability` | 13 |
| verdict | pass |

The 13 are in `trigger_conditions`, `wake_latency`, `component_runtime`,
`component_dispatch`, `component_param` and `signal_fd_wake`, all panicking at
`fixtures/zenohd_router.rs:560`.

## Not in scope here

The `nros-platform-cffi` timer red seen in the same run is issue 1168 — a
compute-budget flake, fixed there — not a skip-accounting problem.
