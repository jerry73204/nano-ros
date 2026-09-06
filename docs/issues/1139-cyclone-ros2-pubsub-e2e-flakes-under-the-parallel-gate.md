---
id: 1139
title: "`nros_rmw_cyclonedds_ros2_pubsub_e2e` fails under the 21-way parallel
  gate and passes solo, every time -- 6 failures, 4 solo passes, one session"
status: open
type: bug
area: testing, rmw-cyclonedds
severity: medium
related: [issue-1009, issue-0741]
---

## What happens

`just ci gate` -> `check::build` -> `rmw-cyclonedds` fails on test 20,
`nros_rmw_cyclonedds_ros2_pubsub_e2e`. Re-running the SAME test alone in the
same build dir passes, every time it has been tried.

Measured over one session, 2026-09-05/06, on a tree whose diff contains no
CycloneDDS code at all:

| run | in-gate | which sub-case failed | solo re-run |
| --- | --- | --- | --- |
| 1 | FAIL | A.2 `ros2 pub -> nros sub` timed out | pass |
| 2 | FAIL | A.1 `nros pub -> ros2 echo` captured nothing | pass |
| 3 | pass | -- | -- |
| 4 | FAIL | A.1 | -- |
| 5 | pass | -- | -- |
| 6 | pass | -- | -- |
| 7 | FAIL | A.1 | pass |
| 8 | FAIL | A.1 | pass |

The failing sub-case is not fixed: A.1 and A.2 have each failed, and each has
passed in the same conditions. That is the signature of a timing or contention
problem rather than a broken assertion -- a broken direction would fail the
same way every time.

Wall-clock also varies wildly for the same lane: 35 s, 155 s, 202 s, 220 s,
316 s across the runs above. The 35 s one failed early rather than timing out.

## Why it matters even though it "passes solo"

A lane that is red for a reason nobody has established has no signal capacity.
This repo already records that lesson (`just nightly-triage`, and the note in
CLAUDE.md that "a red CI lane answers one of two questions and they look
identical"). Right now a real CycloneDDS regression landing in this cell would
look exactly like the eighth flake, and the honest response -- rerun it solo --
is also exactly what would hide the regression.

It also costs a full `just ci gate` cycle every time it fires, because the lane
stops at the first failure and withdraws every step after it.

## What has NOT been established

* Whether the contention is CPU (the gate runs `-P20` and this cell starts a
  real `ros2` process pair), the DDS bus, or the discovery timeout.
* Whether it predates the parallel gate's current width.
* Whether the two sub-cases share a cause or fail for different ones.

Nothing here has been bisected. The table above is observation, not diagnosis.

## Where to look first

Issue 1009 is the near neighbour and worth reading before touching this: the
interop DDS bus is pinned to loopback by a PROFILE FILE the harness writes
(`nros_tests::dds_isolation`), and getting that half-right isolates one side of
a pair and measures zero with empty output. If this cell's isolation is
partial, a busy machine is exactly when a foreign participant would win a
discovery race.

`ROS_LOCALHOST_ONLY=1` is NOT the fix and is documented as making it worse.

## Workaround in the meantime

Re-run the lane; it passes about a third of the time. Do NOT read a solo pass
as proof the diff is clean -- it is proof of nothing about the diff, only that
the cell is not deterministically broken.
