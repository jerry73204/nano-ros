---
id: 1161
title: "`check-required-features-tests` runs bare `cargo nextest`, so a `skip!` precondition is a FAILURE there and a skip in `test-unit` — and `just ci gate` runs both"
status: open
type: bug
area: testing, ci
severity: medium
found: 2026-09-06
related: [0774, 0652, 0612]
---

# The same panic, the same host, two opposite verdicts in one lane

`just ci gate` on a host with no `ros-<distro>-rmw-zenoh-cpp` installed:

```
check::build   FAILED   required-features-tests
  Summary [0.074s] 20 tests run: 7 passed, 13 failed, 0 skipped
  panicked at fixtures/zenohd_router.rs:560:
  [SKIPPED:capability] no `rmw_zenoh_cpp/rmw_zenohd` found. ...

test-unit      ok
  Summary [0.907s] 1158 tests run: 1155 passed, 3 failed, 2 skipped
  rewrite-skipped-junit: rewrote 3 [SKIPPED] failure(s) to <skipped>
  All failures were [SKIPPED] preconditions — treating as pass.
```

Both numbers count the same `nros_tests::skip!` macro, raised for the same
reason. `test-unit` reclassifies them and passes; `check-required-features-tests`
does not and fails. The lane is 13 `cargo nextest run` invocations
(`just/check/lanes.just:1130-1157`), each bare, with no junit rewrite behind
them.

## Why it matters

CLAUDE.md already records the mechanism -- "Bare `cargo nextest` counts
`nros_tests::skip!` panics as FAILURES; only `just test-all`'s junit rewrite
makes them skips" -- as advice for a human reading a red. Here it is wired into
a lane, so the consequence is structural:

* **`just ci gate` cannot be green on a host without an optional ROS package**,
  and `ci gate` is the documented run-before-every-push tier. A contributor with
  no `rmw_zenoh_cpp` gets a red they cannot fix and did not cause.
* **The lane loses its signal capacity**, which is the class recorded for
  uniformly-red lanes: a real regression in these 13 targets arrives looking
  exactly like today's 13 skips. The targets are there because issues
  0652/0612/0667 found four of them broken and one capability non-functional
  when they finally ran -- signal worth keeping.
* It reads as a REGRESSION to whoever runs it. It is the first thing
  `check::build` reports, and `ci gate` stops at the first failure, so it also
  WITHDRAWS `api-parity`, `test-unit` and `test-lane-contracts` from the run
  (issue 0952's rule) for a reason that is not a defect.

## What it is not

Not issue 0774. That one was a router that RESOLVED but SEGV'd on a mismatched
`libzenohc.so`, on a host that HAS ROS; it is fixed, and the fixture now pins
the pairing. This is the plain absence of `rmw_zenoh_cpp`, which `skip!` is the
correct response to -- the bug is only in how the lane counts it.

## Shape of a fix

Route these invocations through whatever `test-unit` uses (the
`rewrite-skipped-junit` step), rather than teaching each call site. Two things
to get right:

1. **A rewrite that swallows everything is worse than the red.** It must
   reclassify only `[SKIPPED...]` panics and leave a real failure red -- the
   `test-unit` step already does exactly this and prints its count, so the
   number rewritten stays visible.
2. **`0 skipped` should not stay the printed summary.** The `Summary` line is
   what a reader believes; if 13 targets skipped, the lane should say 13
   skipped, or the next person re-derives this issue from the same output.

## Also seen in the same run, separately

`nros-platform-cffi::c_port_posix_timer periodic_timer_fires_repeatedly` failed
with `expected at least 4 fires over 40 ms, got 0` inside the `-P12` build and
passed 3 of 3 solo with the lane's own `--features posix-c-port`. Its sibling
`rust_trait_periodic_fires` passed in the same run. A timing assertion under a
saturated jobserver -- the load-flake class of issue 1159, not this issue's
mechanism, and not filed separately because a lower bound on a 40 ms window is
inherently load-sensitive and the fix is the same conversation.
