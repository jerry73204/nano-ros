---
id: 1101
title: "`use_sim_time_attaches_and_detaches_the_clock_source` fails in the suite and passes alone — the `/clock` armed flag is a process global"
status: resolved
type: bug
area: core, testing
severity: medium
found: 2026-09-05
related: [1076, 0952, phase-425]
---

# A red on `main` that only the full suite sees

```
$ cargo test -p nros-node --lib --features std,sim-time,param-services
executor::tests::use_sim_time_attaches_and_detaches_the_clock_source --- FAILED
  panicked at packages/core/nros-node/src/executor/tests.rs:1860:5:
  installed and not armed would drop every sample
test result: FAILED. 344 passed; 1 failed; 1 ignored
```

```
$ cargo test -p nros-node --lib --features std,sim-time,param-services \
      use_sim_time_attaches
test result: ok. 2 passed; 0 failed
```

Reproduced 2/2 in the suite and 0/2 alone, on **plain `origin/main`** — not on
any branch. Also reproduced on `fix/1050-rmw-selector-baked-rung` and on
phase-429's branch, which is how it was found: it is the only `check::build`
red left there once issue 1076 is fixed.

## The mechanism

The failing assertion is the second of the pair:

```rust
assert!(executor.ros_time_source_installed(), …);   // passes
assert!(crate::time_source::is_active(),           // FAILS in-suite
        "installed and not armed would drop every sample");
```

`installed` is executor state; `is_active` is not:

```
packages/core/nros-node/src/time_source.rs:46
static SIM_TIME_ACTIVE: AtomicBool = AtomicBool::new(true);
```

A **process global**, shared by every test in the binary, which `cargo test`
runs on multiple threads by default. Some other test in the suite clears it —
plausibly the "detaches" half of this same test, or another `sim-time` case —
and this one observes the cleared value. The default `true` is why it passes
alone: nothing has run to clear it yet.

So the test is order- and concurrency-dependent, and its subject is a global.

## Why it matters beyond one red

This is CLAUDE.md's uniformly-red-lane hazard forming: `check::build` runs on
`schedule` / `workflow_dispatch` only, never on `pull_request` or
`merge_group`, so nothing merge-gating observes it. A second regression landing
in `node-std-tests` would look exactly like this one.

It is also the same shape as issue 1076 — a red on `main` in a lane the queue
cannot see — found the same way, by a branch that had to run the tier locally.

## Not diagnosed here

**Which test clears the flag.** That wants a bisect over test names
(`--test-threads=1` plus a targeted subset), not a guess. The two obvious
candidates are the detach half of this test and any other `sim-time` case in
the file.

## Fix direction

1. Establish the interfering test, by name, before changing anything.
2. Then choose: make the flag per-executor rather than process-global (right if
   two executors in one process should be able to disagree), or serialize the
   `sim-time` tests against a shared lock (right if the global is genuinely a
   process-wide fact). The first is a design answer, the second an admission —
   pick deliberately, because a `#[serial]` attribute on a test whose subject
   SHOULD be per-instance hides the defect rather than fixing it.
3. Whichever wins, the test must fail when the flag is not armed, so keep the
   assertion.

## Resolution — FIXED, as issue 1104

**This is the same defect as issue 1104**, filed independently the same day by
another session. Same test, same assertion, same mechanism: `time_source`'s
armed flag is a process-global `AtomicBool` while an `Executor` is per-test, and
`reconcile_ros_time_source` writes it from `spin_once`.

Fixed on `main` by `SimTimeGuard` in `packages/core/nros-node/src/executor/
tests.rs` — a mutex both sim-time-aware tests take, restoring on drop the value
OBSERVED ON ENTRY rather than a guessed default. The clobbering neighbour turned
out to be `ros_time_timer_follows_the_simulated_clock`, whose own doc comment had
already reasoned about the hazard and solved half of it ("One test rather than
four, deliberately... so four tests would race each other inside the one test
binary") — which handles those four and nothing about a fifth test landing beside
them.

Measured there, both directions: 349 passed 0 failed over 5 runs; and with the
guard removed from the NEIGHBOUR only, the original failure returns 3 of 3.

Read `docs/issues/archived/1104-node-std-tests-red-time-source-process-global.md`
for the full record, including a wider product guard that was tried and REJECTED
because a mutation test showed the lock alone was sufficient.

**Note for the next reader:** 1104 also records that under `--test-threads=1` a
DIFFERENT test fails for an unrelated reason (a 1 ms window asserted off a
free-running process clock) — that is issue 1105, not this one.

No further work is needed here; this doc is archived so the defect is not fixed
twice.
