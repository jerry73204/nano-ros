---
id: 1183
title: "`check node-std-tests` is red on main — the executor-backing latch test asserts a process-global that 367 sibling tests share"
status: open
type: bug
area: testing
related: [phase-392, issue-1145]
---

## Problem

`just check node-std-tests` fails on `main`, reproducibly:

```
cargo test -p nros-node --lib --features std,sim-time,param-services --quiet
...
---- executor::backing::tests::the_static_is_handed_out_once_and_only_once stdout ----
panicked at packages/core/nros-node/src/executor/backing.rs:213:9:
an over-large request must not claim the latch
test result: FAILED. 365 passed; 1 failed; 1 ignored
```

Measured 3/3 on `bd204b986` with no local changes (`git stash` of every touched
file, three consecutive runs, identical result). It is NOT caused by whatever
change you are holding — check this issue before spending time on it.

## Why it fails

`EXECUTOR_BACKING` is handed out ONCE per process, guarded by a `TAKEN` latch.
The test's own doc comment says so and explains that all three cases live in one
test for exactly that reason: *"`TAKEN` is process-scoped by design, so a second
test would observe an already-consumed latch and assert nothing — the vacuous
shape `check-no-vacuous-tests` exists to catch."*

The reasoning is right and one scope too small. `cargo test` runs the whole
`--lib` suite in ONE process on a thread pool, and every sibling that builds an
`Executor` through the static path claims the same latch. So the test races 367
other tests for a resource it must observe untouched, and the assertion that
loses is the second one — `!is_taken()` after a refused over-large request.

The suite is green under **nextest**, which gives each test its own process
(`cargo nextest run -p nros-node --features std,alloc --lib` — 339 passed).
That is why `test-unit` does not see this and the `node-std-tests` lane does:
the lane is the one place in the tree that runs `nros-node`'s unit tests through
plain `cargo test`, and it does that deliberately, to reach the seven
`cfg(feature = "std")` tests `--workspace` never builds (its own comment
records the four days that cost).

## Fix, and the shape it has to keep

Not "delete the test" and not "serialise the suite". The test is a positive
control for a module that would otherwise be inert while reading as working, so
it has to keep observing a virgin latch. Either:

* run this lane under `nextest` (process per test) with a filter that still
  reaches the std-only tests, or
* give the test its own cargo invocation, the way the same recipe ALREADY does
  for `boot_report::tests` — that block's comment explains the identical defect
  ("the record is a process-global static... Sharing a binary means sharing a
  process... Two cargo invocations are two processes") and then does not apply
  the lesson one function down.

The second is a two-line change and reuses a pattern the recipe already
documents.

## Signal cost while it stands

A lane that is red for every input has no signal capacity — a regression landing
in it looks exactly like this failure. `node-std-tests` is in the derived
fast-serial gate list, so that is the whole lane.
