---
id: 1186
title: "`the_static_is_handed_out_once_and_only_once` reads a latch its sibling tests have already claimed"
status: open
type: bug
area: core, testing
severity: medium
found: 2026-09-07
related: [0417, 1145]
---

## What

`just check node-std-tests` is RED on `main`:

```
thread 'executor::backing::tests::the_static_is_handed_out_once_and_only_once'
panicked at packages/core/nros-node/src/executor/backing.rs:213:9:
an over-large request must not claim the latch

test result: FAILED. 365 passed; 1 failed; 1 ignored
```

The assertion is the test's own SECOND line:

```rust
assert!(take(EXECUTOR_BACKING_U64S + 1).is_none(), ...);
assert!(!is_taken(), "an over-large request must not claim the latch");
```

`TAKEN` is a process-scoped `AtomicBool` and the message names the wrong
culprit: nothing about the over-large request claimed it. Something ELSE in the
same test binary — 365 other tests, several of which construct an `Executor` —
took the static first, and this test read the latch after them. Its own
doc-comment already knows the hazard ("`TAKEN` is process-scoped by design, so a
second test would observe an already-consumed latch and assert nothing") and
guards against SPLITTING the test, which is not the same as guarding against a
sibling.

Issue 0417's class exactly, one module over: process-global statics shared by
tests in one binary, where the verdict depends on nextest/libtest scheduling.

## Measured

* On this tree, `just check node-std-tests` fails identically with the working
  tree stashed clean and with a change to `packages/cli/**` applied — so it is
  not a regression from either.
* Two consecutive runs failed the same way, so it is at least reliably red here
  rather than a rare interleaving.
* NOT measured: whether it is red on every host, or only where the sibling that
  claims the latch happens to schedule first. The failure is order-dependent by
  construction, so a green elsewhere would not contradict this.

## Why it stayed

`node-std-tests` lives in `check-build`, which since phase-396 W1 runs on
`schedule` / `workflow_dispatch` only. No merge-gating event reaches it, so a
red there does not block anything and nobody is told. Same shape as issue 0319.

## What fixing it looks like

Not "make the assertion tolerant" — that turns the positive control into the
vacuous test the doc-comment is trying to avoid. Either give this test its own
process (a `#[test]` in a dedicated integration binary, or a nextest
`test-group` of one), or make the latch injectable so the test drives its own
instance and the process-global one is exercised once, deliberately.
