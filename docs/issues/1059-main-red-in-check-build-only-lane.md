---
id: 1059
title: "two reds sit on `main` in `check-build`, which no merge-gating event runs
  — a half-landed `type Format` and a clippy `format!`-in-`assert!`"
status: open
type: bug
area: ci, core
related: [phase-421, phase-395, issue-1013]
---

## Problem

`just ci gate` on a branch rebased onto `main` at `6cbc85fe7` fails step 3 with
two `check-build` gates red. Both reproduce on **`main` itself**, checked out
clean in a separate worktree — neither comes from the branch.

### 1. `node-std-tests` — `type Format` is declared by nine impls and by no trait

```
error[E0437]: type `Format` is not a member of trait `nros_serdes::schema::Message`
   --> packages/core/nros-node/src/executor/node.rs:2555
```

`main`'s `47880d2e1` (*feat(phase-421 W1-W3): the serialization format stops
being a comment*) added `type Format = nros_serdes::format::Cdr;` to nine
`Message` impls — one in `executor/node.rs`, eight in `executor/tests.rs` — but
`nros_serdes::schema::Message` (`schema.rs:135`) declares no such associated
type. `git grep "type Format" origin/main -- packages/core/nros-serdes` returns
nothing.

Reproduced on a pristine `origin/main` worktree with the gate's own feature set:

```
$ cargo test -p nros-node --lib --features std --no-run
error[E0437]: type `Format` is not a member of trait `nros_serdes::schema::Message`   (×3)
```

Note `cargo check -p nros-node --lib --features std` **passes** — all nine sites
are behind `#[cfg(test)]`, so only a test build reaches them. That is why a
lane which merely compiles the library says nothing about this.

### 2. `test-targets` — `format!` inside `assert!` args

```
error: `format!` in `assert!` args
   --> packages/testing/nros-tests/tests/rtos_e2e.rs:903
```

From `05bf241eb` (*fix(#1013): the pubsub cell lets its talker run…*), same day.

## Why both survived

`check-build` runs on `schedule` / `workflow_dispatch` only. The required `CI`
context on `pull_request` is `check-fast` + `check-submodule-commits-reachable`
+ `check-compile-smoke` + `check-cli-tests`, and the merge queue adds
`test-unit` — **`check-build` is in none of them**. So a red here blocks
nothing, is reported to nobody at merge time, and is first seen by whoever next
runs the full local tier.

This is the third instance in one session. `board_facts.rs`'s
`items_after_test_module` red (`e453399de`) rode in the same way and was fixed
by `09b653ec2` only because a local tier run surfaced it. CLAUDE.md already
records the pattern for `check-build`; what it does not say is that the lane has
now caught three separate reds that reached `main`, which makes this a gap in
coverage rather than a stale gate.

## Fix

Two separate fixes and one policy question.

* **`type Format`** — decide whether phase-421's associated type belongs on
  `Message`. If yes, land the trait half (`type Format: SerializationFormat;`,
  presumably with a default so generated messages need no edit). If the nine
  impls were speculative, delete them. Do not guess: RFC-0088 makes format a
  TYPE, so the trait shape is a design decision, not a mechanical repair.
* **the clippy site** — inline the captured identifiers, or hoist the message.
* **the lane** — the reason `check-build` is not merge-gating is cost, and that
  reason still holds for the whole lane. But `node-std-tests` needs no fixture,
  no SDK and no QEMU, and `test-targets` is clippy. Moving just those two into
  a merge-gating event would have caught all three of this session's reds. That
  is a `check-lane-contracts` question: an affordability tier may only resolve
  artifacts the job itself builds, and both of these qualify.


## CORRECTION 2026-09-06 — the lane fix was written and never merged

This issue's third fix item — move `node-std-tests` into a merge-gating event —
was written, verified locally, and described in three places as landed. **It
was not.** `grep -c "node-std-tests" .github/workflows/gate.yml` returns 0 on
`origin/main` and 0 at PR #438's merge commit; exactly one commit in the repo
ever contained the step, on a branch that was still open.

So the fourth red — `use_sim_time_attaches_and_detaches_the_clock_source`
(`5200437c5`, phase-425 W3b) — did not slip past a gate. It reached `main`
through the absence of one, for the ordinary reason this issue already
describes.

The wrong turn is the instructive part. `5200437c5` **is** a descendant of
#438's merge commit, so the tree the queue tested would have contained the step
had the step existed. That looked like a hole in a live gate and produced two
elaborate explanations — an in-flight queue batch, an environment-dependent
test — before the simplest check. One `git merge-base --is-ancestor` and one
`grep -c` settled it.

**A claim believed because it was AUTHORED rather than OBSERVED** is what this
campaign documents in other people's work. It was made here, three times, about
this issue's own remedy.

Landed on `ci/land-node-std-gate` (`00e8f3087`). This issue stays open until
that merges.
