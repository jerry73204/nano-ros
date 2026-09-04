---
id: 1051
title: "`check-submodule-pinned-locks` blames the lock for a drifted CHECKOUT, and
  the remedy it prints would rewrite a correct lock to match the wrong tree"
status: open
type: bug
area: tooling, ci
---

## Problem

The gate resolves each submodule-pinned leaf under `--locked` and, on failure,
prints:

```
  The submodule pointer moved and the lock did not follow (issue 0560).
  Update it the sanctioned way — never a bare `cargo generate-lockfile`:
      just lock-update "" "" <leaf-dir>
```

That is one of **two** causes, and the message asserts it as the only one.

The other: the pointer did NOT move, the **worktree checkout** drifted from it.
A rebase does this routinely — the superproject's gitlink advances and the
submodule working tree stays where it was, which is the state CLAUDE.md
describes as "the superproject's pin disagreeing with your checkout".

Both produce the identical symptom, because `--locked` resolution reads the
checked-out source either way.

## Why the wrong remedy is destructive

Following the printed advice on the second cause regenerates the lock **against
the drifted checkout** — so a correct lock is rewritten to match a tree that is
not what the pin names, and the result is committed looking like deliberate
dependency work. The gate then passes, which is the worst part: the tree is
consistent with itself and inconsistent with the pin.

Encountered 2026-09-04 on `packages/cli/nros-launch-resolve` after a rebase.
The lock and the gitlink were **byte-identical to `origin/main`** — nothing had
moved — while the checkout sat at `8fda8d89` against a pin of `4c214a63`. The
fix was `git submodule update packages/cli/third-party/play_launch`, which the
message does not mention.

## Fix

Distinguish the two before advising, which is cheap — the gate already knows
the leaf, so it can compare the recorded gitlink against the submodule's
`HEAD`:

* `HEAD != gitlink` → the CHECKOUT drifted. Print
  `git submodule update <path>`, and do NOT mention `lock-update`.
* `HEAD == gitlink` and the lock still fails → the pin genuinely moved ahead of
  the lock. Print today's message.

Worth a rule beyond this gate: **a remedy line is part of the diagnostic, and a
wrong one is worse than none.** A gate that names a cause it has not
distinguished sends the next person to make the tree worse in a way that then
passes. Related in shape to issue 0445, where an absorbing STALE verdict
replaced the runtime result with a confident wrong explanation.
