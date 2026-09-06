---
id: 883
title: "`docs/issues/README.md` is generated, committed, and touched by nearly every
  PR — so one merge ejects all the others from the queue"
status: resolved
type: tech-debt
area: ci
related: [phase-396, issue-0871, issue-0884]
resolved_in: "issue 0884"
---

## Resolved — recovered 2026-09-02

This file was written but never reached `main`: the commit carrying it
(`bda4a5478`) was not merged, while the FIX it argues for was landed separately
under issue 0884. So the defect was fixed and its analysis was lost, leaving
`CLAUDE.md` and `docs/issues/README.md` citing "issues 0883/0884" with only one
of them on disk.

Recovered rather than deleted, because the two issues say different things:
0884 records the ledger conflicting on concurrent filings; THIS one records why
the conflict serialises the QUEUE — one merge ejecting every other entry — which
is the argument for `merge=union` on a generated list rather than for a tidier
list. The 0884 fix (move the generated rows out of the authored `README.md` into
`docs/issues/open.md`, `merge=union`) is what closes both.

Verified on main: `docs/issues/README.md` carries 0 generated rows,
`.gitattributes` marks `docs/issues/open.md merge=union`, and five concurrent
rebases on 2026-09-02 produced no conflict in that file.


## Problem

`docs/issues/README.md` is **generated** by `scripts/gen-issue-index.py` from the
frontmatter of every issue file, and it is **committed**. Almost every pull
request in this repository files, resolves or archives an issue, so almost every
pull request rewrites it — and two pull requests that both rewrite it conflict,
even when their actual changes are unrelated.

Measured 2026-08-29, test-merging each open branch against `main` in an isolated
worktree:

```
fix/0865-nuttx-qemu-throttling      CONFLICT: docs/issues/README.md
fix/0819-xrce-fragmented-receive    CONFLICT: docs/issues/README.md
fix/0871-ci-compile-check-fixtures  CONFLICT: docs/issues/README.md
fix/heap-free-tier-gate             merges CLEAN
fix/0831                            merges CLEAN
```

Three of five, and `docs/issues/README.md` is the **only** conflicting path in
each. Nothing else disagrees.

## What it costs

It serialises the merge queue. On 2026-08-29, seven pull requests were rebased
onto `main`, gated green and armed with auto-merge. `#9` — a two-line issue
resolution — merged. That single merge conflicted the index in the other six,
which were ejected from the queue with their auto-merge cancelled, and needed a
full rebase round to get back.

So the throughput of a queue that now works correctly is capped at **one
doc-touching pull request per rebase round**, and essentially every pull request
is doc-touching. This is the bottleneck that remains after phase-396 fixed the
required check.

## Why the obvious fixes do not work

**A custom merge driver does not help.** `.gitattributes` plus
`merge.<driver>.driver = python3 scripts/gen-issue-index.py` would resolve this
perfectly on a developer's machine — the file is a pure function of the issue
files, so regenerating IS the correct merge. But GitHub's merge queue performs
its rebase **server-side**, where custom merge drivers do not run. It would fix
local rebases and leave the queue exactly as it is, which is the half that hurts.

**`merge=union` is wrong.** The index is a sorted, structured table with a
counted header; union merge produces duplicate rows and a count that disagrees
with them, and `check-issue-index` fails on the result — correctly.

**Sorting differently does not help either.** The rows are already ordered by
id, so two pull requests filing adjacent ids touch adjacent lines. And today's
conflicts were mostly *removals* (issues resolved and archived), which conflict
regardless of ordering.

## Options, with what each gives up

1. **Stop committing it.** Gitignore the file, generate on demand
   (`just issues-index`), and publish it from CI rather than from the tree. Ends
   the conflict class outright. Gives up the browsable index on GitHub, which is
   most of why the file exists — and `check-issue-index` becomes a check with
   nothing to compare against.
2. **Commit it, but regenerate on `main` after each merge.** A `post-submit` job
   regenerates and pushes. Ends the conflicts; costs a bot commit per merge, and
   collides with `required_linear_history` plus the PR-only rule — the bot needs
   a bypass actor, and `bypass_actors` is deliberately empty.
3. **Shrink the file to what does not churn.** Keep a stable index that names
   only the issue *files* (which are append-only) and move the volatile columns —
   status, title, area — out of the committed artifact. Smaller conflict surface
   without losing navigability. Most design work of the three.
4. **Accept it**, and make the cost explicit: batch doc changes, and expect one
   rebase round per merge.

Not decided. Option 1 is the smallest change and the biggest loss; option 3 is
the one that keeps what the file is for.

## Reproduce

```
W=$(mktemp -d); git worktree add -q --detach "$W" origin/main
cd "$W"
for b in $(git for-each-ref --format='%(refname:short)' refs/remotes/origin | grep -v HEAD); do
    git checkout -q --detach origin/main
    git merge --no-commit --no-ff "$b" >/dev/null 2>&1 \
        && echo "$b: clean" \
        || echo "$b: CONFLICT $(git diff --name-only --diff-filter=U | tr '\n' ' ')"
    git merge --abort 2>/dev/null
done
```

Note `git merge-tree --write-tree` is NOT available on this host's git; an
earlier version of this measurement used it and reported "CONFLICT" for every
branch, which was the command failing rather than a merge result. Use a real
worktree merge.

## Acceptance

- Two pull requests that each file an issue, and touch nothing else in common,
  can both merge without a rebase round between them.
- `check-issue-index` still fails when the index disagrees with the issue files,
  whatever form the index takes.
