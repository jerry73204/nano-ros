---
id: 986
title: "The pre-push hook writes into the repository it is guarding"
status: resolved
area: tooling
severity: high
related: [0840, 0966, 0988]
resolved: 2026-09-06
---

# A selftest that runs against the caller's repo

## What happens

Git can put a repository-local environment in a hook — `GIT_DIR`, and with it
`GIT_WORK_TREE` / `GIT_INDEX_FILE` / `GIT_CONFIG` — and those override BOTH a
path argument and `git -C`. Three scripts reachable from `.githooks/pre-push`
build throwaway repositories:

* `scripts/ci/submodule-commits-reachable.sh` (`git init -q "$tmp/work"`,
  `git init -q "$tmp/super"`, then `git -C "$tmp/super" update-index --add`)
* `scripts/check-source-manifest.sh` (`git init -q .` in two places, and a
  `git add -A` inside the sandbox)
* `scripts/reserve-claim.sh` (four `git init` calls in its selftest)

So the selftests do not run against a temp repo. They run against the caller's.

## Measured, on this checkout

Two distinct kinds of damage, both observed:

1. **Config.** `git init -q "$tmp/work"` with `GIT_DIR` set writes
   `core.bare = true` into `GIT_DIR`'s config. Every later
   `git rev-parse --show-toplevel` then fails with "fatal: this operation must
   be run in a work tree" -- including the one on line 25 of the hook itself.
   The push aborts, the config stays broken, and the next push repeats it.

2. **Index.** `git -C "$tmp/super" update-index --add --cacheinfo "160000,$1,dep"`
   staged a gitlink named `dep` into the CALLER's index, carrying the selftest's
   deliberately invalid sha:

       AD dep
       Submodule dep 000000000...012345678 (new submodule)

   That is a bogus submodule pin staged into a working repo by the hook whose
   stated job is to refuse bad submodule pins. Had it been committed by an
   unrelated `git commit -a`, the hook would have been the source of exactly the
   failure it exists to prevent.

## WHEN git actually supplies that environment — measured, git 2.34.1

The original filing said "Git sets `GIT_DIR` in a hook's environment". That is
not true in general, and getting it right matters, because it explains both why
the bug was never seen by hand and why it is nonetheless live here. Probed by
running a hook that dumps every name in `git rev-parse --local-env-vars`:

| how the push is made | what the hook inherits |
| --- | --- |
| ordinary `git push` from an ordinary checkout | `GIT_PREFIX=` only |
| push from a **linked worktree** | `GIT_DIR=<common>/.git/worktrees/<name>`, `GIT_PREFIX=` |
| `git --git-dir=X --work-tree=Y push` | `GIT_DIR` + `GIT_WORK_TREE` + `GIT_PREFIX` |
| any caller that already exported `GIT_DIR` | inherited unchanged |
| `git -c a.b=c push` | `GIT_CONFIG_PARAMETERS` |

A linked worktree is the normal shape for the parallel agent sessions this repo
runs, and a linked worktree's `$GIT_DIR/index` **is** that worktree's live index
— so the index half of the damage lands on real state, not a scratch file.

The two shapes also do *different* damage, which is why both are exercised by
the gate: with `GIT_WORK_TREE` also set, `git init` writes `core.worktree`;
without it, it writes `core.bare = true`, which is what was measured above.

## Why it went unseen

The selftests redirect to `/dev/null` and return a status, so the damage is
silent. And running `bash .githooks/pre-push` by hand does NOT reproduce it,
because an interactive shell has no `GIT_DIR`. A hand-run hook passes and the
real one corrupts.

Worse, it is self-masking: run 1 sets `core.bare=true`, and from run 2 onward
the hook dies at `rev-parse --show-toplevel` BEFORE reaching the selftests, so
the symptom presents as a broken repository rather than as a bad script.

## Fix

Two parts.

**One spelling for the clear.** `scripts/lib/git-hook-env.sh` exposes
`nros_clear_inherited_git_env`, which unsets every name
`git rev-parse --local-env-vars` reports — git's own answer to "which variables
are repository-local", so it grows when git grows. The hook and all three
scripts call it. The first pass at this fix wrote the `unset` four times by
hand, and the four copies had already diverged: the scripts cleared four names,
the hook cleared five, and none cleared `GIT_CONFIG` (which redirects `git
config` WRITES) or `GIT_ALTERNATE_OBJECT_DIRECTORIES`. That is the #282 -> #326
shape — a second spelling instead of a shared helper.

**A gate that runs the code the way git runs it.** `check-hook-repo-side-effects`
(fast lane, ~5 s, no build, no network) — see issue 0988 for why nothing weaker
could have caught this. It:

* enumerates every tracked script under `scripts/` and `.githooks/` that invokes
  `git init` (derived, not a hand-list — a hand-list is what let three scripts
  share one defect) and requires each to call the shared helper;
* builds a victim repo shaped like this one (separate gitdir, `core.worktree`
  set, no `bare` key, plus a linked worktree), exports the hook environment of
  BOTH shapes above pointed at it, runs each hazardous script and then the hook
  end to end with git-shaped stdin, and compares the victim byte for byte
  including mtimes;
* carries its own negative control on every run: a miniature offender (the two
  damage sites above, verbatim in shape) must be reported DIRTY in both shapes,
  and the same script with the clear must be reported CLEAN.

Byte identity rather than "config looks fine" is deliberate. Anything that
rewrites a tracked file re-stales every prebuilt fixture in the tree, so a hook
with a harmless-looking write costs a full rebuild.

## Verification

    BEFORE (no clear — as filed)      bare=false index=1  ->  bare=true  index=2
                                                              staged: 160000 0123456789…  dep
    AFTER  (nros_clear_inherited_git_env)  bare=false index=1  ->  bare=false index=1

against a disposable repo with a linked worktree, under exactly the environment
git supplies there.

End to end on this checkout, driven by a real `git push --dry-run` to a local
bare remote (a linked worktree, so git sets `GIT_DIR` itself): `git status
--porcelain`, the mtime+size of every tracked file, `.git/config` and the staged
index are all byte-identical across the hook run.

Mutation test: deleting the helper call from
`scripts/ci/submodule-commits-reachable.sh` turns the gate red in both halves —
the static one naming the script, the dynamic one printing the config bytes,
reflog and loose objects that appeared in the victim.
