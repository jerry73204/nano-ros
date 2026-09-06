---
id: 988
title: "No gate runs a hook the way git runs it — with `GIT_DIR` set — so a
  script that corrupts the caller's repository passes every check"
status: resolved
type: tech-debt
area: tooling
related: [issue-0986, issue-0196]
resolved: 2026-09-06
---

## The gap

Issue 0986 found `.githooks/pre-push` writing into the repository it guards:
three scripts reachable from it build throwaway repos in their selftests, and an
inherited `GIT_DIR` overrides BOTH a path argument and `git -C`. So the
selftests ran against the caller's repo — setting `core.bare = true` in its
config, and staging a gitlink named `dep` carrying a deliberately invalid sha
into its index. The hook whose job is refusing bad submodule pins was staging
one.

That bug is FIXED. This issue was the reason it was possible, which the fix did
not close:

**Nothing in this repo ran a hook under a hook's environment and asserted the
repository was unchanged afterwards.** Verified, not assumed:

* `grep -rln "githooks/pre-push" scripts/ just/` → **no matches**. No gate
  invoked the hook at all.
* The only files mentioning `GIT_DIR` were the three offenders plus the hook
  itself — i.e. the bug and its fix. Nothing tested the property.

## Why this class is invisible to ordinary checking

Two properties, both recorded in 0986 and both measured there:

1. **It does not reproduce by hand.** `bash .githooks/pre-push` from an
   interactive shell has no `GIT_DIR`, so the hook passes. Only git running it
   does the damage. 0986's author reports drawing the wrong conclusion twice —
   first that the hook was innocent, then that the push transport was writing
   the config.
2. **It is self-masking.** Run 1 sets `core.bare=true`; from run 2 the hook dies
   at `rev-parse --show-toplevel` BEFORE reaching the selftests. The symptom
   presents as a broken repository rather than a bad script, and the cause stops
   being executed — so the evidence destroys itself.

A selftest cannot catch it either, because the selftests ARE the thing that
misbehaves, and they redirect to `/dev/null` and return a status.

## The rule, now gated

> A script reachable from a git hook must not modify the invoking repository —
> not its config, not its index, not its object store — whatever git puts in the
> environment.

`check-hook-repo-side-effects` (`scripts/check-hook-repo-side-effects.sh`, fast
lane, ~5 s, no build and no network) enforces it, following this issue's own
sketch:

1. Builds a victim repo shaped like the one 0986 measured on — separate gitdir,
   `core.worktree` set, no `bare` key — plus a linked worktree, and snapshots
   every path, type, size, mtime and content hash under it.
2. Exports the hook environment pointed at that victim, in **two** shapes,
   because they do different damage: `GIT_DIR` alone (what a linked-worktree
   push actually produces; `git init` then writes `core.bare = true`) and the
   `GIT_DIR` + `GIT_WORK_TREE` + `GIT_INDEX_FILE` + `GIT_OBJECT_DIRECTORY`
   superset (`git init` writes `core.worktree` instead).
3. Runs each hazardous script, then `.githooks/pre-push` itself with git-shaped
   stdin (`<local-ref> <local-sha> <remote-ref> <remote-sha>`, local == remote,
   so every stage is reached with nothing for the pin checks to fetch).
4. Asserts byte identity, and prints the diff — which path, which key.

The script set is **derived**, as this issue asked: every tracked script under
`scripts/` and `.githooks/` that invokes `git init`. Today that is exactly the
three offenders plus the hook, the lib and the gate. Parsing the hook's
invocations was the other candidate and is weaker — two of the three offenders
are reached through `just check fast`, not named in the hook.

The gate carries a negative control on the normal path, both halves and both
directions: a miniature offender (0986's two damage sites, verbatim in shape) is
reported DIRTY under both environment shapes, and the same script with the clear
is reported CLEAN, so the gate cannot pass by examining nothing.

## The two "not established" points, now measured

* **Does any reachable script still modify the caller's repo?** No. Every one is
  a standing CLEAN on every push, rather than a one-time check.
* **Does git actually set `GIT_INDEX_FILE` / `GIT_OBJECT_DIRECTORY` for
  `pre-push`?** No — and it does not reliably set `GIT_DIR` either. Measured on
  git 2.34.1 with a hook that dumps every name in
  `git rev-parse --local-env-vars`: an ordinary `git push` from an ordinary
  checkout hands the hook only `GIT_PREFIX=`. `GIT_DIR` appears when the push
  comes from a **linked worktree** (the normal shape for parallel agent sessions
  here), when the caller already exported it, or via `git --git-dir=…`, which
  adds `GIT_WORK_TREE`. So the defensive clear was right, and the gate does not
  test a fiction: it drives the shape that occurs and the superset that bounds
  it. The clearing helper takes its list from git itself
  (`git rev-parse --local-env-vars`) rather than naming variables by hand, which
  is what the four divergent hand-written copies had already got wrong.

## Scope

`just setup-hooks` installs `core.hooksPath=.githooks`, and `.githooks/` holds
exactly one hook: `pre-push`. The gate says so rather than leaving it ambiguous
— a second hook file appearing there is caught by the derived enumeration, since
any hook that builds a throwaway repo lands in the hazardous set.
