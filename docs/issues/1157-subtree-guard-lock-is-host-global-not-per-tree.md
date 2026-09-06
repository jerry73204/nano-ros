---
id: 1157
title: "`subtree-guard`'s lock is keyed by build NAME in a host-global directory, so an
  unrelated nano-ros checkout blocks this one's gates while the refusal says
  \"two builds in one tree\""
status: open
type: bug
area: [build, testing]
severity: medium
found: 2026-09-06
related: [0853, 1025]
---

## What

`scripts/build/subtree-guard.sh` refuses to start a build when another build of
the same name is already running. The lock it consults:

```sh
_nros_guard_lock_path() {
    local name="$1" root
    root="${NROS_GUARD_LOCK_DIR:-${TMPDIR:-/tmp}/nros-build-guards}"
    mkdir -p "$root" 2>/dev/null || true
    printf '%s/%s.pgid\n' "$root" "$name"
}
```

`/tmp/nros-build-guards/<name>.pgid`. The path is keyed by the build NAME and by
nothing else — not the repo root, not the git common dir, not `NROS_BUILD_ROOT`.
Its own comment says so, and scopes only against a sibling lane:

> Where the lock lives. One per NAME, so `fixtures` and a platform lane do not
> evict each other.

Membership is then resolved with a host-wide `ps -eo pid=,pgid=,stat=`, so the
scan matches the scope of the lock.

**The guard's stated purpose is per-tree**, and the refusal asserts it:

```
subtree-guard: a 'jobserver-pool' build is already running (pgid 2199954, 81 process(es)).
  Two builds in one tree corrupt each other's artifacts, so this one refuses to start.
```

Two builds in *one tree* is the hazard. What the guard actually enforces is one
build of a given name *per host*.

## Measured

`just ci gate` in the primary checkout failed at `check-source-gates` with the
message above. The lock it tripped on:

```
$ cat /tmp/nros-build-guards/jobserver-pool.pgid
2907358 2909028

$ ps -o pid,pgid,cmd -p 2909028
2909028 /home/aeon/.nros/sdk/make/4.4.1/bin/make -j20 --jobserver-style=fifo \
  -f /home/aeon/repos/simple-autoware-safety-island/third-party/nano-ros/build/jobserver-pool/compile-check-2907358.mk

$ readlink /proc/2909028/cwd
/home/aeon/repos/simple-autoware-safety-island/third-party/nano-ros
```

A **different checkout of nano-ros, vendored into a different project**, held
the lock. The two trees share no build directory, no `--target-dir` and no
artifacts. Nothing was at risk of corruption, and the gate refused anyway while
telling the reader the opposite of what had happened.

## Why it matters more than an inconvenience

The refusal is indistinguishable from a real defect at the point of reading. It
arrives as `check-source-gates` FAILED inside `check::build`, and that lane
stops at the first failure, so `check::api-parity`, `test-unit` and
`test-lane-contracts` are all withdrawn behind it (issue 0952's rule). One
unrelated checkout on the host therefore converts a full local gate run into a
red with three steps unrun — and the printed cause names a hazard that does not
exist here.

It is also not rare on this host by construction. nano-ros is vendored as a
submodule into consumer projects, agent worktrees live under
`.claude/worktrees/`, and a self-hosted CI runner keeps its own checkout at
`/home/aeon/actions-runner/_work/nano-ros/nano-ros`. Any two of those building
at once collide.

## Not the same as the shared-workspace hazard

There IS a real cross-checkout hazard on this host, and it is a different one:
the self-hosted runner's `.env` sets
`NROS_ZEPHYR_WORKSPACE` to a `zephyr-workspace/` inside the primary developer
checkout, so CI builds Zephyr into a developer's tree. That one is genuine sharing and deserves a
guard. This issue is the opposite error — refusing trees that share nothing.
Fixing this must not weaken that.

## Fix direction

Key the lock by the tree as well as the name. The repo root is already resolved
in `scripts/build/build-root.sh` (`NROS_BUILD_ROOT` -> `NROS_REPO_ROOT` ->
`NROS_REPO_DIR` -> the script's own repo), and `nros_build_dir` is the existing
spelling every other build path goes through — so the lock belongs under the
build root, not `/tmp`, or in `/tmp` under a hash of the resolved root.

Note `git rev-parse --git-common-dir` is NOT the right key on its own: a linked
worktree returns the MAIN repo's git dir, so every agent worktree would hash to
the same value as the primary checkout and keep colliding. The resolved
BUILD ROOT is the correct identity, because it is what the artifacts are keyed
by — the same reasoning as issue 1025, where a build's group key had to be a
function of the inputs the artifacts actually depend on.

Whatever the key becomes, the refusal message must describe what was really
detected. If a host-wide scope is deliberate for some lane, say "on this host"
and name the other tree's path — the lock already records a pgid, and
`/proc/<pid>/cwd` resolves it, as the measurement above shows.

## Reproduce

1. Start a `jobserver-pool` build in any nano-ros checkout.
2. In a second, unrelated nano-ros checkout, run `just check source-gates`.
3. It refuses, naming a tree-corruption hazard that cannot apply.

## Corroborated independently the same day

A parallel agent working issue 1028 in a `.claude/worktrees/` worktree reported
`check-build` reds it did not cause, and listed among them "another agent's
build holding the jobserver lock" — the same refusal, reached from a third tree,
without knowledge of this issue. It also hit a neighbouring cross-tree symptom
worth recording here as a lead rather than a claim: a cmake cache in its
worktree carrying paths generated from the primary checkout. That is a different
mechanism (a configured build dir remembering an absolute source path, not a
lock) and needs its own measurement before anyone files it.

Escape hatch today: `NROS_GUARD_FORCE=1`, or `NROS_GUARD_LOCK_DIR` pointed
somewhere per-tree. Both require knowing this issue exists.
