---
id: 1043
title: "`check-submodule-pins` fails CLOSED on any submodule CI does not
  initialise, so a whole class of pin bump could never pass the required lane"
status: resolved
type: bug
area: ci, tooling
severity: high
found: 2026-09-04
resolved: 2026-09-06
related: [issue-0996, issue-1034, issue-0650, phase-395]
---

## What happened

The nuttx pin bump in issue 1034 passed `just check fast` locally (197 gates OK)
and failed the required `CI` context with:

    ===== FAIL (submodule-pins, rc=1, 459ms) =====
    submodule-pins: CANNOT VERIFY third-party/nuttx/nuttx
        the pin moved c3fa5dfb0673 -> aea00d736d69 but the submodule is not
        initialised here, so its history cannot be read.
        Run: git submodule update --init third-party/nuttx/nuttx

The remedy it prints is addressed to a developer with a checkout. In CI nobody
can act on it: the `check` job initialises exactly one submodule
(`packages/cli/third-party/play_launch`), plus the `-sys` sources `nros setup`
provisions on the compile tier. `third-party/nuttx/nuttx` is in neither set, so
the objects the gate needs were never going to exist.

**So the failure is unconditional.** Re-running, rebasing, or pushing the
submodule commit first — which was already done — changes nothing. Any pin bump
to any submodule outside that small set is unmergeable through the required
lane, and the gate reports it as if it were the author's mistake.

## Why the existing machinery did not cover it

`gate.yml` already has a step for the SHALLOW version of this trap ("Fetch
submodule history for check-submodule-pins"), whose own comment says the gate
"cannot fetch (check-fast is network-free), so the workflow provides". That step
is guarded by `[ -e "$path/.git" ] || continue` — it deepens submodules that
exist and skips the ones that do not, which is exactly the case that cannot
recover on its own.

The class is the one CLAUDE.md names for `check-lane-contracts`: a gate in an
affordability tier may only resolve artifacts the job itself provides.
`check-lane-contracts` enforces that for fixture STAMPS; a submodule object
store is the same kind of dependency and is not covered.

## Fixed here (the workflow half)

A step ahead of the existing one initialises, commits-only, any submodule whose
pin differs between the base and the head:

    git submodule update --init --filter=tree:0 "$path"

`--filter=tree:0` because `merge-base --is-ancestor` needs commits and nothing
else. NOT `--depth`: the neighbouring comment already records that a shallow
fetch grafts the commit as a root with no parent links, so the ancestry check
then reports `DIVERGED` on a clean fast-forward. Only pins that actually moved
are touched, so a PR that moves none pays nothing — and a pin that moved is
precisely the one the gate is about to need.

## Fixed — the gate half (2026-09-06)

`scripts/ci/submodule-pins-check.sh` now has THREE outcomes where it had two,
because "cannot evaluate" and "evaluated and bad" were the same red:

    FAIL          the ancestry was MEASURED and the move is not a fast-forward.
    NOT VERIFIED  the objects to measure with are absent here. Reported per
                  path, in the verdict line, AND in the lane's skip ledger.
    OK            measured, fast-forward.

Three things changed.

**1. The object store is resolved, not assumed.** The old test was
`[ -e "$path/.git" ]`, which asks about the WORKTREE. `git submodule deinit` —
and a `git worktree` checkout, which is how every agent session here works —
removes the worktree and keeps the objects at `.git/modules/<name>`. Measured in
this worktree: **20 of 20 submodules have an empty worktree dir and a readable
module store**, so the old spelling called every one of them unverifiable. It
is `--git-common-dir`, not `--git-dir`, because in a linked worktree the latter
is `.git/worktrees/<wt>`, which has no `modules/`.

Replaying the issue-1034 bump in a checkout where `third-party/nuttx/nuttx/` is
an empty directory:

    # `old-gate.sh` is `git show <pre-fix-rev>:scripts/ci/submodule-pins-check.sh`
    $ bash tmp/old-gate.sh fd2f17bca~1 fd2f17bca
    submodule-pins: CANNOT VERIFY third-party/nuttx/nuttx
        the pin moved c3fa5dfb0673 -> aea00d736d69 but the submodule is not
        initialised here, so its history cannot be read.
    rc=1

    $ bash scripts/ci/submodule-pins-check.sh fd2f17bca~1 fd2f17bca
    submodule-pins: OK (1 pin(s) moved — 1 verified fast-forward)
    rc=0

Same shas as the CI failure quoted above. This is not a downgrade to a skip: it
is a real verdict the old gate was throwing away.

The same fallback restores the gate's REASON for existing on this host. Replaying
the 2026-08-15 zenoh-pico rewind the script's own header is about:

    $ bash tmp/old-gate.sh e56354410~1 e56354410
    submodule-pins: CANNOT VERIFY packages/rmw/zenoh/zpico-sys/zenoh-pico
    $ bash scripts/ci/submodule-pins-check.sh e56354410~1 e56354410
    submodule-pins: packages/rmw/zenoh/zpico-sys/zenoh-pico
        REWIND — the new pin is an ANCESTOR of the old one
          was: d3f0d2683b84  zephyr: socket lengths are socklen_t, not unsigned int
          now: 43ddb0ec99b6  unicast rx: reset the buffer only when empty ...
    rc=1

Both reds; only one of them is about the pin.

**2. A store that genuinely is not here is a REPORTED skip, not a failure.**
Both unreadable arms (no store; store present but the commits absent even after
a fetch) now call `unverifiable`, which counts the path and prints it. The
remedy line finally splits by audience, which is what "Still open" asked for:
locally it says `git submodule update --init <path>` *turns this skip into a
verdict*; under `$GITHUB_ACTIONS`/`$CI` it says THIS LANE does not check out
that path, names `gate.yml`'s "Init submodules whose pin moved" step as the
thing that was supposed to provide it, and says plainly that nothing the author
pushes can fix it. Against a retired submodule (`packages/codegen`, which has no
store anywhere):

    submodule-pins: NOT VERIFIED packages/codegen
        the pin moved f2b3363e756a -> 32f56eed7eac; this checkout cannot read
        that submodule's history, so the move was NOT evaluated either way.
        Run `git submodule update --init packages/codegen` to turn this skip
        into a verdict; the pin is unchecked until you do.
    submodule-pins: NARROWED — 1 of 1 moved pin(s) were NOT evaluated ...
    [SKIPPED] submodule-pins: PARTIAL — 1 of 1 moved pin(s) NOT VERIFIED ...
    submodule-pins: OK (1 pin(s) moved — 0 verified fast-forward; 1 NOT VERIFIED)
    rc=0

That last `[SKIPPED]` line is the point. `run-gates-parallel.sh` DISCARDS the
output of every gate that exits 0, so a narrowing printed on the happy path is
invisible in the lane that runs on every push — the fix would have been a silent
skip, which is the defect issue 0650 removed from six other gates. So the
narrowing goes through the SHARED ledger (`nros_check_skip`, `check-skip.sh`),
which both the serial and the parallel closing lines read, rather than a second
spelling of the same idea.

A lane that really does provide every submodule can have the old behaviour with
`NROS_SUBMODULE_PINS_STRICT=1`, which turns the skip back into a failure. It is
deliberately opt-in and wired to no lane: setting it on a lane that checks out a
subset re-creates this issue exactly.

**3. Both directions are mutation-tested on the NORMAL path.** The gate already
re-invoked itself to prove the unresolvable-baseline decision; it now also
builds a throwaway superproject (`update-index --cacheinfo`, no clone, no
network) whose pin REWINDS and runs itself against it twice — once with the
submodule readable (must FAIL, saying REWIND) and once with it absent (must exit
0, saying NOT VERIFIED). Measured cost on this host, no pins moved: **56 ms
before, 107 ms after** — the whole increase is the two mutations, and it buys
the first evidence this gate has ever had that its red still fires.

Mutating the gate to break each direction reds the selftest, which is the only
evidence that the control is live:

| mutation | selftest |
| --- | --- |
| drop the `fail=1` after the REWIND report | `SELFTEST FAILED: a readable REWIND must fail (got rc=0, want 1 with 'REWIND')` |
| restore `fail=1` on the not-checked-out arm | `SELFTEST FAILED: an ABSENT submodule must skip and report it (got rc=1, want 0 with 'NOT VERIFIED')` |

A third mutation found itself: the first version resolved `$0` relatively and
the mutation runs `cd` into the fixture, so it reported `rc=127 ... No such file
or directory` instead of passing.

## What is deliberately NOT changed

* **The workflow half stays as it landed.** `gate.yml` still initialises, with
  `--filter=tree:0`, every submodule whose pin moved, so CI keeps getting a real
  verdict rather than a skip. The gate change is the floor under it, not a
  replacement: when that step cannot fetch, the lane now reports a narrowing
  instead of blaming the author.
* **`check-lane-contracts` still covers only fixture stamps.** The rule it
  encodes ("a gate in an affordability tier may only resolve artifacts the job
  itself provides") is what this issue violated, and it is now satisfied by
  construction: this gate no longer REQUIRES an object store the lane did not
  build — a missing one degrades to a reported skip. Extending lane-contracts to
  submodule stores would guard a class with no live member; the honest note is
  that it remains the issue-0196 shape and nothing in this fix widens it.
* **The pin-rewind gate is still not a substitute for
  `check-submodule-commits-reachable`.** The "commit is not in the object store"
  arm names it explicitly now: cause 1 (never pushed) is that gate's verdict to
  give, which is why this arm no longer has to fail for it.
