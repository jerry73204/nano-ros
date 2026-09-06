---
id: 1173
title: "`check-grep-q-error-conflation` never looked at `tests/`, where 10 of 12 scripts are gate-invoked — 131 sites, two of them flaking green->red under `-P12`"
status: resolved
type: bug
area: testing, ci
severity: medium
found: 2026-09-06
resolved: 2026-09-06
related: [0726, 0732, 0196, 1077, 1159]
---

# The gate for this exact class exists, and `tests/` was not in its search roots

```python
SEARCH_ROOTS = ["scripts", "just", "justfile", "packages", "tools"]
```

`check-grep-q-error-conflation` was written for precisely the failure below and
has been ratcheting since issue 0726. Its own rationale names the population
that matters:

> phase-395 W11 spent one of those lowerings: the 46 sites in the 21 scripts a
> `check-fast` gate INVOKES are converted, and those files are at 0. **That is
> the population fan-out actually stresses**, so it is the population where the
> conflation is not theoretical.

`tests/` is that population — **10 of its 12 shell scripts are invoked by a
`check-fast` gate** — and it was never in scope. Issue 0196's class, for the
fourth time: a gate whose coverage is narrower than the rule it enforces.

## What it let through

Two gates failed inside `just ci gate`'s `-P12` fan-out and passed solo:

```
===== FAIL (message-bound-knobs, rc=1) =====
  [FAIL] P: the small class was not sized from the one subscribed type
===== FAIL (reconfigure-on-change, rc=1) =====
  [FAIL] the bound was hit silently -- a build sized from a stale answer must say so:
  ...
  CMake Warning at cmake/NanoRosReconfigure.cmake:418 (message):
    nros: ... past NROS_RECONFIGURE_MAX_PASSES=2.
```

Read the second one twice. The check reports that the build said nothing, and
the output it prints as evidence **contains the string it was grepping for**.

```sh
if printf '%s\n' "$BUILD_OUT" | grep -q 'NROS_RECONFIGURE_MAX_PASSES'; then
```

`grep -q` exits on the match, the `printf` takes SIGPIPE, and `set -o pipefail`
— which every script in `tests/` sets — makes that the condition's status. The
hazard scales with how much output follows the match, which is why it fires
under load and why it fires on the biggest logs. Issue 1077's mechanism, in a
directory 1077's sweep could not see.

## Fixed

* **`tests` added to `SEARCH_ROOTS`.** Everything below follows from the gate
  then being able to see the directory.
* **131 sites converted to `nros_grep_q`**, and `tests/` is baselined at 0 the
  way the other two fan-out directories already were. The 28 PIPED sites became
  herestrings, which removes the pipe rather than only fixing the status —
  `printf … | grep -q PAT` becomes `nros_grep_q PAT <<<"$var"`. The four
  `nm … | grep -q` and one `tr … | grep -q` capture first, for the same reason
  with a different writer.
* **The helper is sourced from `tests/lib/common.sh`**, which 10 of the 12
  already source, rather than per script. Two that do not (`codegen-version-refusal`,
  `cmake-reconfigure-stale`) source it directly, and an audit checks every user
  of the helper can resolve it — both of those failed with
  `nros_grep_q: command not found` on the first full run, which is the loud
  direction.

## The conversion found a check that had never fired

`cmake-message-bounds-tests.sh` case P grepped `$T/p-out2.cmake` **before** the
`cmake -P` that writes it:

```
grep: /tmp/nros-message-bounds.aBwCdA/p-out2.cmake: No such file or directory
FATAL: grep failed (rc=2) searching for: ^set\(NROS_DERIVED_
```

Under `if … grep -q …` a missing file exits 2 and the conditional reads it as
"no match", so the assertion silently did not fire — issue 0726's *other*
direction, the one where the check is skipped rather than falsified. It has been
inert for as long as it has existed. The block now runs after the `cmake -P`,
and the file it asserts about exists.

That is the argument for converting the whole population rather than only the
two sites that flaked: the loud direction is a flake somebody re-runs, and the
quiet direction is a check nobody knows is gone.

## Acceptance

```
just check grep-q-error-conflation   OK (87 baselined site(s), no file grew one)
just check message-bound-knobs       92 assertions held
just check reconfigure-on-change     PASS
just check fast                      248 ran, 1 SKIPPED (ledger), 0 failed
```

`grep -nE '\|\s*grep -q' tests/*.sh` — empty. That is the sweep.
