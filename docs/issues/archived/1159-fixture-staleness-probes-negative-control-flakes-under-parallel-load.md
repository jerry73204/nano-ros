---
id: 1159
title: "`check-fixture-staleness-probes`' negative control fails under `-P12` and passes solo, so the gate's own proof-of-sensitivity is the flaky part"
status: resolved
type: bug
area: testing, ci
severity: low
found: 2026-09-06
resolved: 2026-09-06
related: [0835, 0945, 1077, 1168]
---

# The assertion that flakes is the one proving the gate can fail

`just check fixture-staleness-probes` runs 21 checks. Twenty are the probe's
own behaviour; the last two are a **negative control** — the pre-0835 rules
re-applied to the same fixture trees, to show those trees still model the
defect the probe was written for. It is the second of those that flakes:

```
== the OLD rules still fire on this fixture (negative control) ==
  [ok]   the old cmake rule (grep the build chatter) reports STALE — as it did forever
  [FAIL] the old cargo rule did NOT fire: this fixture no longer models the defect
```

Measured on `work/phase-418-1-and-3` at 1ab2fa632, on a tree the gate does not
touch:

| how it ran | result |
| --- | --- |
| `just check fixture-staleness-probes`, solo | PASS, 3 of 3 |
| inside `just check fast` (`-P12`) | 2 green runs, 2 red |

Same binary, same tree, same commit. Nothing in the branch's diff touches the
gate, its script, or its fixtures (`git diff --name-only 89d7fd763..HEAD` names
neither).

## Why this is worth a number rather than a retry

The failing assertion is the gate's evidence that it is **sensitive**. A probe
that decides staleness from an artifact's bytes is exactly the shape that can
silently answer "fresh" to everything; the negative control exists so that a
green gate means something. An intermittent negative control is worse than a
missing one, because the message it prints — "this fixture no longer models the
defect" — reads like a real finding about the fixture and is, most of the time,
a statement about the machine's load.

## The mechanism is not the 1077 one, and it is not load either

Issue 1077 was `printf | grep -q` taking SIGPIPE under `pipefail`, and it is
fixed. The two hypotheses filed here — timing under `-P12`, and the phase-340
shared cargo target dir — are both **wrong**, and the discriminator this issue
proposed is what disproved them:

* The target dir is NOT shared. `NROS_BUILD_ROOT="$work/build-root"` with
  `work="$(mktemp -d)"`, so every run gets a private one. No cross-gate
  coupling exists to find.
* CPU load alone does not reproduce it: the whole script passes under 12 busy
  loops on a 12-core host.

## What it actually is: the control cannot fail honestly

```sh
out="$(cd "$work/leaf" && cargo build --profile "$probe_profile" \
    --target-dir "$NROS_BUILD_ROOT/cargo-fixtures/linux" \
    --message-format=json --quiet 2>/dev/null)"
if printf '%s' "$out" | grep -q '"fresh":false'; then
```

**stderr discarded, exit status never read.** Any build that does not run
produces no `"fresh":false`, and the check reports

```
[FAIL] the old cargo rule did NOT fire: this fixture no longer models the defect
```

— a sentence aimed at the fixture, for a fault in the environment. Proven by
making the target dir unwritable, which is one of several ways to reach it (a
full disk is the one that produced the original reds; a stale lock left by a
killed sibling is another):

```
error: failed to open: .../nros-relwithdebinfo/.cargo-build-lock
  Permission denied (os error 13)
NEGCTL_RC=101
[FAIL] the old cargo rule did NOT fire: this fixture no longer models the defect
```

That also explains the correlation this issue mistook for load. The red runs
were the ones where the disk was filling toward the 100% it eventually hit; the
green ones had headroom.

## Fixed

Both halves of the negative control — cmake and cargo — now capture the exit
status before reading the output, and neither discards stderr. A build that
failed reports

```
[FAIL] the old cargo rule could not be evaluated: the probe BUILD failed (exit 101).
       This says nothing about the fixture.
       error: failed to open: .../.cargo-build-lock
         Permission denied (os error 13)
```

Verified in both directions: the unmutated gate passes 21 of 21, and the
unwritable-target-dir mutation produces exactly the message above instead of the
fixture accusation.

The cmake half had the same hole and was fixed with it — it captured `2>&1` so
the text survived, but it too never checked the status, so a build that printed
`Building C object` before dying would have PASSED the control.

## Why it was worth a number

The 21st check is this gate's evidence that it is SENSITIVE: a probe deciding
staleness from an artifact's bytes is exactly the shape that can silently answer
"fresh" to everything, and the control is what makes a green gate mean
something. A control that misreports a failed build as a modelling problem sends
the next reader to edit the fixture.
