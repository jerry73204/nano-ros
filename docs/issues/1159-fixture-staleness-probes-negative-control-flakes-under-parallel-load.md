---
id: 1159
title: "`check-fixture-staleness-probes`' negative control fails under `-P12` and passes solo, so the gate's own proof-of-sensitivity is the flaky part"
status: open
type: bug
area: testing, ci
severity: low
found: 2026-09-06
related: [0835, 0945, 1077]
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

## The mechanism is not the 1077 one

Issue 1077 was `printf | grep -q` taking SIGPIPE under `pipefail`, and it is
fixed. This is a different shape: the old cargo rule decides by **grepping
cargo's build chatter**, so it needs cargo to do observable work in the
fixture's temp leaf. Two candidate causes, neither yet separated:

* **Timing.** Under `-P12` the jobserver is saturated, and the fixture's build
  interleaves differently with the surrounding gates.
* **Shared cargo state.** The phase-340 groups put unrelated leaves in one
  target dir; if a concurrent gate has already built the same unit, the
  fixture's cargo says nothing and the "old rule" cannot fire. That would make
  it a real cross-gate coupling, not just slowness, and is the hypothesis worth
  testing first.

The cheap discriminator: run the gate solo with a **cold, private**
`CARGO_TARGET_DIR` and see whether the control still fires. If it does, the
coupling is the cause; if it does not, the fixture's own freshness is.

## Not blocking

`just check fast` reached 241/241 on the same tree on a later run, so this does
not gate the branch. It does mean a red from this gate cannot be read as a
verdict without a solo re-run — the same rule already written down for QEMU
lanes.
