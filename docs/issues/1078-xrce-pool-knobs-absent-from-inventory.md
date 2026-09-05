---
id: 1078
title: "The five XRCE pool knobs that size ~86 % of `xrce_session_state_t` appear nowhere in the static pool inventory"
status: open
type: bug
area: rmw, docs
severity: low
found: 2026-09-05
related: [0271, 0739, phase-420]
---

# A knob nobody can enumerate is a knob nobody sets

`book/src/reference/static-pool-inventory.md` is the answer to "what static
memory does an image budget, and which knob moves it" — issue 0271's rule, which
`gen-pool-inventory.py` exists to keep true. Five XRCE knobs are not in it:

- `NROS_XRCE_MAX_SUBSCRIBERS`
- `NROS_XRCE_MAX_SERVICE_SERVERS`
- `NROS_XRCE_MAX_SERVICE_CLIENTS`
- `NROS_XRCE_SUBSCRIBER_RING_DEPTH`
- `NROS_XRCE_BUFFER_SIZE`

Between them they size roughly 86 % of `xrce_session_state_t`. Absent before
phase-420 W9 and absent after it — this is not a regression W9 introduced, and
W9 is only where it was noticed.

Verify: `grep NROS_XRCE_MAX_SUBSCRIBERS book/src/reference/static-pool-inventory.md`
returns nothing, on this commit and on any earlier one.

## Why they are missing

`gen-pool-inventory.py` finds knobs by reading the *sources* that name them. The
XRCE knobs are `-D` defines the build script passes to the C compiler, not Rust
constants with a nameable declaration site, so the generator's scan never sees
them.

## Why it is cheap now, and was not before

phase-420 W9 moved the whole XRCE configuration surface into
`packages/rmw/xrce/xrce-config.txt`, whose `define` records enumerate exactly
these five with their knob names and minimums:

```
define  XRCE_MAX_SUBSCRIBERS       NROS_XRCE_MAX_SUBSCRIBERS       1
define  XRCE_MAX_SERVICE_SERVERS   NROS_XRCE_MAX_SERVICE_SERVERS   1
define  XRCE_MAX_SERVICE_CLIENTS   NROS_XRCE_MAX_SERVICE_CLIENTS   1
define  XRCE_SUBSCRIBER_RING_DEPTH NROS_XRCE_SUBSCRIBER_RING_DEPTH 1
define  XRCE_BUFFER_SIZE           NROS_XRCE_BUFFER_SIZE           64
```

So the fix is for `gen-pool-inventory.py` to read that manifest as a source of
knobs, rather than to hand-add five rows — which would be five more
restatements, the shape phase-420 W9 spent itself removing.

## Not just a docs gap

Issue 0739 measured ~145 KB in one image lost to a pool nobody could enumerate.
These five are the same class: an integrator sizing an XRCE image has no
document that names them.
