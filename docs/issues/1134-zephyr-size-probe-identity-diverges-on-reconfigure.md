---
id: 1134
title: "A Zephyr reconfigure changes the size-probe identity, so `west build -t
  run` can die after a build that succeeded"
status: open
type: bug
area: build
related: [phase-215, rfc-0064]
---

## Problem

The nano-ros size-probe identity is every `NROS_*` environment variable plus the
`CONFIG_NROS_*` lines of `$DOTCONFIG`. `west build -t run` re-enters the build
graph, and a reconfigure that does not reproduce that environment computes a
DIFFERENT identity than the build did — so the probe resolves against crate
defaults (a 64 KiB platform heap where the image was built with a 448 KiB
executor arena) and the run dies after a build that reported success.

Measured downstream: `autoware-safety-island` declined `west build --target run`
for exactly this and launches through its own `build.sh --run`, from the same
place the build ran.

## Why it is filed now

phase-215 shipped a `west fvp run` verb, and its stated purpose was to own the
FVP launch. It did not avoid this: its last statement was

```python
os.execvpe('west', ['west', 'build', '-d', args.build_dir, '-t', 'run'], env)
```

— the exact command. The verb added `ARMFVP_BIN_PATH` to the environment and
changed nothing about the identity, so the failure was reachable through it too.
RFC-0064 R5 D4 retired the verb (phase-215.K.3/K.4), which makes this the only
remaining obstacle to the documented launch path being honest, rather than one
of two.

## What "fixed" means

A reconfigure of an already-configured Zephyr build directory must compute the
same size-probe identity as the configure that produced it, or must fail loudly
rather than silently substituting defaults. Whichever way it goes, the outcome to
verify is `west build -d <dir> && west build -d <dir> -t run` on an FVP board,
with no environment re-export between the two.

## Blocks

phase-215's remaining acceptance bullet ("the run path launches
`FVP_BaseR_AEMv8R` end to end") and 215.K.4. Both need the licence-gated model,
so verification is a maintainer step.
