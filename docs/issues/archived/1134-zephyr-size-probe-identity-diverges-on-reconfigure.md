---
id: 1134
title: "A Zephyr reconfigure changes the size-probe identity, so `west build -t
  run` can die after a build that succeeded"
status: resolved
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

## Resolution (2026-09-06)

**The silent fallback is fixed; the FVP verification is now possible and is a
separate step.**

The mechanism was `nros_zephyr_build::knob_usize`, whose last line read

```rust
dotconfig_usize(kconfig_key).unwrap_or(default)
```

`dotconfig_usize` returned `None` for three situations that mean different
things, and the `unwrap_or` treated them alike:

1. `DOTCONFIG` unset — not a Zephyr build. Taking the crate default is CORRECT;
   these build scripts also compile for the host and every other platform.
2. `DOTCONFIG` set, file read, key absent — a Kconfig int left at its default is
   not written to `.config`, so taking the crate default is CORRECT.
3. **`DOTCONFIG` set, file unreadable** — there IS a configuration and we could
   not read it. Taking the crate default here is the bug: the image compiles,
   links, and behaves as though its Kconfig said nothing.

`KnobSource` now distinguishes the three, and case 3 panics with the path, the
OS error, the knob it could not resolve, and the likely cause (a reconfigure
that did not inherit the configure's environment). Cases 1 and 2 are unchanged.

Covered by `a_knob_distinguishes_absent_from_unreadable`, which exercises all
three through the real `DOTCONFIG` variable and is mutation-tested: collapsing
the `ConfigUnreadable` arm back into `AbsentFromConfig` makes it fail.

**What this does not claim.** It converts a silent wrong-sized image into a
loud build failure. Whether a `west build -t run` reconfigure actually loses
`DOTCONFIG` on an FVP board is still unverified — that needs the model, which
[phase-215](../../roadmap/phase-215-board-crate-as-importable-unit.md) can now
provision (`nros setup --tool arm-fvp`, landed the same day). If it does, the
build now says so instead of producing a broken image.

## Blocks

phase-215's remaining acceptance bullet ("the run path launches
`FVP_BaseR_AEMv8R` end to end") and 215.K.4. Both need the licence-gated model,
so verification is a maintainer step.
