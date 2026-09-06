---
id: 1180
title: "`mem-report --baseline` silently reports no delta for LLVM-internalised symbols — including `EXECUTOR_BACKING`, the largest RAM symbol in every Rust image"
status: open
type: bug
area: tooling
related: [phase-392]
---

## Problem

`scripts/nros-mem-report.py --baseline` matches symbols by NAME:

```python
base_syms = {t["symbol"]: t["bytes"] for t in baseline.get("top_ram", [])}
...
if row["symbol"] in base_syms:
    d = row["bytes"] - base_syms[row["symbol"]]
```

A symbol LLVM internalises carries a per-build `.llvm.<hash>` suffix, and the
reporter keeps it in the name. The hash is not stable across builds of the same
crate, so the lookup misses and the row is printed with **no delta annotation** —
which is exactly how the tool prints a symbol that did not change.

Measured, two builds of the same test binary differing only in one source line:

```
baseline: nros_node::executor::backing::EXECUTOR_BACKING (.llvm.12488621184790092911)  86,216
current : nros_node::executor::backing::EXECUTOR_BACKING (.llvm.13033674138034542000)  86,216
```

Same bytes here, so the missing annotation happened to be right — but it would
have been printed identically had the number moved, because the two names never
met. `EXECUTOR_BACKING` is 38.1% of that image's attributed RAM and is the
single symbol phase-392 W6 created *in order to be measurable*, so this is the
worst possible symbol to be unable to diff.

## Why it matters more than it looks

phase-392's standing rule is that no wave claims a saving it did not measure,
and `mem-report --json --baseline` is the named instrument. A `--baseline` run
that cannot match a symbol produces the same output as one that measured no
change, so a wave can read a clean before/after out of a probe that compared
nothing. W5 already had to withdraw a causal claim for the adjacent reason (a
before/after that built the same configuration twice) and W6 discarded a
-11,199 B reading for it.

## Fix

Normalise the symbol key before comparing: strip a trailing `.llvm.<digits>`
(and the sibling `.<digits>` suffixes LLVM appends for local symbols) when
building both `base_syms` and the lookup, keeping the full name for DISPLAY.
Where two symbols collide after normalisation, sum or report both rather than
silently taking one.

Give it a positive control, per `check-gate-selftests`' rule: a fixture pair
whose only difference is the `.llvm.` suffix must report the byte delta, and a
pair with equal bytes must report none. A `--baseline` that cannot fail is the
defect being fixed.

## Workaround until then

Read the number directly and compare it yourself:

```
nm -S <elf> | grep EXECUTOR_BACKING
```

Positive control for that reading, measured: rebuilding with
`NROS_SUBSCRIPTION_BUFFER_SIZE=2048` moves the symbol
`0x150c8` (86,216) -> `0x180c8` (98,504), +12,288 bytes.
