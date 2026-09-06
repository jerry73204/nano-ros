---
id: 1154
title: "`config/bare-metal` defines `cortex-m4f` and `riscv32gc` and lists neither, so an STM32F4 or riscv32gc bare-metal build resolves no arch profile"
status: open
type: bug
area: platform, build
severity: medium
found: 2026-09-06
related: [0385, phase-418]
---

# phase-385 W1's defect, twice more, in a file that phase did not look at

`config/bare-metal/nros-platform.toml`:

```toml
arch = ["cortex-m3", "riscv32imc"]      # line 40

[arch.cortex-m3]                        # line 57  — listed
[arch.cortex-m4f]                       # line 62  — NOT listed
[arch.riscv32imc]                       # line 66  — listed
[arch.riscv32gc]                        # line 73  — NOT listed
```

The `arch = [...]` list is what the resolver walks; a block that is defined and
not listed is unreachable. So a bare-metal `thumbv7em-none-eabihf` build (the
STM32F4 shape `[arch.cortex-m4f]` exists for) and a `riscv32gc` build both
resolve nothing.

This is exactly phase-385 W1's finding — `cortex-r52` was defined and missing
from the FreeRTOS platform's list — reproduced in a different platform file.
W1 fixed the site it found; the class survived one directory over.

## How it surfaced

`check-arch-profile-resolution`, added for phase-418 item 418.3, asserts both
directions (listed ⇒ defined, defined ⇒ listed) across every platform manifest.
These two are its only findings on the tree, and they are currently held in a
named `KNOWN_UNREACHABLE` allowlist with this issue as the reason, so the gate
is green while the debt stays visible.

## Why it is not simply "delete the dead blocks"

Both look like intent rather than debris — `cortex-m4f` carries a complete
`-mfpu=fpv4-sp-d16 -mfloat-abi=hard` flag set, `riscv32gc` a complete
`-march=rv32gc -mabi=ilp32d`. Someone wrote them for a board. Adding them to
`arch` is a BEHAVIOUR change for any bare-metal build whose triple they would
now claim, and `[arch.cortex-m4f]`'s `target_match = "thumbv7em"` is a
substring that also matches `thumbv7em-none-eabi` (soft-float) — the same
over-claim the FreeRTOS `cortex-m7` profile documents as a KNOWN LIMITATION.

So the fix needs a decision per profile: list it and accept what it claims, give
it a `target_exclude`, or delete it and say the board it was for is gone.

## `riscv32gc` is broken TWICE, and fixing one half looks like fixing both

Found after this issue was first written, by the gate's fifth rule (every
defined profile must admit at least one triple): `riscv32gc` is unlisted, **and**
its `target_match = "riscv32gc"` appears in no row of `config/rust-targets.txt`
— the provisioned RISC-V triples are `riscv32imc-unknown-none-elf`,
`riscv64gc-unknown-none-elf` and `riscv32imac-unknown-nuttx-elf`. So even once
listed it admits nothing.

`check-arch-profile-resolution` carries it in **both** `KNOWN_UNLISTED` and
`KNOWN_INERT` for exactly that reason, and the two allowlists are deliberately
not merged: merging them would mean fixing one half silently keeps suppressing
the other. Each entry is also self-cleaning — the gate refuses an allowlist row
whose fault no longer occurs — so a fix that adds `riscv32gc` to `arch = [..]`
and stops there will fail on the remaining entry rather than look done.

## Verify

```sh
python3 scripts/check-arch-profile-resolution.py    # reports 2 known unreachable
grep -nE '^arch = |^\[arch\.' config/bare-metal/nros-platform.toml
```
