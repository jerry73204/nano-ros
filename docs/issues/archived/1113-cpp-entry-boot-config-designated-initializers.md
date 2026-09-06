---
id: 1113
title: "RETRACTED — the C++ entry's designated initializers are fine; the build was using an unpinned host toolchain"
status: wontfix
type: bug
area: cli, codegen
severity: low
found: 2026-09-06
related: [1117]
---

# Retracted, with the two wrong diagnoses recorded

This was filed as a codegen bug: `nros build freertos` on
`examples/workspaces/cpp` failed with

```
freertos_entry_nros_main_generated.cpp: error: C99 designator 'node_name' outside aggregate initializer
```

and the boot-config block `codegen/entry/mod.rs` emits was blamed. **There is no
defect in that emitter.** The real cause is issue 1117: the build was compiling
with Ubuntu's `gcc-arm-none-eabi` 10.3.1 from `/usr/bin`, three major versions
below the `arm-none-eabi-gcc 13.2.rel1` this tree pins in `nros-sdk-index.toml`,
because the SDK store had never been provisioned with the toolchain.

After `nros setup --tool arm-none-eabi-gcc`, the pinned 13.2.1 compiles the same
generated file at `-std=c++14`, `c++17` AND `c++20` with zero errors, and the
image links.

## Both diagnoses were wrong, and how

Kept because the reasoning is instructive and because a retracted issue that
does not say what it got wrong teaches nothing.

**First:** "designated initializers are C++20; GCC accepts them as a `gnu++`
extension, so the host escaped by defaulting to `gnu++17` while the cross lane
passes `-std=c++14`." Wrong. Measured: host g++ 11.4 accepts the construct at
strict `-std=c++14`, and the cross compiler rejects it at `-std=gnu++20`. The
standard is not the variable.

**Second:** "a GCC 10 parser bug." True of GCC 10, and irrelevant — the project
does not use GCC 10. Being right about a compiler nobody in the supported set
runs is still the wrong answer.

The measurement that settled it, on `arm-none-eabi-g++ 10.3.1`:

| construct | result |
|---|---|
| designators, scalar members only | OK |
| `char n[8]` from a **string literal** via designator | error |
| `char n[8]` from a **braced char list** via designator | OK |
| any of the above at c++14 / gnu++14 / c++17 / gnu++17 / c++20 / gnu++20 | identical |

and on the pinned `arm-none-eabi-g++ 13.2.1`: no errors at any standard.

## What was nearly done because of this

A change to `codegen/entry/mod.rs` replacing designated with positional
initialization "for portability", which would have traded a self-documenting
generated file — one people read post-link — for compatibility with a compiler
this project does not ship. The issue also recommended raising the entry TU to
C++20, which would NOT have helped: GCC 10.3 rejects the construct there too.

The real lesson is issue 1117's: nothing said which compiler was being used.
