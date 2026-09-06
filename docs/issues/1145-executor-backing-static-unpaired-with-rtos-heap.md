---
id: 1145
title: "The executor backing is a `.bss` static now, but on an RTOS the allocator
  arena it used to come out of was never lowered — so those images reserve the
  same bytes twice, and nobody has measured it"
status: open
type: tech-debt
area: core
related: [phase-392, RFC-0002, 0163, 0880]
---

## Problem

phase-392 W6 moved the executor's per-entry storage off the heap and into a
named `.bss` static (`nros_node::executor::backing::EXECUTOR_BACKING`), because
`mem-report` reads symbols and a `Box::leak` has none.

On a **hosted** target that is free: the allocator is the OS heap, which has no
fixed reservation, so nothing was double-counted and the bytes simply became
visible. Measured on the native zenoh talker — see phase-392 W6.

On an **RTOS** it is not free, because the allocator arena is *itself* a fixed
static sized to hold this backing:

| platform | the knob that already reserves these bytes |
| --- | --- |
| Zephyr | `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE` (CLAUDE.md records "executor backing alone needs ~75 KB"; the default is 16 KB, so every Zephyr Rust image raises it) |
| FreeRTOS | `configTOTAL_HEAP_SIZE` / heap_4, which also draws the task stacks |
| NuttX / ThreadX / ESP-IDF | the port's own heap sizing |

Nothing lowers those knobs, so an RTOS image built after W6 carries the
reservation **twice**: once as `.bss`, once as heap headroom it no longer needs.

## Why it was not fixed in W6

The pairing is per-image and the second half can only be established by
building the image and measuring — lowering a heap knob by an argued number is
exactly the "never claim a saving you did not measure" rule this phase carries.
W6 built and measured **one** image, a native host ELF; it built no Zephyr,
FreeRTOS, NuttX, ThreadX or ESP32 image, and says so.

## What W6 left in place instead

An opt-out, so no RTOS image is forced to pay twice while this is open:

```
NROS_EXECUTOR_BACKING_U64S=0
```

emits no static at all and restores the `Box::leak`. A non-zero value overrides
the reservation's size. Read by `nros-node/build.rs` through the same
env → `$DOTCONFIG` reader every other executor knob uses (issue 0460).

**The Zephyr `CONFIG_` half is deliberately NOT declared yet, and declaring it
is part of closing this.** The reader already looks for
`CONFIG_NROS_EXECUTOR_BACKING_U64S` in the dotconfig, but `zephyr/Kconfig`
declares no such symbol, and a symbol nothing declares never reaches the
dotconfig — so on Zephyr today the env knob is the only interface. It was left
out because the obvious declaration is wrong in an interesting way: an
`int … default 0` would read as **"no static"** on every Zephyr image from the
day it landed, silently reverting the change for the whole platform, and there
is no other integer that spells "use the crate's default". Whoever does the
measurement below decides the shape — most likely a `bool NROS_EXECUTOR_BACKING_STATIC`
(`default y`) for the on/off decision plus the existing int for the size, one
symbol per decision.

## What closing this looks like

For each RTOS platform, in this order, and one platform per commit:

1. Build the platform's talker fixture at HEAD, record `just mem-report <elf> --json`.
2. Lower that platform's allocator-arena knob by the measured
   `EXECUTOR_BACKING` size, rebuild, and confirm the image still **boots and
   delivers** — not merely links. An under-sized allocator fails at the first
   allocation the executor no longer makes but something else still does.
3. Record the before/after in phase-392 W6's table with the exact command.

Do not do this as one sweeping commit: the failure mode is a runtime allocation
failure on one platform, and a six-platform diff makes that unattributable.

## Related

- phase-392 W6 — the change that created this, and the measurement that is done.
- Issue 0163 (archived) — where the "~75 KB from the picolibc arena" figure comes from.
- Issue 0880 — the other half of amendment A: a named section for the static
  (`NROS_EXECUTOR_BACKING_SECTION`) exists but nothing places it yet.
