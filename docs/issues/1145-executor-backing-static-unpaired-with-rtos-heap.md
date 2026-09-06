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

**The Zephyr `CONFIG_` half is DECLARED now** — issue 1171 did it, because the
hand-copied subtrahend this issue left behind could not be fixed without it.
The concern recorded here was real and the resolution is not the `bool` this
paragraph guessed at: `int … default -1`, the tree's DERIVE sentinel (issue
0940), which `dotconfig_usize` already reads as "no value" and so leaves the
crate's own sizing in place. `0` keeps its documented meaning of "no static".
An image that lowers its arena STATES the word count, and the reservation is
`8 x` it on every target — see issue 1171 for why a measured number could not
be right for both boards this conf builds for.

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

## Zephyr, one leaf: DONE 2026-09-06

`examples/zephyr/rust/talker` (zenoh), both boards it builds for. `malloc_arena`
is an `nm`-visible symbol, so both halves of the pairing read off one ELF.

| symbol | before | after |
| --- | ---: | ---: |
| `malloc_arena` (mps2_an385) | 1,048,576 | 961,320 |
| `EXECUTOR_BACKING` (mps2_an385) | 87,256 | 87,256 |
| `EXECUTOR_BACKING` (native_sim/native/64) | 88,328 | 88,328 |

Lowered by the SMALLER of the two boards' backings (87,256), so neither board
loses headroom it had before W6.

Whole-image RAM on mps2/an385, west's own report:

```
before   RAM: 1789196 B / 4 MB  (42.66%)
after    RAM: 1701940 B / 4 MB  (40.58%)
delta         -87,256 B
```

Exactly the backing size. **Boots and delivers, not merely links**:
native_sim published 20 messages in 25 s — identical to the pre-change control
run — and a stock ROS 2 consumer received them
(`ros2 topic echo /chatter std_msgs/msg/String --once` -> `data: 'Hello World: 9'`).

### Found on the way, and fixed first

The Zephyr build could not run at all. Two blockers, both filed and neither
this issue's:

* **issue 1167** — a red on `main` from the same day: `#if ZPICO_MAX_SESSIONS < 1`
  sat ABOVE the knob's default, so every Zephyr zenoh image failed to compile.
  Two sessions found it hours apart from opposite ends and fixed it in opposite
  directions; `91f3ce7c9` is the one that shipped, and the gate that asks
  whether a guard can FIRE is on its own branch.
* **issue 1166** (filed the same day, on its own branch) — the self-hosted
  runner builds Zephyr into a developer checkout and claims the shared west
  build dir, so `west` refused this measurement's first build. Worked around
  here with a private `-d`; that issue is the fix.

## Zephyr, swept: DONE 2026-09-06

All twelve Zephyr Rust confs whose image actually has a picolibc arena now pair
it, using the STATED mechanism from issue 1171 rather than a copied `nm` figure:

```
# nros-arena-base: <base>
CONFIG_NROS_EXECUTOR_BACKING_U64S=11041
CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=<base - 88328>
```

`8 * 11041 = 88,328` is the derived size on the larger of the two targets a
Zephyr Rust leaf builds for, so it is at or above the requirement on both, and
`nros-node` refuses to compile naming the knob if it ever is not.

No leaf overrides an executor knob (`MAX_CBS`, `MAX_SC`, `MAX_NODES`, the
arena), which is why one number serves all twelve.

| base | new arena | confs |
| ---: | ---: | --- |
| 1,048,576 | 960,248 | 6 zenoh |
| 16,777,216 | 16,688,888 | 6 cyclonedds |

Verified by BUILDING, since the compile-time refusal is the check:

| image | `EXECUTOR_BACKING` | `malloc_arena` |
| --- | ---: | ---: |
| `action-server` / zenoh / mps2_an385 | 88,328 | 960,248 |
| `talker` / cyclonedds / native_sim | 88,328 | 16,688,888 |

`action-server` is the heaviest role in the tree, so a stated size that suffices
there suffices for every lighter role at the same base. Whole-image RAM on
mps2/an385: **1,701,948 B / 4 MB (40.58 %)** — level with the already-paired
talker's 1,701,940, which is the point.

### The six XRCE confs were REVERTED, and that is the finding

They set `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE` and their images **do not select
picolibc**, so the symbol never reaches the resolved `.config`. Pairing them
would have been arithmetic over a knob that does not exist — and it PASSED the
textual gate, which cannot know. Caught by building one and reading its
`.config`. Filed as issue 1189.

`EXECUTOR_BACKING` is present in the XRCE image (88,328 B, same as its
siblings), so W6 applies there; what is absent is the fixed arena those bytes
used to come out of. There is nothing to lower, and "state nothing" is correct.

## Still open

* ~~Every other Zephyr Rust leaf.~~ **DONE** — see the sweep above. Twelve confs
  paired; the six XRCE ones deliberately not, because their images have no
  picolibc arena to pair (issue 1189).
* **FreeRTOS, NuttX, ThreadX, ESP32** — untouched. One platform per commit, per
  the plan above, because the failure mode is a runtime allocation failure and a
  six-platform diff makes it unattributable. The knob is Zephyr-only so far: the
  other ports have no Kconfig, so their spelling is still the `NROS_*` build env.
* ~~**The subtrahend is a hand-copied literal.**~~ RESOLVED —
  [issue 1171](archived/1171-arena-backing-pairing-is-hand-maintained.md).

## Related

- phase-392 W6 — the change that created this, and the measurement that is done.
- Issue 0163 (archived) — where the "~75 KB from the picolibc arena" figure comes from.
- Issue 0880 — the other half of amendment A: a named section for the static
  (`NROS_EXECUTOR_BACKING_SECTION`) exists but nothing places it yet.
