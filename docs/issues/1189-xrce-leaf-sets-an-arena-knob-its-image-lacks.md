---
id: 1189
title: "The six Zephyr XRCE Rust leaves set `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE`,
  and their images do not select picolibc — so the knob never reaches the
  resolved `.config` and the comment above it describes another image"
status: open
type: bug
area: [embedded, build]
severity: low
found: 2026-09-06
related: [1145, 1171, 0876, 0163]
---

## What

Every `examples/zephyr/rust/*/prj-xrce.conf` carries:

```
# Issue 0163 — the Zephyr Rust global allocator is picolibc malloc, so the
# executor's ~75 KB leaked default backing (phase-271) and the in-image
# backend's buffers come from COMMON_LIBC_MALLOC_ARENA_SIZE (default 16 KB),
# NOT the kernel heap.
CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=1048576
```

The XRCE images do not use picolibc, so that symbol is never selected and the
line does nothing.

## Measured

Resolved `.config` of four Zephyr Rust images built from the same tree:

| image | `CONFIG_PICOLIBC=y` | `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE` |
| --- | :-: | --- |
| `listener` / xrce / native_sim | **no** | **absent** |
| `talker` / cyclonedds / native_sim | yes | 16688888 |
| `talker` / zenoh / native_sim | yes | 961320 |
| `action-server` / zenoh / mps2_an385 | yes | 960248 |

The XRCE image reports `CONFIG_ZEPHYR_PICOLIBC_MODULE=y` and
`CONFIG_FULL_LIBC_SUPPORTED=y` with `# CONFIG_PICOLIBC_MODULE is not set` — it
links the full libc, whose allocator is not the `COMMON_LIBC` arena.

## Why it matters

Three ways, in increasing order:

1. **The comment is wrong about that image**, and it is the comment a reader
   consults when sizing an XRCE board. It names picolibc as "the Zephyr Rust
   global allocator" without qualification.
2. **A 1 MiB number that looks load-bearing is not.** Anyone budgeting RAM for
   an XRCE image from this file starts from a figure the image never had.
3. **It nearly produced a false claim.** Issue 1145's sweep paired the arena
   against `EXECUTOR_BACKING` across all 18 Zephyr Rust confs, and the pairing
   arithmetic is checked textually by `check-executor-backing-arena-pairing` —
   which cannot know whether the symbol reaches the image. The six XRCE confs
   passed the gate while pairing a knob that does not exist. Caught by building
   one and reading its `.config`; the six were reverted and only the twelve
   picolibc confs paired.

## What is NOT wrong

`EXECUTOR_BACKING` **is** in the XRCE image — measured at 88,328 B, same as its
siblings. phase-392 W6 applies there like everywhere else. What is absent is the
fixed arena those bytes used to come out of, so on XRCE there is nothing to
lower and no double reservation to remove. The right answer for these six confs
is therefore "state nothing", which is what they now do.

## Fix direction

Delete the dead line and correct the comment, OR — if an XRCE image *should* be
on picolibc like its siblings, which is a real question nobody has asked here —
select it and keep the knob. **Do not simply delete without answering that**: a
silent libc difference between the RMW variants of the same example is a bigger
finding than a dead config line, and it may explain other divergence between
the XRCE and zenoh Zephyr lanes.

Same family as issue 0876 (a leaf Kconfig value that changes nothing because a
later fragment sets it), one mechanism over: here nothing overrides the value,
the symbol is simply not part of the image's configuration.

## Reproduce

```
west build -b native_sim/native/64 -d <dir> examples/zephyr/rust/listener -- \
  -DCONF_FILE="prj.conf;prj-xrce.conf;cmake/zephyr/native-sim-line-3.7.conf" …
grep -E 'PICOLIBC=|COMMON_LIBC_MALLOC_ARENA_SIZE' <dir>/zephyr/.config
```
