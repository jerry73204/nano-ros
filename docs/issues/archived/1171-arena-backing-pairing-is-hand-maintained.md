---
id: 1171
title: "The picolibc arena is lowered by a hand-copied `EXECUTOR_BACKING` size, so an
  executor knob change silently re-creates the double reservation it removed"
status: resolved
type: tech-debt
area: [core, embedded, build]
related: [1145, 0163, 0460, 0940, 1166, phase-392]
---

## What

Issue 1145 paired the two halves of phase-392 W6 on Zephyr: the executor
backing became a `.bss` static, so `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE` no
longer has to hold it, and the leaf conf was lowered by the measured size.

The number in the conf was a **literal, copied from `nm` output**:

```
CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=961320   # 1048576 - 87256
```

`EXECUTOR_BACKING`'s size is `ExecutorSizing::DEFAULT.u64_len()`, which is
derived from the executor knobs (`MAX_CBS`, `ARENA_SIZE`, `RX_BUF_SIZE`, …).
Move any of those and the subtrahend is wrong, in whichever direction, and
nothing checked it.

## The premise, re-verified before fixing it

All of it still held on `origin/main` at `bd204b986`:

* `examples/zephyr/rust/talker/prj-zenoh.conf` still carried `961320`, and it
  is still the only conf in the tree that does — every other Zephyr Rust leaf
  is still at `1048576`.
* `EXECUTOR_BACKING_DEFAULT_U64S` is still `ExecutorSizing::DEFAULT.u64_len()`,
  whose inputs are `config::{MAX_CBS, MAX_SC, ARENA_SIZE, MAX_NODES}` — all
  knob-derived in `nros-node/build.rs`.
* No gate referenced `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE`.

**One thing the issue understated.** The derived size is not only knob-dependent,
it is TARGET-dependent — the executor's carved tables hold pointers. Measured
here, from one and the same conf:

```
mps2_an385            EXECUTOR_BACKING = 0x154d8 = 87,256 B
native_sim/native/64  EXECUTOR_BACKING = 0x15908 = 88,328 B
```

So a single subtrahend could never pair both boards, and 1145's — the smaller of
the two, chosen so neither board lost headroom — left `native_sim` still
double-reserving 1,072 bytes. That is the sharper form of the argument for
fixing the mechanism rather than repeating the measurement.

## The fix: state the size, do not measure it

`zephyr/Kconfig` declares `NROS_EXECUTOR_BACKING_U64S`, defaulting to the tree's
`-1` DERIVE sentinel, and `nros-node`'s build script already read it through the
env → `$DOTCONFIG` path every executor knob uses (issue 0460). A leaf that lowers
its arena now writes three lines:

```
# nros-arena-base: 1048576
CONFIG_NROS_EXECUTOR_BACKING_U64S=11041
CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=960248
```

The reservation is then `8 * words` bytes — a `[MaybeUninit<u64>; N]`, the same
size on every target — so the pairing is exact by CONSTRUCTION rather than by a
number someone kept up to date. The remaining obligation is one-directional: the
stated size must not be BELOW what the executor needs, and that is the `const`
assertion `executor::backing` already carried, which now has something to
compare against:

```
NROS_EXECUTOR_BACKING_U64S is below the default executor sizing, so the
reservation can never be taken and is pure dead weight; use 0 to decline
the static entirely
```

A knob move that grows the backing is therefore a compile error naming the knob,
where before it was a silent partial re-creation of the double reservation.

### Why the other two candidate shapes lose

The issue listed three. Both rejected ones fail for the same reason, one level
apart:

1. **A post-link gate on the ELF** (assert `malloc_arena <= base - sizeof
   EXECUTOR_BACKING`) checks the right thing and can never run: it needs a built
   Zephyr image, and **no merge-gating lane builds Zephyr** (the fact that let
   issues 1158 and 1167 reach `main`). It would also be a check with no fix
   attached — it tells you the number is stale without telling you what it is.
2. **Publish the size from `nros-node/build.rs` and generate the conf** crosses a
   one-way boundary in the wrong direction. `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE`
   is resolved in the Zephyr **Kconfig** stage, which completes before the crate
   that defines the size is compiled; the build script's answer arrives after the
   arena has already been baked into `autoconf.h`. Issue 0460 is this same wall
   from the other side, and its answer — make the Rust side READ the dotconfig —
   is the direction that works, which is what shape 3 uses.

Kconfig has no arithmetic and its preprocessor runs before symbol values exist,
so "let Kconfig compute `base - 8 * words`" is not available either; the two
numbers stay two numbers, and a gate relates them.

## The gate

`check-executor-backing-arena-pairing` (`scripts/`, fast lane, derived
membership). Over every tracked `*.conf`:

* a conf stating `CONFIG_NROS_EXECUTOR_BACKING_U64S=<n>` with `n > 0` must set
  `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE` and carry `# nros-arena-base: <bytes>`,
  and `arena + 8 * n == base`;
* the marker and the statement are required of each other, both ways — a base
  with no statement says the arena was lowered by nothing in particular;
* a conf may not set the symbol at all unless `zephyr/Kconfig` DECLARES it.
  Kconfig silently ignores an assignment to an unknown symbol, so such a line
  reads as a decision and reaches nothing — issue 0460's failure mode one layer
  up, and the single most likely way this mechanism would be quietly reverted.

Last-wins on repeated assignments, because that is how Zephyr merges fragments
(issue 0876).

**Mutation-tested against the real tree**, not only in its self-test — the
lesson of issue 1167, where `check-c-array-pool-floors` keyed on a guard
EXISTING rather than on whether it could fire:

| mutation to `prj-zenoh.conf` | verdict |
| --- | --- |
| (none) | OK — 1 conf pairs, 156 scanned |
| arena `960248` → `961320` (1145's old number) | FAIL, naming the file and `1048576 - 8 * 11041 = 960248` |
| words `11041` → `12000` (a knob grew, arena not moved) | FAIL, same line |
| delete the `# nros-arena-base:` marker | FAIL — "no `# nros-arena-base: <bytes>`, so the arena's lowering cannot be checked against anything" |
| delete the `CONFIG_NROS_EXECUTOR_BACKING_U64S` line | FAIL — marker with nothing to pair |
| remove the symbol from `zephyr/Kconfig` | FAIL — "zephyr/Kconfig does not declare it … the line reads as a decision and reaches nothing" |

Ten self-test cases run on the NORMAL path (not behind `--self-test`), per
AGENTS.md: a negative control nobody runs decays into a comment.

`packages/tooling/nros-zephyr-build` also gained
`the_derive_sentinel_is_not_a_size`, pinning the behaviour every `-1 = derive`
knob in the tree already depends on: `dotconfig_usize` returns `None` for `-1`
because `"-1".parse::<usize>()` fails, and nothing said so. A future reader
"fixing" that to parse an `i64` would hand every one of those knobs `-1 as usize`.

## Measured

`examples/zephyr/rust/talker`, `prj.conf;prj-zenoh.conf;<board conf>`, each board
built twice in the SAME build directory with only the conf changed between the
two builds:

```
mps2_an385 (cmake/zephyr/qemu-cortex-m3.conf)
                     before                     after
  EXECUTOR_BACKING   0x154d8 =  87,256 B        0x15908 =  88,328 B   (+1,072)
  malloc_arena       0xeab28 = 961,320 B        0xea6f8 = 960,248 B   (-1,072)
  image RAM                 1,701,940 B                1,701,940 B    (   0  )

native_sim/native/64 (cmake/zephyr/native-sim-line-3.7.conf)
  EXECUTOR_BACKING   0x15908 =  88,328 B        0x15908 =  88,328 B   (    0 )
  malloc_arena       0xeab28 = 961,320 B        0xea6f8 = 960,248 B   (-1,072)
```

Read with

```
<sdk>/arm-zephyr-eabi-nm -S --size-sort <build>/zephyr/zephyr.elf \
  | grep -iE 'EXECUTOR_BACKING|malloc_arena'      # mps2
/usr/bin/nm            -S --size-sort <build>/zephyr/zephyr.exe \
  | grep -iE 'EXECUTOR_BACKING|malloc_arena'      # native_sim
```

(the host default `nm` here is LLVM's and dies with `Invalid encoding` on a
native_sim image, hence the absolute path).

**On mps2 this is deliberately a ZERO-delta result, and that is the claim.** 1145
already took the 87,256 bytes out; this is not another saving, it is the same
saving made durable. mps2's static grows 1,072 because it now reserves the
STATED 11,041 words rather than its own derived 10,907, and its arena falls by
exactly the same 1,072 — a move, not a cost. `native_sim` is where the pairing
was previously WRONG: its derived backing was already 88,328 while the arena had
only been lowered by 87,256, so it kept 1,072 bytes reserved twice. Those come
back. Nothing here re-measures 1145's `-87,256`; that measurement stands and is
not re-claimed.

**The refusal was exercised too, not reasoned about.** With
`CONFIG_NROS_EXECUTOR_BACKING_U64S=100` the mps2 build stops at `cargo build`
with exit 101:

```
error[E0080]: evaluation panicked: NROS_EXECUTOR_BACKING_U64S is below the
default executor sizing, so the reservation can never be taken and is pure dead
weight; use 0 to decline the static entirely
```

## Not done, on purpose

The **sweep** — `listener`, `service-client`, `service-server`, the workspace
`zephyr_entry` leaves, and FreeRTOS / NuttX / ThreadX / ESP32 — is 1145's
remainder and stays open there. It is now mechanical for Zephyr: three lines per
leaf, no `nm` run, and a leaf whose executor needs more than the stated count
fails to compile naming the knob instead of silently under-paying. The other RTOS
ports have no Kconfig, so their spelling is still the `NROS_EXECUTOR_BACKING_U64S`
build environment variable.

Whether the arena should be 1 MiB at all is still not this issue.

## Found on the way

**A private west workspace built from symlinks does not isolate the tree.** The
documented workaround for issue 1166 (the self-hosted runner owning the shared
`zephyr-workspace`) is a private `-d`; a private *workspace* whose `zephyr` is a
symlink into the shared one is not enough. Zephyr's cmake runs `west list` with
`WORKING_DIRECTORY ${ZEPHYR_BASE}`, python's `getcwd()` returns the PHYSICAL
path, and west then resolves the PRIMARY workspace's topdir — so the nano-ros
module, and every `zephyr/Kconfig` in it, came from the primary checkout while
the Rust lane compiled this worktree. Measured: the build's own
`zephyr/kconfig/sources.txt` named the PRIMARY checkout's `zephyr/Kconfig`
rather than the worktree's, so the first "before" build of this issue was a
two-tree image that would have been reported as a clean baseline. `cmake -DZEPHYR_MODULES=<list>` bypasses west module discovery and is
the isolation that actually holds.

## Related

- Issue 1145 — the pairing this makes durable; its sweep is still open.
- Issue 0460 — knobs reach the Zephyr Rust lane only through `$DOTCONFIG`.
- Issue 0940 — where the `-1` DERIVE sentinel comes from.
- Issue 1167 — the gate-that-cannot-fire class this one is mutation-tested against.
- Issue 1166 — the shared west build dir; see "Found on the way".
- RFC-0002 § 4.4b, phase-392 W6 — the move that created the pairing.
