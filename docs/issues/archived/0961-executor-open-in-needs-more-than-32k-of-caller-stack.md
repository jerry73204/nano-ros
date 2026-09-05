---
id: 961
title: "Executor::open_in needs more than 32 KiB of the calling thread's stack, and nothing says so"
status: resolved
area: core, memory
severity: high
found: 2026-08-31
related: [0900, 0271, 0563, 0739, phase-403, phase-409, phase-3]
---

# The init path costs more stack than a small part can give it

## What was measured

mr-canhubk344 (S32K344, 320 KiB SRAM), RFC-0043 C++ components over zenoh
on serial, `CONFIG_MAIN_STACK_SIZE` bisected against a booting image:

| main stack | outcome |
| ---: | --- |
| 8192 | overflow |
| 16384 | overflow |
| 28672 | overflow, NAMED once `CONFIG_HW_STACK_PROTECTION=y` |
| 32768 | no named overflow, but `main` corrupts the idle thread's stack |

Named report at 28672:

```
>>> ZEPHYR FATAL ERROR 2: Stack overflow on CPU 0
Current thread: main
Faulting instruction address (r15/pc): __aeabi_memclr8
r14/lr:                                 Executor::open_in
```

So the requirement is somewhere above 32 KiB, on a part with 320 KiB of SRAM
in total. The consumer is a `memclr` reached from `open_in`.

## Why it is hard to see

**Without the MPU stack guard the overflow does not report as one.** `main`
runs off its stack into whatever is next -- on this image the idle thread's
stack -- and the fault surfaces later, in a different thread, as
`USAGE FAULT: Illegal load of EXC_RETURN into PC` with `pc = 0`. That reads as
memory corruption anywhere in the image, and it names the WRONG THREAD:
`Current thread: idle`. Two separate bring-up sessions were spent on the idle
thread before the guard was enabled.

The guard is not free on a tight image (it selects `MPU_STACK_GUARD`, which adds
a guard region to every thread stack -- measured 23244 B over on this one), so
the tool that names the failure is the tool such an image cannot afford. That is
the same shape as issue 0900's note about the arena being invisible to
`mem-report`.

## What is NOT the cause

* Not the executor arena. Holding the stack at 28672 and taking
  `NROS_EXECUTOR_ARENA_SIZE` from 49152 to 40960 changed nothing -- same
  `MMFAR`, same registers.
* The FPU registers in the fault dump are stale. `s[3]`/`s[4]` read `0x0000c000`
  in both runs, which looks like an arena size and is not one; it did not track
  the knob. Do not infer from them.

## Why it matters beyond one board

`open_in` is on every image's boot path. A hosted target with an 8 MiB main
thread never notices; a 320 KiB part cannot pay it, and gets a fault that names
neither the cost nor the thread that owns it. phase-403 makes receive buffers
type-sized, which took this image's arena from 86108 B to 24516 B -- and none of
that helps, because the stack requirement is independent of it.

## Located (2026-08-31): `Executor` is a ~16 KiB VALUE, moved twice

Frame sizes read from the linked image with `objdump`, largest first:

```
19840  ZenohSession::names_and_types_filtered   (not on the boot path)
16000  Executor::open_in                        (sub.w sp, sp, #16000)
15104  nros_cpp_init
11712  Executor::register_action_server_raw
```

`open_in` and `nros_cpp_init` are the same call chain, so roughly 31 KiB of
prologue is committed before either function does any work. Both frames are one
value: `open_in` builds an `Executor` on its stack and returns it BY VALUE, and
`nros_cpp_init` holds the returned value before `ptr::write`ing it into the
caller's storage. `size_of::<Executor>()` is about 16 KiB, cross-checked against
the generated `NROS_EXECUTOR_SIZE`: it moved by exactly the arena delta (90112 B)
across two builds, leaving Executor plus tables at 22888 B.

**What is inline in a struct whose tables were supposedly moved out.**
phase-271 (issue 0110) moved six sized tables to borrowed storage and left
others behind. The dominant one:

```rust
group_sched_table: heapless::Vec<
    (String<64>, String<64>, String<32>, SchedContextId),
    { crate::config::MAX_CBS },
>
```

about 168 bytes per slot, INLINE, scaled by `MAX_CBS`. Also inline:
`extra_sessions: heapless::Vec<ConcreteSession, MAX_NODES>` (6 x 524 B here),
the `SessionStore` itself, `nodes`, `dispatch_slots`, `component_slots`.

**So `MAX_CBS` costs stack, not just arena.** Raising it from 14 to 36 to fit
this image's 33 handles added roughly 3.7 KiB to the frame of every function
that moves an `Executor`. That coupling is invisible at the knob: nothing in
`NROS_EXECUTOR_MAX_CBS`'s help says it grows the main thread's stack, and the
failure it produces is a stack overflow in a function the knob does not name.
Same shape as `NROS_ZEPHYR_TASK_STACK_SIZE` inheriting `MAIN_STACK_SIZE`.

## The fix, and one that does NOT work

**Rejected: boxing the table.** Tried and reverted. `alloc` is optional in
`nros-node` (`#[cfg(feature = "alloc")] extern crate alloc`), and the `params`
field that looks like a precedent is behind `param-services`. A `Box` compiles
on the std lane and breaks any `no_std` target without alloc -- which is most of
the targets this crate exists for. It also trades stack for heap on parts where
both are scarce.

**The fix is to finish phase-271:** move the remaining `MAX_CBS`- and
`MAX_NODES`-scaled members into the carved `backing` alongside the six tables
already there. That works with no allocator, puts the storage where the CALLER
chose (`.bss` for a static holder, and it is already sized by
`ExecutorSizing`), and removes the coupling rather than relocating it. It
touches `executor/storage.rs`'s `carve` + `ExecutorSizing`, which the C FFI
sizes `_opaque` from, so the generated sizes move with it -- deliberate, and
covered by the existing size-probe gates.

**Independently, STATE the number.** A boot-time check like the heap gate
(`nros: the executor arena cannot fit in the platform heap`) naming the required
main-thread stack would have turned four bring-up sessions into one build error.

## Also found

`ZenohSession::names_and_types_filtered` carries a 19840-byte frame -- larger
than `open_in`. It is not on the boot path, so it did not cause this, but any
image that calls graph introspection on a small part will hit the same wall.

## phase-409 landed the fix (2026-08-31); still open on the board

All nine members are carved now (`CarvedVec<'s, T>` in `executor/storage.rs`).
Measured on the host, not inferred:

* `size_of::<Executor>()` 5072 -> 1016 at the shipped knobs, and 12768 -> 1016 at
  this image's `MAX_CBS=36` / `MAX_NODES=6`. The SAME number at both, which is
  the coupling this issue is about being gone.
* `Executor::open_in`'s prologue 9544 -> 3368 bytes and `nros_cpp_init`'s
  7992 -> 1816, read with `objdump` from a linked x86_64 image over
  `libnros_cpp.a`. At this image's knobs the two together were 35040 bytes and
  are now 5184. x86_64, so not the Cortex-M numbers above; the ratio is.

Two things this did NOT do, and they are why the issue stays open:

1. ~~**The board has not booted it.**~~ **MET 2026-08-31, recorded here
   2026-09-03.** phase-409 carved the remaining members and the island entry
   boots on `mr_canhubk3/s32k344` at `CONFIG_MAIN_STACK_SIZE=16384` — half the
   32768 that was previously the smallest workable value
   ([phase-409](../../roadmap/phase-409-executor-inline-storage.md), Acceptance 3;
   superproject commit `8d586b2`). Measured on the same two frames this issue
   names: `Executor::open_in` 16000 → 2244 and `nros_cpp_init` 15104 → 1396,
   so 31104 bytes of prologue on that call chain became 3640.

   This criterion was written before that landed and stayed unticked for three
   days, which is the failure mode this repo keeps recording in the other
   direction: a HIGH-severity issue whose text overstates what is still broken
   is as misleading as one that understates it.
2. **Nothing STATES the number yet.** The boot-time check this issue asks for --
   a gate naming the main-thread stack an image requires, the way the heap gate
   names the arena -- is not written. The requirement is smaller now; it is still
   unnamed, and that is the half of this issue that cost four bring-up sessions.

`ZenohSession::names_and_types_filtered` (19840 bytes) is untouched.

## Where the "state the number" check can and cannot live (2026-09-03)

Scoped but NOT implemented. Recording the constraints, because the obvious two
placements are both wrong and finding that out costs a build each.

**It cannot go inside `Executor::open_in`.** The overflow happens in that
function's PROLOGUE — `sub.w sp, sp, #16000` before phase-409, and the named
fault above points at `__aeabi_memclr8` with `lr = open_in`. Any check written
in the body runs after the stack has already been claimed, so it cannot fire on
the image it is meant to protect. The same argument rules out `nros_cpp_init`,
which is the other half of the same call chain and commits its own frame first.
A runtime check has to sit in the CALLER, before the call — which means the
board's entry, per platform.

**The configure-time route needs a probe that does not exist yet.** The heap
gate this issue asks to imitate works because both of its numbers are knobs
resolved at configure. The stack requirement is not: it derives from
`size_of::<Executor>()`, which is only knowable by compiling the crate. There IS
a probe of that shape — `nros-build-helpers::c` reads a compiled rlib and emits
`EXECUTOR_OPAQUE_U64S` — but that constant is `size_of::<ExecutorInlineStorage>()`,
the executor value PLUS its carved backing. The backing is not on the stack, so
reusing it would overstate the requirement, and on a part where the overstatement
is a build error that is not a safe direction to be wrong in.

So the work is: add a probe for `size_of::<Executor>()` alone, emit it beside
the existing macros, and compare `2 x` it (the issue's own "one value, moved
twice" finding) against `CONFIG_MAIN_STACK_SIZE` — as a NECESSARY condition, not
a sufficient one. The frames hold locals besides the executor: measured on
x86_64 after phase-409, `open_in` is 3368 and `nros_cpp_init` 1816 against a
1016-byte `Executor`, so `2 x size_of` is a floor and roughly a third of the
real cost.

Being explicit about that ratio matters: a gate that claims to state THE number
while checking a third of it is the shape this campaign keeps finding — a value
that reads as applied and is not. Either it says "necessary minimum" in its own
message, or it waits for a measured frame budget.

## The "state the number" check, IMPLEMENTED (2026-09-04)

Built to the scope above, including its caveat: the check says "necessary
minimum" in its own message rather than claiming to be the requirement.

**The probe.** `nros::sizes` gains `EXECUTOR_VALUE_SIZE = size_of::<Executor>()`
beside the existing `EXECUTOR_SIZE = size_of::<ExecutorInlineStorage>()`. They
are far apart, which is the reason the section above refused to reuse the
existing one — read out of a freshly built host rlib:

| symbol | bytes |
| --- | --- |
| `EXECUTOR_SIZE` (value + carved backing) | 89,816 |
| `EXECUTOR_VALUE_SIZE` (what a stack pays) | **1,552** |

Reusing `EXECUTOR_SIZE` would have overstated the stack requirement by ~58x, and
on a 320 KiB part that overstatement is a build error refusing an image that
fits.

**What is emitted**, by both header producers (`nros-build-helpers::{c,cpp}`)
and both `nros-c` templates:

```c
#define NROS_EXECUTOR_SIZE           89816
#define NROS_EXECUTOR_VALUE_SIZE     1552
#define NROS_EXECUTOR_MAIN_STACK_MIN 3104
```

**Where the check sits.** In the generated header, so it lands in the CALLER's
translation unit — which is what this issue's "cannot go inside `open_in`"
finding requires, since the overflow happens in that function's prologue. It is
`#if defined(CONFIG_MAIN_STACK_SIZE) && CONFIG_MAIN_STACK_SIZE <
NROS_EXECUTOR_MAIN_STACK_MIN` → `#error`, so it is present exactly where Zephyr
force-includes `autoconf.h` and simply absent elsewhere rather than wrong.
`NROS_STACK_MIN_ACKNOWLEDGE` opts out for an image that builds its executor off
the main thread, where the number is about a stack this header cannot see.

**Mutation-tested, not assumed** — the three cases, compiled:

| `CONFIG_MAIN_STACK_SIZE` | expected | result |
| --- | --- | --- |
| 2048 | fail | `#error` fires, naming the number |
| 16384 (the tree's real minimum) | pass | clean |
| 2048 + `NROS_STACK_MIN_ACKNOWLEDGE` | pass | clean |

**Would it have caught the original?** Yes, and the arithmetic is worth stating
rather than asserting. The bisect at the top of this issue overflowed at 16384.
That was BEFORE phase-409, when the executor value was ~16 KiB — so the floor
this check computes would have been ~32 KiB, and 16384 would have failed to
compile with a message naming the number. At today's post-phase-409 sizes the
floor is 3104 and every in-tree image (minimum 16384) passes, so the check is
silent on the current tree and armed against a regression in `MAX_CBS`,
`MAX_NODES`, or anything else that grows the value.

**What this still does NOT do.** ~~The board has not booted it -- item 1 of the
two above is unchanged and needs the part.~~ **That sentence was wrong when
written (2026-09-04).** Item 1 was met on 2026-08-31 and says so in its own
text three sections up: the island entry boots on `mr_canhubk3/s32k344` at
`CONFIG_MAIN_STACK_SIZE=16384`. Two statements about the same criterion, in one
file, disagreeing — the same failure this issue already recorded once, in the
same direction: a HIGH-severity issue overstating what is still broken. And the floor remains roughly a third of
the true frame cost, by this issue's own x86_64 measurement; it is a necessary
condition and its message says so in those words.


## RESOLVED 2026-09-05 — both criteria met, and the check is now GATED

Both acceptance criteria are satisfied:

1. **The board has booted it** — met 2026-08-31 on `mr_canhubk3/s32k344` at
   `CONFIG_MAIN_STACK_SIZE=16384`, half the previously smallest workable value,
   with `Executor::open_in` 16000 -> 2244 and `nros_cpp_init` 15104 -> 1396.
2. **Something states the number** — `NROS_EXECUTOR_MAIN_STACK_MIN` is emitted
   with its `#if` / `#error` guard, in the caller's translation unit, by all
   four producers: `nros-build-helpers::cpp` inlines the text, `::c` substitutes
   `@EXECUTOR_STACK_MIN@` into `nros_config_generated.h.template` and
   `nros_config_generated_exact.h.template`, which carry it.

### What was missing until now: nothing kept it there

The check had no gate and no test. It was mutation-tested by hand on 2026-09-04
and then nothing would have noticed it going away — which matters more here than
usual, because both ways of losing it are SILENT:

* drop the `#define` and the `#if` compares against an undefined identifier,
  which the preprocessor reads as `0`, so the guard passes for every stack size;
* keep the `#define` and drop the `#if` and the number becomes documentation —
  which is precisely the state this issue was filed about.

`scripts/check-executor-stack-floor.py` (gate `check-executor-stack-floor`)
asserts every producer still defines the floor, still compares against it, still
has an `#error`, and still offers the `NROS_STACK_MIN_ACKNOWLEDGE` opt-out.
Source-level deliberately: the generated headers are build artifacts and a gate
in an affordability tier may not resolve one, but the disarming regression is
visible in the emitters without building anything.

Mutation-tested against a REAL producer, not only synthetic text: deleting the
`#if` line from `nros_config_generated.h.template` fails the gate naming that
file. Its selftest runs on the normal path so the controls cannot rot, and one
of those controls earned its keep immediately — the first `#define` regex used
`\s+`, which matches a NEWLINE, so a valueless `#define` followed by the `#if`
line passed. Anchored to `[^\S\n]` now.

### What is deliberately still true

The floor is a NECESSARY condition, roughly a third of the true frame cost by
this issue's own x86_64 measurement, and its message says so in those words. An
image below it provably cannot boot; one above it may still not. The value is
that the failure arrives at compile time naming the number, instead of as a
prologue overflow four bring-up sessions deep.
