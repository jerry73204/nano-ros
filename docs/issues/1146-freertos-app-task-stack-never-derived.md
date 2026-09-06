---
id: 1146
title: "The FreeRTOS app-task stack is 384 KiB by bisection, its C/C++ mirror says
  512 KiB, and the reason all three documents gave for the number stopped being
  true five phases ago"
status: open
type: tech-debt
area: boards
related: [phase-392, phase-76, 0271, 0739, 1145]
---

## Problem

`nros_board_common::freertos_config::DEFAULT_APP_STACK_BYTES` is **393216**
(384 KiB), drawn from the FreeRTOS heap. It has never been derived from
anything. Its own doc comment is a bisection log:

> The default is 384 KiB: the Rust zenoh executor can exceed 160 KiB opening a
> FreeRTOS session with lwIP up, and the phase-212 Entry / run-plan Executor
> open overflows the older 256 KiB (issue #46). […] A 10-node macro entry's
> register pass overflows even 384 KiB, hence the override.

Three separate things follow from that, and all three are open:

1. **The number is unattributed.** Nobody knows how much of the 384 KiB is the
   zenoh-pico session open, how much is the executor-open call chain, and how
   much is headroom. No high-water measurement exists.
2. **The C/C++ carrier disagrees.** `cmake/templates/freertos_app_config.c.in`
   sets `.app_stack_bytes = 524288u` — 512 KiB, a third independent number.
   The divergence is documented there but not justified by a measurement
   either.
3. **Nothing sets the override.** `NROS_FREERTOS_APP_STACK_KB` exists and no
   file in the tree uses it, so every FreeRTOS example runs at the default.

## The stale explanation, corrected

phase-392 W6 was to "re-derive one over-large task stack against the new
figure — the FreeRTOS action examples' 64 KB app task". **Every noun in that
sentence was stale**, which is why W6 could not carry it:

| the claim | the fact |
| --- | --- |
| `APP_TASK_STACK` is 64 KB | `APP_TASK_STACK` was **deleted in phase-76**. The live knob is `app_stack_bytes` and the default is 384 KiB — six times larger, in the opposite direction from the one the wave predicted. |
| the executor's arena is inline on that stack | `Executor` has held `arena: &'s mut [MaybeUninit<u8>]` since phase-271 (issue 0110). On FreeRTOS the backing is a `.bss` static (phase-392 W6). Nothing arena-sized is in that frame. |
| the action examples pin `NROS_EXECUTOR_ARENA_SIZE=8192` | No example in the tree sets that knob. It is pre-phase-271 prose in `docs/guides/freertos-lan9118-debugging.md`. |

So the wave's hypothesis — "the stack is carrying the arena, drop it and the
number falls" — is **already false**: the arena left that frame five phases
before the wave was written. Whatever the 384 KiB is holding, it is not the
arena, and lowering the knob on that reasoning would have been a guess dressed
as a measurement.

## What deriving it actually requires

A FreeRTOS image, a stack high-water reading, and a run — not a link:

1. Build `examples/qemu-arm-freertos/rust/action-server` (and the `c`/`cpp`
   twins, which take a different init path — `Node::global_storage()` rather
   than the Rust `alloc` constructor, so their frames differ).
2. Enable `configRECORD_STACK_HIGH_ADDRESS` / `uxTaskGetStackHighWaterMark`
   and read the app task's mark after the registration pass, which is the
   deepest point the doc comment names.
3. Attribute the peak: session open vs `Executor::open_in`'s frame vs the
   register pass. `Executor::open_in` builds an `Executor` in its own frame and
   returns it by value, so the header's size is in there too (phase-409 shrank
   it; that shrink has never been measured against this knob either).
4. Set the default from the measurement plus a stated margin, reconcile
   `freertos_app_config.c.in` to it or document why the C carrier differs, and
   put the command in the commit message.

Acceptance is a booting image that still delivers, at the lower number — the
failure mode here is `lwIP ASSERT: Invalid mbox` from a stack overflow
corrupting `tcpip_mbox`, which is silent until it is fatal.

## Related

- phase-392 W6 — landed the visible-arena half; this is the task-stack half it
  could not honestly carry.
- Issue 1145 — the same "the bytes moved, the knob that reserved them did not"
  shape, one layer over, on the RTOS allocator arena.
- Issues 0271 / 0739 — the knob-nobody-can-enumerate class this belongs to.
