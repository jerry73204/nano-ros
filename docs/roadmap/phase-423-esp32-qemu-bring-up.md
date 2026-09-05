# Phase 423 — the esp32-qemu board has no working runtime path

**Status (2026-09-04).** Not started — opened as a HOME for four issues that
had none. No work item here has been attempted; the sequencing below is a
decision about ORDER, not a claim of progress.

**Opened 2026-09-04 as a HOME, not as a plan.** Four issues filed against this
board on the same day describe one situation from four angles: nothing on
esp32-qemu currently runs end to end, and each layer fails for its own reason.
None of them had a phase, which is the shape phase-414 measured — an issue with
no home is an issue nobody is accountable for, and it does not merely sit, it
goes stale and gets re-diagnosed.

## Why these four are one phase

They are not four bugs that happen to share a board. They are a stack, and the
order matters: an image that cannot be packed cannot be booted, an image that
boots but drops its logs cannot be observed, and a cell that greps a marker
nothing prints cannot report on any of it. Fixing them in isolation would mean
each fix lands with no way to demonstrate it.

* **[#1025](../issues/archived/1025-esp32-flash-image-consumer-drops-the-row-variant.md) — the image
  cannot be built.** The flash packer asks for the group dir using the row's env
  string, so ESP32 flash images can never be produced. Everything below is
  unobservable until this is true.
* **[#1006](../issues/1006-esp32-qemu-configure-nondeterministic-deps.md) —
  the configure builds backends the board never uses.** Cost and blast radius,
  and it widens what a bring-up failure could be caused by.
* **[#1052](../issues/1052-esp32-talker-faults-after-network-bringup.md) — the talker
  takes an instruction-access fault right after network bring-up.** The runtime
  defect proper.
* **[#1048](../issues/1048-esp32-log-records-are-silently-dropped.md) — every `log::info!` is
  silently dropped**, so four e2e cells grep for a marker nothing prints. This is
  the OBSERVER, and it is the reason to sequence the phase rather than pick off
  whichever looks easiest: while it stands, the other three can only be debugged
  by inference.

## Sequencing, and the reason for it

1. **#1048 first.** A board whose diagnostics do not reach the console is one
   nobody can debug, and issue 0870 is this project's own record of what that
   costs — five diagnoses over three weeks against a platform whose `printk` arm
   compiled away. Fix the observer before the observed.
2. **#1025**, so an image exists to run.
3. **#1052**, which is then a fault with a console behind it.
4. **#1006** last: it is correctness of scope, not of behaviour, and it changes
   what the other three build.

## Acceptance

* One esp32-qemu cell runs end to end and asserts on output the board actually
  printed.
* The four e2e cells that currently grep an absent marker either pass or fail
  for a reason in the image rather than in the logger.
* Each issue is resolved or reassigned with the reason recorded.

## Deliberately not in scope

Real esp32 hardware. Every issue here is about the QEMU board, and a hardware
lane has different provisioning, different timing and no shared evidence with
these four.
