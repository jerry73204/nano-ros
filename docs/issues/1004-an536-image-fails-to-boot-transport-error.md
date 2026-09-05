---
id: 1004
title: "an536 boot failures on this host were HOST LOAD, not a code regression — and the measurements taken during them should not be trusted"
status: open
area: [rmw, platform, embedded]
severity: high
related: [0997, 1000, phase-177]
---

# The same image that ran for minutes this morning now halts at boot

## What happens

Autoware Safety Island, `freertos-an536` on QEMU `mps3-an536`, nano-ros pin
`d2a8955c5`:

```
[INFO] nros: Wall clock set from SNTP
[INFO] nros: Starting Controller Node...
[INFO] nros: Actuation Safety Island is Live
[nros] FATAL: ComponentNode "controller" failed at create_subscription (code=-100) — halting boot
```

`-100` is `nros::Result::TransportError` (`nros-cpp/include/nros/result.hpp:70`).

One run in three failed differently and earlier, before any component came up:

```
[INFO] nros: Wall clock set from SNTP
[INFO] nros: Starting Controller Node...
*** STACK OVERFLOW:  ***
```

with an EMPTY task name in the banner.

Those two are almost certainly one defect. `nros-board-freertos`'s own
`lwipopts.h` records the same pairing from phase 177.26, in as many words:

> `recvUC` overflowed on the first real ROS payload (a 13 KiB Autoware
> trajectory) with `*** STACK OVERFLOW: recvUC ***`, and — because the overflow
> lands in the adjacent heap — the SAME image also failed at
> `create_subscription` with a bad-free heap_4 assert when it booted into an
> already-populated graph.

So: a stack overflow corrupts adjacent heap, and the corruption presents later
as a `create_subscription` failure. The difference from 177.26 is that the
banner names no task, so which stack is overflowing is not yet known.

## CORRECTED — there is no bracket, and it is not a regression in those 31 commits

The table below was built from one or two observations per pin. Re-tested with a
scripted predicate that boots each build THREE times and calls a pin bad if any
attempt fails:

| pin | 3-attempt result |
| --- | --- |
| `d2a8955c5` (was "bad") | **BAD** — stack overflow on attempt 1 |
| `2b03606ca` (was "good") | **BAD** — attempt 2 |

**Both ends fail.** So the bracket in the original table does not hold, the
"regression inside 31 commits" framing is unsupported, and the correlation drawn
with island tracing being absent is unsupported with it. A bisect was prepared
and abandoned: with a noisy predicate it would have named an innocent commit with
confidence, which is the failure mode `AGENTS.md` records for issue 0268 — "a
first-bad that cannot plausibly cause the symptom means the verdicts tracked a
confounder".

What the retest also showed is that the run I called good was good: attempt 1 at
`2b03606ca` produced 9440 lines and ran the full 75 s. Attempt 2 produced NINE:

```
Network ready
[INFO] nros: ARM - Autoware: Actuation Safety Island
[INFO] nros: Wall clock set from SNTP
[INFO] nros: Starting Controller Node...
qemu-system-arm: terminating on signal 15 (timeout)
```

It reaches SNTP, starts the controller, and then hangs silently — no overflow
banner, no FATAL, no further output. That is a THIRD signature alongside the
stack overflow and the `create_subscription -100`, and all three land in the same
window: immediately after `Starting Controller Node`.

So the honest description is one nondeterministic hang at controller
construction, presenting three ways, on both pins — not a version regression.
Roughly one run in three on this host.

**Method note, because it caused the wrong issue to be filed:** a single boot
attempt is not evidence about a ~1-in-3 failure. The original bracket came from
1–2 samples per pin and was wrong in the direction that felt most explanatory.
Any future claim here needs a repeat count stated with it.

Environment checked and NOT the cause: the SNTP server is running throughout
(`sntp-server.py --bind 192.0.3.1`), and the failing boot reaches
`Wall clock set from SNTP`, so it is not a missing time source.

## Why this looked new (superseded by the correction above)

The same tree booted this image repeatedly earlier the same day. Sequence, all
on one host with one tap and one QEMU (`11.0.0-nros2`):

| pin | boots? | evidence |
| --- | --- | --- |
| `2b03606ca` | yes | full 7-size delivery sweep completed; later runs booted and ran for minutes |
| `d2a8955c5` | yes, WITH island tracing | ran 161 s past the SPDP stall, logging throughout |
| `d2a8955c5` | **no**, without tracing | 3 of 3 runs failed: two `create_subscription -100`, one stack overflow |

The last row is the current committed pin.

Two things are worth separating, because it is tempting to collapse them:

* **It is not the instrumentation.** The failure reproduces with the local
  Cyclone counters fully reverted (`git checkout` of `q_xevent.c`, verified
  clean) and with no `<Tracing>` block in `kEmbeddedCycloneConfig`.
* **It correlates with tracing being ABSENT.** Every long successful run at this
  pin had island-side discovery tracing enabled; every failure has it off.
  Tracing over semihosted stdout is slow, so this has the shape of a timing
  window that logging happens to close — which is a Heisenbug, and is why
  "add tracing to see it" is not a viable next step on its own.

## Why it matters beyond itself

It blocks [#0997](0997-island-announces-spdp-once-then-lease-expires.md) and
[#1000](archived/1000-spdp-periodic-event-orphaned-by-handler-early-return.md). Neither
can be confirmed or refuted while the image does not reach a steady state
deterministically, and #1000's proposed mechanism is already contradicted by the
one counter read that did complete (`nros_dbg_spdp_unknown_guid = 0`,
`nros_dbg_spdp_no_writer = 0` across three handler invocations).

It also means the an536 delivery numbers taken today cannot be trusted as a
before/after of anything, which is recorded in #0997 for the same reason.

## RETRACTED 2026-09-04 — the variable was host load, and it was never checked

Two eight-boot runs, same tree, same pin, back to back:

| build | result |
| --- | --- |
| with the ACKNACK fix below | 1 pass / 7 fail |
| unmodified | **0 pass / 8 fail** |

The unmodified tree is the same one that booted repeatedly and completed a
seven-size delivery sweep the same morning. At that failure rate 1/8 against 0/8
is noise: the fix neither helped nor hurt, and nothing here distinguishes them.

Then the machine was finally looked at:

```
load average: 24.93, 13.48, 9.77      (rising)
%CPU  COMMAND
 371  CarlaUE4-Linux-                 ~3.7 cores
99.9  FVP_BaseR_AEMv8                 a second ARM emulator
```

Memory was fine (40 GB available) and none of the load was leftover processes
from this investigation. A CARLA simulator and an FVP model — other work on a
shared box.

QEMU emulating a Cortex-R52 with a FreeRTOS tick and lwIP timeouts is acutely
timing-sensitive. Under this load, timing-dependent boot paths fail. That one
fact accounts for every confusing observation in this issue:

* **The "regression" between pins.** There was none. `2b03606ca` passed in the
  morning and failed in the afternoon; the load rose across the day.
* **Tracing "helping".** Semihosted logging slows the guest and moves the timing
  window. This issue already called that a Heisenbug without drawing the
  conclusion.
* **The worsening rate** — ~1 in 3, then 7/8, then 8/8 — tracks the load curve,
  not any code change.
* **The three signatures** (stack overflow, `create_subscription` TransportError,
  silent hang) are three ways a timing-skewed boot can fail, not three faces of
  one defect.

**The methodological failure is the useful part.** Across this investigation the
controlled variables were: the pin, island tracing, local instrumentation, the
codegen caps, and the C++ standard. `uptime` was never run. Every conclusion
about *when* the an536 image works was therefore drawn against an uncontrolled
confounder, and the earlier "3-attempt predicate" that retracted the bracket was
itself measured under the same contamination.

**Any an536 timing measurement in this issue, in
[#0997](0997-island-announces-spdp-once-then-lease-expires.md), and in the
autoware-safety-island delivery sweeps from the same period should be re-taken on
a quiet host before being cited.** A `uptime` reading belongs beside every
future number.

### What survives the retraction

The **ACKNACK livelock is real and load-independent**, because it was read out of
guest memory rather than inferred from timing: `tsched` pinned at
2,171,000,000 ns across two reads 200 s apart while `xTickCount` advanced from
140 s to 344 s, the value reproducible from config as
`t_last_nack + nack_delay`, and `AANR_SUPPRESSED_NACK` being the only case in
`make_and_resched_acknack` that re-arms without first setting
`t_last_nack = tnow`. That is a genuine upstream defect and is written up below.

What does NOT survive is the claim that it causes the boot failures. It was
fixed, and the boots did not improve.

## The ACKNACK livelock (real, but NOT the cause of the boot failures)

Caught a hang under the gdb stub and read the event queue. Connection health was
verified first (a live `xTickCount`, no `packet error` / `vMustReplyEmpty` in the
session) because an unhealthy stub returns zeros that read like data:

```
roots  = 0x217032d0                        <- tree NOT empty
min_ev   kind = 1 (XEVK_ACKNACK)
         tsched = 2,171,000,000 ns  (2.171 s)
tick   = 140304                     (~140 s)
```

`handle_xevents` loops on

```c
while (earliest_in_xeventq(xevq).v <= tnow.v)
```

and the earliest event is an ACKNACK stuck **138 seconds in the past**. The
condition is therefore permanently true: extract, handle, re-arm still in the
past, extract again — forever, holding `evq->lock`. That is why `app`, `recv` and
`dq.builtins` are all convoyed on the event-queue object
(`an536-blocked-on.py` reports them sharing one wait object) and the image never
reaches `Actuation Safety Island is Live`.

The `heapnode` offset used for the cast is the one `evq_xevents_fhdef` is built
from (`q_xevent.c:172`), and `kind = 1` is `XEVK_ACKNACK` in the enum at `:60`.

### Where the past time comes from

`make_and_resched_acknack` (`ddsi_acknack.c:387`) re-arms using absolute times
derived from PAST events rather than from now:

```c
(void) resched_xevent_if_earlier (ev, ddsrt_mtime_add_duration (rwn->t_last_nack, gv->config.nack_delay));
...
(void) resched_xevent_if_earlier (ev, ddsrt_mtime_add_duration (rwn->t_last_nack, gv->config.auto_resched_nack_delay));
```

`t_last_nack` + a small delay is ~2.171 s — exactly the `tsched` measured — while
the clock is at 140 s. `resched_xevent_if_earlier` accepts it because anything is
earlier than `DDS_NEVER`, so the event goes back on the heap already overdue and
is extracted again on the very next iteration.

**Not yet proven:** which of those branches is taken, and the actual values of
`t_last_nack`, `nack_delay` and `auto_resched_nack_delay` on the hung image.
Those are three more reads on a caught hang and would turn this from "the shape
matches" into "this is the line".

Also noted for the #1000 audit: `handle_xevk_acknack` has TWO early returns
(proxy-writer lookup fails, reader match lookup fails) that neither reschedule
nor delete — the same class of silent event loss that issue describes, in a
second handler.

### Two of my earlier readings here were wrong

Recorded because both were stated confidently and neither survived measurement:

* **"Confirmed spin at one instruction."** Seven gdb attaches reported an
  identical PC, which I read as a tight spin. Single-stepping advances normally
  (`+92 → +96 → +100 → +104 → +108`). Repeated attaches on QEMU halt at a
  consistent point, so attach-sampling is NOT a profiler and the identical PCs
  were an artifact of the stub, not of where time was spent.
* **"Corrupt circular sibling list."** `an536-fibheap-walk.py` walked it: a clean
  3-node list closing on the start node. The inner `do/while` terminates fine;
  the spin is the OUTER loop.

### Relationship to #0997

Probably NOT the same defect, and worth keeping apart. Here the tree is populated
with a stale event and the thread spins; in #0997 the tree was EMPTY
(`roots = 0x0`) and `tev` slept indefinitely. Opposite states of the same queue.

## Reproduce

```
# ASI at 3ebde20 or later, nano-ros pin d2a8955c5
sudo ip link set tap1 up            # 192.0.3.1/24, netem 100mbit
ASI_RX_COUNTERS=1 NROS_DOMAIN_ID=2 ./build.sh --platform freertos-an536
# boot with -net tap,ifname=tap1 and watch for the FATAL line
```

A gated runner is what made this visible rather than being mistaken for a
measurement — it requires the island to boot AND a publisher to match AND the
match to drop before it will call a run data, and it named which gate failed
each time. A run that fails to boot is otherwise easy to read as "the scenario
reproduced and nothing arrived", which is exactly the confusion that cost a day
here.

## What to establish first

1. **Attach to a hung instance.** The silent hang is the most tractable of the
   three signatures: it reaches `Starting Controller Node` and stops, so the
   image is alive and gdb can be attached to ask which task is stuck and on
   what. `an536-blocked-on.py` in the consumer repo answers exactly that, and it
   is what identified the condvar waiters in #0997.
2. **Which stack overflows.** The banner prints an EMPTY task name, so the
   FreeRTOS overflow hook reports without one. That reporting needs fixing
   regardless, because it is the difference between "some stack" and a fix.
3. **NOT a bisect.** Both ends fail; there is nothing to bisect until the
   failure is deterministic or the predicate is made reliable.
4. **Only then** return to #0997.
