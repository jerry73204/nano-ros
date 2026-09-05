# Phase 358 — Embedded runtime under load: footprint, overrun, overload

**Status (2026-08-21). FOUR of five DONE; W1 RESTATED (its premise refuted, #271 closed); W3 DONE — its acceptance met by a read-task budget, and one of its own conclusions refuted in the process.** This header
said "PLANNING — nothing implemented" while ELEVEN commits named the phase — my
error, corrected here.

* **W1 (#271, Orin SPE footprint)** — RESTATED 2026-08-21, was BLOCKED. #271 is
  now RESOLVED and archived at 91 % recovered, with the remaining ~11 KB
  consumer-side. The re-measurement still cannot run here (the three API
  removals are real, and the `autoware_sentinel` repro is not on this host), but
  the seam its consumer port waits on — phase-346 — COMPLETED 2026-08-12, before
  the entry that recorded the block. **This phase's premise did not survive**:
  see W1 below.
* **W2 (#505, timer overrun)** — DONE. The policy is written down where the
  decision belongs (`74badc8f2`); #505 resolved.
* **W3 (#506, transport budget)** — OPEN, and the blocker cleared twice over:
  #567 landed, then `7eb86ae05` bumped zenoh-pico because the read TASK can keep
  a remainder too. `af5f5d7e4` corrects a claim made from reading a diff rather
  than running it. #506 open.
* **W4 (#579, NuttX boot tier priority)** — DONE. The tier adopts its declared
  priority, and the GATE that accepted the bug was fixed alongside
  (`64fee4e60`, `8424db51c`). #579 resolved.
* **W5 (#557, Zephyr Cyclone boot)** — DONE. "Three layers of hiding, each one
  covering the next" (`b838fc19d`, `a45c36abe`). #557 resolved.

**Owns:** [issue 0271](../issues/archived/0271-orin-spe-btcm-footprint-regression.md),
[issue 0505](../issues/archived/0505-timer-backlog-replay-no-overrun-policy.md)
(**RESOLVED** — W1's own body already says so),
[issue 0506](../issues/0506-transport-band-unbounded-preemption.md),
[issue 0557](../issues/archived/0557-zephyr-cyclone-action-ddsrt-thread-reuse.md),
[issue 0579](../issues/archived/0579-nuttx-boot-tier-priority-never-applied.md)
(**RESOLVED** — the body's W-tail already records 0579 and 0583 resolved).

**Related:** [issue 0567](../issues/archived/0567-zpico-rx-cannot-resume-partial-buffer.md)
(RESOLVED 2026-08-14 — this UNBLOCKS #506's device half; see W3), [phase-352](archived/phase-352-platform-clock-ns.md)
(COMPLETE), [phase-349](phase-349-rtos-integration-shells.md),
`docs/reference/platform-implementation-notes.md`.

## The common shape — no policy, so the default is "whatever happens"

* **#505** — after a stall, periodic timers replay the whole backlog (6 control
  callbacks 88 µs apart). No overrun policy, and no overrun counter, so it is
  invisible as well as unbounded.
* **#506** — transport above application tiers is the right default but has no
  budget; inbound overload preempts every tier for ~200 ms bursts.
* **#579** — the NuttX boot tier never adopts its declared priority, so a
  `[tiers.*.nuttx] priority` ordering can silently INVERT.
* **#271** — a footprint regression nothing gates, so the image stopped fitting
  256 KB between two commits.
* **#557** — a Zephyr boot failure that the readiness timeout HIDES.

In each case the system does something defensible-looking and wrong, without
saying so.

---

## W1 — Re-measure the Orin SPE footprint before acting (#271)

The regression is ~+195 KB between `d9af52be` and `21a3a4248`; a minimal
`Executor::open`+spin image no longer fits 256 KB BTCM.

**This number is likely stale in the project's favour.** `58d271471`
(2026-08-15, issue 0563) landed "carve the remap table — Executor 11632 → 4992
bytes". Re-measure before designing anything; a large part of #271 may already
be recovered.

CLAUDE.md's rule applies directly: re-measure any perf number on cleanly rebuilt
fixtures before acting on it (archived issues 0148/0164).

**Acceptance.** A current BTCM figure for the minimal image on this tree, and
either #271 closed with evidence or restated with the remaining delta. If it
still overflows, a size gate is the follow-on — the regression went unnoticed
because nothing measured it.

**ATTEMPTED 2026-08-15 — the figure cannot be produced, and the reason is the
result.** The repro lives in `autoware_sentinel`, which pins nano-ros by git rev
— and that pin is still `d9af52be`, the GOOD one, so the sentinel as checked out
builds the image that FIT. Re-measuring means bumping to current main, where
three things it names are gone: `nros-board-orin-spe` (crate removed),
`platform-orin-spe` (feature removed — phase-337 W7.b, a back-compat alias for
`platform-freertos`), and `rmw-zenoh` (retired by RFC-0054; the same removal
that had broken `just book`, issue 0581).

So W1 is a CONSUMER PORT before it is a measurement, and until that port lands
#271's number can never be refreshed — which blocks anything downstream that
needs to know the current footprint. Recorded in the issue with the table of
what moved. This phase's suspicion that `58d271471` already recovered much of
the regression is plausible and remains UNTESTED; testing it needs an armv7r
build of the minimal image, i.e. the port.

A host-side `EXECUTOR_OPAQUE_U64S` under #271's knob set was measured (18031
u64s ≈ 144 KB) and deliberately NOT offered as the answer: the budget is a
256 KB BTCM on armv7r, where pointers are half the width.

**RESTATED 2026-08-21 — the premise this wave was built on does not hold.**

W1 opens "This number is likely stale in the project's favour … a large part of
#271 may already be recovered" by `58d271471` (Executor 11632 -> 4992). That
commit did not delete the remap table. It CARVED it out of the struct into
caller-owned storage, in its own words:

> The required backing grows by the same bytes, but that lands in the caller's
> STATIC buffer instead of on the stack, which is the entire point.

6,664 bytes moved from a stack temporary to static backing. It fixes a STACK
overflow (issue 0552), not image size — the bytes never left the image, and
#271's remaining 14,404 were all `.bss`. So re-measuring is unlikely to show the
recovery this wave hoped for, and the urgency that justified "no footprint work
before W1's re-measurement" is largely gone.

**#271 is CLOSED** (2026-08-21) at 91 % recovered — 168,760 bytes of overflow ->
~11,268 once `NROS_RMW_MESSAGE_INFO_SLOTS=8` is applied — with everything
remaining consumer-side. Its durable half became issue 0739 (the static-pool
inventory), which is DONE and needed no consumer port at all.

**What is left of W1**, honestly: a current BTCM figure is still unproduced and
still needs the consumer port, which is now possible (phase-346 landed the
out-of-tree board seam) but is consumer work, not nano-ros work. It is no longer
a blocker on anything in this tree — the size-gate follow-on this wave named can
be designed against the pool inventory instead of against a number nobody can
refresh.

## W2 — Timer overrun policy and counter (#505)

Two separable pieces:

1. **A counter** — make overrun observable. Small, and it turns every future
   report of this from anecdote into data.
2. **A policy** — what *should* happen after a stall: replay all, replay one,
   skip to now, or configurable. This is a design decision and should be stated
   in an RFC or in the RT scheduling design rather than chosen implicitly by the
   implementation.

Do (1) first; it is independently useful and it informs (2).

**Acceptance.** An overrun is counted and reportable. The policy is written down
with its rationale before it is coded, and the default is stated in the docs.

**DONE 2026-08-15 — and this item was STALE when I picked it up.** It says "do
(1) first"; (1) and (2) had both been implemented on 2026-08-11, along with a
microsecond-resolution fix the item never anticipated. Issue 0505's own status
section said so; the phase doc did not. Verified in code before believing either
— `#[default] Skip`, `elapsed_us %= period_us` (phase-preserving), saturating
`overruns`, `timer-overrun-runtime` in `monitor.rs`, microseconds end to end
including the C ABI's ns→µs conversion.

What was genuinely outstanding was the half the code cannot satisfy: the policy
*written down*. Until now the rationale lived only in the issue and in
doc-comments — precisely the "chosen implicitly by the implementation" outcome
this item set out to prevent. Now in **RFC-0002 § 4.4a**, with the measurement
that actually justifies the `Skip` default (under `CatchUp` a stalling tier
reports 100.03 Hz on a declared 100 Hz loop, so the rate monitor reports health
during the fault it exists to catch), and the user-facing default stated in
**book/src/user-guide/configuration.md**.

One part of the acceptance cannot be met retroactively: it asked for the policy
to be written down BEFORE it was coded, and it was not. Recorded rather than
glossed. Issue 0505 resolved; three follow-ups left explicitly out of scope there
(no diagnostics drain on the FreeRTOS lane, policy not declarable in launch
metadata, period/spin quantization silent).

## W3 — A budget for the transport band (#506)

The history is worth stating precisely because it was measured: issue 0567 found `_zp_unicast_read` RESETS its receive buffer on every call, so
the drain loop cannot stop early without losing frames. Capping the loop at 4/16
frames improved cadence (stalls 10 → 4/5, missed periods 1.79 % → 0.59/0.85 %)
but collapsed inbound delivery 282 → 10 msg/s — **a drop policy, not a budget**.

**That blocker has since cleared.** #567 was RESOLVED on 2026-08-14: the reset
is now conditional in the zenoh-pico fork (`43ddb0ec`), which the superproject
pointer already carries. So the resumable rx path #506 was waiting on exists,
and this work item is **actionable now** — it was written as blocked and is not.

Re-read #506 against the post-`43ddb0ec` runtime before designing: the numbers
in it were measured against a receive path that reset unconditionally, so both
the overload behaviour and the cost of a cap may differ.

**Acceptance.** A budget that bounds preemption WITHOUT reducing steady-state
delivery. The control #567 established is the baseline any proposal must beat: a
cap of 1 degenerates to the pre-loop single-frame path and matched unbounded on
every column, while 4/16 improved cadence and collapsed delivery 282 → 10 msg/s.
Report the same columns so the comparison is direct.

**REVISITED 2026-08-15 — blocker confirmed cleared IN CODE; both tables are now
stale, including the baseline.** `43ddb0ec` makes the reset conditional (reset
only when the buffer is empty, else `_z_zbuf_compact`), and its own message says
the unconditional reset "is why … a budget on that loop is lossy rather than
deferring work". So **#567's conclusion — "a frame cap here is a drop policy,
not a budget" — is no longer true by construction**: an early exit leaves the
remainder buffered.

One amendment to the acceptance above: the #567 control cannot be *the baseline*
as written, because it too was measured pre-`43ddb0ec`. It has to be re-taken
alongside the capped cells. The falsifiable question is narrow — does a cap
still cost delivery? If inbound rx/s holds near unbounded while stalls and miss%
improve, the frame cap IS the budget and the remaining design work is choosing
the cap and exposing the deferral counter.

NOT run here: the four columns come from `NEWSLabNTU/nano-ros-rt-eval` on the
FreeRTOS mps2-an385 QEMU lane, which is not present on this host, and nothing
in-tree measures them (`nros-bench/stress-zenoh` is a native throughput bench).
Same shape as W1 — the analysis is in-tree, the measurement lives in a consumer
repo. Details and the restated experiment are in issue 0506.

**CORRECTED 2026-08-16 — the paragraphs above are wrong on both counts, and
BUILDING it is what showed that. They are kept, not edited, because the mistake
is the lesson: a diff was read and a conclusion drawn, twice, without running
anything.**

1. **The harness was never a blocker.** `nano-ros-rt-eval` is public, clones in
   seconds, pins nano-ros at `c10371776` (post-#567), and runs on the FreeRTOS
   QEMU lane this host already has. "Not present on this host" was true and
   irrelevant.
2. **The blocker had NOT cleared for this lane.** `43ddb0ec` fixed
   `_zp_unicast_read` — the POLLED path. `nm` on the built image shows it
   exports `_zp_unicast_read_task`: the `Z_FEATURE_MULTI_THREAD` path, which
   still reset unconditionally at the end of every peer's turn. So on exactly
   the lane #506 and #567 were measured on, a cap was still a drop policy. The
   claim "no longer true by construction" was construction-only, and wrong.

Found because capped images came out byte-identical to the control, through a
chain where each theory was disproved in turn (env var, cargo freshness, a
`cargo clean`, the issue-0475 relink trap, an `#error` that never fired) and
ended at `nm`.

**Done since:** the conditional reset is ported to the task path, guarded on the
transport having a single peer — `_zbuf` is SHARED across the peer list, so
carrying a remainder across peers would feed peer A's stream to peer B; client
mode (the island) is always single-peer. Landed as zenoh-pico `f4ce3d9f` and
pinned. Measured neutral without a budget, which is the expected result and the
point: delivery did not collapse, so carrying a remainder does not corrupt the
stream.

**ANSWERED 2026-08-16 — the constant does not reach codegen because NEITHER cap
site is in the image, and the four-column table cannot be produced as specified.**

`nm` on the FreeRTOS mps2-an385 island built from current main: the image
contains `_zp_unicast_read_task`, `_z_unicast_client_read` and
`_z_unicast_process_messages`, and does NOT contain `_zp_unicast_read`,
`_zp_unicast_process_peer_event` or `_z_unicast_peer_read`. Statics are visible
in that output, so the absences are real.

Both cap attempts sat in functions this image does not link. A value in dead
code folds away — that is the whole of "cap=1/4/16 produced one identical
binary", with all three differing from unbounded because the `#if` around the
site changed the TU regardless.

Two corrections to the entries above, both of which this build overturns:

* **`43ddb0ec` does not apply to this lane** — it fixed the POLLED
  `_zp_unicast_read`, absent here.
* **`f4ce3d9f`, recorded above as "Done since", is dead code on this lane** —
  it lives in `_zp_unicast_process_peer_event`, whose only call site is behind
  `#if Z_FEATURE_UNICAST_PEER == 1`, and `nros-zpico-build` emits
  `#define Z_FEATURE_UNICAST_PEER 0`. The port is correct; it is not reached.
  Its "measured neutral" result follows from that and evidenced nothing.

The live path has **no inner drain loop**: one `_z_unicast_client_read` + one
`_z_unicast_process_messages` per iteration of the read task's loop, and over a
TCP stream link `to_read` is a single frame. The lane is already at cap = 1,
losslessly. Unbounded / 16 / 4 / 1 are not four configurations of this image.

So #506's premise needs restating, not its experiment re-running: rx runs on its
own FreeRTOS task, so the preemption measured is a scheduling property (priority
and CPU share against the tiers), not an unbounded loop the app calls. A budget
is the wrong instrument for that. Detail and the two follow-on options in #506.

**Blocked on the way, fixed and filed as #0621:** the cells need the harness
pin bumped to current main, and doing that broke the consumer's build — a
vendored nano-ros splices its 272 example packages into the CONSUMER's package
index, and the first duplicate (`demo_bringup`, ×18 by design) ends their build
while naming two directories they do not own. Fixed with `.nros-ignore` at the
repo root, which `build_pkg_index` ignores when nano-ros IS the root
(`depth() == 0` is exempt) and honours when it is nested. Three packages also
genuinely collided (`native_talker` ×3, including a *listener* declaring itself
a talker) — renamed. With both fixed the island builds against current main,
which is what made the `nm` above possible.

**Still open:** no budget exists, and the four-column table does not either. A
budget built on top did not differentiate cap=1/4/16 in codegen — all three
produced one identical binary — so it was dropped rather than shipped unproven.

That "why" is now ANSWERED above (2026-08-16): both cap sites are absent from
the image, so the value had nothing to apply to. The cells do not follow from
it, because unbounded/16/4/1 are not four configurations of this lane. What
remains is a design choice, not a measurement:

* bound the READ TASK — its priority and CPU share against the tiers, which is
  what actually preempts them here; or
* take the polled-path cells on an image that links `_zp_unicast_read`
  (`Z_FEATURE_MULTI_THREAD=0`, the multi-executor build), where `43ddb0ec`
  makes a frame cap defer rather than drop.

Either is a fresh work item with its own acceptance. Detail in #506.

**DONE 2026-08-21 — option 1 taken, and this item's acceptance is MET.**

The acceptance above asks for "a budget that bounds preemption WITHOUT reducing
steady-state delivery", reported in the same columns. Measured on this lane, six
to twelve runs per cell, interleaved:

| cell | n | stalls/run | worst | rx/s med | chain % | chain p50 |
|---|---|---|---|---|---|---|
| unbounded | 12 | 9.1 | 610 ms | 249 | 10.2 | 39 ms |
| budget 8 | 12 | **0.0** | **21 ms** | 899 | 11.6 | 85 ms |
| budget 32 | 6 | **0.0** | **38 ms** | 386 | 12.4 | 38 ms |
| budget 128 | 6 | 12.5 | 132 ms | 386 | 8.8 | 56 ms |

Preemption bounded (12/12 budgeted runs at zero stalls against 6/6 unbounded
with 2-13, no overlap), delivery not reduced (chain flat-to-better; rx too noisy
on this harness to claim either way), and no chain-latency cost — that last one
checked separately and answered NO, the apparent p50 difference being an artifact
of a bimodal distribution whose p50 sits in the valley.

**One sentence above is refuted by that result and is left standing as the
lesson.** "A budget is the wrong instrument for that" — for a scheduling
property — is wrong. A frame budget ON THE TASK is a budget, and it is the right
instrument: the mechanism is not rate limiting but bounding the task's
CONTIGUOUS run, which is exactly the scheduling property the trace attributed.
The sweep gives it a closed form, `worst_gap = 0.94 x FRAMES + 11 ms`, which
predicts the observed pass/fail boundary and makes 128 frames worse than
unbounded on stall COUNT.

What that leaves for #506 is implementation, not investigation: the `ingress`
declaration, the router-rule emitter, the budget landed in the vendored fork
behind a Kconfig knob, and the counters. RFC-0074 carries the compile relation
and the two resolve-time constraints; its remaining open questions are 1 and 3,
both answered, and 2/4/5, all now closed by the per-frame cost `c`.

Option 2 (polled-path cells on a `Z_FEATURE_MULTI_THREAD=0` image) stays
available but is lower value: no shipped configuration on this lane links that
path.

## W4 — NuttX boot tier drops its declared priority (#579)

A `[tiers.*.nuttx] priority` ordering can silently invert. Filed 2026-08-14
alongside a stdout panic hook.

This is the smallest and most clearly-wrong item in the phase: a declared value
is not adopted. It also has a nasty adjacency — the NuttX `pthread_attr_t`
mirror overflow (#569/#570/#572) was three issues written from three symptoms of
one overflow, and priority handling sits in the same area. Check whether #579 is
a fourth symptom before treating it as independent.

**Acceptance.** The boot tier runs at its declared priority, verified by
observing the ordering rather than by reading the code. The check for
"fourth symptom of the mirror overflow" is recorded either way.

**DONE 2026-08-15, except the runtime observation — which is blocked, and the
blocker is new.** The fix landed (`64fee4e60`): the boot tier adopts its
declared priority through the same shim its own board's C arm uses. Then the
guest run the acceptance demands turned up something else.

* **The gate was narrower than the rule.** `sched_dims_applied_e2e`'s
  tier-priority cell asked only whether the accept marker appeared ANYWHERE in
  the log; the spawned tier's line satisfied that for the whole image, so the
  cell was green throughout #579. Replaced with a per-tier, per-value shape
  (`EachTierOrFailNote`) — the issue-0196 class, and the reason the knob could
  be accepted and discarded unnoticed.
* **"Fourth symptom of the mirror overflow?" — no.** #579 already establishes
  this from an execution trace and records why the misreading repeats; verified
  independently here: `check-nuttx-libc-struct-sizes` is green
  (`pthread_attr_t` 56 B vs mirror 56 B).
* **The ordering could not be observed** — filed as **issue 0583**. On the
  `workspace-rust-nuttx-realtime` fixture the boot tier never resumes after
  spawning the low tier, so it never reaches the priority call; being the
  session owner, its stall means nothing is flushed and the router drops the
  guest on lease expiry ~7 s in. Evidence: guest console, a NIC packet dump
  (one TCP connection, guest silent after ~7 s, unanswered router FINs), a
  revert-rebuild at `64fee4e60^` producing an identical console, and the C++ arm
  of the SAME board running the same workspace correctly for 60 s at the
  expected ~10:1 tick ratio.

**W4 COMPLETE later the same day — 0583 was fixed, and the observation landed.**
0583 turned out not to be a scheduling bug at all: the image linked a `std`
compiled 2026-08-10 against crates.io `libc`'s 20-byte `pthread_attr_t` while
NuttX's is 56, so every thread spawn wrote 36 bytes past the attr on
`Thread::new`'s own frame and the caller returned to ~0. Issue 0570's fork fix
never reached those artifacts because the workspace fixture signature was blind
to the vendored libc and these rows set `skip_probe = true`. Fixed by hashing the
pin into the signature AND dropping the build-std artifacts when it moves.

With that cleared, the guest shows exactly what this item asked for:

```
nros: tier priority set tier=`high` prio=110      <- the boot tier
nros: tier priority set tier=`low`  prio=100
nros: tier `high` alive — 3000 spin(s), 2437 timer(s) fired, 0 error(s)
nros: tier `low`  alive —  300 spin(s),  142 timer(s) fired, 0 error(s)
```

Ordering observed rather than read: the ~10:1 ratio matches the declared 1 ms /
10 ms periods, both tiers publish, and the guest survives the full run. Issues
0579 and 0583 both resolved.

## W5 — Zephyr Cyclone action images fail at boot, hidden by a timeout (#557)

`tid … is in use!` and `rc=-100` at boot; the readiness timeout converts an
immediate, specific failure into a slow, generic one.

Fix the hiding first — that is phase-356's principle applied here, and it is
usually cheap. A boot failure that reports itself is a different debugging
problem from one that times out.

Note the Zephyr-specific hazard already documented: Cyclone on Zephyr now uses a
NATIVE ddsrt sync backend (`DDSRT_WITH_ZEPHYR` picks the types,
`nros_rmw_cyclonedds.cmake` swaps the TU) and **both halves move together or the
layouts disagree** — and `k_mutex` is recursive where a pthread NORMAL mutex
deadlocks, so a self-relock bug hangs natively and passes on Zephyr.

**Acceptance.** The boot failure surfaces as itself, with `rc=-100` and the tid
message attributed. Then the underlying cause.

**DONE 2026-08-15 — both halves.** Three layers were hiding, each covering the
next, and the item's own instinct (fix the hiding first) was what made the cause
reachable.

* **The test** reported the wait, not the failure. Fixed and guest-verified. One
  correction to the fix itself: `first_guest_failure` scanned in LINE order and
  led with `<err> os: tid … is in use!`, four lines above the real error. It is
  rank-major now, with `is in use!` ranked low because it is BENIGN — Zephyr's
  `pthread_attr_destroy` frees the stack of the thread it just created, logs
  `-EBUSY` and returns 0 (leaking 32 KB each time). The tid message is
  attributed, as the acceptance asked, and attributed correctly: to nothing.
* **The FFI** discarded the `NodeError` (`Err(_) => TRANSPORT_ERROR`), and the
  mapper it should have used had a `_` arm swallowing eight more variants. Both
  fixed; the wildcard is gone entirely (rustc now rejects it as unreachable, so a
  new variant fails to compile until someone maps it). `rc=-100` became `rc=-1`.
* **The cause**: `ros_form_to_dds` appended a trailing `_` to a name that already
  had one, and the doubled underscore defeated issue #234's idempotence guard,
  producing the very doubled type name that guard prevents. Only C/C++ reached it
  — Rust advertises the DDS form, which returns early. One-line fix.

Verified after a clean rebuild of every Cyclone leaf: `case_17` (C), `case_18`
(C++), `case_16` (Rust control) and `case_14` (C service) all PASS, ~8 s each
against a 60 s readiness timeout. Issue 0557 resolved; 0586 and 0589 filed for
debt this uncovered.

---

## Deliberately not doing

* **No new executor.** Every item here is a policy or a diagnostic gap in the
  runtime that exists.
* **No footprint work before W1's re-measurement.** Explicitly the trap this
  phase opens by naming.
* **Not touching #532** (platform clock resolution). Phase 352 is COMPLETE and
  its title claims exactly that scope; #532 should be checked for staleness
  against it rather than re-planned here.

## Issues homed here (survey 2026-09-03)
Every open issue was checked for a home phase; these had none, or were
mentioned here only in passing. A mention is not an owner — an issue with
no work item is an issue nobody is accountable for, which is the same shape
as a gate sitting in a lane no CI job runs. Each row is a work item: the issue
holds the evidence, the item is *close it*.

| issue | why it belongs here |
| --- | --- |
| [#0902](../issues/0902-action-goal-completion-is-variable.md) | action goals complete between 20 % and 90 % of the time on the same build — a load-dependent success rate, which is this phase's subject |
| [#0913](../issues/0913-the-debugger-is-not-a-passive-instrument.md) | attaching pyocd RTT kills the zenoh session: the debugger perturbs the thing being measured |
| [#0917](../issues/0917-an536-fragmented-sample-never-syncs.md) | the emulated LAN9118 RX FIFO cannot hold an 8-fragment RTPS burst — an overrun with a known cause |

