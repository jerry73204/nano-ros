# Phase 414 — RTOS runtime correctness: the e2e failures that reproduce SOLO

**Status (2026-09-04). ALL FIVE WORK ITEMS CLOSED.** W3 — issue 0870 — was closed green with the cause UNATTRIBUTED, an owner decision taken on 383 consecutive passes across three build regimes and two emulators, and it records the objection it overrides plus the two experiments that reopen it without argument. The phase stays ACTIVE for the three issues it adopted later (#0997, #1004, #0968), not for its original five.** Opened
as a HOME, not as a plan: five open issues had no phase that could hold them,
and the survey that found that is the whole reason this doc exists — an issue
with no home is an issue nobody is accountable for, the same shape as a gate
that sits in a lane no CI job runs.

**The phase's central question is answered: W2 and W3 do NOT share a cause.**
The phase does not shrink; but W2 turned out to be fixed already and W3's real
blocker is one layer below where three diagnoses had been aimed.

**Two of the five were not defects in the state the issues described them in.**
W2 was fixed on 2026-08-28 and never closed. W1's evidence predates the fix for
issue 0906 by one day. That is a second instance of this phase's own premise:
an issue with no owner does not merely sit, it goes STALE, and a stale issue
costs more than an open one because it is read as current.

## Why these five are one phase, and why the two neighbours could not take them

They are RUNTIME failures on a real RTOS: the image builds, links and boots, and
then does the wrong thing. That is a different activity from either neighbour:

* **[phase-349](phase-349-rtos-integration-shells.md)** is BUILD integration —
  "make FreeRTOS an imported library like the rest". It is about how the RTOS
  enters the build, not about what the image does afterwards.
* **[phase-358](phase-358-embedded-runtime-under-load.md)** is footprint,
  overrun and overload — failures that appear when you PUSH the runtime. These
  five fail at rest.

The distinguishing property, and the reason they are worth grouping: **each
reproduces SOLO.** CLAUDE.md's standing advice for a QEMU red is to retest it
alone before believing it, because full-sweep lanes flake under load. These
already survive that test, so they are not the flake class — they are defects
with a stable reproduction and no owner.

## Work items

Each is an existing issue. The item is "close it"; the issue holds the evidence.

* **W1 — [issue 0877](../issues/archived/0877-freertos-pubsub-passes-by-hand-fails-under-harness.md),
  FreeRTOS pubsub delivers NOTHING under the test. REDIAGNOSED, not yet
  closed.** The issue's evidence is dated 2026-08-29; issue **0906** (every
  zenoh-pico session dropping every ~20 s because `Z_TRANSPORT_LEASE` was 10 s
  against a 30 s router keep-alive) was fixed on 2026-08-30, and its own
  reproduction is this exact FreeRTOS image pair, measured 19 heard of 77
  before and 77 of 77 after.
  **MEASURED: every built FreeRTOS fixture in the tree still bakes
  `Z_TRANSPORT_LEASE 10000` while the source says 60000** — museum binaries
  carrying precisely the defect 0906 fixed. Worse, the staleness probe cannot
  see it: the constant lives in `nros-zpico-build`, a build-script DEPENDENCY
  crate that never appears in `zpico-sys`'s recorded `cargo:rerun-if-changed`
  set, so the probe reports FRESH. That probe gap is issue-0196 class and
  wants its own issue.
  A second, harness-side defect is real regardless: `wait_for_output` KILLS the
  talker 15 s in (`rtos_e2e.rs:729`, `qemu.rs:448`) — it is a run-to-completion
  wait aimed at a free-running 1 Hz publisher — and 20 s settle + 15 s life +
  30 s listener wait is the reported ~65 s exactly. The service and action
  shapes use `collect_until` and let the server live, which is why they pass on
  the same host.
  **VERIFIED 2026-09-03: rebuilt, bake confirmed 60000, cell is GREEN — 9 of 9
  across all three languages, 12 published / 12 heard, ~35.4 s each.** The
  symptom does not reproduce.
  **But the attribution to 0906 is REFUTED, by counterfactual rather than by
  doubt.** Putting the lease back to 10000 and rebuilding still passes 6 of 6.
  The cell kills its talker 15 s in, so it emits 12 publishes and the first
  lease lapse (~20 s of session life) never arrives — the constant is invisible
  here. The rebuild fixed it; 0906 did not. The real cause is somewhere in the
  twenty days Aug 20 -> Sep 3 and is NOT identified (best untested candidate:
  `7cb213c43`, lan9118 RX driven from the interrupt, which fits "delivers by
  hand, nothing under the harness").
  **Corollary that outlives the issue: this cell can never regression-test
  0906**, and nothing else currently does.
  **CLOSED 2026-09-04, accepted green with the cause UNATTRIBUTED** — an
  owner decision, not a conclusion. The residual risk is stated in the issue:
  if the real fix is later reverted, this symptom returns and neither the cell
  (bounded, issue 1013) nor the probe (blind, issue 1005) will catch it. The
  bisect over Aug 20 -> Sep 3 is the first thing to do if it does.
  The `queue.c:1673` assert this issue also records is **issue 0899, already
  resolved** — the same 0906 session churn one layer down.
* **W2 — [issue 0867](../issues/archived/0867-nuttx-c-action-goal-send-times-out.md).
  CLOSED — it was fixed before this phase opened.** `bb0631e5f` (2026-08-28)
  landed `start_server_then_client`; the issue was simply never closed. Cause
  was harness ordering, not the image: `start_pair` launched both NuttX
  instances at once, keyed on the PLATFORM — right for pub/sub, wrong for
  request/response where the client asks ONCE. 3/3 failing at 72-92 s ->
  passing at 16.2 s.
* **W3 — [issue 0870](../issues/archived/0870-nuttx-cpp-action-client-transport-tx-failed.md),
  NuttX C++ `create_action_client` fails. OPEN, and no longer blind.**
  **NOT shared with W2**, structurally: W2's fix covers all three languages
  (`rtos_e2e.rs:922`), so C++ has been starting after the server's banner all
  along and still fails ~2 in 3 — and the failure points differ, W2 at
  `send_goal` after the declarations succeeded, this one INSIDE them.
  Landed here: **NuttX had no `printk` arm in `zpico.c`**, so every shim
  diagnostic compiled away, including the two that name this fault. Fixed.
  Killed by measurement: the queryable-capacity and TX-buffer leads the issue
  was carrying — both leaves' shim constants are byte-identical and
  `ZPICO_MAX_QUERYABLES` is **32, not 8**.
  Why C++ and not C is still unexplained, and the issue now says so rather
  than offering a fourth guess.
  **EXPERIMENT RUN 2026-09-03: could not reproduce — 28 solo runs, retries
  disabled, 28 PASS, idle and under load.** At the reported 2-in-3 rate that is
  (1/3)^28, so the rate does not hold for the current build. NOT a fix: this
  issue records that removing the diagnostics restored FAIL/PASS/FAIL, so the
  fault is timing-sensitive and moves with image CONTENT, and today's image
  carries more code than any previously measured one.
  The instrumentation is now armed — both shim diagnostics are linked into the
  binaries (`strings`, absent from the Aug-21 ones), so the next FAILING run
  answers the 6-of-6 vs 1-of-6 question at zero cost. Verified by LINKAGE, not
  observation: every printk is on a failure path and nothing failed.
  First hard number on the asymmetry: **C 26.4-27.2 s, C++ 44-50 s** for the
  same round trip. A measurement, not a mechanism.
  Also: the Aug-21 binaries in this tree predate the Aug-29 instrumentation that
  produced this issue's quoted output, so "it passes now" is not a delta against
  "it failed then" — different images, not diffable.
  **2026-09-04: the asymmetry is EXPLAINED and withdrawn as a lead — issue
  1034.** It measured the EMULATOR. The harness prefers the `nros setup` store
  QEMU (`11.0.0-nros2`) over `PATH`, and on that build a NuttX arm image whose
  `.bss` shares a `PT_LOAD` with its `.data` burns **~19.6 s of CPU before the
  guest's first console byte**; on the distro QEMU 6.2.0 it starts in 0.05 s.
  Correlation with program-header shape is exact over all 19 NuttX arm images,
  and the cost is linear in the zero-fill size (~31 KB/s). Instrumented: both
  action SERVERS stall, `cpp/action-client` stalls, `c/action-client` does not —
  so the C cell pays one stall (26.1 s) and the C++ cell two (45.7 s), and the
  difference IS the stall. Hand-run on 6.2.0 the C++ round trip is **2.2 s**.
  `c/action-client` and `c/action-server` differ by 16 bytes of `.bss`.
  The `-100` failure is UNTOUCHED by this — no mechanism connecting stall to
  failure is offered. What changes is method: this issue says the fault "moves
  with image CONTENT", and 1034 is a way 16 bytes of content move timing by 20 s,
  so a two-build comparison must check the `PT_LOAD` shape, and a hand-run is not
  comparable to a cell unless it runs the same emulator binary.
* **W4 — [issue 0847](../issues/archived/0847-xrce-entity-drop-after-session-close.md).
  CLOSED.** The fix is a refcount (`live_entities` + `session_closed`), applied
  to all four entity destructors, with each checking `xrce_session_is_closed`
  before touching the session. The two shapes the issue left open were both
  rejected for stated reasons: the binding side protects Rust callers only on a
  C ABI, and the back-pointer sweep needs a fourth static pool because the
  session has slot tables for everything EXCEPT publishers — the entity the
  crash was reported on — on the backend whose current campaign is removing
  unpriced static RAM.
  Cyclone is immune because it stores validatable HANDLES, not pointers; that
  contrast is what settled the shape.
  Gated by `tests/entity_lifetime.c` under `just check rmw-xrce`, asserting the
  EXIT STATUS as the issue required, and mutation-checked.
* **W5 — [issue 0741](../issues/archived/0741-xrce-service-reply-history-payload-too-small.md),
  RESOLVED 2026-09-03 — the 28-byte sample is a FOREIGN peer's, on another
  host.** Everything below is the investigation that got there, kept because
  four of its steps were dead ends worth not repeating.
* **W5 — [issue 0741](../issues/archived/0741-xrce-service-reply-history-payload-too-small.md),
  `test_xrce_service_ros2_client` fails on main — Fast-DDS refuses the
  request.** INTEROP with a real ROS 2 peer.
  **ROUTING DECIDED: it STAYS here. Not encoding, so not phase-303.** The
  defect is wire FRAMING of the request/reply mapping, outside our serializer —
  not XCDR2/extensibility, which is 303's class and is parked besides.
  The issue's own premise is refuted: 15 bytes is the CORRECT reader history
  size for `AddTwoInts_Response` (`align4(4+8)` + 3), confirmed by five
  environments that accept the reply. The sample is oversized by 16, not the
  buffer undersized — the title is inverted.
  `28 = 4 + 24`: the Agent appears to consume only 8 of the 24-byte
  SampleIdentity our client prefixes and leak the other 16 into the DDS
  payload. MEASURED up to the agent boundary; INFERRED across it, since
  `third-party/xrce/agent` is uninitialised here.
  **Landed here: this issue's own mitigation was unreachable.** `ca224e271`
  built the agent against the sourced ROS's Fast-CDR, but `just xrce setup`
  short-circuited on file existence and never called the script that decides —
  so any host that had ever published an agent kept the skewed one. This host
  still carried the pre-mitigation wrapper nine days later. Fixed.
  **RE-MEASURED 2026-09-03 on a genuinely zero-skew agent (Fast-DDS 2.6.12 /
  Fast-CDR 1.0.29, the same library FILES as the ROS peer, verified by `ldd`):
  the failure SURVIVES.** 66 runs, `--retries 0`, 64 pass / 2 fail — batch A
  alone 14 of 15, the same order as the historical rate. So 0741 cannot be
  closed as "the mitigation was never applied"; it was not applied, and
  applying it does not fix this.
  **My own inference above is REFUTED.** In the failing run the Agent never
  received the DDS request and never wrote a DDS reply — `read_fn=0`,
  `write=0`, complete trace, truncation excluded. It did not mis-slice the
  SampleIdentity; it did nothing. The question changes shape: the request never
  reached the Agent AND something else wrote a 28-byte sample on the reply
  topic. That is an endpoint-matching/discovery anomaly, not a serialization
  one.
  **Also fixed here: the only non-root instrument was compiled out.**
  `build.sh` passed the logger profile OFF at both sites, so
  `NROS_XRCE_AGENT_VERBOSE` was silently inert against the agent the mitigation
  publishes. Now selectable, recorded in the stamp so it rebuilds, derived at
  file scope so both build paths see it, and the test side says what an empty
  log means.
  Next is a DDS capture (needs root, unavailable here): the writer GUID of the
  28-byte sample decides Agent-framing versus foreign peer.

## The phase's own answer, at close

Its acceptance asked for two things, and both are met.

**"Each of the five is resolved, or reassigned with the reason recorded."** W2 and
W4 fixed; W1 and W3 closed green with the cause unattributed, each recording the
residual risk and what reopens it; W5 resolved when the 28-byte sample turned out
to belong to a foreign peer on another host.

**"For W2/W3, an explicit statement of whether the cause was shared."** NO, and
the reasoning is structural rather than empirical: W2 was harness ordering and its
fix covers all three languages, so C++ had been starting after the server's banner
all along; and the failure POINTS cannot be one defect, since W2 failed at
`send_goal` after the declarations succeeded while W3 failed inside them.

**What the phase found that was in none of the five issues.** Three of them were
UNREADABLE rather than unfixed — W2 was fixed on 2026-08-28 and nobody closed it;
W1's probe reported FRESH over a museum bake because the constant lived in a
build-script dependency crate; W3 had no `printk` arm on NuttX, so every shim
diagnostic compiled away; W5's own mitigation had been unreachable for nine days
behind a file-existence short-circuit. Two of the five were not defects in the
state their issues described.

That is the phase's real result, and it generalised: it is why phase-423 sequences
its dropped logger first, and why "an issue with no home goes stale" became a
survey of all 85 open issues rather than a remark.

## Acceptance

* Each of the five is resolved, or reassigned to a phase that fits better with
  the reason recorded.
* For W2/W3, an explicit statement of whether the cause was shared — that
  answer is worth more than either fix.

**Progress 2026-09-03.** W2 and W4 closed; W5's routing decided (stays);
W1 and W3 rediagnosed with their standing guesses killed by measurement.

**Progress 2026-09-04. W5 is CLOSED** — issue 0741 resolved: the 28-byte sample
on the reply topic belongs to a FOREIGN peer on another host, which is also what
issue 1009 fixed (the interop bus is now pinned to loopback on both sides). The
inference this phase recorded about the Agent mis-slicing a SampleIdentity stays
refuted; the sample was never ours to explain.

**W3 is the phase's only open item.** Its C-vs-C++ asymmetry is now explained
and withdrawn (issue 1034, above); its `-100` failure still needs a failing run,
and the instrumentation for that is armed and linked.

**Progress 2026-09-04, and W3 is now a DECISION rather than an experiment.**
Issue 1034's `.bss` fix cut this cell from 45.7 s to 4.8 s, so the sweep this
issue had been deferring became affordable on the emulator the harness actually
resolves: **150 / 150 idle and 60 / 60 under load average 33**, `--retries 0`,
which with the earlier batches is 274 consecutive passes. Two bounds are stated
in the issue rather than glossed: the images carry zenoh-pico 1.7.2 because
issue 1035 makes main's 1.8.0 pin fail to compile for NuttX at all, and the
`.bss` fix moved every address, so this is a THIRD build regime and not a longer
run of the second — which is the same objection W3 itself raised against reading
the previous batch as a delta.

Every cheap avenue is spent: instrumentation armed and linkage-verified,
`retries = 0` in place so a red is a red, and a sweep now costing minutes.
Closing it means accepting green with the cause unattributed, exactly as W1 was
closed — an owner call, not a conclusion, and deliberately not taken here.

**Both bounds are now discharged.** Issue 1035's fix landed, so `main` builds
NuttX again and the nightly observer is restored; re-measured on `main` with the
shipped zenoh-pico pin: 6/6 NuttX C/C++ cells, and **100 / 100** on this cell.
That is 374 consecutive passes over three build regimes and two emulators, none
of which produced a failing run to read. Nothing further is measurable; the call
is the owner's.

The shared-cause answer: **NO.** W2 was harness ordering and is already fixed;
W3 fails inside construction, before any interaction with the server exists, so
server ordering cannot reach it. The phase does not shrink.

**What this phase actually found, and it was not in any of the five issues.**
Three of the five were unreadable rather than unfixed:

* W2 was fixed on 2026-08-28 and nobody closed it.
* W1's fixtures were baked 2026-08-20 and the staleness probe reported FRESH
  for all of them, because the constant lives in a build-script DEPENDENCY
  crate that never enters `cargo:rerun-if-changed` (#1005). A rebuild makes the
  cell green — though NOT, as first written here, because of issue 0906: the
  counterfactual passes too, so the fix is real and unattributed. And the cell
  cannot see that constant in either direction, which is the sharper half.
* W3 could not be read at all: NuttX had no `printk` arm, so every shim
  diagnostic compiled away.
* W5's own mitigation had been unreachable for nine days behind a
  file-existence short-circuit.

The phase opened on the premise that an issue with no owner is an issue nobody
is accountable for. The sharper version, measured here: it goes STALE, and a
stale issue costs MORE than an open one, because it is read as current and its
recorded guesses get re-run.

## What this phase deliberately does NOT do

It does not add tests, lanes or gates. Every one of these already has a failing
test; the problem is that nothing was accountable for making them pass.


## Adopted issues (2026-09-04) — three more that reproduce on an RTOS

This phase opened as a home for five runtime failures with no owner. Three more
were filed since and had none either. They meet the same admission test: the
image builds, links and boots, and then does the wrong thing.

* **[#0997](../issues/0997-island-announces-spdp-once-then-lease-expires.md)** —
  the timed-event tree empties itself on FreeRTOS: the SPDP resend is scheduled
  and never lands. Same platform and layer as W1.
* **[#1004](../issues/1004-an536-image-fails-to-boot-transport-error.md)** — the
  an536 boot failures on this host were HOST LOAD, not a code regression. Kept
  rather than closed because the retraction is the finding: it is the third time
  a load artefact was filed as a defect here, and the phase's own W1 and W5 are
  two of the others.
* **[#0968](../issues/0968-tier2-runtime-failures-unreproduced.md)** — tier 2
  has ~12 runtime e2e failures on main, unreproduced. This one is the phase's
  admission test applied at scale: *reproduces solo* is what makes a failure
  worth an owner, and nobody has run the tier to find out which of the twelve do.

**Read #1004 before diagnosing any of them.** Its retraction, and the four
retracted issues CLAUDE.md records from a sweep whose fixtures predated the fix,
are the standing reason to check artifact mtimes against the code being blamed
before writing a cause down.
