---
id: 870
title: "NuttX C++ action client fails `create_action_client` — the session
  reports `Transport(ConnectionFailed)` against a router the server reached"
status: resolved
type: bug
area: rmw, examples
related: [issue-0867, issue-0891, issue-0460, issue-1007]
---

## Symptom

`test_rtos_action_e2e::platform_2_Platform__Nuttx::lang_3_Lang__Cpp`, run alone
on an idle host (load 0.53), fails its first two attempts and passes the third:

    Waiting for action goals (Ctrl+C to exit)...          # server is up
    [nros] examples/qemu-arm-nuttx/cpp/action-client/src/main.cpp:40
           node.create_action_client(client, "/fibonacci") -> -100

`-100` is `NROS_CPP_RET_TRANSPORT_ERROR` / `_Z_ERR_TRANSPORT_TX_FAILED`
(`nros_cpp_ffi.h:585`, `result.hpp:67`): the transport could not TRANSMIT. So
this is not the client failing to find the server — it is the client's own
declaration failing to leave the box.

## Not issue 0867, and not fixed by it

0867 is the C client's `Failed to send goal: -2` (`NROS_RET_TIMEOUT`), caused by
the client being started alongside the server and asking before the server's
queryable existed; the fix orders the request/response start on the server's
readiness banner. That fix is verified and it does help this cell — nuttx C++
action passed at 23.5 s in one 9-cell run — but the `-100` predates it and
survives it, and it is a different failure at a different point: 0867's client
gets as far as `Sending goal`, this one never finishes construction.

Both were present before either fix, which is why they were easy to conflate:
the same cell produced `-2` and `-100` on different runs.

## What is known

* Reproduces solo on an idle host, so it is not the host-load class of 0891.
* Roughly 2 failures in 3 attempts, and nextest's retries mask it — the cell is
  reported FLAKY rather than failing, so it has been passing CI on its third try.
* The server side is healthy and prints its banner every time.
* C on the same board and the same transport does not hit it.

## Where to look

`_Z_ERR_TRANSPORT_TX_FAILED` on a DECLARATION suggests the zenoh-pico session's
TX path is not ready, or is out of a resource, at the moment the C++ binding
declares the action client's entities. An action client is several entities at
once (goal / cancel / result queries plus feedback and status subscriptions),
declared back-to-back — a burst the C client does not produce identically.

Two candidates, neither yet tested:

* TX buffer / batch sizing on the zenoh-pico session during a declaration burst.
* Queryable and subscriber pool capacity — `ZPICO_MAX_QUERYABLES` is 8 embedded
  and `[param_services]` + `[lifecycle]` claim slots before the app declares
  anything (issue 0460). An exhausted pool surfacing as a TX failure rather than
  as `-6` (`NROS_RET_FULL`) would also explain why the error names the transport.

## Measured: the error was hidden behind THREE layers of collapse

The guesses above (TX buffer sizing, queryable pool capacity) were both wrong,
and so was the title. They were reached by reading return codes that lie. Three
separate seams each replaced a typed error with a less specific one:

1. `nros_cpp_action_client_create` — `Err(_) => NROS_CPP_RET_TRANSPORT_ERROR`,
   discarding a typed `NodeError`. Fixed: it now calls `node_error_to_cpp_ret`,
   which already existed and already prints the variant (issue 0557 built it for
   exactly this collapse, one layer in).
2. That revealed `NodeError::ActionCreationFailed` — itself a flattening of 17
   `session.create_*` sites in `executor/action.rs`, every one
   `map_err(|_| NodeError::ActionCreationFailed)`. `NodeError` ALREADY carries
   `Transport(TransportError)`; nothing needed adding, the error was simply not
   passed on. Swept all 17.
3. Which finally names it:

       [ERROR] nros: NodeError::Transport(ConnectionFailed)

So `-100` was accidentally in the right FAMILY and useless about the cause: not
a TX failure, a **connect** failure. The session cannot establish its link when
the client declares its entities — while the action SERVER, on the same port and
the same router, connected fine and printed its banner.

## What that reframes

The server reaching the router proves the router is up and reachable on that
port, so this is not "the router was not started". Two QEMU guests each connect
outward through their own slirp stack to `10.0.2.2:<port>`; the second one
fails. Candidates, none tested:

* the client's connect deadline is too short for a loaded host (two arm-virt
  QEMUs under `-icount`), so the TCP connect times out and surfaces as
  `ConnectionFailed`;
* something about the second guest's slirp path to the same host port.

Note this is NOT the 0867 ordering bug — that fix (start the client only after
the server's banner) is in, and this cell still fails. If anything the ordering
makes the client start LATER, so a connect-deadline theory has to explain why
later is not better.

## Measured: the real error is `ZpicoError::Generic`, and it is DETERMINISTIC

A fourth collapse sat below the three already fixed: `From<ZpicoError> for
TransportError` maps BOTH `Generic` and `Session` to `ConnectionFailed`, so even
the corrected chain could not say which. Issue 0465 records the cost of exactly
this pair — an exhausted session pool "spent two months looking like
`Transport(ConnectionFailed)` — a router/network problem, and chased as one".

With a diagnostic naming the variant (restricted to those two: this conversion is
on `drive_io`'s hot path, where `Timeout` converts on every quiet tick, so
logging unconditionally would flood a WORKING image):

    [ERROR] nros: zpico Generic -> ConnectionFailed
    [ERROR] nros: NodeError::Transport(ConnectionFailed)
    create_action_client(client, "/fibonacci") -> -100

`Generic` is C-shim return code `-1` (`zpico.rs:84`).

**And it reproduces BY HAND**: router up, server given a 32 s head start, idle
host, two QEMU. So this is not load, not the harness, and not the 0867 ordering
race — it is deterministic in the C++ image. The **C** action client, through the
SAME `register_action_client_raw`, succeeds under the same conditions.

## CORRECTION: it is INTERMITTENT, not deterministic

An earlier revision of this issue said the failure "reproduces BY HAND …
deterministic in the C++ image". That was wrong, and the way it was wrong is
worth recording because it nearly produced a false fix.

The hand-run reproduced it once, and a handful of test runs failed, so it was
called deterministic. Later, after a rebuild, the same cell passed 3/3 twice in
a row — which looked like a fix, from instrumentation that only logs on the
FAILURE path and therefore cannot run at all on a passing one. Reverting the
instrumentation and rebuilding: still 3/3.

The discriminating run was a rebuild with EVERY diagnostic removed:

    run 1: FAIL    run 2: PASS    run 3: FAIL

So the cell is flaky at roughly two failures in three — exactly what this issue
originally reported — and three consecutive passes was luck, not evidence. Two
consecutive clean sets of three had a prior of about 0.1 %, which is precisely
why "it passes now" should not have been treated as a result.

**Nothing is fixed.** The build sequence that produced the passes is recorded
here only so the next person does not mistake it for one.

## Not yet known

Which of the four declarations returns `-1`. `register_action_client_raw` makes
three `create_client` calls plus one `create_subscription`, and the C client
makes the same four. Ruled out as the source: `ZenohServiceClient::new` (returns
`TopicNameInvalid` / `ServiceClientCreationFailed`, never `Generic`) and
`declare_entity_liveliness` (swallows errors with `.ok()`). Finding it needs
per-call instrumentation in the shim — the next step, and the last layer.

Worth fixing regardless of cause: `TransportError::ServiceClientCreationFailed`
exists and is precise for a failure inside `create_client`; `ConnectionFailed` is
the wrong name for it and is what sent both earlier diagnoses at the network.

## Acceptance

* The cell passes on its FIRST attempt, repeatably, on an idle host.
* MET ALREADY: the failure names its own cause rather than reporting a generic
  transport error — that half is fixed and is worth keeping independently of the
  connect bug, since it is what made the connect bug findable at all.

## phase-414 W3 (2026-09-03): not shared with 0867, and the blindness that hid it

**The shared-cause question is answered: NO.** 0867's cause was harness ordering
and its fix (`start_server_then_client`) covers all three languages, so C++ has
been starting after the server's banner all along and still fails ~2 in 3. And
the failure POINTS cannot be one defect: 0867 failed at `send_goal`, after this
client's declarations had all succeeded; this fails INSIDE them, before any
interaction with the server exists.

### Why nobody could read the real error: `printk` was a no-op on NuttX

`zpico.c`'s printk chain had arms for Zephyr, FreeRTOS, ThreadX and bare-metal,
then `#else #define printk(...)`. NuttX defines `ZENOH_NUTTX` + `ZENOH_LINUX` and
matched no arm, so **every diagnostic in the shim compiled away** — including the
two that name this fault outright:

    zpico: z_declare_subscriber (ring) failed: %d for '%s'
    zpico: z_liveliness_declare_token failed: %d for '%s'

the second of which has a comment calling a failed token "a SILENT graph outage
… say so on the console". Not a platform limitation: NuttX has full POSIX stdio
and the TU already includes `<unistd.h>`. FIXED — NuttX now routes printk to
`printf`. This is the fifth error-collapse layer in this issue's chain and the
first one below the Rust boundary.

### Which declaration fails, by elimination — INFERRED, not observed

`register_action_client_raw_sized` makes four `session.create_*` calls. The three
`create_client` calls do NO network I/O (`ZenohServiceClient::new` only builds
keyexprs; its only errors are `TopicNameInvalid` / `ServiceClientCreationFailed`),
and the liveliness declares inside them are swallowed by `.ok()`
(`shim/session.rs:564`). The feedback `create_subscription` is the ONLY site in
the whole construction that converts a `ZpicoError`, so it is the only one that
can produce the observed `Generic -> ConnectionFailed`. That narrows to
`z_declare_subscriber() < 0` (`zpico.c:2246`).

Construction performs six network ops: five liveliness declares (all invisible)
plus one `z_declare_subscriber` (the only one that speaks). **Whether the other
five also failed is currently unknowable, and that distinction is the
diagnosis:** 6-of-6 means the session's declare/TX path is dead at that moment
(which is what `ConnectionFailed` accidentally named correctly); 1-of-6 means
something subscriber-specific.

### MEASURED, and it kills this issue's two standing guesses

Both leaves' compiled shim constants are byte-identical (same fingerprint hash):
`ZPICO_MAX_QUERYABLES = 32` — **not 8** — `MAX_SUBSCRIBERS = 8`,
`MAX_LIVELINESS = 16`, `MAX_PENDING_GETS = 4`, `MAX_SESSIONS = 1`. Neither image
registers param services or lifecycle (both opt-in, neither example calls them),
so issue 0460's "6+5 slots claimed before the app" does not apply here. **The
queryable-capacity lead is dead**, and so is the TX-buffer one for the same
reason: the two languages share the shim config exactly.

C and C++ also declare the same four entities, with the same names, resolving to
the same session slot 0. Fixture rows are symmetric but for the port.

### Still open, and it is the whole remaining question

**Why C++ and not C is unexplained.** No structural asymmetry was found by
reading: same shim, same pools, same entities, same names, same slot, same
ordering relative to session open. The remaining differences are timing-shaped
(C interposes `nros_executor_init` between session open and the declarations;
C++ goes through `Executor::open_in`). None is a mechanism defensible from
reading alone. **Treat any C-vs-C++ story that has not been measured as a
guess — this issue has already burned three of them.**

### Next step, now cheap

The four per-declaration diagnostics from `f5674ed52` are still in the tree
(`action.rs:1236/1246/1256/1266`) and `nros_log` demonstrably reaches the NuttX
console. With printk unmuted, the next FAILING run names which declaration failed
AND prints zenoh-pico's raw return code — at zero extra cost. Worth pairing with
a log at the swallowed liveliness site (`shim/session.rs:564`), which is the
canary the design deliberately muted.

## 2026-09-03 experiment: COULD NOT REPRODUCE. The issue stays OPEN.

The decisive question — do all SIX network operations fail, or only the
subscriber — is **still unanswered**, because nothing failed.

**MEASURED: 28 solo C++ runs, `--retries 0`, 28 PASS, 0 FAIL.**

| batch | condition | result |
| --- | --- | --- |
| 22 runs | idle host (~5 of 48 cores) | 22/22 PASS, 44-50 s |
| 3 runs | C++ co-selected with the C cell | 3/3 PASS |
| 3 runs | 32 busy loops, load avg 35 | 3/3 PASS, 61-67 s |

Every pass is a real round trip, not an early exit — `Result received:
[0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]`. At the reported 2-in-3 failure rate,
28 consecutive passes has probability (1/3)^28. **The reported rate does not
hold for the current build on this host.**

**This is NOT a fix and must not be read as one.** This issue's own record says
removing the diagnostics restored FAIL/PASS/FAIL, which points at a
timing-sensitive fault whose probability moves with image CONTENT — and today's
image carries more code than any previously measured one. Which change moved it
is unknown, and per this issue's own history (three guesses already burned) no
fourth is offered.

### The instrumentation is armed now, verified by linkage

`strings` on the fixtures, Aug-21 binary vs today:

| string | Aug-21 | today |
| --- | ---: | ---: |
| `zpico: z_declare_subscriber (ring) failed: %d for '%s'` | 0 | 1 |
| `zpico: z_liveliness_declare_token failed: %d for '%s'` | 0 | 1 |
| `action client: feedback subscription failed` | — | 1 |
| `action client: send_goal client failed` | — | 1 |

Both C and C++; sizes grew 806,892 -> 852,964 (C++) and 752,604 -> 793,336 (C).
Every `printk` in `zpico.c` is on a failure path, so **the NuttX printk arm is
verified by LINKAGE, not by observation** — nothing emitted because nothing
failed. The next failing run will speak; this one had nothing to say.

### The history this reframes

The binaries in this checkout when the experiment started were dated **Aug 21
09:01**. This issue's quoted output (`zpico Generic -> ConnectionFailed`) comes
from instrumentation landed **Aug 29** (`f5674ed52`). So the binaries in this
tree were never the ones that produced those measurements — that work happened
against a build that no longer exists here, and "it passes now" is not a delta
against "it failed then". The two are not the same image and cannot be diffed.

### First hard number on the C-vs-C++ asymmetry

**C: 3/3 PASS at 26.4-27.2 s. C++: 44-50 s.** The C++ image takes ~20 s longer
for the same action round trip on the same board and transport, even when it
succeeds. That is a measurement, not a mechanism — but it is the first evidence
of any kind about the asymmetry, which until now was only guesses.

### Also found, filed separately

* **Issue 1007** — a clean `just nuttx build-fixtures-arm` can leave every arm
  cell unrunnable, and the remedy it prints is the command that just
  short-circuited. Cost a forced kernel build plus a second full fixture rebuild
  before any measurement could be taken.
* Every NuttX C/C++ configure logs `no Corrosion at the pinned prefix … falling
  through to FetchContent`, i.e. it clones Corrosion from git. It succeeded here
  (network available); this is the offline-failure shape of issues 0500/0726.

### What would actually settle this

A failing run. Since it will not fail on demand here, the cheapest honest option
is to leave the instrumentation in place and catch it in a sweep — the cell is
reported FLAKY by nextest retries, so the CI history already knows when it fails
even though no one has read a failing run since the diagnostics landed.

## 2026-09-04 — policy: NO RETRIES on this cell

Owner decision, and it reframes what the cell is for. These e2e cells exist to
show an RTOS meeting its obligations **including under load** — that is the
property under test. A cell that fails once and passes on the third attempt has
not demonstrated it; it has demonstrated that the guarantee is PROBABILISTIC,
which is the failure. Retrying converts that finding into a green.

`retries = 2` -> `retries = 0` for `binary(rtos_e2e) and test(Platform__Nuttx)`.

That retry is what kept this issue unreadable for weeks: the cell failed roughly
two runs in three, was reported FLAKY, passed CI on its third try, and nobody
looked. Both halves of the blindness are now gone:

* the shim diagnostics are linked in (verified by `strings`, absent from the
  Aug-21 binaries) — the NuttX `printk` arm exists now;
* `rtos_e2e.rs` already prints server boot, server post-boot and client output
  on EVERY run, so a failure carries them;
* the action assertion now names the question its own output answers.

**A red here is information, not noise.** And if concurrency is what makes a
cell fail, the fix is the `test-group` and port routing — configuring the
concurrency correctly — not re-rolling the dice.

### What a failing run will now settle

The one remaining unknown: **six of six, or one of six?** Construction performs
five liveliness declares plus the feedback subscriber. If all six failed, the
session's declare/TX path is dead at that moment and `ConnectionFailed` was
accidentally the right family. If only the subscriber failed, it is
subscriber-specific and zenoh-pico's raw code names it. Nobody has ever seen
that, because nobody has read a failing run since the diagnostics landed.

### Deliberately NOT done

The other three RTOS e2e overrides still carry `retries = 2`:
`Platform__Freertos`, `Platform__Threadx*`, `ThreadxLinux`. They mask the same
class and the same argument applies to them, but issue 0968 records ~12
unreproduced tier-2 e2e failures — flipping all four at once produces a wall of
red that obscures rather than reveals. Extending this is a one-line change per
override and a separate decision, not a rediscovery.

---

# CORRECTION 2026-09-04 — the capacity lead was never possible, and the instrumentation is not in this tree

## My "the instrumentation is absent" claim was WRONG — the fixture was stale

An earlier draft of this section asserted, from `strings` on
`examples/qemu-arm-nuttx/cpp/action-client/build-zenoh/cpp_action_client`, that
the four diagnostics were not linked and that the standing plan was therefore
false. **Retracted.** The binary is dated 2026-08-27 03:55; the diagnostics
landed on 2026-09-03 (`action.rs:1266` has
`"action client: feedback subscription failed: {:?}"` in the source right now).
`strings` was reading a week-old artifact.

That is precisely the trap CLAUDE.md records from issues 0859-0862 — four ghost
issues filed from a sweep whose fixtures predated the fix — and the remedy it
prescribes is the one I skipped: `stat -c '%y' <artifact>` against
`git log -1 --format=%ci` of the code being blamed, BEFORE drawing a conclusion.

What survives, and is worth keeping as a standing check rather than a finding:
**`strings` the artifact you are about to run, not the tree you are reading.**
A stale fixture makes an armed diagnostic invisible, which is the same
observation as an unarmed one.

## Pool exhaustion cannot produce this error code — the lead was structurally impossible

The issue retired the capacity hypothesis by measuring that the pools are big
enough. The stronger reason is that **no exhaustion anywhere in the path can
arrive as `Generic`**, so the lead was dead regardless of what the constants
say — and stays dead for any future NuttX build:

| exhausted pool | code it returns | file:line |
| --- | --- | --- |
| zpico subscriber slots | `ZPICO_ERR_FULL` (-6) | `zpico.c:2236` |
| liveliness slots | `ZPICO_ERR_FULL` | `zpico.c:2681` |
| Rust subscriber index | `TransportError::SubscriberCreationFailed` | `subscriber.rs:674` |
| payload block | `TransportError::SubscriberCreationFailed` | `subscriber.rs:682` |

The observed code is `ZpicoError::Generic` (-1), and at the only site that can
produce it during construction (`zpico.c:2267`) it means exactly one thing:
**`z_declare_subscriber()` itself returned < 0.**

Inside zenoh-pico, `_z_register_subscriber` (`net/primitives.c:210-251`) has
four failure exits: keyexpr declaration, sync-group notifier, registration OOM
(`-78`), and `_z_send_declare` -> `_Z_ERR_TRANSPORT_TX_FAILED` (`-100`). OOM is
implausible on 126 MB, and the C image carries MORE `.bss` (549,456 vs 540,976)
without failing. That leaves the TX exit.

## Entity accounting, corrected

An action **client** opens **four** entities, not "three services plus feedback
and status": there is no client-side status subscription
(`action.rs:1128-1270`). The two publishers belong to the **server**
(`action.rs:679-685`). That is a real divergence from `rclcpp_action` and worth
knowing, but it is not this bug.

Construction performs **seven** network ops in the C++ shape, not six: two node
liveliness tokens (`session.rs:445` at open, plus `session.rs:862` from
`ensure_node_liveliness`, because the entity's node name differs from
`primary_node_name`) + four entity tokens + one subscriber declare. The
"6-of-6 versus 1-of-6" framing is really 7-of-7 versus 1-of-7 — small, but it
changes what a diagnostic must print to answer the question.

## Split out, not buried: the queryable budget is an accident

`ZPICO_MAX_QUERYABLES = 32` on NuttX is reported here as a fact. It is correct,
and it is not a choice: `runner.rs:246` tests `target_os != "none"`, and NuttX
reports `target_os = "nuttx"`, so an RTOS takes the budget written for Linux.
Measured cost in this very image: **142,336 B of `.bss`** with zero queryables
declared. Filed as [#1028](../1028-nuttx-classified-hosted-takes-linux-queryable-budget.md).

## The client-side analogue of 0460 exists, but not at construction

`pending_gets[ZPICO_MAX_PENDING_GETS]` is 4 slots (`zpico.c:460`) **shared**
between service gets and liveliness gets (`zpico.c:3270` says so). An action
client holds three service clients that can each park a standing discovery get,
plus `wait_for_action_server`, plus `send_goal`'s 500 ms resend loop which
allocates a new slot per generation (`shim/service.rs:759-766` documents
overflowing 4). Genuine latent risk on the RUNTIME path, plausibly relevant to
#0867 — **not reachable at construction**, so it does not explain this issue.

## Still unexplained: why C++ and not C

Checked and found nothing that survives. Both images link the **same**
`zpico-sys` artifact (`zpico-sys-ec52ae90a97507ef`), so every shim constant and
every zenoh-pico `#define` is provably identical. Both pool symbols are
byte-identical. Neither carries param or lifecycle services. Both open in
`SessionMode::Client`. The one structural difference — C opens through
`open_session` with an empty namespace and the session name, C++ through
`Executor::open_in` with the resolved node name and namespace — only changes
which node token is declared, and every liveliness declare is `.ok()`-swallowed.

**No fourth story is offered.** Three have been burned; the next step is
evidence, not another hypothesis.

## Fix direction: stop collapsing the error, do not raise a limit

Nothing is being exhausted, so nothing should be raised. The remaining defect
in the chain is that `zpico.c` discards zenoh-pico's return code at six declare
sites (`:2106`, `:2154`, `:2210`, `:2267`, `:2320`, `:2695`), and
`subscriber.rs:725` then relabels the survivor as `ConnectionFailed` — which is
what pointed the first two diagnoses at the router. Nothing about that call is a
connection: the session is open and a DECLARE failed.

Add `ZPICO_ERR_TX` / `ZPICO_ERR_NOMEM` (the bands do not overlap, so one shared
`_zpico_map_zerr` helper suffices — a second spelling is how six sites come to
disagree), mirror them in `ZpicoError`, and return
`SubscriberCreationFailed` at `subscriber.rs:725` like the two exhaustion arms
above it already do. Then `-11` vs `-12` vs `-1` splits the remaining
hypothesis space in one run, which is the question this issue has been unable
to ask for four rounds. RAM cost: none.

## Confirmed sound

`nros_log` **does** reach the NuttX console in a C/C++ image —
`nros-board-nuttx-qemu/src/entry.rs:50` calls `init_default()` inside
`nsh_main` before `main`, and `nsh_main` is in the built ELF. Without that the
four `nros_error!` calls would sit in the pre-init ring and never print. The
plan is sound; only the binaries are wrong.

**Corroborated 2026-09-04 from a second session, on freshly rebuilt fixtures:**
`strings` on `cpp_action_client` and `c_action_client` (both rebuilt 10:38 that
day) finds all four diagnostics, 1 occurrence each. So the retraction above is
right and the standing plan holds — the diagnostics ARE armed in a current
build. The section heading still says otherwise; the body is what to believe.

## 2026-09-04 — the C-vs-C++ asymmetry is EXPLAINED, and it is not about C++

This issue's most recent measurement — "**C: 3/3 PASS at 26.4-27.2 s. C++:
44-50 s.** The C++ image takes ~20 s longer for the same action round trip …
the first evidence of any kind about the asymmetry" — has a cause, and it is
the EMULATOR, not the binding. Filed as **issue 1034**; the part that matters
here is that this lead is dead.

The harness resolves `qemu_system_arm_path()` to the `nros setup` store build
(`~/.nros/sdk/qemu/11.0.0-nros2`) in preference to `PATH`. On that build, a
NuttX arm image whose `.bss` is folded into the same `PT_LOAD` as its `.data`
takes **~19.6 s of CPU-bound emulator work before the guest's first console
byte**; on the distro `qemu-system-arm` 6.2.0 the same image starts in 0.05 s.
The correlation with program-header shape is exact across all 19 NuttX arm
images in the tree, and the stall is linear in the zero-fill size.

Instrumented cell timings on this tree:

    nuttx C   action: server banner 20.14 s | client collect  3.51 s | cell 26.1 s
    nuttx C++ action: server banner 19.94 s | client collect 23.10 s | cell 45.7 s

Both servers pay the stall. `c/action-client` does NOT (it is one of two C
images linked with `.bss` in its own segment); `cpp/action-client` does. So the
C cell pays it once and the C++ cell twice, and **the entire ~20 s "asymmetry"
is one emulator stall**. With the stall gone — hand-run on QEMU 6.2.0, router
up, server first — the C++ client completes goal → accept → feedback → result in
**2.2 s**, and there is no asymmetry left to explain.

`c/action-client` and `c/action-server` differ by 16 bytes of `.bss`, and one
stalls while the other does not, so nothing about the C++ toolchain, the C++
binding or the declaration burst is implicated by the timing.

### What this does and does not settle

* **Settled, and removed as a lead:** the "C++ is slower for the same round
  trip" measurement. It measured the emulator. Any future C-vs-C++ story must
  not cite it.
* **NOT settled:** the `-100` / `Generic → ConnectionFailed` failure itself.
  Nothing here reproduces it, and no mechanism connecting the stall to the
  failure is offered — that would be a fifth guess, and this issue has already
  burned four.
* **Worth knowing for the next experiment:** this issue records that the fault
  "is timing-sensitive and moves with image CONTENT". Issue 1034 is a mechanism
  by which 16 bytes of image content move timing by 20 seconds. So an experiment
  that compares two builds is comparing two emulation regimes unless the
  `PT_LOAD` shape is checked first (`arm-none-eabi-readelf -l <image> |
  grep '^  LOAD'`), and a hand-run reproduction is not comparable to a cell
  unless it uses the same emulator binary — the two differ by 400x here.

### The reproduction attempt is now ~10x cheaper, and it was spent

Issue 1034's finding is directly useful here: with `QEMU_SYSTEM_ARM=/usr/bin/
qemu-system-arm` the cell runs in **4.7 s instead of 46 s**, because it skips two
~19.6 s emulator stalls. So the "run it many times and wait for a failure"
strategy — which this issue chose in September as "the cheapest honest option" —
costs a tenth of what it did.

Spent immediately, `--retries 0`:

| batch | condition | result |
| --- | --- | --- |
| 24 runs | idle host, QEMU 6.2.0 | 24/24 PASS, 4.6-4.8 s each |
| 12 runs | 32 busy loops, load avg 22 | 12/12 PASS |

With the 28 runs already recorded, that is **64 consecutive passes**. The
reported 2-in-3 rate does not hold for any build or emulator measured here.

**Stated bound:** these 36 runs are on the DISTRO emulator, which is not the one
CI or a default `just` invocation uses, so they do not extend the store-QEMU
evidence — they extend the evidence that the C++ image itself completes the
round trip. That is deliberate: a cheap batch on a different emulator is worth
more than no batch, provided nobody later reads it as the same experiment.

## 2026-09-04 — 210 more runs on the CI emulator, all green. Disposition is now an owner call.

Issue 1034's `.bss` fix made this cell cost **4.8 s** instead of 45.7 s, so the
"leave the instrumentation in and catch it in a sweep" plan became affordable in
the configuration that matters — the `nros setup` store QEMU the harness
actually resolves, not the distro one the earlier cheap batch used.

`--retries 0`, `test_rtos_action_e2e` NuttX C++ only, one cell per process:

| batch | condition | result |
| --- | --- | --- |
| 150 runs | idle host, store QEMU `11.0.0-nros2` | **150 / 150 PASS** |
| 60 runs | 32 busy loops, load average ~33 | **60 / 60 PASS** |

With the batches already recorded here, that is **274 consecutive passes** across
two emulators, idle and loaded. At the originally reported ~2-in-3 failure rate
the last 210 alone have probability (1/3)^210.

### Two bounds, and the second one matters

* The images were built with zenoh-pico pinned at **1.7.2** (`fa7ad0f5`), not
  main's 1.8.0, because **issue 1035** makes every NuttX zenoh fixture fail to
  compile on main — `#if X == true` against NuttX's cast-valued `stdbool.h`. That
  is the same pin every earlier measurement in this issue used, so these runs are
  comparable with the issue's history rather than with today's `main`.
* **The `.bss` fix changed the image layout**, and this issue's own record says
  the fault "is timing-sensitive and moves with image CONTENT". Splitting `.bss`
  into its own `PT_LOAD` moves every subsequent address. So this is a *third*
  distinct build regime, not a longer run of the second — the same objection this
  issue raised against reading the September 3 batch as a delta. It is stated
  rather than argued around: 210 passes say the current build does not fail, and
  say nothing about why an earlier one did.

### What is left, and it is a decision rather than an experiment

Every cheap avenue is spent. The instrumentation is armed and verified by
linkage, `retries = 0` is in place so a red is a red, and the cell is now fast
enough that a sweep costs minutes. Nothing further can be learned without a
failing run, and no build measured in the last two days produces one.

The parallel is W1, which the owner closed as "accepted green with the cause
UNATTRIBUTED" and a stated residual risk. The same disposition is available here
and is not taken unilaterally: **closing this needs an owner's call**, and the
residual risk to accept is that if the real cause is later reverted, the symptom
returns and the only thing that will catch it is this cell going red — which it
now can, where retries used to hide it.

### The one observer that could settle this is itself down

Checked before proposing any disposition: the nightly `nuttx` platform cell is
**red across all three runs in the window, for issue 1035** — the zenoh-pico
1.8.0 / NuttX `stdbool.h` preprocessor break, identical error text in CI run
`33847619657`. It fails at BUILD, so it never reaches a cell.

That is decisive for what to do next. The 210 runs above are local, on a 1.7.2
pin; CI has not successfully run this cell since the zenoh-pico pin moved. So
"leave the instrumentation armed and catch it in a sweep" currently has no sweep
to be caught by, and closing this issue on a green tripwire would be closing it
on a tripwire that cannot fire.

The fix already exists and is queued: `a1c741db` on the zenoh-pico fork's
`nano-ros` branch compares against `1` instead of `true`, with `c5853157` behind
it for a second NuttX guard, and PR #345 carries the pin bump. So the observer
comes back on its own; what this issue must not do is close before it does.

**Order matters: land issue 1035's fix first** (PR #345, queued). It restores
the only observer, and then this
issue resolves itself either way — the nightly stays green and this closes with
a tripwire that actually works, or it goes red and hands over the failing run
that four rounds of investigation have been unable to produce on demand.

### 2026-09-04, later — the observer is back, and 100 more runs on the REAL pin

Issue 1035's fix landed (PR #345: `a1c741db` compares against `1` instead of
`true`, plus `c5853157`), so `main` builds NuttX zenoh images again. Verified on
`main` at `fa7d09073` with the pinned zenoh-pico `c5853157`, rather than assumed:

* `just nuttx build-fixtures-arm` — clean.
* All six NuttX C/C++ `rtos_e2e` cells — **6 / 6 PASS**, 134.6 s wall.
* This cell, `--retries 0`, store QEMU `11.0.0-nros2`, idle host —
  **100 / 100 PASS** at ~4.8 s each.

That closes the bound the previous batch had to state. The 210 runs above were
on the 1.7.2 pin because main could not compile; these 100 are on the pin main
actually ships, with the `.bss` fix and the shim diagnostics both in the image.

**So the disposition question is now clean.** The observer is restored, the
tripwire can fire, and 374 consecutive passes across three build regimes and two
emulators have produced nothing to read. What is left is the owner call this
issue has been waiting for: close it as green-with-cause-unattributed, the way
W1 was closed, and accept that if the real fix is later reverted this cell going
red is what says so.

---

# 2026-09-04 — DOES NOT REPRODUCE, 9/9, in the ROS distrobox

First run of this cell against a real router since the diagnostics landed. It
passes, and it keeps passing.

```
PASS  test_rtos_action_e2e::platform_2_Platform__Nuttx::lang_3_Lang__Cpp  (5.873s)
  [PASS] nuttx cpp action E2E: accepted=true, completed=true
```

Then eight consecutive repeats of the same cell, alone:

```
run 1..8: pass=1   (8/8)
```

**9/9 including the first.** The symptom above is "fails its first two attempts
and passes the third"; that shape did not occur once.

The sibling languages pass too, on the same tree and router — Rust 49.1 s,
C 11.4 s, C++ **5.9 s**. The C++ cell was the FASTEST of the three, which is not
what a construction failure that retries into success looks like.

## What was actually under test

Everything built in the ROS distrobox on its own tree
(CLAUDE.md: box in play ⇒ every job in the box), so no host artifact is
involved:

* `cpp_action_client`, 864,736 B, built 21:45 in the box — it did not exist
  twenty minutes before the run;
* the four `nros_error!` diagnostics VERIFIED PRESENT by `strings` before
  running, not assumed;
* `nros`, `nros-launch-resolve` and corrosion all rebuilt in the box, into the
  box's own SDK store (`~/.nros-box`);
* the router is ROS Humble's own `rmw_zenohd`, resolved through
  `AMENT_PREFIX_PATH` per RFC-0075 rather than a hand-set path.

## What this does and does not establish

**Does:** the cell is green today, repeatably, on a freshly built tree with a
real router. Since main removed `retries = 2` from this cell, a red here is now
information — and there is no red.

**Does not:** prove the defect is gone. Nine runs on one host cannot refute an
intermittent failure whose original report is two-in-three. The honest reading
is that the environment differs from the one that produced the symptom, and
several things landed today that plausibly bear on it:

* **#1035** — zenoh-pico 1.8.0 did not compile on NuttX at all (two upstream
  breaks). Whatever this cell was doing before, it was doing it on 1.7.2.
* **#0902** — a declined query kept its reply slot for ever, and a failed reply
  stranded one too. Four exhausted slots make a queryable answer nothing.
* **#0852** — every Zephyr transport task ran at the least-urgent priority.
  NuttX is not Zephyr, but the same `zpico.c` arm was audited.

## What would settle it

Not more repeats here. Either:

1. **Run it under the load the original report had.** The symptom was recorded
   "alone on an idle host (load 0.53)" — but the retry that masked it lived in
   the full-sweep lane, where 32-way parallelism is the norm. Run the cell
   inside a full `just ci matrix` rather than solo.
2. **Bisect against the pre-#1035 tree.** If the cell is green on today's main
   and red on last week's, the difference names the fix. That is cheap now that
   the box can build NuttX at all — which it could not this morning.

Until one of those runs, this issue should stay OPEN with this evidence
attached, not closed on nine green runs. A "does not reproduce" is a
measurement, not a diagnosis.

## CLOSED 2026-09-04 — green, with the cause UNATTRIBUTED. An owner decision.

The owner has closed this as green-with-cause-unattributed, the same disposition
W1 was closed under. This section records what that does and does not mean, and
the objection it overrides, so nobody has to reconstruct either.

### The evidence at close

| batch | condition | result |
| --- | --- | --- |
| 28 runs | 2026-09-03, idle + under load | 28 / 28 |
| 24 + 12 | distro emulator, idle then load avg 22 | 36 / 36 |
| 150 + 60 | store QEMU `11.0.0-nros2`, idle then load avg 33 | 210 / 210 |
| 100 | main's shipped zenoh-pico pin, after #1035 | 100 / 100 |
| 9 | ROS distrobox, freshly built in-box, real router | 9 / 9 |

**383 consecutive passes**, three build regimes, two emulators, two trees. At the
reported ~2-in-3 rate the last 210 alone are (1/3)^210.

### The objection, which is recorded rather than answered

The section above this one argues the issue should stay OPEN until one of two
things runs: the cell inside a full `just ci matrix` (32-way parallelism, which
is where the masking retry actually lived), or a bisect against the pre-#1035
tree. That argument is sound — *"does not reproduce" is a measurement, not a
diagnosis* — and closing does not refute it.

What the decision weighs against it is the cost of the alternative. This phase's
own premise, measured on three of its five items, is that an open issue nobody
is accountable for does not merely sit: it goes STALE, is read as current, and
its recorded guesses get re-run. Four have already been burned here. An issue
kept open on the possibility that a 383-pass symptom returns is exactly that
shape.

### The residual risk, stated

If the real fix is later reverted, this symptom returns, and **the only thing
that will catch it is this cell going red**. That is now possible where it was
not: `retries = 2` is gone from `binary(rtos_e2e) and test(Platform__Nuttx)`, so
a red is information rather than a FLAKY badge nobody reads. The instrumentation
is linked in and verified by `strings`, so the first failing run answers the
question four rounds of investigation could not ask: **six of six, or one of
six** — do all five liveliness declares plus the feedback subscriber fail (the
session's declare/TX path is dead), or only the subscriber?

### Reopen on either of these, without argument

1. A red from this cell in any lane.
2. Either experiment above producing a failure — the `ci matrix` run under real
   parallelism, or the pre-#1035 bisect.

Both are cheap now in a way they were not for most of this issue's life: the cell
costs ~4.8 s since #1034's `.bss` fix, against 45.7 s before, and NuttX builds at
all again since #1035.

### What this issue leaves behind, and it is worth more than the fix

Four wrong diagnoses were killed by measurement here, and each is recorded where
the next person will meet it: the TX-buffer and queryable-capacity leads (both
leaves' shim constants are byte-identical, `ZPICO_MAX_QUERYABLES` is 32 not 8);
the C-vs-C++ timing asymmetry (issue 1034 — it measured the emulator, not the
binding); and the error-collapse chain, five layers deep, that made any of it
readable at all. The last of those is a permanent improvement to every NuttX
investigation that follows.
