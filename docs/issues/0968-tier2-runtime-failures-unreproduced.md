---
id: 968
title: "Tier 2 has ~12 runtime e2e failures on main, unreproduced — nobody has run the tier in a long time"
status: open
area: testing
severity: medium
found: 2026-08-31
related: [0967, phase-410, RFC-0061]
---

# What was measured, and what was not

A full tier-2 run on a frozen tree with freshly rebuilt fixtures (8/8 modules
OK):

```
1795 tests run · 1595 passed (2 flaky) · 6 skipped
Real failures: 17
```

`Real failures` is the junit-rewritten count. Nextest's raw line said **200
failed** — the difference is `nros_tests::skip!` panics, which a bare read
counts as failures. Any triage starting from 200 is starting from the wrong
number.

Of the 17, **five were lane/justfile meta-tests** and are fixed (they failed
identically on a fresh `origin/main` worktree; the tier ladder had drifted from
the justfile and its own gate could not see module recipes).

The remaining **12 are runtime e2e**, and this issue exists so they are tracked
rather than remembered:

```
emulator        test_qemu_rtic_service_e2e
esp32_emulator  test_esp32_talker_listener_e2e, test_esp32_workspace_entry_e2e,
                test_esp32_to_native, test_native_to_esp32
native_api      test_threadx_linux_cyclonedds_{action,service,talker_to_native_listener}
zephyr          example_e2e::case_{21,24,27}_xrce_cpp_{pubsub,service,action}_e2e
logging_smoke   logging_smoke_esp32_qemu_emits_every_severity
```

They fail SOLO as well as in the sweep, so they are not the QEMU load-flakiness
CLAUDE.md warns about.

## They are not from the phase-405/407/410 work — proved by diff, not argued

Across every tree that feeds a built image — `packages/`, `examples/`,
`zephyr/`, `integrations/`, `cmake/` — that branch changes exactly two files,
both in `packages/testing/nros-tests/src/` (the test harness; compiled into no
target image). The esp32, zephyr, threadx and QEMU images are therefore
byte-identical to main's, so a runtime difference cannot originate there.

## Why nobody noticed

`post-submit`'s tier-2 job has **never run** — it is interlocked on
`vars.NROS_SELF_HOSTED_READY`, which is not set, and a skipped job does not
colour its run. `host-tests`, the only other lane that executes E2E, has been
red for 20 consecutive runs on issue 0967, dying before its tests start.

So between them, no fixture-backed test has run in CI for a long time. A
backlog of undetected runtime failures is exactly what that predicts.

## NOT DIAGNOSED — read this before acting

**No root cause is claimed for any of the 12.** One sample was examined
(`test_threadx_linux_cyclonedds_service`): the client printed `Locator:` empty
and the server produced no output at all, i.e. it did not start. Whether that
is one cause or several is unknown.

This issue deliberately stops at the list. Four issues (0859–0862) were filed
from a sweep in this repo and all four retracted, two of them carrying confident
wrong root causes — which is worse than the bogus filing, because it aims the
next person at a dead end.

## Work

1. Rebuild tier-2 fixtures and re-run the 12 — the artifacts from the measuring
   run were cleared by `runner-sweep.sh` afterwards, so nothing here is
   currently reproducible.
2. Triage per suite, not per test: esp32 (5 of 12) and threadx-cyclone (3) are
   the two clusters and likely share a cause each.
3. Check each against main at the commit the fixtures were built from —
   `stat -c '%y'` on the artifact against `git log -1 --format=%ci`, per the
   0859-0862 rule.
4. File per cause, once reproduced. Not before.

## Step 1 attempted 2026-09-04 — and it found TWO build failures before any test ran

`just build-test-fixtures lane=tier2` on `a1c6d0d22`. Seven of eight modules
built (zephyr, native, qemu, freertos, nuttx, threadx_linux). The other two are
new findings, both COMPILE/BUILD failures rather than runtime ones, and both are
this issue's own thesis arriving one stage earlier than it predicted:

* **[#1023](archived/1023-sertype-hosted-includes-break-freestanding.md)** —
  `threadx_riscv64` cannot compile `nros_sertype.cpp`: it includes `<memory>`
  and `<string>` and the target is freestanding. The file is new in issue 0970's
  commit, and `examples/fixtures.toml:3146` declares the coordinate, so this is
  a supported cell that has been unbuildable since it landed.
* **[#1025](archived/1025-esp32-flash-image-consumer-drops-the-row-variant.md)** — ESP32
  QEMU flash images cannot be packed. The ELF builds fine; the packer looks in
  `build/cargo-fixtures/qemu-esp32-baremetal/` while the build writes to
  `qemu-esp32-baremetal-4118800323`, because the packer asks
  `nros_fixture_row_artifact_dir` for the group dir with the row's env stripped
  (`"" ""`). Live since `41a7d8de7` on 2026-08-31 — hours before this issue was
  filed.

**1025 bears directly on this issue's list and does not close any of it.** Five
of the twelve are esp32 and all five need a flash image that cannot be produced.
That is a plausible single cause for the whole esp32 cluster; it is NOT
established as their cause, because what was reproduced is the BUILD failing,
not those tests failing for that reason. Establishing it means fixing 1025,
rebuilding, and re-running the five. Written this way on purpose — 0859-0862
were four issues filed from a sweep in this repo and all four retracted, two
carrying confident wrong root causes.

**Status of the twelve: still unreproduced.** The seven non-esp32 ones
(qemu-rtic 1, threadx_linux 3, zephyr xrce-cpp 3) have their modules built and
are ready to run; the five esp32 ones are blocked on 1025.

**Method note for whoever continues.** Run each candidate SOLO, not in a sweep —
CLAUDE.md's rule for QEMU reds, and this issue's list came out of a sweep. And
`cargo nextest` reports a `nros_tests::skip!` panic as a FAILURE while a filter
matching nothing exits 0, so a re-run needs four verdicts (pass / fail /
skipped-precondition / not-found), not two. A two-verdict harness would report
an unmet precondition as a regression and a renamed test as a pass.

## The threadx_linux cyclonedds cluster, DIAGNOSED 2026-09-04 (3 of the 12)

`test_threadx_linux_cyclonedds_{service,action,talker_to_native_listener}`.
Reproduced solo, on fixtures built at `a1c6d0d22`, and narrowed by elimination.
Every line below is a measurement; the two that turned out to be measurement
ARTIFACTS are kept, because both looked like findings.

### What this issue said, and why it was wrong

> the client printed `Locator:` empty and the server produced no output at all,
> i.e. it did not start

**The server starts.** Run directly it prints its whole banner through to
`Waiting for service requests (Ctrl+C to exit)...`. The empty `server` section
in the failure message is a HARNESS artifact:

```rust
server.wait_for_output_pattern(SERVICE_SERVER_READY_MARKER, 30s);  // consumes the banner
let server_out = server.collect_until("Incoming request", 2s);      // only what came after
```

`wait_for_output_pattern` has already consumed the startup output, so
`server_out` holds only what arrived afterwards — nothing. Anyone reading that
panic message concludes the server never started, which is what this issue did.

### Eliminated, each by measurement

| hypothesis | result |
| --- | --- |
| server does not start | starts; full banner to a file AND through a pipe |
| stdio block-buffering loses the output | 496 bytes reach a pipe; the harness also uses `stdbuf` |
| domain mismatch | server honours `ROS_DOMAIN_ID`; both bind 34150/34151 on domain 107 |
| isolated network stack (NetX/NSOS) | 4 host UDP sockets, one on the real LAN address |
| multicast group not joined | both threadx and native join the DDS group |
| stale binaries straddling 0970's sertype change | all three built the same night, well after it |
| discovery is merely slow | client TIMES OUT after a 75 s budget |
| the client or the Cyclone backend is broken | **CONTROL PASSES**: native client + native Cyclone server returns `Result of add_two_ints: 5` |

### What IS established

The native client never matches any endpoint of the threadx-linux server.
Comparing `Tracing/finest` from the working pair against the failing one, the
working trace carries `ACKNACK` traffic and
`ddsi_delete_proxy_{participant,reader,writer}`; the failing trace carries none
of them. ACKNACK is reliable-protocol traffic that only occurs with a matched
remote endpoint, so its absence is not a shutdown-path artifact the way the
`delete_*` lines partly are.

SPDP frames ARE seen on both sides (5-6 each), so packets flow. **Discovery does
not complete: the participant is never admitted.**

### NOT established — what the next person should do

WHY the SPDP is not admitted. That needs the announcement's CONTENT compared
between a threadx and a native participant (GUID prefix, locators, lease
duration, domain tag), which is packet-level work this stopped short of.

Do not assume it generalises to the other two of the three. `service` is what
was diagnosed; `action` and `talker_to_native_listener` share a fixture family
and a plausible cause, and that is a hypothesis, not a result.

### Two measurement artifacts, kept because both looked like findings

1. **"Zero proxy participants discovered."** `new_proxy_participant` is not the
   trace spelling — the WORKING control reports zero for it too. Caught by
   running the same grep against the pair that passes, which is the only reason
   it was not written up as the finding.
2. **Unequal time budgets.** The control ran 25 s and the first threadx attempt
   20 s, so "the client never completes" was not comparable until the 75 s run
   made it so.


## The zephyr xrce-cpp cluster, DIAGNOSED 2026-09-04 (3 of the 12)

`case_{21,24,27}_xrce_cpp_{pubsub,service,action}_e2e`. The image says why, in
its own boot output:

```
*** Booting Zephyr OS build v3.7.0 ***
nros: HEAP EXHAUSTED: request 427968 bytes, arena 66048 bytes
      (raise CONFIG_NROS_ZEPHYR_HEAP_SIZE / NROS_ZEPHYR_HEAP_SIZE)
```

**All three, byte-identical** — same request, same arena. The test then reports
"listener received 0 sample(s)" and blames XRCE session-name collisions, which
is a stale hint from Phase 96.1: nothing was ever received because the image
never finished booting.

That the request does NOT vary across pubsub / service / action is the useful
part: it is a STATIC, knob-derived allocation, not a function of what the image
wires up.

From the built image's own `.config`:

```
CONFIG_NROS_ZEPHYR_HEAP_SIZE   65536      (the reported arena is 66048)
CONFIG_NROS_EXECUTOR_ARENA_SIZE    0      (the DERIVE sentinel)
CONFIG_NROS_EXECUTOR_MAX_CBS       4
CONFIG_NROS_EXECUTOR_ACTION_CLIENTS 4     (the worst case — issue 0900's subject)
```

**NOT established: what composes the 427,968.** The derived executor arena at
those knobs is 74,240 (measured for issue 0900), so the arena is a SIXTH of the
request and the rest is unaccounted. Deriving `ACTION_CLIENTS` would take the
arena to 16,384 and save 57,856 — real, and not enough on its own. Anyone
continuing should attribute the remaining ~350 KB before changing a knob;
raising `NROS_ZEPHYR_HEAP_SIZE` until it boots would hide the question this
campaign exists to ask.

## `test_qemu_rtic_service_e2e`, NARROWED not solved (1 of the 12)

The firmware reports its own failure:

```
[ERROR] nros: zpico Session -> ConnectionFailed (the two are indistinguishable downstream)
[ERROR] nros: RMW session open failed — ConnectionFailed
```

So the bare-metal server boots, brings up LAN9118 and smoltcp, gets its IP, and
then cannot open a zenoh session.

**The obvious cause is ruled out.** The test starts its own router
(`ZenohRouter::start_slirp`) and asserts the port is reachable before booting
any firmware — that assertion PASSED, since the run reaches "Starting RTIC
service server QEMU...". And the ports agree: `BAREMETAL.zenohd_port_for(Service,
Rust)` is 10210, and the firmware's `Cargo.toml` bakes
`locator = "tcp/10.0.2.2:10210"`. No mismatch.

So the router is listening on the host and the guest is dialling the right
gateway and port, and the connection still fails. The next suspect is
[issue 0774](archived/0774-zenohd-loads-unpaired-libzenohc.md): the router is ROS's `rmw_zenohd`, which loads whatever
`libzenohc.so` the loader finds and SEGVs mid-startup when the pairing is wrong
— a router that binds, satisfies `wait_for_port`, and is gone by the time
firmware connects would produce exactly this. Checking that means watching the
router process across the firmware boot, which this stopped short of.


## The zephyr xrce-cpp cluster: cause FIXED, cells still red for a DIFFERENT reason (2026-09-04)

The heap exhaustion diagnosed above is fixed (issues 0968/1033). The three cells
still fail, and the honest reading is that fixing one cause exposed the next.

### The allocation, measured per image

`sizeof(xrce_session_state_t)` after the MTU fix, the derivable-ladder wiring
and admitting a zero cap, against a 66,048-byte arena:

| image | before | after |
| --- | ---: | ---: |
| talker | 427,968 | **25,792** |
| service-client | 427,968 | **25,792** |
| service-server | 427,968 | **30,176** |
| action-server | 427,968 | **38,944** |
| listener | 427,968 | **59,088** |
| action-client | 427,968 | **59,088** |

All six fit. Re-running the three cells: **`HEAP EXHAUSTED` appears zero times**
in all three, where it was the first thing every image printed.

### They now fail on `zeth`, which was always missing and previously MASKED

```
<err> eth_posix: Cannot create zeth (-1)
```

`zeth` is the TAP device Zephyr's native_sim networking needs, created by
`net-setup.sh` as root. **This host has no `zeth` interface at all** — checked
against `/sys/class/net`.

It is not a regression, and the progression shows why it was invisible:

| image's session struct | `Cannot create zeth` | `HEAP EXHAUSTED` |
| ---: | ---: | ---: |
| 427,968 (original) | 0 | 1 |
| 76,624 (partly fixed) | 1 | 1 |
| 59,088 (fits) | 1 | **0** |

At 427,968 the image died in the allocation BEFORE it reached network setup, so
the missing TAP never printed. Each fix advanced it to the next unmet
precondition. That ordering is why this issue's original sample looked like "the
server did not start": it stopped earlier than anyone was looking.

### So what these cells prove, and what they do not

* **Proved:** the heap cause is real and fixed. Zero exhaustions, six images
  under budget, from a declaration the tree was already capturing and throwing
  away.
* **Not proved:** that the cells pass. They cannot on this host — the TAP device
  needs root, which an agent does not take. A host with `net-setup.sh` run is
  what closes them.

### A caution for whoever re-runs this

The FIRST re-run of these cells was contaminated by six leftover `zephyr.exe`
processes from manual debugging, which held the interface and produced the same
`Cannot create zeth` line. Those verdicts were discarded and the cells re-run
after killing every stray. A red you caused reads exactly like a red that was
already there — the per-cell `heap=` / `zeth=` counts above are recorded so the
next reader can tell them apart without taking anyone's word.
## The esp32 cluster, RUN for the first time 2026-09-04 (5 of the 12)

These five could not run at all until [issue 1025](archived/1025-esp32-flash-image-consumer-drops-the-row-variant.md) was fixed —
no ESP32 QEMU flash image could be packed. With that fixed and every image
rebuilt (talker, listener and ws-entry all stamped within minutes of the run, so
no stale-artifact confound):

| test | verdict |
| --- | --- |
| `logging_smoke_esp32_qemu_emits_every_severity` | **PASS** |
| `test_esp32_talker_listener_e2e` | FAIL |
| `test_esp32_to_native` | FAIL |
| `test_native_to_esp32` | FAIL |
| `test_esp32_workspace_entry_e2e` | FAIL |

**One of the twelve is closed by 1025 alone.** `logging_smoke` is the test whose
image `just esp32 build-logging-smoke` produces, and it passes now that the
image exists. That is the whole of what 1025 bought here, and it is worth
saying plainly: 1025 was NECESSARY for all five and SUFFICIENT for one.

### The four that still fail share a shape, not yet a cause

Every image BOOTS and brings up its network. From the listener's own output:

```
nros ESP32-C3 QEMU Platform
Initializing OpenETH...   MAC: 02:00:00:00:00:01
IP: 10.0.2.51/24  Gateway: 10.0.2.2
Ethernet ready.
Application setup complete — entering spin loop.
```

…and then it never prints `Subscriber created for topic:`. So **application
setup reports success and the entity is not there**, with no error on the way.
The stage each test dies at differs:

| test | last thing the image did |
| --- | --- |
| `talker_listener` | setup complete; no `Subscriber created for topic:` |
| `esp32_to_native` | no `Publishing:` within 60 s |
| `native_to_esp32` | listener `failed to start` |
| `workspace_entry` | never reaches `Application setup complete` |

The first three complete setup and never become functional; the fourth does not
finish setup. Whether that is one cause presenting at different stages or more
than one is NOT established, and this section stops there.

**Do not read the shape as the cause.** The obvious guess — the zenoh session to
the router fails, as it visibly does in the rtic cell — is a guess: these images
print no session error, whereas the rtic firmware prints two. Confirming it
means instrumenting the ESP32 side, not inferring from a sibling.

## The esp32 cluster, NARROWED (2026-09-04) — the entity exists; its marker does not print

Four of the five still fail after issue 1025 made their images buildable. This
narrows what is wrong and, more usefully, rules three things out.

### The subscription IS created

The listener's output ends with:

```
Application setup complete — entering spin loop.
```

That line means the run-plan closure returned `Ok(())`. The subscription is made
inside `register()` with `?`, so a failure there would propagate and this line
could not print. **The entity exists.** What is missing is the NEXT line,

```rust
log::info!("Subscriber created for topic: /chatter");
```

which is what the test greps for. So the shape is CLAUDE.md's documented pitfall
— "if a test times out, FIRST diff the grep pattern against what the fixture
actually prints" — and not, as it appears, an image that fails to subscribe.

### Three hypotheses ELIMINATED

* **No logger installed.** One is: `nros-board-esp32-qemu/src/node.rs:363` calls
  `esp_println::logger::init_logger(log::LevelFilter::Info)` inside
  `init_hardware`, and that function demonstrably ran — its own prints
  (`Initializing OpenETH`, `IP: 10.0.2.51/24`) are in the output.
* **Compile-time level stripping.** No `release_max_level_*` or `max_level_*`
  feature is enabled anywhere; the resolved features are `[]` / `["std"]` and
  the feature tree shows only `log feature "default"`. The macros are compiled
  in.
* **Two `log` facades, so `set_logger` served the wrong one.** WRONG, and it was
  a GREP ARTIFACT: the pattern `log v[0-9.]+` matched **`nros-log v0.5.0`**.
  There is exactly one `log` crate in the target graph (0.4.33), and it is the
  same one the example binds and the same one esp-println installs into. Anyone
  re-deriving this from `cargo tree` should anchor the match.

### So what is NOT established

Why a `log::info!` record, on a facade with a logger installed at `Info`, does
not reach a console that the same image's `println!` reaches. That is the
question, and reasoning from the dependency graph has now produced three wrong
answers.

The next step is the method that worked for the threadx cluster: run the image
under QEMU directly, add a `println!` beside the `log::info!`, and see which one
appears. That distinguishes "the record is dropped" from "the record is emitted
somewhere the capture does not read" in one run, where the graph cannot.

### The other three, unchanged

`test_esp32_to_native` (no `Publishing:`), `test_native_to_esp32` (listener
"failed to start") and `test_esp32_workspace_entry_e2e` (never reaches
`Application setup complete`) are not claimed to share this cause. The first two
plausibly do — they grep for the same kind of marker — and the third does not,
since it fails BEFORE the line that proves setup succeeded.

## The esp32 marker: every STATIC hypothesis is now closed (2026-09-04)

Adding a `println!` beside the `log::info!` was the plan. It did not survive
contact, and what it turned up is worth more than the probe would have been.

**Reachability was already proved, so the probe was the wrong instrument.**
`Application setup complete` prints only when the run-plan closure returns
`Ok(())`, and the `log::info!` sits INSIDE `register()` before that `Ok`.
Execution demonstrably reaches and passes the line. The question was never "is
it reached" but "why is the record dropped", and a `println!` next to it answers
the first.

**Five hypotheses, all closed:**

| hypothesis | verdict |
| --- | --- |
| execution never reaches the line | closed — see above |
| no logger installed | closed — `init_logger(LevelFilter::Info)` in `init_hardware`, which ran (its own prints are in the output) |
| compile-time level stripping | closed — no `max_level_*` / `release_max_level_*` feature; resolved features `[]` / `["std"]` |
| two `log` facades | closed — GREP ARTIFACT (`log v[0-9.]+` matched `nros-log v0.5.0`); there is one `log`, 0.4.33 |
| esp-println built without `log-04` | closed — the resolved features for this image include `log-04` |

So: one facade, logger installed on it at `Info`, macros compiled in, line
reached, and no record on a console the same image's `println!` reaches. Nothing
further is answerable from the dependency graph.

### A separate fact the attempt produced, and it reinforces issue 1025

**The listener does not LINK without its manifest row's env.** A bare
`cargo build` of the leaf fails at:

```
rust-lld: error: .../stack.x:11: unable to move location counter (0x3fccf734)
          backward to 0x3fcce400 for section '.stack'
```

With `ZPICO_MAX_QUERYABLES=2` — the value `env` on its `[[fixture]]` row
supplies — it links. The image sits close enough to its RAM ceiling that the
row's env is load-bearing for the LINK, not merely for size. That is the same
env whose absence from the packer's group-key computation was issue 1025, and it
is why probing this image by adding code is awkward: two `println!` string
literals and an `esp-println` dependency were enough to push it back over.

### What the next attempt should do

Probe WITHOUT growing the image: replace the `log::info!` with a direct write
through a path the image already links (the board registers a platform writer at
`nros-platform-esp32-qemu/src/lib.rs:87`, reached via
`PlatformLog::write`), rather than adding a print beside it. That needs a dep
the example does not currently have, so it is a temporary edit to the leaf's
manifest — build it with the row's env, and revert both.


## The esp32 marker, ANSWERED by probe (2026-09-04) — filed as issue 1048

The probe ran, through the path the board already exports, and the answer is
unambiguous. A `println!` on EACH side of the `log::info!`, image built with its
row's env, QEMU with a zenoh router up:

```
PROBE-A
PROBE-B

Application setup complete — entering spin loop.
```

**Both probes print; the `log::info!` between them prints nothing.** No
`log`-crate output appears anywhere in the boot. The subscription exists and the
line announcing it is dropped by the facade — so the cell greps for a marker the
image is structurally unable to emit, and reports a messaging failure for a
logging defect.

Filed as [issue 1048](1048-esp32-log-records-are-silently-dropped.md) with the
five closed hypotheses and the one remaining candidate (`log::set_logger`
succeeds at most once per process, and this image has two log paths racing for
the facade — the board registers a platform writer immediately above its
`init_logger` call).

That accounts for `test_esp32_talker_listener_e2e`. `test_esp32_to_native` and
`test_native_to_esp32` grep for markers of the same kind and plausibly share it,
unverified. `test_esp32_workspace_entry_e2e` does NOT — it fails before
`Application setup complete`.

### Two notes on getting there

* The first standalone run failed at `Executor::open failed:
  Transport(ConnectionFailed)` because no router was up — not comparable to the
  test, which starts one. Worth knowing before reading a solo esp32 run as a
  result.
* The leaf does not LINK without `ZPICO_MAX_QUERYABLES=2` from its `[[fixture]]`
  row, so a probe must be built with that env and must not grow the image; the
  board's re-exported `esp_println` is what makes a dependency-free probe
  possible.
