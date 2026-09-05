---
id: 852
title: "the zenoh read task inherits the executor's priority on Zephyr — the
  declared priority is discarded by the port, so a 20 ms timeslice starves the
  polled serial RX and it overruns"
status: open
type: bug
area: rmw, platform
related: [issue-0848, issue-0839, issue-0736, issue-0626, rfc-0079]
---

## Problem

`_z_read_serial_internal` (`src/system/zephyr/network.c`) receives byte by byte:

```c
res = uart_poll_in(sock._serial, &raw_buf[i]);
if (res != 0) { if (past deadline) return SIZE_MAX; k_yield(); }
```

Polled RX is the surface. **The root cause is that the read task runs at the
same priority as the executor, because the priority it was given is thrown
away by this port** — so `k_yield()` in that loop hands the executor a full
scheduler timeslice while bytes keep arriving.

## Root cause: the priority is discarded twice

`CONFIG_NROS_ZENOH_READ_PRIORITY` (default 16 on the normalised 0-31 band) is
wired through Kconfig and CMake, reaches `zpico_set_task_config`, and is stored:

```
zpico.c:1431   g_default_read_nros_attr.priority = NROS_PLATFORM_PRIORITY_RAW(read_priority)
zpico.c:1443   g_default_read_task_opts.task_attributes = (z_task_attr_t *) &g_default_read_nros_attr
```

zenoh-pico then hands that attr to the port. Both Zephyr entry points drop it.

**1. the zenoh-pico system shim** — `zephyr/nros_zenoh_zephyr_system.c:29`

```c
z_result_t _z_task_init(_z_task_t *task, z_task_attr_t *attr,
                        void *(*fun)(void *), void *arg) {
    (void)attr;                                     /* <-- discarded */
    return nros_zephyr_task_create(task, fun, arg) == 0 ? 0 : -1;
}
```

**2. the platform task ABI itself** — `nros-platform-zephyr/src/platform.c:291`

```c
const nros_platform_task_attr_t *a = (const nros_platform_task_attr_t *) attr;
(void) a;                                           /* <-- discarded */
```

The comment there explains why `stack_bytes` cannot be honoured (Zephyr's
`k_thread_create` needs an MPU-aligned `K_THREAD_STACK_DEFINE` region). That
reasoning is sound and does **not** extend to `priority`:
`CONFIG_POSIX_PRIORITY_SCHEDULING=y` in these images and
`pthread_attr_setschedpolicy` / `setschedparam` work.

Neither call site sets `PTHREAD_EXPLICIT_SCHED`, so
`nros_zephyr_task_create`'s bare `pthread_attr_init`
(`nros_platform_zephyr_shims.c:438`) leaves the Zephyr default in place:

```
zephyr/lib/posix/options/pthread.c:950   attr->inheritsched = PTHREAD_INHERIT_SCHED
zephyr/lib/posix/options/pthread.c:653   -> new thread takes the CREATOR's priority
```

The read task is started from the executor thread, so it inherits
`CONFIG_MAIN_THREAD_PRIORITY=0` — identical to the executor.

### This is the failure issue 0626 warned about, one layer down

Issue 0626 fixed the Zephyr arm of `zpico_set_task_config`, and its own comment
says:

> `PTHREAD_EXPLICIT_SCHED` is load-bearing: the default is
> `PTHREAD_INHERIT_SCHED`, under which the policy and param set below are
> IGNORED and the new thread silently takes the creator's. A scheduling
> attribute that is quietly dropped is the failure this issue is about, so it
> must not be reintroduced one layer down.

It was reintroduced one layer down, in nano-ros's own port. Issue 0736 records
the same shape on NuttX ("zenoh read/lease threads inherit whichever thread
opened the session"), and [RFC-0079](../design/0079-priority-is-allocated-not-authored.md)
is the general statement of it.

## Why equal priority loses bytes — arithmetic, not a race

```
CONFIG_TIMESLICING=y
CONFIG_TIMESLICE_SIZE=20      ms
CONFIG_TIMESLICE_PRIORITY=0   every preemptible thread is timesliced
```

At equal priority `k_yield()` moves the reader to the tail of its ready queue
and the executor gets a full slice. 20 ms at 115200 baud is **~230 bytes**
against an LPUART RX FIFO a few entries deep.

The loop's own comment shows the trap was seen but mis-scoped:

> Yield rather than sleep between polls: at 115200 a byte is 87 us and the
> shortest `z_sleep_ms(1)` blocks a whole tick, overrunning the UART.

`k_yield()` at equal priority blocks for a whole **timeslice**, which is 20x
worse than the tick the comment was avoiding.

## Proof

Instrumented read loop reporting `uart_err_check`, against a locally built
zenoh router logging every serial write:

```
DIAG-RX: frame rb=9  ok hdr=0x03  uart_err=0x0     <- handshake, board idle
DIAG-RX: frame rb=83 ok hdr=0x00  uart_err=0x0
DIAG-RX: frame rb=15 ok hdr=0x00  uart_err=0x0
DIAG-RX: timeout, rb=4  uart_err=0x1 OVERRUN       <- keepalive frame, truncated
DIAG-RX: timeout, rb=0  uart_err=0x0  x many       <- nothing left to lose
```

The overrun flag is set on **exactly** the truncated frame and nowhere else.

## Why it looks load-dependent

The priority story explains the split completely:

- **handshake survives** — the executor is blocked in its wake wait, so the
  read task is the only runnable thread and `k_yield()` returns immediately
- **keepalives die** — three queryables, two publishers and their callbacks
  make the executor runnable, so each `k_yield()` costs a full 20 ms slice
- **the talker soaks for five minutes** — one publisher, executor blocked
  almost always, reader keeps up

That is the whole reason this presented as "actions are broken and pub/sub is
fine". No amount of stack or lease tuning could move it.

## The red herring worth recording

[Issue 0848](archived/0848-router-sends-no-keepalives-on-serial.md) chased this
as a router defect for a long time, ending on "the keepalive is a 1-byte write
that never frames". The 1-byte figure was the payload handed to the link;
z-serial frames it as `header(1) + len(2) + payload + crc32(4)` and
COBS-encodes it, so ~10 bytes reach the wire. The board caught 4 of them and
overran. **The small write was never the anomaly — the receiver was.**

The router is exonerated: its timer fires, the keepalive arm fires, `write_all`
+ `flush` both succeed, and the frames it emits are well formed.

## Fix

The board uses `uart_mcux_lpuart.c`, not the LinFlexD driver — worth stating
because it changes what is available:

```
CONFIG_UART_MCUX_LPUART=y
CONFIG_UART_NXP_LPUART_ASYNC_API_SUPPORT=y   eDMA path exists
CONFIG_SERIAL_SUPPORT_INTERRUPT=y
CONFIG_SERIAL_SUPPORT_ASYNC=y
```

Neither `CONFIG_UART_INTERRUPT_DRIVEN` nor `CONFIG_UART_ASYNC_API` is enabled.
Both are available, and `mcux_lpuart_fifo_read` drains the whole hardware FIFO
in a loop (unlike LinFlexD's one-byte shim), so the ISR path is efficient here.

Cheapest first:

1. **Honour the declared priority.** Pass `attr` through `_z_task_init`, give
   `nros_zephyr_task_create` a priority parameter, set `PTHREAD_EXPLICIT_SCHED`
   + `SCHED_FIFO` + the mapped band value. Fix the platform ABI's `task_init`
   in the same change — it accepts a priority it silently discards on every
   Zephyr image, which is a latent RT defect independent of this issue.
   Does not remove polling; removes the 20 ms starvation window.
2. **Call `uart_err_check` and surface overruns.** Diagnostics only. Keep it:
   invisibility is what let this hide behind six other hypotheses.
3. **`CONFIG_UART_INTERRUPT_DRIVEN=y`** — ISR does `uart_fifo_read` into a
   `ring_buf`, the reader blocks on a `k_sem`. RX stops depending on thread
   scheduling at all. This is the actual fix.
4. **`CONFIG_UART_ASYNC_API=y` + eDMA** — zero per-byte CPU, double-buffered
   via `UART_RX_BUF_REQUEST`. The real-time target.

1 and 2 are small and independently correct; land them first. 3 is the fix.

## Status — what has landed and what it measured

**Landed (steps 1 and 2).** The priority is honoured at all three sites that
dropped it, and overruns are reported:

- `_z_task_init` forwards `attr` to `nros_platform_task_init` instead of
  `(void)attr;`
- the Zephyr platform port maps the band and applies it with
  `PTHREAD_EXPLICIT_SCHED`, via a new `nros_zephyr_task_create_prio`
- **SCHED_RR, not SCHED_FIFO.** On Zephyr `SCHED_FIFO` selects the COOPERATIVE
  band (`lib/posix/options/pthread_sched.h`), and a cooperative busy-poller
  would never be preempted — trading a 20 ms starvation of the reader for an
  unbounded starvation of everything else. The posix port picks SCHED_FIFO
  because on Linux that is simply "the real-time policy"; the same constant
  means something different here, which is why the map is per-port.
- `CONFIG_MAIN_THREAD_PRIORITY=5` on the serial images. Necessary: SCHED_RR
  maps as `zephyr = NUM_PREEMPT - posix - 1`, so the top preemptible slot is
  Zephyr priority 0 — exactly where main sat by default. The reader cannot be
  placed above a main thread already holding the top slot, so honouring the
  priority alone changes nothing.

### The goal-completion numbers do NOT reproduce — treat them as withdrawn

An earlier revision of this issue recorded "0 of 5 goals before, 2 of 5 after"
as a measured improvement from the priority fix. **That does not hold up.**

Re-run under a fixed harness (`/tmp/trial.sh`: kill every 7447/serial holder by
PID, start the router and wait for its "Started Zenoh router" line, reset the
board exactly once, then measure) the result is **0 of 5 on both builds**, and
also 0 of 5 when the goals are sent immediately after connect with no
`ros2 node list` ahead of them. The 2 of 5 came from hand-run sequences whose
timing was not controlled, and it has not been reproduced since.

What IS reproducible and does hold:

| | node visible in `ros2 node list` |
| --- | --- |
| polled | yes |
| interrupt-driven | yes |

So the priority fix is still correct on its own terms — the three discard sites
were real, the `(void)attr;` lines are gone, and a declared priority now takes
effect — but **this issue can no longer claim a measured improvement in goal
completion.** Whatever stops goals from completing has not been isolated, and
the RX path is not currently implicated in it: both RX paths behave the same.

**Written but NOT enabled (step 3).** Interrupt-driven RX is implemented on the
zenoh-pico fork branch `fix/zephyr-serial-irq-rx`: `uart_fifo_read` in the ISR
into a `ring_buf`, reader blocked on a `k_sem`, falling back to the poll loop
when `CONFIG_UART_INTERRUPT_DRIVEN` is off. Board-side behaviour under it is
clean — full bring-up in 0.27 s, **zero** `uart_err_check` overruns, **zero**
ring overflows.

It ships **disabled**, but not for the reason a previous revision gave.

That revision claimed the ISR path broke the board's declarations from reaching
the router, on the strength of an A/B with a host talker as a control. With
[issue 0864](archived/0864-board-zid-is-identical-on-every-boot.md) fixed — the
board now draws a distinct zid per boot — **that claim is withdrawn**: the ISR
build's node appears in `ros2 node list` normally. The apparent difference was
the shared-zid confound, exactly as the first analysis said before it was
over-corrected.

One real defect was found in the ISR while chasing this, and is fixed: the
handler guarded its loop with `uart_irq_is_pending()` and `break`-ed when RX was
not ready. `uart_irq_is_pending` is also true for a TX cause, so that could exit
with the interrupt still asserted and re-enter forever. It is now the canonical
Zephyr shape — `uart_irq_update()` once, then drain while `uart_irq_rx_ready()`.

It stays disabled because it is not yet shown to be BETTER, not because it is
shown to be broken. Both paths currently complete zero goals.

## The zid confound is fixed — [issue 0864](archived/0864-board-zid-is-identical-on-every-boot.md)

The board drew the same zenoh id on every boot, so a router could not tell a
rebooted board from the peer it already had, and every reconnect-shaped
measurement depended on router history. That is fixed: three boots now give
three distinct zids.

Two claims in this issue were made under that confound and are corrected above:
the goal-completion improvement (withdrawn) and the ISR-breaks-declarations
finding (withdrawn).

**A separate operational hazard remains.** z-serial has no link-level resync, so
resetting the board while the router holds the link puts the router into a
repeating `Unexpected Init flag in message` and the link never recovers. The
router must be restarted after any board reset. This is not the same thing as
the zid problem and survives its fix.

## Adjacent, found while tracing this

`_z_read_serial_internal` calls `z_malloc` **twice per frame**
(`_Z_SERIAL_MAX_COBS_BUF_SIZE` + `_Z_SERIAL_MFS_SIZE`) on the receive hot path.
Unbounded-latency allocator calls inside RX. Belongs to the heap-unification
campaign, not to this fix.

## Impact

Any zenoh-over-serial image whose executor is busy enough to hold a timeslice
loses frames silently. Observed as session expiry at `2 x Z_TRANSPORT_LEASE`
([issue 0839](archived/0839-action-image-session-expires-every-20s.md)), because the
dropped frames are the router's keepalives.

## Step 3 measured on the shipping pin: the ISR does not earn its place

Interrupt-driven RX is implemented and now correct -- see the rebind fix below --
but it does not improve the failure this issue is about, so
`CONFIG_UART_INTERRUPT_DRIVEN` stays `n`.

Same harness for every row: fresh router, fresh board boot, ten sequential
`ros2 action send_goal /fibonacci` with order 5.

| zenoh-pico pin | RX path              | goals    | router errors |
| -------------- | -------------------- | -------- | ------------- |
| 1.7.2 (ships)  | polled               | **8/10** | 0             |
| 1.7.2 (ships)  | ISR + rebind fix     | **6/10** | 0             |
| 1.10           | polled               | 8/10     | 0             |
| 1.10           | ISR + rebind fix     | 0/7      | 3             |
| 1.10           | ISR, no rebind fix   | 0/10     | 80            |

Read carefully:

* **The ISR is not better on the pin we ship.** 6/10 against 8/10 polled, and it
  costs 1096 bytes of RAM in an image already at 94.4 %.
* **6 vs 8 out of 10 is not a strong separation.** Two failures per arm. This
  supports "the ISR is not better", NOT "the ISR is worse". Thirty or more goals
  per arm would be needed to claim the latter.
* **The catastrophic rows are 1.10, not the ISR design.** The same ISR code with
  the same fix scores 6/10 on 1.7.2 and 0/7 on 1.10. That isolates the remaining
  defect to the 1.10 port, so it belongs with the migration branch and is not
  this issue's problem.

### The rebind fix is worth landing regardless

`_z_open_serial_from_dev` calls `uart_configure()` immediately before
`_z_serial_rx_bind`, on every open, and a reconnect is an open. On the mcux
LPUART that reinitialises the peripheral and clears the RX interrupt enable,
while the bind returned early for an already-bound device and skipped
`uart_irq_rx_enable`. RX was armed on the first open and never again, so a single
dropped session left the receiver dead for the life of the image.

That is a real bug on the shipping pin, independent of whether the ISR is ever
enabled, and it is inert while the knob is off. Its effect is visible in the
table: 80 router reconnect errors before, 3 after on 1.10, 0 on 1.7.2.

Diagnosed with counters in `.bss` read over SWD, because RTT kills the session
(issue 0913). Those counters also refuted the ring-overflow theory outright:
high-water 19 bytes of 1024, zero drops, while 145 bytes arrived in total and the
reader timed out 75 times.

---

# REOPENED 2026-09-04 — the fix landed, had no effect, and made it worse

The status above says the priority "is honoured at all three sites that dropped
it". It is honoured at none of them on Zephyr. Every fact below was verified
directly in this checkout.

## The mechanism is a STRUCT TYPE CONFUSION, not a discarded attribute

zenoh-pico dispatches `ZENOH_ZEPHYR` **before** `ZENOH_GENERIC`
(`system/common/platform.h:37` vs `:55`), so Zephyr gets
`system/platform/zephyr.h:39`:

```c
typedef pthread_attr_t z_task_attr_t;
```

not the generic header's `typedef nros_platform_task_attr_t z_task_attr_t`. But
`zephyr/nros_zenoh_zephyr_system.c:58` casts that pointer straight through:

```c
return nros_platform_task_init((void *)task, (void *)attr, fun, arg)
```

The layouts do not match, and the read is out of bounds:

| | 32-bit | 64-bit |
| --- | ---: | ---: |
| `struct pthread_attr` = `{void *stack; uint32_t details[2];}` | **12 B** | **16 B** |
| offset of `nros_platform_task_attr_t::priority` (after `name`, `stack_bytes`, `stack_mem`) | **12** | **24** |

So `priority` is read from **past the end of the object** on both.

## Why this is a REGRESSION, not merely an unfixed bug

`pthread_attr_init` leaves `stack = NULL` unless `CONFIG_DYNAMIC_THREAD_STACK_SIZE`
is set, and no built image sets it. The bytes at the priority offset are the
adjacent `lease_task_attr.stack`, i.e. zero. So:

```
a->priority == 0  ->  band 0  ->  sched_get_priority_min(SCHED_RR) = 0
                  ->  POSIX_TO_ZEPHYR_PRIORITY(0, SCHED_RR) = 15 - 0 - 1 = 14
```

Measured against `zephyr-workspace/build-rust-talker-zenoh/zephyr/.config`:
`CONFIG_NUM_PREEMPT_PRIORITIES=15`, `CONFIG_MAIN_THREAD_PRIORITY=0`.

**Transport at 14 — the least urgent preemptible slot. Executor at 0 — the most
urgent.** Before the fix the read task merely TIED with the executor. The fix
moved it to the bottom of the band. `a->name` reads the same NULL, so the thread
naming added in the same commit is inert too.

## The tell was already written down, and Zephyr was left off the list

`zpico.c:1642` guards the correct assignment with

```c
#if defined(ZENOH_NUTTX) || defined(ZENOH_LINUX) || defined(ZENOH_MACOS)
```

and its own comment describes this exact hazard — "*Pointing at
`read_task_attr` handed it a `pthread_attr_t`, whose bytes at the `priority`
offset are 0 whatever was...*". That is issue 0803. It was fixed for three
platforms and Zephyr was omitted — the one platform whose `z_task_attr_t`
genuinely IS a `pthread_attr_t`, and therefore the only one where the bug is
guaranteed rather than incidental.

## Two normalised bands meet in one scheduler — issue 0623, one platform over

Fixing the pointer alone is NOT enough:

* `zpico_posix_set_priority` (`zpico.c:1311`) maps a **0-31** band.
* `nros_zephyr_native_priority` (`platform.c:473`) maps a **0-255** band
  (`NROS_PLATFORM_PRIORITY_MAX`).

The Kconfig default is `16`, documented "normalized 0-31". Through the 255-wide
map that is `(16*14)/255 = 0` — still the bottom slot. The two spellings must be
unified to one band in the same change, or the fix reproduces the defect.

## The static gate certifies a function whose output is discarded

`packages/boards/zephyr/nros-board.toml` declares `derived = "zephyr"`, so
`scripts/lib/priority_plan.py:resolve_zephyr_plan` models
`zpico_posix_set_priority` — the function filling the `pthread_attr_t` nobody
reads. `check-tier-priority-plan-image` is therefore green about a band no image
ever had, while `examples/workspaces/realtime-c/.../system.toml` authors
`tiers.high.zephyr = 9` against that fiction. **The declared ordering is exactly
inverted from reality and no gate can see it.** Zephyr also has no sibling of
FreeRTOS's `report_tiers_above_transport` (`nros-board-freertos/src/entry.rs:838`),
which is the boot-time check that cannot be fooled by dead code.

## Retracted / corrected from the text above

* "Landed (steps 1 and 2)" — there was a FOURTH site (`zpico.c:1642`) and it was
  not touched. Net effect is a regression.
* The root-cause citations (`zpico.c:1431`/`:1443`) name the **Linux/NuttX/macOS**
  arm, not Zephyr's. Reading them as Zephyr's is what produced the bad cast.
* "`nros_zephyr_task_create_prio` — the native SCHED_FIFO priority"
  (`nros_platform_zephyr_shims.c:481`) contradicts its own code, which uses
  `SCHED_RR` (`:534`).
* The timing arithmetic assumes `CONFIG_TIMESLICE_SIZE=20` and
  `CONFIG_MAIN_THREAD_PRIORITY=5`. Neither appears in this repo; all sampled
  images have `TIMESLICE_SIZE=0`, `MAIN_THREAD_PRIORITY=0`. The 20 ms / 230-byte
  figure is true of an out-of-tree board only, and the issue does not say so.
* The title still describes the PRE-fix state ("inherits the executor's
  priority"). Post-fix it is worse than inheritance.
* **Stands:** choosing `SCHED_RR` over `SCHED_FIFO` (on Zephyr `SCHED_FIFO`
  selects the cooperative band). That reasoning is correct and should survive
  any rewrite.

## Adjacent, unlinked, still a live bug on the fork

zenoh-pico's own `src/system/zephyr/system.c:158` also drops a non-NULL `attr`.
`nros_rmw_zenoh.cmake` adds only `network.c`/`isotp.c` by name, so it is not
linked here — but anyone who does compile it gets the same class.

## What a fix must include

1. Point Zephyr at the `nros_platform_task_attr_t` (add `ZENOH_ZEPHYR` to the
   `:1642` guard), and delete `zpico_posix_set_priority`'s Zephyr arm rather
   than leaving a second spelling of a scale beside it — that is how 0623
   happened.
2. Do the same for the **tx-flush** task (`zpico.c:1739`), which is live
   whenever `CONFIG_NROS_ZENOH_TX_BATCH` is on. Same class, same commit.
3. Unify the band to 0-255 in Kconfig, and STATE the ordering. Transport above
   the app is what `[board.priority_plan]` already declares and what FreeRTOS
   ships (app 3 / transport 4); the reverse starves the RX drain (0623). Note
   `CONFIG_MAIN_THREAD_PRIORITY=0` leaves no room above it.
4. Re-point `resolve_zephyr_plan` at `nros_zephyr_native_priority`, or the gate
   keeps certifying the deleted map.
5. A `_Static_assert` in the shim that the cast is sound. It fails today, which
   is the point.

## Deployment note — a Kconfig DEFAULT does not reach an already-configured build

Found while verifying the fix, and it cost an hour. Changing
`NROS_ZENOH_READ_PRIORITY`'s default in `zephyr/Kconfig` from 16 to 200 left
every existing `zephyr-workspace/build-*` image on **16**, and
`cmake <build-dir>` did **not** move it: Zephyr's `.config` is STICKY (it keeps
`.config.old` beside it and treats the saved value as authoritative). So a
re-configure — normally the sanctioned escape hatch for a bad generated build
file — is not enough here.

The targeted re-derive is to delete the generated config and let Kconfig
rebuild it:

```bash
find zephyr-workspace -maxdepth 3 -path "*/zephyr/.config*" -delete
just zephyr build-rust-examples
```

That is not a wipe: build dirs, objects and caches are untouched, and only the
generated file that is the problem is removed. Afterwards:

```
CONFIG_NROS_ZENOH_READ_PRIORITY=200
[ok] build-rust-talker-zenoh: transport [4, 4], pool [5, 14] — 8 pin(s)
tier-priority-plan-image: OK (48 pin-check(s) over 6 image(s))
```

**CI is unaffected** — it configures from scratch — so this is a local-tree
hazard only, and exactly the kind that makes a fix look inert when it is not.

### The gate had to be made able to FAIL before it could confirm anything

The sequence is the argument for the fix, and it is worth keeping:

| | transport | tiers | `check-tier-priority-plan-image` |
| --- | --- | --- | --- |
| before | k_thread **14** (bottom) | 9, 10 | **OK** — modelled the dead function |
| corrected gate, stale `.config` | 14 | 9, 10 | **FAILED** — inversion finally visible |
| after the fix | **4** | 9, 10 | **OK**, and truthfully |

The middle row matters most: a gate that models code nobody runs cannot fail,
and a gate that cannot fail cannot confirm. It caught the real inversion in the
first run where it was capable of seeing one.
