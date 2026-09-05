---
id: 997
title: "The timed-event tree empties itself on FreeRTOS: the SPDP resend is scheduled, never lands in the queue, and every peer expires the participant's lease"
status: open
area: [rmw, platform, embedded]
severity: high
related: [0917, 0888, phase-177]
---

# The announcements are correct, on schedule, and then they stop

## What was measured

Autoware Safety Island on QEMU `mps3-an536` (FreeRTOS, Cortex-R52, nano-ros
Cyclone backend), against a ROS 2 Humble publisher on the host over stock
`rmw_cyclonedds`, on a paced tap. Cyclone discovery tracing on the HOST side:

```
1788405861.745  SPDP ST0 1108270:34688031:ce03eb9a:1c1 bes fc3f NEW
                  (nano-ros/0.10.5/Linux/Generic) data udp/192.0.3.10:58376
1788405942.943  gc: lease expired: l 0x… guid 1108270:34688031:ce03eb9a:1c1
                  tend 1883239996692042 < now 1883239996755175
1788405942.943  gc: ddsi_delete_proxy_participant_by_guid(…:1c1) - deleting
1788405942.943  gc: delete_ppt(…:1c1) - deleting endpoints
```

Then every proxy reader and writer belonging to that participant is
garbage-collected, and the publisher's own match count goes `1` → `0` three
seconds later:

```
1788405861  matched kinematic_state=1 acceleration=1 steering_status=1
1788405946  matched kinematic_state=0 acceleration=0 steering_status=0
```

**Announcements from the island in the whole run: one.**

```
$ grep -c "SPDP ST0 1108270:34688031:ce03eb9a:1c1" cyclone-discovery.log
1
$ grep -c "SPDP" cyclone-discovery.log
2
```

## Why one is wrong

The embedded config in `nros-rmw-cyclonedds/src/session.cpp`
(`kEmbeddedCycloneConfig`) sets no `Discovery/SPDPInterval` and no
`Discovery/LeaseDuration`, so Cyclone's defaults apply
(`third-party/dds/cyclonedds/src/core/ddsi/defconfig.c`):

```c
cfg->spdp_interval  = INT64_C (30000000000);   /* 30 s */
cfg->lease_duration = INT64_C (10000000000);   /* 10 s */
```

and `handle_xevk_spdp` schedules the resend
(`src/core/ddsi/src/q_xevent.c:1044-1056`):

```c
/* schedule next when 80% of the interval has elapsed, or 2s
   before the lease ends, whichever comes first … */
else if (ldur < DDS_SECS (10)) intv = 4 * ldur / 5;
else                           intv = ldur - DDS_SECS (2);
if (intv > gv->config.spdp_interval) intv = gv->config.spdp_interval;
```

With `ldur = 10 s` that is `intv = 8 s`, capped by the 30 s interval — so the
island should announce roughly every **8 seconds**, about ten times across the
81-second window. It announced once.

The `tev` thread that drives those timed events is not missing: the same config
names it and gives it a 16 KiB stack, precisely because the FreeRTOS default of
1 KiB is too small for any Cyclone worker (phase 177.26). So this is not "the
thread was never created" — that would have failed differently and earlier.

## What it costs

Every DDS peer deletes the island roughly a minute or two into a run, and it
never comes back. On the receive side that is total: after the deletion the
island's readers no longer exist for the publisher, and the application sees
nothing arrive again, forever, while its control loop keeps spinning and
correctly reporting that its inputs are stale.

This is a long-run defect, so a short test does not see it. The an536 delivery
sweep (`autoware-safety-island/scripts/an536-size-sweep.sh`) walks seven
trajectory sizes; a run that finishes inside the window reports every size
delivered, and a run that crosses it reports every size failing — including
908 B, which is a single fragment and involves no fragmentation at all. That
non-determinism is what sent an entire day's measurements chasing a payload-size
cliff that was not there. Worth stating plainly for issue
[0917](0917-an536-fragmented-sample-never-syncs.md), whose recorded
size-vs-rate curve was presumably taken inside the window: a sweep of this shape
cannot distinguish "too big" from "the participant is gone", and its numbers
should be re-taken once this is fixed.

## AMENDED 2026-09-03 — island-side tracing, and the headline was wrong

Everything above is the HOST's view, and it supported the wrong conclusion. With
tracing added on the ISLAND (a `<Tracing>` block in `kEmbeddedCycloneConfig`,
plus `q_xevent.c`'s two "xmit spdp" `GVTRACE` calls routed to `GVLOGDISC` so the
transmit side lands in the cheap `discovery` category rather than the `trace`
firehose — the firehose over semihosted stdout is slow enough to change the
timing of the thing being measured):

```
1788407870  tev: xmit spdp … (resched 8s)
1788407878  tev: xmit spdp … (resched 8s)
1788407887  tev: xmit spdp … (resched 8s)
1788407895  tev: xmit spdp … (resched 8s)
1788407903  tev: xmit spdp … (resched 8s)
1788407909  tev: xmit spdp … (resched 1s)   <- directed reply to the publisher
1788407911  tev: xmit spdp … (resched 8s)   <- the last one, ever
```

**Seven transmits, on the `tev` thread, at exactly the 8-second cadence the
scheduler computes.** The interval is not absent, the timed-event thread is not
missing, and the config is not at fault — every conclusion in the section above
about a missing announcement is wrong. What happens is that the transmits STOP,
while the island keeps running for another 161 seconds, still logging.

The correlation is tight, and it is not elapsed time:

| time | event |
| --- | --- |
| 1788407909 | publisher reports `matched … =1` |
| 1788407911 | island's LAST spdp transmit |
| 1788407921 | host: `gc: lease expired` → proxy participant deleted |
| 1788407924 | publisher reports `matched … =0` |

Expiry lands 10.3 s after the last transmit — exactly the advertised
`lease_duration` of 10 s. So the "81 seconds rather than 10" puzzle below is
resolved and needs no theory: the lease simply ran from whenever the transmits
stopped. **The data-traffic-renewal speculation in that section is retracted.**

Two findings, and they should not be conflated:

1. **The `tev` thread stops doing its work within ~2 s of the first real traffic
   arriving.** That is the defect. It is a stall, not a missing interval, and its
   trigger looks like load rather than time.
2. **Of seven SPDP packets sent, the host logged one arriving.** Separate, and
   unexplained. Six went missing on a paced local tap, which is its own question
   — and if it were the whole story the lease would still have been renewed by
   the ones that did arrive.

A stalled `tev` also stops PMD heartbeats and lease renewals, which fits the
symptom exactly. Candidates worth separating, in the order they are cheap to
test: `tev` starved or blocked once `recv` / `dq.user` begin real work, or `tev`
overflowing its 16 KiB stack — the same class of bug phase 177.26 fixed for
`recvUC` at 1 KiB, and FreeRTOS's `an536-tasks.py` reports per-task high-water
marks over the QEMU gdb stub.

## ROOT CAUSE LOCATED 2026-09-03 — the event tree is empty

Read live over the QEMU gdb stub, on a stalled island, via
`dds_global.m_domains.root` -> `dds_domain.gv.xevents`:

```
xevents                     = {roots = 0x0}      <- timed-event tree EMPTY
msg_xevents                 = {root  = 0x0}
non_timed_xmit_list_oldest  = 0x0
non_timed_xmit_list_newest  = 0x2170b228         <- non-NULL
terminate                   = 0
cond.tasks                  = {len = 10, cnt = 1}
```

The SPDP resend that the last handler scheduled 8 seconds out — the trace line
reads `(resched 8s)` at `cyclone+16.003` — **is not in the queue**. So `tev` is
not failing to wake: it correctly computed "nothing is scheduled", waited with
`portMAX_DELAY`, and FreeRTOS put it on the suspended list. Its indefinite wait
is the SYMPTOM of an empty tree, not a defect in the wait.

The queue is not shutting down (`terminate = 0`), and the waiter is properly
registered (`cond.tasks.cnt = 1`).

`non_timed_xmit_list` is also inconsistent: `oldest == NULL` while
`newest != NULL`. A singly-linked list with a tail and no head, in the same
struct that lost its event tree.

So the question is why a scheduled event does not end up in `xevents`. The
mechanism to look at first is `resched_xevent_if_earlier`, which re-arms ONLY
when the new time is earlier than the stored one and silently does nothing
otherwise — a lost update rather than a deadlock, which is also what the
half-updated transmit list suggests.

### Ruled out, by measurement rather than argument

Each of these was a live hypothesis in this issue at some point. Recording the
eliminations because re-deriving them is the expensive part:

| hypothesis | why it is wrong |
| --- | --- |
| interval absent / `tev` never scheduled | seven transmits at the computed 8 s cadence |
| `tev` stack overflow (phase 177.26 shape) | `used_below_sp` is FREE space below a descending stack: `tev` had 15912 of 16384 bytes unused |
| heap exhaustion failing `ddsrt_tasklist_push` | `xMinimumEverFreeBytesRemaining` = 26,825,856 of 33,554,432. It never came close, and `cond.tasks.cnt = 1` proves the push succeeded |
| lock convoy behind one stuck object | no two tasks share a wait object (`xEventListItem.pvContainer` differs for all) |
| FreeRTOS tick / monotonic clock stalling | wall-clock vs Cyclone-internal drift is ~0.0 s across every transmit |
| `tcpip_thread`'s NULL lwIP TLS semaphore | normal. `LWIP_NETCONN_SEM_PER_THREAD` semaphores are for netconn CLIENTS; lwIP's core thread is not one, nor are IDLE, `Tmr Svc`, or `poll` (netif input, tcpip mbox). Every socket-using thread has one |
| CPU starvation by the priority-7 app task | IDLE is the running task |

Also worth stating so the next reader does not chase it: `dq.user` and
`dq.builtins` sitting suspended is NORMAL — a delivery queue waits indefinitely
when empty. The anomaly is narrowly `tev`, because `tev` should always have the
SPDP resend pending.

### Instruments

`autoware-safety-island/scripts/an536-blocked-on.py` (new) reports, per task,
the object it is waiting on, by reading `xEventListItem.pvContainer` and mapping
it back to the owning queue. That is what separates "blocked on a queue" from
"blocked on a direct task notification" — the latter has NO wait object, which
is how ddsrt implements condition variables on FreeRTOS
(`ddsrt_cond_wait` -> `ulTaskNotifyTake`, `ddsrt_cond_signal` ->
`xTaskNotifyGive`), and is what identified these three threads as condvar
waiters rather than lock victims.

## What was NOT explained (superseded by the amendments above)

**Why the lease survived 81 seconds rather than 10.** With a 10-second advertised
lease and a single announcement, expiry should land near `+10 s`, not `+81 s`.
The likely reason is that Cyclone renews a proxy participant's lease on receiving
any traffic from it, and the island was publishing control commands for most of
that window — so the lease rode on data, not on SPDP, and expired when the data
stopped. That ordering matters for the fix, because it means SPDP silence may be
the second symptom rather than the first: something may stop the island
transmitting altogether, with the lease expiry following.

Whoever picks this up should establish that ordering before assuming the timed
event is at fault. Two cheap discriminators:

1. Trace with `<Category>discovery,throttle</Category>` on the host and find
   whether island DATA stops at the same instant SPDP would have been due.
2. Trace on the ISLAND side. Everything above is the host's view; nothing here
   proves whether the island sent an announcement that was lost versus never
   sent one. `kEmbeddedCycloneConfig` has no `<Tracing>` block, so that needs
   adding — and on a target with semihosted stdout, sized accordingly.

## Reproduce

`autoware-safety-island` at pin `2b03606ca` or `d2a8955c5` (both reproduce):

```
sudo ip link set tap1 up                     # 192.0.3.1/24, netem 100mbit
ASI_RX_COUNTERS=1 NROS_DOMAIN_ID=2 ./build.sh --platform freertos-an536
# boot the image with -net tap,ifname=tap1, then publish with a
# CYCLONEDDS_URI whose <Tracing><Category>discovery</Category> writes a log
```

The island's own RX counters, read over the QEMU gdb stub
(`scripts/an536-rx-counters.py`), freeze at the deletion and never advance:

```
                t0        t+20s
asi_rx_traj     154        154
asi_rx_odom     635        635
asi_rx_accel    613        613
asi_rx_steer    612        612
```

Confirm the guest is still running before believing that — gdb halts the target
on attach, and a failed resume produces the same frozen numbers for an entirely
different reason. Here the island's log kept advancing across the reads.

---

# CORRECTION 2026-09-04 — two of the observations above are not evidence

A full audit of the vendored fork's timed-event handlers (see
[#1000](archived/1000-spdp-periodic-event-orphaned-by-handler-early-return.md), whose own
proposed fix was found unsafe and is corrected there) retracts two claims made
above. Both were read as corroborating anomalies; neither is one.

## RETRACTED — "`oldest == NULL` while `newest != NULL`" is not an inconsistency

The text above calls this "a singly-linked list with a tail and no head". It is
the documented drained state. The field is declared:

```c
/* src/core/ddsi/src/q_xevent.c:149 */
/* undefined if ..._oldest == NULL */
```

and `getnext_from_non_timed_xmit_list` (`q_xevent.c:260-274`) deliberately never
clears `newest`. **Every** drained non-timed list looks exactly like this. It is
a stale pointer by design, so it corroborates nothing and should not have been
presented beside the empty-tree reading.

## RETRACTED — the `resched_xevent_if_earlier` lost-update theory cannot happen here

The text above names it "the mechanism to look at first", on the grounds that it
re-arms ONLY when the new time is earlier. On the post-fire path it always
re-arms: `handle_xevents` stamps `xev->tsched.v = DDS_NEVER` (`INT64_MAX`) at
`q_xevent.c:1232` *before* calling the handler, so any finite reschedule
satisfies `tsched.v < ev->tsched.v` and the insert happens (`:400-408`).

The only way that comparison no-ops is a concurrent `delete_xevent` having set
`TSCHED_DELETE`, which is the documented and intended behaviour
(`q_xevent.c:394-398`). So this is not a lost update and not the first thing to
look at.

## What the empty tree actually implies

Still open, and now better constrained. #1000's SPDP orphaning **cannot alone**
produce `xevents = {roots = 0x0}`: `handle_xevk_pmd_update` reschedules
unconditionally (`q_xevent.c:1097`) on a finite lease, and the lease is the
default 10 s. An empty tree needs SPDP **and** PMD orphaned together, which
means the participant became **un-findable in the entity index** while the
application was still running.

That is the thing to hunt, and it is upstream of both handlers. The next run
should therefore add a `GVLOGDISC` to `handle_xevk_pmd_update:1073-1075` — which
today returns with **no logging at all** — alongside the two SPDP lines #1000
names. Two "unknown guid" lines within ~8 s of each other confirms it.

Also worth suspicion on timing alone: the submodule tip `d97a71e` is
"ddsrt: one funnel heap for every port". If neither handler logs and no further
`xmit spdp` appears, the loss is at INSERT time and `ddsrt/src/fibheap.c` is
where to look, not the handlers.
