---
id: 913
title: "attaching pyocd RTT kills the zenoh session — the debugger perturbs the
  system under test, and issue 0879 makes the perturbation permanent"
status: open
type: bug
area: testing, embedded
related: [issue-0852, issue-0879, issue-0839]
---

## Measurement

Same image, same router, same board. The only variable is whether a debug probe
is attached.

```
no probe attached      ros2 action list -> /fibonacci   (twice)
pyocd rtt attached     ros2 action list -> empty        (twice)
probe detached again   ros2 action list -> empty        (twice)
router restarted       ros2 action list -> /fibonacci
```

Attaching the probe kills the session. Detaching does not bring it back; only a
router restart does, which is [issue 0879](archived/0879-serial-link-has-no-resync-after-peer-reset.md)
— the serial link cannot resynchronise once a peer has gone quiet.

## Mechanism

pyocd reads target memory over the debug port, which halts the core. On a link
whose liveness is a keepalive against `Z_TRANSPORT_LEASE`, enough halting means
missed keepalives, and the session expires at twice the lease. 0879 then turns a
transient stall into a permanent one.

## Why this matters more than a testing annoyance

**Every observation of this board made through RTT was made under the instrument
that breaks it.** That includes the crash filed as the root cause of
[issue 0852](0852-zephyr-serial-rx-is-polled-and-overruns.md) —
`_z_network_message_elem_copy`, "Illegal load of EXC_RETURN into PC". That fault
was captured with RTT attached, on an image also running at 98.7 % SRAM.

With the probe detached and the image at 92 % SRAM after the DTCM relocation
([issue 0880](0880-tcm-unused-while-sram-exhausted.md)), **action goals
complete**: 6 of 7, `SUCCEEDED`, correct sequences. The failure that motivated
0852's stack hunt does not reproduce.

So the honest position is that the crash was **at least partly induced** by the
conditions of its own observation, and 0852's remaining claim — that the receive
path is not implicated — still stands on the polled-versus-ISR A/B, which did
not depend on RTT.

## What to do instead

The post-mortem fault log (`CONFIG_NROS_FAULT_LOG`, phase-392 amendment D)
exists for this: it records the fault reason, PC/LR/PSR and the thread NAME into
`.noinit` from the fault handler and reports it at the next boot, with an
optional flash copy. Reading a post-mortem record needs no live session, so the
observation cannot destroy what it observes.

Rules that follow, worth writing into any bring-up runbook:

- **Do not attach a debug probe to a board whose transport is under test.**
  Attach it after the failure, to read what was recorded.
- A negative result gathered with a probe attached is not evidence.
- When a probe must be attached, restart the router afterwards before measuring
  anything, because 0879 will not let the link recover on its own.

## Fix direction

pyocd can be told to read memory without halting on targets that support
background access (`--no-halt-on-connect` style options, and DAP access modes
that do not stop the core). Whether the S32K344's port supports it is unverified
and worth an experiment — if it does, RTT becomes usable and this issue shrinks
to a documentation note. If it does not, the fault log is the answer.
