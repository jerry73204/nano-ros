---
id: 1088
title: "cyclone `take_request` consumes a request, destroys it, and reports an
  empty queue — 33rd request onward is lost with `taken = false`"
status: open
type: bug
area: rmw
related: [phase-428, rfc-0089]
---

## Problem

`cyclonedds/src/service.cpp:826-853`. `take_typed_wire` is a **destructive**
`dds_takecdr` + `ddsi_serdata_unref` (`:535`, `:551`). Only *after* consuming
the sample does it look for a free correlation slot, and when there is none it
returns `WOULD_BLOCK` (`:853`) — which the adapter collapses to
`*taken = false; return OK` (`:879-881`).

`kRequestSlots = 32` (`:241`), freed only in `send_response` (`:983`). So with
32 requests outstanding, every further request is taken off the wire,
destroyed, and reported to the caller as "nothing there".

Upstream (`rmw.h:2348, 2353`): `taken == false` with `RMW_RET_OK` means
**nothing was consumed**.

## The sibling gets it right

`xrce/src/service.c:293` returns `WOULD_BLOCK` **without popping the ring**, and
`:335-338` maps only `NO_DATA` to OK. Two implementations of one contract
disagreeing about both the loss and the reporting.

## Impact

Silently wrong. A busy server stops answering and looks idle; the client sees
timeouts. Nothing counts the drop.

## Fix

Reserve the correlation slot **before** the destructive take, or peek before
taking. Report exhaustion as a distinct condition — `WOULD_BLOCK` reaching the
caller, or a counted-and-logged drop — never as an empty queue.
