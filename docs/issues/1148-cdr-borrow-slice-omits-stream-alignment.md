---
id: 1148
title: "`nros_cdr_borrow_le_slice_{i64,u64,f64}` omits CDR stream alignment — 8-byte views read garbage and desync every following field"
status: open
type: bug
area: c, serdes
related: [phase-428, rfc-0089]
---

## Problem

`packages/api/nros-c/include/nros/view.h:93-104` reads the sequence length
prefix and then takes `*ptr` directly, with no alignment of the stream cursor
to `sizeof(CT)`. CDR requires 8-byte primitives to start on an 8-byte boundary
relative to the stream origin.

The writer aligns (`nros-serdes/src/cdr.rs:363`). The Rust reader aligns
(`:773`). The owned C reader aligns. Only the borrow view does not.

Executed against a buffer the C serializer itself produced: `[1111, 2222]`
read back as `[4771708665856, 9543417331712]`, and the cursor was left 4 bytes
short, so **every field after the sequence misparses**. The view returns 0
(success).

Live in the committed corpus:
`packages/cli/rosidl-codegen/tests/fixtures/fingerprint-corpus/expected/configured/Probe.action.c:238`.

Only 2- and 4-byte element views are safe, and the only tests exercise `u16`
and `f32` — exactly the widths where alignment is a no-op after a 4-byte
length prefix.

## The ledger's stated reason is wrong

`serde.json` `c:cdr_borrow_le_slice_f64` justifies the design by HOST pointer
alignment. That conflates two requirements: the host may tolerate unaligned
loads, but the WIRE format places the element at an aligned offset, and reading
from the unaligned offset reads padding as data. Issue-1022 class.

## Fix

Align the cursor before taking the pointer, as the three sibling paths do. This
needs an `nros_cdr_align` primitive that `cdr.h` does not currently expose to
the view macros. Add a test at 8-byte width. Correct the ledger row.
