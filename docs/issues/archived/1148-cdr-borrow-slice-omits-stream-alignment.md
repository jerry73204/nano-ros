---
id: 1148
title: "`nros_cdr_borrow_le_slice_{i64,u64,f64}` omits CDR stream alignment — 8-byte views read garbage and desync every following field"
status: resolved
type: bug
area: c, serdes
related: [phase-428, rfc-0089, issue-1149]
resolved_in: "PR #635"
---

## Problem

`packages/api/nros-c/include/nros/view.h` read the sequence length prefix and
then took `*ptr` directly, with no alignment of the stream cursor to
`sizeof(CT)`. CDR requires 8-byte primitives to start on an 8-byte boundary
relative to the stream origin.

The writer aligned. The Rust reader aligned. The owned C reader aligned. Only
the borrow view did not.

Executed against a buffer the C serializer itself produced: `[1111, 2222]`
read back as `[4771708665856, 9543417331712]`, and the cursor was left 4 bytes
short, so **every field after the sequence misparsed**. The view returned 0
(success). Live in the committed corpus (`Probe.action.c`, an `int64[]` first
field). Only 2- and 4-byte element views were safe, and the only tests
exercised `u16` and `f32` — exactly the widths where alignment is a no-op
after a 4-byte length prefix.

The `serde.json` ledger row justified the design by HOST pointer alignment.
That conflates two requirements: the host may tolerate unaligned loads, but
the WIRE format places the element at an aligned offset, and reading from the
unaligned offset reads padding as data. Issue-1022 class.

## Resolution — FIXED in PR #635 (`fix(#1148, #1149): the C borrow views skipped the stream padding and trusted the wire count`)

`nros_cdr_align(ptr, end, origin, alignment)` is a new `nros-c` export: the
Rust `CdrReader::align` behind the C ABI, so the header-only views carry the
reader's own rule (alignment measured from the stream ORIGIN, never the buffer
pointer; XCDR2 caps 8-byte primitives at 4) rather than a C re-derivation that
could drift. The `nros_cdr_borrow_le_slice_*` macro calls it between the count
read and taking `*ptr`.

**Only when there is an element to align.** The writer pads before each
primitive it writes and never for an empty sequence, so an unconditional align
desyncs a 1-byte field (or the end of the buffer) that follows an empty
`float64[]`. The Rust reference reader had exactly that defect; fixed there
too, with a unit test for each layout.

Test: `packages/api/nros-c/tests/run/cdr_borrow_le_slice.c`, the one
compile+LINK+RUN TU in `just check c`, builds the stream with the C writers
and asserts the view begins where the writer put element 0 and the cursor
lands on the trailing field, for `u32 a; float64[] xs` (aligned by luck, the
task's literal layout), `u32 a; u32 b; float64[] xs` (four bytes of padding,
the layout that measured the defect), the `u64` integer family, and the empty
sequence. Negative control against the pre-fix header: 12 checks fail, 0
after. The ledger's sentence was corrected in all 10 rows that carried it.

`NROS_CODEGEN_VERSION` 2 → 3 (a new runtime symbol the generated headers
require); `_MIN` stays 2, a version-2 tree picks the fix up by recompiling.
