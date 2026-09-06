---
id: 1149
title: "`nros_cdr_borrow_le_slice_*` multiplies a wire-supplied count by element size without overflow check — on every 32-bit target that is an unbounded read"
status: resolved
type: bug
area: c, serdes, security
related: [phase-428, issue-1148]
resolved_in: "PR #635"
---

## Problem

`packages/api/nros-c/include/nros/view.h`:

```c
size_t bytelen = (size_t)cnt * sizeof(CT);
```

`cnt` comes off the wire. On `arm-none-eabi` — measured, `__SIZE_MAX__` is
`0xffffffffU` — `cnt = 0x40000000` with `CT = float` wraps `bytelen` to 0. The
bounds check then passes for any buffer, the UN-overflowed `count` is stored,
and the `_get` accessor had no bounds check at all. Every embedded target this
project ships for is 32-bit, and the defect is structurally invisible to the
native lane, where `size_t` is 64 bits. Untrusted wire data produced an
out-of-bounds read, reported as success, on the RMW receive path's input.

## Resolution — FIXED in PR #635 (`fix(#1148, #1149): the C borrow views skipped the stream padding and trusted the wire count`)

`nros_cdr_seq_byte_len(count, elem_size, limit, &out)` in `nros/cdr.h` is the
ONE place a wire count meets `sizeof`: it returns -1 when `count * elem_size`
would exceed `limit`, and every call site passes `SIZE_MAX`. The limit is a
parameter so the 64-bit native lane can exercise the 32-bit wrap it cannot
otherwise see: the run test feeds `0x40000000 * 4` and `0xFFFFFFFF * 8`
against a 32-bit limit and asserts -1, then feeds the macro path a wire count
of `0xFFFFFFFF` and asserts an error with the view untouched — on a 32-bit
`size_t` that is the overflow check firing, on 64-bit the bounds check, and the
test prints which. `_get` returns the zero value for an index at or past
`count` rather than reading outside the view.

The class sweep (command in the commit) found the same product in two more
places and fixed both: the generated heap-sequence `_deserialize`
(`packs/c/_field.jinja`) passed `(size_t)len * sizeof` to
`nros_platform_malloc`, so the wrap under-allocated and the element loop then
wrote past it — now routed through `nros_cdr_seq_byte_len`, golden corpus
regenerated; and the Rust reference reader's `read_le_slice` computed
`len * T::SIZE` unchecked — now `checked_mul`. `nros_cdr_borrow_string` /
`_bytes` (1-byte elements, no product), `LeSpan::operator[]` and the view
`_get` index arithmetic (bounded by a count checked at borrow time),
`HeapSequence::reserve` (caller capacity, not wire data) and the parameter
pool's `cap_n` (compile-time capacity) are clean.

What this does NOT do: the native lane still cannot run the macro path with a
32-bit `size_t`; the compile-time probe the issue asked for is the `#if
SIZE_MAX == 0xFFFFFFFFu` branch of the same test, which reports which path it
exercised rather than claiming coverage it does not have.
