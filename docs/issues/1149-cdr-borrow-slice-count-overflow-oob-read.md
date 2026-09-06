---
id: 1149
title: "`nros_cdr_borrow_le_slice_*` multiplies a wire-supplied count by element size without overflow check — on every 32-bit target that is an unbounded read"
status: open
type: bug
area: c, serdes, security
related: [phase-428, issue-1148]
---

## Problem

`packages/api/nros-c/include/nros/view.h:98`:

```c
size_t bytelen = (size_t)cnt * sizeof(CT);
```

`cnt` comes off the wire. On `arm-none-eabi` — measured, `__SIZE_MAX__` is
`0xffffffffU` — `cnt = 0x40000000` with `CT = float` wraps `bytelen` to 0. The
bounds check at `:99` then passes for any buffer, `:101` stores the
UN-overflowed `count`, and the `_get` accessor has no bounds check at all.

**Every embedded target this project ships for is 32-bit**, and the defect is
structurally invisible to the native lane, where `size_t` is 64 bits.

Rust's counterpart uses `checked_mul` (`nros-serdes/src/cdr.rs:831`).

## Impact

Untrusted wire data produces an out-of-bounds read, reported as success. This
is the RMW receive path's input.

## Fix

Reject when `cnt > SIZE_MAX / sizeof(CT)` before multiplying; bounds-check
`_get`. Add a compile-time probe for 32-bit `size_t` — the native lane cannot
exercise this and must not be read as covering it.
