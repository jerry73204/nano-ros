---
id: 1110
title: "`just book` and the docs deploy have been red since 2026-09-03 — a doc link to a method that commit deleted"
status: open
type: bug
area: docs, core
severity: medium
found: 2026-09-06
related: [1008, 1076]
---

# The link outlived the method it points at

```
$ just book
error: unresolved link to `Self::is_server_ready`
    --> packages/core/nros-rmw/src/traits.rs:2837:29
     |
2837 |     /// [`is_server_ready`](Self::is_server_ready), which collapses
     |                             ^^^^^^^^^^^^^^^^^^^^^ the trait `ClientTrait` has no associated item named `is_server_ready`

error: could not document `nros-rmw`
error: recipe `book` failed with exit code 101
```

`3941b569a` (*"fix(#1008): `wait_for_service` never waited — W6 decision 2
deletes the collapse"*, 2026-09-03) removed `is_server_ready` from
`ClientTrait`. The paragraph **beside** it still contrasts `server_available()`
with that method, by intra-doc link:

```rust
/// "don't know" and "no server" into the same `false` answer.
/// Distinct from [`is_server_ready`](Self::is_server_ready), which collapses
```

`grep -c 'fn is_server_ready' packages/core/nros-rmw/src/traits.rs` → **0**.

## Why it went unnoticed for three days

rustdoc's broken-link lint is deny-level in this configuration, so this is an
`error`, not a warning — but nothing on the merge-gating lanes runs rustdoc.
`just book` does, and it is not in `check-fast`, `ci gate`, or the required `CI`
context. `docs.yml` runs on `push` to `main` under a path filter that
**includes** `packages/core/nros-rmw/**`, so the docs deploy has been failing
since that commit landed.

Same class as issue 0319 (a backend suite nothing on the `just check` line ran)
and 0896 (`check-cli-tests` living only in a lane no merge-gating event runs):
the check exists, is correct, and is not on a path anything traverses.

## The fix, and the fix for the class

1. The paragraph. `server_available()` is now the only spelling, so the contrast
   has no other side — the sentence should be rewritten to state what
   `server_available()` returns rather than what the deleted method collapsed.
   Deleting the link alone leaves a dangling "Distinct from" clause.

2. The class. Deleting a method must fail somewhere that runs on a PR. Either
   `just book` joins a lane, or a cheap `cargo doc --no-deps` over the core
   crates does — the full book build is expensive (it also runs Doxygen and the
   rustdoc driver), and the value here is the link check, not the HTML.

Found while verifying a docs change for phase-431 W6: the book built fine
(`mdbook build` alone is green), and `just book` was red before the change and
after it.
