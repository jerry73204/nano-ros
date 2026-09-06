---
id: 1170
title: "Issue 1162's fix deleted the duplicate definition and left its three callers on the old signature, so `nros-c` still did not compile"
status: wontfix
type: bug
area: api, build
severity: high
found: 2026-09-06
resolved: 2026-09-06
related: [1162, 1169, 0417, 0196]
---

> **DUPLICATE of issue 1169 — retracted 2026-09-07.** I diagnosed and fixed this
> independently while `30fb71de7 fix(#1169, phase-413 W2): the stale nros-c test
> call sites` was landing on main. Same defect, same three call sites, filed
> minutes apart. 1169 has the number; everything below is kept because the
> diagnosis stands and because the two fixes differ in one way worth recording.
>
> **1169 passes `core::ptr::null_mut()` at all three sites.** That compiles, and
> asserts nothing about the parameter the two definitions had disagreed about.
> The `out_len` coverage described below was re-applied on top of 1169's fix
> rather than dropped with the duplicate: the composition test now checks
> `out_len == expected.len()` for all five namespace spellings, the short-buffer
> test checks that `NROS_RET_FULL` STILL reports the size a caller needs
> (mutation-verified — moving the write after the length check fails it), and
> one NULL call is kept so that branch stays covered.

# The duplicate is gone and the crate still does not build

Issue 1162 — *"`nros_node_get_fully_qualified_name` was defined TWICE with
different signatures — main's C API could not compile, and no merge-gating lane
noticed"* — is marked `resolved`. Measured on a clean `origin/main` worktree at
`fdc0604c5`, with no branch of mine involved:

```
$ cargo test --no-run -p nros-c --lib --features rmw-cffi --locked
error[E0061]: this function takes 4 arguments but 3 arguments were supplied
error[E0061]: this function takes 4 arguments but 3 arguments were supplied
error[E0061]: this function takes 4 arguments but 3 arguments were supplied
error: could not compile `nros-c` (lib test) due to 3 previous errors
```

The acceptance that issue's title states — main's C API compiles — is still
false. What changed is which half is wrong.

## What 1162 fixed, and what it did not

`add92bfcf` kept the **right** definition. The C header is the ABI SSoT
(RFC-0054), and `nros_generated.h:5740` declares four parameters:

```c
nros_ret_t nros_node_get_fully_qualified_name(const struct nros_node_t *node,
                                              char *buf,
                                              size_t buf_len,
                                              size_t *out_len);
```

It also added a compile probe pinning that signature through a function pointer
(`tests/compile/node_timer_accessors.c:58`), whose own comment names this issue's
parent. Both halves are correct.

What it did not do is follow the deletion into the **callers**. Three sites in
the crate's own `#[cfg(test)]` module still passed three arguments —
`node.rs:1511`, `:1533`, `:1546`. They compiled before only because the OTHER
definition, the one that was deleted, took three.

This is issue 0196's rule and CLAUDE.md's "fix the CLASS, then prove the sweep",
in the narrowest possible form: the sweep for a deleted signature is every
caller of it, and here that was one `grep` in one file. The reason it survived
is the reason 1162 gives for its own invisibility — `check-build` is the only
lane that compiles this, and no merge-gating event runs it — so a fix verified
by anything short of building the crate reads as complete.

## Fixed

The three callers now pass `out_len`, and they ASSERT it rather than merely
satisfying the compiler. That parameter is the one the two definitions disagreed
about, so it is the one that needed coverage and had none:

* the composition test checks `out_len == expected.len()` for all five
  namespace spellings — the length excludes the terminator;
* the short-buffer test checks that `NROS_RET_FULL` **still reports the size**,
  which is the whole point of the out-parameter: a caller that gets FULL learns
  how big a buffer to bring back. Mutation-verified — moving the write after the
  length check fails with *"NROS_RET_FULL must still report the length the
  caller needs"*;
* a third call passes `NULL` for `out_len`. The implementation branches on it
  and the documentation calls it optional; that branch is reachable from C and
  had no test.

Acceptance:

```
cargo test -p nros-c --lib --features rmw-cffi   103 passed
just check required-features-tests               rc=0
```

The second is the lane this blocked. It had been red through two causes in
sequence — issue 1168's timer flake, and then this — and this is the first run
of it that reaches `required-features test targets passed!`.
