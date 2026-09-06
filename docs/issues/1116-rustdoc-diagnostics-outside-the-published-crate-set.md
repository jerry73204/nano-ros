---
id: 1116
title: "~70 rustdoc diagnostics in crates the book does not publish — five of them fail to document at all"
status: open
type: tech-debt
area: docs, core
severity: low
found: 2026-09-06
related: [1110, 0319, 0896]
---

# What issue 1110 fixed, and what it deliberately did not

Issue 1110 put rustdoc on a lane a pull request runs (`just check rustdoc-links`, in the `compile-smoke`
job). Its scope is the **deployed** crate set — the six crates `just book`
publishes — because that is what keeps the docs deploy green.

A workspace-wide pass is a different story. Measured on a cold pass,
2026-09-06:

```
$ cargo doc --no-deps --workspace $HOST_UNCHECKABLE
… ~70 diagnostics …
error: could not document `nros-node`
error: could not document `nros-orchestration-ir`
error: could not document `nros-platform`
error: could not document `nros-serdes`
error: could not document `zpico-alloc`
```

Five crates do not document at all. The diagnostics fall into three kinds, and
they need different fixes:

1. **A link to an item that no longer exists** — the 1110 shape, e.g.
   `crate::Node::session_mut` (3×), `crate::Executor::register_lifecycle_node`
   (2×), `Executor::open_multi`. Each is a rename or deletion whose prose was
   not moved with it.
2. **A public item linking to a PRIVATE one** — ~14 of them
   (`ExecutorSizing` → `carve`, `loan` → `TxArena::release`, `dispatch_callback`
   → `DispatchSlot`). rustdoc rejects these because a reader of the public page
   cannot follow the link. The fix is prose or `pub(crate)` visibility, not a
   link.
3. **Markdown that reads as a link and is not one** — `[iu]{8,16,32,64}` in
   `nros-serdes::schema`, `[must_use]`, `[planned]`. Backticks, not brackets.

## Why this is not urgent, and not nothing

Nothing publishes these crates, so no deploy is broken by them and no user sees
a 404. What they cost is the ability to WIDEN the gate: `rustdoc-links` cannot
grow to the workspace while 70 diagnostics stand, so every crate outside the
published six keeps the property 1110 was about — a doc link can rot with
nothing to say so.

## Doing it

A ratchet is the wrong tool here (the count only goes down, and the fixes are
mechanical). Better: fix by kind, in three commits, then widen
`NROS_RUSTDOC_CRATES` in `scripts/build/rustdoc-set.sh` to the workspace and
delete this issue's reason for existing. The gate is ~3 s warm on six crates and
was measured at 3.2 s warm on the whole workspace, so cost is not the obstacle.
