---
id: 1181
title: "`ZPICO_SUBSCRIBER_BUFFER_SIZE` is documented in four places and read by nothing"
status: open
type: bug
area: docs, rmw-zenoh, memory
severity: low
found: 2026-09-07
related: [1125, 0940]
---

## What

The knob that sizes the zenoh `small` payload class is
**`NROS_SUBSCRIBER_BUFFER_SIZE`**. `packages/rmw/zenoh/nros-rmw-zenoh/build.rs`
reads exactly that name:

```rust
let sub_size: usize = env_usize("NROS_SUBSCRIBER_BUFFER_SIZE", 1024);
```

and `zephyr/cmake/nros_cargo_build.cmake` resolves it under that spelling, with
a comment recording WHY it moved:

> phase-403 -- ONE name, the backend-agnostic one, which is also what the
> Kconfig symbol has always been called. This resolved under
> `ZPICO_SUBSCRIBER_BUFFER_SIZE`, and the mismatch was not cosmetic: it is what
> let a live delivery bug hide.

The old `ZPICO_`-prefixed name survives in the docs and in `.env.example`:

```
$ grep -rln ZPICO_SUBSCRIBER_BUFFER_SIZE --exclude-dir=docs/issues .
.env.example
book/src/reference/environment-variables.md
docs/guides/embedded-tuning.md
docs/design/0038-zero-copy-data-transport.md
```

`docs/guides/embedded-tuning.md` gives it in **five** copy-pasteable command
lines, including the memory-tuning recipes for the 256 KB-class boards, and
`environment-variables.md` lists it as a table row with a default. None of them
does anything: a user who exports it gets the crate default of 1024 and no
diagnostic, which is the shape issue 0940 records — a human tuning a knob by
reading a document and the image not moving.

## Not measured beyond the grep

Found while writing issue 1125's `[env]` sidecar, which emits the LIVE name. I
did not build an image with the stale name exported to confirm it is inert; the
build script's own source is the evidence, and there is no `env_usize` or
`rerun-if-env-changed` for the `ZPICO_` spelling anywhere in the tree.

## What closing it looks like

Rename in the four files, or — better, since a document cannot be gated —
teach `config-knob-census` (or a sibling) that a knob NAME appearing in
`book/`, `docs/guides/` or `.env.example` must be read by some build script.
That is the general form: this is one instance, and phase-403 moved several
names.

Note `ZPICO_SUBSCRIBER_LARGE_SIZE` and `ZPICO_SUBSCRIBER_SIZE_THRESHOLD` are
NOT this bug — those two kept their `ZPICO_` names and are read.
