# Codegen — the pack pipeline

nano-ros generates ROS 2 message/service/action types (and their per-package
scaffolding) from `.msg`/`.srv`/`.action` files. The generator is a four-stage
pipeline (RFC-0068): **parse → resolve → lower → render**. This page covers the
last stage — **render** — and how to change or add a target language.

## Render = a data pack + a runtime template

Every backend renders through one `minijinja` environment
(`packages/cli/rosidl-codegen/src/render.rs`) over **data packs** under
`packages/cli/rosidl-codegen/packs/`:

| pack | output |
| --- | --- |
| `packs/c/` | C headers + sources |
| `packs/rmw/` | RRR-compatible Rust message layer |
| `packs/rust/` | idiomatic (rclrs-style) Rust |
| `packs/nros/` | embedded (`no_std`) Rust |
| `packs/cpp/` | C++ headers + the Rust FFI glue |
| `packs/scaffold/` | per-package `Cargo.toml` / `lib.rs` / `build.rs` |

A pack is just `.jinja` templates. The Rust side hands each template a
**`serde`-serialized view struct** (the render context) and never spells a type
itself — the type strings are composed **in the pack** by registered filters:

- `c_type` / `c_array_suffix`, `cpp_type` / `cpp_array_suffix`,
  `cpp_repr_c_type` / `cpp_view_repr_type`
- `rust_type_rmw` / `rust_type_idiomatic`, `nros_type`
- `snake_case`

The view struct carries only **neutral facts** (the parsed `field_type`, resolved
capacity, storage mode, `current_package`, …); the filter maps those to the
language's syntax. This is RFC-0068's "what vs how" seam: *what* a type is lives in
the IR; *how* a language spells it lives in the pack + its filter.

## Changing a template

Edit the `.jinja` under `packs/<lang>/` and rebuild the CLI (`just setup-cli`).
The codegen **fingerprint** (RFC-0061) hashes every bundled pack's content, so any
template edit marks the affected fixtures stale — no separate bookkeeping.

## Overriding a pack at runtime — no rebuild

Point the renderer at an external directory of `.jinja` files: a file named
`<template-name>` or `<template-name>.jinja` there **overrides** the bundled pack
of that name; anything absent falls back to bundled.

```sh
export NROS_TEMPLATE_DIR=/path/to/my/pack
nros generate-rust …          # uses the override, no recompile
```

(Equivalently, `rosidl_codegen::render::set_template_dir(dir)` from Rust, called
once before the first render.)

The stable template names are the keys of `PACKS` in `render.rs` (e.g.
`message.h`, `message_nros.rs`, `cargo.toml`, `_field.jinja`). `tests/
external_pack_smoke.rs` proves the override + fallback.

> **Do not set `NROS_TEMPLATE_DIR` during fixture or CI builds.** The fingerprint
> hashes the *bundled* packs; an external override would silently produce output
> that disagrees with the recorded fingerprint.

## Adding a language

### Step 0 — decide which SURFACES you need

This is the step that sizes the work, and skipping it is why "add a language"
sounds bigger than it is. A pack is a **(language × surface)** pair, not a
language: Rust has four message packs, and the `cpp` pack emits Rust as well as
C++.

There are two independent axes. On the **message** side: idiomatic,
embedded-idiomatic (`no_std`), FFI/ABI (`repr(C)`), bridge glue, packaging. On
the **entry** side: the entry TU, and the component seam. A language can take
an entry surface with no message surface (it consumes another language's
messages) or the reverse.

RFC-0091 §6 has the full table, including what each surface *drags in* — an
FFI surface inherits the cross-language memory-agreement gate, a bridge surface
is two spellings of one type that must move together. **A language can ship
with two surfaces and gain the rest later**: a Zig component installed by a C
or C++ entry needs the idiomatic and FFI message surfaces and the component
seam, and nothing else.

### Step 1 — the message pack

1. add its `.jinja` templates (a new `packs/<lang>/`) and its rows in `PACKS`;
2. if it needs type spelling the existing filters don't cover, add a **filter
   set** for the language (`rosidl_codegen::filters::FILTER_SETS`) wrapping a
   `*_spelling` function in `types.rs`. A language's Rust surface area is a
   pack plus a filter set and nothing more; the set is keyed by the pack that
   CALLS the filter, not by the syntax it emits — `cpp_repr_c_type` returns
   Rust;
3. add the generator entry that builds the view struct and calls
   `render::render("<template-name>", &ctx)`.

### Step 2 — the entry pack, if the language writes entries

1. `packs/entry/<surface>/entry.<ext>.jinja` over the entry view. Board paths,
   boot shape, tier rows, QoS codes and escaped literals all arrive already
   computed — a template decides where a value goes, never how to quote one;
2. `packs/entry/<surface>/pack.toml` — the extension, whether the TU is
   C-family, the entry template and its partials. CMake reads this through
   `nros codegen entry-pack` rather than deriving it, so this file is what
   makes the build know how to name and compile the output;
3. one row in the template registry (`render.rs`), keyed
   `<artifact>_<surface>.<ext>` for an output and `*.jinja` for a partial;
4. one variant on `Language` in `nros-lang`. Every consumer sees it — one
   enumeration.

### Step 3 — the goldens

Add the coordinate to the entry golden harness, run
`NROS_UPDATE_GOLDEN=1 cargo test -p nros-cli-core --lib codegen::entry::golden`,
and **read the diff**. The generated source is a file, not a claim.

`just check entry-pack-conformance` refuses a half-wired pack: a directory with
no manifest, a language pack missing the fields CMake needs, a `Language`
variant nothing renders, or a language whose bytes are recorded in no golden.
Run it before you believe the pack works.

### What this does NOT make cheap

The **toolchain story** — how CMake compiles the language, how it links
`libnros`, how its components declare themselves. The codegen cost becomes a
pack; the build-integration cost does not. Budget for it separately.

No per-language type logic lives in the builders — the packs and their filters
own it. Implemented by phase-335 (RFC-0068) and phase-432 (RFC-0091).
