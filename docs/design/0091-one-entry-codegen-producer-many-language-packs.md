---
rfc: 0091
title: "One codegen producer, many language packs — target-agnostic output, pinned representations"
status: Draft
since: 2026-09
last-reviewed: 2026-09
implements-tracked-by: [phase-432]
supersedes: []
superseded-by: null
---

# RFC-0091 — One codegen producer, many language packs

**Amends:** [RFC-0068](0068-language-neutral-codegen-ir.md) — retires its
`TargetProfile` (measured inert, see §5) and extends its four-stage shape to the
ENTRY pipeline, which it never scoped.

> **Goal.** Adding a target language is *adding a pack*. Exactly one path
> produces any given artifact. Generated code is target-AGNOSTIC: the compiler
> resolves the target, and where two languages must agree on a representation,
> the representation is PINNED in the source rather than modelled by codegen.

Issue **#1102** is the debt this pays down. Issue **#1003** is what the current
shape cost.

---

## 1. The problem, measured

Entry code has **four** producers, not the two issue 1003 assumed:

| # | Producer | Size | Emits |
| --- | --- | --- | --- |
| 1 | CMake templates (`nano_ros_add_node`) | 6 `.cpp.in` | the C/C++ entry TU |
| 2 | CLI emitters (`nros codegen entry`) | 3748 lines | C++, C, Rust entry TUs |
| 3 | `nros::main!()` proc-macro | **3822 lines** | the Rust entry |
| 4 | `builder/entry.rs` | 766 lines | an entry *package* |

**(4) is the model for the fix, not a problem**: it emits a package whose body is
`nros::main!(...)` — it delegates rather than restating.

**(2) and (3) are two paths to one outcome.** `emit_rust.rs`'s doc says it exists
"for byte-level diffs against the proc-macro output". That diff does not exist —
no gate, and the only test naming both never mentions the emitter. Nothing
consumes it either: `nano_ros_entry` passes `--lang` as `c` or `cpp` only, and
Rust entries reach the proc-macro via `rust_cargo_application()`.

**The duplication is not sloppiness.** Issue **0083** / phase-262 removed the
macro's `nros-build` dependency because it pulled all of `nros-cli-core` —
planner, codegen, orchestration, a submodule — into every *user's* entry build.
**Dependency weight at user-build time is the binding constraint**, and any
design ignoring it gets rejected the same way. It is the same force that made
`rosidl-lower` a leaf crate.

### The language vocabulary — three concerns wearing one word

| Type | Variants | What it is |
| --- | --- | --- |
| `entry::Lang` | Rust, Cpp, C | the enumeration |
| `ComponentLanguage` | Rust, C, Cpp | the same set, SERIALIZED — a wire format |
| `generate::Lang` | + **All** | a clap argument; `All` is not a language |
| `ComponentLang` | Rust, **Other** | a binary predicate |
| `PayloadLang` | Rust, C | a narrowing — two emitters, `Cpp` invalid |

Collapsing all five would be wrong three ways: it forces `All` into a language,
makes `Other` and `Cpp` coexist meaninglessly, and couples an on-disk wire
format to internal enums. The real defect is that the **enumeration** is declared
twice and the narrowings re-declare rather than derive. Issue **#1062** is that
already shipped: two language readers disagreeing, "and the loser is a silent
`C`".

---

## 2. Shape

```
                      nros-lang            leaf · serde only
                     Language {Rust,C,Cpp}
                          |
     +--------------------+-----------------+------------------+
     v                    v                 v                  v
 nros-cli-core      rosidl-codegen     nros-macros      nros-entry-lower
 entry + system      message packs     the proc-macro    entry Stage 2
```

Two pipelines, one architecture, one vocabulary:

```
  MESSAGES                             ENTRY
  .msg/.srv/.action                    launch.xml + system.toml + Cargo metadata
    |  PARSE   rosidl-parser             |  RESOLVE  plan_from_model
    v                                    v           nros-launch-parser
  Ast                                  Plan
    |  RESOLVE rosidl-resolve            |
    v                                    |
  ResolvedType                           |
    |  LOWER   rosidl-lower              |  LOWER    nros-entry-lower
    v          (no TargetProfile)        v           (no target facts)
  LoweredType                          LoweredEntry
    |  RENDER  packs/<lang>/             |  RENDER   packs/entry/<lang>/
    v                                    v
  generated messages                   generated entry
    |                                    |
    +----------> goldens / size corpus <-+   byte-compared every run
```

Both LOWER stages are **target-agnostic** (§5). Both RENDER stages are packs of
templates. `Language` selects the pack; it is read nowhere else in lowering.

---

## 3. Stage -1 — one `Language`

`Language { Rust, C, Cpp }`, declared once, carrying the
`serde(rename_all = "snake_case")` representation `ComponentLanguage` already
writes — so on-disk source metadata does not change. That repr is a
compatibility surface and wants a **gate**, not care.

Placement is forced, not chosen: `rosidl-codegen` does not depend on
`nros-pkg-index`, and the proc-macro depends on neither `nros-orchestration-ir`
nor `nros-cli-core`. The only home every consumer can afford is a leaf whose
dependency list is `serde` and nothing else.

The other three types stay and stop being independent: `All` becomes a CLI
affordance over `Language`; `ComponentLang` becomes a comparison; `PayloadLang`
stays a genuine narrowing and **says** it is one.

> **Rule: one enumeration, many narrowings — and a narrowing DERIVES rather than
> re-spells.**

---

## 4. Stage 2 — `nros-entry-lower`

All *computation*, inside the proc-macro's dependency budget
(`nros-pkg-index`, `nros-launch-parser`, `nros-orchestration-ir`): boot shape,
board path, tier rows, QoS codes, boot config, and **escaping**. If this crate
grows a heavy dependency the proc-macro cannot adopt it and the duplication
returns — so the budget is a design constraint, not a preference.

Escaping in particular stays here: a template that quoted a string wrong emits
code that compiles into the wrong behaviour, silently, on a target. Templates
place values; they never compute one.

---

## 5. Target facts — none, and representations are pinned

**Decision: retire `TargetProfile`.**

RFC-0068 introduced `TargetProfile { ptr_width, enum_width }`. Measured in the
tree:

- one consumer outside its own crate, passing `TargetProfile::host()`
  **unconditionally** — no caller ever supplies an embedded profile;
- `enum_width` read **nowhere**;
- `ptr_width` read once as "a conservative stand-in" on a path whose own comment
  says the result never changes an outcome.

It is inert, which is why passing `host()` for an ARM build has never broken
anything. The hazard it was built for cannot occur in the current emitters
either: `enum_width` guards the short-enums ABI (a `repr(C)` enum is 1 byte on
armv7a-nuttx, 4 on x86_64), and the C and Rust message packs emit **no `enum`
at all**.

**What replaces it.** Generated code is target-agnostic; the compiler resolves
the target. Where two languages must agree on a representation, the source
**pins** it — `uint8_t` against `#[repr(u8)]`, widths written down — rather than
codegen modelling the target's default ABI. A pinned representation is
target-agnostic by construction; a profile is a model of the toolchain, and a
model can be wrong silently.

**The agreement is gated, not assumed.** The precedent is the sizes-header
mirror (issues 0088 -> 0268): one side is authoritative and the other consumes,
rather than both guessing. Its lesson is also its warning — a stale mirror was
"silent memory corruption", 336 bytes short on freertos C. So: for a corpus of
generated types, compile the C and the Rust for the SAME non-host target and
compare `sizeof` against `size_of`. A representation that agrees only on the
host is exactly what this design must not ship.

---

## 6. Stage 3 — language packs

`packs/<language>/*.jinja` (messages) and `packs/entry/<language>/` (entry),
registered in a `PACKS`-style table of `include_str!` rows. This mirrors the
message side deliberately rather than inventing a second convention — that
file's own comment already states the goal: *"Adding a language = adding rows
here plus its `.jinja` files — no other Rust."*

Two entry renderers exist today (`rust_entry.rs.jinja`, `c_entry.c.jinja` +
`c_service_trailer.c.jinja`) with 20 goldens proving byte-equivalence with the
emitters they replaced. They should move from `templates/` to
`packs/entry/<lang>/` so there is one convention.

**The tier table must use DESIGNATED initialisers.** It is emitted positionally
against `nros_native_tier_spec_t`, a struct mirrored across **nine** files whose
own comment asks a human to keep them in sync — and `check-ffi-struct-mirrors`
does not cover it (it compares `component.h` against `nros_cpp_ffi.h`). A field
inserted anywhere but the end silently mis-assigns every generated entry:
issue 0160's class, ungated. `{ .name = ..., .groups = ... }` makes drift a
compile error at the generated TU; extending the mirror gate makes it loud at
the point of edit. Both.

### The C pack is PURE C

Today an embedded C entry routes to the C++ emitter and compiles as a `.cpp`
TU. That costs a C-only shop — MISRA, a certified C compiler, no C++ runtime —
a C++ toolchain to build an entry whose components are all C.

The reason is an accident of which entry points were declared, not a design
decision. The C-callable board surface:

| board | `run_components` | `run_tiers` |
| --- | --- | --- |
| native | yes (+`_named`) | yes |
| freertos / zephyr / nuttx | — | **yes** |
| threadx | — | — |

And the layering runs opposite to the dispatch's justification:
`nros_board_freertos_run_tiers` is **666 lines of C**, and
`FreertosBoard::run_tiers` is the C++ veneer calling it. The board layer is
already C at the bottom.

So exposing `nros_board_<rtos>_run_components` beside the existing `run_tiers`
is not new capability — it is the single-tier shape declared the way the
multi-tier one already is. That deletes the language-crossing branch from
`cmd/codegen.rs` and lets the C pack serve every board the C++ one does.
**ThreadX has no C board API at all and is declared C++-entry-only**, rather
than silently routed.

---

## 7. One path per outcome

**Delete `emit_rust.rs`.** No consumer; the CLI's Rust story is
`builder/entry.rs` emitting `nros::main!(...)`.

**The proc-macro consumes `LoweredEntry`**, so it stops re-deriving tiers (91
references), QoS (23) and board paths (6). It keeps `quote!` rendering behind a
**parity gate** that renders a corpus both ways and compares — the diff
`emit_rust` promised and never delivered.

Rendering through the pack instead (parse text -> `TokenStream`) was considered
and rejected: it compiles `minijinja` into every user's entry build, which is
the dependency weight that created this duplication once already.

**The CMake templates become a pack.** Phase-416 already collapsed six into one
parameterised RTOS template; `check-entry-session-name` gates both producers for
the fact that drifted. Until they are a pack, that gate is what holds them
together.

---

## 8. Adding a language — the whole procedure

1. `packs/entry/<lang>/entry.<ext>.jinja` — over `LoweredEntry`. Boot shape,
   board path, tier rows, QoS codes and escaped literals arrive computed.
2. One row in the pack registry (`include_str!`).
3. One variant on `Language`. Every consumer sees it — one enumeration.
4. Goldens: add the coordinate, `NROS_UPDATE_GOLDEN=1`, **read the diff**. The
   generated source is a file, not a claim.
5. If the language shares a representation with another (a C ABI struct, a
   message layout), add it to the cross-language size corpus in §5.

No lowering code, no emitter, no dispatch arms.

**What this does NOT make cheap, stated plainly:** the toolchain story — how
CMake compiles it, how it links `libnros`, how its components declare
themselves. The codegen cost becomes a pack. The build-integration cost does
not.

---

## 9. Acceptance

- `Language` declared once in a serde-only leaf; the `snake_case` wire repr of
  the metadata it replaces is unchanged, and gated.
- No type re-spells the enumeration; a narrowing derives and says so.
- `nros-entry-lower` exists inside the proc-macro's dependency budget;
  `LoweredEntry` is `serde`-serializable and carries **no target facts**.
- `TargetProfile` is gone from `rosidl-lower`, and nothing replaces it.
- A gate compiles the generated C and Rust for one non-host target and compares
  `sizeof` against `size_of` over a type corpus.
- The proc-macro and the CLI share lowering; a gate compares the two Rust
  renderings.
- The tier table uses designated initialisers, and the mirror gate covers
  `nros_native_tier_spec_t` across its nine sites.
- Adding a language touches no Rust beyond one `Language` variant.
- Goldens stay byte-stable across the migration — they are how each step is
  proven, and they already caught a rebase-introduced field change and three
  whitespace faults review missed.

---

## 10. Non-goals

- Changing what the entry DOES. This is about who produces it.
- Merging the entry and message pipelines. They share an architecture, not a
  pipeline: the inputs and the lowered facts have nothing in common.
- Removing `nano_ros_add_node`. It is a supported surface; it becomes a pack.

## 11. Open questions

- Retiring `TargetProfile` amends RFC-0068, which is **Stable**. Does that edit
  land there or here? Recorded as an amendment above; the mechanics are a
  maintainer call.
- `nros_board_threadx_run_components` does not exist and neither does its
  `run_tiers` — is pure-C ThreadX worth the shim, or is the declaration enough?
- Does the designated-initialiser change land with the C++ pack conversion or
  ahead of it? It is byte-visible in every generated entry, so it wants its own
  goldens diff.
- Does the mirror gate extend (`check-ffi-struct-mirrors` assumes TWO files) or
  get a sibling keyed on a struct with nine?

## 12. Known gap in the current goldens

Five C rows (`c_nuttx_one`, `c_zephyr_one`, `c_freertos_one`, `c_threadx_one`,
`c_nuttx_tiers`) come from calling `emit_c` directly, which the pipeline never
does for those boards — an embedded C entry routes to the C++ emitter. They are
byte-identical to the native rows except a comment and read as board coverage
they do not have. Route the harness through the dispatch, or relabel them to pin
what they actually prove. Tracked on issue 1102.
