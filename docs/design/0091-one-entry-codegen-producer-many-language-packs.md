---
rfc: 0091
title: "One entry-codegen producer, many language packs — and no two paths to the same entry"
status: Draft
since: 2026-09
last-reviewed: 2026-09
implements-tracked-by: [phase-432]
supersedes: []
superseded-by: null
---

# RFC-0091 — One entry-codegen producer, many language packs

> **Relationship to other RFCs.** This is [RFC-0068](0068-language-neutral-codegen-ir.md)'s
> architecture applied to the OTHER codegen pipeline. RFC-0068 covers messages
> (`.msg/.srv/.action` → Rust/C/C++/IDL) and is Stable; it never scoped the ENTRY
> (launch + `system.toml` → the program that boots). Entry generation grew its own
> shape, and this records what that shape should be. Issue **#1102** is the debt
> this pays down; issue **#1003** is what the current shape cost.

**Goal.** Adding a target language to ENTRY codegen becomes *adding a pack*, never a
Rust change — and exactly one path produces any given entry.

## The problem, measured

Entry code has **four** producers, not the two issue 1003 assumed:

| # | Producer | Size | Emits | Input |
| --- | --- | --- | --- | --- |
| 1 | CMake templates (`nano_ros_add_node`) | 6 `.cpp.in` | the C/C++ entry TU | cmake vars |
| 2 | CLI emitters (`nros codegen entry`) | 3748 lines | C++, C, Rust entry TUs | `Plan` |
| 3 | `nros::main!()` proc-macro | **3822 lines** | the Rust entry | launch + `system.toml` |
| 4 | `builder/entry.rs` | 766 lines | an entry *package* | build plan |

Two facts about that table matter more than its size.

**(4) is not a problem and is the model for the fix.** It emits a package whose body is
`nros::main!(...)` — it *delegates* rather than restating. Its own header asks "Why
`nros::main!` and not the expanded form".

**(2) and (3) are two paths to the same outcome.** Both take launch input and produce a
Rust entry. `emit_rust.rs`'s doc says it exists "for byte-level diffs against the
proc-macro output". **That diff does not exist** — no test, no gate; the only test naming
both (`native_main_macro_forms.rs`, 46 lines) never mentions the emitter. So the second
producer's stated justification is a comparison nobody runs. That is issue 1003's shape
exactly — two spellings of one fact with nothing comparing them — over a ~7500-line
surface.

And **nothing consumes the Rust one**: `nano_ros_entry` passes `--lang` as `c` or `cpp`
only; Rust entries reach the proc-macro through `rust_cargo_application()`. The only
callers of `emit_rust::emit` are the CLI verb dispatch and a golden test.

### The language vocabulary is fragmented — but NOT five copies of one enum

A first reading of this counted "six language enums" and proposed collapsing
them. Reading what each one MEANS says otherwise, and the difference decides
the design:

| Type | Variants | What it actually is |
| --- | --- | --- |
| `codegen/entry/mod.rs::Lang` | Rust, Cpp, C | the language enumeration |
| `orchestration/source_metadata.rs::ComponentLanguage` | Rust, C, Cpp | the same set, but SERIALIZED (serde, snake_case) — a wire format |
| `cmd/generate.rs::Lang` | Rust, C, Cpp, **All** | a clap argument; `All` is a CLI affordance, not a language |
| `cmd/codegen_system.rs::ComponentLang` | Rust, **Other** | a binary predicate ("is this component Rust?") |
| `rosidl-codegen::PayloadLang` | Rust, C | which of TWO emitters is asking, for a storage check |

So there are **three concerns wearing one word**: the enumeration (twice), an
argument type that adds a non-language variant, and two narrowings. Collapsing
all five into one type would be wrong in three separate ways — it would force
`All` into a language, make `Other` and `Cpp` coexist meaninglessly, and couple
an on-disk wire format to internal enums.

What IS a defect is that the enumeration is declared twice, and that the
narrowings re-declare rather than derive. A language added to one is invisible
to the others, and nothing says they are related. Issue **#1062** is that
failure already shipped: "`nano_ros_add_node` has two language readers that
disagree — cmake expands SOURCES, the scanner reads the raw text, and the loser
is a silent `C`".

### Why the duplication exists (it is not sloppiness)

The proc-macro deliberately does NOT depend on `nros-cli-core`. Issue **0083** /
phase-262 removed its `nros-build` dependency because that pulled the whole planner,
codegen and orchestration tree — plus a submodule — into every user's entry build. It now
depends only on `nros-pkg-index` and `nros-launch-parser`, "the two leaf crates the macro
needs".

So the constraint is **dependency weight at user-build time**, and any design that says
"just share the code" without respecting it will be rejected the same way. This is the
same force that produced `rosidl-lower` as a separate leaf crate for messages.

## Design

RFC-0068's four stages, applied to the entry. The stages already exist in the entry path;
they are simply implicit, and each renderer re-derives them.

```
 launch.xml + system.toml + Cargo metadata
      │  STAGE 0/1  RESOLVE      (exists: plan_from_model, nros-launch-parser)
      ▼
   Plan                                    board-neutral · serde
      │  STAGE 2  LOWER   ( ⊗ BoardProfile )        ←  NEW leaf crate
      ▼
   LoweredEntry = per-node and per-tier facts made CONCRETE:
        boot_shape   : KernelMain | AppMain | HostMain
        board_path   : resolved through nros-orchestration-ir
        tiers        : spec rows, groups arrays, per-tier membership
        boot_config  : the baked blob
        literals     : every string ALREADY ESCAPED for its target language
      │                                    LANGUAGE-neutral, BOARD-specific · serde
      │  STAGE 3  RENDER  ( per language, DATA-driven )
      ▼
   generated C++ / C / Rust / <new language>
```

### Stage -1 — one `Language`, in a crate everyone can afford

```
                       nros-lang          (leaf: serde only)
                          │
        ┌─────────────────┼──────────────────┬────────────────────┐
        ▼                 ▼                  ▼                    ▼
  nros-cli-core     rosidl-codegen      nros-macros        nros-entry-lower
  entry + system     message packs      the proc-macro      Stage 2 facts
```

`Language { Rust, C, Cpp }` — the enumeration, declared once, with the
`serde(rename_all = "snake_case")` representation `ComponentLanguage` already
writes, so the on-disk source metadata does not change. That repr is a
compatibility surface and wants a gate, not just care.

Placement is forced by the dependency budget, not by taste: `rosidl-codegen`
does not depend on `nros-pkg-index`, and the proc-macro depends on neither
`nros-orchestration-ir` nor `nros-cli-core` (issue 0083). The only home every
consumer can afford is a new leaf whose dependency list is `serde` and nothing
else.

The other three types stay, and stop being independent:

- `generate::Lang` becomes a clap argument over `Language` plus `All`, where
  `All` EXPANDS to the set. It is a CLI affordance and should read as one.
- `ComponentLang { Rust, Other }` becomes a question asked of `Language`, not a
  second enum: the call sites want "is this Rust", which is a comparison.
- `PayloadLang { Rust, C }` is a genuine NARROWING — only two emitters ask that
  storage question, and `Cpp` is not a valid answer. It keeps its own type, and
  documents that it is a subset rather than re-deriving the parent.

The rule this encodes: **one enumeration, many narrowings, and a narrowing must
be derived from the enumeration rather than re-spelled beside it.**

### Stage 2 — `nros-entry-lower` (new leaf crate)

Everything that is *computation*: boot shape, board resolution, tier lowering, QoS
override lowering, boot-config baking, and escaping. Dependencies limited to
`nros-pkg-index`, `nros-launch-parser`, `nros-orchestration-ir` — the set the proc-macro
already accepts. That bound is the design's load-bearing constraint, not an afterthought:
if `nros-entry-lower` grows a heavy dependency, the proc-macro cannot adopt it and the
duplication returns.

### Stage 3 — language packs

`packs/entry/<language>/` — templates over `LoweredEntry`, plus a manifest saying which
template renders which output. This deliberately mirrors the MESSAGE side's layout
(`rosidl-codegen/packs/<language>/*.jinja`, registered in a `PACKS` table of
`include_str!` rows) rather than inventing a second convention; that file's own comment
already states the goal — "Adding a language = adding rows here plus its `.jinja` files —
no other Rust". The entry renderer landed with a `TEMPLATES` table under `templates/`,
which should move to `packs/entry/<lang>/` so there is one convention, not two. **Per-language generators are expected and correct**; what
must not be per-language is the pipeline that feeds them.

Two of three renderers are already here: `templates/rust_entry.rs.jinja` and
`templates/c_entry.c.jinja` (+ `c_service_trailer.c.jinja`) render through a bundled
`minijinja` environment, with 20 goldens proving byte-equivalence with the emitters they
replaced. `emit_cpp.rs` is the remaining one, together with the three `pub(super)`
`emit_declare_*` helpers it shares with `emit_c`.

### The CMake templates are a pack, not a fifth producer

`cmake/templates/*_entry_main*` render the same artifact from cmake variables rather than
from `LoweredEntry`. Phase-416 already collapsed six of them into one parameterised RTOS
template, and `check-entry-session-name` gates both producers for the one fact that
drifted. They should become a pack rendered from `LoweredEntry` like any other; until
then the gate is what holds them together.

## Resolving the duplicate outcome

Two steps, independently valuable, in this order.

**Step 1 — delete `emit_rust.rs` (no consumer).** The CLI's Rust story is already
`builder/entry.rs`: emit a package containing `nros::main!(...)` and let the proc-macro
expand it. This removes a ~370-line path to an outcome another path already owns, and it
is verifiable today: nothing but the verb dispatch and a golden calls it.

The counter-argument is the byte-diff the doc claims. That diff was never implemented, so
deleting loses nothing that exists — and if the diff is wanted, it should be built as
Step 2's parity gate, which is strictly better than an unused emitter.

**Step 2 — the proc-macro consumes `LoweredEntry`.** It stops re-deriving tiers (91
references), QoS (23) and board paths (6), and becomes: lower (shared) → render. Two ways
to render, and the choice is a real trade:

- **(a) Same packs.** The macro renders text through the pack and parses it to a
  `TokenStream`. One renderer, no drift by construction. Cost: `minijinja` compiles into
  every user's entry build, and generated-code spans become synthetic.
- **(b) Keep `quote!`, add a parity gate.** The macro keeps its own backend but consumes
  the shared `LoweredEntry`, and a gate renders a corpus of plans both ways and compares.
  Cost: parity is by test rather than by construction — but it is the diff `emit_rust`
  promised and never delivered.

**Recommendation: (b).** The dependency-weight constraint is what created this
duplication once already; (a) reintroduces it in a new form. (b) removes the *derivation*
duplication — which is where correctness lives — and makes the remaining spelling
duplication checked rather than assumed.

## What stays in Rust

Unchanged from RFC-0068 and load-bearing here for the same reason: all computation.
Escaping in particular — a template that quoted a string wrong would emit code that
compiles into the wrong behaviour, silently, on a target. Templates place values; they
never compute one.

## Acceptance

- `nros-entry-lower` exists as a leaf crate within the proc-macro's dependency budget,
  and `LoweredEntry` is `serde`-serializable.
- The proc-macro and the CLI emitters both consume it; neither re-derives boot shape,
  board path, tier rows or QoS codes.
- A gate compares the two Rust renderings over a corpus of plans (the diff `emit_rust`
  promised).
- Adding a language to entry codegen touches no Rust beyond ONE `Language`
  variant: a pack directory, a manifest row, and that variant.
- `Language` is declared once, in a leaf crate whose only dependency is
  `serde`, and the `snake_case` wire representation of the metadata it replaces
  is unchanged — gated, because it is an on-disk compatibility surface.
- No type re-spells the enumeration. A narrowing (`PayloadLang`) derives from
  it and says so.
- The goldens (`testdata/entry/*.golden`) stay byte-stable across the whole migration —
  they are how each step is proven, and they already caught a rebase-introduced field
  change and three whitespace faults that review missed.

## Adding a language — the whole procedure

The measure of this design is what a new language costs. Zig, end to end:

1. **`packs/entry/zig/entry.zig.jinja`** — the entry TU shape, over
   `LoweredEntry`. Boot shape, board path, tier rows, QoS codes and escaped
   literals arrive already computed; the pack places them.
2. **One row** in the pack registry (`include_str!`), exactly as the C and Rust
   packs are registered today.
3. **One variant** on `Language` in `nros-lang`. Every consumer — the entry
   pipeline, source metadata, the CLI argument, the proc-macro — sees it,
   because there is one enumeration rather than five.
4. **Goldens**: add the coordinate to the harness, run
   `NROS_UPDATE_GOLDEN=1`, and READ the diff. That is the whole review: the
   generated source is a file, not a claim.

No lowering code, no emitter, no dispatch arms.

**What this does NOT make cheap, stated plainly.** A language also needs a
TOOLCHAIN story — how CMake compiles it, how it links against `libnros`, how
its components declare themselves. That is real work and this design does not
touch it. The claim here is narrow: the CODEGEN cost of a language becomes a
pack. The build-integration cost does not.

## Workflow, end to end

```
  launch.xml + system.toml + Cargo metadata
        │
        │  STAGE 0/1  RESOLVE        plan_from_model · nros-launch-parser
        ▼
     Plan                                              board-neutral · serde
        │
        │  STAGE 2  LOWER  ( ⊗ BoardProfile )          nros-entry-lower
        │            boot shape · board path · tier rows · QoS codes
        │            boot config · EVERY literal escaped
        ▼
     LoweredEntry                            language-neutral · board-specific
        │
        │  STAGE 3  RENDER  ( ⊗ Language )             packs/entry/<lang>/
        │            templates place values; they compute none
        ▼
   generated C++ / C / Rust / <new language>
        │
        └── goldens (testdata/entry/*.golden) — byte-compared every run

  Language (nros-lang) is read at Stage 3 to pick the pack, and by the CLI,
  source metadata and the proc-macro. One enumeration; the narrowings derive.

  The proc-macro enters at Stage 2, not Stage 0: it lowers with the same crate
  and renders with `quote!`, and a gate compares its output against the pack's.
```

## Non-goals

- Changing what the entry DOES. This is about who produces it.
- Merging entry and message codegen. They share an architecture, not a pipeline: the
  inputs (`.msg` vs launch + `system.toml`) and the lowered facts have nothing in common.
- Making the CMake path disappear. `nano_ros_add_node` is a supported surface; it becomes
  a pack, not a casualty.

## The two open questions, answered

Both were raised in the first draft and both have been studied against the
tree. One dissolves; the other reveals a larger ungated risk than the question
asked about.

### 1. Does `LoweredEntry` need a `TargetProfile`? — **No, and the reason matters**

`LoweredType` needs one because message layout IS target-specific (pointer
width, short enums, `repr(C)` order). The entry bakes **no layout fact**. It
emits a positional initialiser whose types the target compiler resolves:

```c
static const nros_native_tier_spec_t __nros_tiers[2] = {
    { "high", __nros_tier_0_groups, 1u, 80LL, 0u, 10000ull,
      &__nros_entry_setup_tier_0, 0u, -1LL, NULL, 0ull, 0ull, 0ull, NULL },
};
```

Every literal suffix is width-SAFE rather than width-ASSUMING: `1u` into a
`size_t n_groups`, `0u` into `size_t stack_bytes`, `80LL` into `int64_t
priority`, `10000ull` into `uint64_t spin_period_us`. No literal is wider than
its field and each converts by the usual arithmetic conversions, so the same
text compiles correctly on a 32- and a 64-bit target. There is nothing for a
`TargetProfile` to decide.

**But the study found the real hazard, which is not target width.** That
initialiser is POSITIONAL, and `nros_native_tier_spec_t` is mirrored across
**nine files** — the struct's own comment enumerates them: "ABI append-only,
keep every mirror in sync: main.hpp `NativeTierSpec`, nros-cpp
`NativeTierSpecC`, the 4 board `nros_tier_spec_t` mirrors, and both entry
emitters."

Insert a field anywhere but the end and every generated entry silently
mis-assigns from that point on — `stack_bytes` lands in `priority`, a function
pointer lands in a `uint64_t`. That is exactly the issue **0160** class, and
`check-ffi-struct-mirrors` does NOT cover it: that gate compares
`component.h` against `nros_cpp_ffi.h` and knows nothing about this struct.
A comment asking nine files to stay in sync, with no gate, is the shape
issue 0196 named — "gates whose coverage was narrower than the rule they
enforce".

Two fixes, and the first belongs in the pack, which is the point:

- **Emit designated initialisers** (`{ .name = "high", .groups = …, … }`, C99
  and valid in the C++ the C++ pack targets from C++20). Field-order drift then
  becomes a COMPILE ERROR at the generated TU instead of a silent
  mis-assignment, and a removed field is named in the diagnostic. This is a
  rendering change — a pack edit and a golden update — which is precisely the
  kind of change this design is meant to make cheap.
- **Extend the mirror gate** to `nros_native_tier_spec_t` across its nine
  sites. Designated initialisers make drift loud at the point of use; the gate
  makes it loud at the point of edit. They are complementary, not alternatives.

### 2. Is the C path being native-only a defect? — **No; it is routed, and correctly**

The first draft read `emit_c` in isolation, saw `nros_board_native_run_*`
hardcoded, and inferred a board-blind emitter. Reading the DISPATCH says
otherwise (`cmd/codegen.rs`):

```rust
Lang::C if !board_is_embedded(&plan.board) => emit_c::emit_typed(&plan)?,
// C++ entries, and embedded C entries (routed here for the C++ board runner).
_ => emit_cpp::emit_typed(&plan)?,
```

An embedded C entry routes to the C++ emitter, which produces a `.cpp` TU that
invokes each C node through its `extern "C"` `__nros_c_component_*` seam; cmake
gives that output a `.cpp` extension and links `NanoRosCpp`. The embedded board
runners are C++ only, so this is the correct shape and not an omission.

So `emit_c` is native-only BY CONTRACT, the contract is enforced one level up,
and archived issue **0097** is genuinely resolved. No lowering refusal is
needed and the C pack should NOT grow RTOS shapes.

**What the study did find is a flaw in the goldens.** The harness calls each
emitter directly, so `c_nuttx_one`, `c_zephyr_one`, `c_freertos_one`,
`c_threadx_one` and `c_nuttx_tiers` record output the real pipeline never
produces — they are byte-identical to the native rows except the board name in
a comment. They read as board coverage and are not. Either the harness should
route through the dispatch, or those rows should say what they actually pin:
that `emit_c` ignores the board, and that the dispatch is what stops that
mattering. Tracked on issue 1102.

## Remaining open questions

- Should the designated-initialiser change land with the C++ pack conversion, or
  ahead of it as its own step? It is a byte-visible change to every generated
  entry, so it wants its own goldens diff rather than riding a larger one.
- Does the mirror gate belong in `check-ffi-struct-mirrors` (extending its
  scope) or as a sibling keyed on this struct? The existing gate's shape —
  extract both bodies, normalise, compare — assumes TWO files; this one has
  nine.
