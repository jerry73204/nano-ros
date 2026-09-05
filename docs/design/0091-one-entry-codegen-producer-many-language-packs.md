---
rfc: 0091
title: "One entry-codegen producer, many language packs — and no two paths to the same entry"
status: Draft
since: 2026-09
last-reviewed: 2026-09
implements-tracked-by: []
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

### Stage 2 — `nros-entry-lower` (new leaf crate)

Everything that is *computation*: boot shape, board resolution, tier lowering, QoS
override lowering, boot-config baking, and escaping. Dependencies limited to
`nros-pkg-index`, `nros-launch-parser`, `nros-orchestration-ir` — the set the proc-macro
already accepts. That bound is the design's load-bearing constraint, not an afterthought:
if `nros-entry-lower` grows a heavy dependency, the proc-macro cannot adopt it and the
duplication returns.

### Stage 3 — language packs

`packs/entry/<language>/` — templates over `LoweredEntry`, plus a manifest saying which
template renders which output. **Per-language generators are expected and correct**; what
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
- Adding a language to entry codegen touches no Rust: a pack directory and a manifest
  row.
- The goldens (`testdata/entry/*.golden`) stay byte-stable across the whole migration —
  they are how each step is proven, and they already caught a rebase-introduced field
  change and three whitespace faults that review missed.

## Non-goals

- Changing what the entry DOES. This is about who produces it.
- Merging entry and message codegen. They share an architecture, not a pipeline: the
  inputs (`.msg` vs launch + `system.toml`) and the lowered facts have nothing in common.
- Making the CMake path disappear. `nano_ros_add_node` is a supported surface; it becomes
  a pack, not a casualty.

## Open questions

- Does `LoweredEntry` want a `TargetProfile` the way `LoweredType` does? Entry code
  currently bakes no layout facts, but the tier spec table is `repr(C)`-adjacent and a
  32-bit target may expose that.
- The C emitter emits `nros_board_native_run_*` regardless of board — the C path is
  native-only (archived issue **0097**). Whether the lowering should refuse a non-native C
  entry, or the pack should grow the RTOS shapes the C++ one has, is a separate decision
  this RFC does not settle.
