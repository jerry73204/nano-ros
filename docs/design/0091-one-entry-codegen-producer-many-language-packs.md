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
"for byte-level diffs against the proc-macro output". That diff did not exist —
no gate, and the only test naming both never mentioned the emitter. **§7 now
has it** (phase-432 W2.4), so the two paths stay two paths on purpose: the
duplication is a deliberate answer to issue 0083, and what was missing was the
measurement. What nothing consumes is the CLI's `--lang rust` VERB —
`nano_ros_entry` passes `--lang` as `c` or `cpp` only, and Rust entries reach
the proc-macro via `rust_cargo_application()` — and that is retired. Read §7
before acting on this paragraph: an earlier draft of it read "delete
`emit_rust.rs`", which would have deleted the gate's other half.

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
  (structured, neutral)                (structured, neutral — NOT rendered text)
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

Escaping is the exception that proves the boundary, and §8b corrects an earlier
version of this section. Quoting IS a correctness property and must stay in
compiled, reviewed Rust — but it is not language-neutral, so it cannot be baked
into the IR. `LoweredEntry` carries RAW values; each pack gets an escaping
FILTER registered on the render environment. Templates place values; they never
compute one, and they never invent a quoting rule.

---

## 5. Target facts — none; two kinds of agreement, one gated

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
the target. A profile is a model of the toolchain, and a model can be wrong
silently.

### Two kinds of agreement, and the first draft conflated them

Implementing W1.1 found that "where two languages must agree on a
representation" describes **two different properties** in this tree, with
different requirements and different gates. Saying it once, as the first draft
did, produced a gate scoped at the wrong pair.

| | **Wire agreement** | **Memory agreement** |
| --- | --- | --- |
| Mechanism | CDR | `repr(C)` |
| Who | idiomatic types (`packs/rust`, `packs/nros`, `packs/c`) | FFI surfaces (`packs/rmw`, `packs/cpp`) |
| Scope | across machines, across targets | ONE image, ONE target |
| Target-dependent | **no** — CDR fixes it by specification | yes — it is the target's C ABI |
| Needs a gate | no: the format is the contract | **yes** |

The measurement that forced this: the **idiomatic generated Rust carries no
`repr(C)` anywhere**. It and the generated C header share no memory and are
under no obligation to agree on layout — what they share is CDR. A size gate on
that pair would assert a property nobody needs and that a legitimate change
could break.

### What the memory-agreement gate is actually checking

Not "does codegen model the target correctly" — nothing models the target any
more. Both sides of an FFI surface render from the SAME `LoweredField` through
their own type filters (`c_type`, `rust_type_rmw`). Rust's `repr(C)` follows
the target's C ABI and the C compiler follows the same ABI, so the two agree
**by construction** unless the two filters SPELL a field differently — `u32`
against `uint16_t`, or an `enum` on one side and a fixed-width integer on the
other.

So the property is **pack consistency over one IR**, not target modelling. That
is the unified-path principle applied to a gate: one lowered fact, two
renderings, and the gate asserts they still denote the same thing.

Comparing symbol sizes after cross-compiling both sides remains the right
MECHANISM, because it needs neither side to know the other's spelling and it
reads the answer from the compiler that will actually build the image. What
changes is the scope — the `repr(C)` surfaces only — and the justification.

The sizes-header mirror (issues 0088 -> 0268) is the precedent and the warning:
one side authoritative, the other consuming, and a stale one was "silent memory
corruption", 336 bytes short on freertos C.

---

## 6. Stage 3 — packs, one per (language x surface)

### A pack is a (language x SURFACE), not a language

The first draft of this RFC — and the comment in `render.rs` it quoted —
say "adding a language = adding a pack". Counting the packs says otherwise:

| pack | language(s) | surface |
| --- | --- | --- |
| `c` | C | idiomatic C |
| `rust` | Rust | idiomatic Rust |
| `nros` | Rust | embedded-idiomatic (`no_std`) |
| `rmw` | Rust | FFI / ABI (`repr(C)`) |
| `cpp` | **C++ AND Rust** | bridge glue (`.hpp` + `repr(C)` `.rs`) |
| `scaffold` | Rust | packaging (`Cargo.toml`, `build.rs`, `lib.rs`) |

Rust has **four** packs. The `cpp` pack emits Rust. So the axis is not
language, and a design that says "one pack per language" mis-predicts the work
of adding one.

What a language actually needs is a DECISION PER SURFACE: does it want an
idiomatic surface, an embedded one, an FFI one, packaging? Zig would want
idiomatic and FFI (it speaks the C ABI natively) and no cargo scaffold. That is
a smaller and much more honest question than "write a pack", and stating it is
most of what §8 owes a newcomer.

The naming should follow: `packs/<surface>/` is what the directories already
are, and the registry keys (`message.h`, `message.c`, `build.rs`) are already
surface names rather than language names. The entry side should join that
convention — `packs/entry/<surface>/` — rather than inventing a second one.

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

**Scoped honestly: this is a C-only-RMW benefit.** zenoh and XRCE are C to the
bottom, so removing the generated `.cpp` removes the last C++ TU. Cyclone
(`rmw_cyclonedds_cpp`) and uORB are C++ libraries, so a C-only user on those
still links `stdc++` however the entry is written. The gain is real and it is
not universal; do not state it unqualified.

The reason is an accident of which entry points were declared, not a design
decision. The C-callable board surface:

| board | `run_components` | `run_tiers` |
| --- | --- | --- |
| native | yes (+`_named`) | yes |
| freertos / zephyr / nuttx | — | **yes** |
| threadx | — | — |

The layering runs opposite to the dispatch's justification for the TIERS path
— `nros_board_freertos_run_tiers` is 666 lines of C under a 4-line C++ veneer
— but **not for `run_components`**, and phase-432 W3.1's assessment corrected
this section: `FreertosBoard::run_components` is fully implemented in the C++
header with no C function under it, and native's C-ABI `run_components` is
Rust. So this IS new code.

The claim that survives is different and stronger: the C++ `run_components`
uses no C++ feature a C function lacks — no exceptions, no RAII, and its
`template <typename Setup>` is only ever instantiated with a plain function
pointer in generated code — while every primitive it calls is already C-ABI and
already called from C by the sibling `run_tiers.c`. Two prerequisites gate it,
both live traps: `nros_board_network_wait` exists only as a weak symbol in a
C++ header, and `NROS_ENTRY_LOCATOR`/`DOMAIN_ID` are derived in `main.hpp`
while the C sibling defines them as `""` and `0` — a pure-C entry would
compile, link, boot and dial nothing, which is issue #174 exactly. See
phase-432 W3.1. That deletes the language-crossing branch from
`cmd/codegen.rs` and lets the C pack serve every board the C++ one does.
**ThreadX has no C board API at all and is declared C++-entry-only**, rather
than silently routed.

---

## 6b. The common interface — what a pack may see and call

The design assumes a pack renders from the IR. Measured on the MESSAGE
pipeline, it does not, and the gap is why "adding a language = adding a pack"
is currently false.

### What is actually shared

| layer | shared? | what it is |
| --- | --- | --- |
| `render(name, ctx)` | **yes** | one entry point |
| `snake_case` filter | **yes** | the one language-neutral filter |
| `CapacityResolver` storage decision | **yes** | reaches every surface |
| the rest of `LoweredField` | **no** | never reaches a template |
| view structs | **no** | **38** of them, per surface |
| type-spelling filters | **no** | 9 of 10 are per-language |

### The two findings

**`lower_fields` output is reduced to storage before anything sees it.** Its
one caller (`lowered_storages`) returns `Vec<FieldStorage>` — the capacity
decision alone. `shape`, `cdr_op`, `align` and `plain` are computed and then
dropped. So Stage 2's IR is not the context; a projection of one field of it
is.

**Each surface re-derives from the PARSER, not the IR.** `RmwField` is built
straight from `rosidl_parser`'s field:

```rust
.map(|f| RmwField { name: escape_keyword(&f.name), field_type: f.field_type.clone(), … })
```

and there are FOUR such views of one field — `RmwField`, `IdiomaticField`,
`NrosField`, `CField` — each projecting independently. That is the same "one
fact, several authored spellings" this RFC exists to remove, one layer above
where it was found.

### The filters are already the right shape

`c_type` takes a `CTypeSpell` and returns a spelling. That IS the
neutral-fact-to-language-syntax boundary the design wants, done properly and
already load-bearing for five surfaces. The view structs are the wrong shape;
the filters are the model to follow.

### What the interface should be

- **One context: `LoweredField` / `LoweredType`.** Not four field views and 38
  template structs. A surface that needs something the IR lacks makes the case
  for adding it to the IR, where every surface gets it, rather than projecting
  privately from the parser.
- **A language contributes a FILTER SET** — its type spellings — registered on
  the environment. That is Rust, and it should be: a spelling is a correctness
  property, and §5's memory-agreement gate exists precisely because two filters
  can disagree.
- Then "adding a language = a pack plus a filter set" becomes TRUE. Today's
  claim in `render.rs` — "no other Rust" — is aspirational, and a newcomer
  discovers that only after writing the pack.

This raises §6's bar for the entry side too: `LoweredEntry` must be the
context, not the seed for a per-surface projection. The entry pipeline has not
paid this cost yet because it has two surfaces; W2.3 (converting `emit_cpp`) is
where it would first bite.

## 7. One path per outcome

**Retire the `--lang rust` VERB, and KEEP the renderer.** *(Amended by
phase-432 W2.4, which implemented this section. The first draft said "delete
`emit_rust.rs` — no consumer", and that is wrong in a way worth recording,
because it contradicts the parity gate two paragraphs down: with one renderer
there is nothing to compare. Measured: `NanoRosEntry.cmake` does reject any
`LANG` but `cpp`/`c`, and a Rust entry does reach the proc-macro through
`rust_cargo_application()` — but `cmd/codegen.rs` dispatched `--lang rust` to
the emitter and the golden harness renders it with two committed goldens. What
was dead is the VERB. It also produced a strictly POORER entry than the macro's
— the `OwnedSpin` register path only, silently ignoring tiers, `[lifecycle]`,
`[param_services]` and executor sizing — so retiring it is a fix, not a
tidy-up. The CLI's Rust STORY is still `builder/entry.rs` emitting
`nros::main!(...)`.)*

**Both Rust producers consume `LoweredEntry`.** *(Also amended: this said the
macro "stops re-deriving tiers (91 references), QoS (23) and board paths (6)".
Measured on `main_macro.rs`: 93 `tier`, 34 `qos`, 15 `board_path` — but every
one is a REFERENCE to shared code in `nros-orchestration-ir`, not a
re-derivation. `board_path_for` delegates (phase-346), QoS goes through
`qos_override::lower_all` (issue 0303), tiers through `resolve_tiers` /
`tier_from_model` / `derive_tiers_from_contracts` (phase-228.G, RFC-0032 §6).
That convergence had already landed. What was genuinely duplicated is the
**per-node runtime bake** — params, remaps, QoS overrides, identity and the
`register` call — which is precisely where the two drifted, and
`sanitize_pkg`, which existed twice character-for-character. Those are what
moved into `nros-entry-lower`.)*

It keeps `quote!` rendering behind a **parity gate** that renders a shared
corpus both ways and compares — the diff `emit_rust` promised and never
delivered. The comparison is TOKEN-wise: `quote!` has no formatting and a
template does, so the two agree on tokens and can never agree on whitespace.
It found a real divergence immediately — a QoS code rendered `1` by the
template where `quote!` interpolating a `u8` emits `1u8`.

Rendering through the pack instead (parse text -> `TokenStream`) was considered
and rejected: it compiles `minijinja` into every user's entry build, which is
the dependency weight that created this duplication once already.

**The CMake templates become a pack.** Phase-416 already collapsed six into one
parameterised RTOS template; `check-entry-session-name` gates both producers for
the fact that drifted. Until they are a pack, that gate is what holds them
together.

---

## 8. Adding a language — the whole procedure

0. **Decide which SURFACES the language needs** — idiomatic, embedded-idiomatic,
   FFI/ABI, bridge glue, packaging. This is the step the first draft omitted by
   assuming one pack per language, and it is the step that sizes the work: Rust
   has four packs, and the `cpp` pack emits Rust as well as C++. A language that
   speaks the C ABI (most of them) needs an FFI surface and inherits the memory
   agreement gate with it (§5).
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

## 8b. Simulation: adding Zig — where this design breaks

The procedure above is only worth what it survives. Walking a real third
language through it finds three defects, all in **Stage 2**, and all of the same
kind: facts that look neutral but are spelled for the C family.

A Zig entry is a normal C-ABI consumer. It would `@cImport` `nros/main.h` and
`nros/boot_config.h`, declare the component seams `extern`, and finish with
`c.nros_board_native_run_components_named(...)`. Nothing about that is exotic —
which is exactly why it is a good probe.

### Defect 1 — `board_path` is C++, not a board

`LoweredEntry` was to carry the board as `::nros::board::LinuxBoard`. That is a
C++ class path. A Zig pack cannot use it, a pure-C pack cannot use it, and the C
pack today does not (it calls `nros_board_native_run_components_named`). The
NEUTRAL fact is the board IDENTITY plus its boot shape; how that becomes a call
is a pack's business — a class path in C++, a function symbol in C and Zig.

### Defect 2 — escaping cannot be done in Stage 2

§4 says Stage 2 escapes every literal, on the grounds that quoting is a
correctness property that must stay in trusted Rust. The first half is right and
the second half does not follow: a C string literal, a Rust string literal and a
Zig string literal have **different** escape rules. There is no such thing as an
"already escaped" literal that is neutral.

The fix keeps the safety and drops the false neutrality: `LoweredEntry` carries
the RAW value, and each pack gets an escaping **filter** — Rust code registered
on the render environment, so quoting is still compiled and reviewed rather than
written in a template. `rosidl-codegen` already does exactly this
(`add_filter("snake_case", …)`, `add_filter("c_type", …)`), so it is the
established mechanism, not a new one.

### Defect 3 — pre-rendered text is a language in disguise

The C conversion that landed carries **six** pre-rendered fields:
`boot_config`, `spec_rows`, `groups_arrays`, `trailer`, per-node `decls`, and
`name_lit`. Every one is C syntax produced in Rust. They are correct for C, and
they are correct for C++ because it shares C's syntax here — which is precisely
why the shortcut survived review and would NOT have survived a third language.

Zig's tier table is `.{ .name = "high", .groups = &groups_0, … }`; its baked
config is `export var … linksection(".nros_boot_config")`; its declarations are
Zig statements. None of that can be reached from a C string.

So Stage 2 must hand over **structured** data — tier rows as fields, not
initialiser text; boot config as values, not a blob; per-node declarations as a
list of what to declare, not C statements — and each pack spells it.

### What the simulation does NOT break

- The pipeline itself. `Plan → lower → render → goldens` holds; only the
  contents of the IR were wrong.
- `Language` as one enumeration. Zig is one variant, seen everywhere.
- The goldens. A Zig row is a coordinate and a file to read, exactly as
  designed.
- Sharing within a language FAMILY. The C and C++ packs legitimately share
  sub-templates (`c_service_trailer.c.jinja` already is one). That is a
  pack-level relationship, and it belongs in Stage 3 — not smuggled into Stage 2
  by pre-rendering.

### Consequence for the conversion already done

`emit_c`'s move to a template is correct and stays: it proved the pipeline, the
renderer and the goldens end to end. What it did not prove is neutrality, and
this RFC should not claim it did. Its view is a C-family view, and the
structured-IR work above is what turns it into a `LoweredEntry` a third language
could render from.

That ordering is deliberate rather than a mistake to unwind: converting one
language first is what made the shortcut visible at all.

## 9. Acceptance

- `Language` declared once in a serde-only leaf; the `snake_case` wire repr of
  the metadata it replaces is unchanged, and gated.
- No type re-spells the enumeration; a narrowing derives and says so.
- `nros-entry-lower` exists inside the proc-macro's dependency budget;
  `LoweredEntry` is `serde`-serializable and carries **no target facts**.
- `LoweredEntry` carries no RENDERED TEXT and no language-specific spelling:
  no C initialiser strings, no pre-escaped literals, no C++ class paths. A
  third language must be renderable from it without a Stage 2 change (§8b).
- Escaping is a per-pack filter in Rust, not a field of the IR.
- `TargetProfile` is gone from `rosidl-lower`, and nothing replaces it.
- A gate covers MEMORY agreement — the `repr(C)` surfaces (`packs/rmw`,
  `packs/cpp`) against the C header — by cross-compiling both and comparing
  symbol sizes. It does NOT gate the idiomatic pair, which shares CDR and not
  memory (§5).
- The surface axis is named as such: `packs/<surface>/`, and the procedure in
  §8 starts by choosing surfaces rather than assuming one.
- The template context IS `LoweredField`/`LoweredType` — no per-surface view
  struct re-derives from the parser, and no field fact is computed by Stage 2
  and then dropped before a template can read it (§6b).
- A language contributes a pack and a FILTER SET, and nothing else in Rust.
  `render.rs`'s "no other Rust" becomes true rather than aspirational.
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
