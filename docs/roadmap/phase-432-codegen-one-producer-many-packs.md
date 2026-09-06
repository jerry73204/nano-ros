# Phase 432 — one codegen producer, many language packs

**Status (2026-09-06).** Not started as a phase; four commits of it already
landed ahead of the design, which is why the design exists. The golden harness,
`emit_rust` and `emit_c` are on templates, the tier table is structured, and
escaping is a per-language filter. Everything below is what those four commits
proved was needed and did not do.

**Implements:** [RFC-0091](../design/0091-one-entry-codegen-producer-many-language-packs.md),
which **amends** [RFC-0068](../design/0068-language-neutral-codegen-ir.md).
**Closes:** [#1102](../issues/1102-entry-emitters-build-source-with-writeln.md).
**Touches:** [#1062](../issues/1062-add-node-language-inference.md)
(two language readers disagreeing — W2.1 removes the second reader).

## Why this phase exists

Issue 1003 cost three months: the generated C++ entry never passed a session
name, so every image registered as `node` and a talker and a listener hashed to
ONE agent client key — **with a correct sibling producer beside it the whole
time**. Neither spelling was a document anyone could put side by side.

Measuring that turned up more producers than the issue assumed. Entry code has
**four**, and two of them are different paths to the same outcome, where the
second's stated justification is a byte-diff against the first that was never
written. Message codegen already has the right shape (RFC-0068); entry codegen
never got it, and the two now differ in ways that make a third language
expensive for no designed reason.

## Starting position — what is already true

Do not re-do these; they are the evidence the rest is worth doing.

- 22 goldens under `packages/cli/nros-cli-core/testdata/entry/`, byte-compared
  every run. They have already caught a rebase-introduced struct field, three
  whitespace faults and a one-byte trailing newline — none of which review saw.
- `emit_rust.rs` and `emit_c.rs` render from `.jinja` templates through a
  bundled `minijinja` environment.
- The tier table is STRUCTURED (`Vec<TierView>` of values, not C initialiser
  strings), and escaping is the `c_str` FILTER rather than an IR field.
- `just check workspace-all` runs on `merge_group`, so a no-alloc break cannot
  reach main again the way issue 1080's did.

## Track 1 — drop `TargetProfile` (independent, do first)

Smallest track, no dependencies, and it makes the other two simpler by deleting
a concept they would otherwise have to thread.

`TargetProfile { ptr_width, enum_width }` is **inert**: one consumer outside its
own crate, passing `host()` unconditionally; `enum_width` read nowhere;
`ptr_width` read once as "a conservative stand-in" on a path whose own comment
says the result changes no outcome. The hazard it guards cannot occur — the C
and Rust message packs emit no `enum` at all.

- **W1.1** Delete `TargetProfile` from `rosidl-lower`; `lower_fields` loses the
  argument. The nested-struct `align` stand-in becomes an explicit named
  constant with the comment that already explains why its value cannot matter
  (`plain` is false for nested regardless).
- **W1.2** The cross-language repr gate — **re-scoped by implementing W1.1**,
  and the correction matters more than the original wording.

  The first draft said "compile the generated C and the generated Rust and
  compare sizes". Measured: the **idiomatic** generated Rust carries no
  `repr(C)` anywhere, so it and the C header share no memory and are not
  required to agree on layout at all — they share CDR, which is
  target-independent by specification. Gating that pair would assert a
  property nobody needs and that a legitimate change could break.

  The pair that genuinely shares memory is the **`repr(C)` one**:
  `packs/rmw/message.rs.jinja` and `packs/cpp/*.jinja` (both emit `repr(C)`)
  against the C header. That is where the short-enums hazard would bite if an
  `enum` were ever emitted, and it is what the gate must cover.

  Shape: for a type corpus, emit a size probe on each side (a sized global
  whose length is `sizeof`/`size_of`), cross-compile both for one non-host
  target (`arm-none-eabi-gcc` + `armv7a-none-eabi` are both present on this
  host), and compare symbol sizes — no execution needed. Skip cleanly when a
  toolchain is absent, and mutation-test it by changing a field width on one
  side only.

  It must land WITH the deletion, not after: removing a model of the target
  without adding a measurement of it is strictly worse than either. Precedent
  and warning both: the sizes-header mirror (0088→0268), where a stale one was
  silent memory corruption, 336 bytes short on freertos C.
- **W1.3** Amend RFC-0068 in place. Already drafted as a `> **Amended by**`
  block at its Stage 2; W1.3 is confirming it reads correctly once the code is
  gone, and moving RFC-0091's own status if it is then fully implemented.

**Acceptance.** `TargetProfile` appears nowhere; W1.2's gate is on a lane that
runs, and fails when a repr is changed on one side only (mutation-test it).

## Track 2 — unify the per-language process

The order matters and is not arbitrary: each step removes a blocker for the
next, and doing them out of order re-creates the duplication the phase exists
to remove.

- **W2.1 — `nros-lang`, one `Language`.** A leaf crate whose only dependency
  is `serde`. Placement is FORCED: `rosidl-codegen` does not depend on
  `nros-pkg-index`, and the proc-macro depends on neither
  `nros-orchestration-ir` nor `nros-cli-core`, so no existing crate is
  reachable by all consumers.
  `Language { Rust, C, Cpp }` carries the `snake_case` serde repr
  `ComponentLanguage` already writes, so on-disk source metadata does not
  change — gate that repr, it is a compatibility surface.
  The other three types are NOT collapsed and the RFC says why: `All` is a CLI
  affordance, `Rust|Other` is a predicate, `Rust|C` is a genuine narrowing
  where `Cpp` is invalid. **One enumeration, many narrowings, and a narrowing
  derives rather than re-spells.**
  Closes the second reader in #1062.

- **W2.2 — `nros-entry-lower`, structured `LoweredEntry`.** All computation,
  inside the proc-macro's dependency budget (`nros-pkg-index`,
  `nros-launch-parser`, `nros-orchestration-ir`).

  **Shape, settled by reading both callers rather than guessing.** The shared
  thing is NOT `Plan`: the proc-macro has no `Plan`, and `Plan` is the CLI's
  own projection of the input. What both already converge on is
  `SystemModel` plus `nros_orchestration_ir::{ResolvedTierTable,
  resolve_tiers}` — the macro imports exactly those today. So
  `nros-entry-lower` takes the SystemModel-derived facts, not `Plan`, and the
  CLI builds its input from `Plan` while the macro builds it from its own
  read. Designing this around `Plan` would have forced `Plan` out of
  `nros-cli-core` (it is used across `cmd/`, `builder/`, `codegen/`) for no
  gain, and would have handed the macro a type it has no way to construct.

  Encouraging for the budget: `NodeOverride` and `TierDef` already LIVE in
  `nros-orchestration-ir`; `cargo_metadata_schema` only re-exports them. The
  one dependency `Plan`'s module has that the budget excludes is `eyre`, and
  a leaf crate should carry a plain error type instead — `nros-lang` (W2.1)
  is the precedent. If this crate grows a heavy
  dependency the proc-macro cannot adopt it and the duplication returns, so the
  budget is a constraint, not a preference.
  Carries **no rendered text** and **no language-specific spelling**: no C
  initialiser strings, no pre-escaped literals, no C++ class paths. The board
  is an IDENTITY plus a boot shape — `::nros::board::LinuxBoard` is a C++ path
  and a Zig or pure-C pack cannot use it.

- **W2.3 — convert `emit_cpp` (2437 lines).** The emitter issue 1003 was
  actually about, and the one that RELEASES the three fields still
  pre-rendered in the C view: `decls`, `trailer`, `boot_config` come from
  `emit_boot_config_static` and the `pub(super)` `emit_declare_*` helpers,
  which `emit_cpp` still calls as string writers. Structuring them before this
  step would give the C++ emitter a second producer of the same text.
  Depends on W2.2.

- **W2.4 — retire the `--lang rust` VERB; both Rust producers consume
  `LoweredEntry`; a parity gate compares them.** *(LANDED. The item as first
  written said "delete `emit_rust`" AND "a gate compares their two Rust
  renderings", which cannot both hold — deleting the renderer removes the
  second thing to compare. Implementing it resolved that, and two of its
  premises measured FALSE. Both corrections are below, because a reader who
  believes the original will redo work that is already done.)*

  **Premise 1, corrected: `emit_rust` had TWO consumers, not none.** The
  claim is true of the BUILD — `NanoRosEntry.cmake` rejects any `LANG` but
  `cpp`/`c`, and a Rust entry reaches the proc-macro through
  `rust_cargo_application()` — and false of the tree: `cmd/codegen.rs`
  dispatched `--lang rust` to it, and the golden harness renders it with two
  committed goldens. So what is dead is the VERB, and that is what W2.4
  deleted. The verb was worth deleting on its own merits, not for tidiness:
  it renders the `OwnedSpin` register path and nothing else, so a plan
  declaring tiers, `[lifecycle]`, `[param_services]` or per-entry executor
  sizing compiled, linked, booted and ignored all four — issue 0302's shape,
  still live for the four features the CLI verb never gained. The RENDERER
  stays, because it is the second rendering the parity gate compares against.

  **Premise 2, corrected: the tier / QoS / board-path convergence had already
  happened.** Measured on `main_macro.rs`: 93 case-insensitive `tier`
  matches, 34 `qos`, 15 `board_path` — close to the doc's 91/23/6, but they
  are REFERENCES TO SHARED CODE, not re-derivations. `board_path_for`
  delegates to `nros_orchestration_ir` (phase-346); QoS goes through
  `qos_override::lower_all`, the same call the CLI emitters make (issue
  0303); tiers go through `resolve_tiers` / `tier_from_model` /
  `derive_tiers_from_contracts` (phase-228.G, RFC-0032 §6). Making the macro
  "stop re-deriving" those would have been a no-op.

  **What WAS duplicated, and is what W2.4 actually moved:** the **per-node
  runtime bake** — `runtime.params` / `remaps` / `qos_overrides` /
  `node_identity` and the `register` call each precedes. That is the text
  both producers emit from the same facts, and it is exactly where they
  drifted: four features reached the macro over four phases and left the
  emitter behind (archived issue 0302). It now lives in
  `nros_entry_lower::LoweredNode` (with `sanitize_pkg`, whose two
  character-for-character copies were `entry::sanitize_pkg` and
  `main_macro::pkg_to_crate_ident`), and the macro's four PARALLEL bake
  vectors — indexed positionally against `pkg_idents`, each pushed from a
  different arm — collapse into one struct per node.

  **The gate.** A corpus of `LoweredEntry` values in
  `packages/cli/nros-entry-lower/testdata/parity/` — in the crate BOTH
  producers depend on, so neither can cover a different set — is rendered by
  `emit_rust` into committed goldens and by the macro's `quote!` path, and
  the two are compared TOKEN-wise (`nros-macros/src/entry_parity.rs`).
  Token-wise and not byte-wise because `quote!` has no formatting and a
  template does; `Spacing` is a formatting hint and is deliberately not part
  of the comparison, or every case would report drift forever and the gate
  would be discarded as noisy. It found a real divergence on its first run:
  the CLI rendered a QoS code as `1` where `quote!` interpolating a `u8`
  emits `Literal::u8_suffixed`, i.e. `1u8`. Semantically identical, textually
  not, and nothing had ever compared them.

  Rendering the macro through the pack instead was considered and rejected:
  it compiles `minijinja` into every user's entry build, which is the
  dependency weight that created this duplication once already (issue 0083).
  Depends on W2.2.

- **W2.5a — the message-side context becomes the IR.** Measured (RFC-0091
  §6b): `lower_fields`' output is reduced to `Vec<FieldStorage>` before any
  template sees it, so `shape`, `cdr_op`, `align` and `plain` are computed and
  dropped; and each surface re-derives from the PARSER — `RmwField`,
  `IdiomaticField`, `NrosField`, `CField`, four views of one field, built from
  `rosidl_parser` rather than from `LoweredField`. Collapse them onto the IR.
  This is the same "one fact, several authored spellings" defect this phase
  exists to remove, one layer above where it was found, and it is what makes
  `render.rs`'s "no other Rust" claim true instead of aspirational.
  Do it surface by surface with the message goldens as the guard, not in one
  commit.

  **LANDED.** Six commits, one per surface plus a guard and an IR-prep, every
  golden byte-identical at each. Three corrections to the measurement, all
  written into RFC-0091 §6b ("Corrections from implementing W2.5a"): it is SIX
  views across FIVE surfaces (the `cpp` pack builds a header/`repr(C)` PAIR),
  two of the four were already half-migrated behind a `pre_storage:
  Option<FieldStorage>` back-door, and `align`/`plain` are not dropped facts —
  they answer `LoweredType::plain` inside `lower()`.

  One thing the item did not anticipate: **the goldens could not guard two of
  the five surfaces.** `emit_corpus()` covers `nros`, `c` and `cpp` and renders
  no `rmw` or idiomatic bytes at all, so a golden had to be recorded first
  (`tests/rust_surface_golden.rs`, deliberately outside `emit_corpus` so it does
  not move the fixture-staleness fingerprint for two surfaces with no production
  caller). Deleted on the way through: `lowered_storages`,
  `primitive_to_cdr_method`, `c_cdr_write_method`, `c_cdr_read_method` — the
  duplicate derivations gone rather than bypassed.

- **W2.5b — a language contributes a FILTER SET.** Nine of the ten registered
  filters are per-language type spellings (`c_type`, `rust_type_rmw`,
  `cpp_repr_c_type`, `nros_type`, …) and they are already the RIGHT shape — a
  neutral fact in, a spelling out. Name that as the language's Rust surface
  area, so "a pack plus a filter set" is the whole contract. The memory-
  agreement gate (W1.2) exists because two filters can disagree, so the two
  work items are each other's justification.

  **LANDED.** `rosidl_codegen::filters` — `FILTER_SETS`, one entry per
  `nros_lang::Language` plus the neutral one, each a declared name list beside
  the `register` fn that adds them. `render.rs` calls `register_all` and knows
  nothing else about languages. Counted rather than repeated, and the count
  HELD: ten registered, `snake_case` the one neutral, nine per-language.

  Three findings, all written into RFC-0091 §6b ("Corrections from implementing
  W2.5b"). The nine split **C 2 / C++ 4 / Rust 3**, not evenly — the C++ pack
  emits a header/`repr(C)` pair. **`cpp_repr_c_type` and `cpp_view_repr_type`
  return Rust**, not C++: the `cpp_` prefix names the pack that calls them. So
  the registry keys ownership by the CALLING PACK, not by emitted syntax —
  otherwise the first language wanting an FFI mirror would grow the *Rust* set.
  And `snake_case`'s neutrality is provable by use: the same `to_snake_case`
  builds C headers, C++ headers and Rust constants.

  What makes it more than a rename is the four tests, in both directions:
  every `Language` has a set, every declared name is really registered, every
  filter a bundled pack CALLS is registered, and every registered filter is
  CALLED. The third is the one with teeth — minijinja resolves filters at
  RENDER time, so a pack naming an unregistered filter used to fail at a user's
  build. It reads the packs with minijinja's OWN lexer
  (`machinery::tokenize`, a dev-dependency feature): the packs emit Rust and
  C++, so their text is full of `|item|` closures and `a || b`: a textual scan
  of `|ident` over the packs yields 23 names, of which **12 are not filters**
  (`item`, `crate`, `i`, `rmw`, `buffer`, `_`, `handler`, `scratch`,
  `ptr_ref`, `callback`, `response`, `request_scratch`). Each check
  was mutation-tested (drop the Rust set / declare an unregistered name / typo
  `c_type` to `c_typo` in `packs/c/message.h.jinja`) — all RED, restored GREEN.

  Not done, on purpose: the entry side's `c_str` filter stays where it is. It
  belongs to the C set by KIND, but the two ENVIRONMENTS must stay separate
  (entry templates are not in `bundled_packs()`, which feeds the codegen
  fingerprint), and `FilterSet::register` takes an `&mut Environment` so both
  can share one registry when W2.5/W2.6 make the entry templates packs.

- **W2.5 — packs move to `packs/<surface>/` and `packs/entry/<surface>/`. DONE.**
  The message side already had the convention; the entry side had a flat
  `templates/` directory with the language in the FILE name. Now
  `packs/entry/{c,cpp,rust,shared}/`, and the registry KEYS follow the message
  side too: an output is `<artifact>_<surface>.<ext>` (`entry_cpp.cpp`, beside
  the message side's `message_rmw.rs`), a partial keeps its `.jinja` suffix so
  the two kinds cannot be confused at a call site.

  `shared/` is a real surface rather than a dumping ground: `boot_config` and
  `declare_calls` are identical C that a C++ entry compiles unchanged, so they
  are one file rather than two spellings (W2.3).

  **"One convention or it drifts" is now checked, not just written down.**
  `every_pack_file_is_registered` walks the pack directory and compares against
  the registry — an unregistered pack file parses, looks wired and emits
  nothing, and that failure would otherwise surface as a missing output on some
  board. `registry_keys_follow_the_message_side_convention` holds the naming.
  Both mutation-tested: an orphan file and a key without its surface each go
  red.

  Two gates went RED on the move and were repointed, which is what naming a
  producer rather than globbing it buys: `check-ffi-struct-mirrors` (the tier
  initialisers) and `check-entry-session-name` (the boot wrapper). Every entry
  golden is byte-identical — this is a move, and the bytes say so.

- **W2.6 — the CMake templates become a pack. LANDED.** The six
  `cmake/templates/*_entry_main*.cpp.in` are DELETED;
  `nano_ros_node_register()` calls `nros codegen entry-node`, which synthesises
  a one-node `Plan` in Rust (`codegen/entry/registered_node.rs`) and renders the
  SAME `cpp_entry.cpp.jinja` the `nros build` path uses.

  **Why not "move the templates into the pack".** `configure_file` does `@VAR@`
  substitution and nothing else, so it cannot render a jinja template — one
  renderer had to win. That is why the retired templates pushed their single
  variable axis (`@NROS_ENTRY_SHAPE_RCLCPP@`) into a C preprocessor branch and
  could express no other; minijinja can express the shape, so it is the one that
  survives. Keeping six pack templates rendered through `nros` would pay the
  whole cost of the CLI dependency and buy none of the unification.

  **Why not a synthesised `SystemModel`.** That trades a duplicated TEMPLATE for
  a duplicated IR. A model is a build artifact of `nros sync`, never
  hand-authored, and `plan_from_model` additionally requires an
  `execution.deploy` slice that a single registered node does not have and would
  have to invent. The facts cross as ARGUMENTS and the synthesis lives in Rust —
  the shape `render_probe_main` already used for the metadata probe.

  **Cost, measured:** all 13 in-tree packages reaching this path declare message
  deps, so `nros_find_interfaces` already shells out to `nros` at configure time.
  The CLI is a new dependency for none of them.

  **Corrected counts.** This doc said `NanoRosNodeRegister.cmake` had FOUR call
  sites selecting `*_entry_main_c_typed.cpp.in`; there were FIVE (native,
  zephyr, nuttx, threadx, freertos), and TEN entry-template `configure_file`
  calls in all — five platform blocks x two languages.

  **`check-entry-session-name` is REDUCED, not deleted.** Its "compare two
  producers" half is retired because there is one producer, but three things it
  guards are not subsumed: the one-argument delegating overload in
  `<nros/main.hpp>` is still reachable and `cpp_boot_wrapper.cpp.jinja` still
  writes the call by hand; the name must still come FROM the node, which is a
  CMake-side property the pack cannot state; and the `--node-name` argument must
  actually be passed. It also now refuses a resurrected
  `cmake/templates/*_entry_main*`.

  **Behavioural change, deliberate.** Images built through the verb now carry the
  `.nros_boot_config` blob (they had none) and take their session name from it
  rather than from a substituted literal. The RTOS families get
  `NROS_ENTRY_LOCATOR` as a compile definition instead of a substituted string —
  the same spelling `nano_ros_entry` has always used, resolved by the same
  `_nros_resolve_entry_locator` (issue 0946). Two diagnostics are LOST: the
  `NROS_<FAM>_ENTRY_DEBUG` printf and the Zephyr C entry's issue-0157 loud
  failure print. The launch lane never had either, so this makes the two paths
  agree; restoring the loud print to the shared boot wrapper is a separate
  change because it moves every Zephyr golden.

**Acceptance.** `LoweredEntry` carries no rendered text; a third language is
renderable without a Stage 2 change; the proc-macro and the CLI share lowering
and a gate compares their two Rust renderings; the template context IS the IR
(no per-surface view re-derives from the parser, no lowered fact is dropped
before a template can read it); a language's Rust surface area is a filter set
and nothing more; goldens byte-stable throughout.

## Track 3 — make wiring a language actually good

RFC-0091 is honest that the codegen cost becomes a pack while the
**build-integration cost does not**. Track 3 is that gap, and it is the track
with the least design behind it — treat the items as scoped questions, not as
settled work.

- **W3.1 — complete the C-ABI board surface.** Assessed in depth; the
  CONCLUSION survives and the JUSTIFICATION did not. Correct the reasoning
  before starting, because the original is checkable and wrong.

  **What was wrong.** This said "declaring `nros_board_<rtos>_run_components`
  is not new capability" because `nros_board_freertos_run_tiers` is 666 lines
  of C. That inference does not hold. `run_tiers` and `run_components` are two
  different architectures: the tiers path IS C underneath a 4-line C++ veneer,
  but **`FreertosBoard::run_components` is fully implemented in the C++ header
  and has no C function under it** — and the one C-ABI `run_components` that
  does exist, native's, is **Rust**. So this is new code, not a
  re-declaration. It is also the DOMINANT embedded path, not a corner:
  `run_tiers` is reached only when a plan declares tiers.

  **The justification that survives inspection, and the part of it that does
  not.** The C++ `run_components` uses no C++ feature a C function lacks — no
  exceptions, no RAII, and the `template <typename Setup>` is only ever
  instantiated with a plain function pointer at every generated call site
  (capturing lambdas appear only in hand-written examples). That half holds.

  **"Every primitive it calls is already C-ABI" does NOT hold — it enumerated
  three and missed the two that matter.** `nros_cpp_init`, `nros_cpp_spin_once`
  and `nros_cpp_fini` are indeed C-ABI and indeed already called from C by
  `run_tiers.c`. But `run_components` also calls `nros::ok()` — through
  `detail::component_spin_loop`, which is its exit condition — and
  `nros::shutdown()`. **Neither is C-ABI, and neither can be made so by
  moving a definition**, which is what closed the other two prerequisites.

  Measured: `nros::ok()` is `Node::global_initialized()`, which is
  `GlobalStorageHolder<>::initialized` — a C++ **template static emitted by the
  header** and COMDAT-collapsed across TUs. `nros-cpp` ships no C++ translation
  unit at all (its `src/` is Rust, the CFFI implementation; the only `.cpp`
  files are tests), so there is nowhere in the crate to put an `extern "C"`
  accessor without adding one. `main.hpp`'s own comment already said this and
  the assessment did not read it: the loop "stays in the header because the
  per-tick yield (`k_yield()` on Zephyr) and the `nros::ok()` check are
  platform / C++-global coupled — there is no Rust-side yield or shutdown
  primitive to move it behind the CFFI".

  **Why `run_tiers.c` does not hit this**, which is what made the omission easy
  to make: its spin loops are bare `for (;;)` with no exit check — embedded
  firmware runs forever — so the sibling cited as proof never calls the two
  primitives that are missing.

  **So W3.1 has a THIRD prerequisite, and it is a different KIND from the other
  two.** Those were "one fact, two spellings; move the definition". This one is
  "the state a C runner must observe does not exist in C, and cannot be
  relocated". It needs a decision, not a move:

  - **(a) Move the global-initialized flag behind the CFFI**, into Rust beside
    `nros_cpp_init` / `nros_cpp_fini`, so `ok()` becomes a call. This is the
    right shape — one owner of "is the global session up" — and it touches a
    hot header path every C++ consumer inlines.
  - **(b) Add a shipped C++ TU to `nros-cpp` exporting
    `extern "C" bool nros_cpp_global_ok(void)`.** Ten lines, but it puts a C++
    TU in a crate that deliberately has none, and adds a second READER of a
    flag that would still have one owner.
  - **(c) Give the C `run_components` a different exit condition entirely.**
    Cheapest and worst: a second answer to "should this loop stop", which is
    this phase's own defect class.

  Estimate (a) or (b) before the ~35-50 lines this item quotes for the runner
  itself; the runner is not the work.

  **Two prerequisites, and they are the real risk — do them FIRST and
  separately:**

  1. **CLOSED.** `nros_board_network_wait` had exactly ONE definition and it
     was a weak symbol in a C++ header (`main.hpp`); the three RTOS
     `run_tiers.c` files called it `extern` and linked only because the
     generated entry was a `.cpp` that included that header. Proven, not
     assumed: a pure-C TU compiled at exit 0 and then failed with `undefined
     reference to 'nros_board_network_wait'`. Every count in this item was
     right — one definition, three callers — which is worth recording because
     four other counts in this phase were not.

     **The prescribed fix was the wrong shape.** A C stub TU would have been a
     SECOND definition of one symbol beside the header's weak one — this
     phase's own defect class — and would have depended on archive extraction
     pulling a member nothing else references, which is the FORCE_LINK /
     staticlib-DCE hazard one layer over. The definition MOVED instead, into
     `<nros/main.h>`: the C header that already declares the four
     `nros_board_*_run_tiers` this hook gates, and which `main.hpp` already
     includes. One definition, both languages reach it, the C++ path
     byte-for-byte unchanged. Nothing in the tree overrides the weak hook
     (measured: zero strong definitions), so it survives as an out-of-tree
     board affordance and now has its first in-tree test.

     **`check-weak-symbols` could not see the defect**, which is why a second
     probe was needed: re-guarding the definition as `#if defined(__cplusplus)`
     leaves that gate green while a pure C entry goes straight back to
     `undefined reference`, and `-fsyntax-only` — what every other C header
     probe in the lane uses — is green either way. `check-c` gains a LINK
     probe instead, linked twice: alone, and beside a TU whose strong
     definition must win.
  2. **CLOSED.** `NROS_ENTRY_LOCATOR` / `NROS_ENTRY_DOMAIN_ID` were derived in
     `main.hpp` while the C sibling `app_main.h` defined them as `""` and `0`
     with no derivation, so a pure-C entry would compile, link, boot — **and
     dial nothing**, a byte-for-byte re-run of issue **#174**. Measured before
     and after with the flags a Zephyr board passes: `[]` became
     `[tcp/127.0.0.1:7447]`.

     There were THREE spellings, not the two this item named. The third was a
     `#ifndef` further down `main.hpp` for the locator-less NuttX overload,
     promising `""` — and it could never fire, because the ladder above it had
     always already defined the macro. It delivered whatever that ladder
     produced, and is deleted.

     One ladder now, in `<nros/entry_config.h>`, included by both entry
     languages. `check-entry-locator-ssot` gained the header half: it already
     made the CMAKE-side locator ladders one producer (issue 0946), and the
     invariant is the same one layer down.

  **Cost, freertos:** ~35-50 lines for the function itself (its helpers are
  already in the TU), ~2-3 days end to end including cmake, gate and one green
  fixture. Zephyr and NuttX ~2-4h each afterwards.

  **ThreadX: do NOT** build it a C runner. It has no C entry runner at all — no
  `nros_board_threadx_run_tiers` either — so it needs a new TU plus `build.rs`
  wiring with nothing to copy.

  **But the prescribed `FATAL_ERROR` was wrong, and is not what landed.** Half
  the reasoning holds: "a ThreadX C entry silently becomes a `.cpp` with no
  diagnostic" was true, and the silence was the defect. The other half does
  not: the routing WORKS — the C++ board runner drives the entry and reaches
  each C node through its `extern "C"` seam — so refusing would break something
  that ships. ThreadX is not special either; every embedded board routes the
  same way, so a ThreadX-only refusal would have been arbitrary as well as
  breaking.

  What landed instead is the diagnostic the item actually wanted: `nros codegen
  entry` prints why the TU is C++ and what would change it, and `nros codegen
  entry-pack` reports `routed=1` so CMake and a human can both see it.

  **Acceptance is a BUILD, not a gate.** `examples/workspaces/c` must build
  and pass with `CONFIG_NROS_CPP_API` and `LANGUAGES CXX` REMOVED — that
  workspace contains zero C++ source and currently turns on Zephyr's whole C++
  subsystem (`zephyr/Kconfig`: `config NROS_CPP_API` … `select CPP`) for one
  generated TU. Anything less leaves the C++-toolchain claim unmeasured.

  **And the headline benefit is RMW-CONDITIONAL — say so, or the acceptance
  fixture measures a claim it cannot support.** "Drops a C++ toolchain
  requirement from users who write only C" is true for **zenoh** and **XRCE**,
  whose stacks are C all the way down. It is FALSE for **cyclonedds** and
  **uorb**: `rmw_cyclonedds_cpp` and the uORB shim are C++ libraries, so a
  C-only user on those RMWs still links `stdc++` whatever language the entry is
  written in. Pin the acceptance fixture to **zenoh**, and state the benefit as
  "on a C-only RMW" rather than unqualified.

  **Blocking sites for deleting the routing branch** (`cmd/codegen.rs:281-295`),
  found by enumeration rather than guess: `NanoRosEntry.cmake`'s `.cpp`
  extension override and its `_lang_tag`; ~~`NanoRosNodeRegister.cmake`'s call
  sites selecting `*_entry_main_c_typed.cpp.in`~~ — **CLEARED by W2.6**, which
  deleted those templates (there were FIVE such sites, not four, across ten
  entry-template `configure_file` calls); `emit_c.rs` + `c_entry.c.jinja`, which
  hardcode `nros_board_native_*` for every board; the two prerequisites above;
  and `check-entry-session-name.py`'s `PRODUCERS`, which errors if a listed glob
  matches nothing.

  On that third one, W2.6 supplies a measurement rather than a guess: the C
  emitter is BOARD-BLIND, and the goldens say so in bytes.
  `c_zephyr_one.c.golden` and `c_freertos_one.c.golden` both emit
  `nros_board_native_run_components_named` and a host `int main(int argc, char**
  argv)`. Nothing ships from that path today — the node-register lane now routes
  every board through the C++ emitter, and the launch lane already routed
  embedded C there — so the blindness is latent, not live. It becomes live the
  moment the routing branch is deleted, which is exactly what W3.1 proposes.

- **W3.1b — the surface decision, written down. DONE.** RFC-0091 §6 now
  carries "The surface decision table": five MESSAGE surfaces (idiomatic,
  embedded-idiomatic, FFI/ABI, bridge glue, packaging) with what each emits,
  when a language needs it, and — the column that was missing from the prose —
  **what each one drags in**. An FFI surface is not just a template; it
  inherits the memory-agreement gate (§5). A bridge surface is two spellings of
  one type that must move together. A packaging surface has the least in common
  between languages of any of them.

  Writing it out produced one thing the prose did not have: **entry surfaces
  are a SECOND AXIS, not a sixth row.** A language may take an entry surface
  with no message surface (it consumes another language's messages), or the
  reverse. The two entry surfaces are the entry TU and the component seam, and
  separating them changes the sizing answer: Zig could ship with the component
  seam alone — a Zig component installed by a C or C++ entry — which is a much
  smaller first version than "add Zig". §8 step 0 now points at the table
  rather than restating it.

- **W3.2 — a pack manifest. DONE.** `packs/entry/<surface>/pack.toml` declares
  the language, the generated TU's extension, whether a C-family compiler
  builds it, and the templates the pack renders. `nros codegen entry-pack
  --lang <l> --board <b>` serves the answer, and `NanoRosEntry.cmake` asks
  instead of deriving.

  **The honest finding: there was no live disagreement, and saying so matters
  more than dressing the change up.** CMake derived the extension from
  `NANO_ROS_PLATFORM != "posix"` while the CLI dispatched on
  `board_family(plan.board).is_embedded()` — two derivations of one decision on
  DIFFERENT inputs — and every (platform, board) pair the tree exercises gives
  the same answer both ways. `freertos-posix` looks like the counterexample (a
  HOST process whose board family is `Freertos`) and is not: its fixture rows
  set `NANO_ROS_PLATFORM = "freertos"`, so both sides say embedded.

  What was wrong is structural. Nothing MADE them agree, they read different
  inputs, `board_family` answers `Native` for any key it has not learned, and
  the failure if they diverged is a C++ TU written into a `.c` file — or worse,
  a `.c` file that happens to compile. `board_family`'s own `freertos-posix`
  comment records that this table has already produced one silent wrong answer
  of exactly that kind.

  Also corrected while measuring: an earlier reading of this called
  `riscv64-qemu` a divergent board key. It is not — that is `NANO_ROS_BOARD`
  (the hardware name), a different namespace from the `plan.board` the dispatch
  reads, which comes from the bringup's `system.toml`.

- **W3.3 — a conformance gate. DONE.** `just check entry-pack-conformance`, on
  the fast line, refuses a half-wired pack: a directory under `packs/entry/`
  with no manifest, a language pack missing a field CMake needs to name its
  output or pick its compiler, a `Language` variant nothing renders, or a
  language whose generated bytes are recorded in no golden.

  Split deliberately between Rust and Python rather than picked by taste. The
  manifest↔registry agreement is a Rust unit test, because it reads the
  bundled data directly where a script could only re-spell the key→path map —
  and re-spelling it is the drift the gate exists to prevent. The DIRECTORY
  check, the `Language` enumeration and the golden corpus are the script's,
  because those are what a contributor gets wrong and the fast line is what
  they run before pushing.

  The script parses its own flat TOML subset: this host's Python is 3.10 (no
  `tomllib`) and repo gates bring no dependencies. It REFUSES a line outside
  that subset rather than skipping it — a parser that ignores what it does not
  understand is how a gate reports green over a file it never read.

  Mutation-tested: a pack directory with no manifest, and a language pack with
  its `extension` removed, each go red with the file named.

- **W3.4 — the documented procedure. DONE.**
  `book/src/internals/codegen-packs.md` "Adding a language" is now four steps
  rather than three, and the new one is FIRST: decide which surfaces you need
  (§6's table), because that is what sizes the work and skipping it is why
  "add a language" sounds bigger than it is. Then the message pack and its
  filter set, then the entry pack and its `pack.toml`, then the goldens — with
  `just check entry-pack-conformance` named as the thing to run before
  believing the pack works.

  The honest caveat kept its own heading rather than a footnote: the toolchain
  story is not made cheap by any of this, and a reader planning the work should
  budget for it separately.

- **W3.5 — a reference third language. THE QUESTION, put properly.** This item
  was always the question rather than the answer, and the rest of Track 3 has
  now changed what the question costs. Recorded here so whoever owns the CI
  budget can decide on evidence.

  **What the paper simulation bought.** Simulating Zig found three real defects
  and they are all fixed: `board_path` was C++ leaking into the IR (W2.2),
  escaping could not be done in Stage 2 (W2.3's `c_str` filter), and
  pre-rendered text was a language in disguise (W2.3's three released fields).
  That is a good return for a design exercise, and it is exhausted — the same
  simulation run again today finds nothing, because the defects it was
  sensitive to are gone.

  **What it cannot reach.** Semantic agreement with the target. Whether Zig's
  `linksection` actually satisfies the boot-config placement the loader
  expects, whether its `extern struct` layout matches the C pack's on a
  cross target, whether its component seam links — none of those are readable,
  they need a build. That is the entire remaining risk surface, and it is
  exactly what a paper exercise is blind to.

  **What a third language would cost now, after W3.2–W3.4.** The codegen half
  is genuinely small: a message pack plus a filter set, an entry pack plus a
  `pack.toml`, a `Language` variant, a golden coordinate — and
  `check-entry-pack-conformance` refuses the half-wired result rather than
  letting it emit nothing. Call it a day's work for someone who reads
  `book/src/internals/codegen-packs.md`.

  The build-integration half is unchanged and is the real number: a toolchain
  CI can install, a cross-compile lane, a linked image, and a runtime cell that
  proves delivery. Track 3's opening line said the codegen cost becomes a pack
  while the build-integration cost does not, and nothing in W3.1–W3.4 changed
  that.

  **So the question is narrow: is a permanent cross-toolchain CI lane worth
  buying, to test a property (the packs generalise beyond C/C++/Rust) that
  nothing in the shipping product depends on?** A cheaper middle option exists
  and is worth naming: a third language carried as a COMPILE-ONLY fixture —
  generate, compile, never link or run — which catches layout and syntax
  divergence for a fraction of the lane cost and leaves the semantic questions
  open. Whoever answers should say which of the three they chose and why, so
  the next person does not re-derive it.

## Ordering

```
  Track 1  W1.1 -> W1.2 -> W1.3          independent, cheapest, do first
  Track 3  W3.1                          independent (board C ABI)
  Track 2  W2.1 -> W2.2 -> W2.3          the spine
                        \-> W2.4
                            W2.5a -> W2.5b -> W2.5 -> W2.6
  Track 3  W3.2 -> W3.3 -> W3.4          needs W2.5's pack layout
           W3.5                          a decision, not a work item
```

W1 and W3.1 can run beside the Track 2 spine. W2.3 is the largest single item
and the one that unblocks the three deferred IR fields.

## Known gap this phase must not inherit

**CLOSED.** Five C goldens (`c_nuttx_one`, `c_zephyr_one`, `c_freertos_one`,
`c_threadx_one`, `c_nuttx_tiers`) recorded output the pipeline never produces —
the harness calls `emit_c` directly, but an embedded C entry routes to the C++
emitter. They were not merely unreachable but WRONG if reached:
`c_freertos_one.c.golden` emitted `int main` +
`nros_board_native_run_components_named` for `board = freertos`, and
`c_nuttx_tiers` emitted `nros_board_native_run_tiers` for `board = nuttx` —
symbols those targets do not have.

Fixed at the SOURCE rather than by relabelling: `emit_c` now REFUSES an
embedded board, because every board call it renders is hardcoded
`nros_board_native_*` and native is the whole C-ABI board surface that exists.
The harness records a refusal as its golden, so the five rows now say what is
true — this emitter cannot serve that board — instead of showing source nobody
compiled. Exactly those five moved and no others, which is the check that the
refusal is scoped to what it claims.

Nothing shipping changed: the dispatch already routed embedded C to the C++
emitter, so only the harness ever called this with an embedded board. That is
precisely why it survived — the bytes were wrong in a file nobody compiled.
When W3.1 gives a board a C-ABI `run_components`, the refusal narrows to what
still lacks one, and those rows become real coverage.

## Also carried, from measuring this area

- ~~The tier table is initialised POSITIONALLY against
  `nros_native_tier_spec_t`~~ — **DONE**, on its own branch after W2.3
  (`fix/nros-native-tier-spec-designated-init`), with its own goldens diff as
  planned.

  **The count was NINE and it is EIGHT.** Measured: six DECLARATIONS —
  `packages/api/nros-c/include/nros/main.h` (canonical
  `nros_native_tier_spec_t`), `packages/api/nros-cpp/include/nros/main.hpp`
  (`NativeTierSpec`), `packages/api/nros-cpp/src/lib.rs` (`NativeTierSpecC`,
  `repr(C)`) and the THREE board `nros_tier_spec_t` mirrors
  (`nros-board-{zephyr,nuttx-qemu,freertos}/c/*_run_tiers.c`) — plus two
  INITIALISERS, `c_entry.c.jinja` and `cpp_entry.cpp.jinja`. main.h's own
  comment said "the 4 board `nros_tier_spec_t` mirrors"; there are three, and
  no fourth ever existed in this tree. `nros_platform::TierSpec`
  (`packages/platform/nros-platform/src/board/tier.rs`) is NOT one of them —
  it is a semantic sibling with different field names (`core`, `class`, an
  extra `time_slice_us`) and no `repr(C)`, so counting it makes the set nine
  and the ABI claim false.

  Both halves landed. Designated initialisers in both packs; the mirror gate
  now compares all eight sites against main.h — the C family by full
  declaration (types included, the `nros_tier_setup_fn_t` typedef normalized),
  C++/Rust by field-name ORDER, and the templates by the `.field` names they
  emit. A template that goes back to positional is a named failure, not a
  silent pass.

  **C++ designated initialisers are C++20 and this tree pins C++14** — the
  thing the item did not price. Measured accepted at `-std=c++14 -Wall
  -Wextra` by g++ 11.4, clang++ 14, arm-none-eabi-g++ 13.2,
  arm-zephyr-eabi-g++ 12.2 (Zephyr SDK 0.16.3) and riscv64-unknown-elf-g++
  10.2 — every compiler this tree reaches, as a GNU/Clang extension. Only
  `-Wpedantic` objects (`-Wc++20-extensions` on GCC 13+, `-Wpedantic` on GCC
  ≤12, `-Wc++20-designator` on Clang), and nothing in-tree sets it on a
  generated entry. The C pack is plain C99 and is `-Wpedantic`-clean. A
  downstream `-Wpedantic -Werror` build of a generated C++ entry — PX4's flag
  set, though PX4 compiles no generated entry — needs `-Wno-pedantic` on that
  TU; that is written into the template's own header comment rather than
  suppressed with a pragma, because the pragma spelling differs per compiler
  and the warning is telling the truth.

  Compile evidence: all eight tier-carrying goldens `-fsyntax-only` clean
  against the REAL headers — the three C ones at `-std=c11 -Wall -Wextra
  -Wpedantic`, the five C++ ones at `-std=c++14 -fno-exceptions -fno-rtti`
  (the embedded profile `check cpp` uses) under both g++ and clang++.
  Negative controls: a renamed field is `has no non-static data member named
  'klass'`, and a reordered one is `designator order ... does not match
  declaration order` — which is the property the change exists to buy.
