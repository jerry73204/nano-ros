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
- **W1.2** The cross-language repr gate. Compile the generated C and the
  generated Rust for ONE non-host target and compare `sizeof` against
  `size_of` over a type corpus. This is the replacement for the profile and
  must land WITH the deletion, not after: removing a model of the target
  without adding a measurement of it is strictly worse than either.
  Precedent and warning both: the sizes-header mirror (0088→0268), where a
  stale one was silent memory corruption, 336 bytes short on freertos C.
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
  `nros-launch-parser`, `nros-orchestration-ir`). If this crate grows a heavy
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

- **W2.4 — delete `emit_rust`; the proc-macro consumes `LoweredEntry`.**
  `emit_rust` has no consumer (`nano_ros_entry` passes `--lang` as `c` or
  `cpp` only; Rust entries reach the proc-macro through
  `rust_cargo_application()`), and its stated justification is a diff that was
  never implemented. The proc-macro stops re-deriving tiers (91 references),
  QoS (23) and board paths (6), keeps `quote!` rendering, and a **parity gate**
  renders a corpus both ways and compares — the diff that was promised.
  Rendering the macro through the pack instead was considered and rejected: it
  compiles `minijinja` into every user's entry build, which is the dependency
  weight that created this duplication once already (issue 0083).
  Depends on W2.2.

- **W2.5 — packs move to `packs/entry/<lang>/`.** Mirrors the message side's
  `rosidl-codegen/packs/<lang>/*.jinja` registry rather than the `templates/`
  layout the renderer shipped with. One convention or it drifts, which is the
  whole point.

- **W2.6 — the CMake templates become a pack.** `cmake/templates/*_entry_main*`
  render the same artifact from cmake variables. Phase-416 already collapsed
  six into one parameterised RTOS template; `check-entry-session-name` is what
  holds the two producers together until this lands.

**Acceptance.** `LoweredEntry` carries no rendered text; a third language is
renderable without a Stage 2 change; the proc-macro and the CLI share lowering
and a gate compares their two Rust renderings; goldens byte-stable throughout.

## Track 3 — make wiring a language actually good

RFC-0091 is honest that the codegen cost becomes a pack while the
**build-integration cost does not**. Track 3 is that gap, and it is the track
with the least design behind it — treat the items as scoped questions, not as
settled work.

- **W3.1 — complete the C-ABI board surface.** The concrete unblocker, and it
  is independent of Tracks 1–2. Most FFI-capable languages speak the C ABI and
  not C++. Today `run_components` is C-callable for `native` only, while
  `run_tiers` is C-callable on freertos/zephyr/nuttx — and the layering runs
  opposite to the dispatch's own justification:
  `nros_board_freertos_run_tiers` is **666 lines of C** that
  `FreertosBoard::run_tiers` wraps. So declaring
  `nros_board_<rtos>_run_components` is not new capability.
  It deletes the language-crossing branch in `cmd/codegen.rs`, makes the C pack
  serve every board the C++ one does, and **drops a C++ toolchain requirement
  from users who write only C** — MISRA and certified-C shops today pay for a
  C++ compiler to build an entry whose components are all C.
  ThreadX has no C board API at all and should be DECLARED C++-entry-only
  rather than silently routed.

- **W3.2 — a pack manifest.** `packs/entry/<lang>/pack.toml`: which template
  renders which output, the file extension, and whether the TU is C-family (so
  CMake knows which compiler and which link libraries). Today that knowledge is
  spread across `NanoRosEntry.cmake`'s `_lang_tag` derivation and the dispatch.

- **W3.3 — a conformance gate.** A pack that is half-wired must fail LOUDLY,
  not emit nothing. Assert every declared template exists and parses, every
  required output is produced for a reference plan, and the language has a
  golden coordinate. Without this, "adding a language is cheap" becomes "adding
  a broken language is cheap".

- **W3.4 — the documented procedure**, in the book. Four steps and the honest
  caveat: pack, registry row, `Language` variant, golden coordinate — and
  separately the toolchain work, which W3.1/W3.2 shrink but do not remove.

- **W3.5 — a reference third language, as a TEST rather than a product.** The
  Zig simulation was on paper and found three real defects; it could not find
  semantic ones (whether `linksection` actually satisfies the boot-config
  placement the loader expects needs a build). Deciding whether to carry a
  third language in-tree is a maintenance question and belongs to whoever owns
  the CI budget — this item is the question, not the answer.

## Ordering

```
  Track 1  W1.1 -> W1.2 -> W1.3          independent, cheapest, do first
  Track 3  W3.1                          independent (board C ABI)
  Track 2  W2.1 -> W2.2 -> W2.3          the spine
                        \-> W2.4
                            W2.5 -> W2.6
  Track 3  W3.2 -> W3.3 -> W3.4          needs W2.5's pack layout
           W3.5                          a decision, not a work item
```

W1 and W3.1 can run beside the Track 2 spine. W2.3 is the largest single item
and the one that unblocks the three deferred IR fields.

## Known gap this phase must not inherit

Five C goldens (`c_nuttx_one`, `c_zephyr_one`, `c_freertos_one`,
`c_threadx_one`, `c_nuttx_tiers`) record output the pipeline never produces —
the harness calls `emit_c` directly, but an embedded C entry routes to the C++
emitter. They are byte-identical to the native rows except a comment and read
as board coverage they do not have. Route the harness through the dispatch, or
relabel them to pin what they actually prove. W3.1 changes this answer: once
the C pack serves every board, those rows become real coverage rather than a
mislabel — so fix them WITH W3.1, not before.

## Also carried, from measuring this area

- The tier table is initialised POSITIONALLY against
  `nros_native_tier_spec_t`, a struct mirrored across **nine** files whose own
  comment asks a human to keep them in sync — and `check-ffi-struct-mirrors`
  does not cover it. A field inserted anywhere but the end silently
  mis-assigns every generated entry (issue 0160's class, ungated). Designated
  initialisers make it a compile error at the generated TU; extending the
  mirror gate makes it loud at the point of edit. Both. It is byte-visible in
  every generated entry, so it wants its own goldens diff rather than riding
  W2.3.
