---
id: 1102
title: "The entry emitters build C++/C/Rust with `writeln!`, so the generated
  text has no single shape to read — and issue 1003 is what that costs"
status: open
type: tech-debt
area: codegen, cli
related: [issue-1003, issue-1017, rfc-0068, rfc-0091, phase-416]
---

## Symptom

`nros`'s three entry emitters assemble target-language source by appending
strings:

| file | lines | `writeln!` | `push_str` | `format!` |
| --- | --- | --- | --- | --- |
| `codegen/entry/emit_cpp.rs` | 2437 | 47 | 85 | 9 |
| `codegen/entry/emit_c.rs` | 1025 | 35 | 52 | 4 |
| `codegen/entry/emit_rust.rs` | 367 | 9 | 1 | 5 |

Nothing in the tree shows what a generated entry LOOKS like. To answer "what
C++ does a two-component zenoh image get?" you execute the emitter in your
head across ~130 append sites and a branch chain. The output has no artifact,
so it has no reviewable shape.

The sibling pipeline already solved this. `rosidl-codegen` renders every
message backend from real template FILES — `templates/*.jinja` plus serde view
structs — through a 319-line `render.rs` (RFC-0068 Stage 3, `minijinja`, which
is already a workspace dependency). Message codegen is data-driven and
readable; entry codegen is not, and the difference is not a considered one —
RFC-0068 simply never scoped entry generation.

## Why it matters (this is not aesthetic)

Issue **1003** is the measured cost. The generated C++ entry never passed a
session name, so every image built through `nano_ros_add_node` registered with
the agent as `node`, a talker and a listener hashed to ONE client key, and the
agent saw a single peer. It lived from 2026-06-13 to 2026-09-03 **with a
correct sibling producer beside it the whole time** — `emit_cpp.rs` had passed
the name since 2026-06-27.

Two spellings of one fact, and nothing compared them, because neither spelling
is a document you can put side by side. The CMake half was six near-identical
`.cpp.in` templates that differed in three tokens; that half was legible enough
to merge into one (phase-416) once someone looked. The Rust half is 2437 lines
of appends and was not, so the same defect there would still be invisible.

Related shapes already found in the same code:

- **1017** — the session name was set in FIVE cmake branches, one per entry
  shape. Same fact, five spellings.
- The boot wrapper (`int main` / `nros_app_main`) was spelled twice inside
  `emit_cpp.rs` alone, and the two spellings tested DIFFERENT predicates
  (`freertos || nuttx` against `board_is_embedded`). They happened to agree
  only because a condition seventy lines away excluded the one board they
  disagree about — an equivalence that has to be re-derived by hand every time
  either chain is touched. Fixed in phase-416 by deriving it once; the
  underlying reason it could happen is this issue.

## Not the whole file, and not a rewrite for its own sake

Much of these emitters is genuinely computation, not text: fixed-capacity
sizing, per-tier scheduling contexts, QoS override lowering, the boot-config
struct. That work belongs in trusted Rust and must NOT move into a template —
RFC-0068 is explicit that every embedded-critical fact "is still computed once
by trusted Rust and can never be recomputed wrong by a template".

The split this issue asks for is the same one RFC-0068 already draws: Rust
lowers a plan into a **view struct**; a **template file** decides the text.
What moves is the scaffolding — includes, the setup function, the component
loop, the boot wrapper — not the arithmetic.

## Acceptance

- Entry generation renders from template files under a `templates/` directory,
  through the existing `minijinja` path, not from `writeln!` chains.
- The view struct each template renders from is `serde::Serialize`, so what a
  template can see is declared rather than implied by what is in scope.
- Byte-for-byte equivalence with the current emitter is demonstrated, not
  assumed, for a set of plans covering every board family and both entry
  shapes — the emitter is what every C/C++ image boots through.
- Fixtures build and the runtime cells that read a session name still pass
  (`roundtrip_xprocess_e2e::two_template_built_entries_register_distinct_names`
  is the one that would catch a regression of 1003).

## Scope correction (RFC-0091)

This issue was filed as "three emitters build source with `writeln!`". That
understates it. Entry code has **four** producers, and two of them —
`emit_rust.rs` and the `nros::main!()` proc-macro (3822 lines) — are different
paths to the SAME outcome. Nothing consumes the CLI's Rust entry; cmake asks
only for `c`/`cpp`.

[RFC-0091](../design/0091-one-entry-codegen-producer-many-language-packs.md)
records the architecture: one producer (a leaf `nros-entry-lower` inside the
proc-macro's dependency budget) and many language PACKS. Converting the
emitters, which is what this issue tracks, is Stage 3 of that design — worth
doing on its own, and not the whole of it.

## Follow-ups the RFC-0091 study surfaced

Two items that belong to this issue's area, both evidenced in
[RFC-0091](../design/0091-one-entry-codegen-producer-many-language-packs.md):

1. **The tier spec table is initialised POSITIONALLY** against
   `nros_native_tier_spec_t`, a struct mirrored across NINE files whose own
   comment asks a human to keep them in sync. `check-ffi-struct-mirrors` does
   not cover it — it compares `component.h` against `nros_cpp_ffi.h` only. A
   field inserted anywhere but the end silently mis-assigns every generated
   entry (issue 0160's class, uncaught). Emitting DESIGNATED initialisers makes
   that a compile error at the generated TU; extending the mirror gate makes it
   loud at the point of edit. Both, ideally.

2. **Five C goldens record output the pipeline never produces.**
   `c_nuttx_one`, `c_zephyr_one`, `c_freertos_one`, `c_threadx_one` and
   `c_nuttx_tiers` come from calling `emit_c` directly, but `cmd/codegen.rs`
   routes an embedded C entry to the C++ emitter. They are byte-identical to
   the native rows except a comment, and they read as board coverage they do
   not have. Route the harness through the dispatch, or relabel them to pin
   what they actually prove — that `emit_c` ignores the board and the dispatch
   is what stops that mattering.

## The C conversion took a C-family shortcut (RFC-0091 §8b)

Simulating a THIRD language (Zig) against the converted `emit_c` found that its
view carries six PRE-RENDERED C fields — `boot_config`, `spec_rows`,
`groups_arrays`, `trailer`, per-node `decls`, `name_lit`. Each is C syntax built
in Rust. They are correct for C, and correct for C++ because it shares C's
syntax here, which is exactly why the shortcut survived: two languages in the
same family cannot expose it.

Zig's tier table is `.{ .name = "high", … }`, its baked config is
`export var … linksection(…)`, its declarations are Zig statements. None is
reachable from a C string.

So the remaining work under this issue is larger than "convert `emit_cpp`":

1. `LoweredEntry` must carry STRUCTURED data — tier rows as fields, boot config
   as values, declarations as a list of what to declare — not rendered text.
2. Escaping moves from the IR to a per-pack FILTER (Rust, registered on the
   render environment, as `rosidl-codegen` already does for `snake_case` and
   `c_type`). A "pre-escaped literal" is not language-neutral.
3. The board fact is an IDENTITY plus a boot shape, not `::nros::board::LinuxBoard`.

The converted emitters stay: they proved the pipeline, the renderer and the
goldens. They did not prove neutrality, and converting one language first is
what made the shortcut visible.

## Notes

`check-entry-session-name` gates the session-name property across both
producers and should keep passing unchanged; it scans the emitter for
`nros_boot_config_node_name`, and a template-rendered emitter must still
satisfy it (the marker moves into the template, which the check's glob
already covers for the CMake side and would need extending for this one).
