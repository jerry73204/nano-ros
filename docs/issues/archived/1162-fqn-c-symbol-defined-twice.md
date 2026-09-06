---
id: 1162
title: "`nros_node_get_fully_qualified_name` was defined TWICE with different signatures — main's C API could not compile, and no merge-gating lane noticed"
status: resolved
type: bug
area: api, build
severity: high
found: 2026-09-06
related: [0417]
---

# Two PRs shipped the same capability, and the collision was invisible

`packages/api/nros-c/src/node.rs` on `origin/main` carried two
`#[unsafe(no_mangle)]` definitions of `nros_node_get_fully_qualified_name`,
with **different signatures**:

| | source | signature |
| --- | --- | --- |
| `:672` | PR #567 (phase-433 side work) | `(node, buf, buf_len, out_len)` |
| `:980` | PR #417 W-B5 | `(node, output_name, output_size)` |

Both were emitted into the committed `nros_generated.h`, so the header
declared the same symbol twice with incompatible prototypes. `check-build`
lost **9 of 21 gates** to it.

Neither author was wrong about the capability — the parity ledger had
`c:node_get_fully_qualified_name` as a `gap`, and two work streams closed it
independently within days. Both even reached the same design conclusions
(compose via `expand_name`, refuse rcl's `const char *`, return `NROS_RET_FULL`
on a short buffer), which is reassuring about the design and damning about the
coordination.

## Why nothing caught it

`check-build` is `schedule`/`workflow_dispatch` only — deliberately, because it
needs generated bindings and prebuilt `.compile-ok` artifacts that no
merge-gating job produces (recorded in CLAUDE.md). It is the **only** lane that
compiles `packages/api/nros-c/tests/compile/*.c`. So a duplicate `no_mangle`
export, a broken generated header, and a compile test pinning the wrong arity
all landed on `main` and stayed.

`just ci gate` — the tier a contributor is told to run before pushing — could
not pass on `main` for as long as this sat there.

## Resolution

Kept the four-argument form and merged the better half of the other:

* **Signature**: `(node, buf, buf_len, out_len)`. `out_len` is a real
  capability — a caller that gets `NROS_RET_FULL` learns the size that would
  have worked — and `nros-cpp`'s `Node::get_fully_qualified_name` and
  `std_compat`'s `std::string` overload are both written against it.
* **Body**: taken from #417's version. It reads the name with
  `inline_str(&node_ref.name, node_ref.name_len)` rather than
  `CStr::from_ptr`, which matters because the name is an inline array with a
  stored length and a name that fills it need not be NUL-terminated. It also
  uses the file's own `validate_not_null!` / `validate_state!` macros.
* **Docs**: kept #417's RFC-0020 violation-class-4 argument for why the
  composition is not written in the C layer.
* The shared `write_fqn` tail stays, so this and the two-string
  `nros_get_fully_qualified_name` cannot disagree about the terminator, the
  too-small report, or what `out_len` means.
* `packages/api/nros-c/tests/compile/node_timer_accessors.c` asserted the
  three-argument shape; updated, with a comment recording why it was wrong.
* Header regenerated (`cargo run -p nros-cbindgen-headers`).

Verified: `cargo build -p nros-c`, `cargo nextest run -p nros-c` 31/31,
`just check c` green.

## The durable question

A duplicate `#[no_mangle]` in one crate is a compile error the moment anything
builds it, so this is not subtle — it is only invisible because the lane that
would see it is unaffordable on a pull request. Two candidate fixes, neither
free:

1. A cheap static gate: no two `#[unsafe(no_mangle)] pub unsafe extern "C" fn`
   in the C API crate may share a name. Buildless, so it can sit on the fast
   line. Narrow, but it catches exactly this.
2. Compile *one* representative C TU on the merge path. That is the general
   answer and it costs what `check-build` costs, which is why it is not there.

Option 1 is worth doing regardless; it is minutes of work and this class has
now cost a day of `ci gate` on `main`.
