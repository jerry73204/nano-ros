// RFC-0090 / phase-429 — the codegen version, the one token that says whether
// generated code and this runtime can work together.
//
// NO `//!` INNER DOC COMMENTS IN THIS FILE, ON PURPOSE. It is `include!`d
// verbatim by `nros-build-helpers`, and an inner doc comment inside an
// `include!` expansion is a hard error (E0753). The module docs live on the
// `pub mod` in `lib.rs`. Keep this file a dependency-free set of `const`s and
// `const fn`s: anything else breaks the include.
//
// THREE READERS, THREE REASONS — do not "unify" them:
//
//   * `nros-core` itself, directly.
//   * `nros-build-helpers` (defines the runtime anchors) and `rosidl-codegen`
//     (stamps the emitted artifacts), both by `include!`. Neither can afford a
//     `nros-core` dependency edge for a pair of `const`s: the first is host-only
//     and appears in every tracked leaf lockfile, and the second lives in the
//     separate `packages/cli` workspace, so the edge would add rows to its
//     lockfile too. `include!` costs nothing in the graph, and rustc records the
//     path in the depfile, so editing the constants rebuilds both.
//   * the CLI's guard, by parsing this file as TEXT. It inspects a CONSUMER's
//     tree at run time, where compiling is not available. This is the only
//     parser, and it exists because the other two options do not apply.

/// The codegen version this runtime emits and accepts.
///
/// **Bump this deliberately** when the interface between generated code and the
/// runtime changes: a trait signature generated code implements, a symbol it
/// defines, a layout rule it obeys. Do NOT bump it for a cosmetic template
/// edit — that moves the fingerprint, which is a different question (see the
/// module docs).
///
/// Bumping invalidates every generated tree. That is affordable here because
/// `generated/` is never committed (CLAUDE.md), so regeneration is always
/// available — and it is exactly why the value must not move on cosmetics.
///
/// Gated by `check-codegen-version-surface`, which fails when the surface
/// generated code names changes and this constant does not.
pub const NROS_CODEGEN_VERSION: u32 = 2;

/// The oldest codegen version this runtime still accepts.
///
/// Equal to [`NROS_CODEGEN_VERSION`] at introduction: no window, because
/// `generated/` is never committed and regeneration is therefore always
/// available. Raise the floor only when a real migration needs one, and lower
/// it never.
///
/// Held at 1 while [`NROS_CODEGEN_VERSION`] moved to 2 (phase-417, rebased onto
/// phase-429's gate). That surface move is ADDITIVE plus one relocation, so a
/// tree generated against version 1 still runs: the C additions are new entry
/// points, the two C++ changes add `size()`/`empty()` and `std::string` interop
/// to `FixedString`/`HeapString`, and the four `nros_ret_t` declarations that
/// left `action.h` / `client.h` / `parameter.h` / `service.h` were CONSOLIDATED
/// into `nros_generated.h`, which all four still reach through `nros/types.h`
/// — the headers say so in a comment, and keeping that include is what makes
/// `#include <nros/action.h>` continue to compile. Nothing a version-1 tree
/// names was withdrawn, so this is the window the doc above describes rather
/// than a migration.
///
/// The range `[NROS_CODEGEN_VERSION_MIN, NROS_CODEGEN_VERSION]` is expressed to
/// C and C++ as a SET OF DEFINED SYMBOLS rather than as a comparison — see
/// `nros-build-helpers`' codegen-version anchor — so there is no range check on
/// that side that could itself be wrong.
pub const NROS_CODEGEN_VERSION_MIN: u32 = 2;

/// Does `emitted` fall in the range this runtime accepts?
///
/// The ONE comparison. Rust call sites reach it through
/// `nros_node::codegen_version_check`, the CLI through `abi_guard`; neither
/// re-spells the bounds.
#[must_use]
pub const fn accepts(emitted: u32) -> bool {
    emitted >= NROS_CODEGEN_VERSION_MIN && emitted <= NROS_CODEGEN_VERSION
}

// The accepted range must be non-empty, checked at COMPILE time because it is a
// property of two constants. A runtime test would be the wrong tool — and
// clippy says so (`assertions_on_constants`) in every crate that `include!`s
// this file, which is how it was found.
const _: () = assert!(
    NROS_CODEGEN_VERSION_MIN <= NROS_CODEGEN_VERSION,
    "the accepted codegen range is empty: nothing could ever be compatible"
);
