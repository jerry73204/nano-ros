---
id: 1069
title: "`MICROCDR_VERSION_STR` compiles as 2.4.1 under CMake and 2.0.2 under cargo, against a tree that is 2.0.2 — three numbers, none authoritative"
status: resolved
type: bug
area: rmw, build
severity: low
found: 2026-09-05
related: [1068, phase-420]
resolved: 2026-09-05
---

# The version is restated by hand in four places

Measured during the phase-420 W9 survey:

| Where | Says |
| --- | --- |
| the `micro-cdr` gitlink | upstream **v2.0.2** |
| the `micro-xrce-dds-client` gitlink | upstream **v3.0.1** |
| `nros-sdk-index.toml` `[source.micro-xrce-dds-client]` | `2.4.3-nros1` |
| `nros-rmw-xrce/CMakeLists.txt:59-62` | `PROJECT_VERSION* = 2.4.1` |

The CMakeLists sets `PROJECT_VERSION*` for the XRCE client and never resets it
before the micro-CDR `configure_file` at line 117, so `MICROCDR_VERSION_STR`
compiles as `"2.4.1"` under CMake and `"2.0.2"` under cargo — for the same
vendored tree, which is neither.

## Severity: low, on evidence

`git grep` finds **zero readers** of those macros in either vendored tree or in
ours, so nothing behaves differently today. It is filed because the next reader
will believe one of them, and because four hand-written restatements of one fact
is the shape that produced issue 1068 next door.

## Fix

Derive it. The gitlink is the fact — `git -C <submodule> describe --tags` — and
everything else should read from one place, the way phase-420 W5 made rmw
descriptor fields derived rather than authored. Correcting the four literals
would leave the same four literals.

## Resolution (phase-420 W9)

**Both lanes now read the version out of the vendored tree's own
`project(<name> VERSION "X.Y.Z")` line.** That statement is upstream telling us
what the checkout is, it is in-tree, and it is present in every clone that has
the submodule at all. `_nros_xrce_project_version()` reads it in
`nros-rmw-xrce/CMakeLists.txt`; `vendored_project_version()` reads it in
`nros-rmw-xrce-cffi/build.rs`. Both `panic`/`FATAL_ERROR` rather than defaulting,
and both watch the file (`CMAKE_CONFIGURE_DEPENDS` / `cargo:rerun-if-changed`) so
a submodule bump re-generates the headers.

Measured after the change, out of the two generated headers:

| Macro | CMake lane | cargo lane |
| --- | --- | --- |
| `MICROCDR_VERSION_STR` | `"2.0.2"` | `"2.0.2"` |
| `UXR_CLIENT_VERSION_STR` | `"3.0.1"` | `"3.0.1"` |
| `UXR_CLIENT_VERSION_MAJOR` | `3` | `3` |

(`<build>/ucdr_generated/ucdr/config.h` + `<build>/uxr_generated/uxr/client/config.h`
vs `target/debug/build/nros-rmw-xrce-cffi-*/out/include/{ucdr,uxr/client}/config.h`.)

The `PROJECT_VERSION*` leak is fixed EXPLICITLY, not by ordering: the CMake lane
re-points all four tokens at micro-CDR immediately before that `configure_file`,
because both upstream templates take the same token names and whichever
`configure_file` runs second would otherwise inherit the first tree's numbers.

Gate: **`just check xrce-vendored-versions`** (`scripts/check-xrce-vendored-versions.py`,
fast line, ~30 ms, buildless, selftest on the normal path).

**It checks WIRING, not shape — because shape was not enough.** The gate's first
version asserted only that no lane spells a version literal, that a derivation
block exists, and that the sdk-index rows match the trees. A review mutation
walked straight through it:

```cmake
-_nros_xrce_project_version("${MICROCDR_DIR}/CMakeLists.txt"          microcdr           _ucdr)
+_nros_xrce_project_version("${MICROXRCEDDS_CLIENT_DIR}/CMakeLists.txt" microxrcedds_client _ucdr)
```

Same call, same arity, no literals, block intact — gate green, and a real
configure then emitted `MICROCDR_VERSION_STR "3.0.1"`, which is this issue back
verbatim. A gate whose coverage is narrower than the rule it enforces is the
issue-0196 class, so the invariant is now stated as *the tokens that reach
micro-CDR's template come from micro-CDR's tree*, per hop, in both lanes:

```
cmake   dir var  →  derivation site  →  prefix  →  configure_file template
cargo   let root →  generator arg    →  in-body derivation + template read
```

The pairing is declared once in a `WIRING` table (dirname, upstream project id,
template, CMake dir var + prefix, Rust binding + generator fn, sdk-index row) and
every hop is checked against it, so a crosswire is a hop naming a field from the
other row.

Full check list: (1) each tree states exactly one parseable version, (2) the
client's own `set(_microcdr_version …)` matches the micro-CDR we vendor, (3)
neither lane spells a version literal, (4) both lanes still carry the derivation
block — without which (3) passes vacuously, (5) CMake wiring, all four hops,
(6) cargo wiring, all three hops, (7) the `nros-sdk-index.toml` rows agree with
the trees.

**Nine mutations, each watched red before landing**: the crosswire above; its
three cargo-lane equivalents (wrong `let` target, wrong generator argument,
wrong in-body binding/project id); a re-introduced version literal; the dropped
micro-CDR re-point (which also reproduces the configure symptom); deleted block
markers; a reverted sdk-index row; and a client bumped to expect micro-CDR 2.1.0.

Asymmetry worth recording: **the CMake crosswire is silent, the cargo ones are
not.** Both trees have a `CMakeLists.txt`, so reading the wrong one succeeds and
the wrong number lands in the header. On the cargo side each generator holds one
`&Path` parameter used for both the template read and the derivation, so a
crosswire either fails to compile (binding out of scope) or panics on a template
path that does not exist. The cargo checks are still worth their cost — they turn
an accident of file layout into a stated invariant, and they answer in 30 ms
instead of at build time — but they are not what was load-bearing here.

## What this issue got wrong

- **`git describe --tags` is not a usable derivation source**, and it is what the
  "Fix" section above proposed. A submodule is fetched by SHA with no tags, so
  `git -C packages/rmw/xrce/xrce-sys/micro-xrce-dds-client describe --tags`
  answers `fatal: No tags can describe '<sha>'` on an ordinary checkout. It also
  needs git at build time, which the CMake lane does not otherwise require.
- **"zero readers" is false.** `uxr/client/config.h.in` ends with
  `#if UXR_CLIENT_VERSION_MAJOR >= 4` guarding three `#error`s — upstream's own
  tripwire for work that must be done when the client crosses to 4.x. Compiling
  `MAJOR = 2` against a 3.0.1 tree armed it a whole major version late, in BOTH
  lanes. Severity stays low (nothing misbehaves today), but the macros are not
  inert.
- **The client literal was wrong in both lanes, not just one.** The table above
  reads as though CMake alone said 2.4.1; `build.rs` said it too. Only micro-CDR
  differed between the lanes.

## Deliberately NOT done

- **`nros-sdk-index.toml`'s `[source.*] version` was not folded into the
  derivation.** `nros setup --source micro-cdr` runs precisely when the checkout
  is absent, so that row cannot read the tree; it is a label
  (`setup --list`, `describe_source`) and the input `gen-support-status.py`
  renders into the book. It stays authored and is gated instead. Its VALUE was
  corrected: `micro-xrce-dds-client` said `2.4.3-nros1`, which is not a client
  version at all but `[tool.xrce-agent]`'s nros release label copied across, and
  `micro-cdr`'s `2.0.x` understated a pinned tag. Now `3.0.1` / `2.0.2`.
- **`book/src/reference/support-status.md` was not regenerated.**
  `python3 scripts/gen-support-status.py` aborts before it reaches any XRCE row,
  on an unrelated pre-existing disagreement: `zenoh-pico version disagrees —
  index says 1.7.2, version.txt says 1.8.0`. That check is at `render()` line 73,
  the XRCE row at line 128, so the abort is independent of this change. The page
  must be regenerated once that is settled.
- **The book's XRCE pairing rule is now visibly false and is left that way.**
  `gen-support-status.py` hard-codes "Client and agent are pinned to the same
  upstream release" for a client at 3.0.1 and an agent at 2.4.3 — and the client
  gitlink has been 3.0.1 since the row existed, so the sentence has always been
  wrong; it was merely invisible while the index restated the agent's number.
  Fixing it means editing that generator's prose, which is out of this change's
  scope. **Follow-up: file it.**
