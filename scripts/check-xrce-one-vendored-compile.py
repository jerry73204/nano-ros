#!/usr/bin/env python3
"""One compile of the vendored XRCE tree — phase-420 W9 step 4.

The sibling of `check-xrce-source-manifest` (which files) and
`check-xrce-config-manifest` (with what values), one axis over: WHO COMPILES
THEM. Until this step, micro-XRCE-DDS-Client and micro-CDR were compiled twice
from the same manifest rows — by `nros-rmw-xrce-cffi/build.rs` through cc-rs for
six target families, and by `nros-rmw-xrce/CMakeLists.txt` through
`add_library(... STATIC ...)` for that project's own two CTest binaries, each
lane picking its own flags. The CMake copy reached no image
(`cmake/NanoRosRmwDispatch.cmake` maps `xrce` to the cargo crate for every
image; `zephyr/cmake/nros_rmw_xrce.cmake` is an explicit no-op; nothing
`add_subdirectory()`s or `find_package()`s the project), so its harness was
validating objects no image contains.

The CMake project now LINKS the cargo lane's archive. This gate is what keeps
that from quietly reverting — deriving without a gate is one refactor away from
a lane growing an `add_library` over the vendored trees again.

WHAT IT CHECKS

1. Each lane DECLARES the `xrce-sources.txt` trees it compiles, inside a
   `NROS-XRCE-COMPILED-TREES` marker block, and the declaration is the FILTER
   rather than a note about one — build.rs asserts row membership against it,
   the CMakeLists skips a row whose tree is not `IN_LIST` it.
2. The cargo lane declares EVERY tree. It is the lane that ships and its archive
   is what both lanes link, so a tree missing there is a tree missing from that
   link.
3. The CMake lane declares NO vendored tree, and locates no vendored source
   root (`_xrce_tree_uxr` / `_xrce_tree_ucdr` are gone) — so it cannot form a
   vendored source path even if the filter were bypassed.
4. The CMake lane actually links the cargo lane's archive: it requires
   `NROS_XRCE_CFFI_OUT_DIR`, reads the pointer file the BUILD SCRIPT writes, and
   declares an `IMPORTED` target located at that archive. There is no fallback
   compile — a fallback would restore the duplication while every symptom said
   it was gone.
5. THE WIRING, not only the shape: the pointer file's name and its two keys have
   ONE statement each, and the two files agree on them. A gate that checks only
   the shape passes a reader pointed at a file the writer does not write, which
   is the failure mode every gate in this family was written after meeting.
6. NO CONSUMER RESTATES A cc-rs FACT. The archive's NAME and the
   generated-header DIRECTORY are stated by the build script and nowhere else;
   reading them out of the pointer file is the only permitted spelling. So
   neither the CMakeLists nor the `rmw-xrce` recipe may name the archive stem,
   and the only path either builds off `NROS_XRCE_CFFI_OUT_DIR` is the pointer
   file itself.

   This rule was added after a review mutation that rules 1-5 all missed:
   replacing `IMPORTED_LOCATION "${NROS_XRCE_VENDOR_ARCHIVE}"` with
   `"${NROS_XRCE_CFFI_OUT_DIR}/libnros_rmw_xrce_c_inline.a"` leaves the shape
   perfect — one declaration per lane, the pointer file still read, each
   vendored tree still compiled once — and quietly restores a mirror. It is
   LATENT rather than live (the spelling happens to be right, and a rename would
   trip the `EXISTS` check at configure rather than link something stale), which
   is exactly how every mirror in this directory started: a comment promising
   the property and nothing checking it.

WHY 2 AND 3 ARE SEPARATE RULES

"Each vendored tree is compiled exactly once" is TRUE of a tree where the CMake
lane compiles the vendored halves and the cargo lane compiles only the backend —
a swap that is shape-valid, satisfies a naive count, and ships an archive with
no XRCE in it. Direction is part of the invariant, so it is stated.

Run: python3 scripts/check-xrce-one-vendored-compile.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
XRCE = REPO / "packages/rmw/xrce"
SOURCES = XRCE / "xrce-sources.txt"
BUILD_RS = XRCE / "nros-rmw-xrce-cffi/build.rs"
CMAKE = XRCE / "nros-rmw-xrce/CMakeLists.txt"
# The `rmw-xrce` recipe is the OTHER consumer of the cargo lane's OUT_DIR, so it
# is under the same no-restatement rule. It hands the directory over whole and
# derives nothing from it.
JUST = REPO / "just/check.just"
RECIPE = "rmw-xrce"

# The trees that arrive as git submodules — the ones this gate exists for. The
# `backend` tree is ours and is legitimately compiled by both lanes (the CMake
# lane compiles it for its warnings, which is the only place `-Wall -Wextra`
# runs over it).
VENDORED = {"uxr", "ucdr"}

MARK_BEGIN = "NROS-XRCE-COMPILED-TREES-BEGIN"
MARK_END = "NROS-XRCE-COMPILED-TREES-END"

# `const COMPILED_TREES: &[&str] = &["uxr", "ucdr", "backend"];`
_RS_TREES = re.compile(r"const\s+COMPILED_TREES\s*:\s*&\[&str\]\s*=\s*&\[([^\]]*)\]")
# `set(_xrce_compiled_trees backend)`
_CM_TREES = re.compile(r"set\s*\(\s*_xrce_compiled_trees\s+([^)]*)\)")
# `set(_xrce_tree_backend "…")`
_CM_TREE_ROOT = re.compile(r"set\s*\(\s*_xrce_tree_([A-Za-z0-9_]+)\s")
# `src <group> <tree> <path>` in the manifest.
_SRC_ROW = re.compile(r"^\s*src\s+\S+\s+(\S+)\s+\S+")
# `let archive_stem = "nros_rmw_xrce_c_inline";` — the build script naming the
# archive cc-rs writes. One statement, in the producer.
_RS_ARCHIVE_STEM = re.compile(r'let\s+archive_stem\s*=\s*"([^"]+)"\s*;')
# A path built off the cargo lane's OUT_DIR in the CMake lane.
_CM_OUT_DIR_PATH = re.compile(r"\$\{NROS_XRCE_CFFI_OUT_DIR\}/([^\"\s)]*)")
# The same in the recipe, where the variable is shell.
_SH_OUT_DIR_PATH = re.compile(r"\$\{?OUT_DIR\}?/([^\"\s]*)")


class GateError(Exception):
    """A structural problem that stops the check rather than adding a finding."""


def block(text: str, label: str) -> str:
    """The marker-delimited region, so a declaration cannot hide elsewhere."""
    try:
        start = text.index(MARK_BEGIN)
        end = text.index(MARK_END)
    except ValueError as exc:
        raise GateError(
            f"{label} has no `{MARK_BEGIN}` … `{MARK_END}` block. Each lane declares the "
            "xrce-sources.txt trees it compiles there, and this gate reads that block."
        ) from exc
    if end < start:
        raise GateError(f"{label} has `{MARK_END}` before `{MARK_BEGIN}`")
    return text[start:end]


def rust_trees(text: str) -> set[str]:
    region = block(text, "build.rs")
    m = _RS_TREES.search(region)
    if not m:
        raise GateError(
            "build.rs's NROS-XRCE-COMPILED-TREES block declares no "
            "`const COMPILED_TREES: &[&str] = &[…]`"
        )
    return {t.strip().strip('"') for t in m.group(1).split(",") if t.strip()}


def cmake_trees(text: str) -> set[str]:
    region = block(text, "CMakeLists.txt")
    m = _CM_TREES.search(region)
    if not m:
        raise GateError(
            "CMakeLists.txt's NROS-XRCE-COMPILED-TREES block declares no "
            "`set(_xrce_compiled_trees …)`"
        )
    return {t.strip() for t in m.group(1).split() if t.strip()}


def manifest_trees(text: str) -> set[str]:
    trees = set()
    for raw in text.splitlines():
        line = raw.split("#", 1)[0]
        m = _SRC_ROW.match(line)
        if m:
            trees.add(m.group(1))
    if not trees:
        raise GateError("xrce-sources.txt has no `src` rows")
    return trees


def pointer_facts(text: str, patterns: dict[str, str], label: str) -> dict[str, str]:
    """Each fact stated exactly once in `label`, or a finding saying so."""
    out: dict[str, str] = {}
    for fact, pattern in patterns.items():
        hits = re.findall(pattern, text)
        if len(hits) != 1:
            raise GateError(
                f"{label} states the `{fact}` fact {len(hits)} times (expected exactly one, "
                f"pattern {pattern!r}). One statement per fact is what makes the two files "
                "checkable against each other."
            )
        out[fact] = hits[0]
    return out


def recipe_body(text: str, name: str) -> str:
    """The named `just` recipe, from its header to the next column-0 name."""
    m = re.search(rf"^{re.escape(name)}:\s*$", text, re.M)
    if not m:
        raise GateError(
            f"just/check.just has no `{name}:` recipe — this gate reads it to check that the "
            "recipe restates no cc-rs fact."
        )
    rest = text[m.end() :]
    nxt = re.search(r"^[A-Za-z_@\[]", rest, re.M)
    return rest[: nxt.start()] if nxt else rest


def problems(sources: str, rs: str, cm: str, just_text: str) -> list[str]:
    bad: list[str] = []

    trees = manifest_trees(sources)
    rs_trees = rust_trees(rs)
    cm_trees = cmake_trees(cm)

    # (2) the shipping lane compiles every tree.
    for missing in sorted(trees - rs_trees):
        bad.append(
            f"build.rs's COMPILED_TREES omits tree `{missing}`, which xrce-sources.txt uses. "
            "The cargo lane compiles EVERY tree — its archive is what both lanes link, so a "
            "tree missing there is a tree missing from that link."
        )
    for extra in sorted(rs_trees - trees):
        bad.append(
            f"build.rs's COMPILED_TREES names tree `{extra}`, which xrce-sources.txt never "
            "uses — dead selection logic, or a manifest that lost its rows."
        )

    # (3) the CMake lane compiles no vendored tree, and cannot name one.
    for vend in sorted(cm_trees & VENDORED):
        bad.append(
            f"CMakeLists.txt's `_xrce_compiled_trees` names vendored tree `{vend}` — that is "
            "the duplicate compile phase-420 W9 step 4 removed. The vendored TUs arrive as "
            "OBJECTS from the cargo lane's archive; only `backend` is compiled here, for its "
            "warnings."
        )
    for extra in sorted(cm_trees - trees):
        bad.append(
            f"CMakeLists.txt's `_xrce_compiled_trees` names tree `{extra}`, which "
            "xrce-sources.txt never uses."
        )
    if not cm_trees:
        bad.append(
            "CMakeLists.txt's `_xrce_compiled_trees` is empty — then its `-Wall -Wextra` "
            "compile of nros-rmw-xrce/src/*.c is gone, which is the only place that "
            "diagnostic runs (the cargo lane sets `warnings(false)`). Say so out loud "
            "before dropping it."
        )
    for root in sorted(set(_CM_TREE_ROOT.findall(cm)) & VENDORED):
        bad.append(
            f"CMakeLists.txt still locates vendored source root `_xrce_tree_{root}`. It "
            "compiles no vendored TU, so it has no business knowing where their `.c` files "
            "are — a root it cannot name is a compile it cannot restore by accident."
        )

    # (1) each declaration is the FILTER, not a note about one.
    if "COMPILED_TREES.contains(" not in rs:
        bad.append(
            "build.rs never reads COMPILED_TREES — the declaration is decorative, so it can "
            "disagree with what the script compiles. Assert row membership against it."
        )
    if "IN_LIST _xrce_compiled_trees" not in cm:
        bad.append(
            "CMakeLists.txt never tests `IN_LIST _xrce_compiled_trees` — the declaration is "
            "decorative, so it can disagree with what the project compiles."
        )

    # (4) the CMake lane links the cargo archive, with no fallback.
    if "IMPORTED" not in cm or "IMPORTED_LOCATION" not in cm:
        bad.append(
            "CMakeLists.txt declares no IMPORTED library — the vendored objects have to "
            "arrive as an imported archive, whose location becomes a link DEPENDENCY of "
            "every consumer. A raw `-Wl,<path>` gets no rebuild edge (issue 0475)."
        )
    if "NROS_XRCE_CFFI_OUT_DIR" not in cm:
        bad.append(
            "CMakeLists.txt never reads NROS_XRCE_CFFI_OUT_DIR, so it cannot be told where "
            "the cargo lane's archive is."
        )
    # The no-fallback guards, checked INSIDE the arm rather than near it.
    #
    # This rule's first version was `NROS_XRCE_CFFI_OUT_DIR` within 400 characters
    # of a `FATAL_ERROR` — shape-valid and wired to nothing: the file mentions the
    # variable several times and FATAL_ERRORs for other reasons, so degrading the
    # unset arm to `message(STATUS …)` left the gate green. Mutation M6 caught it.
    for guard, why in (
        (
            r"if\(NOT NROS_XRCE_CFFI_OUT_DIR\)",
            "when NROS_XRCE_CFFI_OUT_DIR is unset",
        ),
        (
            r'if\(NOT EXISTS "\$\{_xrce_vendor_pointer\}"\)',
            "when the cargo lane's pointer file is absent",
        ),
    ):
        m = re.search(guard + r"([\s\S]*?)endif\(\)", cm)
        if not m:
            bad.append(
                f"CMakeLists.txt has no `{guard}` … `endif()` arm — nothing refuses to "
                f"configure {why}."
            )
        elif "FATAL_ERROR" not in m.group(1):
            bad.append(
                f"CMakeLists.txt does not FATAL_ERROR {why} — the arm is there but it does "
                "not stop the build. Silently falling back to compiling the vendored "
                "sources here would put the duplication back while every symptom said it "
                "was fixed."
            )

    # (5) the pointer file's name and keys: one statement each, and they agree.
    try:
        writer = pointer_facts(
            rs,
            {
                "pointer file name": r'VENDOR_BUILD_POINTER:\s*&str\s*=\s*"([^"]+)"',
                "archive key": r'\b(archive)=\{\}',
                "include key": r'\b(include)=\{\}',
            },
            "build.rs",
        )
        reader = pointer_facts(
            cm,
            {
                "pointer file name": r'set\(_xrce_vendor_pointer\s+"\$\{NROS_XRCE_CFFI_OUT_DIR\}/([^"]+)"\)',
                "archive key": r'_line MATCHES "\^(archive)=',
                "include key": r'_line MATCHES "\^(include)=',
            },
            "CMakeLists.txt",
        )
    except GateError as exc:
        bad.append(str(exc))
    else:
        for fact in writer:
            if writer[fact] != reader[fact]:
                bad.append(
                    f"the two lanes disagree about the {fact}: build.rs writes "
                    f"`{writer[fact]}`, CMakeLists.txt reads `{reader[fact]}`. The reader "
                    "would find nothing and the error would name a missing cargo build."
                )

    # (6) no consumer restates a cc-rs fact.
    #
    # The archive's NAME and the generated-header DIRECTORY belong to the build
    # script; a consumer reads them out of the pointer file or does without.
    # Rules 1-5 are all satisfied by a CMakeLists that reads the pointer file
    # AND then spells `${NROS_XRCE_CFFI_OUT_DIR}/libnros_rmw_xrce_c_inline.a`
    # anyway — perfect shape, mirror restored.
    stems = _RS_ARCHIVE_STEM.findall(rs)
    if len(stems) != 1:
        bad.append(
            f"build.rs states `let archive_stem = \"…\"` {len(stems)} times (expected exactly "
            "one). The archive's name is a cc-rs fact with one home; this gate reads it from "
            "there to check that no consumer repeats it."
        )
    else:
        stem = stems[0]
        for label, text in (("CMakeLists.txt", cm), (f"the `{RECIPE}` recipe", just_text)):
            for spelling in (stem, f"lib{stem}.a"):
                if spelling in text:
                    bad.append(
                        f"{label} names `{spelling}` — that is the cargo lane's archive name, a "
                        "cc-rs fact stated by build.rs. Read it from the pointer file's "
                        "`archive=` line instead. A consumer that spells it keeps the two in "
                        "step only for as long as nobody renames it."
                    )
                    break

    for derived in sorted(set(_CM_OUT_DIR_PATH.findall(cm))):
        if derived == reader_pointer_name(cm):
            continue
        bad.append(
            f"CMakeLists.txt builds `${{NROS_XRCE_CFFI_OUT_DIR}}/{derived}` — the only path this "
            "lane may derive from the cargo lane's OUT_DIR is the pointer file. Everything else "
            "in that directory (the archive, the generated headers) is named BY the pointer "
            "file, so that build.rs stays the one statement of it."
        )
    for derived in sorted(set(_SH_OUT_DIR_PATH.findall(just_text))):
        bad.append(
            f"the `{RECIPE}` recipe builds `$OUT_DIR/{derived}` — it hands the directory over "
            "whole and derives nothing. Naming a file inside it restates a cc-rs fact one file "
            "further out than the CMakeLists does."
        )
    return bad


def reader_pointer_name(cm: str) -> str:
    """The one OUT_DIR-relative path the CMake lane is allowed to build."""
    m = re.search(
        r'set\(_xrce_vendor_pointer\s+"\$\{NROS_XRCE_CFFI_OUT_DIR\}/([^"]+)"\)', cm
    )
    return m.group(1) if m else ""


def self_test() -> None:
    """Runs on the NORMAL path — a negative control nobody runs is a comment."""
    good_sources = (
        "group backend_core always\n"
        "src backend_core backend vtable.c\n"
        "src ucdr ucdr common.c\n"
        "src uxr_session uxr core/session/session.c\n"
    )
    good_rs = (
        f"// {MARK_BEGIN}\n"
        'const COMPILED_TREES: &[&str] = &["uxr", "ucdr", "backend"];\n'
        f"// {MARK_END}\n"
        'const VENDOR_BUILD_POINTER: &str = "nros-xrce-vendor-build.txt";\n'
        "assert!(COMPILED_TREES.contains(&row.tree.as_str()));\n"
        'let archive_stem = "nros_rmw_xrce_c_inline";\n'
        'format!("x\\\n             archive={}\\\n             include={}\\\n", a, b);\n'
    )
    good_just = (
        "rmw-xrce:\n"
        "    cargo build -p nros-rmw-xrce-cffi --message-format=json > \"$JSON\"\n"
        '    cmake -S x -B "$BD" -DNROS_XRCE_CFFI_OUT_DIR="$OUT_DIR"\n'
        "\n"
        "rmw-uorb:\n"
        "    true\n"
    )
    good_cm = (
        f"# {MARK_BEGIN}\n"
        "set(_xrce_compiled_trees backend)\n"
        f"# {MARK_END}\n"
        'set(_xrce_tree_backend "${CMAKE_CURRENT_SOURCE_DIR}/src")\n'
        "if(NOT _rtree IN_LIST _xrce_compiled_trees)\n"
        'set(NROS_XRCE_CFFI_OUT_DIR "" CACHE PATH "x")\n'
        "if(NOT NROS_XRCE_CFFI_OUT_DIR)\n"
        "  message(FATAL_ERROR \"x\")\n"
        "endif()\n"
        'set(_xrce_vendor_pointer "${NROS_XRCE_CFFI_OUT_DIR}/nros-xrce-vendor-build.txt")\n'
        'if(NOT EXISTS "${_xrce_vendor_pointer}")\n'
        '  message(FATAL_ERROR "y")\n'
        'endif()\n'
        'if(_line MATCHES "^archive=(.+)$")\n'
        'elseif(_line MATCHES "^include=(.+)$")\n'
        "add_library(nros_rmw_xrce_vendored STATIC IMPORTED GLOBAL)\n"
        "set_target_properties(nros_rmw_xrce_vendored PROPERTIES IMPORTED_LOCATION x)\n"
    )
    assert problems(good_sources, good_rs, good_cm, good_just) == [], problems(
        good_sources, good_rs, good_cm
    )

    # The SHAPE-VALID cross-wire: swap the two declarations. Every vendored tree
    # is still compiled exactly once in the tree, so a naive count passes — but
    # the archive that ships would hold no XRCE.
    swapped_rs = good_rs.replace(
        'const COMPILED_TREES: &[&str] = &["uxr", "ucdr", "backend"];',
        'const COMPILED_TREES: &[&str] = &["backend"];',
    )
    swapped_cm = good_cm.replace(
        "set(_xrce_compiled_trees backend)",
        "set(_xrce_compiled_trees uxr ucdr backend)",
    ).replace(
        'set(_xrce_tree_backend "${CMAKE_CURRENT_SOURCE_DIR}/src")',
        'set(_xrce_tree_backend "x")\nset(_xrce_tree_uxr "y")\nset(_xrce_tree_ucdr "z")',
    )
    swapped = problems(good_sources, swapped_rs, swapped_cm, good_just)
    assert any("COMPILED_TREES omits tree" in p for p in swapped), swapped
    assert any("names vendored tree" in p for p in swapped), swapped
    assert any("locates vendored source root" in p for p in swapped), swapped

    # A decorative declaration.
    inert = problems(
        good_sources,
        good_rs,
        good_cm.replace("IN_LIST _xrce_compiled_trees", "STREQUAL x"),
        good_just,
    )
    assert any("decorative" in p for p in inert), inert

    # A reader pointed at a file the writer does not write.
    crosswired = problems(
        good_sources,
        good_rs.replace("nros-xrce-vendor-build.txt", "nros-xrce-vendor.txt"),
        good_cm,
        good_just,
    )
    assert any("disagree about the pointer file name" in p for p in crosswired), crosswired

    # No fallback: the FATAL_ERROR arm is what forbids a second compile. The
    # mutation softens ONLY the unset arm and leaves the file's other
    # FATAL_ERRORs standing, because "a FATAL_ERROR somewhere nearby" is exactly
    # the vacuous version of this rule that mutation M6 caught in review.
    soft = problems(
        good_sources,
        good_rs,
        good_cm.replace(
            'if(NOT NROS_XRCE_CFFI_OUT_DIR)\n  message(FATAL_ERROR "x")\n',
            'if(NOT NROS_XRCE_CFFI_OUT_DIR)\n  message(STATUS "x")\n',
        ),
        good_just,
    )
    assert any("does not FATAL_ERROR when NROS_XRCE_CFFI_OUT_DIR is unset" in p for p in soft), soft

    # (6) the restatement mutation review found: PERFECT SHAPE, mirror restored.
    # Every rule above still passes on this input — one declaration per lane,
    # the pointer file still read, each vendored tree still compiled once — and
    # the archive's name is spelled a second time.
    restated = problems(
        good_sources,
        good_rs,
        good_cm.replace(
            "IMPORTED_LOCATION x",
            'IMPORTED_LOCATION "${NROS_XRCE_CFFI_OUT_DIR}/libnros_rmw_xrce_c_inline.a"',
        ),
        good_just,
    )
    assert any("names `nros_rmw_xrce_c_inline" in p for p in restated), restated
    assert any("may derive from the cargo lane's OUT_DIR is the pointer file" in p for p in restated), restated

    # The same restatement one file out, in the recipe.
    in_recipe = problems(
        good_sources,
        good_rs,
        good_cm,
        good_just.replace(
            '-DNROS_XRCE_CFFI_OUT_DIR="$OUT_DIR"',
            '-DNROS_XRCE_ARCHIVE="$OUT_DIR/libnros_rmw_xrce_c_inline.a"',
        ),
    )
    assert any("recipe builds `$OUT_DIR/" in p for p in in_recipe), in_recipe
    assert any("names `nros_rmw_xrce_c_inline" in p for p in in_recipe), in_recipe

    # A generated-header directory derived by a consumer is the same fact.
    hdr = problems(
        good_sources,
        good_rs,
        good_cm.replace(
            "IMPORTED_LOCATION x",
            'INTERFACE_INCLUDE_DIRECTORIES "${NROS_XRCE_CFFI_OUT_DIR}/include"',
        ),
        good_just,
    )
    assert any("include`" in p and "OUT_DIR is the pointer file" in p for p in hdr), hdr

    # And the producer must state the name exactly once.
    twice = problems(
        good_sources,
        good_rs + 'let archive_stem = "other";\n',
        good_cm,
        good_just,
    )
    assert any("states `let archive_stem" in p for p in twice), twice


def main() -> int:
    self_test()

    for p in (SOURCES, BUILD_RS, CMAKE, JUST):
        if not p.exists():
            print(
                f"check-xrce-one-vendored-compile: missing {p.relative_to(REPO)}",
                file=sys.stderr,
            )
            return 1

    try:
        bad = problems(
            SOURCES.read_text(encoding="utf-8"),
            BUILD_RS.read_text(encoding="utf-8"),
            CMAKE.read_text(encoding="utf-8"),
            recipe_body(JUST.read_text(encoding="utf-8"), RECIPE),
        )
    except GateError as exc:
        print(f"check-xrce-one-vendored-compile: {exc}", file=sys.stderr)
        return 1

    if bad:
        print("check-xrce-one-vendored-compile: FAIL", file=sys.stderr)
        for m in bad:
            print(f"  - {m}", file=sys.stderr)
        return 1

    rs_trees = sorted(rust_trees(BUILD_RS.read_text(encoding="utf-8")))
    cm_trees = sorted(cmake_trees(CMAKE.read_text(encoding="utf-8")))
    print(
        "check-xrce-one-vendored-compile: OK — cargo lane compiles "
        f"{rs_trees}, cmake lane compiles {cm_trees} and LINKS the rest; "
        f"vendored trees {sorted(VENDORED)} are compiled once, by the lane that ships"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
