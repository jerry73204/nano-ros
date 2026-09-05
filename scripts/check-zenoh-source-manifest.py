#!/usr/bin/env python3
"""One vendored tree, two compilers, ONE source list — phase-420 W9.

The vendored zenoh-pico C is compiled twice: by `nros-zpico-build`
(`add_zenoh_pico_core_sources`, cc-rs, six platforms) for the cargo lane, and by
`zephyr/cmake/nros_rmw_zenoh.cmake` for the west/cmake lane. Until W9 each lane
held its own copy of the selection — the same nine directory globs, written out
twice — and nothing checked that they agreed. That is the defect issue 1068
fixed next door for micro-XRCE, and the remedy is the same: derive the list once
(`packages/rmw/zenoh/zpico-sys/zenoh-sources.txt`) and have both lanes read it.

THIS GATE IS WHAT MAKES THAT STICK. Deriving without a gate is one refactor away
from a lane quietly regrowing a `file(GLOB_RECURSE …)` or a `src_dir.join("api")`,
and then we are back to a mirror nobody declared.

Directories, not files
----------------------

`xrce-sources.txt` names individual files; this manifest names DIRECTORIES, and
that difference is deliberate (its header argues it). The safety a file list
would buy — a new upstream `.c` cannot land unnoticed — is bought here by
check (6) below instead, which is strictly sharper: a new file in a listed
directory is compiled, which is always what was wanted, while a new DIRECTORY or
a new `system/zephyr/*.c` fails the gate, which is the case that needs a human.

What it checks
--------------

1. The manifest parses, and is internally consistent: no group declared twice,
   no record naming an undeclared group or an unknown tree, no group with no
   records, no path listed twice.
2. Every record is attached to the RIGHT KIND of group — **platform-conditional
   compilation is legitimate only for the platform trees**, so a path under
   `src/system/<platform>/` must be in a CONDITIONAL group and everything else
   (the core) in an unconditional one. This is the only check here that is not
   about shape, and it is the only one that catches a record MOVING between two
   legitimately-declared groups: `dir core … utils` → `dir zephyr_system …
   utils` leaves every count in this gate identical and drops nine `.c` from
   every non-Zephyr build, under the "selected no sources" guard because 125 is
   not 0. See `attachment_problems`.
3. Every directory and file it names EXISTS. Skipped LOUDLY when the zenoh-pico
   submodule is not checked out.
4. NEITHER LANE NAMES A PATH INSIDE THE VENDORED TREE. This is the check that
   makes the mirror unrecreatable, and it is pure text so it holds on a clone
   with no submodules.
5. Both lanes answer EXACTLY the set of condition tokens the manifest uses — no
   more, no fewer. A token only one lane answers is the same defect one
   conditional over; a token the manifest never uses is dead selection logic.
6. Every `.c` in the vendored tree is accounted for: covered by a record, or
   under a `src/system/<platform>/` tree that platform selects for itself, or on
   the documented not-compiled list. The inverse drift — a file the tree has and
   neither lane builds.
7. Both lanes actually READ the manifest.

Run: python3 scripts/check-zenoh-source-manifest.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
ZENOH = REPO / "packages/rmw/zenoh"
MANIFEST = ZENOH / "zpico-sys/zenoh-sources.txt"
CMAKE = REPO / "zephyr/cmake/nros_rmw_zenoh.cmake"
# The cargo lane is two files: `lib.rs` holds the reader and the condition
# block, `runner.rs` the call site. Both are checked for source literals,
# because a regrown list in either is the same regression.
RUST_LIB = ZENOH / "nros-zpico-build/src/lib.rs"
RUST_RUNNER = ZENOH / "nros-zpico-build/src/runner.rs"

# tree name → root, relative to the repo.
TREES = {
    "zenoh_pico": "packages/rmw/zenoh/zpico-sys/zenoh-pico/src",
}
# Trees that arrive as git submodules — absent in a clone that has not run
# `nros setup --source zenoh-pico`. Their existence checks skip loudly.
VENDORED = {"zenoh_pico"}

# `src/system/<platform>/` trees NOT claimed by this manifest, with where they
# ARE claimed. These are the per-platform axis: one declaration per platform,
# read by one lane, so there is no mirror to drift. Zephyr is absent from this
# list on purpose — the cmake lane IS its `extra_sources`, so its files are in
# the manifest and every `.c` under `system/zephyr/` must be accounted for.
PER_PLATFORM_TREES = {
    "arduino": "no nano-ros platform selects it",
    "emscripten": "no nano-ros platform selects it",
    "espidf": "nros-platform-esp-idf, via its own build",
    "flipper": "no nano-ros platform selects it",
    "freertos": "packages/platform/nros-platform-freertos/nros-platform.toml extra_sources",
    "mbed": "no nano-ros platform selects it",
    "rpi_pico": "no nano-ros platform selects it",
    "threadx": "packages/platform/nros-platform-threadx/nros-platform.toml (alias TU + task.c)",
    "unix": "packages/platform/nros-platform-{posix,nuttx}/nros-platform.toml extra_sources",
    "windows": "no nano-ros platform selects it",
}

# Vendored `.c` files deliberately compiled by NEITHER lane.
NOT_COMPILED = {
    # Phase 129 — clock/memory/sleep/random/threading/time on Zephyr come from
    # the alias TU (`zpico-sys/c/zpico/platform_aliases.c`) compiled into the
    # cargo-built staticlib, which forwards each `_z_*` to the canonical
    # `nros_platform_*` ABI. Kept on disk, compiled nowhere; listed here so
    # "nothing builds it" is a recorded fact rather than a silence.
    "system/zephyr/system.c": "phase 129 — replaced by the alias TU",
}

# A path literal a lane may legitimately name, with the reason. Keyed by lane
# file. Empty today: after W9 neither lane spells a vendored path at all.
LANE_PATH_ALLOWLIST: dict[Path, set[str]] = {
    CMAKE: set(),
    RUST_LIB: set(),
    RUST_RUNNER: set(),
}

# The delimited block in each lane that answers the condition tokens.
_BEGIN = "NROS-ZENOH-CONDITIONS-BEGIN"
_END = "NROS-ZENOH-CONDITIONS-END"
# `"zephyr_isotp" => …` — a match arm in lib.rs.
_RS_TOKEN = re.compile(r'"([A-Za-z0-9_]+)"\s*=>')
# `set(_zenoh_cond_zephyr_isotp …)` — a variable in the cmake lane.
_CMAKE_TOKEN = re.compile(r"set\s*\(\s*_zenoh_cond_([A-Za-z0-9_]+)")

# A vendored path as the cmake lane can only spell it.
_CMAKE_VENDOR_PATH = re.compile(r"\$\{ZENOH_PICO_DIR\}/src/[A-Za-z0-9_*./-]+")
# Any double-quoted string in Rust; filtered against the vendored roots after.
_RS_STRING = re.compile(r'"([A-Za-z0-9_*./-]+)"')


class ManifestError(Exception):
    pass


def parse_manifest(text: str, where: str = str(MANIFEST)):
    """→ (groups: {name: condition}, rows: [(kind, group, tree, path)]).

    The same grammar both lanes implement: `#` comments, blank lines ignored,
    whitespace-separated columns, three record types.
    """
    groups: dict[str, str] = {}
    rows: list[tuple[str, str, str, str]] = []
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        f = line.split()
        if f[0] == "group" and len(f) == 3:
            if f[1] in groups:
                raise ManifestError(f"{where}:{n}: group `{f[1]}` declared twice")
            groups[f[1]] = f[2]
        elif f[0] in ("dir", "src") and len(f) == 4:
            rows.append((f[0], f[1], f[2], f[3]))
        else:
            raise ManifestError(
                f"{where}:{n}: expected `group <name> <condition>`, "
                f"`dir <group> <tree> <path>` or `src <group> <tree> <path>`, got `{line}`"
            )
    return groups, rows


# The one condition token that means "every lane, every platform". Named rather
# than inferred, because `attachment_problems` below partitions the manifest on
# it and a second unconditional spelling would silently widen the conditional
# half.
UNCONDITIONAL = "always"


def is_platform_path(path: str) -> bool:
    """Is `path` inside a `src/system/<platform>/` tree?

    `system/common` is NOT one: it is the platform-INDEPENDENT half of
    `src/system/` (`platform.c`, `serial.c`) and is core like any other
    directory. A bare `system` IS one — it would pull every platform's TUs in at
    once, which is the failure this predicate exists to name, not an exemption
    from it.
    """
    seg = path.split("/")
    if seg[0] != "system":
        return False
    return len(seg) == 1 or seg[1] != "common"


def attachment_problems(groups, rows, where: str = str(MANIFEST)) -> list[str]:
    """Is each record attached to the RIGHT group? — the shape-preserving drift.

    THE RULE, in words: **platform-conditional compilation is legitimate only
    for the platform trees.** A path under `src/system/<platform>/` is compiled
    by whichever lane serves that platform, so its group MUST carry a condition;
    everything else is the zenoh-pico core, which every lane on every platform
    compiles, so its group MUST be unconditional. Formally, for every record:

        path is under `src/system/<platform>/`  ⟺  group's condition != `always`

    This is the manifest's own argument turned into a check. Its header says the
    core is a RULE ("the whole core, nine subtrees") and that the only per-file
    decisions live under `src/system/<platform>/`; if that is true, then a core
    directory can never be conditional and a platform TU can never be
    unconditional.

    Why it needs its own check: every other check here is about SHAPE — a group
    that is not declared, a directory that is not listed, a token one lane does
    not answer. Moving a record between two legitimately-declared groups breaks
    no shape. `dir core zenoh_pico utils` → `dir zephyr_system zenoh_pico utils`
    keeps the group count, the record count, the token set and the tree coverage
    all exactly as they were, and drops nine `.c` from every non-Zephyr cargo
    build — under the "selected no sources" guard, because 125 is not 0. The
    mirror direction is as bad and quieter: `src core zenoh_pico
    system/zephyr/network.c` would compile a Zephyr-only TU on every platform.
    """
    bad = []
    for _kind, group, _tree, path in rows:
        cond = groups.get(group)
        if cond is None:  # already reported by `manifest_problems`
            continue
        if is_platform_path(path) and cond == UNCONDITIONAL:
            bad.append(
                f"{where}: `{path}` is a per-platform tree but sits in group `{group}`, "
                f"whose condition is `{UNCONDITIONAL}` — that compiles one platform's TU on "
                "EVERY platform. Move it to a group with a real condition, or, if it is "
                "genuinely platform-independent, it belongs under `system/common`."
            )
        if not is_platform_path(path) and cond != UNCONDITIONAL:
            bad.append(
                f"{where}: `{path}` is zenoh-pico CORE but sits in group `{group}`, whose "
                f"condition is `{cond}` — that silently drops it from every lane that answers "
                f"`{cond}` false, and no count in this gate changes. Core is compiled by every "
                f"lane on every platform: put it in a group whose condition is "
                f"`{UNCONDITIONAL}`. Only `src/system/<platform>/` paths may be conditional."
            )
    return bad


def manifest_problems(groups, rows, where: str = str(MANIFEST)) -> list[str]:
    """Internal consistency of the manifest alone — no lanes involved."""
    bad = []
    seen: set[tuple[str, str]] = set()
    used: set[str] = set()
    for _kind, group, tree, path in rows:
        used.add(group)
        if group not in groups:
            bad.append(f"{where}: `{path}` names group `{group}`, which no `group` line declares")
        if tree not in TREES:
            bad.append(f"{where}: `{path}` names unknown tree `{tree}` (known: {sorted(TREES)})")
        if (tree, path) in seen:
            bad.append(f"{where}: `{tree}/{path}` listed twice")
        seen.add((tree, path))
    for group in sorted(set(groups) - used):
        bad.append(
            f"{where}: group `{group}` declares a condition and lists no sources — "
            "a group both lanes must answer and neither compiles"
        )
    if not rows:
        bad.append(f"{where}: no `dir` or `src` records at all")
    return bad


def strip_comments(text: str, marker: str) -> str:
    """Drop everything from `marker` to end-of-line, line by line.

    Crude on purpose. It can only ever hide a path that was ALREADY in a
    comment, which is exactly what this gate does not care about; a path in
    compiled position always precedes the comment on its line.
    """
    return "\n".join(line.split(marker, 1)[0] for line in text.splitlines())


def strip_test_modules(text: str) -> str:
    """Drop `#[cfg(test)] …  { … }` blocks, brace-balanced.

    A path literal inside a test module is a FIXTURE, not compiled position:
    `nros-zpico-build`'s own manifest-parser tests name `api` and
    `system/zephyr/network.c` on purpose, and a gate that forbade that would
    forbid testing the parser. Balanced rather than cut-to-end-of-file because
    `runner.rs` carries three `#[cfg(test)]` blocks and only one is last.

    Brace counting ignores braces inside string literals; a test module whose
    strings are unbalanced would over-strip, which fails OPEN. That is the same
    trade `strip_comments` makes and for the same reason — the alternative is a
    Rust parser in a 40 ms gate.
    """
    out = []
    i = 0
    while True:
        j = text.find("#[cfg(test)]", i)
        if j < 0:
            out.append(text[i:])
            return "".join(out)
        out.append(text[i:j])
        brace = text.find("{", j)
        if brace < 0:
            return "".join(out)
        depth = 0
        k = brace
        while k < len(text):
            if text[k] == "{":
                depth += 1
            elif text[k] == "}":
                depth -= 1
                if depth == 0:
                    break
            k += 1
        i = k + 1


def vendor_roots(rows) -> set[str]:
    """First path segment of every record — the vendored subtrees, from the
    manifest itself, so the lane check cannot go stale against it."""
    return {path.split("/", 1)[0] for _kind, _group, _tree, path in rows}


def lane_paths(text: str, marker: str, roots: set[str], cmake: bool, allow: set[str]) -> list[str]:
    """Vendored-tree paths a lane names in compiled position, minus the allowlist.

    Pure text: it needs no checkout, which matters because this is the check
    that has to hold on a bare clone.
    """
    body = strip_comments(text, marker)
    if cmake:
        hits = set(_CMAKE_VENDOR_PATH.findall(body))
    else:
        body = strip_test_modules(body)
        hits = {
            s
            for s in _RS_STRING.findall(body)
            if s.split("/", 1)[0] in roots and (s in roots or "/" in s or s.endswith(".c"))
        }
    return sorted(h for h in hits if h not in allow)


def lane_tokens(text: str, pattern: re.Pattern, marker: str) -> set[str]:
    """Condition tokens a lane answers, read from its delimited block."""
    start = text.find(_BEGIN)
    end = text.find(_END)
    if start < 0 or end < 0 or end < start:
        raise ManifestError(
            f"no {_BEGIN}…{_END} block — the parser and the lane have drifted; "
            "fix the parser rather than deleting the gate"
        )
    return set(pattern.findall(strip_comments(text[start:end], marker)))


def coverage_problems(root: Path, rows) -> tuple[list[str], list[str]]:
    """(6) — every `.c` in the vendored tree is accounted for.

    This is what replaces a file list. A `.c` is accounted for when a `dir`
    record covers it, a `src` record names it, it sits under a
    `src/system/<platform>/` tree some platform selects for itself, or it is on
    the documented NOT_COMPILED list.
    """
    dirs = [p for kind, _g, _t, p in rows if kind == "dir"]
    files = {p for kind, _g, _t, p in rows if kind == "src"}
    bad: list[str] = []
    per_platform = 0
    # walk-ok: the subject is a vendored SUBMODULE tree. Its files are not in
    # the superproject's index, so `git ls-files` here returns nothing and the
    # check would pass vacuously on every `.c` upstream adds — which is the one
    # thing this check exists to catch. Scoped to `<zenoh-pico>/src`, ~1200
    # files, and the tree is absent-and-skipped when the submodule is not
    # checked out (see the `root.is_dir()` guard above).
    for f in sorted(root.rglob("*.c")):
        rel = f.relative_to(root).as_posix()
        if rel in files or any(rel.startswith(d + "/") for d in dirs):
            continue
        if rel in NOT_COMPILED:
            continue
        seg = rel.split("/")
        if seg[0] == "system" and len(seg) > 1 and seg[1] in PER_PLATFORM_TREES:
            per_platform += 1
            continue
        bad.append(
            f"zenoh-pico/src/{rel} is compiled by neither lane — add it to "
            f"{MANIFEST.relative_to(REPO)} (or, if it is a whole new "
            "`system/<platform>/` tree, to this gate's PER_PLATFORM_TREES; or to "
            "NOT_COMPILED with the reason)."
        )
    notes = [
        f"{per_platform} `.c` under src/system/<platform>/ left to the per-platform axis "
        f"({', '.join(sorted(PER_PLATFORM_TREES))})"
    ]
    return bad, notes


def self_test() -> None:
    """Runs on the NORMAL path — a negative control nobody runs decays into a
    comment (`check-gate-selftests`)."""
    good = (
        "# a comment\n"
        "group core   always\n"
        "group zeph   zephyr\n"
        "\n"
        "dir core zenoh_pico api\n"
        "src zeph zenoh_pico system/zephyr/network.c  # trailing\n"
    )
    groups, rows = parse_manifest(good, "T")
    assert groups == {"core": "always", "zeph": "zephyr"}, groups
    assert rows == [
        ("dir", "core", "zenoh_pico", "api"),
        ("src", "zeph", "zenoh_pico", "system/zephyr/network.c"),
    ], rows
    assert manifest_problems(groups, rows, "T") == []
    assert attachment_problems(groups, rows, "T") == []
    assert vendor_roots(rows) == {"api", "system"}

    # THE SHAPE-PRESERVING REGRESSION: a record moves between two perfectly
    # legitimate groups. Nothing about the manifest's SHAPE changes — same group
    # count, same record count, same token set, same tree coverage — so every
    # other check here stays green while the build silently loses (or gains)
    # sources.
    assert is_platform_path("system/zephyr/network.c")
    assert is_platform_path("system/unix")
    assert is_platform_path("system")  # every platform at once
    assert not is_platform_path("system/common")
    assert not is_platform_path("system/common/serial.c")
    assert not is_platform_path("utils")
    assert not is_platform_path("api/api.c")

    # core → conditional: nine `.c` vanish from every non-Zephyr build.
    g, r = parse_manifest("group core always\ngroup zeph zephyr\ndir zeph zenoh_pico utils\n", "T")
    msgs = attachment_problems(g, r, "T")
    assert any("is zenoh-pico CORE but sits in group `zeph`" in m for m in msgs), msgs
    assert any("Only `src/system/<platform>/` paths may be conditional" in m for m in msgs), msgs
    # platform → unconditional: a Zephyr-only TU compiled on every platform.
    g, r = parse_manifest("group core always\nsrc core zenoh_pico system/zephyr/network.c\n", "T")
    msgs = attachment_problems(g, r, "T")
    assert any("is a per-platform tree but sits in group `core`" in m for m in msgs), msgs
    # and `system/common` is core, so it is legal exactly where `system/zephyr`
    # is not.
    g, r = parse_manifest("group core always\ndir core zenoh_pico system/common\n", "T")
    assert attachment_problems(g, r, "T") == []
    # an undeclared group is `manifest_problems`' business, not this one — no
    # double report, and no crash on the missing key.
    g, r = parse_manifest("group core always\ndir ghost zenoh_pico utils\n", "T")
    assert attachment_problems(g, r, "T") == []

    # Manifest defects each report.
    _, r = parse_manifest("group core always\ndir nope zenoh_pico api\n", "T")
    assert any(
        "no `group` line declares" in m for m in manifest_problems({"core": "always"}, r, "T")
    )
    g2, r2 = parse_manifest("group core always\ndir core zzz api\n", "T")
    assert any("unknown tree" in m for m in manifest_problems(g2, r2, "T"))
    g3, r3 = parse_manifest("group core always\ngroup dead zephyr\ndir core zenoh_pico api\n", "T")
    assert any("lists no sources" in m for m in manifest_problems(g3, r3, "T"))
    g4, r4 = parse_manifest("group c always\ndir c zenoh_pico api\nsrc c zenoh_pico api\n", "T")
    assert any("listed twice" in m for m in manifest_problems(g4, r4, "T"))
    for broken in ("group core\n", "dir a b\n", "banana\n", "glob core zenoh_pico api\n"):
        try:
            parse_manifest(broken, "T")
        except ManifestError:
            pass
        else:  # pragma: no cover
            raise AssertionError(f"parser accepted `{broken!r}`")

    # THE REGRESSION: a lane regrows a source list of its own.
    roots = {"api", "collections", "system"}
    assert lane_paths(
        'file(GLOB_RECURSE _x "${ZENOH_PICO_DIR}/src/api/*.c")', "#", roots, True, set()
    ) == ["${ZENOH_PICO_DIR}/src/api/*.c"]
    assert lane_paths('src_dir.join("collections")', "//", roots, False, set()) == ["collections"]
    assert lane_paths('build.file(z.join("system/zephyr/network.c"));', "//", roots, False, set()) == [
        "system/zephyr/network.c"
    ]
    # ...and it must not fire on a clean lane, on the tree ROOT, on an
    # allowlisted path, or on a path that only appears in prose.
    assert lane_paths('set(_zenoh_tree_zenoh_pico "${ZENOH_PICO_DIR}/src")', "#", roots, True, set()) == []
    assert lane_paths('zenoh_pico_src.join("include")', "//", roots, False, set()) == []
    assert lane_paths('# see ${ZENOH_PICO_DIR}/src/api/*.c\n', "#", roots, True, set()) == []
    assert lane_paths('src_dir.join("api")', "//", roots, False, {"api"}) == []
    # ...and a fixture inside a test module is not compiled position, while the
    # code AFTER that module still is (runner.rs has three, mid-file).
    assert (
        lane_paths(
            '#[cfg(test)]\nmod t {\n  fn f() { g("api"); }\n}\nfn real() {}\n',
            "//",
            roots,
            False,
            set(),
        )
        == []
    )
    assert lane_paths(
        '#[cfg(test)]\nmod t { fn f() { g("api"); } }\nfn real() { h("collections"); }\n',
        "//",
        roots,
        False,
        set(),
    ) == ["collections"]

    # THE OTHER REGRESSION: the two lanes answer different token sets.
    rs = f'{_BEGIN}\n "always" => true,\n "zephyr" => z,\n{_END}'
    cm = f"{_BEGIN}\nset(_zenoh_cond_always TRUE)\n{_END}"
    assert lane_tokens(rs, _RS_TOKEN, "//") == {"always", "zephyr"}
    assert lane_tokens(cm, _CMAKE_TOKEN, "#") == {"always"}
    assert lane_tokens(rs, _RS_TOKEN, "//") - lane_tokens(cm, _CMAKE_TOKEN, "#") == {"zephyr"}
    try:
        lane_tokens("no markers here", _RS_TOKEN, "//")
    except ManifestError:
        pass
    else:  # pragma: no cover
        raise AssertionError("missing-marker block accepted")


def main() -> int:
    self_test()
    if "--self" in sys.argv[1:]:
        print("check-zenoh-source-manifest: self-test OK")
        return 0

    bad: list[str] = []
    notes: list[str] = []

    lanes = (CMAKE, RUST_LIB, RUST_RUNNER)
    for p in (MANIFEST, *lanes):
        if not p.exists():
            print(f"check-zenoh-source-manifest: missing {p.relative_to(REPO)}", file=sys.stderr)
            return 1

    try:
        groups, rows = parse_manifest(MANIFEST.read_text(encoding="utf-8"))
    except ManifestError as e:
        print(f"check-zenoh-source-manifest: {e}", file=sys.stderr)
        return 1
    bad += manifest_problems(groups, rows)
    bad += attachment_problems(groups, rows)

    texts = {p: p.read_text(encoding="utf-8") for p in lanes}

    # (3) every listed directory / file exists, and (6) coverage.
    for tree in sorted(TREES):
        root = REPO / TREES[tree]
        listed = [(k, p) for k, _g, t, p in rows if t == tree]
        if not root.is_dir():
            if tree in VENDORED:
                notes.append(
                    f"SKIP tree `{tree}` — {TREES[tree]} not checked out "
                    f"(`nros setup --source zenoh-pico`); {len(listed)} records unverified, "
                    "and the tree-coverage check with them"
                )
                continue
            bad.append(f"tree `{tree}` root {TREES[tree]} does not exist")
            continue
        for kind, path in listed:
            target = root / path
            if kind == "dir" and not target.is_dir():
                bad.append(
                    f"{MANIFEST.name} lists directory `{tree}/{path}`, which is not a directory"
                )
            elif kind == "src" and not target.is_file():
                bad.append(f"{MANIFEST.name} lists source `{tree}/{path}`, which does not exist")
        cov_bad, cov_notes = coverage_problems(root, rows)
        bad += cov_bad
        notes += cov_notes

    # (4) neither lane names a path inside the vendored tree.
    roots = vendor_roots(rows)
    for path in lanes:
        is_cmake = path == CMAKE
        for hit in lane_paths(
            texts[path], "#" if is_cmake else "//", roots, is_cmake, LANE_PATH_ALLOWLIST[path]
        ):
            bad.append(
                f"{path.relative_to(REPO)} names vendored path `{hit}` — phase-420 W9: the "
                f"lanes hold no zenoh-pico paths, they read {MANIFEST.relative_to(REPO)}. Add it "
                "there (or to this gate's LANE_PATH_ALLOWLIST, with the reason it is not shared)."
            )

    # (5) both lanes answer exactly the manifest's condition tokens.
    try:
        rs_tokens = lane_tokens(texts[RUST_LIB], _RS_TOKEN, "//")
        cm_tokens = lane_tokens(texts[CMAKE], _CMAKE_TOKEN, "#")
    except ManifestError as e:
        print(f"check-zenoh-source-manifest: {e}", file=sys.stderr)
        return 1
    wanted = set(groups.values())
    for label, got in (("nros-zpico-build/src/lib.rs", rs_tokens), (CMAKE.name, cm_tokens)):
        for tok in sorted(wanted - got):
            bad.append(
                f"{label} does not answer condition token `{tok}`, which {MANIFEST.name} uses. "
                "A token only one lane answers is the same defect one conditional over."
            )
        for tok in sorted(got - wanted):
            bad.append(
                f"{label} answers condition token `{tok}`, which {MANIFEST.name} never uses — "
                "dead selection logic, or a manifest that lost a group."
            )

    # (7) both lanes actually read the manifest.
    if MANIFEST.name not in texts[CMAKE]:
        bad.append(f"{CMAKE.relative_to(REPO)} never names {MANIFEST.name} — it reads no manifest")
    if MANIFEST.name not in texts[RUST_LIB] + texts[RUST_RUNNER]:
        bad.append(f"the cargo lane never names {MANIFEST.name} — it reads no manifest")

    for note in notes:
        print(f"check-zenoh-source-manifest: {note}")
    if bad:
        print("check-zenoh-source-manifest: FAIL", file=sys.stderr)
        for m in bad:
            print(f"  - {m}", file=sys.stderr)
        return 1

    kinds = {k: len([1 for kk, _g, _t, _p in rows if kk == k]) for k in ("dir", "src")}
    print(
        "check-zenoh-source-manifest: OK — "
        f"{kinds['dir']} directories + {kinds['src']} files in {len(groups)} groups, "
        f"conditions {sorted(wanted)}, both lanes derive them"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
