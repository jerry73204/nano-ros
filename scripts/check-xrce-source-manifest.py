#!/usr/bin/env python3
"""One vendored tree, two compilers, ONE source list — issue 1068.

micro-XRCE-DDS-Client + micro-CDR + the in-repo K.2 backend TUs are compiled
twice: by `nros-rmw-xrce-cffi/build.rs` (cc-rs, the Rust lane) and by
`nros-rmw-xrce/CMakeLists.txt` (the C/C++ lane). Until issue 1068 each lane
held its own hand-copied list of the same files, held together by a COMMENT.

They drifted, and the drift was silent. `build.rs` honoured `NROS_LINK_IP=0`
(phase-204.7, the knob a serial-only node uses to shed the IP link) and dropped
`profile/transport/ip/udp/udp_transport{,_posix}.c`; the CMakeLists compiled
them unconditionally. So the knob worked from Rust and did nothing from C/C++,
and the symptom was not a build error — it was a larger image than the user
asked for. The comment asserting the lockstep also named
`packages/rmw/xrce/xrce-sys/build.rs`, a file phase-321 W1.d DELETED, so for two
phases the invariant pointed at nothing.

The fix was to derive the list once (`packages/rmw/xrce/xrce-sources.txt`) and
have both lanes read it. THIS GATE IS WHAT MAKES THAT STICK. Deriving without a
gate is one refactor away from a lane quietly growing a `build.file(...)` again,
and then we are back to a mirror nobody declared.

What it checks
--------------

1. The manifest parses, and is internally consistent: no group declared twice,
   no source naming an undeclared group or an unknown tree, no group with no
   sources, no file listed twice.
2. Every file it lists EXISTS. Vendored trees are skipped LOUDLY when their
   submodule is not checked out; the in-repo `backend` tree is always checked.
3. NEITHER LANE NAMES A SOURCE OF ITS OWN. This is the check that makes the
   mirror unrecreatable: a `.c` path literal in either lane is a file the other
   lane cannot see.
4. Both lanes answer EXACTLY the set of condition tokens the manifest uses —
   no more, no fewer. A token only one lane answers is issue 1068 again, one
   conditional over; a token the manifest never uses is a lane carrying dead
   selection logic.
5. Every `.c` beside the backend sources is in the manifest, or on the
   documented not-compiled list. The inverse drift: a file the tree has and
   neither lane builds.

Run: python3 scripts/check-xrce-source-manifest.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
XRCE = REPO / "packages/rmw/xrce"
MANIFEST = XRCE / "xrce-sources.txt"
# phase-420 W9 — a SECOND manifest reads the same condition vocabulary. The
# `NROS-XRCE-CONDITIONS` block in each lane answers tokens for both, because
# "which files" (`xrce-sources.txt`) and "which profile defines"
# (`xrce-config.txt`) have to agree: a header promising `UCLIENT_PROFILE_UDP`
# whose `udp_transport.c` was not compiled is issue 1068 wearing a link error.
# So check (4)'s "exactly the tokens the manifest uses" is over BOTH.
CONFIG_MANIFEST = XRCE / "xrce-config.txt"
BUILD_RS = XRCE / "nros-rmw-xrce-cffi/build.rs"
CMAKE = XRCE / "nros-rmw-xrce/CMakeLists.txt"

# tree name → root, relative to the repo. Both lanes spell the same three.
TREES = {
    "uxr": "packages/rmw/xrce/xrce-sys/micro-xrce-dds-client/src/c",
    "ucdr": "packages/rmw/xrce/xrce-sys/micro-cdr/src/c",
    "backend": "packages/rmw/xrce/nros-rmw-xrce/src",
}
# Trees that arrive as git submodules — absent in a clone that has not run
# `nros setup --source …`. Their file-existence check skips loudly.
VENDORED = {"uxr", "ucdr"}

# `.c` literals a lane may legitimately name, with the reason.
#
# build.rs also compiles the sibling `nros-platform-posix` C port into its own
# archive (phase-129.NET.3) so a Rust image gets a platform provider. The CMake
# lane reaches the same code a different way — `add_subdirectory()` on that
# project's own CMakeLists — so those three files are NOT a shared list and have
# no business in the XRCE manifest.
LANE_SOURCE_ALLOWLIST = {
    BUILD_RS: {"platform.c", "net.c", "timer.c"},
    CMAKE: set(),
}

# Backend `.c` files that are deliberately compiled by NEITHER lane.
#
# EMPTY, and that is the intended steady state — every backend TU is in
# `xrce-sources.txt`. The mechanism stays because check (5) needs somewhere to
# point: a file the tree grows and neither lane builds must be declared with a
# reason, not merely tolerated. Its only ever entry was
# `transport_zephyr_udp.c`, which issue 1073 DELETED after establishing that
# 129.C.1 had both stopped compiling it (the `platform-zephyr` feature went)
# and switched off the `UCLIENT_PLATFORM_ZEPHYR` its entire body sat behind —
# so it could not have contributed a symbol even if a lane had compiled it.
#
# Adding a row here is a claim about a file that WOULD compile to something.
# A TU whose body is entirely behind an `#if` this repo defines as `never`
# (see `xrce-config.txt`) is not that; it is dead, and belongs deleted.
NOT_COMPILED: dict[str, str] = {}

# The delimited block in each lane that answers the condition tokens.
_BEGIN = "NROS-XRCE-CONDITIONS-BEGIN"
_END = "NROS-XRCE-CONDITIONS-END"
# `"posix_ip" => …` — a match arm in build.rs.
_RS_TOKEN = re.compile(r'"([A-Za-z0-9_]+)"\s*=>')
# `set(_xrce_cond_posix_ip …)` — a variable in CMakeLists.
_CMAKE_TOKEN = re.compile(r"set\s*\(\s*_xrce_cond_([A-Za-z0-9_]+)")

# A `.c` path in a lane. Rust names them as string literals; CMake as bare list
# items. One pattern covers both once comments are stripped.
_C_LITERAL = re.compile(r"[A-Za-z0-9_./-]*[A-Za-z0-9_-]\.c\b")


class ManifestError(Exception):
    pass


def parse_manifest(text: str, where: str = str(MANIFEST)):
    """→ (groups: {name: condition}, rows: [(group, tree, path)]).

    Same grammar both lanes implement: `#` comments, blank lines ignored,
    whitespace-separated columns, two record types.
    """
    groups: dict[str, str] = {}
    rows: list[tuple[str, str, str]] = []
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        f = line.split()
        if f[0] == "group" and len(f) == 3:
            if f[1] in groups:
                raise ManifestError(f"{where}:{n}: group `{f[1]}` declared twice")
            groups[f[1]] = f[2]
        elif f[0] == "src" and len(f) == 4:
            rows.append((f[1], f[2], f[3]))
        else:
            raise ManifestError(
                f"{where}:{n}: expected `group <name> <condition>` or "
                f"`src <group> <tree> <path>`, got `{line}`"
            )
    return groups, rows


def manifest_problems(groups, rows, where: str = str(MANIFEST)) -> list[str]:
    """Internal consistency of the manifest alone — no lanes involved."""
    bad = []
    seen: set[tuple[str, str]] = set()
    used: set[str] = set()
    for group, tree, path in rows:
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
        bad.append(f"{where}: no `src` records at all")
    return bad


def strip_comments(text: str, marker: str) -> str:
    """Drop everything from `marker` to end-of-line, line by line.

    Crude on purpose. It can only ever hide a `.c` that was ALREADY in a
    comment, which is exactly what this gate does not care about; a path in
    compiled position always precedes the comment on its line.
    """
    return "\n".join(line.split(marker, 1)[0] for line in text.splitlines())


def lane_sources(text: str, marker: str, allow: set[str]) -> list[str]:
    """`.c` paths a lane names in compiled position, minus the allowlist."""
    hits = _C_LITERAL.findall(strip_comments(text, marker))
    return sorted({h for h in hits if h.split("/")[-1] not in allow})


def lane_tokens(text: str, pattern: re.Pattern) -> set[str]:
    """Condition tokens a lane answers, read from its delimited block."""
    start = text.find(_BEGIN)
    end = text.find(_END)
    if start < 0 or end < 0 or end < start:
        raise ManifestError(
            f"no {_BEGIN}…{_END} block — the parser and the lane have drifted; "
            "fix the parser rather than deleting the gate"
        )
    return set(pattern.findall(text[start:end]))


def config_conditions(path: Path) -> set[str]:
    """Condition tokens `xrce-config.txt`'s `flag` records use — phase-420 W9.

    Its own gate (`check-xrce-config-manifest`) validates that file; this reads
    only the third column of its `flag` rows, so the two gates share a
    vocabulary without sharing a parser. A missing file contributes nothing
    rather than erroring: the config gate is what reports its absence.
    """
    if not path.exists():
        return set()
    out: set[str] = set()
    for raw in path.read_text(encoding="utf-8").splitlines():
        f = raw.split("#", 1)[0].split()
        if len(f) == 4 and f[0] == "flag":
            out.add(f[3])
    return out


def self_test() -> None:
    """Runs on the NORMAL path — a negative control nobody runs decays into a
    comment (`check-gate-selftests`)."""
    good = (
        "# a comment\n"
        "group core always\n"
        "group ip   posix_ip\n"
        "\n"
        "src core ucdr common.c\n"
        "src ip   uxr  profile/transport/ip/udp/udp_transport.c  # trailing\n"
    )
    groups, rows = parse_manifest(good, "T")
    assert groups == {"core": "always", "ip": "posix_ip"}, groups
    assert rows == [
        ("core", "ucdr", "common.c"),
        ("ip", "uxr", "profile/transport/ip/udp/udp_transport.c"),
    ], rows
    assert manifest_problems(groups, rows, "T") == []

    # Manifest defects each report.
    _, r = parse_manifest("group core always\nsrc nope ucdr common.c\n", "T")
    assert any("no `group` line declares" in m for m in manifest_problems({"core": "always"}, r, "T"))
    g2, r2 = parse_manifest("group core always\nsrc core zzz common.c\n", "T")
    assert any("unknown tree" in m for m in manifest_problems(g2, r2, "T"))
    g3, r3 = parse_manifest("group core always\ngroup dead posix\nsrc core ucdr a.c\n", "T")
    assert any("lists no sources" in m for m in manifest_problems(g3, r3, "T"))
    for broken in ("group core\n", "src a b\n", "banana\n"):
        try:
            parse_manifest(broken, "T")
        except ManifestError:
            pass
        else:  # pragma: no cover
            raise AssertionError(f"parser accepted `{broken!r}`")

    # THE REGRESSION: a lane grows a source of its own.
    assert lane_sources('build.file(uxr.join("udp_transport.c"));', "//", set()) == [
        "udp_transport.c"
    ]
    assert lane_sources("    src/vtable.c\n", "#", set()) == ["src/vtable.c"]
    # ...and it must not fire on a clean lane, on an allowlisted file, or on a
    # path that only appears in prose.
    assert lane_sources("# see udp_transport.c for why\n", "#", set()) == []
    assert lane_sources('posix.join("net.c");', "//", {"net.c"}) == []
    assert lane_sources("list(APPEND _x ${_tree}/${_path})\n", "#", set()) == []

    # The second manifest contributes to the vocabulary (phase-420 W9).
    assert config_conditions(CONFIG_MANIFEST) >= {"never"}, (
        "xrce-config.txt must contribute its flag conditions, or a lane answering "
        "`never` reads as dead selection logic"
    )
    assert (
        config_conditions(Path("/nonexistent/xrce-config.txt")) == set()
    ), "a missing config manifest must contribute nothing, not raise"

    # THE OTHER REGRESSION: the two lanes answer different token sets.
    rs = f'{_BEGIN}\n "always" => true,\n "posix" => p,\n{_END}'
    cm = f"{_BEGIN}\nset(_xrce_cond_always TRUE)\n{_END}"
    assert lane_tokens(rs, _RS_TOKEN) == {"always", "posix"}
    assert lane_tokens(cm, _CMAKE_TOKEN) == {"always"}
    assert lane_tokens(rs, _RS_TOKEN) - lane_tokens(cm, _CMAKE_TOKEN) == {"posix"}
    try:
        lane_tokens("no markers here", _RS_TOKEN)
    except ManifestError:
        pass
    else:  # pragma: no cover
        raise AssertionError("missing-marker block accepted")


def main() -> int:
    self_test()

    bad: list[str] = []
    notes: list[str] = []

    for p in (MANIFEST, BUILD_RS, CMAKE):
        if not p.exists():
            print(f"check-xrce-source-manifest: missing {p.relative_to(REPO)}", file=sys.stderr)
            return 1

    try:
        groups, rows = parse_manifest(MANIFEST.read_text(encoding="utf-8"))
    except ManifestError as e:
        print(f"check-xrce-source-manifest: {e}", file=sys.stderr)
        return 1
    bad += manifest_problems(groups, rows)

    # (2) every listed file exists.
    for tree in sorted(TREES):
        root = REPO / TREES[tree]
        listed = [p for g, t, p in rows if t == tree]
        if not root.is_dir():
            if tree in VENDORED:
                notes.append(
                    f"SKIP tree `{tree}` — {TREES[tree]} not checked out "
                    f"(`nros setup --source …`); {len(listed)} paths unverified"
                )
                continue
            bad.append(f"tree `{tree}` root {TREES[tree]} does not exist")
            continue
        for path in listed:
            if not (root / path).is_file():
                bad.append(f"xrce-sources.txt lists `{tree}/{path}`, which does not exist")

    # (3) neither lane names a source of its own.
    rs_text = BUILD_RS.read_text(encoding="utf-8")
    cm_text = CMAKE.read_text(encoding="utf-8")
    for path, text, marker in (
        (BUILD_RS, rs_text, "//"),
        (CMAKE, cm_text, "#"),
    ):
        for hit in lane_sources(text, marker, LANE_SOURCE_ALLOWLIST[path]):
            bad.append(
                f"{path.relative_to(REPO)} names source `{hit}` — issue 1068: the lanes hold "
                "no source paths, they read packages/rmw/xrce/xrce-sources.txt. Add it there "
                "(or to this gate's allowlist, with the reason it is not shared)."
            )

    # (4) both lanes answer exactly the manifest's condition tokens.
    try:
        rs_tokens = lane_tokens(rs_text, _RS_TOKEN)
        cm_tokens = lane_tokens(cm_text, _CMAKE_TOKEN)
    except ManifestError as e:
        print(f"check-xrce-source-manifest: {e}", file=sys.stderr)
        return 1
    wanted = set(groups.values()) | config_conditions(CONFIG_MANIFEST)
    for label, got in (("build.rs", rs_tokens), ("CMakeLists.txt", cm_tokens)):
        for tok in sorted(wanted - got):
            bad.append(
                f"{label} does not answer condition token `{tok}`, which xrce-sources.txt uses. "
                "A token only one lane answers IS issue 1068, one conditional over."
            )
        for tok in sorted(got - wanted):
            bad.append(
                f"{label} answers condition token `{tok}`, which xrce-sources.txt never uses — "
                "dead selection logic, or a manifest that lost a group."
            )

    # (5) no orphan backend source.
    backend_root = REPO / TREES["backend"]
    if backend_root.is_dir():
        listed = {p for g, t, p in rows if t == "backend"}
        for f in sorted(backend_root.glob("*.c")):
            if f.name not in listed and f.name not in NOT_COMPILED:
                bad.append(
                    f"{f.relative_to(REPO)} is compiled by neither lane — add it to "
                    "xrce-sources.txt, or to this gate's NOT_COMPILED list with the reason."
                )

    for note in notes:
        print(f"check-xrce-source-manifest: {note}")
    if bad:
        print("check-xrce-source-manifest: FAIL", file=sys.stderr)
        for m in bad:
            print(f"  - {m}", file=sys.stderr)
        return 1

    selected = {
        g: len([1 for rg, _, _ in rows if rg == g]) for g in groups
    }
    print(
        "check-xrce-source-manifest: OK — "
        f"{len(rows)} sources in {len(groups)} groups, conditions {sorted(wanted)}, "
        "both lanes derive them "
        + ", ".join(f"{g}:{n}" for g, n in sorted(selected.items()))
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
