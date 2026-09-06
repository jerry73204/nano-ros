#!/usr/bin/env python3
"""Every Zephyr west build-dir name the RUN can produce must be one the manifest models.

Issue 1016. `fixtures::lane::require_west_leaf_in_lane(build_name, …)` is the
whole of the run side's lane narrowing for Zephyr west leaves: it looks
`build_name` up in `fixtures-manifest.py west-leaves` and skips the cell when
the leaf's coordinate is outside the run's lane. On a name it cannot find it
FAILS OPEN — "an unrecognised leaf is a resolver the manifest does not model,
and guessing 'out of lane' there would silently stop running it".

Failing open is the right default and it is also a hole with no floor. A
build-dir name absent from the manifest is, by construction:

  * emitted by `west-leaves` for NO lane, so no lane's fixture build makes it;
  * missed by the lookup, so no lane's run can skip it.

That is issue 0828's shape — the run reaching for a row no lane builds —
arriving as `BuildFailed("Zephyr fixture is STALE …")` or "binary not prebuilt",
which reads character-for-character like a cell that ran and failed. Issue 1016
measured a reader losing six cells to exactly that message and issue 0968
reasoned about three more as failures.

Nineteen such spellings existed when this gate was written, none of them
reachable from a test: the eighteen `zephyr-dds-*-a9` aliases in `decode_alias`
(no `[[fixture]]` row, no recipe, no caller) and `build_zephyr_rust_example_rmw`'s
`build-rs-<case>-<rmw>`, which is the `rs` lang tag issue 0539 retired from both
producers. They are deleted; this keeps the vocabulary a SUBSET of the manifest.

THE RULE — the west build-dir names this crate can produce are a subset of the
names `fixtures-manifest.py west-leaves` emits.

Three producers, because the name reaches the filesystem three ways:

  1. `decode_alias` in `packages/testing/nros-tests/src/zephyr.rs` — the alias
     table, whose arms `build_dir_for_example` turns into
     `build-<lang>-<case>-<rmw>`.
  2. every string literal that spells a west image path (`…/zephyr/zephyr.exe`
     or `.elf`): the component before `/zephyr/` IS the build-dir name. A `{…}`
     placeholder becomes a wildcard, so `build-cortex-m-{}-{}-{}` is checked as
     a SHAPE and `build-rs-{}-{}` matches nothing.
  3. every literal or constant handed to `require_west_leaf_in_lane`.

Doc comments are stripped first: `/// workspace.join("build/zephyr/zephyr.exe")`
is prose, not a resolver.

Vacuity is the failure mode a source-scraping gate has, so each harvest asserts
a non-empty, plausible yield before comparing — a regex that stopped matching
would otherwise report a clean tree forever.

Usage::

    check-west-leaf-vocabulary.py [--selftest]
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ZEPHYR_RS = ROOT / "packages/testing/nros-tests/src/zephyr.rs"
TEST_CRATE = ROOT / "packages/testing/nros-tests"

# `decode_alias`'s arms all have the shape
#     "alias" | "alias" => ("lang", "case", "rmw"),
# possibly wrapped in a block by rustfmt. Only the TUPLE is harvested — the
# alias spellings on the left never reach a build-dir name.
#
# Every field is `[a-z0-9+-]`-shaped rather than narrowly typed: a regex that
# only matches the arms already present would MISS the arm a mutation adds,
# which is the one thing this gate exists to catch (measured — an `rmw` bound of
# `[a-z]+` let a `"cyclonedds-a9"` arm through silently).
_ARM = re.compile(
    r'=>\s*\{?\s*\(\s*"([a-z+]+)"\s*,\s*"([a-z0-9-]+)"\s*,\s*"([a-z0-9-]+)"\s*\)'
)

# A west image path literal. The build-dir name is the component before
# `/zephyr/`; everything left of that is workspace-root prose.
_IMAGE_PATH = re.compile(r'"([^"\n]*)/zephyr/zephyr\.(?:exe|elf)"')

_LITERAL = re.compile(r'require_west_leaf_in_lane\(\s*"([^"\n]+)"')
# …and the constant form, `require_west_leaf_in_lane(ZEPHYR_WORKSPACE_ENTRY_BUILD_DIR, …)`.
# Resolved rather than ignored: a name spelled once in a `const` is still a name
# this resolver can produce, and it was the FIRST of the two spellings to exist.
_LITERAL_CONST = re.compile(r"require_west_leaf_in_lane\(\s*([A-Z][A-Z0-9_]*)\s*,")
_CONST_DEF = re.compile(r'const\s+([A-Z][A-Z0-9_]*)\s*:\s*&\'?[a-z]*\s*str\s*=\s*"([^"]*)"')

# `decode_alias` has ~54 live arms; a harvest far below that means the regex
# stopped matching, not that the table shrank.
_MIN_ARMS = 30
# …and the image-path literals name ~14 entry images plus the two shapes.
_MIN_PATHS = 10


def strip_doc_comments(text: str) -> str:
    """Drop `///` and `//!` lines. Prose is not a resolver."""
    return "\n".join(
        line for line in text.split("\n") if not line.lstrip().startswith(("///", "//!"))
    )


def _slice_decode_alias(text: str) -> str:
    start = text.index("fn decode_alias")
    end = text.index("fn build_dir_for_example", start)
    return text[start:end]


def alias_build_names(text: str) -> set[str]:
    """`build-<lang>-<case>-<rmw>` for every arm of `decode_alias`."""
    return {
        f"build-{lang}-{case}-{rmw}"
        for lang, case, rmw in _ARM.findall(_slice_decode_alias(strip_doc_comments(text)))
    }


def _tracked_rust_sources(root: Path) -> list[Path]:
    """Tracked `*.rs` under `root` — `git ls-files`, never a filesystem walk.

    `check-no-tracked-file-find` forbids the walk and gives the number: 7m36s
    against 0.8s for the same paths, because a walk stats every directory it
    considers pruning. This gate runs on the fast line, so it pays that.
    """
    out = subprocess.run(
        ["git", "ls-files", "-z", "--", str(root.relative_to(ROOT)) + "/**/*.rs"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    return sorted(ROOT / p for p in out.split("\0") if p)


def _sources(root: Path) -> dict[Path, str]:
    return {
        p: strip_doc_comments(p.read_text(encoding="utf-8"))
        for p in _tracked_rust_sources(root)
    }


def _consts(sources: dict[Path, str]) -> dict[str, str]:
    consts: dict[str, str] = {}
    for text in sources.values():
        consts.update(dict(_CONST_DEF.findall(text)))
    return consts


def literal_build_names(root: Path) -> set[str]:
    """Every name handed to `require_west_leaf_in_lane` under `root`."""
    sources = _sources(root)
    consts = _consts(sources)
    names: set[str] = set()
    for text in sources.values():
        names |= set(_LITERAL.findall(text))
        for ident in _LITERAL_CONST.findall(text):
            value = consts.get(ident)
            if value is None:
                raise SystemExit(
                    "check-west-leaf-vocabulary: "
                    f"`require_west_leaf_in_lane({ident}, …)` names a constant this "
                    "gate cannot resolve to a string. Resolve it here or inline the "
                    "literal — an unresolved name is a hole in the vocabulary check."
                )
            names.add(value)
    return names


def image_path_names(text: str) -> set[str]:
    """The build-dir component of every west image path literal in `text`."""
    names = set()
    for prefix in _IMAGE_PATH.findall(text):
        component = prefix.rsplit("/", 1)[-1]
        if component:
            names.add(component)
    return names


def image_path_build_names(root: Path) -> set[str]:
    """[`image_path_names`] over every tracked source under `root`, `{CONST}` resolved."""
    sources = _sources(root)
    consts = _consts(sources)
    names: set[str] = set()
    for text in sources.values():
        names |= image_path_names(text)
    return {resolve_placeholders(n, consts) for n in names}


def resolver_build_names() -> set[str]:
    """The whole vocabulary: alias arms, image paths, and lane-call names.

    THE harvest, so this gate and
    `lane_build_covers_run::every_west_leaf_the_run_can_name_is_built_or_skippable`
    read one list rather than two.
    """
    return (
        alias_build_names(ZEPHYR_RS.read_text(encoding="utf-8"))
        | literal_build_names(TEST_CRATE)
        | image_path_build_names(TEST_CRATE)
    )


def resolver_concrete_names() -> set[str]:
    """[`resolver_build_names`] with every SHAPE expanded to the leaves it matches.

    `build-cortex-m-{}-{}-{}` is one entry in the raw vocabulary and three real
    build directories; a consumer asking "does this lane build it?" needs the
    three. A whole-component placeholder (`{}`) expands to nothing: it
    constrains nothing, and its concrete values come from `decode_alias`, which
    is in the harvest already.

    Exists so the gate and
    `lane_build_covers_run::every_west_leaf_the_run_can_name_is_built_or_skippable`
    ask one question of one list.
    """
    modelled = manifest_build_names()
    out: set[str] = set()
    for name in resolver_build_names():
        if re.fullmatch(r"\{[^{}]*\}", name):
            continue
        if "{" not in name:
            out.add(name)
            continue
        matched = {m for m in modelled if is_modelled(name, {m})}
        # A shape that matches NOTHING must survive as itself. Expanding it to
        # the empty set would DELETE the very case a consumer is looking for —
        # measured: `build-rs-{}-{}` (issue 0539's retired lang tag) vanished
        # from the vocabulary and the binding test went green on the mutation it
        # exists to catch.
        out |= matched or {name}
    return out


def resolve_placeholders(name: str, consts: dict[str, str]) -> str:
    """`{CONST}` -> its value; every other `{…}` stays a placeholder."""
    return re.sub(
        r"\{([A-Za-z_][A-Za-z0-9_]*)\}",
        lambda m: consts.get(m.group(1), m.group(0)),
        name,
    )


def is_modelled(name: str, modelled: set[str]) -> bool:
    """Does `name` (a literal, or a shape with `{…}` wildcards) name a leaf?

    A component that is ENTIRELY a placeholder constrains nothing — that is
    `format!("{}/zephyr/zephyr.exe", build_dir)`, whose value came from
    `decode_alias` and is checked there. Anything else is a shape: `{…}` matches
    one path segment, so `build-cortex-m-{}-{}-{}` matches a real leaf and
    `build-rs-{}-{}` matches nothing.
    """
    if re.fullmatch(r"\{[^{}]*\}", name):
        return True
    if "{" not in name:
        return name in modelled
    pattern = "".join(
        "[^/]+" if part.startswith("{") else re.escape(part)
        for part in re.split(r"(\{[^{}]*\})", name)
        if part
    )
    rx = re.compile(f"^{pattern}$")
    return any(rx.match(m) for m in modelled)


def manifest_build_names() -> set[str]:
    out = subprocess.run(
        [sys.executable, "scripts/build/fixtures-manifest.py", "west-leaves"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    return {line.split("\x1f")[6] for line in out.splitlines() if line}


def _selftest() -> int:
    """The gate must FAIL on a vocabulary the manifest cannot explain."""
    sample = (
        "fn decode_alias(x: &str) {\n"
        "    match x {\n"
        '        "a" => ("rust", "talker", "zenoh"),\n'
        '        "b" => ("c", "listener", "cyclonedds"),\n'
        "    }\n"
        "}\n"
        "fn build_dir_for_example() {}\n"
    )
    got = alias_build_names(sample)
    want = {"build-rust-talker-zenoh", "build-c-listener-cyclonedds"}
    if got != want:
        print(f"selftest: alias harvest {sorted(got)} != {sorted(want)}", file=sys.stderr)
        return 1

    modelled = {"build-rust-talker-zenoh", "build-cortex-m-c-talker-zenoh"}
    checks = [
        ("build-rust-talker-zenoh", True, "a modelled literal"),
        ("build-c-listener-cyclonedds", False, "an unmodelled literal"),
        ("build-cortex-m-{}-{}-{}", True, "a shape that matches a leaf"),
        ("build-rs-{}-{}", False, "the retired `rs` lang-tag shape (issue 0539)"),
        ("{}", True, "a whole-component placeholder constrains nothing"),
    ]
    for name, expect, why in checks:
        if is_modelled(name, modelled) != expect:
            print(f"selftest: {name!r} ({why}) classified wrongly", file=sys.stderr)
            return 1

    doc = '/// let b = j("build/zephyr/zephyr.exe");\nlet c = j("build-x/zephyr/zephyr.exe");\n'
    if image_path_names(strip_doc_comments(doc)) != {"build-x"}:
        print("selftest: doc-comment prose was not stripped", file=sys.stderr)
        return 1

    print("check-west-leaf-vocabulary: selftest OK")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return _selftest()

    aliases = alias_build_names(ZEPHYR_RS.read_text(encoding="utf-8"))
    if len(aliases) < _MIN_ARMS:
        print(
            f"check-west-leaf-vocabulary: harvested only {len(aliases)} build-dir "
            f"name(s) from `decode_alias` in {ZEPHYR_RS.relative_to(ROOT)} "
            f"(expected >= {_MIN_ARMS}). The arm regex has stopped matching, so "
            "this gate is checking nothing — fix the regex, do not lower the "
            "bound.",
            file=sys.stderr,
        )
        return 2

    literals = literal_build_names(TEST_CRATE)
    if not literals:
        print(
            'check-west-leaf-vocabulary: no `require_west_leaf_in_lane("…")` '
            "name found in the test crate. Either the call was renamed or the "
            "lane narrowing for west leaves is gone; both need a look.",
            file=sys.stderr,
        )
        return 2

    paths = image_path_build_names(TEST_CRATE)
    if len(paths) < _MIN_PATHS:
        print(
            f"check-west-leaf-vocabulary: harvested only {len(paths)} west image "
            f"path literal(s) (expected >= {_MIN_PATHS}). The path regex has "
            "stopped matching — fix it, do not lower the bound.",
            file=sys.stderr,
        )
        return 2

    modelled = manifest_build_names()
    candidates = aliases | literals | paths
    unmodelled = sorted(n for n in candidates if not is_modelled(n, modelled))
    if unmodelled:
        print(
            "check-west-leaf-vocabulary: the test harness can name "
            f"{len(unmodelled)} Zephyr west build-dir(s) that "
            "`fixtures-manifest.py west-leaves` does not model.\n"
            "\n"
            "  `require_west_leaf_in_lane` fails OPEN on a name it cannot find, "
            "so each of these is a leaf every lane's RUN demands and no lane's "
            "BUILD produces. The verdict a reader sees is\n"
            '    BuildFailed("Zephyr fixture is STALE …")\n'
            "  which is indistinguishable from a cell that ran and failed "
            "(issues 1016, 0968).\n"
            "\n"
            '  Fix: add the `[[fixture]] builder = "west"` row, or delete the '
            "resolver name.\n",
            file=sys.stderr,
        )
        for name in unmodelled:
            print(f"    {name}", file=sys.stderr)
        return 1

    print(
        f"check-west-leaf-vocabulary: {len(candidates)} resolver build-dir "
        f"name(s)/shape(s), all modelled by {len(modelled)} west leaves."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
