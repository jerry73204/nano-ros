#!/usr/bin/env python3
"""Every west build-dir name the RUN can name must be one the manifest models.

Issue 1016. `fixtures::lane::require_west_leaf_in_lane(build_name, …)` is the
whole of the run side's lane narrowing for Zephyr west leaves: it looks
`build_name` up in `fixtures-manifest.py west-leaves` and skips the cell when
the leaf's coordinate is outside the run's lane. On a name it cannot find it
FAILS OPEN — "an unrecognised leaf is a resolver the manifest does not model,
and guessing 'out of lane' there would silently stop running it".

Failing open is the right default and it is also a hole with no floor. A build
name absent from the manifest is, by construction:

  * never emitted by `west-leaves`, so NO lane's fixture build produces it; and
  * never skippable by the run, because the lookup misses.

That is issue 0828's shape — the run reaching for a row the lane never built —
arriving as `BuildFailed("Zephyr fixture is STALE …")`, which reads
character-for-character like a cell that ran and failed. Issue 1016 measured a
reader losing six cells to exactly that message, and issue 0968 reasoned about
them as failures.

Eighteen such names existed when this gate was written: the `zephyr-dds-*-a9`
aliases in `decode_alias`, which no test called, no `[[fixture]]` row modelled
and no recipe built. They are deleted; this keeps the vocabulary a subset.

The two halves of that vocabulary:

  * `decode_alias` in `packages/testing/nros-tests/src/zephyr.rs` — the alias
    table, whose arms `build_dir_for_example` turns into
    `build-<lang>-<case>-<rmw><suffix>`.
  * every LITERAL passed to `require_west_leaf_in_lane` anywhere in the test
    crate — `build-ws-rs-entry-zenoh`, `build-logging-smoke`, …

Vacuity is the failure mode a source-scraping gate has, so both halves assert a
non-empty, plausible harvest before they compare: a regex that stops matching
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
#     "alias" | "alias" => ("lang", "case", "rmw", "suffix"),
# possibly wrapped in a block by rustfmt. Only the TUPLE is harvested — the
# alias spellings on the left do not reach a build-dir name.
_ARM = re.compile(
    r'=>\s*\{?\s*\(\s*"([a-z+]+)"\s*,\s*"([a-z0-9-]+)"\s*,\s*"([a-z]+)"\s*,\s*"(-?[a-z0-9]*)"\s*\)'
)

_LITERAL = re.compile(r'require_west_leaf_in_lane\(\s*"([a-z0-9-]+)"')

# …and the constant form, `require_west_leaf_in_lane(ZEPHYR_WORKSPACE_ENTRY_BUILD_DIR, …)`.
# Resolved rather than ignored: a name spelled once in a `const` is still a name
# this resolver can produce, and it was the FIRST of the two spellings to exist.
_LITERAL_CONST = re.compile(r"require_west_leaf_in_lane\(\s*([A-Z][A-Z0-9_]*)\s*,")
_CONST_DEF = re.compile(r'const\s+([A-Z][A-Z0-9_]*)\s*:\s*&\'?[a-z]*\s*str\s*=\s*"([^"]*)"')

# `decode_alias` has ~54 live arms; a harvest far below that means the regex
# stopped matching, not that the table shrank.
_MIN_ARMS = 30


def _slice_decode_alias(text: str) -> str:
    start = text.index("fn decode_alias")
    end = text.index("fn build_dir_for_example", start)
    return text[start:end]


def alias_build_names(text: str) -> set[str]:
    """`build-<lang>-<case>-<rmw><suffix>` for every arm of `decode_alias`."""
    return {
        f"build-{lang}-{case}-{rmw}{suffix}"
        for lang, case, rmw, suffix in _ARM.findall(_slice_decode_alias(text))
    }


def literal_build_names(root: Path) -> set[str]:
    """Every constant name handed to `require_west_leaf_in_lane` in the crate."""
    sources = {p: p.read_text(encoding="utf-8") for p in sorted(root.rglob("*.rs"))}
    consts: dict[str, str] = {}
    for text in sources.values():
        consts.update(dict(_CONST_DEF.findall(text)))
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
        'fn decode_alias(x: &str) {\n'
        '    match x {\n'
        '        "a" => ("rust", "talker", "zenoh", ""),\n'
        '        "b" => ("c", "listener", "cyclonedds", "-a9"),\n'
        '    }\n'
        '}\n'
        'fn build_dir_for_example() {}\n'
    )
    got = alias_build_names(sample)
    want = {"build-rust-talker-zenoh", "build-c-listener-cyclonedds-a9"}
    if got != want:
        print(f"selftest: alias harvest {sorted(got)} != {sorted(want)}", file=sys.stderr)
        return 1
    modelled = {"build-rust-talker-zenoh"}
    if not (got - modelled):
        print("selftest: an unmodelled name was not detected", file=sys.stderr)
        return 1
    print("check-west-leaf-vocabulary: selftest OK")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return _selftest()

    text = ZEPHYR_RS.read_text(encoding="utf-8")
    aliases = alias_build_names(text)
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
            "check-west-leaf-vocabulary: no `require_west_leaf_in_lane(\"…\")` "
            "literal found in the test crate. Either the call was renamed or the "
            "lane narrowing for west leaves is gone; both need a look.",
            file=sys.stderr,
        )
        return 2

    modelled = manifest_build_names()
    unmodelled = sorted((aliases | literals) - modelled)
    if unmodelled:
        print(
            "check-west-leaf-vocabulary: the test harness can name "
            f"{len(unmodelled)} Zephyr west build-dir(s) that "
            "`fixtures-manifest.py west-leaves` does not model.\n"
            "\n"
            "  `require_west_leaf_in_lane` fails OPEN on a name it cannot find, "
            "so each of these is a leaf every lane's RUN demands and no lane's "
            "BUILD produces. The verdict a reader sees is\n"
            '    BuildFailed(\"Zephyr fixture is STALE …\")\n'
            "  which is indistinguishable from a cell that ran and failed "
            "(issues 1016, 0968).\n"
            "\n"
            "  Fix: add the `[[fixture]] builder = \"west\"` row, or delete the "
            "resolver name.\n",
            file=sys.stderr,
        )
        for name in unmodelled:
            print(f"    {name}", file=sys.stderr)
        return 1

    print(
        f"check-west-leaf-vocabulary: {len(aliases | literals)} resolver "
        f"build-dir name(s), all modelled by {len(modelled)} west leaves."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
