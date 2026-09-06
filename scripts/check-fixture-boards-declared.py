#!/usr/bin/env python3
"""Every Zephyr board this tree BUILDS is a board this tree DECLARES.

WHY THIS EXISTS. phase-375 W6 measured four different ways to add a board, and
the fourth was the one with no record at all: `examples/fixtures.toml` naming a
raw Zephyr board id as a string, with no package, no descriptor, no
announcement and no registry row. Three Zephyr board ids were built on
2026-09-06 and exactly ONE was declared anywhere -- and that one only because
its id had been wedged into the `zephyr` descriptor's `names` array beside a
real name, for want of anywhere else to put it.

An undeclared board is not a small omission. It cannot be given a tier, an
owner, or a demotion; `check-board-tiers` cannot see it, `nros board list` does
not list it, and the per-board facts that DO exist end up spread across the
fixture rows that use it. The board that took this route (`qemu_cortex_a53`)
had been in the tree for months.

WHAT IT CHECKS. Every `board = "..."` in a `[[fixture]]` row whose platform is
Zephyr must equal some board descriptor's `[board.zephyr] west_board`.

WHAT IT DELIBERATELY DOES NOT CHECK. That `board =` names the board KEY rather
than the Zephyr id. It stays the id on purpose: the value is also passed to
`west build -b` and appears in west ids and artifact paths, so re-spelling it is
a rename of build outputs, not a schema improvement. Declaring the board is the
part that was missing.

Buildless -- TOML plus a regex, no cmake, no cargo.
"""

import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIXTURES = os.path.join(ROOT, "examples", "fixtures.toml")

BOARD_GLOBS = [
    "packages/boards/*/nros-board.toml",
    "packages/boards/*/boards/*/nros-board.toml",
]

# `west_board = "<id>"` inside a `[board.zephyr]` block. A regex rather than a
# TOML parse for the reason `fixtures-manifest.py` records: CI hosts are not
# guaranteed a TOML library on this repo's Python.
WEST_BOARD_RE = re.compile(r'^\s*west_board\s*=\s*"([^"]+)"', re.M)
# Historical spelling: the `zephyr` descriptor still carries its board id in
# `names`, from before `[board.zephyr]` existed.
NAMES_RE = re.compile(r"^\s*names\s*=\s*\[([^\]]*)\]", re.M)


def declared_boards(root):
    """Every Zephyr board id any descriptor claims."""
    out = set()
    for pattern in BOARD_GLOBS:
        for path in glob.glob(os.path.join(root, pattern)):
            with open(path, encoding="utf-8") as fh:
                body = fh.read()
            out.update(WEST_BOARD_RE.findall(body))
            for group in NAMES_RE.findall(body):
                for raw in group.split(","):
                    name = raw.strip().strip('"').strip("'")
                    # Only a name SHAPED like a Zephyr board id counts here: a
                    # board id has a `/` or is a bare soc name. Taking every
                    # name would make this gate unfalsifiable.
                    if name and ("/" in name or "_" in name):
                        out.add(name)
    return out


def built_boards(path):
    """Every `board = "..."` in the fixture manifest."""
    if not os.path.isfile(path):
        return {}
    seen = {}
    with open(path, encoding="utf-8") as fh:
        for lineno, line in enumerate(fh, 1):
            m = re.match(r'^board\s*=\s*"([^"]+)"', line)
            if m:
                seen.setdefault(m.group(1), lineno)
    return seen


def scan(root, fixtures_path):
    declared = declared_boards(root)
    built = built_boards(fixtures_path)
    problems = []
    for board, lineno in sorted(built.items()):
        if board not in declared:
            problems.append(
                f"examples/fixtures.toml:{lineno}: board {board!r} is BUILT but "
                f"declared by no board package.\n"
                f"    A board with no descriptor cannot be given a tier, an "
                f"owner or a demotion, and `check-board-tiers` cannot see it.\n"
                f"    Create one:  just board-new <key> --platform zephyr "
                f"--west-board {board}"
            )
    return problems, len(built), len(declared)


def self_test(quiet=False):
    """The rule must FIRE on a built-but-undeclared board.

    On the normal path, not behind a flag: this gate exists because an
    invariant nobody checked was false for months.
    """
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        os.makedirs(os.path.join(tmp, "packages/boards/nros-board-x"))
        with open(os.path.join(tmp, "packages/boards/nros-board-x/nros-board.toml"), "w") as fh:
            fh.write('[[board]]\nnames = ["x"]\n\n[board.zephyr]\nwest_board = "plank/soc/smp"\n')
        fixtures = os.path.join(tmp, "fixtures.toml")

        # Declared board: silent.
        with open(fixtures, "w") as fh:
            fh.write('[[fixture]]\nboard = "plank/soc/smp"\n')
        problems, built, declared = scan(tmp, fixtures)
        assert declared == 1, f"expected 1 declared board, saw {declared}"
        assert not problems, f"a declared board must pass; got {problems}"

        # Undeclared board: fires, and names the fix.
        with open(fixtures, "w") as fh:
            fh.write('[[fixture]]\nboard = "ghost/soc/smp"\n')
        problems, _, _ = scan(tmp, fixtures)
        assert len(problems) == 1, f"expected 1 problem, got {problems}"
        assert "just board-new" in problems[0], "the message must name the fix"

        # A BUNDLE board one level deeper is declared too -- the depth that
        # every other board gate had to learn (phase-375 W6).
        os.makedirs(os.path.join(tmp, "packages/boards/nros-board-fam/boards/deep"))
        with open(
            os.path.join(tmp, "packages/boards/nros-board-fam/boards/deep/nros-board.toml"), "w"
        ) as fh:
            fh.write('[[board]]\nnames = ["deep"]\n\n[board.zephyr]\nwest_board = "ghost/soc/smp"\n')
        problems, _, _ = scan(tmp, fixtures)
        assert not problems, f"a bundle-depth declaration must count; got {problems}"

    if not quiet:
        print("check-fixture-boards-declared self-test: OK")
    return 0


def main(argv):
    if "--self-test" in argv:
        return self_test()
    rc = self_test(quiet=True)
    if rc:
        return rc

    problems, built, declared = scan(ROOT, FIXTURES)
    if problems:
        print("check-fixture-boards-declared: FAILED")
        for p in problems:
            print(f"  - {p}")
        return 1
    if built == 0:
        print(
            "check-fixture-boards-declared: no `board =` row matched — refusing "
            "to pass on an empty set"
        )
        return 1
    print(
        f"check-fixture-boards-declared: OK ({built} board(s) built, all declared; "
        f"{declared} declared in total)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
