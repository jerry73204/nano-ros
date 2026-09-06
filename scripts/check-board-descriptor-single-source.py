#!/usr/bin/env python3
"""A board states its facts once, in `nros-board.toml`.

WHY THIS EXISTS. phase-215 gave one board a second descriptor format: a
`board.cmake` sidecar carrying 14 `NROS_BOARD_*` variables, plus a
`[package.metadata.nros.board]` mirror in Cargo.toml for the CLI to read. Two
faces of the same facts, kept honest by a drift audit.

That audit examined ZERO boards, for two independent reasons, and said OK:

  * it walked top-level `packages/boards/nros-board-*/` dirs needing BOTH files,
    and the only `board.cmake` in the tree sat one level deeper, in a bundle; and
  * no board carried `[package.metadata.nros.board]` at all, so even a fixed
    glob would have hit its "not migrated yet" skip.

RFC-0064 R5 D4 deleted both faces. cmake now reads a MECHANICAL PROJECTION of
the descriptor (`nros board cmake-vars`), regenerated per configure into the
build directory. There is nothing to drift because there is one authored file.

WHAT IT CHECKS. That the second faces stay gone: no tracked `board.cmake`
anywhere, and no `[package.metadata.nros.board]` table in any tracked manifest.
A gate rather than a comment because the pressure that created them is real --
cmake cannot read TOML, and the shortest path from that fact to a working build
is to write the values again in a language cmake speaks.

Exit 0 when both faces are absent, 1 otherwise.
"""

import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# `[package.metadata.nros.board]`, allowing whitespace the way TOML does. A
# mention inside a comment or a doc string is NOT a table header, so the match
# is anchored at line start.
CARGO_FACE_RE = re.compile(r"^\s*\[package\.metadata\.nros\.board[.\]]", re.M)


def tracked(root, *patterns):
    """Tracked paths matching `patterns`.

    `git ls-files`, not a walk: `check-no-tracked-file-find` bans the walk, and
    tracked-only is the correct semantic anyway -- an uncommitted `board.cmake`
    in someone's working tree is their business until they commit it.
    """
    r = subprocess.run(
        ["git", "-C", root, "ls-files", "--", *patterns],
        capture_output=True, text=True, check=False,
    )
    if r.returncode != 0:
        raise SystemExit(
            f"check-board-descriptor-single-source: `git ls-files` failed in "
            f"{root}.\n  {r.stderr.strip()}"
        )
    return [p for p in r.stdout.splitlines() if p.strip()]


def scan(root):
    """(problems, checked) for the tree at `root`."""
    problems = []

    # `zephyr/cmake/nano_ros_use_board.cmake` is the CONSUMER of the projection,
    # not a board's sidecar, so the pattern is a file NAMED board.cmake.
    for path in tracked(root, "*/board.cmake", "board.cmake"):
        if os.path.basename(path) != "board.cmake":
            continue
        problems.append(
            f"{path}: a second, authored descriptor face.\n"
            "    A board states its facts once, in `nros-board.toml`. cmake reads\n"
            "    a projection of that (`nros board cmake-vars`, written into the\n"
            "    build dir per configure), so nothing needs to be written twice.\n"
            "    RFC-0064 R5 D4."
        )

    manifests = tracked(root, "*Cargo.toml")
    for rel in manifests:
        path = os.path.join(root, rel)
        try:
            with open(path, encoding="utf-8", errors="replace") as fh:
                body = fh.read()
        except OSError:
            continue
        if CARGO_FACE_RE.search(body):
            problems.append(
                f"{rel}: `[package.metadata.nros.board]` is the retired Cargo\n"
                "    face of the board descriptor. Put the facts in the board's\n"
                "    `nros-board.toml`; `nros board info <name>` reads them from\n"
                "    there. RFC-0064 R5 D4."
            )
    return problems, len(manifests)


def self_test(quiet=False):
    """Negative control: the rule must FIRE on each face.

    On the normal path, not behind a flag. The gate this replaces was green
    while examining nothing, so a control that only runs when someone remembers
    to ask for it would be repeating the mistake in a smaller way.
    """
    def git(tmp, *args):
        subprocess.run(["git", "-C", tmp, *args], check=True,
                       capture_output=True, text=True)

    with tempfile.TemporaryDirectory() as tmp:
        git(tmp, "init", "-q")
        os.makedirs(os.path.join(tmp, "packages/boards/nros-board-x"))
        open(os.path.join(tmp, "packages/boards/nros-board-x/Cargo.toml"), "w").write(
            '[package]\nname = "x"\n'
        )
        git(tmp, "add", "-A")
        problems, checked = scan(tmp)
        assert checked == 1, f"expected 1 manifest, saw {checked}"
        assert not problems, f"a clean tree must pass; got {problems}"

        # Face 1: a board.cmake sidecar.
        open(os.path.join(tmp, "packages/boards/nros-board-x/board.cmake"), "w").write(
            'set(NROS_BOARD_ZEPHYR_ID "x")\n'
        )
        git(tmp, "add", "-A")
        problems, _ = scan(tmp)
        assert any("board.cmake" in p for p in problems), \
            f"the rule must fire on a board.cmake; got {problems}"

        # Face 2: the Cargo mirror. Note it goes in at BUNDLE depth, which is
        # where the gate this replaces could not look.
        os.makedirs(os.path.join(tmp, "packages/boards/nros-board-x/boards/deep"))
        open(os.path.join(tmp, "packages/boards/nros-board-x/boards/deep/Cargo.toml"), "w").write(
            '[package]\nname = "deep"\n\n[package.metadata.nros.board]\nzephyr_board = "y"\n'
        )
        git(tmp, "add", "-A")
        problems, _ = scan(tmp)
        assert any("package.metadata.nros.board" in p for p in problems), \
            f"the rule must fire on the Cargo face at bundle depth; got {problems}"

        # A MENTION is not a declaration: prose about the retired face, in a
        # comment, must not trip the gate -- this file's own docstring names it,
        # and so do several roadmap docs.
        open(os.path.join(tmp, "packages/boards/nros-board-x/Cargo.toml"), "w").write(
            '[package]\nname = "x"\n# NOTE: no [package.metadata.nros.board] here, deliberately\n'
        )
        os.remove(os.path.join(tmp, "packages/boards/nros-board-x/board.cmake"))
        os.remove(os.path.join(tmp, "packages/boards/nros-board-x/boards/deep/Cargo.toml"))
        git(tmp, "add", "-A")
        problems, _ = scan(tmp)
        assert not problems, f"a comment naming the table must not fire; got {problems}"

    if not quiet:
        print("check-board-descriptor-single-source self-test: OK")
    return 0


def main(argv):
    if "--self-test" in argv:
        return self_test()
    rc = self_test(quiet=True)
    if rc:
        return rc

    problems, checked = scan(ROOT)
    if problems:
        print("check-board-descriptor-single-source: FAILED")
        for p in problems:
            print(f"  - {p}")
        return 1
    print(
        f"check-board-descriptor-single-source: OK ({checked} manifest(s); no "
        "board.cmake, no [package.metadata.nros.board])"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
