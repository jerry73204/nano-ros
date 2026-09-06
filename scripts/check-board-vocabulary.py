#!/usr/bin/env python3
"""Every `deploy=` and `board=` a package.xml exports must resolve somewhere.

phase-422 W7. `<export><nano_ros deploy=".." board=".."/></export>` is how a
workspace states where it deploys, and `nros setup --workspace` turns that into
a provisioning command. That only works if the values mean something.

WHAT THE VALUES ACTUALLY MEAN, measured rather than assumed. `board=` is the
CMAKE BOARD vocabulary: all five values across `examples/` resolve to a
`cmake/board/nano-ros-board-<name>.cmake` file.

They used to resolve to NOTHING ELSE. When this gate was written only
`threadx-linux` was also an index key, so `nros setup <board>` failed for four
of five and `nros setup --workspace` had to validate before printing a command.
W7's additive half closed that: `mps2-an385-freertos`, `nuttx-qemu-arm`,
`nuttx-qemu-riscv` and `riscv64-qemu` are `[board.*]` entries now, each marked
`# = [board.<other-spelling>]` against the entry it duplicates. All five values
resolve in BOTH namespaces, which is what lets the second assertion below —
`board=` must be an index key — be enforced rather than aspired to.

Five namespaces exist for closely related concepts, overlapping partially:

  1. `cmake/board/nano-ros-board-*.cmake` — what `board=` names (canonical here)
  2. `[board.*]` in nros-sdk-index.toml   — what `nros setup <board>` accepts
  3. `packages/boards/nros-board-*`       — the board crate
  4. `examples/fixtures.toml` NANO_ROS_BOARD — the test coordinate
  5. `_NROS_SCOPE_PLATFORMS`              — what `just setup <scope>` accepts

This gate does NOT unify them. Which becomes canonical across all five is a
decision phase-422 W7 leaves to a human, and renaming boards through 90+
package.xml files is not something to do as a side effect. What it stops is a
value resolving in NONE of them — the case where a printed remedy fails and a
user pays for it.

Three reader bugs were found by RUNNING this gate rather than reasoning about
it, each one reporting well-defined boards as undefined: fixtures.toml spells
the board `NANO_ROS_BOARD = ".."` inside `cmake_defs`, not `board = ".."`; and
the cmake board files were missing from the namespace list entirely. A gate
that is wrong about where names live sends someone renaming working examples.

Run:  python3 scripts/check-board-vocabulary.py [--self-test]
"""

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def exports(text):
    """[(deploy, board)] from `<nano_ros deploy=".." board=".."/>` tags."""
    out = []
    for m in re.finditer(r"<nano_ros\s([^>]*)>", text):
        tag = m.group(1)
        d = re.search(r'deploy="([^"]*)"', tag)
        if not d:
            continue
        b = re.search(r'board="([^"]*)"', tag)
        out.append((d.group(1), b.group(1) if b else None))
    return out


def index_boards(root):
    try:
        with open(os.path.join(root, "nros-sdk-index.toml"), encoding="utf8") as fh:
            return set(re.findall(r"^\[board\.([a-z0-9._-]+)\]", fh.read(), re.M))
    except OSError:
        return set()


def board_crates(root):
    d = os.path.join(root, "packages", "boards")
    try:
        return {n[len("nros-board-") :] for n in os.listdir(d) if n.startswith("nros-board-")}
    except OSError:
        return set()


def fixture_boards(root):
    """Boards named by the fixture matrix.

    NOT a bare `board = "..."`: fixtures.toml carries the board inside a cmake
    definition table, `cmake_defs = { NANO_ROS_BOARD = "nuttx-qemu-arm", .. }`.
    Reading the wrong spelling made this gate report three real, well-defined
    boards as resolving nowhere — a false positive that would have sent someone
    renaming working examples.
    """
    boards = set()
    for rel in [("examples", "fixtures.toml")]:
        try:
            with open(os.path.join(root, *rel), encoding="utf8") as fh:
                t = fh.read()
        except OSError:
            continue
        boards |= set(re.findall(r'NANO_ROS_BOARD\s*=\s*"([^"]+)"', t))
        boards |= set(re.findall(r'^\s*board\s*=\s*"([^"]+)"', t, re.M))
    return boards


def cmake_boards(root):
    """Boards defined by `cmake/board/nano-ros-board-<name>.cmake`.

    This is what `board=` in a package.xml export actually names, and all five
    values in this repo resolve here. They are ALSO index keys now (W7's
    additive half); this namespace stays in the OR because it is the one the
    BUILD uses, and a value that resolved only here would still be a real board
    — just not a provisionable one, which the stricter check below reports
    separately.
    """
    d = os.path.join(root, "cmake", "board")
    try:
        names = os.listdir(d)
    except OSError:
        return set()
    out = set()
    for n in names:
        m = re.fullmatch(r"nano-ros-board-(.+)\.cmake", n)
        if m:
            out.add(m.group(1))
    return out


def board_entries(root):
    """{key: {arch, platform, packages}} for every `[board.*]`, plus its `# =` pair.

    A deliberately small parser: these entries are three scalar/array fields and
    a section header, and pulling in a TOML dependency to read them would be the
    larger change (the same reasoning `prereq_resolve::depend_names` records).
    """
    try:
        with open(os.path.join(root, "nros-sdk-index.toml"), encoding="utf8") as fh:
            text = fh.read()
    except OSError:
        return {}
    out, pair = {}, {}
    cur = None
    pending_pair = None
    for line in text.split("\n"):
        st = line.strip()
        m = re.match(r"^#\s*=\s*\[board\.([a-z0-9._-]+)\]", st)
        if m:
            pending_pair = m.group(1)
            continue
        h = re.match(r"^\[board\.([a-z0-9._-]+)\]$", st)
        if h:
            cur = h.group(1)
            out[cur] = {}
            if pending_pair:
                pair[cur] = pending_pair
                pending_pair = None
            continue
        if st.startswith("["):
            cur = None
            pending_pair = None
            continue
        if cur is None or st.startswith("#") or "=" not in st:
            continue
        k, _, v = st.partition("=")
        k, v = k.strip(), v.strip()
        if k in ("arch", "platform"):
            # Take the QUOTED value, never the rest of the line. These entries
            # carry trailing comments (`platform = "nuttx" # NuttX kernel/apps
            # are submodule SDKs`), and reading the raw remainder made this gate
            # report two IDENTICAL pairs as diverged — a false positive that
            # would have sent someone "fixing" correct data.
            q = re.match(r'"([^"]*)"', v)
            out[cur][k] = q.group(1) if q else v.split("#", 1)[0].strip().strip('"')
        elif k == "packages":
            # Same hazard: only read inside the brackets, so a trailing comment
            # containing a quoted word cannot be mistaken for a package.
            arr = v.split("]", 1)[0]
            out[cur][k] = re.findall(r'"([^"]+)"', arr)
    return out, pair


def scopes(root):
    try:
        with open(os.path.join(root, "scripts", "build", "scope.sh"), encoding="utf8") as fh:
            t = fh.read()
    except OSError:
        return set()
    m = re.search(r'_NROS_SCOPE_PLATFORMS="([^"]*)"', t)
    return set(m.group(1).split()) if m else set()


def self_test():
    got = exports('<nano_ros deploy="threadx" board="riscv64-qemu" rmw="zenoh"/>')
    assert got == [("threadx", "riscv64-qemu")], got
    assert exports('<nano_ros deploy="native"/>') == [("native", None)]
    # Siblings are not deploy exports.
    assert exports('<nano_ros_provides kind="board" name="threadx"/>') == []
    sys.stdout.write("check-board-vocabulary self-test: OK\n")


def main():
    if "--self-test" in sys.argv:
        self_test()
        return 0
    self_test()

    files = subprocess.run(
        ["git", "ls-files", "*package.xml"], cwd=ROOT, capture_output=True, text=True
    ).stdout.split()
    idx, crates, fixtures, scope_set, cmakeb = (
        index_boards(ROOT),
        board_crates(ROOT),
        fixture_boards(ROOT),
        scopes(ROOT),
        cmake_boards(ROOT),
    )
    if not scope_set or not idx:
        sys.stderr.write(
            "error: could not read the scope list or the index boards.\n"
            "This gate would then accept anything and pass vacuously.\n"
        )
        return 1

    bad_board, bad_deploy, not_index = {}, {}, {}

    # phase-422 W7 — the DUPLICATE-PAIR assertion. The index carries four
    # entries that duplicate a counterpart under the other spelling, marked
    # `# = [board.X]`, and its comment told readers to EDIT BOTH while claiming
    # this gate asserted the pairing. It did not: it only ever regexed section
    # NAMES. That is the issue-0196 class — a gate credited with a rule wider
    # than the one it enforces — so the assertion is added rather than the claim
    # softened.
    #
    # Driven off the `# =` markers, so a pair opts IN by being marked. The
    # `native`/`posix` entries are byte-identical and deliberately NOT a pair
    # (CLAUDE.md: they answer ROLE vs REACH, and `check-host-platform-vocabulary`
    # enforces the distinction) — they carry no marker and are correctly ignored.
    entries, pairs = board_entries(ROOT)
    mismatched = {}
    for key, counterpart in sorted(pairs.items()):
        if counterpart not in entries:
            mismatched[key] = "names `[board.%s]`, which does not exist" % counterpart
            continue
        a, b = entries.get(key, {}), entries[counterpart]
        diffs = [f for f in ("arch", "platform", "packages") if a.get(f) != b.get(f)]
        if diffs:
            mismatched[key] = "diverged from `[board.%s]` in: %s" % (
                counterpart,
                ", ".join(diffs),
            )
    seen_board, seen_deploy = {}, {}
    for f in files:
        try:
            with open(os.path.join(ROOT, f), encoding="utf8") as fh:
                text = fh.read()
        except OSError:
            continue
        for deploy, board in exports(text):
            seen_deploy.setdefault(deploy, f)
            # A deploy is a scope, or splits into `<deploy>_*` scopes by board.
            if deploy not in scope_set and not any(
                s.startswith(deploy + "_") for s in scope_set
            ):
                bad_deploy.setdefault(deploy, f)
            if board is None:
                continue
            seen_board.setdefault(board, f)
            if (
                board not in cmakeb
                and board not in idx
                and board not in crates
                and board not in fixtures
            ):
                bad_board.setdefault(board, f)
            # SECOND, STRICTER assertion (phase-422 W7). The check above is an OR
            # over five namespaces, and `cmake/board/*.cmake` alone satisfies
            # every value — so it passed both before and after the index entries
            # were added, and never tested the property W7 exists to create.
            # Measured A/B, not assumed: identical OK line either way.
            #
            # `[board.*]` is the namespace `nros setup <board>` looks up, with an
            # exact-key lookup and no fallback. A board that resolves only in the
            # cmake namespace is one an out-of-tree user cannot provision — which
            # was true of four of five.
            elif board not in idx:
                not_index.setdefault(board, f)

    if mismatched:
        sys.stderr.write("check-board-vocabulary: %d mirror problem(s)\n\n" % len(mismatched))
        for k, why in sorted(mismatched.items()):
            sys.stderr.write(
                "  - [board.%s] %s.\n"
                "      The two are the same physical board under two spellings and the\n"
                "      index says EDIT BOTH. Until an alias key exists they must stay\n"
                "      identical, or `nros setup` provisions differently depending on\n"
                "      which name the user happened to read off their package.xml.\n\n"
                % (k, why)
            )
        return 1

    if bad_board or bad_deploy or not_index:
        sys.stderr.write(
            "check-board-vocabulary: %d problem(s)\n\n"
            % (len(bad_board) + len(bad_deploy) + len(not_index))
        )
        for b, f in sorted(not_index.items()):
            sys.stderr.write(
                "  - board=%r (%s) resolves, but NOT as an index `[board.*]` key.\n"
                "      That is the namespace `nros setup <board>` looks up (exact key,\n"
                "      no fallback), so an out-of-tree user cannot provision it — they\n"
                "      have no justfile to fall back on.\n"
                "      Add `[board.%s]` to nros-sdk-index.toml mirroring the equivalent\n"
                "      entry. Adding it is ADDITIVE; renaming is not, and is not asked\n"
                "      for here.\n\n" % (b, f, b)
            )
        for d, f in sorted(bad_deploy.items()):
            sys.stderr.write(
                "  - deploy=%r (%s) is not a scope and no scope starts with %r.\n"
                "      `nros setup --workspace` prints a provisioning command from this;\n"
                "      an unresolvable value makes that command fail.\n"
                "      Scopes: %s\n\n" % (d, f, d + "_", " ".join(sorted(scope_set)))
            )
        for b, f in sorted(bad_board.items()):
            sys.stderr.write(
                "  - board=%r (%s) resolves in NONE of the five namespaces:\n"
                "      cmake/board/nano-ros-board-*.cmake | [board.*] index key |\n"
                "      packages/boards/nros-board-* | fixtures.toml NANO_ROS_BOARD\n"
                "      Name one that exists, or add the board where it belongs.\n\n" % (b, f)
            )
        return 1

    sys.stdout.write(
        "check-board-vocabulary: OK — %d deploy value(s), %d board value(s); "
        "each resolves AND is an index key; %d mirrored pair(s) identical "
        "(cmake %d / index %d / crate %d / fixture %d).\n"
        % (
            len(seen_deploy),
            len(seen_board),
            len(pairs),
            len(cmakeb),
            len(idx),
            len(crates),
            len(fixtures),
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
