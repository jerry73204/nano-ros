#!/usr/bin/env python3
"""A board alias names ONE board — phase-435 W5.

Every `names = [...]` entry in a `nros-board.toml` descriptor is a spelling an
entry may `deploy =` to. If two descriptors claim the same spelling, that name
selects two board implementations, and an implementation is what decides the
toolchain: `threadx-linux` builds for the host, `threadx-qemu-riscv64` needs
`riscv-none-elf-gcc` and an emulator. A name that resolves to both cannot
resolve to one provisioning row.

WHY THIS IS A GATE BEFORE A BUG RATHER THAN AFTER ONE

`threadx` is claimed by both today, and NO entry uses the bare name — every one
spells `threadx-linux` or `threadx-qemu-riscv64`. So nothing is broken; what
exists is a name that would break the first time somebody reached for the
obvious short spelling, and would do it by silently picking whichever descriptor
the scan happened to read first.

That is the shape RFC-0062 amendment 4 recorded and phase-422 W7 keeps meeting
from the other side: two vocabularies that each look consistent until something
has to resolve across them.

WHAT IS DELIBERATELY ALLOWED, AND WHY `threadx` IS NOT A BUG

Running this gate for the first time answered its own question. Excluding test
fixtures, exactly one name is claimed twice: `threadx`, by
`nros-board-threadx-linux` and `nros-board-threadx-qemu-riscv64`. That looked
like the defect the gate was written for, and it is not.

`threadx` is a DEPLOY FAMILY, not a board. 25 `package.xml` files carry
`<nano_ros deploy="threadx" board="riscv64-qemu"/>` or `board="threadx-linux"`,
and per RFC-0087 D3 `deploy=` names a `[deploy.*]` block rather than a provider —
the `board=` attribute beside it is what selects the implementation. So a family
name appearing in two boards' `names` lists is the vocabulary working: both
boards ARE threadx, and nothing resolves on the family alone.

The rule is therefore narrower than "a name claims one board": **a name that is
not a deploy family must claim one board.** A family name is exempt by
derivation, not by an allow-list entry somebody has to maintain — the families
are read from the same `package.xml` exports that use them.

WHAT IS DELIBERATELY ALLOWED

One descriptor may hold SEVERAL blocks, and blocks in one descriptor may share a
name — `nros-board-nuttx-qemu` carries an arm block and a riscv block, and both
legitimately answer to `nuttx` in their own `names` lists only if the file's
author meant it. The rule is per FILE, not per block: two different descriptor
FILES claiming one name is the ambiguity, because that is what makes the choice
depend on scan order rather than on anything declared.

Run:  python3 scripts/check-board-alias-unique.py [--self-test]
"""

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# (name -> reason) for a claim that may be shared. Checked in BOTH directions.
#
# EMPTY. An entry needs a reason that makes the choice DETERMINISTIC without the
# name — and if such a reason existed, the name would not be ambiguous.
EXEMPT = {}


def names_in(text):
    """Every `names = [...]` value in a descriptor, flattened."""
    out = []
    for m in re.finditer(r"^\s*names\s*=\s*\[([^\]]*)\]", text, re.M):
        out.extend(re.findall(r'"([^"]+)"', m.group(1)))
    return out


# A fixture workspace is a scanned unit in its own right — its descriptors are a
# deliberate copy of the board tree, read only by the test that owns it. Two
# copies of one board are not an ambiguity in either tree.
PRUNE = ("packages/cli/nros-cli-core/tests/fixtures/",)


def descriptors():
    """[(path, [names])] for every tracked board descriptor outside a fixture."""
    listing = subprocess.run(
        ["git", "-C", ROOT, "ls-files", "*nros-board.toml"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.split()
    out = []
    for rel in listing:
        if any(rel.startswith(p) for p in PRUNE):
            continue
        with open(os.path.join(ROOT, rel), encoding="utf8") as fh:
            out.append((rel, names_in(fh.read())))
    return out


def deploy_families():
    """Every `deploy="…"` a package.xml export declares.

    Read rather than listed: a family name is exempt because something uses it
    as one, and an allow-list would go stale the moment a family is added or
    retired.
    """
    listing = subprocess.run(
        ["git", "-C", ROOT, "ls-files", "*package.xml"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.split()
    out = set()
    for rel in listing:
        with open(os.path.join(ROOT, rel), encoding="utf8", errors="replace") as fh:
            for tag in re.findall(r"<nano_ros ([^/]*)/>", fh.read()):
                m = re.search(r'deploy="([^"]+)"', tag)
                if m:
                    out.add(m.group(1))
    return out


def clashes(entries, families=frozenset()):
    """{name: [files]} for every non-family name claimed by more than one FILE."""
    seen = {}
    for path, names in entries:
        for n in set(names):
            seen.setdefault(n, []).append(path)
    return {
        n: sorted(f)
        for n, f in seen.items()
        if len(set(f)) > 1 and n not in families
    }


def self_test():
    t = 'names = ["a", "b"]\nplatform = "x"\nnames = ["c"]\n'
    assert names_in(t) == ["a", "b", "c"], names_in(t)

    one = [("boards/x/nros-board.toml", ["threadx", "threadx-linux"])]
    assert clashes(one) == {}, clashes(one)

    two = [
        ("boards/x/nros-board.toml", ["threadx", "threadx-linux"]),
        ("boards/y/nros-board.toml", ["threadx", "threadx-riscv64"]),
    ]
    got = clashes(two)
    assert list(got) == ["threadx"], got
    assert len(got["threadx"]) == 2, got

    # ...unless it is a deploy FAMILY, which `board=` disambiguates.
    assert clashes(two, families={"threadx"}) == {}, clashes(two, {"threadx"})

    # A non-family name is still a clash even beside a family exemption.
    three = two + [("boards/z/nros-board.toml", ["threadx-linux"])]
    assert list(clashes(three, families={"threadx"})) == ["threadx-linux"]

    # The families really are read from the tree.
    fams = deploy_families()
    assert "threadx" in fams and "native" in fams, sorted(fams)

    # Two BLOCKS in one file may share a name — the rule is per file.
    same_file = [("boards/z/nros-board.toml", ["nuttx", "nuttx", "nuttx-riscv"])]
    assert clashes(same_file) == {}, clashes(same_file)

    # The scan really reads the tree, so a rename cannot make it vacuous.
    real = descriptors()
    assert real, "no board descriptors found — the gate would pass vacuously"
    assert any(ns for _, ns in real), "no descriptor declares any name"

    sys.stdout.write("check-board-alias-unique self-test: OK\n")


def main():
    if "--self-test" in sys.argv:
        self_test()
        return 0
    self_test()

    entries = descriptors()
    families = deploy_families()
    if not entries:
        sys.stderr.write(
            "error: no `nros-board.toml` descriptors found. This gate would then\n"
            "pass vacuously; check the tree or the glob.\n"
        )
        return 1

    problems = []
    seen_exempt = set()
    for name, files in sorted(clashes(entries, families).items()):
        if name in EXEMPT:
            seen_exempt.add(name)
            continue
        problems.append(
            "board alias %r is claimed by %d descriptors:\n        %s\n"
            "    A name selects a board IMPLEMENTATION, and the implementation\n"
            "    decides the toolchain — so an ambiguous alias cannot resolve to\n"
            "    one provisioning row, and would pick whichever descriptor the\n"
            "    scan read first. Give each board its own spelling."
            % (name, len(files), "\n        ".join(files))
        )

    for name in EXEMPT:
        if name not in seen_exempt:
            problems.append(
                "STALE exemption %r matches no clash.\n"
                "    Delete it — an allow-list checked one way stops covering\n"
                "    what it claims to." % name
            )

    if problems:
        sys.stderr.write("check-board-alias-unique: %d problem(s)\n\n" % len(problems))
        for p in problems:
            sys.stderr.write("  - %s\n\n" % p)
        return 1

    total = sum(len(set(ns)) for _, ns in entries)
    sys.stdout.write(
        "check-board-alias-unique OK — %d descriptor(s), %d alias claim(s); each\n"
        "non-family name claims one board (%d deploy famil(y/ies) exempt: %s).\n"
        % (len(entries), total, len(families), ", ".join(sorted(families)))
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
