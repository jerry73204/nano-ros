#!/usr/bin/env python3
"""Every core crate declares `#![no_std]` UNCONDITIONALLY.

ARCHITECTURE section 2 / phase-359: the terminal state of the core crates is
`core` and `core+alloc`. `std` there is a second implementation of the platform
layer, not a convenience over it.

WHY THE SPELLING MATTERS, and why this is not a style gate.

`#![cfg_attr(not(feature = "std"), no_std)]` compiles a DIFFERENT CRATE
depending on a feature. Both configurations are then real, both need testing,
and the one an embedded image uses is the one CI is least likely to build --
every merge-gating lane in this repo builds with `std`, because `std` is what a
host test needs. So the conditional form quietly creates the configuration that
nothing checks.

`#![no_std]` unconditional removes the choice. A core crate that cannot name
`std` cannot grow a dependency on it, and `check-no-std-stdio` (the sibling
gate) then has a fixed target rather than a moving one.

This does NOT stop a crate using `alloc` -- that is a separate axis and a
legitimate one (`extern crate alloc` behind `feature = "alloc"`). Issue 1177 is
about the alloc axis; this gate is about the std one.

Run: python3 scripts/check-core-crates-are-no-std.py [--self-test]
"""

import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]

# The core crates: the layer that must run on a target with no operating
# system. Deliberately a LIST rather than a glob over `packages/core` -- a new
# crate landing there should have to be added here on purpose, with a person
# deciding it is core, instead of being swept in by a directory name.
CORE_CRATES = [
    "packages/core/nros-node",
    "packages/core/nros-core",
    "packages/core/nros-rmw",
    "packages/core/nros-log",
    "packages/core/nros-params",
    "packages/platform/nros-platform-api",
]

UNCONDITIONAL = re.compile(r"^\s*#!\[no_std\]", re.M)
CONDITIONAL = re.compile(r"^\s*#!\[cfg_attr\([^)]*no_std", re.M)


def offenders(roots=None, repo=None):
    base = Path(repo) if repo else REPO
    out = []
    for rel in roots if roots is not None else CORE_CRATES:
        lib = base / rel / "src" / "lib.rs"
        if not lib.is_file():
            out.append((rel, "no src/lib.rs"))
            continue
        text = lib.read_text(errors="replace")
        if UNCONDITIONAL.search(text):
            continue
        if CONDITIONAL.search(text):
            out.append((rel, "conditional `cfg_attr(..., no_std)` -- make it unconditional"))
        else:
            out.append((rel, "no `#![no_std]` at all"))
    return out


def self_test():
    """A crate with each spelling, so the gate is known to be able to fail."""
    import tempfile

    cases = [
        ("#![no_std]\n", 0, "unconditional passes"),
        ('#![cfg_attr(not(feature = "std"), no_std)]\n', 1, "conditional is caught"),
        ("// nothing\n", 1, "missing is caught"),
        ("//! docs\n#![no_std]\n", 0, "unconditional after a doc comment passes"),
    ]
    failures = 0
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        for i, (body, want, name) in enumerate(cases):
            rel = f"crate{i}"
            (root / rel / "src").mkdir(parents=True)
            (root / rel / "src" / "lib.rs").write_text(body)
            got = len(offenders(roots=[rel], repo=root))
            if got != want:
                print(f"  self-test FAIL: {name} -- got {got}, want {want}", file=sys.stderr)
                failures += 1
    if failures:
        print(f"check-core-crates-are-no-std self-test: FAILED ({failures})", file=sys.stderr)
        return 1
    print(f"check-core-crates-are-no-std self-test: OK ({len(cases)} cases)")
    return 0


def main(argv):
    if len(argv) == 2 and argv[1] == "--self-test":
        return self_test()
    if self_test():
        return 1
    bad = offenders()
    if not bad:
        print(f"check-core-crates-are-no-std: OK -- {len(CORE_CRATES)} core crate(s) are unconditionally no_std.")
        return 0
    print("check-core-crates-are-no-std: a core crate does not declare `#![no_std]`:", file=sys.stderr)
    for rel, why in bad:
        print(f"  {rel}: {why}", file=sys.stderr)
    print("", file=sys.stderr)
    print("  A core crate runs where there is no operating system. The conditional", file=sys.stderr)
    print("  form compiles a different crate per feature, and the configuration an", file=sys.stderr)
    print("  embedded image uses is the one no merge-gating lane builds.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
