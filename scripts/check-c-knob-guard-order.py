#!/usr/bin/env python3
"""issue 1167 — a `#if KNOB < 1` guard must come AFTER the knob is defined.

WHY THIS EXISTS. Issue 1015 gave every knob-sized C array a compile-time
refusal:

    #if ZPICO_MAX_SESSIONS < 1
    #error "ZPICO_MAX_SESSIONS must be >= 1: it sizes a fixed C array"
    #endif

The C preprocessor evaluates an UNDEFINED identifier in `#if` as 0. So a guard
that runs before its knob's `#ifndef`/`#define` default does not protect the
array — it refuses the build outright, and only on the configurations that do
not pass a `-D` for that knob. Nine of the ten guards were fine because their
defaults sit in the defaults block above; `ZPICO_MAX_SESSIONS`'s default was
~100 lines BELOW, and it is precisely the knob with no Kconfig row and no cmake
bridge, so the Zephyr C lane never supplies a `-D`. Every Zephyr zenoh image
failed to compile, and no merge-gating lane builds Zephyr, so it reached `main`.

The rule is about ORDER, which is why a human reading the diff does not catch
it: both the guard and the default are correct in isolation.

A knob is considered defined before the guard if either
  * an `#ifndef KNOB` / `#define KNOB` appears earlier in the same file, or
  * a header under the same component's `include/` defines it unconditionally.

Usage:
    python3 scripts/check-c-knob-guard-order.py [--self-test]
"""

import re
import subprocess
import sys
from pathlib import Path

GUARD_RE = re.compile(r"^#if\s+([A-Z][A-Z0-9_]*)\s*<\s*1\s*$")
IFNDEF_RE = re.compile(r"^#ifndef\s+([A-Z][A-Z0-9_]*)\s*$")
DEFINE_RE = re.compile(r"^#\s*define\s+([A-Z][A-Z0-9_]*)\b")


def guards_and_defs(text):
    """(knob -> first guard line, knob -> first define line), 1-indexed."""
    guards, defs = {}, {}
    for n, line in enumerate(text.split("\n"), 1):
        m = GUARD_RE.match(line)
        if m and m.group(1) not in guards:
            guards[m.group(1)] = n
        m = IFNDEF_RE.match(line) or DEFINE_RE.match(line)
        if m and m.group(1) not in defs:
            defs[m.group(1)] = n
    return guards, defs


def header_defines(repo: Path, headers):
    """knob -> header that defines it unconditionally, over TRACKED headers only.

    Built once from `git ls-files`, never by walking: a recursive walk reaches
    build output and other checkouts' worktrees, and is ~500x slower on this
    tree (`check-no-tracked-file-find` measures 7m36s -> 0.8s). It also caught
    the first draft of THIS script, which is the gate working.

    Deliberately coarse — any `#define KNOB` in a tracked header under an
    `include/` directory counts, wherever it sits. A knob that is genuinely
    defined by a header the TU does not include would then read as safe here;
    that is a false negative this gate accepts, because the compiler catches it
    immediately and loudly (it is the very `#error` this rule is about).
    """
    found = {}
    for rel in headers:
        if "/include/" not in f"/{rel}":
            continue
        path = repo / rel
        if not path.is_file():
            continue
        for line in path.read_text(errors="replace").split("\n"):
            m = DEFINE_RE.match(line)
            if m:
                found.setdefault(m.group(1), rel)
    return found


def check_file(text, in_headers):
    guards, defs = guards_and_defs(text)
    bad = []
    for knob, gline in sorted(guards.items(), key=lambda kv: kv[1]):
        dline = defs.get(knob)
        if dline is not None and dline < gline:
            continue
        if knob in in_headers:
            continue
        bad.append((knob, gline, dline))
    return bad


SELF_TESTS = [
    # (source, expected offending knobs)
    ("#ifndef A\n#define A 1\n#endif\n#if A < 1\n#error x\n#endif\n", []),
    ("#if A < 1\n#error x\n#endif\n#ifndef A\n#define A 1\n#endif\n", ["A"]),
    ("#if A < 1\n#error x\n#endif\n", ["A"]),
    # a guard with no `< 1` shape is not this rule's business
    ("#if A > 3\n#error x\n#endif\n", []),
    # two knobs, one ordered right and one wrong
    (
        "#ifndef A\n#define A 1\n#endif\n#if A < 1\n#error\n#endif\n"
        "#if B < 1\n#error\n#endif\n#ifndef B\n#define B 2\n#endif\n",
        ["B"],
    ),
    # an unconditional #define (no #ifndef) before the guard also counts
    ("#define A 4\n#if A < 1\n#error x\n#endif\n", []),
]


def self_test():
    bad = 0
    for i, (src, expect) in enumerate(SELF_TESTS, 1):
        guards, defs = guards_and_defs(src)
        got = sorted(k for k, g in guards.items() if defs.get(k) is None or defs[k] > g)
        if got != sorted(expect):
            print(f"  case {i}: expected {sorted(expect)}, got {got}")
            bad += 1
    if bad:
        print(f"check-c-knob-guard-order --self-test: {bad} case(s) FAILED")
        return 1
    print(f"check-c-knob-guard-order --self-test: {len(SELF_TESTS)} case(s) OK")
    return 0


def main():
    repo = Path(
        subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            capture_output=True, text=True, check=True,
        ).stdout.strip()
    )
    if "--self-test" in sys.argv:
        return self_test()
    if self_test():
        return 1

    # Tracked files only: a filesystem walk reaches build output and other
    # checkouts' worktrees (issues 1157/1166).
    files = [
        f
        for f in subprocess.run(
            ["git", "ls-files", "-z", "*.c", "*.h"],
            capture_output=True, text=True, check=True, cwd=repo,
        ).stdout.split("\0")
        if f
    ]
    in_headers = header_defines(repo, [f for f in files if f.endswith(".h")])

    failures, scanned, guarded = [], 0, 0
    for rel in files:
        path = repo / rel
        if not path.is_file():
            continue
        text = path.read_text(errors="replace")
        if "< 1" not in text:
            continue
        scanned += 1
        bad = check_file(text, in_headers)
        guarded += len(guards_and_defs(text)[0])
        for knob, gline, dline in bad:
            where = f"defined at line {dline}" if dline else "never defined here"
            failures.append(f"  {rel}:{gline}: `#if {knob} < 1` — {knob} is {where}")

    if failures:
        print("check-c-knob-guard-order: FAIL\n")
        print("\n".join(failures))
        print(
            "\nThe preprocessor reads an UNDEFINED identifier in `#if` as 0, so a\n"
            "guard placed before its knob's default does not protect the array —\n"
            "it refuses every build that does not pass a -D for that knob. Move\n"
            "the `#ifndef`/`#define` ABOVE the guard (issue 1167)."
        )
        return 1

    print(
        f"check-c-knob-guard-order: OK ({guarded} `< 1` guard(s) across "
        f"{scanned} file(s); each knob is defined before its guard)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
