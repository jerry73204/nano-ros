#!/usr/bin/env python3
"""No test whose body only PRINTS — a test that cannot fail is not coverage.

CLAUDE.md: "Tests must fail on unmet preconditions (`assert!`/`bail!`/
`nros_tests::skip!`). Bare `eprintln!`+`return` reports PASS — never." This gate
enforces the shape-level half of that rule.

## The shape

The 2026-08-21 cleanup removed 17 of these across 10 files. Every one was the
same:

    #[test]
    fn test_freertos_detection() {
        let freertos = is_freertos_available();
        let qemu = is_qemu_available();
        eprintln!("FreeRTOS available: {}", freertos);
        eprintln!("QEMU available: {}", qemu);
    }

It reads probes and prints them. It asserts nothing, so it passes on a host with
no FreeRTOS, no QEMU and no toolchain — the machines where it was supposed to
tell you something. `zephyr.rs`'s copy said so itself in its last line: "These
are informational - don't fail if Zephyr isn't set up."

These are not merely useless. They inflate the pass count with results that
carry no information, and they duplicate probes that ARE load-bearing three
lines away as the `skip!` guards on the real tests — so the same call reads as
coverage in one place and as a precondition in the other.

## Why the signature is "prints only", not "has no assert"

The obvious rule — flag a test body containing no `assert!` — is wrong here and
would have flagged ~40 correct tests. Plenty of real tests delegate:

    #[test]
    fn freertos_board_run_executes_run_plan() {
        boot_and_connect("talker", "talker");   # asserts inside
    }

So the rule keys on what the body DOES, not on which tokens it contains. A body
is vacuous when every statement is a `let` binding, a print, or control flow
around prints — i.e. nothing in it can observe a wrong answer. A bare call
statement (`boot_and_connect(...)`) is an effect this gate deliberately treats
as possibly-asserting, because it is: the assertion is one frame down. That
makes the gate conservative in the right direction — it flags only bodies whose
effects are exhausted by printing.

## What to write instead

If the probe is a precondition, put it where it can stop the run:
`nros_tests::skip!("qemu-system-arm not found")`. If it is a property, assert
it. If it is neither, it is a `log` line in someone else's test, not a test.

## The sibling shape this gate does NOT catch — issue 1135

A body that does plenty on the passing path but opens with

    if !require_node_discoverable(&locator) {
        return;
    }

is the same lie and is invisible here: the body is full of real effects, so
"prints only" never fires. It cost four green `ros2 param *` tests on a host
where our node was missing from the graph.

The call site is not statically separable from its legitimate twin — both spell
`if <cond> { return; }`, and only the meaning of `<cond>` says whether the
return follows a probe FAILING or a check SUCCEEDING (`actions.rs` and
`services.rs` both return on success, correctly). Measured across the tree, a
blanket "no bare `return;` in a test body" rule flags 40 sites, mostly the
legitimate form, and would buy a 40-entry authored allowlist.

So the sibling rule keys on the HELPER's signature instead, where it is both
decidable and load-bearing: `scripts/check-test-precondition-guards.py`.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import tempfile
from pathlib import Path

PRINT_MACROS = ("eprintln!", "println!", "eprint!", "print!", "dbg!", "log::")

# Anything here means the body can observe a wrong answer.
ASSERT_TOKENS = (
    "assert!",
    "assert_eq!",
    "assert_ne!",
    "debug_assert",
    "panic!",
    "unreachable!",
    "todo!",
    "unimplemented!",
    "bail!",
    "skip!",
    ".expect(",
    ".unwrap(",
    "expect_err",
    "unwrap_err",
    "?;",
)

TEST_ATTRS = ("#[test]", "#[rstest]", "#[tokio::test]")

# Control-flow headers are structure, not effect.
CONTROL_RE = re.compile(
    r"^\s*(\}?\s*(match|if|if\s+let|else|else\s+if|for|while|loop)\b|\}|\{|\)|\],?|.*=>\s*\{?$)"
)
LET_RE = re.compile(r"^\s*let\b")


def strip_comments(text: str) -> str:
    out = []
    for line in text.split("\n"):
        s = line.strip()
        if s.startswith("//"):
            out.append("")
        else:
            out.append(line)
    return "\n".join(out)


def test_bodies(src: str):
    """Yield (fn_name, line_no, body) for every test fn in `src`."""
    lines = src.split("\n")
    i = 0
    while i < len(lines):
        m = re.match(r"\s*(?:pub )?(?:async )?fn (\w+)\s*\(", lines[i])
        if not m:
            i += 1
            continue
        back = "\n".join(lines[max(0, i - 8) : i])
        if not any(a in back for a in TEST_ATTRS):
            i += 1
            continue
        depth = 0
        j = i
        body = []
        while j < len(lines):
            body.append(lines[j])
            depth += lines[j].count("{") - lines[j].count("}")
            if depth <= 0 and j > i:
                break
            j += 1
        yield m.group(1), i + 1, "\n".join(body[1:])  # drop signature line
        i = j + 1


def is_vacuous(body: str) -> bool:
    body = strip_comments(body)
    if any(tok in body for tok in ASSERT_TOKENS):
        return False
    saw_print = False
    # A print macro's arguments routinely span lines:
    #     eprintln!(
    #         "ARM toolchain available: {}",
    #         available
    #     );
    # Those continuation lines are not statements, and reading them as such made
    # the first version of this gate miss 3 of the 17 tests it was written for.
    # Track the macro's paren depth and consume its arguments as part of it.
    pending = 0
    for line in body.split("\n"):
        s = line.strip()
        if not s:
            continue
        if pending > 0:
            pending += s.count("(") - s.count(")")
            continue
        if any(p in s for p in PRINT_MACROS):
            saw_print = True
            pending = max(0, s.count("(") - s.count(")"))
            continue
        if LET_RE.match(s) or CONTROL_RE.match(s):
            continue
        # A statement that is neither a binding, a print, nor control flow is a
        # real effect — possibly an assertion one frame down. Not our shape.
        return False
    return saw_print


def scan(paths) -> list[str]:
    violations = []
    for p in paths:
        src = Path(p).read_text(encoding="utf-8", errors="replace")
        for name, line, body in test_bodies(src):
            if is_vacuous(body):
                violations.append(f"{p}:{line}: `{name}` only prints — it cannot fail")
    return violations


def tracked_test_files() -> list[Path]:
    root = Path(
        subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    )
    out = subprocess.run(
        ["git", "ls-files", "*/tests/*.rs", "tests/*.rs"],
        capture_output=True,
        text=True,
        check=True,
        cwd=root,
    ).stdout.split()
    return [root / f for f in out]


SELF_TESTS = [
    (
        "prints only -> flagged",
        "#[test]\nfn t() {\n    let a = probe();\n    eprintln!(\"a: {}\", a);\n}\n",
        True,
    ),
    (
        "match arms that only print -> flagged",
        "#[test]\nfn t() {\n    match ws() {\n        Some(p) => eprintln!(\"{}\", p),\n"
        "        None => eprintln!(\"none\"),\n    }\n}\n",
        True,
    ),
    (
        "print plus assert -> ok",
        "#[test]\nfn t() {\n    let a = probe();\n    eprintln!(\"a: {}\", a);\n"
        "    assert!(a);\n}\n",
        False,
    ),
    (
        "delegates to an asserting helper -> ok",
        "#[test]\nfn t() {\n    boot_and_connect(\"talker\", \"talker\");\n}\n",
        False,
    ),
    (
        "print plus skip guard -> ok",
        "#[test]\nfn t() {\n    if !avail() {\n        nros_tests::skip!(\"nope\");\n    }\n"
        "    eprintln!(\"ok\");\n}\n",
        False,
    ),
    (
        "no print, no assert (empty-ish) -> not this shape",
        "#[test]\nfn t() {\n    let _x = 1;\n}\n",
        False,
    ),
    (
        "commented-out assert does not rescue it",
        "#[test]\nfn t() {\n    // assert!(a);\n    eprintln!(\"a\");\n}\n",
        True,
    ),
    (
        "non-test fn that only prints -> ignored",
        "fn helper() {\n    eprintln!(\"a\");\n}\n",
        False,
    ),
    # The first version of this gate read a print macro's continuation lines as
    # statements, so a multi-line print looked like a real effect and 3 of the
    # 17 tests this was written for slipped through. These two pin that.
    (
        "multi-line print, prints only -> flagged",
        "#[test]\nfn t() {\n    let a = probe();\n    eprintln!(\n"
        '        "toolchain available: {}",\n        a\n    );\n}\n',
        True,
    ),
    (
        "multi-line print then a real effect -> ok",
        "#[test]\nfn t() {\n    let a = probe();\n    eprintln!(\n"
        '        "toolchain available: {}",\n        a\n    );\n'
        '    boot_and_connect("talker");\n}\n',
        False,
    ),
]


def self_test() -> int:
    failures = 0
    with tempfile.TemporaryDirectory() as td:
        for name, src, expect_flag in SELF_TESTS:
            f = Path(td) / "case.rs"
            f.write_text(src)
            got = bool(scan([f]))
            ok = got == expect_flag
            print(f"  [{'OK' if ok else 'FAIL'}] {name}")
            if not ok:
                failures += 1
                print(f"        expected flagged={expect_flag}, got {got}")
    if failures:
        print(f"\ncheck-no-vacuous-tests --self-test: {failures} case(s) FAILED")
        return 1
    print(f"\ncheck-no-vacuous-tests --self-test: {len(SELF_TESTS)} case(s) OK")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()

    files = tracked_test_files()
    violations = scan(files)
    if violations:
        print("check-no-vacuous-tests: FAIL — tests that only print:\n")
        for v in violations:
            print(f"  {v}")
        print(
            "\nA test whose body only prints reports PASS on the machine where it was\n"
            "supposed to tell you something. Make the probe a `nros_tests::skip!` guard\n"
            "(so an unmet precondition stops the run) or assert the property.\n"
            "See CLAUDE.md 'Tests must fail on unmet preconditions'."
        )
        return 1
    print(f"check-no-vacuous-tests: OK ({len(files)} test files)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
