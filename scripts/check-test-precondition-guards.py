#!/usr/bin/env python3
"""A precondition guard in a TEST FILE must own its verdict — issue 1135.

CLAUDE.md: "Tests must fail on unmet preconditions (`assert!`/`bail!`/
`nros_tests::skip!`). Bare `eprintln!`+`return` reports PASS — never."
`check-no-vacuous-tests` enforces the half of that rule that is a property of a
test BODY (it only prints). This gate enforces the half that is a property of a
test-local HELPER's SIGNATURE.

## The shape

    fn require_node_discoverable(locator: &str) -> bool {  // params.rs, pre-1135
        for attempt in 1..=3 { ... return true; }
        eprintln!("Skipping test: nros node /demo/talker not discoverable ...");
        false
    }

    #[rstest]
    fn test_ros2_param_list(zenohd_unique: ZenohRouter) {
        ...
        if !require_node_discoverable(&locator) {
            return;              // <-- a bare `return` from a #[test] is a PASS
        }
        ...
    }

Four call sites, four identical mistakes, four green tests. And note WHAT was
swallowed: zenohd, ROS 2 and the fixture had all already passed their own
guards, so a `false` here meant "everything is up and our node is not in the
graph" — a delivery failure, the exact thing those four tests exist to detect.
Under `--failure-output never` (what `just native test-ros2-params` passes) the
`eprintln!` was not shown either, so the only evidence was invisible.

## Why the signature and not the call site

The call-site shape is NOT statically separable from its legitimate twin. Both
of these are `if <cond> { return; }` inside a test body:

    if !require_node_discoverable(&locator) { return; }   // defect: PASS on failure
    if server.wait_for_output_pattern(MARKER, T).is_ok() { return; }  // fine: PASS on success

Only the meaning of `<cond>` tells them apart, and a scanner cannot read it. A
rule of "no bare `return;` in a test body" IS decidable, but it is a different
rule from the one we want: measured across the tree it flags 40 sites, most of
them the legitimate success-path form, which buys an authored 40-entry
allowlist — the kind of map CLAUDE.md already records drifting (the rmw parity
map read "gap" for 28 slots that had landed).

The signature is decidable AND load-bearing. A guard named `require_*` /
`ensure_*` / `need_*` / `maybe_*` that hands back a `bool` (or an `Option<()>`,
which is a bool wearing a hat) has delegated the verdict to callers nobody
checks; a guard that returns `()` has kept it, and `skip!` is then the only way
out. Five such helpers existed in this tree and every one of them either was the
bug (`require_node_discoverable`, `require_nuttx_setup`) or carried its dead
branch (`require_native_env`, `maybe_skip`) or reported "check failed" with the
real reason `eprintln!`ed into a stream the runner discards
(`require_freertos`, `require_threadx_riscv64`, `require_esp32_networked`).

## What is deliberately NOT covered

* **Library probes in `packages/testing/nros-tests/src/`** — `require_zenohd()`,
  `require_ros2()`, `is_*_available()` and friends legitimately return `bool`:
  they are composed, and each call site writes its own message. The verdict
  belongs to the test-local guard that composes them, which is what this gate
  scopes to.
* **A helper returning a real value** — `fn require_preconditions() ->
  Option<(PathBuf, PathBuf)>` is fine and is not flagged: the caller needs the
  paths, and `let Some(x) = f() else { skip!(..) }` is the correct spelling.
  Only `bool` and the value-free `Option<()>` are refused.
* **A test that inlines the probe and returns.** The gate keys on helpers
  because that is where the leverage was: one helper, four wrong call sites.

## What to write instead

    fn require_node_discoverable(locator: &str) {
        for attempt in 1..=3 { ... return; }
        nros_tests::skip_class!(resource, "/demo/talker not discoverable ...");
    }

    require_node_discoverable(&locator);   // no verdict for a caller to drop
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import tempfile
from pathlib import Path

# A helper whose NAME announces it is a precondition gate.
GUARD_NAME_RE = re.compile(r"^(require|ensure|need|maybe)_\w+$")

# `fn <name>(<args>) -> <ret> {` — args may span lines, so match the name and
# then find the `->` that belongs to this signature (before the opening brace).
FN_RE = re.compile(r"^\s*(?:pub(?:\([^)]*\))?\s+)?(?:async\s+)?fn\s+(\w+)\s*\(")

# The two return types that are a VERDICT and nothing else.
VERDICT_RET_RE = re.compile(r"->\s*(bool|Option\s*<\s*\(\s*\)\s*>)\s*\{?\s*$")


def strip_line_comments(line: str) -> str:
    """Drop a trailing `//` comment. Crude but sufficient: no `//` appears
    inside a string literal in any signature this gate reads."""
    i = line.find("//")
    return line if i < 0 else line[:i]


def guard_violations(path: str, src: str) -> list[str]:
    out: list[str] = []
    lines = src.split("\n")
    for i, raw in enumerate(lines):
        line = strip_line_comments(raw)
        m = FN_RE.match(line)
        if not m:
            continue
        name = m.group(1)
        if not GUARD_NAME_RE.match(name):
            continue
        # Accumulate the signature until its opening brace, so a multi-line
        # argument list still exposes the return type.
        sig = line
        j = i
        while "{" not in sig and j + 1 < len(lines) and j - i < 12:
            j += 1
            sig += " " + strip_line_comments(lines[j]).strip()
        # Only the text before the body brace is the signature.
        sig = sig.split("{", 1)[0] + "{"
        if VERDICT_RET_RE.search(sig):
            ret = VERDICT_RET_RE.search(sig).group(1)
            out.append(
                f"{path}:{i + 1}: `{name}` returns `{ret}` — a test-local precondition "
                f"guard must own its verdict (return `()` and `nros_tests::skip!`), "
                f"not hand a caller a value they can drop with a bare `return`"
            )
    return out


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


def scan(paths) -> list[str]:
    violations: list[str] = []
    for p in paths:
        src = Path(p).read_text(encoding="utf-8", errors="replace")
        violations.extend(guard_violations(str(p), src))
    return violations


SELF_TESTS = [
    (
        "require_* -> bool is the 1135 shape",
        "fn require_node_discoverable(locator: &str) -> bool {\n    true\n}\n",
        True,
    ),
    (
        "require_* -> Option<()> is a bool wearing a hat",
        "fn require_nuttx_setup() -> Option<()> {\n    Some(())\n}\n",
        True,
    ),
    (
        "maybe_* -> bool too — `if maybe_skip(..) { return; }` was three call sites",
        "fn maybe_skip(p: Platform, l: Lang) -> bool {\n    false\n}\n",
        True,
    ),
    (
        "the fixed spelling -> ()",
        "fn require_node_discoverable(locator: &str) {\n"
        '    nros_tests::skip!("nope");\n}\n',
        False,
    ),
    (
        "a guard returning a real value is correct — the caller needs it",
        "fn require_preconditions() -> Option<(PathBuf, PathBuf)> {\n    None\n}\n",
        False,
    ),
    (
        "Result<T> guard is not this shape",
        "fn require_agent() -> TestResult<XrceAgent> {\n    todo!()\n}\n",
        False,
    ),
    (
        "a non-guard name returning bool is nobody's precondition",
        "fn is_freertos_available() -> bool {\n    true\n}\n",
        False,
    ),
    (
        "multi-line argument list still exposes the return type",
        "fn require_cell_runnable(\n    platform: Platform,\n    lang: Lang,\n"
        ") -> bool {\n    false\n}\n",
        True,
    ),
    (
        "pub visibility does not exempt it",
        "pub fn require_thing() -> bool {\n    true\n}\n",
        True,
    ),
    (
        "a commented-out signature is not code",
        "// fn require_thing() -> bool {\n//     true\n// }\n",
        False,
    ),
    (
        "Option<Something> spanning a line break is not Option<()>",
        "fn require_paths() -> Option<\n    (PathBuf, PathBuf),\n> {\n    None\n}\n",
        False,
    ),
]


def self_test(quiet: bool = True) -> int:
    failures = 0
    with tempfile.TemporaryDirectory() as td:
        for name, src, expect_flag in SELF_TESTS:
            f = Path(td) / "case.rs"
            f.write_text(src)
            got = bool(scan([f]))
            ok = got == expect_flag
            if not quiet or not ok:
                print(f"  [{'OK' if ok else 'FAIL'}] {name}")
            if not ok:
                failures += 1
                print(f"        expected flagged={expect_flag}, got {got}")
    if failures:
        print(f"\ncheck-test-precondition-guards self-test: {failures} case(s) FAILED")
        return 1
    if not quiet:
        print(
            f"\ncheck-test-precondition-guards self-test: {len(SELF_TESTS)} case(s) OK"
        )
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--self-test",
        action="store_true",
        help="run the classifier's own cases verbosely and exit (they also run "
        "on every normal invocation)",
    )
    args = ap.parse_args()
    if args.self_test:
        return self_test(quiet=False)

    files = tracked_test_files()
    violations = scan(files)
    if violations:
        print("check-test-precondition-guards: FAIL\n")
        for v in violations:
            print(f"  {v}")
        print(
            "\nA `require_*`/`ensure_*`/`need_*`/`maybe_*` helper in a test file that\n"
            "returns `bool` or `Option<()>` hands its verdict to the caller. Issue 1135:\n"
            "four callers of one such helper wrote `if !guard() { return; }`, and a bare\n"
            "`return` from a `#[test]` is a PASS — so four ROS 2 param tests reported\n"
            "green on a host where our node never joined the graph.\n"
            "Return `()` and `nros_tests::skip!` (or `skip_class!`) inside the guard.\n"
            "A guard returning a real value (`Option<PathBuf>`, `TestResult<T>`) is fine."
        )
        return 1
    print(f"check-test-precondition-guards: OK ({len(files)} test files)")
    return 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test(quiet=False))
    # Always, not only behind the flag. A negative control that only runs when
    # someone remembers to ask for it decays into a comment, and a gate whose
    # classifier has quietly stopped classifying reports OK over nothing —
    # which is the same false green this gate exists to refuse.
    if self_test():
        sys.exit(1)
    sys.exit(main())
