#!/usr/bin/env python3
"""Gate: every code identifier the book quotes must exist in the tree.

The 2026-08-21 five-persona book review found one disease under most of
its ~60 findings: prose describing code that had been renamed or
retired — `ExecutorConfig::max_callbacks_per_spin` (never existed),
`nros_board_init_clocks` (never existed), `try_recv_safe` (renamed),
`zenohd` install paths (retired), `spin_period_polling` (renamed).
Each was plausible, so nothing failed until a reader typed it.

This gate closes the class for the spellings that are cheaply and
reliably checkable:

  * `nros::path::to::item` Rust paths in backtick spans — the final
    segment must occur (word-bounded) in the public crates' sources.
  * `nros_lowercase_ident` C/shell identifiers in backtick spans —
    must occur somewhere in the non-doc tree.
  * `nros_*()` / `nano_ros_*()` CMake calls in the book — must be
    defined by a `function()`/`macro()` in the tree's cmake.

Spans containing `<`, `…`, or `*` are placeholders and skipped. False
positives go in EXEMPT with a reason, not silenced by loosening the
regexes.

Run:  python3 scripts/check-book-identifiers.py
Gate: just check book-identifiers
"""

import os
import re
import subprocess
import sys

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parent / "lib"))
from tracked import tracked  # issue 0721: index lookup, not a walk

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOOK = os.path.join(ROOT, "book", "src")

# identifier -> why it is exempt
EXEMPT = {
    # Historical names the book cites AS retired (prose about the past):
    "nano_ros_deploy": "configuration.md names it as the retired predecessor",
    # Book-local illustrative names the reader is told to invent:
    "nros_board_myboard": "porting/overview scaffold output name",
    "nros_platform_myrtos": "porting/overview scaffold output name",
    "nros_board_stm32f4_freertos": "vendor-overlay worked example (out-of-tree crate)",
    "my_stm32f4_board": "stm32f4-out-of-tree worked example",
    # phase-417 widened this gate to `rcl_*`/`rclc_*`, and a MIGRATION table
    # names upstream's identifier on the left of the arrow. `rcl_node_init` is
    # micro-ROS's, not ours -- ours is `rclc_node_init_default`. The gate reads
    # backtick spans and cannot see arrow direction, so the "from" side of a
    # port table needs naming here rather than being silently accepted.
    "rcl_node_init": "comparison-vs-microros names it as micro-ROS's spelling, the FROM side of the port table",
}

SPAN = re.compile(r"`([^`\n]+)`")
RUST_PATH = re.compile(r"^nros(::[A-Za-z_][A-Za-z0-9_]*!?)+(\(\))?$")
# phase-417 stage 6 — `rcl_*` / `rclc_*` too, not only `nros_*`.
#
# The book now teaches rcl/rclc spellings, because the C API took them
# (RFC-0089). Before this, every `rcl_get_zero_initialized_node` a doc author
# typed was INVISIBLE to this gate: it validated `nros_*` only, so a typo in the
# new vocabulary shipped green while the old vocabulary was checked. W-B4
# grep-verified its 22 new identifiers by hand for exactly that reason.
#
# `rmw_*` is deliberately NOT here: the book names upstream `rmw_qos_profile_*`
# constants we do not declare, and they would all read as unknown.
C_IDENT = re.compile(r"^(nros|rcl|rclc)_[a-z0-9_]+$")
CMAKE_CALL = re.compile(r"^(nros_[a-z0-9_]+|nano_ros_[a-z0-9_]+)\(")


def book_files():
    # The book's pages are all tracked, so the index answers this — a walk here
    # descends whatever `just book` left in book/ (issue 0721).
    for path in sorted(tracked(BOOK, suffix=".md")):
        yield str(path)


def git_grep_word(ident, pathspecs):
    """True when `ident` occurs word-bounded in the given pathspecs."""
    res = subprocess.run(
        ["git", "grep", "-lw", "--", ident] + pathspecs,
        cwd=ROOT, capture_output=True, text=True)
    return res.returncode == 0


TREE_SPECS = [
    ":(glob)packages/**", ":(glob)cmake/**", ":(glob)zephyr/**",
    ":(glob)integrations/**", ":(glob)scripts/**", "justfile",
    ":(glob)just/**", ":(glob)examples/**",
    # exclude the things that quote rather than define:
    ":(exclude)*.md", ":(exclude)packages/*/third-party/**",
]
# `nros::x` spans are Rust paths OR C++ `::nros` namespace members
# (nros-cpp headers) — accept a definition in either surface.
RUST_SPECS = [":(glob)packages/api/**/*.rs", ":(glob)packages/core/**/*.rs",
              ":(glob)packages/api/nros-cpp/include/**",
              ":(glob)packages/api/nros-c/include/**"]
CMAKE_SPECS = [":(glob)cmake/**", ":(glob)zephyr/**", ":(glob)integrations/**",
               ":(glob)packages/**/*.cmake", "nano_rosConfig.cmake"]


def cmake_defined(name):
    res = subprocess.run(
        ["git", "grep", "-liE", "--", rf"(function|macro)\s*\(\s*{name}\b"]
        + CMAKE_SPECS, cwd=ROOT, capture_output=True, text=True)
    return res.returncode == 0


def main():
    failures = []
    seen = {}  # ident -> verdict cache
    for path in book_files():
        rel = os.path.relpath(path, ROOT)
        for lineno, line in enumerate(open(path, encoding="utf8"), 1):
            for span in SPAN.findall(line):
                span = span.strip()
                if any(ch in span for ch in "<>…*$"):
                    continue
                ident, kind = None, None
                if RUST_PATH.match(span):
                    ident = span.rstrip("()").split("::")[-1].rstrip("!")
                    kind = "rust"
                elif C_IDENT.match(span):
                    ident, kind = span, "c"
                else:
                    m = CMAKE_CALL.match(span)
                    if m:
                        ident = m.group(1)
                        # `nros_x(...)` is usually a C call, sometimes a
                        # CMake fn — accept either definition site.
                        # `nano_ros_x(...)` is CMake-only vocabulary.
                        kind = "cmake" if ident.startswith("nano_ros_") else "callable"
                if ident is None or ident in EXEMPT:
                    continue
                key = (kind, ident)
                if key not in seen:
                    if kind == "rust":
                        seen[key] = git_grep_word(ident, RUST_SPECS)
                    elif kind == "cmake":
                        seen[key] = cmake_defined(ident)
                    elif kind == "callable":
                        seen[key] = (cmake_defined(ident)
                                     or git_grep_word(ident, TREE_SPECS))
                    else:
                        seen[key] = git_grep_word(ident, TREE_SPECS)
                if not seen[key]:
                    failures.append(f"{rel}:{lineno}: `{span}` — {kind} "
                                    f"identifier `{ident}` not found in the tree")
    if failures:
        print("check-book-identifiers: the book quotes identifiers the tree "
              "does not define:\n", file=sys.stderr)
        for f in failures:
            print("  " + f, file=sys.stderr)
        print(f"\n  {len(failures)} stale quote(s). Fix the page, or add a "
              "justified EXEMPT entry in scripts/check-book-identifiers.py.",
              file=sys.stderr)
        sys.exit(1)
    print(f"book-identifiers OK ({len(seen)} distinct identifier(s) verified)")


if __name__ == "__main__":
    main()
