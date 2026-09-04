#!/usr/bin/env python3
"""issue 0737 — an example's message callback may not drop a sample silently.

A subscription callback that does

    if (deserialize(&msg, data, len) != 0) {
        return;
    }

turns "the message was rejected here" into "no output", and those two are the
same observation from outside the process. That ambiguity is what made 0737
cost two hosts a full investigation each: the sample was discovered, matched,
stored and TAKEN — Cyclone's own trace printed `take: returning 1` — while the
only symptom available to anyone was an absence of `Received:` lines. Every
layer below the callback had to be cleared by hand before the callback itself
became a suspect, and one line of output would have skipped all of it.

These are EXAMPLES, which makes it worse twice over: users copy them, and tests
GREP them. A silent arm in code a test reads for its verdict is the same defect
as a test that reports PASS on an unmet precondition, one level out.

So: a `return` inside a failed-deserialize arm must be preceded by a print in
the same arm. Not a rule about error handling — dropping is often right — a
rule about SAYING SO.

Run: python3 scripts/check-no-silent-sample-drop.py
"""
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.tracked import tracked  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
SCOPE = ["examples"]
# The arm opener: any `_deserialize(...)` used as a failure test.
OPENER = re.compile(r"_deserialize\s*\(.*\)\s*!=\s*0\s*\)\s*\{")
# `log_error`/`log_warn` are the post-phase-417 Rust spellings; `nros_error`/
# `nros_warn` stay listed because the C API and older prose still use them.
SAYS_SOMETHING = re.compile(
    r"\b(printf|fprintf|puts|log_error|log_warn|nros_error|nros_warn|NROS_LOG|std::cerr)\b"
)


def offenders(path: Path):
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return []
    out = []
    for i, line in enumerate(lines):
        if not OPENER.search(line):
            continue
        # Walk the arm to its closing brace (these are small, flat blocks).
        body, depth = [], 1
        for nxt in lines[i + 1 : i + 25]:
            depth += nxt.count("{") - nxt.count("}")
            if depth <= 0:
                break
            body.append(nxt)
        arm = "\n".join(body)
        if "return" in arm and not SAYS_SOMETHING.search(arm):
            out.append(f"{path.relative_to(ROOT)}:{i + 1}")
    return out


def main() -> int:
    files = [
        f
        for d in SCOPE
        for f in tracked(ROOT / d)
        if f.suffix in (".c", ".cpp", ".cc", ".h", ".hpp")
    ]
    bad = [o for f in files for o in offenders(f)]
    if bad:
        print("check-no-silent-sample-drop: FAIL\n", file=sys.stderr)
        for b in bad:
            print(f"  {b}: a rejected sample returns with no output", file=sys.stderr)
        print(
            "\n  Say what was dropped and why. From outside the process a silent\n"
            "  `return` and a message that never arrived are the SAME observation,\n"
            "  and issue 0737 spent two hosts' investigations on that ambiguity.\n"
            "  One `fprintf(stderr, ...)` in the arm is the whole fix.",
            file=sys.stderr,
        )
        return 1
    print(f"check-no-silent-sample-drop: OK ({len(files)} example source file(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main())
