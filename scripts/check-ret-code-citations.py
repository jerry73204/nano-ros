#!/usr/bin/env python3
"""Gate: a doc may not name an `NROS_RET_*` / `NROS_RMW_RET_*` code that no header defines.

TWO ABIs, ONE PREFIX FAMILY, AND THE COMPILER CHECKS NEITHER COMMENT
--------------------------------------------------------------------
`nros_ret_t` (the USER ABI, `packages/api/nros-c/src/error.rs` -> the generated
`nros_generated.h`) and `nros_rmw_ret_t` (the RMW ABI,
`packages/core/nros-rmw-abi/include/nros/rmw_ret.h`) have overlapping but
DIFFERENT vocabularies, and the difference is not guessable:

    BUFFER_TOO_SMALL   RMW only  — `NROS_RET_BUFFER_TOO_SMALL` does not exist
    TRY_AGAIN          user only — `NROS_RMW_RET_TRY_AGAIN` does not exist

Both headers are in scope in the same TU, so a comment that spells the wrong
half is a plausible sentence that compiles into nothing. The caller who trusts
it gets "use of undeclared identifier", with nothing pointing back at the doc
that told them to write it.

WHY A GATE, WHEN THE ISSUE SAID ONE INSTANCE
--------------------------------------------
Issue 1126 reported one site and closed its own gate question with a sweep:
`grep -rn 'NROS_RET_BUFFER_TOO_SMALL' packages/` returns two hits, one authored
and one generated, therefore not a class. That sweep asked about ONE SPELLING.
Asking the general question — every token of the family, checked against what
the headers define — found three live sites, two of them never reported:

  * `packages/api/nros-c/src/publisher.rs`  `NROS_RET_BUFFER_TOO_SMALL` (1126)
  * `packages/core/nros-rmw-abi/include/nros/rmw_vtable.h`  `..._TRY_AGAIN`,
    in the header that IS the ABI contract a third-party backend writes against
  * `packages/core/nros-rmw/src/custom_transport.rs`  `..._ALREADY_INIT`,
    promising a behaviour as well as a constant, neither of which exists

plus two in the book. A class whose members are invisible to a
one-spelling grep is exactly the kind that gets re-filed.

HOW TO WRITE ABOUT A CODE THAT DOES NOT EXIST
----------------------------------------------
There is no exemption list, deliberately: drop the prefix. `a future
INCOMPATIBLE_TYPE code` and `it used to say TRY_AGAIN` carry the same meaning
without spelling an identifier a reader can copy into C. Every legitimate
mention this gate met — a forward reference in the book, three corrections
explaining the old wrong name — reads better that way, so the rule that closes
the class costs nothing to obey. Exemptions would reintroduce it: a list of
"spellings allowed to be wrong" is indistinguishable from the defect.

SCOPE
-----
Tracked `.rs/.h/.hpp/.c/.cpp/.md` under `packages/`, `book/src/`, `docs/` and
`examples/`. `archived/` is excluded — an archived issue or a retired phase doc
is a RECORD of what was said at the time, and rewriting history to satisfy a
gate is worse than the stale spelling. Vendored and codegen'd trees are
excluded because we do not author them.

Note `check-book-identifiers.py` does not overlap: its `C_IDENT` is lowercase
by construction (`nros_[a-z0-9_]+`), so no uppercase constant was ever in its
reach, and it sees only `book/`.

Run:  python3 scripts/check-ret-code-citations.py [--self-test]
Gate: just check ret-code-citations
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "lib"))
from tracked import tracked  # issue 0721: index lookup, not a walk  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent

USER_ABI_CONSTS = ROOT / "packages" / "api" / "nros-c" / "src" / "error.rs"
RMW_ABI_HEADER = ROOT / "packages" / "core" / "nros-rmw-abi" / "include" / "nros" / "rmw_ret.h"

SCAN_ROOTS = ("packages", "book/src", "docs", "examples")
SUFFIXES = (".rs", ".h", ".hpp", ".c", ".cpp", ".md")
SKIP_PARTS = ("third-party", "archived", "generated")

TOKEN = re.compile(r"\bNROS_(?:RMW_)?RET_[A-Z0-9_]+")

# A harvest that silently returns few names would make this gate pass on
# everything. Both floors are well under today's counts (19 user, 22 RMW) and
# well over "the regex stopped matching".
MIN_USER = 10
MIN_RMW = 10


def defined_codes():
    """(names, problem). The two ABIs' vocabularies, read from their SSoT."""
    user = set(
        re.findall(
            r"pub\s+const\s+(NROS_RET_[A-Z0-9_]+)\s*:\s*nros_ret_t",
            USER_ABI_CONSTS.read_text(encoding="utf-8", errors="replace"),
        )
    )
    rmw = set(
        re.findall(
            r"^\s*#\s*define\s+(NROS_RMW_RET_[A-Z0-9_]+)\b",
            RMW_ABI_HEADER.read_text(encoding="utf-8", errors="replace"),
            re.MULTILINE,
        )
    )
    # `NROS_RMW_RET_H` is the file's include guard, not a code — but it IS
    # defined, and the three places it appears are the guard itself. Leaving it
    # in the set costs nothing (a doc naming it is odd, not wrong) and keeps
    # the harvest a plain reading of the header rather than a reading plus a
    # list of exceptions.
    problem = None
    if len(user) < MIN_USER or len(rmw) < MIN_RMW:
        problem = (
            f"harvest looks broken: {len(user)} user code(s) from "
            f"{USER_ABI_CONSTS.relative_to(ROOT)}, {len(rmw)} RMW code(s) from "
            f"{RMW_ABI_HEADER.relative_to(ROOT)}. Refusing to judge citations "
            f"against a vocabulary this small — fix the harvest, not the docs."
        )
    return user | rmw, problem


def citations(text, defined):
    """[(lineno, token)] for every token the ABIs do not define."""
    out = []
    for n, line in enumerate(text.splitlines(), 1):
        for tok in TOKEN.findall(line):
            if tok not in defined:
                out.append((n, tok))
    return out


def files():
    for root in SCAN_ROOTS:
        for p in tracked(ROOT / root):
            if p.suffix not in SUFFIXES:
                continue
            if any(part in SKIP_PARTS for part in p.parts):
                continue
            yield p


def self_test():
    defined = {"NROS_RET_FULL", "NROS_RMW_RET_BUFFER_TOO_SMALL"}
    cases = [
        ("returns `NROS_RET_FULL` on overflow", []),
        ("returns `NROS_RMW_RET_BUFFER_TOO_SMALL`", []),
        # The two real defects, one per direction of the prefix confusion.
        ("returns `NROS_RET_BUFFER_TOO_SMALL`", ["NROS_RET_BUFFER_TOO_SMALL"]),
        ("returns `NROS_RMW_RET_TRY_AGAIN`", ["NROS_RMW_RET_TRY_AGAIN"]),
        # The sanctioned way to talk about a code that does not exist.
        ("a future `INCOMPATIBLE_TYPE` code", []),
        ("it used to say `TRY_AGAIN`", []),
        # A longer name is a different token, not a prefix match.
        ("`NROS_RET_FULLY_QUALIFIED`", ["NROS_RET_FULLY_QUALIFIED"]),
    ]
    bad = 0
    for text, want in cases:
        got = [tok for _, tok in citations(text, defined)]
        if got != want:
            print(f"  self-test FAIL: {text!r} -> {got}, want {want}")
            bad += 1
    if bad:
        print(f"check-ret-code-citations self-test: {bad} case(s) FAILED")
        return 1
    print(f"check-ret-code-citations self-test: OK ({len(cases)} cases)")
    return 0


def main(argv):
    if "--self-test" in argv:
        return self_test()
    if self_test() != 0:
        return 1

    defined, problem = defined_codes()
    if problem:
        print(f"check-ret-code-citations: {problem}", file=sys.stderr)
        return 1

    scanned = 0
    bad = []
    for path in files():
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            print(f"check-ret-code-citations: cannot read {path}: {exc}", file=sys.stderr)
            return 1
        scanned += 1
        for lineno, tok in citations(text, defined):
            bad.append((path.relative_to(ROOT), lineno, tok))

    if not scanned:
        print(
            "check-ret-code-citations: scanned NO files — this gate would pass "
            "vacuously.",
            file=sys.stderr,
        )
        return 1

    if bad:
        print(
            f"check-ret-code-citations: {len(bad)} citation(s) of a return code "
            f"neither ABI defines:",
            file=sys.stderr,
        )
        for rel, lineno, tok in bad:
            other = (
                tok.replace("NROS_RET_", "NROS_RMW_RET_")
                if not tok.startswith("NROS_RMW_")
                else tok.replace("NROS_RMW_RET_", "NROS_RET_")
            )
            hint = f" (the other ABI defines `{other}`)" if other in defined else ""
            print(f"  {rel}:{lineno}: `{tok}`{hint}", file=sys.stderr)
        print(
            "\n  Name the code the function actually returns. To write ABOUT a\n"
            "  code that does not exist, drop the prefix (`a future\n"
            "  INCOMPATIBLE_TYPE code`) — there is no exemption list, because a\n"
            "  list of spellings allowed to be wrong is the defect itself.\n"
            "  Issue 1126.",
            file=sys.stderr,
        )
        return 1

    print(
        f"check-ret-code-citations: OK — {scanned} file(s), every "
        f"NROS_RET_*/NROS_RMW_RET_* citation is one of the {len(defined)} "
        f"codes the two ABIs define"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
