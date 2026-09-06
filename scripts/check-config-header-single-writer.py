#!/usr/bin/env python3
"""`*config_generated.h` has exactly ONE writer: the mirror script. Issue 0985.

The per-build sizes headers (`nros_config_generated.h`,
`nros_cpp_config_generated.h`) are mirrored into the in-tree include dirs by
`scripts/build/mirror-generated-header.sh`, which since issue 0978 knows the
precedence: prefer the leaf-INDEPENDENT copy in the shared cargo target dir,
fall back to the leaf's own. That precedence exists because a shared
`--target-dir` makes cargo run a build script once per (crate, feature set),
not once per leaf, so the leaf's own copy is present and arbitrarily old for
every leaf after the first.

A second writer does not merely duplicate that logic — it DEFEATS it. Issue
0985: `nros-cpp/CMakeLists.txt` healed the mirror at configure time with a
plain `file(COPY_FILE)` from `${CMAKE_CURRENT_BINARY_DIR}`, i.e. exactly the
stale source 0978 stopped trusting. It wrote a museum header over a correct one
AND stamped it with a new mtime, so ninja found the mirror's OUTPUT newer than
its trigger and skipped the command that would have fixed it. A repair for
drift that caused drift and then suppressed its own fix.

The whole 0088 -> 0114 -> 0122 -> 0123 -> 0245 -> 0268 -> 0978 -> 0985 family is
one shape: two ways to answer "which bytes are the current sizes header". This
gate keeps the answer to one.

Allowed: any invocation of `mirror-generated-header.sh` (that IS the writer),
and comments.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
HEADER = "config_generated"
WRITER = "mirror-generated-header.sh"

# cmake commands that write a file somewhere.
_WRITE_CMD = re.compile(r"\b(configure_file|file)\s*\(", re.I)


def strip_comments(text: str) -> str:
    """Blank out cmake `#` comments, preserving line numbering."""
    out = []
    for line in text.split("\n"):
        i = line.find("#")
        # A `#` inside a quoted string is not a comment; the only such cases in
        # this tree are inside strings with no `config_generated`, so a simple
        # rule is enough and errs toward keeping MORE text (never fewer hits).
        if i >= 0 and line.count('"', 0, i) % 2 == 0:
            line = line[:i]
        out.append(line)
    return "\n".join(out)


# `foreach(V "a_config_generated.h" ...)` / `set(V "...config_generated.h")` —
# a variable that CARRIES the header name. Issue 0985's own code named the
# header only through such a variable, so a literal-only scan passed on the
# very code it was written to catch. (Measured: the first version of this gate
# did exactly that.)
_BINDS = re.compile(r"\b(?:foreach|set)\s*\(\s*([A-Za-z_][A-Za-z0-9_]*)([^)]*)\)", re.I | re.S)


def header_vars(stripped: str) -> set[str]:
    return {m.group(1) for m in _BINDS.finditer(stripped) if HEADER in m.group(2)}


def offenders(text: str) -> list[tuple[int, str]]:
    """(line number, snippet) for every write command touching a sizes header."""
    stripped = strip_comments(text)
    hvars = header_vars(stripped)
    found = []
    for m in _WRITE_CMD.finditer(stripped):
        # Take the balanced argument span, capped so a runaway never scans the
        # whole file.
        depth, i, end = 0, m.end() - 1, None
        while i < len(stripped) and i < m.end() + 4000:
            if stripped[i] == "(":
                depth += 1
            elif stripped[i] == ")":
                depth -= 1
                if depth == 0:
                    end = i
                    break
            i += 1
        span = stripped[m.start(): (end + 1) if end else m.end() + 400]
        names_header = HEADER in span or any(
            f"${{{v}}}" in span for v in hvars
        )
        if names_header and WRITER not in span:
            line = stripped.count("\n", 0, m.start()) + 1
            found.append((line, " ".join(span.split())[:110]))
    return found


def self_test() -> None:
    """Runs on the NORMAL path — `check-gate-selftests`."""
    bad = 'file(COPY_FILE "${D}/nros_config_generated.h" "${E}/nros_config_generated.h")'
    assert offenders(bad), "a bare COPY_FILE of the sizes header must be caught"
    # Issue 0985's ACTUAL shape: the header is named through a loop variable, so
    # the write command's own text never contains `config_generated`. The first
    # version of this gate passed on this and had to be fixed — keep the case.
    via_var = ('foreach(_h "nros_cpp_config_generated.h" "nros_config_generated.h")\n'
               '  file(COPY_FILE "${B}/${_h}" "${I}/nros/${_h}" ONLY_IF_DIFFERENT)\n'
               'endforeach()')
    assert offenders(via_var), "a header named through a variable must be caught"
    # The real writer is allowed.
    ok = ('execute_process(COMMAND bash "${SH}/mirror-generated-header.sh" '
          '"${A}/nros_config_generated.h" "${B}" gen nros_config_generated.h "${C}")')
    assert not offenders(ok), "the mirror script itself must be allowed"
    # A comment naming the old spelling must not fire (issue 0985 leaves several).
    assert not offenders('# file(COPY_FILE) of nros_config_generated.h, retired'), \
        "a comment is not a writer"
    # An unrelated copy must not fire.
    assert not offenders('file(COPY "${X}/src" DESTINATION "${Y}")'), \
        "unrelated file(COPY) must not be flagged"


def main() -> int:
    self_test()

    files = sorted(
        list(REPO.glob("cmake/**/*.cmake"))
        + list(REPO.glob("packages/**/CMakeLists.txt"))
        + list(REPO.glob("packages/**/*.cmake"))
        + list(REPO.glob("zephyr/**/*.cmake"))
        + list(REPO.glob("integrations/**/*.cmake"))
    )
    # RELATIVE to the repo. Against the ABSOLUTE path this skipped every file
    # whenever the checkout itself lived under a directory called
    # `third-party` -- which is how the safety island vendors this repo -- and
    # the gate then reported "OK -- 0 cmake file(s)" while examining nothing.
    files = [f for f in files if "third-party" not in f.relative_to(REPO).parts]
    # A gate that scanned nothing must not read as a pass. This one had no such
    # floor, so the vacuous state was indistinguishable from a clean one; its
    # sibling `check-ret-code-citations` had the identical path bug AND a floor,
    # went red instead of green, and is how the class was found at all.
    if not files:
        print(
            "check-config-header-single-writer: scanned NO cmake files -- this "
            "gate would pass vacuously.",
            file=sys.stderr,
        )
        return 1

    hits = []
    for f in files:
        try:
            text = f.read_text(encoding="utf-8")
        except OSError:
            continue
        for line, snippet in offenders(text):
            hits.append((f.relative_to(REPO), line, snippet))

    if not hits:
        print(f"check-config-header-single-writer: OK — {len(files)} cmake file(s), "
              "the mirror script is the only writer.")
        return 0

    print("check-config-header-single-writer: a SECOND writer of the per-build "
          "sizes header.", file=sys.stderr)
    for path, line, snippet in hits:
        print(f"  {path}:{line}: {snippet}", file=sys.stderr)
    print("", file=sys.stderr)
    print("  Route it through scripts/build/mirror-generated-header.sh, which "
          "knows the", file=sys.stderr)
    print("  precedence (issue 0978: the leaf's own copy is present and "
          "arbitrarily old once", file=sys.stderr)
    print("  leaves share a cargo target dir). A second writer does not just "
          "duplicate that", file=sys.stderr)
    print("  logic — it stamps the mirror's OUTPUT with a new mtime and so "
          "suppresses the", file=sys.stderr)
    print("  ninja edge that would have corrected it (issue 0985).", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
