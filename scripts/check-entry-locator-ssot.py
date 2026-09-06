#!/usr/bin/env python3
"""issue 0946 — the entry-locator SSoT gate.

`NROS_ENTRY_LOCATOR` is the address an embedded image dials. It was produced by
TWO independent per-platform ladders — `nano_ros_entry()` in NanoRosEntry.cmake
and the three RTOS typed-entry carriers in NanoRosNodeRegister.cmake — and they
DISAGREED on threadx and freertos. NanoRosEntry.cmake carried the instruction
"Mirrors NanoRosNodeRegister.cmake freertos branch; keep in sync", which is a
request that a human notice, and nothing checked it.

THE INVARIANT IS "ONE PRODUCER", NOT "EQUAL LITERALS".

This gate deliberately does NOT assert that the two lanes agree. They do not,
and that may well be correct: they cite different networks (a static-lwIP
192.0.3.0/24 gateway vs the QEMU slirp host 10.0.2.2), and deciding which is
right needs a QEMU run per lane rather than a reading. What is asserted is that
there is exactly ONE place a locator literal may be written, so the difference is
a visible choice in one table instead of a coincidence between two files nobody
diffs.

Why a check and not a convention: drift here is SILENT. A wrong locator builds
and links cleanly; the image simply connects nowhere, and the only symptom is a
test that times out with no delivery.

WHY THIS IS PYTHON AND NOT A GREP

The first version of this gate was shell, and its "every writer goes through the
resolver" rule was per-FILE: does this file mention `_nros_resolve_entry_locator`
anywhere? That passed a planted regression in which two of three writers were
switched back to their own defaults, because the third still called the
resolver. A file-level answer to a per-WRITE question is the issue-0196 shape —
a gate whose coverage is narrower than the rule it enforces. So the check reads
each write's right-hand side and demands it be a value the resolver produced.

Usage::

    check-entry-locator-ssot.py            # the gate
    check-entry-locator-ssot.py --audit    # show every site, never fails
"""

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# THE SSoT. The only file permitted to spell an entry-locator literal.
SSOT = "cmake/NanoRosEntryLocator.cmake"
RESOLVER = "_nros_resolve_entry_locator"

# A locator literal.
LOCATOR = re.compile(r"tcp/[0-9A-Za-z.\-]+:[0-9]+")
# A cmake comment line.
COMMENT = re.compile(r"^\s*#")
# A write of the locator: the cmake variable the entry templates substitute
# (`@NROS_ENTRY_LOCATOR@`), or the compile definition portable sources read.
# Group 1 is the right-hand side.
WRITE_SET = re.compile(r"""set\(\s*NROS_ENTRY_LOCATOR\s+"?\$\{([A-Za-z0-9_]+)\}""")
WRITE_DEF = re.compile(r"""NROS_ENTRY_LOCATOR=\\?"\$\{([A-Za-z0-9_]+)\}""")
# A write whose RHS is not a plain variable expansion at all (a literal, a
# concatenation). Caught separately so the message can say which it is.
WRITE_ANY = re.compile(r"""set\(\s*NROS_ENTRY_LOCATOR\s|NROS_ENTRY_LOCATOR=""")
# A resolver call. The out_var is the last argument; calls wrap across lines, so
# this is matched against the whole file with DOTALL.
RESOLVE_CALL = re.compile(
    RESOLVER + r"\s*\(\s*[^()]*?([A-Za-z0-9_]+)\s*\)", re.DOTALL
)
# A hand-sync instruction about the locator — the documented form of the defect
# this gate replaces.
SYNC_NOTE = re.compile(r"keep\s+(?:it\s+|them\s+)?in\s+sync", re.IGNORECASE)

# Files allowed to spell a `tcp/…` literal despite not being the SSoT.
#
# These emit `NROS_APP_CONFIG.locator` — a DIFFERENT symbol with a different
# consumer (the board's `startup.c`), mirroring each board crate's Rust
# `build.rs::emit_app_config_def` so a cmake build and a cargo build of the same
# board produce a byte-identical symbol. They are NOT the entry locator and must
# not be folded into it: measured on a real configure (issue 0946), a
# threadx-linux entry bakes `NROS_ENTRY_LOCATOR=tcp/127.0.0.1:7447` while this
# board file's app-config says `tcp/127.0.0.1:7555`, and the generated template
# states `.zenoh.locator` is cosmetic on the typed path. Folding them would
# change one or the other on a reading rather than a measurement. If that
# pairing is ever shown to matter it is its own issue, not this gate's business.
ALLOWED_LITERALS = {
    "cmake/board/nano-ros-board-threadx-linux.cmake",
    "cmake/board/nano-ros-board-riscv64-qemu.cmake",
}


def analyze(files):
    """The whole rule, over {path: text}. Returns a list of violation strings.

    Kept pure and content-addressed so the selftest can drive it with synthetic
    files and assert on real verdicts, rather than re-typing its regexes.
    """
    problems = []
    for path in sorted(files):
        text = files[path]
        lines = text.splitlines()

        # 1. Exactly one producer: no locator literal outside the SSoT and the
        #    documented app-config allowlist.
        if path != SSOT and path not in ALLOWED_LITERALS:
            for n, line in enumerate(lines, 1):
                if COMMENT.match(line):
                    continue
                if LOCATOR.search(line):
                    problems.append(
                        f"{path}:{n}: a locator literal outside {SSOT} — "
                        f"add a rung there and call {RESOLVER}() instead\n"
                        f"    {line.strip()}"
                    )

        # 2. Every WRITE of NROS_ENTRY_LOCATOR takes its value from a resolver
        #    out_var. Per-write, not per-file: a file may hold several writes and
        #    only some of them go through the resolver, which is exactly the
        #    regression the shell version of this gate let through.
        if path != SSOT:
            out_vars = set(RESOLVE_CALL.findall(text))
            # A resolver whose out_var IS the locator variable writes it directly.
            out_vars.add("NROS_ENTRY_LOCATOR")
            for n, line in enumerate(lines, 1):
                if COMMENT.match(line) or not WRITE_ANY.search(line):
                    continue
                m = WRITE_SET.search(line) or WRITE_DEF.search(line)
                if m is None:
                    # A write whose RHS is not a bare `${var}`. Rule 1 catches it
                    # when it is a literal; anything else is still opaque to this
                    # gate, so say so rather than passing it silently.
                    if not LOCATOR.search(line):
                        problems.append(
                            f"{path}:{n}: NROS_ENTRY_LOCATOR is written from an "
                            f"expression this gate cannot attribute to "
                            f"{RESOLVER}()\n    {line.strip()}"
                        )
                    continue
                rhs = m.group(1)
                if rhs not in out_vars:
                    problems.append(
                        f"{path}:{n}: NROS_ENTRY_LOCATOR is written from "
                        f"${{{rhs}}}, which no {RESOLVER}() call in this file "
                        f"produces — that is a second producer\n"
                        f"    {line.strip()}"
                    )

        # 3. A 'keep in sync' instruction about the locator must not come back.
        for n, line in enumerate(lines, 1):
            if SYNC_NOTE.search(line) and re.search(r"locator", line, re.I):
                problems.append(
                    f"{path}:{n}: a 'keep in sync' instruction about the locator "
                    f"— a hand-sync note has no enforcement; that is how the two "
                    f"ladders drifted\n    {line.strip()}"
                )
    return problems


def selftest():
    """Exercise the failure path on EVERY run (phase-395 ratchet).

    A negative control behind a `--selftest` flag is run once, by its author, on
    the day it is written; afterwards it is prose. This gate especially has to
    prove it can go red, because its whole subject is a check that did not exist
    while two ladders drifted apart unnoticed.

    These drive `analyze()` itself, so they test the shipping logic rather than a
    re-typed copy of its regexes.
    """
    def expect(name, files, want_hit, needle=None):
        got = analyze(files)
        hit = bool(got)
        if hit != want_hit:
            print(
                f"FAIL: check-entry-locator-ssot selftest '{name}': "
                f"expected {'a violation' if want_hit else 'no violation'}, "
                f"got {got!r}",
                file=sys.stderr,
            )
            sys.exit(1)
        if needle and not any(needle in g for g in got):
            print(
                f"FAIL: check-entry-locator-ssot selftest '{name}': "
                f"violation did not mention {needle!r} — got {got!r}",
                file=sys.stderr,
            )
            sys.exit(1)

    clean = (
        '    _nros_resolve_entry_locator(entry\n'
        '        "${NANO_ROS_PLATFORM}" "${NANO_ROS_BOARD}" _loc)\n'
        '    target_compile_definitions(t PRIVATE "NROS_ENTRY_LOCATOR=\\"${_loc}\\"")\n'
        '    set(NROS_ENTRY_LOCATOR "${_loc}")\n'
    )
    expect("compliant file passes", {"cmake/Ok.cmake": clean}, False)

    # A planted SECOND PRODUCER — a per-platform ladder rung somewhere else.
    expect(
        "second producer caught",
        {"cmake/Other.cmake": '    set(_loc "tcp/10.0.2.2:7447")\n'},
        True,
        "a locator literal outside",
    )

    # A COMMENT mentioning a locator must NOT red, or every explanatory note
    # about 10.0.2.2 becomes a failure (NanoRosEntry.cmake has one).
    expect(
        "comment is not a producer",
        {"cmake/Other.cmake": "    # bakes the default (tcp/127.0.0.1:7447) and never connects\n"},
        False,
    )

    # THE REGRESSION THE SHELL VERSION MISSED: a file that still calls the
    # resolver once, but whose other write takes its value from somewhere else.
    expect(
        "per-write, not per-file",
        {
            "cmake/Other.cmake": (
                '    _nros_resolve_entry_locator(node-register\n'
                '        "${NANO_ROS_PLATFORM}" "${NANO_ROS_BOARD}" _loc)\n'
                '    set(NROS_ENTRY_LOCATOR "${NROS_NUTTX_LOCATOR}")\n'
            )
        },
        True,
        "second producer",
    )

    # The hand-sync instruction.
    expect(
        "keep-in-sync note caught",
        {"cmake/Other.cmake": "    # Mirrors the other locator branch; keep in sync.\n"},
        True,
        "keep in sync",
    )

    # The allowlisted board app-config emitters keep their literal.
    expect(
        "app-config allowlist respected",
        {"cmake/board/nano-ros-board-threadx-linux.cmake":
            '"        .locator = \\"tcp/127.0.0.1:7555\\",\\n"\n'},
        False,
    )


def cmake_files():
    # `--others --exclude-standard` alongside `--cached`: a second producer
    # added but not yet committed must red NOW, while the author is looking at
    # it, not on someone else's push. Ignored paths stay out, so build trees
    # under cmake/ (there are none today) could never leak in.
    out = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard",
         "cmake/*.cmake", "cmake/**/*.cmake"],
        cwd=ROOT, capture_output=True, text=True, check=True,
    ).stdout.split()
    files = {}
    for rel in sorted(set(out)):
        p = os.path.join(ROOT, rel)
        if os.path.isfile(p):
            with open(p, encoding="utf-8", errors="replace") as fh:
                files[rel] = fh.read()
    return files



# --------------------------------------------------------------------------
# The header-side half (phase-432, W3.1 prerequisite)
# --------------------------------------------------------------------------
#
# The CMake half above makes the locator LITERAL have one producer. It says
# nothing about the C-preprocessor ladder that turns Kconfig into
# `NROS_ENTRY_LOCATOR` / `NROS_ENTRY_DOMAIN_ID` inside the headers, and that
# had THREE spellings:
#
#   * `<nros/main.hpp>` derived both from Kconfig (zenoh locator, else a
#     synthesised XRCE `host:port`, else `""`);
#   * `<nros/app_main.h>` — the C sibling — defined both as `""` and `0` with
#     no derivation at all, so a TU that saw only the C header dialled nothing
#     on a board whose locator comes from `CONFIG_NROS_ZENOH_LOCATOR`;
#   * a third `#ifndef` further down `main.hpp` for the locator-less NuttX
#     overload, unreachable because the ladder above it had always already
#     defined the macro — it promised `""` and delivered whatever the first
#     ladder produced.
#
# Same failure mode as the CMake half and the same reason it needs a gate
# rather than a convention: a wrong locator compiles, links and boots, and the
# only symptom is a connection that never happens.
#
# The rule is ONE DEFINING HEADER, not "the values agree" — deciding what a
# board should dial is a per-board question, and this file is not the place it
# gets answered.

HEADER_SSOT = "packages/api/nros-c/include/nros/entry_config.h"
HEADER_MACROS = ("NROS_ENTRY_LOCATOR", "NROS_ENTRY_DOMAIN_ID")
_DEFINE = re.compile(r"^\s*#\s*define\s+(NROS_ENTRY_LOCATOR|NROS_ENTRY_DOMAIN_ID)\b")


def header_files():
    """Every tracked-or-new C/C++ header and source under `packages/api/`.

    `--others --exclude-standard` for the same reason as `cmake_files()`: a
    second ladder added but not yet committed must red while its author is
    looking at it. Generated headers are gitignored and so stay out.
    """
    out = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "packages/api/"],
        cwd=ROOT, capture_output=True, text=True, check=True,
    ).stdout.split()
    files = {}
    for rel in sorted(set(out)):
        if not rel.endswith((".h", ".hpp", ".c", ".cpp", ".cc")):
            continue
        path = os.path.join(ROOT, rel)
        if os.path.isfile(path):
            with open(path, encoding="utf-8", errors="replace") as fh:
                files[rel] = fh.read()
    return files


def analyze_headers(files):
    """Definitions of the two entry macros outside the one defining header."""
    problems = []
    for rel, text in sorted(files.items()):
        if rel == HEADER_SSOT:
            continue
        for n, line in enumerate(text.splitlines(), 1):
            m = _DEFINE.match(line)
            if m:
                problems.append(
                    f"{rel}:{n}: defines {m.group(1)} — a second ladder. "
                    f"Include {HEADER_SSOT} instead."
                )
    return problems


def header_selftest():
    """The predicate must catch a planted second ladder and allow a mention.

    Without this the check could pass by examining nothing, which is the shape
    it exists to prevent one layer up.
    """
    planted = {
        HEADER_SSOT: "#define NROS_ENTRY_LOCATOR \"\"\n",
        "packages/api/x/other.h": "#define NROS_ENTRY_LOCATOR \"tcp/1.2.3.4:1\"\n",
    }
    if len(analyze_headers(planted)) != 1:
        print("SELFTEST FAIL: a planted second ladder was not caught.", file=sys.stderr)
        sys.exit(1)
    allowed = {
        HEADER_SSOT: "#define NROS_ENTRY_LOCATOR \"\"\n",
        # A mention, a doc comment and an `#ifdef` are all fine: the rule is
        # about who DEFINES the macro, not who reads it.
        "packages/api/x/reader.h": (
            "/* NROS_ENTRY_LOCATOR is derived in entry_config.h */\n"
            "#ifdef NROS_ENTRY_LOCATOR\n"
            "static const char* l = NROS_ENTRY_LOCATOR;\n"
            "#endif\n"
        ),
    }
    if analyze_headers(allowed):
        print("SELFTEST FAIL: a mention was reported as a definition.", file=sys.stderr)
        sys.exit(1)


def main():
    selftest()

    files = cmake_files()
    if SSOT not in files:
        print(f"FAIL: the entry-locator SSoT {SSOT} does not exist.", file=sys.stderr)
        return 1
    if f"function({RESOLVER}" not in files[SSOT]:
        print(f"FAIL: {SSOT} does not define {RESOLVER}().", file=sys.stderr)
        return 1

    if "--audit" in sys.argv:
        print(f"entry-locator SSoT: {SSOT}")
        for rel, text in files.items():
            for n, line in enumerate(text.splitlines(), 1):
                if LOCATOR.search(line) or WRITE_ANY.search(line):
                    kind = "LITERAL" if LOCATOR.search(line) else "write"
                    if COMMENT.match(line):
                        kind = "comment"
                    print(f"  {kind:8} {rel}:{n}: {line.strip()[:96]}")
        return 0

    problems = analyze(files)
    if problems:
        print("FAIL: the entry locator must have exactly ONE producer.", file=sys.stderr)
        print(
            "  A second ladder drifts silently — a wrong locator builds and links\n"
            "  cleanly, and the image just connects nowhere.\n",
            file=sys.stderr,
        )
        for p in problems:
            print(f"  {p}", file=sys.stderr)
        return 1

    header_selftest()
    headers = header_files()
    if HEADER_SSOT not in headers:
        print(
            f"FAIL: the header-side SSoT {HEADER_SSOT} does not exist.",
            file=sys.stderr,
        )
        return 1
    missing = [m for m in HEADER_MACROS if f"#define {m}" not in headers[HEADER_SSOT]]
    if missing:
        print(
            f"FAIL: {HEADER_SSOT} does not define {', '.join(missing)} — "
            "it is the SSoT and must be the thing that derives them.",
            file=sys.stderr,
        )
        return 1

    header_problems = analyze_headers(headers)
    if header_problems:
        print(
            "FAIL: the entry connect macros must have exactly ONE defining header.",
            file=sys.stderr,
        )
        print(
            "  A second ladder drifts silently — the image compiles, links and\n"
            "  boots, and simply dials the wrong address or none at all (#174).\n",
            file=sys.stderr,
        )
        for p in header_problems:
            print(f"  {p}", file=sys.stderr)
        return 1

    print(
        f"check-entry-locator-ssot: OK (one cmake producer: {SSOT}; "
        f"one header producer: {HEADER_SSOT}, {len(headers)} header(s) scanned)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
