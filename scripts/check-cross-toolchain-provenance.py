#!/usr/bin/env python3
"""A cross build must SAY which compiler it picked and where it came from.

Issue 1117. `nros-sdk-index.toml` pins `[tool.arm-none-eabi-gcc]` at ARM's
13.2.rel1. On a host whose SDK store had never been provisioned, `nros build
freertos` compiled with Ubuntu's `/usr/bin/arm-none-eabi-g++` 10.3.1 — three
major versions below the pin — and printed nothing about the substitution. The
resulting compile error was diagnosed twice as a nano-ros codegen bug
(docs/issues/archived/1113-*.md, retracted) and nearly produced a change to the
entry emitter to accommodate a compiler this project does not ship.

The fallback itself is DELIBERATE — activate.sh says so in as many words: "A
system cross-gcc (e.g. /usr/bin/arm-none-eabi-gcc) still resolves when the store
has none." A contributor with a working distro toolchain should not need a
150 MB download. So the rule is not "refuse the fallback"; it is the issue-0500
remedy one layer down, where `nano-ros: Corrosion <ver> via <origin>` made the
choice legible: RECORD IT.

WHAT IT CHECKS

For every toolchain file under `cmake/toolchain/`:

  R1  a file that sets CMAKE_C_COMPILER / CMAKE_CXX_COMPILER includes
      NanoRosCrossToolchain.cmake and calls BOTH resolve() and report()
  R2  the compiler is set from a ${variable}, not a literal `<prefix>-gcc`
  R3  the TOOL named is a real `[tool.<name>]` in nros-sdk-index.toml
  R4  resolve() and report() in one file agree — same TOOL, same OVERRIDE_VAR,
      and report()'s PREFIX/ORIGIN read the variables resolve() wrote

R4 is the one that is not obvious, and it is the reason this gate is worth
having rather than a grep for the helper name. A file can call both helpers,
pass every argument, look entirely correct — and report a DIFFERENT tool than
the one it configures, or read a stale variable some earlier version wrote. That
mutant is shape-valid: cmake accepts it, the configure prints a provenance line,
and the line is about the wrong compiler. A gate that only asks "is the helper
called" gives that mutant a pass, which is the same failure the rmw parity map
had — an authored claim agreeing with nothing.

R3 is the same idea pointed outward: the TOOL string is a name in another file,
so it can drift when the index is renamed. Checking it against the index means
the map cannot claim a territory that does not exist.

SIBLINGS OUTSIDE cmake/toolchain/, and why they are a listed gap

Four more places in this tree select a cross compiler and are NOT under
cmake/toolchain/. They have the identical defect and are not fixed here (they
belong to other owners); each is listed in EXEMPT with issue 1117 in the reason,
which is the only form of tolerated gap this repo accepts. The list can only
shrink: the gate fails if an exempt path stops existing or stops setting a
compiler, so a file that gets fixed must leave the list rather than sit in it
looking like coverage.

Usage::

    check-cross-toolchain-provenance.py            # the gate (self-tests first)
    check-cross-toolchain-provenance.py --selftest # the negative control alone
"""

import argparse
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLCHAIN_DIR = "cmake/toolchain"
MODULE = "NanoRosCrossToolchain.cmake"
INDEX = os.path.join(ROOT, "nros-sdk-index.toml")

# Files outside cmake/toolchain/ that select a cross compiler. Every entry
# names issue 1117: an unreported gap is indistinguishable from coverage.
EXEMPT = {
    "packages/boards/nros-board-nuttx-qemu/armv7a-nuttx-toolchain.cmake":
        "issue 1117 — same defect, board-owned file; reached via "
        "cmake/board/nano-ros-board-nuttx-qemu-arm.cmake",
    "packages/boards/nros-board-nuttx-qemu/riscv-nuttx-toolchain.cmake":
        "issue 1117 — same defect, board-owned file; reached via "
        "cmake/board/nano-ros-board-nuttx-qemu-riscv.cmake",
    "scripts/qemu/arm-none-eabi-cortex-m3.cmake":
        "issue 1117 — find_program on ${TOOLCHAIN_PREFIX}, so it resolves but "
        "still reports neither version nor origin",
    "integrations/s32ds/CMakeLists.txt":
        "issue 1117 — WRITES a toolchain file at configure time (deliberately "
        "derived from the S32DS project, not checked in); the emitted file "
        "hardcodes arm-none-eabi-gcc",
}

# `CMAKE_C_COMPILER_LAUNCHER` and friends are not compiler selection.
SET_COMPILER = re.compile(
    r"""(?:set|find_program)\s*\(\s*(CMAKE_(?:C|CXX|ASM)_COMPILER)\b(?!_)\s*([^\s\)]*)""")
INCLUDE_MODULE = re.compile(r"include\s*\([^)]*" + re.escape(MODULE))
TOOL_SECTION = re.compile(r"^\[tool\.([^\]]+)\]", re.M)


def strip_comments(text):
    """Drop `#` comments. Nothing in these files quotes a `#`."""
    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


# The keyword names each helper takes. A KNOWN set, not "any all-caps token":
# `NROS_ARM_NONE_EABI_PREFIX` and `_P` are also all-caps, and treating them as
# keywords silently split every call into nonsense — a parser that reads the
# compliant file as broken would have made this gate unlandable and, worse,
# could have been "fixed" by loosening the rule instead of the parser.
RESOLVE_KEYS = {"TOOL", "PREFIXES", "OVERRIDE_VAR", "OUT_PREFIX", "OUT_ORIGIN"}
REPORT_KEYS = {"TOOL", "PREFIX", "ORIGIN", "OVERRIDE_VAR"}


def call_args(text, fname, keys):
    """Parsed `KEY value…` pairs of the first `fname(...)` call, or None."""
    m = re.search(re.escape(fname) + r"\s*\(", text)
    if not m:
        return None
    depth, i = 1, m.end()
    while i < len(text) and depth:
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
        i += 1
    args, key = {}, None
    for tok in text[m.end(): i - 1].split():
        if tok in keys:
            key = tok
            args[key] = []
        elif key:
            args[key].append(tok.strip('"'))
    return args


def one(args, key):
    """The single value of `key`, or "" — never an IndexError."""
    vals = args.get(key) or [""]
    return vals[0]


def index_tools(read_index):
    return set(TOOL_SECTION.findall(read_index()))


def offenders(files, read, tools):
    """(path, message) for every rule broken. `files` are repo-relative."""
    bad = []
    for rel in sorted(files):
        raw = read(rel)
        text = strip_comments(raw)
        sets = SET_COMPILER.findall(text)
        if not sets:
            continue

        # R1 — the helper is used at all.
        if not INCLUDE_MODULE.search(text):
            bad.append((rel, f"sets a cross compiler but never includes {MODULE}"))
            continue
        resolve = call_args(text, "nros_cross_toolchain_resolve", RESOLVE_KEYS)
        report = call_args(text, "nros_cross_toolchain_report", REPORT_KEYS)
        if resolve is None or report is None:
            missing = "resolve()" if resolve is None else "report()"
            bad.append((rel, f"includes {MODULE} but never calls "
                             f"nros_cross_toolchain_{missing[:-2]}"))
            continue

        # R2 — the compiler comes from the resolved variable, not a literal.
        for var, value in sets:
            if "${" not in value:
                bad.append((rel, f"{var} is set to the literal `{value}` — a "
                                 f"resolved compiler must come from "
                                 f"${{{one(resolve, 'OUT_PREFIX')}}}"))

        # R3 — the tool name exists in the index.
        tool = one(resolve, "TOOL")
        if tool not in tools:
            bad.append((rel, f"resolve(TOOL {tool}) names no [tool.{tool}] in "
                             f"nros-sdk-index.toml"))

        # R4 — the two calls are wired to each other, not merely both present.
        rep_tool = one(report, "TOOL")
        if rep_tool != tool:
            bad.append((rel, f"report(TOOL {rep_tool}) disagrees with "
                             f"resolve(TOOL {tool}) — the provenance line would "
                             f"name a compiler this file does not configure"))
        r_ov = one(resolve, "OVERRIDE_VAR")
        p_ov = one(report, "OVERRIDE_VAR")
        if r_ov != p_ov:
            bad.append((rel, f"report(OVERRIDE_VAR {p_ov}) disagrees with "
                             f"resolve(OVERRIDE_VAR {r_ov}) — the remedy would "
                             f"name a knob that does nothing"))
        for arg, out in (("PREFIX", "OUT_PREFIX"), ("ORIGIN", "OUT_ORIGIN")):
            want = one(resolve, out)
            got = one(report, arg)
            if got != "${%s}" % want:
                bad.append((rel, f"report({arg} {got}) does not read "
                                 f"resolve({out} {want})"))
    return bad


def exempt_still_needed(read, exists):
    """An exempt path that no longer needs the exemption must leave the list."""
    bad = []
    for rel, why in sorted(EXEMPT.items()):
        if not exists(rel):
            bad.append((rel, "exempt path no longer exists — delete the EXEMPT entry"))
            continue
        if not SET_COMPILER.search(strip_comments(read(rel))):
            bad.append((rel, "exempt path no longer selects a cross compiler — "
                             "delete the EXEMPT entry"))
        if "1117" not in why:
            bad.append((rel, "exemption reason names no tracked issue id"))
    return bad


def tracked_toolchain_files():
    out = subprocess.run(
        ["git", "ls-files", f"{TOOLCHAIN_DIR}/*.cmake"],
        cwd=ROOT, capture_output=True, text=True, check=True).stdout.split()
    return [p for p in out if os.path.basename(p) != MODULE]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest(verbose=True)
    # On the NORMAL path, every time: a negative control nobody runs decays
    # into a comment (AGENTS.md, "a gate must run its own selftest").
    if selftest():
        return 1

    def read(rel):
        with open(os.path.join(ROOT, rel), encoding="utf8", errors="replace") as fh:
            return fh.read()

    with open(INDEX, encoding="utf8") as fh:
        index_text = fh.read()
    tools = index_tools(lambda: index_text)
    if not tools:
        print("check-cross-toolchain-provenance: parsed ZERO [tool.*] from "
              "nros-sdk-index.toml — the R3 check would be vacuous", file=sys.stderr)
        return 1

    files = tracked_toolchain_files()
    if not files:
        print(f"check-cross-toolchain-provenance: found no toolchain files under "
              f"{TOOLCHAIN_DIR}/ — this gate would examine nothing", file=sys.stderr)
        return 1

    bad = offenders(files, read, tools) + exempt_still_needed(
        read, lambda rel: os.path.exists(os.path.join(ROOT, rel)))
    if bad:
        print("check-cross-toolchain-provenance: FAILED", file=sys.stderr)
        for rel, msg in bad:
            print(f"  {rel}: {msg}", file=sys.stderr)
        print(
            "\nA cross build that does not name its compiler is issue 1117: the pin in\n"
            "nros-sdk-index.toml said 13.2.rel1, the build used a distro 10.3.1, and the\n"
            "failure read as a codegen bug for two diagnoses. The fallback is fine; the\n"
            "silence is not. Wire the file through cmake/toolchain/" + MODULE + ":\n"
            "    include(\"${CMAKE_CURRENT_LIST_DIR}/" + MODULE + "\")\n"
            "    nros_cross_toolchain_resolve(TOOL <index tool> PREFIXES <p>\n"
            "        OVERRIDE_VAR <NROS_..._PREFIX> OUT_PREFIX _P OUT_ORIGIN _O)\n"
            "    nros_cross_toolchain_report(TOOL <index tool> PREFIX \"${_P}\"\n"
            "        ORIGIN \"${_O}\" OVERRIDE_VAR <NROS_..._PREFIX>)\n"
            "    set(CMAKE_C_COMPILER ${_P}-gcc)   # etc\n",
            file=sys.stderr)
        return 1
    print(f"check-cross-toolchain-provenance: OK ({len(files)} toolchain file(s), "
          f"{len(EXEMPT)} listed gap(s))")
    return 0


# ---------------------------------------------------------------------------
GOOD = """
set(CMAKE_SYSTEM_NAME Generic)
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosCrossToolchain.cmake")
nros_cross_toolchain_resolve(
    TOOL         arm-none-eabi-gcc
    PREFIXES     arm-none-eabi
    OVERRIDE_VAR NROS_ARM_NONE_EABI_PREFIX
    OUT_PREFIX   _P
    OUT_ORIGIN   _O)
nros_cross_toolchain_report(
    TOOL   arm-none-eabi-gcc
    PREFIX "${_P}"
    ORIGIN "${_O}"
    OVERRIDE_VAR NROS_ARM_NONE_EABI_PREFIX)
set(CMAKE_C_COMPILER   ${_P}-gcc)
set(CMAKE_CXX_COMPILER ${_P}-g++)
"""

TOOLS = {"arm-none-eabi-gcc", "riscv-none-elf-gcc"}


def selftest(verbose=False):
    ok = fail = 0

    def chk(what, cond):
        nonlocal ok, fail
        if cond:
            ok += 1
            if verbose:
                print(f"  ok   {what}")
        else:
            fail += 1
            print(f"  FAIL {what}", file=sys.stderr)

    def run(body):
        return offenders(["t.cmake"], lambda _rel: body, TOOLS)

    chk("the compliant shape passes", run(GOOD) == [])

    # A file that selects no compiler is not this gate's business.
    chk("a non-toolchain cmake file is ignored",
        run("include(\"x.cmake\")\nadd_library(a b.c)\n") == [])

    # R1
    chk("R1 no module include",
        any("never includes" in m for _, m in run(
            "set(CMAKE_C_COMPILER arm-none-eabi-gcc)\n")))
    chk("R1 include but no report()",
        any("never calls nros_cross_toolchain_report" in m for _, m in run(
            GOOD.replace("nros_cross_toolchain_report", "nros_unrelated_call"))))

    # R2 — the wire-crossing that keeps the helper call and hardcodes anyway.
    chk("R2 helper called AND compiler hardcoded",
        any("literal `arm-none-eabi-gcc`" in m for _, m in run(
            GOOD.replace("set(CMAKE_C_COMPILER   ${_P}-gcc)",
                         "set(CMAKE_C_COMPILER   arm-none-eabi-gcc)"))))

    # R3
    chk("R3 tool absent from the index",
        any("names no [tool." in m for _, m in run(
            GOOD.replace("TOOL         arm-none-eabi-gcc",
                         "TOOL         arm-none-eabi-gcc-13"))))

    # R4 — every mutant below keeps the SHAPE valid: cmake accepts it, a
    # provenance line still prints, and it is about the wrong thing.
    chk("R4 report names a different TOOL than resolve",
        any("disagrees with resolve(TOOL" in m for _, m in run(
            GOOD.replace("    TOOL   arm-none-eabi-gcc",
                         "    TOOL   riscv-none-elf-gcc"))))
    chk("R4 report names a different OVERRIDE_VAR",
        any("name a knob that does nothing" in m for _, m in run(
            GOOD.replace("    OVERRIDE_VAR NROS_ARM_NONE_EABI_PREFIX)",
                         "    OVERRIDE_VAR NROS_RISCV64_PREFIX)"))))
    chk("R4 report reads a variable resolve never wrote",
        any("does not read resolve(OUT_PREFIX" in m for _, m in run(
            GOOD.replace('PREFIX "${_P}"', 'PREFIX "${_STALE}"'))))
    chk("R4 report's ORIGIN reads the PREFIX variable",
        any("does not read resolve(OUT_ORIGIN" in m for _, m in run(
            GOOD.replace('ORIGIN "${_O}"', 'ORIGIN "${_P}"'))))

    # The EXEMPT ratchet: an entry that stops being needed must be removed.
    chk("exempt path that vanished is reported",
        any("no longer exists" in m for _, m in exempt_still_needed(
            lambda _r: "", lambda _r: False)))
    chk("exempt path that stopped selecting a compiler is reported",
        any("no longer selects" in m for _, m in exempt_still_needed(
            lambda _r: "# nothing here\n", lambda _r: True)))

    if verbose:
        print(f"check-cross-toolchain-provenance --selftest: {ok} ok, {fail} failed")
    if fail:
        print(f"check-cross-toolchain-provenance: SELFTEST FAILED ({fail})",
              file=sys.stderr)
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
