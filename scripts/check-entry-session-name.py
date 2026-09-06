#!/usr/bin/env python3
"""Every emitted `run_components` call must name a SESSION.

Issue 1003: ten `cmake/templates/*_entry_main*` templates called
`run_components(&__nros_entry_setup)`. One argument binds the delegating
overload that hard-codes `"node"`, so every image built through
`nano_ros_add_node` registered under that name — and the XRCE client key derives
from it, so a talker and a listener hashed to ONE client and the agent saw a
single peer. It is also the name `ros2 node list` shows.

The defect lived from 2026-06-13 to 2026-09-03 with a CORRECT sibling producer
beside it the whole time: the `nros build` path has passed the boot-config name
since 2026-06-27. Two spellings of one fact, and nothing compared them.

WHAT PHASE-432 W2.6 CHANGED, AND WHAT IS LEFT TO GUARD
------------------------------------------------------
This check was written as a STOPGAP: it held two producers together until one
of them could be deleted. W2.6 deleted it. `nano_ros_node_register()` no longer
renders its own entry TU — it calls `nros codegen entry-node`, which builds a
one-node plan and renders the SAME templates as `nros build`. The six
`cmake/templates/*_entry_main*.cpp.in` files are gone.

So the "compare two spellings" half is retired, because there is one spelling.
Three things survive, and none of them is subsumed by having one producer:

 1. THE OVERLOAD IS STILL REACHABLE. `<nros/main.hpp>` still declares the
    one-argument delegating overload that supplies `"node"` — it must, the
    header is a public API — and `cpp_boot_wrapper.cpp.jinja` still writes the
    call by hand. One producer means one place to get it wrong, not zero. The
    original defect is a single edit away.

 2. THE NAME MUST STILL COME FROM THE NODE. That is a property of the CMake
    side, which the shared pack cannot state: the emitter renders whatever name
    it is given, so a `--node-name` argument drifting to a literal would give
    every image built through the verb one name, with the templates left
    correct. That is issue 1003's collision reached by a different road.

 3. THE ARGUMENT MUST ACTUALLY BE PASSED. A `set(NROS_ENTRY_NODE_NAME …)` that
    reaches no invocation is the same silence as a template that substituted
    nothing.

WHY THIS CHECK DOES NOT KNOW ABOUT OVERLOADS (issue 1017)
--------------------------------------------------------
The obvious gate — "the call passes N arguments" — is wrong, and writing it
would reproduce the very defect it guards. The arity carrying the name differs
by board: `LinuxBoard` takes `(session_name, setup)` and has no locator
parameter, while zephyr/freertos/nuttx/threadx take `(locator, session_name,
setup)`. Encoding that here would make this file a THIRD authored copy of the
overload sets, free to drift from `main.hpp` and from the templates.

So the check asserts something weaker and more durable: whatever the arity, the
call must MENTION a session source. Every board satisfies that today, and a
future board with a fourth shape satisfies it without touching this file.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# The ONE producer of the emitted call (phase-432 W2.6). Named rather than
# globbed, deliberately: this gate refuses to report green on a producer that
# emits no call, so the next move of the emitter fails here loudly instead of
# silently leaving the file guarding nothing. That is how W2.3's move was
# caught, and it is why the entry stays a name.
PRODUCERS: list[tuple[str, str, str]] = [
    # (glob, marker that proves a session name is passed, human name)
    (
        "packages/cli/nros-cli-core/src/codegen/entry/packs/entry/cpp/"
        "boot_wrapper.jinja",
        "nros_boot_config_node_name",
        "the shared entry pack's boot wrapper",
    ),
]


def strip_tests(text: str) -> str:
    """Cut a Rust file at `#[cfg(test)]`.

    A Rust emitter's tests assert on the emitted C++ — including
    `!src.contains("::run_components(")` — and a scanner cannot tell a test's
    quoted fragment from a producer's. Tests are not producers, so they are not
    scanned.

    This assumes tests come last, which is the convention here and true today.
    The assumption is not silent: a producer file that yields NO call after this
    cut is a hard error below, so moving an emitter after the test module fails
    loudly rather than skipping it."""
    idx = text.find("#[cfg(test)]")
    return text if idx == -1 else text[:idx]


def code_only(text: str) -> str:
    """Drop comment lines, then join — so a comment mentioning the call is not
    mistaken for the call, and a call split across source lines still reads as
    one. A Rust emitter writes its call across a string continuation, and these
    files carry prose about `run_components` that must not be scanned."""
    out: list[str] = []
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("//") or s.startswith("///") or s.startswith("*"):
            continue
        # A Rust string continuation (`\\` at end of line) joins with NO
        # separator: an emitter that splits its `run_components(` call across
        # one would otherwise be truncated at the backslash — the scan then
        # reads `run_components(")` and reports a false positive.
        if line.rstrip().endswith("\\"):
            out.append(line.rstrip()[:-1])
        else:
            out.append(line + " ")
    return "".join(out)


def calls(blob: str) -> list[str]:
    """Every `run_components( … )` with balanced parens.

    Not a regex: the argument list nests (`nros_boot_config_node_name(&…)`), and
    `[^)]*` would stop at the inner close and silently read a truncated call —
    which is exactly the kind of check that passes while proving nothing."""
    out = []
    for m in re.finditer(r"run_components\s*\(", blob):
        depth, i = 0, m.end() - 1
        while i < len(blob):
            if blob[i] == "(":
                depth += 1
            elif blob[i] == ")":
                depth -= 1
                if depth == 0:
                    out.append(blob[m.start() : i + 1])
                    break
            i += 1
    return out


def scan(text: str, marker: str, is_rust: bool) -> list[str]:
    """Offending calls in `text` — the pure predicate the selftest drives."""
    if is_rust:
        text = strip_tests(text)
    return [c for c in calls(code_only(text)) if marker not in c]


# --------------------------------------------------------------------------
# The other half: the name must be PER NODE (issue 1017)
# --------------------------------------------------------------------------
#
# The check above proves a session name is PASSED. It cannot prove the names are
# DISTINCT — a producer that passed the same constant everywhere would satisfy
# it, and that is exactly the collision issue 1003 was: every image registering
# as `"node"`.
#
# Distinctness is not a property of the pack. The emitter renders the name it is
# handed, and whether two images differ depends on what CMake hands it.
# `NanoRosNodeRegister.cmake` derives it from the node's own name and passes it
# as `--node-name`; both halves are checked below, because either alone is
# satisfied by a broken build.

_NODE_NAME_ASSIGN = re.compile(r'set\s*\(\s*NROS_ENTRY_NODE_NAME\s+([^)]*)\)')
_EXPECTED_SOURCE = "${_NRC_NAME}"

# phase-432 W2.6 — the variable must REACH the emitter. Before this wave the
# templates read `@NROS_ENTRY_NODE_NAME@` and `configure_file` substituted it;
# now it is an argument, and an argument that is never passed is the same
# silence as a substitution that never happened.
_NODE_NAME_ARG = re.compile(r'--node-name\s+"\$\{NROS_ENTRY_NODE_NAME\}"')


def cmake_code_only(text: str) -> str:
    """Drop CMake comments.

    `code_only` strips `//` and `///` because its inputs are C++ and Rust; CMake
    comments start with `#`, and `#` cannot be stripped globally without eating
    Rust attributes like `#[cfg(test)]`. Found by the selftest: a commented-out
    `# set(NROS_ENTRY_NODE_NAME "node")` was being counted as a real site."""
    return "\n".join(
        line for line in text.splitlines() if not line.lstrip().startswith("#")
    )


def node_name_assignments(text: str) -> list[str]:
    """Every `set(NROS_ENTRY_NODE_NAME ...)` value, comments excluded."""
    return [
        m.group(1).strip() for m in _NODE_NAME_ASSIGN.finditer(cmake_code_only(text))
    ]


def check_node_name_is_per_node(text: str) -> list[str]:
    """Assignments that do NOT take the node's own name."""
    return [v for v in node_name_assignments(text) if _EXPECTED_SOURCE not in v]


def node_name_arg_sites(text: str) -> list[str]:
    """Every `--node-name "${NROS_ENTRY_NODE_NAME}"` argument, comments
    excluded."""
    return _NODE_NAME_ARG.findall(cmake_code_only(text))


# phase-432 W2.6 — the retired templates must stay retired.
#
# Six copies of one template is what let the session name be right in some
# entries and wrong in others. They are deleted, and the emitter is shared; a
# file reappearing here is a SECOND PRODUCER coming back, whatever its content.
_RETIRED_TEMPLATE_GLOB = "cmake/templates/*_entry_main*"


def retired_templates_on_disk() -> list[str]:
    """Any resurrected per-path entry template."""
    return sorted(p.name for p in ROOT.glob(_RETIRED_TEMPLATE_GLOB))


# --------------------------------------------------------------------------
# selftest — runs on the NORMAL path, never behind a flag (phase-395)
# --------------------------------------------------------------------------
#
# Real shapes, not paraphrases: each is lifted from the file it models, so the
# cases are evidence rather than a restatement of the rule.

_TPL_FIXED_KERNEL = """
    return static_cast<int>(
        {{ boot.board_path }}::run_components(NROS_ENTRY_LOCATOR, nros_boot_config_node_name(&NROS_BOOT_CONFIG), &__nros_entry_setup));
"""

_TPL_FIXED_HOST = """
    return {{ boot.board_path }}::run_components(nros_boot_config_node_name(&NROS_BOOT_CONFIG), &__nros_entry_setup);
"""

# Issue 1003 exactly as it stood: ONE argument, so the delegating overload
# supplies `"node"` and every image of that language collides on one client key.
_TPL_BROKEN = """
    return static_cast<int>(
        {{ boot.board_path }}::run_components(&__nros_entry_setup));
"""

# An emitter that splits the call across a Rust string continuation. Joining
# lines with a SPACE truncates it at the backslash and the scan reads
# `run_components(")` — a false positive this case exists to keep out.
_RS_CONTINUATION = (
    '                "    return static_cast<int>({board}::run_components(\\\n'
    'NROS_ENTRY_LOCATOR, nros_boot_config_node_name(&NROS_BOOT_CONFIG), '
    '&__nros_entry_setup));"\n'
)

# An emitter's own tests assert on its emitted text. Tests are not producers.
_RS_TEST_ONLY = (
    "#[cfg(test)]\n"
    "mod tests {\n"
    "    fn t() {\n"
    '        assert!(!src.contains("::run_components("));\n'
    "    }\n"
    "}\n"
)


def self_test(quiet: bool = True) -> int:
    cases = [
        # (name, text, marker, is_rust, expect_violation)
        ("fixed kernel/app shape (3-arg)", _TPL_FIXED_KERNEL, "nros_boot_config_node_name", False, False),
        ("fixed host shape (2-arg)", _TPL_FIXED_HOST, "nros_boot_config_node_name", False, False),
        ("issue-1003 shape", _TPL_BROKEN, "nros_boot_config_node_name", False, True),
        ("rust string continuation", _RS_CONTINUATION, "nros_boot_config_node_name", True, False),
        ("rust test-only mention", _RS_TEST_ONLY, "nros_boot_config_node_name", True, False),
    ]

    # The per-node half (issue 1017). A constant here collides every image built
    # through the verb while the pack stays correct — the hole the call check
    # above cannot see.
    name_cases: list[tuple[str, str, bool]] = [
        ("name taken from the node", 'set(NROS_ENTRY_NODE_NAME "${_NRC_NAME}")', False),
        ("name is a constant", 'set(NROS_ENTRY_NODE_NAME "node")', True),
        (
            "one of several sites drifted",
            'set(NROS_ENTRY_NODE_NAME "${_NRC_NAME}")\n'
            'set(NROS_ENTRY_NODE_NAME "entry")',
            True,
        ),
        (
            "a commented-out assignment is not a site",
            '# set(NROS_ENTRY_NODE_NAME "node")\n'
            'set(NROS_ENTRY_NODE_NAME "${_NRC_NAME}")',
            False,
        ),
    ]

    # phase-432 W2.6 — the variable must reach the emitter as an argument.
    # `expect` here is "the site is FOUND", the inverse polarity of the two
    # lists above, because this half asserts presence rather than absence.
    arg_cases: list[tuple[str, str, bool]] = [
        (
            "the invocation passes the node name",
            '        --node-name "${NROS_ENTRY_NODE_NAME}"\n',
            True,
        ),
        (
            "the invocation passes a literal instead",
            '        --node-name "node"\n',
            False,
        ),
        (
            "a commented-out argument is not a site",
            '#        --node-name "${NROS_ENTRY_NODE_NAME}"\n',
            False,
        ),
    ]

    bad = 0
    for name, text, marker, is_rust, expect in cases:
        got = bool(scan(text, marker, is_rust))
        if got != expect:
            bad += 1
            want = "a violation" if expect else "no violation"
            print(f"SELFTEST FAIL: {name}: expected {want}, got the opposite", file=sys.stderr)
        elif not quiet:
            print(f"  ok: {name}")
    for name, text, expect in name_cases:
        got = bool(check_node_name_is_per_node(text))
        if got != expect:
            bad += 1
            want = "a violation" if expect else "no violation"
            print(f"SELFTEST FAIL: {name}: expected {want}, got the opposite", file=sys.stderr)
        elif not quiet:
            print(f"  ok: {name}")
    for name, text, expect in arg_cases:
        got = bool(node_name_arg_sites(text))
        if got != expect:
            bad += 1
            want = "the site found" if expect else "no site"
            print(f"SELFTEST FAIL: {name}: expected {want}, got the opposite", file=sys.stderr)
        elif not quiet:
            print(f"  ok: {name}")

    if bad:
        print(
            "check-entry-session-name: SELFTEST FAILED — the check cannot be "
            "trusted about the tree until it is right about these.",
            file=sys.stderr,
        )
    elif not quiet:
        print(
            "check-entry-session-name selftest: OK "
            f"({len(cases) + len(name_cases) + len(arg_cases)} case(s))"
        )
    return 1 if bad else 0


def main() -> int:
    # The negative control runs BEFORE the tree is inspected: a gate that cannot
    # detect the defect it was written for must not report on anything else.
    if self_test() != 0:
        return 2
    failures: list[str] = []
    checked = 0

    # phase-432 W2.6 — a resurrected entry template is a second producer coming
    # back. Checked first and cheaply: if one exists, everything below is
    # reporting on half the tree.
    stray = retired_templates_on_disk()
    if stray:
        print(
            "check-entry-session-name: a per-path entry template is back on "
            "disk (issue 1003 / phase-432 W2.6):\n",
            file=sys.stderr,
        )
        for name in stray:
            print(f"  cmake/templates/{name}", file=sys.stderr)
        print(
            "\n  `nano_ros_node_register()` renders through the SHARED entry\n"
            "  pack now (`nros codegen entry-node` ->\n"
            "  codegen/entry/packs/entry/cpp/entry.cpp.jinja), so a template here\n"
            "  is a SECOND producer of the entry TU — the shape that let the\n"
            "  session name be right in some images and wrong in others for\n"
            "  three months. Extend the pack instead.",
            file=sys.stderr,
        )
        return 1

    # The name must also come FROM THE NODE, and must REACH the emitter — see
    # `check_node_name_is_per_node` and `node_name_arg_sites`. Checked before
    # the calls below: if the name is a constant, every call "passes" while
    # every image collides.
    reg = ROOT / "cmake/NanoRosNodeRegister.cmake"
    if not reg.is_file():
        print(
            f"ERROR: {reg.relative_to(ROOT)} is missing — it is what makes the\n"
            "       emitted name PER NODE, so this check cannot vouch for\n"
            "       distinctness without it.",
            file=sys.stderr,
        )
        return 2
    reg_text = reg.read_text(encoding="utf8")
    assigns = node_name_assignments(reg_text)
    if not assigns:
        print(
            "ERROR: no `set(NROS_ENTRY_NODE_NAME ...)` in "
            f"{reg.relative_to(ROOT)}.\n"
            "       That variable is what the entry invocation passes as\n"
            "       `--node-name`; if nothing sets it the name is empty and\n"
            "       every image shares it.",
            file=sys.stderr,
        )
        return 2
    bad = check_node_name_is_per_node(reg_text)
    if bad:
        print(
            "check-entry-session-name: a session name is not taken from the "
            "node (issue 1017):\n",
            file=sys.stderr,
        )
        for v in bad:
            print(f"  set(NROS_ENTRY_NODE_NAME {v})", file=sys.stderr)
        print(
            f"\n  Expected every site to use `{_EXPECTED_SOURCE}`. A constant "
            "here gives\n  every image built through that entry shape the SAME "
            "name, which is\n  issue 1003's collision with the emitter left "
            "correct.",
            file=sys.stderr,
        )
        return 1

    args = node_name_arg_sites(reg_text)
    if not args:
        print(
            "check-entry-session-name: `NROS_ENTRY_NODE_NAME` is set but never "
            "passed to the emitter (phase-432 W2.6):\n\n"
            "  expected `--node-name \"${NROS_ENTRY_NODE_NAME}\"` in the\n"
            "  `nros codegen entry-node` invocation.\n\n"
            "  The entry TU used to read `@NROS_ENTRY_NODE_NAME@` through\n"
            "  `configure_file`; it is an ARGUMENT now, and an argument that is\n"
            "  never passed is the same silence as a substitution that never\n"
            "  happened — the emitter would fall back to whatever its own\n"
            "  default is, for every image built through the verb.",
            file=sys.stderr,
        )
        return 1

    for glob, marker, label in PRODUCERS:
        paths = sorted(ROOT.glob(glob))
        if not paths:
            print(
                f"ERROR: `{glob}` matched no files. A producer that cannot be\n"
                f"       found is not a producer that is correct — refusing to\n"
                f"       report green having checked nothing.\n"
                f"       ({label})",
                file=sys.stderr,
            )
            return 2
        for path in paths:
            raw = path.read_text(encoding="utf8")
            if path.suffix == ".rs":
                raw = strip_tests(raw)
            blob = code_only(raw)
            found = calls(blob)
            if not found:
                print(
                    f"ERROR: {path.relative_to(ROOT)} is listed as a producer "
                    f"({label})\n       but emits no `run_components` call. "
                    f"Either it stopped being one\n       or the emitter moved; "
                    f"either way this check\n       is no longer guarding it.",
                    file=sys.stderr,
                )
                return 2
            for call in found:
                checked += 1
                if marker not in call:
                    rel = path.relative_to(ROOT)
                    failures.append(
                        f"  {rel}\n"
                        f"    {' '.join(call.split())}\n"
                        f"    -> no `{marker}`: this call names no session, so the\n"
                        f"       image registers as the hard-coded default and every\n"
                        f"       image of its language collides on one client key."
                    )

    if failures:
        print(
            "check-entry-session-name: a generated `run_components` call passes no "
            "session name (issue 1003):\n",
            file=sys.stderr,
        )
        print("\n".join(failures), file=sys.stderr)
        print(
            "\n  Pass the node's own name. The ARITY differs by board — `LinuxBoard`\n"
            "  is (session_name, setup) with no locator, the RTOS boards are\n"
            "  (locator, session_name, setup) — so copy the shape from the sibling\n"
            "  branch of the same template rather than assuming one form.",
            file=sys.stderr,
        )
        return 1

    if checked == 0:
        print(
            "ERROR: found no `run_components` call in any producer. Either the\n"
            "       emitter moved or this check's globs are stale; either way it\n"
            "       is guarding nothing.",
            file=sys.stderr,
        )
        return 2

    print(
        f"check-entry-session-name: OK ({checked} generated call(s) name a "
        f"session; {len(assigns)} cmake site(s) take it from the node and "
        f"{len(args)} pass it to the emitter)"
    )
    return 0


if __name__ == "__main__":
    if "--self-test" in sys.argv or "--selftest" in sys.argv:
        sys.exit(self_test(quiet=False))
    sys.exit(main())
