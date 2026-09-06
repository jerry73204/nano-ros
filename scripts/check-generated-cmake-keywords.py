#!/usr/bin/env python3
"""A keyword the CLI writes into a generated CMakeLists must be PARSED.

Issue 1136. `nros` generates the workspace's root `CMakeLists.txt`
(`builder/cmake_root.rs`) as `nano_ros_workspace(...)` plus one
`nano_ros_add_executable(...)` per image. Both sides are literals in this tree,
and nothing compared them.

phase-405 W1 removed `LAUNCH_ARGS` from `nano_ros_entry`'s
`cmake_parse_arguments` on a survey that counted "authored users in the tree
(generated CMakeLists excluded, since those are tool output rather than a
caller's choice)". The generator is the only caller that runs in CI. cmake does
not reject an unknown keyword — it collects it into `UNPARSED_ARGUMENTS`, which
`nano_ros_add_executable` splices into `_srcs` and forwards as `SOURCES`, so two
words became positional sources and every workspace-fixture configure died on::

    CMake Error at cmake/NanoRosEntry.cmake:426 (add_executable):
      Cannot find source file:
        LAUNCH_ARGS

WHAT IT CHECKS

1. Every keyword `cmake_root.rs` can emit into a `nano_ros_*` call is in the
   keyword list of EVERY cmake frame that call traverses. `nano_ros_add_
   executable` is a forwarder, so a generated keyword has to survive two
   `cmake_parse_arguments` — the verb's and `nano_ros_entry`'s. Only the verb's
   frame is where the keyword ARRIVES; both are where it can be dropped.

2. Every keyword `nano_ros_add_executable` forwards through `_entry_extra` is
   parsed by `nano_ros_entry`. That is the same rule one frame later, and it
   had its own live violation: phase-405 W4 retired `MODEL` from
   `nano_ros_entry` and left the verb appending it, so a `MODEL <path>` reached
   the callee's `UNPARSED_ARGUMENTS` and was silently DROPPED — an entry built
   from the bringup default instead of the model it named.

WHY NOT JUST "DON'T EMIT IT"

`LAUNCH_ARGS host=<h>` is the ONLY difference between the `native_robot1` and
`native_robot2` images of `examples/workspaces/{c,mixed}`: it selects the
`[[model]]` variant, so without it both resolve the whole system and the two
images are one program. Measured, on the tree that removed it::

    nros model-path --launch multihost.launch.xml                    -> config/multihost_model.yaml
    nros model-path --launch multihost.launch.xml --arg host=robot1  -> config/multihost_robot1_model.yaml
    nros model-path --launch multihost.launch.xml --arg host=robot2  -> config/multihost_robot2_model.yaml

THE SIBLING, AND WHY IT CANNOT COVER THIS

`check-retired-cmake-keywords` asks the same question from the other end: does
any CALLER still pass a keyword some verb retired. It scans tracked cmake, and
a generated root is a build artifact under `build/<coord>/` that is tracked by
nothing — so the one caller that runs in CI is exactly the one it cannot see.
This gate reads the GENERATOR instead. Both are needed.

WHY A SKELETON AND NOT A GREP

The keywords are attributed to the call they appear in, which a grep cannot do:
`BACKEND` belongs to `nano_ros_workspace` and `BOARD` to
`nano_ros_add_executable`, and checking either against the other's keyword list
would be noise. So the emitter's `out.push_str(...)` literals are concatenated
in source order into the text skeleton the generator produces, and that
skeleton is parsed as cmake.

A generated call to a `nano_ros_*` verb this gate has no chain for is a
FAILURE, not a skip — otherwise a new generated verb escapes the rule by being
new.

Usage::

    check-generated-cmake-keywords.py [--selftest]
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

EMITTER = "packages/cli/nros-cli-core/src/builder/cmake_root.rs"

# Which cmake frames a generated call traverses. A keyword must survive EVERY
# one of them: `nano_ros_add_executable` re-emits into `nano_ros_entry`, and a
# keyword dropped at either end lands in SOURCES.
CHAINS: dict[str, list[tuple[str, str]]] = {
    "nano_ros_workspace": [("cmake/NanoRosWorkspace.cmake", "nano_ros_workspace")],
    "nano_ros_add_executable": [
        ("cmake/NanoRosVerbs.cmake", "nano_ros_add_executable"),
        ("cmake/NanoRosEntry.cmake", "nano_ros_entry"),
    ],
}

# Rule 2's forwarding vector and its destination frame.
FORWARDER = ("cmake/NanoRosVerbs.cmake", "nano_ros_add_executable")
FORWARD_VAR = "_entry_extra"
FORWARD_TARGET = ("cmake/NanoRosEntry.cmake", "nano_ros_entry")

# `out.push_str("…")` / `out.push_str(&format!("…", …))` — the emitter's only
# two writing spellings. `out.push('\n')` adds no keyword.
PUSH_RE = re.compile(r'out\.push_str\(\s*(?:&format!\(\s*)?"((?:[^"\\]|\\.)*)"', re.S)

# A cmake keyword as the generator writes it: indented, ALL CAPS, at the start
# of an emitted line.
KEYWORD_RE = re.compile(r"^[ \t]+([A-Z][A-Z0-9_]*)(?![a-z0-9_])")

# A call opening at column 0 in the emitted skeleton.
CALL_RE = re.compile(r"^([a-z_][a-z0-9_]*)\s*\(")


def rust_test_split(text: str) -> str:
    """The emitter's non-test half.

    `mod tests` asserts on rendered text with literals of the same shape; those
    are ASSERTIONS about the output, not the output.
    """
    m = re.search(r"^#\[cfg\(test\)\]", text, re.M)
    return text[: m.start()] if m else text


def unescape(lit: str) -> str:
    """Rust string-literal escapes, as far as this needs them.

    The line-continuation (`\\` before a real newline) comes FIRST and eats the
    following indentation, exactly as rustc does — without it a wrapped literal
    contributes phantom indented lines to the skeleton, and an indented line is
    what this gate reads as a keyword.
    """
    lit = re.sub(r"\\\n[ \t]*", "", lit)
    lit = re.sub(r"\\x([0-9A-Fa-f]{2})", lambda m: chr(int(m.group(1), 16)), lit)
    return lit.replace("\\n", "\n").replace("\\t", "\t").replace('\\"', '"').replace("\\\\", "\\")


def skeleton(rust: str) -> str:
    """The cmake text `cmake_root.rs` writes, with `{…}` placeholders intact."""
    return "".join(unescape(m.group(1)) for m in PUSH_RE.finditer(rust_test_split(rust)))


def emitted_keywords(text: str) -> dict[str, set[str]]:
    """`{call name: keywords}` over a generated cmake skeleton.

    Depth is counted over parentheses OUTSIDE `{…}` placeholders and outside
    double quotes, so `"{ws_rel}"` and a quoted path with a paren in it do not
    close the call.
    """
    out: dict[str, set[str]] = {}
    call: str | None = None
    depth = 0
    for line in text.splitlines():
        if depth == 0:
            m = CALL_RE.match(line)
            if m:
                call = m.group(1)
                out.setdefault(call, set())
        elif call:
            k = KEYWORD_RE.match(line)
            if k:
                out[call].add(k.group(1))
        depth += _paren_delta(line)
        if depth <= 0:
            depth, call = 0, None
    return out


def _paren_delta(line: str) -> int:
    delta = 0
    in_str = in_placeholder = False
    for c in line:
        if c == '"':
            in_str = not in_str
        elif c == "{":
            in_placeholder = True
        elif c == "}":
            in_placeholder = False
        elif not in_str and not in_placeholder:
            if c == "(":
                delta += 1
            elif c == ")":
                delta -= 1
    return delta


def function_body(cmake: str, name: str) -> str:
    """The body of `function(<name> …)`, up to its `endfunction()`."""
    m = re.search(r"^function\(\s*" + re.escape(name) + r"[\s)]", cmake, re.M)
    if not m:
        return ""
    end = re.compile(r"^endfunction\(", re.M).search(cmake, m.end())
    return cmake[m.end() : end.start() if end else len(cmake)]


def parsed_keywords(cmake: str, name: str) -> set[str]:
    """The keywords `<name>`'s first `cmake_parse_arguments` accepts.

    All three lists — options, one-value, multi-value — are one namespace as
    far as "does this word get consumed" goes, which is the question here.
    """
    body = function_body(cmake, name)
    m = re.search(r"cmake_parse_arguments\(([^)]*)\)", body, re.S)
    if not m:
        return set()
    lists = re.findall(r'"([^"]*)"', m.group(1))
    words: set[str] = set()
    for group in lists:
        words.update(w for w in group.split(";") if w)
    return words


def forwarded_keywords(cmake: str, name: str, var: str) -> set[str]:
    """Uppercase tokens appended to `<var>` in `<name>`'s body.

    Narrow on purpose: `list(APPEND _entry_extra <KW> …)` is the ONE vector by
    which this verb hands optional keywords on, and it is where `MODEL`
    outlived its callee. The fixed arguments of the `nano_ros_entry(…)` call
    itself are covered by rule 1.
    """
    body = function_body(cmake, name)
    found: set[str] = set()
    for m in re.finditer(r"list\(\s*APPEND\s+" + re.escape(var) + r"\s+([A-Z][A-Z0-9_]*)", body):
        found.add(m.group(1))
    return found


def findings(read):
    """`(rule, detail)` pairs; empty means the emitter and the parsers agree."""
    bad = []
    emitted = emitted_keywords(skeleton(read(EMITTER)))
    for call, keywords in sorted(emitted.items()):
        if not call.startswith("nano_ros_"):
            continue  # a cmake builtin: project(), find_package(), set().
        if call not in CHAINS:
            bad.append(("chain", f"generated call `{call}(…)` has no frame chain in this gate"))
            continue
        for path, fn in CHAINS[call]:
            accepted = parsed_keywords(read(path), fn)
            for kw in sorted(keywords - accepted):
                bad.append(
                    ("emit", f"{EMITTER} emits `{call}(… {kw} …)`, {path}'s {fn}() does not parse it")
                )
    fwd_path, fwd_fn = FORWARDER
    tgt_path, tgt_fn = FORWARD_TARGET
    accepted = parsed_keywords(read(tgt_path), tgt_fn)
    for kw in sorted(forwarded_keywords(read(fwd_path), fwd_fn, FORWARD_VAR) - accepted):
        bad.append(
            ("forward", f"{fwd_path}'s {fwd_fn}() forwards `{kw}`, {tgt_path}'s {tgt_fn}() does not parse it")
        )
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest(verbose=True)
    # On the normal path too: a negative control nobody runs decays into a
    # comment (AGENTS.md, "a gate must run its own selftest").
    selftest()

    read = lambda rel: (REPO / rel).read_text(errors="replace")  # noqa: E731
    bad = findings(read)
    if bad:
        print("check-generated-cmake-keywords: FAILED", file=sys.stderr)
        for _, detail in bad:
            print(f"  {detail}", file=sys.stderr)
        print(
            "\ncmake does not reject an unknown keyword — it collects it into\n"
            "UNPARSED_ARGUMENTS, which `nano_ros_add_executable` splices into `_srcs`\n"
            "and forwards as SOURCES. The keyword then reaches `add_executable` and the\n"
            "configure dies naming a missing SOURCE FILE (issue 1136). Restore the\n"
            "keyword in the parse list, or stop emitting it — but read what it carries\n"
            "first: `LAUNCH_ARGS` is the whole difference between the per-host images.",
            file=sys.stderr,
        )
        return 1
    calls = sum(1 for c in emitted_keywords(skeleton(read(EMITTER))) if c.startswith("nano_ros_"))
    print(f"check-generated-cmake-keywords: OK ({calls} generated nano_ros calls)")
    return 0


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

    emitter = r'''
fn render() -> String {
    let mut out = String::new();
    out.push_str("cmake_minimum_required(VERSION 3.20)\n");
    out.push_str(&format!("project({} LANGUAGES C CXX)\n", name));
    out.push_str("nano_ros_workspace(\n");
    out.push_str(&format!("    BACKEND  {}\n", spec.rmw));
    out.push_str("    ORDER_FROM_DEPENDS\n");
    out.push_str(")\n");
    out.push_str(&format!("nano_ros_add_executable({}\n", e.name));
    out.push_str(&format!("    LAUNCH  {}\n", e.launch));
    for (k, v) in &e.args {
        out.push_str(&format!("    LAUNCH_ARGS {k}={v}\n"));
    }
    out.push_str("    TYPED\n");
    out.push_str(&format!("    DEPLOY  {})\n", e.deploy));
    out
}
#[cfg(test)]
mod tests {
    fn t() { assert!(body.contains("    NEVER_EMITTED x\n")); }
}
'''
    ws = 'function(nano_ros_workspace)\n  cmake_parse_arguments(_NRW\n    "ORDER_FROM_DEPENDS"\n    "BACKEND"\n    "SUBDIRS"\n    ${ARGN})\nendfunction()\n'
    verbs_ok = (
        "function(nano_ros_add_executable name)\n"
        '  cmake_parse_arguments(_NRE "TYPED" "LAUNCH" "DEPLOY;LAUNCH_ARGS" ${ARGN})\n'
        "  list(APPEND _entry_extra LAUNCH ${_NRE_LAUNCH})\n"
        "endfunction()\n"
    )
    entry_ok = (
        "function(nano_ros_entry)\n"
        '  cmake_parse_arguments(_NRA "TYPED" "LAUNCH" "DEPLOY;LAUNCH_ARGS" ${ARGN})\n'
        "endfunction()\n"
    )
    files = {
        EMITTER: emitter,
        "cmake/NanoRosWorkspace.cmake": ws,
        "cmake/NanoRosVerbs.cmake": verbs_ok,
        "cmake/NanoRosEntry.cmake": entry_ok,
    }
    run = lambda over=None: findings((files | (over or {})).get)  # noqa: E731

    chk("an emitter whose keywords are all parsed is clean", run() == [])

    # The 1136 mutation, on each frame in turn.
    drop_entry = entry_ok.replace(";LAUNCH_ARGS", "")
    chk(
        "dropping LAUNCH_ARGS from the CALLEE is caught",
        [r for r, _ in run({"cmake/NanoRosEntry.cmake": drop_entry})] == ["emit"],
    )
    drop_verb = verbs_ok.replace(";LAUNCH_ARGS", "")
    chk(
        "dropping it from the receiving VERB is caught too",
        [r for r, _ in run({"cmake/NanoRosVerbs.cmake": drop_verb})] == ["emit"],
    )
    chk(
        "the message names the keyword and the file that stopped parsing it",
        "LAUNCH_ARGS" in run({"cmake/NanoRosEntry.cmake": drop_entry})[0][1]
        and "NanoRosEntry" in run({"cmake/NanoRosEntry.cmake": drop_entry})[0][1],
    )

    # Rule 2 — the MODEL shape: forwarded by the verb, unparsed by the callee.
    fwd_model = verbs_ok.replace(
        "  list(APPEND _entry_extra LAUNCH", "  list(APPEND _entry_extra MODEL ${_NRE_MODEL})\n  list(APPEND _entry_extra LAUNCH"
    )
    chk(
        "a keyword forwarded into _entry_extra but unparsed by the callee is caught",
        [r for r, _ in run({"cmake/NanoRosVerbs.cmake": fwd_model})] == ["forward"],
    )

    # Attribution: keywords belong to the call they appear in.
    kws = emitted_keywords(skeleton(emitter))
    chk("BACKEND is attributed to nano_ros_workspace", kws["nano_ros_workspace"] == {"BACKEND", "ORDER_FROM_DEPENDS"})
    chk(
        "LAUNCH/LAUNCH_ARGS/TYPED/DEPLOY are attributed to the entry call",
        kws["nano_ros_add_executable"] == {"LAUNCH", "LAUNCH_ARGS", "TYPED", "DEPLOY"},
    )
    chk("a `#[cfg(test)]` assertion is not an emission", "NEVER_EMITTED" not in str(kws))
    chk("cmake builtins are collected but not ruled on", "project" in kws)

    # A new generated verb must not escape by being new.
    plus = emitter.replace(
        '    out\n}',
        '    out.push_str("nano_ros_new_verb(\\n    THING\\n)\\n");\n    out\n}',
    )
    chk(
        "a generated nano_ros call with no declared chain FAILS",
        [r for r, _ in findings((files | {EMITTER: plus}).get)] == ["chain"],
    )

    # The parser itself, against the real files — a rule that reads nothing
    # passes for the wrong reason.
    real = lambda rel: (REPO / rel).read_text(errors="replace")  # noqa: E731
    chk("the real emitter yields a non-empty skeleton", len(skeleton(real(EMITTER))) > 200)
    for path, fn in CHAINS["nano_ros_add_executable"]:
        chk(f"{fn}() has a readable keyword list", len(parsed_keywords(real(path), fn)) >= 5)
    chk(
        "the real verb forwards at least one keyword",
        len(forwarded_keywords(real(FORWARDER[0]), FORWARDER[1], FORWARD_VAR)) > 0,
    )

    if verbose:
        print(f"\n{ok} passed, {fail} failed")
    if fail:
        print("check-generated-cmake-keywords self-test: FAILED", file=sys.stderr)
        raise SystemExit(1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
