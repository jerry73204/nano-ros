#!/usr/bin/env python3
"""issue 1033 — a RETIRED cmake API keyword must survive in no caller.

# The failure this prevents

Our public cmake verbs retire a keyword by KEEPING IT PARSED and raising a
`FATAL_ERROR` naming the replacement, so an old caller fails loudly instead of
having the keyword and its values fall into `UNPARSED_ARGUMENTS` and become
source files nobody can find. That is the right shape — but it only fires when
somebody CONFIGURES the caller, and this repo has callers that only a Zephyr
SDK + west lane configures. Nothing on the `pull_request` lane does.

So phase-412 retired `nano_ros_node_register(... ENTITIES ...)`, migrated the
six `examples/workspaces/cpp` packages, and MISSED the six standalone
`examples/zephyr/cpp` leaves that issue 1033 had taught to declare the day
before. All six — and the 19 `examples/fixtures.toml` rows over them, across
zenoh, xrce AND cyclonedds, since the error is raised before the RMW matters —
could not CONFIGURE on `main`, and stayed that way because the only lane that
would have said so is one the schedule reaches.

The retirement is a good mechanism reporting to nobody. This gate is the
reader.

# What it checks

The retired set is DISCOVERED from the cmake sources, never hand-listed: every
`message(FATAL_ERROR "<fn>(...): <KEYWORD> was retired|removed ...")` under
`cmake/` contributes one `(function, keyword)` pair. So retiring the next
keyword arms this gate by itself, and a retirement whose wording moves makes
the gate BLIND rather than silently permissive — which is why finding zero
pairs is a failure.

Every tracked `CMakeLists.txt` / `*.cmake` OUTSIDE `cmake/` and `tests/` is
then scanned for the keyword as a standalone argument token. `cmake/` is
exempt because that is where the retirement and its keyword-forwarding
wrapper legitimately live (`nros_components_register_node` must keep parsing
and forwarding `ENTITIES`, or the bare-keyword spelling silently joins
SOURCES); `tests/` is exempt because the gate harnesses set `_NRC_ENTITIES`
to exercise the refusal.
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# `message(FATAL_ERROR "nano_ros_node_register(${_NRC_NAME}): ENTITIES was
#  retired (phase-412).\n"` — and the `HOST was removed (phase-326 ...)` shape
# in `nano_ros_entry`. Both say `<fn>(<anything>): <KEYWORD> was <verb>`.
RETIREMENT = re.compile(
    r'"(?P<fn>[A-Za-z_][A-Za-z0-9_]*)\([^"]*\):\s*'
    r'(?P<kw>[A-Z][A-Z0-9_]*)\s+was\s+(?:retired|removed)\b'
)

# Directories whose cmake IS the API (the retirement guard and the wrappers
# that forward the keyword into it), plus the gate harnesses that exercise it.
EXEMPT_PREFIXES = ("cmake/", "tests/")


def retired_keywords(root):
    """{keyword: (function, file)} discovered from the cmake API sources."""
    found = {}
    api = os.path.join(root, "cmake")
    # walk-ok: `cmake/` is a single flat directory of the API modules, and the
    # self-test's synthetic tree is not a git repository, so there is no index
    # to query.
    for dirpath, _dirnames, filenames in os.walk(api):
        for fn in sorted(filenames):
            if not fn.endswith(".cmake"):
                continue
            rel = os.path.relpath(os.path.join(dirpath, fn), root)
            try:
                with open(os.path.join(dirpath, fn), encoding="utf8", errors="replace") as fh:
                    text = fh.read()
            except OSError:
                continue
            for m in RETIREMENT.finditer(text):
                found.setdefault(m.group("kw"), (m.group("fn"), rel))
    return found


def caller_files(root):
    """Tracked CMakeLists.txt / *.cmake outside the API and the harnesses."""
    out = subprocess.run(
        ["git", "-C", root, "ls-files", "*CMakeLists.txt", "*.cmake"],
        capture_output=True, text=True,
    )
    if out.returncode == 0:
        paths = out.stdout.split()
    else:
        paths = []
        # walk-ok: the self-test builds a synthetic tree with no git index.
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if d not in (".git", "build", "target")]
            for fn in filenames:
                if fn == "CMakeLists.txt" or fn.endswith(".cmake"):
                    paths.append(os.path.relpath(os.path.join(dirpath, fn), root))
    return [
        p for p in paths
        if not p.startswith(EXEMPT_PREFIXES) and "/third-party/" not in p
        and not p.startswith("third-party/")
    ]


def check(root):
    """Return a list of problem strings (empty == pass)."""
    problems = []
    retired = retired_keywords(root)
    if not retired:
        # Never pass because the pattern stopped matching: that is a blind gate
        # reporting success, and the thing it guards configures on no PR lane.
        return [
            "no retirement guards found under cmake/ — this gate looks for\n"
            '  message(FATAL_ERROR "<fn>(...): <KEYWORD> was retired|removed ...")\n'
            "  and found none. Either the wording moved (fix the pattern) or the\n"
            "  guards were deleted. Do not delete this check."
        ]

    # A standalone argument token: `\n    ENTITIES\n` or `ENTITIES value)`.
    matchers = {
        kw: re.compile(rf"(?<![A-Za-z0-9_]){re.escape(kw)}(?![A-Za-z0-9_])")
        for kw in retired
    }
    for rel in caller_files(root):
        try:
            with open(os.path.join(root, rel), encoding="utf8", errors="replace") as fh:
                lines = fh.read().split("\n")
        except OSError:
            continue
        for i, line in enumerate(lines, 1):
            # Comments legitimately explain that a keyword is retired.
            code = line.split("#", 1)[0]
            if not code.strip():
                continue
            for kw, rx in matchers.items():
                if rx.search(code):
                    fn, where = retired[kw]
                    problems.append(
                        f"{rel}:{i} passes {kw}, which {fn}() RETIRED "
                        f"({where}). Configuring this file is a FATAL_ERROR — and "
                        f"nothing on the pull_request lane configures it, so the "
                        f"error reports to nobody. Delete the argument and follow "
                        f"the replacement the guard names."
                    )
    return problems


def _write(root, guard, caller):
    for rel, text in (("cmake/NanoRosNodeRegister.cmake", guard),
                      ("examples/leaf/CMakeLists.txt", caller)):
        os.makedirs(os.path.join(root, os.path.dirname(rel)), exist_ok=True)
        with open(os.path.join(root, rel), "w", encoding="utf8") as fh:
            fh.write(text)


GUARD = (
    'message(FATAL_ERROR\n'
    '    "nano_ros_node_register(${_NRC_NAME}): ENTITIES was retired (phase-412).\\n"\n'
    '    "  state it in the contract sidecar instead")\n'
)
GUARD_REMOVED = (
    'message(FATAL_ERROR\n'
    '    "nano_ros_entry(${_NRA_NAME}): HOST was removed (phase-326) - use MODEL.")\n'
)


def self_test():
    """Every probe asserts a failure this gate must catch, plus the clean case,
    so a gate that stopped matching anything cannot report success."""
    clean_caller = (
        "nros_components_register_node(listener_lib\n"
        "    EXECUTABLE listener\n"
        "    # phase-412 -- ENTITIES is retired; the contract states it.\n"
        ")\n"
    )
    dirty_caller = (
        "nros_components_register_node(listener_lib\n"
        "    EXECUTABLE listener\n"
        "    ENTITIES\n"
        "             sub:std_msgs/msg/String:/chatter)\n"
    )
    bare_caller = (
        "nros_components_register_node(listener_lib\n"
        "    ENTITIES)\n"
    )
    substring_caller = (
        "set(BUDGET_IDENTITIES 4)\n"
        "set(UCLIENT_SHARED_MEMORY_MAX_ENTITIES 4)\n"
    )
    cases = [
        ((GUARD, clean_caller), 0, "a caller that deleted the retired keyword"),
        ((GUARD, dirty_caller), 1, "a caller still passing ENTITIES with specs"),
        ((GUARD, bare_caller), 1, "a caller passing the BARE keyword"),
        ((GUARD, substring_caller), 0,
         "IDENTITIES / MAX_ENTITIES must not match as substrings"),
        ((GUARD_REMOVED, "nano_ros_entry(app HOST alpha)\n"), 1,
         "the `was removed` spelling is a retirement too"),
        ((GUARD_REMOVED, "nano_ros_entry(app MODEL m.yaml)\n"), 0,
         "the same guard with a compliant caller"),
        (("# no retirement guard at all\n", clean_caller), 1,
         "the guard wording moved and this gate went blind"),
    ]
    failures = 0
    tmp = tempfile.mkdtemp()
    try:
        for args, want, label in cases:
            root = os.path.join(tmp, "t")
            shutil.rmtree(root, ignore_errors=True)
            _write(root, *args)
            got = 1 if check(root) else 0
            if got != want:
                sys.stderr.write(f"  self-test FAIL: {label} — got {got}, want {want}\n")
                failures += 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        sys.stderr.write(f"check-retired-cmake-keywords: {failures} self-test failure(s)\n")
        return 1
    print(f"check-retired-cmake-keywords: self-test OK ({len(cases)} cases)")
    return 0


def main():
    # On the NORMAL path, not behind a flag: a negative control nobody runs
    # decays into a comment (`check-gate-selftests`).
    if self_test():
        return 1
    if "--self-test" in sys.argv:
        return 0
    problems = check(ROOT)
    if problems:
        sys.stderr.write("check-retired-cmake-keywords FAILED:\n")
        for p in problems:
            sys.stderr.write(f"  {p}\n")
        return 1
    retired = retired_keywords(ROOT)
    names = ", ".join(f"{k} ({v[0]})" for k, v in sorted(retired.items()))
    print(f"check-retired-cmake-keywords OK: no caller passes {names}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
