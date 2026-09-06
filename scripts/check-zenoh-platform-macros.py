#!/usr/bin/env python3
"""A port may declare its own platform to zenoh-pico, never somebody else's.

Issue 1039. `nros-platform-nuttx` defined `ZENOH_LINUX` alongside `ZENOH_NUTTX`
so that zenoh-pico's `getrandom()` arm would compile -- NuttX has the function,
and the NuttX arm it shadows opens `/dev/urandom`, which a NuttX configuration
is not obliged to provide. The arm was the right one. The NAME was not.

Reaching a capability by asserting a platform is not a local shortcut. The
macro is tested by every other guard in the library, so the image claims to be
Linux everywhere at once, and the claim then becomes load-bearing in places
nobody chose: three guards in `system/common/platform.c` (`<arpa/inet.h>` and
two endpoint helpers) were being reached through it, which is why the define
could not simply be dropped. A capability macro -- `ZENOH_HAS_*` -- says what
the port has and is inert everywhere else.

TWO PRODUCERS, because one was not enough
-----------------------------------------
The NuttX define set is authored twice: in `nros-platform.toml` and again in
`nros-zpico-build`'s `runner.rs` as `build.define(...)`. A check that read only
the manifests would pass while the build still defined the macro. Both are
scanned.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# zenoh-pico's platform macros. `ZENOH_GENERIC` is a build MODE (use the
# generated config header), not a platform, so it is not one of these.
PLATFORM_MACROS = {
    "ZENOH_LINUX",
    "ZENOH_MACOS",
    "ZENOH_BSD",
    "ZENOH_WINDOWS",
    "ZENOH_NUTTX",
    "ZENOH_ZEPHYR",
    "ZENOH_THREADX",
    "ZENOH_FREERTOS_LWIP",
    "ZENOH_ORIN_SPE",
    "ZENOH_ESPIDF",
    "ZENOH_MBED",
    "ZENOH_ARDUINO",
    "ZENOH_RPI_PICO",
    "ZENOH_EMSCRIPTEN",
    "ZENOH_FLIPPER",
}

# Which platform macro each of our ports is entitled to claim. `posix` is the
# host build and is Linux here (see CLAUDE.md "Naming" -- the board is
# `["native", "linux"]`).
OWN_MACRO = {
    "posix": {"ZENOH_LINUX", "ZENOH_MACOS", "ZENOH_BSD"},
    "nuttx": {"ZENOH_NUTTX"},
    "zephyr": {"ZENOH_ZEPHYR"},
    "threadx": {"ZENOH_THREADX"},
    # issue 1143 -- the freertos port claims BOTH FreeRTOS arms. Which one a
    # given image gets is a BOARD fact (`capabilities.ip_stack`), selected by
    # `[build.zenoh] defines_conditional`, so the port is entitled to name
    # either and `check-capability-conditionals` is what checks it names the
    # right one for the right value.
    "freertos": {"ZENOH_FREERTOS_LWIP", "ZENOH_ORIN_SPE"},
}


def macros_in(text: str) -> set[str]:
    """Every ZENOH_* platform macro this text DECLARES, comments excluded.

    Matches a TOML `defines` entry and a Rust `build.define("...")` alike; both
    quote the macro, and a comment mentioning one must not count -- the
    `runner.rs` block explaining this very issue names `ZENOH_LINUX` four
    times."""
    found: set[str] = set()
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("#") or s.startswith("//"):
            continue
        for m in re.finditer(r'"(ZENOH_[A-Z0-9_]+)"', line):
            if m.group(1) in PLATFORM_MACROS:
                found.add(m.group(1))
    return found


def platform_of(path: Path) -> str | None:
    """The port a file belongs to, or None if it is not port-specific."""
    for part in path.parts:
        if part.startswith("nros-platform-"):
            return part[len("nros-platform-") :]
    return None


def main() -> int:
    failures: list[str] = []
    checked = 0

    manifests = sorted(ROOT.glob("packages/platform/*/nros-platform.toml"))
    if not manifests:
        print(
            "ERROR: no packages/platform/*/nros-platform.toml found — this "
            "check would pass by having nothing to inspect.",
            file=sys.stderr,
        )
        return 2

    for man in manifests:
        plat = platform_of(man)
        if plat is None or plat not in OWN_MACRO:
            continue
        checked += 1
        for macro in sorted(macros_in(man.read_text(encoding="utf8"))):
            if macro not in OWN_MACRO[plat]:
                failures.append(
                    f"{man.relative_to(ROOT)}: the {plat} port declares {macro}"
                )

    # Second producer: the zpico build script defines the same set in Rust.
    runner = ROOT / "packages/rmw/zenoh/nros-zpico-build/src/runner.rs"
    if not runner.is_file():
        print(
            f"ERROR: {runner.relative_to(ROOT)} is missing — it is the second\n"
            "       place the define set is authored, so this check cannot\n"
            "       vouch for the build without it.",
            file=sys.stderr,
        )
        return 2

    # Per-port blocks are `} else if use_<plat> {`; attribute each define to the
    # block it sits in.
    #
    # The attribution MUST clear at the closing `} else {`, or it latches the
    # last port seen and blames it for everything after. That is not a
    # hypothetical: the first version of this loop reported the POSIX fallback
    # (`} else {` — which defines ZENOH_LINUX only when the target triple says
    # linux, and is correct) as "the threadx branch defines ZENOH_LINUX". A
    # confident wrong attribution is worse than no check.
    plat_now: str | None = None
    for lineno, line in enumerate(runner.read_text(encoding="utf8").splitlines(), 1):
        m = re.search(r"\buse_([a-z0-9_]+)\s*\{", line)
        if m:
            plat_now = m.group(1)
        elif re.match(r"\s*\}\s*else\s*\{", line) or line.startswith("fn "):
            plat_now = None
        if plat_now not in OWN_MACRO:
            continue
        for macro in sorted(macros_in(line)):
            if macro not in OWN_MACRO[plat_now]:
                failures.append(
                    f"{runner.relative_to(ROOT)}:{lineno}: the {plat_now} "
                    f"branch defines {macro}"
                )
    checked += 1

    if failures:
        print(
            "check-zenoh-platform-macros: a port claims a platform that is not "
            "its own (issue 1039):\n",
            file=sys.stderr,
        )
        for f in failures:
            print(f"  {f}", file=sys.stderr)
        print(
            "\n  zenoh-pico tests these macros everywhere, not only where you\n"
            "  wanted the behaviour, so the claim becomes load-bearing in\n"
            "  guards nobody chose. If the port needs a CAPABILITY, name it:\n"
            "  `ZENOH_HAS_GETRANDOM` is inert outside the arms that test it.\n"
            "  If zenoh-pico has no capability macro for what you need, add\n"
            "  one there rather than borrowing a platform here.",
            file=sys.stderr,
        )
        return 1

    print(
        f"check-zenoh-platform-macros: OK ({checked} producer(s); every port "
        "declares only its own platform)"
    )
    return 0


def self_test(quiet: bool = True) -> int:
    """Negative control. The tree is expected to be clean, and a check that
    only ever sees clean input prints the same thing whether or not it works."""
    cases = [
        # `macros_in` reports every platform macro DECLARED; whether the port
        # is entitled to it is judged against OWN_MACRO afterwards. So the
        # port's own macro is expected here, not filtered out.
        (
            "a port declaring its own macro",
            'defines = ["ZENOH_GENERIC", "ZENOH_NUTTX"]',
            {"ZENOH_NUTTX"},
        ),
        (
            "a port borrowing another platform",
            'defines = ["ZENOH_GENERIC", "ZENOH_NUTTX", "ZENOH_LINUX"]',
            {"ZENOH_LINUX", "ZENOH_NUTTX"},
        ),
        (
            "a capability macro is not a platform",
            'defines = ["ZENOH_NUTTX", "ZENOH_HAS_GETRANDOM"]',
            {"ZENOH_NUTTX"},
        ),
        (
            "a TOML comment naming a macro is not a declaration",
            '# ZENOH_LINUX used to be here\ndefines = ["ZENOH_NUTTX"]',
            {"ZENOH_NUTTX"},
        ),
        (
            "a Rust comment naming a macro is not a declaration",
            '// this used to be "ZENOH_LINUX", see issue 1039\n'
            'build.define("ZENOH_HAS_GETRANDOM", None);',
            set(),
        ),
        (
            "a Rust define is a declaration",
            'build.define("ZENOH_LINUX", None);',
            {"ZENOH_LINUX"},
        ),
    ]
    bad = 0
    for name, text, want in cases:
        got = macros_in(text)
        if got != want:
            bad += 1
            print(
                f"SELFTEST FAIL: {name}: expected {sorted(want)}, got {sorted(got)}",
                file=sys.stderr,
            )
        elif not quiet:
            print(f"  ok: {name}")
    if bad:
        print(
            "check-zenoh-platform-macros: SELFTEST FAILED — the check cannot "
            "be trusted about the tree until it is right about these.",
            file=sys.stderr,
        )
    elif not quiet:
        print(f"check-zenoh-platform-macros selftest: OK ({len(cases)} case(s))")
    return 1 if bad else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv or "--selftest" in sys.argv:
        sys.exit(self_test(quiet=False))
    # The negative control runs BEFORE the tree is inspected.
    if self_test() != 0:
        sys.exit(2)
    sys.exit(main())
