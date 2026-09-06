#!/usr/bin/env python3
"""Every guard on a knob-sized C array must FIRE at 0, proven by compiling it.

The build-tier half of `check-c-array-pool-floors`. That gate is on the fast
line, must run on a bare worktree, and therefore reasons about the SOURCE: it
checks that a guard sits after the `#ifndef` fallback that defines its knob.
That catches the two shapes that shipped, and nothing else.

This one asks the compiler. It is the only way to answer "can this guard fire"
for a guard that is unreachable for a reason no static rule enumerates -- nested
inside an unrelated `#if`, disabled by a config header, or in a file the
translation unit never reaches.

WHY IT IS A SEPARATE GATE AND NOT A BRANCH OF THE OTHER ONE

It needs the zenoh-pico submodule, which the push lane does not check out. A
gate that silently weakens depending on which lane ran it reports the same OK
either way, and the reader cannot tell which answer they got -- the shape this
campaign has removed four times. Two gates, two lanes, two honest answers.

WHAT IT DOES

For every macro `check-c-array-pool-floors` classifies as GUARDED:

  * compile the file at DEFAULTS and require NO guard to fire. This is the
    half that catches `#607`: a guard placed before its `#define` reads the
    macro as undefined, which the preprocessor evaluates as 0, so it fired on
    every build.
  * compile with `-D<KNOB>=0` and require THAT guard to fire. This is the half
    that catches the first attempted fix: a guard inside the `#ifndef` is
    skipped by the very `-D` it is meant to catch.

Failures that are not a guard diagnostic are ignored on purpose. This asks one
question about the preprocessor; whether the whole TU type-checks is
`check-c`'s job, and coupling the two would make this red for reasons that say
nothing about guards.

Run:  python3 scripts/check-c-array-guard-probe.py [--self-test]
"""

import importlib.util
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# The include set a zpico translation unit needs to reach its guards. Kept here
# rather than derived from the build: this compiles ONE file to ask ONE
# preprocessor question, and reproducing the build's flag assembly would couple
# a source gate to the build system it is checking. If a build change breaks
# these, the probe reports that it could not compile rather than passing.
INCLUDES = [
    "packages/rmw/zenoh/zpico-sys/c/include",
    "packages/rmw/zenoh/zpico-sys/zenoh-pico/include",
    "packages/platform/nros-platform-api/include",
]
DEFINES = ["-DZENOH_LINUX=1", "-DZ_FEATURE_UNSTABLE_API=1"]
GUARD_TEXT = "must be >= 1"

# The submodule whose absence means this gate cannot run. Named as a FILE, not a
# directory: an initialised-but-empty submodule dir is the state that makes a
# "does the directory exist" probe lie.
SUBMODULE_PROBE = "packages/rmw/zenoh/zpico-sys/zenoh-pico/include/zenoh-pico/config.h"


def guarded_knobs():
    """{macro: path} for everything the source-level gate calls GUARDED."""
    path = os.path.join(ROOT, "scripts", "check-c-array-pool-floors.py")
    spec = importlib.util.spec_from_file_location("pool_floors", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    found = mod.scan(mod.read_sources())
    return {m: i["path"] for m, i in found.items() if i["guarded"]}


def compile_probe(rel_path, extra):
    """Guard diagnostics from compiling `rel_path`. (text, ran) — never raises."""
    cmd = ["cc", "-fsyntax-only", "-std=c11", *DEFINES, *extra]
    for inc in INCLUDES:
        cmd += ["-I", os.path.join(ROOT, inc)]
    cmd.append(os.path.join(ROOT, rel_path))
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    except (OSError, subprocess.SubprocessError) as exc:
        return f"could not run the compiler: {exc}", False
    return r.stderr, True


def main():
    if "--self-test" in sys.argv:
        return self_test()
    if self_test() != 0:
        return 1

    if not os.path.isfile(os.path.join(ROOT, SUBMODULE_PROBE)):
        sys.path.insert(0, os.path.join(ROOT, "scripts", "build"))
        print(
            "[SKIPPED] c-array-guard-probe: zenoh-pico submodule not checked out "
            "(git submodule update --init packages/rmw/zenoh/zpico-sys/zenoh-pico)"
        )
        return 0

    knobs = guarded_knobs()
    if not knobs:
        sys.stderr.write(
            "check-c-array-guard-probe: the source gate reports NO guarded knob, "
            "so this would compile nothing and pass vacuously.\n"
        )
        return 1

    problems = []
    files = sorted(set(knobs.values()))

    # One compile per file at defaults: nothing may fire.
    for rel in files:
        out, ran = compile_probe(rel, [])
        if not ran:
            sys.stderr.write(f"check-c-array-guard-probe: {out}\n")
            return 1
        fired = [l for l in out.split("\n") if GUARD_TEXT in l]
        if fired:
            problems.append(
                f"{rel} fires {len(fired)} guard at DEFAULT knob values.\n"
                "      A guard that fires with no `-D` at all is not guarding a "
                "zero — it is\n"
                "      reading its macro as UNDEFINED, which the preprocessor "
                "evaluates as 0.\n"
                "      First: " + fired[0].strip()
            )

    # One compile per knob at zero: exactly that knob's guard must fire.
    for macro, rel in sorted(knobs.items()):
        out, ran = compile_probe(rel, [f"-D{macro}=0"])
        if not ran:
            sys.stderr.write(f"check-c-array-guard-probe: {out}\n")
            return 1
        if not any(GUARD_TEXT in l and macro in l for l in out.split("\n")):
            problems.append(
                f"{macro} ({rel}) has a guard that does NOT fire at `-D{macro}=0`.\n"
                "      The `#if` is present but the preprocessor never reaches it "
                "with the\n"
                "      macro set — inside its own `#ifndef` (a `-D` skips the "
                "block), behind\n"
                "      an unrelated `#if`, or in a file this TU does not include."
            )

    if problems:
        sys.stderr.write(
            "check-c-array-guard-probe: %d problem(s)\n\n" % len(problems)
        )
        for p in problems:
            sys.stderr.write(f"  [FAIL] {p}\n\n")
        return 1

    print(
        "check-c-array-guard-probe: OK — %d guard(s) across %d file(s) fire at 0 "
        "and are silent at defaults." % (len(knobs), len(files))
    )
    return 0


def self_test():
    """Prove the probe can fail, on synthetic C rather than on the tree."""
    import tempfile

    good = ("#ifndef K\n#define K 8\n#endif\n"
            "#if K < 1\n#error \"K must be >= 1\"\n#endif\n"
            "int a[K];\n")
    before = ("#if K < 1\n#error \"K must be >= 1\"\n#endif\n"
              "#ifndef K\n#define K 8\n#endif\nint a[K];\n")
    inside = ("#ifndef K\n#define K 8\n"
              "#if K < 1\n#error \"K must be >= 1\"\n#endif\n#endif\n"
              "int a[K];\n")
    bad = 0
    with tempfile.TemporaryDirectory() as d:
        for name, src, want_default, want_zero in [
            ("good", good, 0, 1),
            ("before", before, 1, 1),   # fires with NO -D at all
            ("inside", inside, 0, 0),   # never fires, even at -DK=0
        ]:
            p = os.path.join(d, f"{name}.c")
            open(p, "w").write(src)
            def probe(extra):
                r = subprocess.run(["cc", "-fsyntax-only", "-std=c11", *extra, p],
                                   capture_output=True, text=True)
                return sum(1 for l in r.stderr.split("\n") if "must be >= 1" in l)
            got_d, got_z = min(probe([]), 1), min(probe(["-DK=0"]), 1)
            if (got_d, got_z) != (want_default, want_zero):
                print(f"  self-test FAIL: {name} -> default={got_d} zero={got_z}, "
                      f"want default={want_default} zero={want_zero}")
                bad += 1
    if bad:
        print(f"check-c-array-guard-probe self-test: {bad} case(s) FAILED")
        return 1
    print("check-c-array-guard-probe self-test: OK (3 cases: good, "
          "guard-before-define, guard-inside-ifndef)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
