#!/usr/bin/env python3
"""Issue 1015 — a number that sizes a FIXED C ARRAY has a floor of one, and
every producer of that number enforces it.

WHAT WENT WRONG. phase-412 W1 derives `MAX_QUERYABLES = service servers +
3 * action servers`. The reference island declares neither, so it derived
exactly 0, and in `zpico.c`:

    queryable_entry_t queryables[ZPICO_MAX_QUERYABLES];

is then a zero-length array that is NOT the last member of its struct — a GNU
extension that compiles silently and changes what the struct IS. Measured: the
board transmitted 0 bytes in 15 s. No panic, no log line, no fault, the core in
WFI. The same image at 4 transmitted 110 bytes.

WHY EVERY GATE WAS GREEN. `check-knob-delivery` confirms the value ARRIVED — it
did; 0 was derived correctly and delivered faithfully. `check-knob-fixpoint`
converges, because 0 is stable. The image links. Every check in the tree asks
whether the number the image was BUILT with is the number that was DERIVED.
None asked whether the number is USABLE. That is the gap this closes.

WHAT IT CHECKS. Three rules, all keyed off the ARRAY DECLARATIONS in `zpico.c`
rather than off a hand-written list, because a hand-written list is what the
first fix was: it guarded three extents and left seven loaded, two of them
(`ZPICO_MAX_LIVELINESS`, `ZPICO_MAX_PENDING_GETS`) Kconfig-settable and sizing
arrays in the very same struct.

  R1  every macro used as a fixed array extent in `zpico.c` has a
      `#if <M> < 1` / `#error` guard there. This is the backstop that binds a
      producer written tomorrow.
  R2  every extent the ZEPHYR cmake lane supplies as a `-D` is passed to
      `nros_assert_c_array_extent()` before it is. That lane is the earliest
      point a Zephyr build can tell, and it binds all four of its upstreams at
      once (environment, `.conf`, derivation, literal fallback).
  R3  every extent the CARGO lane supplies through `ShimConfig::defines()` is
      named in `C_ARRAY_EXTENT_DEFINES`, and nothing else is. That lane is
      reached by builds cmake never sees, and the reverse direction matters
      too: an entry naming a knob that is NOT an extent asserts a floor where
      the hazard does not exist, which is how a check stops meaning anything.

A producer floors what it SUPPLIES — R2 and R3 deliberately do not demand the
full extent set. `ZPICO_MAX_PENDING_REPLIES` and `ZPICO_ZID_SIZE` have no
external producer at all, and requiring cmake to floor a macro it never emits
would be a rule about nothing.

Python rather than shell, for the reason `check-infra-queryable-counts.py`
gives: `check-gate-selftests`'s call detector requires parentheses, which a
bash function call never has.
"""

import os
import re
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ZPICO_C = "packages/rmw/zenoh/zpico-sys/c/zpico/zpico.c"
ZENOH_CMAKE = "zephyr/cmake/nros_rmw_zenoh.cmake"
ZPICO_BUILD = "packages/rmw/zenoh/nros-zpico-build/src/lib.rs"

# A declaration whose extent is a macro: `queryable_entry_t queryables[M];`,
# including the two-dimensional `stored_queries[M][N];`.
DECL_EXTENT = re.compile(
    r"^[ \t]*[A-Za-z_]\w*(?:[ \t]+\w+)*[ \t*]+\w+((?:\[[A-Z_][A-Z0-9_]*\])+)[ \t]*;",
    re.M,
)
# The broad net beside it: any `[ZPICO_*]` subscript at all. The project's
# macros are prefixed, and nothing here subscripts with a macro, so this costs
# no false positives and catches a declaration written in a shape the precise
# pattern does not know. Two patterns because a gate whose coverage is narrower
# than the rule it enforces is the audit finding this whole class keeps
# producing.
ANY_EXTENT = re.compile(r"\[\s*(ZPICO_[A-Z0-9_]+)\s*\]")
MACRO_IN_BRACKETS = re.compile(r"\[([A-Z_][A-Z0-9_]*)\]")


def read(root, rel):
    with open(os.path.join(root, rel), encoding="utf-8") as fh:
        return fh.read()


def extents(zpico_c_text):
    """Every macro used as a fixed C array extent in `zpico.c`."""
    found = set()
    for m in DECL_EXTENT.finditer(zpico_c_text):
        found.update(MACRO_IN_BRACKETS.findall(m.group(1)))
    found.update(ANY_EXTENT.findall(zpico_c_text))
    return found


def guarded_in_c(zpico_c_text):
    """Macros carrying a `#if <M> < 1` whose body raises `#error`.

    The `#error` is required, not assumed: `#if M < 1` with an empty body is a
    no-op that reads exactly like a guard.
    """
    ok = set()
    for m in re.finditer(
        r"#if\s+([A-Z_][A-Z0-9_]*)\s*<\s*1\s*\n(.*?)#endif",
        zpico_c_text,
        re.S,
    ):
        if "#error" in m.group(2):
            ok.add(m.group(1))
    return ok


def cmake_supplied(cmake_text):
    """Extent-shaped macros this lane writes as `-D`, and what it writes.

    Returns `{macro: (value expression, offset)}`. The VALUE half is what makes
    R2b possible, and R2b is the rule that matters: phase-412 W1's most
    expensive defect was a name built up by a loop
    (`NROS_DERIVED_${_pool}` -> `NROS_DERIVED_NROS_MAX_SUBSCRIBERS`) that named
    nothing, resolved EMPTY, and reached the compiler. An assertion that guards
    a different variable from the one that is emitted is exactly as green and
    exactly as useless.
    """
    supplied = {}
    for block in re.finditer(
        r"zephyr_compile_definitions\s*\((.*?)\)", cmake_text, re.S
    ):
        for m in re.finditer(
            r"\b(ZPICO_[A-Z0-9_]+)\s*=\s*(\S+)", block.group(1)
        ):
            supplied[m.group(1)] = (m.group(2), block.start() + m.start())
    return supplied


def cmake_asserted(cmake_text):
    """Macros passed to `nros_assert_c_array_extent(...)`, and how.

    Returns `{macro: (value expression, offset)}`.
    """
    asserted = {}
    for m in re.finditer(
        r"nros_assert_c_array_extent\s*\((.*?)\n?\s*\)", cmake_text, re.S
    ):
        args = m.group(1)
        macro = re.search(r"\bMACRO\s+([A-Z_][A-Z0-9_]*)", args)
        value = re.search(r'\bVALUE\s+"([^"]*)"', args)
        if macro:
            asserted[macro.group(1)] = (
                value.group(1) if value else None,
                m.start(),
            )
    return asserted


def rust_supplied(lib_rs_text):
    """Macros `ShimConfig::defines()` emits."""
    body = re.search(
        r"fn defines\(&self\)[^\n]*\{(.*?)\n    \}", lib_rs_text, re.S
    )
    if not body:
        return None
    return set(re.findall(r'\(\s*"(ZPICO_[A-Z0-9_]+)"', body.group(1)))


def rust_listed(lib_rs_text):
    """Entries of `C_ARRAY_EXTENT_DEFINES`."""
    block = re.search(
        r"C_ARRAY_EXTENT_DEFINES\s*:\s*&\[&str\]\s*=\s*&\[(.*?)\];",
        lib_rs_text,
        re.S,
    )
    if not block:
        return None
    return set(re.findall(r'"(ZPICO_[A-Z0-9_]+)"', block.group(1)))


def check(root):
    problems = []
    c_text = read(root, ZPICO_C)
    ext = extents(c_text)
    if not ext:
        # A pattern that matches nothing passes every rule. The file has held
        # array pools since phase-328; zero of them means the scanner broke,
        # not that the hazard went away.
        return [
            f"{ZPICO_C}: no fixed array extents found at all — the scanner is "
            f"broken, not the source"
        ]

    # R1 — the backstop.
    guarded = guarded_in_c(c_text)
    for macro in sorted(ext - guarded):
        problems.append(
            f"{ZPICO_C}: {macro} sizes a fixed C array with no floor. Add\n"
            f'      #if {macro} < 1\n'
            f'      #error "{macro} must be >= 1: it sizes a fixed C array (issue 1015)"\n'
            f"      #endif\n"
            f"    beside the others. Zero is not a smaller pool — it is a "
            f"zero-length array, a GNU extension that compiles silently and "
            f"changes the struct's layout."
        )

    # R2 — the Zephyr configure lane floors what it supplies, guarding the SAME
    # expression it emits, BEFORE it emits it.
    cmake_text = read(root, ZENOH_CMAKE)
    asserted = cmake_asserted(cmake_text)
    supplied_cmake = cmake_supplied(cmake_text)
    for macro in sorted(set(supplied_cmake) & ext):
        emitted, emit_at = supplied_cmake[macro]
        if macro not in asserted:
            problems.append(
                f"{ZENOH_CMAKE}: {macro} is written as a -D and sizes a fixed C "
                f"array, but nothing calls nros_assert_c_array_extent(MACRO "
                f"{macro} ...) first. The C #error would catch it at COMPILE; "
                f"this lane can catch it at CONFIGURE, where the person who set "
                f"the knob is standing."
            )
            continue
        guarded, guard_at = asserted[macro]
        # R2b — the same expression. A guard on a different variable is exactly
        # as green as a real one and exactly as useless.
        if guarded is not None and guarded != emitted:
            problems.append(
                f"{ZENOH_CMAKE}: {macro} is emitted as `{emitted}` but the "
                f"assertion guards `{guarded}`. Two different expressions, so "
                f"the value that reaches the compiler is not the value that was "
                f"checked — the shape of phase-412 W1's built-up name that "
                f"resolved EMPTY while every gate stayed green."
            )
        # R2c — before, not after. A check downstream of the thing it protects
        # protects nothing.
        if guard_at > emit_at:
            problems.append(
                f"{ZENOH_CMAKE}: {macro} is asserted AFTER it is written as a "
                f"-D. The assertion must run first, or the bad value has "
                f"already reached the compile line."
            )

    # R3 — the cargo lane floors what it supplies, and claims nothing else.
    lib_text = read(root, ZPICO_BUILD)
    supplied = rust_supplied(lib_text)
    listed = rust_listed(lib_text)
    if supplied is None:
        problems.append(f"{ZPICO_BUILD}: could not read ShimConfig::defines()")
    if listed is None:
        problems.append(f"{ZPICO_BUILD}: could not read C_ARRAY_EXTENT_DEFINES")
    if supplied is not None and listed is not None:
        for macro in sorted((supplied & ext) - listed):
            problems.append(
                f"{ZPICO_BUILD}: {macro} is emitted by ShimConfig::defines() "
                f"and sizes a fixed C array, but is missing from "
                f"C_ARRAY_EXTENT_DEFINES, so check_c_array_extents() never "
                f"floors it. A cargo-only build (a fixtures.toml env row, a "
                f"shell export) has no cmake to catch it."
            )
        for macro in sorted(listed - ext):
            problems.append(
                f"{ZPICO_BUILD}: C_ARRAY_EXTENT_DEFINES names {macro}, which is "
                f"not an array extent in {ZPICO_C}. Either the array went away "
                f"and the entry should follow it, or the entry asserts a floor "
                f"where the hazard does not exist."
            )
    return problems


def _write(root, c_text, cmake_text, rs_text):
    for rel, text in (
        (ZPICO_C, c_text),
        (ZENOH_CMAKE, cmake_text),
        (ZPICO_BUILD, rs_text),
    ):
        path = os.path.join(root, rel)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(text)


GOOD_C = """\
struct zpico_session {
    queryable_entry_t queryables[ZPICO_MAX_QUERYABLES];
    liveliness_entry_t liveliness[ZPICO_MAX_LIVELINESS];
};
#if ZPICO_MAX_QUERYABLES < 1
#error "ZPICO_MAX_QUERYABLES must be >= 1: it sizes a fixed C array (issue 1015)"
#endif
#if ZPICO_MAX_LIVELINESS < 1
#error "ZPICO_MAX_LIVELINESS must be >= 1: it sizes a fixed C array (issue 1015)"
#endif
"""

GOOD_CMAKE = """\
nros_assert_c_array_extent(MACRO ZPICO_MAX_QUERYABLES VALUE "${X}")
nros_assert_c_array_extent(MACRO ZPICO_MAX_LIVELINESS VALUE "${Y}")
zephyr_compile_definitions(
    ZPICO_MAX_QUERYABLES=${X}
    ZPICO_MAX_LIVELINESS=${Y}
    ZPICO_GET_POLL_INTERVAL_MS=${Z}
)
"""

# R2c's control: the same file with the assertions moved BELOW the emission.
LATE_CMAKE = """\
zephyr_compile_definitions(
    ZPICO_MAX_QUERYABLES=${X}
    ZPICO_MAX_LIVELINESS=${Y}
)
nros_assert_c_array_extent(MACRO ZPICO_MAX_QUERYABLES VALUE "${X}")
nros_assert_c_array_extent(MACRO ZPICO_MAX_LIVELINESS VALUE "${Y}")
"""

GOOD_RS = """\
impl ShimConfig {
    pub fn defines(&self) -> Vec<(&'static str, String)> {
        let mut out = vec![
            ("ZPICO_MAX_QUERYABLES", self.max_queryables.to_string()),
            ("ZPICO_MAX_LIVELINESS", self.max_liveliness.to_string()),
            ("ZPICO_TX_BATCH", "1".to_string()),
        ];
        out
    }
}

pub const C_ARRAY_EXTENT_DEFINES: &[&str] = &[
    "ZPICO_MAX_QUERYABLES",
    "ZPICO_MAX_LIVELINESS",
];
"""


def self_test():
    cases = [
        ((GOOD_C, GOOD_CMAKE, GOOD_RS), 0, "a fully floored tree passes"),
        (
            (GOOD_C.replace('#if ZPICO_MAX_LIVELINESS < 1', '#if 0'),
             GOOD_CMAKE, GOOD_RS),
            1,
            "R1: an extent with no #error guard is caught",
        ),
        (
            # The guard is there but its body does nothing — a no-op that reads
            # exactly like a guard, which is the shape worth catching.
            (GOOD_C.replace(
                '#error "ZPICO_MAX_LIVELINESS must be >= 1: it sizes a fixed C array (issue 1015)"',
                "/* nothing */"),
             GOOD_CMAKE, GOOD_RS),
            1,
            "R1: an empty guard body is not a guard",
        ),
        (
            (GOOD_C,
             GOOD_CMAKE.replace(
                 'nros_assert_c_array_extent(MACRO ZPICO_MAX_LIVELINESS VALUE "${Y}")\n', ""),
             GOOD_RS),
            1,
            "R2: a -D'd extent with no configure assertion is caught",
        ),
        (
            # R2b: the guard names a DIFFERENT variable from the one emitted.
            # Green, and useless — phase-412 W1's built-up name exactly.
            (GOOD_C,
             GOOD_CMAKE.replace(
                 'nros_assert_c_array_extent(MACRO ZPICO_MAX_LIVELINESS VALUE "${Y}")',
                 'nros_assert_c_array_extent(MACRO ZPICO_MAX_LIVELINESS VALUE "${NROS_DERIVED_Y}")'),
             GOOD_RS),
            1,
            "R2b: a guard on a different expression is caught",
        ),
        (
            (GOOD_C, LATE_CMAKE, GOOD_RS),
            1,
            "R2c: an assertion placed after the -D is caught",
        ),
        (
            (GOOD_C, GOOD_CMAKE,
             GOOD_RS.replace('    "ZPICO_MAX_LIVELINESS",\n', "")),
            1,
            "R3: a supplied extent missing from the list is caught",
        ),
        (
            (GOOD_C, GOOD_CMAKE,
             GOOD_RS.replace('    "ZPICO_MAX_LIVELINESS",',
                             '    "ZPICO_MAX_LIVELINESS",\n    "ZPICO_TX_BATCH",')),
            1,
            "R3: a listed knob that is NOT an extent is caught",
        ),
        (
            # The scanner's own negative control. A `zpico.c` with no array
            # declarations passes R1 vacuously, and a check that cannot fail is
            # the defect this whole campaign keeps producing.
            ("int x = 1;\n", GOOD_CMAKE, GOOD_RS),
            1,
            "a source with no extents at all is reported, not passed",
        ),
    ]
    tmp = tempfile.mkdtemp(prefix="c-array-extent-selftest-")
    failures = 0
    try:
        for args, want, label in cases:
            root = os.path.join(tmp, "t")
            shutil.rmtree(root, ignore_errors=True)
            _write(root, *args)
            got = 1 if check(root) else 0
            if got != want:
                sys.stderr.write(
                    f"  self-test FAIL: {label} — got {got}, want {want}\n"
                )
                failures += 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        sys.stderr.write(
            f"check-c-array-extent-floors self-test: FAILED ({failures})\n"
        )
        sys.exit(1)
    print("check-c-array-extent-floors self-test: OK")


def main():
    # On the NORMAL path, not behind a flag: a negative control nobody runs
    # decays into a comment (`check-gate-selftests`).
    self_test()
    if "--self-test" in sys.argv:
        return
    problems = check(ROOT)
    if problems:
        sys.stderr.write(
            "check-c-array-extent-floors: %d problem(s) — issue 1015:\n"
            % len(problems)
        )
        for p in problems:
            sys.stderr.write(f"  - {p}\n")
        sys.exit(1)
    c_text = read(ROOT, ZPICO_C)
    ext = sorted(extents(c_text))
    print(f"  ok    {len(ext)} fixed C array extent(s) in {ZPICO_C}, all floored at 1")
    for macro in ext:
        print(f"          {macro}")
    print("c-array-extent-floors: no pool can be sized zero.")


if __name__ == "__main__":
    main()
