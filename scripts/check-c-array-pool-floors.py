#!/usr/bin/env python3
"""A knob that SIZES A FIXED C ARRAY must state what its smallest legal value is.

Issue 1015. `MAX_QUERYABLES` is derived as "service servers + actions x 3", the
reference island declares neither, so it derived exactly 0 — and

    queryable_entry_t queryables[ZPICO_MAX_QUERYABLES];

became a zero-length array. Measured on that board, same image, one variable:
at 0 it transmitted **0 bytes in 15 s** (no panic, no log line, core in WFI); at
1 and at 4 it transmitted 110. Every gate stayed green through it:
`check-knob-delivery` confirms the value ARRIVED — it did — and
`check-knob-fixpoint` converges, because 0 is stable. Nothing in the tree asked
whether the number was USABLE.

WHY THIS IS NOT SIMPLY "POOLS ARE POSITIVE"
-------------------------------------------
Because it is measurably false one backend over. Issue 1033 lowered the XRCE
pool minima from 1 to 0 ON PURPOSE, with three measurements behind it: those
arrays are plain members mid-struct rather than flexible array members, `arr[0]`
compiles on gnu99/c11/gnu11, and every walk is `for (i = 0; i < MAX; ++i)`,
which at 0 simply does not run. A zero there is worth 33,296 bytes of heap per
subscriber slot and 4,384 per service-server slot, read from the zephyr cpp
listener's own DWARF — the difference between an image that fits its 65,536-byte
heap and one that does not.

So zero-legality is a property of the STORAGE and of the code around it, and it
has to be DECIDED per knob rather than assumed in either direction. This gate
does not decide it. It requires that somebody has, and that the decision is
still true of the source:

  GUARDED      the file carries `#if <KNOB> < N` + `#error` beside the array,
               for some N >= 1. Usually 1; `STRESS_SIZE` is 16, because
               `build_payload()` writes a 12-byte header with no bounds test,
               so `< 1` there would state a minimum the code does not have.
               `check-c-array-guard-probe` then proves the guard FIRES.
  ZERO_LEGAL   the decision is "0 is a legal size here", with the issue that
               measured it. Such a knob must NOT also carry a guard.
  UNCLASSIFIED the honest state of the rest: an array sized by a build-settable
               macro that nobody has ruled on. Ratcheted — the list may shrink,
               and `UNCLASSIFIED_CEILING` makes growth a deliberate edit.

WHERE THE FLOOR LIVES, AND WHY THE GATE CHECKS THAT TOO
-------------------------------------------------------
The first fix for 1015 floored the three zenoh pools in the SHARED derivation
(`EntityInventory::derive`). That is the wrong layer, and it is wrong in a way
no existing gate could see: `NROS_DERIVED_MAX_SUBSCRIBERS` and
`NROS_DERIVED_MAX_QUERYABLES` also feed `NROS_XRCE_MAX_SUBSCRIBERS` and
`NROS_XRCE_MAX_SERVICE_SERVERS` (`nros_cargo_build.cmake`), so the floor reached
the backend that had just measured zero as its answer — landing the day BEFORE
issue 1033's fix, which then could not deliver a zero on the derive rung at all.
One derivation, two consumers, two different legal minima.

The floor therefore belongs to the consumer that names the knob. This gate holds
that arrangement in place from both ends: each producer of a GUARDED knob must
apply the floor, and the shared derivation must not.

    check-c-array-pool-floors.py            # the gate (runs its selftest first)
    check-c-array-pool-floors.py --audit    # the full classification, never fails
    check-c-array-pool-floors.py --selftest # controls only
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

# --- the C/C++ scan --------------------------------------------------------

EXTS = {".c", ".h", ".cpp", ".hpp"}
# Vendored, generated, and codegen golden trees. None of them is ours to guard,
# and the fingerprint corpora are expected OUTPUT compared byte-for-byte.
SKIP = ("third-party/", "/generated/", "zenoh-pico/", "/tests/fixtures/", "/build/")

# A macro is a BUILD KNOB when the file gives it an `#ifndef` fallback: that is
# the shape a `-D` on the command line overrides, and it is exactly the set a
# producer can hand a number to. A plain `#define` is an internal constant no
# build can move, so it is out of scope by construction rather than by a list.
IFNDEF = re.compile(r"^\s*#\s*ifndef\s+([A-Z][A-Z0-9_]*)\s*$")
DEFINE = re.compile(r"^\s*#\s*define\s+([A-Z][A-Z0-9_]*)\s+\S")
ARRAY = re.compile(r"\[\s*([A-Z][A-Z0-9_]*)\s*\]")
# `#if K < N` for any POSITIVE integer literal, not only `< 1`. The gate's
# question is "what is the smallest legal value", and for several arrays the
# honest answer is above one: `STRESS_SIZE` is 16 because `build_payload()`
# writes a 12-byte header with no bounds test. `< 0` is still not a guard —
# it is the disarm the selftest controls for, and `[1-9][0-9]*` excludes it.
GUARD = re.compile(r"#\s*if\s+([A-Z][A-Z0-9_]*)\s*<\s*[1-9][0-9]*\b")
ERROR = re.compile(r"^\s*#\s*error\b", re.M)


class Failure(Exception):
    """The gate could not run — never a silent pass."""


# --- the decisions ---------------------------------------------------------

# Knobs whose arrays may legally be EMPTY, and what measured it. A knob here
# must NOT carry a `#if ... < 1` guard: the two statements contradict, and the
# guard is the one the compiler obeys.
ZERO_LEGAL = {
    "XRCE_MAX_SUBSCRIBERS": (
        "issue 1033 — a pub-only image reserves nothing; a slot is "
        "XRCE_SUBSCRIBER_RING_DEPTH x XRCE_BUFFER_SIZE = 33,296 bytes of heap, "
        "read from the zephyr cpp listener's DWARF. Every walk is bounded and "
        "the array is a plain mid-struct member, not a flexible one."
    ),
    "XRCE_MAX_SERVICE_SERVERS": (
        "issue 1033 — 4,384 bytes a slot, same measurement, same argument. Fed "
        "by NROS_DERIVED_MAX_QUERYABLES, so a floor in the derivation defeats it."
    ),
    "XRCE_MAX_SERVICE_CLIENTS": (
        "issue 1033 — 1,040 bytes a slot; build.rs lowered the minimum to 0 with "
        "the other two."
    ),
}

# Arrays sized by a build knob that nobody has ruled on. NOT a pass: an honest
# record of the wider question issue 1015 left open ("what other derived value
# has an illegal special case at some boundary"). Classifying one means moving
# it OUT of here — into a guard beside its array, or into ZERO_LEGAL with the
# measurement that settled it.
#
# None of these is fed by a derivation today; each is reachable only by a person
# stating a number, which is why the campaign's silent-zero failure has not
# reproduced through them. That is a reason to rank them below the derived
# knobs, not a reason to call them safe.
#
# Issue 1131 ruled the other eleven (nine guarded here, and two the widened scan
# below turned out to have carried a guard all along). What is LEFT is left on
# purpose: each of these two has a real argument that zero is the right answer,
# and that argument needs a measurement nobody has taken. A guard would foreclose
# the saving the way issue 1015's first fix foreclosed issue 1033's.
UNCLASSIFIED = {
    # 16,384 bytes of stack a slot x 4 slots = 64 KiB of .noinit, present in
    # EVERY Zephyr image whether or not the system declares a tier — nothing
    # produces this knob, so 4 is what every image compiles. A tierless image
    # setting 0 is the same shape as XRCE_MAX_SUBSCRIBERS=0, and the refusal is
    # already loud (`entry_tiers.rs` prints "failed to spawn tier ... pool
    # exhausted?" per tier). NEEDS: a Zephyr build at
    # -DNROS_ZEPHYR_MAX_TIERS=0 proving K_THREAD_STACK_ARRAY_DEFINE accepts a
    # count of 0, plus a `just mem-report` delta for the 64 KiB.
    "NROS_ZEPHYR_MAX_TIERS": "zephyr/nros_platform_zephyr_shims.c",
    # `Slot g_pool[N]`, where a Slot is `alignas(CallbackAdapter) unsigned char
    # storage[sizeof(CallbackAdapter)]` — a px4::WorkItem subclass — x 64. The
    # exhaustion path is a DOCUMENTED FALLBACK ("Caller falls back to polling"),
    # not a failure, which is exactly what makes 0 arguable: an image that wants
    # polling only would reclaim the pool. NEEDS: the PX4 SDK, to size
    # CallbackAdapter, and confirmation from the uORB caller that the polling
    # fallback is a supported mode rather than a degradation nobody reports.
    "NROS_RMW_UORB_PX4_MAX_CALLBACKS": "packages/rmw/uorb/nros-rmw-uorb/src/px4_callback_glue.cpp",
}
# The list may SHRINK. Raising this is a deliberate edit that says "one more
# knob-sized array ships unruled on", beside the entry that says which.
UNCLASSIFIED_CEILING = 2

# --- the producer half -----------------------------------------------------

# Every producer that hands a DERIVED number to a guarded knob, and the floor
# call it must apply. Two lanes, because a Zephyr Rust image and a cargo leaf
# reach the same pools by different paths (issue 0460).
# A producer either FLOORS the knob or ABSTAINS from producing it at all, and
# the abstention has to be written down: `ZPICO_MAX_QUERYABLES` is deliberately
# not derived on the cargo lane (a leaf has no `NROS_DECLARED_INFRA_QUERYABLES`
# channel, so a bare count would be SHORT for any image with param services),
# and a rule with no spelling for that would push it into an unwanted floor.
FLOOR_PRODUCERS = [
    (
        "zephyr/cmake/nros_cargo_build.cmake",
        r"_nros_c_array_pool_floor\([^)]*\b{knob}\b",
        None,
        "the CMake ZPICO bridge",
    ),
    (
        "packages/cli/nros-cli-core/src/leaf_entity_env.rs",
        r'\("{knob}",\s*floor\(',
        r'NOT_DERIVED[A-Z_]*: &str = "{knob}"',
        "the cargo-leaf `[env]` sidecar",
    ),
]
FLOORED_KNOBS = ["ZPICO_MAX_PUBLISHERS", "ZPICO_MAX_QUERYABLES", "ZPICO_MAX_SUBSCRIBERS"]

# The shared derivation, and the slice of it that computes the pools. Nothing in
# that slice may raise a count: it is the image's DEMAND, and two consumers read
# it with different minima.
DERIVATION = "packages/cli/nros-cli-core/src/entity_inventory.rs"
DERIVATION_START = "let n = |tag: &str|"
DERIVATION_END = "let max_nodes ="


def next_code_line(lines: list[str], i: int) -> int | None:
    """Index of the first line after `i` that is neither blank nor a comment.

    The `#ifndef` fallback and its `#define` are NOT always adjacent: the two
    biggest knobs in the tree document themselves between the two lines, and
    an adjacency test therefore could not see `XRCE_SUBSCRIBER_RING_DEPTH`
    (whose Kconfig already says `range 1 1024`) or `ZPICO_GRAPH_CACHE_SIZE`
    (65,536 bytes of .bss). Both sit in files whose SIBLING knobs this gate
    already rules on, which is the issue-0196 shape: a gate whose coverage is
    narrower than the rule it enforces, reporting OK over what it cannot see.
    """
    j = i + 1
    in_block = False
    while j < len(lines):
        t = lines[j].strip()
        if in_block:
            if "*/" in t:
                in_block = False
            j += 1
            continue
        if not t or t.startswith("//"):
            j += 1
            continue
        if t.startswith("/*"):
            if "*/" not in t:
                in_block = True
            j += 1
            continue
        return j
    return None


def scan(files: dict[str, str]) -> dict[str, dict]:
    """macro -> {path, guarded} for every build knob that sizes an array.

    Pure over a {path: text} mapping so the controls below can feed it source
    that does not exist on disk.
    """
    found: dict[str, dict] = {}
    for path, text in sorted(files.items()):
        lines = text.splitlines()
        settable = {}
        for i, line in enumerate(lines):
            m = IFNDEF.match(line)
            if m:
                k = next_code_line(lines, i)
                d = DEFINE.match(lines[k]) if k is not None else None
                if d and d.group(1) == m.group(1):
                    # Remember where the fallback block ENDS. A guard is only
                    # reachable after it: before, the macro is undefined and the
                    # preprocessor reads that as 0, so `#if K < 1` fires on
                    # EVERY build; inside, a `-D` skips the block and takes the
                    # guard with it. Both shipped (see `guard_placement`).
                    depth, end = 0, None
                    for j in range(i, len(lines)):
                        t = lines[j].strip()
                        if t.startswith(("#if", "#ifdef", "#ifndef")):
                            depth += 1
                        elif t.startswith("#endif"):
                            depth -= 1
                            if depth == 0:
                                end = j
                                break
                    settable[m.group(1)] = end
        arrays = set(ARRAY.findall(text))
        guards = {}
        for i, line in enumerate(lines):
            g = GUARD.search(line)
            # A comparison with no `#error` under it is a number nobody checks.
            if g and ERROR.search("\n".join(lines[i + 1 : i + 4])):
                guards.setdefault(g.group(1), i)
        for macro in sorted(set(settable) & arrays):
            at = guards.get(macro)
            fallback_end = settable[macro]
            found[macro] = {
                "path": path,
                "guarded": at is not None,
                # None when there is no guard, or when the fallback block is
                # unterminated (a malformed file, reported elsewhere).
                "reachable": None if at is None or fallback_end is None
                else at > fallback_end,
                "guard_line": None if at is None else at + 1,
                "fallback_end_line": None if fallback_end is None else fallback_end + 1,
            }
    return found


def check_arrays(found: dict[str, dict]) -> list[str]:
    problems = []

    for macro, info in sorted(found.items()):
        if info["guarded"] and info.get("reachable") is False:
            problems.append(
                f"{macro} ({info['path']}:{info['guard_line']}) is guarded where the "
                f"guard CANNOT FIRE.\n"
                f"      Its `#ifndef` fallback closes at line {info['fallback_end_line']}, "
                f"and the guard is at {info['guard_line']}.\n"
                "      Before that line the macro is UNDEFINED, which the preprocessor\n"
                "      evaluates as 0 — so the guard fires on every build; inside the\n"
                "      block, a `-D` skips it and takes the guard with it. Both shipped.\n"
                "      Move the guard AFTER the `#endif` that closes the fallback."
            )
            continue
        if info["guarded"]:
            if macro in ZERO_LEGAL:
                problems.append(
                    f"{macro} ({info['path']}) is in ZERO_LEGAL and ALSO carries a\n"
                    f"    `#if {macro} < 1` guard. The guard is what the compiler\n"
                    f"    obeys, so the table is stating a size the build refuses.\n"
                    f"    Delete one of them — and say which measurement moved."
                )
            if macro in UNCLASSIFIED:
                problems.append(
                    f"{macro} ({info['path']}) is guarded now: remove it from\n"
                    f"    UNCLASSIFIED. The list is a ratchet; a classified knob\n"
                    f"    left in it makes the count read worse than the tree is."
                )
            continue
        if macro in ZERO_LEGAL or macro in UNCLASSIFIED:
            continue
        problems.append(
            f"{macro} ({info['path']}) sizes a fixed C array and states no\n"
            f"    smallest legal value. Either\n"
            f"      * guard it beside the array —\n"
            f"          #if {macro} < 1\n"
            f'          #error "{macro} must be >= 1: it sizes a C array (issue 1015)"\n'
            f"          #endif\n"
            f"        and floor it in every producer that derives it, or\n"
            f"      * add it to ZERO_LEGAL with what MEASURED that zero is a legal\n"
            f"        size for this array (issue 1033 is the worked example).\n"
            f"    Issue 1015: a derived 0 here left a board transmitting nothing\n"
            f"    for 15 s with no diagnostic, and every gate green."
        )

    # Both directions. An authored table that no longer describes the tree is
    # the failure the rmw parity map taught: two green tools, 25 symbols apart.
    for macro, why in sorted(ZERO_LEGAL.items()):
        if macro not in found:
            problems.append(
                f"ZERO_LEGAL names {macro}, which no longer sizes an array from a\n"
                f"    build knob. Reason on file: {why[:60]}...\n"
                f"    Delete the entry — a decision about code that is gone reads\n"
                f"    as coverage of code that is not."
            )
    for macro, where in sorted(UNCLASSIFIED.items()):
        if macro not in found:
            problems.append(
                f"UNCLASSIFIED names {macro} ({where}), which no longer sizes an\n"
                f"    array from a build knob. Delete the entry; the list may only\n"
                f"    shrink, and a stale name is a shrink nobody gets credit for."
            )
    if len(UNCLASSIFIED) > UNCLASSIFIED_CEILING:
        problems.append(
            f"UNCLASSIFIED holds {len(UNCLASSIFIED)} entries, ceiling is "
            f"{UNCLASSIFIED_CEILING}.\n"
            f"    Classify the knob instead of raising the ceiling: the ceiling\n"
            f"    exists so that adding one is a visible decision, not a habit."
        )
    return problems


def check_producers(texts: dict[str, str]) -> list[str]:
    """Every producer floors the guarded knobs, and the derivation does not."""
    problems = []
    for path, pattern, abstain, what in FLOOR_PRODUCERS:
        text = texts.get(path)
        if text is None:
            raise Failure(
                f"{path}: missing. This gate names its producers explicitly, so a\n"
                "renamed or deleted one is a failure rather than a silent pass."
            )
        for knob in FLOORED_KNOBS:
            if abstain and re.search(abstain.format(knob=knob), text):
                continue
            if not re.search(pattern.format(knob=knob), text):
                problems.append(
                    f"{path}: {what} produces {knob} without the C-array floor.\n"
                    f"    That knob's array carries `#if {knob} < 1 / #error`, so an\n"
                    f"    image declaring none of that entity would FAIL TO BUILD —\n"
                    f"    and a producer that floors it in a second spelling is how\n"
                    f"    the two lanes come to disagree (issues 0135, 0460).\n"
                    f"    A producer that deliberately does not derive this knob says\n"
                    f"    so instead, in the spelling this gate reads back."
                )

    text = texts.get(DERIVATION)
    if text is None:
        raise Failure(f"{DERIVATION}: missing.")
    start = text.find(DERIVATION_START)
    end = text.find(DERIVATION_END, start + 1) if start >= 0 else -1
    if start < 0 or end < 0:
        raise Failure(
            f"{DERIVATION}: cannot locate the pool arithmetic between\n"
            f"  `{DERIVATION_START}` and `{DERIVATION_END}`. The gate reads that\n"
            f"  slice; if the code moved, move these markers with it rather than\n"
            f"  letting the check quietly examine nothing."
        )
    if ".max(" in text[start:end]:
        problems.append(
            f"{DERIVATION}: the shared derivation RAISES a pool count.\n"
            f"    It publishes the image's DEMAND, and two consumers read it with\n"
            f"    different legal minima: the zenoh pools cannot be 0 (issue 1015)\n"
            f"    and the XRCE pools want 0 (issue 1033 — 33,296 bytes of heap a\n"
            f"    subscriber slot). A floor here reaches both, silently, and it did:\n"
            f"    it landed the day before 1033's fix and defeated it on the derive\n"
            f"    rung. Floor at the consumer that names the knob."
        )
    return problems


def read_sources() -> dict[str, str]:
    """Every TRACKED C/C++ source, from the git index.

    `git ls-files`, not a filesystem walk: `check-no-tracked-file-find` forbids
    the walk and measured why (7m36s -> 0.8s for the same 232 paths, because
    `find` stats every directory it considers pruning). It is also the more
    correct set — a walk would read whatever a local build tree happens to have
    dropped in `packages/`, and rule on arrays nobody committed.

    Found the honest way: this gate walked, its own fast-lane runs stayed green
    because the script was still UNTRACKED and therefore invisible to that gate,
    and the `pre-push` hook caught it on the first push.
    """
    names = subprocess.run(
        ["git", "-C", str(REPO), "ls-files", "-z", "--", "*.c", "*.h", "*.cpp", "*.hpp"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.split("\0")

    out = {}
    for rel in names:
        if not rel or any(s in "/" + rel for s in SKIP):
            continue
        p = REPO / rel
        # A tracked path can be absent: an uninitialised submodule's contents,
        # a file deleted in the working tree. Skipping is right; reporting OK
        # over an EMPTY set is not, which is what the check below is for.
        if p.is_file():
            out[rel] = p.read_text(errors="ignore")
    if not out:
        raise Failure("no C/C++ sources found — refusing to report OK over nothing.")
    return out


def read_producers() -> dict[str, str]:
    out = {}
    for rel in [p[0] for p in FLOOR_PRODUCERS] + [DERIVATION]:
        f = REPO / rel
        if f.is_file():
            out[rel] = f.read_text()
    return out


def run(audit: bool = False) -> int:
    found = scan(read_sources())
    problems = check_arrays(found) + check_producers(read_producers())

    if audit:
        for macro, info in sorted(found.items()):
            state = (
                "GUARDED"
                if info["guarded"]
                else "ZERO_LEGAL"
                if macro in ZERO_LEGAL
                else "unclassified"
                if macro in UNCLASSIFIED
                else "UNSTATED"
            )
            print(f"  {state:13} {macro:34} {info['path']}")
        print(f"\n  {len(found)} knob-sized arrays, {len(problems)} problem(s)")
        return 0

    if problems:
        print(
            f"check-c-array-pool-floors: {len(problems)} problem(s)\n", file=sys.stderr
        )
        for p in problems:
            print(f"  [FAIL] {p}\n", file=sys.stderr)
        return 1

    guarded = sum(1 for i in found.values() if i["guarded"])
    print(
        f"check-c-array-pool-floors: OK ({len(found)} knob-sized C arrays: "
        f"{guarded} guarded, {len(ZERO_LEGAL)} zero-legal, "
        f"{len(UNCLASSIFIED)} unclassified)"
    )
    return 0


# --- controls, run on the normal path so they cannot rot -------------------

GOOD_C = """
#ifndef POOL_MAX
#define POOL_MAX 8
#endif
#if POOL_MAX < 1
#error "POOL_MAX must be >= 1: it sizes a C array (issue 1015)"
#endif
struct s { entry_t slots[POOL_MAX]; };
"""

# The guard PRECEDES the fallback, so `POOL_MAX` is undefined where it is
# tested. The preprocessor reads an undefined macro as 0, so this fires on
# every build. Shipped as `#607`; broke `just setup tier2`.
GUARD_BEFORE_DEFINE_C = """
#if POOL_MAX < 1
#error "POOL_MAX must be >= 1: it sizes a C array (issue 1015)"
#endif
#ifndef POOL_MAX
#define POOL_MAX 8
#endif
struct s { entry_t slots[POOL_MAX]; };
"""

# The guard is INSIDE the fallback, so a `-D` skips the block and the guard
# with it — the mirror image, and the first attempted fix for the above.
GUARD_INSIDE_IFNDEF_C = """
#ifndef POOL_MAX
#define POOL_MAX 8
#if POOL_MAX < 1
#error "POOL_MAX must be >= 1: it sizes a C array (issue 1015)"
#endif
#endif
struct s { entry_t slots[POOL_MAX]; };
"""

# The fallback documents itself BETWEEN the `#ifndef` and the `#define`, which
# is what two of the tree's knobs do. An adjacency test sees no build knob here
# at all and reports OK over an array it never looked at (issue 1131).
DOCUMENTED_FALLBACK_C = """
#ifndef POOL_MAX
/* Why this number is what it is.
 *
 * Several lines of it, because the number is a decision. */
#define POOL_MAX 8
#endif
#if POOL_MAX < 1
#error "POOL_MAX must be >= 1: it sizes a C array (issue 1015)"
#endif
struct s { entry_t slots[POOL_MAX]; };
"""

# A floor ABOVE one. `STRESS_SIZE` is 16 because `build_payload()` writes a
# 12-byte header with no bounds test, so `< 1` would be a guard that states a
# minimum the code does not have.
FLOOR_ABOVE_ONE_C = """
#ifndef POOL_MAX
#define POOL_MAX 64
#endif
#if POOL_MAX < 16
#error "POOL_MAX must be >= 16: the header write is unbounded (issue 1015)"
#endif
struct s { entry_t slots[POOL_MAX]; };
"""

GOOD_CMAKE = """
_nros_c_array_pool_floor(_a "${X}" ZPICO_MAX_PUBLISHERS)
_nros_c_array_pool_floor(_b "${X}" ZPICO_MAX_SUBSCRIBERS)
_nros_c_array_pool_floor(_c "${X}" ZPICO_MAX_QUERYABLES)
"""

GOOD_RUST = """
const NOT_DERIVED_NEEDS_INFRA_COUNT: &str = "ZPICO_MAX_QUERYABLES";
        ("ZPICO_MAX_PUBLISHERS", floor(knobs.max_publishers)),
        ("ZPICO_MAX_SUBSCRIBERS", floor(knobs.max_subscribers)),
"""

GOOD_DERIVE = """
        let n = |tag: &str| per_kind.get(tag).copied().unwrap_or(0);
        let max_subscribers = n(sub) + n(ac) * ACTION_CLIENT_SUBSCRIPTIONS;
        let max_nodes = self.components().len();
"""


def producer_texts(cmake=GOOD_CMAKE, rust=GOOD_RUST, derive=GOOD_DERIVE):
    return {
        FLOOR_PRODUCERS[0][0]: cmake,
        FLOOR_PRODUCERS[1][0]: rust,
        DERIVATION: derive,
    }


def selftest() -> int:
    # POSITIVE: a guarded array with an `#ifndef` default is the shape we want.
    found = scan({"a.c": GOOD_C})
    assert found["POOL_MAX"]["guarded"] and found["POOL_MAX"]["reachable"], found

    # NEGATIVE, both shapes that shipped: a guard the preprocessor can never
    # reach with the macro defined is not a guard, and grepping for `#if K < 1`
    # cannot tell the difference. Both of these satisfied the old check.
    before = scan({"a.c": GUARD_BEFORE_DEFINE_C})["POOL_MAX"]
    assert before["guarded"] and before["reachable"] is False, before
    assert check_arrays({"POOL_MAX": before}), "a guard before its #define must FAIL"
    inside = scan({"a.c": GUARD_INSIDE_IFNDEF_C})["POOL_MAX"]
    assert inside["guarded"] and inside["reachable"] is False, inside
    assert check_arrays({"POOL_MAX": inside}), "a guard inside its #ifndef must FAIL"

    # POSITIVE: the fallback documents itself, so the `#define` is not the next
    # LINE. Two knobs in the tree are shaped this way, and the adjacency test
    # this replaced saw neither — the gate reported OK over arrays it had never
    # looked at, one of which was already guarded and got no credit (1131).
    doc = scan({"a.c": DOCUMENTED_FALLBACK_C})
    assert "POOL_MAX" in doc, "a documented #ifndef fallback must still be a knob"
    assert doc["POOL_MAX"]["guarded"] and doc["POOL_MAX"]["reachable"], doc

    # POSITIVE: a floor ABOVE one is a floor. The gate asks for the smallest
    # LEGAL value, and for some arrays that is not 1.
    above = scan({"a.c": FLOOR_ABOVE_ONE_C})["POOL_MAX"]
    assert above["guarded"] and above["reachable"], above
    unruled = [p for p in check_arrays({"POOL_MAX": above}) if "POOL_MAX" in p]
    assert not unruled, unruled
    # ...but `< 0` is still the disarm, not a floor: no value fails it.
    disarmed = scan({"a.c": FLOOR_ABOVE_ONE_C.replace("< 16", "< 0")})["POOL_MAX"]
    assert disarmed["guarded"] is False, disarmed

    # NEGATIVE 1 — the guard is gone. This is issue 1015 exactly.
    no_guard = GOOD_C.replace("#if POOL_MAX < 1", "#if POOL_MAX < 0")
    f = scan({"a.c": no_guard})
    assert f["POOL_MAX"]["guarded"] is False
    probs = check_arrays(f)
    assert any("states no\n    smallest legal value" in p for p in probs), probs

    # NEGATIVE 2 — the comparison survives and the `#error` does not, which is
    # the disarm that reads as a guard.
    no_error = GOOD_C.replace('#error "POOL_MAX must be >= 1: it sizes a C array (issue 1015)"', "")
    assert scan({"a.c": no_error})["POOL_MAX"]["guarded"] is False

    # A macro with no `#ifndef` default is not a build knob, so it is out of
    # scope — otherwise every internal constant lands in the report.
    internal = "#define INTERNAL_MAX 4\nstruct s { int a[INTERNAL_MAX]; };\n"
    assert scan({"a.c": internal}) == {}

    # An array whose extent is a knob but which nobody guards and nobody has
    # ruled on is a FAILURE, not a pass: that is the whole point.
    assert check_arrays({"NEW_MAX": {"path": "n.c", "guarded": False}})

    # ZERO_LEGAL and UNCLASSIFIED both suppress it — and both fail when the
    # tree no longer matches.
    zl = next(iter(ZERO_LEGAL))
    assert not [
        p
        for p in check_arrays({zl: {"path": "x.h", "guarded": False}})
        if "smallest legal value" in p
    ]
    assert any(
        "ALSO carries" in p for p in check_arrays({zl: {"path": "x.h", "guarded": True}})
    )
    uc = next(iter(UNCLASSIFIED))
    assert any(
        "remove it from\n    UNCLASSIFIED" in p
        for p in check_arrays({uc: {"path": "x.c", "guarded": True}})
    )
    # A table entry the scan no longer finds at all.
    stale = check_arrays({})
    assert any(f"ZERO_LEGAL names {zl}" in p for p in stale), stale
    assert any(f"UNCLASSIFIED names {uc}" in p for p in stale), stale

    # PRODUCERS. Green when both lanes floor all three knobs.
    assert check_producers(producer_texts()) == []
    # One knob dropped from the cmake bridge.
    dropped = GOOD_CMAKE.replace(
        '_nros_c_array_pool_floor(_c "${X}" ZPICO_MAX_QUERYABLES)', ""
    )
    assert any(
        "produces ZPICO_MAX_QUERYABLES without the C-array floor" in p
        for p in check_producers(producer_texts(cmake=dropped))
    )
    # The sidecar emitting a raw value.
    raw = GOOD_RUST.replace(
        '("ZPICO_MAX_SUBSCRIBERS", floor(knobs.max_subscribers))',
        '("ZPICO_MAX_SUBSCRIBERS", knobs.max_subscribers)',
    )
    assert any(
        "ZPICO_MAX_SUBSCRIBERS" in p for p in check_producers(producer_texts(rust=raw))
    )
    # An abstention only counts while it is WRITTEN DOWN: drop the line that
    # says this lane does not derive the queryable count and the knob is back to
    # being an unfloored production.
    silent = GOOD_RUST.replace(
        'const NOT_DERIVED_NEEDS_INFRA_COUNT: &str = "ZPICO_MAX_QUERYABLES";', ""
    )
    assert any(
        "ZPICO_MAX_QUERYABLES" in p for p in check_producers(producer_texts(rust=silent))
    )
    # The regression this gate was written after: the floor back in the shared
    # derivation, where it also reaches the XRCE pools.
    floored = GOOD_DERIVE.replace(
        "let max_subscribers = n(sub)", "let max_subscribers = (n(sub)).max(1) + n(sub)"
    )
    assert any(
        "RAISES a pool count" in p for p in check_producers(producer_texts(derive=floored))
    )
    # A moved marker must FAIL rather than examine an empty slice.
    try:
        check_producers(producer_texts(derive="fn derive() {}"))
    except Failure as exc:
        assert "cannot locate the pool arithmetic" in str(exc)
    else:  # pragma: no cover
        raise AssertionError("a derivation with no markers must be a Failure")

    print("check-c-array-pool-floors: selftest OK")
    return 0


if __name__ == "__main__":
    try:
        if "--selftest" in sys.argv:
            sys.exit(selftest())
        selftest()  # on the NORMAL path too, so the controls cannot rot
        sys.exit(run(audit="--audit" in sys.argv))
    except Failure as exc:
        print(f"check-c-array-pool-floors: {exc}", file=sys.stderr)
        sys.exit(1)
