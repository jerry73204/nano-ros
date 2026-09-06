#!/usr/bin/env python3
"""issue 1171 — the picolibc arena and the executor backing are ONE decision.

WHY THIS EXISTS. phase-392 W6 moved the executor's per-entry storage into the
named `.bss` static `nros_node::executor::backing::EXECUTOR_BACKING`. On Zephyr
the Rust global allocator is picolibc malloc, whose arena is itself a fixed
static sized by `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE`, and before W6 the
backing was a `Box::leak` out of it. So an image that turns the static on
without lowering that arena reserves the same bytes TWICE.

Issue 1145 paired them on one leaf by copying the size out of `nm` output:

    CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=961320   # 1048576 - 87256

That subtrahend is `ExecutorSizing::DEFAULT.u64_len() * 8`, derived from the
executor knobs. Move `MAX_CBS`, the arena or the rx buffer and it is silently
wrong, in whichever direction, and nothing said so. It was also wrong on the
OTHER board the same conf builds for: the derived size is 87,256 B on
mps2_an385 and 88,328 B on native_sim/native/64, so the second board kept
double-reserving 1,072 bytes.

THE MECHANISM this gate backs. `CONFIG_NROS_EXECUTOR_BACKING_U64S` (zephyr/
Kconfig) lets the image STATE the reservation instead of measuring it. The
reservation is then exactly `8 * words` bytes on every target, `nros-node`
refuses to compile if that is below what the executor needs, and the arena's
lowering is arithmetic anyone can check — which is what this does.

THE RULE. A `.conf` that states `CONFIG_NROS_EXECUTOR_BACKING_U64S=<words>`
with `words > 0` must also

  * set `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE`, and
  * record what it lowered that arena FROM, as `# nros-arena-base: <bytes>`,

and the three numbers must satisfy

    arena + 8 * words == base

The marker is required because "how much was this lowered by?" has no other
answer in the file: the pre-W6 number is gone the moment it is edited, and a
reviewer cannot tell a paired arena from an arbitrary one. It is checked both
ways — a marker with no pairing is as broken as a pairing with no marker.

The gate also refuses a `CONFIG_NROS_EXECUTOR_BACKING_U64S` line in a conf when
`zephyr/Kconfig` does not declare that symbol. Kconfig silently ignores an
assignment to an undeclared symbol, so such a line reads as a decision and
reaches nothing — issue 0460's failure mode, one layer up.

Usage:
    python3 scripts/check-executor-backing-arena-pairing.py [--self-test]
"""

import re
import subprocess
import sys
from pathlib import Path

BACKING_KEY = "CONFIG_NROS_EXECUTOR_BACKING_U64S"
ARENA_KEY = "CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE"
BASE_MARKER = "nros-arena-base"

# A word of the reservation is a `MaybeUninit<u64>`, so it is 8 bytes on every
# target nano-ros builds for -- which is the whole reason the knob is spelled in
# words rather than bytes: the DERIVED size is target-dependent and a stated one
# is not.
BYTES_PER_WORD = 8

BACKING_RE = re.compile(rf"^\s*{BACKING_KEY}\s*=\s*(-?\d+)\s*$", re.M)
ARENA_RE = re.compile(rf"^\s*{ARENA_KEY}\s*=\s*(\d+)\s*$", re.M)
BASE_RE = re.compile(rf"#\s*{BASE_MARKER}\s*:\s*(\d+)")


def last_int(pattern, text):
    """The LAST assignment wins, the way Kconfig merges fragments (issue 0876)."""
    found = pattern.findall(text)
    return int(found[-1]) if found else None


def check_conf(text):
    """-> list of complaint strings for one conf file's body."""
    words = last_int(BACKING_RE, text)
    arena = last_int(ARENA_RE, text)
    base = last_int(BASE_RE, text)

    stated = words is not None and words > 0
    if not stated and base is None:
        return []

    bad = []
    if base is not None and not stated:
        bad.append(
            f"carries `# {BASE_MARKER}: {base}` but states no {BACKING_KEY}, so "
            f"nothing says what the arena was lowered BY"
        )
    if stated and base is None:
        bad.append(
            f"states {BACKING_KEY}={words} but no `# {BASE_MARKER}: <bytes>`, so "
            f"the arena's lowering cannot be checked against anything"
        )
    if stated and arena is None:
        bad.append(
            f"states {BACKING_KEY}={words} but does not set {ARENA_KEY}, so the "
            f"reservation is made and nothing is given back"
        )
    if stated and base is not None and arena is not None:
        want = base - words * BYTES_PER_WORD
        if arena != want:
            bad.append(
                f"{ARENA_KEY}={arena}, but {base} - {BYTES_PER_WORD} * {words} "
                f"= {want}. Set the arena to {want}, or restate the base."
            )
    return bad


SELF_TESTS = [
    # (name, body, expected number of complaints)
    ("paired exactly", f"# {BASE_MARKER}: 1048576\n{BACKING_KEY}=11041\n{ARENA_KEY}=960248\n", 0),
    ("silent about both", f"{ARENA_KEY}=1048576\n", 0),
    # the drift this gate exists for: a knob moved, the backing was restated,
    # the arena was not.
    ("arena not lowered with it",
     f"# {BASE_MARKER}: 1048576\n{BACKING_KEY}=12000\n{ARENA_KEY}=960248\n", 1),
    ("arena lowered too far",
     f"# {BASE_MARKER}: 1048576\n{BACKING_KEY}=11041\n{ARENA_KEY}=900000\n", 1),
    ("stated with no base",
     f"{BACKING_KEY}=11041\n{ARENA_KEY}=960248\n", 1),
    ("base with no statement",
     f"# {BASE_MARKER}: 1048576\n{ARENA_KEY}=960248\n", 1),
    ("stated with no arena at all",
     f"# {BASE_MARKER}: 1048576\n{BACKING_KEY}=11041\n", 1),
    # `0` declines the static, so there is nothing to pair; `-1` derives.
    ("declined", f"{BACKING_KEY}=0\n{ARENA_KEY}=1048576\n", 0),
    ("derived", f"{BACKING_KEY}=-1\n{ARENA_KEY}=1048576\n", 0),
    # last-wins, the way Zephyr merges fragments (issue 0876)
    ("last assignment wins",
     f"# {BASE_MARKER}: 1048576\n{BACKING_KEY}=11041\n{ARENA_KEY}=1\n{ARENA_KEY}=960248\n", 0),
]


def self_test():
    bad = 0
    for name, body, expect in SELF_TESTS:
        got = check_conf(body)
        if len(got) != expect:
            print(f"  {name}: expected {expect} complaint(s), got {len(got)}: {got}")
            bad += 1
    if bad:
        print(f"check-executor-backing-arena-pairing --self-test: {bad} case(s) FAILED")
        return 1
    print(
        "check-executor-backing-arena-pairing --self-test: "
        f"{len(SELF_TESTS)} case(s) OK"
    )
    return 0


def main():
    repo = Path(
        subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            capture_output=True, text=True, check=True,
        ).stdout.strip()
    )
    if "--self-test" in sys.argv:
        return self_test()
    # On the NORMAL path too, never behind a flag: a negative control nobody
    # runs decays into a comment (AGENTS.md "a gate must run its own selftest").
    if self_test():
        return 1

    # Tracked files only -- a filesystem walk reaches build output and other
    # checkouts' worktrees (issues 1157/1166).
    confs = [
        f
        for f in subprocess.run(
            ["git", "ls-files", "-z", "*.conf"],
            capture_output=True, text=True, check=True, cwd=repo,
        ).stdout.split("\0")
        if f
    ]

    kconfig = (repo / "zephyr" / "Kconfig").read_text(errors="replace")
    declared = re.search(
        r"^config\s+NROS_EXECUTOR_BACKING_U64S\s*$", kconfig, re.M
    ) is not None

    failures, paired = [], 0
    for rel in confs:
        path = repo / rel
        if not path.is_file():
            continue
        text = path.read_text(errors="replace")
        if BACKING_KEY not in text and BASE_MARKER not in text:
            continue
        if BACKING_KEY in text and not declared:
            failures.append(
                f"  {rel}: sets {BACKING_KEY}, which zephyr/Kconfig does not "
                f"declare — Kconfig ignores an assignment to an unknown symbol, "
                f"so the line reads as a decision and reaches nothing"
            )
            continue
        bad = check_conf(text)
        if not bad:
            paired += 1
        for complaint in bad:
            failures.append(f"  {rel}: {complaint}")

    if failures:
        print("check-executor-backing-arena-pairing: FAIL\n")
        print("\n".join(failures))
        print(
            "\nThe executor backing is a `.bss` static since phase-392 W6, and on\n"
            "Zephyr the picolibc arena it used to be leaked out of is itself a\n"
            "fixed static. An image that states the reservation must give the\n"
            "same bytes back:\n"
            "\n"
            f"    # {BASE_MARKER}: <the arena before it was lowered>\n"
            f"    {BACKING_KEY}=<words>\n"
            f"    {ARENA_KEY}=<base - 8 * words>\n"
            "\n"
            "See issues 1145 and 1171."
        )
        return 1

    print(
        "check-executor-backing-arena-pairing: OK "
        f"({paired} conf(s) pair the arena with a stated backing; "
        f"{len(confs)} conf(s) scanned)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
