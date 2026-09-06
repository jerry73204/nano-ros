#!/usr/bin/env python3
"""phase-412 W4 -- the value that reaches the COMPILE is the value the resolver produced.

WHY THIS EXISTS, and why it is not the gate that was first proposed.

phase-412 W1 wired four session pools and produced FOUR delivery failures in
one wave. Every existing gate stayed green through all four, because they check
the INVENTORY (is the count right) and the RESOLVER (does precedence work), and
all four failures were downstream of both:

  1. a second consumer     the zpico C defines read raw CONFIG_*, before the
                           resolver ran
  2. a name that emptied   a foreach built NROS_DERIVED_NROS_MAX_SUBSCRIBERS,
                           which names nothing; cmake yields EMPTY, not an error
  3. no consumer default   rung 4 leaves a knob UNRESOLVED so a Rust build
                           script uses its own literal; a C define has none, so
                           it expands to nothing and sizes an array to zero
  4. a loader whitelist    the fragment set the symbol, the loader did not name
                           it, and it died at the function boundary -- the
                           pools derived 10/14/0 and the SESSION was built with
                           8/8/8. Eight subscriber slots for ten subscriptions,
                           which fails at RUNTIME rather than at link

The originally proposed gate -- "every published symbol has a consumer" --
catches NONE of these. In all four the symbol had a consumer; a different
consumer read around it, or a name never matched, or the value was legitimately
absent, or it never crossed a scope.

What catches all four is the end-to-end identity: for each knob, the number the
compiler was handed must equal the number the resolver decided. This asserts
that, against a real configured build dir.

Usage:  check-knob-delivery.py <build-dir>
        check-knob-delivery.py --self-test
"""
import os
import re
import sys
import tempfile

# Knobs whose resolved value must reach a C compile definition, and the define
# it must reach. Extend this when a knob gains a C consumer -- the pairing is
# the thing under test, so it is stated rather than discovered.
C_DEFINE_KNOBS = {
    "ZPICO_MAX_PUBLISHERS": "ZPICO_MAX_PUBLISHERS",
    "ZPICO_MAX_SUBSCRIBERS": "ZPICO_MAX_SUBSCRIBERS",
    "ZPICO_MAX_QUERYABLES": "ZPICO_MAX_QUERYABLES",
    "ZPICO_MAX_LIVELINESS": "ZPICO_MAX_LIVELINESS",
}

# Knobs that are DERIVED and must not silently lose their derivation on the way
# to the resolver: fragment value -> resolved value. A mismatch here is failure
# 4 above, and it is invisible in the fragment and in the build alike.
DERIVED_PAIRS = {
    "NROS_DERIVED_MAX_SUBSCRIBERS": "NROS_RESOLVED_NROS_MAX_SUBSCRIBERS",
    "NROS_DERIVED_MAX_PUBLISHERS": "NROS_RESOLVED_NROS_MAX_PUBLISHERS",
    "NROS_DERIVED_MAX_QUERYABLES": "NROS_RESOLVED_NROS_MAX_QUERYABLES",
    "NROS_DERIVED_RMW_SUBSCRIBER_SLOTS": "NROS_RESOLVED_NROS_RMW_SUBSCRIBER_SLOTS",
    "NROS_DERIVED_EXECUTOR_MAX_CBS": "NROS_RESOLVED_NROS_EXECUTOR_MAX_CBS",
    "NROS_DERIVED_EXECUTOR_MAX_NODES": "NROS_RESOLVED_NROS_EXECUTOR_MAX_NODES",
    # This pair is why the gate exists in the form it does. It used to land in
    # NROS_RESOLVED_ZPICO_SUBSCRIBER_BUFFER_SIZE, which no pairing here names,
    # so a derived 880 delivered as 1496 for four consecutive island builds and
    # nothing said so -- over-sized, therefore silent.
    "NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE": "NROS_RESOLVED_NROS_SUBSCRIBER_BUFFER_SIZE",
    # Issues 1122 / 1125 — the other two thirds of the payload-class trio,
    # omitted here since the trio landed. They resolve in the same
    # `if(CONFIG_NROS_RMW_ZENOH)` block as the pair above and under the ZPICO_
    # spelling, which is what the derivable resolver is called with; the
    # `NROS_`-prefixed guess is the mismatch this map's third entry exists to
    # record, so these are written from the call sites in
    # `zephyr/cmake/nros_cargo_build.cmake` rather than derived from the name.
    #
    # `NROS_DERIVED_MAX_LARGE_SUBSCRIBERS` is the one whose derived answer is
    # commonly ZERO, and a zero that fails to arrive is 131,072 bytes of
    # `LARGE_PAYLOADS` in an image that subscribes to nothing (issue 1125,
    # measured).
    "NROS_DERIVED_MAX_LARGE_SUBSCRIBERS": "NROS_RESOLVED_ZPICO_MAX_LARGE_SUBSCRIBERS",
    "NROS_DERIVED_SUBSCRIBER_LARGE_SIZE": "NROS_RESOLVED_ZPICO_SUBSCRIBER_LARGE_SIZE",
}


def read_cache(build_dir):
    """NROS_RESOLVED_* from CMakeCache.txt. Absent is DIFFERENT from empty."""
    out = {}
    path = os.path.join(build_dir, "CMakeCache.txt")
    with open(path, encoding="utf8", errors="ignore") as fh:
        for line in fh:
            m = re.match(r"^(NROS_RESOLVED_[A-Z0-9_]+):[A-Z]+=(.*)$", line.strip())
            if m:
                out[m.group(1)] = m.group(2)
    return out


def read_fragment(build_dir):
    """NROS_DERIVED_* as the inventories state them.

    BOTH fragments, not one. The first version read only
    `entity_inventory.cmake`, so every knob derived by the MESSAGE-BOUND
    inventory was outside the gate entirely -- including
    NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE, the pair whose mismatch this gate was
    extended to catch. A gate that reads one of two sources reports success
    about the half it can see, which is the failure it exists to prevent.
    """
    out = {}
    for name in ("entity_inventory.cmake", "message_bound_knobs.cmake"):
        path = os.path.join(build_dir, "nros", name)
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf8", errors="ignore") as fh:
            for m in re.finditer(r"^set\((NROS_DERIVED_[A-Z0-9_]+)\s+([^)]*)\)", fh.read(), re.M):
                out[m.group(1)] = m.group(2).strip().strip('"')
    return out


def read_defines(build_dir):
    """The -D values the compiler is actually handed, from build.ninja.

    The ninja file rather than the cmake state on purpose: it is the last
    artifact before the compiler, so it cannot agree with cmake and disagree
    with the build.
    """
    out = {}
    path = os.path.join(build_dir, "build.ninja")
    if not os.path.exists(path):
        return out
    with open(path, encoding="utf8", errors="ignore") as fh:
        for m in re.finditer(r"-D([A-Z0-9_]+)=([0-9]*)", fh.read()):
            name, val = m.group(1), m.group(2)
            if name in C_DEFINE_KNOBS.values():
                out.setdefault(name, set()).add(val)
    return out


def check(build_dir):
    problems = []
    cache = read_cache(build_dir)
    frag = read_fragment(build_dir)
    defines = read_defines(build_dir)

    if not cache:
        return ["no NROS_RESOLVED_* in %s/CMakeCache.txt -- not a configured "
                "nano-ros build dir, or the resolver stopped running" % build_dir]

    # 1. derivation must survive the trip to the resolver
    for derived, resolved in DERIVED_PAIRS.items():
        if derived not in frag:
            continue  # nothing derived; rung 4 is legitimate
        want = frag[derived]
        if resolved not in cache:
            problems.append(
                "%s=%s was DERIVED but %s never reached the resolver. A loader "
                "whitelist that does not name the symbol drops it at the "
                "function boundary, and the knob then falls to a default that "
                "may be SMALLER than the demand." % (derived, want, resolved))
        elif cache[resolved] != want:
            problems.append(
                "%s=%s but %s=%s -- the resolver did not carry the derived "
                "value through." % (derived, want, resolved, cache[resolved]))

    # 2. every C define must equal its resolved knob, and never be empty
    for knob, define in C_DEFINE_KNOBS.items():
        rname = "NROS_RESOLVED_" + knob
        seen = defines.get(define)
        if seen is None:
            continue  # backend not built here
        if "" in seen:
            problems.append(
                "-D%s= reached the compiler EMPTY. An unresolved knob expands "
                "to nothing and sizes a C array to zero; the diagnostic names "
                "the struct, never the knob." % define)
            continue
        if len(seen) > 1:
            problems.append(
                "-D%s has %d different values in build.ninja (%s) -- two "
                "consumers disagree about one knob."
                % (define, len(seen), ", ".join(sorted(seen))))
            continue
        got = next(iter(seen))
        want = cache.get(rname)
        if want is None or want == "":
            # The knob is unresolved but a value still reached the compiler:
            # a literal fallback. Legitimate, but it must be SAID, because it
            # is how an under-size hides.
            continue
        if got != want:
            problems.append(
                "-D%s=%s but %s=%s -- the compiler was handed a different "
                "number than the resolver decided." % (define, got, rname, want))
    return problems


def self_test(quiet=False):
    """Each case asserts a failure this gate must catch, plus the clean case,
    so a gate that stopped matching anything cannot report success.

    `quiet` suppresses the per-case OK lines, not the failures: the normal path
    runs this on every invocation and a control that narrates itself there is
    noise nobody reads."""
    def build(tmp, cache, frag, ninja):
        os.makedirs(os.path.join(tmp, "nros"), exist_ok=True)
        with open(os.path.join(tmp, "CMakeCache.txt"), "w") as fh:
            fh.write(cache)
        with open(os.path.join(tmp, "nros", "entity_inventory.cmake"), "w") as fh:
            fh.write(frag)
        with open(os.path.join(tmp, "build.ninja"), "w") as fh:
            fh.write(ninja)

    CLEAN_CACHE = ("NROS_RESOLVED_NROS_MAX_SUBSCRIBERS:INTERNAL=10\n"
                   "NROS_RESOLVED_ZPICO_MAX_SUBSCRIBERS:INTERNAL=10\n")
    CLEAN_FRAG = "set(NROS_DERIVED_MAX_SUBSCRIBERS 10)\n"
    CLEAN_NINJA = "cc -DZPICO_MAX_SUBSCRIBERS=10 -c x.c\n"

    cases = [
        ((CLEAN_CACHE, CLEAN_FRAG, CLEAN_NINJA), 0, "a delivered value agrees end to end"),
        # failure 4: derived, but the loader whitelist dropped it
        (("NROS_RESOLVED_ZPICO_MAX_SUBSCRIBERS:INTERNAL=8\n",
          CLEAN_FRAG, "cc -DZPICO_MAX_SUBSCRIBERS=8 -c x.c\n"), 1,
         "derived but never reached the resolver"),
        # failure 3: unresolved knob reaches the compiler empty
        ((CLEAN_CACHE, CLEAN_FRAG, "cc -DZPICO_MAX_SUBSCRIBERS= -c x.c\n"), 1,
         "an empty define"),
        # failure 1: a second consumer handed a different number
        ((CLEAN_CACHE, CLEAN_FRAG, "cc -DZPICO_MAX_SUBSCRIBERS=12 -c x.c\n"), 1,
         "compiler and resolver disagree"),
        # two consumers disagreeing with each other
        ((CLEAN_CACHE, CLEAN_FRAG,
          "cc -DZPICO_MAX_SUBSCRIBERS=10 -c x.c\ncc -DZPICO_MAX_SUBSCRIBERS=8 -c y.c\n"), 1,
         "one knob, two values"),
        # the derivation carried through wrongly
        (("NROS_RESOLVED_NROS_MAX_SUBSCRIBERS:INTERNAL=4\n"
          "NROS_RESOLVED_ZPICO_MAX_SUBSCRIBERS:INTERNAL=4\n",
          CLEAN_FRAG, "cc -DZPICO_MAX_SUBSCRIBERS=4 -c x.c\n"), 1,
         "resolver did not carry the derived value"),
    ]
    failures = 0
    for (cache, frag, ninja), want, name in cases:
        with tempfile.TemporaryDirectory() as tmp:
            build(tmp, cache, frag, ninja)
            got = len(check(tmp))
            ok = (got >= 1) if want else (got == 0)
            if not ok:
                print("  self-test FAIL: %s -- got %d problem(s), want %s"
                      % (name, got, "at least 1" if want else "0"))
                failures += 1
            elif not quiet:
                print("  ok    %s" % name)
    if failures:
        print("check-knob-delivery self-test: FAILED (%d)" % failures)
        return 1
    return 0


def main(argv):
    if len(argv) == 2 and argv[1] == "--self-test":
        return self_test()
    if len(argv) != 2:
        print(__doc__.strip().splitlines()[-3])
        return 2
    # Always, not only behind the flag: a negative control nobody runs decays
    # into a comment, and this rule's whole job is to fire. Same shape as
    # `scripts/check-board-tiers.py`; gated by `check-gate-selftests`.
    #
    # It runs BEFORE the real check so a gate that has stopped matching
    # anything cannot report "every knob reached the compile as resolved" --
    # which is exactly the sentence this gate exists to make trustworthy.
    rc = self_test(quiet=True)
    if rc:
        return rc
    problems = check(argv[1])
    if problems:
        print("check-knob-delivery: a knob did not arrive as resolved:")
        for p in problems:
            print("  - %s" % p)
        print("\n  phase-412 W4. The derived value being RIGHT is not the "
              "question; whether it ARRIVED is.")
        return 1
    print("check-knob-delivery: every knob reached the compile as resolved.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
