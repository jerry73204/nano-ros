#!/usr/bin/env python3
"""Every `NROS_DECLARED_*` fact is produced, consumed, and WATCHED — issue 1122.

A declared fact is how a number CMake computed crosses into cargo on a lane
that has no Kconfig. `NROS_DERIVED_*` is the Zephyr road (`nros_resolve_knobs()`
-> `NROS_RESOLVED_*`, gated by `check-knob-delivery`); `NROS_DECLARED_*` is the
lane-independent one, carried by `corrosion_set_env_vars` and read as a DEFAULT
by a build script.

Issue 1122 measured what happens when the second road is missing: every derived
pool knob in the tree was computed, written to `message_bound_knobs.cmake`, and
consumed only under `zephyr/`, so a FreeRTOS image carried 131,072 B of
`LARGE_PAYLOADS` while its own build dir held
`set(NROS_DERIVED_MAX_LARGE_SUBSCRIBERS 0)`.

Three rules, and each is a defect this tree has already had:

1. PRODUCED — the name appears in a `corrosion_set_env_vars` payload under
   `cmake/`. A fact nothing writes is a default that silently never applies.

2. CONSUMED — some Rust build-side file reads it with `std::env::var`. A fact
   nothing reads is 1122's own shape one step earlier: computed, delivered,
   discarded.

3. WATCHED — the file that reads it also declares
   `cargo:rerun-if-env-changed=<name>`. This is the rule with a measured
   history: `resolve_queryable_default` consumed `NROS_DECLARED_SERVICE_SERVERS`
   and `NROS_DECLARED_INFRA_QUERYABLES` without declaring either, so an entry
   that gained or lost a service server kept its previously-sized tables until
   something else forced a rebuild -- the sizing read as applied while being
   stale, and setting the variable by hand produced a byte-identical image.

Deliberately NOT checked: that the VALUE is right. That is a build's job, and
`check-knob-delivery` already owns the Zephyr half. This gate answers the
cheaper question the tree kept getting wrong -- whether the wire is connected at
all.
"""

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# issue 1199 — the two roads a derived number takes into cargo, paired.
#
# `leaf_entity_env.rs` writes cargo `[env]` for a Rust LEAF; `NanoRosEntityFacts.cmake`
# writes `NROS_DECLARED_*` through `corrosion_set_env_vars` for a CMake image
# with no leaf. They must deliver the same KNOBS, or an image's sizing depends
# on which lane built it -- which is issue 1122 with the roads swapped.
#
# The names differ by construction: the sidecar may write the knob itself
# (cargo `[env]` without `force` is still overridable), while the CMake road
# must not (the child environment is not), so it hands over a DECLARED fact the
# build script takes as a default. This map is the one place that pairing is
# written down.
ROAD_PAIRS = {
    # sidecar DERIVED_ENV_KEYS
    "NROS_EXECUTOR_ACTION_CLIENTS": "NROS_DECLARED_EXECUTOR_ACTION_CLIENTS",
    "NROS_EXECUTOR_MAX_CBS": "NROS_DECLARED_EXECUTOR_MAX_CBS",
    "NROS_RMW_SUBSCRIBER_SLOTS": "NROS_DECLARED_RMW_SUBSCRIBER_SLOTS",
    "ZPICO_MAX_PUBLISHERS": "NROS_DECLARED_MAX_PUBLISHERS",
    "ZPICO_MAX_SUBSCRIBERS": "NROS_DECLARED_MAX_SUBSCRIBERS",
    # sidecar DERIVED_PAYLOAD_ENV_KEYS
    "NROS_SUBSCRIBER_BUFFER_SIZE": "NROS_DECLARED_SUBSCRIBER_BUFFER_SIZE",
    "ZPICO_MAX_LARGE_SUBSCRIBERS": "NROS_DECLARED_LARGE_SUBSCRIBERS",
    "ZPICO_SUBSCRIBER_LARGE_SIZE": "NROS_DECLARED_SUBSCRIBER_LARGE_SIZE",
}

# Facts with no leaf-road twin, each for a stated reason.
ROAD_UNPAIRED = {
    "NROS_DECLARED_INFRA_QUERYABLES":
        "completes ZPICO_MAX_QUERYABLES, which is DELIBERATELY NOT DERIVED "
        "(issue 1061): the count excludes the param and lifecycle service "
        "families a feature enables. A leaf has no such channel.",
    "NROS_DECLARED_SERVICE_SERVERS":
        "the raw declared count behind the same queryable sizing (phase-392 W5).",
    "NROS_DECLARED_QOS_MODELS": "QoS wiring, not a pool size.",
    "NROS_DECLARED_QOS_PENDING": "QoS wiring, not a pool size.",
    "NROS_DECLARED_QOS_SCHEDULED": "QoS wiring, not a pool size.",
}

LEAF_ENV = "packages/cli/nros-cli-core/src/leaf_entity_env.rs"

NAME_RE = re.compile(r"\bNROS_DECLARED_[A-Z0-9_]+\b")
WATCH_RE = re.compile(r"cargo:rerun-if-env-changed=(NROS_DECLARED_[A-Z0-9_]+)")
# A quoted literal in a build-side file is the READ. It used to be
# `env::var("…")`, which stopped being where the name appears once the readers
# became helpers taking the name as a parameter — and a rule that only sees one
# spelling reports a wired fact as unconsumed (issue 1199, caught by this gate
# on itself). Quoted, so a name mentioned in prose does not count; comments in
# this tree spell them in backticks.
READ_RE = re.compile(r'"(NROS_DECLARED_[A-Z0-9_]+)"')


def tracked(*globs):
    out = subprocess.run(
        ["git", "-C", str(ROOT), "ls-files", "--", *globs],
        capture_output=True, text=True, check=True,
    ).stdout.split()
    return [ROOT / p for p in out]


def produced():
    """Names appearing in a `corrosion_set_env_vars` payload under cmake/."""
    names = set()
    for f in tracked("cmake/*.cmake", "cmake/**/*.cmake"):
        text = f.read_text(errors="replace")
        # The payload is built up in a list variable and passed through, so the
        # honest test is "this file both names the fact and calls the setter" —
        # matching the call's arguments would miss every accumulating site,
        # which is how every producer in this tree is written.
        if "corrosion_set_env_vars" not in text:
            continue
        names |= set(NAME_RE.findall(text))
    return names


def consumed():
    """name -> {file: (reads, watches)} over the Rust build side."""
    seen = {}
    # BUILD-SIDE only. `packages/**/*.rs` also sweeps CLI and runtime sources,
    # where a `NROS_DECLARED_*` literal is the PRODUCER naming what it writes,
    # not a consumer reading it — and demanding `rerun-if-env-changed` of a
    # non-build script is a finding that cannot be acted on.
    for f in tracked("packages/**/build.rs", "packages/**/*-build/src/*.rs"):
        text = f.read_text(errors="replace")
        if "NROS_DECLARED_" not in text:
            continue
        reads = set(READ_RE.findall(text))
        watches = set(WATCH_RE.findall(text))
        for n in reads | watches:
            seen.setdefault(n, []).append((f, n in reads, n in watches))
    return seen


def check_roads(prod):
    """The two roads must deliver the same knobs (issue 1199)."""
    findings = []
    leaf = (ROOT / LEAF_ENV).read_text(errors="replace")
    leaf_keys = set()
    for const in ("DERIVED_ENV_KEYS", "DERIVED_PAYLOAD_ENV_KEYS"):
        m = re.search(const + r"[^=]*=\s*&\[(.*?)\];", leaf, re.S)
        if m:
            leaf_keys |= set(re.findall(r'"([A-Z0-9_]+)"', m.group(1)))
    for key in sorted(leaf_keys - set(ROAD_PAIRS)):
        findings.append(
            "%s is on the cargo-LEAF road and has no CMake twin in ROAD_PAIRS.\n"
            "       An image sized one way by a leaf build and another way by a\n"
            "       CMake build is issue 1122 with the roads swapped." % key
        )
    for key, fact in sorted(ROAD_PAIRS.items()):
        if key not in leaf_keys:
            findings.append(
                "%s is paired to %s here but is no longer on the\n"
                "       cargo-LEAF road. Either the leaf dropped it deliberately --\n"
                "       then say so in ROAD_UNPAIRED -- or the roads have drifted."
                % (key, fact)
            )
        elif fact not in prod:
            findings.append(
                "%s is on the cargo-LEAF road; its CMake twin %s\n"
                "       is produced by no cmake file, so a CMake image with no Rust\n"
                "       leaf keeps the crate default (issue 1122)." % (key, fact)
            )
    return findings


def check():
    prod = produced()
    cons = consumed()
    findings = []

    for name in sorted(prod - set(cons)):
        findings.append(
            "%s is PRODUCED by cmake and read by no build script.\n"
            "       A fact nothing consumes is a default that never applies —\n"
            "       issue 1122's shape one step earlier." % name
        )

    for name in sorted(set(cons) - prod):
        findings.append(
            "%s is read on the Rust side and PRODUCED by no cmake file.\n"
            "       Nothing will ever set it, so the default silently wins."
            % name
        )

    for name in sorted(set(cons) & prod):
        for path, reads, watches in cons[name]:
            rel = path.relative_to(ROOT)
            if reads and not watches:
                findings.append(
                    "%s is read in %s with no\n"
                    "       `cargo:rerun-if-env-changed=%s`. cargo will not\n"
                    "       re-run this script when the declaration changes, so the\n"
                    "       sizing reads as applied while being STALE (issue 1122)."
                    % (name, rel, name)
                )
    findings += check_roads(prod)
    return findings


def self_test():
    """The gate must reject each of the three shapes it exists to catch."""
    failures = 0

    def case(label, prod_set, cons_map, want):
        nonlocal failures
        global produced, consumed, check_roads
        p_orig, c_orig, r_orig = produced, consumed, check_roads
        check_roads = lambda _p: []  # noqa: E731
        produced = lambda: prod_set                      # noqa: E731
        consumed = lambda: cons_map                      # noqa: E731
        try:
            got = len(check())
        finally:
            produced, consumed, check_roads = p_orig, c_orig, r_orig
        ok = (got > 0) == want
        if not ok:
            failures += 1
        print("  %-46s %s" % (label, "ok" if ok else "FAILED"))

    f = ROOT / "x.rs"
    case("produced, consumed and watched -> clean",
         {"NROS_DECLARED_X"}, {"NROS_DECLARED_X": [(f, True, True)]}, False)
    case("produced, never consumed -> finding",
         {"NROS_DECLARED_X"}, {}, True)
    case("consumed, never produced -> finding",
         set(), {"NROS_DECLARED_X": [(f, True, True)]}, True)
    case("consumed without a watch -> finding",
         {"NROS_DECLARED_X"}, {"NROS_DECLARED_X": [(f, True, False)]}, True)
    # A watch with no read is fine: a script may watch a fact it forwards.
    case("watched but not read -> clean",
         {"NROS_DECLARED_X"}, {"NROS_DECLARED_X": [(f, False, True)]}, False)

    print("check-declared-fact-carriers --self-test: %d check(s) failed" % failures)
    return 1 if failures else 0


def main():
    # The negative control runs on the NORMAL path, not only behind a flag:
    # a gate that can no longer fail is a comment, and nobody types the flag.
    if self_test():
        return 1
    if "--self-test" in sys.argv:
        return 0
    findings = check()
    if findings:
        print("check-declared-fact-carriers: %d problem(s)" % len(findings),
              file=sys.stderr)
        for f in findings:
            print("  " + f, file=sys.stderr)
        return 1
    prod = produced()
    print("check-declared-fact-carriers: %d declared fact(s) produced, "
          "consumed and watched: %s" % (len(prod), ", ".join(sorted(prod))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
