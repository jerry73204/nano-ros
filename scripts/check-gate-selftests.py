#!/usr/bin/env python3
"""A gate must exercise its own failure path every time it runs — phase-395.

WHY THIS IS THE ONE OBLIGATION CI CAN ACTUALLY CARRY

Of the three things we ask of agent-submitted work, two are not mechanically
checkable and should not pretend to be: "separate what you measured from what
you reasoned" and "name the command behind a coverage claim" are conventions,
because CI can check that a section EXISTS and never that it is HONEST. A gate
that only checks the form is present reports compliance it never established,
which is the failure this whole family of checks exists to prevent.

This one is different. "The gate can fail" is a property of the gate, not a
claim about a person, so it can be enforced.

WHY 'RUNS ON THE NORMAL PATH' AND NOT MERELY 'EXISTS'

`check-board-tiers.py` says it best, in its own comment: a negative control
nobody runs decays into a comment. A selftest behind `--selftest` is run once,
by its author, on the day it is written; afterwards it is prose. Running it on
every invocation converts "someone once demonstrated a red" from a claim in a
commit message into something re-verified on every push — which is exactly the
difference between a report and a measurement.

That is also why the fix for a violation is never "add `--selftest` to the
justfile line". Two invocations of the same script is twice the cost and still
leaves the direct callers unprotected.

WHY A BASELINE RATCHET, NOT A DIFF SCOPE

Diff-scoping looks natural (only check what changed) and is quietly vacuous:
run it on `main` with nothing in the diff and it examines zero files while
printing OK. A baseline checks all of them, every time, and tightens by itself —
a script that GAINS a selftest must leave the baseline, so the debt can only
shrink.

Usage::

    check-gate-selftests.py                  # the gate
    check-gate-selftests.py --audit          # full picture, never fails
    check-gate-selftests.py --write-baseline # after fixing scripts
"""

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASELINE = os.path.join(ROOT, ".config", "gate-selftest-baseline.txt")

# A call to a selftest routine. Definitions and flag-dispatch lines are excluded
# by the caller, not here, so both halves are visible at the use site.
CALL = re.compile(r"\b(self_?test|_selftest|selftest)\s*\(")
# The same call as SHELL writes it: a bare command at statement position, no
# parentheses.
#
# Without this the rule was UNSATISFIABLE for shell. `CALL` requires `name(`,
# which bash call syntax never produces, so a `.sh` could only ever classify
# `none` or `flag-only` however good its negative control was — and the two
# spellings were indistinguishable. Demonstrated by running the classifier on
# a script that does the right thing:
#
#     self_test() { return 0; }
#     self_test || exit 1        →  classified `flag-only`
#
# identical to the verdict for the same call hidden behind `--self-test`.
#
# Measured when this was found: of 159 gate scripts, 75 are shell and 0 of them
# were compliant; all 40 compliant scripts were `.py`. A ratchet one whole
# language cannot pay into is not a ratchet on that language, it is a silent
# exemption for it — and it was silent in the direction that matters, since
# those 75 sat in the "still owe one" count looking like ordinary debt.
#
# `DEFN` still excludes the definition line (`self_test() {`), so this matches
# the invocation only, and the flag-branch lookback below applies unchanged.
BASH_CALL = re.compile(r"^\s*[A-Za-z0-9_]*self_?test[A-Za-z0-9_]*\s*(?:\|\||&&|;|$)")
DEFN = re.compile(r"^\s*(def |function |\w+\s*\(\)\s*\{)")
FLAG = re.compile(r"--self")
HAS_SELFTEST = re.compile(r"self.?test", re.I)


# `import "just/x.just"` MERGES that file's recipes into the root namespace —
# unlike `mod`, which namespaces them. A gate that enumerates `check-*` recipes
# therefore has to read the imported files too, or every moved recipe reads as
# "backs no `check-*` recipe any more" while it is running on every push.
#
# Same latent hole `check-just-recipe-refs` carried: invisible while every gate
# happened to live in one file, and surfacing as a flood — 117 at once — the
# moment phase-399 moved 200 of them into `just/check.just`.
IMPORT_DEF = re.compile(r'^import\s+[\'"]([^\'"]+)[\'"]', re.M)


def _justfile_sources():
    """The root justfile plus every file it `import`s (not `mod`s)."""
    root = os.path.join(ROOT, "justfile")
    with open(root, encoding="utf8") as fh:
        text = fh.read()
    files = [root]
    for rel in IMPORT_DEF.findall(text):
        f = os.path.join(ROOT, rel)
        if os.path.exists(f):
            files.append(f)
    return files


def gate_scripts():
    """Scripts invoked by a gate — i.e. by a recipe in the `check` module.

    A gate used to be spelled `check-foo:` at the root, so "is this a gate?"
    was a question about the NAME. The `check` module now holds them as bare
    names (`foo:`), so it becomes a question about the FILE — which is the more
    honest one anyway, since the prefix was only ever a namespace worn as a
    name.

    Deliberately the `just/check.just` import closure and not the root: widening
    it to every recipe in every justfile made this report 135 problems, most of
    them root verbs like `bootstrap.sh` that assert nothing and were never
    gates.
    """
    # `just/check.just` AND the topic files it imports. The gates moved into
    # `just/check/*.just`; reading only the index finds the seven lane recipes
    # and no gate scripts at all, which would report every one of the 121
    # baseline entries as "backs no `check-*` recipe any more" -- loud, but
    # loud about the wrong thing.
    gate_file = os.path.join(ROOT, "just", "check.just")
    with open(gate_file, encoding="utf8") as fh:
        text = fh.read()
    for rel in re.findall(r"^import\s+'([^']+)'", text, re.MULTILINE):
        imported = os.path.join(os.path.dirname(gate_file), rel)
        if os.path.isfile(imported):
            with open(imported, encoding="utf8") as fh:
                text += "\n" + fh.read()
    lines = text.split("\n")
    found, in_recipe = set(), False
    for line in lines:
        if re.match(r"^[a-z][a-z0-9-]*[ :]", line):
            in_recipe = True
            continue
        if line and not line[0].isspace():
            in_recipe = False
        if not in_recipe:
            continue
        for m in re.finditer(r"scripts/[A-Za-z0-9._/-]+\.(?:sh|py)", line):
            p = m.group(0)
            # `scripts/build/**` PRODUCES artifacts; it does not assert, so it
            # is not a gate and cannot have a failure path to exercise. A gate
            # legitimately invokes one as a prerequisite —
            # `check-source-gates` builds its own compile-check stamps — and
            # counting that as a gate demanded a selftest of a build step.
            if p.startswith("scripts/build/"):
                continue
            if os.path.exists(os.path.join(ROOT, p)):
                found.add(p)
    return sorted(found)


def classify(rel):
    """('auto' | 'flag-only' | 'none', evidence)."""
    with open(os.path.join(ROOT, rel), encoding="utf8", errors="replace") as fh:
        lines = fh.read().split("\n")
    if not any(HAS_SELFTEST.search(l) for l in lines if not l.strip().startswith("#")):
        return "none", ""
    for i, line in enumerate(lines, 1):
        stripped = line.strip()
        if stripped.startswith("#") or DEFN.match(line) or FLAG.search(line):
            continue
        if not CALL.search(line) and not BASH_CALL.match(line):
            continue
        # A call INSIDE the flag branch is not an automatic run. Line matching
        # alone cannot see that — `self_test()` under `if "--selftest" in argv:`
        # carries no `--self` on its own line — so look back for a guard opened
        # at a shallower indent. This gate's own selftest caught the line-based
        # version classifying exactly that shape as automatic.
        indent = len(line) - len(line.lstrip())
        guarded = False
        for prev in reversed(lines[max(0, i - 4):i - 1]):
            if not prev.strip() or prev.strip().startswith("#"):
                continue
            prev_indent = len(prev) - len(prev.lstrip())
            if prev_indent < indent and FLAG.search(prev):
                guarded = True
            if prev_indent < indent:
                break
        if not guarded:
            return "auto", f"{rel}:{i}: {stripped[:70]}"
    return "flag-only", ""


def load_baseline():
    if not os.path.exists(BASELINE):
        raise SystemExit(
            f"check-gate-selftests: baseline missing at {BASELINE}.\n"
            "  It is tracked, so its absence is a PATH bug, not an empty ratchet.\n"
            "  Regenerate deliberately with --write-baseline."
        )
    with open(BASELINE, encoding="utf8") as fh:
        return {l.strip() for l in fh if l.strip() and not l.startswith("#")}


def write_baseline(states):
    debt = sorted(r for r, (s, _) in states.items() if s != "auto")
    with open(BASELINE, "w", encoding="utf8") as fh:
        fh.write(
            "# Gate scripts that do NOT yet run their own selftest on the normal\n"
            "# path. A RATCHET, not an allowlist: this file may only shrink.\n"
            "#\n"
            "# `check-gate-selftests` fails when a script here gains a selftest\n"
            "# (remove its line) or disappears (delete its line) — so the debt\n"
            "# cannot silently grow and cannot silently go stale, which is the\n"
            "# issue-0743 class.\n"
            "#\n"
            "# Regenerate: python3 scripts/check-gate-selftests.py --write-baseline\n"
        )
        for r in debt:
            fh.write(r + "\n")
    print(f"wrote {BASELINE} — {len(debt)} script(s) of {len(states)} still owe a selftest")


def main():
    audit = "--audit" in sys.argv
    states = {r: classify(r) for r in gate_scripts()}

    if "--write-baseline" in sys.argv:
        write_baseline(states)
        return 0

    if audit:
        by = {}
        for r, (s, _) in states.items():
            by.setdefault(s, []).append(r)
        print(f"gate scripts backing a `check-*` recipe: {len(states)}")
        for k, label in (("auto", "runs its selftest on the normal path"),
                         ("flag-only", "has a selftest, only behind a flag"),
                         ("none", "no selftest at all")):
            print(f"  {len(by.get(k, [])):3d}  {label}")
        for r in sorted(by.get("flag-only", [])):
            print(f"    flag-only: {r}")
        return 0

    baseline = load_baseline()
    errs = []

    for rel, (state, _ev) in sorted(states.items()):
        if state != "auto" and rel not in baseline:
            errs.append(
                f"{rel}: a gate must run its own selftest on the NORMAL path.\n"
                f"      state: {state}\n"
                f"      A negative control nobody runs decays into a comment. Call\n"
                f"      it from main (see scripts/check-board-tiers.py), do NOT add\n"
                f"      a second `--selftest` invocation to the justfile."
            )

    for rel in sorted(baseline):
        if rel not in states:
            errs.append(
                f"{rel}: in the baseline but backs no `check-*` recipe any more.\n"
                f"      Delete the line. A stale entry is inert while reading as\n"
                f"      tracked debt — the issue-0743 class."
            )
        elif states[rel][0] == "auto":
            errs.append(
                f"{rel}: now runs its selftest — remove it from the baseline.\n"
                f"      The ratchet only tightens; leaving it here lets the gate\n"
                f"      be silently loosened again later."
            )

    if errs:
        print(f"check-gate-selftests: {len(errs)} problem(s):\n", file=sys.stderr)
        for e in errs:
            print(f"  - {e}", file=sys.stderr)
        print(f"\n  Baseline: {os.path.relpath(BASELINE, ROOT)} "
              f"(--write-baseline after fixing, --audit for the full picture)",
              file=sys.stderr)
        return 1

    auto = sum(1 for s, _ in states.values() if s == "auto")
    print(f"check-gate-selftests OK — {auto}/{len(states)} gate script(s) run their own "
          f"selftest; {len(baseline)} still owe one and may only decrease.")
    return 0


def self_test(quiet=True):
    """Prove the classifier can fail. Runs on EVERY invocation — this gate must
    hold itself to the rule it enforces, or it is advice rather than a gate."""
    import tempfile
    cases = [
        ("auto", "def self_test():\n    pass\n\ndef main():\n    self_test()\n"),
        ("flag-only", 'def self_test():\n    pass\n\nif "--selftest" in sys.argv:\n'
                      "    self_test()\n"),
        ("none", "def main():\n    return 0\n"),
        # A call on a `--self` dispatch line must NOT count as automatic.
        ("flag-only", 'def selftest():\n    pass\n\nif a == "--self-test": selftest()\n'),
        # A commented-out call must not count either.
        ("flag-only", "def self_test():\n    pass\n\n#    self_test()\n"),
        # SHELL call syntax — no parentheses. Before `BASH_CALL` these read as
        # `flag-only` however good the control was, so 0 of 75 shell gate
        # scripts could ever comply.
        ("auto", "self_test() {\n    return 0\n}\n\nself_test || exit 1\n"),
        ("auto", "_cc_selftest() {\n    return 0\n}\n\n_cc_selftest\n"),
        # …and a shell call behind a flag branch must STILL be flag-only,
        # exactly as the Python one is. Without this case the fix above would
        # be free to classify every shell selftest as automatic, which trades a
        # silent exemption for a false pass.
        ("flag-only", 'self_test() {\n    return 0\n}\n\nif [ "$1" = "--self-test" ]; then\n'
                      "    self_test\nfi\n"),
    ]
    fails = []
    with tempfile.TemporaryDirectory() as d:
        for i, (want, body) in enumerate(cases):
            rel = f"t{i}.py"
            with open(os.path.join(d, rel), "w", encoding="utf8") as fh:
                fh.write(body)
            g, real = globals(), globals()["ROOT"]
            g["ROOT"] = d
            got = classify(rel)[0]
            g["ROOT"] = real
            if got != want:
                fails.append(f"case {i}: expected {want}, got {got}")
    if fails:
        for f in fails:
            print(f"check-gate-selftests self-test: FAIL {f}", file=sys.stderr)
        raise SystemExit(1)
    if not quiet:
        print("check-gate-selftests self-test: OK")
    return 0


if __name__ == "__main__":
    if "--self-test" in sys.argv or "--selftest" in sys.argv:
        sys.exit(self_test(quiet=False))
    # Always, not only behind the flag: this gate enforces exactly this rule.
    self_test()
    sys.exit(main())
