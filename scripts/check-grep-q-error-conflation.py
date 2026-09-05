#!/usr/bin/env python3
"""Forbid NEW `grep -q` conditionals that cannot tell an error from a non-match.

Issue 0726. `grep` exits 1 for "no match" and >=2 for an error. Both natural
spellings conflate them, in opposite directions:

    if ! … grep -q PAT …    an error becomes a FINDING that is not real
    if   … grep -q PAT …    an error makes the check silently NOT FIRE

The first was a live defect: under a 32-way gate fan-out a forked grep failed to
start and `check-rmw-force-link-anchor` reported a missing force-link anchor for
an example that has one. Only ever green->red under load, which is the direction
that teaches people to stop believing a gate.

The fix is `nros_grep_q` (scripts/lib/grep-q.sh): 0 match, 1 no-match, exit 2 on
tool failure.

This gate is a RATCHET, not a cleanup. The sweep found 87 pre-existing sites, and
converting them blind would churn 87 diffs to fix an unknown fraction — for many
the searched text is certainly present-or-absent and no error is possible. So the
existing sites are baselined by COUNT PER FILE and the gate fails when a file
grows a new one. Lowering a baseline is always allowed; raising it is the thing
that needs a reason.

phase-395 W11 spent one of those lowerings: the 46 sites in the 21 scripts a
`check-fast` gate INVOKES are converted, and those files are at 0. That is the
population fan-out actually stresses, so it is the population where the
conflation is not theoretical. The rest of the baseline is untouched debt.

Then a second lowering for the SAME reason one directory over: the 30 sites in
the 7 `packages/testing/nros-tests/tests/*.sh` gate scripts that
`check-{provider-index,build-root,workspace-order,cargo-target-spelling,
fixture-groups,package-xml-comments}` invoke, plus `size_probe_verify.sh`.
Issue 0732 put that directory in scope after `workspace_order_gate.sh`
announced a false finding from a SIGPIPE; these scripts are in the fan-out set
too, so the conflation there is no more theoretical than it was in `scripts/`.
Both lowerings left the CAPTURE form (`grep … || true` into a variable the
caller then tests for emptiness) untouched where it survives — this gate does
not see it, and there is no `-q` helper for it.
"""

import json
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASELINE = os.path.join(ROOT, "scripts", "grep-q-baseline.json")

# A `grep -q` whose STATUS drives control flow. A bare `grep -q` on its own line
# (status discarded, or captured into a variable the caller then inspects) is
# not this defect — that is the shape the fix itself uses.
COND = re.compile(
    r"""(?x)
    (?: \bif\s+!?\s* .* \bgrep\s+-[a-zA-Z]*q )      # if [!] … grep -q
  | (?: \bgrep\s+-[a-zA-Z]*q .* \|\| )              # grep -q … || …
  | (?: \bgrep\s+-[a-zA-Z]*q .* \&\& )              # grep -q … && …
  | (?: \bwhile\s+!?\s* .* \bgrep\s+-[a-zA-Z]*q )   # while [!] … grep -q
    """
)

SUFFIXES = (".sh", ".just", ".py")


# Issue 0732 — the gate scripts under `packages/*/tests/` are checkers too, and
# they were outside this sweep. `workspace_order_gate.sh` announced "the provider
# stopped being discoverable" from a SIGPIPE in its own harness while sitting in
# no baseline at all, because nothing here looked at that directory. A gate whose
# scope is narrower than the rule it enforces is issue 0196's shape, and the
# whole point of this one is that a checker must not report a tool failure as a
# finding — which is no less true of a checker that happens to live beside the
# tests it guards.
SEARCH_ROOTS = ["scripts", "just", "justfile", "packages", "tools"]

# A checker whose SUBJECT is this pattern quotes it, in regexes, in its docstring
# and in its self-test fixtures. Those are not shell conditionals — they are the
# thing that finds shell conditionals — so counting them makes the count say
# something it does not mean. Exempted BY NAME rather than by a baseline bump: a
# baseline entry would record 22 "sites" that are not sites, and the next person
# lowering that number would be chasing text in a Python string.
#
# `check-pipefail-sigpipe-assertions` (issue 1077) covers the neighbouring rule —
# a status-consuming pipeline into an early-exit matcher — which this gate
# structurally cannot see, because it skips any line naming `nros_grep_q` and
# `printf | nros_grep_q` has the same race plus a subshell that swallows the
# helper's `exit 2`.
SELF_REFERENTIAL = ("scripts/check-pipefail-sigpipe-assertions.py",)


def tracked():
    out = subprocess.run(
        ["git", "-C", ROOT, "ls-files", "--", *SEARCH_ROOTS],
        capture_output=True, text=True, check=True,
    ).stdout.split()
    return [
        f for f in out
        # `packages/` and `tools/` carry vendored and generated shell too; only
        # the trees this repo authors as checks/build glue are in scope.
        if (f.endswith(SUFFIXES) or f == "justfile")
        and "/third-party/" not in f
        and "/generated/" not in f
        and f not in SELF_REFERENTIAL
    ]


def count(rel):
    path = os.path.join(ROOT, rel)
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            lines = fh.readlines()
    except OSError:
        return 0
    n = 0
    for line in lines:
        stripped = line.lstrip()
        if stripped.startswith("#"):
            continue          # prose about the rule is not a violation
        if "nros_grep_q" in line:
            continue          # the fix
        if COND.search(line):
            n += 1
    return n


def scan():
    me = os.path.relpath(os.path.abspath(__file__), ROOT)
    return {f: c for f in tracked() if f != me and (c := count(f))}


HELPER = os.path.join(ROOT, "scripts", "lib", "grep-q.sh")

# Every case runs the REAL helper in a real bash. A pure-regex selftest can only
# show this gate still recognises the bad shape; it cannot show the sanctioned
# replacement still behaves, and the replacement is the half that 46 converted
# sites now depend on. The `-i`/`-E`/`-F`/`--` rows exist because the conversion
# needed flag passthrough: if `nros_grep_q` ever drops the flags on the floor,
# `grep -qi X` becomes a case-SENSITIVE search that quietly stops matching, and
# a gate reading "if ! … then FAIL" would report a confident, specific, wrong
# finding — issue 0726's exact failure mode arriving through the fix for it.
#
# The expected outcome is a STATUS *and* whether the helper returned or exited.
# Those are not the same thing and the difference is the entire contract: on a
# tool failure the helper must END THE SCRIPT, and a version that merely
# `return`s 2 looks identical to `case $?` at the call site while letting the
# caller sail on and verdict from evidence it never got. So every case runs a
# marker statement after the call and the FATAL rows require it to be absent.
RETURNS, FATAL = "returns", "fatal"
MARKER = "__nros_reached_next_statement__"

# (argv after the helper name, stdin, expected status, RETURNS|FATAL)
HELPER_CASES = [
    # The pre-existing no-flag contract, unchanged.
    (["Hello"], "Hello\n", 0, RETURNS),
    (["Nope"], "Hello\n", 1, RETURNS),
    # -i is passed through: without it the same pattern must NOT match.
    (["-i", "hello"], "Hello\n", 0, RETURNS),
    (["hello"], "Hello\n", 1, RETURNS),
    # -E is passed through: `x+` is a repetition in ERE, a literal `+` in BRE.
    (["-E", "x+y"], "xxy\n", 0, RETURNS),
    (["x+y"], "xxy\n", 1, RETURNS),
    # -F is passed through: `x+y` matches only as a literal.
    (["-F", "x+y"], "x+y\n", 0, RETURNS),
    (["-F", "x+y"], "xxy\n", 1, RETURNS),
    # A bundle keeping the original `-q` spelling still parses.
    (["-qiE", "h(e)l+o"], "HELLO\n", 0, RETURNS),
    # `--` ends the flags, so a pattern starting with `-` is a PATTERN. Without
    # passthrough of `--`, `-v` would invert the match and this would be 1.
    (["-F", "--", "-v"], "-v\n", 0, RETURNS),
    # A flag taking a separate operand is refused rather than guessed at: a
    # permissive parser would search for `1` inside the file named `Hello`.
    (["-m", "1", "Hello"], "Hello\n", 2, FATAL),
    (["--colour=always", "Hello"], "Hello\n", 2, FATAL),
    # A missing file is a TOOL failure, with flags exactly as without them —
    # and it must be FATAL, not a returned 2 the caller can walk past.
    (["Hello", "/nonexistent/nros"], "", 2, FATAL),
    (["-iE", "Hello", "/nonexistent/nros"], "", 2, FATAL),
]

# The `git grep` sibling: same three statuses over `git grep`'s own 0/1/128.
# `-E` vs BRE is the discriminator that a dropped flag cannot fake — the
# alternation is literal text under BRE, so it matches nothing.
GIT_CASES = [
    (["-E", "nros_(grep|git)_q", "--", "scripts/lib/grep-q.sh"], 0, RETURNS),
    (["nros_(grep|git)_q", "--", "scripts/lib/grep-q.sh"], 1, RETURNS),
    (["-i", "NROS_GREP_Q", "--", "scripts/lib/grep-q.sh"], 0, RETURNS),
    (["NROS_GREP_Q", "--", "scripts/lib/grep-q.sh"], 1, RETURNS),
    # `--` here separates the PATHSPECS, so it must survive to git untouched.
    (["-E", "nros_(grep|git)_q", "--", "scripts/lib/tracked.py"], 1, RETURNS),
    (["-E", "x", "--", "/nonexistent/outside/repo"], 2, FATAL),
]


# The flag allowlist is CLOSED, and end-to-end cases cannot show that: feed
# `-m 1 Hello` to a permissive parser and grep itself errors out, so the helper
# still exits 2 and the case passes for the wrong reason. These probe the
# predicate directly, which is the only place the closure is decidable.
FLAG_ACCEPT = ["-i", "-E", "-F", "-v", "-qiE", "-w", "-x", "-s",
               "--ignore-case", "--fixed-strings"]
FLAG_REJECT = ["-e", "-f", "-m", "-A", "-B", "-C", "-d", "-D", "-eE",
               "--include=x", "--colour=always", "--max-count=1", "-", ""]


def flag_allowlist_test():
    fails = []
    script = ('set -uo pipefail\n. "$1"\n'
              'if _nros_grep_q_flag_ok "$2"; then echo ok; else echo no; fi\n')
    for flag, want in ([(f, "ok") for f in FLAG_ACCEPT]
                       + [(f, "no") for f in FLAG_REJECT]):
        got = subprocess.run(
            ["bash", "-c", script, "_", HELPER, flag],
            capture_output=True, text=True, check=False,
        ).stdout.strip()
        if got != want:
            fails.append(f"_nros_grep_q_flag_ok {flag!r}: want {want}, got {got!r}"
                         f" — an operand-taking flag that slips through makes the"
                         f" NEXT argument the pattern, silently")
    return fails


def _run_helper(fn, argv, stdin):
    """(status, reached_next_statement)."""
    script = (f'set -uo pipefail\n. "$1"\nshift\n{fn} "$@"\n'
              f'rc=$?\nprintf "%s" "{MARKER}"\nexit "$rc"\n')
    p = subprocess.run(
        ["bash", "-c", script, "_", HELPER, *argv],
        input=stdin, cwd=ROOT, capture_output=True, text=True, check=False,
    )
    return p.returncode, MARKER in p.stdout


def helper_self_test():
    fails = []
    for fn, cases in (("nros_grep_q", [(a, s, w, k) for a, s, w, k in HELPER_CASES]),
                      ("nros_git_grep_q", [(a, "", w, k) for a, w, k in GIT_CASES])):
        for argv, stdin, want, kind in cases:
            got, reached = _run_helper(fn, argv, stdin)
            where = f"{fn} {' '.join(argv)!r}"
            if got != want:
                fails.append(f"{where}: want status {want}, got {got}")
            elif reached != (kind is RETURNS):
                fails.append(
                    f"{where}: want it to {kind}, but the caller "
                    f"{'CONTINUED' if reached else 'stopped'} after the call — a "
                    f"tool failure must END the script, not hand back a status")
    return fails + flag_allowlist_test()


def self_test():
    """Both directions — a checker that stopped checking passes silently."""
    good = [
        "grep -q foo bar.txt; rc=$?",
        "nros_grep_q \"$pat\" \"$f\"",
        "# if ! grep -q foo; then   (prose)",
    ]
    bad = [
        'if ! printf "%s" "$t" | grep -q "$pat"; then',
        "if grep -q foo bar; then",
        "grep -q foo bar || fail=1",
        "grep -q foo bar && continue",
    ]
    fails = []
    for s in good:
        st = s.lstrip()
        if not st.startswith("#") and "nros_grep_q" not in s and COND.search(s):
            fails.append(f"false positive: {s}")
    for s in bad:
        if not COND.search(s):
            fails.append(f"MISSED: {s}")
    helper_fails = helper_self_test()
    fails += helper_fails
    if fails:
        print("check-grep-q-error-conflation --self-test FAILED:")
        print("\n".join("  " + f for f in fails))
        return 1
    n_helper = (len(HELPER_CASES) + len(GIT_CASES)
                + len(FLAG_ACCEPT) + len(FLAG_REJECT))
    print(f"check-grep-q-error-conflation --self-test: "
          f"{len(good) + len(bad)} regex case(s) + {n_helper} live "
          f"nros_grep_q/nros_git_grep_q case(s) OK")
    return 0


def main():
    if "--self-test" in sys.argv:
        return self_test()
    if self_test():
        return 1

    current = scan()
    if "--write-baseline" in sys.argv:
        with open(BASELINE, "w", encoding="utf-8") as fh:
            json.dump(current, fh, indent=2, sort_keys=True)
            fh.write("\n")
        print(f"wrote baseline: {sum(current.values())} site(s) "
              f"across {len(current)} file(s)")
        return 0

    try:
        with open(BASELINE, encoding="utf-8") as fh:
            base = json.load(fh)
    except OSError:
        print(f"FAIL: missing baseline {BASELINE}; "
              f"regenerate with --write-baseline", file=sys.stderr)
        return 1

    grew = []
    for f, n in sorted(current.items()):
        was = base.get(f, 0)
        if n > was:
            grew.append((f, was, n))
    if grew:
        print("FAIL: new `grep -q` conditional(s) that cannot distinguish a")
        print("      tool ERROR (exit >=2) from a NON-MATCH (exit 1):")
        for f, was, n in grew:
            print(f"  {f}: {was} -> {n}")
        print()
        print("  Source scripts/lib/grep-q.sh and use `nros_grep_q`:")
        print("    nros_grep_q \"$pat\" \"$file\"; case $? in 0) ;; 1) finding ;; esac")
        print("  It exits 2 on a tool failure instead of reporting a finding.")
        print()
        print("  Issue 0726: a forked grep that failed to start under a 32-way")
        print("  fan-out was reported as a missing force-link anchor — a false,")
        print("  specific claim, and only ever under load.")
        return 1

    total = sum(current.values())
    shrank = sum(1 for f, n in current.items() if n < base.get(f, 0))
    gone = sum(1 for f in base if f not in current)
    msg = (f"check-grep-q-error-conflation: OK ({total} baselined site(s), "
           f"no file grew one)")
    if shrank or gone:
        msg += f" — {shrank + gone} file(s) improved; rerun --write-baseline"
    print(msg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
