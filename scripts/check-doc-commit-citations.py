#!/usr/bin/env python3
"""A live document may not cite a commit that does not exist.

The sibling of `check-ledger-orphan-refs`, one layer out. That gate refuses a
LEDGER row that cites a file which is not there; this refuses a DOCUMENT that
cites a commit which is not there. Both answer the same question — does the
evidence a claim rests on still exist — and until now the prose half was
unchecked while the structured half was gated.

Found the ordinary way: phase-428's sweep doc cited eight commits by short
hash. Two resolved. Six named nothing at all, because five of them were on the
branch the document lives on and that branch had been rebased twice. A rebase
mints new objects; nobody re-reads the prose that cited the old ones. The
document containing those six is the same one whose own CORRECTION section is
about "a claim believed because it was AUTHORED rather than OBSERVED".

WHAT IS CHECKED, AND WHY THIS SHAPE

Measured across 2 272 tracked markdown files before choosing any rule — 1 446
distinct backticked hex strings, 2 284 occurrences.

*Width.* This repo cites commits at NINE hex characters, and that is the only
width where the population is mostly commits:

    len |  resolves | dangling
      7 |        15 |      168
      8 |       111 |      156
      9 |       748 |      172      <- the convention
     16 |         0 |       40      <- cargo `-C metadata` identities
     32 |         0 |        3
     40 |         0 |        7

Seven and eight characters are dominated by hex that was never a hash. Sixteen
never resolves because it is not a commit at all: those are cargo unit
identities (`ef7a4cf9ec5986e9`), which phase-334 and phase-340 quote by the
dozen. Checking those widths would report far more noise than defect, so the
gate reads exactly nine and says so rather than pretending to a coverage it
does not have.

*Live documents only.* Of 186 dangling occurrences, **178 are under
`archived/`**. An archived phase or issue is a record of what happened; its
citations were true when written, and rewriting them edits history to make a
gate green. The eight that remain are in documents people still act on, which
is where a pointer to nothing actually misleads someone.

*An exemption file, because "does not resolve here" is not the same as
"wrong".* Seven of the live citations name commits in OTHER repositories — two
zenoh core commits in RFC-0082, three `ros2` distrobox tree states in issues
1127/1137, a submodule pointer and a parallel agent's divergent `main` in
phase-311. Those are correct sentences about foreign history, and no amount of
looking will resolve them in this checkout. They are declared, with their
reason, in the baseline.

THE BASELINE IS A RATCHET

`.config/doc-commit-citations-baseline.txt` may only SHRINK. A new dangling
citation fails, and so does an entry that no longer offends — a stale exemption
is how a ratchet stops ratcheting. Each entry carries the reason it is there,
because "foreign repo, correctly cited" and "in-repo, genuinely dead" are
different states and only the second is debt.

SHALLOW CLONES

`git cat-file` can only answer for objects the clone has. A shallow checkout
would report a live commit as dangling, so the gate SKIPS there rather than
failing: an answer that depends on clone depth is worse than no answer.

Run:  python3 scripts/check-doc-commit-citations.py [--self-test]
"""

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASELINE = os.path.join(ROOT, ".config", "doc-commit-citations-baseline.txt")

# Nine hex characters between backticks. See the width table above for why not
# seven, eight or sixteen.
CITATION = re.compile(r"`([0-9a-f]{9})`")


def tracked_markdown():
    """Live tracked markdown — `archived/` excluded, for the reason above."""
    out = subprocess.run(
        ["git", "-C", ROOT, "ls-files", "*.md"],
        capture_output=True, text=True, check=True).stdout.split("\n")
    return [f for f in out if f.endswith(".md") and "/archived/" not in f]


def citations(files):
    """{hash: [(path, lineno)]} for every 9-hex citation in `files`."""
    found = {}
    for rel in files:
        path = os.path.join(ROOT, rel)
        try:
            with open(path, encoding="utf8", errors="replace") as fh:
                lines = fh.read().split("\n")
        except OSError:
            continue
        for n, line in enumerate(lines, 1):
            for m in CITATION.finditer(line):
                found.setdefault(m.group(1), []).append((rel, n))
    return found


def resolve(hashes):
    """The subset of `hashes` that names a commit in this repository.

    One `cat-file --batch-check` for the whole set rather than a process per
    hash: 1 446 of them is 1 446 forks otherwise, and this gate is on the fast
    line.
    """
    if not hashes:
        return set()
    query = "\n".join(f"{h}^{{commit}}" for h in hashes)
    r = subprocess.run(["git", "-C", ROOT, "cat-file", "--batch-check"],
                       input=query, capture_output=True, text=True)
    ok = set()
    for h, line in zip(hashes, r.stdout.strip().split("\n")):
        if " commit " in line:
            ok.add(h)
    return ok


def read_baseline(path=None):
    entries = {}
    try:
        with open(path or BASELINE, encoding="utf8") as fh:
            for line in fh:
                line = line.split("#", 1)[0].strip()
                if line:
                    entries[line] = True
    except FileNotFoundError:
        pass
    return entries


def is_shallow():
    r = subprocess.run(["git", "-C", ROOT, "rev-parse", "--is-shallow-repository"],
                       capture_output=True, text=True)
    return r.stdout.strip() == "true"


def main():
    if "--self-test" in sys.argv:
        return self_test()
    if self_test() != 0:
        return 1

    if is_shallow():
        print("[SKIPPED] doc-commit-citations: shallow clone — `git cat-file` "
              "cannot tell a dead commit from an unfetched one "
              "(git fetch --unshallow)")
        return 0

    files = tracked_markdown()
    found = citations(files)
    if not found:
        sys.stderr.write(
            "check-doc-commit-citations: found NO commit citations in %d live "
            "markdown file(s) — the pattern or the file list is wrong, and this "
            "gate would pass vacuously.\n" % len(files))
        return 1

    hashes = sorted(found)
    live = resolve(hashes)
    baseline = read_baseline()
    dangling = [h for h in hashes if h not in live]

    problems = [h for h in dangling if h not in baseline]
    stale = [h for h in baseline if h not in dangling]

    if stale:
        sys.stderr.write(
            "check-doc-commit-citations: %d baseline entry/entries no longer "
            "needed — the citation resolves now, or the line is gone. Remove "
            "them:\n" % len(stale))
        for h in sorted(stale):
            sys.stderr.write(f"  {h}\n")
        return 1

    if problems:
        sys.stderr.write(
            "check-doc-commit-citations: %d citation(s) name a commit this "
            "repository does not have:\n\n" % len(problems))
        for h in sorted(problems):
            for rel, n in found[h]:
                sys.stderr.write(f"  {rel}:{n}   `{h}`\n")
        sys.stderr.write(
            "\nA short hash is invalidated by any rebase of the branch it names, "
            "and prose is not re-read when that happens.\n"
            "Cite something that survives instead — a PR or issue number, or a "
            "description of the commit — or, if the commit is\n"
            "genuinely elsewhere (a submodule, an upstream project, another "
            "checkout), add it to\n"
            "%s with the reason.\n"
            % os.path.relpath(BASELINE, ROOT))
        return 1

    print("check-doc-commit-citations: OK — %d citation(s) across %d live "
          "file(s) resolve (%d declared foreign/known-dead)."
          % (len(live), len(files), len(baseline)))
    return 0


def self_test():
    """Prove the gate can fail, on synthetic input rather than on the tree."""
    import tempfile

    real = subprocess.run(["git", "-C", ROOT, "rev-parse", "--short=9", "HEAD"],
                          capture_output=True, text=True).stdout.strip()
    dead = "0123456789"[:9]          # not a commit, and never will be
    cases = [
        ("a resolving citation passes", f"see `{real}` for it\n", False),
        ("a dangling citation fails", f"see `{dead}` for it\n", True),
        ("a 16-char cargo identity is ignored",
         "unit `ef7a4cf9ec5986e9` has nine copies\n", False),
        ("an 8-char hex is ignored (below the convention)",
         "see `0123abcd` there\n", False),
        ("bare hex outside backticks is ignored",
         f"see {dead} for it\n", False),
    ]
    bad = 0
    with tempfile.TemporaryDirectory() as d:
        for name, text, want_fail in cases:
            p = os.path.join(d, "doc.md")
            with open(p, "w") as fh:
                fh.write(text)
            found = {}
            with open(p, encoding="utf8") as fh:
                for n, line in enumerate(fh.read().split("\n"), 1):
                    for m in CITATION.finditer(line):
                        found.setdefault(m.group(1), []).append(("doc.md", n))
            got_fail = bool([h for h in found if h not in resolve(sorted(found))])
            if got_fail != want_fail:
                print(f"  self-test FAIL: {name} -> fails={got_fail}, "
                      f"want {want_fail}")
                bad += 1

    # The ratchet's own control: a stale entry must be reported, or the
    # baseline silently becomes an allowlist.
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "baseline.txt")
        with open(p, "w") as fh:
            fh.write("# comment\n" + dead + "  # a reason\n\n")
        entries = read_baseline(p)
        if set(entries) != {dead}:
            print(f"  self-test FAIL: baseline parse -> {sorted(entries)}")
            bad += 1

    if bad:
        print(f"check-doc-commit-citations self-test: {bad} case(s) FAILED")
        return 1
    print("check-doc-commit-citations self-test: OK (%d cases + baseline parse)"
          % len(cases))
    return 0


if __name__ == "__main__":
    sys.exit(main())
