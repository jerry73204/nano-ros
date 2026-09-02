#!/usr/bin/env python3
"""Issue 0989 — a prose reference to an issue id must resolve to a file.

WHY `check-doc-refs` DOES NOT COVER THIS

That gate checks PATHS: `docs/issues/NNNN-slug.md` written into prose, cmake
strings, frontmatter. It was built for a renumbered RFC whose stale path sat in
a cmake error message. It is the right check for a path, and it cannot see the
form people actually write in prose:

    (issues 0883/0884)          → issues 0632, 0640.          see issue 0940

No path, no link, nothing for a path-checker to resolve. Measured 2026-09-02:
`check-doc-refs` was GREEN while `CLAUDE.md` cited three ids with no file on
main — 0883 (written, never merged), 0632 and 0640 (no file on any branch,
ever). The two gates are complements, not duplicates.

WHY IT MATTERS MORE THAN A BROKEN LINK

CLAUDE.md is loaded into every agent session. A pointer there is an instruction
to go read something, and when the something does not exist the reader cannot
tell whether they are looking at a lost document or their own bad search. The
0883 case was the expensive shape: the analysis existed, was never merged, and
the fix landed under a different id — so the tree said "see 0883" for four days
about a file that had never been on main.

WHAT COUNTS AS A REFERENCE

A 4-digit id preceded by `issue`/`issues`, optionally `#`-prefixed and
hyphen- or space-separated. Deliberately narrow: bare 4-digit numbers appear
everywhere (ports, sizes, years, sha prefixes) and matching them would make this
gate noise. `issue-0196` and `issues 0883/0884` are the spellings in this tree.

WHAT IS EXEMPT, AND WHY

  * `docs/issues/archived/**` — an archived issue legitimately cites its
    contemporaries, some of which may since have been renumbered or dropped.
    Rewriting history to satisfy a gate is worse than the dangling ref.
  * A `related:` / `resolved_in:` frontmatter list — those are already the
    business of `check-issue-ids`, and duplicating that check here would give
    two error messages for one defect.
  * An id whose file is ADDED IN THE WORKING TREE but not yet committed: the
    gate reads the filesystem, so a new issue and the doc that cites it can land
    in one commit. That is the normal flow and must not be blocked.

The last one is the reason this reads the filesystem rather than `git ls-files`:
a gate that forced you to commit the issue before you could cite it would push
people to cite nothing.

IN-FLIGHT REFERENCES ARE THE HARD CASE

An id can be cited correctly by a doc whose issue file is in ANOTHER open PR —
`book/src/getting-started/zephyr.md` cites 0940, whose file is in PR #151. On
that branch the gate is green; on main it would be red, and the "fix" (deleting
a correct citation) is wrong.

So violations are a RATCHET against a tracked baseline, not a hard zero. A new
dangling reference fails; the known in-flight ones are listed and disappear from
the baseline when their issue lands. Same shape as
`.config/gate-selftest-baseline.txt`.
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASELINE = os.path.join(ROOT, ".config", "prose-issue-ref-baseline.txt")

# `issue 0883`, `issues 0883/0884`, `issue-0196`, `issue #0883`
REF = re.compile(r"issues?[-\s]#?(\d{4})(?!-\d)\b", re.I)
# A second id after a separator: `issues 0883/0884`, `issues 0632, 0640`
# `(?!-\d)` rejects a DATE. `issue 0364, 2026-07-31` is one id and a date, not
# a pair — without this the year is read as a second issue and the gate invents
# a dangling reference to issue 2026. Found by running the audit on this tree.
TAIL = re.compile(r"\s*[/,]\s*#?(\d{4})(?!-\d)\b")

SEARCH_ROOTS = ("docs", "book/src", "cmake", "scripts", "just")
SEARCH_FILES = ("CLAUDE.md", "AGENTS.md", "README.md")
EXEMPT_DIRS = (os.path.join("docs", "issues", "archived"),)
# This gate's own source. Its docstring must name the ids that motivated it,
# and a gate that flags itself for documenting its own cases is unusable —
# `check-doc-refs` records hitting exactly this and solved it by paraphrasing;
# an explicit exemption is clearer than prose contorted to avoid a regex.
EXEMPT_FILES = ("scripts/check-prose-issue-refs.py",)


def existing_ids():
    """Every issue id with a file on disk, open or archived."""
    ids = set()
    for d in (os.path.join(ROOT, "docs", "issues"),
              os.path.join(ROOT, "docs", "issues", "archived")):
        if not os.path.isdir(d):
            continue
        for name in os.listdir(d):
            m = re.match(r"(\d{4})-", name)
            if m:
                ids.add(m.group(1))
    return ids


def candidate_files():
    out = []
    for f in SEARCH_FILES:
        p = os.path.join(ROOT, f)
        if os.path.isfile(p):
            out.append(f)
    for root in SEARCH_ROOTS:
        base = os.path.join(ROOT, root)
        for dirpath, dirnames, filenames in os.walk(base):
            rel = os.path.relpath(dirpath, ROOT)
            if any(rel == e or rel.startswith(e + os.sep) for e in EXEMPT_DIRS):
                dirnames[:] = []
                continue
            dirnames[:] = [d for d in dirnames if d != ".git"]
            for name in filenames:
                if name.endswith((".md", ".sh", ".py", ".cmake", ".just", ".txt")):
                    out.append(os.path.relpath(os.path.join(dirpath, name), ROOT))
    return sorted(set(out))


def refs_in(text):
    """Every id referenced, including the tail of `issues A/B` and `A, B`."""
    found = set()
    for m in REF.finditer(text):
        found.add(m.group(1))
        pos = m.end()
        while True:
            t = TAIL.match(text, pos)
            if not t:
                break
            found.add(t.group(1))
            pos = t.end()
    return found


def scan():
    """{ 'file:id' } for every reference with no file on disk."""
    have = existing_ids()
    bad = set()
    for rel in candidate_files():
        if rel in EXEMPT_FILES:
            continue
        path = os.path.join(ROOT, rel)
        try:
            with open(path, encoding="utf8", errors="replace") as fh:
                text = fh.read()
        except OSError:
            continue
        # Frontmatter `related:` / `resolved_in:` belong to check-issue-ids.
        text = re.sub(r"^(related|resolved_in):.*$", "", text, flags=re.M)
        for n in refs_in(text):
            if n not in have:
                bad.add(f"{rel}:{n}")
    return bad


def load_baseline():
    if not os.path.exists(BASELINE):
        raise SystemExit(
            f"check-prose-issue-refs: baseline missing at {BASELINE}.\n"
            "  It is tracked, so its absence is a PATH bug, not an empty ratchet.\n"
            "  Regenerate deliberately with --write-baseline."
        )
    with open(BASELINE, encoding="utf8") as fh:
        return {l.strip() for l in fh if l.strip() and not l.startswith("#")}


def write_baseline(bad):
    os.makedirs(os.path.dirname(BASELINE), exist_ok=True)
    with open(BASELINE, "w", encoding="utf8") as fh:
        fh.write(
            "# Prose references to an issue id with no file on disk.\n"
            "#\n"
            "# A RATCHET, not an allowlist: this file may only shrink. Each row is\n"
            "# `<path>:<id>`. The legitimate reason to be here is an IN-FLIGHT issue —\n"
            "# the citing doc is on main, its issue file is in another open PR — where\n"
            "# deleting the correct citation would be the wrong fix.\n"
            "#\n"
            "# Regenerate: python3 scripts/check-prose-issue-refs.py --write-baseline\n"
        )
        for row in sorted(bad):
            fh.write(row + "\n")
    print(f"wrote {BASELINE} — {len(bad)} known dangling reference(s)")


def self_test():
    """The gate's own failure path, run on the normal path (phase-395)."""
    fails = 0

    def case(name, text, want):
        nonlocal fails
        got = refs_in(text)
        if got != want:
            print(f"  [FAIL] {name}: got {sorted(got)}, want {sorted(want)}", file=sys.stderr)
            fails += 1

    case("bare", "see issue 0883 for detail", {"0883"})
    case("slashed pair", "(issues 0883/0884) — the ledger", {"0883", "0884"})
    case("comma pair", "→ issues 0632, 0640.", {"0632", "0640"})
    case("hyphenated", "the issue-0196 rule", {"0196"})
    case("hash", "issue #0975 deadlocked the queue", {"0975"})
    # A bare number is NOT a reference — this is what keeps the gate quiet.
    case("bare number ignored", "port 7447 and 0640 bytes", set())
    case("word boundary", "issue 09755 is not an id", set())

    if fails:
        print(f"check-prose-issue-refs: self-test FAILED ({fails} case(s))", file=sys.stderr)
        return False
    return True


def main():
    if not self_test():
        return 1

    bad = scan()

    if "--write-baseline" in sys.argv:
        write_baseline(bad)
        return 0

    if "--audit" in sys.argv:
        print(f"{len(bad)} dangling prose reference(s):")
        for row in sorted(bad):
            print(f"  {row}")
        return 0

    baseline = load_baseline()
    new = bad - baseline
    fixed = baseline - bad

    if new:
        print("check-prose-issue-refs: FAIL — reference(s) to an issue with no file:", file=sys.stderr)
        for row in sorted(new):
            path, n = row.rsplit(":", 1)
            print(f"    {path} cites issue {n}, and docs/issues/{n}-*.md does not exist", file=sys.stderr)
        print(file=sys.stderr)
        print("  Either the issue was never filed, or its file is in another open PR.", file=sys.stderr)
        print("  If it is in flight, add the row to the baseline and say which PR:", file=sys.stderr)
        print("    python3 scripts/check-prose-issue-refs.py --write-baseline", file=sys.stderr)
        print("  Otherwise fix the citation — or file the issue it points at.", file=sys.stderr)
        return 1

    if fixed:
        print("check-prose-issue-refs: FAIL — the baseline is STALE.", file=sys.stderr)
        print("  These no longer dangle (their issue landed, or the citation went):", file=sys.stderr)
        for row in sorted(fixed):
            print(f"    {row}", file=sys.stderr)
        print("  A ratchet that does not tighten stops being one. Regenerate:", file=sys.stderr)
        print("    python3 scripts/check-prose-issue-refs.py --write-baseline", file=sys.stderr)
        return 1

    print(f"check-prose-issue-refs: OK ({len(baseline)} known in-flight, no new dangling reference)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
