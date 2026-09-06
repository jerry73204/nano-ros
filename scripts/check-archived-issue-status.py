#!/usr/bin/env python3
"""An issue in `archived/` must say it is resolved. That is what archiving MEANS.

Issue 0937's shape, and the reason this is a gate rather than a convention:

    4f1305788   R100  docs/issues/0937-*.md -> docs/issues/archived/0937-*.md

`R100` is a 100% similarity rename -- the archiving commit moved the file with
ZERO content change. The author had rewritten the diagnosis in their working
tree, a `git checkout origin/main -- <file>` reverted it, and the archiving
commit then moved the ORIGINAL text while the commit message and the pull
request both described the correction. The file sat in `archived/` for a day
saying `status: open` and blaming a cause that had been disproved, which is
worse than no record: it aims the next person at a dead end.

`c7f2b09a0` restored it, +81/-72, and its message names the mechanism.

WHY THIS RULE AND NOT "THE ARCHIVE MOVE MUST NOT DELETE TEXT"

Measured before choosing. Of 668 archive renames in history, 127 -- 19% --
delete body text, and most are legitimate: an archiving commit routinely
rewrites an `Options` list into a `Resolution`. A no-deletions rule would fire
on a fifth of every archiving commit and teach people to ignore it.

The revert-detection rule was measured too ("the archived content equals an
older committed version"): zero hits across all 668, including 0937 itself,
at a cost of 27 seconds. It does not work, because the reverted text was the
FIRST version and there was no older one to match.

What 0937 actually leaves behind is a file in `archived/` whose frontmatter
still says `open` -- because an `R100` rename cannot have changed it. That is
one cheap tree-state read, it fires precisely on the damage, and it is what
this checks.

`resolved_in:` is deliberately NOT required: 648 of 991 archived issues lack
it, so requiring it would be a 648-entry ratchet that says nothing about
whether the record is honest.

Run:  python3 scripts/check-archived-issue-status.py [--self-test]
"""

import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARCHIVED = os.path.join(ROOT, "docs", "issues", "archived")

OK_STATUS = ("resolved", "wontfix")

# Files archived before this gate existed whose status nobody has ruled on.
# A ratchet: it may only SHRINK, and each entry says what is unresolved about
# it. Not a general exemption -- a new archived issue cannot join this list
# without someone editing it, which is the deliberate edit the gate is for.
BASELINE = {
    # EMPTY, and that is the intended end state. It held
    # `0203-mixed-workspace-cpp-interface-overgeneration.md` for exactly one
    # commit: the gate found it, `e2c413582` turned out to have closed it back
    # on 2026-07-16 without flipping `status:`, and the file now says so. An
    # entry here means "archived, and nobody has ruled on whether it is
    # resolved" -- a state to remove, not to keep.
    #
    # The list may only SHRINK: a stale entry fails this gate, which is what
    # made the 0203 fix land rather than sit behind an exemption.
}

FRONTMATTER = re.compile(r"\A---\n(.*?)\n---\n", re.S)
STATUS = re.compile(r"^status:\s*(\S+)", re.M)


def verdict(name, text, baseline=None):
    """None if fine, else the reason it is not."""
    if name in (BASELINE if baseline is None else baseline):
        return None
    m = FRONTMATTER.match(text)
    if not m:
        return "no YAML frontmatter, so nothing states its status"
    st = STATUS.search(m.group(1))
    if not st:
        return "frontmatter has no `status:` field"
    if st.group(1) not in OK_STATUS:
        return f"frontmatter says `status: {st.group(1)}` — an archived issue is not open"
    return None


def self_test():
    good = "---\nid: 1\nstatus: resolved\n---\n\nbody\n"
    wont = "---\nid: 1\nstatus: wontfix\n---\n\nbody\n"
    open_ = "---\nid: 1\nstatus: open\n---\n\nbody\n"
    nofm = "# 0364 — a title\n\n**Status:** Resolved\n"
    nost = "---\nid: 1\ntype: bug\n---\n\nbody\n"
    cases = [
        ("x.md", good, None),
        ("x.md", wont, None),
        ("x.md", open_, "not open"),
        ("x.md", nofm, "frontmatter"),
        ("x.md", nost, "status"),
        # A baselined file passes whatever it says — that is what a ratchet
        # is. Driven through a synthetic entry, because BASELINE is empty now
        # and a self-test that skips when the list is empty proves nothing
        # about the mechanism it is there to cover.
        ("_synthetic_baselined_.md", open_, None),
    ]
    bad = 0
    for name, text, want in cases:
        got = verdict(name, text, {"_synthetic_baselined_.md": "self-test"})
        if (want is None) != (got is None) or (want and want not in got):
            print(f"  self-test FAIL: {name} {text!r} -> {got!r}, want {want!r}")
            bad += 1
    if bad:
        print(f"check-archived-issue-status self-test: {bad} case(s) FAILED")
        return 1
    print(f"check-archived-issue-status self-test: OK ({len(cases)} cases)")
    return 0


def main():
    if "--self-test" in sys.argv:
        return self_test()
    if self_test() != 0:
        return 1

    files = sorted(glob.glob(os.path.join(ARCHIVED, "[0-9]*.md")))
    if not files:
        sys.stderr.write(
            "check-archived-issue-status: found NO archived issues under %s — "
            "this gate would pass vacuously.\n" % os.path.relpath(ARCHIVED, ROOT)
        )
        return 1

    problems = []
    for path in files:
        name = os.path.basename(path)
        with open(path, encoding="utf8", errors="replace") as fh:
            why = verdict(name, fh.read())
        if why:
            problems.append((name, why))

    # The baseline may only shrink: an entry that no longer needs it is a stale
    # exemption, and a stale exemption is how a ratchet stops ratcheting.
    stale = [
        n for n in BASELINE
        if not os.path.isfile(os.path.join(ARCHIVED, n))
        or verdict("_probe_", open(os.path.join(ARCHIVED, n), encoding="utf8").read()) is None
    ]
    if stale:
        sys.stderr.write(
            "check-archived-issue-status: %d baseline entry/entries no longer "
            "need it — remove them:\n" % len(stale)
        )
        for n in stale:
            sys.stderr.write(f"  {n}\n")
        return 1

    if problems:
        sys.stderr.write(
            "check-archived-issue-status: %d archived issue(s) do not say they "
            "are resolved:\n\n" % len(problems)
        )
        for name, why in problems:
            sys.stderr.write(f"  {name}\n      {why}\n")
        sys.stderr.write(
            "\nArchiving is the claim that an issue is DONE. A file in "
            "`archived/` that still reads `open` is the issue-0937 shape: an\n"
            "`R100` rename moved the record unchanged, so the correction the "
            "commit message described never reached the file.\n"
            "Set `status: resolved` (or `wontfix`) as part of the move, or "
            "leave the file in `docs/issues/`.\n"
        )
        return 1

    print(
        "check-archived-issue-status: OK — %d archived issue(s) say they are "
        "resolved (%d baselined)." % (len(files) - len(BASELINE), len(BASELINE))
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
