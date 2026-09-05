#!/usr/bin/env python3
"""A markdown LINK must resolve exactly. A prose mention may resolve loosely.

WHY THIS EXISTS (issue 1085). `check-doc-refs` already guards every
`docs/{design,issues}/NNNN-*.md` path written anywhere in the tree, and it
deliberately accepts a reference whose basename is at the named path OR in that
series' `archived/` directory. Its own header says why: archiving a resolved
issue MOVES it, and requiring every prose mention to be retargeted on archive
would be churn against a hundred documents that are still telling the truth.

That is the right rule for prose, and for a cmake error string. It is the wrong
rule for a MARKDOWN LINK, because a link is a thing a reader CLICKS. When
`docs/roadmap/phase-424-*.md` said

    | [#0835](../issues/0835-fixture-staleness-probe-families-restale-each-other.md) |

and the file had moved to `../issues/archived/…`, `check-doc-refs` was green and
the link was a 404 on GitHub. Swept 2026-09-05: **160 dead links across the live
docs**, of which 123 were exactly this — a live document pointing at where an
issue, phase or RFC used to be.

So the two gates split by what the reference IS, not by where it points:

  * `check-doc-refs`  — a PATH MENTIONED anywhere must name a document that
    exists somewhere in its series. Loose on purpose.
  * this gate         — a `[text](target)` LINK must resolve to the file or
    directory it actually names. Exact on purpose.

SCOPE, and why each exclusion is here rather than in a list of special cases:

  * `book/` has `check-book-links`, which knows about mdBook's `SUMMARY.md` and
    the generated `api/` trees this gate would flag wrongly.
  * `**/archived/**` is a HISTORICAL RECORD. A link in an archived document
    pointing at where a file was when that document was written is not a defect
    to fix; retargeting 434 of them would be a large diff no reader benefits
    from. They are counted instead, as a ratchet that may only fall.
  * Code spans are stripped before matching. `T operator[](i)` in prose is C++,
    not a link, and a gate that cannot tell the difference reports a defect in
    a sentence about operator overloading.

Exit 0 when every live markdown link resolves, 1 otherwise.
"""

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# A tracked-but-absent target that is CORRECT for a reader. Each entry needs a
# reason, because an exemption list is how a gate stops being one.
EXEMPT = {
    # Generated and gitignored ON PURPOSE (issues 0883/0884): a committed open
    # list was the per-PR conflict site that serialised the merge queue, and
    # GitHub's server-side rebase runs no `.gitattributes` merge driver, so
    # untracking it was the only fix that reached the queue. It exists on any
    # tree that has run `python3 scripts/gen-issue-index.py`, which is what the
    # sentence linking it tells the reader to do.
    "docs/issues/open.md",
}

LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)\)")


def strip_code(text):
    """Remove code from the text, keeping line numbers intact.

    Three forms, and the third was found the hard way: this gate's OWN issue
    file quotes the broken link it is about, as a 4-space indented block —

            | [#0835](../issues/0835-….md) | fixtures | … |

    — and the first version of this function stripped fences and inline spans
    only, so it reported the illustration as a defect. A gate that cannot read
    its own documentation is not one anybody keeps.
    """
    # Fenced, ``` or ~~~.
    text = re.sub(r"^(```|~~~).*?^\1", lambda m: "\n" * m.group(0).count("\n"),
                  text, flags=re.S | re.M)
    # Indented: four spaces or a tab, but only where a block can START — after a
    # blank line — so an ordinary wrapped list item is not swallowed.
    out, prev_blank = [], True
    for line in text.split("\n"):
        if prev_blank and (line.startswith("    ") or line.startswith("\t")) and line.strip():
            out.append("")
        else:
            out.append(line)
            prev_blank = not line.strip()
    text = "\n".join(out)
    # Inline spans.
    return re.sub(r"`[^`\n]*`", "", text)


def tracked_paths(root):
    r = subprocess.run(["git", "-C", root, "ls-files"], capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f"check-markdown-links: `git ls-files` failed in {root}")
    return set(r.stdout.split())


def dead_links(root, files, tracked, dirs):
    out = []
    for f in files:
        try:
            with open(os.path.join(root, f), encoding="utf8") as fh:
                text = strip_code(fh.read())
        except OSError:
            continue
        for m in LINK.finditer(text):
            target = m.group(1).split("#")[0]
            if not target or target.startswith(("http://", "https://", "mailto:", "<")):
                continue
            p = os.path.normpath(os.path.join(os.path.dirname(f), target))
            if p in tracked or p in dirs or p in EXEMPT:
                continue
            line = text[:m.start()].count("\n") + 1
            out.append((f, line, target, p))
    return out


def partition(tracked):
    live, archived = [], []
    for f in sorted(tracked):
        if not f.endswith(".md") or f.startswith("book/"):
            continue
        (archived if "/archived/" in f else live).append(f)
    return live, archived


def baseline_path(root):
    return os.path.join(root, ".config", "archived-doc-dead-links.txt")


def read_baseline(root):
    try:
        with open(baseline_path(root), encoding="utf8") as fh:
            for line in fh:
                line = line.split("#")[0].strip()
                if line:
                    return int(line)
    except (OSError, ValueError):
        pass
    return None


def self_test(quiet=False):
    """Negative control: the rule must FIRE on the shape it exists for.

    Runs on the NORMAL path — `check-gate-selftests` holds this file to that,
    and a control nobody runs decays into a comment.
    """
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        os.makedirs(os.path.join(tmp, "docs", "issues", "archived"))
        os.makedirs(os.path.join(tmp, "docs", "roadmap"))
        moved = "docs/issues/archived/0835-a.md"
        open(os.path.join(tmp, moved), "w").write("x\n")
        page = "docs/roadmap/phase-1-b.md"

        def check(body, tracked_extra=()):
            with open(os.path.join(tmp, page), "w", encoding="utf8") as fh:
                fh.write(body)
            tracked = {moved, page} | set(tracked_extra)
            dirs = {os.path.dirname(p) for p in tracked}
            return dead_links(tmp, [page], tracked, dirs)

        # The exact 1085 shape: the file moved to archived/ and the link did not.
        got = check("see [#0835](../issues/0835-a.md)\n")
        assert len(got) == 1, f"the moved-to-archived link must fire; got {got}"

        # Retargeted — silent.
        got = check("see [#0835](../issues/archived/0835-a.md)\n")
        assert got == [], f"a resolving link must not fire; got {got}"

        # C++ in prose is not a link. This one matters: `T operator[](i)` is in
        # a live design doc and a gate that flags it is a gate people turn off.
        got = check("`T operator[](i)` does the same\n")
        assert got == [], f"an inline code span must not be parsed as a link; got {got}"

        # …and the same text inside a fence.
        got = check("```\nT operator[](i)\n```\n")
        assert got == [], f"a fenced block must not be parsed as a link; got {got}"

        # An INDENTED block is code too. This case is not hypothetical: issue
        # 1085's own file quotes the dead link it is about as an indented block,
        # and the first version of this gate flagged its own documentation.
        got = check("prose\n\n    [#0835](../issues/0835-a.md)\n\nmore prose\n")
        assert got == [], f"an indented code block must not be parsed as a link; got {got}"

        # But an indented line that is NOT after a blank line is a wrapped list
        # item, not a code block, and its link still counts.
        got = check("- a list item\n    [#0835](../issues/0835-a.md)\n")
        assert len(got) == 1, f"a wrapped list item is prose, not code; got {got}"

        # An external link is not this gate's business.
        got = check("[x](https://example.invalid/a.md)\n")
        assert got == [], f"an http link must not fire; got {got}"

        # A link to a tracked DIRECTORY resolves.
        got = check("[dir](../issues/archived)\n")
        assert got == [], f"a directory link must not fire; got {got}"

    if not quiet:
        print("check-markdown-links self-test: OK (8 case(s))")
    return 0


def main(argv):
    if "--self-test" in argv:
        return self_test()
    # Always, not only behind the flag. See `scripts/check-board-tiers.py`.
    rc = self_test(quiet=True)
    if rc:
        return rc

    tracked = tracked_paths(ROOT)
    dirs = {os.path.dirname(p) for p in tracked}
    live, archived = partition(tracked)

    bad = dead_links(ROOT, live, tracked, dirs)
    arch_bad = dead_links(ROOT, archived, tracked, dirs)

    if "--write-baseline" in argv:
        os.makedirs(os.path.dirname(baseline_path(ROOT)), exist_ok=True)
        with open(baseline_path(ROOT), "w", encoding="utf8") as fh:
            fh.write(
                "# Dead markdown links inside `**/archived/**`, which this gate\n"
                "# counts rather than fixes: an archived document pointing at where\n"
                "# a file was WHEN IT WAS WRITTEN is a historical record, not a\n"
                "# defect. A ratchet, so the number may only fall.\n"
                f"{len(arch_bad)}\n"
            )
        print(f"check-markdown-links: baseline written ({len(arch_bad)})")
        return 0

    failed = False
    if bad:
        failed = True
        print(f"check-markdown-links: {len(bad)} dead link(s) in live documents:")
        for f, line, target, resolved in bad:
            print(f"  {f}:{line}")
            print(f"      [..]({target})  ->  {resolved}  (no such file or directory)")
        print(
            "\n  A markdown link is CLICKED, so it must resolve exactly.\n"
            "  `check-doc-refs` accepts the `archived/` sibling on purpose — that is\n"
            "  right for a prose mention and wrong for a link, which is how\n"
            "  `phase-424`'s #0835 row 404'd on a green gate (issue 1085).\n"
            "\n  Most of these are one edit: the document moved under `archived/`\n"
            "  when its issue was resolved. If the target is gone for good, drop the\n"
            "  link and keep the name as prose — a reader is better served by a name\n"
            "  than by a 404."
        )

    baseline = read_baseline(ROOT)
    if baseline is None:
        failed = True
        print("check-markdown-links: no archived-link baseline; run --write-baseline.")
    elif len(arch_bad) > baseline:
        failed = True
        print(
            f"\ncheck-markdown-links: archived documents now hold {len(arch_bad)} dead\n"
            f"  link(s), up from the baseline {baseline}. The ratchet only falls.\n"
            "  An archived document is a historical record, so its links are not\n"
            "  retargeted — but ADDING one means a live document was archived with\n"
            "  a link that was already dead, which is worth fixing before it moves."
        )

    if failed:
        return 1
    print(
        f"check-markdown-links: OK ({len(live)} live document(s), every link resolves; "
        f"{len(arch_bad)} dead in archived documents, baseline {baseline})"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
