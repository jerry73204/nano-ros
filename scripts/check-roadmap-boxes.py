#!/usr/bin/env python3
"""A phase section whose declared NEW files all exist, with boxes still open.

WHY THIS EXISTS (issue 1103). Auditing the eleven oldest phase docs still in
`docs/roadmap/` found phase-215 reading 20 of 38 boxes with a `**Status.** OPEN`
line implying the schema work had not started. It was 27 of 38: seven boxes had
landed and were never ticked -- the `board.cmake` schema, the
`nano_ros_use_board()` import, the FVP run delegation and the book page were
all in the tree while the doc read as available work. Phase 215 is `P1`, so
anyone scanning for something to pick up saw seven finished items.

`check-roadmap-claims` passes on that file, and its rules explain why: they
compare the status TEXT against the box COUNTS, an `**Owns:**` line against an
issue's frontmatter, and whether a document disclaims being a phase. Every one
compares the document against ITSELF. Nothing asked whether the work described
had arrived.

## WHAT IT CHECKS, and why not the obvious thing

The obvious rule -- "an UNTICKED box naming a path that exists" -- was written
first and MEASURED against the known answer. It catches **2 of the 7**:

    caught  215.B.2 (`zephyr/CMakeLists.txt`), 215.I.1 (the book page)
    missed  215.A.1  names cmake VARIABLES, not a path
            215.A.3  "Documented schema cross-references this phase doc" -- prose
            215.B.3  names a FUNCTION and a call-order constraint
            215.D.4  names `just zephyr run-fvp-aemv8r*`, a recipe that does not exist
            215.I.2  "SUMMARY.md update" -- no `/`, so not a path

and on today's tree it fires on 2 boxes, BOTH false positives: 216.A.6 and
W3.4 of phase-325 name a file they intend to EDIT, which of course exists. A
gate that is wrong more often than right gets switched off, so that rule is not
what shipped.

What ships is one level up. A phase SECTION carries a `**Files:**` line naming
what the section creates, with `(new)` on the ones that do not exist yet. When
every path that line declares is in the index and the section still has open
boxes, the work described has arrived and the record has not moved.

Measured against the same known answer: **3 of the 4 relevant sections**
(215.B, 215.D, 215.I -- five of the seven boxes), and **zero** findings on
current `main`. The fourth (215.A) is missed for an interesting reason worth
keeping: its `**Files:**` line names
`packages/boards/nros-board-fvp-aemv8r-smp/board.cmake`, and the file is at
`packages/boards/nros-board-zephyr/boards/fvp-aemv8r-smp/board.cmake`. The doc's
own path is stale, so the section reads 1-of-2 and is not flagged. That is the
gate declining to guess, which is the behaviour to keep.

## IT REPORTS; THE RATCHET FAILS

Issue 1103 argued for warn-only, on the grounds that a failing gate pushes
people to tick boxes for green. That is right about a per-BOX rule, and I no
longer think it is right here: a warning nobody must act on decays into a
comment, which is `check-gate-selftests`' own argument. The resolution is the
ratchet the tree already uses -- the known set is committed, may only SHRINK,
and a new entry fails with a one-line escape (`--write-baseline`). Ticking a box
to silence it is not available, because the flag is on the section, not the box:
the only ways out are to finish the section, correct its `**Files:**` line, or
say in the baseline why it stays.

Exit 0 when no unbaselined section is flagged, 1 otherwise.
"""

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASELINE = os.path.join(ROOT, ".config", "roadmap-boxes-baseline.txt")

SECTION = re.compile(r"^### ")
OPEN_BOX = re.compile(r"^- \[ \]")
BACKTICKED = re.compile(r"`([^`\s]+)`")
FILES_LINE = re.compile(r"\*\*Files:\*\*(.+?)(?:\n\n|\Z)", re.S)


def tracked_paths(root):
    r = subprocess.run(["git", "-C", root, "ls-files"], capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(
            f"check-roadmap-boxes: `git ls-files` failed in {root}.\n"
            f"  {r.stderr.strip()}\n"
            "  This reads the INDEX on purpose — issue 0844, and a walk of\n"
            "  `docs/` would still be the wrong question."
        )
    return set(r.stdout.split())


def declared_paths(text):
    """Repo-relative paths a `**Files:**` line names, in order."""
    m = FILES_LINE.search(text)
    if not m:
        return []
    out = []
    for tok in BACKTICKED.findall(m.group(1)):
        tok = tok.rstrip(".,;:)")
        # A path, not a symbol: `nano_ros_use_board()` and `NROS_BOARD_RUNNER`
        # are named on these lines too and are not files.
        if "/" in tok and not tok.endswith("()"):
            out.append(tok)
    return out


def scan(root, docs, tracked):
    """(doc, heading, declared, open_box_count) per flagged section."""
    flagged = []
    for d in sorted(docs):
        try:
            with open(os.path.join(root, d), encoding="utf8") as fh:
                lines = fh.read().split("\n")
        except OSError:
            continue
        heads = [i for i, l in enumerate(lines) if SECTION.match(l)]
        for n, i in enumerate(heads):
            end = heads[n + 1] if n + 1 < len(heads) else len(lines)
            body = "\n".join(lines[i:end])
            open_boxes = sum(1 for l in lines[i:end] if OPEN_BOX.match(l))
            if not open_boxes:
                continue
            declared = declared_paths(body)
            if not declared:
                continue
            if all(p in tracked for p in declared):
                flagged.append((d, lines[i].strip(), declared, open_boxes))
    return flagged


def active_docs(tracked):
    return [
        p for p in tracked
        if p.startswith("docs/roadmap/") and "/archived/" not in p and p.endswith(".md")
    ]


def read_baseline():
    known = set()
    try:
        with open(BASELINE, encoding="utf8") as fh:
            for line in fh:
                line = line.split("#")[0].strip()
                if line:
                    known.add(line)
    except OSError:
        pass
    return known


def key(doc, head):
    return f"{doc}\t{head}"


def self_test(quiet=False):
    """Negative control, on the NORMAL path (`check-gate-selftests` holds this
    file to that). Builds the phase-215 shape and the two ways it must stay
    quiet."""
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        os.makedirs(os.path.join(tmp, "docs", "roadmap"))
        doc = "docs/roadmap/phase-1-x.md"
        tracked = {doc, "a/made.md", "b/also.md"}

        def check(body):
            with open(os.path.join(tmp, doc), "w", encoding="utf8") as fh:
                fh.write(body)
            return scan(tmp, [doc], tracked)

        # The 215 shape: every declared file exists, a box is still open.
        got = check("### S — thing\n\n- [ ] **1.A** do it\n\n"
                    "- **Files:** `a/made.md` (new), `b/also.md` (new).\n")
        assert len(got) == 1, f"the landed-but-open section must fire; got {got}"

        # Ticked — silent.
        got = check("### S — thing\n\n- [x] **1.A** do it\n\n"
                    "- **Files:** `a/made.md` (new).\n")
        assert got == [], f"a section with no open box must not fire; got {got}"

        # One declared file still missing — the work is NOT all there, so this
        # must stay quiet. It is also how phase-215.A escaped: its `**Files:**`
        # line names a path that moved, and guessing would have been worse.
        got = check("### S — thing\n\n- [ ] **1.A** do it\n\n"
                    "- **Files:** `a/made.md` (new), `c/absent.md` (new).\n")
        assert got == [], f"a partially-built section must not fire; got {got}"

        # No `**Files:**` line — nothing to check, and prose is not evidence.
        got = check("### S — thing\n\n- [ ] **1.A** do it in `a/made.md`\n")
        assert got == [], f"a section without a Files line must not fire; got {got}"

        # Symbols on the Files line are not paths.
        got = check("### S — thing\n\n- [ ] **1.A** do it\n\n"
                    "- **Files:** `nano_ros_use_board()`, `NROS_BOARD_RUNNER`.\n")
        assert got == [], f"symbols must not be read as paths; got {got}"

    if not quiet:
        print("check-roadmap-boxes self-test: OK (5 case(s))")
    return 0


def main(argv):
    if "--self-test" in argv:
        return self_test()
    rc = self_test(quiet=True)
    if rc:
        return rc

    tracked = tracked_paths(ROOT)
    docs = active_docs(tracked)
    flagged = scan(ROOT, docs, tracked)

    if "--write-baseline" in argv:
        os.makedirs(os.path.dirname(BASELINE), exist_ok=True)
        with open(BASELINE, "w", encoding="utf8") as fh:
            fh.write(
                "# Phase sections whose declared `**Files:**` all exist while boxes\n"
                "# stay open (issue 1103). A ratchet: entries may only be REMOVED.\n"
                "# An entry here is a claim that the section is genuinely unfinished\n"
                "# despite its files landing — say why beside it.\n"
            )
            for d, h, _, _ in flagged:
                fh.write(f"{key(d, h)}\n")
        print(f"check-roadmap-boxes: baseline written ({len(flagged)} section(s))")
        return 0

    known = read_baseline()
    new = [f for f in flagged if key(f[0], f[1]) not in known]
    stale = known - {key(d, h) for d, h, _, _ in flagged}

    if new:
        print(f"check-roadmap-boxes: {len(new)} phase section(s) look LANDED but "
              "still carry open boxes:")
        for d, h, declared, n in new:
            print(f"  {d}")
            print(f"      {h}")
            print(f"      {n} open box(es); all {len(declared)} declared file(s) exist:")
            for p in declared:
                print(f"        {p}")
        print(
            "\n  Every path the section's `**Files:**` line declares is in the index,\n"
            "  and the section still reads as work to pick up. That is how phase-215\n"
            "  showed 20 of 38 boxes while 27 were done (issue 1103) — a P1 phase\n"
            "  advertising seven finished items as available.\n"
            "\n  Three ways out, and only the first two are cheap:\n"
            "    * tick the boxes whose work has landed, WITH the witness;\n"
            "    * correct the `**Files:**` line if it names a path that moved\n"
            "      (215.A's did, which is why it is not on this list);\n"
            "    * `--write-baseline` and say beside the entry why it stays open."
        )
    if stale:
        print(f"\ncheck-roadmap-boxes: {len(stale)} baseline entry/entries no longer "
              "apply — remove them (the ratchet only shrinks):")
        for k in sorted(stale):
            print(f"  {k.replace(chr(9), '  ')}")

    if new or stale:
        return 1
    print(f"check-roadmap-boxes: OK ({len(docs)} active phase doc(s), "
          f"{len(flagged)} known section(s), none new)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
