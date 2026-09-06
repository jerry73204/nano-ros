---
id: 989
title: "A prose `issue NNNN` reference resolves to nothing and no gate notices —
  `check-doc-refs` validates paths, not the form people actually write"
status: resolved
type: tech-debt
area: build
related: [issue-0883, issue-0196]
resolved_in: "issue 0989 (this filing)"
---

## Symptom

`CLAUDE.md` — loaded into every agent session — pointed at three issues that do
not exist on `main`, and every gate was green:

| ref | cited in | what it was |
| --- | --- | --- |
| 0883 | `CLAUDE.md`, `docs/issues/README.md` | file written, never merged |
| 0632 | `CLAUDE.md` | no file, on any branch, ever |
| 0640 | `CLAUDE.md` | no file, on any branch, ever |

A pointer in CLAUDE.md is an instruction to go read something. When the
something is absent the reader cannot tell whether the document was lost or
their search was bad — and 0883 was the expensive shape: the analysis existed,
was never merged, and the fix landed under a different id, so the tree said
"see 0883" about a file that had never been on main.

## Why `check-doc-refs` did not catch it

That gate checks **paths** — `docs/issues/NNNN-slug.md` written into prose, into
cmake strings, into frontmatter. It exists because a renumbered RFC left a stale
path in a cmake error message, and it is the right check for a path.

Prose does not contain a path. It contains an id with a word in front of it, and
there is nothing for a path-checker to resolve. The two gates are complements.

## Fix

`scripts/check-prose-issue-refs.py`, on the fast line as `just check
prose-issue-refs`. Buildless, milliseconds.

It matches a 4-digit id preceded by `issue`/`issues`, with an optional `#`, and
follows a comma- or slash-separated tail so a cited pair is two references
rather than one. Deliberately narrow: bare 4-digit numbers are everywhere in
this tree (ports, sizes, buffer bounds) and matching them would drown the gate
in noise.

**A RATCHET against a tracked baseline, not a hard zero.** A doc on `main` can
correctly cite an issue whose file is still in another open PR — six references
are in exactly that state today (0940 and 0949, both filed in PR #151). Failing
those would push someone to delete a correct citation, which is worse than the
dangling ref. New violations fail; a row that stops dangling must leave the
baseline, so the debt can only shrink.

## Two false-positive classes, both found by running it

Recorded because both were *plausible* regexes that produced wrong answers, and
the audit is what settled it — the first draft reported 12 violations of which
half were fictional.

1. **A date reads as a second id.** `issue 0364, 2026-07-31` is one reference
   and a date, but the comma-tail rule took `2026` as its pair — inventing a
   dangling reference to "issue 2026". Fixed with a `(?!-\d)` guard; a real id
   is never followed by `-<digit>`.
2. **The gate flags itself.** Its own docstring must name the ids that motivated
   it, and its `just` recipe comment wanted to quote an example. Both tripped
   it. The script is exempt by path, and the recipe comment paraphrases instead
   — `check-doc-refs` records hitting the same thing and solving it the second
   way.

## Verified

* Negative control: injecting `issue 9123` into `docs/design/ARCHITECTURE.md`
  fails the gate and names the file and id; removing it passes.
* Self-test on the normal path (phase-395), seven cases — the bare form, a
  slashed pair, a comma pair, the hyphenated `issue-NNNN` spelling, a `#`
  prefix, and two negatives that must NOT match (a bare number, and a 5-digit
  run).
* The scan that found the original three now reports only the in-flight rows.

## Not covered

Only `issue`-prefixed references. A doc that says "see 0640" with no such word
is invisible to this, and deliberately so — the alternative is matching every
4-digit number in the repo. If that shape ever appears in practice it is a
different gate, not a wider regex here.
