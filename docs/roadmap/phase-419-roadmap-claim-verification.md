# Phase 419 — a roadmap phase that contradicts itself should not need a reader to notice

**Status (2026-09-04). W1, W2 (three of four rules) and W3 LANDED in one change;
R2 remains, with its cost measured.**

* **W1 done.** `check-doc-refs` owns the numbered doc series;
  `check-ci-doc-workflow-refs` owns workflow names but only inside three
  hand-listed `docs/development/` files, so roadmap docs are covered by
  NEITHER — 20 dead workflow references across 8 phases, 11 of them in
  phase-196 alone. R1, R3 and R4: not covered by anything.
* **W2 done for R1, R3, R4** — `scripts/check-roadmap-claims.py`, fast line,
  self-testing, ratcheted. Four findings baselined and **two already worked**:
  phase-292 (W1/W3/W4 landed the day after its "Draft" line was written, plus
  three as-landed claims that have since drifted) and phase-325 (W0-W2 landed
  on the very date its status says "Not started", and its blocker — issue 0362
  — is resolved and archived, so W3 was never actually blocked). Baseline 4 →
  2 → **0**: 230 and 275 landed in #322, the ratchet reported both as "no
  longer offend", and the lines were deleted. The gate now enforces ZERO, so
  the next contradiction is a hard failure rather than a baseline row.
* **W2 R2 NOT done, and the reason is measured**: extending
  `check-ci-doc-workflow-refs` means giving it a baseline it does not have, for
  20 references most of which are legitimate history. Reported by W3 instead.
* **W3 done** — `just roadmap-audit` + a monthly `roadmap-audit.yml`, a REPORT
  and never a gate.

## The finding

A sweep of the 13 oldest open phases on 2026-09-04 found **8 whose status line
had outlived its content**, 1 verified accurate, and 4 not verified. That ratio
is the argument for this phase: at eight in thirteen, a reader cannot use a
status line without re-deriving it, which is the same as not having one.

What each cost is not tidiness. phase-196's single open acceptance criterion
named `zephyr-dual-line.yml`, a workflow phase-253 deleted — so the criterion
could be neither met nor refuted, and the phase sat "In progress" for three
months waiting on a lane that had been consolidated into `nightly.yml`. Chasing
that down is what surfaced issue 1029 (`1029-zephyr-nightly-never-produces-a-verdict`, filed in PR #320 and not yet on `main` — link it once that lands)
(eight consecutive scheduled runs producing no zephyr verdict at all), which
nobody was looking for.

The corrected phases: 403, 296, 209, 356, 357, 196, 230, plus 275 misfiled as a
phase at all, and 41's stale paths. [phase-162](phase-162-rt-scheduling-harness.md)
was checked and is CORRECT — recorded here because after eight in a row the next
error is assuming every old status line is wrong.

## The split that decides the design

**`scripts/check-roadmap-status.sh` already exists**, is on the fast line, and
deliberately declines this job:

> The check is deliberately shallow: it does not judge whether the status is
> CURRENT, only that one exists and is findable near the top. Staleness of the
> CONTENT is a human call.

That reasoning is right for most of what was found and wrong for some of it, and
the boundary is sharp. When a header says "nothing landed" and the body says
`LANDED`, no judgment is involved — the document contradicts **itself**. When a
phase is superseded because a later phase reversed its direction, judgment is
all there is.

Sorting the 2026-09-04 findings by which side they fall on:

| finding | detectable how | side |
| --- | --- | --- |
| 403 header "Design, nothing landed"; body has five `LANDED` waves | header vs body | mechanical |
| 230 header `Planned`; Waves 0 and 3 marked `✅ DONE` | header vs body | mechanical |
| 236 header `Proposed`; 7 of 12 boxes ticked | header vs body | mechanical |
| 196 acceptance names 12 workflows, ALL deleted | cited path missing | mechanical |
| 41 two cited paths predate the `packages/cli/` move | cited path missing | mechanical |
| 356 W3 remainder waits on issue 0260, which is resolved+archived | linked issue state | mechanical |
| 275 self-declares "NOT A WORK-ITEM PHASE", sits in the active dir | phrase + location | mechanical |
| 209 "none of the templates is in any fixture row" — 9 rows exist | claim of ABSENCE | judgment |
| 296 superseded: phase-330 W7 reversed `model =` → `launch =` | cross-phase reversal | judgment |
| 211 proposes retiring Verus; `just verify` still depends on it | premise reversed | judgment |
| 357 W1 "type/validator still to come"; `wcet.rs` has both | needs the right file | judgment |

**7 mechanical, 4 judgment.** Two layers, and neither should pretend to be the
other.

---

## W1 — measure the overlap with the gates that already exist, FIRST — DONE

`check-doc-refs`, `check-doc-recipe-refs` and `check-ci-doc-workflow-refs`
already check references from docs. Before adding a rule, establish which of the
mechanical rules below are already covered — R2 (cited path missing) is the
likely duplicate, and `check-doc-recipe-refs` already carries a baseline of 49
known-dead references.

If a rule is covered, EXTEND that gate rather than add a sibling. A second gate
answering the same question is how two green tools come to disagree — CLAUDE.md
records the api-parity pair doing exactly that, 25 symbols apart.

Acceptance: for each of R1-R4, a one-line statement of "covered by X" or "not
covered", with the command that establishes it.

## W2 — the contradiction gate — R1/R3/R4 DONE, R2 deferred to W3

Buildless, offline, self-testing on the normal path (`check-gate-selftests`
requires that), ratcheted against a baseline that may only SHRINK. Fast line.

* **R1 — header claims not-started while the body says otherwise.** The header
  matches `not started|nothing landed|Planned|Proposed|Draft`, the body has
  ticked boxes or `LANDED`/`✅` marks, **and the header does not itself admit
  progress.**

  That last clause is not a refinement, it is the difference between a usable
  gate and one that gets switched off. Measured 2026-09-04: without it the rule
  flags 12 phases including `phase-375`, whose header reads *"PROPOSED — W0
  landed, W1–W5 not started"* — honest self-disclosure, and a gate that flags
  honesty teaches people to ignore it. With it, five:

      phase-230  ticked=7   landed-marks=20    (confirmed stale, corrected)
      phase-236  ticked=7   landed-marks=4     (confirmed stale, corrected)
      phase-292  ticked=17  landed-marks=1     "Draft", 17 of 18 boxes ticked
      phase-325  ticked=14  landed-marks=0     "Draft. Not started." + 14 ticked
      phase-206  ticked=0   landed-marks=3     weak — likely prose, not a claim

  Four strong, one weak. **292 and 325 are live findings this rule surfaced**,
  not yet investigated — treat them as the rule's first real output rather than
  as backlog to grandfather silently.

  **`landed-marks` alone is NOT a usable signal, and THIS DOCUMENT is the
  counterexample.** Its header says "PROPOSED — nothing landed" and it contains
  13 occurrences of `LANDED` — every one of them prose about OTHER phases, in
  the table above. `phase-206` is the same shape at a smaller scale (ticked=0,
  marks=3). So the rule must key on **ticked checkboxes**, which are a structural
  claim about THIS document's own work items, with mark-counts at most a weak
  secondary that never fires alone. A first cut that counted words would have
  flagged the phase proposing the gate, on its first run.

* **R2 — a repo path cited by a phase does not exist.** Pending W1's overlap
  answer.

* **R3 — a phase presents work as outstanding while the issue it names is
  `resolved` or archived.** Keyed on the frontmatter `status:`, not on prose.

* **R4 — a doc in `docs/roadmap/` that says it is not a work-item phase.**
  One line, one case today (275), and it recurs whenever branch notes get filed
  as a phase.

Acceptance: the gate reproduces 403, 230, 236, 356 and 275 against the tree AS
IT WAS before their corrections (they are fixed now, so a self-test with fixture
documents is the only honest way to assert this), passes on `phase-375` and
`phase-162`, and its baseline shrinks by at least the four strong R1 hits.

## W3 — the periodic pass, for the four the gate cannot reach — DONE

A scheduled agent pass, monthly. Output is an issue or a correction PR —
**never a red gate.** A scheduled required check is the #0975 deadlock, and
`pr-verdicts.yml` already documents that shape and why it stays advisory.

Its job is the judgment column: supersession, reversed premises, claims of
absence. Give it the four worked examples above as its brief, because each was
found by a different move and the moves are the transferable part:

* **296** — read the phase that came AFTER and check whether it reversed this
  one. The tell was phase-330 W7's own text naming "the `nros::main!(launch = …)`
  form that phase-296 R4 removed".
* **211** — take the phase's central verb ("retire Verus") and check the tree
  for it (`just verify` still depends on `verify-verus`; zero `creusot` files).
* **209** — a claim of absence is checkable by looking for the thing claimed
  absent; "none in any fixture row" fails against 9 rows in `fixtures.toml`.
* **357** — a claim that an artifact is "still to come" is checkable once you
  guess its name; `wcet.rs` carried `WcetProfile` and its validator.

Acceptance: one pass run, its findings filed, and the time it took recorded — so
the next person can decide whether monthly is the right cadence or whether the
gate made it unnecessary.

## Explicitly not in this phase

**Judging whether a phase's plan is still a good idea.** This is about whether a
document describes the tree, not whether the work is worth doing. phase-211 is
the case that shows the difference: its status is accurate (a proposal, nothing
started) and its PREMISE is dead, and only the second is in scope here — the
decision about whether to adopt Creusot is a maintainer's, not a gate's.

**Auto-correcting a status line.** Every correction on 2026-09-04 needed the
reason written down, and a generated status line would carry the claim without
the evidence. The gate's output is a FAILURE naming the contradiction; a human
or an agent writes what actually happened.

## Related

* [phase-411](archived/phase-411-scope-is-the-spec-verb-first-workflow.md) — a skip is
  indistinguishable from a pass; this is the same shape one series over, where a
  stale status is indistinguishable from a current one.
* [phase-413](phase-413-ci-workflow-user-parity.md) — "a uniformly-red lane has
  no signal capacity". A roadmap where most status lines are wrong has none
  either.
* issue 1029 (`1029-zephyr-nightly-never-produces-a-verdict`, filed in PR #320 and not yet on `main` — link it once that lands) —
  found by chasing phase-196's stranded acceptance criterion, and the reason
  this phase is worth its cost.
