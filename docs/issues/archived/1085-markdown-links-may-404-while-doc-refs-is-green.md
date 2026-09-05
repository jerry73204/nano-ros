---
id: 1085
title: "146 markdown links in live documents were dead while `check-doc-refs`
  was green — a link is CLICKED, and the gate that guards it resolves loosely"
status: resolved
resolved: 2026-09-05
type: bug
area: tooling, docs
related: [1071, 1072, 0883, 0884]
---

## Symptom

`docs/roadmap/phase-424-build-graph-freshness-truth.md` carried

    | [#0835](../issues/0835-fixture-staleness-probe-families-restale-each-other.md) | fixtures | … |

while the file had been at `../issues/archived/0835-…` since it was resolved.
On GitHub that link is a 404. `check-doc-refs` passed.

Swept the tree: **146 dead links across 698 live documents**, and 123 of them
were exactly this shape — a live document still pointing at where an issue,
phase or RFC used to be.

## Why the gate was green, and why it was right to be

`check-doc-refs` resolves a reference if the basename is at the named path **or**
in that series' `archived/` directory. Its own header gives the reason:

> Renumbering is rare but not exceptional — parallel sessions collide on ids
> often enough that `just issue-new` exists to reserve them — and archiving
> MOVES a resolved issue under `archived/`.

That is a deliberate, correct decision **for what that gate checks**: a path
MENTIONED in prose, in issue frontmatter, or inside a cmake error string. Making
it exact would mean retargeting every prose mention of an issue the moment it is
archived — churn across a hundred documents that are still telling the truth.

The defect is that a `[text](target)` LINK was riding the same rule. A mention
is read; a link is CLICKED. Those want different strictness, and one gate was
answering for both.

This is issue 1071's shape again — *the gate exists, it runs, it is green, and
the rule it enforces is narrower than the rule its name implies* — with the twist
that here the narrow rule is right and the mistake was reusing it for a second
question.

## What the sweep found, by category

| | count | shape |
| --- | --- | --- |
| retargetable to `archived/` | 123 | the document moved when its issue resolved |
| moved package directories | ~15 | `packages/drivers/` split into `net/serial/ipc/sys` (phase-321 W2.f); platform crates are under `packages/platform/`, not `core/` or `platforms/` |
| target deleted | ~8 | the `ws-*` workspace dirs (phase-331), `nros-board-stm32f4`, `cargo-ros2`, `TEST_COVERAGE.md` |
| globbed on purpose | 3 | `[issue 0774](0774-*)` — the author did not know whether it was archived, which is the same uncertainty this change removes |

## Fixed

* **123 retargeted mechanically**, under a safety condition rather than a guess:
  a link is rewritten only when the basename is UNIQUE in the tree, so there is
  nothing to choose between and the rewrite cannot pick the wrong file.
* **The moved directories were looked up**, not inferred.
* **Dead-for-good targets are UNLINKED, keeping the name as prose.** A reader is
  better served by a name than by a 404, and a link that cannot resolve is not
  made better by being a link.
* **`docs/issues/open.md` is exempt, with the reason in the gate.** It is
  generated and gitignored on purpose (0883/0884): a committed open list was the
  per-PR conflict site that serialised the merge queue, and GitHub's server-side
  rebase runs no `.gitattributes` merge driver, so untracking it was the only fix
  that reached the queue. It exists on any tree that has run the generator, which
  is what the sentence linking it tells the reader to do.

## Gated

`check-markdown-links`, on the fast line. Every link in a live markdown document
must resolve to the file or directory it names.

Three scope decisions, each because the alternative makes the gate worse:

* **`book/` is out** — `check-book-links` knows about mdBook's `SUMMARY.md` and
  the generated `api/` trees this gate would flag wrongly.
* **`**/archived/**` is COUNTED, not fixed.** An archived document pointing at
  where a file was when it was written is a historical record; retargeting 429 of
  them is a large diff no reader benefits from. A ratchet that may only fall, so
  archiving a document with an already-dead link is still caught.
* **Code spans are stripped first.** `T operator[](i)` in
  `docs/design/0033-message-field-capacity-config.md` is C++, and a gate that
  reports it as a broken link is a gate people switch off.

Self-test on the normal path, **8 cases**, including the exact 1085 shape and
the `operator[]` false positive. Mutation-checked against the unfixed tree:
**146 findings before, 0 after.**

Two of those cases exist because the first version of the gate **flagged its own
documentation.** This file quotes the broken link it is about, as a 4-space
indented block, and `strip_code` handled fences and inline spans only — so
archiving this issue pushed the archived count from 429 to 430 and the ratchet
went red on the very commit that introduced it. Indented blocks are stripped
now, but only where one can START (after a blank line), so a wrapped list item
is still prose and its link still counts. Both directions are asserted.

## What this does NOT claim

The 429 dead links inside archived documents are still dead. That is a stated
trade, not an oversight, and the ratchet is what keeps it from growing.
