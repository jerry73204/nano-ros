---
id: 1141
title: "The book has no visual identity — no favicon, no logo, no accent CSS, so the public front door looks like a stock mdBook"
status: open
type: enhancement
area: docs
severity: low
related: [phase-188]
found: 2026-09-06
---

# What is left of phase-188, rehomed

Phase 188 ("Book front-door presentation") is **closed and archived**: 188.A
(landing page + architecture entry) and 188.C (funnel + deploy hygiene) landed
2026-05-28, `mdbook build book` is clean, and the dead-link sweep found 0 broken
across 29 + 81 link targets.

Only workstream **188.B** remained, and it is not engineering — it is two
theming files that were **scoped but deferred on purpose, pending a logo
decision** nobody has made in the three and a half months since:

- **Favicon + logo** in `book/theme/`, wired via `[output.html]` in
  `book/book.toml`.
- **Accent CSS** at `book/theme/custom.css`, via `additional-css`.

Filed as an issue rather than kept as a live phase because a phase held open by
a favicon reads, to anyone scanning the roadmap, as an engineering workstream
in progress. It is a design task with a design blocker.

## The actual blocker

**A logo decision.** Not a technical one: mdBook's `additional-css` /
`[output.html]` wiring is already in place for mermaid
(`book/book.toml` → `[preprocessor.mermaid]` + `additional-js`), so the plumbing
this needs has a working precedent in the same file. Someone has to choose the
mark and the accent colour; the wiring after that is small.

## Acceptance

- `mdbook build book` stays clean.
- The deployed site at `NEWSLabNTU.github.io/nano-ros-book/` no longer renders
  as an unstyled stock mdBook — a favicon in the browser tab, and an accent that
  is not mdBook's default.

## Not covered — carried over from phase-188 and still true

A few stale source comments name shims that no longer exist:
`nros-board-mps2-an385/src/lib.rs:19`, `nros-board-orin-spe/src/node.rs`,
`nros-platform-api/src/lib.rs` (x2). Harmless — comments, not code — and worth a
pass with the next platform-crate edit. Recorded here so closing the phase does
not lose them.
