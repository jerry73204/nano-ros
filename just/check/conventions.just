# nano-ros gates: conventions
#
# Cross-cutting source conventions that belong to no one subsystem.
#
# One of the topic files `just/check.just` imports. The names stay FLAT -- a
# gate is still `just check <name>` -- because `import` merges definitions
# rather than namespacing them, and ~350 references across docs, scripts and
# workflows spell them that way.
#
# Adding a gate here does not touch any other file, which is the point: for
# most of this repo's life every new gate appended to ONE file, and
# `just/check.just` became the worst conflict surface in the tree at 48 of
# 300 commits.

# Issue 0498 — a sync-owned file a CONCURRENT process reads must be written
# temp + `rename(2)`, never `fs::write` (which truncates to zero, then fills).
# `cmd/ws.rs` already had a private `atomic_write` whose doc called it "the
# discipline every other sync-owned file here uses"; the metadata sidecar one
# directory over had three plain `fs::write` writers and died mid-sweep on an
# empty read. Buildless — greps the guarded function bodies.
[private]
atomic-sync-writes:
    @bash scripts/check-atomic-sync-writes.sh

# Issues 0400/0706/0712 — EVERY `export -f` list in scripts/build/ must close
# over its call graph, or a helper added to an exported function dies
# "<name>: command not found" in the make WORKER and nowhere else. Three times
# now. Supersedes the #0717 gate, which checked one entry point on the reasoning
# that build_root_derivation.sh covered the other — true, but that scenario
# EXECUTES one call path, so a helper on a branch not taken is invisible to it.
[private]
export-f-closure:
    @bash scripts/check-export-f-closure.sh

# eyre's `Context` alias is behind `#[cfg(feature = "anyhow")]` from 0.6.13, so
# code using it compiles only against a graph that resolves 0.6.12.
[private]
eyre-context-alias:
    @bash scripts/check-eyre-context-alias.sh

# Issue 0481 — a readiness grep naming an AMBIGUOUS literal waits out its whole
# timeout in silence and still passes. Buildless: reads sources only.
[private]
readiness-marker-literals:
    @bash scripts/check-readiness-marker-literals.sh

# String-convention guards (forbidden org / retired-tool refs in user surfaces).
# The `check.yml` step (SSoT).
[private]
string-conventions:
    @scripts/ci/string-conventions-check.sh
