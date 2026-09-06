---
id: 1133
title: "RETRACTED — `just setup-cli` leaving a STALE CLI was not reproducible,
  and the stated cause is refuted by measurement"
status: wontfix
type: bug
area: build
related: [issue-0466, issue-0627, phase-375, phase-431]
---

## Retraction

**Filed 2026-09-06, retracted the same day. The stated cause is wrong and the
symptom does not reproduce.** Keeping the file rather than deleting it, because
the measurement is worth more than the bug would have been.

The claim was: `source_stamp` hashes all of `packages/cli` while cargo builds a
subset, so adding a file cargo does not build changes the tree's stamp, gives
cargo nothing to rebuild, and leaves `just setup-cli` printing `built:` over an
unchanged baked stamp.

## What was measured

Two candidate triggers, both against an UNPATCHED `setup-cli`:

| Trigger | Stamp after edit | Run 1 | Run 2 | Run 3 |
| --- | --- | --- | --- | --- |
| new file under `nros-cli-core/tests/` | STALE | FRESH | FRESH | FRESH |
| edit a `.rs` under a `cli-source-dirs.txt` closure dir (`examples/templates/multi-node-workspace`) | STALE | FRESH | FRESH | FRESH |

**One `just setup-cli` clears both.** Adding a test file changes the package's
fingerprint, so cargo does relink the bin target; and the closure dirs are real
cargo inputs, declared by `build.rs`, so an edit there rebuilds too. The premise
that a stamp input can sit outside cargo's input set is not demonstrated by
either candidate.

A third file class was also checked and is a non-event: a README under a closure
dir does not move the stamp at all, because `is_cli_input` watches only `.rs`,
`.jinja`, `Cargo.toml`, `Cargo.lock`, `cli-source-dirs.txt` and `askama.toml`.

## What actually happened

The observation behind the filing was real but is better explained by CLI
RESOLUTION, which is documented and already guarded:

* The failing gate resolved `nros` through `PATH`, which pointed at
  `/home/aeon/repos/nano-ros/packages/cli/target/release/nros` — the MAIN
  checkout's binary — while `just setup-cli` was rebuilding a WORKTREE's. No
  number of rebuilds in one tree can refresh the other tree's binary. That is
  the "museum-CLI trap" `nros_resolve_cli` documents and phase-431 W1's
  `refuse_if_foreign` exists for.
* Overriding `PATH` did not immediately help, most plausibly because cmake
  caches the resolution (`_NROS_CLI_RESOLVED`) in the build directory, so the
  already-configured fixture kept the path it first found.

Neither of those is the bug as filed, and neither is fixed by making
`setup-cli` retry.

## Why no fix landed

A retry loop was written and then reverted. It would have been machinery for a
condition nobody has demonstrated, in a recipe whose job is already subtle —
exactly the speculative mechanism this tree keeps having to delete. If the
symptom recurs with a reproduction, reopen with the trigger, not the theory.

## What the episode did justify

One diagnostic gap is real: the STALE message names the binary but not which
checkout its stamp describes, and in a session with several worktrees that is
the datum you need first. Addressed separately.
