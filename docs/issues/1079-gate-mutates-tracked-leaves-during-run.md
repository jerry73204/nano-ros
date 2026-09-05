---
id: 1079
title: "`fixture_group_collision_gate.sh` appends a bogus `[[bin]]` to two TRACKED example `Cargo.toml`s while ~20 other gates are reading them"
status: open
type: bug
area: testing, tooling
severity: medium
related: [1077, 0196]
found: 2026-09-05
---

# A gate that proves itself by editing the tree everything else is reading

`packages/testing/nros-tests/tests/fixture_group_collision_gate.sh` proves its
gate can fire by creating the collision for real. It picks two leaves out of the
fixture manifest — real, tracked example directories —

```sh
a="${leaves[0]}"          # from `fixtures-manifest.py list --lang rust`
b="${leaves[1]}"
…
printf '\n[[bin]]\nname = "%s"\npath = "src/main.rs"\n' "$collider" >> "$f"
```

and restores them from an `EXIT` trap. Proving a gate fires by making the defect
is the right instinct — a gate nobody watched fail is not a gate. Doing it in
the shared worktree is the problem.

For the ~2 s those two files carry `tripwire_collider`, **every other gate in
the same `check-fast` fan-out that reads leaf `Cargo.toml` files sees it**.
`check-fast` runs 214+ gates at `-P$(nproc)`, and around 20 of them read leaf
manifests.

## Status: a structural hazard read off the code, NOT a measured failure

Stated precisely, because the difference matters and issues 0859–0862 are what
happens when it is blurred:

- **Established, by reading the script:** it appends to two tracked files in the
  shared tree, concurrently with gates that read those files.
- **NOT established:** that this has actually caused a red. A `fixture-groups`
  red naming `tripwire_collider` was seen once on this host, but another agent
  was writing the same checkout at the time, so it is **not attributable** and
  is not offered as evidence.

The measurement this needs: run `fixture_group_collision_gate.sh` in a loop
against a concurrent `check-fast`, and see whether any manifest-reading gate
reports `tripwire_collider`. Until someone does that, this is a hazard, not a
diagnosis.

## Why it is worth fixing regardless

An `EXIT` trap does not run on `SIGKILL`, so an interrupted run can leave two
tracked files modified — and a `git add -u` after that commits a bogus `[[bin]]`
into an example, which CLAUDE.md's "never `git add -A`" rule exists to catch
only by luck.

`run-gates-parallel.sh` already carries a note about reds that look like
"something transiently rewrote what that gate reads" without ever naming a
cause. This is at least one such thing.

## Shape of a fix

Do the perturbation in a scratch copy of the two leaves rather than in place.
The gate needs a tree whose manifest rows collide; it does not need that tree to
be the working one. If the fixture manifest cannot be pointed at a copy, that is
the thing to fix first.

## Note

This is adjacent to issue 1077 but a **separate mechanism**. 1077 was
`grep -q` + `pipefail` turning a match into a failed pipeline, and it is fixed;
this is a gate writing the shared tree. Do not close one against the other.
