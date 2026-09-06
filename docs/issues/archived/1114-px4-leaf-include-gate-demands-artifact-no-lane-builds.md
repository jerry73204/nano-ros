---
id: 1114
title: "`_require-leaf-includes` demands `generated/px4_msgs` from every lane, so tier 2 dies on three leaves it never builds"
status: resolved
type: bug
area: build, testing
severity: high
related: [0463, 0474, 0510, 1025, 0319]
found: 2026-09-06
---

# The exemption was right; its predicate asked about the host

## Symptom

`run-matrix.yml` — the ONLY automated path to a tier-2 runtime verdict — fails in
`just build tier2`:

```
error: 3 path dep(s) into `generated/` do not exist, across 3 manifest(s).

  examples/px4/rust/companion/offboard-companion/Cargo.toml
      path -> generated/px4_msgs  (absent)
  examples/px4/rust/companion/px4-probe/Cargo.toml
      path -> generated/px4_msgs  (absent)
  examples/px4/rust/companion/px4-stub/Cargo.toml
      path -> generated/px4_msgs  (absent)

error: recipe `_require-leaf-includes` failed with exit code 1
```

The step's own `nros sync` had just run and reported success — for
`builtin_interfaces` and `std_msgs`, the only two packages it can produce.

## Cause

`generated/px4_msgs` is **not** produced by `nros sync`. Issue 0510 records why:
px4_msgs is not an ament package, so only `nros generate-px4-msgs` emits it, from
the PX4 `.msg` tree, and only `just px4 build-fixtures` runs that.

`scripts/build/leaf-config-includes.py` already knew this and carried an
exemption. **The exemption's PREDICATE was wrong, not its existence:**

```python
px4_available = px4_msg_tree().is_dir()
...
if PX4_PRODUCED_RE.search(dep_path) and not px4_available:
    gen_unprovisioned.append((rel, dep_path))   # reported, not required
    continue
gen_missing.append((rel, dep_path))             # FAILS
```

It asked *"could the producer have run?"* — a fact about the **host**. The
question is about the **lane**. On a machine with PX4 provisioned (the
self-hosted runner, and this dev box) the exemption lifts and the directory
becomes mandatory for **every** lane — including the ones that never build a px4
leaf.

**px4 has ZERO rows in `examples/fixtures.toml`** — not `[[fixture]]`, not
`[[workspace_fixture]]`, not `[[compile_check_fixture]]`. It is in no lane at
all. So tier 2 was blocked by three leaves no tier builds.

This is CLAUDE.md's affordability rule one level down:

> A gate in an affordability tier may only resolve artifacts the JOB ITSELF
> builds — `check-lane-contracts` enforces this for every merge-gating lane.

Here the gate resolved an artifact whose producer is a lane the job never
invokes. `check-lane-contracts` did not catch it because the dependency is not a
fixture stamp; it is a directory a *different* lane writes.

## Why it hid

The condition needs BOTH halves: PX4 provisioned AND `generated/px4_msgs`
absent. A dev box that has ever run `just px4 build-fixtures` has the directory,
so the guard passes and the bug is invisible — this one did, which is why the
first local reproduction attempt was green and the "local fails the same way"
guess was wrong. It reproduces by moving the three directories aside.

## Fix

`generated/px4_msgs` is now **reported, never required**:

```python
if PX4_PRODUCED_RE.search(dep_path):
    gen_unprovisioned.append((rel, dep_path))
    continue
```

Correct because the one lane that CONSUMES these leaves PRODUCES the directory
first — `just px4 build-fixtures` generates into all three before building any of
them — so that lane passes on the directory EXISTING, not on this guard's
verdict, and a lane that has not run the producer was never going to parse those
manifests.

Reporting rather than dropping keeps the diagnosis the guard exists for: a bare
`cargo metadata` over a px4 leaf still gets cargo's raw manifest-parse error, and
the note is what explains it. The note now names the LANE reason and tailors its
hint to whether PX4 is provisioned.

### Measured, both directions

With the three directories moved aside on a PX4-provisioned host:

| | rc |
| --- | --- |
| before the fix | **1** — the exact CI failure |
| after the fix | **0**, with the narrowing reported |

## Not covered

* Whether `check-lane-contracts` should grow a rule for "artifact produced by
  another lane", which is the class this instance belongs to. It currently
  reasons about fixture stamps only.
* px4 having no `fixtures.toml` row at all. That is a real coverage gap — the
  px4 leaves are built by `just px4` and by no tier — and it is what made this
  guard the only thing standing between tier 2 and a green run. Filed here as
  context, not fixed.
