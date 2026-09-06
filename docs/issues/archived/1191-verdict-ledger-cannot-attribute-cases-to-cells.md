---
id: 1191
title: "one failing case blocks recording verdicts for the four other cells sharing that binary — the ledger keys on the BINARY, not the cell"
status: resolved
type: bug
area: testing
severity: medium
found: 2026-09-07
related: [1127, 1190]
resolved_in: "phase-433 — interop::CASE_CELLS, the case → cell map"
---

# Five cells, one binary, one veto

`scripts/check-interop-verdicts.py --record <cell> --junit <f>` refuses a junit
in which every case of that cell's binary skipped, and refuses a `fail` with no
`--issue`. Both are right, and both are what make the ledger trustworthy.

But it decided on the **binary**, not the cell. `interop_e2e` hosts FIVE
`Tier::Runtime` cells:

    native-pubsub-rust-zenoh-n2r    native-service-rust-zenoh-r2n
    native-pubsub-c-cyclone-n2r     native-service-c-cyclone-r2n
    native-lifecycle-rust-zenoh

Measured 2026-09-07 in the box, that binary ran 10 cases: **9 passed, 1
failed** (`case_2_zenoh_pubsub_ros2_to_nano`, issue 1190). Recording the four
cells whose own cases had all passed was refused, five times, with the same
message about `case_2`:

```
check-interop-verdicts: interop::case_2_zenoh_pubsub_ros2_to_nano FAILED —
record it with --issue NNNN
```

So four cells that DID produce a live pass stayed `NEVER` in the report, and the
count read 6 of 19 where it should have read 10 of 19. The ledger's whole
purpose is to distinguish "never ran" from "ran and passed", and there it could
not.

## Why the conservative behaviour was nonetheless correct

Without a case→cell mapping the tool could not know that `case_2` belongs to
`native-pubsub-rust-zenoh-n2r` and not to `native-lifecycle-rust-zenoh`.
Recording a pass for a cell whose case actually failed would be exactly the
false claim the ledger exists to prevent — worse than an undercount. Refusing
was the right default for a tool that cannot tell. **The fix was to supply the
map, not to loosen the refusal.**

## Resolution — `interop::CASE_CELLS`

Four binaries host more than one Runtime cell: `interop_e2e` (5),
`graph_interop` (2), `ros2_action_e2e` (2), `xrce_ros2_interop` (2). The other
seven host one each, where binary-wide attribution IS cell-wide and nothing
changes.

`packages/testing/nros-tests/src/interop.rs` gained `CASE_CELLS`: one row per
case of a shared binary, naming the cell that case is evidence for, plus
`unowned(...)` rows for a case that is evidence for none. The case is spelled as
the JUNIT spells it (`interop::case_2_zenoh_pubsub_ros2_to_nano`).

The map is **authored**, because nothing in the tree derives it and for
`ros2_action_e2e` nothing could: its two cells are ONE
`(Linux, Rust, Cyclonedds, Action)` coordinate and differ only by direction,
which `coords_for` collapses on purpose. What makes an authored map trustworthy
is that it is checked, statically, against the cases each binary really runs:

* `check-interop-verdicts.py::source_cases` DERIVES a binary's junit case names
  from its source — `#[test]` / `#[rstest]` plus rstest 0.24's own
  `case_<n>_<description>` naming, zero-padding included. A shape it cannot
  name (`#[values(...)]`) raises rather than guessing.
* `map_problems` requires the map to be exhaustive and disjoint over those
  names, in both directions: a renamed, added or deleted case turns the gate
  red instead of moving evidence quietly. A shared binary with no map is an
  error, and so is a cell of a shared binary that owns no case.
* `--record` refuses a junit carrying a case of a mapped binary that the map
  does not name: the binary that RAN and the map disagree, so attribution is
  unknown.
* The ledger check now rejects an entry citing a SIBLING cell's case — the same
  false claim the binary-wide attribution used to make, written into the file.
* Rust-side unit tests in `interop.rs` bind the map to `CELLS`: every row names
  a Runtime cell OF THAT BINARY, no case is claimed twice, every cell of a
  shared binary owns a case.

Attribution by SUBSTRING (matching a cell id against a case name) was
considered and rejected in the issue text, and is still not done: they are
different vocabularies, and it reads as working until a rename.

Verified on the measured run: with the phase-433 W2 ledger and a junit carrying
the nine `interop_e2e` results as recorded (case_2 failed, the rest passed), the
four cells record cleanly and `--report` moves from **6 of 19** to **10 of 19**,
while `native-pubsub-rust-zenoh-n2r` — the cell whose own case failed — is still
refused without `--issue`.

## Fallout to expect

Ledger entries written by the binary-wide recorder cite cases belonging to other
cells (phase-433 W2 gave both `graph_interop` cells both cases, and
`native-pubsub-rust-zenoh-n2r` all nine `interop_e2e` cases). Those entries are
now gate failures and must be re-recorded from the same junit.

`xrce_ros2_interop` runs three ACTION cases and `interop::CELLS` declares no
Xrce/Action cell, so they are `unowned(...)` with that reason: they counted as
the pubsub and service cells' evidence until this map said otherwise. The gap is
a missing CELL, not a missing case.
