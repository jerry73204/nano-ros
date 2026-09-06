---
id: 1191
title: "one failing case blocks recording verdicts for the four other cells sharing that binary — the ledger keys on the BINARY, not the cell"
status: open
type: bug
area: testing
severity: medium
found: 2026-09-07
related: [1127, 1190]
---

# Five cells, one binary, one veto

`scripts/check-interop-verdicts.py --record <cell> --junit <f>` refuses a junit
in which every case of that cell's binary skipped, and refuses a `fail` with no
`--issue`. Both are right, and both are what make the ledger trustworthy.

But it decides on the **binary**, not the cell. `interop_e2e` hosts FIVE
`Tier::Runtime` cells:

    native-pubsub-rust-zenoh-n2r    native-service-rust-zenoh-r2n
    native-pubsub-c-cyclone-n2r     native-service-c-cyclone-r2n
    native-lifecycle-rust-zenoh

Measured 2026-09-07 in the box, that binary ran 10 cases: **9 passed, 1
failed** (`case_2_zenoh_pubsub_ros2_to_nano`, issue 1190). Recording the four
cells whose own cases all passed is refused, five times, with the same message
about `case_2`:

```
check-interop-verdicts: interop::case_2_zenoh_pubsub_ros2_to_nano FAILED —
record it with --issue NNNN
```

So four cells that DID produce a live pass stay `NEVER` in the report, and the
count reads 6 of 19 where it should read 10 of 19. The ledger's whole purpose
is to distinguish "never ran" from "ran and passed", and here it cannot.

## Why the conservative behaviour is nonetheless correct today

Without a case→cell mapping, the tool cannot know that `case_2` belongs to
`native-pubsub-rust-zenoh-n2r` and not to `native-lifecycle-rust-zenoh`.
Recording a pass for a cell whose case actually failed would be exactly the
false claim the ledger exists to prevent — worse than an undercount. Refusing
is the right default for a tool that cannot tell.

## The mapping already exists, in two places

1. Each multi-cell test carries `interop::assert_test_bound(<test>, &COORDS)`,
   and `interop_e2e` goes further — `scenario_coord()` maps every scenario to
   its `(platform, lang, rmw, workload)`, which is precisely the cell key. That
   function is already the tripwire's source of truth.
2. The case NAMES encode it (`case_7_cyclone_pubsub_ros2_to_nano`), which is
   how a human reads the report — but a name convention is the weaker of the
   two and should not be the mechanism.

So the fix is to have the binary emit its case→cell map, or to have the ledger
consume `scenario_coord`'s, rather than to loosen the refusal.

## Not to be fixed by

* Recording per-cell verdicts by hand — the ledger's value is that the tool
  refuses what a run does not support.
* Relaxing to "record a pass if the cell's own NAME does not appear in a failing
  case" — a cell id and a case name are different vocabularies, and matching
  them by substring is the kind of coupling that reads as working until a
  rename.

## Found by

phase-433 W2's after-run, recording the first six live verdicts. The undercount
is visible in `just interop-verdicts` today.
