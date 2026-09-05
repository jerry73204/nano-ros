---
id: 1077
title: "`just check fast` flakes on the gates that spawn cmake, and each flake reads as a different real bug"
status: open
type: bug
area: tooling, testing
severity: medium
found: 2026-09-05
related: [0876, 0445]
---

# Four gates, four different-looking failures, all green when run alone

Measured on 2026-09-05 across four `just check fast` runs on this host
(12 cores, `-P$(nproc)`):

| run | gates | failed |
| --- | --- | --- |
| 1 | 214 | `message-bound-knobs` |
| 2 | 213 | `message-bound-knobs`, `reconfigure-on-change` |
| 3 | 216 | `image-paths-apply-policy`, `no-tracked-file-find`* |
| 4 | 216 | `fixture-staleness-probes` |

\* `no-tracked-file-find` was a real red, since fixed. Every other cell above
passes when its gate is run alone, immediately afterwards, with no tree change.

What the four flaking gates have in common: each shells out to a real `cmake`
(and in one case `ninja`) rather than reading files. `check-fast` runs 214–216
gates at `-P$(nproc)`, and each of those gates then spawns its own subprocesses.

## Why this costs more than the re-run

**Each flake reads as a different, plausible, specific bug.** Not a timeout, not
a crash — a considered assertion message pointing at real code:

- `[FAIL] O: a subscribed type with no bound did NOT refuse -- a payload class
  would be sized from a blank _RX`
- `[FAIL] O: the refusal does not name the offending type`
- `[FAIL] first pass did not read the placeholder`
- `[FAIL] the old cmake rule did NOT fire: this fixture no longer models the
  defect`

The last one reads exactly like a fixture that has stopped testing what it was
written for — the thing this repo files issues about. The first two are the
*same gate* failing *different assertions* on consecutive runs, which is the
clearest evidence that neither is a real defect: a regression fails the same
assertion twice.

This is issue 0876's shape one lane over. A lane whose reds are not verdicts has
no signal capacity, and a red that is articulate is worse than one that is
obviously infrastructure, because it survives a reviewer's judgement.

## Reproduced on demand

Six concurrent copies of the same gate, no other load:

```
for i in 1 2 3 4 5 6; do ./tests/cmake-message-bounds-tests.sh & done; wait
# 5 × rc=0, 1 × rc=1
```

So it is concurrency, not a specific pairing of gates, and it does not need a
loaded machine — six copies of one 2.7 s script is enough.

## The observation that should be the next person's starting point

In the failing copy, the assertion is:

```sh
_o_out=$(cmake -DFRAG="$T/o-bad.cmake" -P "$T/o-join.cmake" 2>&1)
if ! printf '%s' "$_o_out" | grep -q "status=refused"; then
    fail "O: a subscribed type with no bound did NOT refuse ..."
    printf '%s\n' "$_o_out"
fi
```

and the `printf` in the failure branch — which prints that same `_o_out` — emitted:

```
-- basis= status=refused why=1 type(s) this image RECEIVES carry no derived bound, ...
```

`cat -A` confirms `status=refused` is present, contiguous, on one line, with no
control characters. **The captured variable contains the string that `grep -q`
reported absent.** Whatever this is, it is not the cmake run misbehaving: cmake
produced the right output and the shell then failed to find it in a variable it
was holding.

Note also `basis=` is empty in the failing output. Whether that is a second
symptom or the same one is not established.

## Hypotheses tested and ELIMINATED — do not redo these

- **Temp-dir collision between concurrent copies.** No: `tests/lib/common.sh:54`
  uses `TEST_TMPDIR="$(mktemp -d …)"`, unique per run.
- **`grep -q` exits early → `printf` takes SIGPIPE → `set -o pipefail` turns a
  MATCH into a non-zero pipeline.** This would explain everything (timing
  dependent, only large outputs, match present but reported absent), and the
  script does set `set -uo pipefail` at line 96. But it does not reproduce:
  with a 2 MB haystack under `pipefail`, `printf … | grep -q` returns 0 and
  `PIPESTATUS` is `0 0` over 200 iterations. Bash's builtin `printf` is not
  taking SIGPIPE here.
- **Fork/exec failure under load returning non-zero, which `if !` cannot
  distinguish from "not found".** No supporting evidence: no `Resource
  temporarily unavailable` or `cannot fork` in any log, `ulimit -u` is 249223,
  `ulimit -n` is 1048576, and 52 GB of the host's 62 GB was available.
- **A stale build artifact.** No: the gates pass solo immediately after failing,
  against the same tree.

## Worth considering

`if ! <cmd> | grep -q X` conflates "X is absent" with "the pipeline could not
run", and every one of these gates is written that way. Even if that is not the
cause here, an assertion that cannot tell those apart is the wrong shape for a
gate — the same argument `check-no-vacuous-tests` makes about a test whose only
effect is a print.

## Not a fix, but a mitigation that exists today

`NROS_GATE_JOBS` caps the parallelism (`scripts/build/run-gates-parallel.sh:38`,
defaults to `nproc`). Whether the cmake-spawning gates should be in a serial
group — the way `check.just` already has a `build-serial` lane — is a design
question, not established here.
