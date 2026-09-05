---
id: 1077
title: "`just check fast` flakes on the gates that spawn cmake, and each flake reads as a different real bug"
status: resolved
type: bug
area: tooling, testing
severity: medium
found: 2026-09-05
related: [0876, 0445, 0726, 0732]
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

Note also `basis=` is empty in the failing output. **Answered: that is not a
symptom at all.** `_nros_bounds_join_subscribed` initialises `<out_basis>` to
`""` and sets it only on the `closure` arm, so an empty `basis=` beside
`status=refused` is exactly what a correct refusal prints. The captured text
was right in every byte.

## Hypotheses tested and ELIMINATED — do not redo these

- **Temp-dir collision between concurrent copies.** No: `tests/lib/common.sh:54`
  uses `TEST_TMPDIR="$(mktemp -d …)"`, unique per run.
- ~~**`grep -q` exits early → `printf` takes SIGPIPE → `set -o pipefail` turns
  a MATCH into a non-zero pipeline.**~~ **THIS ELIMINATION WAS WRONG — it is
  the cause.** See the Resolution below for the trace and the counts. The
  2 MB probe that cleared it tested the wrong regime twice over: the hazard
  needs a match followed by *another line to write*, and it scales with the
  number of LINES after the match, not with the size of the haystack. The real
  haystack is 339 bytes over three lines and reproduces at 60 of 3000 calls.
- **Fork/exec failure under load returning non-zero, which `if !` cannot
  distinguish from "not found".** No supporting evidence: no `Resource
  temporarily unavailable` or `cannot fork` in any log, `ulimit -u` is 249223,
  `ulimit -n` is 1048576, and 52 GB of the host's 62 GB was available.
- **A stale build artifact.** No: the gates pass solo immediately after failing,
  against the same tree.

## RESOLUTION — the writer takes SIGPIPE and `pipefail` calls it the verdict

**The pipe is the bug. Not cmake, not the temp dirs, not fork pressure.**

Bash's builtin `printf` flushes **per LINE**, and `grep -q` exits at the
**first match**. So whenever the needle is on any line but the last, grep can
close the read end while `printf` still has a line to send. `printf` then takes
SIGPIPE, and `set -o pipefail` promotes that **141** to the *pipeline's* status
— so `if ! … | grep -q …` takes its failure branch on a **successful match**.

Traced under `strace -f -e status=failed`, on the real assertion:

```
3202021 write(1, "    demo/msg/Open (unbounded)\n", 30) = -1 EPIPE (Broken pipe)
3202021 --- SIGPIPE {si_signo=SIGPIPE, si_code=SI_USER, …} ---
3202021 +++ killed by SIGPIPE +++
```

and with `PIPESTATUS` captured before anything could clobber it (the first
instrumented run read `[0]` only because `rc=$?` is itself a command and resets
the array):

```
ITER 2157 left=141 right=0     # writer SIGPIPE'd, grep MATCHED
```

Deterministic once you know it — no load, no concurrency:

```sh
set -o pipefail
{ printf 'status=refused\n'; sleep 0.2; printf 'second line\n'; } | grep -q "status=refused"
# rc=141, PIPESTATUS=(141 0), three times out of three
```

### Why every observation fits

| observation | explanation |
| --- | --- |
| the variable *contains* what `grep -q` said was absent | grep found it; the writer died afterwards |
| four gates, four different assertion messages | all four are the same `printf … \| grep -q` idiom |
| the *same* gate failing *different* assertions | two consecutive assertions, same shape, independent races |
| needs concurrency, not load | the window is the writer's next `write(2)`; contention widens it |
| green when run alone, immediately after | the writer usually wins |
| `basis=` empty | correct output for a refusal — never a symptom |

Block O was the **only** site in `cmake-message-bounds-tests.sh` using a pipe;
its other ~40 assertions use `<<<` or a file, and none has ever flaked. The
assertion that *cannot* flake is the `status=derived` one — that output is a
single line, so there is nothing left to write after the match.

### Measured

| | pipe | here-string |
| --- | --- | --- |
| primitive, 3-line haystack, idle host | **60 / 3000** (2.0%) | **0 / 5000** |
| `cmake-message-bounds-tests.sh`, 12 concurrent copies | **13 / 300** | **0 / 300** |
| same, 6 concurrent copies | 6 / 396 | 0 / 240 |

### The fix

The repo already had the right helper and the wrong scope. `nros_grep_q`
(`scripts/lib/grep-q.sh`, issue 0726) gives the 0 / 1 / `exit 2` contract that
stops "absent" meaning the same thing as "could not run" — and
`check-grep-q-error-conflation` has kept new conflations out since phase-395
W11. But its `SEARCH_ROOTS` are `scripts, just, justfile, packages, tools`:
**`tests/` is not in scope**, which is why these four call sites still carried
the raw idiom. Issue 0196's rule, again.

Converted to `nros_grep_q <pattern> <<<"$text"` — helper for the status
question, here-string for this one:

- `tests/cmake-message-bounds-tests.sh` (3 sites, `message-bound-knobs`)
- `tests/cmake-reconfigure-tests.sh` (5 sites, `reconfigure-on-change`)
- `tests/fixture-staleness-probe-tests.sh` (2 sites, `fixture-staleness-probes`)
- `tests/cmake-support-library-tests.sh` (5 sites — same shape, same lane, not
  yet observed flaking; `nm … | grep -q` has a far wider window than `printf`)
- `scripts/check-image-paths-apply-policy.sh` (3 sites,
  `image-paths-apply-policy`). Its two hot sites fail in *opposite* directions:
  `… || continue` silently **skips** a candidate file, `if … then continue`
  turns a compliant file into a **named violation**.

**Using the helper is not enough on its own.** `printf … | nros_grep_q PAT`
has the identical race, and worse: a pipeline element is a subshell, so the
helper's `exit 2` ends only that segment. `scripts/lib/grep-q.sh` now says so
in its header.

### The gate

`check-pipefail-sigpipe-assertions` (`scripts/check-pipefail-sigpipe-assertions.py`,
on the `fast-serial` line) forbids a pipeline whose **last stage is a matcher
predicate that can stop reading before EOF** (`grep -q`, `grep -m N`,
`nros_grep_q`) in a position where its **status is read**, in a file that
enables `pipefail`. It is the sibling of `check-grep-q-error-conflation`, not a
duplicate: that gate is about the exit *status* and skips any line naming
`nros_grep_q`; this one is about the *pipe*. It scans `*.sh` plus the `just`
recipe bodies, self-tests both directions on nine shapes (including the literal
line from this issue, and the fix, which must not trip it), and carries a
**content-addressed** ratchet — keyed on the line text, not a line number, so an
insertion above a listed site does not turn into a red naming the wrong thing.

29 pre-existing sites in 22 files are allowlisted: same shape, outside the lane
where this was measured. Sweep them with the gate itself
(`python3 scripts/check-pipefail-sigpipe-assertions.py` after deleting a line).

## Still open, recorded here rather than lost

- **`check-grep-q-error-conflation` does not scan `tests/`.** That is the
  coverage gap that let these four sites keep the raw idiom for as long as they
  did. Adding `tests` to its `SEARCH_ROOTS` would baseline **115 further sites
  across 7 files** — real 0726 debt, none of it this bug (they are here-strings
  and file greps, exposed only to the status conflation). Deliberately not done
  here: that gate's own docstring makes lowering its baseline a reasoned act,
  and folding a 115-site expansion into a bug fix would bury it.

- **A SECOND, INDEPENDENT MECHANISM in the same lane, not yet filed.**
  `packages/testing/nros-tests/tests/fixture_group_collision_gate.sh` proves
  `check-fixture-groups` fires by **perturbing tracked files in place** — it
  appends a `[[bin]] name = "tripwire_collider"` to two real
  `examples/qemu-arm-baremetal/rust/*/Cargo.toml`, runs the gate, and restores
  from an EXIT trap. For those ~2 seconds the working tree carries a bogus
  binary name, and **20 other `check-fast` gates read leaf `Cargo.toml` files**
  (`check-example-leaf-target-dirs`, `check-leaf-lockfiles`,
  `check-cargo-config-tracked`, `check-std-census`, …). This is exactly the
  "something transiently rewrites what that gate reads" that
  `run-gates-parallel.sh`'s own header warned about and never identified.

  Stated as a structural hazard read off the code, **not** as the cause of a
  specific red: a `check-fixture-groups` failure naming `tripwire_collider` was
  seen once during this work, but another agent was editing the same checkout
  concurrently, so it is not attributable. It wants its own issue and its own
  measurement — a gate that mutates the shared tree should stage a copy, not
  edit in place.

## Not needed after all

`NROS_GATE_JOBS` and a serial group were the standing mitigations. Neither is
required: the flake is a property of the assertion idiom, not of the
parallelism, and it reproduces at `-P1` given a writer that is descheduled at
the wrong moment. Nothing about the fan-out changed.
