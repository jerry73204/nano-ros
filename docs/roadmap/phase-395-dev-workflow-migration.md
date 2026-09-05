# Phase 395 — migrating to the multi-agent dev workflow

**Status (2026-08-28). Plan, nothing landed.** The design and its measurements
live in [multi-agent-ci-workflow.md](../development/multi-agent-ci-workflow.md);
this is the ordered work to get there. Each wave is independently landable and
each one pays for the next. Waves 0–3 need no runner, no queue and no new
service — they are worth doing even if the rest is never adopted.

## Why, in one paragraph

Ten-plus agents each run a 40–90 minute local verification before pushing, they
duplicate each other's work, and the GitHub CI meant to catch the rest takes
hours because a 9.2 GB Zephyr SDK cannot fit in a 10 GB `actions/cache` on a
14 GB runner. Meanwhile `just ci` runs `NROS_TEST_SCOPE=native` and never
exercises the ~50 runtime cells that are host-executable and free.

## W0 — Measure before changing anything

Half a day, no code. Two numbers decide the rest of the sequence.

1. **What fraction of the tests in `just ci` actually need a fixture?**
   **Measured 2026-08-28: 89 of 163 test files (55%) call a fixture resolver;
   74 (45%) do not.**

   Method matters here — a first pass grepping for `build_*` returned 27%, and
   was wrong: `qemu_baremetal_main_e2e_binary` is a resolver too. The figure
   above is against the authoritative list of all 377 `pub fn` resolvers in
   `fixtures/binaries/`. Same mistake class as guessing a pattern instead of
   reading the source of truth; recorded so the next person does not repeat it.

   Caveat: file-level, not test-level. `rstest` cases are not statically
   countable, so a file needing one fixture counts the same as one needing ten.

   *Consequence for this plan:* 45% of test files are gated behind a
   precondition they do not need, so W2 is a real and immediate win — but it is
   not "most", so **W10 cannot be deferred as far as first written**. Roughly
   half the suite genuinely needs built artifacts, which keeps the fixture cache
   load-bearing rather than an optimisation.
2. **Wall-clock split of one PR run.** **Measured 2026-08-28, and it inverts
   the premise of this plan.** Nothing on GitHub takes hours:

   | workflow | median | state |
   | --- | ---: | --- |
   | `pr-checks` | **2.5 min** | **14 of 14 runs failing** |
   | `host-tests` | 3 min | failing |
   | `nightly` | 30–34 min | failing |
   | `images` | 8–13 min | succeeding |

   Every signal-bearing workflow is RED, and has been for days. Causes so far:

   - `pr-checks` — `check-subtree-guard` counting a recycled PGID rather than
     its own processes. Fixed; it failed only on the runner because 4 vCPUs
     running `check-fast` 32-way parallel churn PIDs hard enough for reuse.
   - `host-tests` — `ERROR: cannot locate rmw_zenoh_cpp/rmw_zenohd`. The CI
     container has no ROS zenoh router, so RFC-0075's resolution finds nothing.
     A provisioning gap in the image, not a code defect.
   - `nightly` — qemu and freertos jobs fail at ~30 min. Root cause **not
     pinned**; the log tail is teardown noise.

   **So the hours are not on GitHub — they are LOCAL.** Agents run the 40–90
   minute treadmill precisely because CI is red and tells them nothing, and a
   permanently-red required check is indistinguishable from no check at all.
   That is a cycle, and it is cheap to break.

**This re-orders the plan.** The bottleneck is not runner capacity or queue
latency; it is that CI carries no signal. Fixing the reds and keeping them fixed
comes before runners (W6), before the queue (W7), and before the cache (W10) —
because none of those help while every result is ignored. W5 (flake quarantine)
is effectively already underway: the first flake was found and fixed during W0.

Everything below assumed the hours were on GitHub. Re-read it with that
corrected.

## W0.5 — Make CI carry signal again

Inserted after W0.2 measured that every signal-bearing workflow is red. Nothing
downstream matters while results are ignored, and this is the cheapest wave in
the plan.

Five of six red jobs are root-caused and fixed; the sixth is diagnosed as far
as this host allows and filed. **Not one was a code regression** — every single one was wiring, provisioning, or a gate that could
not see its own rule. That is the finding, more than the individual fixes.

| job | defect |
| --- | --- |
| `pr-checks` | `check-subtree-guard`. **Still failing — my diagnosis was wrong.** I read it as PGID recycling and rewrote the check to require same-pid-AND-same-pgid; it still fails, which PROVES the survivors are genuine. Containerisation and CPU starvation are also ruled out (passes in `ubuntu:22.04`, passes at `--cpus=0.3`). The rewrite made the diagnostic honest and the hypothesis falsifiable, which is how the error surfaced. Tracked as [issue 0853](../issues/archived/0853-subtree-guard-fails-only-on-github-runner.md); needs the CI image to diagnose. |
| `host-tests` | `just zenohd setup` built the VENDORED router, deleted by RFC-0075 / phase-362. `zenohd` takes a LOCATOR, so `setup` was one — the step could never work. |
| nightly `qemu` | ran `test-wcet`, which deliberately refuses under emulation (no DWT counter). Red by construction. Also silently narrowed: `build-all`/`build-examples` did not exist for this module alone, so the `\|\|` chain fell through to the lightest build. |
| nightly `nuttx`, `threadx_linux` | a nextest `-E` filter cannot survive just's UNQUOTED variadic interpolation — `args=(-E test(Nuttx))` is a bash syntax error. The only two callers with parens; exactly the two failures. |
| nightly `freertos` | the platform job never installed cross targets; `armv8r-none-eabihf` (s32z270, Cortex-R52) was absent, so cmake configure died. |

Three of these were invisible to gates that exist for the class:
`check-just-recipe-refs` read `just/*.just` and never `.github/workflows/`
(widened — but it still cannot see `just ${{ matrix.plat }} build-all`, because
it skips any line containing `{{`); and the recycled-identity lesson was already
written down as `group_ledger::start_time()` but never applied to PGIDs.

**Then keep it green**: a red required check that persists for days is the
   condition this wave exists to prevent, and issue 0840's `pre-push`
   `check-fast` hook is the complementary half — it stops reds being *created*,
   this stops them being *tolerated*.

Only after CI is trustworthy does it make sense to spend on runners, queues or
caches — those reduce the cost of a signal nobody currently reads.

## W1 — Generate the issue index

**Blocker for batching, not a cleanup.** `docs/issues/README.md` is a
hand-maintained registry of files that already carry frontmatter. A merge queue
needs its batch to merge cleanly, so three agents filing issues concurrently
produce a textual conflict that stops the batch from *forming*. With ten
doc-heavy agents that is the common case.

- `scripts/gen-issue-index.py` emitting the open list from each file's
  frontmatter, in the established generated-page style (pool inventory, RMW
  matrix, support status).
- `check-issue-index` becomes a drift check against the generated output rather
  than a hand-comparison. It currently compares distinct ids to files, which is
  why `main` has carried **two open `#0824` rows** while reporting OK.
- The "Recently resolved" prose stays hand-written; only the open list is
  generated.

## W2 — Lane recipes, and redefine the local verb

The largest coverage gain available, and it needs no infrastructure.

- Define `just ci-l1` (affected crates), `just ci-l2` (the four
  host-executable platforms), `just ci-l3` (cross build + link + symbol),
  `just ci-l4-tier1`.
- **Redefine `just ci` as L0+L1+L2** and keep `just ci-full` for the
  pre-release sweep.
- **Update CLAUDE.md and AGENTS.md in the same commit.** CLAUDE.md currently
  instructs every agent to "run the TIER your change earns" and names
  `ci-matrix` / `ci-full`. Agents follow that faithfully — one session ran the
  full treadmill four times. If the verbs change without the instructions,
  agents keep paying the old cost and the queue re-tests what they already ran.
- Ship a minimal `CONTRIBUTING.md` here: run `just ci`, expect L0–L2, do not
  attempt the cross lanes.

`ci-l2` widens coverage from `native` to `Linux` + `ZephyrNativeSim` +
`ThreadxLinux` + `FreertosPosix`. `ZephyrNativeSim` builds with
`ZEPHYR_TOOLCHAIN_VARIANT=host`, so this needs no SDK.

## W3 — Make L3 a real lane

Today `rust-rtos-link-check` is the only cross-target gate. Two more witnesses
landed in phase 392 and are wired into nothing:

- `scripts/nros-mem-report.py --check` over cross ELFs — turns the static-RAM
  campaign into a gate instead of a manual measurement.
- `scripts/check-no-alloc-image.py` — the link-time allocation gate.

Together these catch 32-bit layout, linker/section, static-RAM and
allocation-freedom defects **without QEMU**. That is most of this project's
recurring embedded classes, at build cost.

Also here: the paths→coordinates map that makes lane selection a function of the
change rather than a fixed set. `row_coord()`, `NROS_TEST_COORDS` and
`CiLane::run_scope` already exist; the map does not.

## W4 — Hosted workflow files

`pr.yml` running L0/L1/L2 on `ubuntu-22.04`, replacing the PR half of
`pr-checks.yml`. Thin `just` callers per
[ci-workflow-reorg.md](../development/ci-workflow-reorg.md); fresh-clone
assumptions per [ci-conventions.md](../development/ci-conventions.md).

Path filters so an express change (docs, an index row) runs L0 and nothing else.
`pr-checks.yml` already has a `changes` job to model this on.

## W5 — Flake quarantine

**Must precede the merge queue.** A batch red is ambiguous between a defect and
a flake, and bisection multiplies the cost — one flake ejects and re-tests four
innocent PRs.

- A registry of quarantined tests: they still run and still record, they do not
  block.
- Automatic solo-retry before a batch red is believed.
- First entry: `action_raw_goal_ships_one_cdr_header` — 60 s timeout in-sweep,
  3.6 s solo, 5/5.

`_nextest-tolerant` and the skip budget are most of the mechanism already; what
is missing is the registry and the does-not-block half.

**Landed.** `.config/flake-quarantine.toml` + `scripts/test/quarantine.py`.
`--demote` rewrites a quarantined `<failure>` into `<skipped
type="nros:quarantine">` keeping the original text, wired into
`_rewrite-skipped-junit` before the `junit-real.xml` snapshot so every consumer
reads one account. `--check` refuses an expired entry, a non-open issue or a
missing field; expiry is a hard failure because quarantine without expiry is
deletion with extra steps. `just retest-failures-solo` produces the evidence an
entry must earn. 19 hermetic selftest assertions, run by the gate. First entry
is issue 0854.

## W6 — Runner scripts, and one runner

- `scripts/ci/runner-register.sh` — registration token via `gh api`,
  `config.sh --ephemeral --labels`, service install.
- `scripts/ci/runner-provision.sh` — make the labels true, reusing `nros setup`
  so a runner and a contributor provision identically.
- `scripts/ci/runner-doctor.sh` — refuse to register a runner that lies about
  its labels. A runner labelled `nros-sdk-zephyr` without the SDK produces a red
  that looks like a code failure.
- `scripts/ci/runner-sweep.sh` — reap orphaned process groups between jobs and
  own disk GC. One session found **71 orphaned `add_two_ints_server`**, oldest
  10 days; on a shared runner one leaked peer is every later job's flake.

Register one machine with `nros-qemu,nros-sdk-zephyr,nros-big`. Outbound 443
only, so NAT is fine. Ephemeral and unprivileged, because this is a public repo.

**Scripts landed; no machine registered.** All four exist and take `--check`.
The doctor is the label SSoT (executable and sourceable, so there is no second
copy of the vocabulary) and register runs it first, refusing a host that fails.
The service install was made OPT-IN (`--with-service`): it is the only sudo in
the four, and nothing in this repo sudos.

**Known gap: `--ephemeral` and a systemd service are not a complete pairing.**
The runner takes one job, de-registers and exits; the unit then restarts a
runner whose registration is gone. Something must re-register — a supervisor
loop or a timer re-running `runner-register.sh`, which is idempotent thanks to
`--replace`. The script prints this after every registration; the supervisor is
not written.

## W7 — The queue

- `queue.yml` on `merge_group`, self-hosted, running `ci-l3`, uploading logs
  unconditionally (a NAT'd runner cannot be reached).
- `post-submit.yml` on `push` to `main` running `ci-l4-tier1`.
- Branch protection with `strict: false` and required checks naming the
  **hosted** lanes only.
- Merge queue on: rebase, batch 4, min 1, 5 min wait, timeout above L3's p99.
- Partition by path so docs never queue behind a Cyclone build.

**Workflows and the apply script landed; the queue itself is NOT enabled** —
that is a repo-admin action and deliberately not an agent's.

`queue.yml` (on `merge_group`) and `post-submit.yml` (on push to `main`) exist.
`scripts/ci/enable-merge-queue.sh` / `just merge-queue` shows the plan and
changes nothing without `--apply`.

Three things this surfaced that the plan did not say:

* **A required check that can never START does not fail — it stays PENDING, and
  a merge queue waits on pending forever.** So making a self-hosted lane
  required before a runner exists does not make merging stricter, it stops it,
  and the symptom is a spinner that reads as GitHub being slow. Both the
  workflows and the script are interlocked on this: the self-hosted jobs are
  gated on `vars.NROS_SELF_HOSTED_READY`, and the script refuses
  `--self-hosted-ready` unless it can see an online runner advertising every
  needed label. Verified: it refuses today, rc 1, against the live API.
* **`cancel-in-progress` is correct for post-submit and wrong for the queue.**
  Post-submit asks "is main good now", so a newer commit subsumes an older
  answer. Cancelling a queue job makes the batch fail with no verdict, and
  GitHub then ejects PRs that were never actually tested.
* **`ci-l4-tier1` does not exist.** The plan names it; inventing its contents
  would be guessing at which cells belong in it. `post-submit.yml` runs tier 2
  (`ci-matrix`) meanwhile — a real cover, and honestly narrower than what the
  plan wants.

**`main` is governed by a RULESET (`main-rules`), not classic branch
protection**, and the two are separate systems that do not read each other —
`branches/main/protection` answers `Branch not protected` on this repo while
`main` is in fact protected. Ask `gh api repos/.../rulesets`.

In force now, targeting `refs/heads/main` only, with **`bypass_actors` empty** so
it binds admins too: `deletion`, `non_fast_forward`, `required_linear_history`.
That last one enforces an invariant the repo already held and had violated by
accident — three `Merge remote-tracking branch` commits from 2026-05-15. Direct
push still works; only the failure mode of an accidental merge changed, from
landing silently to being rejected.

Two corrections to earlier advice on this page, both in the ruleset's favour:

* **The merge queue needs no web-UI step.** A ruleset carries it as a rule type
  (`merge_queue`, with `merge_method` / `max_entries_to_build` /
  `min_entries_to_merge` / `min_entries_to_merge_wait_minutes` /
  `check_response_timeout_minutes` / `grouping_strategy`), so
  `enable-merge-queue.sh --apply --with-queue` sets it through the API. The
  "no REST API" note was true of classic protection only.
* **Required status checks are what end direct-push**, with or without a
  `pull_request` rule: a commit you are about to push has no check results yet,
  so the push is refused. That makes enabling them a workflow change for every
  agent rather than a setting, which is why the policy is written into
  AGENTS.md "Branch policy" first.

Verified under the active ruleset, not reasoned: pushing AND deleting
`refs/issue-ids/9999` both succeeded, so branch rulesets do not touch the custom
refs `just issue-new` and `just claim` depend on. Targeting `~ALL` instead of
`refs/heads/main` would have broken every agent's own `fix/<id>` branch and
every outside contributor on day one.

**Was blocked on issue 0853. FIXED and CONFIRMED — `pr-checks` run 33110917045
went green on `5d62867dd`, the first green PR gate on this repo. The reason it
blocked still generalises.** `check (fast on push;
full on PR/nightly)` is in the required set and is DETERMINISTICALLY RED on the
runner — `check-subtree-guard` fails on every GitHub run and passes everywhere
reproducible locally. Enabling the queue against an always-red required check
freezes merging exactly as surely as the always-pending case does; they are the
same failure wearing different clothes.

So the precondition for W7 is not "the workflows exist" but **every required
check must be able to go green on the runner** — now satisfied for the hosted
set. Two ways to have satisfied it, and only one was honest:

* fix 0853 — the guard exists to stop a killed build orphaning its descendants
  (issue 0762), and 71 orphaned `add_two_ints_server` processes, oldest 10 days,
  are what its failure looks like when nobody is watching; or
* narrow the required set to checks that are green, and say which coverage that
  gives up.

Quarantining it is NOT one of them: the flake registry is for a test that passes
solo, and this one fails REPRODUCIBLY in one environment. That is a defect, and
0853 says so. It was fixed the first way — the "survivors" were zombies, which a
GitHub container job's PID 1 (`tail -f /dev/null`) never reaps.

## W8 — Claims

`just claim` / `claim-renew` / `claim-release`, modelled on
`scripts/reserve-issue-id.sh` — arbitration at origin, unique object per
attempt, push-rejection as the CAS.

Beyond id reservation: a TTL in the object, renewal driven by **liveness**
rather than by the agent remembering, and stealing an expired claim with
`--force-with-lease=<ref>:<oid>`. An open PR supersedes the claim, so the TTL
governs only the window before first push — hours, not days.

Includes the predecessor check (a wave cannot be claimed while its predecessor
is claimed-and-unlanded) and the rule that agents check **both** claim refs and
GitHub issue assignees, because outside contributors cannot write refs.

## W9 — The contributor path

- Full `CONTRIBUTING.md` with the six-stage flow.
- `.github/PULL_REQUEST_TEMPLATE.md` asking which lanes ran, what could not be
  verified, and whether the work was AI-assisted — the last for review
  emphasis, not as a filter.
- `just doctor` gains a one-line answer to "what can I run?" for someone with a
  fresh clone.
- Require approval for outside collaborators; document that **enqueueing a fork
  PR is the trust decision**, because that is when its code reaches
  self-hosted hardware.

## W10 — Content-addressed fixture cache

Key on (input hash, coordinate) using `row_coord()`, shared on the runner's
persistent disk. This is the term multiplied by N agents, and it dissolves the
treadmill class: a rebase that does not change a fixture's inputs stops
invalidating it.

**Promoted ahead of putting L2 in the queue** (2026-08-28). The four
host-executable platforms own 309 of the manifest's 314 fixture rows, so an L2
pre-merge lane builds nearly everything. L2 is not affordable until this exists.

### Most of the machinery is already here

`fixtures/staleness.rs` already content-hashes: it hashes the artifact, collects
the input files, and stores a baseline of `<mtime> <size> <hash> <path>` per
input. The input set is not hand-maintained either — it comes from
`dep_file_newer_than_for(binary.with_extension("d"))`, the **compiler's own `.d`
dep file**, which is authoritative for what the compiler read.

### The blocker is the inputs the compiler cannot see

A `.d` file covers sources and headers. It does not cover the inputs this
tree's issue history is largely *about*:

| input | invisible because | issue |
| --- | --- | --- |
| a whole-archived `.a` behind `-Wl,…` | CMake cannot see a file inside a flag string | 0475 |
| env vars as build inputs | `rerun-if-env-changed` compares TEXT, and one directory has three spellings | 0491 |
| Kconfig knobs | reach the C lane and not the Rust one | 0460 |
| the CLI's own source closure | a textual `path =` walk was wrong in BOTH directions | 0627 |

For a **staleness probe** an incomplete set is survivable: it errs toward
rebuilding, and the code returns `None` rather than "fresh" when it examined
nothing, deferring to the stricter mtime verdict. That fallback is what makes
the current design safe.

For a **cache** there is no fallback. A hit skips the build, so an incomplete
key does not cause a redundant rebuild — it **silently serves a wrong
artifact**. That is the museum-binary failure mode with its one safeguard
removed, and museum binaries have cost this tree a bisect more than once.

### So: shadow mode first

Do not let a hit skip work until the key has earned it.

1. Compute the key. Build anyway. Compare the built artifact against what the
   cache would have served, and record every mismatch with the input that
   differed.
2. Run that way across a real spread of changes — a rebase, a Kconfig edit, an
   env change, a linker-flag change, a toolchain bump. Those are the four rows
   above; each one is a test the key has to pass.
3. Only when shadow mode is quiet does a hit get to skip a build.

This turns "is the key complete?" from an argument into a measurement, which is
the discipline that has already corrected this plan three times (CI was red not
slow; host-executable is not cheap; `check-build` is not a compile tier).

### What landed (shadow mode only, 2026-08-28)

`nros_tests::fixtures::cache_key` + the `fixture-cache-shadow` bin. **There is
no lookup verb and no restore verb**, so nothing here can make a build skip
work; step 3 above is deliberately not implemented.

* **Key** = `fnv1a` over a canonical preimage of `(row_coord() coordinate,
  provenance, every COVERED class witness, every measured input's content
  hash)`. The input set is the toolchain's own record, read through the shared
  readers now in `fixtures::staleness` (`dep_file_paths` for cargo's `.d`,
  `ninja_dep_paths` for `ninja -t deps`) — the same readers the freshness probe
  walks, so there is one reading of the dep graph rather than two.
* **Refusals, never degradation.** No dep record, an empty input set, an
  unreadable artifact, or a path that attributes to no row (or an ambiguous one
  — issue 0517) all REFUSE. A key over an unmeasured input set matches
  everything, which is the object this design exists to keep out of a cache.
* **Recording** is one file per observation under
  `target/nros-fixture-cache-shadow/` (gitignored by `**/target/`), atomically
  written; `NROS_FIXTURE_CACHE_SHADOW=1` turns the fixture resolvers into
  recorders, off by default.
* **Report**: per coordinate, observations / novel / predicted / correct /
  mismatches, plus every mismatch with the input that differed and the issue
  that predicted it. `--check` exits 1 on any mismatch.

Coverage of the four rows above, as the tool itself reports it
(`fixture-cache-shadow coverage`) — this table IS the "record explicitly that it
is not covered" half of the deliverable:

| class | issue | in the key? | why |
| --- | --- | --- | --- |
| `link-archives` | 0475 | **NO** | an archive under a build root is an OUTPUT; a key must be computable before the build. Covering it needs the archive's own inputs resolved transitively. Witnessed by hashing every `.a` under the artifact's build root. |
| `env-vars` | 0491 | **NO** | the recorder's env is not necessarily the build's env — that is 0491's point — and cargo's per-unit env fingerprints are an internal format. Witnessed with the declared names' values as seen by the recorder, labelled as such. Names come from `check-path-env-fingerprints.py --list-env-names`, the tree's one enumerator of BOTH producers. |
| `kconfig` | 0460 | **yes**, when observable | the resolved `<build>/[zephyr/].config` is the knob set and is one file; hashed into the key. `not-observable` (recorded per observation) when the artifact has none. |
| `cli-closure` | 0627 | **yes** | `nros source-stamp` over the GENERATED `cli-source-dirs.txt` closure, which is 0627's fix. Deliberately over-broad: a CLI edit invalidates every key, and over-broad is the safe direction. |

Two classes uncovered means **shadow mode is not close to done** — a hit must
not skip a build until a real spread of changes has been observed and each
uncovered class has actually been exercised. Read the `predicted` column before
believing a green report: a key that has never been re-seen has never been
tested.

Also measured while building this: **8 of 221 fixture artifact roots carry rows
at more than one coordinate** (33 of 256 rows — every native rust
talker/listener/service/action leaf, whose zenoh/xrce/cyclonedds rows all land
in `<leaf>/target` since the `target_dir` column was dropped).
`lane::attribute_path` fails closed on those per issue 0517, so path attribution
alone cannot key them; the caller names the coordinate (`--coord`), or the
resolver passes its already-selected `GroupRow`.

## W11 — Unblock issue 0726, then turn on gate fan-out

**The largest measured latency win left**, and it helps every stage: `check-fast`
runs on the pull request, on the merge group and on every push to `main`.

`scripts/build/run-gates-parallel.sh` exists and is measured: **90 s serial ->
~8.8 s at -P24**, because the 133 gates are 56 s of work spread over 90 s at
1–2 runnable cores. It is opt-in because one gate went red under fan-out and
green standalone.

**That cause is already found and fixed; the note keeping it off is stale.**
`scripts/lib/grep-q.sh` names it: under fan-out a forked `grep` can fail to
start (EAGAIN) or be killed, and `grep -q` cannot distinguish that from "no
match" — so the gate reported a missing anchor for an example that has one.
Green-to-red under load and never the reverse, which is the signature.
`check-rmw-force-link-anchor` now treats `rc >= 2` as fatal.

Measured 2026-08-28, 420 s of watching across three full fan-out runs: **0 file
transitions and 0 of 90,533 `git ls-files` short or errored** — so neither
standing hypothesis (a gate rewriting another's inputs, index contention) has
any support.

What remains is the CLASS, not the instance: **46 `grep -q` sites across 21
scripts reachable from a `check-fast` gate**, each the same latent conflation.
Convert them to `nros_grep_q`; the helper and `check-grep-q-error-conflation`
already exist. Then flip `check-fast` to the parallel runner and delete the
stale "conflicting pair unidentified" note.

The sweep was **two populations, not one**, because the first scoping stopped at
`scripts/`. The other 30 sites live in the seven gate scripts under
`packages/testing/nros-tests/tests/` that `check-{provider-index,build-root,
workspace-order,cargo-target-spelling,fixture-groups,package-xml-comments}`
invoke — all six are in the fan-out set, and issue 0732 had already pulled that
directory into scope after `workspace_order_gate.sh` announced a false finding
from a SIGPIPE. Both populations are now at 0 in `grep-q-baseline.json`.
Measured after the second pass: five fan-out runs (three at `-P24`, two at
`NROS_GATE_JOBS=64`), 133/133 gates green each time.

What the two passes did NOT convert is the **capture form** — `hits="$(grep …
|| true)"` followed by a test on emptiness. It carries the identical
conflation, `check-grep-q-error-conflation` cannot see it (no `-q`), and there
is no helper for it because the caller wants the LINE, not a status. Four sites
in `build_root_derivation.sh` fail SILENT that way (an errored grep reads as
"no literal remains"); the two that fail LOUD — a false "the shadowed provider
was not reported" and a false "the case never ran" — were split by hand at the
call site. A `nros_grep_lines` sibling would be the shared fix.

## W12 — `check-dep-chain` out of the merge path

158 s measured, the single most expensive gate in the compile tier after the
backend lanes. It is an 8-cell board×rmw matrix (`nros setup --dry-run` +
codegen + `cargo tree` per cell) — a MATRIX LANE wearing a gate's clothes, and
its own header still claims it runs "in seconds", which was true per cell and
stopped being true as cells accumulated.

No single pull request changes the board×rmw wiring, so paying it per merge buys
little. Move it to post-submit, where a regression is still bounded to one
commit.

## W13 — Delete `queue.yml`'s L1 job

`pr-checks`'s `check` now runs the compile tier on `merge_group`, and it covers
strictly more than `just ci-l1`. Both run on every merge group, so the same tree
compiles twice on the critical path to landing. `queue.yml` keeps its L3 job,
which is gated on a self-hosted runner and does something different.

## W14 — A claim liveness supervisor

`just claim-renew` is idempotent and cheap, and nothing drives it. A claim
therefore lapses during exactly the work it is meant to protect — a long fixture
build, a slow QEMU sweep — and another agent can legitimately steal live work,
which is worse than no claim at all because the steal LOOKS sanctioned.

The supervisor must key on the agent PROCESS being alive, not on progress
between steps: 40 minutes inside one build is not death. `reserve-claim.sh`
already prints that instruction on every successful claim; what is missing is
something that obeys it.

## W15 — Separate the SUPPORT tier from the BUILD cost (landed)

`check-board-tiers` certified tier 1 while the only lane covering tier 1 was
failing, because one predicate — `tier 1 => a nightly lane covers it` — binds a
PROMISE to a lane NAME, which is a scheduling artifact. A lane can be red and
the tier still certifies; restructuring CI breaks tier claims for no substantive
reason; and a platform that could be covered MORE often gets no credit for it.

**Rust's structure, adopted.** Their tier IS the CI obligation, stated as a
guarantee: tier 1 "guaranteed to work" = tests RUN in CI; tier 2 "guaranteed to
build" = builds, no runs; tier 3 = no CI. And the cost axis is a MODIFIER —
`Tier 2 with host tools` vs `without` — not a parallel taxonomy. That is exactly
our host-executable/cross-only split, and we were about to invent a second table
for it.

**Zephyr's two additions.** A tier names WHO commits and how often (tier 0 blocks
PRs; tier 1 is a named team running the suite regularly), and ONE TIER-0 PLATFORM
PER ARCHITECTURE. We already had that finding — RISC-V as "the witness that the
platform layer is not accidentally ARM-shaped" — this makes it a rule.

**Both projects refuse to auto-demote on a red, and the first draft of this had
it wrong.** Demoting a Rust tier-1 target needs a full RFC, and the policy
explicitly permits temporarily disabling a target's tests WITHOUT demoting it.
The maintainer obligation is worded "should", not "must". The reason is worth
keeping: auto-demotion on red is precisely the pressure that makes people
silence tests. A tier is a promise between people — it changes when someone
decides, with a record, not as a side effect of a bad night.

Landed:

- `execution_class` on every tiered row: `host-executable` / `cross-run` /
  `hardware`. It states what the platform NEEDS, so a tier is never a promise
  with no stated cost.
- **tier 1 requires `host-executable`**, because only that class can afford a
  per-merge runtime cadence. Both predicates mutation-proved.
- **`NuttxArm` and `FreertosMps2` demoted 1 -> 2.** Both are cross-run, so their
  runtime evidence can only ever be nightly — which is tier 2's promise — and
  nightly is currently red for exactly those platforms. The equally legitimate
  alternative is buying the cadence with a self-hosted runner and promoting them
  back on evidence; what is not legitimate is claiming the tier without it.

The relationship, stated rather than implied: **the class bounds the cadence you
can afford; the cadence bounds the tier you can honestly claim.** The class never
CONFERS a tier — `FreertosPosix` is host-executable and free to run and a
two-cell smoke, so it stays tier 2 and the way up is more cells.

## W16 — tier 1's lane must cover tier 1's platforms (DESIGNED + MEASURED, not landed)

Investigated to a decision point and deliberately NOT shipped half-done; the
partial change was reverted so the tree stays green. Four findings, each
measured, and the first two overturn the original plan.

**1. `ci-l2` as "run the host-executable group" is the wrong shape.** The
host-executable platforms are **309 of 424 fixture coordinates — 73 %**
(`linux` 196, `zephyr` native_sim 68, `threadx-linux` 43, `freertos-posix` 2).
So `host-executable` means **needs no SDK**, NOT **few fixtures**; a lane
running all of it per merge would cost close to running everything. The
affordable shape is the greedy set COVER the lane machinery already computes.

**2. Name-based filtering cannot express it, so there is no cheap path.** The
matrix binaries' rstest case names are MIXED: `Platform__ThreadxLinux` carries
the variant, `Platform__Freertos` carries only the family — it cannot separate
`FreertosPosix` (host) from `FreertosMps2` (cross-run). Same finding as issues
0357/0482 for tier 2, verified here by listing real case names.

**3. So W16 is not a new lane — it is widening `CiLane::Tier1`.** Its pool was
`PlatformId::Linux` alone, which made the lane narrower than the support tier:
`board-support.toml` grants tier 1 to `ZephyrNativeSim` and `ThreadxLinux` too,
and tier 1 promises runtime evidence every merge.

**4. The pool is not what drives the cover — the SPEC is.** Widening the pool
alone changed nothing: the cover stayed 10 coordinates and Linux-only, because
`spec(Tier1)` is 1-wise(Workload, Kind) + pairwise(Lang × Rmw) with **no
Platform axis**, so a platform that satisfies no requirement is never chosen.
Adding `Axis::Platform` to the singles gives **12 coordinates covering all three
tier-1 platforms** — two extra cells to keep the promise:

    linux,{c,cpp,rust}×{cyclonedds,xrce,zenoh} + linux,mixed,zenoh
    threadx-linux,c,zenoh
    zephyr,rust,zenoh

**What stops it landing today, and why that is not a detail.** Tier 1's RUN is
name-scoped (`NROS_TEST_SCOPE=native`) and its BUILD is the `native` MODULE —
and `lane-filter.sh native` excludes the `zephyr` and `threadx` tokens, so the
new cells would be filtered out of the run while their fixtures went unbuilt.
Shipping the cover alone produces exactly the mass "Binary not found" failure
`fixture-lane.sh` documents. The remaining chain:

  * `tier1_is_native_only` -> an invariant over the tier-1 PLATFORM SET, ideally
    derived from `board-support.toml` so the lane and the registry cannot
    disagree (the discipline `check-flavour-lanes.py` already establishes for
    the `nostd` lane);
  * the documented cell table, 16 -> 17;
  * `lane-filter.sh`: a tier-1 scope that INCLUDES the tier-1 platforms rather
    than excluding everything non-Linux;
  * `nros_lane_build_lane`: tier 1 stops mapping to the `native` module;
  * a COLD measurement of the resulting fixture build — zephyr native_sim and
    threadx-linux fixtures are the new cost and it is unmeasured.

That last item is the gate on the whole thing, and it is the same caution as
before: every timing to hand is warm on a 24-core host, and warm numbers have
produced two wrong answers in this campaign.

## W16.a — `ci-l2` (superseded by W16)

Tier 1 promises runtime evidence every merge, and **no lane delivers that today**
— the host-executable group's ~50 runtime cells run only in the nightly sweep.
`ci-l2` is the lane that would keep the promise: `ZephyrNativeSim` builds with
`ZEPHYR_TOOLCHAIN_VARIANT=host`, plain host gcc and no SDK, so three RTOS APIs
fit on a hosted runner.

**Measure it COLD before any tier promise depends on it.** Every timing to hand
is warm on a 24-core host, and warm local numbers have already produced two
wrong answers in this campaign — both because state survived from an earlier
build. A tier-1 cadence promised on a warm number is that same error where
retracting it costs much more.

## W17 — `just tier-health`: report the evidence, do not gate on it

`check-board-tiers` is in `check-fast`: offline, deterministic, no network. It
can check that a tier's obligation is STRUCTURALLY met and cannot know whether
the evidence is GREEN — and it must not imply otherwise, which is the gap that
let a half-red nightly certify tier 1.

So a separate verb queries CI and reports which tier each platform's evidence
actually supports. It does not gate, per the demotion policy above. Same split as
`check-*` versus `merge-queue --readiness`.

## W18 — scope precisely; a skip is not a selector

A skip is a RUNTIME answer to "can this host run this test". Scope is a
SELECTION answer to "is this test mine to run". Using the first where the second
belongs is what makes "the lane passed with 200 skips" unreadable — and today
`[SKIPPED:lane]` is the dominant class (13 of 16 emission sites).

**Where the scope IS name-expressible, a lane skip is imprecision.** The test was
selected, started, and discovered it was never wanted. Fixing the selector
removes both the skip and the work.

**Where it is NOT, the resolver IS the selector**, and this is not a defect to
design away. Tier 2 is 1-wise over platform, so every platform is in the lane and
the real narrowing is lang×rmw WITHIN a platform — which test names do not
encode (0357/0482, confirmed 2026-08-28: `Platform__Freertos` cannot separate
`FreertosPosix` from `FreertosMps2`). The selection therefore has to happen where
the test↔fixture binding physically exists, and its output is reported through
the only channel available, a skip.

So the actionable half is REPORTING, not re-architecting:

  * `lane` is a DESELECTION — by design, nobody acts on it.
  * `capability` / `resource` are provisioning gaps — the only classes anyone
    can act on.

`check-skip-budget` now reports them separately (`N ran, M deselected,
K skipped for an unmet precondition`) instead of one total, because burying the
second in the first makes a lane with a real hole read exactly like one that
merely narrowed its scope.

The remaining structural work is W16's: tier 1's run is name-scoped
(`NROS_TEST_SCOPE=native`), which is the coarse end of this. Making it
coordinate-scoped would let the lane select precisely instead of selecting
broadly and deselecting at the fixture — but that is the same change W16
measured and did not land, for the same reason.

**Not measured:** how many lane deselections a real tier-1 run actually emits.
No representative junit was to hand, so the claim that tier 1's name scoping is
coarse rests on reading the filter, not on counting its output.

## W19 — one derivation for "which platforms does this tier cover"

**The root the last three work items were symptoms of.** Three places answer
that question independently, and they disagree — verified 2026-08-28:

| source | says tier 1 is |
| --- | --- |
| `board-support.toml` (the promise) | `Linux`, `ZephyrNativeSim`, `ThreadxLinux` |
| `ci_lane.rs` `pool(Tier1)` (what is selected) | `PlatformId::Linux`, hardcoded |
| `lane-filter.sh native` (what may execute) | Linux only — it **excludes** `zephyr` and `threadx` |

So the lane covers one of the three platforms the tier promises, and the run
filter actively removes the other two. W15 (a tier claiming more than CI
delivers), W16 (the pool narrower than the tier) and W18 (scope coarser than
selection) are three faces of that one split.

**Target: one derivation, three consumers** — the shape
`check-flavour-lanes.py --print` -> `lane-filter.sh nostd` already has, whose own
comment states the reason: *one derivation, used by the gate and by this lane,
so they cannot disagree*.

`board-support.toml` is the source of truth for (platform, tier,
execution_class). `matrix_platform` is already spelled exactly as the
`PlatformId` variant, so no name mapping is needed or invented.

**Landed:** `check-board-tiers.py --print-tiers` emits
`platform<TAB>tier<TAB>execution_class`. Additive, no behaviour change, and it
is the keystone the rest hangs off.

**Landed (2026-08-28): tier 1 now works like tier 2 — coordinate-scoped run AND
build.** That is a better target than the inclusion-union plan below, and the
reason is the Zephyr caveat: no token spelling separates `ZephyrNativeSim` from
`ZephyrQemuCortexM`, so a name filter could not express tier 1 without guessing.
Coordinates express it exactly, and phase-340 W3 had already made the same move
for tier 2 for the same reason.

  * `pool(Tier1)` — the three tier-1 platforms, with a new test
    (`tier1_pool_matches_board_registry`) asserting the list EQUALS
    `--print-tiers`. Hardcoding without that assertion is precisely how the two
    drifted apart; the test is the SSoT enforcement.
  * `spec(Tier1)` — gains `Axis::Platform`. Without it the greedy cover is
    driven by requirements the pool cannot influence and stays Linux-only
    however wide the pool gets. Cover: 10 -> 12 coordinates, 16 -> 17 cells.
  * `run_scope(Tier1)` — `LaneCoords`, not `Native`. The name filter EXCLUDED
    `zephyr` and `threadx`, the very platforms tier 1 promises.
  * `just ci` — exports `NROS_TEST_COORDS` like `ci-matrix`, enforced by
    `recipes_run_the_scope_their_lane_declares`, which failed the moment the
    declaration changed and the recipe did not.
  * `nros_lane_build_lane tier1` — maps to ITSELF, not the `native` module. The
    old mapping had become actively wrong: `native` builds no zephyr native_sim
    or threadx-linux fixtures, and `lane-coords tier1 --modules` now returns
    `native threadx_linux zephyr`.

174 lib tests green; the three coupling tests each failed first and had to be
satisfied, which is what makes the change trustworthy rather than merely
compiling.

**STILL UNMEASURED, and it gates the promise:** what a tier-1 fixture build now
costs. It gained the zephyr native_sim and threadx-linux modules, and every
timing to hand is warm on a 24-core host. Nothing here should be read as "tier 1
is affordable per merge" until that is measured COLD.

**The remaining steps, in dependency order:**

1. **`lane-filter.sh tier1`** — an INCLUSION union over the tier-1 platforms
   read from `--print-tiers`, replacing `native`'s exclusion list. It is the
   step that stops the run filter contradicting the registry.

   **Viability, checked rather than assumed.** Tier 1 is name-expressible where
   tier 2 is not, and the reason is specific: `rtos_e2e`'s `Platform` enum is
   `{Freertos, Nuttx, ThreadxLinux, ThreadxRiscv64}`, so `ThreadxLinux` carries
   its variant in the case name, and the one family that does NOT split
   (`Freertos`) is tier 2 on both its variants — so the ambiguity never reaches
   tier 1's inclusion set.

   **The remaining care is Zephyr.** Its binaries distinguish by NAME
   (`zephyr` vs `zephyr_cortex_m_qemu`), so a bare `~zephyr` inclusion would
   drag the cortex-m binary in; the union needs `and not test(~cortex_m)` or
   equivalent. And two binaries are not obviously either variant —
   `cli_bringup_zephyr`, `qos_zephyr_ros2_interop_e2e` — so which side they
   belong on must be READ, not guessed. That is the same overlap hazard
   `lane-filter.sh`'s `nostd` arm already documents ("the QemuBaremetal token is
   qemu, and tests/nuttx_qemu.rs matches it"), and it is why this step is small
   but not trivial.
2. **`pool(Tier1)` reads the same table.** Rust cannot read the TOML at compile
   time, and the house answer to exactly that is a GENERATED committed file with
   a staleness gate — `nros-rmw-cffi/src/generated.rs` (`check-abi-bindings`)
   and `cli-source-dirs.txt` (`check-cli-source-dirs`) are the precedents. So:
   generate the tier table into Rust, gate the drift.
3. **`spec(Tier1)` gains `Axis::Platform`** — measured in W16: without it the
   greedy cover is driven by requirements the pool cannot influence, and stays
   Linux-only no matter what the pool holds. With it, 12 coordinates cover all
   three platforms.
4. **The build lane follows** — `nros_lane_build_lane` maps tier 1 to the
   `native` MODULE, which does not contain zephyr or threadx-linux fixtures.
   This is the expensive step and the one W16 stopped at.
5. **Cold-measure it** before tier 1's per-merge promise depends on the result.

Steps 1–3 are mechanical and independently verifiable. Step 4 changes `just ci`
for everyone, and 5 is the gate on believing any of it.

## W20 — the design rule: a required check must be UNCONDITIONALLY reported

Four times this campaign has frozen merging, and they are not four bugs. They
are one design defect: **a required status check that fails to produce a verdict
blocks forever, and GitHub cannot tell "not applicable" from "not yet".**

| # | shape | what it looked like |
| --- | --- | --- |
| 1 | always RED | issue 0853 — the gate could not pass on the runner |
| 2 | always PENDING | a self-hosted required check with no runner registered |
| 3 | cannot REPORT | the check had no `merge_group` trigger |
| 4 | never TRIGGERED | a `paths:` filter the PR's files did not match |

**The mechanism, from GitHub's own docs, and it is the whole rule:**

> If a workflow is skipped due to path filtering, branch filtering or a commit
> message, then checks associated with that workflow will remain in a *Pending*
> state. A pull request that requires those checks to be successful will be
> blocked from merging.

and, decisively:

> If a job within a workflow is skipped due to a conditional, it will report its
> status as **Success**. It will not prevent a pull request from merging, even if
> it is a required check.

So a skipped **workflow** blocks forever; a skipped **job** passes. GitHub states
the conclusion outright: *"You should not use path or branch filtering to skip
workflow runs if the workflow is required."*

### The rule

**If a check is REQUIRED, its workflow triggers unconditionally on every event
where it is required, and ALL conditionality moves to job-level or step-level
`if:`.** Cost control is not given up — it moves one level in, where a skip
reports success instead of pending.

**If a check is NOT required, trigger-level filtering is fine and cheaper**,
because nothing waits on it. `nightly` and `host-tests` keep their filters.

### What this validates and what it condemns

Already correct, by instinct rather than by rule: `queue.yml`'s L3 job gates on
`vars.NROS_SELF_HOSTED_READY` at JOB level, so with no runner it skips and
reports success. Had that been a trigger condition it would have been failure
mode 2 with extra steps.

Condemned: `pr-checks.yml` on `main` still path-filters `pull_request`. PR #16
touches only `ci/docker/**`, which the filter does not list, so its required
check has NEVER RUN — and it carries the fix for the ROS environment gap that is
failing PR #6's merge group. Two pull requests, each blocked on the other, and
`bypass_actors` is empty by design so there is no override.

### Enforced

`enable-merge-queue.sh` refuses to make a context required when its workflow
path-filters a required event, naming the remedy (move it to a job-level `if:`).
Proved against the real state on `main`: rc 1.

## Not in scope

- Replacing GitHub's merge queue with a third-party tool. The native one has
  batching and ejection; partitioning is `paths:` filters. Revisit only if
  measurement shows the queue is the bottleneck.
- Remote *execution* (as opposed to caching). Much larger, and the cache is
  where the measured win is.
- Auto-revert automation before W7 has run long enough to trust culprit
  attribution.

## Exit criteria

Stated so this is falsifiable rather than a one-way door:

- If median time-to-land does not fall below today's local-verification time,
  the queue is not paying for itself — go back to direct pushes.
- If batch reds are more often flakes than defects, W5 failed; stop batching.
- If the self-hosted runner becomes a single point of failure for landing,
  move L3 back to hosted and accept a narrower L3.

## Issues homed here (survey 2026-09-03)
Every open issue was checked for a home phase; these had none, or were
mentioned here only in passing. A mention is not an owner — an issue with
no work item is an issue nobody is accountable for, which is the same shape
as a gate sitting in a lane no CI job runs. Each row is a work item: the issue
holds the evidence, the item is *close it*.

| issue | why it belongs here |
| --- | --- |
| [#0895](../issues/0895-format-walks-unbuilt-colcon-workspace-leaves.md) | `just format` is red or green depending on whether a migrated colcon workspace has been BUILT |
| [#0925](../issues/0925-box-sync-copies-generated-manifests.md) | `ros2-box-sync.sh` copies the GENERATED workspace manifests, so the box and the host disagree |
| [#0957](../issues/0957-format-blocked-by-unexcluded-workspace-leaf.md) | `just format` fails whole-tree — a workspace leaf is in neither the root members nor the excludes |
| [#0986](../issues/0986-pre-push-hook-corrupts-the-repo-it-guards.md) | the pre-push hook writes into the repository it is guarding |
| [#0988](../issues/0988-hook-scripts-never-exercised-under-a-hook-environment.md) | no gate runs a hook the way git runs it — with `GIT_DIR` set — so a hook can pass every check and still fail in git's hands |

