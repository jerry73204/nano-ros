# phase-330 — the SystemModel becomes a build artifact (implement RFC-0063)

**Implements:** [RFC-0063](../design/0063-system-model-is-a-build-artifact.md)
**Closes:** issue 0380 (and retires its transitional guards)
**Touches:** phase-296 (model as sole plan input), issue 0320 (content-addressed
staleness), RFC-0047 / RFC-0052 (scheduling dims + fail-loud contract)

**Status.** IN PROGRESS — W1 landed 2026-08-02 (wave 1).

## Goal

The committed `config/system_model.yaml` disappears. The model is generated per
build into `build/` in the colcon manner, readable for inspection, authored by
nobody. The user maintains three inputs and only these: the **launch file**, the
**project config**, the **system config**.

## The one ordering constraint that matters

The committed models carry hand-authored `execution.tiers` dims that the inputs
**cannot currently express**. So:

> **The inputs must gain the dims BEFORE any committed model is deleted.**

Reverse that order and the migration reproduces issue 0380 across nine
workspaces at once instead of two. W1 gates everything after it; there is no
useful partial progress on W3–W5 without it.

## Work items

### W1 — Give the dims a resolver input (BLOCKS EVERYTHING)

- [x] **W1.a** Extend the system-config schema (`nros_orchestration_ir`) with
      per-platform scoped tier dims: `zephyr.deadline_us`,
      `nuttx.budget_us`/`period_us`, `threadx.preempt_threshold`/
      `time_slice_us`, per-platform `core`, and the generic `class`. The
      resolver already carries `posix.core` / `sched_class` — this widens the
      same table rather than inventing a second one.
- [x] **W1.b** ~~Teach the resolver to read them~~ — **NOT NEEDED.** The
      resolver parses this same `[tiers.*]` block into the shared
      `ros_launch_manifest_sched::TierPlatformSpec`, which has carried `core` /
      `deadline_us` / `budget_us` / `period_us` / `time_slice_us` all along.
      The gap was entirely on the nano-ros side: two mirrors of one concept
      drifted, and the NARROWER one (`nros_orchestration_ir::TierRtosSpec`)
      defined what users could write. No fork change, so no maintainer push is
      in the critical path — the risk this phase recorded does not apply.
- [x] **W1.c** Round-trip acceptance, using the tooling issue 0380 already
      landed: for `ws-realtime-rust`, delete the committed model, re-resolve,
      and assert `nros ws model-dims` returns the SAME 20 dims. Equivalently:
      `nros sync` must not refuse, because there is nothing left to drop.
      **This is the falsifiable definition of "the inputs can express it".**
      **PASS (2026-08-02)** for `ws-realtime-rust`: dims moved into
      `system.toml`, model deleted, `nros sync` exits 0 (nothing to drop) and
      `nros ws model-dims` returns the same 20 dims.

### W2 — Prove the round-trip across the family

**The census is bigger than "the realtime workspaces".** Measured 2026-08-02:

| Set | Count |
| --- | --- |
| Tracked `*/config/*model.yaml`, repo-wide | **120** |
| …in `examples/workspaces/` | 80 (36 `system_model.yaml` + 44 variants) |
| …in STANDALONE copy-out examples | **24** |
| …in `packages/` (test fixtures) | 16 |
| Models carrying `execution.tiers` dims | **11** (86 dims) |

The 11 dim-carrying models are the *migration risk*; the other 109 are the
*migration work*. Both have to move — every one of them is a derived artifact
committed under `src/`.

- [x] **W2.a** (superseded by W2.b's 121-model sweep, which covered all 11 —
      see the status note below) Round-trip W1.c across the 11 dim-carrying models: the nine
      `ws-realtime-*` workspaces plus the two `orchestration_tiers_{native,
      freertos}` test fixtures. `scripts/model-dims-baseline.txt` is the oracle.
- [x] **W2.b** Round-trip the remaining models — **DONE (2026-08-02).** All
      **121** committed models regenerated and the diffs classified. **Zero**
      failed to regenerate.

      | Class | Count |
      | --- | --- |
      | byte-IDENTICAL | **117** |
      | drops `target:` (issue 0356) | 1 |
      | provenance only (`sha256`/comments) | 1 |
      | ADDS `meta.inputs` provenance | 2 |

      **The answer: regeneration never loses data. Every difference is the
      COMMITTED copy being stale**, from three distinct causes:

      1. `ws-realtime-cpp` — its deploy layer still carries pre-0356
         `target: linux` because commit `07025368b` RESTORED it by grafting the
         block out of git history (the #380 rescue) rather than re-resolving.
         Hand-restoration reintroduced old content.
      2. the two `orchestration_tiers_*` fixtures — last touched 2026-07-24,
         before `meta.inputs` provenance was emitted, so regeneration ADDS it.
      3. `ws-realtime-cpp-mps2` — `sha256`/comment churn only.

      Omitting `target` is the DOCUMENTED contract, not a loss: `Deploy.target =
      None` means board-agnostic — "a multi-board system runs the same nodes on
      every board, so the consuming entry's own board decides (nano-ros issue
      0356)". Both workspaces declare one `kind = "self"` plus FOUR
      `kind = "embedded"` blocks, which is exactly that case. The fork history
      confirms it is deliberate: `69c13d2 chore: bump rlm — multi-board
      placement is board-agnostic (nano-ros #356)`.

      **A correction to the W2.a note above:** the earlier report that
      `ws-realtime-c` drops `target` was produced by the STALE resolver. With
      the rebuilt one it is byte-identical. Two of this phase's observations
      have now been distorted by that stale binary — re-measure after
      `just setup-launch-resolve`, not before.

      **CORRECTION (2026-08-02, from the W4.a dry run): the conclusion below
      was too strong.** `nros sync` SKIPS a model that is not stale
      (`if !stale(&model) && provenance.is_none() { continue; }`), so re-running
      sync over an INTACT tree leaves most models untouched — "byte-identical"
      there means "never re-resolved", not "reproduced". The real test is
      regeneration with the committed copy ABSENT, and under that test 13 of 76
      bringups lose content. W2.b's finding holds for what it actually measured
      (nothing got WORSE on refresh; four committed copies are stale), but it is
      not evidence that regeneration is lossless.

      **So `resolve(inputs)` is total for the structure and deploy layers.** The
      only thing regeneration could not reproduce was the execution dims, which
      W1 fixed. That is the premise RFC-0063 needs, now measured rather than
      assumed.

**W2.a is complete at 11/11.** The sweep regenerated every dim-carrying model;
the four non-identical diffs touch `target:`/`sha256`/comments/`inputs` only —
no `execution.tiers` line moved in any of them.
- [x] **W2.c** (all gaps recorded, none special-cased: `[[model]]` declarations
      for arg-bound variants, `[[component]] params`/`params_files` in the
      resolver fork — shipped in the rlm tags — and `ws-sizing`'s legacy
      `structure.topics` classified inert, see W4.a) Any dim that cannot round-trip is a W1 schema gap, not an
      exception to grant. Record it; do not special-case it.

**W2.a status (2026-08-02): 11 of 11 — see W2.b's sweep, which supersedes this.**
`ws-realtime-rust` (wave 1), `-c`, `-c-mps2`, `-cpp-fvp`, `-cpp-rclcpp`,
`-cpp-subnode`, `-cpp-subnode-portable`. Not yet run: `ws-realtime-cpp`,
`-cpp-mps2`, and the two `orchestration_tiers_*` fixtures.

The dim gap was far smaller than assumed: **9 of 11 already carried their dims
in `system.toml`**; only 5 dims across 2 workspaces needed authoring, and they
were exactly the fields W1.a enabled.

Two findings:

1. **The mps2 failures were a STALE RESOLVER, not a schema gap.** They failed
   with "node '/ctrl_node' is not placed — with multiple [deploy.*] blocks every
   node needs a `nodes = [..]` entry", yet the fork's source already contains the
   fix, in a comment naming `ws-realtime-c-mps2` and `ws-realtime-cpp-mps2`
   explicitly. `just setup-launch-resolve` rebuilt it and `-c-mps2` passed. This
   is issue 0363 C's lagging-resolver class, live.

2. **Regeneration is not byte-identical, and the committed models look STALE.**
   Re-resolving `ws-realtime-c` and `-cpp` drops `execution.deploy.<node>.target:
   linux` (and the long doc comments, which is expected — those moved into
   `system.toml` with the dims). The dim gate does not see this: it watches
   `execution.tiers` only. Either the committed copies predate a resolver
   placement change, or regeneration loses deploy content — W2.b must answer
   which BEFORE W4 deletes anything. Restored to committed state pending that
   answer, rather than folded into an unrelated commit.

### W3 — Relocate the artifact

- [x] **W3.a** DECIDED 2026-08-02 — see "The W3.a decision" below.

      ~~Decide the location.~~ This is the phase's central open question,
      and it is wider than the model — **the model is not the only derived
      artifact living in `src/`**:

      | Artifact | Today | Tracked? |
      | --- | --- | --- |
      | `<pkg>/config/*model.yaml` | in `src/` | **committed** ✗ |
      | `<pkg>/metadata/*.json` | in `src/` | gitignored |
      | `<pkg>/generated/<msg crate>/` | in `src/` | gitignored |

      Two of the three are already gitignored, so the pattern is HALF-MIGRATED:
      the model is simply the one that got committed. Whatever `build/` means
      should mean the same thing for all three, or the next artifact repeats
      issue 0380.

      Options:

      1. **Workspace-level `build/<pkg>/…`** — matches colcon, matches the
         framing that motivated this RFC. One root to delete.
      2. **Per-package output subtree** — smallest diff, keeps relative paths
         short, but leaves derived output interleaved with sources, which is
         the shape being retired.

      **`generated/` is what makes option 1 non-free**: each leaf's
      `.cargo/config.toml` redirects msg crates by RELATIVE path
      (`std_msgs = { path = "generated/std_msgs" }`, RFC-0048 W9, `nros
      sync`-managed). Moving the tree rewrites 110 redirects and lengthens
      every one; issue 0378 is a live reminder that when those redirects are
      wrong, resolution falls through to a third party's crate on crates.io.
      Sequence `generated/` deliberately, or scope W3 to the model and leave
      the other two where they are with a recorded reason.
## The W3.a decision (2026-08-02)

**There is no single directory, and looking for one was the wrong question.**

Every consumer derives the model the same way today — `<bringup source
dir>/config/system_model.yaml`:

| Consumer | How it resolves the path |
| --- | --- |
| `nros-macros` (proc-macro) | `pkg_index.resolve_pkg(bringup).join("config/system_model.yaml")` |
| `nros-build` (build-script lib) | `bringup_dir.join("config/system_model.yaml")` |
| `cmake/NanoRosEntry.cmake` | takes `MODEL <path>` explicitly, defaulting to the same |

So the relocation is not "move a file"; it is **"change the anchor from the
bringup's SOURCE dir to the active build's OUTPUT dir"** — and each build
system already has such a dir:

- **CMake** — `CMAKE_BINARY_DIR`. `NanoRosEntry` is already parameterized
  (`MODEL <path>`), so this side needs a default change, not a redesign.
- **Cargo** — `OUT_DIR`. An entry crate's `build.rs` resolves the model into
  `OUT_DIR`; the proc-macro reads `OUT_DIR` at expansion. Idiomatic, and it is
  how generated code normally reaches a macro.
- **Workspace-level `build/`** — what `nros build` (RFC-0065) owns, for the
  case where several entries share one bringup and should share one
  regeneration.

**The decision: the model is generated into the ACTIVE BUILD's output dir, and
consumers RECEIVE its path rather than deriving it.** The workspace-level
`build/` is then an optimization (shared regeneration across entries), not the
mechanism — which is what makes W3.c answerable at all.

Consequences worth stating:

- `OUT_DIR` is per-CRATE while a model is per-BRINGUP, so two entry crates
  sharing a bringup each regenerate it. Correct but wasteful; the workspace
  build root is the fix when a builder owns it, not a reason to reject
  `OUT_DIR`.
- `metadata/` and `generated/` should follow the same rule for the same
  reason — but `generated/`'s 110 msg redirects are RELATIVE paths in each
  leaf's `.cargo/config.toml`, so it moves on its own schedule (issue 0378 is
  the reminder of what a wrong redirect costs).

### W3.c — standalone copy-out examples: ANSWERED

**24 of the 120 committed models live in standalone copy-out trees**
(`examples/<platform>/<lang>/<example>/config/system_model.yaml`), which by
RFC-0026 have no workspace to walk up to.

Under the decision above they need nothing special: `OUT_DIR` and
`CMAKE_BINARY_DIR` exist for a single-package build exactly as they do inside a
workspace. That is the argument FOR anchoring on the per-toolchain build dir
rather than a workspace-level `build/` — a workspace-relative answer would have
forced copy-out examples to keep a committed model, splitting the contract in
two.

- [x] **W3.b** SEAM LANDED 2026-08-02 (the default has NOT flipped — that is W4).

      `nros_orchestration_ir::model_location` is now the ONE definition of the
      search order, and all three consumer families call it:

      | Order | Location | Set by |
      | --- | --- | --- |
      | 1 | `$NROS_MODEL_DIR/<name>` | RFC-0065's builder, for a shared regeneration |
      | 2 | `$OUT_DIR/nros/<name>` (cargo) / `${CMAKE_BINARY_DIR}/nros/<name>` | the active build |
      | 3 | `<bringup>/<model_rel>` | the committed copy — still authoritative |

      Landable on its own precisely because nothing generates into (1) or (2)
      yet, so every consumer resolves exactly as before. Verified both ways on
      `ws-realtime-rust`'s `native_entry`:

      * fallback — builds clean with no override (rc=0);
      * override — with `NROS_MODEL_DIR` pointing at a model whose `low` tier
        was removed, the build fails with "callback group `telem_node/telem`
        names tier `low`, which has no `[tiers.low]` definition". That error is
        only reachable if the proc-macro read the OVERRIDE, which is the
        positive proof the seam redirects rather than the negative proof that
        nothing broke.

      Not-yet-verified: the cmake branch is exercised only by a fixture build,
      which was not run here. Its logic mirrors the Rust order and is guarded by
      `if(EXISTS ...)`, so its failure mode is "keeps using MODEL as passed".

      Remaining for W4: something must GENERATE into (2), and the ten consumers
      below stop needing the committed copy. Ten files resolve the path today:

      | Consumer | Why it is awkward |
      | --- | --- |
      | `cmd/ws.rs`, `cmd/plan.rs`, `cmd/codegen_system.rs`, `cmd/codegen.rs` | ordinary CLI paths |
      | `nros-build/src/lib.rs` | build-script library — runs before the build dir is conventional |
      | `nros-macros/src/main_macro.rs` | **a proc-macro** reads the model at expansion time; it has no build-dir context |
      | `cmake/NanoRosEntry.cmake` | cmake must agree with the CLI on the location |
      | `scripts/build/compile-check-fixtures.sh` | fixture staging |
      | `examples/workspaces/ws-realtime-rust/src/threadx_entry/src/main.rs` | an EXAMPLE source references it |

      **Not started.** Ordering: phase-331's W2–W3 fold should land first (it
      deletes 18 workspaces and 29 of these models), and W3.b must stay outside
      phase-331's W1→W5 measurement window. The cmake half is small — the
      argument already exists; the cargo half needs a `build.rs` seam per entry
      crate plus the proc-macro reading `OUT_DIR`, which is the real work.

- [x] **W3.c** ANSWERED — see above. 24 of the 120 models are in standalone
      copy-out trees, and under the W3.a decision they need no special case:
      `OUT_DIR` / `CMAKE_BINARY_DIR` exist for a single-package build too. The
      remaining question is not WHERE but WHO RUNS THE RESOLVER in a copied-out
      tree — `build.rs` can invoke it, but `nros-launch-resolve` must then be
      on the copied-out machine (`just setup-launch-resolve`). Sequence that
      with W3.b.

### W4 — Delete the committed models

**DONE 2026-08-03 (W4.a executed with W5 + W7 in one batch).** The 13
"losses" reduced to zero (see the reclassification below + issue 0392's
resolution), `nros sync` now DEFAULTS `--model-dir` to
`<ws>/build/nros/models/` (namespaced by bringup DIR name — the same key the
consumer ladder uses), the ladder gained the workspace-build-root and
self-bringup rungs so macro-only entries resolve with NO env wiring, and all
112 tracked models were deleted. `check-no-tracked-models` (check-fast) holds
the invariant.

`nros sync --model-dir <DIR>` writes resolved models there instead of into each
bringup's `config/`, closing the loop with W3.b's search order. Proven
end-to-end on `ws-realtime-rust`, positively and negatively:

* generate into `build/nros/`, DELETE the committed model, build `native_entry`
  with `NROS_MODEL_DIR` → **rc=0**;
* the same build without the override → fails with "SystemModel not found",
  naming the committed path.

So a build can run from a generated model alone. What still blocks W4.a:

- [x] **W4.0 — DONE 2026-08-02. The variant set now comes from inputs.**

      Classifying all 120 models showed 112 were already derivable and 8 were
      not:

      | Class | Count | From |
      | --- | --- | --- |
      | `system_model.yaml` | 76 | `default_launch` |
      | `<stem>_model.yaml` | 36 | a non-included launch file |
      | **binding** (`multihost_robot{1,2}`) | **8** | a launch ARGUMENT — not derivable |

      **The 8 were the whole problem, and they were worse than "not derivable".**
      `multihost.launch.xml host:=robot1` is a different system from
      `host:=robot2`, and the only machine-readable record of which bindings
      matter was `meta.args` INSIDE each committed model — the very artifact W4
      deletes. `sync` read it back to replay the binding (issue 0364), so
      deleting the models would have destroyed the instructions for
      regenerating them.

      So the rule is derive-plus-declare:

      * **derive** — every launch file that is neither the default nor
        `<include>`d by another gets `<stem>_model.yaml`. Includes are pulled in
        by their parent's resolve; baking one separately would treat a fragment
        as a system (`ws-launch-rust`'s `sensors.launch.xml`).
      * **declare** — `[[model]] { launch, out, args }` in `system.toml`, now a
        real field on `SystemToml`. Migrated the 8 mechanically out of their own
        `meta.args`.
      * a launch file WITH declarations is fully described by them, so
        declarations REPLACE derivation for it — otherwise `multihost`'s unbound
        resolve gets baked too, and its `all` default leaves nodes the deploy
        blocks cannot place (that failure was observed, not theorised).

      **Verified with the committed `config/` MOVED ASIDE** — the set is
      reproduced from inputs alone, and the CONTENT matches:

      | Workspace | Generated | Committed | Non-`sha256` diff lines |
      | --- | --- | --- | --- |
      | `c` | 7 | 7 | **0** |
      | `cpp` | 7 | 7 | **0** |
      | `mixed` | 7 | 7 | **0** |
      | `rust` | 9 | 9 | **0** |
      | `ws-launch-rust` | 1 | 1 | — (include correctly skipped) |

      The only differences are `meta.inputs[].sha256` for `system.toml`, which
      MUST move because `system.toml` is what gained the declarations.

      Known over-generation, accepted: `n9_workspace`'s `sim.launch.xml` is
      neither included nor declared, so the rule bakes a `sim_model.yaml` that
      has no committed counterpart. It is a leftover from the
      `nros::main!(launch = …)` form that phase-296 R4 removed — referenced only
      by archived phase docs. As build output rather than a committed file, one
      spurious model costs a resolve and nothing else; deleting the dead launch
      file is the real fix and belongs with phase-331.

      ~~The VARIANT SET must become derivable from inputs.~~ `nros sync`
      decides which variant models to refresh by SCANNING `config/` for
      existing `*_model.yaml`, so those committed files are not just the
      artifact — they are the DECLARATION of which variants exist. Delete them
      and the defaults still regenerate (that target is unconditional) while
      every variant silently stops.

      Measured across 76 bringups: 70 are exactly 1:1 launch↔model; the four
      large workspaces are `launch + 1` (each launch file's variant plus the
      default's `system_model.yaml`); and **two have a launch file with
      deliberately NO model** — `ws-launch-rust`'s `sensors.launch.xml` and
      `n9_workspace`'s `sim.launch.xml`, both alongside a `system.launch.xml`.
      So "one model per launch file" is nearly right and would over-generate
      for those two, which look like includes rather than entries. The rule
      needs either include-analysis (the resolver already records included
      files in `meta.inputs`) or an explicit declaration in `system.toml`.

- [x] **W4.0b — DONE 2026-08-02. One env var, both halves, zero call-site churn.**

      `nros sync` now falls back to `NROS_MODEL_DIR` when `--model-dir` is
      absent, so the variable that tells consumers where models are READ is the
      variable that tells sync where to WRITE them. The alternative was
      threading `--model-dir` through **15** `just` call sites across
      native/freertos/zephyr/nuttx/threadx/esp32/px4 — a second spelling of one
      fact in fifteen places.

      **Wiring the cmake path surfaced four consumers W3.b had missed**, each
      re-deriving `join("config/system_model.yaml")` independently:
      `cmd/plan.rs` (twice — dir input and the cmake seam's file input),
      `cmd/codegen_system.rs`, and `cmd/ws.rs`'s bridge-bringup branch. `nros
      plan` is what the cmake workspace seam actually shells out to, so the C
      workspace failed to CONFIGURE until it went through `model_location`.
      W3.b's claim to have wired "all three consumer families" was wrong: there
      were seven sites, not three.

      Proven end-to-end on `examples/workspaces/c` (7 models — `system_model`
      plus 6 variants) with the committed `config/` MOVED ASIDE:

      * `NROS_MODEL_DIR=… nros sync` → all 7 written to the build dir, committed
        tree untouched;
      * `cmake -S . -B build/cm -DNANO_ROS_PLATFORM=posix` → **rc=0**;
      * `cmake --build build/cm` → **rc=0**;
      * control, same configure with NO env var → **rc=1**, "has no committed
        SystemModel".

      So both build systems can now run with zero committed models. W4.a stays
      blocked on W4.0 alone (the variant SET is still declared by the committed
      files).

**W4.b (issue 0320's staleness text) is deliberately deferred** until the models
actually move — editing it now would describe a state the tree is not in.

> **Coordinate with phase-331 (RFC-0066) — added 2026-08-02.** That phase folds
> 18 themed workspaces into the four large ones and deletes them. **29 of the
> 120 models below live inside those 18**, plus 10 workspace-root
> `CMakeLists.txt`. Let phase-331 W2–W3 land FIRST: the census then drops to 91
> and this phase does not migrate files that are about to be deleted. Nothing
> here blocks phase-331 — W1 is already landed, and none of the 18 folded
> workspaces carries an `execution.tiers` dim, so their re-resolve is free of
> the issue-0380 hazard.
>
> **Do not run W3–W4 between phase-331's W1 baseline and its W5 re-measure**:
> moving model generation moves fixture build wall-clock, and W5's delta would
> then mix two causes.

- [x] **W4.a — EXECUTED 2026-08-03 (`c6535536e`): all 112 tracked models
      `git rm`'d, regeneration verified for both the workspace and standalone
      classes, `check-no-tracked-models` gates the ban.** The blockers below
      resolved first: the SYNCFAIL class closed as issue 0392 (fixed
      2026-08-02), the resolver-side gaps shipped in the rlm/resolver fork
      tags (consumed at v0.1.4), and the known-stale committed-copy diffs
      (`scope`, issue-0356 `target`) died with the committed copies. The
      measurement record below is kept as the evidence trail.
      ~~BLOCKED. Re-measured 2026-08-03 against the post-phase-331
      layout; do not delete yet.~~ The census is now **111 models / 55
      bringups** (was 120/76 — phase-331 deleted 15 themed workspaces). A fresh
      dry run (move `config/` aside, regenerate from inputs, compare parsed
      YAML, restore) gives:

      | Result | Count |
      | --- | --- |
      | OK — reproduced exactly | **38** |
      | DIFF | 9 |
      | SYNCFAIL — sync errors before producing anything | 8 |

      Of the 9 DIFF, 14 of 17 individual diffs are the known-stale
      `structure.nodes.<n>.scope` (rlm documents it as a launch-file id, not a
      namespace; nothing in nros reads it). The three that are not:
      `features/` `param_sources: len 2 vs 1` (the REGENERATED side is richer —
      the committed model predates the fork merge, so it is a stale commit, not
      a loss), `sizing/` `structure.topics` (the one legacy block in the tree,
      spelled `publishers`/`subscribers` where `TopicWiring` emits `pub`/`sub`,
      so it does not deserialize into the current schema at all), and
      `orchestration_e2e` `meta.args` (a no-op binding — `talker_name`'s default
      IS `talker`, so the structure is identical and only the provenance record
      differs; that workspace uses a root `nros.toml`, so `[[model]]` does not
      apply to it).

      **The 8 SYNCFAIL are the real blocker** and are unchanged in kind from
      issue 0392: a bringup that cannot sync cannot regenerate what W4.a
      deletes.

      ~~BLOCKED. Measured 2026-08-02; do not delete yet.~~

      W4.0 and W4.0b are done, so I ran the deletion as a DRY RUN over every
      bringup: move `config/` aside, regenerate from inputs alone, compare set
      and content, restore. 76 bringups:

      | Result | Count | Meaning |
      | --- | --- | --- |
      | **OK** | 51 | reproduced exactly |
      | **RICHER** | 4 | generated has MORE — the committed copy is stale |
      | **LOSS** | 13 | committed content the regenerate does NOT reproduce |
      | **SYNCFAIL** | 24 | sync errors before producing anything |

      **UPDATE 2026-08-02 — 13 became 3. Ten were measurement artifacts, and one
      of those was a bug in this phase's own code.**

      The sweep compared TEXT LINES, which cannot tell reformatting from data
      loss. Re-comparing PARSED YAML (`tmp/semantic_cmp.py`, provenance
      stripped) reclassifies them:

      | Cause | Count | Real? |
      | --- | --- | --- |
      | `structure.nodes.<n>.scope` / `scopes./` | 18 diffs | **no** — committed is stale |
      | `execution.deploy.<n>.target` | 4 | **no** — issue 0356 removal |
      | list indentation (`  - x` vs `    - x`) | — | **no** — formatting |
      | flat `--model-dir` COLLISION | 1 bringup | **no** — my bug, fixed |
      | genuinely hand-authored content | **3** | **YES** |

      `scope` is the biggest and the most clearly stale: rlm documents the field
      as "STRUCTURAL — the launch file the scope corresponds to … NOT a
      namespace", so the current `scope: system.launch.xml` is right and the
      committed `scope: /` predates it. Nothing in nros reads `.scope`.

      **The collision was mine.** `--model-dir` wrote every model FLAT, so a
      workspace with two bringups (`ws-lifecycle-cpp`: `demo_bringup` +
      `managed_bringup`) had one `system_model.yaml` silently overwrite the
      other — which presented as "regeneration dropped a node". Output is now
      namespaced `<dir>/<bringup>/<model>`, and `model_location` searches the
      namespaced path before the flat one.

      **The 3 that remain are real, and are the W1 pattern:**

      | Bringup | Hand-authored content absent from every input |
      | --- | --- |
      | `ws-params-rust` | `params_files` + a second `param_sources` entry |
      | `ws-qos-rust` | node `params` (`qos_overrides.…`) |
      | `ws-sizing-rust` | `structure.topics` |

      Each was verified against the launch file: `ws-qos-rust`'s launch declares
      NO `<param>`, yet its model carries a `qos_overrides` param;
      `ws-params-rust`'s launch declares the inline param (which resolves) but
      not the file source. So these were typed into the artifact, exactly like
      the 17 tier dims of issue 0380.

      **RESOLVED 2026-08-02 (2 of 3 in the fork; the third was not a gap).**

      | Bringup | Outcome |
      | --- | --- |
      | `ws-qos-rust` | FIXED — `[[component]] params` |
      | `ws-params-rust` | FIXED — `[[component]] params_files` |
      | `ws-sizing-rust` | NOT A GAP — see below |

      The fork gains `[[component]] params` / `params_files`, projected onto
      `structure.nodes` by `apply_params_to_nodes` — a structure-layer entry
      point beside the execution-layer `apply_to`, exactly where `[lifecycle]
      autostart` already lives. Launch values win over declared ones (the launch
      description is the more specific statement), unmatched component names are
      diagnostics rather than errors, and `ComponentBlock` has no
      `deny_unknown_fields`, so an OLDER resolver ignores the new keys instead
      of failing — which is why the declarations can land here before the fork
      does.

      Verified by re-resolving both workspaces against their committed models:
      the only differences left are the known-stale `scope` (and, for
      `ws-qos-rust`, committed `execution: {}` versus an omitted empty map —
      the same value).

      **`ws-sizing-rust` is not a resolver gap.** It is the ONLY model in the
      tree with `structure.topics`, and it spells the endpoints `publishers` /
      `subscribers` while `TopicWiring` renames them to `pub` / `sub`. Those
      keys therefore do not deserialize into the current schema at all — the
      block is already inert data, not content the resolver fails to produce.
      Fixing it means regenerating it in the current spelling or dropping it,
      not teaching the resolver anything. Left for whoever owns
      `executor_sizing_e2e`.

      **Fork branches are committed but NOT pushed, and the superproject pointer
      is deliberately NOT bumped** — it would name commits no one else can
      fetch. Branch `nros-330-component-params` in both
      `ros-launch-resolve` (`9499034`) and its nested `ros-launch-manifest`
      (`17ba4af`), ready to fast-forward.

      ~~Fixing them means teaching the resolver to accept the declarations —~~
      per-node `params`/`params_files` and `topics` from `system.toml` — which
      is a change to the vendored `ros-launch-manifest` fork, the same shape as
      `75d57f59b`'s `[param_services]` fix. Left for a session that can push the
      fork branches.

      ~~**The 13 LOSS cases are the blocker**, and they are the W1 pattern again:~~
      content only a human put there. `lifecycle_autostart: active`
      (ws-lifecycle-{cpp,rust}), resolved parameter tables (ws-params-rust), and
      fields in the safety/sizing/qos demos. Deleting would silently drop them
      exactly as issue 0380 dropped 17 tier dims. Each needs the W1 treatment —
      become expressible in `system.toml` — before its model can go.

      **SYNCFAIL triaged 2026-08-02 → issue 0392. It is 12, not 24** — the
      sweep script emitted two lines per failure (an old and a new format), and
      I read the line count as the case count.

      | Cause | Count | Verdict |
      | --- | --- | --- |
      | probe walked up one level too far | 3 | **not a bug** — sync is `rc=0` in the right dir |
      | `@NANO_ROS_ROOT@` unsubstituted template | 3 | **not a bug** — templates are materialised before use |
      | legacy `[system]` schema | 3 (2 files) | **real** — 0392 A |
      | component declares no `class` | 2 | **real or negative fixture** — 0392 B |
      | launch needs an uninstalled package | 1 | environment — 0392 C |

      So six are real W4.a blockers, not 24: a bringup that cannot sync cannot
      regenerate its model. Detail, evidence and the "do not migrate blind"
      warning on the FreeRTOS descriptor are in
      [issue 0392](../issues/archived/0392-sync-blocked-fixtures.md).

      Original item: **W4.a** Remove all **120** tracked `*/config/*model.yaml` (80 under
      `examples/workspaces/`, 40 under test fixtures); add the build location
      to `.gitignore`.
- [x] **W4.b** (2026-08-03 — `meta.inputs` kept for inspection/staleness of the BUILD copy; provenance-as-promise retired with the committed files) Update issue 0320's staleness text: a per-build artifact does not
      need `meta.inputs[]` sha256 provenance any more than an object file needs
      it. Decide whether `meta.inputs` stays for inspection or goes.

### W5 — Retire the transitional guards

- [x] **W5.a** (2026-08-03) Remove the `nros sync` dim-loss refusal (`prior_model_dims` +
      the drop check) — it protects a committed file that no longer exists.
- [x] **W5.b** (2026-08-03 — `nros ws model-dims` kept) Remove `check-model-dims`, `scripts/model-dims-baseline.txt` and
      the `check-fast` wiring. Keep `nros ws model-dims`: inspection is a
      REQUIREMENT of RFC-0063, not a leftover.
- [x] **W5.c** (same change as W4.a, as specified) Do W5 in the SAME change as W4. A guard removed early leaves the
      models unprotected; a guard left behind fails on files that no longer
      exist.

### W7 — Input-addressed entries: `launch =` replaces `model =` (2026-08-03)

**Decision (maintainer, 2026-08-03): the model is TRANSPARENT to users.** A
user writes a launch file and a `system.toml`; the build system carries them
end-to-end to the image. The resolved SystemModel is an intermediate artifact
— interested users find it under the build dir; nobody references it, and
NOTHING under `src/` commits it. This closes the last visible seam W3/W4 left:
entries still NAME the artifact (`nros::main!(model = "bringup[:file.yaml]")`),
which is what keeps the artifact user-facing.

**Sequencing: W7 lands BEFORE W4.a.** Entries must stop naming models before
the committed models vanish, or the deletion breaks every entry site.

The input contract — the user-facing coordinates of a system are
`(bringup, launch file, launch args)`:

```rust
nros::main!(launch = "demo_bringup");                         // default_launch
nros::main!(launch = "demo_bringup:rust_safety.launch.xml");  // named launch
nros::main!(launch = "demo_bringup:multihost.launch.xml", args(host = "robot1"));
```

```cmake
nano_ros_entry(NAME robot1_entry
    LAUNCH demo_bringup:multihost.launch.xml ARGS host=robot1 ...)
```

This is NOT the phase-296-deleted parse path returning: no consumer parses
launch XML. The `launch =` spelling is resolved to a model by the BUILD layer
and the entry consumes the model from the build dir — the W3.a decision,
completed:

- **Cargo**: `nros-build` (the entry's build.rs) maps `(bringup, launch,
  args)` → model path: `$NROS_MODEL_DIR/<bringup>/<stem>_model.yaml` when the
  workspace build (nros sync) already resolved it, else invoke
  `nros-launch-resolve` into `OUT_DIR` (the standalone copy-out path — the
  resolver is a declared setup artifact, W3.c). The macro reads the model
  path from build.rs via env, never a source-tree path.
- **CMake**: `nano_ros_entry(LAUNCH <bringup>:<file> [ARGS ...])` resolves at
  configure into `${CMAKE_BINARY_DIR}/nros/models/<bringup>/`, dependency-
  tracked on the launch file + `system.toml`; passes the generated path where
  `MODEL` flowed before. `MODEL <path>` stays as the expert override.
- **Variants**: an entry with `args(...)` must match a `[[model]]` declaration
  (W4.0 derive-plus-declare) — the declaration is the SSoT for which bindings
  exist; the entry's `args` select one.

- [x] **W7.a** (2026-08-03) `nros-macros`: accept `launch = "<bringup>[:<file>]"` +
      `args(k = "v", ...)`; deprecation-warn on `model =` (one release, then
      remove). Internally both converge on "resolve to a model path via
      nros-build".
- [x] **W7.b** (2026-08-03 — the ladder covers macro-only entries via the workspace/self build-root rungs; nros-build tracks inputs) `nros-build`: the launch→model mapping above (env from the
      workspace build, OUT_DIR resolve fallback), emitting
      `cargo:rerun-if-changed` for the launch file + system.toml so edits
      re-resolve.
- [x] **W7.c** (2026-08-03) `NanoRosEntry.cmake`: `LAUNCH`/`ARGS` keywords; configure-time
      resolve into the binary dir with dependency tracking; keep `MODEL` as
      override; case-normalize per the cmake pitfall rule.
- [x] **W7.d** (2026-08-03 — 40 Rust + 68 cmake sites) Migrate the ~90 in-tree entry sites `model =` → `launch =`
      (mechanical; the 8 multihost entries read their binding from the
      existing `[[model]]` declarations). First case:
      `safety/src/zephyr_rust_safety_entry` (today
      `model = "demo_bringup:config/rust_safety_model.yaml"` →
      `launch = "demo_bringup:rust_safety.launch.xml"`).
- [x] **W7.e** (2026-08-03 — `check-no-tracked-models` in check-fast; the `model =` source ban follows the deprecation window) Gate the contract: `check-fast` fails on any tracked
      `*_model.yaml` (the no-committed-models invariant) AND on any
      `model =` in a non-test entry after the deprecation window.
- [x] **W7.g — the compile-check FORM GENERATOR also names models** (found
      2026-08-03, handing over). `scripts/build/compile-check-fixtures.sh`
      GENERATES two of the `nros::main!` forms it proves:

          main_macro_form3: nros::main!(model = "demo_bringup");
          main_macro_form4: nros::main!(board = …, model = "demo_bringup:config/system_model.yaml");

      W7.d swept the in-tree entry SOURCES; this script writes its forms at
      build time, so it was not in that sweep. It stages with
      `cp -r <template>/. <staged>/` — which copies the template's committed
      model — and never runs `nros sync`. Once W4.a deletes the committed
      models the staged trees hold no model, both forms fail to compile, and
      `check-fast`'s compile-check goes red.

      The fix is a DECISION, not a rewrite: forms 3/4 exist to prove the
      DEPRECATED `model =` still compiles during its window. Either the staging
      gains a resolve step (keeps that coverage working with no committed
      model), or the forms move to `launch =` and the deprecated-form coverage
      ends early. Whoever owns W7.e/W4.a should pick.

      Same shape as the other W4.a blockers: the committed model is not just an
      artifact, it is something else's INPUT — here, a compile proof's.

      **RESOLVED with W4.a (2026-08-03):** the generator now runs `nros sync`
      in each staged workspace (materialising `build/nros/models/`), form 3
      moved to the canonical `launch =`, and form 4 stays `model =` as the ONE
      deprecated-arm compile proof — resolving against the BUILD artifact via
      the ladder, no committed file involved.

- [x] **W7.f** (2026-08-03) Then W4.a (delete the 120 committed models) + W5 (retire the
      transitional guards) proceed as written.

### W6 — Documentation

- [x] **W6.a** (2026-08-03 — workspace-bringup, workspace-entry-pkg,
      workspace-cpp, component-and-entry-pkg pages carry the `launch =` /
      `BRINGUP`+`LAUNCH` spellings and the build/nros/models location) Book: where the model lands, how to read it, that it is output.
- [x] **W6.b** (2026-08-03 — CLAUDE.md bullet rewritten to the build-artifact
      rule) CLAUDE.md: the model line currently implies a committed file.
- [x] **W6.c** (2026-08-03 — RFC-0063 `Stable`; issue 0380 resolved +
      archived) Mark RFC-0063 `Stable` and close issue 0380.

## Acceptance

- [x] No tracked `*/config/*model.yaml` remains (112 at deletion time;
      `check-no-tracked-models` in check-fast enforces it).
- [ ] A clean checkout builds every realtime workspace (post-331 names:
      `realtime-{rust,c,cpp,cpp-subnode-portable}`) and the generated
      models carry all 86 dims (`nros ws model-dims` against the retired
      baseline as the oracle). *Pending: the tier-2 validation round.*
- [ ] Deleting a build directory and rebuilding reproduces **semantically
      identical** models (parsed-YAML compare — the W4.a dry run's oracle;
      byte-identity against the *committed* copies was impossible by
      construction: stale `scope`, issue-0356 `target`). A
      rebuild-vs-rebuild byte check has not been run; semantic identity is
      the property the cache actually needs.
- [ ] The realtime e2e family (the ~17 tests that lost their subject in 0380)
      passes against generated models. *Pending: the tier-2 validation round
      (fixture build fully green in round 24; ci-matrix reached its final
      check stage).*
- [x] A copied-out standalone example still builds (W3.c) — the esp32 /
      freertos / native standalone families built green in the round-24
      fixture sweep, and the compile-check forms (W7.g) cover the
      `launch =` spellings.

## Risks

- **Phase-331 is folding examples underneath this phase.** See the note on W4:
  order its W2–W3 before this phase's W4, and keep this phase's W3–W4 outside
  its W1→W5 measurement window. RFC-0065's builder would remove most of the
  per-move churn that phase pays, but it is a Draft and phase-331 should not
  wait on it.

- **The proc-macro path (W3.b).** `nros-macros` reads the model at expansion
  time. If it cannot see the build directory, either the macro stops reading
  the model or the model must also exist somewhere the macro can reach — which
  would partly reintroduce what this phase removes.
- **The fork boundary (W1.b).** The schema change lands in a vendored fork the
  agent may not push; sequencing needs the maintainer.
- **Nine workspaces at once (W2).** The dims differ per workspace, so W1's
  schema must be complete before the family migrates, or 0380 recurs at scale.
