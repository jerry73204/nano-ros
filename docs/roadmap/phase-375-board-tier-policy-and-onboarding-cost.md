# Phase 375 — The board tier is a promise with an owner, and onboarding is the cost

**Status (2026-09-06). W0 and W1 LANDED; W2–W9 open.** Opened from the
question "more and more boards appear; it bloats — balance the platforms per
tier". The measurement says the tiers are NOT what bloats, so the waves below
target owners and onboarding instead.

**W6–W9 added 2026-09-06** for RFC-0064 revision 5. W2 asked for a scaffold that
emits the five artifacts a board needs; auditing what those artifacts should
CONTAIN found that the tree has four different board-addition processes and no
single descriptor shape to scaffold. W6–W9 make one process exist, so W2 has
something to emit.

**Implements:** [RFC-0064](../design/0064-board-support-organization.md)
revision 4 (2026-08-22) and revision 5 (2026-09-06).
**Related:** [phase-320](archived/phase-320-board-support-tiers.md) (W3.b opened
the maintainer field and left it unenforced — this closes it),
[phase-346](archived/phase-346-out-of-tree-board-seam.md) (the seam W5 needs,
COMPLETE), [phase-370](phase-370-freertos-posix-board-cyclone.md) and
[phase-372](phase-372-s32z270-freertos-board-bundle.md) (the two boards whose
onboarding cost is the evidence).

## The measurement

| | |
| --- | --- |
| `matrix::CELLS` | 191 — 181 Runtime, 5 BuildOnly, 5 CarveOut |
| `fixtures.toml` rows | 422, of which `linux` is 195 (46 %) |
| lane coordinates | tier 1 **10**, tier 2 **14**, nightly **37**, tier 3 **51** |
| board registry | 5 tier-1, 6 tier-2, 2 tier-3, 9 infra — **0 with a maintainer** |
| tier-2 fixture BUILD | ~33 min (one observation, this host) |
| tier-2 RUN | 128 s for 1673 tests |

A new platform costs **+1 coordinate in tier 1, tier 2 and nightly** (1-wise and
pairwise both absorb a new axis value cheaply) and +2 in tier 3. The run is
nearly free; the BUILD dominates, and it scales with fixture ROWS, where the
mass is `linux` at 46 % — not the new boards, which carry 2–3 rows each.

What actually cost: **`s32z270` landed red on FIVE gates** (weak symbols, board
tiers, leaf lock, provider announcements, matrix orphan) and `freertos-posix` on
two plus a lane-table cascade. Each gate was correct. The cost was discovering
them serially, on main, by whoever noticed.

## W0 — Model the S32Z270 row — **LANDED 2026-08-22**

- [x] `Tier::BuildOnly` cell for `(FreertosMps2, Cpp, Cyclonedds, EntryPubsub,
      Workspace)`, string naming what unlocks it (a hardware or simulator
      witness). Clears `fixture_rows_all_modeled_by_matrix`, the last real tier-1
      failure.
- [x] `check-board-tiers` taught the BORROWED-token case: a tier-3 row whose
      platform is claimed at tier 1/2 by a DIFFERENT crate is not proven by cells
      that are the other board's witness. Inferred from existing rows, not a new
      key; the exemption prints. Negative control: with the owner also at tier 3,
      the rule fires again.

**A correction this wave produced.** An earlier reading of the matrix reported
"181 cells, all Runtime" and concluded the table had no vocabulary for a
build-only board — so the first proposal here was to ADD a `Build` kind. Wrong:
the regex matched only bare-identifier tiers and silently dropped
`BuildOnly("reason")`. The vocabulary has existed since revision 3. Re-measure
before proposing a mechanism, and prefer a count the code produces over one a
regex infers.

## W1 — Maintainers become the tier gate — **LANDED 2026-08-23**

- [x] Adopted Rust's counts in `check-board-tiers`: tier 1 >=3, tier 2 >=2,
      tier 3 >=1 named maintainer. `infra` and `scaffold` owe nobody — neither
      makes a tier promise, and charging the honest states more than the
      dishonest one is how a rule gets routed around.
- [x] Landed as a RATCHET: `scripts/board-maintainer-baseline.json` grandfathers
      the 13 non-infra rows that predate the rule, and only shrinks. Claiming a
      board is `maintainers = [...]` plus `--write-baseline`; meeting the rule
      while baselined is PRINTED, never failed, because a ratchet that punishes
      the good deed gets bypassed for the same reason a cliff does.
- [x] Demotion is automatic and printed: the gate cannot edit the registry, so
      the demotion it performs is a refusal that names the tier the row IS
      entitled to (`tier_supported_by`) rather than leaving the author to
      re-derive the table.
- [x] Two leaks closed that a plain exemption list would have had. The baseline
      is keyed by the `(crate, platform)` PAIR the registry is keyed by, so a
      crate serving several witnesses at different tiers (phase-337 W1.c) cannot
      have one row grandfather another. And an exemption does not travel UPWARD:
      a row baselined at tier 3 and later promoted to tier 2 is making a stronger
      promise than anyone grandfathered, so it needs its own owners. A demotion
      stays exempt — the rule must never block the direction it is asking for.
- [x] Negative controls run on EVERY invocation, not behind `--self-test`: a
      control nobody runs decays into a comment, and this rule's whole job is to
      fire. Nine cases, each asserting the refusal happens and then that the
      intended escape silences it. A MISSING baseline is a failure, not an empty
      exemption set — empty is the strictest reading and still the wrong one,
      failing all 13 rows at once with a message about maintainers rather than
      about the absent file.

**Acceptance — met.** A new board row without a maintainer cannot be tier 1 or 2
(it is not in the baseline, and the baseline is a committed file, so growing it
is a reviewable diff rather than a silent default). The list only shrinks.

**Still true and still the point:** all 22 rows carry `maintainers = []`. W1 did
not assign owners — inventing one would be worse than recording none. It made
the field load-bearing, so the next board pays the question and the existing
ones pay it when someone answers.

## W2 — `just board-new` — onboarding is a scaffold, not a scavenger hunt

- [x] Emit, in one command: the `nros-board.toml` descriptor, the `package.xml`
      `<nano_ros_provides>` export mirroring its `names`, and the
      `board-support.toml` row (at `tier = "scaffold"`, the tier that promises
      nothing and so needs no maintainer). _(landed as `nros board new` /
      `just board-new`.)_ The weak-symbol allowlist stub and the leaf lock are
      NOT emitted: a scaffolded board has no crate, so it has neither a symbol
      to allow nor a lock to write. A board that later grows a crate needs both,
      and `just board-new` says nothing about that yet.
- [x] Those are exactly the five gates `s32z270` tripped. The scaffold's
      acceptance is that a board created by it passes `just check fast` on the
      first run. _(verified: `board_new_scaffold.rs` scaffolds in-tree and
      out-of-tree and asserts the board RESOLVES; `just check fast` is 236 green
      with the scaffolded board present.)_

**Acceptance:** a scaffolded board is green on `check-fast` before its first
commit; the five gates stay unchanged (this does not weaken them).

## W3 — A board below tier 2 must not redden a shared lane

- [x] **The rule, in RFC-0064 terms: a board that is not COMPLETE does not
      resolve, so it cannot redden anything.** Completeness is now structural
      rather than a checklist — `BoardCatalog::require_announcement` refuses a
      descriptor with no `package.xml`, `check-board-tiers` refuses a board
      directory with no registry row and a row with no directory, and
      `check-provider-announcements` refuses a descriptor whose announcement
      disagrees with it. An incomplete board fails at the point of resolution,
      naming itself, instead of surfacing three gates later as someone else's
      red main.
- [x] **Mechanism: onboarding-complete-at-merge, option 1.** W2 is landed, which
      is the condition this wave set for choosing it — the scaffold makes
      completeness cheap, so buying isolation by excluding tier-3 boards from
      `check-fast` would trade coverage for nothing. Recorded as a decision
      rather than left as a default, per W5's own standard.
- [x] W2 first — the second option trades coverage for isolation and should only
      be reached for if the scaffold proves insufficient. _(It did not.)_

**What this does NOT claim.** The rule covers a board that is missing something.
It does not cover a board that is COMPLETE and BROKEN — a tier-3 board whose
fixture row fails to build still reddens a shared lane, and nothing here changes
that. W4's smoke floor and witness-gated ceiling is the wave that addresses it,
and it is the one this phase says needs agreement before implementation.

**Acceptance:** adding a build-only board cannot make main red for people who do
not use it.

## W4 — Smoke floor, witness-gated ceiling

- [x] **The floor is LANDED, and it was already met — that is the finding.**
      Measured 2026-09-06 before implementing: Linux 72 Runtime cells,
      ZephyrNativeSim 39, ThreadxLinux 18, FreertosMps2 16, NuttxArm 14,
      ThreadxRiscv64 10, NuttxRiscv 4, ZephyrQemuCortexM 3, Esp32Qemu 2,
      FreertosPosix 2, QemuBaremetal 1. **No platform had zero**, and the
      minimum was already exactly the floor this wave asks for. So the rule
      changes nothing today, which is the point: the distribution was
      defensible and UNDECLARED, and an undeclared floor is one the next
      platform lands beneath unnoticed.

      Implemented as a direction added to `check-board-tiers`, not a second
      gate: it already asserted "a platform CI runs must carry a tier promise",
      and the floor is the converse — "a tier promise must have Runtime
      evidence". `scaffold` and `infra` are exempt by TIER (scaffold promises
      nothing, which is what `just board-new` starts every board at), and
      `execution_class = "hardware"` is exempt by DECLARED PROPERTY — such a row
      states that CI does not run it, so demanding a Runtime cell would demand
      evidence the row says nobody collects, and the cheapest way to satisfy
      that is to fake the cell.

      **It found a real violation on its first run**: `nros-board-zephyr`'s FVP
      row, tier 3, naming `matrix_platform = "Fvp"` — a PlatformId with ZERO
      cells of any kind. It is exempt as `hardware`, correctly, but the row's
      stated reason was wrong in two ways and is now fixed: the model is not
      licence-gated (`nros setup --tool arm-fvp` fetches it), and the
      board-import fixture was verified booting on it the same day. What keeps
      CI out is cost and x86_64-only hosting, not permission.

- [x] **DECIDED, split.** Floor landed (above); ceiling NOT implemented and
      recorded as such, to override rather than to re-litigate. Ticked because
      a decision IS the outcome for this box — leaving it open would say work
      remains, and none does unless you reverse the call. Original wording:

      ~~Every supported platform earns exactly ONE Runtime cell (boots, delivers a
      message) to sit in tier 2; full cells in nightly require a witness.
      Today's spread is Linux 72 / ZephyrNativeSim 39 / … / QemuBaremetal 1 —
      defensible but undeclared. This makes it intentional and gives a new board
      a known, bounded entry cost.~~ _(the floor half is landed above; the
      CEILING half is deliberately NOT done — see below)_

**THE CEILING IS NOT IMPLEMENTED, and this is a judgement call to override if
you disagree.** "Full cells in nightly require a witness" REMOVES coverage that
exists today: a regression in Linux's 72 cells would surface a day later instead
of at the merge queue. This phase's own Risks section flags it as needing
agreement before implementation, and the measurement argues against it — the
same measurement that opened this phase says the tier-2 BUILD dominates and
scales with fixture ROWS, where `linux` is 46 %, not with cells. The ceiling
would cut the thing that is not expensive.

If tier-2 build time later becomes the binding constraint, re-open it against
rows rather than cells.
- [x] Do NOT replace the computed 1-wise/pairwise cover with declared per-test
      platform lists. Zephyr's equivalent (`integration_platforms`) is reported
      by their own issue #57595 as trial-and-error and untested; this tree's
      cover is gated by `documented_lane_table_is_live` and cannot drift.

**Acceptance:** the per-platform cell count follows a stated rule, and a new
platform's tier-2 entry cost is one cell.

## W5 — Decide S32Z270's home

**DECISION (2026-09-06): RECOMMEND MOVING IT OUT OF TREE. Not executed — the
move needs ASI to accept hosting it and to run the link check, which is a
commitment this repo cannot make on their behalf.** Recording the reasoning so
the decision is made once rather than defaulted to repeatedly, which is this
wave's whole acceptance.

**What changed since the wave was written.** Two things, both today:

* `just board-new --out-of-tree <dir>` exists (W2/W9), so the move is one
  command plus one `NROS_EXTRA_BOARD_PATH` line. Before, the out-of-tree seam
  was built but nobody had walked it end to end; now
  `a_scaffolded_out_of_tree_board_resolves_by_name` walks it in CI.
* W4's smoke floor makes the in-tree cost explicit rather than notional.

**The measurement.** `s32z270-freertos` contributes **0 Runtime cells**, ~2
fixture rows, and **0 maintainers**. Its registry row deliberately carries no
`matrix_platform` (reusing `FreertosMps2` would claim a platform whose cells
assert running code — the defect phase-320 W1.a fixed one family over).

**The trade, stated both ways:**

| | in tree | out of tree |
| --- | --- | --- |
| evidence it still links | every `just ci` | ASI's CI only |
| who is accountable | a named maintainer under W1 — currently NOBODY | ASI, by construction |
| cost of the move | — | one command; the seam is tested |

**Why the recommendation is "move".** The thing in-tree residence buys —
"someone who is not ASI notices when it breaks" — is the CHEAP half, and ASI
running it against their own consumer is strictly better evidence than us
running it against a consumer we cannot see. What in-tree residence REQUIRES is
a named maintainer, and there is none. So today it is an unowned promise in a
shared tree, which is exactly the state W1 was written to stop.

**What would change the recommendation:** somebody putting their name on the
row. That is a person, not a patch.

- [x] It exists for `autoware-safety-island`. Zephyr's guidance is that a
      product board belongs in the product repo, and phase-346 landed the
      out-of-tree seam that makes it possible. _(Considered; the DECISION block
      above is the outcome.)_
- [x] The trade is explicit: out-of-tree costs in-tree evidence that the board
      still links; in-tree costs a maintainer under W1 and the onboarding under
      W2. Either is fine; defaulting without deciding is not. _(Stated as a
      table above, both ways.)_
- [x] If it stays, it needs a named maintainer and it stays BuildOnly until a
      witness exists. _(It has none, which is the argument for moving it.)_

**Acceptance:** a recorded decision, not a default. **MET** — the decision is
recorded above, with what would change it.

**Not yet EXECUTED, and deliberately not by me.** The move needs ASI to accept
hosting the board and to run the link check in their CI. That is a commitment
this repo cannot make on their behalf, and executing half of it — deleting the
board here before it has a home there — is the one outcome worse than either
choice. The nano-ros half is one `just board-new --out-of-tree` plus removing
the registry row, and takes minutes once ASI has agreed.

## W6 — One board process: a board is a package, found by a scan

RFC-0064 R5 D1/D2/D5. The measurement is in R5: four routes, and our own
`qemu_cortex_a53` takes route 4 — an inline `board = "…"` string in
`fixtures.toml` and a `.conf` copied into each example leaf, with no package, no
descriptor and no announcement.

- [x] Every board is a directory with `package.xml` announcing
      `<nano_ros_provides kind="board" name="…"/>` beside an `nros-board.toml`.
      A crate only where the board needs bring-up code. _(landed.
      `BoardCatalog::require_announcement` REFUSES rather than warns, and it
      runs in the catalog as well as the gate because the gate sees only the
      nano-ros tree — a consumer's own boards and `$NROS_EXTRA_BOARD_PATH` are
      exactly what it structurally cannot reach.)_
- [x] Boards share `provider_scan`'s walk RULES — `is_pruned_dir`,
      `is_ignored_dir`, `IGNORE_MARKERS`, reached by `pub` rather than copied —
      and `BoardCatalog::collect_board_dirs` recurses instead of stopping one
      level down, so a bundle board can own a descriptor. _(landed. NOT yet one
      literal walk: the descriptor scan still runs separately. That is the
      remaining half and it is cheap once `board.cmake` is gone.)_ Original
      wording: boards are discovered by `provider_scan` like every other
      provider;
      `nros-board.toml` is read for a package that announced itself as a board.
      This collapses two walks into one. The out-of-tree ROOTS already work
      (`extra_board_roots()` reads a PATH-style `NROS_EXTRA_BOARD_PATH`, and
      `load_with_packages` absorbs a board declared inside the consumer's own
      workspace package) — what does not is that `BoardCatalog::load_root` stops
      at one level, so a bundle board cannot own a descriptor and is patched in
      by `attach_bundle_aliases` reading `board.cmake`. W7 deletes that file, so
      this is a prerequisite, not a tidy-up.
- [x] `check-provider-announcements` loses `if not os.path.exists(pkg_xml):
      continue`. Ratchet against today's one offender,
      `nros-board-mps3-an536-freertos` — the newest board in the tree, which
      skipped the step because the step was optional. _(landed: its
      `package.xml` is written, the gate now globs bundle depth too, and the
      gate gained the negative control it never had — mutation-tested by
      restoring the `continue`, which the control catches.)_
- [x] `qemu-cortex-a53` and the Zephyr `mps2_an385` flavour are real board
      packages — scaffolded with `just board-new` (dogfooding W2), declared,
      announced, and carrying registry rows at `tier = "scaffold"`. Gated
      going forward by `check-fixture-boards-declared`: every Zephyr `board =`
      in `examples/fixtures.toml` must equal some descriptor's
      `[board.zephyr] west_board`.

      **Two halves of this box were dropped on measurement, both because the
      original wording assumed a defect that is not there:**

      * *"their fixture rows name a board instead of a Zephyr id"* — the id is
        what `west build -b` takes, and it is also a component of west ids and
        artifact paths. Re-spelling it renames build outputs; it does not
        improve a schema. Declaring the board was the part that was missing.
      * *"the per-board `.conf` moves out of the example leaf"* — that file is
        Zephyr's own app-level `boards/<board>.conf` convention, auto-included
        for the app being built. It is in the right place already;
        `examples/workspaces/realtime-c/src/zephyr_entry` does not call
        `nano_ros_use_board()` at all, so there is no nano-ros duplication to
        remove.

      **Deliberately not done in the W6 pass, and worth saying why rather than
      half-doing it.** The package half alone is cheap and useless: it would add
      two declared boards nothing reads, which is the "correct and unreachable"
      shape this whole wave exists to remove. The half with the value is the
      fixture retarget — `examples/fixtures.toml` carries `board =
      "qemu_cortex_a53/qemu_cortex_a53/smp"` as a Zephyr id passed straight to
      `west build -b`, so naming a board KEY means teaching the fixture builder
      to resolve through the catalog. That is verifiable only by a Zephyr
      fixture build, which needs the SDK and west. Do the two halves together,
      on a host that can run `just build-test-fixtures`.

**Acceptance:** every board the tree builds is announced; the announcement gate
runs on all of them, not 10 of 14; an out-of-tree board directory resolves with
no in-tree edit beyond a `package_paths` entry.

## W7 — `[board.zephyr]`, and the projection replaces `board.cmake`

RFC-0064 R5 D3/D4. Pairs with 215.K, which is the same change seen from the FVP
board's side.

- [x] `[board.zephyr]` block: `west_board`, `sdk_abi`, `default_rmw`,
      `default_transport`, `runner` (stated only when it differs from Zephyr's
      own board definition).
- [x] Conventions supply the rest: `prj.conf` and
      ``boards/<west_board with `/` → `_`>.{conf,overlay}`` present-if-exists; the
      Rust-support Kconfig module derived AND generated, since its body is
      `default y if BOARD_<UPPER(first segment)>`.
- [x] `nros board cmake-vars <name> --out <build-dir>/…`; `nano_ros_use_board()`
      includes the result. Signature unchanged, so no consumer call site moves.
      The projection is a build artifact, never committed (phase-330 precedent).
- [x] Delete `board.cmake`, `board_metadata.rs` and
      `phase215_f_manifest_drift.rs`. Replace with a projection round-trip test
      (descriptor → projection → parse back == descriptor), which has no
      "skip if the other face is missing" arm and so cannot go vacuous the way
      the drift gate did.
- [x] Gate `check-board-descriptor-single-source`: no `board.cmake` anywhere, no
      `[package.metadata.nros.board]` anywhere.

**Acceptance:** one authored descriptor per board; the drift gate that checked
zero boards is gone rather than repaired.

## W8 — The descriptor holds primitives only

RFC-0064 R5 D6 carries the full audit over all 12 `[[board]]` rows. Landable
field group by field group; each group is a gate plus a mechanical edit.

- [x] **`platform_feature` and `link_kind` are the platform's now — but NOT via
      a platform descriptor, and the difference is the finding.** 30 authored
      values removed (15 + 15).

      The wave said "move it to the platform descriptor". Attempting that
      surfaced why that shape cannot work: there is no `config/esp32/`, and BOTH
      ThreadX kinds map to one feature — so the mapping is keyed on the
      `PlatformKind` ENUM, not on a directory, and a descriptor per platform
      would have needed directories that do not exist for exactly the cases
      that made the field worth moving. `PlatformKind::platform_feature()` and
      `::link_kind()` carry it; the board fields default from them, and
      `check-derived-descriptor-fields` refuses a stated value that disagrees.
      Board override survives: `link_kind` is an `Option` with an accessor.

- [x] **`[board.priority_plan]` has a per-platform home** — landed for the one
      platform that actually duplicated. `config/freertos/nros-platform.toml`
      carries the plan; both FreeRTOS boards dropped their byte-identical
      copies and now inherit, with a board-level block still overriding.

      The duplication was worse than untidy: `load_plans()` builds a
      `platform -> plan` map, so with two boards declaring for one platform the
      LATER one silently won, and a divergence would have been invisible. (The
      loader does carry a conflict check — it fired during this work, which is
      how the next item was found.)

      **Two defects found while doing it, both in `priority_plan.py`:**

      * `load_plans()` carried its own inline copy of the parser — in the module
        whose docstring says the two consumers "had a table each for about an
        hour, which is the second spelling this codebase keeps paying for".
        There is one `_parse_plan_block` now, shared with the platform reader.
      * That parser gated on `if header not in text`, which a COMMENT naming the
        table satisfies. The boards that moved their plan kept a comment saying
        where it went, and the substring test then produced an EMPTY plan that
        "conflicted" with the platform's. Issue 0516's hazard, in this module.
        It requires the table to be entered now.

      Only FreeRTOS moved. posix, nuttx, threadx and zephyr each have exactly
      ONE board declaring a plan, so there is nothing to de-duplicate and
      moving them would be churn; they move when a second board of that
      platform arrives, which is when the loader's conflict check would
      otherwise start deciding by file order.

      ~~Superseded wording: **`[board.priority_plan]` still wants a per-platform home** — the two
      FreeRTOS QEMU boards hold byte-identical blocks, which is
      `configMAX_PRIORITIES`. Unlike the two fields above it is per-RTOS DATA
      (ranges, reserved bands, a resolver) rather than a mapping, so the enum
      trick does not apply and a real platform descriptor IS the right home.
      That needs `nros-platform.toml` to exist for five more platforms, which
      is its own wave.
- [x] **Derive:** ~~`entry_kind`~~ (the "zero exceptions" claim was WRONG —
      `freertos-posix` and `mps2-an385-freertos` are both `platform = "freertos"`
      with different `entry_kind`; it stays authored), `local_aliases`
      (8 of 10 are `[platform_feature]`), `[board.entry] crate_name` (7 of 7 are
      `snake_case(board_crate)`), `[board.entry] signature` (4 of 7 identical).
- [ ] **Decompose `cargo_config`** — a raw TOML blob in 8 of 12 rows, holding
      `rustflags`/`runner`/`linker` as text with no schema check. Promote them to
      real fields; the CLI composes the leaf `.cargo/config.toml` from those
      instead of pasting the blob through.

      **Left for a host that can build fixtures.** This is the largest remaining
      W8 item and the only one that changes what lands in a leaf's
      `.cargo/config.toml`. Its failure mode is a leaf that resolves against the
      wrong target or links with the wrong flags — visible in a fixture build
      and invisible to `just check fast` and the CLI unit tests, which is all
      this host can run. Doing it blind and reporting it green would be the
      museum-binary mistake in a new place.
- [x] **Drop:** `capability_features` (7 rows, all the single value
      `["safety-e2e"]`; the only read in the tree is inside
      `fn a_board_advertises_the_safety_capability_feature()`),
      `[board.entry] comment` (an escaped Rust `//` comment in TOML),
      `crate_path` and `board_features` (in the struct, authored by nobody).
- [x] **Reclassify** `target_contains` — a row disambiguator, not a board
      property. Renamed to `disambiguate_by_target`, with the old spelling kept
      as a serde ALIAS: an out-of-tree board descriptor is a user's file, and
      breaking it to improve a name in our tree is a cost we impose and do not
      pay.
- [x] Rung 3 survives: `crate_root_extra` / `crate_root_deps` / `closure_extra`
      stay. ~~renamed to say they are the escape hatch~~ — **not renamed, and
      the reason is recorded on `BoardEntry`**: a rename is churn in every
      out-of-tree descriptor using one, buying a label the doc-comment already
      carries in the same place a reader meets the fields. `target_contains`
      earned its rename because the name led a reader to a wrong conclusion;
      these do not. The rung-3 rule the RFC attaches to them — *it must always
      exist* — is now stated where they are defined.
- [x] Gate, in RFC-0087 D4's ratchet shape: a stated derivable field must equal
      its derived value. _(landed as three additions to the EXISTING
      `check-derived-descriptor-fields` — a sibling-field derivation mechanism,
      the bundle glob, and `[board.entry]` flattened in — rather than a second
      gate beside it.)_ The "moved field must not be restated" half waits on the
      move, which waits on the platform descriptors below.

**Acceptance:** a QEMU Zephyr board authors 6 keys where today's equivalent
authors 19, and the in-tree and out-of-tree descriptors for comparable boards are
identical field for field.

## W9 — `just board-new` emits the shape W6–W8 defined

W2 restated once there is one shape to scaffold. Kept separate because W2's
acceptance (green `check-fast` before first commit) is unchanged and still the
point.

- [x] Scaffold emits the W6 package (`package.xml` + descriptor) and the
      `board-support.toml` row. _(landed.)_ Allowlist stub and leaf lock: see
      W2 — a crate-less scaffold has neither.
- [x] A `--out-of-tree <dir>` mode emits the same thing and prints the
      `NROS_EXTRA_BOARD_PATH` line that reaches it, so a user's board is one
      command too. _(landed;
      `a_scaffolded_out_of_tree_board_resolves_by_name` is the claim as a
      test.)_

**Acceptance:** the scaffolded board and a hand-written in-tree board are the
same files.

## Risks

**The ratchet is the whole of W1.** A maintainer rule that fails every existing
row on landing day gets bypassed, and a bypassed gate is worse than none — the
same argument CLAUDE.md makes about a tier nobody can afford being followed
selectively.

**W4 changes what a tier PROMISES**, so it is the one wave that needs agreement
before implementation rather than after. The smoke floor is cheap; the nightly
ceiling removes coverage that exists today.

**W8 moves fields between descriptors, and a move is invisible in a diff.** A
board that silently loses its priority plan gets the platform default, which is
plausible and wrong. The ratchet has to fire on a RESTATED field, not only on a
missing one, and the move lands per field group with the gate in the same commit.
