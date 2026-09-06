# phase-422 — provisioning has one path, and CI walks it

**Status (2026-09-06). Every work item is landed or open by decision.** W1
(duplicate producers retired), W2 (mdbook + verus are index tools; the bespoke
scripts are deleted), W3 (dispatcher passthrough, the workflow sweep, the prose
sweep and its ratchet arm), W4 (the nightly zephyr jobs walk the user path), W5
(the ratchet, now five gates — the last one asserts what W1 fixed stays fixed),
W6 (the scope verb reports the system closure), W7 (the additive board entries
plus the gate), W8 (scoped to `infra`).

**OPEN BY DECISION, not by omission:** W7's true unification — the index
namespace is what users type while the cmake namespace is what the build uses,
and neither is obviously the one to keep. That is the only one left: W2's
clang-format exception was retired on 2026-09-06 when its stated reason expired
(the machinery it said would serve one consumer had been built for another), and
the four un-indexed cargo tools went in with it.

**The 2026-09-05 header was stale in both directions** and this is a note about
reading them, not only about this one: it listed W2, W4 and W7's additive half
as IN FLIGHT when all three had landed, and claimed W5's four gates when the
item's own text says a fifth was missing — which it was. A status line written
once and not re-derived is the same authored-mirror class the repo keeps paying
for; each claim above was checked against the tree.

Implements the provisioning half of RFC-0014 and finishes what phase-413 W5
started. Prior phases: 413 (CI workflow user parity), 398 (`package.xml`
dependency ladder), 365 (versioned SDK store).

## The problem

There is supposed to be one way to provision this tree: `nros setup` reading
`nros-sdk-index.toml`, wrapped by `just setup <scope>`. In practice there are
four, and they disagree.

**1. Two producers for the same tool.** `[tool.corrosion]` in the index and
`just workspace install-corrosion` both install corrosion, into the same
versioned prefix, each with its own stamp logic. Same for
`play_launch_parser`. Their version pins are not even spelled the same:

| tool | `just/workspace.just` | `nros-sdk-index.toml` |
| --- | --- | --- |
| corrosion | `v0.6.1` | `0.6.1-nros1` |
| play_launch_parser | git SHA `838ce948…` | `0.1.0-nros1` |

Issue 0500 is the cost of this already being paid once: the SDK store
accumulates, prefixes resolve newest-first, and **both provisioning paths print
success either way** — so a stale Corrosion shadowed the pin that had just been
installed, and `mixed` could not link. A second producer is not a convenience;
it is a second answer to "which version is installed?".

`just workspace install-sccache` is NOT in this category — it is a thin wrapper
that shells `nros setup --tool sccache`. It should still lose the wrapper, but
it is tidying, not a correctness fix.

**2. Installers that never reached the index at all.** `clang-format` (pip
wheel download, `scripts/`-side logic), `mdbook` (`scripts/setup-mdbook.sh` +
a hand-maintained `scripts/mdbook-checksums.txt`), `verus`
(`scripts/setup-verus.sh`, reached by `just verify-verus`). Each re-implements what the index already does for
16 other tools: pinned version, download, checksum, install prefix, smoke
check. None of them can be asked "are you at the pin?" the way
`nros setup --tool X --check` can.

**3. Two spellings of the setup verb, one of which silently does less.**
`just setup <scope>` runs `_setup-common` then the module recipe;
`just <scope> setup` runs the module recipe alone. `_setup-common` is where
the host facts every tier asserts get provisioned. nightly's platform job used
the module spelling and carried a hand-rolled "Install cross targets" step to
compensate — a workaround for a defect one word wide (fixed, #397).

**4. CI lanes that do not walk the user's path.** phase-413 W5 converted
`build-wide`, `run-matrix`, `queue` and `host-tests`. The nightly zephyr jobs
still spell out six provisioning steps each, and cannot convert today because
they pass `just zephyr setup --skip-sdk` and the dispatcher has no argument
passthrough.

### What this cost, measured

Three consecutive host-tests runs, each ~40 minutes, each failing on a
different missing prerequisite that `just setup` did not provision:

1. cross Rust targets (`armv8r-none-eabihf`), pinned corrosion, fixture stamp
2. — same run after fixes: reached `check fast`, died on `clang-format`
3. the layer below that is what W5's gate now finds statically

Each layer only became visible once the one above it was fixed, because a
missing prerequisite fails the run at the first gate that needs it.

## Work items

### W1 — retire the duplicate producers — DONE

**Decided and landed.** Both are forwarders to `nros setup --tool <name>`. The
index path was verified working BEFORE anything was retired
(`present 0.6.1-nros1 (skip)`, `--check` -> `[OK]`).

Retiring them exposed the drift immediately: corrosion's two pins AGREED
(`CORROSION_VERSION` = the index's `upstream`), but play_launch_parser's did
NOT — a git SHA against `0.1.0-nros1` — so forwarding made `just doctor` call a
correctly installed tool MISSING. The doctor now asks the CLI rather than
comparing a second constant, and `PLAY_LAUNCH_PARSER_VERSION` is deleted.

Original text:

`install-corrosion` and `install-play-launch-parser` become forwarders to
`nros setup --tool <name>`, or are deleted and their callers changed. The
version constants `CORROSION_VERSION` / `PLAY_LAUNCH_PARSER_VERSION` in
`just/workspace.just` go with them: the index is the pin.

Care: `_setup-common` calls `just workspace install-corrosion` today (landed in
#365). Whichever spelling survives must still be reachable from every
`just setup <scope>`, or `check-preconditions-provisioned` fails — which is the
gate doing its job.

**Acceptance.** No tool has two installers. Each tool has exactly one pin.
`check-preconditions-provisioned` (#387) stays green.

### W2 — move the bespoke installers into the index

`clang-format`, `mdbook`, `verus` become `[tool.*]` entries with a pinned
version, a download, a checksum and a `smoke` command, like the other 16.
`scripts/setup-mdbook.sh`, `scripts/mdbook-checksums.txt` and
`scripts/setup-verus.sh` are deleted, not left as dead alternates.

**DECIDED (2026-09-05): split this item.**

The index ALREADY expresses per-host artifacts — `dist.linux-x86_64 = { url,
sha256 }`, as `[tool.qemu]` and `[tool.arm-none-eabi-gcc]` do. So the question
was never "can the index do this", it was "which of these three fit that
shape".

* **mdbook and verus fit it exactly** — plain release tarballs, no new
  machinery. `scripts/setup-verus.sh` additionally resolves "latest" through
  the GitHub releases API, a moving target the index replaces with a pin.
  DOING THIS NOW.
* **clang-format does not.** It is a pip wheel whose binary sits at
  `clang_format/data/bin/clang-format` inside a zip, so it needs the index to
  unzip and know an inner path — new machinery for exactly one consumer. It
  KEEPS its recipe as a documented exception until a second wheel-shaped tool
  appears. One exception with a stated reason beats index machinery serving one
  caller.

**Acceptance.** `scripts/setup-mdbook.sh` and `scripts/setup-verus.sh` no longer
exist as bespoke downloaders. Every remaining bespoke installer is listed in this doc
with a reason.

**The second half landed 2026-09-06, and the clang-format exception is gone
with it.** The exception's stated reason had expired: W2 declined clang-format
because the index would need "to unzip and know an inner path — new machinery
for exactly one consumer", and that machinery now exists and serves two.
`DistArtifact.install` was added for `[tool.ninja]`, whose upstream zip holds a
bare binary; a pip wheel is the same shape one directory deeper. So it is a
`dist` row with an `install` step, not an exception:

```toml
install = "unzip -q -o {archive} -d {prefix}/.wheel && \
           install -Dm755 {prefix}/.wheel/clang_format/data/bin/clang-format \
                          {prefix}/bin/clang-format && rm -rf {prefix}/.wheel"
```

**`.clang-format-version` is deleted.** The version lives in
`[tool.clang-format]` and nowhere else, which is the point rather than a side
effect: two homes for this exact number had already drifted invisibly — gate.yml
pinned 17.0.5 in a `pip install` while the file said 17.0.6, and the resolver's
fallback order meant CI checked a different formatting standard than every local
run for as long as that stood. `scripts/dev/clang-format.sh` reads the pin from
the index and resolves the store path, and `just doctor` calls that same
resolver instead of re-deriving either.

**And the four cargo tools are indexed**, which W5's audit had listed as "left,
because nothing judges them". `cargo-nextest`, `cargo-llvm-cov`, `rustfilt` and
`cargo-show-asm` were bare `cargo install <name> --locked` — the drift
`[tool.espflash]`'s own comment already names, *"resolve against crates.io at
whatever version is current that day"*. Prebuilt where upstream publishes one
(nextest and llvm-cov, both Linux arches), source otherwise.

`cargo-nextest` is the one that mattered: **it was the only tool here with no
pin at all**, CI's runners installed it through `just setup native`, and
`.config/nextest.toml` is version-sensitive enough that a stale filter is a
parse error killing every nextest run in the repo (issue 0743). The tool running
the whole suite was the unpinned one.

They join `scripts/sdk-path-tools.txt`, because `cargo nextest` is cargo finding
`cargo-nextest` **on PATH** — a cargo subcommand is invoked by bare name by
cargo itself, which is what that list is for. In `~/.cargo/bin` rustup provided
that; in the versioned store nothing does.

*Observed, installing each into a scratch `$NROS_HOME`:*

```
cargo-nextest    0.9.143   (dist)     clang-format  17.0.6  (dist, wheel)
cargo-llvm-cov   0.9.0     (dist)     rustfilt      0.2.1   (source)
cargo-show-asm   0.2.9     (source)
```

*One smoke check was wrong and running it is what said so.* `bin/cargo-llvm-cov
--version` fails — it is a cargo subcommand binary and answers
`expected subcommand 'llvm-cov', found argument '--version'`. The probe is
`cargo-llvm-cov llvm-cov --version` now. That is issue 0929's argument playing
out once more: `system` answers "are its libraries present", and only smoke
answers "does it run".

### W3 — retire the old spelling everywhere — DONE

**Landed.** `just setup <scope>` takes a variadic tail and forwards it, so
`just setup zephyr --skip-sdk` works and the exemption can go.

The subtlety, found by testing rather than assuming: `just` fills positional
parameters IN ORDER, so the flag binds to `tier`, not to the variadic tail. A
tier is never a flag, so it is re-homed — rather than making a user write
`just setup zephyr "" --skip-sdk`.

The workflow half landed with W4 below: no workflow uses the module spelling and
`check-workflow-setup-spelling`'s exemption list is empty.

**The prose half landed too.** 64 occurrences across 38 files became
`just setup <scope>`: `book/` (17), the living `docs/` subtrees — guides,
reference, development, design, release (25) — and every instructional README
under `examples/`, `packages/`, `tests/`, `scripts/` (22). The swap is
word-for-word the same length, so comment columns in fenced blocks did not move.

**What was deliberately NOT converted, because the module spelling is what the
line MEANS.** Three record series narrate what a command DID at a time, and
rewriting them falsifies the record rather than fixing a reader's copy-paste:

| left alone | why |
| --- | --- |
| `docs/roadmap/archived/**`, `docs/issues/archived/**` | historical record |
| `docs/roadmap/**` (196, 215, 413, 414, book-audit, and this doc) | phase docs record work at a time; #1038 and #413's `just esp32 setup` lines are claims about the MODULE recipe specifically, which is the defect they name |
| `docs/issues/**` | the ledger's "Recently resolved" entries and #1038's diagnosis are past tense; `docs/issues/README.md` is also the merge-queue conflict path (0883/0884), so it is not touched for cosmetics |
| `docs/research/**` | dated snapshots (2026-05-04, 2026-05-25) that COUNT the commands as they were, against a workspace layout since retired |
| `docs/development/zephyr-version-support.md:117` | `NROS_ZEPHYR_VERSION=4.4 just zephyr setup` exited 0 on a workspace that could not build Cortex-M. That run happened with that spelling; the step's instructions above it are converted |
| `packages/testing/nros-tests/fixtures/board_import_fvp/CMakeLists.txt:12` | out of the prose scope, and touching a fixture source re-stales fixtures for no reader benefit |
| `just qemu setup-qemu`, `just qemu setup-network`, `just zenohd setup` | different verbs — thin `nros setup --tool` callers — not the module `setup` recipe |

**The ratchet.** `check-workflow-setup-spelling` grew a PROSE arm over `book/`,
`AGENTS.md`, `CLAUDE.md`, `docs/{design,development,guides,reference,release}/`
and every README outside vendored/build trees — 576 tracked files. The three
record series are pruned by path with the reason stated in `PRUNE_DIRS`, which
is what keeps the arm usable: gating them would produce a steady false-positive
stream, since new issue and phase prose will legitimately keep quoting the
module spelling. Scoped that way the sweep left exactly ONE prose exemption,
keyed on `(path, exact line text)` — not a line number, which moves — and
checked in both directions.

Two things the prose arm does that the workflow arm does not, both because a
doc is not a YAML file: it does NOT skip `#` lines (in markdown those are
headings, and inside a fence they are shell comments that teach just as loudly),
and it matches ACROSS a line wrap. The wrap is not hypothetical —
`docs/reference/zephyr-armv8r-setup.md` carried `` `just zephyr\nsetup` `` and
a single-line grep could not see it. It also needs a `(?![-\w])` the workflow
arm never needed, because `\b` matches `just qemu setup-qemu`.

Mutation-tested in four directions: reverting one conversion fails it, a
line-wrapped module spelling fails it, an exemption matching nothing fails it,
and pruning `book/` from the scan set fails the self-test.

Original text:

`check-workflow-setup-spelling` (#397) forbids `just <scope> setup` in
workflows and carries exactly one exemption: the nightly zephyr jobs, which pass
`--skip-sdk`. Give the `setup` dispatcher argument passthrough so the exemption
can be deleted.

Then sweep the non-workflow callers — docs, `book/src/`, `AGENTS.md`,
`CLAUDE.md` — so the documented spelling is the one that provisions correctly.
A reader who copies `just zephyr setup` out of the book gets the module recipe
and none of `_setup-common`.

**Acceptance.** The exemption list in `check-workflow-setup-spelling` is empty.
No INSTRUCTIONAL prose teaches the module spelling — amended from "no prose in
the repo", which was the wrong target: the record series must keep it to stay
true, and a gate that demanded otherwise would be one nobody could keep green.

### W4 — the nightly zephyr jobs walk the user path — DONE

**Landed.** All four module-spelling call sites in `nightly.yml` are the
dispatcher now — one in `zephyr-example-matrix`, two in
`zephyr-dual-line-summary` (3.7 and 4.4), one in `zephyr-copy-out`. Three jobs,
four sites; W3's text said "four zephyr jobs" and there are three. The
`NROS_ZEPHYR_VERSION=` prefixes are unchanged: they are shell env on the
command, and the dispatcher passes them through to the module recipe by
inheriting them.

The flag path was PROVEN, not assumed, because the re-homing rule is new. `just
--dry-run setup zephyr --skip-sdk` shows `just` binding `target=zephyr`,
`tier=--skip-sdk`, `extra=""`; running that interpolated script with a `just`
shim on `PATH` prints exactly two invocations, `[_setup-common]` then
`[zephyr] [setup] [--skip-sdk]`; and `just --dry-run zephyr setup --skip-sdk`
shows `ARGS="--skip-sdk"` reaching `./scripts/zephyr/setup.sh $ARGS`. The
no-flag and two-flag shapes were checked the same way.

`check-workflow-setup-spelling`'s `EXEMPT` is now empty, and both of its
directions were re-mutated after the change: reverting one conversion fails it,
and adding an exemption that matches nothing fails it.

**One step removed per job, and only one.** The "Provision Zephyr sources via
nros" step (`nros setup --source zenoh-pico --source cyclonedds-src --source
px4-rs`) is the first thing `just/zephyr-setup.just`'s `setup` does — with an
explicit `--index`, and before the `$WORKSPACE/zephyr` short-circuit, so it runs
on every invocation. 10/10/9 steps -> 9/9/8.

**What did NOT go, and why**, since the W5 table counts ~6 provisioning steps
per job and only one was redundant:

| step | kept because |
| --- | --- |
| `./.github/actions/setup-nros-cli` | Its own docstring records the decision: `_setup-common` does all three of its steps, but clones `play_launch` in FULL where the action uses `--depth 1`, and it is the only thing that puts the CLI on `$GITHUB_PATH`. A clone-cost change into a lane with no signal cannot be verified. |
| Register the baked Zephyr SDK for this HOME | `--skip-sdk` exists precisely so setup does NOT touch the SDK. Nothing in `_setup-common` or the module recipe registers a CMake package. |
| Reclaim disk (#0078) | Container housekeeping, not provisioning. |
| Unblock rustup clippy-preview conflict / `rustup set profile minimal` | Image workarounds that `rustup target add` needs — and `_setup-common` runs `just workspace rust-targets`, so the conversion makes them MORE load-bearing, not less. |
| Install clang + libclang for bindgen | An OS package. phase-422 W6 has `_setup-common` REPORT the system closure and never install it. |

**Acceptance.** Met for the zephyr jobs. Not yet swept for the whole
`.github/workflows/` tree — `host-tests` and `queue` are still W5's rows.

### W5 — the ratchet

Two gates landed already and are the model:

- `check-preconditions-provisioned` — every tier precondition is classified
  setup/build/manual, and `setup` ones are reachable from `_setup-common`.
- `check-workflow-setup-spelling` — workflows invoke the dispatcher form.

One more is missing, and it is what W1 needs to stay fixed: **a gate asserting
no indexed tool has a second installer.** Shape: for each `[tool.X]`, no
`install-X` / `setup-X` recipe may perform its own download or stamp
comparison; forwarding to `nros setup --tool X` is fine.

**Acceptance.** Reintroducing `install-corrosion`'s own stamp logic fails a
gate. Both directions checked — a gate row naming a tool the index dropped
fails too.

**Landed 2026-09-06** as `check-one-producer-per-tool` (fast lane; the list is
derived, so it joined by existing). Mutation-verified against the stated
acceptance: putting `install-corrosion`'s download and untar back turns it red.

*It found one, on its first run.* `just workspace cargo-tools` still ran
`cargo install espflash --locked` beside `[tool.espflash]` — a second copy in
`~/.cargo/bin`, with no pin, against a store install pinned to `v4.5.0`. Both
reach PATH (`scripts/sdk-path-tools.txt` adds the store's bin dir), so which one
`just esp32 build-qemu` packs with was an accident of PATH order: issue 0500's
shape, in the one lane issue 1025 had already broken. It forwards now, and stays
non-fatal — espflash is only needed to FLASH, and a workstation that never
touches hardware should not fail its whole tool setup over it.

**The rule is LINE-granular, and the first draft was not.** Asking "does this
body name the tool and contain a producer verb" reported 25 problems, 25 of them
false — every script in the tree contains the word `nros`, `nros` is itself an
indexed tool since phase-431 W3, and a script that apt-installs something
unrelated then reads as a second producer of the CLI. Two more refinements came
from the same run: a line that only PRINTS is advice, not provisioning (an
`echo` naming `apt install gcc-arm-none-eabi`; a skip message quoting a colcon
package), and `_` closes an identifier, so `nros_check_skip` is a helper rather
than a naming of `nros`.

**What this gate is NOT about**, stated because the audit that produced it kept
finding them: `[prereq.*]` OS packages are a different class with a different
rule — composing the install command is nano-ros's job and running it is the
user's (RFC-0062), which `check-sysdep-remedies` enforces. The audit did find
one thing worth recording there, filed as
[#1128](../issues/archived/1128-prereq-apt-name-is-distro-parametric-and-the-index-cannot-say-so.md)
and **fixed**: `just ci provision-zenohd` composed
`ros-${ROS_DISTRO}-rmw-zenoh-cpp` itself because the index could only spell one
distro, and the two agreed only on humble. `{ros_distro}` makes the parameter
data, `check-prereq-placeholders` holds that vocabulary to one name, and the
recipe asks for the name instead of building it. And four cargo tools
(`cargo-nextest`, `cargo-llvm-cov`, `rustfilt`, `cargo-show-asm`) are installed
by `cargo-tools` and are NOT in the index, so nothing here judges them; whether
they should be is a W2-shaped question nobody has asked yet.

### W6 — the bootstrap actually pulls the system closure

Measured: 24 of the 46 `[prereq.*]` keys have no explicit consumer, nothing in
the repo INSTALLS via `--system`, and `just setup <scope>` never reaches it. On
a developer host that is 5 missing packages (`libgcrypt-dev`, `libpixman-dev`,
`make`, `ninja`, `openocd`) that no documented command installs.

The fix is NOT "run `--system --sudo` from setup": composing the command and
running it are deliberately separate (RFC-0062), and a provisioning verb that
sudo-installs behind the user's back is worse than the gap. What is missing is
that `just setup <scope>` never even PRINTS the closure, so the user is not
told. Make the scope verb report missing system prerequisites with the composed
command, the way `nros setup --system` already does.

**Acceptance.** `just setup <scope>` on a host missing a `role = package` or
`role = workspace` key names it and prints the install command. Nothing is
installed without an explicit `--sudo`.

### W7 — one board vocabulary — DONE (additively; unification still open)

`board=` in a `package.xml` export and `[board.*]` in the index were two
namespaces that did not line up: of five boards declared across `examples/`,
only `threadx-linux` was an index key. `nros setup --workspace` worked around it
by validating before printing a command, which is honest but is a workaround.

Same for `deploy=`: `threadx` is not a scope, it splits into `threadx_linux` /
`threadx_riscv64`.

**DECIDED (2026-09-05): additive first, unification deferred.** Add the missing
`[board.*]` entries so every exported board is ALSO an index key — no renames,
and it makes `nros setup <board>` work for all five instead of one. True
unification (picking one canonical spelling and renaming through 90+
package.xml files) stays open, because the index namespace is what USERS type
while the cmake namespace is what the BUILD uses, and neither is obviously the
one to keep. This is the two-vocabularies class this repo keeps paying for — the
`native`/`posix`/`linux` collapse, `[system.*]` vs `[prereq.*]`, the module vs
dispatcher setup spelling.

**Acceptance.** A gate asserts every `board=` in a package.xml export resolves
to an index board (or to a documented alias), and every `deploy=` resolves to a
scope.

### What landed

**The four missing index entries.** `mps2-an385-freertos`, `nuttx-qemu-arm`,
`nuttx-qemu-riscv` and `riscv64-qemu` are `[board.*]` keys now, each marked
`# = [board.<other-spelling>]` against the entry it duplicates. All five
exported boards resolve in both namespaces, and `nros setup <board>` works for
five of five instead of one.

**`check-board-vocabulary`**, on the fast line, with three assertions:

1. every `board=` resolves in at least one of the five namespaces (cmake board
   file, index, board crate, fixture coordinate, scope);
2. every `board=` is specifically an **index key** — the namespace
   `nros setup <board>` looks up, exact-match with no fallback;
3. the four `# =` mirrored pairs are byte-identical in `arch`, `platform` and
   `packages`. The index tells readers to edit both; nothing asserted it, which
   is the issue-0196 class (a gate credited with a rule wider than the one it
   enforces).

Assertion 2 is the one that tests what W7 created. Assertion 1 alone was
satisfied by `cmake/board/*.cmake` for every value, before and after the index
entries existed — measured A/B, identical OK line either way.

`deploy=` resolves as a scope or splits into `<deploy>_*` scopes by board, which
is how `threadx` is legal: it is a deploy FAMILY, and the `board=` beside it
selects `threadx_linux` or `threadx_riscv64`. `check-board-alias-unique`
(phase-435 W5) enforces the same distinction from the descriptor side.

**A silent hole in the doctor surface, found by running the command.**
`nros setup <board> --check` dropped the board argument: the arm returned before
the board was ever looked up, so `nros setup not-a-board --check` printed the
same all-clear as `nros setup threadx-linux --check` and exited 0. The one
command a user runs to ASK whether a board is provisionable answered yes for a
name that is not a board — while the same typo without `--check` was a clear
error listing the known boards. The board is validated now, and the notice says
what `--check` actually covered rather than implying the board narrowed it (it
walks `[system.*]`/`[rust.*]`/`[python.*]`, a different axis from a board's
package set; narrowing that is a doctor redesign, not this fix).

**Still open:** true unification — picking one canonical spelling and renaming
through 90+ package.xml files. The index namespace is what USERS type, the
cmake namespace is what the BUILD uses, and the mirrored pairs are the cost of
deferring: two entries that a gate now keeps identical rather than one entry
with an alias key.

### W8 — refuse a wrong-role dependency — DONE (scoped to `infra`)

**Landed, split in two, and the cheaper half mattered more.** The unresolved-dep
error was itself teaching the defect: it told users to add a `[prereq.*]` key
with no mention of role, so a build failing on `<depend>qemu-system-arm</depend>`
was instructed to make it resolve. Fixed first.

The refusal covers `role = infra` only. `workspace` and `vendor` are NOT
refused — a package building against a vendored source tree naming it is
arguable, and refusing them risks more than it buys.

No warn-first window for `infra`, deliberately: MEASURED, zero packages in this
tree name a non-`package` key, so there is nothing in-tree to migrate and a
window would only delay the fix while the misleading message kept shipping. A
window still makes sense for `workspace`/`vendor` if they are ever refused.

`NROS_ALLOW_INFRA_DEPS=1` is its own hatch, not a reuse of
`NROS_ALLOW_UNRESOLVED_DEPS`: a wrong-role dep resolves and we decline it, so
overloading one variable would make silencing one silence the other.

Original text:

Deferred by decision, not oversight. `<depend>qemu-system-arm</depend>` resolves
silently today; `nros setup --workspace` reports it as a category error without
failing. Turning it into a hard error breaks a working tree, so it needs a
deprecation window: report -> warn -> error.

**Acceptance.** A package.xml naming a `role = infra` key fails resolution, and
the error names the deploy target it should have come from instead.

## Non-goals

- **Not** replacing the CI image's baked tooling. The image is a cache; the
  index is the contract. `ci/docker/ci-base/Dockerfile` reading
  `config/rust-targets.txt` (#365) is the pattern — bake from the SSoT, do not
  hand-list.
- **Not** moving `setup-cli`, `setup-launch-resolve` or `setup-hooks` into the
  index. Those build in-tree code or configure the user's git; they are not
  third-party artifacts and have no version to pin.
- **Not** touching `[prereq.*]`. OS packages are RFC-0062's namespace and
  phase-413 W3 already gated workflows against restating them.
