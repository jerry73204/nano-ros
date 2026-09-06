# nano-ros gates: workflows
#
# CI workflows, provisioning, the SDK index, and the runner fleet.
#
# One of the topic files `just/check.just` imports. The names stay FLAT -- a
# gate is still `just check <name>` -- because `import` merges definitions
# rather than namespacing them, and ~350 references across docs, scripts and
# workflows spell them that way.
#
# Adding a gate here does not touch any other file, which is the point: for
# most of this repo's life every new gate appended to ONE file, and
# `just/check.just` became the worst conflict surface in the tree at 48 of
# 300 commits.

# issue 0372 — `activate.sh` / `activate.fish` are SOURCED, so anything that
# aborts them mid-file silently drops every export below the failure. Two
# unmatched SDK-store globs did exactly that under zsh (fatal `nomatch`), on
# empty stores AND on the versioned layout `nros setup` writes, while no lane
# sourced either file under a non-bash shell. This gate sources both in every
# available shell against both store shapes and asserts they reach their last
# line. zsh/fish absent = loud skip, never a silent pass.
[private]
activate-shells:
    @bash scripts/check-activate-shells.sh

# zenoh-pico's `config.h` is BOTH a cmake-generated artifact (configure_file
# writes it into the SOURCE tree) and a committed, hand-maintained file that is
# the only config nano-ros reads. Being both is how it drifted three ways at
# once — dropped `#ifndef` guards that made every `-D` inert, three of our own
# features missing from the header, and a `#ifdef ZENOH_NUTTX` split the
# template could not express. Phase 415 made generation lossless; this keeps it
# so. Same shape as RFC-0054's committed bindgen output plus
# `check-abi-bindings`. Offline: no cmake, no configure.
# `ros2-box-sync.sh` excludes build-output directories by NAME, and the tree has
# tracked SOURCE whose names match. That collision has eaten tracked files FOUR
# times, each found as a build failure inside the box naming something else —
# most recently `packages/cli/build-support/`, which left the box unable to
# compile `nros` at all. Anchoring fixed one instance and directory-only
# patterns fixed another; neither fixed the class.
[private]
box-sync-covers-tracked-source:
    @python3 scripts/check-box-sync-covers-tracked-source.py

# The reverse of `check-sysdep-remedies` — phase-413 W3.
#
# That gate refuses a hand-written native install command in a `just` recipe,
# and `[prereq.doxygen]`'s own `why` reads "found undeclared by
# check-sysdep-remedies".
#
# DESCRIBED, not quoted — `check-sysdep-remedies` is a text scan and cannot tell
# a command from a sentence about one, so a comment naming the forbidden
# spelling fails it. `provision-zenohd`'s comment says exactly this and takes
# exactly this way out; writing that phrase here anyway cost this PR a red.
#
# Nothing guarded the other direction: a WORKFLOW may
# not restate a package `[prereq.*]` already declares. Three did — doxygen and
# graphviz in `docs.yml`, clang and libclang-dev in `nightly.yml` — each with
# its dnf/pacman/brew spellings and a presence probe sitting in the index.
#
# Buildless and source-only, so it belongs on the fast line.
# A workflow must provision with `just setup <scope>`, not `just <scope> setup`.
# Only the dispatcher runs `_setup-common`, where the host facts every tier
# asserts get provisioned; the module spelling skips it silently and the lane
# fails later on a precondition nothing provisioned. nightly's platform job did
# exactly that and carried a hand-rolled cross-target step to compensate.
#
# The sibling of `check-preconditions-provisioned`: that one asks whether setup
# PROVIDES the facts, this asks whether workflows INVOKE the form that runs them.
#
# phase-422 W3 added a second arm over PROSE — book/, AGENTS.md, CLAUDE.md, the
# living `docs/` subtrees and every README — because 64 occurrences across 38
# files taught the module spelling to readers. The three RECORD series
# (`docs/roadmap/`, `docs/issues/`, `docs/research/`) are deliberately NOT
# scanned: they narrate what a command DID at a time, so gating them would be a
# steady false-positive stream. The recipe keeps its `workflow-` name because
# the workflow arm is still the one a red usually means.
# phase-431 W2 — CI builds the CLI from source and PROVES it, and never
# installs the release. Two directions: no workflow acquires a release asset
# (a released binary emits its own codegen, so such a job stops testing this
# tree while staying green — and RFC-0090's version does not catch it, because
# a release at the same version is different, not incompatible), and every
# from-source build asserts `nros source-stamp` in the SAME step (phase-429 W5;
# the CLI cache key is narrower than the stamp's watch set). The second arm
# found `gate.yml`'s CLI warm-up asserting nothing, four sites into a five-site
# rule.
[group("checks")]
ci-cli-from-source:
    @python3 scripts/check-ci-cli-from-source.py

# A CI doc must not cite a workflow file that does not exist. Same class as
# `check-just-recipe-refs`: a reference that silently stops resolving, in a
# place that still reads as current.
[private]
ci-doc-workflow-refs:
    @python3 scripts/check-ci-doc-workflow-refs.py

# The zephyr CI image must install every Python module its own checker demands
# (issue 0878). It did not, so all 21 Zephyr nightly cells failed at
# `just zephyr setup` before any build — and a uniformly red lane cannot report
# a regression, which is how issue 0876 rode in.
[group("checks")]
ci-image-python-deps:
    @python3 scripts/check-ci-image-python-deps.py

# CI may not set NROS_FIXTURES_OPTIONAL. A lane names its scope, so a missing
# fixture INSIDE that scope is a failure; the flag turned it into a skip and made
# correctness depend on remembering to unset a variable (phase-411 W4). The
# reader survives as a local opt-in — the rule is about WHERE it is set.
[private]
ci-no-fixture-tolerance:
    @python3 scripts/check-ci-no-fixture-tolerance.py

# phase-411 — a CI step must not fall back from one `just` verb to another.
# `build-all || build-examples || build` made a FAILED verb build something
# smaller and report success, which is the "skip and report success" shape
# phase-411 removes, written in shell instead of a skip helper. Buildless: YAML
# plus a regex, and it skips comments and heredocs for the reason
# `check-workflow-repo-env` documents.
[private]
ci-no-verb-fallback:
    @python3 scripts/check-ci-no-verb-fallback.py

# The claim CAS, proven against a throwaway bare repo — hermetic, no network,
# ~2 s. On the fast line because a coordination primitive that cannot fail is
# worse than none: it would report agreement it never established.
[private]
claim-protocol:
    @bash scripts/reserve-claim.sh --selftest

# Issue 0926 / phase-327 W4 — a dist's `system = [..]` must cover the libraries
# it ACTUALLY needs, re-measured from the provisioned dists themselves.
#
# NOT on the fast line, and not in an affordability tier: it needs a provisioned
# store, which no such lane builds (`check-lane-contracts`). It SKIPS loudly
# when there is no store, so running it anywhere is safe — it simply cannot
# answer without its input, and says which.
#
# `just doctor` is its home: doctor is where host state is examined, and this is
# the one gate whose subject lives in `~/.nros/sdk` rather than in the tree.
[private]
dist-runtime-deps:
    @python3 scripts/check-dist-runtime-deps.py

[private]
package-xml-comments:
    @bash packages/testing/nros-tests/tests/package_xml_comment_stripping.sh

# RFC-0087 D3 / phase-420 W1 — `<nano_ros_uses kind= name=/>` and the
# `<nano_ros deploy= board= rmw=/>` sugar must mean the same thing to the cmake
# reader, and `deploy=` must NOT become a selection (it names a system.toml
# block, not a provider). Divergence between the two spellings would be
# invisible: both still configure, against different values. Buildless.
[private]
package-xml-uses:
    @bash packages/testing/nros-tests/tests/package_xml_uses_general_form.sh

# Every lane that runs the peer-spawning suites sweeps first (issues 0659,
# 0923). The reaper's own unit tests call `sweep_in` directly, so they cannot
# see whether a LANE calls it — `just test` ran the same suites without one and
# left 67 orphans holding DDS ports.
[private]
peer-sweep-lanes:
    @bash scripts/check-peer-sweep-lanes.sh

# Issue 0617 — an RTOS `platform-*` feature must SUPPLY malloc and panic. A
# no_std final artifact needs exactly one `#[global_allocator]` and one
# `#[panic_handler]`; a HOST build cannot catch a missing provider because
# `std` supplies both, which is how NuttX shipped with neither.
platform-provider-features:
    @python3 scripts/check-platform-provider-features.py

# Every tier precondition must have a provisioner, and it must be reachable
# from `just setup <scope>`. `check-tier-preconditions` asserts host facts;
# `_setup-common` provisions them; nothing kept the two lists in agreement, so
# the gaps surfaced one 40-minute CI run at a time (issue: host-tests tier1 ->
# three unmet preconditions -> then clang-format one layer deeper).
#
# Classifies each precondition as setup / build / manual and checks the map in
# BOTH directions, because a map checked one way drifts silently.
[group("checks")]
preconditions-provisioned:
    @python3 scripts/check-preconditions-provisioned.py

[group("checks")]
prereq-roles:
    @python3 scripts/check-prereq-roles.py

# issue 0516 — a COMMENTED-OUT element in a package.xml must not read as a
# declaration. cmake has no XML parser, so every reader here regexes raw text;
# before the `nros_read_package_xml_body()` helper, a package.xml that merely
# DOCUMENTED a tag declared it. Covers the greedy-strip failure mode too, where
# `<!--.*-->` silently deletes everything between two comments. Buildless.
# phase-348 W2 — a provider announces itself (package.xml) and declares what it
# lowers to (its descriptor). Nothing structural keeps the two name lists equal,
# so compare them. ONE gate for every provider family: a copy of the rule beside
# each family's descriptors is the second-spelling antipattern (#282 -> #326).
#
# Catches: a descriptor with no `<nano_ros_provides>` beside it (invisible to
# the phase-348 scan while looking migrated); a name list that disagrees with
# the descriptor's, or is in the wrong order, for the three families whose
# descriptors still carry `names`; and, for a family whose descriptor carries
# none (`serdes`, RFC-0087 D4 — the announcement is the only spelling), two
# providers announcing one name. phase-421 W4 added that fourth family as one
# FAMILIES row, not a second script.
[private]
provider-announcements:
    @python3 scripts/check-provider-announcements.py

[private]
provider-index:
    @bash packages/testing/nros-tests/tests/provider_index_gate.sh

# A `ps` scan enumerating process-GROUP membership must exclude zombies — a
# corpse keeps its pgid and, under a PID 1 that never reaps (a GitHub Actions
# container job: `tail -f /dev/null`), never leaves `ps`. Issue 0853: three
# sites had the idiom and all three were wrong.
[private]
ps-zombie-blind:
    @bash scripts/check-ps-zombie-blind.sh

# Issue 0763 — a test's ROS 2 environment is spelled in ONE place
# (`nros_tests::ros_env::RosEnv` + `Middleware`, RFC-0058 / phase-309, over the
# `ros2::ros2_env_setup_*` helpers). A hand-rolled `source /opt/ros/<distro>/
# setup.bash && export ...` elsewhere is a second place, and every second place
# has drifted invisibly: one dropped the peer's ROS_DOMAIN_ID (the first segment
# of an rmw_zenoh keyexpr, and the key ros2cli's daemon is singleton on),
# another opened with a `ros2 daemon stop` that killed the daemon a parallel
# test was mid-query against, another hardcoded `humble` so every guarded test
# SKIPPED forever on a jazzy host. The gate does not forbid the exception — it
# makes adding one a reviewable line in a diff. Buildless: greps tracked source.
[private]
ros-env-spelling:
    @python3 scripts/check-ros-env-spelling.py

# Issue 0980 — `runner-doctor` answered "where is the Zephyr SDK?" from the job
# CHECKOUT while the build answers it from the CMake user package registry.
# `scripts/zephyr/sdk/` is gitignored and `actions/checkout` runs
# `git clean -ffdx`, so a self-hosted runner's SDK is necessarily somewhere
# else — which made the derived path wrong by construction and failed every
# merge-group `L3` job on a host that builds fine.
#
# Same shape as the gate above, for the same reason: the self-test lives in the
# script so the rule and its cases cannot drift apart, and it stages temp dirs
# only (no SDK, no cmake, no network), so it belongs on the fast line.
[private]
runner-doctor-sdk-resolution:
    @bash scripts/ci/runner-doctor.sh --self-test

# issue 1094 — `runner-sweep` must evict the least recently WRITTEN entry, not
# the one with the oldest directory inode. A cargo `--target-dir` writes several
# levels down, so a shared fixture group's dir mtime freezes near creation while
# the group is used daily: eleven of twenty `cargo-fixtures` children were wrong
# by ~20 days, and the sweep proposed deleting the 12.8 GiB NATIVE group — used
# the previous day — to reclaim a 60 MiB overrun. Silent either way; the cost
# shows up as the next job rebuilding a fixture nobody touched. ~1 s, tmpdir.
[private]
runner-sweep-eviction-order:
    @bash tests/runner-sweep-eviction-order-tests.sh

# Issue 0833 — every Rust cross target DECLARED in-tree (board nros-board.toml,
# cmake/toolchain Rust_CARGO_TARGET, leaf .cargo/config.toml [build] target) has
# a row in config/rust-targets.txt, the list both `just workspace rust-targets`
# and `just doctor` read.
#
# The shared list fixes the divergence that existed; this fixes the one that
# would happen next. A board can land a new triple in three different files and
# none of them is the provisioning recipe — armv8r-none-eabihf was declared by
# two boards, one toolchain file and two fixture rows while the doctor's own
# copy of the list had never heard of it, so `just doctor` said [OK] and `just
# freertos build-fixtures` failed at cmake configure with an error four layers
# from its cause.
[group("ci")]
rust-targets-covered:
    @python3 scripts/check-rust-targets-covered.py --self-test
    @python3 scripts/check-rust-targets-covered.py

# phase-411 W3 — `just <verb> <scope>` puts platform names and preset names in
# ONE argument position, so a name may not mean two things. `native` is both a
# platform module and a fixture lane, which is legal only while they denote the
# same scope; nothing checked that, and a preset that quietly means something
# else re-scopes a run without changing a single command anyone types. Buildless
# (NROS_SCOPE_NO_BUILD=1) — a collision whose expansion needs a compiled
# selector is itself the failure. Self-tests its own rule on every run.
[private]
scope-namespace:
    @python3 scripts/check-scope-namespace.py

# A `-z "${VAR:-}"` guard on a variable sdk-env.just always exports can never
# fire. Six did, and they broke the UNUSUAL way: an intended skip became a cmake
# failure on an unprovisioned host (issue 0955).
[private]
sdk-guard-can-fire:
    @python3 scripts/check-sdk-guard-can-fire.py

# Verify nros-sdk-index.toml + the QEMU configure flags
# (the `sdk-index` job in .github/workflows/gate.yml). Buildless + fast.
#
# The third one is issue 1006: a qemu recipe with no `dist` is source-built by
# EVERY user, so a backend it leaves auto-detected makes the runtime dependency
# set a property of that user's machine. `check-dist-runtime-deps` can only see
# that after a provision (and CI never provisions esp32-qemu), so the static
# half lives here.
[group("ci")]
sdk-index:
    python3 scripts/sdk/verify-index.py nros-sdk-index.toml
    ./scripts/sdk/check-qemu-configure.sh
    python3 scripts/sdk/check-qemu-source-features.py nros-sdk-index.toml

# phase-365 W4 / issue 0625 — the SDK store is CONSTRUCTED from the project's
# pin, never enumerated. A wildcard where the version belongs means a consumer
# is picking a version by searching a store SHARED between projects, to answer a
# pin that is PER-PROJECT.
sdk-store-not-enumerated:
    @python3 scripts/check-sdk-store-not-enumerated.py

# phase-348 W3 — the provider index and the cmake seam that reads it. cmake asks
# the CLI for TAB rows rather than parsing the index (no second parser to
# drift), every recorded package.xml is watched for reconfigure, and a
# provider added AFTER the index was written is caught by rescan-and-compare —
# the case no file watch can cover (issue 0196's shape). Needs `just setup-cli`.
# phase-351 W2 — this repo's site config (`[deploy.<n>.nros]`) agrees with
# `just/sdk-env.just`. Both spellings are live during the migration, so the gate
# is what stops them drifting (the phase-347 pattern). `--write` renders missing
# blocks; generator and gate are one file so they cannot disagree.
[private]
site-config:
    @python3 scripts/check-site-config.py

# issue 0762 — a killed build launcher must take its whole subtree with it.
# Drives real process trees and asserts on `ps`: the orphan bug is silent by
# construction (the terminal comes back while the build keeps running), so
# nothing short of watching the processes can catch a regression.
[private]
subtree-guard:
    @bash packages/testing/nros-tests/tests/subtree_guard.sh

# Support-status page (tiers, RTOS/RMW/toolchain pins, editions) — version
# strings parsed from nros-sdk-index.toml + rust-toolchain.toml so a pin bump
# moves the page in the same commit.
[private]
support-status:
    @python3 scripts/gen-support-status.py --check

# phase-327 W3 (issue 0368) — no hand-written sudo-apt remedy text in just
# recipes; the [system.*] index class + `nros setup --system` own that text.
sysdep-remedies:
    @bash scripts/check-sysdep-remedies.sh

# A submodule pin may only move FORWARD. Every submodule here keeps linear
# history on its branch, so a bump is a fast-forward to a descendant; a rewind
# silently unships whatever the skipped commits fixed, and `-Subproject commit
# <hex>` is not something a reviewer can order by eye. 2026-08-15: a 24-file
# commit about issue-ID renumbering moved zenoh-pico BACK over a Zephyr
# `socklen_t` build fix, and nothing noticed for seven hours.
# Deliberate rollback: NROS_ALLOW_SUBMODULE_REWIND=1 (and say why).
# Every tier RFC-0061 declares must be RUN by some workflow. Two of four were
# invoked by nothing at all, and a tier nobody runs cannot go red to say so
# (phase-411).
[private]
tier-has-ci-owner:
    @python3 scripts/check-tier-has-ci-owner.py

# RFC-0087 D5 / phase-420 W8 — a fetch without a digest is the same defect as an
# unpinned submodule, one mechanism over. `check-submodule-pins` can go and ask
# the submodule because a gitlink IS a full commit id; a `GIT_TAG v0.6.1` names
# a ref on someone else's server, and if they move it this tree switches trees
# with NO local diff at all. The two gates partition the surface: gitlink there,
# CMake argument here.
#
# Rejects a short sha (a prefix the REMOTE resolves, not an identity), MD5/SHA1
# (forgeable), and `URL_SHA256` — which is not a CMake option at all, so CMake
# takes it as an unknown keyword and verifies nothing while reading as more
# careful than the correct spelling.
#
# Reports NOTHING TO CHECK with the population searched when a rule has no
# subject, because "the rule holds" and "the rule has no subject" are different
# facts. Buildless, ~0.15 s.
[private]
vendor-fetch-pinned:
    @python3 scripts/check-vendor-fetch-pinned.py

[private]
version-lockstep:
    @./scripts/check-version-lockstep.sh

# phase-413 W2 — a job asserts its runner's labels AFTER `just setup`, never
# before. `runner-doctor.sh` looks for `build/qemu/bin/qemu-system-arm` FIRST
# and `just setup tier2` is what creates it, so doctoring first asks the host
# for what the next step supplies. The defect had three sites, #279 fixed one,
# and the other two were still live months later because nothing was watching —
# which is the whole argument for the gate rather than a third fix.
[group("main")]
workflow-doctor-after-setup:
    @python3 scripts/check-workflow-doctor-after-setup.py

[group("main")]
workflow-indexed-apt:
    @python3 scripts/check-workflow-indexed-apt.py

# issue 0933 — a CI step that invokes `just`/`nros`/`west` must source the repo
# environment. `activate.sh` is what exports `nano_ros_ROOT`, and that is how
# `find_package(nano_ros)` resolves: the config lives at the checkout root and a
# generated workspace root sets no prefix on purpose. `just doctor` enforces this
# for a developer shell; nothing enforced it for a CI step, which is the one
# place the shell is fresh every time.
#
# Pure YAML + regex over `.github/workflows/`, so it belongs in the source-free
# tier. It skips comment prose AND heredoc bodies — both false positives were
# live in the tree (a comment mentioning `nros sync`; a pull-request comment
# whose TEXT tells the author to run `just queue-triage`).
[private]
workflow-repo-env:
    @python3 scripts/check-workflow-repo-env.py

# nano-ros is PUBLIC and the self-hosted runners share machines with unrelated
# research work. A self-hosted job reachable from a fork's `pull_request` runs
# contributor code on that hardware. The property held by care alone until
# phase-407 — this makes a `runs-on` typo fail loudly instead of silently.
[private]
workflow-runner-isolation:
    @python3 scripts/check-workflow-runner-isolation.py

[group("checks")]
workflow-setup-spelling:
    @python3 scripts/check-workflow-setup-spelling.py

# A hook must leave the repository it guards byte-identical — issues 0986/0988.
#
# The three scripts above build throwaway repos in their selftests, and an
# inherited GIT_DIR overrides BOTH a path argument and `git -C`, so they ran
# against the CALLER's repo instead: `core.bare = true` written into its config,
# a gitlink with an invalid sha staged into its index — by the hook whose job is
# refusing bad submodule pins. Nothing could see it, because the damage does not
# reproduce by hand (an interactive shell has no GIT_DIR) and it is self-masking
# (run 2 dies before reaching the cause).
#
# So this runs them, and the hook, under the environment git actually supplies,
# against a victim repo compared byte for byte including mtimes — which matters
# here beyond correctness: anything that rewrites a tracked file re-stales every
# prebuilt fixture in the tree. ~6 s, no network, no build.
[private]
hook-repo-side-effects:
    @bash scripts/check-hook-repo-side-effects.sh
