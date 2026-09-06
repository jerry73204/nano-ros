# nano-ros gates: codegen
#
# Generated code and the headers that describe it -- codegen inputs,
# message emission, entry emitters, and the per-build sizes header.
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

# `--self-test` (issue 0661) drives both era verdicts over a synthetic tree,
# because the failure mode here is a wrong VERDICT rather than a wrong count:
# the early widening branch used to print "counts ALL … an accumulated tree can
# inflate it" and then, from an uninitialised flag, "accumulation is ruled out".
# A reader who believed the second one went looking for a regression that was
# not there, which is how 0661 spent its length on two ordinary host
# build-dependency artifacts.
[private]
artifact-identity-budget:
    @bash scripts/check-artifact-identity-budget.sh

# phase-412 -- the boot self-report is decoded POSITIONALLY out of a memory
# dump, so a field reordered on one side and not the other does not fail: it
# decodes, and prints an arena size that is really a callback count.
[private]
boot-report-layout:
    @python3 scripts/check-boot-report-layout.py

# Canonical `nros codegen` invocation-shape guard. The `check.yml` step (SSoT).
[private]
codegen-invocation:
    @scripts/ci/codegen-invocation-check.sh

# issue #202 — run the CLI sub-workspace's test suite (unit tests across
# nros-cli-core / rosidl-* / nros-build + the plan-pipeline e2e). Before this
# lane existed NOTHING ran `cargo test` on packages/cli — the orchestration
# e2e suite sat 17/17 red for months without any lane noticing (the #181
# silent-lane class). The metadata-mode tests compile tiny probe crates at
# runtime BY DESIGN (the verb under test is a compile-driver; see
# nros-cli-core/tests/plan_pipeline_e2e.rs).
# Issue 0981 — the two codegen tests that pin the derived size bound, lifted out
# of `cli-tests` and onto the FAST line so they gate a merge.
#
# `cli-tests` below runs the whole `packages/cli` workspace, including a
# plan-pipeline e2e that compiles probe crates at runtime; it costs minutes and
# needs the play_launch submodule, so it lives on `check::build` — which is
# `schedule`/`workflow_dispatch` only in CI. No pull request and no merge group
# runs it. That is how `codegen_golden` sat red on `main` for a day: the
# required `CI` context is `check-fast` + `test-unit`, and both were green on
# the commit that broke it.
#
# These two need none of that. They are pure functions of the corpus and the
# emitters — measured 8.1 s from a COLD target dir, 0.1 s warm, inside the fast
# line's existing spread. So the narrow thing moves and the expensive suite
# stays put; promoting `cli-tests` wholesale would put a runtime-compiling e2e
# on the merge-gating lane, which `check-lane-contracts` exists to forbid.
[private]
codegen-size-bound-golden:
    @cargo test --manifest-path packages/cli/Cargo.toml -p rosidl-codegen \
        --test codegen_golden --test message_size_bound_parity --quiet
    @echo "codegen size-bound golden + C/C++/Rust parity: OK"

# Issue 1018 -- the `generated/` regeneration stamp keys on what the CLI EMITS.
#
# `scripts/build/codegen-stamp.sh` decides whether a leaf's cached `generated/`
# tree survives. It watched `nros-core/src/action.rs` and nothing about the
# emitters, so an edit to `rosidl-codegen` left the tree in place -- and in
# `just/zephyr-ci.just`, the one lane whose `nros sync` is conditional, the leaf
# then compiled message crates the PREVIOUS CLI emitted.
#
# The term added is `nros codegen-fingerprint`, not the binary: 168 distinct
# `nros` binaries on this host produced 11 distinct fingerprints, so a
# binary-keyed stamp would wipe every leaf on 157 rebuilds that emit identical
# code (phase-424's constraint). Both halves are asserted, because either alone
# has a trivial wrong implementation, and the pre-fix rule runs on the same
# trees as the negative control.
[private]
codegen-stamp-inputs:
    @./tests/codegen-stamp-tests.sh

# A configure-time emitter must make a rebuilt `nros` re-run its configure
# (issue 1018). The build-time generator says so in `DEPENDS`; `execute_process`
# cannot, so the tool goes in `CMAKE_CONFIGURE_DEPENDS` via
# `nros_codegen_tool_reconfigure()`. #182 registered it at ONE of four sites,
# inline, so the other three kept museum generated code after a `just setup-cli`
# unless the image also happened to call `nano_ros_entry()`.
[group("checks")]
codegen-tool-reconfigure:
    @python3 scripts/check-codegen-tool-reconfigure.py

# RFC-0090 / phase-429 W5 — the codegen-version check must FIRE.
#
# `check-fast` proves the tree is green. Only a negative control proves the
# mechanism is armed, and this one guards the failure that stopped nano-ros
# shipping a prebuilt `nros`: a binary emitting code the runtime does not
# accept, which COMPILES and is wrong.
#
# Three arms, from the REAL emitted artifact rather than a hand-written
# imitation — the block is extracted verbatim from a committed golden header, so
# a template edit that dropped the check takes these with it. In range compiles;
# out of range is refused in both C and C++; a config header that defines
# neither bound is refused as MISSING rather than as out-of-range, because an
# undefined macro reads as 0 to `#if` and would otherwise produce a range error
# about a version nobody set.
#
# The Rust half is NOT covered here — see the script's header for why, and for
# the by-hand verification that stands in for it.
[private]
codegen-version-refusal:
    @./tests/codegen-version-refusal-tests.sh

# Issue 0985 — the per-build sizes header must have ONE writer, the mirror
# script, which since issue 0978 knows that the leaf's own copy is present and
# arbitrarily old once leaves share a cargo target dir.
#
# A second writer does not merely duplicate that precedence, it DEFEATS it:
# `nros-cpp/CMakeLists.txt` healed the mirror at configure time with a plain
# `file(COPY_FILE)` from the leaf's binary dir, wrote a museum header over a
# correct one, and stamped it with a new mtime — so ninja found the mirror's
# OUTPUT newer than its trigger and skipped the command that would have fixed
# it. A repair for drift that caused drift and suppressed its own fix.
# Static, no build. The whole 0088 -> 0114 -> 0122 -> 0123 -> 0245 -> 0268 ->
# 0978 -> 0985 family is one shape: two ways to answer "which bytes are the
# current sizes header".
[private]
config-header-single-writer:
    @python3 scripts/check-config-header-single-writer.py

# `nros sync` skips the leaf with a count instead of a name. Buildless.
# RFC-0087 D4 / phase-420 W5 — a descriptor carries only what no convention can
# produce, so a field it DOES state must equal the value convention derives.
# Six of eleven rmw fields were convention written longhand; five are now gone
# and this keeps the rest honest. A ratchet: `crate` is grandfathered for
# `nros-rmw-xrce`, which ships no Cargo.toml and names a sibling — the
# measurement that showed D4's own table was wrong about `crate` being
# derivable. Buildless, ~0.2 s.
[private]
derived-descriptor-fields:
    @python3 scripts/check-derived-descriptor-fields.py

# Tool-emitted messages must not prescribe a bare `just` recipe — users do
# not have it; the front door covers both binaries. Found live: the exact
# error a fresh user hits first said `just setup-launch-resolve`, and a
# threadx board error named `just setup-threadx`, a recipe that does not
# exist. Same class as check-book-no-just, one layer down.
[private]
emitter-just-spelling:
    @bash scripts/check-emitter-just-spelling.sh

# phase-403 W9 (issue 0965) -- the READER for the ENTITY inventory, the second
# source `NanoRosMessageBounds.cmake` says by name that it needs.
#
# The derivation itself is unit-tested in `nros_cli_core::entity_inventory`
# (`cargo test -p nros-cli-core`), including the island's 33-entities /
# 19-slots case. This gates what CMAKE owns: a refusal publishes NO number, an
# unknown schema FATALs rather than being read field-by-field, a missing input
# refuses rather than breaking a clean-tree configure, and a broken declaration
# is fatal and names the component. The CLI is STUBBED, so this needs cmake and
# nothing else -- no cargo build, which is what keeps it affordable on the fast
# line beside its sibling above.
entity-inventory-knobs:
    #!/usr/bin/env bash
    set -euo pipefail
    if ! command -v cmake >/dev/null 2>&1; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip entity-inventory-knobs "missing host tool(s): cmake"
        exit 0
    fi
    ./tests/cmake-entity-inventory-tests.sh

# issue 0946 — the connect locator has exactly ONE producer.
#
# `NROS_ENTRY_LOCATOR` (the address an embedded image dials) was computed by two
# independent per-platform ladders that DISAGREED on threadx and freertos, one of
# which carried the note "Mirrors NanoRosNodeRegister.cmake freertos branch; keep
# in sync" — a hand-sync instruction with nothing enforcing it. The invariant is
# ONE PRODUCER, not equal literals: the lanes cite different networks and may
# each be right, so the gate demands one table rather than one value. Drift is
# silent (a wrong locator builds and links; the image just connects nowhere), so
# this needs a check and not a convention. Buildless — pure source analysis.
[private]
entry-locator-ssot:
    @python3 scripts/check-entry-locator-ssot.py

# Issue 1003 / 1017 — every emitted `run_components` call must NAME a session.
#
# Ten CMake entry templates passed only the setup fn, which binds the overload
# that hard-codes `"node"`. Every image built through `nano_ros_add_node` then
# registered under that name, and the XRCE client key derives from it — a talker
# and a listener hashed to ONE client. It survived from 2026-06-13 to 2026-09-03
# with a CORRECT sibling producer beside it (`emit_cpp.rs` has passed the
# boot-config name since 2026-06-27): two spellings of one fact, uncompared.
#
# The check covers BOTH producers, and deliberately does NOT know the per-board
# overload sets — encoding those here would make this a third authored copy,
# which is the defect it guards. See the script's header.
entry-session-name:
    @python3 scripts/check-entry-session-name.py

# phase-432 W3.3 — an entry pack that is half-wired must fail LOUDLY rather
# than emit nothing. "Adding a language is cheap" is only good news if adding a
# BROKEN language is not equally cheap: a pack directory with no manifest, a
# language pack missing the fields CMake needs to name its output, a `Language`
# variant nothing renders, or a language whose generated bytes are recorded in
# no golden. Each of those builds green today and fails on someone's board.
entry-pack-conformance:
    @python3 scripts/check-entry-pack-conformance.py

# A leaf that consumes its own `generated/` must be visible to
# `scripts/regenerate-bindings.sh` (which globs tracked package.xml), or its
# bindings freeze silently — eight test bins had frozen several phases back.
generated-leaf-regenerable:
    @bash scripts/check-generated-leaf-regenerable.sh

# phase-330 W7.e — committed SystemModels are BANNED: the model is a build
# artifact (generated into <ws>/build/nros/models by `nros sync`); tracking
# one re-opens the issue-0380 hand-edit/regeneration conflict. Supersedes
# check-model-dims (W5.b: the dim baseline protected committed files that no
# longer exist; `nros ws model-dims` remains for inspection).
# issue 0571 — a matrix consumer the lane filter cannot reach by NAME must
# narrow its own cell list (`nros_tests::lane_scope::admits`). Four consumers
# are ONE generically-named test each over every platform's cells, so
# `scripts/test/lane-filter.sh native` excludes nothing of theirs: tier 1 boots
# whatever images exist, and the cells whose images do not exist vanish into a
# green. Buildless.
# phase-351 W5 — every cargo target cmake creates must receive the resolved
# board facts + site config. The failure this guards is NOT a wrong value but no
# value, defaulted, with no diagnostic (issue 0529's shape) — which is how the
# board rung stayed dead from phase-290 to here. Buildless.
# issue 0606 — every `[deploy.*].board` resolves to exactly ONE descriptor. The
# field carries the DOWNSTREAM ecosystem's board id, so the descriptor covering
# it must claim that spelling; otherwise the deploy resolves to nothing and
# Phase 380 W0 — every COMMITTED generated message must carry `Message::FIELDS`.
#
# The schema is what `nros_serdes::size` computes a serialized-size bound from,
# and a missing one is SILENT: phase-380 W4's build-time buffer assertion has
# nothing to check and passes, so the subscription drops samples at runtime
# instead. Measured before W0: 27 of 64 structs had none, purely because three
# crates predated the schema emitter and nobody said so.
[private]
generated-schema-coverage:
    @python3 scripts/check-generated-schema-coverage.py

# phase-363 (W2's class) — `file(GLOB)` over `.msg`/`.srv`/`.action` captures the
# interface set at CONFIGURE time, so a newly added message is invisible until
# an unrelated reconfigure and the build ships the OLD generated sources. W2
# fixed the file it was looking at; the Zephyr COPY kept the bug until the
# phase's standing re-sweep found it. Gated so there is no third occurrence.
interface-glob-configure-depends:
    @python3 scripts/check-interface-glob-configure-depends.py

# phase-403 W8 (issue 0940) -- the READER for the exported message-bound
# inventory must derive the four size knobs a bound inventory can answer, and
# must REFUSE loudly rather than derive over a closure holding an unbounded
# type. The second half is the one worth gating: deriving over the bounded
# subset yields a plausible number a real sample can exceed, and the drop is
# SILENT on the C++ arena dispatch path. `cmake -P` against hand-written
# fragments -- no build, no codegen, no toolchain, seconds -- so it belongs on the
# fast line beside its cmake sibling above.
message-bound-knobs:
    #!/usr/bin/env bash
    set -euo pipefail
    _missing=""
    for _t in cmake ninja; do
        command -v "$_t" >/dev/null 2>&1 || _missing="$_missing $_t"
    done
    if [ -n "$_missing" ]; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip message-bound-knobs "missing host tool(s):$_missing"
        exit 0
    fi
    ./tests/cmake-message-bounds-tests.sh

# Issue 0978 — the generated config header a leaf compiles against must be the
# CURRENT one, not merely a PRESENT one.
#
# `mirror-generated-header.sh` chooses between the leaf's own corrosion copy and
# the leaf-independent copy in the shared cargo target dir. Issue 0805 made it
# prefer the leaf's and fall back only when that was ABSENT, on the grounds that
# the two are "the same bytes" — true only when one build-script run wrote both.
# Once leaves share a cargo target dir the script runs for ONE of them, so every
# other leaf keeps its copy from whenever it last ran: present, and arbitrarily
# old. Measured 19 of 20 native leaves six days stale, caught by the 0369 size
# anchor as `undefined reference to nros_config_variant_sz_<hash>` — a link
# error that names a hash and no file.
#
# The self-test is in the script so the rule and its cases live together; it
# stages temp dirs only (no cargo, no cmake, milliseconds), so it belongs on the
# fast line.
[private]
mirror-header-precedence:
    @bash scripts/build/mirror-generated-header.sh --self-test

# RFC-0067 D1 / phase-333 W2 — a generated message crate must be referenced as a
# PATH dep, never by registry name. Replaces the interim `check-msg-dep-redirect`,
# which could only assert that SOME `[patch.crates-io]` redirect existed up the
# config chain — a mitigation cargo ignores when it loads config from a different
# cwd (`--manifest-path` from the repo root), the hole issue 0378 called
# unclosable. A path dep has no registry in its resolution at all, from any cwd.
# Also fails on leftover retired message patch entries. Buildless.
[private]
msg-dep-is-path:
    @bash scripts/check-msg-dep-is-path.sh

# Issue 0320 — committed SystemModels must be portable: no absolute host paths in
# `meta.inputs[].path`. Buildless; regenerate with `nros sync`.
[private]
no-absolute-model-paths:
    @bash scripts/check-no-absolute-model-paths.sh

# issue 0834 — a per-build sizes header must never lose its `.stamp` twin.
#
# The pair is written by ONE function, header first, so a stamp with no header
# is a state the writers cannot produce — and it does not repair itself: a build
# whose probe yields 0 declines to write the header and reports success. The
# consumer-side symptom is a stub `#error` (C++) or an undeclared macro (C),
# from a file nobody was looking at. This names the directory instead.
[private]
orphan-generated-stamp:
    @python3 scripts/check-orphan-generated-stamp.py

# Issue 0739 — every build-time sizing knob is enumerable, and the pools that
# declared their arithmetic carry a byte figure. Issue 0271 audited a 256 KB
# image that already tuned NINE of these and still inherited ~145 KB of defaults
# it did not know existed; four separate features had each added a static pool
# with a knob, silently. Generated, not transcribed — a hand-written list is the
# thing that goes stale the first time a feature lands.
[private]
pool-inventory:
    # WRITES the inventory rather than diffing a committed copy: the file is
    # generated and no longer tracked, so `--check` would report "stale" on
    # every fresh clone (a missing file compares unequal to the rendered text).
    # The assertion that mattered is `self_test()` + a scan that evaluates every
    # formula, and both still run here; the artifact the book needs falls out.
    @python3 scripts/gen-pool-inventory.py

# phase-432 W1.2 (RFC-0091) — the two packs that describe ONE memory agree.
#
# W1.1 deleted `TargetProfile`: codegen models no target. Removing a model of
# the target without adding a MEASUREMENT of it is worse than either, so this
# is the measurement. It cross-compiles size/alignment/field-offset probes from
# the C pack's `typedef struct` and the C++ pack's `#[repr(C)]` Rust glue for
# one 32-bit non-host target and compares the SYMBOL SIZES — no linking, no
# execution, and neither side ever reads the other's spelling.
#
# The idiomatic Rust pack is deliberately NOT in this pair: it carries no
# `repr(C)` and shares CDR (a wire format) rather than memory. Neither is the
# `rmw` pack, whose memory partner is upstream `rosidl_generator_c`. The header
# of the script has both arguments in full.
#
# Skips on a missing cross toolchain or an absent/stale in-tree CLI (the packs
# are `include_str!`-bundled, so a stale binary would answer for other sources).
# `--self-test` is its negative control and needs neither.
[private]
repr-memory-agreement:
    #!/usr/bin/env bash
    set -euo pipefail
    python3 scripts/check-repr-memory-agreement.py --self-test
    _missing=""
    for _t in arm-none-eabi-gcc arm-none-eabi-nm rustc; do
        command -v "$_t" >/dev/null 2>&1 || _missing="$_missing $_t"
    done
    if [ -z "$_missing" ] && [ ! -d "$(rustc --print target-libdir --target armv7a-none-eabi)" ]; then
        _missing=" rust-std for armv7a-none-eabi (rustup target add armv7a-none-eabi)"
    fi
    if [ -n "$_missing" ]; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip repr-memory-agreement "missing:$_missing"
        exit 0
    fi
    nros="packages/cli/target/release/nros"
    # shellcheck source=scripts/build/cli-usable.sh
    source scripts/build/cli-usable.sh
    if ! nros_cli_usable "$nros"; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip repr-memory-agreement "$nros_cli_unusable_reason"
        exit 0
    fi
    python3 scripts/check-repr-memory-agreement.py --nros "$nros"

# Issue 0268 — the per-build sizes headers (`nros_{,cpp_}config_generated.h`)
# mirrored onto the consumer include path must equal the copy build.rs wrote.
# A stale mirror sizes the C `_opaque` buffers from museum data while Rust
# placement-constructs the current object into them — silent corruption
# (0268: freertos C, 336 bytes; 0245: zephyr C++, 32 bytes). Scans whatever
# build trees exist locally; prints the tree count so a vacuous pass is visible.
[private]
sizes-header-mirrors:
    @bash scripts/check-sizes-header-mirrors.sh
