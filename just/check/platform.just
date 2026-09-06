# nano-ros gates: platform
#
# Per-platform and per-board rules -- Zephyr, NuttX, FreeRTOS,
# ThreadX, ESP32 -- plus the tier and Kconfig plumbing.
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

# phase-340 W4 — the artifact-identity budget: how many times one crate is
# COMPILED, and how many dirs one compilation is written into, for a single
# workspace at a single feature set. `examples/workspaces/mixed` is the fixture
# (Rust + C++ node packages over a shared `nros-core`), and today it answers 8
# identities for `nros-core` — three ×2 axes, zero sharing. The numbers and the
# reasoning live in the script; when W2/W3/W5 land, lower them there.
#
# `check-fast`, for two reasons. It is buildless in the strict sense — it reads
# FILENAMES under a build tree that already exists, never invoking cargo, rustc
# or workspace resolution — and it must not be able to fail a BUILD: a
# long-lived incremental tree accumulates rlibs from earlier builds, so an
# over-count from history alone is possible, and a gate that reds a build on
# that gets switched off. Failing a static check whose remedy is "wipe the tree
# and rebuild" is survivable. Consequence, stated rather than hidden: on the
# pristine per-push CI checkout there is no tree and this gate SKIPS (loudly,
# naming the build command); its live coverage is the developer who just built
# fixtures and then ran the tier their change earned.
# issue 0616 — at most ONE Rust archive per image defines the allocator.
#
# The companion to `check-feature-contract` clause (e), which counts
# `#[global_allocator]` DEFINITIONS IN SOURCE — there is exactly one, and there
# always will be. `packages/` has four `crate-type = ["staticlib"]` roots, each
# of which BAKES that one definition into its own `.a`; linking two gives an
# image two allocators while the source still has one. Source counting cannot
# see that, so this asks the artifacts.
#
# Needs built output, so it belongs with the lanes that produce it rather than
# in `check-fast`.
[private]
archive-lang-items:
    @bash scripts/check-archive-lang-items.sh

# issue 0440 — a leaf that deploys to a board must carry that board's STATIC
# link args. `nros-board.toml`'s `cargo_config` is the SSoT (RFC-0032 third
# leg), the leaf `.cargo/config.toml` is TRACKED and `nros sync` leaves it
# alone, so the two drift by hand. phase-338 W2's `-entry` collapse kept the
# node package's config and dropped the whole `-l<kernel lib>` group — valid
# TOML, happy cargo, and every NuttX Rust entry failing at LINK time.
[private]
board-cargo-config-applied:
    #!/usr/bin/env bash
    # phase-341 W4 — REPLACED. The old gate (a shell script of this name,
    # deleted 2026-08-21 once nothing invoked it) asked whether a leaf still
    # carried a REPRESENTATIVE `-l` arg copied from its board, because the leaf
    # mirrored the descriptor BY HAND. It caught a lost GROUP (issue 0440) but
    # not a lost argument, and it covered 8 leaves of the 59 that carried a
    # board block.
    #
    # The block is now a generated projection, so the question changes from "did
    # a human copy enough of it" to "is the committed file what the descriptor
    # renders to" — an EXACT comparison, which makes drift uncommittable rather
    # than detectable. `nros ws check-board-projections` answers it by sharing
    # the renderer with `nros sync`; a second implementation in shell is the
    # drift this phase exists to remove.
    set -euo pipefail
    nros="packages/cli/target/release/nros"
    # ABSENT and STALE are one verdict: this lane is contractually source-free
    # and CLI-free, so it cannot demand a CLI, and asserting against a binary
    # built from other sources is worse than not asserting. One restaled stamp
    # used to surface as three red gates naming one remedy.
    # shellcheck source=scripts/build/cli-usable.sh
    source scripts/build/cli-usable.sh
    if ! nros_cli_usable "$nros"; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip board-projections "$nros_cli_unusable_reason"
        exit 0
    fi
    fail=0
    while IFS= read -r cfg; do
        leaf="$(dirname "$(dirname "$cfg")")"
        NROS_REPO_DIR="$PWD" "$nros" ws check-board-projections "$leaf" >/dev/null 2>&1 || {
            NROS_REPO_DIR="$PWD" "$nros" ws check-board-projections "$leaf" 2>&1 | sed 's/^/  /' >&2
            fail=1
        }
    done < <(git ls-files '*/.cargo/nros-board.toml')
    [ "$fail" = 0 ] || exit 1
    echo "board projections OK ($(git ls-files '*/.cargo/nros-board.toml' | wc -l) leaf/leaves match their descriptor)"

[private]
board-facts-delivery:
    @python3 scripts/check-board-facts-delivery.py

# Phase 215.F.2 — board-crate manifest drift gate. For every
# `packages/boards/nros-board-*` carrying BOTH a `board.cmake` sidecar
# and a `[package.metadata.nros.board]` table, run `nros board info
# <name> --check-drift` and fail on any field-by-field drift between the
# cmake face and the Cargo face. Skips when the in-tree `nros` CLI isn't

# RFC-0064 R5 D4 / phase-375 W7 — a board states its facts ONCE.
#
# The two retired faces: a `board.cmake` sidecar (14 `NROS_BOARD_*` variables)
# and a `[package.metadata.nros.board]` mirror in Cargo.toml. cmake now reads a
# mechanical projection of `nros-board.toml` instead (`nros board cmake-vars`,
# written into the build dir per configure), so there is nothing to write twice.
#
# A gate rather than a note because the pressure is real: cmake cannot read
# TOML, and the shortest path from that to a working build is to write the
# values again in a language cmake speaks. That is how the first face appeared,
# and its drift audit then examined zero boards for two independent reasons
# while reporting OK.
[private]
board-descriptor-single-source:
    @python3 scripts/check-board-descriptor-single-source.py

# built (the packages/cli phase215_f integration test still covers it).
[private]
board-manifest-drift:
    @bash scripts/check-board-manifest-drift.sh

[private]
board-tiers:
    @python3 scripts/check-board-tiers.py
    @python3 scripts/gen-board-support-table.py --check

# A Kconfig line that a later CONF_FILE fragment overrides is a line that does
# nothing, and nothing said so (issue 0876). A RATCHET, not a prohibition —
# shared board fragments are legitimately an override layer. Refresh with
# `python3 scripts/check-kconfig-overridden-values.py --update`, and say why.
# Every `[prereq.*]` key declares who may NAME it (RFC-0062 amendment 3).
# The Rust field defaults to `unclassified` so it could land incrementally;
# without this gate a new key inherits that default and `nros setup --workspace`
# reports a toolchain as a content dependency — the conflation `role` removes.
# phase-422 W7 — every `deploy=`/`board=` a package.xml exports must resolve.
# `nros setup --workspace` turns those values into a provisioning command, so a
# value meaning nothing becomes a remedy that fails. Five namespaces exist for
# the concept; this does not unify them, it stops a value in none of them.
[group("checks")]
board-vocabulary:
    @python3 scripts/check-board-vocabulary.py

# issue 0942 — the C++ twin of the rule above. `std::fprintf` in a TU that is
# cross-compiled for a board's own toolchain depends on a guarantee freestanding
# libstdc++ does not make; it built on every hosted target and failed only on
# threadx-riscv64.
[group("check")]
cpp-no-std-stdio:
    @python3 scripts/check-cpp-no-std-stdio.py

[private]
deploy-board-resolves:
    @python3 scripts/check-deploy-board-resolves.py

# issue 0835 — an example leaf must not overrule the platform its own build
# passes. Six threadx rust leaves carried a plain `set(NANO_ROS_PLATFORM
# threadx)`, a NORMAL variable that shadows the cache entry
# `-DNANO_ROS_PLATFORM=threadx_riscv64` wrote — so each leaf and its own
# CMakeCache disagreed, written in the same second. Visible cost: four shared
# corrosion cargo groups where the configuration space has two, 2.8 GB of
# duplicate output. Quieter cost: two `stdc++` link decisions had come to depend
# on the wrong label, so making the leaves honest would have stopped them
# matching, silently. Buildless, reads the manifest and the leaves.
[private]
example-platform-not-shadowed:
    @python3 scripts/check-example-platform-not-shadowed.py

# Issue 0961 — `Executor::open_in` builds the executor as a VALUE in its own
# frame and `nros_cpp_init` moves it again, so an image whose main thread is
# smaller overflows in a function PROLOGUE, before any statement runs and so
# before any check written inside those functions could execute. Four bring-up
# sessions went into finding that once. The answer is a compile-time guard in
# the CALLER's translation unit; it exists in all four producers and nothing
# kept it there. Source-level by necessity: the generated headers are build
# artifacts and an affordability tier may not resolve one, but "does every
# producer still emit the define AND still compare against it" is exactly the
# regression that would silently disarm it.
[private]
executor-stack-floor:
    @python3 scripts/check-executor-stack-floor.py

# `native` is a ROLE (host build); `posix` and `linux` are two REACHES and a
# board has one. See CLAUDE.md "Naming".
[private]
host-platform-vocabulary:
    @python3 scripts/check-host-platform-vocabulary.py

# phase-366 M6 / RFC-0077 — every image declares exactly one ending, and what it
# declares matches what it supplies. `check-archive-lang-items` counts per LINK
# LINE, which catches duplication and is blind to ABSENCE (an image with no
# provider has no archive to count — issue 0617). Buildless, source-level.
image-panic-policy:
    @python3 scripts/check-image-panic-policy.py

# Issue 0719 — every path that BUILDS an image applies the image's policies.
# `nano_ros_add_executable` delegates to `nano_ros_entry`, so ~160 call sites are
# covered by construction; the handful that cannot call the entry (a board seam,
# the ESP-IDF component shim) must call `nros_apply_panic_policy` instead. Two of
# them did not, and each surfaced as a `#[panic_handler]` error four crates from
# its cause (#0688, #0700).
#
# STRUCTURAL, and it self-tests the one thing that makes it so: the sweep that
# preceded it keyed on the text `nano_ros_entry(` and excluded a file because a
# COMMENT there mentioned the name.
image-paths-apply-policy:
    @bash scripts/check-image-paths-apply-policy.sh

# issue 0460 — a knob Kconfig forwards must be read by the Rust lane too. The
# cmake `set(ENV{...})` exports reach the C lane's re-baked command and NOT
# zephyr-lang-rust's `rust_cargo_application`, so every Zephyr Rust image
# compiled its crates' defaults whatever Kconfig said — silently, for every
# knob at once.
[private]
kconfig-knob-forwarding:
    @bash scripts/check-kconfig-knob-forwarding.sh

[group("checks")]
kconfig-overridden-values:
    @python3 scripts/check-kconfig-overridden-values.py

# phase-412 W4 — the value that reaches the COMPILE is the value the resolver
# produced. The knob gates beside this one check whether the derived number is
# RIGHT; this one checks whether it ARRIVED, which is a different question and
# the one that kept going wrong. W1 shipped six delivery failures in one wave
# with every other gate green: a second consumer reading raw CONFIG_*, a name
# that resolved to empty, a C consumer with no default under rung 4, a loader
# whitelist that dropped symbols at a function boundary, and a knob resolved
# twice where the second call won with the DERIVE sentinel. Two of those were
# live in pushed work, and one built the zenoh session with 8 subscriber slots
# for 10 subscriptions — a runtime failure, invisible at link.
#
# Self-test only here: the end-to-end assertion needs a CONFIGURED build dir,
# which the fast tier does not have. Point it at one by hand:
#     python3 scripts/check-knob-delivery.py <build-dir>
[private]
knob-delivery:
    @python3 scripts/check-knob-delivery.py --self-test

# Issue 1002 — a configure sequence must reach a FIXPOINT, and the pass count
# is not the instrument. 0991's remedy says configure TWICE; the receive
# payload class needed THREE, and four island builds shipped the stale value
# because the stale one is LARGER and therefore silent. This configures until
# nothing moves rather than counting, so a knob that gains a longer chain fails
# here instead of over-sizing for a month.
#
# Self-test only in the fast tier: the real assertion needs a configured build
# dir. Point it at one by hand:
#     python3 scripts/check-knob-fixpoint.py <build-dir> --configure "<cmd>"
[private]
knob-fixpoint:
    @python3 scripts/check-knob-fixpoint.py --self-test

# Issue 1015 — a knob that SIZES A FIXED C ARRAY states its smallest legal
# value, and the floor is applied by the CONSUMER that names the knob.
#
# `MAX_QUERYABLES` derived 0 on an image declaring no service servers, and the
# board then transmitted 0 bytes in 15 s: no panic, no log, core in WFI. Every
# knob gate stayed green — `check-knob-delivery` confirms the value ARRIVED
# (it did) and `check-knob-fixpoint` converges (0 is stable). None asked
# whether the number was USABLE.
#
# "Pools are positive" would be the wrong rule: issue 1033 lowered the XRCE
# minima to 0 on purpose, worth 33,296 bytes of heap a subscriber slot. So the
# gate requires a DECISION per knob — a `#if X < 1 / #error` beside the array,
# or a ZERO_LEGAL entry naming what measured it — and holds the two producers
# of the guarded knobs to flooring at the pool rather than in the shared
# derivation, which is where the first fix put it and where it silently
# defeated 1033 the day before that landed.
[private]
c-array-pool-floors:
    @python3 scripts/check-c-array-pool-floors.py

# A knob is resolved EXACTLY ONCE. phase-412 hit this three times: a knob
# gained a derivable resolution and kept its old plain one, the second call
# won, and what it passed was the raw Kconfig value — which for a derivable
# knob is the `-1` DERIVE SENTINEL, resolved as though someone had stated it.
# Invisible to every other gate, because the derived value was right and the
# resolver did run.
#
# Branch-aware: two arms of one conditional are not duplicates. Its first real
# run said they were, and a checker that flags a correct idiom trains people to
# ignore it.
[private]
knob-resolved-once:
    @python3 scripts/check-knob-resolved-once.py

# phase-400 W8 — a knob migrated into the RFC-0049 ladder has exactly one
# reader. Retirement is a wave, not a side effect: a mechanism that still
# resolves is one people still use, and a fallback that silently wins is how
# issues 0135 and 0316 happened.
[private]
knob-single-reader:
    @python3 scripts/check/check-knob-single-reader.py

[group("main")]
literal-domain-id:
    @python3 scripts/check-literal-domain-id.py

# Phase 313 W6 — forbid the retired `nros_board_common::board_init` API from
# creeping back (boards use `nros_platform::board::*` or the C ABI). Buildless.
[private]
no-board-init:
    @bash scripts/check-no-board-init.sh

# Phase 230.0.2 (RFC-0034) — no crate may call the host kernel allocator
# directly except a platform port; everything routes through
# nros_platform_alloc. Advisory until Wave 1 migrates the inventory
# (set NROS_ALLOC_GATE_HARD=1 to enforce).
[private]
no-direct-kernel-alloc:
    @bash scripts/check-no-direct-kernel-alloc.sh

# Issue 0589 — no `std::`-qualified stdio in a `#![no_std]` crate's `src/`.
#
# On Zephyr `native_sim` a Rust `std::eprintln!` does not print, it SIGSEGVs the
# image: `zvfs_write(1, …)` re-enters itself through `stdinout_write_vmeth` and
# `k_mutex` is recursive, so the stack runs out. The print that found this was a
# diagnostic — so an error path got LESS informative the more it said. The Kconfig
# is identical in cells that pass, which makes it latent in every native_sim image
# rather than a bug in any one of them; a gate is the only thing that keeps the
# next `eprintln!` out.
#
# Buildless. `--self-test` drives 16 synthetic trees, including the two false
# readings this gate shipped with and had to fix: a `cfg_attr(not(feature =
# "std"), no_std)` crate read as hosted, and two calls on one line counted once.
[group("ci")]
no-std-stdio:
    @python3 scripts/check-no-std-stdio.py

# issue 0488 residue 4 follow-up — the NuttX integration Makefile is on a path no
# lane executes (`CONFIG_NROS` is set by no shipped defconfig), so it rots.
[private]
nuttx-integration-makefile:
    @bash scripts/check-nuttx-integration-makefile.sh

[private]
nuttx-libc-struct-sizes:
    @python3 scripts/check-nuttx-libc-struct-sizes.py

# phase-339 W3 / issue 0433 — no NuttX consumer may link the SHARED live kernel
# tree. Both arches build in one in-tree checkout, so `staging/` belongs to
# whichever built last; a consumer that links it silently stales the OTHER
# architecture's fixtures. Consumers resolve the per-arch export snapshot
# through `nros_board_common::nuttx_export` instead. Source grep, buildless.
[private]
nuttx-links-snapshot:
    @bash scripts/check-nuttx-links-snapshot.sh

# Issue 0525 — NuttX is built IN PLACE and one checkout serves both arches, so
# `$NUTTX_DIR/include/nuttx/config.h` belongs to whichever arch was configured
# LAST. A compile input taken from there silently gets the other arch (issue
# 0511: the ARM image linked with the RISC-V memory map). Buildless.
[private]
nuttx-shared-tree-headers:
    @python3 scripts/check-nuttx-shared-tree-headers.py

# `nros-platform-posix` is named for a standard — keep it holding to one, so
# the board layer stays the only place that is Linux-only (phase-401 W4).
[private]
posix-platform-purity:
    @python3 scripts/check-posix-platform-purity.py

# phase-379 W5 — `nros::prelude` is the rclrs-shaped API, `nros::embedded` is
# the RTOS machinery. Membership is a LEDGER QUERY (a name may sit in the
# prelude only if the parity ledger does not call it an `extension`), plus an
# argued allow-list for the extensions a node cannot start without.
[private]
prelude-tiers:
    @python3 scripts/check-prelude-tiers.py

# Phase 176.3 — verify the orchestration generator's PlatformProfile
# board-crate references match the actual board crates (existence +
# `run` entry). Skips when the colcon-nano-ros submodule is absent.
[private]
profile-board-mirror:
    @bash scripts/check-profile-board-mirror.sh

# A PX4 module compiles against a per-build generated header and links a
# separately-built `libnros_cpp.a`. The two are independent files that each
# outlive the build that wrote them, so nothing made them agree — issue 1050's
# link wall (`undefined reference to nros_cpp_config_variant_...`) is that
# disagreement, found ~1100 targets into a ten-minute build. Issue 1046 is why
# the guard is on CONTENT: the predicate it replaced asserted a DIRECTORY, which
# survives the build it stood for, so it passed on exactly the tree it existed to
# reject. This gate runs the real cmake predicate against fixtures in BOTH
# directions; a guard nobody has seen fail is the bug it is guarding against.
[private]
px4-archive-header-pairing:
    @bash scripts/check-px4-archive-header-pairing.sh

# Issue 0555 — `nros_platform_clock_{ms,us}` are `static inline` wrappers in
# `nros/platform.h` now (RFC-0073), so the header IS the definition. The rename
# broke four consumers in a row, each visible only after the previous cleared
# (#541, upstream 5dc2fa869, #547, #548), and #548 asked for this gate by name.
# Two arms: a use with the header out of scope, and a hand-written `extern`
# declaration — which compiles and then fails at link. Buildless.
retired-platform-clock-symbols:
    @python3 scripts/check-retired-platform-clock-symbols.py

# Issue 0589 — `std::println!`/`eprintln!` from a crate Zephyr links does not
# print on native_sim, it recurses in `zvfs_write` until the stack is gone. Go
# through `cpp_diag!`, which uses std stdio only where that is safe.
[private]
rust-stdio-on-zephyr:
    @python3 scripts/check-rust-stdio-on-zephyr.py

# RFC-0079 — a tier priority pin must respect its port's declared address plan.
# Scheduling priority is ONE space shared by application tiers and system tasks;
# pins were hand-chosen into it with no record of what was already there, and
# 34 of 38 in the tree were colliding, preempting, or on a port that declares
# nothing. A pin landing ON a reserved band FAILS; one that merely outranks a
# band warns until `above = "<band>"` exists to make it a stated choice. Also
# cross-references each plan against the code it describes, so the plan cannot
# drift into a second spelling of the same fact. Buildless.
[private]
tier-priority-plan:
    @python3 scripts/check-tier-priority-plan.py

# RFC-0079 §4.1 — finish the check `check-tier-priority-plan` DEFERS.
#
# Zephyr's reserved band is computed per IMAGE from Kconfig, so the static gate
# cannot judge its pins and says so instead of pretending. This resolves the
# band from every built `.config` and checks the pins against each. With no
# argument it examines EVERY image in the tree: a derived band is a property of
# an image, so letting the caller pick which one to prove is how a green comes
# to mean less than it looks.
#
# SKIPS loudly on a host with no built Zephyr image, rather than passing —
# issue 0599's rule. Buildless (reads `.config`, builds nothing).
tier-priority-plan-image config="":
    @python3 scripts/check-tier-priority-plan-image.py {{config}}

# Issue 0636 option 3 — every tier spin loop reaches a scheduling point.
[group("check")]
tier-spin-gap:
    @python3 scripts/check-tier-spin-gap.py

# Issue 0654 — the router takes NO command-line configuration, so a `zenohd
# --listen …` line is not merely a stale name: the flags are UNREAD rather than
# rejected, and the reader gets a default-configured router with no diagnostic.
# Gated because the class regenerates by copy-paste from a neighbouring example
# header — which is how it reached ~95 files.
# Issue 1039 — a port may declare its own platform to zenoh-pico, never
# somebody else's. `nros-platform-nuttx` defined ZENOH_LINUX to reach the
# `getrandom()` arm; the arm was right and the name was not, and the claim
# then became load-bearing in three unrelated guards. Scans BOTH producers
# (the platform manifests and `runner.rs`'s `build.define`), and runs its own
# negative control first.
zenoh-platform-macros:
    @python3 scripts/check-zenoh-platform-macros.py

# issue 0651 — every Zephyr symbol `zephyr/Kconfig` names must EXIST on BOTH
# supported lines (3.7 LTS + the 4.4 rolling line). Kconfig answers an undefined
# symbol with a WARNING, so a `select` that is right on 3.7 and misspelled on
# 4.4 silently drops what it would have enabled — and 4.4 builds only in the
# nightly, so that surfaces a day later against whatever else moved.
#
# Needs SOURCE, not a build, and not a west workspace. Off the fast line because
# it has a real precondition (a tree must be present); it FAILS rather than
# passes when it can check nothing (issue 0702).
#
#   3.7:  just zephyr setup
#   4.4:  just zephyr kconfig-trees      (two shallow clones, no west workspace)
#
# Issue 0651 second half: an unchecked supported line is now a FAILURE, not an
# OK with a footnote. It used to fail only when NO line was present, so the
# ordinary dev host — 3.7 present, 4.4 nowhere — went green having measured
# nothing about the line the gate exists for.
zephyr-kconfig-symbols:
    @python3 scripts/check-zephyr-kconfig-symbols.py

# issue 0529 — Zephyr's zenoh tx knobs have TWO sources: `zephyr/Kconfig`
# defaults (forwarded to the C lane by nros_rmw_zenoh.cmake) and
# `config/zephyr/nros-platform.toml`'s [knobs.zenoh.tx] (the RFC-0049 ladder,
# Rust lane). They agree today only by coincidence; this compares them so a
# divergence is loud. Buildless.
[private]
zephyr-knob-agreement:
    @python3 scripts/check-zephyr-knob-agreement.py
