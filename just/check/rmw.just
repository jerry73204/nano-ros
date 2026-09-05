# nano-ros gates: rmw
#
# The RMW backends: zenoh, cyclonedds, XRCE, uORB, and the entity
# budgets they share.
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

# issue 0460 — a capability's service count must match the slots the executor
# sizing reserves for it. The counts live in `executor_sizing` (only a crate the
# proc-macro depends on can supply a const at expansion time) while the services
# live in `nros-node`, which does not depend on it — so nothing in the type
# system ties them together.
[private]
capability-slot-counts:
    @bash scripts/check-capability-slot-counts.sh

# Issue 0984 — `libnros_rmw_cyclonedds.a` is built from TWO authored source
# lists (the cmake target, and `let cpp_files` in the sys crate's build.rs), and
# #0970 added `nros_sertype.cpp` to only one. The cmake path kept linking, so
# nothing looked wrong; every RUST fixture that reached the link stage died on
# an undefined `create_nros_sertype` referenced from inside the archive.
#
# `build.rs` already carried a comment warning about this exact drift. A comment
# is not a check.
[private]
cyclone-backend-sources:
    @python3 scripts/check-cyclone-backend-sources.py

# issue 0974 — `CONFIG_NROS_CYCLONE_DOMAIN_ID` defaults to `NROS_DOMAIN_ID` so
# the two knobs cannot split; pinning a literal breaks that link silently, which
# is phase-180's split-brain (issue 0161). Both `nros new`'s template and the
# Zephyr getting-started page were emitting the pin, so every generated
# cyclonedds project started split-brained and the book taught the pattern.
[group("check")]
cyclone-domain-not-pinned:
    @python3 scripts/check-cyclone-domain-not-pinned.py --self-test
    @python3 scripts/check-cyclone-domain-not-pinned.py

# RESTORED (2026-09-04). This recipe and its fast-lane list entry were deleted
# by `e569d9a55`, a commit about which Zephyr SDK the CI image bakes, whose
# message does not mention them -- an accidental loss in a rebase, not a
# decision. For a day the test script sat tracked with NO caller anywhere in
# `just/`, `justfile` or `.github/`: a gate that runs nowhere, guarding the
# delivery half of phase-403 step 2, which had merged hours earlier.
#
# `check-test-scripts-have-callers` now catches that class -- it was written
# from this incident and fires on exactly this state.
# phase-403 step 2 -- does the declared `@depth=` REACH A COMPILER?
#
# `just check cpp` proves the CHECK: it compiles a TU whose declared depth and
# passed QoS disagree and asserts the build is rejected, with the topic and both
# numbers in the diagnostic. It hands the table to the compiler with a `-I` of
# its own, so it says nothing about DELIVERY.
#
# This is the delivery half, and it is the half that goes wrong silently: a
# component whose generated table never lands on its include path compiles with
# every declared-depth assertion disabled, and looks exactly like a component
# whose depths all agree. Six mechanisms in this campaign have been correct and
# unreachable for precisely that reason.
#
# Drives `_nros_emit_declared_qos_header()` in a real configure with the real
# CLI. Needs cmake, a C compiler and a built `nros` -- and SKIPS rather than
# passing when any is missing, because a gate that examined nothing must not
# read as a pass.
declared-qos-header:
    #!/usr/bin/env bash
    set -euo pipefail
    _missing=""
    command -v cmake >/dev/null 2>&1 || _missing="$_missing cmake"
    command -v cc >/dev/null 2>&1 || command -v gcc >/dev/null 2>&1 \
        || _missing="$_missing cc"
    _cli="${NROS_CLI:-packages/cli/target/release/nros}"
    [ -x "$_cli" ] || _missing="$_missing nros(just setup-cli)"
    if [ -n "$_missing" ]; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip declared-qos-header "missing:$_missing"
        exit 0
    fi
    ./tests/cmake-declared-qos-header-tests.sh

# phase-428 W10 — the named QoS presets are upstream's, field by field.
#
# `QOS_PROFILE_PARAMETER_EVENTS` was KEEP_ALL/0 where upstream is
# KEEP_LAST/1000, under a comment claiming parity and above a test asserting
# the wrong value, in one of FOUR hand-transcribed copies of the same seven
# profiles. This binds the SSoT table in `nros-rmw/src/traits.rs` to
# `docs/reference/rmw-qos-profiles.txt`, which is extracted from a real ROS
# install; where one is present it re-extracts and checks the record too.
#
# Buildless and 0.08 s, so it belongs on the fast line — no ROS, no cargo, no
# fixtures, and it does NOT degrade to a pass when ROS is absent.
[private]
qos-profile-ssot:
    @python3 scripts/check-qos-profile-ssot.py

# phase-403 W9 (issue 0965) — the per-kind CALLBACK SLOT cost has ONE
# definition, tied to the registration sites that actually claim a slot.
#
# `nros ws entity-inventory` derives NROS_EXECUTOR_MAX_CBS by summing
# `EntityKind::callback_slots()`, and that function is a MIRROR: the CLI is a
# host binary and nros-node is no_std and built for the target, so it restates
# what `Executor::next_entry_slot()` decides. Same rule as the queryable counts
# above — a mirror is acceptable only while something holds it to the
# definition. The specific claim it defends is that a PUBLISHER claims no slot,
# which is why the island's 33 declared entities want 19 and why the bring-up's
# hand-count of "33 handles" over-sized by 14.
[private]
entity-slot-costs:
    @python3 scripts/check-entity-slot-costs.py

# Issue 1050 defect (3) — `NROS_ENTRY_RMW` may only name a backend that
# registers under that name.
#
# The RMW selector gained a BAKED rung, so `nano_ros_entry()` /
# `nros_px4_add_module()` now put a NAME on the target and the core resolves it
# through `nros_rmw_cffi::resolve_backend`. That is a LOOKUP, and a miss is
# `Unknown` — a hard error, not a fallback to the registry default — so a rename
# on either side turns every image built for that RMW into one that dies at
# `nros::init()`.
#
# The two vocabularies (cmake's `NROS_RMW_KNOWN`, and the string each backend
# passes to `nros_rmw_cffi_register_named`) agreed by convention and nothing
# held them there. Buildless: both are read out of the sources.
[private]
entry-rmw-vocabulary:
    @python3 scripts/check-entry-rmw-vocabulary.py

# Issue 0827 — the infrastructure-queryable counts have ONE definition each,
# tied to the number of servers actually created. A service server IS a zenoh
# queryable, so these are a term in every service-buffer pool the RMW sizes;
# they had SEVEN spellings and no definition, two of them already wrong.
[private]
infra-queryable-counts:
    @python3 scripts/check-infra-queryable-counts.py

# Issue 0737 — an example's message callback may not drop a sample silently.
# From outside the process a silent `return` and a message that never arrived
# are the SAME observation; that ambiguity cost two hosts an investigation each.
[private]
no-silent-sample-drop:
    @python3 scripts/check-no-silent-sample-drop.py

# issue 0777 — seven ABI deviations were justified by "pools are baked", a claim
# true of one backend in five. The conclusion held; the reason did not. This is
# the re-run: every allocation site in a backend, split by whether it is on the
# per-message path or at entity creation. `--check` fails on a steady-state
# allocation with no declared reason, so a new one is argued for rather than
# merged. Source-only, no ROS install, fast line.
rmw-alloc-sites:
    @python3 scripts/rmw-alloc-sites.py --self-test
    @python3 scripts/rmw-alloc-sites.py --check

# Issue 0330 (class of 0155/0163) — a Zephyr Rust example whose `rmw-*` feature
# forwards to a real backend dep must invoke `nros::force_link_backend!`, or
# rustc's staticlib DCE drops the backend's `#[no_mangle]` register export and
# the image boots with NO backend registered. Mutation-verified silent: removing
# the anchor still builds AND links. The anchor used to be emitted by
# `zephyr_component_main!` (so it could not go missing); 0330 moved it to the
# app crate, which is what makes this gate necessary. Buildless.
[private]
rmw-force-link-anchor:
    @bash scripts/check-rmw-force-link-anchor.sh

# Issues 0332 + 0349 — the RMW vtable's required-slot list and its
# `.expect("rmw vtable: …")` dispatch sites must be the SAME set. A slot
# expect-ed but not required panics mid-spin on no_std (0332); a slot required
# but not expect-ed refuses working backends (0349 — how xrce became
# unregistrable). Checked both directions. Buildless.
[private]
rmw-required-slots:
    @bash scripts/check-rmw-required-slots.sh

# Issue 0934 R1 / phase-405 W2 — `nano_ros_workspace(BACKEND … SYSTEM …)` names
# the RMW on one line and, on the next, a bringup whose `system.toml` names it
# again. `NanoRosWorkspace.cmake` builds the path to that file (`:351`) and
# never parses it, so the two authored copies can contradict each other with no
# diagnostic: the C/C++ lane links BACKEND, the CLI resolves `[system].rmw`.
# This fails on a disagreement and NAMES the one-sided declarations that can
# become one. It does not decide which side is the SSoT — that is W2's
# follow-up. Buildless.
[private]
workspace-rmw-agreement:
    @python3 scripts/check-workspace-rmw-agreement.py

# phase-348 W4 — build order derived from package.xml `<depend>`, not from the
# order a SUBDIRS list happens to be written in. Every workspace CMakeLists
# carries "Node pkgs BEFORE entries so the entry codegen sees their metadata";
# the entry packages already state that as <exec_depend>. Covers the cycle and
# bad-subdir rejections and the cmake seam. Needs `just setup-cli`.
# Issue 1068 / RFC-0087's mirror class — micro-XRCE-DDS is compiled by TWO
# lanes from one vendored tree, and the two source lists drifted: `build.rs`
# honoured `NROS_LINK_IP=0` and dropped the UDP transports while the CMakeLists
# compiled them unconditionally, so a serial-only node built from C/C++ carried
# an IP transport the user had removed — a larger image, not an error.
#
# `packages/rmw/xrce/xrce-sources.txt` is now the one list, and this asserts
# that neither lane names a `.c` of its own, that both answer exactly the
# manifest's condition tokens (both directions — a token only one lane answers
# IS 1068, one conditional over), and that no backend source is compiled by
# nobody. Buildless, ~40 ms.
[private]
xrce-source-manifest:
    @python3 scripts/check-xrce-source-manifest.py

[private]
zenoh-config-template:
    @python3 scripts/check-zenoh-config-template.py

zenohd-flag-invocations:
    @python3 scripts/check-zenohd-flag-invocations.py

# Issue 0653 — the shell and Rust router resolvers must resolve the SAME router.
#
# `just <plat> zenohd` is shell and the test harness is Rust; neither can call
# the other, so `zenohd.sh` carried "the two must agree" as a comment and they
# drifted regardless — both searching only `/opt/ros` while `AMENT_PREFIX_PATH`
# is what a sourced ROS actually announces. On a ROS built from source, or this
# repo's own Arch/Fedora/NixOS distrobox route, that is not `/opt/ros`: you could
# source a working ROS and be told there is no router.
#
# The answer to two implementations is one TABLE
# (`scripts/dev/zenohd-resolution-cases.tsv`), answered here for the shell and by
# `zenohd_resolution_matches_the_shared_table` for the Rust — behaviour on both
# sides, not a diff of two languages.
zenohd-resolution-parity:
    @bash scripts/check-zenohd-resolution-parity.sh

# issue 0804 — `ZenohRouter::start*(...).expect(...)` turns "this host has no ROS
# router" into a test failure. `fixtures::or_skip` is the one reading of
# `RouterUnavailable` (skip), and it keeps every other router error a failure.
# 31 sites went around it, which is why tier 2 reported 7 real failures on a
# host whose ROS lives in a distrobox. Source-only, fast line.
zenohd-router-skips:
    @python3 scripts/check-zenohd-router-skips.py --self-test
    @python3 scripts/check-zenohd-router-skips.py

# Issue 0573 — ZenohRouter must stay the only zenohd spawner.
zenohd-spawn-sites:
    @bash scripts/check-zenohd-spawn-sites.sh

# phase-206 W3 — the zpico run-time config-key table vs zenoh-pico's config.h.
#
# `zp_config_insert(config, Z_CONFIG_<X>_KEY, value)` is zenoh-pico's PRIMARY
# configuration method and its only one — the pico client has no config-file
# format (JSON5/YAML belongs to `zenohd`). So the name -> key map in `zpico.c`
# is the entire configuration surface, and it was a HAND-WRITTEN closed list
# covering 10 of the 23 keys upstream defines, with the other 13 unreachable
# and an unknown name silently dropped. The list is derived from config.h now;
# this gate is what keeps it derived across an upstream bump.
#
# Skips rather than fails when the zenoh-pico submodule is absent: `check-fast`
# must stay green on a pristine detached worktree (issue 0650).
[private]
zpico-config-keys:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ ! -f packages/rmw/zenoh/zpico-sys/zenoh-pico/include/zenoh-pico/config.h ]; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip zpico-config-keys \
            "zenoh-pico submodule not checked out (git submodule update --init packages/rmw/zenoh/zpico-sys/zenoh-pico)"
        exit 0
    fi
    python3 scripts/gen-zpico-config-keys.py --check
