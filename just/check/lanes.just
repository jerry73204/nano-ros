# nano-ros gates: lanes
#
# The COMPILE and LINK lanes: each builds or resolves something, and
# these are what `build-serial:` fans out over. Grouped by being a
# lane rather than by subject, because that is how they run and how
# they fail.
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

# Phase 379 W2 (landed 2026-08-25) — every item where the nano-ros user API does
# not correspond to rclc/rclcpp/rclrs carries a written verdict. 2158 rows across
# 17 topic shards under docs/reference/api-parity-ledger/.
#
# NOT buildless, which is why it is here and not on the fast lane: it re-extracts
# OUR surface every run (clang for C/C++, nightly rustdoc for Rust) so an edit
# that moves us away from ROS 2 fails the gate instead of aging the ledger. The
# ROS 2 side is the recorded surface and needs no ROS install.
#
# A new divergence fails this with the row's key and the bucket it landed in.
# Add a row to the matching topic shard; `--topic <name>` shows the stage.
#
# Verify the user API still corresponds to rclc / rclcpp / rclrs.
[group("check")]
api-parity:
    @python3 scripts/api-parity.py --check --require-disposition

# Borrowed-view runtime E2E (RFC-0033 / #0423) — link the C + C++ proof binaries at
# the build stage, then RUN them (they assert every borrowed view aliases the CDR
# buffer). Bespoke recipe like `check-staticlib-symbols` because the composite
# (cargo + codegen + a raw gcc/g++ link with a weak config-variant anchor) fits no
# `compile_check_fixture` builder.
[private]
borrowed-e2e:
    #!/usr/bin/env bash
    set -e
    bash scripts/build/borrowed-e2e-fixture.sh
    # Skip-capable: routed through the tolerant runner so an unmet
    # precondition reads as a SKIP rather than a red naming a test the
    # change never touched. Gated by `check-skippable-tests-tolerant`.
    just _nextest-tolerant -p nros-tests --no-fail-fast --test borrowed_e2e

# Check C code: formatting + nros-c umbrella header syntax. COMPILES nros-c
# (→ nros-macros → nros-build → nros-cli-core → the ros-launch-manifest submodule;
# issue 0083) to emit the OPAQUE_U64S macro header, so it needs sources/CLI
# submodule → build tier (check-build), NOT the source-free push lane.
[private]
c: c-fmt
    #!/usr/bin/env bash
    set -e
    echo "Checking C code (build + syntax)..."
    echo "  - syntax (nros-c umbrella header)"
    # The per-variant `<nros/nros_config_generated.h>` (defining the
    # OPAQUE_U64S macros referenced by `<nros/nros_generated.h>`) is
    # emitted by `nros-c`'s build.rs into `target/nros-c-generated/`.
    # Build first so the syntax check has those macros; otherwise
    # the source-tree stub fires its `#error`.
    # issue 0400 follow-up — this build GENERATES the headers the checks below
    # compile against, so its failure must not be swallowed. It used to run
    # `--quiet 2>/dev/null || true`, which turned "the generator never ran" into
    # "your C headers are broken": every `*_OPAQUE_U64S` came back undeclared
    # from a `cc` line that says nothing about cargo. Diagnosing that cost four
    # wrong hypotheses; the build's own error names the cause in one line.
    cargo build -p nros-c --no-default-features --features "{{C_API_SHIPPED_FEATURES}}" --quiet
    # Variant dir FIRST so its `nros_config_generated.h` (with the
    # real OPAQUE_U64S macros) wins over the source-tree stub.
    cc -fsyntax-only \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -include packages/api/nros-c/include/nros/nros.h \
        -x c /dev/null
    echo "  - RFC-0033 borrow views: stream alignment + checked count (issues 1148/1149)"
    # The one COMPILE+LINK+RUN TU in this lane. The defects it pins are
    # invisible to a syntax check: `nros_cdr_borrow_le_slice_*` returned 0 on
    # every 8-byte sequence while handing back padding as data, and a wire
    # count of 0x40000000 wrapped `(size_t)cnt * sizeof` to 0 on every 32-bit
    # target. The binary links the `libnros_c.a` the build above just produced
    # (so `nros_cdr_align` is the Rust reader's own alignment, not a C copy of
    # it) and exercises the 32-bit wrap through `nros_cdr_seq_byte_len`'s
    # explicit limit, because this host's `size_t` cannot. `test -f` first —
    # a missing TU must not read as a pass (issue 0196's class).
    test -f packages/api/nros-c/tests/run/cdr_borrow_le_slice.c || { \
        echo "ERROR: cdr_borrow_le_slice.c is MISSING -- this check passes on absence." >&2; exit 1; }
    mkdir -p tmp
    # profile-literal-ok: unprofiled: the archive is from the plain `cargo build`
    # at the top of this recipe, so `target/debug/` IS its derived output dir.
    nros_c_lib=target/debug/libnros_c.a
    cc -std=c11 -Wall -Wextra \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/run/cdr_borrow_le_slice.c \
        "$nros_c_lib" -lpthread -ldl -lm \
        -o tmp/cdr_borrow_le_slice
    ./tmp/cdr_borrow_le_slice
    echo "  - entry seam links from C alone + the strong override wins (phase-432 W3.1)"
    # `nros_board_network_wait()` is the board hook the three RTOS
    # `run_tiers.c` call `extern`. Its weak default used to live in
    # `<nros/main.hpp>`, so it reached an image only through the generated
    # `.cpp` entry: a pure C entry compiled and then failed at LINK. This is a
    # LINK probe because that is the only stage where the bug is visible —
    # `-fsyntax-only` is green either way, and so is `check-weak-symbols`,
    # which checks WHERE the attribute lives and not whether C can reach it.
    # No archive, no C++ header: the seam must resolve from the C headers a C
    # entry already includes. `test -f` first — a missing TU must not read as a
    # pass (issue 0196's class).
    for _tu in entry_seam_c_linkage entry_seam_c_override; do \
        test -f "packages/api/nros-c/tests/run/$_tu.c" || { \
            echo "ERROR: $_tu.c is MISSING -- this check passes on absence." >&2; exit 1; }; \
    done
    mkdir -p tmp
    cc -std=c11 -Wall -Wextra -Wmissing-prototypes \
        -Ipackages/api/nros-c/include \
        packages/api/nros-c/tests/run/entry_seam_c_linkage.c \
        -o tmp/entry_seam_c_linkage
    ./tmp/entry_seam_c_linkage
    # The override is the reason the hook is weak; nothing else in tree
    # exercises it, so a regression would surface only in an out-of-tree image.
    cc -std=c11 -Wall -Wextra -Wmissing-prototypes \
        -Ipackages/api/nros-c/include \
        packages/api/nros-c/tests/run/entry_seam_c_linkage.c \
        packages/api/nros-c/tests/run/entry_seam_c_override.c \
        -o tmp/entry_seam_c_override
    ./tmp/entry_seam_c_override > tmp/entry_seam_c_override.log
    # `nros_grep_q`, not `grep -q`: grep exits 1 for "no match" and >=2 for a
    # tool ERROR, and a bare `grep -q … || finding` reports the second as the
    # first (issue 0726). Here that would claim the override stopped working.
    # shellcheck source=scripts/lib/grep-q.sh
    source scripts/lib/grep-q.sh
    # `&& _rc=0 || _rc=$?`, not a bare call followed by `case $?`: this recipe
    # runs under `set -e`, which exits on the non-zero BEFORE the case is
    # reached, so the branch's diagnostic never prints and the lane fails
    # naming nothing. (`nros_grep_q` still `exit 2`s past this on a tool
    # failure — `||` cannot catch an exit.)
    nros_grep_q 'STRONG-OVERRIDE-RAN' tmp/entry_seam_c_override.log && _rc=0 || _rc=$?
    if [ "$_rc" -ne 0 ]; then \
        echo "ERROR: the strong nros_board_network_wait override did NOT win over" >&2; \
        echo "       the weak default in <nros/main.h>. A board can no longer take" >&2; \
        echo "       the hook. See tmp/entry_seam_c_override.log" >&2; \
        exit 1; \
    fi
    echo "  - executor verb spelling + deprecated aliases (issue 0338)"
    # The entity-registration family is `nros_executor_add_*` (rclc's spelling);
    # the old `register_*` names survive one release as macro aliases. The probe
    # takes function POINTERS, so an alias naming a symbol that does not exist
    # fails here rather than at some consumer's link step.
    cc -fsyntax-only -std=c11 \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/executor_verb_aliases.c
    echo "  - graph query entry points are DECLARED, not just defined (phase-381 W4)"
    # The first version of these five was macro-generated. It compiled, this
    # lane passed, and cbindgen emitted nothing for them — cbindgen does not
    # expand macros, so four of five had no header declaration and were
    # uncallable from C while the Rust side stayed green. The probe takes
    # function POINTERS, so a missing declaration is a build failure here.
    cc -fsyntax-only -std=c11 \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/graph_query_entry_points.c
    echo "  - a message's serialization format matches the linked backend (RFC-0088 D5)"
    # phase-421 W6 — every generated C message header carries a
    # `_Static_assert` comparing its format id against the image's. The positive
    # TU proves the assertion is reachable and does not fire on a matching pair;
    # the probe below proves it fires on a mismatched one.
    cc -fsyntax-only -std=c11 \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/serialization_format.c
    # Expected-failure, and asserted-present first for the same reason the
    # parameter probe below is: a missing file is also a non-zero cc, and reads
    # as a pass.
    test -f packages/api/nros-c/tests/compile/serialization_format_mismatch_probe.c || { \
        echo "ERROR: serialization_format_mismatch_probe.c is MISSING -- the expected-failure" >&2; \
        echo "       check below would PASS on a missing file." >&2; \
        exit 1; \
    }
    if cc -fsyntax-only -std=c11 \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/serialization_format_mismatch_probe.c 2>/dev/null; then \
        echo "ERROR: a message declaring a format the linked backend does not speak compiled clean." >&2; \
        echo "       NROS_ASSERT_MESSAGE_FORMAT is not reaching callers, so a uORB-encoded" >&2; \
        echo "       message would publish through a CDR backend with no diagnostic." >&2; \
        exit 1; \
    fi
    echo "  - parameter / service-reply / receive-verb / action-count entry points"
    # phase-417 stage 6 step B RETIRED every NROS_DEPRECATED_MSG forwarder in
    # <nros/*.h>, so the four TUs below no longer have an old half to pin and
    # the five expected-failure deprecation probes that asserted the attribute
    # reached callers are DELETED with them. Deleting rather than keeping was
    # the only honest option: an expected-failure compile cannot tell "the
    # deprecation fired" from "the identifier does not exist", so each probe
    # would have gone on passing while asserting nothing.
    #
    # What survives is the half that was never about deprecation: every live
    # entry point, pinned by SIGNATURE. Function POINTERS again, so a
    # declaration whose argument list drifts from the Rust definition cbindgen
    # emitted fails here rather than at a consumer's call site.
    #
    # `test -f` first on every one: a syntax-only compile of a MISSING file is
    # a pass, and phase-379 W5 shipped two gates whose probes were never
    # committed (issue 0196's class).
    for probe in param_entry_points service_reply_entry_point \
                 receive_verb_entry_points action_goal_count_entry_point; do \
        test -f "packages/api/nros-c/tests/compile/$probe.c" || { \
            echo "ERROR: $probe.c is MISSING -- this check passes on absence." >&2; exit 1; }; \
        cc -fsyntax-only -std=c11 \
            -Itarget/nros-c-generated \
            -Ipackages/api/nros-c/include \
            -Ipackages/platform/nros-platform-api/include \
            "packages/api/nros-c/tests/compile/$probe.c" || exit 1; \
    done

    # phase-417 stage 5 — the typed C surface a ported rclc file reaches, plus
    # the rcl spelling layer. `test -f` first on every one: a syntax-only
    # compile of a MISSING file is a pass, and this campaign has already shipped
    # one probe that never ran (issue 0196's class).
    echo "  - phase-417 stage 5: typed delivery, node/timer accessors, rcl spellings"
    for probe in typed_subscription_delivery node_timer_accessors rcl_compat_aliases; do \
        test -f "packages/api/nros-c/tests/compile/$probe.c" || { \
            echo "ERROR: $probe.c is MISSING -- this check passes on absence." >&2; exit 1; }; \
        cc -fsyntax-only -std=c11 -Wall -Wextra \
            -Itarget/nros-c-generated \
            -Ipackages/api/nros-c/include \
            -Ipackages/platform/nros-platform-api/include \
            "packages/api/nros-c/tests/compile/$probe.c" || exit 1; \
    done
    # The typed SERVICE probe includes the committed golden output of the
    # service codegen pack, so a pack edit that emits uncompilable C fails here
    # in seconds rather than during a fixture build.
    test -f packages/api/nros-c/tests/compile/typed_service_handler.c || { \
        echo "ERROR: typed_service_handler.c is MISSING." >&2; exit 1; }
    cc -fsyntax-only -std=c11 -Wall -Wextra \
        -Ipackages/cli/rosidl-codegen/tests/fixtures/fingerprint-corpus/expected/configured \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/typed_service_handler.c
    # phase-417 stage 6 — the rename+reorder guards. Both carry their pragma
    # INSIDE the TU, so the wiring needs no flags; both are expected-FAILURE.
    #
    # The reorder one matters most and its guard is measured, not assumed:
    # WITHOUT `#pragma GCC diagnostic error "-Wincompatible-pointer-types"` the
    # same file compiles at rc=0, because C diagnoses an incompatible pointer
    # ARGUMENT as a warning even under -Wall -Wextra. That is why a C reorder
    # ships with a RENAME beside it (RFC-0089): a stale call must fail on the
    # identifier, which C does diagnose fatally.
    for probe in rcl_node_init_reorder_probe; do \
        f="packages/api/nros-c/tests/compile/$probe.c"; \
        test -f "$f" || { echo "ERROR: $f is MISSING -- an expected-failure check" >&2; \
                          echo "       passes on a missing file." >&2; exit 1; }; \
        if cc -fsyntax-only -std=c11 -Wall -Wextra \
            -Itarget/nros-c-generated \
            -Ipackages/api/nros-c/include \
            -Ipackages/platform/nros-platform-api/include \
            "$f" 2>"tmp/rename-$probe.log"; then \
            echo "ERROR: $f COMPILED. The stage-6 guard for '$probe' is not firing;" >&2; \
            echo "       a stale call now binds silently. See RFC-0089." >&2; exit 1; \
        fi; \
    done

    # ...and the rcl collision guard must FIRE. nano-ros and the real rcl define
    # the same RCL_RET_* with DIFFERENT values, so a TU holding both compiles
    # and silently takes whichever won the include race.
    test -f packages/api/nros-c/tests/compile/rcl_compat_collision_probe.c || { \
        echo "ERROR: rcl_compat_collision_probe.c is MISSING -- the expected-failure" >&2; \
        echo "       check below would PASS on a missing file." >&2; exit 1; }
    if cc -fsyntax-only -std=c11 \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/rcl_compat_collision_probe.c 2>/dev/null; then \
        echo "ERROR: rcl_compat_collision_probe.c compiled clean. <nros/rcl_compat.h>'s" >&2; \
        echo "       guard against the real <rcl/types.h> is gone; a TU with both now" >&2; \
        echo "       silently takes one of two DIFFERENT numberings." >&2; exit 1; \
    fi
    echo "  - cross-include (nros_cpp_ffi.h + component.h in one TU)"
    # Issue 0160 — the C prototypes and struct typedefs component.h re-declares
    # must stay compatible with cbindgen's canonical nros_cpp_ffi.h (the
    # phase-273 callback_group arity drift class). Including BOTH headers in
    # one TU (ffi.h first, so component.h's hand mirrors are guarded out) makes
    # the compiler the drift gate: any divergence is a "conflicting types"
    # error. Field-level struct-mirror parity is the buildless
    # ffi-struct-mirrors gate (push lane).
    # issue 0400 follow-up — this build GENERATES the headers the checks below
    # compile against, so its failure must not be swallowed. It used to run
    # `--quiet 2>/dev/null || true`, which turned "the generator never ran" into
    # "your C headers are broken": every `*_OPAQUE_U64S` came back undeclared
    # from a `cc` line that says nothing about cargo. Diagnosing that cost four
    # wrong hypotheses; the build's own error names the cause in one line.
    cargo build -p nros-cpp --no-default-features --features "{{C_API_SHIPPED_FEATURES}}" --quiet
    cc -fsyntax-only \
        -Itarget/nros-c-generated \
        -Itarget/nros-cpp-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/api/nros-cpp/include \
        -include packages/api/nros-cpp/include/nros/nros_cpp_ffi.h \
        -include packages/api/nros-c/include/nros/component.h \
        -x c /dev/null
    echo "  - rmw ABI layout static-asserts (issue #238/#239)"
    # The RMW C headers and their Rust `#[repr(C)]` mirrors are hand-kept
    # in lockstep. `abi_layout_check.c` is a `_Static_assert`-only TU that
    # pins the C-side widths (event-kind int-size, qos size, handle
    # alignment, vtable pointer-slot count); the Rust half is the
    # `abi_layout` const-assert block in nros-rmw-cffi/src/lib.rs. Either
    # side drifting fails exactly one guard. `-fsyntax-only` still
    # evaluates static asserts.
    cc -fsyntax-only \
        -Ipackages/core/nros-rmw-abi/include \
        packages/rmw/cffi/tests/c_stubs/abi_layout_check.c
    echo "All C checks passed!"

# issue 0379 — clippy gate for the CLI sub-workspace. No lane ran clippy on
# packages/cli, so ~107 warnings accreted unnoticed. Mirrors cli-tests
# (separate workspace, its own Cargo.toml/lock). Two deliberate deviations from
# a bare `--workspace` clippy:
#   * `--exclude ros-launch-manifest-{model,sched,types}` — these are vendored
#     submodule crates under third-party/, pulled in as path-dep members; their
#     lints are upstream's, out of scope here.
#   * `--locked` sits BEFORE `--`: the scripts/bin/cargo shim (issues 0359/0378)
#     appends $NROS_CARGO_FLAGS at the END of argv, which would land after `--`
#     and reach clippy-driver ("Unrecognized option: 'locked'"). Passing it
#     ourselves makes the shim skip its own injection.
[private]
cli-clippy:
    #!/usr/bin/env bash
    set -e
    cargo clippy --manifest-path packages/cli/Cargo.toml --workspace --all-targets \
        --exclude ros-launch-manifest-model \
        --exclude ros-launch-manifest-sched \
        --exclude ros-launch-manifest-types \
        --locked -- -D warnings
    echo "CLI clippy passed!"

# Issue 0363 — a stale in-tree `nros` used to surface at `check-dep-chain`,
# minutes in, as nine failed cells whose printed cause was a cargo resolution
# error. Same predicate (it CALLS `nros_cli_bin`), better position. Buildless.
#
# Issue 0466 — that position is the head of `check-build`, NOT of `check-fast`.
# `check-dep-chain`, the gate this exists to front-run, is a build-tier gate; no
# fast-tier gate execs the CLI at all. Sitting in the fast tier it contradicted
# that tier's whole contract ("needs neither the nros CLI nor any provisioned
# source"): every buildless gate became unreachable on a tree whose CLI was
# merely out of date — which ANY pull, rebase or stash makes it, since the stamp
# covers CLI sources. Measured: `just check fast` failed in 0.77s having checked
# nothing. Four source-level reds sat on main behind exactly this.
#
# phase-395 W12 — `check-dep-chain` has since MOVED OUT of `check-build` to
# post-submit (158 s measured, an 8-cell board×rmw matrix that no single pull
# request changes), so this no longer front-runs it and must not be read as
# doing so. It stays at the head of `check-build` on its own merits: a stale CLI
# breaks codegen for every gate below it, and failing in 0.1 s with the real
# cause beats failing minutes in with a cargo resolution error. The post-submit
# lane runs this same predicate before its own dep-chain step.
[private]
cli-fresh:
    @bash scripts/check-cli-fresh.sh

[private]
cli-tests:
    #!/usr/bin/env bash
    set -e
    cargo test --manifest-path packages/cli/Cargo.toml --workspace --quiet
    echo "CLI tests passed!"

# A COMPILE SMOKE TEST for the pull-request gate — phase-395.
#
# `check-build` compiles the workspace roughly ten times: six clippy feature
# combos, `--all-targets` with `-D warnings`, plus the C/C++/Cyclone/XRCE
# backend lanes. That is 587 s of an 878 s gate, and it is why the compile tier
# moved to the merge group.
#
# But moving it there means a plainly broken pull request is caught by the QUEUE
# rather than by its own gate, where it costs a batch slot and an ejection. This
# compiles the workspace ONCE — no lints, no feature matrix, no backends — to
# catch that class on the PR while leaving the expensive verification batched.
#
# `--all-targets` on purpose: tests and benches are where a signature change
# most often fails to propagate, and they are not built by a bare `cargo check`.
#
# It does NOT replace `check-build` and must never be confused for it: no
# clippy, so no `-D warnings`; no feature combinations, so a break that only
# appears under `--no-default-features` survives this and is caught in the
# queue. It is a smoke test, and the name says so.
[private]
compile-smoke:
    #!/usr/bin/env bash
    set -euo pipefail
    source scripts/build/cargo.sh
    # The same exclusions `check-test-targets` uses: these crates are `no_std`
    # or build-only and cannot compile for the host — a bare `--workspace` dies
    # on `#[panic_handler] function required` and reports a defect that is not
    # one.
    cargo check --workspace --all-targets --quiet {{HOST_UNCHECKABLE}}
    # Issue 1163 — the two C-API crates in the shape they SHIP, not their
    # default. `nros-c` / `nros-cpp` are in `HOST_UNCHECKABLE` because their
    # LINK step needs a target, but a `cargo check` never links, and their
    # default feature (`panic-platform`) compiles almost none of the C surface:
    # a duplicate `#[no_mangle]` inside an `rmw-cffi` region reached main green
    # through this very smoke test, while `check-c` — the lane that compiles
    # `C_API_SHIPPED_FEATURES` — runs on schedule only. Measured 25 s warm.
    # A `check`, not a build: the header generation `check-c` needs from the
    # build stays where it is.
    cargo check -p nros-c -p nros-cpp --no-default-features --features "{{C_API_SHIPPED_FEATURES}}" --quiet
    echo "compile smoke: workspace + all targets check clean (no lints; see check-build);"
    echo "  nros-c + nros-cpp check clean in the shipped shape ({{C_API_SHIPPED_FEATURES}})."

# Check C++ headers: formatting + freestanding syntax + nros-cpp clippy. The
# clippy (rmw-zenoh-cffi) + syntax probe COMPILE nros-cpp/nros-c (zpico-sys pulls
# the zenoh-pico source submodule) → source-dependent → build tier (check-build),
# NOT the source-free push lane.
[private]
cpp: cpp-fmt
    #!/usr/bin/env bash
    set -e
    echo "Checking C++ headers (build + syntax + clippy)..."
    echo "  - freestanding syntax (c++14)"
    # parameter.hpp re-exposes the C-side `nros_parameter_*` API from
    # nros-c, so the syntax probe needs nros-c on the include path too.
    # The per-variant `<nros/nros_cpp_config_generated.h>` (defining
    # `NROS_CPP_EXECUTOR_STORAGE_SIZE` and friends, referenced by
    # `executor.hpp`'s `uint8_t storage_[NROS_CPP_EXECUTOR_STORAGE_SIZE]`)
    # is emitted by `nros-cpp`'s build.rs into
    # `target/nros-cpp-generated/`. Same C-side header for nros-c.
    # Build both first; variant dirs go FIRST on the include path so
    # their real headers win over the source-tree stubs.
    # issue 0400 follow-up — this build GENERATES the headers the checks below
    # compile against, so its failure must not be swallowed. It used to run
    # `--quiet 2>/dev/null || true`, which turned "the generator never ran" into
    # "your C headers are broken": every `*_OPAQUE_U64S` came back undeclared
    # from a `cc` line that says nothing about cargo. Diagnosing that cost four
    # wrong hypotheses; the build's own error names the cause in one line.
    cargo build -p nros-c -p nros-cpp --no-default-features --features "{{C_API_SHIPPED_FEATURES}}" --quiet
    # issue 0730 — the (zephyr-embedded × C++ × cyclonedds) coordinate is a
    # pairwise-class gap no runtime tier covers (cyclone fixtures run on
    # native_sim, which appends `,std`), and it failed E0599 at COMPILE time:
    # the cyclone branch was the one bare-`rmw-cffi` composition, so
    # `TransportError::BackendDynamic` (alloc-gated) vanished under the
    # exhaustive mapper. Check that exact composition on an embedded target.
    # The feature string is READ FROM the zephyr module's cyclone branch, so
    # this gate follows the lane instead of being a second spelling of it.
    if rustup target list --installed | grep -qx aarch64-unknown-none; then
        _cyc_features="$(sed -n '/CONFIG_NROS_RMW_CYCLONEDDS/,/endif()/ s/.*set(_nros_cpp_features "\([^"]*\)").*/\1/p' zephyr/CMakeLists.txt | head -1)"
        if [ -z "$_cyc_features" ]; then
            echo "check-cpp: could not extract the cyclone _nros_cpp_features line from zephyr/CMakeLists.txt" >&2
            exit 1
        fi
        echo "  - embedded cyclone coordinate (issue 0730): ${_cyc_features},panic-platform @ aarch64-unknown-none"
        cargo check -p nros-cpp --no-default-features \
            --features "${_cyc_features},panic-platform" \
            --target aarch64-unknown-none --quiet
    else
        echo "  - embedded cyclone coordinate (issue 0730): SKIP — aarch64-unknown-none not installed"
    fi
    for hdr in packages/api/nros-cpp/include/nros/*.hpp; do
        # Phase 209 — `rclcpp_compat.hpp` is a source-compat shim still
        # being aligned with the live nros::Result / nros::QoS API. The
        # clang-format check above still covers it; the freestanding
        # C++14 probe stays opt-out until 209 lands its API touch-ups.
        case "$hdr" in *rclcpp_compat.hpp) continue ;; esac
        # issue #52 — `main.hpp` is the HOSTED entry runtime (LinuxBoard / NuttX):
        # its rtos_e2e readiness/sample banners call `::std::printf`, which
        # `-ffreestanding` is not required to expose from `<cstdio>` (only the global
        # `printf`). Probe it hosted so it keeps full syntax coverage; every other
        # header stays freestanding.
        free="-ffreestanding"
        case "$hdr" in *main.hpp) free="" ;; esac
        # issue #52 — `nros-platform-api/include` carries `<nros/platform.h>`, pulled
        # by `heap_sequence.hpp` (Phase 229.5); without it the probe fails
        # `fatal error: nros/platform.h: No such file or directory`.
        c++ -fsyntax-only -std=c++14 $free -fno-exceptions -fno-rtti \
            -Itarget/nros-cpp-generated \
            -Itarget/nros-c-generated \
            -Ipackages/api/nros-cpp/include \
            -Ipackages/api/nros-c/include \
            -Ipackages/platform/nros-platform-api/include \
            -include "$hdr" -x c++ /dev/null
    done
    # Issue 0089 gap-4 — typed-API INSTANTIATION probe (the header loop only
    # parses templates). Compiles a TU that instantiates `nros::bind_service`
    # against a generated-shape service type, so the template body is checked.
    echo "  - typed bind_service instantiation (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/bind_service.cpp
    echo "  - graph query entry points are DECLARED, not just defined (phase-381 W4)"
    # Same trap as the C side: the C++ FFI header is cbindgen-generated, and
    # cbindgen does not expand macros — a macro-generated `#[no_mangle]` gets no
    # declaration and is uncallable while the Rust side stays green. Method
    # POINTERS make that a build failure here.
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/graph_query_entry_points.cpp
    echo "  - lifecycle transition ids ARE lifecycle_msgs, in BOTH languages (issue 1099)"
    # `nros::LifecycleNode::trigger_transition(uint8_t)` has rclcpp's exact
    # signature, so its id space must be rclcpp's. It was not: four of eight
    # ids disagreed with `lifecycle_msgs/msg/Transition`, and a ported
    # `trigger_transition(2)` on an Inactive node ACTIVATED it and returned OK
    # where ROS 2 cleans it up. Nothing could catch it -- `correlate.py` never
    # compares enum VALUES, only names and arities.
    #
    # The TU also pins the C++ enum against the C `NROS_LIFECYCLE_TRANSITION_*`
    # macros: one id space per image, so a mixed C/C++ node cannot hold two.
    # And it pins `shutdown()` resolving from the CURRENT state -- it hardcoded
    # 5, whose only legal source state is Unconfigured.
    test -f packages/api/nros-cpp/tests/compile/lifecycle_transition_ids.cpp || { \
        echo "ERROR: lifecycle_transition_ids.cpp is MISSING -- issue 1099 is unpinned." >&2; \
        exit 1; \
    }
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/lifecycle_transition_ids.cpp
    echo "  - a module can NAME the backend it means (issue 1050 defect 3)"
    # `nros::init()` opens whichever backend registered FIRST, and on a hosted
    # target that order is decided before `main` by `.init_array` ctors. A PX4
    # uORB-only module linked against a zenoh-carrying archive was handed zenoh
    # and failed dialling a router nobody asked for. The capability existed on
    # `Executor::open_with_rmw` and `NodeBuilder().rmw(...)`; the one-liner the
    # examples use could not reach it. Surface probe, including the defaults
    # that make it a one-liner.
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/init_names_its_backend.cpp
    echo "  - a message's serialization format matches the linked backend (RFC-0088 D5)"
    # phase-421 W6 — codegen specialises `nros::format_of<M>` per message and the
    # typed creators `static_assert` on it. The positive TU proves the assertion
    # is reachable; the probe proves it fires.
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/serialization_format.cpp
    echo "  - an unbounded type cannot size a buffer (issue 0964)"
    # Expected-failure, and asserted-present first for the same reason every
    # other probe here is: a missing file is also a non-zero c++, and reads as a
    # pass.
    test -f packages/api/nros-cpp/tests/compile/unbounded_buffer_probe.cpp || { \
        echo "ERROR: unbounded_buffer_probe.cpp is MISSING -- the expected-failure" >&2; \
        echo "       check below would PASS on a missing file." >&2; \
        exit 1; \
    }
    if c++ -fsyntax-only -std=c++17 \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/unbounded_buffer_probe.cpp 2>/dev/null; then \
        echo "ERROR: a type with NO derived bound sized a buffer and compiled clean." >&2; \
        echo "       detail::buffer_bounds' unbounded arm has been disarmed, so an" >&2; \
        echo "       INVENTED number is back in a stack array -- and the estimate" >&2; \
        echo "       under-states as well as over-states, so the buffer can be too" >&2; \
        echo "       small for a message the user legitimately sends (issue 0964)." >&2; \
        exit 1; \
    fi
    test -f packages/api/nros-cpp/tests/compile/serialization_format_mismatch_probe.cpp || { \
        echo "ERROR: serialization_format_mismatch_probe.cpp is MISSING -- the expected-failure" >&2; \
        echo "       check below would PASS on a missing file." >&2; \
        exit 1; \
    }
    if c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/serialization_format_mismatch_probe.cpp 2>/dev/null; then \
        echo "ERROR: a message declaring a format the linked backend does not speak compiled clean." >&2; \
        echo "       NROS_CPP_ASSERT_MESSAGE_FORMAT is not reaching the typed creators." >&2; \
        exit 1; \
    fi
    # issue 0338 — `spin` verb SHAPE probe: `spin()` blocks until shutdown
    # (rclcpp/C/Rust semantics) and the bounded verb is `spin_for(...)`. The
    # defect was the shape of the API, so a compile-time assertion on which
    # arities exist is what catches a regression.
    echo "  - spin verb shape (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/spin_verbs.cpp
    # issue 0278 — PollingSubscription<M> (latest-value polling subscriber)
    # instantiation: the wrapper bodies (drain/take_data/take_new_data/take)
    # and the create_polling_subscription factory path are type-checked against
    # a generated-shape message type.
    echo "  - PollingSubscription instantiation (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/polling_subscription.cpp
    # phase-408 W1/W4 (issue 0896) — the receive-buffer hint a typed C++
    # subscription supplies is `nros::rx_size_bound<M>`, the type's own DERIVED
    # bound, and no longer `M::SERIALIZED_SIZE_MAX` (an estimate that
    # under-sizes a nested type over 512 B — issue 0964). That is a template
    # BODY, so the header parse loop above never type-checks it; this TU
    # instantiates `bind_subscription` and pins the three shapes the trait
    # distinguishes: derived-and-bounded, no-derivation (legacy fallback), and
    # derived-but-poisoned (which must be INCLUDABLE without error).
    # issue 0964 — the same TU also covers `nros::rx_buffer_capacity<M>`, which
    # is what the 13 RECEIVE buffers inside these headers now stack, and
    # instantiates every one of them for a BOUNDED and an UNBOUNDED type. The
    # second column is the point: a blanket switch to `rx_size_bound<M>` would
    # be a compile error for 81 of 120 stock Humble types.
    echo "  - rx_size_bound / rx_buffer_capacity / receive sites (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/rx_size_bound.cpp
    # issue 0278 Half B — Client<Svc>::call_polling (callback-safe bounded
    # service call, no executor spin) instantiation: the method body
    # (serialize -> nros_cpp_service_client_call_raw(..., timeout_ms) ->
    # deserialize) is type-checked against a generated-shape service type.
    echo "  - Client::call_polling instantiation (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/service_client_call_polling.cpp
    # issue 0796 — the action CALLBACK tier: the accepted-goal hook
    # (`set_accepted_callback`) and the client-side `cancel_goal`, both of
    # which C and Rust had and C++ did not. Template BODIES, so the header
    # parse loop above never type-checks them; this TU instantiates them and
    # static-asserts that the per-goal `CancelResponse` and the RPC
    # `CancelReturnCode` stay distinct types.
    echo "  - action callback tier instantiation (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/action_callback_tier.cpp
    # phase-379 W5 — the QoS surface under its rclcpp names. The policy enums
    # are namespace-scope and public, the getters return them instead of `int`,
    # and the three time windows take/return `nros::Duration`. This TU asserts
    # the RETURN TYPES and the millisecond boundary, not just that the names
    # resolve: a `deadline()` that came back as `uint32_t` would pass a name
    # check and re-introduce the ledgered defect. It also static_asserts that
    # `nros_cpp_qos_t`'s `*_ms` FIELDS are still `uint32_t` milliseconds — the
    # same token is a struct field (ABI, must not move) and was a class method
    # (renamed), so a textual sweep is exactly what this catches (issue 0160).
    # `-Wno-deprecated-declarations` because the TU names the old spellings
    # deliberately; the probe below is what proves they still warn.
    echo "  - QoS policy accessors + ms boundary (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti -Wno-deprecated-declarations \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/qos_policy_accessors.cpp
    # ...and the old spellings must WARN. A deprecation nobody is told about is
    # an alias. NEGATIVE assertion: with `-Werror=deprecated-declarations` the
    # old spelling must FAIL to compile. (The C lane had four probes of this
    # shape; phase-417 stage 6 step B retired the forwarders they guarded and
    # deleted them with it. The C++ ones stay because the C++ deprecations do.)
    # Written as an expected-failure because
    # a passing compile is exactly what a silently-dropped attribute looks like.
    # An expected-failure compile cannot distinguish "the deprecation
    # attribute fired" from "the file is not there" -- both are a non-zero
    # cc, and the second reads as a pass. Phase-379 W5 landed this check
    # without its probe and the gate was silently vacuous. Assert first.
    test -f packages/api/nros-cpp/tests/compile/qos_deprecation_probe.cpp || { \
        echo "ERROR: qos_deprecation_probe.cpp is MISSING -- the expected-failure check below" >&2; \
        echo "       would PASS on a missing file, which is why this asserts first." >&2; \
        exit 1; \
    }
    if c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti -Werror=deprecated-declarations \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/qos_deprecation_probe.cpp 2>/dev/null; then \
        echo "ERROR: qos_deprecation_probe.cpp compiled clean under -Werror=deprecated-declarations." >&2; \
        echo "       The [[deprecated]] attributes on QoS::*_raw() / QoS::*_ms() /" >&2; \
        echo "       QoS::Liveliness in nros/qos.hpp are not reaching callers, so the" >&2; \
        echo "       rename ships with no migration warning at all." >&2; \
        exit 1; \
    fi
    # phase-403 step 2 — the DECLARED `@depth=` reaches the compiler, and a
    # disagreement between it and the QoS at the call site FAILS THE BUILD.
    #
    # `-std=c++17` and not the c++14 of its neighbours: C++17 is what the
    # component lane actually compiles with, and the absence of `__VA_OPT__`
    # there is the reason `NROS_SUBSCRIBE` dispatches on argument count at all.
    # Checking the macro under a standard no component uses would test a
    # different preprocessor.
    #
    # The fixture dir carries `nros/nros_declared_qos_generated.h` — what
    # `nros ws entity-inventory --output-header` renders, and the same bytes a
    # real component's configure puts on its own include path (issue 1084: the
    # register call claims the dir, a deferred render fills it from the
    # contract's model). Putting it on `-I` here exercises the same
    # `__has_include` pickup; `check-declared-qos-header` is the DELIVERY half
    # and drives the real configure. The header and its input are held together by
    # `the_committed_compile_fixture_is_what_this_emitter_renders` in
    # nros-cli-core, so this fixture cannot drift into a table that matches
    # nothing (which would leave every assertion below vacuous and green).
    # phase-417 stages 1 + 2b — the ROS 2 adoption surface. Two probes, both
    # POSITIVE: the facilities a ported rclcpp file reaches must exist and
    # type-check. `test -f` first, because a syntax-only compile of a missing
    # file is a pass, and six mechanisms in this campaign were right and never
    # ran (issue 0196's class).
    #
    # `ros2_api_adoption.cpp` is HOSTED deliberately — no -ffreestanding. It
    # includes `rclcpp_compat.hpp`, which pulls <memory>/<string> unconditionally
    # and hands out std::shared_ptr in public signatures, so a freestanding
    # compile would fail for a reason that says nothing about the surface.
    for probe in ros2_api_adoption node_graph_forwarders; do \
        test -f "packages/api/nros-cpp/tests/compile/$probe.cpp" || { \
            echo "ERROR: $probe.cpp is MISSING -- this check would pass on absence." >&2; \
            exit 1; \
        }; \
    done
    echo "  - phase-417: ported-file surface COMPILES (aliases, string interop, graph, actions, executor)"
    for probe in ros2_api_adoption node_graph_forwarders ros2_api_adoption_stage2 ros2_one_dispatch_path \
                 ros2_init_argv_refusal action_goal_uuid executor_cancel; do \
        c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
            -Itarget/nros-cpp-generated \
            -Itarget/nros-c-generated \
            -Ipackages/api/nros-cpp/include \
            -Ipackages/api/nros-c/include \
            -Ipackages/platform/nros-platform-api/include \
            "packages/api/nros-cpp/tests/compile/$probe.cpp" || exit 1; \
    done
    # c++17 alone: it enables NROS_SYSTEM_PARAM_SERVICES in-file, which reaches
    # `ComponentNode::adopt_launch_seed_`'s `if constexpr`.
    c++ -fsyntax-only -std=c++17 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/ros2_param_launch_seed.cpp

    # `nros_grep_q`, not `grep -q`: grep exits 1 for "no match" and >=2 for a
    # tool ERROR, and a bare `grep -q ... || finding` reports the second as the
    # first (issue 0726 — a forked grep that failed to start under a 32-way
    # fan-out was reported as a missing anchor, a false and specific claim).
    # shellcheck source=scripts/lib/grep-q.sh
    source scripts/lib/grep-q.sh
    # phase-417 stage 3 — the REFUSALS must FAIL, each with its OWN diagnostic.
    # Exit code alone is not enough: a typo in an include path also exits
    # non-zero and would read as "the refusal fired". So each is grepped for the
    # marker its message carries, and `test -f` runs first because a missing
    # file compiles to nothing and passes.
    echo "  - phase-417: the refusals FAIL, each naming its own constraint"
    for probe in node_options system_defaults_qos log_throttle service_callback; do \
        f="packages/api/nros-cpp/tests/compile/ros2_refuse_${probe}_probe.cpp"; \
        test -f "$f" || { echo "ERROR: $f is MISSING -- this check passes on absence." >&2; exit 1; }; \
        if c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
               -Itarget/nros-cpp-generated -Itarget/nros-c-generated \
               -Ipackages/api/nros-cpp/include -Ipackages/api/nros-c/include \
               -Ipackages/platform/nros-platform-api/include "$f" 2>"tmp/refuse-$probe.log"; then \
            echo "ERROR: $f COMPILED. The refusal for '$probe' is not firing." >&2; exit 1; \
        fi; \
        nros_grep_q "REFUSED by nano-ros" "tmp/refuse-$probe.log" && _rc=0 || _rc=$?; \
        if [ "$_rc" -ne 0 ]; then \
            echo "ERROR: $f failed WITHOUT the refusal diagnostic -- it broke for" >&2; \
            echo "       another reason, so this check proves nothing. See tmp/refuse-$probe.log" >&2; \
            exit 1; \
        fi; \
    done

    echo "  - declared QoS depth: the three cases COMPILE (c++17)"
    c++ -fsyntax-only -std=c++17 -fno-exceptions -fno-rtti \
        -Ipackages/api/nros-cpp/tests/compile/declared-qos-fixture \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/declared_qos_depth.cpp
    # ...and a DISAGREEMENT must not. NEGATIVE assertion, and the one that makes
    # the check reachable rather than merely correct: six mechanisms in this
    # campaign were right and never ran. The positive TU above is compiled FIRST
    # so a missing include path or a fixture that matches nothing shows up as a
    # failure there, not as a silent pass here — an expected-failure compile
    # cannot tell "the assertion fired" from "the file is not there".
    test -f packages/api/nros-cpp/tests/compile/declared_qos_depth_probe.cpp || { \
        echo "ERROR: declared_qos_depth_probe.cpp is MISSING -- the expected-failure check" >&2; \
        echo "       below would PASS on a missing file, which is why this asserts first." >&2; \
        exit 1; \
    }
    echo "  - declared QoS depth: a MISMATCH fails the build (c++17, expected failure)"
    _dq_out="$(c++ -fsyntax-only -std=c++17 -fno-exceptions -fno-rtti \
        -Ipackages/api/nros-cpp/tests/compile/declared-qos-fixture \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/declared_qos_depth_probe.cpp 2>&1)" && { \
        echo "ERROR: declared_qos_depth_probe.cpp COMPILED. The fixture declares" >&2; \
        echo "       sub:std_msgs/msg/Int32:/chatter@depth=1 and the call site passes" >&2; \
        echo "       nros::QoS(10); NROS_SUBSCRIBE's static_assert did not fire, so the" >&2; \
        echo "       declared depth is not reaching the compiler and nothing is checked." >&2; \
        exit 1; \
    }
    # A rejection is not enough: it must be OUR rejection, naming the topic and
    # BOTH depths. A typo or a missing header would also exit non-zero, and a
    # diagnostic the reader cannot act on is the same as no diagnostic.
    for _want in '"/chatter"' 'declared_depth_agrees<1, 10>' 'contract.yaml'; do \
        case "$_dq_out" in \
            *"$_want"*) ;; \
            *) echo "ERROR: the declared-depth diagnostic does not contain: $_want" >&2; \
               echo "       It must name the TOPIC and BOTH depths, or a reader cannot tell" >&2; \
               echo "       which subscription is wrong or which of the two numbers to fix." >&2; \
               echo "--- compiler said ---" >&2; echo "$_dq_out" >&2; \
               exit 1 ;; \
        esac; \
    done
    # phase-379 W6 decision 1 — the receive verb under its rclcpp name.
    # `Subscription<M>::take` / `take_serialized` / `take_sequence` /
    # `take_validated` and `Service<S>::take_request`, with the `try_recv*`
    # spellings kept as `[[deprecated]]` forwarders. The header `-fsyntax-only`
    # loop above only PARSES the templates; this TU INSTANTIATES them, so a
    # forwarder whose argument list drifted from the method it forwards to
    # fails here. `-Wno-deprecated-declarations` because the TU names the old
    # spellings deliberately; the probe below is what proves they still warn.
    echo "  - receive verb aliases + deprecated forwarders (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti -Wno-deprecated-declarations \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/receive_verb_aliases.cpp
    # ...and the old spellings must WARN. NEGATIVE assertion, the C half's shape
    # one language over. An expected-failure compile cannot distinguish "the
    # deprecation attribute fired" from "the file is not there" -- both are a
    # non-zero c++, and the second reads as a pass. Assert first.
    test -f packages/api/nros-cpp/tests/compile/receive_deprecation_probe.cpp || { \
        echo "ERROR: receive_deprecation_probe.cpp is MISSING -- the expected-failure check below" >&2; \
        echo "       would PASS on a missing file, which is why this asserts first." >&2; \
        exit 1; \
    }
    if c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti -Werror=deprecated-declarations \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/receive_deprecation_probe.cpp 2>/dev/null; then \
        echo "ERROR: receive_deprecation_probe.cpp compiled clean under -Werror=deprecated-declarations." >&2; \
        echo "       The [[deprecated]] attributes on Subscription::try_recv* /" >&2; \
        echo "       Service::try_recv_request are not reaching callers, so the" >&2; \
        echo "       rename ships with no migration warning at all." >&2; \
        exit 1; \
    fi
    # issue #201 — HeapSequence element-destructor RUNTIME probe: compiled AND
    # executed (counting allocator in the TU; asserts zero live allocations
    # across dtor / move-assign / clear / reserve-relocation of a two-level
    # heap element shape).
    echo "  - HeapSequence element-lifetime runtime probe (c++14)"
    mkdir -p target/nros-cpp-tests
    c++ -std=c++14 -fno-exceptions -fno-rtti \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/platform/nros-platform-api/include \
        -o target/nros-cpp-tests/heap_sequence_lifetime \
        packages/api/nros-cpp/tests/compile/heap_sequence_lifetime.cpp
    ./target/nros-cpp-tests/heap_sequence_lifetime
    echo "  - nros-cpp clippy (zenoh-cffi + posix + humble)"
    cargo clippy --quiet -p nros-cpp --no-default-features --features "std,rmw-zenoh-cffi,platform-posix,ros-humble"
    echo "All C++ checks passed!"

# Phase 104.A.4 — assert `nros` + `nros-node` Cargo deps stay free of
# concrete RMW / platform crates. The umbrella must consume only the
# generic ABI (`nros-rmw-cffi` vtable + `nros-platform-cffi` C header);
# selecting a backend or platform is the outer build system's job.
#
# Today this guard is EXPECTED TO FAIL — Phase 104.A is the migration
# that brings it to green. Wire it as a required check once the
# migration completes.
[group("debug")]
decoupling:
    @bash scripts/check-decoupling.sh

# Per-platform (board, rmw) dependency-chain resolution — proves each cell's dep
# chain resolves (nros setup --dry-run + codegen + cargo tree, no compile). The
# `check.yml` step (SSoT). Needs ROS 2 sourced (for std_msgs .msg defs) + the
# nros CLI; SKIPS cleanly when ROS isn't sourced so `just check` still runs
# everywhere (CI sources ROS).
[private]
dep-chain:
    #!/usr/bin/env bash
    set -e
    if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
        if [ -f /opt/ros/humble/setup.bash ]; then
            source /opt/ros/humble/setup.bash
        else
            # shellcheck source=scripts/build/check-skip.sh
            source scripts/build/check-skip.sh
            nros_check_skip dep-chain "ROS 2 not sourced (AMENT_PREFIX_PATH unset)"; exit 0
        fi
    fi
    source scripts/build/cargo.sh
    NROS="$(nros_cli_bin)" scripts/ci/dep-chain-check.sh

# Embedded feature-unification guard — no `feature "std"` activation path may
# reach an embedded target's production-link view. The `check.yml` step (SSoT).
[private]
embedded-feature-unification:
    #!/usr/bin/env bash
    set -e
    # shellcheck source=scripts/lib/grep-q.sh
    source scripts/lib/grep-q.sh
    tree=$(cargo tree -p nros-serdes --edges=normal,build \
        --target thumbv7em-none-eabihf --no-default-features --workspace 2>&1)
    # issue 0726/0732 — a `grep -q` that cannot start reports "no `std` here",
    # which is this gate's PASS. A pipe makes it likelier still: the writer sees
    # SIGPIPE when grep dies early. `nros_grep_q` exits 2 rather than verdicting.
    nros_grep_q 'feature "std"' <<<"$tree" && found=0 || found=$?
    if [ "$found" -eq 0 ]; then
        echo "feature std activation paths under embedded target:" >&2
        echo "$tree" | grep -B2 'feature "std"' | head -50 >&2
        echo "Move the offending dep under [target.'cfg(not(target_os = \"none\"))'.dependencies]." >&2
        exit 1
    fi
    echo "no feature std paths under embedded target."

# issue 0560 reason 2 — `setup-launch-resolve` was a dependency of
# `build-test-fixtures` and NOTHING else, so a compile regression in the resolver
# waited for whoever next ran the ~40-minute fixture lane rather than failing its
# author. #560 gated the LOCK drift (sub-second, in `check-fast`); this covers
# the compile, which needs a build tier.
#
# It invokes the REAL recipe rather than a second `cargo check` spelling: same
# profile, same flags, and it catches link errors a `cargo check` would miss —
# ~14 s warm against 6 s, which is the cheaper half of the trade. The artifact it
# leaves is the one `nros sync` wants anyway.
#
# SKIPS when the submodule is absent, so `just check` still runs on a bare clone.
# That is deliberately NOT what `setup-launch-resolve` itself does: issue 0409
# made that recipe FAIL there, because its job is to produce the binary and
# exiting 0 without one let `nros sync` run on a stale resolver. A verification
# lane answers a different question — "can this be checked?" — and the honest
# answer without the submodule is no.
[private]
launch-resolve-builds:
    #!/usr/bin/env bash
    set -euo pipefail
    resolve="packages/cli/third-party/play_launch/src/ros-launch-resolve/resolve/Cargo.toml"
    if [ ! -f "$resolve" ]; then
        echo "[check-launch-resolve-builds] SKIP — play_launch submodule not initialised"
        echo "    git submodule update --init packages/cli/third-party/play_launch"
        exit 0
    fi
    just setup-launch-resolve

# Issue 0816 — the book promises no-alloc integrations and nothing verified the
# LINKED image; a Cargo feature gate is necessary and not sufficient, because a
# vendored C dependency or a weak-symbol fallback pulls an allocator in without
# any feature moving. Selftest + claims roster only: BUILDLESS, so it sits on
# the fast line beside the other self-testing generators.
#
# The image-level tiers are deliberately NOT wired into `ci` yet. `--claims`
# reports 0 of 4 book claims backed, because no example is BUILT no-alloc (all
# 13 bare-metal Rust leaves enable `alloc`) — the missing thing is a fixture,
# not a gate, and that is 0816's remaining half. The `--tier unified` object
# check is green on the zenoh funnel and red on a cyclone leaf (`operator new`
# in four TUs), so it waits on a fix or an `--allow` carrying a tracked issue id.
[private]
no-alloc-image:
    @python3 scripts/check-no-alloc-image.py --selftest
    @python3 scripts/check-no-alloc-image.py --claims

# no_std core-crate compile check across the embedded targets `ci.yml` gates
# (.github/workflows/ci.yml). Bare portable crates only — no SDKs, no link.
[group("ci")]
no-std:
    #!/usr/bin/env bash
    set -e
    crates="-p nros-core -p nros-log -p nros-serdes -p nros-params \
        -p nros-platform-api -p nros-platform-cffi -p nros-platform-critical-section -p nros-rmw \
        -p nros-node -p nros-platform -p nros-diagnostics -p nros"
    # phase-359 W1 — the four crates on the second line joined 2026-08-15. This
    # lane covered 9 of the 32 crates that DECLARE `no_std`, and `nros-node` —
    # the one with 85 of the ~190 `cfg(feature = "std")` sites, i.e. the crate
    # where a `std::` slip is most likely — was not among them. Until now the
    # only thing catching such a slip was an embedded fixture build: a real
    # backstop, but a ~40-minute one that fails whoever runs it rather than its
    # author.
    #
    # `$rmw_crates` below is NOT redundant with `$crates`, and the reason is the
    # whole point of the work item. `nros-node`'s executor lives behind
    # `#[cfg(any(has_rmw, test))]` — `has_rmw` is set by build.rs only when an
    # RMW feature is on — so a bare `--no-default-features` check compiles the
    # crate SHELL and none of the 85 sites. Verified rather than assumed: a
    # `std::string::String` planted in `node_record.rs` passes the bare check
    # (0.06s, cached) and fails the RMW-enabled one with
    # `cannot find module or crate `std``. A lane that only ran the bare check
    # would have been decoration — the issue-0196 shape, in a gate written to
    # prevent it.
    #
    # Still NOT covered: the board/platform-specific crates (they need SDKs and
    # a linker, which this lane deliberately excludes).
    rmw_crates="-p nros-node -p nros"
    # issue 0814 — the LENDING slice, heap-free. `nros-rmw-cffi/lending` and
    # `nros-node/rmw-lending` are the whole zero-copy surface, and NOTHING
    # builds them: the feature is enabled by exactly one crate in the tree
    # (`packages/testing/nros-tests`, and it names `platform-posix` while doing
    # it), it appears zero times under `cmake/` and `zephyr/`, and
    # `nros-rmw-zenoh-staticlib` declares no forwarder. So the one surface
    # sold as "for RAM-tight embedded targets" was compiled for no embedded
    # target at all, and the C half of it had in fact stopped compiling
    # entirely when issue 0812 retyped the loan token — for months, silently,
    # because no lane looked.
    #
    # This is a RATCHET, not new work: both rows passed before they were
    # written (measured on thumbv7em-none-eabi and re-measured on this lane's
    # two targets). What it buys is that they cannot quietly stop.
    #
    # NO `alloc`, deliberately — that combination is the one issue 0814's
    # `try_lend_slot` fix is about, and it is the tier where a heap is denied
    # outright.
    #
    # Why not a QEMU lane, which issue 0814 originally proposed: it would not
    # prove what it appears to. zenoh-pico memcpys the payload into a
    # non-expandable wire buffer regardless of whether we aliased it
    # (`_z_buf_encode` takes its no-copy branch only for an expandable wbuf;
    # every transport builds its TX wbuf with `_z_wbuf_make(mtu, false)`), so a
    # green "zero-copy works on hardware" lane would be green while every byte
    # is still copied. This costs seconds and catches the class that actually
    # bit.
    #
    # NOT here: `nros-c --features rmw-cffi,lending`. Its build script needs
    # `arm-none-eabi-gcc` for the C anchor, and SDK-needing crates are what
    # this lane deliberately excludes (see the note above). Its host-target
    # row lives in `check::workspace-features` instead — same class, same
    # issue, a lane that can afford it.
    lending_feats="nros-rmw-cffi/lending,nros-node/rmw-cffi,nros-node/rmw-lending"
    # `rustup target add` FIRST, serially: two concurrent adds of different
    # targets touch the same toolchain dir.
    for target in thumbv7m-none-eabi riscv32imc-unknown-none-elf; do
        rustup target add "$target" >/dev/null 2>&1 || true
    done
    # The two targets are independent `cargo check`s, so run them under the
    # jobserver rather than back to back (phase-336 W7). nros-rmw-cffi needs ptr
    # atomics — Cortex-M only (riscv32imc lacks them; mirrors ci.yml's per-target
    # crate set).
    source scripts/build/jobserver-pool.sh
    printf '%s\n' \
        "echo '== check-no-std: thumbv7m-none-eabi ==' && cargo check $crates -p nros-rmw-cffi --no-default-features --target thumbv7m-none-eabi" \
        "echo '== check-no-std: riscv32imc-unknown-none-elf ==' && cargo check $crates --no-default-features --target riscv32imc-unknown-none-elf" \
        "echo '== check-no-std: executor slice (alloc+rmw-cffi), thumbv7m ==' && cargo check $rmw_crates --no-default-features --features alloc,rmw-cffi --target thumbv7m-none-eabi" \
        "echo '== check-no-std: executor slice (alloc+rmw-cffi), riscv32imc ==' && cargo check $rmw_crates --no-default-features --features alloc,rmw-cffi --target riscv32imc-unknown-none-elf" \
        "echo '== check-no-std: lending slice (NO alloc), thumbv7m ==' && cargo check -p nros-rmw-cffi -p nros-node --no-default-features --features $lending_feats --target thumbv7m-none-eabi" \
        "echo '== check-no-std: lending slice (NO alloc), riscv32imc ==' && cargo check -p nros-rmw-cffi -p nros-node --no-default-features --features $lending_feats --target riscv32imc-unknown-none-elf" \
        | nros_pool_run no-std
    echo "no-std OK."

# nros-node's `std`-gated unit tests. `just test-all` runs `cargo nextest
# --workspace`, which builds each crate with its OWN default features — and
# `std` is not one of nros-node's — so seven `#[cfg(feature = "std")]` tests in
# `executor/tests.rs` existed in no lane at all. Confirmed against a full tier-1
# sweep: 152 `nros-node` cases ran and not one of those seven was among them.
#
# A test nothing runs is worse than no test, and this is what that costs:
# `violations_beyond_the_ring_are_counted` (issue 0514's overflow-counting
# proof) asked for `MAX_VIOLATIONS + 4` callback slots against a `MAX_CBS` whose
# default has been 4 since 2026-03, so it panicked `ExecutorFull` on the fifth
# `register_timer`. It had NEVER passed — it landed broken 2026-08-11 and no
# lane noticed for four days.
#
# Same shape as issue 0319 (a backend suite with no lane, red on main for two
# days) and the issue-0196 rule generally: a gate whose coverage is narrower
# than the thing it is supposed to enforce.
#
# issue 0687 added the second entry: `nros`'s `env` module — the tree's one
# reader of the process environment — is behind a non-default `env` feature for
# the same reason, so its 11 tests were in no lane the hour they were written.
# Same recipe, because it is the same defect class, not a new one.
node-std-tests:
    #!/usr/bin/env bash
    set -e
    # phase-425 W3 added `sim-time` to the same list for the same reason: the
    # `/clock` time source is behind a non-default feature, so `--workspace`
    # builds it with `sim-time` off and its tests are in no lane at all. Note
    # `has_rmw` is false here and the module is `cfg(any(has_rmw, test))`, so it
    # compiles under `cargo test` exactly as its unit tests need.
    # `param-services` joined for W3b: `use_sim_time` is a PARAMETER, so its two
    # tests need the store — and `Executor::declare_parameter`, the seam W3b
    # hooks, lives in a `#[cfg(feature = "param-services")]` impl block.
    cargo test -p nros-node --lib --features std,sim-time,param-services --quiet
    # Issue 1036 — the boot self-report, third entry and the same defect class.
    # `boot_report` is behind a BUILD-TIME cfg set from an env var
    # (`NROS_BOOT_REPORT=1`, Zephyr `CONFIG_NROS_BOOT_REPORT=y`), and nothing in
    # `just` or `.github/` set it, so every test in `boot_report::tests` — the
    # ones issue 1036's resolution calls "unit-tested on the host" — had never
    # run. An env-var cfg is worse than a cargo feature here: `--workspace`
    # cannot even unify it in by accident.
    #
    # TWO INVOCATIONS, and that is not tidiness. The record is a process-global
    # static keeping only the FIRST allocation failure; `boot_report::tests` has
    # a `note_alloc_failed(100, 8)` case, and the link test asserts the record
    # is untouched before it writes. Sharing a binary means sharing a process
    # and the link test reads the other's numbers — measured, it failed with
    # `(100, 8)`. Two cargo invocations are two processes.
    NROS_BOOT_REPORT=1 cargo test -p nros-node --lib --features std --quiet boot_report::tests
    NROS_BOOT_REPORT=1 cargo test -p nros-node --lib --features std --quiet -- \
        --exact executor::tests::arena_exhaustion_reaches_the_boot_record
    # issues 1183/1186 — a THIRD process-global in this crate, found the same
    # way and answered the same way. `executor::backing::TAKEN` is a
    # process-scoped latch, and `spin.rs`'s `default_backing` claims it on every
    # `Executor::open` under `alloc` — so any of the other 365 tests in this
    # binary that opens an executor consumes the one handout, and
    # `the_static_is_handed_out_once_and_only_once` loses its positive control.
    # Measured: green alone, red in the full lane. It is `#[ignore]`d for that
    # reason and runs here, in its own process.
    #
    # The `1 passed` assertion is not belt-and-braces. `--exact` against a
    # renamed test matches NOTHING and cargo exits 0 having run none — the
    # silently-inert shape this file already guards against for
    # `required-features` targets. A skip must be loud or it is not a lane.
    backing_out="$(cargo test -p nros-node --lib \
        --features std,sim-time,param-services --quiet -- --ignored --exact \
        executor::backing::tests::the_static_is_handed_out_once_and_only_once 2>&1)"
    printf '%s\n' "$backing_out"
    case "$backing_out" in
        *"1 passed"*) ;;
        *) printf '%s\n' \
             "node-std-tests: the executor-backing latch test ran 0 tests." \
             "  \`--exact\` matched nothing, so it was renamed or its cfg is off." \
             "  A filter that matches nothing exits 0 — fix the name here." >&2
           exit 1 ;;
    esac
    # `env,std` and NOT `rmw-cffi`: the cffi flavour makes the wall clock an
    # extern the linker wants a platform port for, and this lane links none.
    cargo test -p nros --lib --features env,std --quiet
    # phase-359 W10 follow-up — `std` AND `platform-clock`, the combination where
    # "which wall clock wins" is observable. `platform_port_outranks_std_for_the
    # _wall_clock` DEFINES the port symbol itself, so this lane needs no port
    # linked; without the combination the test is in no lane at all, which is how
    # the `not(std)` gate on `platform_wall_clock` survived unnoticed.
    cargo test -p nros-core --lib --no-default-features --features std,platform-clock --quiet
    echo "non-default-feature unit tests passed (nros-node std + boot-report, nros env, nros-core clock)!"

# Phase 166.R.5 — guard `nros-log` on CAS-less ESP32-C3 /
# riscv32imc so portable-atomic fallback regressions surface in
# the standard check tier.
[private]
nros-log-riscv32:
    @echo "Checking nros-log for riscv32imc..."
    cargo check -p nros-log --target riscv32imc-unknown-none-elf --no-default-features

# Issue 0652 — the `required-features` targets that were in NO lane.
#
# Cargo skips such a target SILENTLY: not reported as filtered, simply never
# built, so it reads as coverage while running nothing. Seven of them sat that
# way, and running them found three real defects that had rotted unobserved —
# a missing force-link anchor (`trigger_conditions` failed
# `Transport(InvalidConfig)`, which reads like a bad locator), an unused import
# fatal under `-D warnings` (`dispatch_strategy` did not compile at all), and a
# pre-phase-258 observable (`component_param` asserted the runtime's Vec where
# the seam moved to the executor's registry).
#
# `loan_e2e` is NOT here: it opens two in-process sessions and needs
# `ZPICO_MAX_SESSIONS=2`, which is a BUILD input — it belongs to
# `test-zpico-multisession`, which already owns that env and its own target dir.
# `custom_transport_loopback` is not here either; it needs a native fixture, so
# it wants a fixture-gated lane rather than this one.
#
# `signal_fd_wake` joined later (issue 0612), from `nros-node`, where it could
# not pass by construction: `NodeWake` is gated `all(alloc, rmw-cffi)` and
# `nros_node::mock` is gated `not(rmw-cffi)`, so the feature set that makes the
# wake path live is the one that removes the only session that crate can open.
# What it needed was a registered backend and a router fixture, which is what
# this crate has. Its first real execution failed outright — the signalfd worker
# asks for an 8192-byte stack against glibc's PTHREAD_STACK_MIN of 16384, so
# `Executor::signal_fd()` had been dead on every Linux host (issue 0667). Same
# thesis as the three above, one capability further out.
[group("test")]
required-features-tests:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    # Issue 0673 — through `_nextest-tolerant`, not a bare `cargo nextest run`.
    # These targets need a real zenoh router, and on a host without
    # `ros-<distro>-rmw-zenoh-cpp` every one of them raises
    # `[SKIPPED:capability]` — which a bare run counts as a FAILURE, making tier 1
    # red for an environment fact and hiding the five steps after it. A real
    # failure here still fails: the tolerance is keyed on the marker, and a
    # build/setup error (nextest exit != 100) is never absorbed.
    just _nextest-tolerant \
        -p nros-tests --no-fail-fast \
        --features trigger-test,component-runtime-test,phase216-substrate,signal-fd-wake-test \
        --test trigger_conditions --test wake_latency \
        --test component_runtime --test tier_filter \
        --test component_dispatch --test component_param \
        --test dispatch_strategy --test signal_fd_wake
    # Issue 0779 — the FILE-CFG half. These are gated by a crate-level
    # `#![cfg(feature = "lending")]` rather than a manifest `required-features`,
    # which is worse: cargo BUILDS them with the feature off, so nextest runs a
    # binary containing zero tests and calls it green. Both files had stopped
    # compiling a phase earlier and nothing noticed.
    #
    # Plain `cargo nextest run`, not `_nextest-tolerant`: these need no router
    # and no capability probe, so a red here is a red. And nextest's default
    # "no tests to run" ERROR is doing real work — it is the signal that would
    # have caught this class, so do not pass `--no-tests`.
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-rmw-cffi --features alloc,lending \
        --test loan_fallback --test loan_native
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-rmw-zenoh --features lending,platform-posix \
        --test lending_traits
    # Issue 0972 — the same class one crate over, and a MODULE rather than a
    # test file. `nros-c`'s `mod node` is inside `rmw_modules!`, whose expansion
    # is `#[cfg(feature = "rmw-cffi")]`; `just test-unit` runs
    # `cargo nextest --workspace`, which activates NO features (its own comment
    # says so). So `node.rs` was never compiled by the lane that runs tests, and
    # `check::workspace-features` compiles it only to CLIPPY it — lint, never
    # execute. Fifty-two unit tests in that file, including the phase-379 W4
    # reference-invalidation set, had therefore never run anywhere.
    #
    # Found by adding a test for 0972 that passed against the BROKEN code: it
    # was not being compiled. A test that cannot fail is worth less than no
    # test, because it reads as coverage.
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-c --features rmw-cffi --lib
    # Issue 0779 — two more of the baseline five, retired. Both were reachable
    # all along; what kept them dark was that the FEATURE NAMED IN THE ISSUE IS
    # NOT ENOUGH. Each file's crate-level cfg is a CONJUNCTION:
    #
    #   custom_transport  #![cfg(all(feature = "platform-posix", feature = "link-custom"))]
    #   registry_race     #![cfg(all(feature = "std",            feature = "bridge-stub"))]
    #   descriptor_seam   #![cfg(feature = "bridge-stub")]
    #
    # So a lane passing only `--features link-custom` builds the target, runs
    # zero tests, and reports green — the very shape 0779 exists to kill, one
    # level in. Pass the whole conjunction or do not bother; the `--features`
    # text here is also what `check-required-features-reachable` reads.
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-rmw-cyclonedds --features std,bridge-stub \
        --test descriptor_seam --test registry_race
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-rmw-zenoh --features platform-posix,link-custom \
        --test custom_transport
    # Issue 0779 — the last three. BASELINE is now empty.
    #
    # `unix-mock` is the third conjunction in this set (`std` too), and the only
    # one whose crate says so: nvidia-ivc carries a `compile_error!` naming the
    # missing feature. The other two failed SILENTLY — a target that builds and
    # runs zero tests — which is why they sat unnoticed while this one would have
    # stopped anyone who tried.
    #
    # `c-stub-test` needed real repairs, not just a flag: its abi_layout_check.c
    # had not compiled since phase-321 W2.e moved the crate (a stale `-I`), and
    # c_stub_platform asserted a call count for a trait that had since shrunk.
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-platform-cffi --features posix-c-port \
        --test c_port_posix --test c_port_posix_critical_section \
        --test c_port_posix_net --test c_port_posix_timer \
        --test c_port_posix_wake --test wake_wrapper
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-platform-cffi --features c-stub-test \
        --test c_stub_platform
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-rmw-cffi --features c-stub-test \
        --test c_stub_transport
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nvidia-ivc --features std,unix-mock \
        --test loopback
    echo "required-features test targets passed!"

# The Cyclone RMW's own CMake/ctest suite (descriptor builder + registry).
#
# This used to be a dedicated `cyclonedds-ci` step on the top-level `ci` line —
# the only RMW with a named slot there. It is not special: it is one backend's
# native test suite, so it belongs with the other per-component lanes here.
#
# Moving it into `check` is also the fix for how issue 0319 survived: that red
# sat on main for two days because `cyclonedds-ci` ran only in `just ci`, and
# `just check` is the recipe people actually run. Warm cost is ~22s.
#
# Best-effort: skips cleanly when the pinned Cyclone submodule is not
# initialised (typical for contributors not touching the DDS backend). The
# `cyclonedds::ci` recipe itself fails hard on real test failures.
[private]
rmw-cyclonedds:
    #!/usr/bin/env bash
    set -e
    if [ ! -f third-party/dds/cyclonedds/CMakeLists.txt ]; then
        echo "Cyclone DDS skip: submodule not initialised"
        echo "  (run \`just cyclonedds setup\` to enable)"
        exit 0
    fi
    just cyclonedds ci

# The per-RMW capability page is derived from the two C vtables + the zenoh
# shim's trait overrides. Prose versions of this table drifted in BOTH
# directions (cyclone services documented unsupported after service.cpp landed;
# zenoh liveliness documented as a no-op while wired) — generated, not
# transcribed, same reasoning as check-pool-inventory.
[private]
rmw-feature-matrix:
    @python3 scripts/gen-rmw-feature-matrix.py --check

rmw-uorb:
    #!/usr/bin/env bash
    set -e
    source scripts/build/build-root.sh
    BD="$(nros_build_dir "$NROS_KIND_UORB_CHECK")"
    # No skip arm: this backend needs nothing but a C++14 compiler. PX4 is
    # opt-in (`NROS_RMW_UORB_LINK_PX4`), and the default OFF build is exactly
    # the surface the RMW ABI changes touch.
    cmake -S packages/rmw/uorb/nros-rmw-uorb -B "$BD" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$BD" --parallel
    ctest --test-dir "$BD" --output-on-failure

# Issue 0787 — host lanes for the two C backends that had none.
#
# `check-rmw-cyclonedds` has compiled that backend on the fast line for
# phases; xrce and uORB were only ever built by tier 2 fixture builds, so five
# phase-376 signature changes crossed their C ABI seam with no compiler
# looking. Both turned out to be buildable on any host all along:
#
#   xrce  vendors micro-XRCE-DDS-Client + micro-CDR as submodules
#   uORB  defaults `NROS_RMW_UORB_LINK_PX4=OFF` precisely so it builds
#         without the PX4 SDK
#
# So the SDKs were never the blocker. Nobody had written the recipe.
#
# Each skips LOUDLY (counted by `scripts/build/check-skip.sh`) when its
# prerequisite is absent, so "not built here" can never read as "builds".
rmw-xrce:
    #!/usr/bin/env bash
    set -e
    source scripts/build/build-root.sh
    if [ ! -f packages/rmw/xrce/xrce-sys/micro-xrce-dds-client/CMakeLists.txt ]; then
        source scripts/build/check-skip.sh
        nros_check_skip rmw-xrce \
            "micro-XRCE-DDS-Client submodule not initialised (git submodule update --init packages/rmw/xrce/xrce-sys/micro-xrce-dds-client)"
        exit 0
    fi
    BD="$(nros_build_dir "$NROS_KIND_XRCE_CHECK")"
    cmake -S packages/rmw/xrce/nros-rmw-xrce -B "$BD" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$BD" --parallel
    ctest --test-dir "$BD" --output-on-failure

# Issue 0260 / phase-356 W3 — the sched-dim ACCEPT arms no build otherwise
# reaches. Syntax-only: it type-checks our `vTaskCoreAffinitySet` /
# `sched_setaffinity` / `tx_thread_smp_core_exclude` call sites against the
# vendored headers' REAL declarations, using a synthetic SMP config, because
# those arms sit behind macros no image defines — so an API misuse in them is
# invisible today.
#
# A green here is NOT an arm being observed ACCEPTING at runtime: that needs a
# real SMP board and is phase-356's separate, larger item.
#
# All THREE arms are covered — freertos (`vTaskCoreAffinitySet`), nuttx
# (`pthread_setaffinity_np`) and threadx (`tx_thread_smp_core_exclude`) — each
# against its own vendored headers. This comment previously said nuttx and
# threadx "are not yet" covered and pointed at the script's output for the
# truth; the script has covered all three since it landed, so the comment was
# describing a state that never shipped. Read the script, not this line, when
# they disagree: it prints one section per arm.
sched-dim-arms:
    @bash scripts/check-sched-dim-arms-compile.sh

# Scheduling wiring matrix — parsed from sched_caps_for() (the function the
# realizer executes) + SchedClass dispatch sites in spin.rs. Prose versions of
# this drifted three ways at once (scheduling-models said EDF unused while
# spin.rs dispatches it). Same reasoning as check-rmw-feature-matrix.
[private]
sched-matrix:
    @python3 scripts/gen-sched-matrix.py --check

# Compile-time SOURCE/precondition gates that ship as `nros-tests` test binaries
# (header-ABI mirror, two-libc precedence, zephyr prj.conf requirements). These
# are the `cargo test -p nros-tests --test …` steps `check.yml` runs inline;
# wrapped here so `just check` runs the identical set. Compiles nros-tests, so
# slower than the buildless gates but still a static/precondition check.
[private]
source-gates:
    #!/usr/bin/env bash
    set -e
    # These tests assert a BUILD-STAGE stamp (`.compile-ok`), so this gate has
    # to PRODUCE it. Without this line the lane silently depends on
    # `build-test-fixtures`, which contradicts ci-l1's documented "NO FIXTURES"
    # contract and made the required CI check permanently red the moment it
    # became required: `BuildFailed("Test fixture binary not prebuilt")` on a
    # runner that builds no fixtures. It costs ~13 s — the snippets are small
    # `cargo check` template crates, which is the whole point of issue 0034's
    # design. The push lane never noticed because it runs `check-fast` only.
    # ONLY the lane this gate asserts. `platform_header_compile` consumes the
    # nine `platform_hdr_*` stamps, and ALL NINE are `cxx-syntax` records —
    # verified against `examples/fixtures.toml`, not inferred from the ids. An
    # earlier version of this line said `cargo-check` on exactly that inference
    # and excluded the only lane the test needs; it passed locally because the
    # stamps already existed from previous full builds, and failed in CI where
    # they do not.
    #
    # Building every lane instead is what this replaced: that also built
    # `freertos_firmware`, a `cargo-build` row needing the FreeRTOS KERNEL
    # SUBMODULE which CI does not provision.
    NROS_COMPILE_CHECK_LANES=cxx-syntax bash scripts/build/compile-check-fixtures.sh
    # Issue 0673 — through `_nextest-tolerant`, not bare `cargo test`.
    #
    # `cross_libc_precedence_gate` already does the right thing: it calls
    # `nros_tests::skip!` when `arm-none-eabi-g++` has no usable libstdc++, with
    # a comment saying "Skip rather than false-fail". But `skip!` PANICS with a
    # `[SKIPPED]` marker, and a bare runner counts that as a FAILURE — only the
    # junit rewrite in `_nextest-tolerant` turns it back into a skip. So a
    # correct skip was reported as a red, and it made the merge group fail on an
    # ENVIRONMENT FACT rather than on anything in the change.
    #
    # That fact is not incidental either: this gate lives in `check-build` ->
    # `ci-l1`, which promises "no SDK, no QEMU and no cross toolchain". A C++
    # capable newlib cross is exactly a cross toolchain, so the skip is the lane
    # contract working, and swallowing it as a failure was the bug.
    #
    # A real failure still fails: the tolerance keys on the marker, and a
    # build/setup error (nextest exit != 100) is never absorbed.
    just _nextest-tolerant \
        -p nros-tests --no-fail-fast \
        --test platform_header_compile \
        --test cross_libc_precedence_gate \
        --test zephyr_prjconf_requirements

# Link-determinism gate (RFC-0042 D3) — build the host staticlib pair, then assert
# the `--allow-multiple-definition` masked dups are ONLY the shared Rust dep
# closure (no app ODR violation). The `check.yml` staticlib step (SSoT).
[private]
staticlib-symbols:
    #!/usr/bin/env bash
    set -e
    bash scripts/build/link-determinism-fixture.sh
    # Skip-capable: routed through the tolerant runner so an unmet
    # precondition reads as a SKIP rather than a red naming a test the
    # change never touched. Gated by `check-skippable-tests-tolerant`.
    just _nextest-tolerant -p nros-tests --no-fail-fast --test staticlib_duplicate_symbols

# Compile the TEST targets. Buildless gates never touch `#[cfg(test)]` code, and
# neither does `check-workspace`'s clippy (no `--all-targets`), so a field added
# to a shared struct can leave every test initializer broken while `check-fast`
# stays green. That happened twice in one session — `TierRtosSpec` gained four
# fields (phase-330 W1.a) and `ComponentConfig` gained `class` (issue 0392 B) —
# and both landed green with seven initializers unbuildable.
#
# `cargo check`, not `cargo test`: this is about COMPILING the test targets;
# running them belongs to `test-all`. Warm cost ~13s (root) + ~19s (CLI); the
# first run on a cold target dir is minutes, like any other compile here.
#
# Issue 0466 — this is a check-BUILD gate, and used to sit in `check-fast`.
# Compiling the workspace needs the `-sys` SOURCE submodules, which the push
# lane deliberately does not provision (gate.yml gates that on
# `event_name != 'push'`), so on every push it died:
#
#     error: failed to run custom build command for `zpico-sys`
#
# `just` stops at the first failed dependency, and 25 gates sat behind this one
# — every fmt gate, the FFI/sizes mirrors, doc-refs, string conventions. So its
# real per-push coverage was ZERO while it masked all of them. Four source-level
# reds reached main during exactly that window.
#
# Moving it costs no coverage anyone actually runs: `just check` is
# `check-fast` + `check-build`, so a local `just check` / `just ci` runs the
# identical set, and CI still runs it on PR + nightly. What changes is that the
# push lane can now finish — measured on a pristine, source-free, CLI-free
# checkout: `just check fast` green in 23s, two graceful skips, no failures.
#
# The reason it was written (a struct field breaking every test initializer
# while buildless gates stay green — twice in one session) is untouched: the
# gate still runs, in the tier that can actually compile.
[private]
test-targets:
    #!/usr/bin/env bash
    set -euo pipefail
    # clippy, not `cargo check`: it COMPILES the same targets, so linting here
    # costs nothing extra over the compile this gate already needed, and it
    # closes the second half of the same blind spot — a lint in code you just
    # wrote could only be seen by the build tier. `-D warnings` on both, which
    # the host clippy did not previously enforce (the embedded one always did).
    cargo clippy --quiet --workspace --all-targets --no-default-features \
        {{HOST_UNCHECKABLE}} -- -D warnings
    # The CLI is its own workspace (own manifest + lock) and its exclusions have
    # a single home in `check-cli-clippy` — call it rather than restate them.
    just check cli-clippy
    echo "clippy + test targets clean (root + cli)."

# Check workspace: formatting and clippy (no_std, native)
# nros-c/nros-cpp/standalone RMW staticlib wrappers excluded from no_std
# check: staticlib/cdylib requires a platform-specific panic/runtime setup.
# nros-rmw-{zenoh,dds,xrce}-cffi excluded because their `*Rmw` type
# imports are platform-feature-gated by the underlying impl crate
# (e.g. `ZenohRmw` only exists when one of `platform-{posix,zephyr,…}`
# is on). `--no-default-features --workspace` strips every feature
# from every member at once, so the cffi shim's `RustBackendAdapter<R>`
# can't resolve its type parameter. Real consumers always specify
# a platform; the per-feature combinations are covered by
# `check-workspace-features` further down.
# Kept as a name others may invoke. Both of its former jobs moved to the FAST
# tier, where they catch a mistake before a build tier nobody runs per task:
# host clippy -> `check-test-targets` (now `--all-targets`, `-D warnings`),
# `fmt --check` -> `check-workspace-fmt`.
[private]
workspace: workspace-fmt
    @echo "check-workspace: fmt + clippy now run in the fast tier."

# Run the host + embedded workspace clippy CONCURRENTLY. They share no
# target-dir (host = `target/`, embedded = `target-embedded/`), so cargo's
# build lock doesn't serialize them; sccache (global RUSTC_WRAPPER) shares the
# dep cache across both. The `NROS_BUILD_JOBS` budget is split in half to each
# via `CARGO_BUILD_JOBS` so total parallelism stays bounded (same knob the
# build recipes thread — no hardcoded `-j`). Both still run standalone.
[private]
workspace-all:
    #!/usr/bin/env bash
    set -uo pipefail
    jobs="${NROS_BUILD_JOBS:-$(nproc 2>/dev/null || echo 8)}"
    half=$(( jobs / 2 )); [ "$half" -lt 1 ] && half=1
    CARGO_BUILD_JOBS="$half" just check workspace &
    host=$!
    CARGO_BUILD_JOBS="$half" just check workspace-embedded &
    emb=$!
    rc=0
    wait "$host" || rc=1
    wait "$emb" || rc=1
    exit "$rc"

# Check workspace for embedded target (Cortex-M4F)
# Excludes zpico-sys: requires native system headers for CMake build
# Excludes nros-tests: requires std (test framework dependencies)
# Excludes nros-c/nros-cpp/standalone RMW staticlib wrappers:
# staticlib/cdylib requires a platform-specific panic/runtime setup.
#
# Builds into a dedicated `target-embedded/` (CARGO_TARGET_DIR) so the
# thumbv7 artifacts never share cargo's per-target-dir build lock with the
# host clippy — letting `check-workspace-all` run the two concurrently.
[private]
workspace-embedded:
    #!/usr/bin/env bash
    set -euo pipefail
    # issue 0287 — the exclude list is DERIVED, not hand-written.
    #
    # cargo unifies features across every workspace member when this builds the
    # workspace for a thumb target, regardless of what firmware can reach. One
    # host-only member therefore turns `std` on for everything and the lane dies
    # in an unrelated crate (`can't find crate for `std`` in nros-serdes, which
    # is merely the first no_std crate cargo reaches). The 20-line hand list
    # that used to live here carried no reasons and nothing tied an entry to its
    # crate; each crate now declares `[package.metadata.nros] host-only = true`
    # with a reason, and the script below derives the flags.
    echo "Checking workspace for embedded target..."
    source scripts/build/cargo.sh
    excludes="$(bash scripts/build/host-only-members.sh)"
    # issue 0400: box-aware target dir — a relative `target-embedded` overrides
    # the ROS distrobox's CARGO_TARGET_DIR redirect and shares build-script
    # binaries with the host (GLIBC-mismatch crash + a misleading host-only hint).
    emb_target="$(nros_scoped_target_dir embedded)"
    if ! CARGO_TARGET_DIR="$emb_target" cargo clippy --quiet --workspace \
            --no-default-features --target thumbv7em-none-eabihf \
            $excludes -- -D warnings; then
        echo "" >&2
        echo "[hint] If this failed with \`can't find crate for \\\`std\\\`\` in a crate you did" >&2
        echo "       not touch, a NEW host-only member is leaking \`std\` through cargo" >&2
        echo "       feature unification (issue 0287). The named crate is a victim, not" >&2
        echo "       the cause. Declare the new crate host-only:" >&2
        echo "" >&2
        echo "         [package.metadata.nros]" >&2
        echo "         host-only = true" >&2
        echo "         host-only-reason = \"...\"" >&2
        exit 1
    fi

# Check workspace with various feature combinations
[private]
workspace-features:
    @echo "Checking feature combinations..."
    # Phase 128.C.3 — `nros/rmw-zenoh-cffi` feature deleted; the
    # umbrella now only carries `rmw-cffi`. Backend selection is
    # done by adding the matching `nros-rmw-<name>` dep.
    # Phase 248 C5c — the `nros` umbrella dropped its `platform-*` features
    # (platform now comes from `nros-platform`/board/RMW crates, not the umbrella),
    # so the combo lints `nros` without `platform-posix` (nros-c/nros-cpp still
    # carry it — see the nros-c combo below).
    @echo "  - nros: cffi + humble"
    cargo clippy --quiet -p nros --no-default-features --features "std,rmw-cffi,ros-humble"
    @echo "  - nros: cffi + iron"
    cargo clippy --quiet -p nros --no-default-features --features "std,rmw-cffi,ros-iron"
    @echo "  - nros-c: zenoh-cffi + posix + humble"
    cargo clippy --quiet -p nros-c --no-default-features --features "{{C_API_SHIPPED_FEATURES}}"
    @echo "  - nros: cffi (no_std)"
    cargo clippy --quiet -p nros --no-default-features --features "rmw-cffi"
    @echo "  - transport: sync-critical-section"
    cargo clippy --quiet -p nros-rmw --no-default-features --features "sync-critical-section" --target thumbv7em-none-eabihf
    @echo "  - nros-rmw (std)"
    cargo clippy --quiet -p nros-rmw --features "std"
    # Phase 214.G.2 — workspace-wide no-default-features smoke. Catches
    # the feature-unification regression class (Track F) at `just check`
    # time rather than waiting for `just test-unit`. `--no-run` compiles
    # all tests without executing — keeps the gate fast (no test runs)
    # while still exercising the trans-feature dep graph.
    #
    # `--exclude nros-c`: pre-existing latent test-compile bug in
    # `packages/api/nros-c/src/cdr.rs:565` references `std::ffi::CStr`
    # but the lib is no_std-by-default. Filed for separate fix; gate
    # remains valid for every other crate. Remove the exclude once the
    # nros-c lib-test gating lands.
    @echo "  - workspace: test-compile --no-default-features"
    cargo test --no-run --workspace --exclude nros-c --no-default-features --quiet
    # issue 0619 — the exclude above is what let nros-c's lib test rot unnoticed:
    # excluded from the ONLY test-compile gate, it was covered by nothing at all.
    # The exclude is about --no-default-features specifically (no panic handler,
    # no platform port), so the fix is not to drop it but to give nros-c its own
    # line with DEFAULT features, where it does link. Issue-0196 rule: a gate
    # must cover the class it claims to.
    @echo "  - nros-c: test-compile (default features)"
    cargo test --no-run -p nros-c --quiet
    # issue 0814 — the loan/borrow surface, which had NOT COMPILED since issue
    # 0812 retyped the backend token (`*mut c_void` -> `*mut rmw_loan_token_t`)
    # and left these three FFI call sites behind. Seven errors, for months,
    # caught by nothing: no lane enables `lending`, so the crate's whole
    # zero-copy API was never built — while the committed cbindgen header went
    # on declaring all five symbols unconditionally.
    #
    # Both rows, because the two differ in what they even contain: without
    # `alloc` the receive half is `cfg`-ed out (it boxes a `RecvView` per
    # borrow — 0812's own open item), so the no-`alloc` row is the publish
    # surface and the `std` row is the whole thing. Checking one would leave
    # the other exactly as dark as before.
    #
    # The embedded-target twin of this is the lending slice in
    # `check::no-std`; `nros-c` cannot go there because its build script needs
    # a cross C toolchain.
    @echo "  - nros-c: lending (publish surface, no alloc)"
    # `panic-platform` is nros-c's own `default` and is named here for the
    # reason its manifest gives: `--no-default-features` builds this crate's
    # staticlib, which is a FINAL artifact and needs a `#[panic_handler]`.
    cargo clippy --quiet -p nros-c --no-default-features --features "panic-platform,rmw-cffi,lending" -- -D warnings
    @echo "  - nros-c: lending + std (publish AND borrow surface)"
    cargo clippy --quiet -p nros-c --no-default-features --features "std,rmw-cffi,lending,platform-posix" -- -D warnings
    @echo "All feature checks passed!"

# Check zenoh transport
[private]
zenoh:
    cargo clippy --quiet -p nros-rmw --features std

# Phase 134.5 — verify the in-tree zenoh staticlib's internal symbol
# parity. For every defined `_z_f_link_*_<transport>` wrapper, the
# matching `_z_*_<transport>` impl must also be defined. Pre-Phase-134
# the POSIX CMake path shipped wrappers without multicast impls and
# every C/C++ native link broke. Run after
# `cargo build -p nros-rmw-zenoh-staticlib --release`.
[group("debug")]
zenoh-archive:
    # profile-literal-ok: symbol fixture: the archive built by build-zenoh-posix-fixture
    @bash scripts/check-zenoh-archive-symbols.sh target/release/libnros_rmw_zenoh_staticlib.a
