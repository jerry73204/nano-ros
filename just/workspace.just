set working-directory := '..'

NIGHTLY := `awk '/^channel/ {gsub(/"/, "", $3); print $3; exit}' tools/rust-toolchain.toml`

# Pinned Corrosion version. Corrosion is the cmake-side Rust bridge used
# by mixed-language workspaces (Phase 212 — `corrosion_import_crate` in
# `cmake/NanoRosThreadxSystemCodegen.cmake` etc.). Installed to
# `~/.nros/sdk/corrosion/` so it can be discovered by `find_package(
# Corrosion)` without polluting the host system; the H.4 and 212.D
# Corrosion tests look it up via `cmake --find-package` with
# `CMAKE_PREFIX_PATH=$HOME/.nros/sdk/corrosion`. Phase 212.H.4.
# issue 0493 — must stay in step with `nros-sdk-index.toml`'s [tool.corrosion]
# ref; the two provisioning paths land at ONE prefix and a mismatch means the
# stamp check reinstalls on every invocation.
CORROSION_VERSION := "v0.6.1"

# Pinned play_launch_parser SHA. Used by the Phase 212.L.6 launch-graph
# resolver in `nros plan` / `nros codegen-system` (synth-from-Component
# + multi-launch precedence). No upstream release tags exist on
# `jerry73204/play_launch_parser` — pinned to a SHA on `main` (same
# convention `[tool.cyclonedds]` uses); bumping is the signal to re-run
# the L.6 gate tests `phase212_l6_launch_synth::*`. Kept in lockstep
# with `nros-sdk-index.toml::[tool.play_launch_parser]` so a
# `just workspace install-play-launch-parser` and a
# `nros setup --tool play_launch_parser` land the same revision.
# The parser now comes from the `play_launch` SUBMODULE, not from the archived
# standalone repo. `jerry73204/play_launch_parser` is ARCHIVED and superseded by
# `NEWSLabNTU/play_launch`, which vendors the same crate at
# `src/ros-launch-resolve/parser/crates/play_launch_parser` and is already a
# submodule here — so the binary and the LIBRARY `nros-launch-resolve` path-deps
# now come from ONE pin instead of two that could drift silently (the class of
# issues 0609 and 0507).
#
# The version is the SUBMODULE COMMIT, so the stamp invalidates exactly when the
# pin moves.

# Workspace module — host-agnostic dev environment setup.
#
# "workspace" here means the developer's dev workspace (host OS packages,
# Rust toolchains, cargo tools) — NOT the Cargo `[workspace]` term-of-art.
# These recipes configure the host machine to be able to do anything with
# this repo. Platform-specific setup lives in the per-platform modules
# (just/esp32.just, just/nuttx.just, etc.).

default:
    @just --list --justfile {{source_file()}}

# Install all workspace prerequisites (apt + rust + cargo tools).
#
# Rust toolchain channels and components are driven by the
# `rust-toolchain.toml` files checked into the repo (root, tools/,
# examples/**/, packages/verification/nros-verification/). No bare
# `stable`/`nightly` channels are installed here — every version is
# pinned in a TOML file and installed by `rust-pinned-toolchains`.
[group("setup")]
setup:
    # phase-327 W2 (issue 0368 F1): every sudo-less installer runs FIRST; the
    # system-package step is LAST and only PRINTS (a missing OS package must
    # never abort the sudo-less remainder — one sudo failure here used to
    # cascade into zephyr/esp32/px4).
    # make + ninja: store tools, not bespoke recipes (see the note where those
    # recipes used to be). `--tool` so a `just workspace setup` still provisions
    # the jobserver pair without the user knowing they moved.
    @nros setup --tool ninja
    @nros setup --tool make
    @just workspace install-corrosion
    @just workspace install-play-launch-parser
    @just workspace rust-pinned-toolchains
    @just workspace rust-targets
    @just workspace cargo-tools
    @just workspace build-codegen
    @just workspace install-sccache
    @just workspace apt-packages

# Verify the host `nros` build tool resolves (the message/IDL codegen used by
# `nros generate-*`, the cmake `nano_ros_generate_interfaces()` path, and the
# NuttX external-app FFI staging). Phase 218 — `nros` is built in-tree at
# `packages/cli/target/release/nros` (the `cargo-tools` step above runs
# `just setup-cli`); this is the setup-stage presence check. Build recipes
# keep `nros_cargo_ensure_codegen_c` as a net.
[group("setup")]
build-codegen:
    #!/usr/bin/env bash
    set -euo pipefail
    source scripts/build/cargo.sh
    nros_cargo_ensure_codegen_c
    echo "[codegen] nros tool: $(nros_cargo_codegen_c_bin)"

# make + ninja are NOT installed here any more.
#
# They were two bespoke recipes filling `third-party/{make,ninja}/`, each with
# its own version variable — a second pin beside `nros-sdk-index.toml`'s, free
# to drift from it. They are ordinary store tools now:
#
#     nros setup --tool make      # 4.4.1, source tarball (upstream ships no binary)
#     nros setup --tool ninja     # 1.13.2, upstream prebuilt zip
#
# Policy: prebuilt when upstream publishes one, source only as the fallback.
# `scripts/sdk-path-tools.txt` puts both on PATH; consumers resolve the exact
# path with `nros sdk-path <tool>`, constructed from the pin (issue 0625).

# Install Corrosion (cmake ↔ Rust bridge) into `~/.nros/sdk/corrosion/`.
# Mixed-language workspaces (Phase 212) use `find_package(Corrosion)` +
# `corrosion_import_crate()` to pull Rust component staticlibs into a
# cmake build. Lives under `~/.nros/sdk/` (not third-party/) because it's
# a per-developer install prefix consumed via `CMAKE_PREFIX_PATH`, not a
# repo-relative tool. Idempotent: skips when the pinned tag is already
# installed.
[group("setup")]
install-corrosion:
    #!/usr/bin/env bash
    set -euo pipefail
    # phase-422 W1 — FORWARDER. This recipe used to be a second PRODUCER: its
    # own download, its own `.installed-version` stamp, writing the same
    # versioned prefix `nros setup --tool corrosion` writes.
    #
    # Two producers for one prefix is issue 0500's failure mode: the store
    # accumulates, prefixes resolve newest-first, and BOTH paths print success —
    # so a stale Corrosion shadowed the pin that had just been installed and
    # `mixed` could not link. A second producer is not a convenience, it is a
    # second answer to "which version is installed?".
    #
    # The index is the pin (`[tool.corrosion] upstream`), and the CLI stamps the
    # same upstream tag this recipe used to, so the doctor's comparison is
    # unchanged.
    nros setup --tool corrosion

[group("setup")]
install-play-launch-parser:
    #!/usr/bin/env bash
    set -euo pipefail
    # phase-422 W1 — FORWARDER, for the reason on `install-corrosion` above.
    # `[tool.play_launch_parser]` in the index is the pin.
    nros setup --tool play_launch_parser

[group("setup")]
install-sccache:
    #!/usr/bin/env bash
    set -uo pipefail
    if command -v sccache >/dev/null 2>&1; then
        echo "[sccache] already on PATH ($(command -v sccache)) — skipping"
        exit 0
    fi
    source scripts/build/cargo.sh
    nros_bin="$(nros_cli_bin 2>/dev/null || true)"
    if [ -z "${nros_bin:-}" ]; then
        echo "[sccache] nros CLI not built yet — run 'just setup-cli', then 'just workspace install-sccache'"
        exit 0
    fi
    echo "[sccache] provisioning via 'nros setup --tool sccache' (source build, one-time)..."
    "$nros_bin" setup --tool sccache \
        || echo "[sccache] provisioning failed (optional cache) — install manually for faster builds"

# Diagnose workspace install status (read-only).
# Returns exit 1 if anything is missing.
[group("setup")]
doctor:
    #!/usr/bin/env bash
    set +e
    fail=0
    check() {
        if "$@" &>/dev/null; then return 0; else return 1; fi
    }
    source scripts/build/cargo.sh
    nros_bin="$(nros_cli_bin 2>/dev/null || true)"

    # --- system packages (phase-327 W3: derived from [system.*], never a
    # hand-written list — the drift class issue 0368 measured) ---
    if [ -x "$nros_bin" ]; then
        if sys_out="$("$nros_bin" setup --system --check 2>&1)"; then
            echo "  [OK] system packages ($(echo "$sys_out" | head -1 | sed 's/^nros setup --system --check: //'))"
        else
            echo "$sys_out" | sed 's/^/  /'
            fail=1
        fi
        # Issue 0929 — the third question, after "is the package here?" and
        # "is the declaration complete?": does the installed tool actually RUN.
        # Only the BROKEN lines are surfaced; doctor does not walk `[tool.*]`
        # itself, and reprinting the whole class here would bury its own
        # findings. 0.26 s, so it is not worth a flag.
        if broken_out="$("$nros_bin" setup --check 2>&1 | grep -A2 '^  \[BROKEN\]' || true)"; then
            if [ -n "$broken_out" ]; then
                echo "$broken_out" | sed 's/^/  /'
                fail=1
            else
                echo "  [OK] installed tools run"
            fi
        fi

        # Issue 0926 — the check above asks "is every DECLARED package here?".
        # This asks the other half: "is the declaration COMPLETE?", by
        # re-measuring the provisioned dists. Both are needed, and only the
        # second would have caught `[tool.qemu] system = ["libslirp"]` being
        # correct and 19 sonames short. Skips itself when there is no store.
        if dist_out="$(python3 scripts/check-dist-runtime-deps.py 2>&1)"; then
            echo "  [OK] dist runtime deps ($(echo "$dist_out" | head -1 | sed 's/^check-dist-runtime-deps[: ]*//'))"
        else
            echo "$dist_out" | sed 's/^/  /'
            fail=1
        fi
    else
        echo "  [SKIP] system packages (nros CLI not built — run: just setup-cli)"
    fi

    # --- pinned ninja (>=1.13 for the GNU-jobserver client, Phase 176) ---
    # Store tool now; path CONSTRUCTED from the index pin (issue 0625), so this
    # cannot drift from what `nros setup --tool ninja` installed. The version is
    # the index's — there is no second pin here to disagree with it.
    ninja_bin="$(nros sdk-path ninja 2>/dev/null)/bin/ninja"
    if [ -x "$ninja_bin" ]; then
        echo "  [OK] ninja $("$ninja_bin" --version) (SDK store; jobserver-capable)"
    else
        echo "  [MISSING] ninja (run: nros setup --tool ninja)"
        fail=1
    fi

    # --- Corrosion (cmake ↔ Rust bridge, Phase 212) ---
    # phase-365 — ask for the prefix; do not hand-join it. W5 moved
    # `install-corrosion` to the VERSIONED destination and this reader kept
    # looking in the flat one, so the status line reported MISSING for a
    # correctly installed tool. Two spellings of one join, drifting the moment
    # one side moved — which is the whole reason the constructor exists.
    corrosion_prefix="$(nros sdk-path corrosion 2>/dev/null || true)"
    # Presence is "the PINNED prefix holds a usable Corrosion", not "this recipe
    # wrote its stamp". `nros setup --tool corrosion` installs the same tool and
    # leaves `.nros-provenance` instead of `.installed-version`, so the
    # stamp-only test reported MISSING for a perfectly good install — a wart
    # that predates phase-365 and that moving to the versioned prefix exposed.
    if [ -n "$corrosion_prefix" ] && \
       [ -f "$corrosion_prefix/lib/cmake/Corrosion/CorrosionConfig.cmake" ]; then
        echo "  [OK] corrosion {{CORROSION_VERSION}} ($corrosion_prefix)"
    else
        echo "  [MISSING] corrosion {{CORROSION_VERSION}} (run: just workspace install-corrosion)"
        fail=1
    fi

    # --- nros-launch-resolve (RFC-0060 layer 2 — the CLI's launch resolver) ---
    # Issue 0336: `nros sync` HARD-REQUIRES this binary (cmd/ws.rs bails
    # without it), and `just setup` builds it unconditionally — but doctor never
    # checked it, so doctor could report a green tree that cannot sync a
    # workspace. Invoked by ABSOLUTE path, never via $PATH (issue 0285: an
    # unrelated ROS 2 `play_launch` won the PATH race and broke every platform's
    # fixture build), so this checks the file, not `command -v`.
    nlr_bin="{{justfile_directory()}}/packages/cli/nros-launch-resolve/target/release/nros-launch-resolve"
    nlr_src="{{justfile_directory()}}/packages/cli/third-party/play_launch/src/ros-launch-resolve"
    if [ -x "$nlr_bin" ]; then
        # issue 0409 — existence is NOT enough. A resolver older than its sources
        # produces models that are missing DATA rather than failing: one built
        # before rlm v0.1.1 silently drops every `[[component]].params` /
        # `params_files` projection, and 22 models in `features/` alone lost
        # their params that way with no error anywhere. Doctor reporting [OK] for
        # a stale binary is how that stayed invisible. Same probe the recipe
        # uses: the sources live in the submodule, so `git -C` it — from the
        # superproject the index holds only the gitlink and the walk matches
        # nothing, which would make every pin bump look current.
        nlr_stale=""
        if [ -d "$nlr_src" ]; then
            while IFS= read -r f; do
                [ -n "$f" ] || continue
                if [ "$nlr_src/$f" -nt "$nlr_bin" ]; then nlr_stale="$f"; break; fi
            done < <(git -C "$nlr_src" ls-files 2>/dev/null || true)
        fi
        if [ -n "$nlr_stale" ]; then
            echo "  [STALE] nros-launch-resolve is older than its sources ($nlr_stale)"
            echo "          a stale resolver drops params from generated models SILENTLY (issue 0409)"
            echo "          run: just setup-launch-resolve"
            fail=1
        else
            echo "  [OK] nros-launch-resolve (packages/cli/nros-launch-resolve/target/release)"
        fi
    elif [ ! -f "{{justfile_directory()}}/packages/cli/third-party/play_launch/src/ros-launch-resolve/resolve/Cargo.toml" ]; then
        echo "  [MISSING] play_launch submodule — nros sync cannot resolve SystemModels"
        echo "            run: git submodule update --init packages/cli/third-party/play_launch"
        fail=1
    else
        echo "  [MISSING] nros-launch-resolve (run: just setup-launch-resolve)"
        fail=1
    fi

    # --- play_launch_parser (TEST-tier prereq only) ---
    # Issue 0336: this is NOT a CLI prereq. The RFC-0060 CLI ships its own
    # resolver (checked above) and never shells out to play_launch_parser — the
    # `nros plan` claim below was true pre-RFC-0060 and is not any more. It stays
    # a FAIL because the launch_synth / self_bringup / orchestration_includes
    # gate tests probe it on PATH; if those move to an absolute-path lookup this
    # whole block can become a warning.
    # Phase 220.J.2 — FAIL (not warn) on either missing install OR
    # binary-not-on-PATH; install-stamp alone is insufficient — agents
    # routinely hit the gap when a fresh worktree forgets `source
    # ./activate.sh` (the script that PATH-prepends
    # `~/.nros/sdk/play_launch_parser/bin/`). Tailored hints distinguish
    # the two failure modes.
    # Same constructor the installer uses, so doctor cannot look somewhere the
    # install never wrote (phase-365 W2; the flat layout resolves since 0628).
    # Falls back to the hand-join only when `nros` is not on PATH — which is one
    # of the two failure modes this very check exists to report, so it must not
    # become a hard error here.
    plp_prefix="$(nros sdk-path play_launch_parser 2>/dev/null || true)"
    if [ -z "$plp_prefix" ]; then
        plp_prefix="${NROS_HOME:-$HOME/.nros}/sdk/play_launch_parser"
    fi
    plp_stamp="$plp_prefix/.installed-version"
    plp_bin="$plp_prefix/bin/play_launch_parser"
    plp_install_ok=0
    # phase-422 W1 — ask the INDEX, not a second constant. This compared the
    # stamp to `PLAY_LAUNCH_PARSER_VERSION` (a git SHA) while the index pins
    # `0.1.0-nros1`; the two producers never agreed, so retiring the recipe's
    # own installer made the doctor report a correctly installed tool MISSING.
    # One pin, and the CLI owns the comparison.
    if nros setup --tool play_launch_parser --check >/dev/null 2>&1 && [ -x "$plp_bin" ]; then
        plp_install_ok=1
    fi
    if [ $plp_install_ok -eq 1 ] && command -v play_launch_parser >/dev/null 2>&1; then
        echo "  [OK] play_launch_parser (index pin) (~/.nros/sdk/play_launch_parser)"
    elif [ $plp_install_ok -eq 1 ]; then
        echo "  [PATH] play_launch_parser installed at ~/.nros/sdk/play_launch_parser/bin/ but not on PATH —"
        echo "         run: source ./activate.sh  (or direnv allow / source ./activate.fish)"
        echo "         (Phase 220.J — agent driver scripts MUST source activate.sh before any just <plat> build)"
        fail=1
    else
        echo "  [MISSING] play_launch_parser (run: just workspace install-play-launch-parser)"
        fail=1
    fi

    # --- pinned make (>=4.4 for the fifo jobserver, Phase 176) ---
    make_bin="$(nros sdk-path make 2>/dev/null)/bin/make"
    if [ -x "$make_bin" ]; then
        echo "  [OK] make $("$make_bin" --version | head -1 | awk '{print $NF}') (SDK store; fifo jobserver)"
    else
        echo "  [MISSING] make (run: nros setup --tool make)"
        fail=1
    fi

    # --- rust-pinned-toolchains ---
    # Every `rust-toolchain.toml` checked into the repo must have its
    # channel installed, together with the components it declares.
    # This is the single source of truth for both versions and components
    # — no separate `rustup toolchain install` / `rustup component add`
    # steps.
    shopt -s globstar nullglob 2>/dev/null
    pinned_missing=()
    comp_missing=()
    rustup_timeout=()
    # Phase 208.D.6 — wrap every `rustup` invocation in `timeout 5s` +
    # disable rustup self-update so a stale rustup or a flaky/slow
    # rustup-dist mirror can't wedge `just doctor` (the audit blocker
    # P13). If a single call exceeds 5 s, doctor reports
    # `rust-pinned-toolchains: rustup unresponsive` and moves on
    # instead of hanging.
    export RUSTUP_NO_SELF_UPDATE=1
    has_component() {
        timeout 5s rustup component list --installed --toolchain "$2" 2>/dev/null \
            | grep -qE "^$1(-[^[:space:]]+)?$"
    }
    for f in $(just workspace _pinned-toolchain-files); do
        channel=$(awk '/^channel *=/ { gsub(/["]/, "", $3); print $3; exit }' "$f")
        [ -z "$channel" ] && continue
        list_out="$(timeout 5s rustup toolchain list 2>/dev/null)"
        rustup_rc=$?
        if [ $rustup_rc -eq 124 ]; then
            rustup_timeout+=("$channel ($f)")
            continue
        fi
        if ! echo "$list_out" | grep -q "^$channel"; then
            pinned_missing+=("$channel ($f)")
            continue
        fi
        components=$(awk '/^components *=/ { sub(/^[^=]*= */, ""); gsub(/[][",]/, " "); print; exit }' "$f")
        for c in $components; do
            [ -z "$c" ] && continue
            has_component "$c" "$channel" || comp_missing+=("$channel/$c")
        done
    done
    if [ ${#pinned_missing[@]} -eq 0 ] && [ ${#comp_missing[@]} -eq 0 ] && [ ${#rustup_timeout[@]} -eq 0 ]; then
        echo "  [OK] rust-pinned-toolchains"
    else
        if [ ${#pinned_missing[@]} -gt 0 ]; then
            echo "  [MISSING] rust-pinned-toolchains channels: ${pinned_missing[*]}"
        fi
        if [ ${#comp_missing[@]} -gt 0 ]; then
            echo "  [MISSING] rust-pinned-toolchains components: ${comp_missing[*]}"
        fi
        if [ ${#rustup_timeout[@]} -gt 0 ]; then
            echo "  [TIMEOUT] rust-pinned-toolchains: rustup unresponsive for ${rustup_timeout[*]}"
            echo "            run: rustup self update; then re-run \`just doctor\`"
        fi
        echo "            run: just workspace rust-pinned-toolchains"
        fail=1
    fi

    # --- rust-targets ---
    # Issue 0833 — read from config/rust-targets.txt, NOT a second copy of the
    # installer's list. The copy was missing armv8r-none-eabihf, so this check
    # printed [OK] on a host where `just freertos build-fixtures` died in cmake
    # configure. A doctor with its own idea of the list is worse than no doctor.
    # shellcheck source=scripts/lib/rust-targets.sh
    source scripts/lib/rust-targets.sh
    tgt_missing=()
    # One `rustup target list` for the whole loop, matched with a shell `case`
    # rather than `grep -q`: a no-match and a grep FAILURE are both exit>=1, and
    # this loop's whole job is to tell those apart (scripts/lib/grep-q.sh).
    installed_targets=" $(timeout 5s rustup target list --installed 2>/dev/null | tr '\n' ' ') "
    while read -r t; do
        [ -n "$t" ] || continue
        case "$installed_targets" in
            *" $t "*) ;;
            *) tgt_missing+=("$t") ;;
        esac
    done < <(nros_rust_targets rustup)
    if [ ${#tgt_missing[@]} -eq 0 ]; then
        echo "  [OK] rust-targets"
    else
        echo "  [MISSING] rust-targets: ${tgt_missing[*]} (run: just workspace rust-targets)"
        fail=1
    fi

    # --- git hooks + settings (issue 0840) ---
    # `core.hooksPath` is the one that matters: unset, git runs `.git/hooks`
    # and every pre-push guarantee — duplicate issue id, submodule pin rewind,
    # and #0840's fast gate tier — is silently INERT. This repo's own hooks had
    # never been installed on the machine relying on them for a whole session,
    # and nothing said so. That is #0840's shape one level down: a gate that
    # exists, is correct, and never runs.
    #
    # Read from config/git-settings.txt, NOT a second copy of what setup-hooks
    # applies — issue 0833 is what a checker with its own idea of the list does.
    # shellcheck source=scripts/lib/git-settings.sh
    source scripts/lib/git-settings.sh
    hooks_missing=()
    hooks_advisory=()
    while IFS=$'\t' read -r key want sev; do
        [ -n "$key" ] || continue
        got="$(git config --get "$key" 2>/dev/null || true)"
        [ "$got" = "$want" ] && continue
        if [ "$sev" = "required" ]; then
            hooks_missing+=("$key=${got:-<unset>} (want $want)")
        else
            hooks_advisory+=("$key=${got:-<unset>} (want $want)")
        fi
    done < <(nros_git_settings)
    if [ ${#hooks_missing[@]} -ne 0 ]; then
        echo "  [MISSING] git hooks: ${hooks_missing[*]} (run: just setup-hooks)"
        echo "            Until then every pre-push guarantee is INERT, not merely unchecked:"
        echo "            duplicate issue ids and submodule pin REWINDS can land, and the"
        echo "            fast gate tier does not run on push (issue 0840)."
        fail=1
    elif [ ${#hooks_advisory[@]} -ne 0 ]; then
        # Not a failure: these only make a pin move LEGIBLE. Saying so beats
        # failing a host whose guarantees are all actually in place.
        echo "  [OK] git hooks (core.hooksPath); note: ${hooks_advisory[*]} (run: just setup-hooks)"
    else
        echo "  [OK] git hooks + submodule legibility settings"
    fi

    # --- cargo-tools ---
    # cargo-installed tools (nextest / llvm-cov / espflash) land in ~/.cargo/bin,
    # which rustup adds to PATH on install. `nros` is different: Phase 218
    # moved the CLI in-tree at `packages/cli/target/release/nros` (preferred)
    # with `~/.nros/bin/nros` as the transitional fallback. The user may not
    # yet have either path on PATH. Recognise each state separately so we can
    # offer a tailored hint (`source ./activate.sh` for the in-tree case,
    # `export PATH=…` for the transitional case) instead of looping on
    # `just workspace cargo-tools` → install → already-installed short-circuit.
    tool_missing=()
    for t in cargo-nextest cargo-llvm-cov espflash; do
        check command -v "$t" || tool_missing+=("$t")
    done
    nros_path_status="ok"
    nros_resolved=""
    nros_intree="{{justfile_directory()}}/packages/cli/target/release/nros"
    # Canonicalise the in-tree path so we can compare against `which nros`
    # after symlink resolution (e.g. the user might have a symlink in
    # ~/.local/bin → packages/cli/target/release/nros).
    if [ -x "$nros_intree" ]; then
        nros_intree_real="$(readlink -f "$nros_intree" 2>/dev/null || echo "$nros_intree")"
    else
        nros_intree_real=""
    fi
    if command -v nros >/dev/null 2>&1; then
        nros_resolved="$(command -v nros)"
        nros_resolved_real="$(readlink -f "$nros_resolved" 2>/dev/null || echo "$nros_resolved")"
        # Phase 220.A.1 — FAIL on stale shadow. If the resolved `nros` on
        # PATH is anything OTHER than the in-tree CLI binary, the user
        # almost certainly has a pre-218 `~/.cargo/bin/nros` (cargo
        # install) or `~/.nros/bin/nros` (deleted install-nros.sh)
        # outranking the post-218 in-tree binary. Stale CLIs emit
        # pre-K.7.1.b `[patch.crates-io]` blocks that break native rust
        # cyclonedds examples — a silent-but-broken state we want to
        # surface loudly at `just doctor` time.
        if [ -n "$nros_intree_real" ] && [ "$nros_resolved_real" = "$nros_intree_real" ]; then
            :  # resolved to the in-tree CLI — OK
        elif [ -n "$nros_intree_real" ]; then
            nros_path_status="stale-shadow"
        else
            # in-tree CLI not built yet; whatever's on PATH is the only
            # `nros` the user has. We can't classify it as a shadow
            # because we have nothing to shadow. Leave as "ok" — the
            # user is on the transitional / external-install path.
            :
        fi
    elif [ -x "$nros_intree" ]; then
        nros_path_status="needs-activate"
    elif [ -x "${NROS_HOME:-$HOME/.nros}/bin/nros" ]; then
        nros_path_status="needs-path"
    else
        nros_path_status="missing"
    fi

    if [ ${#tool_missing[@]} -eq 0 ] && [ "$nros_path_status" = "ok" ]; then
        echo "  [OK] cargo-tools"
    else
        if [ "$nros_path_status" = "stale-shadow" ]; then
            echo "  [FAIL] nros on PATH ($nros_resolved) shadows the in-tree CLI" \
                 "($nros_intree)."
            echo "         A stale pre-218 \`nros\` emits outdated"
            echo "         \`[patch.crates-io]\` blocks (from \`nros sync\`) that"
            echo "         break native rust cyclonedds examples and other downstream"
            echo "         consumers. Clean it up:"
            echo "             rm -f \"\$HOME/.cargo/bin/nros\" \"\$HOME/.nros/bin/nros\""
            echo "             source ./activate.sh"
            echo "         (Or whatever path \`which nros\` reports above, if"
            echo "         neither cargo-bin nor ~/.nros/bin is the source.)"
            fail=1
        elif [ "$nros_path_status" = "needs-activate" ]; then
            echo "  [PATH] nros built at packages/cli/target/release/nros but not on PATH —"
            echo "         run: source ./activate.sh  (or direnv allow / source ./activate.fish)"
            fail=1
        elif [ "$nros_path_status" = "needs-path" ]; then
            echo "  [PATH] nros at ~/.nros/bin/nros (transitional) but not on PATH —"
            echo "         add: export PATH=\"\$HOME/.nros/bin:\$PATH\"  (or migrate to 'just setup-cli')"
            fail=1
        elif [ "$nros_path_status" = "missing" ]; then
            tool_missing+=("nros")
        fi
        if [ ${#tool_missing[@]} -gt 0 ]; then
            echo "  [MISSING] cargo-tools: ${tool_missing[*]} (run: just workspace cargo-tools)"
            fail=1
        fi
    fi

    # --- toolchain env vars (exported by .envrc with sensible defaults) ---
    env_missing=()
    for v in NUTTX_DIR THREADX_DIR NETX_DIR FREERTOS_DIR LWIP_DIR; do
        if [ -z "${!v:-}" ]; then
            env_missing+=("$v")
        fi
    done
    if [ ${#env_missing[@]} -eq 0 ]; then
        echo "  [OK] toolchain env vars exported"
    else
        echo "  [INFO] toolchain env vars not in shell: ${env_missing[*]}"
        echo "        Run 'direnv allow' or 'source .envrc' to export defaults."
    fi

    exit $fail

# =============================================================================
# Sub-recipes — each can be run individually
# =============================================================================

# System OS packages — a thin printer over the `[system.*]` index class
# (phase-327 W2, RFC-0062). The name survives for muscle memory, but the list
# lives in nros-sdk-index.toml (abstract keys, per-manager mappings, probes)
# and this recipe never runs sudo itself: it probes, composes ONE native
# install command for the detected manager, and prints it. Run
# `nros setup --system --sudo` to execute it.
[group("setup")]
apt-packages:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    nros_bin="$(nros_cli_bin 2>/dev/null || true)"
    if [ ! -x "$nros_bin" ]; then
        echo "apt-packages: nros CLI not built (run: just setup-cli) — cannot resolve [system.*]"
        exit 0
    fi
    "$nros_bin" setup --system

# Print the list of project-owned `rust-toolchain.toml` files that
# `rust-pinned-toolchains` and `doctor` walk. Third-party vendored
# files (third-party/, scripts/zenohd/) are excluded. Private helper —
# kept as a recipe so the list has a single definition.
[private]
_pinned-toolchain-files:
    #!/usr/bin/env bash
    set -e
    # Phase 208.D.6 — replaced `shopt -s globstar; for f in
    # examples/**/rust-toolchain.toml` with a `find -prune` because
    # bash globstar descends into every `target/`, `target-*/`,
    # `build-*/` dir under `examples/` (which is GB of build output
    # on a warm tree) and took ~78 s on this host — the symptom the
    # audit reported as "`just doctor tier=default` hangs". The find
    # form prunes those dirs and returns in ~10 ms.
    emit() { [ -f "$1" ] && echo "$1"; }
    emit rust-toolchain.toml
    emit tools/rust-toolchain.toml
    emit packages/verification/nros-verification/rust-toolchain.toml
    # Tracked-file enumeration via the git index (no traversal of build trees).
    git ls-files 'examples/**/rust-toolchain.toml' 2>/dev/null

# Install every pinned toolchain referenced by a checked-in
# `rust-toolchain.toml`, together with the components each file declares.
#
# This is the single source of truth for Rust toolchain versions in the
# repo:
#   - rust-toolchain.toml                                   stable + workspace components
#   - tools/rust-toolchain.toml                             pinned nightly for workspace tooling
#   - examples/qemu-arm-nuttx/rust-toolchain.toml           nightly for -Z build-std + patched libc
#   - examples/zephyr/rust-toolchain.toml                   nightly for Corrosion + Zephyr Rust
#   - packages/verification/nros-verification/…             Verus-required channel
#
# Channels and components are never duplicated into shell scripts; the
# TOML files are authoritative. Running cargo in a directory triggers
# rustup's auto-download of the channel, but *not* of declared
# components — so this recipe installs both explicitly.
[group("setup")]
rust-pinned-toolchains:
    #!/usr/bin/env bash
    set -e
    declare -A seen
    # Split installs into two steps:
    #   1. `rustup toolchain install --profile minimal <channel>` — idempotent;
    #      skips when the toolchain is already present.
    #   2. `rustup component add <c>` per declared component — also idempotent;
    #      `rustup component add` reports "already installed" and exits 0 when
    #      the component is present.
    #
    # The two-step form replaces the prior `rustup toolchain install … -c <c>`
    # one-liner, which forces a whole-toolchain re-validation. On a tree where
    # someone has previously added a non-stock target via `rustup target add`
    # (e.g. armv7r-none-eabihf via `just workspace rust-targets`) the
    # re-validation tries to roll back changes that reference a manifest the
    # local toolchain never had (`manifest-rust-std-thumbv7em-none-eabi`
    # without the `hf` suffix, etc.) — the install then aborts mid-rollback
    # and leaves rustup state partially uninstalled. `rustup component add`
    # is per-component and self-healing, so a single missing component is
    # the only thing it touches.
    while read -r f; do
        [ -z "$f" ] && continue
        channel=$(awk '/^channel *=/ { gsub(/["]/, "", $3); print $3; exit }' "$f")
        components=$(awk '/^components *=/ { sub(/^[^=]*= */, ""); gsub(/[][",]/, " "); print; exit }' "$f")
        [ -z "$channel" ] && continue
        key="$channel | $components"
        if [ -n "${seen[$key]}" ]; then continue; fi
        seen[$key]=1
        echo "=== $channel ($f; components: ${components:-<none>}) ==="
        rustup toolchain install --profile minimal "$channel"
        for c in $components; do
            [ -z "$c" ] && continue
            rustup component add --toolchain "$channel" "$c"
        done
    done < <(just workspace _pinned-toolchain-files)

# Install cross-compilation targets.
[group("setup")]
rust-targets:
    #!/usr/bin/env bash
    set -e
    # Issue 0833 — the target list is DATA (config/rust-targets.txt), read by
    # this recipe and by `just doctor` through the same helper. It used to be
    # two hand-authored copies, and they drifted: armv8r-none-eabihf (Cortex-R52
    # — phase-372's S32Z270 and phase-385's MPS3-AN536, both with
    # `workspace-cpp-*-freertos` fixture rows) was added here and never to the
    # doctor, so a host that could not configure the FreeRTOS C++ lane at all
    # still reported `[OK] rust-targets`.
    # shellcheck source=scripts/lib/rust-targets.sh
    source scripts/lib/rust-targets.sh
    while read -r t; do
        [ -n "$t" ] || continue
        rustup target add "$t"
    done < <(nros_rust_targets rustup)
    rustup +{{NIGHTLY}} target add thumbv7m-none-eabi
    if rustc +{{NIGHTLY}} --print target-list 2>/dev/null | grep -q armv7a-nuttx-eabi; then
        echo "  armv7a-nuttx-eabi (NuttX Tier 3): supported via nightly + build-std"
    else
        echo "  WARNING: armv7a-nuttx-eabi not in nightly target list — NuttX builds may fail"
    fi

# Install cargo tools (nextest, llvm-cov, espflash, nano-ros CLI).
[group("setup")]
cargo-tools:
    #!/usr/bin/env bash
    set -e
    cargo install cargo-nextest --locked
    cargo install cargo-llvm-cov --locked
    # phase-422 W5 — FORWARDER, not a second producer. `espflash` is
    # `[tool.espflash]` in the index, pinned to `v4.5.0` with a source recipe
    # that installs into the versioned store prefix; `cargo install` put a
    # second copy in `~/.cargo/bin` with no pin at all. Both end up on PATH
    # (`scripts/sdk-path-tools.txt` adds the store's bin dir), and which one
    # wins is an accident of PATH order — issue 0500's shape, which is why the
    # `esp32 build-qemu` pack step exists to be broken by it.
    #
    # Still non-fatal: espflash is only needed to FLASH, and a workstation that
    # never touches hardware should not fail its whole tool setup over it.
    nros setup --tool espflash           || echo "WARNING: espflash provisioning failed (non-fatal)"
    cargo install rustfilt --locked      || echo "WARNING: rustfilt install failed (non-fatal)"
    cargo install cargo-show-asm --locked || echo "WARNING: cargo-show-asm install failed (non-fatal)"
    # Phase 218 — `nros` lives in-tree at `packages/cli/` (the merge
    # back into nano-ros). Build from source via `just setup-cli`
    # (idempotent: skips if the binary is fresh relative to
    # packages/cli/Cargo.lock). Contributors hacking on nros itself
    # still have `$NROS_CLI` as the override knob.
    just setup-cli
