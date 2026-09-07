#!/usr/bin/env bash
# Bootstrap nano-ros from a fresh checkout without requiring `just` first.

set -euo pipefail

# `cd -P` resolves symlinks: a checkout reached through a symlinked parent
# (e.g. ~/data -> /mnt/wd) must record the ONE physical path, or the alias ends
# up in the rc line proposed below, in NROS_REPO_DIR, and in RFC-0048's
# absolute-path `nros sync` output — two names for one tree (issue 0375).
# activate.sh / activate.fish resolve the same way.
SCRIPT_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd -P "${SCRIPT_DIR}/.." && pwd -P)"

# Global flags — set by main() before subcommand dispatch.
DRY_RUN=0
PROMPT=0
NO_PROMPT=0

usage() {
    cat <<'EOF'
Usage:
  scripts/bootstrap.sh [flags] <subcommand> [args]

Subcommands:
  (no subcommand)            THE front door: build the nros CLI from source
                             (installs rustup if needed; no just required),
                             then print the next step (`nros setup <board>`)
  nros                       alias of the no-subcommand front door
  base                       contributor quick-start (front door + just +
                             `just setup base`)
  all                        full contributor / test-all setup
  platform <name>            focused platform setup
  doctor [tier]              read-only diagnosis of build environment
                             (delegates to `just doctor`)
  shell-doctor               read-only diagnosis of SHELL state
                             (PATH, version lockstep, rc-file activate
                             line, stale deprecated-verb aliases)

Flags:
  --dry-run                  print every destructive command, run none
  --prompt                   ask before each destructive step (default off)
  --no-prompt                never ask (CI mode); overrides --prompt
  -h, --help                 this message

This is the CONTRIBUTOR front door, and it builds from source on purpose.
Inside a checkout the tree's own build is the only correct binary: a released
`nros` run against a nano-ros checkout is refused (RFC-0090 / phase-431 W1)
and reported by `just doctor` as a shadow (W2), and it cannot stand in for the
checkout's binary either, since `nros source-stamp` compares against the
sources it was built from. It leaves the binary at
packages/cli/target/release/nros; `just setup-cli` is the internal alias for
the same build.

USERS who only want the tool need no checkout and no cargo (phase-431 W4):

  curl -fsSL https://raw.githubusercontent.com/NEWSLabNTU/nano-ros/main/scripts/install.sh | sh

That installs a released binary into ~/.nros and fronts it at ~/.nros/bin/nros.

After a successful front-door / `base` run the script offers to append a
`source <repo>/activate.sh` line to your shell rc (auto-detected from
$SHELL — bash / zsh / fish). Decline with --no-prompt to skip.

Examples:
  scripts/bootstrap.sh
  scripts/bootstrap.sh --dry-run base
  scripts/bootstrap.sh --no-prompt base
  scripts/bootstrap.sh platform zephyr
  scripts/bootstrap.sh shell-doctor
EOF
}

# ---------------------------------------------------------------------------
# Dry-run / prompt helpers
# ---------------------------------------------------------------------------

# run_cmd "<step description>" <cmd...>
#   - Always echoes the command.
#   - If --dry-run, returns 0 without executing.
#   - If --prompt (and not --no-prompt), asks before executing.
run_cmd() {
    local desc="$1"; shift
    echo "bootstrap: $desc"
    printf '  +'
    printf ' %q' "$@"
    printf '\n'

    if [[ $DRY_RUN -eq 1 ]]; then
        return 0
    fi

    if [[ $PROMPT -eq 1 && $NO_PROMPT -ne 1 ]]; then
        if [[ ! -t 0 ]]; then
            echo "bootstrap: --prompt requested but stdin is not a TTY; treating as 'no'." >&2
            return 1
        fi
        local reply
        read -r -p "  Proceed? [y/N] " reply
        case "${reply,,}" in
            y|yes) ;;
            *)
                echo "bootstrap: skipping."
                return 1
                ;;
        esac
    fi

    "$@"
}

# run_shell "<step description>" "<shell-string>"
#   - Same prompt/dry-run gate as run_cmd, but evaluates the string in `sh`.
#     Used for the rustup curl-pipe + the `set -a; source .env` shell snippets.
run_shell() {
    local desc="$1"; shift
    local snippet="$1"; shift
    echo "bootstrap: $desc"
    echo "  + sh -c '$snippet'"

    if [[ $DRY_RUN -eq 1 ]]; then
        return 0
    fi

    if [[ $PROMPT -eq 1 && $NO_PROMPT -ne 1 ]]; then
        if [[ ! -t 0 ]]; then
            echo "bootstrap: --prompt requested but stdin is not a TTY; treating as 'no'." >&2
            return 1
        fi
        local reply
        read -r -p "  Proceed? [y/N] " reply
        case "${reply,,}" in
            y|yes) ;;
            *)
                echo "bootstrap: skipping."
                return 1
                ;;
        esac
    fi

    sh -c "$snippet"
}

# ---------------------------------------------------------------------------
# Install steps
# ---------------------------------------------------------------------------

ensure_path() {
    case ":$PATH:" in
        *":$HOME/.local/bin:"*) ;;
        *) export PATH="$HOME/.local/bin:$PATH" ;;
    esac
    case ":$PATH:" in
        *":$HOME/.cargo/bin:"*) ;;
        *) export PATH="$HOME/.cargo/bin:$PATH" ;;
    esac
}

install_rustup_if_missing() {
    if command -v cargo >/dev/null 2>&1; then
        return 0
    fi
    if ! command -v curl >/dev/null 2>&1; then
        echo "bootstrap: cargo and curl are missing; install Rust or curl, then rerun." >&2
        exit 1
    fi
    run_shell "installing rustup (non-interactive, minimal profile)" \
        "curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --no-modify-path" \
        || return 0
    # shellcheck disable=SC1091
    [[ -f "$HOME/.cargo/env" ]] && source "$HOME/.cargo/env"
    ensure_path
}

ensure_just() {
    ensure_path
    if command -v just >/dev/null 2>&1; then
        return 0
    fi

    install_rustup_if_missing
    if ! command -v cargo >/dev/null 2>&1; then
        # --dry-run path leaves cargo absent. Echo what we WOULD do then return.
        echo "bootstrap: (cargo absent — skipping just install)"
        return 0
    fi
    mkdir -p "$HOME/.local"
    run_cmd "installing just into $HOME/.local/bin" \
        cargo install just --locked --root "$HOME/.local" \
        || return 0
    ensure_path

    if [[ $DRY_RUN -eq 0 ]] && ! command -v just >/dev/null 2>&1; then
        echo "bootstrap: just installed but not found on PATH; add $HOME/.local/bin." >&2
        exit 1
    fi
}

# Init the submodules the in-tree CLI build needs. Scoped to the CLI's own
# third-party dep — NOT a blanket `--init --recursive` over the whole repo, which
# would drag in the gitignored platform SDKs (zenoh, cyclonedds, …).
#
# RFC-0060 as amended by phase-332: the launch pin is `play_launch`; layer 2
# (the resolver `nros-launch-resolve` compiles in) is REGULAR FILES at
# `src/ros-launch-resolve` inside it, and ros-launch-manifest is a git-tag
# cargo dep — nothing nests, so the init is NON-recursive. `--recursive`
# would drag in play_launch's layer-3 runtime submodules, which nano-ros
# never builds. (This function has drifted on every rename — 0336 named the
# retired `ros-launch-manifest`, then this spelling named the retired
# `ros-launch-resolve` — so the retired paths are gated by
# scripts/check-retired-submodule-refs.sh.)
ensure_cli_submodules() {
    local sub="packages/cli/third-party/play_launch"
    # Already populated (has the resolver manifest setup-launch-resolve reads)?
    if [[ -f "${REPO_ROOT}/${sub}/src/ros-launch-resolve/resolve/Cargo.toml" ]]; then
        return 0
    fi
    if ! command -v git >/dev/null 2>&1; then
        echo "bootstrap: git not on PATH — cannot init CLI submodule ${sub}." >&2
        return 1
    fi
    run_cmd "initializing in-tree CLI submodule (${sub})" \
        git -C "${REPO_ROOT}" submodule update --init "${sub}" \
        || return 1
}

# Build the in-tree CLI (`packages/cli/`) from source. Used by `base`.
build_in_tree_cli() {
    if [[ ! -f "${REPO_ROOT}/packages/cli/Cargo.toml" ]]; then
        echo "bootstrap: packages/cli/Cargo.toml not found — Phase 218 monorepo merge?" >&2
        return 1
    fi
    ensure_cli_submodules || return 1
    if ! command -v cargo >/dev/null 2>&1; then
        if [[ $DRY_RUN -eq 1 ]]; then
            echo "bootstrap: (cargo absent — would build CLI from packages/cli/)"
            return 0
        fi
        echo "bootstrap: cargo not on PATH; rustup install must have failed." >&2
        return 1
    fi
    run_cmd "building in-tree nros CLI (packages/cli/)" \
        # profile-literal-ok: host tool: builds the CLI before any profile machinery exists
        cargo build --release --manifest-path "${REPO_ROOT}/packages/cli/Cargo.toml" --bin nros \
        || return 0
    # phase-368 W4 — the launch resolver is part of the front door, not an
    # extra step. Every workspace configure (`nano_ros_entry`, `nros sync`)
    # resolves the bringup's launch file through `nros-launch-resolve`; a CLI
    # without it fails the FIRST `nros new --workspace` build with an error
    # naming a `just` recipe the user does not have. Built into the shared
    # packages/cli target dir, it is a SIBLING of `nros` — resolution rule 2
    # in `resolve_launch_resolver`, so nothing else needs configuring. The
    # clean-container probe caught this: steps 10–40 passed and the quick
    # start died at configure. The resolver is its OWN workspace (own lock,
    # deliberately excluded from packages/cli — see its manifest header), so
    # it builds by its own manifest path and lands at
    # packages/cli/nros-launch-resolve/target/release/ — resolution rule 3
    # in `resolve_launch_resolver` (the per-checkout build path).
    # profile-literal-ok: host tool: the launch resolver's own binary
    # `|| return 1`, NOT the CLI step's `|| return 0`: the first probe run of
    # this step failed on a missing libclang, the swallow printed the happy
    # "next →" banner anyway, and the failure surfaced two commands later as
    # a cmake configure error naming a `just` recipe. A front door that
    # cannot build what the quick start needs must say so HERE.
    run_cmd "building nros-launch-resolve (the launch resolver)" \
        # profile-literal-ok: host tool: the launch resolver's own binary, same rule as the CLI build above
        cargo build --release --manifest-path "${REPO_ROOT}/packages/cli/nros-launch-resolve/Cargo.toml" \
        || { echo "bootstrap: nros-launch-resolve build FAILED — the workspace quick start needs it (see error above; libclang missing is the common cause)" >&2; return 1; }
    export PATH="${REPO_ROOT}/packages/cli/target/release:$PATH"
}

# Phase 288 (D1/D2) — THE user front door. nano-ros is a source
# distribution: the only way to obtain `nros` is to build the in-tree
# sub-workspace at `packages/cli/` with cargo (rustup is installed on
# demand; `just` is NOT required). Two states:
#   1. Already on PATH (a previous build or activate.sh) — no-op.
#   2. Otherwise — rustup if needed, init the CLI submodule, cargo build.
install_nros_source() {
    ensure_path
    if command -v nros >/dev/null 2>&1; then
        echo "bootstrap: nros already on PATH ($(command -v nros))."
        echo "bootstrap: next →  nros setup <board>   then use cargo / cmake / west / idf.py"
        return 0
    fi

    install_rustup_if_missing
    build_in_tree_cli || return 1
    echo "bootstrap: next →  source ./activate.sh   then   nros setup <board>"
    offer_shell_rc_update
}

# Phase 222.E.1 — the bare-machine path. Runs the cold-cache route:
#   1. install rustup (if `cargo` is absent)
#   2. install just  (if `just` is absent)
#   3. build the in-tree CLI (`packages/cli/`) -> `packages/cli/target/release/nros`
#   4. export PATH for the CURRENT shell (~/.cargo/bin + packages/cli/target/release)
#   5. offer to append the activate-source line to the user's shell rc
# After this completes the user has `cargo`, `just`, `nros` on PATH and
# (optionally) every future shell auto-sources activate.
install_base() {
    install_rustup_if_missing
    ensure_just
    build_in_tree_cli
    # PATH already exported into this shell by ensure_path + build_in_tree_cli.
    echo "bootstrap: base install done."
    echo "bootstrap: next →  source ./activate.sh   then   nros setup <board>"
    offer_shell_rc_update
}

# ---------------------------------------------------------------------------
# Phase 222.E.2 — shell rc update
# ---------------------------------------------------------------------------

detect_shell_rc() {
    # Echo the rc file path AND the source-snippet on two lines: rc\nsnippet.
    local rc snippet
    local shell_name
    shell_name="$(basename "${SHELL:-bash}")"
    case "$shell_name" in
        zsh)
            rc="$HOME/.zshrc"
            snippet="# nano-ros workspace (Phase 218.C activation)
source \"${REPO_ROOT}/activate.sh\""
            ;;
        fish)
            rc="$HOME/.config/fish/config.fish"
            snippet="# nano-ros workspace (Phase 218.C activation)
source \"${REPO_ROOT}/activate.fish\""
            ;;
        bash|*)
            rc="$HOME/.bashrc"
            snippet="# nano-ros workspace (Phase 218.C activation)
source \"${REPO_ROOT}/activate.sh\""
            ;;
    esac
    printf '%s\n%s\n' "$rc" "$snippet"
}

offer_shell_rc_update() {
    local rc snippet detect
    detect="$(detect_shell_rc)"
    rc="$(printf '%s' "$detect" | sed -n '1p')"
    snippet="$(printf '%s' "$detect" | sed -n '2,$p')"

    # Idempotence: if the activate path is already mentioned in rc, skip silently.
    local activate_path
    case "$rc" in
        *fish*) activate_path="${REPO_ROOT}/activate.fish" ;;
        *)      activate_path="${REPO_ROOT}/activate.sh" ;;
    esac
    if [[ -f "$rc" ]] && grep -qF "$activate_path" "$rc" 2>/dev/null; then
        echo "bootstrap: $rc already sources $activate_path — skipped."
        return 0
    fi

    # Print the snippet on stderr so the user always sees what would land.
    echo "bootstrap: proposed addition to $rc:" >&2
    printf '%s\n' "$snippet" >&2

    if [[ $NO_PROMPT -eq 1 ]]; then
        echo "bootstrap: --no-prompt set — leaving $rc untouched."
        return 0
    fi

    if [[ $DRY_RUN -eq 1 ]]; then
        echo "bootstrap: (dry-run) would append the snippet above to $rc."
        return 0
    fi

    # Default-on prompt for rc edits regardless of --prompt flag — this is
    # a touch of the user's $HOME, never silent.
    if [[ ! -t 0 ]]; then
        echo "bootstrap: stdin not a TTY; leaving $rc untouched. Append the snippet manually."
        return 0
    fi
    local reply
    read -r -p "Append to $rc? [y/N] " reply
    case "${reply,,}" in
        y|yes) ;;
        *)
            echo "bootstrap: leaving $rc untouched."
            return 0
            ;;
    esac

    mkdir -p "$(dirname "$rc")"
    {
        printf '\n'
        printf '%s\n' "$snippet"
    } >>"$rc"
    echo "bootstrap: appended to $rc."
}

# ---------------------------------------------------------------------------
# Phase 222.E.3 — bootstrap.sh shell-doctor
# ---------------------------------------------------------------------------

# Read-only diagnosis of the user's SHELL state — distinct from
# `just doctor` which inspects BUILD state. Pre-Phase-222.C lane:
# surfaces stale `nros build` / `nros run` / `nros deploy` / `nros monitor`
# aliases that 0.5.0 will delete.
shell_doctor() {
    local fail=0 warn=0

    echo "=== nano-ros shell-doctor ==="

    # 1. nros on PATH?
    if command -v nros >/dev/null 2>&1; then
        echo "[OK] nros on PATH: $(command -v nros)"
    else
        echo "[FAIL] nros not on PATH — run \`source ./activate.sh\` or \`scripts/bootstrap.sh base\`."
        fail=1
    fi

    # 2. Version lockstep.
    if [[ -x "${REPO_ROOT}/scripts/check-version-lockstep.sh" ]]; then
        if "${REPO_ROOT}/scripts/check-version-lockstep.sh" >/dev/null 2>&1; then
            echo "[OK] nros --version matches packages/cli/Cargo.toml"
        else
            echo "[WARN] nros --version drift vs packages/cli/Cargo.toml — run \`just setup-cli\`."
            warn=1
        fi
    else
        echo "[WARN] scripts/check-version-lockstep.sh missing — cannot check version drift."
        warn=1
    fi

    # 3. Activate-source line in shell rc.
    local detect rc activate_path
    detect="$(detect_shell_rc)"
    rc="$(printf '%s' "$detect" | sed -n '1p')"
    case "$rc" in
        *fish*) activate_path="${REPO_ROOT}/activate.fish" ;;
        *)      activate_path="${REPO_ROOT}/activate.sh" ;;
    esac
    if [[ -f "$rc" ]] && grep -qF "$activate_path" "$rc" 2>/dev/null; then
        echo "[OK] $rc sources $activate_path"
    else
        echo "[WARN] $rc does NOT source $activate_path — run \`scripts/bootstrap.sh base\` to add."
        warn=1
    fi

    # 4. Stale deprecated-verb aliases (Phase 222.B pre-flight).
    local rc_glob alias_hits=0
    for rc_glob in "$HOME/.bashrc" "$HOME/.zshrc" "$HOME/.config/fish/config.fish" "$HOME/.bash_profile" "$HOME/.profile"; do
        [[ -f "$rc_glob" ]] || continue
        # Match either `alias nros-build=…` or `alias <foo>='nros build …'`,
        # for each of the four deprecated verbs.
        if grep -Eq "^[[:space:]]*alias[[:space:]][^=]*=['\"]?nros[[:space:]]+(build|run|deploy|monitor)\b|^[[:space:]]*alias[[:space:]]+nros-(build|run|deploy|monitor)[[:space:]]*=" "$rc_glob" 2>/dev/null; then
            echo "[WARN] $rc_glob references a deprecated nros verb (build/run/deploy/monitor) — to be removed in nros 0.5.0."
            alias_hits=$((alias_hits + 1))
        fi
    done
    if [[ $alias_hits -eq 0 ]]; then
        echo "[OK] no deprecated-verb aliases (nros build/run/deploy/monitor) in shell rc files."
    else
        warn=1
    fi

    echo "=== end shell-doctor ==="
    if [[ $fail -ne 0 ]]; then
        return 1
    fi
    if [[ $warn -ne 0 ]]; then
        return 0  # warnings don't fail the lane
    fi
    return 0
}

# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

main() {
    # Parse leading flags.
    local args=()
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --dry-run)
                DRY_RUN=1
                shift
                ;;
            --prompt)
                PROMPT=1
                shift
                ;;
            --no-prompt)
                NO_PROMPT=1
                PROMPT=0
                shift
                ;;
            -h|--help|help)
                usage
                exit 0
                ;;
            --)
                shift
                args+=("$@")
                break
                ;;
            *)
                args+=("$1")
                shift
                ;;
        esac
    done
    set -- "${args[@]}"

    case "${1:-}" in
        ""|nros)
            # Phase 288 — the default IS the front door: source-build the CLI.
            install_nros_source
            exit 0
            ;;
        shell-doctor)
            shell_doctor
            exit $?
            ;;
        base|quickstart|default|minimal)
            install_base
            # Hand off to `just setup base` for the SDK tier on top of the
            # base CLI install (idempotent — skips work already done).
            if command -v just >/dev/null 2>&1 && [[ $DRY_RUN -eq 0 ]]; then
                cd "$REPO_ROOT"
                exec just setup base
            fi
            exit 0
            ;;
    esac

    ensure_just
    cd "$REPO_ROOT"

    case "${1:-}" in
        all|everything|contributor|extended)
            echo "bootstrap: full setup will fetch/install all supported platform SDKs."
            exec just setup all
            ;;
        platform)
            if [[ ${#args[@]} -lt 2 ]]; then
                echo "bootstrap: missing platform name." >&2
                usage >&2
                exit 2
            fi
            exec just "${args[1]}" setup
            ;;
        doctor)
            if [[ ${#args[@]} -ge 2 ]]; then
                exec just doctor "${args[1]}"
            fi
            exec just doctor
            ;;
        *)
            echo "bootstrap: unknown command: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
}

main "$@"
