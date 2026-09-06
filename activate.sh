# nano-ros workspace activation — bash / zsh / plain POSIX sh (dash: source
# it FROM the checkout root; a wrong cwd is refused loudly).
#
# Phase 218.C — single source of truth for env exports + PATH wiring.
# Source this once after `git clone`:
#
#     source ./activate.sh
#
# direnv users get it for free via `.envrc` (which sources this file).
# fish users source `./activate.fish` instead — the two files stay
# manually mirrored.
#
# This file does NOT install anything. It WIRES paths to artifacts that
# `just setup` (or `just setup-cli` for the CLI alone) produces. If the
# corresponding binaries / SDKs are absent, the export is harmlessly
# skipped — the script never errors.

# Resolve the workspace root per shell: ${BASH_SOURCE[0]} for bash,
# ${(%):-%N} for zsh, and for a plain POSIX sh the current directory —
# verified, because sh cannot know a sourced file's own path.
# when the script is `source`d.
#
# `cd -P` resolves symlinks, so a checkout reached through a symlinked parent
# (e.g. ~/data -> /mnt/wd) still records the ONE physical path. Plain `cd` +
# `pwd` keeps the alias, and the alias then propagates into NROS_REPO_DIR,
# nano_ros_ROOT, the rc line bootstrap.sh proposes, RFC-0048's absolute-path
# `nros sync` output, and every path-keyed build cache — two names for one
# tree (issue 0375). scripts/bootstrap.sh resolves the same way.
# The CONDITION tests shell-version variables, not ${BASH_SOURCE[0]} — a
# plain POSIX sh (dash) rejects that as "Bad substitution" when the line
# EXECUTES, and the old `[ -n "${BASH_SOURCE[0]:-}" ]` guard executed in
# every shell. The branch BODIES are safe unquoted: a substitution a shell
# cannot parse only errors on the line that runs, and each body runs only
# in its own shell (measured: dash runs a file carrying both spellings in
# untaken branches without complaint).
if [ -n "${BASH_VERSION:-}" ]; then
    _nros_root="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
elif [ -n "${ZSH_VERSION:-}" ]; then
    _nros_root="$(cd -P "$(dirname "${(%):-%N}")" && pwd -P)"
else
    # Plain POSIX sh cannot know a sourced file's own path ($0 is the SHELL
    # here, not this file). Fall back to the current directory.
    _nros_root="$(pwd -P)"
fi
# Verify in EVERY shell — a wrong silent root would poison NROS_REPO_DIR,
# the patch tables, and every path-keyed cache (issue 0375's
# two-names-for-one-tree, worse). This is what catches a shell whose
# self-path idiom quietly broke (an eval'd `${(%):-%N}` returned the CALLER
# under zsh during this fix's own review — root=/tmp, no complaint).
if [ ! -f "$_nros_root/activate.sh" ] || [ ! -f "$_nros_root/nros-sdk-index.toml" ]; then
    echo "activate.sh: resolved workspace root '$_nros_root' does not look like the" >&2
    echo "  nano-ros checkout. Under bash/zsh, source the file by its real path;" >&2
    echo "  under a plain POSIX sh, run it FROM the checkout root" >&2
    echo "  (cd <checkout> && . ./activate.sh)." >&2
    return 1 2>/dev/null || exit 1
fi
export NROS_REPO_DIR="$_nros_root"

# --- SDK-store lookup helpers (issue 0372) ---------------------------------
# THE RULE FOR THIS FILE: never let a glob reach the shell's word expansion.
# zsh's default `nomatch` makes an UNMATCHED glob a FATAL error, and this file
# is `source`d — so a bare `for d in "$store"/*/bin` (or `ls "$d"/*-gcc`) does
# not "skip harmlessly", it kills activation mid-file and silently drops every
# export below it. `find` reports nothing instead of failing, in every shell.
# Both lookups below go through these two helpers; add a third caller here
# rather than reintroducing a glob at the call site.

# List `bin` directories exactly $2 levels below root $1 (nothing if absent).
_nros_bin_dirs() {
    [ -d "$1" ] || return 0
    find "$1" -mindepth "$2" -maxdepth "$2" -type d -name bin 2>/dev/null
}

# True when directory $1 holds at least one `*-gcc` (cross-toolchain probe).
_nros_dir_has_gcc() {
    [ -n "$(find "$1" -maxdepth 1 -name '*-gcc' 2>/dev/null)" ]
}
# RFC-0048 (phase-287): the ament shape's `find_package(nano_ros REQUIRED)`
# locates the in-tree `nano_rosConfig.cmake` via CMake's `<pkg>_ROOT` env var.
# Exporting it here means a sourced shell needs no `-Dnano_ros_ROOT`; a copy-out
# built outside a sourced shell passes `-Dnano_ros_ROOT=<checkout>` (or a `nros
# setup` CMakePreset carries it).
export nano_ros_ROOT="$_nros_root"

# Rustup-managed toolchain: `scripts/bootstrap.sh` installs rustup, but only
# FUTURE shells pick up `~/.cargo/bin` (rustup edits .bashrc/.profile). The
# book's fresh-machine flow stays in the bootstrap shell, so wire it here —
# otherwise `nros setup`'s source builds die with `cargo: not found`
# (issue #204 probe finding).
if [ -d "$HOME/.cargo/bin" ]; then
    case ":$PATH:" in
        *":$HOME/.cargo/bin:"*) ;;
        *) export PATH="$HOME/.cargo/bin:$PATH" ;;
    esac
fi

# ROS 2 Humble — sourcing setup.bash exports AMENT_PREFIX_PATH,
# CMAKE_PREFIX_PATH, ROS_DISTRO, etc. Required by `nros generate-rust`
# (resolves .msg defs via rosidl_adapter) + the cyclonedds codegen +
# every rmw_zenoh interop test.
#
# Issue 0639 — pick the file the CURRENT shell can actually read, and then
# check that it worked.
#
# This used to source `setup.bash` unconditionally. `setup.bash` is a bash
# script: under zsh it sets NOTHING (it reads `${BASH_SOURCE[0]}`, which zsh
# does not define) and it fails SILENTLY, so a zsh user got an activate.sh that
# reported success and no ROS environment at all. That is how a fixture lane
# launched from zsh invoked `/opt/ros/humble/bin/idlc` with no
# `LD_LIBRARY_PATH` and died `FAILED: [code=127]` on the first `.idl`, which
# reads as a missing tool rather than a missing environment.
#
# Measured on this host:
#
#   bash + setup.bash -> ROS_DISTRO=humble, LD_LIBRARY_PATH set
#   zsh  + setup.bash -> BOTH UNSET
#   zsh  + setup.sh   -> ROS_DISTRO=humble, LD_LIBRARY_PATH set
#   bash + setup.sh   -> ROS_DISTRO=humble, LD_LIBRARY_PATH set
#
# `setup.sh` is the POSIX one and works in both, so it is the fallback. bash
# keeps `setup.bash` so a working setup sees no change at all (it also pulls
# bash completions, which `setup.sh` does not).
#
# `activate.fish` has always handled this correctly — it looks for `setup.fish`
# and, when there is none, says so and names the remedy. This file was the one
# that assumed its own shell.
_nros_ros_setup=""
if [ -n "${BASH_VERSION:-}" ] && [ -f /opt/ros/humble/setup.bash ]; then
    _nros_ros_setup=/opt/ros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.sh ]; then
    _nros_ros_setup=/opt/ros/humble/setup.sh
elif [ -f /opt/ros/humble/setup.bash ]; then
    _nros_ros_setup=/opt/ros/humble/setup.bash
fi

if [ -n "$_nros_ros_setup" ]; then
    # ROS's setup scripts are not `set -u`-clean (`AMENT_TRACE_SETUP_FILES:
    # unbound variable` at setup.bash:8) — a strict-mode shell (`set -u`, the
    # probe's extracted script, any CI runner using `set -euo pipefail`) dies
    # INSIDE the sourced file, which reads as an activate.sh failure. Sourcing
    # someone else's script inherits their hygiene: suspend nounset for the
    # duration, restore whatever the caller had. `case $-` is POSIX and works
    # in bash and zsh (zsh's $- carries the same letters for set-options).
    case $- in *u*) _nros_had_nounset=1 ;; *) _nros_had_nounset="" ;; esac
    set +u
    # shellcheck disable=SC1090,SC1091
    . "$_nros_ros_setup"
    if [ -n "$_nros_had_nounset" ]; then set -u; fi
    unset _nros_had_nounset
    # Sourcing can succeed and set nothing (the zsh/`setup.bash` case above).
    # Say so rather than leave a build to discover it as `code=127` later.
    if [ -z "${ROS_DISTRO:-}" ]; then
        # Name the SHELL, not `$0` — when a file is sourced, `$0` is the file,
        # which would point the reader at activate.sh when the shell is the
        # thing that matters.
        if [ -n "${ZSH_VERSION:-}" ]; then _nros_shell=zsh
        elif [ -n "${BASH_VERSION:-}" ]; then _nros_shell=bash
        else _nros_shell="sh-compatible"; fi
        echo "activate.sh: sourced $_nros_ros_setup but ROS_DISTRO is still unset —" \
            "the ROS environment did NOT load (shell: $_nros_shell). ROS-dependent" \
            "recipes will fail with missing tools or 'error while loading shared" \
            "libraries'. See issue 0639." >&2
    fi
    unset _nros_ros_setup _nros_shell
else
    # issue 0373 F3 — the bare "ROS-dependent recipes will fail" left a
    # first-time reader unable to tell whether their setup was broken. It is
    # not: setup, `nros sync`, codegen and every BUILD work without ROS 2
    # (interface sources are vendored in packages/cli/interfaces/, verified
    # end-to-end on a ROS-less host). Name what actually needs it.
    #
    # issue 0653 — RUNNING split off from BUILDING when RFC-0075 made the zenoh
    # router `rmw_zenoh_cpp/rmw_zenohd`, which ships with ROS. zenoh-pico is a
    # client, so a two-process zenoh example now needs that router and this host
    # has none. Say so here: the alternative is a silent hang in `Executor::open`
    # with nothing connecting it back to the missing ROS install.
    echo "activate.sh: /opt/ros/humble/setup.bash not found — ROS-dependent recipes will fail" \
        "(interop tests, \`ros2\` CLI verification). Setup, codegen and every BUILD do not" \
        "need it. RUNNING a multi-process zenoh example does: its router is ROS's" \
        "rmw_zenohd (set NROS_RMW_ZENOHD to override), or use --rmw cyclonedds, which" \
        "needs no daemon — see book/src/getting-started/installation.md" >&2
fi

# Issues 0359/0378 — project-wide cargo args, injected by a PATH shim.
#
# `Cargo.lock` only means something if builds REFUSE to rewrite it, and cargo
# has no config key or env var for `--locked` (both verified: `[build] locked`
# is an "unused config key", `CARGO_LOCKED` is ignored). It is CLI-only.
#
# Rather than append the flag at 57 call sites — which would still miss cmake
# and corrosion, since they invoke `cargo` BY NAME — the flags are defined once
# here and injected by `scripts/bin/cargo`. Same mechanism this file already
# uses for `nros`, `play_launch_parser` and `zenohd`.
#
# Escape hatch, for a deliberate dependency change:
#     NROS_CARGO_FLAGS= just <recipe>      # or: just lock-update <crate>
: "${NROS_CARGO_FLAGS=--locked}"
export NROS_CARGO_FLAGS

# pyo3 refuses an interpreter newer than the maximum it knows, and a rolling
# distro outruns that — this fired as:
#
#   error: the configured Python interpreter version (3.14) is newer than
#          PyO3's maximum supported version (3.13)
#
# pyo3's own remedy is to build against the stable ABI. It lived only in
# `just setup-launch-resolve`, so every OTHER lane that builds the resolver
# — `check-cli-clippy`, `check-test-targets`, … — still died on a 3.14 host.
# One export here covers all of them; it is inert where the interpreter is
# already supported, so there is no version probe and no second code path.
#
# The "revisit when the pin moves past 0.24" this comment used to end with has
# happened: `packages/cli` is on pyo3 0.29, which supports 3.14 NATIVELY —
# verified by building `colcon-nano-ros` with this variable explicitly UNSET.
# The export stays anyway, and is not dead weight: it is the forward guard for
# the next interpreter to outrun the pin, which is the same event that produced
# it. What it no longer does is paper over the CURRENT host.
#
# NOTE: `packages/cli/third-party/play_launch` is VENDORED and still pins 0.24,
# so that tree does depend on this. Do not remove the export while that pin
# stands.
: "${PYO3_USE_ABI3_FORWARD_COMPATIBILITY=1}"
export PYO3_USE_ABI3_FORWARD_COMPATIBILITY
case ":$PATH:" in
    *":$_nros_root/scripts/bin:"*) ;;
    *) export PATH="$_nros_root/scripts/bin:$PATH" ;;
esac

# `nros` CLI resolution: the in-tree per-checkout binary at
# `packages/cli/target/release/nros`, built by `just setup-cli`. Each
# nano-ros worktree carries its own CLI — no global PATH skew across
# trees. This is the sole source: the pre-218 `~/.nros/bin/nros` curl
# install (`scripts/install-nros.sh`) is retired, and the standalone
# `NEWSLabNTU/nros-cli` repo was merged in-tree at `packages/cli/`.
# Prepend UNCONDITIONALLY, whether or not the binary is there yet.
#
# This used to be `if [ -x <the binary> ]`, which loses the race a CI job runs
# every time: `source ./activate.sh` happens BEFORE `just setup <scope>` builds
# the CLI, so on a fresh checkout the directory was empty, the prepend was
# skipped, and PATH stayed fixed for the rest of the step. `setup-cli` then
# built the CLI into a directory that was not on PATH, and `nros` kept
# resolving to whatever else the host had — on the self-hosted runner, a
# DIFFERENT checkout at /mnt/evo/aeon/nano-ros. That CLI was stale against its
# own sources, so `just setup tier2` died with
#
#     Error: in-tree nros CLI is STALE — its sources changed since it was built
#
# blaming this checkout for another one's state. `build-wide` failed this way on
# every run once the runner was provisioned.
#
# Prepending a not-yet-populated directory is harmless — nothing resolves from
# it until the build lands — and it makes "this checkout's CLI wins" true
# regardless of the order the two steps run in. The hint below still fires
# correctly: `command -v nros` cannot find anything in an empty directory.
export PATH="$_nros_root/packages/cli/target/release:$PATH"
if [ -x "$_nros_root/packages/cli/target/release/nros" ]; then
    :
elif [ -z "${NROS_QUIET_ACTIVATE:-}" ] && ! command -v nros >/dev/null 2>&1; then
    # Phase 222.F.1 — first-run hint. The checkout has no built CLI AND
    # `nros` is not resolvable from any other PATH entry (e.g. ~/.nros/bin).
    # Tell the user how to fix it explicitly instead of letting a silent
    # "command not found" surprise them minutes later. Suppress with
    # NROS_QUIET_ACTIVATE=1 (CI lanes that build the CLI as a separate step).
    echo "[nano-ros] CLI not built yet. Run:" >&2
    echo "  ./scripts/bootstrap.sh           (builds the CLI from source; installs rustup if needed)" >&2
    echo "  Equivalent, if you have cargo:" >&2
    echo "  git submodule update --init packages/cli/third-party/play_launch \\" >&2
    echo "    && cargo build --release --manifest-path packages/cli/Cargo.toml --bin nros" >&2
    echo "  (set NROS_QUIET_ACTIVATE=1 to suppress this hint.)" >&2
fi

# `play_launch_parser` — installed by `just workspace install-play-
# launch-parser` to `~/.nros/sdk/play_launch_parser/bin/`. The Phase
# 212.L.6 launch-graph resolver shells out to it. The Phase 212.M-F.20
# pkg-index resolver inside the parser eats `AMENT_PREFIX_PATH` so the
# ROS source above must run FIRST.
if [ -x "${NROS_HOME:-$HOME/.nros}/sdk/play_launch_parser/bin/play_launch_parser" ]; then
    export PATH="${NROS_HOME:-$HOME/.nros}/sdk/play_launch_parser/bin:$PATH"
else
    # phase-327 W3 — `nros setup --tool play_launch_parser` (the prebuilt
    # remedy the doctor names) installs to the VERSIONED store layout
    # (sdk/<tool>/<version>/bin), which the unversioned path above misses.
    # Wire whichever version is present so both install paths work.
    while IFS= read -r _plp_bin; do
        [ -n "$_plp_bin" ] || continue
        if [ -x "$_plp_bin/play_launch_parser" ]; then
            export PATH="$_plp_bin:$PATH"
            break
        fi
    done <<EOF
$(_nros_bin_dirs "${NROS_HOME:-$HOME/.nros}/sdk/play_launch_parser" 2)
EOF
    unset _plp_bin
fi

# Cross-compiler toolchains installed by `nros setup` land in the SDK store
# (~/.nros/sdk/<tool>/<version>/bin). Unlike qemu — which the test harness
# resolves via a build/<tool> prefix and deliberately keeps OFF the global
# PATH — a cross-gcc MUST be on PATH for cargo's `linker=` and the
# NuttX/Zephyr `make` to find it (e.g. riscv-none-elf-gcc for the riscv NuttX
# board, Phase 194.3c). Scope to store bin dirs that hold a whitelisted tool
# so the build/<tool> convention for qemu is preserved. A system cross-gcc
# (e.g. /usr/bin/arm-none-eabi-gcc) still resolves when the store has none.
#
# THAT FALLBACK STAYS, AND IT IS NO LONGER SILENT — issue 1117. PATH is not the
# whole story for a CMake cross build, and reading it as if it were cost two
# wrong diagnoses (issue 1113): this loop runs when activate.sh is SOURCED, so
# `nros setup --tool arm-none-eabi-gcc` in an already-active shell never reached
# PATH, and the build kept using the distro's 10.3.1 against a 13.2.rel1 pin
# with nothing printed. `cmake/toolchain/NanoRosCrossToolchain.cmake` now
# resolves the store DIRECTLY, so provisioning takes effect immediately, and it
# prints which compiler it picked and from where:
#
#   -- nano-ros: arm-none-eabi-gcc 13.2.1 via SDK store - <path> (pin 13.2.rel1)
#
# Its order is: -D/env NROS_<PREFIX>_PREFIX  ->  SDK store (newest version
# first)  ->  this PATH. Point it at a toolchain of your own with, e.g.,
# `NROS_ARM_NONE_EABI_PREFIX=/opt/gcc/bin/arm-none-eabi`; before issue 1117 the
# only such knob was NROS_RISCV64_PREFIX, documented nowhere outside its own
# file.
# `zenohd` was on this whitelist for the book's first-node flow (issue #204),
# back when `nros setup native` installed a router into the store. It is gone:
# RFC-0075 / phase-362 retired the vendored router, nothing provisions one, and
# the entry outlived it — a RETIRED store copy kept winning `command -v` for
# months (issue 0653). Removing the name from `scripts/sdk-path-tools.txt` is
# what stopped that; `just doctor` reports a leftover directory.
#
# The router is resolved, never found on PATH: `nros_zenohd_bin` reads
# NROS_RMW_ZENOHD, then AMENT_PREFIX_PATH, then $ROS_DISTRO under /opt/ros.
_nros_sdk="${NROS_HOME:-$HOME/.nros}/sdk"
if [ -d "$_nros_sdk" ]; then
    # Depth 3 is the versioned layout `nros setup` writes
    # (sdk/<tool>/<version>/bin); depth 2 is the older unversioned
    # sdk/<tool>/bin. Enumerated via the helper, NOT a glob: the store almost
    # never holds both layouts at once, and one empty pattern used to abort the
    # whole file under zsh (issue 0372).
    while IFS= read -r _nros_tcbin; do
        [ -n "$_nros_tcbin" ] || continue
        [ -d "$_nros_tcbin" ] || continue
        # Cross-gcc toolchains, plus build host tools the RTOS `make` invokes by
        # bare name (genromfs — the NuttX rv-virt etc/ ROMFS bake, Phase 194.3c),
        # and sccache (issue #74) — the justfile's `RUSTC_WRAPPER` + the zephyr
        # fixture CMake launcher auto-use it once it's on PATH.
        # espflash joined for issue 0486: `just esp32 build-qemu` packs its
        # flash image by invoking `espflash` by BARE NAME, so provisioning it
        # into the store is not enough — `nros setup --tool espflash` succeeded
        # and the pack step still skipped, because nothing put the store bin
        # dir on PATH. Same reason genromfs is here (an RTOS `make` calls it by
        # bare name).
        # issue 0663 — the list is DATA, in scripts/sdk-path-tools.txt, read by
        # this file and by activate.fish. It was a hand-written chain in both
        # and had already drifted (espflash in one, not the other), so the same
        # host behaved differently depending on the shell.
        _nros_want=0
        if _nros_dir_has_gcc "$_nros_tcbin"; then
            _nros_want=1
        else
            while IFS= read -r _nros_tool; do
                case "$_nros_tool" in ''|\#*) continue ;; esac
                if [ -x "$_nros_tcbin/$_nros_tool" ]; then
                    _nros_want=1
                    break
                fi
            done < "$_nros_root/scripts/sdk-path-tools.txt"
        fi
        if [ "$_nros_want" = "1" ]; then
            export PATH="$_nros_tcbin:$PATH"
        fi
    done <<EOF
$(_nros_bin_dirs "$_nros_sdk" 3; _nros_bin_dirs "$_nros_sdk" 2)
EOF
    unset _nros_tcbin
fi
unset _nros_sdk

# Pinned make (>=4.4, fifo jobserver server) and ninja (>=1.13, its client) —
# Phase 176. NO block here any more: both are ordinary SDK-store tools now
# (`nros setup --tool make` / `--tool ninja`), so the generic store loop above
# puts them on PATH via `scripts/sdk-path-tools.txt`. They used to live under
# `third-party/` behind two bespoke `just workspace install-*` recipes, which
# meant a second version pin (`MAKE_VERSION`/`NINJA_VERSION`) that could drift
# from the index's.

# Zephyr's `west` is NOT put on PATH here. It is resolved when a Zephyr lane
# needs it, by `nros_zephyr_activate` in scripts/build/zephyr-python.sh.
#
# This used to prepend `scripts/zephyr/.venv/bin` whenever that directory held
# an executable `west`, which was wrong twice over. The venv serves ONE lane but
# PATH is the whole session, so it also replaced `python3` for the 37
# `check-*.py` gates, colcon, rosidl_adapter and the cyclonedds descriptor
# codegen — the 4.4 line already refused to do that (just/zephyr-dev.just
# invokes west THROUGH the venv interpreter for exactly this reason) while the
# 3.7 line did it globally. And the test was presence, not usability: a venv's
# `bin/python3` is a symlink and its packages live in
# `lib/python3.<minor>/site-packages`, so a tree copied to a host with a
# different minor version passes `-x` and still cannot import west. Measured in
# the ROS distrobox: venv built by Arch 3.14, symlink resolving to Ubuntu 3.10,
# `west --version` dying on ImportError with the venv first on PATH.

# Project `.env` overrides (runtime config, buffer tuning, SDK paths)
# + the just/sdk-env.just SSoT defaults. direnv loads `.env` via
# `dotenv_if_exists`; outside direnv we shell-source it here.
if [ -f "$_nros_root/.env" ]; then
    set -a
    # shellcheck disable=SC1091
    . "$_nros_root/.env"
    set +a
fi
# shellcheck disable=SC1091
. "$_nros_root/scripts/sdk-env.sh"

unset _nros_root
unset -f _nros_bin_dirs _nros_dir_has_gcc 2>/dev/null || true
