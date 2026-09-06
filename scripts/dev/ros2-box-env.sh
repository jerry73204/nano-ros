# Per-box environment for a ROS 2 distrobox whose $HOME is shared with the host
# (see docs/development/ros2-on-non-ubuntu.md). Source before any nano-ros
# command INSIDE the box:
#
#     distrobox enter ros2 -- bash -c '. scripts/dev/ros2-box-env.sh && <cmd>'
#
# On a host with BOTH docker and podman, prefix that with
# `DBX_CONTAINER_MANAGER=docker` (or podman) — distrobox prefers podman when it
# is present, and against a docker-created box it reports `no such container`
# and offers to create an empty Fedora one instead of finding yours.
#
# GLIBC DIRECTION IS THE WHOLE STORY. glibc is backward compatible: a binary
# linked against the box's OLDER glibc runs on a newer host, never the reverse.
# So artifacts built IN THE BOX work on both sides, and everything the host
# built is unusable here. Each override below exists because the host may have
# already written that location.
#
#   CARGO_TARGET_DIR   A shared target dir does not merely churn — it FAILS:
#                      cargo re-runs cached build-script EXECUTABLES, and a
#                      host-built `build-script-build` dies here with
#                      `GLIBC_2.xx not found`. Verified, not assumed.
#   NROS_HOME          A shared SDK store reports the host's zenohd "present" at
#                      the version the index pins, then hands the box a binary
#                      it cannot exec. The box provisions its own.
#   CARGO_INSTALL_ROOT ~/.cargo/bin holds host-built tools (`just`), which fail
#                      the same way; keeping the box's copies elsewhere leaves
#                      the host's tools intact.
#
# Shared safely: ~/.rustup (toolchains target an old glibc) and the cargo
# registry/git caches (sources, not objects).

_nros_box_root="$(cd -P "$(dirname "${BASH_SOURCE[0]:-$0}")/../.." && pwd -P)"

# distrobox mounts the whole host filesystem at /run/host AND translates the
# entry cwd into it, so the SAME checkout is reachable inside the box by two
# paths — /run/host/mnt/… and the bind-mounted /mnt/… — and which one you land
# on depends on how you entered. Either is a working tree; only one matches the
# host's own absolute path, and the mismatch is the issue-0375 hazard for real:
# `nros sync` writes absolute paths, and cargo/cmake caches key on them, so a
# box build under /run/host/… and a host build under /mnt/… silently disagree.
# Strip the prefix when the stripped path is the same checkout.
case "$_nros_box_root" in
    /run/host/*)
        _nros_box_stripped="${_nros_box_root#/run/host}"
        if [ -d "$_nros_box_stripped/packages/cli" ]; then
            _nros_box_root="$_nros_box_stripped"
            cd "$_nros_box_root" || return 1
        fi
        unset _nros_box_stripped
        ;;
esac

export NROS_HOME="${NROS_HOME:-$HOME/.nros-box}"
# Beside the CHECKOUT, not in $HOME. A second full target tree is tens of GB
# (40 GB when this bit here), and $HOME is commonly on a small root filesystem
# while the checkout sits on a big data disk — which is exactly how this box
# filled a 235 GB root to 0 bytes and killed a tier run with
# `No space left on device`, in a lane that had nothing to do with disk.
# Keeping it next to the repo puts the box's build tree on the same filesystem
# that already holds the host's `target/`, so whoever sized that disk sized
# this too. Override CARGO_TARGET_DIR to move it elsewhere.
# issues 0400/0401 — CARGO_TARGET_DIR is only needed while the box SHARES the
# host's checkout. `scripts/dev/ros2-box-sync.sh` gives the box its own tree, and
# then redirecting is actively harmful: the fixture contract is LEAF-RELATIVE
# (`examples/**/target-<rmw>/…`), so moving cargo's output puts fixtures where
# the tests do not look — the build reports success and every test fails STALE
# against whatever the host left behind (0401).
#
# So: in a box-owned tree, do NOT redirect. Cargo writes to the leaf paths the
# fixture contract names, inside a tree the host never touches, and the
# host/box artifact hazards (0400) have no shared tree to occur in.
#
# Sharing the host tree is still supported — that is what a checkout with no
# `.nros-box-tree` marker means — and keeps the redirect, because there the
# alternative is host-built build scripts dying on GLIBC.
#
# issue 0759 — that support is now REFUSED by default, because the redirect
# never covered the paths the fixture builds actually use. They pass their own
# `--target-dir` (the RFC-0070 cache root `build/cargo-fixtures/<family>/`) or
# are leaf-relative BY CONTRACT (`examples/**/target-fixtures/<plat>/`, issue
# 0401 — those cannot be redirected at all). Host and box then share artifacts
# built by DIFFERENT compilers against a different libc, with nothing anywhere
# checking that the two toolchains agree.
#
# The failure is not merely that it breaks. A cargo unit that is a fingerprint
# HIT is reused without ever being EXECUTED, so a box run over a
# host-populated tree reports every fixture built — and the first source edit
# after that fails on a host binary: the build script (`GLIBC_2.39 not found`),
# then the proc-macro `.so`, which the compiler dlopens and reports AT A SOURCE
# LINE, so it reads as a compile error in code that is fine. The green run is
# the harmful part: on 2026-08-22 it made a mutation test rebuild nothing and
# the unchanged museum binary PASS, i.e. deleting the thing under test appeared
# not to matter.
#
# So: if the box is in play, EVERY job runs in the box, against the box's own
# tree. `scripts/dev/ros2-box-sync.sh` makes one.
if [ -f "$_nros_box_root/.nros-box-tree" ]; then
    unset CARGO_TARGET_DIR
elif [ -n "${NROS_ALLOW_SHARED_BOX_TREE:-}" ]; then
    echo "box: WARNING — shared host tree by request (NROS_ALLOW_SHARED_BOX_TREE)." >&2
    echo "     Host and box artifacts collide under build/cargo-fixtures and the" >&2
    echo "     leaf target-fixtures dirs; a build can succeed and never REBUILD" >&2
    echo "     (issue 0759). Wipe those before switching sides." >&2
    export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$(cd -P "$_nros_box_root/.." && pwd -P)/.cargo-target-box}"
else
    echo "box: REFUSING to run against the host's own checkout (issue 0759)." >&2
    echo "       tree: $_nros_box_root  (no .nros-box-tree marker)" >&2
    echo "" >&2
    echo "  Host and box have different compilers and a different libc, and they" >&2
    echo "  would share build artifacts here — nothing guarantees the two" >&2
    echo "  toolchains are compatible. A build may even SUCCEED, by reusing" >&2
    echo "  cached units it never executes, and fail only on the next edit." >&2
    echo "" >&2
    echo "  Give the box its own tree, then work there:" >&2
    echo "      scripts/dev/ros2-box-sync.sh" >&2
    echo "      cd $(cd -P "$_nros_box_root/.." && pwd -P)/$(basename "$_nros_box_root")-box" >&2
    echo "" >&2
    echo "  Deliberate exception: NROS_ALLOW_SHARED_BOX_TREE=1, and say why." >&2
    return 1 2>/dev/null || exit 1
fi
export CARGO_INSTALL_ROOT="${CARGO_INSTALL_ROOT:-$HOME/.local-box}"

# `nros sync` runs the launch resolver by ABSOLUTE path (issue 0285) and its
# default is `packages/cli/nros-launch-resolve/target/release/…` — one location
# for two incompatible binaries. The host links libpython3.14, the box
# libpython3.10, and NEITHER loads on the other side, so whoever built last
# broke the other (issue 0400). The CLI already honours an explicit override,
# so point it at the box's own build, which lands under CARGO_TARGET_DIR above.
if [ -n "${CARGO_TARGET_DIR:-}" ]; then
    export NROS_LAUNCH_RESOLVE="${NROS_LAUNCH_RESOLVE:-$CARGO_TARGET_DIR/release/nros-launch-resolve}"
fi
# In a box-owned tree the resolver builds to its normal in-tree path and the CLI
# finds it there — no override, and #409's pin check now catches a mismatch
# rather than relying on this env var being right.

cd "$_nros_box_root" || return 1

# `nros` discovery expects packages/cli/target/release/nros — activate.sh puts
# that directory on PATH and cmake's find_program HINTS look there directly, so
# CARGO_TARGET_DIR alone hides the box's CLI from every consumer. Publish the
# box build to that path; being the older-glibc binary it keeps working on the
# host too. A host-side `cargo build` of the CLI overwrites it with a host
# binary and breaks the box again — re-run this after that happens.
# The root is exported, NOT read from `$_nros_box_root`: that variable is unset
# at the end of this file, so a function closing over it silently resolved to
# "" and installed to /packages/cli/… — which fails, while the second install
# still returned 0 and the function reported success.
export NROS_BOX_REPO="$_nros_box_root"

nros_box_publish() {
    # Where cargo ACTUALLY put it, which depends on whether this tree is
    # box-owned. In a box-owned tree CARGO_TARGET_DIR is deliberately unset
    # (issues 0400/0401), so the old unconditional "$CARGO_TARGET_DIR/release"
    # expanded to the absolute path "/release/nros" and this function refused
    # with "box: no CLI at /release/nros" — right after a build that succeeded.
    # The two-tree fix unset the variable and did not revisit its readers.
    local built
    if [ -n "${CARGO_TARGET_DIR:-}" ]; then
        built="$CARGO_TARGET_DIR/release/nros"
    else
        built="$NROS_BOX_REPO/packages/cli/target/release/nros"
    fi
    if [ ! -x "$built" ]; then
        echo "box: no CLI at $built — build it first:" >&2
        echo "  cargo build --release --manifest-path packages/cli/Cargo.toml --bin nros" >&2
        return 1
    fi
    mkdir -p "$CARGO_INSTALL_ROOT/bin" "$NROS_BOX_REPO/packages/cli/target/release"
    # In a box-owned tree the build path IS the publish path, so skip the
    # self-copy — `install` onto itself truncates the source before reading it.
    if [ "$built" != "$NROS_BOX_REPO/packages/cli/target/release/nros" ]; then
        install -m755 "$built" "$NROS_BOX_REPO/packages/cli/target/release/nros" || return 1
    fi
    install -m755 "$built" "$CARGO_INSTALL_ROOT/bin/nros" || return 1
    echo "box CLI published: $NROS_BOX_REPO/packages/cli/target/release/nros + $CARGO_INSTALL_ROOT/bin/nros"
}

# issue 1144 — SEED THE HOST'S CARGO BIN AT THE TAIL FIRST, or the prepend
# below is undone by the very file it is ordered against. activate.sh prepends
# `$HOME/.cargo/bin` *conditionally*:
#
#     if [ -d "$HOME/.cargo/bin" ]; then
#         case ":$PATH:" in
#             *":$HOME/.cargo/bin:"*) ;;
#             *) export PATH="$HOME/.cargo/bin:$PATH" ;;
#         esac
#     fi
#
# In a LOGIN shell the container profile already put that dir on PATH, the
# guard skips, and the box's dir stays in front — which is the only case the
# comment below was ever tested in. In a NON-login shell (`podman exec … bash
# -c`, the fast way to drive the box) it is absent, so activate.sh prepends the
# HOST's glibc-2.39 `just` in front of the box's and it dies with
# `GLIBC_2.39 not found`, naming neither PATH nor the box.
#
# Putting it on the tail ourselves makes that guard a no-op BY CONSTRUCTION, in
# both shell kinds, while keeping `cargo`/`rustup` reachable (rustup's own shims
# are old-glibc and do run here — it is the cargo-INSTALLED tools beside them
# that do not). It is the precondition activate.sh's guard was already written
# against; we now establish it instead of assuming it.
#
# Not "prepend the box dir again AFTER activate.sh": activate.sh also prepends
# `packages/cli/target/release` (unconditionally), and that ordering is
# deliberate — a later in-box `just setup-cli` rewrites the CLI there and NOT
# the `$CARGO_INSTALL_ROOT/bin/nros` copy `nros_box_publish` made, so a box dir
# in front would shadow a fresh CLI with a stale one. Seeding the tail fixes
# the ordering without disturbing any of activate.sh's own prepends.
if [ -d "$HOME/.cargo/bin" ]; then
    case ":$PATH:" in
        *":$HOME/.cargo/bin:"*) ;;
        *) export PATH="$PATH:$HOME/.cargo/bin" ;;
    esac
fi

# BEFORE activate.sh: it sources scripts/sdk-env.sh, which shells out to `just`,
# and the host's ~/.cargo/bin/just would otherwise win the PATH race and print
# `GLIBC_2.xx not found`. activate.sh prepends packages/cli/target/release after
# this, which is correct — nros_box_publish put a box binary there.
export PATH="$CARGO_INSTALL_ROOT/bin:$PATH"

# shellcheck disable=SC1091
. "$_nros_box_root/activate.sh"

# ASSERT the ordering rather than reasoning about it (issue 1144). Every tool
# the box installed for itself must beat the host copy of the same name: the
# host's are glibc-2.39 binaries that cannot exec here, and their failure names
# the loader, never PATH. Keyed on the HOST dir, not on the box dir, because
# `nros` legitimately resolves to `packages/cli/target/release/nros` (see
# above) — what is never legitimate is resolving into `$HOME/.cargo/bin`.
nros_box_check_path() {
    [ -d "${CARGO_INSTALL_ROOT:-}/bin" ] || return 0
    [ -d "$HOME/.cargo/bin" ] || return 0
    # A box that deliberately installs into the host's dir has nothing to check.
    [ "$CARGO_INSTALL_ROOT/bin" != "$HOME/.cargo/bin" ] || return 0

    local tool name resolved rc=0
    for tool in "$CARGO_INSTALL_ROOT"/bin/*; do
        [ -x "$tool" ] || continue
        name="$(basename "$tool")"
        resolved="$(command -v "$name" 2>/dev/null)" || continue
        case "$resolved" in
            "$HOME/.cargo/bin/"*)
                echo "box: PATH is wrong — \`$name\` resolves to the HOST binary" >&2
                echo "       $resolved" >&2
                echo "     shadowing the box's $tool" >&2
                echo "     (host tools are glibc-2.39 and die here as" >&2
                echo "      \`GLIBC_2.39 not found\`; issue 1144)" >&2
                rc=1
                ;;
        esac
    done
    return "$rc"
}

if ! nros_box_check_path; then
    unset _nros_box_root
    return 1 2>/dev/null || exit 1
fi

unset _nros_box_root
