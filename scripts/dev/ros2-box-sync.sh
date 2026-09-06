#!/usr/bin/env bash
# Mirror this checkout into the ROS distrobox's OWN tree (issues 0400, 0401).
#
# WHY A SECOND TREE AT ALL
#
# Host and box ran against one checkout, and every build artifact in it is
# glibc- and toolchain-specific. That single fact produced a different failure
# every time it surfaced:
#
#   - a host-built `build-script-build` re-run in the box   -> GLIBC_2.39 not found
#   - a host-configured CMake cache reused in the box       -> `sccache` not found,
#                                                              then `strlcpy` undefined
#   - a host-built `nros` / `nros-launch-resolve`           -> cannot exec / wrong libpython
#   - fixtures built into the box's redirected target dir   -> tests stat leaf paths and
#                                                              find the HOST's stale binary
#
# Each was fixed where it appeared. There were five of them in one session,
# because the premise — one tree, two incompatible toolchains — was never
# addressed. `CARGO_TARGET_DIR` redirection cannot address it either: the
# fixture contract is LEAF-RELATIVE by design (`examples/**/target-<rmw>/…`), so
# redirecting the box's cargo output moves fixtures away from where the tests
# look. That is issue 0401: the two mechanisms are mutually exclusive.
#
# Two trees make the question go away. The box compiles into its own checkout,
# at its own paths, and nothing it writes is visible to the host.
#
# WHY A MIRROR AND NOT `git worktree`
#
# A worktree cannot check out the branch the host already has, and — decisive —
# it carries only COMMITTED state. The normal loop here is edit, build in the
# box, test; a worktree would test the last commit instead of the edit. So this
# rsyncs the working tree, uncommitted changes included.
#
# `.git` comes along (1.3 GB): build scripts read it for source stamps, and the
# play_launch pin check (#409) runs `git -C` inside a submodule. Without it the
# box would stamp `unknown` everywhere and silently lose those guards.
#
# ONE-WAY, always. Edits belong on the host; the box is a build target. The
# mirror is refreshed before box work, never merged back.
set -euo pipefail

src="$(cd -P "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
dst="${NROS_BOX_TREE:-$(cd -P "$src/.." && pwd -P)/$(basename "$src")-box}"

if [ "$src" = "$dst" ]; then
    echo "ros2-box-sync: source and destination are the same path ($src)" >&2
    exit 1
fi

# Build outputs are per-tree by definition and are the whole point of the split:
# never copy them, and never delete the box's own (rsync protects excluded paths
# from --delete unless --delete-excluded is passed, which it deliberately is not).
#
# `.cargo-target-box` is the box's redirected dir from before this existed; it
# sits beside the checkout and must not be dragged in.
# EVERY build-output pattern here ends in `/`, which makes rsync match
# DIRECTORIES ONLY. That is not style — it is the fix for a bug this file has
# now produced three times, because an rsync pattern without a slash matches a
# BASENAME AT ANY DEPTH, files included:
#
#   `build`     matched `scripts/build/` (tracked source). The mirror lost
#               `scripts/build/cargo.sh` and `check-board-manifest-drift` failed.
#               Anchoring to `/build` fixed that instance.
#   `build-*`   matched `scripts/build/build-root.sh` — a tracked SCRIPT, not a
#               build dir. The mirror's fixture build died with
#               "scripts/build/build-root.sh: No such file or directory".
#               21 tracked files have a `build-*` basename (book pages, CI
#               workflows); all were silently absent from the mirror.
#
# Anchoring cures one instance and leaves the class: the intent was always
# "directories that hold build OUTPUT", and only `/` expresses that. A source
# file can then be named anything without vanishing from the box.
#
# `build/` is excluded at ANY depth, and the single tracked directory with that
# name is re-included AHEAD of it (rsync takes the first matching rule, so an
# include must precede the exclude it carves out of).
#
# The anchor this replaces was costing gigabytes. Nested `build/` dirs are leaf
# CMAKE OUTPUT — `examples/templates/multi-node-workspace-cpp/build` alone is
# 2.7 GB and there are 46 such dirs — so `/build/` mirrored the exact class of
# glibc-incompatible artifact the two-tree split exists to keep apart, and made
# a routine re-sync take tens of minutes. It was anchored only to dodge
# `scripts/build/`; `git ls-files` says that is the ONLY tracked `build/`, so
# naming it is the precise fix where the anchor was a blunt one that happened
# to work.
exclusions=(
    --include '/scripts/build/***'
    # …and the same carve-out for the vendored Zephyr, which is NOT tracked, so
    # the `git ls-files` reasoning above did not cover it. Zephyr ships four
    # source directories named `build`:
    #
    #   zephyr/scripts/build        zephyr/scripts/tests/build
    #   zephyr/doc/build            zephyr/share/sysbuild/build
    #
    # and without them the mirror's Zephyr is subtly broken rather than absent:
    # cmake gets far enough to print a version and a board, then dies with
    #
    #   python3: can't open file '…/zephyr/scripts/build/dir_is_writeable.py'
    #   CMake Error at …/boards.cmake:198: Error finding board: mps2
    #
    # — which reads as a bad board argument, not as a missing file. So the box
    # could never build ANY Zephyr target, and the reason was two directories
    # totalling 340 KB. Zephyr's own build OUTPUT is `build-<name>/`, still
    # excluded below, so this costs ~2 MB (`doc/build` is the bulk).
    #
    # `third-party/` needs no equivalent: it has no directory named `build` at
    # all, measured, and if one appears it will be output.
    --include '/zephyr-workspace/**/build/***'
    --exclude 'target/'
    --exclude 'target-*/'
    --exclude 'build/'
    # issue 1046 — `build-*/` is a BUILD-OUTPUT pattern and `packages/cli/
    # build-support/` is tracked SOURCE. Excluding it left the box unable to
    # compile `nros` at all:
    #
    #   error: couldn't read `nros-cli-core/../build-support/submodule_watch.rs`
    #
    # This is the FOURTH time this rule has eaten tracked source. The comment
    # above records the first three and the fix each time — anchor it, then make
    # every pattern end in `/` so it matches directories only. Both were right
    # and neither was sufficient: a tracked directory whose NAME begins with
    # `build-` still matches a directory-only pattern.
    #
    # Re-included AHEAD of the exclusion, the same way `/zephyr-workspace/**/
    # build/***` is above — rsync takes the first matching rule. There is
    # exactly one such directory in the tree today:
    #
    #   git ls-files | grep -oE '(^|/)build-[^/]+/' | sort -u
    #
    # `check-box-sync-covers-tracked-source` runs that sweep so a second one
    # cannot be added silently.
    --include '/packages/cli/build-support/***'
    --exclude 'build-*/'
    # issue 0925 — the GENERATED workspace manifests, which NAME the `build/`
    # members the rule above excludes.
    #
    # `examples/workspaces/<ws>/Cargo.toml` is a build artifact (phase-383 W3.a,
    # and its first line says "GENERATED ... DO NOT EDIT"). Its `members` are
    # `build/<platform>-<rmw>/<entry>` paths from the generating host. Copying
    # it while excluding `build/` hands the mirror a manifest whose members do
    # not exist, and `cargo metadata` fails on the first one — so EVERY fixture
    # build in the box died in `generate-bindings`, on a different workspace
    # each time. Four ~25 min cycles were spent on it.
    #
    # Excluded rather than repaired: `nros build` regenerates it in the mirror
    # from what the mirror actually has, which is the only correct member list
    # for that tree. None of the seven is tracked.
    --exclude '/examples/workspaces/*/Cargo.toml'
    --exclude '.cargo-target-box/'
    # HOST-BUILT TOOL BINARIES. Everything else under `third-party/` is SOURCE
    # and must be mirrored (zenoh-pico, cyclonedds, px4, …); these two are
    # compiled executables that `just workspace install-make` / `install-ninja`
    # produce for the tree they run in. Mirroring them is the very failure this
    # split exists to prevent — a make(1) built on Arch lands in Ubuntu 22.04
    # and dies before doing anything:
    #
    #   make: error while loading shared libraries: libguile-3.0.so.1
    #
    # and it dies at the TOP of the fixture build, where it reads as a broken
    # recipe rather than as a wrong-libc binary.
    #
    # The two excludes below are now VESTIGIAL and kept only so a leftover
    # directory from before the move is not mirrored: make and ninja are SDK
    # store tools (`~/.nros/sdk/...`), which is outside this tree and so was
    # never synced by this script anyway.
    #
    # Note what that changes, because it is not nothing: a distrobox shares
    # $HOME, so host and box now see ONE make binary rather than one per tree.
    # That is safe for exactly the reason the guile bug was fixed — the index's
    # source recipe configures `--without-guile`, so the binary carries no
    # distro-specific shared-library dependency and genuinely serves both
    # sides. If a future recipe change reintroduces such a dependency, this
    # becomes a two-distro breakage again, and `check-tier-preconditions.sh`
    # (which names `nros setup --tool make`) is what reports it.
    --exclude '/third-party/make/'
    --exclude '/third-party/ninja/'
    # Agent worktrees. `.claude/worktrees/<id>/` is a full source checkout per
    # concurrent agent session — nine of them here, host-session state that is
    # definitionally not box input. Only `target/` inside them was excluded, so
    # the mirror was taking a copy of the tree once per live agent and
    # `--delete` churning them as sessions came and went.
    --exclude '/.claude/worktrees/'
    --exclude '/tmp/'
    --exclude '/test-logs/'
    --exclude 'node_modules/'
    # issue 0401 follow-up — path-carrying GENERATED files must NOT be mirrored.
    # `nros sync` writes ABSOLUTE paths (RFC-0048 W9), so a copied
    # `nros-patch.toml` / leaf `.cargo/config.toml` points at the SOURCE tree.
    # Worse than useless: it half-works. The box then has some leaves rewritten
    # to box paths and a central patch still naming the host, and
    # `check-dep-chain` fails on four boards with `no matching package named
    # \`nros\`` — a resolution error that says nothing about mirroring.
    #
    # Only the ROOT `nros-patch.toml` is excluded, and only because it is
    # gitignored and purely generated. Leaf `.cargo/config.toml` files are NOT:
    # 63 of them are TRACKED source, and excluding the name globally deleted
    # those from the mirror, which broke resolution far worse (`no matching
    # package named `nros`` with no config present at all). They are mirrored,
    # and the box's own `nros sync` rewrites the absolute paths inside them.
    --exclude '/nros-patch.toml'
    # The box-owned marker. Excluded so `--delete` cannot remove it (the SOURCE
    # tree has no such file, so without this rsync deletes it as extraneous).
    --exclude '/.nros-box-tree'
)

mkdir -p "$dst"

# Write the marker BEFORE the transfer, not after.
#
# `ros2-box-env.sh` reads it to decide this tree is box-OWNED and must NOT get
# the CARGO_TARGET_DIR redirect. It used to be touched after rsync, which is
# correct only when rsync finishes: `set -e` means ANY interrupted sync — a
# timeout, a Ctrl-C, a full disk — exits before the touch and leaves a tree that
# LOOKS like a plain shared checkout.
#
# That failure is silent and expensive. The redirect comes back, fixtures build
# to a path the leaf-relative test resolver never stats, and the run fails far
# away from the cause (issues 0400/0401). Measured: one killed sync, then four
# wrong hypotheses before anyone thought to print $CARGO_TARGET_DIR.
#
# Marker first + excluded from --delete means an interrupted sync leaves a tree
# that is merely INCOMPLETE — which the next run fixes — rather than one that is
# complete-looking and mis-targeted.
touch "$dst/.nros-box-tree"

echo "ros2-box-sync: $src -> $dst"
rsync -a --delete "${exclusions[@]}" "$src/" "$dst/"

# Re-touched after a SUCCESSFUL transfer as well. Belt and braces: the exclusion
# above is what actually preserves it (a marker deleted by `--delete` was the
# original bug — the box silently fell back to the redirect, everything still
# built, and the symptom surfaced far away as `check-c` compiling against
# headers the redirected build had written to the OLD location), and this second
# touch costs nothing while covering any future edit that drops the exclusion.
touch "$dst/.nros-box-tree"

# `--delete` CANNOT remove a directory whose surviving content is EXCLUDED.
#
# rsync protects excluded paths from deletion — deliberately, because that is
# what keeps the box's own `target/` and `build/` output alive across a re-sync.
# The cost is that a directory RETIRED upstream never leaves the mirror once it
# holds any build output: rsync deletes the tracked files inside it, then stops
# at the excluded ones and reports
#
#   cannot delete non-empty directory: examples/workspaces/ws-bridge-rust
#
# — a warning, on stderr, in the middle of a long transfer, and the sync still
# exits 0. Measured on 2026-08-19: eleven retired `ws-*` workspaces (phase-331
# W3 deleted them; CLAUDE.md says "verified zero remain") plus
# `examples/stm32f4`, retired by phase-337 W7.a and still holding 1.2 GB of
# build output. The tree the box builds against therefore disagreed with the
# tree the host committed, in exactly the direction that makes a museum binary:
# the artifact is present, nothing rebuilds it, and no gate looks for a
# directory that is not supposed to exist.
#
# So: after the transfer, delete every mirror directory whose SOURCE
# counterpart is gone. Output dirs are skipped by NAME (they are the box's own
# and legitimately have no counterpart) and not descended into — the box's
# `build/cargo-fixtures/**` is thousands of paths the source will never have.
# The tops this script excludes outright are box-owned for the same reason.
_nros_prune_orphans() {
    python3 - "$src" "$dst" <<'PRUNE_PY'
import os, shutil, sys

src, dst = sys.argv[1], sys.argv[2]

# Box-owned by construction: this script excludes them from the transfer, so
# the source has no counterpart and never will.
EXEMPT = {"tmp", "test-logs", ".cargo-target-box",
          os.path.join("third-party", "make"), os.path.join("third-party", "ninja")}


def is_output(name):
    return name in ("target", "build") or name.startswith(("target-", "build-"))


removed, freed = [], 0
stack = [""]
while stack:
    rel = stack.pop()
    try:
        entries = sorted(os.listdir(os.path.join(dst, rel)))
    except OSError:
        continue
    for name in entries:
        if name == ".git" or is_output(name):
            continue
        child = os.path.join(rel, name) if rel else name
        if child in EXEMPT:
            continue
        path = os.path.join(dst, child)
        if not os.path.isdir(path) or os.path.islink(path):
            continue
        if os.path.exists(os.path.join(src, child)):
            stack.append(child)
            continue
        # Retired upstream. Everything under it is build output rsync could not
        # reach; the source says this directory should not exist at all.
        for walk_root, _, files in os.walk(path):
            for f in files:
                try:
                    freed += os.lstat(os.path.join(walk_root, f)).st_size
                except OSError:
                    pass
        shutil.rmtree(path, ignore_errors=True)
        removed.append(child)

for r in removed:
    print(f"ros2-box-sync: pruning retired directory (absent from source): {r}")
if removed:
    print(f"ros2-box-sync: pruned {len(removed)} retired director"
          f"{'y' if len(removed) == 1 else 'ies'}, {freed / 1e9:.2f} GB")
PRUNE_PY
}

_nros_prune_orphans

# A CMakeCache.txt records the ABSOLUTE source and build directories it was
# generated for. One copied from the source tree therefore points at the source
# tree, and cmake refuses outright:
#
#   CMake Error: The current CMakeCache.txt directory <dst>/…/build/CMakeCache.txt
#   is different than the directory <src>/…/build where CMakeCache.txt was created.
#
# Excluded paths are protected from `--delete`, which is right for the box's OWN
# build output but means anything copied under an EARLIER, wronger exclusion set
# stays forever. That is how these arrived: a window where the rule was anchored
# to `/build` and nested build dirs were mirrored.
#
# Remove only caches that provably belong to another tree — the ones naming a
# home directory outside this destination. The box's own caches name the box.
while IFS= read -r cache; do
    [ -n "$cache" ] || continue
    home="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$cache" 2>/dev/null | head -1)"
    case "$home" in
        "$dst"/*|"") continue ;;
    esac
    echo "ros2-box-sync: dropping foreign cmake cache ($home) -> $(dirname "$cache")"
    rm -rf "$(dirname "$cache")"
done < <(find "$dst" -name CMakeCache.txt 2>/dev/null)

# `nros sync` writes ABSOLUTE paths into each leaf's `.cargo/config.toml`
# (RFC-0048 W9), so a mirrored leaf still points at the SOURCE tree and cargo
# resolves `nros` against crates.io instead of the checkout. This is the
# documented "moved checkout -> re-run `nros sync`" rule; it applies to a mirror
# for exactly the same reason.
echo "ros2-box-sync: run \`just generate-bindings\` (or \`nros sync <ws>\`) inside the box before"
echo "               building: the path-carrying generated files are deliberately NOT mirrored,"
echo "               so the box writes its own with box paths."
echo ""
echo "ros2-box-sync: done. Enter the box against the MIRROR:"
echo "    DBX_CONTAINER_MANAGER=docker distrobox enter ros2 -- \\"
echo "        bash -c 'cd $dst && . scripts/dev/ros2-box-env.sh && <cmd>'"
