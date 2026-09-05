#!/usr/bin/env bash
# A leaf `.cargo/config.toml` is tracked if and only if it carries content
# `nros sync` cannot reproduce.
#
# Sync writes these files (RFC-0048 W9): an `include = ["…/nros-patch.toml"]`
# line pointing at a gitignored, host-specific central file, and a
# `[patch.crates-io]` block of `# nros-managed` path entries. A config holding
# only that is a pure artifact — recreated by `nros sync` in any checkout — and
# committing it means a file that churns on every sync as the patch set moves.
#
# Many configs also carry hand-authored content: `[build] target` for a
# cross-compiled leaf, a QEMU `runner`, linker `rustflags`. Sync refreshes the
# patch block INSIDE those, but nothing can regenerate the rest. They must stay
# tracked, or a fresh clone loses the target selection and the leaf builds for
# the host, or not at all.
#
# `**/.gitignore` cannot tell the two apart — they share a filename and sit in
# the same directories — so the ignore rule is blanket and this gate supplies
# the discrimination it cannot. Without it, a new embedded example's
# hand-authored config would be silently ignored: fine on the machine that
# wrote it, missing for everyone else.
set -uo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# issue 0726 — `grep -q … || return 1` reads a grep that FAILED as "no include
# line", i.e. as evidence about the leaf. `nros_grep_q` exits 2 there.
#
# Two conversions need care and both are noted at the site: the file-existence
# tolerance the old `2>/dev/null` provided is now an explicit `[ -r ]` test (so
# an absent config is still "not a finding" while an unreadable one is fatal),
# and the two pipelines are split so the helper runs in THIS shell — a pipeline
# segment is a subshell where `exit 2` would end only the segment and hand the
# caller back a status it cannot tell from "no match".
#
# Note `projection_missing` is called once with `2>&1 >/dev/null` (the lazy
# header probe) before the real call, so a FATAL message from that first call is
# discarded. The exit status still ends the gate, which is the property that
# matters; the second, unredirected call is simply never reached.
# shellcheck source=scripts/lib/grep-q.sh
source scripts/lib/grep-q.sh

# The canonical build-tree prune list (phase-300 W3). Without it the walk
# descends into `build-zenoh/_deps/corrosion-src/test/…`, where vendored
# corrosion ships its own hand-authored `.cargo/config.toml` fixtures — none of
# which are ours to track.
# shellcheck source=scripts/build/prune-dirs.sh
source scripts/build/prune-dirs.sh

# Content beyond what sync writes: anything that is not blank, the include line,
# the patch table header, an `# nros-managed` entry, or sync's own comment decor.
#
# Issue 0587 — a COMMENT counts as authored content, and excluding `^\s*#`
# wholesale is what made this gate demand the deletion of documentation. Six
# `examples/threadx-linux/rust/*/.cargo/config.toml` files were reported as
# "pure sync output" with a `git rm --cached` remedy, when the comment IS the
# file: it records issue 0582's finding (why there is deliberately NO
# `[build] target` — the literal `x86_64-unknown-linux-gnu` read as a host pin
# on an x86 machine and a CROSS COMPILE everywhere else) and where the artifact
# actually lands. `nros sync` will never rewrite that paragraph, so by this
# repo's own rule — the authored half stays tracked because a clone cannot
# regenerate it — it is authored content.
#
# The predicate needs no judgement about which comments "look important",
# because sync's output is FIXED and knowable: it emits exactly the BEGIN/END
# decor below plus a trailing `# nros-managed` on each patch row (see
# `nros-cli-core/src/cmd/ws.rs`). Any other comment came from a human.
has_authored_content() {
    grep -qvE '^\s*$|^\s*# === (BEGIN|END) nros-managed \[patch\.crates-io\] ===\s*$|^include = |^\[patch\.crates-io\]|# nros-managed\s*$' "$1"
}

# Issue 0457 — a TRACKED config must not patch a `generated/` message crate
# whose tree is not itself committed.
#
# Those crates are produced by `nros sync` from the USER's ament install, so the
# row is host-derived: it names a path that does not exist in a fresh clone, and
# cargo fails on a `[patch]` (or path dep) pointing at a missing manifest. This
# is the same rule that already keeps `generated/` and the leaf `Cargo.lock`
# beside it out of git — a tracked artifact must not assert which ROS install
# built it.
#
# The exception is exact and narrow: `packages/interfaces/*` commit their
# generated msg crates on purpose (core crates need those messages BEFORE any
# codegen runs), so a config patching a COMMITTED `generated/` tree is fine.
# Hence the test is "is that tree tracked", not "does the path say generated".
#
# Sync itself can no longer create this: its managed block now lives in the
# gitignored `nros-managed-patch.toml` sidecar. What this catches is a
# HAND-written row, which is how all three offenders got here.
ament_rows_tracked=0
has_uncommitted_generated_patch() {
    local cfg="$1" leaf rel
    leaf="$(dirname "$(dirname "$cfg")")"
    # Every `path = ".../generated/<crate>"` value in the file, sub-table form
    # (`[patch.crates-io.x]` + `path = …`) and inline form alike.
    while IFS= read -r rel; do
        [ -n "$rel" ] || continue
        # Patch paths resolve against the leaf dir (cargo's invocation cwd).
        if [ -z "$(git ls-files -- "$leaf/$rel" 2>/dev/null)" ]; then
            echo "    $cfg" >&2
            echo "      patches '$rel', which is not committed" >&2
            return 0
        fi
    done < <(grep -oE 'path *= *"[^"]*generated/[^"]*"' "$cfg" | sed -E 's/.*"([^"]*)".*/\1/')
    return 1
}

# Issue 0463 — every `include` entry in a TRACKED config must name a target some
# generator owns.
#
# Cargo raises a missing `include` as a HARD error while PARSING the manifest,
# so the leaf becomes unreadable, not merely unbuildable: `cargo metadata`,
# `cargo tree` and every gate that walks the leaf fail with it. (Both #272 and
# #457 recorded the opposite — "cargo ignores a missing include SILENTLY" — and
# built on it; measured on cargo 1.97.1 it does not.)
#
# Exactly three targets are generated: the central `nros-patch.toml`, the
# per-leaf `nros-managed-patch.toml` sidecar, and the per-leaf `nros-board.toml`
# board-`cargo_config` projection (phase-341 W2). The first two are gitignored,
# so a fresh clone has neither and `_require-leaf-includes` sends the developer
# to `nros sync`; the third is COMMITTED precisely so a clone can LINK without
# running sync. An include naming anything ELSE has no generator at all, so no
# sync run will ever satisfy it — that leaf is bricked for everyone, forever.
# This gate catches that at authoring time, which the sync-time check in
# `cmd/ws.rs` cannot: that one only validates the central entry it just wrote.
orphan_includes=0
untracked_projection=0

# issue 0559 / phase-341 — a TRACKED config that `include`s `nros-board.toml`
# must have that projection COMMITTED beside it.
#
# The comment above states the rule ("the third is COMMITTED precisely so a
# clone can LINK without running sync") and nothing enforced it, which is how
# `examples/threadx-linux/rust/talker` shipped with the include and without the
# file: `nros sync` rewrote both on every fixture build, so the tree was dirty
# after every run and `git pull --rebase` refused until you discarded them.
#
# This became load-bearing when `**/.cargo/nros-board.toml` was gitignored: the
# ignore is what silences the ~39 projections whose leaf config is ITSELF
# generated (nothing tracked references those), and gitignore cannot express
# "…unless a tracked config includes you". This arm is that condition, the same
# shape `check-cargo-config-tracked` already uses to stop the blanket
# `config.toml` ignore swallowing a hand-authored one.
projection_missing() {
    local cfg="$1" dir
    dir="$(dirname "$cfg")"
    [ -r "$cfg" ] || return 1
    nros_grep_q '^include' "$cfg" || return 1
    local includes
    includes="$(sed -n '/^\[/q;p' "$cfg" | grep -oE '^include *= *\[[^]]*\]')" || true
    nros_grep_q 'nros-board\.toml' <<<"$includes" || return 1
    git ls-files --error-unmatch "$dir/nros-board.toml" >/dev/null 2>&1 && return 1
    echo "    $cfg" >&2
    echo "      include -> 'nros-board.toml' — but $dir/nros-board.toml is NOT committed" >&2
    return 0
}

# phase-351 W3 — the DUAL of `projection_missing`, and the reason a config can
# be "pure sync output" and still have to stay tracked.
#
# W3 moved the last hand-authored rows out of twelve embedded leaves (the NuttX
# `libc` patch, the ThreadX `[env]` block) into their board descriptor, which
# `nros sync` now delivers. Those configs then held nothing but the `include`
# line and the managed patch block — "pure artifact" by the test above, so the
# gate demanded they be untracked.
#
# That would be wrong, and in exactly the direction issue 0559 was about. The
# include names `nros-board.toml`, which IS committed, and which carries the
# leaf's `[build] target` and link rustflags. Drop the config from git and a
# fresh clone has the projection with nothing reaching it: the leaf builds for
# the host, or not at all — the failure the header at the top of this file
# describes. Sync could recreate the line, but "run sync first" is precisely
# what committing the projection exists to avoid.
#
# So: including a COMMITTED projection is content, even though sync wrote it.
#
# Used in ONE direction only — it stops the gate demanding these be UNTRACKED.
# It deliberately does not demand the converse (track every config that includes
# a committed projection): a WORKSPACE MEMBER's `.cargo/config.toml` is never
# read for the builds we run, because cargo discovers config from the invocation
# CWD upward and corrosion invokes cargo from the workspace root
# (`workspace_toml_dir`). Demanding those be committed would be policy about a
# file that does not govern — phase-349 W2.0 measured exactly that.
includes_committed_projection() {
    local cfg="$1" dir
    dir="$(dirname "$cfg")"
    local includes
    includes="$(sed -n '/^\[/q;p' "$cfg" | grep -oE '^include *= *\[[^]]*\]')" || true
    nros_grep_q 'nros-board\.toml' <<<"$includes" || return 1
    git ls-files --error-unmatch "$dir/nros-board.toml" >/dev/null 2>&1
}

has_orphan_include() {
    local cfg="$1" entry base
    # Only the top-level key: scanning past the first table header would let a
    # `[target.…]` value containing the word masquerade as one.
    while IFS= read -r entry; do
        [ -n "$entry" ] || continue
        base="$(basename "$entry")"
        case "$base" in
            # `nros-managed-env.toml` is issue 0827's per-leaf derived `[env]`
            # sidecar: same generator, same gitignore, same
            # appear-and-disappear-together rule as the patch one.
            nros-patch.toml | nros-managed-patch.toml | nros-board.toml \
                | nros-managed-env.toml) continue ;;
        esac
        echo "    $cfg" >&2
        echo "      include -> '$entry' — no generator writes this" >&2
        return 0
    done < <(sed -n '/^\[/q;p' "$cfg" | grep -oE '^include *= *\[[^]]*\]' |
        grep -oE '"[^"]*"' | tr -d '"')
    return 1
}

untracked_authored=0
tracked_pure=0
orphan_residue=0
orphan_dirs=()

# Issue 1074 — a config whose PACKAGE no longer exists is residue, not a finding.
#
# `git rm` cannot delete a gitignored file, and every leaf `.cargo/config.toml`
# is gitignored (`.gitignore:111`, blanket, which is the whole reason this gate
# exists). So deleting a package leaves its config on disk, in a directory that
# is no longer a package — and the walk below, which is a FILESYSTEM walk, finds
# it and reports it as an untracked authored config.
#
# phase-338 W2 (`ab486a8db`) collapsed 18 `-entry` example packages into their
# node packages. Twelve of the eighteen corpses were still reporting here a
# month later, and the remedy the gate printed — track it — would have
# RESURRECTED config files for packages the tree had deliberately removed.
#
# The discriminator is "does anything in this package's directory survive in
# git". Nothing does, for a deleted package; something always does for a live
# one, because a package needs at least a manifest. Cheap, and it cannot
# mistake a live leaf for a dead one.
#
# NOT VISIBLE ON CI, which is why it lasted: a fresh clone has no residue, so
# this only ever fires on a machine that once built the old packages.
package_is_deleted() {
    local dir
    dir="$(dirname "$(dirname "$1")")"   # <pkg>/.cargo/config.toml -> <pkg>
    [ -z "$(git ls-files "$dir" 2>/dev/null)" ]
}

while IFS= read -r -d '' cfg; do
    cfg="${cfg#./}"
    if package_is_deleted "$cfg"; then
        orphan_residue=$((orphan_residue + 1))
        orphan_dirs+=("$(dirname "$(dirname "$cfg")")")
        continue
    fi
    tracked=0
    git ls-files --error-unmatch "$cfg" >/dev/null 2>&1 && tracked=1


    if [ "$tracked" -eq 1 ]; then
        if [ "$ament_rows_tracked" -eq 0 ]; then
            # Header printed lazily, before the first offender's detail lines.
            if has_uncommitted_generated_patch "$cfg" >/dev/null 2>&1; then
                echo "check-cargo-config-tracked: tracked config patches an UNCOMMITTED generated/ tree:" >&2
            fi
        fi
        if has_uncommitted_generated_patch "$cfg"; then
            ament_rows_tracked=$((ament_rows_tracked + 1))
        fi

        if [ "$orphan_includes" -eq 0 ]; then
            if has_orphan_include "$cfg" >/dev/null 2>&1; then
                echo "check-cargo-config-tracked: tracked config includes a file NO generator writes:" >&2
            fi
        fi
        if has_orphan_include "$cfg"; then
            orphan_includes=$((orphan_includes + 1))
        fi

        if [ "$untracked_projection" -eq 0 ]; then
            if projection_missing "$cfg" >/dev/null 2>&1; then
                echo "check-cargo-config-tracked: tracked config includes an UNCOMMITTED board projection:" >&2
            fi
        fi
        if projection_missing "$cfg"; then
            untracked_projection=$((untracked_projection + 1))
        fi
    fi

    if has_authored_content "$cfg"; then
        if [ "$tracked" -eq 0 ]; then
            if [ "$untracked_authored" -eq 0 ]; then
                echo "check-cargo-config-tracked: hand-authored cargo config NOT tracked:" >&2
            fi
            echo "  $cfg" >&2
            untracked_authored=$((untracked_authored + 1))
        fi
    elif [ "$tracked" -eq 1 ] && ! includes_committed_projection "$cfg"; then
        if [ "$tracked_pure" -eq 0 ]; then
            echo "check-cargo-config-tracked: pure sync-output cargo config IS tracked:" >&2
        fi
        echo "  $cfg" >&2
        tracked_pure=$((tracked_pure + 1))
    fi
done < <(find examples packages tests "${NROS_FIND_PRUNE[@]}" -o -path '*/.cargo/config.toml' -print0 2>/dev/null)

rc=0
if [ "$untracked_authored" -ne 0 ]; then
    {
        echo
        echo "  These hold content `nros sync` cannot regenerate — a \`[build] target\`,"
        echo "  a runner, or link flags. \`**/.cargo/config.toml\` is gitignored because"
        echo "  most of these files are pure sync output, so committing one takes:"
        echo "      git add -f <path>"
    } >&2
    rc=1
fi
if [ "$ament_rows_tracked" -ne 0 ]; then
    {
        echo
        echo "  These rows are built from the USER's ament install, so a fresh clone has"
        echo "  no such path and cargo fails on the missing manifest. Drop the row: sync"
        echo "  writes what it owns to the gitignored \`.cargo/nros-managed-patch.toml\`,"
        echo "  and a leaf that path-deps its msg crate in Cargo.toml needs no patch at all."
        echo "  (\`packages/interfaces/*\` are exempt — they COMMIT their generated tree.)"
    } >&2
    rc=1
fi
if [ "$untracked_projection" -ne 0 ]; then
    echo "" >&2
    echo "  A clone gets the include and not the file, so the leaf cannot LINK" >&2
    echo "  until someone runs \`nros sync\` — and every sync that does run" >&2
    echo "  rewrites a TRACKED file, leaving the tree dirty (issue 0559)." >&2
    # `-f`: the blanket `**/.cargo/nros-board.toml` ignore covers the generated
    # majority, so adding a committed one needs the override. Saying `git add`
    # plainly here would hand the reader a command that refuses.
    echo "  Commit the projection:  git add -f <leaf>/.cargo/nros-board.toml" >&2
    exit 1
fi

if [ "$orphan_includes" -ne 0 ]; then
    {
        echo
        echo "  Cargo raises a missing \`include\` as a HARD error during manifest parse,"
        echo "  so these leaves cannot be READ — \`cargo metadata\` fails too. Only"
        echo "  \`nros-patch.toml\` (central), \`nros-managed-patch.toml\` (per-leaf) and"
        echo "  \`nros-board.toml\` (per-leaf board projection, phase-341) are"
        echo "  generated; an include naming anything else can never be satisfied by any"
        echo "  \`nros sync\` run. Drop the entry, or make sync write the target."
        echo "  (See docs/issues/0463-*.)"
    } >&2
    rc=1
fi
if [ "$tracked_pure" -ne 0 ]; then
    {
        echo
        echo "  These hold nothing but sync's own include + [patch.crates-io] block, so"
        echo "  they are recreated by \`nros sync\` and only churn in git. Untrack with:"
        echo "      git rm --cached <path>"
    } >&2
    rc=1
fi

# Issue 1074 — the skip above is REPORTED, never silent.
#
# Skipping a config because its package is gone is right, but a skip nobody can
# see is the same defect one layer over: the residue would sit there forever,
# and the next person to wonder why an old example still has a `generated/` tree
# would have nothing to read. So the count is printed, with the remedy that is
# actually correct — DELETE the directory. Not `git add -f`, which is what this
# gate used to imply and which would resurrect a deleted package's config.
#
# Advisory, not a failure: residue is local state that no clone and no CI run
# has, so failing on it would make a green tree depend on a developer's history.
if [ "$orphan_residue" -ne 0 ]; then
    {
        echo
        echo "  note: skipped $orphan_residue cargo config(s) whose package has no tracked"
        echo "  files — residue of a deleted package (\`git rm\` cannot remove a gitignored"
        echo "  file, and every leaf config is gitignored). Not a finding, and not tracked"
        echo "  by anything; delete the directories when convenient:"
        printf '      rm -rf %s\n' "${orphan_dirs[@]}" | sort -u
    } >&2
fi

[ "$rc" -eq 0 ] && echo "check-cargo-config-tracked: OK (tracked <=> hand-authored content)"
exit "$rc"
