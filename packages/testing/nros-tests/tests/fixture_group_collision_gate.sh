#!/usr/bin/env bash
# phase-343 W3 — the collision gate detects an unhashed artifact collision.
#
# `check-fixture-groups.py` is the thing standing between a shared cargo target
# dir and a green test running the WRONG binary. Two rows in one group that emit
# the same unhashed artifact name overwrite each other, last writer wins, and the
# resolver hands one row's test the other row's artifact.
#
# The gate has been widened twice, and both times the widening was verified by
# hand and the verification survived only in a commit message:
#
#   phase-340 W1  owners keyed on the ROW, not the leaf directory — without it
#                 `examples/native/rust/talker`'s four rows dedup to one owner
#                 and 11 real `linux` collisions report as zero.
#   phase-340 B1  LIB artifacts included — staticlib/cdylib/dylib land flat and
#                 unhashed. `libnros_c.a` exists at 438 copies across ~30
#                 distinct sizes; the gate had been reporting "no collisions"
#                 over a namespace it was not looking at.
#
# A gate whose only proof of life is prose in a commit is a gate nobody can
# re-check. This asserts both arms mechanically, on perturbed leaf manifests, so
# a future narrowing of `artifacts()` or `collisions()` fails HERE rather than in
# whichever migration first trusts a false "no collisions".
#
#   T3  the real tree must PASS with an empty record, so T1/T2 are evidence the
#       gate discriminates rather than evidence it is stuck red
#   T0  the SHADOW tree — unperturbed — must produce byte-identical output to
#       the real tree, so T1/T2's reds are attributable to the perturbation and
#       not to the copy
#   T1  a deliberately re-collided BINARY name must be REPORTED
#   T2  a deliberately re-collided STATICLIB name must be REPORTED — the arm B1
#       added, and the one a narrowing would silently drop
#   T4  the real leaf manifests are byte-for-byte what they were at entry
#
# # A non-zero exit is NOT evidence
#
# The first draft of this file asserted only `rc != 0`, and T2 passed with B1
# reverted — because appending a second `[lib]` to a leaf that already has one
# is invalid TOML, so the gate died in `tomllib.load` with rc=1 and the
# assertion could not tell a crash from a detection. Both halves are fixed here:
# the perturbation EDITS the existing table instead of appending a second one,
# and every expectation matches the gate's MESSAGE for the artifact name it was
# supposed to find. Verified by reverting B1 and confirming T2 then fails.
#
# # Issue 1079 — the perturbation happens in a SHADOW tree, never in place
#
# Until 2026-09-05 the arms below appended `[[bin]] name = "tripwire_collider"`
# to two TRACKED example manifests in the shared worktree and restored them from
# an `EXIT` trap. Two things were measured on this host and neither is
# hypothetical:
#
#   * With a second `check-fixture-groups.py` looping concurrently, **22 of 43
#     of its runs reported `tripwire_collider`** and exited 1 — a red naming a
#     binary nobody authored, in a gate that reads only tracked files.
#   * Two concurrent runs of THIS script both printed "all checks passed" and
#     left `action-server-rtic/Cargo.toml` carrying the bogus `[[bin]]`: run A's
#     backup was taken while run B had already perturbed, so A "restored" the
#     perturbation. No `SIGKILL` needed; a later `git add -u` commits it.
#
# So the tree the gate reads is now a scratch overlay: one symlink per repo-root
# entry, with the two leaf directories on the path to the perturbed manifests
# split into real directories of symlinks and the two `Cargo.toml`s copied in.
# `check-fixture-groups.py` derives its root with `os.path.abspath` (which does
# NOT resolve symlinks) and runs every subprocess with `cwd=<that root>`, so
# invoking the copy under the overlay points the whole gate — manifest, group
# derivation, leaf reads — at the overlay.
#
# That the overlay is really what got read is SELF-ASSERTING rather than
# asserted: the collision exists only there, so a T1/T2 that fell back to the
# real tree would report no collision and FAIL. T4 closes the other direction.
set -uo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
cd "$repo_root"

# Issue 0726 — `expect_report` below concludes "the gate never mentioned
# <artifact>, so it did not detect this collision" from a `grep -q` that cannot
# tell rc 1 from rc>=2. Under the gate fan-out that verdict is reachable with
# the collision detected and the message printed. `nros_grep_q` exits 2 on
# rc>=2 rather than returning a status the caller reads as a finding.
# shellcheck source=../../../../scripts/lib/grep-q.sh
. "$repo_root/scripts/lib/grep-q.sh"

gate="scripts/check-fixture-groups.py"
collider="tripwire_collider"
fail=0

scratch="$(mktemp -d)"
shadow="$scratch/tree"
trap 'rm -rf "$scratch"' EXIT

# --- the shadow overlay -------------------------------------------------
#
# Every path these three functions create is asserted to be inside `$scratch`
# before anything is written. Belt and braces for a helper whose whole purpose
# is "do not touch the worktree" — and not theoretical: an `ln -s <target> <dir>`
# where <dir> is an existing symlink-to-directory silently creates the link
# INSIDE the real directory, which is exactly how a first draft of this overlay
# left eight stray symlinks in the tracked tree.

# in_scratch <path> — die unless <path> is under $scratch.
in_scratch() {
    case "$1" in
        "$scratch"/*) : ;;
        *) echo "  FATAL shadow helper asked to write outside the scratch dir:" \
                "$1" >&2; exit 1 ;;
    esac
}

# link_entries <real-dir> <shadow-dir> — one symlink per entry of <real-dir>.
#
# `dotglob` so `.gitignore` / `.cargo` come along; `nullglob` so an empty
# directory is not a literal `*`. `ln -T` refuses to descend into an existing
# directory rather than creating the link inside it.
link_entries() {
    local real="$1" dest="$2" e
    in_scratch "$dest"
    local reset
    reset="$(shopt -p dotglob nullglob)"
    shopt -s dotglob nullglob
    for e in "$real"/*; do
        ln -sT "$e" "$dest/$(basename "$e")" || { eval "$reset"; return 1; }
    done
    eval "$reset"
}

# shadow_split <reldir> — make $shadow/<reldir> a REAL directory whose entries
# are symlinks into the real tree. Idempotent; a directory already split is left
# alone so a deeper split does not undo a shallower one.
shadow_split() {
    local rel="$1" abs="$shadow/$1"
    in_scratch "$abs"
    if [ -d "$abs" ] && [ ! -L "$abs" ]; then
        return 0
    fi
    rm -f "$abs"
    mkdir -p "$abs" || return 1
    link_entries "$repo_root/$rel" "$abs"
}

# shadow_writable <relfile> — split every ancestor of <relfile>, then replace it
# with a real copy the caller may edit.
shadow_writable() {
    local rel="$1" acc="" part
    local IFS=/
    # shellcheck disable=SC2206  # deliberate split on the path separator
    local parts=($rel)
    unset IFS
    local n=$(( ${#parts[@]} - 1 ))
    local i
    for (( i = 0; i < n; i++ )); do
        acc="${acc:+$acc/}${parts[i]}"
        shadow_split "$acc" || return 1
    done
    in_scratch "$shadow/$rel"
    rm -f "$shadow/$rel"
    cp -p "$repo_root/$rel" "$shadow/$rel"
}

mkdir -p "$shadow"
if ! link_entries "$repo_root" "$shadow"; then
    echo "  FAIL could not build the shadow tree under $shadow" >&2
    exit 1
fi

# The platform A1 actually checks. Read from the file that owns the eligibility
# list rather than hardcoded: an arm keyed on a platform that has since left the
# shared list would pass vacuously.
shared_platform="$(bash -c '. scripts/build/fixtures-target-dir.sh
printf "%s" "$NROS_FIXTURE_SHARED_PLATFORMS"' | awk '{print $1}')"
if [ -z "$shared_platform" ]; then
    echo "  FAIL no shared platform in NROS_FIXTURE_SHARED_PLATFORMS —" \
         "A1 checks nothing and this tripwire would pass vacuously" >&2
    exit 1
fi

mapfile -t leaves < <(
    python3 scripts/build/fixtures-manifest.py list --lang rust --with-platform \
    | awk -F'\x1f' -v p="$shared_platform" '$1 == p { print $2 }' \
    | sort -u
)
if [ "${#leaves[@]}" -lt 2 ]; then
    echo "  FAIL platform $shared_platform has ${#leaves[@]} rust leaf dir(s);" \
         "a collision needs two" >&2
    exit 1
fi
a="${leaves[0]}"
b="${leaves[1]}"
echo "  .... platform $shared_platform, colliding $a against $b"

# The two manifests the arms perturb — as COPIES under $shadow. `shadow_reset`
# puts a pristine copy back between arms; the originals are never opened for
# writing, which is the whole point of issue 1079.
shadow_reset() {
    local rel
    for rel in "$a/Cargo.toml" "$b/Cargo.toml"; do
        shadow_writable "$rel" || return 1
    done
}
if ! shadow_reset; then
    echo "  FAIL could not stage the leaf manifests under $shadow" >&2
    exit 1
fi

# T4's evidence, captured BEFORE any arm runs.
real_before="$scratch/real-before"
mkdir -p "$real_before"
for rel in "$a/Cargo.toml" "$b/Cargo.toml"; do
    cp -p "$rel" "$real_before/$(echo "$rel" | tr '/' '_')"
done

# run_gate <root> — the gate, rooted at <root>. `$root/$gate` is what fixes the
# root: `check-fixture-groups.py` takes `dirname(dirname(abspath(__file__)))`
# and runs every subprocess with `cwd=` that, so the copy under the overlay
# reads the overlay's `examples/` and the real one reads the real tree.
run_gate() {
    python3 "$1/$gate" 2>&1
}

# expect_report <root> <artifact> <label> — the gate must FAIL *and say why*.
#
# Matching the message is the whole point: rc alone cannot distinguish "found
# the collision" from "crashed before looking", and a crash is exactly what a
# malformed perturbation produces.
expect_report() {
    local root="$1" artifact="$2" label="$3" out rc
    out="$(run_gate "$root")"
    rc=$?
    if [ "$rc" -eq 0 ]; then
        echo "  FAIL $label — gate PASSED on a tree containing the collision"
        echo "        gate said: $out"
        fail=1
        return
    fi
    # NOT `printf … | nros_grep_q`: a pipeline runs the helper in a SUBSHELL,
    # where its `exit 2` ends only that pipeline segment and hands the caller
    # back the exact rc it exists to remove. A herestring keeps it in-process.
    if ! nros_grep_q "$artifact" <<<"$out"; then
        echo "  FAIL $label — gate failed but never mentioned '$artifact', so it"
        echo "        did not detect this collision (a crash exits non-zero too)"
        echo "        gate said: $out"
        fail=1
        return
    fi
    echo "  ok   $label"
}

expect_clean() { # <root> <label>
    local root="$1" label="$2" out rc
    out="$(run_gate "$root")"
    rc=$?
    if [ "$rc" -ne 0 ]; then
        echo "  FAIL $label — gate FAILED on the unperturbed tree"
        echo "        gate said: $out"
        fail=1
        return
    fi
    real_tree_output="$out"
    echo "  ok   $label"
}

# --- T3 first: the real tree passes -------------------------------------
#
# Deliberately BEFORE the perturbations. If the tree is already red, T1 and T2
# would report "ok" for the wrong reason and this file would claim a working
# gate while proving nothing.
real_tree_output=""
expect_clean "$repo_root" "T3 real tree passes with an empty collision record"
if [ "$fail" -ne 0 ]; then
    echo "fixture_group_collision_gate: baseline is red — refusing to draw" \
         "conclusions from the tripwire arms" >&2
    exit 1
fi

# --- T0: the shadow tree is a faithful stand-in --------------------------
#
# Without this, "T1 reported the collision" would only tell you the shadow tree
# is BROKEN somewhere — the gate's counts are assertions (see its docstring),
# and an overlay that dropped rows would still report the collision it was fed.
# Byte-identical output is the cheapest way to say the copy changed nothing.
shadow_output="$(run_gate "$shadow")"
shadow_rc=$?
if [ "$shadow_rc" -ne 0 ] || [ "$shadow_output" != "$real_tree_output" ]; then
    echo "  FAIL T0 shadow tree does not reproduce the real tree's verdict —" \
         "the arms below would be measuring the overlay, not the gate"
    echo "        real tree:   $real_tree_output"
    echo "        shadow tree: $shadow_output"
    echo "fixture_group_collision_gate: FAILED" >&2
    exit 1
fi
echo "  ok   T0 shadow tree reproduces the real tree's verdict byte for byte"

# --- T1: two rows claiming one BINARY name ------------------------------
#
# `[[bin]]` is an array of tables, so an extra one is valid TOML on any leaf.
for f in "$shadow/$a/Cargo.toml" "$shadow/$b/Cargo.toml"; do
    printf '\n[[bin]]\nname = "%s"\npath = "src/main.rs"\n' "$collider" >> "$f"
done
expect_report "$shadow" "$collider" \
    "T1 two rows claiming one BINARY name are reported"
shadow_reset

# --- T2: two rows claiming one STATICLIB name (the B1 arm) --------------
#
# EDIT the existing `[lib]` rather than appending a second one: these leaves
# already declare `[lib] crate-type = ["rlib"]`, and a duplicate table is a
# TOMLDecodeError, which the gate reports as rc=1 with no collision found.
python3 - "$collider" "$shadow/$a/Cargo.toml" "$shadow/$b/Cargo.toml" <<'PY'
import sys

name, paths = sys.argv[1], sys.argv[2:]

def set_lib(path):
    """Force `[lib] name = <collider>, crate-type = ["staticlib", "rlib"]`.

    Line-oriented on purpose: tomllib reads but cannot write, and pulling in a
    TOML writer for a tripwire would add a dependency the gate itself does not
    have. Rewrites the `[lib]` table in place when present, appends it when not.
    """
    lines = open(path).read().splitlines()
    try:
        start = next(i for i, l in enumerate(lines) if l.strip() == "[lib]")
    except StopIteration:
        lines += ["", "[lib]", f'name = "{name}"',
                  'crate-type = ["staticlib", "rlib"]']
        open(path, "w").write("\n".join(lines) + "\n")
        return
    end = len(lines)
    for i in range(start + 1, len(lines)):
        if lines[i].lstrip().startswith("["):
            end = i
            break
    body = [f'name = "{name}"', 'crate-type = ["staticlib", "rlib"]']
    open(path, "w").write("\n".join(lines[:start + 1] + body + lines[end:]) + "\n")

for p in paths:
    set_lib(p)
PY
expect_report "$shadow" "lib${collider}.a" \
    "T2 two rows claiming one STATICLIB name are reported (phase-340 B1 arm)"

# --- T4: the worktree never saw any of it -------------------------------
#
# Issue 1079's regression guard, and the only arm that can fail if someone
# "simplifies" the overlay back into an in-place edit with a restore trap. A
# restore that ran is NOT the same claim: two concurrent runs both restored and
# still left a bogus `[[bin]]` behind, because one backed up the other's
# perturbation. Comparing against a copy taken at entry catches that; comparing
# against the gate's own verdict does not.
for rel in "$a/Cargo.toml" "$b/Cargo.toml"; do
    if ! cmp -s "$real_before/$(echo "$rel" | tr '/' '_')" "$rel"; then
        echo "  FAIL T4 $rel differs from its state at entry — this script wrote"
        echo "        the tracked worktree, which is issue 1079 returning"
        fail=1
    fi
done
if [ "$fail" -eq 0 ]; then
    echo "  ok   T4 both tracked leaf manifests are untouched"
fi

if [ "$fail" -ne 0 ]; then
    echo "fixture_group_collision_gate: FAILED" >&2
    exit 1
fi
echo "fixture_group_collision_gate: all checks passed"
