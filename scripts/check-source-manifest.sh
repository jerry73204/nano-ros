#!/usr/bin/env bash
# phase-363 W3/W4 — self-test for the ONE source-signature helper.
#
# The properties below are the ones whose absence cost real investigations, so
# they are asserted rather than trusted:
#
#   1. NO TYPE FILTER. The predecessor hashed an extension allowlist and dropped
#      `.conf` (Zephyr Kconfig — issues 0167 and 0466, a kernel dump each),
#      `.msg` (codegen input) and a `.json` that is a custom Rust TARGET SPEC.
#      Nobody decided those were not inputs; they were simply not on the list.
#   2. Build output cannot leak in. Enumeration goes through the git index, so
#      an ignored tree is invisible — this is what makes (1) safe.
#   3. Order is content-determined, not filesystem-determined.
#   4. A dep-info closure is parsed as Make syntax, escaped spaces included. A
#      naive `.split()` truncates a path with a space and silently SHRINKS the
#      closure, which hashes to a perfectly valid-looking signature.
#   5. Failures are fatal. The predecessor swallowed errors into a shorter
#      stream (issue 0466 finding (c): a probe that breaks reports "fresh").
#
# Hermetic: everything happens in a throwaway git repo under $TMPDIR, so a
# failure here can never leave the real worktree dirty.
set -uo pipefail

# This script builds a throwaway repository (and runs `git add -A` inside it),
# so an inherited repository-local git environment (GIT_DIR and friends)
# silently redirects that work into the CALLER'S repository. Observed as a
# pre-push hook that corrupted the repo it was guarding (issue 0986). Cleared
# here because every git invocation below names its target explicitly.
# shellcheck source=scripts/lib/git-hook-env.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]}")/lib" && pwd)/git-hook-env.sh"
nros_clear_inherited_git_env


repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/build/source-manifest.sh
source "$repo_root/scripts/build/source-manifest.sh"
# issue 0726 — every assertion below reads a `grep -q` STATUS as an answer about
# the manifest, in BOTH directions (`&& bad … || ok …` as well as `if … else
# bad`). A grep that failed to start therefore does not merely lose a check: on
# the exclusion arms it reports that build output leaked into the manifest, and
# on the inclusion arms that a listed source was dropped. `nros_grep_q` exits 2.
# All of them already search a HERESTRING, so the helper runs in this shell and
# its `exit` ends the gate rather than a pipeline subshell.
# shellcheck source=scripts/lib/grep-q.sh
source "$repo_root/scripts/lib/grep-q.sh"

fail=0
ok() { echo "  ok    $1"; }
bad() {
    echo "  FAIL  $1" >&2
    fail=1
}

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

sandbox="$tmp/repo"
mkdir -p "$sandbox/leaf/sub" "$sandbox/leaf/target/debug"
(
    cd "$sandbox"
    git init -q .
    git config user.email t@t.t
    git config user.name t
    printf 'target/\n' > .gitignore
)
printf 'CONFIG_A=y\n' > "$sandbox/leaf/prj.conf"
printf 'int32 v\n' > "$sandbox/leaf/sub/Echo.msg"
printf '{"arch":"riscv32"}\n' > "$sandbox/leaf/spec.json"
printf 'MEMORY {}\n' > "$sandbox/leaf/memory.x"
printf '# docs\n' > "$sandbox/leaf/README.md"
printf 'BUILD OUTPUT\n' > "$sandbox/leaf/target/debug/artifact.o"
(cd "$sandbox" && git add -A >/dev/null 2>&1 && git commit -qm init)

echo "check-source-manifest: no type filter"
manifest="$(nros_source_manifest "$sandbox" leaf)" || bad "manifest failed"
for f in prj.conf sub/Echo.msg spec.json memory.x; do
    if nros_grep_q " leaf/$f\$" <<< "$manifest"; then ok "$f is hashed"; else bad "$f was DROPPED"; fi
done

echo "check-source-manifest: exclusions"
nros_grep_q "README.md" <<< "$manifest" && bad ".md should be skipped" || ok ".md skipped"
nros_grep_q "artifact.o" <<< "$manifest" && bad "gitignored build output LEAKED" || ok "build output excluded"

echo "check-source-manifest: content, not mtime"
before="$(nros_source_signature "$sandbox" leaf)"
touch "$sandbox/leaf/prj.conf"
[ "$(nros_source_signature "$sandbox" leaf)" = "$before" ] \
    && ok "an mtime bump with identical bytes is NOT a change" \
    || bad "mtime bump changed the signature (the treadmill returns)"
printf 'CONFIG_A=n\n' > "$sandbox/leaf/prj.conf"
[ "$(nros_source_signature "$sandbox" leaf)" != "$before" ] \
    && ok "a content edit IS a change" \
    || bad "a .conf content edit did not move the signature"

echo "check-source-manifest: deterministic order"
a="$(nros_source_manifest "$sandbox" leaf)"
b="$(nros_source_manifest "$sandbox" leaf)"
[ "$a" = "$b" ] && ok "stable across runs" || bad "manifest is not deterministic"

echo "check-source-manifest: dep-info closure"
dep_src="$sandbox/leaf/sub/with space.rs"
printf 'fn a() {}\n' > "$dep_src"
(cd "$sandbox" && git add -A >/dev/null 2>&1 && git commit -qm sp)
# Cargo escapes an embedded space as `\ ` — the case a naive split truncates.
printf '%s: %s %s\n' \
    "$sandbox/leaf/target/debug/x" \
    "$sandbox/leaf/prj.conf" \
    "${dep_src// /\\ }" > "$sandbox/leaf/target/debug/x.d"
closure="$(nros_dep_closure_manifest "$sandbox" "$sandbox/leaf/target/debug")" || bad "closure failed"
nros_grep_q "leaf/prj.conf\$" <<< "$closure" && ok "closure picks up a listed dep" || bad "closure missed a dep"
nros_grep_q "with space.rs\$" <<< "$closure" \
    && ok "an escaped space is one path, not two" \
    || bad "escaped space truncated the closure"

# issue 0635 — VCS state is not a build input. `nros-cli-core/build.rs` watches
# `.git/index` deliberately (its source stamp reads index blob SHAs), so any row
# whose closure reaches the CLI inherits a file that `git add`, `git commit` and
# `git status` rewrite with no source change — three compile-check rows reported
# STALE forever, across four rebuilds, each separated from its check by a commit.
printf '%s: %s %s\n' \
    "$sandbox/leaf/target/debug/x" \
    "$sandbox/leaf/prj.conf" \
    "$sandbox/.git/index" > "$sandbox/leaf/target/debug/x.d"
closure_git="$(nros_dep_closure_manifest "$sandbox" "$sandbox/leaf/target/debug")" \
    || bad "closure failed with a .git dep listed"
nros_grep_q "\.git/" <<< "$closure_git" \
    && bad "a listed .git/ path reached the closure" \
    || ok "a listed .git/ path is excluded from the closure"
nros_grep_q "leaf/prj.conf\$" <<< "$closure_git" \
    && ok "excluding .git/ keeps the real deps" \
    || bad ".git/ exclusion dropped a real dep"

# Both of these were REAL defects in the first version of the extractor, found
# only because a row's closure came back empty when it plainly should not have.
echo "check-source-manifest: dep-info shapes"
# gcc/clang `-MD` wraps with backslash continuations; cargo does not. A per-line
# parser sees the continuation lines as colon-less and skips them, returning only
# the first dependency — an empty-ish closure that hashes just fine.
printf 'x.o: \\\n %s \\\n %s\n' "$sandbox/leaf/prj.conf" "$sandbox/leaf/memory.x" \
    > "$sandbox/leaf/target/debug/wrapped.d"
wrapped="$(nros_dep_closure_manifest "$sandbox" "$sandbox/leaf/target/debug")"
nros_grep_q "leaf/memory.x\$" <<< "$wrapped" \
    && ok "a backslash-continued depfile yields every dep, not just the first" \
    || bad "continuation lines were dropped"
# A depfile may record paths relative to the compiler's cwd rather than absolute.
printf 'x.o: leaf/prj.conf\n' > "$sandbox/leaf/target/debug/rel.d"
relout="$(nros_dep_closure_manifest "$sandbox" "$sandbox/leaf/target/debug")"
nros_grep_q "leaf/prj.conf\$" <<< "$relout" \
    && ok "relative dep paths resolve against the repo root" \
    || bad "relative dep paths were dropped"

# A depfile lists BUILD OUTPUT as readily as source. Hashing that arms a rebuild
# on what the build just produced, so the signature never settles — three
# cxx-syntax rows reported STALE immediately after a successful build until this
# was fixed. Excluding the row's own build dir is not enough; the tree has other
# output roots. Git's ignore rules are the answer both halves of a signature use.
echo "check-source-manifest: build output stays out of the closure"
printf 'x.o: %s %s\n' "$sandbox/leaf/target/debug/artifact.o" "$sandbox/leaf/prj.conf" \
    > "$sandbox/leaf/target/debug/out.d"
outc="$(nros_dep_closure_manifest "$sandbox" "$sandbox/leaf/target/debug")"
nros_grep_q "artifact.o" <<< "$outc" \
    && bad "a gitignored build artifact entered the closure" \
    || ok "gitignored output is excluded (signature can settle)"
nros_grep_q "leaf/prj.conf\$" <<< "$outc" \
    && ok "tracked sources in the same depfile are kept" \
    || bad "the ignore filter dropped a tracked source"

# `git check-ignore` REFUSES a path inside a submodule (exit 128) instead of
# answering, which took the whole extraction down — 17 rows reported "signature
# failed". Submodule content is tracked source, so it is kept without asking.
echo "check-source-manifest: submodule paths survive the ignore filter"
mkdir -p "$sandbox/sub"
(
    cd "$sandbox/sub"
    git init -q .
    git config user.email t@t.t
    git config user.name t
    printf 'int x;\n' > hdr.h
    git add -A >/dev/null 2>&1
    git commit -qm sub
)
printf '[submodule "sub"]\n\tpath = sub\n\turl = ./sub\n' > "$sandbox/.gitmodules"
printf 'x.o: %s\n' "$sandbox/sub/hdr.h" > "$sandbox/leaf/target/debug/sub.d"
subc="$(nros_dep_closure_manifest "$sandbox" "$sandbox/leaf/target/debug")" \
    && ok "extraction survives a submodule path" \
    || bad "extraction failed on a submodule path"
nros_grep_q "sub/hdr.h\$" <<< "$subc" \
    && ok "submodule content is kept (it is tracked source)" \
    || bad "submodule content was dropped"
rm -f "$sandbox/leaf/target/debug/sub.d"

echo "check-source-manifest: cmake configure inputs"
mkdir -p "$sandbox/leaf/target/debug/CMakeFiles"
printf 'set(CMAKE_MAKEFILE_DEPENDS\n  "%s"\n  )\n' "$sandbox/leaf/memory.x" \
    > "$sandbox/leaf/target/debug/CMakeFiles/Makefile.cmake"
cm="$(nros_dep_closure_manifest "$sandbox" "$sandbox/leaf/target/debug")"
nros_grep_q "leaf/memory.x\$" <<< "$cm" \
    && ok "CMAKE_MAKEFILE_DEPENDS is read (configure-only rows have no compile)" \
    || bad "cmake configure inputs were not picked up"

echo "check-source-manifest: errors are fatal"
nros_source_manifest "$sandbox" >/dev/null 2>&1 && bad "no-path call should fail" || ok "no-path call fails"
nros_source_manifest "$tmp/not-a-repo" leaf >/dev/null 2>&1 \
    && bad "enumeration outside a repo should fail" \
    || ok "enumeration failure is fatal"

if [ "$fail" -ne 0 ]; then
    echo "check-source-manifest: FAILED" >&2
    exit 1
fi
echo "check-source-manifest: OK"
