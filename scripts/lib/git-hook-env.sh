#!/usr/bin/env bash
# The ONE way to drop a git-repository-local environment — issue 0986.
#
# WHAT GOES WRONG
#
# `GIT_DIR` and its family override BOTH a path argument and `git -C`. A script
# that builds a throwaway repository (`git init "$tmp/work"`) therefore does not
# build one when such a variable is inherited: every command lands in the
# repository the variable names. Measured on this tree, from `.githooks/pre-push`:
#
#   git init -q "$tmp/work"                      -> core.bare = true written into
#                                                   the CALLER's config, after which
#                                                   `git rev-parse --show-toplevel`
#                                                   fails for everything downstream
#   git -C "$tmp/super" update-index --add \
#       --cacheinfo "160000,0123456789...,dep"   -> a gitlink named `dep`, carrying a
#                                                   deliberately invalid sha, staged
#                                                   into the CALLER's index
#
# i.e. the hook whose job is refusing bad submodule pins was staging one, and the
# guard was mutating the thing it was reporting on. Both were silent: the
# selftests redirect to /dev/null and return a status.
#
# WHEN IT IS ACTUALLY REACHED — measured, git 2.34.1, not reasoned about
#
# `git push` from an ORDINARY checkout sets only `GIT_PREFIX=` for the hook, so
# the bug does not reproduce there, and it does not reproduce by hand either
# (an interactive shell has no GIT_DIR). It reproduces where the work happens:
#
#   push from a LINKED WORKTREE   GIT_DIR=<common>/.git/worktrees/<name>
#   git --git-dir=X --work-tree=Y push  GIT_DIR + GIT_WORK_TREE
#   any caller that already exported GIT_DIR   GIT_DIR inherited unchanged
#   git -c a.b=c ...              GIT_CONFIG_PARAMETERS
#
# A linked worktree is the normal shape for parallel agent sessions here, and a
# linked worktree's `$GIT_DIR/index` IS that worktree's real index — so the
# index half lands on live state.
#
# WHY THE LIST IS ASKED FOR AND NOT WRITTEN DOWN
#
# `git rev-parse --local-env-vars` is git's own answer to "which variables are
# repository-local", it needs no repository to answer, and it grows when git
# grows. Four hand-written copies of a subset is what this repo already had —
# the three scripts cleared four names, the hook cleared five, and none cleared
# `GIT_CONFIG` (which redirects `git config` WRITES) or
# `GIT_ALTERNATE_OBJECT_DIRECTORIES`. That divergence is the #282 -> #326 shape:
# a second spelling instead of a shared helper.
#
# USE
#
#   source "$(dirname "${BASH_SOURCE[0]}")/lib/git-hook-env.sh"
#   nros_clear_inherited_git_env
#
# Call it BEFORE the first git invocation, and before resolving a repo root —
# the point is that everything afterwards resolves from the working directory,
# which is where git puts a hook and where a gate is run from.
#
# Do NOT call this from a script that is SUPPOSED to act on the repository the
# environment names (a hook helper operating on the pushed repo, say). Nothing
# in this tree is: every git invocation in the callers names its target.

# nros_clear_inherited_git_env — unset every repository-local git variable.
#
# Returns 0 always; unsetting a variable that is not set is not an error, and a
# missing/ancient `git` falls back to the literal list rather than leaving the
# environment armed.
nros_clear_inherited_git_env() {
    local vars

    if vars="$(git rev-parse --local-env-vars 2>/dev/null)" && [ -n "$vars" ]; then
        # Word splitting is the point: one name per line from git.
        # shellcheck disable=SC2086
        unset $vars
        return 0
    fi

    # Fallback for a git too old to answer. Kept as the list git 2.34 prints, so
    # a diff against the live answer is a one-liner if this ever has to be
    # revisited. It is a FALLBACK, not a second source of truth — the branch
    # above is what runs.
    unset \
        GIT_ALTERNATE_OBJECT_DIRECTORIES \
        GIT_CONFIG \
        GIT_CONFIG_PARAMETERS \
        GIT_CONFIG_COUNT \
        GIT_OBJECT_DIRECTORY \
        GIT_DIR \
        GIT_WORK_TREE \
        GIT_IMPLICIT_WORK_TREE \
        GIT_GRAFT_FILE \
        GIT_INDEX_FILE \
        GIT_NO_REPLACE_OBJECTS \
        GIT_REPLACE_REF_BASE \
        GIT_PREFIX \
        GIT_INTERNAL_SUPER_PREFIX \
        GIT_SHALLOW_FILE \
        GIT_COMMON_DIR
    return 0
}
