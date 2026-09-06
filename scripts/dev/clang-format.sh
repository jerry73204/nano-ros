#!/usr/bin/env bash
# Resolve the clang-format binary at the version the repo was formatted with.
#
# clang-format output drifts between major versions (e.g. v17 vs v22 reformat
# `reinterpret_cast<T(*)[N]>` differently), so an unpinned PATH `clang-format`
# produces spurious `just format` diffs / `check-*-fmt` failures across machines.
#
# THE VERSION HAS ONE HOME: `[tool.clang-format]` in `nros-sdk-index.toml`
# (phase-422 W2). It used to also live in `.clang-format-version`, and the two
# drifted invisibly — gate.yml pinned 17.0.5 in a `pip install` while the file
# said 17.0.6, and this resolver's fallback order meant CI checked a different
# formatting standard than every local run for as long as that stood. The file
# is gone; provision with `just setup-clang-format`, which is
# `nros setup --tool clang-format`.
#
# `nros_clang_format` echoes the resolved binary path (pinned if present, else the
# PATH one with a loud version-skew warning) or errors with a setup hint.

# The pinned version, asked of the index rather than restated here.
#
# Parsed with `sed` rather than through `nros`: this is sourced by the format
# recipes, which run in trees where the CLI may not be built yet, and the
# alternative is a bootstrap loop (build the CLI to learn how to format the
# sources you are about to build it from). The index is still the SSoT; this
# reads it.
nros_clang_format_version() {
    local root
    root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    sed -n '/^\[tool\.clang-format\]/,/^\[/{s/^upstream = "\([^"]*\)".*/\1/p}' \
        "$root/nros-sdk-index.toml" 2>/dev/null | head -1
}

nros_clang_format() {
    local root want pinned have
    root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    want="$(nros_clang_format_version)"
    want="${want:-17.0.6}"
    # The SDK store, where `nros setup --tool clang-format` puts it. Constructed
    # from the pin, never searched (issue 0625) — the store accumulates, and a
    # glob would return whichever version sorted first.
    pinned="${NROS_HOME:-$HOME/.nros}/sdk/clang-format/${want}-nros1/bin/clang-format"
    if [ -x "$pinned" ]; then
        printf '%s\n' "$pinned"
        return 0
    fi
    # The pre-phase-422 project-local prefix, for a checkout provisioned before
    # this moved. Residue is what people HAVE (issue 0628's lesson), and it is
    # the right binary — the same wheel, unpacked to a different place.
    pinned="$root/build/clang-format/bin/clang-format"
    if [ -x "$pinned" ]; then
        printf '%s\n' "$pinned"
        return 0
    fi
    if command -v clang-format >/dev/null 2>&1; then
        have="$(clang-format --version 2>/dev/null | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)"
        if [ "$have" != "$want" ]; then
            printf 'WARN: clang-format %s on PATH != pinned %s — run `just setup-clang-format` for a consistent version (different versions reformat differently → spurious fmt diffs).\n' "${have:-unknown}" "$want" >&2
        fi
        command -v clang-format
        return 0
    fi
    printf 'ERROR: clang-format not found. Run `just setup-clang-format` (installs the pinned %s).\n' "$want" >&2
    return 1
}
