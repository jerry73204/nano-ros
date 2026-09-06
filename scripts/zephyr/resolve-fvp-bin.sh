#!/usr/bin/env bash
# scripts/zephyr/resolve-fvp-bin.sh
#
# Phase 214.A — resolve the directory containing the ARM Fast Models
# `FVP_BaseR_AEMv8R` binary, the way Zephyr's `cmake/emu/armfvp.cmake`
# expects (`find_program(ARMFVP PATHS ENV ARMFVP_BIN_PATH NAMES
# ${ARMFVP_BIN_NAME})`).
#
# Resolution order (first hit wins):
#   1. `ARMFVP_BIN_PATH` (Zephyr's canonical env; honoured verbatim).
#   2. `ARM_FVP_DIR/models/Linux64_GCC-*` (the layout `nros-sdk-index.toml`
#      `[gated.arm-fvp]` describes — `ARM_FVP_DIR` points at the install
#      root the user accepted the Arm license for).
#   3. The nano-ros SDK store — `nros sdk-path arm-fvp`. Since 2026-09-06 the
#      model is a normal `[tool.arm-fvp]` the provisioner FETCHES (it is a
#      public Arm CDN permalink with a pinned digest, not a login wall), so
#      `nros setup --tool arm-fvp` is now the answer to "I do not have it".
#   4. `dirname $(command -v FVP_BaseR_AEMv8R)` — PATH fallback.
#
# Prints the absolute directory containing the FVP binary on stdout, or
# exits non-zero with a clear hint on stderr (callers can capture stdout
# and bail on non-zero).
#
# This script never downloads; `nros setup --tool arm-fvp` does. Keeping the
# fetch out of the resolver is deliberate: a resolver is called from build
# recipes and test probes, and one that can spend 68 MB of network is a
# resolver nobody can call in a loop.

set -euo pipefail

BIN_NAME="${ARMFVP_BIN_NAME:-FVP_BaseR_AEMv8R}"

emit_dir() {
    # $1 = candidate directory; verify the binary actually lives there.
    if [ -x "$1/$BIN_NAME" ]; then
        # Canonicalise (FVP install may be symlinked).
        cd "$1" && pwd -P
        return 0
    fi
    return 1
}

# 1. ARMFVP_BIN_PATH (Zephyr canonical).
if [ -n "${ARMFVP_BIN_PATH:-}" ]; then
    if emit_dir "$ARMFVP_BIN_PATH"; then
        exit 0
    fi
    echo "resolve-fvp-bin: ARMFVP_BIN_PATH=$ARMFVP_BIN_PATH does not contain $BIN_NAME" >&2
    exit 1
fi

# 2. ARM_FVP_DIR — sdk-index gated entry.
if [ -n "${ARM_FVP_DIR:-}" ]; then
    if [ ! -d "$ARM_FVP_DIR" ]; then
        echo "resolve-fvp-bin: ARM_FVP_DIR=$ARM_FVP_DIR is not a directory" >&2
        exit 1
    fi
    # Try common Linux64 GCC sublayouts (Arm ships /models/Linux64_GCC-<ver>).
    for cand in "$ARM_FVP_DIR" "$ARM_FVP_DIR"/models/Linux64_GCC-* \
                "$ARM_FVP_DIR"/bin "$ARM_FVP_DIR"/Base_RevC_AEMv8R_pkg/models/Linux64_GCC-* \
                "$ARM_FVP_DIR"/Base_RevC_AEMv8R_pkg/bin; do
        if emit_dir "$cand" 2>/dev/null; then
            exit 0
        fi
    done
    echo "resolve-fvp-bin: $BIN_NAME not found under ARM_FVP_DIR=$ARM_FVP_DIR" >&2
    echo "resolve-fvp-bin: hint — expected at \$ARM_FVP_DIR/models/Linux64_GCC-<ver>/$BIN_NAME" >&2
    exit 1
fi

# 3. PATH fallback.
if cmd_path=$(command -v "$BIN_NAME" 2>/dev/null); then
    emit_dir "$(dirname "$cmd_path")"
    exit 0
fi

# 3. The SDK store. `nros sdk-path` prints the prefix for an installed tool and
#    exits non-zero when it is absent, so a missing store is silent here rather
#    than an error the caller has to distinguish from "no FVP anywhere".
if command -v nros >/dev/null 2>&1; then
    if _store="$(nros sdk-path arm-fvp 2>/dev/null)" && [ -n "$_store" ]; then
        if emit_dir "$_store/bin" 2>/dev/null || emit_dir "$_store" 2>/dev/null; then
            exit 0
        fi
    fi
fi

# Nothing worked — emit setup hint.
cat >&2 <<EOF
resolve-fvp-bin: no $BIN_NAME found.

Set one of:
  ARMFVP_BIN_PATH  Directory containing $BIN_NAME (Zephyr-canonical env).
  ARM_FVP_DIR     Arm FVP install root (\$ARM_FVP_DIR/models/Linux64_GCC-*/$BIN_NAME).
  PATH             Add the directory containing $BIN_NAME.

Or let nano-ros fetch it (x86_64 Linux only — Arm publishes no other host
build of this model):

  nros setup --tool arm-fvp

It is a public Arm CDN permalink with a pinned digest, modelled as
\`[tool.arm-fvp]\` in nros-sdk-index.toml. It was treated as license-gated
until 2026-09-06, which cost every user a manual install for a file we
could always have downloaded.
EOF
exit 1
