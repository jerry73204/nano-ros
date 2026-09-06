#!/usr/bin/env bash
#
# NOTE (2026-09-06): for the COMMON case you no longer need this script.
# `nros setup --tool arm-fvp` downloads and installs the model — it is a public
# Arm CDN permalink with a pinned digest (`[tool.arm-fvp]`), not a licence wall,
# and this file was written on the belief that it was one.
#
# What survives here is the OTHER case: you already have an FVP — from Arm
# Development Studio, a corporate image, a different version — and want
# nano-ros to find it. That is discovery over an existing install, which the
# provisioner deliberately does not do.
# scripts/installers/arm-fvp-installer.sh
#
# Phase 217.B.1 — discovery + symlink for the license-gated Arm Fast
# Models `FVP_BaseR_AEMv8R` install. nano-ros NEVER downloads gated
# SDKs (same policy as `nv-spe-fsp` for NVIDIA Orin SPE); the user
# accepts the Arm EULA, extracts the tarball, and points
# `ARM_FVP_DIR` at the extraction root.
#
# Behaviour:
#   1. Refuse to run without `ARM_FVP_DIR` (no interactive prompts).
#   2. `find` the `FVP_BaseR_AEMv8R` binary under `$ARM_FVP_DIR`.
#   3. Symlink the directory that contains it to
#      `~/.nros/sdks/arm-fvp/current/` (atomic via `ln -sfn`) so
#      `nros doctor` + `scripts/zephyr/resolve-fvp-bin.sh` have a
#      stable landing path even when `ARM_FVP_DIR` is unset later.
#   4. Print a one-liner showing how to export `ARMFVP_BIN_PATH` so
#      Zephyr's `cmake/emu/armfvp.cmake` finds the binary
#      (`find_program(... PATHS ENV ARMFVP_BIN_PATH)`).
#
# Flags:
#   --help       Print usage + exit 0.
#   --print-env  Print the `export` line for `ARMFVP_BIN_PATH` (uses
#                the canonical landing path) + exit 0. Useful for
#                `eval "$(./arm-fvp-installer.sh --print-env)"`.
#
# Index reference: `[gated.arm-fvp]` in `nros-sdk-index.toml`.

set -euo pipefail

EULA_URL="https://developer.arm.com/downloads/-/arm-ecosystem-fvps"
BIN_NAME="${ARMFVP_BIN_NAME:-FVP_BaseR_AEMv8R}"
LANDING_DIR="${HOME}/.nros/sdks/arm-fvp"
LANDING_LINK="${LANDING_DIR}/current"

usage() {
    cat <<EOF
arm-fvp-installer.sh — discover + symlink an Arm FVP you already have install.

Usage:
  ARM_FVP_DIR=<extracted-fvp-root> $0
  $0 --print-env
  $0 --help

Required env:
  ARM_FVP_DIR   Directory the user extracted the Arm FVP tarball into.
                Must contain $BIN_NAME somewhere underneath
                (typical layout: \$ARM_FVP_DIR/models/Linux64_GCC-*/$BIN_NAME).

Download the FVP from $EULA_URL after accepting the Arm EULA.
nano-ros NEVER downloads license-gated SDKs.

On success, symlinks the discovered directory to:
  $LANDING_LINK
…and prints the export line for ARMFVP_BIN_PATH (Zephyr canonical env).
EOF
}

print_env() {
    printf 'export ARMFVP_BIN_PATH=%s\n' "$LANDING_LINK"
}

case "${1:-}" in
    --help|-h)
        usage
        exit 0
        ;;
    --print-env)
        print_env
        exit 0
        ;;
    "")
        ;;
    *)
        echo "arm-fvp-installer: unknown flag: $1" >&2
        usage >&2
        exit 1
        ;;
esac

if [ -z "${ARM_FVP_DIR:-}" ]; then
    cat >&2 <<EOF
arm-fvp-installer: ARM_FVP_DIR is required.

  ARM_FVP_DIR=<extracted-fvp-root> $0

Download the FVP from $EULA_URL after accepting the Arm EULA, then
extract the tarball and re-run pointing ARM_FVP_DIR at the extraction
root. nano-ros NEVER downloads license-gated SDKs.

Run \`$0 --help\` for full usage.
EOF
    exit 1
fi

if [ ! -d "$ARM_FVP_DIR" ]; then
    echo "arm-fvp-installer: ARM_FVP_DIR=$ARM_FVP_DIR is not a directory" >&2
    exit 1
fi

found="$(find "$ARM_FVP_DIR" -name "$BIN_NAME" -type f -executable 2>/dev/null | head -1)"
if [ -z "$found" ]; then
    cat >&2 <<EOF
arm-fvp-installer: $BIN_NAME not found anywhere under
  ARM_FVP_DIR=$ARM_FVP_DIR

Expected layout (Arm ships):
  \$ARM_FVP_DIR/models/Linux64_GCC-<ver>/$BIN_NAME

Did you extract the right tarball? The FVP is downloaded from
  $EULA_URL
after accepting the Arm EULA. nano-ros NEVER downloads license-gated
SDKs.
EOF
    exit 1
fi

bin_dir="$(cd "$(dirname "$found")" && pwd -P)"

mkdir -p "$LANDING_DIR"
ln -sfn "$bin_dir" "$LANDING_LINK"

echo "arm-fvp-installer: discovered $BIN_NAME at"
echo "  $bin_dir"
echo "arm-fvp-installer: symlinked"
echo "  $LANDING_LINK -> $bin_dir"
echo
echo "Export for Zephyr's armfvp.cmake (find_program PATHS ENV ARMFVP_BIN_PATH):"
print_env
echo
echo "Verify with: nros doctor --board fvp-aemv8r-smp"
