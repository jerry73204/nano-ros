#!/usr/bin/env bash
#
# issue 0460 — every knob Kconfig forwards must be READ by the Rust lane too.
#
# # The failure this prevents
#
# `zephyr/cmake/nros_cargo_build.cmake` publishes each tuning knob with
# `set(ENV{<NAME>} ...)`, which only touches the CONFIGURE-time cmake process.
# The C lane survives that because `nros_cargo_build()` re-bakes the variables
# into its build command (`cmake -E env ...`). The RUST lane's command is built
# by zephyr-lang-rust's `rust_cargo_application`, which passes its own fixed
# variable list and inherits nothing — so a Zephyr Rust image compiled its
# crates' DEFAULTS whatever Kconfig said, for EVERY knob at once.
#
# It surfaced as three `workspaces/features` entries dying after "Network
# ready" with a bare `Transport(ServiceServerCreationFailed)`:
# `CONFIG_NROS_MAX_QUERYABLES=16` reached the cmake-compiled shim TU and not
# the cargo-compiled one, which kept the default of 8 while the entries
# registered eleven capability services.
#
# The fix is a `$DOTCONFIG` fallback in each reading build script
# (`nros_zephyr_build::knob_usize` / `::dotconfig_usize` — one spelling). This
# gate is what keeps the two lists from drifting apart again: a knob added to
# the cmake side and not to a reader is one more silently-defaulted image, and
# nothing else in the build would say so.
#
# # What it checks
#
# Every `_nros_resolve_knob(<ENV_NAME> ...)` in the cmake module is either
# * named in a reader's `KCONFIG_KNOBS` table, or
# * read through a derived `CONFIG_<ENV_NAME>` lookup (nros-node, xrce-cffi),
# * or listed in NO_RUST_READER below with a reason.

set -euo pipefail
cd "$(dirname "$0")/.."

# issue 0726 — the three reader-shape checks below are the `if ! grep -q` shape,
# whose failure mode is a grep that could not START being reported as "$f has no
# KCONFIG_KNOBS table": a confident, specific, false claim, and only under load.
# `nros_grep_q` exits 2 on a tool failure instead of returning "no match".
# shellcheck source=scripts/lib/grep-q.sh
source scripts/lib/grep-q.sh

CMAKE=zephyr/cmake/nros_cargo_build.cmake
READERS=(
    packages/rmw/zenoh/nros-zpico-build/src/runner.rs
    packages/rmw/zenoh/nros-rmw-zenoh/build.rs
)
# Readers that derive the Kconfig name (`CONFIG_` + the env name) instead of
# tabulating pairs. Their knobs are matched by the env name appearing in the
# file at all.
DERIVED_READERS=(
    packages/core/nros-node/build.rs
    packages/rmw/xrce/nros-rmw-xrce-cffi/build.rs
    # The five parameter-store knobs (NROS_MAX_PARAMETERS,
    # NROS_MAX_PARAM_NAME_LEN, ...). Added when #0749 taught the cmake side to
    # forward NROS_MAX_PARAMETERS and this gate found its only reader still on
    # a plain env::var — which on a Zephyr Rust image reads the crate default
    # whatever Kconfig says (issue 0460).
    packages/core/nros-params/build.rs
    # The three RMW sizing knobs (NROS_RMW_MAX_BACKENDS,
    # NROS_RMW_SUBSCRIBER_SLOTS, NROS_RMW_MESSAGE_INFO_SLOTS). Added when
    # #0752 forwarded SUBSCRIBER_SLOTS and this file was still env-only.
    packages/rmw/cffi/build.rs
    # phase-8 — NROS_ZEPHYR_HEAP_SIZE, the rlsf arena behind the Zephyr
    # allocation funnel. Resolved here rather than with `option_env!` in source
    # for both reasons this gate exists: a bare environment read misses
    # `$DOTCONFIG` on the Rust lane, and with no build script cargo never
    # invalidates on a change to the knob.
    packages/platform/nros-platform/build.rs
)

# phase-420 W9 — a reader may name its knobs in a shared MANIFEST rather than in
# its own source. `packages/rmw/xrce/xrce-config.txt` is the one statement of
# every XRCE build value, read by BOTH the cargo lane (`build.rs`) and the cmake
# lane (`nros-rmw-xrce/CMakeLists.txt`); before it, the CMake lane read none of
# these six options at all and compiled the defaults.
#
# The names appear UNQUOTED there (a whitespace-separated column), so this list
# is matched with a word-boundary grep rather than the `"NAME"` literal the
# source readers use. A manifest only counts once something reads it: each entry
# names the readers that must appear in DERIVED_READERS above, and those are
# already held to the `nros_zephyr_build` requirement, so the `$DOTCONFIG` rung
# issue 0751 is about is still proved — by the file that resolves the knob, not
# by the file that lists it.
MANIFEST_READERS=(
    packages/rmw/xrce/xrce-config.txt
)

# Knobs the cmake side exports that no Rust build script reads. Each needs a
# reason: an unread export is either dead or a C-lane-only knob.
NO_RUST_READER=(
    # The cmake comment says xrce-sys/build.rs reads it unprefixed; that crate
    # was deleted in phase-321 W1.d and the surviving build script uses the
    # `XRCE_TRANSPORT_MTU_DEFAULT` const. C-lane only today.
    XRCE_TRANSPORT_MTU
)

[ -f "$CMAKE" ] || { echo "[FAIL] missing $CMAKE" >&2; exit 1; }

knobs="$(grep -oE '_nros_resolve_knob\(([A-Z0-9_]+)' "$CMAKE" \
    | sed 's/^_nros_resolve_knob(//' | sort -u)"
[ -n "$knobs" ] || { echo "[FAIL] no _nros_resolve_knob() calls found in $CMAKE" >&2; exit 1; }

fail=0
checked=0
for knob in $knobs; do
    checked=$((checked + 1))
    found=0
    for f in "${READERS[@]}" "${DERIVED_READERS[@]}"; do
        [ -f "$f" ] || continue
        if nros_grep_q -F "\"$knob\"" "$f"; then
            found=1
            # issue 0751 — the name APPEARING is not the name being resolved
            # through `$DOTCONFIG`. A forwarded knob read with a bare
            # `env::var("<KNOB>")` yields the crate DEFAULT on a Zephyr Rust
            # image: issue 0460 itself, wearing the shape that satisfies this
            # gate's own test.
            #
            # Not hypothetical. That is what `nros-params/build.rs` did before
            # #0749's follow-up, and it was caught only because the file was not
            # yet listed as a reader — once listed, this arm passed over it.
            if nros_grep_q -F "env::var(\"$knob\")" "$f"; then
                echo "[FAIL] $f reads forwarded knob $knob with a bare env::var" >&2
                echo "       On a Zephyr Rust image that yields the crate default" >&2
                echo "       whatever Kconfig says (issue 0460). Resolve it with" >&2
                echo "       nros_zephyr_build::knob_usize() instead." >&2
                fail=1
            fi
            break
        fi
    done
    if [ "$found" = 0 ]; then
        # A knob stated in a shared manifest. Unquoted, word-bounded.
        for f in "${MANIFEST_READERS[@]}"; do
            [ -f "$f" ] || continue
            if nros_grep_q -E "(^|[[:space:]])$knob([[:space:]]|\$)" "$f"; then
                found=1
                break
            fi
        done
    fi
    if [ "$found" = 0 ]; then
        for allowed in "${NO_RUST_READER[@]}"; do
            [ "$knob" = "$allowed" ] && { found=1; break; }
        done
    fi
    if [ "$found" = 0 ]; then
        echo "[FAIL] $knob is forwarded by $CMAKE but no Rust build script reads it" >&2
        fail=1
    fi
done

# The tabulating readers must route their rows through the shared helper — a
# table nobody consults is the same silence with extra steps.
for f in "${READERS[@]}"; do
    [ -f "$f" ] || continue
    nros_grep_q 'KCONFIG_KNOBS' "$f" || {
        echo "[FAIL] $f has no KCONFIG_KNOBS table" >&2; fail=1; continue
    }
    nros_grep_q 'nros_zephyr_build::\(knob_usize\|dotconfig_usize\)' "$f" || {
        echo "[FAIL] $f never calls the shared \`nros_zephyr_build\` fallback" >&2; fail=1
    }
done

# Derived readers must route through the shared helper too. The tabulating
# readers above are already held to this; the derived arm was not, which is the
# asymmetry issue 0751 records — its whole check is "the name appears", and a
# file can satisfy that while resolving nothing from `$DOTCONFIG`.
for f in "${DERIVED_READERS[@]}"; do
    [ -f "$f" ] || continue
    nros_grep_q 'nros_zephyr_build::\(knob_usize\|dotconfig_usize\)' "$f" || {
        echo "[FAIL] $f is listed as a derived reader but never calls the" >&2
        echo "       shared nros_zephyr_build fallback — so nothing it names" >&2
        echo "       is actually resolved from \$DOTCONFIG (issue 0751)." >&2
        fail=1
    }
done

# A manifest reader is only a reader if a lane actually consumes it. Listing a
# file here and having nothing read it would be exactly the silence this gate
# exists to break, one indirection further out.
for f in "${MANIFEST_READERS[@]}"; do
    [ -f "$f" ] || { echo "[FAIL] MANIFEST_READERS names missing $f" >&2; fail=1; continue; }
    consumed=0
    for r in "${READERS[@]}" "${DERIVED_READERS[@]}"; do
        [ -f "$r" ] || continue
        # COMMENTS STRIPPED, and the full repo-relative path, not the basename.
        # Every one of these readers also NAMES the manifest in prose, so a
        # basename grep over the raw file passes on a reader that only talks
        # about it — the "is this coverage or is it a mention?" confusion this
        # gate is otherwise about. Here-string, never a pipe (issue 1077).
        stripped="$(sed 's|//.*||' "$r")"
        nros_grep_q -F -- "$f" <<<"$stripped" && { consumed=1; break; }
    done
    if [ "$consumed" = 0 ]; then
        echo "[FAIL] $f is listed as a knob manifest but no Rust reader reads it" >&2
        echo "       — a manifest nobody parses states knobs that reach nothing." >&2
        fail=1
    fi
done

if [ "$fail" != 0 ]; then
    echo "" >&2
    echo "  A Zephyr RUST image inherits none of cmake's set(ENV{...}) knob" >&2
    echo "  exports (issue 0460). Add the knob to a reader's KCONFIG_KNOBS" >&2
    echo "  table and resolve it with nros_zephyr_build::knob_usize()." >&2
    exit 1
fi

echo "kconfig-knob-forwarding OK — $checked forwarded knob(s), each read by the Rust lane."
