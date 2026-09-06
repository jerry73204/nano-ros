#!/usr/bin/env bash
# Negative control for scripts/nros-reconfigure-stale.sh.
#
# The script reports "OK, N build dir(s) load" on a healthy tree, and that
# verdict is worth nothing on its own: a probe that can never fail prints the
# same thing as a probe that works. So this drives it against a build dir that
# is ACTUALLY wedged, in the way ninja actually wedges — a manifest error
# raised at LOAD, which is why ninja cannot re-run cmake to repair itself
# (issue 0882).
#
# Real cmake + real ninja, no compiler (`project(... NONE)`), ~2 s.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/grep-q.sh
source "$ROOT/scripts/lib/grep-q.sh"
SCRIPT="$ROOT/scripts/nros-reconfigure-stale.sh"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

fail() { echo "FAIL: $*" >&2; exit 1; }

# ---------------------------------------------------------------- fixture
mkdir -p "$TMP/src"
cat > "$TMP/src/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.16)
project(reconfigure_stale_probe NONE)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/marker.txt" "configured\n")
add_custom_target(noop COMMAND ${CMAKE_COMMAND} -E true)
EOF

BUILD="$TMP/build"
cmake -S "$TMP/src" -B "$BUILD" -G Ninja >/dev/null 2>&1 \
    || fail "could not configure the probe project"

# ------------------------------------------------- 1: healthy dir passes
if ! "$SCRIPT" --check "$TMP" >/dev/null 2>&1; then
    fail "a freshly configured build dir was reported as wedged"
fi
echo "  ok: a healthy build dir loads"

# ------------------------------------------------- 2: wedged dir is caught
# Two build statements producing one output — `multiple rules generate`, raised
# at LOAD. This is the shape that made the dir unrecoverable by ninja itself.
cat >> "$BUILD/build.ninja" <<'EOF'

rule nros_probe_dup
  command = true
build dup_output.txt: nros_probe_dup
build dup_output.txt: nros_probe_dup
EOF

if ninja -C "$BUILD" -t targets >/dev/null 2>&1; then
    fail "the manifest still loads — the wedge did not take, so the rest of \
this test would pass vacuously"
fi
echo "  ok: the wedge makes the manifest unloadable"

if "$SCRIPT" --check "$TMP" >/dev/null 2>&1; then
    fail "--check reported a wedged build dir as healthy"
fi
echo "  ok: --check detects the wedged dir"

# Capture first, then grep: the script exits non-zero by design here, and under
# `pipefail` that failure would propagate through the pipeline and be read as
# "grep found nothing".
check_out="$("$SCRIPT" --check "$TMP" 2>&1 || true)"
nros_grep_q "$BUILD" <<<"$check_out" || fail "--check did not name the offending dir"
echo "  ok: --check names the dir"

# ------------------------------------------------- 3: repair fixes in place
touch "$BUILD/marker.txt"
before="$(find "$BUILD" -name CMakeCache.txt | wc -l)"
"$SCRIPT" "$TMP" >/dev/null 2>&1 || fail "repair reported failure"

if ! ninja -C "$BUILD" -t targets >/dev/null 2>&1; then
    fail "the manifest still does not load after repair"
fi
echo "  ok: re-configure repairs the manifest in place"

after="$(find "$BUILD" -name CMakeCache.txt | wc -l)"
[ "$before" = "$after" ] || fail "repair did not preserve the build dir"
[ -f "$BUILD/marker.txt" ] || fail "repair deleted build-dir contents — it must \
never wipe, that is the whole point"
echo "  ok: repair preserved the build dir (no wipe)"

# ------------------------------------ 4: no CMakeCache => reported, not wiped
NOCACHE="$TMP/owned"
mkdir -p "$NOCACHE"
printf 'this is not a valid manifest\n' > "$NOCACHE/build.ninja"
out="$("$SCRIPT" "$TMP" 2>&1 || true)"
nros_grep_q "no CMakeCache.txt" <<<"$out" \
    || fail "a build dir with no CMakeCache was not reported as unrepairable"
[ -f "$NOCACHE/build.ninja" ] \
    || fail "the unrepairable dir was deleted — it must be left as evidence"
echo "  ok: an unrepairable dir is reported and left in place"

echo "cmake-reconfigure-stale-tests: OK (7 case(s))"
