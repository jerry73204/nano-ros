#!/usr/bin/env bash
# phase-431 W4 — `scripts/install.sh` must INSTALL, and must REFUSE.
#
# This is the script a user runs from a curl pipe on a machine with nothing
# provisioned, which makes its refusals the interesting half: a corrupted or
# unsigned asset that installs anyway is worse than one that fails, because the
# binary it leaves behind emits code into someone's workspace. A check nobody
# has watched fail is a check nobody has evidence still works.
#
# Everything is served from a LOOPBACK http server over a tarball built here
# from the checkout's own `nros`, so the test needs no release to exist and
# reaches no network. That is also why `NROS_INSTALL_URL` relaxes the scheme
# restriction — see the comment beside it in the script.
#
#   1. happy path       -> installed at <store>/nros/<version>, fronted, runs
#   2. version pinning  -> the prefix comes from the asset's share/nros/VERSION,
#                          not from `nros --version` (they differ by `-nrosN`)
#   3. bad checksum     -> refuses, and installs NOTHING
#   4. absent .sha256   -> refuses (unverified is not a fallback)
#   5. no asset at all  -> refuses, naming the source build as the way forward
set -uo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
installer="$root/scripts/install.sh"
cli="$root/packages/cli/target/release/nros"

FAILURES=0
fail() { echo "FAIL: $*" >&2; FAILURES=$((FAILURES + 1)); }
ok() { echo "  [OK] $*"; }

[ -x "$cli" ] || { echo "SKIP: no in-tree nros at $cli (just setup-cli)" >&2; exit 0; }
for tool in python3 curl tar zstd; do
    command -v "$tool" >/dev/null || { echo "SKIP: $tool not on PATH" >&2; exit 0; }
done

tmp="$(mktemp -d "${TMPDIR:-/tmp}/nros-installer-test.XXXXXX")"
srv_pid=""
cleanup() {
    [ -n "$srv_pid" ] && kill "$srv_pid" 2>/dev/null
    rm -rf "$tmp"
}
trap cleanup EXIT INT TERM

# Build the asset in the shape a release has: prefix-rooted, with the store
# version beside the binary.
mkdir -p "$tmp/serve" "$tmp/rel/bin" "$tmp/rel/share/nros"
cp "$cli" "$tmp/rel/bin/nros"
echo "9.9.9-nros7" >"$tmp/rel/share/nros/VERSION"
tar --zstd -cf "$tmp/serve/nros-asset.tar.zst" -C "$tmp/rel" bin share
( cd "$tmp/serve" && sha256sum nros-asset.tar.zst >nros-asset.tar.zst.sha256 )

# Port 0 lets the OS choose, so parallel runs of this test cannot collide — the
# fixed-port version of this file would have been a flake generator.
python3 -c '
import http.server, os, socketserver, sys, threading
os.chdir(sys.argv[1])
srv = socketserver.TCPServer(("127.0.0.1", 0), http.server.SimpleHTTPRequestHandler)
print(srv.server_address[1], flush=True)
srv.serve_forever()
' "$tmp/serve" >"$tmp/port" 2>/dev/null &
srv_pid=$!
port=""
for _ in $(seq 1 50); do
    port="$(cat "$tmp/port" 2>/dev/null)"
    [ -n "$port" ] && break
    sleep 0.1
done
[ -n "$port" ] || { echo "FAIL: loopback server never reported a port" >&2; exit 1; }
base="http://127.0.0.1:$port"

run_install() {
    local home="$1" url="$2"
    NROS_HOME="$home" NROS_INSTALL_URL="$url" sh "$installer" >"$home.log" 2>&1
}

# --- 1 + 2: it installs, at the version the ASSET names -------------------
home="$tmp/h1"
if run_install "$home" "$base/nros-asset.tar.zst"; then
    prefix="$home/sdk/nros/9.9.9-nros7"
    [ -x "$prefix/bin/nros" ] || fail "1: no binary at $prefix/bin/nros"
    [ -f "$prefix/.nros-provenance" ] || fail "1: no provenance marker"
    [ -L "$home/bin/nros" ] || fail "1: $home/bin/nros is not a symlink"
    [ "$(readlink "$home/bin/nros")" = "$prefix/bin/nros" ] \
        || fail "1: front link points at $(readlink "$home/bin/nros")"
    "$home/bin/nros" --codegen-version >/dev/null 2>&1 \
        || fail "1: the fronted binary does not run"
    # The whole point of arm 2: `nros --version` says 0.5.0, the asset says
    # 9.9.9-nros7, and the STORE must use the asset's — otherwise a later
    # `nros setup --tool nros` installs a second copy under the other name.
    [ -d "$prefix" ] || fail "2: prefix is not the asset's version"
    got="$(ls "$home/sdk/nros")"
    [ "$got" = "9.9.9-nros7" ] || fail "2: store holds [$got], not just the asset's version"
    ok "installs at the asset's version, fronts it, and it runs"
else
    fail "1: install failed
$(cat "$home.log")"
fi

# --- 3: a corrupted asset installs NOTHING --------------------------------
cp "$tmp/serve/nros-asset.tar.zst.sha256" "$tmp/serve/keep.sha256"
printf 'deadbeef  nros-asset.tar.zst\n' >"$tmp/serve/nros-asset.tar.zst.sha256"
home="$tmp/h2"
if run_install "$home" "$base/nros-asset.tar.zst"; then
    fail "3: a checksum mismatch INSTALLED"
else
    grep -q "checksum MISMATCH" "$home.log" || fail "3: refused without naming the cause"
    [ -e "$home" ] && fail "3: refused but left $home behind"
    ok "a checksum mismatch refuses, and installs nothing"
fi
cp "$tmp/serve/keep.sha256" "$tmp/serve/nros-asset.tar.zst.sha256"

# --- 4: no checksum beside the asset is not a fallback --------------------
cp "$tmp/serve/nros-asset.tar.zst" "$tmp/serve/unsigned.tar.zst"
home="$tmp/h3"
if run_install "$home" "$base/unsigned.tar.zst"; then
    fail "4: an asset with no .sha256 INSTALLED"
else
    grep -q "refusing to install unverified" "$home.log" \
        || fail "4: refused for the wrong reason"
    ok "an asset with no .sha256 refuses"
fi

# --- 5: nothing to download says what to do instead -----------------------
home="$tmp/h4"
if run_install "$home" "$base/absent.tar.zst"; then
    fail "5: a 404 INSTALLED"
else
    grep -q "bootstrap.sh" "$home.log" \
        || fail "5: a missing release must name the source build as the way forward"
    ok "a missing release names the source build"
fi

if [ "$FAILURES" -ne 0 ]; then
    echo "nros-installer-tests: $FAILURES failure(s)" >&2
    exit 1
fi
echo "nros-installer-tests: OK — installs, fronts, and refuses everything unverified."
