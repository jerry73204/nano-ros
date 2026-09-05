#!/usr/bin/env sh
# Install the `nros` CLI without a nano-ros checkout — phase-431 W4.
#
#   curl -fsSL https://raw.githubusercontent.com/NEWSLabNTU/nano-ros/main/scripts/install.sh | sh
#
# WHY THIS IS NOT `scripts/bootstrap.sh --download`
#
# The obvious shape was "bootstrap downloads by default, `--from-source` opts
# out". It is not implementable, and the reason is worth stating because it
# looks like a preference and is not:
#
#   * `bootstrap.sh` lives IN a checkout, so its download would land a released
#     binary on a contributor's PATH next to that checkout's sources;
#   * a released binary run against a nano-ros checkout is refused by
#     `refuse_if_foreign_to_workspace` (W1) and reported as a FAIL by
#     `just doctor` (W2) — both deliberately;
#   * and it cannot BE the checkout's binary either: `nros source-stamp`
#     compares against the sources it was built from, so a release from another
#     commit reads stale the moment it is copied into `packages/cli/target/`.
#
# That is RFC-0090's thesis rather than an inconvenience: the release is for
# people building THEIR workspace, and inside nano-ros itself the tree's own
# build is the only correct binary. So the two front doors split by AUDIENCE:
#
#   scripts/bootstrap.sh   in a checkout   -> builds from source (contributors)
#   scripts/install.sh     anywhere        -> downloads a release (users)
#
# This script therefore never touches a checkout. It installs into the SDK
# store — the same `<store>/<tool>/<version>/` layout `nros setup` writes — and
# points `$NROS_HOME/bin/nros` at the newest installed version, which is the
# "one command" promise (phase-431 W3).
#
# POSIX sh on purpose: it runs from a curl pipe on a machine with nothing
# provisioned, so bash is not assumed. No `nros` is assumed either, which is the
# bootstrap paradox this script exists to break.

set -eu

REPO="${NROS_INSTALL_REPO:-NEWSLabNTU/nano-ros}"
NROS_HOME="${NROS_HOME:-$HOME/.nros}"
STORE="$NROS_HOME/sdk"
BIN="$NROS_HOME/bin"
VERSION="${NROS_INSTALL_VERSION:-latest}"

die() {
    echo "nros-install: $*" >&2
    exit 1
}

need() {
    command -v "$1" >/dev/null 2>&1 || die "$1 is required and not on PATH."
}

usage() {
    cat <<'EOF'
Install the `nros` CLI.

  install.sh [--version <v>] [--dry-run]

  --version <v>   install this release (default: the newest published)
  --dry-run       print what would happen, change nothing
  -h, --help      this message

Environment:
  NROS_HOME             install root (default ~/.nros)
  NROS_INSTALL_VERSION  same as --version
  NROS_INSTALL_REPO     the GitHub repo to fetch from
  NROS_INSTALL_URL      fetch this exact asset instead (a mirror, an air-gapped
                        copy, a test server). Its `.sha256` must sit beside it
                        and is still REQUIRED; only the scheme restriction is
                        relaxed, because an operator naming a URL has chosen it

Contributors want scripts/bootstrap.sh instead: inside a nano-ros checkout the
tree's own build is the only correct binary (RFC-0090, phase-431 W1/W2).
EOF
}

DRY=0
while [ $# -gt 0 ]; do
    case "$1" in
        --version) VERSION="${2:?--version needs a value}"; shift 2 ;;
        --version=*) VERSION="${1#--version=}"; shift ;;
        --dry-run) DRY=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "unknown argument: $1 (try --help)" ;;
    esac
done

# The host key `nros-sdk-index.toml` uses (`sdk_index::host_key`): `<os>-<arch>`
# with aarch64 spelled arm64. Linux only for now — the tree defers macOS
# support, so an asset for it would be a support claim we do not make.
os="$(uname -s)"
arch="$(uname -m)"
case "$os" in
    Linux) os_key=linux ;;
    Darwin) die "macOS is not supported yet (the tree defers it).
Build from source instead:  git clone https://github.com/$REPO && cd nano-ros && ./scripts/bootstrap.sh" ;;
    *) die "unsupported host OS: $os" ;;
esac
case "$arch" in
    x86_64|amd64) arch_key=x86_64 ;;
    aarch64|arm64) arch_key=arm64 ;;
    *) die "unsupported host architecture: $arch" ;;
esac
HOST="$os_key-$arch_key"

need curl
need tar
need uname
# `.tar.zst` needs the external `zstd` binary — tar shells out to it, and a
# host without it fails DEEP inside tar with "zstd: Cannot exec", after the
# download. Same probe-before-downloading rule as `sdk_store::execute` (issue
# 0385), for the same reason: the error must name the missing package.
command -v zstd >/dev/null 2>&1 || die "the release is zstd-compressed and \`zstd\` is not on PATH.
Install it:  apt-get install zstd   |   dnf install zstd   |   pacman -S zstd   |   brew install zstd"
if command -v sha256sum >/dev/null 2>&1; then
    sha_cmd="sha256sum"
elif command -v shasum >/dev/null 2>&1; then
    sha_cmd="shasum -a 256"
else
    die "need sha256sum or shasum to verify the download."
fi

# Resolve the release. `latest` follows GitHub's own redirect rather than
# parsing the API, so this needs no token and no jq.
# An explicit URL wins over the release-name construction. `--proto '=https'`
# is dropped for it and ONLY for it: the default is a URL this script chose, so
# it holds the line; a URL the operator typed is one they picked, and refusing
# their `http://` mirror would just push them to `curl | tar` with no checksum
# at all. The checksum stays mandatory either way.
if [ -n "${NROS_INSTALL_URL:-}" ]; then
    url="$NROS_INSTALL_URL"
    sha_url="$url.sha256"
    label="$url"
    proto_args=""
elif [ "$VERSION" = latest ]; then
    url="https://github.com/$REPO/releases/latest/download/nros-$HOST.tar.zst"
    sha_url="$url.sha256"
    label="the newest release"
    proto_args="--proto =https --tlsv1.2"
else
    url="https://github.com/$REPO/releases/download/nros-$VERSION/nros-$HOST.tar.zst"
    sha_url="$url.sha256"
    label="$VERSION"
    proto_args="--proto =https --tlsv1.2"
fi

echo "nros-install: $label for $HOST"
echo "nros-install:   from $url"
echo "nros-install:   into $STORE/nros/<version>, fronted at $BIN/nros"
if [ "$DRY" = 1 ]; then
    echo "nros-install: (--dry-run: nothing changed)"
    exit 0
fi

tmp="$(mktemp -d "${TMPDIR:-/tmp}/nros-install.XXXXXX")"
# shellcheck disable=SC2064  # expand $tmp now, on purpose
trap "rm -rf '$tmp'" EXIT INT TERM

# shellcheck disable=SC2086  # proto_args is a deliberate word-split
if ! curl -fsSL $proto_args -o "$tmp/nros.tar.zst" "$url"; then
    die "could not download $url

No release for this host yet? nano-ros is still a source distribution until
phase-431 W5 cuts the first one. Build it instead:
  git clone https://github.com/$REPO && cd nano-ros && ./scripts/bootstrap.sh"
fi

# The checksum is REQUIRED, not best-effort. An asset served over a hijacked
# CDN and an asset that arrived intact look identical to `tar`, and this script
# is run from a curl pipe by someone who cannot inspect what it fetched.
# shellcheck disable=SC2086
curl -fsSL $proto_args -o "$tmp/nros.sha256" "$sha_url" \
    || die "release asset has no .sha256 beside it — refusing to install unverified.
This is a release-side defect; report it against $REPO."
want="$(cut -d' ' -f1 <"$tmp/nros.sha256")"
got="$($sha_cmd "$tmp/nros.tar.zst" | cut -d' ' -f1)"
[ -n "$want" ] || die "empty checksum in $sha_url"
[ "$want" = "$got" ] || die "checksum MISMATCH for nros-$HOST.tar.zst
  expected $want
  got      $got
Refusing to install. Retry; if it persists, report it against $REPO."

mkdir -p "$tmp/unpack"
tar -xf "$tmp/nros.tar.zst" -C "$tmp/unpack"
# The asset is prefix-rooted (`bin/nros`), the mirror shape every other dist in
# `nros-sdk-index.toml` uses, so it lands in the store unchanged.
[ -x "$tmp/unpack/bin/nros" ] || die "the archive has no bin/nros — wrong asset?"

# The version names a STORE PREFIX, so it has to be the one the SDK index uses
# for `[tool.nros]` (`0.5.0-nros1`) — not the crate version `nros --version`
# prints (`0.5.0`). They differ by the `-nrosN` repackaging counter, and getting
# it wrong means `nros setup --tool nros` later installs a SECOND copy of the
# same binary under the other name.
#
# So the asset carries it, written by the release workflow from the index
# (phase-431 W5), and this reads it rather than deriving it. Not from the URL:
# `latest` has no version in it, and a filename is a claim while a file inside
# the artifact is the artifact.
if [ -f "$tmp/unpack/share/nros/VERSION" ]; then
    ver="$(tr -d ' \t\r\n' <"$tmp/unpack/share/nros/VERSION")"
else
    # A pre-W5 asset. Fall back to the binary's own answer and SAY SO, because
    # the prefix will then not match what the index pins.
    ver="$("$tmp/unpack/bin/nros" --version 2>/dev/null | awk '{print $NF}')"
    echo "nros-install: WARNING: this asset carries no share/nros/VERSION;" >&2
    echo "nros-install:   using \`nros --version\` ($ver), which may not match the" >&2
    echo "nros-install:   store version \`[tool.nros]\` pins." >&2
fi
[ -n "$ver" ] || die "the downloaded binary does not run on this host (\`nros --version\` printed nothing)."
prefix="$STORE/nros/$ver"

if [ -d "$prefix" ]; then
    echo "nros-install: $ver already installed at $prefix — refreshing"
    rm -rf "$prefix"
fi
mkdir -p "$prefix"
# `cp -a .` rather than `mv`: the temp dir may be on another filesystem.
(cd "$tmp/unpack" && tar -cf - .) | (cd "$prefix" && tar -xf -)
cat >"$prefix/.nros-provenance" <<EOF
kind = "prebuilt"
version = "$ver"
sha256 = "$want"
EOF

# Point `$NROS_HOME/bin/nros` at the newest installed version — through the
# binary's own `sdk-front`, not a symlink written here. One implementation of
# "which version does `nros` mean" (phase-431 W3): a second one in shell is how
# the two come to disagree, and this script would be the one that is wrong,
# because the store is what accumulates.
# `--front` because there is no index yet: a user installing `nros` outside a
# checkout has no `nros-sdk-index.toml` until this install finishes. The path is
# the one input the binary cannot supply itself; WHICH version gets linked is
# still `front_newest`'s decision, reading the store.
"$prefix/bin/nros" sdk-front nros --front bin/nros >/dev/null \
    || die "installed $ver at $prefix, but could not front it at $BIN/nros (see above)."

echo "nros-install: installed $ver"
echo "nros-install:   $prefix"
echo "nros-install:   $BIN/nros -> $(readlink "$BIN/nros" 2>/dev/null || echo "$prefix/bin/nros")"

case ":$PATH:" in
    *":$BIN:"*) ;;
    *)
        echo ""
        echo "nros-install: $BIN is not on your PATH. Add it:"
        echo "    export PATH=\"$BIN:\$PATH\""
        ;;
esac
echo ""
echo "nros-install: next →  nros setup <board>    (e.g. \`nros setup native\`)"
echo "nros-install:         nros new <name>       scaffold a project"
