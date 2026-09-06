#!/usr/bin/env bash
# RFC-0090 / phase-429 W5 — the codegen-version check must FIRE.
#
# A check that has never been observed to fail is a check nobody has evidence
# still works, and this one guards the failure that stopped nano-ros shipping a
# prebuilt `nros`: a binary emitting code the runtime does not accept, which
# COMPILES and is wrong. `check-fast` proves the tree is green; only a negative
# control proves the mechanism is armed.
#
# Three arms, all from the REAL emitted artifact rather than a hand-written
# imitation: the block under test is extracted verbatim from a committed golden
# header, so a template edit that removed the check would take these with it.
#
#   1. in range      -> compiles          (negative control; a refusal that
#                                          refuses everything proves nothing)
#   2. out of range  -> #error, both C and C++
#   3. config header present but silent -> #error (fail-closed)
#
# Arm D covers the RUST half, and it exists because the first version of this
# gate declined to — on a reason that was wrong. It said "CLAUDE.md bans
# compiling inside tests", which is true and irrelevant: that ban is on
# COMPILING AT TEST RUNTIME, and this is a GATE. `check-c` has compiled an
# expected-failure probe for years (`serialization_format_mismatch_probe.c`),
# which is the same shape one language over. `format_check.rs` is right that
# there is no `trybuild`; it does not follow that the negative case cannot be
# proven, only that it is proven with cargo instead.
#
# The probe is a scratch crate rather than an in-tree file, because the
# assertion under test lives in EMITTED code: a tracked probe would have to be
# hand-written to look like emitted code and could drift from it silently.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/grep-q.sh
source "$root/scripts/lib/grep-q.sh"
golden="$root/packages/cli/rosidl-codegen/tests/fixtures/fingerprint-corpus/expected/inline/Shapes.h"
[ -r "$golden" ] || { echo "FAIL: no golden header at $golden" >&2; exit 1; }

for tool in gcc g++; do
    command -v "$tool" >/dev/null || {
        echo "SKIP: $tool not on PATH — cannot compile the negative controls" >&2
        exit 0
    }
done

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
mkdir -p "$work/inc/nros"

# The block under test, verbatim from the emitted artifact.
python3 - "$golden" "$work/stamp.h" <<'PY'
import sys
# Extraction must DEGRADE, not throw. The mutation this gate exists to catch is
# "the range check was removed from the pack", and that removes the very anchor
# an extractor keys on -- so a naive extractor dies with a traceback, which is a
# failure with a useless message. Write what was found and let the assertions
# below name what is missing.
src = open(sys.argv[1]).read()
head = '#include <nros/nros_config_generated.h>'
start = src.find(head)
if start < 0:
    open(sys.argv[2], 'w').write('/* no codegen-version block in the golden */\n')
    sys.exit(0)
check = src.find('#elif NROS_EMITTED_CODEGEN_VERSION', start)
if check < 0:
    nl = src.find('\n', src.find('#define NROS_EMITTED_CODEGEN_VERSION', start))
    end = nl if nl > 0 else len(src)
else:
    end = src.index('#endif', check) + len('#endif')
open(sys.argv[2], 'w').write(src[start:end] + '\n')
PY

nros_grep_q '#define NROS_EMITTED_CODEGEN_VERSION' "$work/stamp.h" \
    || { echo "FAIL: extracted block carries no emitted-version define" >&2; exit 1; }
if ! nros_grep_q '#elif NROS_EMITTED_CODEGEN_VERSION' "$work/stamp.h"; then
    echo "FAIL: the emitted header carries no range check -- the pack's codegen-version" >&2
    echo "      partial has lost its '#elif', so generated C/C++ asserts NOTHING at" >&2
    echo "      compile time (RFC-0090)." >&2
    exit 1
fi

emitted="$(sed -n 's/^#define NROS_EMITTED_CODEGEN_VERSION \([0-9]*\)$/\1/p' "$work/stamp.h")"

mkconfig() {  # $1=dir $2=min $3=max ; empty min/max = define neither
    local d="$1"
    mkdir -p "$d/nros"
    { echo '#ifndef NROS_CONFIG_GENERATED_H'
      echo '#define NROS_CONFIG_GENERATED_H'
      [ -n "$2" ] && echo "#define NROS_CODEGEN_VERSION_MIN $2"
      [ -n "$3" ] && echo "#define NROS_CODEGEN_VERSION $3"
      echo '#endif'; } > "$d/nros/nros_config_generated.h"
    sed 's/NROS_CONFIG_GENERATED_H/NROS_CPP_CONFIG_GENERATED_H/g' \
        "$d/nros/nros_config_generated.h" > "$d/nros/nros_cpp_config_generated.h"
}

printf '#include "stamp.h"\nint main(void){return 0;}\n' > "$work/tu.c"
printf '#include "stamp.h"\nint main(){return 0;}\n'     > "$work/tu.cpp"

fails=0
ok()   { printf '  ok    %s\n' "$1"; }
bad()  { printf '  FAIL  %s\n' "$1"; fails=$((fails + 1)); }

echo "codegen-version refusal (RFC-0090, emitted version $emitted)"

# 1 — in range, both languages: must compile.
mkconfig "$work/good" "$emitted" "$emitted"
for t in "gcc:$work/tu.c:C" "g++:$work/tu.cpp:C++"; do
    IFS=: read -r tool src lang <<< "$t"
    if "$tool" -I"$work" -I"$work/good" -fsyntax-only "$src" 2>/dev/null; then
        ok "A  $lang: version $emitted inside the accepted range compiles"
    else
        bad "A  $lang: the emitted artifact does NOT compile against a runtime that accepts it"
    fi
done

# 2 — out of range, both languages: must refuse, and say so.
mkconfig "$work/narrow" $((emitted + 1)) $((emitted + 2))
for t in "gcc:$work/tu.c:C" "g++:$work/tu.cpp:C++"; do
    IFS=: read -r tool src lang <<< "$t"
    out="$("$tool" -I"$work" -I"$work/narrow" -fsyntax-only "$src" 2>&1 || true)"
    if nros_grep_q 'codegen version the runtime does not accept' <<< "$out"; then
        ok "B  $lang: a version outside the range is REFUSED, naming the remedy"
    else
        bad "B  $lang: out-of-range version was accepted (or the message changed)"
        printf '        %s\n' "$(head -1 <<< "$out")"
    fi
done

# 3 — fail-closed: config header present, macros absent. An undefined macro
#     reads as 0 to `#if`, so without the explicit guard this would produce a
#     range error about a version nobody set.
mkconfig "$work/silent" "" ""
out="$(gcc -I"$work" -I"$work/silent" -fsyntax-only "$work/tu.c" 2>&1 || true)"
if nros_grep_q 'did not define NROS_CODEGEN_VERSION' <<< "$out"; then
    ok "C  a silent config header is refused as MISSING, not as out-of-range"
else
    bad "C  a silent config header did not produce the fail-closed diagnostic"
    printf '        %s\n' "$(head -1 <<< "$out")"
fi

# D — the Rust half: a crate-scope `const _: () = assert!(…)` against the same
#     runtime constant, which is what every generated Rust crate carries.
if command -v cargo >/dev/null; then
    probe="$work/rustprobe"
    mkdir -p "$probe/src"
    cat > "$probe/Cargo.toml" <<TOML
[package]
name = "codegen_version_probe"
version = "0.0.0"
edition = "2021"
[dependencies]
nros-core = { path = "$root/packages/core/nros-core", default-features = false }
[workspace]
TOML
    write_probe() {  # $1 = emitted version
        cat > "$probe/src/lib.rs" <<RS
#![no_std]
pub const NROS_EMITTED_CODEGEN_VERSION: u32 = $1;
const _: () = assert!(
    nros_core::codegen_version::accepts(NROS_EMITTED_CODEGEN_VERSION),
    "this crate's NROS_EMITTED_CODEGEN_VERSION is not accepted by the nros-core \
     it is being compiled against"
);
RS
    }
    # `NROS_CARGO_FLAGS=` clears the repo's `--locked` shim: this crate is
    # synthesized outside any workspace and has no lock to honour.
    write_probe "$emitted"
    if NROS_CARGO_FLAGS= cargo check -q --manifest-path "$probe/Cargo.toml" 2>/dev/null; then
        ok "D  Rust: version $emitted inside the accepted range compiles"
    else
        bad "D  Rust: the emitted version does NOT compile against a runtime that accepts it"
    fi
    write_probe "$((emitted + 6))"
    out="$(NROS_CARGO_FLAGS= cargo check -q --manifest-path "$probe/Cargo.toml" 2>&1 || true)"
    if nros_grep_q 'E0080' <<< "$out"; then
        ok "D  Rust: a version outside the range is REFUSED at compile time"
    else
        bad "D  Rust: out-of-range version was accepted (expected error[E0080])"
        printf '        %s\n' "$(head -1 <<< "$out")"
    fi
else
    echo "  SKIP  D  Rust: cargo not on PATH" >&2
fi

echo
if [ "$fails" -ne 0 ]; then
    echo "codegen-version refusal: $fails check(s) FAILED" >&2
    exit 1
fi
echo "codegen-version refusal: all checks passed"
