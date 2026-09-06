#!/usr/bin/env bash
#
# Issue 0160 — drift gate for hand-mirrored FFI structs.
#
# TWO families, one class of bug.
#
# 1. `component.h` re-declares a few `nros_cpp_ffi.h` structs behind
#    `#ifndef NROS_CPP_FFI_H` so a plain-C TU can use the component API without
#    the (cbindgen-generated) C++ FFI header. Those mirrors are hand-written and
#    have drifted on every append so far (phase-273 `callback_group`, phase-282
#    `tx_express` — the #131 "stale mirror" ABI class: a mirror-only TU passes a
#    SHORTER struct by value than the FFI consumer reads, so the tail field is
#    stack garbage).
#
# 2. `nros_native_tier_spec_t` (phase-432, RFC-0091 §5). The tier table crosses
#    C, C++ and Rust and is declared SIX times by hand — a header comment asks a
#    human to keep them in sync, which is exactly the arrangement family 1 exists
#    to replace. Worse, it is INITIALISED by two codegen templates, so a field
#    inserted anywhere but the end used to mis-assign every generated entry
#    silently. The templates now use designated initialisers, and this gate reads
#    their `.field` names back: declaration order and initialiser order are one
#    fact checked in one place.
#
# Both families normalize comments/whitespace and per-language type spellings and
# fail on any field difference. Hooked from `just check fast` so an append that
# misses a mirror fails the push lane, not a NuttX rebuild three days later.

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

python3 - <<'PY'
import re
import sys

failed = False


def fail(msg):
    global failed
    failed = True
    print(msg, file=sys.stderr)


def read(path):
    with open(path) as fh:
        return fh.read()


def strip_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    return text


def decls(body):
    """Split a C/C++ struct body into normalized field declarations."""
    out = []
    for decl in strip_comments(body).split(";"):
        decl = " ".join(decl.split())
        if decl:
            out.append(decl)
    return out


# ---------------------------------------------------------------------------
# Family 1 — component.h mirrors of nros_cpp_ffi.h
# ---------------------------------------------------------------------------

MIRROR = "packages/api/nros-c/include/nros/component.h"
CANONICAL = "packages/api/nros-cpp/include/nros/nros_cpp_ffi.h"

# (struct tag, {mirror-prefix: canonical-prefix} applied to the mirror's text)
CHECKS = [
    ("nros_cpp_qos_t", {"nros_c_qos_": "nros_cpp_qos_"}),
    ("nros_cpp_integrity_status_t", {}),
]


def struct_fields(path, tag, prefix_map):
    src = read(path)
    m = re.search(
        r"typedef struct %s \{(.*?)\n\} %s;" % (re.escape(tag), re.escape(tag)),
        src,
        re.S,
    )
    if not m:
        sys.exit(f"check-ffi-struct-mirrors: struct '{tag}' not found in {path}")
    body = m.group(1)
    for old, new in prefix_map.items():
        body = body.replace(old, new)
    return decls(body)


for tag, prefix_map in CHECKS:
    mirror = struct_fields(MIRROR, tag, prefix_map)
    canonical = struct_fields(CANONICAL, tag, {})
    if mirror != canonical:
        fail(f"FFI struct mirror DRIFTED: {tag}")
        print(f"  canonical ({CANONICAL}):", file=sys.stderr)
        for f in canonical:
            marker = " " if f in mirror else "+"
            print(f"   {marker} {f}", file=sys.stderr)
        print(f"  mirror ({MIRROR}, enum prefixes normalized):", file=sys.stderr)
        for f in mirror:
            marker = " " if f in canonical else "!"
            print(f"   {marker} {f}", file=sys.stderr)
        print(
            "  A field appended to the canonical struct MUST be appended to the\n"
            "  hand mirror too (and initialized in any *_default() helper) — a\n"
            "  mirror-only TU otherwise passes a shorter struct by value (ABI\n"
            "  mismatch, issue 0160 / the #131 stale-mirror class).",
            file=sys.stderr,
        )

# ---------------------------------------------------------------------------
# Family 2 — nros_native_tier_spec_t across its six declarations and two
# codegen initialisers.
# ---------------------------------------------------------------------------

TIER_CANONICAL = "packages/api/nros-c/include/nros/main.h"

# The C family is compared by full DECLARATION (types included): these four are
# cast to one another at runtime, so a type change is an ABI change. The only
# legitimate spelling difference is the setup fn-ptr typedef's name.
TIER_C_MIRRORS = [
    "packages/boards/nros-board-zephyr/c/zephyr_run_tiers.c",
    "packages/boards/nros-board-nuttx-qemu/c/nuttx_run_tiers.c",
    "packages/boards/nros-board-freertos/c/freertos_run_tiers.c",
]

# The C++ and Rust declarations are compared by FIELD NAME ORDER only. They
# cannot be compared by type: `NativeTierSpec` spells `groups` as `const char**`
# where C has `const char* const*`, and Rust spells everything differently. Name
# order is what the initialisers key on and what an insertion breaks.
TIER_NAME_ONLY = [
    ("packages/api/nros-cpp/include/nros/main.hpp", "cpp", "NativeTierSpec"),
    ("packages/api/nros-cpp/src/lib.rs", "rust", "NativeTierSpecC"),
]

# The two entry packs. Each emits ONE `.field = value` row inside a `{% for %}`,
# so each field name appears exactly once, in emission order.
# phase-432 W2.5 — the entry packs moved from a flat `templates/` directory to
# `packs/entry/<surface>/`, mirroring the message side. This gate went RED on
# the move, which is what it is for: it names its producers rather than
# globbing them, so a pack that walks away from it fails loudly.
ENTRY_PACKS = "packages/cli/nros-cli-core/src/codegen/entry/packs/entry"
TIER_INITIALISERS = [
    (f"{ENTRY_PACKS}/c/entry.c.jinja", "nros_native_tier_spec_t __nros_tiers["),
    (
        f"{ENTRY_PACKS}/cpp/entry.cpp.jinja",
        "::nros::board::NativeTierSpec __nros_tiers[",
    ),
]

# Field-name spelling is one fact; the fn-ptr typedef is not.
TIER_TYPE_NORMALIZE = {"nros_tier_setup_fn_t": "nros_c_entry_setup_fn"}


def anon_typedef_body(path, tag):
    src = read(path)
    m = re.search(
        r"typedef struct\s*\{(.*?)\n\}\s*%s;" % re.escape(tag), src, re.S
    )
    if not m:
        sys.exit(f"check-ffi-struct-mirrors: '{tag}' not found in {path}")
    return m.group(1)


def cpp_struct_body(path, name):
    src = read(path)
    m = re.search(r"\nstruct %s \{(.*?)\n\};" % re.escape(name), src, re.S)
    if not m:
        sys.exit(f"check-ffi-struct-mirrors: 'struct {name}' not found in {path}")
    return m.group(1)


def c_field_name(decl):
    """Field name from a normalized C/C++ declaration, fn-ptrs included."""
    m = re.search(r"\(\s*\*\s*([A-Za-z_]\w*)\s*\)", decl)
    if m:
        return m.group(1)
    m = re.search(r"([A-Za-z_]\w*)\s*(?:\[[^\]]*\])?$", decl)
    if not m:
        sys.exit(f"check-ffi-struct-mirrors: cannot name the field in '{decl}'")
    return m.group(1)


def rust_field_names(path, name):
    src = read(path)
    m = re.search(r"\npub struct %s \{(.*?)\n\}" % re.escape(name), src, re.S)
    if not m:
        sys.exit(f"check-ffi-struct-mirrors: 'struct {name}' not found in {path}")
    names = []
    for line in m.group(1).splitlines():
        line = line.strip()
        if not line or line.startswith("///") or line.startswith("//"):
            continue
        if line.startswith("#["):
            continue
        fm = re.match(r"pub\s+([A-Za-z_]\w*)\s*:", line)
        if fm:
            names.append(fm.group(1))
    return names


def initialiser_field_names(path, anchor):
    src = read(path)
    start = src.find(anchor)
    if start < 0:
        sys.exit(f"check-ffi-struct-mirrors: '{anchor}' not found in {path}")
    end = src.find("\n};", start)
    if end < 0:
        sys.exit(f"check-ffi-struct-mirrors: unterminated tier table in {path}")
    names = re.findall(r"^\s*\.([A-Za-z_]\w*)\s*=", src[start:end], re.M)
    if not names:
        sys.exit(
            f"check-ffi-struct-mirrors: the tier table in {path} uses no designated\n"
            "  initialisers. It must — a positional row silently mis-assigns every\n"
            "  field after an insertion (issue 0160's class, RFC-0091 §5)."
        )
    return names


def normalize_types(decl):
    for old, new in TIER_TYPE_NORMALIZE.items():
        decl = decl.replace(old, new)
    return decl


def report_names(label, names, reference):
    print(f"  {label}:", file=sys.stderr)
    for i, n in enumerate(names):
        want = reference[i] if i < len(reference) else "<none>"
        marker = " " if n == want else "!"
        print(f"   {marker} [{i}] {n}" + ("" if n == want else f"  (want {want})"),
              file=sys.stderr)


tier_canonical_decls = [
    normalize_types(d) for d in decls(anon_typedef_body(TIER_CANONICAL, "nros_native_tier_spec_t"))
]
tier_canonical_names = [c_field_name(d) for d in tier_canonical_decls]

TIER_ADVICE = (
    "  `nros_native_tier_spec_t` is declared by hand in six places and initialised\n"
    "  by two codegen templates. Every one of them must carry the same fields in\n"
    "  the same ORDER — a mirror that is short passes a truncated struct by value,\n"
    "  and an initialiser whose field names disagree with the declaration builds a\n"
    "  row for a struct nobody has (issue 0160's class, RFC-0091 §5). Append to ALL\n"
    "  of them together; never insert."
)

for path in TIER_C_MIRRORS:
    got = [normalize_types(d) for d in decls(anon_typedef_body(path, "nros_tier_spec_t"))]
    if got != tier_canonical_decls:
        fail(f"tier-spec mirror DRIFTED: {path}")
        print(f"  canonical ({TIER_CANONICAL}):", file=sys.stderr)
        for d in tier_canonical_decls:
            print(f"   {' ' if d in got else '+'} {d}", file=sys.stderr)
        print(f"  mirror ({path}, setup typedef normalized):", file=sys.stderr)
        for d in got:
            print(f"   {' ' if d in tier_canonical_decls else '!'} {d}", file=sys.stderr)
        print(TIER_ADVICE, file=sys.stderr)

for path, lang, name in TIER_NAME_ONLY:
    if lang == "rust":
        got = rust_field_names(path, name)
    else:
        got = [c_field_name(d) for d in decls(cpp_struct_body(path, name))]
    if got != tier_canonical_names:
        fail(f"tier-spec mirror DRIFTED (field names): {path} :: {name}")
        report_names(f"canonical ({TIER_CANONICAL})", tier_canonical_names, got)
        report_names(f"mirror ({path})", got, tier_canonical_names)
        print(TIER_ADVICE, file=sys.stderr)

for path, anchor in TIER_INITIALISERS:
    got = initialiser_field_names(path, anchor)
    if got != tier_canonical_names:
        fail(f"tier-spec INITIALISER DRIFTED: {path}")
        report_names(f"canonical ({TIER_CANONICAL})", tier_canonical_names, got)
        report_names(f"template ({path})", got, tier_canonical_names)
        print(TIER_ADVICE, file=sys.stderr)

sys.exit(1 if failed else 0)
PY

echo "FFI struct mirrors in sync (component.h + nros_native_tier_spec_t ×8)."
