#!/usr/bin/env python3
"""phase-432 W1.2 — the two packs that describe ONE memory agree about it.

WHAT THIS GATES, AND WHAT IT DELIBERATELY DOES NOT
--------------------------------------------------
W1.1 deleted `TargetProfile`: codegen models no target at all. Removing a model
of the target without adding a MEASUREMENT of it is strictly worse than either,
so this is the measurement.

The pair is chosen by asking which two artifacts a single byte range is read
through, not by which two look similar:

  * `packs/c/message.h.jinja`         -> `typedef struct pkg_msg_name {...}`
  * `packs/cpp/message_types.rs.jinja`-> `#[repr(C)] pub struct pkg_msg_name_t`

The C++ pack's Rust glue receives a `*const c_void` that points at a C/C++
object and reinterprets it as its `repr(C)` struct. Rust `repr(C)` and the C
compiler follow the SAME ABI, so the two agree BY CONSTRUCTION unless the two
pack filters (`c_type` + `c_array_suffix` vs `cpp_repr_c_type`) SPELL the same
lowered field differently. That spelling is the whole hazard, and it is what
this measures.

NOT gated, on purpose:

  * The IDIOMATIC Rust pack (`packs/nros/*.jinja`, what `nros generate rust`
    emits) carries no `repr(C)` anywhere. It shares CDR with the C header --
    a wire format, target-independent by specification -- and NOT memory.
    Gating that pair would assert a property nobody needs.
  * The `packs/rmw/*.jinja` pack also emits `repr(C)`, but its memory partner
    is upstream ROS `rosidl_generator_c` (`crate::rosidl_runtime_rs::String` is
    the `{char*, size, cap}` trio rclcpp's C generator emits), NOT this repo's C
    pack. Our C header spells an unbounded string `char[256]`. Those two are
    SUPPOSED to differ; pairing them would be the same mistake one pack over.

HOW IT MEASURES
---------------
Neither side is asked to describe the other. Each side is asked, in its own
language, for numbers:

    C     char probe__X__size[sizeof(X) + 1];
          char probe__X__f__a[offsetof(X, a) + 1];
    Rust  #[no_mangle] static probe__X__size: [u8; size_of::<X>() + 1]
          #[no_mangle] static probe__X__f__a: [u8; offset_of!(X, a) + 1]

Both are cross-compiled for the SAME non-host target (32-bit `armv7a-none-eabi`
/ `arm-none-eabi-gcc -march=armv7-a`) to object files -- no linking, no
execution -- and the symbol SIZES are compared with `nm --print-size`. The `+ 1`
exists only so a zero offset still produces a sized symbol.

A 32-bit target is the point, not a convenience: `size_t` and pointers are 4
bytes there and 8 on the host, so the RFC-0033 `heap` containers
(`{T* data; size_t size; size_t capacity;}` vs `{*mut T, usize, usize}`) are
measured where the answer actually depends on the target.

WHAT A MISMATCH LOOKS LIKE
--------------------------
Three properties per struct pair, so a fault has somewhere to land:

  * total size + alignment   -- a width or capacity that differs
  * every top-level offset   -- a reordering, or an earlier field's size
  * every sub-field offset of an anonymous C container (`{size, data}` vs
    `{data, size, capacity}`) -- the sequence-container shape, which a
    top-level offset cannot see because the container is ONE field

Field NAMES are compared first: a pack that renames a field is a mismatch this
reports directly rather than as a number.

CORPUS
------
`packages/cli/rosidl-codegen/tests/fixtures/fingerprint-corpus` -- the same
in-tree corpus `codegen_fingerprint` hashes, so a shape added for one is covered
by the other. It needs no ROS install and no ament index. Both storage arms are
run: `inline` (empty resolver -- fixed arrays) and `configured`
(`nros-codegen.toml` -- `heap` pointers and `borrowed` views).

The packs reach this gate through the BUILT `nros` binary (`include_str!`
bundles them), exactly as `codegen_fingerprint` does -- so a stale CLI would
answer for sources that are no longer in the tree. `nros_cli_usable` in the
recipe is what makes that a SKIP rather than a wrong verdict.

`--self-test` is the negative control: it re-runs the comparator over a
deliberately perturbed reading and requires it to report. A gate that can only
say OK is not evidence.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
CORPUS = REPO / "packages/cli/rosidl-codegen/tests/fixtures/fingerprint-corpus"
# The one hand-written C header the generated ones reach for. `borrowed` fields
# are declared with its types, so the agreement spans a generated Rust struct
# and an AUTHORED C one -- exactly the pairing issue 0346 created.
VIEW_H = REPO / "packages/api/nros-c/include/nros/view.h"

# One target, named in one place. 32-bit so pointer/size_t width is exercised.
RUST_TARGET = "armv7a-none-eabi"
CC_FLAGS = ["-march=armv7-a", "-ffreestanding", "-fno-common", "-std=gnu11"]

PROBE = "nros_repr_probe"

# C-pack structs with NO counterpart in the C++ pack's Rust glue, and why.
#
# A two-way ratchet: an unlisted orphan is a failure, and a listed one that
# GAINS a counterpart is also a failure, so the map can only shrink. It is
# empty, and both halves have already earned their keep -- the first run put
# `fingerprint_corpus_action_probe_goal_View` here, the capacity-key fix in
# `generate_cpp_action_package` then gave it a counterpart, and the second half
# of the ratchet is what said so instead of leaving a permanent excuse behind.
C_ONLY_EXPECTED: dict[str, str] = {}

# C spellings that are types rather than struct references. Anything else that
# looks like an identifier in a field declaration must resolve to a struct this
# script extracted, or the script says so instead of guessing.
C_BUILTIN = {
    "bool",
    "char",
    "double",
    "float",
    "int",
    "long",
    "short",
    "signed",
    "unsigned",
    "void",
    "size_t",
    "ptrdiff_t",
    "int8_t",
    "int16_t",
    "int32_t",
    "int64_t",
    "uint8_t",
    "uint16_t",
    "uint32_t",
    "uint64_t",
    "struct",
    "const",
}


# --------------------------------------------------------------------------
# extraction
# --------------------------------------------------------------------------


def _balanced_body(text: str, open_at: int) -> tuple[str, int]:
    """The `{...}` starting at `open_at`, and the index just past its `}`."""
    depth = 0
    for i in range(open_at, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[open_at + 1 : i], i + 1
    raise SystemExit(f"{PROBE}: unbalanced braces at offset {open_at}")


def _split_members(body: str) -> list[str]:
    """Depth-0 `;`-separated declarations of a struct body."""
    out, depth, cur = [], 0, []
    for ch in body:
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
        if ch == ";" and depth == 0:
            decl = "".join(cur).strip()
            if decl:
                out.append(decl)
            cur = []
        else:
            cur.append(ch)
    return out


def _member_name(decl: str) -> str:
    """The declarator name of one C member declaration (no trailing `;`)."""
    # Drop array suffixes and pointer-to-array parens so the name is the last
    # token: `char data[64][256]` -> `char data`; `char (*data)[8]` -> handled
    # by the paren arm below.
    m = re.search(r"\(\s*\*\s*([A-Za-z_]\w*)\s*\)", decl)
    if m:
        return m.group(1)
    stripped = re.sub(r"\[[^\]]*\]", "", decl).strip()
    stripped = re.sub(r"\*", " ", stripped)
    toks = stripped.split()
    if not toks:
        raise SystemExit(f"{PROBE}: cannot name declarator in `{decl}`")
    return toks[-1]


def _struct_info(body: str) -> dict:
    fields, subs = [], {}
    for decl in _split_members(body):
        name = _member_name(decl)
        fields.append(name)
        if re.match(r"^struct\s*\{", decl):
            inner, _ = _balanced_body(decl, decl.index("{"))
            subs[name] = [_member_name(d) for d in _split_members(inner)]
    return {"body": body, "fields": fields, "subs": subs}


def extract_c_macro_structs(text: str) -> dict[str, dict]:
    """`view.h`'s LE views, which a macro defines once and instantiates nine times.

    They are still AUTHORED C, so they are extracted rather than re-spelled: the
    macro's own `typedef struct {...}` body is read, its `CT` parameter is bound
    from the invocation lines at the bottom of the header, and the result is a
    normal struct entry. A `view`-mode numeric sequence field lands on one of
    these, so without them the `configured` arm cannot be probed at all.
    """
    ctypes = {
        m.group(1): m.group(2).strip()
        for m in re.finditer(r"NROS__LE_VIEW_(?:INT|FLT)\(\s*(\w+)\s*,\s*([^,]+),", text)
    }
    m = re.search(r"#define\s+NROS__LE_VIEW_COMMON\(\s*SUFFIX\s*,\s*CT\s*\)", text)
    if not m or not ctypes:
        return {}
    tail = text[m.end() :]
    end = re.search(r"(?<!\\)\n\s*\n", tail)
    macro = re.sub(r"\\\n", "\n", tail[: end.start() if end else len(tail)])
    tm = re.search(r"typedef\s+struct\s*\{", macro)
    if not tm:
        return {}
    body, after = _balanced_body(macro, tm.end() - 1)
    if not re.match(r"\s*nros_le_slice_view_##SUFFIX##_t\s*;", macro[after:]):
        return {}
    out = {}
    for sfx, elem in ctypes.items():
        out[f"nros_le_slice_view_{sfx}_t"] = _struct_info(re.sub(r"\bCT\b", elem, body))
    return out


def extract_c_structs(text: str) -> dict[str, dict]:
    """`typedef struct <tag>? { ... } NAME;` blocks, by NAME.

    Records the raw body (re-emitted verbatim into the probe TU, so the probe
    measures the pack's own bytes and not a re-spelling of them), the top-level
    field names in order, and -- for a field declared as an ANONYMOUS struct --
    that container's own member names.
    """
    out: dict[str, dict] = {}
    for m in re.finditer(r"\btypedef\s+struct\b[^{;]*\{", text):
        body, after = _balanced_body(text, m.end() - 1)
        tail = text[after:]
        nm = re.match(r"\s*([A-Za-z_]\w*)\s*;", tail)
        if not nm:
            continue  # `typedef struct {...} *P;` and friends: not our shape
        out[nm.group(1)] = _struct_info(body)
    return out


def extract_rust_structs(text: str) -> dict[str, dict]:
    """`#[repr(C)] pub struct NAME { pub f: T, ... }` blocks, by NAME."""
    out: dict[str, dict] = {}
    for m in re.finditer(r"#\[repr\(C\)\]\s*(?:#\[[^\]]*\]\s*)*pub struct\s+(\w+)\s*\{", text):
        body, _ = _balanced_body(text, m.end() - 1)
        fields = re.findall(r"\bpub\s+(\w+)\s*:", body)
        out[m.group(1)] = {"decl": text[m.start() : m.end()] + body + "}", "fields": fields}
    return out


# --------------------------------------------------------------------------
# corpus generation
# --------------------------------------------------------------------------


def interface_files() -> list[str]:
    files = (
        sorted(CORPUS.glob("msg/*.msg"))
        + sorted(CORPUS.glob("srv/*.srv"))
        + sorted(CORPUS.glob("action/*.action"))
    )
    if not files:
        raise SystemExit(f"{PROBE}: no interface files under {CORPUS}")
    return [str(p) for p in files]


def generate(nros: str, work: Path, arm: str, configured: bool) -> tuple[Path, Path]:
    """Emit the C and C++ packs for one storage arm from the SAME corpus."""
    outs = {}
    for lang in ("c", "cpp"):
        out = work / f"{arm}-{lang}"
        out.mkdir(parents=True, exist_ok=True)
        args = {
            "package_name": "fingerprint-corpus",
            "output_dir": str(out),
            "interface_files": interface_files(),
            "dependencies": [],
            "ros_edition": "humble",
        }
        if configured:
            args["codegen_config"] = str(CORPUS / "nros-codegen.toml")
        af = work / f"{arm}-{lang}-args.json"
        af.write_text(json.dumps(args))
        r = subprocess.run(
            [nros, "codegen", "--language", lang, "--args-file", str(af)],
            capture_output=True,
            text=True,
        )
        if r.returncode != 0:
            raise SystemExit(
                f"{PROBE}: `nros codegen --language {lang}` failed for the {arm} arm:\n"
                f"{r.stdout}{r.stderr}"
            )
        outs[lang] = out
    return outs["c"], outs["cpp"]


# --------------------------------------------------------------------------
# probe emission
# --------------------------------------------------------------------------


def topo_order(structs: dict[str, dict]) -> list[str]:
    """C needs a complete type before it is embedded; Rust does not."""
    order, seen = [], set()

    def visit(name: str, stack: tuple[str, ...]) -> None:
        if name in seen:
            return
        if name in stack:
            raise SystemExit(f"{PROBE}: cyclic C struct reference: {' -> '.join(stack + (name,))}")
        body = structs[name]["body"]
        for other in structs:
            if other != name and re.search(rf"\b{re.escape(other)}\b", body):
                visit(other, stack + (name,))
        seen.add(name)
        order.append(name)

    for name in structs:
        visit(name, ())
    return order


def unresolved_c_types(structs: dict[str, dict]) -> set[str]:
    """Identifiers used as types that this script did not extract."""
    known = set(structs) | C_BUILTIN
    missing: set[str] = set()

    def walk(body: str) -> None:
        for decl in _split_members(body):
            declared = {_member_name(decl)}
            flat = decl
            while "{" in flat:
                at = flat.index("{")
                inner, after = _balanced_body(flat, at)
                walk(inner)
                for d in _split_members(inner):
                    declared.add(_member_name(d))
                flat = flat[:at] + " " + inner + " " + flat[after:]
            for tok in re.findall(r"[A-Za-z_]\w*", flat):
                if tok in declared or tok in known:
                    continue
                missing.add(tok)

    for info in structs.values():
        walk(info["body"])
    return missing


def c_probe_tu(structs: dict[str, dict], pairs: list[str], arm: str) -> str:
    lines = [
        "/* generated by scripts/check-repr-memory-agreement.py - do not edit */",
        "#include <stdint.h>",
        "#include <stdbool.h>",
        "#include <stddef.h>",
        "",
    ]
    for name in topo_order(structs):
        lines.append(f"typedef struct {name} {{{structs[name]['body']}}} {name};")
    lines.append("")
    for name in pairs:
        info = structs[name]
        lines.append(f"char {sym(arm, name, 'size')}[sizeof({name}) + 1];")
        lines.append(f"char {sym(arm, name, 'align')}[_Alignof({name}) + 1];")
        for f in info["fields"]:
            lines.append(f"char {sym(arm, name, 'f', f)}[offsetof({name}, {f}) + 1];")
            for sub in info["subs"].get(f, []):
                lines.append(
                    f"char {sym(arm, name, 'f', f, sub)}[offsetof({name}, {f}.{sub}) + 1];"
                )
    return "\n".join(lines) + "\n"


def rust_probe_crate(structs: dict[str, dict], pairs: list[tuple[str, str]], arm: str) -> str:
    lines = [
        "// generated by scripts/check-repr-memory-agreement.py - do not edit",
        "#![no_std]",
        "#![allow(non_camel_case_types, non_upper_case_globals, dead_code)]",
        "",
    ]
    for name in sorted(structs):
        lines.append(structs[name]["decl"])
    lines.append("")
    for c_name, r_name in pairs:
        for kind, expr in (
            ("size", f"core::mem::size_of::<{r_name}>()"),
            ("align", f"core::mem::align_of::<{r_name}>()"),
        ):
            s = sym(arm, c_name, kind)
            lines.append(
                f"#[unsafe(no_mangle)] pub static {s}: [u8; {expr} + 1] = [0; {expr} + 1];"
            )
    return "\n".join(lines) + "\n"


def rust_field_probes(
    c_structs: dict[str, dict], pairs: list[tuple[str, str]], arm: str
) -> list[str]:
    """Offset probes, emitted from the C side's FIELD NAMES only.

    The names are the one thing the two packs must share; the TYPES are never
    read across. A field the Rust struct does not have is a rustc error naming
    it, which is the verdict we want anyway.
    """
    out = []
    for c_name, r_name in pairs:
        info = c_structs[c_name]
        for f in info["fields"]:
            for path, sfx in [(f, (f,))] + [
                (f"{f}.{sub}", (f, sub)) for sub in info["subs"].get(f, [])
            ]:
                expr = f"core::mem::offset_of!({r_name}, {path})"
                s = sym(arm, c_name, "f", *sfx)
                out.append(
                    f"#[unsafe(no_mangle)] pub static {s}: [u8; {expr} + 1] = [0; {expr} + 1];"
                )
    return out


def sym(arm: str, struct: str, kind: str, *rest: str) -> str:
    parts = [PROBE, arm.replace("-", "_"), struct, kind, *rest]
    return "__".join(parts)


# --------------------------------------------------------------------------
# compile + read
# --------------------------------------------------------------------------


def nm_sizes(nm: str, obj: Path) -> dict[str, int]:
    r = subprocess.run(
        [nm, "--print-size", "--defined-only", str(obj)], capture_output=True, text=True
    )
    if r.returncode != 0:
        raise SystemExit(f"{PROBE}: {nm} failed on {obj}:\n{r.stderr}")
    out = {}
    for line in r.stdout.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[3].startswith(PROBE):
            out[parts[3]] = int(parts[1], 16)
    return out


def run(cmd: list[str], what: str) -> None:
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f"{PROBE}: {what} failed:\n{' '.join(cmd)}\n{r.stdout}{r.stderr}")


# --------------------------------------------------------------------------
# comparison
# --------------------------------------------------------------------------


def compare(c_vals: dict[str, int], r_vals: dict[str, int]) -> list[str]:
    """Every probe present on both sides must report the same number.

    A probe missing from ONE side is reported too: it means a struct or field
    exists in one pack and not the other, which is a spelling mismatch that
    happens to be invisible to arithmetic.
    """
    problems = []
    for name in sorted(set(c_vals) | set(r_vals)):
        cv, rv = c_vals.get(name), r_vals.get(name)
        # `name` came from the UNION, so at least one side has it; the type
        # checker cannot see that, and spelling it out is cheaper than an
        # ignore comment that would also hide a real None later.
        if cv is None and rv is not None:
            problems.append(f"  {name}: absent from the C probe, {rv - 1} in Rust")
        elif rv is None and cv is not None:
            problems.append(f"  {name}: {cv - 1} in C, absent from the Rust probe")
        elif cv is None or rv is None:
            problems.append(f"  {name}: absent from BOTH probes — unreachable")
        elif cv != rv:
            problems.append(f"  {name}: C says {cv - 1}, Rust says {rv - 1}")
    return problems


# --------------------------------------------------------------------------
# driver
# --------------------------------------------------------------------------


def check_arm(
    arm: str, nros: str, cc: str, nm: str, work: Path, configured: bool
) -> tuple[int, int, list[str]]:
    c_dir, cpp_dir = generate(nros, work, arm, configured)

    c_structs: dict[str, dict] = {}
    # walk-ok: `c_dir` is codegen output this function just produced under a
    # temp dir — nothing here is tracked, so `git ls-files` would return
    # nothing. The gate's own text allows scanning for untracked artifacts
    # scoped to a build dir, which is exactly this.
    for h in sorted(c_dir.rglob("*.h")):
        for name, info in extract_c_structs(h.read_text(encoding="utf-8")).items():
            c_structs[name] = info
    generated_names = sorted(c_structs)

    rust_structs: dict[str, dict] = {}
    # walk-ok: same — `cpp_dir` is this run's generated pack output, untracked.
    for rs in sorted(cpp_dir.rglob("*_types.rs")):
        for name, info in extract_rust_structs(rs.read_text(encoding="utf-8")).items():
            prev = rust_structs.get(name)
            if prev and prev["decl"] != info["decl"]:
                raise SystemExit(
                    f"{PROBE}: two different `{name}` declarations in the C++ pack output"
                )
            rust_structs[name] = info

    if not generated_names or not rust_structs:
        raise SystemExit(
            f"{PROBE}: the {arm} arm produced "
            f"{len(generated_names)} C struct(s) and {len(rust_structs)} Rust struct(s) "
            "- a corpus that emits nothing cannot agree about anything"
        )

    # Types the generated headers reach for but do not define (`nros_view_str_t`
    # and friends) come from the authored C header, extracted the same way.
    missing = unresolved_c_types(c_structs)
    if missing:
        view_text = VIEW_H.read_text(encoding="utf-8")
        authored = extract_c_structs(view_text) | extract_c_macro_structs(view_text)
        for name, info in authored.items():
            if name in missing:
                c_structs[name] = info
        missing = unresolved_c_types(c_structs)
    if missing:
        raise SystemExit(
            f"{PROBE}: the {arm} arm's C output names type(s) this script cannot "
            f"resolve: {', '.join(sorted(missing))}.\n"
            f"  They are not defined in the generated headers and not extractable\n"
            f"  from {VIEW_H.relative_to(REPO)}. Extend the extractor rather than\n"
            "  dropping the type - a silently skipped type is a gate that stops\n"
            "  covering whatever it was added for."
        )

    # Pair by name. `X` -> `X_t`, `X_View` -> `X_t_view`: the only two spellings
    # the C++ pack uses, and an unpaired generated struct is a failure, not a
    # skip.
    pairs: list[tuple[str, str]] = []
    unpaired: list[str] = []
    for c_name in generated_names:
        if c_name.endswith("_View"):
            r_name = c_name[: -len("_View")] + "_t_view"
        else:
            r_name = c_name + "_t"
        if r_name in rust_structs:
            if c_name in C_ONLY_EXPECTED:
                unpaired.append(
                    f"  {c_name}: recorded in C_ONLY_EXPECTED, but `{r_name}` now EXISTS.\n"
                    "      The asymmetry is gone -- delete the entry so the pair is compared."
                )
            pairs.append((c_name, r_name))
        elif c_name in C_ONLY_EXPECTED:
            continue
        else:
            unpaired.append(
                f"  {c_name}: no `{r_name}` in the C++ pack's Rust glue.\n"
                "      One pack emits a type the other does not. If that is intended,\n"
                "      record it in C_ONLY_EXPECTED with the reason; do not drop it."
            )
    if unpaired:
        return 0, 0, unpaired

    # Field NAMES first: a renamed field is a mismatch with a better message
    # than an offset ever gives.
    name_problems = []
    for c_name, r_name in pairs:
        cf, rf = c_structs[c_name]["fields"], rust_structs[r_name]["fields"]
        if cf != rf:
            name_problems.append(
                f"  {c_name}: field names differ\n"
                f"      C   : {', '.join(cf)}\n"
                f"      Rust: {', '.join(rf)}"
            )
    if name_problems:
        return len(pairs), 0, name_problems

    c_src = work / f"{arm}-probe.c"
    c_src.write_text(c_probe_tu(c_structs, [c for c, _ in pairs], arm))
    c_obj = work / f"{arm}-probe-c.o"
    run([cc, "-c", *CC_FLAGS, "-o", str(c_obj), str(c_src)], f"the {arm} C probe")

    rs_src = work / f"{arm}-probe.rs"
    rs_text = rust_probe_crate(rust_structs, pairs, arm)
    rs_text += "\n".join(rust_field_probes(c_structs, pairs, arm)) + "\n"
    rs_src.write_text(rs_text)
    rs_obj = work / f"{arm}-probe-rs.o"
    run(
        [
            "rustc",
            "--edition",
            "2024",
            "--target",
            RUST_TARGET,
            "--crate-type",
            "rlib",
            "--emit",
            "obj",
            "-C",
            "opt-level=0",
            "-o",
            str(rs_obj),
            str(rs_src),
        ],
        f"the {arm} Rust probe",
    )

    c_vals = nm_sizes(nm, c_obj)
    r_vals = nm_sizes(nm, rs_obj)
    if not c_vals or not r_vals:
        raise SystemExit(
            f"{PROBE}: the {arm} arm read {len(c_vals)} C and {len(r_vals)} Rust probe "
            "symbols - a comparison over nothing is not a pass"
        )
    return len(pairs), len(c_vals), compare(c_vals, r_vals)


def self_test() -> int:
    """The negative control: the comparator must SAY SO when readings differ.

    A gate whose only demonstrated behaviour is printing OK has not been shown
    to have any behaviour at all.
    """
    base = {f"{PROBE}__x__S__size": 41, f"{PROBE}__x__S__f__a": 9}
    fails = 0

    if compare(base, dict(base)):
        print("self-test: identical readings were reported as a mismatch", file=sys.stderr)
        fails += 1

    perturbed = dict(base)
    perturbed[f"{PROBE}__x__S__f__a"] = 5
    if not compare(base, perturbed):
        print("self-test: a differing field offset was NOT reported", file=sys.stderr)
        fails += 1

    if not compare(base, {f"{PROBE}__x__S__size": 41}):
        print("self-test: a probe missing from one side was NOT reported", file=sys.stderr)
        fails += 1

    if fails == 0:
        print("repr-memory-agreement self-test OK (3 comparator cases)")
    return 1 if fails else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--nros", default="packages/cli/target/release/nros")
    ap.add_argument("--cc", default="arm-none-eabi-gcc")
    ap.add_argument("--nm", default="arm-none-eabi-nm")
    ap.add_argument("--keep", action="store_true", help="keep the probe sources for inspection")
    ap.add_argument("--self-test", action="store_true", help="negative control, no toolchain")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    for tool in (args.cc, args.nm):
        if shutil.which(tool) is None:
            raise SystemExit(f"{PROBE}: `{tool}` not on PATH (the recipe should have skipped)")

    tmp_root = REPO / "tmp"
    tmp_root.mkdir(exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix="repr-memory-agreement.", dir=tmp_root))
    try:
        total_pairs = total_probes = 0
        problems: list[str] = []
        for arm, configured in (("inline", False), ("configured", True)):
            pairs, probes, probs = check_arm(
                arm, args.nros, args.cc, args.nm, work, configured
            )
            total_pairs += pairs
            total_probes += probes
            problems += [f"  [{arm}] {p.strip()}" for p in probs]
        if problems:
            print(
                "repr memory agreement FAILED - the C pack and the C++ pack's "
                f"repr(C) Rust glue describe different memory on {RUST_TARGET}:",
                file=sys.stderr,
            )
            for p in problems:
                print(p, file=sys.stderr)
            print(
                "\n  Both render from ONE lowered field. A disagreement is the two\n"
                "  pack type filters (`c_type`/`c_array_suffix` vs `cpp_repr_c_type`)\n"
                "  spelling it differently - fix the filter, not the number.",
                file=sys.stderr,
            )
            return 1
        print(
            f"repr memory agreement OK ({total_pairs} struct pairs, "
            f"{total_probes} probes, target {RUST_TARGET}, 2 storage arms)"
        )
        return 0
    finally:
        if args.keep:
            print(f"  probe sources kept in {work}")
        else:
            shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
