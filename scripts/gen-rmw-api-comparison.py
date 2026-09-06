#!/usr/bin/env python3
"""Generate the nano-ros RMW vs ROS 2 rmw comparison, as HTML.

THREE sources, no fourth:

  1. ROS 2's side   `docs/reference/rmw-implementation-signatures.txt`
                    (name / return / params / header, extracted from the Humble
                    headers by `rmw-api-inventory.py --signatures`)
  2. nano-ros' side `packages/core/nros-rmw-abi/include/nros/rmw_vtable.h`
                    for per-backend SLOTS, and the sibling ABI headers for
                    GLOBAL functions — the two are different things and the
                    table says which
  3. the reasons    `docs/reference/rmw-api-map.toml`, the authored map that
                    `just check rmw-api-parity` also reads. One file, two
                    consumers: two copies of a map is how a document ends up
                    describing a tree that moved.

Every row is one upstream symbol: what ROS 2 declares, what we provide, and the
reason when they differ — including when only the ARGUMENTS differ, which is the
case a name-only comparison silently passes.

  python3 scripts/gen-rmw-api-comparison.py           # rewrite
  python3 scripts/gen-rmw-api-comparison.py --check   # fail if it drifted
  python3 scripts/gen-rmw-api-comparison.py --html P  # also render a standalone page

`--html` exists because the book page is the SOURCE and not the thing anyone
reads outside the book: reviewing this table meant hand-wrapping the generated
markdown in a page shell, which had been done once by hand and left in `tmp/`
with no way to refresh it. One generator, two outputs — a second script would be
a second copy of the same three sources, which is the failure mode the docstring
above already warns about.

The shell is a shell ONLY. The comparison table ships its own `<style>` inside
the markdown, keyed on `color-scheme` so it follows the reader's theme in the
book and the OS outside it; restating any of that here would give the page two
palettes that drift.
"""
import argparse
import collections
import html
import importlib.util as _util
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOC = os.path.join(ROOT, "book", "src", "reference", "rmw-api-comparison.md")
ABI_DIR = os.path.join(ROOT, "packages", "core", "nros-rmw-abi", "include", "nros")


def _load(name, path):
    spec = _util.spec_from_file_location(name, os.path.join(ROOT, "scripts", path))
    mod = _util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _split_params(raw):
    """`['const rmw_publisher_t *publisher', 'size_t len']` — NAMES KEPT.

    The upstream extract drops parameter names deliberately (a renamed argument
    is not an ABI difference). Ours are kept because they are the only place the
    reader learns what the argument MEANS — `size_t` twice in a row is not a
    signature anyone can act on. Comparison still runs on TYPES; this is for the
    eye, not the diff.
    """
    raw = " ".join(raw.split())
    if raw in ("", "void"):
        return []
    out, depth, cur = [], 0, ""
    for ch in raw:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        if ch == "," and depth == 0:
            out.append(cur.strip())
            cur = ""
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def vtable_slots_named():
    """{slot: [param text with names]} straight from the vtable header."""
    src = open(os.path.join(ABI_DIR, "rmw_vtable.h"), encoding="utf-8").read()
    body = re.sub(r"/\*.*?\*/", " ", src, flags=re.S)
    body = re.sub(r"(?m)//.*$", " ", body)
    body = body[body.index("typedef struct nros_rmw_vtable_t {"):]
    body = body[: body.index("} nros_rmw_vtable_t;")]
    out = {}
    for m in re.finditer(r"\(\s*\*\s*([a-z_0-9]+)\s*\)\s*\(", body):
        depth, params = 1, ""
        for ch in body[m.end():]:
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    break
            params += ch
        out.setdefault(m.group(1), _split_params(params))
    return out


def global_signatures():
    """{name: (ret, [params])} for the plain exported ABI functions.

    These are NOT vtable slots: they are defined once for the image rather than
    per backend, which is a real distinction for a reader deciding where a
    behaviour can vary. `rmw_qos_profile_check_compatible` is the example — its
    answer must not differ by backend, and its useful call sites may run before
    any backend registers.
    """
    out = {}
    for fn in sorted(os.listdir(ABI_DIR)):
        if not fn.endswith(".h"):
            continue
        src = open(os.path.join(ABI_DIR, fn), encoding="utf-8").read()
        src = re.sub(r"/\*.*?\*/", " ", src, flags=re.S)
        src = re.sub(r"(?m)//.*$", " ", src)
        # A declaration, not a slot: `ret name(params);` at file scope. Slots
        # are `ret (*name)(params)` and are excluded by the absence of `(*`.
        for m in re.finditer(
            r"(?m)^\s*([A-Za-z_][A-Za-z0-9_ ]*?[\w*])\s+((?:nros_rmw|rmw)_[a-z_0-9]+)\s*\(([^;{]*?)\)\s*;",
            src,
            re.S,
        ):
            ret, name, params = m.group(1).strip(), m.group(2), m.group(3)
            params = " ".join(params.split())
            out.setdefault(name, (ret, _split_params(params)))
    return out


def sig(ret, params):
    inner = ", ".join(params) if params else "void"
    return f"{ret} ({inner})"


CSS = """<style>
/* Scoped to this page, and the palette follows the READER'S THEME, which is a
   choice in the book and an OS setting outside it.

   The hook is `color-scheme`, not a list of theme names. Every mdBook theme
   declares its own `--color-scheme: light|dark`; custom properties inherit, so
   re-applying it here lets `light-dark()` resolve against whatever theme is
   active — including a custom one this file has never heard of. Outside the
   book `--color-scheme` is unset, the fallback `light dark` applies, and the
   same declarations follow `prefers-color-scheme` instead.

   Three layers, and each is load-bearing:
     1. plain light values — what a browser without `light-dark()` keeps, since
        it drops the later declarations as unparseable;
     2. the named dark themes — so those same old browsers still get the dark
        palette on Coal/Navy/Ayu, which is the case the enumeration was for.
        Higher specificity, identical values, so it is inert where (3) works;
     3. `light-dark()` — everything else, no enumeration. */
.rmwcmp{color-scheme:var(--color-scheme,light dark);
--ret:#8250df;--fn:#0550ae;--ty:#116329;--pu:#8b8b9a;
--del:#cf222e;--delbg:#ffebe9;--add:#0a7d33;--addbg:#e6ffec;--renbg:#fff8c5;
--line:var(--table-border-color,#ddd);--chip:var(--table-header-bg,#f2f2f7)}
.coal .rmwcmp,.navy .rmwcmp,.ayu .rmwcmp{--ret:#d2a8ff;--fn:#79c0ff;--ty:#7ee787;
--pu:#8b8b9a;--del:#ff7b72;--delbg:#3d1c1f;--add:#56d364;--addbg:#12301c;--renbg:#3a3018}
.rmwcmp{--ret:light-dark(#8250df,#d2a8ff);--fn:light-dark(#0550ae,#79c0ff);
--ty:light-dark(#116329,#7ee787);--del:light-dark(#cf222e,#ff7b72);
--delbg:light-dark(#ffebe9,#3d1c1f);--add:light-dark(#0a7d33,#56d364);
--addbg:light-dark(#e6ffec,#12301c);--renbg:light-dark(#fff8c5,#3a3018);
--line:var(--table-border-color,light-dark(#ddd,#2c2c38));
--chip:var(--table-header-bg,light-dark(#f2f2f7,#1e1e28))}
.rmwcmp table{table-layout:fixed;width:100%;border-collapse:collapse;margin:0}
.rmwcmp th{font-size:11px;text-transform:uppercase;letter-spacing:.06em;
text-align:left;padding:.5rem .7rem;background:var(--chip);border:none}
.rmwcmp td{padding:.65rem .7rem;border:none;border-top:1px solid var(--line);
vertical-align:top}
.rmwcmp td.c{width:29%}.rmwcmp td.why{width:42%;font-size:12.5px;opacity:.86}
.rmwcmp pre{margin:0;padding:0;background:none;border:none;
font:12.5px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace;
white-space:pre-wrap;overflow-wrap:anywhere}
.rmwcmp .ret{color:var(--ret)}.rmwcmp .fn{color:var(--fn);font-weight:600}
.rmwcmp .ty{color:var(--ty)}.rmwcmp .pu{color:var(--pu)}
.rmwcmp .del{background:var(--delbg);color:var(--del);border-radius:3px;padding:0 .15em}
.rmwcmp .add{background:var(--addbg);color:var(--add);border-radius:3px;padding:0 .15em}
.rmwcmp .ren{background:var(--renbg);border-radius:3px;padding:0 .15em}
.rmwcmp .none{color:var(--del);font-weight:600;font-size:13px}
.rmwcmp .elsewhere{color:var(--fn);font-weight:600;font-size:13px}
.rmwcmp tr.inert td.c:nth-child(2) pre{opacity:.45}
/* status chip — the WHAT axis. It heads the REASON column, not the nano one:
   the two signature cells must stay line-for-line comparable. */
.rmwcmp .st{font-size:10.5px;font-weight:600;letter-spacing:.04em;
text-transform:uppercase;margin:0 0 .45rem;opacity:.95}
.rmwcmp .nosig{opacity:.35}
.rmwcmp .s-same,.rmwcmp .s-re-shaped{color:var(--add)}
.rmwcmp .s-re-mapped{color:var(--fn)}
.rmwcmp .s-not-supported{color:var(--del)}
.rmwcmp .s-not-implemented{color:var(--ret)}
.rmwcmp .answers{margin:0 0 .55rem;font-size:12px;opacity:.9}
.rmwcmp .ans{padding:.05rem 0 .05rem .8rem;position:relative}
.rmwcmp .ans:before{content:"\2192";position:absolute;left:0;opacity:.55}
.rmwcmp .wrap{border:1px solid var(--line);border-radius:8px;overflow:hidden;margin:1rem 0}
</style>"""


def fmt_sig(ret, params, name, is_slot, dropped=(), added=(), renamed=False, types=None):
    """One signature, format A: return / name / one argument per line.

    Collapsed to a single line at zero or one argument, where the vertical form
    buys nothing.

    `is_slot` decides `(*name)` versus `name`, which is the whole notation for
    "per-backend vtable slot" versus "global function defined once" — the
    C syntax already says it, so nothing else has to.
    """
    e = html.escape
    # Nested same-type quotes inside an f-string are PEP 701, i.e. Python 3.12+.
    # CI's python3 is 3.10, where this is a SyntaxError at IMPORT time — the
    # whole gate dies before it runs.
    ren_cls = " ren" if renamed else ""
    nm = f"<span class='fn{ren_cls}'>{e(name)}</span>"
    head = f"<span class=pu>(*</span>{nm}<span class=pu>)</span>" if is_slot else nm

    def ty(i, p):
        # Diff on the TYPE (names are not ABI); render whatever `params` holds,
        # which on our side includes the argument name.
        key = types[i] if types else p
        cls = "ty del" if key in dropped else ("ty add" if key in added else "ty")
        return f"<span class='{cls}'>{e(p)}</span>"

    r = f"<span class=ret>{e(ret)}</span>"
    if not params:
        return f"{r}\n{head}<span class=pu>(</span><span class=ty>void</span><span class=pu>)</span>"
    if len(params) == 1:
        return f"{r}\n{head}<span class=pu>(</span>{ty(0, params[0])}<span class=pu>)</span>"
    body = "<span class=pu>,</span>\n".join("  " + ty(i, p) for i, p in enumerate(params))
    return f"{r}\n{head}<span class=pu>(</span>\n{body}\n<span class=pu>)</span>"


def arg_rules():
    """The systematic reasons an argument list differs, from the map."""
    try:
        import tomllib  # Python 3.11+
    except ModuleNotFoundError:  # the repo's interpreter is 3.10
        import tomli as tomllib

    with open(os.path.join(ROOT, "docs", "reference", "rmw-api-map.toml"), "rb") as fh:
        return tomllib.load(fh).get("arg_rule", [])


def explain_args(up_params, our_params, rules):
    """([(title, why)], [unexplained]) for the arguments upstream has and we do not.

    Only upstream-side parameters are attributed: an argument we ADD is our
    shape and is described by the slot's own row, while one we DROP is a
    decision that needs a stated cause. An unexplained drop fails `--check`,
    because "the lists differ" is not a reason.
    """
    seen, unexplained = [], []
    for p in up_params:
        if p in our_params:
            continue
        hit = next((r for r in rules if r["match"] in p), None)
        if hit is None:
            unexplained.append(p)
        elif (hit["title"], hit["why"]) not in seen:
            seen.append((hit["title"], hit["why"]))
    return seen, unexplained


SURFACE_PREFIX = (
    ("executor:", "executor"),
    ("platform:", "platform"),
    ("build time:", "build time"),
    ("nros-serdes:", "serdes"),
    ("runtime:", "runtime"),
)
SURFACES = ("vtable", "global", "executor", "platform", "build time", "serdes",
            "runtime", "none")
STATUS_ORDER = ("same", "re-shaped", "re-mapped", "not-supported", "not-implemented")
CHIP = {"same": "●", "re-shaped": "●", "re-mapped": "◆",
        "not-supported": "✕", "not-implemented": "○"}


def surface_of(where, cstatus, answers):
    """Which surface actually provides the capability.

    `where` alone cannot say: `declined` is the answer for `rmw_wait`, which is
    five live vtable slots, and for `rmw_init_publisher_allocation`, which is
    nothing. `answers` carries the distinction, so read it first.
    """
    if cstatus == "not-supported":
        return "none"
    for a in answers:
        for prefix, col in SURFACE_PREFIX:
            if a.startswith(prefix):
                return col
    return {"layer": "serdes", "declined": "vtable"}.get(where, where)


def status_of(sym, row, where, identical, shared_targets, name):
    """(status, answers) — authored where it must be, derived where it can be.

    Derived: `same` and `re-shaped` come from the signatures, and a MERGE is
    visible in the map itself (two upstream symbols naming one slot). Authored:
    everything the types cannot show — whether an absent symbol was decomposed
    or dropped, and whether a declared-but-unfilled slot is a plan or a gap.
    """
    authored = row.get("status")
    answers = list(row.get("answers") or [])
    if authored:
        return authored, answers
    if where in ("vtable", "global"):
        if identical:
            return "same", answers
        if name in shared_targets:
            # N upstream symbols, one slot: answered, but not 1:1.
            return "re-mapped", answers or [name]
        return "re-shaped", answers
    # `check_status` in the parity gate rejects this before it can be rendered.
    return "re-mapped", answers


def build():
    parity = _load("_parity", "rmw-api-parity.py")
    shape = _load("_shape", "rmw-abi-shape.py")
    # A slot named by more than one upstream symbol is a MERGE — the capability
    # is answered, but not one-for-one, and that is invisible in either
    # signature. Counted from the map so it cannot fall out of date.
    _claims = collections.Counter(
        r.get("nano") for r in parity.MAP_ROWS.values() if r.get("nano")
    )
    shared_targets = {k for k, n in _claims.items() if n > 1}

    contract = parity.read_contract()
    # TWO views of the same extract. The comparison is on TYPES — a renamed
    # argument is not an ABI difference — while the page RENDERS names, because
    # our column beside it shows them and a bare `const rmw_publisher_t *` next
    # to `const rmw_publisher_t *publisher` reads as though upstream declared it
    # nameless. It does not; the extract simply used to drop them.
    upstream = shape.upstream_signatures()
    upstream_named = shape.upstream_signatures(with_names=True)
    slots, rets = shape.vtable_slots()
    globs = global_signatures()
    slots_named = vtable_slots_named()
    kinds = parity._slot_kinds()
    rules = arg_rules()

    rows, unexplained = [], {}
    tally = {"same": 0, "redesigned": 0, "absent": 0}
    matrix = collections.Counter()

    for sym in contract:
        where, detail = parity.MAP.get(sym, ("gap", ""))
        up_ret, up_params = upstream.get(sym, ("?", []))
        # What the page prints for the upstream side. `up_params` stays the
        # type-only list every comparison below is computed from.
        up_display = upstream_named.get(sym, (up_ret, up_params))[1]
        mechanical = sym[4:] if sym.startswith("rmw_") else sym

        our_html = ""
        name = ""
        status = "absent"
        causes = []
        inert = False
        note = detail if where in ("layer", "declined") else ""

        if where in ("vtable", "global"):
            if where == "vtable":
                name = re.split(r"[ ,(]", detail.strip())[0]
                inert = kinds.get(name) == "inert"
                our_ret, our_params = rets.get(name, "?"), slots.get(name, [])
                our_display = slots_named.get(name, our_params)
                is_slot = True
            else:
                name = sym
                our_ret, our_named = globs.get(sym, ("?", []))
                # Compare on types, show the names.
                our_params = [
                    " ".join(re.sub(r"\b[a-z_][a-z_0-9]*\s*(\[\s*\])?$", "", q).split())
                    for q in our_named
                ]
                our_display = our_named
                is_slot = False

            identical = our_params == up_params and our_ret == up_ret and name in (sym, mechanical)
            status = "same" if identical else "redesigned"
            renamed = name not in (sym, mechanical)
            dropped = [p for p in up_params if p not in our_params]
            added = [p for p in our_params if p not in up_params]
            if not identical:
                causes, missing = explain_args(up_params, our_params, rules)
                if missing:
                    unexplained[sym] = missing
            our_html = fmt_sig(
                our_ret, our_display, name, is_slot,
                dropped=(), added=added if not identical else (), renamed=renamed,
                types=our_params,
            )
            up_html = fmt_sig(
                up_ret, up_display, sym, False,
                dropped=dropped if not identical else (),
                types=up_params,
            )
        else:
            up_html = fmt_sig(up_ret, up_display, sym, False, types=up_params)

        map_row = parity.MAP_ROWS.get(sym, {})
        cstatus, answers = status_of(
            sym, map_row, where, status == "same", shared_targets, name if where == "vtable" else sym
        )
        surface = surface_of(where, cstatus, answers)
        matrix[(cstatus, surface)] += 1

        tally[status] += 1
        rows.append({
            "sym": sym, "where": where, "up": up_html, "ours": our_html,
            "status": status, "causes": causes, "note": note, "inert": inert,
            "cstatus": cstatus, "answers": answers, "surface": surface,
            "issue": map_row.get("issue"),
            "merge_n": _claims.get(name, 0) if where == "vtable" else 0,
            "slot_name": name,
            "renamed_to": (
                re.split(r"[ ,(]", detail.strip())[0]
                if where == "vtable" and re.split(r"[ ,(]", detail.strip())[0] not in (sym, mechanical)
                else ""
            ),
        })

    return contract, rows, tally, matrix, unexplained


def render(contract, rows, tally, matrix, _un):
    e = html.escape
    o = []
    w = o.append

    w("<!-- GENERATED by scripts/gen-rmw-api-comparison.py — do not edit by hand.")
    w("     Regenerate: python3 scripts/gen-rmw-api-comparison.py")
    w("     Gated by:   just check rmw-api-comparison -->")
    w("")
    w("# RMW API — every upstream symbol, side by side")
    w("")
    w("What ROS 2's `rmw` asks an implementation for, what nano-ros provides, and")
    w("the reason wherever the two differ. **Derived from three sources** — the ROS 2")
    w("signature extract, the nano-ros ABI headers, and the authored reason map")
    w("`docs/reference/rmw-api-map.toml` that `just check rmw-api-parity` also reads —")
    w("so a slot changing shape moves this page in the same commit or fails the gate.")
    w("")
    w("For the prose rationale behind the big divergences, see")
    w("[RMW API: Differences from upstream `rmw.h`](../design/rmw-vs-upstream.md).")
    w("")
    w("## How to read a row")
    w("")
    w("**The signature says which surface it is on.** nano-ros answers an upstream")
    w("symbol in one of two ways, and C syntax already distinguishes them:")
    w("")
    w("```c")
    w("rmw_ret_t (*count_publishers)(...)   // vtable slot — a function pointer,")
    w("                                     // may differ per backend")
    w("rmw_ret_t rmw_compare_gids_equal(...) // global — a plain exported function,")
    w("                                     // defined once for the image")
    w("```")
    w("")
    w("Slot names drop the `rmw_` prefix because that is genuinely the name in")
    w("`nros_rmw_vtable_t`; showing `rmw_count_publishers` on the right would flatter")
    w("the comparison.")
    w("")
    w("**Each side is spelled the way its own headers spell it.** Upstream writes")
    w("`const rmw_publisher_t * publisher`, we write `const rmw_publisher_t *publisher`,")
    w("and the two columns keep their own convention rather than being normalised to")
    w("a third that neither project uses. Parameter NAMES are shown on both sides;")
    w("the comparison behind the highlighting is on TYPES only, because a renamed")
    w("argument is not an ABI difference.")
    w("")
    w("**The chip says what we DID with the symbol**, which is a different question")
    w("from what the signature shows:")
    w("")
    w("| chip | means |")
    w("| --- | --- |")
    w("| ● `same` | identical signature and name |")
    w("| ● `re-shaped` | one slot, one symbol, different signature |")
    w("| ◆ `re-mapped` | answered, but NOT 1:1 — decomposed, merged, or off this seam. The arrows under it name what provides the capability |")
    w("| ✕ `not-supported` | a decision, permanent; the reason names the constraint |")
    w("| ○ `not-implemented` | a gap, with the issue tracking it |")
    w("")
    w("`same` and `re-shaped` are derived from the signatures, so the map cannot")
    w("assert a match the types deny. The other three are authored, because no")
    w("signature can say whether an absent symbol was decomposed or dropped — and")
    w("`not-implemented` must name an issue, so silence cannot turn a gap into a")
    w("decision.")
    w("")
    w("**Marks show the difference.** Red — upstream takes it, we do not. Green — we")
    w("take it, upstream does not. Yellow — the name differs from the mechanical one")
    w("(upstream minus `rmw_`). A row with **no marks is identical on both sides** and")
    w("carries no reason, because there is nothing to explain.")
    w("")
    w("**Argument names appear on the right only.** The ROS 2 side is read from a")
    w("signature extract that drops parameter names on purpose — a renamed argument is")
    w("not an ABI difference, and reporting one trains people to skim. Ours are kept")
    w("because they are the only place a reader learns what an argument *means*:")
    w("`size_t` twice in a row is not a signature anyone can act on. The comparison")
    w("itself runs on **types**, so a name never colours a row.")
    w("")
    w("**An empty right-hand cell** reads *rejected* when the symbol is deliberately")
    w("absent, or *answered elsewhere* when the capability ships outside the RMW seam —")
    w("in the executor, in codegen, or inside a backend. Both carry the reason.")
    w("")
    w("**A dimmed right-hand cell** is an *inert* slot: declared in the vtable, written")
    w("and read by nothing. A reserved shape, not a working capability (issue 0800).")
    w("")
    w("## What is being compared")
    w("")
    w("Not `rmw.h` against `rmw_vtable.h` — that comparison is wrong twice. Upstream")
    w("declares 177 `RMW_PUBLIC` functions, but most are utilities `rmw` itself")
    w("*defines*; an implementation links those, so comparing against 177 manufactures")
    w("~90 phantom gaps. And our vtable is only the backend seam: plenty of what")
    w("upstream calls rmw lives one layer up in the executor, one layer down in a")
    w("backend, or in codegen.")
    w("")
    w(f"So the contract is **empirical** — the {len(contract)} `rmw_*` symbols that")
    w("`librmw_fastrtps_cpp.so` and `librmw_zenoh_cpp.so` both define. Two independent")
    w("implementations with identical symbol sets is a better definition of \"what an")
    w("rmw must provide\" than any reading of the headers.")
    w("")
    # Two axes, because one was never enough. A row says WHAT we did with the
    # symbol; a column says WHERE the capability lives. The old four-line tally
    # collapsed both into `where`, so "decomposed into five slots" and "nothing
    # crosses this seam" shared the word `rejected` — and an inert slot,
    # declared with nothing behind it, was counted as answered.
    used_cols = [c for c in SURFACES if any(matrix[(st, c)] for st in STATUS_ORDER)]
    w("| | " + " | ".join(used_cols) + " | **total** |")
    w("| --- |" + " --- |" * (len(used_cols) + 1))
    for st in STATUS_ORDER:
        cells = [matrix[(st, c)] for c in used_cols]
        if not sum(cells):
            continue
        pretty = {"not-supported": "not supported — *by decision*",
                  "not-implemented": "not implemented — *tracked*"}.get(st, st)
        w(f"| {pretty} | " + " | ".join(str(c or "") for c in cells)
          + f" | **{sum(cells)}** |")
    totals = [sum(matrix[(st, c)] for st in STATUS_ORDER) for c in used_cols]
    w("| **total** | " + " | ".join(f"**{t}**" for t in totals)
      + f" | **{len(contract)}** |")
    w("")
    w("Read a row for what we did, a column for where it lives. Only")
    w("**not implemented** should shrink over time; **not supported** is the one")
    w("line that is a decision rather than a state, so it is expected to stay.")
    w("")
    # Vocabulary. Asked directly ("can we rename session to context?"), and the
    # answer is a real structural difference rather than a spelling, so it
    # belongs on the page beside the rows it explains.
    w("## Vocabulary — why we say *session* where upstream says *context*")
    w("")
    w("An `rmw_context_t` is two things in one struct: the **options** the")
    w("process was initialised with (`rmw_init_options_t` — domain id, enclave,")
    w("security, discovery) and an opaque **`impl`** the backend allocates to")
    w("hold its running state. Upstream can fuse them because both are")
    w("per-process, heap-allocated at `rmw_init` and freed at `rmw_shutdown`.")
    w("")
    w("Ours are not one thing. The options half is a **build-time POD** —")
    w("that is the recorded reason `rmw_init_options_{init,copy,fini}` are")
    w("declined, since \"copy\" is `=` and \"fini\" is nothing — and it lives on")
    w("`nros::Context`. The running half is a **`Session`**, one per backend,")
    w("created and destroyed through vtable slots, and an image may hold")
    w("several at once: the bridge entries run a zenoh ingress and a Cyclone or")
    w("XRCE egress in one process. There is no per-process singleton to name.")
    w("")
    w("So the rename is unavailable in both directions. `Context` is already")
    w("taken by the half that is *not* the backend state, and calling the")
    w("backend state `context` would promise a one-per-process object that a")
    w("bridge image visibly does not have. The word `session` is also the one")
    w("the transports themselves use (a zenoh session, an XRCE-DDS session),")
    w("which is what a reader of `create_session` is actually looking at.")
    w("")
    w("Read `rmw_context_t` on this page as *our `Context` plus one or more")
    w("`Session`s*, and the `rmw_init`/`rmw_shutdown` rows follow from that.")
    w("")
    w("## Every contract symbol")
    w("")
    w(CSS)
    w('<div class=rmwcmp><div class=wrap><table>')
    w("<tr><th>ROS 2</th><th>nano-ros</th><th>reason</th></tr>")
    for r in rows:
        cls = " class=inert" if r["inert"] else ""
        w(f"<tr{cls}>")
        w(f"<td class=c><pre>{r['up']}</pre></td>")
        # The chip is the STATUS axis, in the cell where the reader is already
        # looking for "what did they do". `where` used to carry this and could
        # not: "rejected" was the label on `rmw_wait`, which is five live slots.
        cs = r["cstatus"]
        note = ""
        if cs == "re-mapped":
            if r["merge_n"] > 1:
                note = f" · {r['merge_n']} upstream → 1 slot"
            elif len(r["answers"]) > 1:
                note = f" · 1 → {len(r['answers'])}"
            elif r["surface"] not in ("vtable", "global"):
                note = f" · {r['surface']}"
        elif cs == "not-supported":
            note = " · by decision"
        elif cs == "not-implemented":
            note = f" · issue {r['issue']:04d}" if r.get("issue") else ""
        chip = f"<div class='st s-{cs}'>{CHIP[cs]} {e(cs)}{e(note)}</div>"

        answers_html = ""
        shown = r["answers"] != [r.get("slot_name")] and r["answers"]
        if shown and (cs == "re-mapped"):
            items = "".join(f"<div class=ans>{e(a)}</div>" for a in shown)
            answers_html = f"<div class=answers>{items}</div>"

        # The two signature cells hold SIGNATURES and nothing else, so a reader
        # can run down them line by line and see the shapes differ. The chip and
        # the arrows are commentary, so they live in the commentary column —
        # putting them above the nano signature pushed it out of step with the
        # upstream one by however many lines they took.
        if r["ours"]:
            w(f"<td class=c><pre>{r['ours']}</pre></td>")
        else:
            w('<td class=c><span class=nosig>—</span></td>')
        bits = []
        if r["renamed_to"]:
            bits.append(f"<b>renamed</b> — the slot is <code>{e(r['renamed_to'])}</code>.")
        if r["inert"]:
            bits.append("<b>inert</b> — declared, written and read by nothing.")
        for title, why in r["causes"]:
            bits.append(f"<b>{e(title)}</b> — {e(why)}")
        if r["note"]:
            bits.append(e(r["note"]))
        w(f"<td class=why>{chip}{answers_html}{'<br><br>'.join(bits)}</td>")
        w("</tr>")
    w("</table></div></div>")
    w("")
    w("## Reproduce")
    w("")
    w("| command | asks |")
    w("| --- | --- |")
    w("| `just check rmw-api-parity` | is every contract symbol classified |")
    w("| `just check rmw-abi-shape` | does the vtable mirror it — name, args, return |")
    w("| `just check rmw-slot-producers` | which slots anything actually writes or reads |")
    w("| `python3 scripts/rmw-api-parity.py --contract` | re-derive the contract from an installed impl |")
    w("")
    return "\n".join(o) + "\n"


def self_test():
    """Negative control, on the NORMAL path.

    Exercises the shipping helpers rather than a copy of them: the signature
    formatter and the argument attribution are where a defect would silently
    produce a WRONG document — a row that reads "identical" because the diff
    never ran is worse than no document.
    """
    # A dropped argument must be attributed, and an unknown one must NOT be.
    rules = [{"match": "rcutils_allocator_t", "title": "t", "why": "w"}]
    causes, missing = explain_args(
        ["rcutils_allocator_t *", "struct wat_t *"], [], rules
    )
    if [t for t, _ in causes] != ["t"]:
        print("selftest: known parameter was not attributed", file=sys.stderr)
        return False
    if missing != ["struct wat_t *"]:
        print("selftest: unknown parameter was not reported", file=sys.stderr)
        return False

    # An argument present on both sides is neither dropped nor added.
    causes, missing = explain_args(["const char *"], ["const char *"], rules)
    if causes or missing:
        print("selftest: a matching parameter was treated as a difference", file=sys.stderr)
        return False

    # The slot form must be distinguishable from the global form — that is the
    # whole notation for "per-backend" vs "defined once".
    slot = fmt_sig("rmw_ret_t", [], "publish", True)
    glob = fmt_sig("rmw_ret_t", [], "rmw_publish", False)
    if "(*" not in slot or "(*" in glob:
        print("selftest: slot and global render the same", file=sys.stderr)
        return False

    # Zero and one argument collapse; two or more do not.
    if "\n  " in fmt_sig("rmw_ret_t", ["int a"], "x", True):
        print("selftest: a single argument was not collapsed", file=sys.stderr)
        return False
    if "\n  " not in fmt_sig("rmw_ret_t", ["int a", "int b"], "x", True):
        print("selftest: two arguments were not broken onto lines", file=sys.stderr)
        return False

    # A parameter NAME must never colour a row: the diff runs on `types`.
    marked = fmt_sig(
        "rmw_ret_t", ["const char *topic"], "x", True,
        added=["const char *"], types=["const char *"],
    )
    if "ty add" not in marked:
        print("selftest: diff did not use the parallel type list", file=sys.stderr)
        return False
    return True



# Where a `.md` cross-reference points once the page is read outside the book.
REPO_BLOB = "https://github.com/NEWSLabNTU/nano-ros/blob/main"

# The standalone page's shell. Deliberately NOT a palette: the comparison table
# carries its own inside the markdown, keyed on `color-scheme` so it follows the
# mdBook theme in the book and `prefers-color-scheme` outside it. Two palettes
# in two files is two things to keep in step.
HTML_SHELL = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RMW API \u2014 nano-ros vs ROS 2</title>
<style>
/* Page shell only. The table's own palette is the page's business and follows
   `color-scheme`; nothing here restates it. */
:root{color-scheme:light dark;
      --bg:light-dark(#fff,#14141a);--fg:light-dark(#1b1b23,#e6e6ef);
      --muted:light-dark(#5b5b6b,#a0a0b4);--line:light-dark(#e2e2ea,#2c2c38);
      --chip:light-dark(#f5f5f9,#1e1e28);--code:light-dark(#f5f5f9,#1e1e28);
      --link:light-dark(#0550ae,#79c0ff)}
html{background:var(--bg)}
body{margin:0 auto;padding:2.5rem 1.5rem 6rem;max-width:1180px;background:var(--bg);
     color:var(--fg);font:15px/1.65 -apple-system,BlinkMacSystemFont,"Segoe UI",
     Roboto,Helvetica,Arial,sans-serif}
h1{font-size:1.9rem;line-height:1.2;margin:0 0 .6rem;text-wrap:balance}
h2{font-size:1.25rem;margin:2.6rem 0 .8rem;padding-top:1.1rem;
   border-top:1px solid var(--line);text-wrap:balance}
h3{font-size:1.02rem;margin:1.8rem 0 .5rem}
p,li{max-width:74ch}
a{color:var(--link)}
code{background:var(--code);padding:.12em .34em;border-radius:4px;
     font:13px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace}
pre{background:var(--code);padding:.9rem 1rem;border-radius:8px;overflow-x:auto}
pre code{background:none;padding:0}
/* the summary/markdown tables \u2014 the comparison table has its own rules */
table:not(.rmwcmp table){border-collapse:collapse;margin:1.2rem 0}
table:not(.rmwcmp table) th,table:not(.rmwcmp table) td{
  border:1px solid var(--line);padding:.4rem .8rem;text-align:left;
  font-variant-numeric:tabular-nums}
table:not(.rmwcmp table) th{background:var(--chip)}
.rmwcmp .wrap{overflow-x:auto}
</style></head><body>
%s
</body></html>
"""


def write_html(markdown_text, dest, fragment=False):
    """Render the generated page into a standalone HTML file.

    The page is already mostly HTML \u2014 the comparison table and its style
    block are emitted as raw markup \u2014 so this converts the surrounding
    prose and leaves the rest alone.
    """
    try:
        import markdown as _md
    except ImportError:
        print(
            "gen-rmw-api-comparison: --html needs python-markdown.\n"
            "  pip install --user markdown   (or drop --html; the book page is\n"
            "  the source and `just book` renders it with the rest of the book)",
            file=sys.stderr,
        )
        return 1
    body = _md.markdown(
        markdown_text,
        extensions=["tables", "fenced_code", "attr_list"],
        output_format="html5",
    )
    # In the book, `../design/x.md` resolves. In a standalone page it is a dead
    # link to a file the reader does not have, which is worse than no link --
    # the one cross-reference on this page is the prose rationale for the very
    # divergences the table lists. Point it at the repo instead. Paths are
    # relative to this document's own directory, `book/src/reference/`.
    def _repo_link(m):
        href = m.group(1)
        target = os.path.normpath(os.path.join("book/src/reference", href))
        return 'href="%s/%s"' % (REPO_BLOB, target)

    body = re.sub(r'href="((?!https?:)[^"]*\.md)"', _repo_link, body)
    os.makedirs(os.path.dirname(os.path.abspath(dest)) or ".", exist_ok=True)
    with open(dest, "w") as fh:
        # A FRAGMENT is the same page without the shell, for a host that wraps
        # it in one of its own. The table's `<style>` rides inside the markdown,
        # so the fragment keeps its palette; only the page furniture is missing.
        fh.write(body if fragment else HTML_SHELL % body)
    print(f"wrote {dest}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument(
        "--html",
        metavar="PATH",
        help="also render a standalone HTML page (implies a rewrite unless --check)",
    )
    ap.add_argument(
        "--html-fragment",
        metavar="PATH",
        help="render the page BODY only, for a host that supplies its own shell",
    )
    args = ap.parse_args()

    if not self_test():
        print("gen-rmw-api-comparison: selftest failed — output is not trustworthy.", file=sys.stderr)
        return 1

    contract, rows, tally, matrix, unexplained = build()
    if unexplained:
        print(
            "ERROR: %d slot(s) drop an upstream argument no rule explains:" % len(unexplained),
            file=sys.stderr,
        )
        for sym, params in sorted(unexplained.items()):
            print("  %-52s %s" % (sym, ", ".join(params)), file=sys.stderr)
        print(
            "\n  Add an `[[arg_rule]]` to docs/reference/rmw-api-map.toml naming the\n"
            "  parameter TYPE and what we do instead. \"The lists differ\" is not a reason.",
            file=sys.stderr,
        )
        return 1
    # An inert slot is declared and unfilled. Under the old taxonomy it counted
    # as ANSWERED, which is the state this axis exists to stop being invisible:
    # say whether it is a plan, a decision, or a gap.
    silent = [
        r["sym"] for r in rows
        if r["inert"] and r["cstatus"] in ("same", "re-shaped")
    ]
    if silent:
        print(
            "ERROR: %d inert slot(s) are counted as answered:" % len(silent),
            file=sys.stderr,
        )
        for sym in silent:
            print("  %s" % sym, file=sys.stderr)
        print(
            "\n  An inert slot is DECLARED and nothing fills it. Give each a `status`\n"
            "  in docs/reference/rmw-api-map.toml:\n"
            "    re-mapped       the capability ships elsewhere; `answers` names where\n"
            "    not-supported   a decision, permanent, with a reason\n"
            "    not-implemented a gap, with `issue = NNNN`\n",
            file=sys.stderr,
        )
        return 1

    new = render(contract, rows, tally, matrix, unexplained)

    if args.check:
        old = open(DOC).read() if os.path.exists(DOC) else ""
        if new != old:
            print(
                "ERROR: book/src/reference/rmw-api-comparison.md is stale — regenerate with\n"
                "  python3 scripts/gen-rmw-api-comparison.py\n"
                "and commit. It derives from the headers and the map the parity gate\n"
                "reads, so drift means one of those moved and the document did not.",
                file=sys.stderr,
            )
            subprocess.run(["git", "--no-pager", "diff", "--stat", "--", DOC], cwd=ROOT)
            return 1
        print(f"rmw-api-comparison OK ({len(contract)} symbols).")
        if args.html:
            return write_html(new, args.html)
        if args.html_fragment:
            return write_html(new, args.html_fragment, fragment=True)
        return 0

    with open(DOC, "w") as fh:
        fh.write(new)
    print(
        f"wrote book/src/reference/rmw-api-comparison.md — {len(contract)} symbols "
        "(" + ", ".join(
            f"{sum(matrix[(st, c)] for c in SURFACES)} {st}"
            for st in STATUS_ORDER
            if sum(matrix[(st, c)] for c in SURFACES)
        ) + ")"
    )
    if args.html:
        return write_html(new, args.html)
    if args.html_fragment:
        return write_html(new, args.html_fragment, fragment=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
