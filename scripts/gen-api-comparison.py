#!/usr/bin/env python3
"""Generate the three user-API comparison pages (C, C++, Rust).

MECHANICAL. Two code inputs, one authored input, nothing inferred:

  * OUR surface        parsed from the headers and crates, by the same
                       extractors `api-parity.py` uses -- clang JSON AST for
                       C/C++, rustdoc JSON for Rust. Never a hand list.
  * ROS 2's surface    `docs/reference/api-surface/{rclc,rclcpp,rclrs}.json`,
                       recorded from an installed ROS 2 by `--refresh`.
  * WHY, and WHAT      `docs/reference/api-parity-ledger/*.json`. This is the
    ANSWERS IT         only file a human writes. `why` carries the reason a
                       row diverges; `provides` names our items that answer an
                       upstream one when the mapping is not 1:1.

The correlation itself -- which of the seven states a row is in -- is computed,
not authored. A state is a function of the bucket the correlator assigns and
the verdict the ledger records, and both of those already have gates.

Adding a reason or a re-mapping arrow means editing the ledger and re-running
this. It must never mean editing a page.

Usage:
    python3 scripts/gen-api-comparison.py                 # -> tmp/api-comparison/
    python3 scripts/gen-api-comparison.py --out DIR
    python3 scripts/gen-api-comparison.py --urls urls.json   # cross-page nav
    python3 scripts/gen-api-comparison.py --self-test
"""

import argparse
import collections
import datetime
import difflib
import html
import importlib.util
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PAGES = os.path.join(ROOT, "scripts/api_parity/pages")
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, os.path.join(ROOT, "scripts/api_parity"))


def load_api_parity():
    """`api-parity.py` has a dash in its name, so it needs an explicit loader."""
    spec = importlib.util.spec_from_file_location(
        "api_parity_main", os.path.join(ROOT, "scripts/api-parity.py")
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------- states
# Ordered as the legend reads them. The glyph is not decoration: state must be
# legible without colour, and seven hues alone would not be.
STATES = [
    ("same", "same", "●"),
    ("reshaped", "re-shaped", "●"),
    ("renamed", "renamed", "●"),
    ("remapped", "re-mapped", "◆"),
    ("rejected", "rejected", "✕"),
    ("missing", "not implemented", "○"),
    ("ours", "ours only", "✥"),
]

VERDICT_STATE = {
    "declined": "rejected",
    "gap": "missing",
    "extension": "ours",
    "divergence": "reshaped",
}

# RFC-0087's four dispositions, rendered as a chip beside the state.
#
# The state answers "what did WE do"; the disposition answers "what does a
# PORTING USER GET", and they are not the same question -- `rejected` covers
# both "the name does not exist" and "the name exists and refuses loudly at
# compile time", which is the whole difference between an hour and a week for
# whoever is porting the file. It is a SEPARATE chip and not a ninth state for
# that reason: state is derived from bucket + verdict, disposition is authored,
# and collapsing them would make one un-derivable from the other.
#
# Optional today (phase-417 W0.b); W-M2 is the pass that fills it in. A row
# without one renders no chip rather than a placeholder -- an "unknown" badge on
# ~700 rows is noise, and the count is already reported by `api-parity.py`.
DISPOSITION_LABEL = {
    "adopt": "adopt",
    "adopt-bounded": "adopt · bounded",
    "refuse-loud": "refuses loudly",
    "absent": "absent",
}


def state(row):
    """The seven-way state, derived -- never authored.

    Order matters. `provides` wins over the verdict because an m-to-n mapping
    is the more specific fact: a row can be BOTH a divergence and answered by
    three of our items, and the arrow is what a reader cannot reconstruct.
    """
    if row["bucket"] == "same":
        return "same"
    if row.get("provides"):
        return "remapped"
    if row.get("verdict") == "rename" or row["bucket"] == "systematic":
        return "renamed"
    return VERDICT_STATE.get(row.get("verdict"), "reshaped")


# ---------------------------------------------------------------- signatures
# Both sides spell the same concept differently BY CONSTRUCTION -- `rcl_node_t`
# against `nros_node_t`, `rclcpp::QoS` against `nros::QoS`. Diffing raw type
# strings would therefore mark every argument of every row as changed, which is
# the same as marking none: the column would carry no information at all.
#
# So the diff runs on types with the vendor rename normalised away. What
# survives is a real difference -- `rmw_qos_profile_t` against `nros_qos_t`
# stays flagged, because a profile struct and a flat QoS word ARE different
# shapes, and that is exactly what a reader needs to see.
VENDOR_PREFIX = re.compile(r"\b(rmw|rcl|rclc|rcutils|rosidl|nros)_")
NAMESPACE = re.compile(r"\b(rclcpp|rclrs|nros|std)::")
QUALIFIERS = re.compile(r"\b(const|struct|enum|class|typename|mut)\b")


def norm_type(text):
    text = QUALIFIERS.sub(" ", text or "")
    text = NAMESPACE.sub("", VENDOR_PREFIX.sub("", text))
    return re.sub(r"\s+", "", text).lower()


# The page can render OUR side under ROS 2's own spelling, so that a reader
# comparing the two columns sees SHAPE and not naming noise. This is a VIEW,
# never a claim about the code: `nros_take` is not called `rcl_take`, and the
# toggle is off by default and labelled for exactly that reason.
#
# The substitution is per-language because the three upstreams disagree about
# what our prefix even maps to: rcl/rclc for C, the `rclcpp::` namespace for
# C++, the `rclrs::` one for Rust.
ALIAS_PREFIX = {
    "c": [("nros_", "rcl_")],
    "cpp": [("nros::", "rclcpp::"), ("nros_", "rcl_")],
    "rust": [("nros::", "rclrs::")],
}


def aliased(text, lang):
    """`nros_take` -> `rcl_take` for display only.

    Longest prefix first: `nros::` must be tried before `nros_`, or a C++ name
    is rewritten to `rcl_:` and the toggle produces nonsense.
    """
    if not text:
        return text
    for src, dst in ALIAS_PREFIX.get(lang, []):
        text = text.replace(src, dst)
    return text


def param_text(param):
    """Type plus name. The NAME is kept even though the diff ignores it: it is
    the only place a reader learns what the argument MEANS, and `size_t` twice
    in a row is not a signature anyone can act on."""
    name = param.get("name")
    return ((param.get("type") or "") + (" " + name if name else "")).strip()


def diff_params(theirs, ours):
    """Per-argument (display, flag) for each side; flag is "", "del" or "add".

    A sequence diff, not a zip: an argument DROPPED from the middle would
    otherwise mis-align every argument after it and report the whole tail as
    changed.
    """
    tn = [norm_type(p.get("type")) for p in theirs]
    on = [norm_type(p.get("type")) for p in ours]
    tflag = [""] * len(tn)
    oflag = [""] * len(on)
    matcher = difflib.SequenceMatcher(None, tn, on, autojunk=False)
    for tag, i1, i2, j1, j2 in matcher.get_opcodes():
        if tag in ("delete", "replace"):
            for i in range(i1, i2):
                tflag[i] = "del"
        if tag in ("insert", "replace"):
            for j in range(j1, j2):
                oflag[j] = "add"
    return ([[param_text(p), f] for p, f in zip(theirs, tflag)],
            [[param_text(p), f] for p, f in zip(ours, oflag)])


def overload_params(item):
    ovs = (item or {}).get("overloads") or []
    return (ovs[0].get("params") or []) if ovs else []


def side(item, flags):
    """One signature cell: {r: ret, n: name, p: [[text, flag]], k: kind, x: n}.

    `p` absent means the item is not callable -- a type, alias, const or trait --
    and the cell renders as the name alone rather than an empty parameter list.
    """
    if not item:
        return None
    cell = {"n": item.get("qual") or item.get("key") or "", "k": item.get("kind") or ""}
    ovs = item.get("overloads") or []
    if ovs:
        cell["r"] = ovs[0].get("ret") or ""
        cell["p"] = flags
        if len(ovs) > 1:
            cell["x"] = len(ovs) - 1
    return cell


def sig(item):
    """Flat one-line signature. Kept for the self-test and for callers that want
    plain text; the page renders from the structured cell instead."""
    if not item:
        return ""
    qual = item.get("qual") or item.get("key") or ""
    overloads = item.get("overloads") or []
    if not overloads:
        kind = item.get("kind") or ""
        if kind in ("type", "alias", "enum", "struct", "const", "trait"):
            return ("%s %s" % (kind, qual)).strip()
        return qual
    first = overloads[0]
    params = ", ".join(param_text(p) for p in first.get("params", []))
    extra = ""
    if len(overloads) > 1:
        n = len(overloads) - 1
        extra = "   /* +%d overload%s */" % (n, "" if n == 1 else "s")
    return ("%s %s(%s)%s" % (first.get("ret") or "", qual, params, extra)).strip()


# ---------------------------------------------------------------- grouping
TOPIC_TITLE = {
    "init": "Context & initialisation",
    "node": "Node",
    "pubsub": "Publish / subscribe",
    "service": "Services",
    "action": "Actions",
    "param": "Parameters",
    "timer": "Timers & clocks",
    "exec": "Executor & waiting",
    "qos": "Quality of service",
    "lifecycle": "Lifecycle",
    "log": "Logging",
    "graph": "Graph introspection",
    "serde": "Serialisation",
    "types": "Shared types",
    "other": "Everything else",
}

# C++ sections are the ledger's own stages. Header FILENAMES would mix ours
# with rclcpp's, giving one concept two vocabularies (`subscription.hpp` beside
# `subscription_base.hpp`) and a section list nobody can scan.
def cpp_section(row, topic):
    return TOPIC_TITLE.get(topic, topic)


# Rust audience is the crate that DEFINES the item. Every Rust item is
# `nros::`-qualified because the facade re-exports the whole tree, so grouping
# on the qualified path puts 1086 of 1778 rows in one bucket and says nothing.
FACADE = "Facade an application writes against"
RUNTIME = "Node, executor & component runtime"
PARAMS = "Parameters"
SEAM = "Backend-author seam (RMW trait)"
WIRE = "Serialisation & wire"
CORE = "Core types"

RUST_CRATE_AUD = [
    ("packages/api/nros/", FACADE),
    ("packages/core/nros-node/", RUNTIME),
    ("packages/core/nros-params/", PARAMS),
    ("packages/core/nros-rmw/", SEAM),
    ("packages/platform/", SEAM),
    ("packages/core/nros-serdes/", WIRE),
    ("packages/core/nros-core/", CORE),
]

# A `theirs-only` row has no item of ours to locate, so it is placed by rclrs's
# own module -- the row is a statement about THEIR surface.
RCLRS_AUD = {
    "node": FACADE, "publisher": FACADE, "subscription": FACADE,
    "client": FACADE, "service": FACADE, "qos": FACADE, "action": FACADE,
    "timer": FACADE, "logging": FACADE,
    "parameter": PARAMS,
    "context": RUNTIME, "executor": RUNTIME, "worker": RUNTIME, "wait_set": RUNTIME,
    "rcl_bindings": SEAM,
    "dynamic_message": WIRE, "serialized_message": WIRE, "vendor": WIRE,
}


def rust_section(row):
    ours = row.get("ours")
    if ours:
        header = ours.get("header") or ""
        for prefix, name in RUST_CRATE_AUD:
            if header.startswith(prefix):
                return name
    theirs = row.get("theirs")
    if theirs:
        seg = (theirs.get("header") or "").replace("rclrs/src/", "").split("/")[0]
        return RCLRS_AUD.get(seg.replace(".rs", ""), CORE)
    return CORE


def owner(key):
    return key.split("::")[0] if "::" in key else key


# ---------------------------------------------------------------- extraction
def collect(langs):
    """Run the correlator and join each row to its ledger entry."""
    ap = load_api_parity()
    ledger = ap.load_ledger()
    out = {}
    with tempfile.TemporaryDirectory() as tmp:
        for lang in langs:
            rows, prov, _removed = ap.run_lang(lang, tmp)
            buckets = {r["key"]: r["bucket"] for r in rows}
            recs = []
            for r in rows:
                entry, inherited = ap.lookup(ledger, lang, r["key"], r["bucket"], buckets)
                entry = entry or {}
                topic = ap.row_topic(r)
                ours, theirs = r.get("ours"), r.get("theirs")
                rec = {
                    "key": r["key"],
                    "bucket": r["bucket"],
                    "verdict": entry.get("verdict") or "",
                    "why": entry.get("why") or "",
                    "provides": entry.get("provides") or [],
                    "ours": ours,
                    "theirs": theirs,
                }
                rec["s"] = state(rec)
                tflags, oflags = diff_params(overload_params(theirs),
                                            overload_params(ours))
                tcell, ocell = side(theirs, tflags), side(ours, oflags)
                # the ROS 2-spelled view of OUR side, precomputed so the toggle
                # is a class swap rather than a re-render
                if ocell:
                    ocell["a"] = aliased(ocell["n"], lang)
                    if ocell.get("r"):
                        ocell["ar"] = aliased(ocell["r"], lang)
                    if ocell.get("p"):
                        ocell["ap"] = [[aliased(txt, lang), f] for txt, f in ocell["p"]]
                recs.append({
                    "k": r["key"],
                    "s": rec["s"],
                    "b": r["bucket"],
                    "v": rec["verdict"],
                    "T": tcell,
                    "O": ocell,
                    "ren": 1 if rec["s"] == "renamed" else 0,
                    "w": rec["why"],
                    "p": rec["provides"],
                    "i": 1 if inherited else 0,
                    # RFC-0087 disposition -- authored, optional, never inferred
                    # from the verdict. "" means nobody has classified this row.
                    "d": entry.get("disposition") or "",
                    # issue 1020's two surfaces. `su` is "ported" when the
                    # compat shim is the ONLY way to reach our side of this
                    # row; `nb` carries the native answer when it differs from
                    # the ported one, so a `same` the shim produced can never
                    # read as the native API having closed the gap.
                    "su": r.get("surface") or "",
                    "nb": (r.get("native_bucket")
                           if r.get("native_bucket") != r["bucket"] else ""),
                    "g": (TOPIC_TITLE.get(topic, topic) if lang == "c"
                          else owner(r["key"])),
                    "sec": ("" if lang == "c"
                            else cpp_section(rec, topic) if lang == "cpp"
                            else rust_section(rec)),
                })
            out[lang] = {"rows": recs, "prov": prov}
    return out


# ---------------------------------------------------------------- rendering
TITLES = {
    "c": ("nano-ros C API vs rclc",
          "Every C entry point in nano-ros set against rclc and rcl, item for item."),
    "cpp": ("nano-ros C++ API vs rclcpp",
            "The nano-ros C++ surface against rclcpp, grouped by the type that owns each member."),
    "rust": ("nano-ros Rust API vs rclrs",
             "The nano-ros Rust surface against rclrs, grouped by who writes against it."),
}
NAVNAME = {"c": "C · rclc", "cpp": "C++ · rclcpp", "rust": "Rust · rclrs"}
LAYOUT = {"c": "flat", "cpp": "sectioned", "rust": "sectioned"}


def provenance_line(lang, prov):
    """Say which ROS 2 the page was compared against, from the recorded surface."""
    if lang == "c":
        ref = (prov.get("rclc_ref") or "")[:7]
        return "rclc + rcl · ROS 2 %s%s" % (
            prov.get("distro", "?"), (" · rclc " + ref) if ref else "")
    if lang == "cpp":
        return "rclcpp · ROS 2 %s" % prov.get("distro", "?")
    ref = (prov.get("ref") or "")[:7]
    return "rclrs %s%s" % (prov.get("version", "?"), (" · " + ref) if ref else "")


def nav(cur, urls):
    out = []
    for lang in ("c", "cpp", "rust"):
        if lang == cur:
            out.append('<span aria-current="page">%s</span>' % NAVNAME[lang])
        elif urls.get(lang):
            out.append('<a href="%s">%s</a>' % (html.escape(urls[lang]), NAVNAME[lang]))
        else:
            out.append("<span>%s</span>" % NAVNAME[lang])
    return "".join(out)


def render(lang, data, urls, stamp):
    shell = open(os.path.join(PAGES, "page.html")).read()
    css = open(os.path.join(PAGES, "page.css")).read()
    js = open(os.path.join(PAGES, "page.js")).read()
    rows = data["rows"]
    title, sub = TITLES[lang]
    counts = collections.Counter(r["s"] for r in rows)
    prov = provenance_line(lang, data["prov"])
    payload = {"lang": lang, "layout": LAYOUT[lang], "rows": rows,
               "counts": dict(counts), "prov": prov, "stamp": stamp}
    # `</` inside the embedded JSON would close the <script> early.
    blob = json.dumps(payload, ensure_ascii=False).replace("</", "<\\/")
    return (shell
            .replace("/*CSS*/", css)
            .replace("/*JS*/", js)
            .replace("__TITLE__", html.escape(title))
            .replace("__SUB__", html.escape(sub))
            .replace("__PROV__", html.escape(prov))
            .replace("__STAMP__", html.escape(stamp))
            .replace("__LANG__", lang)
            .replace("__TOTAL__", str(len(rows)))
            .replace("__NAV__", nav(lang, urls))
            .replace('"__DATA__"', blob)), counts


# ---------------------------------------------------------------- self-test
def self_test():
    """The derived parts must stay derived, and the embed must stay safe."""
    fails = []

    def check(cond, msg):
        if not cond:
            fails.append(msg)

    # state() is a pure function of bucket + verdict + provides
    check(state({"bucket": "same", "verdict": "", "provides": []}) == "same",
          "an exact correlation is not `same`")
    check(state({"bucket": "theirs-only", "verdict": "gap", "provides": []}) == "missing",
          "a gap is not `not implemented`")
    # the ordering that matters: an m-to-n arrow outranks the verdict, because a
    # row can be both a divergence and answered by three of our items
    check(state({"bucket": "differs", "verdict": "divergence",
                 "provides": ["nros::A"]}) == "remapped",
          "`provides` did not outrank the verdict")
    check(state({"bucket": "systematic", "verdict": "", "provides": []}) == "renamed",
          "a systematic transform is not `renamed`")
    check(state({"bucket": "ours-only", "verdict": "extension", "provides": []}) == "ours",
          "an extension is not `ours only`")
    # a gap must never be silently arrowed -- that would make the page assert
    # the opposite of its own chip
    check(state({"bucket": "theirs-only", "verdict": "gap", "provides": []}) != "remapped",
          "an unarrowed gap rendered as re-mapped")

    # signatures come from the parsed record, never from a name
    s = sig({"qual": "nros_take", "kind": "function", "overloads": [
        {"params": [{"name": "sub", "type": "nros_subscription_t *"}], "ret": "nros_ret_t"}]})
    check(s == "nros_ret_t nros_take(nros_subscription_t * sub)", "signature render: %r" % s)
    check(sig(None) == "", "a missing item did not render empty")
    check("+1 overload */" in sig({"qual": "f", "overloads": [
        {"params": [], "ret": "int"}, {"params": [], "ret": "int"}]}),
        "overload count not reported")

    # the vendor rename must NOT read as an argument difference -- otherwise
    # every row diffs on every argument, which is the same as diffing on none
    check(norm_type("rcl_node_t *") == norm_type("nros_node_t *"),
          "the vendor prefix survived normalisation")
    check(norm_type("const rcl_subscription_t *") == norm_type("struct nros_subscription_t *"),
          "qualifiers or `struct` survived normalisation")
    check(norm_type("rclcpp::QoS") == norm_type("nros::QoS"), "namespace not normalised")
    # ...but a genuinely different shape must stay flagged
    check(norm_type("rmw_qos_profile_t") != norm_type("nros_qos_t"),
          "normalisation erased a real type difference")

    # a dropped MIDDLE argument must not mis-align the tail
    tf, of = diff_params(
        [{"type": "rcl_node_t *", "name": "node"},
         {"type": "rcl_allocator_t", "name": "alloc"},
         {"type": "size_t", "name": "n"}],
        [{"type": "nros_node_t *", "name": "node"},
         {"type": "size_t", "name": "n"}])
    check([f for _, f in tf] == ["", "del", ""], "middle drop mis-aligned: %r" % tf)
    check([f for _, f in of] == ["", ""], "our side wrongly flagged: %r" % of)

    # an argument only WE take is an addition, on our side alone
    tf, of = diff_params([{"type": "rcl_node_t *"}],
                         [{"type": "nros_node_t *"}, {"type": "nros_qos_t"}])
    check([f for _, f in tf] == [""], "upstream flagged for our addition")
    check([f for _, f in of] == ["", "add"], "addition not flagged: %r" % of)

    # a non-callable item renders as a name, not as an empty argument list
    cell = side({"qual": "nros::QoS", "kind": "type"}, [])
    check("p" not in cell, "a type rendered with a parameter list")
    check(side(None, []) is None, "a missing side did not render as absent")

    # the ROS 2-spelling VIEW: longest prefix first, or a C++ name is rewritten
    # to `rcl_:` and the toggle produces nonsense
    check(aliased("nros::Subscription::take", "cpp") == "rclcpp::Subscription::take",
          "C++ namespace alias: %r" % aliased("nros::Subscription::take", "cpp"))
    check(aliased("nros_take", "c") == "rcl_take", "C prefix alias")
    check(aliased("nros::Node", "rust") == "rclrs::Node", "Rust namespace alias")
    check("rcl_:" not in aliased("nros::QoS", "cpp"), "prefix order produced `rcl_:`")
    # it must never touch the ROS 2 side, and never reach an unrelated word
    check(aliased("rcl_node_t *", "c") == "rcl_node_t *", "alias mangled upstream text")
    check(aliased("", "c") == "", "alias on empty text")

    # every state has a glyph, so the page is legible without colour
    check(len({g for _, _, g in STATES}) >= 4, "states are not glyph-distinguishable")
    check(len(STATES) == 7, "state count changed without updating the legend")

    # the embed escape must survive a ledger reason containing markup
    blob = json.dumps({"w": "see </script> and <b>"}, ensure_ascii=False).replace("</", "<\\/")
    check("</script>" not in blob, "embedded JSON can close the script tag")

    # ---- phase-417 W0.b: the RFC-0087 disposition ------------------------
    # A SECOND axis, so it must not perturb the derived state. If it ever did,
    # an authored field would be silently deciding a computed one.
    check(state({"bucket": "theirs-only", "verdict": "declined", "provides": [],
                 "disposition": "refuse-loud"}) == "rejected",
          "the disposition changed the derived state")
    # The four labels are RFC-0087's, and the page's JS copy must agree with the
    # generator's -- two spellings of one vocabulary is how a chip goes blank.
    ap = load_api_parity()
    check(set(DISPOSITION_LABEL) == set(ap.DISPOSITIONS),
          "the page's dispositions disagree with the ledger's: %r vs %r"
          % (sorted(DISPOSITION_LABEL), sorted(ap.DISPOSITIONS)))
    js = open(os.path.join(PAGES, "page.js")).read()
    for d in ap.DISPOSITIONS:
        check('"%s":' % d in js, "page.js has no label for disposition %r" % d)
        check(".d-%s{" % d in open(os.path.join(PAGES, "page.css")).read(),
              "page.css has no chip style for disposition %r" % d)

    # ---- phase-417 W0.a: the two surfaces --------------------------------
    # The page must be able to SAY which surface a row is on, or the C++ page
    # merges the two questions again one layer down from where issue 1020
    # separated them.
    check('r.su === "ported"' in js, "page.js does not mark a compat-shim row")
    check("r.nb" in js, "page.js does not show the native-surface answer")
    check(".surf{" in open(os.path.join(PAGES, "page.css")).read(),
          "page.css has no style for the surface note")

    # templates must still carry every placeholder the renderer fills
    shell = open(os.path.join(PAGES, "page.html")).read()
    for token in ("/*CSS*/", "/*JS*/", "__TITLE__", "__SUB__", "__PROV__",
                  "__STAMP__", "__TOTAL__", "__NAV__", '"__DATA__"'):
        check(token in shell, "page.html lost the %s placeholder" % token)

    for f in fails:
        print("  FAIL: " + f)
    print("gen-api-comparison self-test: %d check(s) failed" % len(fails))
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "tmp/api-comparison"))
    ap.add_argument("--urls", help="JSON {lang: url} for the cross-page nav")
    ap.add_argument("--langs", default="c,cpp,rust")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    urls = json.load(open(args.urls)) if args.urls else {}
    stamp = os.environ.get("NROS_DOC_DATE") or datetime.date.today().isoformat()
    langs = [l.strip() for l in args.langs.split(",") if l.strip()]
    os.makedirs(args.out, exist_ok=True)

    data = collect(langs)
    for lang in langs:
        page, counts = render(lang, data[lang], urls, stamp)
        path = os.path.join(args.out, "api-%s.html" % lang)
        open(path, "w").write(page)
        print("%-5s %5d rows  %7.1f KB  %s" % (
            lang, len(data[lang]["rows"]), len(page) / 1024, path))
        print("      " + "  ".join(
            "%s=%d" % (k, counts.get(k, 0)) for k, _, _ in STATES))
    return 0


if __name__ == "__main__":
    sys.exit(main())
