#!/usr/bin/env python3
"""Extract a Rust public API surface from rustdoc JSON.

Phase 379. Sibling of `extract_cxx.py`; same record shape out, so the
correlator does not care which language a surface came from.

rustdoc JSON rather than a source sweep for the same reason clang is used on the
C++ side: the campaign's question is about ARGUMENTS, and `pub use` re-exports,
`impl` blocks, generic bounds and `#[cfg]` gating all defeat a text scan. Our
`nros` crate is a FACADE -- almost everything a user touches is `pub use`d from
`nros-core` / `nros-node` -- so a sweep of `packages/api/nros/src` would report a
handful of items and miss the entire surface.

`--document-private-items` is deliberately NOT passed: the surface under
comparison is the one a user can reach.

rustdoc writes ONE JSON PER CRATE, and a re-exported item's id belongs to the
crate that defined it, not to the facade. Following `pub use nros_node::Executor`
therefore means opening `nros_node.json` -- resolving the id through the facade's
`paths` table to a module path, then finding that path in the defining crate.
Without that step the facade extracts as a dozen items and the entire executor,
node-context and publisher surface reads as absent.
"""

import json
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# `nros`'s `default = []` (issue 0591), and 18 `#[cfg(feature = "rmw-cffi")]`
# blocks in its lib.rs gate the entire runtime surface -- `Executor`,
# `NodeHandle`, `EmbeddedPublisher`, `Promise`, the concrete `Subscription`.
# Documenting with default features therefore extracts a facade with the runtime
# removed, which inflates `theirs-only` (things we HAVE read as gaps) and
# deflates `ours-only`. The features below are what a real Rust image enables;
# `--all-features` is not used because several are mutually exclusive backends.
NROS_FEATURES = [
    "rmw-cffi",
    "macros",
    "alloc",
    "std",
    "param-services",
    "lifecycle-services",
    "stream",
    "safety-e2e",
]


def _pinned_nightly():
    """The nightly the repo PINS, not a bare `+nightly`.

    `cargo +nightly` asks for a toolchain named exactly `nightly`. A developer
    box usually has one, and rustup auto-installs it otherwise, so the bare
    spelling worked everywhere it was tried. The CI container installs
    `nightly-2026-04-11` and NO `nightly` alias, so there the bare form either
    fails or silently fetches a different, unpinned nightly -- and rustdoc's
    JSON is an UNSTABLE format, so "some other nightly" is not a harmless
    substitution: its schema is what this extractor parses.

    Read from `tools/rust-toolchain.toml`, where the pin already lives, so a
    bump still moves exactly one file.
    """
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    manifest = os.path.join(root, "tools", "rust-toolchain.toml")
    with open(manifest) as fh:
        for line in fh:
            m = re.match(r'\s*channel\s*=\s*"([^"]+)"', line)
            if m:
                return m.group(1)
    raise RuntimeError("no `channel = \"...\"` line in " + manifest)


def rustdoc_json(manifest_dir, with_deps, target_dir=None, extra_env=None, features=None):
    """Build rustdoc JSON for the lib target of the crate at `manifest_dir`.

    Nightly only -- `--output-format json` is unstable. CLAUDE.md already
    requires nightly for rustfmt, so this adds no new toolchain requirement.

    Doc lints are silenced. A broken intra-doc link in a crate under comparison
    is a real defect but not THIS tool's defect, and letting one abort the
    extraction makes the parity report hostage to unrelated doc rot.
    """
    env = dict(os.environ)
    env["RUSTDOCFLAGS"] = "-Z unstable-options --output-format json -A warnings"
    if target_dir:
        env["CARGO_TARGET_DIR"] = target_dir
    if extra_env:
        env.update(extra_env)

    cmd = ["cargo", "+" + _pinned_nightly(), "doc", "--lib"]
    if not with_deps:
        cmd.append("--no-deps")
    if features:
        cmd += ["--features", ",".join(features)]
    proc = subprocess.run(cmd, cwd=manifest_dir, env=env, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            "cargo doc failed in %s:\n%s" % (manifest_dir, proc.stderr[-3000:])
        )

    out = target_dir or os.path.join(manifest_dir, "target")
    docdir = os.path.join(out, "doc")
    # cargo writes <crate_name>.json; the crate name is not always the dir name.
    candidates = [f for f in os.listdir(docdir) if f.endswith(".json")]
    if not candidates:
        raise RuntimeError("no rustdoc JSON produced under " + docdir)
    return docdir, candidates


def load(docdir, crate_json):
    with open(os.path.join(docdir, crate_json)) as fh:
        return json.load(fh)


class Docs:
    """Every crate JSON from one `cargo doc` run, joined across crate boundaries.

    `resolve(doc, id)` returns `(defining_doc, item)` for an id that may belong
    to a crate other than `doc` -- which is the normal case for a facade.
    """

    def __init__(self, docdir):
        self.docdir = docdir
        self.by_crate = {}
        for fn in os.listdir(docdir):
            if not fn.endswith(".json"):
                continue
            try:
                doc = load(docdir, fn)
            except (ValueError, OSError):
                continue
            name = doc.get("index", {}).get(str(doc.get("root")), {}).get("name")
            if name:
                self.by_crate[name] = doc
        self._path_index = {}

    def crate(self, name):
        return self.by_crate.get(name)

    def _paths_of(self, doc):
        key = id(doc)
        if key not in self._path_index:
            table = {}
            for item_id, meta in doc.get("paths", {}).items():
                if meta.get("crate_id") == 0:
                    table[tuple(meta.get("path", []))] = item_id
            self._path_index[key] = table
        return self._path_index[key]

    def resolve(self, doc, item_id):
        """(doc, item) for `item_id`, following it into its defining crate."""
        local = doc["index"].get(str(item_id)) or doc["index"].get(item_id)
        if local is not None:
            return doc, local

        meta = doc.get("paths", {}).get(str(item_id)) or doc.get("paths", {}).get(item_id)
        if not meta:
            return None, None
        crate_name = None
        cid = meta.get("crate_id")
        if cid == 0:
            crate_name = doc["index"].get(str(doc["root"]), {}).get("name")
        else:
            crate_name = (doc.get("external_crates", {}).get(str(cid)) or {}).get("name")
        target = self.by_crate.get(crate_name)
        if target is None:
            return None, None
        tid = self._paths_of(target).get(tuple(meta.get("path", [])))
        if tid is None:
            return None, None
        return target, (target["index"].get(str(tid)) or target["index"].get(tid))


def _fmt_type(t):
    """Render a rustdoc JSON type node as source-like text.

    Only the shapes that appear in a client-library signature are handled.
    Anything else falls back to a tagged placeholder rather than a wrong string:
    a plausible-but-wrong rendering would show up in the report as a bogus
    argument difference, which is the one failure mode this campaign cannot
    afford.
    """
    if t is None:
        return ""
    if isinstance(t, str):
        return t
    if not isinstance(t, dict):
        return str(t)

    if "primitive" in t:
        return t["primitive"]
    if "generic" in t:
        return t["generic"]
    if "resolved_path" in t:
        rp = t["resolved_path"]
        # rustdoc renamed this field across format versions ("name" -> "path").
        # Falling back to "?" made every `Arc<NodeState>` render as `?<?>`.
        name = rp.get("name") or rp.get("path") or "?"
        args = rp.get("args") or {}
        angle = args.get("angle_bracketed") if isinstance(args, dict) else None
        if angle and angle.get("args"):
            inner = []
            for a in angle["args"]:
                if isinstance(a, dict) and "type" in a:
                    inner.append(_fmt_type(a["type"]))
                elif isinstance(a, dict) and "lifetime" in a:
                    continue
                else:
                    inner.append("_")
            if inner:
                return "%s<%s>" % (name, ", ".join(inner))
        return name
    if "borrowed_ref" in t:
        br = t["borrowed_ref"]
        mut = "mut " if br.get("is_mutable") else ""
        return "&" + mut + _fmt_type(br.get("type"))
    if "raw_pointer" in t:
        rp = t["raw_pointer"]
        mut = "mut" if rp.get("is_mutable") else "const"
        return "*%s %s" % (mut, _fmt_type(rp.get("type")))
    if "slice" in t:
        return "[%s]" % _fmt_type(t["slice"])
    if "array" in t:
        a = t["array"]
        return "[%s; %s]" % (_fmt_type(a.get("type")), a.get("len", "_"))
    if "tuple" in t:
        return "(%s)" % ", ".join(_fmt_type(x) for x in t["tuple"])
    if "impl_trait" in t:
        return "impl " + " + ".join(_bound(b) for b in t["impl_trait"])
    if "dyn_trait" in t:
        dt = t["dyn_trait"]
        return "dyn " + " + ".join(
            p.get("trait", {}).get("name", "?") for p in dt.get("traits", [])
        )
    if "qualified_path" in t:
        qp = t["qualified_path"]
        return "%s::%s" % (_fmt_type(qp.get("self_type")), qp.get("name", "?"))
    return "<unrendered:%s>" % ",".join(sorted(t.keys()))


def _bound(b):
    if isinstance(b, dict) and "trait_bound" in b:
        return b["trait_bound"].get("trait", {}).get("name", "?")
    return "?"


def _span(item):
    """The source file an item was declared in, for the public-surface filter."""
    return ((item or {}).get("span") or {}).get("filename", "")


def _fn_record(name, inner, path):
    sig = inner.get("sig") or inner.get("decl") or {}
    inputs = sig.get("inputs") or []
    params = [{"name": n, "type": _fmt_type(t), "default": False} for n, t in inputs]
    ret = _fmt_type(sig.get("output"))
    generics = inner.get("generics") or {}
    tparams = [
        p.get("name", "_")
        for p in (generics.get("params") or [])
        if isinstance(p.get("kind"), dict) and "lifetime" not in p.get("kind", {})
    ]
    return {
        "kind": "function",
        "qual": path,
        "name": name,
        "template": tparams,
        "params": params,
        "ret": ret,
    }


def _with_span(record, item):
    record["header"] = _span(item)
    return record


def _item_name(idx, item_id):
    it = idx.get(str(item_id)) or idx.get(item_id)
    return it


# A client library that vendors its generated message bindings -- rclrs puts
# `test_msgs`, `std_msgs` and friends inside its own crate -- would otherwise
# contribute hundreds of MESSAGE types to an API comparison. Messages are
# codegen output on both sides and are compared by RFC-0023/0033, not here.
MESSAGE_MODULES = ("_msgs", "_srvs", "_interfaces", "_actions")


def _is_message_module(name):
    return any(name.endswith(sfx) for sfx in MESSAGE_MODULES)


def surface(docs, doc, crate_prefix):
    """Flatten a rustdoc JSON crate into records rooted at its top-level module.

    Walks `pub use` re-exports ACROSS CRATES, which is not optional: `nros` is a
    facade whose root module holds almost nothing of its own.
    """
    root = doc["root"]
    out = []
    seen = set()
    unresolved = []

    def walk_module(d, item_id, path):
        d2, it = docs.resolve(d, item_id)
        if it is None:
            return
        mod = it.get("inner", {}).get("module")
        if mod is None:
            return
        for child in mod.get("items", []):
            emit(d2, child, path)

    def emit(d, item_id, path):
        key = (str(item_id), path)
        if key in seen:
            return
        seen.add(key)
        d, it = docs.resolve(d, item_id)
        if it is None:
            # An id that resolves to no item in any documented crate. Recording
            # the name alone would claim a surface whose signature is unknown,
            # so it is dropped -- and counted, so the loss is never silent.
            unresolved.append(str(item_id))
            return
        if it.get("visibility") not in ("public", "default", None):
            return
        name = it.get("name") or ""
        inner = it.get("inner", {})
        kind = next(iter(inner), None)
        qual = path + "::" + name if name else path

        if kind == "use":
            u = inner["use"]
            target = u.get("id")
            if _is_message_module(u.get("name") or ""):
                return
            if u.get("is_glob"):
                if target is not None:
                    walk_module(d, target, path)
                return
            if target is not None:
                emit(d, target, path)
            return

        if kind == "module":
            if name and not _is_message_module(name):
                walk_module(d, item_id, qual)
            return

        if kind == "function":
            out.append(_with_span(_fn_record(name, inner["function"], qual), it))
            return

        if kind in ("struct", "enum", "trait", "union"):
            members = []
            body = inner[kind]
            impls = body.get("impls") or []
            for impl_id in impls:
                _, im = docs.resolve(d, impl_id)
                if im is None:
                    continue
                ii = im.get("inner", {}).get("impl", {})
                # A trait impl is the trait's surface, not this type's.
                if ii.get("trait"):
                    continue
                for m_id in ii.get("items", []):
                    _, m = docs.resolve(d, m_id)
                    if m is None or m.get("visibility") not in ("public", "default", None):
                        continue
                    minner = m.get("inner", {})
                    if "function" not in minner:
                        continue
                    rec = _fn_record(m.get("name", ""), minner["function"], "")
                    members.append(
                        {
                            "name": rec["name"],
                            "template": rec["template"],
                            "params": rec["params"],
                            "ret": rec["ret"],
                        }
                    )
            if kind == "trait":
                for m_id in body.get("items", []):
                    _, m = docs.resolve(d, m_id)
                    if m is None:
                        continue
                    minner = m.get("inner", {})
                    if "function" not in minner:
                        continue
                    rec = _fn_record(m.get("name", ""), minner["function"], "")
                    members.append(
                        {
                            "name": rec["name"],
                            "template": rec["template"],
                            "params": rec["params"],
                            "ret": rec["ret"],
                        }
                    )
            record = {
                "kind": "enum" if kind == "enum" else "type",
                "qual": qual,
                "name": name,
                "template": [],
                "header": _span(it),
                "members": members,
            }
            if kind == "enum":
                record["values"] = [
                    (docs.resolve(d, v)[1] or {}).get("name", "")
                    for v in body.get("variants", [])
                ]
            out.append(record)
            return

        if kind in ("type_alias", "typedef"):
            out.append(
                {
                    "kind": "alias",
                    "qual": qual,
                    "name": name,
                    "header": _span(it),
                    "type": _fmt_type(inner[kind].get("type")),
                }
            )
            return

        if kind in ("constant", "static"):
            out.append({"kind": "const", "qual": qual, "name": name, "header": _span(it)})
            return

        if kind == "macro":
            out.append({"kind": "macro", "qual": qual, "name": name, "header": _span(it)})
            return

    walk_module(doc, root, crate_prefix)
    if unresolved:
        print(
            "note: %d re-exported id(s) resolved to no documented item; "
            "their signatures are absent from this surface" % len(unresolved),
            file=sys.stderr,
        )
    return out
