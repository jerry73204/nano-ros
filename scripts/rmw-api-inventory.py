#!/usr/bin/env python3
"""Enumerate the functions an rmw implementation is expected to provide.

The upstream `rmw` package declares its API across ~40 headers, and the
declarations are multi-line by convention:

    RMW_PUBLIC
    RMW_WARN_UNUSED
    rmw_ret_t
    rmw_create_node(rmw_context_t * context, const char * name, ...);

so a line-oriented grep undercounts badly — `^rmw_[a-z_]*\\(` found 68 of them in
`rmw.h` alone while the file has 126 declaration starts, and it silently drops
every function whose return type shares the name's line.

This walks each header, strips comments and preprocessor lines, then takes the
identifier immediately before the parameter list of every declaration that is
marked `RMW_PUBLIC`. That marker is what makes a declaration part of the ABI a
middleware must supply, which is exactly the set worth comparing against.

Usage:
    scripts/rmw-api-inventory.py [--include DIR] [--json]
    scripts/rmw-api-inventory.py --self-test

`--include` defaults to the rmw package found through AMENT_PREFIX_PATH, the
same resolution order RFC-0075 uses for the router: a ROS install is located by
the environment that a sourced setup.bash exports, never by a hardcoded path.
"""

import argparse
import json
import os
import re
import sys

# A declaration the ABI must supply. Everything else in these headers is a type,
# a macro, or a static inline convenience.
PUBLIC = "RMW_PUBLIC"

# `<name>(` where <name> is the last identifier before the parameter list.
DECL = re.compile(r"\b(rmw_[A-Za-z0-9_]+)\s*\(")

# Attributes that sit between RMW_PUBLIC and the return type.
ATTRS = ("RMW_WARN_UNUSED", "RMW_DEPRECATED", "RMW_PUBLIC_TYPE")


def strip_noise(text):
    """Comments and preprocessor lines. Both carry `rmw_*(` shapes that are not
    declarations — a doc block naming `rmw_create_node()` is the classic one,
    and counting it would be issue 0719's trap in a new file."""
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    text = re.sub(r"(?m)//.*$", " ", text)
    text = re.sub(r"(?m)^\s*#.*$", " ", text)
    return text


def normalise_params(raw):
    """`(rmw_node_t * node, const char * name)` -> `rmw_node_t * node, const char * name`.

    Whitespace collapsed; parameter NAMES KEPT.

    They used to be dropped here, and the reason given was sound but applied at
    the wrong layer: a renamed argument is not an ABI difference, so COMPARING
    names would report drift that is not drift and train people to skim. That is
    an argument about the comparison, not about the record — and dropping them
    at extraction time meant nothing downstream could show them either.

    The rendered comparison page put the cost on display: the nano-ros column
    showed `const rmw_publisher_t *publisher` while the ROS 2 column beside it
    showed a bare `const rmw_publisher_t *`, which reads as though upstream
    declared it nameless. It does not.

    So the record keeps them and the COMPARISON strips them:
    `rmw_abi_shape.strip_param_names` is the one place that happens, and
    `upstream_signatures()` applies it by default, so every existing caller
    compares exactly what it compared before.
    """
    raw = raw.strip()
    if not raw or raw == "void":
        return []
    out = []
    depth = 0
    cur = ""
    for ch in raw:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        if ch == "," and depth == 0:
            out.append(cur)
            cur = ""
        else:
            cur += ch
    out.append(cur)

    types = []
    for p in out:
        p = " ".join(p.split())
        if not p:
            continue
        # A function-pointer parameter keeps its shape AND its name, which C
        # writes inside the parentheses (`void (*cb)(void *)`). It used to be
        # rewritten to `(*)` by the same name-dropping pass as everything else;
        # now that names are kept, dropping this one would be the only
        # inconsistency on the page.
        if "(*" in p:
            types.append(p)
            continue
        p = re.sub(r"\[\s*\]$", " []", p)
        # Spacing is normalised around `*` for the TYPE half only, so the name
        # stays attached to the last `*` the way C is conventionally written
        # (`rmw_node_t * node`, not `rmw_node_t *node`) and a reader can still
        # tell type from name by the final token.
        m = re.match(r"^(.*?[\s*])([A-Za-z_][A-Za-z0-9_]*)((\s*\[\s*\])?)$", p)
        if m:
            ty = " ".join((m.group(1)).replace("*", " * ").split())
            types.append(f"{ty} {m.group(2)}{m.group(3).strip()}")
            continue
        types.append(" ".join(p.replace("*", " * ").split()))
    return types


def functions_in(text, with_signature=False):
    """Every RMW_PUBLIC-marked function, in declaration order.

    With `with_signature`, yields `(name, return_type, [param types])`.
    """
    body = strip_noise(text)
    out = []
    for chunk in body.split(PUBLIC)[1:]:
        # The declaration ends at the first `;` — a struct body or a later
        # function must not leak into this one's match.
        decl = chunk.split(";", 1)[0]
        for attr in ATTRS:
            decl = decl.replace(attr, " ")
        m = DECL.search(decl)
        if not m:
            continue
        if not with_signature:
            out.append(m.group(1))
            continue
        ret = " ".join(decl[: m.start(1)].replace("*", " * ").split())
        # Params run from the matched `(` to its matching `)`.
        rest = decl[m.end() :]
        depth = 1
        params = ""
        for ch in rest:
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    break
            params += ch
        out.append((m.group(1), ret, normalise_params(params)))
    return out


def resolve_include(explicit=None):
    """Where the rmw headers are. AMENT_PREFIX_PATH, never a hardcoded path."""
    if explicit:
        return explicit
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        for cand in (
            os.path.join(prefix, "include", "rmw", "rmw"),
            os.path.join(prefix, "include", "rmw"),
        ):
            if os.path.isdir(cand) and os.path.exists(os.path.join(cand, "rmw.h")):
                return cand
    return None


def inventory(include_dir, with_signature=False):
    """{name: header} for every RMW_PUBLIC function under `include_dir`.

    With `with_signature`, values are `(header, return_type, [param types])`."""
    found = {}
    # walk-ok: an installed ROS include tree, not a repo path
    for root, _dirs, files in os.walk(include_dir):
        for f in sorted(files):
            if not f.endswith(".h"):
                continue
            path = os.path.join(root, f)
            try:
                text = open(path, encoding="utf-8", errors="replace").read()
            except OSError:
                continue
            rel = os.path.relpath(path, include_dir)
            for item in functions_in(text, with_signature):
                if with_signature:
                    name, ret, params = item
                    found.setdefault(name, (rel, ret, params))
                else:
                    found.setdefault(item, rel)
    return found


def self_test():
    """The multi-line shape, and the two traps."""
    bad = []
    sample = """
/** Create a node. Calls rmw_fake_from_a_comment() internally. */
RMW_PUBLIC
RMW_WARN_UNUSED
rmw_node_t *
rmw_create_node(rmw_context_t * context, const char * name);

// not public — no marker
rmw_ret_t
rmw_internal_helper(void);

RMW_PUBLIC
rmw_ret_t
rmw_destroy_node(rmw_node_t * node);

typedef struct rmw_thing_s { int x; } rmw_thing_t;
"""
    got = functions_in(sample)
    if got != ["rmw_create_node", "rmw_destroy_node"]:
        bad.append(f"expected the two RMW_PUBLIC decls, got {got}")

    # A declaration whose return type shares the name's line — the shape the
    # line-oriented grep dropped.
    one_line = "RMW_PUBLIC\nrmw_ret_t rmw_init(const rmw_init_options_t * o, rmw_context_t * c);"
    if functions_in(one_line) != ["rmw_init"]:
        bad.append(f"one-line return type not matched: {functions_in(one_line)}")

    # Signature extraction: parameter names are KEPT (they are stripped by the
    # comparison, not by the record — see `normalise_params`), and a function
    # pointer parameter keeps its shape.
    sig = functions_in(
        "RMW_PUBLIC\nrmw_ret_t\nrmw_x(rmw_node_t * node, const char * n, void (*cb)(void *), int a[]);",
        with_signature=True,
    )
    # A function-pointer parameter keeps its own spelling — it returns before
    # the `*`-spacing pass, deliberately, since `void ( * )(void *)` would be a
    # worse thing to print at every call site than the shape as written. It also
    # keeps its name inside the `(*name)`, which is where C puts it.
    want = (
        "rmw_x",
        "rmw_ret_t",
        ["rmw_node_t * node", "const char * n", "void (*cb)(void *)", "int a[]"],
    )
    if sig != [want]:
        bad.append(f"signature extraction: got {sig}, want [{want}]")
    # An UNNAMED parameter is legal C and upstream does use it; it must survive
    # unchanged rather than losing its last type token to the name matcher.
    unnamed = functions_in(
        "RMW_PUBLIC\nrmw_ret_t\nrmw_y(const rmw_publisher_t *, void * *);",
        with_signature=True,
    )
    want_unnamed = ("rmw_y", "rmw_ret_t", ["const rmw_publisher_t *", "void * *"])
    if unnamed != [want_unnamed]:
        bad.append(f"unnamed parameters: got {unnamed}, want [{want_unnamed}]")
    if normalise_params("void") != []:
        bad.append("`void` must normalise to no parameters")

    if bad:
        for b in bad:
            sys.stderr.write("rmw-api-inventory --self-test: " + b + "\n")
        return 2
    print("rmw-api-inventory --self-test: OK (6 case(s))")
    return 0


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--include", help="rmw include dir (default: via AMENT_PREFIX_PATH)")
    ap.add_argument("--json", action="store_true")
    ap.add_argument(
        "--signatures", action="store_true",
        help="emit `name<TAB>return<TAB>param, param<TAB>header` — the form the "
             "shape comparison consumes offline",
    )
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test()

    inc = resolve_include(args.include)
    if not inc:
        sys.stderr.write(
            "rmw-api-inventory: no rmw headers found.\n"
            "  Source a ROS install first (`source /opt/ros/<distro>/setup.bash`),\n"
            "  which exports AMENT_PREFIX_PATH, or pass --include.\n"
        )
        return 2

    found = inventory(inc, with_signature=args.signatures)
    if args.signatures:
        # A FORMAT MARKER, so a reader can tell a names-kept extract from the
        # names-dropped one that preceded it. Without it the two are
        # indistinguishable and stripping the older file a second time eats the
        # last token of every by-value parameter: `const rmw_qos_profile_t`
        # became `const`. Measured, on exactly one symbol, which is how close
        # that came to shipping unnoticed.
        print("# Signatures of the rmw implementation contract — ROS 2 Humble.")
        print("# name<TAB>return<TAB>params<TAB>header. Parameter NAMES ARE KEPT;")
        print("# the COMPARISON drops them (`rmw_abi_shape.strip_param_names`),")
        print("# because a renamed argument is not an ABI difference and reporting")
        print("# one trains people to skim — while a page that shows our named")
        print("# slots beside upstream's bare types reads as though upstream")
        print("# declared them nameless. Regenerate in the distrobox:")
        print("#   scripts/rmw-api-inventory.py --signatures \\")
        print("#     > docs/reference/rmw-implementation-signatures.txt")
        print("# format: 2 (parameter names kept; strip them to compare)")
        for name in sorted(found):
            header, ret, params = found[name]
            print(f"{name}\t{ret}\t{', '.join(params)}\t{header}")
        return 0
    if args.json:
        print(json.dumps({"include": inc, "functions": found}, indent=2, sort_keys=True))
        return 0
    print(f"# rmw API inventory — {inc}")
    print(f"# {len(found)} RMW_PUBLIC function(s)")
    for name in sorted(found):
        print(f"{name}\t{found[name]}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
