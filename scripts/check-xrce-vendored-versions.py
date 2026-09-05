#!/usr/bin/env python3
"""One vendored tree, one version, WIRED TO ITSELF — issue 1069.

`MICROCDR_VERSION_STR` compiled as `"2.4.1"` under the CMake lane and `"2.0.2"`
under the cargo lane, against a micro-CDR checkout that is 2.0.2. Three numbers,
none of them authoritative. The client half was worse: both lanes said `2.4.1`
for a tree pinned at `bdfa2809` = "Release v3.0.1", which also disarmed
upstream's own `#if UXR_CLIENT_VERSION_MAJOR >= 4` tripwire in
`uxr/client/config.h.in` by a whole major version — so the issue's "zero
readers" claim was wrong; there is one, and it is upstream's.

The mechanism was a shared variable name. Both templates take `@PROJECT_VERSION*@`,
`nros-rmw-xrce/CMakeLists.txt` set them once for the client, and the micro-CDR
`configure_file` further down inherited them.

THE FIX WAS TO DERIVE IT, and this gate is what makes that stick. Each vendored
tree states its version in its own `CMakeLists.txt` — `project(<name> VERSION
"X.Y.Z")`, upstream telling us what the tree is — and both lanes read that line:
`_nros_xrce_project_version()` in the CMakeLists, `vendored_project_version()`
in build.rs. Correcting four literals would have left four literals.

Not `git describe --tags`, which issue 1069 proposed: a submodule is fetched by
SHA with no tags, so `git -C …/micro-xrce-dds-client describe --tags` answers
"No tags can describe" on an ordinary checkout. Measured, phase-420 W9.

WHY THIS GATE CHECKS WIRING AND NOT MERELY SHAPE
------------------------------------------------

Its first version checked shape — no version literals, a derivation block
exists, the sdk-index rows match the trees — and a review mutation walked
straight through it::

    -_nros_xrce_project_version("${MICROCDR_DIR}/CMakeLists.txt" microcdr _ucdr)
    +_nros_xrce_project_version("${MICROXRCEDDS_CLIENT_DIR}/CMakeLists.txt"
    +    microxrcedds_client _ucdr)

Shape perfect, wires crossed, gate green — and a real configure then emitted
`MICROCDR_VERSION_STR "3.0.1"`, which is issue 1069 back verbatim. A gate whose
coverage is narrower than the rule it enforces is the issue-0196 class.

So the invariant is stated as WIRING, declared once in `WIRING` below and
checked end to end in both lanes: *the tokens that reach micro-CDR's template
come from micro-CDR's tree*, not merely *no literals appear anywhere*. Every
hop is checked, because a crosswire at any one of them is silent:

    cmake    dir var  →  derivation site  →  prefix  →  configure_file template
    cargo    let root →  generator arg    →  in-body derivation + template read

What it checks
--------------

1. Each vendored tree states exactly one `project(<name> VERSION "X.Y.Z")`, and
   it parses. Vendored trees are skipped LOUDLY when the submodule is absent.
2. The client's own `set(_microcdr_version X.Y.Z)` — upstream saying which
   micro-CDR this client expects — matches the micro-CDR we vendor. Catches
   bumping one submodule and not the other. It lives here rather than in either
   lane so there is one implementation of it, not one per lane.
3. NEITHER LANE HOLDS A VERSION LITERAL. An `X.Y.Z` in either file that is not
   on the allowlist is a restatement, and a restatement is issue 1069.
4. Both lanes actually carry the derivation: a delimited
   `NROS-XRCE-VERSIONS-BEGIN/END` block reading `CMakeLists.txt`. Without this,
   check (3) passes vacuously on a lane that stopped setting the version at all.
5. CMAKE WIRING. `MICROCDR_DIR` points at the `micro-cdr` directory; the one
   `_nros_xrce_project_version()` site for that tree names that dir var, that
   project id and that prefix together; and the four `PROJECT_VERSION*`
   re-points immediately before micro-CDR's `configure_file` all come from that
   prefix. (5c) is the 1069 defect itself: both templates take the same
   `@PROJECT_VERSION*@` names, so a lane that sets them once and leaves them
   standing configures the second header from the first tree's numbers.
6. CARGO WIRING. `let microcdr = xrce_sys.join("micro-cdr")`; `main`'s
   `generate_config(...)` call for micro-CDR reads its template from that
   binding AND derives its version from that same binding, naming that tree's
   project id. Three separate crosswire vectors, each silent on its own.
   phase-420 W9 collapsed the two per-tree generators into ONE
   `generate_config`, which moved all three hops into a single call expression
   — the pairing got easier to check, not looser: the template path and the
   `vendored_project_version()` argument now sit three lines apart in the same
   call, and this gate reads them as a pair.
7. `nros-sdk-index.toml`'s `[source.micro-*] version` equals the vendored
   version. That row is AUTHORED on purpose — `nros setup --source micro-cdr`
   runs when the checkout is absent, so it cannot derive — but it is a factual
   claim about the tree (it is what `gen-support-status.py` renders into the
   book), so it is checked rather than trusted.

Buildless by construction: everything above is read out of the sources. A gate
on the fast line that shells out to cmake or cargo is a gate nobody runs.

Run: python3 scripts/check-xrce-vendored-versions.py
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
XRCE = REPO / "packages/rmw/xrce"
BUILD_RS = XRCE / "nros-rmw-xrce-cffi/build.rs"
CMAKE = XRCE / "nros-rmw-xrce/CMakeLists.txt"
SDK_INDEX = REPO / "nros-sdk-index.toml"


@dataclass(frozen=True)
class Wiring:
    """Every name one vendored tree is known by, in one place.

    THE POINT OF THE DATACLASS is that the pairing is declared once and checked
    everywhere, rather than each hop being independently plausible. A crosswire
    is a hop that names a field from the OTHER row.
    """

    dirname: str  # checkout dir under packages/rmw/xrce/xrce-sys/
    project: str  # upstream `project(<id> VERSION …)` id
    template: str  # its `config.h.in`, relative to the checkout
    cmake_dir_var: str  # the CMake cache PATH var
    cmake_prefix: str  # `_uxr` / `_ucdr` — the derived-version var prefix
    rs_root: str  # the `let` binding in build.rs `main`
    sdk_row: str  # `[source.<name>]` in nros-sdk-index.toml


WIRING = {
    "micro-xrce-dds-client": Wiring(
        dirname="micro-xrce-dds-client",
        project="microxrcedds_client",
        template="include/uxr/client/config.h.in",
        cmake_dir_var="MICROXRCEDDS_CLIENT_DIR",
        cmake_prefix="_uxr",
        rs_root="microxrce",
        sdk_row="micro-xrce-dds-client",
    ),
    "micro-cdr": Wiring(
        dirname="micro-cdr",
        project="microcdr",
        template="include/ucdr/config.h.in",
        cmake_dir_var="MICROCDR_DIR",
        cmake_prefix="_ucdr",
        rs_root="microcdr",
        sdk_row="micro-cdr",
    ),
}
VENDOR_ROOT = "packages/rmw/xrce/xrce-sys"

# The delimited block in each lane that holds the derivation.
_BEGIN = "NROS-XRCE-VERSIONS-BEGIN"
_END = "NROS-XRCE-VERSIONS-END"

# `X.Y.Z` anywhere in a lane file, comments stripped.
_TRIPLE = re.compile(r"\b\d+\.\d+\.\d+\b")

# Version triples a lane may legitimately spell, with the reason. Anything else
# is a restatement of a vendored version — issue 1069.
LITERAL_ALLOWLIST = {
    CMAKE: {
        # This project's OWN version, not a vendored tree's.
        "0.1.0": "project(nros_rmw_xrce VERSION 0.1.0)",
    },
    BUILD_RS: {},
}


# `project(microcdr VERSION "2.0.2" LANGUAGES C)` — the fact both lanes read.
# Same shape both lanes parse: optional quotes, X.Y.Z.
def _project_version_re(project: str) -> re.Pattern[str]:
    return re.compile(
        r"^\s*project\(\s*" + re.escape(project) + r"\s+VERSION\s+\"?(\d+\.\d+\.\d+)\"?"
    )


# `set(_microcdr_version 2.0.2)` in the CLIENT's CMakeLists — upstream's own
# statement of the micro-CDR it expects.
_EXPECTED_UCDR = re.compile(r"^\s*set\(\s*_microcdr_version\s+\"?(\d+\.\d+\.\d+)\"?\s*\)")

# --- CMake lane wiring ------------------------------------------------------

# `set(MICROCDR_DIR "…/micro-cdr" CACHE PATH …)` — spans lines, so the value is
# taken as the first quoted string after the name.
_CM_SET_STR = re.compile(r"set\(\s*([A-Za-z0-9_]+)\s+\"([^\"]*)\"")

# `_nros_xrce_project_version("${MICROCDR_DIR}/CMakeLists.txt" microcdr _ucdr)`
_CM_DERIVE = re.compile(
    r"_nros_xrce_project_version\(\s*\"\$\{([A-Za-z0-9_]+)\}/CMakeLists\.txt\"\s+"
    r"([A-Za-z0-9_]+)\s+(_[A-Za-z0-9_]+)\s*\)"
)

# `set(PROJECT_VERSION_MAJOR ${_uxr_VERSION_MAJOR})` — the CMake lane re-pointing
# the SHARED `@PROJECT_VERSION*@` token set at one tree before its
# `configure_file`. Captures the tree prefix (`_uxr` / `_ucdr`).
_REPOINT = re.compile(
    r"set\(\s*PROJECT_VERSION(?:_MAJOR|_MINOR|_PATCH)?\s+\$\{(_[A-Za-z0-9]+?)_VERSION"
)

# `configure_file(` and the first quoted argument that follows it.
_CM_CONFIGURE = re.compile(r"configure_file\(\s*\"([^\"]*)\"", re.S)

# --- cargo lane wiring ------------------------------------------------------

# `let microcdr = xrce_sys.join("micro-cdr");`
_RS_ROOT = re.compile(r"let\s+([A-Za-z0-9_]+)\s*=\s*xrce_sys\.join\(\"([^\"]+)\"\)")
# `generate_config(` — the one generator, phase-420 W9. Its arguments are
# extracted by balancing parens rather than by regex: the call spans lines and
# nests `vendored_project_version(...)`, which no flat pattern can bound.
_RS_CALL_HEAD = "generate_config("
# `vendored_project_version(&microcdr.join("CMakeLists.txt"), "microcdr")`
_RS_DERIVE = re.compile(
    r"vendored_project_version\(\s*&([A-Za-z0-9_]+)\.join\(\"CMakeLists\.txt\"\)\s*,\s*"
    r"\"([A-Za-z0-9_]+)\"\s*\)",
    re.S,
)
# `read_to_string(microcdr.join("include/ucdr/config.h.in"))`
_RS_TEMPLATE = re.compile(r"([A-Za-z0-9_]+)\.join\(\"(include/[^\"]*config\.h\.in)\"\)")
# A top-level `fn` — used to slice build.rs into bodies.
_RS_FN = re.compile(r"^fn\s+([A-Za-z0-9_]+)", re.M)

# `version = "3.0.1"` under a `[source.<name>]` header in nros-sdk-index.toml.
_TOML_SECTION = re.compile(r"^\s*\[([^\]]+)\]\s*$")
_TOML_VERSION = re.compile(r'^\s*version\s*=\s*"([^"]*)"')


def project_versions(text: str, project: str) -> list[str]:
    """Every `project(<project> VERSION X.Y.Z)` a CMakeLists states."""
    pat = _project_version_re(project)
    out = []
    for raw in text.splitlines():
        m = pat.match(raw.split("#", 1)[0])
        if m:
            out.append(m.group(1))
    return out


def strip_comments(text: str, marker: str) -> str:
    """Drop `marker`-to-end-of-line. Crude on purpose: over-stripping can only
    hide a literal inside a string, and a version literal in a string is exactly
    what a lane must not have anyway."""
    return "\n".join(line.split(marker, 1)[0] for line in text.splitlines())


def lane_literals(text: str, marker: str, allow: dict[str, str]) -> list[str]:
    """Version triples a lane spells that are not allowlisted."""
    return sorted({t for t in _TRIPLE.findall(strip_comments(text, marker)) if t not in allow})


def lane_block(text: str, label: str) -> str:
    """The derivation block, or raise."""
    if _BEGIN not in text or _END not in text:
        raise LookupError(
            f"{label} has no {_BEGIN}/{_END} block — the derivation is gone, so "
            "nothing states where the version comes from."
        )
    return text.split(_BEGIN, 1)[1].split(_END, 1)[0]


def cmake_vars(text: str) -> dict[str, str]:
    """`set(<name> "<value>")` pairs. First assignment wins, matching the file's
    own single-assignment style."""
    out: dict[str, str] = {}
    for m in _CM_SET_STR.finditer(strip_comments(text, "#")):
        out.setdefault(m.group(1), m.group(2))
    return out


def expand(value: str, vars_: dict[str, str], rounds: int = 4) -> str:
    """Resolve `${var}` against `vars_`, leaving unknown names alone."""
    for _ in range(rounds):
        new = re.sub(r"\$\{([A-Za-z0-9_]+)\}", lambda m: vars_.get(m.group(1), m.group(0)), value)
        if new == value:
            break
        value = new
    return value


def path_tree(path: str) -> str | None:
    """Which vendored tree a resolved path names, by directory SEGMENT.

    Segment-exact so no dirname can be a substring of another's path.
    """
    segs = set(path.split("/"))
    hits = [name for name, w in WIRING.items() if w.dirname in segs]
    return hits[0] if len(hits) == 1 else None


def template_tree(path: str) -> str | None:
    """Which vendored tree a resolved `config.h.in` path belongs to, by the
    template suffix. Returns None for a path that is not a vendored template."""
    hits = [name for name, w in WIRING.items() if path.endswith(w.template)]
    return hits[0] if len(hits) == 1 else None


def rust_fn_bodies(text: str) -> dict[str, str]:
    """→ {fn name: source from its `fn` line to the next top-level `fn`}."""
    marks = [(m.start(), m.group(1)) for m in _RS_FN.finditer(text)]
    out: dict[str, str] = {}
    for i, (start, name) in enumerate(marks):
        end = marks[i + 1][0] if i + 1 < len(marks) else len(text)
        out[name] = text[start:end]
    return out


def sdk_index_versions(text: str) -> dict[str, str]:
    """`[source.<name>] version` rows."""
    out: dict[str, str] = {}
    section = ""
    for raw in text.splitlines():
        m = _TOML_SECTION.match(raw)
        if m:
            section = m.group(1)
            continue
        m = _TOML_VERSION.match(raw)
        if m and section.startswith("source."):
            out.setdefault(section[len("source.") :], m.group(1))
    return out


def cmake_wiring_problems(text: str) -> list[str]:
    """(5) — the CMake lane's dir var → derivation → prefix → template chain."""
    bad: list[str] = []
    vars_ = cmake_vars(text)

    # (5a) each dir var points at its own checkout.
    for name, w in WIRING.items():
        val = vars_.get(w.cmake_dir_var)
        if val is None:
            bad.append(
                f"CMakeLists.txt never sets `{w.cmake_dir_var}` — the gate cannot tell which "
                f"checkout {name}'s version is read from."
            )
        elif path_tree(val) != name:
            bad.append(
                f"CMakeLists.txt points `{w.cmake_dir_var}` at `{val}`, which is not the "
                f"`{w.dirname}` checkout. Every hop for {name} must name {name}."
            )

    # (5b) exactly one derivation site per tree, and its three names agree.
    sites = _CM_DERIVE.findall(strip_comments(text, "#"))
    expected = {(w.cmake_dir_var, w.project, w.cmake_prefix): n for n, w in WIRING.items()}
    if len(sites) != len(WIRING):
        bad.append(
            f"CMakeLists.txt has {len(sites)} `_nros_xrce_project_version(…)` site(s), expected "
            f"{len(WIRING)} — one per vendored tree."
        )
    seen: set[str] = set()
    for dir_var, project, prefix in sites:
        tree = expected.get((dir_var, project, prefix))
        if tree is None:
            bad.append(
                f"CMakeLists.txt derives `{prefix}_VERSION*` from "
                f"`${{{dir_var}}}/CMakeLists.txt` as project `{project}` — those three names do "
                "not describe one tree, so the derivation is CROSSWIRED. Expected pairings: "
                + "; ".join(
                    f"{n}: ({w.cmake_dir_var}, {w.project}, {w.cmake_prefix})"
                    for n, w in sorted(WIRING.items())
                )
                + ". Shape alone is not the invariant — micro-CDR's tokens must come from "
                "micro-CDR's tree (issue 1069)."
            )
        else:
            seen.add(tree)
    for name in sorted(set(WIRING) - seen):
        bad.append(f"CMakeLists.txt has no correctly-wired derivation site for {name}.")

    # (5c) the four re-points immediately before each vendored `configure_file`
    #      come from that template's own tree. THE 1069 DEFECT ITSELF.
    repoints = [(m.start(), m.group(1)) for m in _REPOINT.finditer(text)]
    for name, w in WIRING.items():
        n = [p for _, p in repoints].count(w.cmake_prefix)
        if n != 4:
            bad.append(
                f"CMakeLists.txt assigns PROJECT_VERSION* from `${{{w.cmake_prefix}_VERSION*}}` "
                f"{n} time(s), expected 4 (MAJOR, MINOR, PATCH and the bare name). Both "
                "upstream templates take the SAME `@PROJECT_VERSION*@` tokens, so each "
                f"`configure_file` must re-point all four at its own tree — {name} here. "
                "Leaving the previous tree's values standing IS issue 1069; ordering the "
                "calls differently only moves which header is wrong."
            )
    configured: set[str] = set()
    for m in _CM_CONFIGURE.finditer(text):
        resolved = expand(m.group(1), vars_)
        tree = template_tree(resolved)
        if tree is None:
            continue  # not a vendored template
        if path_tree(resolved) not in (tree, None):
            bad.append(
                f"CMakeLists.txt configures `{resolved}` — the template belongs to {tree} but "
                f"the path is under {path_tree(resolved)}'s checkout. Crosswired."
            )
        configured.add(tree)
        prior = [p for off, p in repoints if off < m.start()][-4:]
        want = WIRING[tree].cmake_prefix
        if prior != [want] * 4:
            got = ", ".join(prior) if prior else "nothing"
            bad.append(
                f"CMakeLists.txt configures {tree}'s `{WIRING[tree].template}` with "
                f"PROJECT_VERSION* last set from [{got}], not four times from "
                f"`${{{want}_VERSION*}}`. That compiles one tree's version string into the "
                "other tree's header — issue 1069 verbatim."
            )
    for name in sorted(set(WIRING) - configured):
        bad.append(
            f"CMakeLists.txt configures no template for {name} — expected a "
            f"`configure_file` of `{WIRING[name].template}`."
        )
    return bad


def rust_call_args(text: str, head: str) -> list[str]:
    """Every `head…)` call in `text`, as the source between its parens.

    Paren-balanced, not regex: the call spans lines and nests another call, so
    a flat pattern would stop at the first `)` and pair the wrong arguments —
    which is the failure this gate exists to catch, one level up.
    """
    out: list[str] = []
    i = text.find(head)
    while i >= 0:
        depth = 0
        j = i + len(head) - 1
        while j < len(text):
            if text[j] == "(":
                depth += 1
            elif text[j] == ")":
                depth -= 1
                if depth == 0:
                    break
            j += 1
        out.append(text[i + len(head) : j])
        i = text.find(head, j)
    return out


def rust_wiring_problems(text: str) -> list[str]:
    """(6) — the cargo lane's let-binding → template arg → version arg chain."""
    bad: list[str] = []
    bodies = rust_fn_bodies(text)
    main = bodies.get("main", "")

    # (6a) each `let` root binds its own checkout directory.
    roots = dict(_RS_ROOT.findall(main))
    for name, w in WIRING.items():
        got = roots.get(w.rs_root)
        if got is None:
            bad.append(
                f"build.rs `main` never binds `let {w.rs_root} = xrce_sys.join(…)` — the gate "
                f"cannot tell which checkout {name}'s version is read from."
            )
        elif got != w.dirname:
            bad.append(
                f"build.rs binds `let {w.rs_root} = xrce_sys.join(\"{got}\")`, not "
                f"`\"{w.dirname}\"`. Every hop for {name} must name {name}."
            )

    # (6b/6c) one `generate_config(...)` call per tree, and INSIDE that call the
    # template read and the version derivation name the same binding and that
    # tree's project id.
    seen: dict[str, int] = {name: 0 for name in WIRING}
    for call in rust_call_args(main, _RS_CALL_HEAD):
        templates = [(b, t) for b, t in _RS_TEMPLATE.findall(call) if template_tree(t)]
        derives = _RS_DERIVE.findall(call)
        if len(templates) != 1 or len(derives) != 1:
            bad.append(
                f"build.rs `generate_config(…)` names {len(templates)} vendored template(s) and "
                f"makes {len(derives)} `vendored_project_version(…)` call(s); expected exactly "
                "one of each, so the two can be paired."
            )
            continue
        (tpl_binding, tpl_path), (der_binding, der_project) = templates[0], derives[0]
        name = template_tree(tpl_path)
        seen[name] += 1
        w = WIRING[name]
        if tpl_binding != w.rs_root:
            bad.append(
                f"build.rs configures `{tpl_path}` from binding `{tpl_binding}`, but that "
                f"template belongs to {name}, whose binding is `{w.rs_root}`. Crosswired."
            )
        if der_binding != tpl_binding:
            bad.append(
                f"build.rs `generate_config` reads its template from `{tpl_binding}` but "
                f"derives the version from `{der_binding}` — CROSSWIRED. The version written "
                "into a template must come from that template's own tree (issue 1069)."
            )
        if der_project != w.project:
            bad.append(
                f"build.rs configures {name}'s template but derives project `{der_project}`; "
                f"{name}'s upstream id is `{w.project}`. Crosswired."
            )
    for name, n in sorted(seen.items()):
        if n != 1:
            bad.append(
                f"build.rs `main` makes {n} `generate_config(…)` call(s) for {name}; expected "
                "exactly one. A template configured twice, or not at all, has no version this "
                "gate can pair."
            )
    return bad


def self_test() -> None:
    """Runs on the NORMAL path (`check-gate-selftests`): a negative control
    nobody runs decays into a comment."""
    # (1) the upstream statement parses, quoted and bare, and only for its own id.
    src = (
        "if(NOT UCDR_SUPERBUILD)\n"
        '    project(microcdr VERSION "2.0.2" LANGUAGES C)\n'
        "else()\n"
        "    project(ucdr_superbuild NONE)\n"
        "endif()\n"
    )
    assert project_versions(src, "microcdr") == ["2.0.2"], project_versions(src, "microcdr")
    assert project_versions(src, "ucdr_superbuild") == []
    assert project_versions("project(x VERSION 1.2.3)\n", "x") == ["1.2.3"]
    # A commented-out statement is not a statement.
    assert project_versions("# project(x VERSION 1.2.3)\n", "x") == []

    # (2) the client's expected-micro-CDR line.
    assert _EXPECTED_UCDR.match("set(_microcdr_version 2.0.2)").group(1) == "2.0.2"

    # (3) the regression this gate exists for: a lane restating a version.
    leaky = 'let h = t.replace("@PROJECT_VERSION@", "2.4.1");\n'
    assert lane_literals(leaky, "//", {}) == ["2.4.1"], lane_literals(leaky, "//", {})
    #     ...and it must not fire on a lane that derives it.
    clean = 'let [maj, min, pat] = vendored_project_version(&p, "microcdr");\n'
    assert lane_literals(clean, "//", {}) == []
    #     ...nor on an allowlisted literal, nor on a version inside a comment.
    assert lane_literals("project(a VERSION 0.1.0)\n", "#", {"0.1.0": "own"}) == []
    assert lane_literals("# was 2.4.1 before issue 1069\n", "#", {}) == []

    # (4) the vacuous-pass guard: a lane with no literals AND no derivation.
    try:
        lane_block("fn main() {}\n", "x")
    except LookupError:
        pass
    else:  # pragma: no cover
        raise AssertionError("lane_block accepted a file with no derivation block")
    assert "microcdr" in lane_block(f"a{_BEGIN} microcdr {_END}b", "x")

    # --- (5) CMake wiring, on a miniature of the real file -------------------
    def cm(cdr_dir="MICROCDR_DIR", cdr_proj="microcdr", ucdr_repoint="_ucdr"):
        head = (
            'set(MICROXRCEDDS_CLIENT_DIR "${S}/../xrce-sys/micro-xrce-dds-client" CACHE PATH "")\n'
            'set(MICROCDR_DIR "${S}/../xrce-sys/micro-cdr" CACHE PATH "")\n'
            '_nros_xrce_project_version("${MICROXRCEDDS_CLIENT_DIR}/CMakeLists.txt"\n'
            "    microxrcedds_client _uxr)\n"
            f'_nros_xrce_project_version("${{{cdr_dir}}}/CMakeLists.txt"\n'
            f"    {cdr_proj} _ucdr)\n"
            'set(_uxr_config_in "${MICROXRCEDDS_CLIENT_DIR}/include/uxr/client/config.h.in")\n'
        )
        uxr = "".join(
            f"set(PROJECT_VERSION{s}  ${{_uxr_VERSION{s}}})\n"
            for s in ("_MAJOR", "_MINOR", "_PATCH", "")
        )
        cdr = "".join(
            f"set(PROJECT_VERSION{s}  ${{{ucdr_repoint}_VERSION{s}}})\n"
            for s in ("_MAJOR", "_MINOR", "_PATCH", "")
        )
        return (
            head
            + uxr
            + 'configure_file("${_uxr_config_in}" "${o}" @ONLY)\n'
            + cdr
            + 'configure_file(\n    "${MICROCDR_DIR}/include/ucdr/config.h.in"\n'
            '    "${o2}"\n    @ONLY)\n'
        )

    assert cmake_wiring_problems(cm()) == [], cmake_wiring_problems(cm())
    #     the review mutation: micro-CDR's derivation reads the CLIENT's tree.
    #     Shape is perfect — same call, same arity, no literals — so only the
    #     pairing catches it.
    crosswired = cmake_wiring_problems(cm(cdr_dir="MICROXRCEDDS_CLIENT_DIR", cdr_proj="microxrcedds_client"))
    assert any("CROSSWIRED" in p for p in crosswired), crosswired
    #     the 1069 leak: micro-CDR's `configure_file` inherits the client's tokens.
    leak = cmake_wiring_problems(cm(ucdr_repoint="_uxr"))
    assert any("issue 1069 verbatim" in p for p in leak), leak
    #     one hop earlier: the dir var itself points at the wrong checkout.
    assert any(
        "not the `micro-cdr` checkout" in p
        for p in cmake_wiring_problems(
            cm().replace('set(MICROCDR_DIR "${S}/../xrce-sys/micro-cdr"', 'set(MICROCDR_DIR "${S}/../xrce-sys/micro-xrce-dds-client"')
        )
    )

    # --- (6) cargo wiring, same three vectors --------------------------------
    def rs(cdr_dir="micro-cdr", tpl_root="microcdr", der_root="microcdr", der_proj="microcdr"):
        return (
            "fn main() {\n"
            f'    let microcdr = xrce_sys.join("{cdr_dir}");\n'
            '    let microxrce = xrce_sys.join("micro-xrce-dds-client");\n'
            "    generate_config(\n"
            "        &out_dir,\n"
            f'        &{tpl_root}.join("include/ucdr/config.h.in"),\n'
            '        "include/ucdr/config.h",\n'
            '        "ucdr",\n'
            f'        vendored_project_version(&{der_root}.join("CMakeLists.txt"), "{der_proj}"),\n'
            "    );\n"
            "    generate_config(\n"
            "        &out_dir,\n"
            '        &microxrce.join("include/uxr/client/config.h.in"),\n'
            '        "include/uxr/client/config.h",\n'
            '        "uxr",\n'
            '        vendored_project_version(&microxrce.join("CMakeLists.txt"),\n'
            '            "microxrcedds_client"),\n'
            "    );\n"
            "}\n"
        )

    assert rust_wiring_problems(rs()) == [], rust_wiring_problems(rs())
    #     vector 1: the `let` binds the wrong checkout.
    assert any(
        'not `"micro-cdr"`' in p for p in rust_wiring_problems(rs(cdr_dir="micro-xrce-dds-client"))
    ), rust_wiring_problems(rs(cdr_dir="micro-xrce-dds-client"))
    #     vector 2: the call reads micro-CDR's template from the client binding.
    assert any(
        "whose binding is `microcdr`" in p
        for p in rust_wiring_problems(rs(tpl_root="microxrce"))
    ), rust_wiring_problems(rs(tpl_root="microxrce"))
    #     vector 3: the derivation in the same call reads the other binding / id.
    assert any("CROSSWIRED" in p for p in rust_wiring_problems(rs(der_root="microxrce")))
    assert any(
        "upstream id is `microcdr`" in p
        for p in rust_wiring_problems(rs(der_proj="microxrcedds_client"))
    )
    #     ...and a template configured twice, or not at all, is reported.
    assert any(
        "expected exactly one" in p
        for p in rust_wiring_problems(rs().replace("uxr/client/config.h.in", "ucdr/config.h.in"))
    )
    # The paren balancer must not stop at the NESTED call's `)`.
    args = rust_call_args("f(a, g(b), c);", "f(")
    assert args == ["a, g(b), c"], args

    # (7) sdk-index rows, and that a `[tool.*]` version cannot be mistaken for one.
    idx = (
        '[tool.xrce-agent]\nversion = "2.4.3-nros1"\n'
        '[source.micro-cdr]\nversion = "2.0.2"\ndest = "x"\n'
    )
    assert sdk_index_versions(idx) == {"micro-cdr": "2.0.2"}, sdk_index_versions(idx)


def main() -> int:
    self_test()

    bad: list[str] = []
    notes: list[str] = []

    for p in (BUILD_RS, CMAKE, SDK_INDEX):
        if not p.exists():
            print(f"check-xrce-vendored-versions: missing {p.relative_to(REPO)}", file=sys.stderr)
            return 1

    rs_text = BUILD_RS.read_text(encoding="utf-8")
    cm_text = CMAKE.read_text(encoding="utf-8")

    # (1) each vendored tree states exactly one version.
    versions: dict[str, str] = {}
    client_text = ""
    for name, w in WIRING.items():
        cml = REPO / VENDOR_ROOT / w.dirname / "CMakeLists.txt"
        if not cml.is_file():
            notes.append(
                f"SKIP `{name}` — {VENDOR_ROOT}/{w.dirname}/CMakeLists.txt not checked out "
                "(`nros setup --source …`); its version is unverified"
            )
            continue
        text = cml.read_text(encoding="utf-8")
        if name == "micro-xrce-dds-client":
            client_text = text
        found = project_versions(text, w.project)
        if len(found) != 1:
            bad.append(
                f"{VENDOR_ROOT}/{w.dirname}/CMakeLists.txt states {len(found)} "
                f"`project({w.project} VERSION …)` lines, expected exactly 1. Upstream changed "
                "how it states its version — update the parser in BOTH lanes "
                "(`_nros_xrce_project_version` in nros-rmw-xrce/CMakeLists.txt, "
                "`vendored_project_version` in nros-rmw-xrce-cffi/build.rs) and here, together."
            )
            continue
        versions[name] = found[0]

    # (2) the client's expected micro-CDR is the one we vendor.
    if client_text and "micro-cdr" in versions:
        expected = [
            m.group(1)
            for m in (_EXPECTED_UCDR.match(l.split("#", 1)[0]) for l in client_text.splitlines())
            if m
        ]
        if len(expected) != 1:
            bad.append(
                "micro-xrce-dds-client/CMakeLists.txt states "
                f"{len(expected)} `set(_microcdr_version …)` lines, expected 1 — upstream "
                "moved its micro-CDR pin, so this cross-check no longer reads it."
            )
        elif expected[0] != versions["micro-cdr"]:
            bad.append(
                f"micro-XRCE-DDS-Client {versions.get('micro-xrce-dds-client', '?')} expects "
                f"micro-CDR {expected[0]}, but we vendor micro-CDR {versions['micro-cdr']}. "
                "One submodule was bumped and the other was not — move the other pin, or "
                "record here why the mismatch is deliberate."
            )

    # (3) neither lane holds a version literal.
    for path, text, marker in ((CMAKE, cm_text, "#"), (BUILD_RS, rs_text, "//")):
        for lit in lane_literals(text, marker, LITERAL_ALLOWLIST[path]):
            bad.append(
                f"{path.relative_to(REPO)} spells the version literal `{lit}` — issue 1069: "
                "neither lane restates a vendored version, both read "
                "`project(<name> VERSION …)` out of the vendored CMakeLists. Derive it "
                "(or add it to this gate's LITERAL_ALLOWLIST with the reason it is not a "
                "vendored version)."
            )

    # (4) both lanes carry the derivation — otherwise (3) passes vacuously.
    for path, text in ((CMAKE, cm_text), (BUILD_RS, rs_text)):
        label = str(path.relative_to(REPO))
        try:
            block = lane_block(text, label)
        except LookupError as e:
            bad.append(str(e))
            continue
        if "CMakeLists.txt" not in block:
            bad.append(
                f"{label}'s {_BEGIN} block does not read a vendored `CMakeLists.txt` — "
                "the version has to come from the tree's own statement."
            )

    # (5)/(6) the wiring: each tree's tokens come from that tree.
    bad += cmake_wiring_problems(cm_text)
    bad += rust_wiring_problems(rs_text)

    # (7) the authored sdk-index rows agree with the trees.
    rows = sdk_index_versions(SDK_INDEX.read_text(encoding="utf-8"))
    for name, w in WIRING.items():
        if name not in versions:
            continue  # skipped loudly above
        if w.sdk_row not in rows:
            bad.append(
                f"nros-sdk-index.toml has no `[source.{w.sdk_row}] version` — the gate that "
                "keeps it honest cannot find it (renamed?)."
            )
        elif rows[w.sdk_row] != versions[name]:
            bad.append(
                f'nros-sdk-index.toml `[source.{w.sdk_row}] version = "{rows[w.sdk_row]}"` but '
                f"the vendored tree says {versions[name]}. That row is a factual claim about "
                "the checkout — `gen-support-status.py` renders it into the book's version "
                f"table — so set it to `{versions[name]}` (and re-run "
                "`python3 scripts/gen-support-status.py` in the same commit). It is authored "
                "rather than derived on purpose: `nros setup --source` runs before the tree "
                "exists."
            )

    for note in notes:
        print(f"check-xrce-vendored-versions: {note}")
    if bad:
        print("check-xrce-vendored-versions: FAIL", file=sys.stderr)
        for m in bad:
            print(f"  - {m}", file=sys.stderr)
        return 1

    if not versions:
        print(
            "check-xrce-vendored-versions: no vendored tree checked out — nothing verified. "
            "Run `nros setup --source micro-cdr --source micro-xrce-dds-client`.",
            file=sys.stderr,
        )
        return 1

    print(
        "check-xrce-vendored-versions: OK — "
        + ", ".join(f"{t} {v}" for t, v in sorted(versions.items()))
        + "; both lanes derive them from `project(<name> VERSION …)`, each tree wired to its "
        "own template end to end, sdk-index agrees."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
