#!/usr/bin/env python3
"""One vendored tree, two compilers, ONE set of values — phase-420 W9.

Sibling of `check-xrce-source-manifest.py`. That one keeps the two XRCE lanes
from re-growing a source list each; this one keeps them from re-growing a
CONFIGURATION each.

What was wrong
--------------

`nros-rmw-xrce-cffi/build.rs` (cc-rs, the Rust lane) filled the two upstream
`config.h.in` templates in by hand, and `nros-rmw-xrce/CMakeLists.txt` (the
C/C++ lane) filled the same two in with `configure_file`. Every `@TOKEN@` value
was written twice — the four MTUs, the session limits, every
`UCLIENT_PROFILE_*` / `UCLIENT_PLATFORM_*` toggle — with nothing between them.
They had already diverged: `zephyr/Kconfig` defines six `CONFIG_NROS_XRCE_*`
options, the Rust lane read all six, and the CMake lane read ZERO
(`grep -c CONFIG_NROS_XRCE` on it was 0), compiling the defaults instead.

Measured (phase-420 W9): that was NOT an issue-0135 ABI split, because no
single image can hold TUs from both lanes — the CMake project is configured
only by `just check rmw-xrce` and its archive reaches only its own two CTest
binaries. It was a CTest harness validating values no image compiles.

What it checks
--------------

1. `xrce-config.txt` parses and is internally consistent: no token bound twice
   in one template, no unknown template, `default >= min`, every env name
   `CONFIG_`-derivable.
2. Both templates are COVERED: every `@TOKEN@` the upstream `config.h.in`
   takes has a `value` or `knob` row (or is a `@PROJECT_VERSION*@`, which is
   derived from the vendored tree — issue 1069), and every `#cmakedefine` has
   a `flag` row. A token the manifest misses is substituted as an empty macro
   body in one lane and whatever the other lane wrote in the other.
3. NEITHER LANE STATES A VALUE OF ITS OWN. This is the check that makes the
   mirror unrecreatable: a `UCLIENT_*` literal in either lane, or an
   `XRCE_MAX_*`-style `-D` name, is a value the other lane cannot see.
4. Every `flag`'s condition token is one the manifest vocabulary already has —
   the one `check-xrce-source-manifest` proves both lanes answer.
5. THE WIRING, not just the shape. Every `CONFIG_NROS_XRCE_*` option
   `zephyr/cmake/nros_cargo_build.cmake` forwards is bound by this manifest,
   and its Kconfig `Maps to <SYMBOL>[, <SYMBOL>]…` line names EXACTLY the
   symbols the manifest binds it to. A row whose shape is perfect and whose
   env name points at the wrong knob passes checks 1-4 and fails this one.

Run: python3 scripts/check-xrce-config-manifest.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
XRCE = REPO / "packages/rmw/xrce"
MANIFEST = XRCE / "xrce-config.txt"
SOURCES = XRCE / "xrce-sources.txt"
BUILD_RS = XRCE / "nros-rmw-xrce-cffi/build.rs"
CMAKE = XRCE / "nros-rmw-xrce/CMakeLists.txt"
KCONFIG = REPO / "zephyr/Kconfig"
KNOB_BRIDGE = REPO / "zephyr/cmake/nros_cargo_build.cmake"

# template name → the upstream `config.h.in` it fills, relative to the repo.
TEMPLATES = {
    "uxr": "packages/rmw/xrce/xrce-sys/micro-xrce-dds-client/include/uxr/client/config.h.in",
    "ucdr": "packages/rmw/xrce/xrce-sys/micro-cdr/include/ucdr/config.h.in",
}

# Tokens the manifest deliberately does NOT state, with the reason. Both come
# from the vendored tree's own `project(<name> VERSION …)` line (issue 1069);
# a version stated in our manifest would be a fifth hand-written restatement of
# a fact upstream already holds.
DERIVED_TOKENS = {
    "PROJECT_VERSION",
    "PROJECT_VERSION_MAJOR",
    "PROJECT_VERSION_MINOR",
    "PROJECT_VERSION_PATCH",
}

# Env names the manifest binds that `nros_cargo_build.cmake` does NOT forward,
# with the reason. Both are env / `[knobs.xrce]`-rung only; neither has a
# Kconfig option, so neither can be mis-forwarded.
NO_KCONFIG_OPTION = {
    "NROS_XRCE_CUSTOM_TRANSPORT_MTU": "phase-207.6 — env + [knobs.xrce] rung; no Kconfig option",
    "NROS_XRCE_SUBSCRIBER_RING_DEPTH": "phase-237 — env only; no Kconfig option",
}

# A `UCLIENT_*` or `XRCE_*` literal a lane may legitimately name, with why.
LANE_VALUE_ALLOWLIST = {
    BUILD_RS: {
        # A compile-ENVIRONMENT define, not a configuration value: it tells
        # `<uxr/client/config_internal.h>` not to require the POSIX TUs the
        # source manifest just dropped, so it is paired with the `posix`
        # condition rather than with a token in either template. The CMake lane
        # is POSIX-only by construction and never needs it. It joins the shared
        # statement when the two COMPILES become one (the remaining W9 step);
        # until then it is at least named here rather than merely unnoticed.
        "UCLIENT_PLATFORM_NO_POSIX",
    },
    CMAKE: set(),
}

# `@TOKEN@` in an upstream template.
_AT_TOKEN = re.compile(r"@([A-Za-z_][A-Za-z0-9_]*)@")
# `#cmakedefine TOKEN`
_CMAKEDEFINE = re.compile(r"^#cmakedefine\s+([A-Za-z_][A-Za-z0-9_]*)\s*$", re.M)
# A configuration symbol named in a lane: `UCLIENT_FOO`, `XRCE_MAX_BAR`.
_LANE_SYMBOL = re.compile(r"\b(UCLIENT_[A-Z0-9_]+|XRCE_(?:MAX|MIN|BUFFER|STREAM|SUBSCRIBER)[A-Z0-9_]*)\b")
# `_nros_resolve_knob(NROS_XRCE_X` / `_nros_resolve_derivable_knob(NROS_XRCE_X`
_FORWARDED = re.compile(r"_nros_resolve(?:_derivable)?_knob\(\s*(NROS_XRCE_[A-Z0-9_]+)")
# `Maps to A, B.` inside a Kconfig help block.
_MAPS_TO = re.compile(r"^\s*Maps to ([A-Za-z0-9_, ]+?)\.\s*$", re.M)
# `config NROS_XRCE_FOO` … up to the next `config`/`endif` at column 0-ish.
_KCONFIG_OPTION = re.compile(
    r"^config (NROS_XRCE_[A-Z0-9_]+)\n(.*?)(?=^config |^endif)", re.M | re.S
)


class ManifestError(Exception):
    pass


def parse_manifest(text: str, where: str = str(MANIFEST)):
    """→ (values, knobs, flags, defines), each a list of tuples.

    Same grammar both lanes implement: `#` comments, blank lines ignored,
    whitespace-separated columns, four record types.
    """
    values: list[tuple[str, str, str]] = []
    knobs: list[tuple[str, str, str, int, int]] = []
    flags: list[tuple[str, str, str]] = []
    defines: list[tuple[str, str, int]] = []
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        f = line.split()
        try:
            if f[0] == "value" and len(f) == 4:
                values.append((f[1], f[2], f[3]))
            elif f[0] == "knob" and len(f) == 6:
                knobs.append((f[1], f[2], f[3], int(f[4]), int(f[5])))
            elif f[0] == "flag" and len(f) == 4:
                flags.append((f[1], f[2], f[3]))
            elif f[0] == "define" and len(f) == 4:
                defines.append((f[1], f[2], int(f[3])))
            else:
                raise ValueError
        except ValueError:
            raise ManifestError(
                f"{where}:{n}: expected `value <template> <token> <literal>`, "
                f"`knob <template> <token> <env> <default> <min>`, "
                f"`flag <template> <token> <condition>` or "
                f"`define <macro> <env> <min>`, got `{line}`"
            ) from None
    return values, knobs, flags, defines


def manifest_problems(values, knobs, flags, defines, where: str = str(MANIFEST)) -> list[str]:
    """Internal consistency of the manifest alone — no lanes, no templates."""
    bad: list[str] = []
    bound: set[tuple[str, str]] = set()
    for template, token, _ in values:
        if template not in TEMPLATES:
            bad.append(f"{where}: `{token}` names unknown template `{template}`")
        if (template, token) in bound:
            bad.append(f"{where}: `{template}/{token}` bound twice")
        bound.add((template, token))
    for template, token, env, default, minimum in knobs:
        if template not in TEMPLATES:
            bad.append(f"{where}: `{token}` names unknown template `{template}`")
        if (template, token) in bound:
            bad.append(f"{where}: `{template}/{token}` bound twice")
        bound.add((template, token))
        if default < minimum:
            bad.append(
                f"{where}: `{token}` default {default} is below its own minimum {minimum} — "
                "both lanes would reject the value nobody stated"
            )
        if not env.startswith("NROS_"):
            bad.append(
                f"{where}: knob env `{env}` does not start with NROS_; the Kconfig option is "
                "`CONFIG_<env>`, which is DERIVED rather than tabulated (issue 0460)"
            )
    for template, token, _cond in flags:
        if template not in TEMPLATES:
            bad.append(f"{where}: `{token}` names unknown template `{template}`")
        if (template, token) in bound:
            bad.append(f"{where}: `{template}/{token}` bound twice")
        bound.add((template, token))
    seen_macro: set[str] = set()
    for macro, env, _min in defines:
        if macro in seen_macro:
            bad.append(f"{where}: `-D{macro}` bound twice")
        seen_macro.add(macro)
        if not env.startswith("NROS_"):
            bad.append(f"{where}: define env `{env}` does not start with NROS_")
    if not (values or knobs or flags):
        bad.append(f"{where}: no substitutions at all")
    return bad


def template_problems(template: str, text: str, values, knobs, flags) -> list[str]:
    """Every `@TOKEN@` and `#cmakedefine` upstream takes is covered."""
    bad: list[str] = []
    stated = {t for tm, t, _ in values if tm == template}
    stated |= {t for tm, t, _e, _d, _m in knobs if tm == template}
    toggles = {t for tm, t, _c in flags if tm == template}
    for token in sorted(set(_AT_TOKEN.findall(text))):
        if token in DERIVED_TOKENS or token in stated:
            continue
        bad.append(
            f"{TEMPLATES[template]} takes `@{token}@`, which xrce-config.txt does not state. "
            "An unstated token substitutes EMPTY in one lane and whatever the other lane "
            "wrote in the other — add a `value`/`knob` row."
        )
    for token in sorted(set(_CMAKEDEFINE.findall(text))):
        if token not in toggles:
            bad.append(
                f"{TEMPLATES[template]} has `#cmakedefine {token}`, which xrce-config.txt does "
                "not state. An omitted toggle is `/* #undef */` under configure_file and an "
                "UNTOUCHED `#cmakedefine` line under build.rs — add a `flag` row."
            )
    for token in sorted(stated | toggles):
        if f"@{token}@" not in text and f"#cmakedefine {token}" not in text:
            bad.append(
                f"xrce-config.txt states `{template}/{token}`, which "
                f"{TEMPLATES[template]} does not take — a value both lanes carry and neither "
                "uses. Upstream renamed it, or the row is dead."
            )
    return bad


def strip_comments(text: str, marker: str) -> str:
    """Drop everything from `marker` to end-of-line, line by line."""
    return "\n".join(line.split(marker, 1)[0] for line in text.splitlines())


def lane_values(text: str, marker: str, allow: set[str]) -> list[str]:
    """Configuration symbols a lane names in code, minus the allowlist."""
    hits = _LANE_SYMBOL.findall(strip_comments(text, marker))
    return sorted({h for h in hits if h not in allow})


def kconfig_maps_to(text: str) -> dict[str, list[str]]:
    """`config NROS_XRCE_*` → the symbols its help block says it maps to."""
    out: dict[str, list[str]] = {}
    for name, body in _KCONFIG_OPTION.findall(text):
        hits = _MAPS_TO.findall(body)
        out[name] = [s.strip() for s in hits[0].split(",")] if hits else []
    return out


def manifest_bindings(knobs, defines) -> dict[str, list[str]]:
    """env name → the symbols the manifest binds it to, in manifest order."""
    out: dict[str, list[str]] = {}
    for _template, token, env, _d, _m in knobs:
        out.setdefault(env, []).append(token)
    for macro, env, _m in defines:
        out.setdefault(env, []).append(macro)
    return out


def self_test() -> None:
    """Runs on the NORMAL path — a negative control nobody runs decays into a
    comment (`check-gate-selftests`)."""
    good = (
        "# a comment\n"
        "value ucdr CONFIG_MACHINE_ENDIANNESS 1\n"
        "knob  uxr  UCLIENT_UDP_TRANSPORT_MTU NROS_XRCE_TRANSPORT_MTU 4096 128\n"
        "flag  uxr  UCLIENT_PROFILE_UDP posix_ip  # trailing\n"
        "define XRCE_BUFFER_SIZE NROS_XRCE_BUFFER_SIZE 64\n"
    )
    v, k, f, d = parse_manifest(good, "T")
    assert v == [("ucdr", "CONFIG_MACHINE_ENDIANNESS", "1")], v
    assert k == [("uxr", "UCLIENT_UDP_TRANSPORT_MTU", "NROS_XRCE_TRANSPORT_MTU", 4096, 128)], k
    assert f == [("uxr", "UCLIENT_PROFILE_UDP", "posix_ip")], f
    assert d == [("XRCE_BUFFER_SIZE", "NROS_XRCE_BUFFER_SIZE", 64)], d
    assert manifest_problems(v, k, f, d, "T") == []

    for broken in ("value uxr X\n", "knob uxr X E 1\n", "flag uxr X\n", "banana\n",
                   "knob uxr X NROS_E abc 1\n"):
        try:
            parse_manifest(broken, "T")
        except ManifestError:
            pass
        else:  # pragma: no cover
            raise AssertionError(f"parser accepted {broken!r}")

    # Manifest defects each report.
    assert any(
        "bound twice" in m
        for m in manifest_problems(
            [("uxr", "A", "1"), ("uxr", "A", "2")], [], [], [], "T"
        )
    )
    assert any(
        "unknown template" in m for m in manifest_problems([("zzz", "A", "1")], [], [], [], "T")
    )
    assert any(
        "below its own minimum" in m
        for m in manifest_problems([], [("uxr", "A", "NROS_E", 1, 128)], [], [], "T")
    )
    assert any(
        "does not start with NROS_" in m
        for m in manifest_problems([], [("uxr", "A", "E", 128, 1)], [], [], "T")
    )

    # Template coverage, both directions.
    tmpl = "#define V @PROJECT_VERSION@\n#cmakedefine UCLIENT_X\n#define M @UCLIENT_Y@\n"
    assert template_problems("uxr", tmpl, [], [], []) != []
    ok = template_problems(
        "uxr", tmpl, [("uxr", "UCLIENT_Y", "1")], [], [("uxr", "UCLIENT_X", "always")]
    )
    assert ok == [], ok
    assert any(
        "does not take" in m
        for m in template_problems(
            "uxr",
            tmpl,
            [("uxr", "UCLIENT_Y", "1"), ("uxr", "UCLIENT_GONE", "1")],
            [],
            [("uxr", "UCLIENT_X", "always")],
        )
    )

    # THE REGRESSION: a lane states a value of its own.
    assert lane_values('set(UCLIENT_UDP_TRANSPORT_MTU 4096)', "#", set()) == [
        "UCLIENT_UDP_TRANSPORT_MTU"
    ]
    assert lane_values('build.define("XRCE_BUFFER_SIZE", "256");', "//", set()) == [
        "XRCE_BUFFER_SIZE"
    ]
    # ...and not on prose, nor on an allowlisted symbol.
    assert lane_values("# UCLIENT_PROFILE_UDP is set by the manifest\n", "#", set()) == []
    assert lane_values('r.rungs.stream_history // XRCE_STREAM_HISTORY\n', "//", set()) == []
    assert lane_values('m("XRCE_STREAM_HISTORY")', "//", {"XRCE_STREAM_HISTORY"}) == []

    # THE WIRING. Shape stays valid; the knob points at the wrong token.
    kcfg = (
        "config NROS_XRCE_TRANSPORT_MTU\n"
        "    int \"x\"\n"
        "    help\n"
        "      Maps to UCLIENT_UDP_TRANSPORT_MTU, UCLIENT_TCP_TRANSPORT_MTU.\n"
        "config NROS_XRCE_BUFFER_SIZE\n"
        "    int \"y\"\n"
        "    help\n"
        "      Maps to XRCE_BUFFER_SIZE.\n"
        "endif\n"
    )
    maps = kconfig_maps_to(kcfg)
    assert maps == {
        "NROS_XRCE_TRANSPORT_MTU": ["UCLIENT_UDP_TRANSPORT_MTU", "UCLIENT_TCP_TRANSPORT_MTU"],
        "NROS_XRCE_BUFFER_SIZE": ["XRCE_BUFFER_SIZE"],
    }, maps
    wired = manifest_bindings(
        [
            ("uxr", "UCLIENT_UDP_TRANSPORT_MTU", "NROS_XRCE_TRANSPORT_MTU", 4096, 128),
            ("uxr", "UCLIENT_TCP_TRANSPORT_MTU", "NROS_XRCE_TRANSPORT_MTU", 4096, 128),
        ],
        [("XRCE_BUFFER_SIZE", "NROS_XRCE_BUFFER_SIZE", 64)],
    )
    for env, syms in maps.items():
        assert sorted(wired[env]) == sorted(syms), (env, wired[env], syms)
    crossed = manifest_bindings(
        [
            # Same shape, wrong wire: the UDP MTU now reads the BUFFER_SIZE knob.
            ("uxr", "UCLIENT_UDP_TRANSPORT_MTU", "NROS_XRCE_BUFFER_SIZE", 4096, 128),
            ("uxr", "UCLIENT_TCP_TRANSPORT_MTU", "NROS_XRCE_TRANSPORT_MTU", 4096, 128),
        ],
        [("XRCE_BUFFER_SIZE", "NROS_XRCE_BUFFER_SIZE", 64)],
    )
    assert sorted(crossed["NROS_XRCE_TRANSPORT_MTU"]) != sorted(
        maps["NROS_XRCE_TRANSPORT_MTU"]
    ), "a crossed wire must not match the Kconfig contract"

    assert _FORWARDED.findall("_nros_resolve_knob(NROS_XRCE_BUFFER_SIZE \"${X}\")") == [
        "NROS_XRCE_BUFFER_SIZE"
    ]
    assert _FORWARDED.findall(
        "_nros_resolve_derivable_knob(NROS_XRCE_MAX_SUBSCRIBERS\n \"${X}\" Y)"
    ) == ["NROS_XRCE_MAX_SUBSCRIBERS"]


def main() -> int:
    self_test()

    bad: list[str] = []
    notes: list[str] = []

    for p in (MANIFEST, SOURCES, BUILD_RS, CMAKE, KCONFIG, KNOB_BRIDGE):
        if not p.exists():
            print(f"check-xrce-config-manifest: missing {p.relative_to(REPO)}", file=sys.stderr)
            return 1

    try:
        values, knobs, flags, defines = parse_manifest(MANIFEST.read_text(encoding="utf-8"))
    except ManifestError as e:
        print(f"check-xrce-config-manifest: {e}", file=sys.stderr)
        return 1
    bad += manifest_problems(values, knobs, flags, defines)

    # (2) template coverage, both directions.
    for template, rel in sorted(TEMPLATES.items()):
        path = REPO / rel
        if not path.is_file():
            notes.append(
                f"SKIP template `{template}` — {rel} not checked out "
                "(`nros setup --source …`); its coverage is unverified"
            )
            continue
        bad += template_problems(
            template, path.read_text(encoding="utf-8"), values, knobs, flags
        )

    # (3) neither lane states a value of its own.
    for path, marker in ((BUILD_RS, "//"), (CMAKE, "#")):
        text = path.read_text(encoding="utf-8")
        for hit in lane_values(text, marker, LANE_VALUE_ALLOWLIST[path]):
            bad.append(
                f"{path.relative_to(REPO)} states `{hit}` — phase-420 W9: the lanes hold no "
                "configuration values, they read packages/rmw/xrce/xrce-config.txt. Put it "
                "there (or on this gate's allowlist, with the reason it is not shared)."
            )

    # (4) every flag's condition is in the shared vocabulary.
    source_conditions = {
        line.split()[2]
        for line in (
            raw.split("#", 1)[0].strip() for raw in SOURCES.read_text(encoding="utf-8").splitlines()
        )
        if line.startswith("group ")
    }
    # `never` is this manifest's own addition — a toggle compiled off on every
    # target has no file to select, so xrce-sources.txt never needs it. Both
    # lanes answer it; `check-xrce-source-manifest` reads BOTH manifests when it
    # decides which tokens a lane may answer.
    vocabulary = source_conditions | {"never"}
    for _t, token, cond in flags:
        if cond not in vocabulary:
            bad.append(
                f"xrce-config.txt: flag `{token}` uses condition `{cond}`, which is not in the "
                f"shared vocabulary {sorted(vocabulary)}. Add a `group` to xrce-sources.txt or "
                "an arm to BOTH lanes' NROS-XRCE-CONDITIONS block — a token only one lane "
                "answers is issue 1068 again."
            )

    # (5) THE WIRING — Kconfig's `Maps to` line against the manifest's binding.
    bindings = manifest_bindings(knobs, defines)
    forwarded = set(_FORWARDED.findall(KNOB_BRIDGE.read_text(encoding="utf-8")))
    maps_to = kconfig_maps_to(KCONFIG.read_text(encoding="utf-8"))

    for env in sorted(forwarded):
        if env not in bindings:
            bad.append(
                f"zephyr/cmake/nros_cargo_build.cmake forwards `{env}`, which xrce-config.txt "
                "binds to nothing — that knob reaches NEITHER lane's compile. This is the "
                "defect phase-420 W9 fixed for all six at once; do not fix one."
            )
    for env in sorted(bindings):
        if env in forwarded or env in NO_KCONFIG_OPTION:
            continue
        bad.append(
            f"xrce-config.txt binds `{env}`, which nros_cargo_build.cmake does not forward and "
            "which is not on this gate's NO_KCONFIG_OPTION list. Either wire it through Kconfig "
            "or record why it is env-only."
        )
    for env in sorted(set(maps_to) & set(bindings)):
        want = sorted(maps_to[env])
        got = sorted(bindings[env])
        if not want:
            bad.append(
                f"zephyr/Kconfig: `config {env}` has no `Maps to <SYMBOL>.` line. That line is "
                f"the CONTRACT this gate checks the wiring against; xrce-config.txt binds it to "
                f"{got}."
            )
        elif want != got:
            bad.append(
                f"zephyr/Kconfig says `{env}` maps to {want}; xrce-config.txt binds it to {got}. "
                "One of the two is wired to the wrong knob — the shape is fine either way, so "
                "nothing but this comparison would say so."
            )

    for note in notes:
        print(f"check-xrce-config-manifest: {note}")
    if bad:
        print("check-xrce-config-manifest: FAIL", file=sys.stderr)
        for m in bad:
            print(f"  - {m}", file=sys.stderr)
        return 1

    print(
        "check-xrce-config-manifest: OK — "
        f"{len(values)} values, {len(knobs)} knobs, {len(flags)} flags, "
        f"{len(defines)} defines; {len(forwarded)} Kconfig knobs reach both lanes"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
