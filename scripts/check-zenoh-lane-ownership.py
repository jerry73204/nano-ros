#!/usr/bin/env python3
"""One vendored tree, two compilers, ONE owner per platform — phase-420 W9.

THE MEASUREMENT THIS GATE PRESERVES (2026-09-05)
------------------------------------------------

W9's zenoh bullet asked whether a single Zephyr image can contain zenoh-pico
TUs from BOTH lanes — the cargo one (`nros-zpico-build`, cc-rs) and the
west/cmake one (`zephyr/cmake/nros_rmw_zenoh.cmake`). Unlike XRCE next door,
the cmake lane here IS reached from a real image build
(`zephyr/CMakeLists.txt:165`), so a positive answer would have been an
issue-0135 ABI split: two independently-resolved `Z_FEATURE_*` sets deciding
flag-gated struct layouts inside one link.

Measured answer: **NO — the lanes are disjoint, per platform.** Zephyr's
`nros-platform.toml` declares `[build.zenoh] compiled_by = "platform"`, and
`runner.rs` skips `build_zenoh_pico_unified` for exactly that (issue 0534/0541;
the cargo lane cannot compile these TUs on Zephyr at all — `system/platform/
zephyr.h` includes `version.h`, which only the west build generates). The one C
TU cargo does emit into a Zephyr image is the in-repo alias TU
`zpico-sys/c/zpico/platform_aliases.c`, which includes no vendored header.

So the duplication W9 names is a PER-PLATFORM SPLIT, not a duplicate compile,
and "one build consumed by both lanes" is not the shape the evidence supports.
What the evidence does support is this: **the split is load-bearing and nothing
was checking it.** It rests on three separate statements that must agree —
a TOML field, a Rust guard, and which lane answers a manifest condition token —
and a Zephyr C/C++ image links with `-Wl,--allow-multiple-definition`
(`zephyr/CMakeLists.txt`, for the Rust-staticlib allocator collision), so if
both lanes ever did compile the tree the duplicate symbols would NOT stop the
link. One lane's objects would silently win. That is the failure this gate
exists to make impossible.

What it checks
--------------

1. Every platform declares its zenoh owner (`[build.zenoh] compiled_by`,
   defaulting to `cargo` exactly as `CompiledBy::default()` does), and the value
   is one of the two the enum has.
2. **`compiled_by = "platform"` ⟺ a non-cargo lane compiles the tree for it**,
   both directions, against the authored `WEST_LANES` table. A platform-owned
   platform with no lane means NOTHING compiles the tree; a lane for a
   cargo-owned platform means BOTH do.
3. The west lane actually compiles what it reads — it names the shared manifest
   AND feeds the expansion into a compile call. A lane that reads the manifest
   and drops the result on the floor satisfies (2) while building nothing.
4. The cargo lane's guard is live: `build_zenoh_pico_unified` is called only
   under a `CompiledBy::Cargo` test. Without it `compiled_by` is a comment.
5. **THE CROSS-WIRE CHECK, and it is the reason this gate exists separately
   from `check-zenoh-source-manifest`.** That gate asserts both lanes answer the
   same SET of condition tokens; it says nothing about WHICH WAY either answers.
   Two shape-preserving mutations pass it and break the build:

     - `set(_zenoh_cond_zephyr FALSE)` in the cmake lane drops
       `system/zephyr/network.c` from every Zephyr image, which is the phase
       160.C ABI mismatch — `Transport(ConnectionFailed)` at session open on
       every Zephyr app, with the token set untouched;
     - `"zephyr" => true` in the cargo lane compiles that same Zephyr-only TU
       into every OTHER platform's build, with the token set untouched.

   So: a token naming a platform must be answered TRUE-capable by exactly the
   lane that OWNS that platform, and false-by-construction by the other. The
   cargo lane's arm must reference the platform test (an `is_<p>` binding proven
   here to be bound to `platform == "<p>"`, or that comparison inline) — never a
   bare constant. The owning west lane must set the bare platform token TRUE
   unconditionally, and a QUALIFIED token (`<p>_<qualifier>`) conditionally,
   because an unconditional qualifier is not a qualifier.
6. **A group's token names the same platform as its paths.** A record under
   `src/system/<dir>/` must sit in a group whose condition token resolves to a
   platform that `<dir>` serves. `check-zenoh-source-manifest` asserts such a
   path is in SOME conditional group; this asserts it is in the RIGHT one — a
   `system/zephyr/*.c` selected by a `freertos` token is valid in shape, agreed
   between the lanes, and wrong.
7. **Nobody else compiles the tree.** Every tracked build-ish file naming the
   vendored SOURCE root must be a declared lane or carry a TRACKED ISSUE ID in
   `KNOWN_UNSHARED_COMPILES`. This found a THIRD compiler that W9's premise, the
   shared manifest's header and `check-zenoh-source-manifest`'s docstring all
   said did not exist: `scripts/qemu/build-zenoh-pico.sh` (issue 1096), with its
   own nine-directory loop, its own config header and its own slot defaults.
   That gate's "NEITHER LANE NAMES A PATH INSIDE THE VENDORED TREE" was true of
   the two lanes it knows and false of the repository.

8. **A group's NAME agrees with its CONDITION**, on platform and on feature.
   Found by review after the first version shipped: SWAPPING two groups'
   conditions (`zephyr_system  zephyr` ⇄ `zephyr_isotp  zephyr_isotp`) leaves
   every token declared, used, answered and counted exactly as before — the
   sibling is green, and check (5) and (6) are green because both tokens name
   `zephyr` — while `system/zephyr/network.c` starts compiling ONLY under ISO-TP
   and `system/zephyr/isotp.c` compiles on every Zephyr build. The rule is the
   manifest's own argument turned into a check: the group name is what a reader
   uses to understand a record, so `zephyr_system` gated on an ISO-TP feature is
   a lie in the file itself.
9. **The mirror of (8): a record in a feature-gated group implements that
   feature.** (8) holds the groups to their names; that leaves the other
   spelling of the same swap, in which the groups are perfect and the two
   RECORDS trade places. A `<platform>_<feature>` group compiles TUs whose paths
   name that feature; a base-platform group compiles none that do.

Run: python3 scripts/check-zenoh-lane-ownership.py
"""

from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
ZENOH = REPO / "packages/rmw/zenoh"
MANIFEST = ZENOH / "zpico-sys/zenoh-sources.txt"
CMAKE = REPO / "zephyr/cmake/nros_rmw_zenoh.cmake"
RUST_LIB = ZENOH / "nros-zpico-build/src/lib.rs"
RUST_RUNNER = ZENOH / "nros-zpico-build/src/runner.rs"

# The platform search path `PlatformsTree::default_search_path` walks, in its
# order. A platform is one subdirectory holding an `nros-platform.toml`.
PLATFORM_ROOTS = ("packages/platform", "config")

# Lanes OTHER than cargo that compile the vendored zenoh-pico tree, keyed by the
# platform they build for, with the call that puts the manifest expansion into a
# compile. AUTHORED — check (2) holds it to the platform manifests in both
# directions, so adding a lane without declaring `compiled_by = "platform"`
# fails, and so does the reverse.
WEST_LANES = {
    "zephyr": (CMAKE, "zephyr_library_sources(${_zenoh_pico_sources})"),
}

# Which nano-ros platforms each `zenoh-pico/src/system/<dir>/` tree serves.
# zenoh-pico's directory vocabulary is upstream's, ours is RFC-0072's, and they
# are not the same words (`unix` serves `posix` and `nuttx`); the mapping is
# stated so check (6) can compare them at all. A `system/<dir>` absent here is a
# failure, not a pass — a new upstream platform tree needs a human.
SYSTEM_DIR_PLATFORMS = {
    "arduino": set(),
    "bare-metal": {"bare-metal"},
    "emscripten": set(),
    "espidf": {"esp-idf"},
    "flipper": set(),
    "freertos": {"freertos", "freertos-lwip"},
    "mbed": set(),
    "rpi_pico": set(),
    "threadx": {"threadx"},
    "unix": {"posix", "nuttx"},
    "windows": set(),
    "zephyr": {"zephyr"},
}

# A record whose PATH does not name the feature its group is gated on, with the
# reason. Check (9)'s escape hatch: the rule it enforces — a feature-gated group
# compiles the TUs that implement that feature, and upstream names its TUs after
# what they implement — is a property of zenoh-pico's layout, not a law, so a
# genuine exception must be sayable. Empty today, and that is the finding: every
# record in this manifest carries its own feature in its name.
FEATURE_TU_EXCEPTIONS: dict[str, str] = {}

# Files that compile the vendored zenoh-pico tree WITHOUT reading the shared
# source manifest, each with the issue tracking it. This is a debt ledger, not
# an exemption list: an entry must name an OPEN issue, and check (7) fails on
# any compiler of the tree that is neither a declared lane nor listed here — so
# a FOURTH copy cannot land quietly while the third is being paid off.
KNOWN_UNSHARED_COMPILES = {
    "scripts/qemu/build-zenoh-pico.sh": "issue 1096",
}

# What "compiles the vendored tree" looks like from the outside: a reference to
# the tree's SOURCE root. Include and header references are not this — a
# consumer naming `zenoh-pico/include` is using the library, not rebuilding it.
#
# Deliberately loose on the punctuation between the two words, because there are
# four spellings of the same reach and all four are real: `zenoh-pico/src`,
# `${ZENOH_PICO_DIR}/src`, `zenoh_pico_src` and `zenoh_pico_src.join("src")`.
# Tight on the words themselves — `zenoh-pico/include` is a CONSUMER of the
# library, not a rebuild of it, and must not match.
_TREE_SRC = re.compile(r"(?:ZENOH_PICO_DIR|zenoh[-_]pico)\S{0,10}src\b", re.IGNORECASE)

# `system/common` is the platform-INDEPENDENT half of the system layer
# (`platform.c`, `serial.c`) — core, not a platform tree, which is why
# `check-zenoh-source-manifest`'s attachment rule lets it sit in an
# unconditional group. It has no platform to be cross-wired to.
CORE_SYSTEM_DIRS = {"common"}

_BEGIN = "NROS-ZENOH-CONDITIONS-BEGIN"
_END = "NROS-ZENOH-CONDITIONS-END"

# `set(_zenoh_cond_<token> <VALUE>)` inside the cmake conditions block.
_CMAKE_SET = re.compile(r"^\s*set\(\s*_zenoh_cond_([A-Za-z0-9_]+)\s+([^)\s]+)\s*\)")
# `"<token>" => <expr>,` inside the Rust conditions block.
_RS_ARM = re.compile(r'^\s*"([A-Za-z0-9_]+)"\s*=>\s*(.+?),\s*$')
# `let is_<x> = platform == "<p>";`
_RS_BIND = re.compile(r'^\s*let\s+([A-Za-z0-9_]+)\s*=\s*platform\s*==\s*"([A-Za-z0-9_-]+)"\s*;')
# `names = ["a", "b"]` at the top level of an `nros-platform.toml`.
_NAMES = re.compile(r'^\s*names\s*=\s*\[\s*"([^"]+)"')
_SECTION = re.compile(r"^\s*\[([^\]]+)\]")
_COMPILED_BY = re.compile(r'^\s*compiled_by\s*=\s*"([^"]*)"')


class LaneError(Exception):
    """A file this gate reads is not in a shape it can read at all."""


_SIBLING = None


def _sibling():
    """`check-zenoh-source-manifest`'s parser, imported rather than re-written.

    A second parser of the same format is the very mirror this area exists to
    delete; the sibling's is the strict one (it rejects malformed records), so
    this gate consumes it instead of growing its own. `strip_comments` comes
    from there for the same reason.
    """
    global _SIBLING
    if _SIBLING is not None:
        return _SIBLING
    path = REPO / "scripts/check-zenoh-source-manifest.py"
    spec = importlib.util.spec_from_file_location("_zenoh_source_manifest", path)
    if spec is None or spec.loader is None:  # pragma: no cover
        raise LaneError(f"cannot load {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    _SIBLING = mod
    return mod


def norm(name: str) -> str:
    """Platform names are written with `-`; condition tokens with `_`.

    Tokens use `_` because CMake interpolates them into a variable name
    (`_zenoh_cond_${cond}`), so `bare-metal` can only ever appear as
    `bare_metal`. One normalisation, used on both sides of every comparison.
    """
    return name.replace("-", "_")


def parse_platform(text: str, dirname: str) -> tuple[str, str]:
    """`(platform name, zenoh owner)` for one `nros-platform.toml`.

    The name rule mirrors `PlatformsTree::load`: the descriptor's first
    `names` entry when it declares one, else the directory name — a platform
    package lives in `nros-platform-<x>/`, so the directory alone would make it
    answer to `nros-platform-zephyr`. The owner defaults to `cargo`, which is
    `CompiledBy::default()`; the absence of the key is a declaration.
    """
    name = dirname
    owner = "cargo"
    section = ""
    for line in text.splitlines():
        stripped = line.split("#", 1)[0]
        m = _SECTION.match(stripped)
        if m:
            section = m.group(1).strip()
            continue
        if not section:
            m = _NAMES.match(stripped)
            if m:
                name = m.group(1)
        elif section == "build.zenoh":
            m = _COMPILED_BY.match(stripped)
            if m:
                owner = m.group(1)
    return name, owner


def load_platforms(repo: Path) -> dict[str, tuple[str, Path]]:
    """platform name → (owner, the manifest that declared it)."""
    out: dict[str, tuple[str, Path]] = {}
    for root in PLATFORM_ROOTS:
        base = repo / root
        if not base.is_dir():
            continue
        for d in sorted(p for p in base.iterdir() if p.is_dir()):
            f = d / "nros-platform.toml"
            if not f.is_file():
                continue
            name, owner = parse_platform(f.read_text(encoding="utf-8"), d.name)
            # First root wins, matching the search path's precedence.
            out.setdefault(name, (owner, f))
    return out


def block(text: str, where: str) -> list[str]:
    """The lines between the two condition markers."""
    try:
        head = text.index(_BEGIN)
        tail = text.index(_END, head)
    except ValueError as e:
        raise LaneError(f"{where}: no {_BEGIN}/{_END} block") from e
    return text[head:tail].splitlines()


def cmake_answers(text: str, where: str = str(CMAKE)) -> dict[str, list[tuple[str, int]]]:
    """token → [(value, if-nesting depth)] for every `set()` in the block.

    Depth is what separates "answered TRUE" from "answered TRUE when a Kconfig
    option is on" — the difference between a platform token and a qualifier.
    """
    out: dict[str, list[tuple[str, int]]] = {}
    depth = 0
    for raw in block(text, where):
        line = raw.split("#", 1)[0].strip()
        m = _CMAKE_SET.match(line)
        if m:
            out.setdefault(m.group(1), []).append((m.group(2).upper(), depth))
        if line.startswith("endif("):
            depth = max(0, depth - 1)
        elif line.startswith("if("):
            depth += 1
    return out


def rust_answers(text: str, where: str = str(RUST_LIB)) -> tuple[dict[str, str], dict[str, str]]:
    """`(token → expression, binding ident → platform it tests)`."""
    arms: dict[str, str] = {}
    binds: dict[str, str] = {}
    for raw in block(text, where):
        line = raw.split("//", 1)[0]
        m = _RS_BIND.match(line)
        if m:
            binds[m.group(1)] = m.group(2)
            continue
        m = _RS_ARM.match(line)
        if m:
            arms[m.group(1)] = m.group(2).strip()
    return arms, binds


def token_platform(token: str, platforms: set[str]) -> str | None:
    """The platform a condition token names, if any.

    Longest match wins and it must end on a `_` boundary, so `zephyr_isotp`
    resolves to `zephyr` while a hypothetical `zephyrisotp` resolves to nothing
    rather than to `zephyr` by accident.
    """
    best: str | None = None
    for p in platforms:
        n = norm(p)
        if token == n or token.startswith(n + "_"):
            if best is None or len(n) > len(norm(best)):
                best = p
    return best


def ownership_problems(
    platforms: dict[str, tuple[str, Path]], west: dict[str, tuple[Path, str]]
) -> list[str]:
    """(1) + (2): every platform has exactly one lane compiling the tree."""
    bad: list[str] = []
    for name, (owner, where) in sorted(platforms.items()):
        if owner not in ("cargo", "platform"):
            bad.append(
                f"{name}: [build.zenoh] compiled_by = \"{owner}\" in "
                f"{where} — the enum has `cargo` and `platform`, nothing else."
            )
            continue
        if owner == "platform" and name not in west:
            bad.append(
                f"{name}: {where} declares compiled_by = \"platform\", so the cargo lane "
                "skips the vendored zenoh-pico tree — and this gate's WEST_LANES names no "
                "lane that compiles it. NOTHING would build zenoh-pico for this platform."
            )
    for name, (lane, _call) in sorted(west.items()):
        if name not in platforms:
            bad.append(
                f"WEST_LANES names platform `{name}` ({lane}), which no nros-platform.toml "
                "declares. Add the platform or drop the lane."
            )
            continue
        owner = platforms[name][0]
        if owner == "cargo":
            bad.append(
                f"{name}: BOTH lanes compile the vendored zenoh-pico tree — {lane} does, and "
                f"{platforms[name][1]} leaves compiled_by at `cargo` so `build_zenoh_pico_"
                "unified` does too. Two independently-resolved `Z_FEATURE_*` sets in one link "
                "is issue 0135, and a Zephyr C/C++ image links with "
                "`-Wl,--allow-multiple-definition`, so it would not even fail loudly."
            )
    return bad


def lane_compile_problems(west: dict[str, tuple[Path, str]], texts: dict[Path, str]) -> list[str]:
    """(3): a west lane reads the manifest AND compiles what it read.

    Comments are stripped first, and that is not a nicety: the mutation that
    found it necessary was `# zephyr_library_sources(${_zenoh_pico_sources})`,
    which leaves the substring in the file while compiling nothing. A gate
    looking for evidence of a CALL must not accept a mention of one.
    """
    bad: list[str] = []
    strip = _sibling().strip_comments
    for name, (lane, call) in sorted(west.items()):
        text = strip(texts[lane], "#")
        if MANIFEST.name not in text:
            bad.append(
                f"{name}: {lane} never names {MANIFEST.name} — it owns the compile for this "
                "platform but reads no shared source list."
            )
        if call not in text:
            bad.append(
                f"{name}: {lane} does not contain `{call}` — it reads the manifest and never "
                "feeds the expansion to a compile, so the tree is read and not built."
            )
    return bad


def guard_problems(text: str, where: str = str(RUST_RUNNER)) -> list[str]:
    """(4): the cargo lane consults `compiled_by` before compiling."""
    lines = text.splitlines()
    calls = [
        i
        for i, ln in enumerate(lines)
        if "build_zenoh_pico_unified(" in ln
        and not ln.lstrip().startswith(("///", "//", "*"))
        and "fn build_zenoh_pico_unified" not in ln
    ]
    if not calls:
        return [
            f"{where}: no call to `build_zenoh_pico_unified(` — this gate can no longer see "
            "where the cargo lane compiles the vendored tree, so it cannot see the guard."
        ]
    bad = []
    for i in calls:
        window = "\n".join(lines[max(0, i - 15) : i])
        if "CompiledBy::Cargo" not in window:
            bad.append(
                f"{where}:{i + 1}: `build_zenoh_pico_unified` is called with no "
                "`CompiledBy::Cargo` test above it. Without that guard `[build.zenoh] "
                "compiled_by` is a comment and cargo compiles the tree for a platform whose "
                "own build system already does."
            )
    return bad


def wiring_problems(
    tokens: set[str],
    platforms: dict[str, tuple[str, Path]],
    cmake: dict[str, list[tuple[str, int]]],
    arms: dict[str, str],
    binds: dict[str, str],
    west: dict[str, tuple[Path, str]],
) -> list[str]:
    """(5): a platform token is answered TRUE-capable by exactly its owner."""
    bad: list[str] = []
    names = set(platforms)
    for token in sorted(tokens):
        plat = token_platform(token, names)
        if plat is None:
            continue
        qualified = token != norm(plat)

        # --- the cargo lane -------------------------------------------------
        expr = arms.get(token)
        if expr is None:
            bad.append(
                f"the cargo lane ({RUST_LIB.relative_to(REPO)}) has no arm for platform token "
                f"`{token}` — `check-zenoh-source-manifest` reports the missing token; this "
                "reports that nothing here can be wired."
            )
        else:
            tests_plat = any(
                ident in re.findall(r"[A-Za-z0-9_]+", expr)
                for ident, p in binds.items()
                if p == plat
            ) or f'platform == "{plat}"' in expr
            if not tests_plat:
                bad.append(
                    f"the cargo lane answers platform token `{token}` with `{expr}`, which "
                    f"does not test the platform it names. It must reference a binding proven "
                    f'here to be `platform == "{plat}"` — a bare constant compiles {plat}\'s '
                    "vendored TUs into every other platform's build (or drops them), and the "
                    "token set stays exactly as it was."
                )

        # --- the west lane --------------------------------------------------
        for lane_plat, (lane, _call) in sorted(west.items()):
            sets = cmake.get(token, [])
            if not sets:
                bad.append(f"{lane.relative_to(REPO)} does not answer platform token `{token}`.")
                continue
            if lane_plat == plat:
                if qualified:
                    if all(depth == 0 for _v, depth in sets):
                        bad.append(
                            f"{lane.relative_to(REPO)} answers qualified token `{token}` "
                            f"unconditionally ({sets[0][0]}). `{token}` narrows `{norm(plat)}` "
                            "by something — a Kconfig option, a link selection — so an "
                            "unconditional answer is not a qualifier at all."
                        )
                elif [v for v, _d in sets] != ["TRUE"] or sets[0][1] != 0:
                    bad.append(
                        f"{lane.relative_to(REPO)} owns the zenoh-pico compile for `{plat}` "
                        f"but answers its own platform token `{token}` "
                        f"{[v for v, _d in sets]} at depth {[d for _v, d in sets]} — it must "
                        "be TRUE, unconditionally, exactly once. This lane IS that platform; "
                        "answering FALSE drops that platform's vendored TUs from every image "
                        "it builds, and no token-set check can see it."
                    )
            elif any(v != "FALSE" for v, _d in sets):
                bad.append(
                    f"{lane.relative_to(REPO)} builds `{lane_plat}` but answers `{plat}`'s "
                    f"token `{token}` {[v for v, _d in sets]}. A lane may only answer its own "
                    "platform's token TRUE."
                )
    return bad


def group_attachment_problems(
    groups: dict[str, str], rows: list[tuple[str, str, str, str]], platforms: set[str]
) -> list[str]:
    """(6): a `system/<dir>/` record's group token names a platform `<dir>` serves."""
    bad: list[str] = []
    for _kind, group, _tree, path in rows:
        parts = path.split("/")
        if len(parts) < 2 or parts[0] != "system":
            continue
        sysdir = parts[1]
        if sysdir in CORE_SYSTEM_DIRS:
            continue
        if sysdir not in SYSTEM_DIR_PLATFORMS:
            bad.append(
                f"`system/{sysdir}` is a zenoh-pico platform tree this gate does not know. "
                "Add it to SYSTEM_DIR_PLATFORMS with the nano-ros platforms it serves (an "
                "empty set means `no platform selects it`) — an unmapped tree cannot be "
                "checked against the token that selects it."
            )
            continue
        token = groups.get(group)
        if token is None:
            continue  # an undeclared group is the sibling gate's report.
        plat = token_platform(token, platforms)
        serves = SYSTEM_DIR_PLATFORMS[sysdir]
        if plat is None or plat not in serves:
            bad.append(
                f"`{path}` is selected by condition token `{token}` (group `{group}`), which "
                f"names {'no platform' if plat is None else '`' + plat + '`'} — but "
                f"`system/{sysdir}` serves {sorted(serves) or 'no nano-ros platform'}. The "
                "record is attached to the wrong platform: every count, every token set and "
                "every coverage check stays green while the file is compiled for the wrong "
                "images."
            )
    return bad


def split_qualifier(name: str, platforms: set[str]) -> tuple[str | None, str]:
    """`(platform, qualifier)` for a group NAME or a condition TOKEN.

    Both are written in the same shape — a platform, optionally narrowed by a
    trailing qualifier — which is what makes checks (8) and (9) possible at all:
    `zephyr_system` and `zephyr_isotp` split the same way `zephyr` and
    `zephyr_isotp` do. A word naming no platform (`core`, `always`) is all
    qualifier and no platform, and the two checks below say so rather than
    pretending it has one.
    """
    plat = token_platform(norm(name), platforms)
    if plat is None:
        return None, norm(name)
    return plat, norm(name)[len(norm(plat)) :].lstrip("_")


def declared_features(groups: dict[str, str], platforms: set[str]) -> set[str]:
    """Every feature a CONDITION narrows a platform by — `{isotp}` today.

    Read off the conditions, never authored here: a feature exists because some
    group is gated on it, so a manifest that gains one teaches this gate about
    it in the same edit.
    """
    out = set()
    for cond in groups.values():
        plat, qual = split_qualifier(cond, platforms)
        if plat is not None and qual:
            out.add(qual)
    return out


def _names_feature(path: str, feature: str) -> bool:
    """Does this path name `feature` as a whole word?

    Segment- and extension-aware, so `isotp` matches `system/zephyr/isotp.c` and
    `link/unicast/isotp.c` but not a file that merely contains the letters.
    """
    p = norm(path.lower())
    return re.search(rf"(^|[/_.]){re.escape(norm(feature.lower()))}([/_.]|$)", p) is not None


def group_naming_problems(groups: dict[str, str], platforms: set[str]) -> list[str]:
    """(8): a group's NAME and its CONDITION must agree, on platform and feature.

    THE HOLE THIS CLOSES, found by review after the first version shipped.
    Swapping two groups' conditions —

        -group zephyr_system  zephyr          +group zephyr_system  zephyr_isotp
        -group zephyr_isotp   zephyr_isotp    +group zephyr_isotp   zephyr

    — leaves every token declared, every token used, every token answered by
    both lanes, and every count identical, so `check-zenoh-source-manifest` is
    green; and both tokens name `zephyr`, so check (6) is green too. What it
    does is compile `system/zephyr/network.c` ONLY when ISO-TP is enabled and
    `system/zephyr/isotp.c` on EVERY Zephyr build. The manifest's own comment
    says why the first half is serious: network.c must come from zenoh-pico's
    Zephyr backend rather than the alias TU, because the two see different
    socket and endpoint layouts (phase 129 / phase 160.C), so a non-ISO-TP
    Zephyr image would lose it and fail at session open.

    THE RULE, and it is a reason rather than a hash of today's layout: **the
    group name is what a reader uses to understand a record, so a group called
    `zephyr_system` gated on an ISO-TP feature is a lie in the file itself.**
    Made precise in two halves, each of which fires on one half of that swap:

    - the platform in the name must be the platform in the condition (and a
      group naming no platform must be gated on a condition naming none);
    - if the condition narrows the platform by a FEATURE, the name must carry
      that same feature; if it does not narrow at all, the name must not claim
      a feature some other condition uses.
    """
    bad: list[str] = []
    features = declared_features(groups, platforms)
    for group, cond in sorted(groups.items()):
        gp, gq = split_qualifier(group, platforms)
        cp, cq = split_qualifier(cond, platforms)
        if gp != cp:
            bad.append(
                f"group `{group}` names platform "
                f"{'`' + gp + '`' if gp else 'none'} but is gated on `{cond}`, which names "
                f"{'`' + cp + '`' if cp else 'none'}. A group's name is what a reader uses to "
                "understand its records; a name that disagrees with the condition is a lie "
                "in the file itself."
            )
            continue
        if gp is None:
            continue  # `core` / `always` — no platform, nothing further to say.
        if cq:
            if gq != cq:
                bad.append(
                    f"group `{group}` is gated on `{cond}`, which narrows `{gp}` by feature "
                    f"`{cq}` — but the group is named for `{gq or 'the bare platform'}`. Name "
                    "and condition must agree on the feature: a `_{0}` group compiled only "
                    "when a DIFFERENT feature is on reads correct and builds wrong, and no "
                    "token-set or count check can see it.".format(gq or "")
                )
        elif gq in features:
            bad.append(
                f"group `{group}` is named for feature `{gq}` but is gated on the bare "
                f"platform token `{cond}`, so it compiles on EVERY {gp} build. `{gq}` is a "
                "feature some other condition narrows by, so the name promises a narrowing "
                "the condition does not make."
            )
    return bad


def feature_record_problems(
    groups: dict[str, str],
    rows: list[tuple[str, str, str, str]],
    platforms: set[str],
) -> list[str]:
    """(9): the MIRROR of (8) — a record moved between two correctly-named groups.

    Check (8) holds a group's name to its condition; that leaves the swap's
    other spelling untouched, in which the groups are named and gated perfectly
    and the two RECORDS trade places. Same effect on the build, same silence
    from every count.

    THE RULE: **a feature-gated group compiles the TUs that IMPLEMENT that
    feature, and a vendored tree names its TUs after what they implement** —
    `system/zephyr/isotp.c` is the ISO-TP binding, `system/zephyr/network.c` is
    the platform's socket backend. So a record in a `<platform>_<feature>` group
    must name that feature in its path, and a record in a base-platform group
    must not name a feature some condition narrows by.

    It is a property of upstream's layout rather than a law, so it has a stated
    escape hatch: a TU whose name genuinely does not carry its feature goes in
    `FEATURE_TU_EXCEPTIONS` with the reason. Empty today — no such file exists
    in this manifest, and an empty table is the honest statement of that.
    """
    bad: list[str] = []
    features = declared_features(groups, platforms)
    for _kind, group, _tree, path in rows:
        cond = groups.get(group)
        if cond is None:
            continue  # an undeclared group is the sibling gate's report.
        plat, feature = split_qualifier(cond, platforms)
        if plat is None:
            continue  # `always` — the core is a rule, not a per-file decision.
        if path in FEATURE_TU_EXCEPTIONS:
            continue
        if feature:
            if not _names_feature(path, feature):
                bad.append(
                    f"`{path}` sits in group `{group}`, which compiles only when `{feature}` "
                    f"is enabled on `{plat}` — but the path does not name `{feature}`. A "
                    "feature-gated group compiles the TUs that IMPLEMENT that feature; a "
                    "record that does not is either in the wrong group or needs a "
                    "FEATURE_TU_EXCEPTIONS entry saying why its name does not carry it."
                )
        else:
            for other in sorted(features):
                if _names_feature(path, other):
                    bad.append(
                        f"`{path}` names feature `{other}` but sits in group `{group}`, which "
                        f"is gated on the bare `{plat}` token — so it compiles on EVERY "
                        f"{plat} build, whether `{other}` was asked for or not. It belongs in "
                        f"the `{other}` group."
                    )
    return bad


def third_compile_problems(
    hits: dict[str, str], west: dict[str, tuple[Path, str]]
) -> list[str]:
    """(7): nobody compiles the vendored tree except a declared lane.

    `hits` is `relative path → the first non-comment line naming the tree's
    source root`. The two lanes and the manifest itself are expected; anything
    else is a copy of the selection that no gate was watching — which is what
    `scripts/qemu/build-zenoh-pico.sh` turned out to be (issue 1096), with its
    own nine-directory loop, its own config header and its own slot defaults,
    while `check-zenoh-source-manifest`'s "neither lane names a vendored path"
    was true of the two lanes and false of the repository.
    """
    expected = {str(p.relative_to(REPO)) for p, _c in west.values()}
    expected |= {
        str(MANIFEST.relative_to(REPO)),
        str(RUST_LIB.relative_to(REPO)),
        str(RUST_RUNNER.relative_to(REPO)),
        "scripts/check-zenoh-lane-ownership.py",
        "scripts/check-zenoh-source-manifest.py",
    }
    bad = []
    for path in sorted(hits):
        if path in expected or path in KNOWN_UNSHARED_COMPILES:
            continue
        bad.append(
            f"{path} names the vendored zenoh-pico SOURCE tree (`{hits[path]}`) and is not a "
            "declared lane. One vendored tree gets one source list: read "
            f"{MANIFEST.relative_to(REPO)}, or — if this really is a third compiler that "
            "cannot — add it to KNOWN_UNSHARED_COMPILES with a TRACKED ISSUE ID, never a "
            "bare reason."
        )
    for path, why in sorted(KNOWN_UNSHARED_COMPILES.items()):
        if not re.fullmatch(r"issue \d{4}", why):
            bad.append(
                f"KNOWN_UNSHARED_COMPILES[{path}] = {why!r} — an entry must name a tracked "
                "issue (`issue NNNN`), because an untracked exemption is the silence this "
                "check exists to break."
            )
        elif path not in hits:
            bad.append(
                f"KNOWN_UNSHARED_COMPILES names {path}, which no longer compiles the vendored "
                f"tree. If {why} is fixed, delete the entry and close it."
            )
    return bad


def scan_tree_compilers(repo: Path) -> dict[str, str]:
    """Every tracked build-ish file naming the vendored tree's source root.

    Tracked files only (`git ls-files`), because an untracked scratch script is
    not something the repository ships. Comment lines are excluded — prose
    about the tree is not a compile of it, and a dozen files legitimately
    mention `zenoh-pico/src/...` in a doc comment.
    """
    import subprocess

    out = subprocess.run(
        ["git", "-C", str(repo), "ls-files", "--", "*.sh", "*.py", "*.cmake", "*.mk",
         "CMakeLists.txt", "*/CMakeLists.txt", "Makefile", "*/Makefile", "*.rs", "*.txt"],
        capture_output=True, text=True, check=True,
    )
    hits: dict[str, str] = {}
    for rel in out.stdout.split():
        if rel.startswith(("third-party/", "packages/rmw/zenoh/zpico-sys/zenoh-pico/")):
            continue
        f = repo / rel
        try:
            text = f.read_text(encoding="utf-8", errors="replace")
        except OSError:  # pragma: no cover
            continue
        for line in text.splitlines():
            code = line.split("#", 1)[0].split("//", 1)[0]
            if code.lstrip().startswith(("*", "///")):
                continue
            if _TREE_SRC.search(code):
                hits[rel] = code.strip()
                break
    return hits


def self_test() -> None:
    """Runs on the NORMAL path — a negative control nobody runs decays into a
    comment (`check-gate-selftests`)."""
    # --- platform parsing ---------------------------------------------------
    assert parse_platform("names = [\"zephyr\"]\n[build.zenoh]\ncompiled_by = \"platform\"\n",
                          "nros-platform-zephyr") == ("zephyr", "platform")
    # no key ⇒ cargo, which is `CompiledBy::default()`; and the directory name
    # is the fallback the loader uses.
    assert parse_platform("[build.zenoh]\ndefines = []\n", "bare-metal") == ("bare-metal", "cargo")
    # `compiled_by` outside `[build.zenoh]` is a different field and must not be
    # read as this one.
    assert parse_platform(
        'names = ["posix"]\n[build.other]\ncompiled_by = "platform"\n', "d"
    ) == ("posix", "cargo")

    # --- token → platform ---------------------------------------------------
    plats = {"zephyr", "posix", "bare-metal", "freertos", "freertos-lwip"}
    assert token_platform("zephyr", plats) == "zephyr"
    assert token_platform("zephyr_isotp", plats) == "zephyr"
    assert token_platform("bare_metal", plats) == "bare-metal"
    assert token_platform("freertos_lwip_x", plats) == "freertos-lwip"  # longest wins
    assert token_platform("always", plats) is None
    assert token_platform("zephyrisotp", plats) is None  # boundary, not prefix

    # --- lane answer parsing ------------------------------------------------
    cm = (
        f"# {_BEGIN}\n"
        "set(_zenoh_cond_always TRUE)\n"
        "set(_zenoh_cond_zephyr TRUE)\n"
        "if(CONFIG_NROS_ZENOH_LINK_ISOTP)\n"
        "    set(_zenoh_cond_zephyr_isotp TRUE)\n"
        "else()\n"
        "    set(_zenoh_cond_zephyr_isotp FALSE)\n"
        "endif()\n"
        f"# {_END}\n"
    )
    ans = cmake_answers(cm, "T")
    assert ans["zephyr"] == [("TRUE", 0)], ans
    assert ans["zephyr_isotp"] == [("TRUE", 1), ("FALSE", 1)], ans
    rs = (
        f"// {_BEGIN}\n"
        '    let is_zephyr = platform == "zephyr";\n'
        '        "always" => true,\n'
        '        "zephyr" => is_zephyr,\n'
        '        "zephyr_isotp" => is_zephyr && link.isotp,\n'
        f"// {_END}\n"
    )
    arms, binds = rust_answers(rs, "T")
    assert arms == {
        "always": "true",
        "zephyr": "is_zephyr",
        "zephyr_isotp": "is_zephyr && link.isotp",
    }, arms
    assert binds == {"is_zephyr": "zephyr"}, binds
    for missing in ("nothing here", f"only {_BEGIN} and no end"):
        try:
            block(missing, "T")
        except LaneError:
            pass
        else:  # pragma: no cover
            raise AssertionError("missing-marker block accepted")

    # --- ownership ----------------------------------------------------------
    p_ok = {"zephyr": ("platform", Path("z.toml")), "posix": ("cargo", Path("p.toml"))}
    west_ok = {"zephyr": (CMAKE, "call")}
    assert ownership_problems(p_ok, west_ok) == []
    # BOTH lanes compile it — the issue-0135 shape, and the one a
    # `--allow-multiple-definition` link would not report.
    msgs = ownership_problems({"zephyr": ("cargo", Path("z.toml"))}, west_ok)
    assert any("BOTH lanes compile" in m for m in msgs), msgs
    # NEITHER lane compiles it.
    msgs = ownership_problems({"zephyr": ("platform", Path("z.toml"))}, {})
    assert any("NOTHING would build" in m for m in msgs), msgs
    # a lane for a platform that does not exist.
    msgs = ownership_problems(p_ok, {"nope": (CMAKE, "call")})
    assert any("no nros-platform.toml declares" in m for m in msgs), msgs
    msgs = ownership_problems({"zephyr": ("west", Path("z.toml"))}, {})
    assert any("nothing else" in m for m in msgs), msgs

    # --- the west lane builds what it reads ---------------------------------
    assert lane_compile_problems(west_ok, {CMAKE: f"{MANIFEST.name}\ncall\n"}) == []
    msgs = lane_compile_problems(west_ok, {CMAKE: f"{MANIFEST.name}\n"})
    assert any("reads the manifest and never" in m for m in msgs), msgs
    msgs = lane_compile_problems(west_ok, {CMAKE: "call\n"})
    assert any("reads no shared source list" in m for m in msgs), msgs
    # THE MUTATION THAT FOUND THE COMMENT-STRIP NECESSARY: commenting the call
    # out leaves its text in the file. A gate that greps for a call must not
    # accept a mention of one.
    msgs = lane_compile_problems(west_ok, {CMAKE: f"{MANIFEST.name}\n# call\n"})
    assert any("reads the manifest and never" in m for m in msgs), msgs

    # --- the cargo guard ----------------------------------------------------
    guarded = (
        "if resolved.compiled_by == manifest::CompiledBy::Cargo {\n"
        "    build_zenoh_pico_unified(\n        &resolved,\n    );\n}\n"
    )
    assert guard_problems(guarded, "T") == []
    msgs = guard_problems("    build_zenoh_pico_unified(\n        &resolved,\n    );\n", "T")
    assert any("CompiledBy::Cargo" in m for m in msgs), msgs
    # a mention of the enum in a COMMENT above the call is not a guard... but a
    # 15-line window cannot tell those apart, and pretending otherwise would be
    # a check that reports more than it establishes. What it does establish:
    # deleting the guard entirely is caught.
    assert guard_problems(
        "if resolved.compiled_by == manifest::CompiledBy::Cargo {\n"
        + "\n" * 20
        + "    build_zenoh_pico_unified(&r);\n}\n",
        "T",
    ), "a guard 20 lines up is out of the window and must report"
    msgs = guard_problems("// nothing calls it\n", "T")
    assert any("no call to" in m for m in msgs), msgs

    # --- THE CROSS-WIRE CHECKS ----------------------------------------------
    tokens = {"always", "zephyr", "zephyr_isotp"}
    plats2 = {"zephyr": ("platform", Path("z")), "posix": ("cargo", Path("p"))}
    good_cm, (good_arms, good_binds) = cmake_answers(cm, "T"), rust_answers(rs, "T")
    assert wiring_problems(tokens, plats2, good_cm, good_arms, good_binds, west_ok) == []

    # (a) the cmake lane answers its OWN platform's token FALSE. Shape valid:
    #     the token set is unchanged, so `check-zenoh-source-manifest` is green,
    #     and every Zephyr image loses `system/zephyr/network.c` — phase 160.C.
    flipped = dict(good_cm, zephyr=[("FALSE", 0)])
    msgs = wiring_problems(tokens, plats2, flipped, good_arms, good_binds, west_ok)
    assert any("answering FALSE drops that platform's vendored TUs" in m for m in msgs), msgs
    # (b) ...and answering it conditionally is the same defect, softer.
    msgs = wiring_problems(
        tokens, plats2, dict(good_cm, zephyr=[("TRUE", 1)]), good_arms, good_binds, west_ok
    )
    assert any("unconditionally, exactly once" in m for m in msgs), msgs
    # (c) the cargo lane answers a platform token with a bare constant: the
    #     Zephyr-only TU is compiled into every other platform's build.
    msgs = wiring_problems(
        tokens, plats2, good_cm, dict(good_arms, zephyr="true"), good_binds, west_ok
    )
    assert any("does not test the platform it names" in m for m in msgs), msgs
    # (d) THE PURE CROSS-WIRE: the arm is a platform test, and it tests the
    #     WRONG platform. Every shape is intact — an identifier, a binding, a
    #     comparison — and the wire goes to the wrong pin.
    msgs = wiring_problems(
        tokens,
        plats2,
        good_cm,
        dict(good_arms, zephyr="is_posix"),
        {"is_posix": "posix"},
        west_ok,
    )
    assert any("does not test the platform it names" in m for m in msgs), msgs
    # (e) ...and the same crossing one level down: the binding NAMED for one
    #     platform is bound to another. `is_zephyr` still appears in the arm.
    msgs = wiring_problems(
        tokens, plats2, good_cm, good_arms, {"is_zephyr": "posix"}, west_ok
    )
    assert any("does not test the platform it names" in m for m in msgs), msgs
    # (f) a qualifier that qualifies nothing.
    msgs = wiring_problems(
        tokens, plats2, dict(good_cm, zephyr_isotp=[("TRUE", 0)]), good_arms, good_binds, west_ok
    )
    assert any("is not a qualifier at all" in m for m in msgs), msgs
    # (g) a lane answering ANOTHER platform's token TRUE.
    msgs = wiring_problems(
        {"posix"}, plats2, {"posix": [("TRUE", 0)]}, {"posix": "is_posix"},
        {"is_posix": "posix"}, west_ok
    )
    assert any("may only answer its own" in m for m in msgs), msgs

    # --- THE SWAP, and its mirror -------------------------------------------
    plats3 = {"zephyr", "posix", "freertos"}
    assert split_qualifier("zephyr_system", plats3) == ("zephyr", "system")
    assert split_qualifier("zephyr", plats3) == ("zephyr", "")
    assert split_qualifier("zephyr_isotp", plats3) == ("zephyr", "isotp")
    assert split_qualifier("core", plats3) == (None, "core")
    assert split_qualifier("always", plats3) == (None, "always")
    assert _names_feature("system/zephyr/isotp.c", "isotp")
    assert _names_feature("link/unicast/isotp.c", "isotp")
    assert not _names_feature("system/zephyr/network.c", "isotp")
    assert not _names_feature("system/zephyr/isotpish.c", "isotp")  # whole word

    good_groups = {"core": "always", "zephyr_system": "zephyr", "zephyr_isotp": "zephyr_isotp"}
    good_rows = [
        ("dir", "core", "zenoh_pico", "api"),
        ("src", "zephyr_system", "zenoh_pico", "system/zephyr/network.c"),
        ("src", "zephyr_isotp", "zenoh_pico", "system/zephyr/isotp.c"),
    ]
    assert declared_features(good_groups, plats3) == {"isotp"}
    assert group_naming_problems(good_groups, plats3) == []
    assert feature_record_problems(good_groups, good_rows, plats3) == []

    # THE HOLE REVIEW FOUND: the two groups SWAP conditions. Every token is
    # still declared, still used, still answered by both lanes; every count is
    # identical; both tokens name `zephyr`, so checks (5) and (6) are green.
    # `system/zephyr/network.c` would compile only under ISO-TP — phase 160.C.
    swapped = {"core": "always", "zephyr_system": "zephyr_isotp", "zephyr_isotp": "zephyr"}
    msgs = group_naming_problems(swapped, plats3)
    assert any("but the group is named for `system`" in m for m in msgs), msgs
    assert any("promises a narrowing the condition does not make" in m for m in msgs), msgs
    # ...and check (9) reports it a second time, from the records' side.
    msgs = feature_record_problems(swapped, good_rows, plats3)
    assert len(msgs) == 2, msgs

    # THE MIRROR: groups named and gated perfectly, the two RECORDS trade
    # places. (8) is silent by construction — this is what (9) is for.
    mirror_rows = [
        ("dir", "core", "zenoh_pico", "api"),
        ("src", "zephyr_isotp", "zenoh_pico", "system/zephyr/network.c"),
        ("src", "zephyr_system", "zenoh_pico", "system/zephyr/isotp.c"),
    ]
    assert group_naming_problems(good_groups, plats3) == []
    msgs = feature_record_problems(good_groups, mirror_rows, plats3)
    assert any("does not name `isotp`" in m for m in msgs), msgs
    assert any("compiles on EVERY zephyr build" in m for m in msgs), msgs

    # the platform half of (8): the name and the condition name DIFFERENT
    # platforms, or one names none.
    msgs = group_naming_problems({"freertos_system": "zephyr"}, plats3)
    assert any("names platform `freertos` but is gated on `zephyr`" in m for m in msgs), msgs
    msgs = group_naming_problems({"core": "zephyr"}, plats3)
    assert any("names platform none but is gated on `zephyr`" in m for m in msgs), msgs
    msgs = group_naming_problems({"zephyr_system": "always"}, plats3)
    assert any("names platform `zephyr` but is gated on `always`" in m for m in msgs), msgs
    # a feature nothing is gated on is not a feature, so a group may be named
    # for it: `zephyr_system` is exactly that case and must stay legal.
    assert group_naming_problems({"zephyr_system": "zephyr"}, plats3) == []

    # (9)'s escape hatch works, and only for the record it names.
    FEATURE_TU_EXCEPTIONS["system/zephyr/network.c"] = "test"
    try:
        msgs = feature_record_problems(good_groups, mirror_rows, plats3)
        assert len(msgs) == 1, msgs
        assert "isotp.c" in msgs[0], msgs
    finally:
        FEATURE_TU_EXCEPTIONS.pop("system/zephyr/network.c")

    # --- nobody else compiles the tree --------------------------------------
    assert _TREE_SRC.search('zenoh-pico/src/api/api.c')
    assert _TREE_SRC.search('set(_x "${ZENOH_PICO_DIR}/src")')
    assert _TREE_SRC.search("let zenoh_pico_src = d;")
    assert _TREE_SRC.search('zenoh_pico_src.join("src")')
    # a CONSUMER of the built library is not a rebuild of it...
    assert not _TREE_SRC.search("zenoh-pico/include/zenoh-pico.h")
    assert not _TREE_SRC.search('#include <zenoh-pico.h>')
    # ...but an identifier NAMED for the source root counts however it is used:
    # a file holding a `zenoh_pico_src` is handling the tree, and which
    # subdirectory it reaches on any one line is not the question.
    assert _TREE_SRC.search('zenoh_pico_src.join("include")')

    west2 = {"zephyr": (CMAKE, "call")}
    base_hits = {str(CMAKE.relative_to(REPO)): "x", str(RUST_LIB.relative_to(REPO)): "y"}
    known = dict(KNOWN_UNSHARED_COMPILES)
    try:
        # the ledger's own entries must be present, or the ledger is stale.
        KNOWN_UNSHARED_COMPILES.clear()
        assert third_compile_problems(base_hits, west2) == []
        # THE REGRESSION: a fourth compiler lands.
        msgs = third_compile_problems(dict(base_hits, **{"tools/build-it.sh": "z"}), west2)
        assert any("is not a declared lane" in m for m in msgs), msgs
        # ...and it cannot be waved through with a bare reason.
        KNOWN_UNSHARED_COMPILES["tools/build-it.sh"] = "legacy, fine for now"
        msgs = third_compile_problems(dict(base_hits, **{"tools/build-it.sh": "z"}), west2)
        assert any("must name a tracked" in m for m in msgs), msgs
        # ...only with a tracked id.
        KNOWN_UNSHARED_COMPILES["tools/build-it.sh"] = "issue 1096"
        assert third_compile_problems(dict(base_hits, **{"tools/build-it.sh": "z"}), west2) == []
        # a ledger entry whose file stopped compiling the tree is DEBT PAID and
        # must be deleted — a stale exemption is the next silence.
        msgs = third_compile_problems(base_hits, west2)
        assert any("no longer compiles" in m for m in msgs), msgs
    finally:
        KNOWN_UNSHARED_COMPILES.clear()
        KNOWN_UNSHARED_COMPILES.update(known)

    # --- group attachment ---------------------------------------------------
    names2 = {"zephyr", "posix", "freertos"}
    ok_rows = [("src", "zs", "zenoh_pico", "system/zephyr/network.c")]
    assert group_attachment_problems({"zs": "zephyr"}, ok_rows, names2) == []
    # THE SHAPE-VALID CROSSING: the record moves to a group whose token names a
    # DIFFERENT platform. Group count, record count, token set, tree coverage
    # and both lanes' answers are all untouched.
    msgs = group_attachment_problems({"zs": "freertos"}, ok_rows, names2)
    assert any("attached to the wrong platform" in m for m in msgs), msgs
    msgs = group_attachment_problems({"zs": "always"}, ok_rows, names2)
    assert any("names no platform" in m for m in msgs), msgs
    # an upstream tree nobody mapped.
    msgs = group_attachment_problems(
        {"zs": "zephyr"}, [("dir", "zs", "zenoh_pico", "system/newos")], names2
    )
    assert any("does not know" in m for m in msgs), msgs
    # core paths are not this check's business.
    assert group_attachment_problems({"c": "always"}, [("dir", "c", "zenoh_pico", "api")],
                                     names2) == []
    assert group_attachment_problems(
        {"c": "always"}, [("dir", "c", "zenoh_pico", "system/common")], names2
    ) == []


def main() -> int:
    self_test()

    bad: list[str] = []
    sibling = _sibling()
    try:
        groups, rows = sibling.parse_manifest(MANIFEST.read_text(encoding="utf-8"), str(MANIFEST))
        texts = {p: p.read_text(encoding="utf-8") for p in {CMAKE, RUST_LIB, RUST_RUNNER}}
        cmake = cmake_answers(texts[CMAKE])
        arms, binds = rust_answers(texts[RUST_LIB])
    except (LaneError, OSError, sibling.ManifestError) as e:
        print(f"check-zenoh-lane-ownership: {e}", file=sys.stderr)
        return 1

    platforms = load_platforms(REPO)
    if not platforms:
        print(
            "check-zenoh-lane-ownership: no nros-platform.toml under "
            f"{', '.join(PLATFORM_ROOTS)} — an empty platform set makes every check here "
            "vacuously green",
            file=sys.stderr,
        )
        return 1

    bad += ownership_problems(platforms, WEST_LANES)
    bad += lane_compile_problems(WEST_LANES, texts)
    bad += guard_problems(texts[RUST_RUNNER])
    bad += wiring_problems(set(groups.values()), platforms, cmake, arms, binds, WEST_LANES)
    bad += group_attachment_problems(groups, rows, set(platforms))
    bad += third_compile_problems(scan_tree_compilers(REPO), WEST_LANES)
    bad += group_naming_problems(groups, set(platforms))
    bad += feature_record_problems(groups, rows, set(platforms))

    if bad:
        print("check-zenoh-lane-ownership: FAIL", file=sys.stderr)
        for m in bad:
            print(f"  - {m}", file=sys.stderr)
        return 1

    owned = sorted(n for n, (o, _w) in platforms.items() if o == "platform")
    cargo = sorted(n for n, (o, _w) in platforms.items() if o == "cargo")
    print(
        "check-zenoh-lane-ownership: OK — "
        f"cargo compiles the vendored tree for {cargo}, "
        f"{sorted(WEST_LANES)} for {owned}; no platform is in both, none in neither"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
