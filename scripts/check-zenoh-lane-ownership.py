#!/usr/bin/env python3
"""One vendored tree, three compilers, ONE owner per platform — phase-420 W9.

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
   the two lanes it knows and false of the repository. The ledger is EMPTY now:
   that script reads the manifest and is a declared lane in the sibling, whose
   `LANE_SPECS` this check reads rather than restating — one list of who the
   lanes are, because two lists is how the third one hid.

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
10. **The readiness lane answers as the lane it PROXIES for.** (5) holds the two
    real lanes to the platform each owns; `scripts/qemu/build-zenoh-pico.sh`
    owns none — it cross-compiles the tree for `bare-metal` into a standalone
    `.a` that no image links, to answer "does the bare-metal build still
    compile?". So it must answer every condition token exactly as the cargo lane
    does for that platform: `always` true, every platform-named token false. A
    `zephyr) return 0 ;;` there leaves every token declared, used, answered and
    counted — sibling green, (5) green, since neither looks at this lane's
    ANSWERS — while the proxy compiles a different source set than the build it
    stands for. The platform is read back out of the script (its shim include
    path) and its owner out of `nros-platform.toml`, so the two facts this rests
    on cannot silently stop describing it.
11. **The readiness lane's config header still mirrors the generator.** This is
    the one place here that ACCEPTS a mirror, on a stated argument:
    `config_header()` resolves its inputs from cargo build-script environment
    (link features, the RFC-0049 wire ladder, `$DOTCONFIG` knobs) that a
    standalone shell script cannot supply, so restating the RESOLUTION in shell
    would be a fourth statement of it rather than one fewer. What the mirror
    lacked was the drift-detector, which is the shape of every duplication this
    area has paid for. Compared by macro NAME for every unconditional `#define`
    the generator emits, and by VALUE wherever the generator's value is a
    literal (a `const`/`let` binding in the same function is resolved, so
    `Z_TRANSPORT_LEASE {}` / `const … = 60_000` compares as `60000`). Measured
    when it was written: SIX macros missing from the mirror and FOUR values
    disagreeing — `Z_TRANSPORT_LEASE` still 10000 after issue 0906 moved it,
    plus `Z_FEATURE_MATCHING`, `Z_FEATURE_LOCAL_SUBSCRIBER`,
    `Z_FEATURE_LOCAL_QUERYABLE` and `Z_FEATURE_BATCHING`, which decide which
    code compiles. 45 of the readiness build's 130 objects changed when they
    were brought back into line.
12. **The readiness lane passes the shim's slot counts, and the right ones.**
    The last of issue 1096's four copies, and the same trade as (11): the
    macro⇄field mapping is read out of `ShimConfig::defines()` and the defaults
    out of `shim_config_from_env()`, so the script's `-DZPICO_*` flags are
    compared rather than trusted. An OMISSION is a failure here, and
    `defines()`' own comment says why: the C shim `#define`s a fallback for
    every one of these, so a missing `-D` is not "no opinion", it is the shim's
    default winning on one lane only — issue 0460's shape. It was live: the real
    build passes ten of these on every target and the readiness script passed
    seven (`ZPICO_MAX_SESSIONS`, `ZPICO_READ_TASK_PRIORITY`,
    `ZPICO_LEASE_TASK_PRIORITY` were missing; their values happen to equal
    today's C fallbacks, so adding them changed no preprocessing — what changed
    is that a moved default is now a red gate instead of a silent divergence).

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
# The bare-metal QEMU readiness lane (issue 1096). Declared in the sibling
# gate's LANE_SPECS; named here because checks (10) and (11) are about this
# file specifically.
QEMU_SH = REPO / "scripts/qemu/build-zenoh-pico.sh"

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
#
# EMPTY, and that is the finding rather than a default: its one entry was
# `scripts/qemu/build-zenoh-pico.sh` (issue 1096), the third compiler check (7)
# found on its first run. That script reads `zenoh-sources.txt` now and is a
# declared lane in the sibling gate, so the entry went with the debt — which is
# the direction check (7)'s own "no longer compiles the tree" arm asks for.
KNOWN_UNSHARED_COMPILES: dict[str, str] = {}

# The platform `scripts/qemu/build-zenoh-pico.sh` cross-compiles the vendored
# tree for. It is a PROXY for the cargo lane's bare-metal build — a standalone
# `.a` no image links, built to answer "does the bare-metal zenoh-pico compile?"
# — so it must answer the manifest's condition tokens exactly as the cargo lane
# does for this platform. Checked, not assumed: check (10) fails if the script
# does not reach for this platform's shim headers, and if the platform's own
# `nros-platform.toml` hands its zenoh compile to someone other than cargo.
READINESS_PLATFORM = "bare-metal"

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
    # WHO THE LANES ARE IS READ FROM THE SIBLING, not restated here. Both gates
    # would otherwise name the same files, and the second copy is what let the
    # third compiler hide: that gate's path check was scoped to the lanes it
    # knew, this gate's exemption list named the one it did not, and neither
    # list could tell you the other was incomplete.
    expected = {str(p.relative_to(REPO)) for p, _c in west.values()}
    expected |= {str(p.relative_to(REPO)) for p in _sibling().LANE_SPECS}
    expected |= {
        str(MANIFEST.relative_to(REPO)),
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


# --- (10) + (11): the bare-metal readiness lane ------------------------------
#
# `scripts/qemu/build-zenoh-pico.sh` is the third compiler of the vendored tree
# (issue 1096). Since it now READS the shared manifest, `check-zenoh-source-
# manifest` holds it to the same shape rules as the other two — it names no
# vendored path, it answers exactly the manifest's token set, it reads the file.
# What that gate cannot say is which WAY it answers, and what it does with the
# values the cargo lane derives. Those are this gate's business, exactly as (5)
# is for the other two lanes.

# `always) return 0 ;;` — one arm of the shell lane's `zenoh_condition`. Shell
# truth is INVERTED (0 is success), which is the cross-wire this pattern has to
# read correctly: a `return 1` on `always` would compile nothing and a `return
# 0` on `zephyr` would drag Zephyr's vendored TUs into a Cortex-M3 build.
_SH_ANSWER = re.compile(r"^\s*([A-Za-z0-9_]+)\)\s*return\s+([01])\s*;;")


def sh_answers(text: str, where: str = str(QEMU_SH)) -> dict[str, bool]:
    """token → the boolean the readiness lane supplies, from its marker block."""
    out: dict[str, bool] = {}
    for raw in block(text, where):
        line = raw.split("#", 1)[0]
        m = _SH_ANSWER.match(line)
        if m:
            out[m.group(1)] = m.group(2) == "0"
    return out


def readiness_lane_problems(
    tokens: set[str],
    platforms: dict[str, tuple[str, Path]],
    answers: dict[str, bool],
    sh_text: str,
) -> list[str]:
    """(10): the readiness lane answers as the lane it proxies for.

    It is a PROXY: it compiles the vendored tree for `bare-metal` into a
    standalone `.a` that no image links, to answer "does the bare-metal build
    still compile?". A proxy that selects a different source set than the build
    it stands for answers a question nobody asked — and does it in green.

    Two of the three statements are checked rather than assumed. The platform it
    builds for is read back out of the script (its shim include path), so
    `READINESS_PLATFORM` cannot quietly stop describing it; and that platform's
    zenoh owner must be `cargo`, because if a west lane ever took it over, the
    build this proxies for would be a different one.
    """
    bad: list[str] = []
    strip = _sibling().strip_comments
    body = strip(sh_text, "#")
    plat = READINESS_PLATFORM
    if plat not in platforms:
        return [
            f"READINESS_PLATFORM = `{plat}`, which no nros-platform.toml declares. "
            f"{QEMU_SH.relative_to(REPO)} cross-compiles the vendored tree for it, so it "
            "is not a name this gate may invent."
        ]
    if f"$PLATFORM_DIR/{plat}" not in body:
        bad.append(
            f"{QEMU_SH.relative_to(REPO)} does not reach for `$PLATFORM_DIR/{plat}` — "
            f"READINESS_PLATFORM says it builds for `{plat}` and the script says otherwise. "
            "One of the two moved; this check is wired to the wrong platform until they agree."
        )
    owner = platforms[plat][0]
    if owner != "cargo":
        bad.append(
            f"{plat}: [build.zenoh] compiled_by = \"{owner}\" in {platforms[plat][1]}, so the "
            f"cargo lane does not compile the vendored tree for it — but "
            f"{QEMU_SH.relative_to(REPO)} is a PROXY for that compile. It now stands for a "
            "build nobody performs."
        )
    for token in sorted(tokens):
        got = answers.get(token)
        if got is None:
            # The missing-token report belongs to the sibling; this would be a
            # second voice on the same defect.
            continue
        named = token_platform(token, set(platforms))
        if token == _sibling().UNCONDITIONAL:
            want = True
            why = "the core, which every lane on every platform compiles"
        elif named is None:
            bad.append(
                f"condition token `{token}` names no platform and is not "
                f"`{_sibling().UNCONDITIONAL}` — this check cannot decide how the "
                f"{plat} readiness lane should answer it. Extend "
                "`readiness_lane_problems` with the rule rather than leaving the answer "
                "unchecked."
            )
            continue
        elif named == plat:
            want = True
            why = f"a token naming `{plat}`, the platform this lane builds for"
        else:
            want = False
            why = f"a token naming `{named}`, which is not `{plat}`"
        if got is not want:
            bad.append(
                f"{QEMU_SH.relative_to(REPO)} answers condition token `{token}` "
                f"{str(got).upper()}, but it is {why}, so it must be {str(want).upper()}. "
                "The token SET is identical either way, which is why the sibling gate is "
                "green: this is the readiness build selecting different sources from the "
                "build it stands for."
            )
    return bad


# The Rust function whose output the readiness lane mirrors, and the shell
# heredoc that mirrors it.
_CONFIG_HEADER_FN = "pub fn config_header("
_SH_HEREDOC_START = 'cat > "$BUILD_DIR/zenoh-config/zenoh_generic_config.h" << \'EOF\''
_DEFINE = re.compile(r"^#define ([A-Za-z_][A-Za-z_0-9]*)(?: (.*))?$")
# `const NAME: ty = 60_000;` / `let name = 1;` — a value bound in the function
# body and then formatted in. Numeric literals only: anything else is not a
# value this gate can compare and is reported as dynamic instead.
_RS_CONST = re.compile(r"\bconst ([A-Za-z_][A-Za-z_0-9]*)\s*:\s*[A-Za-z0-9_]+\s*=\s*([^;]+);")
_RS_LET = re.compile(r"\blet ([a-z_][A-Za-z_0-9]*)\s*=\s*([^;]+);")


def _rust_fn_writelns(text: str, sig: str) -> list[tuple[int, str, str]]:
    """`(brace depth, format string, argument text)` for each `writeln!` in a fn.

    Depth 0 is the function body itself — an UNCONDITIONAL emission. Anything
    deeper sits inside an `if`, which is how `Z_FEATURE_UNSTABLE_API` (emitted
    only under the cargo feature) is told apart from the ~50 macros every
    generated header carries. Same trick as `cmake_answers`' if-nesting count,
    for the same reason.

    Hand-scanned rather than regexed because the format strings contain braces
    (`{}`) and the body contains comments; a scanner that counted those would
    put every emission at a nonsense depth. It asserts it ends balanced, so a
    body shape it cannot read is a LOUD failure, not a silently empty result.
    """
    i = text.index(sig)
    open_brace = text.index("{", text.index(") -> String", i))
    out: list[tuple[int, str, str]] = []
    depth = 0
    k = open_brace + 1
    n = len(text)
    while k < n:
        c = text[k]
        if c == '"':
            k = _skip_string(text, k)
            continue
        if text.startswith("//", k):
            k = text.find("\n", k)
            if k < 0:
                break
            continue
        if c == "{":
            depth += 1
        elif c == "}":
            if depth == 0:
                return out
            depth -= 1
        elif text.startswith("writeln!(", k):
            args, k = _macro_args(text, k + len("writeln!("))
            fmt, rest = _first_string(args)
            if fmt is not None:
                out.append((depth, fmt, rest))
            continue
        k += 1
    raise LaneError(f"{sig} — unbalanced braces; the scanner cannot read this function body")


def _skip_string(text: str, k: int) -> int:
    """Index just past the double-quoted string literal starting at `k`."""
    k += 1
    while k < len(text):
        if text[k] == "\\":
            k += 2
            continue
        if text[k] == '"':
            return k + 1
        k += 1
    raise LaneError("unterminated string literal")


def _macro_args(text: str, k: int) -> tuple[str, int]:
    """The text inside a macro's parens, and the index just past the `)`."""
    depth = 1
    start = k
    while k < len(text):
        c = text[k]
        if c == '"':
            k = _skip_string(text, k)
            continue
        if c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return text[start:k], k + 1
        k += 1
    raise LaneError("unterminated macro invocation")


def _first_string(args: str) -> tuple[str | None, str]:
    """The first string literal in a `writeln!`'s arguments, and what follows.

    `writeln!(header)` and `writeln!(header, x)` have none — a blank line and a
    non-literal, neither of which declares a macro.
    """
    k = args.find('"')
    if k < 0:
        return None, ""
    end = _skip_string(args, k)
    return args[k + 1 : end - 1], args[end:].lstrip(" ,\n")


def _numeric(value: str) -> str | None:
    """`60_000` → `60000`; anything not a plain integer literal → None."""
    v = value.strip().replace("_", "")
    return v if v.isdigit() else None


def rust_config_macros(text: str) -> tuple[dict[str, str | None], set[str]]:
    """`({macro → literal value or None}, {conditionally-emitted macros})`.

    A `None` value means "emitted, but its value is computed" — a link flag, a
    resolved buffer size, a knob. Those are compared by NAME only, and the OK
    line says how many, because a gate that quietly checked half of what it
    named would be the same silence one level up.
    """
    emitted = _rust_fn_writelns(text, _CONFIG_HEADER_FN)
    body_start = text.index(_CONFIG_HEADER_FN)
    body = text[body_start : text.index("\n}\n", body_start)]
    bound: dict[str, str] = {}
    for pat in (_RS_CONST, _RS_LET):
        for name, value in pat.findall(body):
            lit = _numeric(value)
            if lit is not None:
                bound[name] = lit
    macros: dict[str, str | None] = {}
    conditional: set[str] = set()
    for depth, fmt, args in emitted:
        m = _DEFINE.match(fmt)
        if not m:
            continue
        name, value = m.group(1), m.group(2)
        if depth > 0:
            conditional.add(name)
            continue
        if value is None:
            macros[name] = None
        elif "{" not in value:
            macros[name] = value.strip()
        elif value.strip() == "{}":
            macros[name] = bound.get(args.strip().rstrip(","))
        else:
            macros[name] = None
    return macros, conditional


def sh_config_macros(text: str) -> dict[str, str | None]:
    """The `#define`s in the readiness lane's generated-config heredoc."""
    start = text.find(_SH_HEREDOC_START)
    if start < 0:
        raise LaneError(
            f"{QEMU_SH.relative_to(REPO)}: no `{_SH_HEREDOC_START}` — this gate can no longer "
            "see the generated config header, so it cannot compare it"
        )
    body = text[start + len(_SH_HEREDOC_START) :]
    end = body.find("\nEOF\n")
    if end < 0:
        raise LaneError(f"{QEMU_SH.relative_to(REPO)}: unterminated config-header heredoc")
    out: dict[str, str | None] = {}
    for line in body[:end].splitlines():
        m = _DEFINE.match(line)
        if m:
            out[m.group(1)] = m.group(2).strip() if m.group(2) else None
    return out


def config_header_mirror_problems(
    rust_text: str, sh_text: str
) -> tuple[list[str], list[str]]:
    """(11): the readiness lane's config header still mirrors the generator.

    THIS ONE ACCEPTS A MIRROR, which the rest of this area does not, and the
    reason is stated so the next reader can disagree with it on purpose:
    `config_header()` resolves its inputs from cargo build-script environment —
    link features from `CARGO_FEATURE_*`, wire sizes through the RFC-0049
    ladder and the platform descriptor, knobs from `$DOTCONFIG` — none of which
    a standalone shell script can supply. Reproducing that resolution in shell
    would be a FOURTH statement of it, not one fewer. So the mirror stays and
    gains the drift-detector it never had.

    What it caught the day it was written (measured 2026-09-05, all real):
    six macros the generator emits and the mirror did not, including
    `Z_TRANSPORT_LEASE_TASK_SLEEP_CHUNK_MS` (issue 0959) and the four RFC-0080
    link flags; `Z_TRANSPORT_LEASE` still 10000 after issue 0906 moved it to
    60000; and `Z_FEATURE_MATCHING`, `Z_FEATURE_LOCAL_SUBSCRIBER`,
    `Z_FEATURE_LOCAL_QUERYABLE` and `Z_FEATURE_BATCHING` disagreeing — four
    flags that decide which code compiles, one of which (`BATCHING`) gates
    transport struct FIELDS. 45 of the 130 objects changed when they were
    brought back into line: the readiness check had been answering for a build
    nobody performs.
    """
    rust, conditional = rust_config_macros(rust_text)
    sh = sh_config_macros(sh_text)
    bad: list[str] = []
    for name in sorted(set(rust) - set(sh)):
        bad.append(
            f"`config_header()` emits `#define {name}` and {QEMU_SH.relative_to(REPO)}'s "
            "generated config header does not. The readiness build compiles the vendored "
            "tree with a different feature set than the build it proxies for."
        )
    for name in sorted(set(sh) - set(rust) - conditional):
        bad.append(
            f"{QEMU_SH.relative_to(REPO)} defines `{name}`, which `config_header()` never "
            "emits — a value the real bare-metal build does not set. Delete it, or add it "
            "to the generator if the real build wants it too."
        )
    checked = 0
    for name in sorted(set(rust) & set(sh)):
        want = rust[name]
        if want is None:
            continue
        checked += 1
        if sh[name] != want:
            bad.append(
                f"`{name}` is `{want}` in `config_header()` and `{sh[name]}` in "
                f"{QEMU_SH.relative_to(REPO)}. Change the generator and mirror it, in that "
                "order — never the mirror alone to make this gate pass."
            )
    dynamic = sorted(n for n in set(rust) & set(sh) if rust[n] is None)
    notes = [
        f"config header mirror: {len(rust)} unconditional macros, {checked} compared by "
        f"VALUE, {len(dynamic)} by NAME only — they carry no value, or one the real build "
        f"resolves from link features / the RFC-0049 ladder / a knob: {', '.join(dynamic)}"
    ]
    return bad, notes


# `-DZPICO_MAX_PUBLISHERS=8` in the readiness lane's compiler flags.
_SH_DEFINE_FLAG = re.compile(r"-D(ZPICO_[A-Z_0-9]+)=([^\s\"]+)")
# `("ZPICO_MAX_PUBLISHERS", self.max_publishers.to_string()),` in `defines()`,
# and the constant form `("ZPICO_TX_BATCH", "1".to_string())` beside it — both,
# because a gate that read only the field form would call a macro the real build
# DOES pass "one the real build does not set".
_RS_SHIM_DEFINE = re.compile(
    r'\(\s*"(ZPICO_[A-Z_0-9]+)"\s*,\s*(?:self\.([a-z_0-9]+)|"([^"]*)")\.to_string\(\)'
)
# `max_publishers: env_usize("ZPICO_MAX_PUBLISHERS", 8),` / `get_reply_buf_size: 4096,`
_RS_ENV_DEFAULT = re.compile(r"([a-z_0-9]+)\s*:\s*env_usize\(\s*\"[A-Z_0-9]+\"\s*,\s*(\d+)\s*\)")
_RS_LIT_DEFAULT = re.compile(r"^\s*([a-z_0-9]+)\s*:\s*(\d+)\s*,\s*$", re.M)


def shim_defines(lib_text: str) -> list[tuple[str, str | None, bool, str | None]]:
    """`(macro, field, unconditional?, constant)` for every `-D` the real build passes.

    Read from `ShimConfig::defines()`, which is the one place that knows the
    macro⇄field mapping; the `if self.tx_batch { … }` pushes are marked
    conditional by brace depth, the same way `config_header()`'s `if
    unstable_api` is.
    """
    i = lib_text.index("pub fn defines(&self)")
    out: list[tuple[str, str | None, bool, str | None]] = []
    depth = 0
    k = lib_text.index("{", lib_text.index("Vec<(&'static str, String)>", i))
    n = len(lib_text)
    k += 1
    while k < n:
        c = lib_text[k]
        if c == '"':
            k = _skip_string(lib_text, k)
            continue
        if lib_text.startswith("//", k):
            k = lib_text.find("\n", k)
            if k < 0:
                break
            continue
        if c == "{":
            depth += 1
        elif c == "}":
            if depth == 0:
                if not out:
                    raise LaneError(
                        "ShimConfig::defines() yielded no (macro, field) pairs — this gate "
                        "cannot read it, and an empty list would compare nothing"
                    )
                return out
            depth -= 1
        elif c == "(":
            m = _RS_SHIM_DEFINE.match(lib_text, k)
            if m:
                out.append((m.group(1), m.group(2), depth == 0, m.group(3)))
                k = m.end()
                continue
        k += 1
    raise LaneError("ShimConfig::defines() — unbalanced braces")


def shim_defaults(runner_text: str) -> dict[str, str]:
    """`ShimConfig` field → its default, for the fields whose default is a literal.

    Read from `shim_config_from_env()`. A field built from anything else — the
    queryable table, which is sized from the resolved model or a per-target
    budget (`resolve_queryable_default`), and the ladder knobs `resolve_wire`
    overwrites — is simply absent, and its macro is then compared by NAME only.
    """
    i = runner_text.index("fn shim_config_from_env()")
    body = runner_text[i : runner_text.index("\n}\n", i)]
    out = {f: v for f, v in _RS_ENV_DEFAULT.findall(body)}
    for f, v in _RS_LIT_DEFAULT.findall(body):
        out.setdefault(f, v)
    return out


def shim_slot_problems(lib_text: str, runner_text: str, sh_text: str) -> tuple[list[str], list[str]]:
    """(12): the readiness lane passes the shim's slot counts, and the right ones.

    Same argument as (11) — a mirror that cannot be a derivation, so it gets the
    detector instead. What is different here is the failure mode of an OMISSION,
    and `ShimConfig::defines()` states it in its own comment: the C shim
    `#define`s its own fallback for every one of these, so leaving a `-D` out is
    not "no opinion", it is the C default silently winning. That is issue 0460's
    shape, and it was live on this lane: the real bare-metal build passes TEN
    of these unconditionally and the readiness script passed SEVEN.
    """
    declared = shim_defines(lib_text)
    defaults = shim_defaults(runner_text)
    got = dict(_SH_DEFINE_FLAG.findall(_sibling().strip_comments(sh_text, "#")))
    known = {macro for macro, _f, _u, _c in declared}
    bad: list[str] = []
    checked = 0
    for macro, field, unconditional, constant in declared:
        want = constant if constant is not None else defaults.get(field)
        if macro not in got:
            if unconditional:
                bad.append(
                    f"{QEMU_SH.relative_to(REPO)} does not pass `-D{macro}` — the real build "
                    f"passes it on every target (`ShimConfig::defines()`), and the C shim "
                    f"`#define`s its own fallback, so omitting it is not 'no opinion': it is "
                    "the shim's default silently winning on this lane only (issue 0460)."
                )
            continue
        if want is None:
            continue
        checked += 1
        if got[macro] != want:
            bad.append(
                f"`{macro}` is `{want}` in `shim_config_from_env()` and `{got[macro]}` in "
                f"{QEMU_SH.relative_to(REPO)}. These are STATIC array sizes in the C shim, so "
                "the readiness build sizes its tables differently from the build it proxies "
                "for. Change the default and mirror it, in that order."
            )
    for macro in sorted(set(got) - known):
        bad.append(
            f"{QEMU_SH.relative_to(REPO)} passes `-D{macro}`, which `ShimConfig::defines()` "
            "never emits — a knob the real build does not set. Delete it, or add it there if "
            "the real build wants it too."
        )
    notes = [
        f"shim slot defines: {len(known)} declared by ShimConfig::defines(), {len(got)} passed "
        f"by the readiness lane, {checked} compared by VALUE"
    ]
    return bad, notes


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

    # --- (10) the readiness lane answers as the lane it proxies for ---------
    sh_block = (
        f"# {_BEGIN}\n"
        "zenoh_condition() {\n"
        "    case \"$1\" in\n"
        "        always) return 0 ;;\n"
        "        zephyr) return 1 ;;\n"
        "        zephyr_isotp) return 1 ;;\n"
        "        *) exit 1 ;;\n"
        "    esac\n"
        "}\n"
        f"# {_END}\n"
        "INCLUDES=\"$INCLUDES -I$PLATFORM_DIR/bare-metal\"\n"
    )
    # SHELL TRUTH IS INVERTED. A reader that got this backwards would call the
    # correct file wrong and the broken one right, which is the only way this
    # check can be worse than nothing.
    assert sh_answers(sh_block, "T") == {
        "always": True,
        "zephyr": False,
        "zephyr_isotp": False,
    }, sh_answers(sh_block, "T")
    # `*)` is a refusal, not an answer.
    assert "*" not in sh_answers(sh_block, "T")

    toks3 = {"always", "zephyr", "zephyr_isotp"}
    plats_ok = {
        "bare-metal": ("cargo", Path("config/bare-metal/nros-platform.toml")),
        "zephyr": ("platform", Path("packages/platform/nros-platform-zephyr/nros-platform.toml")),
    }
    assert readiness_lane_problems(toks3, plats_ok, sh_answers(sh_block, "T"), sh_block) == []

    # THE CROSS-WIRE, and it is shape-valid in every other gate: every token is
    # declared, used, answered by all three lanes and counted identically. This
    # one drags Zephyr's vendored `system/zephyr/network.c` into a Cortex-M3
    # readiness build — the proxy stops standing for the build it proxies.
    crossed = dict(sh_answers(sh_block, "T"), zephyr=True)
    msgs = readiness_lane_problems(toks3, plats_ok, crossed, sh_block)
    assert any("answers condition token `zephyr` TRUE" in m for m in msgs), msgs
    # ...and its mirror: the core answered false compiles nothing at all.
    msgs = readiness_lane_problems(
        toks3, plats_ok, dict(sh_answers(sh_block, "T"), always=False), sh_block
    )
    assert any("answers condition token `always` FALSE" in m for m in msgs), msgs

    # the three statements this check refuses to assume.
    msgs = readiness_lane_problems(toks3, plats_ok, sh_answers(sh_block, "T"),
                                   sh_block.replace("$PLATFORM_DIR/bare-metal", "$PLATFORM_DIR/x"))
    assert any("does not reach for `$PLATFORM_DIR/bare-metal`" in m for m in msgs), msgs
    plats_taken = dict(plats_ok, **{"bare-metal": ("platform", Path("p.toml"))})
    msgs = readiness_lane_problems(toks3, plats_taken, sh_answers(sh_block, "T"), sh_block)
    assert any("stands for a build nobody performs" in m for m in msgs), msgs
    msgs = readiness_lane_problems(toks3, {"zephyr": ("platform", Path("p"))},
                                   sh_answers(sh_block, "T"), sh_block)
    assert any("no nros-platform.toml declares" in m for m in msgs), msgs
    # a token this rule cannot decide fails CLOSED rather than going unchecked.
    msgs = readiness_lane_problems(
        {"isotp"}, plats_ok, {"isotp": True}, sh_block
    )
    assert any("cannot decide how the bare-metal readiness lane" in m for m in msgs), msgs

    # --- (11) the config-header mirror --------------------------------------
    fake_rs = (
        "pub fn config_header(link: &L) -> String {\n"
        '    writeln!(header, "#define ZENOH_GENERIC_CONFIG_H").unwrap();\n'
        "    const LEASE_MS: u32 = 60_000;\n"
        "    let loopback = 1;\n"
        '    writeln!(header, "#define Z_TRANSPORT_LEASE {}", LEASE_MS).unwrap();\n'
        '    writeln!(header, "#define Z_FEATURE_LOCAL_SUBSCRIBER {}", loopback).unwrap();\n'
        '    writeln!(header, "#define Z_FEATURE_MATCHING 1").unwrap();\n'
        '    writeln!(header, "#define Z_FEATURE_RX_CACHE 0").unwrap();\n'
        '    writeln!(header, "#define Z_FEATURE_LINK_TCP {}", link.tcp_flag()).unwrap();\n'
        "    writeln!(header).unwrap();\n"
        "    if unstable_api {\n"
        '        writeln!(header, "#define Z_FEATURE_UNSTABLE_API").unwrap();\n'
        "    }\n"
        "    header\n"
        "}\n"
    )
    macros, conditional = rust_config_macros(fake_rs)
    assert macros == {
        "ZENOH_GENERIC_CONFIG_H": None,
        "Z_TRANSPORT_LEASE": "60000",      # `const` resolved, `_` separators dropped
        "Z_FEATURE_LOCAL_SUBSCRIBER": "1",  # `let` resolved
        "Z_FEATURE_MATCHING": "1",
        "Z_FEATURE_RX_CACHE": "0",
        "Z_FEATURE_LINK_TCP": None,         # computed → NAME only
    }, macros
    # emitted under an `if`, so it is not required of the mirror.
    assert conditional == {"Z_FEATURE_UNSTABLE_API"}, conditional

    def _sh(body: str) -> str:
        return _SH_HEREDOC_START + "\n" + body + "\nEOF\n"

    good_sh = _sh(
        "#define ZENOH_GENERIC_CONFIG_H\n"
        "#define Z_TRANSPORT_LEASE 60000\n"
        "#define Z_FEATURE_LOCAL_SUBSCRIBER 1\n"
        "#define Z_FEATURE_MATCHING 1\n"
        "#define Z_FEATURE_RX_CACHE 0\n"
        "#define Z_FEATURE_LINK_TCP 1\n"
    )
    bad_msgs, notes_t = config_header_mirror_problems(fake_rs, good_sh)
    assert bad_msgs == [], bad_msgs
    assert "4 compared by VALUE" in notes_t[0], notes_t
    assert "2 by NAME only" in notes_t[0], notes_t

    # the drift that was actually there: a macro the generator gained and the
    # mirror never did (`Z_TRANSPORT_LEASE_TASK_SLEEP_CHUNK_MS`, issue 0959).
    msgs, _ = config_header_mirror_problems(
        fake_rs, good_sh.replace("#define Z_FEATURE_MATCHING 1\n", "")
    )
    assert any("emits `#define Z_FEATURE_MATCHING`" in m for m in msgs), msgs
    # ...and the reverse: a value the real build does not set at all.
    msgs, _ = config_header_mirror_problems(
        fake_rs, good_sh.replace("#define Z_FEATURE_RX_CACHE 0", "#define Z_FEATURE_GHOST 1")
    )
    assert any("defines `Z_FEATURE_GHOST`" in m for m in msgs), msgs
    # ...and a value that drifted (`Z_TRANSPORT_LEASE` 10000 vs 60000).
    msgs, _ = config_header_mirror_problems(
        fake_rs, good_sh.replace("Z_TRANSPORT_LEASE 60000", "Z_TRANSPORT_LEASE 10000")
    )
    assert any("`Z_TRANSPORT_LEASE` is `60000`" in m for m in msgs), msgs

    # THE SHAPE-PRESERVING CROSSING: the mirror's macro SET is identical, every
    # value is one the generator emits somewhere, and two of them have traded
    # places. Nothing about the file's shape changed — this is the mutation that
    # a name-set check alone reports as green.
    swapped = good_sh.replace("#define Z_FEATURE_MATCHING 1", "#define Z_FEATURE_MATCHING 0")
    swapped = swapped.replace("#define Z_FEATURE_RX_CACHE 0", "#define Z_FEATURE_RX_CACHE 1")
    msgs, _ = config_header_mirror_problems(fake_rs, swapped)
    assert len(msgs) == 2, msgs
    assert any("`Z_FEATURE_MATCHING` is `1`" in m for m in msgs), msgs
    assert any("`Z_FEATURE_RX_CACHE` is `0`" in m for m in msgs), msgs

    # a conditional emission is not required of the mirror, but is accepted in
    # it — the mirror may legitimately carry a `#define` the generator only
    # writes under a cargo feature.
    msgs, _ = config_header_mirror_problems(
        fake_rs, good_sh.replace("#define Z_FEATURE_RX_CACHE 0",
                                 "#define Z_FEATURE_RX_CACHE 0\n#define Z_FEATURE_UNSTABLE_API")
    )
    assert msgs == [], msgs

    # a shape this gate cannot read is LOUD, never an empty comparison: an empty
    # macro set would pass every check above while comparing nothing.
    for broken in ("", _SH_HEREDOC_START + "\n#define X 1\n"):
        try:
            sh_config_macros(broken)
        except LaneError:
            pass
        else:  # pragma: no cover
            raise AssertionError(f"unreadable heredoc accepted: {broken!r}")
    try:
        _rust_fn_writelns("pub fn config_header(x) -> String {\n  if a {\n", _CONFIG_HEADER_FN)
    except LaneError:
        pass
    else:  # pragma: no cover
        raise AssertionError("unbalanced fn body accepted")

    # --- (12) the shim slot defines ----------------------------------------
    fake_defines = (
        "    pub fn defines(&self) -> Vec<(&'static str, String)> {\n"
        "        let mut out = vec![\n"
        '            ("ZPICO_MAX_PUBLISHERS", self.max_publishers.to_string()),\n'
        '            ("ZPICO_MAX_QUERYABLES", self.max_queryables.to_string()),\n'
        "        ];\n"
        "        if self.tx_batch {\n"
        '            out.push(("ZPICO_TX_BATCH", "1".to_string()));\n'
        "        }\n"
        '        out.push(("ZPICO_READ_TASK_PRIORITY", self.read_task_priority.to_string()));\n'
        "        out\n"
        "    }\n"
    )
    fake_runner = (
        "fn shim_config_from_env() -> ShimConfig {\n"
        "    let max_queryables = env_usize(\"ZPICO_MAX_QUERYABLES\", sizing.default);\n"
        "    ShimConfig {\n"
        '        max_publishers: env_usize("ZPICO_MAX_PUBLISHERS", 8),\n'
        "        max_queryables,\n"
        '        read_task_priority: env_usize("ZPICO_READ_TASK_PRIORITY", 16),\n'
        "        get_poll_interval_ms: 10,\n"
        "    }\n"
        "}\n"
    )
    assert shim_defines(fake_defines) == [
        ("ZPICO_MAX_PUBLISHERS", "max_publishers", True, None),
        ("ZPICO_MAX_QUERYABLES", "max_queryables", True, None),
        # a CONSTANT push, and behind an `if` — both facts have to survive, or a
        # macro the real build does pass reads as one it does not set.
        ("ZPICO_TX_BATCH", None, False, "1"),
        ("ZPICO_READ_TASK_PRIORITY", "read_task_priority", True, None),
    ], shim_defines(fake_defines)
    assert shim_defaults(fake_runner) == {
        "max_publishers": "8",
        "read_task_priority": "16",
        "get_poll_interval_ms": "10",
    }, shim_defaults(fake_runner)
    # `max_queryables` is sized from the resolved model or a per-target budget,
    # so it has NO literal default and is compared by name only. A gate that
    # invented one would report every correct build as drifted.
    assert "max_queryables" not in shim_defaults(fake_runner)

    good_flags = (
        'DEFINES="$DEFINES -DZPICO_MAX_PUBLISHERS=8"\n'
        'DEFINES="$DEFINES -DZPICO_MAX_QUERYABLES=8"\n'
        'DEFINES="$DEFINES -DZPICO_READ_TASK_PRIORITY=16"\n'
    )
    msgs, notes_s = shim_slot_problems(fake_defines, fake_runner, good_flags)
    assert msgs == [], msgs
    assert "2 compared by VALUE" in notes_s[0], notes_s
    # THE OMISSION, which is issue 0460's shape: the C shim's own `#ifndef`
    # fallback wins on this lane and nowhere else, so nothing looks wrong.
    msgs, _ = shim_slot_problems(
        fake_defines, fake_runner,
        good_flags.replace('DEFINES="$DEFINES -DZPICO_READ_TASK_PRIORITY=16"\n', ""),
    )
    assert any("does not pass `-DZPICO_READ_TASK_PRIORITY`" in m for m in msgs), msgs
    # ...and a CONDITIONAL one may be absent without complaint.
    assert not any("ZPICO_TX_BATCH" in m for m in msgs), msgs
    # THE CROSSING: the flag set is unchanged and two counts trade places.
    swapped = good_flags.replace("-DZPICO_MAX_PUBLISHERS=8", "-DZPICO_MAX_PUBLISHERS=16")
    swapped = swapped.replace("-DZPICO_READ_TASK_PRIORITY=16", "-DZPICO_READ_TASK_PRIORITY=8")
    msgs, _ = shim_slot_problems(fake_defines, fake_runner, swapped)
    assert len(msgs) == 2, msgs
    assert any("`ZPICO_MAX_PUBLISHERS` is `8`" in m for m in msgs), msgs
    assert any("`ZPICO_READ_TASK_PRIORITY` is `16`" in m for m in msgs), msgs
    # a knob the real build does not set at all.
    msgs, _ = shim_slot_problems(
        fake_defines, fake_runner, good_flags + 'DEFINES="$DEFINES -DZPICO_INVENTED=3"\n'
    )
    assert any("passes `-DZPICO_INVENTED`" in m for m in msgs), msgs
    # a comment is not a compile flag.
    msgs, _ = shim_slot_problems(
        fake_defines, fake_runner, good_flags + "# was -DZPICO_INVENTED=3\n"
    )
    assert msgs == [], msgs
    # a `defines()` this gate cannot read is LOUD, never an empty comparison.
    try:
        shim_defines(
            "    pub fn defines(&self) -> Vec<(&'static str, String)> {\n        vec![]\n    }\n"
        )
    except LaneError:
        pass
    else:  # pragma: no cover
        raise AssertionError("empty defines() accepted")


def main() -> int:
    self_test()

    bad: list[str] = []
    sibling = _sibling()
    try:
        groups, rows = sibling.parse_manifest(MANIFEST.read_text(encoding="utf-8"), str(MANIFEST))
        texts = {
            p: p.read_text(encoding="utf-8")
            for p in {CMAKE, RUST_LIB, RUST_RUNNER, QEMU_SH}
        }
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
    bad += readiness_lane_problems(
        set(groups.values()), platforms, sh_answers(texts[QEMU_SH]), texts[QEMU_SH]
    )
    mirror_bad, notes = config_header_mirror_problems(texts[RUST_LIB], texts[QEMU_SH])
    bad += mirror_bad
    slot_bad, slot_notes = shim_slot_problems(
        texts[RUST_LIB], texts[RUST_RUNNER], texts[QEMU_SH]
    )
    bad += slot_bad
    notes += slot_notes

    if bad:
        print("check-zenoh-lane-ownership: FAIL", file=sys.stderr)
        for m in bad:
            print(f"  - {m}", file=sys.stderr)
        return 1

    owned = sorted(n for n, (o, _w) in platforms.items() if o == "platform")
    cargo = sorted(n for n, (o, _w) in platforms.items() if o == "cargo")
    for note in notes:
        print(f"check-zenoh-lane-ownership: {note}")
    print(
        "check-zenoh-lane-ownership: OK — "
        f"cargo compiles the vendored tree for {cargo}, "
        f"{sorted(WEST_LANES)} for {owned}; no platform is in both, none in neither; "
        f"the {READINESS_PLATFORM} readiness lane "
        f"({QEMU_SH.relative_to(REPO)}) proxies the cargo one"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
