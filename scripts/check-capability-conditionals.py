#!/usr/bin/env python3
"""A `when.capability` gate must decide something, and decide it the right way.

Issue 1143. `[build.<component>]` blocks gained `defines_conditional` and a
`when.capability` matcher so the FreeRTOS family can pick its zenoh-pico
`system/` TU set from a BOARD fact (`capabilities.ip_stack`) instead of from
the platform's name. The mechanism has three ways to be silently wrong, and
none of them is a parse error:

1.  **The fact is never declared.** `matches()` applies RFC-0086's
    absent-is-not-false rule, so a matcher naming a capability that neither the
    platform nor any board declares matches NEITHER `true` NOR `false`. A
    two-arm conditional over an undeclared key therefore drops BOTH arms, and
    the build reaches `#error "Unknown platform"` with nothing in the manifest
    looking wrong.

2.  **The arms do not partition.** `defines_conditional` is how a platform
    header gets chosen. Two rows on the same capability with the same value
    define two headers; one row with only the `true` arm leaves the `false`
    build with no header at all. Exactly one arm must win, always.

3.  **The arms are crossed.** This is the one that survives every structural
    check, and it is the reason this gate reads zenoh-pico's own headers rather
    than a table written here. `ZENOH_FREERTOS_LWIP` selects
    `system/platform/freertos/lwip.h`, whose `_z_sys_net_socket_t` is
    `{ int _socket; }` and which `#include`s `lwip/sockets.h`;
    `ZENOH_ORIN_SPE` selects `orin_spe.h`, whose socket is a one-byte
    placeholder. Swap the two `when` values and the manifest still parses, the
    build still links, and a caller writes an `int` through a one-byte struct —
    issue 0135's class, arrived at by an edit that looks like a rename.

So: the define chosen when a board says it has NO IP stack must resolve to a
placeholder socket, and the define chosen when it HAS one must not. Both sides
are read out of the vendored header, so the check cannot drift from the code it
describes.

Rule 5 is the live half on today's tree: a platform whose zenoh build compiles
a netstack-specific `system/**/network.c` has to DECLARE `ip_stack`, because
that source is the thing the fact is about.

Rules 7 and 8 close the same crossing ONE FIELD OVER, which is where rule 4
cannot see. Rule 4 reads the socket type out of the header a DEFINE selects, so
it is blind to every other row that has to move with that define: the vendor
`network.c` that implements the socket, the include roots its headers live
under, and the env var that spells them. Each of those is an independent `when`,
and flipping any one of them leaves a manifest that parses, partitions, and
passes rule 4 while compiling lwIP's `network.c` against `orin_spe.h`'s
placeholder socket -- issue 0135's split, reached by editing a field rule 4 does
not read. So: whichever define arm resolves to a REAL socket is the NETSTACK
arm, and rule 7 requires the netstack source to carry that arm's condition,
while rule 8 requires every `{env:NAME}` that only the netstack rows name to
carry it too, in `required_env` and in both include-path lists alike. A var no
conditional row names (`FREERTOS_DIR`) says nothing about the netstack and is
left alone.

Rule 3 keeps the two halves of 1143 wired to one string: the Rust selector
(`LinkPolicy::for_board`, keyed on `nros_board_common::policy::IP_STACK`) and
the manifest rows have to name the same capability, and a rename on one side
that leaves the other alone is exactly the silent-inert shape rule 1 describes.

Usage::

    check-capability-conditionals.py          # the gate (selftest runs first)
    check-capability-conditionals.py --self-test
"""

import pathlib
import re
import sys

try:
    import tomllib
except ModuleNotFoundError:  # 3.10 backport, as the sibling gates spell it
    import tomli as tomllib  # type: ignore

ROOT = pathlib.Path(__file__).resolve().parent.parent

POLICY_RS = ROOT / "packages/boards/nros-board-common/src/policy.rs"
ZENOH_PICO = ROOT / "packages/rmw/zenoh/zpico-sys/zenoh-pico"
PLATFORM_DISPATCH = ZENOH_PICO / "include/zenoh-pico/system/common/platform.h"
PLATFORM_INCLUDE_ROOT = ZENOH_PICO / "include"

# `#elif defined(ZENOH_X)` / `#include "zenoh-pico/system/platform/y.h"` pairs.
DISPATCH_MACRO = re.compile(r"^#\s*(?:el)?if\s+defined\s*\(\s*([A-Z0-9_]+)\s*\)")
DISPATCH_INCLUDE = re.compile(r'^#\s*include\s+"([^"]+)"')

# `pub const IP_STACK: &str = "ip_stack";`
RUST_CONST = re.compile(r'pub const\s+([A-Z0-9_]+)\s*:\s*&str\s*=\s*"([^"]+)"\s*;')

# A netstack-specific vendor source: `system/<rtos>/<stack>/network.c`.
NETSTACK_SOURCE = re.compile(r"system/[a-z0-9_]+/[a-z0-9_+]+/network\.c")

# `{env:NAME}` as the manifest spells an environment variable in a path.
ENV_REF = re.compile(r"\{env:([A-Z0-9_]+)\}")

# The socket struct a platform header declares. Captured body, not parsed C —
# the only question asked of it is whether it holds anything but a placeholder.
# `(?:(?!typedef).)*?` and not a bare `.*?`: a platform header declares four or
# five structs before this one, and a non-greedy body anchored at the FIRST
# `typedef struct {` in the file swallows all of them — which read
# `orin_spe.h`'s placeholder socket as a real one, in the direction that passes.
SOCKET_TYPEDEF = re.compile(
    r"typedef\s+struct\s*\{((?:(?!typedef).)*?)\}\s*_z_sys_net_socket_t\s*;", re.S
)


def platform_manifests(root: pathlib.Path) -> list[pathlib.Path]:
    """Every `nros-platform.toml` a platform package ships."""
    return sorted(root.glob("packages/platform/*/nros-platform.toml"))


# ── the manifest side ────────────────────────────────────────────────────────


def capability_matchers(block: dict) -> list[tuple[str, str, bool, dict]]:
    """Every `when.capability` entry in one `[build.<component>]` block.

    Yields `(where, capability, wanted_value, row)` so a failure can name the
    row rather than the block. `where` is the field the row came from, which is
    what tells a reader whether the partition rule (defines only) applies.
    """
    out: list[tuple[str, str, bool, dict]] = []
    fields = [
        ("defines_conditional", block.get("defines_conditional") or []),
        ("extra_sources", block.get("extra_sources") or []),
        ("include_paths_conditional", block.get("include_paths_conditional") or []),
    ]
    for field, rows in fields:
        if not isinstance(rows, list):
            continue
        for row in rows:
            if not isinstance(row, dict):
                continue
            when = row.get("when") or {}
            caps = when.get("capability") or {}
            if not isinstance(caps, dict):
                continue
            for name, want in caps.items():
                out.append((field, name, bool(want), row))
    return out


def declared_capabilities(doc: dict) -> set[str]:
    caps = doc.get("capabilities") or {}
    return set(caps) if isinstance(caps, dict) else set()


def build_blocks(doc: dict) -> dict[str, dict]:
    build = doc.get("build") or {}
    return {k: v for k, v in build.items() if isinstance(v, dict)}


def env_conditions(
    block: dict, cap: str
) -> dict[str, list[tuple[str, str, bool | None]]]:
    """Every `{env:NAME}` this block references, and under what condition.

    Maps NAME -> [(field, spelling, want)], `want` being the value of `cap` the
    referencing row is gated on, or `None` when that row is unconditional. A
    `required_env` row is ABOUT the variable it names rather than spelling one,
    so its name is synthesized into the same shape -- the point of the map is
    that one variable's rows are compared against each other, and a
    `required_env` that disagrees with the include roots it exists to validate
    is exactly the drift rule 8 is looking for.
    """
    out: dict[str, list[tuple[str, str, bool | None]]] = {}

    def note(field: str, spelling: str, row: dict | None) -> None:
        want: bool | None = None
        if isinstance(row, dict):
            caps = (row.get("when") or {}).get("capability") or {}
            if isinstance(caps, dict) and cap in caps:
                want = bool(caps[cap])
        for name in ENV_REF.findall(spelling):
            out.setdefault(name, []).append((field, spelling, want))

    for path in block.get("include_paths") or []:
        if isinstance(path, str):
            note("include_paths", path, None)
    for field in ("include_paths_conditional", "extra_sources"):
        rows = block.get(field) or []
        if not isinstance(rows, list):
            continue
        for row in rows:
            if isinstance(row, dict):
                note(field, str(row.get("path", "")), row)
    for row in block.get("required_env") or []:
        if isinstance(row, dict) and row.get("name"):
            note("required_env", "{env:%s}" % row["name"], row)
    return out


def netstack_sources(block: dict) -> list[str]:
    """Vendor `system/<rtos>/<stack>/network.c` rows this block compiles."""
    out = []
    for row in block.get("extra_sources") or []:
        if isinstance(row, dict) and NETSTACK_SOURCE.search(str(row.get("path", ""))):
            out.append(str(row["path"]))
    return out


# ── the zenoh-pico side (rule 4's source of truth) ───────────────────────────


def dispatch_map(text: str) -> dict[str, str]:
    """`ZENOH_<PLATFORM>` -> the header its `#elif` arm includes."""
    out: dict[str, str] = {}
    pending: str | None = None
    for line in text.splitlines():
        m = DISPATCH_MACRO.match(line)
        if m:
            pending = m.group(1)
            continue
        m = DISPATCH_INCLUDE.match(line)
        if m and pending:
            out[pending] = m.group(1)
            pending = None
    return out


def socket_is_placeholder(header_text: str) -> bool | None:
    """Does this platform header's `_z_sys_net_socket_t` carry no real member?

    `None` when the header declares no such type — the caller reports that as
    "cannot answer", never as a pass.
    """
    m = SOCKET_TYPEDEF.search(header_text)
    if not m:
        return None
    body = m.group(1)
    # Strip comments, braces and the union keyword; what remains are members.
    body = re.sub(r"/\*.*?\*/", " ", body, flags=re.S)
    body = re.sub(r"//[^\n]*", " ", body)
    members = [
        seg.strip()
        for seg in re.split(r"[;{}]", body.replace("union", " "))
        if seg.strip()
    ]
    return all("_placeholder" in m_ for m_ in members)


# ── the rules ────────────────────────────────────────────────────────────────


def check_manifest(
    label: str,
    doc: dict,
    dispatch: dict[str, str],
    header_text,
    ip_stack: str,
) -> list[str]:
    """Rules 1, 2, 4 and 5 for one platform manifest document.

    `header_text` maps a header path (as the dispatch chain spells it) to its
    source, or returns `None` for one this tree does not ship. `ip_stack` is
    the capability rule 4 reasons about — passed in rather than read from
    `policy.rs` here, so a RENAME of that constant is reported by rule 3, which
    is the rule about it, instead of collapsing the selftest into a failure
    that names nothing.
    """
    problems: list[str] = []
    declared = declared_capabilities(doc)

    for component, block in sorted(build_blocks(doc).items()):
        rows = capability_matchers(block)

        # Rule 1 — the fact has to be declared, or the matcher decides nothing.
        for field, cap, want, _row in rows:
            if cap not in declared:
                problems.append(
                    f"{label}: [build.{component}] {field} gates on "
                    f"`capability.{cap} = {str(want).lower()}`, but this platform "
                    f"declares no `[capabilities] {cap}`.\n"
                    f"    Absent is not false (RFC-0086): the matcher then matches "
                    f"NEITHER value, so every arm is dropped and the build gets "
                    f"none of them. Declare the fact, or drop the gate."
                )

        # Rule 2 — the define arms on one capability must partition.
        by_cap: dict[str, list[bool]] = {}
        for field, cap, want, _row in rows:
            if field == "defines_conditional":
                by_cap.setdefault(cap, []).append(want)
        for cap, wants in sorted(by_cap.items()):
            for value in (True, False):
                n = wants.count(value)
                if n == 0:
                    problems.append(
                        f"{label}: [build.{component}] has defines_conditional rows "
                        f"on `{cap}` but none for `{cap} = {str(value).lower()}`.\n"
                        f"    A build on that side of the fact gets NO define from "
                        f"this group. For a zenoh platform macro that is "
                        f'`#error "Unknown platform"`; for anything else it is a '
                        f"silently different compile."
                    )
                elif n > 1:
                    problems.append(
                        f"{label}: [build.{component}] defines {n} conditional "
                        f"defines for `{cap} = {str(value).lower()}`; exactly one "
                        f"arm may win."
                    )

        # Rule 4 — the arms must not be crossed. Read the socket type out of
        # the header the define actually selects.
        for field, cap, want, row in rows:
            if field != "defines_conditional":
                continue
            name = str(row.get("name", ""))
            header = dispatch.get(name)
            if header is None:
                # Not a zenoh-pico platform macro; nothing to cross-check.
                continue
            text = header_text(header)
            if text is None:
                problems.append(
                    f"{label}: [build.{component}] defines `{name}`, whose "
                    f"dispatch arm includes `{header}` — not present in this "
                    f"tree, so the socket ABI cannot be checked."
                )
                continue
            placeholder = socket_is_placeholder(text)
            if placeholder is None:
                problems.append(
                    f"{label}: [build.{component}] defines `{name}` -> `{header}`, "
                    f"which declares no `_z_sys_net_socket_t`."
                )
                continue
            # `ip_stack` is the fact this rule is about; other capabilities say
            # nothing about sockets and are left to rules 1 and 2.
            if cap != ip_stack:
                continue
            if want is False and not placeholder:
                problems.append(
                    f"{label}: [build.{component}] selects `{name}` when "
                    f"`{cap} = false`, but `{header}` declares a REAL "
                    f"`_z_sys_net_socket_t`.\n"
                    f"    A board with no IP stack would compile a socket type "
                    f"whose backing netstack is not in the image — the 63 "
                    f"undefined `lwip_*` symbols issue 1143 measured, or worse, "
                    f"a link that succeeds against a shorter struct (issue 0135)."
                )
            if want is True and placeholder:
                problems.append(
                    f"{label}: [build.{component}] selects `{name}` when "
                    f"`{cap} = true`, but `{header}`'s `_z_sys_net_socket_t` is a "
                    f"PLACEHOLDER.\n"
                    f"    A board with real sockets would hand `_z_open_tcp` a "
                    f"one-byte struct to write an `int` into. Same two rows, "
                    f"values swapped, is how this arrives."
                )

        # Rules 7 and 8 — the netstack ARM, and everything that travels with
        # it. The arm is read from the DEFINES, because the define is what
        # selects the platform header: whichever arm resolves to a REAL socket
        # is the netstack one. Rule 4 stops there; these two follow the arm
        # into the fields rule 4 does not read.
        netstack_want: bool | None = None
        for field, cap, want, row in rows:
            if field != "defines_conditional" or cap != ip_stack:
                continue
            header = dispatch.get(str(row.get("name", "")))
            text = header_text(header) if header else None
            if text is not None and socket_is_placeholder(text) is False:
                netstack_want = want

        if netstack_want is not None:
            arm = str(netstack_want).lower()

            # Rule 7 — the vendor `network.c` IS the netstack. It has to be
            # gated, and gated the same way the define is.
            for row in block.get("extra_sources") or []:
                if not isinstance(row, dict):
                    continue
                path = str(row.get("path", ""))
                if not NETSTACK_SOURCE.search(path):
                    continue
                caps = (row.get("when") or {}).get("capability") or {}
                got = caps.get(ip_stack) if isinstance(caps, dict) else None
                if got is None:
                    problems.append(
                        f"{label}: [build.{component}] compiles `{path}` "
                        f"UNCONDITIONALLY, but the platform header is chosen by "
                        f"`{ip_stack}`.\n"
                        f"    Every board in this family would compile the "
                        f"netstack TU, including one whose header declares no "
                        f"socket to implement — issue 1143 as it was filed."
                    )
                elif bool(got) != netstack_want:
                    problems.append(
                        f"{label}: [build.{component}] compiles `{path}` when "
                        f"`{ip_stack} = {str(bool(got)).lower()}`, but the netstack "
                        f"define arm is `{ip_stack} = {arm}`.\n"
                        f"    The TU and the header it implements would be "
                        f"selected by OPPOSITE values, so the build that gets "
                        f"the placeholder socket is the one that compiles the "
                        f"real netstack against it (issue 0135)."
                    )

            # Rule 8 — an env var only the netstack rows name has to be gated
            # with them. A var no conditional row names says nothing about the
            # netstack and is not this rule's business.
            for name, uses in sorted(env_conditions(block, ip_stack).items()):
                wants = {want for _field, _spelling, want in uses}
                if wants == {None} or wants == {netstack_want}:
                    continue
                where = ", ".join(
                    f"{field} (`{ip_stack}` "
                    + ("unconditional" if want is None else str(want).lower())
                    + ")"
                    for field, _spelling, want in uses
                )
                problems.append(
                    f"{label}: [build.{component}] references `{{env:{name}}}` "
                    f"under inconsistent conditions: {where}.\n"
                    f"    The netstack define arm is `{ip_stack} = {arm}`; a "
                    f"variable its rows name must be gated the same way "
                    f"everywhere, or a build either demands a path it has no "
                    f"use for or compiles without one it needs."
                )

        # Rule 5 — a block that compiles a netstack has to say the fact exists.
        srcs = netstack_sources(block)
        if srcs and ip_stack not in declared:
            problems.append(
                f"{label}: [build.{component}] compiles {', '.join(srcs)} but the "
                f"platform declares no `[capabilities] {ip_stack}`.\n"
                f"    That source IS the netstack; a board that cannot reach one "
                f"has no way to say so, which is issue 1143."
            )

    return problems


def rust_ip_stack_name(_cache: dict = {}) -> str:
    """The capability string `LinkPolicy::for_board` keys on.

    Read from `policy.rs` rather than repeated here — the point of rule 3 is
    that there is ONE string, so this file must not become a second copy of it.
    """
    if "v" not in _cache:
        consts = dict(
            (k, v) for k, v in RUST_CONST.findall(POLICY_RS.read_text(encoding="utf-8"))
        )
        _cache["v"] = consts.get("IP_STACK", "")
    return _cache["v"]


RUNNER_RS = ROOT / "packages/rmw/zenoh/nros-zpico-build/src/runner.rs"


def check_selector_is_reached() -> list[str]:
    """Rule 6 — something has to CALL the board rung.

    The state issue 1143 found the tree in was not a missing policy: it was
    `LinkPolicy::ivc_only()`, written and correct and carrying an
    `#[allow(dead_code)]` because no caller existed. A selector nothing invokes
    is a comment, and it looks exactly like a working one from every other
    gate's angle — the tests pass, the manifest parses, and every build quietly
    keeps the per-platform answer.

    So the wire itself is checked: `nros-zpico-build`'s runner must apply
    `LinkPolicy::for_board`, and `for_board` must still be the thing that
    reaches `ivc_only()`.
    """
    problems: list[str] = []
    if not RUNNER_RS.is_file():
        return [f"{RUNNER_RS.relative_to(ROOT)}: missing — cannot check the selector."]
    runner = RUNNER_RS.read_text(encoding="utf-8")
    if "LinkPolicy::for_board(" not in runner:
        problems.append(
            f"{RUNNER_RS.relative_to(ROOT)}: does not call "
            f"`LinkPolicy::for_board(...)`.\n"
            f"    The per-platform if/else then stands alone and the board rung "
            f"is inert — `ivc_only()` goes back to being unreachable, which is "
            f"the whole of issue 1143."
        )
    policy = POLICY_RS.read_text(encoding="utf-8")
    if "fn for_board(" not in policy:
        problems.append(
            f"{POLICY_RS.relative_to(ROOT)}: no `for_board` — the runner calls it."
        )
    elif "ivc_only()" not in for_board_body(policy):
        problems.append(
            f"{POLICY_RS.relative_to(ROOT)}: `for_board` no longer selects "
            f"`ivc_only()`.\n"
            f"    Then no in-tree path reaches it and the `#[allow(dead_code)]` "
            f"1143 removed is owed again."
        )
    return problems


def for_board_body(policy_src: str) -> str:
    """`fn for_board(`'s own body, and nothing after it.

    Slicing to the end of the FILE looked equivalent and was not: the doc
    comment above the function names `ivc_only`, and so does the test module
    below it, so a `for_board` mutated to return the platform default
    unchanged still "contained" the call. The mutation that removes the only
    behaviour this gate exists to protect was invisible to it — measured, not
    supposed. Ends at the method's closing brace (rustfmt puts it at four
    spaces), which is checked by the selftest in both directions.
    """
    head = policy_src.split("fn for_board(", 1)
    if len(head) == 1:
        return ""
    body = head[1]
    end = body.find("\n    }")
    return body[:end] if end != -1 else body


def check_rust_binding(manifests: list[pathlib.Path]) -> list[str]:
    """Rule 3 — the Rust selector's fact must be one the tree declares."""
    name = rust_ip_stack_name()
    if not name:
        return [
            f"{POLICY_RS.relative_to(ROOT)}: no `pub const IP_STACK: &str = \"…\"`.\n"
            f"    `LinkPolicy::for_board` selects `ivc_only()` from that constant; "
            f"without it the manifest rows and the Rust selector are two strings "
            f"nothing compares."
        ]
    declaring = [
        p.relative_to(ROOT).as_posix()
        for p in manifests
        if name in declared_capabilities(tomllib.loads(p.read_text(encoding="utf-8")))
    ]
    if not declaring:
        return [
            f"{POLICY_RS.relative_to(ROOT)}: `IP_STACK = \"{name}\"` is declared by "
            f"NO platform manifest.\n"
            f"    Absent is not false, so `LinkPolicy::for_board` can never see it "
            f"and `ivc_only()` is unreachable again — the state issue 1143 exists "
            f"to leave."
        ]
    return []


# ── selftest ────────────────────────────────────────────────────────────────

_LWIP_HEADER = """
#include "lwip/sockets.h"
typedef struct {
    int _socket;
} _z_sys_net_socket_t;
"""

_SPE_HEADER = """
typedef struct {
    union {
        uint8_t _placeholder;
    };
} _z_sys_net_socket_t;
"""

_DISPATCH = {"ZENOH_FREERTOS_LWIP": "lwip.h", "ZENOH_ORIN_SPE": "orin_spe.h"}
_HEADERS = {"lwip.h": _LWIP_HEADER, "orin_spe.h": _SPE_HEADER}

_GOOD = """
[capabilities]
ip_stack = true
[build.zenoh]
defines = ["ZENOH_GENERIC"]
defines_conditional = [
  { name = "ZENOH_FREERTOS_LWIP", when = { capability = { ip_stack = true } } },
  { name = "ZENOH_ORIN_SPE", when = { capability = { ip_stack = false } } },
]
extra_sources = [
  { path = "{src}/system/freertos/lwip/network.c", when = { capability = { ip_stack = true } } },
]
required_env = [
  { name = "FREERTOS_DIR", validate_subdir = "include" },
  { name = "LWIP_DIR", validate_subdir = "src/include", when = { capability = { ip_stack = true } } },
]
include_paths = ["{env:FREERTOS_DIR}/include"]
include_paths_conditional = [
  { path = "{env:LWIP_DIR}/src/include", when = { capability = { ip_stack = true } } },
]
"""


def self_test(quiet: bool = True) -> int:
    """Every rule, in both directions, on the normal path.

    The mutations are deliberately SHAPE-VALID: each one parses, each one is a
    plausible edit, and three of them differ from the good manifest only in a
    value. A gate whose negative controls all break structure proves it can
    reject garbage and nothing about whether it is wired to the right facts.
    """
    cases: list[tuple[str, str, str | None]] = [
        ("the good manifest passes", _GOOD, None),
        (
            "rule 1: a gate on an undeclared capability",
            _GOOD.replace("ip_stack = true\n[build", "heap = true\n[build", 1),
            "declares no `[capabilities] ip_stack`",
        ),
        (
            "rule 2: only the true arm",
            _GOOD.replace(
                '  { name = "ZENOH_ORIN_SPE", when = { capability = { ip_stack = false } } },\n',
                "",
            ),
            "none for `ip_stack = false`",
        ),
        (
            "rule 2: both arms claim the same value",
            _GOOD.replace("ZENOH_ORIN_SPE\", when = { capability = { ip_stack = false",
                          "ZENOH_ORIN_SPE\", when = { capability = { ip_stack = true"),
            "defines 2 conditional defines",
        ),
        (
            # THE wires-crossed case: names and values all still legal, the two
            # rows simply swapped which fact they answer to.
            "rule 4: the arms are swapped",
            _GOOD.replace("ZENOH_FREERTOS_LWIP", "__TMP__")
            .replace("ZENOH_ORIN_SPE", "ZENOH_FREERTOS_LWIP")
            .replace("__TMP__", "ZENOH_ORIN_SPE"),
            "declares a REAL `_z_sys_net_socket_t`",
        ),
        (
            # Rule 7's own wires-crossed case, and the one rule 4 cannot see:
            # the DEFINES are untouched and correct, only the TU moved.
            "rule 7: the netstack TU on the other arm",
            _GOOD.replace(
                'lwip/network.c", when = { capability = { ip_stack = true } }',
                'lwip/network.c", when = { capability = { ip_stack = false } }',
            ),
            "netstack define arm is `ip_stack = true`",
        ),
        (
            "rule 7: the netstack TU ungated (issue 1143 as filed)",
            _GOOD.replace(
                'lwip/network.c", when = { capability = { ip_stack = true } } }',
                'lwip/network.c" }',
            ),
            "UNCONDITIONALLY",
        ),
        (
            # Rule 8, the `required_env` half: a board with no IP stack would
            # be made to produce an `LWIP_DIR` it has no use for.
            "rule 8: the netstack env var required on the other arm",
            _GOOD.replace(
                'src/include", when = { capability = { ip_stack = true } } },\n  { name',
                'src/include", when = { capability = { ip_stack = false } } },\n  { name',
            ).replace(
                '{ name = "LWIP_DIR", validate_subdir = "src/include", '
                "when = { capability = { ip_stack = true } } }",
                '{ name = "LWIP_DIR", validate_subdir = "src/include", '
                "when = { capability = { ip_stack = false } } }",
            ),
            "inconsistent conditions",
        ),
        (
            # Rule 8, the include-root half: the header dir left unconditional
            # while the TU that needs it is gated.
            "rule 8: the netstack include root ungated",
            _GOOD.replace(
                'include_paths = ["{env:FREERTOS_DIR}/include"]',
                'include_paths = ["{env:FREERTOS_DIR}/include", "{env:LWIP_DIR}/src/include"]',
            ),
            "inconsistent conditions",
        ),
        (
            "rule 5: a netstack source with the fact undeclared",
            _GOOD.replace("[capabilities]\nip_stack = true\n", "[capabilities]\n"),
            "compiles {src}/system/freertos/lwip/network.c",
        ),
    ]

    bad = 0
    for name, text, want in cases:
        doc = tomllib.loads(text)
        got = check_manifest(
            "selftest.toml", doc, _DISPATCH, _HEADERS.get, "ip_stack"
        )
        joined = "\n".join(got)
        if want is None and got:
            bad += 1
            print(f"SELFTEST FAIL: {name}: expected clean, got:\n{joined}", file=sys.stderr)
        elif want is not None and want not in joined:
            bad += 1
            print(
                f"SELFTEST FAIL: {name}: expected a problem naming {want!r}, got:\n"
                f"{joined or '  (nothing)'}",
                file=sys.stderr,
            )
        elif not quiet:
            print(f"  ok: {name}")

    # Rule 4's source of truth is the reader, so read it against real headers:
    # a placeholder classifier that answers `True` for everything would pass
    # every case above.
    real = [
        ("orin_spe.h", True),
        ("lwip.h", False),
        ("freertos_plus_tcp.h", False),
        ("unix.h", False),
    ]
    for header, want_placeholder in real:
        path = PLATFORM_INCLUDE_ROOT / "zenoh-pico/system/platform"
        found = list(path.rglob(header))
        if not found:
            continue  # submodule not checked out; the tree scan reports that
        got = socket_is_placeholder(found[0].read_text(encoding="utf-8"))
        if got != want_placeholder:
            bad += 1
            print(
                f"SELFTEST FAIL: {header}: socket_is_placeholder said {got}, "
                f"expected {want_placeholder}",
                file=sys.stderr,
            )
        elif not quiet:
            print(f"  ok: {header} placeholder={got}")

    # Rule 6's slicer, in both directions. The `ivc_only` in the doc comment
    # and in the tests must NOT count; the one in the body must.
    live = (
        "    /// selects [`LinkPolicy::ivc_only`].\n"
        "    pub fn for_board(a: Self, c: &Caps) -> Self {\n"
        "        match c.get(IP_STACK) {\n"
        "            Some(false) => Self::ivc_only(),\n"
        "            _ => a,\n"
        "        }\n"
        "    }\n"
        "}\n"
        "mod tests { fn t() { LinkPolicy::ivc_only(); } }\n"
    )
    dead = live.replace("Some(false) => Self::ivc_only(),", "Some(false) => a,")
    for label, src, want in (("live", live, True), ("neutered", dead, False)):
        got = "ivc_only()" in for_board_body(src)
        if got != want:
            bad += 1
            print(
                f"SELFTEST FAIL: for_board_body({label}): saw ivc_only={got}, "
                f"expected {want}",
                file=sys.stderr,
            )
        elif not quiet:
            print(f"  ok: for_board_body {label}")

    if bad:
        print(
            "check-capability-conditionals: SELFTEST FAILED — the check cannot be "
            "trusted about the tree until it is right about these.",
            file=sys.stderr,
        )
    elif not quiet:
        print(f"check-capability-conditionals selftest: OK ({len(cases) + len(real)} case(s))")
    return 1 if bad else 0


def main() -> int:
    manifests = platform_manifests(ROOT)
    if not manifests:
        print(
            "check-capability-conditionals: no packages/platform/*/nros-platform.toml "
            "found — refusing to report OK over an empty scan.",
            file=sys.stderr,
        )
        return 2

    if PLATFORM_DISPATCH.is_file():
        dispatch = dispatch_map(PLATFORM_DISPATCH.read_text(encoding="utf-8"))

        def header_text(rel: str) -> str | None:
            # The dispatch chain spells its includes relative to `include/`.
            p = PLATFORM_INCLUDE_ROOT / rel
            return p.read_text(encoding="utf-8") if p.is_file() else None

    else:
        # zenoh-pico is a submodule; without it rule 4 has no source of truth.
        # Say so rather than passing over it (issue 0702).
        print(
            f"check-capability-conditionals: {PLATFORM_DISPATCH.relative_to(ROOT)} "
            f"is missing — run `git submodule update --init` for the zenoh-pico "
            f"submodule. The socket-ABI rule cannot be checked without it.",
            file=sys.stderr,
        )
        return 2

    problems = check_rust_binding(manifests) + check_selector_is_reached()
    for path in manifests:
        doc = tomllib.loads(path.read_text(encoding="utf-8"))
        problems += check_manifest(
            path.relative_to(ROOT).as_posix(),
            doc,
            dispatch,
            header_text,
            rust_ip_stack_name() or "ip_stack",
        )

    if problems:
        print("check-capability-conditionals: FAILED\n", file=sys.stderr)
        for p in problems:
            print(f"  {p}\n", file=sys.stderr)
        return 1

    gated = sum(
        len(capability_matchers(b))
        for path in manifests
        for b in build_blocks(tomllib.loads(path.read_text(encoding="utf-8"))).values()
    )
    print(
        f"check-capability-conditionals: OK "
        f"({len(manifests)} platform manifest(s), {gated} capability-gated row(s), "
        f"IP_STACK = {rust_ip_stack_name()!r})"
    )
    return 0


if __name__ == "__main__":
    if "--self-test" in sys.argv or "--selftest" in sys.argv:
        sys.exit(self_test(quiet=False))
    # The negative control runs BEFORE the tree is inspected.
    if self_test() != 0:
        sys.exit(2)
    sys.exit(main())
