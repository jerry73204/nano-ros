#!/usr/bin/env python3
"""The named QoS presets match upstream, field by field — phase-428 W10.

WHAT WENT WRONG

Upstream's seven `rmw_qos_profile_*` constants (plus `rcl_action`'s status
profile) are hand-transcribed in FOUR places in this tree — `nros-rmw`'s
`traits.rs`, `nros::qos`, `nros/qos.hpp` and `rmw_entity.h`'s
`NROS_RMW_QOS_PROFILE_*` macros — and until phase-428 nothing bound them to
each other or to upstream. Comments claimed parity; nothing measured it.

`QOS_PROFILE_PARAMETER_EVENTS` was `KeepAll, depth 0` where upstream is
`KEEP_LAST, 1000`, directly beneath a comment reading *"matches
rmw_qos_profile_parameter_events"*, with a unit test asserting the wrong value.
Issue 0793 had already caught the sibling profile (`PARAMETERS` claiming
TRANSIENT_LOCAL), fixed that one, and stopped — the F2 shape: a fix landing
where the symptom was seen. The C++ surface was correct the whole time, so the
two languages shipped different profiles under one name.

Not cosmetic. Both DDS backends map KEEP_ALL faithfully, and a RELIABLE writer
with KEEP_ALL BLOCKS when history fills, where KEEP_LAST(1000) overwrites. That
inverts behaviour rather than weakening it.

WHAT THIS CHECKS

1. The SSoT table (`qos_profiles! { ... }`, fenced by `nros-qos-table:
   BEGIN/END` in `traits.rs`) parses, and every row declares a PROVENANCE: an
   upstream symbol, or `nros: <why>` for a preset we invented.
2. Every upstream symbol in the record is claimed by exactly one row, or is
   declared absent with a reason.
3. Every upstream-mirroring row equals the record on all nine upstream fields.
4. A mismatch is tolerated ONLY when a `nros-qos-deviation:` line covers that
   (profile, field) AND pins BOTH values — and the pinned values are
   re-measured against the table and the record on every run.
5. A deviation covering nothing is an error. That is the issue-0743 class: a
   stale exemption reads as tracked debt while being inert.
6. No `QoSProfile` preset is defined outside the fence.
7. Where a ROS install is present, the record is RE-EXTRACTED from its headers
   and must match — so the checked-in file cannot rot on the machines that can
   tell.

WHY A CHECKED-IN RECORD AND NOT JUST THE HEADERS

The fast gate lane runs on hosts with no ROS (every embedded CI leg, every
fresh clone). A gate that silently degrades to "no ROS, nothing to check"
reports a pass it never established — the vacuous-test class. So upstream's
values are recorded in `docs/reference/rmw-qos-profiles.txt` with provenance,
exactly as `docs/reference/rmw-implementation-contract.txt` records the symbol
contract, and the headers become a CONFIRMATION step rather than a
precondition.

Usage::

    check-qos-profile-ssot.py                  # the gate (+ its selftest)
    check-qos-profile-ssot.py --write-upstream # regenerate the record (needs ROS)
    check-qos-profile-ssot.py --verbose-selftest
"""

from __future__ import annotations

import copy
import glob
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TRAITS = ROOT / "packages" / "core" / "nros-rmw" / "src" / "traits.rs"
RECORD = ROOT / "docs" / "reference" / "rmw-qos-profiles.txt"

# The nine fields upstream's `rmw_qos_profile_t` declares, in DECLARATION
# order (`rmw/include/rmw/types.h`). Order is load-bearing twice: the SSoT rows
# are written in it so a row reads against the header positionally, and the
# header extractor below consumes a brace initialiser POSITIONALLY.
UPSTREAM_FIELDS = (
    "history",
    "depth",
    "reliability",
    "durability",
    "deadline_ms",
    "lifespan_ms",
    "liveliness_kind",
    "liveliness_lease_ms",
    "avoid_ros_namespace_conventions",
)

# `tx_express` is ours (phase-282 / issue 0145), a transport hint with no
# upstream counterpart. It is not compared against the record; it is required
# to be false in an upstream-mirroring row, because a preset named after an
# upstream constant must not carry an extension the constant does not have.
NROS_ONLY_FIELDS = ("tx_express",)

ENUM_FIELDS = ("history", "reliability", "durability", "liveliness_kind")
INT_FIELDS = ("depth", "deadline_ms", "lifespan_ms", "liveliness_lease_ms")
BOOL_FIELDS = ("avoid_ros_namespace_conventions",)

# Rust variant -> the canonical (upstream) spelling.
#
# `QoSLivelinessPolicy::None` maps to SYSTEM_DEFAULT deliberately: the two
# collapsed onto discriminant 0 in phase-376 W5/B2, and `None` lowers to
# `NROS_RMW_LIVELINESS_SYSTEM_DEFAULT` across the C ABI. That is the one
# non-obvious entry here; every other row is a spelling change.
RUST_TO_CANONICAL = {
    "history": {
        "SystemDefault": "SYSTEM_DEFAULT",
        "KeepLast": "KEEP_LAST",
        "KeepAll": "KEEP_ALL",
    },
    "reliability": {
        "SystemDefault": "SYSTEM_DEFAULT",
        "Reliable": "RELIABLE",
        "BestEffort": "BEST_EFFORT",
    },
    "durability": {
        "SystemDefault": "SYSTEM_DEFAULT",
        "Volatile": "VOLATILE",
        "TransientLocal": "TRANSIENT_LOCAL",
    },
    "liveliness_kind": {
        "None": "SYSTEM_DEFAULT",
        "Automatic": "AUTOMATIC",
        "ManualByNode": "MANUAL_BY_NODE",
        "ManualByTopic": "MANUAL_BY_TOPIC",
    },
}

# Upstream enumerator -> canonical spelling, for the header extractor.
C_TO_CANONICAL = {
    "RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT": "SYSTEM_DEFAULT",
    "RMW_QOS_POLICY_HISTORY_KEEP_LAST": "KEEP_LAST",
    "RMW_QOS_POLICY_HISTORY_KEEP_ALL": "KEEP_ALL",
    "RMW_QOS_POLICY_HISTORY_UNKNOWN": "UNKNOWN",
    "RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT": "SYSTEM_DEFAULT",
    "RMW_QOS_POLICY_RELIABILITY_RELIABLE": "RELIABLE",
    "RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT": "BEST_EFFORT",
    "RMW_QOS_POLICY_RELIABILITY_UNKNOWN": "UNKNOWN",
    "RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT": "SYSTEM_DEFAULT",
    "RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL": "TRANSIENT_LOCAL",
    "RMW_QOS_POLICY_DURABILITY_VOLATILE": "VOLATILE",
    "RMW_QOS_POLICY_DURABILITY_UNKNOWN": "UNKNOWN",
    "RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT": "SYSTEM_DEFAULT",
    "RMW_QOS_POLICY_LIVELINESS_AUTOMATIC": "AUTOMATIC",
    "RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE": "MANUAL_BY_NODE",
    "RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC": "MANUAL_BY_TOPIC",
    "RMW_QOS_POLICY_LIVELINESS_UNKNOWN": "UNKNOWN",
}

# Alternatives the selftest draws mutants from.
MUTANTS = {
    "history": ["KEEP_ALL", "KEEP_LAST", "SYSTEM_DEFAULT"],
    "reliability": ["BEST_EFFORT", "RELIABLE", "SYSTEM_DEFAULT"],
    "durability": ["TRANSIENT_LOCAL", "VOLATILE", "SYSTEM_DEFAULT"],
    "liveliness_kind": ["MANUAL_BY_TOPIC", "AUTOMATIC", "SYSTEM_DEFAULT"],
    "depth": [42, 7, 0],
    "deadline_ms": [500, 1],
    "lifespan_ms": [500, 1],
    "liveliness_lease_ms": [500, 1],
    "avoid_ros_namespace_conventions": [True, False],
}

TABLE_BEGIN = "// nros-qos-table: BEGIN"
TABLE_END = "// nros-qos-table: END"

ROW_HEAD = re.compile(r"^\s{4}([A-Z][A-Z0-9_]*)\s*:\s*\"([^\"]*)\"\s*=\s*\{\s*$")
FIELD = re.compile(r"^\s{8}([a-z_]+)\s*:\s*([^,]+),\s*$")
DEPTH_CONST = re.compile(r"^pub const DEPTH_SYSTEM_DEFAULT: u32 = (\d+);", re.M)

DEVIATION = re.compile(r"^//\s*nros-qos-deviation:\s*(.+?)\s*$", re.M)
ABSENT = re.compile(r"^//\s*nros-qos-absent:\s*(.+?)\s*$", re.M)
KEYVAL = re.compile(r"([a-z_]+)=(\"[^\"]*\"|\S+)")

# A tracking reference must be well-formed. Filesystem RESOLUTION of an issue
# id is phase-428 W11's work item and covers every authored table at once, not
# just this one; do not fork a second, weaker resolver here.
REF_OK = re.compile(r"^(issue \d{4}|phase-\d+ W\d+)$")

# A preset defined outside the fence. `pub const NAME: Self` / `: QoSProfile`
# is the only shape a preset has ever taken in this crate.
#
# `Self` needs the enclosing `impl` to disambiguate: `QoSPolicyMask` declares
# thirteen `pub const NAME: Self` bit constants in the same file, and matching
# on the spelling alone reported every one of them as a stray preset. Which is
# the useful failure — the first version of this rule was written from the
# regex outwards and would have been "green" only because nobody read it.
IMPL_OPEN = re.compile(r"^impl(?:<[^>]*>)?\s+([A-Za-z_][A-Za-z0-9_]*)")
SELF_PRESET = re.compile(r"^\s*pub const ([A-Z][A-Z0-9_]*)\s*:\s*Self\s*=")
NAMED_PRESET = re.compile(r"^\s*pub const ([A-Z][A-Z0-9_]*)\s*:\s*QoSProfile\s*=")

UPSTREAM_SYMBOL = re.compile(r"^(?:rmw|rcl|rcl_action)_[a-z0-9_]+$")


class Fail(Exception):
    pass


# --------------------------------------------------------------------------
# Parsing the Rust SSoT
# --------------------------------------------------------------------------


def parse_table(text):
    """{name: {'provenance': str, 'fields': {...}, 'line': int}} from the fence."""
    lines = text.split("\n")
    try:
        begin = next(i for i, l in enumerate(lines) if l.strip() == TABLE_BEGIN)
        end = next(i for i, l in enumerate(lines) if l.strip() == TABLE_END)
    except StopIteration:
        raise Fail(
            f"{TABLE_BEGIN} / {TABLE_END} fence not found. The QoS SSoT table is "
            "the thing this gate checks; losing the fence is not a reason to pass."
        )
    if end <= begin:
        raise Fail("nros-qos-table END precedes BEGIN")

    m = DEPTH_CONST.search(text)
    if not m:
        raise Fail("`pub const DEPTH_SYSTEM_DEFAULT: u32 = N;` not found")
    depth_sentinel = int(m.group(1))

    rows, name, fields, head_line = {}, None, None, 0
    for i in range(begin + 1, end):
        line = lines[i]
        head = ROW_HEAD.match(line)
        if head:
            if name is not None:
                raise Fail(f"{TRAITS.name}:{i + 1}: row {head.group(1)} opened inside {name}")
            name, fields, head_line = head.group(1), {}, i + 1
            rows[name] = {"provenance": head.group(2), "fields": fields, "line": head_line}
            continue
        if name is None:
            continue
        if line.strip() == "};":
            missing = [f for f in UPSTREAM_FIELDS + NROS_ONLY_FIELDS if f not in fields]
            if missing:
                raise Fail(
                    f"{TRAITS.name}:{head_line}: {name} states no {', '.join(missing)}.\n"
                    "      Every field is stated at the authoring site — that is the "
                    "point of the table.\n"
                    "      The `Self::build()` helper it replaced hid five of ten."
                )
            name, fields = None, None
            continue
        f = FIELD.match(line)
        if f:
            key, raw = f.group(1), f.group(2).strip()
            if key in fields:
                raise Fail(f"{TRAITS.name}:{i + 1}: {name} states {key} twice")
            fields[key] = _rust_value(key, raw, depth_sentinel, f"{TRAITS.name}:{i + 1}")

    if name is not None:
        raise Fail(f"{TRAITS.name}:{head_line}: row {name} is not closed")
    if not rows:
        raise Fail("the QoS table fence is empty")
    return rows


def _rust_value(key, raw, depth_sentinel, where):
    if key in ENUM_FIELDS:
        table = RUST_TO_CANONICAL[key]
        if raw not in table:
            raise Fail(f"{where}: {key} = {raw!r} is not a known variant ({sorted(table)})")
        return table[raw]
    if key in INT_FIELDS:
        if raw == "DEPTH_SYSTEM_DEFAULT":
            return depth_sentinel
        if not re.fullmatch(r"\d+", raw):
            raise Fail(f"{where}: {key} = {raw!r} is not an integer literal")
        return int(raw)
    if key in BOOL_FIELDS or key in NROS_ONLY_FIELDS:
        if raw not in ("true", "false"):
            raise Fail(f"{where}: {key} = {raw!r} is not a bool literal")
        return raw == "true"
    raise Fail(f"{where}: {key!r} is not a QoSProfile field this gate knows")


def _keyvals(rest):
    out = {}
    for k, v in KEYVAL.findall(rest):
        out[k] = v[1:-1] if v.startswith('"') else v
    return out


def parse_deviations(text):
    out = []
    for m in DEVIATION.finditer(text):
        kv = _keyvals(m.group(1))
        missing = [k for k in ("profile", "field", "ours", "upstream", "ref") if k not in kv]
        if missing:
            raise Fail(
                f"nros-qos-deviation is missing {', '.join(missing)}: {m.group(1)!r}\n"
                "      A deviation names the field and BOTH values, or it is a reason\n"
                "      string against a membership test — the shape phase-428 F4 found\n"
                "      drifted in `ARG_DEVIATIONS`."
            )
        if not REF_OK.match(kv["ref"]):
            raise Fail(
                f"nros-qos-deviation ref={kv['ref']!r} is not `issue NNNN` or `phase-NNN WN`"
            )
        if kv["field"] not in UPSTREAM_FIELDS + NROS_ONLY_FIELDS:
            raise Fail(f"nros-qos-deviation names unknown field {kv['field']!r}")
        out.append(kv)
    return out


def parse_absences(text):
    out = []
    for m in ABSENT.finditer(text):
        kv = _keyvals(m.group(1))
        if "upstream" not in kv or "ref" not in kv:
            raise Fail(f"nros-qos-absent needs upstream= and ref=: {m.group(1)!r}")
        if not REF_OK.match(kv["ref"]):
            raise Fail(f"nros-qos-absent ref={kv['ref']!r} is not `issue NNNN` or `phase-NNN WN`")
        out.append(kv)
    return out


def stray_presets(text):
    """Preset definitions outside the fence — the drift this gate exists to stop."""
    lines = text.split("\n")
    begin = next((i for i, l in enumerate(lines) if l.strip() == TABLE_BEGIN), None)
    end = next((i for i, l in enumerate(lines) if l.strip() == TABLE_END), None)
    out, impl_of = [], None
    for i, line in enumerate(lines):
        opened = IMPL_OPEN.match(line)
        if opened:
            impl_of = opened.group(1)
        if begin is not None and end is not None and begin <= i <= end:
            continue
        m = NAMED_PRESET.match(line)
        if not m and impl_of == "QoSProfile":
            m = SELF_PRESET.match(line)
        if m:
            out.append((i + 1, m.group(1)))
    return out


# --------------------------------------------------------------------------
# The upstream record
# --------------------------------------------------------------------------


def parse_record(text):
    out = {}
    for raw in text.split("\n"):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 1 + len(UPSTREAM_FIELDS):
            raise Fail(
                f"{RECORD.name}: {len(parts)} columns, expected "
                f"{1 + len(UPSTREAM_FIELDS)}: {line!r}"
            )
        sym, values = parts[0], parts[1:]
        row = {}
        for field, v in zip(UPSTREAM_FIELDS, values):
            if field in INT_FIELDS:
                row[field] = int(v)
            elif field in BOOL_FIELDS:
                if v not in ("true", "false"):
                    raise Fail(f"{RECORD.name}: {sym}.{field} = {v!r} is not a bool")
                row[field] = v == "true"
            else:
                if v not in set(C_TO_CANONICAL.values()):
                    raise Fail(f"{RECORD.name}: {sym}.{field} = {v!r} is not a canonical value")
                row[field] = v
        out[sym] = row
    if not out:
        raise Fail(f"{RECORD.name} records no profile")
    return out


def format_record(rows, provenance_lines):
    widths = [max(len(s) for s in rows)] + [
        max(len(_fmt(r[f])) for r in rows.values()) for f in UPSTREAM_FIELDS
    ]
    header = "# columns: profile " + " ".join(UPSTREAM_FIELDS)
    body = []
    for sym in sorted(rows):
        cells = [sym.ljust(widths[0])] + [
            _fmt(rows[sym][f]).ljust(w) for f, w in zip(UPSTREAM_FIELDS, widths[1:])
        ]
        body.append(" ".join(cells).rstrip())
    return "\n".join(provenance_lines + [header] + body) + "\n"


def _fmt(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    return str(v)


# --------------------------------------------------------------------------
# Extracting from a real ROS install
# --------------------------------------------------------------------------


def ros_include_dirs():
    """Prefixes to look for `rmw/` and `rcl_action/` headers under.

    `AMENT_PREFIX_PATH` first (it names the install the user actually sourced),
    then `$ROS_DISTRO` under `/opt/ros`. A bare `/opt/ros/*` glob is
    deliberately last and only when nothing else answered — same reasoning as
    `nros_zenohd_bin`: guessing an install nobody chose is how you measure the
    wrong thing.
    """
    seen, out = set(), []

    def add(p):
        if p and p not in seen and os.path.isdir(p):
            seen.add(p)
            out.append(p)

    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        add(os.path.join(prefix, "include"))
    distro = os.environ.get("ROS_DISTRO")
    if distro:
        add(f"/opt/ros/{distro}/include")
    if not out:
        for p in sorted(glob.glob("/opt/ros/*/include")):
            add(p)
    return out


def find_headers():
    """{'qos_profiles.h': path, 'default_qos.h': path, 'time.h':…, 'types.h':…}"""
    wanted = {
        "qos_profiles": "rmw/rmw/qos_profiles.h",
        "types": "rmw/rmw/types.h",
        "time": "rmw/rmw/time.h",
        "action": "rcl_action/rcl_action/default_qos.h",
    }
    found = {}
    for inc in ros_include_dirs():
        for key, rel in wanted.items():
            if key in found:
                continue
            p = os.path.join(inc, rel)
            if os.path.exists(p):
                found[key] = p
    return found


PROFILE_DEF = re.compile(
    r"static\s+const\s+rmw_qos_profile_t\s+([a-z_]+)\s*=\s*\{(.*?)\}\s*;", re.S
)


def extract_upstream(headers):
    """Parse the profile initialisers positionally. Raises Fail on anything odd."""
    if "types" not in headers or "time" not in headers:
        raise Fail("rmw/types.h and rmw/time.h are needed to resolve the sentinels")

    types_src = Path(headers["types"]).read_text(encoding="utf8")
    time_src = Path(headers["time"]).read_text(encoding="utf8")

    m = re.search(r"RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT\s*=\s*(\d+)", types_src)
    if not m:
        raise Fail("RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT not found in rmw/types.h")
    depth_sentinel = int(m.group(1))

    durations = {}
    for name in ("RMW_DURATION_UNSPECIFIED", "RMW_DURATION_INFINITE"):
        d = re.search(rf"#define\s+{name}\s*\{{\s*(-?\d+)LL?\s*,\s*(-?\d+)LL?\s*\}}", time_src)
        if not d:
            raise Fail(f"{name} not found in rmw/time.h")
        sec, nsec = int(d.group(1)), int(d.group(2))
        # Our u32 window fields are milliseconds with 0 meaning "off/infinite".
        # UNSPECIFIED is {0,0} and lands on 0; INFINITE is deliberately NOT
        # folded to 0 here — if upstream ever ships a profile using it, this
        # gate should fail loudly rather than call it equal to "off".
        durations[name] = sec * 1000 + nsec // 1_000_000
    aliases = {
        "RMW_QOS_DEADLINE_DEFAULT": durations["RMW_DURATION_UNSPECIFIED"],
        "RMW_QOS_LIFESPAN_DEFAULT": durations["RMW_DURATION_UNSPECIFIED"],
        "RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT": durations["RMW_DURATION_UNSPECIFIED"],
    }

    rows = {}
    for key in ("qos_profiles", "action"):
        if key not in headers:
            continue
        src = Path(headers[key]).read_text(encoding="utf8")
        for sym, body in PROFILE_DEF.findall(src):
            toks = [t.strip() for t in body.split(",")]
            toks = [t for t in toks if t and not t.startswith("//")]
            if len(toks) != len(UPSTREAM_FIELDS):
                raise Fail(
                    f"{sym}: {len(toks)} initialisers, expected {len(UPSTREAM_FIELDS)}.\n"
                    "      `rmw_qos_profile_t` gained or lost a field — the SSoT table's\n"
                    "      field list has to move with it, which is why this is fatal."
                )
            row = {}
            for field, tok in zip(UPSTREAM_FIELDS, toks):
                row[field] = _c_value(sym, field, tok, depth_sentinel, aliases)
            rows[sym] = row
    if not rows:
        raise Fail("no `static const rmw_qos_profile_t` definitions found")
    return rows


def _c_value(sym, field, tok, depth_sentinel, aliases):
    if field in ENUM_FIELDS:
        if tok not in C_TO_CANONICAL:
            raise Fail(f"{sym}.{field}: unknown enumerator {tok!r}")
        return C_TO_CANONICAL[tok]
    if field in INT_FIELDS:
        if tok in aliases:
            return aliases[tok]
        if tok == "RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT":
            return depth_sentinel
        if re.fullmatch(r"\d+", tok):
            return int(tok)
        raise Fail(f"{sym}.{field}: cannot resolve {tok!r} to a number")
    if field in BOOL_FIELDS:
        if tok not in ("true", "false"):
            raise Fail(f"{sym}.{field}: expected true/false, got {tok!r}")
        return tok == "true"
    raise Fail(f"{sym}: unexpected field {field!r}")


# --------------------------------------------------------------------------
# The comparison — the whole verdict lives here so the selftest can drive it
# --------------------------------------------------------------------------


def compare(rows, record, deviations, absences, strays=()):
    """Return a list of error strings. Empty list == the tree is consistent."""
    errs = []

    claimed = {}
    for name, row in sorted(rows.items(), key=lambda kv: kv[1]["line"]):
        prov = row["provenance"]
        if prov.startswith("nros:"):
            if not prov[len("nros:"):].strip():
                errs.append(f"{name}: provenance `nros:` carries no reason")
            continue
        if not UPSTREAM_SYMBOL.match(prov):
            errs.append(
                f"{name}: provenance {prov!r} is neither an upstream symbol nor "
                "`nros: <why>`"
            )
            continue
        if prov not in record:
            errs.append(
                f"{name}: claims upstream `{prov}`, which {RECORD.name} does not record"
            )
            continue
        if prov in claimed:
            errs.append(f"{name}: upstream `{prov}` is already claimed by {claimed[prov]}")
            continue
        claimed[prov] = name

    declared_absent = {a["upstream"] for a in absences}
    for sym in sorted(record):
        if sym not in claimed and sym not in declared_absent:
            errs.append(
                f"{sym}: recorded upstream but no preset mirrors it and no "
                "`nros-qos-absent:` line declares why"
            )
    for sym in sorted(declared_absent):
        if sym not in record:
            errs.append(f"nros-qos-absent names `{sym}`, which {RECORD.name} does not record")
        elif sym in claimed:
            errs.append(
                f"nros-qos-absent names `{sym}`, but {claimed[sym]} mirrors it. "
                "A stale exemption reads as tracked debt while being inert (issue 0743)."
            )

    # A deviation is LIVE only if it covers a mismatch that actually happened.
    used = set()

    for prov, name in sorted(claimed.items()):
        ours, theirs = rows[name]["fields"], record[prov]
        for field in UPSTREAM_FIELDS:
            if ours[field] == theirs[field]:
                continue
            dev = _covering(deviations, name, field)
            if dev is None:
                errs.append(
                    f"{name}.{field}: ours {_fmt(ours[field])}, upstream `{prov}` "
                    f"{_fmt(theirs[field])}\n"
                    f"      traits.rs:{rows[name]['line']} — fix it, or declare a\n"
                    "      `nros-qos-deviation:` pinning both values and a tracking ref."
                )
                continue
            used.add(id(dev))
            if dev["ours"] != _fmt(ours[field]):
                errs.append(
                    f"{name}.{field}: deviation pins ours={dev['ours']}, table says "
                    f"{_fmt(ours[field])}"
                )
            if dev["upstream"] != _fmt(theirs[field]):
                errs.append(
                    f"{name}.{field}: deviation pins upstream={dev['upstream']}, "
                    f"{RECORD.name} says {_fmt(theirs[field])}"
                )
        for field in NROS_ONLY_FIELDS:
            if ours[field] is not False and _covering(deviations, name, field) is None:
                errs.append(
                    f"{name}.{field}: a preset named after `{prov}` sets the nano-ros "
                    f"extension {field}={_fmt(ours[field])}, which upstream has no field for"
                )

    for dev in deviations:
        if id(dev) not in used:
            errs.append(
                f"nros-qos-deviation profile={dev['profile']} field={dev['field']}: "
                "covers no mismatch.\n"
                "      Either the code was fixed and the line was not deleted, or it\n"
                "      never described anything. A stale exemption is the issue-0743\n"
                "      class: inert while reading as tracked debt."
            )

    for line, name in strays:
        errs.append(
            f"traits.rs:{line}: `{name}` is a QoSProfile preset defined OUTSIDE the "
            f"{TABLE_BEGIN[3:]} fence.\n"
            "      Four hand-transcribed copies is what phase-428 W10 exists to end; a\n"
            "      fifth inside the SSoT crate is the same defect at zero distance."
        )

    return errs


def _covering(deviations, profile, field):
    for dev in deviations:
        if dev["field"] != field:
            continue
        if dev["profile"] == "*" or dev["profile"] == profile:
            return dev
    return None


# --------------------------------------------------------------------------
# Negative control — runs on the NORMAL path (check-gate-selftests)
# --------------------------------------------------------------------------


def _synthetic():
    """A minimal consistent world: one mirroring preset, one invention."""
    rows = {
        "QOS_PROFILE_DEFAULT": {
            "provenance": "rmw_qos_profile_default",
            "line": 1,
            "fields": {
                "history": "KEEP_LAST",
                "depth": 10,
                "reliability": "RELIABLE",
                "durability": "VOLATILE",
                "deadline_ms": 0,
                "lifespan_ms": 0,
                "liveliness_kind": "AUTOMATIC",
                "liveliness_lease_ms": 0,
                "avoid_ros_namespace_conventions": False,
                "tx_express": False,
            },
        },
        "QOS_PROFILE_CLOCK": {
            "provenance": "nros: /clock has no rmw constant",
            "line": 2,
            "fields": {
                "history": "KEEP_LAST",
                "depth": 1,
                "reliability": "BEST_EFFORT",
                "durability": "VOLATILE",
                "deadline_ms": 0,
                "lifespan_ms": 0,
                "liveliness_kind": "AUTOMATIC",
                "liveliness_lease_ms": 0,
                "avoid_ros_namespace_conventions": False,
                "tx_express": False,
            },
        },
    }
    record = {
        "rmw_qos_profile_default": {
            "history": "KEEP_LAST",
            "depth": 10,
            "reliability": "RELIABLE",
            "durability": "VOLATILE",
            "deadline_ms": 0,
            "lifespan_ms": 0,
            "liveliness_kind": "AUTOMATIC",
            "liveliness_lease_ms": 0,
            "avoid_ros_namespace_conventions": False,
        }
    }
    return rows, record


def self_test(verbose=False):
    """Prove the gate goes red for every drift it claims to catch.

    Two halves, and both matter. The SYNTHETIC half pins the comparison's
    semantics (a deviation that lies, a deviation that covers nothing, an
    unmirrored upstream symbol, a stray preset). The LIVE half mutates the
    REAL parsed table, one field of one profile at a time, and asserts a red —
    which is the only version that keeps working when someone edits the table,
    because a synthetic fixture drifts away from the thing it stands in for.
    """
    checks = 0

    def expect_red(label, *args):
        nonlocal checks
        errs = compare(*args)
        if not errs:
            raise Fail(f"selftest: {label} produced NO error — the gate cannot see it")
        checks += 1
        if verbose:
            print(f"  red as expected: {label}\n      {errs[0].splitlines()[0]}")

    def expect_green(label, *args):
        nonlocal checks
        errs = compare(*args)
        if errs:
            raise Fail(f"selftest: {label} should be clean, got:\n  " + "\n  ".join(errs))
        checks += 1

    rows, record = _synthetic()
    expect_green("the synthetic baseline", rows, record, [], [])

    # A field drifts, undeclared.
    bad = copy.deepcopy(rows)
    bad["QOS_PROFILE_DEFAULT"]["fields"]["depth"] = 1
    expect_red("an undeclared depth drift", bad, record, [], [])

    # A deviation that pins the WRONG `ours` — the F4 failure, where a reason
    # string sat against a membership test and nothing re-measured it.
    lying = [
        {
            "profile": "QOS_PROFILE_DEFAULT",
            "field": "depth",
            "ours": "10",
            "upstream": "10",
            "ref": "phase-428 W10",
        }
    ]
    expect_red("a deviation pinning values that do not match", bad, record, lying, [])

    honest = [
        {
            "profile": "QOS_PROFILE_DEFAULT",
            "field": "depth",
            "ours": "1",
            "upstream": "10",
            "ref": "phase-428 W10",
        }
    ]
    expect_green("an honest, live deviation", bad, record, honest, [])
    # ... and the SAME deviation against the unmutated table is stale.
    expect_red("a deviation that covers no mismatch", rows, record, honest, [])

    # An upstream symbol nothing mirrors and nothing declares absent.
    wider = dict(record)
    wider["rmw_qos_profile_unknown"] = dict(record["rmw_qos_profile_default"])
    expect_red("an unmirrored upstream symbol", rows, wider, [], [])
    expect_green(
        "the same symbol, declared absent",
        rows,
        wider,
        [],
        [{"upstream": "rmw_qos_profile_unknown", "ref": "phase-428 W10"}],
    )

    # A preset defined outside the fence.
    expect_red("a preset outside the fence", rows, record, [], [], [(99, "QOS_PROFILE_SNEAKY")])

    # The nano-ros extension set on a preset named after an upstream constant.
    ext = copy.deepcopy(rows)
    ext["QOS_PROFILE_DEFAULT"]["fields"]["tx_express"] = True
    expect_red("tx_express set on an upstream-mirroring preset", ext, record, [], [])

    # --- the LIVE half ---
    real_rows, real_record, real_devs, real_absences = _load()
    expect_green("the real tree", real_rows, real_record, real_devs, real_absences)

    for name, row in real_rows.items():
        if row["provenance"].startswith("nros:"):
            continue
        for field in UPSTREAM_FIELDS:
            current = row["fields"][field]
            dev = _covering(real_devs, name, field)
            forbidden = {current}
            if dev is not None:
                # Mutating INTO the declared deviation value would be covered
                # and legitimately green, so it is not a mutation this gate
                # should be asked to catch.
                forbidden.add(dev["ours"])
                forbidden.add(_fmt(current))
            mutant = next(
                (m for m in MUTANTS[field] if _fmt(m) not in {_fmt(f) for f in forbidden}),
                None,
            )
            if mutant is None:
                raise Fail(f"selftest: no mutant available for {name}.{field}")
            mutated = copy.deepcopy(real_rows)
            mutated[name]["fields"][field] = mutant
            expect_red(f"{name}.{field} -> {_fmt(mutant)}", mutated, real_record, real_devs,
                       real_absences)

    # And the record itself: if upstream's recorded value moves, we notice.
    #
    # Only for the symbols something MIRRORS. A record row nobody mirrors —
    # `rmw_qos_profile_unknown`, declared absent — has nothing to compare
    # against by construction, so demanding a red there would be demanding the
    # gate invent an opinion. The first version of this control did demand it
    # and went red on its own tree, which is the control working. That row is
    # not unwatched: on a host with ROS the record-vs-headers freshness check in
    # `main` reads every row, mirrored or not.
    mirrored = {
        r["provenance"] for r in real_rows.values() if not r["provenance"].startswith("nros:")
    }
    for sym in sorted(mirrored):
        mutated = copy.deepcopy(real_record)
        mutated[sym]["depth"] = mutated[sym]["depth"] + 7
        expect_red(f"{RECORD.name} {sym}.depth moved", real_rows, mutated, real_devs,
                   real_absences)

    # The header extractor has its own control: a mutated initialiser must not
    # silently parse to the old value.
    checks += _extractor_self_test(verbose)
    checks += _stray_self_test(verbose)

    if verbose:
        print(f"selftest: {checks} negative/positive controls passed")
    return checks


SYNTH_TYPES_H = """
enum {RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT = 0};
"""
SYNTH_TIME_H = """
#define RMW_DURATION_INFINITE {9223372036LL, 854775807LL}
#define RMW_DURATION_UNSPECIFIED {0LL, 0LL}
"""
SYNTH_QOS_H = """
static const rmw_qos_profile_t rmw_qos_profile_default =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  %DEPTH%,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};
"""


SYNTH_RS = f"""
impl QoSPolicyMask {{
    pub const RELIABILITY: Self = Self(1 << 0);
}}

{TABLE_BEGIN}
qos_profiles! {{
    QOS_PROFILE_DEFAULT: "rmw_qos_profile_default" = {{
        history: KeepLast,
    }};
}}
{TABLE_END}

impl QoSProfile {{
%STRAY%
}}
"""


def _stray_self_test(verbose):
    """The fence rule discriminates, in BOTH directions.

    A gate that reports every `pub const NAME: Self` is not enforcing the rule,
    it is failing to read Rust — and it would have fired thirteen times on
    `QoSPolicyMask` in this very file.
    """
    clean = stray_presets(SYNTH_RS.replace("%STRAY%", "    pub const fn new() {}"))
    if clean:
        raise Fail(f"selftest: false positive on a clean file: {clean}")

    for stray in (
        "    pub const SNEAKY: Self = Self { history: KeepAll };",
        "    pub const SNEAKY: QoSProfile = QoSProfile { history: KeepAll };",
    ):
        got = stray_presets(SYNTH_RS.replace("%STRAY%", stray))
        if [n for _, n in got] != ["SNEAKY"]:
            raise Fail(f"selftest: missed a stray preset ({stray!r}) — got {got}")

    # The real file: the fence is where every preset lives.
    live = stray_presets(TRAITS.read_text(encoding="utf8"))
    if live:
        raise Fail(f"selftest: {TRAITS.name} has strays the gate should already report: {live}")

    if verbose:
        print("  stray-preset controls passed")
    return 4


def _extractor_self_test(verbose):
    import tempfile

    with tempfile.TemporaryDirectory() as d:
        def write(name, text):
            p = os.path.join(d, name)
            Path(p).write_text(text, encoding="utf8")
            return p

        headers = {
            "types": write("types.h", SYNTH_TYPES_H),
            "time": write("time.h", SYNTH_TIME_H),
        }
        headers["qos_profiles"] = write("qos.h", SYNTH_QOS_H.replace("%DEPTH%", "10"))
        got = extract_upstream(headers)["rmw_qos_profile_default"]
        if got["depth"] != 10 or got["liveliness_kind"] != "SYSTEM_DEFAULT":
            raise Fail(f"selftest: extractor misread the synthetic header: {got}")

        headers["qos_profiles"] = write("qos2.h", SYNTH_QOS_H.replace("%DEPTH%", "1000"))
        moved = extract_upstream(headers)["rmw_qos_profile_default"]
        if moved["depth"] != 1000:
            raise Fail("selftest: extractor did not follow a changed depth")

        # A dropped field must be fatal, not silently short.
        headers["qos_profiles"] = write(
            "qos3.h", SYNTH_QOS_H.replace("%DEPTH%", "10").replace(
                "  RMW_QOS_LIFESPAN_DEFAULT,\n", ""
            )
        )
        try:
            extract_upstream(headers)
        except Fail:
            pass
        else:
            raise Fail("selftest: extractor accepted a profile with 8 initialisers")

    if verbose:
        print("  extractor controls passed")
    return 3


# --------------------------------------------------------------------------


def _load():
    text = TRAITS.read_text(encoding="utf8")
    rows = parse_table(text)
    deviations = parse_deviations(text)
    absences = parse_absences(text)
    if not RECORD.exists():
        raise Fail(
            f"{RECORD} is missing. It is TRACKED, so its absence is a path bug, not an\n"
            "  empty check. Regenerate on a host with ROS:\n"
            "    python3 scripts/check-qos-profile-ssot.py --write-upstream"
        )
    record = parse_record(RECORD.read_text(encoding="utf8"))
    return rows, record, deviations, absences


def _provenance_header(headers):
    import datetime

    distro = os.environ.get("ROS_DISTRO", "unknown")
    return [
        "# The upstream QoS profile constants — every field, verbatim.",
        "#",
        "# Derived, not asserted: the brace initialisers of every",
        "# `static const rmw_qos_profile_t` in the headers below, with the duration and",
        "# depth sentinels resolved from `rmw/time.h` and `rmw/types.h`. Recorded here",
        "# because the fast gate lane runs on hosts with no ROS, and a gate that",
        "# degrades to `nothing to check` reports a pass it never established.",
        "#",
        "# Sources:",
    ] + [
        f"#   {headers[k]}" for k in sorted(headers)
    ] + [
        "#",
        "# Windows are milliseconds; RMW_DURATION_UNSPECIFIED ({0,0}) lands on 0, which",
        "# is also nano-ros's `off`. `liveliness` is upstream's own spelling —",
        "# `QoSLivelinessPolicy::None` is nano-ros's name for SYSTEM_DEFAULT (phase-376",
        "# W5/B2 collapsed the two onto discriminant 0).",
        "#",
        "# Regenerate where a ROS install exists:",
        "#   python3 scripts/check-qos-profile-ssot.py --write-upstream",
        "#",
        f"# Generated {datetime.date.today().isoformat()}, ROS 2 {distro}.",
    ]


def main():
    verbose = any(a.startswith("--verbose-self") for a in sys.argv[1:])

    try:
        if "--write-upstream" in sys.argv:
            headers = find_headers()
            if "qos_profiles" not in headers:
                raise Fail(
                    "no ROS install found. Source one (`source /opt/ros/<distro>/setup.bash`) "
                    "and retry."
                )
            rows = extract_upstream(headers)
            RECORD.write_text(format_record(rows, _provenance_header(headers)), encoding="utf8")
            print(f"wrote {RECORD.relative_to(ROOT)} — {len(rows)} profile(s)")
            return 0

        # The negative control runs on the NORMAL path, not behind a flag —
        # `check-gate-selftests` requires it, and for the reason that gate
        # states: a control nobody runs decays into a comment.
        self_test(verbose=verbose)

        rows, record, deviations, absences = _load()
        errs = compare(rows, record, deviations, absences, stray_presets(
            TRAITS.read_text(encoding="utf8")
        ))

        # Where a ROS install IS present, the checked-in record must still be
        # what the headers say. Absent one, the record stands on its recorded
        # provenance — but it can never quietly rot on a machine that could
        # have told us.
        headers = find_headers()
        if "qos_profiles" in headers:
            live = extract_upstream(headers)
            for sym in sorted(set(live) | set(record)):
                if sym not in record:
                    errs.append(
                        f"{sym}: defined by {headers.get('qos_profiles')} and absent from "
                        f"{RECORD.name} — regenerate with --write-upstream"
                    )
                elif sym not in live:
                    continue  # a distro that dropped it; the record is the contract
                else:
                    for field in UPSTREAM_FIELDS:
                        if live[sym][field] != record[sym][field]:
                            errs.append(
                                f"{RECORD.name} is STALE: {sym}.{field} records "
                                f"{_fmt(record[sym][field])}, the installed headers say "
                                f"{_fmt(live[sym][field])} — regenerate with --write-upstream"
                            )
        elif verbose:
            print("no ROS install found; checked against the recorded values only")

    except Fail as exc:
        print(f"check-qos-profile-ssot: {exc}", file=sys.stderr)
        return 1

    if errs:
        print(f"check-qos-profile-ssot: {len(errs)} problem(s):\n", file=sys.stderr)
        for e in errs:
            print(f"  - {e}", file=sys.stderr)
        print(
            "\n  The SSoT is the `qos_profiles!` table in "
            "packages/core/nros-rmw/src/traits.rs;\n"
            f"  upstream's values are recorded in {RECORD.relative_to(ROOT)}.",
            file=sys.stderr,
        )
        return 1

    print(f"check-qos-profile-ssot: OK ({len(rows)} presets, {len(record)} upstream constants)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
