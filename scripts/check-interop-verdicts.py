#!/usr/bin/env python3
"""A live-peer cell that has never produced a VERDICT — issue 1127, phase-433 W5.

`nros_tests::interop::CELLS` declares the cells whose subject is a LIVE ROS 2
peer. Phase-433 W4 closed "nothing can be AIMED at this cell"
(`check-interop-cell-runners`). This closes the half that outlives it:

    a `skip!` is not a verdict, and nothing distinguishes a cell that has
    skipped on every host since the day it was written from one that passes.

`_rewrite-skipped-junit` turns the skip into `<skipped>`, every consumer counts
that as not-a-failure, and the lane is green. Six checks surround such a cell
(`matrix_fixture_coverage.rs` G1-G5, plus W4's runner gate) and all six are
statements about DECLARATIONS. This is issue 0445's absorbing-verdict class one
level up: there a STALE fixture replaces the runtime's answer with a message
that explains itself, and 0444 hid behind it for exactly that long; here a green
tick does the same job. W1 measured what it costs — `graph_interop` had been
green since it was written, and the first time it was aimed at a live peer,
zenoh passed and Cyclone failed nine of twelve slots (issue 1137).

WHAT IS RECORDED, AND WHY ABSENCE IS THE DEFAULT

`.config/interop-verdicts.toml` holds one entry per cell that has actually
produced a result: date, the case names, pass or fail, the command, where it
ran, the observed output. **A cell with no entry has NEVER produced a verdict**,
and that is the default on purpose. A new cell needs no ledger line and starts
life honestly unproven; nothing can be satisfied by a cell that never runs,
which is the property the mechanism has to have. It also keeps the file out of
the merge queue's way: it holds only POSITIVE claims, written at most once per
cell — 17 edits ever, not one per pull request, so it is not the committed
generated file that serialised the queue in issues 0883/0884.

THE LOAD-BEARING RULE: A BINDING TEST IS NOT EVIDENCE

Every one of the eleven live-peer binaries also carries a test that calls
`interop::assert_test_bound` and nothing else (`cases_bound_to_interop_cells`
and its siblings). Those pass on any host, with no peer, forever. So "did this
binary produce a non-skip result" is TRUE for all eleven on a machine with no
ROS at all — which is why a junit-derived skip streak cannot work, and why an
entry citing only such a case is REFUSED here. Letting one count as evidence
would re-create, inside the mechanism, the exact defect it exists to catch.

WHAT THIS CANNOT DO

It cannot know that a hand-written entry is TRUE — only that it is well-formed,
names a real Runtime cell, and cites cases that exist in that cell's declared
test binary and are not binding-only. `--record` is the honest path: it writes
an entry from a junit and refuses one in which the cell's binary only skipped.
Nor does it know when a verdict goes stale; a cell verified today can break
tomorrow, and only phase-433 W5's scheduled lane will say so. The age is
reported in days rather than expired, because an expiry over cells that only a
ROS-carrying lane can refresh would go red for every contributor and stay red
(CLAUDE.md: a uniformly-red lane has no signal capacity).

Usage::

    check-interop-verdicts.py                      # the gate
    check-interop-verdicts.py --report             # every cell, verdict or not
    check-interop-verdicts.py --after-run [JUNIT]  # one line at a sweep's tail
    check-interop-verdicts.py --record CELL --junit F --where TEXT [--issue N]
    check-interop-verdicts.py --self-test          # the negative controls
"""

from __future__ import annotations

import argparse
import datetime
import importlib.util
import re
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:  # python < 3.11
    import tomli as tomllib  # type: ignore

ROOT = Path(__file__).resolve().parent.parent
LEDGER = ROOT / ".config" / "interop-verdicts.toml"
TESTS_DIR = ROOT / "packages" / "testing" / "nros-tests" / "tests"
DEFAULT_JUNIT = ROOT / "target" / "nextest" / "default" / "junit.xml"

# The cell table has exactly one parser, in the sibling gate — the repo rule
# (`row_coord()` in fixtures-manifest.py is the canonical statement of it: a
# second derivation left 67 of 240 rows in no lane at all). Importing it also
# means a shape change in `interop.rs` fails ONE parser loudly rather than
# leaving this one quietly reading fewer rows.
_spec = importlib.util.spec_from_file_location(
    "_interop_cell_runners", ROOT / "scripts" / "check-interop-cell-runners.py"
)
_sib = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_sib)  # type: ignore[union-attr]

REQUIRED = ("cell", "date", "verdict", "tests", "run", "where", "evidence")
OPTIONAL = ("issue", "note")
VERDICTS = ("pass", "fail")
DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
FN_RE = re.compile(r"\bfn\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(")
USE_RE = re.compile(r"^\s*use\s[^;]*;", re.M)
ATTR_RE = re.compile(r"^\s*#!?\[[^\]]*\]", re.M)


class LedgerError(Exception):
    pass


def _rel(p: Path) -> str:
    """Repo-relative if it is inside the repo, else as given (`--ledger` demos)."""
    try:
        return str(p.resolve().relative_to(ROOT))
    except ValueError:
        return str(p)


# --- the cells -----------------------------------------------------------


def runtime_cells() -> list[dict]:
    """[{id, tier, test, line}] for every `Tier::Runtime` interop cell."""
    cells = _sib.parse_cells(_sib.INTEROP_RS.read_text(encoding="utf-8"))
    return [c for c in cells if c["tier"] == "Runtime"]


def test_sources(cells: list[dict]) -> dict[str, str | None]:
    """{test binary: source text or None if the file is gone}."""
    out: dict[str, str | None] = {}
    for c in cells:
        test = c["test"]
        if test is None or test in out:
            continue
        path = TESTS_DIR / f"{test}.rs"
        out[test] = path.read_text(encoding="utf-8") if path.is_file() else None
    return out


# --- which cases can be EVIDENCE ----------------------------------------


def fn_bodies(src: str) -> dict[str, str]:
    """{fn name: its `{...}` body} for every function in a Rust source."""
    cat = _sib.categorize(src)
    out: dict[str, str] = {}
    for m in FN_RE.finditer(src):
        if cat[m.start()] != _sib.CODE:
            continue
        try:
            close = _sib._match(src, cat, m.end() - 1, ")")
        except ValueError:
            continue
        brace = src.find("{", close)
        if brace < 0:
            continue
        # `fn f() -> T;` in a trait — a declaration, not a body.
        if ";" in src[close + 1 : brace]:
            continue
        try:
            end = _sib._match(src, cat, brace, "}")
        except ValueError:
            continue
        out[m.group(1)] = src[brace : end + 1]
    return out


def is_binding_only(body: str) -> bool:
    """Does this body do NOTHING but bind the test to `interop::CELLS`?

    `cases_bound_to_interop_cells` (params, qos_override_e2e, and seven more) is
    a `use` line and one `assert_test_bound(...)` call. It needs no fixture, no
    peer and no ROS, so it PASSES on every host — which is exactly why it may
    not be cited as evidence that a cell met a peer.

    The test is "nothing else is here", not "no `skip!` is here": `graph_interop`
    and `interop_e2e` call `assert_test_bound` INSIDE the real case, and reading
    a keyword out of the body would drop the only cases those binaries have.
    """
    if "assert_test_bound" not in body:
        return False
    rest = "".join(
        " " if k != _sib.CODE else ch for ch, k in zip(body, _sib.categorize(body))
    )
    rest = ATTR_RE.sub(" ", USE_RE.sub(" ", rest))
    while True:
        at = rest.find("assert_test_bound")
        if at < 0:
            break
        # Take the whole path with it (`nros_tests::interop::assert_test_bound`),
        # or the leading `interop::` is what is left over and every binding test
        # reads as "does something else too".
        while at and (rest[at - 1].isalnum() or rest[at - 1] in "_:"):
            at -= 1
        cat = _sib.categorize(rest)
        paren = rest.find("(", at)
        if paren < 0:
            break
        try:
            close = _sib._match(rest, cat, paren, ")")
        except ValueError:
            break
        semi = rest.find(";", close)
        rest = rest[:at] + rest[(semi + 1 if semi >= 0 else close + 1) :]
    return not rest.strip(" \t\r\n{}")


def evidence_cases(src: str) -> tuple[set[str], set[str]]:
    """(every fn, the binding-only ones) for one test source."""
    bodies = fn_bodies(src)
    return set(bodies), {n for n, b in bodies.items() if is_binding_only(b)}


def base_case(name: str) -> str:
    """`interop::case_3_zenoh_service` -> `interop`. rstest names its cases
    under the parent fn, and only the parent appears in the source."""
    return name.split("::", 1)[0].strip()


# --- the ledger ----------------------------------------------------------


def load_ledger(text: str) -> list[dict]:
    try:
        data = tomllib.loads(text)
    except Exception as e:  # tomllib raises TOMLDecodeError
        raise LedgerError(f"{LEDGER.name}: not valid TOML: {e}") from None
    stray = sorted(k for k in data if k != "verdict")
    if stray:
        raise LedgerError(
            f"{LEDGER.name}: top-level key(s) {', '.join(stray)} — every entry is "
            f"a `[[verdict]]` table."
        )
    entries = data.get("verdict", [])
    if not isinstance(entries, list):
        raise LedgerError(f"{LEDGER.name}: `verdict` is not a `[[verdict]]` array.")
    return entries


def issue_status(num: str) -> tuple[bool, str | None]:
    """(exists, status) for `docs/issues/NNNN-*.md` — mirrors quarantine.py."""
    base = ROOT / "docs" / "issues"
    for d in (base, base / "archived"):
        if not d.is_dir():
            continue
        for p in d.iterdir():
            if p.name.startswith(f"{num}-") and p.suffix == ".md":
                head = p.read_text(encoding="utf-8", errors="replace")[:4000]
                m = re.search(r"^status:\s*(\S+)", head, re.M)
                return True, (m.group(1) if m else "?")
    return False, None


def evaluate(
    cells: list[dict],
    ledger_text: str,
    sources: dict[str, str | None],
    issue_lookup=issue_status,
    today: datetime.date | None = None,
) -> tuple[list[str], dict]:
    """(problems, stats). The whole rule, over data the caller supplied."""
    today = today or datetime.date.today()
    problems: list[str] = []
    if not cells:
        problems.append(
            "  no Runtime cells in interop::CELLS — refusing to report OK over an "
            "empty set."
        )
    by_id = {c["id"]: c for c in cells}
    entries = load_ledger(ledger_text)

    seen: set[str] = set()
    verified: dict[str, dict] = {}
    for i, e in enumerate(entries):
        cid = str(e.get("cell", "")).strip()
        who = cid or f"<entry {i + 1}>"
        unknown = sorted(set(e) - set(REQUIRED) - set(OPTIONAL))
        if unknown:
            problems.append(
                f"  {who}: unknown field(s) {', '.join(unknown)}.\n"
                f"      A misspelt field is a field that silently carries nothing —\n"
                f"      the evidence would read as absent while looking present."
            )
        missing = [f for f in REQUIRED if not str(e.get(f, "")).strip()]
        if missing:
            problems.append(f"  {who}: missing or empty field(s): {', '.join(missing)}")
            continue
        if cid in seen:
            problems.append(
                f"  {who}: recorded twice. One entry per cell — the history is in "
                f"git, not in duplicate rows."
            )
            continue
        seen.add(cid)

        cell = by_id.get(cid)
        if cell is None:
            problems.append(
                f"  {who}: not a `Tier::Runtime` cell in interop::CELLS.\n"
                f"      A verdict for a cell that no longer exists is a claim of\n"
                f"      coverage nobody can act on, and the next cell to take that\n"
                f"      id inherits it silently."
            )
            continue

        if not DATE_RE.match(str(e["date"])):
            problems.append(f"  {who}: `date` is not YYYY-MM-DD: {e['date']!r}")
        else:
            when = datetime.date.fromisoformat(str(e["date"]))
            if when > today:
                problems.append(
                    f"  {who}: `date` {when} is in the future — a typo, or a run "
                    f"that has not happened."
                )

        verdict = str(e["verdict"]).strip()
        if verdict not in VERDICTS:
            problems.append(
                f"  {who}: `verdict` is {verdict!r}, not one of {'/'.join(VERDICTS)}."
            )
        elif verdict == "fail":
            num = str(e.get("issue", "")).strip()
            if not re.fullmatch(r"\d{3,4}", num):
                problems.append(
                    f"  {who}: verdict `fail` with no `issue = \"NNNN\"`.\n"
                    f"      A failing cell is a FINDING, not a blocker (issue 1127\n"
                    f"      Direction item 2) — but an unattributed one is debt\n"
                    f"      nobody owns."
                )
            else:
                exists, status = issue_lookup(num)
                if not exists:
                    problems.append(
                        f"  {who}: issue {num} does not exist under docs/issues/."
                    )
                elif status != "open":
                    problems.append(
                        f"  {who}: issue {num} is `{status}`, not open. Re-run the "
                        f"cell and record the new verdict, or reopen the issue."
                    )

        tests = e["tests"]
        if not isinstance(tests, list) or not all(isinstance(t, str) for t in tests):
            problems.append(f"  {who}: `tests` is not a list of case names.")
            continue
        src = sources.get(cell["test"])
        if src is None:
            problems.append(
                f"  {who}: its cell names test binary `{cell['test']}`, and\n"
                f"      packages/testing/nros-tests/tests/{cell['test']}.rs does not\n"
                f"      exist — the evidence cites a binary nothing can run."
            )
            continue
        known, binding = evidence_cases(src)
        real = 0
        for t in tests:
            fn = base_case(t)
            if fn not in known:
                problems.append(
                    f"  {who}: cites `{t}`, and `{cell['test']}.rs` has no `fn {fn}`.\n"
                    f"      A renamed or deleted case leaves the entry INERT while it\n"
                    f"      still reads as coverage (the issue-0743 class)."
                )
            elif fn in binding:
                problems.append(
                    f"  {who}: cites `{t}`, which is a BINDING test — its whole body\n"
                    f"      is `interop::assert_test_bound(...)`. It passes on any\n"
                    f"      host, with no peer, forever, so it is evidence of nothing.\n"
                    f"      Cite the case that met the peer."
                )
            else:
                real += 1
        if real == 0:
            problems.append(
                f"  {who}: no cited case can be evidence — the entry records a "
                f"verdict nothing produced."
            )
        else:
            verified[cid] = e

    return problems, {
        "runtime": len(cells),
        "verified": len(verified),
        "passed": sum(1 for e in verified.values() if e["verdict"] == "pass"),
        "failed": sum(1 for e in verified.values() if e["verdict"] == "fail"),
        "never": len(cells) - len(verified),
    }


# --- the report ----------------------------------------------------------


def report_lines(
    cells: list[dict], entries: list[dict], runners: dict[str, list[str]], today
) -> list[str]:
    by_cell = {str(e.get("cell", "")): e for e in entries}
    width = max((len(c["id"]) for c in cells), default=10)
    out = [
        "Live-peer interop cells — has a real ROS 2 peer ever answered?",
        f"  {_rel(LEDGER)} — a cell with no entry has NEVER produced a",
        "  verdict. That is the default, not an omission (issue 1127).",
        "",
    ]
    for c in sorted(cells, key=lambda c: c["id"]):
        e = by_cell.get(c["id"])
        if e is None:
            where = runners.get(c["test"])
            hint = f"runner: {where[0]}" if where else "NO FOCUSED RUNNER"
            out.append(
                f"  NEVER  {'—':<16}  {c['id']:<{width}}  {c['test']}  ({hint})"
            )
            continue
        when = str(e.get("date", "?"))
        stamp = when
        if DATE_RE.match(when):
            stamp = f"{when} {(today - datetime.date.fromisoformat(when)).days}d ago"
        tag = str(e.get("verdict", "?")).upper()
        extra = f"  issue {e['issue']}" if e.get("issue") else ""
        out.append(
            f"  {tag:<5}  {stamp:<16}  {c['id']:<{width}}  {c['test']}{extra}"
        )
    n, tot = len(by_cell), len(cells)
    out += [
        "",
        f"  {n} of {tot} Runtime cells have ever produced a verdict; {tot - n} have not.",
    ]
    return out


# --- what a RUN saw ------------------------------------------------------


def observed(junit: Path, cells: list[dict], sources: dict[str, str | None]) -> dict:
    """{test binary: {"verdict": [...], "skipped": [...]}} for interop binaries.

    A case that is binding-only is in NEITHER list: it passes with no peer on
    every host, so counting it as a verdict is the defect this file exists to
    catch, and counting it as a skip would be a lie in the other direction.
    """
    wanted = {c["test"] for c in cells}
    out: dict[str, dict[str, list[str]]] = {}
    try:
        root = ET.parse(junit).getroot()
    except (OSError, ET.ParseError):
        return out
    binding: dict[str, set[str]] = {}
    for test, src in sources.items():
        binding[test] = evidence_cases(src)[1] if src else set()
    for case in root.iter("testcase"):
        cls = (case.get("classname") or "").split("::")[-1]
        if cls not in wanted:
            continue
        name = case.get("name") or "?"
        if base_case(name) in binding.get(cls, set()):
            continue
        row = out.setdefault(cls, {"verdict": [], "skipped": []})
        skipped = case.find("skipped") is not None
        row["skipped" if skipped else "verdict"].append(name)
    return out


def after_run(junit: Path, cells: list[dict], sources, entries) -> list[str]:
    """The one line a sweep's tail prints. Silent when no interop binary ran."""
    seen = observed(junit, cells, sources)
    if not seen:
        return []
    recorded = {str(e.get("cell", "")) for e in entries}
    ran = [t for t, r in seen.items() if r["verdict"]]
    dumb = [t for t, r in seen.items() if not r["verdict"]]
    out = [
        f"Live-peer interop: {len(recorded)}/{len(cells)} cells have ever produced a "
        f"verdict. This run reached {len(seen)} of their binaries — {len(ran)} "
        f"produced a result, {len(dumb)} skipped wholesale (`just interop-verdicts`)."
    ]
    fresh = sorted(
        c["id"]
        for c in cells
        if c["id"] not in recorded and c["test"] in ran
    )
    if fresh:
        out.append(
            "  This run produced a result for cells with no recorded verdict: "
            + ", ".join(fresh)
        )
        out.append(
            "  Record it: python3 scripts/check-interop-verdicts.py --record "
            f"{fresh[0]} --junit {junit} --where '<host>'"
        )
    return out


# --- writing an entry ----------------------------------------------------


def _toml_str(s: str) -> str:
    return '"' + str(s).replace("\\", "\\\\").replace('"', '\\"') + '"'


def render(entries: list[dict], header: str) -> str:
    out = [header.rstrip("\n"), ""]
    for e in sorted(entries, key=lambda e: str(e.get("cell", ""))):
        out.append("[[verdict]]")
        for k in REQUIRED + OPTIONAL:
            if k not in e:
                continue
            v = e[k]
            if isinstance(v, list):
                out.append(f"{k} = [{', '.join(_toml_str(x) for x in v)}]")
            else:
                out.append(f"{k} = {_toml_str(v)}")
        out.append("")
    return "\n".join(out).rstrip("\n") + "\n"


def header_of(text: str) -> str:
    at = text.find("[[verdict]]")
    return text if at < 0 else text[:at]


def record(args, cells, sources) -> int:
    cell = next((c for c in cells if c["id"] == args.record), None)
    if cell is None:
        print(
            f"check-interop-verdicts: `{args.record}` is not a Tier::Runtime cell in "
            f"interop::CELLS.",
            file=sys.stderr,
        )
        return 1
    junit = Path(args.junit) if args.junit else DEFAULT_JUNIT
    seen = observed(junit, [cell], sources).get(cell["test"])
    if not seen:
        print(
            f"check-interop-verdicts: {junit} contains no case of `{cell['test']}` "
            f"(binding tests excluded — they are evidence of nothing).",
            file=sys.stderr,
        )
        return 1
    if not seen["verdict"]:
        print(
            f"check-interop-verdicts: every case of `{cell['test']}` in {junit} "
            f"SKIPPED — that is no verdict, and it is exactly the absence this "
            f"ledger exists to make visible. Skipped: "
            f"{', '.join(seen['skipped'])}",
            file=sys.stderr,
        )
        return 1
    failed = _failures(junit, cell["test"], seen["verdict"])
    if failed and not args.issue:
        # Refuse HERE rather than let the gate reject the file afterwards: a
        # failing cell is a finding (issue 1127 Direction item 2), and the
        # moment to attribute it is while the output is still on screen.
        print(
            f"check-interop-verdicts: {', '.join(sorted(failed))} FAILED — record it "
            f"with --issue NNNN. Reserve one with `just issue-new <slug>`; a failing "
            f"cell is a finding, not a blocker, but an unattributed one is debt "
            f"nobody owns.",
            file=sys.stderr,
        )
        return 1
    entry = {
        "cell": cell["id"],
        "date": datetime.date.today().isoformat(),
        "verdict": "fail" if failed else "pass",
        "tests": sorted(seen["verdict"]),
        "run": args.run or f"cargo nextest run -p nros-tests --test {cell['test']}",
        "where": args.where,
        "evidence": args.evidence
        or (
            f"{len(seen['verdict'])} case(s) produced a result in {junit.name}"
            + (f"; FAILED: {', '.join(sorted(failed))}" if failed else "")
        ),
    }
    if args.issue:
        entry["issue"] = args.issue
    text = LEDGER.read_text(encoding="utf-8") if LEDGER.exists() else ""
    entries = [e for e in load_ledger(text) if e.get("cell") != cell["id"]]
    entries.append(entry)
    LEDGER.write_text(render(entries, header_of(text)), encoding="utf-8")
    print(f"check-interop-verdicts: recorded `{cell['id']}` as {entry['verdict']}")
    return 0


def _failures(junit: Path, test: str, names: list[str]) -> set[str]:
    out: set[str] = set()
    try:
        root = ET.parse(junit).getroot()
    except (OSError, ET.ParseError):
        return out
    for case in root.iter("testcase"):
        if (case.get("classname") or "").split("::")[-1] != test:
            continue
        if (case.get("name") or "") in names and (
            case.find("failure") is not None or case.find("error") is not None
        ):
            out.add(case.get("name") or "?")
    return out


# --- negative controls ---------------------------------------------------

_MINI_RS = '''
pub const CELLS: &[InteropCell] = &[
    ic("cell-alpha",
       c(Linux, Rust, Zenoh, Pubsub, Interop, Runtime),
       NativeFixtures, RosEdition(Zenoh), NanoToRos, "alpha_e2e"),
    ic("cell-beta",
       c(Linux, Rust, Zenoh, Service, Interop, Runtime),
       NativeFixtures, RosEdition(Zenoh), BiDir, "beta_e2e"),
];
'''

_ALPHA_RS = '''
#[test]
fn meets_a_peer() {
    interop::assert_test_bound("alpha_e2e", &CELLS);
    if !require_ros2() {
        nros_tests::skip!("ROS 2 not available");
    }
    assert!(talks_to(&peer));
}

/// The binding test — passes on any host, forever.
#[test]
fn cases_bound_to_interop_cells() {
    #[allow(unused_imports)]
    use nros_tests::matrix::{Lang::*, PlatformId::*};
    nros_tests::interop::assert_test_bound("alpha_e2e", &[(Linux, Rust, Zenoh, Pubsub)]);
}
'''

_BETA_RS = '''
#[test]
fn cases_bound_to_interop_cells() {
    nros_tests::interop::assert_test_bound("beta_e2e", &[(Linux, Rust, Zenoh, Service)]);
}
'''


def _junit(rows: list[tuple[str, str, str]]) -> str:
    """rows of (binary, case, outcome in pass/fail/skip)."""
    body = []
    for binary, name, outcome in rows:
        inner = {
            "pass": "",
            "fail": "<failure message='boom'>boom</failure>",
            "skip": "<skipped type='nros:capability' message='[SKIPPED] no ROS'/>",
        }[outcome]
        body.append(
            f"<testcase classname='nros-tests::{binary}' name='{name}'>{inner}</testcase>"
        )
    return "<testsuites><testsuite name='s'>" + "".join(body) + "</testsuite></testsuites>"


_OK_LEDGER = """
[[verdict]]
cell = "cell-alpha"
date = "2026-09-06"
verdict = "pass"
tests = ["meets_a_peer"]
run = "cargo nextest run -p nros-tests --test alpha_e2e"
where = "ros2 distrobox"
evidence = "PASS [18.7s] alpha_e2e meets_a_peer"
"""


def self_test(verbose: bool = False, tmp: Path | None = None) -> None:
    """Both directions, on synthetic input, on the normal path.

    `check-gate-selftests` requires the call here rather than behind a flag: a
    negative control nobody runs decays into a comment.
    """
    cells = [c for c in _sib.parse_cells(_MINI_RS) if c["tier"] == "Runtime"]
    assert [c["id"] for c in cells] == ["cell-alpha", "cell-beta"], cells
    src = {"alpha_e2e": _ALPHA_RS, "beta_e2e": _BETA_RS}
    today = datetime.date(2026, 9, 6)
    issues = {"1137": (True, "open"), "0001": (True, "resolved")}
    look = lambda n: issues.get(n, (False, None))  # noqa: E731

    # The binding-only classifier, both directions — the load-bearing rule.
    known, binding = evidence_cases(_ALPHA_RS)
    assert known == {"meets_a_peer", "cases_bound_to_interop_cells"}, known
    assert binding == {"cases_bound_to_interop_cells"}, (
        f"selftest: binding-only classifier is wrong: {binding}"
    )
    assert "meets_a_peer" not in binding, (
        "selftest: a real case that ALSO binds its coordinate was classified as "
        "binding-only — that is graph_interop's and interop_e2e's shape"
    )

    problems, stats = evaluate(cells, _OK_LEDGER, src, look, today)
    assert problems == [], f"selftest: rejected a well-formed ledger: {problems}"
    assert stats == {"runtime": 2, "verified": 1, "passed": 1, "failed": 0,
                     "never": 1}, stats

    def bad(text, want, why):
        got = evaluate(cells, text, src, look, today)[0]
        assert any(want in p for p in got), f"selftest: {why} (got {got})"

    bad(
        _OK_LEDGER.replace('tests = ["meets_a_peer"]',
                           'tests = ["cases_bound_to_interop_cells"]'),
        "BINDING test",
        "a binding-only case was accepted as evidence",
    )
    bad(
        _OK_LEDGER.replace('tests = ["meets_a_peer"]', 'tests = ["gone_away"]'),
        "no `fn gone_away`",
        "a case that no longer exists was accepted",
    )
    bad(
        _OK_LEDGER.replace('cell = "cell-alpha"', 'cell = "cell-ghost"'),
        "not a `Tier::Runtime` cell",
        "a verdict for a cell that does not exist was accepted",
    )
    bad(
        _OK_LEDGER.replace('verdict = "pass"', 'verdict = "fail"'),
        "no `issue",
        "an unattributed failing verdict was accepted",
    )
    bad(
        _OK_LEDGER.replace('verdict = "pass"', 'verdict = "fail"\nissue = "0001"'),
        "is `resolved`, not open",
        "a failing verdict pointing at a closed issue was accepted",
    )
    bad(
        _OK_LEDGER.replace('date = "2026-09-06"', 'date = "2099-01-01"'),
        "in the future",
        "a future date was accepted",
    )
    bad(
        _OK_LEDGER.replace("where =", "wehre ="),
        "unknown field",
        "a misspelt field was accepted",
    )
    bad(_OK_LEDGER + _OK_LEDGER, "recorded twice", "a duplicate entry was accepted")
    bad(_OK_LEDGER.replace('evidence = "PASS [18.7s] alpha_e2e meets_a_peer"', ""),
        "missing or empty", "an entry with no evidence was accepted")
    assert any("empty set" in p for p in evaluate([], "", src, look, today)[0]), (
        "selftest: reported OK over zero Runtime cells"
    )
    for text, want in (
        ("not = [toml", "not valid TOML"),
        ('[[verdit]]\ncell = "x"\n', "top-level key"),
    ):
        try:
            evaluate(cells, text, src, look, today)
        except LedgerError as e:
            assert want in str(e), f"selftest: wrong error for {want!r}: {e}"
        else:
            raise AssertionError(f"selftest: accepted a malformed ledger ({want})")

    # --- what a run saw, on synthetic junit -----------------------------
    # A TemporaryDirectory, not mkdtemp: this runs on the normal path of every
    # invocation, `_test-summary` calls it at the tail of every sweep, and a
    # leaked directory per test run is a slow mess nobody would attribute here.
    with tempfile.TemporaryDirectory(prefix="nros-interop-verdicts-") as td:
        _self_test_junit(Path(td) if tmp is None else tmp, cells, src)
    if verbose:
        print("check-interop-verdicts: self-test OK")


def _self_test_junit(tmp: Path, cells: list[dict], src: dict[str, str | None]) -> None:
    """The junit half of the negative controls — one temp dir, cleaned up."""
    only_skips = tmp / "skips.xml"
    only_skips.write_text(
        _junit([("alpha_e2e", "meets_a_peer", "skip"),
                ("alpha_e2e", "cases_bound_to_interop_cells", "pass")])
    )
    seen = observed(only_skips, cells, src)
    assert seen["alpha_e2e"]["verdict"] == [], (
        "selftest: a binding test was counted as a verdict — the whole defect"
    )
    assert seen["alpha_e2e"]["skipped"] == ["meets_a_peer"], seen
    assert after_run(only_skips, cells, src, load_ledger(_OK_LEDGER)), (
        "selftest: a wholly-skipped interop binary printed nothing"
    )

    real = tmp / "real.xml"
    real.write_text(_junit([("beta_e2e", "delivers", "pass"),
                            ("beta_e2e", "cases_bound_to_interop_cells", "pass")]))
    src2 = dict(src, beta_e2e=_BETA_RS + "\n#[test]\nfn delivers() { assert!(x); }\n")
    seen = observed(real, cells, src2)
    assert seen["beta_e2e"]["verdict"] == ["delivers"], seen
    lines = after_run(real, cells, src2, load_ledger(_OK_LEDGER))
    assert any("cell-beta" in ln for ln in lines), (
        f"selftest: a cell that produced a verdict with no record was not named: {lines}"
    )

    assert observed(tmp / "nope.xml", cells, src) == {}, (
        "selftest: invented results from a junit that does not exist"
    )


# --- entry point ---------------------------------------------------------


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--report", action="store_true")
    ap.add_argument("--after-run", nargs="?", const=str(DEFAULT_JUNIT), default=None)
    ap.add_argument("--record", metavar="CELL")
    ap.add_argument("--junit")
    ap.add_argument("--where", default="")
    ap.add_argument("--run", default="")
    ap.add_argument("--evidence", default="")
    ap.add_argument("--issue", default="")
    ap.add_argument("--ledger", default=None, help="override (self-test / demos)")
    ap.add_argument(
        "--list-passing",
        action="store_true",
        help="cell ids with a recorded PASS, one per line — the membership of "
        "phase-433 W5's regression lane, which is the ledger and not a "
        "hand-kept list",
    )
    ap.add_argument(
        "--by-binary",
        action="store_true",
        help="with --list-passing, emit the distinct TEST BINARIES those cells "
        "live in instead of the cell ids (several cells share one binary)",
    )
    args = ap.parse_args(argv)

    if args.self_test:
        self_test(verbose=True)
        return 0
    # Always, not behind a flag: a negative control nobody runs is a comment.
    self_test()

    global LEDGER
    if args.ledger:
        LEDGER = Path(args.ledger).resolve()

    try:
        cells = runtime_cells()
    except _sib.ParseError as e:
        print(f"check-interop-verdicts: {e}", file=sys.stderr)
        return 1
    sources = test_sources(cells)
    text = LEDGER.read_text(encoding="utf-8") if LEDGER.exists() else ""
    today = datetime.date.today()

    if args.record:
        if not args.where:
            print(
                "check-interop-verdicts: --record needs --where (which host, which "
                "box, which lane produced this). A verdict with no provenance is "
                "the claim, not the evidence.",
                file=sys.stderr,
            )
            return 1
        return record(args, cells, sources)

    if args.after_run is not None:
        try:
            entries = load_ledger(text)
        except LedgerError:
            entries = []
        for line in after_run(Path(args.after_run), cells, sources, entries):
            print(line)
        return 0

    if args.list_passing:
        try:
            entries = load_ledger(text)
        except LedgerError as e:
            print(f"check-interop-verdicts: {e}", file=sys.stderr)
            return 1
        passing = sorted(
            e["cell"] for e in entries if e.get("verdict") == "pass"
        )
        if args.by_binary:
            by_id = {c["id"]: c for c in cells}
            # A cell whose id is no longer in CELLS is the ledger gate's problem,
            # not this lane's — skip it here rather than crash the run that is
            # supposed to detect regressions.
            bins = sorted(
                {by_id[c]["test"] for c in passing if c in by_id and by_id[c].get("test")}
            )
            print("\n".join(bins))
        else:
            print("\n".join(passing))
        return 0

    if args.report:
        try:
            entries = load_ledger(text)
        except LedgerError as e:
            print(f"check-interop-verdicts: {e}", file=sys.stderr)
            return 1
        print("\n".join(report_lines(cells, entries, _sib.runners(), today)))
        return 0

    try:
        problems, stats = evaluate(cells, text, sources, issue_status, today)
    except LedgerError as e:
        print(f"check-interop-verdicts: {e}", file=sys.stderr)
        return 1
    if problems:
        print("check-interop-verdicts: the live-peer verdict ledger is wrong\n")
        print("\n\n".join(problems))
        print(
            f"\n{_rel(LEDGER)} records that a cell HAS met a peer. A cell "
            f"with no entry has never produced a verdict, which is the default and "
            f"needs no line."
        )
        return 1
    print(
        f"check-interop-verdicts OK — {stats['verified']}/{stats['runtime']} Runtime "
        f"interop cells have ever produced a verdict against a live peer "
        f"({stats['passed']} pass, {stats['failed']} fail, {stats['never']} NEVER RUN; "
        f"`just interop-verdicts`)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
