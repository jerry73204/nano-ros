#!/usr/bin/env python3
"""A Runtime interop cell nobody INVOKES — phase-433 W4, issue 1127.

`nros_tests::interop::CELLS` is the intent list for every test whose subject is
a LIVE ROS 2 peer. `tests/matrix_fixture_coverage.rs` G1 is supposed to be its
coverage gate, and its doc comment says so:

    G1 -- test coverage. Every Runtime interop/bridge cell names a real test
    binary (`tests/<test>.rs` exists) ... A Runtime cell nothing runs ... fails
    here.

What G1 asserts is `tests_dir.join(format!("{}.rs", c.test)).is_file()`. That is
the FILE existing. Five gates (G1-G5) surround a cell -- coverage, build-coord
match, tier, peer-declaration -- and none of them asks whether anything ever
invokes the binary. "A Runtime cell nothing runs" was the promise; "a Runtime
cell whose source file exists" was the check.

MEASURED 2026-09-06, before this gate: 17 Runtime cells naming 11 distinct test
binaries, of which 3 are named by a recipe --

    interop_e2e        `just native test-ros2`, `just native test-ros2-lifecycle`
    params             `just native test-ros2-params`
    xrce_ros2_interop  `just xrce test-ros2`

-- and 8 are named by no recipe and no workflow, covering 9 cells. Every one of
those 9 declares a live peer, a direction and a build channel, and is bound to
its coordinate by `interop::assert_test_bound`. The binding is exact and nothing
is aimed at it.

WHAT "NO RUNNER" DOES AND DOES NOT MEAN

It does NOT mean the binaries never execute, and an earlier draft of this file
said they did not. Measured 2026-09-06: the ROS-interop exclusion
(`not (group(=ros2-interop) or binary(xrce_ros2_interop) or ...)`) belongs to
`just native test` and `just test-integration`, NOT to the root sweep. Root
`just test-all` runs `cargo nextest --workspace` and filters only by lane
coordinate, absent toolchains and `ros_editions`, so the ten native interop
binaries are in it -- including on `host-tests.yml`'s integration job, which
runs `just ci tier1` on every push to main inside the
`ghcr.io/newslabntu/nano-ros-ci:humble` container. (The one exception is
`qos_zephyr_ros2_interop_e2e`: a zephyr coordinate, out of the tier-1 cover.)

The predicate here is a FOCUSED runner, and that is a different thing from
being swept along:

  * A sweep cannot be aimed at ONE cell, which is what running a live-peer test
    against a real ROS 2 node takes -- issue 1127's Direction is explicit that
    the order is "one cell end to end, by hand, recorded" BEFORE any lane.
  * A sweep brings up no peer. The named runners do (`test-ros2*` want the
    distro's `rmw_zenoh_cpp`; `just xrce test-ros2` wants an Agent).
  * An unmet precondition inside a sweep is `nros_tests::skip!`, which
    `_rewrite-skipped-junit` turns into `<skipped>` -- so a green sweep is not
    evidence that a cell met a peer. Whether these nine currently skip there is
    NOT measured by this gate and is not claimed here.

So the check is issue 1127's Direction item 4 exactly as written: "a Runtime
cell's test binary is named by at least one recipe".

WHY THE CELL LIST IS PARSED, NOT BUILT

`interop::CELLS` is Rust, and the obvious alternative is `lane-coords`'s shape:
a `nros-tests` bin that emits the table as JSON. That would put this gate in
`build-serial:`, and issue 0993 is what that lane costs -- no pull request and
no merge group runs it, so a red is invisible until somebody reads a nightly.
This gate's whole job is to catch a cell added in a PR with no lane behind it,
which is a verdict that has to arrive on that PR.

So it parses, like `check-lane-scope-consumers.py` parses `matrix.rs`. The parse
is made non-fragile by being STRICT rather than tolerant: `ic(...)` must have
exactly 6 arguments and its `c(...)` exactly 6, the tier token must be one this
file knows, a Runtime row must carry a string test, and finding zero rows is an
error. A shape change turns this red and names the row -- it cannot go vacuous,
which is the only failure mode of a parser that actually matters here.

THE LEDGER SHRINKS

The 9 unrun cells are issue 1127's, not this gate's, so they are listed in
`.config/interop-cells-without-runner.txt`, each naming that issue. A cell added
without a runner fails. A listed cell that GAINS one also fails, telling you to
delete the line -- so phase-433 W5 empties the file as it lands lanes, and
nothing can quietly re-fill it. There is deliberately no `--write-ledger` flag:
a regenerate button turns "add a cell no lane runs" back into a one-command
move, which is the thing being ratcheted.

Usage::

    check-interop-cell-runners.py             # the gate
    check-interop-cell-runners.py --list      # every cell, runner or not
    check-interop-cell-runners.py --self-test # the negative controls, verbose
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
INTEROP_RS = ROOT / "packages" / "testing" / "nros-tests" / "src" / "interop.rs"
LEDGER = ROOT / ".config" / "interop-cells-without-runner.txt"

# Where a runner can be written down. Pathspecs, not directories to walk.
RUNNER_PATHSPECS = ("justfile", "just", ".github/workflows")

CELLS_HEADER = "pub const CELLS: &[InteropCell] = &["

# `Tier`'s three variants, as `matrix.rs` spells them. Listed so an unknown
# token is an ERROR rather than a silent "not Runtime" -- a tier added later
# would otherwise drop its cells out of this gate without failing anything.
TIERS = ("Runtime", "BuildOnly", "CarveOut")

# code / string / comment, per character. Splitting needs to skip structural
# characters inside strings and comments; the id and test name need the string
# contents kept, so this is three categories rather than a boolean.
CODE, STRING, COMMENT = 0, 1, 2


def categorize(text: str) -> list[int]:
    """Per-character CODE / STRING / COMMENT for a Rust source fragment."""
    cat = [CODE] * len(text)
    i, n = 0, len(text)
    while i < n:
        if text[i] == '"':
            cat[i] = STRING
            i += 1
            while i < n:
                cat[i] = STRING
                if text[i] == "\\":
                    if i + 1 < n:
                        cat[i + 1] = STRING
                    i += 2
                    continue
                if text[i] == '"':
                    i += 1
                    break
                i += 1
            continue
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            for k in range(i, j):
                cat[k] = COMMENT
            i = j
            continue
        if text.startswith("/*", i):
            j = text.find("*/", i)
            j = n if j < 0 else j + 2
            for k in range(i, j):
                cat[k] = COMMENT
            i = j
            continue
        i += 1
    return cat


def _match(text: str, cat: list[int], open_at: int, close_ch: str) -> int:
    """Index of the bracket closing the one at `open_at`, structural only."""
    open_ch = text[open_at]
    depth = 0
    for k in range(open_at, len(text)):
        if cat[k] != CODE:
            continue
        if text[k] == open_ch:
            depth += 1
        elif text[k] == close_ch:
            depth -= 1
            if depth == 0:
                return k
    raise ValueError(f"unbalanced `{open_ch}` at offset {open_at}")


def _split_args(text: str, cat: list[int], start: int, end: int) -> list[str]:
    """Top-level comma split of `text[start:end]`, comments blanked out."""
    args, depth, cur = [], 0, start
    for k in range(start, end):
        if cat[k] != CODE:
            continue
        c = text[k]
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        elif c == "," and depth == 0:
            args.append(_blank_comments(text, cat, cur, k))
            cur = k + 1
    tail = _blank_comments(text, cat, cur, end)
    if tail.strip():
        args.append(tail)
    return args


def _blank_comments(text: str, cat: list[int], start: int, end: int) -> str:
    return "".join(
        " " if cat[k] == COMMENT else text[k] for k in range(start, end)
    ).strip()


STRING_LIT = re.compile(r'"((?:[^"\\]|\\.)*)"', re.S)


def _string_of(arg: str) -> str | None:
    """The one string literal in `arg`, with `\\`-continuations folded out."""
    m = STRING_LIT.fullmatch(arg.strip())
    if not m:
        return None
    return re.sub(r"\\\s*\n\s*", "", m.group(1))


class ParseError(Exception):
    pass


def parse_cells(src: str) -> list[dict]:
    """[{id, tier, test, line}] for every `ic(...)` row in `CELLS`.

    STRICT on purpose -- see the module docstring. Every arity, every tier token
    and an empty result are errors, so a formatting change fails loudly instead
    of quietly shrinking the set this gate examines.
    """
    cat = categorize(src)
    head = src.find(CELLS_HEADER)
    if head < 0:
        raise ParseError(f"no `{CELLS_HEADER}` in {INTEROP_RS.name}")
    open_at = head + len(CELLS_HEADER) - 1
    end = _match(src, cat, open_at, "]")

    rows = []
    for m in re.finditer(r"\bic\s*\(", src):
        if m.start() < open_at or m.end() > end or cat[m.start()] != CODE:
            continue
        paren = m.end() - 1
        close = _match(src, cat, paren, ")")
        line = src.count("\n", 0, m.start()) + 1
        args = _split_args(src, cat, paren + 1, close)
        if len(args) != 6:
            raise ParseError(
                f"{INTEROP_RS.name}:{line}: `ic(...)` has {len(args)} argument(s), "
                f"expected 6 (id, cell, build, peer, dir, test)"
            )
        cid = _string_of(args[0])
        if cid is None:
            raise ParseError(
                f"{INTEROP_RS.name}:{line}: first `ic(...)` argument is not a "
                f"string id: {args[0]!r}"
            )
        rows.append({"id": cid, "line": line, **_parse_cell(args[1], line), **_parse_test(args[5], line)})
    if not rows:
        raise ParseError(
            f"parsed 0 rows from {INTEROP_RS.name} -- refusing to report OK over "
            f"an empty table"
        )
    return rows


def _parse_cell(arg: str, line: int) -> dict:
    """The tier out of the row's `c(platform, lang, rmw, workload, kind, tier)`."""
    arg = arg.strip()
    if not arg.startswith("c("):
        raise ParseError(
            f"{INTEROP_RS.name}:{line}: second `ic(...)` argument is not a "
            f"`c(...)` cell: {arg[:40]!r}"
        )
    cat = categorize(arg)
    inner = _split_args(arg, cat, arg.index("(") + 1, _match(arg, cat, arg.index("("), ")"))
    if len(inner) != 6:
        raise ParseError(
            f"{INTEROP_RS.name}:{line}: `c(...)` has {len(inner)} argument(s), "
            f"expected 6 (platform, lang, rmw, workload, kind, tier)"
        )
    tier = re.split(r"[\s(]", inner[5].strip(), maxsplit=1)[0]
    if tier not in TIERS:
        raise ParseError(
            f"{INTEROP_RS.name}:{line}: unknown tier `{tier}`. Add it to TIERS in "
            f"this gate -- an unrecognised tier would silently read as "
            f"not-Runtime and drop its cells out of the check."
        )
    return {"tier": tier}


def _parse_test(arg: str, line: int) -> dict:
    """The test binary, or None for the `NO_TEST` sentinel."""
    arg = arg.strip()
    if arg == "NO_TEST":
        return {"test": None}
    test = _string_of(arg)
    if test is None:
        raise ParseError(
            f"{INTEROP_RS.name}:{line}: last `ic(...)` argument is neither a "
            f"string test name nor `NO_TEST`: {arg[:40]!r}"
        )
    return {"test": test}


# --- who invokes a test binary ------------------------------------------

# `cargo test --test X` / `cargo nextest run --test X`.
DASH_TEST = re.compile(r"--test[=\s]+([A-Za-z0-9_]+)")
# A nextest filterset predicate. `binary(=x)` is the exact-match spelling.
BINARY_PRED = re.compile(r"\bbinary\(\s*=?\s*([A-Za-z0-9_]+)\s*\)")
NOT_GROUP = re.compile(r"\bnot\s*\(")


def _negated_spans(text: str) -> list[tuple[int, int]]:
    """[(start, end)] of every `not ( ... )` filterset group.

    `just native test` and `just test-integration` spell their exclusions
    `-E 'not (group(=ros2-interop) or binary(xrce_ros2_interop) or ...)'`.
    Reading that as an invocation would credit a runner to every binary those
    lanes REFUSE to run, which is the inverse of the truth. (Root `just
    test-all` carries no such exclusion — see the module docstring.)

    Nested negation (`not (a or not (b))`) is treated as exclusion throughout.
    That is the safe direction: over-reporting a missing runner asks for a
    ledger line and is loud, while under-reporting is the silence this gate
    exists to remove.
    """
    out = []
    for m in NOT_GROUP.finditer(text):
        depth = 0
        for k in range(m.end() - 1, len(text)):
            if text[k] == "(":
                depth += 1
            elif text[k] == ")":
                depth -= 1
                if depth == 0:
                    out.append((m.start(), k + 1))
                    break
    return out


# A YAML step LABEL. `check-lane-contracts` learned this one the hard way: a
# step NAMED "just check build + no_std" is prose, and reading it as an
# invocation attributed a recipe to every event the workflow had.
YAML_LABEL = re.compile(r"^\s*-?\s*name\s*:")


def _is_prose(line: str) -> bool:
    """A line that can NAME a binary without invoking it.

    Both `just` and YAML comment with `#`, and a workflow step label is prose in
    a `name:` value. Everything else is scanned, including bare shell fragments
    like `args=(-p nros-tests --test interop_e2e)` -- which is how two of the
    three real runners are spelled, so a "the line must also say cargo" filter
    would drop them.
    """
    return line.lstrip().startswith("#") or bool(YAML_LABEL.match(line))


def invocations_in(text: str) -> dict[str, int]:
    """{test binary: 1-based line} for every POSITIVE naming in one file."""
    lines = text.split("\n")
    body = "\n".join("" if _is_prose(ln) else ln for ln in lines)
    negated = _negated_spans(body)
    out: dict[str, int] = {}
    for pattern in (DASH_TEST, BINARY_PRED):
        for m in pattern.finditer(body):
            if any(s <= m.start() < e for s, e in negated):
                continue
            out.setdefault(m.group(1), body.count("\n", 0, m.start()) + 1)
    return out


def runner_sources() -> list[Path]:
    """The files that can name a runner: the justfiles and the workflows.

    `git ls-files`, not a walk -- both because that is the repo rule
    (`check-no-tracked-file-find`: an index lookup, not a stat of every
    directory) and because it handles nesting for free. `just/check.just` was
    split into `just/check/*.just` topic files while this gate was in review,
    and a flat glob would have stopped reading them silently.
    """
    out = subprocess.run(
        ["git", "-C", str(ROOT), "ls-files", "--", *RUNNER_PATHSPECS],
        capture_output=True, text=True, check=True,
    ).stdout.split()
    paths = [
        ROOT / p for p in out
        if p == "justfile" or p.endswith((".just", ".yml", ".yaml"))
    ]
    if not any(p.name == "justfile" for p in paths):
        raise SystemExit(
            "check-interop-cell-runners: `git ls-files` returned no root "
            "justfile -- refusing to report OK over a source list that cannot "
            "contain a runner."
        )
    return sorted(p for p in paths if p.is_file())


def runners() -> dict[str, list[str]]:
    """{test binary: ["<file>:<line>", ...]} across every runner source."""
    out: dict[str, list[str]] = {}
    for path in runner_sources():
        text = path.read_text(encoding="utf-8", errors="replace")
        for test, line in invocations_in(text).items():
            out.setdefault(test, []).append(
                f"{path.relative_to(ROOT)}:{line}"
            )
    return out


# --- the ledger ----------------------------------------------------------

ISSUE_REF = re.compile(r"\bissue\s+(\d{4})\b")


def parse_ledger(text: str) -> tuple[list[str], list[str]]:
    """(cell ids, problems). Every entry is `<cell-id>  # ... issue NNNN ...`."""
    ids: list[str] = []
    problems: list[str] = []
    for lineno, raw in enumerate(text.split("\n"), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        cid, sep, reason = line.partition("#")
        cid = cid.strip()
        if not cid or " " in cid:
            problems.append(
                f"  {LEDGER.name}:{lineno}: `{line}` is not `<cell-id>  # reason`"
            )
            continue
        if not sep or not ISSUE_REF.search(reason):
            problems.append(
                f"  {LEDGER.name}:{lineno}: `{cid}` names no tracked issue.\n"
                f"      Every entry is debt somebody owns -- write `# issue NNNN`\n"
                f"      and the reason, or an exclusion nobody can close becomes\n"
                f"      permanent."
            )
        ids.append(cid)
    if ids != sorted(ids):
        first = next((a for a, b in zip(ids, sorted(ids)) if a != b), "?")
        problems.append(f"  {LEDGER.name}: not sorted (first out of order: `{first}`).")
    dupes = sorted({i for i in ids if ids.count(i) > 1})
    if dupes:
        problems.append(f"  {LEDGER.name}: listed twice: {', '.join(dupes)}")
    return ids, problems


def evaluate(cells: list[dict], have: dict[str, list[str]],
             ledger_text: str) -> tuple[list[str], dict]:
    """(problems, stats). The whole rule, over data the caller supplied."""
    listed, problems = parse_ledger(ledger_text)
    runtime = [c for c in cells if c["tier"] == "Runtime"]
    if not runtime:
        problems.append(
            "  no Runtime cells in interop::CELLS -- refusing to report OK over "
            "an empty set."
        )
    covered = [c for c in runtime if c["test"] in have]
    uncovered = [c for c in runtime if c["test"] not in have]
    unc_ids = {c["id"] for c in uncovered}

    for c in uncovered:
        if c["id"] in listed:
            continue
        problems.append(
            f"  {c['id']} (interop.rs:{c['line']}) is Tier::Runtime and its test\n"
            f"      binary `{c['test']}` is named by no `just` recipe and no\n"
            f"      workflow. Nothing can be aimed at it, so its peer, direction\n"
            f"      and coordinate are a declaration with no measurement behind\n"
            f"      them. Give it a focused runner, or add it to {LEDGER.name}\n"
            f"      with the issue that owns the gap."
        )
    for cid in listed:
        if cid in unc_ids:
            continue
        cell = next((c for c in cells if c["id"] == cid), None)
        if cell is None:
            problems.append(
                f"  {LEDGER.name}: `{cid}` is not a cell in interop::CELLS.\n"
                f"      A stale exclusion is a note nobody can act on, and the\n"
                f"      next cell to take that id inherits it silently."
            )
        elif cell["tier"] != "Runtime":
            problems.append(
                f"  {LEDGER.name}: `{cid}` is {cell['tier']}, not Runtime.\n"
                f"      Only a Runtime cell needs a runner; delete the line."
            )
        else:
            where = ", ".join(have[cell["test"]])
            problems.append(
                f"  {LEDGER.name}: `{cid}` now HAS a runner (`{cell['test']}` at\n"
                f"      {where}). Delete the line -- this ledger only shrinks."
            )
    return problems, {
        "runtime": len(runtime),
        "covered": len(covered),
        "uncovered": len(uncovered),
        "binaries": len({c["test"] for c in runtime}),
        "run_binaries": len({c["test"] for c in covered}),
        "listed": len(listed),
    }


# --- negative controls ---------------------------------------------------

_MINI_RS = '''
/// A doc comment naming ic( and Runtime, which is not a row.
#[rustfmt::skip]
pub const CELLS: &[InteropCell] = &[
    // tests/alpha_e2e.rs -- the one with a lane.
    ic("cell-alpha",
       c(Linux, Rust, Zenoh, Pubsub, Interop, Runtime),
       NativeFixtures, RosEdition(Zenoh), NanoToRos, "alpha_e2e"),
    ic("cell-beta",
       c(Linux, Rust, Zenoh, Service, Interop, Runtime),
       NativeFixtures, RosEdition(Zenoh), BiDir, "beta_e2e"),
    ic("cell-carved",
       c(ZephyrNativeSim, Cpp, Cyclonedds, Qos, Interop,
         CarveOut("no lane; the string mentions ic( and Runtime on purpose")),
       ZephyrWestLeaves, RosEdition(Cyclonedds), BiDir, NO_TEST),
];
'''


def self_test(verbose: bool = False) -> None:
    """Both directions, on synthetic input, on the normal path.

    `check-gate-selftests` requires the call here rather than behind a flag: a
    negative control nobody runs decays into a comment.
    """
    cells = parse_cells(_MINI_RS)
    assert [c["id"] for c in cells] == ["cell-alpha", "cell-beta", "cell-carved"], cells
    assert [c["tier"] for c in cells] == ["Runtime", "Runtime", "CarveOut"], cells
    assert cells[2]["test"] is None, "selftest: NO_TEST was not recognised"
    assert cells[0]["test"] == "alpha_e2e", cells[0]

    # A comment above a row must not become a row, and a `CarveOut` string
    # containing `ic(` must not open one either -- both are in _MINI_RS.
    assert len(cells) == 3, f"selftest: parsed {len(cells)} rows, expected 3"

    for bad, want in (
        ('pub const CELLS: &[InteropCell] = &[\n    ic("x", c(A,B,C,D,E,Runtime), F, G, H),\n];',
         "expected 6"),
        ('pub const CELLS: &[InteropCell] = &[\n    ic("x", c(A,B,C,D,Runtime), F, G, H, "t"),\n];',
         "expected 6"),
        ('pub const CELLS: &[InteropCell] = &[\n    ic("x", c(A,B,C,D,E,Whenever), F, G, H, "t"),\n];',
         "unknown tier"),
        ('pub const CELLS: &[InteropCell] = &[\n];', "0 rows"),
        ("nothing here", "no `pub const CELLS"),
    ):
        try:
            parse_cells(bad)
        except ParseError as e:
            assert want in str(e), f"selftest: wrong error for {want!r}: {e}"
        else:
            raise AssertionError(f"selftest: accepted malformed source ({want})")

    # --- the runner scan -------------------------------------------------
    just = "\n".join([
        "test-ros2:",
        "    cargo nextest run -p nros-tests --test alpha_e2e --no-fail-fast",
        "test-all:",
        "    cargo nextest run -E 'not (group(=ros2) or binary(beta_e2e))'",
        "# a comment naming --test gamma_e2e is not a runner",
    ])
    inv = invocations_in(just)
    assert "alpha_e2e" in inv, "selftest: missed a `--test` invocation"
    assert "beta_e2e" not in inv, "selftest: read a `not (...)` exclusion as a runner"
    assert "gamma_e2e" not in inv, "selftest: read a comment as a runner"
    pos = invocations_in("x:\n    cargo nextest run -E 'binary(beta_e2e)'")
    assert "beta_e2e" in pos, "selftest: missed a positive `binary()` filter"
    assert "delta_e2e" not in invocations_in(
        "      - name: run --test delta_e2e\n        run: just check fast\n"
    ), "selftest: read a workflow step LABEL as a runner"
    assert "eps_e2e" in invocations_in(
        "r:\n    args=(-p nros-tests --test eps_e2e --no-fail-fast)\n"
    ), "selftest: missed a `--test` on a bare shell-array line"

    # --- the verdict -----------------------------------------------------
    have = {"alpha_e2e": ["just/native.just:2"]}
    ok = "cell-beta  # issue 1127 -- no lane yet\n"
    problems, stats = evaluate(cells, have, ok)
    assert problems == [], f"selftest: rejected a well-formed tree: {problems}"
    assert stats == {"runtime": 2, "covered": 1, "uncovered": 1,
                     "binaries": 2, "run_binaries": 1, "listed": 1}, stats

    assert any("named by no `just` recipe" in p
               for p in evaluate(cells, have, "")[0]), (
        "selftest: an unrun Runtime cell with no ledger line passed"
    )
    assert any("names no tracked issue" in p
               for p in evaluate(cells, have, "cell-beta  # later\n")[0]), (
        "selftest: accepted a ledger line with no issue id"
    )
    assert any("not a cell" in p
               for p in evaluate(cells, have, ok + "cell-ghost  # issue 1127\n")[0]), (
        "selftest: accepted a ledger line for a cell that does not exist"
    )
    assert any("not Runtime" in p
               for p in evaluate(cells, have, "cell-carved  # issue 1127\n")[0]), (
        "selftest: accepted a ledger line for a non-Runtime cell"
    )
    both = {"alpha_e2e": ["just/native.just:2"], "beta_e2e": ["just/x.just:9"]}
    assert any("now HAS a runner" in p for p in evaluate(cells, both, ok)[0]), (
        "selftest: a listed cell that gained a lane was not asked to leave"
    )
    assert any("not sorted" in p
               for p in evaluate(cells, {}, "cell-beta  # issue 1127\n"
                                           "cell-alpha  # issue 1127\n")[0]), (
        "selftest: missed an unsorted ledger"
    )
    assert any("listed twice" in p
               for p in evaluate(cells, have, ok + ok)[0]), (
        "selftest: missed a duplicated ledger entry"
    )
    assert any("empty set" in p for p in evaluate([], {}, "")[0]), (
        "selftest: reported OK over zero Runtime cells"
    )
    if verbose:
        print("check-interop-cell-runners: self-test OK")


def main(argv: list[str]) -> int:
    if "--self-test" in argv:
        self_test(verbose=True)
        return 0
    self_test()

    try:
        cells = parse_cells(INTEROP_RS.read_text(encoding="utf-8"))
    except ParseError as e:
        print(f"check-interop-cell-runners: {e}", file=sys.stderr)
        return 1
    have = runners()
    ledger_text = LEDGER.read_text(encoding="utf-8") if LEDGER.is_file() else ""

    if "--list" in argv:
        for c in sorted(cells, key=lambda c: c["id"]):
            where = ", ".join(have.get(c["test"], [])) or "-"
            print(f"{c['tier']:<10} {c['id']:<40} {c['test'] or '(none)':<36} {where}")
        return 0

    problems, stats = evaluate(cells, have, ledger_text)
    if problems:
        print("check-interop-cell-runners: a Runtime interop cell has no runner\n")
        print("\n".join(problems))
        print(
            "\nA `Tier::Runtime` cell in interop::CELLS declares a live peer, a\n"
            "direction and a coordinate. `matrix_fixture_coverage.rs` G1 checks\n"
            "its test FILE exists; this checks something NAMES it. The root\n"
            "`just test-all` sweep does reach these binaries, and does not count:\n"
            "it cannot be aimed at one cell, it brings up no peer, and an unmet\n"
            "precondition inside it becomes a <skipped> nobody can tell from a\n"
            "pass. A live-peer cell needs a runner that sets its peer up."
        )
        return 1

    print(
        f"check-interop-cell-runners OK — {stats['covered']}/{stats['runtime']} "
        f"Runtime cell(s) have a named runner "
        f"({stats['run_binaries']}/{stats['binaries']} test binaries), "
        f"{stats['listed']} awaiting a lane in {LEDGER.name}."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
