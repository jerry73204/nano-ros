#!/usr/bin/env python3
"""phase-320 W2/W3 — validate packages/boards/board-support.toml against evidence.

A support tier that is merely *asserted* drifts. This repo has the receipts:
`book/src/reference/supported-boards.md` claimed ARM FVP was "Tested" under a
legend defining Tested as "boots in CI", while the model is license-walled and
runs in no CI lane at all; and `matrix.rs` carried two FVP `Runtime` cells whose
tests skip on every host. Both are the shape of issue 0232's false green.

So the tier is declared once, in the registry, and CHECKED here against the
structures that actually decide it:

  matrix.rs                       -> does the platform have Runtime cells?
  examples/fixtures.toml          -> does it have fixture rows?
  .github/workflows/nightly.yml   -> is it in the nightly sweep?
  justfile rust-rtos-link-check   -> is it compiled by `just ci`?
  packages/boards/                -> does every board appear exactly once?

phase-375 W1 adds the one input that is not a structure but a person: a tier is
a promise, so it needs enough named maintainers to be one (3/2/1 for tiers
1/2/3), grandfathered for existing rows by scripts/board-maintainer-baseline.json.

Dependency-free on purpose: CI hosts are not guaranteed a TOML library (this
repo's Python is 3.10, so no `tomllib`), and `scripts/build/fixtures-manifest.py`
already establishes regex parsing as the house pattern for exactly this reason.
"""

import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
REGISTRY = ROOT / "packages/boards/board-support.toml"
BASELINE = ROOT / "scripts/board-maintainer-baseline.json"
MATRIX = ROOT / "packages/testing/nros-tests/src/matrix.rs"
FIXTURES = ROOT / "examples/fixtures.toml"
NIGHTLY = ROOT / ".github/workflows/nightly.yml"
JUSTFILE = ROOT / "justfile"
BOARDS_DIR = ROOT / "packages/boards"

VALID_TIERS = {"1", "2", "3", "scaffold", "infra"}

# phase-375 W1 — a tier is a promise, and a promise needs someone who made it.
#
# Rust's target-tier policy is the model: tier 1 requires three named
# maintainers, tier 2 two, tier 3 one, and a target whose maintainers go
# unreachable is DEMOTED rather than quietly kept. What collects abandoned
# targets is that rule, not the directory layout — which is why this repo's
# registry has carried a `maintainers` field since phase-320 W2 while enforcing
# nothing: every one of the 22 rows is `[]`.
#
# Counts are lower than Rust's for tiers 1 and 2 would be for a project this
# size, but the shape is theirs deliberately: more owners for a stronger
# promise, and the weakest promise still has exactly one person to ask.
MAINTAINER_MIN = {"1": 3, "2": 2, "3": 1}


# phase-395 W15 — the COST axis, borrowed from Rust's `Tier 2 with host tools`
# modifier rather than invented as a parallel taxonomy.
#
# The support tier is a PROMISE; the execution class is a FACT about what the
# platform needs in order to run. They are independent, and the relationship
# runs one way: the class bounds the CADENCE a platform can afford, and the
# cadence bounds the tier that can honestly be claimed. The class never CONFERS
# a tier.
#
# `FreertosPosix` is the case that makes the distinction concrete — host-
# executable, so free to run, and a two-cell smoke. Cheap does not mean covered,
# so it is tier 2 and the way up is more cells, not a promotion.
EXECUTION_CLASSES = {
    # runs on a hosted runner with nothing but a host compiler, so a per-merge
    # runtime cadence is affordable
    "host-executable",
    # needs a cross toolchain and usually QEMU; nightly is the realistic floor
    "cross-run",
    # needs a board or a licensed model, so no automated cadence exists at all
    "hardware",
}

# Only a host-executable platform can meet tier 1's per-merge runtime
# obligation. A cross-run platform's evidence can only ever be nightly, which is
# tier 2's cadence by definition — claiming tier 1 for it promises more than the
# cost structure can deliver, which is exactly what `NuttxArm` and
# `FreertosMps2` did while the nightly lane covering them was red.
TIER_1_CLASSES = {"host-executable"}


def tier_supported_by(count):
    """The strongest tier `count` maintainers can carry, or None.

    This is the "demotion is automatic and printed" half of the rule: the gate
    cannot edit the registry, so the demotion it performs is a REFUSAL that
    names the tier the row is actually entitled to. A message that only says
    "not enough maintainers" leaves the author to re-derive the table.
    """
    for tier in ("1", "2", "3"):
        if count >= MAINTAINER_MIN[tier]:
            return tier
    return None


def row_key(entry):
    """Baseline key: the (crate, platform) PAIR the registry is keyed by.

    Not the crate alone — phase-337 W1.c made one crate able to hold several
    rows at different tiers (`nuttx-qemu` is arm at tier 1 and riscv at tier 2),
    so a crate-keyed exemption would grandfather a witness nobody looked at.
    """
    return f"{entry.get('crate', '<no crate key>')}@{entry.get('matrix_platform') or '-'}"


def parse_registry(text):
    """Minimal `[[board]]` reader — scalar strings, ints, and string arrays."""
    entries = []
    cur = None
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith("#") or not line:
            continue
        if line == "[[board]]":
            cur = {}
            entries.append(cur)
            continue
        if cur is None:
            continue
        m = re.match(r'^([a-z_]+)\s*=\s*(.+)$', line)
        if not m:
            continue
        key, val = m.group(1), m.group(2).strip()
        if val.startswith("["):
            items = re.findall(r'"([^"]*)"', val)
            cur[key] = items
        elif val.startswith('"'):
            cur[key] = val.strip('"')
        else:
            cur[key] = val.strip('"')
    return entries


def runtime_platforms(text):
    """Platforms with at least one `Runtime` cell."""
    out = set()
    # cell(...) spans lines; join then scan each invocation.
    for m in re.finditer(r'cell\(\s*(\w+)\s*,(.*?)\)\s*,\s*\n', text, re.S):
        plat, rest = m.group(1), m.group(2)
        if re.search(r'\bRuntime\b', rest):
            out.add(plat)
    return out


def platform_variants(text):
    """The `PlatformId` variants that actually exist.

    W3.a — a tier 1/2/3 board must name a REAL platform. Scaffold boards named
    none by definition, which was the honest encoding of "absent from
    PlatformId". phase-337 W7 then deleted every board in that state, so the
    scaffold tier is currently unpopulated — kept because the state is what
    stops the next unfinished board from reading as support.
    """
    m = re.search(r'pub const fn fixture_tokens\(self\).*?\n    \}', text, re.S)
    body = m.group(0) if m else ""
    return set(re.findall(r'PlatformId::(\w+)\s*=>', body))


def fixture_platforms(text):
    return set(re.findall(r'^platform\s*=\s*"([^"]+)"', text, re.M))


def nightly_tokens(text):
    """Modules the nightly per-platform job can run.

    Parse `runnable="..."`, NOT `all="..."`. The `all` set is computed from the
    lane at runtime (phase-318 W4.e made the sweep derive from `matrix::CELLS`
    so adding a platform there extends the sweep with no second edit), and the
    only literal `all="qemu freertos …"` left in the file is a COMMENT
    describing the old hand-written shape — which this parser matched on its
    first outing. `runnable` is the honest static bound.

    `zephyr` and `native` have their own homes in the tier (the 05:00 Zephyr
    cron and host-tests); the workflow says so and `lane-coverage` asserts it,
    so they count as covered.
    """
    m = re.search(r'^\s*runnable="([^"]+)"', text, re.M)
    toks = set(m.group(1).split()) if m else set()
    toks.update({"zephyr", "native"})
    return toks


def link_check_examples(text):
    body = re.search(r'^rust-rtos-link-check:.*?(?=\n^\w)', text, re.S | re.M)
    if not body:
        return set()
    return set(re.findall(r'cd (examples/[\w./-]+)', body.group(0)))


def write_baseline(reg):
    """Regenerate the grandfather list from rows that do not yet meet the rule."""
    out = {}
    for e in reg:
        tier = str(e.get("tier", ""))
        need = MAINTAINER_MIN.get(tier)
        if need is None:
            continue
        if len(e.get("maintainers") or []) < need:
            out[row_key(e)] = tier
    BASELINE.write_text(json.dumps(dict(sorted(out.items())), indent=2) + "\n")
    print(f"wrote {BASELINE.relative_to(ROOT)}: {len(out)} row(s) grandfathered")
    return 0


def self_test(quiet=False):
    """Negative controls — a gate whose rule never fires proves nothing.

    Each case asserts the rule FIRES, then that the intended escape silences it.
    """
    def run(entry, baseline):
        errors, exempt, improved = [], [], []
        check_maintainers(entry, entry["crate"], str(entry["tier"]), baseline,
                          errors, exempt, improved)
        return errors, exempt, improved

    new_t1 = {"crate": "new-board", "matrix_platform": "Linux", "tier": 1,
              "maintainers": []}
    errors, _, _ = run(new_t1, {})
    assert errors, "a NEW tier-1 row with no maintainer must fail"
    assert "supports tier" not in errors[0], "0 maintainers supports no tier at all"

    # The escape is naming owners, not editing the baseline.
    errors, _, _ = run({**new_t1, "maintainers": ["a", "b", "c"]}, {})
    assert not errors, "three maintainers must satisfy tier 1"
    errors, _, _ = run({**new_t1, "maintainers": ["a", "b"]}, {})
    assert errors and "supports tier 2" in errors[0], \
        "two maintainers must be refused at tier 1 AND told they support tier 2"

    # Grandfathering: the exemption holds at the tier it was granted at...
    errors, exempt, _ = run(new_t1, {"new-board@Linux": "1"})
    assert not errors and len(exempt) == 1, "a baselined row must be exempt"

    # ...and NOT above it. This is the ratchet: a promotion is a new promise.
    promoted = {"crate": "b", "matrix_platform": "P", "tier": 2, "maintainers": []}
    errors, exempt, _ = run(promoted, {"b@P": "3"})
    assert errors and "promotion" in errors[0], \
        "an exemption granted at tier 3 must not cover tier 2"
    # A DEMOTION is the direction the rule wants, so it must never be blocked.
    errors, exempt, _ = run({**promoted, "tier": 3}, {"b@P": "2"})
    assert not errors and exempt, "a demotion must stay exempt"

    # infra/scaffold promise nothing, so they owe nobody.
    for tier in ("infra", "scaffold"):
        errors, _, _ = run({"crate": "x", "tier": tier, "maintainers": []}, {})
        assert not errors, f"{tier} must not require a maintainer"

    # Meeting the rule while baselined is reported, never failed — a ratchet
    # that punishes the good deed gets bypassed too.
    errors, _, improved = run({**new_t1, "maintainers": ["a", "b", "c"]},
                              {"new-board@Linux": "1"})
    assert not errors and len(improved) == 1, "an improved row must be reported, not failed"

    # One crate, two witnesses at different tiers — the key must separate them.
    assert row_key({"crate": "n", "matrix_platform": "A"}) != \
        row_key({"crate": "n", "matrix_platform": "B"}), \
        "rows are keyed by (crate, platform), not crate (phase-337 W1.c)"

    # A missing baseline must FAIL rather than read as an empty exemption set.
    # Empty would be the strictest possible reading and still the wrong one: it
    # fails every unowned row at once, which is the cliff W1 exists to avoid,
    # and it does so with a message about maintainers rather than about the
    # missing file.
    global BASELINE
    real, BASELINE = BASELINE, ROOT / "scripts/does-not-exist.json"
    try:
        errors = []
        assert load_baseline(errors) == {} and errors, \
            "a missing baseline must be an error, not a silent empty set"
    finally:
        BASELINE = real

    if not quiet:
        print("check-board-tiers self-test: OK")
    return 0


def load_baseline(errors):
    """The grandfather list: rows that predate the maintainer rule.

    A gate that fails 22 rows on the day it lands gets bypassed, and a bypassed
    gate is worse than a narrower one that binds — so the rule binds NEW rows
    immediately and existing ones as owners are found. The exemption is a
    committed file, so growing it is a reviewable diff rather than a silent
    default, which is the same shape `scripts/grep-q-baseline.json` uses.
    """
    if not BASELINE.exists():
        errors.append(
            f"missing maintainer baseline {BASELINE.relative_to(ROOT)} — regenerate "
            "with --write-baseline. A missing baseline is a FAILURE, not an empty "
            "exemption set: silently treating it as empty would fail every "
            "unowned row and invite the bypass this ratchet exists to avoid")
        return {}
    try:
        return json.loads(BASELINE.read_text())
    except (OSError, ValueError) as exc:
        errors.append(f"cannot read maintainer baseline: {exc}")
        return {}


def check_maintainers(entry, crate, tier, baseline, errors, exempt, improved):
    """Enforce MAINTAINER_MIN against the grandfather baseline (phase-375 W1)."""
    need = MAINTAINER_MIN.get(tier)
    if need is None:
        # `infra` carries no tier promise and `scaffold` is explicitly NOT
        # supported — neither promises anyone will answer, so neither needs an
        # owner. Requiring one would make the honest states cost more than the
        # dishonest one.
        return
    have = len(entry.get("maintainers") or [])
    key = row_key(entry)
    grandfathered = baseline.get(key)
    if have >= need:
        if grandfathered is not None:
            improved.append((key, tier, have))
        return
    # An exemption is granted at a TIER, and it does not travel upward: a row
    # baselined at tier 3 that is later promoted to tier 2 is making a stronger
    # promise than the one anybody grandfathered. Weaker is fine (a demotion
    # must never be blocked by the rule that asks for demotions).
    # A bigger tier NUMBER is a weaker promise, so the exemption holds when the
    # declared tier is numerically >= the one it was granted at.
    if grandfathered is not None and int(tier) >= int(grandfathered):
        exempt.append((key, tier, have, str(grandfathered)))
        return
    supported = tier_supported_by(have)
    if grandfathered is not None:
        errors.append(
            f"{key}: baselined at tier {grandfathered} but declared tier {tier} — a "
            f"promotion is a NEW promise, so it needs its own owners "
            f"({have} named, tier {tier} needs {need}). The grandfather list covers "
            "the promise that was already there, not a stronger one")
        return
    errors.append(
        f"{key}: declared tier {tier} with {have} maintainer(s); tier {tier} needs "
        f"{need}. "
        + (f"This row supports tier {supported} — declare that, or name "
           f"{need - have} more maintainer(s)."
           if supported else
           "With no maintainer named, no tier promise holds — the honest states "
           "are `scaffold` (unfinished) and `infra` (not a board).")
        + " Recording an owner is the point; inventing one is worse than leaving "
          "the row at the tier it can carry")


def print_tiers(reg):
    """Emit `platform<TAB>tier<TAB>execution_class`, one row per tiered board.

    phase-395 W19 — the ONE derivation. `board-support.toml` is the source of
    truth for which platforms a tier promises, and until now three places
    answered that question independently and disagreed:

        registry            tier 1 = Linux, ZephyrNativeSim, ThreadxLinux
        ci_lane.rs pool     PlatformId::Linux, hardcoded
        lane-filter native  Linux only — and it EXCLUDES zephyr and threadx

    So the lane covered one of the three platforms the tier promised, and the
    run filter actively removed the other two. W15 (a tier claiming more than
    CI delivers), W16 (the pool narrower than the tier) and W18 (scope coarser
    than the selection) are three symptoms of that one split.

    Same shape as `check-flavour-lanes.py --print`, and for the same stated
    reason: one derivation, consumed by the gate and by the lane, so they
    cannot disagree.

    `matrix_platform` is already spelled exactly as the `PlatformId` variant,
    so no name mapping is needed or invented here.
    """
    for e in sorted(reg, key=lambda x: x.get("matrix_platform") or ""):
        plat = e.get("matrix_platform")
        tier = str(e.get("tier", ""))
        cls = e.get("execution_class")
        if not plat or tier not in ("1", "2", "3") or not cls:
            continue
        print(f"{plat}\t{tier}\t{cls}")
    return 0


def main():
    if "--self-test" in sys.argv:
        return self_test()
    # Always, not only behind the flag: a negative control nobody runs decays
    # into a comment, and this rule's whole job is to fire.
    self_test(quiet=True)
    reg = parse_registry(REGISTRY.read_text())
    if "--print-tiers" in sys.argv:
        return print_tiers(reg)
    if "--write-baseline" in sys.argv:
        return write_baseline(reg)
    matrix_txt = MATRIX.read_text()
    rt = runtime_platforms(matrix_txt)
    fx = fixture_platforms(FIXTURES.read_text())
    nl = nightly_tokens(NIGHTLY.read_text())
    variants = platform_variants(matrix_txt)
    lc = link_check_examples(JUSTFILE.read_text())

    errors = []

    # --- completeness, both directions (W3.a) ---------------------------------
    # A board is a directory git TRACKS CONTENT under — not merely a directory.
    #
    # Deleting a board crate does not delete the `target/` its last build left
    # behind: nothing under it is tracked, so git leaves the directory in place.
    # `nros-board-esp32` was dropped in 9211101cb and left 732 MB of residue, and
    # this gate reported it as "exists in packages/boards/ but absent from the
    # registry" — permanently red on `check-fast` for anyone who had ever built
    # that board, with a message pointing at the registry instead of at the
    # leftovers.
    #
    # Tracked-content is the right predicate rather than "has a Cargo.toml":
    # boards come in two shapes, crate boards (`Cargo.toml`) and declarative ones
    # (`nros-board.toml`, e.g. posix/zephyr), and a manifest-name check silently
    # dropped the declarative half.
    tracked = subprocess.run(
        ["git", "ls-files", "-z", "--", str(BOARDS_DIR)],
        cwd=ROOT, capture_output=True, text=True, check=True,
    ).stdout
    _rel = BOARDS_DIR.relative_to(ROOT)
    on_disk = set()
    for p in tracked.split("\0"):
        if not p:
            continue
        parts = Path(p).relative_to(_rel).parts
        # `len > 1` skips files that sit directly in packages/boards/ — notably
        # board-support.toml itself, which is the registry, not a board.
        if len(parts) > 1:
            on_disk.add(parts[0])
    # phase-337 W1.c — rows are keyed by (crate, matrix_platform), not by crate.
    #
    # One crate may serve several WITNESSES at different tiers: after W3 a single
    # `nuttx-qemu` crate carries arm (tier 1) and riscv (tier 2), and after W9
    # one `nros-board-zephyr` carries three. Tier is a per-row promise — "`just
    # ci` exercises it" is true of the arm witness and false of the riscv one —
    # so those cannot share a row, and a list-valued `matrix_platform` would
    # force them to. What must stay unique is the PAIR: two rows claiming the
    # same crate on the same platform are a genuine duplicate.
    #
    # --- the execution class, and what it permits (phase-395 W15) ------------
    for e in reg:
        crate = e.get("crate", "<no crate key>")
        tier = str(e.get("tier", ""))
        if tier not in ("1", "2", "3"):
            continue          # infra / scaffold carry no tier promise
        cls = e.get("execution_class")
        where = f"{crate}@{e.get('matrix_platform') or '-'}"
        if not cls:
            errors.append(
                f"{where}: tier {tier} row has no `execution_class`. It states what "
                f"the platform NEEDS in order to run — one of "
                f"{', '.join(sorted(EXECUTION_CLASSES))} — and without it the tier "
                f"is a promise with no stated cost.")
        elif cls not in EXECUTION_CLASSES:
            errors.append(
                f"{where}: unknown execution_class {cls!r}. Known: "
                f"{', '.join(sorted(EXECUTION_CLASSES))}.")
        elif tier == "1" and cls not in TIER_1_CLASSES:
            errors.append(
                f"{where}: tier 1 requires an execution_class in "
                f"{', '.join(sorted(TIER_1_CLASSES))}, and this row is {cls!r}.\n"
                f"    Tier 1 promises runtime evidence EVERY MERGE. A {cls} platform "
                f"cannot deliver that cadence — its evidence can only be nightly, "
                f"which is tier 2's promise.\n"
                f"    Either demote the row to tier 2, or buy the cadence (a "
                f"self-hosted runner moves cross-run toward per-merge) and then "
                f"promote it back on evidence.")

    # Every crate on disk must still appear at least once; that completeness
    # check is the reason this registry exists.
    listed = [e.get("crate", "<no crate key>") for e in reg]
    keys = [(e.get("crate", "<no crate key>"), e.get("matrix_platform")) for e in reg]
    dupes = {k for k in keys if keys.count(k) > 1}
    for c, plat in sorted(dupes, key=lambda k: (k[0], k[1] or "")):
        errors.append(
            f"{c}: listed more than once for matrix_platform {plat!r}. "
            "A crate MAY appear on several rows, one per witness it serves "
            "(phase-337 W1.c), but not twice for the same one")
    missing = on_disk - set(listed)
    for c in sorted(missing):
        errors.append(
            f"{c}: exists in packages/boards/ but is absent from the registry. "
            "A board enumerated nowhere is the failure this gate exists to catch "
            "(four boards were in this state before phase-320)")
    extra = set(listed) - on_disk
    for c in sorted(extra):
        errors.append(f"{c}: in the registry but no such directory under packages/boards/")

    # phase-337 W2 — the OTHER direction of completeness, and the one the check
    # above structurally cannot do.
    #
    # Everything so far is keyed on board DIRECTORIES, which silently assumes a
    # board is a crate. RFC-0064's whole direction is that it is not: the Zephyr
    # Cortex-M witness is `cmake/zephyr/mps2-an385.conf` and owns no directory
    # under packages/boards/, so it could carry Runtime cells — a tier-1-or-2
    # promise — while being enumerated nowhere, which is the exact failure this
    # gate was written to catch. As boards keep becoming conf bundles (W9), the
    # directory check covers less and less of the rule it is enforcing.
    #
    # So assert it from the matrix side too: a platform that CI actually runs
    # must be somebody's declared promise.
    declared_platforms = {
        e.get("matrix_platform") for e in reg if e.get("matrix_platform")
    }
    for plat in sorted(rt - declared_platforms):
        errors.append(
            f"platform {plat} has Runtime cells but no registry row names it as a "
            "matrix_platform. A board that CI runs must carry a tier promise — "
            "and a board with no directory (a conf bundle) is invisible to the "
            "directory check above, which is why this one exists")

    # phase-375 W4 — THE SMOKE FLOOR, and it is the other direction.
    #
    # Above: a platform CI runs must carry a tier promise. Here: a platform that
    # carries a tier promise must have Runtime evidence for it — at least ONE
    # cell that boots and delivers a message.
    #
    # Measured 2026-09-06 before writing this: every platform already clears the
    # floor (Linux 72 Runtime, ZephyrNativeSim 39, … QemuBaremetal 1). So this
    # gate changes nothing today, which is the point — the distribution was
    # DEFENSIBLE and UNDECLARED, and an undeclared floor is one the next
    # platform can land beneath without anyone noticing. The minimum was already
    # exactly 1; this says so.
    #
    # `scaffold` and `infra` rows are exempt BY TIER, not by name: scaffold
    # promises nothing (that is what the tier is for, and phase-375 W2's
    # `just board-new` starts every board there), and infra rows are not boards.
    # A tier with a number is a promise, and a promise needs evidence.
    floor_exempt = {"scaffold", "infra"}
    for entry in reg:
        plat = entry.get("matrix_platform")
        tier = str(entry.get("tier", ""))
        if not plat or tier in floor_exempt:
            continue
        # `execution_class = "hardware"` is the row DECLARING that CI does not
        # run this platform — a maintainer does, on demand. Requiring a Runtime
        # cell of it would be requiring evidence the row already says nobody
        # collects automatically, and the fix would be to fake the cell.
        #
        # This is an exemption by DECLARED PROPERTY, not by name: any row can
        # claim it, and claiming it is visible in the registry and in
        # `book/src/reference/board-support-tiers.md`. A name-keyed exemption
        # list would have been the second-spelling antipattern, and would have
        # hidden which rows had opted out.
        if entry.get("execution_class") == "hardware":
            continue
        if plat not in rt:
            errors.append(
                f"{entry.get('crate', '?')}: tier {tier} names matrix_platform "
                f"{plat}, which has NO Runtime cell. A tier is a promise that the "
                "platform works; the evidence for that is at least one cell that "
                "boots and delivers a message (phase-375 W4's smoke floor). Add "
                "one, or move the row to tier = \"scaffold\", which promises "
                "nothing.")

    # --- per-entry predicates -------------------------------------------------
    baseline = load_baseline(errors)
    unowned = 0
    exempt = []
    improved = []
    seen_keys = set()
    borrowed: list[tuple[str, str, str]] = []
    for e in reg:
        crate = e.get("crate", "<no crate key>")
        tier = str(e.get("tier", ""))
        if tier not in VALID_TIERS:
            errors.append(f"{crate}: tier {tier!r} is not one of {sorted(VALID_TIERS)}")
            continue
        seen_keys.add(row_key(e))
        if not e.get("maintainers"):
            unowned += 1
        check_maintainers(e, crate, tier, baseline, errors, exempt, improved)
        if tier == "infra":
            continue

        plat = e.get("matrix_platform")
        if plat and plat not in variants:
            errors.append(
                f"{crate}: matrix_platform {plat!r} is not a PlatformId variant "
                f"({sorted(variants)})")
        has_runtime = plat in rt if plat else False

        if tier in ("1", "2"):
            if not plat:
                errors.append(f"{crate}: tier {tier} must name a matrix_platform")
            elif not has_runtime:
                errors.append(
                    f"{crate}: declared tier {tier} but platform {plat} has NO Runtime cell "
                    "in matrix.rs — tiers 1 and 2 promise asserted runtime coverage")
        if tier == "3" and has_runtime:
            # A platform's Runtime cells prove SOME board. When a DIFFERENT row
            # already claims that platform at tier 1/2, those cells are its
            # witness, and a build-only board sharing the token is not thereby
            # proven — so the contradiction this rule names does not exist.
            #
            # phase-372's `nros-board-s32z270-freertos` is the first of these:
            # a Cortex-R52 bundle that borrows `FreertosMps2` for the freertos
            # family lane "until a hardware witness exists", while the MPS2 board
            # owns that platform's cells. Read as a function platform -> tier,
            # the row looks like the FVP defect; read against the registry, it is
            # honest, and its `notes` already say so.
            #
            # Inferred from rows that already exist rather than a new key: a
            # `borrowed = true` flag would be a claim a board makes about itself,
            # and this is a fact about who owns the cells.
            owner = next(
                (o.get("crate") for o in reg
                 if o.get("crate") != crate
                 and o.get("matrix_platform") == plat
                 and str(o.get("tier", "")) in ("1", "2")),
                None,
            )
            if owner:
                borrowed.append((crate, plat, owner))
            else:
                errors.append(
                    f"{crate}: declared tier 3 (build-only) but platform {plat} HAS Runtime "
                    "cells and NO other row claims that platform at tier 1/2, so those cells "
                    "are this board's own witness. Either it is really tier 2, or those cells "
                    "overstate what the lane can do — the exact defect phase-320 W1.a fixed "
                    "for FVP")
        if tier == "scaffold":
            if has_runtime:
                errors.append(
                    f"{crate}: declared scaffold but platform {plat} has Runtime cells")
            if plat and any(t in fx for t in (plat,)):
                errors.append(f"{crate}: declared scaffold but has fixture rows")
        if tier == "1":
            tok = e.get("nightly_token")
            # `Linux` (was `Native` until phase-337 W8.b) is exempt because
            # `just ci` runs it DIRECTLY on every push — stronger than a nightly
            # build, which is what the token stands in for everywhere else.
            if plat != "Linux" and (not tok or tok not in nl):
                errors.append(
                    f"{crate}: declared tier 1 but no nightly lane covers it "
                    f"(nightly_token={tok!r}, workflow sweep={sorted(nl)}). Tier 1 promises a "
                    "regression fails before merge; without a lane nothing can fail")
            ex = e.get("link_check_example")
            if ex and ex not in lc:
                errors.append(
                    f"{crate}: link_check_example {ex!r} is not built by rust-rtos-link-check")

    for crate, plat, owner in sorted(borrowed):
        print(f"  tier-3 {crate} BORROWS platform {plat}; its Runtime cells are "
              f"{owner}'s witness, not this board's")
    print(f"board-support registry: {len(reg)} entries, {len(on_disk)} directories")
    print(f"  platforms with Runtime cells: {', '.join(sorted(rt)) or '(none)'}")
    print(f"  nightly sweep: {', '.join(sorted(nl)) or '(none)'}")
    # A baseline key naming no row is dead weight, and dead weight is how a
    # ratchet slips: rename a row and its exemption survives, attached to
    # nothing, ready to grandfather whatever later takes the name back.
    for key in sorted(set(baseline) - seen_keys):
        errors.append(
            f"maintainer baseline names {key}, which is not a row in the registry. "
            "An exemption for a row that no longer exists can only ever be "
            "reclaimed by accident — drop it (--write-baseline)")

    print(f"  maintainer rule: tier 1 needs {MAINTAINER_MIN['1']}, tier 2 "
          f"{MAINTAINER_MIN['2']}, tier 3 {MAINTAINER_MIN['3']} "
          f"({unowned} of {len(reg)} rows have no maintainer)")
    if exempt:
        # The count every run, the roster only near the end. Thirteen lines on
        # every `check-fast` is noise a reader learns to scroll past, and the
        # roster is a committed file anyone can read; but once the ratchet is
        # nearly done, naming who is left is exactly the nudge that finishes it.
        print(f"  {len(exempt)} row(s) grandfathered by "
              f"{BASELINE.relative_to(ROOT)} — the rule binds them as owners are "
              "found, and binds a NEW board immediately")
        if len(exempt) <= 5:
            for key, tier, have, at in sorted(exempt):
                at_note = "" if at == tier else f", baselined at tier {at}"
                print(f"    still unowned: {key} (tier {tier}, "
                      f"{have} maintainer(s){at_note})")
    if improved:
        for key, tier, have in sorted(improved):
            print(f"  {key} now has {have} maintainer(s) and meets tier {tier} — "
                  "drop it from the baseline (--write-baseline); the list only shrinks")

    if errors:
        print("\n[FAIL] board support tiers disagree with the evidence:", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        return 1
    print("Board support tiers match the evidence.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
