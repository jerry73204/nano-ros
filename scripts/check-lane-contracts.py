#!/usr/bin/env python3
"""A tier's affordability claim must be TRUE, not aspirational — phase-395.

CLAUDE.md says `just ci-l1` is "compile + unit, NO FIXTURES … it needs no
fixture build, no SDK, no QEMU and no cross toolchain". That claim was FALSE,
and had been for as long as it had been written down: `ci-l1` -> `check-build`
-> `check-source-gates` -> `platform_header_compile`, which resolves a fixture.

Nothing noticed, because the push lane runs `check-fast` alone. It surfaced only
when the PR lane became a REQUIRED status check and every CI run went red on
`BuildFailed("Test fixture binary not prebuilt")` — a required check that could
never pass, which is the frozen-repo failure this campaign has now met four
times.

A tier claim nobody can check is a promise about COST that quietly stops being
true, and cost is the whole reason the tier exists: an instruction nobody can
afford gets followed selectively, which is worse than a smaller instruction
followed honestly.

THE DISTINCTION THAT MATTERS: COMPILE-STAGE vs RUNTIME

Not every `require_*` is equal, and collapsing them would ban something
legitimate:

  * COMPILE-STAGE — `require_compile_check{,_bin}`. A `.compile-ok` stamp from a
    `cargo check` of a small template crate: ~13 s, no SDK, no emulator. That is
    a compile artifact, so it BELONGS in a compile tier. The gate simply has to
    PRODUCE it rather than assume someone else did — which is the fix that ships
    alongside this gate.
  * RUNTIME — `require_entry_binary`, `require_cmake_fixture`,
    `require_idf_fixture`, `require_west_fixture`. These need
    `build-test-fixtures`: an SDK, a cross toolchain, sometimes QEMU. Nothing
    reachable from L1 may touch one, because L1's entire value is being
    affordable before every push.

Usage::

    check-lane-contracts.py [--selftest]
"""

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JUSTFILE = os.path.join(ROOT, "justfile")
TESTS_DIR = os.path.join(ROOT, "packages", "testing", "nros-tests", "tests")

# Tiers that promise affordability, and what each promises.
#
# AUTHORED, and deliberately so: these two make a claim in CLAUDE.md that a
# reader relies on. Everything else this gate checks is DERIVED from the
# workflows (see `ci_job_lanes`), because a hand-maintained list of "tiers CI
# runs" is exactly what went stale — phase-395 put `check-build` on the merge
# group, nothing here knew, and the required check was red for every pull
# request for a day (phase-396).
# Keyed by the recipe name as `just --show` resolves it. Both tiers now live in
# MODULES (`just ci l1`, `just check fast`), so the key is the module path —
# spelling them the old flat way made this gate report "no such recipe" for a
# lane that had simply moved, which is the same false-staleness it exists to
# catch elsewhere.
LANES = {
    # phase-410 W4 — `ci::l1` is now `ci::gate`. It visits no coordinates, so it
    # was never a rung on the breadth ladder; `l1` survives as a forwarder and
    # the CLAIM belongs to the recipe that holds the body.
    "ci::gate": "compile + unit, NO fixture build (CLAUDE.md)",
    # BUILD depth over the tier-2 breadth. Same affordability shape as the gate
    # — it must not resolve a fixture — for a different reason: it is the
    # per-merge lane, run by `queue.yml` on `merge_group`, so a fixture
    # dependency here would make every merge pay a fixture build. (It used to
    # name build-wide.yml, which ran the SAME recipe on push to main until
    # phase-413 W1 collapsed the duplicate; that file is dispatch-only now.)
    "ci::_matrix-build": "cross build + link, NO fixture build (phase-410)",
    "check::fast": "buildless and source-only",
}

WORKFLOW_DIR = os.path.join(ROOT, ".github", "workflows")

# Recipes that PRODUCE the artifacts a tier may need. A CI job that runs a tier
# needing one of these must run the producer too — in the same job, since
# nothing carries a build dir between jobs.
PRODUCERS = (
    "build-test-fixtures",
    "build-compile-check-fixtures",
    "generate-bindings",
    "build-examples",
)


PRODUCER_CALL = re.compile(r"just\s+(build-test-fixtures|build-compile-check-fixtures"
                           r"|generate-bindings|build-examples)")


def required_producers(recipes, reached):
    """{producer: recipe} for every recipe in `reached` that HARD-FAILS telling
    you to run a producer first.

    The declaration is the recipe's own remediation text. `native::check` ends:

        echo "  Run 'just generate-bindings' (or 'just build-test-fixtures')..."
        exit 1

    which is a precondition stated in the only place it was ever stated. Keyed
    on the pair (mentions a producer, can exit non-zero) so an ADVISORY mention
    — a comment, a hint printed on success — does not count.
    """
    out = {}
    for r in reached:
        body = "\n".join(recipes.get(r, {}).get("body", []))
        if "exit 1" not in body and "exit 2" not in body:
            continue
        for m in PRODUCER_CALL.finditer(body):
            out.setdefault(m.group(1), r)
    return out


def ordered_setup_requirements(recipes):
    """{recipe: {setup-X}} — "X must run before this", per the JUSTFILE's own
    dependency lists (issue 1030).

    `just` runs a recipe's dependencies in order, so

        _codegen: setup-launch-resolve generate-bindings _require-leaf-includes

    states that `generate-bindings` needs `setup-launch-resolve` first. That is
    a declaration someone wrote for their own reasons, which makes it a
    CROSS-CHECK rather than a table this gate maintains and lets rot — the
    distinction the `rmw-api-parity` map got wrong.

    Complements `required_producers`, which reads a recipe's hard-failing
    remediation TEXT. Neither subsumes the other: `generate-bindings` fails
    inside `nros sync`, a Rust binary whose message no recipe body contains, so
    only the ordering sees it; a recipe that hard-fails without appearing in any
    wrapper is only seen by the text rule.
    """
    out = {}
    for spec in recipes.values():
        deps = spec.get("deps", [])
        for i, d in enumerate(deps):
            if not d.startswith("setup-"):
                continue
            for later in deps[i + 1:]:
                out.setdefault(later, set()).add(d)
    return out


def workflow_jobs():
    """[(workflow, job, [just recipes the job runs], [producers it runs])].

    Text-scanned rather than YAML-parsed: the `run:` blocks are shell, the
    `if:` guards are GitHub expressions, and this gate only needs "which
    recipes does this job invoke" — a question the text answers exactly.
    """
    out = []
    if not os.path.isdir(WORKFLOW_DIR):
        return out
    job_re = re.compile(r"^  ([A-Za-z0-9_-]+):\s*$")
    just_re = re.compile(r"(?:^|[;&|]|\s)just\s+((?:[a-z0-9-]+\s*)+)")
    for fn in sorted(os.listdir(WORKFLOW_DIR)):
        if not fn.endswith((".yml", ".yaml")):
            continue
        path = os.path.join(WORKFLOW_DIR, fn)
        with open(path, encoding="utf8", errors="replace") as fh:
            lines = fh.read().split("\n")
        # Workflow-level events, used when a step carries no `if:`.
        wf_events = set(re.findall(r"^\s{2}(pull_request|merge_group|push|schedule"
                                   r"|workflow_dispatch|workflow_run)\s*:", "\n".join(lines),
                                   re.M))
        job, recipes = None, []
        step_if = ""
        if_indent = None
        in_jobs = False
        for line in lines:
            if line.startswith("jobs:"):
                in_jobs = True
                continue
            if not in_jobs:
                continue
            m = job_re.match(line)
            if m:
                if job:
                    out.append((fn, job, recipes))
                job, recipes, step_if = m.group(1), [], ""
                if_indent = None
                continue
            # A new step resets the guard; `if:` inside a step sets it. Steps
            # are the granularity that matters — one job runs `check-fast` on
            # every event and `check-build` on only some, and treating the job
            # as uniform is how a nightly-only tier reads as a required one.
            if re.match(r"^\s*- (name|uses|run):", line):
                if re.match(r"^\s*- (name|uses):", line):
                    step_if = ""
                    if_indent = None
            # A FOLDED guard carries its events on the following lines:
            #
            #     if: >-
            #       ${{ contains(fromJSON('["pull_request","merge_group"]'), ...
            #
            # Reading only the `if:` line finds no event name in it, so
            # `_events_of` returned "every event the workflow declares". For the
            # tier rules that over-includes, which their doc calls the safe
            # direction; for the producer-coverage rule (issue 1030) it is the
            # UNSAFE one — a producer credited with every event covers every
            # consumer, and the rule can never fire. It could not see the very
            # step whose guard was too narrow.
            #
            # So keep appending while the block is more-indented than the `if:`
            # key and has not started another mapping key.
            m_if = re.match(r"^(\s+)if:", line)
            if m_if:
                step_if = line
                if_indent = len(m_if.group(1))
            elif step_if and if_indent is not None:
                stripped = line.strip()
                indent = len(line) - len(line.lstrip())
                if (stripped
                        and indent > if_indent
                        and not re.match(r"^(name|uses|run|with|env|shell|id|"
                                         r"continue-on-error|timeout-minutes):",
                                         stripped)):
                    step_if += "\n" + line
                else:
                    if_indent = None
            # Only `run:` shell counts. A step NAMED "just check build + no_std"
            # is a label, and reading it as an invocation attributed the recipe
            # to every event the workflow has — which is precisely the
            # nightly-vs-required distinction this is here to make.
            if re.match(r"^\s*-?\s*(name|uses):", line):
                continue
            if job and "just " in line and not line.lstrip().startswith("#"):
                ev = _events_of(step_if, wf_events)
                for jm in just_re.finditer(line):
                    recipes.extend(
                        (w, ev) for w in jm.group(1).split()
                        if not w.startswith("-")
                    )
        if job:
            out.append((fn, job, recipes))
    return [
        (w, j, r, [x for x, _ in r if x in PRODUCERS]) for w, j, r in out
    ]


def lane_invocations(words, recipes_map):
    """{(qualified recipe, events)} for the lanes a job's `run:` lines invoke.

    `workflow_jobs` yields a FLAT word stream, and a lane is often two words:
    `just check build`, `just ci tier1`, `just native build-workspace-fixtures`.
    Testing each word against the recipe map on its own is wrong twice over.

      * It MISSES every module lane. Neither `ci` nor `tier1` is a recipe, so
        `just ci tier1` resolved to nothing and host-tests.yml's tier-1 lane —
        the one that runs `just check`, the whole default tier — was invisible
        to this rule.
      * It MISATTRIBUTES. `just check api-parity` has two candidate recipes and
        the bare-word test picks the wrong one: the ROOT `api-parity lang=""`
        is a parameterised REPORT, while the gate is `check::api-parity`. Their
        closures differ, so the rule was answering about a recipe the workflow
        never runs.

    So: pair a word with the next one when `<a>::<b>` is a real recipe, and only
    then fall back to the bare word. Pairing is tried FIRST because the module
    spelling is the more specific of the two, which is what makes the
    `api-parity` collision resolve the way the workflow means it.

    Measured 2026-09-05: 3 lanes resolved before, 15 after, 0 findings either
    way — coverage, not a new baseline.
    """
    out, i = set(), 0
    while i < len(words):
        word, events = words[i]
        if (i + 1 < len(words) and words[i + 1][1] == events
                and f"{word}::{words[i + 1][0]}" in recipes_map):
            out.add((f"{word}::{words[i + 1][0]}", tuple(sorted(events))))
            i += 2
            continue
        if word in recipes_map:
            out.add((word, tuple(sorted(events))))
        i += 1
    return out


# Events on which a merge cannot happen without the step passing.
GATING_EVENTS = {"pull_request", "merge_group"}

# Events that produce a VERDICT nobody is blocked on — post-submit, the nightly,
# and a hand-fired run.
REPORTING_EVENTS = {"push", "schedule", "workflow_dispatch"}

# The tier rules' scope. WIDENED beyond `GATING_EVENTS` — issue 1030's stated
# gap, closed with a measurement rather than an argument.
#
# 1030 recorded the narrow scope as a deliberate severity call and did not
# propose reversing it: "a broken tier on `schedule` is a bad nightly, a broken
# tier on `merge_group` is a repository nobody can merge into". That is true
# about SEVERITY and was mistaken as a reason to look away. A lane whose job
# cannot build what it resolves is broken on every event it runs on; on the
# nightly it simply fails silently, which is the property issue 1040 is about —
# reds accumulate in a lane nobody is blocked on until someone finds five at
# once.
#
# MEASURED 2026-09-05 before widening: the scope goes from 7 merge-gating lane
# invocations to 49, and the finding count goes from 0 to 0. There is no cost
# to weigh against the severity call, because nothing new fails. What the
# narrow scope actually bought was blindness — the scheduled gate was red four
# consecutive nights on exactly this shape (1030's own defect) and the tier
# rules were structurally unable to look at it.
#
# The severity distinction is KEPT, in the output rather than in the scope:
# every finding is labelled `[gating]` or `[report]`, so "a repository nobody
# can merge into" still reads differently from "a bad nightly". Dropping the
# lane was never the only way to say that.
#
# `workflow_run` is deliberately absent. `queue-notify.yml` runs on it and its
# `just ci l1` is TEXT — a line in the comment it posts telling a human what to
# run — so scanning it attributes lanes to a workflow that runs none of them.
SCANNED_EVENTS = GATING_EVENTS | REPORTING_EVENTS


EVENT = r"(?:pull_request|merge_group|push|schedule|workflow_dispatch|workflow_run)"
# `contains(fromJSON('["a","b"]'), github.event_name)` — the shape every step
# guard in this repo uses, and the only one that is unambiguous.
_IN_LIST = re.compile(r"fromJSON\(\s*'\[([^\]]*)\]'\s*\)\s*,\s*github\.event_name")
# `github.event_name == 'x'` / `!= 'x'` — only when `github.event_name` is the
# left side. A bare quoted event elsewhere in the expression is NOT a claim
# about which events run.
_EQ = re.compile(r"github\.event_name\s*==\s*['\"](" + EVENT + r")['\"]")
_NE = re.compile(r"github\.event_name\s*!=\s*['\"](" + EVENT + r")['\"]")


def _events_of(step_if, wf_events):
    """Which events this guard can run on. Over-approximates on purpose.

    phase-396 follow-up. The first version keyed on "the guard mentions an
    event" and treated `!=` as exclusion unless an `==` appeared anywhere. That
    is wrong on the one guard that matters most — `pr-checks`'s `check` job:

        always() && (github.event_name != 'pull_request'
                     || needs.changes.outputs.code == 'true')

    The `==` there is about `needs.changes.outputs.code`, not about an event, so
    the old rule saw "both operators" and fell through to `named & wf_events`,
    concluding the job runs on `pull_request` ONLY. It runs on every event, and
    merely narrows the pull-request case to code-touching diffs.

    So: read only comparisons whose left side IS `github.event_name`, and when
    an exclusion is OR-ed with anything else, do not treat it as an exclusion —
    the other arm can still admit the event.

    The bias is deliberate. This feeds "does this lane gate a merge", where
    over-including costs an extra check and under-including silently drops a
    gating lane from the contract. Fail toward checking more.
    """
    if not step_if:
        return set(wf_events)
    g = str(step_if)

    in_list = _IN_LIST.search(g)
    if in_list:
        named = set(re.findall(r"['\"](" + EVENT + r")['\"]", in_list.group(1)))
        if "!" in g[: in_list.start()].rstrip()[-1:]:
            return set(wf_events) - named
        return (named & set(wf_events)) or named

    eq = set(_EQ.findall(g))
    if eq:
        return (eq & set(wf_events)) or eq

    ne = set(_NE.findall(g))
    if ne:
        # An exclusion that is one arm of an `||` does not exclude: the other
        # arm can admit the event. Only a lone negation narrows.
        if "||" in g:
            return set(wf_events)
        return set(wf_events) - ne

    return set(wf_events)

RUNTIME_RESOLVERS = (
    "require_entry_binary",
    "require_cmake_fixture",
    "require_idf_fixture",
    "require_west_fixture",
    "require_west_leaf_in_lane",
)
# Legitimate in a compile tier, when the gate produces them itself.
COMPILE_RESOLVERS = ("require_compile_check", "require_compile_check_bin")

# Parameters may be UPPERCASE and their defaults quoted — `check JOBS="75%":`
# is a real recipe header, and the old `[a-z_]+=\S*` matched neither the
# name nor the `"75%"`. It therefore skipped `native::check`, which is the
# one recipe whose missing precondition froze the merge queue (phase-396).
# phase-410 W4 — a leading `_` is allowed. `[private]` recipes in this repo are
# `_`-prefixed by convention (`_lane-gate`, `_require-fixtures`,
# `_matrix-build`), and this pattern could not match ANY of them. So the closure
# below silently stopped at every private boundary — `ci::matrix` calls
# `just _lane-gate tier2` and the walk never followed it. A gate that cannot see
# the preconditions it exists to check is the vacuous-pass shape this file's own
# header describes.
RECIPE = re.compile(
    r"^(_?[a-z][a-z0-9-]*)"
    r"(?:\s+[A-Za-z_][A-Za-z0-9_]*(?:=(?:\"[^\"]*\"|'[^']*'|\S+))?)*"
    r"\s*:(.*)$"
)
CARGO_TEST = re.compile(r"--test\s+([A-Za-z0-9_]+)")


def parse_justfile():
    """{recipe: {deps, body}} across the root justfile AND `just/*.just`.

    phase-396 W5 — modules were invisible, and that is where the defect lived.
    `check-build`'s last dependency is `native::check`, which hard-requires
    generated message bindings; the closure walk stopped at the root justfile,
    so the gate could not see it and reported the tier clean while the required
    check was red for every pull request.

    Module recipes are keyed `<module>::<recipe>`, which is how the root
    justfile already spells them.
    """
    recipes = {}
    for mod, path in _just_sources():
        try:
            with open(path, encoding="utf8", errors="replace") as fh:
                recipes.update(_parse_one(fh.read().split("\n"), mod))
        except OSError:
            continue
    return recipes


IMPORT = re.compile(r"^import\??\s+['\"]([^'\"]+)['\"]", re.M)


def _just_sources():
    """[(module or None, path)]: the root justfile, every `just/*.just` module,
    and every file any of them `import`s — keyed to the IMPORTING module.

    Issue 1163 — the gates were invisible. Phase-399 split the 200 gate recipes
    out of `just/check.just` into `just/check/*.just` behind `import`, which
    MERGES definitions (a `mod` namespaces them). The walk below stopped at
    `just/*.just`, so `check::compile-smoke`, `check::c`, `check::build` and
    every other gate were not recipes to this file: a lane invocation of
    `just check compile-smoke` resolved to nothing, and the closure of
    `check::fast` was its two forwarders. The tier rules reported OK over a
    tree they could not see, which is the vacuous shape this file already
    carries three cases against.

    Beside the justfile being parsed, NOT under ROOT: the self-tests redirect
    `JUSTFILE` to a temp tree, and a module dir pinned to ROOT made them read
    the real repo's modules while claiming to test a synthetic justfile. Same
    answer in production (the root justfile IS in ROOT), honest under test.
    """
    mod_dir = os.path.join(os.path.dirname(os.path.abspath(JUSTFILE)), "just")
    roots = [(None, os.path.abspath(JUSTFILE))]
    if os.path.isdir(mod_dir):
        roots += [
            (fn[:-5], os.path.join(mod_dir, fn))
            for fn in sorted(os.listdir(mod_dir)) if fn.endswith(".just")
        ]
    out, seen = [], set()
    for mod, path in roots:
        # Depth-first from each root so a file the ROOT justfile imports
        # (`just/sdk-env.just`) is keyed bare, as `just` keys it — not as the
        # module the directory listing would otherwise make of it.
        stack = [path]
        while stack:
            cur = stack.pop()
            if cur in seen:
                continue
            seen.add(cur)
            out.append((mod, cur))
            try:
                with open(cur, encoding="utf8", errors="replace") as fh:
                    text = fh.read()
            except OSError:
                continue
            for m in IMPORT.finditer(text):
                stack.append(os.path.normpath(
                    os.path.join(os.path.dirname(cur), m.group(1))))
    return out


VARIABLE = re.compile(r'^([A-Za-z_][A-Za-z0-9_-]*)\s*:=\s*"([^"]*)"\s*$', re.M)


def just_variables():
    """{name: {values}} for every string-literal `NAME := "..."` across the
    justfile, its modules and their imports. A SET of values, because the
    point of reading one is to find out whether it has one spelling."""
    out = {}
    for _mod, path in _just_sources():
        try:
            with open(path, encoding="utf8", errors="replace") as fh:
                text = fh.read()
        except OSError:
            continue
        for m in VARIABLE.finditer(text):
            out.setdefault(m.group(1), set()).add(m.group(2))
    return out


def _parse_one(lines, mod):
    """Parse one justfile; `mod` prefixes every recipe name when not None."""
    recipes, cur = {}, None
    i = 0
    while i < len(lines):
        line = lines[i]
        m = RECIPE.match(line)
        if m and not line.startswith((" ", "\t")):
            cur = f"{mod}::{m.group(1)}" if mod else m.group(1)
            dep_text = m.group(2)
            # A trailing `\` continues the dependency list onto later lines —
            # `check-build` spells its 30-odd dependencies that way.
            while dep_text.rstrip().endswith("\\") and i + 1 < len(lines):
                i += 1
                dep_text = dep_text.rstrip()[:-1] + " " + lines[i]
            deps = [d for d in re.split(r"[\s()]+", dep_text) if d and not d.startswith("#")]
            # A bare dep inside a module refers to that module's own recipe.
            if mod:
                deps = [d if "::" in d else f"{mod}::{d}" for d in deps]
            recipes[cur] = {"deps": deps, "body": []}
        elif cur and line.startswith((" ", "\t")):
            recipes[cur]["body"].append(line)
        elif not line.strip():
            pass
        else:
            cur = None
        i += 1
    return recipes


# `just a b c` inside a recipe body. Header dependencies are NOT the only edge
# in this justfile, and assuming they were made this gate pass over everything:
# `ci-l1` declares no dependencies at all and calls `@just check cli-fresh
# check-fast check-build check-api-parity` in its body, so a header-only walk
# found a closure of size 1 and cheerfully reported "0 test target(s)" — a gate
# that verified nothing while printing OK.
JUST_CALL = re.compile(r"^\s*@?just\s+((?:[a-z0-9_:-]+)(?:\s+[a-z0-9_:-]+)*)\s*$")


def closure(recipes, root):
    seen, stack = set(), [root]
    while stack:
        r = stack.pop()
        if r in seen or r not in recipes:
            continue
        seen.add(r)
        stack.extend(recipes[r]["deps"])
        for line in recipes[r]["body"]:
            m = JUST_CALL.match(line)
            if not m:
                continue
            args = m.group(1).split()
            stack.extend(args)
            # `just <mod> <recipe>` is ONE edge to `<mod>::<recipe>`, not two
            # edges to recipes named `<mod>` and `<recipe>`. The CI ladder
            # became a module (`mod ci 'just/ci.just'`), and the flat `ci-l1`
            # forwarder is now `@just ci l1` — which this loop read as two
            # names that resolve to nothing, so the tier's closure came back
            # EMPTY and the gate reported OK over zero test targets. That is
            # the same vacuous shape the wrapped-invocation case below covers,
            # reached by a different route: a rename, not a line break.
            for i in range(len(args) - 1):
                stack.append(f"{args[i]}::{args[i + 1]}")
    return seen


def _join_continuations(body):
    """Fold backslash-continued shell lines into one logical line.

    Issue 0922 — the scan below is per-LINE and keys on the line ALSO containing
    `cargo test`/`nextest`. A wrapped invocation puts the verb on one line and
    its `--test` targets on the next, so the gate saw the recipe, found no test
    target in it, and reported OK over an empty set — the vacuous shape this
    file already carries a case against one level up. `ci-l1` gained exactly
    such a recipe (`test-lane-contracts`) and it was invisible.
    """
    out, pending = [], ""
    for line in body:
        stripped = line.rstrip()
        if stripped.endswith("\\"):
            pending += stripped[:-1] + " "
            continue
        out.append(pending + stripped)
        pending = ""
    if pending:
        out.append(pending)
    return out


def tests_invoked(recipes, names):
    """{test_name: recipe} for every `--test NAME` in the closure's bodies."""
    out = {}
    for r in names:
        for line in _join_continuations(recipes.get(r, {}).get("body", [])):
            if "cargo test" not in line and "nextest" not in line:
                continue
            for m in CARGO_TEST.finditer(line):
                out.setdefault(m.group(1), r)
    return out


ID_RE = re.compile(r'require_compile_check(?:_bin)?\(\s*"([A-Za-z0-9_]+)"')


def stamp_ids_used(test_name):
    """The compile-check fixture ids a test names literally."""
    path = os.path.join(TESTS_DIR, f"{test_name}.rs")
    if not os.path.exists(path):
        return set()
    with open(path, encoding="utf8", errors="replace") as fh:
        text = fh.read()
    ids = set(ID_RE.findall(text))
    # Most tests keep the ids in a `const FOO: &[&str] = &["a", "b"];` and pass
    # the loop variable, so the literal call site names nothing. Fall back to
    # every string literal that the manifest actually knows as a fixture id —
    # over-broad is safe here, since an id that is not in the manifest is
    # dropped by the caller.
    ids |= set(re.findall(r'"([a-z][a-z0-9_]{3,})"', text))
    return ids


def builder_of_ids():
    """{fixture id: builder} from the manifest — READ, never inferred."""
    manifest = os.path.join(ROOT, "examples", "fixtures.toml")
    if not os.path.exists(manifest):
        return {}
    try:
        import tomllib
    except ModuleNotFoundError:  # python < 3.11
        try:
            import tomli as tomllib
        except ModuleNotFoundError:
            return {}
    with open(manifest, "rb") as fh:
        d = tomllib.load(fh)
    return {
        f["id"]: f.get("builder")
        for f in d.get("compile_check_fixture", [])
        if f.get("id")
    }


def lane_filters(recipes, reached):
    """Every NROS_COMPILE_CHECK_LANES=... value the lane sets, as a set of
    builder names. Empty set means the lane never filters (all builders)."""
    out, filtered = set(), False
    for r in reached:
        for line in recipes.get(r, {}).get("body", []):
            m = re.search(r"NROS_COMPILE_CHECK_LANES=([A-Za-z0-9,_-]+)", line)
            if m:
                filtered = True
                out |= {x for x in re.split(r"[,\s]+", m.group(1)) if x}
    return out if filtered else None


def resolvers_used(test_name):
    path = os.path.join(TESTS_DIR, f"{test_name}.rs")
    if not os.path.exists(path):
        return set(), False
    with open(path, encoding="utf8", errors="replace") as fh:
        text = fh.read()
    return {r for r in RUNTIME_RESOLVERS + COMPILE_RESOLVERS if r in text}, True


# ---- issue 1163 — a crate that SHIPS in a non-default shape must be compiled
# in that shape by a merge-gating lane ----------------------------------------
#
# `nros-c`'s default feature is `panic-platform`, which compiles almost none of
# the C surface; what ships is `std,rmw-cffi,platform-posix,ros-humble`, and
# every consumer says so with `default-features = false`. PR #329 landed a
# second `#[no_mangle] nros_node_get_fully_qualified_name` inside an
# `rmw-cffi` region: `cargo check -p nros-c` green, the shipped shape E0428.
# `compile-smoke` (the PR gate) checked defaults; `check-c` compiles the
# shipped set but runs on schedule; `test-unit` on the merge group compiles
# defaults again. Green with no signal capacity, on every lane that gates.
#
# The subject set is DERIVED, not authored: any crate under `packages/` with a
# non-empty default feature set that EVERY consumer takes `default-features =
# false` is a crate whose default is not what ships. What the shipped shape IS
# has to be authored, and the one place it may be authored is a `just`
# variable, because the lanes read that variable and a second spelling drifts.

# crate -> the justfile variable holding its shipped feature list.
SHIPPED_SHAPES = {
    "nros-c": "C_API_SHIPPED_FEATURES",
    "nros-cpp": "C_API_SHIPPED_FEATURES",
}

# A subject with no host shape at all, and why. A reason, not a bare name:
# an exemption without one is a rule with a hole nobody can re-check.
SHIPPED_EXEMPT = {
    "nros-board-nuttx": (
        "cross-only — a NuttX board crate has no host shape; its one consumer "
        "is the QEMU board family, built by the nuttx fixture lane, a runtime "
        "tier no merge-gating job runs"
    ),
}

# Where a crate's dependency tables live in a manifest, `target.*` included.
_DEP_TABLES = ("dependencies", "dev-dependencies", "build-dependencies")
_MANIFEST_SKIP = {"third-party", ".git", ".claude", "node_modules", "build",
                  "generated", "tmp"}


def _toml():
    try:
        import tomllib
        return tomllib
    except ModuleNotFoundError:  # python < 3.11
        try:
            import tomli as tomllib
            return tomllib
        except ModuleNotFoundError:
            return None


def _manifests(root):
    """Every tracked `Cargo.toml` under `root`, from the INDEX — a walk over
    `examples/` pays for every build tree beneath it (issue 0721: >300 s
    against 0.002 s for the same paths). Vendored trees are gitlinks, so they
    are not listed; `third-party/` is skipped for the one that is not."""
    r = subprocess.run(["git", "-C", root, "ls-files", "--", "Cargo.toml",
                        "*/Cargo.toml"], capture_output=True, text=True)
    if r.returncode == 0:
        for rel in r.stdout.split():
            if not rel.startswith("third-party/") and "/third-party/" not in rel:
                yield os.path.join(root, rel)
        return
    # walk-ok: the self-test's temp tree is not a repository, so there is no
    # index to consult and the tree is a handful of files it wrote itself.
    for dp, dn, fn in os.walk(root):
        dn[:] = sorted(d for d in dn
                       if d not in _MANIFEST_SKIP and not d.startswith("target"))
        if "Cargo.toml" in fn:
            yield os.path.join(dp, "Cargo.toml")


def _dep_specs(manifest):
    """[(crate name, spec dict-or-str)] over every dependency table."""
    out = []
    tables = [manifest.get(t, {}) for t in _DEP_TABLES]
    for tgt in manifest.get("target", {}).values():
        tables += [tgt.get(t, {}) for t in _DEP_TABLES]
    for table in tables:
        for key, spec in table.items():
            name = spec.get("package", key) if isinstance(spec, dict) else key
            out.append((name, spec))
    return out


def shipped_shape_subjects(root=None, subject_dir="packages"):
    """{crate: {"default": [...], "consumers": [manifest paths]}} for every
    crate under `<root>/<subject_dir>` with a non-empty default feature set that
    EVERY consumer (anywhere under root) takes with `default-features = false`.

    A self-dependency (`nros-c`'s dev-dep on `path = "."`) is not a consumer.
    A `workspace = true` entry inherits its spec from the nearest ancestor
    `[workspace.dependencies]`, which is where `default-features` would be.
    """
    root = root or ROOT
    toml = _toml()
    if toml is None:
        raise SystemExit("check-lane-contracts: no TOML parser (python >= 3.11 "
                         "or `tomli`) — the shipped-shape rule cannot run, "
                         "and a rule that cannot run must not report OK")
    loaded = {}
    for path in _manifests(root):
        try:
            with open(path, "rb") as fh:
                loaded[path] = toml.load(fh)
        except (OSError, ValueError):
            continue
    subject_root = os.path.join(root, subject_dir)
    crates = {}
    for path, m in loaded.items():
        name = m.get("package", {}).get("name")
        if name and path.startswith(subject_root + os.sep):
            crates[name] = {"default": m.get("features", {}).get("default", []),
                            "consumers": [], "path": path}

    def inherited(path, name):
        d = os.path.dirname(path)
        while True:
            parent = os.path.dirname(d)
            cand = os.path.join(parent, "Cargo.toml")
            spec = loaded.get(cand, {}).get("workspace", {}).get("dependencies", {})
            for key, val in spec.items():
                if (val.get("package", key) if isinstance(val, dict) else key) == name:
                    return val
            if parent == d or not parent.startswith(root):
                return None
            d = parent

    takes_defaults = {}
    for path, m in loaded.items():
        me = m.get("package", {}).get("name")
        for name, spec in _dep_specs(m):
            if name not in crates or name == me:
                continue
            if isinstance(spec, dict) and spec.get("workspace") is True:
                spec = inherited(path, name) or {}
            dff = isinstance(spec, dict) and spec.get("default-features") is False
            crates[name]["consumers"].append(path)
            takes_defaults.setdefault(name, False)
            takes_defaults[name] = takes_defaults[name] or not dff
    return {
        n: c for n, c in crates.items()
        if c["default"] and c["consumers"] and not takes_defaults.get(n, True)
    }


CARGO_LINE = re.compile(r"\bcargo\s+(?:\+\S+\s+)?(?:check|build|clippy)\b(.*)$")
_PKG = re.compile(r"(?:^|\s)(?:-p|--package)[\s=]+([A-Za-z0-9_-]+)")
_FEATS = re.compile(r"--features[\s=]+(?:\"([^\"]*)\"|'([^']*)'|(\S+))")


def _cargo_shapes(line):
    """[(package, {features}, no_default)] for one cargo check/build/clippy."""
    m = CARGO_LINE.search(line)
    if not m:
        return []
    args = m.group(1)
    pkgs = _PKG.findall(args)
    fm = _FEATS.search(args)
    raw = next((g for g in fm.groups() if g is not None), "") if fm else ""
    feats = {f for f in re.split(r"[,\s]+", raw) if f}
    return [(pkg, feats, "--no-default-features" in args) for pkg in pkgs]


def gating_lanes(recipes_map):
    """Every recipe a workflow job runs on a merge-gating event. Producer jobs
    INCLUDED — the artifact rule exempts those because they may resolve what
    they build; this rule asks a different question of them."""
    out = set()
    for _wf, _job, recs, _p in workflow_jobs():
        for recipe, events in lane_invocations(recs, recipes_map):
            if set(events) & GATING_EVENTS:
                out.add(recipe)
    return out


def shipped_shape_gaps(recipes, variables, gating, subjects):
    """([(crate, message)], {crate: [recipes that cover it]}). A gap is a
    subject NOT compiled in its shipped shape by any recipe reachable from a
    merge-gating lane."""
    reached = set()
    for lane in gating:
        reached |= closure(recipes, lane)
    gaps, covered = [], {}
    for crate in sorted(subjects):
        if crate in SHIPPED_EXEMPT:
            continue
        var = SHIPPED_SHAPES.get(crate)
        n = len(subjects[crate]["consumers"])
        if var is None:
            gaps.append((crate, (
                f"`{crate}` defaults to {subjects[crate]['default']}, and all {n} of "
                f"its consumers take it `default-features = false` — so its default "
                f"is not what ships, and nothing says what does. Name the shipped "
                f"feature list as a `just` variable and add `{crate}` to "
                f"SHIPPED_SHAPES in this script (or SHIPPED_EXEMPT, with a reason)."
            )))
            continue
        values = variables.get(var, set())
        if len(values) != 1:
            gaps.append((crate, (
                f"`{crate}`'s shipped shape is the justfile variable `{var}`, which "
                + ("is not defined as a string literal anywhere." if not values else
                   f"has {len(values)} spellings: {sorted(values)}. One spelling.")
            )))
            continue
        want = {f for f in next(iter(values)).split(",") if f}
        hits = []
        for r in sorted(reached):
            for line in _join_continuations(recipes.get(r, {}).get("body", [])):
                for pkg, feats, no_default in _cargo_shapes(line):
                    if pkg != crate or not no_default:
                        continue
                    if feats == {"{{" + var + "}}"} or feats == want:
                        hits.append(r)
        if hits:
            covered[crate] = sorted(set(hits))
        else:
            gaps.append((crate, (
                f"`{crate}` ships as `--no-default-features --features "
                f"\"{{{{{var}}}}}\"` (= {','.join(sorted(want))}) — all {n} consumers "
                f"take it `default-features = false` — but NO merge-gating lane "
                f"({'/'.join(sorted(GATING_EVENTS))}) compiles it in that shape. "
                f"Its default compiles a different crate, so a green gate says "
                f"nothing about the one that ships (issue 1163: a duplicate "
                f"`#[no_mangle]` in an `rmw-cffi` region reached main green). "
                f"Add `cargo check -p {crate} --no-default-features --features "
                f"\"{{{{{var}}}}}\"` to a recipe a gating job runs (`compile-smoke`)."
            )))
    return gaps, covered


def main():
    if "--selftest" in sys.argv:
        return selftest(verbose=True)
    # Always, not only behind the flag: a negative control nobody runs decays
    # into a comment.
    selftest()

    recipes = recipes_map = parse_justfile()
    errs, checked = [], 0

    for lane, promise in LANES.items():
        if lane not in recipes:
            errs.append(f"{lane}: no such recipe — this gate's lane list is stale")
            continue
        reached = closure(recipes, lane)
        lane_builders = lane_filters(recipes, reached)
        id_builder = builder_of_ids()
        # Does the lane PRODUCE compile-stage stamps anywhere in its closure?
        produces_stamps = any(
            "compile-check-fixtures.sh" in line
            for r in reached
            for line in recipes.get(r, {}).get("body", [])
        )
        for test, via in sorted(tests_invoked(recipes, reached).items()):
            used, found = resolvers_used(test)
            if not found:
                continue
            checked += 1
            bad = sorted(u for u in used if u in RUNTIME_RESOLVERS)
            if bad:
                errs.append(
                    f"{lane} reaches `{test}` (via `{via}`), which resolves a RUNTIME "
                    f"fixture: {', '.join(bad)}.\n"
                    f"      {lane} promises: {promise}.\n"
                    f"      A runtime fixture needs `build-test-fixtures` — an SDK, a\n"
                    f"      cross toolchain, sometimes QEMU. Either move the test to a\n"
                    f"      fixture-bearing lane, or make it a compile-stage check whose\n"
                    f"      gate produces its own stamp."
                )
                continue
            # The rule that would have caught the real defect. A compile-stage
            # resolver is ALLOWED, but only because the stamp is cheap enough
            # for the lane to produce — so the lane must actually produce it.
            # `platform_header_compile` used one legitimately while nothing in
            # the closure ran `compile-check-fixtures.sh`, so the lane silently
            # depended on `build-test-fixtures` having been run by someone,
            # somewhere. On a CI runner nobody had, and the required check was
            # red on `BuildFailed("Test fixture binary not prebuilt")`.
            if used and not produces_stamps:
                errs.append(
                    f"{lane} reaches `{test}` (via `{via}`), which resolves a "
                    f"COMPILE-STAGE stamp ({', '.join(sorted(used))}), but NOTHING in\n"
                    f"      the lane produces one — no `compile-check-fixtures.sh` in the\n"
                    f"      whole closure. The lane therefore depends on\n"
                    f"      `build-test-fixtures` having been run by someone else, which\n"
                    f"      is exactly what `{lane}` promises it does not need:\n"
                    f"      {promise}.\n"
                    f"      Have the gate build its own stamps (~13 s), as\n"
                    f"      `check-source-gates` does."
                )
                continue
            # PRESENCE IS NOT ENOUGH, and this is the half that was missing.
            # The rule above asks only "does the lane run the fixture builder at
            # all", which a lane passes even while filtering OUT the very
            # builder the test needs. `check-source-gates` requested
            # `NROS_COMPILE_CHECK_LANES=cargo-check` while all nine
            # `platform_hdr_*` rows are `cxx-syntax`, so the gate produced
            # stamps, produced the WRONG ones, and this check stayed green. It
            # failed only in CI, because locally the stamps already existed from
            # an earlier unfiltered build.
            if used and lane_builders is not None:
                want = {
                    id_builder[i] for i in stamp_ids_used(test)
                    if i in id_builder and id_builder[i]
                }
                missing = sorted(want - lane_builders)
                if missing:
                    errs.append(
                        f"{lane} reaches `{test}` (via `{via}`), whose stamps are built by\n"
                        f"      {', '.join(missing)} — but the lane filters to\n"
                        f"      NROS_COMPILE_CHECK_LANES={','.join(sorted(lane_builders))}.\n"
                        f"      The builder RUNS and produces the wrong rows, so the test\n"
                        f"      fails on a fresh checkout with 'Test fixture binary not\n"
                        f"      prebuilt' while passing anywhere the stamps happen to\n"
                        f"      survive from an earlier build.\n"
                        f"      Builders are READ from examples/fixtures.toml; do not\n"
                        f"      infer them from the fixture ids."
                    )

    # ---- phase-396 W5 — the same rule, for every tier a CI JOB invokes ----
    #
    # The loop above checks two AUTHORED tiers. That is not where this bit us:
    # phase-395 put `check-build` on the merge group, `check-build` reaches
    # `native::check` (generated message bindings) and `check-source-gates`
    # (`.compile-ok` stamps), the job produces neither, and the required check
    # was red for EVERY pull request for a day. Nothing here looked, because
    # `check-build` was not in LANES.
    #
    # So the lane list is now DERIVED: every `just <recipe>` any workflow job
    # runs is a lane, and the artifacts it may resolve are the ones that JOB
    # produces — not the ones some other job, or a developer's tree, happens to
    # have.
    #
    # A RATCHET, because the derived set legitimately contains known-bad states
    # today (the nightly arm of `pr-checks/check` still runs `check-build`
    # without a producer — same defect, on a lane issue 0878 has already
    # established nobody is watching). Recording them is the point: a new one
    # fails, and refreshing the baseline is a deliberate act.
    ci_findings = []

    # issue 1030 — a PROVISIONING step narrower than what consumes it.
    #
    # Separate loop, before the tier rules, because it differs from them twice
    # over and both differences are why the defect it catches went unseen for
    # four days:
    #
    #  * It is EVENT-LEVEL. The tier rules ask "does the job run the producer",
    #    and gate.yml's `check` job does — on `pull_request`/`merge_group`. On
    #    `schedule` that step is skipped while two steps needing it still run,
    #    so a presence test answers yes and the lane dies anyway.
    #  * It is not restricted to `GATING_EVENTS`. That restriction is a stated
    #    severity call for the tier rules and stands. It does not apply here: a
    #    step that cannot run AT ALL on its own event is broken on every event
    #    it claims, and the scheduled gate was red four consecutive nights
    #    proving it.
    #
    # It also runs on jobs that DO build artifacts — the `if producers` exemption
    # below is about a job being allowed to resolve what it builds, which says
    # nothing about whether it builds it on the right events.
    ordered = ordered_setup_requirements(recipes_map)
    for wf, job, recipes, _producers in workflow_jobs():
        ran_on = {}
        for name, events in recipes:
            if name.startswith("setup-"):
                ran_on.setdefault(name, set()).update(events)
        for recipe, events in sorted({(r, tuple(sorted(e))) for r, e in recipes}):
            for need in sorted(ordered.get(recipe, ())):
                gap = sorted(set(events) - ran_on.get(need, set()))
                if gap:
                    ci_findings.append(
                        f"{wf}:{job} runs `just {recipe}` on {','.join(gap)} "
                        f"without `just {need}`, which the justfile orders before "
                        f"it — widen that step's `if:` to cover these events"
                    )

    for wf, job, recipes, producers in workflow_jobs():
        if producers:
            continue  # the job builds artifacts; it may resolve them
        for recipe, events in sorted(lane_invocations(recipes, recipes_map)):
            # Every lane that produces a verdict, gating or not — see
            # SCANNED_EVENTS. `workflow_run` is excluded because the only lane
            # names on it are advice text in a posted comment.
            if not (set(events) & SCANNED_EVENTS):
                continue
            sev = "gating" if set(events) & GATING_EVENTS else "report"
            reached = closure(recipes_map, recipe)
            job_makes_stamps = any(
                "compile-check-fixtures.sh" in line
                for r in reached
                for line in recipes_map.get(r, {}).get("body", [])
            )
            for producer, via in sorted(required_producers(recipes_map, reached).items()):
                ci_findings.append(f"[{sev}] {wf}:{job} runs `{recipe}` -> `{via}` "
                                   f"hard-requires `just {producer}`, which the job never runs")
            for test, via in sorted(tests_invoked(recipes_map, reached).items()):
                used, found = resolvers_used(test)
                if not found or not used:
                    continue
                runtime = sorted(u for u in used if u in RUNTIME_RESOLVERS)
                if runtime:
                    ci_findings.append(f"[{sev}] {wf}:{job} runs `{recipe}` -> `{test}` "
                                       f"needs RUNTIME fixture ({','.join(runtime)})")
                elif not job_makes_stamps:
                    ci_findings.append(f"[{sev}] {wf}:{job} runs `{recipe}` -> `{test}` "
                                       f"needs a COMPILE stamp nothing in the job builds")

    base_path = os.path.join(ROOT, ".config", "lane-contract-baseline.json")
    if "--update" in sys.argv:
        os.makedirs(os.path.dirname(base_path), exist_ok=True)
        import json as _json
        with open(base_path, "w", encoding="utf8") as fh:
            _json.dump({
                "_comment": (
                    "CI jobs that run a tier needing an artifact the job does not "
                    "build (phase-396 W5). Each line is a required check that "
                    "cannot pass on a clean runner. Refresh with --update and say "
                    "why in the commit."
                ),
                "findings": sorted(set(ci_findings)),
            }, fh, indent=2)
            fh.write("\n")
        print(f"check-lane-contracts: baseline written — {len(set(ci_findings))} known.")
        return 0

    known = set()
    if os.path.exists(base_path):
        import json as _json
        with open(base_path, encoding="utf8") as fh:
            known = set(_json.load(fh)["findings"])
    for f in sorted(set(ci_findings) - known):
        errs.append(
            f"{f}.\n"
            f"      A CI job may resolve an artifact only if that JOB builds it —\n"
            f"      nothing carries a build dir between jobs, and a developer tree\n"
            f"      where `build-test-fixtures` has run is not the runner. This is\n"
            f"      the shape that froze the merge queue (phase-396): a required\n"
            f"      check red for every input looks exactly like a broken PR.\n"
            f"      Add the producer to the job, or take the tier off that event.\n"
            f"      If it is intended, record it:\n"
            f"        python3 scripts/check-lane-contracts.py --update"
        )

    # ---- issue 1163 — shipped shapes. A HARD rule, not a ratchet: the subject
    # set is three crates and every one is either covered or exempt today.
    subjects = shipped_shape_subjects()
    shape_gaps, shape_covered = shipped_shape_gaps(
        recipes_map, just_variables(), gating_lanes(recipes_map), subjects)
    for _crate, msg in shape_gaps:
        errs.append(msg)

    if errs:
        print(f"check-lane-contracts: {len(errs)} tier violation(s):\n", file=sys.stderr)
        for e in errs:
            print(f"  - {e}", file=sys.stderr)
        print(
            "\n  A tier claim nobody can check is a promise about COST that quietly\n"
            "  stops being true. This one was false long enough to make a REQUIRED\n"
            "  CI check permanently red.",
            file=sys.stderr,
        )
        return 1

    # Count what the rule RESOLVED, not how many words it saw. The old count
    # was over the raw word stream, so it reported 51 "lane invocations" for 15
    # lanes — a number that grows when a workflow adds a `setup` step and says
    # nothing about coverage, which is the shape this gate's own history warns
    # about (a green summary beside an uncovered lane).
    def _count(scope):
        return sum(
            1 for _w, _j, r, p in workflow_jobs() if not p
            for _rec, ev in lane_invocations(r, recipes_map)
            if set(ev) & scope
        )
    gating = _count(GATING_EVENTS)
    scanned = _count(SCANNED_EVENTS)
    # Say what the producer rule EXAMINED, not just that it passed. A rule that
    # covers nothing prints the same "OK" as one that covers everything, and
    # this gate's own history is a green summary next to an uncovered lane.
    ordered_now = ordered_setup_requirements(recipes_map)
    covered = sum(
        1 for _w, _j, r, _p in workflow_jobs()
        for rec, _ev in {(a, tuple(sorted(b))) for a, b in r}
        if rec in ordered_now
    )
    print(
        f"check-lane-contracts OK — {checked} test target(s) across "
        f"{len(LANES)} affordability tier(s) and {scanned} CI lane invocation(s) "
        f"({gating} merge-gating, {scanned - gating} report-only); none resolves "
        f"an artifact its job does not build. "
        f"{covered} step invocation(s) on any event carry a justfile-ordered "
        f"`setup-*` precondition; each runs where its producer does. "
        f"{len(shape_covered)} crate(s) whose default is not what ships "
        f"({shape_desc(shape_covered)}) are compiled in their shipped shape "
        f"by a merge-gating lane; "
        f"{len([c for c in subjects if c in SHIPPED_EXEMPT])} exempt with a reason."
    )
    return 0


def shape_desc(covered):
    return ", ".join(f"{c} via {'/'.join(v)}" for c, v in sorted(covered.items()))


def selftest(verbose=False):
    """Prove it can fail. Runs on every invocation."""
    import tempfile

    real = (JUSTFILE, TESTS_DIR)
    ok = fail = 0

    def chk(desc, cond):
        nonlocal ok, fail
        if verbose or not cond:
            print(f"  {'ok   ' if cond else 'FAIL '} {desc}")
        if cond:
            ok += 1
        else:
            fail += 1

    with tempfile.TemporaryDirectory() as d:
        jf = os.path.join(d, "justfile")
        td = os.path.join(d, "tests")
        os.makedirs(td)
        globals()["JUSTFILE"], globals()["TESTS_DIR"] = jf, td

        with open(jf, "w", encoding="utf8") as fh:
            fh.write("ci-l1: gate-a\n\ngate-a:\n    cargo test -p x --test t_runtime\n")
        with open(os.path.join(td, "t_runtime.rs"), "w", encoding="utf8") as fh:
            fh.write('fn f() { require_cmake_fixture("a", "b"); }\n')
        r = parse_justfile()
        chk("a dependency is followed into the closure", "gate-a" in closure(r, "ci-l1"))
        chk("a RUNTIME resolver in a reached test is detected",
            "require_cmake_fixture" in resolvers_used("t_runtime")[0])

        with open(os.path.join(td, "t_compile.rs"), "w", encoding="utf8") as fh:
            fh.write('fn f() { require_compile_check("a"); }\n')
        used = resolvers_used("t_compile")[0]
        chk("a COMPILE-stage resolver is NOT a violation",
            bool(used) and not any(u in RUNTIME_RESOLVERS for u in used))

        with open(jf, "w", encoding="utf8") as fh:
            fh.write("ci-l1: \\\n    gate-a \\\n    gate-b\n\n"
                     "gate-b:\n    cargo test --test t_runtime\n")
        r = parse_justfile()
        chk("a backslash-continued dependency list is parsed",
            "gate-b" in closure(r, "ci-l1"))

        chk("a test file that does not exist is not a violation",
            resolvers_used("no_such_test") == (set(), False))

        # The shape `ci-l1` actually has: no header dependencies, gates invoked
        # from the BODY. Missing this made the gate report OK over an empty set.
        with open(jf, "w", encoding="utf8") as fh:
            fh.write("ci-l1:\n    @just gate-a check-other\n\n"
                     "gate-a:\n    cargo test --test t_runtime\n")
        r = parse_justfile()
        chk("a `just a b` call in a BODY is an edge, not just header deps",
            "gate-a" in closure(r, "ci-l1"))
        chk("...and the reached test is then actually inspected",
            "t_runtime" in tests_invoked(r, closure(r, "ci-l1")))

        # Issue 0922 — the same invocation, WRAPPED. This is the shape
        # `test-lane-contracts` has, and the per-line scan could not see it:
        # the verb and its `--test` targets are on different lines.
        with open(jf, "w", encoding="utf8") as fh:
            fh.write("ci-l1:\n    @just gate-a\n\n"
                     "gate-a:\n    cargo nextest run \\\n"
                     "        --test t_wrapped --test t_second\n")
        r = parse_justfile()
        found = tests_invoked(r, closure(r, "ci-l1"))
        chk("a backslash-continued cargo invocation is still inspected",
            "t_wrapped" in found and "t_second" in found)

        # A module lane reached through a flat forwarder: `@just ci l1`.
        with open(jf, "w", encoding="utf8") as fh:
            fh.write("mod ci 'just/ci.just'\n\nci-l1:\n    @just ci l1\n")
        os.makedirs(os.path.join(os.path.dirname(jf), "just"), exist_ok=True)
        with open(os.path.join(os.path.dirname(jf), "just", "ci.just"), "w",
                  encoding="utf8") as fh:
            fh.write("l1:\n    cargo nextest run --test t_module\n")
        r = parse_justfile()
        chk("a `just <mod> <recipe>` call is ONE edge to <mod>::<recipe>",
            "t_module" in tests_invoked(r, closure(r, "ci-l1")))

    globals()["JUSTFILE"], globals()["TESTS_DIR"] = real
    # ---- phase-396 W5: the pieces that were BLIND, each with a case ----

    # The recipe header regex skipped `check JOBS="75%":` — uppercase parameter,
    # quoted default with a `%`. That one miss hid `native::check`, which is the
    # recipe whose unmet precondition froze the queue.
    mod = _parse_one(['check JOBS="75%":', '    echo hi', '', 'other:', '    x'], "native")
    chk("a recipe with an UPPERCASE quoted-default parameter is parsed",
        "native::check" in mod)
    chk("module recipes are keyed <module>::<recipe>", "native::other" in mod)
    bare = _parse_one(["a: b", "    x", "b:", "    y"], "native")
    chk("a bare dep inside a module resolves to that module",
        bare["native::a"]["deps"] == ["native::b"])

    # The producer rule: a hard-failing remediation IS the declaration.
    rp = {"r": {"deps": [], "body": ["    echo \"Run 'just generate-bindings' first\"",
                                     "    exit 1"]}}
    chk("a hard-failing 'run just <producer> first' is a declared precondition",
        required_producers(rp, ["r"]) == {"generate-bindings": "r"})
    advisory = {"r": {"deps": [], "body": ["    echo 'hint: just generate-bindings'"]}}
    chk("an ADVISORY mention with no failure path is not a precondition",
        required_producers(advisory, ["r"]) == {})

    # issue 1030 — the justfile's dependency ORDER as a second declaration
    # source, and the folded `if:` that hid the step it applies to.
    chk("a setup-* ordered before a recipe is that recipe's precondition",
        ordered_setup_requirements(
            {"_codegen": {"deps": ["setup-launch-resolve", "generate-bindings"],
                          "body": []}}
        ) == {"generate-bindings": {"setup-launch-resolve"}})
    chk("a setup-* ordered AFTER a recipe is not its precondition",
        ordered_setup_requirements(
            {"w": {"deps": ["generate-bindings", "setup-launch-resolve"], "body": []}}
        ) == {})
    chk("a wrapper with no setup-* declares nothing",
        ordered_setup_requirements({"w": {"deps": ["a", "b"], "body": []}}) == {})
    # The scanner bug that made the rule unable to fire: a folded guard's events
    # live on the CONTINUATION lines, and reading only the `if:` line credited
    # the step with every event the workflow has.
    folded = ("      - name: p\n"
              "        if: >-\n"
              "          ${{ contains(fromJSON('[\"pull_request\"]'), github.event_name)\n"
              "              && !cancelled() }}\n"
              "        run: just setup-launch-resolve\n")
    chk("a folded `if:` keeps the events on its continuation lines",
        _events_of(folded.split("if:")[1].split("run:")[0],
                   {"pull_request", "schedule"}) == {"pull_request"})

    # Event attribution decides required-vs-nightly, which is the whole severity
    # split. A step's `if:` wins over the workflow's event list.
    allev = {"pull_request", "merge_group", "push", "schedule"}
    chk("no `if:` means every event the workflow declares",
        _events_of("", allev) == allev)
    chk("a fromJSON list narrows to exactly those events",
        _events_of("""if: ${{ contains(fromJSON('["schedule","workflow_dispatch"]'), github.event_name) }}""",
                   allev) == {"schedule"})
    chk("a merge_group step is still gating",
        bool(_events_of("""if: ${{ contains(fromJSON('["merge_group"]'), github.event_name) }}""",
                        allev) & GATING_EVENTS))
    chk("a schedule-only step is NOT gating",
        not (_events_of("""if: ${{ contains(fromJSON('["schedule"]'), github.event_name) }}""",
                        allev) & GATING_EVENTS))
    # ...and is still SCANNED. Issue 1030 named the narrow scope as the reason
    # the tier rules could not see the scheduled gate; keeping only the first
    # assertion is what let that stand, so both directions are pinned here.
    chk("a schedule-only step IS in scope for the tier rules",
        bool(_events_of("""if: ${{ contains(fromJSON('["schedule"]'), github.event_name) }}""",
                        allev) & SCANNED_EVENTS))
    chk("`workflow_run` is NOT in scope — queue-notify's lane names are advice text",
        not (_events_of("", {"workflow_run"}) & SCANNED_EVENTS))

    # Lane resolution. `just ci tier1` is one lane spelled in two words, and
    # `just check api-parity` collides with a DIFFERENT root recipe of the same
    # name — the two failures the bare-word test had.
    ev = ("pull_request",)
    words = [("ci", ev), ("tier1", ev)]
    chk("a `just <mod> <recipe>` pair resolves to <mod>::<recipe>",
        lane_invocations(words, {"ci::tier1": {}}) == {("ci::tier1", ev)})
    chk("the MODULE spelling wins over a root recipe of the same name",
        lane_invocations([("check", ev), ("api-parity", ev)],
                         {"api-parity": {}, "check::api-parity": {}})
        == {("check::api-parity", ev)})
    chk("a bare word that IS a recipe still resolves",
        lane_invocations([("test-unit", ev)], {"test-unit": {}}) == {("test-unit", ev)})
    chk("a word that names no recipe resolves to nothing",
        lane_invocations([("setup", ev)], {"test-unit": {}}) == set())
    chk("words with DIFFERENT event sets are never paired",
        lane_invocations([("ci", ("push",)), ("tier1", ev)], {"ci::tier1": {}}) == set())
    chk("a LONE `!=` guard subtracts rather than selects",
        _events_of("if: ${{ github.event_name != 'pull_request' }}", allev)
        == allev - {"pull_request"})
    # The guard this got wrong. `pr-checks`'s `check` job runs on EVERY event
    # and merely narrows the pull-request case to code-touching diffs; the old
    # rule read "both != and == are present" and concluded pull_request ONLY.
    chk("an exclusion OR-ed with a non-event condition does NOT exclude",
        _events_of("if: ${{ always() && (github.event_name != 'pull_request'"
                   " || needs.changes.outputs.code == 'true') }}", allev) == allev)
    chk("a quoted event that is not compared to github.event_name is ignored",
        _events_of("if: ${{ needs.x.outputs.name == 'merge_group' }}", allev) == allev)
    chk("`github.event_name == 'x'` selects exactly x",
        _events_of("if: ${{ github.event_name == 'pull_request' }}", allev)
        == {"pull_request"})

    # ---- issue 1163 — the shipped-shape rule, end to end on a temp tree ----
    real_wf = WORKFLOW_DIR
    with tempfile.TemporaryDirectory() as d:
        jf = os.path.join(d, "justfile")
        wf = os.path.join(d, "workflows")
        os.makedirs(os.path.join(d, "just", "check"))
        os.makedirs(wf)
        globals()["JUSTFILE"], globals()["WORKFLOW_DIR"] = jf, wf
        with open(jf, "w", encoding="utf8") as fh:
            fh.write("mod check 'just/check.just'\n")
        with open(os.path.join(d, "just", "check.just"), "w", encoding="utf8") as fh:
            fh.write('SHIPPED := "a,b"\n\nimport \'check/lanes.just\'\n\nfast:\n    echo\n')
        smoke = ("compile-smoke:\n    cargo check --workspace\n"
                 "    cargo check -p nros-c --no-default-features "
                 "--features \"{{SHIPPED}}\" --quiet\n")
        lanes = os.path.join(d, "just", "check", "lanes.just")

        def write_lanes(text):
            with open(lanes, "w", encoding="utf8") as fh:
                fh.write(text)
            return parse_justfile()

        with open(os.path.join(wf, "gate.yml"), "w", encoding="utf8") as fh:
            fh.write("on:\n  pull_request:\n  schedule:\njobs:\n  check:\n    steps:\n"
                     "      - run: just check fast\n"
                     "      - if: ${{ github.event_name == 'pull_request' }}\n"
                     "        run: just check compile-smoke\n")
        subj = {"nros-c": {"default": ["panic-platform"], "consumers": ["x", "y"]}}
        saved = dict(SHIPPED_SHAPES)
        SHIPPED_SHAPES.clear()
        SHIPPED_SHAPES["nros-c"] = "SHIPPED"
        r = write_lanes(smoke)
        chk("a recipe in an `import`ed file is keyed to the importing module",
            "check::compile-smoke" in r)
        chk("a string-literal `NAME := \"...\"` in a module is read",
            just_variables().get("SHIPPED") == {"a,b"})
        gating = gating_lanes(r)
        chk("a `just check <gate>` on pull_request is a gating lane",
            "check::compile-smoke" in gating)
        gaps, cov = shipped_shape_gaps(r, just_variables(), gating, subj)
        chk("a gating lane compiling the crate as `{{VAR}}` covers it",
            gaps == [] and cov == {"nros-c": ["check::compile-smoke"]})
        # THE negative control: the line `compile-smoke` gained for 1163,
        # removed. This is the state main was in when PR #329 landed.
        r = write_lanes("compile-smoke:\n    cargo check --workspace\n")
        gaps, _ = shipped_shape_gaps(r, just_variables(), gating_lanes(r), subj)
        chk("REMOVING the shipped-shape line from the gating lane is a violation",
            [c for c, _m in gaps] == ["nros-c"])
        chk("...and the message names the variable and the fix",
            "SHIPPED" in gaps[0][1] and "compile-smoke" in gaps[0][1])
        # A literal spelling equal to the variable is accepted (order-free)...
        r = write_lanes("compile-smoke:\n    cargo check -p nros-c "
                        "--no-default-features --features \"b,a\"\n")
        gaps, _ = shipped_shape_gaps(r, just_variables(), gating_lanes(r), subj)
        chk("a literal spelling equal to the variable's value covers it", gaps == [])
        # ...a DIFFERENT literal is drift, which is the reason the variable exists.
        r = write_lanes("compile-smoke:\n    cargo check -p nros-c "
                        "--no-default-features --features \"a\"\n")
        gaps, _ = shipped_shape_gaps(r, just_variables(), gating_lanes(r), subj)
        chk("a literal that DIFFERS from the variable does not cover it",
            [c for c, _m in gaps] == ["nros-c"])
        # The right line on a lane only the nightly runs is the exact shape
        # `check-c` has today — and it is not coverage.
        r = write_lanes(smoke.replace("compile-smoke:", "c:"))
        gaps, _ = shipped_shape_gaps(r, just_variables(), gating_lanes(r), subj)
        chk("the shipped shape on a lane NO gating job runs is a violation",
            [c for c, _m in gaps] == ["nros-c"])
        r = write_lanes(smoke.replace("--no-default-features ", ""))
        gaps, _ = shipped_shape_gaps(r, just_variables(), gating_lanes(r), subj)
        chk("the feature list WITHOUT --no-default-features is not the shipped shape",
            [c for c, _m in gaps] == ["nros-c"])
        r = write_lanes(smoke)
        gaps, _ = shipped_shape_gaps(r, just_variables(), gating_lanes(r),
                                     {"other": subj["nros-c"]})
        chk("a subject with NO declared shipped shape is a violation, not a skip",
            [c for c, _m in gaps] == ["other"] and "SHIPPED_SHAPES" in gaps[0][1])
        r = write_lanes(smoke + '\nSHIPPED := "a,c"\n')
        gaps, _ = shipped_shape_gaps(r, just_variables(), gating_lanes(r), subj)
        chk("TWO spellings of the shipped variable is a violation",
            [c for c, _m in gaps] == ["nros-c"] and "2 spellings" in gaps[0][1])
        SHIPPED_SHAPES.clear()
        SHIPPED_SHAPES.update(saved)

        # Subject derivation on a synthetic cargo tree.
        pk = os.path.join(d, "packages")
        for name, body in (
            ("lib", 'name = "lib"\n[features]\ndefault = ["p"]\np = []\n'
                    '[dev-dependencies]\nlib = { path = "." }\n'),
            ("lib2", 'name = "lib2"\n[features]\ndefault = ["p"]\np = []\n'),
            ("nodef", 'name = "nodef"\n'),
            ("a", 'name = "a"\n[dependencies]\nlib = { path = "../lib", '
                  'default-features = false }\nnodef = { path = "../nodef", '
                  'default-features = false }\n'
                  'lib2 = { path = "../lib2", default-features = false }\n'),
        ):
            os.makedirs(os.path.join(pk, name))
            with open(os.path.join(pk, name, "Cargo.toml"), "w", encoding="utf8") as fh:
                fh.write("[package]\n" + body)
        os.makedirs(os.path.join(d, "examples", "b"))
        with open(os.path.join(d, "examples", "b", "Cargo.toml"), "w",
                  encoding="utf8") as fh:
            fh.write('[package]\nname = "b"\n[dependencies]\nlib2 = { path = "x" }\n')
        subs = shipped_shape_subjects(d)
        chk("a crate every consumer takes default-features=false is a subject",
            "lib" in subs and subs["lib"]["consumers"] == [
                os.path.join(pk, "a", "Cargo.toml")])
        chk("a self dev-dependency is not a consumer",
            len(subs["lib"]["consumers"]) == 1)
        chk("a crate with NO default features is not a subject", "nodef" not in subs)
        chk("one consumer taking the defaults (even outside packages/) disqualifies",
            "lib2" not in subs)
    globals()["JUSTFILE"], globals()["WORKFLOW_DIR"] = real[0], real_wf

    if verbose:
        print(f"\n{ok} passed, {fail} failed")
    if fail:
        print("check-lane-contracts self-test: FAILED", file=sys.stderr)
        raise SystemExit(1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
