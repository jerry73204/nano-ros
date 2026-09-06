#!/usr/bin/env python3
"""Every tier precondition must have a provisioner, and it must be reachable.

`check-tier-preconditions.sh` asserts host facts before a tier runs. `just setup
<scope>` is what a user runs to satisfy them. Nothing kept the two in agreement,
and the cost was paid one CI round-trip at a time:

  - `just ci tier1` in host-tests failed on three unmet preconditions at once
    (cross Rust targets, pinned corrosion, fixtures). Two were provisioning gaps
    that `just setup` simply did not cover, for ANY scope.
  - Fixing those let the lane run 40 minutes further and fail on `c-fmt` --
    clang-format not provisioned either. Same class, one layer deeper, and only
    visible once the layer above was fixed.

Each layer costs a ~40-minute CI run to discover. This gate finds the next one
without spending it.

WHAT IT CHECKS

  1. Every `probe` in check-tier-preconditions.sh is CLASSIFIED here.
  2. Every classification names a probe that still exists (the reverse
     direction -- an authored map that is only checked one way drifts, which is
     what the RMW parity map did while reading green).
  3. Every probe classified `setup` has its remedy reachable from
     `just setup <scope>`: the command appears in `_setup-common` or in a
     module's `setup` recipe.
  4. Every `just setup-*` recipe named in a gate's failure text is either
     reachable the same way or classified here. This is the clang-format case:
     it was never a *precondition*, it was a GATE that needed a tool, and no
     precondition check would ever have caught it.

The three classes are not interchangeable:

  setup   `just setup <scope>` must provision it. Checked for reachability.
  build   the BUILD verb produces it (fixtures, `nros sync` output). Provisioning
          must NOT do it -- compiling at setup time is a different contract.
  manual  a human decides (a submodule pointer move, deleting stray build
          output). Automating these would be wrong, not merely unimplemented.

Run:  python3 scripts/check-preconditions-provisioned.py [--self-test]
"""

import os
import re
import subprocess
import sys
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent / "lib"))
# The gates live across `just/check/*.just` now; the index alone is a
# SMALLER closure than `just` sees, and this gate fails quietly on it.
from check_just_sources import check_just_text

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PRECONDITIONS = os.path.join(ROOT, "scripts", "check-tier-preconditions.sh")
JUSTFILE = os.path.join(ROOT, "justfile")
JUST_DIR = os.path.join(ROOT, "just")

# Keyed by a stable substring of the probe LABEL. Value: (class, reason).
#
# Adding a probe to check-tier-preconditions.sh without a row here fails this
# gate -- deliberately. The question "who provisions this?" has to be answered
# when the precondition is written, not when CI discovers the gap.
CLASSIFICATION = {
    "cross Rust target": (
        "setup",
        "`just workspace rust-targets` reads config/rust-targets.txt, the SSoT.",
    ),
    "a submodule is not at the commit": (
        "manual",
        "Moving a submodule pointer is a human decision (CLAUDE.md): a rewind "
        "unships whatever the skipped commits fixed, and setup must never "
        "silently pick a side.",
    ),
    "in-tree nros CLI is stale": (
        "setup",
        "`just setup-cli`, run by `_setup-common` on every setup path.",
    ),
    "leaf .cargo/config.toml includes": (
        "build",
        "`nros sync` writes the generated targets; it is part of the build "
        "stage, and the fixture builds run it.",
    ),
    "test fixtures not ready": (
        "build",
        "`just build-test-fixtures` IS the build verb. Provisioning must not "
        "compile -- that is the `no compilation inside tests` contract one "
        "stage earlier.",
    ),
    "build output beside workspace source": (
        "manual",
        "Stray build output is deleted by a human who can see what it is; "
        "`rm -rf` from a setup recipe is how you lose work.",
    ),
    "corrosion in the SDK store": (
        "setup",
        "`just workspace install-corrosion`, run by `_setup-common`.",
    ),
}

# `just setup-*` recipes a gate tells the user to run. Same rule: reachable from
# `just setup`, or classified with a reason.
TOOL_RECIPES = {
    "setup-cli": ("setup", "in `_setup-common`."),
    "setup-clang-format": (
        "setup",
        "`check fast` runs c-fmt/cpp-fmt and every tier runs `check fast`.",
    ),
    "setup-nuttx": (
        "manual",
        "Platform-specific: reached by `just setup nuttx`, not by every scope. "
        "The fast line does not run a gate that needs it.",
    ),
    "setup-threadx": (
        "manual",
        "Platform-specific, same as nuttx: `just setup threadx_linux` provisions "
        "it for the scope that needs it, and no fast-line gate does.",
    ),
    "setup-launch-resolve": (
        "setup",
        "in `_setup-common`: `nros sync` shells out to the resolver by absolute "
        "path beside the CLI (issue 0285), so every scope needs it.",
    ),
    # `setup-mdbook` was here and is GONE: phase-422 W2 moved mdBook into the
    # index as `[tool.mdbook]`, so no gate names the recipe any more and the row
    # went stale. This gate's both-directions check caught that itself — which
    # is the point of checking an authored map in both directions rather than
    # only asking "is every mention classified".
    "setup-hooks": (
        "manual",
        "Writes to the user's git config (hooks path, diff.submodule, "
        "push.recurseSubmodules). Setup must not reconfigure someone's git "
        "behind their back -- it is opt-in by design.",
    ),
}


def read(path):
    with open(path, encoding="utf8") as fh:
        return fh.read()


def probes(text):
    """[(label, remedy)] for each `probe "label" \\ "remedy"` call."""
    out = []
    for m in re.finditer(r'^probe\s+"([^"]+)"\s*\\\s*\n\s*"([^"]+)"', text, re.M):
        out.append((m.group(1), m.group(2)))
    return out


def setup_closure(justfile_text, _just_files):
    """Text of `_setup-common` -- what EVERY `just setup <scope>` runs.

    Deliberately NOT the union with each module's `setup` recipe. A precondition
    that `check-tier-preconditions` asserts for every tier must be provisioned on
    every setup path, and only `_setup-common` is on every path.

    Getting this wrong made the gate useless in exactly the case it exists for:
    with module recipes folded in, `just/workspace.just`'s own `setup` contains
    `just workspace install-corrosion`, so deleting that line from
    `_setup-common` still "passed" -- while `just setup native` (which never
    chains `just workspace setup`) left corrosion unprovisioned. That is the
    original bug, reading green. Caught by mutation-testing this gate.
    """
    m = re.search(r"^_setup-common:\n(.*?)(?=\n^[a-zA-Z_@\[])", justfile_text, re.M | re.S)
    return m.group(1) if m else ""


def remedy_commands(remedy):
    """Candidate commands from a remedy string; ANY one being reachable is enough.

    Remedies legitimately offer alternatives ("nros setup --tool corrosion (or:
    just workspace install-corrosion)") and either spelling provisions the thing.
    """
    cands = re.findall(r"just [a-z0-9 _-]+|nros setup --tool [a-z0-9-]+", remedy)
    return [c.strip() for c in cands]


_TOOL_CACHE = {}


def gate_tool_recipes(root):
    """`just setup-X` mentioned anywhere a gate reports failure."""
    if root in _TOOL_CACHE:
        return _TOOL_CACHE[root]
    found = set()
    # `git ls-files`, not a filesystem walk: `check-no-tracked-file-find` forbids
    # walking to locate tracked files, and it is right — a walk also descends
    # into build output and untracked scratch, so it answers a different
    # question than "what does this repo ship".
    listing = subprocess.run(
        ["git", "-C", root, "ls-files", "scripts/*.sh", "scripts/*.py", "scripts/**/*.sh", "scripts/**/*.py"],
        capture_output=True,
        text=True,
    ).stdout.split()
    for rel in listing:
        try:
            # Some checked-in scripts carry non-UTF8 bytes (fixture payloads,
            # encoding probes). Read tolerantly: we are grepping for an ASCII
            # recipe name, so a lossy decode cannot hide one.
            with open(os.path.join(root, rel), encoding="utf8", errors="replace") as fh:
                t = fh.read()
        except OSError:
            continue
        found.update(re.findall(r"just (setup-[a-z-]+)", t))
    found.update(re.findall(r"just (setup-[a-z-]+)", check_just_text(root)))
    _TOOL_CACHE[root] = found
    return found


def self_test():
    ps = probes(
        'probe "a thing is missing" \\\n    "just workspace do-it   (bypass: X=1)" \\\n    bash x.sh\n'
    )
    assert ps == [("a thing is missing", "just workspace do-it   (bypass: X=1)")], ps
    assert remedy_commands("nros setup --tool corrosion   (or: just workspace install-corrosion)") == [
        "nros setup --tool corrosion",
        "just workspace install-corrosion",
    ]
    # A remedy naming no command must not silently pass as "reachable".
    assert remedy_commands("rm -rf the named dir(s)") == []
    sys.stdout.write("check-preconditions-provisioned self-test: OK\n")


def main():
    if "--self-test" in sys.argv:
        self_test()
        return 0
    self_test()

    text = read(PRECONDITIONS)
    found = probes(text)
    if not found:
        sys.stderr.write(
            "error: parsed ZERO probes from %s.\n"
            "The `probe \"label\" \\\\ \"remedy\"` shape changed; this gate would\n"
            "pass vacuously, which is worse than failing.\n" % PRECONDITIONS
        )
        return 1

    just_files = [
        os.path.join(JUST_DIR, f) for f in sorted(os.listdir(JUST_DIR)) if f.endswith(".just")
    ]
    closure = setup_closure(read(JUSTFILE), just_files)

    errors = []
    matched_keys = set()

    for label, remedy in found:
        key = next((k for k in CLASSIFICATION if k in label), None)
        if key is None:
            errors.append(
                "precondition NOT CLASSIFIED: %r\n"
                "    remedy: %s\n"
                "    Add a row to CLASSIFICATION in %s saying who provisions it:\n"
                "      setup  -> `just setup <scope>` must do it (checked for reachability)\n"
                "      build  -> the build verb produces it\n"
                "      manual -> a human decides, and say why automating is wrong"
                % (label, remedy, os.path.relpath(__file__, ROOT))
            )
            continue
        matched_keys.add(key)
        kind, _reason = CLASSIFICATION[key]
        if kind != "setup":
            continue
        cands = remedy_commands(remedy)
        if not cands:
            errors.append(
                "precondition %r is classified `setup` but its remedy names no\n"
                "    command this gate can look for: %s" % (label, remedy)
            )
            continue
        if not any(c in closure for c in cands):
            errors.append(
                "precondition %r is classified `setup` but NO setup recipe runs it.\n"
                "    remedy offers: %s\n"
                "    Nothing in `_setup-common` or a module `setup` matches, so\n"
                "    `just setup <scope>` leaves this precondition unmet and the\n"
                "    tier fails on a host that followed the documented chain."
                % (label, ", ".join(cands))
            )

    for key in CLASSIFICATION:
        if key not in matched_keys:
            errors.append(
                "STALE classification %r matches no probe in %s.\n"
                "    A map checked in only one direction drifts; delete the row or\n"
                "    fix the key." % (key, os.path.relpath(PRECONDITIONS, ROOT))
            )

    for recipe in sorted(gate_tool_recipes(ROOT)):
        if recipe not in TOOL_RECIPES:
            errors.append(
                "gate tool recipe `just %s` is UNCLASSIFIED.\n"
                "    A gate tells users to run it, so either `just setup` provisions\n"
                "    it or it is deliberately manual. Add a row to TOOL_RECIPES."
                % recipe
            )
            continue
        kind, _reason = TOOL_RECIPES[recipe]
        if kind == "setup" and recipe not in closure:
            errors.append(
                "gate tool recipe `just %s` is classified `setup` but no setup\n"
                "    recipe runs it. A gate will fail on a host that ran\n"
                "    `just setup <scope>` exactly as documented." % recipe
            )

    for recipe in TOOL_RECIPES:
        if recipe not in gate_tool_recipes(ROOT):
            errors.append(
                "STALE tool row `%s`: no gate mentions it any more." % recipe
            )

    if errors:
        sys.stderr.write("check-preconditions-provisioned: %d problem(s)\n\n" % len(errors))
        for e in errors:
            sys.stderr.write("  - %s\n\n" % e)
        return 1

    kinds = {}
    for k, (kind, _r) in CLASSIFICATION.items():
        kinds[kind] = kinds.get(kind, 0) + 1
    sys.stdout.write(
        "check-preconditions-provisioned: OK — %d precondition(s) "
        "(%s), %d gate tool recipe(s).\n"
        % (
            len(found),
            ", ".join("%d %s" % (v, k) for k, v in sorted(kinds.items())),
            len(TOOL_RECIPES),
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
