#!/usr/bin/env python3
"""Every `just <recipe>` printed in a document must name a recipe that exists.

Issue 0936.

`check-just-recipe-refs` (issue 0660) verifies recipe references inside
justfiles and workflows. Its docstring hands documents to someone else:

    * `check-doc-refs` covers documents, not recipe references.

`check-doc-refs` validates links and paths, not recipe names, so nothing covered
the gap — and a document is the one place a `just` invocation can rot with no
gate watching, while being exactly where a human copies an invocation from.
Measured when this was written: 116 references across 453 documents naming
recipes that do not exist, including `just build-zenohd`, the retired recipe
whose twelve dead callers are why 0660 exists at all.

## Why this is a separate gate

Same rule, different POLICY. The justfile/workflow scan is zero-tolerance: a
dead reference there is a recipe that fails on invocation, and there are none
left. The document scan carries a RATCHET, because the 116 are real debt that
predates the gate and fixing them is a research task per line — each needs the
recipe it should have named, and guessing would replace a dead command with a
wrong one. Mixing a zero-tolerance policy and a ratchet in one gate makes
neither legible.

## Reading a document is not reading a script

A gate here is only worth having if it can tell a command from a sentence about
a command. Every one of these was a live false positive in the first pass:

* **Prose.** `just` is an English word. A naive scan flagged `just the ...`
  three times.
* **Flag arguments.** `just --group main --list` names no recipe: `main` is the
  value of `--group`. The value-taking flags are enumerated from `just --help`.
* **Placeholders.** `just test-<name>`, `just <plat> build` — a shape, not a
  name.
* **Trailing comments.** `just ci l1  # the tier the queue runs` — the comment
  is not an argument.

So only fenced code blocks and inline code spans are read, and within them the
command line is tokenised rather than pattern-matched.

## Namespace by location

`packages/cli/` is a separate workspace with its OWN justfile — 22 flat
recipes, no `check` module — so its documents resolve against that file. A scan
that assumes one namespace reports every CLI document as broken, and a
phase-399 sweep that assumed the opposite rewrote `just check-python` into
`just check python` there: correct in the root namespace, meaningless in that
one.

Run: python3 scripts/check-doc-recipe-refs.py [--self-test]
"""

import argparse
import importlib.util
import re
import shlex
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "lib"))
from tracked import tracked  # issue 0721: index lookup, not a walk

REPO = Path(__file__).resolve().parent.parent
BASELINE = REPO / ".config" / "doc-recipe-refs-baseline.txt"

# `just --help`. A flag that takes a value swallows the token after it, which
# would otherwise read as the recipe name.
VALUE_FLAGS = {
    "--alias-style",
    "-c",
    "--ceiling",
    "--chooser",
    "--color",
    "--command",
    "--command-color",
    "--completions",
    "--cygpath",
    "-d",
    "--dotenv-command",
    "--dotenv-filename",
    "--dotenv-path",
    "--dump-format",
    "-E",
    "--evaluate-format",
    "-f",
    "-F",
    "--group",
    "--indentation",
    "--jobs",
    "--justfile",
    "--justfile-name",
    "--list-heading",
    "--list-prefix",
    "-s",
    "--shell",
    "--shell-arg",
    "--show",
    "--tempdir",
    "--timestamp-format",
    "--usage",
    "--working-directory",
}
TWO_VALUE_FLAGS = {"--set"}

# Flags that ACT instead of running a recipe. Nothing after them is a recipe
# name — `just --list          215 lines, opening with a wall of check-*` is a
# code block annotating output, and read `215` as the recipe.
TERMINAL_FLAGS = {
    "--list",
    "-l",
    "--summary",
    "--help",
    "-h",
    "--version",
    "-V",
    "--dump",
    "--edit",
    "-e",
    "--evaluate",
    "--fmt",
    "--init",
    "--variables",
    "--changelog",
    "--choose",
    "--groups",
    "--man",
}

# A shape rather than a name. Three spellings, all found in the tree:
#   angle/brace/bracket  `just test-<name>`, `just {{plat}} build`
#   enumeration          `just zephyr build-one/ci-both/check-copy-out`
#                        `just zephyr talker/listener/...`
#   glob                 `just zephyr test*`, `just zephyr run-fvp-aemv8r*`
# A recipe name contains none of these characters, so treating them as shapes
# cannot hide a real dead reference — and NOT treating them so reports a
# sentence, which is the failure mode that makes a gate ignorable.
PLACEHOLDER = re.compile(r"[<>{}\[\]…/*,|]|-$")

FENCE = re.compile(r"^\s*(?:```+|~~~+)")
INLINE = re.compile(r"`([^`\n]+)`")
ENV_PREFIX = re.compile(r"^(?:[A-Za-z_][A-Za-z0-9_]*=\S*\s+)+")
PROMPT = re.compile(r"^(?:\$|>|#)\s+")


def load_namespace(justfile: Path):
    """(roots, mods) for a justfile, via `check-just-recipe-refs`'s parser.

    Imported rather than re-implemented: two parsers for one grammar is this
    repository's named defect class, and the sibling gate already has one that
    handles `import` (a merge) and `mod` (a namespace) correctly.
    """
    spec = importlib.util.spec_from_file_location(
        "_refs", REPO / "scripts" / "check-just-recipe-refs.py"
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    roots, mods = mod.recipe_namespace()
    if justfile == REPO / "justfile":
        return roots, mods
    # A sub-workspace justfile is flat (no modules) — but its documents may
    # legitimately name a ROOT recipe too: `just setup-cli` in
    # `packages/cli/CLAUDE.md` is the repo-root recipe that builds this CLI, run
    # from the repo root. Requiring the sub-workspace namespace alone reported
    # every such line as dead. The union is the honest test: the name must
    # exist in a namespace the reader could plausibly be in.
    return mod.names_in(justfile) | roots, mods


def snippets(text: str):
    """Fenced-block lines and inline code spans — never running prose."""
    out = []
    in_fence = False
    for line in text.split("\n"):
        if FENCE.match(line):
            in_fence = not in_fence
            continue
        if in_fence:
            out.append(line)
            continue
        out.extend(INLINE.findall(line))
    return out


def invocation(snippet: str):
    """(recipe, next_token) for a `just` command line, else None."""
    s = PROMPT.sub("", snippet.strip())
    s = ENV_PREFIX.sub("", s)
    if not re.match(r"^just(\s|$)", s):
        return None
    try:
        toks = shlex.split(s, comments=True, posix=True)
    except ValueError:
        toks = s.split()
    toks = toks[1:]
    i = 0
    while i < len(toks):
        t = toks[i]
        if t.startswith("-"):
            if t in TERMINAL_FLAGS:
                return None
            if "=" in t:
                i += 1
            elif t in TWO_VALUE_FLAGS:
                i += 3
            elif t in VALUE_FLAGS:
                i += 2
            else:
                i += 1
            continue
        nxt = (
            toks[i + 1]
            if i + 1 < len(toks) and not toks[i + 1].startswith("-")
            else None
        )
        return t, nxt
    return None


def resolves(recipe, second, roots, mods) -> bool:
    if PLACEHOLDER.search(recipe):
        return True
    if recipe in roots:
        return True
    if recipe in mods:
        # `just <mod>` alone lists it; `just <mod> <recipe>` must resolve.
        return (
            second is None or second in mods[recipe] or bool(PLACEHOLDER.search(second))
        )
    return False


def doc_groups():
    """[(justfile, [documents])] — the namespace a document resolves against."""
    # issue 0721 — the index, not a walk. `book/` and `docs/` are cheap, but
    # `packages/cli` below is not: it holds `target/`, and a `**/*.md` glob
    # descends into every build tree to produce the paths it then discards.
    # Same helper as the sibling gates rather than a second spelling.
    root_docs = set()
    for p in tracked("docs", "book/src", suffix=".md"):
        # `archived/` records what was true THEN; a retired recipe named
        # there is history, not a broken instruction.
        if "archived" in p.parts:
            continue
        root_docs.add(p)
    for name in ("README.md", "AGENTS.md", "CLAUDE.md"):
        if (REPO / name).is_file():
            root_docs.add(REPO / name)

    cli_root = REPO / "packages" / "cli"
    # `git ls-files`, not a glob: these are TRACKED docs, and a recursive walk
    # under `packages/cli` stats the whole vendored `third-party/play_launch`
    # tree and every `target/` to throw the results away
    # (`check-no-tracked-file-find`; measured 7m36s -> 0.8s for the same set).
    #
    # Resolved against main's independent fix of the same bug, which inlined a
    # `git ls-files` subprocess here. Same diagnosis, but scripts/lib/tracked.py
    # exists precisely so this is not spelled twice — CLAUDE.md's "add ONE
    # shared helper rather than a second spelling", which is the rule the
    # #282 -> #326 pair was filed under. The `target` filter main carried is
    # dropped as dead: git never tracks it, so the index cannot yield it.
    # The `third-party` test is RELATIVE to the repo, and that is the whole of
    # the fix. It read `p.parts` on the ABSOLUTE path, so a checkout that
    # happens to live under a directory of that name filtered out EVERY cli
    # document instead of just the vendored `packages/cli/third-party/`
    # subtree -- and vendoring nano-ros as `<super>/third-party/nano-ros` is
    # exactly how the safety island consumes it.
    #
    # The gate then read GREEN on that checkout while scanning nothing, which
    # is worse than red: on 2026-09-05 it made ten baseline waivers look dead,
    # they were pruned as "no longer offend", and CI -- whose checkout is not
    # under a `third-party` directory -- failed on all ten. A gate whose
    # coverage depends on where the tree was cloned reports the clone, not the
    # code.
    cli_docs = {p for p in tracked("packages/cli", suffix=".md")
                if "third-party" not in p.relative_to(REPO).parts}
    return [
        (REPO / "justfile", sorted(root_docs)),
        (cli_root / "justfile", sorted(cli_docs)),
    ]


def offenders():
    out = []
    for justfile, docs in doc_groups():
        if not justfile.is_file():
            continue
        roots, mods = load_namespace(justfile)
        for doc in docs:
            for sn in snippets(doc.read_text(errors="replace")):
                if "{{" in sn:
                    continue
                inv = invocation(sn)
                if not inv:
                    continue
                recipe, second = inv
                if not resolves(recipe, second, roots, mods):
                    out.append((str(doc.relative_to(REPO)), recipe))
    return sorted(set(out))


def read_baseline():
    if not BASELINE.is_file():
        return set()
    rows = set()
    for line in BASELINE.read_text().split("\n"):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        path, _, recipe = line.partition("\t")
        rows.add((path.strip(), recipe.strip()))
    return rows


def self_test() -> int:
    roots = {"ci", "format"}
    mods = {
        "check": {"fast", "default"},
        "native": {"build"},
        "zephyr": {"build-one", "ci-both"},
    }
    cases = [
        ("just ci", True),
        ("just format", True),
        ("just check fast", True),
        ("just check", True),
        ("just check nope", False),
        ("just nope", False),
        # flag arguments are not recipes
        ("just --group main --list", True),
        ("just --list", True),
        ("just -f other/justfile ci", True),
        # placeholders are shapes, not names
        ("just test-<name>", True),
        ("just check <gate>", True),
        # env prefix, prompt, trailing comment
        ("NROS_X=1 just ci", True),
        ("$ just ci", True),
        ("just ci  # the tier the queue runs", True),
        # prose is not a command — `just` is an English word
        ("just the thing", False),
        # terminal flags ACT; what follows annotates output, it is not a recipe
        ("just --list          215 lines, opening with a wall of check-*", True),
        ("just --summary", True),
        ("just -l", True),
        # enumerations and globs are shapes, not names
        ("just zephyr build-one/ci-both/check-copy-out", True),
        ("just zephyr test*", True),
        ("just freertos|nuttx|threadx_linux setup", True),
        # ...but a single real module recipe still has to resolve
        ("just check nope", False),
    ]
    failures = 0
    for text, want_ok in cases:
        inv = invocation(text)
        got = True if inv is None else resolves(inv[0], inv[1], roots, mods)
        if got != want_ok:
            print(f"  self-test FAIL: {text!r}: resolves={got}, want {want_ok}")
            failures += 1

    # `just the thing` is the one that must be caught, and it is caught as an
    # OFFENDER rather than skipped — the gate reports it and a reviewed baseline
    # is where a sentence gets rejected, not a silent regex.
    if snippets("prose `just ci` more\n```\njust check fast\n```\ntail") != [
        "just ci",
        "just check fast",
    ]:
        print("  self-test FAIL: snippets() did not read inline + fenced code")
        failures += 1
    if snippets("plain prose with just the word in it\n") != []:
        print("  self-test FAIL: snippets() read running prose")
        failures += 1

    if failures:
        print(f"check-doc-recipe-refs self-test: {failures} case(s) FAILED")
        return 1
    print(f"check-doc-recipe-refs self-test: OK ({len(cases)} cases + extraction)")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument(
        "--write-baseline",
        action="store_true",
        help="rewrite the baseline from the current tree (review the diff)",
    )
    args = ap.parse_args()

    if args.self_test:
        return self_test()
    if self_test() != 0:
        return 1

    found = set(offenders())

    if args.write_baseline:
        lines = [
            "# `just <recipe>` references in documents that name no recipe.",
            "#",
            "# A RATCHET, not an allowlist: this file may only SHRINK. The gate",
            "# fails when a new dead reference appears AND when a line here stops",
            "# offending — so the debt cannot grow and cannot go stale.",
            "#",
            "# NOT every entry is a defect, and this is the part to read before",
            "# 'fixing' one. Three classes name a non-existent recipe CORRECTLY,",
            "# and rewriting them would replace a true sentence with a wrong",
            "# command:",
            "#",
            "#   NEGATION      AGENTS.md: 'there is no supported ... `just",
            "#                 install-local` flow'. The point IS that it does",
            "#                 not exist.",
            "#   PROPOSAL      docs/research/sdk-ux/: '`just monitor <board>`",
            "#                 that decodes panics' — a UX being argued for, not",
            "#                 an instruction.",
            "#   RELEASE NOTE  migration-install-local-removal.md names the",
            "#                 recipe it documents the REMOVAL of.",
            "#",
            "# The rest are stale instructions: a reader copies them and gets",
            "# `Justfile does not contain recipe`. Those are worth fixing, and",
            "# each needs the recipe that replaced it — `just --list`, or the",
            "# module's own list via `just <mod>`.",
            "#",
            "# Regenerate deliberately with --write-baseline and read the diff.",
            "",
        ]
        lines += [f"{p}\t{r}" for p, r in sorted(found)]
        BASELINE.write_text("\n".join(lines) + "\n")
        print(f"check-doc-recipe-refs: wrote {len(found)} baseline entr(ies)")
        return 0

    baseline = read_baseline()
    new = sorted(found - baseline)
    fixed = sorted(baseline - found)

    rc = 0
    if new:
        print(
            f"check-doc-recipe-refs: {len(new)} document(s) name a recipe that does not exist:"
        )
        for path, recipe in new:
            print(f"  {path}: just {recipe}")
        print()
        print("  A reader copies these. Name the recipe that exists, or drop the line.")
        print(
            "  `just --list` shows every recipe; module recipes are `just <mod> <name>`."
        )
        rc = 1
    if fixed:
        print(
            f"check-doc-recipe-refs: {len(fixed)} baseline entr(ies) no longer offend — delete them:"
        )
        for path, recipe in fixed:
            print(f"  {path}\t{recipe}")
        print()
        print("  The baseline may only SHRINK, and it may not go stale: a line that")
        print("  no longer describes anything is one nobody can act on.")
        rc = 1
    if rc == 0:
        print(
            f"check-doc-recipe-refs: OK — {len(baseline)} known dead reference(s), no new ones."
        )
    return rc


if __name__ == "__main__":
    sys.exit(main())
