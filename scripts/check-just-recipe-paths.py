#!/usr/bin/env python3
"""A recipe's literal in-repo paths must exist, and carry the manifest its tool needs.

Issue 1097. `just` never looks at a path a recipe names — the shell does, when
the recipe RUNS. So a directory that moved, a script that was never committed,
or a crate that turned into a CMake project leaves the recipe parsing fine,
listing fine, and dying only for whoever types it. If that recipe is also on no
lane, nobody types it, and the rot is invisible for months.

Two such recipes were live when this gate was written, and BOTH were born dead —
neither ever worked, not once, on any host:

* `just xrce check-rust-rmw` (issue 1097) — `cd packages/rmw/xrce/nros-rmw-xrce`
  then `cargo check`. That directory is a CMake project: `CMakeLists.txt`,
  `src/*.c`, `package.xml`, no `Cargo.toml`. The crate it names was deleted on
  2026-05-12 (`f5aeb8faf`, phase-115.K.2) and the recipe was written on
  2026-05-13 (`68c89568c`, phase-121.7.i), whose roadmap entry claims "CI now
  exercises it".
* `just native test-ros2-shell` — `./tests/ros2-interop.sh`, a script phase-16
  planned and never committed. It has never existed in git history at all.

The 1097 case is the sharper one, and it is why this gate checks the MANIFEST
and not merely the directory. `cargo` walks UP for a manifest, so the missing
`Cargo.toml` does not produce "could not find Cargo.toml" — cargo resolves the
ROOT workspace and the recipe silently becomes a full-workspace `cargo check`.
It therefore checks every crate EXCEPT the xrce one it claims to check (the
cffi crate is in `HOST_UNCHECKABLE`), and fails on `nros-c`'s duplicate
`panic_impl` — a red that says nothing whatsoever about XRCE. A directory-only
check would have called that recipe healthy.

## What is checked

Inside recipe bodies only (never comments), for LITERAL paths only:

1. `cd <dir>` / `pushd <dir>` — the directory must exist.
2. The tool run in that directory must find its manifest there:
   `cargo` -> `Cargo.toml`, `cmake`/`west build`/`idf.py` -> `CMakeLists.txt`,
   `nros generate-rust` -> `package.xml`.
3. `cmake -S <dir>` — `<dir>/CMakeLists.txt` must exist (an explicit `-S`
   replaces the working directory, so rule 2 does not apply to that call).
4. Every `scripts/…` or `tests/…` `.sh`/`.py` a body names must exist.

Recipe bodies come in two shapes and the difference is load-bearing: WITHOUT a
`#!` shebang, `just` runs each line in its own shell, so a bare `cd` line is
inert and the tool two lines down runs at the repo root. This gate models that
— it only demands a manifest of a directory the tool actually runs in. Wiring
those together without the shebang distinction would invent failures in the
majority of the corpus.

## What is deliberately NOT checked, and why the claim is scoped to match

* **Dynamic targets** — `cd "$WORKSPACE"`, `cd "{{justfile_directory()}}/$LEAF"`.
  14 of the 42 `cd` sites are these; the value is not known until run time and
  guessing would produce false positives on the one shape a human cannot verify
  either. Issue 0196's rule is that a gate must not claim more coverage than it
  has, so the claim here is "every LITERAL in-repo path", not "every path".
* **Paths inside a registered submodule, and git-ignored paths.** A fresh clone
  legitimately lacks `third-party/tracing/Tonbandgeraet/tools` and
  `scripts/zephyr/.venv`; recipes that use them already guard on existence.
  Both sets are DERIVED (`.gitmodules`, `git check-ignore`), never a hardcoded
  list of today's exceptions.
* **A literal whose first segment is not a directory at the repo root** —
  `cd accept_app` runs inside a scratch tree `nros new` just created. Not a
  repo path, so not this gate's business.
* **Recipe NAMES** — `check-just-recipe-refs.py` is the sibling gate for those,
  and already carries the `--test <target>` rule for the same reason. Names
  there, filesystem paths here; merging them is defensible, but a gate called
  `recipe-refs` reporting a missing directory reads as the wrong gate failing.

Run: python3 scripts/check-just-recipe-paths.py [--selftest]
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

# A recipe DEFINITION at column 0: name, optional parameters/dependencies, `:`.
# `:=` is an assignment, not a recipe.
RECIPE_DEF = re.compile(r"^([A-Za-z_][A-Za-z0-9_-]*)(?:\s+[^:\n]*)?:(?!=)")

# `cd`/`pushd` at a command position: line start, after `&&`/`||`/`;`/`(`, or
# after a `@`/`-` recipe prefix.
CD = re.compile(r"^\s*(?:[@-]\s*)?(?:cd|pushd)\s+(\S+)")

# Which manifest each tool needs in the directory it runs in. Written as
# tool -> filename so a crossed wire (cargo wanting CMakeLists.txt) is one
# swapped value, which is exactly what the self-test's mutation cases attack.
TOOL_MANIFEST = {
    "cargo": "Cargo.toml",
    "cmake": "CMakeLists.txt",
    "west": "CMakeLists.txt",
    "idf.py": "CMakeLists.txt",
    "nros generate-rust": "package.xml",
}

# A tool at a command position within one `&&`/`;`-delimited segment. `west`
# only configures a build under `west build`; `west update` cares nothing for a
# CMakeLists.txt, so the tool key has to include the subcommand.
TOOL_AT_COMMAND = re.compile(
    r"^\s*(?:[@-]\s*)?(?:[A-Za-z_][A-Za-z0-9_]*=\S+\s+)*"
    r"(cargo|cmake|idf\.py|west\s+build|nros\s+generate-rust)\b"
)

# `cmake -S <dir>`: an explicit source dir replaces the working directory.
CMAKE_S = re.compile(r"\bcmake\b[^\n]*?\s-S\s+(\S+)")

# A repo script a body names. Kept to `scripts/` and `tests/` because those are
# the two trees whose contents a recipe executes; a `docs/` path in a body is
# prose, and this gate does not check prose.
SCRIPT_REF = re.compile(
    r"(?<![A-Za-z0-9_./-])(?:\./)?((?:scripts|tests)/[A-Za-z0-9._/-]+\.(?:sh|py))"
)

# Anything that makes a token's value unknowable before the recipe runs.
DYNAMIC = ("{{", "$", "*", "?", "`")


def just_files(repo):
    """The justfile corpus: the root file plus every module/import under just/."""
    if (repo / "justfile").is_file():
        yield repo / "justfile"
    yield from sorted((repo / "just").glob("*.just"))


def submodule_prefixes(repo):
    """Paths registered in `.gitmodules`, as repo-relative strings.

    Derived rather than listed: a submodule added tomorrow is skipped for the
    same reason today's are, with no edit here.
    """
    gm = repo / ".gitmodules"
    if not gm.is_file():
        return []
    return re.findall(r"^\s*path\s*=\s*(.+?)\s*$", gm.read_text(errors="replace"), re.M)


def ignored(repo, paths):
    """The subset of `paths` git deliberately does not track.

    `--no-index` because plain `check-ignore` refuses any path that lies inside
    a submodule ("fatal: Pathspec ... is in submodule"), which would abort the
    whole gate over a path it was going to skip anyway. If git is unavailable
    the answer is "nothing is ignored" — a gate that cannot consult git should
    over-report, not silently exempt everything.
    """
    if not paths:
        return set()
    try:
        proc = subprocess.run(
            ["git", "-C", str(repo), "check-ignore", "--no-index", "--stdin"],
            input="\n".join(sorted(paths)) + "\n",
            capture_output=True,
            text=True,
            timeout=30,
        )
    except (OSError, subprocess.SubprocessError):
        return set()
    return {line.strip() for line in proc.stdout.splitlines() if line.strip()}


def unquote(tok):
    tok = tok.strip()
    for q in ('"', "'"):
        if len(tok) >= 2 and tok[0] == q and tok[-1] == q:
            return tok[1:-1]
    return tok


def is_repo_literal(repo, tok):
    """True when `tok` is a literal path naming a tree at the repo root.

    The first-segment test is what keeps `cd accept_app` (a scratch dir `nros
    new` creates under tmp/) out of scope without naming it: its first segment
    is not a directory at the repo root, so it is not a repo path.
    """
    if not tok or tok.startswith("/") or tok.startswith("~"):
        return False
    if any(d in tok for d in DYNAMIC):
        return False
    if tok.startswith("./"):
        tok = tok[2:]
    if ".." in tok.split("/"):
        return False
    first = tok.split("/", 1)[0]
    return bool(first) and (repo / first).is_dir()


def normalise(tok):
    return tok[2:] if tok.startswith("./") else tok


def recipes(path):
    """Yield (name, start_lineno, [(lineno, text)], has_shebang) per recipe."""
    lines = path.read_text(errors="replace").splitlines()
    i = 0
    while i < len(lines):
        m = RECIPE_DEF.match(lines[i])
        if not m:
            i += 1
            continue
        name, start = m.group(1), i + 1
        body, j = [], i + 1
        while j < len(lines):
            line = lines[j]
            if line.strip() and not line[:1].isspace():
                break
            if line.strip():
                body.append((j + 1, line))
            j += 1
        shebang = bool(body) and body[0][1].strip().startswith("#!")
        yield name, start, body, shebang
        i = j


def segments(line):
    """Split one body line into command segments on `&&`, `||`, `;`."""
    return [s for s in re.split(r"&&|\|\||;", line) if s.strip()]


def scan(repo):
    """[(path, lineno, kind, detail, line)] — every literal path defect found."""
    subs = submodule_prefixes(repo)
    candidates, findings = set(), []

    def in_submodule(rel):
        return any(rel == s or rel.startswith(s + "/") for s in subs)

    # Pass 1: collect. Pass 2 needs the git-ignore verdict for every candidate
    # at once, so the gate shells out to git exactly once however large the
    # corpus grows.
    raw = []
    for path in just_files(repo):
        for _name, _start, body, shebang in recipes(path):
            cwd = None  # None = repo root; "?" = dynamic, unknowable
            for lineno, line in body:
                text = line.strip()
                if text.startswith("#"):
                    continue
                if not shebang:
                    # Each line is its own shell: a previous line's `cd` is gone.
                    cwd = None
                for seg in segments(line):
                    m = CD.match(seg)
                    if m:
                        tok = unquote(m.group(1))
                        if any(d in tok for d in DYNAMIC):
                            cwd = "?"
                        elif is_repo_literal(repo, tok):
                            cwd = normalise(tok)
                            candidates.add(cwd)
                            raw.append((path, lineno, "cd", cwd, text, None))
                        else:
                            cwd = "?"
                        continue
                    sm = CMAKE_S.search(seg)
                    if sm:
                        tok = unquote(sm.group(1))
                        if is_repo_literal(repo, tok):
                            d = normalise(tok)
                            candidates.add(d)
                            raw.append((path, lineno, "cmake-s", d, text, "CMakeLists.txt"))
                        continue
                    tm = TOOL_AT_COMMAND.match(seg)
                    if tm and cwd not in (None, "?"):
                        tool = re.sub(r"\s+", " ", tm.group(1)).replace("west build", "west")
                        want = TOOL_MANIFEST.get(tool)
                        if want:
                            candidates.add(cwd)
                            raw.append((path, lineno, "manifest", cwd, text, want))
                for tok in SCRIPT_REF.findall(line):
                    if any(d in tok for d in DYNAMIC) or not is_repo_literal(repo, tok):
                        continue
                    d = normalise(tok)
                    candidates.add(d)
                    raw.append((path, lineno, "script", d, text, None))

    skip = ignored(repo, candidates)
    for path, lineno, kind, target, text, want in raw:
        if target in skip or in_submodule(target):
            continue
        full = repo / target
        if kind == "script":
            if not full.is_file():
                findings.append((path, lineno, kind, f"{target} (no such file)", text))
        elif kind == "cd":
            if not full.is_dir():
                findings.append((path, lineno, kind, f"{target} (no such directory)", text))
        else:
            if not full.is_dir():
                # A missing `cd` target is already reported once, as the `cd`.
                # Reporting it again for every tool that follows turns one
                # defect into a paragraph and buries the next one.
                if kind == "manifest":
                    continue
                findings.append((path, lineno, kind, f"{target} (no such directory)", text))
            elif not (full / want).is_file():
                findings.append(
                    (path, lineno, kind, f"{target} has no {want}", text)
                )
    return findings


# --------------------------------------------------------------------------
# Self-test. Runs on EVERY invocation, not behind a flag: a negative control
# nobody runs decays into a comment (scripts/check-gate-selftests.py).
#
# The cases are built as a REAL throwaway repo tree, because the property under
# test is "does the parser wire this line to that file on disk" — which a
# regex-only test cannot exercise, and which is precisely where the two mutation
# classes below live.
# --------------------------------------------------------------------------

SELFTEST_JUSTFILE = """\
set dotenv-load

# a comment names a dead path: cd packages/gone && cargo build --in-a-comment
good-cargo:
    #!/usr/bin/env bash
    cd packages/rustcrate
    cargo build --good-cargo

crossed-cargo:
    #!/usr/bin/env bash
    cd packages/cmakeproj
    cargo check --crossed-cargo

crossed-cmake:
    #!/usr/bin/env bash
    cd packages/rustcrate
    cmake -B build --crossed-cmake

good-cmake:
    #!/usr/bin/env bash
    cd packages/cmakeproj
    cmake -B build --good-cmake

missing-dir:
    #!/usr/bin/env bash
    cd packages/gone
    cargo build --missing-dir

no-shebang-cd-is-inert:
    cd packages/cmakeproj
    cargo build --inert

same-line-joins:
    cd packages/cmakeproj && cargo build --joined

dynamic-target:
    #!/usr/bin/env bash
    cd "$SOMEWHERE"
    cargo build --dynamic

interpolated-target:
    #!/usr/bin/env bash
    cd "{{justfile_directory()}}/leaf"
    cargo build --interpolated

scratch-dir-not-a-repo-path:
    #!/usr/bin/env bash
    cd accept_app
    cargo build --scratch

generate-good:
    #!/usr/bin/env bash
    cd packages/msgpkg
    nros generate-rust --gen-good -o generated

generate-crossed:
    #!/usr/bin/env bash
    cd packages/rustcrate
    nros generate-rust --gen-crossed -o generated

idf-good:
    #!/usr/bin/env bash
    cd packages/cmakeproj
    idf.py build --idf-good

idf-crossed:
    #!/usr/bin/env bash
    cd packages/rustcrate
    idf.py build --idf-crossed

west-good:
    #!/usr/bin/env bash
    cd packages/cmakeproj
    west build --west-good

west-crossed:
    #!/usr/bin/env bash
    cd packages/rustcrate
    west build --west-crossed

west-is-not-always-a-build:
    #!/usr/bin/env bash
    cd packages/rustcrate
    west update --west-update

cmake-source-dir-ok:
    cmake -S packages/cmakeproj -B build --s-ok

cmake-source-dir-bad:
    cmake -S packages/rustcrate -B build --s-bad

script-present:
    ./scripts/real.sh

script-absent:
    ./scripts/ghost.sh

# ./scripts/ghost-in-a-comment.sh
comment-only:
    @echo hi

boundary-donor:
    #!/usr/bin/env bash
    cd packages/rustcrate
    echo donor-done

boundary-receiver:
    #!/usr/bin/env bash
    cmake -B build --receiver
"""

# Every finding the corpus above MUST produce, keyed on a substring unique to
# one line of it — so a finding attributed to the wrong line fails the case
# rather than passing by coincidence.
EXPECTED = [
    ("manifest", "--crossed-cargo",
     "cargo in a CMake-only dir wants Cargo.toml"),
    ("manifest", "--crossed-cmake",
     "cmake in a cargo-only dir wants CMakeLists.txt"),
    ("manifest", "--joined",
     "`cd X && tool` on ONE line: the cd applies even with no shebang"),
    ("cd", "cd packages/gone",
     "a cd target that does not exist"),
    ("cmake-s", "--s-bad",
     "`cmake -S` at a directory with no CMakeLists.txt"),
    ("manifest", "--gen-crossed",
     "`nros generate-rust` in a dir with no package.xml"),
    ("manifest", "--idf-crossed",
     "`idf.py` in a dir with no CMakeLists.txt"),
    ("manifest", "--west-crossed",
     "`west build` in a dir with no CMakeLists.txt"),
    ("script", "ghost.sh",
     "a script the recipe runs and nobody has"),
]

# Every case that must produce NO finding. Each names the mutation it kills:
# these are the wires, and a gate whose negative controls are all "malformed
# input" only proves it does not crash.
CLEAN = [
    ("--good-cargo",
     "a cargo crate that HAS its Cargo.toml — kills a mutation that flags every cd"),
    ("--good-cmake",
     "a cmake project that HAS its CMakeLists.txt — same, for the other tool"),
    ("--inert",
     "MUTATION (shape valid, wires crossed): with no shebang each line is its own "
     "shell, so this recipe's `cd packages/cmakeproj` is INERT and the `cargo` "
     "below it runs at the repo root. Dropping the shebang rule would blame a "
     "real directory with the wrong manifest — a finding that looks reasonable"),
    ("--receiver",
     "MUTATION (shape valid, wires crossed): dropping the RECIPE BOUNDARY would "
     "let boundary-donor's `cd packages/rustcrate` reach this recipe's cmake and "
     "report a cargo-only dir as missing CMakeLists.txt"),
    ("--dynamic", "a dynamic `cd \"$VAR\"` is unknowable, not wrong"),
    ("--interpolated", "a `{{...}}` interpolation is not a literal"),
    ("--scratch", "`cd accept_app` is a scratch tree, not a repo path"),
    ("--s-ok", "`cmake -S` at a real CMake project"),
    ("--missing-dir",
     "a missing `cd` target is reported ONCE, as the cd — not again for every "
     "tool that follows it in the body"),
    ("--gen-good",
     "a package.xml dir under `nros generate-rust` — with --gen-crossed above, "
     "this pins the codegen wire in BOTH directions; a mutation swapping "
     "package.xml for Cargo.toml survived until these two cases existed"),
    ("--idf-good", "a CMakeLists dir under `idf.py`"),
    ("--west-good", "a CMakeLists dir under `west build`"),
    ("--west-update",
     "`west update` is not a build and needs no CMakeLists.txt — the tool key "
     "carries the subcommand, and this is what says so"),
    ("scripts/real.sh", "a script that exists"),
    ("ghost-in-a-comment", "a path inside a comment is prose, not an invocation"),
]


def _selftest_repo(tmp):
    """A real throwaway tree: the property under test is the wiring from a line
    to a file on disk, which no regex-only fixture can exercise."""
    root = Path(tmp)
    (root / "just").mkdir()
    (root / "packages" / "rustcrate").mkdir(parents=True)
    (root / "packages" / "rustcrate" / "Cargo.toml").write_text('[package]\nname="x"\n')
    (root / "packages" / "cmakeproj").mkdir(parents=True)
    (root / "packages" / "cmakeproj" / "CMakeLists.txt").write_text("project(x)\n")
    (root / "packages" / "msgpkg").mkdir(parents=True)
    (root / "packages" / "msgpkg" / "package.xml").write_text("<package/>\n")
    (root / "scripts").mkdir()
    (root / "scripts" / "real.sh").write_text("#!/bin/sh\n")
    (root / "justfile").write_text(SELFTEST_JUSTFILE)
    return root


def selftest(verbose=False):
    import tempfile

    ok = fail = 0

    def chk(desc, cond):
        nonlocal ok, fail
        if verbose or not cond:
            print(f"  {'ok   ' if cond else 'FAIL '} {desc}")
        ok += 1 if cond else 0
        fail += 0 if cond else 1

    with tempfile.TemporaryDirectory() as tmp:
        found = scan(_selftest_repo(tmp))
        got = [(kind, line) for _p, _l, kind, _d, line in found]

        for kind, needle, why in EXPECTED:
            chk(f"flags:  {why}",
                any(k == kind and needle in line for k, line in got))

        for needle, why in CLEAN:
            chk(f"clean:  {why}",
                not any(needle in line for _k, line in got))

        # A gate that finds the six it was told about AND three it was not is
        # not passing — it is over-reporting into a corpus nobody re-reads.
        chk(f"the corpus produces EXACTLY the {len(EXPECTED)} expected findings "
            f"(got {len(found)})",
            len(found) == len(EXPECTED))

        # The manifest map must be consulted, not assumed: swapping its two
        # values keeps the shape valid, so only a case per direction catches it.
        chk("the manifest map has a distinct answer per tool",
            TOOL_MANIFEST["cargo"] != TOOL_MANIFEST["cmake"])

    if verbose:
        print(f"\n{ok} passed, {fail} failed")
    if fail:
        print("check-just-recipe-paths self-test: FAILED", file=sys.stderr)
        raise SystemExit(1)
    return 0


def main():
    ap = argparse.ArgumentParser(description="see module docstring")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest(verbose=True)

    corpus = list(just_files(REPO))
    if not corpus:
        raise SystemExit(
            "check-just-recipe-paths: found NO justfiles — the corpus has moved, "
            "and a gate with an empty corpus passes forever"
        )
    total = sum(len(list(recipes(p))) for p in corpus)
    if total == 0:
        raise SystemExit(
            "check-just-recipe-paths: parsed NO recipes out of "
            f"{len(corpus)} justfile(s) — the definition regex has rotted"
        )

    findings = scan(REPO)
    if not findings:
        print(
            f"check-just-recipe-paths: OK ({total} recipe(s) in {len(corpus)} "
            "justfile(s); every literal in-repo path exists and carries its "
            "tool's manifest)"
        )
        return 0

    print(
        "check-just-recipe-paths: a recipe names a path that cannot work:",
        file=sys.stderr,
    )
    for path, lineno, kind, detail, line in findings:
        print(f"  {path.relative_to(REPO)}:{lineno}: [{kind}] {detail}", file=sys.stderr)
        print(f"      {line}", file=sys.stderr)
    print(
        "\n"
        "  `just` resolves none of this — the shell does, when the recipe RUNS.\n"
        "  A recipe on no lane can therefore be dead for months and still read,\n"
        "  to the next person, as coverage that exists. Issue 1097 is one that\n"
        "  was born dead and sat for four months; `just native test-ros2-shell`\n"
        "  named a script that was never committed at all.\n"
        "\n"
        "  Point it at what replaced the coverage, or delete it — but do not\n"
        "  leave it naming something nobody has had since the move.\n",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    # Always, not only behind the flag: a negative control nobody runs decays
    # into a comment.
    selftest()
    sys.exit(main())
