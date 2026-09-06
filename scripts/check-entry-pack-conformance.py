#!/usr/bin/env python3
"""phase-432 W3.3 — an entry pack that is half-wired must fail LOUDLY.

"Adding a language is cheap" is only good news if adding a BROKEN language is
not equally cheap. A pack can be half-wired in several ways, and every one of
them is quiet:

  * a directory under `packs/entry/` with no `pack.toml` — a pack nobody
    describes, so CMake cannot name its output or pick its compiler;
  * a manifest naming a template that does not exist, or omitting one that
    does — minijinja resolves templates at RENDER time, so this surfaces on
    some user's build rather than here;
  * a language pack missing `extension`, `c_family` or `entry_template`;
  * a `Language` variant with no pack at all — the enumeration says the
    language exists and nothing renders it;
  * a language with no GOLDEN coordinate, so its bytes are recorded nowhere and
    a change to them is invisible.

The Rust-side unit tests in `codegen/entry/pack.rs` and `render.rs` cover the
manifest/registry agreement, because they can read the bundled data directly
where a script could only re-spell it. This gate covers what those cannot see:
the DIRECTORY (a pack that exists on disk and in no manifest), the LANGUAGE
enumeration, and the golden corpus.

WHY THIS IS A SEPARATE FILE FROM THE UNIT TESTS

`check-cli-tests` runs the unit tests and is on the required PR context, so the
Rust half is merge-gating already. This half is on the FAST line, which is what
a contributor runs before pushing — the point of a conformance gate is that it
answers early, and a cargo build is not early.

Usage::

    check-entry-pack-conformance.py            # the gate
    check-entry-pack-conformance.py --audit    # show what it found, never fails
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PACKS = ROOT / "packages/cli/nros-cli-core/src/codegen/entry/packs/entry"
GOLDENS = ROOT / "packages/cli/nros-cli-core/testdata/entry"
LANG_SRC = ROOT / "packages/cli/nros-lang/src/lib.rs"

# A language pack must declare all of these; the shared pack must declare none.
LANGUAGE_FIELDS = ("language", "extension", "c_family", "entry_template")


class ManifestError(Exception):
    """A manifest this parser cannot read. Never a silent skip."""


def parse_manifest(text: str) -> dict:
    """The flat subset of TOML a `pack.toml` uses, parsed without a dependency.

    This host's Python is 3.10, which has no `tomllib`, and the repo's rule is
    that a gate brings no dependencies (see `check-provider-announcements.py`).
    A pack manifest is deliberately flat — bare `key = value` and
    `key = ["a", "b"]`, no tables, no nesting — so the subset needed is small.

    It REFUSES anything outside that subset rather than skipping the line. A
    parser that silently ignores what it does not understand is how a gate ends
    up reporting green over a manifest it never read.
    """
    out: dict = {}
    pending_key: str | None = None
    pending_items: list[str] = []
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip() if not raw.strip().startswith("#") else ""
        if not line:
            continue
        if pending_key is not None:
            pending_items += re.findall(r'"([^"]*)"', line)
            if "]" in line:
                out[pending_key] = pending_items
                pending_key, pending_items = None, []
            continue
        m = re.match(r'^([A-Za-z_][\w-]*)\s*=\s*(.*)$', line)
        if not m:
            raise ManifestError(f"cannot parse line: {line!r}")
        key, value = m.group(1), m.group(2).strip()
        if value.startswith("["):
            items = re.findall(r'"([^"]*)"', value)
            if "]" in value:
                out[key] = items
            else:
                pending_key, pending_items = key, items
        elif value.startswith('"') and value.endswith('"') and len(value) >= 2:
            out[key] = value[1:-1]
        elif value in ("true", "false"):
            out[key] = value == "true"
        else:
            raise ManifestError(f"unsupported value for `{key}`: {value!r}")
    if pending_key is not None:
        raise ManifestError(f"unterminated list for `{pending_key}`")
    return out


def tracked_pack_dirs() -> list[Path]:
    """Every `packs/entry/<surface>/` directory, from git rather than a walk.

    `--others --exclude-standard` alongside `--cached` so a pack added but not
    yet committed reds NOW, while its author is looking at it, rather than on
    someone else's push. That is the same choice `check-entry-locator-ssot`
    makes, and for the same reason.
    """
    rel = PACKS.relative_to(ROOT)
    out = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", str(rel)],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.split()
    dirs = {ROOT / Path(p).parent for p in out}
    return sorted(d for d in dirs if d != PACKS)


def declared_languages() -> list[str]:
    """The `Language` variants, read from the crate that owns the enumeration.

    Parsed rather than restated: a second list of "which languages exist" is
    exactly the drift phase-432 removes, and this gate would be a poor place to
    reintroduce it.
    """
    text = LANG_SRC.read_text(encoding="utf-8")
    m = re.search(r"pub enum Language\s*\{(.*?)\n\}", text, re.S)
    if not m:
        raise SystemExit(
            f"check-entry-pack-conformance: cannot find `pub enum Language` in "
            f"{LANG_SRC.relative_to(ROOT)} — the parse this gate depends on has "
            "moved, so it would report green having checked nothing."
        )
    body = re.sub(r"//[^\n]*", "", m.group(1))
    return [v.lower() for v in re.findall(r"^\s*([A-Z]\w*)\s*,", body, re.M)]


def golden_languages() -> set[str]:
    """Languages that have at least one recorded golden.

    Keyed on the file EXTENSION rather than the name prefix: a row is named for
    what it exercises, and tying this to a naming convention would make it a
    check on names instead of on coverage.
    """
    seen = set()
    for p in GOLDENS.glob("*.golden"):
        # `<name>.<ext>.golden`
        parts = p.name.split(".")
        if len(parts) >= 3:
            seen.add(parts[-2])
    return seen


def check() -> list[str]:
    problems: list[str] = []

    dirs = tracked_pack_dirs()
    if not dirs:
        problems.append(
            f"no pack directories under {PACKS.relative_to(ROOT)} — this gate "
            "would pass having checked nothing"
        )
        return problems

    manifests: dict[str, dict] = {}
    for d in dirs:
        mf = d / "pack.toml"
        rel = d.relative_to(ROOT)
        if not mf.is_file():
            problems.append(
                f"{rel}: a pack directory with no `pack.toml`. CMake cannot name "
                "its output or pick its compiler, and nothing says whether it is "
                "a language pack or shared partials."
            )
            continue
        try:
            manifests[d.name] = parse_manifest(mf.read_text(encoding="utf-8"))
        except ManifestError as e:
            problems.append(f"{rel}/pack.toml: {e}")

    for surface, m in sorted(manifests.items()):
        rel = f"packs/entry/{surface}/pack.toml"
        if m.get("shared"):
            stray = [f for f in LANGUAGE_FIELDS if f in m]
            if stray:
                problems.append(
                    f"{rel}: shared pack declares {', '.join(stray)} — a shared "
                    "pack renders no TU, so declaring a language pack's fields "
                    "makes the two kinds confusable."
                )
            if not m.get("partials"):
                problems.append(f"{rel}: shared pack declares no partials, so it is nothing.")
            continue
        missing = [f for f in LANGUAGE_FIELDS if f not in m]
        if missing:
            problems.append(
                f"{rel}: language pack is missing {', '.join(missing)}. "
                "A pack CMake cannot describe is a pack that emits nothing and "
                "looks wired."
            )

    # NOT checked here: that each template a manifest names exists on disk.
    # A manifest names REGISTRY KEYS, and the registry maps keys to paths with
    # `include_str!` — so a named-but-missing FILE is a compile error, and a
    # key the registry does not have is caught by the Rust test
    # `manifests_and_the_registry_describe_the_same_templates`. Re-checking it
    # here would mean re-spelling the key→path map, which is the drift this
    # gate exists to prevent.

    # Every declared Language has a pack, and every language pack names a
    # declared Language.
    langs = set(declared_languages())
    if not langs:
        problems.append("`Language` parsed to zero variants — refusing to report green.")
    pack_langs = {m["language"] for m in manifests.values() if "language" in m}
    for lang in sorted(langs - pack_langs):
        problems.append(
            f"language `{lang}` is a `Language` variant with no entry pack. The "
            "enumeration says it exists; nothing renders it."
        )
    for lang in sorted(pack_langs - langs):
        problems.append(
            f"a pack declares language `{lang}`, which is not a `Language` "
            "variant — one of the two is wrong."
        )

    # Every language has a golden coordinate. Without one its bytes are
    # recorded nowhere and a change to them is invisible.
    goldens = golden_languages()
    # `.get`, not `[...]`: a manifest missing `extension` has ALREADY been
    # reported above, and a gate that crashes on the second finding tells you
    # about one problem when it found two.
    ext_by_lang = {
        m["language"]: m.get("extension")
        for m in manifests.values()
        if "language" in m
    }
    for lang in sorted(pack_langs):
        ext = ext_by_lang.get(lang)
        if ext and ext not in goldens:
            problems.append(
                f"language `{lang}` has no golden with extension `.{ext}` in "
                f"{GOLDENS.relative_to(ROOT)} — its generated bytes are recorded "
                "nowhere, so a change to them is invisible."
            )
    return problems


def selftest() -> None:
    """The predicates must catch what they exist for, and pass what they must.

    Run on the NORMAL path, not behind a flag: a negative control nobody runs
    decays into a comment. Every case here is a shape this gate was written for,
    driven through the pure functions so it needs no files on disk.
    """
    # The manifest parser refuses what it cannot read, rather than skipping it.
    for bad in ('language = c\n', 'partials = ["a"\n', "language = 'c'\n"):
        try:
            parse_manifest(bad)
        except ManifestError:
            pass
        else:
            raise SystemExit(
                f"SELFTEST FAIL: parse_manifest accepted {bad!r} — a parser that "
                "ignores what it does not understand reports green over a file it "
                "never read."
            )

    # ...and reads the shapes a real manifest uses, including a multi-line list
    # and a comment on its own line.
    got = parse_manifest(
        '# a comment\n'
        'language = "c"   # trailing comment\n'
        'c_family = true\n'
        'partials = [\n    "a.jinja",\n    "b.jinja",\n]\n'
    )
    want = {"language": "c", "c_family": True, "partials": ["a.jinja", "b.jinja"]}
    if got != want:
        raise SystemExit(f"SELFTEST FAIL: parse_manifest gave {got!r}, want {want!r}")

    # The `Language` parse must find variants. If it silently found none, every
    # "variant with no pack" check below would vacuously pass.
    if not declared_languages():
        raise SystemExit(
            "SELFTEST FAIL: declared_languages() found no variants, so the "
            "language checks would pass having compared nothing."
        )

    # A golden extension the corpus really has, and one it cannot have.
    goldens = golden_languages()
    if not goldens:
        raise SystemExit("SELFTEST FAIL: golden_languages() found nothing.")
    if "nosuchext" in goldens:
        raise SystemExit("SELFTEST FAIL: golden_languages() invented an extension.")


def main() -> int:
    selftest()

    if "--audit" in sys.argv:
        print(f"pack dirs under {PACKS.relative_to(ROOT)}:")
        for d in tracked_pack_dirs():
            has = "pack.toml" if (d / "pack.toml").is_file() else "NO MANIFEST"
            print(f"  {d.name:8} {has}")
        print(f"Language variants: {', '.join(declared_languages())}")
        print(f"golden extensions: {', '.join(sorted(golden_languages()))}")
        return 0

    problems = check()
    if problems:
        print(
            "FAIL: an entry pack is half-wired.\n"
            "  A pack that is described incompletely does not fail to build — it\n"
            "  emits nothing, or emits into a file the toolchain will not compile,\n"
            "  and the first symptom is on someone's board.\n",
            file=sys.stderr,
        )
        for p in problems:
            print(f"  {p}", file=sys.stderr)
        return 1

    dirs = tracked_pack_dirs()
    print(
        f"check-entry-pack-conformance: OK ({len(dirs)} pack(s), "
        f"{len(declared_languages())} language(s), each with a manifest and a golden)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
