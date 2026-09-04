#!/usr/bin/env python3
"""No `std::`-qualified stdio in a `#![no_std]` crate's `src/`.

Issue 0589. On Zephyr `native_sim`, a Rust `std::println!` / `std::eprintln!`
does not print — it kills the image. `zvfs_write(1, …)` dispatches to
`stdinout_write_vmeth`, which calls `zvfs_write(1, …)` again, and `k_mutex` is
recursive, so instead of deadlocking it recurses until the stack is gone. The
image dies with `dumped core` and no message. The print that did it was a
diagnostic: `eprintln!("nros: NodeError::{err:?}")`, visible in the backtrace as
a 17-byte buffer.

That is the worst shape a bug can have. The diagnostic replaces the failure it
was describing with a silent core dump, so the more informative the error path,
the less you learn. It is latent in EVERY native_sim image — the Kconfig is
identical in cells that pass — and fires only when a Rust std stdio call is
finally reached, which makes it a landmine rather than a bug in any one test.

## Why this rule, and why it is exactly the right width

`native_sim` compiles Rust for the HOST triple (`nros_host_rust_triple`,
issue 0582) and `zephyr/CMakeLists.txt` appends `,std` to the feature list, so a
`no_std` crate on that image has `std` available and `cfg(feature = "std")` on.
Every landmine has therefore had the same shape:

    #[cfg(feature = "std")]
    std::eprintln!("nros: …");

A `no_std` crate cannot reach `println!` any other way. The macro comes from the
std prelude, which `#![no_std]` does not have, so a bare `println!` in such a
crate is either a compile error or a crate-local macro (`esp_println::println!`,
`nros-board-nuttx`'s own, the mps2 semihosting one) — a platform's own console,
not this hazard. Qualifying with `std::` is thus not a style choice: it is the
only spelling that reaches libstd's stdio, which makes it a complete and exact
signature. Matching bare `println!` too would flag ~150 correct board-console
and build-script calls and teach people to add exemptions.

## What to write instead

`nros_log` — `log_error!` / `log_warn!` / `log_info!` / `nros_trace!` through
`nros_log::get_logger("<crate>")`. It reaches the console when a sink is wired
and is dropped when one is not; never fatal either way. Core and RMW crates
carry no `platform-*` feature to gate on (ARCHITECTURE §2), so the sink has to
be the one that already knows where it is running. It also reaches `no_std`
targets, which every `cfg(feature = "std")` arm silently did not — the pool
diagnostic in `nros-rmw-zenoh` was mute on exactly the firmware where a
fixed-size pool actually fills.

## Exemption

    // nros-allow-std-stdio: <reason>

on the line immediately before. One spelling, because per-site subsets of a rule
are how issue 0442 happened. There is one in the tree: `nros-cpp`'s `cpp_diag!`
std arm, whose `cfg` already excludes `platform-zephyr` and which exists so the
other arms can route Zephyr through `nros_log`.
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

# `std::`-qualified stdio. `::std::` is the same reach with a leading root
# token, which is what a macro EXPANSION spells, so both forms count.
STDIO_RE = re.compile(r"(?<![A-Za-z0-9_])(?:::)?std::(?:e?println|e?print|dbg)!")

ALLOW_RE = re.compile(r"//\s*nros-allow-std-stdio:")

# `#![no_std]`, including the conditional spelling
# `#![cfg_attr(not(feature = "std"), no_std)]`. Matched as "an inner attribute
# naming `no_std`" rather than by parsing the `cfg_attr` predicate: the
# predicate nests parens (`not(feature = "std")`), which a `[^)]*` body silently
# fails to cross — it read that whole family as hosted, which is the half of the
# tree this rule most needs to cover. The self-test carries the case.
NO_STD_RE = re.compile(r"^\s*#!\[[^\n]*\bno_std\b", re.MULTILINE)

PROC_MACRO_RE = re.compile(r"^\s*proc-macro\s*=\s*true", re.MULTILINE)

REPO = Path(__file__).resolve().parent.parent

SKIP_DIRS = {"generated", "target", "build", ".git", "third-party", "node_modules"}


def crate_is_no_std(crate_root: Path):
    """(is_no_std, lib_rs) for a crate directory, or (False, None)."""
    manifest = crate_root / "Cargo.toml"
    if not manifest.is_file():
        return False, None
    text = manifest.read_text(errors="replace")
    if PROC_MACRO_RE.search(text):
        # A proc macro always runs on the host and links libstd by definition.
        # Its EMITTED `::std::println!` lands in the caller's crate and is that
        # crate's business, checked there.
        return False, None
    for name in ("lib.rs", "main.rs"):
        entry = crate_root / "src" / name
        if entry.is_file() and NO_STD_RE.search(entry.read_text(errors="replace")):
            return True, entry
    return False, None


# `git ls-files`, not `rglob`. Every file this gate wants is TRACKED, and the
# roots it is given are the two biggest trees in the repo once built: measured
# here, `examples/` is 828 GB on disk and `Path.rglob("Cargo.toml")` over it had
# not finished after 300 s, while the index answers for both roots in 0.002 s.
#
# SKIP_DIRS did not save it, and that is the whole trap: the filter runs on what
# rglob has ALREADY YIELDED, so the walk still descends every `target/`,
# `build/` and `third-party/` tree to discover the paths it then discards.
# Pruning cannot be done after the fact. Same lesson `scripts/
# check-no-tracked-file-find.sh` records for `find -prune`, and the same one
# `check-image-panic-policy.py` learned for `glob("**")` — that gate is the
# reason this comment can cite it, and this file is the sibling nobody fixed.
#
# SKIP_DIRS is still applied: some skipped dirs ARE tracked (the committed
# `packages/interfaces/*/generated` trees), so the index lists them.
def _tracked_files(roots):
    """Paths under `roots`: the git index for in-repo roots, a walk otherwise.

    The walk is not a silent fallback for the real scan — it exists because
    `--self-test` builds synthetic crates in a temp dir, which is untracked by
    construction and tiny. An in-repo root always takes the index path, so the
    cost above can never come back through here.
    """
    found, walked = [], []
    for root in roots:
        root = Path(root)
        try:
            walked.append(str(root.relative_to(REPO)))
        except ValueError:
            if root.is_dir():
                # walk-ok: out-of-repo root has no index entry; serves --self-test temp trees only
                found.extend(sorted(q for q in root.rglob("*") if q.is_file()))
    if walked:
        out = subprocess.run(
            ["git", "-C", str(REPO), "ls-files", "-z", "--"] + walked,
            capture_output=True, text=True, check=True,
        ).stdout.split("\0")
        found.extend(REPO / r for r in out if r)
    return found


def crate_roots(roots):
    seen = set()
    for path in _tracked_files(roots):
        if path.name != "Cargo.toml":
            continue
        if any(p in SKIP_DIRS for p in path.parts):
            continue
        if path.parent not in seen:
            seen.add(path.parent)
            yield path.parent


def exempted_above(lines, i):
    """Is line `i` covered by an allow marker in the comment block above it?

    Anywhere in the CONTIGUOUS run of comment lines immediately above, not just
    the single preceding line. An exemption worth granting is worth explaining,
    and an explanation runs to several lines — requiring the marker to be the
    last of them would push people to write a one-line reason instead, which is
    the opposite of what this rule wants from an exemption.
    """
    j = i - 1
    while j >= 0:
        stripped = lines[j].strip()
        if not stripped.startswith("//"):
            return False
        if ALLOW_RE.search(lines[j]):
            return True
        j -= 1
    return False


def scan_crate(crate_root):
    """[(path, lineno, line)] of unexempted `std::` stdio under `src/`."""
    hits = []
    src = crate_root / "src"
    if not src.is_dir():
        return hits
    for rs in sorted(_tracked_files([src])):
        if rs.suffix != ".rs":
            continue
        if any(p in SKIP_DIRS for p in rs.parts):
            continue
        lines = rs.read_text(errors="replace").splitlines()
        for i, line in enumerate(lines):
            # Per OCCURRENCE, not per line: two calls on one line are two
            # landmines, and reporting the line once leaves the second one
            # unfixed and unmentioned.
            for m in STDIO_RE.finditer(line):
                # Comments are documentation of the rule — an issue reference,
                # a `///` example, a "was … before 0589" note. Both the
                # whole-line form and a trailing one after real code.
                comment = line.find("//")
                if comment != -1 and m.start() > comment:
                    continue
                if line.lstrip().startswith(("//", "*", "/*")):
                    continue
                if exempted_above(lines, i):
                    continue
                hits.append((rs, i + 1, line.strip()))
    return hits


def check(roots):
    findings = []
    crates = 0
    for crate_root in crate_roots(roots):
        is_no_std, _ = crate_is_no_std(crate_root)
        if not is_no_std:
            continue
        crates += 1
        for path, lineno, line in scan_crate(crate_root):
            findings.append((crate_root, path, lineno, line))
    return crates, findings


SELF_TESTS = [
    # (name, {relpath: content}, expected number of findings)
    (
        "no_std crate with a std::eprintln! is a finding",
        {
            "Cargo.toml": "[package]\nname = 'a'\n",
            "src/lib.rs": '#![no_std]\nfn f() {\n    std::eprintln!("x");\n}\n',
        },
        1,
    ),
    (
        "the `::std::` spelling counts too",
        {
            "Cargo.toml": "[package]\nname = 'a'\n",
            "src/lib.rs": '#![no_std]\nfn f() { ::std::println!("x"); }\n',
        },
        1,
    ),
    (
        "cfg_attr(not(feature = 'std'), no_std) is a no_std crate",
        {
            "Cargo.toml": "[package]\nname = 'a'\n",
            "src/lib.rs": (
                '#![cfg_attr(not(feature = "std"), no_std)]\n'
                'fn f() { std::eprintln!("x"); }\n'
            ),
        },
        1,
    ),
    (
        "a hosted crate may use std stdio freely",
        {
            "Cargo.toml": "[package]\nname = 'a'\n",
            "src/lib.rs": 'fn f() { std::eprintln!("x"); }\n',
        },
        0,
    ),
    (
        "a proc-macro crate is hosted even when it emits ::std::println!",
        {
            "Cargo.toml": "[package]\nname = 'a'\n[lib]\nproc-macro = true\n",
            "src/lib.rs": '#![no_std]\nfn f() { ::std::println!("x"); }\n',
        },
        0,
    ),
    (
        "a bare println! is a crate-local or board macro, not this hazard",
        {
            "Cargo.toml": "[package]\nname = 'a'\n",
            "src/lib.rs": (
                "#![no_std]\n"
                'fn f() { println!("x"); esp_println::println!("y"); }\n'
            ),
        },
        0,
    ),
    (
        "the allow comment exempts the next line",
        {
            "Cargo.toml": "[package]\nname = 'a'\n",
            "src/lib.rs": (
                "#![no_std]\n"
                "// nros-allow-std-stdio: gated away from Zephyr\n"
                'fn f() { std::eprintln!("x"); }\n'
            ),
        },
        0,
    ),
    (
        "the allow comment does NOT exempt the line after next",
        {
            "Cargo.toml": "[package]\nname = 'a'\n",
            "src/lib.rs": (
                "#![no_std]\n"
                "// nros-allow-std-stdio: gated away from Zephyr\n"
                'fn ok() { std::eprintln!("x"); }\n'
                'fn bad() { std::eprintln!("y"); }\n'
            ),
            # only the first is exempt
        },
        1,
    ),
    (
        "the marker may sit anywhere in the comment block above",
        {
            "Cargo.toml": "[package]\nname = 'a'\n",
            "src/lib.rs": (
                "#![no_std]\n"
                "// nros-allow-std-stdio: unreachable on Zephyr\n"
                "// because the arm above is cfg'd away there, and the\n"
                "// Zephyr arm routes through nros_log.\n"
                'fn f() { std::eprintln!("x"); }\n'
            ),
        },
        0,
    ),
    (
        "a blank line ends the comment block, so the marker no longer covers",
        {
            "Cargo.toml": "[package]\nname = 'a'\n",
            "src/lib.rs": (
                "#![no_std]\n"
                "// nros-allow-std-stdio: reason\n"
                "\n"
                'fn f() { std::eprintln!("x"); }\n'
            ),
        },
        1,
    ),
    (
        "a commented-out call is prose, not code",
        {
            "Cargo.toml": "[package]\nname = 'a'\n",
            "src/lib.rs": '#![no_std]\n// was std::eprintln!("x") before 0589\n',
        },
        0,
    ),
    (
        "generated/ under src is not authored code",
        {
            "Cargo.toml": "[package]\nname = 'a'\n",
            "src/lib.rs": "#![no_std]\n",
            "src/generated/g.rs": 'fn f() { std::eprintln!("x"); }\n',
        },
        0,
    ),
    (
        "tests/ is host-run and out of scope",
        {
            "Cargo.toml": "[package]\nname = 'a'\n",
            "src/lib.rs": "#![no_std]\n",
            "tests/t.rs": 'fn f() { std::eprintln!("x"); }\n',
        },
        0,
    ),
    (
        "a std::env call is not stdio",
        {
            "Cargo.toml": "[package]\nname = 'a'\n",
            "src/lib.rs": '#![no_std]\nfn f() { let _ = std::env::var("X"); }\n',
        },
        0,
    ),
    (
        "several findings in one crate all report",
        {
            "Cargo.toml": "[package]\nname = 'a'\n",
            "src/lib.rs": '#![no_std]\nfn f() { std::eprintln!("x"); }\n',
            "src/b.rs": 'fn g() { std::print!("y"); std::dbg!(1); }\n',
        },
        3,
    ),
    (
        "a crate with no src/ is skipped rather than crashing",
        {"Cargo.toml": "[package]\nname = 'a'\n"},
        0,
    ),
]


def self_test():
    import tempfile

    failures = 0
    for name, files, expected in SELF_TESTS:
        with tempfile.TemporaryDirectory() as td:
            crate = Path(td) / "crate"
            for rel, content in files.items():
                p = crate / rel
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text(content)
            _, findings = check([td])
            got = len(findings)
            if got != expected:
                failures += 1
                print(f"  FAIL  {name}: expected {expected}, got {got}")
                for f in findings:
                    print(f"          {f[1]}:{f[2]}: {f[3]}")
            else:
                print(f"  ok    {name}")
    if failures:
        print(f"\ncheck-no-std-stdio --self-test: {failures} case(s) FAILED")
        return 1
    print(f"\ncheck-no-std-stdio --self-test: {len(SELF_TESTS)} case(s) OK")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("roots", nargs="*", default=None)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    repo = Path(__file__).resolve().parent.parent
    roots = args.roots or [str(repo / "packages"), str(repo / "examples")]

    crates, findings = check(roots)
    if not findings:
        print(f"check-no-std-stdio: OK ({crates} no_std crate(s), no std-qualified stdio)")
        return 0

    print("check-no-std-stdio: std-qualified stdio in a no_std crate:", file=sys.stderr)
    for crate_root, path, lineno, line in findings:
        try:
            shown = path.relative_to(repo)
        except ValueError:
            shown = path
        print(f"  {shown}:{lineno}: {line}", file=sys.stderr)
    print(
        "\n"
        "  Issue 0589 — on Zephyr native_sim these do not print, they SIGSEGV the\n"
        "  image: `zvfs_write(1, …)` re-enters itself through `stdinout_write_vmeth`\n"
        "  and `k_mutex` is recursive, so the stack runs out. A diagnostic that can\n"
        "  kill the image it is diagnosing is worse than no diagnostic, and it is\n"
        "  latent in EVERY native_sim image.\n"
        "\n"
        "  Write `nros_log` instead:\n"
        "\n"
        "      nros_log::log_error!(nros_log::get_logger(\"<crate>\"), \"…\");\n"
        "\n"
        "  It reaches the console when a sink is wired, is dropped when one is not,\n"
        "  and is never fatal. It also reaches no_std targets, which the\n"
        "  `#[cfg(feature = \"std\")]` arm around these calls never did.\n"
        "\n"
        "  If a site is genuinely unreachable on Zephyr, say why on the line before:\n"
        "\n"
        "      // nros-allow-std-stdio: <reason>\n",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
