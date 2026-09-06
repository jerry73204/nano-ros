#!/usr/bin/env python3
"""No build file may declare a C++ standard below the one nros-cpp requires.

Issue 1118. The tree declared C++14 in 95 places while `component_node.hpp`
used `if constexpr` — a C++17 construct — at three sites (four with
`NROS_CPP_STD`). GCC only WARNS on that, so nothing failed, and the declared
minimum was a claim the code violated for as long as the claim existed.

WHY A GATE AND NOT A REVIEW NOTE

The number was restated in four vocabularies:

    set(CMAKE_CXX_STANDARD 14)            75 CMakeLists
    target_compile_features(... cxx_std_14)  2 cmake files
    CONFIG_STD_CPP14=y                    18 Zephyr prj.conf
    -std=c++14 in CMAKE_CXX_FLAGS_INIT     6 toolchain files

Four vocabularies for one fact is four chances to disagree, and they already
did: `arm-freertos-armcr52.cmake` said `-std=c++17` while its four sibling
toolchains said 14, and `realtime-cpp/src/fvp_entry/prj.conf` said
`CONFIG_STD_CPP17` while its own workspace's `zephyr_entry` said 14. Nobody
noticed either, because a standard that is too LOW only warns. That is the
shape a gate is for; a reviewer reading one diff cannot see the other 94 lines.

WHAT IT CHECKS, AND WHAT IT DELIBERATELY DOES NOT

Covered: the three vocabularies that DECLARE a standard as a standard —
`CMAKE_CXX_STANDARD`, the `cxx_std_N` compile feature, and Zephyr's
`CONFIG_STD_CPPN`. Each must name a standard at or above the floor, OR carry
`# nros-cxx-floor-exempt: <reason>` on itself or the line above.

The escape exists because the floor is nros-cpp's REQUIREMENT, so it binds
nros-cpp's consumers. Two things here declare a standard and consume nothing
from nros-cpp: `nros-rmw-uorb`, compiled inside PX4 (`-std=gnu++14 -Werror`),
and `nros-rmw-cyclonedds`, which the Zephyr lane builds at `-std=c++11`.
Raising those would assert a dialect the consuming build does not provide.
The reason is required, must be more than a word, and the exempt sites are
PRINTED on the green path — "none declares below C++17" would be false while
two do, and an exception nobody is shown is one nobody revisits. Its one known
limit: an already-exempt site can be lowered FURTHER and the gate reports the
new number rather than failing on it. That is the price of a prose reason;
the number is on screen, in the diff, every run.

Not covered: a raw `-std=c++N` in a toolchain file's `CMAKE_CXX_FLAGS_INIT`.
That is a lane DEFAULT, not a declared minimum, and measured it does not decide
anything for a target that sets its own standard:

    target_compile_options(zephyr_like INTERFACE -std=c++14)   # raw
    set(CMAKE_CXX_STANDARD 17) + CXX_STANDARD_REQUIRED ON
    => FLAGS = -std=c++14 -std=gnu++17          # CMake's flag is LAST, and wins

The six raw-flag sites are enumerated in `docs/issues/1118-*.md` as stale lane
defaults. They are a legibility defect, not a correctness one, and they live in
files this change does not own.

Also not covered: the `-std=c++14` compile PROBES in `just check cpp` and
`scripts/build/compile-check-fixtures.sh`. Those are not declarations either —
they assert a STRICTER property that survives the floor moving: every nros-cpp
header outside the `NROS_SYSTEM_PARAM_SERVICES` block is still written in the
freestanding C++14 subset, and `just check cpp` still proves it. A gate that
rewrote those would delete a real portability property in the name of tidiness.

WHY THE FLOOR IS READ AND NOT WRITTEN HERE

This gate contains no version number. It reads the floor out of the one line
that makes the requirement true —

    target_compile_features(nros-cpp-headers INTERFACE cxx_std_NN)

in `packages/api/nros-cpp/CMakeLists.txt` — so raising the floor is editing that
line, and this file follows. A gate that restated the number would be the 96th
restatement of the very thing it exists to stop.

Usage::

    check-cxx-standard-floor.py            # the gate (self-test always runs)
    check-cxx-standard-floor.py --self-test
    check-cxx-standard-floor.py --root DIR # scan a different tree
"""

from __future__ import annotations

import re
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]

# The one line that carries the floor. Kept as a pair (path, pattern) because
# both halves have to move together if the target is ever renamed.
FLOOR_FILE = Path("packages/api/nros-cpp/CMakeLists.txt")
FLOOR_RE = re.compile(
    r"target_compile_features\(\s*nros-cpp-headers\s+INTERFACE\s+cxx_std_(\d+)\s*\)"
)

# R1 — the declaration a leaf package writes. `CXX_STANDARD_REQUIRED ON` beside
# it is what makes it beat a toolchain's raw `-std=`; this rule is about the
# NUMBER, so it does not look at that.
R_CMAKE_STANDARD = re.compile(r"\bset\s*\(\s*CMAKE_CXX_STANDARD\s+(\d+)\s*\)")
# R2 — the requirement a library propagates to its consumers.
R_COMPILE_FEATURE = re.compile(r"\bcxx_std_(\d+)\b")
# R3 — Zephyr's, which reaches the compiler through Kconfig and not through
# CMake at all, so nothing in R1/R2 can see it.
R_KCONFIG = re.compile(r"^\s*CONFIG_STD_CPP(\d+)\s*=\s*y\s*$", re.M)

RULES = (
    ("CMAKE_CXX_STANDARD", R_CMAKE_STANDARD),
    ("cxx_std_N", R_COMPILE_FEATURE),
    ("CONFIG_STD_CPPN", R_KCONFIG),
)

# The escape, and why it is INLINE rather than a side file.
#
# The floor is nros-cpp's requirement, so it binds nros-cpp's CONSUMERS. Two
# things in this tree declare a standard and consume nothing from nros-cpp: the
# uORB backend, which is compiled inside PX4 (`-std=gnu++14 -Werror`), and the
# Cyclone wrapper, which the Zephyr lane builds at `-std=c++11`. For those,
# "raise it to 17" would assert a dialect the consuming build does not provide.
#
# A `.config/` allowlist would put the reason in a file nobody opens while
# editing the line. This marker sits ON the declaration, so the next person to
# consider raising it reads the reason first — and an exemption with no reason
# is refused, which is what keeps it from becoming a rubber stamp.
EXEMPT = re.compile(r"nros-cxx-floor-exempt:\s*(\S.*?)\s*$")
MIN_REASON_WORDS = 5

CMAKE_NAMES = ("CMakeLists.txt",)
CMAKE_SUFFIXES = (".cmake",)
CONF_SUFFIXES = (".conf",)

# Trees whose C++ standard is not ours to declare, plus build output. A
# generated/`build` tree is a COPY of a source declaration: flagging it reports
# the same defect twice and goes red on residue from a checkout that has since
# been fixed.
SKIP_PARTS = (
    "third-party",
    "zephyr-workspace",
    "node_modules",
    "CMakeFiles",
    ".git",
    "target",
    "tmp",
)
SKIP_DIR_NAMES = ("build", "generated", "install", "log")


def _skip(rel: Path) -> bool:
    parts = rel.parts
    if any(p in SKIP_PARTS for p in parts):
        return True
    # A `build`/`generated` DIRECTORY anywhere on the path — but not a file
    # named e.g. `build.cmake`, which is source.
    return any(p in SKIP_DIR_NAMES for p in parts[:-1])


# A `#` outside quotes starts a CMake comment. Stripping matters here more than
# it usually would: the floor's OWN declaration site explains the old value in
# prose, so a gate that reads comments reports `cxx_std_14` in the very file
# that raised it — which is how a gate teaches people to delete the
# explanation instead of the defect.
def strip_cmake_comments(text: str) -> str:
    out = []
    for line in text.split("\n"):
        quoted = False
        cut = len(line)
        for i, ch in enumerate(line):
            if ch == '"':
                quoted = not quoted
            elif ch == "#" and not quoted:
                cut = i
                break
        out.append(line[:cut])
    return "\n".join(out)


def tracked(root: Path) -> set[str] | None:
    """Paths git tracks, or None when `root` is not a checkout.

    Build output and scratch dirs are gitignored, so "is it tracked?" answers
    "is it a declaration someone wrote?" exactly, where the name-based skip
    list above only approximates it. The list stays as the fallback for the
    self-test's temp tree, which is not a repo.
    """
    import subprocess

    try:
        out = subprocess.run(
            ["git", "-C", str(root), "ls-files", "-z"],
            capture_output=True,
            check=True,
        ).stdout
    except (OSError, subprocess.CalledProcessError):
        return None
    names = {n.decode("utf8", "replace") for n in out.split(b"\0") if n}
    return names or None


def read_floor(root: Path, errors: list[str]) -> int | None:
    """The declared floor, or None with an error appended.

    Refusing to guess is the point: a gate that fell back to a literal when the
    line moved would keep passing while checking a number nobody declares.
    """
    path = root / FLOOR_FILE
    if not path.is_file():
        errors.append(
            f"{FLOOR_FILE} does not exist — the floor has no home, so there is "
            f"nothing to check against. Do not add a literal here; move the "
            f"declaration and update FLOOR_FILE."
        )
        return None
    m = FLOOR_RE.search(path.read_text(encoding="utf8", errors="replace"))
    if not m:
        errors.append(
            f"{FLOOR_FILE} no longer contains "
            f"`target_compile_features(nros-cpp-headers INTERFACE cxx_std_NN)`.\n"
            f"      That line IS the C++ floor (issue 1118). If it was renamed, "
            f"point FLOOR_RE at the new spelling; if it was deleted, the floor "
            f"is undeclared and every consumer silently takes the compiler "
            f"default."
        )
        return None
    return int(m.group(1))


def candidates(root: Path) -> list[Path]:
    """Every build file that could declare a standard, relative to `root`."""
    keep = tracked(root)
    # Enumerating the tracked set is also what makes this cheap: `rglob("*")`
    # over a tree with warm build dirs walks ~10^6 paths to reject nearly all
    # of them, and measured that is 4 s against 0.1 s here.
    walk = (
        (Path(n) for n in keep)
        if keep is not None
        else (p.relative_to(root) for p in root.rglob("*") if p.is_file())
    )
    out = []
    for rel in walk:
        if _skip(rel):
            continue
        if (
            rel.name in CMAKE_NAMES
            or rel.suffix in CMAKE_SUFFIXES
            or rel.suffix in CONF_SUFFIXES
        ) and (root / rel).is_file():
            out.append(rel)
    return sorted(out)


def _exemption(raw_lines: list[str], lineno: int) -> str | None:
    """The reason on the declaration's own line or the one above it, if any."""
    for probe in (lineno - 1, lineno - 2):
        if 0 <= probe < len(raw_lines):
            m = EXEMPT.search(raw_lines[probe])
            if m:
                return m.group(1)
    return None


def scan(
    root: Path, floor: int, exempted: list | None = None
) -> list[tuple[str, int, str, int]]:
    """(relpath, lineno, rule, declared) for every declaration BELOW `floor`.

    A below-floor declaration carrying a `nros-cxx-floor-exempt: <reason>` on
    itself or the line above is not reported — but an exemption whose reason is
    missing or perfunctory IS, as its own error, because a bare marker is how a
    considered exception decays into a habit.
    """
    hits = []
    if exempted is None:
        exempted = []
    for rel in candidates(root):
        text = (root / rel).read_text(encoding="utf8", errors="replace")
        raw_lines = text.split("\n")
        is_conf = rel.suffix in CONF_SUFFIXES
        if not is_conf:
            text = strip_cmake_comments(text)
        for rule, pattern in RULES:
            # A `.conf` carries only Kconfig; a CMake file carries only the
            # other two. Crossing them would let a `.cmake` containing the
            # literal text `CONFIG_STD_CPP14` in a COMMENT read as a
            # declaration.
            if is_conf != (rule == "CONFIG_STD_CPPN"):
                continue
            for m in pattern.finditer(text):
                declared = int(m.group(1))
                if declared >= floor:
                    continue
                lineno = text.count("\n", 0, m.start()) + 1
                reason = _exemption(raw_lines, lineno)
                if reason is None:
                    hits.append((str(rel), lineno, rule, declared))
                elif len(reason.split()) < MIN_REASON_WORDS:
                    hits.append((str(rel), lineno, "exempt-without-a-reason", declared))
                else:
                    exempted.append((str(rel), lineno, declared, reason))
    return sorted(hits)


def exemptions(root: Path, floor: int) -> list[tuple[str, int, int, str]]:
    """The below-floor declarations that carry a reason. Reported, never silent.

    Printing them on the OK path is the point: "none declares below C++17" would
    be false while two do, and an exception nobody is shown is an exception
    nobody revisits.
    """
    out: list[tuple[str, int, int, str]] = []
    scan(root, floor, out)
    return sorted(out)


def _write(root: Path, rel: str, text: str) -> None:
    p = root / rel
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(text, encoding="utf8")


def self_test(quiet: bool = False) -> int:
    """Fire every rule, then silence each one the way it is meant to be silenced.

    Always, not behind a flag: a negative control nobody runs decays into a
    comment, and the thing this gate is guarding against is precisely a check
    that looks satisfied because it never looked.
    """
    problems: list[str] = []

    def want(cond: bool, msg: str) -> None:
        if not cond:
            problems.append(msg)

    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        _write(
            root,
            str(FLOOR_FILE),
            "add_library(nros-cpp-headers INTERFACE)\n"
            "target_compile_features(nros-cpp-headers INTERFACE cxx_std_17)\n",
        )
        errors: list[str] = []
        floor = read_floor(root, errors)
        want(floor == 17, f"floor parsed as {floor!r}, expected 17 ({errors})")
        assert floor is not None

        # --- each rule fires -------------------------------------------------
        _write(root, "leaf/CMakeLists.txt", "set(CMAKE_CXX_STANDARD 14)\n")
        _write(root, "compat/x.cmake", "target_compile_features(a INTERFACE cxx_std_11)\n")
        _write(root, "app/prj.conf", "CONFIG_CPP=y\nCONFIG_STD_CPP14=y\n")

        fired = {(h[0], h[2]) for h in scan(root, floor)}
        want(
            ("leaf/CMakeLists.txt", "CMAKE_CXX_STANDARD") in fired,
            "R1 did not fire on `set(CMAKE_CXX_STANDARD 14)`",
        )
        want(
            ("compat/x.cmake", "cxx_std_N") in fired,
            "R2 did not fire on `cxx_std_11`",
        )
        want(
            ("app/prj.conf", "CONFIG_STD_CPPN") in fired,
            "R3 did not fire on `CONFIG_STD_CPP14=y`",
        )

        # --- SHAPE-VALID, WIRES CROSSED --------------------------------------
        # The mutation that a rule-per-vocabulary check is most likely to miss:
        # every line is a well-formed declaration and every number is legal,
        # but each vocabulary carries the number that belongs to another file.
        # A gate keyed on "does a low number appear anywhere in the tree" passes
        # this; a gate that attributes each number to its own site does not.
        _write(root, "leaf/CMakeLists.txt", "set(CMAKE_CXX_STANDARD 17)\n")
        _write(root, "compat/x.cmake", "target_compile_features(a INTERFACE cxx_std_17)\n")
        _write(
            root,
            "app/prj.conf",
            # Kconfig spelling, legal symbol, below the floor — and the CMake
            # files above are now clean, so the tree-wide "is 14 mentioned"
            # question has the same answer as a fully-fixed tree.
            "CONFIG_CPP=y\nCONFIG_STD_CPP14=y\n",
        )
        crossed = scan(root, floor)
        want(
            [(h[0], h[2], h[3]) for h in crossed] == [("app/prj.conf", "CONFIG_STD_CPPN", 14)],
            f"crossed-wires mutation: expected exactly the prj.conf hit, got {crossed}",
        )

        # A `.cmake` mentioning the Kconfig symbol in prose is NOT a
        # declaration — the vocabulary is bound to the file kind. And a CMake
        # COMMENT is never a declaration in any vocabulary: the floor's own
        # site explains the value it replaced, so a comment-blind gate goes red
        # on the fix.
        _write(
            root,
            "compat/x.cmake",
            "# lowering (e.g. CONFIG_STD_CPP14 for the entry)\n"
            "# it read `cxx_std_14`, and set(CMAKE_CXX_STANDARD 14) with it\n"
            "target_compile_features(a INTERFACE cxx_std_17)\n",
        )
        want(
            not [h for h in scan(root, floor) if h[0] == "compat/x.cmake"],
            "a standard quoted in a CMake COMMENT was read as a declaration",
        )
        # ...but a declaration on the SAME LINE as a trailing comment still is
        # one. Stripping must not swallow code.
        _write(
            root,
            "compat/x.cmake",
            "set(CMAKE_CXX_STANDARD 14)  # why it is low\n",
        )
        want(
            [(h[2], h[3]) for h in scan(root, floor) if h[0] == "compat/x.cmake"]
            == [("CMAKE_CXX_STANDARD", 14)],
            "comment stripping swallowed a declaration that precedes a trailing `#`",
        )
        _write(root, "compat/x.cmake", "target_compile_features(a INTERFACE cxx_std_17)\n")

        # --- each escape silences its own rule -------------------------------
        _write(root, "app/prj.conf", "CONFIG_CPP=y\nCONFIG_STD_CPP17=y\n")
        want(not scan(root, floor), f"a fully-raised tree still reports {scan(root, floor)}")

        # A standard ABOVE the floor is allowed: the rule is a floor, not an
        # equality. Without this the gate would forbid the very C++20 move the
        # issue says needs its own survey.
        _write(root, "leaf/CMakeLists.txt", "set(CMAKE_CXX_STANDARD 20)\n")
        want(not scan(root, floor), "a standard ABOVE the floor was rejected")

        # Build output is a COPY of a source declaration; flagging it would go
        # red on residue from a tree that has since been fixed.
        _write(root, "leaf/build/x/CMakeLists.txt", "set(CMAKE_CXX_STANDARD 14)\n")
        _write(root, "third-party/dep/CMakeLists.txt", "set(CMAKE_CXX_STANDARD 11)\n")
        want(not scan(root, floor), f"build/third-party output was scanned: {scan(root, floor)}")

        # --- the exemption, and the shape that must NOT silence a rule -------
        _write(
            root,
            "backend/CMakeLists.txt",
            "# nros-cxx-floor-exempt: built inside PX4 at -std=gnu++14 -Werror\n"
            "set(CMAKE_CXX_STANDARD 14)\n",
        )
        want(
            not [h for h in scan(root, floor) if h[0] == "backend/CMakeLists.txt"],
            "a reasoned `nros-cxx-floor-exempt` did not silence its declaration",
        )
        # A marker with no reason must FAIL, and fail as its own diagnosis —
        # otherwise the escape is just "type the magic word".
        _write(
            root,
            "backend/CMakeLists.txt",
            "# nros-cxx-floor-exempt: legacy\nset(CMAKE_CXX_STANDARD 14)\n",
        )
        want(
            [h[2] for h in scan(root, floor) if h[0] == "backend/CMakeLists.txt"]
            == ["exempt-without-a-reason"],
            "a bare `nros-cxx-floor-exempt: legacy` was accepted as a reason",
        )
        # An exemption reaches ONLY the line it sits above — two lines up is
        # already someone else's declaration.
        _write(
            root,
            "backend/CMakeLists.txt",
            "# nros-cxx-floor-exempt: built inside PX4 at -std=gnu++14 -Werror\n"
            "set(CMAKE_CXX_STANDARD 14)\n"
            "add_library(x INTERFACE)\n"
            "target_compile_features(x INTERFACE cxx_std_14)\n",
        )
        want(
            [h[1] for h in scan(root, floor) if h[0] == "backend/CMakeLists.txt"] == [4],
            "an exemption leaked past its own declaration to a later one",
        )
        (root / "backend/CMakeLists.txt").unlink()

        # And the floor itself must be unreadable rather than guessed.
        _write(root, str(FLOOR_FILE), "add_library(nros-cpp-headers INTERFACE)\n")
        errs2: list[str] = []
        want(
            read_floor(root, errs2) is None and errs2,
            "a MISSING floor declaration did not raise — the gate would then "
            "check against a number nobody declares",
        )

    if problems:
        print("[FAIL] check-cxx-standard-floor self-test:", file=sys.stderr)
        for p in problems:
            print(f"  - {p}", file=sys.stderr)
        return 1
    if not quiet:
        print("self-test OK — every rule fires and every escape silences it.")
    return 0


def main() -> int:
    argv = sys.argv[1:]
    if "--self-test" in argv:
        return self_test()
    # Always. See the docstring on `self_test`.
    rc = self_test(quiet=True)
    if rc:
        return rc

    root = REPO
    if "--root" in argv:
        root = Path(argv[argv.index("--root") + 1]).resolve()

    errors: list[str] = []
    floor = read_floor(root, errors)
    if floor is None:
        print("[FAIL] check-cxx-standard-floor:", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        return 1

    exempt: list[tuple[str, int, int, str]] = []
    hits = scan(root, floor, exempt)
    files = candidates(root)
    if hits:
        print(
            f"\n[FAIL] {len(hits)} C++ standard declaration(s) below the "
            f"floor of C++{floor} (issue 1118):",
            file=sys.stderr,
        )
        for rel, lineno, rule, declared in hits:
            print(f"  - {rel}:{lineno}: {rule} declares {declared}", file=sys.stderr)
        print(
            f"\n  The floor is `cxx_std_{floor}` in {FLOOR_FILE}. nros-cpp's\n"
            f"  `component_node.hpp` uses `if constexpr`, which is C++17, and GCC\n"
            f"  only WARNS when the declared standard is lower — so a declaration\n"
            f"  below the floor fails nothing and is wrong anyway.\n"
            f"  Raise the site, or raise the floor and re-run.\n"
            f"\n  A site that genuinely must stay lower — it consumes no nros-cpp\n"
            f"  header and its own build imposes an older dialect — carries\n"
            f"  `# nros-cxx-floor-exempt: <reason>` on the line above. The reason\n"
            f"  is required and must say WHICH build imposes it; a bare marker is\n"
            f"  reported as `exempt-without-a-reason`.",
            file=sys.stderr,
        )
        return 1

    print(
        f"check-cxx-standard-floor OK — {len(files)} build file(s) scanned, "
        f"none declares below C++{floor} unexplained "
        f"(floor read from {FLOOR_FILE})."
    )
    # Printed on the GREEN path on purpose: "none declares below C++17" would be
    # false while these do, and an exception nobody is shown is one nobody
    # revisits when the reason expires.
    for rel, lineno, declared, reason in sorted(exempt):
        print(f"  exempt: {rel}:{lineno} declares {declared} — {reason}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
