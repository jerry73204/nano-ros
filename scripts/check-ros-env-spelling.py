#!/usr/bin/env python3
"""issue 0763 — a test's ROS 2 environment is spelled in ONE place, not per site.

Everything that talks to a real ROS 2 install needs the same four decisions made
the same way: which distro to source, which RMW to export, which domain to join,
and which zenoh router to point at. This repo already has that one place —
`packages/testing/nros-tests/src/ros_env.rs` (the `RosEnv` trait + the
`Middleware` enum, RFC-0058 / phase-309), implemented over the
`ros2::ros2_env_setup_*` helpers in `packages/testing/nros-tests/src/ros2.rs`.
A hand-rolled `source /opt/ros/<distro>/setup.bash && export ...` anywhere else
is a second place, and every second place this repo has had was wrong in a way
nobody could see from the site itself:

  * `HostRosEnv::env_snippet` destructured `Middleware::Zenoh { locator, .. }`
    and threw the peer's `domain_id` away, while the docker backend exported it.
    One `Middleware` value, two different environments. The domain is the FIRST
    segment of an rmw_zenoh keyexpr, so peers that disagree about it never
    discover each other at all — and ros2cli keys its daemon on ROS_DOMAIN_ID
    alone, so every test that forgot the domain shared domain 0's singleton
    daemon with every other one.
  * Every hand-rolled setup used to open with `ros2 daemon stop`. Under a
    parallel suite that is not a reset, it is a cross-test kill: it stopped the
    daemon another test was mid-query against.
  * Availability probes hardcoded `humble`. On a jazzy host they probe a prefix
    that does not exist, so the guarded tests SKIP — silently, and forever, on
    exactly the hosts the guard was written to protect.

None of those are visible while reading the offending line; each surfaced as a
flake or as a coverage hole found much later. So the point of this gate is not
that a hand-rolled environment is always wrong — it is that adding one must be a
DELIBERATE, REVIEWABLE act (an allowlist entry with a reason in the diff) rather
than something that arrives by copy-paste and is discovered a year later.

What it checks
--------------
Tracked source files are scanned for the hand-rolled spellings (`source
/opt/ros/`, `export RMW_IMPLEMENTATION=`). Three things are NOT a bypass and are
skipped structurally rather than by allowlist:

  1. The two sanctioned implementation files. Those ARE the procedure.
  2. Comments and doc comments. `//`, `///`, `//!`, `#`, block-comment `*`
     continuations, and python `\"\"\"` docstrings — a doc comment that shows a
     reader the command is documentation, not a second implementation. (`#` is
     deliberately NOT a comment marker in Rust: `r#"source /opt/ros/..."#` is a
     raw string holding a real script, and treating its `#` as a comment would
     blind the gate to the one shape most likely to be a bypass.)
  3. Placeholder spellings — `source /opt/ros/<distro>/setup.bash`. A literal
     `<distro>` cannot execute; it is prose addressed to a human, whether it
     sits in a comment, in an `eprintln!` hint or in a `--help` string. This is
     rule 2's reasoning applied to the one case that is provably not code.

Everything else that is legitimate is named in ALLOWLIST below, path-keyed, one
reason each. Deliberately not a pattern: a glob like `scripts/*` would silently
cover the next script somebody adds, which is the failure mode this gate exists
to prevent.

Scope
-----
Executable text only (see SCANNED_SUFFIXES). Markdown, TOML and JSON are not
scanned: nothing runs from them, so a match there is prose or configuration
text — rule 2 at file granularity. Build output and vendored trees are excluded
twice over (`git ls-files` plus an explicit path filter), because a tracked
`build-*/` inside an example leaf would otherwise report matches nobody wrote.

Dependency-free Python 3.10, house style per `scripts/check-board-tiers.py`:
no third-party libs and no `tomllib`, because CI hosts are not guaranteed either.
"""

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# The two files that ARE the sanctioned procedure. Everything else delegates.
SANCTIONED = {
    "packages/testing/nros-tests/src/ros2.rs",
    "packages/testing/nros-tests/src/ros_env.rs",
    # This file. Its self-test fixtures are deliberate SPECIMENS of the thing it
    # forbids — a rule that cannot state what it rejects cannot be tested.
    #
    # It is here because the gate went red the moment it was committed and not
    # one run before: `git ls-files` does not list an untracked file, so every
    # verification of it — the author's and the reviewer's — scanned a tree that
    # did not contain it. A gate whose first honest run happens after the commit
    # is a gate nobody has actually run.
    "scripts/check-ros-env-spelling.py",
}

# Legitimate hand-rolled sites, path-keyed, one reason each.
#
# Growing this is the reviewable act the gate exists to force: a new entry is a
# line in a diff that says, in prose, why this site cannot go through `RosEnv`.
# Two reasons recur and both are structural — the site is not Rust and so cannot
# call a Rust trait, or the site is outside the `nros-tests` dependency graph.
# Directory prefixes that carry an exemption, for the same reason as the
# path-keyed entries below but WITHOUT breaking every time the files move.
# Measured: they have moved twice — `justfile` -> `just/check.just`
# (phase-399), then `just/check.just` -> `just/check/*.just` (the topic split)
# — and each move silently dropped the exemption, turning a recipe that never
# changed into a finding.
ALLOWED_PREFIXES = {
    "just/check/":
        "the gate recipe layer, split into topic files. Same exemption as "
        "`just/check.just` below, keyed on the directory so the next move of "
        "these recipes does not drop it again",
}


def _allowed_prefix(path):
    return any(path.startswith(p) for p in ALLOWED_PREFIXES)


ALLOWLIST = {
    # --- CI job bootstrap: the OUTER environment the harness runs INSIDE ------
    # These source ROS before invoking `just`, so the Rust harness has an ament
    # prefix to find at all. They cannot delegate to `RosEnv` — `RosEnv` has not
    # been compiled yet. The distro literal is the runner image's own pin.
    ".github/workflows/host-tests.yml":
        "CI job bootstrap: sources ROS for the runner image before `just` runs; "
        "distro is the image pin, not a test's choice",
    ".github/workflows/nightly.yml":
        "CI job bootstrap, as host-tests.yml",
    ".github/workflows/gate.yml":
        "CI job bootstrap, as host-tests.yml",

    # --- developer environment SSoT ------------------------------------------
    "activate.fish":
        "activate.* IS the env/PATH SSoT this repo's contract points at; it is "
        "what makes a ROS env exist for everything downstream",

    # --- recipe layer: shell, so `RosEnv` is not reachable --------------------
    "justfile":
        "recipe-layer shell sourcing ROS before a build that needs .msg defs; "
        "not a test env",
    "just/check.just":
        "same recipe layer as `justfile` — phase-399 moved the 200 gate recipes "
        "there with `import`, which is a namespace MERGE, so `check-dep-chain` "
        "is the same recipe at a new path. This allowlist keys on PATH, so the "
        "move dropped it out of an exemption it still earns; that is the entry",
    "just/ros-editions.just":
        "the docker ROS-edition axis (issue 0327): the distro IS the variable "
        "under test, supplied per-edition, not a hardcode",

    # --- separate cargo workspace: cannot depend on nros-tests ----------------
    "packages/cli/rosidl-bindgen/tests/edition_hash_oracle.rs":
        "packages/cli is its own workspace with no nros-tests dep; the script "
        "runs inside a pinned docker image whose distro is the fixture",
    "packages/cli/testing_workspaces/complex_workspace/justfile":
        "sample USER workspace — demonstrates the documented user procedure",
    "packages/cli/testing_workspaces/my_robot_node/justfile":
        "sample USER workspace — demonstrates the documented user procedure",
    "tests/simple-workspace/justfile":
        "sample USER workspace — demonstrates the documented user procedure",
    "tests/simple-workspace/.envrc":
        "sample USER workspace — demonstrates the documented user procedure",

    # --- shell test suites: not Rust ------------------------------------------
    # These are the closest thing to a real bypass on the list. They are shell,
    # so the remedy the gate names is unreachable from them; what keeps them
    # honest is that each exports exactly one RMW and is invoked by a lane that
    # names that RMW. Porting them into a cell (RFC-0051) retires both entries.
    "packages/rmw/cyclonedds/nros-rmw-cyclonedds/tests/ros2_pubsub_e2e.sh":
        "shell e2e for the cyclone lane; exports the one RMW the lane names",
    "packages/rmw/cyclonedds/nros-rmw-cyclonedds/tests/ros2_srv_e2e.sh":
        "shell e2e for the cyclone lane; exports the one RMW the lane names",
    "scripts/test/isotp-ros-interop.sh":
        "RFC-0083 / phase-394 W6 manual harness: shell, so `RosEnv` is "
        "unreachable, and it is a hand-run bring-up (needs vcan0, the "
        "can-isotp module and a purpose-built isotp libzenohc.so), not a lane. "
        "It exports one RMW, rmw_zenoh_cpp, which is the only one that can "
        "load the ISO-TP libzenohc.so the test is about. Distro is "
        "`${ROS_DISTRO:-humble}`, a fallback rather than a hardcode, and the "
        "sourcing step `die`s if it does not yield `ros2`. KNOWN GAP: it pins "
        "no ROS_DOMAIN_ID, so it runs on domain 0 — the second hazard this "
        "gate names. It stops the daemon to blunt that, but a developer with "
        "their own graph up on domain 0 will still see the peers. Left as-is "
        "deliberately: the nano-ros side takes its domain from its own baked "
        "config, so moving only the ROS side would break a harness that cannot "
        "be re-run without vcan0 and a custom libzenohc. Porting it to a cell "
        "(RFC-0051) retires this entry and the gap together",

    # --- container runtime: the IMAGE pins the distro -------------------------
    "docker/can-demo/entrypoint.sh":
        "runs INSIDE the can-demo container, whose Dockerfile is `FROM "
        "ros:humble-ros-base` — the distro literal is the image's own pin, not "
        "a test's choice, exactly as for the CI job bootstraps above. It also "
        "exports the one RMW the demo is about (rmw_zenoh_cpp), and `RosEnv` is "
        "unreachable: this is the container's PID 1, with no cargo in the image",

    # --- build script whose match is PRINTED ADVICE, not execution ------------
    "scripts/can/build-zenohc-can.sh":
        "hand-run build script; the match is inside a `cat <<EOF` telling the "
        "operator what to source afterwards, so nothing is sourced by the "
        "script itself. Unlike scripts/debug/capture-ros2-keyexpr.sh below, the "
        "advice does NOT hardcode a distro — it prints "
        "`${ROS_DISTRO:-humble}`, so it stays correct on a jazzy host. Kept as "
        "an entry rather than exempting heredocs structurally, for the same "
        "reason echo is not exempted: a bypass hidden in a heredoc would be "
        "invisible",

    # --- hand-run interop scripts: shell, so `RosEnv` is unreachable ----------
    "scripts/test/isotp-ros-params.sh":
        "hand-run ISO-TP/CAN interop script; shell, so the RosEnv remedy is not "
        "reachable from it. It sources ROS to get `ros2` and the stock "
        "rmw_zenoh router config on PATH, and exports the one RMW the script is "
        "about. The distro is NOT hardcoded — it reads `${ROS_DISTRO:-humble}`, "
        "so it stays correct on a jazzy host",

    # --- one-off developer / provisioning scripts, not test machinery ---------
    "scripts/debug/capture-ros2-keyexpr.sh":
        "hand-run debug script; the match is an `echo` telling the operator "
        "what to source. NOTE it hardcodes `humble` in that advice, which is "
        "wrong on a jazzy host — a real (if minor) instance of the third defect "
        "above. Kept as an entry rather than exempting `echo` structurally: a "
        "bypass hidden inside an echo would then be invisible",
    "scripts/debug/check-ros2-hash.sh":
        "hand-run debug script; probes both distros explicitly, by design",
    "scripts/debug/compare-keyexprs.sh":
        "hand-run debug script comparing a real ros2 peer's keyexprs",
    "scripts/dev/ros2-distrobox-setup.sh":
        "provisions the Ubuntu distrobox itself — it CREATES the ROS env",
    "scripts/probe/verify-zenoh-interop.sh":
        "book-flow probe: reproduces what the BOOK tells a user to type, so it "
        "must not go through the harness",
    "scripts/ros/capture-edition-fixtures.sh":
        "captures per-edition fixtures; distro is the axis, passed in",
    "scripts/ros/domain-bridge-repro.sh":
        "standalone repro script; distro passed in",
}

# Executable text only. A `.md`/`.toml`/`.json` match is prose or config data.
SCANNED_SUFFIXES = {
    ".rs", ".py", ".sh", ".bash", ".zsh", ".fish", ".just",
    ".yml", ".yaml", ".cmake", ".c", ".cc", ".cpp", ".cxx",
    ".h", ".hh", ".hpp",
}
SCANNED_NAMES = {"justfile", ".envrc", "Justfile"}

# Excluded twice over: `git ls-files` already hides untracked build output, but a
# tracked `build-*/` inside an example leaf would still reach us.
EXCLUDED_SEGMENTS = {"third-party", "build", "generated", ".git", "node_modules"}
EXCLUDED_PREFIXES = ("target", "build-")

# `//`-comment languages; `#` is NOT a comment in these (Rust raw strings).
SLASH_SUFFIXES = {".rs", ".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp"}
# `#`-comment languages.
HASH_SUFFIXES = {".py", ".sh", ".bash", ".zsh", ".fish", ".just", ".yml",
                 ".yaml", ".cmake"}

SPELLINGS = (
    ("source /opt/ros/", re.compile(r"(?:source|\.)\s+/opt/ros/(?P<tail>\S*)")),
    ("export RMW_IMPLEMENTATION=",
     re.compile(r"export\s+RMW_IMPLEMENTATION\s*=\s*(?P<tail>\S*)")),
)

REMEDY = (
    "Use `nros_tests::ros_env::RosEnv` + `Middleware` (RFC-0058), which is "
    "implemented over the `ros2::ros2_env_setup_*` helpers in "
    "packages/testing/nros-tests/src/ros2.rs. If this site genuinely cannot "
    "(it is not Rust, or it is outside the nros-tests workspace), add it to "
    "ALLOWLIST in this script WITH A REASON, so the exception is a line in a "
    "diff someone reviews."
)

CONSEQUENCE = (
    "A second spelling drifts from the first invisibly: the last one dropped "
    "the peer's ROS_DOMAIN_ID, which is the first segment of an rmw_zenoh "
    "keyexpr (peers never discover each other) and the key ros2cli's daemon is "
    "singleton on (parallel tests fight over domain 0's daemon); another "
    "hardcoded `humble`, so every guarded test SKIPPED forever on a jazzy host."
)


def comment_style(path):
    """Which comment markers this file's language uses.

    Returned per-language rather than as one union because the union is wrong in
    the direction that matters: `#` opens a comment in shell and closes a raw
    string delimiter in Rust, and `r#"source /opt/ros/..."#` is precisely the
    shape a copy-pasted bypass takes.
    """
    suffix = Path(path).suffix
    if suffix in SLASH_SUFFIXES:
        return "slash"
    if suffix in HASH_SUFFIXES:
        return "hash"
    if Path(path).name in SCANNED_NAMES:
        return "hash"
    return "hash"


def comment_col(line, style):
    """Column where a comment starts on this line, or None.

    A match at a column AFTER this one is inside the comment. A match BEFORE it
    is code with a trailing remark, which is still code.
    """
    if style == "slash":
        m = re.search(r"(?<![:/])//", line)
        if m:
            return m.start()
        m = re.search(r"/\*", line)
        if m:
            return m.start()
        if line.lstrip().startswith("*"):
            return 0
        return None
    # hash languages: a `#` at line start, or preceded by whitespace. `#` glued
    # to a word is a fragment (`$#`, `foo#bar`, a URL anchor), not a comment.
    m = re.search(r"(?:^|(?<=\s))#", line)
    return m.start() if m else None


def is_placeholder(tail):
    """`source /opt/ros/<distro>/setup.bash` — prose by construction.

    A literal `<distro>` never executes, so the line is addressed to a human
    whether it sits in a comment, an `eprintln!` hint or a `--help` string. This
    is the doc-comment rule applied to the one case that is PROVABLY not code,
    which is why it can be structural instead of an allowlist entry: several
    such hints exist in nros-tests itself, all telling a user how to fix an
    unsourced shell.
    """
    return tail.startswith("<")


def scan_text(path, text):
    """Hand-rolled ROS-env spellings in `text`, as (lineno, spelling, line).

    The whole rule lives here so the self-test can drive it on synthetic input
    rather than on whatever the working tree happens to contain.
    """
    if path in SANCTIONED or path in ALLOWLIST or _allowed_prefix(path):
        return []
    style = comment_style(path)
    is_python = Path(path).suffix == ".py"
    hits = []
    in_block = False   # /* ... */
    in_doc = None      # python triple-quote delimiter currently open
    for n, line in enumerate(text.splitlines(), start=1):
        stripped = line.strip()

        if is_python:
            if in_doc is not None:
                if in_doc in line:
                    in_doc = None
                continue
            for delim in ('"""', "'''"):
                first = line.find(delim)
                if first != -1 and line.count(delim) % 2 == 1:
                    in_doc = delim
                    line = line[:first]
                    break

        if style == "slash":
            if in_block:
                if "*/" in line:
                    in_block = False
                continue
            if "/*" in line and "*/" not in line:
                in_block = True

        if not stripped:
            continue

        ccol = comment_col(line, style)
        for name, pattern in SPELLINGS:
            for m in pattern.finditer(line):
                if ccol is not None and m.start() > ccol:
                    continue          # inside a comment
                if is_placeholder(m.group("tail")):
                    continue          # prose addressed to a human
                hits.append((n, name, stripped))
    return hits


def scannable(path):
    parts = Path(path).parts
    for seg in parts[:-1]:
        if seg in EXCLUDED_SEGMENTS or seg.startswith(EXCLUDED_PREFIXES):
            return False
    p = Path(path)
    return p.suffix in SCANNED_SUFFIXES or p.name in SCANNED_NAMES


def tracked_files():
    out = subprocess.run(
        ["git", "ls-files", "-z"],
        cwd=ROOT, capture_output=True, text=True, check=True,
    ).stdout
    return [p for p in out.split("\0") if p]


def self_test(quiet=False):
    """Negative controls — a rule that never fires proves nothing.

    Run on EVERY invocation, not only behind `--self-test`: a control nobody
    runs decays into a comment, and this gate's entire value is that it fires.
    """
    def fires(path, text):
        return bool(scan_text(path, text))

    # (1) The bypass this gate is about: real code, both spellings.
    bypass = ('    let cmd = format!("source /opt/ros/{d}/setup.bash && '
              'export RMW_IMPLEMENTATION=rmw_zenoh_cpp && ros2 topic list");')
    assert fires("packages/testing/nros-tests/src/other.rs", bypass), \
        "a hand-rolled setup in a non-sanctioned Rust file must FIRE"
    assert len(scan_text("a/b.rs", bypass)) == 2, \
        "both spellings must be reported, not just the first"

    # (2) Doc comments describe the procedure; they do not perform it.
    for prefix in ("//!", "///", "//", " * ", "#"):
        path = "a/b.rs" if prefix.startswith(("//", " *")) else "a/b.sh"
        assert not fires(path, f"{prefix} source /opt/ros/humble/setup.bash"), \
            f"{prefix!r} comment must NOT fire"
    assert not fires("a/b.rs", "/*\nsource /opt/ros/humble/setup.bash\n*/"), \
        "a block comment must NOT fire"
    assert not fires("a/b.py", '"""\nsource /opt/ros/humble/setup.bash\n"""'), \
        "a python docstring must NOT fire"
    assert fires("a/b.py", 'run("source /opt/ros/humble/setup.bash")'), \
        "python code after a docstring-free line must FIRE"
    assert not fires("a/b.rs", 'let x = 1; // source /opt/ros/humble/setup.bash'), \
        "a trailing comment must NOT fire"
    assert fires("a/b.sh", 'source /opt/ros/humble/setup.bash  # bootstrap'), \
        "code with a trailing REMARK is still code and must FIRE"

    # (3) `#` is a comment in shell and a raw-string delimiter in Rust. Reading
    #     `r#"` as a comment would blind the gate to the likeliest bypass shape.
    assert fires("a/b.rs", 'let s = r#"source /opt/ros/humble/setup.bash"#;'), \
        "a Rust raw string must FIRE — `#` is not a Rust comment"
    assert not fires("a/b.sh", '# source /opt/ros/humble/setup.bash'), \
        "a shell comment must NOT fire"

    # (4) The sanctioned files ARE the procedure.
    for path in SANCTIONED:
        assert not fires(path, bypass), f"{path} is sanctioned and must NOT fire"

    # (5) A placeholder distro cannot execute — it is a hint for a human.
    assert not fires("a/b.rs",
                     'eprintln!("run `source /opt/ros/<distro>/setup.bash`");'), \
        "a <distro> placeholder must NOT fire"
    # This script's own fixtures must not fire — see SANCTIONED. Asserted so the
    # exemption cannot be dropped by someone tidying the set.
    assert not fires(
        "scripts/check-ros-env-spelling.py",
        'assert fires("a/b.sh", "source /opt/ros/humble/setup.bash")',
    ), "the gate must exempt its own self-test specimens"
    assert fires("a/b.rs", 'run("source /opt/ros/humble/setup.bash");'), \
        "a CONCRETE distro in the same position must FIRE"

    # (6) The mention alone is not the spelling.
    assert not fires("a/b.sh", 'if [ -n "$RMW_IMPLEMENTATION" ]; then'), \
        "reading the variable is not exporting it"

    # (7) Allowlisting silences a file — and every entry must carry a reason and
    #     name a file that exists, else the list rots into a place to hide.
    victim = sorted(ALLOWLIST)[0]
    assert not fires(victim, bypass), "an allowlisted path must NOT fire"
    for path, reason in ALLOWLIST.items():
        assert reason and len(reason) > 20, f"{path}: allowlist entry needs a reason"
        assert path not in SANCTIONED, f"{path}: sanctioned files need no entry"
        assert (ROOT / path).exists(), \
            f"{path}: allowlisted but no such file — a stale exemption can only " \
            "ever be reclaimed by accident"

    # (8) Scope filtering: build output and vendored trees never reach the rule.
    for path in ("third-party/x/y.sh", "examples/foo/build/gen.sh",
                 "examples/foo/target/x.rs", "examples/foo/build-arm/x.sh",
                 "packages/x/generated/y.rs", "docs/notes.md", "a/b.toml"):
        assert not scannable(path), f"{path} must not be scanned"
    for path in ("packages/a/b.rs", "scripts/x.sh", "justfile",
                 "just/x.just", ".github/workflows/ci.yml"):
        assert scannable(path), f"{path} must be scanned"

    if not quiet:
        print("check-ros-env-spelling self-test: OK")
    return 0


def main():
    if "--self-test" in sys.argv:
        return self_test()
    self_test(quiet=True)

    findings = []
    scanned = 0
    for path in tracked_files():
        if not scannable(path):
            continue
        full = ROOT / path
        if not full.is_file():
            continue
        try:
            text = full.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        scanned += 1
        for lineno, spelling, line in scan_text(path, text):
            findings.append((path, lineno, spelling, line))

    print(f"ROS env spelling: {scanned} tracked source files scanned, "
          f"{len(SANCTIONED)} sanctioned, {len(ALLOWLIST)} allowlisted")

    if findings:
        print("\n[FAIL] hand-rolled ROS 2 environment setup outside the "
              "sanctioned files:", file=sys.stderr)
        for path, lineno, spelling, line in findings:
            print(f"  - {path}:{lineno}: {spelling}", file=sys.stderr)
            print(f"      {line[:160]}", file=sys.stderr)
        print(f"\n  WHAT TO USE INSTEAD: {REMEDY}", file=sys.stderr)
        print(f"\n  WHY IT MATTERS: {CONSEQUENCE}", file=sys.stderr)
        return 1

    print("Every ROS 2 environment goes through nros_tests::ros_env::RosEnv.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
