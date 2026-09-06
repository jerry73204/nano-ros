#!/usr/bin/env python3
"""An indexed tool has ONE producer: `nros setup --tool <name>` — phase-422 W5.

WHAT THIS EXISTS TO KEEP FIXED

W1 retired two second producers. `just workspace install-corrosion` and
`install-play-launch-parser` each did their own download into the same versioned
prefix `nros setup --tool` writes, each with its own stamp logic, and their
version pins were not even spelled the same:

    corrosion            just: `v0.6.1`         index: `0.6.1-nros1`
    play_launch_parser   just: SHA 838ce948…    index: `0.1.0-nros1`

Two producers for one prefix is issue 0500's failure mode. The store accumulates
and nothing prunes it, prefixes resolve newest-first, and BOTH paths print
success — so a stale Corrosion shadowed the pin that had just been installed and
the `mixed` workspace could not link. It is not a convenience: it is a second
answer to "which version is installed?", and the two answers do not have to be
wrong to be a problem, only different.

Both are forwarders now. Nothing stops the next one from growing back, which is
what this gate is for.

WHAT COUNTS AS PRODUCING

A recipe or script body that NAMES an indexed tool and also does one of:

  * fetches it            curl / wget / git clone / a release URL
  * unpacks it            tar -x / unzip into a prefix
  * builds it             cargo install / make install / cmake --install
  * installs it via the OS  apt / dnf / pacman / brew install
  * decides freshness itself   `.installed-version`, a version stamp compare

...without handing the work to `nros setup --tool <name>` in the same body.

Forwarding is FINE and is the point: `install-corrosion` still exists, still has
the name people type, and its body is one `nros setup --tool corrosion`. So is a
body that only READS the store (`nros sdk-path`), reports (`doctor`), or names
the tool in prose.

WHAT IS OUT OF SCOPE, AND WHY

`[prereq.*]` OS packages are a different class with a different rule: composing
the install command is nano-ros's job and running it is the user's (RFC-0062),
which `check-sysdep-remedies` already enforces. A lane that apt-installs a
declared PREREQ as root is judged by that gate, not this one.

Run:  python3 scripts/check-one-producer-per-tool.py [--self-test]
"""

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Scanned surfaces: the `just` recipes people invoke, and the shell scripts they
# call. `scripts/probe/` is excluded — it provisions a PRISTINE CONTAINER from
# the book's own instructions, which is the thing being tested rather than a
# second path into this tree's store.
JUST_FILES = ["justfile"]
JUST_DIR = "just"
SCRIPT_DIRS = ["scripts"]
SCRIPT_PRUNE = ("scripts/probe/",)

# Producing, in the sense above. Each is (regex, what it does).
PRODUCE = [
    (re.compile(r"\bcurl\s+-|\bwget\s"), "downloads"),
    (re.compile(r"\bgit\s+clone\b"), "clones"),
    (re.compile(r"\btar\s+-?x|\bunzip\b"), "unpacks"),
    (re.compile(r"\bcargo\s+install\b"), "builds with cargo install"),
    (re.compile(r"\bmake\s+install\b|cmake\s+--install\b"), "builds and installs"),
    (
        re.compile(r"\b(apt-get|apt|dnf|pacman|brew)\s+install\b"),
        "installs an OS package",
    ),
    (re.compile(r"\.installed-version\b"), "keeps its own install stamp"),
]

# Handing the work to the one producer.
FORWARDS = re.compile(r"nros\s+setup\b[^\n]*--tool")

# A line whose whole job is to say something.
PRINTS = re.compile(r"^(echo|printf|>&2\s|nros_(check|lane)_skip|warn|die|fail)\b")

# (file, recipe-or-function name) -> reason. Checked in BOTH directions.
#
# EMPTY. An entry needs a reason that is a property of the TOOL, not of the
# caller: "this one is faster to install directly" is not one, because that is
# exactly what both retired producers were.
EXEMPT = {}


def indexed_tools(path):
    """`[tool.<name>]` keys, excluding the `[tool.<name>.source]` sub-tables."""
    out = []
    with open(path, encoding="utf8") as fh:
        for line in fh:
            m = re.match(r"^\[tool\.([A-Za-z0-9._-]+)\]\s*$", line.strip())
            if m and not m.group(1).endswith(".source"):
                out.append(m.group(1))
    return sorted(set(out))


def just_bodies(text):
    """[(name, first_line, body)] for every recipe in a justfile.

    A recipe header sits at column 0 and ends in `:`; its body is the indented
    run that follows. Attributes (`[group("x")]`) and comments are not headers.
    """
    lines = text.split("\n")
    out = []
    i = 0
    header = re.compile(r"^([a-z0-9][a-z0-9_-]*)\s*(\+?\*?[A-Za-z0-9_= \"'{}.-]*):")
    while i < len(lines):
        m = header.match(lines[i])
        if not m:
            i += 1
            continue
        name = m.group(1)
        body = []
        j = i + 1
        while j < len(lines):
            line = lines[j]
            if line.strip() and not line[:1].isspace():
                break
            body.append(line)
            j += 1
        out.append((name, i + 1, "\n".join(body)))
        i = j
    return out


def shell_bodies(text):
    """[(name, first_line, body)] for shell functions, plus the whole file.

    A script without functions still counts: `scripts/setup-mdbook.sh` was one
    long body, and that is the shape this gate has to catch.
    """
    out = [("<file>", 1, text)]  # a script with no functions is still one body
    lines = text.split("\n")
    header = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)\s*\(\)\s*\{")
    for i, line in enumerate(lines):
        m = header.match(line)
        if not m:
            continue
        depth = 0
        body = []
        for j in range(i, len(lines)):
            depth += lines[j].count("{") - lines[j].count("}")
            body.append(lines[j])
            if depth <= 0 and j > i:
                break
        out.append((m.group(1), i + 1, "\n".join(body)))
    return out


def strip_comments(body, marker="#"):
    """Drop comment lines. A comment QUOTING a download is prose, not a producer."""
    kept = []
    for line in body.split("\n"):
        s = line.strip()
        if s.startswith(marker):
            continue
        kept.append(line)
    return "\n".join(kept)


def offenders(bodies, tools, origin):
    """[(origin, name, lineno, tool, what)] for every second producer.

    LINE granularity, not body. The first version asked "does this body name the
    tool AND contain a producer verb anywhere", and that is far too coarse: every
    script in this tree contains the word `nros` somewhere, `nros` is itself an
    indexed tool since phase-431 W3, and a script that apt-installs something
    unrelated then reads as a second producer of the CLI. It reported 25
    problems, 25 of them false.

    The real shape puts the two together on one line -- `curl … corrosion.tar.gz`,
    `cargo install --root $prefix corrosion`, `apt-get install ninja-build` -- so
    that is what is matched.
    """
    found = []
    for name, lineno, body in bodies:
        lines = body.split("\n")
        forwards = FORWARDS.search(strip_comments(body))
        for offset, raw in enumerate(lines):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            # A line that PRINTS is advice, not provisioning. Both of this
            # gate's first false positives were exactly this: an `echo` telling
            # a user to `apt install gcc-arm-none-eabi`, and a skip message
            # quoting `apt install python3-colcon-common-extensions`. Neither
            # installs anything, and `check-sysdep-remedies` is what judges
            # whether a printed remedy should have been derived from the index.
            if PRINTS.match(line):
                continue
            for pattern, what in PRODUCE:
                if not pattern.search(line):
                    continue
                for tool in tools:
                    # `_` closes the identifier too: `nros_check_skip` and
                    # `nros_lane_skip` are helpers, not the tool `nros`, which
                    # became an indexed tool in phase-431 W3 and so made every
                    # such helper look like a naming of it.
                    if not re.search(
                        r"(?<![A-Za-z0-9_-])%s(?![A-Za-z0-9_-])" % re.escape(tool), line
                    ):
                        continue
                    if forwards:
                        continue
                    found.append((origin, name, lineno + offset, tool, what))
                break
    return found


def scan_paths():
    """Every file this gate reads, from `git ls-files`.

    The index, not a filesystem walk (`check-no-tracked-file-find`): an untracked
    path cannot be a second producer anyone runs, and a walk wanders into
    `scripts/zephyr/sdk/`, which is a PROVISIONED Zephyr SDK carrying upstream's
    own installer -- flagged by the walk, and not ours to change.
    """
    out = subprocess.run(
        ["git", "-C", ROOT, "ls-files", "-z"], capture_output=True, check=True
    ).stdout.decode("utf8")
    paths = []
    for rel in out.split("\0"):
        if not rel:
            continue
        if any(rel.startswith(p) for p in SCRIPT_PRUNE):
            continue
        if rel in JUST_FILES or (rel.startswith(JUST_DIR + "/") and rel.endswith(".just")):
            paths.append((rel, "just"))
        elif rel.endswith(".sh") and any(
            rel.startswith(d + "/") for d in SCRIPT_DIRS
        ):
            paths.append((rel, "sh"))
    return paths


def self_test():
    tools = ["corrosion", "mdbook", "qemu"]

    # A forwarder is fine, even though it names the tool.
    ok = [("install-corrosion", 1, "    nros setup --tool corrosion\n")]
    assert offenders(ok, tools, "just") == [], offenders(ok, tools, "just")

    # A second producer is not — this is `install-corrosion` before W1.
    bad = [
        (
            "install-corrosion",
            10,
            "    curl -L https://example/corrosion.tar.gz -o c.tgz\n"
            "    tar -xf c.tgz -C $prefix\n",
        )
    ]
    got = offenders(bad, tools, "just")
    assert len(got) == 1 and got[0][3] == "corrosion", got
    assert got[0][4] == "downloads", got

    # Its own freshness stamp counts even with no download in sight.
    stamp = [
        ("install-corrosion", 3, "    cat ~/.nros/sdk/corrosion/.installed-version\n")
    ]
    assert offenders(stamp, tools, "just")[0][4] == "keeps its own install stamp"

    # A comment quoting the retired command is prose.
    prose = [
        (
            "install-corrosion",
            1,
            "    # used to be: curl -L https://example/corrosion.tar.gz\n"
            "    nros setup --tool corrosion\n",
        )
    ]
    assert offenders(prose, tools, "just") == [], offenders(prose, tools, "just")

    # Advice is not provisioning — the gate's first two false positives.
    advice = [
        ("x", 1, '    echo "Install it (e.g. sudo apt install qemu-system-arm)"\n'),
        ("y", 1, '    nros_check_skip parity "colcon not found (apt install colcon)"\n'),
    ]
    assert offenders(advice, tools, "just") == [], offenders(advice, tools, "just")

    # A helper whose name merely STARTS with a tool name is not that tool.
    helper = [("z", 1, "    nros_lane_skip qemu-lane \"apt install x\"\n")]
    assert offenders(helper, ["nros"], "just") == [], offenders(helper, ["nros"], "just")

    # A body that downloads something ELSE is not this gate's business.
    other = [("setup-rustup", 1, "    curl -sSf https://sh.rustup.rs | sh\n")]
    assert offenders(other, tools, "just") == [], offenders(other, tools, "just")

    # Substring safety: `qemu-baremetal` in a path must not read as `qemu`
    # having been named... but `qemu` as a bare word must.
    sub = [("x", 1, "    tar -xf build/qemu-baremetal-notes.tgz\n")]
    assert offenders(sub, tools, "just") == [], offenders(sub, tools, "just")
    named = [("x", 1, "    tar -xf qemu.tgz\n")]
    assert len(offenders(named, tools, "just")) == 1

    # --- body parsing ---
    j = "\n".join(
        [
            "[group(\"setup\")]",
            "install-corrosion:",
            "    nros setup --tool corrosion",
            "",
            "other-recipe arg=\"x\":",
            "    echo hi",
        ]
    )
    names = [b[0] for b in just_bodies(j)]
    assert names == ["install-corrosion", "other-recipe"], names

    sh = "\n".join(
        [
            "#!/usr/bin/env bash",
            "install_it() {",
            "    curl -L https://example/mdbook.tgz",
            "}",
            "echo done",
        ]
    )
    fns = [b[0] for b in shell_bodies(sh)]
    assert fns == ["<file>", "install_it"], fns

    # --- the index parse, which everything else depends on ---
    tools_real = indexed_tools(os.path.join(ROOT, "nros-sdk-index.toml"))
    assert "corrosion" in tools_real and "mdbook" in tools_real, tools_real
    assert not any(t.endswith(".source") for t in tools_real), tools_real

    sys.stdout.write("check-one-producer-per-tool self-test: OK\n")


def main():
    if "--self-test" in sys.argv:
        self_test()
        return 0
    self_test()

    index = os.path.join(ROOT, "nros-sdk-index.toml")
    tools = indexed_tools(index)
    if not tools:
        sys.stderr.write(
            "error: parsed ZERO `[tool.*]` entries from nros-sdk-index.toml.\n"
            "This gate would then match nothing and pass vacuously.\n"
        )
        return 1

    problems = []
    seen_exempt = set()
    scanned = 0
    for rel, kind in scan_paths():
        # `errors="replace"`: a `.sh` under `scripts/` may be a test FIXTURE
        # carrying binary bytes, and a decode error there must not take the gate
        # down — the patterns below are ASCII either way.
        with open(os.path.join(ROOT, rel), encoding="utf8", errors="replace") as fh:
            text = fh.read()
        scanned += 1
        bodies = just_bodies(text) if kind == "just" else shell_bodies(text)
        for _origin, name, lineno, tool, what in offenders(bodies, tools, rel):
            if (rel, name) in EXEMPT:
                seen_exempt.add((rel, name))
                continue
            problems.append(
                "%s:%d  `%s` %s `%s`, which the index already declares.\n"
                "    Two producers for one prefix is issue 0500: the store\n"
                "    accumulates, prefixes resolve newest-first, and BOTH paths\n"
                "    print success — so the stale one shadows the pin that was\n"
                "    just installed. Forward instead:\n"
                "        nros setup --tool %s" % (rel, lineno, name, what, tool, tool)
            )

    for key in EXEMPT:
        if key not in seen_exempt:
            problems.append(
                "STALE exemption %r matches nothing.\n"
                "    Delete it — an allow-list checked one way stops covering\n"
                "    what it claims to." % (key,)
            )

    if problems:
        sys.stderr.write("check-one-producer-per-tool: %d problem(s)\n\n" % len(problems))
        for p in problems:
            sys.stderr.write("  - %s\n\n" % p)
        return 1

    sys.stdout.write(
        "check-one-producer-per-tool OK — %d indexed tool(s), %d file(s) scanned;\n"
        "every caller forwards to `nros setup --tool`.\n" % (len(tools), scanned)
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
