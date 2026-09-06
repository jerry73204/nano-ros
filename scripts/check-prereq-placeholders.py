#!/usr/bin/env python3
"""A `[prereq.*]` package name may carry ONE placeholder: `{ros_distro}`.

Issue 1128. A ROS package name is distro-parametric — `ros-<distro>-rmw-zenoh-cpp`
— and the index could spell only one distro, so it pinned humble and every
consumer that cared composed the name itself: `just ci provision-zenohd`,
`scripts/ros/domain-bridge-repro.sh` and `docker/ros-editions/Dockerfile` all
build `ros-${DISTRO}-…` by hand. Two spellings of one package, agreeing only
when the distro is humble.

`{ros_distro}` fixes that by making the parameter DATA. This gate is what keeps
the fix from turning into the thing phase-422 W2 declined to build.

WHY THE VOCABULARY IS GATED AT ALL

The expansion has TWO implementations, in two languages, and neither can call
the other:

    PrereqContext::expand           packages/cli/nros-cli-core/…/sdk_index.rs
    expand()                        scripts/sdk/prereq-packages.py

The Python helper exists precisely so a job needing two package names does not
have to build the CLI first (phase-413 W3), so "make one call the other" is not
available. Two implementations of one rule is the drift this repo keeps paying
for — `check-rmw-api-parity`'s map, the sizes-header mirror, the fixture
coordinate derived twice. What makes it survivable here is that the rule is ONE
LINE LONG in each language, and that is only true while the vocabulary is one
name. This gate is the thing that holds it to one.

So: adding `{arch}` is not a matter of editing two files. It means arguing that
a second parametric prereq exists (today `ros-rmw-zenoh-cpp` is the only one),
then updating both implementations, this gate, and both self-tests.

Run:  python3 scripts/check-prereq-placeholders.py [--self-test]
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INDEX = os.path.join(ROOT, "nros-sdk-index.toml")

# The whole vocabulary. Adding to this is the decision the docstring describes.
KNOWN = {"ros_distro"}

MANAGERS = ("apt", "dnf", "pacman", "brew")
PLACEHOLDER = re.compile(r"\{([A-Za-z0-9_]*)\}")

# The two files that must expand the same set. Each is checked by EXERCISING
# its expander, not by grepping the file: the first version grepped, and a
# mutation that made the Python `expand` return its input unchanged still
# passed, because the name it no longer expanded was still written in a comment
# and a fixture three lines away.
RUST_IMPL = "packages/cli/nros-cli-core/src/orchestration/sdk_index.rs"
PY_IMPL = "scripts/sdk/prereq-packages.py"

def load(path):
    try:
        import tomllib as toml
    except ModuleNotFoundError:  # py<3.11
        import tomli as toml
    with open(path, "rb") as fh:
        return toml.load(fh)


def offenders(index):
    """[(key, manager, package, name)] for every unknown placeholder."""
    out = []
    for table in ("prereq", "system"):
        for key, entry in (index.get(table) or {}).items():
            if not isinstance(entry, dict):
                continue
            for manager in MANAGERS:
                for pkg in entry.get(manager) or []:
                    for m in PLACEHOLDER.finditer(pkg):
                        if m.group(1) not in KNOWN:
                            out.append((key, manager, pkg, m.group(1)))
    return out


def used(index):
    """The placeholder names the index actually uses."""
    seen = set()
    for table in ("prereq", "system"):
        for entry in (index.get(table) or {}).values():
            if not isinstance(entry, dict):
                continue
            for manager in MANAGERS:
                for pkg in entry.get(manager) or []:
                    seen.update(PLACEHOLDER.findall(pkg))
    return seen


def rust_expands(name):
    """Does `PrereqContext::expand` name `{name}` IN ITS BODY?

    Body-scoped on purpose. The whole file mentions `{ros_distro}` in doc
    comments and tests, so a file-wide search answers a different question from
    the one that matters — which is whether the substitution happens.
    """
    path = os.path.join(ROOT, RUST_IMPL)
    try:
        with open(path, encoding="utf8") as fh:
            text = fh.read()
    except OSError:
        return False
    m = re.search(r"pub fn expand\(&self, s: &str\) -> String \{(.*?)\n    \}", text, re.S)
    if not m:
        return False
    return ("{%s}" % name) in m.group(1)


def python_expands(name):
    """Does the Python `expand` actually SUBSTITUTE `{name}`?

    Imported and called, so a body that stopped substituting fails here however
    many times the name still appears in its comments.
    """
    import importlib.util

    path = os.path.join(ROOT, PY_IMPL)
    spec = importlib.util.spec_from_file_location("_nros_prereq_packages", path)
    if spec is None or spec.loader is None:
        return False
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except Exception:  # a helper that will not even import cannot expand
        return False
    ctx = {name: "SENTINEL"}
    try:
        return mod.expand("x-{%s}-y" % name, ctx) == "x-SENTINEL-y"
    except Exception:
        return False


def implementations_handle(name):
    """[path] for every implementation that does not expand `{name}`."""
    missing = []
    if not rust_expands(name):
        missing.append(RUST_IMPL)
    if not python_expands(name):
        missing.append(PY_IMPL)
    return missing


def self_test():
    ok = {"prereq": {"z": {"apt": ["ros-{ros_distro}-rmw-zenoh-cpp"], "dnf": ["x"]}}}
    assert offenders(ok) == [], offenders(ok)
    assert used(ok) == {"ros_distro"}, used(ok)

    bad = {"prereq": {"q": {"apt": ["qemu-{arch}-static"]}}}
    got = offenders(bad)
    assert len(got) == 1 and got[0][3] == "arch", got

    # `[system.*]` is the same table under its alias, so it is scanned too.
    aliased = {"system": {"q": {"brew": ["{nope}"]}}}
    assert len(offenders(aliased)) == 1, offenders(aliased)

    # An empty `{}` is not a placeholder anyone declared, and must not pass.
    empty = {"prereq": {"e": {"apt": ["a-{}-b"]}}}
    assert len(offenders(empty)) == 1, offenders(empty)

    # A name with no braces is fine under any manager.
    plain = {"prereq": {"d": {"apt": ["doxygen"], "pacman": ["doxygen"]}}}
    assert offenders(plain) == [] and used(plain) == set()

    # The two implementations really do name the one placeholder — this is the
    # arm that fails if someone adds a name to KNOWN and edits only one of them.
    assert implementations_handle("ros_distro") == [], implementations_handle("ros_distro")

    sys.stdout.write("check-prereq-placeholders self-test: OK\n")


def main():
    if "--self-test" in sys.argv:
        self_test()
        return 0
    self_test()

    index = load(INDEX)
    problems = []

    for key, manager, pkg, name in offenders(index):
        problems.append(
            "[prereq.%s] %s = [.. \"%s\" ..] uses `{%s}`, which nothing expands.\n"
            "    The vocabulary is exactly %s. A name the index invents is\n"
            "    installed LITERALLY, braces and all, by whichever consumer got\n"
            "    there first — which is the drift issue 1128 records, with the\n"
            "    braces made visible.\n"
            "    Adding one means arguing a second parametric prereq exists,\n"
            "    then updating BOTH expanders and their self-tests:\n"
            "        %s"
            % (
                key,
                manager,
                pkg,
                name,
                sorted("{%s}" % k for k in KNOWN),
                "\n        ".join(rel for rel, _ in IMPLEMENTATIONS),
            )
        )

    # Both directions. A vocabulary entry nothing uses is a rule with no
    # subject, and one an implementation has forgotten is the drift itself.
    in_use = used(index)
    for name in sorted(KNOWN):
        if name not in in_use:
            problems.append(
                "`{%s}` is in the vocabulary and NOTHING in the index uses it.\n"
                "    Delete it — a placeholder with no subject is two\n"
                "    implementations kept alive for nobody." % name
            )
            continue
        missing = implementations_handle(name)
        if missing:
            problems.append(
                "`{%s}` is used by the index but not named in:\n        %s\n"
                "    That file expands it to nothing, or leaves the braces in.\n"
                "    Both expanders must know every name in the vocabulary."
                % (name, "\n        ".join(missing))
            )

    if problems:
        sys.stderr.write("check-prereq-placeholders: %d problem(s)\n\n" % len(problems))
        for p in problems:
            sys.stderr.write("  - %s\n\n" % p)
        return 1

    sys.stdout.write(
        "check-prereq-placeholders OK — vocabulary %s, used by the index and named\n"
        "by both expanders.\n" % sorted("{%s}" % k for k in KNOWN)
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
