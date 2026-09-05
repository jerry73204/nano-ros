#!/usr/bin/env python3
"""CI builds the `nros` CLI from source and proves it -- it never installs the
release (phase-431 W2, RFC-0090).

Two directions, and neither implies the other.

ARM A -- no workflow acquires the CLI as a RELEASE ASSET.

    Phase-431 ships a prebuilt `nros`. From that day a workflow can get the
    binary in one line, and a job that does stops testing THIS TREE while
    staying green: it exercises whatever emitter the last release carried. The
    codegen version does not catch it -- a release at the SAME version emits
    ITS OWN bytes (that is exactly the gap phase-431 W1's workspace guard
    closes for a developer, and a workflow has no developer to warn).

    So the release is for users; contributors and CI build from source, always.

ARM B -- every from-source build ASSERTS the binary matches the checkout.

    phase-429 W5's rule. The CLI cache key is narrower than `source_stamp.rs`'s
    watch set (it misses `.jinja`, `cli-source-dirs.txt`, the out-of-tree crate
    dirs and the `play_launch` pin), so an exact-key cache hit exists for a tree
    whose stamp has moved, and cargo will not rebuild it. `nros source-stamp`
    is the binary's own answer to "do I match these sources"; a build step that
    skips it is hoping.

    This arm found its first offender on the run that introduced it: `gate.yml`
    warms the CLI for `check cli-tests` and asserted nothing, so four of five
    sites carried the rule and the fifth read exactly like the other four.

Run:  python3 scripts/check-ci-cli-from-source.py [--self-test]
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORKFLOWS = os.path.join(ROOT, ".github", "workflows")

# The from-source build, and the assertion that must accompany it.
BUILD_RE = re.compile(r"cargo build\b.*--bin nros\b")
STAMP_RE = re.compile(r"\bsource-stamp\b")

# Arm A. Each entry is (regex, what it would do). Matched against every
# non-comment LOGICAL line of every workflow (backslash continuations joined),
# and a match counts only when that logical line also names `nros`: CI legitimately
# downloads OTHER projects' releases, and `docs.yml` fetching `mdbook-mermaid`
# from a GitHub release URL was this gate's first false positive.
#
# `scripts/install.sh` is here because it IS the release path: phase-431 W4 made
# it the user front door, downloading a release into the SDK store. A workflow
# running it is a workflow testing the last release instead of this tree.
#
# `scripts/bootstrap.sh` is deliberately NOT here. W4 first planned to make
# download ITS default with a `--from-source` opt-out, and that turned out not
# to be implementable: bootstrap runs inside a checkout, and a released binary
# cannot serve a checkout — `refuse_if_foreign_to_workspace` refuses it (W1),
# `just doctor` fails on it (W2), and copying it into `packages/cli/target/`
# does not help either, because `nros source-stamp` compares against the
# sources it was built from. So the two front doors split by AUDIENCE, and
# bootstrap building from source is exactly what this gate wants.
RELEASE_ACQUISITION = [
    (
        re.compile(r"gh release download"),
        "downloads a release asset",
    ),
    (
        re.compile(r"releases/download/"),
        "fetches a release asset by URL",
    ),
    (
        re.compile(r"\bnros-[^\s\"']*\.tar\.(zst|gz|xz)"),
        "names the released CLI tarball",
    ),
    (
        re.compile(r"--tool[= ]nros\b"),
        "installs the CLI through the SDK store",
    ),
    (
        re.compile(r"scripts/install\.sh"),
        "runs the user installer, which downloads a release",
    ),
]

# (workflow, exact stripped line) -> reason. Checked in BOTH directions: an
# exemption matching nothing is a stale allow-list, which is how a gate quietly
# stops covering what it names.
#
# EMPTY. A per-line entry needs a reason that is a property of the LANE.
EXEMPT = {}

# Workflows that PRODUCE the release rather than acquire it -- arm A does not
# apply to them, arm B still does.
#
# The distinction is this gate's whole subject. `release-nros.yml` names the
# asset a dozen times because it BUILDS one, and it runs `scripts/install.sh`
# against what it just built -- the strongest check the release has, since the
# installer's own refusals are gated against a synthetic tarball and only this
# catches an asset that is well-formed and wrong. Banning that removes a check
# rather than adding one.
#
# Arm B is what keeps this from being a hole: a producer must still build the
# CLI from source and assert its source stamp, which matters most for the one
# artifact nobody downstream can re-check.
RELEASE_WORKFLOWS = {
    "release-nros.yml": (
        "phase-431 W5 -- it BUILDS the release, and installs its own asset as "
        "a pre-publish check"
    ),
}


def run_blocks(text):
    """[(lineno_of_run_key, [body lines])] for every `run:` in a workflow.

    Block-scoped rather than file-scoped on purpose: arm B asks whether the
    assertion sits WITH the build. A `source-stamp` in some other job's step
    proves nothing about this one, and a file-wide search would accept it.
    """
    lines = text.split("\n")
    out = []
    i = 0
    while i < len(lines):
        block = re.match(r"^(\s*)run:\s*([|>][-+0-9]*)?\s*$", lines[i])
        if not block:
            inline = re.match(r"^(\s*)run:\s+(\S.*)$", lines[i])
            if inline:
                out.append((i + 1, [inline.group(2)]))
            i += 1
            continue
        key_indent = len(block.group(1))
        body = []
        j = i + 1
        while j < len(lines):
            line = lines[j]
            if line.strip():
                indent = len(line) - len(line.lstrip())
                if indent <= key_indent:
                    break
            body.append(line)
            j += 1
        out.append((i + 1, body))
        i = j
    return out


def build_sites(text):
    """[(lineno, line, asserts_stamp)] for every from-source CLI build."""
    out = []
    for start, body in run_blocks(text):
        has_stamp = any(
            STAMP_RE.search(b) for b in body if not b.strip().startswith("#")
        )
        for offset, line in enumerate(body):
            stripped = line.strip()
            if stripped.startswith("#"):
                continue
            if BUILD_RE.search(stripped):
                out.append((start + 1 + offset, stripped, has_stamp))
    return out


NROS_RE = re.compile(r"\bnros\b")
INSTALLER_RE = re.compile(r"scripts/install\.sh")


def logical_lines(text):
    """[(lineno_of_first_line, joined text)] with `\\` continuations joined.

    A shell command wrapped over three lines is ONE command, and the URL that
    decides whether it acquires our CLI is often not on the line the verb is.
    Attributed to the first line, which is where a reader looks.
    """
    out = []
    start = None
    buf = []
    for i, raw in enumerate(text.split("\n"), 1):
        line = raw.strip()
        if start is None:
            start = i
        buf.append(line[:-1].rstrip() if line.endswith("\\") else line)
        if line.endswith("\\"):
            continue
        out.append((start, " ".join(buf).strip()))
        start = None
        buf = []
    if buf:
        out.append((start, " ".join(buf).strip()))
    return out


def release_sites(text):
    """[(lineno, line, what)] for every release-asset acquisition of the CLI."""
    out = []
    for i, line in logical_lines(text):
        if not line or line.startswith("#"):
            continue
        # `scripts/install.sh` is exempt from the name filter: it is the nros
        # installer and spells the binary nowhere on its own invocation line.
        if not NROS_RE.search(line) and not INSTALLER_RE.search(line):
            continue
        for pattern, what in RELEASE_ACQUISITION:
            if pattern.search(line):
                out.append((i, line, what))
                break
    return out


def workflow_files():
    return sorted(
        fn for fn in os.listdir(WORKFLOWS) if fn.endswith((".yml", ".yaml"))
    )


def self_test():
    # --- run_blocks ---
    y = "\n".join(
        [
            "jobs:",
            "  a:",
            "    steps:",
            "      - name: build",
            "        run: |",
            "          cargo build --release --manifest-path packages/cli/Cargo.toml --bin nros",
            '          "$X/nros" source-stamp >/dev/null',
            "      - name: other",
            "        run: echo hi",
            "  b:",
            "    steps:",
            "      - name: build without the assertion",
            "        run: |",
            "          cargo build --release --bin nros",
            "      - name: elsewhere",
            "        run: nros source-stamp",
        ]
    )
    blocks = run_blocks(y)
    assert len(blocks) == 4, blocks
    assert blocks[1][1] == ["echo hi"], blocks[1]

    got = build_sites(y)
    assert [(g[0], g[2]) for g in got] == [(6, True), (14, False)], got

    # A stamp in a DIFFERENT step must not satisfy the build in this one --
    # that is the whole reason the scan is block-scoped.
    assert got[1][2] is False, got[1]

    # A commented-out assertion is not an assertion.
    y2 = "\n".join(
        [
            "      - name: build",
            "        run: |",
            "          cargo build --bin nros",
            "          # nros source-stamp   (disabled while debugging)",
        ]
    )
    assert build_sites(y2)[0][2] is False, build_sites(y2)

    # --- arm A ---
    bad = "\n".join(
        [
            "        run: |",
            "          gh release download v1.2.3 -p 'nros-*'",
            "          curl -L https://example/releases/download/v1/nros.tar.zst",
            "          tar xf nros-x86_64-linux.tar.zst",
            "          nros setup --tool nros",
            "          sh scripts/install.sh",
            "          # gh release download in a comment is prose",
        ]
    )
    hits = release_sites(bad)
    assert [h[0] for h in hits] == [2, 3, 4, 5, 6], hits
    assert "installer" in hits[4][2], hits[4]

    # bootstrap builds from source, which is the point — never an offender.
    boot = "        run: ./scripts/bootstrap.sh"
    assert release_sites(boot) == [], release_sites(boot)

    # Another project's release is not ours. `docs.yml` fetching mdbook-mermaid
    # from a `releases/download/` URL was this gate's first false positive.
    other = "\n".join(
        [
            "        run: |",
            "          curl -sSL https://github.com/badboy/mdbook-mermaid/releases/download/v0.17.0/x.tar.gz \\",
            "            | tar -xz",
        ]
    )
    assert release_sites(other) == [], release_sites(other)

    # ...but a wrapped acquisition of OURS is still one command.
    wrapped = "\n".join(
        [
            "        run: |",
            "          curl -sSL \\",
            "            https://github.com/NEWSLabNTU/nano-ros/releases/download/v1/nros-linux.tar.zst \\",
            "            -o nros.tar.zst",
        ]
    )
    assert [h[0] for h in release_sites(wrapped)] == [2], release_sites(wrapped)

    ok = "\n".join(
        [
            "        run: |",
            "          ./scripts/bootstrap.sh",
            "          cargo build --release --bin nros",
        ]
    )
    assert release_sites(ok) == [], release_sites(ok)

    # A producer is exempt from arm A and NOT from arm B -- the half that keeps
    # the exemption from being a hole.
    prod = "\n".join(
        [
            "        run: |",
            "          gh release create v1 nros-linux-x86_64.tar.zst",
            "          cargo build --release --bin nros",
        ]
    )
    assert len(release_sites(prod)) == 1, release_sites(prod)
    assert build_sites(prod)[0][2] is False, build_sites(prod)

    sys.stdout.write("check-ci-cli-from-source self-test: OK\n")


def main():
    if "--self-test" in sys.argv:
        self_test()
        return 0
    self_test()

    problems = []
    seen_exempt = set()
    total_builds = 0

    known = set(workflow_files())
    for name, reason in RELEASE_WORKFLOWS.items():
        if name not in known:
            problems.append(
                "STALE producer exemption %r (%s) -- no such workflow.\n"
                "    Delete it; an allow-list checked one way stops covering\n"
                "    what it claims to." % (name, reason)
            )

    for fn in workflow_files():
        with open(os.path.join(WORKFLOWS, fn), encoding="utf8") as fh:
            text = fh.read()

        for lineno, line, what in ([] if fn in RELEASE_WORKFLOWS else release_sites(text)):
            if (fn, line) in EXEMPT:
                seen_exempt.add((fn, line))
                continue
            problems.append(
                "%s:%d %s\n"
                "      %s\n"
                "    CI must build the CLI from source. A released binary emits\n"
                "    ITS OWN codegen, so this job would stop testing the tree while\n"
                "    staying green -- and the codegen version does not catch it,\n"
                "    because a release at the same version is not incompatible,\n"
                "    merely different (RFC-0090, phase-431 W1/W2).\n"
                "    Build it:  cargo build --release --manifest-path packages/cli/Cargo.toml --bin nros\n"
                "               \"$GITHUB_WORKSPACE/packages/cli/target/release/nros\" source-stamp >/dev/null"
                % (fn, lineno, what, line)
            )

        for lineno, line, has_stamp in build_sites(text):
            total_builds += 1
            if has_stamp or (fn, line) in EXEMPT:
                if (fn, line) in EXEMPT:
                    seen_exempt.add((fn, line))
                continue
            problems.append(
                "%s:%d builds the CLI without asserting it matches the checkout\n"
                "      %s\n"
                "    The CLI cache key is narrower than `source_stamp.rs`'s watch\n"
                "    set, so an exact-key hit exists for a tree whose stamp moved\n"
                "    and cargo will not rebuild it (phase-429 W5). Add to the SAME\n"
                "    step:\n"
                "        \"$GITHUB_WORKSPACE/packages/cli/target/release/nros\" source-stamp >/dev/null"
                % (fn, lineno, line)
            )

    for key in EXEMPT:
        if key not in seen_exempt:
            problems.append(
                "STALE exemption %r matches no workflow line.\n"
                "    Delete it -- an allow-list checked one way stops covering\n"
                "    what it claims to." % (key,)
            )

    # Anti-vacuity. Arm A passes on a tree that acquires the CLI nowhere, and
    # arm B on a tree that builds it nowhere -- both of which mean the workflows
    # were restructured out from under this gate, not that they are clean.
    if total_builds == 0:
        sys.stderr.write(
            "error: no workflow builds the nros CLI from source.\n"
            "This gate would then pass vacuously. Either the build moved into a\n"
            "recipe (teach BUILD_RE about it) or CI stopped building the CLI,\n"
            "which is the very thing phase-431 W2 forbids.\n"
        )
        return 1

    if problems:
        sys.stderr.write(
            "check-ci-cli-from-source: %d problem(s)\n\n" % len(problems)
        )
        for p in problems:
            sys.stderr.write("  - %s\n\n" % p)
        return 1

    sys.stdout.write(
        "check-ci-cli-from-source OK -- %d from-source CLI build(s), each asserting\n"
        "its source stamp; no workflow installs the release.\n" % total_builds
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
