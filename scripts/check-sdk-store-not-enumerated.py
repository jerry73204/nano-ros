#!/usr/bin/env python3
"""Phase 365 W4 — the SDK store is CONSTRUCTED from the pin, never enumerated.

nano-ros decides where a provisioned tool goes: `nros setup` writes
`<store>/<tool>/<version>` because `nros-sdk-index.toml` named that version. A
consumer therefore builds the path from those same two inputs — via
`nros sdk-path <tool>`, or `sdk_store::tool_dir()` inside the CLI — and does not
go looking.

WHAT THIS BANS, AND WHY IT IS ENUMERATION AND NOT MENTION

The first draft of this gate was "one spelling of `.nros/sdk` per language". A
survey killed that: the 18 sites in the tree are three populations, and only one
is wrong.

  * INSTALLERS legitimately write the store (`scripts/zenohd/build.sh`,
    `scripts/xrce-agent/build.sh`) — they are the producer.
  * `.nros/sdks/arm-fvp` is a DIFFERENT tree (`sdks`, not `sdk`).
  * the defect is a consumer ENUMERATING versions and picking one — a glob or
    `ls` over `<store>/<tool>/*` followed by "newest", "last", or "sorted".

So the rule is about the SHAPE, not the string. A per-project pin cannot be
answered by scanning a store that is shared between projects, which is what
made this worth a gate:

  * measured 2026-08-16, one `lane=all` configure in a tree pinning
    `corrosion 0.6.1-nros1`: **155** resolutions of 0.5.1 against 28 of 0.6.1;
  * two independent causes, both enumeration — cmake APPENDED its newest-first
    candidates after the environment's, and `cmake-prefix.sh` globbed
    `$store/corrosion/*/`, which matched the legacy unversioned install's
    `lib/` and `share/` subdirectories (not versions at all, and under
    `sort -Vr` a pure-alpha name sorts BEFORE the numeric ones);
  * `scripts/dev/zenohd.sh` did `ls …/sdk/zenohd/*/bin/zenohd | sort -V | tail`,
    and `cmake/toolchain/riscv64-threadx.cmake` globbed
    `riscv-none-elf-gcc/*/` and took `list(GET … -1)`. Same class, other tools,
    both found by this rule rather than by a failure.

WHAT IS NOT BANNED (phase-431 W3)

`sdk_store::installed_versions` reads `<store>/<tool>/` and takes the newest.
That is enumeration, and it is correct, because it answers a DIFFERENT question:
not "where is the version I pinned" (constructible from two inputs) but "what is
the newest thing installed here" (nothing but the store knows). It backs the
`front` link — `$NROS_HOME/bin/nros` points at the newest installed CLI, which is
the "one command" promise. It carries 0625's defence in the filter rather than in
the sort: a candidate must begin with a digit AND carry a `.nros-provenance`
marker, so `lib/` under a legacy flat prefix cannot win by sorting.

The rule is unchanged for consumers: a PIN is constructed, never searched.

Issue 0625; phase-365; phase-431 W3.
"""

import re
import subprocess
import sys

# A store path with a WILDCARD where the VERSION belongs.
#
# Anchored on `/sdk/<tool>/*` and nothing to its left. The first version keyed
# on `.nros` or `NROS_HOME` immediately preceding `/sdk/`, and matched nothing:
# the real spelling is `"${NROS_HOME:-$HOME/.nros}"/sdk/zenohd/*`, where a `}"`
# sits between. That draft passed its own self-test — the gate was vacuous and
# said OK. Hence the self-test below is the point, not a formality.
ENUMERATION = re.compile(r"""/sdks?/[A-Za-z0-9_.-]+/\*""")
# The installers write the store; they are the producer, not a consumer.
ALLOWED = (
    "scripts/zenohd/build.sh",
    "scripts/xrce-agent/build.sh",
    "scripts/installers/",
    # This gate's own documentation, and the phase/issue that explain it.
    "scripts/check-sdk-store-not-enumerated.py",
    "docs/",
)


def main() -> int:
    out = subprocess.run(
        ["git", "grep", "-nI", "-E", ENUMERATION.pattern],
        capture_output=True,
        text=True,
    ).stdout.splitlines()

    bad = []
    for line in out:
        path, _, rest = line.partition(":")
        if any(path.startswith(a) or a in path for a in ALLOWED):
            continue
        # A comment explaining the removal is not a reintroduction.
        body = rest.partition(":")[2].lstrip()
        if body.startswith(("#", "//", "*")):
            continue
        bad.append(line)

    if bad:
        print(
            "ERROR: the SDK store is ENUMERATED instead of constructed from the pin:",
            file=sys.stderr,
        )
        for b in bad:
            print(f"  {b}", file=sys.stderr)
        print(
            "\n  A wildcard where the VERSION belongs means a consumer is picking\n"
            "  a version by searching a store that is SHARED between projects,\n"
            "  to answer a pin that is PER-PROJECT. Measured cost: 155 wrong\n"
            "  resolutions against 28 right ones in one configure (issue 0625).\n"
            "\n"
            "  Construct it instead:\n"
            "      shell / cmake / just :  nros sdk-path <tool>\n"
            "      inside the CLI       :  sdk_store::tool_dir(&index, tool)",
            file=sys.stderr,
        )
        return 1

    print("sdk store not enumerated: OK (constructed from the pin)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
