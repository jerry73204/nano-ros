#!/usr/bin/env python3
"""Every `[prereq.*]` key declares a `role`, and the role is one of four.

RFC-0062 amendment 3. `role` answers "who may NAME this key", which is what
lets `nros setup --workspace` tell a dependency from a toolchain:

  package    a package's CONTENT needs it; a package.xml may name it
  workspace  a repo-level recipe needs it; no package does
  infra      emulators, cross toolchains, probes — it comes from WHERE you
             deploy, not from what your code needs
  vendor     a third-party source tree this repo builds

Why a gate rather than a convention: the field defaults to `unclassified` in
the Rust struct so it could land before all 46 entries carried one. Without
this, a new key silently inherits that default and `--workspace` reports it as
a content dependency — the exact conflation the field exists to remove.

The measurement that motivated the split: of 46 keys, exactly THREE are ever
named by a package.xml (`cargo`, `cmake`, `nros`) across 407 package.xml files.
The other 43 are provisioning facts no package's content depends on.

Run:  python3 scripts/check-prereq-roles.py [--self-test]
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INDEX = os.path.join(ROOT, "nros-sdk-index.toml")
# RFC-0062 amendment 4 (phase-435 W1) added `buildtool`: a tool the BUILDER
# shells out to. It is not `package`, because no `package.xml` names it —
# `<build_type>` already implies it — and it is not `infra`, which is about
# where you DEPLOY. Amendment 3's own table had grouped 16 such keys; the
# vocabulary simply lacked the value, so seven sat under `package` and
# `nros setup --workspace` reported `cargo` as something a workspace "names".
VALID = {"package", "workspace", "infra", "vendor", "buildtool"}


def entries(text):
    """[(key, role_or_None)] for every `[prereq.*]` block, in file order."""
    out = []
    blocks = re.split(r"^\[", text, flags=re.M)
    for b in blocks:
        m = re.match(r"prereq\.([a-z0-9._+-]+)\]", b)
        if not m:
            continue
        key = m.group(1)
        # Only the block's OWN lines: a nested `[prereq.x.y]` starts a new block,
        # and a `role =` inside prose would be a comment, not a key.
        role = None
        for line in b.split("\n")[1:]:
            if line.startswith("["):
                break
            mm = re.match(r"role\s*=\s*\"([a-z_]+)\"", line.strip())
            if mm:
                role = mm.group(1)
                break
        out.append((key, role))
    return out


def self_test():
    t = (
        '[prereq.alpha]\nrole = "package"\napt = ["a"]\n\n'
        '[prereq.beta]\nwhy = "no role here"\napt = ["b"]\n\n'
        '[prereq.gamma]\nrole = "infra"\n'
    )
    got = entries(t)
    assert got == [("alpha", "package"), ("beta", None), ("gamma", "infra")], got
    # A commented mention must not be read as a declaration.
    t2 = '[prereq.delta]\n# see role = "infra" elsewhere\napt = ["d"]\n'
    assert entries(t2) == [("delta", None)], entries(t2)
    sys.stdout.write("check-prereq-roles self-test: OK\n")


def main():
    if "--self-test" in sys.argv:
        self_test()
        return 0
    self_test()

    with open(INDEX, encoding="utf8") as fh:
        text = fh.read()
    found = entries(text)
    if not found:
        sys.stderr.write(
            "error: parsed ZERO [prereq.*] entries from nros-sdk-index.toml.\n"
            "The block shape changed; this gate would pass vacuously.\n"
        )
        return 1

    missing = [k for k, r in found if r is None]
    bad = [(k, r) for k, r in found if r is not None and r not in VALID]

    if missing or bad:
        sys.stderr.write("check-prereq-roles: %d problem(s)\n\n" % (len(missing) + len(bad)))
        for k in missing:
            sys.stderr.write(
                "  - [prereq.%s] declares no `role`.\n"
                "      Add one of: %s\n"
                "      `package` means a package.xml may name it — pick that ONLY if a\n"
                "      package's CONTENT needs it. A toolchain or emulator is `infra`:\n"
                "      it comes from where you deploy, not from a <depend>.\n\n"
                % (k, ", ".join(sorted(VALID)))
            )
        for k, r in bad:
            sys.stderr.write(
                "  - [prereq.%s] has role = %r, which is not one of %s\n\n"
                % (k, r, ", ".join(sorted(VALID)))
            )
        return 1

    counts = {}
    for _k, r in found:
        counts[r] = counts.get(r, 0) + 1
    sys.stdout.write(
        "check-prereq-roles: OK — %d key(s) (%s).\n"
        % (len(found), ", ".join("%d %s" % (v, k) for k, v in sorted(counts.items())))
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
