#!/usr/bin/env python3
"""The ONE way to read the gate recipes, now that they live in several files.

`just/check.just` is an index: the ~290 gate recipes sit in `just/check/*.just`
and are brought in with `import`, which MERGES definitions rather than
namespacing them. So `just check <name>` is unchanged, but any script that
wants to see a gate's recipe or its BODY must read the closure, not the index.

Six scripts read it. Before this helper existed, two had been fixed by hand and
the other four still read the index alone -- and they did not fail loudly, they
failed QUIETLY and differently:

    check-gate-lists                 derived 8 gates instead of 234
    check-gate-selftests             found 0 gate scripts
    check-preconditions-provisioned  "STALE tool row `setup-threadx`"
    check-test-scripts-have-callers  every test script "has no caller"
    check-required-features-reachable  every laned target "in no lane"

Each of those is a gate reporting a real-looking problem about the wrong thing,
which is worse than a crash. One helper, so the next reader cannot get a
smaller closure than the one `just` sees.

Never glob `just/check/*.just`: the set is what the index actually IMPORTS. A
topic file that is present but not imported contributes nothing to `just`, and
counting it here would make a gate visible to the checks and invisible to the
runner.
"""

import os
import re

_IMPORT = re.compile(r"^import\s+'([^']+)'", re.MULTILINE)


def check_just_sources(root):
    """`just/check.just` and every topic file it imports, in import order."""
    index = os.path.join(root, "just", "check.just")
    if not os.path.isfile(index):
        return []
    with open(index, encoding="utf8") as fh:
        text = fh.read()
    out = [index]
    for rel in _IMPORT.findall(text):
        path = os.path.normpath(os.path.join(os.path.dirname(index), rel))
        if os.path.isfile(path):
            out.append(path)
    return out


def check_just_text(root):
    """Every gate recipe as one string, for scripts that grep rather than parse."""
    parts = []
    for path in check_just_sources(root):
        with open(path, encoding="utf8") as fh:
            parts.append(fh.read())
    return "\n".join(parts)


def self_test():
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        os.makedirs(os.path.join(tmp, "just", "check"))
        with open(os.path.join(tmp, "just", "check.just"), "w") as fh:
            fh.write("import 'check/a.just'\nimport 'check/missing.just'\nfast:\n    @true\n")
        with open(os.path.join(tmp, "just", "check", "a.just"), "w") as fh:
            fh.write("gate-a:\n    @python3 scripts/check-a.py\n")
        got = check_just_sources(tmp)
        assert len(got) == 2, got            # the missing import is skipped, not fatal
        text = check_just_text(tmp)
        assert "gate-a:" in text and "fast:" in text
        # An un-imported topic file must NOT contribute.
        with open(os.path.join(tmp, "just", "check", "orphan.just"), "w") as fh:
            fh.write("gate-orphan:\n    @true\n")
        assert "gate-orphan" not in check_just_text(tmp)
    print("check_just_sources self-test: OK")


if __name__ == "__main__":
    self_test()
