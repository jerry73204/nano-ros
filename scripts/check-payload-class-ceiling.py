#!/usr/bin/env python3
"""Issue 1125 — the small/large payload split has THREE spellings; they must agree.

# What this exists to stop

A subscribed type is served from the zenoh `large` class when its `rx` bound
exceeds the class split. Two producers now CLASSIFY against that number and a
third is the number the RUNTIME routes on:

  1. `NROS_MESSAGE_BOUNDS_DEFAULT_SMALL_CEILING` — cmake/NanoRosMessageBounds.cmake
     (the CMake lane's `nros_derive_message_bound_knobs`)
  2. `DEFAULT_SMALL_CLASS_CEILING` — packages/cli/rosidl-codegen/src/bounds.rs
     (the cargo-leaf lane's `nros_cli_core::leaf_payload_classes`, issue 1125)
  3. `ZPICO_SUBSCRIBER_SIZE_THRESHOLD`'s crate default —
     packages/rmw/zenoh/nros-rmw-zenoh/build.rs, which becomes
     `SUBSCRIBER_SIZE_THRESHOLD` and half of the shim's `SMALL_CLASS_CEILING`

If (1) or (2) is LARGER than (3), a type between them is classified `small` by
the derivation and routed `large` at runtime — into a class the derivation may
have just declared empty, which `alloc_payload_block` then refuses at
`create_subscription`. That is issue 0841's defect one layer up, and neither
`check-knob-delivery` nor `check-knob-fixpoint` can see it: every number is
derived correctly and delivered faithfully, and they disagree about what they
MEAN.

There is no shared storage available — cmake cannot read a Rust `const` — so
agreement is checked rather than made structural. That is the same trade
`check-ffi-struct-mirrors` makes.

The threshold has NO Kconfig row (zephyr/Kconfig says so, under
`NROS_SUBSCRIBER_BUFFER_SIZE`), which is why there is no fourth arm. If one is
ever added, add it here with it — a spelling outside this list is one nothing
holds.

# Usage

    scripts/check-payload-class-ceiling.py             # the gate (selftest first)
    scripts/check-payload-class-ceiling.py --selftest  # controls only
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# (label, file, regex with ONE numeric group)
SPELLINGS = [
    (
        "cmake NROS_MESSAGE_BOUNDS_DEFAULT_SMALL_CEILING",
        "cmake/NanoRosMessageBounds.cmake",
        r"set\(NROS_MESSAGE_BOUNDS_DEFAULT_SMALL_CEILING\s+(\d+)",
    ),
    (
        "rust DEFAULT_SMALL_CLASS_CEILING",
        "packages/cli/rosidl-codegen/src/bounds.rs",
        r"pub const DEFAULT_SMALL_CLASS_CEILING:\s*usize\s*=\s*(\d+)",
    ),
    (
        "crate default ZPICO_SUBSCRIBER_SIZE_THRESHOLD",
        "packages/rmw/zenoh/nros-rmw-zenoh/build.rs",
        r'env_usize\("ZPICO_SUBSCRIBER_SIZE_THRESHOLD",\s*(\d+)\)',
    ),
]


def read_spelling(text, pattern):
    m = re.search(pattern, text)
    return int(m.group(1)) if m else None


def collect(root):
    """[(label, value_or_None)] for every spelling."""
    out = []
    for label, rel, pattern in SPELLINGS:
        path = root / rel
        text = path.read_text(encoding="utf8", errors="ignore") if path.is_file() else ""
        out.append((label + f"  ({rel})", read_spelling(text, pattern)))
    return out


def problems(found):
    """The findings, as a list of strings. Empty means agreement."""
    out = []
    missing = [label for label, v in found if v is None]
    for label in missing:
        out.append(
            f"could not read the payload-class split from: {label}. A spelling this "
            "gate cannot find is one it cannot hold, so a move is a FAILURE and not "
            "a skip."
        )
    values = {}
    for label, v in found:
        if v is None:
            continue
        # A Kconfig `-1` is the DERIVE sentinel, not a size: it means "take the
        # derived value", so it is agreement by construction rather than a
        # number to compare. Anything else is a stated size and must match.
        if v == -1:  # noqa: PLR2004 -- the DERIVE sentinel, see above
            continue
        values.setdefault(v, []).append(label)
    if len(values) > 1:
        lines = []
        for v in sorted(values):
            for label in values[v]:
                lines.append(f"    {v}  <- {label}")
        out.append(
            "the payload-class split disagrees across its spellings:\n"
            + "\n".join(lines)
            + "\n  A derivation whose ceiling is ABOVE the runtime's "
            "`min(threshold, SUBSCRIBER_BUFFER_SIZE)` classifies a type small "
            "and routes it large -- issue 0841's defect one class up, and a "
            "SubscriberCreationFailed at registration."
        )
    return out


def selftest():
    """Both controls, on the NORMAL path: a gate that cannot fail is a comment."""
    ok = [("a", 2048), ("b", 2048), ("c", -1)]
    assert problems(ok) == [], "agreement must produce no finding"

    bad = [("a", 2048), ("b", 1024)]
    assert problems(bad), "a disagreement must be reported"
    assert "1024" in problems(bad)[0]

    gone = [("a", 2048), ("b", None)]
    assert problems(gone), "an unreadable spelling must FAIL, not skip"

    # Each pattern must actually match the shape it targets — a regex that
    # matches nothing would report "could not read", which reads as a moved
    # spelling rather than as a broken gate.
    samples = [
        "set(NROS_MESSAGE_BOUNDS_DEFAULT_SMALL_CEILING 2048 CACHE INTERNAL\n",
        "pub const DEFAULT_SMALL_CLASS_CEILING: usize = 2048;\n",
        'let t: usize = env_usize("ZPICO_SUBSCRIBER_SIZE_THRESHOLD", 2048);\n',
    ]
    for (label, _, pattern), text in zip(SPELLINGS, samples):
        assert read_spelling(text, pattern) == 2048, label
        assert read_spelling("nothing here", pattern) is None, label
    print("check-payload-class-ceiling: selftest OK")


def main():
    if "--selftest" in sys.argv:
        selftest()
        return 0
    selftest()  # on the NORMAL path too, so the controls cannot rot
    found = collect(ROOT)
    probs = problems(found)
    if probs:
        print("check-payload-class-ceiling: FAIL")
        for p in probs:
            print("  " + p)
        return 1
    stated = {v for _, v in found if v is not None and v != -1}
    print(
        f"check-payload-class-ceiling: {len(found)} spellings agree on "
        f"{stated.pop()} B"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
