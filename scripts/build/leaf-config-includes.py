#!/usr/bin/env python3
"""Issue 0463 — check that every tracked leaf `.cargo/config.toml` `include`
target exists, before cargo turns their absence into an unreadable trace.

Why this needs a guard at all
-----------------------------
A tracked leaf config reaches sync-generated content through `include`:

    include = ["../../../../../nros-patch.toml", "nros-managed-patch.toml"]

Both targets are GITIGNORED and written by `nros sync` — the central one since
#272, the sidecar since #457. Neither exists in a fresh clone.

Two comments in `cmd/ws.rs` justified that arrangement with "cargo ignores a
missing `include` SILENTLY". Measured on cargo 1.97.1, it does the opposite: a
missing target is a hard error raised during MANIFEST PARSE, so `cargo
metadata`, `cargo tree` and every gate that reads the leaf fail too, four
frames deep, naming a path with no mention of sync:

    error: failed to parse manifest at `<leaf>/Cargo.toml`
    Caused by: could not load Cargo configuration
    Caused by: failed to load config include `nros-managed-patch.toml` from ...
    Caused by: No such file or directory (os error 2)

A fresh clone cannot build these leaves in any case — their patches point at
`generated/` message crates that only sync produces from the USER's ament
install. So the fix is not to make the include optional; it is to say
"run `nros sync`" at the seam, once, instead of once per leaf in cargo's words.

A SECOND shape of the same failure
----------------------------------
An `include` target is not the only sync-produced path a manifest can name. A
workspace member can path-dep a crate under `generated/` too:

    freertos_realtime_entry_nros_selection = { path = "../../generated/nros-selection/..." }

and cargo fails identically — during manifest parse, four frames deep, naming a
path and never `nros sync`. Issue 0474 wired this guard ahead of `format` for
exactly that reason, but the guard only read `include` targets, so
`native::format` still died on an unsynced leaf while the guard reported "OK"
(realtime-rust, 2026-08-24). A gate whose coverage is narrower than the rule it
enforces reads as coverage — so both shapes are checked here, in ONE gate,
rather than growing a second spelling elsewhere.

Only path deps pointing INTO a `generated/` directory are checked: those have a
known producer (`nros sync`) and a known remedy. A path dep that is merely typo'd
is cargo's to report.

Exit 0 when every include and every generated path dep resolves, 1 otherwise.
"""

import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# `include` is a top-level key, so it precedes the first table header. Scanning
# only that prefix keeps a `[target.…]` value that happens to contain the word
# from being mistaken for it.
INCLUDE_RE = re.compile(r'^\s*include\s*=\s*(\[[^\]]*\])', re.M | re.S)
STR_RE = re.compile(r'"([^"]*)"')

# A path dep whose target lives under a `generated/` directory — the shape
# `nros sync` produces. Matches `path = "…/generated/…"` in either the inline
# table or the expanded form.
GENERATED_PATH_RE = re.compile(r'path\s*=\s*"([^"]*\bgenerated/[^"]*)"')

# ...with ONE exception, and it is about the PRODUCER rather than the shape.
# `generated/px4_msgs` is not written by `nros sync` — issue 0510 records that
# px4_msgs is not an ament package, so only `nros generate-px4-msgs` can emit it,
# from the PX4 `.msg` tree, and only `just px4 build-fixtures` runs that.
#
# ISSUE 1114 — the exemption's PREDICATE was wrong, not its existence. It read
# "require it only when the producer COULD have run", keyed on the PX4 `.msg`
# tree being present. That is a fact about the HOST, and the question is about
# the LANE: on a machine with PX4 provisioned the exemption lifted and the guard
# demanded `generated/px4_msgs` from every lane, including the ones that never
# build a px4 leaf. px4 has ZERO rows in `examples/fixtures.toml` — it is in no
# lane at all — so tier 2 died in `just build tier2` on three leaves it does not
# build, and the scheduled `run-matrix` lane produced no verdict.
#
# That is CLAUDE.md's affordability rule ("a gate in an affordability tier may
# only resolve artifacts the JOB ITSELF builds", `check-lane-contracts`) broken
# one level down: the gate resolved an artifact whose producer is a lane the job
# never invokes.
#
# So it is never REQUIRED here, and always REPORTED. The one lane that consumes
# these leaves PRODUCES the directory first -- `just px4 build-fixtures`
# generates into all three before it builds any of them -- so a lane that has
# run the producer passes on the directory EXISTING, not on this guard's say-so,
# and a lane that has not run it was never going to parse those manifests.
# Reporting keeps the diagnosis: a bare `cargo metadata` over a px4 leaf still
# gets cargo's raw manifest-parse error, and this note is what explains it.
#
# Same reasoning the `packages/cli/testing_workspaces/**` exclusion above rests
# on: match the gate to the leaves the rule covers.
PX4_PRODUCED_RE = re.compile(r"\bgenerated/px4_msgs\b")


def px4_msg_tree() -> Path:
    """The PX4 `.msg` tree `just px4 build-fixtures` generates px4_msgs from."""
    env = os.environ.get("PX4_AUTOPILOT_DIR")
    base = Path(env) if env else ROOT / "third-party" / "px4" / "PX4-Autopilot"
    return base / "msg"


def includes(text: str):
    """The `include` array's entries, or [] when the key is absent."""
    head = text.split("\n[", 1)[0]
    m = INCLUDE_RE.search(head)
    return STR_RE.findall(m.group(1)) if m else []


def main() -> int:
    out = subprocess.run(
        ["git", "ls-files", "*/.cargo/config.toml", ".cargo/config.toml"],
        cwd=ROOT, capture_output=True, text=True, check=True,
    ).stdout

    missing = []   # (leaf_config, unresolved_target)
    checked = 0
    for rel in out.split():
        cfg = ROOT / rel
        try:
            text = cfg.read_text()
        except OSError:
            continue
        checked += 1
        for entry in includes(text):
            # Cargo resolves a relative include against the INCLUDING file's
            # directory — `.cargo/`, not the leaf root.
            target = (cfg.parent / entry).resolve()
            if not target.is_file():
                missing.append((rel, entry))

    # Second shape: a manifest path-deps a crate under `generated/` that sync
    # has not produced yet. Same seam, same remedy, same failure text from cargo.
    gen_missing = []   # (manifest, unresolved_path)
    gen_unprovisioned = []   # (manifest, path) — producer absent, not required
    px4_available = px4_msg_tree().is_dir()
    # Scoped to `examples/**` — the leaves the lanes this guard fronts actually
    # walk (`native::format` enumerates exactly this set). `packages/cli/
    # testing_workspaces/**` also path-deps `generated/`, but those are CLI test
    # fixtures that the tests using them sync on demand, so requiring them here
    # would fail `just format` for trees that are not broken. Matching the gate
    # to the RULE means matching it to the leaves the rule covers — widening it
    # past them is the same defect in the other direction.
    manifests = subprocess.run(
        ["git", "ls-files", "examples/**/Cargo.toml"],
        cwd=ROOT, capture_output=True, text=True, check=True,
    ).stdout.split()
    for rel in manifests:
        man = ROOT / rel
        try:
            text = man.read_text()
        except OSError:
            continue
        for dep_path in GENERATED_PATH_RE.findall(text):
            target = (man.parent / dep_path / "Cargo.toml").resolve()
            if target.is_file():
                continue
            if PX4_PRODUCED_RE.search(dep_path):
                # Issue 1114 — reported, never required. See the note above.
                gen_unprovisioned.append((rel, dep_path))
                continue
            gen_missing.append((rel, dep_path))

    if gen_unprovisioned:
        # Say what was exempted and why. A gate that narrows its own scope has
        # to report the narrowing, or "OK" overstates what was checked.
        print(
            f"note: {len(gen_unprovisioned)} path dep(s) into `generated/px4_msgs` "
            f"not required by this lane (issue 1114)."
        )
        print("      They are produced by `just px4 build-fixtures` (via "
              "`nros generate-px4-msgs`), not by `nros sync`, and px4 has no "
              "`examples/fixtures.toml` row — so no tier lane builds these leaves.")
        if px4_available:
            print("      PX4 IS provisioned here, so `just px4 build-fixtures` "
                  "would produce them.")
        else:
            print(f"      No PX4 `.msg` tree at {px4_msg_tree()}; provision it "
                  "with `just setup px4` if you need those leaves.")

    if not missing and not gen_missing:
        print(
            f"leaf config includes OK ({checked} tracked configs, "
            f"{len(manifests)} manifests scanned for generated path deps)"
        )
        return 0

    if gen_missing and not missing:
        leaves = sorted({m[0] for m in gen_missing})
        print(
            f"error: {len(gen_missing)} path dep(s) into `generated/` do not "
            f"exist, across {len(leaves)} manifest(s).\n",
            file=sys.stderr,
        )
        for rel, dep in gen_missing[:5]:
            print(f"  {rel}\n      path -> {dep}  (absent)", file=sys.stderr)
        if len(gen_missing) > 5:
            print(f"  … and {len(gen_missing) - 5} more", file=sys.stderr)
        print(
            "\n`generated/` is produced by `nros sync` from the USER's own message "
            "packages and is\nnever committed, so a clone never has it. Cargo "
            "treats the missing manifest as a HARD\nerror during manifest parse — "
            "`cargo metadata`, `cargo fmt` and every gate that reads\nthe leaf fail "
            "before anything mentions sync.\n\n  source ./activate.sh && nros sync"
            "\n\nBypass with NROS_SKIP_LEAF_INCLUDE_CHECK=1.\nSee docs/issues/0463-* "
            "and 0474.",
            file=sys.stderr,
        )
        return 1

    leaves = sorted({m[0] for m in missing})
    print(
        f"error: {len(missing)} unresolved `include` target(s) across "
        f"{len(leaves)} of {checked} tracked .cargo/config.toml files.\n",
        file=sys.stderr,
    )
    for rel, entry in missing[:5]:
        print(f"  {rel}\n      include -> {entry}  (absent)", file=sys.stderr)
    if len(missing) > 5:
        print(f"  … and {len(missing) - 5} more", file=sys.stderr)
    print(
        "\nThese targets are gitignored and generated by `nros sync`; a clone "
        "never has them.\nCargo treats a missing include as a HARD error during "
        "manifest parse, so the\naffected leaves cannot even be read until sync "
        "runs.\n\n  source ./activate.sh && nros sync\n\n"
        "Bypass with NROS_SKIP_LEAF_INCLUDE_CHECK=1 (the cargo error will then "
        "be yours to read).\nSee docs/issues/0463-*.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
