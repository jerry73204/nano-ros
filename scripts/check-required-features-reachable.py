#!/usr/bin/env python3
"""Issue 0652 — a `required-features` target that no recipe enables is invisible.

Cargo skips a `[[test]]` whose `required-features` are off. Silently: it is not
reported as filtered, it simply is not built. So a target behind a feature no
recipe enables is indistinguishable from a deleted one — except that it still
LOOKS like coverage when you read the tests directory, which is how one of them
sat failing without anyone noticing.

This is issue 0319's shape one level down. There the gate existed and the lane
did not run it; here the lane would run it and the TARGET is unreachable. Both
end the same way: a green line that stands for nothing.

WHAT THIS CHECKS

Every `required-features` value declared anywhere in the workspace appears in at
least one `just` recipe as a feature actually enabled. Not "is mentioned" — the
word must appear in a `--features`/`features = ` position, because `rmw` occurs
250 times in the justfiles as a substring of `rmw-zenoh`, `check-rmw-*` and
friends while being enabled as a feature exactly never.

WHY A BASELINE

Five features are unreachable today, covering nine test targets. Gating them all
at once would fail on day one and get bypassed, so they are listed as a
SHRINKING BACKLOG — the same shape `check-leaf-lockfiles` uses and says out
loud: a baseline is not a permanent exemption. What the gate buys immediately is
that the SIXTH one cannot be added silently.
"""

import re
import subprocess
import sys
from pathlib import Path
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent / "lib"))
from check_just_sources import check_just_sources

ROOT = Path(__file__).resolve().parent.parent

# Unreachable today. Remove an entry when its targets join a lane — or when the
# targets are deleted, which for an obsolete test is the more honest of the two.
#
# Nine targets hide behind these:
#   trigger-test            trigger_conditions, wake_latency
#   component-runtime-test  component_runtime, tier_filter, component_dispatch,
#                           component_param
#   loan-e2e                loan_e2e
#   phase216-substrate      dispatch_strategy
#   rmw                     custom_transport_loopback
# Shrunk 2026-08-18: `trigger-test`, `component-runtime-test` and
# `phase216-substrate` now run in `check-required-features-tests` (20 tests,
# `signal-fd-wake-test` having joined them from `nros-node` per issue 0612).
#
# Shrunk again the same day, twice and independently, emptying it:
#
#   loan-e2e  wired into `test-zpico-multisession`, which already owns
#             `ZPICO_MAX_SESSIONS=2` and its own target dir. The old entry NAMED
#             that recipe as its home and recorded "verified passing 2/2 under
#             that env" — but the wiring was never added, so the target sat in
#             no lane after all, which is the exact condition this gate exists
#             to make impossible. A baseline row is the thing to delete, not a
#             note to keep.
#   rmw       `custom_transport_loopback` joined `test-all`'s lane, which is the
#             fixture-gated one it wanted: `test-all` depends on
#             `_require-fixtures`, `check-required-features-tests` does not. It
#             was never broken, only unassessable — on rebuilt fixtures it
#             passes (3 TCP hops, 3 messages delivered, 0.58 s).
#
# BASELINE is now EMPTY, which is what a shrinking backlog is for. Do not add a
# row back without a dated reason and the lane it is waiting on: the promise
# this file made when it went in with five was that the sixth could not appear
# silently, and an empty set is the strongest form of that.
# The `required-features` half of this gate is EMPTY and stays that way.
#
# The FILE-CFG half arrived 2026-08-24 (issue 0779) with six unreachable
# features covering fifteen files, and the same reasoning applies as when this
# file went in with five: switching them all on in the commit that found them
# fails on day one and gets bypassed. `lending` was wired immediately because
# it needed only a lane (48 tests vs 44, green). The other five want more than
# a flag — `posix-c-port` and `c-stub-test` build C stubs, `unix-mock` wants a
# loopback harness, `bridge-stub` and `link-custom` want link-time setups — so
# they are a DATED BACKLOG with an issue on them, not an exemption. Delete a
# row when its files join a lane, or when the files go.
# EMPTY since 2026-08-25 (issue 0779): every gating feature is now reachable
# from a recipe. Keep it that way — a name added here is a test nobody runs.
BASELINE: set[str] = set()

# `--features a,b`, `--features "a b"`, `features = ["a"]`, `--all-features`.
FEATURE_CONTEXT = re.compile(
    r"--features[= ]\s*\"?([A-Za-z0-9_,\- ]+)\"?|features\s*=\s*\[([^\]]*)\]"
)


def declared_required_features() -> dict[str, list[str]]:
    """feature -> manifests declaring a target behind it."""
    out: dict[str, list[str]] = {}
    listing = subprocess.run(
        ["git", "-C", str(ROOT), "ls-files", "--", "*Cargo.toml"],
        capture_output=True, text=True, check=True,
    ).stdout.split()
    for rel in listing:
        try:
            text = (ROOT / rel).read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.startswith("#") or "required-features" not in stripped:
                continue
            for feat in re.findall(r'"([^"]+)"', stripped):
                out.setdefault(feat, []).append(rel)
    return out


# The SAME lie, a different mechanism: a test FILE whose first attribute is
# `#![cfg(feature = "x")]` compiles to an empty binary when `x` is off. Cargo
# builds it, nextest runs it, and it reports zero tests — greener than
# `required-features`, which at least does not build. `nros-rmw-cffi`'s
# `loan_fallback` / `loan_native` sat like that behind `lending`: 4 tests never
# run, and by 2026-08-24 they no longer COMPILED (a W3.d rename left
# `NROS_RMW_RET_ERROR` unimported and `has_data` at its old one-argument
# shape), which nothing noticed because nothing built them.
FILE_CFG = re.compile(r'#!\[cfg\((.*?)\)\]', re.S)


def declared_file_cfg_features() -> dict[str, list[str]]:
    """feature -> test files gated on it by a crate-level `#![cfg(...)]`."""
    out: dict[str, list[str]] = {}
    listing = subprocess.run(
        ["git", "-C", str(ROOT), "ls-files", "--", "packages/*/tests/*.rs",
         "packages/*/*/tests/*.rs", "packages/*/*/*/tests/*.rs"],
        capture_output=True, text=True, check=True,
    ).stdout.split()
    for rel in listing:
        try:
            text = (ROOT / rel).read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        # Only the crate-level attribute, which must precede any item.
        head = text[:2000]
        for m in FILE_CFG.finditer(head):
            for feat in re.findall(r'feature\s*=\s*"([^"]+)"', m.group(1)):
                out.setdefault(feat, []).append(rel)
    return out


def features_enabled_by_recipes() -> set[str]:
    enabled: set[str] = set()
    # The gate recipes moved into `just/check/*.just`, and a non-recursive glob
    # sees the index alone -- which reports every laned target as reachable
    # from no recipe, a real-looking answer about the wrong file set.
    #
    # NOT `rglob`: that is a filesystem WALK, which `check-no-tracked-file-find`
    # forbids for the reason it measured (7m36s -> 0.8s). The import closure is
    # the right set anyway -- it is what `just` reads.
    files = [ROOT / "justfile"] + sorted((ROOT / "just").glob("*.just"))
    files += [Path(p) for p in check_just_sources(str(ROOT)) if Path(p) not in files]
    for path in files:
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        if "--all-features" in text:
            # Enables everything, including targets nobody named.
            return {"*"}
        for m in FEATURE_CONTEXT.finditer(text):
            blob = m.group(1) or m.group(2) or ""
            for feat in re.split(r"[,\s\"]+", blob):
                if feat:
                    enabled.add(feat)
    return enabled


def main() -> int:
    declared = declared_required_features()
    if not declared:
        sys.stderr.write(
            "[FAIL] no `required-features` targets found — this gate would pass\n"
            "       vacuously. Either they are all gone (delete this check) or the\n"
            "       manifest scan broke.\n"
        )
        return 1

    file_gated = declared_file_cfg_features()
    enabled = features_enabled_by_recipes()
    wildcard = "*" in enabled

    combined = dict(declared)
    for feat, files in file_gated.items():
        combined.setdefault(feat, []).extend(files)

    unreachable = {
        f: m for f, m in combined.items() if not wildcard and f not in enabled
    }
    new = {f: m for f, m in unreachable.items() if f not in BASELINE}
    fixed = sorted(BASELINE - set(unreachable))

    rc = 0
    if new:
        sys.stderr.write(
            "[FAIL] `required-features` value(s) that no recipe enables (issue 0652):\n"
        )
        for feat, manifests in sorted(new.items()):
            sys.stderr.write(f"         {feat}  ({', '.join(sorted(set(manifests)))})\n")
        sys.stderr.write(
            "\n       Cargo skips such a target SILENTLY — it is not reported as\n"
            "       filtered, it is simply never built, so it looks like coverage\n"
            "       while running nothing. Put it in a lane, or delete the target.\n"
        )
        rc = 1
    if fixed:
        sys.stderr.write(
            "[FAIL] baselined feature(s) now reachable — remove them from BASELINE\n"
            "       in this script; it is a shrinking backlog, not an exemption:\n"
        )
        for feat in fixed:
            sys.stderr.write(f"         {feat}\n")
        rc = 1

    if rc == 0:
        n = len(combined)
        sys.stderr.write("")
        print(
            f"required-features reachable: OK ({n} gating feature(s): "
            f"{len(declared)} via `required-features`, {len(file_gated)} via a "
            f"test file's `#![cfg(feature)]`; {len(BASELINE)} baselined backlog)"
        )
    return rc


if __name__ == "__main__":
    sys.exit(main())
