#!/usr/bin/env python3
"""Print the OS packages `[prereq.*]` declares for a manager — phase-413 W3.

WHY A SCRIPT AND NOT `nros setup --system`

`nros setup --system` is the user-facing verb and stays so. This exists for the
jobs that need two packages and nothing else from the toolchain: making a docs
deploy build the `nros` CLI to learn that `doxygen` is spelled `doxygen` costs
minutes to answer a question the index answers in milliseconds.

It is not a second source of truth. It reads `nros-sdk-index.toml`, the same
file the CLI reads, exactly as `check-dist-runtime-deps.py` already does — the
SSoT is the index, not any one consumer of it.

WHY IT REFUSES UNKNOWN KEYS

Silently printing nothing for a typo would install nothing and fail later at the
compiler, which is the shape RFC-0062 exists to delete. An unknown key is an
error naming the key, the same rung the `<depend>` ladder ends on.

Usage:
    prereq-packages.py --manager apt doxygen graphviz
    prereq-packages.py --self-test
"""

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
INDEX = os.path.join(ROOT, "nros-sdk-index.toml")
MANAGERS = ("apt", "dnf", "pacman", "brew")


def load(path=INDEX):
    try:
        import tomllib as toml
    except ModuleNotFoundError:  # py<3.11
        import tomli as toml
    with open(path, "rb") as fh:
        return toml.load(fh)


# Issue 1128 — the ONE placeholder a `[prereq.*]` package name may carry.
#
# `ros-{ros_distro}-rmw-zenoh-cpp` expands against $ROS_DISTRO. Kept to exactly
# one name, and `check-prereq-placeholders` refuses any other, because this is a
# second implementation of the CLI's `PrereqContext::expand` and the two can
# only stay honest while the vocabulary is small enough to hold in one line.
DEFAULT_ROS_DISTRO = "humble"


def prereq_context(env=None):
    """The values placeholders expand to, read once from the environment."""
    env = os.environ if env is None else env
    distro = (env.get("ROS_DISTRO") or "").strip()
    return {"ros_distro": distro or DEFAULT_ROS_DISTRO}


def expand(name, ctx):
    """Substitute the placeholders `ctx` knows. Mirrors `PrereqContext::expand`."""
    return name.replace("{ros_distro}", ctx["ros_distro"])


def packages_for(index, manager, keys, ctx=None):
    """The manager's packages for `keys`, in the order given, deduped.

    Raises on an unknown key, or on a key the index declares for no package
    under this manager — "declared but not for your OS" is a real answer and a
    silent empty string is not.
    """
    ctx = prereq_context() if ctx is None else ctx
    prereq = index.get("prereq", {})
    out, seen, missing, unmapped = [], set(), [], []
    for k in keys:
        entry = prereq.get(k)
        if entry is None:
            missing.append(k)
            continue
        pkgs = entry.get(manager) or []
        if not pkgs:
            unmapped.append(k)
            continue
        for p in pkgs:
            p = expand(p, ctx)
            if p not in seen:
                seen.add(p)
                out.append(p)
    if missing:
        raise SystemExit(
            f"prereq-packages: no [prereq.{missing[0]}] in nros-sdk-index.toml"
            + (f" (and {len(missing) - 1} more)" if len(missing) > 1 else "")
            + "\n  Declare it there — the index is the SSoT (RFC-0062)."
        )
    if unmapped:
        raise SystemExit(
            f"prereq-packages: [prereq.{unmapped[0]}] declares no `{manager}` package"
            + (f" (and {len(unmapped) - 1} more)" if len(unmapped) > 1 else "")
            + f"\n  Add the `{manager} = [..]` line to that entry."
        )
    return out


def self_test():
    index = {
        "prereq": {
            "doxygen": {"apt": ["doxygen"], "dnf": ["doxygen"]},
            "graphviz": {"apt": ["graphviz"]},
            "dup": {"apt": ["doxygen"]},
            "ros-rmw-zenoh-cpp": {"apt": ["ros-{ros_distro}-rmw-zenoh-cpp"]},
        }
    }
    failures = 0

    got = packages_for(index, "apt", ["doxygen", "graphviz"])
    if got != ["doxygen", "graphviz"]:
        print(f"  FAIL: order/content {got}")
        failures += 1

    # A package named by two keys is emitted once — the caller pastes this into
    # one `apt-get install` line.
    if packages_for(index, "apt", ["doxygen", "dup"]) != ["doxygen"]:
        print("  FAIL: duplicate package not deduped")
        failures += 1

    # Issue 1128 — the placeholder expands, and the default is humble.
    #
    # A dict is passed rather than `os.environ` being poked: the expansion is a
    # pure function of its context, so its test needs no environment mutation
    # (the process-global hazard issue 1101 records).
    jazzy = prereq_context({"ROS_DISTRO": "jazzy"})
    if packages_for(index, "apt", ["ros-rmw-zenoh-cpp"], jazzy) != [
        "ros-jazzy-rmw-zenoh-cpp"
    ]:
        print("  FAIL: {ros_distro} did not expand")
        failures += 1
    for env, why in (({}, "unset"), ({"ROS_DISTRO": "   "}, "blank")):
        if prereq_context(env)["ros_distro"] != DEFAULT_ROS_DISTRO:
            print(f"  FAIL: {why} $ROS_DISTRO did not default to humble")
            failures += 1
    # A name with no placeholder is returned unchanged — the other 45 keys.
    if packages_for(index, "apt", ["doxygen"], jazzy) != ["doxygen"]:
        print("  FAIL: a plain name was rewritten")
        failures += 1

    for keys, why in ((["nope"], "unknown key"), (["graphviz"], "unmapped manager")):
        manager = "apt" if why == "unknown key" else "dnf"
        try:
            packages_for(index, manager, keys)
        except SystemExit:
            pass
        else:
            print(f"  FAIL: {why} did not raise")
            failures += 1

    if failures:
        print(f"prereq-packages self-test: {failures} case(s) FAILED")
        return 1
    print("prereq-packages self-test: OK")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manager", default="apt", choices=MANAGERS)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("keys", nargs="*")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    if not args.keys:
        ap.error("name at least one [prereq.*] key")
    print(" ".join(packages_for(load(), args.manager, args.keys)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
