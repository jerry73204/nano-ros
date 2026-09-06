#!/usr/bin/env python3
"""issue 1137 — if the PEER's bus is pinned, OUR side's must be too.

Issue 1009 confined every DDS interop peer to loopback, because a participant on
the LAN is otherwise a peer: `ros2_env_setup_rmw_with_domain` exports a
`FASTRTPS_DEFAULT_PROFILES_FILE` / `CYCLONEDDS_URI` naming a whitelist profile.
That helper builds a `source setup.bash && export …` STRING, so it reaches
exactly one kind of process — a host `ros2` peer. Our own side of every pair is
a bare `std::process::Command`, and it got nothing.

The issue's own headline says what that costs: **pin both sides or neither; half
is no discovery.** It was measured for Fast-DDS (0 of 15 with the variable on one
side) and then half-applied. Two days later `cyclone_enumerates_a_stock_ros2_node`
— which had PASSED live on 2026-08-30, issue 0927 — enumerated one node, itself,
against a stock talker that was up. The stock talker was pinned to `127.0.0.1`
with `AllowMulticast=false`; the nano-ros probe kept Cyclone's default interface
pick. Neither side's SPDP could reach the other, and the symptom is
indistinguishable from a broken `ros_discovery_info` reader — which is what it
was filed as, for the second time.

What this checks
----------------
A tracked test source that starts a HOST ROS 2 DDS peer (the helpers that pin,
listed in PINNING_PEERS) must also apply the matching pin to the processes it
spawns itself — `nros_tests::dds_isolation::apply_to_command` /
`apply_cyclone_config` / `apply_fastdds_profile`.

Two things are structurally out of scope rather than allowlisted:

  1. The implementation files themselves (`src/dds_isolation.rs`, `src/ros2.rs`,
     `src/ros_env.rs`). Those ARE the procedure.
  2. `DockerRosEnv` peers. A container has its own mount namespace, so it cannot
     read a host profile path, and those pairs are symmetric-UNPINNED today.
     Pinning our half of one would CREATE this bug rather than fix it — so the
     detector keys on `HostRosEnv` / the host `Ros2*Process` helpers, never on
     the `Middleware` enum both backends share.

Anything else legitimate is in ALLOWLIST, path-keyed, one reason each.

Dependency-free Python 3.10, house style per `scripts/check-ros-env-spelling.py`.
"""

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TESTS = "packages/testing/nros-tests/"

# Helpers that start a host ROS 2 peer whose env is pinned by
# `ros2_env_setup_rmw_with_domain`. Every one of these funnels through it.
PINNING_PEERS = (
    "Ros2DdsProcess::",
    "ros2_env_setup_dds",
    "ros2_env_setup_cyclonedds",
    "ros2_env_setup_rmw_with_domain",
)

# `HostRosEnv` is only a pinning peer for a DDS middleware — a zenoh peer is
# keyed by locator, not by a bus, and nothing pins it. Requires BOTH tokens,
# because `DockerRosEnv` names the same `Middleware` variants and must not match.
HOST_ENV = "HostRosEnv::new"
DDS_MIDDLEWARE = ("Middleware::Cyclonedds", "Middleware::FastRtps")

# Our side of the pair, applied to a `Command`.
APPLIES_PIN = (
    "dds_isolation::apply_to_command",
    "dds_isolation::apply_cyclone_config",
    "dds_isolation::apply_fastdds_profile",
)

# The files that ARE the procedure — see docstring rule 1.
SANCTIONED = {
    TESTS + "src/dds_isolation.rs",
    TESTS + "src/ros2.rs",
    TESTS + "src/ros_env.rs",
}

ALLOWLIST = {
    TESTS
    + "tests/xrce_ros2_interop.rs": (
        "The nano-ros side is an XRCE client, not a DDS participant: it speaks "
        "XRCE to the micro-XRCE Agent, and the Agent is the process that joins "
        "the peer's DDS bus. The Agent IS pinned, by "
        "`fixtures::xrce_agent`'s `apply_fastdds_profile` (issue 1009). "
        "Pinning the XRCE client would export a variable it cannot read, which "
        "is the `ROS_LOCALHOST_ONLY` mistake one layer over."
    ),
}

REMEDY = (
    "call `nros_tests::dds_isolation::apply_to_command(&mut cmd)` on every "
    "Command this file spawns that joins the peer's DDS bus"
)
CONSEQUENCE = (
    "the peer is confined to 127.0.0.1 with AllowMulticast=false and our side "
    "keeps the middleware's default interface pick, so neither one's discovery "
    "traffic reaches the other. The cell then reports an EMPTY graph / no "
    "delivery, which reads as a backend defect (issues 0927 and 1137 were both "
    "filed as exactly that)."
)


def tracked_files():
    out = subprocess.run(
        ["git", "ls-files", TESTS + "tests", TESTS + "src", TESTS + "bins"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    return [p for p in out.stdout.splitlines() if p.endswith(".rs")]


def starts_a_pinned_peer(text):
    """Does this source start a HOST ROS 2 peer whose bus issue 1009 pinned?"""
    for token in PINNING_PEERS:
        if token in text:
            return token
    if HOST_ENV in text:
        for mw in DDS_MIDDLEWARE:
            if mw in text:
                return f"{HOST_ENV} + {mw}"
    return None


def applies_the_pin(text):
    return any(token in text for token in APPLIES_PIN)


def self_test(quiet=False):
    bad = []

    # A host DDS peer with no pin on our side is the defect.
    defect = 'Ros2DdsProcess::topic_echo_cyclonedds_with_domain(...);\nCommand::new(bin)'
    if starts_a_pinned_peer(defect) is None or applies_the_pin(defect):
        bad.append("the 1137 shape is not detected")

    # The same file, fixed.
    fixed = defect + "\nnros_tests::dds_isolation::apply_to_command(&mut cmd);"
    if not applies_the_pin(fixed):
        bad.append("a fixed file is still reported")

    # A DOCKER peer must NOT be treated as a pinned peer: those pairs are
    # symmetric-unpinned, and pinning our half would create the bug. This is the
    # case that makes the `HostRosEnv`/`Middleware` distinction load-bearing.
    docker = 'DockerRosEnv::new(&ed, Middleware::Cyclonedds { domain_id: d });'
    if starts_a_pinned_peer(docker) is not None:
        bad.append("a DockerRosEnv peer was read as a pinned peer")

    # A zenoh host peer is not a DDS bus at all.
    zenoh = "HostRosEnv::new(distro, Middleware::zenoh_default());"
    if starts_a_pinned_peer(zenoh) is not None:
        bad.append("a zenoh HostRosEnv peer was read as a pinned peer")

    # Every allowlisted / sanctioned path still exists — a stale entry is a hole
    # that reads like a decision.
    for path in sorted(SANCTIONED | set(ALLOWLIST)):
        if not (ROOT / path).is_file():
            bad.append(f"{path} is named here but does not exist")

    if bad:
        for b in bad:
            sys.stderr.write("check-dds-isolation-symmetry --self-test: " + b + "\n")
        return 2
    if not quiet:
        print("check-dds-isolation-symmetry --self-test: OK (4 case(s), "
              f"{len(SANCTIONED)} sanctioned, {len(ALLOWLIST)} allowlisted)")
    return 0


def main():
    if "--self-test" in sys.argv:
        return self_test()
    if self_test(quiet=True) != 0:
        return 2

    findings = []
    peers = 0
    for path in tracked_files():
        if path in SANCTIONED:
            continue
        full = ROOT / path
        if not full.is_file():
            continue
        try:
            text = full.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        token = starts_a_pinned_peer(text)
        if token is None:
            continue
        peers += 1
        if path in ALLOWLIST or applies_the_pin(text):
            continue
        findings.append((path, token))

    print(
        f"DDS isolation symmetry: {peers} file(s) start a pinned host ROS 2 DDS "
        f"peer, {len(ALLOWLIST)} allowlisted"
    )

    if findings:
        print(
            "\n[FAIL] a pinned ROS 2 DDS peer with an UNPINNED nano-ros side:",
            file=sys.stderr,
        )
        for path, token in findings:
            print(f"  - {path}: starts a peer via `{token}`", file=sys.stderr)
        print(f"\n  WHAT TO DO: {REMEDY}", file=sys.stderr)
        print(f"\n  WHY IT MATTERS: {CONSEQUENCE}", file=sys.stderr)
        return 1

    print("Every pinned peer has a pinned partner.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
