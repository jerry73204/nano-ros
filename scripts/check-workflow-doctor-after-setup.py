#!/usr/bin/env python3
"""A job asserts its runner's labels AFTER provisioning, never before.

phase-413 W2. `runner-doctor.sh` checks
`$root/build/qemu/bin/qemu-system-arm` FIRST -- "the project-local patched
build is the PRIMARY path (phase-143) ... a host with it needs nothing from
the system" -- and falls back to the system qemu only when that is absent.

That path is inside the CHECKOUT, and `just setup tier2` is what creates it:
the tier-2 preset includes the `qemu` module, whose `setup` ends in
`just qemu setup-qemu`. So a job that runs the doctor BEFORE `just setup` asks
the host for something the next step supplies. On `run-matrix` that killed
every run at step 2 against a system qemu 6.2, having provisioned and tested
nothing.

WHY A GATE AND NOT THREE FIXES

The defect had three sites and was fixed at ONE. #279 corrected
`run-matrix.yml`; the sibling fixes for `build-wide.yml` and `queue.yml` were
written the same week, sat on branches whose pull requests merged without
them, and were still broken on `main` months later -- found only by auditing
local branches for unlanded commits. Nothing was watching, so nothing said so.

That is the issue-0196 rule: a class fixed site-by-site regrows, and the gate
is what makes the sweep stick. A fourth lane added tomorrow gets the same
answer without anyone remembering this.

WHAT IS CHECKED

Within one job, if a step invokes `runner-doctor` and another invokes
`just setup`, the doctor step must come after the FIRST setup step.

Only ordering WITHIN a job is checked, because that is the only place the
dependency exists -- `build/qemu` is created in the job's own checkout, and
nothing carries it between jobs.

A job with a doctor and no `just setup` is fine: there is nothing to
provision, so there is nothing to assert too early. A genuinely mislabeled
runner still fails either way -- one step later, and `setup` would have failed
on it first.

Run:  python3 scripts/check-workflow-doctor-after-setup.py [--self-test]
"""

import argparse
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORKFLOWS = os.path.join(ROOT, ".github", "workflows")

# `./scripts/ci/runner-doctor.sh <labels>` and `just runner-doctor <labels>`
# are the two spellings a job uses to assert its labels.
DOCTOR = re.compile(r"\brunner-doctor\b")
# `just setup <scope>` -- the provisioning verb (phase-411: a CI job is
# `just setup <scope>` then one command a developer can type).
SETUP = re.compile(r"\bjust\s+setup\b")


def step_kinds(steps):
    """(doctor indices, setup indices) for one job's step list."""
    doctor, setup = [], []
    for i, step in enumerate(steps or []):
        run = (step or {}).get("run") or ""
        if DOCTOR.search(run):
            doctor.append(i)
        if SETUP.search(run):
            setup.append(i)
    return doctor, setup


def violations(steps):
    """Doctor indices that precede the first `just setup` in the same job."""
    doctor, setup = step_kinds(steps)
    if not doctor or not setup:
        return []
    first_setup = min(setup)
    return [i for i in doctor if i < first_setup]


def load_workflows():
    import yaml

    docs = []
    for path in sorted(glob.glob(os.path.join(WORKFLOWS, "*.yml"))):
        with open(path, encoding="utf8") as fh:
            docs.append((os.path.basename(path), yaml.safe_load(fh)))
    return docs


def scan(docs):
    """(offenders, number of jobs that have BOTH a doctor and a setup)."""
    bad, pairs = [], 0
    for name, doc in docs:
        for job_name, job in ((doc or {}).get("jobs") or {}).items():
            steps = (job or {}).get("steps") or []
            doctor, setup = step_kinds(steps)
            if doctor and setup:
                pairs += 1
            for i in violations(steps):
                step_name = (steps[i] or {}).get("name") or f"step {i}"
                setup_name = (steps[min(setup)] or {}).get("name") or f"step {min(setup)}"
                bad.append((name, job_name, i, step_name, min(setup), setup_name))
    return bad, pairs


def self_test():
    def steps(*runs):
        return [{"name": f"s{i}", "run": r} for i, r in enumerate(runs)]

    cases = [
        # the defect: doctor first
        (steps("./scripts/ci/runner-doctor.sh nros-big", "just setup tier2"), [0]),
        # the fix: doctor after
        (steps("just setup tier2", "./scripts/ci/runner-doctor.sh nros-big"), []),
        # a doctor with nothing to provision is fine
        (steps("./scripts/ci/runner-doctor.sh nros-big", "cargo build"), []),
        # a setup with no doctor is fine
        (steps("just setup tier2", "just ci matrix build"), []),
        # the `just` spelling of the doctor is caught too
        (steps("just runner-doctor nros-big", "just setup native"), [0]),
        # several setups: the FIRST one is the bound
        (steps("just setup tier2", "just runner-doctor nros-big", "just setup zephyr"), []),
        (steps("just runner-doctor nros-big", "just setup tier2", "just setup zephyr"), [0]),
        # a checkout before either is not a step this gate has an opinion about
        (steps("git submodule update --init", "just setup tier2", "just runner-doctor x"), []),
    ]
    failures = 0
    for i, (s, want) in enumerate(cases):
        got = violations(s)
        if got != want:
            print(f"  self-test FAIL: case {i} -> {got}, want {want}")
            failures += 1

    # A step with no `run:` (a `uses:` step) must not crash the scan.
    if violations([{"uses": "actions/checkout@v4"}, {"run": "just setup tier2"}]) != []:
        print("  self-test FAIL: a `uses:` step was not tolerated")
        failures += 1

    if failures:
        print(f"check-workflow-doctor-after-setup self-test: {failures} case(s) FAILED")
        return 1
    print(f"check-workflow-doctor-after-setup self-test: OK ({len(cases)} cases)")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    if self_test() != 0:
        return 1

    docs = load_workflows()
    bad, pairs = scan(docs)

    # Refuse to report OK over nothing. A gate whose subject count reaches zero
    # -- the spelling moved, the workflows were restructured -- passes while
    # asserting nothing, which is the failure mode it exists to prevent one
    # level up. `check-lane-contracts` was found printing exactly that
    # ("OK -- 0 test target(s)") while the class it guards went unwatched.
    if pairs == 0:
        sys.stderr.write(
            "check-workflow-doctor-after-setup: found NO job with both a\n"
            "`runner-doctor` step and a `just setup` step, so this gate would\n"
            "pass vacuously. Either the spelling moved (see DOCTOR/SETUP in\n"
            "this file) or the lanes were restructured -- fix the gate, do not\n"
            "delete it.\n"
        )
        return 1

    if bad:
        sys.stderr.write(
            "check-workflow-doctor-after-setup: %d job(s) assert their labels "
            "BEFORE provisioning:\n\n" % len(bad)
        )
        for name, job, di, dname, si, sname in bad:
            sys.stderr.write(
                "  %s  [%s]\n"
                "      step %d %r runs the doctor,\n"
                "      step %d %r provisions.\n"
                "      `runner-doctor.sh` looks for build/qemu/bin/qemu-system-arm\n"
                "      FIRST, and `just setup` is what creates it -- so this asks the\n"
                "      host for what the next step supplies. Move the doctor step\n"
                "      after `just setup` (phase-413 W2; run-matrix.yml is the\n"
                "      worked example).\n\n" % (name, job, di, dname, si, sname)
            )
        return 1

    print(
        "check-workflow-doctor-after-setup: OK "
        "(%d job(s) provision before asserting their labels)" % pairs
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
