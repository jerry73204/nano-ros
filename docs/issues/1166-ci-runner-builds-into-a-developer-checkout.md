---
id: 1166
title: "The self-hosted runner builds Zephyr into a developer's checkout, the lane's own
  verification reports that as `[OK]`, and a west build dir carries neither the
  board nor the checkout in its name"
status: open
type: bug
area: [ci, build]
severity: high
related: [1157, 1025, 0759, 0980]
---

## What

The self-hosted runner's environment pins the Zephyr west workspace into a
developer's working tree:

```
$ cat <runner home>/actions-runner/.env
…
NROS_ZEPHYR_WORKSPACE=<primary developer checkout>/zephyr-workspace
```

`just/zephyr.just` reads that variable first
(`ZEPHYR_WORKSPACE := env("NROS_ZEPHYR_WORKSPACE", …)`), so every scheduled
tier-2 run builds Zephyr images into the developer's tree while its sources and
its `nros` come from the runner's own checkout under
`actions-runner/_work/nano-ros/nano-ros`.

## Measured

The 07:10 scheduled nightly wrote into the developer tree:

```
$ grep APPLICATION_SOURCE_DIR <dev checkout>/zephyr-workspace/build-rust-talker-zenoh/CMakeCache.txt
APPLICATION_SOURCE_DIR:PATH=<runner checkout>/examples/zephyr/rust/talker

$ stat -c '%y' <dev checkout>/zephyr-workspace/build-rust-talker-zenoh/CMakeCache.txt
2026-09-06 16:37 +0800      # the 07:10 UTC run
```

A build directory inside one checkout, configured for an application in
another.

## Three distinct defects, and only the first is host configuration

**1. The runner writes into a developer tree.** Host config, not repo code —
and it is load-bearing rather than accidental: the runner checkout has no
`zephyr-workspace/` and no `.west/` of its own, so it has nowhere else to build.
Repointing it means provisioning a second Zephyr workspace first. Recorded here
because the other two defects are what let it go unnoticed for as long as it
has.

**2. The lane's own verification reports it as `[OK]`.** From the same run:

```
[OK] Zephyr workspace: <dev checkout>/zephyr-workspace
```

The step is named `Verify this runner's labels are true`. It checks that a
Zephyr workspace EXISTS; it does not check that the workspace belongs to the
checkout under test. A verification that cannot fail on the condition it is
named for is the vacuous-pass class this tree already gates against elsewhere
(`check-no-vacuous-tests`). This is the half that made the sharing invisible,
and it is a one-line assertion to fix: the resolved workspace must be inside
the checkout, or the step must say out loud that it is not.

**3. A west build dir names neither the board nor the checkout.**
`just/zephyr-dev.just` computes it as:

```sh
bd="build-$(echo '{{example}}' | tr '/' '-')-{{rmw}}"
```

So `rust/talker` + `zenoh` is `build-rust-talker-zenoh` for **every** board and
**every** checkout sharing the workspace. Two consequences, and the second is
the one that bites:

* `mps2_an385` and `native_sim/native/64` builds of the same example collide
  with each other.
* When the colliding builds come from different checkouts, west refuses, which
  is correct but arrives as an error about the developer's own command:

```
ERROR: Build directory ".../zephyr-workspace/build-rust-talker-zenoh" is for
application "<runner checkout>/examples/zephyr/rust/talker", but source
directory "<dev checkout>/examples/zephyr/rust/talker" was specified; please
clean it, use --pristine, or use --build-dir to set another build directory
FATAL ERROR: refusing to proceed without --force due to above error
```

This is not hypothetical: it blocked the first measurement for issue 1145 on
2026-09-06, and the suggested remedies (`--pristine`, clean) would have
destroyed CI's build rather than fixed anything.

## Not the same as issue 1157

1157 is the opposite error one layer over: `subtree-guard` refuses builds in
trees that share **nothing**, because its lock is host-global. This issue is
real sharing that nothing refuses. A fix for either must not be mistaken for a
fix for the other — and note that fixing 1157 by keying on the build root would
NOT catch this, because here the shared thing is the west workspace, which is
not under `build/`.

## Fix direction

* **Defect 2 first — it is small and it is what restores the signal.** Assert
  the resolved workspace is inside the checkout under test, and fail (or warn
  loudly and name both paths) when it is not.
* **Defect 3:** put the board in the build dir name, and make the workspace
  root per-checkout, or accept sharing and key the dir by checkout too. Moving
  the test-side locator in the same commit is the standing rule for this class
  (#393).
* **Defect 1** is the maintainer's call: provision the runner its own Zephyr
  workspace (disk cost, and a Zephyr workspace is not cheap), or keep the
  sharing deliberately and record it — in which case defects 2 and 3 are the
  whole fix, because they are what make deliberate sharing safe to live with.

Do not "fix" this by deleting the developer's build dirs from CI, or CI's from
the developer's tree: whichever side runs second destroys the other's cache,
and the failure looks like a slow build rather than a bug.
