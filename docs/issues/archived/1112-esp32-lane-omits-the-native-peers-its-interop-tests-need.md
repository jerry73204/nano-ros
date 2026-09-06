---
id: 1112
title: "The esp32 lane never builds the NATIVE half of its own interop tests, so the failure names the esp32 side"
status: resolved
area: testing, build
severity: medium
related: [1025, 1070, 1106, 0968, 0584]
---

# Half of an interop test is a fixture the lane does not build

## What

`test_esp32_to_native` and `test_native_to_esp32` each start ONE esp32 image and
ONE host binary. The esp32 half comes from `just esp32 build-all`; the host half
is a `linux` fixture that module never builds. So both tests fail immediately:

```
Failed to build native listener: BuildFailed("Test fixture binary not prebuilt:
  /__w/nano-ros/nano-ros/build/cargo-fixtures/linux/nros-relwithdebinfo/listener
Run `just build-test-fixtures` first.")
```

Three retries, 0.36–0.41 s each. The speed is the tell: a delivery problem takes
seconds to time out, a missing prerequisite fails instantly.

## Why it read as an esp32 fault

The failure is reported under `esp32_emulator`, in the esp32 job, on a lane whose
last three reds were genuinely esp32 defects (issues 1025, 1070, 1106). Every
signal points at the board. Nothing in the message says "this half of the test is
a different platform's fixture" — it names a path, and reading the path is what
answers it.

Issue 0968 listed `test_native_to_esp32` among its undiagnosed failures for that
reason.

## Fix

`just esp32 build-fixtures` stages the two peers by ID:

```
for _peer in native-rust-talker native-rust-listener; do
    bash scripts/build/fixtures-build.sh linux rust --id "$_peer"
done
```

Same reasoning the recipe already applies one line up to the workspace entry —
"the test's skip message (correctly) points users to `build-fixtures`, so this
lane must stage it too". The tests belong to this module, so their prerequisites
are this lane's to stage.

**By ID, not `fixtures-build.sh linux rust`**, which is 74 rows to obtain two.
The rows had no `id` key, so they gained one (`native-rust-talker`,
`native-rust-listener`) — named for what the fixture IS, per the manifest
validator's rule against work-item ids.

Verified: after deleting both artifacts, the two commands rebuild them at exactly
the paths the tests demand.

## What this does NOT fix

`test_esp32_workspace_entry_e2e` still reports a missing `int32-sink` fixture,
and its own message says the deeper problem:

> Since issue 0584 an absent in-lane fixture is a hard failure, not a skip — a
> gated run already asserted the lane's fixtures are built. **Something is still
> laundering the resolver's Err into a `[SKIPPED]`.**

That is a different defect in the skip path, not a missing build step, and it
wants its own look rather than another fixture added to this list.
