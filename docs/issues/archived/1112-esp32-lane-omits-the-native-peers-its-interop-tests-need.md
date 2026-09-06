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

## The workspace entry: BOTH halves, and the second one is the interesting half

`test_esp32_workspace_entry_e2e` needed the same treatment plus a fix the peers
did not:

**The build set.** It subscribes with `bins/int32-sink` — a `linux` fixture,
because the entry's own in-process listener sees nothing (no same-session zenoh
loopback), so that native binary IS the test's only observable endpoint. Staged
by id like the peers. The three `int32-sink` rows had no `id`, so all three
gained one (`int32-sink-{zenoh,xrce,cyclonedds}`); the esp32 lane builds the
zenoh one, since the images speak zenoh. It lands in a variant group dir
(`linux-3263301353`) rather than beside the peers because the row carries
`no_default_features` + `features`.

**The laundering, which was in the TEST.** The skip-budget check kept saying
"Something is still laundering the resolver's Err into a `[SKIPPED]`" and this
was it:

```rust
let native_listener = match build_int32_sink() {
    Ok(p) => p.to_path_buf(),
    Err(e) => nros_tests::skip!("int32-sink fixture not prebuilt ({e})"),
};
```

Issue 0584: an absent IN-LANE fixture is a hard failure, because a gated run has
already asserted the lane's fixtures are built. "Not prebuilt" there means the
LANE is wrong, and a skip reports that as coverage. Its own two sibling tests
twenty lines up already `.expect()` their native peers — one file, two spellings,
and the inconsistent one is the one that hid a lane gap.

Now `.expect("Failed to build int32-sink")`.

## The class, NOT swept — deliberately

Twelve sites across five files share the shape (`bridge_mixed_rmw`,
`bridge_zenoh_to_cyclonedds`, `declarative_bridge_zenoh_to_{cyclonedds,xrce}`,
`zephyr`). CLAUDE.md says fix the class, and here that would be wrong without
evidence per site: 0584's rule is about an **in-lane** fixture, and a bridge test
whose fixture is genuinely out of lane SHOULD skip. Converting all twelve would
turn correct skips into hard failures across several lanes.

What made this one answerable is that the skip-budget check named it on a lane
that had already asserted its fixtures were built. Each of the other twelve wants
that same evidence before it moves.

## What this does NOT fix

Nothing further on this lane is known-open. For the record, the message that
pointed at the laundering was:

> Since issue 0584 an absent in-lane fixture is a hard failure, not a skip — a
> gated run already asserted the lane's fixtures are built. **Something is still
> laundering the resolver's Err into a `[SKIPPED]`.**

That is a different defect in the skip path, not a missing build step, and it
wants its own look rather than another fixture added to this list.
