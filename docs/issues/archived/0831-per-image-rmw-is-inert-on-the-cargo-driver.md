---
id: 831
title: "`[image.<id>].rmw` configures nothing on the cargo driver — and a
  workspace fixture row's `rmw` does not either, so two tier-2 coordinates
  test zenoh while claiming cyclonedds and XRCE"
status: resolved
type: bug
area: build
related: [rfc-0065, phase-383, issue-0828, issue-0482, issue-0270, issue-0937]
---

## Problem

RFC-0065 D6 makes `[image.<id>]` the buildable unit and its declaration the
SSoT. One of its keys does not reach the build.

* **cmake driver** — `image.rmw` becomes `-DNROS_RMW=<rmw>` and selects the
  backend. Correct.
* **cargo driver** — `image.rmw` reaches exactly one thing: `coordinate()`,
  which names the build DIRECTORY. The backend comes from the
  `<entry>_nros_selection` facade `nros sync` generates from the bringup's
  `[system] rmw`, and nothing in that chain consults the image.

So `[image.native_cyclonedds] rmw = "cyclonedds"` produces
`build/posix-cyclonedds/native_cyclonedds_entry` containing a **zenoh** binary.
A directory named for a backend it does not contain reads as coverage.

## The same hole one layer up, and it is live

`examples/fixtures.toml` has the identical shape, and it is not hypothetical:

```
[[workspace_fixture]]
id = "workspace-rust-native-cyclonedds"
platform = "linux"
lang = "rust"
rmw = "cyclonedds"
dir = "examples/workspaces/rust"
target_dir = "target-fixtures-cyclonedds"
```

Measured on the artifact that row builds:

```
strings target-fixtures-cyclonedds/nros-relwithdebinfo/native_entry | grep -ci cyclone   # 0
strings target-fixtures-cyclonedds/nros-relwithdebinfo/native_entry | grep -ci zenoh     # 1916
```

A row's `rmw` is read in exactly three places — `cmake_defs()` (cmake rows
only), `row_coord()`, and the label printer. **For a cargo row it is
coordinate metadata, not build configuration.** `workspace-fixtures-build.sh`
never mentions it.

`linux,rust,cyclonedds` is one of tier 2's fourteen coordinates. Tier 2
believes it covers rust-on-cyclonedds; it builds and runs zenoh. Same for
`workspace-rust-native-xrce`. This predates phase-383 — the migration to
`nros build` only made it visible, because an image declaring `rmw` put the
claim somewhere a reader looks.

## Mitigation in place (superseded — see Resolution)

`nros build` REFUSED a cargo image whose `rmw` differed from the bringup's
`[system] rmw`, naming both and pointing here. A loud refusal is strictly better
than a directory that lies, and it cost nothing at the time: no shipped image
declared a divergent rmw.

The fixture rows were deliberately NOT changed. Relabelling them `zenoh` would
silently drop two coordinates' worth of claimed coverage, and deleting them
would drop it outright — which of those is right is a coverage decision, not a
mechanical one.

## Fix

The backend is selected by the facade, so per-image RMW means a facade per
image rather than per entry name. `nros sync` generates
`generated/nros-selection/<entry>/`; an image is already the thing that names
an entry (`<image>_entry`), so the natural shape is for sync to read the
`[image.*]` table and emit one facade per image, with `[system] rmw` as the
default when an image declares none.

That also closes the fixture hole without touching the rows: once the image
carries the RMW, a row naming `[image.native_cyclonedds]` gets a cyclonedds
binary, and `row_coord()` stops being a claim nobody checks.

**Until then, add a runtime assertion rather than trusting the coordinate.**
The interop and matrix cells that name an RMW should assert the backend the
binary actually linked — the artifact knows, and `strings` proved it in one
command here.

## Sweep

```sh
grep -rn 'NROS_RMW\|image.rmw\|row_coord' scripts/build packages/cli/nros-cli-core/src/cmd/build.rs
grep -n 'rmw' scripts/build/workspace-fixtures-build.sh      # no hits: the driver never reads it
```

## Resolution

Three landings, in this order.

**1. The facade reads the image** (`101799504`, 2026-08-29). `facade::image_rmw`
matches `package_name(image_id) == entry_name` FORWARD, folds `[image_defaults]`
first and falls back to `[system] rmw`; the board's default RMW is carved out
(`default-features = false` on BOTH the facade and the generated entry, because
cargo unions features and cannot subtract a default — issue 0270). Measured with
`nm` on the three rust-workspace native images: `native` 777 zenoh `_z_` / 0
`dds_` / 0 `uxr_`, `native_cyclonedds` 328 `dds_` and nothing else,
`native_xrce` 232 `uxr_` and nothing else. `rmw_coordinate_truth.rs` was added as
the standing artifact assertion the issue asked for, and the two rows gained
`[image.native_cyclonedds]` / `[image.native_xrce]` so no coordinate was
dropped. The stand-in refusal above was deleted.

**2. The tolerance the fix removed, which the code kept claiming.** Landing 1
made the facade the ONE carrier of the RMW, which retired the board's
`default = ["rmw-zenoh"]` as a fallback — but the missing-facade path still
WARNED, on the pre-0831 grounds that "the RMW and ROS edition have defaults the
Entry can build against". Measured on this tree with the facade absent:

```
$ nros build native_cyclonedds --dry-run
nros build: warning: no selection facade at …/generated/nros-selection/native_cyclonedds_entry
  — building `native_cyclonedds` without its RMW and ROS edition. […] Its system
  declares no capabilities, so nothing else is lost.

$ grep board build/posix-cyclonedds/native_cyclonedds_entry/Cargo.toml
nros-board-linux = { path = "…", default-features = false }
```

No `rmw-*` at all — and no `ethernet` / `image-runtime` either, since the facade
is what re-supplies the board's non-RMW defaults. So `[image.<id>].rmw` was
declared, could not take effect, and the build said so in a line that reads as
reassurance: this issue's own shape, one door over, reachable from any unsynced
workspace (a fresh clone's first fixture build is one — `nros sync` keys facades
off the entry PACKAGE, which `nros build` is what generates).

The warn/fail split is gone. A missing facade now takes the SELF-HEAL path
phase-413 W2 built for the capability case — write it from the entry just
generated, then regenerate the entry against it — for every image, and is fatal
only when it cannot be written. Nothing that can be repaired is refused, and no
build proceeds without the declaration it was given. Proof, on a tree with no
facades for the generated entries, from one `nros build --dry-run` each:

```
native            → nros-board-linux features = ["rmw-zenoh"]
native_cyclonedds → nros-board-linux features = ["rmw-cyclonedds"]
native_xrce       → nros-board-linux features = ["rmw-xrce"]
```

Mutation: delete `rmw = "cyclonedds"` from `[image.native_cyclonedds]` and the
same command writes `features = ["rmw-zenoh"]` — the declaration is load-bearing
rather than incidental.

**3. The row's claim is checked STATICALLY too.** `rmw_coordinate_truth.rs`
answers this on the artifact, which is the authority, but only in a lane that
built it — and the lane that would have caught the original may be a nightly
away. `fixtures-manifest.py::_require_image_rmw` now asks the same question of
the DECLARATIONS at `just check` speed: a `[[workspace_fixture]]` row's `rmw`
must be one its image LINKS, where the image's backend set is
`image.rmw` over `[image_defaults]` over `[system] rmw`, plus one per
`[[domain]]` — deliberately the same union `facade::image_backends` compiles in,
which is why the two BRIDGE rows pass (one image, two backends by declaration)
and the pre-fix shape does not. 110 rows validate; restoring the pre-fix
declaration reproduces the original bug as a `just check` failure naming the row,
the image and the remedy.

