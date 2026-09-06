# Importing a Board Crate

This chapter is for **consumers** of a nano-ros board crate -- vendors and downstream applications (the Autoware Safety Island archetype) who want to bring a board into their own Zephyr app with as little glue as possible. If you are *writing* a new board crate, see [The `Board` Trait Family](board-trait.md) and [Custom Board Package](custom-board.md) instead.

## Goal

Consume a nano-ros board crate from a downstream Zephyr application with a **single CMake call**. The consumer's build script SHOULD NOT carry any of:

- a hand-curated `EXTRA_CONF_FILE` listing the board crate's `prj.conf` and per-board Kconfig snippets
- a hand-curated `DTC_OVERLAY_FILE`
- a hardcoded `BOARD=<id>` string
- a hardcoded `-DNANO_ROS_RMW=<rmw>` define
- a hand-rolled FVP / qemu launch shell

All of those are owned by the board crate. The consumer's `CMakeLists.txt` declares which board it wants; nano-ros layers the rest in.

## Prereqs

Before importing a board crate, the consumer needs:

1. **A Zephyr 3.7+ workspace managed by west.** Zephyr 3.7 is the floor (the official in-tree `zephyr-lang-rust` module did not exist below it, so nothing earlier can link the nano-ros Rust staticlib); newer LTS lines work too.
2. **All gated SDK packages for the target board.** Run `nros doctor --board <name>` to check; missing items can be installed with `nros setup <board>` (or `nros setup --tool <t>` for a single dependency). See `nros setup` in the [build commands reference](../reference/build-commands.md).
3. **nano-ros listed as a project in your `west.yml`**, and nano-ros's Zephyr module exported on `ZEPHYR_EXTRA_MODULES` so Zephyr picks up its Kconfig + DTS roots.

A minimal `west.yml` fragment:

```yaml
manifest:
  remotes:
    - name: newslab
      url-base: https://github.com/NEWSLabNTU
  projects:
    - name: nano-ros
      remote: newslab
      revision: nros-v0.5.0    # pin a release tag — a moving branch
                                 # unships your tested pairing (see
                                 # installation.md "Pinning a version")
      path: deps/nano-ros
      import: false
  self:
    west-commands: deps/nano-ros/scripts/west-commands.yml
```

And the matching environment:

```sh
export ZEPHYR_EXTRA_MODULES="$PWD/deps/nano-ros/zephyr"
```

## The one-call pattern

A consumer `CMakeLists.txt` should look like this and nothing more:

```cmake
cmake_minimum_required(VERSION 3.20)

# Single import call. Layers in BOARD, EXTRA_CONF_FILE, DTC_OVERLAY_FILE,
# NANO_ROS_RMW default, and the runner hint.
nano_ros_use_board(fvp-aemv8r-smp)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_app LANGUAGES CXX)

target_sources(app PRIVATE src/main.cpp)
```

**Call order matters.** `nano_ros_use_board()` MUST precede `find_package(Zephyr ...)`. Zephyr reads `BOARD`, `EXTRA_CONF_FILE`, and `DTC_OVERLAY_FILE` during its `find_package` call; setting them after that point has no effect. The pattern above is the canonical shape -- copy it verbatim and only fill in the board name.

`nano_ros_use_board()` is shipped by the nano-ros Zephyr module. It is available the moment `ZEPHYR_EXTRA_MODULES` includes `deps/nano-ros/zephyr`; no extra `include()` is required.

> **Out-of-tree boards.** The lookup searches
> `<nano-ros>/packages/boards/` plus every root named by
> `NROS_EXTRA_BOARD_PATH` (environment, or a CMake cache var) — so a
> board crate in YOUR tree is found without copying it into the
> checkout. Each root has the same layout as `packages/boards/`.
> Board keys are global: a name resolving under more than one root is
> a fatal ambiguity, never shadowed by search order.

## What the call layers in

| Source | Effect |
| --- | --- |
| `BOARD` | Set to `NROS_BOARD_ZEPHYR_ID` (from the board's descriptor) if the user did not pass `-DBOARD=...` on the command line. |
| `EXTRA_CONF_FILE` | The board crate's `prj.conf` and any per-board Kconfig fragments (e.g. an HWv2 snippet) are appended. Any consumer-supplied `EXTRA_CONF_FILE` is preserved and layered AFTER the board's. |
| `DTC_OVERLAY_FILE` | The board crate's per-board DTS overlay is appended. Consumer overlays are preserved and layered after. |
| `NANO_ROS_RMW` | Defaulted to `NROS_BOARD_DEFAULT_RMW` (from the board's descriptor) when the consumer did not pass `-DNANO_ROS_RMW=...`. |
| `NROS_BOARD_RUNNER` | Cached so a launcher can pick up the right simulator binary / target-launcher. |
| `NROS_BOARD_RUST_SUPPORT_MODULE` | Appended to `ZEPHYR_EXTRA_MODULES` when the board carries a Rust support module. (Not yet in the `board.cmake` ↔ `Cargo.toml` drift-audit field list — drift here is unaudited.) |

The values come from ONE authored file, the board's `nros-board.toml`. cmake cannot read TOML, so `nano_ros_use_board()` runs `nros board cmake-vars` to write a mechanical projection of the descriptor into the build directory and includes that. The projection is a build artifact: never committed, regenerated every configure, and editing it edits nothing that survives.

This replaced a second AUTHORED file per board (`board.cmake`, 14 `NROS_BOARD_*` variables) plus a `[package.metadata.nros.board]` mirror in `Cargo.toml`. Two faces of the same facts need a drift audit; that audit examined zero boards for two independent reasons and reported OK for as long as it existed. One authored file has nothing to drift against.

## Per-app overrides

The one-call pattern still gives the consumer escape hatches:

- **Pin a different Zephyr board id.** Pass `-DBOARD=<other>` on the CMake / `west build` command line; `nano_ros_use_board()` notices the user value and emits a warning rather than clobbering it.
- **Pick a different RMW.** Pass `-DNANO_ROS_RMW=<rmw>` (`zenoh`, `xrce`, or `cyclonedds` -- see [Backend Reference](../user-guide/rmw-backends.md)). The board crate's default is used only when nothing is set.
- **Layer extra Kconfig / DTS.** Set `EXTRA_CONF_FILE` and `DTC_OVERLAY_FILE` *after* the call to `nano_ros_use_board()`. They will be applied on top of the board crate's contributions, not in place of them.

```cmake
nano_ros_use_board(fvp-aemv8r-smp)

# Extra app-specific Kconfig layered on top of the board's defaults.
list(APPEND EXTRA_CONF_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boards/fvp-aemv8r-smp.conf")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_app LANGUAGES CXX)
```

## Running

The board's `[board.zephyr] runner` reaches cmake as `NROS_BOARD_RUNNER`, and
Zephyr's own emulator target does the rest:

```sh
west build -d build
west build -d build -t run
```

For an FVP board, `nros setup board <name>` has already fetched the model
(`[tool.arm-fvp]`, a pinned Arm CDN permalink with a checked digest), so the
resolver finds it in the SDK store. Export the directory Zephyr's `armfvp`
runner looks in:

```sh
export ARMFVP_BIN_PATH="$(bash scripts/zephyr/resolve-fvp-bin.sh)"
west build -d build -t run
```

> There used to be a `west fvp run` verb here. It read `NROS_BOARD_RUNNER` from
> `CMakeCache.txt`, ran that same resolver, exported `ARMFVP_BIN_PATH`, and then
> exec'd `west build -t run` — so it was env wiring in front of the stock verb,
> and env wiring belongs in your environment. It was retired in RFC-0064
> revision 5.

## Inspecting the manifest

To see exactly what `nano_ros_use_board()` will layer in for a given board:

```sh
nros board info fvp-aemv8r-smp
```

This prints the resolved Zephyr board id, the `prj.conf` + Kconfig fragment list, the DTS overlay, the default RMW, and the runner hint -- everything the call would set.

`nros board info` also prints the exact `cmake_vars` text the build will
include, so "what does cmake actually see for this board?" is answerable without
configuring anything.

There is no drift audit any more, and nothing to audit: the descriptor is the
only file a human writes.

## Anti-patterns

Things that LOOK reasonable but the one-call pattern obviates:

- **Don't hand-list the board's `prj.conf` in `EXTRA_CONF_FILE`.** It is already there; doing it again either duplicates or fights the layering order.
- **Don't hardcode `BOARD=<id>` in a `build.sh`.** The call sets it; hardcoding it short-circuits the `nros board info` inspection and the drift audit.
- **Don't carry your own copy of `boards/<id>.conf` or `boards/<id>.overlay` mirroring the board crate's.** Vendor a delta only -- the base ships in the crate.
- **Don't reimplement the FVP launch in a `build.sh`.** Export `ARMFVP_BIN_PATH` and use `west build -t run`; Zephyr's own `armfvp` runner covers `FVP_BaseR_AEMv8R` and the other supported simulators with the right CLI flags, the same way `west build` covers compilation.
- **Don't `include()` files from `deps/nano-ros/cmake/` directly.** The public surface is `nano_ros_use_board()`; anything else is internal and may move.

## Migrating an existing hand-glued consumer

Most downstream consumers (the ASI archetype is the canonical example) carry years of accumulated glue. The migration is mechanical:

1. **Find the per-board Kconfig and overlay entries in the current `CMakeLists.txt` / `build.sh`.** Anything listing the *board crate's* `prj.conf`, board overlay, or HWv2 snippet -- delete it. Keep only entries that point at the *consumer's* own deltas.
2. **Find any hardcoded `BOARD=<id>` string** (in CMake `set(BOARD ...)`, in `west build -b <id>`, or in `build.sh`). Delete it. `nano_ros_use_board()` will set it from `board.cmake`.
3. **Replace `find_package(Zephyr) + ... + manual FVP launch` with the canonical pattern** above -- one `nano_ros_use_board()` call, then `find_package(Zephyr)`, then `project()`, then `target_sources(app ...)`. Replace the FVP shell script call with `west fvp run -d build`.
4. **Keep only app-specific deltas in `boards/<id>.conf` / `boards/<id>.overlay`.** For the ASI archetype this typically means Autoware-msg sizing (large topic / participant memory) and the application's own GPIO map -- everything generic moves into the board crate.

After migration, the consumer's CMake should be roughly twenty lines, with `nano_ros_use_board()` as the only nano-ros-specific call.

## Cross-references

- [The `Board` Trait Family](board-trait.md) -- for *implementers* of a new board crate. This chapter is the consumer-side dual.
- [Custom Board Package](custom-board.md) -- the full board-crate authoring guide.
- [Vendor Overlay Board Crate](vendor-overlay.md) -- the lighter "I just want to override one field" path.
- [Build commands reference](../reference/build-commands.md) -- `nros setup`, `nros doctor`, `nros board info`.
- [RMW backends](../internals/rmw-backends.md) -- the menu of values for `-DNANO_ROS_RMW=...`.
