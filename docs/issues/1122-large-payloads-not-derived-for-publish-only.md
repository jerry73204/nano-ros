---
id: 1122
title: "`LARGE_PAYLOADS` is 131,072 B on an image that subscribes to nothing — CMake derives 0 and no non-Zephyr lane can deliver it"
status: open
type: bug
area: cmake, rmw-zenoh, memory
severity: high
found: 2026-09-06
related: [1033, 0827, 1120, 1121]
---

# The number is derived, written to disk, and read by nobody on this lane

`nros_derive_message_bound_knobs()` writes `set(NROS_DERIVED_MAX_LARGE_SUBSCRIBERS 0)`
into the build dir of a FreeRTOS image, on the `subscribed` basis, with
`NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS "derived"`. The same build dir compiles
`pub const MAX_LARGE_SUBSCRIBERS: usize = 2;`. Both files are quoted below, from
one build.

`ZPICO_MAX_LARGE_SUBSCRIBERS` has exactly one consumer in the tree —
`_nros_resolve_derivable_knob` in `zephyr/cmake/nros_cargo_build.cmake` — and
that file is reached only through `zephyr/CMakeLists.txt`, which only
`find_package(Zephyr)` processes. The FreeRTOS / ThreadX / NuttX / posix CMake
lanes never call `nros_resolve_knobs()` at all, so **no** `NROS_DERIVED_*` value
reaches cargo on them.

This is not one knob. It is the whole family: see "The class" below.

## The measurement

First out-of-tree consumer,
[`orin-spe-heartbeat`](https://github.com/jerry73204/orin-spe-heartbeat) — one
C++ node, one publisher on `/spe/heartbeat`, one 1 Hz timer, **no
subscriptions, no services, no actions**. Image `freertos`, board
`mps2-an385-freertos`, rmw zenoh, `arm-none-eabi-gcc 13.2.rel1`.

```
$ arm-none-eabi-size build/freertos-mps2-an385-freertos/cmake/freertos_entry
   text	   data	    bss	    dec	    hex	filename
 503924	   3172	3596144	4103240	 3e9c48	freertos_entry
```

```
$ arm-none-eabi-nm --size-sort -S build/.../freertos_entry | grep PAYLOADS
20039ba8 00008000 b nros_rmw_zenoh::shim::subscriber::SMALL_PAYLOADS
20019ba4 00020000 b nros_rmw_zenoh::shim::subscriber::LARGE_PAYLOADS
```

`0x20000` = 131,072 B of large payload blocks on a node that never calls
`declare_subscriber`. The target this exists to reach is the Orin SPE: 256 KB of
BTCM all-in, of which NVIDIA's own FreeRTOS demo takes ~142 KB, leaving ~117 KB.
`LARGE_PAYLOADS` alone is 112 % of that budget.

## The mechanism, established

### 1. The derivation runs, and it is right

`nros build freertos` prints (second configure onward — the fragment is a
BUILD-time output, see "Lag" below):

```
-- nros: message-bound sizing DERIVED from 1 types in 1 interface packages (all bounded)
-- nros:   0 over the 2048 B ceiling -> NROS_MAX_LARGE_SUBSCRIBERS
-- nros:   payload classes derived over the 0 SUBSCRIPTIONS this image declares, not the 1-type closure
```

and `build/freertos-mps2-an385-freertos/cmake/nros/message_bound_knobs.cmake`
ends:

```cmake
set(NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS "derived")
set(NROS_MESSAGE_BOUNDS_BASIS "subscribed")
# Derived over the 0 SUBSCRIPTIONS this image declares
set(NROS_DERIVED_MAX_LARGE_SUBSCRIBERS 0)
```

The entity half agrees — `.../cmake/nros/entity_inventory.cmake`:

```cmake
set(NROS_ENTITY_INVENTORY_STATUS "derived")
set(NROS_ENTITY_SUBSCRIBED_TYPES_STATUS "resolved")
set(NROS_ENTITY_SUBSCRIBED_ENTITY_COUNT 0)
```

So the join succeeded, on the narrow basis, and answered zero.

### 2. The crate compiled 2

Same build dir,
`cmake/cargo/nano-ros_1147c/thumbv7m-none-eabi/nros-relwithdebinfo/build/nros-rmw-zenoh-*/out/buffer_config.rs`:

```rust
/// (set via ZPICO_MAX_LARGE_SUBSCRIBERS, default 2).
pub const MAX_LARGE_SUBSCRIBERS: usize = 2;
```

`grep NROS_RESOLVED_ build/freertos-mps2-an385-freertos/cmake/CMakeCache.txt`
returns **nothing**. The resolver never ran in this build dir, because
`nros_resolve_knobs()` is called from exactly one place:

```
$ grep -rn "nros_resolve_knobs()" --include=*.cmake --include=*.txt .
zephyr/CMakeLists.txt:152:nros_resolve_knobs()
```

### 3. Why `SERVICE_BUFFERS` DID shrink, and `LARGE_PAYLOADS` did not

They travel different roads, and only one of the two roads exists off Zephyr.

* `ZPICO_MAX_QUERYABLES` **never uses the derived-knob ladder.** Its default is
  computed inside the build script — `resolve_queryable_default()` in
  `packages/rmw/zenoh/nros-zpico-build/src/runner.rs` — from
  `NROS_DECLARED_SERVICE_SERVERS` / `NROS_DECLARED_INFRA_QUERYABLES`, which
  `cmake/NanoRosEntityFacts.cmake` puts on the cargo command with
  `corrosion_set_env_vars`. That carrier is lane-independent, and the log says so
  on this FreeRTOS build:

  ```
  -- nano-ros: queryable table sized from the declaration — infrastructure none,
     0 declared service server(s) (phase-392 W5)
  ```

  `SERVICE_BUFFERS` is 4,428 B here (one slot) against 35,424 B (eight) at the
  crate default. That is the mechanism working.

* `ZPICO_MAX_LARGE_SUBSCRIBERS` uses the ladder, and only the ladder.
  `env_usize("ZPICO_MAX_LARGE_SUBSCRIBERS", 2)` in
  `packages/rmw/zenoh/nros-rmw-zenoh/build.rs:78` has no declared-fact default to
  fall back to, and no `NROS_DECLARED_*` fact describes subscriptions.

**Correction to the report that opened this issue: `SMALL_PAYLOADS` did *not*
shrink either.** It is 32,768 B in the measurement above — `ZPICO_MAX_SUBSCRIBERS`
(8) × `RING_DEPTH` (4) × `SUBSCRIBER_BUFFER_SIZE` (1024), every one a crate
default, while `NROS_DERIVED_MAX_SUBSCRIBERS` is 1 in the same build dir. The
mechanism that *would* deliver it on a cargo leaf — `nros sync`'s gitignored
`.cargo/nros-managed-env.toml` sidecar (`leaf_entity_env.rs`,
`DERIVED_ENV_KEYS`) — is not written for this consumer, because it has no Rust
leaf. So the earlier "the derivation removed `SMALL_PAYLOADS`" reading was about
a different image (the `linux` one), not this one.

### 4. The class — this is not one knob

```
$ grep -rn "NROS_DERIVED_" --include=*.cmake . | grep -v third-party
cmake/NanoRosEntityInventory.cmake     <- producer
cmake/NanoRosMessageBounds.cmake       <- producer
zephyr/cmake/nros_cargo_build.cmake    <- the ONLY consumer
```

Every derived pool knob in the tree — the four payload classes, the four session
pools, the three executor counts — is published lane-independently and consumed
Zephyr-only. On a non-Zephyr CMake lane **exactly two** derived facts reach
cargo, `NROS_DECLARED_SERVICE_SERVERS` and `NROS_DECLARED_INFRA_QUERYABLES`.
Everything else is computed, written to disk, and discarded.

Priced, on this image, by setting by hand every value CMake had already derived
(`ZPICO_MAX_LARGE_SUBSCRIBERS=0 ZPICO_MAX_SUBSCRIBERS=1 ZPICO_MAX_PUBLISHERS=1
NROS_RMW_SUBSCRIBER_SLOTS=1 NROS_EXECUTOR_MAX_CBS=1 NROS_EXECUTOR_MAX_NODES=1
NROS_EXECUTOR_ACTION_CLIENTS=0 NROS_SUBSCRIPTION_BUFFER_SIZE=16`):

| | text | data | bss | dec |
|---|---:|---:|---:|---:|
| as built | 503,924 | 3,172 | 3,596,144 | 4,103,240 |
| derived values applied by hand | 422,196 | 3,172 | 3,353,456 | 3,778,824 |
| **delta** | **−81,728** | 0 | **−242,688** | **−324,416** |

`ucHeap` (3,145,728 B, the board's FreeRTOS heap) is unchanged in both, so the
nano-ros + lwIP + app share falls 957,512 → 633,096 B. `LARGE_PAYLOADS` and
`SMALL_PAYLOADS` disappear from the symbol table entirely, and
`nros::Node::GlobalStorageHolder<0>::storage` falls 88,600 → 18,320 B.

`ZPICO_MAX_LARGE_SUBSCRIBERS=0` **alone**:

```
   text	   data	    bss	    dec	    hex	filename
 503756	   3172	3465072	3972000	 3c9ba0	freertos_entry
```

−131,072 B of bss exactly, and `nm` no longer lists `LARGE_PAYLOADS`. The image
links and the linker raises nothing.

## Is zero legal?

**Yes, and it is measured above, not only read.**

* `packages/rmw/zenoh/nros-rmw-zenoh/build.rs:78` — `env_usize(...)`, no
  `.max(1)`. Phase-403 W4 removed the floor deliberately; the comment there
  states the premise 0827's floor did not meet.
* `alloc_payload_block` (`shim/subscriber.rs:585`) tests
  `if idx >= MAX_LARGE_SUBSCRIBERS` **before** subscripting `LARGE_PAYLOADS`, so
  a zero-length pool returns `None` and is never indexed.
* `LARGEST_PAYLOAD_CLASS` (`subscriber.rs:238`) already reads
  `if MAX_LARGE_SUBSCRIBERS > 0 && ...`, falling back to the small block, and
  `required_rx_bytes` answers from it — so `create_subscription` refuses a hint
  no class can hold rather than dropping every sample at the transport.
* The unit tests at `subscriber.rs:2133`/`2148` are already written for
  `MAX_LARGE_SUBSCRIBERS == 0`.
* Empirically: the `=0` build above links, and the symbol is gone.

This mirrors issue 1033 on the XRCE side — the caps admit 0 and the clamp was
the defect — and issue 0827's rule that a knob may not reserve memory while
reading as though it were honoured.

## Lag, so the next reader does not misread a refusal

On a CLEAN build dir the message-bound fragment
(`.../pkg/<pkg>/nano_ros_cpp/<msgs>/nros_message_bounds.cmake`) is a **build-time**
output, so the first `nros build` configures with it absent and every configure
in that invocation prints:

```
-- nros: message-bound sizing not available this configure -- 1 of 1 fragments
   are still a build-time output.
```

`nros_derive_message_bound_knobs` `return()`s at the `_pending` check, which is
*before* `_nros_bounds_join_subscribed`, so nothing is derived at all. The
derivation appears only on the **second** `nros build`. That is issue 0991/1002's
chain, and it is not the bug here — the second invocation derives correctly and
the image is still byte-identical.

## What to do — plan, not landed

Not fixed here, and the reasons are worth stating.

* The number has to cross a lane boundary that lives in `cmake/`
  (`NanoRosEntityFacts.cmake` or a sibling), which is shared by every CMake lane
  including Zephyr. There is no fix reachable from `packages/rmw/zenoh/**` alone:
  the build script has no fact to read.
* It needs a decision the measurement cannot make: on Zephyr the derived value
  applies only when Kconfig states the `-1` DERIVE sentinel — an explicit opt-in.
  A non-Zephyr lane has no Kconfig and therefore no spelling for that opt-in, so
  delivering the derived value there changes the default for every existing
  FreeRTOS / ThreadX / NuttX / posix image. Under-sizing this pool is a
  `SubscriberCreationFailed` at `create_subscription`, and picking that up by
  accident is worse than 131 KB on one image.

### Proposed shape

1. `cmake/NanoRosEntityFacts.cmake` (or a new `NanoRosPayloadFacts.cmake`) reads
   `${CMAKE_BINARY_DIR}/nros/message_bound_knobs.cmake` at its deferred flush —
   the FILE, not a variable, because the flush runs in the top-level scope where
   `nros_find_interfaces()`'s variables are not visible — and, **only** when
   `NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS` is `derived` **and**
   `NROS_MESSAGE_BOUNDS_BASIS` is `subscribed`, appends
   `NROS_DECLARED_LARGE_SUBSCRIBERS=${NROS_DERIVED_MAX_LARGE_SUBSCRIBERS}` to the
   `corrosion_set_env_vars` call it already makes.

   The two conditions are the whole safety argument: on the `closure` basis the
   count is a count of large *types* in the linked closure, which under-counts an
   image with two subscriptions on one large type. Refuse there, keep the
   default 2.

2. `packages/rmw/zenoh/nros-rmw-zenoh/build.rs` gains the
   `resolve_queryable_default()` shape one crate over: read
   `NROS_DECLARED_LARGE_SUBSCRIBERS` (with `cargo:rerun-if-env-changed`), use it
   as the **default** for `env_usize("ZPICO_MAX_LARGE_SUBSCRIBERS", …)`, else 2.
   A named `ZPICO_MAX_LARGE_SUBSCRIBERS` still wins, so rung 1 of the ladder is
   preserved — which a bare `corrosion_set_env_vars(ZPICO_MAX_LARGE_SUBSCRIBERS=…)`
   would silently break, since it sets the variable in the child environment.

3. A gate. `check-knob-delivery.py` is the right home and does not cover this:
   its `DERIVED_PAIRS` map omits `NROS_DERIVED_MAX_LARGE_SUBSCRIBERS`,
   `NROS_DERIVED_SUBSCRIBER_LARGE_SIZE` and
   `NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE` entirely, and its very first check is
   `if not cache: return [...]` on a build dir with no `NROS_RESOLVED_*` — i.e.
   it *cannot* be pointed at a non-Zephyr build dir at all. `just check
   knob-delivery` runs `--self-test` only, so nothing in CI has ever asserted
   delivery on any lane. Either extend it to accept the `NROS_DECLARED_*` carrier
   as a second delivery road, or add a sibling that asserts the negative form:
   *no `NROS_DERIVED_*` symbol may have its only consumer under `zephyr/`.* That
   sibling fails today for the entire family, which is the finding.

4. The rest of the family (§4) is phase-412's remaining work, not this issue's:
   324,416 bytes on this one image.

## Reproduction

```sh
cd <orin-spe-heartbeat>
export PATH="$HOME/.nros/sdk/arm-none-eabi-gcc/13.2-nros4/bin:<nano-ros>/packages/cli/target/release:$PATH"
nros sync  --nano-ros-path <nano-ros>
nros build freertos --nano-ros-path <nano-ros>   # twice — see "Lag"
grep MAX_LARGE build/freertos-mps2-an385-freertos/cmake/nros/message_bound_knobs.cmake
grep MAX_LARGE $(find build/freertos-mps2-an385-freertos -name buffer_config.rs | head -1)
```

The pinned toolchain must be **first** on PATH. A build dir configured with the
system `/usr/bin/arm-none-eabi-g++` (10.3.1) is cached in `CMakeCache.txt` and
`PATH` cannot move it — the entry TU is compiled `-std=c++14`
(`cmake/toolchain/arm-freertos-armcm3.cmake:27`) and 10.3.1 rejects the
designated initializers the entry codegen emits for
`struct nros_baked_boot_config`:

```
freertos_entry_nros_main_generated.cpp:47:1: error: C99 designator 'node_name'
    outside aggregate initializer
```

13.2 accepts them. Recovering a build dir configured with the wrong compiler
means starting a new build dir, which is the one thing CMake genuinely cannot do
in place. Whether the entry codegen should emit C++14-safe initializers at all is
a separate question and not filed here.

## Also found, filed nowhere yet

Rebuilding `nros-launch-resolve` to the pin this `nros` demands
(`just setup-launch-resolve`, play_launch `4c214a6` → `db4af87`) makes the
resolver stop reading `<stem>.contract.yaml`. The old model listed
`launch/system.contract.yaml` under `meta.inputs` and carried a `contracts:`
block; the new one lists only the launch file and `system.toml` and carries no
wiring, so `nros ws entity-inventory` refuses with 1120's message and every pool
falls back to the closure basis. That is a regression in the pinned resolver and
should be its own issue; it is recorded here because it silently reverses this
issue's premise ("the entity inventory now derives correctly") and it confounded
an experiment run against it.
