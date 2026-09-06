---
id: 1117
title: "A cross build silently accepted an unpinned host toolchain — the pin said
  13.2.rel1, the compiler was Ubuntu's 10.3.1, and nothing printed either number"
status: open
type: bug
area: cmake, build, cli
severity: medium
found: 2026-09-06
related: [0500, 0774, 1113]
---

## Symptom

`nros build freertos` on `examples/workspaces/cpp` failed inside a generated
C++ TU:

```
freertos_entry_nros_main_generated.cpp:
    error: C99 designator 'node_name' outside aggregate initializer
```

Nothing in the build output named the compiler. Recovered from the build tree
afterwards:

```
$ grep COMPILER build/freertos-zenoh-mps2-an385-freertos/cmake/CMakeFiles/3.22.1/CMakeCXXCompiler.cmake
set(CMAKE_CXX_COMPILER "/usr/bin/arm-none-eabi-g++")
set(CMAKE_CXX_COMPILER_VERSION "10.3.1")
```

`nros-sdk-index.toml` pins that toolchain three major versions higher:

```toml
[tool.arm-none-eabi-gcc]
version  = "13.2-nros4"
upstream = "13.2.rel1"
```

The SDK store on that host held only `mdbook` — the toolchain had never been
provisioned — so the build fell through to Ubuntu 22.04's `gcc-arm-none-eabi`
package. After `nros setup --tool arm-none-eabi-gcc`, the pinned 13.2.1 builds
the same image cleanly.

## The measurement

Reproduced directly, outside the build system, on the exact construct
`codegen/entry/mod.rs` emits for a node name:

```c++
struct C { const char* topic; char node_name[8]; int depth; };
static const C c = { .topic = "t", .node_name = "abc", .depth = 3 };
```

| compiler | `-std=c++14` | `-std=c++17` | `-std=c++20` |
| --- | --- | --- | --- |
| `arm-none-eabi-g++` 10.3.1 (`/usr/bin`, Ubuntu 22.04) | error | error | error |
| `g++` 11.4.0 (host — same C++ frontend, different target) | ok | ok | ok |
| `arm-none-eabi-g++` 13.2.1 (SDK store, the pin) | ok | ok | ok |

So the variable is the FRONTEND, not the language standard, and the frontend is
target-independent — which is why one floor constant covers arm and riscv alike.

## What it cost

Two wrong diagnoses, recorded in `docs/issues/archived/1113-*.md` (retracted).
The first blamed the C++ standard the cross lane passes; the second blamed a GCC
10 parser bug, which was *true* and irrelevant, because this project does not
use GCC 10 — it pins 13.2. A change to `codegen/entry/mod.rs` replacing
designated with positional initialization "for portability" was nearly made, to
accommodate a compiler nano-ros does not ship.

None of that time was spent on a hard question. It was spent because the build
never answered an easy one: *which compiler is this?*

## Root cause: PATH was the whole resolution, and PATH is not enough

Four of the five toolchain files under `cmake/toolchain/` selected the compiler
by BARE NAME:

```cmake
set(CMAKE_C_COMPILER    arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER  arm-none-eabi-g++)
```

so the answer was whatever `PATH` resolved, with no version check, no
comparison against the pin, and no output. `riscv64-threadx.cmake` was the
exception — issue 0657 had already given it override → store → PATH resolution —
but even there the `message(STATUS)` printed the PREFIX and neither the VERSION
nor the ORIGIN, so "the provisioned pin" and "whatever the distro ships" printed
identically.

There is a second, sharper failure inside this one. The SDK store reaches `PATH`
only through `activate.sh`, which computes it when it is SOURCED. So

```
$ nros setup --tool arm-none-eabi-gcc     # succeeds, installs 13.2.1
$ nros build freertos                     # still uses /usr/bin 10.3.1
```

in one shell — provisioning that reports success and changes nothing, which
reads exactly like the pin having no effect.

## The tension, which is real and is not resolved by refusing the fallback

`activate.sh` documents the fallback deliberately:

> A system cross-gcc (e.g. `/usr/bin/arm-none-eabi-gcc`) still resolves when the
> store has none.

That is a choice, not an oversight: a contributor with a working distro
toolchain should not need a 150 MB download to build. The defect is that the
choice was **silent and unrecorded**, not that it exists.

This is the third time this class has been filed here:

* **issue 0500** — a stale Corrosion in the accumulating SDK store shadowing the
  pinned one. Remedy: print `nano-ros: Corrosion <ver> via <origin>` and make
  that line the evidence. This issue copies that remedy.
* **issue 0774** — the loader picking a `libzenohc.so` the router was not built
  against. Same shape one layer lower: two legitimate installs, no record of
  which one won.

## Fix

`cmake/toolchain/NanoRosCrossToolchain.cmake` (new) — one resolver and one
reporter, used by all five toolchain files in that directory.

**Resolution order**, highest priority first:

1. `-DNROS_<PREFIX>_PREFIX=…` or the environment variable of the same name
   (`NROS_ARM_NONE_EABI_PREFIX`, `NROS_RISCV_NONE_ELF_PREFIX`, and the existing
   `NROS_RISCV64_PREFIX`), set to a compiler prefix — bare or absolute.
2. The SDK store (`$NROS_SDK_STORE`, else `$NROS_HOME/sdk`, else `~/.nros/sdk`)
   under `<tool>/<version>/bin/`, **newest version first** — the store
   accumulates (issue 0500), so a stale copy must not shadow the pin.
3. `PATH` — the documented distro fallback, unchanged and still supported.

Reading the store directly is what makes `nros setup` take effect in the shell
you are already in, and what makes the pin beat an unpinned distro copy.

**What it prints.** Provisioned:

```
-- nano-ros: arm-none-eabi-gcc 13.2.1 via SDK store — /home/…/.nros/sdk/arm-none-eabi-gcc/13.2-nros4/bin/arm-none-eabi-gcc (pin 13.2.rel1, store package 13.2-nros4)
```

Falling back:

```
-- nano-ros: arm-none-eabi-gcc 10.3.1 via system PATH — arm-none-eabi-gcc (pin 13.2.rel1, store package 13.2-nros4)
nano-ros: arm-none-eabi-gcc is UNPINNED — 10.3.1 from system PATH, not the SDK store.
          This tree pins 13.2.rel1, store package 13.2-nros4, per nros-sdk-index.toml.
          Supported (the documented system-toolchain fallback); this is a notice, not an error.
          To use the pin instead:  nros setup --tool arm-none-eabi-gcc
```

The pinned version is read out of `nros-sdk-index.toml` at configure time
rather than restated in cmake: an authored copy of a pinned version is a map
that drifts from its territory, which is what `check-rmw-api-parity` records at
length.

**The floor is a WARNING, not a refusal**, and the asymmetry is deliberate.
What is measured is that GCC < 11 cannot compile the generated C++ **entry**.
What is *not* measured is a pure-C cross image on GCC 10 — `nros-cpp` is a Rust
crate compiled by cargo, so a C-only workspace may hand the cross `g++` nothing
that trips the frontend bug. Refusing at configure would turn "supported
fallback" into "cannot build at all" on stock Ubuntu 22.04, the very host that
found this — resolving the tension by fiat, in the wrong direction. So the
warning does the thing that costs nothing and would have ended issue 1113 on the
spot: it quotes the error the user is about to get, and names it.

```
CMake Warning: nano-ros: arm-none-eabi-gcc 10.3.1 (via system PATH) is below the
  floor of GCC 11.
    It CANNOT compile the C++ entry TU nano-ros generates. If this image has a
    C++ entry, the build will fail later with
        error: C99 designator '<member>' outside aggregate initializer
    in *_entry_nros_main_generated.cpp. THAT ERROR IS THIS — a GCC 10 C++
    frontend limitation, not a codegen bug (issue 1117; the wrong diagnosis it
    caused is issue 1113). Raising -std= does NOT help …
```

A user on a working 12.x sees the provenance line and the UNPINNED notice, no
floor warning, and builds normally.

Promoting the floor to `FATAL_ERROR` is a one-constant change, and the evidence
that would justify it is specific: a measurement showing every nano-ros cross
image — C-only included — feeds a generated C++ TU to that compiler.

**Gate:** `check-cross-toolchain-provenance` (`just check
cross-toolchain-provenance`, fast lane). It requires every toolchain file under
`cmake/toolchain/` to route through the shared helpers, to set the compiler from
the resolved variable rather than a literal, to name a `[tool.*]` that exists in
the index, and — the part a grep for the helper name would miss — to have
`resolve()` and `report()` wired to **each other**. That last mutant keeps the
shape entirely valid: cmake accepts it and prints a provenance line about a
compiler the file does not configure.

## Found while fixing it: a build tree can outlive the resolution, silently

The first version of the fix printed the right line and was still wrong, in the
same way as the original bug. CMake bakes the chosen compiler into
`CMakeFiles/<ver>/CMake<LANG>Compiler.cmake` on the first configure and reloads
it on every later one; a toolchain file's plain `set(CMAKE_C_COMPILER …)` is a
NORMAL variable and does not displace it, and CMake raises nothing. Measured on
the build tree that found this issue — after the module landed, a re-configure
printed

```
-- nano-ros: arm-none-eabi-gcc 13.2.1 via SDK store — …
```

while every object in that directory was still compiled by `/usr/bin/arm-none-eabi-g++`
10.3.1, and the build failed with the same designator error as before. A
provenance line that describes a compiler the build does not use is worse than
no line.

`nros_cross_toolchain_report()` now reads the persisted
`CMake<LANG>Compiler.cmake` and refuses when it disagrees with what was
resolved. Note the near-miss: checking the *variable* `CMAKE_CXX_COMPILER`
looked correct and detected nothing, because a cross build has no such CACHE
entry (the toolchain file sets a normal variable, so nothing writes one) — the
check has to read the file.

This is one of the two states where "use a fresh build directory" is the honest
answer and not the `rm -rf` antipattern: CMake declares a compiler change
unrecoverable in place, and no dependency edge can express it.

## Open: four siblings outside `cmake/toolchain/`

Listed in the gate's `EXEMPT` table with this issue id, not fixed here (other
owners), and they have the identical defect:

| file | note |
| --- | --- |
| `packages/boards/nros-board-nuttx-qemu/armv7a-nuttx-toolchain.cmake` | bare `arm-none-eabi-gcc`; live, via `cmake/board/nano-ros-board-nuttx-qemu-arm.cmake` |
| `packages/boards/nros-board-nuttx-qemu/riscv-nuttx-toolchain.cmake` | same, riscv side |
| `scripts/qemu/arm-none-eabi-cortex-m3.cmake` | `find_program`, so it resolves — and still reports neither version nor origin |
| `integrations/s32ds/CMakeLists.txt` | writes a toolchain file at configure time (deliberately derived from the S32DS project); the emitted file hardcodes `arm-none-eabi-gcc` |

The gate's exemption list can only shrink: an entry whose file stops selecting a
compiler, or stops existing, fails the gate until the entry is deleted.

## Also found, on the machine that fixed this

`riscv64-unknown-elf-gcc` **10.2.0** is on this host's `PATH` while
`[tool.riscv-none-elf-gcc]` pins **14.2.0-3**. Every ThreadX riscv64 configure
here was four major versions below the pin, and — until this change — said
nothing about it either. It now prints the same UNPINNED notice and the floor
warning.
