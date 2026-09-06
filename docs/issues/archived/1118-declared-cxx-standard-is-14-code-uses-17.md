---
id: 1118
title: "The declared C++ standard is 14; the code needs 17, and only a warning held the difference"
status: resolved
type: bug
area: cmake, cpp, build
severity: medium
found: 2026-09-06
related: [1113, 1117]
---

# The declared minimum was a claim the code violated

`packages/api/nros-cpp/include/nros/component_node.hpp` uses `if constexpr` —
a **C++17** construct — at three sites in `adopt_launch_seed_` (a fourth behind
`NROS_CPP_STD`), around lines 566–583. Everything that declared a standard
declared **C++14**.

Measured, on this tree, before the fix:

| Vocabulary | Sites | Value |
| --- | --- | --- |
| `set(CMAKE_CXX_STANDARD n)` in a `CMakeLists.txt` | 75 | 14 |
| `target_compile_features(… cxx_std_n)` | 2 | 14 |
| `CONFIG_STD_CPPn=y` in a Zephyr `prj.conf` | 18 | 14 (1 already 17) |
| `-std=c++n` in a toolchain's `CMAKE_CXX_FLAGS_INIT` | 6 | 14 (1 already 17) |
| `set(CMAKE_CXX_STANDARD n)` emitted by a CLI scaffold | 3 | 14 |

Nothing failed, because **GCC only warns**:

```
$ c++ -fsyntax-only -std=c++14 -ffreestanding -DNROS_SYSTEM_PARAM_SERVICES \
      -include nros/component_node.hpp -x c++ /dev/null
component_node.hpp:566:12: warning: 'if constexpr' only available with '-std=c++17' or '-std=gnu++17'
component_node.hpp:571:19: warning: 'if constexpr' only available with '-std=c++17' or '-std=gnu++17'
component_node.hpp:576:19: warning: 'if constexpr' only available with '-std=c++17' or '-std=gnu++17'
```

## Why the gate that exists could not see it

This tree does have a C++14 gate: `just check cpp` runs ~15 `-std=c++14
-ffreestanding` syntax probes over every `nros-cpp` header. It was green
throughout, and still is.

It is green because the whole `adopt_launch_seed_` block sits behind
`#if defined(NROS_SYSTEM_PARAM_SERVICES)`, and that macro is defined in exactly
one place — `cmake/NanoRosCapabilities.cmake:75` — which no probe runs. The same
command **without** the `-D` above emits no diagnostics at all. So the gate was
not weak; it was aimed at a preprocessor state in which the offending code does
not exist.

That is also not a warning that would have been noticed in a build log: it is
three lines inside a `-Wall`-less cross build that already prints newlib
`_close is not implemented` warnings on every link.

## Why the degradation is not benign

At C++14 an `if constexpr` chain degrades to a runtime `if`/`else if`, so
**every arm is instantiated**. The `NROS_CPP_STD` arm's `T = std::string` then
meets the first arm's `def = v` with `v` a `bool`. `std::string` has
`operator=(char)` and `bool` converts to `char`, so this can compile and assign
a character rather than fail. The behaviour was correct only because the
declared standard was never the one the compiler used on the paths that
mattered (see the flag ordering below).

## The decision: C++17, and not C++20

C++17, because:

- the code already requires it;
- ROS 2 **Humble** is C++17 — `rclcpp` exports `cxx_std_17`, so a nano-ros node
  sharing headers with `rclcpp` is compiled at 17 regardless of what we say;
- the pinned cross toolchain (`arm-none-eabi-gcc 13.2.rel1`,
  `nros-sdk-index.toml`) is fully C++17;
- `just check.just:4288` already said so in prose — *"C++17 is what the
  component lane actually compiles with"* — while the declaration said 14.

Not C++20, because nothing in `nros-cpp` uses a C++20 feature, C++20 support
varies across the embedded toolchains this project targets, and there is a
**live external C++14 constraint**: PX4 builds every module with
`-std=gnu++14 -Werror` (see the comment on `kDomainIdExplicitZero` in
`node.hpp`, and phase-325 W2, where an `inline` variable made `<nros/nros.hpp>`
uncompilable in a PX4 module). C++20 needs its own survey; this is not it.

## The toolchain floor, established

`if constexpr` is GCC 7+; C++17 is feature-complete in GCC 8. Every toolchain
this tree can reach is well past that.

| Toolchain | Used by | Version | C++17 | How established |
| --- | --- | --- | --- | --- |
| host `g++` | native | 11.4.0 | yes | compiled a `if constexpr` + `static_assert(__cplusplus >= 201703L)` TU at `-std=c++17` |
| `arm-none-eabi-gcc` (nros SDK pin) | FreeRTOS, NuttX-arm | 13.2.1 (13.2.rel1) | yes | same TU, `-ffreestanding`; `__cplusplus 201703L`, `__cpp_if_constexpr 201606L` |
| `arm-none-eabi-gcc` (Ubuntu 22.04 system) | the shadow issue 1117 is about | 10.3.1 | yes | same, measured — the shadowing is a *version* problem, not a *standard* problem |
| `riscv-none-elf-gcc` (pin `14.2-nros1`) | ThreadX riscv64, NuttX-riscv | 14.2 | yes | **not provisioned on this host** — from the pin in `nros-sdk-index.toml` and GCC's C++17 completeness at 8 |
| Zephyr SDK `0.16.8` / `0.17.0-nros1` | Zephyr | GCC 12.2 / 14.x | yes | **not provisioned on this host**; Zephyr exposes `CONFIG_STD_CPP17` as a first-class Kconfig choice, and `examples/workspaces/realtime-cpp/src/fvp_entry/prj.conf` already selected it in this tree |
| ESP-IDF | esp32 | — | n/a | **not a constraint**: the esp32 lane compiles no C++. `examples/qemu-esp32-baremetal/` contains only `rust/`, and the single esp32 fixture row (`examples/fixtures.toml:324`) is `lang = "rust"` |

No toolchain fails the floor, so nothing changes the answer.

## One place or 75? Measured, and the answer is "both, for different reasons"

The temptation was to delete the 75 leaf `set(CMAKE_CXX_STANDARD 14)` lines and
let `target_compile_features(nros-cpp-headers INTERFACE cxx_std_17)` carry it.
Propagation does work:

```cmake
add_library(iface INTERFACE)
target_compile_features(iface INTERFACE cxx_std_17)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_library(consumer STATIC src.cpp)   # -> FLAGS = -std=gnu++17
target_link_libraries(consumer PUBLIC iface)
add_library(nolink   STATIC src.cpp)   # -> FLAGS = -std=gnu++14
```

But a compile FEATURE emits **no flag at all** when the compiler default
already satisfies it. Mimicking Zephyr's `zephyr_interface` (a raw `-std=c++14`
compile option) against `cxx_std_17` alone:

```
FLAGS = -std=c++14          # the feature contributed nothing; c++14 is the whole answer
```

Add the leaf's declaration back and CMake emits its own flag **last**, which is
what actually wins:

```
FLAGS = -std=c++14 -std=gnu++17
```

That is not hypothetical. It is the real compile line of the FreeRTOS C++ image
after this fix (`examples/qemu-arm-freertos/cpp/talker/build-zenoh/build.ninja`):

```
FLAGS = -mcpu=cortex-m3 -mthumb … -fno-rtti -std=c++14 -ffreestanding -O3 -DNDEBUG -std=c++17
         ^ from cmake/toolchain/arm-freertos-armcm3.cmake                            ^ from the leaf
```

So: **the compile feature is the requirement, the leaf declaration is the
enforcement**, and deleting the 75 would have left every cross-compiled C++ TU
silently back at C++14 with nothing able to raise it. Both stay. What does not
stay is the duplication being *unchecked* — see the gate.

## What changed

- `packages/api/nros-cpp/CMakeLists.txt` — `cxx_std_14` → `cxx_std_17`. **This
  line is now the floor's only home**; the gate reads the number out of it.
- `cmake/compat/diagnostic-updater/CMakeLists.txt` — `cxx_std_14` → `cxx_std_17`.
- 73 `CMakeLists.txt` — `set(CMAKE_CXX_STANDARD 14)` → `17`
  (71 under `examples/`, 2 PX4 host-compile test fixtures).
- 18 Zephyr `prj.conf` — `CONFIG_STD_CPP14=y` → `CONFIG_STD_CPP17=y`, plus the
  12 comments in them that said "compiles as C++14".
- `component_node.hpp` — a note at `adopt_launch_seed_` saying *this* is why the
  floor is 17, why the c++14 probes cannot see it, and that the fix if it ever
  has to go back is tag dispatch, not a lower floor.
- `span.hpp` / `size_bound.hpp` / `declared_qos.hpp` — comments corrected.
  `size_bound.hpp:51` said outright "`std::void_t` is C++17; nros-cpp is C++14",
  which is now false.
- New gate `check-cxx-standard-floor` (below).

## Two sites deliberately stay at C++14

`packages/rmw/uorb/nros-rmw-uorb/CMakeLists.txt` and
`packages/rmw/cyclonedds/nros-rmw-cyclonedds/CMakeLists.txt` were raised and
then put back, because the floor is *nros-cpp's requirement* and it binds
nros-cpp's **consumers**. Measured: neither backend includes any `nros/*.hpp`
nor links `NanoRos::NanoRosCpp`, so `if constexpr` never reaches either. What
does reach them is the opposite constraint:

- **uORB** is consumed by `add_subdirectory` from a PX4 module shell, and PX4
  builds every module `-std=gnu++14 -Werror`. `CXX_STANDARD_REQUIRED` would emit
  `-std=c++17` LAST and win over PX4's own flag, compiling this TU at a dialect
  the rest of the firmware is not built with — untested here, since there is no
  PX4 build on this host. Its own file said so and I had overwritten the
  comment while contradicting it.
- **Cyclone wrapper** ships into Zephyr / FreeRTOS, and the Zephyr lane compiles
  it at `-std=c++11` (issues 1011 / 1014, phase-416). Declaring 17 would assert
  a dialect that lane does not provide.

Both now carry `# nros-cxx-floor-exempt: <reason>` on the line above the
declaration, and the gate prints them on its green path. Putting the reason ON
the line, rather than in a `.config/` allowlist, is deliberate: the next person
to consider raising it reads why first.

## What deliberately did NOT change

**The `-std=c++14` probes in `just check cpp`.** They assert a *stricter*
property that survives the floor moving: every `nros-cpp` header outside the
`NROS_SYSTEM_PARAM_SERVICES` block is still written in the freestanding C++14
subset. That is what keeps `<nros/nros.hpp>` usable from a PX4 module at
`-std=gnu++14 -Werror`. `just check cpp` is green after this change, so the
property is enforced and not merely claimed. Two levels, both true:

- **declared minimum for building nano-ros: C++17**;
- **dialect the header set is written in: the freestanding C++14 subset**, minus
  one capability-gated block.

## Known remaining sites (not this change's to fix)

Two areas were owned by concurrent work and are untouched here. Neither changes
what any current target compiles at — the leaf declaration wins over both — but
each is a stale restatement that will mislead:

- `cmake/toolchain/*.cmake` — raw `-std=c++14` in `CMAKE_CXX_FLAGS_INIT`:
  `arm-freertos-armcm3.cmake`, `armv7a-nuttx-eabi.cmake`,
  `riscv32imac-nuttx-elf.cmake`, `riscv64-threadx.cmake`. Note
  `arm-freertos-armcr52.cmake` already says `-std=c++17`, so these four already
  disagreed with their own sibling. Same shape in
  `packages/boards/nros-board-nuttx-qemu/{armv7a,riscv}-nuttx-toolchain.cmake`
  and `packages/boards/nros-board-common/src/nuttx_ffi_build.rs:220,230`.
  A target that sets no standard of its own gets these — measured on the link
  line of the FreeRTOS image above.
- `packages/cli/` — the scaffold still EMITS 14, so every newly created package
  reintroduces the defect: `cargo-nano-ros/src/scaffold.rs:551,1270` and
  `nros-cli-core/src/orchestration/metadata_probe_cmake.rs:92`.
- Book/guide snippets users copy: `book/src/getting-started/{first-node-cpp,
  porting-a-cpp-node,your-own-msg-package}.md`, `docs/guides/{cpp-api,
  creating-examples}.md`.

## The gate

`check-cxx-standard-floor` (`scripts/check-cxx-standard-floor.py`, fast lane).

**It contains no version number.** It reads the floor out of
`target_compile_features(nros-cpp-headers INTERFACE cxx_std_NN)` and requires
every other declaration to be at or above it, so raising the floor is one edit
and the gate follows. A gate that restated the number would be the 96th
restatement of the thing it exists to stop.

Covers the three vocabularies that declare a standard *as a standard*:
`CMAKE_CXX_STANDARD`, the `cxx_std_N` compile feature, and `CONFIG_STD_CPPN`.
Each must be at or above the floor, or carry
`# nros-cxx-floor-exempt: <reason>` — required, more than a word, and printed
on the green path so "none declares below C++17" is not a false summary.
Each is bound to its file kind, so a Kconfig symbol quoted in a CMake comment
(which `NanoRosWorkspace.cmake:366` does) is not read as a declaration, and
CMake comments are stripped — otherwise the floor's own site, which explains the
value it replaced, would go red on the fix.

Out of scope, deliberately: a raw `-std=` in a toolchain's
`CMAKE_CXX_FLAGS_INIT` (a lane default, not a declared minimum — and provably
overridden by `CXX_STANDARD_REQUIRED`, above), and the `-std=c++14` compile
probes, which assert the stricter subset property on purpose.

Self-test runs on the normal path. Mutations exercised: one leaf back to 14
(caught, named); one Zephyr conf back to `CPP14` (caught — the vocabulary CMake
cannot see); the file-kind/vocabulary binding inverted (self-test red on all
three rules); comment stripping removed (self-test red). The shape-valid
crossed-wires case is in the self-test itself: every CMake declaration raised
and only the Kconfig one left low, so the tree-wide question "does 14 appear
anywhere" has the same answer as a fully-fixed tree. Two more on the escape:
the reason requirement removed (self-test red — a bare marker was accepted),
and the marker honoured anywhere in the file rather than on the line above
(self-test red — it leaked to a later declaration).

**Known limit, stated rather than hidden:** an already-exempt site can be
lowered FURTHER (14 → 11) and the gate reports the new number instead of
failing on it. That is the price of a prose reason; the number is on screen and
in the diff on every run.

## Verification

- `examples/native/cpp/talker` — configures, compiles, links.
  `build.ninja` carries `-std=gnu++17` (it was `gnu++14`); the artifact is an
  11 MB aarch64 ELF.
- `examples/qemu-arm-freertos/cpp/talker` cross-built with the pinned
  `arm-none-eabi-gcc 13.2.1` first on PATH — configures, compiles, links.
  507424 text / 3176 data / 3626544 bss, ELF 32-bit ARM. Compile line ends
  `-std=c++17`.
- `just check cpp`, `just check c`, `just check cxx-standard-floor`,
  `just check kconfig-overridden-values`, `just check cmake-image-policy`,
  `just check cmake-support-library`, `just check gate-lists`,
  `just check gate-selftests`, `just check gate-visibility`,
  `just check default-gates-run-somewhere`, `just check just-recipe-refs` — all
  green.
- **Not built here**: any Zephyr image (no Zephyr SDK and no west workspace on
  this host) and any ThreadX / NuttX-riscv image (`riscv-none-elf-gcc` not
  provisioned). The 18 `CONFIG_STD_CPP17` edits and the riscv lanes need the
  tier-2 / nightly matrix to confirm.
