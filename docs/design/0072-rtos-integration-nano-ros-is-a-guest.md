---
rfc: 0072
title: "RTOS integration: nano-ros is an imported library, and the RTOS owns its own build"
status: Draft
since: 2026-08
last-reviewed: 2026-08
implements-tracked-by: [phase-349, phase-351]
supersedes: []
superseded-by: null
---

# RFC-0072 — RTOS integration: nano-ros is an imported library

**Status.** Draft. Grounded in a survey of six real FreeRTOS distributions
(upstream, ESP-IDF, Raspberry Pi Pico SDK, STM32Cube, NXP MCUXpresso, Infineon
ModusToolbox) conducted 2026-08-12, and in the three integration shells nano-ros
already ships.

**Relates to.** RFC-0012 (board/BSP integration — Stable; this makes its Layer 3
uniform and refines its FreeRTOS row), RFC-0049 (hierarchical platform/board
config), RFC-0071 (provider descriptors), phase-348 (source-time discovery).

---

## 1. The principle, and the one place we break it

**nano-ros is a library the user's project imports. The RTOS owns its own
build.** This is already how three of five platforms work:

| host build | shell | shape |
| --- | --- | --- |
| Zephyr | `zephyr/` | `module.yml` + `Kconfig` + `CMakeLists.txt` — a west module |
| NuttX | `integrations/nuttx/` | `Make.defs` + `Kconfig` + `Makefile` — an `apps/external/` app; the staticlib lands on `EXTRA_LIBS` |
| ESP-IDF | `integrations/nano-ros/` | `CMakeLists.txt` + `idf_component.yml` + `Kconfig.projbuild` — an IDF component |

One shape in three languages: **build glue + Kconfig + the host's package
manifest.** In each case the RTOS kernel, its config header, its network stack
and its drivers belong to the host. nano-ros contributes a staticlib and its
generated code.

**FreeRTOS is the anomaly.** It has no shell. Instead
`cmake/platform/nano-ros-freertos.cmake` + `nros_freertos_build_kernel()` make
nano-ros compile the kernel itself from `FREERTOS_DIR` + `FREERTOS_PORT`. That
choice dates from when upstream FreeRTOS shipped no build system. It does now.

Everything else in this RFC follows from removing that anomaly.

## 2. Why the survey settles it

| SDK | linkable targets? | who owns `FreeRTOSConfig.h` | who owns the stack |
| --- | --- | --- | --- |
| upstream `FreeRTOS-Kernel` | yes — `freertos_kernel` | user, via a `freertos_config` INTERFACE target | separate repo |
| ESP-IDF | yes — `idf::freertos`, `idf::lwip` | **generated** from Kconfig | forked lwIP, welded in |
| Pico SDK | yes, but **renamed**; ignores `freertos_config` | per-example file | SDK hand-lists lwIP |
| STM32Cube | **none** — IDE project XML only | per-example (106 copies in F4) | submodule; `ethernetif.c` is app source |
| NXP MCUXpresso | **none** — no imported targets, no `find_package` | **generated**; filename comes from a Kconfig menu prompt | NXP's lwIP fork *contains* the netifs |
| Infineon MTB | none — make + `COMPONENT_<core>/` naming | per-core, shadowed by an app copy | config-only meta-library |

Six conventions for `FreeRTOSConfig.h`; five for port selection. NXP inverts
control outright — the *application* `include()`s the SDK root, and everything is
`mcux_add_source()` onto one target, so there is no library to link against.

Any design where nano-ros compiles the RTOS must reimplement all of that. Under
§1's principle we implement none of it.

## 3. The LIVE design — what the tree does today (surveyed 2026-08-13)

Documented before proposing changes, because two prior drafts of this RFC
re-derived things the tree already had.

### RFC-0003 already states this RFC's principle

[RFC-0003](0003-rtos-integration-pattern.md) §1 (Draft, 2026-06):

> *"Vendor SDK keep native build tool — west, make+Kconfig, cmake, idf.py, pio.
> Always. nano-ros never replace. nano-ros instead plug into vendor's external
> library / external module / component hook."*

and its universal rule: **vendor owns build + link; nano-ros owns a per-vendor
adapter shim plus host-time codegen.** §1 is this RFC's §1, three months
earlier. What this RFC adds is not the principle but **where board information
lives** — which RFC-0003 does not address.

RFC-0003 also supplies a taxonomy this RFC must respect:

| | vendors | codegen runs |
| --- | --- | --- |
| **hook-capable** | Zephyr, ESP-IDF, ThreadX, NuttX, FreeRTOS | at the vendor's configure phase |
| **hookless** | PlatformIO, **PX4** | *ahead* of the vendor tool, emitting a vendor-native tree |

A design that assumes a configure-time hook silently excludes PX4 and PIO.

### Where board information lives today — five homes

| home | count | holds | read by | when |
| --- | ---: | --- | --- | --- |
| `packages/boards/*/nros-board.toml` | 8 files, 9 boards | the descriptor | `BoardCatalog` (CLI) | `nros sync` / `setup` |
| `<bringup>/system.toml` `[image.<id>]` / `[host.<name>]` / `[board_config.<board>]` | 123 / 28 / 24 blocks | build description, placement, site config — one table each | `nros codegen system`, `nros build` | codegen |
| `<leaf>/.cargo/nros-board.toml` | 44 | projection of `cargo_config` | cargo | every build |
| `cmake/board/nano-ros-board-*.cmake` | 7 | toolchain file, kernel/stack targets | cmake, **by filename** | configure |
| `just/sdk-env.just` | — | `FREERTOS_DIR`, `LWIP_DIR`, … | build scripts, via shell env | build |

### The nine boards, and what each descriptor actually asserts

| board | platform | link_kind | entry_kind | net_stack | site content mixed in |
| --- | --- | --- | --- | --- | --- |
| linux | posix | none | hosted-main | nanoros-owned | — |
| esp32-qemu | esp32 | none | board-run | nanoros-owned | linker script |
| freertos | freertos | none | board-run | nanoros-owned | **runner**, linker script |
| baremetal | bare-metal | none | board-run | nanoros-owned | **runner**, linker script |
| nuttx | nuttx | nuttx-staging | board-run | rtos-owned | linker script, `${workspace}` |
| nuttx-riscv | nuttx | nuttx-staging | board-run | rtos-owned | `${workspace}` |
| threadx | threadx-linux | none | hosted-main | nanoros-owned | — |
| threadx | threadx-riscv64 | none | board-run | nanoros-owned | linker script, `${workspace}` |
| zephyr | zephyr | none | zephyr-staticlib | rtos-owned | — |

Three observations that shape the redesign:

1. **`entry_kind` already encodes RFC-0003's adapter shape** — `hosted-main`,
   `board-run`, `zephyr-staticlib`. The integration axis is present and named.
2. **`net_stack` is a real ownership axis** (`rtos-owned` for Zephyr/NuttX,
   `nanoros-owned` elsewhere) — but it is parsed and never read, and it does not
   say *which* stack.
3. **Six of nine descriptors carry site content.** `${workspace}` is stripped by
   a withholding filter that warns and discards — a filter with no destination,
   which is the signature of a missing category.

### The categories the tree is straining against

**A — board facts** (reusable, shippable by us, a vendor, or the user): identity,
platform, target triple, capabilities, entry shape, link kind.
**B — site/instance config** (this checkout, this machine, this app): SDK roots,
netstack choice, config-header dirs, runner, linker-script path.
**C — derived** (never authored): the leaf projection, the provider index, baked
headers.

A and B are mixed in the descriptor; B additionally lives in `just/sdk-env.just`,
which an out-of-tree user does not have. That is the whole problem.

### The mechanical constraints any carrier must satisfy (measured)

* **Cargo config discovery is CWD-based and merges ancestors, deepest wins.** A
  per-member `.cargo/config.toml` is read only when CWD is at/under that member.
* **Corrosion runs cargo from `workspace_toml_dir`** — the workspace root for a
  member — so per-member config is invisible to it.
* **`[target.<triple>]` is board-disambiguating; `[env]` is not.** The workspace
  root of `examples/workspaces/rust` carries an mps2-an385 `runner` and linker
  script for a workspace that also holds nuttx/esp32/threadx/zephyr entries; it
  is safe *only* because cargo applies `[target.*]` per triple.
* **The consumer is a dependency.** `zpico-sys`'s build script has
  `CARGO_MANIFEST_DIR` four levels under the repo root, so no walk-up reaches a
  leaf.
* **Exactly one board is active per configure** — cmake selects entries with
  `if/elseif` on `NANO_ROS_BOARD`, and the toolchain file must be fixed before
  `project()`. So the *source* must describe N boards while *delivery* carries 1.

## 4. What six ecosystems agree on — and the revised model

Surveyed 2026-08-12/13: upstream FreeRTOS, ESP-IDF, Pico SDK, STM32Cube, NXP
MCUXpresso, Infineon MTB, Eclipse ThreadX + ST X-CUBE-AZRTOS, PX4.

### The convergent model

Every one of them separates a **board directory** from **the user's project**,
and none of them lets the user's project edit the board. PX4 is the cleanest
instance and worth copying outright:

```
boards/<vendor>/<board>/          ← everything board-intrinsic
  default.px4board                  Kconfig defconfig (the board's baseline)
  <label>.px4board                  variants, stored as DELTAS over default
  nuttx-config/<cfg>/defconfig      the RTOS config, owned by the BOARD
  nuttx-config/scripts/*.ld         memory map
  nuttx-config/include/board.h      clocks, pinmux, DMA map
  src/*.cpp                         bring-up (the `drivers_board` library)
  init/rc.board_*                   board default parameters
  firmware.prototype                flashing identity (board_id, image_maxsize)
  cmake/upload.cmake                board-idiomatic flashing (18 boards ship one)
```

and the user supplies exactly two things: `EXTERNAL_MODULES_LOCATION=<abs path>`
and *which board×label to build*. Nothing else.

Three properties worth stealing:

1. **A board exists because a file exists.** No registry — the target list is
   `find boards -name '*.px4board'`. That is phase-348's discovery rule,
   arrived at independently.
2. **Variants are deltas over a default**, merged by `merge_config.py`. That is
   RFC-0049's ladder, arrived at independently.
3. **The board owns the RTOS config.** `make … savedefconfig` writes back *into*
   `boards/<v>/<b>/nuttx-config/<cfg>/defconfig`. The RTOS is a submodule the
   board configures — not something the project reconfigures.

### The refined categories

The survey corrects two things in §2b's A/B split.

**Flashing is board-intrinsic; simulation is not.** PX4 puts `cmake/upload.cmake`
in the board dir but keeps simulator runners in
`src/modules/simulation/…/sitl_targets_*.cmake`. That resolves the runner
question: the **mechanism** by which a board is programmed (DFU, ST-LINK,
`rsync` to a Pi) is a board fact; the **instance parameters** (`AUTOPILOT_HOST`,
which serial port, which probe serial) are project facts; and an **emulator
invocation is test-harness configuration**, belonging to neither. nano-ros's
QEMU strings are the third kind, which is why they sit awkwardly in board
descriptors today.

**Some facts are jointly owned.** ThreadX/ST shows a linker script carrying both
the memory map (board) and the driver's DMA sections — `.RxDescripSection` must
land in a DMA-reachable region (board *constrains*) at a placement the project
*chooses*, spelled differently per toolchain (GCC attribute, IAR `@`, Keil a
hard-coded `at(0x24030000)`). A pure A/B split cannot express this; the board
must be able to state a **constraint** that the project satisfies.

### The compatibility fact a free choice cannot express

NetX Duo ships a **smaller port table than ThreadX** — 24 arches against 47. A
ThreadX arch with no NetX counterpart **cannot be paired**. So "the user
declares the netstack" (§3.3) is too permissive: the pairing has a validity
domain, and the honest form is that a provider declares which platforms it
supports and the resolver rejects the rest with a list of what is available.

### The PX4 constraint that changes our integration story

> *"Everything a board declares is Kconfig-selectable per label; nothing an
> external module provides is."*

An external module gets **one lever** — `EXTERNAL_MODULES_LOCATION`,
all-or-nothing — and one insertion point, `add_subdirectory` at
`CMakeLists.txt:454`, which is **before** `src/lib` and `platforms`. Consequences
we must design around rather than discover:

* nano-ros **cannot be per-board selectable** on PX4 as an external module. To
  get a `<label>.px4board` opt-in, code must be in-tree — which is exactly what
  PX4 did for zenoh (`src/modules/zenoh/` + `boards/px4/fmu-v6x/zenoh.px4board`,
  with `zenoh-pico` as a PX4-forked submodule).
* `DEPENDS` on `px4_work_queue`/`cdr` etc. **hard-errors** from an external
  module, because those targets do not exist yet at line 454. Only
  `uorb_headers`, `git_*` and self-defined targets are available; a later
  `target_link_libraries()` still works.
* The `EXTERNAL` keyword on `px4_add_module()` is parsed and **never read**, and
  `external_module_paths` is accumulated and never read. Neither is a hook.

This is RFC-0003's *hookless vendor* case, now with the precise reason: not that
PX4 lacks a configure-time hook, but that its configuration namespace is closed
to out-of-tree contributors.

## 5. The revised design, optimised for UX

### One rule, borrowed because users already know it

**A board is a directory. It exists because it exists.** No registry, no central
edit — PX4's rule, and phase-348's scan already implements it. A user adds a
board by creating a directory in their workspace; nothing in nano-ros changes.

### What a board package contains (category A)

Everything the six ecosystems agree is board-intrinsic:

```
<ws>/src/my_board/
  package.xml            <nano_ros_provides kind="board" name="nucleo-h723zg"/>
  nros-board.toml        identity, platform, target triple, capabilities,
                         entry shape, toolchain file, arch profile,
                         supported netstacks, flashing MECHANISM,
                         memory constraints the project must satisfy
```

Purged of: runner strings, `${workspace}` paths, config-header locations —
everything §2b measured as site content in six of nine descriptors.

### Where site facts go (category B) — no new file

The bringup's `system.toml` **already exists** and is already read at codegen
time, so the site keys go in it rather than in a sibling file nobody would
discover. They go in a table keyed by the BOARD, because that is what the fact
is about:

```toml
[board_config."nucleo-h723zg"]                       # (B)
sdk.cube      = "{env:CUBE_PROJECT}"      # machine path, env-interpolated
netstack      = "lwip"
config_files  = { tx_user = "Core/Inc/tx_user.h", nx_user = "Core/Inc/nx_user.h" }
upload        = { port = "/dev/ttyACM0" }  # instance params; MECHANISM is the board's
```

The board and what is built for it are declared separately, in the table that
owns each — `[image.<id>] board = "nucleo-h723zg"` says what to build, and the
descriptor supplies the rustc triple, so no block restates it.

This satisfies the SSoT requirement (one file, per workspace, greppable),
survives multi-board workspaces (keyed per board, so a workspace with two
FreeRTOS boards cannot reach the wrong block), and needs no new discovery path.
`config_files` is a **named map** rather than a directory, because ThreadX needs
`TX_USER_FILE` *and* `NX_USER_FILE` and FreeRTOS+TCP needs `FreeRTOSConfig.h`
*and* `FreeRTOSIPConfig.h`.

#### Amendment 1 — the key is the BOARD, not the deploy (issue 0951)

`[deploy.<name>.nros]` was the shipped spelling for three phases. It is now
`[board_config.<board>]` in the same file: same struct, same file, same
`{env:…}` interpolation, same named `config_files` map.

*(This amendment originally said "the rest of this section stands unchanged".
It did not — the section's block count, its worked example, its sequencing step
and all eight §6 examples still showed `[deploy.*]`. They were rewritten on
2026-08-31 when `[deploy.*]` reached zero blocks tree-wide. Fixing the key
without the prose left the page teaching the shape it had just retired.)*

What was wrong was one clause above — "already keyed per board". A deploy name
is *usually* a board name — the `nucleo-h723zg` block this section used to show
was one of the cases where it is, which is exactly why the clause read as true.
But the two are not in bijection, and the tree drifted apart in both directions:
`[deploy.threadx-linux]` pairs with `[image.threadx]`, `[deploy.an536]` with
`[image.mps3_an536]`, and several workspaces carry BOTH `[deploy.freertos]` and
`[deploy.mps2-an385-freertos]` for one board.

The cost was measured rather than argued: **30 authored site blocks held
exactly THREE distinct value-sets**, because a block had to be repeated under
every deploy name that reached its board. `board_facts` grew a comparison that
refused two blocks disagreeing about one board — detection standing in for a
shape that should not be representable. Keying on the board removes the shape:
`[board_config.freertos]` and `[board_config."mps2-an385-freertos"]` resolve to
one block through `BoardCatalog::resolve_deploy`, the same rule every other
board spelling goes through (issue 0606).

This also detaches category B from the `[deploy.*]` retirement. The original
argument for putting site keys on `[deploy.*]` was that the table "already
exists" — which stops being a reason once that table is being taken apart into
`[host.*]` (placement) and `[image.*]` (build). Site config belongs to neither,
and a board-keyed table is what lets it outlive both.

### Test-harness config is a third thing

QEMU invocations are neither board nor project — they are how *our tests* run a
payload. They move to the test harness, keyed by board, leaving both the board
package and the user's deploy block clean. A user with real hardware never sees
them.

### The UX this buys

| profile | what they write | what they never touch |
| --- | --- | --- |
| our fixtures | nothing | — |
| Zephyr / NuttX / ESP-IDF | one manifest line (unchanged) | board packages |
| vendored FreeRTOS or ThreadX, CMake host | one board package + one `[image.*]` block | nano-ros itself |
| IDE host (CubeIDE, IAR, MTB) | the same, plus `nros emit` | — |
| PX4 | `EXTERNAL_MODULES_LOCATION` | everything else — and they get no per-board opt-in (§2c) |

The measurable win is against what the vendors make users do today: ST
duplicates one memory map across **twelve** application directories, three
linker-script dialects and three startup files per application. A first-class
board package is something **neither upstream ThreadX nor ST provides** — so
this is new value, not repackaging.

### Sequencing

1. `[board_config.*]` carries the site keys; `nros config explain` reports file, section
   and rung. Nothing moves yet.
2. Generate this repo's site values from `just/sdk-env.just`; both live, gated
   for agreement (the phase-347 pattern).
3. Move runner / linker / `${workspace}` out of descriptors; delete the
   withholding filter, which then has nothing to filter.
4. Board packages declare **supported netstacks**; the resolver rejects an
   unsupported pair with the list (the NetX Duo lesson).
5. Retire the interim `[env] NROS_BOARD_TOML` row.

## 6. Worked examples, one per integration shape

The site block **shrinks as the RTOS takes over**. That gradient is the design
working: nano-ros asks for exactly what the host does not already provide.

### 6.1 Linux / native — the degenerate case

```toml
# packages/boards/linux/nros-board.toml                    (A — exists today)
[[board]]
names = ["linux", "native", "posix"]
platform = "posix"
target = "x86_64-unknown-linux-gnu"
entry_kind = "hosted-main"        # nano-ros owns main()
link_kind  = "none"
net_stack  = "host"               # the OS has sockets; nothing to choose
```
```toml
# <bringup>/system.toml                                     (B)
[image.native]
board = "native"
```

No SDK root, no netstack, no flashing. Every other row is a delta from this.

### 6.2 FreeRTOS on MPS2-AN385 — our fixture

```toml
# packages/boards/nros-board-mps2-an385-freertos/nros-board.toml   (A)
[[board]]
names = ["freertos", "mps2-an385-freertos"]
platform = "freertos"
target = "thumbv7m-none-eabi"
arch = "cortex-m3"
entry_kind = "board-run"          # the board crate owns main()
net_stack  = "nanoros-owned"
supported_netstacks = ["lwip", "freertos_plus_tcp"]     # the pairing domain
[board.cmake]
toolchain_file = "cmake/toolchain/arm-freertos-armcm3.cmake"   # must precede project()
```
```toml
[image.freertos]
board = "mps2-an385-freertos"

[board_config."mps2-an385-freertos"]                       # (B)
netstack     = "lwip"
sdk.freertos = "{env:FREERTOS_DIR}"
sdk.lwip     = "{env:LWIP_DIR}"
config_files = { freertos = "boards/mps2/FreeRTOSConfig.h", lwip = "boards/mps2/lwipopts.h" }
```
```toml
# test-harness config                                       (C)
[run.mps2-an385-freertos]
emulator = "qemu-system-arm -cpu cortex-m3 -machine mps2-an385 -nographic …"
```

That last block is the `runner` **removed** from the descriptor. It is duplicated
across `baremetal` and `freertos` today precisely because it describes the
*machine*, not the board package.

### 6.3 Vendored FreeRTOS — STM32Cube, the real user

The board package is **theirs**, in their workspace, and holds no paths:

```toml
# my_ws/src/nucleo_h723zg/nros-board.toml                   (A — user-authored)
[[board]]
names = ["nucleo-h723zg"]
platform = "freertos"
target = "thumbv7em-none-eabihf"
arch = "cortex-m7"
entry_kind = "board-run"
supported_netstacks = ["lwip"]
[board.upload]
mechanism = "st-link"             # HOW this board is programmed — a board fact
```
```toml
[image.nucleo]
board = "nucleo-h723zg"

[board_config."nucleo-h723zg"]                       # (B)
netstack = "lwip"
sdk.cube = "{env:CUBE_PROJECT}"
include_dirs = [
  "{sdk.cube}/Middlewares/Third_Party/FreeRTOS/Source/include",
  "{sdk.cube}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F",
  "{sdk.cube}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
  "{sdk.cube}/Middlewares/Third_Party/LwIP/src/include",
  "{sdk.cube}/Core/Inc",
]
upload = { port = "/dev/ttyACM0" }      # instance params — a project fact
```

Three things this states that no derivation could: the port directory is
`ARM_CM4F` **on a Cortex-M7** (ST's own examples do this); `Core/Inc` supplies
*both* config headers because they are application files; and the CMSIS-RTOS
directory is named **once**, so the two files both called `cmsis_os.h` cannot be
resolved by accident of `-I` order.

### 6.4 Zephyr — the RTOS owns everything

```toml
[[board]]                                                  # (A)
names = ["zephyr"]
platform = "zephyr"
entry_kind = "zephyr-staticlib"   # a west module, not main()
net_stack  = "rtos-owned"         # zsock; nothing for us to choose
```
```toml
[image.zephyr]
board = "zephyr"

[board_config."zephyr"]                       # (B)
zephyr_board = "qemu_cortex_m3"   # west's namespace, not ours
```

No `sdk.*`, no `config_files`, no `netstack`: Zephyr owns the stack and its
Kconfig owns the knobs.

### 6.5 NuttX — RTOS-owned, but we stage into its tree

```toml
[[board]]                                                  # (A)
names = ["nuttx", "NuttX"]
platform = "nuttx"
target = "armv7a-nuttx-eabihf"
entry_kind = "board-run"
link_kind = "nuttx-staging"      # we land on EXTRA_LIBS
net_stack  = "rtos-owned"
```
```toml
[image.nuttx]
board = "nuttx"

[board_config."nuttx"]                       # (B)
sdk.nuttx      = "{env:NUTTX_DIR}"
sdk.nuttx_apps = "{env:NUTTX_APPS_DIR}"
```

This is where today's `${workspace}/third-party/nuttx/libc` goes — it was always
a site fact, which is why the withholding filter had to strip it.

### 6.6 PX4 — the hookless case, and what the design must NOT promise

```toml
[[board]]                                                  # (A)
names = ["px4-fmu-v6x"]
platform = "nuttx"
entry_kind = "px4-module"
link_kind = "px4-external"
net_stack  = "rtos-owned"
```
```toml
[image.px4]
board = "px4-fmu-v6x"

[board_config."px4-fmu-v6x"]                       # (B)
px4.dir    = "{env:PX4_DIR}"
px4.target = "px4_fmu-v6x_default"        # <vendor>_<model>_<label>
```

PX4's configuration namespace is **closed to out-of-tree code**: everything a
board declares is Kconfig-selectable per label; nothing an external module
provides is. So no block in `system.toml` **can** produce a `<label>.px4board`
opt-in, and codegen must run *ahead* of the vendor tool (RFC-0003's hookless
path), emitting module dirs into `$PX4_DIR/src/modules/`. Per-board selection
would require being in-tree, as PX4 itself did for zenoh.

### 6.7 ThreadX — one name, two integrations

Today `threadx` is **two** descriptor entries with different `platform` and
`entry_kind`. The revision makes that explicit rather than accidental:

```toml
[[board]]                                                  # (A) host simulator
names = ["threadx-linux"]
platform = "threadx-linux"
target = "x86_64-unknown-linux-gnu"
entry_kind = "hosted-main"

[[board]]                                                  # (A) RISC-V target
names = ["threadx-riscv64"]
platform = "threadx-riscv64"
target = "riscv64gc-unknown-none-elf"
entry_kind = "board-run"
supported_netstacks = ["netxduo"]
```
```toml
[image.threadx_rv64]
board = "threadx-riscv64"

[board_config."threadx-riscv64"]                       # (B)
sdk.threadx = "{env:THREADX_DIR}"
sdk.netxduo = "{env:NETXDUO_DIR}"
config_files = { tx_user = "boards/rv64/tx_user.h", nx_user = "boards/rv64/nx_user.h" }
```

`config_files` is a **named map** because of this row: ThreadX needs
`TX_USER_FILE` *and* `NX_USER_FILE`, and one `config_dir` cannot express it.

### 6.8 ESP32 — IDF component

```toml
[[board]]                                                  # (A)
names = ["esp32-qemu", "esp32", "esp32c3"]
platform = "esp32"
target = "riscv32imc-unknown-none-elf"
entry_kind = "board-run"
net_stack = "rtos-owned"          # esp_netif over IDF's lwIP fork
```
```toml
[image.esp32]
board = "esp32-qemu"

[board_config."esp32-qemu"]                       # (B)
idf.dir = "{env:IDF_PATH}"
```

No `config_files`: IDF **generates** `FreeRTOSConfig.h` and `lwipopts.h` from
`sdkconfig`, so the site block must not pretend to own them.

## 7. Build process

### 5.1 Host build is CMake

```
 1  host configure       user's project (CubeMX, NXP, IDF, Pico, plain CMake)
 2  import               add_subdirectory(nano-ros) — or the west/IDF/NuttX shell
 3  discover             nros ws providers → board pkg → platform + integration
 4  resolve              capabilities merged; required-vs-provided checked
 5  contribute           integration supplies include dirs / defines / targets
 6  select port          backend descriptor picks its sources from (rtos, netstack)
 7  compile              Rust staticlib  +  C shim  +  backend port
                         ── all with the HOST's include dirs and defines ──
 8  link                 user links NanoRos::NanoRos
```

Steps 6–7 are the load-bearing ones. The backend's port sources and the C shim
must see the host's config headers; that is the whole reason nano-ros cannot
ship a prebuilt C half, and why the include dirs in §4.2 are the real contract.

### 5.2 Host build is not CMake

Steps 1–6 run under `nros emit`, step 7 happens inside the user's IDE project,
step 8 is their linker settings. The generated `README-INTEGRATION.md` names
their paths because it is rendered from their integration.

### 5.3 What fails, and how

* **A required capability nothing provides** → configure error naming the
  capability, the platform that required it, and the integration that did not
  supply it.
* **A `(rtos, netstack)` pair the backend has no port for** → error listing the
  pairs that backend does support. This is how "FreeRTOS+TCP is unreachable"
  becomes a message instead of a silent mis-pairing.
* **An include dir that does not exist** → checked at configure, named. The ST
  case makes this essential: six paths, several of them submodules that may be
  uninitialised.

## 8. Configuration method

RFC-0049's ladder is unchanged:

```
builtin  <  platform toml  <  board toml  <  env
```

Additions:

* **Capabilities merge along the same ladder.** An integration asserts what the
  host provides.
* **A vendor build's own config may be a knob source.** ESP-IDF and NXP generate
  their config headers from Kconfig; nano-ros already reads Zephyr's `.config`
  via `kconfig_fallback_str`. An integration may name where the host's resolved
  config lives, so knobs resolve from it instead of being restated. Optional,
  and worth it only for the Kconfig-based SDKs.

Two rules taken from ESP-IDF's documented mistakes:

* **Config visibility is not build membership** — "a `CONFIG_*` option being set
  does not mean the component that defines it is part of the build". Separate
  predicates, separately named.
* **Record provenance on override.** IDF keeps a documented total order
  (`COMPONENT_SOURCE`) *and* `COMPONENT_OVERRIDEN_DIR` so the winner can reach
  the loser. That is stronger than phase-348 W5's "warn with both paths" and
  should replace it.

## 9. What changes in-tree

1. `config/freertos-lwip/` → `config/freertos/`, with
   `names = ["freertos", "freertos-lwip"]`.
2. The six lwIP-specific lines move into the zenoh backend descriptor as a
   `(rtos, netstack)` port row; `freertos_plus_tcp` gains a row and becomes
   reachable.
3. `[build.zenoh]` leaves every platform file. Platforms keep `[capabilities]`,
   `[arch.*]`, `compile`, `required_env`.
4. `nros_freertos_build_kernel()` and `nros_freertos_build_lwip()` are retired;
   the mps2 fixture adopts upstream's `CMakeLists.txt`.
5. `FREERTOS_PORT` stops being ours. Upstream owns that name and takes an enum
   (`GCC_ARM_CM3`); we currently take a path fragment (`GCC/ARM_CM3`) under the
   same name, which fails confusingly for anyone arriving from upstream docs.
6. A `freertos` integration shell joins the other three, so the FreeRTOS row
   stops being the exception.

## 10. Maintainability

**Vendor knowledge lives in the user's workspace.** We ship descriptors only for
what we test, and never chase six config conventions across five SDK release
trains. NXP alone pins FreeRTOS across two west repos joined by a hard-coded
sibling path — not something we could track, and under this design we never try.

What nano-ros owns shrinks to: the capability vocabulary, the integration
shells, the backend port tables, and its own code.

## 11. Open questions

1. **Capability vocabulary.** `threads`, `sockets_bsd`, `select`,
   `per_fd_tx_ceiling` exist informally. Needs one authoritative list plus a
   gate that every declared capability is consumed by something.
2. **`generic` platform dir** — nothing inherits from it and `inherits` is unset
   in all seven files. Delete or document.
3. **`esp32` has no platform config** at all, so it declares no requirements.
4. **Prebuilt Rust staticlib distribution** (§4.3) needs a triple × feature
   matrix decision; out of scope here.
5. **Does `board.integration` stay inline?** One file is better UX; a shared
   `stm32cube-h7` integration reused across boards argues for factoring it into
   its own discovered provider package. Inline first, factor when a second board
   needs it.

## 12. Prerequisites

Both are RESOLVED (2026-08-13), so this work inherits neither.

* **`FREERTOS_PORT`'s two vocabularies** (§7.5) —
  [issue 0530](../issues/archived/0530-freertos-port-two-vocabularies.md). The
  builder now accepts upstream's enum as well as our path fragment, so §7.5's
  migration to upstream's `CMakeLists.txt` changes nothing for anyone already
  writing `GCC_ARM_CM3`.
* **Zephyr unselectable by the zpico resolver** —
  [issue 0529](../issues/archived/0529-zephyr-platform-knobs-never-resolve.md). The
  resolver gap was real and is fixed.

  **The severity asserted in this section's first draft was wrong.** It claimed
  `config/zephyr/nros-platform.toml`'s `[knobs.zenoh.tx] batch = true` — the
  phase-290 promotion measured at 15–20× streaming — never applies. It does
  apply: the C lane gets it from `zephyr/Kconfig` defaults forwarded by
  `nros_rmw_zenoh.cmake`, and there is no ABI split either, because
  `build_c_shim` is skipped on Zephyr and `rust_consts()` never emits
  `tx_batch`. The real defect was two sources for one fact agreeing only by
  coincidence, now compared by `check-zephyr-knob-agreement`.
## Appendix A — the earlier framing, and why it changed

Kept rather than deleted: two drafts of this RFC reached conclusions the survey
later corrected, and the corrections are the useful part.

**Draft 1 led with three integration modes** (`extern` / `adopt` / `build`),
treating "nano-ros compiles the RTOS" as one option among three. That is
over-modelled — it is the anomaly to delete, not a supported mode. §1 replaced
it with the principle.

**Draft 2 put `[board.integration]` in the board descriptor.** Every field in it
(`rtos`, `netstack`, `include_dirs`, `defines`) is site configuration, so it
belonged in the project, not the board package. §5 relocates it to
`[board_config.<board>]` (via `[deploy.<name>.nros]`, which Amendment 1
re-keyed).

**Draft 2 also proposed a `netstack` provider kind with selection.** The survey
found no vendor leaves the stack selectable — NXP's lwIP fork *contains* the
ethernetif drivers — and that NetX Duo ships a smaller port table than ThreadX,
so pairings have a validity domain rather than being a free choice. §4 replaces
selection with a `supported_netstacks` declaration the resolver checks.

**Draft 3 nearly invented `nros-site.toml`** beside `[deploy.<name>]`, which
already existed with 59 blocks in the tree.

The superseded section bodies follow, for anyone tracing why a decision moved.

### A.1 — superseded "Overall design" (draft 2)


### 3.1 Four things, each owning one fact

```
platform   — facts about the RTOS family.        Names no backends, no stacks.
board      — a concrete target: platform + arch + which integration.
integration— how the host build reaches us: include dirs, defines, link targets,
             and the FACTS the user has already decided (rtos, netstack).
backend    — an RMW. Owns its OWN port selection, keyed by (rtos, netstack).
```

### 3.2 Capabilities are the contract

nano-ros code keys on *what is true*, never on *which implementation*:

```toml
# config/freertos/nros-platform.toml
names = ["freertos", "freertos-lwip"]      # alias keeps old spellings resolving
requires_capabilities = ["threads", "sockets_bsd"]
```

This is ESP-IDF's own answer — `ESP_NETIF_USES_TCPIP_WITH_BSD_API` is a hidden
bool `select`ed by whichever stack provides it, so consumers test the capability
and the choice names the implementation. nano-ros already has `[capabilities]`
tables; today only `zephyr` populates one.

### 3.3 The network stack is a user-declared FACT, not a platform identity

`freertos-lwip` collapses two orthogonal facts into one name. **Our own
dependency models this better than we do:**

```
zenoh-pico/src/system/freertos/
  system.c                     ← the RTOS half
  lwip/network.c               ← stack half
  freertos_plus_tcp/network.c  ← stack half
```

zenoh-pico ships **both** FreeRTOS stacks, split along exactly this axis. The
cost of collapsing it is concrete: **FreeRTOS+TCP is vendored in our tree and
unreachable** — its only mention under `config/` is a comment. That comment also
documents a hand-tuned cross-pairing (we compile the *lwIP* `network.c` while
zenoh-pico's dispatch selects the *freertos_plus_tcp* platform header, justified
as ABI coherence), which is the sort of splice one makes when an axis exists in
a dependency but not in the model. XRCE has the same axis independently
(`UCLIENT_PLATFORM_FREERTOS_PLUS_TCP`).

So: **platform is `freertos`. The stack is stated by whoever knows it — the
user, based on the board they have.**

### 3.4 Platforms name no backends

`[build.zenoh]` should not live in a platform file. *Which zenoh-pico sources to
compile for FreeRTOS+lwIP* is a fact about zenoh-pico, and RFC-0071 already gave
backends their own descriptors. So:

```toml
# packages/rmw/zenoh/nros-rmw-zenoh/nros-rmw.toml
[[rmw.port]]
rtos = "freertos"
netstack = "lwip"
sources = ["{src}/system/freertos/system.c", "{src}/system/freertos/lwip/network.c"]
defines = ["ZENOH_FREERTOS_LWIP"]

[[rmw.port]]
rtos = "freertos"
netstack = "freertos_plus_tcp"
sources = ["{src}/system/freertos/system.c",
           "{src}/system/freertos/freertos_plus_tcp/network.c"]
```

The platform keeps only what is genuinely about the RTOS and the CPU:
`[capabilities]`, `[arch.*]`, `compile`, `required_env`. It stops knowing that
zenoh exists.

This extends phase-347's *core names no backends* to **platforms name no
backends**, and makes adding FreeRTOS+TCP a row in a descriptor rather than a
new platform directory.

### 3.5 No `netstack` provider kind, and no selection mechanism

The user declares a fact; nano-ros does not choose. Every vendor has already
welded its stack (NXP's lwIP fork *contains* the ethernetif drivers, with a
`lwipopts.h` that `#include`s `fsl_device_registers.h`), so there is nothing for
a selector to select. Capabilities give the decoupling. A selection mechanism
can wait for a genuine second stack on one RTOS — declaring the second tenant
before the first asks is the mistake the `[knobs.*]` deferral avoided.

### A.2 — superseded worked examples (draft 2)


### 4.1 The user already has an RTOS project

That is the premise. They are not starting from nano-ros; they have a Cube,
MCUXpresso, IDF, west or Makefile project that builds and runs. The workflow
adds nano-ros to it.

```
1. describe the target      one board package in their workspace
2. nros sync                discovery + codegen
3. import                   the shell for their host build system
4. build                    their build, unchanged, now linking nano-ros
```

Step 1 is the only new authoring, and it is one file.

### 4.2 What they write — vendored FreeRTOS, CMake host

```
my_robot_ws/
  src/
    my_nucleo_board/
      package.xml           <nano_ros_provides kind="board" name="nucleo-h723zg"/>
      nros-board.toml
    my_node/                their nano-ros node package
```

```toml
# src/my_nucleo_board/nros-board.toml
[[board]]
names    = ["nucleo-h723zg"]
platform = "freertos"
arch     = "cortex-m7"

[board.integration]
# FACTS the user already decided, by having the board they have.
rtos     = "freertos"
netstack = "lwip"
capabilities = { threads = true, sockets_bsd = true }

# What nano-ros needs to COMPILE against. No sources, no port selection,
# no kernel build — their Cube project already builds all of that.
include_dirs = [
  "{env:CUBE_PROJECT}/Middlewares/Third_Party/FreeRTOS/Source/include",
  "{env:CUBE_PROJECT}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F",
  "{env:CUBE_PROJECT}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
  "{env:CUBE_PROJECT}/Middlewares/Third_Party/LwIP/src/include",
  "{env:CUBE_PROJECT}/Middlewares/Third_Party/LwIP/system",
  "{env:CUBE_PROJECT}/Core/Inc",   # FreeRTOSConfig.h AND lwipopts.h live here
]
defines = ["USE_HAL_DRIVER", "STM32H723xx"]
```

```cmake
# their CMakeLists.txt
add_subdirectory(/opt/nano-ros nano_ros)
target_link_libraries(my_firmware PRIVATE NanoRos::NanoRos)
```

Three things this gets right that `FREERTOS_DIR` + `LWIP_DIR` could not:

* the **port directory is stated, not derived** — ST's Cortex-M7 H7 examples
  really do use `ARM_CM4F`, so deriving it from `-mcpu` is wrong;
* `Core/Inc` supplies **both** config headers, because they are application
  files, not SDK files;
* the CMSIS-RTOS directory is listed explicitly and once, so the two different
  files both named `cmsis_os.h` cannot be resolved by accident of `-I` order.

### 4.3 The same user on STM32CubeIDE (no CMake)

nano-ros cannot join an Eclipse build, so it emits a drop-in:

```
nros emit --board nucleo-h723zg --out ./nano_ros_drop/
```

```
nano_ros_drop/
  include/                 add to the include path
  src/                     C shim + zenoh-pico port — add as a source folder
  lib/libnros_rust.a       Rust staticlib, prebuilt for the triple
  nano_ros.mk              file list + defines, for make-based SDKs
  README-INTEGRATION.md    the exact steps, with their paths
```

The split is forced, not stylistic: the **Rust** half talks only through the
stable C ABI (RFC-0054) and prebuilds per triple; the **C** half must be
compiled by their project so it sees *their* `FreeRTOSConfig.h` and
`lwipopts.h`. Compiling it against anything else is issue 0135's silent ABI
break.

### 4.4 Other hosts

| host | what they do |
| --- | --- |
| Zephyr | add the project to `west.yml`, `CONFIG_NROS=y` |
| NuttX | symlink `apps/external/nano-ros`, `make menuconfig` |
| ESP-IDF | `idf.py add-dependency nano-ros` |
| upstream FreeRTOS sources | board package with `link_targets = ["freertos_kernel"]` |
| Pico SDK | same, naming the SDK's targets |

### 4.5 nano-ros's own example is not special

The mps2-an385 fixture uses the same path as §4.4's upstream row. CI provisions
upstream FreeRTOS into `third-party/` exactly as it provisions the Zephyr and
NuttX SDKs, and then **upstream's own `CMakeLists.txt` builds it**:

```toml
[board.integration]
rtos     = "freertos"
netstack = "lwip"
cmake_add_subdirectory = "{env:FREERTOS_DIR}"
cmake_cache  = { FREERTOS_PORT = "GCC_ARM_CM3", FREERTOS_HEAP = "4" }
link_targets = ["freertos_kernel"]
config_target = "freertos_config"    # we define it; it carries FreeRTOSConfig.h
```

The shipped FreeRTOS is an example, in the same sense that the nano-ros tree is
merely search root 0 in phase-348. Nothing in the product depends on it.

### A.3 — superseded UX summary (draft 2)


| profile | what they write |
| --- | --- |
| Zephyr / NuttX / ESP-IDF | one manifest line — unchanged from today |
| upstream FreeRTOS sources | board package naming `freertos_kernel` |
| vendored FreeRTOS, CMake host | board package with include dirs — one file |
| vendored FreeRTOS, IDE host | same file + `nros emit`, then three IDE steps |

The risk is concentrated in the last two rows: the include set is not guessable.
The mitigation is a scaffolder, not documentation — `nros init --from-cube
<project>` reading `.cproject` linked-resource XML, and the NXP equivalent
reading `prj.conf`. Both are mechanical; the information is already in those
files.

