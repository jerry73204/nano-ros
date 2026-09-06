---
rfc: 0064
title: "Board support organization: nano-ros as an embeddable library, not a board framework"
status: Draft (LIVE — under active exploration)
since: 2026-07
last-reviewed: 2026-09-06
implements-tracked-by: [phase-337, phase-375, phase-215]  # R3's matrix; 375 = tier policy + onboarding cost + R5's process; 215 = the FVP migration
supersedes: []
superseded-by: null
---

# RFC-0064 — Board support organization

> **Live document.** Written while bringing the Autoware safety island up on an
> NXP MR-CANHUBK3 (S32K344 + FreeRTOS) — a board that is out-of-tree by
> necessity. **[OPEN]** marks unresolved points. The **Exploration log** records
> what was checked and what it showed; body claims should trace to it.
> (Renumbered 0062 → 0064 on 2026-08-01 after an id collision with
> unified-dependency-ssot; 0064 is now the settled id.)
>
> **Revision 5 (2026-09-06)** makes board ARRIVAL uniform. A tree-wide audit
> found four different processes for adding a board and three mechanisms that
> were correct and unreachable, so R5 states one process (a board is a package,
> discovered by `provider_scan`), folds the FVP board's private `board.cmake`
> format into it, and audits every descriptor field down to primitives. R5
> changes no R2/R3/R4 principle.
>
> **Revision 3 (2026-08-04)** adds "The supported matrix" — the witness set, the
> `net_stack` cost axis it keys on, the `native`→`linux` naming decision, and the
> `matrix::CELLS` consequences. R3 changes no R2 principle; it applies them to
> pick a shippable set. It also closes three R2 [OPEN]s with measurement: the net
> ABI is per-platform not per-board (ThreadX runs one `net.c` over two backings),
> `net_stack` has no build-responsibility consumer, and the three "selected by no
> caller" board modules are confirmed to contribute zero test cells.
>
> **Revision 2 (2026-08-01)** replaced revision 1's central proposal. R1 modelled
> a board by *enumerating support dimensions* in the descriptor (image ownership,
> net stack, provenance). That was wrong: the dimensions depend on the shape of
> the user's RTOS, so any enumeration is a list nano-ros must keep extending
> forever. R2's spine is the opposite — nano-ros declares what it *needs*, the
> integrator satisfies it however their ecosystem already does. R1's measurements
> and defect findings are unchanged and retained below.

## Summary

**nano-ros should be an embeddable library plus the build system to compile a
ROS-like project against it. Board support is not a nano-ros concept.**

The host ecosystem — Zephyr, ESP-IDF, NuttX, a vendor SDK, PlatformIO — already
owns boards, drivers, pin mux, linker scripts and DTS. nano-ros competing with
that produces one crate per `(vendor × board × SDK-variant)`, which does not
scale and which RFC-0012 already rejected in 2026-05.

What nano-ros owns is narrow and stable:

1. a **portability seam** — the `nros_platform_*` C ABI,
2. a **capability declaration** — which link features it wants, not which stacks exist,
3. a **build system** — codegen, entry generation, `NanoRos::NanoRos`,
4. a **≤200-line integration shell** per host ecosystem.

Everything else is the integrator's, and nano-ros's job is to stay out of the
way and be discoverable from outside its own tree.

## The stance, and the evidence it already works

`integrations/nano-ros/CMakeLists.txt` is the whole ESP-IDF story — **146 lines**,
under Phase 139's documented ≤200 LoC hard cap per shell. It does four things:

```cmake
set(NANO_ROS_PLATFORM "esp_idf" CACHE STRING "" FORCE)   # 1. name the platform
# ... map CONFIG_NROS_* (Kconfig) → nano-ros cache vars   # 2. adopt host config
add_subdirectory("${_nros_root}" nano_ros_root)           # 3. pull in the build system
target_link_libraries(${COMPONENT_LIB} INTERFACE NanoRos::NanoRos)  # 4. re-export
```

Plus, in `cmake/platform/nano-ros-esp_idf.cmake`, the only board-ish work there
ever is:

```cmake
add_library(freertos_kernel INTERFACE)
target_link_libraries(freertos_kernel INTERFACE idf::freertos)
add_library(lwip INTERFACE)
target_link_libraries(lwip INTERFACE idf::lwip)
```

That is **every ESP32 part supported**, with no board crate, no board module, no
`NANO_ROS_BOARD`, and no descriptor. The module comment says so outright: *"No
NANO_ROS_BOARD requirement — IDF supplies every artefact the FreeRTOS board
overlays would have shipped."*

The same shape already exists for NuttX (`integrations/nuttx/` — CMake entry plus
Kconfig/Make.defs siblings for the make-driven configs), PX4, PlatformIO, and
Zephyr (nano-ros ships `zephyr/module.yml` declaring `build.cmake`,
`build.kconfig`, `snippet_root`, and Twister `samples`). Four ecosystems, all
integrated, none of them enumerated dimension by dimension.

**This is the model. It is already the Stable design of record** — RFC-0012,
"Board / BSP Integration Architecture", drafted 2026-05 *"in response to 'vendor
BSPs vary; we can't ship a board crate per (vendor × board × SDK-variant)
combo.'"* Its Layer 4 is labelled **"Vendor BSP — owned BY the vendor, NOT by
nano-ros."** RFC-0064 does not invent this; it finishes it and removes the parts
of the tree that still contradict it.

## What nano-ros needs (the seam)

### The ABI: 92 C functions

`packages/platform/nros-platform-api/include/nros/` declares the entire
portability contract as `nros_platform_*`. Grouped:

| Group | Count (approx) | Examples |
|---|---|---|
| clock / time | 7 | `clock_us`, `clock_ms`, `time_since_epoch_nanos` |
| sleep / yield | 4 | `sleep_ms`, `yield_now` |
| tasks | 6 | `task_init`, `task_join`, `task_detach` |
| sync | 20 | `mutex_*`, `mutex_rec_*`, `condvar_*`, `critical_section_*` |
| wake | 7 | `wake_init`, `wake_signal_from_isr`, `wake_wait_ms` |
| alloc / heap | 5 | `alloc`, `free`, `heap_used_bytes` |
| random | 5 | `random_u32`, `random_fill` |
| log | 3 | `log_write`, `register_log_writer` |
| **sockets / net** | **~33** | `tcp_*`, `udp_*`, `udp_mcast_*`, `socket_*`, `network_poll` |
| misc | 2 | `atomic_load_bool`, `zephyr_wait_network` |

Satisfy these and nano-ros runs. That is the honest definition of "embeddable
library", and it is a real boundary, not a slogan — the ESP-IDF, NuttX and Zephyr
shells all cross it without either side knowing much about the other.

**[OPEN]** How much of the 92 is *mandatory*? A serial-only, single-threaded,
no-heap deployment plainly does not need 33 socket functions. If the mandatory
core is ~25 and the rest is capability-gated, "port nano-ros to my RTOS" becomes
a weekend, and that number belongs in the porting guide's first paragraph. Worth
measuring: build a minimal platform impl and see what the linker actually demands.

### The capabilities: link features, not stacks

Revision 1 proposed enumerating net stacks (`lwip | smoltcp | netx | rtos |
host`). **That was the wrong shape** — it names implementations, so the list
grows with every ecosystem nano-ros meets, and a user whose RTOS has a network
stack nobody has heard of is simply unrepresentable.

nano-ros already has the right shape and has had it since Phase 134.
`nros-board-common/src/policy.rs`:

```rust
pub struct LinkFeatures {
    pub tcp: bool, pub udp_unicast: bool, pub udp_multicast: bool,
    pub serial: bool, pub raweth: bool, pub tls: bool,
    pub ivc: bool,          // Phase 100.4 — NVIDIA Tegra IVC
    pub custom: bool,       // Phase 115.B — runtime-pluggable user transport
}
```

paired with `LinkPolicy`, a per-platform mask using `Force(false)` / `Follow` —
*"Orin SPE has no Ethernet → `Force(false)` masks TCP / UDP / MC / SERIAL / TLS."*

This is capability negotiation: nano-ros says *which transports it wants*, the
platform says *which it can provide*, and the intersection decides what gets
compiled and which socket functions must exist. nano-ros never learns whether
the bytes leave through lwIP, NetX Duo, smoltcp, a Zephyr driver, or a vendor
DMA ring. **`custom` is the escape hatch that makes the set closed rather than
open-ended** — an RTOS with an exotic link implements one transport vtable and
stops there.

So: **delete `net_stack` from the descriptor rather than extend it.** It is
redundant with `LinkFeatures` and strictly less general.

**[DONE — phase-351 W6, 2026-08-15]** Checked, and nothing read it: `net_stack`
was parsed and never consulted from the day it was added, for either meaning.
It is deleted, along with the `NetStack` type. The question it half-answered
("who owns NIC + IP bring-up") is a property of the integration shell and is
carried by RFC-0072's A/B split; the question consumers actually ask — *which*
stack this board can be built with — is `supported_netstacks` (phase-351 W4),
which IS read: by `BoardDescriptor::resolve_netstack`, by `nros ws board-facts`,
and by `check-site-config`.

## Where board support survives, and why

Three consumption surfaces, decided by one question: **does the host ecosystem
have a build system?**

| Surface | When | Cost | Examples |
|---|---|---|---|
| **Integration shell** | ecosystem owns boards + build | ≤200 LoC once **per ecosystem**, then zero per board | Zephyr, ESP-IDF, NuttX, PX4, PlatformIO, vendor SDKs |
| **Board module** | no ecosystem build system — nano-ros must produce the image | per **board** | bare FreeRTOS, bare ThreadX, bare metal |
| **Nothing** | hosted | zero | POSIX, native |

Only the middle row is genuine "board support", and it exists only because bare
FreeRTOS and bare ThreadX ship a kernel and nothing else — no board abstraction,
no driver model, no linker script, no build. Someone must supply those, and if
the vendor has not, it falls to whoever is porting.

Two consequences worth stating plainly:

- **The middle row should shrink, not be systematised.** A vendor SDK build
  (S32DS, MCUXpresso, STM32CubeIDE) is an ecosystem with a build system, so it
  belongs in row 1 — a shell, not a board module. The MR-CANHUBK3 FreeRTOS
  deliverable is an S32DS project with `.cproject`, `Debug_FLASH/makefile`, RTD
  drivers, startup and two linker scripts. **The right integration may be
  `integrations/s32ds/` rather than `cmake/board/nano-ros-board-canhubk344-freertos.cmake`.**
  That inverts the plan RFC-0064 R1 proposed and is the most important open
  question in this document. **[OPEN — test it during the CANHUBK3 bring-up.]**
- **For the boards that genuinely stay in row 2, keep the module dumb.** It
  supplies linker script, startup, kernel config, netif, and nothing else. It
  should not carry arch policy, language standard, or config that belongs to the
  platform or the app.

## Worked example — MR-CANHUBK3 via `integrations/s32ds/`

The NXP S32K344 board is the first target chosen deliberately as a *row 1* case:
its FreeRTOS deliverable is an S32DS project (`.cproject`, `Debug_FLASH/makefile`,
RTD drivers, startup, two linker scripts), i.e. an ecosystem that owns its build.
It therefore gets a shell, not a board module.

### Target workflow

**A — host, once.** Note there is deliberately no `nros setup --board <name>`
(see §"`[board.*]` is a package-set alias" below):

```sh
git clone <nano-ros> && just setup-cli && source activate.sh
nros setup --tool arm-none-eabi-gcc --rmw zenoh
nros setup --source lwip                       # only if not using a vendor stack
export NXP_S32DS_PROJECT=~/MR_CANHUBK3_IEEE1722
```

**B — the ROS-shaped project, unchanged.** `package.xml` + `CMakeLists.txt` +
verbatim `.cpp`, plus a bringup pkg with `system.toml`:

```sh
nros codegen-system --workspace my_ws --bringup safety_island_bringup \
                    --target freertos --out build/nros-system
```

Deploy token is **`freertos`**, not a new one — the entry codegen's token set is
closed (`freertos nuttx posix threadx zephyr`) and ESP-IDF already reuses
`freertos`.

**C — build the nano-ros artefacts for the board.**

```sh
cargo build -p nros-cpp --target thumbv7em-none-eabihf --release
cmake -S <nano-ros>/packages/platform/nros-platform-freertos -B build/plat \
      -DCMAKE_TOOLCHAIN_FILE=<nano-ros>/cmake/toolchain/arm-freertos-armcm7.cmake \
      -DFREERTOS_DIR=$NXP_S32DS_PROJECT/FreeRTOS/Source \
      -DFREERTOS_PORT=GCC/ARM_CM7/r0p1 \
      -DFREERTOS_CONFIG_DIR=<your config dir>
```

`nros-platform-freertos` is already a standalone CMake project (C only,
parameterised by `NROS_PLATFORM_CFFI_INCLUDE` + FreeRTOS headers), so it builds
against NXP's kernel fork unmodified.

**D — S32DS links them.**

```sh
cp <nano-ros>/integrations/s32ds/makefile.defs $NXP_S32DS_PROJECT/
make -C $NXP_S32DS_PROJECT/Debug_FLASH all
```

### The injection point already exists

CDT's generated `Debug_FLASH/makefile` contains, in order:

```make
-include ../makefile.init
-include objects.mk                #  USER_OBJS :=   LIBS := -lc -lm -lgcc
-include ../makefile.defs          #  ← included BEFORE the rules
...elf: $(OBJS) <ld> $(USER_OBJS)
	arm-none-eabi-gcc -o "...elf" "@....args"  $(USER_OBJS)
-include ../makefile.targets
```

None of `makefile.init` / `.defs` / `.targets` exist in the shipped project —
they are CDT's sanctioned extension points. So the shell drops a `makefile.defs`
appending nano-ros's `.a` files to `USER_OBJS` and `-I` flags to the compile
args. Structurally identical to `integrations/nuttx/Make.defs` appending to
`EXTRA_LIBS` / `EXTRA_LIBPATHS`.

**ABI flags must match exactly.** From the project's own link `.args`:

```
-mcpu=cortex-m7  -mthumb  -mfloat-abi=hard  -mfpu=fpv5-sp-d16
-nostartfiles  -specs=rdimon.specs
```

`fpv5-**sp**-d16` — single precision. And `rdimon.specs` (semihosting), not the
`nosys.specs` the mps2 board module uses.

**[OPEN]** `Debug_FLASH/`'s `.elf` rule carries a hardcoded prerequisite
`C:/Users/nxf56445/workspaceS32DS.3.4/.../linker_flash_s32k344.ld`. On Linux
make will fail "no rule to make target" unless the project is regenerated from
S32DS locally. If regeneration is required per machine, the shell must say so —
possibly shipping a `makefile.init` that patches it.

### `[board.*]` is a package-set alias, not board support

`nros setup <board>` resolves `[board.<name>].packages` and errors *"Add a
`[board.{board}]` entry to nros-sdk-index.toml"*. But look at what an entry
holds:

```toml
[board.qemu-arm-freertos]
arch = "cortex-m3"  ;  platform = "freertos"
packages = ["arm-none-eabi-gcc", "qemu", "freertos-kernel", "lwip"]
```

Nothing there is specific to MPS2-AN385. It is "Cortex-M + QEMU + FreeRTOS" — an
**(arch × ecosystem) profile** wearing a board's name. That is why the table
would otherwise grow once per board, defeating the whole point of the shell.

Three ways not to add an entry, in increasing cost:

1. **Don't use `--board`.** `--tool <name>` and `--source <name>` already reach
   index packages directly. Zero nano-ros change; this is what the workflow above
   does. The board name was only ever an alias for a list.
2. **User-owned alias.** `--index <path>` already exists, so a project can ship
   its own index. **Caveat: there is no index composition** — no `extends`,
   `include` or merge in the parser — so a private index must redefine every
   `[tool.*]` it references. Making `--index` repeatable (or adding `extends`) is
   small and makes this the right answer downstream.
3. **RFC-0013 / phase-201** — boards self-describe deps, read from a path.
   Precedent exists in a sibling verb: `nros setup board <name>
   --zephyr-workspace` already reads the *board package's* provisioning contract
   rather than the index.
   **Both superseded 2026-09-04** (phase-201 archived unstarted, RFC-0013 marked
   Superseded): RFC-0087 makes a board an ordinary `package.xml` package whose
   deps are its `<depend>` entries, so there is no index lookup left to extend
   and no board-specific descriptor to read. The `--zephyr-workspace` precedent
   still stands as evidence the pull was real — it is that shape, board-scoped.

**Proposed fix:** re-key the table as `[profile.<arch>-<ecosystem>]` and make
`[board.*]` an optional alias pointing at a profile. One `cortex-m-freertos`
profile then covers every Cortex-M FreeRTOS board that will ever exist, and
`--rmw` stays the orthogonal axis it already is. Keep existing board names as
aliases so nothing breaks.

### Revisions this workflow requires

| # | Change | Where | Status |
|---|---|---|---|
| 1 | FreeRTOS platform runs board-less when a shell composed `freertos_platform` | `cmake/platform/nano-ros-freertos.cmake` | **DONE** |
| 2 | Cortex-M7 arch profile; `arch` scalar → list | `config/freertos-lwip/nros-platform.toml` | **DONE** |
| 3 | `integrations/s32ds/` — probe, CMake shell, `makefile.defs`, README | new dir | **PARTIAL** — 226 code lines (**over** the 200 cap, see below); configures, builds the shim, emits the link fragment; full link unexercised |
| 4 | CM7 toolchain file; split arch from `-fno-exceptions`/`-std=c++14` policy | `cmake/toolchain/arm-freertos-armcm7.cmake` | todo |
| 5 | Document the mandatory subset of the 92-function platform ABI | `nros-platform-api` + porting guide | todo |
| 6 | Porting guide: "integrating nano-ros into a vendor SDK build" | `book/src/porting/` | todo |
| 7 | `FREERTOS_PORT` help text: add the CM7 form | `config/freertos-lwip/…` | **DONE** (with 2) |
| 8 | `NANO_ROS_BOARD_PATH` search roots | 4 files | todo, demoted — this board never needs it |
| 9 | `[board.*]` → `[profile.*]` keyed on (arch × ecosystem); index composition | index + `setup.rs` | todo |
| 10 | Make `--tool` repeatable (`Option<String>` → `Vec<String>`), matching `--source` | `setup.rs` | todo |

Nothing else: no new deploy token, no `PlatformId`, no board crate, no
descriptor, no `cmake/board/` module, no `board-support.toml` row, no tier entry,
no SDK-index entry. **This board adds zero per-board files to nano-ros.**

## Discovery: Yocto layers vs the Zephyr model

Both were raised as candidates. They answer different questions.

| | Yocto | Zephyr |
|---|---|---|
| Unit | layer (`BBLAYERS`, priorities) | module (`zephyr/module.yml`) + search roots |
| Out-of-tree hardware | layer contributes recipes | `BOARD_ROOT` / `SOC_ROOT` / `DTS_ROOT` / `ZEPHYR_EXTRA_MODULES` |
| Override | `.bbappend` can patch any recipe | no override; compose via Kconfig + DTS overlays |
| Hardware description | code (recipes, machine conf) | **data** (devicetree), drivers bind on `compatible` |
| Failure mode | layer-priority and append-order bugs | DTS/Kconfig verbosity |

**Recommendation: the Zephyr model, minus devicetree.**

- **Take the search roots.** `BOARD_ROOT` is exactly the `NANO_ROS_BOARD_PATH`
  proposal, proven at Zephyr's scale where most boards now live out of tree. It
  is additive, order-independent, and has no priority semantics to get wrong.
- **Take the module manifest.** nano-ros already *is* a Zephyr module; it should
  also *accept* modules — a directory declaring what it contributes (boards,
  platform impls, drivers, toolchain files) discovered via a root variable.
- **Take the binding-by-capability idea, not devicetree itself.** Devicetree
  earns its cost when one framework owns hundreds of drivers. nano-ros owns
  none — it consumes what the host provides. `LinkFeatures` is already the
  right-sized version of the same idea.
- **Reject Yocto's override model.** `.bbappend`-style patching of nano-ros's
  own recipes would let an integrator silently reshape the build, and layer
  priority is the part of Yocto that people fight most. nano-ros does not need
  it: an integration shell composes, it does not override.

The one Yocto idea worth keeping is *provenance separation* — Yocto's
`meta-<vendor>` layers exist precisely because vendor BSPs cannot live upstream.
That is the same reason `NXP_RTD_DIR` must be an env pointer and not a vendored
tree.

## The customization ladder

A declarative model that cannot be escaped is worse than none — real boards
always have one thing nobody anticipated, and if the only answer is "patch
nano-ros", the model failed exactly when it mattered. Three rungs, and **rung 3
must always exist**:

| Rung | Cost | Mechanism |
|---|---|---|
| 1. Declare | one line | a capability flag or a path pointer |
| 2. Hook | a function | a `#[no_mangle]` / weak C symbol the glue calls |
| 3. Escape | own the file | hand-written artifact; the generator defers to it |

Worked for the customization asked about — Ethernet:

| Situation | Rung | What the user does |
|---|---|---|
| Host ecosystem has a driver (Zephyr, IDF, NuttX) | — | nothing; enable it in the host's own config |
| Stack exists, board needs wiring | 2 | implement `nros_board_init_eth()` |
| Exotic or proprietary link | 1 + 2 | `link.custom`, implement the transport vtable (`zpico-platform-custom`, Phase 115.B) |
| None of the above fits | 3 | implement the ~33 net functions of the platform ABI directly |

Note that no rung requires nano-ros to know what the stack *is*. That is the
test the R1 design failed and this one passes.

Rung 3 also carries the rule that keeps any future generator honest: **if a board
ships an artifact by hand, the generator does not overwrite it and the drift gate
does not flag it.** That makes generated glue adoptable file by file rather than
as a flag day.

## The supported matrix (revision 3, 2026-08-04)

The ladder above says how a user customizes. This says **what we ship as bases
and what we promise about each** — the set chosen so that most users start from
a witness that already resembles their hardware, and so that the maintained
surface stays small enough to actually run per task.

### The axis that decides cost: who owns NIC + IP bring-up

Every board falls into one of two classes, and the classes differ by an order of
magnitude in per-board cost.

> **The descriptor no longer declares this** (phase-351 W6): `net_stack` was
> deleted after four waves confirmed nothing read it. The class below is still
> the right cost axis — it is now read off `supported_netstacks` instead, which
> is the same partition stated as a fact consumers can act on: a board that
> declares stacks supplies them (**nanoros-owned**); an empty list means the
> host ecosystem does (**rtos-owned**).

| Class | Who supplies boot/driver/stack | Per-board cost | Members |
|---|---|---|---|
| **rtos-owned** | the host ecosystem (Zephyr, NuttX, ESP-IDF, a vendor SDK) | a config bundle — **0–160 lines, nearly all data** | zephyr, nuttx-*, esp32-qemu, s32ds shell |
| **nanoros-owned** | nano-ros supplies stack *and* MAC driver | **~76 lines of board delta + ~129 mechanical + a driver crate** (measured, phase-337 W5.f) | mps2-an385{,-freertos}, threadx-*, stm32f4, orin-spe |

> **The two numbers, and why both matter** (measured phase-337 W5.f, 2026-08-04;
> the earlier "~60–80 lines" counted only the first of them). A second FreeRTOS
> board is **205 lines**, split:
>
> - **~76 lines a porter must reason about** — the vector table, the three
>   memory-map numbers, the CPU clock, the netif registration, the driver
>   reference. This is the number the original estimate named, and it held.
> - **~129 lines of mechanical scaffolding** — the board ZST and its four trait
>   impls, `build.rs`, `Cargo.toml`. Identical for every Cortex-M FreeRTOS
>   board, copied without thought.
> - **plus a MAC driver** (~507 lines for LAN9118) when the board needs one that
>   nano-ros does not already have. Not reducible by any template.
>
> Quoting only 205 overstates the difficulty and only 76 understates the typing,
> which is why both are stated. Collapsing the 129 behind a macro is possible
> but not free: it would put `panic-semihosting` into the family crate and so
> impose a panic strategy on every consumer.

Measured, not assumed. `nros-board-fvp-aemv8r-smp` — a real non-native Zephyr
board — is **160 lines** and self-describes as "a thin Cargo + config bundle";
Zephyr owns boot, MMU, net stack and the ethernet driver. Against that, the
FreeRTOS MPS2 crate is ~1600 lines plus a 727-line `startup.c`, of which the
irreducible board delta is only ~60–80 (vector table 19, memory map 3 numbers,
CPU clock 1, cflags 1, netif registration 4, driver reference). The remainder is
copy-paste that templating removes — see "Defects" below.

**The strategic consequence:** breadth is bought in the rtos-owned class, where
it is nearly free, and NOT by adding nanoros-owned board crates. An industrial
FreeRTOS user almost always has a vendor SDK (ESP-IDF, S32DS, MCUXpresso,
STM32Cube) and therefore belongs in the rtos-owned class via an integration
shell — which is exactly what `integrations/s32ds/` already demonstrates for the
MR-CANHUBK3, contributing **zero** files to this tree.

### Three layers, not two — where "one source, many arches" actually cuts

The instinct to merge sibling board crates is right, but the cut is not
board-against-board. Every RTOS family decomposes into **three** layers, and
which of them nano-ros has to own is what really differs:

| Layer | What it is | Zephyr | NuttX | ThreadX | FreeRTOS |
|---|---|---|---|---|---|
| 1. Family driver | RTOS-generic entry/spawn/session logic | upstream | upstream | **in-tree, exists, already board-generic** | **in-tree, exists** |
| 2. **Arch port** | RTOS × ISA — context switch, trap, `tx_port.h` | upstream | upstream | **forked in-tree, TRAPPED inside a board crate** | upstream + cflags |
| 3. Board overlay | memory map, net bring-up, console, exit | the whole crate (160 lines) | the whole crate | entangled with layer 2 | ~76 lines + ~129 mechanical + a MAC driver |

Verified per family:

- **ThreadX — the generic lift already happened; the arch port is the problem.**
  `nros-board-threadx/src/entry.rs` (1120 lines) is *already* parameterized over
  the board — `run_entry::<MyBoard, Config, F, E>(cfg, setup)`, internals bounded
  `B: BoardPrint + BoardExit` — and **both** boards already delegate to it. So
  layer 1 is done. But `nros-board-threadx-qemu-riscv64` is 3598 lines of which
  **~1250 are layer 2**: `tx_thread_{schedule,context_save,context_restore}.S`
  (1002) + `config/tx_port.h` (252), a *modified copy* of
  `third-party/threadx/kernel/ports/risc-v64/`. The fork is legitimate — upstream's
  rv64 port types `ULONG` as 8 bytes, which breaks NetX Duo, whose packet code does
  `ULONG *` arithmetic assuming 4-byte words; retyping `ULONG` shifts every
  `TX_THREAD` field offset, which forces the assembly fork. `threadx-linux` has no
  such file, because upstream's Linux port already uses a 4-byte `ULONG`.

  **So merging the two ThreadX boards into one crate is the WRONG cut** — it would
  put 1250 lines of RISC-V assembly into a crate that also serves Linux, `cfg`-gated:
  one crate with two disjoint halves. Extract layer 2 instead. Both boards then
  become ~150–300-line overlays, and a future riscv32 or Cortex-R ThreadX board
  reuses the port rather than forking it a second time. `src/config.rs` is already
  84 % identical between them (55 diff lines of 339), so W1.g's shared `BaseConfig`
  absorbs most of the remainder.

  **LANDED 2026-08-04 (phase-337 W4).** Layer 2 is
  `packages/boards/nros-board-threadx-port-riscv64` — 1690 lines of forked port
  (the header plus FIVE `.S` files, not three: `tx_thread_stack_build.S` and
  `tx_thread_system_return.S` carry the same `ULONG` retype, and the earlier
  count of "~1250" had missed them). `tx_initialize_low_level.S` stayed with the
  board: it is the qemu_virt BSP's low-level init, not the arch port's. Both
  boards adopted `BaseConfig`, which deleted the two hand-copied
  `DeployOverlay`→`Config` merges and the two hand-copied `nros.toml` parsers.
  Consumers reach the port through one precedence rule (its `inc/` before
  upstream's), enforced by a `_Static_assert(sizeof(ULONG) == 4)` in
  `nros-board-common/c/threadx_hooks.c` — because the failure mode of losing it
  is a clean compile and corrupted packets, not a diagnostic. Behaviour-neutral:
  42 + 23 fixture rows and 18 + 12 cells unchanged.

  This revises phase-322 W1.f. Its "KEEP SEPARATE" verdict is right about the crate
  boundary, but the reasons it cites — distinct `#[panic_handler]`, hosted-vs-bare
  link model, different net drivers — are all things the `BoardInit`/`BoardPrint`/
  `BoardExit`/`BoardEntry` trait set *already* abstracts. The un-abstracted thing is
  the arch port, which W1.f does not mention.

- **NuttX — merge, exactly as asked.** Layers 1 and 2 both come from upstream
  (kernel + arch ports; the link list is *discovered* by scanning
  `$NUTTX_DIR/staging` for `lib*.a` rather than hardcoded per board), so the
  in-tree crates are pure layer 3. Their `build.rs` bodies are literally the same
  four calls, 1059 lines are byte-identical duplicates, and the crate to hold them
  already exists with both boards depending on it. Per-arch difference is data: a
  defconfig, a toolchain cmake file, nine `NUTTX_*` env values, a
  `.cargo/config.toml`. This is phase-322 W1.a, unchanged.

- **Zephyr — already there; nothing to merge.** All three layers come from Zephyr.
  The board crate *is* layer 3 and the existing non-native one is 160 lines of
  mostly config data. Adding an architecture is adding a `boards/<board>.conf`, not
  a crate. **Direction:** Zephyr boards should be conf bundles under one
  `nros-board-zephyr`, with `nros-board-fvp-aemv8r-smp` folded in as the first
  migration once W1.g's shared `Config` lands — a per-Zephyr-board *crate* is the
  duplication this RFC exists to prevent.

  **DONE (phase-337 W9.a, 2026-08-05).** `nros-board-fvp-aemv8r-smp` is now
  `nros-board-zephyr/boards/fvp-aemv8r-smp/`. The prediction held better than
  written: the crate's Rust half was not merely thin, it had **zero consumers** —
  `nros_board_fvp_aemv8r_smp` appeared in one file in the whole tree, its own
  `Cargo.toml`. `nano_ros_use_board(<name>)` is unchanged at every call site
  because the LOOKUP widened (crate first, then conf bundle under a family crate)
  rather than the callers moving. And the same crate now serves three witnesses at
  three tiers — native_sim (1), the `mps2_an385` Cortex-M witness (2), the FVP
  (3) — which is exactly the `(crate, matrix_platform)` registry key the paragraph
  below predicted would be needed.

**Consequence for the tier registry.** `board-support.toml` keys tier by CRATE and
its completeness gate asserts every board directory appears exactly once. A merged
`nuttx-qemu` crate serves *two* arches at *two different tiers* (arm tier 1, riscv
tier 2), and a conf-bundle `nros-board-zephyr` serves three. So the registry's row
key must become the **witness** — `(crate, matrix_platform)` — or `matrix_platform`
must become a list. Merging without this change breaks the gate's one-row-per-board
invariant.

### The witnesses

Chosen by *(integration shape × arch)*, not by board. A shape with no witness is
a shape we cannot claim; a second board of a shape we already witness earns its
keep only if it adds an arch.

| Tier | Witness | Shape | Arch |
|---|---|---|---|
| 1 | linux (host process) | hosted, reference — all three RMWs | x86_64 |
| 1 | Zephyr native_sim | rtos-owned, NSOS host sockets | x86_64 |
| 1 | NuttX qemu-arm | rtos-owned, cross | ARMv7-A |
| 1 | FreeRTOS mps2-an385 | nanoros-owned (the no-SDK reference) | ARMv7-M |
| 1 | ThreadX-linux | nanoros-owned via NSOS shim | x86_64 |
| 2 | **Zephyr on QEMU Cortex-M — NEW** | rtos-owned, *real* in-kernel net stack | ARMv7-M, 32-bit |
| 2 | NuttX qemu-riscv | rtos-owned, cross | riscv32 |
| 2 | ThreadX qemu-riscv64 | nanoros-owned, real NetX Duo | riscv64 |
| 2 | esp32-qemu | vendor SDK owns image | Xtensa |
| 2 | bare-metal mps2 (+ one RTIC witness) | no-RTOS floor; executor shape | ARMv7-M |
| 3 | fvp-aemv8r-smp | rtos-owned, license-gated model | AArch64 / Cortex-R |

Leaving the matrix:

- **`stm32f4` + `rtic-stm32f4`** — tier 3, hardware-only, **zero Runtime cells**;
  a lane that can never gate anything. Cortex-M is already witnessed by MPS2 in
  QEMU. These serve users far better as the book's worked customization example
  than as in-tree crates nobody can boot in CI.
- **The four scaffolds** (`embassy-stm32f4`, `esp32s3`, `s32z270dc2-r52`,
  `orin-spe`) — they contribute **zero cells to the test matrix**, so deleting
  them costs no coverage at all. They are precisely the boards this RFC says
  should arrive through integration shells. **Caveat on `orin-spe`:** despite
  being nominally a scaffold it is load-bearing in link-feature selection
  (`CARGO_FEATURE_ORIN_SPE`, `LinkPolicy::orin_spe()`, `zpico_backend="orin-spe"`
  — `nros-zpico-build/src/runner.rs:225,419-420,528-529`). It is modelled as a
  pseudo-platform, so it needs untangling, not a blind delete.

### Why the new Zephyr Cortex-M witness is the one real addition

It closes a coverage hole, not a marketing gap. **All 28 Zephyr runtime configs
target `native_sim/native/64`**, and `cmake/zephyr/native-sim-line-*.conf` sets
`CONFIG_NET_SOCKETS_OFFLOAD=y` with `CONFIG_ETH_NATIVE_TAP=n` — chosen so tests
need no TAP device or root. So **every Zephyr test nano-ros runs bypasses
Zephyr's own IP stack**, using host sockets, at 64-bit pointer width. There is
zero coverage of Zephyr's in-kernel net stack, of a real driver, or of 32-bit
Zephyr. `nros-platform-zephyr/src/net_wait.c:53` carries an
`#ifdef CONFIG_BOARD_NATIVE_SIM` whose *other* branch has never run in CI.

The cost is unusually low: the harness already boots
`qemu-system-arm -machine mps2-an385 -nic user,model=lan9118` for the FreeRTOS
and bare-metal lanes (`nros-tests/src/qemu.rs:242`), and Zephyr is the cheapest
family in the tier-2 build (~200 s against FreeRTOS's ~1370 s).

**SETTLED (phase-337 W2.a, 2026-08-04): `mps2/an385` via `smsc911x`.** Measured
against the Zephyr 3.7 checkout `just setup zephyr` provisions, which is the
checkout this RFC said the question needed:

- `boards/arm/mps2/mps2_an385.dts` already carries an ENABLED
  `eth0: eth@40200000 { compatible = "smsc,lan9220"; }` — no overlay required.
- `drivers/ethernet/Kconfig.smsc911x` makes `ETH_SMSC911X` `default y` on that
  node, and sets `ETH_NIC_MODEL = "lan9118"` — the driver names the QEMU model
  it expects.
- `boards/arm/mps2/board.cmake` runs `qemu-system-arm -cpu cortex-m3 -machine
  mps2-an385`, the same invocation `nros-tests/src/qemu.rs:242` already makes
  for the FreeRTOS and bare-metal lanes.

The two candidates were not equal: this one needs no overlay, no SLIP pty and no
new runner, while SLIP on `qemu_cortex_m3` would have added a second runner
shape for less realism.

### `native` vs `posix` vs `linux` — the name follows the promise

Today there are **two crates and two names for one implementation**:
`board_path_for` maps *both* `"native"` and `"posix"` to
`::nros_board_linux::LinuxBoard` (`nros-orchestration-ir/src/lib.rs:78`), so
`nros-board-linux` (549 lines) is never named by any generated entry — the
observation phase-322 W1.e already made.

What is actually supported, measured:

| Claim | Evidence |
|---|---|
| Windows | none — no `_WIN32` anywhere in the tree |
| macOS | *contemplated but unbuilt*: `nros-platform-posix/src/platform.c` carries five `__APPLE__` branches (a `pthread_cond` fallback for absent unnamed `sem_t`), **but** `src/timer.c:72` calls `timer_create(CLOCK_MONOTONIC, SIGEV_THREAD)` with no fallback, and macOS does not implement POSIX timers |
| *BSD | plausible — the only other OS gate is `#ifdef __linux__` for `MSG_NOSIGNAL` (`net.c:216`) with a portable `else`; FreeBSD/NetBSD have `timer_create`, OpenBSD does not |
| Linux | the only thing tested — **all 19 CI jobs are `ubuntu-*`** |

So "native" is the least accurate of the three: it implies any general-purpose
OS, and Windows/macOS do not build. "posix" describes the *source* honestly but
promises BSD and macOS that nothing tests and `timer_create` partly breaks.

**Decision: separate the two layers, which is what the rest of the tree already
does.**

- **Platform stays `posix`** — `nros-platform-posix` is a genuine portability
  seam written to POSIX, and the platform layer names software-stack facts
  (RFC-0049's duty rule).
- **The board/target becomes `linux`** — the board layer names the concrete
  thing we claim to support, and a tier-1 promise means "`just ci` exercises
  it", which only Linux does.
- **`native` is retired** as a public spelling; `nros-board-linux` and
  `nros-board-linux` merge (phase-322 W1.e) into the single `linux` board.

This buys the same thing the rest of the matrix buys: `macos` and `freebsd` can
later join as tier-3 (build-only) or tier-2 boards on the *unchanged* `posix`
platform, which is the one-source-many-targets model applied to hosted OSes.
Prerequisite for either: a `timer_create` fallback (dispatch-source or
`pthread_cond` timed wait).

### What this does to the test matrix

`matrix::CELLS` today is **202 cells — 174 Runtime, 17 BuildOnly, 11 CarveOut**:

| Platform | Runtime | BuildOnly | CarveOut | total |
|---|---|---|---|---|
| Native | 72 | 2 | 0 | 74 |
| ZephyrNativeSim | 39 | 0 | 0 | 39 |
| ThreadxLinux | 18 | 0 | 0 | 18 |
| FreertosMps2 | 15 | 1 | 1 | 17 |
| NuttxArm | 14 | 1 | 2 | 17 |
| ThreadxRiscv64 | 9 | 3 | 0 | 12 |
| NuttxRiscv | 4 | 3 | 2 | 9 |
| Esp32Qemu | 2 | 2 | 2 | 6 |
| QemuBaremetal | 1 | 0 | 2 | 3 |
| Stm32F4 | **0** | 3 | 0 | 3 |
| Fvp | **0** | 2 | 1 | 3 |
| Px4 | **0** | 0 | 1 | 1 |

The matrix barely moves, which is the point — the proposal removes promises and
files, not coverage:

1. **`PlatformId::Native` → `PlatformId::Linux`.** A rename across the enum, its
   `index()` band, and `fixture_tokens()` (`native` → `linux`). Because
   `fixture_tokens` is the one-way SSoT for that vocabulary and
   `fixture_token_mapping_round_trips` gates it, this is a mechanical rename with
   a gate that proves it landed everywhere. `examples/fixtures.toml`'s 187
   `platform = "native"` rows move with it.
2. **`Stm32F4` leaves** — 3 BuildOnly cells, **0 Runtime**, so no runtime
   coverage is lost. Its 8 `fixtures.toml` rows go with it.
3. **The 4 scaffolds** contribute **no cells at all** — invisible to the matrix,
   so their removal is provably free on this axis.
4. **`Fvp` and `Px4` are unchanged** — already 0 Runtime, already carrying their
   reasons, which is exactly what `Tier::BuildOnly`/`CarveOut` exist to record.
5. **One new `PlatformId::ZephyrQemuCortexM`** with a starter cell group
   (pubsub × {rust, c, cpp} × zenoh as Runtime; service/action BuildOnly until
   they run). Adding it extends the allocator's port/domain bands, and the
   injectivity gate re-proves collision-freedom automatically.

On the fixture side the new witness follows the **existing Zephyr exemption**
rather than growing `fixtures.toml`: `every_runtime_cell_has_a_fixture_row`
already exempts ZephyrNativeSim examples and non-Rust workspaces because they are
built by the west leaves lane (`scripts/build/zephyr-fixture-leaves.sh`) with its
own staleness signature. The Cortex-M witness is the same shape — a west build
with a `boards/<board>.conf` overlay — so it joins that exemption with its board
named, and the gate keeps holding in both directions.

Net effect on the numbers: 202 cells → ~202; 27 board crates → ~15–16 (phase-322
takes 27→19, the demotions above take out 6 more); ~60 files naming a single
board become the RFC-0064 target of near-zero for anything rtos-owned.

## Target state (the thing to build toward)

All three tables are the *same* decision viewed from three angles. Current
numbers measured 2026-08-04.

### Boards — the promise surface: 27 crates → 16

| # | Crate | Tier(s) | Serves | Change |
|---|---|---|---|---|
| 1 | `nros-board-linux` | 1 | x86_64 host | **merge** of `native` + `posix`; renamed (W1.e) |
| 2 | `nros-board-zephyr` | 1 / 2 / 3 | native_sim · **QEMU Cortex-M** · FVP | conf bundles, not crates; absorbs `fvp-aemv8r-smp` |
| 3 | `nros-board-mps2-an385-freertos` | 1 | ARMv7-M, nanoros-owned | **DONE** (phase-337 W5) — templated: overlay 2497 → 1065 lines (−57 %); a second board measures 205 lines, of which 76 are the board delta |
| 4 | `nros-board-nuttx-qemu` | 1 / 2 | ARMv7-A · riscv32 | **DONE** (phase-337 W3) — merge of `-qemu-arm` + `-qemu-riscv`: 3350 → 2054 lines (−1296, −39 %), two `[[board]]` witnesses in one descriptor, `Config` on `BaseConfig` |
| 5 | `nros-board-threadx-linux` | 1 | x86_64, NSOS shim | **DONE** (phase-337 W4.c) — thinned to overlay: 1265 → 1097 lines |
| 6 | `nros-board-threadx-qemu-riscv64` | 2 | riscv64, real NetX Duo | **DONE** (phase-337 W4.b) — thinned to overlay: 4023 → 2185 lines; the arch port moved to `nros-board-threadx-port-riscv64` |
| 7 | `nros-board-mps2-an385` | 2 | bare-metal floor | **DONE** (phase-337 W6.a) — absorbed `rtic-mps2-an385` as the `rtic` feature |
| 8 | `nros-board-esp32-qemu` | 2 | Xtensa, vendor SDK | unchanged |
| 9–16 | infra: `common`, `cffi`, `freertos`, `nuttx`, `threadx`, **`threadx-port-riscv64`** (NEW, layer 2 — **DONE 2026-08-04**, phase-337 W4.a), `mps2-an385-pac`, descriptors | — | — | `bare-metal` **DELETED 2026-08-04** (phase-337 W7.c — no board ever opted in) |

**Progress against this table: COMPLETE, measured 2026-08-05.** 27 → **17**
board directories (15 crates + the two descriptor-only dirs, which the target's
"16" counted as crates and which phase-321 W2 moves out of `packages/boards/`
altogether). Every row above is now DONE. Landed: the NuttX merge (W3), the
ThreadX arch-port extraction (W4), the FreeRTOS templating (W5), the RTIC fold
(W6), the STM32F4 family's departure and the four scaffolds (W7), the `native` +
`posix` → `nros-board-linux` merge with the descriptor rename and the
fixture-token move (W8), the FVP fold (W9.a), and row 2's Cortex-M witness
(W2) — which is the one that turned "Zephyr" from a single 64-bit host config
into a platform with a real 32-bit witness, at the cost of five defects that only
a non-native_sim Zephyr could surface.

**Row 2 is the RFC's thesis, demonstrated.** One `nros-board-zephyr` now serves
THREE witnesses at three tiers, and the two non-native_sim ones own no crate at
all: `mps2_an385` is `cmake/zephyr/mps2-an385.conf`, the FVP is
`nros-board-zephyr/boards/fvp-aemv8r-smp/`. Adding a Zephyr architecture really
did become adding a conf bundle.

**One caveat worth carrying forward.** The Cortex-M witness's cells are C and
C++ only. The pinned `zephyr-lang-rust` cannot compile the `zephyr` crate for ANY
board whose devicetree has gpio nodes (issue 0432) — essentially every real
board — so Rust-on-Zephyr remains native_sim-only until that is fixed upstream.
Nothing in this RFC's structure causes it, and nothing in this RFC's structure can
fix it.

**Deleted (12):** `native`, `posix` (→ `linux`), `nuttx-qemu-riscv` (→ merged),
`rtic-mps2-an385` (→ feature), `fvp-aemv8r-smp` (→ conf bundle), `stm32f4`,
`rtic-stm32f4` (→ book customization example), `embassy-stm32f4`, `esp32s3`,
`s32z270dc2-r52`, `orin-spe` (scaffolds → integration shells), `bare-metal`.
**Added (1):** the ThreadX riscv64 arch port — **landed 2026-08-04** as
`packages/boards/nros-board-threadx-port-riscv64` (phase-337 W4.a). It holds
the five forked context-switch `.S` files and the forked `tx_port.h`; both
ThreadX boards and every CMake path reach it through the ONE precedence rule
its `src/lib.rs` states, backed by a `_Static_assert(sizeof(ULONG) == 4)` in
`nros-board-common/c/threadx_hooks.c` so a lost override fails the build
rather than the network.

`orin-spe` cannot be deleted blind — it is load-bearing as a pseudo-platform in
link-feature selection (`runner.rs:225,419-420,528-529`). Untangle first.

### Fixtures — 344 rows → 336

| `platform =` token | now | target | note |
|---|---|---|---|
| `native` → **`linux`** | 188 | 188 | **DONE** (phase-337 W8.c) — rename only, no count change. 188, not 187: the RFC's figure was one low. Moved together with the two other token-derived vocabularies (the builder's platform argument, the lane-coordinate prefix); the LANE name, the `just` module and `examples/native/` deliberately keep `native` |
| `threadx-linux` | 42 | 42 | — |
| `nuttx` | 30 | 30 | one crate now builds it |
| `freertos` | 25 | 25 | — |
| `threadx-riscv64` | 23 | 23 | — |
| `qemu-arm-baremetal` | 20 | 20 | — |
| `stm32f4` | 8 | **0** | leaves with the board |
| `nuttx-riscv` | 4 | 4 | same crate as `nuttx`, separate row |
| `qemu-esp32-baremetal` | 3 | 3 | — |
| `zephyr` | 1 | 1 | + Cortex-M via the **west lane**, not this file |
| `esp32` | 1 | 1 | — |
| **total** | **344** | **336** | |

The Zephyr Cortex-M witness adds **zero** `fixtures.toml` rows: it joins the
existing west-lane exemption in `every_runtime_cell_has_a_fixture_row`, which
already covers ZephyrNativeSim examples and non-Rust workspaces because
`scripts/build/zephyr-fixture-leaves.sh` builds them under its own staleness
signature. The exemption gains the new board by name.

Note what does *not* shrink: merging crates removes no fixture rows, so it buys
**no CI time**. Build time is dominated by FreeRTOS (~1370 s) and native
(~1300 s) per lane; `stm32f4`'s 8 rows are 2 % of the manifest. Crate merging is
a maintenance-surface lever (duplicated lines, silent drift — the forks have
already rotted twice unnoticed); the wall-clock lever is the tier/lane split,
which phase-318 already delivered.

### Tests — 202 cells → 208 (Runtime 174 → 177)

| Platform | Runtime | BuildOnly | CarveOut | change |
|---|---|---|---|---|
| **Linux** (was Native) | 72 | 2 | 0 | rename of `PlatformId` + `fixture_tokens` |
| ZephyrNativeSim | 39 | 0 | 0 | — |
| **ZephyrQemuCortexM** | **3** | **6** | 0 | **NEW** — pubsub×{rust,c,cpp}×zenoh Runtime; service/action BuildOnly |
| ThreadxLinux | 18 | 0 | 0 | — |
| FreertosMps2 | 15 | 1 | 1 | — |
| NuttxArm | 14 | 1 | 2 | — |
| ThreadxRiscv64 | 9 | 3 | 0 | — |
| NuttxRiscv | 4 | 3 | 2 | — |
| Esp32Qemu | 2 | 2 | 2 | — |
| QemuBaremetal | 1 | 0 | 2 | — |
| ~~Stm32F4~~ | ~~0~~ | ~~3~~ | ~~0~~ | **removed** — 0 Runtime, so no coverage lost |
| Fvp | 0 | 2 | 1 | unchanged (already build-only) |
| Px4 | 0 | 0 | 1 | unchanged (already a carve-out) |
| **total** | **177** | **20** | **11** | **208** |

Three properties this preserves, and they are why the demotions are safe:

1. **A cell is a runtime promise, so merging crates never merges cells.** One
   source producing N arches keeps N rows and N cells — booting rv64 with real
   NetX Duo proves something x86_64-with-a-host-socket-shim does not.
2. **Every removal is provably free on this axis.** `Stm32F4` has **0 Runtime**
   cells; the four scaffolds contribute **zero cells of any tier**.
3. **Gaps stay visible.** `Fvp` and `Px4` keep their `BuildOnly`/`CarveOut` rows
   with their reasons — RFC-0051's rule that a gap is a visible row, never an
   absent file.

## Defects found along the way

These are independent of which model wins; all were verified.

### The per-board tax

`mps2-an385-freertos` — one board — is named in **45 functional files** (plus
~80 docs):

```
cmake/board/…mps2-an385-freertos.cmake   cmake/platform/nano-ros-freertos.cmake
cmake/board/…mps2-an385.cmake            cmake/NanoRosPackageXml.cmake
cmake/templates/freertos_app_config.c.in examples/fixtures.toml
packages/boards/board-support.toml       just/sdk-env.just
nros-sdk-index.toml                      packages/tooling/nros-build-paths/src/lib.rs
Cargo.toml (workspace members)           config/freertos-lwip/nros-platform.toml
6 board crates · 2 platform crates · 6 CLI sources · 8 test fixtures · scripts/
```

Nothing regenerates any of it, and `scripts/` carries three drift-checkers
(`check-board-manifest-drift.sh`, `check-board-abi-mirror.sh`,
`check-profile-board-mirror.sh`) whose only job is to notice when the copies
diverge. Under the R2 model most of this simply should not exist: an ecosystem
integrated by a shell adds **zero** per-board files.

### The arch trap — hard blocker for new silicon

`config/freertos-lwip/nros-platform.toml`, a *platform* manifest, pins the CPU:

```toml
arch = "cortex-m3"                 # scalar, not a list
[arch.cortex-m3]
target_match   = "thumbv7m"
target_exclude = "thumbv7em"       # ← excludes Cortex-M4F / M7
```

No other `[arch.*]` block exists, so FreeRTOS + lwIP on Cortex-M7 is excluded by
construction. Three faults at once:

1. CPU arch is a hardware fact in the platform layer — a direct RFC-0049
   duty-rule violation (*"platform toml = software-stack facts; board toml =
   hardware facts"*).
2. The mechanism already supports better: `config/bare-metal` uses a list,
   `arch = ["cortex-m3", "riscv32imc"]`, first-`target_match`-wins, and even
   defines an unused `[arch.cortex-m4f] target_match = "thumbv7em"`.
3. The board layer cannot fix it from outside. Verified in
   `nros-board-common/src/platform_config.rs`: `PlatformConfigFile` owns `arch`
   and `[build.zenoh]`; `BoardKnobsFile` parses **only `[knobs]`**, commented
   *"The rest of nros-board.toml … is ignored here."*

So RFC-0049's four-rung ladder is implemented for **policy knobs** and absent for
the **build block** — an out-of-tree board on new silicon must edit a file inside
nano-ros, the exact thing RFC-0049 §Resolution promises it will not have to.

The same file also names a specific board in `required_env`'s help text
(`packages/boards/nros-board-mps2-an385-freertos/config`). Help text only, no
build coupling — but the same inversion: the platform layer knows about one
board because that board is the only one that ever existed there.
`config/freertos-lwip/` is the MPS2-AN385 board manifest wearing a platform's
name; a second FreeRTOS board is what forces them apart.

### "Platform" names four different things

| Source | Values |
|---|---|
| `cmake/platform/nano-ros-<p>.cmake` | `baremetal esp_idf freertos nuttx posix threadx zephyr` |
| `config/<p>/nros-platform.toml` | `bare-metal freertos-lwip generic nuttx orin-spe posix threadx zephyr` |
| `nros-sdk-index.toml [board.*].platform` | `bare-metal freertos nuttx posix threadx zephyr` |
| `nros-board.toml` `platform =` | `bare-metal esp32 freertos nuttx orin-spe posix stm32 threadx-linux threadx-riscv64 zephyr` |

`baremetal` vs `bare-metal`; `freertos` vs `freertos-lwip`; `esp_idf` has a cmake
module but no `config/` dir; `generic` the reverse; and the descriptor mixes in
**silicon families** (`stm32`, `esp32`) that are not platforms at all. RFC-0049's
chain — *"the app names its board → the board toml names its platform → the
loader follows that two-hop chain"* — is only sound with one namespace.

### Hardcoded board discovery

`cmake/platform/nano-ros-{freertos,nuttx,baremetal}.cmake` FATAL_ERROR unless the
module is at `${CMAKE_CURRENT_LIST_DIR}/../board/nano-ros-board-${NANO_ROS_BOARD}.cmake`,
and `nano_ros_use_board()` resolves `${NROS_REPO_DIR}/packages/boards/nros-board-${NAME}`.
Both in-tree only. Confirmed that **no consumer works around this** — every
`NANO_ROS_BOARD` value set anywhere (`mps2-an385-freertos`, `nuttx-qemu-arm`,
`nuttx-qemu-riscv`, `riscv64-qemu`, `threadx-linux`) has an in-tree module,
because nothing else can.

### Documentation contradicts a decided policy

`book/src/porting/vendor-overlay.md` §"Naming convention" tells vendors to
*"Publish to crates.io as `nros-board-<vendor>-<chip>-<rtos>`"*. RFC-0040 D1:
*"No `nros*` crate is ever published."* The book page is the first thing a new
vendor reads. Fix regardless of everything else here.

## Constraints this design inherits

| Decision | Source | Effect here |
|---|---|---|
| nano-ros is a source distribution; nothing on crates.io | RFC-0040 D1 | Distribution is git + index, never a package registry |
| Vendor BSP is owned by the vendor; integrate per-RTOS | **RFC-0012 (Stable)** | R2's spine; this RFC finishes it |
| One schema, one file per package, 4-rung ladder | RFC-0049 (Stable) | Extend the ladder to the build block |
| Keep low-tier boards in-tree | phase-320 W3.d | Do not relocate anything; shells add nothing to relocate |
| Tier is metadata, not layout | phase-320 W3 | No tier or vendor directories |
| Board crate merges 27 → 19 | phase-322 (deferred) | Orthogonal |
| Out-of-tree boards self-describe deps | RFC-0087 / phase-420 W6+W8 (RFC-0013 + phase-201 superseded 2026-09-04) | The revival case — answered generally, not per-board |

## Sequencing

0. ~~**Unblock Cortex-M7 on FreeRTOS**~~ — **DONE 2026-08-01.**
1. ~~**Board-less FreeRTOS platform**~~ — **DONE 2026-08-01.** Together, 0 and 1
   are everything nano-ros needs before the shell can be written.
2. **Write `integrations/s32ds/`** and bring MR-CANHUBK3 up through it. This is
   now the verification step for the whole R2 model: if the shell stays under
   the 200-LoC cap and adds no per-board files, the model holds. Discovered, not
   inferred — phase-321's retro is explicit that inferred gaps do not survive.
3. **CM7 toolchain file**, splitting arch from language/ABI policy.
4. **Measure the mandatory subset of the 92-function ABI** and put the number at
   the top of the porting guide.
5. **Delete `net_stack`**; document `LinkFeatures` + `LinkPolicy` as *the*
   network-customization contract, with `custom` as the documented escape.
6. **`[board.*]` → `[profile.*]`** + index composition, so no downstream board
   ever needs an entry in nano-ros's index.
7. **Unify the platform vocabulary** to one namespace.
8. **Extend the RFC-0049 ladder to the build block** — board layer contributes
   `[arch.*]`, `include_paths`, `required_env` (allowlist, not open merge). The
   `-mfpu` hardcoded by revision 2 is the standing debt this pays off.
9. Fix `vendor-overlay.md` vs RFC-0040 D1.

Revision 3 adds the matrix work, ordered by value per unit of risk:

10. **Finish the Cortex-M4F/M7 unblock.** Step 0 fixed the *config* half
    (`config/freertos-lwip/nros-platform.toml` now lists
    `arch = ["cortex-m3", "cortex-m7"]`), but `nros-board-freertos/build.rs:273-287`
    still **hard-panics** on any `thumb*` target that is not `thumbv7m` unless
    `FREERTOS_CFLAGS` is set. Every industrial FreeRTOS board this RFC wants to
    reach (S32K344 is a Cortex-M7) trips it. Small, and it gates the rest.
11. **phase-322 W1.g, then W1.a** — the shared runtime `Config` FIRST (12
    hand-rolled `Config` structs, ≥9 carrying the identical
    `{mac, ip, netmask, gateway, locator, domain_id}` core; without it every merge
    below re-forks), then merge the NuttX board crates (1059 byte-identical lines,
    into the `nros-board-nuttx` crate that already exists and that both boards
    already depend on). Merging forces the tier-registry row key to become
    `(crate, matrix_platform)` — do that in the same change or the completeness
    gate breaks.
11b. **Extract the ThreadX riscv64 arch port** (`tx_thread_*.S` + `tx_port.h`,
    ~1250 lines) out of the board crate into its own layer-2 unit. Layer 1 is
    already board-generic (`run_entry::<B, C, F, E>`), so both ThreadX boards then
    thin to ~150–300-line overlays and a future riscv32/Cortex-R board reuses the
    port instead of forking it again. **Do NOT merge the two ThreadX boards into
    one crate** — see "Three layers, not two".
12. **Add the Zephyr QEMU Cortex-M witness** — settle the `[OPEN]` board choice,
    add `PlatformId::ZephyrQemuCortexM` + its cell group, join the west-lane
    fixture exemption. Then fold `nros-board-fvp-aemv8r-smp` into
    `nros-board-zephyr` as a conf bundle, so Zephyr boards stop being crates.
13. **Template the FreeRTOS per-board files** so a second nanoros-owned board is
    ~80 lines: hoist the 299 lines of config headers (2 board-specific values) to
    shared defaults, delete the `configure_arm_cm3` / `emit_nros_app_config`
    duplication in `build.rs`, retire the 727-line `startup.c` shadow copy, and
    drop the ~180-line LAN9118 diagnostic duplicated into both C files.
14. **Rename `native` → `linux`** (platform stays `posix`); merge
    `nros-board-linux` + `nros-board-linux` per phase-322 W1.e.
15. **Make `net_stack` a real axis** — the tier registry and the matrix key off
    it instead of it being documentation-grade metadata.
16. **Demote**: `stm32f4`/`rtic-stm32f4` out of the matrix and into the book's
    customization example; delete the scaffolds, untangling `orin-spe`'s
    pseudo-platform role first.

Note what is *not* here any more: R1's "generate the per-board mirrors" work.
Under R2 most of those files should not exist for new targets at all, so
generating them would be automating something that ought to be deleted. Keep
generation on the table only for whatever survives step 2.

## Open questions

- **[OPEN]** Is a vendor SDK build (S32DS/RTD) row 1 or row 2? The single most
  consequential question here; step 2 answers it.
- **[OPEN]** What is the mandatory subset of the 92-function platform ABI?
- ~~**[OPEN]** Does anything read `net_stack` for build-responsibility rather than
  stack identity?~~ **ANSWERED (R3, 2026-08-04): nothing reads it at all.** Its
  only consumer is the `NetStack` field of the descriptor struct
  (`board_descriptor.rs:77-82`) — documentation-grade metadata. R3 proposes
  promoting it to the matrix's cost axis (sequencing step 15).
- **[OPEN]** `[board.fvp-aemv8r-smp] platform = "bare-metal"` in the index vs its
  Zephyr `board.cmake` — drift, or does the index's `platform` mean
  "provisioning family"? If the latter, the field is misnamed.
- **[OPEN]** `config/bare-metal` defines `[arch.cortex-m4f]` but never lists it in
  `arch = [...]`. Dead, or reachable another way?
- **[OPEN]** Three in-tree board modules — `esp32-c3`, `mps2-an385`,
  `stm32f4-nucleo` — are selected by no caller found. **Not** a deadness claim;
  phase-321's retro is explicit about that failure mode. Needs checking.
  *(R3 partial answer: on the test axis the question is settled — `Stm32F4`
  carries 3 BuildOnly and **0 Runtime** cells, and the four scaffold board crates
  contribute zero cells of any tier. That bounds what removing them can cost;
  it does not by itself prove no build-side caller exists.)*
- **[OPEN]** Which QEMU-able Zephyr board carries a usable Ethernet driver for
  the new Cortex-M witness (`mps2/an385` + `smsc911x`, versus SLIP/TAP on
  `qemu_cortex_m3`)? Blocks sequencing step 12; not answerable from this tree.
- **[OPEN]** Does a hosted non-Linux target (macOS, FreeBSD) earn a tier-3 row
  once `timer_create` has a fallback, or is `linux` the only hosted board we ever
  claim? R3 makes either possible without renaming the platform.
- **[OPEN]** Should there be a `integrations/generic-cmake/` shell — the fallback
  for "my RTOS has a CMake build and nothing else"? It would be the honest
  default for the long tail.

---

## Exploration log

### 2026-07-31 — initial survey

- **Per-board tax**: `rg -l mps2-an385-freertos` over the functional tree → 43,
  later corrected to **45** (the first scan omitted the repo root and `config/`).
- **Artifact coverage**: 12/27 boards have `nros-board.toml`; 2/27 have
  `board.cmake`; 8 `cmake/board/*.cmake` modules live outside `packages/boards/`.
- **Platform module contracts**: `NANO_ROS_BOARD` required by freertos, nuttx,
  threadx, baremetal; not by esp_idf, zephyr, posix.
- **Gating works end to end**: `NROS_BOARD_GATED_PKGS` → `[gated.arm-fvp] env =
  "ARM_FVP_DIR"` → `nros doctor --board`.
- **Two board-support impls acknowledged in-tree** (phase-274 W3 comment), and it
  had already caused an undefined-reference link failure.
- **Cyclone DDS on FreeRTOS C/C++ is unexercised**: every FreeRTOS row in
  `examples/fixtures.toml` is `rmw = "zenoh"`; `just/freertos.just` says the
  c/cpp cyclonedds cells *"are out of scope (none exist today)"* while
  `cmake/platform/nano-ros-freertos.cmake` carries phase-186 Cyclone
  self-provision flags. Plumbing exists, no consumer.

### 2026-07-31 — second pass: the RFC-0049 config layer

- **The arch trap** (see §Defects). Cross-checked all eight platform manifests;
  only `bare-metal` uses the list form.
- **Board layer cannot contribute build config** — `BoardKnobsFile` parses only
  `[knobs]`.
- **Net stack is a platform fork**: `config/freertos-lwip/` vs `config/orin-spe/`
  (FreeRTOS + IVC). Does not compose.
- **`required_env`** (`name` + `help` + `validate_subdir`) already models external
  SDKs well; two boards re-implement it by hand in `build.rs`.
- **NetX contract**: C `driver_entry` passed to `nx_ip_create()`, handling
  `NX_LINK_*`. Both NetX drivers are C + CMake, no Rust crate.
- **Scaffolder covers only the Rust half**: `nros new board --for-platform` emits
  `Cargo.toml`, `src/lib.rs`, `nros-board.toml`, and nothing else.

### 2026-08-01 — third pass: the contract and the vocabulary

- Extracted the six-item board contract; found ESP-IDF satisfies it by aliasing.
- Four incompatible "platform" vocabularies.
- Descriptor value sets: `entry_kind` ∈ {`board-run` ×10, `hosted-main` ×2,
  `zephyr-staticlib` ×1}; `link_kind` ∈ {`none` ×11, `nuttx-staging` ×2};
  `toolchain` ∈ {`stable` ×9, `nightly` ×3, `esp` ×1}. Narrow closed sets — good
  enum candidates.

### 2026-08-01 — fourth pass: the reframe (R2)

Prompted by the objection that enumerating net stacks does not scale, and that
nano-ros should be an embeddable library shipping its build system.

- **RFC-0012 is Stable and already says this**, drafted 2026-05 against the
  `(vendor × board × SDK-variant)` explosion. Its Layer 3 is "integration shell
  per RTOS"; Layer 4 is "Vendor BSP — owned BY the vendor, NOT by nano-ros".
  Revision 1 of this RFC had partially re-derived it while still modelling boards
  by enumeration. R2 drops the enumeration.
- **Layer 3 is real, not aspirational**: `integrations/nano-ros` (ESP-IDF, 3
  files, 146 lines under a documented 200-line cap), `integrations/nuttx` (15
  files — CMake entry + Kconfig/Make.defs for make-driven configs),
  `integrations/px4` (13), `integrations/platformio` (2, thin). Plus `zephyr/`
  with `module.yml` declaring cmake + kconfig entry points, `snippet_root`, and
  six Twister samples.
- **The seam is 92 `nros_platform_*` functions**, ~33 of them sockets. That is
  the embeddable-library boundary.
- **`LinkFeatures` already is the anti-enumeration mechanism** — {tcp,
  udp_unicast, udp_multicast, serial, raweth, tls, ivc, custom} with `LinkPolicy`
  per-platform masks (*"Orin SPE has no Ethernet → Force(false)"*), and `custom`
  as a runtime-pluggable transport since Phase 115.B. Capability negotiation, not
  stack identity. `net_stack` is redundant with it and strictly weaker.
- **Zephyr's search roots** (`BOARD_ROOT`, `SOC_ROOT`, `DTS_ROOT`,
  `ZEPHYR_EXTRA_MODULES`) are the discovery model to copy; Yocto's `.bbappend`
  override model is the part to reject.

### 2026-08-01 — fifth pass: revisions 1 and 2 landed

Two changes, both verified, both strictly additive.

**Revision 1 — board-less FreeRTOS** (`cmake/platform/nano-ros-freertos.cmake`).
The two `FATAL_ERROR`s were the *only* thing requiring a board: every later use
of the overlay's outputs is already guarded (`if(DEFINED FREERTOS_STARTUP_*)`,
`if(TARGET freertos_platform)`, `if(COMMAND nros_board_link_app)`), and the Phase
186 Cyclone block already falls back for `FREERTOS_DIR` / `LWIP_DIR`. So the edit
is: enter board-less mode when `NANO_ROS_BOARD` is undefined *and*
`freertos_platform` already exists, else fail with a message that now names the
shell option. Verified: `if`/`endif` and `function`/`endfunction` balance
(17/17, 1/1); the `NANO_ROS_BOARD`-defined path is untouched, so mps2 is
unaffected.

**Revision 2 — Cortex-M7** (`config/freertos-lwip/nros-platform.toml`).
`arch = "cortex-m3"` → `["cortex-m3", "cortex-m7"]` plus an `[arch.cortex-m7]`
block. Verified against the real file with the actual selection rule
(first `target_match` substring hit, `target_exclude` vetoes):

| triple | profile |
|---|---|
| `thumbv7m-none-eabi` | `cortex-m3` (unchanged) |
| `thumbv7em-none-eabihf` | `cortex-m7` (new) |
| `thumbv7em-none-eabi` | none — fails loudly |
| `riscv32imc-…` | none |

`target_match` is the **full hard-float triple**, not the bare `thumbv7em` token:
`target_exclude` is also a substring test and `thumbv7em-none-eabi` is a
substring of `thumbv7em-none-eabihf`, so soft float cannot be excluded — only
not-matched. Without this, a soft-float thumbv7em build would have silently
compiled with `-mfloat-abi=hard`.

All eight `config/*/nros-platform.toml` still parse; `cargo test -p
nros-board-common --features build-helpers` green (8 + 8 tests). No other file
defines `[arch.cortex-m7]`, so the "duplicate arch blocks must be byte-identical"
rule (`arch_identical_duplicate_ok_conflict_errors`) is not engaged.

**Debt knowingly incurred:** `-mfpu=fpv5-sp-d16` is a *hardware* fact now sitting
in a *platform* manifest — the exact fault §"The arch trap" describes. It is
correct for MR-CANHUBK3 and for nothing else in particular, and `ArchEntry` has
no per-board gate to express it (`target_match` / `target_exclude` only).
Sequencing item 8 pays this off; the block carries a comment saying so.

### 2026-08-01 — sixth pass: revision 3, and revision 1 validated by a consumer

`integrations/s32ds/` written — probe, CMake shell, `makefile.defs`, README.

**It ends at 226 code lines, over Phase 139's ≤200-LoC-per-shell cap** (92 +
132 + 2, comments and blanks excluded). It started at 193 and grew as validation
found real requirements: the Windows-path re-rooting, the `LWIP_CONFIG_DIR`
guard, and the link-fragment generation. Reporting rather than squeezing,
because the overage is informative: **this shell is doing work the IDF shell
does not**, namely reverse-engineering a foreign build system's flags. ESP-IDF
hands nano-ros `idf::freertos` and `idf::lwip` directly; S32DS hands it a
CDT-generated directory and nothing else.

The cap is a good instinct — a shell that keeps growing means nano-ros is
absorbing responsibility that belongs to the host. But 132 of the 226 lines are
the probe, and the probe exists precisely so the *user* does not hand-maintain
duplicated ABI flags. Either raise the cap for ecosystems with no CMake surface,
or split the probe into a shared helper for the vendor-SDK shells that will
follow (STM32Cube, MCUXpresso). **[OPEN]**

**Board-less mode confirmed by a real consumer.** Configuring the shell against
the MR-CANHUBK3 project emits, verbatim:

```
-- nano-ros-freertos: board-less mode — an integration shell already composed
   `freertos_platform`; skipping the per-board overlay.
-- Configuring done / Generating done
```

Corrosion picked up `thumbv7em-none-eabihf` from the generated toolchain. **The
board contributed zero files to the nano-ros tree**, which is the R2 claim.

**`libnros_platform_freertos.a` builds for Cortex-M7 against NXP's kernel fork**
(all four TUs: `platform.c`, `timer.c`, `cyclonedds_compat.c`, `net.c`). And the
ABI is objectively right — `readelf -A` on our archive versus the NXP project's
own shipped `msg_converter.o`:

| Tag | ours | NXP's |
|---|---|---|
| `Tag_CPU_arch` | v7E-M | v7E-M |
| `Tag_CPU_name` | 7E-M | 7E-M |
| `Tag_FP_arch` | FPv5/FP-D16 for ARMv8 | FPv5/FP-D16 for ARMv8 |
| `Tag_ABI_VFP_args` | VFP registers | VFP registers |

Identical. Probing CDT's `.args` instead of retyping flags is validated against
the vendor's own artifacts, not just against itself.

**A claim in the probe was wrong, and building found it.** The first version
dropped every Windows-absolute include path, on the stated basis that each was a
duplicate of a project-relative one. False: only `RTD/include` is duplicated.
`generate/include`, `generate/src` and `board` appear **only** as
`C:/Users/.../<project>/…`, and `generate/include` is where `FreeRTOSConfig.h`
lives — so the shim compile failed with `FreeRTOSConfig.h: No such file or
directory`. Fixed by splitting the two cases: paths *inside* the project are
re-rooted onto the local copy; paths *outside* it (the `C:/NXP/S32DS…`
PlatformSDK headers) are dropped and **listed** so a later missing-header error
has an actionable trail. This is exactly the kind of premise that does not
survive verification (phase-321's retro) — it was inferred from a sample of the
include list rather than checked against all of it.

**Two integrator-owned files the shell now demands up front**, rather than
failing deep in a C compile:

- `LWIP_DIR` — the stack itself; the error text spells out the three options
  (lwIP + netif, implement the socket ABI subset, or `LinkFeatures{custom}`).
- `LWIP_CONFIG_DIR` — `lwipopts.h` + `arch/cc.h`. These are per-deployment
  (buffer counts, `MEM_SIZE`), and the mps2 pair is a QEMU-sized starting point
  that must be re-tuned for the S32K344's 320 KB SRAM.

**`NANO_ROS_ROOT` vs `NANO_ROS_ROOT_DIR`.** `packages/api/nros-{c,cpp}` include
`${NANO_ROS_ROOT}/cmake/NanoRosFeatureSet.cmake`, but the root CMakeLists sets
`NANO_ROS_ROOT_DIR` — a different variable. Setting the former is the consumer's
job by convention (`nano_ros_workspace()`, `NanoRosBootstrap.cmake`,
`zephyr/CMakeLists.txt`, `integrations/px4/NanoRosPx4Module.cmake` all do it).
The IDF shell does **not**, and leans on the user's component to do it — worth
tidying, since the failure is the opaque `include could not find requested file:
/cmake/NanoRosFeatureSet.cmake`.

**No regression: `just freertos build-fixture-extras` exits 0** with revisions 1
and 2 in place. The mps2/CM3 path selects `--target=thumbv7m-none-eabi` with
`platform-freertos` exactly as before; remaining output is the usual newlib
`_gettimeofday/_isatty/_kill is not implemented` bare-metal warnings.

Getting to that answer took three attempts, and two of the failures were
environmental rather than caused by the change — worth recording because they
will recur:

1. **A `| tail` pipeline masked the failure.** The first run was reported as
   "exit code 0"; that was `tail`'s exit, not the build's. It had actually died
   on the stale-CLI guard. Redirect build output to a file and echo `$?`; do not
   pipe build commands into `tail`.
2. **Stale-CLI guard, twice.** First for a genuinely stale binary
   (`just setup-cli` fixed it), then again with *uncommitted CLI edits* —
   `packages/cli/nros-cli-core/src/{cmd/mod.rs,lib.rs,stale_guard.rs}` were
   modified in the working tree, one of them being the guard itself. Cleared
   with the documented `NROS_SKIP_STALE_CHECK=1` escape, which is correct here:
   the change under test is CMake, orthogonal to the CLI.
3. Disk exhaustion mid-run (`/home` 100% full, `target/` at 115 GB) truncated an
   edit and failed a cargo build. Unrelated to the design, but it is why a
   partially-applied file appeared briefly.

**Link fragment generation validated.** `nros-libs.mk` emits the shim's concrete
path plus a make-time `$(wildcard $(NROS_CARGO_LIB_DIR)/*.a)` over Corrosion's
`<build>/corrosion/` output dir. Deferring cargo libs to make time avoids
Corrosion's imported targets, which do not reliably support `TARGET_FILE`.

One self-inflicted trap worth naming: `file(GENERATE)` evaluates generator
expressions **everywhere in CONTENT, including inside text intended as comments
in the emitted file**. A comment mentioning a bare `TARGET_FILE` genex with an
empty argument aborted the generate step with "Expression syntax not
recognized". Do not write literal genex syntax into generated content.

Not yet checked:
- Whether any code reads `net_stack` (needed before deleting it).
- The mandatory subset of the platform ABI.
- `integrations/platformio/` is 2 files — is the PlatformIO shell real or a stub?
- Whether an `integrations/<vendor-sdk>/` shell is viable for S32DS specifically:
  it is a make-driven Eclipse CDT project, closer to NuttX's make path than to
  IDF's CMake path.

### 2026-08-04 — seventh pass: the supported matrix (revision 3)

Written to answer "which platform/board/arch set covers most users while easing
the maintainer's burden", and to test the maintainer's principle that one source
should serve many SoCs via build options. Everything below was measured in-tree.

**The net seam is already generic — this was the load-bearing check.** The 92
platform functions split `platform.h` 58 / `platform_net.h` 29 / `platform_timer.h`
4 / `platform_zephyr.h` 1, and networking is deliberately isolated in its own
header so stackless targets can omit it. There is **one net implementation file
per platform, never per board**. Per-board deltas are expressed as (a) weak C
symbols — `nros_board_init_eth`, `nros_board_register_netif`,
`nros_board_poll_netif`; (b) a macro argument naming the MAC driver —
`define_network_state!(NETWORK_STATE: Lan9118, …)`; (c) descriptor data.

The decisive case is ThreadX, because it is exactly the maintainer's worry
(host-offloaded sockets versus a real driver): `nros-platform-threadx/src/net.c`
is **one file with two backings** — on threadx-linux the NetX-Duo BSD calls
resolve to a shim over host POSIX sockets (`drivers/net/nsos-netx/src/nsos_netx.c:218`),
on threadx-qemu-riscv64 they are real NetX Duo over virtio-net. The switch is
which archive the board's `build.rs` links. No `#ifdef`, no branch. Adding a
board on an existing platform edits no `net.c`/`net.rs` anywhere.

The only two board-shaped compile-time branches in the tree are `orin-spe`
(modelled as a pseudo-platform in link-feature selection) and a ThreadX std/no_std
libc-tier split in `NanoRosFeatureSet.cmake:116-128`. Neither is networking.

**Conclusion: portability is not the blocker; per-board FILES are.** The seam was
built right and the board crates were never factored down to the data it needs.

**Zephyr is already board-neutral, and its coverage is narrower than it looks.**
`cmake/platform/nano-ros-zephyr.cmake` (53 lines) names no board — Zephyr's own
`BOARD=` owns it, via `zephyr/cmake/nano_ros_use_board.cmake:50-59`.
`nros-board-zephyr` (540 lines) implements only `NetworkWait` over `net_if.h`.
A real non-native Zephyr board already exists at 160 lines
(`nros-board-fvp-aemv8r-smp`, Cortex-R52, tier 3 only because the model is
license-gated), structured as `boards/<board>.conf` + `.overlay` + `prj.conf` +
a `Config` — i.e. the thin-overlay shape, already proven twice.

But all 28 Zephyr runtime configs are `native_sim/native/64`, and the overlay
sets `CONFIG_NET_SOCKETS_OFFLOAD=y` / `CONFIG_ETH_NATIVE_TAP=n` so no TAP device
or root is needed. So Zephyr's own IP stack, a real driver, and 32-bit pointer
width have **never** been exercised. `nros-platform-zephyr/src/net_wait.c:53`
guards the NSOS path with `#ifdef CONFIG_BOARD_NATIVE_SIM`; its else-branch is
untested. This is what makes the Cortex-M witness a coverage fix rather than a
breadth claim, and the QEMU side is free — `nros-tests/src/qemu.rs:242` already
boots `-machine mps2-an385 -nic user,model=lan9118` for two other families.

**NuttX per-board delta is already data; the files just were not moved.** The
shared helpers take `NUTTX_CROSS`, `NUTTX_ARCH_CFLAGS`, `NUTTX_LIBGCC_FLAGS`,
`NUTTX_ARCH`, `NUTTX_VECTORTAB_OBJ`, `NUTTX_LINKER_SCRIPT`, `NUTTX_ARCH_INCLUDES`,
`NUTTX_DIR`, and the staging link list is **discovered by scanning
`$NUTTX_DIR/staging` for `lib*.a`** rather than hardcoded per board. Both board
crates' `build.rs` bodies are the same four calls. Yet 1059 lines are
byte-identical duplicates (`c/nuttx_run_tiers.c` 587, `src/config.rs` 261,
`src/node.rs` 133, `src/entry.rs` 43, `c/nuttx_builtins_stub.c` 35), and the
crate to hold them (`nros-board-nuttx`) exists with both boards already depending
on it. A move, not a design change — phase-322 W1.a.

**FreeRTOS: ~80 % parameterizable, and the residue is small.** Classified
`startup.c` (727) and `c/board_mps2.c` (346): genuine non-optional silicon is
~42 lines in the former (vector table 19 + `Reset_Handler` 17 + ~6 LAN9118 config)
and ~23 in the latter. The single largest "silicon" block —
`nros_freertos_diag_network`, ~180 lines of raw LAN9118 CSR reads and
hand-assembled ARP frames — is a diagnostic nothing calls on the working path,
and it is duplicated into **both** files. `startup.c` itself is a pre-152.1.B
monolith: ~575 of its 727 lines are a live copy of code already lifted into
`nros-board-freertos`, kept alive only because the CMake lane compiles it while
the cargo lane compiles `board_mps2.c`. `config/lwipopts.h` (133) and
`config/arch/cc.h` (55) have **zero** board-specific content;
`config/FreeRTOSConfig.h` (111) has two lines; the linker script has three
numbers. The shared crate already owns the kernel/lwIP build and a 973-line
family driver over a working weak-hook seam.

**Naming.** `board_path_for` maps both `"native"` and `"posix"` to
`::nros_board_linux::LinuxBoard` (`nros-orchestration-ir/src/lib.rs:78`) — two
names, one implementation, and `nros-board-linux` (549 lines) is never named by
any generated entry. On portability: no `_WIN32` anywhere; five `__APPLE__`
branches in `nros-platform-posix/src/platform.c` (a `pthread_cond` fallback for
absent unnamed `sem_t`) but `src/timer.c:72` calls `timer_create` with no
fallback and macOS has no POSIX timers; the only other OS gate is
`#ifdef __linux__` for `MSG_NOSIGNAL` (`net.c:216`) with a portable else. All 19
CI jobs are `ubuntu-*`. Hence: platform `posix` (the seam), board `linux` (the
promise), `native` retired.

**Matrix cost of the proposal, counted.** `matrix::CELLS` = 202 (174 Runtime,
17 BuildOnly, 11 CarveOut). `Stm32F4` has 0 Runtime cells and the four scaffolds
have zero cells of any tier, so both removals are free on the test axis. The one
addition is the Zephyr Cortex-M cell group, which joins the existing west-lane
exemption in `every_runtime_cell_has_a_fixture_row` rather than growing
`fixtures.toml`.

Not verified here, and carried as [OPEN]: which QEMU-able Zephyr board has a
usable Ethernet driver (no Zephyr checkout on the authoring host); whether a
build-side caller exists for the three unreferenced board modules (the test axis
is settled, the build axis is not).

### 2026-08-04 — eighth pass: the three-layer cut, and the ThreadX arch port

Prompted by the maintainer asking whether `threadx-{linux,qemu-riscv64}` could
merge into one crate with per-arch fixtures, and the same for NuttX and Zephyr.
Measured all three; the answer differs per family, and the useful cut turned out
not to be board-against-board.

**ThreadX layer 1 is already done.** `nros-board-threadx/src/entry.rs:1-40`
documents the shape and the code is generic over the board:
`run_entry::<MyBoard, Config, F, E>(cfg, setup)`, with internals bounded
`B: BoardPrint + BoardExit` (`entry.rs:92,232,259,348,412`). Both board crates
already delegate to it. So "lift the shared logic into a family driver" is not
outstanding work — it shipped.

**What is left in the ThreadX board crates is asymmetric, and the asymmetry is
layer 2.** riscv64 totals 3598 lines against linux's 1191, and the delta is
almost entirely `c/tx_thread_context_restore.S` (392),
`c/tx_thread_schedule.S` (317), `c/tx_thread_context_save.S` (293) and
`config/tx_port.h` (252). Their headers state the provenance: modified copies of
`third-party/threadx/kernel/ports/risc-v64/gnu/src/`. The reason is recorded in
`tx_port.h:1-15` and is legitimate — upstream's rv64 port types `ULONG` as
`unsigned long` (8 bytes), but NetX Duo's packet code does `ULONG *` arithmetic
assuming 4-byte words, so the port retypes `ULONG` to `unsigned int` "matching the
Linux x86_64 and all AArch64 ThreadX ports". That retype shifts every `TX_THREAD`
field offset, so the assembly had to be forked to use explicit byte offsets
(`tx_port.h:37-56`). `threadx-linux` carries no `.S` at all, because upstream's
Linux port already uses a 4-byte `ULONG`.

That is arch-port code, not board code: a second riscv64 ThreadX board would need
every line of it unchanged. Merging the two boards would `cfg`-gate 1250 lines of
RISC-V assembly inside a crate that also serves Linux. Extracting layer 2 is
strictly better and leaves both boards as overlays. Rust-side residue supports
this: `src/config.rs` is 339 vs 330 with only 55 diff lines (~84 % identical, and
W1.g's `BaseConfig` takes most of it), while `src/lib.rs` (284 vs 447, 443 diff)
is exactly the `BoardInit`/`BoardPrint`/`BoardExit`/`BoardEntry` impls plus
console/exit/panic — i.e. the trait surface that already exists to differ.

This revises phase-322 W1.f: its "KEEP SEPARATE" verdict on the crate boundary
stands, but its stated reasons (distinct `#[panic_handler]`, hosted-vs-bare link
model, different net drivers) are all abstracted by the 212.N trait set already.
The un-abstracted thing it does not mention is the arch port.

**NuttX confirms the same model from the other side** — both upper layers come
from upstream, so its crates are pure layer 3, which is exactly why W1.a's merge
is straightforward while ThreadX's is not.

**A merge consequence nobody had recorded:** `board-support.toml` keys tier by
crate and gates "every board directory appears exactly once". A merged
`nuttx-qemu` crate serves arm (tier 1) and riscv (tier 2); a conf-bundle
`nros-board-zephyr` would serve three tiers. The registry row key must become
`(crate, matrix_platform)`, or the merge breaks its own completeness gate.

**Counted the target state** (boards 27→16, fixtures 344→336, cells 202→208 with
Runtime 174→177) and recorded it as three views of one decision. The honest
negative result: **crate merging buys no CI time.** Fixture rows are what cost
wall clock, and merging removes none of them — `stm32f4`'s 8 rows are 2 % of the
manifest against FreeRTOS's ~1370 s and native's ~1300 s per lane. Crate merging
is a maintenance-surface lever; the wall-clock lever is the tier/lane split that
phase-318 already shipped.

### 2026-08-04 — ninth pass: the FreeRTOS template, and the "~80 lines" claim measured

phase-337 W5 executed the FreeRTOS half of the target state. Three results, one
of which corrects this RFC.

**The mechanical part held, and is provably a pure move.** `config/lwipopts.h`
(133), `config/arch/cc.h` (55) and `config/FreeRTOSConfig.h` (111) hoisted into
`nros-board-freertos/config/`, leaving 12 lines in the board (two numbers and
three `#include`s); the 135-line linker script became a 7-line memory map that
`INCLUDE`s a shared section layout. Proof that this changed nothing: 19 objects
(FreeRTOS kernel, lwIP, the family glue) compiled against the pre- and post-move
headers are **byte-identical**, and the same object linked with the old script
and the new pair produces an identical section layout, symbol table and loadable
image under both GNU ld and rust-lld.

**The shadow copy is gone.** `startup.c` (727 lines) is deleted. Both lanes now
compile one set of sources: the family's `freertos_hooks.c` + `network_glue.c` +
`freertos_run_tiers.c` and the board's `board_mps2.c`, plus a new
`freertos_c_entry.c` that carries the ~150 lines that were genuinely
C-lane-only (semihosting stdio, the log writer, task creation, `main`). The
drift the split was hiding was real and is recorded in that file's header: the
shadow copy seeded the **platform** PRNG where the shared glue seeds `srand()` —
two different generators, one of which zenoh-pico's session ID actually reads.

**The "~60–80 lines for a second board" claim is HALF right, and the honest
number is 205.** W5.f wrote the complete file set for a hypothetical S32K344
board (`book/src/porting/freertos-board.md`) and measured it:

| | lines |
|---|---:|
| `config/*` (4 files) | 12 |
| `c/board_<name>.c` — vector table, reset, netif registration | 64 |
| **per-board delta — what this RFC estimated at 60–80** | **76** |
| `src/lib.rs` — board ZST + four trait impls | 57 |
| `build.rs` | 45 |
| `Cargo.toml` | 27 |
| **total a user actually writes** | **205** |

So the estimate was right about the layer it was counting (vector table, memory
map, clock, cflags, netif registration) and silent about the Rust and Cargo
scaffolding, which is 129 lines and is **not board-specific at all** — every
Cortex-M FreeRTOS board writes the same `BoardPrint`/`BoardExit` semihosting
impls and the same two-line `BoardEntry` delegations with a different type name.
That is the next template, and it is a macro in `nros-board-freertos`, not a
per-board file. Until it exists, quote 205.

The one cost no template removes is the MAC driver: `lan9118_lwip.c` is ~507
lines, and a board whose vendor SDK ships no lwIP netif pays that between its
silicon and lwIP.

## Revision 4 (2026-08-22) — the tier is a PROMISE with an OWNER, and onboarding is the real cost

Written after the S32Z270 bundle landed and the question "more boards keep
arriving; how do we balance platforms per tier" was asked directly. The answer
this RFC gave in revision 3 — a matrix of cells with tiers — is right and is
kept. What it lacked is an entry POLICY, and the evidence says the policy should
govern owners and onboarding, not test counts.

### The measurement that reframes it

| | |
| --- | --- |
| `matrix::CELLS` | **191** — 181 Runtime, 5 BuildOnly, 5 CarveOut |
| `fixtures.toml` rows | **422**, of which `linux` is **195 (46 %)** |
| lane coordinates | tier 1 **10**, tier 2 **14**, nightly **37**, tier 3 **51** |
| board registry | 5 tier-1, 6 tier-2, 2 tier-3, 9 infra — **0 with a maintainer** (13 non-infra rows, all grandfathered by W1) |

(Revision 3 quoted 202 cells / 174 / 17 / 11 from 2026-08-04. BuildOnly has since
fallen 17 -> 5, mostly by promotion. Re-measure before quoting; an early pass of
this revision counted 181 cells and "all Runtime" because its regex matched only
bare-identifier tiers and silently dropped `BuildOnly("reason")`.)

**A new platform costs +1 coordinate in tier 1, tier 2 AND nightly**, because
1-wise and pairwise both absorb a new axis VALUE cheaply, and +2 in tier 3. That
is not where bloat comes from. Measured cost of the last two boards:

* `freertos-posix` (phase-370): +2 cells, +1 coordinate per lane — and two RED
  gates (a recipe graph that could not produce its token, the lane table).
* `s32z270` (phase-372): 0 Runtime cells, +1 tier-3 coordinate — and **five** red
  gates: weak symbols, board tiers, leaf lock, provider announcements, matrix
  orphan.

**So the marginal cost of a board is not its tests. It is the onboarding
gauntlet, paid by whoever notices main is red.**

### What the ecosystem does

**Rust's target tier policy** makes ownership the gate: tier 3 requires >=1
named maintainer, tier 2 >=2, tier 1 >=3, with automatic demotion when the
requirement lapses. Two clauses transfer directly:

> Tier 2 targets **must not impose burden on the authors of pull requests** … do
> not post comments that derail or suggest a block on the PR based on tests
> failing for the target.

> Cannot substantially slow CI.

Under that policy S32Z270 — a *tier 3* board — reddening main for everyone is
the disqualifying condition, not a nuisance. `board-support.toml` carried
`maintainers = []` on all 22 rows with `check-board-tiers` printing "not
enforced yet (phase-320 W3.b)" — the valve existed and was not turned. **W1
turned it** (3/2/1 by tier), as a ratchet: the 13 rows that predate the rule are
grandfathered in a list that only shrinks, so a new board is bound the day it
lands and an existing one when someone claims it. The field is still empty
everywhere, which is the honest state — inventing an owner is worse than
recording none — and the ratchet is what lets the rule bind anyway.

**Zephyr's Twister** separates two things this tree fuses: `platform_allow` says
where a test CAN run, `integration_platforms` says where CI runs it BY DEFAULT,
and the docs warn against using the capability list for CI scoping. It also
tiers by test PURPOSE (`levels:` — smoke, unit, integration, acceptance, system,
regression), orthogonal to platform.

**But Zephyr's own issue #57595 argues FOR this tree's design**: their scope
rules are trial-and-error and untested, so "there is no warranty that the
behavior won't suddenly change". nano-ros's computed 1-wise/pairwise cover
cannot drift and is gated (`documented_lane_table_is_live`). **Keep it.** An
earlier draft of this revision proposed replacing it with declared per-test
platform lists; that is rejected on Zephyr's evidence.

**Zephyr on vendor boards:** "for product work, out-of-tree is the right call, as
it keeps your definition independent of Zephyr version updates and lives in your
own repo."

### The policy

1. **A tier is a promise with an owner.** Tier 1 >=3 maintainers, tier 2 >=2,
   tier 3 >=1. A board whose owner lapses demotes rather than rots. This makes
   the board count self-limiting without anyone arbitrating worth, and it is
   phase-320 W3.b's own intent.

2. **A board below tier 2 must not be able to redden a shared lane.** Onboarding
   is complete at merge or the board does not merge — which makes a scaffold
   (`just board-new`) an entry requirement rather than a convenience.

3. **Witness-less boards are `Tier::BuildOnly`, and the vocabulary already
   exists.** A consumer-required board with no runner takes a BuildOnly cell
   whose string says what unlocks it, plus a borrowed platform token. It
   contributes zero Runtime cells and ~2 fixture rows. This is what S32Z270 got
   (2026-08-22) and it is the shape every future vendor board should take.

4. **A smoke floor, a witness-gated ceiling.** Every supported platform earns
   exactly ONE Runtime cell (boots, delivers one message) to sit in tier 2; full
   cells in nightly require a witness. Today's distribution — Linux 72 cells,
   QemuBaremetal 1 — is defensible but undeclared, and a floor plus a ceiling
   makes it intentional. This is Zephyr's `levels:` idea applied to the axis
   this tree actually tiers on.

5. **A product board belongs in the product repo.** The out-of-tree seam is
   built (RFC-0064's own target state; phase-346 COMPLETE 2026-08-12). S32Z270
   exists for `autoware-safety-island`; it is a candidate to live there, with
   nano-ros keeping a link check and the consumer running it in its own CI. The
   trade is visibility for size, and it is the only durable answer to "more and
   more boards appear".

### The one honest tension

(5) trades away in-tree evidence that a board still links. If a board stays
in-tree instead, (1) and (3) contain its cost — but then it needs a named
maintainer under (1). Choosing per board is fine; choosing by default is what
this revision asks for.

Sources: [Rust target tier
policy](https://doc.rust-lang.org/nightly/rustc/target-tier-policy.html);
[Zephyr Twister](https://docs.zephyrproject.org/latest/develop/twister/index.html);
[twister scope rules #57595](https://github.com/zephyrproject-rtos/zephyr/issues/57595);
[Zephyr board porting](https://docs.zephyrproject.org/latest/hardware/porting/board_porting.html).

---

## Revision 5 (2026-09-06) — one board process, and the descriptor holds only primitives

Written after auditing every board in the tree to answer a simple question: does
a board of ours arrive the same way a user's board does? It does not. Revision 2
established that nano-ros declares what it needs and the integrator satisfies it;
revision 5 makes the *arrival* uniform, so that "no builtin road" is a property of
the code rather than a statement in a document.

R5 changes no R2/R3/R4 principle. It applies them to the descriptor and to
discovery, and it deletes the one board that grew its own format.

### The measurement — four processes, not one

Twelve `[[board]]` rows across eleven `nros-board.toml` files, plus one board
with no descriptor at all (2026-09-06):

| Route | How a board is added | Boards |
| --- | --- | --- |
| 1. Package | crate + `nros-board.toml` + `package.xml` announcing `<nano_ros_provides kind="board">` | 9 |
| 2. Descriptor-only package | route 1 minus the crate (`packages/boards/{linux,zephyr}/`) | 2 |
| 3. Bundle + `board.cmake` | 14 `NROS_BOARD_*` cmake variables, a Cargo mirror, and a `west` extension | 1 (`fvp-aemv8r-smp`) |
| 4. Nothing at all | an inline `board = "…"` string in `examples/fixtures.toml` plus a `.conf` hand-placed in each example leaf | the rest |

Route 4 is the finding that reframes this. Three Zephyr board ids are actually
built — `native_sim/native/64`, `mps2_an385`, `qemu_cortex_a53/qemu_cortex_a53/smp`
— and only the first is declared anywhere, wedged into the `zephyr` descriptor's
`names` array beside a real name. `qemu_cortex_a53` has no package, no descriptor
and no announcement; its per-board `.conf` lives in the example leaf that uses
it, so a second consumer copies the file. **Our own QEMU board did the same job
as the FVP board with zero files, while the FVP board spent eight plus a west
verb.** Neither is the process.

Three mechanisms are correct and unreachable — the class this tree keeps
re-finding (issues 0896, 0963, and `check-test-scripts-have-callers`):

1. **The phase-215 drift gate checks zero boards.**
   `phase215_f_manifest_drift.rs` walks top-level `packages/boards/nros-board-*/`
   dirs needing both `Cargo.toml` and `board.cmake`. The only `board.cmake` in
   the tree sits one level deeper, at
   `nros-board-zephyr/boards/fvp-aemv8r-smp/`, so the loop never reaches it —
   and no board carries `[package.metadata.nros.board]` at all, so even a fixed
   glob would hit `if msg.contains("no `[package.metadata.nros.board]`")
   { continue; }`. The gate guards an invariant nothing satisfies.

2. **The announcement is optional, so the newest board skipped it.**
   `check-provider-announcements.py` reads `if not os.path.exists(pkg_xml):
   continue  # not migrated yet`. `nros-board-mps3-an536-freertos` (phase-385,
   the most recent board) has a descriptor and no `package.xml`. Nothing said so.

3. **Discovery is one level deep, and the bundle case is patched with a
   `board.cmake` read.** `BoardCatalog::load_root` scans
   `<root>/*/nros-board.toml` and no deeper, so a bundle board cannot carry a
   descriptor of its own. `fvp-aemv8r-smp` resolves today only because
   `attach_bundle_aliases` walks `nros-board-<family>/boards/*/board.cmake` and
   appends the bundle name and its `NROS_BOARD_ZEPHYR_ID` as ALIASES onto the
   family descriptor — so the FVP board has no descriptor, it borrows the
   `zephyr` one. D4 deletes `board.cmake`, which deletes that alias path, so the
   depth fix is a prerequisite rather than a tidy-up.

   **The out-of-tree ROOTS, by contrast, are already right, and an earlier draft
   of this revision said otherwise.** `extra_board_roots()` reads a PATH-style
   `NROS_EXTRA_BOARD_PATH`, and `load_with_packages` absorbs a board declared
   inside the user's own workspace package — the colcon property that a
   workspace carries what it needs. What D2 asks for is the DEPTH and one scan
   shared with `provider_scan`, not the root list, which exists.

   Note none of this is what phase-346 fixed: that phase made the board
   FRAMEWORK resolvable from an out-of-tree leaf, through the Entry's build
   script. Descriptor discovery was never part of it.

### D1 — A board is a package, and that is the whole identity

Adopt RFC-0087 D1 for boards without exception: a directory with a `package.xml`
announcing `<nano_ros_provides kind="board" name="…"/>`, plus a sibling
`nros-board.toml` saying what it lowers to. A crate only if the board needs
bring-up code.

```
<any dir>/
├── package.xml        <nano_ros_provides kind="board" name="…"/>   ← identity
├── nros-board.toml    what it lowers to                            ← behaviour
├── prj.conf           optional, Zephyr only
├── boards/<flat>.conf / .overlay   optional, derived by convention
└── Cargo.toml, src/   optional — rung 2 of the customization ladder
```

Ours and a user's differ in exactly one respect: which search root found them.

### D2 — One scan, depth-independent, shared with `provider_scan`

The root list is already right (see the measurement's point 3). Two things are
not:

* **Depth.** `load_root` stops at one level, so a bundle board cannot own a
  descriptor and is patched in by reading `board.cmake` — the file D4 deletes.
* **Two scans.** `provider_scan` already walks `package.xml` with
  `COLCON_IGNORE` / `AMENT_IGNORE` / `NROS_IGNORE` honoured;
  `BoardCatalog::load_root` walks descriptors separately with its own rules.
  Boards are found by the descriptor walk and announced to the other one, which
  is exactly how a board comes to have a descriptor and no announcement.

D2 is therefore: boards are discovered by `provider_scan` like every other
provider, and `nros-board.toml` is read for a package that announced itself as a
board. That collapses the two walks, makes depth a non-question, and makes the
missing `package.xml` a resolution failure instead of a silent skip.

### D3 — `[board.zephyr]`, so routes 3 and 4 stop existing

One block carries the Zephyr facts a board needs, and conventions supply the
rest:

* `west_board` — the Zephyr board id. It stops being a string in
  `fixtures.toml` (route 4) and a `NROS_BOARD_ZEPHYR_ID` cmake variable
  (route 3), and it stops being smuggled through `names`.
* ``boards/<west_board with `/` → `_`>.{conf,overlay}`` — derived,
  present-if-exists. `board.cmake` already stated this rule in a comment and
  then hand-wrote both paths anyway.
* `prj.conf` — derived, present-if-exists.
* `runner` — from Zephyr's own board definition; stated only when it differs
  (the FVP's `armfvp` does).
* a Rust-support Kconfig module — derived and GENERATED, since its whole body is
  `default y if BOARD_<UPPER(first segment of west_board)>`.

### D4 — cmake reads a generated projection, never a second authored file

`nano_ros_use_board(<name>)` keeps its signature — a downstream consumer's call
site does not move — but its body runs `nros board cmake-vars <name> --out
<build-dir>/…` and `include()`s the result. A build artifact, never committed
(the SystemModel precedent, phase-330). `board.cmake` and the
`[package.metadata.nros.board]` mirror are deleted, and with them the two-faces
drift problem rather than the vacuous gate that watched for it.

The replacement test is a round-trip — descriptor → projection → parse back ==
descriptor — which cannot go vacuous, because it has no "skip if the other face
is missing" arm.

### D5 — The announcement becomes mandatory

`check-provider-announcements` loses the `continue`: a descriptor with no
sibling `package.xml` is an error, ratcheted against today's single offender.

### D6 — The descriptor holds primitives only

A field audit over all 12 rows. The rule is RFC-0087 D4's — a descriptor carries
only what no convention can produce — plus RFC-0049's duty rule, which says a
software-stack fact belongs to the platform and a hardware fact to the board.

**Keep (primitive):** `names`, `platform`, `supported_netstacks`,
`[board.capabilities]` heap/atomics/threads, `board_crate`, `target`, `chip`,
`[board.cmake] toolchain_file`.

**Move to the platform descriptor** — the board is copying a fact it can get
wrong:

| Field | Evidence |
| --- | --- |
| `platform_feature` (12/12) | 3 of 12 are NOT `platform-<platform>`: esp32 → `platform-bare-metal`, threadx-linux and threadx-riscv64 → `platform-threadx` |
| `link_kind` (12/12) | 11 `none`, 1 `nuttx-staging` — and BOTH NuttX rows carry it |
| `[board.priority_plan]` (6) | `mps2-an385-freertos` and `mps3-an536-freertos` hold byte-identical blocks (`[1,7]`, transport `[4,4]`, app `[1,3]`). That is `configMAX_PRIORITIES`, a FreeRTOS fact. A board that retunes it still overrides |

The Zephyr row's `resolver = "scripts/lib/priority_plan.py:resolve_zephyr_plan"`
is a file path and a Python symbol living in a board descriptor. It moves with
the block.

**The move has a prerequisite this audit did not see.** Only `config/bare-metal`
and `config/generic` HAVE an `nros-platform.toml`; the other five platforms
carry a `package.xml` and nothing else. "Move it to the platform descriptor"
assumes a descriptor to move it into, and for five of seven platforms there is
none. Creating them is its own wave, and it is where the move belongs.

**Derive, override permitted:**

| Field | Rule | Exceptions today |
| --- | --- | --- |
| ~~`entry_kind` (12)~~ | **WRONG — corrected 2026-09-06.** The rule was stated as "hosted platform → `hosted-main`; zephyr → `zephyr-staticlib`; else `board-run`, no exceptions". Measured while implementing: `freertos-posix` and `mps2-an385-freertos` are BOTH `platform = "freertos"` and their values are `hosted-main` and `board-run`. Whether a port is hosted is a fact no field carries, so the derivation needs a NEW field — not a simplification. It stays authored. | — |
| `local_aliases` (10) | default `[platform_feature]` | 2 real (esp32, threadx-riscv64) |
| `[board.entry] crate_name` (7) | `snake_case(board_crate)` | none — 7 of 7 |
| `[board.entry] signature` (7) | default `#[unsafe(no_mangle)] extern "C" fn main() -> !` | 3 real (esp-hal, cortex-m-rt, plain `fn main`) |

**Decompose.** `cargo_config` (8/12) is a raw TOML blob — a TOML document inside
a TOML document, with no schema check on the inner one. The `rustflags` (8),
`runner` (3) and `linker` (2) counts in this audit came from *inside* it as
text; those primitives do not exist as fields. They should:

```toml
# today
cargo_config = '''
[target.thumbv7m-none-eabi]
runner = "qemu-system-arm -cpu cortex-m3 -machine mps2-an385 -nographic …"
rustflags = ["-C","link-arg=-Tlink.x","-C","link-arg=--gc-sections"]
'''

# after
target    = "thumbv7m-none-eabi"
runner    = "qemu-system-arm -cpu cortex-m3 -machine mps2-an385 -nographic …"
rustflags = ["-C","link-arg=-Tlink.x","-C","link-arg=--gc-sections"]
```

The CLI already composes the leaf `.cargo/config.toml`; it should build it from
those rather than paste a blob through. This is also the field a user is most
likely to get wrong, and the one they get the least help with.

**Drop:**

* `capability_features` (7) — all seven hold the identical value
  `["safety-e2e"]`, and the only read in the entire tree is
  `board_descriptor.rs:1118`, inside
  `fn a_board_advertises_the_safety_capability_feature()`. A field whose sole
  reader is a test asserting that the field is used.
* `[board.entry] comment` (4) — a Rust `//` comment stored as an escaped
  multi-line TOML string. Presentation, not a board fact.
* ~~`crate_path`, `board_features`~~ — **WRONG — corrected 2026-09-06.** Authored
  by no in-tree descriptor, which is what the audit measured, but
  `builder/entry.rs` reads both. They are an out-of-tree extension point with no
  in-tree user, not fields to delete.

**Keep but reclassify:** `target_contains` (2) is not a board property; it is a
disambiguator so two `[[board]]` rows in one file can be told apart by target
substring. Rename it to say so.

**Left alone, flagged:** `toolchain` (10 stable / 2 nightly). Both nightly rows
sit on custom targets needing `build-std`, which suggests a target fact rather
than a board one. Not enough evidence to move it.

Rung 3 of the customization ladder survives intact: `crate_root_extra`,
`crate_root_deps` and `closure_extra` remain the escape hatch, renamed to say
that is what they are. A model that cannot be escaped is worse than none.

### What it costs to add a board, after

A QEMU Zephyr board authors **6 keys**. Today's equivalent authors 19.

```toml
[[board]]
names               = ["qemu-cortex-a53"]
platform            = "zephyr"
supported_netstacks = []

[board.zephyr]
west_board = "qemu_cortex_a53/qemu_cortex_a53/smp"

[board.capabilities]
heap = true
atomics = true
threads = true
```

`platform_feature`, `local_aliases`, `link_kind`, `entry_kind`, `toolchain` and
the priority plan all follow from `platform = "zephyr"`.

A real out-of-tree board on real silicon — the MR-CANHUBK3 this RFC was written
against — authors the same shape plus what genuinely differs: a crate for rung-2
bring-up (`nros_board_init_eth()`), a `[board.cmake] toolchain_file`, a linker
script in `rustflags`, and a `[board.provisioning] gated` naming its own SDK
entry. Field for field, that is the in-tree `s32z270-freertos` descriptor. The
only difference is which root found it — which is the whole claim of D1 and D2.
Today the ROOT part of that already works (`NROS_EXTRA_BOARD_PATH`, and a board
declared inside the consumer's own workspace package); what does not is that a
board is found by a different walk from every other provider, and so can exist
without ever announcing itself.

### Gates this revision asks for

* `check-board-descriptor-single-source` — no `board.cmake` anywhere, no
  `[package.metadata.nros.board]` anywhere.
* The projection round-trip, replacing `phase215_f_manifest_drift.rs`.
* D4's ratchet, in RFC-0087's shape: a stated derivable field must equal its
  derived value, and a moved field must not be restated at board level unless it
  differs.
* `check-provider-announcements` without the `continue` (D5).

Tracked by [phase-375](../roadmap/phase-375-board-tier-policy-and-onboarding-cost.md)
W6–W9; the FVP migration is [phase-215](../roadmap/phase-215-board-crate-as-importable-unit.md)
215.K. The package-shape half lands inside
[phase-420](../roadmap/archived/phase-420-package-identity-and-provider-format.md) W3.
