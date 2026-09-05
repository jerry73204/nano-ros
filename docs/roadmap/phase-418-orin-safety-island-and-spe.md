# Phase 418 — NVIDIA Orin: what "safety island" actually means, and what nano-ros needs to run there

**Status (2026-09-04). Planning; §2a records a survey of the vendor BSP.** No
work item started. This doc is the study
NVIDIA's Orin safety architecture asked for, plus the gap list between it and the
tree as it stands at `bed98e8ef`. It supersedes nothing; it re-opens the ground
that archived [phase-100](archived/phase-100-orin-spe-infra.md) covered and that
phase-337 W7.b partially deleted.

Implements/informs: RFC-0012 (board/BSP integration), RFC-0064 (board tiers),
RFC-0005/0006 (feature axes). Related: [phase-100 —
Orin SPE infrastructure](archived/phase-100-orin-spe-infra.md),
[phase-100.04 — IVC link design](archived/phase-100-04-link-ivc-design.md),
[phase-377 — CAN link for zenoh-pico](phase-377-can-link-for-zenoh-pico.md).

---

## 0. The correction this phase exists to record

The motivating request was "run nano-ros on the Orin safety island and talk to
ROS 2 on Jetson Linux". Three facts change the shape of that:

1. **The Functional Safety Island (FSI) is not available on this hardware.**
   The development host is a Jetson AGX Orin Developer Kit (`p3737-0000+p3701-0005`,
   `tegra234`, L4T R36.4.4 / JetPack 6.2). NVIDIA states plainly that "Jetson AGX
   Orin commercial modules are not enabled with functional safety elements.
   Functional Safety will be offered on NVIDIA IGX platform." The FSI IP is on the
   die — it is not enabled for Jetson SKUs. Any plan that puts nano-ros on "the
   safety island of this box" is unimplementable as stated.

2. **Even on DRIVE AGX Orin, the FSI is not a place customers deploy code.**
   Per the DRIVE OS SDK docs, the FSI is 4× dual-core-lockstep Cortex-R52
   (ARMv8-R), each with 256 KB ATCM / 128 KB BTCM / 128 KB CTCM plus 32 KB I/D
   caches, 3 MB shared SRAM, its own XTAL and voltage rail, 2× CAN and 1× SPI
   (the SPI is reserved for the external safety MCU under DRIVE OS Safety
   Services). MB2 loads NVIDIA-signed FSI binaries. The published integration
   surface is *messaging*, not hosting: `NvFsiCom` — a CCPLEX-side library
   (`libNvFsiCom.so`) plus an auto-launched `NvFsiCom` daemon, moving raw bytes
   over an FSI CPU NS DRAM carveout with mailbox notification, against a
   `cddCcplexCom` AUTOSAR CDD on the FSI side. Third-party application code on
   FSI is not a documented DRIVE OS feature.

3. **The Orin part that *does* accept our code, on this exact board, is the SPE.**
   The Sensor Processing Engine (a.k.a. the AON cluster) is a Cortex-R5 in the
   always-on power domain with 256 KB of TCM-attached SRAM at up to 200 MHz.
   NVIDIA ships a FreeRTOS V10.4.3-based FSP for it (`spe-freertos-bsp` /
   `rt-aux-cpu-demo-fsp`), documented for R36.x, and the AGX Orin feature list
   includes **IVC** alongside GPIO, I2C, SPI, CAN, UART, timers, GTE and AODMIC.
   IVC is a shared-memory channel signalled by HSP doorbells; channels are
   declared in the platform DTSI (`tegra234-p3737-0000+p3701-xxxx-nv-common.dtsi`
   for this board) and the shipped demo exposes exactly one, `aon_echo`, through
   `/sys/devices/platform/bus@0/bus@0:aon_echo/data_channel`.

So the deliverable is: **nano-ros on the SPE R5 under NVIDIA's FreeRTOS FSP,
speaking zenoh over an IVC link to a CCPLEX-side bridge, with `rmw_zenoh_cpp`
ROS 2 nodes on Jetson Linux on the other side of that bridge.** "Safety island"
in the ISO 26262 sense stays out of reach on Jetson silicon; the SPE is a
freestanding always-on core that boots before Linux and survives a Linux crash,
which is the property the sentinel workload actually wants.

A second, separate target — DRIVE AGX Orin — is discussed in §5 and is not in
scope for the work items here.

## 1. What the tree already has

Phase 100 (archived 2026-05-18, "Done, all 10 sub-items landed") built most of
the plumbing. Phase 337 W7.b then deleted the two crates that made it an actual
target, because they were scaffolds with zero matrix cells. What survives at
`bed98e8ef`:

| Piece | Location | State |
|---|---|---|
| Tegra IVC driver, `fsp` + `unix-mock` + inert stub backends | `packages/drivers/ipc/nvidia-ivc/` | Present, **`workspace.exclude`d** |
| `PlatformIvc` trait | `packages/platform/nros-platform-api/src/lib.rs:971` | Present |
| zenoh-pico IVC link forwarders (`_z_open_ivc`, `_z_ivc_*`) | `packages/rmw/zenoh/zpico-link-ivc/` | Present |
| `link-ivc` feature → `Z_FEATURE_LINK_IVC` | `zpico-sys/Cargo.toml:90`, `nros-zpico-build/src/lib.rs:346` | Present |
| Vendored IVC link C (`src/link/unicast/ivc.c`, 14 KB) | zenoh-pico fork `nano-ros` branch, at pin `fa7ad0f5` | Present |
| Link policy that models "SPE has no Ethernet" | `nros-board-common/src/policy.rs` (`LinkFeatures::ivc`, `LinkPolicy`) | Present |
| Wire-format E2E over the mock backend | `packages/testing/nros-tests/tests/nvidia_ivc_mock_wire_format.rs` | Present, no `required-features`, runs in the unit lane |
| `armv7r-none-eabihf` target | `rust-toolchain.toml`, `nros-sdk-index.toml:1133` (`[rust.target.armv7r-hf]`) | Present |

The wire format that test pins, and that any CCPLEX-side bridge must match
byte-for-byte: 64-byte fixed frames, a 4-byte little-endian header per frame
(`u16 total_len`, `u16 offset`), ≤60 payload bytes per frame, SPSC FIFO with no
reordering, and `total_len == 0 && offset == 0` reserved as a silently-dropped
keep-alive.

## 2. What is missing

Deleted or never built:

- **`nros-board-orin-spe`** — deleted in `53a3402d2` (phase-337 W7.b) along with
  its `nros-board.toml`, `build.rs`, `printf_shim.c` and config. This was the
  crate that bound `NV_SPE_FSP_DIR`, selected the `ARM_CR5` FreeRTOS port and
  exposed `tegra_ivc_channel_*`.
- **`nros-platform-orin-spe`** — folded into the board by phase 121.10
  (`364fe527a`), then removed with it. Its `ivc.rs` and `random.rs` are gone.
- **`cortex-r5` arch profile** — `packages/platform/nros-platform-freertos/nros-platform.toml`
  declares `cortex-m3`, `cortex-m7` and `cortex-r52` only. ARMv7-R (`-mcpu=cortex-r5`,
  and the softfp-vs-hardfp question NVIDIA's BSP forces) has no block. This is
  the same omission phase-385 W1 found for `cortex-r52`.
- **SDK provisioning for the SPE FSP** — no `just` recipe fetches or builds it,
  no `nros-sdk-index.toml` entry describes it. Phase 100.7's `bsp-download` /
  `bsp-build` / `bsp-clean` recipes are not in `just/`. RFC-0014's "provision or
  explain" rule applies — and per §2a this is a provision, not an explain: the
  BSP is a public BSD-3 download, not the EULA-gated artifact this tree has been
  describing.
- **The CCPLEX side.** Nothing in this tree talks IVC from Linux. `nvidia-ivc`'s
  `unix-mock` is a socketpair, not a Tegra channel; the real Linux side needs a
  device-tree channel plus a kernel-side owner, because the only stock userspace
  entry point is `aon_echo`'s `data_channel` sysfs node. The bridge daemon that
  phase 100 assumed lives in `autoware_sentinel/src/ivc-bridge/`, out of tree.
- **Registry, matrix and fixture presence.** No `board-support.toml` row, no
  `PlatformId` variant, no `examples/orin-spe/`, no `fixtures.toml` row, no
  `matrix::CELLS` or `interop::CELLS` entry. Per RFC-0064 the board is therefore
  not even tier 3 — it does not exist.
- **A `nvidia-ivc` CI lane.** The crate is workspace-excluded, so no `just check`
  lane compiles it. The `unix-mock` path is exercised only indirectly, via the
  `nros-tests` dependency at `packages/testing/nros-tests/Cargo.toml:77`; the
  `fsp` and default-stub configurations are compiled by nothing.

Unresolved technical questions, in the order they will bite:

1. **Does the image fit?** 256 KB of TCM SRAM, against phase-100's own estimate of
   a ~30 KB heap budget. `just mem-report` on a current FreeRTOS zenoh image is
   the measurement to take *before* any board crate is written; issue 0810 and
   0900 (executor arena sized by worst-case shape) are the relevant cost drivers.
   If the SPE can also execute from a DRAM carveout on Orin, that changes the
   budget entirely and must be established from the FSP, not assumed.
2. **Throughput.** 60 payload bytes per frame against a zenoh batch size chosen
   for MTU-sized links. The IVC link's batching is `__z_ivc_send_batch` /
   `__z_ivc_recv_batch`; nobody has measured it against a real HSP doorbell.
3. **Float ABI.** NVIDIA's BSP is softfp; `armv7r-none-eabihf` is not. This is a
   link-time question, and phase-100 flagged it without settling it.
4. **Whether the fork's `ivc.c` still builds against zenoh-pico 1.8.0.**
   [phase-415](archived/phase-415-zenoh-pico-1-8-0-patch-line.md) is bumping the patch
   line; `ivc.c` is a carried patch, and the pin (`fa7ad0f5`) is behind the
   `nano-ros` branch tip (`0101b80d`). Nothing currently compiles that file, so
   a rebase can break it silently — exactly the class issue 1014 just recorded
   for the Cyclone sertype TU.

## 2a. The vendor BSP, surveyed (2026-09-04, L4T 36.4.4)

Downloaded and read after §2 was written. Four things it settles, three of which
contradict what this tree currently says:

- **Where it lives.** No standalone download exists. The BSP is a 2.6 MB tarball
  nested inside the 216 MB Jetson Linux public sources archive at
  `Linux_for_Tegra/source/spe-freertos-bsp.tbz2`, shipped with its own
  `.sha1sum` (`6b5b29c7…`). It holds `FreeRTOSV10.4.3/`, `fsp/` (drivers plus an
  OS-abstraction layer with FreeRTOS/LittleKernel/SafeRTOS backends) and
  `rt-aux-cpu-demo-fsp/`.
- **It is not licence-gated.** `nvidia-ivc`'s README and manifest call the FSP
  "closed-source", "ships under SDK Manager EULA", and say "anyone without an
  Orin DevKit account cannot build the `fsp` backend". None of that is true: 117
  files carry the BSD 3-Clause notice, the bundled FreeRTOS is MIT, there is no
  EULA in the tree, and the download needs no account. **The `fsp` backend can
  therefore be built in CI from a scripted fetch** — which changes 418.2 and
  418.4 from "explain the gate" to "provision it".
- **It builds no static library.** `make bin_t23x` compiles objects straight
  into `out/t23x/spe.elf`; there is no `tegra_aon_fsp.a` anywhere. The `fsp`
  backend's link model needs rethinking (418.6): either the board build compiles
  the FSP sources, or provisioning archives them.
- **The float ABI is `softfp`.** The BSP compiles with
  `-mcpu=cortex-r5 -mthumb-interwork -mfloat-abi=softfp -mfpu=vfpv3-d16` and
  links `-nostartfiles -e_stext --gc-sections -lc`. `armv7r-none-eabihf` — the
  triple `rust-toolchain.toml` and `nros-sdk-index.toml` carry for this target —
  is hard-float ABI and incompatible. 418.3 must pick `armv7r-none-eabi` plus
  the FPU as a target feature.

Two more numbers 418.1 was going to have to establish anyway:

- **256 KB, all-in.** The link script has one region:
  `btcm : ORIGIN = 0x0c480000, LENGTH = 0x40000`. Text, rodata, data, bss, heap
  and all five ARM mode stacks share it. The image is loaded at `0x70000000` and
  relocated into BTCM at boot. Whether an AST window onto a DRAM carveout can
  host code or heap is open, and it decides whether a middleware fits at all.
- **The IVC API matches.** `tegra_ivc_{channel_notified,channel_is_synchronized,
  rx_get_read_available,rx_get_read_frame,rx_notify_buffers_consumed,
  tx_get_write_space,tx_get_write_buffer,tx_send_buffers}` in the FSP's
  `include/tegra-ivc.h` is exactly what `nvidia-ivc/src/fsp.rs` declares. Phase
  100 read the ABI correctly from documentation. The shipped channel is one
  CCPLEX channel, 16 frames × 64 bytes each way, carveout base `0x80000000`,
  HSP `top1` doorbell.

Also worth recording: the FSP ships **no CAN driver**, though the SPE feature
list names CAN.

The downstream consumer already has a working fetch-verify-stage script for all
of this — `autoware-safety-island/scripts/provision-orin-spe-bsp.sh` — and its
[phase-10](https://github.com/NEWSLabNTU/autoware-safety-island/blob/main/docs/roadmap/phase-10-orin-spe-entry.md)
is the application-side plan. 418.4 should follow its shape rather than invent a
second one.

## 3. Work items

- [ ] **418.1 — Measure before building.** `just mem-report` a FreeRTOS +
      zenoh-pico image at the smallest viable entity count, and record the number
      against the SPE's 256 KB TCM. Establish from the R36.x FSP whether SPE code
      and data can live in a DRAM carveout. **Acceptance:** a table in this doc
      giving image size, static RAM, arena size and the resulting verdict
      (fits / needs a carveout / does not fit), with the command that produced it.

- [ ] **418.2 — Compile what already exists.** Put `nvidia-ivc` on a `check` lane
      in all three configurations (default stub, `unix-mock`, and `fsp` when
      `NV_SPE_FSP_DIR` is set), and compile the fork's `ivc.c` by enabling
      `link-ivc` in that lane. **Acceptance:** `just check` fails if either the
      driver or the vendored IVC link stops compiling. This is the cheapest item
      and it is the one that stops phase-415 from silently breaking §2 Q4.

- [ ] **418.3 — `cortex-r5` arch profile.** Add the `[arch.cortex-r5]` block to
      `nros-platform-freertos/nros-platform.toml`, settling the float ABI in the
      profile rather than per board. **Acceptance:** the profile resolves cflags
      for `armv7r-none-eabihf` the way `cortex-r52` does for ARMv8-R, and a
      `check` gate covers it as phase-385 W1's fix did.

- [ ] **418.4 — SPE FSP provisioning.** An `nros-sdk-index.toml` entry plus a
      `just` verb that fetches the BSP — a plain scripted download, per §2a, not
      an SDK-Manager gate — verifies both digests and stages it. Never `sudo`.
      Follow the ASI script rather than writing a second one. **Acceptance:**
      `just check tier-preconditions` reports a missing FSP as a named unmet
      precondition instead of a build failure four frames deep, and the `fsp`
      backend compiles in a lane.

- [ ] **418.5 — CCPLEX-side IVC channel.** Add a non-`aon_echo` IVC channel to
      the AGX Orin DTSI, and decide the Linux-side owner: a small out-of-tree
      kernel module exposing a char device, or reuse of the echo channel's
      driver. Document the kernel rebuild and flash steps. **Acceptance:** a
      documented loopback between a userspace process on Jetson Linux and the
      stock `ivc-echo-task` on the SPE, on this board, with the frame format
      matching `nvidia_ivc_mock_wire_format.rs`.

- [ ] **418.6 — Bring back `nros-board-orin-spe`.** Recreate the board crate over
      the canonical platform C ABI (not as a platform crate — phase 121.9 settled
      that), binding `NV_SPE_FSP_DIR`, the `ARM_CR5` port and the FSP's
      `tegra_ivc_channel_*`. Restore it from `53a3402d2` rather than rewriting.
      **Acceptance:** it builds for `armv7r-none-eabihf` with the FSP present, it
      has a `board-support.toml` row, and per RFC-0064 that row is tier 3 with a
      named maintainer until 418.7 lands.

- [ ] **418.7 — End-to-end on hardware.** A talker on the SPE reaching a
      `ros2 topic echo` on Jetson Linux through the CCPLEX bridge and
      `rmw_zenoh_cpp`. **Acceptance:** an `interop::CELLS` row (there is no
      fixture row — the SPE side is flashed, not launched), and a promotion of the
      registry row once a lane can assert it.

- [ ] **418.8 — Bridge daemon: decide where it lives.** Either vendor
      `autoware_sentinel`'s `src/ivc-bridge/` into this tree or declare it a
      downstream consumer and pin the wire format as the contract. **Acceptance:**
      a one-paragraph decision in this doc plus, if in-tree, a crate that shares
      `nvidia-ivc`'s API on both sides of the wire.

## 4. What this is *not*

- Not functional safety. Nothing here makes nano-ros ASIL-anything, and the SPE
  on a Jetson devkit carries no safety claim. The FSI's properties — lockstep,
  independent rail and clock, hardware safety manager — are not inherited by the
  SPE.
- Not a new RMW backend. IVC is a *link*, one peer of TCP/UDP/serial/raweth/CAN,
  and `LinkFeatures` already treats it that way.
- Not a Zephyr or NuttX port of the SPE. NVIDIA's FSP is FreeRTOS; using it is
  what buys the vendor drivers and the IVC implementation.

## 5. DRIVE AGX Orin — the other target, for later

If the goal is a real safety island rather than an always-on R5, the DRIVE
platform's shape is different enough to be its own phase:

- Customer code does not run on FSI. The realistic homes are **a guest VM under
  the DRIVE OS hypervisor** (QNX or Linux), or **an external safety MCU** on the
  FSI's dedicated SPI under Safety Services.
- Guest-to-guest transport is **NvSciIpc**, whose `INTER_VM` backend is an IVC
  queue declared in the PCT (`platform_config.h`): frame size in multiples of 64
  bytes, at most 512 queue entries, and a channel whose queue ID is absent from
  the PCT is silently ignored. A channel declared in the NvSciIpc cfg but not in
  the PCT does not error — it disappears.
- That 64-byte frame granularity is the same shape `nvidia-ivc` already speaks,
  which is the one piece of leverage this tree has toward DRIVE OS.
- CCPLEX↔FSI messaging, if it is ever needed, is `NvFsiCom` — carveout plus
  mailbox, daemon plus `libNvFsiCom.so` — not a socket API.

## References

- [Functional Safety Island (FSI) — DRIVE OS 7.0.3 Linux SDK](https://developer.nvidia.com/docs/drive/drive-os/7.0.3/public/drive-os-linux-sdk/embedded-software-components/Functional_Safety_Island_FSI/Functional_Safety_Island.html)
- [FSI-CCPLEX Communication — DRIVE OS 7.0.3](https://developer.nvidia.com/docs/drive/drive-os/7.0.3/public/drive-os-linux-sdk/core-concepts/FSI-CCPLEX_communication.html)
- [Adding a New INTER_VM Channel — DRIVE OS](https://developer.nvidia.com/docs/drive/drive-os/7.0.3/public/drive-os-linux-sdk/platform-customization/AddingaNewINTER_VMChannel28_3.html)
- [Jetson Sensor Processing Engine (SPE) Developer Guide — R36.4](https://docs.nvidia.com/jetson/archives/r36.4/spe/index.html)
- [SPE IVC — R36.4.3](https://docs.nvidia.com/jetson/archives/r36.4.3/spe/md__home_jenkins_workspace_Utilities_rt_aux_cpu_demo_fsp_docs_work_rt_aux_cpu_demo_fsp_doc_ivc.html)
- [Cortex-R52 and Cortex-R5 cores in Jetson AGX Orin — NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/cortex-r52-and-cortex-r5-cores-in-jetson-agx-orin/239914)
