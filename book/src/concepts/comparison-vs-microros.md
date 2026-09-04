# nano-ros vs micro-ROS

The closest peer project to nano-ros is **micro-ROS**. Both ship a
full embedded ROS 2 framework — client library + RMW + tooling
targeted at MCU-class hardware. This page is an apples-to-apples
comparison.

> **Why not compare to rmw_zenoh or ros2_rust?** Scope mismatch.
> `rmw_zenoh` is a single RMW backend; nano-ros is a full client
> stack. `ros2_rust` (`rclrs`) targets hosted Linux only — it can't
> run on the bare-metal or RTOS targets nano-ros and micro-ROS both
> address. Comparing nano-ros to either would be misleading.

## Side-by-side

| Axis | nano-ros | micro-ROS |
|---|---|---|
| **Project home** | NEWSLab NTU | ROS 2 ecosystem (community + Bosch + eProsima) |
| **First release** | 2024 | 2019 |
| **Primary language** | Rust (with C + C++ bindings) | C (`rclc`) |
| **User-facing APIs** | Rust + C + C++ | C only (`rclc`); experimental C++ in `rclcpp_lite` |
| **RMW backend choice** | Zenoh, XRCE-DDS, Cyclone DDS — pick at compile time | XRCE-DDS only |
| **Network model** | Peer-to-peer (Zenoh / Cyclone DDS) **or** agent-based (XRCE) | Agent-based only |
| **Bridge process required?** | No for Zenoh / Cyclone DDS; yes for XRCE | Yes (Micro-XRCE-DDS Agent) |
| **Supported RTOSes** | FreeRTOS, NuttX, ThreadX, Zephyr, ESP-IDF, PX4 (NuttX), POSIX, bare-metal | FreeRTOS, NuttX, Zephyr, ESP-IDF, POSIX; PX4 is the canonical deployment |
| **`no_std` core** | Yes — entire client stack compiles `no_std` + heapless | No — `rclc` requires libc + a heap |
| **Heap usage** | Optional on bare-metal (XRCE backend is fully static); required for Zenoh / Cyclone DDS | Required (malloc-based DDS-XRCE client) |
| **RT scheduling story** | SchedContext classes (Fifo/Edf/Sporadic live in dispatch; TT deprecated-cooperating) + per-platform kernel capabilities — the generated [Scheduling Wiring Matrix](../reference/sched-matrix.md) is the per-cell truth | rclc executor with priority callbacks; no SchedContext / EDF / TT story |
| **Multi-executor preemption** | `Executor::open_threaded` per-RTOS via `PlatformScheduler` trait | Single executor per process |
| **Multi-backend bridge in one binary** | Yes — `Executor::open_with_rmw` + multi-Node | No (single XRCE session per process) |
| **Discovery** | Zenoh liveliness, RTPS SPDP, XRCE-via-Agent | XRCE-via-Agent |
| **QoS support** | Backend-dependent — per-policy table in [RMW vs upstream §7](../design/rmw-vs-upstream.md) | Subset of XRCE QoS |
| **Formal verification** | 148 Kani harnesses + 83 Verus proofs (CDR, scheduling, RMW glue) | None published |
| **E2E safety protocol** | CRC-32/ISO-HDLC + sequence tracking, EN 50159-mapped (`safety-e2e` feature) | None |
| **ROS 2 distro coverage** | Humble, Iron, Jazzy (`ros-humble`/`ros-iron`/`ros-jazzy` features, RFC-0056) | Humble, Iron, Jazzy |
| **Build system** | Cargo + CMake + platform tools (`west`, `idf.py`, `probe-rs`) plus `just` recipes; C/C++ consume via `add_subdirectory(<repo>)` | colcon + CMake; per-RTOS meta-build (`create`/`configure`/`build`/`flash_firmware.sh`) |
| **Deploy/config model** | Entry packages select board/RMW/deploy shape; Bringup packages own launch topology; platform tools build and flash | `colcon.meta` (hand-tuned static sizing) + `configure_firmware.sh -t <transport>` flags + hand-coded `rclc` app |
| **Host-side broker** | none (Zenoh P2P / Cyclone DDS brokerless); Agent only for XRCE | Micro-XRCE-DDS **Agent always required** |
| **Release model** | Source-only (no crates.io, no precompiled binaries) | Source-only + per-distro Debian packages |
| **Code-size (Cortex-M XRCE talker)** | ~75 KB flash (XRCE), ~100 KB+ (Zenoh) — order-of-magnitude, not benchmarked | ~30–50 KB (XRCE + rclc, as commonly quoted upstream — not measured here) |
| **License** | MIT OR Apache-2.0 (dual) | Apache-2.0 |
| **Governance** | Single-academic-lab maintainership today | ROS 2 community + corporate stewards |
| **Commercial support** | None as of writing | Bosch + eProsima offer services |

## Pick nano-ros when…

- You want **Rust as a first-class API**, not bolted on. Memory
  safety + ownership semantics extend to your application code.
- You want **multi-backend flexibility** — same binary can speak
  Zenoh + DDS, or you want to pick Zenoh's peer-to-peer model over
  XRCE's agent dependency.
- You need **scheduling primitives beyond priority callbacks** —
  EDF / Sporadic / ARINC-653 TT classes, per-callback runtime
  accounting, formal `SchedContext` API.
- You're targeting **safety-aware deployments** that benefit from
  the E2E CRC + sequence tracking and the Kani / Verus harness
  coverage.
- You want a **lean source-only consumption model**:
  `git clone` + `add_subdirectory(<repo>)`. No crates.io drift, no
  pre-built per-distro binaries.

## Pick micro-ROS when…

- You're already in the **micro-ROS ecosystem** (existing rclc
  code, established Agent deployment, ROS 2 Jazzy / Iron support).
- Your toolchain is **C-only** and you don't want to introduce Rust
  into the build pipeline.
- Your target's **flash budget** is tight (~30 KB ceiling) — the
  rclc + XRCE client is smaller than nano-ros + Zenoh.
- You need **commercial support contracts today** (Bosch, eProsima,
  PIWeb).

## Migration sketch

If you're porting from micro-ROS to nano-ros:

Several rows below are now **identity** on the C side: phase-417 took rcl/rclc's
spellings, so `rclc_publisher_init_default` and `rclc_executor_t` are what
nano-ros calls them too. The migration is a rename only where the shape differs.

- `rcl_node_init` → `Executor::create_node` (Rust) /
  `rclc_node_init_default` (C — same name as micro-ROS, but the ARGUMENTS are
  rclc's order `(node, name, namespace, support)`).
- `rclc_executor_t` → `nros::Executor` (Rust/C++); **unchanged** in C.
- `rclc_publisher_init_default` → `Node::create_publisher::<M>` (Rust) or
  the identically-spelled `rclc_publisher_init_default` (C).
- micro-ROS's `rmw_uros_set_custom_transport` → nano-ros's custom
  transport pattern via `nros_platform_*` C ABI (see
  [Custom Transport](../porting/custom-transport.md)).
- XRCE Agent deployment stays the same — point nano-ros's XRCE
  backend at the same Agent.

## See also

- [Build / config / deploy workflow comparison](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/research/build-config-deploy-comparison.md)
  — historical comparison of the three workflow axes vs micro-ROS, Zenoh-pico,
  embedded DDS, and Arduino-ROS.
- [Choosing an RMW Backend](../user-guide/rmw-backends.md) — the
  backend capability matrix.
- [Production Readiness Checklist](../internals/production-readiness.md)
  — pilot-deployment validation.
- [Supported Boards](../reference/supported-boards.md) — per-board
  status.
- [Migration Guide](../start-here/migration-guide.md) — for
  porters coming from rclcpp / rclrs / rclc.
