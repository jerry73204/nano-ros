# ThreadX Platform Assessment for nano-ros

## Context

Eclipse ThreadX (formerly Azure RTOS) is the only open-source RTOS with IEC 61508 SIL 4 and ISO 26262 ASIL D certifications. This document evaluates ThreadX as a nano-ros platform target, focusing on safety certification, verification features, and porting feasibility.

---

## Architecture Overview

ThreadX uses a **picokernel** (non-layered) architecture where kernel services plug directly into the core rather than being stacked. This produces the smallest footprint and fastest context switches of any mainstream RTOS.

| Property | Value |
|----------|-------|
| Min ROM | ~2 KB |
| Min RAM | ~1 KB |
| Priority levels | 32–1024 (configurable) |
| Context switch | Sub-microsecond |
| Scheduling | Priority-based preemptive + preemption-threshold |
| License | MIT (Eclipse Foundation) |
| Deployments | 14+ billion devices |
| Latest release | v6.4.5 (Jan 2026) |

**Preemption-threshold** is unique to ThreadX: a thread can specify a priority ceiling that blocks preemption from threads below that ceiling while still allowing higher-priority preemption. This reduces unnecessary context switches in critical sections and has been the subject of academic scheduling analysis.

### Comparison with Current nano-ros Platforms

| Aspect | ThreadX | FreeRTOS | Zephyr |
|--------|---------|----------|--------|
| Architecture | Picokernel (non-layered) | Microkernel | Monolithic with subsystems |
| Min ROM | ~2 KB | ~5–10 KB | ~8 KB+ |
| Preemption-threshold | Yes (unique) | No | No |
| Networking | NetX Duo (integrated) | lwIP (external) | Built-in full stack |
| Build system | CMake | Make/CMake | West + CMake |
| Safety cert | IEC 61508 SIL 4 | None (SafeRTOS separate) | In progress |
| Rust support | Ferrous Systems demo | zenoh-pico built-in | Active Rust module |

---

## Safety Certifications

### Certified Standards (SGS-TUV Saar)

| Standard | Scope | Level |
|----------|-------|-------|
| IEC 61508-3:2010 | Industrial functional safety | SIL 4 (Route 3S) |
| IEC 62304:2015 | Medical device software | Class C |
| ISO 26262-8:2018 | Automotive functional safety | ASIL D |
| EN 50128:2011 | Railway applications | SW-SIL 4 |
| UL/IEC 60730 | Home appliances | Annex H |
| UL/IEC 60335 | Household appliances | Annex R |

**Security**: Common Criteria EAL4+ (ThreadX, NetX Duo, NetX Secure TLS, NetX MQTT). FIPS 140-2 certified crypto libraries.

### Certified Versions

Only the **6.1.x series** carries current certifications:

| Component | Certified Version | Certificate |
|-----------|------------------|-------------|
| ThreadX Core | 6.1.1 | FS/71/220/23/1532 |
| ThreadX SMP | 6.1.3 | FS/71/220/24/1533 |
| NetX Duo | 6.1.9 | FS/71/220/24/1535 |
| GUIX | 6.1.7 | FS/71/220/24/1534 |
| USBX | 6.1.11 | FS/71/220/24/1536 |

The 6.4.x series certification is planned for **2026** but not yet complete.

### "SIL 4" Context

IEC 61508-3 limits software-only components to Systematic Capability 3 (SC 3). ThreadX's SIL 4 designation means the development process was assessed against SIL 4 rigor via **Route 3S** (statistical testing, clause 7.4.2.12). The final system SIL depends on the complete hardware + software safety architecture. This is the industry-standard approach — the same as how other RTOSes claim their safety levels.

### Certification Artifacts

The safety documentation package includes:

- Safety Manual (system integrator guidance)
- Software Safety Requirements + trace matrices
- V&V Plan, procedures, and test reports (unit + integration)
- Development Plan, QA Plan, Configuration Management Plan
- Design Description + Requirements Specification
- Coverage Analysis (MC/DC for SIL 4)
- MISRA C compliance reports
- Certified source code configuration index

**Availability**: Source code is MIT-licensed on GitHub. Safety artifacts require **ThreadX Alliance membership** (EUR 5K–25K/year depending on company revenue) plus a separate commercial license for the artifacts themselves. The artifacts are under CC-BY-NC-SA (non-commercial), so commercial use requires the paid license.

### MISRA C Compliance

- Targets **MISRA-C:2004** and **MISRA C:2012**
- All mandatory and required rules: compliant
- All but two advisory rules: compliant
- Requires building with `TX_MISRA_ENABLE` defined
- Verified with IAR EWARM 8.11.1 + C-STAT 1.4.4

### RTOSX: The Commercial Fork

The original Express Logic authors (including William Lamie) launched **RTOSX KERNEL** (v7.0.0, January 2025):

- Derived from Eclipse ThreadX, completely refactored with PX5 RTOS technology
- **100% API-compatible** with ThreadX (drop-in replacement)
- **Freshly certified** by SGS-TUV Saar (IEC 61508 SIL 4, ISO 26262 ASIL D)
- Simplified to 2-file source distribution (vs Eclipse ThreadX's 100+ files)
- Comes with full indemnification and professional support
- Includes Pointer/Data Verification (PDV) and central error handling

This means two certified ThreadX-compatible kernels now exist: Eclipse ThreadX (community, MIT, artifacts separately licensed) and RTOSX KERNEL (commercial, from the original authors, current certification).

### Competitor Comparison

| Feature | ThreadX | SafeRTOS | VxWorks CERT | QNX Safety | Zephyr |
|---------|---------|----------|-------------|------------|--------|
| IEC 61508 | SIL 4 (Route 3S) | SIL 3 | — | SIL 3 | Targeting SIL 3 (not yet) |
| ISO 26262 | ASIL D | ASIL D | — | ASIL D | Not yet |
| DO-178C | ED-12B Cert Pack | Support | **DAL A** | — | Not yet |
| IEC 62304 | Class C | Class C | — | Class C | Not yet |
| Open source | Yes (MIT) | No | No | No | Yes (Apache 2.0) |
| Cert body | SGS-TUV Saar | TUV SUD | OEM-specific | TUV Rheinland | — |

ThreadX has the broadest certification portfolio of any open-source RTOS. For avionics (DO-178C DAL A), VxWorks remains unmatched.

---

## Formal Verification and nano-ros Safety

### ThreadX's Verification Approach

ThreadX does **not** use formal verification. Its certification is achieved through:

- 107 pseudo-applications performing functional black-box testing
- TraceX graphical analysis for real-time event tracing
- SGS-TUV Saar test-based certification (Route 3S)
- MISRA C static analysis (IAR C-STAT)

| Aspect | ThreadX | nano-ros |
|--------|---------|----------|
| Bounded model checking | None | **Kani** (115 harnesses) |
| Deductive proofs | None | **Verus** (92 unbounded proofs) |
| Runtime detection | None | **Miri** (UB detection) |
| Static analysis | MISRA C (IAR C-STAT) | Clippy + unsafe auditing |
| Testing | 107 black-box apps | nextest + QEMU E2E |
| Safety certification | IEC 61508 SIL 4 | N/A |

### Complementary Safety Argument

Running nano-ros on ThreadX creates a strong layered safety argument:

```
Layer 4 — Application Verification (nano-ros)
  │  Kani: bounded model checking (buffer overflows, state machines)
  │  Verus: unbounded deductive proofs (ghost types, invariants)
  │  Miri: undefined behavior detection
  │  E2E CRC-32 + sequence tracking (Phase 35)
  │
Layer 3 — Middleware Safety (nano-ros)
  │  Rust memory safety (no null pointers, no data races)
  │  no_std + static allocation (deterministic memory)
  │  Type-safe pub/sub (no message type mismatch)
  │
Layer 2 — RTOS Kernel Safety (ThreadX)
  │  IEC 61508 SIL 4 certified development process
  │  MISRA C compliant source code
  │  Pre-certified as SEooC (Safety Element out of Context)
  │
Layer 1 — Hardware
  │  MCU with lockstep / ECC / watchdog
```

ThreadX provides certified kernel-level guarantees that nano-ros cannot (process-level safety certification). nano-ros provides formal verification guarantees that ThreadX does not (mathematical proofs of correctness). Together, they address different failure modes in the safety argument.

---

## Networking: NetX Duo

NetX Duo is ThreadX's integrated TCP/IP stack. It is certified alongside ThreadX (IEC 61508 SIL 4).

### Capabilities

| Feature | Status |
|---------|--------|
| TCP/UDP/IP | Native (dual-stack IPv4/IPv6) |
| BSD sockets | Compatibility layer (`nxd_bsd.c`) |
| TLS/DTLS | NetX Secure (FIPS 140-2 certified) |
| MQTT | Built-in |
| DHCP/DNS/FTP/HTTP | Built-in |
| TSN | CBS, TAS, FPE, PTP (API defined, driver-dependent) |
| IPsec | Full IP layer security |

### BSD Socket Layer

NetX Duo provides a POSIX-compatible BSD socket API:

- `socket()`, `bind()`, `listen()`, `accept()`, `connect()`, `close()`
- `send()`, `recv()`, `sendto()`, `recvfrom()`
- `select()` (read only — `writefds`/`exceptfds` not supported)
- `setsockopt()`, `getsockopt()`, `getaddrinfo()`

Limitation: `select()` only supports `readfds`. This may require adaptation for zenoh-pico's event loop.

### TSN APIs

NetX Duo includes a `tsn/` module with:

- `nx_shaper_cbs_*` — Credit-Based Shaper (802.1Qav)
- `nx_shaper_tas_*` — Time-Aware Shaper (802.1Qbv)
- `nx_shaper_fpe_*` — Frame Preemption (802.1Qbu)
- `nx_ptp_client_*` — IEEE 1588 PTP (implemented)
- MRP/SRP — Stream Reservation Protocol

The shaper APIs are the most comprehensive of any open-source RTOS but are hardware-driver-dependent. The reference platform is NXP i.MX RT1170/1180.

---

## Hardware Platform Support

### Supported Architectures

| Architecture | Cores |
|-------------|-------|
| ARM Cortex-M | M0, M0+, M3, M4, M7, M23, M33, M55, M85 |
| ARM Cortex-A | A5, A7, A8, A9, A15, A34–A78 (SMP) |
| ARM Cortex-R | R4, R5 |
| ARM TrustZone | ARMv8-M |
| RISC-V | RV32 (RV64 via Microchip port), QEMU port (v6.4.2) |
| Xtensa | ESP32-class |
| Renesas RX | RXv1, RXv2, RXv3 |
| ARC | ARC HS (incl. SMP) |
| x86 | Win32/Linux simulation |

### Safety-Relevant MCUs (Cortex-M class)

| Vendor | MCUs | SDK Integration |
|--------|------|-----------------|
| STMicroelectronics | STM32 (all families) | STM32CubeIDE |
| NXP | i.MX RT, LPC, Kinetis | MCUXpresso |
| Renesas | RA, RX, Synergy | FSP |
| Microchip | SAM, PIC32 | MPLAB |
| Silicon Labs | EFM32, EFR32 | Simplicity Studio |

### Compilers

ARM GNU Toolchain (`arm-none-eabi-gcc`), IAR EWARM, Keil MDK (AC6). Same toolchain as nano-ros's existing bare-metal targets.

---

## Build System

ThreadX uses **CMake** with straightforward cross-compilation:

```bash
cmake -Bbuild -GNinja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/cortex_m4.cmake \
  -DTHREADX_ARCH=cortex_m4 \
  -DTHREADX_TOOLCHAIN=gnu \
  .
cmake --build ./build
```

Integration pattern (CMake subdirectory or git submodule):

```cmake
add_subdirectory(threadx)
target_link_libraries(app PRIVATE azrtos::threadx)
```

Configuration via `tx_user.h` (analogous to `FreeRTOSConfig.h`): controls priority count, stack checking, trace hooks, feature toggles.

The CMake build integrates straightforwardly into zpico-sys `build.rs` using the `cmake` or `cc` crate, following the same pattern as zenoh-pico compilation.

---

## Memory Management

### Two Pool Types

| Feature | Byte Pool (`tx_byte_pool`) | Block Pool (`tx_block_pool`) |
|---------|---------------------------|------------------------------|
| Block size | Variable | Fixed |
| Fragmentation | Possible | Never |
| Allocation time | Non-deterministic | **Deterministic** |
| Safety suitability | General purpose | Safety-critical |

### Static Allocation (No Heap)

ThreadX fully supports static allocation — all kernel objects can use pre-allocated memory:

- Thread stacks: pass a `static mut` buffer to `tx_thread_create()`
- Thread control blocks: statically allocated `TX_THREAD` structs
- Memory pools: user provides the backing array
- No implicit `malloc()`/`free()` in the kernel path

This aligns well with nano-ros's `no_std` patterns and `heapless` containers. The block pool (fixed-size, deterministic, no fragmentation) is particularly compatible with Rust's static allocation model.

---

## Rust Integration

### Existing Work: Ferrous Systems

[`ferrous-systems/threadx-experiments`](https://github.com/ferrous-systems/threadx-experiments) demonstrates Rust + ThreadX:

- **`threadx-sys`**: Bindgen-generated FFI bindings to ThreadX C API
- **`nrf52-app`**: Rust app for nRF52840-DK (Cortex-M4)
- **`qemu-cortex-r5-app`**: Rust app for QEMU Cortex-R5

Build pattern:
1. ThreadX C sources compiled with `arm-none-eabi-gcc` via `build.rs`
2. `bindgen` generates Rust FFI declarations
3. Rust binary links against `libthreadx.a`
4. Rust provides `extern "C" fn tx_application_define()` entry point

This maps directly to nano-ros's `zpico-sys` / `zpico-platform-*` pattern.

---

## Eclipse Foundation Status

### Timeline

| Date | Event |
|------|-------|
| Nov 2023 | Microsoft donates Azure RTOS to Eclipse Foundation |
| Q1 2024 | Available under MIT license |
| Oct 2024 | ThreadX Alliance launched; safety certifications transferred |
| Feb 2025 | v6.4.2 (RISC-V QEMU support) |
| Jan 2026 | v6.4.5 (latest release) |

### ThreadX Alliance Members

AMD, ARM, CypherBridge, Ericsson, Microsoft, NXP, PX5, Renesas, Silicon Labs, STMicroelectronics, Witekio (Avnet).

### Community Assessment

- 23 releases since open-sourcing
- Smaller contributor community than Zephyr or FreeRTOS
- Development pace is vendor-driven (Alliance members)
- Safety artifact commercialization model separates code (MIT) from certification evidence (paid)
- RTOSX fork by original authors suggests some concern about Eclipse Foundation's long-term certification ability

### Licensing

MIT license — fully permissive, no copyleft. **Note**: MIT does not include an explicit patent grant (unlike Apache 2.0). Safety artifacts are CC-BY-NC-SA; commercial use requires separate license.

---

## nano-ros Porting Feasibility

### Architecture Alignment

| Aspect | ThreadX Capability | nano-ros Requirement | Fit |
|--------|-------------------|---------------------|-----|
| Static allocation | Block pools, no mandatory heap | `no_std` + `heapless` | Good |
| Threading | Real tasks + mutexes + event flags | `Z_FEATURE_MULTI_THREAD=1` | Good |
| BSD sockets | NetX Duo wrapper | zenoh-pico TCP/UDP | Good |
| TLS | NetX Secure (FIPS 140-2) | mbedTLS or equivalent | Good |
| CMake build | Native | zpico-sys `build.rs` via `cc`/`cmake` crate | Good |
| ARM toolchain | `arm-none-eabi-gcc` | Same as bare-metal targets | Good |
| RISC-V | RV32 + QEMU port | ESP32-C3 (existing) | Good |
| Safety cert | IEC 61508 SIL 4 | Desired for safety islands | Excellent |

### New Crates Required

```
packages/rmw/zenoh/zpico-platform-threadx/     # FFI symbols backed by ThreadX APIs
packages/boards/nros-<board>-threadx/       # User-facing board crate
```

### zenoh-pico Platform Symbols Mapping

| zenoh-pico Symbol | ThreadX Implementation |
|-------------------|----------------------|
| `z_clock_now()` | `tx_time_get()` (system tick) |
| `z_sleep_ms()` | `tx_thread_sleep()` |
| `z_malloc()` / `z_free()` | `tx_byte_allocate()` / `tx_byte_release()` |
| `_z_task_init()` | `tx_thread_create()` |
| `_z_mutex_init/lock/unlock()` | `tx_mutex_create/get/put()` |
| `_z_condvar_*()` | `tx_event_flags_*()` (ThreadX uses event flags, not condvars) |
| `_z_open_tcp()` | NetX Duo BSD `socket()` + `connect()` |
| `_z_read_tcp()` | NetX Duo BSD `recv()` |
| `_z_send_tcp()` | NetX Duo BSD `send()` |

### Build Integration (zpico-sys)

```rust
// zpico-sys/Cargo.toml
[features]
threadx = []  // alongside posix, zephyr, bare-metal, freertos, nuttx

// zpico-sys/build.rs
// Environment variables:
//   THREADX_DIR - path to ThreadX kernel source
//   THREADX_CONFIG_DIR - path to tx_user.h
//   NETX_DIR - path to NetX Duo source
```

### Feature Flag Chain

```
nros/platform-threadx
  → nros-node/platform-threadx
    → nros-rmw-zenoh/platform-threadx
      → zpico-sys/threadx
```

### Key Challenges

1. **zenoh-pico ThreadX networking**: zenoh-pico has a ThreadX platform in `src/system/threadx/` but only serial transport. TCP/UDP over NetX Duo BSD sockets would need a new network transport layer (~300–500 LOC C)

2. **Condition variables**: ThreadX uses event flags instead of POSIX condvars. zenoh-pico's condvar usage needs mapping to `tx_event_flags_set/get()`. This is solvable but requires careful semantics matching

3. **`select()` limitation**: NetX Duo's BSD `select()` only supports `readfds`. zenoh-pico may need adaptation if it relies on `writefds`

4. **No established QEMU + Ethernet target**: Unlike FreeRTOS (MPS2-AN385 + LAN9118) or bare-metal (MPS2-AN385 + smoltcp), ThreadX has no existing QEMU target with Ethernet networking for nano-ros testing. A QEMU bring-up would be needed

5. **NetX Duo compilation**: NetX Duo is a substantial codebase. Cross-compiling it via `cc` crate in `build.rs` adds build complexity

### Effort Estimate

**Medium-high** — comparable to FreeRTOS Phase 54:

| Component | Estimated LOC | Notes |
|-----------|--------------|-------|
| `zpico-platform-threadx` | ~500–800 Rust | Clock, memory, RNG, threading FFI |
| zenoh-pico NetX transport | ~300–500 C | BSD socket wrapper for TCP/UDP |
| `zpico-sys/build.rs` additions | ~200 Rust | ThreadX + NetX compilation branch |
| Board crate | ~300 Rust | Init, config, `run()` API |
| QEMU bring-up | ~500 C/Rust | ThreadX + NetX on MPS2-AN385 or similar |
| **Total** | ~1800–2300 | |

---

## Strategic Assessment

### Value Proposition

ThreadX's primary value for nano-ros is **safety certification**:

| Platform | Safety Cert | TSN | Market |
|----------|------------|-----|--------|
| Bare-metal (smoltcp) | None | None | Prototyping, education |
| FreeRTOS (Phase 54) | None (SafeRTOS separate) | Via NXP GenAVB | Broadest embedded market |
| Zephyr | In progress (SIL 3 target) | 2 drivers | Industrial IoT, Linux-adjacent |
| NuttX (Phase 55) | None | PTP only | POSIX-compatible embedded |
| **ThreadX** | **IEC 61508 SIL 4, ISO 26262 ASIL D** | **NetX Duo TSN APIs** | **Safety-critical, automotive, medical** |

For customers building safety-critical ROS 2 nodes on automotive safety islands or medical devices, ThreadX + nano-ros provides:

- **Certified RTOS kernel** (IEC 61508 SIL 4) — strongest available
- **Formally verified middleware** (Kani + Verus) — unique to nano-ros
- **Certified networking** (NetX Duo, same SIL 4 cert)
- **Certified TLS** (NetX Secure, FIPS 140-2)
- **TSN-ready networking** (NetX Duo shaper APIs)
- **Open source** (MIT license for code)

No other embedded ROS 2 stack can offer this combination.

### When to Prioritize

ThreadX should be prioritized **after FreeRTOS (Phase 54) and NuttX (Phase 55)** because:

1. FreeRTOS and NuttX cover a larger share of the embedded market
2. zenoh-pico's existing ThreadX support is serial-only
3. No QEMU + Ethernet target exists for ThreadX testing
4. ThreadX's differentiator (safety certification) matters only for the subset of users who need it

However, if a specific customer or use case requires IEC 61508 / ISO 26262 certification, ThreadX should be fast-tracked. The porting effort is well-understood and bounded.

### Implementation

See [Phase 58: ThreadX Platform Support](../roadmap/archived/phase-58-threadx-platform.md) for the full implementation plan. Two validation targets:

1. **Linux simulation port** — ThreadX runs as pthreads on host Linux with NetX Duo real networking (raw socket driver). Zero cross-compilation, fastest iteration.
2. **QEMU RISC-V 64-bit virt** — Official ThreadX QEMU port with virtio-net Ethernet. Real embedded architecture (rv64gc), real interrupt model (PLIC + CLINT).

---

## References

### Eclipse ThreadX
- GitHub: https://github.com/eclipse-threadx/threadx
- NetX Duo: https://github.com/eclipse-threadx/netxduo
- Documentation: https://github.com/eclipse-threadx/rtos-docs
- ThreadX Alliance: https://threadxalliance.org/
- 2025 Roadmap: https://blogs.eclipse.org/post/fr%C3%A9d%C3%A9ric-desbiens/eclipse-threadx-charting-our-course-2025

### Safety Certifications
- ThreadX Alliance Safety: https://threadxalliance.org/subscription/safety-certifications
- RTOSX KERNEL: https://rtosx.com/press/rtosx-kernel-safety-certifiable-alternative-to-eclipse-threadx/
- ThreadX FAQ: https://threadx.io/faq/

### Rust Integration
- Ferrous Systems ThreadX: https://github.com/ferrous-systems/threadx-experiments
- Ferrous Blog: https://ferrous-systems.com/blog/rust-and-threadx/

### Competitor Safety Certifications
- SafeRTOS (WITTENSTEIN): https://www.highintegritysystems.com/safertos/
- VxWorks CERT: https://www.windriver.com/products/vxworks/safety-platforms
- QNX Safety: https://blackberry.qnx.com/en/developers/certifications
- Zephyr Safety: https://docs.zephyrproject.org/latest/safety/safety_overview.html
