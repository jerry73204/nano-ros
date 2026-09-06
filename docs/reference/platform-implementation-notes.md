# Platform & RMW implementation notes

Implementation-level detail relocated out of `CLAUDE.md` so the auto-loaded instruction file
stays a thin router. This is **reference** material (how the current impl behaves), not design
rationale — for the *why*, see the RFCs under [docs/design/](../design/). Pitfall one-liners
that agents need hot stay indexed in `CLAUDE.md`; the detail is here.

## Build-profile constraints (phase-336)

Two platforms cannot use whatever profile the build resolved. Both are named
constants in `packages/tooling/nros-cargo-profile` (`NUTTX_RUST_PROFILE`,
`FREERTOS_QEMU_PROFILE`, reachable from shell as `nros profile carve-out
<name>`) so the builder and every fixture resolver read one value — when they
were separate literals, a test looked for a binary in a directory the builder
never wrote to (#156).

- **NuttX Rust images require `lto = "fat"`.** At `lto = "off"` a
  non-deterministic `armv7a-nuttx-eabihf` cross-CGU miscompile corrupts the std
  `lang_start` main-closure fat pointer: the image reboots before `main` with no
  console output. Never root-caused (phase-177.8.c; phase-285 W5 rode the same
  dodge for nuttx-riscv).
- **FreeRTOS QEMU (Cortex-M3) images require real optimization.** Not a
  miscompile — a timing floor. `qemu-system-arm` emulating an M3 is slow enough
  that a lightly-optimized zenoh-pico misses its session handshake window, and
  the image boots but never connects. Issue #126 is the C-side face of the same
  constraint (fixture rows pin `CMAKE_BUILD_TYPE=Release`).

Both resolve to `nros-minsizerel`, which carries the settings `[profile.release]`
had before phase-336 split size from speed — so these images are unchanged by
that phase.

## Spin / yield

`zpico_spin_once` event-driven wake:

- POSIX/Zephyr: `_z_condvar_wait_until` on `g_spin_cv`.
- FreeRTOS: `xSemaphoreTake(g_spin_sem, …)`.
- NuttX: `sem_timedwait(&g_spin_sem_posix, …)` (pthread condvar hangs — archived Phase 55.12).
- Bare-metal: single-thread `zp_read` loop.

Cooperative yield (`PlatformYield`): POSIX/NuttX `sched_yield()`, Zephyr `k_yield()`, FreeRTOS
`vPortYield()`, ThreadX `tx_thread_relinquish()`, bare-metal `core::hint::spin_loop()` default,
opt-in `cortex_m::asm::wfi()` via `BoardIdle`. RTOS yields are not ISR-safe; `spin_loop()` is.

**Multi-threaded `zpico_spin_once` pitfall (critical):** on multi-threaded platforms
(Zephyr, POSIX) use `z_sleep_ms()`, not `select()`. `select()` returns immediately on any zenoh
protocol traffic (keep-alives, interest msgs), burning a `Promise::wait()` budget of 500
`spin_once(10)` iterations in ~39 ms instead of 5000 ms. FreeRTOS uses `vTaskDelay()`, smoltcp a
clock loop, single-threaded `select()+zp_read()` — all correct. Zephyr native_sim also needs
`CONFIG_NATIVE_SIM_SLOWDOWN_TO_REAL_TIME=y` for service/action clients.

## smoltcp multicast (bare-metal)

- `Interface::join_multicast_group(addr)` needs a multicast addr; smoltcp 0.12 returns
  `Unaddressable` for `0.0.0.0`. Pass the GROUP (`239.255.0.1`).
- `set_recv_timeout(_, 0)` in `define_smoltcp_platform!` = non-blocking poll.
- LAN9118 emulator filter rejects multicast unless `MAC_CR.MCPAS`; promiscuous (`PRMS`)
  recommended for QEMU `-nic socket,…`.
- `MAX_UDP_SOCKETS` default 4. RTPS needs 3/participant; zenoh/xrce 0..=1.

## NetX Duo BSD (ThreadX)

- `SO_RCVTIMEO` takes `struct nx_bsd_timeval *`, NOT `INT` ms. Wrong type →
  `wait_option = NX_WAIT_FOREVER` → deadlock. Use `nros-platform-threadx::set_recv_timeout_ms`.
- `fcntl(F_SETFL, O_NONBLOCK)` works (toggles `NX_BSD_SOCKET_ENABLE_OPTION_NON_BLOCKING`).
- NSOS-NetX shim translates `SO_RCVTIMEO` for threadx-linux. Accepts INT-ms and `nx_bsd_timeval`.

## Board transport features

`ethernet` (default MPS2-AN385/STM32F4/ESP32-QEMU) or `wifi` (ESP32) → TCP/UDP via
`zpico-smoltcp`; `serial` → UART via `zpico-serial` (bare-metal) or zenoh-pico built-in (ESP32,
Zephyr). `Config` fields are `#[cfg(feature)]`-gated. ≥1 transport required (`compile_error!`).
Transports coexist (locator selects). ESP32/ESP32-QEMU use zenoh-pico's serial (no `zpico-serial`).

## Parameter services

`param-services` feature in `nros-node` → `~/get_parameters`, `~/set_parameters`, etc. Uses
`nros-rcl-interfaces`. Handlers return `Box<Response>`.

## XRCE embedded build

`nros-rmw-xrce-cffi` (C FFI shim) gates `UCLIENT_PROFILE_{UDP,TCP,SERIAL}` +
`UCLIENT_PLATFORM_POSIX` + `transport_posix_{udp,serial}.c` on `target_os = linux|macos|*bsd`.
Bare-metal (`target_os = "none"`) gets only `UCLIENT_PROFILE_{DISCOVERY,CUSTOM_TRANSPORT,
STREAM_FRAMING}` and must inject its own custom transport. `just check workspace-embedded`
excludes `nros-rmw-xrce{,-cffi,-cffi-staticlib}` (header-only backend's `internal.h` references
UDP types unconditionally; the staticlib sibling needs panic_handler resolution at compile time).
The `-staticlib` sibling lets Corrosion import a real `staticlib` target without forcing the cffi
rlib to emit one.

**XRCE-DDS RMW bugs (critical):** `uxr_buffer_request_data` must be flushed with
`uxr_run_session_time` immediately after the call — unflushed request_data in the reliable output
stream causes intermittent timeouts when a later service request is buffered (pre-301 `call_raw`; fixed in all
three `create_*` methods). Reliable streams need `STREAM_HISTORY >= 2` (we use 4); history=1 fails
to recycle the single slot. The service client (requester) needs `uxr_buffer_request_data` to
receive replies, same as subscribers and repliers.

## Probe-only opaque sizes

`EXECUTOR_OPAQUE_U64S` etc. derive from `nros::sizes::EXECUTOR_SIZE` via the `nros_sizes_build`
rlib probe — no hand-math upper bound. Per-consumer `const _: () = assert!(size_of::<Ty>() <=
STORAGE_SIZE …)` enforces compile-time correctness. Probe=0 only on `cargo check
--no-default-features` (warns + 1-word placeholder; the resulting rlib must not be linked).
`CppContext` adds explicit `CPP_CONTEXT_OVERHEAD = 8` (u32 domain_id + alignment padding) on top
of `Executor`.

## Wrapper timing

`Future::wait()`, `Stream::wait_next()`, `Executor::spin(duration_ms)` budget by wall-clock via
`nros_cpp_time_ns()`. Iteration-count loops collapse on early-wake from signaled condvars
(keep-alives, discovery gossip).

## cbindgen output as canonical FFI

nros-cpp `*.hpp` headers `#include "nros_cpp_ffi.h"` directly; per-file hand-written
`extern "C"` redeclaration blocks were removed (drift broke things once). `qos.hpp` keeps a
fallback redef under `#ifndef NROS_CPP_FFI_H`. Exceptions: `parameter.hpp` cross-references
nros-c's `<nros/parameter.h>`; `action_{client,server}.hpp` `reinterpret_cast` `goal_id` at FFI
callsites (cbindgen renders `*const [u8; 16]` as ptr-to-array); `set_callbacks` excluded from
cbindgen via `[export.exclude]` and declared locally with plain fn-ptr typedefs. cbindgen variants
are prefixed with the enum name (`prefix_with_name = true`) to avoid C++ name collisions.

## QEMU networked tests

- Slirp networking (no TAP/sudo/bridges).
- Per-platform zenohd ports in `nros_tests::platform`: baremetal=7450, freertos=7451,
  nuttx=7452, threadx-riscv=7453, esp32=7454, threadx-linux=7455, zephyr=7456.
  `ZenohRouter::start(platform::FREERTOS.zenohd_port)`. Bridge-net (threadx-linux veth):
  `ZenohRouter::start_on("0.0.0.0", port)`.
- Subscriber first, then publisher. 5–10 s stabilization. Per-platform nextest groups
  (`max-threads = 1`); platforms run in parallel.
- Domain ID: compile-time on embedded (Zephyr Kconfig `CONFIG_NROS_DOMAIN_ID`; others via each
  example's `config.toml` `domain_id` → generated `app_config.h`), runtime env on native via
  `nros_tests::unique_ros_domain_id()`. For Cyclone (RTPS ports = `7400 + 250*domain`), parallel
  fixtures bake a distinct domain per communicating role-set.
- Patched `qemu-system-arm` (Phase 143): use `nros_tests::qemu::qemu_system_arm_cmd()`, never
  `Command::new("qemu-system-arm")`. New justfile recipes gate through the `QEMU_BIN` path_exists
  check. See `book/src/internals/qemu-patched-binary.md`.
- QEMU clock: `-icount shift=auto` (sleep=on) makes virtual time track wall-clock during WFI;
  full detail in [qemu-icount.md](qemu-icount.md).

## FreeRTOS pitfalls

- Stack overflow → "Invalid mbox". **The three numbers this entry used to name are all gone —
  read it as a symptom, not as a recipe** (corrected phase-392 W6, 2026-09-06):
  - `Executor` has NOT held an inline `arena: [MaybeUninit<u8>; ARENA_SIZE]` since phase-271
    (issue 0110). It holds `arena: &'s mut [MaybeUninit<u8>]`, a slice borrowed from
    caller-supplied backing, and on FreeRTOS that backing is a named `.bss` static
    (`nros_node::executor::backing::EXECUTOR_BACKING`, phase-392 W6) — not the task stack.
  - `APP_TASK_STACK` was deleted in phase-76. The live knob is `app_stack_bytes`
    (`nros_board_common::freertos_config::DEFAULT_APP_STACK_BYTES`, **393216** = 384 KiB),
    overridable with `NROS_FREERTOS_APP_STACK_KB`. No file in the tree sets that override.
    The C/C++ carrier's own mirror in `cmake/templates/freertos_app_config.c.in` is 524288.
  - No example pins `NROS_EXECUTOR_ARENA_SIZE=8192`; that was pre-phase-271 prose.

  What is still true is the SYMPTOM: the app task's frame is dominated by the zenoh-pico
  session open and the `Executor` value `open_in` builds and returns by value, so an
  undersized app task dies as "Invalid mbox" rather than as a stack fault. Size it by
  measuring, not by copying a number out of a document.
- Deterministic `rand()` starts from seed 1 → duplicate Zenoh session IDs across QEMU instances;
  `srand()` with an IP-based unique seed in `nros_freertos_init_network()`.
- Manual-polling action server: `create_action_server()` is not arena-registered, so `spin_once()`
  does not process get_result queries — call `server.try_handle_get_result()` after
  `complete_goal()`.
- Poll task priority must be ≥ 4 (same as zenoh-pico read/lease tasks) to drain the RX FIFO.
- Debug guide: [freertos-lan9118-debugging.md](../guides/freertos-lan9118-debugging.md).

## Zephyr POSIX resource limits

Defaults `CONFIG_MAX_PTHREAD_MUTEX_COUNT=5` / `CONFIG_MAX_PTHREAD_COND_COUNT=5` are too low;
zenoh-pico needs ~8+ mutexes (transport TX/RX/peer + a write-filter mutex per publisher under
`Z_FEATURE_INTEREST=1`). Exhaustion makes `pthread_mutex_init` fail → zenoh-pico returns -80.
Set `CONFIG_MAX_PTHREAD_MUTEX_COUNT=32` and `CONFIG_MAX_PTHREAD_COND_COUNT=16` in `prj.conf`.

**These pools are per-OBJECT, so anything that allocates a mutex per entity turns them into a
cap on workload size, not just a startup constant.** Zephyr's `pthread_mutex_t` and
`pthread_cond_t` are handles into `static` arrays sized by those knobs; a library that puts a
mutex in every object scales its demand with the number of objects. CycloneDDS does exactly
that (three per writer: `e.lock`, `qos_lock`, `rdary_lock`, plus one per addrset), so a
native_sim image joining a ~40-participant Autoware graph exhausted 16384 slots ~19 s in and
needed 131072 (~4.1 MiB static) — issue 0371 was that crash, 0496 the sizing rule.

## Cyclone on Zephyr uses a NATIVE ddsrt sync backend, not the POSIX one (issue 0496)

Since `cyclonedds@a09babf3`, `ddsrt_mutex_t` is an embedded `struct k_mutex` and `ddsrt_cond_t`
an embedded `struct k_condvar`, so cyclone takes NO pthread pool slots for either. The knob is
back to the 256 example default and no longer scales with graph size. Wiring: `DDSRT_WITH_ZEPHYR`
in `zephyr/cyclonedds-config/dds/config.h` picks the types
(`ddsrt/sync/zephyr.h`), and `zephyr/cmake/nros_rmw_cyclonedds.cmake` swaps
`sync/posix/sync.c` for `sync/zephyr/sync.c`. **Both halves must move together** — the types and
the implementation — or the struct layouts disagree.

Two consequences worth knowing:

- **`k_mutex` is RECURSIVE for its owner; a pthread NORMAL mutex deadlocks.** Correct code cannot
  tell the difference, but code that re-acquires its own lock hangs on POSIX and silently
  succeeds on Zephyr. So a locking bug can reproduce natively and NOT on Zephyr — which is what
  happened with the striped addrset locks, whose nesting hazard only ever hung the native build.
  When a lock-shaped bug is platform-asymmetric in that direction, suspect this.
- `ddsrt_rwlock_t` and `ddsrt_once_t` are still pthreads (one rwlock exists in all of cyclone —
  the `log.c` sink — and `pthread_once_t` is caller-owned), so the pools are used but not scaled.

## Zephyr zsock per-fd serialization vs zenoh-pico (issues 0129/0139)

Zephyr's socket layer takes a per-fd `fdtable` mutex for the ENTIRE blocking call —
NSOS offload included — so zenoh-pico's blocking read task holds the session socket
for a full `SO_RCVTIMEO` window between inbound packets, and every tx (entity or
liveliness declare, lease keepalive, publish, query reply) queues behind it.
Consequences and rules:

- **Total image tx ≈ sends-per-second on the shared socket, NOT the sum of
  publish rates** (issue #145 / phase-279). Measured on native_sim (ws-realtime
  ctrl 100 Hz + telem 10 Hz, 20 s window): at the 100 ms default both tiers
  converge to ~4.3 msg/s each (~8.6 total — a 100 Hz and a 10 Hz publisher get
  the SAME throughput under contention); at 5 ms, ~39 total (ctrl 33 / telem 5.5).
  `Z_CONFIG_SOCKET_TIMEOUT` trades read-wake rate for tx budget but stays
  window-bound. Mitigation (phase-279, opt-in): `ZPICO_TX_BATCH=1` env /
  Kconfig `CONFIG_NROS_ZENOH_TX_BATCH=y` enables tx batching PLUS a dedicated
  flush thread (multi-threaded platforms except ThreadX; flush cadence
  `ZPICO_TX_BATCH_FLUSH_MS`, default 50 ms) — measured **4× total throughput at
  the 100 ms default (34.1 vs 8.6 msg/s) and 1.35× at 5 ms (52.5 vs 39)**, with
  the 10 Hz tier reaching ≈ ideal. Flushing from the tier threads themselves
  measured WORSE-or-equal (4.7-9.2) — the flush must live on its own thread.
  Phase-282 closed the residual: `ZPICO_TX_SPLIT_LOCK=1` /
  `CONFIG_NROS_ZENOH_TX_SPLIT_LOCK=y` (requires TX_BATCH) makes the flush
  STEAL the batch (buffer swap) and write the socket under a separate link
  mutex, and a batch overflow PARKS the buffer instead of blocking the
  publisher — tiers 43.2 total (+27%), streaming **~181 msg/s vs ~9 baseline
  (20×)** with a tight-loop publisher completing instead of stalling. Wire
  order stays == SN order. Per-publisher escape: `tx_express` in the QoS
  profile (Rust `.tx_express(true)` / C `nros_qos_t.tx_express` / C++
  `nros::QoS().tx_express(true)`) bypasses the batch for latency-critical
  low-rate topics (on Zephyr an express put pays the socket window itself —
  never use it for streams). Flush-thread attrs:
  `zpico_set_flush_task_config()`. Gets + query replies go express under the
  knob; keepalives bound batch sit-time. NOTE: TX_BATCH flips
  `Z_FEATURE_BATCHING` and TX_SPLIT_LOCK flips `Z_FEATURE_TX_SPLIT_LOCK` in
  the SHARED generated zenoh config (both gate transport-struct fields — the
  issue-0135 every-TU rule); rebuild fixtures after changing them. Harnesses:
  `tests/w1_zephyr_tx_throughput_measure.rs` (`--ignored`, tiers) +
  `packages/testing/nros-bench/stress-zenoh-zephyr` (streaming; bench
  listeners MUST build with `ZPICO_SUBSCRIBER_RING_DEPTH=1024` — the default
  4-slot ring drop-newests batched callback bursts). User-facing decision
  tree: book "TX Throughput & Latency Tuning" page.
- **`Z_CONFIG_SOCKET_TIMEOUT` must stay short on Zephyr (100 ms, like the unix
  port).** At 5000 ms the client's ~3.3 s lease keepalives miss the 10 s lease and
  zenohd silently drops the session — the image keeps spinning against a dead
  transport (boot-time declares crawl at one per recv window, then everything
  wedges). Fork patch in `zenoh-pico/include/zenoh-pico/config.h` (+ the
  `zenoh_generic_config.h` twin); NuttX intentionally keeps 5000.
- **Intra-image pub→sub needs `Z_FEATURE_LOCAL_SUBSCRIBER=1`** (set by
  `zephyr/cmake/nros_rmw_zenoh.cmake` on Zephyr; the generated
  `zenoh_generic_config.h` from `nros-zpico-build` sets it for every
  cargo-built lane, embedded included): all nodes in an image share ONE session
  (RFC-0015 Model 1), and neither zenoh-pico nor zenohd loops a publication back
  to the session it came from. Without it a same-image pair (e.g. ws-qos-rust's
  `reliable_talker → qos_listener`, or the multi-tier FreeRTOS safety island's
  gate→actuator command) never delivers, silently — remote routes keep working,
  so the drop masquerades as a per-route scheduling bug.
- The #129 per-node-liveliness "deadlock" was this same mechanism — the first
  declare to hit the recv window. The `platform-zephyr` gate it prompted was
  LIFTED after the timeout fix (issue 0143): per-node NN tokens declare fine
  on Zephyr now, and multi-node images list every component in
  `ros2 node list`.
- **tx throughput ceiling**: because every send waits for the read task's recv
  window, total tx is capped at ~one send per window (plus inbound-traffic
  wakes). At the 100 ms default that is ~10 msg/s for the WHOLE image — fine
  for 1 Hz demo lanes, fatal for high-rate publishers. Tune per-app via
  `CONFIG_NROS_ZENOH_SOCKET_TIMEOUT_MS` (→ `Z_CONFIG_SOCKET_TIMEOUT`); the
  ws-realtime zephyr entry (100 Hz ctrl tier) uses 5 ms.
- **Concurrent entity declares race the interest write filter**: a declare
  triggers an interest handshake, and when two threads declare concurrently
  on one session (only on `Z_FEATURE_MULTI_THREAD=0` — embedded; native's
  MT=1 has internal session mutexes) the losing publisher's zenoh-pico write
  filter can stay closed — `z_publisher_put` succeeds while nothing reaches
  the wire. All embedded `run_tiers` paths (`ZephyrBoard::run_tiers`, the
  FreeRTOS Rust `run_tiers_entry`, and the FreeRTOS C
  `nros_board_freertos_run_tiers`) therefore **chain-spawn** the tiers: the
  boot tier runs its setup, then spawns exactly one task for the next tier,
  which runs ITS setup and spawns the next — so no two `setup()` (declare)
  bursts ever overlap, for any tier count (issue 0144; the 0128 fix only
  closed boot↔tier for two tiers). Spins may overlap the next tier's setup —
  safe, a spin exchanges keepalives/data, not declares.

## NuttX ↔ zenoh-pico cooperation (Phase 225.O)

zenoh-pico is **not** platform-agnostic on the C side and is not meant to be;
the Phase 227.3 "platform-agnostic" refactor (`365d5cdce`) only made the
**Rust shim** (`nros-rmw-zenoh`) generic (no `target_os`/NuttX branches —
just feature gates). zenoh-pico C keeps per-platform system layers and
`#ifdef ZENOH_NUTTX` accommodations. How NuttX wires up:

1. **Feature → define.** `nros/platform-nuttx` forwards
   `nros-rmw-zenoh?/platform-nuttx` → `zpico-sys/nuttx` → `CARGO_FEATURE_NUTTX`,
   and `nros-zpico-build` then `#define ZENOH_NUTTX` + selects the **`unix`**
   system layer (`zenoh-pico/system/common/platform.h`: NuttX is grouped with
   `ZENOH_LINUX`/`MACOS`/`BSD`) + `LinkPolicy::nuttx()`. The forwarding clause
   is load-bearing: without it `ZENOH_NUTTX` is undefined and the setsockopt
   guards below stay off.
2. **`unix` system layer = direct POSIX.** NuttX is a hosted POSIX RTOS, so
   `system/unix/system.c` backs the primitives directly — `z_malloc`→libc
   `malloc`, `_z_task_*`→`pthread_create`/`join`, `_z_mutex_*`→`pthread_mutex_*`,
   sockets→NuttX kernel BSD sockets. No bare-metal platform shim is needed
   (contrast bare-metal/ESP32, which route through `nros-platform-*` + smoltcp).
   `nros-platform-nuttx` is a thin C-only glue crate (`platform.c`/`net.c`).
3. **`ZENOH_NUTTX` accommodations** (6 branches in `system/unix/network.c`):
   skip `<ifaddrs.h>`/`getifaddrs` (NuttX lacks it → multicast binds
   `INADDR_ANY`); skip `SO_LINGER` (no `CONFIG_NET_SOLINGER`); skip
   `TCP_NODELAY` (host vs NuttX optname value mismatch under cross-compile);
   use `MSG_NOSIGNAL` on `send` and free `getaddrinfo` results (both shared
   with `ZENOH_LINUX`). The SO_LINGER/TCP_NODELAY skips are why
   step 1's `ZENOH_NUTTX` define matters — otherwise those setsockopts fail on
   NuttX and `_z_open_tcp` returns `Transport(ConnectionFailed)`.
4. **Backend registration is explicit on NuttX.** The unified-RMW
   `nros_rmw_register_backend!` macro expands to a `linkme` distributed-slice
   entry on supported targets but to **nothing** on NuttX (linkme unsupported),
   and the standalone flat image does not run the auto-register `.init_array`.
   So `nros-board-nuttx::run_entry` calls `nros_rmw_zenoh::register()`
   explicitly before `Executor::open` (feature `rmw-zenoh`, wired entry →
   `nros-board-nuttx-qemu` → `nros-board-nuttx`) — same shape as the esp32
   board.
5. **pthread pool.** Like Zephyr, NuttX needs enough pthread mutex/sem
   resources (zenoh-pico uses ~8+ mutexes; transport TX/RX/peer + per-publisher
   write-filter under `Z_FEATURE_INTEREST=1`). The reference `qemu-armv7a`
   defconfig suffices for talker+listener; raise the NuttX pthread limits for
   heavier graphs.

Verified 2026-06-09: native_sim-less, on `qemu-system-arm -M virt -cpu
cortex-a7`, the NuttX workspace Entry boots → registers → publishes `/chatter`
→ an external native listener receives it cross-process.

## Platform-ABI carve-outs: Serial / IVC / PlatformLibc are Rust-only (issue 0244)

Not every platform primitive has a C-ABI mirror in the `nros_platform_*`
headers. `PlatformSerial` and `PlatformIvc` (Tegra IVC mailbox) are **Rust
traits only** — no `nros_platform_serial` / `nros_platform_ivc` header — because
they are consumed exclusively by Rust boards. `PlatformLibc` is documentary-only
(link-resolved from `nros-baremetal-common`, never dispatched).

This is a **deliberate carve-out, not an oversight.** Post-RFC-0054 the C headers
are the platform-ABI SSoT and the Rust side consumes committed bindgen output;
hand-written C mirrors of Rust traits are the RETIRED model. So the rule is: a
primitive joins the portable C ABI only when a C consumer needs it, at which
point it is AUTHORED as an SSoT header and the Rust trait wraps the generated
bindings — never hand-mirrored ahead of a consumer. Until then, Rust-only is
correct.

Separately, `zpico-sys/src/platform_smoltcp.rs` exposes `smoltcp_set_clock_ms` /
`smoltcp_clock_now_ms` — an externally-fed monotonic tick (PUSH: the board timer
ISR calls `set_clock_ms`) the bare-metal smoltcp stack needs, beside the
canonical `nros_platform_clock_ms` (PULL). It is required today; unifying it onto
the SSoT platform-clock header is a phase-230 normalization item.
