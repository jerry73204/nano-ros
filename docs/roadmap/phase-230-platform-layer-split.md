# Phase 230 — Platform layer split: enforce the system-ABI boundary (alloc first)

**Goal:** Make the platform/RMW/core split (RFC-0034) a real, enforced
invariant. Today the `nros_platform_*` ABI is bypassed on every RTOS:
zenoh-pico and the Rust `#[global_allocator]` call `pvPortMalloc` /
`k_malloc` / `tx_byte_allocate` directly, so the platform layer's RTOS
providers are dead code. Route the **allocator** through the ABI on all
platforms first (the starter), then the other scalar services, dedupe the
per-RMW bridge, and add a lint that keeps the boundary from re-rotting.
The unified allocation funnel also yields the true heap stats that
[issue 0006](../issues/archived/0006-rtos-dual-heap.md) needs.

**Status (2026-09-04). NOT "Planned" — Wave 0 and Wave 3 are marked DONE in
this doc, the lint is on the fast line, and every scalar service Wave 2 names is
in the shipped ABI.** The one-word status has been wrong for long enough that
the phase reads as unstarted work; what is actually left is two items this doc
already labels follow-ups.

Measured against the tree today, not inferred:

* `packages/platform/nros-platform-cffi/src/generated.rs` exports **98**
  `nros_platform_*` symbols, including everything 230.2.1 asks for —
  `sleep_ms`/`sleep_us`/`sleep_s`, `clock_ms`/`clock_ns`/`clock_resolution_ns`,
  `yield_now`, `random_fill`/`random_u*` — plus 230.1.6's heap stats
  (`heap_total_bytes`, `heap_used_bytes`).
* The lint 230.0.2 specified EXISTS and is enforced:
  `scripts/check-no-direct-kernel-alloc.sh`, wired at `just/check.just:208` in
  the **fast** list, so it runs on every push rather than on request.
* The stated defect is gone. `grep` for `pvPortMalloc` / `k_malloc` /
  `tx_byte_allocate` across the zpico C shim returns NOTHING; the goal text
  below still describes them as bypassing the ABI "on every RTOS".
* [issue 0006](../issues/archived/0006-rtos-dual-heap.md) — the accurate heap
  accounting this phase said it would unblock — is `status: resolved` and
  archived.
* [phase-391](phase-391-allocation-unification-and-tier-model.md) reports
  "W1-W5 landed; the tier is real and gated", covering the same allocation
  ground from the tier side.

**Deliberately NOT claimed:** that 230.1.1-230.1.6 are individually complete.
Their ✅ marks are absent and what is measured above is the ABI SURFACE, which
is weaker evidence than a per-item check (a fork guard or an init-order
contract does not show up as a symbol). They are left unmarked rather than
promoted on a surface reading.

What remains, and it is what the doc already calls it: the two unchecked
follow-up boxes — Z5 (promote to the canonical platform ABI) and 1b.3 (the
public accessor behind the C/C++ API).

**Priority:** Medium — architecture/tech-debt. Design is locked (RFC-0034);
no new public API. Unblocks accurate embedded heap accounting and a
genuinely single system-access layer.

**Depends on:** RFC-0034 (this phase implements it), RFC-0006 (C-ABI
canonical + platform free-symbol model), [platform-c-abi.md](../../book/src/internals/platform-c-abi.md)
(symbol contract + drift gate). Touches the vendored zenoh-pico fork
(`packages/rmw/zenoh/zpico-sys/zenoh-pico`) and `nros-zpico-build`.

## Overview

RFC-0034 establishes: (1) `nros_platform_*` is the sole system boundary;
(2) **scalar** services (alloc/sleep/clock/random) unify through it on
every platform, **opaque-struct** services (task/sync/net) stay
per-RTOS-vendored by design; (3) one platform-owned `z_* → nros_platform_*`
bridge, category-gated; (4) the platform port owns + inits the heap; (5) a
no-direct-kernel-call lint enforces it.

This phase delivers that in waves, alloc first. Each wave is independently
landable and leaves the tree green.

## Architecture

- **Bridge ownership:** the `z_* → nros_platform_*` alias TU becomes a
  single platform-layer artifact consumed by both `zpico-sys` and
  `nros-rmw-xrce` (retiring the duplicate `platform_aliases.c`). Emission
  is per-category: a **memory-only** alias on FreeRTOS/ThreadX/Zephyr
  (vendor keeps task/net), full alias where it already works
  (POSIX/bare-metal).
- **Vendor strip:** zenoh-pico's strong scalar defs (`z_malloc`/`z_free`/
  `z_realloc` and later `z_sleep_*`/`z_clock_*`) are guarded behind a fork
  `#ifdef` (`Z_FEATURE_NROS_PLATFORM_ALLOC` / `…_SCALAR`) so the alias wins
  with no double-definition.
- **Heap ownership:** each port owns + initializes its pool before first
  alloc; the C side always calls `nros_platform_alloc`. The Rust
  `#[global_allocator]` is an **optional, board-selected singleton**
  (RFC-0034 D6): nano-ros provides one (→ `nros_platform_alloc`) only where
  no framework claims the slot; on Zephyr/esp-hal/native the framework owns
  it and nano-ros yields.
- **Stats (two-mode, RFC-0034 D7):** where nano-ros owns the allocator, the
  single `nros_platform_alloc` funnel (`used`/`peak`) is the exact C+Rust
  total; where the framework owns it, the funnel counts the C side and the
  platform-native heap query gives the unified total.

## Work items

### Wave 0 — Audit + lint scaffold

#### 230.0.1 — Direct-kernel-call audit  ✅ DONE
`scripts/check-no-direct-kernel-alloc.sh` is the executable audit. It found
**40 bypass sites** — broader than RFC-0034's initial table:

- **Rust `#[global_allocator]`** — `nros-c`/`nros-cpp/src/lib.rs`
  (FreeRTOS→`pvPortMalloc`, Zephyr→`k_malloc`, ThreadX→`z_malloc`) + the
  cbindgen-emitted `extern` re-decls in `nros_generated.h` / `nros_cpp_ffi.h`.
- **C-API inline platform headers** — `nros-c/include/nros/platform/{freertos,zephyr}.h`.
- **Board crates (newly surfaced)** — `nros-board-freertos`
  (`entry.rs`/`node.rs`: task-context + `AppContext` allocation via
  `pvPortMalloc`), `nros-board-orin-spe` (same), `nros-board-threadx-qemu-riscv64`
  + `nros-board-common` (net/IP/ARP/BSD pools via `tx_byte_allocate`).
- **Vendored zenoh-pico** `system/{freertos,threadx,zephyr}/system.c`
  `z_malloc` (out of the lint's scope — guarded separately in 230.1.1).

Implication: Wave 1 scope grew. Board-crate task-context allocations + the
C-API inline headers are additional funnel sites. The board allocations
are a distinct sub-case (board glue sizing its own task contexts / net
pools) and may legitimately keep direct calls if scoped out — decided per
site during 230.1.

#### 230.0.2 — `no-direct-kernel-alloc` lint  ✅ DONE
`scripts/check-no-direct-kernel-alloc.sh` — word-boundaried symbol scan
(`pvPortMalloc`/`vPortFree`/`k_malloc`/`k_free`/`tx_byte_allocate`/
`tx_byte_release`/`heap_caps_*`), excludes vendored zenoh-pico/mbedtls +
build output, allows `nros-platform-*` / `platforms/*` ports plus the
documented ThreadX task/net byte-pool carve-out (`TASK_NET_ALLOW_RE`).
**HARD by default** as of 230.1.7 (nros-owned surface clean);
`NROS_ALLOC_GATE_HARD=0` reverts to advisory for triage. Wired into
`just check`.

### Wave 1 — Allocator unification (the starter)

**Active slice — VERIFIED FINDING (2026-06): the Zephyr C-side funnel
already exists.** Attempting Z1/Z2 (fork-guard + memory-only alias) broke
the link (`undefined z_sleep_s`/`z_random_fill`), which proved the
opposite of the earlier assumption: the nano-ros Zephyr build **does not
compile** vendored `system/zephyr/system.c` at all — the
`platform_aliases.c` TU is the sole `z_*` provider and already forwards
**all** scalar services to `nros_platform_*`. Disassembly of the built
`zephyr.exe` confirms it:

```
<z_malloc>:  jmp <nros_platform_alloc>
<z_free>:    jmp <nros_platform_dealloc>
```

So on Zephyr there is **no bypass to remove** — zenoh-pico's C allocation
is already funneled through `nros_platform_alloc` (k_heap-backed via
`nros-platform-zephyr`). Z1–Z4 are **unnecessary on Zephyr** and were
reverted. The earlier audit/RFC premise ("vendored zenoh-pico `z_malloc` →
`k_malloc` bypass") is true only for a *standalone* zenoh-pico Zephyr-module
build, **not** the nano-ros Rust-entry build. The Zephyr slice reduces to:

- [x] Z5 (Zephyr) — heap-stats query **landed + verified**.
  `nros_platform_heap_used_bytes()` / `_total_bytes()` in
  `nros-platform-zephyr/src/platform.c` query `_system_heap` via
  `sys_heap_runtime_stats_get` under `CONFIG_SYS_HEAP_RUNTIME_STATS`
  (enabled in the listener `prj-zenoh.conf`). Verified end to end on
  native_sim: `heap used=8792 total=64896` — the exact D7 Mode-B unified
  figure (both zenoh-pico via `nros_platform_alloc` and zephyr-lang-rust's
  Rust allocator draw from `_system_heap`). Closes #6 for Zephyr.
- [ ] Z5 (follow-up) — promote to the **canonical** platform ABI
  (`platform.h` + `nros-platform-cffi` mirror + export macro + drift gate)
  with a **default = 0 ("unknown")** so existing ports compile unchanged;
  add the native-query impls for ThreadX (`tx_byte_pool_info_get`), POSIX
  (best-effort), bare-metal (`FreeListHeap::used`); back the public
  `nros_heap_used_bytes()` with it on RTOS. Cross-cutting (every port + the
  gate) — own change. On ThreadX both the C/C++ Rust allocator and
  zenoh-pico funnel through `nros_platform_alloc` → `tx_byte_allocate`, so
  the pool query will be exact there too.

**Per-RTOS reality (verified 2026-06).** The C-side funnel exists wherever
the alias TU compiles — confirmed by `objdump` on built binaries:
- **Zephyr** `zephyr.exe`: `z_malloc → jmp nros_platform_alloc` ✅
- **ThreadX** `threadx_c_talker` (build-zenoh): `z_malloc → jmp
  nros_platform_alloc`, `z_realloc → jmp nros_platform_realloc` ✅
- POSIX / bare-metal: alias TU active ✅ (by the same gate).

**FreeRTOS is the ONLY genuine C-side bypass** — the alias TU is explicitly
skipped (`runner.rs` `!use_freertos`) and vendored
`system/freertos/system.c` defines `z_malloc → pvPortMalloc`. So the real
cross-RTOS C-side work is **FreeRTOS-only** (guard + alias, mirroring the
existing POSIX/ThreadX path) + the optional global-allocator (230.1.4) +
heap stats (Z5). The 40-site static grep over-counted: the RMW C path on
Zephyr/ThreadX is already funneled.

---

## Picked: Wave 1b — canonical heap-stats ABI → close #6 cross-platform

Next wave. Bounded, mostly mechanical, hosted-verifiable. Promotes the
verified Zephyr Z5 query into the canonical platform ABI so every platform
reports a true heap total, closing #6 everywhere (not just Zephyr).

- [x] **1b.1 — Canonical symbol.** `nros_platform_heap_used_bytes()` +
  `nros_platform_heap_total_bytes()` added to `<nros/platform.h>`, the
  `nros-platform-cffi` `extern` mirror, the `nros_platform_export_alloc!`
  macro, and the `PlatformAlloc` trait with **default = 0** (Rust-macro
  ports — bare-metal — auto-get the 0 stub). Drift gate auto-extracts +
  passes (`platform.h` now 54 symbols, header↔mirror↔macro consistent).
- [x] **1b.2 — Port impls.** Zephyr `sys_heap` (done in Z5); ThreadX
  `tx_byte_pool_info_get` (used = pool size − available); POSIX glibc
  `mallinfo2` (`uordblks`); FreeRTOS `configTOTAL_HEAP_SIZE −
  xPortGetFreeHeapSize`; esp `heap_caps_get_{total,free}_size`. Bare-metal
  Rust ports = 0 stub via the trait default (real `FreeListHeap::used`
  deferred).
- [ ] **1b.3 — Public accessor (follow-up).** Back the C/C++-API
  `nros_heap_used_bytes()` (+ a `nros_heap_total_bytes()`) with the
  platform query on RTOS; document D7 two-mode. The canonical
  `nros_platform_heap_used_bytes()` is already user-callable (public ABI),
  so the unified figure is available now; this just routes the existing
  convenience accessor.
- [x] **1b.4 — Verify.** Gate ✅; POSIX `cargo build` (native listener)
  links + compiles the `mallinfo2` path ✅; threadx-linux `cmake --build`
  compiles the `tx_byte_pool_info_get` path + links ✅; Zephyr native_sim
  runtime (Z5) `heap used=8792 total=64896` ✅. FreeRTOS/esp use standard
  APIs (`xPortGetFreeHeapSize` / `heap_caps_*`) — build-verify in their
  lanes / Wave 1c. [issue 0006] resolved (unified figure available + verified).

**Deferred to later waves (split completion, heavier verify):**
#### Wave 1c — FreeRTOS C-side funnel (EXPANDED 2026-06)

The last genuine C-side bypass: baseline `objdump` confirms FreeRTOS
`z_malloc → b.w pvPortMalloc`. A first attempt (reverted) proved the guard
half works — defining `Z_FEATURE_NROS_PLATFORM_ALLOC` on the freertos
vendored cc build (`build_zenoh_pico_unified`) removed `z_malloc`/
`pvPortMalloc` from the rebuilt `libzenohpico.a`. But no alias TU was
linked, so `z_malloc` would be **undefined at the final ELF link**.

Design exploration (re-scope): the alias TU is **default-on**
(`zpico-sys default = ["platform-aliases", …]`; the `nros-rmw-zenoh →
zpico-sys` edge does not disable defaults). The real blocker was the
`runner.rs` alias gate (`!use_freertos`) skipping FreeRTOS — the earlier
"feature off" read was a build-cache artifact (the env never relinked).
So the **code is ~the original size**, but two subtleties expand it:

**Implemented 2026-06 (commit fc3b6a464 + fork branch `nros-platform-scalar-funnel`
@ 735ee769). Correction to the earlier diagnosis:** the alias gate's
`!use_freertos` was one half; the *other* half was that the FreeRTOS board
(`nros-board-mps2-an385-freertos`) pulls `zpico-sys` with
`default-features = false, features = ["freertos"]` — so `platform-aliases`
was OFF on FreeRTOS all along (not the `nros-rmw-zenoh` edge). Enabling it on
the board is what activates the funnel.

- [x] **1c.1 — Couple guard ⇔ alias.** `Z_FEATURE_NROS_PLATFORM_ALLOC` (the
  vendored-guard define, Step 6.5 in `build_zenoh_pico_unified`) AND the
  memory-only alias compile are both gated on `CARGO_FEATURE_PLATFORM_ALIASES`
  (+ `CARGO_FEATURE_FREERTOS`). Off ⇒ vendored `z_malloc` stays = today's
  behaviour. orin-spe keeps `platform-aliases` off (FSP-native) ⇒ unchanged.
- [x] **1c.2 — Memory-only alias for FreeRTOS.** `runner.rs` adds a FreeRTOS
  alias-gate branch compiling `platform_aliases.c` with
  `NROS_ZP_ALIAS_MEMORY_ONLY` (only `z_malloc`/`z_realloc`/`z_free`; vendored
  keeps sleep/random/clock/task/net). `system.c` guards the vendored defs
  behind `#ifndef Z_FEATURE_NROS_PLATFORM_ALLOC`; `platform_aliases.c` wraps
  everything after `z_free` in `#ifndef NROS_ZP_ALIAS_MEMORY_ONLY`.
- [x] **1c.3 — Coverage.** Board feature enabled on
  `nros-board-mps2-an385-freertos` (shared by the workspace entry + every
  role example), so all FreeRTOS zenoh fixtures inherit the funnel.
- [~] **1c.4 — Verification.** **Static proof done locally** (contradicts the
  earlier "can't relink locally" assumption): on a clean thumbv7m build the
  guarded `libzenohpico.a` has `z_malloc` def=0 / 32 undef-refs, the
  memory-only alias archive defines `z_malloc → nros_platform_alloc`, and the
  full FreeRTOS firmware **links clean** (only weak C++ EH symbols undefined)
  — proving `z_malloc` resolves through the funnel at the real embedded link.
  Heap-identical to before (`nros_platform_alloc → pvPortMalloc`, same
  heap_4). **Runtime QEMU pub/sub E2E remains the CI gate** — the local
  fixture-staging env (`NROS_PLATFORM_FREERTOS_SRC`, `_entry` ELF rename)
  blocks running it here; the FreeRTOS QEMU CI lane runs it.

Push order: push the `nros-platform-scalar-funnel` fork branch FIRST, then
push fc3b6a464 (the superproject pointer references the unpushed fork
commit). 1c.4 runtime E2E is the gating acceptance on the CI lane.
- **Wave 1d — optional Rust global allocator (D6)** — largely landed
  (2026-06):
  - The optional, board-selected provider **already existed**:
    `nros-platform`'s `global-allocator` feature installs
    `PlatformGlobalAllocator` → `<ConcretePlatform as PlatformAlloc>` →
    `nros_platform_alloc` (off by default; the example/board crate opts in).
    `nros-platform-mps2-an385` similarly exposes `global-alloc` →
    `FreeListHeap` (bare-metal single heap). So D6's "optional, owned where
    the slot is free, off where a framework owns it" is satisfied.
  - **Funnel fix (landed):** `nros-c`/`nros-cpp`'s per-platform
    `#[global_allocator]`s (FreeRtos/Zephyr/ThreadX, the C/C++ API path) now
    call `nros_platform_alloc`/`_dealloc` instead of `pvPortMalloc`/
    `k_malloc`/`z_malloc` directly — one funnel (Mode A: exact heap stats
    via 1b). Verified: the no-direct-kernel-alloc inventory dropped 40 → 20
    (both crates cleared). Embedded link verified in their lanes.
  - **At-most-one-provider:** enforced by **rustc** (a second
    `#[global_allocator]` in the link is a hard compile error). The
    providers (`nros-platform/global-allocator`,
    `nros-platform-mps2/global-alloc`, `nros-c`/`nros-cpp` per-platform,
    `zpico-alloc/global-alloc`) are mutually exclusive by the platform-`*`
    feature compile-error + opt-in features; a board enables exactly one.
  - **Remaining:** real bare-metal `FreeListHeap::used` for the 1b heap
    query (currently 0-stub via the trait default); optional `just check`
    enumerator of `#[global_allocator]` sites for earlier auditability.
- **Wave 1e — board-crate task-context sites + flip the lint hard.  ✅ DONE**
  FreeRTOS (`nros-board-freertos` `entry.rs`/`node.rs`) + orin-spe
  (`nros-board-orin-spe` `node.rs`) task-context `pvPortMalloc`/`vPortFree`
  sites now route through `nros_platform_alloc`/`_dealloc` (the
  platform-freertos provider wraps `pvPortMalloc`/`heap_4` — same heap, one
  funnel). The ThreadX board `tx_byte_allocate` sites (`threadx_hooks.c` app
  thread stack; `board_threadx_qemu_riscv64.c` NetX packet/IP/ARP/BSD pools)
  are the vendored TASK + NET opaque-struct services and stay direct, now on
  a documented symbol-scoped lint allowlist (`TASK_NET_ALLOW_RE`). With the
  nros-owned surface clean, 230.0.2 flips HARD by default (230.1.7). Embedded
  ELF link verification of the board edits is CI-gated (full firmware build).

- **Wave 1f — alloc funnel on ThreadX + Zephyr  ✅ DONE (both already funneled).**
  Investigation (post-1c) found that neither ThreadX nor Zephyr actually had a
  bypass: only FreeRTOS compiled a vendored `z_malloc`-defining `system.c`. The
  pre-existing alias TU (`platform_aliases.c`, `platform-aliases` on) already
  routes `z_malloc → nros_platform_alloc` on both. The only change was removing
  ThreadX's dead weak footgun. Details per-RTOS below.

  **ThreadX — ALREADY FUNNELED (verified 2026-06); only the footgun removed.**
  The earlier plan here assumed a vendored `system/threadx/system.c` to guard
  like FreeRTOS. That file does **not** exist: nano-ros ThreadX uses zenoh-pico's
  **generic `system/common`** platform (`zenoh_platforms.toml`
  `include = ["system/common"]`), which defines **no** `z_malloc` — so there is
  no vendored bypass to guard. The funnel is owned by the alias TU
  (`platform_aliases.c`), which is **on by default** for the ThreadX boards
  (`nros-board-threadx*` keep `zpico-sys` default features, unlike the FreeRTOS
  board) and emits a STRONG `z_malloc`/`z_free` → `nros_platform_alloc`/
  `_dealloc`. Verified on a threadx-linux zenoh build: `objdump` shows
  `z_malloc: jmp nros_platform_alloc` (strong `T`, from `platform_aliases.o`),
  `z_free → nros_platform_dealloc`. Mode-A heap stats already exact (1b).

  So no fork guard, no `runner.rs` change, no board change is needed for ThreadX
  allocation. The one cleanup (DONE): **removed the dead
  `__attribute__((weak)) z_malloc`/`z_free` in
  `nros-platform-threadx/src/platform.c`** — RFC-0034's "footgun". It was
  silently shadowed by the strong alias (never linked), so removal cannot change
  the linked output; it makes a `platform-aliases`-off ThreadX zenoh build fail
  to link **loudly** (no `z_malloc` provider) instead of falling back to a hidden
  weak forwarder. (If such a config is wanted later, it needs `platform-aliases`
  on — there is no vendored `z_malloc` to fall back to on the generic platform.)
  - Verify (CI): threadx-linux + ThreadX QEMU (riscv64) links + e2e stay green.

  **Zephyr — C-side ALREADY FUNNELED (confirmed by design 2026-06); no guard
  needed.** Same correction as ThreadX. zenoh-pico C is built by **Zephyr CMake**
  (`zephyr/cmake/nros_rmw_zenoh.cmake`), which globs only `src/system/common/*`
  + `src/system/zephyr/network.c` — it does **NOT** compile
  `src/system/zephyr/system.c` (the file that defines the vendored
  `z_malloc → k_malloc`). So that vendored def never reaches the Zephyr link;
  there is no bypass to guard. The CMake comment is explicit: Zephyr's
  memory/clock/sleep "is replaced by the alias TU (`platform_aliases.c`)
  compiled inside the cargo staticlib... the single replacement provider." The
  alias TU is compiled for Zephyr (the runner.rs alias gate fires —
  `platform-aliases` is on via `nros-rmw-zenoh`'s default `zpico-sys` dep — with
  the network section elided via `NROS_ZENOH_PLATFORM_USES_UNIX`, memory aliases
  kept), emitting `z_malloc → nros_platform_alloc` resolved against
  `nros-platform-zephyr`'s k_heap provider.

  So the **C side is funneled** with no guard, no `runner.rs`/CMake change. The
  **Rust `#[global_allocator]` stays zephyr-lang-rust's** (k_malloc) by design
  (D6 — nano-ros yields the slot to the framework; `global-alloc` off on
  Zephyr), and stats are **Mode B** via the native `sys_heap` query (both the
  framework Rust heap and zenoh-pico share `k_heap`, so the native total is
  exact — already wired, Z5/1b). This is the RFC-0034 D6/D7 Zephyr end-state.
  - Verify (CI): Zephyr native_sim + QEMU zenoh e2e (SDK not provisioned in this
    dev env, so confirmed by design here; the Zephyr CI lane is the runtime
    gate). `objdump` on the image should show zenoh `z_malloc →
    nros_platform_alloc`.

  **Status: all ✅.** ThreadX (already funneled; dead weak footgun removed),
  Zephyr (C-side already funneled via the alias TU; Rust-side framework-owned
  per D6), and **NuttX** (also generic `system/common` + `system/unix/
  network.c`, no vendored `z_malloc` → already funneled via the alias TU).
  **FreeRTOS (1c/Wave 2) was the only RTOS that compiled a vendored
  `z_malloc`-defining `system.c`** (via `build_zenoh_pico_unified`) and thus the
  only real bypass; every other RTOS uses zenoh-pico's generic `system/common`
  (which defines no `z_malloc`), so the pre-existing alias TU already funneled
  them. RFC-0034's motivation table (which showed ThreadX/Zephyr `z_malloc` as
  *direct*) reflected a pre-alias-TU snapshot; corrected in the RFC. Runtime
  confirmation of all RTOSes rides the per-RTOS CI lanes.

> **Zephyr-slice investigation (2026-06).** On the Zephyr *Rust* path there
> are two allocators and neither is nros's: the `#[global_allocator]` is
> **zephyr-lang-rust's** (`modules/lang/rust/zephyr/src/alloc_impl.rs` →
> `k_malloc`), and zenoh-pico's C `z_malloc` → `k_malloc` independently.
> `nros-c`/`nros-cpp`'s `ZephyrAllocator` only governs the **C/C++ API**
> path, not the Rust entry. `nros-platform-zephyr` does provide
> `nros_platform_alloc` (k_heap-backed) as a Zephyr CMake module. So a true
> single funnel on Zephyr Rust needs BOTH: (a) route zenoh-pico `z_malloc`
> → `nros_platform_alloc` (guard + alias), and (b) install an nros
> `#[global_allocator]` in the entry/board that wraps `nros_platform_alloc`,
> shadowing zephyr-lang-rust's.
>
> **Decision (2026-06, revised): C-side funnel + optional Rust allocator
> (RFC-0034 D6/D7).** The earlier "full funnel via patching zephyr-lang-rust"
> plan is dropped. zephyr-lang-rust's `#[global_allocator]`
> (`ZEPHYR_ALLOCATOR` → `malloc`) is unconditional and Rust allows one per
> binary; rather than patch a framework allocator, nano-ros's
> `global-alloc` is **off on Zephyr** (the framework owns the Rust heap).
> nano-ros still routes the **C side** (zenoh-pico `z_malloc`) through
> `nros_platform_alloc`, and reads the **true heap total from Zephyr's
> native `sys_heap` runtime stats** (`CONFIG_SYS_HEAP_RUNTIME_STATS`) —
> both the framework Rust allocator and zenoh-pico share `k_heap`, so the
> native query is exact without owning the Rust allocator. No
> zephyr-lang-rust patch, no entry-side allocator boilerplate.

**Concrete Zephyr 230.1 steps (ready to execute, revised):**
1. Fork-edit `zenoh-pico/src/system/zephyr/system.c`: guard `z_malloc`/
   `z_free` (+ the NULL `z_realloc`) behind `#ifndef Z_FEATURE_NROS_PLATFORM_ALLOC`.
   Commit in the submodule (the project's own fork); bump the pointer.
2. `nros-zpico-build`: emit a **memory-only** alias (`z_malloc` →
   `nros_platform_alloc`, no sleep/random/clock — those stay vendored to
   avoid dup symbols) for Zephyr and define `Z_FEATURE_NROS_PLATFORM_ALLOC`.
3. Ensure `nros-platform-zephyr`'s `nros_platform_alloc` is on the Zephyr
   app link line (ships as a Zephyr CMake module — wire into the entry's
   `west` build if not already pulled).
4. **No zephyr-lang-rust patch.** nano-ros `global-alloc` feature stays
   **off** on Zephyr; the framework keeps `ZEPHYR_ALLOCATOR`.
5. Stats: instrument `nros_platform_alloc` (`used`/`peak`) for the C side,
   and expose the Zephyr-native `sys_heap` total as the unified figure
   (enable `CONFIG_SYS_HEAP_RUNTIME_STATS`); document the two-number mode
   per D7.
6. Build `rust/listener/zenoh` + `rust/talker/zenoh` Zephyr fixtures; run
   `test_zephyr_to_native_e2e` / `test_native_to_zephyr_e2e`; confirm green
   + zenoh-pico allocations route through `nros_platform_alloc`.

#### 230.1.1 — Fork guard for vendored scalar alloc
Guard `z_malloc`/`z_free`/`z_realloc` in zenoh-pico's
`system/{freertos,threadx,zephyr}/system.c` behind
`Z_FEATURE_NROS_PLATFORM_ALLOC`. Commit on the fork branch with linear
history; bump the submodule pointer per the vendored-fork workflow (agent
leaves the branch ready; maintainer pushes the fork).

#### 230.1.2 — Memory-only alias emission on RTOS
Add a `NROS_ZP_ALIAS_MEMORY_ONLY` path to the alias TU + `nros-zpico-build`
so FreeRTOS/ThreadX/Zephyr emit the scalar (`z_malloc`→`nros_platform_alloc`)
forwarders while leaving task/net to the vendor. Define
`Z_FEATURE_NROS_PLATFORM_ALLOC` for those targets. Remove the ineffective
ThreadX weak-`z_malloc` footgun (`nros-platform-threadx/src/platform.c`).

#### 230.1.3 — Zephyr scalar port surface
Stand up the scalar `nros_platform_alloc/dealloc/realloc` provider for
Zephyr (k_heap-backed) — today Zephyr has no C `nros_platform_*` provider
on the link path. Wire it so the memory-only alias resolves.

#### 230.1.4 — Optional nano-ros Rust global allocator (RFC-0034 D6)
nano-ros's `#[global_allocator]` becomes optional + board-selected. Where
no framework owns the slot (bare-metal, FreeRTOS-via-nros-board, ThreadX),
provide one (a small `nros-alloc` gated by `nros-global-alloc`, or the
existing `nros-c`/`nros-cpp` allocators) that wraps `nros_platform_alloc`/
`_dealloc` — one funnel for C + Rust. Where a framework owns it (Zephyr
zephyr-lang-rust, esp-hal esp-alloc, native `std`), the feature is **off**
and nano-ros installs nothing — never patch the framework allocator. Add a
`just check` assertion that at most one global-allocator provider is on the
link line.

#### 230.1.5 — Init-order contract
Ensure each port initializes its pool before first alloc; document the
contract in [platform-c-abi.md](../../book/src/internals/platform-c-abi.md)
(board/runtime platform-init → transport/alloc). Verify on
ThreadX/FreeRTOS QEMU + Zephyr native_sim.

#### 230.1.6 — Heap stats (two-mode, RFC-0034 D7)
Instrument `nros_platform_alloc` (`used`/`peak`, opt-in `alloc-stats`).
**Mode A** (nano-ros owns the allocator, D6 on): the funnel counter is the
exact C+Rust total; `nros_heap_used_bytes()` reads it. **Mode B**
(framework owns the allocator, D6 off — Zephyr/esp-hal): the funnel counts
the C side; expose the platform-native heap total (Zephyr `sys_heap`,
FreeRTOS `xPortGetFreeHeapSize`) as the unified figure. Document which mode
each platform is in. Update + close
[issue 0006](../issues/archived/0006-rtos-dual-heap.md).

#### 230.1.7 — Flip the lint to hard-fail  ✅ DONE
`check-no-direct-kernel-alloc.sh` defaults to `HARD_FAIL=1`. The
precondition is the **nros-owned** surface being clean (nros-c/nros-cpp
allocators 1d, C-API headers 1e, board task-context sites 1e); it is met.
The vendored zenoh-pico scalar funnel (1c) is OUT of this gate's scope
(`EXCLUDE_RE` drops the submodule) and is enforced separately by the fork
`#ifndef Z_FEATURE_NROS_PLATFORM_ALLOC` guard + its CI relink lane, so it is
not a precondition. `NROS_ALLOC_GATE_HARD=0` reverts to advisory.

### Wave 2 — Remaining scalar services

#### 230.2.1 — sleep / clock / yield / random
Apply the Wave-1 pattern to the other scalar services (no struct ABI):
guard vendored defs, alias to `nros_platform_*`, extend the lint. Lower
risk than alloc (no heap-ownership/init subtlety).

**Audit finding (2026-06).** The nros-owned scalar-time/sleep surface has
NO portable-layer bypass to migrate (unlike alloc's 1d/1e): the remaining
direct `vTaskDelay`/`tx_thread_sleep`/`k_msleep`/`k_uptime_get` calls are
all either platform PROVIDERS (board `startup.c`, the C-API inline platform
headers — they *implement* the ABI) or board-composition crates
(`nros-board-*`, RTOS-specific by definition — routing them adds indirection
for zero portability gain). So Wave 2's real payload is the **vendored**
funnel, same CI-relink-gated mechanism as 1c.

**Landed nros-owned slice — XRCE Zephyr clock (✅ DONE).** Exception found:
`xrce-zephyr/src/xrce_zephyr.c` defined `uxr_millis`/`uxr_nanos` via direct
`k_uptime_get()` and, being an app object, shadowed the canonical
`nros-rmw-xrce/src/platform_aliases.c` (a static-archive member) on the
Zephyr link — so XRCE-on-Zephyr ran the bypass. Fixed: drop `xrce_zephyr.c`
from the Zephyr build (its net-readiness moved to `nros-platform-zephyr` in
Phase 200.1; clock was its only other content) and delete the now-dead dir;
the canonical alias now resolves `uxr_*` for every target. Also corrected
the alias to use the **monotonic** `nros_platform_clock_ms` (not wall-clock
`nros_platform_time_now_ms`, which steps/returns 0 on Zephyr without an RTC)
— micro-XRCE uses these only for relative deadline deltas. Statically
verified: Zephyr's `nros_platform_clock_ms` == `k_uptime_get()` (semantics
preserved); `nros_platform_clock_ms`/`_us` are defined in
`nros-platform-zephyr/src/platform.c` (on the link).

**Vendored FreeRTOS sleep + random — ✅ DONE (Wave 2).** Same shape as 1c:
guard the vendored `system/freertos/system.c` `z_sleep_*` / `z_random_*`
behind `Z_FEATURE_NROS_PLATFORM_SLEEP` / `_RANDOM` (fork branch
`nros-platform-scalar-funnel-wave2` @ `b59c963c`, ff-merged to the fork
`main`), and extend the FreeRTOS alias TU from memory-only to **scalar-only**
(`NROS_ZP_ALIAS_MEMORY_ONLY` → `NROS_ZP_ALIAS_SCALAR_ONLY`): it now emits
`z_malloc`/`z_realloc`/`z_free` + `z_sleep_*` + `z_random_*` →
`nros_platform_*`. `runner.rs` Step 6.5 defines the two new guards alongside
`_ALLOC`, all coupled to `CARGO_FEATURE_PLATFORM_ALIASES`. ThreadX/Zephyr
already forward sleep/random via the full alias TU, so FreeRTOS was again the
only target. Verified at archive level on a clean thumbv7m build:
`libzenohpico.a` has `z_sleep_ms`/`z_random_u32` def=0 (guarded) and the
scalar alias defines them → `nros_platform_*`; firmware links clean.

**Clock / time — deferred (ABI).** `z_clock_t` is FreeRTOS's `TickType_t`
(not a portable scalar) and `z_time_now` rides the same clock section, so
those stay vendored — the scalar-only alias deliberately excludes them
(no `z_time_now` forwarder emitted). Revisit only if a portable `z_clock_t`
is needed.

### Wave 3 — Bridge dedup + boundary documentation

#### 230.3.1 — One platform-owned bridge  ✅ DONE (reframed; not a code merge)
The premise ("duplicated `platform_aliases.c`") does not hold on inspection:
the two files have **zero symbol overlap**. `zpico-sys/c/zpico/
platform_aliases.c` defines the zenoh-pico names (`z_*` / `_z_*` /
`smoltcp_*`); `nros-rmw-xrce/src/platform_aliases.c` defines only the
micro-XRCE names (`uxr_millis` / `uxr_nanos`). They are **disjoint per-RMW
adapters**, each mapping its vendor's symbol spelling onto the same canonical
`nros_platform_*` ABI. There is no shared code to collapse.

The **one bridge** D3 asks for is the `nros_platform_*` ABI itself — and both
adapters already consume only it (no RMW reaches the host kernel directly).
Merging the two thin files into one crate would *add* coupling (each is tied
to its RMW's build gating — `nros-zpico-build`'s platform/feature matrix vs.
`nros-rmw-xrce-cffi`'s) for no dedup benefit. So D3 is satisfied by the
shared ABI, not by a file merge; nothing to do. (Same false-premise pattern
as the ThreadX/Zephyr "bypass" that turned out to be already-funneled.)

#### 230.3.2 — Document the opaque-struct boundary  ✅ DONE
Recorded in [platform-c-abi.md] §"The scalar / opaque-struct boundary":
scalar services (alloc/sleep/clock/time/yield/random) fully unify; opaque-
struct services (task/sync/net) stay per-RTOS-vendored by ABI constraint —
a design boundary, not debt — with the canonical fixed-layout +
`size_probe`/`_Static_assert` escape hatch noted for any future move (net
the first candidate). Ties the ThreadX board lint allowlist to the
classification. ARCHITECTURE.md sync deferred to RFC-0034 → Stable.

## Out of scope

- Unifying opaque-struct services (task/mutex/condvar/socket) — RFC-0034
  D2; needs canonical layouts + static-asserts, deferred.
- Runtime platform pluggability — one port per binary stays (RFC-0006).
- **Patching a framework's `#[global_allocator]`** (zephyr-lang-rust,
  esp-alloc) — RFC-0034 D6; nano-ros yields the slot, never reroutes it.
- Touching the working POSIX/bare-metal alias path beyond the dedup.

## Done when

- zenoh-pico's C allocations on FreeRTOS/ThreadX/Zephyr resolve to
  `nros_platform_alloc`; no direct kernel-allocator calls remain outside
  the ports + the one optional global-allocator provider (lint hard-fails
  on violation).
- The Rust `#[global_allocator]` is nano-ros-owned (→ `nros_platform_alloc`)
  where no framework claims the slot, and cleanly yielded where one does;
  at most one provider links.
- `nros_heap_used_bytes()` reports the exact C+Rust total where nano-ros
  owns the allocator; the platform-native heap query is the documented
  unified figure where the framework owns it. [issue 0006] closed.
- The ThreadX weak-`z_malloc` footgun and the dead `nros-platform-*` alloc
  paths are gone.
- All embedded E2E (ThreadX/FreeRTOS QEMU, Zephyr native_sim, NuttX) stay
  green across the migration.
