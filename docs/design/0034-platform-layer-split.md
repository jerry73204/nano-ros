---
rfc: 0034
title: "Platform Layer Split & System-ABI Ownership"
status: Draft
since: 2026-06
last-reviewed: 2026-06
implements-tracked-by: [phase-230]
supersedes: []
superseded-by: null
---

# RFC-0034 — Platform Layer Split & System-ABI Ownership

## Summary

The platform layer owns **all** OS/kernel access (heap, time, sleep,
threading, sync, network, timer) behind a single canonical C ABI
(`nros_platform_*`, defined in RFC-0006 + [platform-c-abi.md]). Core and
RMW layers depend on that ABI and **never** touch the host kernel
directly. Today that boundary holds on POSIX and bare-metal but is
**bypassed on every RTOS** (FreeRTOS/ThreadX/Zephyr): vendored zenoh-pico
and the Rust `#[global_allocator]` call `pvPortMalloc` / `k_malloc` /
`tx_byte_allocate` directly, so the platform ABI's RTOS providers are dead
code. This RFC makes the boundary an enforced invariant, classifies which
services can be unified (scalar) vs which are constrained by opaque-struct
ABI (threads/sync/net), and mandates the **allocator** as the first
service to unify — which also yields the true unified heap stats that
[issue 0006] needs.

## Motivation / problem

Intended layering:

```
core (nros-node, runtime, codegen output)  ─┐
RMW  (zenoh-pico, xrce, cyclonedds, dust)   ─┼─► nros_platform_*  ─► one port per binary
                                             ┘     (link-time)        (posix/freertos/threadx/zephyr/…)
```

Reality (measured — see [issue 0006] + the Phase 230 audit). **Note (corrected
2026-06):** the `z_malloc` row below was an early snapshot; the phase-230
implementation found that only **FreeRTOS** actually compiled a vendored
`z_malloc`-defining `system.c` and so was the only real bypass. ThreadX (uses
zenoh-pico's generic `system/common`, no vendored `z_malloc`) and Zephyr (CMake
compiles `system/common` + `network.c`, **not** `system/zephyr/system.c`) were
already funneled through the alias TU. The corrected, post-phase-230 state:

| Service | POSIX / bare-metal | FreeRTOS | ThreadX | Zephyr |
|---|---|---|---|---|
| `z_malloc` (zenoh-pico) | → `nros_platform_alloc` (alias TU) | → `nros_platform_alloc` (1c: fork guard + memory-only alias) | → `nros_platform_alloc` (alias TU; already funneled, 1f) | → `nros_platform_alloc` (alias TU; already funneled, 1f) |
| Rust `#[global_allocator]` | (host) | → `nros_platform_alloc` (D6 provider) | → `nros_platform_alloc` (D6 provider) | zephyr-lang-rust → `k_malloc` (D6: framework owns it; native `sys_heap` stats) |

Concrete defects this creates:

- **Dead platform code.** `nros-platform-{freertos,threadx}`'s C
  `nros_platform_alloc` (and the would-be Zephyr equivalent) is never on
  the link path — a platform layer that exists but isn't used. False
  sense of a clean split.
- **A footgun (RESOLVED, phase-230 1f).** `nros-platform-threadx` shipped a
  `__attribute__((weak)) z_malloc → nros_platform_alloc`. The original framing
  ("silently shadowed by zenoh-pico's strong `z_malloc`") was inaccurate for
  ThreadX: nano-ros ThreadX uses zenoh-pico's generic `system/common`, which
  defines no `z_malloc`, so the weak was shadowed by the *alias TU*'s strong
  `z_malloc` (also → `nros_platform_alloc`) — i.e. ThreadX was in fact already
  funneled. The dead weak forwarder is removed; a `platform-aliases`-off
  ThreadX zenoh build now fails the link loudly instead of relying on a hidden
  fallback. (Verified: `objdump` shows `z_malloc → nros_platform_alloc` on a
  threadx-linux zenoh build.)
- **Duplicated bridge.** The `z_* → nros_platform_*` alias TU
  (`platform_aliases.c`) is copied per-RMW — once in `zpico-sys`, again in
  `nros-rmw-xrce`. The split should have *one* bridge the platform layer
  owns, both RMWs consume.
- **No unified heap accounting.** Because there is no single allocation
  funnel on RTOS, `nros_heap_used_bytes()` counts only the Rust side;
  zenoh-pico's C traffic is invisible ([issue 0006]'s open item).

The platform ABI itself is fine and mature (94 symbols, drift-gated, two
port shapes — RFC-0006). The gap is **enforcement**, not design.

## Design

### D1. The boundary is an invariant

`nros_platform_*` is the **sole** system boundary. No core crate, no RMW
crate (including vendored transport C), and no Rust `#[global_allocator]`
may call a host-kernel allocator/clock/thread/socket primitive directly.
Every such call resolves — at link time — to a `nros_platform_*` symbol
provided by exactly one port crate.

Consumers never depend on a *specific* port crate (no
`nros-rmw-zenoh → nros-platform-threadx` edge); they depend on the **ABI**
(the headers + the `extern "C"` mirror) and the final binary chooses the
port. This is already RFC-0006's link-time-resolution model; D1 just
forbids the bypass.

### D2. Service classification — scalar vs opaque-struct

The deciding constraint on *what can be unified* is ABI shape, not effort:

| Class | Services | ABI shape | Unify through `nros_platform_*`? |
|---|---|---|---|
| **Scalar** | alloc/free/realloc, sleep, clock, yield, random | `(scalar) → scalar` | **Yes, on every platform.** No struct layout to disagree on. |
| **Opaque-struct** | task, mutex, condvar, socket, endpoint | by-value `_z_*_t` whose layout is per-RTOS (`TX_THREAD*` embedded, `int fd`, …) | **Constrained.** A generic alias layout has a different by-value ABI than the vendor layout → pass-by-value mismatch → crashes (the documented ESP32/NuttX `lhu`-on-an-IP faults; see `nros-zpico-build/src/runner.rs` Phase 160 notes). |

**Rule:** scalar services MUST route through the platform ABI on all
platforms. Opaque-struct services MAY stay in the per-RTOS vendored
system layer; unifying them requires a *canonical* opaque layout plus a
`size_probe` `_Static_assert` (the pattern bare-metal/ThreadX net already
use), and is **explicitly out of scope** for the initial split. This is a
documented design boundary, not debt.

### D3. One bridge, platform-owned, category-gated

The `z_* → nros_platform_*` forwarder TU moves to a single
platform-layer artifact, consumed by both zenoh-pico and XRCE (retire the
duplicate `platform_aliases.c` copies). Emission is **per-category**:

- On platforms where the vendor system layer owns opaque services
  (FreeRTOS/ThreadX/Zephyr task+net), emit a **memory-only** (scalar)
  alias and leave task/net to the vendor — extends the existing
  `NROS_PLATFORM_ALIASES_SKIP_TASK` / net-gating precedent.
- The strong vendored scalar definitions (`z_malloc` etc.) are stripped
  behind a fork guard (e.g. `Z_FEATURE_NROS_PLATFORM_ALLOC`) so the alias
  wins instead of double-defining. The vendored zenoh-pico is already a
  fork carrying `ZENOH_*` accommodations; this is one more, scoped to the
  scalar functions.

> **Verified link-level reality (2026-06).** This bridge is **already
> active** on every platform where the alias TU compiles — POSIX,
> bare-metal, and **Zephyr** (disassembly: `z_malloc → jmp
> nros_platform_alloc`; the nano-ros Zephyr build does not even compile
> vendored `system/zephyr/system.c`), and presumably **ThreadX** (alias
> gated `!freertos` — verify with `objdump`). So the fork-strip above is
> needed **only on FreeRTOS**, where the alias TU is explicitly skipped and
> the vendored `system/freertos/system.c` `z_malloc → pvPortMalloc` is the
> genuine bypass. The static audit (issue 0006 / [phase-230] 230.0.1) could
> not see this routing; the real C-side worklist is much smaller than the
> 40-site grep implied — mostly FreeRTOS + the board-crate / Rust-allocator
> sites, not the RMW C path on Zephyr/ThreadX.

### D4. Allocator ownership + init contract

The platform port **owns and initializes the heap/pool** (ThreadX
`tx_byte_pool`, FreeRTOS heap region, Zephyr `k_heap`/`sys_heap`,
bare-metal `FreeListHeap`) before the first allocation. Init order is a
contract: *board/runtime platform-init → then any Rust or RMW allocation*.
The **C side** (zenoh-pico) always routes through `nros_platform_alloc`
(D3). The **Rust `#[global_allocator]`** is handled by D6 — it is a
binary-wide singleton that a framework may already own, so nano-ros
provides it only optionally.

### D6. The Rust global allocator is an optional, board-selected singleton

Rust permits **exactly one** `#[global_allocator]` per binary, and on many
targets the framework/runtime integration already installs it
(zephyr-lang-rust → `malloc`; esp-hal → `esp-alloc`; `std` → system). That
is a *different* concern from the platform ABI: the allocator is owned by
whoever owns the binary's boot/runtime, not necessarily by nano-ros.

So nano-ros provides its `#[global_allocator]` **optionally**, behind a
feature the **board/platform crate selects** (it knows whether a framework
already claims the slot):

| Platform | Framework allocator? | nano-ros `global-alloc` | Rust heap owner |
|---|---|---|---|
| bare-metal (MPS2 / STM32F4 / ESP32-bare) | no | **on** → wraps `nros_platform_alloc` | nano-ros |
| FreeRTOS (nros-board) | no | **on** → wraps `nros_platform_alloc` | nano-ros |
| ThreadX | no | **on** → wraps `nros_platform_alloc` | nano-ros |
| Zephyr | yes (zephyr-lang-rust) | **off** | framework |
| esp-hal | yes (esp-alloc) | **off** | framework |
| native / POSIX | yes (`std`) | **off** | `std` |

When **on**, nano-ros's allocator wraps `nros_platform_alloc`/`_dealloc`
→ one funnel for C + Rust. When **off**, nano-ros installs nothing and the
framework owns the Rust heap. The feature MUST be off wherever a framework
allocator is linked (a double `#[global_allocator]` is a compile error);
the board crate is the single point that enforces this, and a `just check`
assertion can guard against two providers landing together. nano-ros
**never patches a framework's allocator** to reroute it — it simply yields
the slot.

This means the platform ABI (`nros_platform_*`) is the layer boundary; the
Rust global allocator is a runtime-integration detail layered on top, not
part of the platform contract.

### D7. Heap stats are two-mode (follows D6)

- **nano-ros owns the allocator** (D6 on): a single funnel through
  `nros_platform_alloc` → instrument once → **exact, source-attributable**
  C+Rust total.
- **Framework owns the allocator** (D6 off): `nros_platform_alloc` counts
  the C (zenoh-pico) side; the **true unified total** comes from the
  platform-native heap query (Zephyr `sys_heap` runtime stats, FreeRTOS
  `xPortGetFreeHeapSize`, …) — both allocators share one kernel heap, so
  the native query is exact without nano-ros owning the Rust allocator.

Either way [issue 0006] is resolved: exact where nano-ros owns the
allocator, native-query elsewhere. The doc records which mode each platform
is in.

### D8. Enforcement gate

A **no-direct-kernel-call** lint (`scripts/check-no-direct-kernel-alloc.sh`,
wired into `just check`): fail the build if any non-port crate references
`pvPortMalloc`/`vPortFree`/`k_malloc`/`k_free`/`tx_byte_allocate`/
`tx_byte_release`/`heap_caps_*` (and, later, the thread/sync primitives)
outside the platform port that legitimately defines the `nros_platform_*`
symbol. Advisory while the inventory is migrated; flips to hard-fail when
the worklist is empty. This keeps the boundary from re-rotting.

## Decision

1. **C-side funnel first** (the starter): route zenoh-pico `z_malloc`/
   `z_free`/`z_realloc` through `nros_platform_alloc` on all RTOSes (D3) —
   guard the vendored defs, emit the memory-only alias.
2. **Rust global allocator is optional** (D6): nano-ros installs its own
   `#[global_allocator]` (wrapping `nros_platform_alloc`) only where no
   framework already claims the slot, board-selected. Never patch a
   framework allocator.
3. **Stats are two-mode** (D7): exact single-funnel where nano-ros owns the
   allocator; platform-native heap query where the framework owns it.
4. **Scalar services next** (sleep/clock/random): same C-side treatment.
5. **Opaque-struct services scoped out**: documented as the per-RTOS
   boundary; revisit only behind a canonical-layout + static-assert effort.
6. **One bridge**: dedupe zenoh/XRCE alias TUs into a platform-owned shim.
7. **Gate it**: the D8 lint prevents regression.

Work breakdown: [phase-230].

## Implementation status (2026-06)

Tracks what [phase-230] has landed against the decisions above.

| Decision | State | Where |
|---|---|---|
| D1 boundary invariant | **enforced** for the nros-owned surface | D8 gate hard (below) |
| D2 scalar vs opaque classification | **documented** | [platform-c-abi.md] §opaque-struct boundary |
| D3 one category-gated bridge | **alloc funnel complete across RTOSes** | POSIX/bare-metal/ThreadX/Zephyr via the alias TU; FreeRTOS via 1c (fork guard + memory-only alias). Sleep/clock/random (Wave 2) + the zpico/xrce alias-TU dedup (Wave 3) remain |
| D4 alloc ownership + init contract | **landed** (FreeRTOS/ThreadX/POSIX/esp) | `nros-platform-*/src/platform.c` |
| D6 optional Rust global allocator | **landed** | `nros-platform-api` `global-allocator` feature → `nros_platform_alloc`; off where a framework owns the slot (Zephyr/esp-hal/std) |
| D7 two-mode heap stats | **landed**; closes [issue 0006] | canonical `nros_platform_heap_used_bytes`/`_total_bytes` (ABI) + per-port impls + bare-metal `FreeListHeap` stats |
| D8 enforcement gate | **HARD** by default | `scripts/check-no-direct-kernel-alloc.sh` (`NROS_ALLOC_GATE_HARD=1` default) |

**nros-owned surface — DONE.** The Rust `#[global_allocator]`s
(nros-c/nros-cpp), the C-API inline platform headers
(`nros-c/include/nros/platform/{freertos,zephyr}.h`), and the board
task-context allocations (`nros-board-freertos`, `nros-board-orin-spe`) all
route through `nros_platform_alloc`/`_dealloc`. The ThreadX board
`tx_byte_allocate` sites (thread stack, NetX pools) are the vendored
TASK/NET opaque-struct services (D2) and stay direct on a documented
lint allowlist.

**Vendored alloc funnel — LANDED across all RTOSes.** zenoh-pico's C
`z_malloc`/`z_free`/`z_realloc` now resolve to `nros_platform_alloc` on every
RTOS: **FreeRTOS** via 1c (fork guard `#ifndef Z_FEATURE_NROS_PLATFORM_ALLOC`
in `system/freertos/system.c` + the memory-only alias, coupled to
`CARGO_FEATURE_PLATFORM_ALIASES`); **ThreadX** and **Zephyr** were already
funneled by the alias TU (they never compiled a vendored `z_malloc` —
generic `system/common` / a Zephyr CMake glob that omits `system/zephyr/
system.c`), so 1f only removed ThreadX's dead weak forwarder. Static link
proof was done locally for FreeRTOS (guarded archive + alias, clean firmware
link) and ThreadX (objdump `z_malloc → nros_platform_alloc`); Zephyr is
confirmed by the CMake source wiring. Runtime confirmation rides the per-RTOS
CI lanes. **Sleep + random** are likewise funneled (Wave 2): FreeRTOS guards
the vendored `z_sleep_*`/`z_random_*` (`Z_FEATURE_NROS_PLATFORM_SLEEP`/
`_RANDOM`) and the FreeRTOS alias goes memory-only → scalar-only; ThreadX/
Zephyr already forward them via the full alias TU. **Remaining vendored
work:** `z_clock`/`z_time` (deferred — `z_clock_t` is a non-portable
`TickType_t`) and the zpico/xrce alias-TU dedup (Wave 3) — both out of D8's
scope (the lint excludes the vendored submodule).

## Open questions

- **Fork maintenance.** Stripping vendored scalar defs behind a guard adds
  rebase cost on upstream zenoh-pico bumps. Acceptable vs the alternative
  (RTOS-native pool *query* for stats only, no unification — [issue 0006]
  Option B)? The query path gets stats without touching the layer split,
  but leaves the bypass (and dead code) in place. This RFC chooses
  unification because the split is the stated goal; revisit if fork churn
  proves heavy.
- **Do opaque services ever move?** Net is the strongest candidate (a
  `nros_platform_net_*` surface already exists, 29 symbols). Threads/sync
  are the least likely (deep struct coupling). Left open.
- **Zephyr port crate.** `nros-platform-zephyr` *does* ship
  `nros_platform_alloc` (k_heap-backed) as a Zephyr CMake module; the gap
  is link-line presence + who installs the Rust global allocator (next
  point), not a missing port.

- **RTOS-owned Rust global allocator — RESOLVED by D6/D7.** Earlier this
  was an open question (do we shadow/patch the framework allocator on
  Zephyr/esp-hal?). Resolved: nano-ros's `#[global_allocator]` is optional
  and board-selected (D6); where a framework owns the slot, nano-ros yields
  it and the true heap total comes from the native query (D7). No framework
  allocator is ever patched.

- **`global-alloc` provider crate + double-provider guard.** Mechanics left
  to [phase-230]: which crate hosts the optional `#[global_allocator]`
  static (candidate: a small `nros-alloc` gated by `nros-global-alloc`,
  installed by the board), and the `just check` assertion that at most one
  global-allocator provider is on the link line.

## Changelog

- 2026-06: Initial draft. Captures the split-enforcement decision, the
  scalar/opaque-struct ABI classification, and the alloc-first plan that
  subsumes [issue 0006].
- 2026-06: D6/D7 — the Rust `#[global_allocator]` is an optional,
  board-selected singleton (nano-ros provides it only where no framework
  claims the slot; never patch a framework allocator), and heap stats are
  two-mode (exact single-funnel when owned; native heap query otherwise).
  Resolves the RTOS-owned-allocator open question; drops the
  zephyr-lang-rust patch from the Zephyr slice. Renumbered the enforcement
  gate to D8.
- 2026-06: Added Implementation status. Wave 1 landed the nros-owned alloc
  surface (D4/D6/D7) + flipped the D8 gate hard; [issue 0006] closed. The
  vendored zenoh-pico `z_*` scalar funnel (D3 on RTOS) + the alias-TU dedup
  (D3 "one bridge") are the CI-relink-gated remainder. D2 boundary recorded
  in [platform-c-abi.md].

## See also

- RFC-0006 [portable-rmw-platform-interface](0006-portable-rmw-platform-interface.md)
  — C-ABI-canonical meta-decision; platform = free `extern "C"` symbols,
  link-time resolution; L0/L1/L2 ladder.
- RFC-0001 [architecture-overview](0001-architecture-overview.md) — layer map.
- RFC-0003 [rtos-integration-pattern](0003-rtos-integration-pattern.md) —
  per-RTOS adapter pattern; §9 std/alloc-per-platform policy.
- [platform-c-abi.md](../../book/src/internals/platform-c-abi.md) — the
  symbol contract + port-authoring guide + drift gate.
- [issue 0006](../issues/archived/0006-rtos-dual-heap.md) — the dual-heap / unified
  stats item this RFC's alloc unification resolves.
- [phase-230](../roadmap/phase-230-platform-layer-split.md) — work breakdown.

[platform-c-abi.md]: ../../book/src/internals/platform-c-abi.md
[issue 0006]: ../issues/0006-rtos-dual-heap.md
