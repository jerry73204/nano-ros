---
rfc: 0042
title: "Platform & build determinism — one canonical interface, capability-driven config, deterministic linking"
status: Stable
since: 2026-06
last-reviewed: 2026-06-15
status-note: "STABLE (2026-06-15). D1/D2/D4 landed (phase-241.B/C, phase-243); D3 landed (single-runtime + the .init_array registration ctor, phase-249 P4b; --allow-multiple-definition removed on the C/C++ link for Linux/BSD). Residual tracked in phase-249: the macOS cyclonedds --allow-multiple-definition (needs a macOS run). The full link-closure validator extension is won't-do — subsumed by the linker once the flag is gone, infeasible at archive level (single-runtime = one archive). Neither blocks Stable (the dup-symbol gate already guards the ODR hazard)."
implements-tracked-by: [archived/phase-241-platform-build-determinism, archived/phase-243-platform-abi-unification, phase-249-one-registration-trigger]
supersedes: []
superseded-by: null
---

# RFC-0042 — Platform & build determinism

## Summary

A recurring class of build failures — libc/std header clashes (#27, #36, #38),
ld single-pass undefined-symbol races (#20), and silent capability mismatches —
all share one root: **determinism in the C/C++/Rust build is enforced by
convention and hand-special-casing, not by structure.** Each new board, cpp
example, or platform-header edit can land green and break a different,
un-gated combination days later (only an on-demand e2e build exercises it).

This RFC makes the platform/build contract *structural* along four pillars:

1. **One canonical platform interface.** A single `<nros/platform.h>` C ABI is
   THE interface (per RFC-0006); the CFFI header stops being a second header of
   the same name; the `malloc`/`free`-over-`alloc`/`dealloc` shim is defined
   once, not copied across five headers.
2. **Capability-driven configuration.** `nros-board.toml` becomes the single
   source of truth for board capabilities (`has_heap`, `has_atomics`,
   `has_threads`, …); one generator lowers them to *every* downstream knob
   (cargo features, cmake `-D`, capability macros, include precedence). Drift
   across the ≥5 sites that today each name the platform becomes impossible.
3. **Deterministic linking.** One registration mechanism for all platforms
   (a generated explicit backend-register table), a generated link manifest that
   fixes archive order + the whole-archive set, and the removal of the
   `--allow-multiple-definition` / `-u <sym>` special-cases that today paper over
   ordering bugs. Linking becomes data, not lore.
4. **Merge-time gate.** A platform × language compile+link matrix runs on every
   PR, so the whole class is caught at merge instead of in a later e2e dispatch.

It **amends** RFC-0006 (canonical C ABI — now enforced, single header),
RFC-0031 (RMW select/lower — now validated against the manifest), and references
RFC-0034 (allocator funnel — kept) and RFC-0035 (vtable slot ABI — kept). It
**depends on** RFC-0012 (board crates) and RFC-0014 (`nros setup` / sdk-index).

## Motivation — determinism by convention fails

Evidence (all real, all recent):

| Symptom | Root | Convention that failed |
| --- | --- | --- |
| #27 NuttX C `time.h`/atomics | newlib (`__rtems__`-gated) vs NuttX libc both on path | RTOS-sysroot-wins include precedence set per-entrypoint |
| #36 NuttX C++ `div_t` | libstdc++ `<cstdlib>` `#include_next` reaches newlib `stdlib.h` after NuttX's | same precedence, re-derived for the cpp entrypoint |
| #38 ThreadX RV64 C++ `malloc/free` | `baremetal.h` defaults `NROS_NO_DYNAMIC_MEMORY`; heap-capable board must remember a `-D` | default-deny capability, opt-in per board |
| #20 ThreadX-linux C++ Cyclone | ld single-pass: whole-archive RMW references a symbol defined in an earlier-scanned archive | hand-injected `-u nros_rmw_cffi_register_named` for that one combo |

Each was fixed point-wise. The architecture has good *pieces* — the unified
allocator funnel (RFC-0034 D6), the frozen 34-slot vtable (RFC-0035), the
distributed-slice registration concept — but the *contracts between them* are
upheld by comments and per-combination workarounds:

- **Two `<nros/platform.h>` headers** (`nros-c/include/...` canonical `malloc`/
  `free`; `nros-platform-cffi/include/...` `alloc`/`dealloc`) resolve by
  `-I`-before-`-isystem` order, not by design. The shim that bridges them is
  copy-pasted in `platform/{posix,zephyr,freertos,baremetal}.h` **and**
  `nros-platform-cffi/platform.h` (5×).
- **Capability macros** (`NROS_PLATFORM_HAS_MALLOC`, `NROS_NO_DYNAMIC_MEMORY`,
  `NROS_PLATFORM_HAS_ATOMICS`, `NROS_PLATFORM_HAS_MUTEX`) are default-deny and
  set in ~10 scattered cmake/board/build.rs sites; platform identity is repeated
  in ≥5 places (board.toml, sdk-index, cargo feature, cmake var + module file,
  platform.h macro) with no cross-check.
- **Two registration paths**: `linkme` distributed-slice (`RMW_INIT_ENTRIES`) on
  hosted targets vs weak `nros_app_register_backends` + a cmake-generated strong
  override on bare-metal (linkme drops `target_os = "none"`). Whole-archive +
  "platform shim must link *after* RMW" (ld single-pass) is hand-maintained;
  `--allow-multiple-definition` hides duplicate-symbol smells.
- **No merge gate**: the cpp heap containers (pulled in by *every* generated
  message type) compile only as a side-effect of the on-demand e2e build. That
  latency is why the class "recurs whenever another developer edits."

## Non-goals

- Re-freezing or re-numbering the vtable slot table — RFC-0035 stands.
- Changing the unified-allocator funnel — RFC-0034 D6 stands; this RFC just makes
  the canonical `malloc`/`free` surface over it single-sourced.
- Changing the RMW *declaration* model (system.toml) — RFC-0031 stands; this RFC
  adds *validation* that lowering matches the declaration.
- Runtime plugin loading — registration stays link-time.

## Design

### D1 — One canonical platform interface

> **Status: STABLE (landed 2026-06-13).** One canonical `<nros/platform.h>` owned
> by `nros-platform-api`; the legacy `nros-c` header + its per-RTOS sub-headers are
> deleted on main. Delivered by phase-241.B + phase-243 (archived).

> **Direction (corrected after the wave-B investigation, 2026-06-12).** There are
> today **two** files named `nros/platform.h` sharing the include guard
> `NROS_PLATFORM_H`, so they can never coexist in a TU — whichever is first on the
> `-I` path wins and ABI-poisons the other. They are **layers, not duplicates**:
> - **A** = `nros-c/include/nros/platform.h` — legacy: compile-time dispatch +
>   capability macros (`NROS_PLATFORM_HAS_*`, `NROS_NO_DYNAMIC_MEMORY`,
>   `nros_mutex_t`) + an *ns* clock + atomics + a typed mutex + malloc/free
>   *prototypes*.
> - **B** = `nros-platform-cffi/include/nros/platform.h` — the canonical **full**
>   C ABI (≈39 symbols: *ms/us* clock, `alloc`/`realloc`/`dealloc` + heap stats,
>   tasks, mutex, condvar, wake, critical section, logging), hand-mirrored by
>   `nros-platform-cffi/src/lib.rs` and guarded by `c_stub_platform.rs`.
>
> Every real implementor + consumer (all RTOS ports, xrce, cyclone, zpico, the
> smoke TUs) already resolves to **B** — and xrce/cyclone call **B-only** symbols
> (`clock_ms`, `time_now_ms`, `alloc`/`dealloc`). Only the nros-cpp heap headers
> (reached via nros-c's INTERFACE include order) and the 241.A host gate resolve
> to **A**, and they use only the malloc/free overlap. So **B is canonical and A
> is legacy** — the *reverse* of this section's original lean.

- **B's surface is THE canonical `<nros/platform.h>`.** A is retired: its
  divergent legacy surface (ns clock, typed mutex, atomics) is removed after
  confirming no live C consumers; its still-needed pieces (the capability macros
  + the malloc/free *consumer view*) move into the canonical header.
- **`nros-platform-api` owns the one canonical header** (`include/nros/platform.h`)
  + the single malloc/free shim + the capability macros. `nros-c` and
  `nros-platform-cffi` both depend on / re-export it, so neither package's
  consumers need the other's include dir — this breaks today's nros-c↔cffi header
  tangle (each currently puts the other on the path inconsistently). The Rust
  extern mirror (`lib.rs`) and `c_stub_platform.rs` parity test move with it.
- The `malloc`→`alloc` / `free`→`dealloc` shim is defined **once** in the
  canonical header, gated by the capability macro, replacing the 5 divergent
  copies (4 forwarders + posix's outlier that calls libc directly, bypassing the
  RFC-0034 D6 funnel — fixed to forward). A platform with a heap exposes the
  canonical `malloc`/`free`; a heap-less one does not — the capability gate
  (preserving the 241.A #38 semantics) stays.
- **Include-precedence rule (normative):** when an RTOS ships its own libc, its
  sysroot headers win over the toolchain's bare newlib/picolibc for *all*
  entrypoints. This is implemented once (see D3's helper), not re-derived per
  compile site. For C++, the RTOS C++ wrapper dir (e.g. NuttX `include/cxx`)
  precedes libstdc++ so `<cstdlib>`'s `#include_next` cannot reach the toolchain
  libc (the #36 mechanism).
- A `static_assert`/CI parity check guarantees the canonical surface and any
  generated mirror stay signature-identical.

### D2 — Capability-driven configuration (single source of truth)

> **Status: STABLE (landed 2026-06-13).** `[board.capabilities]` (heap/atomics/
> threads) is the SSoT; `cmake/NanoRosCapabilities.cmake` lowers it to the
> `-DNROS_PLATFORM_HAS_*` defines; a merge-gate lint requires every in-tree board
> to declare it. Delivered by phase-241.C.

- `nros-board.toml` gains a `[board.capabilities]` block — the SSoT:
  ```toml
  [board.capabilities]
  heap     = true     # has a usable allocator (drives malloc/free + NO_DYNAMIC_MEMORY)
  atomics  = true     # NROS_PLATFORM_HAS_ATOMICS
  threads  = true     # NROS_FEATURE_THREADS / HAS_MUTEX
  libc     = "nuttx"  # rtos | newlib | picolibc | host — drives include precedence
  ```
- One generator (in the codegen/cmake glue) lowers capabilities to **every**
  downstream knob: cargo features, cmake `-D NROS_PLATFORM_HAS_*`, the capability
  macros, and the include-precedence selection. The ~10 scattered `EXTRA_DEFINES`
  / per-header defaults are removed; they read the generated values.
- Capability defaults become **deny-only-when-known-absent**: a board states what
  it has; the build never silently drops a symbol a linked consumer needs.
- Platform identity is named **once** (board.toml); cmake/cargo/sdk-index consume
  the descriptor (extends the Phase 195.C board-descriptor mechanism) instead of
  re-declaring it.

### D3 — Deterministic linking

> **Status: STABLE (2026-06-15).** Bullet 3 (single-shared-runtime, no dup) landed;
> bullet 1 (registration trigger = the `.init_array` ctor — **§D3.3**) landed in
> [phase-249](../roadmap/archived/phase-249-one-registration-trigger.md) P4b (linkme deleted);
> `--allow-multiple-definition` removed on the C/C++ link (Linux/BSD). Residual:
> the macOS cyclonedds flag (needs a macOS run) is tracked in phase-249. The full
> link-closure validator extension is **won't-do** — subsumed by the linker once the
> flag is gone (undefined owned ref → link error; dup → multiple-definition error),
> and infeasible at archive level (single-runtime = one archive; ~737 legit undefined
> owned refs). Neither blocks Stable (the dup-symbol gate guards ODR).

- **One registration trigger.** *(Mechanism settled 2026-06-15 — see §D3.3.)* The
  three coexisting self-register mechanisms (linkme distributed-slice, `.init_array`
  ctor, explicit call) collapse to **two**: the backend self-registers via an
  **`.init_array` ctor** on **hosted** (Rust + C/C++ — the loader fires it before
  `main`), and via an **explicit `nros_rmw_<x>::register()` call** on **embedded**
  (RTOS / bare-metal, where `.init_array` is not reliably walked). **`linkme` is
  deleted** — the earlier "codegen emits an explicit backend-register *table* on all
  platforms" framing is superseded by the ctor (the table is not needed; the ctor +
  the cmake stub are the triggers). `--allow-multiple-definition` and the
  `linkme-register` DUPCHECK feature are gone (single-runtime removed the multi-archive
  dup that made the ctor unsafe — which is *why* the ctor is now safe everywhere).
- **Generated link manifest.** The codegen/cmake glue emits the deterministic
  link line for the binary: which archives are whole-archived, and the archive
  order that satisfies ld single-pass (platform shim after RMW, message libs
  before the FFI glue, …). The ordering rules move from comments into generated
  data.
- **No papering-over.** `--allow-multiple-definition` and the per-combo
  `-u <symbol>` injections (e.g. #20) are removed; the manifest's ordering +
  whole-archive set make extraction deterministic, so duplicate/undefined symbols
  surface as real errors, not silently-resolved ones.
- **Link-closure validator — folded into the linker (2026-06-15).** The original
  goal: a generation-time pass asserting every FFI-glue-referenced symbol is satisfied
  by exactly one manifest entry. Once `--allow-multiple-definition` is gone, `ld` itself
  *is* that pass — an undefined owned ref fails the link, a real duplicate is a
  multiple-definition error. A separate archive-level check is infeasible (single-runtime
  bundles to one `libnros_c.a` carrying ~737 legit undefined owned refs that resolve at
  the final link). The cheaper pre-link ODR signal stays as `staticlib_duplicate_symbols`.
- The unified allocator (RFC-0034) and vtable ABI (RFC-0035) are unchanged; D3
  only makes their *linkage* deterministic.

#### D3 implementation — single shared runtime (2026-06-13, Stable)

The "no papering-over" goal landed via a **single-runtime** model, NOT a generated
whole-archive manifest. Detail + work items: `docs/roadmap/archived/phase-241-d3-single-runtime.md`.

- **One Rust staticlib per binary.** The FFI crate *is* the umbrella: a C binary
  links only `libnros_c.a`, a C++ binary only `libnros_cpp.a`. Each bundles the
  cffi shim **and** the selected Rust RMW backend (`nros-rmw-zenoh` /
  `nros-rmw-xrce-cffi`) as an rlib dependency, force-linked via the umbrella's
  `rmw_backend` module. One cargo build ⇒ `std`/`compiler-builtins` monomorphized
  **once** ⇒ no `EMPTY_PANIC`/`rust_eh_personality` duplicates ⇒ the flag is gone
  for real. (Reverts the Phase 134.fix backend-dep drop, whose multi-staticlib
  hazard cannot occur with one archive.)
- **Not whole-archive.** Whole-archiving a Rust staticlib force-includes its
  `compiler-builtins` intrinsics (`__popcountdi2`, `__mulsc3`, …) which collide
  with the system `libgcc.a` — a duplicate the old flag also masked. So the
  umbrella is linked **normally**: the C surface is pulled lazily by each binary's
  own references, and the backend is registered by the existing NanoRosLink.cmake
  `nros_app_register_backends()` stub (it calls `nros_rmw_<x>_register`, now defined
  in the umbrella). The single registration path of D3's first bullet is satisfied
  by that stub; the `linkme` distributed-slice remains the pure-Rust-binary path.
- **Cyclone** is a C++ CMake lib (not a Rust staticlib, no Rust `std`), linked
  separately + whole-archived; `libstdc++` is wired for **all** languages incl. C.
- **Superseded:** the slice-4 `nros-rmw-cffi-provider` archive + `external-registry`
  feature (a workaround for the *multi*-staticlib cffi `REGISTRY` duplicate) are
  **retired** — one archive ⇒ one `REGISTRY`, defined with a plain `#[no_mangle]`.
- **Retained:** the standalone `nros-rmw-{zenoh,xrce}-cffi-staticlib` crates stay —
  the **Zephyr** build (`zephyr/CMakeLists.txt`) links the cargo-built staticlib
  directly (its own west link model, not the cmake umbrella), and the archive-symbol
  / header-parity tests consume them.
- **Remaining D3 goals (bullets 1+2) — [issue 0062](../issues/archived/0062-d3-completion-one-registration-path-and-link-manifest.md).**
  Single-runtime delivers bullet 3 (no dup). Bullet 1 (one registration trigger) is
  settled as the `.init_array` ctor — see §D3.3. Bullet 2 (generated link manifest —
  the RMW dispatch table, incl. Cyclone's `+libstdc++`) rides on this foundation
  (emit it as data from `resolve_rmw`). The weak `nros_app_register_backends` default
  is already deleted (phase-249 P4a, closes
  [issue 0050](../issues/archived/0050-weak-symbol-audit-and-checkers.md) W3.1).

#### D3.3 — Registration trigger: the `.init_array` ctor (decision 2026-06-15)

Bullet 1 ("one registration path") is **mechanism-settled**: the backend's
self-register is an **`.init_array` ctor**, not the `linkme` distributed-slice and
not a codegen-emitted explicit table.

- **Why ctor over linkme (UX + maintainability).** linkme is an external crate that
  centralises the section discipline, but its cost accreted: a 12-OS allow-list
  duplicated ×3 (each new hosted OS must be vetted + added; Phase 142 had to *drop*
  `target_os="none"` because cortex-m's link script lacks the `__start_/__stop_`
  anchors and the slice iterator **hung**), a `linkme-register` feature to suppress
  the multi-archive **DUPCHECK** collision (Phase 134.fix), the `linkm2_` section-name
  coupling + Mach-O/COFF variants, and a lazy runtime walk. The `.init_array` ctor is
  the std crt mechanism: **no external dep, no DUPCHECK** (independent ctors, idempotent
  `register`), **no per-OS allow-list** (hosted vs `target_os="none"`), broader hosted
  coverage. Single-runtime (bullet 3) eliminated the multi-archive dup that made the
  ctor unsafe — which is *why* the ctor is now safe everywhere — so consolidating **on**
  the ctor (and deleting linkme), rather than re-enabling linkme, is the cleaner end.

- **The two triggers (D7-compatible).** **Hosted** (Rust + C/C++): the backend ships an
  `.init_array` ctor (Rust `#[used] link_section=".init_array"` / `__DATA,__mod_init_func`;
  C `__attribute__((constructor))`), emitted by the `nros_rmw_register_backend!` (Rust)
  and `NROS_RMW_REGISTER_BACKEND` (C) macros, gated `not(target_os="none")`. The native
  app keeps its `#[used] __FORCE_LINK_*` anchor (phase-244 D7 "Shape B" — no `register()`
  call in app source; the anchor keeps the backend closure + its ctor past DCE). The
  loader fires the ctor before `main`, so no runtime walk is needed.
  **Embedded** (RTOS / bare-metal, `target_os="none"` + nuttx/zephyr/esp): the ctor is a
  no-op (`.init_array` not reliably walked); the board / typed carrier calls
  `nros_rmw_<x>::register()` **explicitly** (phase-249 P1). The cmake `nano_ros_link_rmw`
  strong stub stays as the deterministic C/C++-via-cmake trigger (phase-249 P2b/P4a).

- **Deleted by this decision.** the `linkme` crate dep + the `linkme-register` feature
  (cffi + the backend crates + passthroughs); the `RMW_INIT_ENTRIES` distributed-slice +
  the no-op stub + the 12-OS allow-list (×3); `nros_rmw_cffi_walk_init_section` + its 3
  call sites (`Executor::open` / `open_multi` / `nros::init`). **Kept:** the
  `#[used] __FORCE_LINK_*` anchors, the embedded explicit calls, the cmake stub.

- **Implemented by [phase-249](../roadmap/archived/phase-249-one-registration-trigger.md) P4b**
  (which redefines from "delete linkme → explicit table" to "consolidate to ctor").

### D4 — Merge-time compile + link gate

> **Status: STABLE (host tier landed 2026-06-13).** The platform×language compile
> gate (`platform_header_matrix.rs` + `platform-header-gate.yml`) is a hard PR gate
> across all platform targets; the dup-symbol validator (`staticlib_duplicate_
> symbols`) is wired into `check.yml`. Residual: the cross-toolchain `.c`-TU tier
> (#27/#36) stays e2e-covered (phase-241.A cross tier). Delivered by phase-241.A.

- A platform × language matrix runs on **every PR** (not just on-demand e2e):
  cross-`check`/compile one representative TU per cell that exercises the
  fragile surface — a generated message type (pulls in `HeapString`/
  `HeapSequence`) plus a minimal entry that forces backend registration + the
  final link. Mirrors the existing `core-libs` cross-`check` lane.
- Cells: {posix, zephyr, freertos, nuttx, threadx, bare-metal, esp-idf} ×
  {C, C++, Rust} where the combination is supported (driven by the board
  capability matrix from D2).
- This is the **first** thing to land (it is the safety net for D1–D3's
  migration) and the cheapest; see the phase doc's Wave A.

## Compatibility & migration

- D1/D2/D3 are source-and-build-tooling changes; the runtime vtable ABI
  (RFC-0035) and allocator funnel (RFC-0034) are untouched, so existing binaries'
  behavior is unchanged once they build.
- Migration is staged so the gate (D4) lands first and guards the rest (see
  phase-241). Each subsequent wave is independently revertible.
- `nros-board.toml` capability block is additive; boards without it get
  conservative inferred defaults during migration, with a lint that flags
  boards still relying on inference.

## Open questions

- **Q1 — RESOLVED (2026-06-12, reversed).** The wave-B investigation showed B
  (cffi) is the canonical full ABI everyone uses and A (nros-c) is legacy — the
  opposite of the original lean. **Decision: `nros-platform-api` owns the single
  canonical `<nros/platform.h>` (= B's surface + capability macros + one shim);
  nros-c and nros-platform-cffi re-export it; A's legacy-only surface is retired.**
  No new generator in wave B (header stays hand-mirrored against `lib.rs`, guarded
  by `c_stub_platform.rs`).
- **Q2 — Link-manifest owner.** Does the manifest live in the cmake glue, in
  `nros` codegen (`nros plan`/`nros sync`), or both? *Lean: codegen emits a
  manifest file; cmake + build.rs consume it — one producer, two consumers,
  matching the existing `APP_FFI_LIBS_FILE` shape.*
- **Q3 — Capability inference fallback.** During migration, how are
  capabilities inferred for a board lacking the block — from `platform`? *Lean:
  a platform→default-capabilities table, with a deprecation lint.*
- **Q4 — Hosted registration.** Keep `linkme` as the hosted generator backend, or
  unify on the explicit table everywhere from day one? *Lean: explicit table
  everywhere; retire linkme as a contract (may stay as an internal optimization).*

## Relationship to existing RFCs

- **Amends RFC-0006** (portable RMW/platform interface): the "C ABI is canonical"
  stance becomes structurally enforced — one header, one shim.
- **Amends RFC-0031** (RMW selection/lowering): adds validation that the lowered
  feature/`-D` matches the system.toml declaration, via the link manifest.
- **References RFC-0034** (platform layer split): allocator funnel kept; canonical
  malloc/free single-sourced over it.
- **References RFC-0035** (vtable ABI): slot table kept; its linkage made
  deterministic.
- **Depends on RFC-0012** (board crates) and **RFC-0014** (sdk-index): the
  capability block extends the board descriptor those define.
