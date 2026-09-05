---
rfc: 0045
title: "Unified boot-config resolution across languages and platforms"
status: Draft
since: 2026-06
last-reviewed: 2026-06
implements-tracked-by: []
supersedes: []
superseded-by: null
---

# RFC-0045 — Unified boot-config resolution across languages and platforms

## Summary

A node's **boot config** — its ROS graph `node_name`, RMW `locator`, and `domain_id`
(plus `namespace`) — is today assembled in three different places (Rust / C / C++) from
four different sources (runtime env, baked `Config`, compile-time `option_env!`, or — on
NuttX — nothing at all). The same `system.toml` / `[deploy.*]` / launch input therefore
produces *different* runtime identity depending on target and language. Issue #98 (the node
name shows `/node`/`/nros_app` instead of the launch-declared name) is one visible symptom;
the C path doesn't even thread the user's node name (it sends a PID-based session name), and
C++ defaults to `"nros_cpp"`.

The fix is small in shape because the **sink is already unified**: all three languages funnel
into one struct + function, `RmwConfig → CffiRmw::open → CffiSession::open`. What diverges is
everything *above* the sink. This RFC defines a single **resolver** with one precedence rule
that feeds that existing sink, reachable from all three languages, and a single **embedded
bake site** designed so a future post-link config-patch tool (and, later, a build-time plan
image) drop in without changing the resolver or its call-sites.

This RFC is the design backing issue [#101](../issues/archived/0101-board-boot-config-not-unified.md)
(and the durable fix for [#98](../issues/archived/0098-nros-main-ignores-component-node-name.md)). It
extends RFC-0004 (the configuration model) on one axis: *how* the configured identity reaches
the running session, uniformly.

## Problem

The boot-config assembly paths (verified 2026-06-26):

| Lang | Builds `RmwConfig` at | Source of identity | `node_name` today |
| --- | --- | --- | --- |
| Rust | board → `ExecutorConfig` → `Executor::open` (`nros-node/.../spin.rs:101`) | board-dependent (4 mechanisms below) | hardcoded / env / `option_env!` |
| C | `nros_support_init` → `open_session` (`nros/src/lib.rs:482`) | `nros_support_init(locator, domain)` params | **PID-based session name — user node name never flows** |
| C++ | `nros_cpp_init` → `ExecutorConfig` (`nros-cpp/src/lib.rs:475`) | `NROS_ENTRY_*` macros / init params | defaults `"nros_cpp"` |

The four Rust mechanisms (board audit): runtime `from_env` + `DeployOverlay` (POSIX/Native —
the only ones that honor the launch name); baked `Config` with hardcoded `.node_name("nros_app")`
(stm32f4, mps2 bare/FreeRTOS, threadx); compile-time `option_env!` (Zephyr, ESP32, RTIC,
Embassy); and the trait-default drop on NuttX, which has no `run_with_deploy` override so the
overlay (locator/domain/node_name) is silently inert.

Consequences: `ros2 node list` is correct on 2 of ~10 boards; `[deploy.nuttx*]` is dead;
locator/domain override mechanism is per-board lore; the C/C++ node name is wrong on every
target. The orchestration IR already resolves the identity language-agnostically — only the
*delivery to the session* is fragmented.

The one thing already unified — the sink:

```
RmwConfig { locator, mode, domain_id, node_name, namespace, properties }
    → CffiRmw::open(&cfg)        (nros-rmw-cffi/src/lib.rs:2957)
    → CffiSession::open(locator, mode, domain_id, node_name)
```

## Design

### Precedence model (A): env override on hosted, overlay as the source, compiled default last

Per field, independently:

```
  env var (hosted only, when set)         e.g. NROS_LOCATOR / ROS_DOMAIN_ID / NROS_NODE_NAME
        ↓  overrides
  baked overlay  (system.toml / [deploy.*] / launch / Kconfig)
        ↓  falls back to
  board compiled default  ("nros_app", multicast/empty locator, domain 0)
```

Rationale (UX + maintainability):

- **One mental model:** `system.toml`/launch is the source of truth; env tweaks it on native
  for fast iteration and tests. Native and embedded behave identically for a given config.
- **Smallest surface:** the env layer is a thin, std-gated, hosted-only wrapper that already
  exists (`ExecutorConfig::from_env`); embedded just passes the baked overlay through. No
  per-RTOS `getenv` shim (rejected alternative B-env), and no break of the native per-run
  `NROS_LOCATOR` workflow that every native test + the zenohd harness relies on (rejected
  alternative bake-only).

### The resolver lives in `nros-node`, on `ExecutorConfig`, with plain-field input

`nros-node` (home of `ExecutorConfig`) and `nros-platform` (home of `DeployOverlay`) do not
depend on each other — they meet only through `nros-platform-api`. So the resolver takes a
plain-field input, not the `DeployOverlay` type, sidestepping the cycle. Everyone already
depends on `nros-node`.

```rust
// nros-node — the single precedence implementation
pub struct BootConfig<'a> {          // session-identity subset; all optional = "unset"
    pub node_name: Option<&'a str>,
    pub locator:   Option<&'a str>,
    pub domain_id: Option<u32>,
    pub namespace: Option<&'a str>,
}

impl<'a> ExecutorConfig<'a> {
    /// Resolve precedence-model-A into a ready ExecutorConfig.
    pub fn resolve(baked: BootConfig<'a>) -> ExecutorConfig<'a> { /* … */ }
    /// …with an environment rung on top, supplied by the caller.
    pub fn resolve_with(baked: BootConfig<'a>, env: Option<EnvRung<'a>>) -> ExecutorConfig<'a> { /* … */ }
}
```

> **Amended by issue 0687 (2026-08-19).** The signature above shipped as
> `resolve(baked, hosted_env: bool)`, with the core reading `std::env::var`
> itself when the flag was set. The flag turned out to be a compile-time
> constant at every call site in the tree — `true` at one board plus the two FFI
> entries, `false` at the other five boards — and it was the last thing keeping
> a process environment in `nros-node`. It is now a VALUE: the caller passes an
> `EnvRung` of already-resolved fields, and `nros::env::resolve_hosted` is the
> hosted edge that fills one from the environment. The precedence model below is
> unchanged; only who reads the variables moved.

Three thin call-sites map their source into `BootConfig`, then call `resolve`:

- **Rust boards:** unpack `DeployOverlay { node_name, locator, domain_id }` → `BootConfig` →
  `ExecutorConfig::resolve(..)` (hosted boards: `nros::env::resolve_hosted(..)`). One line per board, replacing the
  current hardcoded `.node_name("nros_app")`. This is what makes #98 land on *every* board.
  NuttX gains a `run_with_deploy` override (it has none today) so the overlay stops being
  dropped.
- **C / C++** — via the shared `.nros_boot_config` blob (see below). Confirmed empirically
  (2026-06-27) to be the same #98-class defect: a native C++ entry whose launch names `talker`
  shows **`/nros_cpp`** in `ros2 node list`, because `create_node("talker")` reuses the primary
  session and the graph shows the session/init default. C defaults that to `nros_{pid}`.

`DeployOverlay` keeps its board-network fields (`ip`/`gateway`/`netmask`/`transport`) in
`nros-platform`; only its session triple maps into `BootConfig` at the board boot site.

### C/C++ reuse the same blob (Option A, decided 2026-06-27)

`BakedBootConfig` is `repr(C)` + magic-tagged for exactly this. C/C++ entries do **not** build a
Rust `BootConfig`; they reuse the bake site directly:

- A C header mirror — `packages/api/nros-c/include/nros/boot_config.h` — declares the same
  `repr(C)` layout + `NRBC` magic + `BOOT_SET_*` bits + an inline reader
  `nros_boot_config_node_name(const struct nros_baked_boot_config*) -> const char*`. A
  `sizeof`/offset assertion guards it against drift from `nros-platform-api`.
- The C/C++ codegen entry emitters (`emit_c.rs` / `emit_cpp.rs`) emit the SAME
  `NROS_BOOT_CONFIG` static into `.nros_boot_config` (they already know the launch node name),
  using `__attribute__((section(...), used))`.
- The generated C entry passes `nros_boot_config_node_name(&NROS_BOOT_CONFIG)` as the
  `session_name` to `nros_support_init_named` (replacing the `NULL` → PID default); the C++ board
  adapters (`main.hpp::run_components`) pass it to `nros::init(locator, domain, name)` (replacing
  the 2-arg `init()` that defaults `"nros_cpp"`). When unset (multi-node / no name), both use the
  unified `"node"` default.

So all three languages bake and read **one** `.nros_boot_config` struct — no Rust FFI on the
C/C++ read path (it is a plain C struct), and the future config-patch tool covers every language.
This is the C/C++ realization of phase-266 W5 (C) + W6 (C++).

### Single embedded bake site: `.nros_boot_config` (seed for the follow-on tracks)

Embedded has no runtime env, so the baked overlay is authoritative there. Require that the
baked config originate from **exactly one** well-defined, stable, `repr(C)` site:

```rust
#[repr(C)]
pub struct BakedBootConfig {
    magic: u32,            // 0x4E524243 "NRBC" — a post-link tool can locate it
    version: u16,
    set_flags: u16,        // one bit per field: baked-set vs use-default
    domain_id: u32,
    node_name: [u8; 64],   // NUL-padded, fixed-size — no pointers, patchable in place
    locator:   [u8; 96],
    namespace: [u8; 64],
}

#[no_mangle]
#[link_section = ".nros_boot_config"]   // KEEP in linker script, #[used]
pub static NROS_BOOT_CONFIG: BakedBootConfig = /* baked by nros::main! (Rust) or cmake (C/C++) */;
```

The resolver reads `NROS_BOOT_CONFIG` → `BootConfig` (Options derived from `set_flags`) →
`resolve`. The struct is language-agnostic (C/C++ emit the same `__attribute__((section))`
struct), so a single tool can later operate on the binary regardless of source language.

Within the scope of this RFC the blob is just a baked const (rebuild to change). Making it a
patchable static costs nothing extra now and unlocks the follow-on tracks below.

## Why this shape (decisions recorded)

- **Unify the source, not the sink** — the sink (`RmwConfig`/`CffiRmw::open`) is already
  shared; rebuilding it would be wasted work. The defect is N source-assembly paths.
- **Plain-field `BootConfig`** rather than moving `DeployOverlay` down a crate — avoids a
  dependency cycle and moves no types.
- **Precedence A** — only model that is simultaneously the simplest mental model and the
  smallest code (reuses the existing hosted env path + existing overlay; touches embedded
  boards once).
- **One bake site** — turns "where does embedded config come from" from per-board lore into a
  single inspectable symbol, and seeds the follow-on tracks at no extra cost.

## Scope and non-goals

**In scope (issue #101):** the resolver + precedence A in `nros-node`; the three call-site
mappings (fixing the C/C++ node-name defects and #98 across all boards, incl. NuttX
`run_with_deploy`); the single `.nros_boot_config` bake site; collapsing the two board-key
maps (`main_macro::board_path_for` + `emit_rust::board_path_for`) into one; a decision on the
near-dead `setup_transport` seam (keep + document, or remove).

**Explicit non-goals (tracked separately):**
- Runtime config from storage (NVS / flash sector) — couples to issue #80 (param persistence
  backends, currently disabled). Deferred.
- Per-node identity for multi-node-on-one-session (the multi-node half of #98) — needs a
  session-per-node or graph-liveliness change; out of scope here.

## Follow-on tracks (designed-for, not built here)

The `.nros_boot_config` bake site is deliberately the **member 0 of a future baked image**, so
two follow-ons are additive rather than rework:

1. **Config patch tool (build-once, re-flash).** Turn the baked const into the patchable
   static above and add `nros config patch <bin> --node-name … --locator … --domain …`:
   scan for `magic`, rewrite the fixed-size fields, set `set_flags`, fix a CRC. ~100 LoC,
   language-agnostic (operates on the binary). UX: flash a fleet with per-unit identity from
   one firmware build — no recompile. The resolver and call-sites are unchanged.

2. **Build-time plan image (the "build-time graph").** Extend `.nros_boot_config` to a
   `.nros_plan_image` carrying config **+ node table + entity table** (topic/type/hash/qos/
   callback/period), generated from the existing `MetadataRecorder` (`nros metadata` →
   `source-metadata.json`). Boot reads entities from the image instead of running each node's
   `register()`. Recorded constraints (from the runtime-vs-buildtime analysis): this is a
   **size + verification** play, **not** a perf win — the RMW declaration (the slow part)
   stays runtime and is irreducible; it must be **opt-in with a runtime-`register()`
   fallback** for nodes that create entities dynamically/conditionally; and it is always
   **generated**, never hand-maintained. This warrants its own RFC when undertaken; RFC-0045's
   single-bake-site is its seed.

## Migration / compatibility

- Behavior-preserving on POSIX/Native (already model A). Behavior-changing on embedded only in
  that the launch-declared `node_name` now appears (the intended #98 fix) and `[deploy.*]`
  locator/domain become authoritative where they were inert (notably NuttX).
- The native env workflow (`NROS_LOCATOR` per run, etc.) is preserved unchanged.
- `BootConfig` + `ExecutorConfig::resolve` are additive; existing `ExecutorConfig::new/from_env`
  builders stay (the resolver is implemented in terms of them).

## Cross-references

- RFC-0004 (configuration-and-transports) — the config model this extends on the delivery axis.
- Issue #101 (boot config not unified) — tracked-by this RFC.
- Issue #98 (component node name ignored) — resolved across all boards by this design.
- Issue #80 (param persistence backends) — gates the deferred storage-backed override.

## Changelog

- 2026-07-16 — issue #206: env rung completed across languages. C++'s
  parallel header implementation removed (both `init()` overloads);
  `nros_cpp_init` and `nros_support_init[_named]` route through the new
  fallible `ExecutorConfig::try_resolve`; `NROS_NODE_NAME` added to the
  hosted env rung; validation unified (`DOMAIN_ID_MAX = 232`, malformed
  `ROS_DOMAIN_ID` = error — never silent 0 / silent skip). Maintainer
  decision recorded: model A applies to explicit init args too (env
  overrides them on hosted, the ROS convention); `domain_id == 0` remains
  the unset sentinel at the C/C++ ABI edge.

- 2026-07-17 — issue #227: `DOMAIN_ID_EXPLICIT_ZERO_C_ABI` (255) +
  `baked_domain_from_c_abi` give the u8 C/C++ init surface an explicit-zero
  escape (0 remains the unset sentinel per the #206 model-A decision).
  u8→u32 domain type unification deferred to the next ABI window.
