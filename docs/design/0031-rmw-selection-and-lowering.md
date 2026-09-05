---
rfc: 0031
title: "RMW backend selection and lowering"
status: Stable
since: 2026-06
last-reviewed: 2026-08
implements-tracked-by: [phase-227]
supersedes: []
superseded-by: null
---

# RFC-0031 — RMW backend selection and lowering

## Summary

A nano-ros build selects exactly one RMW backend per binary. The selection is a
**declared, language-agnostic config value** (in `system.toml`, or a CLI/build
flag), which the toolchain **lowers** to each language's native build mechanism
(a Rust cargo feature, a CMake cache var). The cargo feature is the *lowering
target*, not the user-facing knob — which resolves a long-standing documentation
contradiction (feature-vs-dependency) and an example inconsistency
(zenoh/xrce wired one way, cyclonedds another).

## Motivation

The repo carried two conflicting stories. `nros` has `rmw-zenoh` / `rmw-xrce` /
`rmw-cyclonedds` cargo features (deleted in Phase 128.C.3, **re-added** in Phase
214.S for parity); some docs say "select by feature," others say "select by
dependency, **not** features on `nros`." Examples were inconsistent: native
talker pulled `nros-rmw-zenoh`/`-xrce` as project-level optional deps but routed
cyclonedds through `nros/rmw-cyclonedds`. And a cargo feature is **Rust-only** —
it cannot be the canonical knob for a C/C++ project, which selects via CMake.
A single, cross-language selection model was needed.

## Design

### Scope: per-deploy, not per-node

A binary links the cffi runtime plus exactly **one** registered backend vtable,
so **RMW is a property of the binary**. All nodes in it inherit it.

> **Amended 2026-08-31 (issue 0938).** This section said "the deploy target /
> binary", which was one thing when it was written: a deploy target WAS the
> buildable unit. RFC-0065 split the two — `[deploy.*]` is PLACEMENT (where
> nodes run) and `[image.*]` is the buildable unit — so the sentence now has to
> pick, and it picks the binary. **RMW is declared on `[image.<id>]`.** In-process multi-RMW exists only via an explicit `[[bridge]]`
(RFC-0009), which opens additional sessions deliberately.

### Declared home, lowered per language

| Scope | User declares RMW in | Lowered by toolchain to |
|---|---|---|
| Workspace | `system.toml` `[image.<id>] rmw` (over `[image_defaults] rmw`, then `[system] rmw`; a `[deploy.<t>] rmw` is DEPRECATED and outranked) | Rust entry/node pkg → the **board crate's** `rmw-<x>` feature; C/C++ node pkg → `-DNANO_ROS_RMW`; C++ entry → CMake cache |
| Single-node, with `system.toml` | `[system] rmw` | same lowering |
| Single-node, no `system.toml` | CLI/build flag, else default | same lowering |

The Rust **board-crate `rmw-<x>` feature** and the CMake `NANO_ROS_RMW` var are
the **lowering targets** the toolchain sets (or a user sets manually as an
override). They are documented as *the mechanism the build uses*, never as *the
way you pick a backend*.

**Phase 248 C5b amendment — board crate is the Rust lowering target.** Earlier
the Rust lowering target was the `nros` umbrella's `rmw-<x>` feature. Under the
RMW/platform-agnosticism convergence (issue #60 / phase-248), the `nros` umbrella
is **agnostic** — it carries no concrete `rmw-*` features or backend deps. The
**board crate becomes the RMW selection point**: it brings the concrete backend
into the link graph and self-registers it via `RMW_INIT_ENTRIES` (proven in C5a;
`nros-board-linux` already works this way). Codegen therefore emits the entry's
**board dep** `features = ["rmw-<x>"]` (e.g. `nros-board-linux`,
`nros-board-mps2-an385-freertos`), with the `nros` dep carrying only the
`rmw-cffi` vtable. The lowered value table (`resolve_rmw`) is unchanged
(`rmw-<x>` cargo feature, `<x>` CMake value, `<TOKEN>` C define); only *which
crate's* feature it lands on moved (`nros` → board). Crate-less host boards
(native/posix, zephyr — no `nros-board-*` crate in some workspaces) are a
transitional exception: codegen still links their backend via the direct
`nros-rmw-*` dep + an explicit `register()` until a board crate carries the
feature.

### Precedence (highest wins)

1. CLI / build flag — `nros … --rmw <x>`, `-DNANO_ROS_RMW=<x>`.
2. `system.toml` `[image.<id>] rmw`, over `[image_defaults] rmw`.
3. `system.toml` `[deploy.<target>] rmw` — **DEPRECATED**, retires with the rest
   of `[deploy.*]`'s build fields (phase-383 W10.b).
4. `system.toml` `[system] rmw`.
5. Default — `zenoh`.

> **Amended 2026-08-31 (issue 0938).** Rung 2 is new and rung 3 was previously
> rung 2. The reason is not tidiness: `nros build` has resolved an image's RMW
> from `[image.*]` since RFC-0065, while `nros plan` and `nros codegen-system`
> read only `[deploy.<t>].rmw` — so a workspace declaring both BUILT one
> backend and BAKED another into `#define NROS_SYSTEM_RMW`, with no diagnostic.
> The ladder above is now what every path resolves, which is what the previous
> "no duality" claim in this RFC had promised without delivering.

> **Status (2026-06-17) — LANDED, single source, both paths (phase-255).** This precedence was
> historically only partly wired: the C/C++ bake read `[system].rmw`, but the Rust build path
> read a SEPARATE `[build].rmw` per-package `nros.toml` overlay (→ the board crate's `rmw-<x>`
> feature), `[deploy.<t>].rmw` was declared-but-unused, and there was no `--rmw` flag — a
> decoupled "RMW duality." [phase-255](../roadmap/archived/phase-255-rmw-config-unify.md) collapsed this:
> a single `SystemToml::resolved_rmw(target, cli)` applies the precedence above, read by BOTH the
> planner (`schema_build_json` → board `rmw-<x>` feature) and the bake (`render_system_config_h`
> → `#define NROS_SYSTEM_RMW{,_<TOKEN>}`). `--rmw` exists on `nros plan` + `nros codegen-system`.
> `[system].rmw` + `[deploy.<t>].rmw` is the one declared home (RFC-0004); the `[build].rmw` /
> `[[transport]].rmw` `nros.toml` overlay is now a **deprecated fallback that warns** (no fixture
> declares it — migration was a no-op), retired after the next release. A binary's multi-RMW link
> set comes from `[[bridge]]` (topology, in `system.toml`) via `SystemToml::bridged_rmws()` →
> `PlanBuildOptions::bridged_rmws` → `rmw_set`, not the build overlay.

### Common runtime

`nros` is always built with `rmw-cffi`, so `ConcreteSession = CffiSession`. The
selected backend crate, once linked, **self-registers** through the
`nros_rmw_vtable_t` C ABI via the link-time `RMW_INIT_ENTRIES` registry; the
walker resolves it at `Executor::open`. Selection therefore reduces to *"link the
chosen backend,"* which every language's lowering achieves.

### CycloneDDS exception

cyclonedds is not pure-cargo linkable — its register symbol lives in the
C++/CMake backend. Cyclone selection always routes through the CMake/Corrosion
build path (RFC-0005 / Phase 175), even for an otherwise-Rust binary. The
*declaration* is identical (`rmw = "cyclonedds"`); only the lowering differs.

### Consumer wiring (examples) — board-owned force-link (C5b)

> **Phase 248 C5b update.** The umbrella-feature force-link below is the
> *pre-convergence* model. Under phase-248 the **board crate** owns the
> force-link + self-register (its `rmw-<x>` feature pulls the backend and keeps
> its `RMW_INIT_ENTRIES` section live), so codegen lowers to the board's
> `rmw-<x>` feature and the `nros` umbrella stays agnostic. The mechanics are
> otherwise identical (a `#[used]` `__FORCE_LINK_*` static referencing the
> backend's `register`), just relocated `nros` → board. The text below is
> retained for the transitional period while crate-less host boards still link
> through the umbrella/direct-dep path.

**All three backends route through the `nros` umbrella feature**
(`rmw-<x> = ["nros/rmw-<x>"]`), with **no `register()` call in user `main.rs`**.
The mechanism (Phase 227.3, reopened 2026-06-09):

- `nros`'s `platform-*` / `ros-*` / `std` / `safety-e2e` / `link-tls` features
  **forward** to the optional backend via `?/` (re-adding the Phase-104.A
  forwarding — `?` keeps it inert for non-selected backends, so the bridge model
  is unaffected). This was safe to restore because 104.A only dropped forwarding
  as collateral of bridge decoupling, and Phase 214.S brought the optional
  backend deps back.
- `nros` carries `#[used] __FORCE_LINK_{ZENOH,XRCE}` statics (gated on the rmw
  feature + a non-bare-metal platform) that reference the backend's `register`,
  keeping its `RMW_INIT_ENTRIES` self-register section in the link graph. This is
  **cycle-free in the facade** because `nros-rmw-zenoh`/`-xrce` do not depend on
  `nros` — an earlier draft wrongly placed it in `nros-node` (where a cycle was
  possible) and concluded "won't-do"; the facade avoids that entirely.
- cyclonedds keeps its `nros-node` `__FORCE_LINK_CYCLONEDDS_SYS` keep-alive (its
  register is a C++ symbol in a leaf `-sys` crate) — same *user-facing* shape.

**Exceptions that stay explicit:** bare-metal / RTOS targets where `linkme` is
unsupported keep an explicit `register()` (supplied from config via the C
`nros_app_register_backends()` stub or a Kconfig overlay); and bridge nodes link
multiple backends and select per-session (`open_multi`).

*Verified 2026-06-09:* the native talker builds + runs on zenoh and xrce through
the umbrella with no `register()` (both reach the transport layer = the backend
self-registered); the old explicit-register build fails identically → no
regression.

### Amendment (Phase 248 C5) — the board crate is the lowering target

Phase 248 (issue #60) moves the **lowering target off the `nros` umbrella and
onto the board crate**, for both the RMW and the platform axis, so the umbrella
stays fully agnostic (it carries only the `rmw-cffi` + `platform-cffi` vtable
shims, never a concrete `rmw-*` / `platform-*` feature or backend/platform dep).

- **RMW axis (C5a/C5b).** The board crate brings the concrete backend into the
  link graph (an optional `nros-rmw-<x>` dep behind the board's own `rmw-<x>`
  feature, plus the backend force-link static) and selects the backend's
  per-platform C port on that dep line (`nros-rmw-zenoh { features =
  ["platform-<rtos>"] }`). Codegen lowers `system.toml` `[system].rmw` /
  `[deploy.<t>].rmw` to the **board-dep** `rmw-<x>` feature, not `nros/rmw-<x>`.

- **Platform axis (C5c).** The board crate brings the concrete platform impl
  DIRECTLY — it deps `nros-platform { features = ["platform-<rtos>"] }` (which
  pulls the `nros-platform-cffi` dispatch types + the matching C-port link
  directive and emits the `__FORCE_LINK_CFFI` anchor), rather than activating
  `nros/platform-<rtos>` through its `nros` umbrella dep. The board is therefore
  the single platform-selection point. Codegen no longer emits
  `nros/platform-<x>` in a generated entry's default features when the entry
  deps a board crate; the board carries the platform.

The CMake `-DNANO_ROS_RMW` / C lowering path is unchanged (the C symbols already
come from the board / CMake layer). After every consumer migrates, the `nros`
umbrella drops its `rmw-*` / `platform-*` features and the optional concrete
deps entirely (phase-248 C5c).

**Crate-less host boards** (`posix` / `zephyr` / `orin-spe`, which have no
board crate in the CLI board descriptor) are the residual exception: their
generated entries still lower `nros/platform-<x>` until they gain a board crate
or a direct `nros-platform` dep. Tracked by phase-248 (C6 / C5c).

This SUPERSEDES the earlier escape hatch in ARCHITECTURE §2 that named the `nros`
umbrella as a permissible lowering target — the board crate is now that target.

### Generalization (Phase 250 / issue 0072) — declared capability/feature axes

RMW and platform are the first two **declared-selection → lowered-build-feature**
axes. The same machinery generalizes to **capability/feature axes** the user toggles
in config — `safety` (E2E message integrity), `param_services` (the external param
server), `link-tls`, and future ones. The forward-reference at the top of this section
(`nros`'s `safety-e2e` / `link-tls` features forward to the backend via `?/`) is the
first hint; this makes it the model.

**Same three lowering targets as RMW** (a declared axis lowers to all that apply):

1. **Entry `nros` feature** — `[safety] → nros/safety-e2e`, `[param_services] →
   nros/param-services` (`generated_default_features`, phase-250 Waves 1/3). This
   compiles the *surface* (the `ctx.integrity()` API, the param-service handlers) in.
2. **Direct backend dep** (board-less native) — the declared axis also enables the
   BACKEND's own feature, because the capability's wire behaviour lives there, not in
   the umbrella, and Cargo features do not propagate upward. `[safety]` →
   `nros-rmw-zenoh { features = ["safety-e2e"] }` (the CRC attach/validate). Threaded
   through `backend_features(build, backend, safety)` (issue 0072, native done). Only
   backends that declare the feature get it (zenoh; xrce/cyclonedds have no
   `safety-e2e`, so the axis no-ops there).
3. **Board crate feature** (board-backed / embedded) — the board is the selection
   point (C5b), so the declared axis lowers to a board-crate feature that forwards to
   the board's own backend: `[safety]` → board `safety-e2e = ["nros-rmw-zenoh?/safety-e2e"]`.
   Each board owns its forwarding (heterogeneous: most carry `nros-rmw-zenoh` optional
   behind `rmw-zenoh`, so `?/safety-e2e` fits; family crates like `nros-board-threadx`
   forward to their overlay; xrce/cyclone-only boards declare `safety-e2e = []`,
   inert). **OPEN** — tracked by [issue 0072](../issues/archived/0072-safety-e2e-backend-feature-not-lowered.md).

**Capability registry (the SSoT, parallels `resolve_rmw`).** Replace the ad-hoc
`if safety && backend == "zenoh"` checks with one table:
```rust
struct ResolvedCapability {
    declared: &str,               // "safety"
    nros_feature: &str,           // "safety-e2e"          (entry umbrella, target 1)
    board_feature: &str,          // "safety-e2e"          (board crate, target 3)
    backends_supporting: &[&str], // ["zenoh"]             (target 2; others no-op)
    cmake_token: Option<&str>,    // C/C++ — None today
    c_define: Option<&str>,       // C/C++ — None today
}
fn resolve_capability(axis: &str) -> Result<ResolvedCapability, UnknownCapability>;
```
The entry-feature, backend-dep, and board-feature lowering all read this table.

**Descriptor gating (avoid build errors).** Emitting a board `safety-e2e` feature for
a board that does not declare one is a Cargo error. So the board descriptor
(`board_descriptor.rs`, already carrying `board_features` + the RFC-0042 `[board.capabilities]`)
advertises which capability features the board supports; codegen emits the board
feature only when advertised, else **skips + warns**. This keeps the per-board change
local + reviewable (each board's forwarding matches its own deps) rather than a fragile
global edit, and is the validatable gate.

**C/C++/CMake.** `safety-e2e` is **Rust-only** today (zero `NROS_SAFETY` in C/CMake;
the CRC machinery is feature-gated inside the zpico Rust shim). A C/C++ embedded build
linking the zenoh backend does not validate CRC. Extending the axis to C/C++ needs a
CMake/C `#define` (the `resolve_capability.cmake_token` / `.c_define` slots, mirroring
`resolve_rmw`'s `-DNANO_ROS_RMW` + `NROS_SYSTEM_RMW_<TOK>`) plus a zpico-C safety gate —
a deeper, separate gap.

**Status:** targets 1 (entry) + 2 (native backend) landed (phase-250 + issue-0072
native); target 3 (board) + the registry landed in phase-252; the C/C++ ABI landed in
issue 0073. The config surface stays the typed `[safety]` / `[param_services]` blocks
(they validate + carry defaults); a generic declared-feature list is a possible future
sugar over the same registry.

**Declared home — `system.toml`, both paths (phase-254).** A capability axis is declared
ONCE in the bringup `system.toml` (typed, beside `[system].rmw`), read by BOTH the Rust
orchestration (`planner` → `NrosPlan` → `generate`) and the C/C++ bake (`codegen_system`
→ `system_config.h`). This supersedes the transitional per-package `nros.toml`
capability-overlay read (Phase-172): per RFC-0004 §5 `nros.toml` is the embedded
direct-mode runtime file only. The single source means a declared `[safety]` lowers to
the Rust `nros/safety-e2e` feature (targets 1-3) AND the C/C++ `#define NROS_SYSTEM_SAFETY_E2E`
— no per-language config divergence, and no RMW-style double declaration.

## Alternatives considered

- **Cargo feature as the canonical knob.** Rejected: Rust-only; cannot express
  C/C++ selection; leaks a build-system detail into the user model.
- **Backend crate as a direct dependency (the project-dep pattern).** Workable
  for Rust but still Rust-only and still not the *declared* config; kept only as
  the lowered form, not the surface.
- **Per-node RMW.** Rejected: a binary links one backend; per-node backends would
  require multiple processes or the bridge, which the `[[bridge]]` path already
  covers explicitly.

## Open questions / gaps (tracked by phase-227)

- Add `rmw` resolution for single-node from `system.toml` / flag (today single-node
  Rust uses the cargo feature directly).
- Converge examples so zenoh / xrce / cyclonedds all lower uniformly from the
  declared value.
- ~~Make `nros new --rmw <x>` actually template the scaffold~~ — **CLOSED
  (verified 2026-08-31).** `args.rmw` is threaded into the scaffold structs at
  `cmd/new.rs:302`, `:460`/`:472` and `:507`, each with a real default. No
  "next steps" banner remains. The RFC had simply not been updated.
- ~~Sync the contradictory book pages~~ — **CLOSED (2026-08-31).**
  `user-guide/rmw-backends.md` had kept documenting the PRE-C5b lowering ("add
  `nros` with the platform feature plus one `nros-rmw-<x>` shim"), which the
  C5b amendment below retired when the board crate became the selection point.
  Measured against real leaves before rewriting: they forward to
  `nros-board-linux/rmw-<x>`, and workspace node pkgs say so in a comment. The
  page now shows that shape and says a workspace declares
  `[image.<id>].rmw` instead of writing any of it.

## Changelog

- 2026-06 — created; resolves the feature-vs-dependency contradiction by making
  RMW a declared-and-lowered, per-deploy, language-agnostic selection.
- 2026-06 (phase-248 C5) — **Rust lowering target moved `nros` → board crate, both
  axes.** The `nros` umbrella is agnostic (vtable-only); the board crate is the
  selection point for RMW (self-links + registers the backend, C5a/C5b) AND
  platform (direct `nros-platform { features = ["platform-<rtos>"] }` dep,
  C5-platform). Codegen (`cargo-nano-ros` scaffold + `nros-cli-core` orchestration
  `generate`) emits the entry's **board dep** `features = ["rmw-<x>"]` and no
  longer emits `nros/rmw-<x>` or `nros/platform-<y>` for board-backed entries.
  `resolve_rmw`'s lowered-value table is unchanged; only the feature's host crate
  moved. Crate-less host boards (native/posix, zephyr, orin-spe) remain a
  transitional exception — resolved in C5c after C6 consumer migration.
- 2026-06 (phase-250 / issue 0072) — **generalized the declared-selection → lowered-
  build-feature pattern to capability/feature axes** (`safety`, `param_services`,
  link-tls, future), reusing the board-as-selection-point model. Three lowering
  targets (entry `nros` feature, direct backend dep, board-crate feature) + a
  `resolve_capability` registry SSoT + descriptor gating. Targets 1–2 landed; the
  board target + registry refactor + C/C++ path tracked in issue 0072. See
  "Generalization (Phase 250 / issue 0072)" above.
