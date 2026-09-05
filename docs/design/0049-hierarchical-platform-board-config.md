---
rfc: 0049
title: "Hierarchical platform/board configuration"
status: Stable
since: 2026-07
last-reviewed: 2026-07-16
implements-tracked-by: [phase-290]
supersedes: []
superseded-by: null
---

# RFC-0049 — Hierarchical platform/board configuration

## Summary

Build-time knobs (the phase-282 zenoh TX levers, executor sizing, ring depths,
heap budgets, …) are scattered across per-lane mechanisms with no ownership
story: env vars for cargo lanes, `defines_kv` in the central
`zenoh_platforms.toml`, Zephyr Kconfig forwards, `NROS_CMAKE_EXTRA_DEFS` for
C/C++ lanes. Nothing says *which platform should default to what* — the
phase-282 promotion decision (zephyr batching, measured 15–20×) had no place
to live — and an out-of-tree platform port cannot join `zenoh_platforms.toml`
without forking the tree.

This RFC defines **one schema, one file per package, a fixed four-rung
resolution ladder, and native per-lane front-ends** — the platform package
declares software-stack capabilities and reasonable defaults; the board
package declares hardware facts and deltas; the app/user overrides through
the lane's native mechanism. Kconfig is used only where a host framework is
Kconfig-native (Zephyr / NuttX / ESP-IDF packaging), as a hand-wired
front-end — never as the internal config system.

Survey basis: Zephyr `Kconfig.defconfig`/HWMv2 `board.yml` layering (the
arch→SoC→board→app duty split), ESP-IDF `sdkconfig.defaults` (+ per-target
suffix files), cargo `config.toml` walk-up merge, systemd drop-ins, NixOS
modules (schema declared once; layers contribute values). Convergent
pattern adopted: schema declared centrally, one file per ownership layer
inside that layer's package, fixed resolution order, capability-derived
defaults at the hardware layers.

> **Amended by [RFC-0086](0086-unified-configuration-transport-tenant-and-coupling.md)
> (2026-08-30), implemented by phase-400.** Three changes. (1) The ladder gains
> knob-to-knob constraints — `requires` / `implies` / `exactly-one-of` — because
> capability cross-checking alone cannot express that choosing a transport turns
> off the links and drivers it does not use. (2) The platform axis is moved into
> its package and resolved by search path, which is what this RFC's own "no
> central file" rule requires and what `config/<p>/` does not do. (3) The
> `[knobs.zenoh.tx]` / `[build.zenoh]` sections are keyed on the resolved
> backend, closing the leak RFC-0071 D8 names.

## Design

### Ownership — one schema, one file per package, no central file

```
nros-platform (core crate)                 — declares the SCHEMA + built-in
                                             defaults (code, versioned)
config/<p>/nros-platform.toml  — platform pkg: capabilities +
                                             knob defaults + [build.zenoh]
packages/boards/<b>/nros-board.toml        — board pkg (EXISTING descriptor,
                                             extended): hw facts + deltas
<app / user board pkg>                     — top rung via lane front-ends
```

`zenoh_platforms.toml` is **retired**: each `[platform.X]` block (defines,
`defines_kv`, sources, includes, arch, compile flags) relocates into that
platform package's `nros-platform.toml` under `[build.zenoh]`. A platform
package becomes fully self-contained; an out-of-tree platform is just a
crate + one toml, zero nano-ros-tree edits.

### Schema (shared tables; every field optional)

```toml
# nros-platform-zephyr/nros-platform.toml
[capabilities]                 # software-stack facts
threads = true
per_fd_tx_ceiling = true       # zsock serializes send/recv per fd —
                               # WHY batching pays on this platform

[knobs.zenoh.tx]               # policy defaults, capability-justified
batch = true                   # phase-282 W2: 15–20× streaming
split_lock = true
flush_ms = 50

[build.zenoh]                  # relocated from zenoh_platforms.toml
defines = ["ZENOH_GENERIC", "ZENOH_ZEPHYR"]
# defines_kv / sources / includes / arch / compile — same keys as before
```

Board file (`nros-board.toml`, extending the RFC-0042 descriptor): the same
`[capabilities]` and `[knobs.*]` tables, deltas only. **Duty rule: platform
toml = software-stack facts; board toml = hardware facts + overrides.**

`[knobs]` is general — `zenoh.tx` is the first tenant. Existing scattered
knobs (`NROS_EXECUTOR_MAX_CBS`, subscriber ring depths, heap sizes, smoltcp
buffer sizes) migrate tenant-by-tenant, each keeping its env/define name as
the lane front-end.

### Resolution — fixed ladder, explicit chain, no discovery magic

```
built-in default  <  platform toml  <  board toml  <  lane front-end
```

Per RFC-0004's rule this is a **fixed, short precedence ladder, not an open
merge**. The chain is explicit: the app names its board (deploy key, as
today) → the board toml names its platform → the loader follows that
two-hop chain. Out-of-tree board/platform directories resolve through the
existing RFC-0014 board-provisioning mechanism — no second resolver, no
cross-package walk-ups. (This named phase-201 as the out-of-tree half; that
phase was archived unstarted 2026-09-04 and the resolution moves to phase-420
W6's search path — `[workspace] package_paths` + `NROS_PACKAGE_PATH`. The
"no second resolver" rule is what survives, and W6 honours it: one search
path for every package kind, not a board-specific fallback.)

Resolved values are emitted into the one generated config header /
`-D` set the shim build already produces, preserving the issue-0135
shared-config ABI rule (flag-gated struct fields flip identically in every
TU).

### Capability cross-check

At resolution time, knob values are validated against capabilities:
`split_lock = true` with `threads = false` (or `batch = true` on a platform
whose `[capabilities]` lacks a flush-thread story) is a build-time warning +
downgrade-to-off, naming both files. Capabilities are facts; knobs are
policy; policy that contradicts fact never silently ships.

### Front-ends — native to each lane; Kconfig only where the host demands it

| lane | top-rung mechanism |
| --- | --- |
| cargo (native, bare-metal, RTOS rust leaves) | env vars (existing names: `ZPICO_TX_BATCH`, …) |
| CMake / ament | `-D` defines (existing names) |
| Zephyr / NuttX / ESP-IDF packaging | the **host's own Kconfig**, hand-wired in that framework-integration layer (as `zephyr/Kconfig` does today), forwarding to the same `-D`s |

Kconfig is a per-framework packaging detail implemented only when the native
framework requires it. There is **no nros-owned Kconfig and no Kconfig
generation machinery**. A framework fragment's `default` lines mirror the
platform toml; a **drift test** (not a generator) asserts the mirror —
cheap, and the fragment count is small (one per Kconfig-native framework).

Front-end semantics are explicit both ways: an unset env/Kconfig means
"defer to the ladder below"; a set value (including explicit `0`/`n`)
overrides. The Zephyr forward therefore always passes `-DZPICO_X=0|1` from
Kconfig rather than only passing on `y` (today's forward cannot express
"off over an on-default").

### Porter UX (the design's acceptance lens)

A porter bringing nano-ros to a new RTOS writes **2 crates + 2 tomls**,
edits nothing central, and touches Kconfig only if their host framework has
it:

- `nros new platform <name>` / `nros new board <name> --platform <p>` —
  scaffold the crates + tomls with the full schema as comments.
- Absent/empty tomls are valid; built-ins always produce a working build.
- `nros config explain --board <b>` — prints every knob: final value + the
  rung that set it (builtin / platform / board / env / Kconfig / -D). The
  primary debugging surface; opaque layered merges are the known failure
  mode of every layered-config system surveyed.
- Unknown keys fail loud with the valid-key list (`deny_unknown_fields`,
  the RFC-0033 precedent).

### Initial policy (capability-respecting, measured-only)

Per phase-282's own principle 5 ("a lever that does not move the number is
reverted or parked"), defaults flip only where measured:

| platform | zenoh.tx defaults | why |
| --- | --- | --- |
| zephyr | `batch + split_lock` on, `flush_ms = 50` | measured 15–20× (phase-282 W2); per-fd ceiling |
| freertos / nuttx | off | threads exist, plausible win — **unmeasured**; flip is one line in their own toml after a bench run |
| threadx / bare-metal / serial | off | no flush thread (spin-driven only) |
| native / posix | off | no fd ceiling; phase-279 W3 negative result for timer-paced systems |

This RESOLVES the phase-282 promotion decision as **option C, scoped to
zephyr, expressed as a platform default** rather than a hardcoded Kconfig
flip. `tx_express` remains the uniform per-publisher low-latency escape in
all three languages; the release note carries the `+flush_ms` latency line
for non-express topics.

## Alternatives considered

- **Kconfig as the internal config system** — rejected: cargo-native lanes
  have no Kconfig host (a `.config` in a cargo workspace is alien; the Rust
  embedded ecosystem uses features + env); non-Kconfig RTOS lanes would
  gain machinery they never asked for; the board descriptor's structured
  data (cargo_config blocks, entry signatures) is not Kconfig-expressible
  anyway. Kept strictly as a per-framework front-end.
- **Generated Kconfig fragments from the schema** — rejected in favor of
  hand-wired fragments + a drift test: the generator is machinery serving
  ~3 small files, and framework fragments carry framework-specific
  dependencies (`depends on NET_SOCKETS`, …) a generator would fight.
- **Keeping `zenoh_platforms.toml` for the build blocks** — rejected: the
  central file is the out-of-tree wall, and "platform build sources" is
  precisely platform-package duty.
- **Open deep-merge (cargo/Nix style) across arbitrary files** — rejected
  per RFC-0004: fixed ladder, explicitly named files only.

## Cross-refs

- RFC-0004 (fixed-ladder configuration philosophy; runtime config — this
  RFC is the BUILD-time sibling and does not touch `system.toml`).
- RFC-0042 (`nros-board.toml` board descriptor + `[board.capabilities]` —
  this RFC extends that file, not a new one).
- RFC-0014 (out-of-tree board/platform dir resolution). Was RFC-0014 /
  phase-201; phase-201 archived unstarted 2026-09-04, superseded by RFC-0087 —
  the search path is phase-420 W6.
- RFC-0033 (config-file precedent: `deny_unknown_fields`, deep-merge
  mechanics, discovery — note codegen capacity config remains its own
  file/system; message capacities are app-scoped, not platform-scoped).
- Issue 0135 (shared generated config ABI rule — preserved).
- Phase-282 (the TX knobs + promotion options this RFC's policy resolves);
  phase-279 (the measured baselines).

## Open questions (all resolved by phase-290)

1. ~~`[build.zenoh]` key layout~~ — **verbatim relocation** (mechanical,
   verified Debug-equal before the central file was deleted).
2. ~~per-knob `why` string in `explain`~~ — **no**: `explain` prints the
   `[capabilities]` line and the platform files carry rationale comments;
   a schema field would duplicate them.
3. ~~out-of-tree board toml on the cmake lane~~ — the board rung enters
   via the `NROS_BOARD_TOML` env hook (works from any lane that can set
   an env var, which all cmake lanes do); automatic export from the
   phase-201 registry is deferred until an in-tree board carries a
   `[knobs]` delta. **There is no phase-201 registry and there will not be**
   (archived unstarted 2026-09-04): under RFC-0087 a board is an ordinary
   package, so the automatic export, if it is ever wanted, comes off
   phase-420 W6's search path. The env hook is unaffected.

## Changelog

- 2026-07-16 — implemented by phase-290 (schema/loader, relocation +
  central-file retirement, tri-state Kconfig front-end + drift test,
  explain/scaffolders, the zephyr flip incl. the issue-0213
  declares-express fork fix); flipped to Stable with all open questions
  resolved.
- 2026-07-16 — created (Draft) from the phase-282 promotion discussion;
  design iterated against the out-of-tree porter UX (self-contained
  platform packages, explicit chain, `config explain`, scaffolders) and
  the "Kconfig only where the native framework requires it" decision.
  Work breakdown: [phase-290](../roadmap/archived/phase-290-hierarchical-config.md).
