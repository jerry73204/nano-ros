# Phase 349 — RTOS integration: make FreeRTOS an imported library like the rest

**Status (2026-09-01). W1 LANDED — with it,
[phase-348](archived/phase-348-source-time-provider-discovery.md) is complete.
W2 is UNBLOCKED (its prerequisite exists now; see its note) but not started.
W3–W6 open.**

**Implements:** [RFC-0072](../design/0072-rtos-integration-nano-ros-is-a-guest.md).

**The principle.** nano-ros is a library the user's project imports; the RTOS
owns its own build. Three of five platforms already work that way — Zephyr as a
west module, NuttX as an `apps/external/` app, ESP-IDF as a component, each
being build glue + Kconfig + the host's package manifest. FreeRTOS is the
anomaly: no shell, and `nros_freertos_build_kernel()` compiling the kernel from
`FREERTOS_DIR` + `FREERTOS_PORT`.

---

## W1 — The platform stops naming a stack — **LANDED**

- [x] `config/freertos-lwip/` → `config/freertos/`, with
      `names = ["freertos", "freertos-lwip"]` so every existing spelling
      resolves. No behaviour change.
- [x] `names` on all seven platform descriptors.
- [x] `platform` joins `check-provider-announcements.py`'s `FAMILIES` — one row,
      not a new gate. 19 providers across 3 families, 44 names.
- [x] `package.xml` provisions for all seven platform packages — **phase-348 W2's
      remaining half is now done.**

*Acceptance, met:* `nros ws providers --kind platform` lists 8 provisions from 7
packages, `--resolve platform:freertos-lwip` resolves to `config/freertos`, and
the zpico drift gate + `nros-board-common` (28 tests) pass unchanged.

**`requires_capabilities` deferred to W2**, deliberately. `PlatformConfigFile`
is `deny_unknown_fields`, so the field must exist before a file may name it —
but nothing consumes a *requirement* until the integration declares what it
provides, and adding an unread field invites the dead-config problem
[#529](../issues/archived/0529-zephyr-platform-knobs-never-resolve.md) is about.

### Alias resolution is the mechanism, and one lookup was blind to it

`names` alone would have broken every `freertos-lwip` lookup, because
`PlatformsTree` keys by DIRECTORY. `resolve_alias()` sits in
`PlatformsTree::chain()`, the one point `capabilities()`, `resolve_tx()` and
`capability_check()` all funnel through, so callers need no alias awareness.

**Except `declared_arch_names()`, which does not use `chain()`** — the arch
table is merged across all files and addressed separately, so it stayed
alias-blind. `freertos_lwip_resolves_both_declared_arches` caught it by
continuing to address the platform by its alias, which is why that test still
does so on purpose. The claim that `chain()` was the single funnel was written
in a comment before it was checked; it is true of three lookups out of four.

## W2 — The stack becomes a declared fact — **UNBLOCKED (2026-09-01), not started**

> **The prerequisite now exists. The note below is kept because its reasoning
> was right and its central claim has since become false.**
>
> It says "nothing anywhere sets that variable (`git grep NROS_BOARD_TOML`
> outside docs finds only the reader)". `cmd/board_facts.rs` sets it, phase-351
> built the cmake-side delivery (`NanoRosBoardFacts.cmake` +
> `corrosion_set_env_vars`, which attaches to the target's own build command —
> the reason `set(ENV{...})` was not enough), and W2.0 was superseded by that
> wave. phase-400 W6 then proved the channel end to end in a different build
> script: `nros-node/build.rs` reads `NROS_BOARD_TOML` and resolves the board
> rung of the knob ladder, using exactly the `watch_path` hazard this note
> names.
>
> `NROS_NETSTACK` is emitted too (`board_facts.rs:210`) and **nothing reads it**
> — the same declared-but-unread shape, now with a writer and no consumer
> rather than a reader and no writer.
>
> **What W2 still needs, and where to start.** Not the channel: the SELECTION
> SITE. `LinkPolicy::freertos_lwip()` (`runner.rs:1011`) hardcodes FreeRTOS ⇒
> lwIP, and the upstream port row exists
> (`zenoh-pico/src/system/freertos/freertos_plus_tcp/`). But the C sources for
> a port are not chosen from a Rust source list — `add_common_c_sources` adds
> only `system/common` and has no caller for the platform dir — so the netstack
> selection is reached through zenoh-pico's own CMake, driven by compile
> definitions. Locate that before writing code; this was checked on 2026-09-01
> and is where the previous investigation would otherwise restart from the
> wrong end.

> Original note, 2026-08-13, before writing any W2 code:
>
> Every bullet below needs `netstack` to be readable by
> `nros-zpico-build/src/runner.rs` at build time, so it can pick the
> `(rtos, netstack)` port row. The reader hook exists — `runner.rs:428` does
> `env_get("NROS_BOARD_TOML")` — but **nothing anywhere sets that variable**
> (`git grep NROS_BOARD_TOML` outside docs finds only the reader). phase-290's
> own roadmap says so: *"auto-export from orchestration deferred until a board
> carries a `[knobs]` delta"*. That day has arrived.
>
> Two false leads, checked and discarded:
> * `.cargo/nros-board.toml` (41 committed leaves, phase-341 W4) is **not** the
>   board descriptor. Same filename, different artifact — it is a projection of
>   the board's `cargo_config` (`[target.*]` runner + rustflags) and carries no
>   `platform` or `netstack`.
> * Deriving `netstack` from a cargo feature the way `use_freertos` works would
>   reintroduce exactly the feature-based selection RFC-0072 removes, and it
>   cannot express a fact the *board* knows.
>
> **The channel is tractable, and is W2.0.** Leaves already carry `[env]` blocks
> in `.cargo/config.toml` (`ZPICO_NO_SMOLTCP`, `NROS_LINK_IP` …), written by
> `nros sync` — so the missing piece is rendering `NROS_BOARD_TOML` there, plus
> the cmake-side export for cmake-driven builds. One hazard is already
> documented: **issue 0491 — never `rerun-if-env-changed` on a PATH variable;
> watch the CONTENT** via `nros_build_paths::{env_or_repo_path, watch_path}`.
> A path env var with three spellings is precisely what thrashed the shared
> cargo group before.

### W2.0 — carry the board descriptor to build scripts *(prerequisite)* — **SUPERSEDED**

> Superseded by [phase-351](archived/phase-351-board-facts-and-site-config.md) (2026-08-13).
> The leaf-`[env]` carrier landed and works for standalone leaves, but the
> six-ecosystem survey showed the payload is wrong: it points at the board
> DESCRIPTOR, when what a build script needs is the RESOLVED facts —
> board (A) merged with the project's `[deploy.*]` site block (B). It also
> never reaches workspace members, because corrosion runs cargo from
> `workspace_toml_dir`. phase-351 W6 retires it.

The investigation below stands and is why phase-351 exists.


Investigated 2026-08-13. Findings below are measured, not reasoned — see the
methodology note at the end, which exists because reasoning got it wrong first.

#### The consumer is a DEPENDENCY, so no walk-up can work

The build script that needs `netstack` is `zpico-sys`'s, and its
`CARGO_MANIFEST_DIR` is `packages/rmw/zenoh/zpico-sys` — the dependency's own
crate, never the leaf. The existing code says so out loud:

```rust
// crate -> zenoh -> rmw -> packages -> repo root
manifest_dir.join("../../../../config")
```

Walking up from there reaches the **nano-ros root**. So "locate a reference
file by walking up from `CARGO_MANIFEST_DIR`" is impossible in every lane, and
the carrier has to be the process environment.

#### Cargo `[env]` works, but only where the config is DISCOVERED

Measured with a leaf + a dependency crate, clean `target/` each time:

| cargo CWD | leaf `.cargo/config.toml` read? | dependency build script sees `[env]`? |
| --- | --- | --- |
| the leaf | yes | **yes** |
| elsewhere, with `--manifest-path <leaf>` | no | no |

Two facts that together make the carrier viable: discovery is **CWD-based**, and
once discovered, `[env]` **does** propagate into dependency build scripts —
which is the property the whole design needs.

`relative = true` resolves against the config file's own directory, so the value
is checkout-independent and therefore committable — the same reasoning that lets
phase-341's `.cargo/nros-board.toml` projection be committed.

#### Which lanes the carrier actually reaches

Corrosion 0.6.1 sets `WORKING_DIRECTORY "${workspace_toml_dir}"` with the
comment *"so that configuration files are found"* — it is deliberate. What
varies is which manifest it is handed:

| lane | manifest handed to cargo | leaf config found? |
| --- | --- | --- |
| standalone leaf, plain cargo | the leaf | **yes** |
| standalone leaf via corrosion (board overlay passes `${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml`) | the leaf | **yes** |
| workspace runtime crate (`NanoRosRuntimeCrate.cmake`) | `${CMAKE_BINARY_DIR}/nros_ws_runtime/Cargo.toml` | **no** — synthesized in the build dir |
| Zephyr (`zephyr/cmake/nros_cargo_build.cmake`) | `${NROS_REPO_DIR}/Cargo.toml` | **no** — the repo root |

So **two lanes need a second mechanism**, and they need different ones:

* The **workspace runtime crate** is generated by us — `NanoRosRuntimeCrate.cmake`
  already `file(WRITE)`s its `Cargo.toml` and `src/lib.rs` into that directory
  and nothing else. Writing a third file, `.cargo/config.toml`, into the same
  generated crate is the natural fix: build-dir output, not committed, and the
  configure already knows `NANO_ROS_BOARD`.
* **Zephyr** must use the explicit export it already uses for knobs. This is
  issue 0460's geometry exactly — the Rust lane is built by
  zephyr-lang-rust's `rust_cargo_application`, which inherits nothing.

A member-level `.cargo/config.toml` under a shared workspace
(`examples/workspaces/rust/src/nuttx_entry/.cargo/config.toml` exists) is **not**
read when the driver is the workspace runtime crate. A workspace builds one
board per configure, so workspace-level granularity is right — but a board fact
placed in a member config would be silently ignored.

#### `net_stack` already exists, and is the wrong field

`nros-board.toml` has carried `net_stack` all along — six boards say
`nanoros-owned`, three `rtos-owned`. It is **not** the field W2 needs:

* wrong axis — it answers *who brings up NIC + IP*, not *which stack*;
* **parsed and never read** — `NetStack` has no consumer beyond one
  construction site in `ws.rs`. Another field of the
  [#529](../issues/archived/0529-zephyr-platform-knobs-never-resolve.md) shape:
  authoritative-looking, inert.

Reusing the name for a finer meaning would silently change what nine
descriptors assert. **Add a separate `netstack` key**, and decide separately
whether the coarse one earns its place.

#### Design

1. `nros sync` renders `NROS_BOARD_TOML` as a **relative** `[env]` row into the
   leaf's `.cargo/config.toml`, resolving the board through the leaf's existing
   `<nano_ros board=…/>` consumption export and phase-348 discovery. One SSoT,
   no new authoring for the user.
2. `NanoRosRuntimeCrate.cmake` writes the same row into the `.cargo/config.toml`
   of the crate it synthesizes.
3. The Zephyr lane exports it explicitly, as it already does for knobs.
4. `zpico-sys`'s reader keeps watching the file's CONTENT
   (`rerun-if-changed`), never the variable — issue 0491. It is already correct
   here; the `rerun-if-env-changed` at `runner.rs:141` is in `env_usize`, a
   different path.

#### The gate, which is the point

Three mechanisms means one will rot, and the failure mode is not a wrong value
but **no value, defaulted, no diagnostic** — #529's shape, which took two wrong
write-ups to characterise. So: **a build whose leaf declares a board must have
the fact arrive, in every lane.** Without that, the RFC-0049 board rung stays
as dead as it has been since phase-290.

#### Methodology note

The first measurement of the `[env]` question was **wrong**, and reported as a
finding for a turn: a second `cargo build` against a warm `target/` replayed the
**cached build-script warning** from the first run, which looked like the leaf
config being read from a foreign CWD. It was not. Anyone re-testing cargo
config or build-script behaviour must wipe `target/` between cases — a build
script that does not re-run prints its previous output, and that output is
indistinguishable from a live result.

### W2 proper — once W2.0 lands

- [ ] `[board.integration]` carries `rtos` and `netstack` as facts, with
      `capabilities`.
- [ ] The six lwIP-specific lines leave `config/freertos*/` for the zenoh
      backend descriptor as a `(rtos, netstack)` port row.
- [ ] `freertos_plus_tcp` gains its row and becomes reachable — zenoh-pico
      already ships `system/freertos/freertos_plus_tcp/network.c`; nothing in
      `config/` can select it today.
- [ ] `[build.zenoh]` leaves **every** platform file. Platforms keep
      `[capabilities]`, `[arch.*]`, `compile`, `required_env`.

*Acceptance:* a fixture builds against `freertos_plus_tcp` without a new
platform directory — the property the old naming made impossible.

**This is where "platforms name no backends" lands**, extending phase-347's
*core names no backends*. Expect the same shape of work: a keying change, not a
schema design.

## W3 — Retire the kernel builder

- [ ] `nros_freertos_build_kernel()` / `nros_freertos_build_lwip()` deleted; the
      mps2-an385 fixture adopts upstream's own `CMakeLists.txt`
      (`add_subdirectory` + a `freertos_config` target + `FREERTOS_PORT` /
      `FREERTOS_HEAP` cache vars).
- [ ] `FREERTOS_PORT` stops being ours. Upstream owns that name and takes an
      **enum** (`GCC_ARM_CM3`); we take a **path fragment** (`GCC/ARM_CM3`)
      under the same name today, which fails confusingly for anyone arriving
      from upstream documentation.

*Acceptance:* the FreeRTOS fixtures build and pass with nano-ros compiling no
kernel source; `git grep -c 'portable/'` in our cmake is zero.

**Risk.** Upstream's port table is 1356 lines of generator expressions and only
*warns* on a cross build with no `FREERTOS_PORT`. Our two ports (`GCC_ARM_CM3`,
`GCC_ARM_CM7`) must be verified against it before deleting ours, not after.

## W4 — A FreeRTOS integration shell

- [ ] `integrations/freertos/` joins the other three, so the FreeRTOS row stops
      being the exception: CMake glue + Kconfig-equivalent + a manifest.
- [ ] Configure-time diagnostics RFC-0072 §5.3 names: unprovided capability, a
      `(rtos, netstack)` pair with no backend port, a missing include dir.

The last is not padding — the ST case has six include paths, several of which
are submodules that may be uninitialised, and the failure without a check is a
compiler error deep in vendor headers.

## W5 — The IDE drop-in

- [ ] `nros emit --board <b> --out <dir>` producing `include/`, `src/` (C to be
      compiled by their project), `lib/libnros_rust.a`, `nano_ros.mk`, and a
      `README-INTEGRATION.md` rendered with their paths.

The split is forced by the ABI rule, not chosen: the Rust half talks only
through the stable C ABI (RFC-0054) and prebuilds per triple; the C half must
see the user's own `FreeRTOSConfig.h` and `lwipopts.h`, and compiling it against
anything else is issue 0135's silent ABI break.

## W6 — The scaffolder (the UX half)

- [ ] `nros init --from-cube <project>` reading `.cproject` linked-resource XML;
      the MCUXpresso equivalent reading `prj.conf`.

**Why this is a wave and not a nicety.** The include set is not guessable: ST's
port directory is not derivable from the core (Cortex-M7 H7 examples use
`ARM_CM4F`), both config headers are application files, and two different files
are named `cmsis_os.h` with `-I` order deciding silently. Asking a user to get
that right by hand is the difference between "imported a library" and "spent an
afternoon".

---

## Order

```
W1 ──► W2 ──► W3 ──► W4 ──► W5
                       └──► W6
```

W1 stands alone and should land first regardless of the rest.

## Prerequisites — both RESOLVED 2026-08-13

* **`FREERTOS_PORT`'s two vocabularies** —
  [#530](../issues/archived/0530-freertos-port-two-vocabularies.md). The builder
  now accepts upstream's enum as well as our path fragment, so W3's move to
  upstream's `CMakeLists.txt` changes nothing for anyone already writing
  `GCC_ARM_CM3`.
* **Zephyr unselectable by the zpico resolver** —
  [#529](../issues/archived/0529-zephyr-platform-knobs-never-resolve.md). Resolver fixed;
  the two knob sources are now compared by `check-zephyr-knob-agreement`.

  **The severity stated in this section's first draft was wrong.** It claimed
  the phase-290 15–20× streaming promotion never applies on Zephyr. It does —
  the C lane gets it from `zephyr/Kconfig` defaults forwarded by
  `nros_rmw_zenoh.cmake` — and there is no ABI split either, because
  `build_c_shim` is skipped on Zephyr and `rust_consts()` never emits
  `tx_batch`. The real defect was two sources for one fact, agreeing only by
  coincidence.

## Deliberately not here

* **A `netstack` provider kind with selection.** Every vendor has welded its
  choice; there is nothing to select. Capabilities give the decoupling, and a
  selector can wait for a real second stack on one RTOS.
* **Prebuilt Rust staticlib distribution** — W5 needs one binary; a published
  triple × feature matrix is a separate decision.
