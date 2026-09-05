---
rfc: 0032
title: "Entry-Codegen Pipeline — main() emission across frameworks + tiers"
status: Draft
since: 2026-06
last-reviewed: 2026-06
implements-tracked-by: [phase-212, phase-228, phase-236]
supersedes: []
superseded-by: null
---

# Entry-Codegen Pipeline — `main()` emission across frameworks + tiers

> **Canonical path (phase-296 R3/R4).** The entry `main()` emission now bakes
> from the play_launch-resolved **SystemModel** (RFC-0050/0052): the Rust
> `nros::main!(model = "<bringup>")` arm and the C/C++
> `nano_ros_add_executable(... MODEL <system_model.yaml>)` keyword read a
> committed `<bringup>/config/system_model.yaml` instead of parsing launch
> XML + `system.toml` at build time. The `launch = …` / `LAUNCH` inputs this
> RFC describes are **transitional** — deprecated in R3
> (`warning[deprecated]`), removed in R4. The emitters, plan IR, and per-tier
> `run_tiers` seam are unchanged; only the plan's *source* moves to the model
> (`plan_from_model` alongside `plan_from_launch`).

**Status:** Draft (design-of-record for the entry emitter; the multi-tier slice
is tracked by Phase 228).

**Scope boundary.** This RFC is the **how-`main()`-is-emitted** doc. It is the
companion to:

- **RFC-0015** (RTOS orchestration) — the *execution model* (priority tiers, one
  `Executor` per tier, one shared session). RFC-0015 says *what* runs; this RFC
  says *how the entry TU is generated* to make it run. RFC-0015 §3/§11 sketch a
  `cargo nano-ros generate-main` template pipeline that **predates** the
  proc-macro entry codegen and is superseded by this RFC (see the RFC-0015
  banner).
- **RFC-0003** (RTOS integration pattern) — the *embedded host-time C baking*
  path (`nros codegen-system` → `system_config.h` + `system_main.c`, consumed at
  the vendor's configure phase). That is the **C/vendor** entry scaffold; this
  RFC's proc-macro is the **Rust** entry scaffold. Both consume the same
  node-set source of truth (launch tree + `system.toml`).
- **RFC-0031** (RMW selection + lowering), **RFC-0023/0024/0025** (workspace +
  codegen discovery), **RFC-0016** (per-RTOS scheduling).

Non-goals: the executor internals, the RMW vtable, message codegen.

---

## 1. Problem

A nano-ros binary needs a `main()` (or the framework's boot symbol) that:

1. brings up the board (HW init, transport, network-wait),
2. opens the RMW session,
3. registers every Node package the launch tree names, and
4. spins.

On Linux this is a hand-written `main.rs`. nano-ros generates it from the launch
tree + `system.toml` so the ROS 2 mental model survives onto RTOS (RFC-0015 §1).
The generation must cover **five framework shapes** (the boot symbol differs per
framework) **and** the single-vs-multi-tier execution model — without forking
into N bespoke generators, and without the multi-tier path perturbing the
single-tier output that ships today.

---

## 2. Two emitters, one node-set source of truth

The Rust entry TU is produced by **two** code paths that resolve the same launch
tree the same way:

| Emitter | Where | When | Why it exists |
| --- | --- | --- | --- |
| **`nros::main!()` proc-macro** | `packages/core/nros-macros/src/main_macro.rs` | the consumer's `cargo build` (compile time) | **Canonical.** Has `proc_macro::Span` for diagnostics a shell-out can't match; expands directly into the Entry crate. |
| **`nros codegen entry --lang rust\|cpp\|c`** | `packages/cli/nros-cli-core/src/codegen/entry/emit_{rust,cpp,c}.rs` | host-time CLI invocation | **Mirror.** Pre-bakes the macro expansion for byte-level diffing, inspection outside a cargo build, and the C/C++ entry TUs. |

Both consult the workspace pkg-index + launch parser (`nros_build::pkg_index` /
`launch_parser`) and lower to a per-node register list. The CLI path lowers into
the shared `codegen::entry::Plan` IR (`board`, `nodes[]`, `bringup`,
`launch_file`, `depfile_paths`); the proc-macro keeps its own in-place walk
because it additionally needs `Span` per node. **The launch tree + `system.toml`
are the single source of truth for the node set** — neither emitter invents
nodes.

The **embedded-C path** (RFC-0003) is a third emitter for a *different language*
(`nros codegen-system` → `system_main.c`), not a competitor: it bakes the same
node set into C for vendors whose build owns `main`. This RFC governs the Rust
emitters; RFC-0003 governs the C bake. They share the node-set inputs, not the
output.

---

## 3. Boot scaffolds — one per framework

`nros::main!` resolves a `Framework` from the Entry pkg's
`[package.metadata.nros.entry] deploy = "<board>"` (or an explicit `board = X`),
then emits the matching boot scaffold. All five register the **same**
launch-resolved `<pkg>::register(runtime)?` calls; only the boot/spin envelope
differs.

| `Framework` | Boot symbol | Spin owner | Notes |
| --- | --- | --- | --- |
| `OwnedSpin` | `fn main` (hosted) + `extern "C" fn main` (`target_os = "none"`) | `BoardEntry::run` | Native/POSIX, FreeRTOS, NuttX, ThreadX. Default for any board key not routed below. |
| `Zephyr` | `extern "C" fn rust_main` (staticlib export) | in-body loop | RTOS owns boot + C `main`; no Rust `fn main`. Gates on `wait_link_up`. |
| `Rtic` | `#[rtic::app]` module + `#[init]` | RTIC tasks | Per-Node `register_dispatch(&mut executor)` splice into the dispatch-slot table. |
| `Embassy` | `#[embassy_executor::main] async fn main` | Embassy tasks | Same `register_dispatch` splice. |
| `Esp32` | `#[esp_hal::main] fn main -> !` | `BoardEntry::run` | esp-hal `_start` requires the hal entry; delegates to the real-runtime `run`. |

The `BoardEntry` trait (`nros-platform`) is the portable seam: codegen names a
per-board ZST and calls `<Board>::run(setup)` without knowing the family; the
family driver crate (`nros-board-linux`, `nros-board-freertos`, …) owns the
lifecycle body (`init_hardware` → open executor → build `RuntimeCtx` → `setup` →
spin/exit).

### 3.1 Embedded panic handler + Node-pkg crate-type (boot-scaffold completeness)

A no_std firmware needs **exactly one** `#[panic_handler]` in its crate graph,
and the `OwnedSpin`-RTOS split (Entry pkg + Node pkg + board crate) must guarantee
it without collision. Two boot-scaffold rules close this (design-of-record;
surfaced by FreeRTOS Entry-pkg bring-up, [issue 0045](../issues/archived/0045-freertos-entry-component-staticlib-panic-handler.md)):

1. **The embedded panic handler is board-owned, cfg-gated.** The per-board family
   crate (e.g. `nros-board-mps2-an385-freertos`) carries
   `#[cfg(target_os = "none")] use panic_semihosting as _;` at its crate root and
   deps the panic crate. Linking the board rlib — which every Entry pkg does —
   brings the `panic_impl` lang item to the firmware bin; the **Entry pkg stays
   untouched** (no panic dep, no macro injection). The `cfg(target_os = "none")`
   gate keeps the board crate host-compatible. This **replaces** the legacy
   board-descriptor `crate_root_extra = "use panic_semihosting as _;"` injection:
   that string was emitted into the generated Entry main by the old
   `nros codegen-system` path (RFC-0003), which `nros::main!()` does **not**
   consume — so under the macro emitter the bin had no handler. Board-ownership
   is emitter-independent (works for both the macro and the CLI mirror) and is the
   single source; `crate_root_extra` is retired for Rust Entry panic.

2. **Node-pkg crate-type is deployment-path-specific, not universal.** The
   pure-cargo Entry path links the Node pkg as an **rlib** only; its `staticlib`
   output is never consumed there, yet rustc still demands a `#[panic_handler]`
   for any no_std `staticlib` it emits — which the rlib must **not** carry (it
   would duplicate the board's). Therefore:
   - **Pure-cargo Entry path** (Rust-native FreeRTOS/NuttX/ThreadX) → Node pkg
     declares `crate-type = ["rlib"]`.
   - **cmake / Corrosion path** (C-owned firmware, the RFC-0003 bake) → Node pkg
     declares `crate-type = ["staticlib"]`; the staticlib's panic comes from the
     vendor/host link (host-linked sims use std; embedded C firmware owns abort).
     Corrosion 0.5 `CRATE_TYPES` only *selects from declared* types, so the path
     that needs a staticlib must declare it.

   This corrects RFC-0024's earlier "irreducible `["rlib", "staticlib"]`" Node-pkg
   shape, which conflated the two paths. See RFC-0024 §6.4.

The boot scaffold is only "complete" when both hold: every `OwnedSpin` firmware
links a board rlib that owns panic, and its Node pkgs are rlib-only on the
pure-cargo path. The linker script is the third leg — the Entry pkg's
`.cargo/config.toml` `link-arg=-T<board>.ld` must track the board descriptor's
`cargo_config` (the board build.rs emits the script to `OUT_DIR`); a stale pin
(`-Tlink.x`) is a config-sync bug, not a codegen one.

> **Amendment (2026-08-07) — "must track" is being replaced by "is generated".**
> Making the leaf mirror the descriptor by hand is the mirror-drift class, and it
> fired as issue 0440: phase-338 W2 kept a node package's config, the NuttX
> entries lost their kernel-archive group, and the result was ~3680 undefined
> references visible only at link time. Measured 2026-08-07: 59 tracked leaf
> configs carry a `[target.*]` block, **23 distinct**, and 54 of 56 in-scope are
> their board's block verbatim or a superset (three extra args exist in total,
> `--gc-sections` accounting for 32 of them). `check-board-cargo-config-applied`
> guards 8 of those 59 and is representative rather than exhaustive by design.
> [phase-341](../roadmap/archived/phase-341-board-cargo-config-is-generated.md) moves the
> block to a generated projection of the descriptor — COMMITTED and gated in the
> `check-abi-bindings` mould, so a clone still links while drift becomes
> uncommittable rather than merely detectable. (A gitignored projection was
> considered and rejected: it would make the IMAGE depend on `nros sync`, which
> is issue 0463's failure aimed at a worse target.) The hoisting it depends on
> has landed; this paragraph's "config-sync bug" stops naming a live class when
> W2-W4 do.

---

## 4. The single-tier contract (today)

`BoardEntry::run<F>(setup: F)` where `F: FnOnce(&mut RuntimeCtx) -> Result<(),E>`.
The board opens **one** `Executor`, wraps it in `ExecutorNodeRuntime`, hands the
`setup` closure a `RuntimeCtx`, and the closure registers every Node pkg. **The
spin is owned by `setup`** on hosted targets (the macro emits a bounded
`NROS_ENTRY_SPIN_MS` loop for the E2E harness; embedded loops forever inside the
board body). This is the shape Phases 94/126/212.N shipped and what every example
builds today.

---

## 5. The multi-tier contract (Phase 228)

When the system declares scheduling tiers, codegen emits **one RTOS task +
`Executor` per tier over one shared session** (RFC-0015 §2.2/§2.3). The board
seam for this is a sibling entry point:

```
<Board>::run_tiers(tiers: &[TierSpec], setup: F)
    where F: Fn(&mut RuntimeCtx) -> Result<(),E> + Sync
```

Contract differences from `run` (and *why*):

- **`setup` is `Fn`, not `FnOnce`, and register-only.** It is invoked **once per
  tier executor** (each tier registers the whole node set; the executor's
  `active_groups` filter admits only that tier's callbacks). The closure must
  *register only* — **the board owns the per-tier spin** so it can install the
  group filter (`set_active_groups`) before spinning. (In `run`, by contrast,
  `setup` owns the spin.)
- **One shared session.** The boot task opens the session once; tier tasks borrow
  it via `Executor::open_with_session(session_ptr)` (the RMW session is a
  process-wide singleton — opening twice fails, RFC-0015 §2.3). The boot task
  runs the highest-priority tier; the rest are spawned with the platform's task
  primitive (`nros_platform_task_init` / `nros_freertos_create_task` / …).
- **`TierSpec`** (`nros-platform`) carries `{name, groups: &[&str], priority:
  i64, stack_bytes, spin_period_us}`. `priority` is the **raw per-RTOS** value
  the author wrote in `[tiers.<name>.<rtos>].priority` (already in the kernel's
  scale; `i64` admits Zephyr negative coop) — passed straight to the spawn call.
  The RFC-0016 `*_priority_for` mappers are a separate utility for authors who
  prefer a normalized 0–31 scale; the codegen path uses the raw value.

### 5.0 Platform applicability — multi-tier ⟹ MT=1, satisfied by every RTOS

`run_tiers` runs N preemptive spin tasks on one shared zenoh-pico session, which
requires the session's **internal mutexes** (`Z_FEATURE_MULTI_THREAD=1`). A 2026-06
study (vs `zenoh_platforms.toml` + the `system/*` backends) established:

- **Every RTOS target already builds MT=1** — POSIX, Zephyr, FreeRTOS, NuttX
  (`system/unix` pthreads), ThreadX (`system/threadx`), ESP-IDF/Orin-SPE
  (`system/freertos`). So multi-tier needs **no MT change** and adds no
  session-MT cost (MT=1 is already paid by the single-session model). The 228.F
  two-executor result generalizes to these.
- **Bare-metal forces MT=0** (no RTOS / no threading backend). Multi-tier is
  inherently N/A there — there are no preemptive tasks. Bare-metal is
  `Framework::Rtic`/`Esp32`-bare; its "tiers" are RTIC `#[task(priority)]`
  interrupt priorities (framework-owned, §8 item 3), not `run_tiers`.

So the emitter never needs an MT-flip or an MT error gate: the `OwnedSpin`-RTOS
boards that take the `run_tiers` path are exactly the MT=1 platforms. (This
corrected RFC-0015 §2.3/§7.1, which had claimed MT=0 for FreeRTOS/NuttX/ThreadX.)

### 5.1 Degenerate gate — single-tier stays byte-identical

The emitter chooses the path on **tier presence**:

- system has no `[tiers.*]` **or** the resolved table `is_single_tier()` (one
  synthesized `default` tier) → emit the **unchanged `BoardEntry::run`** path.
  Every example today takes this branch; output is byte-identical to pre-228.
- otherwise → emit `run_tiers(&[TierSpec{…}, …], run_plan)` with a register-only
  `run_plan`.

This keeps the multi-tier blast radius to systems that opt in (none today): a
bug in the new path cannot affect a single-tier build.

---

## 6. Where tier data enters the emitter — the shared resolver

The proc-macro must resolve the tier table **at expansion time**, and it must
resolve it **identically** to the CLI's `codegen-system` bake (else a binary's
compile-time entry and its baked `nros-plan.json` disagree). Three sources were
considered (Phase 228):

1. **Re-read + re-implement in the macro** — duplicates `resolve_tiers`; drifts.
2. **Read the baked `nros-plan.json`** — true SSoT, but native builds use plain
   `cargo build` with no prior bake step → ordering dependency.
3. **Shared resolver crate** *(chosen)* — extract the tier schema + `resolve_tiers`
   into a leaf crate both consumers depend on.

**Decision: a shared leaf crate `nros-orchestration-ir`** (runtime workspace,
serde + thiserror only) owns the tier schema (`TierDef`, `TierRtosSpec`,
`CallbackGroupDecl`, `NodeOverride`, `CallbackGroupOverride`) + `resolve_tiers`.
- `nros-cli-core` path-deps it (re-exports the types; `codegen-system` calls it).
- `nros-macros` path-deps it (same workspace) and calls the *same* function at
  expansion. The archived GitHub `nros-build` git-dep is untouched — the macro
  reads `system.toml` + node `[package.metadata.nros.node].callback_groups`
  itself and feeds the shared resolver.

`resolve_tiers` takes the **decomposed** inputs `(tiers, node_overrides,
component_names, callback_groups, target_rtos)` rather than a whole `SystemToml`,
so the leaf crate stays free of the full CLI config type. The macro derives
`target_rtos` from the resolved board (`native`/`posix` → `posix`, `freertos*` →
`freertos`, …).

```
launch tree + system.toml + node callback_groups
        │
        ├── nros codegen-system ──► resolve_tiers ──► nros-plan.json (bake / RFC-0003)
        └── nros::main! (expand) ──► resolve_tiers ──► run / run_tiers emit
                                        ▲
                              nros-orchestration-ir  (one definition, no drift)
```

---

## 7. Invariants

- **Single-tier byte-identical.** No `[tiers.*]` ⇒ the `run` emit is unchanged.
  (Phase 228 keeps a parity check.)
- **One node-set SSoT.** Both Rust emitters + the C bake resolve the launch tree
  identically; none invents nodes.
- **One resolver.** Compile-time (macro) and bake-time (CLI) tier resolution call
  the same `nros-orchestration-ir::resolve_tiers`.
- **Node-pinned-to-tier (v1).** A node's callback groups must all resolve to one
  tier (enforced in the resolver). Cross-tier data is `[[shared_state]]`
  (RFC-0015 §8). v2 relaxes with multi-task state-sync.
- **Raw per-RTOS priority.** Authored in `[tiers.<name>.<rtos>].priority`, emitted
  verbatim into the spawn call; codegen does not auto-flip direction.
- **Instance identity.** `callback_groups` are per-*package* metadata
  (`[package.metadata.nros.node].callback_groups`); tiers + `[[node_overrides]]`
  key by node *instance* name. The emitter keys groups by the node pkg, applies
  instance overrides by name, and **requires the launch `<node name=…>` to equal
  the `system.toml [[component]].name`** — mismatch is a hard error at emit
  (matches `codegen-system`). Two instances of one pkg share the pkg's groups but
  can be reassigned independently via overrides.

---

## 8. Status, decisions, open items

**Landed (Phase 228):** the runtime mechanism — the `active_groups` registration
gate, the `.callback_group()` label, `Executor::session_ptr` /
`open_with_session` / `set_active_groups`, `TierSpec` + RFC-0016 maps,
`LinuxBoard::run_tiers`, and the `nros-orchestration-ir` extraction. Validated by
`phase228_tier_filter.rs` (two executors, one shared session, off-tier callbacks
gated to zero) against real zenohd.

**Decided (2026-06 design discussion):**

- **MT model** — multi-tier ⟹ MT=1, satisfied by every RTOS target with no
  change; bare-metal (MT=0) has no `run_tiers` (§5.0). No MT-flip or error gate.
- **Testing the emitted multi-tier example** — use an **external-observer E2E**
  (spawn the binary, observe topic output, kill it — matches the existing
  Zephyr/FreeRTOS workspace E2E). Do **not** add a bounded-spin mode to
  `run_tiers`. The runtime *mechanism* is already covered by
  `phase228_tier_filter.rs`.
- **Native multi-tier is advisory-priority** — `LinuxBoard::run_tiers` uses
  `std::thread` (default scheduler, no strict preemption); it validates the
  task-per-tier + filter shape + serves dev ergonomics. Real preemption is
  FreeRTOS/embedded. `SCHED_FIFO` via libc is a later optional add.
- **Node-pinned-to-tier (v1) accepted** — a node's groups all resolve to one
  tier; mixed-criticality within one node splits the package or routes through
  `[[shared_state]]`. v2 relaxes (§7, RFC-0015 §8).
- **RTIC / Embassy multi-tier is a non-goal v1** — those frameworks express
  priority via `#[task(priority)]` / Embassy executors; tiers there map to
  framework priorities, not `run_tiers`. `run_tiers` applies only to
  `OwnedSpin`-RTOS boards.

**Decided (2026-06-12 design discussion — FreeRTOS Entry-pkg bring-up, issue 0045):**

- **Embedded panic handler is board-owned, cfg-gated** (§3.1 rule 1). The board
  family crate carries `#[cfg(target_os = "none")] use panic_semihosting as _;`;
  the Entry pkg stays panic-free. Retires the board-descriptor `crate_root_extra`
  panic injection (which `nros::main!()` never consumed → the macro-emitted bin
  had no handler).
- **Node-pkg crate-type is deployment-path-specific** (§3.1 rule 2): `["rlib"]` on the
  pure-cargo Entry path, `["staticlib"]` on the cmake/Corrosion path. Corrects
  RFC-0024's "irreducible `["rlib","staticlib"]`".
- **Validated end-to-end:** with both rules + the linker-script sync,
  `freertos_rs_talker_entry` (`thumbv7m-none-eabi`) compiles, links, and boots
  through the board lifecycle under QEMU.

**Open (implementation):**

4. **Land the §3.1 boot-scaffold fixes (Phase 212.O.1).** Board panic line on the
   `nros-board-*-freertos` family; the 6 FreeRTOS Node examples → `["rlib"]`;
   linker-script `.cargo/config.toml` sync (audit all freertos examples for the
   `-Tlink.x` drift, ideally regen via `nros sync`). Tracked by phase-212 O.1
   + issue 0045.
5. **O.1 runtime residual (separate from the panic design).** After §3.1, the app
   task stack-overflows at Executor creation because the firmware links **both**
   rmw backends (`zpico_sys` zenoh + `nros_rmw_cyclonedds` via the Node's `nros`
   umbrella `rmw-cffi`) despite `deploy.rmw = "zenoh"`. This is RMW-backend
   selection (RFC-0031) + inline-arena/stack tuning, not entry codegen — un-ignore
   `freertos_run_plan_runtime` only after it lands.

1. **Proc-macro emit** — wire `nros-macros` to `nros-orchestration-ir`, resolve
   tiers at expansion (keying groups per §7 instance-identity), emit `run_tiers`
   behind the §5.1 gate with a register-only `run_plan`. Add a multi-tier example
   fixture + the external-observer E2E above.
2. **FreeRTOS `run_tiers`** — port the native primitive via
   `nros_freertos_create_task` at the raw FreeRTOS priority; this is where real
   preemption is validated on QEMU (MT=1 already, §5.0).
3. **Spin-period bound check** — emit a warning when `spin_period_us` exceeds the
   tightest timer period in the tier (RFC-0015 §4.3). Low priority.

## 8a. Embedded board adapter + NodeContext runtime binding (C++ Entry path)

The C++ Entry path (Phase 219) emits the launch tree → register sequence →
`NodeContext` dispatch, but only against `nros::board::LinuxBoard` and only
with a **recording** `NodeContextOps` (every op a no-op — see
`packages/api/nros-cpp/include/nros/main.hpp`). So a generated C++ `main()`
exercises codegen + symbol resolution + launch-order dispatch end-to-end, but
constructs **no** live publishers/subscriptions, on native or embedded.

**Decided (2026-06-11 design discussion, ASI as driving consumer):**

- **The `NodeContextOps` seam is the runtime binding point.** The recorded
  op set is replaced with a real one that maps each entity to an `nros-cpp`
  construction call (`create_node`, pub/sub/service/client/timer create,
  callback→poll wiring). No new IR; identity stays codegen-resolved (RFC-0024).
- **Embedded gets a sibling `Board::run()`** to `LinuxBoard`, owning the
  Zephyr + Cyclone `init → network-wait → register → spin → shutdown` ritual.
  It is selected through the **Phase 215** `nano_ros_use_board(<name>)` import
  (`board.cmake` feeds default RMW + runner).
- **ASI's working imperative runtime is the reference implementation.** The
  `actuation_module` `common/node` shim (`node_nros.hpp`
  `SubscriptionHandler<T>` + `create_publisher`/`create_subscription` over
  `nros::Node`) and its hand-written `main.cpp` boot ARE, respectively, the
  real `NodeContextOps` and the embedded `Board::run()`. This phase lifts that
  proven code under the seam rather than designing a runtime from scratch.
- **Tracked by Phase 236** (`docs/roadmap/phase-236-cpp-entry-embedded-runtime.md`),
  native-first (235.A) then embedded Zephyr (235.B), validated by ASI (235.C).

**Open (decide during Phase 236 impl):**

- **Callback bodies — RESOLVED by [RFC-0043](0043-entry-real-callback-binding.md)
  (`→` Phase 236.D).** The 236.A/B runtime constructs entities and *synthesizes* a
  `std_msgs/Int32` counter for a timer-`Publishes` binding; it runs **no real user
  callback bodies** (the talker/listener demo passes on the counter, but ASI's
  MPC/PID `Controller` cannot be driven). RFC-0043 resolves this **against** the
  first sub-decision of this section ("the `NodeContextOps` seam is the runtime
  binding point"): the type-erased string-descriptor register cannot carry a
  body, so the binding point is the **executor callback registration**, not the
  recording op set. The Entry path routes real callbacks to the Rust executor
  (RFC-0041); the component becomes a stateful object binding callbacks **by
  identity** (no naming, RFC-0019 thin wrapper); the synthesizing
  `EntryNodeRuntime` + `DeclaredNode`/`record_callback_effect` string layer are
  retired. Surfaced 2026-06-11 by ASI; the NuttX executor-callback path was
  spike-validated 2026-06-12. Board granularity resolved:
- **Board granularity** — RESOLVED (236.B): one `ZephyrBoard` parameterized by
  `board.cmake`, not per-board adapters — everything board-specific comes from
  the Phase 215 import + Kconfig at build time.
- **Entity handle storage** — ASI uses `std::shared_ptr<Publisher<M>>`; the
  `no_std` C++ Entry runtime needs an `alloc`-free equivalent (executor-owned
  arena, sized via the Phase 118.B opaque-size probe).
- **Parameter arrays** — ASI keeps `std::vector<double>` MPC weights in a local
  map because `nros::ParameterServer` is scalar-only; the Entry runtime inherits
  this gap until the parameter API grows sequences (separate phase).

---

## 9. References

- Execution model: RFC-0015 (esp. §2.2, §2.3, §3, §10).
- Embedded C bake: RFC-0003 §1.
- Per-RTOS scheduling: RFC-0016.
- Proc-macro: `packages/core/nros-macros/src/main_macro.rs`.
- CLI mirror + Plan IR: `packages/cli/nros-cli-core/src/codegen/entry/`.
- BoardEntry seam: `packages/platform/nros-platform/src/board/entry.rs`.
- `run_tiers` + `TierSpec`: `packages/boards/nros-board-linux/src/lib.rs`,
  `packages/platform/nros-platform/src/board/tier.rs`.
- Shared resolver: `packages/core/nros-orchestration-ir/`.
- Phase tracking: `docs/roadmap/phase-228-per-tier-orchestration-codegen.md`
  (multi-tier emit); `docs/roadmap/phase-236-cpp-entry-embedded-runtime.md`
  (C++ embedded board adapter + NodeContext runtime, §8a);
  `docs/roadmap/phase-212-ux-cargo-native-and-file-consolidation.md` §212.O.1
  (embedded panic + Node-pkg crate-type, §3.1).
- Embedded panic / crate-type: `docs/issues/0045-freertos-entry-component-staticlib-panic-handler.md`,
  RFC-0024 §6.4, RFC-0031 (rmw selection — the O.1 dual-backend residual).
