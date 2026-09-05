---
rfc: 0024
title: "Phase 212 — Multi-Node Workspace Layout (LIVE DESIGN)"
status: Draft
since: 2026-06
last-reviewed: 2026-06
implements-tracked-by: [phase-212, phase-227]
supersedes: []
superseded-by: null
---

# Phase 212 — Multi-Node Workspace Layout (LIVE DESIGN)

## 1. Status & Audience

**LIVE doc, WIP.** Audience = phase-212 implementers + reviewers. Decisions marked **LOCKED** are settled; **OPEN:** marks live debate.

> **Resolution update (2026-06).** Several §8 open questions are now settled in
> code/book and the unified config model — read them as resolved:
> - **`system.toml` location** → in the bringup pkg; a workspace-root `nros.toml`
>   is *rejected* by the CLI. (RFC-0004.)
> - **`nros launch` vs `ros2 launch`** → the Entry-pkg binary *is* the launch
>   product; the standalone `launch_plan` wrapper was removed in 0.5.0.
> - **Orchestration pkg `Cargo.toml`** → Path A (no `Cargo.toml`); the workspace
>   root `exclude`s it.
> - **`[system].components` schema** → `[[component]]` tables (`pkg`/`class`/`name`),
>   not a flat list.
> - **`system.toml` scope** → it is the *universal* system descriptor; single-node
>   may omit it (implicit 1-component system synthesized). (RFC-0004 §2.)
>
> Configuration authority is RFC-0004; RMW selection is RFC-0031. **Embedded-MCU
> multi-pkg composition is also decided** (2026-06): one binary links N node-pkg
> staticlibs + a generated Entry; one Executor per priority tier + one shared
> session (RFC-0015); codegen is ahead-of-vendor with a configure-time convenience
> hook (RFC-0003 §7). Multi-node RT config has a home (RFC-0015 reconciliation /
> RFC-0004 §7). Genuinely-open items now narrow to **cross-language workflow**
> (Cluster C: Rust-cargo vs C++-cmake asymmetry, interface-pkg placement,
> mixed-language bootstrap) and multi-system shared config — tracked by phase-227.

> **Revision 2026-06-03 — Three roles: Bringup pkg + Node pkg + Entry pkg.**
> Supersedes the 2026-06-02 "Entry pkg subsumes Bringup pkg" revision.
> See §11 (LOCKED) for the canonical shape:
>
> - **Bringup pkg** — pure declarative (`package.xml` + `system.toml`
>   + `launch/*.launch.xml` + `config/`). No `Cargo.toml`, no
>   `CMakeLists.txt`. **Optional**: required only when ≥2 Entry pkgs
>   share one system topology (multi-target deployment). Single-Entry
>   workspaces fold the launch + system.toml into the Entry pkg.
> - **Node pkg** (renamed from "Component pkg") — Rust lib OR C++ lib
>   carrying one or more `nros::node!()` / `NROS_NODE()` declarations.
>   ROS 2 composable-node parallel.
> - **Entry pkg** — language-specific binary that boots the topology
>   against one Board. Rust: `nros::main!(...)` proc-macro reads
>   workspace pkg index + launch XML at expansion time and emits
>   `fn main`. C++ (future): `NROS_MAIN(...)` + `nros_entry(...)`
>   cmake fn with identical resolution semantics.
>
> Pkg discovery is **language-agnostic** — workspace walk for
> `package.xml` files, identical to ament. Launch XML is the **ROS 2
> schema verbatim** — copy-paste from nav2/Autoware works (tags +
> `$(find <pkg>)` resolved at build time). Python `.launch.py` not
> v1; XML only.
>
> §3-§10 below retain the historical 2026-06-02 Path-A bringup-pkg
> analysis as design context. The 2026-06-02 banner's claim that
> Bringup pkg is RETIRED is itself superseded — §11 reinstates it as
> optional.

---

## 2. Constraints (locked)

1. **ROS standard layout.** Launch files live in dedicated orchestration package (`<system>_bringup` convention). Component packages stay code-only.
2. **No colcon as primary orchestrator.** Error attribution (rustc/gcc diagnostics swallowed by `Failed <<<`), embedded targets ignored, install/ tree dead weight for MCUs, cross-language codegen invisible to colcon DAG. Colcon stays *available* for Autoware-style outer integration via `colcon-cargo-ros2` seam; never the inner workflow.
3. **cargo / cmake stay user-facing.** `cargo build` works at workspace root for Rust-majority. `cmake --build build` works for C++-majority. Rustc errors stay rustc errors.
4. **nros never a build verb.** No `nros build` / `nros test` / `nros flash`. nros = provisioner + codegen + metadata. Idf.py-shaped, not colcon-shaped.
   > **AMENDED 2026-08-25 by [RFC-0065](0065-colcon-like-workspace-builder.md) §D1.** The
   > objection recorded in §9 is to *wrapping the compiler* — "hides cargo/cmake
   > diagnostics" — not to *deriving the root build file*. `nros build` returns with two
   > properties that keep §2.2 and §2.3 intact: its final stage is an **`exec`**, not a
   > pipe, so diagnostics are byte-identical to the native tool's; and what it generates
   > is a **real build tree** a user can drive by hand thereafter. `nros test` and `nros
   > flash` stay rejected — neither has a derivation to perform, so both would be pure
   > wrapping.
5. **One-package workflow stays canonical for tiny fixtures.** Multi-package shape kicks in at ≥2 components. (Phase 212 already decided single-package workflow user-facing.)

---

## 3. Reference patterns

- **nav2 / Autoware / turtlebot3** — `<system>_bringup` carries `launch/`, `config/`, `package.xml` with only `<exec_depend>` lines pulling in component packages. No `<build_depend>` — orchestration role is pure runtime resolution. **Takeaway:** orchestration package is a *role*, not a build artifact. Zero compiled code.
- **Cargo workspace** — single dependency graph, unified feature DAG per (package, target), `-j N` rayon scheduler. Build-scripts are crates-first. **Takeaway:** cargo IS the orchestrator for Rust; do not wrap it.
- **CMake `add_subdirectory` vs `ExternalProject_Add`** — former shares cache + targets, latter isolates. **Takeaway:** own-code uses `add_subdirectory`; corrosion provides bidirectional cargo↔cmake bridge.
- **Zephyr west + ESP-IDF idf.py + PlatformIO** — all share shape: SSoT manifest at root (`west.yml` / `platformio.ini` / `idf` component scan) + per-component manifest (`module.yml` / `library.json` / `idf_component_register`). Tool synthesizes one CMake/SCons graph at invocation. **Takeaway:** root manifest + per-package manifest is the dominant pattern; nros.toml follows it.
- **Corrosion** — already used Phase 175.A. `corrosion_import_crate` makes Rust staticlib a normal CMake target. **Takeaway:** when graph crosses languages, CMake should be the top-level driver because cargo cannot consume CMake targets in reverse.

Citations: `docs/design/0027-ros2-user-workflow.md`, `docs/roadmap/phase-212-ux-cargo-native-and-file-consolidation.md` (lines 473–731), `nros-cli/packages/colcon-cargo-ros2/`, `CLAUDE.md` Examples + CMake Path Convention sections.

---

## 4. The orchestration package

**LOCKED shape:**

```
demo_bringup/
├── package.xml         # <name>demo_bringup</name>
│                       # <buildtool_depend>ament_cmake</buildtool_depend>  (or absent — see OPEN below)
│                       # <exec_depend>talker_pkg</exec_depend>
│                       # <exec_depend>listener_pkg</exec_depend>
├── system.toml         # [system] launch=..., components=[...], rmw=..., domain_id=...
│                       # [deploy.native] / [deploy.qemu-mps2-an385] / ...
│                       # [[domain]] / [[bridge]]
├── launch/
│   └── system.launch.xml
├── config/             # optional — params.yaml, rviz, etc.
└── README.md
```

**No `Cargo.toml`. No `CMakeLists.txt`. No `src/`.** Pure declarative.

**Naming: `<system>_bringup`.** Aligns nav2/Autoware/turtlebot3. Accept `<system>_launch` as documented alias. Reject plain `<system>` (collides w/ ament metapackage idiom).

**Dependencies — two layers, both mandatory:**
- `package.xml` `<exec_depend>` → ament/colcon discovery + install ordering when used inside outer-colcon workspace.
- `system.toml` `[system].components` → nros planner's authoritative runtime set.

`nros check` cross-validates the two. Single source = `system.toml`; `nros emit package-xml` regenerates `<exec_depend>` block (mirrors Phase 212.C `[package.metadata.ament]` pattern).

**Lint:** `nros check` rejects orchestration pkg w/ `[[bin]]`/`[lib]`/`add_executable`. Code goes in sibling component pkg.

**OPEN: should orchestration pkg ship a stub Cargo.toml?** Two paths:
- **Path A** (recommended): no Cargo.toml. Pkg not a cargo workspace member. `nros plan demo_bringup` finds it via dir walk + `package.xml`. Pro: cleaner, no fake `lib.rs`. Con: `nros plan` must walk outside `[workspace] members`.
- **Path B**: stub `Cargo.toml` w/ empty lib. Pkg IS workspace member. Pro: `nros plan -p demo_bringup` works via cargo's `-p` flag. Con: fake target pollutes `cargo build` output, needs `[lib] path = "src/lib.rs"` w/ empty file.

Leaning A. Need to prototype `nros plan <dir>` discovery first.

**OPEN: `buildtool_depend` in `package.xml`?** ament_cmake assumes empty CMakeLists installs `share/<pkg>/launch/`. Without colcon in inner loop, who installs? the codegen step reads `launch/` directly from the source tree — no install step. Maybe omit `<buildtool_depend>` entirely. Need to check if `ros2 launch` (when user *does* run colcon outside) still resolves.

---

## 5. The workspace root

**Workspace root = thin pointer, not a system definition.**

```
my_ws/
├── Cargo.toml          # [workspace] members = ["talker_pkg", "listener_pkg"]
│                       #             exclude  = ["demo_bringup"]    (if Path A from §4)
│                       # [workspace.metadata.nros]
│                       #   default_system = "demo_bringup"
│                       #   # optional global overrides:
│                       #   # rmw_override = "cyclonedds"
├── CMakeLists.txt      # OPTIONAL — only for C++-majority workspaces
│                       # project(my_ws); include(nano_ros_workspace_metadata)
│                       # nano_ros_workspace_metadata(SYSTEM demo_bringup)
│                       # add_subdirectory(listener_pkg)
├── .gitignore          # /target/  /build/  /install/
└── (component pkgs + bringup pkg, siblings)
```

**No workspace-root `nros.toml`.** Retired. System definition lives in `<bringup>/system.toml`. Rationale: matches ROS muscle memory (every nav2/Autoware tutorial points users at `nav2_bringup/launch/`, not a root TOML). Decouples workspace's build graph from the system graph.

**Workspace-root metadata reduced to:**
- `default_system` — disambiguates `nros plan` with no args.
- Optional global RMW / deploy-target overrides (rare; for `nros plan --override` workflows).

**Per-system definition** (`<bringup>/system.toml`):
- `[system]` — components list, launch file, default RMW, default domain.
- `[deploy.<target>]` — per-target overrides.
- `[[domain]]` / `[[bridge]]` — per-system topology.

**No duplication.** Root pointer + per-system definition is two different concerns. The temptation to mirror per-system fields at root (e.g. `[workspace.metadata.nros.system.demo]`) is rejected — re-creates colcon's monorepo-of-systems pattern + breaks per-system `<exec_depend>` hygiene.

**OPEN: multiple bringup pkgs sharing fragments?** If `sim_bringup` and `field_bringup` share 80% of `[[domain]]`/`[[bridge]]` config, where does the shared fragment live? Options:
- (a) Duplicate. Honest, traceable, no magic. Painful at scale.
- (b) `include = "../shared/domains.toml"` in `system.toml`. nros expands. Adds path resolution semantics.
- (c) Workspace-root `[workspace.metadata.nros.defaults]` table that per-system TOMLs inherit + override.

Leaning (a) until a real workspace hits the pain. Don't pre-build inheritance.

---

## 6. Build orchestration without colcon

### 6.1 Rust-majority workspace (cargo top-level)

`cargo build` at workspace root → cargo's native scheduler builds all `[workspace] members`. Each component crate has `[package.metadata.nros.component]` table + `nros-build` build-dep (Phase 212.B). `build.rs` reads sibling `*.msg` via `cargo:rerun-if-changed=` for incremental correctness.

Orchestration pkg excluded from `[workspace] members` (Path A) → never built by cargo. `nros plan demo_bringup` invoked separately reads `system.toml` + workspace component manifests → emits `target/nros/demo_bringup/plan.json`. No build step.

Diagnostics path unchanged: rustc → stderr → user terminal. No colcon wrapping.

### 6.2 C++-majority workspace (cmake top-level)

`cmake -S . -B build && cmake --build build` at workspace root. Root `CMakeLists.txt`:

```cmake
project(my_ws)
find_package(NanoRos REQUIRED)   # NO — Phase 140 deleted this
                                  # actually: add_subdirectory(<nano-ros-repo-root>)
include(nano_ros_workspace_metadata)
nano_ros_workspace_metadata(SYSTEM demo_bringup)   # shells `nros plan` at configure time
add_subdirectory(talker_pkg)
add_subdirectory(listener_pkg)
```

`nano_ros_workspace_metadata` (≤150 LoC, Phase 212.D) shells `nros plan <bringup>` at *configure* time, emits a generated `nros_components.cmake`, `include()`s it. CMake sees component targets natively.

Diagnostics: gcc/clang → stderr → user terminal. No wrapping.

### 6.3 Mixed Rust + C++ workspace

**CMake is the top-level driver.** Rule: cargo can be consumed as a cmake target (via Corrosion); cmake cannot be consumed as a cargo target. So when graph crosses languages, cmake wins:

```cmake
project(my_ws)
add_subdirectory(<nano-ros-repo-root>)
include(nano_ros_workspace_metadata)
nano_ros_workspace_metadata(SYSTEM demo_bringup)

corrosion_import_crate(MANIFEST_PATH talker_pkg/Cargo.toml)
add_subdirectory(listener_pkg)
target_link_libraries(listener_pkg PRIVATE NanoRos::NanoRos talker_pkg)
```

For pure-Rust workspaces, cargo stays top-level — no cmake needed.

**Cross-language topo-ordering** at workspace level (e.g. "build all C++ RMW backends before Rust examples linking them") handled by `nros plan` emitting a topo DAG `plan.json`. Per-component dep resolution stays inside corrosion + build.rs.

### 6.4 Embedded path (Zephyr / FreeRTOS / ESP-IDF)

Untouched. Per-RTOS shells at `integrations/<rtos>/` (Phase 139/140) re-export root CMake. `west build` / `idf.py build` drive their own ninja graph; nros is the provisioner + codegen layer underneath. Orchestration package's `launch/` is irrelevant on MCU — flashed `.elf` runs `nros::init` from baked config.

**OPEN:** does the orchestration pkg's `system.toml` get baked into MCU firmware (like `app_config.h` today)? Or is `system.toml` purely host-side (host-tool reads to drive deploy/flash)? Leaning host-side. Per-node `[package.metadata.nros.component]` in component crate's `Cargo.toml` stays the bake source.

**Node-pkg `crate-type` is deployment-path-specific (corrected 2026-06-12, [RFC-0032 §3.1](0032-entry-codegen-pipeline.md), [issue 0045](../issues/archived/0045-freertos-entry-component-staticlib-panic-handler.md)).** An earlier draft listed `crate-type = ["rlib", "staticlib"]` as an *irreducible* Node-pkg item; that conflated two build paths:

- **Pure-cargo Entry path** (Rust-native FreeRTOS / NuttX / ThreadX) — the Entry pkg links the Node as an **rlib** only. The Node declares `crate-type = ["rlib"]`. A redundant `staticlib` here is dead weight that still forces rustc to demand a no_std `#[panic_handler]` the rlib must not carry (it would collide with the board-owned handler — RFC-0032 §3.1).
- **cmake / Corrosion path** (C-owned firmware, RFC-0003 bake) — the C firmware links the Node's **staticlib**. The Node declares `crate-type = ["staticlib"]` (the threadx fixtures already do, host-linked). Corrosion 0.5 `CRATE_TYPES` only selects from *declared* types, so this path must declare the staticlib.

A Node reused across both paths needs both declarations split by path (e.g. per-target manifest or a thin staticlib wrapper) — it is **not** a single universal `["rlib","staticlib"]`.

---

## 7. End-to-end user workflow

### Step 1 — Scaffold workspace

```bash
mkdir my_ws && cd my_ws
nros new system robot_bringup --components talker,listener   # NROS
```

Tree:
```
my_ws/
├── Cargo.toml                      # [workspace] members=["talker","listener"]
│                                   # [workspace.metadata.nros] default_system="robot_bringup"
├── robot_bringup/
│   ├── package.xml
│   ├── system.toml
│   └── launch/system.launch.xml
├── talker/
│   ├── Cargo.toml
│   └── src/lib.rs                  # #[nros::component] stub
└── listener/
    ├── Cargo.toml
    └── src/lib.rs
```

### Step 2 — Edit components

User edits `talker/src/lib.rs` + `listener/src/lib.rs`. Normal Rust authoring.

### Step 3 — Build

```bash
cargo build                                                  # CARGO
```

Cargo workspace builds talker + listener. Build-scripts (`nros-build`) regenerate message bindings on `.msg` change. Orchestration pkg untouched (excluded from workspace members).

### Step 4 — Plan + verify

```bash
nros check                                              # NROS (transitive via cargo subcmd)
nros plan                                               # NROS (default_system = robot_bringup)
```

`check` cross-validates `<exec_depend>` ↔ `[system].components`. `plan` emits `target/nros/robot_bringup/plan.json`.

### Step 5 — Deploy + launch

```bash
cargo run -p native_entry            # native run (no nros deploy — Phase 222)
ros2 launch robot_bringup system.launch.xml                  # ROS 2 (when wanted)
# or
nros launch robot_bringup                                     # NROS (host-side, no ament install needed)
```

Multiple processes spawn per `plan.json`. Each reads its baked domain/RMW config.

**OPEN:** is `nros launch` a real verb? Or does the user always go through `ros2 launch` after a (one-time) ament install of `<bringup>/launch/`? Conflict: §2 constraint says "no colcon as primary" — but `ros2 launch` reads ament index. Maybe `nros launch` parses the same `system.launch.xml` independently of ament. Need to scope.

### C++-majority variant (step 3 replacement)

```bash
cmake -S . -B build && cmake --build build                   # CMAKE
```

Steps 4–5 unchanged. Step 4's `nros plan` becomes either bare `nros plan robot_bringup` (no cargo subcmd) or `cmake --build build --target nros-plan`. **OPEN** — see §8.

---

## 8. Open questions

1. **Orchestration pkg `Cargo.toml`?** (Path A no-toml vs Path B stub-toml.) Decision blocked on prototype: can `nros plan <dir>` cleanly walk outside `[workspace] members`? §4.
2. **Multi-system shared config.** Duplicate vs `include =` vs workspace-root `[defaults]`. Wait for real pain. §5.
3. **`nros launch` vs `ros2 launch`.** Host-side launcher independent of ament, or always shell to `ros2 launch` after a one-off install? Affects whether orchestration pkg needs `<buildtool_depend>ament_cmake</buildtool_depend>`. §7.
4. **RESOLVED:** **C++ workspaces — `cmake nros` subcommand?** No cmake plugin idiom. C++ users invoke `nros plan` / `nros codegen-system` directly, then build with native `cmake --build` (no `nros deploy`). Asymmetric vs cargo's `nros plan`. Phase 212 line 670 already accepts this asymmetry as honest. §6.
5. **Does `system.toml` belong to the orchestration pkg or stay workspace-root?** This doc says move to bringup pkg. Argument for staying root: a workspace w/ exactly one bringup pkg has indirection-for-nothing. Argument for moving: multi-system workspaces, ROS muscle memory, decouples build graph from system graph. Leaning move. §5.
6. **`[system].components` schema.** List of crate names, or list of `{name, role, qos_overrides}` tables? Today's `nros.toml` already has per-component override blocks. Where do they live in the split? Leaning: simple list in `[system].components`; per-component QoS lives in component crate's `[package.metadata.nros.component]`. Cross-cutting overrides go in `[[deploy.*]]`. §4.
7. **RESOLVED:** **Mixed-language workspace bootstrap.** First-time user runs `cargo build` against a workspace containing a C++ component pkg — what happens? Cargo ignores non-Cargo dirs. User must know to `cmake -S . -B build` instead. Mixed-language uses the CMake-boss + Corrosion flow (RFC-0025 Case 5): CMake drives the cross-language graph, Corrosion imports the Rust crates. Options once were: (a) document, (b) generate a top-level `Makefile` shim, (c) `nros build` (rejected by constraint 4). Resolved to CMake-boss + Corrosion. §6.3.
8. **Codegen interface package shape.** Where does `my_interfaces/` (a `.msg`-only package) sit? Today: `packages/interfaces/<pkg>/` w/ codegen via `nros generate-rust`. In multi-pkg workspace: sibling `my_interfaces/` pkg w/ `package.xml` declaring `<member_of_group>rosidl_interface_packages</member_of_group>`? Component crates `cargo:rerun-if-changed=` against it. Not yet sketched.
9. **Embedded MCU + multi-pkg workspace.** Multi-component on Zephyr: does each component get its own `west` app, or one app composing multiple components via Kconfig? Phase 172.K.5 (per-node multi-domain routing) suggests one-app-N-components. Need pattern check w/ §7's launch step.

---

## 9. Rejected alternatives (so far)

- **Colcon inner loop.** Error attribution, embedded coverage, install/ overhead. §2 constraint 2.
- **`nros build` / `nros test` / `nros flash`.** Re-creates colcon's wrapping anti-pattern; hides cargo/cmake diagnostics. §2 constraint 4. **`nros build` re-admitted 2026-08-25 as a generate-then-`exec` builder — see the amendment note on §2 constraint 4 and [RFC-0065](0065-colcon-like-workspace-builder.md). `test` / `flash` remain rejected.**
- **Single workspace-root `nros.toml` w/ `[system.<name>]` sub-tables.** Re-creates colcon monorepo-of-unrelated-systems pattern; breaks per-system `<exec_depend>` hygiene. §5.
- **Bringup pkg ships `CMakeLists.txt` + empty `install(DIRECTORY launch ...)`.** Drags ament_cmake into a pure-Rust workspace for zero benefit. nros reads `launch/` from source. §4.
- **`find_package(NanoRos)` consumption.** Already deleted Phase 140. Confirmed not coming back. Consumption stays `add_subdirectory(<repo-root>)`. §6.2.
- **Plain `<system>` naming (no `_bringup` suffix).** Collides w/ ament metapackage idiom. Forces awkward `<exec_depend>demo</exec_depend>` reading. §4.

---

## 10. Next concrete steps

1. **Prototype 3-package fixture** at `packages/testing/nros-tests/fixtures/multi_pkg_workspace/`: `demo_bringup` + `talker_pkg` + `listener_pkg`. Path A (no Cargo.toml in bringup). Run `cargo build` + `nros plan demo_bringup` + `cargo run -p native_entry`. Confirm cargo workspace happy w/ excluded pkg.
2. **Spike `nros emit package-xml`** from `system.toml`. Validate against colcon-outer workflow (run `colcon build` on the fixture inside a host ROS 2 install). Confirms Autoware-style outer integration unbroken.
3. **Spike mixed-language fixture**: `talker_pkg` (Rust) + `listener_pkg` (C++) + `demo_bringup`. Top-level `CMakeLists.txt` + `corrosion_import_crate`. Confirm rustc errors still reach terminal verbatim through cmake.
4. **Resolve OPEN 3** (`nros launch`). Prototype host-side launcher reading `system.launch.xml` w/o ament index. If clean, retire `<buildtool_depend>` from bringup pkg.
5. **Document `nros plan <dir>` discovery semantics** once Path A vs B settled. Update Phase 212.B writeup.
6. **Validate OPEN 9 (embedded multi-component)** on Zephyr w/ a 2-component bringup → one west app linking both. Phase 172.K.5 generator output should already cover this; confirm.
7. **Update `docs/design/0027-ros2-user-workflow.md` §"nros new system"** scaffolding to match §4 LOCKED shape (Path A, no Cargo.toml in bringup). Today's writeup pre-dates this design doc.

---

## 11. LOCKED 2026-06-03 — Three-pkg-role shape + ROS 2 launch compat

Supersedes §4-§5 + §7's bringup-pkg shape. Driven by two user constraints:
**(a) ROS 2 launch.xml from nav2/Autoware must copy-paste and Just Work;
(b) C++ Entry pkg support is available through CMake, so resolution must be
language-agnostic.**

### 11.1 Three pkg roles

| role | shape | required? |
|---|---|---|
| **Bringup pkg** | pure declarative: `package.xml` + `system.toml` + `launch/*.launch.xml` + `config/`. No Cargo.toml, no CMakeLists.txt. | Optional — only when ≥2 Entry pkgs share one topology |
| **Node pkg** | language-specific lib carrying `nros::node!()` (Rust) / `NROS_NODE()` (C++) declarations. `package.xml` + `Cargo.toml` (Rust) / `CMakeLists.txt` (C++). | Yes (one per node) |
| **Entry pkg** | language-specific binary that boots a topology against a Board. `package.xml` + `Cargo.toml` + `src/main.rs` with `nros::main!(...)` (Rust). C++ future: `CMakeLists.txt` with `nros_entry(...)` cmake fn + `main.cpp` with `NROS_MAIN(...)`. | Yes (one per deploy target) |

"Component pkg" terminology retires in favour of "Node pkg" — matches
ROS 2 composable-node naming.

**Phase 212.J.5 resolution (2026-06-03).** Bringup pkg `package.xml`
omits `<buildtool_depend>` since `nros launch` reads `launch/` from the
source tree, not from an install share path. Users wanting `ros2
launch <bringup>` compatibility add
`<buildtool_depend>ament_cmake</buildtool_depend>` themselves. The
`nros` CLI's generator (`nros emit package-xml`, internal helper) skips
the tag for the same reason; closes OPEN 3 in §10 and the §4 open
question.

### 11.2 Canonical workspace layout

```
my_ws/
├── Cargo.toml                            # [workspace] (Rust path)
├── CMakeLists.txt                        # OPTIONAL (C++-majority path)
├── .colcon_workspace                     # OPTIONAL marker for ament discovery
└── src/
    ├── talker_pkg/                       # Rust Node pkg
    │   ├── package.xml
    │   ├── Cargo.toml
    │   └── src/lib.rs
    ├── perception_cpp/                   # C++ Node pkg (future)
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   └── src/perception.cpp
    ├── demo_bringup/                     # Bringup pkg (pure declarative)
    │   ├── package.xml
    │   ├── system.toml                   # [system] + [deploy.<target>]
    │   ├── launch/
    │   │   ├── system.launch.xml         # default
    │   │   ├── talker_only.launch.xml
    │   │   └── sim.launch.xml
    │   ├── config/
    │   │   └── params.yaml
    │   └── README.md
    └── native_entry/                     # Entry pkg (Rust)
        ├── package.xml
        ├── Cargo.toml
        └── src/main.rs                   # nros::main!(launch = "demo_bringup:system.launch.xml")
```

For a workspace with one Entry pkg + one Node pkg, fold the bringup
pkg's `launch/` + `system.toml` into the Entry pkg directly. No
duplication SSoT problem because there's only one boot path.

### 11.3 Bringup pkg with multiple launch files (nav2 convention)

`<system>_bringup/launch/` holds many `.launch.xml` files. Each is a
distinct topology entry point. `system.toml` carries
`[system] default_launch = "system.launch.xml"`. Users select with
`nros launch <bringup-pkg> [--launch <file>]` or with macro
`launch = "<bringup>:<file>"`. NOT split across `<sub>_launch` pkgs.

Naming: `<system>_bringup` default; `<system>_launch` accepted alias.
Matches nav2 / Autoware / turtlebot3.

### 11.4 Pkg discovery — workspace walk

Language-agnostic. Build-time mechanism (compile-time for proc-macro,
configure-time for cmake fn):

1. **Workspace root detection** — walk up from
   `CARGO_MANIFEST_DIR` / `CMAKE_SOURCE_DIR` looking for, in order:
   1. `NROS_WORKSPACE_ROOT` env var (explicit override).
   2. `.colcon_workspace` marker or `COLCON_IGNORE` ancestor.
   3. `Cargo.toml` containing `[workspace]`.
   4. `.git/` (last-resort fallback).
2. **Pkg-index build** — recurse from workspace root, collect every
   `package.xml`. Pkg name = `<name>` element; pkg dir = parent dir.
3. **Cache** — emit `$OUT_DIR/.nros-pkg-index.json` keyed on combined
   `package.xml` mtimes. Re-scan only when mtime changes.

Identical algorithm runs from Rust proc-macro AND C++ cmake fn — the
shared logic lives in `nros-build` library, surfaced through both
front-ends.

### 11.5 Launch XML — ROS 2 schema verbatim

Tags supported at v1:

- `<launch>` — root.
- `<arg name="..." default="..." value="..." />` — launch arg.
- `<node pkg="..." exec="..." name="..." namespace="..." />` — spawn.
- `<param name="..." value="..." />` — per-node param.
- `<remap from="..." to="..." />` — topic/service remap.
- `<group ns="..." />` — namespace wrapper.
- `<include file="..." />` — recursive XML pull; args pass-through.

Substitutions supported v1:

- `$(find <pkg>)` — resolves to pkg source dir via the workspace pkg-
  index.
- `$(var <arg>)` — launch-arg reference.
- `$(env <name>)` — env lookup at build time.

Stock ROS 2 launch.xml from nav2 / Autoware / turtlebot3 paste in,
**Just Works** modulo unsupported tags. Python `.launch.py` form is
NOT supported v1; require XML.

### 11.6 Macro surface — Rust

```rust
// Single-node self-bringup — reads [package.metadata.nros.entry] deploy
nros::main!();

// Single-node, explicit board override
nros::main!(board = LinuxBoard);

// Multi-node, default launch from bringup pkg's system.toml
nros::main!(launch = "demo_bringup");

// Multi-node, explicit launch file
nros::main!(launch = "demo_bringup:sim.launch.xml");

// All explicit
nros::main!(
    board  = LinuxBoard,
    launch = "demo_bringup:sim.launch.xml",
    args   = [("use_sim", "true")],
);
```

One macro, four forms. Replaces today's `build.rs + include!()`
shape end-to-end — Entry pkg `Cargo.toml` drops the `nros-build`
build-dep, `main.rs` collapses to one line.

### 11.7 Macro surface — C++ (future)

Design parity:

```cpp
#include <nros/main.hpp>
NROS_MAIN(nros::board::LinuxBoard, "demo_bringup:sim.launch.xml");
```

```cmake
find_package(nano_ros REQUIRED)
nros_entry(
    NAME    native_entry
    BOARD   native
    LAUNCH  "demo_bringup:sim.launch.xml"
)
```

Both call shared `nros-build` codegen via cmake-driven shell. Same
pkg-index + launch parser + emitted boot stub.

### 11.8 Custom spin loop — escape hatch

When users want their own executor lifecycle, they skip
`nros::main!()` and write `main.rs` directly:

```rust
use nros::{Executor, ExecutorConfig, BoardEntry};
use nros_board_linux::LinuxBoard;

fn main() {
    let outcome = <LinuxBoard as BoardEntry>::run(|runtime| {
        runtime.runtime.register_dispatch_slot_dyn(
            __nros_component_talker_register,
            __nros_component_talker_init,
            __nros_component_talker_dispatch,
            __nros_component_talker_tick,
            "talker_pkg",
        )?;
        Ok(())
    });
    outcome.unwrap();
}
```

Or, even fully manual:

```rust
fn main() -> Result<(), Box<dyn Error>> {
    let executor = nros::Executor::open(&ExecutorConfig::default())?;
    let node = executor.create_node("my_talker")?;
    let publisher = node.create_publisher::<Int32>("/chatter", QoS::default())?;
    loop {
        publisher.publish(&Int32 { data: 0 })?;
        std::thread::sleep(Duration::from_secs(1));
    }
}
```

`nros::main!()` is convenience; `BoardEntry::run` + manual register
is the universal escape hatch; pure `Executor::open` is the rclcpp-
style direct path.

### 11.9 Rejected alternatives (this round)

- **Cargo-metadata-based pkg discovery.** Rust-only; would force a
  stub `Cargo.toml` on bringup pkgs; doesn't extend to C++. Replaced
  by language-agnostic workspace walk.
- **Workspace-root `[workspace.metadata.nros.bringups]` index.**
  Adding a bringup pkg = 2 edits (create dir + register in root
  table). Silent breakage on missed registration. Replaced by
  automatic `package.xml` walk.
- **Bringup pkg mandatory.** Over-prescription for single-target
  workspaces.
- **Custom launch XML schema.** Breaks copy-paste from
  nav2/Autoware/turtlebot3.
- **Python `.launch.py` v1.** Needs a Python interpreter at build
  time; revisit on demand.
- **Splitting launches across `<sub>_launch` sibling pkgs.**
  Un-ROS-2-idiomatic at our scale (nav2 et al. ship one bringup pkg
  with many launches inside).

### 11.10 Work items (cross-refs to phase doc)

- **N.5** (scope updated) — single-node codegen path. `nros::main!()`
  no-arg form reads `[package.metadata.nros.entry] deploy = "X"`,
  emits Board boot + this-pkg register.
- **N.9** — `nros::main!()` / `nros::launch!()` proc-macro family.
- **N.10** — workspace-walk pkg-index + `$(find <pkg>)` resolver.
- **N.11** — ROS 2 launch.xml parser supporting the v1 tag set.
- **N.12** — Component → Node rename sweep (mechanical, single wave).
- **(future)** — C++ Entry pkg surface (`NROS_MAIN` + `nros_entry`).
