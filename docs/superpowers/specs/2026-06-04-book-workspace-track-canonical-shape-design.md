# Book: ROS-style workspace track + canonical-shape reconcile — Design

**Date:** 2026-06-04
**Status:** Approved-for-planning
**Scope:** `book/src/` only (docs). No code/example changes except flagged.

## Goal

Two outcomes in the nano-ros book:

1. **Keep the app-node starter, make every example-shape reference canonical.**
   The first-node tutorials stay as-is (single-package "app" talker is the
   canonical *starter* shape, by owner directive). Fix the few pages that
   still cite the pre-Phase-118 `examples/<plat>/<lang>/<rmw>/<example>/`
   path layout — RMW is selected at build time, not in the directory path.

2. **Add a ROS-style "grow into a workspace" tutorial track** in
   Getting Started, and reconcile the existing reference pages to the
   **3-role model** so the new track does not contradict them.

## Canonical model (anchor)

Authoritative source: `docs/design/0024-multi-node-workspace-layout.md` §11
("LOCKED 2026-06-03"), cross-checked against the **shipped `nros` 0.3.7
CLI** and on-disk examples. The three package roles:

| Role | Shape | Required? |
|---|---|---|
| **Node pkg** | language lib carrying `nros::node!()` (Rust) declarations; `package.xml` + `Cargo.toml`; `[package.metadata.nros.node]` | Yes — one per node |
| **Bringup pkg** | pure declarative: `package.xml` + `system.toml` + `launch/*.launch.xml` + `config/`. No `Cargo.toml`/`CMakeLists.txt`. `<system>_bringup` naming | Optional — only when ≥2 Entry pkgs share one topology; a single-Entry workspace folds `launch/` + `system.toml` into the Entry pkg |
| **Entry pkg** | language binary that boots a topology against a `Board`; `package.xml` + `Cargo.toml` + `src/main.rs` with `nros::main!(...)`; `[package.metadata.nros.entry] deploy = "<board>"` | Yes — one per deploy target |

Ground-truth facts the book must reflect (verified against installed CLI +
source + examples, **not** the WIP §2 of the design doc):

- **Macros:** `nros::node!(T)` is current; `nros::component!()` is **gone**.
  `nros::main!()` is current (four forms, §11.6). `[package.metadata.nros.node]`
  and `[package.metadata.nros.entry]` are current; `.component` is stale.
  C/C++ `NROS_NODE`/`NROS_MAIN` are **future** (Phase 216/219) — mark as such.
- **CLI verbs (shipped 0.3.7):** `new generate generate-rust codegen
  codegen-system metadata plan check explain config build deploy launch
  setup run monitor doctor board ws version completions`.
  - `nros build` / `nros deploy` **exist but delegate** to the per-platform
    build framework (auto-detect cargo / cmake / west / idf). They do **not**
    re-implement building. The design doc's §2 "nros never a build verb" is
    **superseded** — do not quote it.
  - `nros launch <bringup> [--launch <file>]` = host-side bringup spawner
    (reads `launch/` from source, no ament install). Native/`native_sim`
    alternative to `ros2 launch`.
  - `nros run` zephyr/qemu paths are "not yet wired" — keep that caveat.
- **Config files — both live, different jobs:**
  - Root `nros.toml` = deploy-target SSOT (`[deploy.<name>]`); the CLI reads
    it for `nros deploy`/`build <name>`. **Keep existing book references.**
  - `<bringup>/system.toml` = multi-node topology (`[system]`, `[[component]]`,
    `[deploy.<target>]`, `[[domain]]`, `[[bridge]]`). The new track introduces it.
  - Do not delete `nros.toml` references; position the two as complementary.
- **Launch XML = ROS 2 schema verbatim** (§11.5): `<launch> <arg> <node>
  <param> <remap> <group> <include>`; subs `$(find <pkg>)`, `$(var)`, `$(env)`.
  nav2/Autoware/turtlebot3 XML copy-pastes. Python `.launch.py` not v1.
- **On-disk reality:** Node-pkg example `examples/stm32f4/rust/talker_pkg/`,
  Entry-pkg `examples/stm32f4/rust/talker-embassy/`. **No bringup pkg example
  ships yet** — only `packages/testing/nros-tests/fixtures/orchestration_e2e/
  demo_pkg_bringup/system.toml`. The bringup tutorial page documents from that
  + the design doc and **flags the missing copy-out template**.

## Workstream A — canonical-shape fixes (mechanical)

Genuinely stale; fix:

- `book/src/internals/creating-examples.md`
  - Lines 14, 18, 104, 153: `examples/<plat>/<lang>/<rmw>/<example>/` →
    `examples/<plat>/<lang>/<example>/`. State RMW is chosen at build time
    (Cargo features / `-DNANO_ROS_RMW=` / Kconfig overlay), per CLAUDE.md.
  - Add one sentence: a single-package "app" example is the canonical
    *starter* shape; multi-package shape (Workstream B) kicks in at ≥2 nodes.
- `book/src/reference/c-api.md:76`: `examples/native/c/<rmw>/action-client/`
  → collapsed path.
- Re-grep `book/src` for any remaining `<rmw>`-in-path or literal
  `/zenoh/`,`/xrce/`,`/cyclonedds/` **directory** layers. **Do not touch**:
  `--features zenoh` (build-time RMW select = correct) or the real carve-out
  `examples/zephyr/cpp/cyclonedds/talker-aemv8r/` (a documented one-board-one-RMW
  reference, not the collapsed shape).

## Workstream B — 3-role reconcile + new tutorial track

### B1. Reconcile existing reference pages

- **`book/src/user-guide/component-and-entry-pkg.md`** → retitle
  **"Node, Bringup & Entry Packages"**. Edits (file is the cookbook):
  - "Component pkg" → "Node pkg" everywhere (note the ROS composable-node
    parallel once). `nros::component!()` → `nros::node!()`.
    `[package.metadata.nros.component]` → `[package.metadata.nros.node]`.
  - Reinstate **Bringup pkg** as an optional declarative role; delete the
    "replaces the retired Bringup pkg" claim (line 160) and the implication
    that Entry subsumes Bringup. Use the §11.1 table.
  - Verify/fix the `nros::main!` four-form description against §11.6.
  - Update the workspace-shape block to the §11.2 `src/`-rooted layout
    (`my_ws/src/{talker_pkg, demo_bringup, native_entry}`).
  - Update the quick-reference table rows.
- **Terminology drift sweep** — grep + fix "Component pkg" / `component!` /
  "Bringup retired" in `user-guide/workflow.md`, `user-guide/deployment.md`,
  `porting/board-trait.md`, `concepts/*` if present. (Targeted, not a rewrite.)
- **`book/src/reference/cli.md`** → make it a clean **usage reference**:
  - Add missing shipped verbs: `launch`, `ws`, `explain`, `codegen-system`,
    `run`, `monitor` (already partly present — verify each against
    `nros <verb> --help`).
  - Reframe `nros build` / `nros deploy` as **delegators to the per-platform
    build framework** (cargo/cmake/west/idf) — nros hands off, does not build.
  - Keep root `nros.toml` references (correct); add a short note pointing to
    `<bringup>/system.toml` for multi-node topology and link the new track.
  - Do not import the design-doc §2 "never a build verb" claim.
- **`book/src/concepts/comparison-vs-microros.md`** (lines 36–37): leave the
  `nros.toml`/deploy wording (it's accurate) but align any "Component" term.

### B2. New Getting Started track (4 pages)

Inserted in `SUMMARY.md` under **Getting Started**, after
`getting-started/your-own-msg-package.md`, before the troubleshooting page.
Each page is tutorial-voice (narrative, copy-paste steps), cites app-shape
examples + the 3-role examples on disk, and links the cookbook (reference)
+ the design doc. Mark designed-but-unshipped steps explicitly.

1. **`getting-started/workspace-from-app-node.md`** — *"From an app node to a
   workspace"*. Why/when you outgrow a single app (≥2 nodes, shared launch,
   per-board deploy). The 3-role map. A ROS↔nano-ros command table:
   `ros2 pkg create`↔`nros new`; `colcon build`↔`cargo build`/`cmake --build`
   (nros delegates); `ros2 launch`↔`nros launch` (+ `nros plan`/`check`).
   Ends by pointing at the next three pages.
2. **`getting-started/workspace-node-pkgs.md`** — *"Prepare node packages"*.
   Scaffold with `nros new`; the Node-pkg anatomy (`package.xml`, `Cargo.toml`
   `[package.metadata.nros.node]`, `src/lib.rs` with `nros::node!(T)` +
   `Node`/`ExecutableNode`); build with `cargo build`. Use
   `examples/stm32f4/rust/talker_pkg/` as the worked reference.
3. **`getting-started/workspace-bringup.md`** — *"Bringup: launch + system.toml"*.
   The optional declarative Bringup pkg; `system.toml` (`[system]`,
   `[[component]]`, `[deploy.<target>]`); `launch/*.launch.xml` in ROS 2 schema
   (the §11.5 tag/sub set, copy-paste from nav2); `nros plan` → `nros check`.
   Cite the new `examples/templates/multi-node-workspace/` (Workstream C) as the
   runnable copy-out; flag the `nros launch` step if not e2e-wired in 0.3.7.
4. **`getting-started/workspace-entry-pkg.md`** — *"Entry package: boot on a
   board"*. Per-board Entry pkg; `nros::main!()` four forms; native target +
   one embedded (FreeRTOS or the stm32f4 embassy example); `[package.metadata.
   nros.entry] deploy`; `nros launch`/`nros deploy native`; the manual escape
   hatch (`BoardEntry::run` / `Executor::open`, §11.8). Note C++ `NROS_MAIN`
   is future (Phase 219).

### B3. SUMMARY.md

Add under Getting Started (after line 14):
```
- [From an app node to a workspace](../../../book/src/getting-started/workspace-from-app-node.md)
- [Prepare node packages](../../../book/src/getting-started/workspace-node-pkgs.md)
- [Bringup: launch + system.toml](../../../book/src/getting-started/workspace-bringup.md)
- [Entry package: boot on a board](../../../book/src/getting-started/workspace-entry-pkg.md)
```

## Workstream C — runnable 3-role workspace template (owner-requested)

The bringup + entry tutorial pages must cite a runnable copy-out workspace.
The existing `examples/templates/multi-package-workspace/` is **stale-shaped**
(`src/pkg_*/src/main.rs` polyglot *app* nodes; no `nros::node!`, no Bringup/Entry,
no `system.toml`) — a different, still-valid lesson. **Do not mangle it.**

- **Create `examples/templates/multi-node-workspace/`** in the §11.2 shape:
  ```
  multi-node-workspace/
  ├── Cargo.toml                  # [workspace] members = src/talker_pkg, src/listener_pkg, src/robot_entry
  ├── .gitignore                  # /target/ /build/
  ├── README.md                   # copy-out instructions + ROS↔nano-ros map
  └── src/
      ├── talker_pkg/             # Node pkg  (package.xml + Cargo.toml [..nros.node] + src/lib.rs nros::node!(Talker))
      ├── listener_pkg/           # Node pkg
      ├── demo_bringup/           # Bringup pkg (package.xml + system.toml + launch/system.launch.xml + config/) — NO Cargo.toml
      └── robot_entry/            # Entry pkg (package.xml + Cargo.toml [..nros.entry] deploy=native + src/main.rs nros::main!(launch="demo_bringup:system.launch.xml"))
  ```
  Model node shape on `examples/stm32f4/rust/talker_pkg/`; bringup `system.toml`
  on the `demo_pkg_bringup` fixture; launch XML on the §11.5 tag set.
- **Verify it:** `cargo build` at template root succeeds; `nros plan demo_bringup`
  + `nros check` pass; attempt `nros launch demo_bringup` (or `nros deploy native`).
  If the host launch/e2e path is not fully wired in 0.3.7, document the build +
  plan + check steps as working and **flag** the launch step as the remaining gap
  rather than claiming a green e2e.
- Per-dir `.gitignore` (`/target/`, `/build/`, `/generated/`); commit **no**
  `target/` or `generated/` artifacts (the old template tracks some — out of scope).
- One-line note in `multi-package-workspace/README.md` pointing readers to the new
  `multi-node-workspace/` for the Node/Bringup/Entry pattern.
- Tutorial pages B2.3 (bringup) and B2.4 (entry) cite `multi-node-workspace/`.

## Scope guardrails

- First-node (app-node) pages unchanged.
- No invented CLI flags/verbs — every command shown is verified against
  `nros <verb> --help` (0.3.7) or cited from `reference/cli.md`.
- Designed-but-unshipped (C/C++ entry macros, `nros run` on zephyr/qemu,
  bringup copy-out template) explicitly flagged.
- No example/code edits except: if the terminology sweep reveals a trivially
  stale in-tree doc string blocking accuracy, flag — do not silently rewrite
  examples/ this pass.

## Validation

- `just book` (mdbook) builds clean; no broken intra-book links (new pages
  resolve, retitled cookbook anchor links updated).
- `grep -rn` over `book/src`: zero `<rmw>`-in-path citations remain (excluding
  the legitimate cases above); zero `nros::component!` / `[package.metadata.
  nros.component]`; zero "retired Bringup".
- Spot-read the 4 new pages: every command copy-pastable; not-yet-shipped
  steps marked.
- `examples/templates/multi-node-workspace/`: `cargo build` succeeds, `nros plan
  demo_bringup` + `nros check` pass; no `target/`/`generated/` committed.

## Open items to resolve during implementation

- Confirm `nros ws` / `nros explain` / `nros codegen-system` help text before
  documenting (run `--help`).
- Confirm whether `examples/templates/multi-package-workspace/` already
  reflects the 3-role `src/` shape; if stale, note in the bringup page rather
  than fixing examples here.
