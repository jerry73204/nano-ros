# Book Workspace Track + Canonical-Shape Reconcile — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every example-shape reference in the book canonical, reconcile the workspace docs to the 3-role model (Node/Bringup/Entry), add a 4-page ROS-style "grow into a workspace" tutorial track, and ship a runnable 3-role `multi-node-workspace` template the tutorial cites.

**Architecture:** Pure docs + one new example template. Branch `docs/book-workspace-track` (already created). Edits are surgical (grep-verified); new pages are tutorial-voice and cite real on-disk examples. The CLI reference is made accurate to the shipped `nros` 0.3.7 binary.

**Tech Stack:** mdbook (`just book`), markdown, `nros` CLI 0.3.7, Cargo workspace, ROS 2 launch XML.

**Spec:** `docs/superpowers/specs/2026-06-04-book-workspace-track-canonical-shape-design.md`

---

## SHARED FACT SHEET — read before any task

These are verified against the shipped CLI + source + on-disk examples. Use them verbatim; do **not** copy from the WIP design doc §2 (it says "nros never a build verb" — that is **superseded**).

**Three package roles** (`docs/design/0024-multi-node-workspace-layout.md` §11, LOCKED 2026-06-03):
- **Node pkg** — lib with `nros::node!(T)`; `package.xml` + `Cargo.toml` with `[package.metadata.nros.node]`. One per node. (Old name "Component pkg" is **retired**.)
- **Bringup pkg** — pure declarative: `package.xml` + `system.toml` + `launch/*.launch.xml` + `config/`. **No** `Cargo.toml`/`CMakeLists.txt`. Naming `<system>_bringup`. **Optional** — only when ≥2 Entry pkgs share one topology; a single-Entry workspace folds `launch/` + `system.toml` into the Entry pkg.
- **Entry pkg** — binary booting a topology against a `Board`; `package.xml` + `Cargo.toml` with `[package.metadata.nros.entry] deploy = "<board>"` + `src/main.rs` with `nros::main!(...)`. One per deploy target.

**Macros (current):** `nros::node!(T)`, `nros::main!()`. Gone: `nros::component!()`. C/C++ `NROS_NODE`/`NROS_MAIN` are **future** (Phase 216/219) — mark as such whenever mentioned.

**`nros::main!()` four forms** (§11.6):
```rust
nros::main!();                                       // single-node self-bringup (reads [..nros.entry] deploy)
nros::main!(board = LinuxBoard);                    // single-node, explicit board
nros::main!(launch = "demo_bringup");                // multi-node, default launch from system.toml
nros::main!(launch = "demo_bringup:sim.launch.xml"); // multi-node, explicit file
nros::main!(board = LinuxBoard, launch = "demo_bringup:sim.launch.xml", args = [("use_sim","true")]);
```

**Escape hatch** (§11.8): skip the macro, call `<LinuxBoard as BoardEntry>::run(|runtime| { ... })`, or go fully manual with `nros::Executor::open(&ExecutorConfig::default())`.

**Shipped CLI verbs (`nros` 0.3.7):** `new generate generate-rust codegen codegen-system metadata plan check explain config build deploy launch setup run monitor doctor board ws version completions`.
- `nros build` / `nros deploy` **exist but DELEGATE** to the per-platform build framework (auto-detect cargo / cmake / west / idf). nros hands the build off; it does not re-implement it.
- `nros launch <bringup> [--launch <file>]` = host-side bringup spawner; reads `launch/` from source, **no ament install**; the native/`native_sim` alternative to `ros2 launch`.
- `nros run` zephyr/qemu = "not yet wired" (keep caveat).

**Config files — both live:** root `nros.toml` = deploy-target SSOT (`[deploy.<name>]`, read by `nros deploy`/`build <name>`) — **keep** existing book refs. `<bringup>/system.toml` = multi-node topology (`[system]`, `[[component]]`, `[deploy.<target>]`, `[[domain]]`, `[[bridge]]`). Complementary, not either/or.

**Launch XML = ROS 2 schema verbatim** (§11.5). Tags v1: `<launch> <arg> <node> <param> <remap> <group> <include>`. Subs: `$(find <pkg>)`, `$(var <arg>)`, `$(env <name>)`. nav2/Autoware/turtlebot3 XML copy-pastes. Python `.launch.py` not v1.

**Canonical example paths (collapsed, Phase 118):** `examples/<plat>/<lang>/<example>/` — RMW chosen at build time (Cargo features / `-DNANO_ROS_RMW=` / Kconfig overlay). NOT `examples/<plat>/<lang>/<rmw>/<example>/`.

**Real on-disk references to cite:**
- Node pkg: `examples/stm32f4/rust/talker_pkg/` — `src/lib.rs:84` `nros::node!(Talker);`; `Cargo.toml:27` `[package.metadata.nros.node]`.
- Entry pkg: `examples/stm32f4/rust/talker-embassy/` — `src/main.rs:28` `nros::main!();`; `Cargo.toml:17` `[package.metadata.nros.entry] deploy = "embassy-stm32f4"`.
- Bringup `system.toml` shape: `packages/testing/nros-tests/fixtures/orchestration_e2e/demo_pkg_bringup/system.toml`.
- App-node starters (keep citing): `examples/native/rust/talker/`, `examples/native/c/talker/`, `examples/native/cpp/talker/`.

**DO NOT "fix" (these are correct):**
- `cargo run --features zenoh` / `--features xrce` (build-time RMW select).
- `examples/zephyr/cpp/cyclonedds/talker-aemv8r/` (real one-board-one-RMW carve-out).
- Root `nros.toml` references (the file ships and is read by the CLI).

---

## Task 1: Canonical example-shape fixes

**Files:**
- Modify: `book/src/internals/creating-examples.md` (lines 14, 18, 104, 153)
- Modify: `book/src/reference/c-api.md` (line ~76)

- [ ] **Step 1: Baseline grep (expect stale hits)**

Run: `grep -rn '<lang>/<rmw>/<example>\|/<rmw>/' book/src/`
Expected: hits in `internals/creating-examples.md` (lines 14, 104) and `reference/c-api.md` (`examples/native/c/<rmw>/action-client/`).

- [ ] **Step 2: Fix `creating-examples.md` table row (line 14)**

Replace the row:
```
| `examples/<plat>/<lang>/<rmw>/<example>/` | The standard cell. |
```
with:
```
| `examples/<plat>/<lang>/<example>/` | The standard cell. RMW is selected at **build time** (Cargo features / `-DNANO_ROS_RMW=` / Kconfig overlay), not encoded in the path. A single-package "app" example here is the canonical **starter** shape; the multi-package workspace shape (Node + Bringup + Entry pkgs) kicks in at ≥2 nodes — see [the workspace track](../../../book/src/getting-started/workspace-from-app-node.md). |
```

- [ ] **Step 3: Fix `creating-examples.md` matrix sentence (line 18)**

Change `The `<plat>` × `<lang>` × `<rmw>` coverage matrix is authoritative` →
`The `<plat>` × `<lang>` coverage matrix (RMW chosen at build time) is authoritative`.

- [ ] **Step 4: Fix `creating-examples.md` per-example contents block (line 104)**

In the fenced block, change `examples/<plat>/<lang>/<rmw>/<example>/` → `examples/<plat>/<lang>/<example>/`.

- [ ] **Step 5: Fix `creating-examples.md` checklist (line 153)**

Change `Confirm `<plat>/<lang>/<rmw>/<name>`` → `Confirm `<plat>/<lang>/<name>``.

- [ ] **Step 6: Fix `c-api.md` action-client path (~line 76)**

Read the line; change `examples/native/c/<rmw>/action-client/` → `examples/native/c/action-client/` (verify on disk with `ls examples/native/c/ | grep action` and use the actual dir name if it differs — e.g. `action-client` may be under a peer name; match reality).

- [ ] **Step 7: Verify no stale shape paths remain**

Run: `grep -rn '/<rmw>/\|<lang>/<rmw>' book/src/`
Expected: **no output** (zero hits).

- [ ] **Step 8: Commit**

```bash
git add book/src/internals/creating-examples.md book/src/reference/c-api.md
git commit -m "docs(book): collapse example paths to canonical <plat>/<lang>/<example> shape

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Reconcile the cookbook to the 3-role model

**Files:**
- Modify: `book/src/user-guide/component-and-entry-pkg.md` (whole file)
- Modify: `book/src/SUMMARY.md` (line 33 title)

This file is the reference cookbook. Apply the fact-sheet model. Keep it reference-voice (the new tutorial pages link here).

- [ ] **Step 1: Retitle the page + heading**

Line 1: `# Component + Entry Pkg Cookbook` → `# Node, Bringup & Entry Packages`.

- [ ] **Step 2: Rewrite the intro (lines 3–10) to three roles**

Replace the two-role intro with the three-role description. Use exactly the role definitions from the fact sheet (Node / Bringup / Entry), state "Old name *Component pkg* is retired in favour of *Node pkg* (matches ROS 2 composable-node naming)", and keep the single-Component convenience pointer but rename it "single-Node convenience".

- [ ] **Step 3: Rename section + macro + metadata (lines 12–40)**

`## Component pkg` → `## Node pkg`. Every "Component pkg" → "Node pkg". `nros::component!(Talker)` → `nros::node!(Talker)` (2 occurrences). `[package.metadata.nros.component]` → `[package.metadata.nros.node]`. Update the `pkgs/talker/` tree comment accordingly.

- [ ] **Step 4: Insert a Bringup pkg section (after the Node pkg section, before Entry pkg)**

Add:
````markdown
## Bringup pkg (optional)

A Bringup pkg is **pure declarative** — it owns the launch topology and
per-target deploy config, and contains no compiled code:

```
demo_bringup/
├── package.xml          # <name>demo_bringup</name>, <exec_depend> per node
├── system.toml          # [system] + [[component]] + [deploy.<target>] (+ [[domain]]/[[bridge]])
├── launch/
│   └── system.launch.xml   # ROS 2 launch schema, verbatim
└── config/                 # optional — params.yaml, etc.
```

No `Cargo.toml`, no `CMakeLists.txt`, no `src/`. Naming convention
`<system>_bringup` (alias `<system>_launch`), matching nav2 / Autoware /
turtlebot3. It is **optional**: required only when two or more Entry pkgs
share one topology. A single-Entry workspace folds `launch/` + `system.toml`
into the Entry pkg directly.

`launch/*.launch.xml` is the ROS 2 launch schema verbatim — `<launch>`,
`<arg>`, `<node>`, `<param>`, `<remap>`, `<group>`, `<include>`, with
`$(find <pkg>)` / `$(var)` / `$(env)` substitutions. Stock nav2/Autoware
XML pastes in and Just Works (Python `.launch.py` is not supported yet).
See [the workspace bringup tutorial](../../../book/src/getting-started/workspace-bringup.md).
````

- [ ] **Step 5: Fix the `nros::main!` forms block (lines 72–95)**

Replace the forms list + prose to match the fact-sheet four-form list exactly. Remove any claim that forms 3/4 need a "separate bringup pkg" framed as legacy; state plainly they reference a Bringup pkg by `<bringup>[:<file>]`.

- [ ] **Step 6: Delete the "retired Bringup pkg" claim**

Line ~160 table row: `Composition root (launch file + deploy config) | Entry pkg (replaces the retired Bringup pkg)` → `Launch topology + per-target deploy config | Bringup pkg (declarative; optional, folds into Entry pkg when single-target)`.

- [ ] **Step 7: Update the workspace-shape block (lines 104–128) to the §11.2 `src/`-rooted layout**

Replace the `my_robot/` tree with:
```
my_ws/
├── Cargo.toml          # [workspace] members = ["src/talker_pkg", "src/listener_pkg", "src/robot_entry"]
│                       # [workspace.metadata.nros] default_system = "demo_bringup"
└── src/
    ├── talker_pkg/         # Node pkg (lib, nros::node!)
    ├── listener_pkg/       # Node pkg
    ├── demo_bringup/       # Bringup pkg (declarative; no Cargo.toml)
    └── robot_entry/        # Entry pkg (bin, nros::main!(launch = "demo_bringup"))
```
Keep the "C++-majority → CMake top-level" pointer.

- [ ] **Step 8: Update remaining "Component" terms + quick-ref table (lines 130–161)**

Rename "single-Component convenience" → "single-Node convenience"; "One Component" → "One Node"; in the quick-ref table `Component pkg (nros::component!())` → `Node pkg (nros::node!())`.

- [ ] **Step 9: Update SUMMARY title**

`book/src/SUMMARY.md` line 33: `[Component + Entry Pkg Cookbook]` → `[Node, Bringup & Entry Packages]`.

- [ ] **Step 10: Verify no stale terms remain in the file**

Run: `grep -n 'Component pkg\|nros::component\|metadata.nros.component\|retired Bringup' book/src/user-guide/component-and-entry-pkg.md`
Expected: **no output**.

- [ ] **Step 11: Commit**

```bash
git add book/src/user-guide/component-and-entry-pkg.md book/src/SUMMARY.md
git commit -m "docs(book): reconcile cookbook to 3-role model (Node/Bringup/Entry)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Terminology drift sweep across other pages

**Files:**
- Modify (as hits dictate): `book/src/user-guide/workflow.md`, `book/src/user-guide/deployment.md`, `book/src/porting/board-trait.md`, `book/src/concepts/comparison-vs-microros.md`, any other hit.

- [ ] **Step 1: Find all remaining stale terms book-wide**

Run: `grep -rn 'Component pkg\|nros::component\|metadata.nros.component\|retired Bringup' book/src/`
Expected: a list (cookbook already clean from Task 2). Record each file:line.

- [ ] **Step 2: Fix each hit**

For every hit: "Component pkg" → "Node pkg"; `nros::component!` → `nros::node!`; `[package.metadata.nros.component]` → `[package.metadata.nros.node]`; remove any "Bringup retired" phrasing (state Bringup is an optional declarative role). Make only the minimal edit — do not rewrite surrounding prose.

- [ ] **Step 3: Verify clean**

Run: `grep -rn 'Component pkg\|nros::component\|metadata.nros.component\|retired Bringup' book/src/`
Expected: **no output**.

- [ ] **Step 4: Commit**

```bash
git add book/src/
git commit -m "docs(book): sweep Component->Node terminology across pages

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Make `reference/cli.md` accurate-to-shipped

**Files:**
- Modify: `book/src/reference/cli.md`

- [ ] **Step 1: Capture real help for the verbs the page may be missing**

Run each and read output (binary is at `packages/cli/target/release/nros`,
built by `just setup-cli`; `source ./activate.sh` puts it on PATH so the
bare `nros` invocation below resolves):
```bash
nros --help
nros launch --help
nros ws --help
nros explain --help
nros codegen-system --help
nros run --help
nros monitor --help
```
Expected: usage text per verb. (If a verb errors/absent, omit it and note so.)

- [ ] **Step 2: Add a `### nros launch …` section**

Insert after the `nros deploy` section. Document: `nros launch <bringup_pkg> [--launch <file>]` — spawns a Bringup pkg's components on the host from `launch/` (no ament install); the `native`/`native_sim` alternative to `ros2 launch`. Use the captured `--help` text as the authority.

- [ ] **Step 3: Add `### nros ws …`, `### nros explain …`, `### nros codegen-system …` sections**

One short section each, transcribed from the captured `--help`. Keep to what the help states; no invention.

- [ ] **Step 4: Reframe `nros build` / `nros deploy` as delegators**

In the `nros build` section, change the framing to: "`nros build` **delegates** to the per-platform build framework — it auto-detects the project flavor and hands off to `cargo` / `cmake` / `west` / `idf.py`; it does not build anything itself." Keep the detection-precedence list. In `nros deploy`, keep the existing per-target runner description (it already delegates).

- [ ] **Step 5: Add a `system.toml` cross-reference note**

Where root `nros.toml` is introduced, add one sentence: "Multi-node *topology* (which nodes, their wiring, per-target overrides) lives in a Bringup pkg's `system.toml` — see [Bringup](../../../book/src/getting-started/workspace-bringup.md). The root `nros.toml` carries deploy *targets* (`[deploy.<name>]`)." Do **not** delete `nros.toml` references.

- [ ] **Step 6: Verify every documented verb exists in the shipped binary**

Run: `nros --help | sed -n '/Commands:/,/Options:/p'` and confirm each `### nros <verb>` heading in `cli.md` is in that list.
Expected: every heading matched; no documented verb missing from the binary.

- [ ] **Step 7: Commit**

```bash
git add book/src/reference/cli.md
git commit -m "docs(book): cli.md accurate to shipped nros 0.3.7 (add launch/ws/explain; build/deploy delegate)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: New page — "From an app node to a workspace"

**Files:**
- Create: `book/src/getting-started/workspace-from-app-node.md`

- [ ] **Step 1: Write the page**

Tutorial voice. Required content:
- Open by recalling the app-node starter (`examples/native/rust/talker/`) the reader already built; one app, one `main`, one package.
- "When you outgrow one app": ≥2 nodes, a shared launch/topology, the same nodes on multiple boards → split into the 3 roles. State the roles in one paragraph each (Node/Bringup/Entry per fact sheet).
- A **ROS ↔ nano-ros command map** table (verbatim):

```markdown
| ROS 2 | nano-ros | Notes |
|---|---|---|
| `ros2 pkg create` | `nros new <name> --platform <plat> [--lang <lang>]` | scaffolds a Node pkg |
| `colcon build` | `cargo build` (Rust) / `cmake --build build` (C++) | `nros build` delegates to these |
| `ros2 launch <pkg> <file>` | `nros launch <bringup> [--launch <file>]` | host-side; no ament install |
| (plan/validate) | `nros plan` → `nros check` | resolve + statically check the topology |
| `ros2 run <pkg> <exe>` | run the Entry pkg binary (`cargo run`) | one Entry pkg per board |
```

- A "Where to go next" list linking the next three pages + the [Node, Bringup & Entry Packages](../../../book/src/user-guide/component-and-entry-pkg.md) reference.
- Note the app-node shape stays perfectly valid for single-node work — don't make a workspace until you need one.

- [ ] **Step 2: Verify cited paths exist**

Run: `ls examples/native/rust/talker/ book/src/user-guide/component-and-entry-pkg.md`
Expected: both exist.

- [ ] **Step 3: Commit**

```bash
git add book/src/getting-started/workspace-from-app-node.md
git commit -m "docs(book): add 'From an app node to a workspace' tutorial page

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: New page — "Prepare node packages"

**Files:**
- Create: `book/src/getting-started/workspace-node-pkgs.md`

- [ ] **Step 1: Write the page**

Required content:
- Scaffold: `nros new talker --platform native --lang rust` (note: creates a project skeleton; for a workspace, place it under `src/talker_pkg/`).
- Node-pkg anatomy with a real fenced tree mirroring `examples/stm32f4/rust/talker_pkg/`:
```
src/talker_pkg/
├── package.xml          # ROS 2 manifest (<exec_depend> ...)
├── Cargo.toml           # [lib] + [package.metadata.nros.node]
└── src/lib.rs           # impl Node + ExecutableNode; nros::node!(Talker);
```
- The `Cargo.toml` metadata block (verbatim, from the real example):
```toml
[package.metadata.nros.node]
class = "talker_pkg::Talker"
name = "talker"
default_namespace = "/"
```
- A minimal `src/lib.rs` skeleton showing `nros::node!(Talker);` and the `Node`/`ExecutableNode` impl shape (cite `examples/stm32f4/rust/talker_pkg/src/lib.rs` for the full worked version; show ~15 lines, not the whole thing).
- State: **no `fn main()`** in a Node pkg (it's a lib linked into an Entry pkg); codegen owns the spin loop.
- Build: `cargo build` from the workspace root builds all Node pkgs.
- Link to the [Node, Bringup & Entry Packages](../../../book/src/user-guide/component-and-entry-pkg.md) reference for the full API.

- [ ] **Step 2: Verify the cited example + macro line**

Run: `grep -n 'nros::node!' examples/stm32f4/rust/talker_pkg/src/lib.rs && grep -n 'metadata.nros.node' examples/stm32f4/rust/talker_pkg/Cargo.toml`
Expected: both match.

- [ ] **Step 3: Commit**

```bash
git add book/src/getting-started/workspace-node-pkgs.md
git commit -m "docs(book): add 'Prepare node packages' tutorial page

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 7: New page — "Bringup: launch + system.toml"

**Files:**
- Create: `book/src/getting-started/workspace-bringup.md`

**Note:** This page cites `examples/templates/multi-node-workspace/` (Task 10). If Task 10 runs after, leave the link in place — Task 11 link-checks the whole book.

- [ ] **Step 1: Write the page**

Required content:
- What a Bringup pkg is (declarative, optional, `<system>_bringup`) — fact-sheet definition.
- The tree (same as cookbook Bringup block).
- `system.toml` worked example (adapt the real fixture `packages/testing/nros-tests/fixtures/orchestration_e2e/demo_pkg_bringup/system.toml`):
```toml
[system]
name = "demo"
rmw = "zenoh"
domain_id = 0

[[component]]
pkg = "talker_pkg"
class = "talker_pkg::Talker"
name = "talker"

[[component]]
pkg = "listener_pkg"
class = "listener_pkg::Listener"
name = "listener"

[deploy.native]
kind = "self"
target = "x86_64-unknown-linux-gnu"
```
- `launch/system.launch.xml` worked example using only v1 tags:
```xml
<launch>
  <node pkg="talker_pkg" exec="talker" name="talker"/>
  <node pkg="listener_pkg" exec="listener" name="listener"/>
</launch>
```
- State the v1 tag set + substitutions (fact sheet) and that nav2/Autoware XML pastes in; `.launch.py` not yet.
- Workflow: `nros plan demo_bringup` → `nros check` → `nros launch demo_bringup`. **Flag**: if your `nros` build's host `launch` path is not fully wired, `nros plan` + `nros check` validate the topology and you run the Entry pkg binary directly (next page).
- Cite `examples/templates/multi-node-workspace/src/demo_bringup/` as the runnable copy-out.

- [ ] **Step 2: Verify the fixture reference exists**

Run: `ls packages/testing/nros-tests/fixtures/orchestration_e2e/demo_pkg_bringup/system.toml`
Expected: exists.

- [ ] **Step 3: Commit**

```bash
git add book/src/getting-started/workspace-bringup.md
git commit -m "docs(book): add 'Bringup: launch + system.toml' tutorial page

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 8: New page — "Entry package: boot on a board"

**Files:**
- Create: `book/src/getting-started/workspace-entry-pkg.md`

- [ ] **Step 1: Write the page**

Required content:
- What an Entry pkg is (per-board boot binary) — fact sheet.
- Tree:
```
src/robot_entry/
├── package.xml
├── Cargo.toml           # [[bin]] + deps on node pkgs + board crate + [package.metadata.nros.entry]
└── src/main.rs          # nros::main!(launch = "demo_bringup");
```
- `Cargo.toml` metadata (verbatim shape from `examples/stm32f4/rust/talker-embassy/Cargo.toml`):
```toml
[package.metadata.nros.entry]
deploy = "native"

[package.metadata.nros.deploy.native]
board     = "posix"
rmw       = "zenoh"
domain_id = 0
```
- The `nros::main!()` four forms (fact sheet, verbatim).
- The escape hatch (fact sheet §11.8): `<LinuxBoard as BoardEntry>::run(...)` and fully-manual `Executor::open`.
- Native run path: `cargo run -p robot_entry` (or `nros launch demo_bringup`). One Entry pkg per board: native + an embedded example — cite `examples/stm32f4/rust/talker-embassy/` (`nros::main!();` + `deploy = "embassy-stm32f4"`).
- **Note**: C++ Entry pkg (`NROS_MAIN` + `nros_entry()` cmake fn) is **future** (Phase 219); Rust is the shipped path today.
- `nros run` on zephyr/qemu = "not yet wired"; use `just <plat> run` for those.

- [ ] **Step 2: Verify cited example + macro lines**

Run: `grep -n 'nros::main!' examples/stm32f4/rust/talker-embassy/src/main.rs && grep -n 'metadata.nros.entry' examples/stm32f4/rust/talker-embassy/Cargo.toml`
Expected: both match.

- [ ] **Step 3: Commit**

```bash
git add book/src/getting-started/workspace-entry-pkg.md
git commit -m "docs(book): add 'Entry package: boot on a board' tutorial page

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 9: Wire the new pages into SUMMARY.md

**Files:**
- Modify: `book/src/SUMMARY.md` (Getting Started section, after line 14)

- [ ] **Step 1: Insert the four entries**

After the `Your own message package` line and before `Troubleshooting — First 10 Minutes`, add:
```
- [From an app node to a workspace](../../../book/src/getting-started/workspace-from-app-node.md)
- [Prepare node packages](../../../book/src/getting-started/workspace-node-pkgs.md)
- [Bringup: launch + system.toml](../../../book/src/getting-started/workspace-bringup.md)
- [Entry package: boot on a board](../../../book/src/getting-started/workspace-entry-pkg.md)
```

- [ ] **Step 2: Verify each linked file exists**

Run: `for f in workspace-from-app-node workspace-node-pkgs workspace-bringup workspace-entry-pkg; do ls book/src/getting-started/$f.md; done`
Expected: all four listed.

- [ ] **Step 3: Commit**

```bash
git add book/src/SUMMARY.md
git commit -m "docs(book): add workspace track to SUMMARY

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 10: Create the runnable 3-role `multi-node-workspace` template

**Files (create):**
- `examples/templates/multi-node-workspace/Cargo.toml`
- `examples/templates/multi-node-workspace/.gitignore`
- `examples/templates/multi-node-workspace/README.md`
- `examples/templates/multi-node-workspace/src/talker_pkg/{package.xml,Cargo.toml,.gitignore,src/lib.rs}`
- `examples/templates/multi-node-workspace/src/listener_pkg/{package.xml,Cargo.toml,.gitignore,src/lib.rs}`
- `examples/templates/multi-node-workspace/src/demo_bringup/{package.xml,system.toml,launch/system.launch.xml}`
- `examples/templates/multi-node-workspace/src/robot_entry/{package.xml,Cargo.toml,.gitignore,src/main.rs}`
- Modify: `examples/templates/multi-package-workspace/README.md` (add pointer)

**Reference shapes to copy from:** `examples/stm32f4/rust/talker_pkg/` (Node pkg `Cargo.toml`/`lib.rs`), `examples/native/rust/talker/` (native deps + `.cargo` if needed), the fixture `demo_pkg_bringup/system.toml` (bringup).

- [ ] **Step 1: Read the reference example files**

Run:
```bash
cat examples/stm32f4/rust/talker_pkg/Cargo.toml examples/stm32f4/rust/talker_pkg/src/lib.rs examples/stm32f4/rust/talker_pkg/package.xml
cat examples/native/rust/talker/Cargo.toml
cat packages/testing/nros-tests/fixtures/orchestration_e2e/demo_pkg_bringup/system.toml
cat examples/stm32f4/rust/talker-embassy/Cargo.toml examples/stm32f4/rust/talker-embassy/src/main.rs
```
Expected: real content to model the template on. Use the **native** dep/feature set from `examples/native/rust/talker/` (std + rmw-zenoh + platform-posix), not the embedded stm32f4 features.

- [ ] **Step 2: Write the workspace root `Cargo.toml`**

```toml
[workspace]
resolver = "2"
members = ["src/talker_pkg", "src/listener_pkg", "src/robot_entry"]
# demo_bringup is declarative (no Cargo.toml) and is NOT a member.

[workspace.metadata.nros]
default_system = "demo_bringup"
```

- [ ] **Step 3: Write `.gitignore` (root + per Rust pkg)**

Root `examples/templates/multi-node-workspace/.gitignore`:
```
/target/
/build/
```
Each Rust pkg dir (`talker_pkg`, `listener_pkg`, `robot_entry`) gets a `.gitignore`:
```
/target/
/generated/
```

- [ ] **Step 4: Write `talker_pkg` (Node pkg)**

`src/talker_pkg/Cargo.toml` — model on `examples/native/rust/talker/` deps but as a lib:
```toml
[package]
name = "talker_pkg"
version = "0.1.0"
edition = "2024"
publish = false

[lib]
crate-type = ["rlib", "staticlib"]

[dependencies]
nros = { path = "../../../../../packages/api/nros", default-features = false, features = ["std", "rmw-cffi", "platform-posix"] }
nros-rmw-zenoh = { path = "../../../../../packages/rmw/zenoh/nros-rmw-zenoh", features = ["std", "platform-posix", "ros-humble"] }

[package.metadata.nros.node]
class = "talker_pkg::Talker"
name = "talker"
default_namespace = "/"
```
(Verify the relative `path = "../../../../../packages/..."` depth against the actual nesting `examples/templates/multi-node-workspace/src/talker_pkg/` → repo root is 5 `..`; adjust if `ls` of the resolved path fails.)

`src/talker_pkg/src/lib.rs` — model the `Node`/`ExecutableNode` impl on `examples/stm32f4/rust/talker_pkg/src/lib.rs`, publishing `std_msgs/Int32` on `/chatter`, ending with `nros::node!(Talker);`.

`src/talker_pkg/package.xml` — copy `examples/stm32f4/rust/talker_pkg/package.xml`, rename `<name>` to `talker_pkg`.

- [ ] **Step 5: Write `listener_pkg` (Node pkg)**

Same shape as Step 4; subscribes to `/chatter`, `class = "listener_pkg::Listener"`, `name = "listener"`, `nros::node!(Listener);`.

- [ ] **Step 6: Write `demo_bringup` (Bringup pkg, no Cargo.toml)**

`src/demo_bringup/system.toml`:
```toml
[system]
name = "demo"
rmw = "zenoh"
domain_id = 0

[[component]]
pkg = "talker_pkg"
class = "talker_pkg::Talker"
name = "talker"

[[component]]
pkg = "listener_pkg"
class = "listener_pkg::Listener"
name = "listener"

[deploy.native]
kind = "self"
target = "x86_64-unknown-linux-gnu"
```
`src/demo_bringup/launch/system.launch.xml`:
```xml
<launch>
  <node pkg="talker_pkg" exec="talker" name="talker"/>
  <node pkg="listener_pkg" exec="listener" name="listener"/>
</launch>
```
`src/demo_bringup/package.xml` — copy the fixture's `demo_pkg_bringup/package.xml`, rename `<name>` to `demo_bringup`.

- [ ] **Step 7: Write `robot_entry` (Entry pkg)**

`src/robot_entry/Cargo.toml`:
```toml
[package]
name = "robot_entry"
version = "0.1.0"
edition = "2024"
publish = false

[[bin]]
name = "robot_entry"
path = "src/main.rs"

[dependencies]
nros = { path = "../../../../../packages/api/nros", default-features = false, features = ["std", "rmw-cffi", "platform-posix"] }
nros-rmw-zenoh = { path = "../../../../../packages/rmw/zenoh/nros-rmw-zenoh", features = ["std", "platform-posix", "ros-humble"] }
nros-board-linux = { path = "../../../../../packages/boards/nros-board-linux" }
talker_pkg = { path = "../talker_pkg" }
listener_pkg = { path = "../listener_pkg" }

[package.metadata.nros.entry]
deploy = "native"

[package.metadata.nros.deploy.native]
board     = "posix"
rmw       = "zenoh"
domain_id = 0
```
(Verify `nros-board-linux` is the correct native board crate name: `ls packages/boards/ | grep posix`; use the actual name.)

`src/robot_entry/src/main.rs`:
```rust
nros::main!(launch = "demo_bringup:system.launch.xml");
```
`src/robot_entry/package.xml` — `<name>robot_entry</name>` with `<exec_depend>talker_pkg</exec_depend>` + `<exec_depend>listener_pkg</exec_depend>`.

- [ ] **Step 8: Write `README.md`**

Copy-out instructions: `cargo build`, `nros plan demo_bringup`, `nros check`, then `nros launch demo_bringup` (or `cargo run -p robot_entry`). Include the ROS↔nano-ros map. State this is the canonical 3-role template.

- [ ] **Step 9: Verify the workspace builds**

Run: `cd examples/templates/multi-node-workspace && cargo build 2>&1 | tail -20`
Expected: builds the two Node pkgs + Entry pkg. If `nros::main!` codegen needs generated msg bindings, run `nros generate-rust` per pkg first (mirror what `examples/native/rust/talker/` requires) and document that in the README. If a dep path / board-crate name is wrong, fix and rebuild. **If the build cannot be made green within reason (e.g. macro needs a workspace pkg-index feature not in 0.3.7), STOP and report — do not commit a broken template.**

- [ ] **Step 10: Verify plan + check**

Run: `cd examples/templates/multi-node-workspace && nros plan demo_bringup && nros check`
Expected: a `plan.json` emitted + check passes. If `nros launch` is not e2e-wired, note it in the README (don't claim a green launch).

- [ ] **Step 11: Confirm no build artifacts staged**

Run: `cd examples/templates/multi-node-workspace && git status --porcelain | grep -E 'target/|generated/' || echo CLEAN`
Expected: `CLEAN` (gitignores working).

- [ ] **Step 12: Add pointer to the old template's README**

In `examples/templates/multi-package-workspace/README.md`, add one line near the top:
`> For the canonical 3-role (Node + Bringup + Entry) workspace pattern, see `multi-node-workspace/`. This template demonstrates a polyglot (Rust/C/C++) app-node workspace.`

- [ ] **Step 13: Commit**

```bash
git add examples/templates/multi-node-workspace/ examples/templates/multi-package-workspace/README.md
git commit -m "examples: add canonical 3-role multi-node-workspace template

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 11: Final validation — build the book + global grep gates

**Files:** none (verification only)

- [ ] **Step 1: Build the book**

Run: `just book 2>&1 | tail -30`
Expected: mdbook builds with no broken-link / missing-file errors. Fix any reported broken intra-book link (most likely a relative path typo in a new page).

- [ ] **Step 2: Global stale-shape gate**

Run: `grep -rn '/<rmw>/\|<lang>/<rmw>' book/src/`
Expected: **no output**.

- [ ] **Step 3: Global terminology gate**

Run: `grep -rn 'Component pkg\|nros::component\|metadata.nros.component\|retired Bringup' book/src/`
Expected: **no output**.

- [ ] **Step 4: New pages are linked**

Run: `grep -c 'workspace-from-app-node\|workspace-node-pkgs\|workspace-bringup\|workspace-entry-pkg' book/src/SUMMARY.md`
Expected: `4`.

- [ ] **Step 5: Spot-check the rendered pages (optional, if a browser is available)**

Run: `just book` already wrote `book/book/` — open the four new pages and the cookbook; confirm tables/code render and links resolve.

- [ ] **Step 6: Commit any link fixes**

```bash
git add book/
git commit -m "docs(book): fix links + finalize workspace track

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```
(Skip if Steps 1–4 needed no changes.)

---

## Self-Review notes (author)

- **Spec coverage:** A (Task 1) · B1 reconcile (Tasks 2–4) · B2 four pages (Tasks 5–8) · B3 SUMMARY (Task 9) · C template (Task 10) · validation (Task 11). All spec sections mapped.
- **Not-yet-shipped flags:** C++ entry macros (Tasks 2,8), `nros run` zephyr/qemu (Task 8), host `nros launch` e2e (Tasks 7,10) — all called out.
- **Guardrails honored:** app-node pages untouched; `--features zenoh` + `talker-aemv8r` carve-out + root `nros.toml` preserved (Tasks 1,4 explicit); broken template must NOT be committed (Task 10 Step 9).
- **Naming consistency:** `nros::node!`, `[package.metadata.nros.node]`, `[package.metadata.nros.entry]`, `demo_bringup`, `talker_pkg`/`listener_pkg`/`robot_entry` used identically across Tasks 2,6,7,8,10.
