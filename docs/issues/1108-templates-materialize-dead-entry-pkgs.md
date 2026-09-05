---
id: 1108
title: "Four copy-out templates still materialize a `robot_entry` package — two of them declare no `[image.*]` at all, so `nros build` refuses them outright"
status: open
type: tech-debt
area: examples, tooling, docs
severity: high
found: 2026-09-06
related: [1107, 1109, rfc-0065, phase-383]
---

# A template is the costliest place for a retired shape

phase-383 W9/W10.a landed RFC-0065 D3/D4: **an image is a ROW, not a
directory.** `nros build` discovers the packages, reads the `[image.*]` table in
a bringup package's `system.toml`, and generates BOTH the root build file and
the entry package from it. `builder::entry`'s own module doc states the claim:

> **This is the headline claim of the phase**: "the entry stops being
> hand-written". Everything else generates a *build file*; this generates the
> program.

W10.a retired the shape across `examples/workspaces/**` — all 15 of them, zero
tracked roots — and `check-no-tracked-workspace-roots` was added (W10.c) so it
cannot come back. **W10's scope was `examples/workspaces/` and nothing else.**
`examples/templates/` was never named in the phase doc, and the templates are
what a user copies.

## The measured state

Verified 2026-09-06 on `docs/issues-1107-1109`:

| template | `[image.*]` rows | entry dirs | root build file | `.colcon_workspace` |
| --- | --- | --- | --- | --- |
| `examples/templates/multi-node-workspace` | 1 (`[image.native]`) | 1 (`src/robot_entry`, cargo) | TRACKED `Cargo.toml` | no |
| `examples/templates/multi-node-workspace-cpp` | 1 (`[image.native]`) | 1 (`src/robot_entry`, cmake) | TRACKED `CMakeLists.txt` | no |
| `examples/templates/c-and-cpp-mixed-workspace` | **0** | 1 (`src/robot_entry`, cmake) | TRACKED `CMakeLists.txt` | no |
| `examples/templates/pure-c-workspace` | **0** | 1 (`src/robot_entry`, cmake) | TRACKED `CMakeLists.txt` | no |

Counted with `grep -c '^\[image\.' <t>/src/demo_bringup/system.toml` and
`find examples/templates/<t> -type d -name '*_entry'`.

`pure-c-workspace/src/demo_bringup/system.toml` is the extreme case — it is six
lines and declares **no `[[component]]` blocks either**, only `[system]`. Every
other bringup in the tree names its components.

## What that costs, measured — not hypothetical

**The two with no `[image.*]` cannot be built by the canonical verb at all:**

```
$ cd examples/templates/pure-c-workspace && nros build --dry-run
Error: this workspace declares no `[image.*]`. An image is the buildable unit — see RFC-0065 D6.
Location:
    nros-cli-core/src/cmd/build.rs:219:69
```

Identical for `c-and-cpp-mixed-workspace`. The two that DO declare an image get
past image resolution and ask only for a sync:

```
$ cd examples/templates/multi-node-workspace-cpp && nros build --dry-run
Error: missing prerequisites for this build:
  - generated message bindings (this workspace has never been synced)
      run: nros sync
```

**And after that sync, the Rust one still cannot be built** — the name
mismatch is fatal, not cosmetic:

```
$ cd examples/templates/multi-node-workspace && nros sync && nros build --dry-run
sync: done.
nros build: warning: no selection facade at …/generated/nros-selection/native_entry
  — building `native` without its RMW and ROS edition. Run `nros sync` first if
  this Entry needs them (issue 0937). …
nros build:   entry → …/examples/templates/multi-node-workspace/build/posix/native_entry
nros build: demo_bringup:native -> board native (platform posix), driver cargo
NROS_DECLARED_INFRA_QUERYABLES=none cargo build -p native_entry

$ cargo build -p native_entry
error: package ID specification `native_entry` did not match any packages
```

Three separate faults in that one transcript:

- The build GENERATES `build/posix/native_entry` because no `src/native_entry`
  exists to suppress it — the hand-written `robot_entry` is invisible to the
  suppression check.
- The tracked root's `members = ["src/talker_pkg", "src/listener_pkg",
  "src/robot_entry"]` does not list it, and `cargo_root::ensure` **will not add
  it**: "If `<ws>/Cargo.toml` declares a `[workspace]`, that is the root and the
  build uses it as-is; generation happens only when there is none." So the
  handoff cargo command cannot resolve its own `-p`.
- The facade warning names `native_entry` and tells the user to run `nros sync`
  — which was just run, and which writes no
  `generated/nros-selection/` for this template at all. The advice cannot work
  because sync and build disagree about what the entry is called.

So `nros build`, the verb the book and the RFC point users at, is **broken on
all four templates today**: two refuse for want of an `[image.*]` row, the Rust
one dies at `-p`, and the C++ one emits two entries (below).

**The two "belt and braces" templates emit the entry TWICE.** After
`nros sync`, `nros build --dry-run` on `multi-node-workspace-cpp` writes
`build/posix-native/CMakeLists.txt`, and the generated root contains both the
hand-written entry as a subdirectory AND the derived one:

```cmake
    SUBDIRS
        "src/listener_pkg"   # listener_pkg
        "src/robot_entry"   # robot_entry
        "src/talker_pkg"   # talker_pkg
)
…
nano_ros_add_executable(native_entry
    BOARD   native
    BRINGUP "${CMAKE_CURRENT_SOURCE_DIR}/../../src/demo_bringup"
    LAUNCH  default
    LANG    cpp
    TYPED
    DEPLOY  native)
```

One topology, one board, two programs. That is what "belt and braces" actually
buys here: `builder::entry::generate_entry` suppresses generation only when a
hand-written package named `<image_id>_entry` exists —

```rust
let want = crate::builder::entry::package_name(image_id);   // "native_entry"
let hand_written = root.join("src").join(&want);
if hand_written.join("Cargo.toml").is_file() || hand_written.join("CMakeLists.txt").is_file() {
    return Ok(None);
}
```

— and the template's package is `robot_entry`, which matches no image id. So
the templates get the WORST of both: a hand-written entry that does not suppress
generation, plus a generated one, plus a tracked root that (on the cargo driver)
`cargo_root::ensure` then refuses to update.

**The blast radius is real, not theoretical.** Issue 1107 records an
out-of-tree consumer scaffolded with entry packages last week. These templates
are the source: `packages/cli/cargo-nano-ros/src/workspace_scaffold.rs`
`include_str!`s `multi-node-workspace{,-cpp}` **file by file, verbatim**, so
`nros new <name> --workspace` writes a `src/robot_entry/` into every workspace
it scaffolds.

## The true shape

`examples/workspaces/c` and `examples/workspaces/cpp` are the worked examples.
Each is exactly three tracked files at its root:

```
examples/workspaces/c/.colcon_workspace
examples/workspaces/c/.gitignore
examples/workspaces/c/README.md
```

`.gitignore` untracks the generated root:

```
/generated/
/build/
/build-*/

# phase-383 W10.a — the root manifest and root CMakeLists are GENERATED by
# `nros build` from the discovered packages plus the `[image.*]` table. They
# are build output like `build/` and `Cargo.lock`, and committing either would
# put a generated member list under review (RFC-0065 D3). The tracked marker
# that this directory IS a workspace root is `.colcon_workspace`.
/Cargo.toml
/CMakeLists.txt
```

and `.colcon_workspace` is the tracked marker that replaces it — its own comment
names the failure it prevents, and names these very templates while doing so:

> `detect_workspace_root` walks up for a `Cargo.toml` carrying `[workspace]` and
> otherwise falls back to `.git/`, so without this marker a package inside a
> migrated workspace resolves its root to the REPOSITORY and indexes every
> package in the tree:
>
>     nros::main!: build_pkg_index: duplicate pkg name `demo_bringup` in
>     workspace `/…/nano-ros`: `examples/templates/c-and-cpp-mixed-workspace/
>     src/demo_bringup` and `examples/templates/multi-node-workspace/src/demo_bringup`

The image rows carry what the entry package used to encode — board, launch file,
launch args, rmw, panic policy, conf fragments — e.g.
`examples/workspaces/c/src/demo_bringup/system.toml`:

```toml
[image_defaults]
rmw = "zenoh"

[image.native]
board = "native"

[image.native_service_client]
board = "native"
launch = "service_client.launch.xml"
```

**The migration is proven to work on a template.** A copy of
`pure-c-workspace` in a scratch dir with the root `CMakeLists.txt` and
`src/robot_entry/` deleted, `.colcon_workspace` added, and
`[[component]]` × 2 + `[host.native]` + `[image.native] board = "native"`
appended to its `system.toml`, builds clean:

```
$ nros build native
nros build: demo_bringup:native -> board native (platform posix), driver cmake
…
[ 86%] Linking C executable native_entry
[100%] Linking CXX executable c_talker_pkg
```

producing `build/posix-native/cmake/{native_entry,pkg/c_talker_pkg/c_talker_pkg,
pkg/c_listener_pkg/c_listener_pkg}` and
`build/posix-native/cmake/nros-metadata.json` — the same programs the
pre-migration tree produces (baseline `cmake -S examples/templates/pure-c-workspace -B <dir>`
+ `cmake --build`: `src/robot_entry/robot_entry`, `src/c_talker_pkg/c_talker_pkg`,
`src/c_listener_pkg/c_listener_pkg`, `nros-metadata.json`), with `robot_entry`
renamed to the image it is derived from.

## What covers these templates

Three of four have a lane. **`multi-node-workspace` (Rust) has none** — no
`examples/fixtures.toml` row, no `just` recipe, no workflow step, no test. Its
only exercise is compiling into the CLI binary as `include_str!` text, which
proves the files EXIST and nothing about whether the workspace builds. That is
the template `nros new --workspace --lang rust` hands a first-time user.

| id | row kind / builder | dir | asserted output |
| --- | --- | --- | --- |
| `cpp_robot_entry` | `compile_check_fixture` / `cmake-configure` | `multi-node-workspace-cpp` | `src/robot_entry/robot_entry` |
| `metadata_cpp` | `compile_check_fixture` / `cmake-configure` | `multi-node-workspace-cpp` | `nros-metadata.json` |
| `c_mixed_workspace` | `compile_check_fixture` / `cmake-configure` | `c-and-cpp-mixed-workspace` | `nros-metadata.json` (row) — but the TEST asserts `src/robot_entry/robot_entry` |
| `pure_c_workspace` | `compile_check_fixture` / `cmake-configure` | `pure-c-workspace` | `nros-metadata.json` (row) — same TEST |
| — | — | `multi-node-workspace` | **nothing** |

The row/test mismatch on the last two is worth its own line:
`examples/fixtures.toml` says the artifact is `nros-metadata.json`, while
`packages/testing/nros-tests/tests/c_mixed_workspace.rs` asks
`require_cmake_fixture(id, "src/robot_entry/robot_entry")`. Both hold today
because the build produces both.

## Why this has not been fixed already — a countervailing recorded decision

Two places in the tree deliberately exempt the templates, and any migration has
to overturn them rather than ignore them:

1. `examples/fixtures.toml`, the `metadata_cpp` row's comment:

   > The template keeps a hand-written root by design (it is what `nros new`
   > scaffolds and what a user copies), so this holds after every example
   > workspace has migrated.

   `metadata_cpp` was retargeted off `examples/workspaces/cpp` ONTO this
   template precisely because the template still had a root to configure.

2. `scripts/check-no-tracked-workspace-roots.py`'s self-test lists
   `examples/templates/multi-node-workspace/Cargo.toml` under `must_not_flag`,
   commented "Not an example workspace at all."

Both are load-bearing for the CMAKE templates specifically, because
`nano_ros_workspace()` in `cmake/NanoRosWorkspace.cmake` has **no `[image.*]`
support at all** — the only match for `image` in that file is line 258's prose
comment "for embedded images too". A hand-written cmake root therefore cannot
materialize an image row — so for those templates "keep the hand-written root"
and "delete the entry package" are mutually exclusive. Migrating means moving
the templates onto the `nros build` flow, which is the same decision W10.a made
for the example workspaces.

## The migration, and everything it touches

Per template: delete the root build file and `src/robot_entry/`; add a tracked
`.colcon_workspace`; add `/Cargo.toml` + `/CMakeLists.txt` to `.gitignore`; add
`[host.native]` + `[image.native] board = "native"` (and, for
`pure-c-workspace`, the two missing `[[component]]` rows) to
`src/demo_bringup/system.toml`; rewrite the README's build ritual to `nros sync`
+ `nros build native`.

The files OUTSIDE `examples/templates/**` that must move in the same commit:

| file | why |
| --- | --- |
| `packages/cli/cargo-nano-ros/src/workspace_scaffold.rs` | `include_str!`s every `src/robot_entry/*` of both `multi-node-workspace{,-cpp}`; deleting them is a hard rustc error, not a warning. Also carries the `("rust", "src/robot_entry/Cargo.toml")` RMW-rewrite branch, the `./build/src/robot_entry/robot_entry` "Next steps" hint, and two unit tests reading `ws/src/robot_entry/Cargo.toml`. |
| `packages/cli/cli-source-dirs.txt` | lines 8–9 name both template dirs in the CLI source-stamp closure (issue 0627); the set of `include_str!`ed dirs changes with the file list. |
| `examples/fixtures.toml` | the four rows above are `builder = "cmake-configure"`, which needs a root `CMakeLists.txt` to `cmake -S`. Migrated templates have none — the rows become `[[workspace_fixture]]` with an `image =`, the shape `examples/workspaces/c` already uses. |
| `packages/testing/nros-tests/tests/c_mixed_workspace.rs` | asserts `src/robot_entry/robot_entry` for both C templates. |
| `packages/testing/nros-tests/tests/cpp_multi_node_entry.rs` | three sites naming the `cpp_robot_entry` fixture and `src/robot_entry/robot_entry`. |
| `packages/testing/nros-tests/tests/workspace_metadata.rs` | `require_cmake_fixture("metadata_cpp", "nros-metadata.json")`. |
| `packages/tooling/nros-sizes-build/src/lib.rs:1946` | `("NROS_FIXTURE_ID", "c_mixed_workspace")` — a literal inside a unit test's env-var list, so cosmetic, but it goes stale with the id. |
| `scripts/check-no-tracked-workspace-roots.py` | its `ROOT_RE` covers `examples/workspaces/` only; templates need adding, and the self-test's `must_not_flag` entry removing, or the shape can come straight back. |
| `book/src/getting-started/{first-project,anatomy}.md`, `book/src/user-guide/{component-and-entry-pkg,deployment}.md` | teach `src/robot_entry/`, `./build/src/robot_entry/robot_entry`, `cargo run -p robot_entry`, and "One Entry pkg per board target". Tracked as issue 1107. |

## Related sweep — the rest of `*_entry`

`find examples -maxdepth 4 -type d -name "*_entry"` returns **17**:

- **4** — the template `robot_entry`s above.
- **12** — `zephyr_entry` / `fvp_entry` / `zephyr_rust_*_entry`, every one
  carrying `prj.conf` and most carrying `boards/`. These are the documented D5
  hold-out: W10.a says "West and FVP entries stay hand-written: they carry
  authored Kconfig no image declaration expresses." Legitimate.
- **1** — `examples/workspaces/rust/src/esp32_entry`. **Not obviously
  legitimate.** Its contents are `Cargo.toml package.xml src` — the same
  three-file shape as a template `robot_entry`, plus a generated
  `.cargo/nros-board.toml` — and **no Kconfig, no `prj.conf`, no `boards/`**,
  so the west/FVP exemption does not cover it. Its `[image.esp32]` row already
  exists (`board = "esp32-qemu"`, `panic = "own"`), and the board descriptor
  `packages/boards/nros-board-esp32-qemu/nros-board.toml` already carries the
  entry's whole no_std shell as data the generator consumes:

  ```toml
  crate_root_extra = "use esp_backtrace as _;\nnros_board_esp32_qemu::esp_bootloader_esp_idf::esp_app_desc!();"
  crate_root_deps = ["esp-backtrace = { version = \"~0.18.0\", features = [\"esp32c3\", \"panic-handler\", \"println\"] }"]
  ```

  It survives only because `generate_entry` suppresses generation when
  `src/<image_id>_entry` exists — and here the image id `esp32` and the package
  name `esp32_entry` DO match, so the hand-written one silently wins. Two things
  it holds that an `[image.*]` row does not obviously express, and which need
  answering before deleting it: `[package.metadata.nros.deploy.esp32-qemu]
  locator = "tcp/10.0.2.2:9830"`, and a `[profile.release]` block
  (`lto = "fat"`, `codegen-units = 1`) that shapes the flashed image's size.
  Filed here as a lead, deliberately NOT migrated.

## Acceptance

1. `find examples/templates -maxdepth 3 -type d -name '*_entry'` returns empty.
2. Each of the four bringups declares at least `[image.native]`, and
   `nros build --dry-run` in each template succeeds after `nros sync`.
3. `nros new <name> --workspace --lang {rust,cpp}` scaffolds a workspace with
   no `src/robot_entry/`, and the scaffolded workspace builds.
4. `multi-node-workspace` gains a lane — it is the one template with none.
5. `check-no-tracked-workspace-roots` covers `examples/templates/` as well, with
   the self-test updated in both directions.
6. `just check gate-lists` and `just check fixture-manifest` green; the four
   fixture rows build and their tests pass.
