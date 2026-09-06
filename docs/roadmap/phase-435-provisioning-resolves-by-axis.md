# Phase 435 — provisioning resolves by axis

**Status (2026-09-06). Every work item landed.** W1–W5 complete; the two axes
that already worked were left alone. Implements
[RFC-0062 amendment 4](../design/0062-unified-dependency-ssot.md), which answers
phase-413 W6 (rosdep parity) and closes the `ros-<distro>` question phase-422
left open.

**Prior phases:** 422 (provisioning has one path), 413 (CI workflow user
parity), 327 (the unified dependency SSoT), 420 (package identity, RFC-0087).

## What the amendment decided, in one table

| axis | declared in `package.xml` as | resolves into | state |
| --- | --- | --- | --- |
| build_type | `<build_type>` | `SelfBuildtool`, then `[build_type.*]` | **W2 adds the second half** |
| depend | `<depend>` | `[prereq.*]` via the ladder | exists |
| board | `<nano_ros_uses kind="board">` / `board=` sugar | `[board.*]` | exists, 50 values, all resolve |
| rmw | `<nano_ros_uses kind="rmw">` / `rmw=` sugar | `[rmw.*]` | exists, 51 values, all resolve |

Two axes are already complete and measured. This phase builds the missing half
of one, adds a role the vocabulary lacks, replaces an interim, and fills the one
hole the amendment recorded.

---

### W1 — `role = "buildtool"`

`PrereqRole` has `Package`, `Workspace`, `Infra`, `Vendor`, `Unclassified` and
nothing for "comes from how you build", so seven build tools are filed
`role = "package"`: `cargo`, `cmake`, `clang`, `libclang-dev`,
`python3-{dev,pip,venv}`. `nros setup --workspace` therefore reports `cargo`
under *"SYSTEM PREREQUISITES this workspace names"*, which is not what a
`<depend>` means. RFC-0062 amendment 3's own table already groups 16 keys as
build tooling; the vocabulary just never gained the value.

**Acceptance.** The seven are `role = "buildtool"`; `check-prereq-roles` accepts
the value and still refuses `unclassified`; `--workspace` reports buildtools
under their own heading, and the three real content deps (`libmbedtls`,
`libz3`, `ros-rmw-zenoh-cpp`) stay `role = "package"`.

**LANDED.** `check-prereq-roles`: 46 keys — 7 buildtool, 19 infra, 4 package,
5 vendor, 11 workspace. A `<depend>` naming a buildtool is reported on its own
line rather than as a category error: rosdep wants
`<buildtool_depend>cmake</…>` and ROS packages carry it, so it is not *wrong*
the way declaring an emulator is — merely not a content dependency.

### W2 — `[build_type.*]`, consulted only where `SelfBuildtool` cannot answer

`Resolution::SelfBuildtool` rests on *"if that builder is building it, the
buildtool is present"*. Phase-431 makes that false for the nano-ros build types:
once `nros` ships prebuilt, a user holds the builder and may have neither cargo
nor cmake — and `nros_cargo`/`nros_cmake` are 375 of 406 packages.

```toml
[build_type.nros_cargo]  packages = ["cargo"]
[build_type.nros_cmake]  packages = ["cmake", "clang"]
```

Not a reopening of the `[prereq.ament_cargo]` option amendment 3 rejected as
fiction: no apt package provides `ament_cargo`, while `cargo` and `cmake` are
real keys already in the index. The gap is only that **nothing reaches them from
a `<build_type>`** — zero of the eleven `role=package` keys is named by any
`[board.*]` or `[rmw.*]` row, so they are reachable only through
`--role package --role workspace`, a bulk list rather than a resolution.

**Acceptance.** A workspace of `nros_cargo` packages resolves `cargo` through
the build_type axis; `ament_cmake` still takes `SelfBuildtool` and gains
nothing; a build type with no row is not an error (it declares no host tool).

**LANDED**, observed on `examples/workspaces/mixed`:

```
BUILDERS (from <build_type>) — each implies a toolchain that must exist:
  nros_cmake       x9    buildtool: (no buildtool implied)   host tools: cmake, clang

HOST TOOLS the builders need (from <build_type>, not from a <depend>):
  clang                        present   (nros_cmake)
  cmake                        present   (nros_cmake)
```

`SdkIndex::validate` refuses a row naming a key that is not a `[prereq.*]`, so
the axis cannot become the fiction amendment 3 rejected.

*One bug, worth recording because TOML invites it:* the rows were first written
above `build_sources = [...]`, a top-level key — which a preceding `[table]`
header swallows. The index then failed to parse with `unknown field
build_sources, expected packages or why`, naming the victim rather than the
cause. Tables go after the file's scalar keys.

### W3 — `ros_package`, and retiring `{ros_distro}`

```toml
[prereq.rmw-zenoh-cpp]
ros_package = "rmw_zenoh_cpp"     # ros-<distro>-rmw-zenoh-cpp, derived
```

All nine ROS names this tree composes by hand follow `ros-<distro>-<pkg _→->`
with no exceptions. apt's pattern is verified (bloom's convention); **pacman's
is not** and must be checked against a real package list before it ships; dnf and
brew stay unmapped, which is already an answer the index prints.

This supersedes issue 1128's `{ros_distro}` placeholder, landed the same day as
the interim. Retiring it is the point rather than a tidy-up: it leaves the
placeholder with zero users, and `check-prereq-placeholders` fails by design when
a vocabulary entry the index does not use remains — so the gate names its own
retirement.

**Acceptance.** `nros setup --system` composes `ros-humble-…` by default and
`ros-jazzy-…` under `ROS_DISTRO=jazzy`, as the placeholder did; `{ros_distro}`,
both expanders and `check-prereq-placeholders` are deleted in one commit;
`just ci provision-zenohd` still gets its name from the index.

**LANDED.** Verified through both readers — `nros setup --system` and
`prereq-packages.py` — at humble, jazzy and iron. The placeholder, both
`expand()` implementations and the gate that guarded their agreement are gone:
one derivation replaced a rule that had to be kept in step across two languages.

The gate named its own retirement, which is the outcome it was written for: it
refuses a vocabulary entry the index does not use, so emptying the vocabulary
made it fail, and the fix was to delete it.

### W4 — `[board.zephyr]` names what a Zephyr build needs

The one hole the amendment recorded. `[tool.zephyr-sdk]` exists with four host
dists, `west` is a `[python.*]` class, and `[board.zephyr]` resolves to **nothing**
— so provisioning reaches them through the `just zephyr` module instead. That is
the second path phase-422 spent four work items removing elsewhere.

**Acceptance.** `[board.zephyr]` names its packages; `nros setup zephyr`
provisions through the index; the module recipe forwards or is retired.

**LANDED**, in the narrow form the evidence supports:

```
$ nros setup zephyr --dry-run
nros setup: zephyr (rmw zenoh) needs 3 package(s):
  zephyr-sdk             prebuilt 0.16.8 (dist linux-x86_64)
  zenoh-pico             source 1.7.2 …
  mbedtls                source 3.x …
```

`west` is deliberately NOT in the row: it is a `[python.*]` class, a pip package
into the interpreter the Zephyr lanes use, and `nros setup --tool` does not
install into a venv — `_setup-common` already reports the python closure. The
Zephyr SOURCES stay outside too: `scripts/zephyr/setup.sh` fetches them into a
west-shaped workspace, which is not a `[source.*]` submodule and would be a
fiction as one. The row now carries what the index can honestly provision, which
is the toolchain.

### W5 — refuse an ambiguous board alias

`threadx` is claimed by two descriptors — `nros-board-threadx-linux` and
`nros-board-threadx-qemu-riscv64`. No entry uses the bare name today, so this is
a gate before a bug rather than after one: an alias that resolves to two boards
cannot resolve to one provisioning row.

**Acceptance.** A gate refuses a `names = [...]` entry claimed by two descriptor
blocks; adding `threadx` to a third turns it red.

**LANDED, and the acceptance above was wrong** — running the gate answered its
own question. Sixteen clashes appeared; fifteen were the fixture workspace at
`packages/cli/nros-cli-core/tests/fixtures/board-workspace/`, a deliberate copy
of the board tree read only by its own test. Those are pruned.

The sixteenth was `threadx`, the case the item was written for — **and it is not
a bug.** `threadx` is a deploy FAMILY: 25 `package.xml` files carry
`<nano_ros deploy="threadx" board="riscv64-qemu"/>` or `board="threadx-linux"`,
and per RFC-0087 D3 `deploy=` names a `[deploy.*]` block while the `board=`
beside it selects the implementation. Both boards ARE threadx, and nothing
resolves on the family alone.

So the rule is narrower than the item assumed: **a name that is not a deploy
family must claim one board.** Families are exempt by DERIVATION — read from the
same `package.xml` exports that use them — so the exemption cannot go stale the
way a hand-kept list would. Mutation-verified: giving a second board the
`threadx-linux` alias turns it red.

## Out of scope, and why

* **The example `CMakeLists.txt` shapes** — 15 call `nano_ros_entry`, 83
  `nano_ros_add_executable`, 68 `nano_ros_auto_add_library`. Resolution reads
  none of them, so nothing here is blocked. Worth consolidating before anything
  starts treating a build file as a declaration.
* **Comparing an entry's `deploy =` against its package's export tag.** They use
  different vocabularies on purpose (RFC-0087 D3); a gate would first have to
  decide which is authoritative, which is phase-422 W7's open question.
* **`[board.native]` / `[board.posix]` resolving to nothing** — defensible: their
  needs are the build_type axis, which W2 supplies.
* **The `pacman` pattern for ROS packages stays unimplemented.** RFC-0062
  amendment 4 records the AUR convention as unverified, and `ros_os_package`
  returns `None` for every manager but apt. A pattern nobody checked is how a
  user gets told to install a package that does not exist; "unmapped" is already
  an answer the index prints.
