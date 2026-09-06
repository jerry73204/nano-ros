# Zephyr (west module)

Single-node starter on Zephyr via the in-tree `zephyr/`
west module. nano-ros ships as a Zephyr module — `west` discovers it
from your workspace's `west.yml`, drops in a `prj.conf` Kconfig
surface, and the standard `west build` / `west flash` flow takes
care of the rest.

> **Contributor path?** Building nano-ros's own Zephyr examples
> straight from this repository (no west-managed workspace) is
> covered at [Zephyr (contributor)](./zephyr.md). The page below is
> the canonical user entry.

> **Just want a working starter?** Clone the
> [`nano-ros-zephyr-example`](https://github.com/NEWSLabNTU/nano-ros-zephyr-example)
> repo (`west init -m …`) — a manifest + zenoh talker app pinned to a tested
> Zephyr, with the same quickstart as below baked in. The steps here explain what
> it does so you can adapt it to your own workspace.

> **Prereqs.** Run `nros setup zephyr --rmw <rmw>` once (see
> [Prerequisites](#prerequisites) below) — it provisions the Zephyr
> west workspace + Zephyr SDK bits, the emulator, and your RMW's host
> daemon into the shared store. No hand-installed Zephyr SDK, `west`,
> or cross-toolchain, and no ROS 2 install required.
> **Python: 3.10+ on Zephyr 3.7 LTS, but ≥ 3.12 on Zephyr 4.x**
> (4.x's `find_package(Python3)` requires 3.12 — see the version
> matrix below). nano-ros's imported west fragment
> `zephyr/west.yml` is a manifest-only file — it does NOT
> pull Zephyr itself; that has to be in your parent manifest
> (`zephyrproject-rtos/zephyr`).

## Which Zephyr? — 3.7 LTS and 4.x both supported

nano-ros consumes as a module on **both** the **3.7 LTS** line (supported
to Jan 2027; the safety-island default) and the current **4.x** rolling
line. You build against **whatever Zephyr your workspace already pins** —
nano-ros adapts. The two lines differ only in *how* you select an RMW and
apply nano-ros's Zephyr patches:

| Capability | Zephyr 3.7 LTS | Zephyr 4.x |
| --- | --- | --- |
| Min Python | 3.10 | **3.12** (`find_package(Python3)`) |
| RMW selection | `prj-<rmw>.conf` overlay (`-DCONF_FILE=...`) | **`-S nros-<rmw>` snippet** (or the overlay) |
| nano-ros patches | applied during `nros setup zephyr` provisioning | applied during `nros setup zephyr` provisioning |
| Examples as samples / Twister | — | **`samples:` + Twister** (`sample.nano-ros.*`) |
| zenoh (native_sim) | ✅ build + e2e | ✅ build + e2e |
| cyclonedds (native_sim) | ✅ build + e2e | ✅ build · publish · receive · multicast-join *(stable 2-node run pending a tracked `k_mutex` fix)* |
| xrce | ✅ | build path WIP |

native_sim networking uses **NSOS** (host loopback) on both lines — no
TAP/bridge/root. The copy-out, snippet, patch-apply, and dual-line build
flows are exercised in CI (contributor lanes `just zephyr ci-both`,
`just zephyr check-copy-out`).

## Project layout

A Zephyr workspace using nano-ros looks like any other Zephyr
project — the **nano-ros module sits beside Zephyr**, your
application sits beside both:

```text
my_zephyr_ws/
├── .west/
├── zephyr/                            # cloned by `west init`
├── modules/
│   └── nano-ros/                      # imported via west.yml
└── apps/
    └── my_app/                        # your application
        ├── CMakeLists.txt
        ├── prj.conf                   # Kconfig — selects nros + RMW
        ├── west.yml                   # (optional) per-app manifest
        └── src/
            └── main.c                 # nros user code
```

The application `CMakeLists.txt` is a stock Zephyr app — `find_package(Zephyr)`
+ `target_sources`. **No `add_subdirectory(<nano-ros>)`** is needed;
the module shell handles it once `CONFIG_NROS=y` flips on.

A minimal `apps/my_app/` looks like this — the shipped
[`examples/zephyr/c/talker/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/zephyr/c/talker)
quoted directly (only the project/class names are trimmed to `my_app`), so
this page can't drift from the real API surface. It is a **stateful C
component** (a struct + a `configure` function, RFC-0043 / phase-244.C2) —
not a hand-written `main()` — because the Zephyr typed carrier generates
the entry point and calls into the component by identity:

```cmake
# apps/my_app/CMakeLists.txt
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_app)

set(NANO_ROS_PLATFORM zephyr)
include("${NROS_REPO_DIR}/cmake/NanoRosNodeRegister.cmake")
include("${NROS_REPO_DIR}/cmake/NanoRosVerbs.cmake")

nano_ros_auto_add_library(talker_lib STATIC src/Talker.c)

nros_components_register_node(talker_lib
    PLUGIN my_app::Talker
    EXECUTABLE talker
    SHAPE configure
    TYPED
    DEPLOY zephyr)
```

```c
// apps/my_app/src/Talker.c
// `talker_configure` creates a raw publisher on `/chatter` + a timer that
// publishes a CDR-encoded Int32 counter each tick. `NROS_C_COMPONENT` emits
// the C-ABI factory/configure the Zephyr typed Entry carrier calls.
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <nros/component.h>

typedef struct {
    _Alignas(8) uint8_t pub[NROS_C_PUBLISHER_STORAGE_SIZE];
    int32_t count;
} talker_t;

static void write_u32_le(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void on_tick(void* ctx) {
    talker_t* self = (talker_t*)ctx;
    /* std_msgs/Int32 CDR: 4-byte encapsulation header (CDR_LE) + int32 data. */
    uint8_t buf[8];
    buf[0] = 0x00;
    buf[1] = 0x01;
    buf[2] = 0x00;
    buf[3] = 0x00;
    write_u32_le(buf + 4, (uint32_t)self->count);
    if (nros_cpp_publish_raw(self->pub, buf, sizeof(buf)) == 0) {
        printf("Published: %d\n", (int)self->count);
    }
    self->count++;
}

static nros_ret_t talker_configure(const nros_cpp_node_t* node, void* executor, talker_t* self) {
    self->count = 0;
    int32_t rc = nros_cpp_publisher_create(node, "/chatter", "std_msgs::msg::dds_::Int32_", "",
                                           nros_c_qos_default(), self->pub);
    if (rc != 0) {
        return rc;
    }
    size_t timer_handle;
    return nros_cpp_timer_create(executor, /*period_ms=*/500, on_tick, self, &timer_handle);
}

NROS_C_COMPONENT(talker_t, talker_configure)
```

See [`examples/templates/zephyr-byo/app/src/Talker.c`](https://github.com/NEWSLabNTU/nano-ros/blob/main/examples/templates/zephyr-byo/app/src/Talker.c)
for the up-to-date source (this is that BYO starter's talker, only stripped
of its file header comment; the canonical in-tree
[`examples/zephyr/c/talker/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/zephyr/c/talker)
uses the generated **typed** `std_msgs/String` bindings instead of raw CDR).
Note the `nros_cpp_*` symbol prefix: those are C-ABI
functions from `nros-cpp` (the `cpp` is a namespace prefix, not C++
linkage) — a C component links against them directly, and shares the same
executor + node as a C++ component would.

`prj.conf` is the one shown in [Configure](#configure) below — note it needs
**both** `CONFIG_NROS_C_API=y` and `CONFIG_NROS_CPP_API=y`: the typed
Zephyr carrier that drives this component's `configure` is C++, so the
`nros-cpp` header surface must be on the app include path even though
`Talker.c` itself is C.

## Prerequisites

`nros setup` provisions the parts nano-ros owns — the **RMW host daemon**
(`zenohd` / Micro-XRCE-DDS agent) and the **RMW transport submodules**
(zenoh-pico + mbedtls for zenoh, the cyclonedds fork) — from a pinned index into
`${NROS_HOME:-~/.nros}/sdk` / the nano-ros checkout. It does **not** replace Zephyr's own SDK,
and interface codegen still needs the ROS message definitions.

1. **Build the in-tree `nros` CLI** (Phase 218, from the nano-ros checkout):
   ```bash
   ./scripts/bootstrap.sh      # builds packages/cli/target/release/nros
   source ./activate.sh        # OR: direnv allow / source ./activate.fish
   ```
2. **Provision the RMW (daemon + transports)** from the nano-ros checkout:
   ```bash
   ( cd modules/nano-ros && nros setup zephyr --rmw zenoh )  # zenohd + zenoh-pico + mbedtls
   ( cd modules/nano-ros && nros setup --source px4-rs )     # workspace cargo-load dep
   ```
3. **Install the Zephyr SDK** the standard Zephyr way (`nros setup` does *not*
   provide it) and expose it — `export ZEPHYR_SDK_INSTALL_DIR=/path/to/zephyr-sdk-<ver>`
   (or register it via the SDK's `setup.sh -c`).
4. **Message definitions for codegen.** The interface codegen resolves a
   package's `msg/*.msg` from `NROS_<PKG>_DIR` (e.g.
   `export NROS_STD_MSGS_DIR=/opt/ros/humble/share/std_msgs`) — point it at a ROS
   install or any dir holding the `.msg` files.

The RMW host daemon must be **running** before an example connects
(for zenoh: `ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false'
ros2 run rmw_zenoh_cpp rmw_zenohd`; the Micro-XRCE-DDS agent for xrce).

## Configure

Add nano-ros (and a Zephyr pin — `zephyr/west.yml` is manifest-only;
it does **not** pull Zephyr itself) to your workspace `west.yml`:

```yaml
manifest:
  remotes:
    - name: nano-ros
      url-base: https://github.com/NEWSLabNTU
    - name: zephyr
      url-base: https://github.com/zephyrproject-rtos
  projects:
    - name: zephyr
      remote: zephyr
      revision: v3.7.0          # or your chosen 3.7 LTS / 4.x SHA
      path: zephyr
      import: true               # pulls Zephyr's own modules
    - name: nano-ros
      remote: nano-ros
      revision: nros-v0.5.0      # pin a release tag (west needs SOME
                                 # revision — its default `master` does
                                 # not exist here). `main` works for
                                 # tracking tip, but a tag keeps your
                                 # build reproducible.
      path: modules/nano-ros
      import:
        file: zephyr/west.yml    # pulls nano-ros's transport deps
```

Then per-application `prj.conf`:

```
CONFIG_NROS=y
CONFIG_NROS_C_API=y
CONFIG_NROS_RMW_ZENOH=y                 # bool per RMW: NROS_RMW_{ZENOH,XRCE,CYCLONEDDS}
# (ROS edition is a build-time Cargo feature, NOT a Kconfig symbol — do not set
#  CONFIG_NROS_ROS_EDITION; Zephyr aborts on the undefined symbol.)

# Required for any networked RMW on QEMU / native_sim:
CONFIG_NETWORKING=y
CONFIG_NET_IPV4=y
CONFIG_NET_TCP=y
```

`CONFIG_NROS=y` activates the shell, which maps Kconfig values to
`NANO_ROS_*` CMake cache vars and `add_subdirectory()`s the root
nano-ros CMake. `NanoRos::NanoRos` is linked into your `app`
library transparently.

## Build

If your workspace is a fresh manifest-only dir (no `.west/`), initialise
it first so `west` knows which `west.yml` is the manifest:

```bash
cd my_zephyr_ws
west init -l .                           # one-time; points west at the
                                         # local west.yml in cwd
west update                              # clones nano-ros + Zephyr into the workspace
```

(If you started from `west init -m <remote>`, both calls above are
already done — go straight to `west build` below.)

### `nros build` on Zephyr: it hands off, it does not take over

The other platforms let `nros build` own the build: it generates a cargo or
CMake root from your `[image.*]` declarations and runs the tool. **Zephyr is not
like that, on purpose.**

You own the west workspace and the application. `west init` / `west update`
populate the workspace, your app carries its own `CMakeLists.txt` and
`prj.conf`, and `west` only runs from inside that workspace. None of that is
ours to generate — an app's Kconfig is authored, not derived (RFC-0065 D5).

So for a Zephyr image `nros build`:

1. resolves the **application** — the entry package for that image's board,
   not the bringup;
2. resolves **your Zephyr** — `$ZEPHYR_BASE`, else `$NROS_ZEPHYR_WORKSPACE`,
   else an in-tree `zephyr-workspace/`, else a sibling `../nano-ros-workspace`.
   Note it is a *Zephyr* that is needed, not a `.west/` directory: with
   `ZEPHYR_BASE` set, `west build` runs from anywhere, which is what makes a
   **freestanding application** — one outside the west workspace — work;
3. runs `west build` from there with the image's overlays
   (`EXTRA_CONF_FILE`, `APPLICATION_CONFIG_DIR`) already applied.

If it cannot find a workspace it does **not** guess. It prints the command and
where to run it:

```text
$ nros build demo_bringup:zephyr
Error: no west workspace found, so `west build` cannot be run for `zephyr`.

Zephyr differs from the other drivers: YOU own the west workspace and the
application, and `west` only runs inside a workspace …

Run this from your west workspace:

    west build -b native_sim/native/64 …/src/zephyr_entry

Point nros at your workspace:

    NROS_ZEPHYR_WORKSPACE=<dir>   # the dir containing zephyr/
```

That is deliberate and it is the same boundary `nros setup --system` draws when
it composes an install command and prints it rather than running `sudo` for you:
where the tool is yours, nano-ros hands you the command instead of assuming it
may act.

**The plain `west build` below keeps working and is still the primary flow.**
`nros build` is the convenience that applies an image's overlays for you; it
never becomes a required layer between you and west.

#### Starting from nothing: `nros new node` and `nros new entry`

A node package is board- and RMW-agnostic — it names no platform, which is what
lets the same package be deployed to Linux and to a Cortex-M without an edit:

```bash
nros new node talker_pkg
```

It writes the package and adds the `[[component]]` row to your bringup. (Use
`nros new <name> --platform native` only for a standalone *runnable project* —
it pins a board crate, so an entry on another platform cannot link it.)

#### The entry: `nros new entry`

You do not have to write the entry package by hand:

```bash
nros new entry zephyr_entry --platform zephyr
```

```text
nros new entry: scaffolded ./src/zephyr_entry (7 file(s))
  declared [image.zephyr_entry] in ./src/demo_bringup/system.toml

Next:
  nros sync
  nros build demo_bringup:zephyr_entry --zephyr-workspace <dir>
```

It writes the package — `Cargo.toml`, `CMakeLists.txt`, `build.rs`,
`src/lib.rs`, `prj.conf`, `prj-zenoh.conf`, `boards/<board>.conf` — and three
things outside it that are easy to miss and invisible until they fail:

* the **`[image.*]` block**, with `board`, `entry` and the `conf` overlay the
  entry's own `CMakeLists.txt` requires;
* an **`exclude` entry in every enclosing cargo workspace**, because a
  west-built `staticlib` must not be an ordinary cargo member;
* a **path dependency per node package** the bringup declares, since the entry
  links them.

`--board`, `--rmw` and `--bringup` override the defaults
(`native_sim/native/64`, `zenoh`, and the workspace's single bringup).

#### The whole flow, end to end

Five commands, run from your workspace. This is the `examples/workspaces/rust`
transcript, verbatim:

```bash
# 0. tell nros where your west workspace is — the directory containing
#    `zephyr/`. On the command line:
#
#      nros build demo_bringup:zephyr --zephyr-workspace /path/to/zephyr-workspace
#
#    or once for the shell, if you would rather not repeat it:
export NROS_ZEPHYR_WORKSPACE=/path/to/zephyr-workspace

# 1. generate the message bindings this workspace's packages depend on.
#    `nros build` refuses with "this workspace has never been synced" until
#    you do — it will not silently build against absent bindings.
nros sync

# 2. build. This resolves the application, applies the image's overlays and
#    runs `west build` for you.
nros build demo_bringup:zephyr

# 3. start the router ROS itself uses — the one `rmw_zenoh_cpp` ships, so it
#    cannot drift from the RMW your ROS 2 nodes use. Separate terminal; it
#    keeps running. Listens on tcp/127.0.0.1:7447.
ros2 run rmw_zenoh_cpp rmw_zenohd

# 4. run the image.
./build/zephyr/zephyr.exe
```

```text
*** Booting Zephyr OS build v3.7.0 ***
<inf> nros_net_wait: Network ready (NSOS — host kernel sockets)
<inf> rust: rustapp: nros: zephyr workspace entry up (2 nodes)
<inf> rust: talker_pkg: talker publishing chatter seq=0
<inf> rust: talker_pkg: talker publishing chatter seq=1
```

and from a ROS 2 shell on the same router, the nodes are ordinary ROS nodes:

```console
$ export RMW_IMPLEMENTATION=rmw_zenoh_cpp
$ ros2 topic echo /chatter --once
data: 6
---
```

Step 3 is not optional and its ORDER does not matter, but the **locator** does:
zenoh-pico is a client, so with no router the image gets

```text
<err> rust: nros: zephyr entry — executor open failed: Transport(ConnectionFailed)
```

which names no port. Both sides default to `tcp/127.0.0.1:7447` — the port
`rmw_zenohd` listens on and a `rmw_zenoh_cpp` node connects to. Override with
`CONFIG_NROS_ZENOH_LOCATOR` on the image side (via a `conf` fragment) and
`ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:<port>"]'` on the
router, and change **both**.

#### What an image declares

The Zephyr images in `examples/workspaces/*` are the worked examples:

```toml
[image.zephyr]
board = "native_sim/native/64"
entry = "zephyr_entry"          # only when several entries match, see below
conf  = ["prj-zenoh.conf"]      # the RMW overlay this app requires
```

* **`conf`** names the app's own Kconfig fragments, in order. They are looked
  for beside the board config dir, then **beside the application**, then beside
  the bringup — the app rung matters because a Zephyr app keeps its
  `prj-<rmw>.conf` next to the `CMakeLists.txt` west builds. These entries
  `FATAL_ERROR` without an RMW overlay, so the image has to say which one.
* **`board`** is the Zephyr board target — and also a name nano-ros must know.
  See below.
* **`entry`** names the application package. Normally leave it out: an entry
  declares the deploy target it serves — `[package.metadata.nros.entry] deploy`
  in `Cargo.toml`, `nano_ros_add_executable(... DEPLOY zephyr)` in
  `CMakeLists.txt` — and one package usually claims a given board. Set it when
  several do (`realtime-cpp` has `zephyr_entry` and `fvp_entry`, both
  `DEPLOY zephyr`, on the same board, for two images that differ in payload).
  Deriving there is a coin flip, so `nros build` refuses and lists the
  candidates rather than picking one.

#### If you live in west: `west nros`

Everything above assumes you stand in your nano-ros workspace and name the
Zephyr one. If you work the other way round — inside the west workspace — west
already knows where Zephyr is, so it can tell `nros` for you:

```bash
cd ~/zephyr-workspace
west nros build --workspace ~/my_robot demo_bringup:zephyr
```

No `ZEPHYR_BASE`, no `NROS_ZEPHYR_WORKSPACE`, no `--zephyr-workspace`. Every
argument after `nros` is passed straight through, so anything `nros` accepts
works here unchanged.

It picks up automatically in any workspace whose manifest lists nano-ros.
`west nros` is now the only nano-ros west extension: `west fvp` shared that
route until RFC-0064 revision 5 retired it, since its body was environment
wiring in front of the stock `west build -t run`. Two things worth knowing:

* **It only exists inside a west workspace.** West loads extension commands
  from the manifest, so from an unrelated directory you get
  `west: unknown command "nros"`. That is why this is an addition rather than a
  replacement: it swaps which of the two trees you have to name, trading the
  Zephyr path (which differs per machine) for your own workspace path (which
  you chose).
* **It never overwrites a `ZEPHYR_BASE` you set yourself** — an exported one is
  a deliberate choice, and this only fills in a missing answer.

#### Which boards you can name

`board` does two jobs from one string: it is passed to `west build -b`
verbatim, and it is looked up in nano-ros's board catalog, where a descriptor
carries a set of names:

```toml
names = ["zephyr", "native_sim/native/64"]
```

So it is not free-form. A board the catalog does not know is refused with the
list of ones it does, because the descriptor supplies more than the `-b` string
— the platform, the toolchain, the entry kind, the declared capabilities. There
is no safe default for those.

**A board is a package — put it in your workspace.** Drop an `nros-board.toml`
beside a package's `package.xml` and it joins the catalog like any other
package in `src/`:

```text
my_robot/src/
├── talker_pkg/
└── my_board/
    ├── package.xml
    └── nros-board.toml
```

```toml
# src/my_board/nros-board.toml
[[board]]
names = ["my-board"]              # what your images say
west_board = "qemu_cortex_m3"     # what west is given
platform = "zephyr"
toolchain = "stable"
platform_feature = "platform-zephyr"
link_kind = "none"
entry_kind = "zephyr-staticlib"
```

```toml
# src/demo_bringup/system.toml
[image.zephyr]
board = "my-board"
```

No environment variable, nothing outside the tree, nothing copied into the
nano-ros checkout. `west_board` is what lets the friendly name be the one your
workspace uses — leave it out when your board name already *is* the west board
target, which is what the shipped descriptors do.

For a board shared by **several** workspaces there is no single workspace to
put it in, and `$NROS_EXTRA_BOARD_PATH` still covers that: PATH-style, naming
directories shaped like `packages/boards/`.

`nros new board <name> --for-platform zephyr` scaffolds the descriptor. Note
this is the *nano-ros* descriptor; an out-of-tree Zephyr board **definition**
(its devicetree and `board.yml`) is contributed the Zephyr way, through your
module's `board_root`.

#### Adding your own drivers, modules and boards

**Everything Zephyr lets you extend, you still extend the Zephyr way.** The
application `CMakeLists.txt` is yours (RFC-0065 D5) and `nros build` runs
`west build` against it, so the extension points are untouched:

| you want | you do | `nros build` involvement |
| --- | --- | --- |
| an out-of-tree module | `list(APPEND ZEPHYR_EXTRA_MODULES <dir>)` before `find_package(Zephyr)` | none |
| a driver in that module | `zephyr_library()` + `zephyr_library_sources()`, gated on your own Kconfig | none |
| its devicetree binding | `dts/bindings/` + `settings: {dts_root: .}` in the module's `zephyr/module.yml` | none |
| an out-of-tree board / SoC / arch / snippet | the matching `*_root` in the same `module.yml` (`board_root`, `soc_root`, `arch_root`, `snippet_root`, `module_ext_root`) | none |
| turning your Kconfig on for one image | a `conf` fragment on `[image.*]` | passed as `EXTRA_CONF_FILE` |
| a devicetree overlay for one image | an `.overlay` in the same `conf` list | passed as `EXTRA_DTC_OVERLAY_FILE` |

Only the last two rows are ours, and they are how an IMAGE differs from its
siblings — the module itself is a property of the application, not of one
image, so it belongs in the app's `CMakeLists.txt` where you would have put it
anyway.

`nros build` passes exactly four things to west of its own —
`APPLICATION_CONFIG_DIR`, `EXTRA_CONF_FILE`, `EXTRA_DTC_OVERLAY_FILE`,
`FILE_SUFFIX`, all Zephyr's own variables — and there is deliberately **no
image key for arbitrary `-D` defines**. Something your application always needs
belongs in its `CMakeLists.txt`, which keeps one place to look.

#### Customizing the west command itself

Anything after `--` goes to west. `west build` has two argument zones, and
`nros build` routes each token to the right one using west's own flag list:

```bash
nros build demo_bringup:zephyr -- --pristine        # a west option
nros build demo_bringup:zephyr -- -t run            # build, then run it
nros build demo_bringup:zephyr -- -DMY_OPT=1        # a cmake option
nros build demo_bringup:zephyr -- -p always -DX=1   # both, in one go
```

which becomes, respectively, the west option zone and the cmake zone:

```text
west build -b <board> <app> -p always -- -DEXTRA_CONF_FILE=… -DX=1
```

`nros build --dry-run` prints that line without running it, so you can always
see exactly what west is being asked to do — and copy it if you would rather
drive west yourself.

**Two flags are refused rather than routed**, because the image already decides
them and a build must not be able to disagree with its own declaration:

```console
$ nros build demo_bringup:zephyr -- --sysbuild
Error: `--sysbuild` is decided by the image, not by the command line.
It comes from the presence of a `sysbuild.conf` beside the application.

Change it there, so one build cannot disagree with the declaration it was
resolved from.
```

The other is `-b`/`--board`, which comes from the image's `board`.

Verified by building one. An out-of-tree module with its own Kconfig, a driver
source gated on it, a devicetree overlay declaring a node and a binding shipped
from the module's `dts_root`, added to `examples/workspaces/rust`'s entry and
enabled through the image:

```toml
[image.zephyr]
conf = ["prj-zenoh.conf", "prj-user-extra.conf", "user-widget.overlay"]
```

```text
*** Booting Zephyr OS build v3.7.0 ***
user_extra: out-of-tree driver init, widget-id=42
<inf> rust: rustapp: nros: zephyr workspace entry up (2 nodes)
<inf> rust: talker_pkg: talker publishing chatter seq=0
```

The driver's `BUILD_ASSERT(DT_NODE_EXISTS(DT_NODELABEL(user_widget)))` is what
makes that a test rather than a screenshot: without the overlay reaching the
build, it fails to compile instead of quietly doing nothing.

A mistake in your module surfaces as Zephyr's own error, unmediated — a wrong
`kconfig:` path in `module.yml` reports

```text
ERROR: "kconfig" key in …/extra_module/zephyr/module.yml has value
"../Kconfig" which does not point to a valid Kconfig file.
```

which is the message you would get from a plain `west build`, because it is a
plain `west build`.

The transports + `px4-rs` come from the [prerequisites](#prerequisites) step
(`west update` clones nano-ros but **not** its submodules). With the Zephyr SDK +
`NROS_STD_MSGS_DIR` exported (also prerequisites), build your app — `nros` on
PATH is auto-resolved as the codegen tool:

```bash
# native_sim (POSIX, no QEMU). The 3.7 line needs the NSOS line overlay; apply
# the NSOS patches first (see "apply nano-ros's patches" below).
overlay="$PWD/modules/nano-ros/cmake/zephyr/native-sim-line-3.7.conf"
west build -b native_sim/native/64 apps/my_app -- -DCONF_FILE="prj.conf;$overlay"

# A real board, e.g. Cortex-A9 (no native_sim overlay):
west build -b qemu_cortex_a9 apps/my_app
```

(Verified end-to-end on a fresh BYO west workspace: this builds to
`zephyr.exe` and runs to `Published: 0` against a router started with
`ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd`.
On the Zephyr 4.4 line, `find_package(Python3)` requires ≥ 3.12 and you
select the RMW with `-S nros-zenoh` instead of the overlay.)

For a quick sanity check that the module is wired correctly:

```bash
west build -t menuconfig                 # confirm CONFIG_NROS=y is visible
```

## Rust applications

Two things differ for a **Rust** app (C/C++ apps skip this section):

1. **The Rust crate's `[lib]` must be named `rustapp`** (`crate-type =
   ["staticlib"]`) — a `zephyr-lang-rust` contract: its `rust_cargo_application()`
   links `librustapp.a`. The Cargo *package* name is free.

2. **Generate the interface crates + the `[patch.crates-io]` wiring for YOUR
   layout** — do **not** copy an in-repo example's `.cargo/config.toml`: its
   `../../../../packages/core/...` paths are repo-relative and break in a
   copied-out app. From your app dir, run (after the [Prerequisites](#prerequisites)
   `nros setup`, which provides the codegen toolchain + message sources):

   ```bash
   nros generate-rust --generate-config \
       --nano-ros-path "$PWD/../../modules/nano-ros/packages/core"
   ```

   This writes `generated/<pkg>/` (the message crates) and a `.cargo/config.toml`
   whose `[patch.crates-io]` points the `nros-*` crates at your
   `modules/nano-ros/packages/core/*` and the generated interfaces at
   `generated/*`. Adjust `--nano-ros-path` to your workspace's
   `modules/nano-ros/packages/core` (the dir holding `nros-core`, `nros-node`, …).
   The example apps' committed `.cargo/config.toml` is for the in-tree build only.

## Run

```bash
# 1. Start the router (ROS's `rmw_zenohd` — nano-ros ships no router
#    of its own) on port 7447, the port your app's Kconfig default
#    picks up (CONFIG_NROS_ZENOH_LOCATOR defaults to
#    "tcp/127.0.0.1:7447"; change either side to match):
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/0.0.0.0:7447"];scouting/multicast/enabled=false' \
    ros2 run rmw_zenoh_cpp rmw_zenohd &

# 2. Boot the app from your BYO west workspace:
# QEMU Cortex-A9:
west build -t run
# native_sim:
./build/zephyr/zephyr.exe

# 3. Verify from stock ROS 2 in another terminal. Match the message
# type to the app you booted: the canonical in-tree talker publishes
# std_msgs/String; the raw BYO Talker.c shown above publishes
# std_msgs/Int32.
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# Talker publishes best-effort; stock `ros2 topic echo` defaults to
# RELIABLE, so the QoS-mismatched echo silently delivers nothing.
# Force best-effort to receive:
ros2 topic echo /chatter std_msgs/msg/String --qos-reliability best_effort   # in-tree talker
# ros2 topic echo /chatter std_msgs/msg/Int32 --qos-reliability best_effort  # BYO my_app
```

> **Contributors:** nano-ros's own zephyr talker has a matching in-tree
> recipe for the canonical `native_sim` build path — see
> [Per-Platform Contributor Lanes](../internals/platform-lanes.md#zephyr).

The Zephyr boot banner runs first, then the talker fires: the
canonical in-tree talker prints `Publishing: 'Hello World: 1'`,
`Publishing: 'Hello World: 2'`, ... (Rust + C + C++ all count from
1, matching the official ROS 2 demo talker); the raw BYO `Talker.c`
above prints `Published: 0`, `Published: 1`, ...

**Readiness signal.** On `native_sim`, expect the first publish line
(`Publishing: 'Hello World: 1'` for the in-tree talker;
`Published: 0` for the BYO app) within 5 seconds of boot (or
`./build/zephyr/zephyr.exe`); on `qemu_cortex_a9` expect it within
~15 seconds (QEMU cold boot + Zephyr init). If no publish line
in 30 seconds:

1. Confirm `CONFIG_NROS=y` lit up via `west build -t menuconfig`;
   without it the module shell never `add_subdirectory`'s nano-ros.
2. Check `CONFIG_NETWORKING=y`, `CONFIG_NET_IPV4=y`, `CONFIG_NET_TCP=y`
   in `prj.conf` — Zephyr networking is opt-in.
3. Confirm `zenohd` reachable from the simulated network (Slirp
   needs `10.0.2.2:7447` on QEMU; native_sim uses host loopback).
4. See [Troubleshooting — First 10 Minutes](./troubleshooting-first-10-min.md).

**Zephyr 4.x build gotchas.**
- `Could NOT find Python3 ... required is at least "3.12"` — 4.x needs
  Python ≥ 3.12. Provision one without sudo (e.g. `uv venv --python 3.12
  .venv312 && uv pip install west -r zephyr/scripts/requirements.txt`) and
  run west through it (`.venv312/bin/python -m west build ...`), so the
  ROS descriptor-codegen subprocess still uses the system ROS Python.
- `attempt to assign the value ... to the undefined symbol ETH_NATIVE_POSIX`
  — that symbol was renamed `ETH_NATIVE_TAP` in 4.x; the version-aware
  NSOS overlay handles it (**contributors:** the in-tree `just zephyr
  build-one` recipe applies it automatically).

## Zephyr 4.x: select the RMW with a snippet

On 4.x, nano-ros ships `west` **snippets** so you pick the RMW on the
build line instead of hand-writing the overlay:

```bash
west build -b native_sim/native/64 -S nros-cyclonedds apps/my_app
#                                   ^^^^^^^^^^^^^^^^^^^  nros-zenoh | nros-cyclonedds | nros-xrce
```

The snippet (shipped via the module's `snippet_root`) carries the
RMW-common Kconfig — equivalent to merging `prj-<rmw>.conf`. The
`prj-<rmw>.conf` / `-DCONF_FILE` path still works (and is the only option
on 3.7).

## nano-ros patches into your workspace

nano-ros needs a few patches to Zephyr's `native_sim` NSOS driver
(`recvmsg`, IPv4-multicast) so the host-loopback-based examples work.
Both supported lines (3.7 LTS and 4.x) take the **same path** —
`nros setup zephyr` reads `zephyr/patches.yml` and applies each patch
against the workspace's Zephyr tree, sha256-checked. No extra step
on your side beyond the provisioning command (already run during
[Prerequisites](#prerequisites)).

To re-apply (e.g. after a `west update` reset the tree):

```bash
nros setup zephyr --rmw zenoh
```

(Earlier nano-ros revisions documented a `west patch apply` flow on
4.x — that required a workspace-side west extension that doesn't ship
with stock Zephyr. Phase 208.D.7 / E.9 unified both lines on the
provisioner. Cyclone-DDS-on-Zephyr patches stay baked into the pinned
cyclonedds submodule, not delivered through `patches.yml`.)

## Copy out an example as your starting point

The `examples/zephyr/<lang>/<role>/` dirs are **copy-out clean** — copy one
into your own app tree and it builds against the nano-ros module with no
reference back into the nano-ros repo:

```bash
cp -r modules/nano-ros/examples/zephyr/c/talker apps/my_app
# cyclonedds examples need the host idlc + the ROS message dirs:
export NROS_STD_MSGS_DIR=/opt/ros/humble/share/std_msgs   # PKG_DIR contract
west build -b native_sim/native/64 -S nros-zenoh apps/my_app
```

Cyclone `idlc` and the descriptor-gen scripts are located via the module's
exported cache vars (`NROS_CYCLONE_IDLC`, `NROS_CYCLONE_SCRIPTS_DIR`,
`NROS_CYCLONE_CMAKE_DIR`); message-package dirs come from `NROS_<PKG>_DIR`
env (defaulting to `/opt/ros/humble/share/<pkg>`). No `/opt/ros` or
repo-relative paths are baked into the example.

See the [`zephyr/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/zephyr)
module dir + its [`Kconfig`](https://github.com/NEWSLabNTU/nano-ros/blob/main/zephyr/Kconfig)
for the canonical in-repo surface.

## GitHub source

- Zephyr module shell:
  [`zephyr/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/zephyr)
- Worked examples:
  [`examples/zephyr/rust/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/zephyr/rust),
  [`examples/zephyr/c/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/zephyr/c),
  [`examples/zephyr/cpp/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/zephyr/cpp)
- Module manifest:
  [`zephyr/module.yml`](https://github.com/NEWSLabNTU/nano-ros/blob/main/zephyr/module.yml)
- Kconfig surface (canonical post-Phase-208.D.7 fold — every `CONFIG_NROS*`
  symbol the doc cites lives here):
  [`zephyr/Kconfig`](https://github.com/NEWSLabNTU/nano-ros/blob/main/zephyr/Kconfig)
- Patches applied by `nros setup zephyr` (the `west patch` flow on this page
  was retired in Phase 208.E.9):
  [`zephyr/patches.yml`](https://github.com/NEWSLabNTU/nano-ros/blob/main/zephyr/patches.yml)

## Next

- Pick a real board (Nordic, NXP, STM32, …): swap `-b <board>` and
  add a board-specific overlay to your `prj.conf`.
- Cyclone DDS on Cortex-A/R: see the DDS section of
  [Choosing an RMW Backend](../user-guide/rmw-backends.md) for the
  required Kconfig deltas.
- Build nano-ros's own Zephyr examples without west:
  [Zephyr (contributor)](./zephyr.md).
