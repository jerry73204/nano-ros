# Workspace Examples

These are product-shaped nano-ros workspaces. They use the package roles
documented in the book:

- `src/*_pkg/`: Node packages with reusable node code only.
- `src/demo_bringup/`: Bringup package with `package.xml`, `system.toml`,
  `launch/`, and optional config files. It has no build file.
- `src/*_entry/`: Entry packages with the `main()` for each target platform.
  Multiple entries may share the same Node and Bringup packages.

Build them with the user workflow:

```bash
source ./activate.sh
cd examples/workspaces/<rust|c|cpp|mixed>
nros setup native
nros sync
nros codegen-system --bringup demo_bringup
```

Then use the platform build tool:

```bash
cargo build -p native_entry
# or
cmake -S . -B build && cmake --build build
```

The Rust workspace ships several Entry packages that all reuse the same Node
and Bringup packages: `src/native_entry/` (host), `src/qemu_freertos_entry/`,
`src/threadx_linux_entry/`, `src/esp32_entry/` (ESP32-C3 QEMU, OpenETH), and
`src/zephyr_entry/` (every Zephyr board). Each picks the board / RMW at build
time; the Entry source is the same one-line
`nros::main!(launch = "demo_bringup:system.launch.xml")`.

## ESP32-C3 QEMU Entry

`src/esp32_entry/` is a single bare-metal image that hosts the whole launch
graph (talker + listener) in one process on the CI-runnable OpenETH board.
It builds for `riscv32imc-unknown-none-elf` with the pinned nightly + `-Z
build-std` (the workspace fixture lane supplies both); the macro routes
`deploy = "esp32-qemu"` to a `#[esp_hal::main]` entry that drives the
board's real-runtime `Esp32QemuEntry`. Build + run via:

```sh
just esp32 build-examples        # builds src/esp32_entry through the workspace lane
just esp32 zenohd &              # router on port 7454
# flash + boot the image under the Espressif qemu fork, then observe /chatter
```

## Zephyr Entry

`src/zephyr_entry/` is a single Zephyr application that hosts the whole launch
graph (talker + listener) in one process, and covers **every Zephyr board** —
the board is chosen at `west build -b` time, not baked into the package. On
Zephyr the RTOS framework is the workflow: `west build` is the build verb and
Kconfig selects the RMW. There is no `nros build` / `nros launch` build path.

```bash
source ./activate.sh

# Platform-agnostic message provisioning (once; sibling to `west update`).
nros sync

# west is the build verb. `-b` picks the board; the -DCONF_FILE Kconfig
# overlay picks the RMW (prj-zenoh.conf / prj-xrce.conf / prj-cyclonedds.conf).
west build -b native_sim/native/64 src/zephyr_entry \
    -- -DCONF_FILE="prj.conf;prj-zenoh.conf"

west build -t run            # native_sim; `west flash` for hardware
```

See the book's [Images → Running on Zephyr](../../book/src/getting-started/workspace-entry-pkg.md)
for the full flow.

## FVP board-crate Entry (ws-realtime-cpp-fvp)

`ws-realtime-cpp-fvp/src/fvp_entry/` (phase-292 W1.a) is the board-crate
variant of the Zephyr Entry: instead of `west build -b <board>` the entry's
CMakeLists calls `nano_ros_use_board(fvp-aemv8r-smp)` BEFORE
`find_package(Zephyr)`, and the board id / base config / DTS overlay /
default RMW (cyclonedds) / runner all flow from
`packages/boards/nros-board-fvp-aemv8r-smp/`. This is the Autoware Safety
Island reference-consumer Entry shape. Build via
`just zephyr build-fvp-ws-entry` (part of `just zephyr build-fvp-all`).
