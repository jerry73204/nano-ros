---
id: 910
title: "migrating to zenoh-pico 1.10: the serial layer moved, `config.h` is no
  longer shipped, and our config generator is 54 knobs behind"
status: open
type: task
area: rmw, build
related: [phase-415, issue-0852, issue-0882]
---

## Why

Our pin is 1863 upstream commits behind. The gap matters for serial specifically:
upstream **restructured the link transports** and now ships its own Zephyr UART
backend, so most of our Zephyr serial code has an upstream equivalent that did
not exist when we wrote it.

## What upstream changed

| | our pin (`dac320e3`) | upstream 1.10 |
| --- | --- | --- |
| Zephyr serial | ours, in `src/system/zephyr/network.c` | theirs, `src/link/transport/serial/uart_zephyr.c` |
| serial protocol | `src/system/common/serial.c` (ours) | `src/link/transport/upper/serial_protocol.c` |
| lwIP / sockets | `src/system/freertos/lwip` | `src/link/transport/**`, `src/system/socket` |
| `include/zenoh-pico/config.h` | checked in | **generated** from `config.h.in` by their CMake |
| system ABI (`_z_mutex_*`, `_z_task_*`) | `src/system/zephyr/` | unchanged — nano-ros's shims still fit |

The last row is the good news: `src/system/zephyr/system.c` still exists, so
`nros_zenoh_zephyr_system.c` continues to override the symbols it always did.

## Progress so far

Two branches on the fork (`jerry73204/zenoh-pico`), both off `upstream/main`:

- **`serial-fixes`** — one commit, for the upstream PR. Fixes
  `_z_zephyr_uart_read`, which busy-spins on `uart_poll_in` with no timeout, no
  yield and no `uart_err_check`, and returns `len` regardless. All three are
  defects we hit and fixed on our own port; the third is
  [issue 0852](0852-zephyr-serial-rx-is-polled-and-overruns.md), where a silent overrun cost a long investigation that
  ended in a bug report against an innocent peer.
- **`nros-integration`** — `serial-fixes` plus a passthrough
  `include/zenoh-pico/config.h`, which upstream no longer ships. Not for
  upstream. A passthrough suffices because every platform in
  `config/*/nros-platform.toml` defines `ZENOH_GENERIC`; the 48 `@TOKEN@`
  substitutions are all in the dead `#else` branch, which is now an `#error` so
  a platform that forgets the define fails loudly.

Build progress against `nros-integration`, each item found by building and
fixed in turn:

1. `nros-platform.toml` named `system/freertos/lwip`, which no longer exists →
   `system/freertos`. (Reverted for now; correct only against the new pin.)
2. `zenoh-pico/config.h` missing → the passthrough above.
3. **Current blocker**, below.

## Current blocker — the config generator is 54 knobs behind

`nros-zpico-build`'s `config_header()` emits 49 of the 95 `Z_*` knobs upstream's
template defines. The build stops on the first missing ones:
`Z_CONFIG_SESSION_ZID_KEY`, `Z_GET_TIMEOUT_DEFAULT`, `Z_RUNTIME_MAX_TASKS`,
`Z_ZID_LENGTH`.

Of the 54 missing: **27 are `*_KEY`** (fixed protocol strings, not tunables),
**12 are `*_DEFAULT`**, and the rest are numeric knobs whose values are the 54
`set(Z_… CACHE STRING …)` lines in upstream's `CMakeLists.txt`.

**Do not transcribe them by hand.** That is what leaves us 54 behind again at
the next bump. `config_header()` should emit only the knobs nano-ros actually
*tunes* — buffer sizes, lease, feature flags — and derive the rest from
upstream's `CMakeLists.txt` defaults at build time. Then a bump costs nothing.

## Also outstanding

- One TU compiles without `ZENOH_GENERIC` and trips the new `#error`. Which one
  is not yet identified; the `#error` exists so it cannot be missed.
- Once it builds, our Zephyr serial code in `src/system/zephyr/network.c`
  should be **deleted** in favour of upstream's `uart_zephyr.c` plus the
  `serial-fixes` commit. That is the point of the migration.
- The INIT-retry scoping (fork commit `67ee0224`) is **not** needed upstream:
  upstream's `_z_connect_serial` never retries, so it never had the flood. That
  commit fixed a regression of ours and should not be carried forward.

## State

The submodule pin is back on `dac320e3` and the board builds. Nothing here is
half-applied; the migration lives on the two fork branches.


## 2026-09-03 — the target changes to 1.8.0, and the work moves to phase-415

This issue is written around **1.10**. Measured in the `ros2` distrobox with the
apt index refreshed: ROS Humble ships zenoh **1.8.0**
(`ZENOH_C "1.8.0"` in `zenoh_configure.h`; `ros-humble-rmw-zenoh-cpp` 0.1.9,
installed == candidate, built 2026-07-23 and unchanged since). We track what ROS
adopts, so 1.10 — two minors AHEAD of ROS — is the wrong target and
`nros-integration-1.10` is further from ROS than our 1.7.2 line is.

None of that branch's three nano-ros commits ports to 1.7.2 either: one is
1.10-only by construction (`config.h` becomes CMake-generated there), one
targets files that do not exist at 1.7.2, and one would REGRESS us — our serial
read already has the deadline, the `k_yield()` and the issue-0852 interrupt-driven
RX that commit lacks.

The 1.7.2 -> 1.8.0 port is surveyed in
[phase 415](../roadmap/archived/phase-415-zenoh-pico-1-8-0-patch-line.md): 85 files apply
clean, **42 conflict**, 2 moved, and one genuine name collision — our added
`session/keyexpr.h` versus upstream's `protocol/keyexpr.h` moved to the same
path. That is a decision, not a merge.

This issue stays open for the parts that are not the version bump: the config
generator being 54 knobs behind, the `ZENOH_GENERIC` TU, and the note that fork
commit `67ee0224`'s INIT-retry must NOT be carried forward because upstream
never had the flood it fixed.

## Update 2026-09-04 — 1.8.0 landed; this issue stays open for 1.10

Phase 415 moved the patch line to **1.8.0**, the zenoh ROS Humble actually
ships (`rmw_zenoh_cpp` 0.1.9 → `ZENOH_C "1.8.0"`). That is the version the
tracking rule names, so it closes the *drift* half of this issue. 1.10 is still
two minors AHEAD of ROS and remains the wrong target under that rule — this
issue stays open as the record of what a later move would cost, not as work
that is queued.

**One row of the table above is wrong and matters, because it is used as an
argument for 1.10 being special.** `include/zenoh-pico/config.h` is not
"checked in at our pin, generated at 1.10": **1.7.2 and 1.8.0 both generate it**,
via `configure_file(config.h.in -> config.h)` writing back into the SOURCE tree
(`CMakeLists.txt:345` at 1.7.2, `:379` at 1.8.0). It only *looks* checked in
because the generated file is committed and because `zpico-sys` compiles the
sources directly and never runs that CMakeLists — so for nano-ros the committed
`config.h` is the effective config and the generator never fires.

The practical consequence, found by building 1.8.0 the upstream way during the
port: anything we write into `config.h` alone is erased by the first `cmake`
configure. Two knobs were in that position and are now in `config.h.in`; the
`#ifndef` wrapping from `49012370` still is not (118 lines). Details and the
sweep command are in [phase-415](../roadmap/archived/phase-415-zenoh-pico-1-8-0-patch-line.md).
