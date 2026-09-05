# nros-platform-mps2-an385

Bare-metal platform for the ARM **MPS2-AN385** Cortex-M3 dev board
(also the QEMU `mps2-an385` machine model). Pairs with the
[`nros-board-mps2-an385`](../../boards/nros-board-mps2-an385) /
[`nros-board-mps2-an385-freertos`](../../boards/nros-board-mps2-an385-freertos)
board crates.

## Role

Implements the trait family in
[`nros-platform-api`](../nros-platform-api) for a single-core
Cortex-M3 with no kernel: CMSDK Timer0 for the monotonic clock, a
linker-script-defined heap region with a bump allocator, no threading
(stubbed `task_*`, no-op mutex/condvar), no native networking — that
ships from the board crate via [`nros-smoltcp`](../../drivers/net/nros-smoltcp).

## Source layout

| File | Role |
|------|------|
| `src/lib.rs` | `Mps2An385Platform` zero-sized type + trait impls. |
| `src/clock.rs` | CMSDK Timer0 driver + ms/us conversion. |
| `src/memory.rs` | Bump allocator over the linker-script heap region. Heap budget gated by the `dds-heap` Cargo feature (2 MB) for DDS variants. |
| `src/random.rs` | Seeded LCG (no entropy source on AN385). |
| `src/critical_section.rs` | `critical-section` impl backed by PRIMASK. |

## When to use

- Bare-metal Cortex-M3 development.
- QEMU `mps2-an385` machine model for CI.

## Caveats

- Single-threaded — `task_init` returns -1, mutex/condvar ops are
  no-ops. Safe because zenoh-pico's lease task spawn is gated on
  `task_init` success and the application drives the spin loop itself.
- The bump allocator is one-shot — no `dealloc`, just leaks. Sized at
  config time via the `dds-heap` feature for DDS use cases.
- Networking depends on a board crate wiring up
  [`lan9118-smoltcp`](../../drivers/net/lan9118-smoltcp) +
  [`nros-smoltcp`](../../drivers/net/nros-smoltcp) ; this crate provides
  only `PlatformNetworkPoll`.

## See also

- [Custom Platform porting guide](../../../book/src/porting/custom-platform.md)
- CLAUDE.md "smoltcp Multicast" pitfalls
- Source on GitHub: <https://github.com/NEWSLabNTU/nano-ros/tree/main/packages/platform/nros-platform-mps2-an385>
