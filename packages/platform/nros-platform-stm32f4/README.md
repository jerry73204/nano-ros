# nros-platform-stm32f4

Bare-metal platform for STM32F4 Cortex-M4F MCUs (e.g.
STM32F407VG / F429ZI). Pairs with the
`nros-board-stm32f4` board crate.

## Role

Implements the trait family in
[`nros-platform-api`](../nros-platform-api) for a single-core
Cortex-M4F with no kernel: DWT cycle counter for the monotonic clock,
a linker-script heap region with a bump allocator, no threading,
networking shipped from the board crate via
[`nros-smoltcp`](../../drivers/net/nros-smoltcp).

## Source layout

| File | Role |
|------|------|
| `src/lib.rs` | `Stm32F4Platform` zero-sized type + trait impls. |
| `src/clock.rs` | DWT cycle counter driver (HSE-aware). |
| `src/memory.rs` | Bump allocator. |
| `src/random.rs` | RNG peripheral if the SoC has one; LCG fallback otherwise. |
| `src/critical_section.rs` | `critical-section` impl backed by PRIMASK. |

## When to use

- STM32F4 series MCU bare-metal port.
- Reference real-hardware target (the QEMU port is for STM32F4-Discovery
  emulation only).

## Caveats

- DWT counter requires `CYCCNTENA` to be set early in boot — the board
  crate handles this in `init_hardware`.
- Networking depends on the board crate wiring an Ethernet driver +
  smoltcp.

## See also

- [Custom Platform porting guide](../../../book/src/porting/custom-platform.md)
- Source on GitHub: <https://github.com/NEWSLabNTU/nano-ros/tree/main/packages/platform/nros-platform-stm32f4>
