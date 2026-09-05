# nros-platform-esp32-qemu

Bare-metal platform for **ESP32-C3** (RISC-V `riscv32imc`) running under
QEMU. Pairs with the
[`nros-board-esp32-qemu`](../../boards/nros-board-esp32-qemu) board
crate. Used for CI smoke tests of the bare-metal RISC-V code path
without real ESP32-C3 hardware.

## Role

Implements the trait family in
[`nros-platform-api`](../nros-platform-api) for ESP32-C3 + QEMU
OpenETH NIC: `esp_timer_get_time` for the monotonic clock,
[`esp-alloc`](https://github.com/esp-rs/esp-alloc) for the heap (gated by
the `dds-heap` Cargo feature for DDS variants), single-threaded,
networking through the board crate via
[`openeth-smoltcp`](../../drivers/net/openeth-smoltcp) +
[`nros-smoltcp`](../../drivers/net/nros-smoltcp).

## Source layout

| File | Role |
|------|------|
| `src/lib.rs` | `Esp32QemuPlatform` zero-sized type + trait impls. |
| `src/clock.rs` | `esp_timer_get_time` → ms/us. |
| `src/memory.rs` | esp-alloc heap. `dds-heap` feature flips the budget from 32 KB → 256 KB. |
| `src/random.rs` | ESP32 hardware RNG (also works under QEMU). |

## When to use

- CI smoke tests of bare-metal RISC-V `riscv32imc` builds without real
  ESP32-C3 hardware.
- A reproducible target for verifying the smoltcp bare-metal multicast
  path.

## Caveats

- `riscv32imc` lacks the `A` extension — no native pointer-CAS. Stdlib
  `alloc::sync::Arc` is gated off (`#[cfg(target_has_atomic = "ptr")]`
  evaluates false). Crates that need `Arc` must route through
  [`portable-atomic-util`](https://crates.io/crates/portable-atomic-util)
  — see [Phase 101](../../../docs/roadmap/archived/phase-101-portable-atomic-arc-substitution.md).
- The `portable_atomic_unsafe_assume_single_core` cfg is set on the
  example crates that depend on `spin` / `portable-atomic`.
- DDS-on-ESP32-C3 currently blocked by the above; see
  [Phase 97](../../../docs/roadmap/archived/phase-97-dds-per-platform-examples.md)
  for the deferral note.

## See also

- [Custom Platform porting guide](../../../book/src/porting/custom-platform.md)
- Source on GitHub: <https://github.com/NEWSLabNTU/nano-ros/tree/main/packages/platform/nros-platform-esp32-qemu>
