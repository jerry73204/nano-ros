> **phase-321 W2.b — scope of this directory.**
> `qemu-smoltcp-bridge` moved to `packages/testing/`: it is test support the
> qemu-baremetal lane builds, not reference material. (It has had a
> `fixtures.toml` row since then; `fixture-inventory.py` never built anything —
> it is a read-only diagnostic, and phase-350 W0 deleted the stale row where it
> claimed this leaf was outside the manifest.) What remains here is `stm32f4-porting/{polling,rtic}` — **porting
> templates that NOTHING builds**. They are not workspace members and
> `just/native.just` only runs `size` on a prebuilt binary `|| echo "build
> failed"`, so they can rot silently. Treat them as documentation-with-code,
> and expect them to lag the APIs they demonstrate.

# Reference Implementations

Low-level reference implementations for BSP developers. These are libraries (not standalone examples) used by the board support crates.

**Most users should use the examples instead:** see [examples/](../../examples/README.md).

## Contents

| Directory | Description |
|-----------|-------------|
| `qemu-smoltcp-bridge` | Shared library that bridges smoltcp sockets to zenoh-pico. Provides the C FFI required by zenoh-pico's smoltcp platform backend. |
| `stm32f4-porting` | STM32F4 porting references (polling loop + RTIC) using internal platform crates. Templates for BSP developers. |

## See Also

- examples/qemu-arm-baremetal/rust/standalone/lan9118/ - LAN9118 driver validation
- examples/stm32f4/rust/standalone/smoltcp/ - smoltcp TCP echo server
- examples/stm32f4/rust/talker-embassy/ - Embassy async
