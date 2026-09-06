---
id: 1028
title: "NuttX is classified `hosted` because its `target_os` is not `\"none\"`, so it
  takes the Linux 32-queryable budget: 142,336 B of `.bss` in an image with zero
  queryables"
status: resolved
type: bug
area: [rmw, memory, build]
related: [0827, 0870, 0460, 1061, 1142]
---

## Resolution (2026-09-06) — predicate fixed 2026-09-04, class swept and gated now

Two commits, and the second exists because the first was one site.

**The reported site was fixed by `7c5e52845` (2026-09-04).** `hosted` is no
longer `CARGO_CFG_TARGET_OS != "none"`: `runner.rs` carries an explicit
`target_os_is_hosted()` over an `RTOS_TARGET_OS` list, with three unit tests
(an RTOS that names itself is not hosted; bare metal and absent stay unhosted;
real hosts stay hosted). That commit MEASURED the saving on the image this
issue names — `SERVICE_BUFFERS` 142,336 B → 35,584 B, **106,752 B of `.bss`
recovered**, matching `ZPICO_MAX_SESSIONS * ZPICO_MAX_QUERYABLES * 4,448 B`
exactly — and verified the host build still reads 32, so the arm the predicate
exists to serve was not traded away. The issue file was simply never moved.

**The class was not fixed, and it had a live second site.** The same question
is also asked as a `cfg` predicate, where a function cannot be called, so the
answer gets re-spelled. `nros-macros` already spelled it correctly
(`not(any(target_os = "none", target_os = "nuttx"))`, five sites);
`nros-rmw-zenoh`'s `effective_client_locator` spelled it
`not(target_os = "none"))` — in a function whose own doc-comment says the
default is "deliberately HOSTED-only" and that on an embedded image dialling
`tcp/127.0.0.1:7447` is "strictly worse than the established no-locator
behaviour". So a NuttX image took the host default. **Latent, not live**: every
`examples/qemu-arm-nuttx` entry supplies an explicit locator, so that arm is
not reached in-tree today. Fixed to the `any(...)` spelling.

The sweep found no third. All 25 `cfg` predicates naming `target_os = "none"`
were read and classified: nine ask a genuinely different question and say so in
their own prose — `critical_section` needs interrupt masking
(`nros-rmw-cyclonedds/src/sync.rs`, whose header already states "Other (e.g.
RTOS targets compiled with their own `target_os`): falls back to
`spin::Mutex`"), the bare-metal boot link section and `OwnedSpin` entry shape
(`main_macro.rs`), `Box` availability (`rust_adapter.rs`), the absent-libc stubs
(`libc_stubs.rs`), and `nros-rmw-cffi`'s registration ctor, which over-includes
the RTOSes ON PURPOSE and documents why (the board also calls `register()`, and
`register()` is idempotent).

**Gate: `check-rtos-target-os`** (`scripts/check/check-rtos-target-os.py`, fast
lane), two rules, both mutation-tested red:

1. Every Rust target triple this tree names resolves to a `target_os` that is
   `"none"`, a known host, or in `RTOS_TARGET_OS` — so a second NuttX arch or a
   future `*-espidf` board cannot land and silently take the hosted budget
   again. (Removing `"nuttx"` from the list → 2 failures.)
2. Every `cfg` predicate mentioning `target_os = "none"` either names each
   REACHABLE RTOS beside it, or is classified in the script with the other
   question it asks. (Reverting `effective_client_locator` → 2 failures.) The
   classification table is a ratchet, and a row whose site disappears fails as
   a stale exemption (the issue-0743 class).

**Three prose claims were false and are corrected**: `nros-rmw-zenoh/src/lib.rs`
said the registration ctor is skipped on "NuttX, Zephyr, ESP-IDF, bare-metal"
via `target_os = "none"` (NuttX gets the ctor); `runner.rs` said the embedded
RTOSes "all share `target_os = \"none\"`" (NuttX does not — the inference below
it is safe only because it is an allowlist of hosts, not a `!= "none"` test);
and `shim_config_from_env` still described the budget split as
`target_os = "none"`.

**Measured vs inferred, for this follow-up.** Nothing here was re-measured on an
image: the 106,752 B is `7c5e52845`'s measurement, not a new one. This change
is measurement-neutral by construction — it alters a `cfg` arm that no in-tree
NuttX build reaches, three comments, and adds a gate. What WAS run: the gate
(green, then red under both mutations, then green again) and
`nros-zpico-build`'s unit tests.

**Not done, deliberately** — the deeper fix phase-392 W5 records as Open. A
standalone copy-out example has no SystemModel, is not Zephyr, and is not a
cargo leaf, so none of the three declaration channels reaches it and the
fallback is the only budget it can get. That needs a user-facing decision about
how a copy-out template declares its entities, and its acceptance is a NuttX
BUILD rather than a gate. Filed as **issue 1142**.

## What

`nros-zpico-build/src/runner.rs:246` picks the queryable budget for an image
that declares nothing:

```rust
None => return if hosted { 32 } else { UNDECLARED_HEADROOM },
```

`hosted` is `CARGO_CFG_TARGET_OS != "none"` (`runner.rs:96`). NuttX's Rust
target is `armv7a-nuttx-eabihf`, and:

```
$ rustc --print cfg --target armv7a-nuttx-eabihf | grep target_
target_family="unix"
target_os="nuttx"
```

`"nuttx" != "none"`, so an RTOS takes the 32-slot guess written for Linux.

**"Is this hosted?" and "is `target_os` set?" are different questions**, and
NuttX is the counterexample: it is POSIX-ish enough to name itself, and it is
still an RTOS on a part with a fixed RAM budget.

## Measured

`examples/qemu-arm-nuttx/cpp/action-client`, an image that opens **zero**
queryables (an action client declares three service *clients* and one feedback
subscription — no queryable at all):

```
$ nm -S .../build-zenoh/cpp_action_client | grep SERVICE_BUFFERS
40202c88 00022c00 b ..._14nros_rmw_zenoh4shim7service15SERVICE_BUFFERS
```

`0x22c00` = **142,336 bytes** of `.bss` — 32 slots x 4,448 B. The same table at
the embedded budget (8) would be 35,584 B. **106,752 B wasted.** Byte-identical
in the C image.

The array is `SERVICE_BUFFERS` (`shim/service.rs:108`), sized
`ZPICO_MAX_SESSIONS * ZPICO_MAX_QUERYABLES` at `:107`.

## Why it has not bitten

`nros-board-nuttx-qemu` has `CONFIG_RAM_SIZE=132120576` (126 MB), so on QEMU
this is waste rather than failure. It is not waste on a real NuttX part, and
the budget is decided at build time by a predicate that does not know which it
is running on.

## Not the cause of #0870

This was found while investigating #0870 (NuttX C++ action client fails
`create_action_client`) and is **not** its cause — the pools are large enough,
and more importantly pool exhaustion cannot produce that issue's error code at
all. Recorded separately so it does not stay buried inside a killed lead.

## Fix direction

Do not widen `hosted`; it is used elsewhere for genuinely POSIX questions.
Narrow this one call site: an RTOS `target_os` (`nuttx`, and check `espidf`,
`horizon`, and anything else in-tree) takes `UNDECLARED_HEADROOM` regardless of
whether it names itself.

The deeper fix is the one phase-392 W5 already names as Open: **every image
declares its entities**, and the fallback budget stops mattering. Zephyr
already derives per-image from the entity inventory
(`zephyr/Kconfig:694` default `-1` -> `nros_cargo_build.cmake:406`); NuttX
never got that treatment.

## Confirm cheaply

`nm -S <elf> | grep SERVICE_BUFFERS` before and after. No run needed.
