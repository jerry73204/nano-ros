---
id: 1143
title: "The FreeRTOS zenoh build compiles lwIP unconditionally, so a mailbox-only board in that family links 80 KB of network stack it cannot use"
status: resolved
type: bug
area: rmw, platform
severity: medium
found: 2026-09-06
resolved: 2026-09-06
related: [0541, 1122, phase-418]
---

# `supported_netstacks` documents a fact it does not cause

`packages/platform/nros-platform-freertos/nros-platform.toml` pulls lwIP at
PLATFORM level, with no branch on the board:

```toml
[build.zenoh]
defines = ["ZENOH_GENERIC", "ZENOH_FREERTOS_LWIP"]
include = ["system/common", "system/freertos/lwip"]
extra_sources = [
  { path = "{src}/system/freertos/system.c" },
  { path = "{src}/system/freertos/lwip/network.c" },
]
```

Every zenoh-on-FreeRTOS image compiles those TUs whatever its board says.
`nros-board-mps2-an385-freertos` declares `supported_netstacks = ["lwip"]` and
`nros-board-freertos-posix` declares `[]`, and the build treats them the same.

Measured on a publish-only C++ image (`orin-spe-heartbeat`, board
`mps2-an385-freertos`): **80,846 bytes** of bss in lwIP + LAN9118 symbols
(`memp_*`, `ram_heap`, `dns_table`, the netif driver).

## Why it matters now

The NVIDIA Orin SPE has no Ethernet at all — its only link to the CCPLEX is a
Tegra IVC mailbox. Against phase-418's measured budget of ~116,736 bytes for
everything above NVIDIA's own FreeRTOS demo, 80,846 is **69 % of the budget**
spent on a stack the hardware cannot reach.

## The failure is already recorded

`examples/workspaces/cpp`'s `[image.freertos_posix]` selects **cyclonedds, not
zenoh**, and says why:

> zenoh-pico's FreeRTOS backend is lwIP-only
> (`system/freertos/lwip/network.c`), and this board has no lwIP — so a zenoh
> build of it compiles that TU and dies at link with 63
> `undefined reference to lwip_*`.

So the experiment has been run. A netstack-less FreeRTOS board cannot currently
build zenoh at all; it can only avoid the question by choosing another RMW.

## What the tree already has

- `LinkPolicy::ivc_only()` — written, correct (`tcp`/`udp_*`/`serial`/`raweth`/
  `tls` all `Force(false)`, `ivc`/`custom` `Follow`), and `#[allow(dead_code)]`.
  Its own comment states the gap: *"`nros-zpico-build` picks a policy per
  PLATFORM, and IVC-only has no in-tree platform since the SPE board left."*
- `link-ivc` → `Z_FEATURE_LINK_IVC` (`nros-zpico-build/src/lib.rs:346`, from
  `link.ivc_flag()`), deliberately board-agnostic.
- The `zpico-link-ivc` forwarder crate, and the fork's vendored
  `src/link/unicast/ivc.c`.

Nothing is missing at the link layer. What is missing is that no board can
SELECT any of it.

## The hard part is the socket ABI, not the deletion

Phase-154's comment in that same `[build.zenoh]` block explains why the vendor
lwIP TUs are compiled at all:

> vendor impls provide `_z_sys_net_socket_t = { int _socket; }`, which
> `src/link/unicast/tcp.c` expects; the alias TU's 32-byte opaque socket would
> mismatch, so `build.rs` skips the alias TU entirely on FreeRTOS.

So removing `network.c` is not a subtraction — it is a choice about which
`system/` TU set provides the socket type, and getting it wrong is the layout
mismatch phase-160.C describes on the Zephyr side: two TUs agreeing about a
struct's NAME and not its SIZE, which links and then corrupts.

## Shape of a fix

1. `[build.zenoh]` becomes netstack-conditional, so a board with
   `supported_netstacks = []` selects an IVC shape rather than the lwIP one.
2. Link-policy selection moves from per-PLATFORM to per-BOARD, which is what
   `ivc_only()` has been waiting for.
3. The IVC path's `system/` TU set is settled, with the socket type stated
   rather than inherited.

(1) and (2) are reachable now. (3) wants a board to test against, which is
phase-418's 418.6 — and a fix that lands (3) untested would be the 0135 class
by construction.

---

## What was established (2026-09-06)

> The title as filed named `supported_netstacks = []` as the predicate. It is
> not one — see the next section — so the title has been corrected to name the
> board shape rather than the field. The 80 KB and the class are unchanged.

### `supported_netstacks = []` is NOT the predicate — that is the finding

The plan above says "a board with `supported_netstacks = []` selects an IVC
shape". Measured against the tree, that rule would break four working boards.
Every in-tree board that declares the empty list has working sockets, and they
declare it for two different reasons, neither of which is "there is no link":

| board | `supported_netstacks` | why |
| --- | --- | --- |
| `linux` | `[]` | the HOST kernel owns the stack |
| `freertos-posix` | `[]` | the HOST kernel owns the stack |
| `nuttx-qemu-arm` / `nuttx-qemu-riscv` | `[]` | the RTOS owns the stack |
| `zephyr`, `fvp-aemv8r-smp` | `[]` | the RTOS owns the stack |

The list answers **which in-image stack this board can be built with**. Empty
means "not ours to select", never "absent". Reading it as "no links" forces
`Force(false)` onto TCP for every native and NuttX zenoh build — a re-policy of
working boards dressed as a fix.

The fact that *does* decide it already exists: **RFC-0086's
`capabilities.ip_stack`**. `nros-platform-freertos` declares `ip_stack = true`
(lwIP and FreeRTOS-Plus-TCP are both named in its `[build]` block), so a
mailbox-only board in the same family needs a rung to say otherwise — and
RFC-0049's ladder already orders board above platform. It only had no board
rung.

### Landed: the board rung, and the schema that spends it

* **`PlatformsTree::capabilities_with_board`** (`nros-platform-config`) — the
  RFC-0086 map with the board's `[board.capabilities]` merged over the
  platform's, board winning per key. Absent stays absent, so every build with
  no board rung is unchanged.
  * The rung is `[board.capabilities]`, **not** a top-level `[capabilities]` in
    `nros-board.toml`. `BoardKnobsFile` has parsed a top-level one since
    phase-400 W6 and it looked like the obvious home — but the CLI's
    `BoardFile` carries `deny_unknown_fields` and knows only `board`, so a
    descriptor with a top-level table stops loading for every consumer.
    `BoardCapabilities` on the CLI side does not deny unknown fields, so one
    table serves the RFC-0042 D2 reader (`heap`/`atomics`/`threads`) and the
    RFC-0086 one without either growing a field.
  * `NROS_BOARD` selects the entry when a file declares several
    (`nros-board-nuttx` declares two that differ in ISA). Several entries
    disagreeing with nothing naming one is REFUSED, not resolved by file order.
* **`LinkPolicy::for_board`** (`nros-board-common/src/policy.rs`) — the board
  rung applied over the per-platform baseline. `ip_stack = false` ⇒
  `ivc_only()`. Its `#[allow(dead_code)]` is gone.
* **Schema (`nros-platform-config/src/manifest.rs`)** — the manifest could not
  express this and now can:
  * `WhenMatcher.capability` — a `{ name = bool }` table compared against the
    resolved ladder. Absent matches NEITHER value.
  * `PlatformEntry.defines_conditional` — the `X` / `X_conditional` pair the
    schema already uses for `include_paths`. A conditional DEFINE is what
    picks the zenoh-pico platform header, and there was no way to write one.
  * `ExtraSource.when` and `RequiredEnv.when` — so the netstack TU and the SDK
    it needs move together with the define.

### The socket ABI: answered from the fork, still unbuilt

The vendored zenoh-pico fork **already carries the IVC-only shape**, which the
issue's step (3) did not know:

* `include/zenoh-pico/system/platform/freertos/orin_spe.h` — selected by
  `ZENOH_ORIN_SPE` in `system/common/platform.h`'s dispatch chain. FreeRTOS
  tasks/mutex/condvar exactly as `lwip.h`, and `_z_sys_net_socket_t` /
  `_z_sys_net_endpoint_t` degenerate to a `uint8_t _placeholder` union.
* `src/system/freertos/system.c` already has `ZENOH_ORIN_SPE` arms (three of
  them) beside its `ZENOH_FREERTOS_LWIP` ones.
* There is **no** `src/system/freertos/orin_spe/network.c`, and none is needed:
  with every socket link feature 0, no socket TU is referenced. `ivc.c` reaches
  the link through `_z_ivc_socket_t` in the `_z_link_t` union.

So the answer to "which `system/` TU set provides the socket type" is: the
`ZENOH_ORIN_SPE` header plus `system/freertos/system.c`, and no network TU.
`lwip.h` cannot be reused for it under any circumstances — it `#include`s
`lwip/sockets.h` at line 21, so an lwIP-less board cannot even PARSE it,
which is why the define has to branch rather than just the source list.

**Still unverified: nothing in-tree has compiled `ZENOH_ORIN_SPE` since the SPE
board left.** The header and the arms are read, not built. Step (3) stays open
and still wants phase-418's board.

### The `[build.zenoh]` block (APPLIED)

Applied to `packages/platform/nros-platform-freertos/nros-platform.toml`, plus
`ZENOH_ORIN_SPE` added to `check-zenoh-platform-macros.py`'s `PLATFORM_MACROS`
and to `OWN_MACRO["freertos"]` — the freertos port now legitimately claims both
FreeRTOS arms, and which one an image gets is the board fact
`check-capability-conditionals` checks.

**No in-tree board declares `ip_stack = false`, so the `ZENOH_ORIN_SPE` arm is
selectable and unexercised.** Every existing FreeRTOS build resolves the same
`true` arm it had before, byte-identically; the `false` arm is proven only
statically, by rules 4/7/8 reading the vendored header. A build of it waits on
phase-418's SPE board.


Only the differences from today's block are shown; `defines_kv`, `include`,
`arch`, `compile` and `rerun_if_env_changed` are unchanged.

```toml
defines = ["ZENOH_GENERIC"]
# issue 1143 — the zenoh-pico PLATFORM header is a BOARD fact, not a platform
# one. `system/common/platform.h` dispatches on exactly one `ZENOH_*` macro and
# the two arms pick incompatible socket types: `freertos/lwip.h` is
# `{ int _socket; }` and `#include`s `lwip/sockets.h`; `freertos/orin_spe.h` is
# a one-byte placeholder and includes no netstack at all. `ZENOH_GENERIC` stays
# unconditional — it selects the generated `zenoh_generic_config.h`, and the
# dispatch chain tests both FreeRTOS arms BEFORE it, so it never competes for
# the type header.
defines_conditional = [
  { name = "ZENOH_FREERTOS_LWIP", when = { capability = { ip_stack = true } } },
  { name = "ZENOH_ORIN_SPE", when = { capability = { ip_stack = false } } },
]
extra_sources = [
  { path = "{src}/system/freertos/system.c" },
  { path = "{src}/system/freertos/lwip/network.c", when = { capability = { ip_stack = true } } },
  { path = "{nros}/c/zpico/zpico.c" },
]
required_env = [
  { name = "FREERTOS_DIR", help = "…", validate_subdir = "include" },
  { name = "FREERTOS_PORT", help = "…" },
  { name = "FREERTOS_CONFIG_DIR", help = "…" },
  # gated: a board with no netstack must not have to invent a directory
  { name = "LWIP_DIR", help = "…", validate_subdir = "src/include", when = { capability = { ip_stack = true } } },
]
include_paths = [ …, ]   # the two {env:LWIP_DIR} entries move out of here
include_paths_conditional = [
  { path = "{env:LWIP_DIR}/src/include", when = { capability = { ip_stack = true } } },
  { path = "{env:LWIP_DIR}/contrib/ports/freertos/include", when = { capability = { ip_stack = true } } },
]
```

`include = ["system/common", "system/freertos/lwip"]` stays as it is and is
**not** a source list: `build_zenoh_pico_unified` selects sources from
`add_zenoh_pico_core_sources`'s hardcoded subdirs plus `extra_sources`, and
`include` is consumed only by the 136.6 drift gate, which asserts the directory
exists in the submodule. Worth knowing before reading the block as if deleting
a row there would drop a TU.

### `check-capability-conditionals` (`just check capability-conditionals`)

Eight rules. Rule 4 reads the socket type out of zenoh-pico's own headers
rather than a table, so a swap of the two `when` values — shape-valid, parses,
links, and hands `_z_open_tcp` a one-byte struct — is rejected in both
directions. Rule 6 checks the wire the issue was actually about: something has
to CALL the board rung, because `ivc_only()` being written and correct was
never the problem.

**Rules 7 and 8 were added on review, because rule 4 covered ONE field of
four.** Rule 4 reads the header a DEFINE selects, which makes it blind to every
other row that has to move with that define. Three shape-valid mutations of the
applied block went undetected by the six-rule gate:

| mutation | what it would have done |
| --- | --- |
| `extra_sources` `lwip/network.c` gated `ip_stack = false` | compiles lwIP's netstack TU into exactly the image whose header declares a placeholder socket — issue 0135's split, arrived at from a field rule 4 does not read |
| `required_env` `LWIP_DIR` gated `ip_stack = false` | a mailbox-only board made to produce an lwIP checkout it has no use for; the lwIP board silently loses its validation |
| `include_paths_conditional` `{env:LWIP_DIR}/src/include` gated `ip_stack = false` | the netstack TU compiled without the header roots it needs |

So the arm is now read once, from the defines — whichever arm resolves to a
REAL `_z_sys_net_socket_t` is the NETSTACK arm — and followed into the other
fields. **Rule 7:** a vendor `system/<rtos>/<stack>/network.c` row must carry
that arm's condition, and must not be unconditional (which is this issue as
filed, and is now what the gate reports for it). **Rule 8:** every `{env:NAME}`
that only the netstack rows name must be gated the same way in `required_env`,
`include_paths` and `include_paths_conditional` alike; a variable no
conditional row names (`FREERTOS_DIR`) says nothing about the netstack and is
left alone. All four mutations above are rejected, and all four are self-test
cases.

### What this does NOT fix

**`examples/workspaces/cpp` `[image.freertos_posix]` still cannot build zenoh,
and this change does not move it.** `freertos-posix` has HOST sockets — it
inherits the platform's `ip_stack = true`, takes the lwIP arm, and reproduces
today's 63 undefined `lwip_*`. Making it work needs a `system/` TU set pairing
FreeRTOS types with host BSD sockets, which zenoh-pico does not ship; that is
new vendored code and a separate piece of work. The board's `[]` is not an
error — its stack really is the host's.

Also unresolved: `s32z270` and `mps3_an536` declare `supported_netstacks =
["lwip"]` and still select cyclonedds in that same file, so the RMW choice
there has a second cause nobody has written down.
