---
id: 1048
title: "Every `log::info!` on the esp32-qemu board is silently dropped, so four e2e cells grep for a marker the image cannot emit"
status: resolved
area: boards, testing
severity: high
found: 2026-09-04
related: [0968, 1025, 0064]
---

# The entity exists; the line announcing it does not

## Reproduced, with a bracketing probe

`examples/qemu-esp32-baremetal/rust/listener` creates its subscription and then
announces it:

```rust
let _sub = node.create_subscription_for_callback_name::<StringMsg>("on_chatter", "/chatter")?;
log::info!("Subscriber created for topic: /chatter");
```

A `println!` was placed on EACH side of that `log::info!` — via
`nros_board_esp32_qemu::esp_println`, which the board already re-exports, so no
dependency was added. Built with the row's env and run under QEMU with a zenoh
router up:

```
PROBE-A
PROBE-B

Application setup complete — entering spin loop.
```

**Both probes print. The `log::info!` between them prints nothing.** Execution
reaches the site, the macro returns, and the record never reaches a console that
the immediately adjacent `esp_println::println!` reaches.

No `log`-crate output appears anywhere in the boot — no INFO, WARN or DEBUG
line at all.

## Why this matters beyond a missing line

Four of issue 0968's twelve tier-2 failures are esp32 cells, and they fail by
GREPPING FOR THIS MARKER:

```
[xrce/cpp/Pubsub] listener received 0 sample(s), expected ≥1
esp32-qemu did not print `Subscriber created for topic:` within 60s
```

The subscription is there. The test is waiting for a line the image is
structurally unable to emit, so the cell reports a messaging failure for a
logging defect. That is CLAUDE.md's "diff the grep pattern against what the
fixture actually prints" pitfall, with the twist that the pattern is correct and
the PRINTING is broken.

## What is NOT the cause — checked, not assumed

| hypothesis | why it is out |
| --- | --- |
| execution never reaches the line | `PROBE-A` / `PROBE-B` bracket it; both print |
| no logger installed | `esp_println::logger::init_logger(LevelFilter::Info)` runs in `init_hardware`, whose own prints are in the output |
| compile-time level stripping | no `max_level_*` / `release_max_level_*` feature; resolved features are `[]` / `["std"]` |
| two `log` facades, so `set_logger` served the wrong one | there is ONE `log` in the target graph (0.4.33), the same one the example binds and esp-println installs into. (An earlier claim of two was a grep artifact: `log v[0-9.]+` also matches `nros-log v0.5.0`.) |
| esp-println built without `log-04` | the resolved features for this image include `log-04` |

## ROOT CAUSE (2026-09-04): `log::set_logger` does not EXIST on this target

Not a race, not an ordering bug, not a filter. In `log` 0.4.33:

```rust
#[cfg(target_has_atomic = "ptr")]
pub fn set_logger(logger: &'static dyn Log) -> Result<(), SetLoggerError> { … }

#[cfg(target_has_atomic = "ptr")]
pub fn set_max_level(level: LevelFilter) { … }
```

The ESP32-C3 target is `riscv32imc-unknown-none-elf` — RV32 **I M C**, no `A`
extension, so no atomic pointer operations. `rustc --print cfg --target
riscv32imc-unknown-none-elf` lists `target_has_atomic_primitive_alignment="ptr"`
and **no `target_has_atomic="ptr"`**, so both functions are compiled out.

Proved directly rather than inferred: a probe that tried to install its own
logger from the board crate does not compile —

```
error[E0425]: cannot find function `set_logger` in crate `log`
```

**No logger can be installed on this board, by anyone, so every `log::*` record
is dropped.** That is why no INFO line appears anywhere in the boot, why the
example's marker never prints, and why `println!`s bracketing it both do.

### What this retires

* `esp_println::logger::init_logger(LevelFilter::Info)` in
  `nros-board-esp32-qemu/src/node.rs` cannot do anything here. Its comment
  ("without it the `log` crate has no logger installed and silently drops every
  record" — issue #64) describes the state the board is still in.
* The `log::max_level() = Info` reading is NOT evidence a logger installed: the
  getter is unconditional, the setter is not.
* `ESP_LOG=info` at build time changes nothing. Tested.

### The fix, and it is a choice

1. **Route markers through the platform writer** the board already registers
   (`nros_platform_esp32_qemu::register_log_writer` — severity, name, message,
   straight to `esp_println`). It works on this target today: a plain fn
   pointer, no atomics. This is the same shape CLAUDE.md already prescribes for
   Zephyr instead of `std` stdio.
2. **Give `log` its atomics.** `portable-atomic` is already in this image's
   graph, but `log` gates on `target_has_atomic` directly with no
   portable-atomic feature, so this means patching `log` — not worth it for a
   marker.

(1) fits the tree. Note the scope: this is not only about test markers. EVERY
`log::info!`/`warn!`/`error!` in any crate this board links is silently
discarded, nros framework diagnostics included, so an image that fails here can
currently only report through `println!` or the platform writer.

### Not esp32-specific in principle

Any `no_std` target without `target_has_atomic = "ptr"` has this property. The
esp32-c3 board is just the one in this tree that does.

## Not to be confused with

`test_esp32_workspace_entry_e2e`, the fourth esp32 cell, fails BEFORE
`Application setup complete` and so is not this. The other two
(`test_esp32_to_native`, `test_native_to_esp32`) grep for markers of the same
kind and plausibly are, but that is unverified.


## FIXED 2026-09-04 — and it took TWO changes, not one

`[INFO] nros: Subscriber created for topic: /chatter` prints. It did not before,
and neither did any other log record on this board.

Both of these had to change; either alone leaves the console silent:

1. **The `log` facade is unusable here** (the root cause above). Left in place
   for boards that have atomics; the markers moved off it.
2. **`nros_log` was no better off.** Nothing ever called `nros_log::init`, so
   `dispatch_to_sinks` found a null sink list and HELD every record in the early
   buffer for a board that never spoke. The board now installs
   `nros_platform_cffi::log::PlatformSink` — where issue 0710 moved it from
   `nros_log::sinks` — which speaks the platform ABI to the fn-pointer writer
   `register_log_writer` already installed.

The second one nearly shipped as a non-fix: routing the markers to `nros_log`
alone moved them from one silent path to another, and the build looked fine. What
caught it was `nm` on the ELF — `nros_platform_log_write` absent, so the sink was
not compiled in at all.

The three markers now emit through `nros_log::nros_info!`, reached via
`nros_board_esp32_qemu::nros_log` rather than a new dependency on each leaf,
because examples are standalone copy-out projects (RFC-0026) and already depend
on the board.

### Confirmed by the harness, not just by hand

`test_esp32_talker_listener_e2e` previously died waiting for the LISTENER's
`Subscriber created for topic:`. It now gets past that wait and fails later, on
the TALKER's `Publishing:`. The listener half of the marker problem is closed.

### What this exposed: the talker does not finish setup

Run alone with a router up, the talker boots, brings up its network, prints
`Ethernet ready.` — and stops. No `Application setup complete`, no `Publishing:`.
The listener, same board and same build, gets all the way through.

That is a SEPARATE defect from this issue, and it was invisible while the log
silence hid where the image stopped. Filed as
[issue 1052](1052-esp32-talker-faults-after-network-bringup.md): an
`Instruction access fault` whose `mepc` and `ra` are printable ASCII from
source-path strings, i.e. a corrupted code pointer. It is what remains of
`test_esp32_talker_listener_e2e` and `test_esp32_to_native`.


## RESOLVED 2026-09-06 — and the defect it exposed is resolved too

The fix (the `nros_log` sink installed on esp32, and the markers routed through
it) landed 2026-09-04. What kept this issue open was the section above: the
silence had hidden a SECOND defect, the talker stopping after `Ethernet ready.`,
filed as issue 1052 and described here as "what remains of
`test_esp32_talker_listener_e2e` and `test_esp32_to_native`".

Both are answered now. Issue 1052 was a stack overflow — `.stack` is the linker
leftover after `.bss` on this board, and `.bss` had squeezed it to 18,572 B — and
in nightly run `33996315057` the esp32 job reports:

| test | verdict |
| --- | --- |
| `test_esp32_qemu_talker_boots` | **PASS** |
| `test_esp32_talker_listener_e2e` | **PASS** |

`test_esp32_to_native` fails for an unrelated reason that is not the board: the
esp32 lane never built the NATIVE half of that test (issue 1112).

The pairing is worth stating because it is the argument for this issue's
severity. A silently dropped log made a crash look like a hang, and the crash was
found only after the logging was fixed — the diagnostic was load-bearing for
diagnosing something else.
