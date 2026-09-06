---
id: 1142
title: "A standalone (non-workspace) example declares no entities, so the guessed
  fallback budget is the only budget it can ever get — NuttX has no channel at all"
status: open
type: tech-debt
area: [rmw, memory, build]
related: [1028, 0827, 1061, 0460]
---

## What

phase-392 W5's end state is that **every image declares its entities and the
fallback budget stops mattering**. Three delivery channels exist today and a
NuttX standalone example is in the gap between all three:

| channel | who it serves | mechanism |
| --- | --- | --- |
| CMake workspace | a workspace entry with a resolved SystemModel | `nros ws entity-facts` → `cmake/NanoRosEntityFacts.cmake` → `corrosion_set_env_vars` |
| Zephyr | any Zephyr image | `zephyr/Kconfig` `CONFIG_NROS_MAX_QUERYABLES` default `-1` (the DERIVE sentinel) → `nros_cargo_build.cmake` |
| cargo leaf | a probeable cargo leaf, or one that declares (issue 1061) | `nros sync` → `<leaf>/metadata/<component>.json` → `leaf_entity_env.rs` → the leaf's `[env]` sidecar |

`examples/qemu-arm-nuttx/cpp/action-client` is none of them. It is a standalone
copy-out CMake project (`find_package(nano_ros)` + `nano_ros_add_executable`),
so:

* it has no bringup and no SystemModel, so `nros_record_entity_facts` returns
  early — by design, and `nros_entity_facts_env` then says nothing rather than
  warning, because "a configure whose models are not resolved yet has always
  sized the table from the backend's own default";
* it is not Zephyr, so the Kconfig sentinel does not apply;
* it is not a cargo leaf, so the `[env]` sidecar does not apply.

So `NROS_DECLARED_SERVICE_SERVERS` and `NROS_DECLARED_INFRA_QUERYABLES` are
both absent, and `queryable_default_from` takes its last arm:

```rust
None => return if hosted { 32 } else { UNDECLARED_HEADROOM },
```

which is a guess by construction. Issue **1028** is what that guess costs when
the predicate feeding it is also wrong: 142,336 B of `.bss` on an image with
ZERO queryables. 1028 fixed the predicate. It did not — and could not — remove
the guess.

## Why this is its own wave

The declaration has to come from somewhere, and for this image shape there is
no "somewhere" yet:

* The **CMake** answer would be `nano_ros_node_register(... ENTITIES ...)` on a
  standalone target, which today only workspace members use. Making a
  copy-out example carry it is a user-facing API decision (RFC-0026 says these
  are standalone templates a user copies), not an internal plumbing change.
* The **cargo** answer for a NuttX leaf is issue 1061's
  `[package.metadata.nros.component] entities` — NuttX leaves are exactly the
  `*.json.unprobeable` case that mechanism exists for (a foreign `[build]
  target` with `[unstable] build-std` cannot be host-compiled by the probe).
  None of the six `examples/qemu-arm-nuttx/rust/*` leaves declares one today,
  so the Rust half of the platform is in the same gap for a different reason.
* Whichever is chosen, **the acceptance is a BUILD** — a measured
  `nm -S <elf> | grep SERVICE_BUFFERS` before and after — not a gate. That
  needs the NuttX toolchain plus a ROS install for `example_interfaces`, which
  is why it did not ride along with 1028.

## Confirm cheaply

For any candidate image, the configure line is the tell. A configure that has a
declaration prints:

```
nano-ros: queryable table sized from the declaration — infrastructure …, … (phase-392 W5)
```

Absence of that line means the fallback decided the budget.

## Related

* **1028** — the fallback's hosted/embedded predicate was wrong for NuttX
  (fixed; `target_os_is_hosted` + `check-rtos-target-os`).
* **0827** — deriving a cargo leaf's pool budgets from the probe.
* **1061** — a leaf DECLARES what the probe cannot read.
* **0460** — the failure direction: an 8-slot table against eleven
  infrastructure queryables, discovered at boot.
