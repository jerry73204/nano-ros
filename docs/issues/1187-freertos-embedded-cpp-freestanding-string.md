---
id: 1187
title: "Every embedded C++ FreeRTOS image fails to compile: `<string>` reaches
  `requires_hosted.h` under `-ffreestanding` on the PINNED arm-none-eabi-gcc,
  and the `__has_include` gate cannot see it"
status: open
type: bug
area: cpp
related: [0112, 0332, 1146]
---

## Problem

Nothing under `examples/qemu-arm-freertos/cpp/` or any FreeRTOS C++ workspace
entry compiles at `bd204b986` with the toolchain `nros-sdk-index.toml` pins
(`arm-none-eabi-gcc 13.2-nros4`). Measured, twice:

```
bash scripts/build/fixtures-build.sh freertos cpp zenoh      # 6 role examples
bash scripts/build/workspace-fixtures-build.sh freertos c    # + cpp, mixed
```

both die with 488 diagnostics whose first is

```
.../13.2.1/bits/requires_hosted.h:34:4: error: #error "This header is not
    available in freestanding mode."
In file included from .../13.2.1/string:38,
                 from packages/api/nros-cpp/include/nros/log.hpp:115,
                 from .../nros/qos.hpp:24, .../nros/node.hpp:30,
                 .../nros/action_server.hpp:605, .../nros/action_client.hpp:26,
                 .../nros/component.hpp:37,
                 from <entry>_nros_main_generated.cpp:11
```

Everything after it is fallout — `'allocator' does not name a type`,
`'class std::allocator' has no member named 'deallocate'`, and a
`spin_until_future_complete` instantiated with `const int&` because
`Node::SharedPtr` never formed.

## Why the existing gate does not catch it

`log.hpp` (and its siblings) gate the `<string>` / `<sstream>` include on

```c++
#elif defined(__has_include)
#if __has_include(<string>)
```

with the rationale, from issue 0112, that `__has_include` is the right question
and `__STDC_HOSTED__` is not. That was correct for the compiler it was written
against. **GCC 13 broke the equivalence**: `<string>` is PRESENT on
arm-none-eabi — `__has_include` is true — and including it under
`-ffreestanding` is a hard `#error`, because libstdc++ added
`bits/requires_hosted.h`. Presence stopped implying usability, so the gate now
answers a question nobody asked.

`NROS_CPP_STD` is the reliable arm and it is not defined on this lane.

## Why it stayed invisible

The embedded C++ FreeRTOS build runs in no merge-gating lane — `check-cpp` is a
HOST lane, and the cross build is reached only by `build-all` /
`workspace-fixtures-build.sh`, i.e. `schedule` and `workflow_dispatch`. A lane
that is uniformly red carries no signal (CLAUDE.md's "a red CI lane answers one
of two questions"), so this reads exactly like yesterday's failure.

## What it blocks

Issue 1146 could measure the FreeRTOS app-task stack on the Rust path (8
images) and on the C path (6 images) and NOT on the C++ path, so
`cmake/templates/freertos_app_config.c.in`'s 512 KiB `.app_stack_bytes` — one
number serving both C and C++ — could not be reduced to the measured figure.

## Fix direction

Make the freestanding gate ask the question it means. Either

- require `NROS_CPP_STD` for the hosted-header arms on a cross target (the
  `__has_include` arm becomes a hosted-only convenience), or
- add `&& !defined(__STDC_HOSTED__) == 0`-style pairing, i.e. keep
  `__has_include` but AND it with a usability probe (`__has_include
  (<bits/requires_hosted.h>)` is not one — the file is present and correct;
  `__STDC_HOSTED__` is).

Whichever, do it for the CLASS. `log.hpp` is one of **13 headers under
`packages/api/nros-cpp/include/nros/` carrying the same two-arm idiom** (19
`__has_include(<...>)` sites: fixed_string, heap_string, options ×2, qos,
publisher, client, declared_qos, log ×2, nros ×3, service,
polling_subscription, timer ×2, subscription ×2), and the 0112 → 0332 history
is this same gate being fixed at one site at a time. One shared spelling, not a
14th.

Acceptance is a BUILD: `bash scripts/build/fixtures-build.sh freertos cpp
zenoh` green, with `NROS_CMAKE_EXTRA_DEFS` carrying
`cmake/toolchain/arm-freertos-armcm3.cmake`.
