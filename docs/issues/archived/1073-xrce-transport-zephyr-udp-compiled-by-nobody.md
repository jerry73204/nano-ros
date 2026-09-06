---
id: 1073
title: "`transport_zephyr_udp.c` is compiled by neither lane — superseded by `transport_nros_udp.c` in phase 129.C.1 and never deleted"
status: resolved
type: bug
area: rmw
severity: low
found: 2026-09-05
related: [1068, 1069]
---

# A translation unit nothing builds

`packages/rmw/xrce/nros-rmw-xrce/src/transport_zephyr_udp.c` is compiled by
neither the cargo lane nor the CMake one. Phase 129.C.1 superseded it with
`transport_nros_udp.c`, which reaches the platform's UDP through
`nros_platform_udp_*` and therefore works on every target; `build.rs` still
carries the `feat_zephyr = false` that switched the old one off, and `git grep`
finds no other reference to it.

Found while building issue 1068's source manifest: the new
`check-xrce-source-manifest` asserts that every backend `.c` is compiled by at
least one lane, and this is the file that made that check fail on its first run.

## Why it is filed rather than deleted

Deleting it was out of scope for 1068, which was a refactor plus one named
behaviour fix. It sits on the gate's documented `NOT_COMPILED` list, so
"nothing builds it" is now a **recorded fact rather than a silence** — which is
the state that matters. Removing the file is the follow-up.

## Before deleting, check

Whether anything outside this repository names `xrce_zephyr_udp_init`. The
symbol was a public entry point for one release, and phase 129.C.1's note says
the posix pair was "kept alongside `transport_nros_udp` for one cycle so callers
that still resolve `xrce_posix_udp_init` keep working" — the same courtesy may
have been intended here. If nothing does, delete the file and its
`NOT_COMPILED` row together; the gate refuses a row naming a file that no longer
exists, so the cleanup cannot be half-done.

## Resolution (2026-09-05) — DELETED

`packages/rmw/xrce/nros-rmw-xrce/src/transport_zephyr_udp.c` is gone, together
with its `xrce_zephyr_udp_init` declaration in `src/internal.h` and the
`NOT_COMPILED` row in `scripts/check-xrce-source-manifest.py`.

**Dead code, and more dead than the issue said.** The issue records that no lane
compiles it. Two further facts settle the decision:

1. **It could not have contributed a symbol even if a lane had compiled it.**
   The file's entire body sits behind `#if defined(UCLIENT_PLATFORM_ZEPHYR)`,
   and `xrce-config.txt` states `flag uxr UCLIENT_PLATFORM_ZEPHYR never` — for
   BOTH lanes, since phase-420 W9. So "make the lane that needs it compile it"
   was never an available fix: it would have produced an empty translation unit
   unless someone also flipped a toggle 129.C.1 deliberately set to `never`,
   which selects upstream's Zephyr *platform*, not our TU.
2. **The "one cycle of fallback" courtesy was never delivered for this half.**
   129.C.1 (`71ef6a4b5`) deleted the `platform-zephyr` cargo feature, and with
   it the `backend_tus.push("transport_zephyr_udp")` that was its only compile
   site, in the same commit that turned the flag off. `xrce_zephyr_udp_init` has
   therefore been absent from every archive since 2026-05; a downstream
   resolving it has been broken for four months, and deleting the file cannot
   regress anything. The posix twin is genuinely different: `transport_posix_udp.c`
   and `transport_posix_serial.c` ARE compiled (`backend_posix` in
   `xrce-sources.txt`), so `xrce_posix_udp_init` really is still resolvable.

Nothing in the tree referenced it. `session.c` routes every bare `host:port`
locator through `xrce_nros_udp_init` on every platform (`session.c:450`), which
is `transport_nros_udp.c` over `nros_platform_udp_*` — that IS the Zephyr UDP
path that works today, and it is in `backend_core`, i.e. compiled by both lanes
unconditionally. `internal.h` is not installed (phase 140 deleted this package's
install rules), so the prototype was never reachable from outside the repo either.

Sibling sweep, as the class rule asks: `NOT_COMPILED` had exactly one entry, and
every other in-repo `.c` under `packages/rmw/xrce/**` is accounted for — ten
backend TUs in `xrce-sources.txt`, and `tests/{smoke,entity_lifetime}.c` compiled
by `nros-rmw-xrce/tests/CMakeLists.txt` (outside the gate's `src/*.c` glob by
design). The list is now empty; the mechanism stays, with a comment saying that
empty is the intended state and that a TU hidden behind a `never` flag is not a
`NOT_COMPILED` candidate but a deletion candidate.

### Deliberately not done

- **`xrce-config.txt` line 191's comment** still reads "`transport_zephyr_udp.c`
  is compiled by nobody (issue 1073)". The `flag uxr UCLIENT_PLATFORM_ZEPHYR
  never` row it annotates stays correct regardless — that row is about upstream's
  platform selection, not about our TU — but the sentence now names a file that
  does not exist. Left to the agent holding that file (issue 1078).
- **`zephyr/CMakeLists.txt:312`** claims the cffi shim's `platform-zephyr`
  feature "drops every POSIX TU and emits `UCLIENT_PLATFORM_ZEPHYR`". That has
  been false since 129.C.1 deleted those features (`build.rs` says so in its own
  comment: "No `CARGO_FEATURE_PLATFORM_*` reads"). Stale comment, different file,
  not touched here.
- **`xrce_posix_udp_init` is compiled but called by nobody** — `session.c` uses
  only `xrce_nros_udp_init`. That is a live symbol kept on purpose for external
  callers (`xrce-sources.txt` says so), so it is not this issue; whether the
  "one cycle" has expired is a separate decision, not made here.

### Verification

- `just check xrce-source-manifest` → `OK — 41 sources in 9 groups, conditions
  ['always', 'never', 'posix', 'posix_ip'], both lanes derive them backend_core:8,
  backend_posix:2, ucdr:5, uxr_ip_udp:2, uxr_misc:3, uxr_posix_time:1,
  uxr_serialization:3, uxr_session:10, uxr_stream:7`. Its check (5) was
  negative-controlled with a throwaway `src/zz_gate_probe.c`, which it rejected —
  an empty `NOT_COMPILED` does not disarm the orphan check.
- `just check xrce-one-vendored-compile` and `just check xrce-config-manifest`
  green (the two gates over the same manifests).
- `cargo build -p nros-rmw-xrce-cffi` OK; `just check rmw-xrce` 2/2 tests passed,
  and its build log names the compiled set — `session.c subscriber.c service.c
  transport_custom.c transport_nros_udp.c transport_posix_udp.c
  transport_posix_serial.c` (plus vtable/publisher/platform_aliases from the
  incremental portion), with no `transport_zephyr_udp.c`.
