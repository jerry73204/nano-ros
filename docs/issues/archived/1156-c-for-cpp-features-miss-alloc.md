---
id: 1156
title: "#1100 fixed one of the two strings that build `nros-c`; a C++-only
  Zephyr image uses the other, and still carries two units"
status: resolved
type: bug
area: build
related: [issue-1100, issue-0745, rfc-0044]
resolved_in: "phase-412 follow-up"
---

## Symptom

#1100's acceptance, run for the first time on the consumer that reported it:

```
$ ./build.sh --platform zephyr-fvp -d build/zephyr-fvp-acc   # exit 0
$ ./build.sh --platform zephyr-fvp -d build/zephyr-fvp-acc   # exit 1

nros-cpp: .../nros-rust/nros-c-generated/nros/nros_config_generated.h was
written by another crate with DIFFERENT probed sizes.
  EXECUTOR_OPAQUE_U64S:         on-disk=59638 vs would-write=59669
  NROS_EXECUTOR_SIZE:           on-disk=477104 vs would-write=477352
  NROS_EXECUTOR_MAIN_STACK_MIN: on-disk=3136 vs would-write=3248
```

The numbers moved by a few bytes since #1100 (unrelated churn in the range);
the failure is identical, and the two variants are unchanged:

```
nros-c-8c9aa73f430f95e6  NROS_CONFIG_VARIANT "alloc_..._param_services_platform_zephyr_..."
nros-c-0408b612d50fd1f9  NROS_CONFIG_VARIANT "critical_section_..._platform_zephyr_..."
```

## Cause

`zephyr/CMakeLists.txt` builds `nros-c` from **two** different feature strings,
and #1100 (`3a54cbbcf`) appended `alloc` / `param-services` to only one of them.

* `_nros_features` (~line 305) — the **C-API-on** path. This is what #1100
  patched, guarded by `if(CONFIG_NROS_CPP_API)`.
* `_nros_c_for_cpp_features` (~line 652) — the `if(NOT CONFIG_NROS_C_API)`
  branch inside the C++ block, which builds the same `nros_c_cargo_build`
  target for an image that has the C++ API and **not** the C API. Untouched.

A C++-only image takes the second path, so the split survives. Straight from
its `build.ninja`:

```
--features rmw-cffi,platform-zephyr,ros-humble,panic-platform                       <- nros_c_cargo_build
--features rmw-cffi,alloc,platform-zephyr,ros-humble,param-services,panic-platform  <- nros_cpp_cargo_build
```

Autoware Safety Island is exactly that configuration: `CONFIG_NROS_CPP_API=y`,
`CONFIG_NROS_C_API` unset. It is the shape #1100 was reported from, and the
shape the fix does not reach.

The rule is already written three lines above the untouched string, for a
different feature:

```cmake
# phase-8 W4 — callback tracing. MUST match the nros-cpp string above:
# both archives are linked into this image from one nros-node dep
# graph, and a feature that reaches only one of them splits the unit.
string(APPEND _nros_c_for_cpp_features "${_nros_trace_suffix}")
```

`alloc` and `param-services` are features that reach only one of them.

## Fix

Apply #1100's append to `_nros_c_for_cpp_features` as well, under the same
condition its sibling uses:

* `alloc` unconditionally — the enclosing block is already
  `if(CONFIG_NROS_CPP_API)`, so the C++ half has it by construction
  (`_nros_cpp_features` sets it at ~line 604);
* `param-services` under `"param_services" IN_LIST NANO_ROS_FEATURES`, which is
  what `_nros_cpp_features` gates on (~line 624).

## Acceptance

The one #1100 stated and could not run: build a C++ Zephyr image, then build
again in the same directory. Both must succeed. ASI's `zephyr-fvp` lane is that
image, and its CI runs a second build implicitly every time it starts a model.

## What the first fix got wrong, found 2026-09-07 on silicon

The fix above was applied and the acceptance still failed — on the mr_canhubk3
board build, not the FVP:

```
nros-cpp: build-<d>/nros-rust/nros-c-generated/nros/nros_config_generated.h was
written by another crate with DIFFERENT probed sizes.
  NROS_EXECUTOR_SIZE:  on-disk=63632 vs would-write=63512
```

Straight from that build's `build.ninja`:

```
nros_c_cargo_build   --features rmw-cffi,cffi-zenoh-cffi,platform-zephyr,ros-humble,alloc,panic-platform
nros_cpp_cargo_build --features rmw-zenoh-cffi,platform-zephyr,ros-humble,panic-platform
```

Two units again, and the split is now the OTHER way round: the C string carries
`alloc` and the C++ string does not.

The fix appended `,alloc` unconditionally, on the reasoning quoted above -- the
branch sits inside `if(CONFIG_NROS_CPP_API)`, "so the C++ half has it by
construction". That is true of two of the three backend strings and false of the
third:

| `_nros_cpp_features` | line | names `alloc`? |
| --- | --- | --- |
| zenoh | 567 | **no** |
| xrce | 585 | yes |
| cyclonedds | 604 | yes |

A Zephyr zenoh image is the configuration #1100 and #1156 were both reported
from, so the append did not close the split there -- it inverted it.

## Fix, second attempt

Ask the C++ string instead of restating what it holds:

```cmake
if(_nros_cpp_features MATCHES "(^|,)alloc(,|$)")
    string(APPEND _nros_c_for_cpp_features ",alloc")
endif()
if(_nros_cpp_features MATCHES "(^|,)param-services(,|$)")
    string(APPEND _nros_c_for_cpp_features ",param-services")
endif()
```

A condition that restates another string's contents is a second copy of it, and
this copy has now drifted three times (#1100, #1156, this).

The zenoh branch also named `cffi-zenoh-cffi`, the back-compat alias kept "so
downstream cmake glue that still passes the old name does not error"
(`nros-c/Cargo.toml:109`, `cffi-zenoh-cffi = ["rmw-zenoh"]`). It resolves to the
same feature and gates no code, but cargo keys a unit by the feature SET, so the
alias spelling asked for a set nothing else names. The in-tree glue is not
downstream glue; it now names `rmw-zenoh`, which is what `nros-cpp` forwards.

## Acceptance, run

`just BOARD_BUILD_DIR=build-stampfix board-build` three times in one directory
on `mr_canhubk3/s32k344`, from a wiped dir: `rc=0`, `rc=0`, `rc=0`, and zero
occurrences of `DIFFERENT probed sizes` in the full log. Before the fix the same
script reported the panic in pass 1's `rom_report`/`ram_report` step and
`PASS 2 rc=1`.

Both `nros-c` build-script units now emit the same header:

```
NROS_EXECUTOR_SIZE 63512
NROS_CONFIG_VARIANT "critical_section_global_allocator_panic_platform_platform_zephyr_rmw_cffi_rmw_zenoh_ros_humble"
```

Not verified on the FVP lane this issue was reported from -- no FVP run was made
here. What it is verified against is the board, which is the harder case
(#1100's fix was NOT VERIFIED against any Zephyr build at all).
