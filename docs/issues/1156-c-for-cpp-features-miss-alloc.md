---
id: 1156
title: "#1100 fixed one of the two strings that build `nros-c`; a C++-only
  Zephyr image uses the other, and still carries two units"
status: open
type: bug
area: build
related: [issue-1100, issue-0745, rfc-0044]
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
