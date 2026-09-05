# Phase 325 — uORB interop: direct, and bridged to any RMW

**Status (2026-09-04). W0, W1 and W2 LANDED 2026-07-31; W3's GATE PASSED the
same day; W3.1-W3.4 remain — and are NO LONGER BLOCKED.**

The 2026-07-31 line read "Draft. Not started." while 14 of this phase's 18 boxes
were ticked and W0/W1/W2 each carry a RESULT section dated that same day. The
doc already said so five lines down, in `**Blocked on:**` — "for W3 only; W0-W2
are done" — so the contradiction was between two lines of one header block, and
a reader who stopped at the status got the wrong answer.

**The blocker is gone too.** `**Blocked on:** issue 0362 (no C++ `px4_msgs`
codegen)` — issue 0362 is `status: resolved` and archived. W3 has been blocked
on paper since July against an issue that was fixed; nothing was waiting on
anything. Whoever picks W3 up should start from W3's own GATE PASSED note
(two backends in one PX4 module, `rc=0`, zero undefined) rather than from the
blocker.

**Previously (2026-07-31): Draft.** Not started.
**Implements:** RFC-0026 (example layout), RFC-0048 (cmake consumption).
**Successor to:** [phase-316](archived/phase-316-example-tree-axes.md) W4, which carried
the decisions but not the work — scoping showed W4 is a phase, not a work item.
**Informed by:** issues 0351 (proofs that observe the wrong thing), 0356
(`px4_e2e` targets a retired tree), 0288, 0159 (`.clang-format-ignore` precedent).
**Blocked on:** issue 0362 (no C++ `px4_msgs` codegen) for W3 only; W0–W2 are done.

## Goal

Two demonstrations, making two distinct claims:

| | claim | proven against | falsified by |
| --- | --- | --- | --- |
| **direct** (W2) | nano-ros speaks PX4's in-memory format, so no serialization happens at all | a **stock, unmodified PX4 module** | a stock module cannot read the topic |
| **bridge** (W3) | nano-ros carries uORB traffic out to any RMW it supports, selected at build time | a **real ROS 2 node** | ROS 2 cannot see it, or only one backend works |

Both acceptances name a **foreign peer**, and that is not incidental. A
nano-ros↔nano-ros test passes identically whether the encoding is right or
wrong, because both ends share the bug — issue 0351's shape, hit twice during
phase-316. The stock PX4 module and the real ROS 2 node ARE the measurement; a
demo that drops them proves nothing it claims to prove.

## Why uORB is the special one

Decided by the maintainer, and load-bearing in the code rather than aspirational:

| | every other backend | uORB |
| --- | --- | --- |
| wire bytes | CDR encoding of the message | the PX4 C struct, verbatim |
| type identity | ROS type name + type hash | `ORB_ID(<topic>)`, a static descriptor |
| serialization cost | encode + decode per sample | none — the payload IS the struct |
| who can read it | another nano-ros / ROS 2 endpoint | **any stock PX4 module**, unmodified |

`publisher_publish_raw` checks `len >= meta->o_size` and hands the caller's bytes
straight to `orb_publish`. `publisher_create` ignores `type_name`, `type_hash`,
`qos` and `domain_id`, resolving the topic through `nros_rmw_uorb_register_topic`
to a `const struct orb_metadata *`. Everywhere else nano-ros interoperates by
speaking a wire protocol; here it interoperates by **sharing PX4's in-memory
type**.

That is also the cleanest statement of why `examples/px4/cpp/uorb/` looked like an
RMW path level and was not one (phase-316): uORB is not a transport choice, it is
the absence of a transport.

## PX4 convention is normative for everything inside a PX4 module

**Maintainer instruction (2026-07-31): the example structure and content follow
PX4 convention.** Not nano-ros's house style, and not a hybrid. A PX4 module is
read, reviewed and maintained by PX4 people; it should look like the modules
next to it in `src/examples/`.

Verified against the vendored tree (`third-party/px4/PX4-Autopilot`) rather than
recalled. The reference is `src/examples/work_item/` — the canonical C++ module.

### Layout

```
<EXTERNAL_MODULES_LOCATION>/src/modules/<snake_name>/
    CMakeLists.txt      BSD 3-clause header, then px4_add_module(...)
    Kconfig             menuconfig <SECTION>_<NAME>, bool, default n, ---help---
    <CamelCase>.hpp     class decl, matching the class name
    <CamelCase>.cpp     impl + the extern "C" entry point
```

C++ modules name files after the CLASS (`WorkItemExample.cpp/.hpp`), not after
the module (`work_item`). Plain-C modules use `snake_name.c`
(`px4_simple_app.c`). Directory is snake_case; class is CamelCase; the
`px4_add_module` `MODULE` argument is `<section>__<name>`
(`examples__work_item`).

**This does not conflict with RFC-0026.** The path rule phase-316 enforced is
about the `<plat>/<lang>/<case>` LEVELS; what an example contains internally is
its own business. So `examples/px4/cpp/firmware/` is the case dir, and inside it
sits the PX4-required `src/modules/<name>/` tree — the example dir IS an
`EXTERNAL_MODULES_LOCATION` root. Note this is the same collision that produced
the hoist+shim phase-316 W3.1 deleted; it is fine here only because the example
dir is the root, not a leaf inside one.

### The module class

```cpp
class NrosUorbTalker : public ModuleBase<NrosUorbTalker>, public ModuleParams,
                       public px4::ScheduledWorkItem
```

`ModuleBase<T>` is what gives a module `start` / `stop` / `status` from the pxh
shell for free. Required members:

| member | why |
| --- | --- |
| `static int task_spawn(int argc, char *argv[])` | ModuleBase contract |
| `static int custom_command(int argc, char *argv[])` | ModuleBase contract |
| `static int print_usage(const char *reason = nullptr)` | ModuleBase contract |
| `int print_status() override` | what `<module> status` prints |
| `void Run() override` | the work-queue tick |
| `bool init()` | schedules the first run / registers callbacks |

Entry point, at the bottom of the `.cpp`:

```cpp
extern "C" __EXPORT int nros_uorb_talker_main(int argc, char *argv[])
{
	return NrosUorbTalker::main(argc, argv);
}
```

`MAIN` in `px4_add_module()` must match the `<name>_main` symbol.

### Usage strings are not optional

```cpp
PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
...
)DESCR_STR");
PRINT_MODULE_USAGE_NAME("nros_uorb_talker", "examples");
PRINT_MODULE_USAGE_COMMAND("start");
PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
```

This is what `<module> help` prints AND what PX4 scrapes for its module
reference docs. A module without it is invisible in PX4's documentation.

### Kconfig

Every module dir carries one:

```
menuconfig EXAMPLES_NROS_UORB_TALKER
	bool "nros_uorb_talker"
	default n
	---help---
		Enable support for nros_uorb_talker
```

`integrations/px4/sitl-overlay/render-overlay.sh` already walks
`<px4>/src/modules/nros_*/` and renders the defconfig fragment — reuse it, do not
add a second mechanism.

### Style: PX4's, and it conflicts with ours

| | PX4 | nano-ros |
| --- | --- | --- |
| indent | **tab**, `tab_width = 8` | 4 spaces |
| max line | 120 | 100 |
| formatter | `Tools/astyle/fix_code_style.sh` | `.clang-format` (LLVM-based) |
| file header | **BSD 3-clause block, every file** | none required |

These are not reconcilable. **Correction (2026-07-31): our formatter is not the
cause.** I assumed `check-c-fmt` / `check-cpp-fmt` were reformatting PX4 modules;
they are not. Both enumerate explicit paths — `nros-c/include`, `zpico-zephyr`,
`examples/native/c/**`, `nros-cpp/include/nros/*.hpp` — and no PX4 module tree is
among them. The 4-space style was simply how the file was written. Checking
before acting is the difference between a guard and a no-op dressed as a fix.

What was true regardless:
`packages/testing/nros-px4-register-check/.../nros_register_check.cpp` is
**4-space indented, carries no BSD header, and is a bare `extern "C"` main** with
no `ModuleBase`, no `print_usage` and therefore no `status` / `stop` / `help`. It
is a nano-ros file wearing a PX4 file's location.

- [x] **W0.1** `.clang-format-ignore` gains the PX4 module trees. A **guard, not a
      fix** — nothing formats them today. But both fmt recipes enumerate paths by
      hand, so the day someone widens a glob to `examples/**/*.cpp` (a likely and
      otherwise-correct change), PX4 trees would be silently reflowed into
      4-space. `.clang-format-ignore` is read by clang-format itself, so the guard
      holds whatever the recipe globs become. Precedent: `cmake/templates/*`
      (issue 0159 — reflow broke `@VAR@` tokens).
- [x] **W0.2** `nros-px4-register-check` brought to PX4 convention.

**W0 lands before W2**, so the first real example is written in the right style
rather than converted afterwards.

### One thing that is NOT PX4 convention, deliberately

A PX4 module normally reaches uORB through `uORB::Publication<T>` /
`uORB::Subscription`. **The demo must not** — those bypass nano-ros entirely, so
a module using them proves nothing about nano-ros and would pass identically if
the backend were deleted. The demo publishes through the **nano-ros** publisher
(`publish_raw` over the `<uORB/topics/*.h>` struct); everything AROUND that call
— module class, Kconfig, usage strings, style, file naming — is PX4's.

Same family of trap as the foreign-peer rule above: a proof that observes
something common to the working and broken cases proves nothing.

## What is already true

Worth stating precisely, because three artifacts look like PX4 integration and
the tree reads as though this is solved:

| artifact | what it actually exercises |
| --- | --- |
| `nros-rmw-uorb/tests/register_smoke.cpp` | drives the RMW **vtable directly**, stubbing `nros_rmw_cffi_register` AND the uORB ABI. Never touches `nros-cpp`. |
| `packages/testing/nros-px4-register-check/` | compiles the backend inline against **real PX4 headers** and calls `nros_rmw_uorb_register()`. Proves it LINKS. Does not link `nros-cpp` — the weak `register_fallback.c` exists precisely so it need not. |
| `integrations/px4/module-template/nano_ros_app.cpp` | the node code is a **comment**: *"Replace this comment block with NodeBuilder / Publisher calls"*. |

So: **no nano-ros node has ever been constructed on the uORB backend.** The
backend's proven surface stops below the node API. `examples/README.md` called the
register-check "the canonical PX4 uORB surface" — true about linking, easy to
misread as usage.

Two things that ARE proven and remove risk:

- **`publish_raw` / `subscription_take` are already public** on both the C and C++
  APIs. The direct example needs no new data-plane machinery.
- **Two live backends in one image works.** `examples/bridges/tt-zenoh-to-cyclonedds`
  does `nros_rmw_zenoh::register()` + `nros_rmw_cyclonedds_sys::register()` then
  `Executor::open_with_rmw("zenoh", &cfg)` and opens a second session.
  `open_with_rmw` takes the backend by **name**, so build-time selection needs
  only a cargo feature choosing which `register()` compiles in and which name
  string is passed.

## The actual gap: consumption, not a platform port

phase-316's note said "there is no `cmake/platform/nano-ros-px4.cmake`, and every
other platform has one". That is true and **the wrong diagnosis** — recorded here
because a wrong diagnosis points at the wrong fix, which is this session's
recurring lesson.

Platform modules are consumed by nano-ros's OWN root `CMakeLists.txt`
(`cmake/platform/nano-ros-${NANO_ROS_PLATFORM}.cmake`, resolved at
`CMakeLists.txt:116`). A PX4 module is built by **PX4's** cmake via
`px4_add_module()` and never enters that file. And SITL is an ordinary host
x86_64 process, so the platform shim it needs is `posix`, which already exists.

The gap is a **consumption path**: how a `px4_add_module()` target links
`libnros_cpp.a` + the posix platform shim + the uORB backend.

~~That is RFC-0048 territory — `find_package(nano_ros)` → `_nros_bootstrap` →
`add_subdirectory`.~~ **Wrong, measured in W1.1 (see below).** `_nros_bootstrap`
works by `add_subdirectory`, which compiles nano-ros sources inside PX4's cmake
under PX4's `-Werror -Wfatal-errors -Wpedantic …` flags, and the posix shim does
not survive them. The working shape is the opposite: **nano-ros builds its own
artifacts, PX4 links prebuilt archives.**

**Real PX4 boards (NuttX, cross-compiled) are explicitly out of scope.** Both
demos run on SITL. A board port is the `nuttx` platform plus a cross toolchain and
is its own phase; nothing here should pretend to deliver it.

## Work items

### W0 — PX4 convention, before anything is written in it

Both items are stated in full under "Style: PX4's, and it conflicts with ours"
above; listed here so the sequencing is unmissable.

- [x] **W0.1** `.clang-format-ignore` the PX4 module trees; PX4's astyle owns them.
      (A guard, not a fix — nothing formats them today; see the note above.)
- [x] **W0.2** Bring `nros-px4-register-check` into PX4 convention (tabs, Kconfig,
      `PRINT_MODULE_*`), or record why not. Done — and NOT `ModuleBase`, which is
      for daemons; this is a one-shot command modelled on `src/systemcmds/gpio`.
      The BSD header is deliberately not adopted (licensing, not style).

**Acceptance:** `nros_register_check help` prints its `PRINT_MODULE_DESCRIPTION`
from the pxh shell. **DONE 2026-07-31** — verified live:

```
### Description
Link/registration check for the nano-ros uORB RMW backend.
Usage: nros_register_check [arguments...]
INFO  [nros_register_check] nros_rmw_uorb_register() -> OK
```

Note the acceptance says `help`, not `status`. An earlier draft said `status` —
wrong: `status` comes from `ModuleBase<T>`, which is for modules that DAEMONIZE.
This one is a one-shot command, modelled on `src/systemcmds/gpio`, so it has no
start/stop/status to offer and inheriting `ModuleBase` to fake them would be
worse than offering nothing. The old header even documented
`nros_register_check start` — an invocation that never existed.

### W1 — a PX4 module can consume nano-ros

- [x] **W1.1** Prove a `px4_add_module()` target can link nano-ros. **DONE** —
      see the result below. The friction predicted here ("PX4's module factory and
      nano-ros's `add_subdirectory` import disagree about flags") is exactly what
      happened, and it disqualifies `find_package(nano_ros)` rather than needing a
      workaround.
- [x] **W1.2** Wrap the result as ONE helper — working name
      `nros_px4_add_module()` — under `integrations/px4/`, so a module author
      writes one call. Not a copy of `px4_add_module`'s argument surface: forward
      to it. Per W1.1 it is a **link helper, not a bootstrap**: two prebuilt `.a`
      paths plus the registration hook, and it must not call
      `find_package(nano_ros)`.
      **DONE** — `integrations/px4/NanoRosPx4Module.cmake`.
- [x] **W1.3** Retire the module-template's comment-block placeholder in favour of
      the helper, so the template compiles what it documents. A template whose
      body is `// Replace this comment block` is how the gap stayed invisible.
      **DONE** — it now builds and runs.
- [x] **W1.4** Export the generated headers alongside the archive (found by W1.3).
      **DONE** — the helper prepends `target/nros-{c,cpp}-generated`, and the
      template now `#include <nros/init.h>` and builds. Filed **issue 0360** on the
      way: those headers are written to a FLAT path, not the `<variant_slug>/` the
      stub documents, so two feature sets overwrite one header carrying storage
      sizes.

#### W1.2 / W1.3 RESULT (2026-07-31)

`nros_px4_add_module()` exists and the module-template — whose body was a comment
for three phases — now BUILDS and RUNS from one call:

```
INFO  [nano_ros_app] nano-ros uORB backend registered
### Description
Usage: nano_ros_app [arguments...]
```

That line means the generated strong `nros_app_register_backends()` ran, reaching
`nros_rmw_cffi_register` in `libnros_cpp.a` — the real symbol, not the weak
fallback the register-check relies on.

The helper takes `BACKENDS` and supplies that backend's sources, includes and
flags itself. The alternative — every module hand-listing the uORB backend's 8
`.cpp` files, two include dirs and one `-D`, as `nros-px4-register-check` does —
is build knowledge copied per module, and copies drift the moment the backend
gains a file. `BACKENDS` names WHAT; the helper decides HOW.

##### Four link shapes were wrong before one was right

Worth recording, because each failed differently and only the last is obvious in
hindsight:

| shape | outcome |
| --- | --- |
| `target_sources()` alone | `undefined reference to nros_app_register_backends`. A linker pulls an archive member only if it resolves an undefined symbol AT THE MOMENT it scans that archive, and PX4 puts the module archive BEFORE `libnros_cpp.a`. |
| an OBJECT library | its object never reaches the link line at all — PX4 assembles the executable from module ARCHIVES it collects itself, not from CMake object propagation. |
| a separate trailing archive | resolved the hook, then failed on `nros_rmw_uorb_register`. The dependency is genuinely CIRCULAR — module.a needs `libnros_cpp.a`, which needs the hook, which needs the backend back in module.a — so no ordering of distinct archives satisfies it. |
| **`-Wl,--undefined=nros_app_register_backends` + `target_sources()`** | works. The symbol is undefined from the START, so the member is pulled on the FIRST scan. Same class as nros-c's FORCE_LINK anchors: a symbol nothing has referenced yet is one the linker feels free to drop. |

(CMake 3.24's `$<LINK_GROUP:RESCAN>` would express the `--start-group` answer
directly; this tree is on 3.22.)

##### A latent bug the probe could not catch

The helper's include paths were written as `packages/core/nros-{cpp,c}/include`
— **wrong**; phase-321 moved both to `packages/api/`. The W1.1 probe declared its
symbols by hand rather than including headers (deliberately, to isolate "does it
link"), so the paths were never exercised and the build stayed green. It would
have failed for the first caller that included anything.

The helper now validates every include dir with `IS_DIRECTORY` at configure time
and fails naming the missing path. A path list nothing reads is a path list
nothing keeps correct.

##### W1.4 — the generated-config gap (CLOSED)

Making the template `#include <nros/init.h>` fails:

```
error: "nros_config_generated.h must be supplied per-build by the build system"
```

nano-ros's own cmake emits it into `${CMAKE_BINARY_DIR}/nros-rust/nros-c-generated/`,
and the prebuilt-archive model exports nothing. It cannot simply be copied from
anywhere: it carries **storage sizes**, so it must come from the same build as
`libnros_cpp.a` — a mismatched copy is the issue-0268 silent-overflow class.

**Resolved.** `build.rs` already emits them to
`$CARGO_TARGET_DIR/nros-{c,cpp}-generated/nros/`; nothing pointed at them. The
helper now adds both, and the template includes `<nros/init.h>` and builds.

Two things make it work, both easy to get wrong:

- **PREPEND, not append.** `packages/api/nros-c/include/nros/` ships a
  same-named STUB whose entire body is the `#error`. Search that directory first
  and the stub wins. Include ORDER is the mechanism.
- **Same cargo invocation.** The headers carry storage sizes; pairing them with a
  differently-featured archive is the issue-0268 silent-overflow class.

Which is where **issue 0360** came from: the stub documents
`nros-c-generated/<variant_slug>/nros/…` ("sorted underscore-joined cargo feature
list") and the code writes a FLAT path — one file per project, not per variant.
So the second point above is currently an unenforced convention, and a second
`cargo build` with different features silently overwrites the header a first
archive's consumer compiles against. Documentation describing a safety mechanism
the code does not implement is worse than none: it tells the next person the
hazard is handled.

##### `nros-px4-register-check` deliberately NOT migrated

It would be a natural second caller, but it exists to prove the backend LINKS
against real PX4 headers using the weak `nros_rmw_cffi_register` fallback —
without `libnros_cpp.a`. Moving it onto the helper would link the strong symbol
and it would stop testing the thing it was built to test.

**Acceptance:** a PX4 SITL build produces a module that links `libnros_cpp.a` and
starts. No node behaviour yet — that is W2. **MET for W1.1.**

#### W1.1 RESULT (2026-07-31): it links, and it runs

**Answered: yes.** A `px4_add_module()` target links `libnros_cpp.a` and starts.
Receipt, from the pxh shell after a full SITL build (`rc=0`, zero undefined
references):

```
INFO  [nros_link_check] nros-cpp linked: nros_rmw_cffi_register=0x5fdc2807ab1d nros_cpp_node_create=0x5fdc28051638
```

The probe takes the ADDRESS of each symbol rather than calling it: the question
was whether they resolve at link time, and printing a real address proves that
without needing an initialised runtime. Calling them would have conflated "does
it link" with "does it work", which is the distinction W1 exists to settle.

##### Three link inputs, not one

`libnros_cpp.a` alone leaves exactly 10 undefined symbols, in two families —
both documented, neither surprising:

| missing | supplied by |
| --- | --- |
| `nros_platform_{wake_init,wake_wait_ms,wake_signal,wake_drop,wake_storage_size,wake_storage_align,sleep_ms}` | the **posix** platform shim (`packages/platform/nros-platform-posix`) |
| `nros_app_register_backends` | normally a strong-stub TU **generated** by `nano_ros_link_rmw()` (`cmake/NanoRosLink.cmake`); a hand-rolled PX4 module gets no generation and must define it |

The platform half confirms the phase's premise: SITL is an ordinary host process,
so it wants the **same posix shim every native build uses**. No platform port.

##### CORRECTION: `find_package(nano_ros)` is the wrong consumption path

This phase said the gap was "RFC-0048 territory —
`find_package(nano_ros)` → `_nros_bootstrap` → `add_subdirectory`". **Measured,
that is wrong**, and it is the third correction to my own analysis in this phase.

`_nros_bootstrap` works by `add_subdirectory`, which builds nano-ros sources
*inside PX4's cmake* — where they inherit PX4's flags:

```
-Werror -Wfatal-errors -Wpedantic -Wnested-externs -Wbad-function-cast
-Wshadow -Wdouble-promotion -Wfloat-equal -Wlogical-op ...
```

That set is far stricter than nano-ros's own, and `nros-platform-posix` does not
survive it — every TU died on `"_DEFAULT_SOURCE" redefined [-Werror]`, PX4 having
already defined it. Fixing that one macro would only buy the next warning.

**The shape that works: nano-ros artifacts are built by nano-ros's build, and PX4
links prebuilt archives.** Each project keeps its own warning policy on its own
sources. This is already how `libnros_cpp.a` reaches the link (cargo builds it;
cmake only links it) — the platform shim simply has to follow the same rule
instead of being pulled into PX4's tree:

```sh
cargo build -p nros-cpp --no-default-features --features std,rmw-cffi --release
cmake -S packages/platform/nros-platform-posix -B <dir> && cmake --build <dir>
```

So W1.2's helper is a **link helper, not a bootstrap**: it points a PX4 module at
two prebuilt `.a` files and provides the registration hook. It must NOT call
`find_package(nano_ros)`.

##### Reproducing the probe

`EXTRA_CMAKE_ARGS` is not forwarded by PX4's Makefile — pass configuration
through the environment, as `NROS_REPO_DIR` already is:

```sh
export NROS_PLATFORM_POSIX_A=<dir>/libnros_platform_posix.a
make -C third-party/px4/PX4-Autopilot px4_sitl_default \
     EXTERNAL_MODULES_LOCATION=<probe-root>
```

##### Toolchain gotcha worth knowing

The system `nm` cannot read these archives — `LLVM gold plugin has failed to
create LTO module: Opaque pointers are only supported in -opaque-pointers mode
(Producer: LLVM22.1.2-rust-1.96.0 Reader: LLVM 14.0.0)` — and reports **no
symbols**, which reads exactly like an empty archive. Use the toolchain's own:

```sh
~/.rustup/toolchains/<tc>/lib/rustlib/x86_64-unknown-linux-gnu/bin/llvm-nm
```

`libnros_cpp.a` exports 124 `nros_cpp_*` symbols; a bare `nm` says 0.


### W2 — the direct demo: nano-ros ↔ a stock PX4 module

- [x] **W2.1** A nano-ros node inside a PX4 module that publishes a real PX4
      topic:
      `nros_rmw_uorb_register_topic("/<topic>", "<ros_type_name>", ORB_ID(<topic>))`,
      then `publish_raw((const uint8_t *)&msg, sizeof msg)` with `msg` a
      `<uORB/topics/*.h>` struct. The message type comes from PX4's headers, NOT
      from `nros generate-*`.
- [x] **W2.2** The subscribe direction, reading a topic a stock PX4 module
      publishes.
- [x] **W2.3** Lands at `examples/px4/cpp/firmware/` — which this creates.
      phase-316 W3.1 deliberately left the dir uncreated rather than empty. That
      dir is an `EXTERNAL_MODULES_LOCATION` root, so the module sits at
      `firmware/src/modules/<snake_name>/` per PX4's requirement.
- [x] **W2.5** Written to PX4 convention throughout — `ModuleBase<T>`, `Kconfig`,
      `PRINT_MODULE_*` usage strings, CamelCase files matching the class, tabs,
      BSD header. See the normative section above. The only deliberate departure
      is that publishing goes through the **nano-ros** publisher rather than
      `uORB::Publication<T>`.
- [x] **W2.4** A test that observes the exchange **from the PX4 side**: `listener
      <topic>` in the SITL shell, or an upstream module that already subscribes
      it. Assert on that output.

      The harness exists — reuse it, do not write a second one. `Px4Sitl` (from
      the `px4-sitl-tests` path-dep in `nros-px4-sitl-test`) gives boot, pxh
      shell, log-wait with timeout, a snapshot on failure, and SIGTERM of the
      process group on `Drop`:

      ```rust
      let sitl = Px4Sitl::boot_in(&build_dir)?;
      sitl.shell("<nros module> start")?;
      sitl.shell("listener <topic>")?;              // the STOCK consumer
      let line = sitl.wait_for_log("<field marker>", RECV_TIMEOUT)
          .map_err(|e| panic!("{e:?}\n{}", sitl.log_snapshot()))?;
      ```

      `wait_for_log` is a SUBSTRING match, not a regex — the deleted test's own
      comment corrected itself about this twice, so it is worth stating once.

      This recipe is recorded here because the test it came from was **deleted**
      (issue 0356): `px4_e2e.rs` drove `nros_listener` + `nros_talker` — two
      NANO-ROS modules — and asserted one logged `recv:`. A loopback, satisfied
      identically by a correct and a broken struct layout, since both ends share
      the bug. The scaffolding was good; what it pointed at was not.

**Acceptance:** a message crosses between a nano-ros node and an unmodified PX4
module, with no serialization step on either side, and the test reads it from the
PX4 end.

**Explicitly NOT acceptance:** nano-ros subscribing its own publication. That
passes identically with a correct and a broken struct layout — it measures the
loopback, not the interop.

#### W2 RESULT (2026-07-31): met, by the stock consumer

`examples/px4/cpp/firmware/src/modules/nros_uorb_demo/`, built by
`just px4 build-sitl-example`. PX4's own `listener` — which knows nothing about
nano-ros — reading a nano-ros publication:

```
TOPIC: debug_key_value
 debug_key_value
    timestamp: 16736000 (0.920000 seconds ago)
    value: 5.00000
    key: "nros"
```

`key` and `value` come back correct, so the struct layout agrees byte for byte;
a disagreement with `orb_metadata` would garble exactly these. And the other
direction, nano-ros reading PX4's commander:

```
INFO  [nros_uorb_demo] recv vehicle_status: nav_state=4 arming_state=1 (10 samples)
INFO  [nros_uorb_demo] published debug_key_value key=nros value=9.0 (10 samples)
```

Foreign peer on both ends, which was the whole requirement.

##### A blocker W1 could not have found: PX4 is C++14

`<nros/nros.hpp>` did not compile in a PX4 module —

```
node.hpp:100: error: inline variables are only available with '-std=c++17' [-Werror]
```

PX4 sets `CMAKE_CXX_STANDARD 14` for everything. W1 could not surface this: the
generated-header stub blocked including nano-ros headers at all until W1.4, and
the template then only reached `<nros/init.h>`.

It was **one** `inline constexpr` in the entire header set, and nano-ros already
intends C++14 compatibility — `just check cpp` runs a `-std=c++14 -fsyntax-only`
freestanding gate. Dropping `inline` fixes it with no loss: `constexpr` at
namespace scope is implicitly const and already has internal linkage per TU, so
the keyword bought nothing.

**The gate did not catch it**, which is the more useful finding — though not for
the reason first recorded here. My initial read was "its file list is narrower
than the rule it enforces". **Wrong**: the loop globs
`packages/api/nros-cpp/include/nros/*.hpp`, so `node.hpp` was always covered.

The actual defect is subtler and worse. The probe ran plain `-std=c++14`, and
**GCC accepts C++17 constructs as EXTENSIONS under `-std=c++14`, warning rather
than erroring**:

```
$ c++ -fsyntax-only -std=c++14 iv.cpp                    # rc=0, warning only
$ c++ -fsyntax-only -std=c++14 -pedantic-errors iv.cpp   # rc=1
```

So the gate named a standard it did not enforce. It could not detect the one
class that breaks PX4, which builds every module `-std=gnu++14 -Werror`.

**FIXED (2026-07-31):** `-pedantic-errors` added to both the header loop and the
`bind_service` instantiation probe. Measured before applying — zero violations
across every header and the probe — so it pins what was already true rather than
forcing a cleanup. Mutation-tested: reintroducing the `inline constexpr` now
fails `just check cpp` naming the line; removing it goes green.

Worth generalising: a syntax gate that names a standard should enforce it. `-std=`
alone is a request, not a constraint.

##### W2.4 — the automated test (DONE)

`packages/testing/nros-px4-sitl-test/tests/px4_uorb_interop_e2e.rs`, run by
`just px4 test-sitl-example`. Passes in ~2 s against a prebuilt SITL; the compile
lives in `build-sitl-example`, per CLAUDE.md's no-build-in-tests rule — unlike the
`px4_e2e` it replaces, which built SITL inside the test.

**Mutation-tested, because a passing assertion proves nothing until you have seen
it fail.** Changing the published key from `"nros"` to `"XXXX"` and rebuilding
turns the test RED; restoring it turns it green. So it is reading the actual
bytes that crossed into uORB, not something incidentally true.

Two things the writing turned up:

- **`Px4Sitl::shell()` returns the command's stdout; it does NOT go through the
  daemon log.** It spawns `bin/px4-<mod>` as a separate process. The first draft
  waited on `wait_for_log("key: \"nros\"")` and timed out at 30 s while the
  module was demonstrably publishing 30 samples. The daemon's own `PX4_INFO`
  lines DO reach the log, which is why the `vehicle_status` direction still uses
  `wait_for_log`.
- **The assertion is on the KEY, not the topic name.** `listener` prints
  `TOPIC: debug_key_value` even for an all-zero sample, so matching that would
  pass against a module publishing nothing.

The test also guards the stale-tree case: PX4 takes exactly ONE
`EXTERNAL_MODULES_LOCATION` per build and every build writes the same
`build/px4_sitl_default`, so a tree built by `build-sitl-cpp` would boot fine and
fail confusingly at `nros_uorb_demo start`. It asserts
`external_modules/modules/nros_uorb_demo/` exists and names the rebuild command —
the stale-fixture class caught at the top of the test rather than diagnosed from
a runtime symptom.

### W3 — the bridge: uORB → the build-time-selected RMW

- [x] **W3.1** A PX4 module holding two sessions: uORB inward
      (`nros_rmw_uorb_register()`), and outward on the RMW chosen at build time —
      cargo `rmw-*` features / `-DNROS_RMW=<backend>`, the same knob every other
      example uses. `Executor::open_with_rmw(<name>, …)` already takes the backend
      by name; the feature picks the `register()` call and the name string.
- [x] **W3.2** ONE path, no `<rmw>/` level and no backend pair in the directory
      name. This is phase-316's rule applied to the thing that used to break it:
      the outward backend is a build-time CHOICE, not a directory axis.
- [x] **W3.3** Build it against **at least two** backends (zenoh + one of
      xrce/cyclonedds). One backend does not demonstrate selection; it
      demonstrates a hardcoded bridge with extra ceremony.
- [ ] **W3.4** A test with a **real ROS 2 node** subscribing the bridged topic.
      `packages/testing/nros-tests/src/ros2.rs` + `ros_env.rs` already spawn real
      ROS 2 peers for the interop cells — reuse that, do not invent a second way.

**Acceptance:** a stock PX4 module's uORB topic reaches a real ROS 2 subscriber
through the bridge, and the same source builds against a second backend.

#### W3.1-W3.3 LANDED (2026-09-04); W3.4 remains and is scoped below

W3.1 and W3.2 were done on 2026-08-06 and the boxes were never ticked — the
module is `examples/px4/cpp/bridge/src/modules/nros_uorb_bridge/`, its README
records "Status (2026-08-06): WORKING" with a SITL receipt, and it holds both
sessions through `nros_cpp_init_multi` (NOT the `nros::init()` + `NodeBuilder`
shape this doc sketches at W3.1 — that one opens exactly one session and cannot
serve a bridge; issue 0436).

**W3.3 landed today.** `NROS_BRIDGE_RMW` was `#ifndef`-guarded with a `"zenoh"`
default and defined NOWHERE, so the choice existed in the source and could not
be made from outside — one backend with extra ceremony, which is what W3.3 says
does not count. One value now drives all three places it must reach: the cargo
feature that compiles the backend into `libnros_cpp.a`, the `BACKENDS` list that
registers it, and the name the session spec selects it by.

Built and verified against BOTH backends, not reasoned about:

    NROS_PX4_BRIDGE_RMW=zenoh just px4 build-bridge-example   -> [534/534], rc=0
    NROS_PX4_BRIDGE_RMW=xrce  just px4 build-bridge-example   -> see receipt below

with cmake logging `nros_uorb_bridge: outward backend = <name>` and the
generated `nros_app_register_backends.c` carrying exactly
`nros_rmw_uorb_register` + `nros_rmw_<name>_register`.

Three defects the BUILD found that the plumbing check did not:

* `rmw=zenoh` bound the literal string to the first positional. `just` has no
  named-argument syntax — archived phase-410 states the rule, and `lane=` works
  only because `scripts/build/fixture-lane.sh:74` strips the prefix by hand. The
  outward backend is an env var for that reason.
* This file's own README told users `build-bridge-example topics=vehicle_status`,
  which generated a message literally named `topics=vehicle_status`. Broken since
  it was written; fixed.
* `COMPILE_FLAGS` is a raw string, so `-DNROS_BRIDGE_RMW="\"${_rmw}\""`
  collapsed to `""zenoh""` and g++ rejected it ten minutes into a SITL build.
  `target_compile_definitions` quotes the value itself.

**W3.4 is NOT done, and it is not a tail of this work.** It needs a real PX4
SITL run with a ROS 2 subscriber; there is no host lane, because the only uORB
double in the tree is a link-time mock inside one CMake smoke binary
(`packages/rmw/uorb/nros-rmw-uorb/tests/register_smoke.cpp`) with no broker and
no second process. So it belongs beside `px4_uorb_interop_e2e.rs` in the
out-of-workspace `packages/testing/nros-px4-sitl-test/`, which today has no
`nros-tests` dependency — adding one is what makes `Ros2Process::topic_echo` and
`ZenohRouter` reachable, and is the "reuse, do not invent a second way" move
this phase asks for.

Two traps for whoever takes it:

* **Do not copy `px4_uorb_interop_e2e.rs`'s stale-tree guard.** It asserts the
  module DIRECTORY exists, and module dirs plus `bin/px4-<mod>` shims survive
  across builds while only the last root's modules are linked. Measured
  2026-09-04 on a tree built from the bridge root: `nros_uorb_demo`'s directory
  is present and the binary contains ZERO references to it, so the guard passes
  and the test then dies at `nros_uorb_demo start` — exactly the confusing
  failure its own comment says it prevents. Assert on binary CONTENT.
* An `interop::CELLS` row is blocked four ways: `BuildChannel` has no PX4
  variant, G2 admits only Linux/ZephyrNativeSim, `examples/fixtures.toml` has no
  px4 row for G5, and a `Tier::CarveOut` cannot name a test under G1.

#### W3 GATE PASSED (2026-07-31): two backends in one PX4 module

The W1-equivalent question for W3 — *can a single PX4 module hold uORB and a
networked backend at once?* — is answered yes, `rc=0`, zero undefined:

```
INFO  [nros_bridge_probe] two backends linked: uorb=0x5c09d8d53150 zenoh=0x5c09d8f4cd42
```

**Build-time RMW selection turns out to happen when the ARCHIVE is built, not in
cmake.** `nros-cpp` has `rmw-{zenoh,xrce,cyclonedds}-cffi` features, each pulling
in `rmw-cffi` — the same seam uORB registers through — so:

```sh
cargo build -p nros-cpp --no-default-features --features std,rmw-zenoh-cffi --release
```

produces one archive carrying both. The cmake layer only needs the register call,
which `BACKENDS uorb zenoh` already generates. That is a cleaner story than the
plan assumed: the outward backend is the same cargo-feature knob every other
example uses, one layer down, and `nros_px4_add_module` needs no per-backend
source lists for networked backends.

Three prerequisites, all now checked at configure time with the fixing command:

1. `zenoh-pico` must be provisioned — `nros setup --source zenoh-pico`. It is not
   part of a default checkout, and its absence surfaces as a cargo build failure
   rather than anything about PX4.
2. The archive must carry the backend, verified with the rust toolchain's
   `llvm-nm` (see below).
3. **zenoh needs a THIRD archive.** `libnros_cpp.a` has the nano-ros zenoh
   backend, but zenoh-pico's own platform layer — `z_clock_*`, `_z_condvar_*`,
   `_z_task_*`, the socket shims, **74 symbols** — lives in
   `nros-rmw-zenoh-staticlib`, built with `--features platform-posix,std`. Its
   DEFAULT feature set is bare `no_std` and fails with ``#[panic_handler]`
   function required`` before producing anything.

##### The symbol precheck had to use the right `nm`, and briefly did not

The guard that verifies (2) first used whatever `nm` was on PATH. The system nm
(binutils + LLVM 14 gold plugin) cannot parse rust-1.96/LLVM 22 bitcode members
— and does not fail cleanly. It reads the FEW non-bitcode members and reports
their symbols: it saw **18** `nros_` symbols while missing
`nros_rmw_zenoh_register` entirely, so it rejected a perfectly good archive.

A first attempt to fail-open on "nm saw nothing" does not survive a PARTIAL read.
The guard now locates `llvm-nm` through `rustc --print sysroot` and SKIPS the
check when it is absent, rather than guessing. **A guard using the wrong tool is
worse than no guard: it is wrong with authority.**

##### The two-session mechanism in C++ (found, not yet used)

The plan said `Executor::open_with_rmw`, which is the **Rust** API. The C++
equivalent is per-NODE binding via `NodeBuilder`:

```cpp
nros::init();
nros::Node in;
nros::NodeBuilder(exec_handle, "px4_bridge_in").rmw("uorb").build(in);
nros::Node out;
nros::NodeBuilder(exec_handle, "px4_bridge_out").rmw(NROS_BRIDGE_RMW).build(out);
```

`NodeBuilder::rmw(name)` selects among backends registered with
`nros_rmw_cffi_register_named`; empty picks the first-registered. The bridge must
name BOTH explicitly rather than rely on registration order — the generated
`nros_app_register_backends()` emits them in `BACKENDS` order, which is an
argument list, not a contract. `void* exec_handle` comes from
`Node::executor_handle()`.

##### Remaining for W3, and it is not plumbing

The gate is passed, the link works, and the session mechanism is identified. What
is left is the part the plan under-described: **the bridge has to TRANSLATE.**

Inward, a payload is a PX4 C struct with identity `ORB_ID(x)`. Outward, a real
ROS 2 subscriber expects **CDR** with a real type name and type hash. Those are
not the same bytes, and no amount of session plumbing makes them so — which is
the honest form of "the serialization uORB avoids comes back at the RMW
boundary", recorded under W4.3 above before any of this was built. So the bridge
needs, per topic:

1. a ROS message type on the outward side with generated C++ bindings (the
   examples get theirs from `nros generate-*` into `generated/`; a PX4 module has
   no such step today), and
2. a field-by-field map from the PX4 struct to it.

Hand-rolling the CDR is not an option worth taking: `rmw_zenoh` keys discovery on
the type hash, so a hand-encoded payload with a guessed hash would either be
invisible to ROS 2 or — worse — visible and wrong.

So the next work item is a **codegen step for PX4 modules**, then one translated
topic end to end, then the ROS 2 peer test (`nros_tests::ros2` / `ros_env` —
reuse it, do not invent a second harness), then a second backend from the same
source to show the selection is real.

**Not claimed:** zero-copy. The serialization uORB avoids returns at the RMW
boundary, necessarily. W2 demonstrates the zero-copy property; W3 demonstrates
reach. Conflating them would overclaim.

### W4 — the existing bridges: DECIDED, do NOT collapse (2026-07-31)

The observation that opened this item was mine, filed during phase-316:
`examples/bridges/tt-zenoh-to-cyclonedds` and `tt-zenoh-to-xrce` "differ only in
an outward backend the build could have chosen", which looked like the per-RMW
axis phase-316 removed from paths, surviving in a name.

**Measured, that is wrong, and the measurement is the answer.** The two `main.rs`
are ~190 lines each and differ in **151** of them. The difference is not a
backend name; it is a different setup contract:

| | XRCE egress | Cyclone egress |
| --- | --- | --- |
| type registration | lazy, from name + hash | **explicit schema staged first** — `nros_rmw::register_type_descriptor` with a `&[Field]` before the raw publisher exists |
| addressing | locator | `ROS_DOMAIN_ID` |

Cyclone resolves a topic through a registered `dds_topic_descriptor_t`, so its
bridge carries a `STRING_FIELDS` schema and a `REG_TYPE_NAME` that the XRCE one
has no use for. A collapsed bridge would need that staging `cfg`-gated per
backend — a conditional in the exact place an example is supposed to be readable.

So: **two directories, correctly.** The name `tt-zenoh-to-cyclonedds` describes
what the example IS, not a build axis it should have selected. That is the test
phase-316 set for a path level — does it name something the leaf does not? — and
here it passes.

**Why px4's bridge is genuinely different**, and why W3 still gets ONE path: its
outward side is uniform. The selected backend changes a cargo feature and a
register call, and nothing else — no schema staging, no addressing difference in
the module. When a second backend needs its own setup contract, the honest answer
is a second example; when it does not, the honest answer is one.

- [x] **W4.1** Decided: leave them. Recorded above with the measurement.

**No code change.** The value here was refusing a refactor that a surface
resemblance suggested — the two bridges look like duplicates in a directory
listing and are not in the source.

## Risks

- **W1 is the real unknown.** W2 and W3 are ordinary example code once a PX4
  module can link nano-ros; W1 is the first time anyone has tried. If it turns out
  hard, the honest move is to say so and stop — not to route around it with a
  demo that skips `nros-cpp` (which is exactly what the register-check does, and
  why this gap survived three phases).
- **Cold SITL builds are ~10 min.** Iterating on W1 means paying that repeatedly.
  Budget for it; do not shorten the loop by testing something smaller that does
  not link `nros-cpp`, because the linking IS the question.
- ~~**`just px4 test-sitl` is currently red**~~ — **cleared 2026-07-31.** Issue
  0356 resolved: `px4_e2e` removed, `test-sitl` runs Track B only and can pass.
  Track A is build-only via `build-sitl-cpp`, and `just/px4.just` +
  `examples/px4/README.md` both say so, so this phase's receipts will not be read
  against a pre-existing red.
- **Concurrent sessions.** Other agents are active; land each W in small pushed
  steps.

## Receipts to collect

| Step | Receipt |
| --- | --- |
| W0 | `just check cpp-fmt` green without touching a PX4 module; `nros_register_check help` prints a `PRINT_MODULE_DESCRIPTION` |
| W1 | PX4 SITL module links `libnros_cpp.a`; `nm` shows resolved nano-ros symbols; module starts from pxh |
| W2 | a stock PX4 consumer (`listener <topic>`) prints a message published by the nano-ros node, asserted by a test |
| W3 | a real ROS 2 subscriber receives a stock PX4 module's uORB topic through the bridge; same source builds against a second backend |
| W4 | decision recorded here before any edit to `examples/bridges/tt-zenoh-to-*` |

## Provenance

Decisions carried from phase-316 W4, recorded there on 2026-07-31 and unchanged:

- **W4.1** — the uORB example demonstrates interop with existing PX4 features; it
  skips serialization so upstream PX4 nodes understand the message format. uORB is
  the special one.
- **W4.3** — the bridge's outward side is the build-time RMW knob, not a fixed
  backend, and the far end is a real ROS 2 node.
