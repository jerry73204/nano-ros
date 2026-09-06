---
id: 1039
title: "NuttX's `stdbool.h` defines `true` as `(bool)1`, which is not an integer
  constant — every NuttX build breaks on zenoh-pico's new keyexpr template"
status: open
type: bug
area: nuttx, rmw-zenoh, build
severity: high
related: [issue-1007]
found: 2026-09-04
---

## Measured

Same tree, same commands, only the `zenoh-pico` pin differs:

| pin | `just nuttx build-fixtures-arm` |
| --- | --- |
| `fa7ad0f5b3f6` | **EXIT=0** (2026-09-04 11:04, leaves relinked) |
| `0101b80d5468` (what main records now) | **EXIT=2**, 12 errors |

The workspace path fails identically —
`nros build demo_bringup:nuttx --workspace examples/workspaces/c` gets past
preflight and dies in the same place:

```
error: failed to run custom build command for `zpico-sys v0.5.0`
  keyexpr_match_template.h:142:14: error: redefinition of '_z_chunk_special_includes'
  nros-nuttx-export-arm/include/stdbool.h:79:25: error: missing binary operator before token "1"
      note: in expansion of macro '_ZP_KE_MATCH_TEMPLATE_INTERSECTS'
```

`include/zenoh-pico/session/keyexpr_match_template.h` is NEW in the current pin
(upstream "Faster key expression matching (#1175)") and absent from
`fa7ad0f5b3f6`. It is not our file and not a stale artifact.

## Cause: NuttX's header is non-conforming, and the new template exposes it

`src/session/keyexpr.c` includes the template TWICE, which is its whole design:

```c
#define _ZP_KE_MATCH_TEMPLATE_INTERSECTS true
#include "zenoh-pico/session/keyexpr_match_template.h"
#define _ZP_KE_MATCH_TEMPLATE_INTERSECTS false
#include "zenoh-pico/session/keyexpr_match_template.h"
```

The macro is used both in `#if` and in `_ZP_CAT` to build distinct symbol names.
That is standard-conforming C, because C99/C11 §7.18 requires:

> `true` ... expands to the integer constant `1`, `false` ... to the integer
> constant `0`

NuttX's export header does not:

```c
/* third-party/nuttx/nuttx/nros-nuttx-export-arm/include/stdbool.h:79 */
#define true  (bool)1
#define false (bool)0
```

Two consequences, and both errors above are exactly them:

1. **`#if` breaks.** `#if _ZP_KE_MATCH_TEMPLATE_INTERSECTS` expands to `(bool)1`.
   In a preprocessor expression `bool` is an identifier, and unknown identifiers
   become `0`, giving `(0)1` — hence *"missing binary operator before token 1"*,
   reported inside `stdbool.h` because that is where the offending tokens live.
2. **Token pasting breaks.** `_ZP_CAT` applied to `(bool)1` and `(bool)0` cannot
   produce two distinct identifiers, so both expansions define
   `_z_chunk_special_includes` — hence *"redefinition ... previous definition
   with type `_Bool(const char *, ...)`"*.

A conforming `stdbool.h` makes both work, which is why every other platform
builds this file.

## Why it surfaced now, and what it says about the lane

This is the SECOND defect unmasked in this module today. The preflight fix
landed as `c003ae608` — the id 0999 was reserved for it and the file was never
written, so the commit is the only thing here anyone can look up. It stopped
`nros build` refusing NuttX at stage 3 for a build-std target; the build then
reaches the compiler and finds this. The
nuttx module of `lane=tier2` has therefore still never completed — for a new
reason, not the old one.

## Fix options, in the order I would try them

1. **Patch our zenoh-pico fork's template to use `1`/`0` rather than
   `true`/`false`.** Smallest and portable: an integer constant is what both the
   `#if` and the paste actually want, and it removes the dependency on any
   platform's `stdbool.h` being right. Goes on the patch line, not upstream
   `main` (CLAUDE.md's vendored-fork workflow).
2. **Fix NuttX's `stdbool.h`.** Correct at the root — the header is simply wrong
   per §7.18 — but it is a vendored third-party export, so it needs its own
   patch line and re-export, and it fixes only our copy.
3. **Define conforming `true`/`false` in the zpico shim before including
   zenoh-pico headers.** Cheapest to write and the most fragile: it papers over
   a broken libc header for one consumer and will not help the next one.

(1) also wants an upstream conversation: the template is fine by the standard,
but `1`/`0` costs upstream nothing and makes it robust against exactly this
class of non-conforming libc.

## Not verified

* Whether NuttX's own `stdbool.h` (upstream, not our export) has the same
  definition, or whether the export step introduces it.
* Whether `riscv` NuttX is affected — only `armv7a-nuttx-eabihf` was measured.
* Whether anything else in the tree relies on `true`/`false` in a preprocessor
  expression on NuttX; if so, this is one site of a class.

## Acceptance

* [ ] `just nuttx build-fixtures-arm` completes against main's zenoh-pico pin.
* [ ] The nuttx module of `lane=tier2` completes.
* [ ] The chosen fix is recorded with why, since option 1 and option 2 disagree
      about whose bug it is.


## 2026-09-04 — the stdbool half is FIXED; two more layers behind it

**Fixed:** `nros-zpico-build` now generates a conforming `true`/`false` header
into `OUT_DIR` and force-includes it (`-include`) ahead of every zenoh-pico TU on
NuttX, keyed on the target triple so all three exports are covered. Pulling
`<stdbool.h>` in first sets NuttX's `__INCLUDE_STDBOOL_H` guard, so the TU's own
later include is a no-op and cannot restore the casts. A `-D` cannot do this —
the header is included afterwards and wins.

Verified: `just nuttx build-fixtures-arm` goes from **12** keyexpr/stdbool errors
to **0**, and the generated header is present in all three nuttx cargo-fixture
target dirs, dated to the run.

A caution about HOW that was verified, because the obvious check is wrong:
grepping the build log for the `-include` flag returns nothing even when the fix
is applied, because the log carries errors, not compiler command lines. I read
that absence as "the fix never ran" and nearly retracted a working change. The
artifact on disk is the evidence; the log is not.

**Still failing, and it is NOT this issue:**

    error: linking with `arm-none-eabi-gcc` failed
    multiple definition of `_z_socket_get_endpoints'    (x6)

`src/system/common/platform.c:135` defines a fallback under

    #if !defined(ZENOH_WINDOWS) && !defined(ZENOH_LINUX) && !defined(ZENOH_MACOS)
        && !defined(ZENOH_BSD) && !defined(ZENOH_ZEPHYR)

and something else defines it too. Note the asymmetry this exposes:
`runner.rs`'s size probe defines **both** `ZENOH_NUTTX` and `ZENOH_LINUX`, with a
long comment explaining that dropping `ZENOH_LINUX` "would silently move every
NuttX image onto six code paths nothing here has ever exercised" — while
`packages/platform/nros-platform-nuttx/nros-platform.toml:22` defines only
`["ZENOH_GENERIC", "ZENOH_NUTTX"]`. Two authored copies of "what NuttX defines",
disagreeing. That is a strong lead and it deserves its own issue.

**HYPOTHESIS, EXPLICITLY UNTESTED.** I added `ZENOH_LINUX` to the manifest and
rebuilt; the result was byte-identical, so I first read it as refuted. It was
not tested at all: **`nros-platform.toml` is not a `rerun-if-changed` input**, so
cargo did not re-run the build script and the edit never reached the compiler.
The generated header's timestamp proves it — unchanged across that build, and
only regenerated once I touched `build.rs`. The change is reverted rather than
kept, and the hypothesis is open.

That last point is worth its own line: **a config file that is not a build input
changes nothing until something unrelated forces a rebuild**, which makes every
experiment on it silently void. Same family as the rest of this tree's staleness
defects.

## 2026-09-06 — filed late; where this stands on `main` today

This document was written on 2026-09-04 and never landed: it sat on
`fix/1016-lane-build-vs-run`, a branch whose pull request merged without it.
Meanwhile SEVEN references to "issue 1039" went in across `just/check.just`,
`scripts/check-zenoh-platform-macros.py` and
`packages/rmw/zenoh/nros-zpico-build/src/runner.rs`, all pointing at a file
that did not exist. `check-prose-issue-refs` (the gate #206 adds) is what
found it.

Two claims below have moved and are left in place rather than rewritten,
because the measurement is the record:

* The pin table reads `0101b80d5468 (what main records now)`. That was true on
  2026-09-04. `main` now records `dd071b8d3a14a72b9d2b96ff70d1cce1f4af1596`.
* The stdbool half is fixed on `main`, and NOT by the `runner.rs` workaround
  this document proposes. It was fixed in the vendored fork instead —
  `a1c741db`, "`== true` in a #if breaks every NuttX build — compare against
  1", which replaces the `true`/`false` template arguments with `1`/`0` at the
  call site in `src/session/keyexpr.c`. Verified: that commit is an ancestor of
  the pin `main` records today.

  Both routes are workarounds for the same non-conforming header; the fork-side
  one shipped, so the `runner.rs` force-include is redundant and is NOT being
  landed with this document.

The issue stays OPEN because three of the four boxes below are still unticked,
and the last of them — the root fix in NuttX's own header, or a documented
decision to carry the workaround — is the one that closes it.

## Revised acceptance

* [x] The stdbool/keyexpr class no longer breaks the NuttX zenoh-pico build.
* [ ] `multiple definition of _z_socket_get_endpoints` resolved (own issue).
* [ ] `nros-platform.toml` is a watched build input, so editing it takes effect.
* [ ] The root fix in NuttX's own header, or a documented decision to carry the
      workaround instead.
