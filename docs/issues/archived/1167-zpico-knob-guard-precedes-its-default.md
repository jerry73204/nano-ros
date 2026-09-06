---
id: 1167
title: "`#if ZPICO_MAX_SESSIONS < 1` sits ABOVE the knob's default, so the guard issue
  1015 added refuses every Zephyr zenoh build instead of protecting the array — and
  no gate asks whether a guard can fire"
status: resolved
type: bug
area: [rmw, build, embedded]
severity: high
found: 2026-09-06
related: [1015, 0460, 1029]
---

## What

Issue 1015 gave every knob-sized C array in `zpico.c` a compile-time refusal:

```c
#if ZPICO_MAX_SESSIONS < 1
#error "ZPICO_MAX_SESSIONS must be >= 1: it sizes a fixed C array (issue 1015)"
#endif
```

The C preprocessor evaluates an **undefined** identifier in `#if` as `0`. Nine
of the ten guards were safe because the knob's `#ifndef`/`#define` default sits
in the defaults block above them. `ZPICO_MAX_SESSIONS`'s default was ~100 lines
**below**:

| knob | guard | default | |
| --- | ---: | ---: | --- |
| `ZPICO_MAX_QUERYABLES` | 373 | 205 | ok |
| `ZPICO_MAX_LIVELINESS` | 376 | 208 | ok |
| `ZPICO_MAX_PENDING_GETS` | 379 | 247 | ok |
| `ZPICO_MAX_PENDING_REPLIES` | 382 | 265 | ok |
| **`ZPICO_MAX_SESSIONS`** | **385** | **484** | **guard first** |
| `ZPICO_GET_REPLY_BUF_SIZE` | 388 | 241 | ok |
| `ZPICO_GRAPH_CACHE_SIZE` | 391 | 226 | ok |
| `ZPICO_ZID_SIZE` | 394 | `zpico.h:28` | ok |
| `ZPICO_MAX_SUBSCRIBERS` | 397 | 202 | ok |
| `ZPICO_MAX_PUBLISHERS` | 400 | 199 | ok |

## Why it is exactly this knob that broke

`ZPICO_MAX_SESSIONS` is the one guarded knob with **no Kconfig row and no cmake
bridge** — `nros-zpico-build/src/runner.rs` says so in as many words ("A knob
with no `KCONFIG_KNOBS` row (`ZPICO_MAX_SESSIONS`, …)"). Every other guarded
knob arrives as a `-D` on the Zephyr C lane's command line, so the guard reads a
real number whatever the ordering. This one does not, so the in-file default was
the only definition the TU would ever get — and it came too late.

The two facts are independently correct and only wrong together, which is why
reading the diff does not catch it.

## Measured

`west build -b mps2_an385 examples/zephyr/rust/talker` (zenoh):

```
FAILED: modules/nros/CMakeFiles/nros.dir/…/zpico.c.obj
zpico.c:386:2: error: #error "ZPICO_MAX_SESSIONS must be >= 1: it sizes a fixed C array (issue 1015)"
```

The compile line carries `-DZPICO_MAX_QUERYABLES=8`, `-DZPICO_MAX_PUBLISHERS=8`,
`-DZPICO_MAX_SUBSCRIBERS=8`, `-DZPICO_MAX_LIVELINESS=16` … and no
`-DZPICO_MAX_SESSIONS`.

## Why CI did not catch it

No merge-gating event builds Zephyr. The Zephyr jobs live in `nightly.yml` and
are gated on `needs.changes.outputs.run_zephyr`; the tier-2 lane that would have
built a Zephyr image is issue 1158 (no runtime verdict in six consecutive runs),
and the one cortex-m Zephyr build that did run on 2026-09-06 died earlier, on
the cross-checkout fault of issue 1166. Three independent gaps, and the guard
landed through all three.

## Resolution — fixed TWICE, independently, and only one half is here

Two sessions hit this within hours and fixed it in opposite directions:

* **`91f3ce7c9` (`fix(#1015)`) moved the GUARD down**, below the `#define` at
  485. That landed on `main` first and is the fix that ships. It was found from
  the other end — `just setup tier2` broke on the merge-queue lane "the moment
  that lane started completing runs again", which is issue 1158 clearing up
  enough to surface it.
* This branch moved the **DEFINE up**, found by trying to build a Zephyr image
  for issue 1145.

Both are correct; the code change here was dropped as superseded rather than
merged, because rebasing produced a file where main's comment ("`ZPICO_MAX_SESSIONS`
is defined further down the file") had been made false by the other fix. Two
correct edits composing into a lying comment is worth more care than a clean
auto-merge suggests.

**What this issue contributes is the GATE, which `main` does not have.**
`91f3ce7c9`'s own commit message says so:

> `check-c-array-pool-floors` still reports 8 guarded / 3 zero-legal / 10
> unclassified: it keys on the presence of a guard, not on whether the guard can
> fire, so it was green through all of this. That gap is real and is not fixed
> here.

**Gate: `check-c-knob-guard-order`** — for every `#if KNOB < 1` guard in a
tracked `.c`/`.h`, the knob must be defined earlier in the same file, or
unconditionally by a tracked header under the component's `include/`. Buildless,
`git ls-files` rather than a filesystem walk (issues 1157/1166 — and the first
draft used `rglob`, which `check-no-tracked-file-find` caught on the pre-push
hook), self-testing with six cases on the normal path.

Mutation-tested against **main's** fix, not against this branch's: putting the
guard back above the define turns the gate red naming the exact site and line,
and restoring it turns it green. That ordering matters — the first mutation run
here reported a false green twice over, once because the working file was not
actually main's and once because piping the gate through `tail` replaced its
exit code with `tail`'s.

## The class, and what is NOT claimed

The rule is about ORDER, and it generalises past this file — any
`#if X < 1`/`#error` pair added above its own default has it. The gate covers
every tracked C source, so a second occurrence anywhere is caught.

Not claimed: that the other nine guards were ever wrong (they were not), or that
`ZPICO_MAX_SESSIONS` should gain a Kconfig row. Whether it should is
phase-392 W5's open question about this knob, and it is a separate decision.
