---
id: 1025
title: "ESP32 flash images can never be built: the packer asks for the group dir with the row's env stripped, so it looks in a directory the build stopped using"
status: resolved
area: build, testing
severity: high
found: 2026-09-04
related: [0968, 0535, 0439, phase-340, 0393, 0181]
---

# The producer writes to a hashed group dir; the consumer asks for the unhashed one

## Reproduced

```
$ just esp32 build-qemu
Building ESP32 QEMU fixtures (from examples/fixtures.toml)...
  → examples/qemu-esp32-baremetal/rust/talker
  → examples/qemu-esp32-baremetal/rust/listener

Creating flash images...
ERROR: build/cargo-fixtures/qemu-esp32-baremetal/riscv32imc-unknown-none-elf/
       nros-relwithdebinfo/esp32_qemu_talker is missing, and nothing narrowed
       this build.
```

**The ELF is not missing.** It is one directory over, and the build had just
written it:

```
build/cargo-fixtures/qemu-esp32-baremetal-4118800323/riscv32imc-unknown-none-elf/
    nros-relwithdebinfo/esp32_qemu_talker      <- produced
build/cargo-fixtures/qemu-esp32-baremetal/…/esp32_qemu_talker  <- sought
```

So no ESP32 QEMU flash image can be produced, and every test that consumes one
fails for a reason that has nothing to do with the code under test.

## The mechanism

`nros_fixture_group_slug` keys the shared cargo target dir on the row's VARIANT
— its cargo args plus its env. A row with no variant gets the bare platform
name; a row with one gets `<platform>-<cksum>`.

`just esp32 build-qemu` packs the ELF, and asks where it landed:

```sh
artifact_dir="$(nros_fixture_row_artifact_dir \
    "examples/qemu-esp32-baremetal/rust/$ex" qemu-esp32-baremetal "" "")"
```

The last two arguments are the cargo args and the env — passed EMPTY. The
producer passes the row's real ones. Same function, two different keys.

## What made it live: a fix-the-class commit, one class down

`41a7d8de7` (2026-08-31 02:31 UTC), *"fix(esp32): the DRAM fix landed on one row;
its three siblings kept overflowing"*, added to the three esp32 rows:

```toml
env = { ZPICO_MAX_QUERYABLES = "2" }
```

That is a correct fix for a real DRAM overflow, and its commit message is about
applying a fix to a whole class rather than one reported site. But env IS the
variant signature, so adding it moved the producer from `qemu-esp32-baremetal`
to `qemu-esp32-baremetal-4118800323` — and the consumer, which names no env,
stayed where it was. Before that commit both sides agreed on the bare name and
the packer worked.

## It is a RECURRENCE, and the helper that exists to prevent it names this site

`nros_fixture_row_artifact_dir`'s own docstring is about this exact failure at
this exact call site:

> phase-340 item 7 — this exists because `just esp32 build-qemu` hand-wrote
> `examples/qemu-esp32-baremetal/rust/$ex/target/...` to find the ELF it packs
> into a flash image. Migrating the platform redirected the BUILD and left that
> path pointing at nothing […] Every gate passed while that happened, which is
> #393's failure mode […] A post-processing step that spells the artifact path
> by hand is a second derivation of the group key; this makes it one.

The call site was duly converted to the helper. Passing `"" ""` re-creates the
second derivation anyway — the key is a function of (platform, args, env), and
supplying two of the three constants a different answer. The helper made the
FORMULA single; the INPUTS are still derived twice, and that is where it
diverged.

## Why nothing caught it

Same reason as [issue 0968](../0968-tier2-runtime-failures-unreproduced.md), whose
step 1 is what surfaced this: tier-2 fixture builds do not run in CI
(`post-submit`'s job is interlocked on an unset `vars.NROS_SELF_HOSTED_READY`,
and `host-tests` has been red on 0967). This is a BUILD failure that a build
lane would have caught on the day.

The `skip` arm cannot mask it and correctly did not: issue 0439 already made the
"absent ELF" case loud unless `NROS_FIXTURE_COORDS` narrowed the build, which is
why this reports rather than exits 0.

## The fix, scoped

Do NOT hardcode `ZPICO_MAX_QUERYABLES=2` at the call site — that is a THIRD
derivation of the row's variant and would break again the next time a row's env
moves.

The consumer should ask the MANIFEST for the row it is packing, by row id
(`qemu-esp32-baremetal-talker` / `-listener`, which the build loop directly
above already names). `scripts/build/fixtures-manifest.py` already owns "THE
single computation of a row's artifact root" in `row_artifact_root`, but it
returns the leaf-relative `<dir>/<target_dir or "target">` rather than the
shared group dir, so it needs to learn the group case before it can answer this.
That is the change, and it belongs with a test that builds the image.

**Acceptance is a BUILD, never a gate** — the helper's own docstring says so
about this same site, and a gate is what passed last time.

## Bearing on issue 0968

0968 lists twelve unreproduced tier-2 runtime failures; FIVE are esp32
(`test_esp32_talker_listener_e2e`, `test_esp32_workspace_entry_e2e`,
`test_esp32_to_native`, `test_native_to_esp32`, and
`logging_smoke_esp32_qemu_emits_every_severity`). All five need a flash image
that cannot currently be produced.

That is a plausible single cause for the esp32 cluster and it is NOT yet
established as their cause: this issue reproduces the BUILD failure, not those
tests' failures. Confirming it means fixing this, rebuilding, and re-running the
five. Recorded that way deliberately — 0968 exists because four issues were
filed here from a sweep and all four retracted, two carrying confident wrong
root causes.


## RESOLVED 2026-09-04

**`nros_fixture_row_artifact_dir_by_id <row-id> <platform>`** — the row's args
and env come from the manifest, which is the one place they are written. The
four-argument form stays for callers that genuinely hold them (the build
itself). Both ESP32 packers now name the row id they are packing, which is the
same id the build loop above each already passes to `fixtures-build.sh --id`.

**Acceptance is the build, as the helper's own docstring demands:**

```
$ just esp32 build-qemu
  build/esp32-qemu/esp32-qemu-talker.bin      Image successfully saved!
  build/esp32-qemu/esp32-qemu-listener.bin    Image successfully saved!
$ just esp32 build-logging-smoke
  …/logging-smoke-esp32-qemu.bin              Image successfully saved!
```

### The class, swept — and two of the five flags were the GATE being wrong

`check-fixture-artifact-dir-inputs` (fast line) forbids a packer of a
lane-built artifact from supplying empty args or env on a shared platform. Its
first version flagged six sites. Only the two ESP32 ones were defects.

The other four — `freertos`, `nuttx`, `qemu-arm-baremetal`, `threadx-linux`,
`threadx-riscv64` run-example recipes — **build the artifact themselves**, and
pass the very same empty arguments to `nros_fixture_target_dir_flag`. Their two
halves therefore agree with each other: they write and read the same unhashed
dir. Measured, not assumed — `nuttx` and `threadx-riscv64` DO diverge from the
fixture lane's key (`nuttx-2965781523` vs `nuttx`), which is what makes them look
like defects, and they are not, because nothing in those recipes reads what the
lane built.

Shipping that first version would have reported a five-site bug that does not
exist. The gate now exempts a call whose recipe also calls
`nros_fixture_target_dir_flag` — a gate wider than the rule it enforces is issue
0196's audit finding, and this one was about to repeat it.

**Mutation-tested on the normal path**, per `check-gate-selftests`: the gate
runs three controls every time — the real 1025 defect (must flag), the by-id fix
(must not), and a self-consistent build-it-yourself recipe (must not). The third
is what pins the narrowing, so a future edit cannot quietly widen it back.

### What it does NOT establish

Whether the five esp32 tests in [issue 0968](../0968-tier2-runtime-failures-unreproduced.md)
pass now. They could not run at all before this; that they CAN run is not the
same as their passing, and running them is 0968's work.
