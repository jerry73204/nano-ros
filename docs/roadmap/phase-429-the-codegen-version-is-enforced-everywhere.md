# Phase 429 — the codegen version is enforced everywhere

**Status (2026-09-06). Every work item is landed or resolved.** W1, W2, W3, W5
and W7 shipped; W4 and W6 closed as "already true" and "smaller than filed", each
recording the measurement that settled it, because both were written from a guess
I did not check first.

The token exists and every generated artifact carries it. C and C++ refuse at
COMPILE time through the preprocessor (the first implementation used a weak
anchor symbol and was replaced: this project avoids those, and the check turned
out to decompose into one new check plus `nros_config_variant_<slug>`, which
already existed). Rust refuses under `cargo check`. The ratchet holds the
authored integer to its surface.

`check-codegen-version-refusal` proves all of it fires, in **seven arms across
three languages** — in range compiles, out of range is refused, and a config
header that defines neither bound is refused as MISSING rather than as
out-of-range. The Rust arm arrived last and is a correction: this phase first
recorded it as unreachable, citing CLAUDE.md's ban on compiling inside tests.
That ban is on compiling at TEST RUNTIME and a negative control is a GATE —
`check-c` has compiled an expected-failure probe for years. The option was never
closed; the reason given for skipping it was wrong.

**Not done, and it is the point of all of it:** shipping a prebuilt binary. The
correctness question that blocked it since phase-287/288 — *can this binary's
output work with this runtime?* — now has an answer asserted at every layer and a
negative control proving each refusal fires. The remaining work is distribution,
and it has its own home:
[phase-431](phase-431-ship-the-nros-binary.md).

**Implements:** [RFC-0090](../design/0090-codegen-version-is-the-compatibility-token.md).
**Closes:** the residue of [#1018](../issues/1018-a-codegen-change-invalidates-generated-interfaces-and-only-a-manual-step-connects-them.md).
**Unblocks:** shipping a prebuilt `nros` binary — [#0171](../issues/archived/0171-no-external-distribution-path.md)'s
"single highest-leverage unlock", which has been blocked since phase-287/288 on
exactly this question.

## Why this phase exists

nano-ros shipped prebuilt `nros` binaries early and stopped, because a released
binary emitted code that had drifted from the runtime. Drifted generated code
*compiles*; the image is simply wrong. So the distribution path — the thing that
would end the `git clone` + `just setup-cli` bootstrap — is blocked on one
missing answer: **can this binary's output work with this runtime?**

Today the only mechanism that asks anything adjacent is the in-tree stale-CLI
refusal, and it asks the wrong question (`binary ↔ its own sources`) in a way
that both over-fires and cannot ship. RFC-0090 §Motivation has the argument.

## The user workflow is the thing being protected

```
obtain the CLI   ->   nros setup   ->   nros build      (codegen runs inside)
```

The CLI is step 0 for everything and is also what makes cloning the monorepo
unnecessary. **CI simulates these steps**, so every work item below lands with
the workflow step that exercises it — a check whose only proof is a unit test is
a check nobody has evidence still works on the path users take.

## Work items

### W1 — the token exists, and every artifact carries it

* `nros_core::NROS_CODEGEN_VERSION` and `NROS_CODEGEN_VERSION_MIN`, both `u32`,
  both `1` at landing. One authored home; nothing derives them.
* Every emitter stamps `G` into what it writes. **The emit surface is narrower
  than the pack list suggests** (surveyed 2026-09-05):
  * The live Rust emitter is `packs/nros/*`. `packs/rust/` and `packs/rmw/` are
    the retired two-layer rclrs-style backend, reachable only from benches and
    tests — do not stamp them.
  * A generated Rust crate's scaffolding is **hand-written Rust, not a
    template**: `rosidl-bindgen/src/generator.rs` (`generate_lib_rs`,
    `generate_cargo_toml`, the three `mod.rs` emitters, `write_bound_inventory`).
    The token goes in `src/lib.rs`, which is emitter output — NOT `Cargo.toml`,
    because the six committed `packages/interfaces/*/generated/**/Cargo.toml`
    are hand-maintained and in three mutually inconsistent shapes.
  * **C has one guaranteed carrier and C++ does not.** Every generated C
    artifact includes `<nros/types.h>` unconditionally; the C++ templates
    include different `nros/*.hpp` sets for message vs service vs action, so the
    C++ carrier is a deliberate choice (the per-package umbrella
    `<pkg>/<pkg>.hpp`, or a new shared include).
* **Mirror `bounds.rs`.** `rosidl-codegen/src/bounds.rs:55` already does exactly
  this one axis over: `INVENTORY_SCHEMA_VERSION: u32 = 1`, one SSoT constant,
  three transports (JSON, cargo `links` metadata, CMake), and the rule
  *"a consumer that does not recognise the version must refuse, never guess."*
  Same integer, same discipline, same refusal.
* Assertions, per RFC-0090 D3:
  * **Rust** — a `const` block in the generated crate against
    `nros_core::NROS_CODEGEN_VERSION`, modelled on
    `nros_node::format_check::assert_message_format`. Remember its caveat: an
    inline `const` in a generic fn is evaluated by the monomorphisation
    collector, so it is a `cargo build` error and `cargo check` does not see it.
  * **C / C++** — the anchor-symbol trick from
    `nros-build-helpers/csrc/variant_symbol.c`. The runtime defines one weak
    `nros_codegen_version_v<K>` for **every** `K` in `[MIN, N]`; a generated
    header references `nros_codegen_version_v<G>`. The accepted range is
    therefore expressed as a SET OF DEFINED SYMBOLS and there is no comparison
    logic anywhere that could itself be wrong. Weak for the reason
    `variant_symbol.c` already is: a mixed C+C++ image links two `nros-c` builds
    and N identical weak definitions merge where two strong ones would collide.

**Acceptance.** A hand-edited `G` in a generated tree fails the build, in each
of Rust / C / C++, with a message naming the version. Negative control: the
unedited tree builds.

### W2 — the binary declares what it emits, and refuses early

**This is a re-token of `abi_guard`, not a new mechanism** (RFC-0090 D1a).
`nros-cli-core/src/abi_guard.rs` already resolves a version, compares it, and
exits with an actionable message; it compares the wrong thing (the CLI's
`CARGO_PKG_VERSION` against `nros-core`'s, read out of a `Cargo.lock` a C/C++
user does not have) and from too few verbs.

* `versions_match` compares **codegen versions**; the value comes from the
  artifacts, not from a lockfile. Keep `NROS_SKIP_VERSION_CHECK` and the
  visible-bypass warning.
* `nros --codegen-version` prints `E`; `nros version` gains a line.
* **Widen the call sites.** Today: `generate`, `generate-rust`, `codegen`,
  `codegen-system` (`cmd/generate.rs:150,171`, `cmd/codegen.rs:173,192`). Add
  `build` — RFC-0065's front door, currently unguarded — and `sync`.
* The check fires **before emitting anything**. A sentence at the start, never
  an undefined symbol twenty minutes later.
* Placement matters and is measured: a check in `nros setup` is already reached
  by `queue.yml`, `build-wide.yml`, `run-matrix.yml`, `host-tests.yml`,
  `gate.yml` and `post-submit.yml`; a check in `scripts/bootstrap.sh` is reached
  **only** by the nightly bootstrap probe. Put it where the lanes already go.
* Where `[MIN, N]` comes from for a user project with no monorepo is part of
  this item — the provisioned runtime is the source, per RFC-0090 D7.

**Acceptance.** A binary whose `E` is forced out of range refuses at `nros
build` start, before any file is written, and the message names how to recover.
A C/C++-only consumer with no `Cargo.lock` is covered — the case `abi_guard`
silently skips today.

### W3 — the level cannot silently go stale (the ratchet)

An authored number drifts. Gate: a committed hash of the **version surface** —
the runtime items generated code names — plus a check that fails when the hash
moves and `NROS_CODEGEN_VERSION` does not. Baseline lives in `.config/`, beside
the other ratchets. Buildless, self-testing on the normal path
(`check-gate-selftests` requires it).

The hash is the gate's evidence and is **never** compared at build time; it is
not the token. RFC-0090 D1/D5.

**Acceptance.** Mutating the surface without bumping turns the gate red and
names what moved; bumping clears it. Both directions exercised in the gate's own
self-test.

### W4 — freshness stops being a stop

`codegen_fingerprint()` answers "would this binary emit different bytes"; the
version answers "can this code work with this runtime". Conflating them is why
an ordinary edit and a genuine incompatibility produce the same refusal.

The CLI owns codegen, so a fingerprint difference is **regenerated**, not
reported. Phase-424 already put the fingerprint in the `generated/` stamp
(`scripts/build/codegen-stamp.sh`), which is the detector; this item makes the
response automatic.

**SATISFIED, and mostly before this phase opened** (recorded 2026-09-05).
`nros_codegen_stamp_check_or_wipe` already wipes on drift and lets `nros sync`
repopulate — it prints `codegen-stamp: drift detected … — wiping (will regen)`
and returns 0. It never stopped anyone; what it could not do was NOTICE an
emitter edit, because its watch set was one file in `nros-core`. Phase-424
supplied the missing detection by hashing `nros codegen-fingerprint` into that
stamp.

So this item is the composition of two things that already exist, and the work
here is to say so rather than to build a third. **Acceptance met:** editing an
emitter, rebuilding the CLI, and running a consumer build regenerates with a
progress line and no refusal.

### W5 — CI exercises the user path, including the failures

The workflows stand in for the user, so they must run the version path and the
refusals, not only the happy build.

* The pristine-host bootstrap probe covers the real order (obtain CLI → setup →
  build).
* **Negative controls, which are the point:** a job that forces `E` out of range
  and asserts `nros build` refuses at the start; a job that hand-edits `G` in a
  generated tree and asserts the Rust / C / C++ assertion fires. A uniformly
  green lane proves nothing about a check that has never fired in it.
* **A CORRECTION worth keeping (2026-09-06).** This item first shipped covering
  C and C++ only, recording the Rust half as unreachable because "there is no
  `trybuild` and CLAUDE.md bans compiling inside tests". The second clause is
  true and irrelevant: the ban is on compiling at TEST RUNTIME, and a negative
  control is a GATE. `check-c` has compiled an expected-failure probe
  (`serialization_format_mismatch_probe.c`) for years — the same shape, one
  language over. Arm D builds a scratch crate carrying a bad token and requires
  `error[E0080]`; mutation-verified by raising `NROS_CODEGEN_VERSION` so the
  out-of-range probe becomes acceptable, which turns the arm red. A scratch
  crate rather than a tracked file, because the assertion under test lives in
  EMITTED code and a hand-written probe could drift from it silently.
* **The CLI cache key is narrower than the freshness stamp, and that is the
  shipped-binary bug reproduced inside our own CI.** Seven `actions/cache`
  blocks share one key:

  ```
  nros-cli-${{ runner.os }}-${{ hashFiles('packages/cli/Cargo.lock',
      'packages/cli/**/Cargo.toml', 'packages/cli/**/*.rs') }}
  ```

  `source_stamp.rs` additionally watches `.jinja` templates, `askama.toml`,
  `cli-source-dirs.txt`, the out-of-tree crate dirs that file lists, and the
  `play_launch` pin. So an EXACT-key hit exists for a tree whose stamp has
  moved — a template edit changes no key input. A fresh checkout's mtimes
  usually make cargo rebuild anyway, which is a mitigation and not a guarantee.
  Either widen the key to the stamp's inputs or ask the binary
  (`nros source-stamp`) after restoring; do not rely on mtimes.
* Of the five CLI-acquisition mechanisms in CI, only `just setup-cli` (via
  `gate.yml`'s `scaffold-journey`) and `post-submit.yml`'s `just check
  cli-fresh` consult the stamp at all. The raw `cargo build --release … --bin
  nros` steps in `gate.yml`, `host-tests.yml` (×2) and
  `.github/actions/setup-nros-cli` never ask.

**Acceptance.** Each negative control fails when its check is reverted.

### W6 — the stale-CLI refusal becomes developer-only

**SCOPE CORRECTED 2026-09-05, after measuring.** This item first said a stale CLI
would be *rebuilt* instead of refused. That is wrong twice over, and RFC-0090 D6
now records why: auto-rebuild is banned by the refusal's own rule (*"compiling at
build/test time is forbidden"*, and it is issue 1018's rejected option 1), and
narrowing the watch set is rejected by issue 0604's measurement that a
hand-rolled closure walk was wrong in both directions.

What is left is real but smaller:

* **Remove watch-set entries that provably cannot affect an emitted byte.** One
  existed and is gone — `rosidl-codegen/templates/`, a byte-identical duplicate
  of `packs/scaffold/` referenced from no `.rs`. Proof is the pair of
  measurements: the codegen fingerprint did not move (nothing emitted changed)
  and the source stamp did (the refusal watches five fewer files).
* **Check the other two reported stops rather than assuming them false.** Both
  were, and both are CORRECT:
  * the `play_launch` pin is a genuine CLI build input (`build.rs` bakes
    `NROS_PLAY_LAUNCH_SHA`; issue 0561 records a pin move that left the stamp
    unchanged while `setup-cli` reported success);
  * `cmd/doctor.rs` is compiled into the binary, so it does change it. The stamp
    asks "does this binary match its sources" and answers correctly.
* **Record who pays.** A user with a released binary has no CLI sources, so
  `checkout_root_of` cannot match and the guard cannot fire for them.

**Acceptance.** The refusal cannot fire for a binary outside a checkout, and no
watch-set entry remains that cannot affect an emitted byte.

**A free narrowing, found while surveying.**
`packages/cli/rosidl-codegen/templates/` is a **byte-identical duplicate** of
`packs/scaffold/` (verified with `diff -q`) referenced from no `.rs` file. It is
dead as an emit input — but `source_stamp.rs` scans `.jinja`, so editing it
moves the stamp and stops every consumer build, while `codegen_fingerprint`
(which hashes `bundled_packs()`) correctly ignores it. That is a watch-set entry
that **provably cannot affect an emitted byte**. Deleting it narrows the refusal
with no judgement call and no authored partition — which is precisely what
issue 0604 says a closure walk must never require.

**Acceptance.** Touching a CLI source and running a consumer build in a dev
checkout completes, having rebuilt the CLI. The same in a tree with no CLI
sources does not attempt a rebuild.

### W7 — the docs say the new order

Book + `AGENTS.md` + `CLAUDE.md`'s one-liner: the user obtains a binary, and
`just setup-cli` is named for what it is — the contributor path. RFC-0014 and
RFC-0065 get the amendment pointer.

## What this phase must not do

**Do not make the version a hash.** RFC-0090 D1 — a hash cannot say which side
is behind, and the difference between "regenerate silently" and "a human must
move a pin" is the entire value of the token. It also moves on a doc comment,
and a check that cries wolf is a check users disable.

**Do not reuse `codegen_fingerprint()` as the compatibility key.** Same
document, D4. It is an emitter identity and it is already correctly employed as
one.

**Do not widen the stale-CLI refusal's watch set to compensate.** It is being
retired from correctness duty, not reinforced.

**Do not ship a binary before W1–W5.** That ordering is the whole point: the
release is what W1–W5 make safe.

## Acceptance for the phase

* Every generated artifact carries `G`, and each language's assertion has been
  observed to FIRE, not merely to exist.
* `E ∉ [MIN, N]` refuses before emitting.
* The ratchet is red on an unbumped surface change.
* A fingerprint difference regenerates without stopping anyone.
* CI runs the user order and both negative controls.
* #1018's two reported stops no longer stop a developer.
