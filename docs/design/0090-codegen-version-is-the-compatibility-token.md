---
rfc: 0090
title: "The codegen version is the compatibility token, and generated code carries it"
status: Draft
since: 2026-09
last-reviewed: 2026-09
implements-tracked-by: [phase-429]
supersedes: []
superseded-by: null
---

# RFC-0090 — The codegen version is the compatibility token, and generated code carries it

**Amends:** [RFC-0014](0014-nros-setup-toolchain-management.md) (`nros setup`
provisions a runtime that must match the binary asking for it),
[RFC-0023](0023-codegen-workspace-discovery.md) and
[RFC-0068](0068-language-neutral-codegen-ir.md) (every emitter now stamps what
it emitted), [RFC-0065](0065-colcon-like-workspace-builder.md) (`nros build`
refuses before emitting rather than after linking).
**Relates to:** [RFC-0088](0088-serialization-format-is-a-compile-time-provider.md)
— the same move one axis over: a cross-artifact agreement asserted at compile
time instead of hoped for.

## Summary

Generated code and the runtime it is compiled against must agree, and until now
nothing said so. The `nros` binary that emitted the code was not asked, the
generated tree recorded nothing, and the runtime accepted whatever it was
handed. A shipped binary whose emitters had moved produced code that compiled
against a newer runtime and was wrong — silently.

This RFC introduces **one authored integer**, the *codegen version*. The runtime
declares the range it accepts, the binary declares what it emits, and every
generated artifact carries what it was emitted at. Disagreement is a compile or
link error at the artifact, and a refusal-with-a-sentence at `nros build`
before a byte is emitted.

## Motivation / problem

### What actually broke

nano-ros shipped prebuilt `nros` binaries early on and stopped, because a
released binary emitted code that had drifted from the runtime. The failure is
not loud: drifted generated code *compiles*. It produces an image that is wrong
in the way the generator was wrong — the transposed `string[]` dimensions that
issue 1018 records (`char data[256][64]` for `[64][256]`) are the canonical
example, and they were caught by a developer's build refusal, not by anything a
user would have.

The relation that breaks is **generated code ↔ runtime**. The binary is only
the thing that produced one side of it.

### What guards it today, and why that is not enough

One mechanism: the in-tree stale-CLI refusal
(`nros-cli-core/src/stale_guard.rs`, issue 0363), which compares the CLI binary
against its own *sources*.

It guards `binary ↔ repo-sources`, a proxy for the relation that matters, and
it fails in both directions:

* **It over-fires.** Its watch set is the whole CLI closure, so appending a
  comment to `cmd/doctor.rs` — a file that cannot change an emitted byte —
  stops every consumer build. Issue 1018 is three such stops in one afternoon,
  one of them from nothing but moving a submodule pin forward.
* **It cannot ship.** A user with a downloaded binary has no CLI sources, so
  the guard cannot exist for them. The one boundary where drift is most likely
  — a binary downloaded months ago — is the one with no check at all.

The consequence is structural: because the refusal is the *only* guard, it must
stay conservative, and because it is conservative it over-fires. Neither half
can be fixed while it is alone.

### Why the bootstrap makes this urgent

`nros` is rosdep and colcon for nano-ros: it provisions prerequisites and builds
the workspace. The intended user order is

```
obtain the CLI  ->  nros setup  ->  nros build   (codegen runs inside)
```

The CLI is step 0 for everything, and it is also what makes cloning the monorepo
unnecessary. Today obtaining it *requires* the monorepo plus `just`
(`just setup-cli`) — step 0 depends on the thing it exists to eliminate. That
workaround is what puts a developer-only refusal on the user path.

Shipping the binary removes the circularity, and it is blocked on exactly one
thing: an answer to *"can this binary's output work with this runtime?"* that
does not depend on having the sources.

## Design

### D1 — The token is an authored integer, not a hash

```
NROS_CODEGEN_VERSION        the level the runtime emits/accepts
NROS_CODEGEN_VERSION_MIN    the oldest level the runtime still accepts
```

A monotone integer, bumped **deliberately** when the interface between
generated code and the runtime changes: a trait signature generated code
implements, a symbol name it defines, a layout rule it obeys.

Not a content hash, and the reasons are not aesthetic:

* **Direction.** A hash says "different". An integer says *which side is
  behind*, and that is the whole difference between "regenerate, silently" and
  "a human must move a pin". The remedies are not interchangeable.
* **Cosmetic stability.** A hash over the surface moves when a doc comment
  moves. A check that fires on cosmetic change is a check users learn to ignore,
  and then there is no check.

A hash still exists, in D5, as the ratchet's evidence — never compared at build
time.

### D1a — This replaces `abi_guard`'s RULE, not its call sites

A version guard already exists:
[`nros-cli-core/src/abi_guard.rs`](../../packages/cli/nros-cli-core/src/abi_guard.rs)
(phase-218.E). It resolves the consumer's authoritative `Cargo.lock`, reads the
resolved `nros-core` version, and compares it to the CLI's own
`CARGO_PKG_VERSION` under **strict SemVer equality**. Its own header anticipates
this RFC almost exactly:

> A future refactor may split the "CLI version" from the "runtime ABI version"
> […]; the call sites here would not change.

This is that refactor. The mechanism, the error shape and the opt-out are right
and are kept. Three things about it are wrong, and each is a reason the token
has to change rather than the plumbing:

1. **The token is a release version.** Two releases can emit identical code and
   two builds of one version need not. Strict equality therefore fires on every
   release bump that changed nothing an emitter does, which is a check people
   learn to bypass — and `NROS_SKIP_VERSION_CHECK` is right there.
2. **It reads a `Cargo.lock`.** A C or C++ user has no `Cargo.lock`, and a
   consumer that has not generated one yet is warned-and-continued. So the guard
   is silently absent for exactly the users a prebuilt binary is for.
3. **It does not cover the verbs that matter now.** Call sites are
   `generate`, `generate-rust`, `codegen` and `codegen-system`. `nros build`
   — RFC-0065's front door — is not among them, and neither are `sync` or
   `setup`.

So: `versions_match` compares codegen versions, the value comes from the
artifacts rather than from a lockfile, and the call sites widen to every verb
that emits or consumes generated code.

### D2 — Three parties, and the rules between them

| party | carries | where it lives |
| --- | --- | --- |
| the `nros` binary | `E` — the level it emits | compiled in |
| a generated artifact | `G` — the level it was emitted at | in the artifact |
| the runtime | `[MIN, N]` — what it accepts | `nros_core` |

1. **`E ∉ [MIN, N]`** → `nros build` refuses **at the start**, before emitting
   anything. This is the shipped-binary case, and it is the one that must never
   be a link error twenty minutes later. The message names the remedy: get a
   newer `nros` — which, because `nros` provisions, it can do itself.
2. **`G ∉ [MIN, N]`** → refuse, naming the direction. `G < MIN`: regenerate.
   `G > N`: the runtime is older than the code; only a human can decide whether
   to move the pin or the binary.
3. **`G ≠ E`, both in range** → **not an error.** Regenerate. This is freshness,
   and it must never stop anyone.

`MIN` buys a deprecation window without a second version axis: one number to
bump, and one number to raise when support is dropped.

### D3 — Enforcement is everywhere, asserted where a mismatch would be silent

Every generated artifact carries `G`, and every consumer asserts it. Two
mechanisms, both already load-bearing in this tree:

* **Rust** — a `const` block in the generated crate comparing its `G` against
  `nros_core::NROS_CODEGEN_VERSION`. This is
  [`nros_node::format_check`](../../packages/core/nros-node/src/format_check.rs)'s
  mechanism (RFC-0088): evaluated during monomorphisation, no runtime branch,
  and it names the offending type at the call site. Note its own caveat — an
  inline `const` block in a generic function is evaluated by the monomorphisation
  collector, so `cargo check` does not see it and `cargo build` does.
* **C / C++** — the anchor-symbol trick from
  `nros-build-helpers/csrc/variant_symbol.c` (issues 0360/0369). The generated
  header references `nros_codegen_version_v<N>`; the runtime defines exactly
  one. A mismatch is an undefined symbol, and *the symbol name is the whole
  payload* — there is no comparison logic that can itself be wrong. The anchor
  is `__attribute__((weak))` for the reason `variant_symbol.c` already is: a
  mixed C+C++ image links two `nros-c` builds and N identical weak definitions
  merge where two strong ones would collide.

### D4 — Freshness and compatibility are different questions with different remedies

They are conflated today, which is why an ordinary edit and a genuine
incompatibility produce the same stop.

| | detected by | remedy | who acts |
| --- | --- | --- | --- |
| **freshness** | `codegen_fingerprint()` differs from the recorded stamp | regenerate | the CLI, silently |
| **compatibility** | `G` or `E` out of `[MIN, N]` | new binary, or move a pin | the CLI, or a human |

`codegen_fingerprint()` (`rosidl-codegen/src/fingerprint.rs`) hashes every byte
the emitters produce for a compiled-in corpus plus every bundled pack. It is the
right key for *"would this binary emit different bytes?"* and the wrong key for
*"can this code work with this runtime?"* — it moves for a comment in a
template. Phase-424 already put it where it belongs: the `generated/`
regeneration stamp.

The CLI owns codegen, so a freshness difference is regenerated rather than
reported. The only surviving stop is a real version mismatch — which is exactly
the case where stopping is correct.

### D5 — The level cannot silently go stale

An authored number drifts; this repo has measured that (CLAUDE.md's own note
that the api-parity map "is AUTHORED, so it drifts when slots move"). So the
bump is ratcheted: a committed hash of the **version surface** — the runtime
items generated code names — and a gate that fails when that hash moves and
`NROS_CODEGEN_VERSION` does not. Same shape as `check-abi-bindings`.

The hash is evidence for the gate. It is never compared at build time, and it is
never the token.

### D6 — What the token does NOT cover, and why that is correct

A developer edits an emitter and does not rebuild the CLI. `E` is unchanged, the
fingerprint is unchanged, and regeneration proceeds with the old binary. **No
property of the binary can detect this** — the question is "do these sources
match this binary", and only a source-closure test answers it.

So the stale-CLI refusal survives, as a **developer-only** mechanism. In the
target shape a user has a released binary and no CLI sources, so it cannot fire
for them.

This reclassifies issue 1018's residue: the refusal is no longer the correctness
guard for a USER, it is a contributor cost with a backstop underneath it.

**But the two remedies this section first proposed are both ruled out, and
saying so is the point** (corrected 2026-09-05, after measuring):

* **Auto-rebuild is banned.** The refusal's own message says why — *"not
  auto-done — compiling at build/test time is forbidden"*. Making a consumer's
  build able to compile a Rust binary is issue 1018's option (1), rejected there
  for the same reason: on the Zephyr lane it is a surprise, and it hides the cost
  of a codegen change rather than naming it.
* **Narrowing the watch set is still rejected.** Issue 0604 measured a
  hand-rolled closure walk wrong in both directions at once, and a narrowing that
  is too narrow is museum code reported as success — strictly worse than a stop.

What CAN be removed is a watch-set entry that provably cannot affect an emitted
byte, which needs no judgement. One was:
`packages/cli/rosidl-codegen/templates/` was a byte-identical duplicate of
`packs/scaffold/` referenced from no `.rs` file, and deleting it moved the stamp
without moving the codegen fingerprint — the measurement that proves it reached
no output.

The other two stops issue 1018 reports are NOT that, and were checked rather
than assumed:

* the `play_launch` pin **is** a real CLI build input — `build.rs` bakes it as
  `NROS_PLAY_LAUNCH_SHA` and the issue-0409 guard compares it, and issue 0561
  records a pin move leaving the stamp unchanged while `setup-cli` skipped the
  rebuild and reported success. So "I only moved a submodule pin" is a CORRECT
  stop.
* `cmd/doctor.rs` is compiled into the binary, so it genuinely changes it. The
  stamp asks "does this binary match its sources", and for that question the
  answer is right. It is the wrong question to ask before *codegen*, but the
  right question cannot be answered without compiling the sources — which is
  where this section started.

So the residue is the price of a correct guard, and what this RFC changes is who
pays it: a user with a released binary has no CLI sources, so the guard cannot
fire for them at all.

### D7 — Provisioning makes the normal path unrepresentable

Because `nros` provisions the runtime (RFC-0014), the strongest form of this is
not a check: **the binary fetches the runtime matching its own `E`.** Drift then
cannot occur on the normal path at all, and D3's assertions guard the two paths
where a user overrides that — a pinned runtime, and an in-tree developer
checkout.

Enforcement stays everywhere regardless. A guard that only runs on the unusual
path is a guard nobody has evidence still works.

## Alternatives considered

**A content hash as the token.** Loses direction and cosmetic stability (D1).
Kept as the ratchet's evidence instead.

**Semantic versioning of the CLI as the token.** Not hypothetical — it is
what `abi_guard` does today, and D1a is the record of why it is being replaced.
A release version answers "which build is this", not "what does it emit"; it
needs a `Cargo.lock` to read, which a C/C++ user does not have; and strict
equality fires on bumps that changed no emitter, which teaches people to set
`NROS_SKIP_VERSION_CHECK`. It is the right thing to print in `nros --version`
and the wrong thing to compare.

**Reuse `codegen_fingerprint()` as the compatibility key.** Rejected in D4: it
is an emitter identity, so it moves on cosmetic template edits. Correct for
freshness, wrong for compatibility.

**Make the CLI a build dependency so a consumer's build rebuilds it** (issue
1018's option 1). Rejected as a general answer: it makes every consumer build
able to compile a Rust binary, which on the Zephyr lane is a surprise, and it
hides the cost of a codegen change rather than naming it. It remains available
*scoped to a developer checkout*, where the toolchain is present by definition —
see D6.

**Runtime version negotiation.** There is nothing to negotiate: one image, one
linked runtime, selected at build time. The same argument RFC-0088 makes for the
serialization format.

## Open questions

1. ~~Does `MIN` start equal to `N`?~~ **ANSWERED 2026-09-06 by phase-431's
   release cadence.** `MIN` starts equal to `N`, and the rule for moving it
   follows from the cadence rather than from taste: **bump `NROS_CODEGEN_VERSION`
   freely, raise `NROS_CODEGEN_VERSION_MIN` only deliberately.** In-tree a bump
   costs one regeneration, because `generated/` is never committed. Raising `MIN`
   costs something different in kind — it strands every RELEASED binary emitting
   below the new floor, and under a curated cadence (tag only when asked) those
   are exactly the binaries users have. Raise it when you are willing to say
   "that release can no longer build against this runtime", and say so in the
   release notes.
2. Where does the C/C++ anchor's `N` come from at *configure* time, for the four
   configure-time emitters phase-424 registered? The binary can be asked
   (`nros --codegen-version`), but that is a process launch per configure.
3. Does the ESP-IDF / PlatformIO integration path (`integrations/`) get the
   anchor, given both emit with a soft failure today?

## Changelog

- 2026-09 — created (phase-429), from the discussion on issue 1018.
