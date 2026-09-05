# Phase 347 — RMW as a declared provider: capabilities, search, and no special cases

**Status (2026-08-11). W1–W4 LANDED. W5 and W6 NOT STARTED, each blocked on a
condition recorded before the work began, not on time.**

| wave | state |
| --- | --- |
| W1 subtractions | landed — and found a user-facing feature wired to nothing |
| W2 descriptor + gate | landed — 4 descriptors (not 8: only 4 of the 8 `packages/rmw` dirs are backends) |
| W3 resolution by name | landed — retired **FOUR** closed lists, not the three the plan named |
| W4 capabilities | landed, acceptance PARTIAL — `packages/core/**` clean, `packages/api/**` not (21 features) |
| W5 codegen hook | **landed** — condition settled by measurement; cyclonedds in the shared codegen file 27 -> 1 |
| W6 platform leak | **landed** for `[build.*]`; `[knobs.*]` deliberately left, with the reason |

**Original status (2026-08-10). PROPOSED — no code landed.** Implements
[RFC-0071](../design/0071-rmw-backend-descriptor.md). The design is settled and
its open questions are answered; this phase is the ordering.

**Scope.** Three things, in the order they unblock each other: remove the
backend names core should not have, give a backend a descriptor, and let
selection resolve through it. **The colcon convention is explicitly NOT in this
phase** — see "Deliberately out of scope".

**UNBLOCKED 2026-08-10.** This phase was blocked on
[issue 0493](../issues/archived/0493-two-workspace-roots-share-one-corrosion-target-dir-duplicate-no-mangle-symbols.md)
while its cause was unknown. It is known and fixed: corrosion **v0.5.1** sets the
cargo target dir to a CONSTANT `${CMAKE_BINARY_DIR}/${build_dir}/cargo/build`, so
two workspace roots configured into one binary dir shared one `deps/` and their
`#[no_mangle]` exports collided; **v0.6.1** hashes the workspace manifest path.
The index pin is bumped, and the fix is verified end-to-end on the tree that had
been failing — `workspace-fixtures-build.sh linux` returns RC=0 with **zero**
duplicate-symbol errors, and the mixed workspace now shows two hashed dirs
(`nano-ros_23c15`, `nros_ws_runtime_16b35`) with ONE cffi identity each instead
of one hashless dir holding two.

Two consequences for this phase. **W2 onward is unblocked** — the linking model
is no longer unexplained, so a declaration format cannot encode a bug it does
not have. And the `nros-c` / `nros-cpp` feature tables are no longer contended,
so W3/W4 may edit them without racing another investigation.

Note for anyone rebuilding: provisioning alone is not enough on a tree that has
already configured — the stale build dirs carry the old topology in their
`CMakeCache.txt` and must be removed.

**Sibling, not overlap:** [phase-346](archived/phase-346-out-of-tree-board-seam.md) is
this problem on the BOARD axis (RFC-0064, a board arrives through an integration
shell nano-ros never sees). RFC-0071 D8 is where the two converge — one
descriptor family across rmw / platform / board. Do not duplicate 346's work
here; when both land, reconcile the two descriptor schemas in one commit.

---

## W1 — Subtractions (no new mechanism, no dependency)

Pure deletions of backend names that are already dead. Independent of everything
else in this phase and of 0493, so it can start now.

- [ ] **`has_rmw` is 3/4 dead.** `nros-node/build.rs` tests
      `CARGO_FEATURE_RMW_{ZENOH,XRCE,UORB}`, and **`nros-node` declares none of
      those features** — only `rmw-cffi`. Delete the three dead disjuncts.
- [ ] **The comment above it is false.** It claims `has_rmw` is also set "when
      compiling for tests (unit tests use MockSession)". There is no such branch.
      Fix the comment or restore the behaviour — decide which, do not carry it.
- [ ] **`unstable-zenoh-api` on `nros-node` is dead.** No other manifest forwards
      it, no `cfg` reads it; its only core mention is a doc comment whose real
      seam is `rmw-lending`. The live chain is entirely in the backend
      (`nros-rmw-zenoh` → `zpico-sys` → `Z_FEATURE_UNSTABLE_API`). Delete it.

*Acceptance:* `packages/core/**` contains no `zenoh|xrce|uorb` outside prose.
`__cyclonedds-link` survives W1 by design — it has a live consumer and is W4's
job.

## W2 — The descriptor exists, and is proven before anything reads it

- [ ] `nros-rmw.toml` for the 8 in-tree backend families, carrying exactly what
      `nros_rmw_dispatch` already computes (cffi feature, rlib dep, extra link
      libs, `needs_cxx_linker`), plus `names` (from `canonical_rmw`'s alias
      table) and `[rmw.provides.*]` per RFC-0071 D7.
- [ ] A gate asserting **descriptor ≡ dispatch**, row for row, while both exist.

The gate is the point of the wave. This repo's recurring defect is two
derivations of one fact drifting (`row_coord`, fixture groups, the sizes-header
mirror); a descriptor is a second derivation until the gate makes it one. W2
changes no behaviour and is fully reversible.

*Acceptance:* the gate passes; deleting a descriptor row fails it.

## W3 — Resolution by name; retire all three lists

- [ ] `resolve_rmw` resolves a declared name through the descriptors' `names`
      tables. `KNOWN_RMW` and `canonical_rmw`'s `match` are deleted.
- [ ] `NanoRosRmwDispatch.cmake` stops being generated and becomes a descriptor
      read (via the CLI — CMake already shells out to `nros` in three modules;
      a second parser of the descriptor is the defect W2's gate exists to
      prevent).
- [ ] **All three** closed lists retire together:
      `resolve_rmw`/dispatch, `NanoRosFeatureSet.cmake`'s validator, and
      `nros-cpp/CMakeLists.txt`'s own chain. Retiring one leaves two
      disagreeing, which is the bug.
- [ ] `uorb` is covered on equal terms — it behaves like every other backend and
      only its PX4 consumer is unusual.

*Acceptance:* **a fifth backend, out of tree, with zero core edits.** Until that
is demonstrated the wave is unproven; every closed list in this area looked open
until someone tried.

## W4 — Capabilities, lowered to both languages

- [ ] `[rmw.capabilities]` maps a capability name to the backend's own feature
      (`zero-copy-receive = "unstable-zenoh-api"`); the user declares
      `[system] capabilities = [...]`.
- [ ] Lower to **both** consumers: the generated selection facade (Rust) and a
      compile define (C/C++). A capability that works in Rust and silently does
      nothing in C++ is worse than not having it.
- [ ] Replace `__cyclonedds-link` with the capability `needs-type-descriptors`.
      The seam it feeds (`MessageForRmw`, `register_type_descriptor`) is already
      generic — only the trigger moves.
- [ ] Reserved vocabulary with written semantics, plus an open `x-<vendor>-*`
      namespace nano-ros never interprets. Open-only means a user switching
      backends silently loses a capability; reserved-only means a third party
      cannot offer one.
- [ ] Selecting a capability the active backend does not declare is an error
      naming what it does offer.

*Acceptance:* PARTIALLY MET — landed 2026-08-11, and the shortfall is recorded
rather than rounded up.

* **`packages/core/**` is CLEAN**: zero backend-named features. That was the
  wave's core goal and it is done.
* **`packages/api/**` is NOT**: 21 backend-named features remain, all in the
  C/C++ umbrella crates (`nros-c` 14, `nros-cpp` 6, `nros` 1) — the
  `rmw-{zenoh,xrce,cyclonedds}[-cffi]` tables that bundle a backend into the one
  Rust staticlib, plus `platform-*` and `xrce-{udp,serial}`.

Those are not a naming slip; they are how the umbrella selects what it bundles,
and unwinding them changes which archive owns the nros symbol set — the same
question issue 0493 opened. **It needs its own wave, sequenced after the
provider design settles**, not a rename here. `check-rmw-agnostic` therefore does
NOT go green in W4; it is scoped to `packages/core/**` for now.

## W5 — The per-message codegen hook (highest risk, may split)

- [ ] `[rmw.codegen].per_message` replaces the hardcoded cyclone branch in
      `NanoRosGenerateInterfaces.cmake` (27 mentions, the most of any file).
- [ ] **It needs a cargo-rooted twin.** D7's 2×2 has a cargo consumer too, for
      which descriptors come from the `codegen_cyclonedds_descriptors` CLI verb
      rather than a CMake function. Whether both can share one contract is
      unresolved — settle that before starting, or split W5 into its own phase.
- [ ] Retires RFC-0031's blanket rule that "Cyclone selection always routes
      through the CMake/Corrosion build path, even for an otherwise-Rust
      binary" — that rule exists only because this cell was undeclared.

*Acceptance:* cyclonedds appears in `cmake/` only inside its own package.

**LANDED 2026-08-11. The condition was settled first, by measurement.**

The blocking question was whether one contract could cover both the CMake hook
and the cargo-rooted path. It can, and the two turned out to be the same work:

  CMake `nros_rmw_cyclonedds_generate_from_msg`  -> msg_to_cyclone_idl.py -> idlc -> <pkg>_<Msg>.{c,h}
  CLI  `nros codegen cyclonedds-descriptors`     -> msg -> IDL -> idlc -> the same pair
                                                    (+ the `_register_descriptors` entry point)

Same tools, same outputs; the CLI verb is the more complete of the two. So the
contract is "per message, run this command", and a cargo-rooted consumer
satisfies it through the verb while cmake satisfies it through the hook. No two
flavours were needed.

**Result: `cyclonedds` in `cmake/NanoRosGenerateInterfaces.cmake` went 27 -> 1**,
and the one is the comment explaining the move. ~180 lines moved into the
backend's own package as `nros_rmw_cyclonedds_typesupport_for_target`, reached
through the descriptor's `[rmw.codegen].per_message` surfaced as
`NROS_RMW_PER_MESSAGE_HOOK`. Nothing in those lines was generic — they know that
Cyclone 0.10.5's idlc aborts on `wstring`, that descriptors are C so the consumer
must enable that language, how the shared IDL include root is laid out, and how
to force-load a static-init registration TU past `--gc-sections`.

Verified by BUILD, not configure: `examples/native/c/talker/build-cyclonedds`
produces `builtin_interfaces__cyclonedds_ts` and `std_msgs__cyclonedds_ts`, and
links a 13 MB `c_talker` carrying 28 descriptor symbols — the pipeline behaves
identically from its new home.

**Not done here:** RFC-0031's blanket "cyclone always routes through the
CMake/Corrosion build path" is now *unnecessary* but not yet deleted — retiring
it needs the cargo-rooted consumer actually exercised, which no in-tree fixture
does today.

**Superseded note —** W5 says, in
its own second bullet, that the cargo-rooted twin "is unresolved — settle that
before starting, or split W5 into its own phase". It is still unresolved:
`[rmw.codegen].per_message` names a CMake function, and a cargo-rooted consumer
reaches the same work through the `codegen_cyclonedds_descriptors` CLI verb
instead. Whether one contract covers both is the question, and starting the
implementation would answer it by accident rather than by design.

Recommend splitting W5 into its own phase, since it is also the only wave that
adds new machinery rather than moving or deleting existing facts.

## W6 — The platform descriptor leak (small, independent)

- [ ] `nros-platform.toml` carries `[knobs.zenoh.tx]` and `[build.zenoh]` —
      backend-named sections in a *platform* file. Key them on the resolved
      backend so a platform declares settings without naming one. Same
      violation as core naming a backend, one axis over.

**LANDED 2026-08-11 for `[build.*]`. My deferral reasoning was WRONG and the
correction is the useful part.**

I deferred this as "no second tenant to validate the map shape against". That
conflated two things. The payload — `PlatformEntry` — carries `defines`,
`defines_kv`, `include`, `exclude`, `system_libs`, `extra_sources`, `arch`,
`compile`, `pic`, … and **not one zenoh-shaped field**. It is generic
vendored-C-library build config, already proven across the seven
`config/*/nros-platform.toml` files. There was nothing to design.

Only the KEY was backend-specific: a struct field where a map key belonged. A
map key needs no second tenant to validate, because it is not a design.

So `BuildSection` is now `BTreeMap<String, PlatformEntry>`, and **none of the
seven platform files changed** — `[build.zenoh]` simply parses as the key
`"zenoh"`. `[build.cyclonedds]` used to be a hard parse ERROR under
`deny_unknown_fields`; it is now merely absent.

The second tenant exists anyway, which is evidence the shape is right rather
than speculation: `nros-rmw-xrce-cffi/build.rs` is ~500 lines hardcoding this
same config (`_DEFAULT_SOURCE`, `_POSIX_C_SOURCE`, posix/embedded branching, a
generated config header) because there was nowhere to declare it. Moving it is
its own task; the schema no longer blocks it.

Two tests, both new: a second `[build.*]` key parses and both keys survive
independently, and the real `config/` tree still loads ≥7 zenoh blocks
(behaviour-preserving is the claim, so it is checked).

**Found while testing:** `platform_config` sits behind
`#[cfg(feature = "build-helpers")]`, so a default `cargo test -p
nros-board-common` runs **11** tests and NONE of this module's. With the feature
it runs **25**. Anyone validating this file must pass `--features build-helpers`
or they are testing nothing.

**`[knobs.zenoh.tx]` is deliberately NOT generalised.** Its fields are typed
policy for zenoh's TX path (`batch`, `split_lock`, `flush_ms`), not generic
vendored-library config, and a second tenant there IS speculative — which is the
argument I wrongly applied to `[build.*]`.

**Superseded note — Measured: the schema is
`BuildSection { zenoh: Option<PlatformEntry> }` with `deny_unknown_fields`, so
`[build.cyclonedds]` is not merely absent — it is REJECTED. Four non-test
consumers (`platform_config.rs` ×2, `cmd/config.rs`, `nros-zpico-build/runner.rs`)
plus a dozen test fixtures.

The generalisation is justified in principle — `xrce-sys` is also a vendored C
library and would want the same block — but **there is no second tenant to
validate the map shape against today**. Converting the schema now would be
designing a generic container from one example, which is how the RMW dispatch
got its shape in the first place. Do it WITH the second tenant, so the shape is
answerable rather than guessed.

---

## Deliberately out of scope: the colcon convention

RFC-0071 D5 settles that discovery is a **search path of workspace roots scanned
at source time**, and why: colcon's index is reached by sourcing `setup.sh`, so
it exists only after an *install* step, and per-target static RTOS artifacts have
no such stage.

Adopting that convention is a **separate program**, not a wave here, because its
cost is a migration rather than a mechanism:

| axis | dirs needing `package.xml` | with a descriptor today |
| --- | ---: | ---: |
| `packages/rmw` | 8 families | 0 |
| `packages/boards` | 17 | 8 |
| `packages/platform` | 14 | — |

None of nano-ros's providers carry a `package.xml`; the 99 in the tree are
interface packages and test fixtures. And the hard part is not the scan — it is
that **a provider in `src/` may need building before its consumer links it**,
while `nano_ros_workspace(SUBDIRS …)` takes an explicit list today. Discovery
becomes scheduling.

**It is blocked on W2 regardless**: you cannot discover providers that do not
describe themselves. Filed as
[phase-348](archived/phase-348-source-time-provider-discovery.md), with these starting
points already established — the two accepted roots (nano-ros tree,
user workspace; no `~/.nros`, no env var), first-match-wins shadowing with a
warning, and `package.xml`'s existing `<depend>` as the ordering source rather
than a second declaration.

## Order

```
W1  ─────────────────────────────────────────►  (independent, start now)
W2 ──► W3 ──► W4                                (0493 cleared 2026-08-10)
  │       └────► W5   (splittable — settle the cargo-rooted twin first)
  └────────────► phase-348  (source-time provider discovery)
W6  ─────────────────────────────────────────►  (independent)
```

W1 and W6 need nothing and can run now. **W2 is the fence**: everything after it
reads a descriptor, everything before it is deletion. W5 and phase-348 both hang
off W2 and are independent of each other.
