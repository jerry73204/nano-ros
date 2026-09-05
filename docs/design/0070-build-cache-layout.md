# RFC-0070 — Build caches: one root, one vocabulary, one derivation

**Status:** Draft (2026-08-06)
**Implements:** [phase-334](../roadmap/archived/phase-334-build-cache-layout.md) W2
**Relates to:** [RFC-0065](0065-colcon-like-workspace-builder.md) — that RFC decides who
*owns* a user workspace's `build/`; this one decides what every build cache in
this repository is *called* and *where it lives*.
**Informed by:** phase-334 W1 (the sharing verdict) and
[phase-340](../roadmap/archived/phase-340-build-artifact-reuse.md) (the identity
measurements).

## Problem

Build caches grew as suffix-named siblings of their sources. Three separate
spellings encode "which RMW" alone:

```
examples/native/rust/talker/target-zenoh/         # cargo, RMW suffix
examples/workspaces/c/build-workspace-fixtures/   # cmake, stage suffix
examples/workspaces/c/build-workspace-fixtures-freertos/   # + platform suffix
build/cargo-fixtures/<group>/                     # phase-226 group key
```

Counted 2026-08-06: **117 `target*` and 249 `build*` directories inside source
trees**, and **236 hardcoded path literals across 17 files** — `justfile`, five
`just/*.just` modules, six `scripts/build/*`, `examples/fixtures.toml`,
`check-fixtures-stale.sh`, and three Rust fixture resolvers under
`packages/testing/nros-tests/src/`.

That literal count is the actual problem. A path convention with 236 spellings
cannot be changed, and it cannot be *verified* — which is issue 0196's class
(build-side probes must watch what test-side gates watch) expressed as
directory names.

## Rule

### R1 — One root — **AMENDED 2026-08-10: scoped by context**

**R1 as originally written was too broad.** It said "Nothing writes build output
inside a source directory" globally. That is wrong for the copy-out examples, and
the amendment matters because acting on the unscoped rule already broke the
copy-out contract once (391 per-leaf `.gitignore` files were consolidated to the
root, after which a copied-out leaf had no ignore at all; restored same day).

The tree holds TWO kinds of project and they take different conventions:

| context | convention | why |
| --- | --- | --- |
| `examples/**` copy-out leaves | **Cargo/CMake native** — `target/` and `build/` beside the source, per-leaf `.gitignore` | a user copies the leaf out; it must behave like a normal Cargo/CMake project in their repo (CLAUDE.md: "Examples are standalone copy-out projects") |
| the nano-ros workspace (`workspaces/`, fixtures, tooling) | **colcon-style shared `build/`** (+ `dist/`, `log/`) | ROS familiarity is a project goal, and this is where sharing is legitimate |

> **AMENDED 2026-08-25 by [RFC-0065](0065-colcon-like-workspace-builder.md) §D8: `install/` → `dist/`.**
> `install/` promises an environment to source. nano-ros bakes ONE image per
> `(entry, board)` pair and composes nothing at runtime, so `dist/setup.bash` can never
> exist — and a ROS user's first move on seeing `install/` is to source it. `dist/<image>/`
> names what is actually there: finished artifacts, keyed by image.

R1 below applies to the SECOND row only. R2, R3 and R4 are unchanged and apply
to both.

**The measurement that forced this.** phase-344 §1.5 established that relocation
is not a disk win at all: of 182.3 GiB in cmake binary dirs, **151.7 GiB (83.2 %)
is corrosion's own cargo tree**, and moving those dirs frees ZERO bytes. What
actually saved 80.6 GB was SHARING — one `--target-dir` per identity group
(phase-340 B3/wave 2) — which is a Cargo-native mechanism that does not require
relocating anything.

So relocation and sharing are different levers and had been conflated.
Duplication comes from one Cargo/Corrosion invocation being ignorant of another:
corrosion derives its target dir from `CMAKE_BINARY_DIR` with no override, so N
cmake leaves give N cargo trees wherever those trees live. **R4 already names the
fix** — sharing must come from ONE invocation over many packages. In the
workspace that is available (one cmake configure over many packages gives
corrosion one cargo tree, the direct analogue of a Cargo workspace unifying
features and target dir). In a copy-out example it is unavailable by
construction, and that is correct rather than a gap.

### R1 (workspace scope) — One root

Every build cache lives under `$NROS_BUILD_ROOT` (default `<repo>/build/`,
overridable so the whole tree can move to a faster or larger volume — the
generalisation of the jobs audit's NVMe relocation, which today only `zephyr`
honours via `NROS_ZEPHYR_BUILD_ROOT`).

**Nothing writes build output inside a source directory.** Not
`examples/**/target-*`, not `examples/**/build-*`, not a workspace dir.

```
$NROS_BUILD_ROOT/
  cargo/<profile>/<variant-sig>/     cargo target dirs
  cmake/<kind>/<coordinate>/         kind = example | workspace | fixture
  west/<leaf>-<rmw>/                 zephyr (already rooted via env)
  models/<bringup>/                  phase-330 W3/W7 artifacts
  tools/…                            zenohd, install prefixes
```

### R2 — One vocabulary

A cache directory is named `<kind>/<coordinate>`, where the coordinate uses the
**fixture-manifest vocabulary already in use** — platform, lang, rmw,
feature-sig — and nothing else. `target-<rmw>`, `build-<rmw>`,
`build-workspace-fixtures[-<plat>]` all become derivations of that one scheme.

**A new ad-hoc suffix is a bug**, not a naming choice. The suffix zoo exists
because each new need invented a spelling instead of extending the coordinate.

### R5 — One rule for the KIND, not just the coordinate

*Added 2026-08-13 (phase-350 W5, issue 0539). R2 governs the `<coordinate>` half
of `<kind>/<coordinate>` and says nothing about `<kind>` itself, so the kind
names grew by precedent — which is how a family ended up spelled two ways.*

**A kind that holds fixture build trees is `<family>-fixtures`. A kind that
holds anything else is the bare `<family>`, named for what it contains.** A lock
guarding a kind is that kind's name plus `.lock`. No abbreviations: the name is
read far more often than typed.

Conforming today (16 of 18):

| shape | kinds |
| --- | --- |
| `<family>-fixtures` | `cmake-fixtures`, `idf-fixtures`, `west-fixtures` |
| bare `<family>` | `cargo`, `tools`, `zenohd`, `xrce-agent`, `qemu-zenoh-pico`, `sizes-probe`, `stack-analysis`, `link-determinism`, `borrowed-e2e`, `fixture-make-driver`, `zephyr-fixture-make-driver` |
| `<family>.lock` | `zephyr-fixture-build.lock`, `px4-msgs-codegen.lock` |

**All 18 kinds conform as of 2026-08-13.** Two were renamed to get there:
`fixtures-cargo` -> `cargo-fixtures`, and `compile-check` ->
`compile-check-fixtures`.

The second had been called un-renameable, and was: the token also names the
compile-check LANE, the `list-compile-checks` subcommand and three scripts
(`compile-check-fixtures.sh`, `-signature.sh`, `compile-check-stale.sh`), so a
global replace rewrote 43 files and produced `list-compile-check-fixturess`. It
became a two-line change once each kind had a named constant — one edit per
language, with the scripts and the subcommand untouched, which is exactly what
the extraction was for.

One kind used to keep a non-conforming SPELLING deliberately — `rmw_zenoh_ws`,
underscored because it mirrored an upstream colcon workspace name. It was
removed with the overlay it named (RFC-0075, amended 2026-08-19), so every
remaining kind is kebab.

**What a kind rename actually costs, measured.** Moving the directory does NOT
preserve the cache: `build/fixtures-cargo` held `CMakeCache.txt` files with the
old absolute path baked in, so after `mv` the nested cmake builds failed with
"The current CMakeCache.txt directory ... is different than the directory ...
where CMakeCache.txt was created". The 14 GB had to be wiped; rebuilding just
the linux/rust slice took 453 s, and the other platforms rebuild when their
lanes next run. Budget a rename accordingly — the cache is forfeit, not moved.

> **A correction, recorded because the inaccurate version is in issue 0539's
> filing.** That issue also lists `borrowed-e` and `px` as "truncations of
> nothing legible", and `zephyr-fixture-build` / `zephyr-fixture-make-driver` as
> "two roots for one family". Neither holds. The first two are `borrowed-e2e`
> and `px4-msgs-codegen` — the truncation was in the audit's own `grep`, whose
> character class stopped at a digit. The `zephyr-fixture-*` pair is a lock file
> and a driver scratch dir: different things that share a prefix. The real
> outlier set is the two rows above.

### R3 — One derivation, consumed by all three sides

The path is computed by ONE function. The build, the staleness gate and the test
resolver call it — they never spell a literal. This is the #393 rule
(build/gate/run derive from one computation) applied to paths.

`scripts/build/fixtures-target-dir.sh` is the working precedent: it already
groups rows by platform, triple, profile, features, env and sync-mode, and both
`fixtures-build.sh` and `rust-fixture-stale.sh` call it *specifically* so the
probe inspects the tree the build wrote. Generalise that function; do not add a
second one.

### R4 — Sharing is a property of the root, not of a build

**Amended 2026-08-08 (phase-340 W2), because the original prohibition rested on
a measurement that was never taken.** It read:

> A shared cache directory is only ever driven by **one** cargo invocation at a
> time. Cargo takes an exclusive lock per target dir, so N concurrent invocations
> against one directory serialise — measured in phase-334 W1.a/W1.c as a net loss
> against sccache, which already deduplicates the compilations.
>
> Where sharing is wanted, it comes from **one invocation over many packages**
> (cargo's internal jobserver parallelism), or from bounded worker concurrency —
> never from pointing concurrent workers at a common directory.

The serialisation is real and reproduces exactly. "A net loss" does not: the
A/B phase-334 W1.a cites varied `incremental`, not target-dir sharing. Timed
directly over 37 standalone leaves in one group, N concurrent invocations
against one dir came out **1.9× faster** than N separate dirs and removed
**21.8:1** of duplicate bytes (9.70 GiB → 455 MiB), never losing in any rep.

**The rule, restated.** Sharing comes from one invocation over many packages, or
from N invocations against one directory — and **which is right is a
measurement, not a principle**. One invocation is faster (measured ~9× against
separate dirs at that group size, ~1.7× against the shared dir) because inner
parallelism beats a flock. It is also the only shape that unions features across
its members, so it constrains the group key where N invocations do not, and in
this repository it requires generating manifests and a merged
`[patch.crates-io]` — a second spelling of `nros sync`'s output, which R3
forbids.

So: **prefer the shape that needs no generated state unless wall clock is the
binding constraint.** Both shapes deduplicate the bytes identically, and bytes
are what R1 exists for. What remains categorically wrong is pricing the flock
without pricing what it removes: every invocation after the first finds the
shared graph already fresh, so it serialises work that no longer exists.

## Consequences

**Deliberately enabled: corrosion's cargo trees become shareable.** phase-334
W1.c measured 32.6 GiB across 21 corrosion dirs with a 9:1 identity duplication
at the bottom of the stack, and found corrosion's own anti-collision hash is
*constant* for the shared nano-ros crates (`nano-ros_0b88c` in all nine
workspaces) — today's separation comes entirely from each workspace's
`CMAKE_BINARY_DIR`. Giving those trees one addressable root under R1 is what
makes the ~25 % disk saving reachable. R4 governs how.

**`.gitignore` collapses.** Per-directory ignore sprawl is replaced by
`build/`. The transition needs both sets until the last writer migrates.

**The mtime treadmill notes in CLAUDE.md name today's paths** and must move in
the same change as the family they describe.

**Out-of-tree consumers are unaffected.** This governs *this repository's*
caches. A user workspace's build root is RFC-0065's subject; the two agree that
`build/` is the root, and this RFC does not reach into a consumer's tree.

## Migration

Non-negotiable ordering, because the 236 literals are what makes this risky:

1. **Derivation first.** Extend the phase-226 resolver to cover every kind, with
   the current paths as its output. No directory moves. Behaviour identical.
2. **Callers second.** Replace literals with calls, one family at a time. Each
   family's build, staleness probe and test resolver move in ONE commit — a
   family split across commits is a red sweep.

   Step 2 splits in two, and the split was not visible when this was written
   (phase-334 W2.b, 2026-08-07). The **already-rooted** families
   (`build/compile-check-fixtures`, `build/cmake-fixtures`, `build/idf-fixtures`,
   `build/west-fixtures`, `build/cargo-fixtures`) migrate cleanly: they are
   R1-shaped but not R3-derived, so `nros_build_dir` reproduces them byte for
   byte and only `NROS_BUILD_ROOT` behaviour changes. The **in-source** suffix
   zoo — which is all 236 of the counted literals — has no root in it to derive,
   so there is no call that emits today's path. That class either needs a
   source-relative sibling derivation or belongs to step 3. Deciding which is a
   precondition for touching it, not a detail to settle mid-migration.
3. **Paths last.** Only once a family reads its path from the derivation does
   the derivation's output change. Then the family rebuilds once.
4. **Gate.** A check that fails on a `target-*` or `build-*` directory inside a
   source tree, and on a literal cache path in a script. Without it the zoo
   regrows; it regrew once already after phase-226 introduced the shared group.

Do not overlap a family's move with an in-flight rename elsewhere in that tree.

## Open

* ~~Which kinds beyond cargo/cmake/west need a coordinate — `compile-check`,
  `install`, and the `tools/` prefixes are currently ad-hoc but stable.~~
  **Answered in part by R5 (2026-08-13):** the kind NAME now has a rule, and 16
  of 18 kinds already follow it. Whether those kinds need a *coordinate* is
  still open and is a separate question from what they are called.
* Whether `$NROS_BUILD_ROOT` should be per-profile at the top level rather than
  under `cargo/`; the models and west trees are profile-independent today.
* The gate in step 4 needs an allowlist for vendored trees that build in place
  (`third-party/`), which cannot follow R1.
