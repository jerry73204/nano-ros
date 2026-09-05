---
id: 1097
title: "The `xrce check-rust-rmw` recipe ran `cargo check` in a directory that has NEVER held a `Cargo.toml` — so it silently checked the whole workspace instead, and was red for an unrelated reason on every host"
status: resolved
type: tech-debt
area: rmw, build
severity: low
found: 2026-09-05
related: [phase-420, phase-140, phase-121, 0196, 0660, 1068, 1069]
---

# A recipe that cannot work, on no lane that runs

`just/xrce.just`:

```just
# Phase 121.7.i — `cargo check` the out-of-workspace nros-rmw-xrce Rust
# crate so CI catches drift after platform-API or trait-surface changes.
# Workspace `cargo check` skips it (it's workspace-excluded for xrce-sys
# linking reasons), so this recipe is the only place it gets compiled.
[group("debug")]
check-rust-rmw:
    cd packages/rmw/xrce/nros-rmw-xrce
    cargo check --quiet
```

`packages/rmw/xrce/nros-rmw-xrce/` is a **CMake project**. It ships
`CMakeLists.txt`, `include/`, `src/*.c`, `tests/` and `package.xml`, and no
`Cargo.toml` — phase-140 deleted the install rules and the Rust side moved to
`nros-rmw-xrce-cffi`. So the recipe fails with `could not find 'Cargo.toml'`,
and the comment above it is wrong twice over: there is no "out-of-workspace
nros-rmw-xrce Rust crate", and this is not "the only place it gets compiled".

It is not on `check`, `ci` or any CI lane — it is `[group("debug")]` and named
by nothing — which is why a recipe that cannot succeed has sat here for
phases. That is the point of filing it rather than leaving it: a broken recipe
nobody runs reads, to the next person, as coverage that exists.

**Found by** phase-420 W9 step 4, while enumerating who compiles the vendored
XRCE trees. Not fixed there: outside that change's scope, and the fix is a
judgement call rather than a mechanical one.

## What the fix probably is

Delete it. The intent it names — "catch drift after platform-API or
trait-surface changes" — is already served, and better, by two things that did
not exist in phase 121:

- `just check rmw-xrce` compiles this project's C (issue 0787) and runs its two
  CTests; since phase-420 W9 step 4 those link the cargo lane's archive, so they
  exercise what images ship;
- `nros-rmw-xrce-cffi` is the actual Rust crate and is built by
  `cargo build -p nros-rmw-xrce-cffi`, which that same recipe now runs.

If instead someone wants the Rust crate `cargo check`ed by name, the recipe
should say `cargo check -p nros-rmw-xrce-cffi` from the workspace root and drop
the `cd`.

## Class

Third in a small family this directory keeps producing: a statement about the
build that nothing verifies. The other two were the source-list mirror
(issue 1068) and the version restatement (issue 1069), both of which had a
correct-looking comment asserting an invariant that had stopped holding. This
one is a whole recipe in that shape.

## Resolution (2026-09-05)

**Deleted, and a gate added, and one sibling found and deleted with it.**

### The issue's own text was wrong in two places, and the second one matters

1. **The recipe did not break at phase-140; it was never able to work.** No
   `Cargo.toml` has ever existed at `packages/rmw/xrce/nros-rmw-xrce/` in this
   repo's history (`git log --all --diff-filter=AD -- '*nros-rmw-xrce*/Cargo.toml'`
   shows the crate's manifest only under the pre-reorg `packages/xrce/` path).
   It was deleted **2026-05-12** by `f5aeb8faf` (phase-115.K.2, "legacy xrce
   delete") and the recipe was written **2026-05-13** by `68c89568c`
   (phase-121.7.i) — one day later, against a directory that was already
   C-only. `git cat-file -e 68c89568c:packages/xrce/nros-rmw-xrce/Cargo.toml`
   fails. The phase-121 roadmap entry that records it says "CI now exercises
   it"; that was never true for a single day.

2. **It did not fail with `could not find 'Cargo.toml'`.** `cargo` walks UP for
   a manifest, and from that directory it resolves the ROOT workspace —
   measured: `cargo locate-project` there returns
   `/home/aeon/repos/nano-ros/Cargo.toml`. So `cargo check --quiet` became a
   **full-workspace check**, i.e. the recipe verified every crate EXCEPT the
   xrce one it claimed to (`nros-rmw-xrce-cffi` is in `HOST_UNCHECKABLE`), and
   died on something unrelated:

   ```
   error[E0152]: found duplicate lang item `panic_impl`
      --> packages/api/nros-c/src/lib.rs:171:1
   error: could not compile `nros-c` (lib) due to 1 previous error
   ```

   `nros-c` is `--exclude`d from the workspace gate for exactly that reason, so
   this red is structural and permanent on every host. This is why the gate
   below checks the MANIFEST and not merely the directory: a directory-only
   check would have called this recipe healthy.

### What covers the intent

`just check rmw-xrce` (`just/check.just`, on the `build-serial` lane): it runs
`cargo build -p nros-rmw-xrce-cffi`, then cmake-builds this C project against
that lane's own vendored archive and runs its CTests. That is strictly more
than a `cargo check` of the crate. Respelling to `cargo check -p
nros-rmw-xrce-cffi` was rejected: it would be a second, weaker spelling of that
gate's first step, in an unlaned `[group("debug")]` recipe — the shape CLAUDE.md
warns against ("add ONE shared helper rather than a second spelling").

### The sweep, and its one other finding

Corpus: the root `justfile` + `just/*.just` (24 files, 775 recipes).

| Axis | Checked | Findings |
| --- | --- | --- |
| `cd`/`pushd` targets | 42 sites (28 literal, 14 dynamic) | 1 — this issue |
| Manifest present for the tool run there | all literal cd sites | 1 — this issue |
| `cmake -S <dir>` source dirs | 6 (3 literal) | 0 executable |
| Scripts a recipe runs (`scripts/`, `tests/`, `.sh`/`.py`) | 300 | 1 — see below |
| `cargo -p <pkg>` names | 32 | 0 |
| `check stack-all`'s 15 example dirs | 15 | 0 |

**Sibling, deleted in the same change: the `native test-ros2-shell` recipe**
(`just/native.just`), `[group("debug")]`, ran `./tests/ros2-interop.sh`.
`git log --all -- '*ros2-interop.sh'` is **empty** — the script phase-16 planned
("Add shell test script `tests/ros2-interop.sh`") was never committed, so that
recipe has never worked on any host either. Same shape exactly: born dead,
unlaned, named by nothing. ROS 2 interop is covered by `just native test-ros2`
(the `interop_e2e` nextest binary) and `interop::CELLS` (RFC-0051).

### The gate

`scripts/check-just-recipe-paths.py`, registered as `check just-recipe-paths` on
the `fast-serial` list. Buildless, 0.19 s. It is the sibling of issue 0660's
`check-just-recipe-refs` one level over: that one checks recipe NAMES, this one
checks filesystem PATHS.

Rule, stated to match its coverage exactly (issue 0196): *inside recipe bodies,
for LITERAL in-repo paths* — a `cd`/`pushd` target exists; the tool run in it
finds its manifest there (`cargo`→`Cargo.toml`, `cmake`/`west build`/`idf.py`→
`CMakeLists.txt`, `nros generate-rust`→`package.xml`); a `cmake -S` dir has a
`CMakeLists.txt`; every `scripts/`/`tests/` `.sh`/`.py` a body names exists.
It models `just`'s two body shapes, because the difference is load-bearing:
without a `#!` shebang each line is its own shell, so a bare `cd` line is inert
and the tool below it runs at the repo root — wiring those together would invent
failures across most of the corpus.

Deliberately out of scope, and said so rather than claimed: dynamic targets
(`cd "$WORKSPACE"` — 14 of 42 sites), paths inside a registered submodule or
git-ignored (both DERIVED from `.gitmodules` / `git check-ignore`, never a
hardcoded exception list), and literals whose first segment is not a repo-root
directory (`cd accept_app` is a scratch tree `nros new` created).

Run standalone it reproduces the sweep's two findings and nothing else, then
goes green after the deletions.

### Deliberately not done

- **Not fixed: `tests/README.md`** documents `tests/ros2-interop.sh` in its tree
  listing and in a usage example (`./tests/ros2-interop.sh all`) — a file that
  has never existed. Outside this change's owned files; worth its own issue.
- **Not fixed: `just/check.just:4362`**, a doc-comment on `check stack` reading
  "Default: examples/qemu/rs-wcet-bench" while the actual default is
  `packages/testing/nros-bench/wcet-cycles-qemu`. `check.just` is heavily
  contended; the comment is wrong but nothing executes it.
- **Fixed in passing: `just/px4.just:129`**, a prereq comment spelling `cmake -S
  packages/core/nros-platform-posix` for a crate that lives at
  `packages/platform/nros-platform-posix`; the two live invocations in that file
  (lines 171, 284) already used the right path.
- **Not gated: dynamic `cd` targets.** Half the remaining risk lives there and
  no static check can reach it. Naming that is better than a gate whose green
  implies coverage it does not have.

### Class note

The issue filed this as third in the "a statement about the build that nothing
verifies" family (1068, 1069). It is that, but the sharper reading is issue
0660's: **a reference `just` resolves only at run time, in a recipe no lane
runs.** 0660 was twelve dangling recipe NAMES; this is two dangling PATHS. Both
sat green for months for the same reason, and the two gates now cover both
halves.
