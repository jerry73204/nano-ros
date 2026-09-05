# Phase 355 — Dependency and fork debt: what we carry that upstream does not

**Status (2026-08-15). W1 LANDED, W2 + W3 DECIDED.** W1 corrected six book
pages (the cited one was already fixed). W2 and W3 owed decisions rather than
code, and both are recorded in their issues; the two residual EDITS — offering
the cyclonedds changes upstream, and deleting `play_launch_parser`'s dead
`anyhow` line — are vendored-fork work, which by repo policy ends with a branch
prepared locally and the maintainer pushing.

Originally: Three standing
tech-debt items about dependencies the tree owns rather than consumes. None is
urgent; all three get worse silently, which is why they want a phase rather than
a backlog line.

**Owns:** [issue 0374](../issues/archived/0374-zenohd-has-no-prebuilt-so-nros-setup-native-source-builds-it.md)
(**RESOLVED 2026-08-17** — its last item was "seed the `1.7.2-nros2` asset", which
[phase-362](archived/phase-362-zenoh-router-from-ros-not-vendored.md) made moot by
retiring the vendored router; the fork-debt framing below no longer applies to it),
[issue 0507](../issues/archived/0507-cyclonedds-fork-carries-two-unupstreamed-changes.md),
[issue 0524](../issues/archived/0524-anyhow-unmaintained-transitive-deps-remain.md)
(**RESOLVED** — see the issue for what closed it).

**Related:** [RFC-0014](../design/0014-nros-setup-toolchain-management.md) (`nros
setup` / SDK index — #374 is a promise made there),
[phase-345](archived/phase-345-one-door-build-parity.md) (COMPLETE; named #374),
[issue 0496](../issues/archived/) (the work that created #507's two changes).

## The common shape

Each is a divergence between what the project SAYS it depends on and what it
actually carries:

* #374 — the book promises prebuilt toolchains; `nros setup native` source-builds.
* #507 — the vendored cyclonedds fork carries changes upstream lacks.
* #524 — the tree standardises on `eyre`; `anyhow` is still in the graph.

Divergence is not automatically a bug. What makes each an issue is that nothing
detects it growing.

---

## W1 — zenohd has no prebuilt, so `nros setup native` builds it (#374)

`book/src/.../installation.md:123-127` tells a first-time user that `nros setup`
"ships **prebuilt toolchains per platform per RMW** … fetched from a pinned
index". For native + zenoh it instead source-builds zenohd and pulls a second
Rust toolchain to do it.

Two honest resolutions, and the phase should pick one rather than drift:

1. **Ship a prebuilt zenohd** in the SDK index, making the book true.
2. **Correct the book** to say native+zenoh source-builds, and say why.

(1) is the better user experience and the larger job — it needs a build,
a host matrix, and a place in `nros-sdk-index.toml`. (2) is honest immediately.
They are not mutually exclusive: do (2) now, (1) when there is a publishing path.

**Acceptance.** The book and the tool agree, verified by `just probe bootstrap`
(the pristine-host runner for `probe=NN`-tagged book blocks, issue 0204) rather
than by reading.

## W2 — The cyclonedds fork's two unupstreamed changes (#507)

Closing issue 0496 added two changes the vendored fork carries alone:

1. **Striped addrset locks** (`cyclonedds@942dda3c`) — `struct addrset` no
   longer carries its own `ddsrt_mutex_t`; the lock comes from a 64-entry stripe
   array.
2. The second change is recorded in the issue.

Both exist because Zephyr's pthread mutex/cond pools are per-OBJECT, so a
mutex-per-entity library turns them into a cap on WORKLOAD size — the failure
CLAUDE.md records as an anonymous `abort()` 20 s into joining a 40-participant
Autoware graph (issues 0371/0496).

The debt is that a fork diverging from upstream gets harder to rebase every
release, and nothing here measures that. Note CLAUDE.md's constraint: the agent
does **not** push fork remotes by default, so any upstreaming work ends with a
branch prepared locally and the maintainer pushing.

**Acceptance.** Each change is either upstreamed (PR link recorded in the
issue), or documented as permanently-ours with the reason it cannot go upstream.
"Still carrying it" with no decision is what this work item exists to end.

**DONE 2026-08-15 — decision recorded; PRs are the maintainer's step.** The
census found **15** fork-only commits against `origin/releases/0.10.x`, not two:
this item was scoped to the newest pair. Each is now classified in
[issue 0507](../issues/archived/0507-cyclonedds-fork-carries-two-unupstreamed-changes.md)
with its reason —

* **offer upstream (10):** the striped addrset locks (`942dda3c`, the strongest
  candidate and not platform-specific), the three "fail loudly / say which pool
  ran out" diagnostics, the Zephyr sync backend (additive, behind
  `DDSRT_WITH_ZEPHYR`, needs a `WITH_ZEPHYR` option in cyclone's own
  `src/ddsrt/CMakeLists.txt`), and the three Zephyr socket-layer fixes;
* **permanently ours (5):** the ThreadX NetX port and its follow-ups (~1450
  lines) plus the FreeRTOS TLS fix. A new platform port is a maintenance burden
  upstream has no CI, users, or hardware for. Stating that is better than hoping
  it goes away, and it is precisely the residue that makes rebases cost.

Submitting the PRs is deliberately NOT part of this: CLAUDE.md keeps fork-remote
pushes with the maintainer, and opening a PR against `eclipse-cyclonedds` is an
outward-facing act. What W2 owed was the decision, and the decision exists.

## W3 — `anyhow`'s three transitive dependants (#524)

Census of 2026-08-12 found four sites; the two first-party ones are gone. Three
transitive remain, and `anyhow` is unmaintained.

Transitive means the fix is not ours to make directly: it is either a dependency
bump, a replacement of the dependant, or an accepted risk. All three are
legitimate; the issue is that none has been chosen.

**Acceptance.** Each of the three is named with its dependant and a decision
(bump / replace / accept-with-reason). If accept, the reason states what an
unmaintained `anyhow` actually risks here — it is a stable, widely-used crate,
and "unmaintained" is not by itself an exploit.

**DONE 2026-08-15 — decided, and one is not the change this expected.** Two
chains, not three sites; both measured with `cargo tree -i` rather than read off
the lockfiles:

* **`play_launch_parser` → REMOVE.** Expected a port to `eyre`; there is nothing
  to port. `anyhow = "1.0"` is declared in the fork crate's manifest and used
  NOWHERE in it (zero hits in `src/` and `tests/`; its errors are `thiserror`) —
  the third instance of the dead-declaration pattern this issue already deleted
  twice. It IS compiled into `nros-launch-resolve`, so removing it is worth
  doing. Left as a prepared decision rather than an edit: it is a vendored fork
  (maintainer pushes, fork branch before superproject pointer) and it moves a
  leaf lock, which must go through `just lock-update`.
* **wasi / wit-bindgen family → ACCEPT.** It enters through `cbindgen →
  tempfile → getrandom → wasip2/p3 → wit-bindgen`, so it is not "someone else's
  tree" — but `cargo tree -i anyhow` prints NOTHING for the host target and
  nothing for `--target all`: the crates that pull it sit behind features
  nothing enables. It is a lockfile entry that is never compiled, in any
  configuration this workspace builds. "Unmaintained crate in our lockfile" and
  "unmaintained crate in our binaries" are different risks, and this is the
  first.

---

## Deliberately not doing

* **No blanket dependency audit.** These three are named because each has a
  concrete divergence. A general sweep produces a list nobody acts on.
* **No fork rebase as part of this phase.** W2 decides what to upstream; the
  mechanical rebase is separate work under the vendored-fork branch workflow in
  CLAUDE.md (fork branch pushed FIRST, then the superproject pointer).

## Issues homed here (survey 2026-09-03)
Every open issue was checked for a home phase; these had none, or were
mentioned here only in passing. A mention is not an owner — an issue with
no work item is an issue nobody is accountable for, which is the same shape
as a gate sitting in a lane no CI job runs. Each row is a work item: the issue
holds the evidence, the item is *close it*.

| issue | why it belongs here |
| --- | --- |
| [#0910](../issues/0910-zenoh-pico-1-10-migration.md) | migrating to zenoh-pico 1.10: the serial layer moved and `config.h` is no longer generated the same way — carried-fork debt coming due |

