# Phase 431 — ship the `nros` binary

**Status (2026-09-06).** Opened. No code yet. [Phase-429](phase-429-the-codegen-version-is-enforced-everywhere.md)
removed the correctness blocker; what remains is distribution mechanics plus one
hazard that shipping CREATES and that is worth fixing before the first release
rather than after.

**Implements:** [RFC-0014](../design/0014-nros-setup-toolchain-management.md)
(provisioning) extended to the CLI itself, and
[RFC-0040](../design/0040-distribution-and-scaffolding-deps.md).
**Unblocks:** [#0171](../issues/archived/0171-no-external-distribution-path.md)'s
D1/D2 — *"ship the `nros` CLI as an installable artifact … the single
highest-leverage unlock"*, blocked since phase-287/288.
**Depends on:** phase-429, which answers *can this binary's output work with this
runtime?* — the question that made shipping unsafe.

## Why this is a phase and not a release chore

The distribution machinery already exists and is in daily use. `nros-sdk-index.toml`
carries `[tool.X]` with `version`, `dist.<host>.{url,sha256}`, a `smoke` check and
a `[tool.X.source]` fallback; assets live on `NEWSLabNTU/nano-ros-sdk` Releases
(`tag = <tool>-<version>`, `asset = <tool>-<host>.tar.zst`); `nros setup --tool
<name>` installs one; and `~/.nros/bin` already fronts `zenohd`,
`MicroXRCEAgent` and `play_launch_parser`.

`nros` is the one tool absent from it, for two reasons that are not paperwork:

1. **The bootstrap paradox.** You cannot `nros setup --tool nros` without an
   `nros`. Something outside the CLI has to place the first one.
2. **Shipping inverts an exemption that is currently correct.** See W1 — this is
   the item to do first, because it is a hazard from the day a binary exists on a
   developer's machine, and it is independent of every other decision here.

## Decisions taken (2026-09-06)

| question | answer |
| --- | --- |
| hosts | **Linux only** for now. The index has `macos-arm64` and the tree defers macOS support; a `nros` asset for it would be a support claim we do not make. |
| cadence | **Tag only when explicitly asked.** The CLI rolls rapidly in-tree; a release is a deliberate act, not a consequence of a version bump. |
| what a release is for | working builds for USERS, cut when needed — not a mirror of `main`. |
| one command | `nros` always means the newest installed. Versions accumulate in the store; the command does not fork. |

### What "rapid in-tree, rare releases" implies for `NROS_CODEGEN_VERSION_MIN`

This answers RFC-0090's open question 1, and the answer follows from the cadence
rather than from taste.

**Bump `NROS_CODEGEN_VERSION` freely; raise `NROS_CODEGEN_VERSION_MIN` only
deliberately.** In-tree a bump costs one regeneration, because `generated/` is
never committed. Raising `MIN` costs something different in kind: it **strands
every released binary that emits below the new floor**, and under a curated
cadence those binaries are exactly the ones users have.

So `MIN` is not a hygiene knob. Raise it when you are willing to say "that
release can no longer build against this runtime", and say so in the release
notes. The set of versions in the wild is small and knowable precisely because
releases are rare — which is what makes this policy affordable.

## Work items

### W1 — the stale guard keys on the WORKSPACE, not on the binary's path

**Do this first. It is a latent hazard today and an active one the day a binary
ships, and it depends on nothing else here.**

`stale_guard.rs` decides whether to check freshness by asking where the BINARY
lives:

```rust
/// `<root>` when `exe` is `<root>/packages/cli/target/**/nros`, else `None`.
fn checkout_root_of(exe: &Path) -> Option<PathBuf>
```

An installed `~/.nros/bin/nros` is therefore **exempt from the staleness check
entirely**. That is correct today: no released binary exists, so a non-checkout
`nros` is someone's deliberate experiment. It stops being correct the moment we
ship — a developer whose PATH resolves to the released binary inside a checkout
gets no freshness check at all, and emits with whatever that release's emitters
were.

**Phase-429 does NOT cover this.** The codegen version catches an INCOMPATIBLE
emitter. A released binary at the same version whose emitters have merely MOVED
is a freshness question, and `codegen-fingerprint` — which answers it — is
consulted by the fixture stamps, not at `nros` invocation.

The question the guard should ask is about the WORKSPACE it is operating on: if
that tree is a nano-ros checkout carrying CLI sources, the running binary must be
that checkout's build, whoever it is. Then a shadow refuses with an explanation
instead of quietly emitting, and correctness stops depending on `activate.sh`
having been sourced.

**Acceptance.** A released-shaped binary invoked against a checkout refuses and
names the remedy. The same binary invoked against a user project with no CLI
sources does not. Both observed, not reasoned.

### W2 — a shadow is an error, and CI may not install the release

* `warn_stale_shadow` in `justfile` warns today while its own text says
  *"`just doctor` will FAIL until this is resolved"*. Make that true.
* **A gate that no workflow installs the released binary.** The five CI
  acquisition sites build from source and assert `nros source-stamp`
  (phase-429 W5). A workflow that quietly switched to the download would stop
  testing the tree while staying green — the class `check-lane-contracts` exists
  for. Contributors and CI build from source, always; the release is for users.

**Acceptance.** The gate fails when a workflow is edited to fetch the asset.

### W3 — the store holds versions, `nros` means the newest

```
~/.nros/sdk/nros/<version>/bin/nros     versions accumulate
~/.nros/bin/nros                        the one command -> newest
```

This is the shape the store already has — `~/.nros/sdk/qemu/` holds
`11.0.0-nros2` and `11.0.0-nros6` side by side, and consumers enumerate
newest-first (`sort -Vr`, `COMPARE NATURAL ORDER DESCENDING`).

**Write `<store>/nros/<version>/` from the first install and never a flat
prefix.** Issue 0625 is what a flat prefix costs: `corrosion` was installed
without a version component, a `<store>/<tool>/*/` glob matched its `lib/` and
`share/` as if they were versions, and under `sort -Vr` a pure-alpha name sorts
AHEAD of numeric ones — `lib` won 155 of 183 resolutions in one configure.

Plus the `[tool.nros]` index entry, with a `smoke` check that runs
`bin/nros --codegen-version` rather than `--version`: the interesting property of
this particular tool is what it emits.

### W4 — `bootstrap.sh` downloads, and still builds from source

The paradox-breaker. Fetch `nros-<host>.tar.zst` + verify sha256, install through
W3, and keep the source build as the fallback for an unsupported host and as the
contributor path (`--from-source`). The book's step 2 stops being "build the CLI"
for a user and stays exactly that for a contributor.

### W5 — the release workflow

Manual dispatch plus an explicit tag, mirroring `nano-ros-sdk`'s `build-tool.yml`
rather than inventing a second shape. Never a local build — the SDK repo's own
convention, and the reason is reproducibility rather than convenience.

**The release must assert what it ships**: the built binary's
`--codegen-version` against the tree it was built from, so a release cannot go
out claiming a compatibility it does not have.

### W6 — the docs stop describing a source-only distribution

`book/src/getting-started/installation.md` currently carries a note explaining
why step 2 builds rather than downloads, added by phase-429 W7. That note becomes
wrong on the first release and should be replaced, not deleted — the reason
prebuilt binaries were withdrawn is worth keeping, with the answer now attached.
`AGENTS.md` and `scripts/bootstrap.sh`'s header (*"there is no prebuilt `nros`
download"*) likewise.

## What this phase must not do

**Do not let CI consume the release.** The whole value of building from source in
CI is that it tests the tree. A green lane running last month's binary is worse
than no lane.

**Do not make `nros` mean two things.** One command, newest installed. A
`nros-<version>` alias per install is a different feature and is not this.

**Do not raise `MIN` to tidy up.** See the policy above: it strands released
binaries, and under this cadence those are the ones users have.

## Acceptance for the phase

* A user on Linux with no checkout can obtain `nros`, run `nros setup`, and build
  a workspace.
* A developer with the release installed cannot silently use it against a
  checkout.
* No CI workflow consumes the release.
* A second install of a newer version moves `nros` without removing the older
  one.
* The release states the codegen version it emits.
