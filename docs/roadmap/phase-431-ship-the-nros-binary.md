# Phase 431 — ship the `nros` binary

**Status (2026-09-06). Every work item is landed.** No release has been cut:
W5 is manual dispatch, and cutting one is a decision, not a consequence — the distribution
mechanics proper. [Phase-429](phase-429-the-codegen-version-is-enforced-everywhere.md)
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

**Landed 2026-09-06.**

*The doctor half.* `_doctor-host` now reports a shadow as `[FAIL]` and
`_doctor-host` / `doctor` / `_doctor-scope native` carry the verdict out, so the
phase-220 promise is true. Three things had to move together, and the second is
the one worth recording: `doctor` had `set -e`, so a failing host block would
have skipped every platform probe — doctor's contract is that it reports every
unmet precondition in ONE run, so the verdict is aggregated and emitted at the
end instead. `just doctor native` reaches the same block and now returns the
same code, because one block with two verdicts is how a check stops meaning
anything.

Why a FAIL and not a WARN, stated in the recipe itself: `nros_cli_bin` resolves
**PATH before the per-checkout binary** (cargo.sh, phase 218.D.3), so a shadow is
not cosmetic — it is the binary every `nros …` in this tree actually runs. W1
refuses it at `nros build`; this is the same answer at the moment someone is
asking whether their machine is ready.

Observed:

```
no `nros` on PATH            -> [INFO], rc 0   (recipes use the in-tree binary)
a foreign `nros` on PATH     -> [FAIL], rc 1   (and the blocks below it still run)
the checkout's own on PATH   -> [OK],   rc 0
```

`setup-cli`'s warning was rewritten with it. Its old remedy said to delete
`~/.nros/bin/nros`, which W3/W4 make the *supported* user install — so the
remedy is now `source ./activate.sh` (put the checkout first), with removing a
pre-218 `~/.cargo/bin/nros` as the fallback.

*The gate half.* `check-ci-cli-from-source` (fast lane, 232 gates) runs two arms:

* **no workflow acquires the CLI as a release asset** — `gh release download`,
  a `releases/download/` URL, the `nros-<host>.tar.zst` asset name,
  `--tool nros`, or `bootstrap.sh` without `--from-source` (W4 makes download
  its default, so an unflagged invocation *is* the release path).
* **every from-source build asserts `nros source-stamp` in the same step** —
  phase-429 W5's rule, which nothing enforced.

The second arm found an offender immediately: `gate.yml`'s CLI warm-up for
`check cli-tests` built the binary and asserted nothing, so **four of five sites
carried the rule and the fifth read exactly like the other four**. Fixed in the
same commit.

Two false-positive lessons are in the script. Arm A first flagged `docs.yml`
downloading **mdbook-mermaid** from a GitHub release — CI legitimately fetches
other projects' releases, so a match now also has to name `nros` (with
`bootstrap.sh` exempt from that filter, since it is the front door and spells
the binary nowhere). And the name is often not on the line the verb is, so the
scan joins backslash continuations into logical lines first.

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

**Landed 2026-09-06.**

The store already wrote `<store>/<tool>/<version>/` — `tool_prefix` has been the
one constructor since phase-365 — so W3 is the *front* half plus the entry.

*Fronting.* `front = ["bin/nros"]` in `[tool.<name>]` names prefix-relative paths
that must be reachable by bare name from `$NROS_HOME/bin`. `front_newest`
recomputes from the store on every install and links the **newest installed**
version, not the one just installed: versions accumulate, nothing prunes them
(issue 0500), and installing an older one on purpose must not silently downgrade
the command. It is a parameter of `execute` rather than a separate call, because
three call sites reach that function and a fourth spelling of "and then link it"
is how one of them ends up not doing it.

`installed_versions` **enumerates** the store, which `check-sdk-store-not-
enumerated` bans — and correctly, for the question that gate is about. These are
two different questions: *where is the version I pinned* is constructible from
two inputs; *what is the newest thing installed here* is not, and nothing but the
store knows it. Issue 0625's defence moved into the filter rather than the sort:
a candidate must begin with a digit **and** carry a `.nros-provenance` marker, so
a legacy flat prefix's `lib/` cannot win by sorting. The comparator is
natural-order on top of that (`nros2` < `nros10`, which a string compare gets
backwards and the store's vocabulary already reaches — `qemu 11.0.0-nros6`).
The gate's docstring now says all of this, so the next reader does not have to
re-derive whether the two rules conflict.

*The verb.* `nros sdk-front <tool>` exists for the one caller that cannot go
through `nros setup`: W4's `bootstrap.sh`, which unpacks the CLI itself and has
no `nros` to run `nros setup --tool nros` with until it has. One implementation,
two callers — a second spelling in shell is how they would come to disagree about
which version `nros` means.

*The entry.* `[tool.nros]` was REMOVED by phase-288 D1/D2, and the index still
carried the comment saying why: the archived standalone repo's 0.3.6 binaries
were 15 releases stale and ABI-mismatched with the checkout, so installing one
was a footgun. That reason is now **answered rather than waived**, and the index
says which three things answer it (RFC-0090, W1, W2). No `dist` rows until W5
seeds an asset; until then the source recipe builds it, pinned to a **commit**
(issue 1060 — a tag is a ref on a server we do not control).

`smoke` asserts the codegen version, which makes it an authored mirror of
`NROS_CODEGEN_VERSION` — the shape that has drifted every time nothing bound it
(`check-rmw-api-parity`, 25 symbols, reading green). A test binds it to
`abi_guard::EMITTED_VERSION`.

*And `nros` is deliberately absent from `scripts/sdk-path-tools.txt`.* That list
puts a store `bin/` on PATH for binaries something we do not control invokes by
bare name; the CLI is not one, and if it were on the list, sourcing `activate.sh`
in a checkout would put a foreign emitter first — the exact state W1 refuses and
W2 fails on. `check-activate-shells` now asserts the behaviour rather than the
list: a store holding `nros/<version>/bin/nros` must not reach PATH. Verified by
mutation (adding `nros` to the list turns it red).

Observed, against a real binary in a scratch store:

```
install 0.5.0-nros1, front  -> ~/.nros/bin/nros -> …/0.5.0-nros1/bin/nros
install 0.6.0-nros1, front  -> …/0.6.0-nros1/bin/nros    (newest wins)
install an OLDER one, front -> unchanged                 (no silent downgrade)
$NROS_HOME/bin/nros --codegen-version -> 1
```

### W4 — the download front door, and it is NOT `bootstrap.sh`

The paradox-breaker. Fetch `nros-<host>.tar.zst` + verify sha256, install through
W3, and keep the source build for an unsupported host and for contributors. The
book's step 2 stops being "build the CLI" for a user and stays exactly that for a
contributor.

**Landed 2026-09-06, with the shape changed.** This item was written as
"`bootstrap.sh` downloads by default, `--from-source` opts out". That is not
implementable, and the reason is structural rather than a preference:

* `bootstrap.sh` lives IN a checkout, so its download would put a released
  binary on a contributor's PATH beside that checkout's sources;
* a released binary run against a nano-ros checkout is refused by W1 and
  reported as a `[FAIL]` by W2 — both deliberately;
* and it cannot BE the checkout's binary either: `nros source-stamp` compares
  against the sources it was built from, so a release from another commit reads
  stale the moment it is copied into `packages/cli/target/`.

That is RFC-0090's thesis, not an inconvenience: the release is for people
building THEIR workspace, and inside nano-ros the tree's own build is the only
correct binary. It also matches the standing constraint that the checkout must
win locally. So the two front doors **split by audience**:

| | who | what |
| --- | --- | --- |
| `scripts/bootstrap.sh` | contributors, in a checkout | builds from source (unchanged) |
| `scripts/install.sh` | users, anywhere | downloads a release into the store |

```
curl -fsSL https://raw.githubusercontent.com/NEWSLabNTU/nano-ros/main/scripts/install.sh | sh
```

POSIX `sh`, because it runs from a curl pipe on a machine with nothing
provisioned — and with no `nros`, which is the bootstrap paradox it exists to
break. It never touches a checkout: it installs into `<store>/nros/<version>/`
and calls the binary it just placed with
`nros sdk-front nros --front bin/nros`, so "which version does `nros` mean" has
exactly one implementation (W3's `front_newest`) and the shell does not get a
second opinion. `--front` exists for precisely this caller: a user outside a
checkout has no `nros-sdk-index.toml` until the install finishes.

**The checksum is mandatory, not best-effort.** An asset served over a hijacked
CDN and an asset that arrived intact look identical to `tar`, and the person
running this cannot inspect what was fetched. A missing `.sha256` is a refusal,
not a fallback. `zstd`'s absence is probed BEFORE the download, the same rule
`sdk_store::execute` follows (issue 0385): otherwise it fails deep inside `tar`
with `zstd: Cannot exec`, after the bytes are already down.

**The store version comes from the asset**, at `share/nros/VERSION`, not from
`nros --version`: those differ by the `-nrosN` repackaging counter
(`0.5.0` vs `0.5.0-nros1`), and the wrong one means a later
`nros setup --tool nros` installs a SECOND copy of the same binary under the
other name. W5's release job writes that file from the index. A pre-W5 asset
falls back to `--version` and says so.

`tests/nros-installer-tests.sh` (`just check nros-installer`, on the fast lane)
serves a tarball built from the checkout's own `nros` over loopback, so it needs
no release to exist and reaches no network. Five arms, and the refusals are the
interesting half — an asset that installs anyway leaves a binary emitting code
into someone's workspace:

```
happy path        -> installed at <store>/nros/9.9.9-nros7, fronted, runs
version pinning   -> the prefix is the ASSET's version, and the only one
bad checksum      -> refuses, and installs NOTHING
absent .sha256    -> refuses (unverified is not a fallback)
no asset at all   -> refuses, naming the source build as the way forward
```

W2's gate moved with the decision: it flagged `bootstrap.sh` without
`--from-source` as the release path, which is now false. It flags
`scripts/install.sh` instead — bootstrap building from source is exactly what
the gate wants.

### W5 — the release workflow

Manual dispatch plus an explicit tag, mirroring `nano-ros-sdk`'s `build-tool.yml`
rather than inventing a second shape. Never a local build — the SDK repo's own
convention, and the reason is reproducibility rather than convenience.

**The release must assert what it ships**: the built binary's
`--codegen-version` against the tree it was built from, so a release cannot go
out claiming a compatibility it does not have.

Two more requirements, discovered by W4 and recorded here rather than left to be
rediscovered:

* **the asset carries `share/nros/VERSION`**, written from `[tool.nros].version`
  in the index, because the store prefix must be the version the index pins and
  `nros --version` prints a different string;
* **the asset ships a `.sha256` beside it**, because `install.sh` refuses an
  asset it cannot verify — a release without one is un-installable, by design;
* and the asset must be **prefix-rooted** (`bin/nros`, `share/…`), the mirror
  shape every other dist in the index uses, so it lands in the store unchanged.

**Landed 2026-09-06** as `.github/workflows/release-nros.yml`. Nothing is
scheduled and nothing triggers on a tag: the tag is created BY the workflow,
from the version you type, and `publish` defaults to **false** so a dispatch
builds and verifies without releasing anything.

*What it asserts*, each because it is a property a user cannot check for
themselves:

| check | what it catches |
| --- | --- |
| `nros source-stamp` | the binary does not match the checkout it was built from |
| `--codegen-version` vs `NROS_CODEGEN_VERSION` in the same tree | a release claiming a compatibility it does not have — the failure that withdrew the last one |
| `[tool.nros].version` == the dispatched version | an asset whose store prefix is not the one the index pins |
| the version is `<crate>-nrosN` | releasing `0.6.0-nros1` off a 0.5.0 tree |

*And it installs its own asset before publishing it.* The installer's refusals
are gated against a synthetic tarball (W4); only this catches an asset that is
well-formed and **wrong** — a missing `VERSION`, a prefix rooted one directory
too deep, a binary that does not run on a clean runner. It serves the tarball
over loopback and asserts the fronted binary runs `--codegen-version` and
`setup --list` **from a directory that is not a checkout**.

That last assertion found a real gap. **A released `nros` could not run
`nros setup <board>` at all**: `locate_index` looked in the cwd and in a
workspace, both of which assume a checkout, and the whole point of shipping a
binary is that there is no checkout. So the asset carries
`share/nros/nros-sdk-index.toml` and `setup::shipped_index` resolves it from the
running executable — from the executable rather than `$NROS_HOME`, so two
versions in the store each answer with their own and stay independently
installable. `resolve_index` falls back only for the DEFAULT: a `--index` the
user typed names a file they chose, and silently reading another one is how a
build gets provisioned from a table nobody looked at.

*Runner and reach.* `ubuntu-22.04`, pinned rather than `latest`: the binary
links the runner's glibc, and a newer one will not run on the LTS the book's
install instructions target — the floor is a property of the release. Linux
x86_64 only; `linux-arm64` is one matrix row whenever a runner is decided on,
and the index already carries the host key.

*The gate exempts it, in one direction only.* `check-ci-cli-from-source`'s arm A
does not apply to a workflow that PRODUCES the release — it names the asset a
dozen times because it builds one, and it runs `scripts/install.sh` on purpose.
Arm B still applies, and that is what keeps the exemption from being a hole:
mutation-verified by deleting the `source-stamp` line, which turns the gate red.

`check-workflow-indexed-apt` caught the workflow restating `zstd` as an apt
package name; it now asks `scripts/sdk/prereq-packages.py`, since the index
carries the per-manager spellings and a workflow copy is the one that goes
stale.

Rehearsed locally end to end — build, verify, stage, serve, install, front, run
outside a checkout — before the workflow was committed, because a release
workflow's first real run should not be its first run.

### W6 — the docs stop describing a source-only distribution

`book/src/getting-started/installation.md` currently carries a note explaining
why step 2 builds rather than downloads, added by phase-429 W7. That note becomes
wrong on the first release and should be replaced, not deleted — the reason
prebuilt binaries were withdrawn is worth keeping, with the answer now attached.
`AGENTS.md` and `scripts/bootstrap.sh`'s header (*"there is no prebuilt `nros`
download"*) likewise.

**Landed 2026-09-06.**

The claim that needed replacing was **"there is no prebuilt `nros`"**, in 12
places across 11 files. That sentence is a fact with an expiry date, and the
replacement deliberately is not: the durable statement is about **audience**, not
existence — a checkout builds its own binary because that is the only one it
accepts, and a user installs a release because they have no checkout. That reads
correctly before the first release and after it.

The four-step quick start stays a *contributor* flow (it opens with `git clone`),
with the user path stated above it as a table. `installation.md` and
`reference/cli.md` lead with `install.sh`; the seven getting-started pages carried
the same parenthetical and were rewritten together rather than one at a time.
`CONTRIBUTING.md` gains the sentence a contributor actually needs — *do not
install a release here, `just doctor` fails on it, and here is why* — since a
contributor who has both is the person this phase's hazard is aimed at.

One thing is said once and not eleven times: **no release is cut yet.** It sits
in `installation.md`, and `install.sh` says the same thing itself when run early,
so a user who skips the book still lands somewhere useful.

**Filed while verifying:** [#1110](../issues/1110-rustdoc-broken-intra-doc-link-in-clienttrait.md)
— `just book` and the docs deploy have been red since 2026-09-03. `3941b569a`
deleted `ClientTrait::is_server_ready`; the paragraph beside it still links to it,
rustdoc's broken-link lint is deny-level, and nothing on a merge-gating lane runs
rustdoc. Not this phase's change (`mdbook build` alone is green, and `just book`
was red before the edit and after it), and the class is 0319/0896 again: a correct
check on a path nothing traverses.

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
