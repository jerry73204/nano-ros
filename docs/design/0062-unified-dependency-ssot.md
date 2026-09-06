---
rfc: 0062
title: "One dependency SSoT, system-aware"
status: Stable
since: 2026-08
last-reviewed: 2026-08
implements-tracked-by: [phase-327, phase-398, phase-404]
supersedes: []
---

# RFC-0062 — One dependency SSoT, system-aware

**Status:** Stable (2026-08-30; Draft 2026-08-01, amended 2026-08-29)

Settled: `[prereq.*]` is the one declaration namespace, phase-398 landed it
end to end (`[system.*]` retired through alias -> warn -> gate -> delete, the
rosdep fallback deleted), and issue 0926 closed the last consumer gap — a
dist's runtime libraries are now measured rather than hand-listed, reported on
the path where the tool is USED, and gated by `check-dist-runtime-deps`.

One item is deliberately outside this RFC and tracked separately: re-cutting
dists with `$ORIGIN` rpath so a declared dependency disappears instead
(issue 0928). That is a nano-ros-sdk change, not a decision this RFC owes.
**Amends:** RFC-0014 (`nros setup` toolchain management) — extends its index
from two dependency classes to all of them; changes no existing `[tool.*]` /
`[source.*]` semantics.
**Amended:** 2026-09-04 (amendment 3) — who may NAME a key, and a
package's own buildtool resolves without ROS; vendor layout may source from the
submodule.
**Amended:** 2026-08-30 (amendment 2) — the provider is chosen by what the
tool DOES, `check` gains a version constraint, and every resolution reports
its provider; tracked by phase-404, no code yet.
**Amended:** 2026-08-29 — `[prereq.*]` (one key namespace over four providers),
unknown keys are an error, rosdep is no longer consulted, and `[system.*]`
MERGES into `[prereq.*]` and retires. See the amendment below; it REVERSES
§"System-aware resolution".
**Motivated by:** issue 0368 — a simulated end-user `just setup all` on a clean
Ubuntu 22.04 host failed 7 of 18 modules, nearly all on dependencies that were
declared nowhere (or declared as a Debian-only sudo list ordered in front of
the sudo-less installers it then aborted).

## Amendment (2026-08-29) — `[prereq.*]`: one key namespace, four providers, no rosdep

Motivated by a second instance of the class this RFC was written for. An agent
hit `libslirp.so.0` missing on the store's QEMU — a dependency **this index
already declares** (`[tool.qemu] system = ["libslirp"]`, probed by
`[system.libslirp] check.sharedlib`), with a comment saying it was declared
precisely so setup and doctor could say so "BEFORE the smoke check fails with a
bare loader error". It could not: nothing consulted the declaration on the path
where the tool is USED, so the loader spoke first. (Fixed separately — the store
resolver now probes.)

That is a consumer gap, not a schema gap. But it exposed the schema gap behind
it: **a user cannot declare a prerequisite at all.** `package.xml` `<depend>`
feeds build ORDER only, and a name that is not a workspace package is silently
ignored by construction. Every prereq in this tree is declared by the index, for
the index's own tools; nothing carries a user's.

### What changes

**1. `[prereq.<key>]` replaces `[system.<key>]` as the user-facing namespace,
and spans all four providers.** The providers already exist as separate classes
— `[system.*]` (OS package), `[tool.*].dist` (download), `[tool.*].source` +
`install` (build from source), `[source.*]` (submodule). What did not exist is
one name a consumer can write without knowing which of the four answers it.

```toml
[prereq.libslirp]                  # provider = "system" is the default
why      = "qemu -netdev user"
apt      = ["libslirp0"]
dnf      = ["libslirp"]
pacman   = ["libslirp"]
brew     = ["libslirp"]
check    = { sharedlib = "libslirp.so.0" }

[prereq.qemu]
provider = "sdk"                   # resolves through the existing [tool.qemu]
[prereq.freertos-kernel]
provider = "source"                # resolves through the existing [source.*]
```

`provider` defaults to `system`, so all 25 existing `[system.*]` entries are
valid `[prereq.*]` entries unchanged — this is a rename plus a default, not a
migration. `check` and `why` are kept and are the point: they are what makes a
missing prereq *diagnosable* rather than a loader error.

**2. Resolution is a ladder, and an unknown key is an ERROR.** For each
`<depend>` in a consumer's `package.xml`:

| rung | outcome |
| --- | --- |
| a workspace package | build ORDER, not a prereq — today's behaviour |
| a generated message package | `nros sync` owns it |
| a `[prereq.*]` key | its provider installs it; `check` decides present/absent |
| anything else | **error, naming the key** |

The message-package rung is not decoration. `std_msgs` is a legitimate key in
other ecosystems and a *generated* crate here; without an explicit rung, sync
and the prereq resolver both claim it and the winner is whichever ran last.

The last rung is the behaviour change, and it is the whole point. Today an
unrecognised `<depend>` is dropped in silence — the same silence that let a
declared dependency reach the dynamic loader. `NROS_ALLOW_UNRESOLVED_DEPS=1`
opts out for a tree mid-migration; it is an escape hatch, not a mode.

**3. rosdep is NOT consulted. This REVERSES §"System-aware resolution" above.**

That section makes rosdep an optional resolver for managers the index does not
map, and reports 12 of 24 keys resolvable from the public database. The
reversal is deliberate and the reasoning is not "rosdep is bad":

* **It answers for one provider of four.** rosdep has no concept of an SDK dist,
  a submodule, or a source build, so it can never be the resolver — only a
  partial one, which means every consumer needs the fallback logic anyway.
* **It cannot carry a `check`.** The probe is what turns "missing package" into
  a named remedy, and it is the half that would have prevented this
  amendment's motivating failure. A resolver that supplies packages but not
  probes leaves the diagnosable part to us regardless.
* **A resolver consulted only sometimes is a resolver whose behaviour depends on
  the host.** "rosdep is installed here and not there" makes the same tree
  resolve differently on two machines, which is the drift this RFC exists to
  delete.

**Keep the key NAMES rosdep uses** where one exists (`libslirp`, not `slirp`).
That is free, makes porting an existing rosdep list mechanical, and costs no
runtime dependency. Compatibility with the *database* was the only real prize,
and it is not worth a host-dependent resolver to reach half of it.

Consequence: the `rosdep_resolve` fallback in `cmd/setup.rs` (phase-327 W6)
becomes dead and should be deleted with this work, not left as an unreachable
branch.

### `[system.*]` MERGES into `[prereq.*]`, and retires

Settled by what the index already does, not by preference.

**A flat cross-provider namespace already ships.** `board.packages` is one list
per board, and its names resolve across FOUR classes today:

| name | resolves in |
| --- | --- |
| `qemu`, `arm-none-eabi-gcc`, `espflash`, `riscv-none-elf-gcc` | `[tool.*]` |
| `freertos-kernel`, `lwip`, `nuttx-{libc,kernel,apps}`, `threadx*` | `[source.*]` |
| `genromfs` | `[tool.*]` **and** `[system.*]` |
| `arm-fvp` | `[gated.*]` |

So `[prereq.*]` does not invent a flat namespace. It names the one boards have
been using, and gives it a declaration table.

**One key already needs several providers, and says so in prose.** `genromfs`
exists in two tables deliberately — `[system.genromfs].why` reads "the
`[tool.genromfs]` source recipe is the store alternative". That is *one
prerequisite, ordered providers* expressed as key duplication plus a comment
tying the halves together. Nothing enforces that the two stay in agreement, and
nothing tells a resolver which to prefer.

**`[system.*]` carries no field `[prereq.*]` would lack** — `why`, the four
manager maps, `check`. It is precisely the `provider = "system"` case.

Therefore: **one table.** `[prereq.<key>]` with an ordered `providers` list
replaces the duplication:

```toml
[prereq.genromfs]
why = "NuttX riscv rv-virt etc/ ROMFS image"
providers = ["system", "source"]   # ordered: prefer the OS package, build if absent
apt = ["genromfs"]; pacman = ["genromfs"]
source = "genromfs"                # the existing [tool.genromfs] source recipe
check = { cmd = "genromfs" }
```

#### Retirement, with the mistake this phase already made

`[system.*]` is parsed as an alias lowering to `provider = "system"`, warns, and
is deleted at the next minor version. That is W1.f's pattern — and W1.f is
exactly why the steps below are explicit, because it shipped a correct,
well-tested deprecation lint that **no production path ever called**, so the
warning reached nobody and the removal would have landed on users who were never
told.

1. `[prereq.*]` lands; `[system.*]` parses as an alias. No behaviour change.
2. The deprecation warning is **wired at index load and a test asserts it is
   reached** — not merely that the lint is correct in isolation. A lint proven
   only by direct unit-test calls is proven against the one caller that is not
   the problem.
3. A gate rejects a key declared in two provider tables. That makes the
   `genromfs` shape illegal once its merged entry exists, so the duplication
   cannot silently return.
4. `[system.*]` is deleted at the next minor — and only after the warning has
   actually shipped in a release, not merely been written.

### The `check` vocabulary

**Today only `[system.*]` has probes: 22 of 25 entries. `[tool.*]` has none of
14, `[source.*]` none of 15.** Presence for those two is implicit — the store
path or the checkout directory exists — which is the state the motivating
failure exploited: the QEMU dist was present by that test and unusable.

The design follows from `genromfs` again. Its probe is `check = { cmd =
"genromfs" }`, and that is correct **whichever provider installed it** — apt or
the source recipe. So:

> **`check` answers "is the capability usable?", never "did provider X install
> it?"** It stays one provider-independent vocabulary. Providers contribute
> INSTALLATION knowledge, not DETECTION knowledge.

That splits cleanly into two questions that were being conflated:

* **`check` — is it usable now?** Provider-independent, OR-ed, tri-state
  (`Present` / `Missing` / `Unknown`). Extends to every provider unchanged.
* **provider verification — is what we installed still what we declared?**
  Store path + version, submodule rev, dist sha256. Already exists in
  `[tool.*]`/`[source.*]`; stays there. It is a different question and it
  belongs to whoever did the installing.

`Unknown` is load-bearing and any new kind must be able to return it: a probe
that cannot answer on this host must not vote (issue 0487 — libgcrypt ships
`.pc` on Arch and `libgcrypt-config` on Ubuntu, so either probe alone is a false
negative on one of them, and a false negative here prints a sudo command for a
package that is already installed).

Two kinds the current four cannot express:

```toml
# RUNS — the resolved binary executes. Not `cmd`: that is `command_exists`, a
# PATH lookup, and a store dist is not on PATH. This is the probe the motivating
# failure needed — QEMU's path existed, and the dynamic loader was the first
# thing to disagree.
#   Unknown when the tool targets a foreign platform (cross toolchains,
#   emulator-less hosts): "cannot execute here" is not "absent".
check = { runs = "qemu-system-arm --version" }

# PATH — a file that must exist inside a checkout. For `source`/submodule
# providers, which have no PATH entry and no soname to probe, so today their
# presence test is "the directory exists" — true of an empty uninitialised
# submodule.
#   Relative to the provider's own `dest`, so the probe does not restate a
#   location the provider already declares.
check = { path = "include/FreeRTOS.h" }
```

Both compose with the existing OR: `{ cmd = "genromfs", runs = "genromfs -h" }`
is present if either answers, which is the libgcrypt rule applied to a stronger
probe.

### What this amendment still does not decide

* ~~**Whether `providers` is ordered preference or a fallback chain with a
  policy**~~ — **DECIDED by amendment 2 below (2026-08-30): ordered
  preference, and the ORDER is derived from what the tool does rather than
  authored per entry.** The `--offline` tension is resolved there too.
* **Whether `check` becomes REQUIRED.** Three `[system.*]` entries have none
  today (`ros-rmw-zenoh-cpp`, `python3-venv`, `picolibc-riscv64-unknown-elf`)
  and report `UNPROBED`; requiring one would force an answer for each, which may
  not exist.

## Amendment 2 (2026-08-30) — the provider is chosen by what the tool DOES

Motivated by a question this RFC's own framing invited: if a tool is available
from the system, what is the point of shipping a copy? The first answer — "we
ship it when the binary is patched" — is close, and measurement shows it
explains the wrong thing.

**Only three tools carry a patch**: `qemu` and `cyclonedds` (NEWSLabNTU forks)
and `play_launch` (ours). By a patched/unpatched rule the other eleven become
candidates for deletion. They should not all move, and the reasons differ per
tool, which is the tell that the criterion is wrong.

### The axis is not "patched", it is two questions

**1. Does its output enter a build artifact?** Then PIN it, whatever the system
has. A compiler, Corrosion, the codegen tool, `idlc` — these decide what the
produced binary IS, and a host-varying build input is this tree's most expensive
bug class: museum binaries, the accumulating store shadowing a pin (issue 0500),
the loader picking a library nobody chose (0774). Corrosion is the concrete
case: `< 0.6.0` shares one `cargo/build` across workspace roots, so `mixed`
fails to link with duplicate `#[no_mangle]` symbols — "whatever the host has" is
a landmine, not a convenience.

**2. Must it match something the host already runs?** Then the system copy is
not merely ALLOWED, it is MANDATORY, and shipping our own is a defect. This is
not new: RFC-0075 already deleted our vendored `zenohd` and resolves ROS's
`rmw_zenohd` instead, because a pinned copy drifted from the zenoh that
`rmw_zenoh_cpp` links (issue 0609 measured 0.1.1 -> 0.1.9 moving zenoh 1.2.0 ->
1.8.0 with our pin taking no part). The same binding holds cyclonedds to the
version ROS ships (0507). Amendment 2 generalises RFC-0075 rather than inventing
a policy.

Everything that is NEITHER — a debugger, a flasher, an on-chip debug server — is
the real "prefer the system if it is good enough" population.

### Why "good enough" is currently undecidable

`check` is presence-only: `cmd`, `sharedlib`, `pkg_config`, `header`. **There is
no version constraint anywhere in the index.** So "use the system copy when it
satisfies the pin" cannot be expressed today, and that — not policy — is what
blocks the whole idea.

Measured on Ubuntu 22.04, which is why the constraint must be real rather than
assumed:

| tool | pin | apt candidate |
| --- | --- | --- |
| `sccache`, `corrosion`, `espflash`, `xrce-agent` | — | **not packaged at all** |
| `arm-none-eabi-gcc` | 13.2 | 10.3 (2021) |
| `riscv-none-elf-gcc` | 14.2 | 10.2, and a DIFFERENT triple (`riscv64-unknown-elf`) |
| `openocd` | 0.12.0 | 0.11.0 |
| `genromfs` | 0.5.7 | 0.5.2 |
| `qemu` | 11.0.0 (fork) | 9.0.2 |
| `cyclonedds` | 0.10.5 (fork) | 0.8.2 |

Four of ten are not packaged; most of the rest are years behind. So on this host
"prefer system" would rarely fire — and on Arch, Fedora or nixpkgs the table
looks very different. That asymmetry is the argument FOR the mechanism (it is
what makes a non-Ubuntu host first-class, cf.
`docs/development/ros2-on-non-ubuntu.md`) and AGAINST deleting dists on the
strength of one distro's archive.

### What this amendment decides

1. **`providers` is ORDERED PREFERENCE**, closing amendment 1's open question.
   The order is not authored per entry by taste; it follows from the two
   questions above, and an entry that departs from its category says why.
2. **`check` gains a version constraint** — a floor plus a declared way to read
   the installed version. Without it, preference is unimplementable.
3. **Every resolution REPORTS its provider.** `nros setup --check` and
   `just doctor` must say which provider satisfied each tool, and at what
   version. A host that quietly used its own `openocd` 0.11 otherwise makes
   "works on my machine" unfalsifiable — the same reason issue 0929's `smoke`
   probes assert on OUTPUT rather than exit status.
4. **`--offline` and air-gapped hosts pin the order to the store.** Amendment 1
   left this tangled with RFC-0065 D14; it is not tangled. Preference is a
   DEFAULT, and `--offline` is an explicit override that removes `system` from
   consideration — the same shape as every other escape hatch here.

### What it does NOT decide, deliberately

* **Which dists actually stop shipping.** That is a per-tool call against the
  measured table once the constraint exists, not a category sweep. Nothing is
  deleted by this amendment. **phase-404 W4 has since classified all 14: nine
  are pinned build inputs, two are patched forks, two are unpackaged, and ONE
  (`openocd`) was opted in. Still nothing deleted — preference removes the
  requirement to install, not the fallback.**
* **Whether a system-provided BUILD input is ever acceptable.** Question 1 says
  no today. If that is ever relaxed it needs its own evidence, because the whole
  reproducibility argument rests on it.
* ~~**How a version is extracted per tool.**~~ **DECIDED in phase-404 W1: a
  known-shape default, no regex.** The scan takes the first version-shaped token
  (a digit run containing a dot), which reads every tool this index pins —
  `Open On-Chip Debugger 0.12.0-g9ea7f3d`, `QEMU emulator version 11.0.0`,
  `arm-none-eabi-gcc (Arm GNU Toolchain 13.2.rel1 ...)`. The one escape hatch is
  `after = "<literal>"`, deliberately not a regex: a pattern per entry is a
  second place for the index to drift.

### Cost of being wrong in each direction

Shipping a dist we did not need costs bytes and a re-cut nobody reads. Using a
system copy we should have pinned costs a build that differs per host and fails
somewhere else entirely — which is the failure this repository has paid for most
often. When the evidence is ambiguous, pin.


## Amendment 3 (2026-09-04) — who may NAME a key, and who satisfies a buildtool

Two questions this RFC had not separated: what a key *is* (settled — one
namespace, four providers) and **who is allowed to name it from a
`package.xml`**. Measuring the tree made the gap concrete.

### The measurement

407 `package.xml` files declare 65 distinct dependency tokens. Of the 46
`[prereq.*]` keys, **exactly three are ever named by a package.xml**: `cargo`,
`cmake`, `nros`. The other 43 are provisioning facts that no package's CONTENT
depends on, in five groups:

| group | n | example | what ROS 2 does with it |
| --- | --- | --- | --- |
| runtime closure of a dist we ship | 11 | `libssl3` | never hand-declared; comes with the binary package |
| build tooling | 16 | `ninja`, `doxygen` | `buildtool_depend` / `doc_depend` — belongs, if a package needs it |
| cross toolchains, emulators, probes | 7 | `qemu-system-arm` | outside rosdep entirely (sysroots, containers) |
| vendored source trees | 6 | `zenoh-pico` | a `*_vendor` PACKAGE, not a key |
| ROS binary package | 1 | `ros-rmw-zenoh-cpp` | plain `<depend>` |

`<depend>qemu-system-arm</depend>` resolves silently today. It should not: QEMU
runs the test lane, it is not a dependency of any package's content, and
resolving it teaches the wrong model.

### DECIDED — a package's own buildtool needs no provider

`<buildtool_depend>ament_cmake</buildtool_depend>` on a package whose
`<build_type>` IS `ament_cmake` is a tautology: if that builder is building it,
the buildtool is present. `Resolution::SelfBuildtool` resolves exactly that,
and it is **off-ROS safe** — which the alternatives were not.

The alternatives were rejected on evidence, not taste:

* **Rely on the ROS rung.** `ros_packages()` reads `AMENT_PREFIX_PATH` and
  returns empty when ROS is not sourced, which is the normal state for an
  embedded-only contributor. Declaring the buildtool would then hard-fail
  `nros build` for a dependency satisfied by definition.
* **Add `[prereq.ament_cargo]` with `apt = [...]`.** Fiction: `ament_cargo` and
  `cargo-ros2` are served by this repo's own
  `packages/cli/colcon-cargo-ros2` colcon extension. No apt package provides
  them, and an index row claiming otherwise is a lie a user would act on.

Two properties make the rung safe rather than a blanket exemption. It sits
**last**, below the ROS rung, so a real provider still wins and gets reported —
that is the answer a user can act on. And it is **conservative**: a name
qualifies only if EVERY package.xml declaring it has a build type implying it,
so one package declaring `ament_cmake` while built another way keeps the name on
the normal ladder.

The build-type mapping is nearly total, which is why inference is worth having:
**345 of 367 packages declare a `<build_type>` and NOT the matching
buildtool**, and the only declarations that are not inferable are
`rosidl_default_generators` (10 message packages) and `cargo-ros2` (1) — which
is exactly what a human should still write by hand. The nano-ros build types
(`ament_nros`, `nros_entry`, `nros_cargo`, `nros_bringup`) map to `nros`: they
are served by this repo's builders and have no upstream buildtool to name.

### DECIDED — a vendor package may take its source from the submodule

The ROS answer for a vendored tree is a `*_vendor` package: a real
`package.xml` plus CMake that fetches and builds pinned upstream source. Adopting
the LAYOUT is compatible with keeping the submodule as the SOURCE:

```cmake
ExternalProject_Add(zenoh_pico
  SOURCE_DIR   "${NROS_ZENOH_PICO_DIR}"   # no GIT_REPOSITORY, no GIT_TAG
  BINARY_DIR   "${CMAKE_CURRENT_BINARY_DIR}/zenoh_pico-build")
```

The decisive property is **one pin, not two**. A `GIT_TAG` in CMake is a second
pin that can drift from the gitlink, and the gitlink is the one
`check-submodule-pins` (forward-only), the pre-push hook and
`diff.submodule=log` already guard — protections built after a zenoh-pico pin
silently rewound over a Zephyr build fix for seven hours. Cloning the submodule
shallowly also works, but needs a `GIT_TAG` to clone *to*, so it reintroduces
the second pin for no gain: the submodule is already a local checkout at the
right commit.

Prototyped and verified: configures clean, an uninitialised submodule fails with
the `git submodule update --init` remedy rather than a downstream error, and the
submodule working tree stays clean because `BINARY_DIR` is out of tree.

### DECIDED — `role`, and a consumer for it in the same change

`[prereq.*]` gains `role`: `package` / `workspace` / `infra` / `vendor`. All 46
keys carry one (11 / 11 / 19 / 5), and `check-prereq-roles` keeps them carrying
it — the Rust field defaults to `unclassified` so it could land incrementally,
and without the gate a new key inherits that default silently.

Role landed WITH its consumer, deliberately. A classification nothing reads is
another authored map waiting to drift, which is the failure this RFC keeps
meeting. The consumer is `nros setup --workspace <path>`: it reads the three
things a tree already states — `<depend>` (content),
`<build_type>`/`<buildtool_depend>` (builder), and
`<export><nano_ros deploy=.. board=.. rmw=../>` (target) — and reports what to
provision. That deploy export is not new: 90+ packages carry it, and
`build.rs`, `doctor.rs` and `workspace.rs` all read it while `setup.rs` alone
ignored it.

**REFUSING a wrong-role dep is deferred**, on purpose. `<depend>qemu-system-arm
</depend>` still resolves; `--workspace` REPORTS it as a category error without
failing. Turning it into an error breaks someone's working tree, so it wants a
deprecation window rather than a flag day.

### Measured while building it — three findings this RFC should carry

**1. 24 of the 46 keys have no explicit consumer.** Nothing in the index and no
recipe references them; they are reachable only if a user happens to run the
blanket `nros setup --system`. They are not marginal: `cmake`, `ninja`, `make`,
`cargo`, `nros`, `python3-*`, `gnu-parallel`, and every cross toolchain.

**2. Nothing in the repo INSTALLS via `--system`.** Five recipes name it only
inside an error message; `just workspace apt-packages` PRINTS the command (by
design — composing it is the tool's job, running it is the user's), and
`just setup <scope>` never reaches even that. Simulated on a developer host: 38
present, 5 missing, 3 unprobed, and `just setup native` pulls none of the five.
So the documented bootstrap does not provision the system closure, and a user
discovers it when a build fails.

**3. `board=` in a package.xml export is NOT the `[board.*]` key namespace.**
Of the five boards declared across this repo's examples, only `threadx-linux`
is an index key — `nros setup mps2-an385-freertos` and three others would fail.
Nor is a `deploy=` value always a scope: `threadx` splits into `threadx_linux`
and `threadx_riscv64` by board. Both were caught by SIMULATING the new command
rather than reasoning about it, and both would have shipped as remedies that
fail for the out-of-tree user this mode exists for. `--workspace` now validates
against the index and `scripts/build/scope.sh` before printing any command.

Finding 3 is a two-vocabularies defect of the same class this RFC exists to
delete, one layer out: the same concept spelled two ways in two files, with
nothing asserting they agree.

### What this amendment does NOT decide

* **When the refusal lands**, and what the deprecation window looks like.
* **Whether the 11 runtime-closure keys move under their owning `[tool.*]`.**
  `libssl3`'s own `why` already says "runtime dep of the cyclonedds,
  xrce-agent dist(s)" — it is a property of those tarballs, not of the
  workspace. Mechanical, but it changes the index shape, and issue 0928's
  `$ORIGIN` re-cut may delete the need entirely. Sequence matters.
* **Whether the 345 missing `<buildtool_depend>` declarations get swept in.**
  Now UNBLOCKED by the rung above, and mechanically derivable from
  `<build_type>`. Still a 345-file diff whose only benefit is rosdep parity for
  consumers who run rosdep — which this project deliberately does not.
* **Which vendored trees adopt the vendor-package layout.** The hybrid works;
  whether six submodules should each grow a `package.xml` is a cost question,
  not a correctness one. A downstream consumer wanting `zenoh_pico_vendor`
  WITHOUT our submodule layout would need a `GIT_REPOSITORY` fallback — and
  that path has two pins again, so it should be decided, not defaulted into.


## Amendment 4 (2026-09-06) — the four axes, and the two rungs a shipped binary breaks

Amendment 3 settled who may NAME a key. This settles **where the answer comes
from when no package names one**, which is the majority case: 375 of 406
`package.xml` files declare a `<build_type>` and their content depends on no
system library at all.

Opened by phase-413 W6, which asked for rosdep parity and required an RFC first.
The answer is that most of it already exists, in three places that had not been
stated together.

### The four axes

A system dependency arrives on exactly one of four axes, and **`package.xml` is
the only input** — never a `CMakeLists.txt`, never a `Cargo.toml`
`[package.metadata.nros.entry]`. Those are build-system facts, they take several
shapes across the example trees, and a resolver that read them would inherit
every shape.

| axis | question | declared as | resolves into |
| --- | --- | --- | --- |
| build_type | how is it built? | `<build_type>` | `Resolution::SelfBuildtool`, below |
| depend | what does the content need? | `<depend>` | `[prereq.*]` via the ladder |
| board | where does it deploy? | `<nano_ros_uses kind="board">` (RFC-0087 D3), sugar `board="…"` | `[board.*]` |
| rmw | which transport? | `<nano_ros_uses kind="rmw">`, sugar `rmw="…"` | `[rmw.*]` |

The first two are properties of the package. The last two are **selections the
workspace has made** — which is why they are an export tag rather than a
`<depend>`, and why an RMW is never a `<depend>` here or in ROS 2.

Measured 2026-09-06: 91 `package.xml` files carry the export tag, and every value
resolves — `board=` 50 (`riscv64-qemu` 13, `mps2-an385-freertos` 12,
`nuttx-qemu-arm` 12, `threadx-linux` 12, `nuttx-qemu-riscv` 1), all `[board.*]`
keys; `rmw=` 51, all `zenoh`. `check-board-vocabulary` already gates the pair in
both directions.

**No new field is needed for the board axis**, and an earlier draft of this
amendment proposed one. It reached for a bridge from an entry's `Cargo.toml`
`deploy =` to a `[board.*]` row, having measured that 37 of 67 such entries name
no index key. They name a board DESCRIPTOR alias instead — 10 of 10 resolve
there — because `deploy=` selects a board implementation and, per RFC-0087 D3, is
not a provider kind at all: it names a `[deploy.*]` block in `system.toml`. Two
vocabularies, each internally consistent, answering different questions. The
lesson is the one this tree keeps relearning: when two vocabularies appear to
disagree, find which question each answers before proposing to unify them.

### The two rungs a shipped `nros` breaks

`Resolution::SelfBuildtool` rests on "if that builder is building it, the
buildtool is present". Phase-431 makes that false for the nano-ros build types.
Before it, having `nros` implied having cargo — you built the CLI from source.
Once `nros` ships as a prebuilt binary, a user can hold the builder and have no
`cargo` and no `cmake`, and `nros_cargo` / `nros_cmake` are 375 of 406 packages.

This does NOT reopen the `[prereq.ament_cargo]` option Amendment 3 rejected:
that was fiction because no apt package provides `ament_cargo`. `cargo` and
`cmake` are real packages already in `[prereq.*]`, and the missing part is only
that nothing reaches them from a `<build_type>` — measured, zero of the eleven
`role=package` keys is named by any `[board.*]` or `[rmw.*]` row, so today they
are reachable only by `--role package --role workspace`, a bulk list rather than
a resolution.

**DECIDED — a build type may name the host tools its builder shells out to**,
consulted only when `SelfBuildtool` cannot answer because the builder is not the
one that provides them:

```toml
[build_type.nros_cargo]  packages = ["cargo"]
[build_type.nros_cmake]  packages = ["cmake", "clang"]
```

**And a fifth role, `buildtool`.** Amendment 3's own table already groups 16 keys
as "build tooling", but `PrereqRole` has `Package`, `Workspace`, `Infra`,
`Vendor` and `Unclassified` — nothing for "comes from how you build". So seven
build tools (`cargo`, `cmake`, `clang`, `libclang-dev`, `python3-{dev,pip,venv}`)
are filed `role = "package"` and `nros setup --workspace` reports `cargo` under
*"SYSTEM PREREQUISITES this workspace names"*, which is not what a `<depend>`
means.

### ROS package dependencies resolve by derivation

Amendment 3's table ends with one row: *ROS binary package | 1 |
`ros-rmw-zenoh-cpp` | plain `<depend>`*. Today such a name takes the
`Resolution::RosPackage` rung — the ambient ament index — and falls to `Unknown`
when ROS is not sourced, which is the normal state for an embedded contributor.

The OS package name is derivable. All nine ROS names this tree composes by hand
follow one rule with no exceptions: `ros-<distro>-<pkg with _ → ->`.

```toml
[prereq.rmw-zenoh-cpp]
role = "package"
ros_package = "rmw_zenoh_cpp"     # the ROS name; the OS name is DERIVED
```

| manager | pattern | status |
| --- | --- | --- |
| apt | `ros-{distro}-{pkg _→-}` | verified — bloom's convention; all nine match |
| pacman | `ros2-{distro}-{pkg _→-}` | **UNVERIFIED** — check against a real package list before shipping |
| dnf, brew | unmapped | ROS is not packaged this way; "unmapped" is already an answer |

An explicit `apt = [...]` overrides the derivation for one key, so a rename or a
split between distros is expressible without a migration.

**Rejected: deriving with no index row at all.** rosdep can, because it validates
against the rosdistro index — a network fetch of a large YAML. We have no such
oracle, so a typo would stop being a gate failure and become an `apt` failure on
a user's machine.

**This supersedes the `{ros_distro}` placeholder** (issue 1128, landed
2026-09-06 as the interim). It leaves the placeholder with zero users, at which
point `check-prereq-placeholders` fails by design — it refuses a vocabulary entry
the index does not use — and the two expanders it guards are deleted with it.

### What this does not fix

* `[board.native]`, `[board.posix]` and `[board.zephyr]` resolve to nothing. For
  native and posix that is defensible; **for zephyr it is a hole** — the SDK,
  west and the python classes all exist, and provisioning reaches them through
  the `just zephyr` module instead, which is the second path phase-422 spent
  four work items removing elsewhere.
* 40 of the 91 export tags carry no `board=` (28 `deploy="native"`, 12
  `deploy="zephyr"`) — harmless for native, the same zephyr hole from the
  manifest side.
* `threadx` is an ambiguous alias, claimed by two descriptors
  (`nros-board-threadx-linux`, `nros-board-threadx-qemu-riscv64`). No entry uses
  the bare name today; a gate should refuse it before one does.
* The example `CMakeLists.txt` files take several shapes — 15 call
  `nano_ros_entry`, 83 `nano_ros_add_executable`, 68 `nano_ros_auto_add_library`.
  Resolution does not read them, so this blocks nothing here; it is a separate
  consolidation, worth doing before anything starts treating a build file as a
  declaration.
* A build-system fact can disagree with the manifest and nothing notices: an
  entry's `deploy =` and its package's export tag are never compared.

### Acceptance

1. `[build_type.*]` exists and is consulted only where `SelfBuildtool` cannot
   answer; `nros setup --workspace` reports buildtools under their own heading.
2. `role = "buildtool"` classifies the seven keys; `check-prereq-roles` enforces.
3. `ros_package` derives apt names; `{ros_distro}` and both expanders are deleted.
4. `[board.zephyr]` names what a Zephyr build needs, reached through the index.

*(RFC id 0092 was reserved for this and released unused: the decisions belong to
the RFC that already owns the question, and a new number citing two others is
worse than an amendment.)*

## Problem

nano-ros's dependencies live in five places with five owners:

| Class | Where declared today | System-aware? | Doctor-checked? |
| --- | --- | --- | --- |
| Pinned prebuilt dists (qemu-nros2, cyclonedds, …) | `nros-sdk-index.toml [tool.*]` | yes (per-host dists) | partially |
| Source/submodule deps (freertos, nuttx, …) | `[source.*]` | yes | mostly |
| OS packages | `just/workspace.just apt-packages` (Debian only) + ad-hoc probes inside module scripts (zephyr wants ninja/aria2c, esp_idf assumes python3-venv, rmw_zenoh prints a sudo apt line) | **no** | drifts |
| Rust layer (toolchains, targets, cargo tools) | `rust-pinned-toolchains` / `rust-targets` / `cargo-tools` recipe bodies | n/a | via recipes |
| Python layer (west, colcon, clang-format wheel, px4 requirements) | scattered pip calls per module | no | no |

The consequences measured in 0368:

- One sudo step (`apt-packages`) ordered first aborted the workspace module's
  own **sudo-less** installers, cascading into three other modules' failures.
- Doctor remedies drift from reality because they are hand-written per module:
  three pointed at apt/sudo where an index prebuilt already existed
  (riscv gcc, idlc-in-cyclonedds, play_launch_parser).
- Whole dependencies were simply undeclared until a clean host hit them
  (`python3-dev`, `libz3-dev`+`libclang-dev`, `libslirp0`, `aria2`,
  `parallel`, `cargo-nextest`).
- A prebuilt dist (`qemu-11.0.0-nros2`) has a RUNTIME system dep (libslirp)
  that no layer could express, so `nros setup` installed a binary that cannot
  execute and only its smoke check caught it.

## Decision

**`nros-sdk-index.toml` becomes the single declaration for every dependency
class.** Setup *and* doctor both derive from it — the remedy a doctor prints
is computed from the entry, never hand-written, which deletes the
remedy-drift class the same way RFC-0061's shared coordinate file deleted
lane-coverage drift.

### The classes

Existing, unchanged: `[tool.*]` (+ `.source`), `[source.*]`, `[gated.*]`.

New:

```toml
# -- OS packages, declared by ABSTRACT key, mapped per package manager. ------
[system.libslirp]
why  = "runtime dep of the qemu-nros dist"
apt    = ["libslirp0"]
dnf    = ["libslirp"]
pacman = ["libslirp"]
brew   = ["libslirp"]
check  = { sharedlib = "libslirp.so.0" }        # ldconfig / dlopen probe

[system.gnu-parallel]
why  = "just format fan-out"
apt = ["parallel"]; dnf = ["parallel"]; pacman = ["parallel"]; brew = ["parallel"]
check = { cmd = "parallel" }

# -- Rust layer. -------------------------------------------------------------
[rust.toolchain.nightly-pinned]
channel = "nightly-2026-04-11"
components = ["rustfmt", "clippy", "rust-src", "miri", "llvm-tools"]

[rust.target.riscv32imc]
triple = "riscv32imc-unknown-none-elf"
toolchain = "nightly-pinned"

[rust.cargo-tool.nextest]
crate = "cargo-nextest"; version = "0.9"; locked = true
check = { cmd = "cargo-nextest" }

# -- Python layer (versioned, into one managed venv or --user). --------------
[python.west]
pip = "west"; version = "1.2"
check = { cmd = "west" }
```

And one addition to the existing class: a `[tool.*]` may declare
`system = ["libslirp"]` — the runtime OS deps of its **dist** — so the qemu
failure mode becomes representable, checkable, and part of the printed plan.

### Consumers declare needs; nothing re-declares content

Boards and modules already reference `[tool.*]` sets (`board.packages`).
The same mechanism extends: a module's setup/doctor asks the index for its
`needs = [...]` closure. Module scripts stop carrying their own prereq
probes (the zephyr ninja/aria2c check, the rmw_zenoh apt hint, the esp_idf
venv assumption all move into entries).

### System-aware resolution

> **REVERSED 2026-08-29** — see the amendment at the top: rosdep is no longer
> consulted at all. The paragraph below describing it as an optional resolver
> is kept for the record of why it was tried and what it measured (12/24 keys),
> not as current behaviour.

`nros setup` detects the package manager (apt/dnf/pacman/brew; `os-release`
plus `command -v` fallback) and resolves `[system.*]` keys through the
in-index mapping. Where a key carries no mapping for the detected manager,
an optional **rosdep backend** fills it: the public rosdistro database
already resolves about half of nano-ros's current keys (measured 12/24), it
runs entirely sudo-less (`pip install --user rosdep`, `ROSDEP_SOURCE_PATH`
pointed at a repo-local sources list — no `rosdep init`), and an in-repo
overlay yaml covers the rest (validated end-to-end; transcript in issue
0368 / the 2026-08-01 session). rosdep is an *optional resolver*, not the
SSoT: keys and their primary mappings live in the index, so a host without
rosdep or network loses nothing on the mapped platforms.

### The sudo boundary

Resolution partitions the plan into:

- **unprivileged** — store dists, source builds, rustup, cargo installs,
  pip (venv/`--user`): `nros setup` executes these itself, ordered FIRST;
- **privileged** — the `[system.*]` closure: composed into **one native
  command per manager** and *printed* (or executed only under an explicit
  `--sudo` opt-in). A missing system package therefore degrades a module's
  setup to "here is the one command to run", never aborts the sudo-less
  remainder — the direct fix for 0368-F1.

### Doctor = the same walk, read-only

`nros doctor` runs each entry's `check` probe and prints the entry-derived
remedy (`nros setup --tool X` / the composed native command / `rustup
target add …`). Hand-written remedy strings in module doctors are replaced
by index lookups; a probe with no index entry becomes a lint (the
issue-0196 rule applied to dependencies: the gate must cover the class).

## What this deliberately does not do

- No new lockfile semantics: `nros-sdk.lock` keeps recording store installs;
  system packages are observed by probe, not pinned (distros own their
  versions).
- No containerization answer: images/devcontainers can be *generated from*
  the index later, but are out of scope here.
- No change to `[gated.*]` (licensed SDKs stay instruction-only).

## Migration (phase-327 — `docs/roadmap/archived/phase-327-unified-dependency-ssot.md`)

1. Add the three new classes to the index; move `apt-packages`' list, the
   module-local probes, and every 0368 discovery into entries. `apt-packages`
   becomes a thin printer over the resolver (fixes F1 ordering as a side
   effect).
2. Generic setup/doctor walkers over the index; delete per-module remedy
   strings as each module's entries land.
3. `[tool.qemu] system = ["libslirp"]` + re-cut of the dist (or rpath-bundle
   libslirp; either way the dep is now declared and checked).
4. Optional rosdep backend + overlay for unmapped managers.

## Evidence

- Issue 0368 — the clean-host walk this RFC answers, including the measured
  cascade and the full list of undeclared deps.
- rosdep feasibility test (2026-08-01): user-level install, no-sudo sources
  via `ROSDEP_SOURCE_PATH`, 12/24 public-db coverage, 9/9 overlay coverage,
  machine-readable `#apt` output — preserved in the session transcript and
  `tmp/rosdep-overlay-test/nano-ros.yaml`.
