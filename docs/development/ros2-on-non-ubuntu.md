# ROS 2 on a non-Ubuntu host (Arch, Fedora, NixOS …)

nano-ros itself needs **no** ROS 2 (see
[installation.md](../../book/src/getting-started/installation.md#do-i-need-ros-2-installed)):
setup, `nros sync`, codegen and the first-node flows all work without it. ROS 2
is required only for the interop side — `ros2` CLI verification, the
`rmw_zenoh_cpp` / cyclone / fastrtps interop cells, and the ROS-bridging lanes
of `just test-all`.

The problem: ROS 2 ships apt packages for one Ubuntu LTS per edition (humble →
22.04, jazzy → 24.04) and nothing else. On Arch the AUR `ros2-humble` package
source-builds the whole tree against the rolling python/boost/OpenCV and breaks
whenever those move — not worth the time. The repo also hardcodes
`/opt/ros/<distro>/setup.bash` in `activate.sh` and in
`packages/testing/nros-tests/src/ros2.rs`, so a relocated prefix (RoboStack,
nix-ros-overlay) does not drop in either.

**The route that works: an Ubuntu distrobox sharing your home and network.**
`/opt/ros/humble` exists inside it, the checkout keeps ONE absolute path, and
every documented lane runs unchanged.

## Setup

```sh
sudo pacman -S --needed distrobox        # or dnf/apt equivalent

# podman is not required if you already use docker:
DBX_CONTAINER_MANAGER=docker distrobox create -n ros2 -i ubuntu:22.04 \
    --volume /path/to/your/checkout/parent:/path/to/your/checkout/parent -Y

# ROS 2 Humble + the packages the interop lanes use:
distrobox enter ros2 -- bash scripts/dev/ros2-distrobox-setup.sh
```

Mount the checkout at the **same absolute path** it has on the host. Different
paths on the two sides is the issue-0375 hazard: `nros sync` writes absolute
paths, and every path-keyed cache then splits in two.

`ros-humble-desktop` is ~2 GB of apt; the script also installs the book's host
prerequisites, `rmw_cyclonedds_cpp`, `rmw_fastrtps_cpp`, `rmw_zenoh_cpp`,
`domain_bridge`, `example_interfaces`, `rosidl_adapter`, colcon and rosdep, then
verifies each.

`rmw_zenoh_cpp` is what the zenoh e2e lanes run as their router (phase-362 W1).
Without it they report `[SKIPPED:capability]`, which reads as green — three
issues were closed "unverifiable" that way on 2026-08-18. `just doctor` reports
which router it can see.

## Where the box's storage lands — check `/` before you create it

`ros-humble-desktop` plus the three RMWs is roughly 3–5 GB of image layers and
apt content, and BOTH docker and rootless podman keep that under `/` by default
(`/var/lib/docker`, `~/.local/share/containers`). On a workstation whose root
filesystem is nearly full, the documented `distrobox create` above will either
fail partway through the apt step or leave the system with no headroom.

Measured on the maintainer host, 2026-08-18: `/` at 97 % (9.1 GB free) while
`/mnt/wd` — the disk holding the checkout — had 218 GB. Three ways out, in the
order worth trying:

1. **Put rootless podman's storage on the big disk.** A user-level
   `~/.config/containers/storage.conf` with `graphroot` pointing at a directory
   on that disk; no sudo, reversible by deleting the file, and `/` is untouched.
   Create the box with the default manager afterwards (drop the
   `DBX_CONTAINER_MANAGER=docker` prefix).

2. **Use docker exactly as documented** and accept the space on `/`. Simplest
   and matches the Setup section verbatim — fine when `/` has ≥ 15 GB free.

3. **Install `ros-humble-ros-base` instead of `desktop`**, keeping
   `rmw-{zenoh,cyclonedds,fastrtps}-cpp`. Roughly half the size, and a
   divergence from `scripts/dev/ros2-distrobox-setup.sh` — so when a lane later
   wants a package `desktop` would have pulled, that divergence is what you will
   be chasing. Prefer 1 or 2 unless disk is genuinely scarce.

Whichever you pick, the checkout still has to be mounted at the SAME absolute
path it has on the host (issue 0375) — see "Two paths to the same checkout".

### What a host without the box gives up

Not a nicety. On a host with no ROS install, `just ci` on 2026-08-18 reported
**176 `[SKIPPED:capability]`** tests — the whole zenoh-router-dependent surface,
including every `native_api` interop case — and they are counted as skips only
because `test-all` rewrites them. The `check-required-features-tests` lane
(issue 0652) runs a BARE `cargo nextest`, where the same skips are FAILURES, so
tier 1 cannot go green on such a host at all. The box is what turns that surface
from "unverified, quietly" into coverage.

## The box gets its OWN tree (issues 0400, 0401)

Host and box originally shared this checkout, and every build artifact in it is
glibc- and toolchain-specific. That produced a different failure each time it
surfaced — a host-built `build-script-build` dying on `GLIBC_2.39`, a
host-configured CMake cache reusing `sccache` and then missing `strlcpy`, a
host-built `nros` that cannot exec, fixtures written where the tests do not
look. Five separate incidents, one premise.

Redirecting the box's `CARGO_TARGET_DIR` cannot fix it: the fixture contract is
LEAF-RELATIVE (`examples/**/target-<rmw>/…`), so moving cargo's output puts
fixtures somewhere the tests never stat (issue 0401). The two mechanisms are
mutually exclusive.

So the box builds in its own tree:

```sh
bash scripts/dev/ros2-box-sync.sh          # host -> <checkout>-box, one way
DBX_CONTAINER_MANAGER=docker distrobox enter ros2 -- bash -c '
    cd /path/to/<checkout>-box && . scripts/dev/ros2-box-env.sh && just ci'
```

A MIRROR, not `git worktree`: a worktree cannot check out the branch the host
has, and carries only committed state — the normal loop is edit, build in the
box, test, and a worktree would test the last commit instead of the edit. The
mirror includes `.git` (build stamps and the #409 play_launch pin check read
it) and excludes every build output.

Re-run `nros sync` inside the box for anything you build there: leaf
`.cargo/config.toml` files carry absolute paths (RFC-0048 W9), so a mirrored
leaf still points at the source tree. Same rule as any moved checkout.

The sync also prunes directories that are gone from the source. It has to do
that itself, because `rsync --delete` cannot remove a directory whose surviving
content is EXCLUDED — and excluding build output is what keeps the box's own
`target/` and `build/` alive across a re-sync. So a directory retired upstream
stayed in the mirror forever once it held any output, and rsync said so only as
a mid-transfer warning on stderr while still exiting 0:

```
cannot delete non-empty directory: examples/workspaces/ws-bridge-rust
```

That is a museum binary by construction — present, never rebuilt, and no gate
looks for a directory that is not supposed to exist. First sweep found twelve:
the eleven retired `ws-*` workspaces (phase-331 W3) and `examples/stm32f4`
(phase-337 W7.a), the latter still holding 1.2 GB.

`ros2-box-env.sh` detects a box-owned tree by its `.nros-box-tree` marker and
does NOT redirect `CARGO_TARGET_DIR` there. A checkout without the marker is
treated as shared and keeps the old redirect, because there the alternative is
host-built build scripts dying on glibc.

## Using it

```sh
distrobox enter ros2 -- bash -c '. scripts/dev/ros2-box-env.sh && <command>'
```

**If both docker and podman are installed, say which one holds the box:**

```sh
DBX_CONTAINER_MANAGER=docker distrobox enter ros2 -- ...
```

distrobox picks podman when it finds it, so on a host that gained podman AFTER
the box was created under docker, a plain `distrobox enter ros2` does not fail —
it reports `no such container "ros2"` and OFFERS TO CREATE a fresh Fedora one.
Answering yes gets you a second, empty box with no ROS in it while the real box
is still running. Check with `docker ps -a` / `podman ps -a` before believing
the box is gone. Export the variable for the shell if you use one manager only.

`ros2-box-env.sh` sources `activate.sh` and adds the box-local overrides.

**Join it to the command with `&&`, not `;`.** The script REFUSES two states —
a shared host tree (issue 0759) and a PATH where a host tool wins (issue 1144,
below) — by returning non-zero, and a `;` runs the command anyway, which is
exactly the run whose result cannot be trusted.

**PATH order does not depend on how you entered the box any more, and it used
to** (issue 1144). `ros2-box-env.sh` prepends `$CARGO_INSTALL_ROOT/bin` before
sourcing `activate.sh`, but `activate.sh` prepends `$HOME/.cargo/bin` when it is
not already on PATH — which in a **non-login** shell (`podman exec … bash -c`,
the fast way to drive the box) put the host's glibc-2.39 `just` back in front,
and it dies as `GLIBC_2.39 not found` naming neither PATH nor the box. A login
shell (what `distrobox enter` gives you) already had that dir on PATH, so the
guard skipped and nobody saw it. `ros2-box-env.sh` now seeds `$HOME/.cargo/bin`
at the TAIL first, so that guard is a no-op in both shell kinds, and then
ASSERTS the result: every tool the box installed for itself must not resolve
into `$HOME/.cargo/bin`, or sourcing fails with the shadowed pair named
(`nros_box_check_path`, re-runnable by hand). Sourcing `activate.sh` first and
`ros2-box-env.sh` second also works, but it is a workaround — box-env sources
`activate.sh` itself.

One-time per box. Every cargo-INSTALLED tool has to be reinstalled here: the
host's `~/.cargo/bin` copies are host-built and die with `GLIBC_2.xx not found`,
which surfaces as `just: … not found` or `no such command: nextest` rather than
as anything mentioning glibc.

```sh
cargo install just --locked
cargo install cargo-nextest --locked   # just test-unit / test / test-all drive it
cargo build --release --manifest-path packages/cli/Cargo.toml --bin nros
nros_box_publish                       # defined by ros2-box-env.sh
nros setup --system                    # system packages — see below
nros setup native --rmw zenoh          # the box needs its OWN SDK store
nros setup native --rmw cyclonedds     # for the DDS interop cells
just doctor                            # confirm before running any tier
```

**Do not skip `nros setup --system`, and run `just doctor` before believing
anything about a slow box.** The distrobox setup script installs only what the
CLI needs to COMPILE. The recipes need a further set, declared in `[system.*]`
of `nros-sdk-index.toml` — and most of the justfile probes those with
`command -v` and *degrades* rather than failing, so a box without them builds
fine, passes, and merely runs wrong.

The measured case: no `parallel` in the box, so `check-examples` printed
`GNU parallel not found — falling back to serial check` and walked all 99
example leaves one at a time. On a 4-core box that reads as a hung tier, not as
a missing package. `just doctor` had been reporting
`[MISSING] gnu-parallel — just format fan-out (issue 0368 F7)` with the exact
apt line the whole time; nothing in the box bootstrap said to run it.

Then the normal tiers, all through the same entry form:

```sh
distrobox enter ros2 -- bash -c '. scripts/dev/ros2-box-env.sh && just build-test-fixtures'
distrobox enter ros2 -- bash -c '. scripts/dev/ros2-box-env.sh && just test-unit'
distrobox enter ros2 -- bash -c '. scripts/dev/ros2-box-env.sh && just test-all'
```

The test harness resolves ROS's `rmw_zenohd` — `nros_zenohd_bin` reads
`NROS_RMW_ZENOHD`, then `AMENT_PREFIX_PATH`, then `$ROS_DISTRO` under `/opt/ros`.
So SOURCE the distrobox's ROS before running the zenoh lanes; nano-ros ships no
router and the SDK store holds none (RFC-0075 / phase-362).

Reading the result: `just test-unit` runs bare `cargo nextest`, which counts a
`nros_tests::skip!` panic as a FAILURE — only `test-all`'s junit rewrite turns
those back into skips. A "failure" whose output starts with `[SKIPPED]` is a
skipped precondition, not a defect.

## Why the overrides exist — glibc direction

glibc is **backward** compatible: a binary linked against the box's older glibc
runs on a newer host, never the reverse. So box-built artifacts work on both
sides, and host-built ones are unusable in the box. That single fact explains
every override:

| Override | Without it |
| --- | --- |
| `CARGO_TARGET_DIR` | cargo re-runs the cached build-script **executables**; a host-built `build-script-build` dies with `GLIBC_2.xx not found`. Not churn — a hard failure. |
| `NROS_HOME` | a shared store reports the host's zenohd "present" at the pinned version, then hands the box a binary it cannot exec |
| `CARGO_INSTALL_ROOT` | `~/.cargo/bin/just` is host-built and fails the same way — and a box-built copy written back to that path would break the host in turn |

Shared safely: `~/.rustup` (toolchains target an old glibc) and the cargo
registry/git caches (sources, not objects).

`CARGO_TARGET_DIR` hides the CLI from `activate.sh`'s PATH entry and from
cmake's `find_program` HINTS, both of which look at
`packages/cli/target/release/nros` — hence `nros_box_publish`, which copies the
box build there. **A host-side CLI rebuild overwrites it and breaks the box
until you re-publish.**

## Two paths to the same checkout

distrobox mounts the whole host filesystem at `/run/host` **and** translates the
entry cwd into it, so the checkout is reachable inside the box as both
`/run/host/mnt/…` and the bind-mounted `/mnt/…`, depending on how you entered.
Both are the same tree; only one matches the host's absolute path. A box build
under one and a host build under the other silently disagree — `nros sync`
writes absolute paths and the cargo/cmake caches key on them (the issue-0375
hazard). `ros2-box-env.sh` strips the `/run/host` prefix when the stripped path
is the same checkout, so `NROS_REPO_DIR` matches the host either way.

## Known rough edges

- **`zstd` is not in a stock Ubuntu 22.04**, and the prebuilt dists are
  `.tar.zst`, so the first prebuilt install fails with `tar (child): zstd:
  Cannot exec`. `sudo apt-get install zstd` inside the box. → issue 0385
- **`rmw_zenoh_cpp` is an apt package on humble+** (`ros-<distro>-rmw-zenoh-cpp`;
  this note used to say humble had none). Install it like the others — there is
  no source overlay to build any more (RFC-0075, amended 2026-08-19).
- **If the box is in play, EVERY job runs in the box — against the box's OWN
  tree.** Not a style preference: the two sides have different compilers and a
  different libc, and nothing in the build system checks that the toolchains
  agree. Sharing one checkout means sharing artifacts built by both. The mode is
  refused outright since issue 0759 (`ros2-box-env.sh` returns 1 against a tree
  with no `.nros-box-tree` marker); `NROS_ALLOW_SHARED_BOX_TREE=1` is the
  deliberate exception. Make the tree with `scripts/dev/ros2-box-sync.sh` and
  `cd` into it. Do not half-share — a host-side `just setup-cli` or a host-side
  fixture build reaches into the same directories.
- **A box created BEFORE that note has no router, and nothing says so loudly.**
  The setup script installs `rmw_zenoh_cpp` today; a box built earlier does not
  gain it, and the only symptom is cells reporting `[SKIPPED] zenohd failed to
  start … no rmw_zenoh_cpp/rmw_zenohd found` — a skip, so the run stays green
  (2026-08-22: every router-needing sched cell skipped here, and a measurement
  script that keys on its cell's own output line read those skips as FAILURES,
  which is the opposite misread and the reason it was noticed at all). Check
  with `just doctor`, fix with `sudo apt-get install ros-<distro>-rmw-zenoh-cpp`
  inside the box.
- **The vendored `make` and `nros-launch-resolve` are host binaries in a shared
  tree**, exactly like the CLI. The pinned make used to link the build host's
  guile and died here as `libguile-3.0.so.1: cannot open shared object file`
  (fixed: the index's source recipe configures `--without-guile`, so one binary
  serves both sides — rebuild it once in the box with
  `nros setup --tool make`).
  `nros-launch-resolve` is per-environment by design (issue 0409): a stale one
  fails `nros sync` with a TOML `unknown field` error naming a field the current
  schema does have, which reads as a broken `system.toml` rather than a stale
  binary. `just setup-launch-resolve` in the box.
- **ROS's `setup.bash` dies under `set -u`** (`AMENT_TRACE_SETUP_FILES: unbound
  variable`). Any script sourcing it needs `set +u` around the source.
- **The stale-CLI guard fires after any pull that touched CLI sources**
  (`in-tree nros CLI is STALE — its sources changed since it was built`).
  Rebuild it in the box and re-run `nros_box_publish`.
- **`source` in a pipeline runs in a subshell** — `source env.sh | tail` leaves
  your environment untouched and looks exactly like a broken activation.

## Verified on this route

`just test-unit` in the box: **817 tests, 816 passed, 2 skipped**, plus one
`[SKIPPED] second session refused — shim built with ZPICO_MAX_SESSIONS=1`
reported as a failure by the bare-nextest quirk above. No box-specific failures.

Arch host (glibc 2.44) + Ubuntu 22.04.5 box (glibc 2.35), 2026-08-01:
`nros setup` for zenoh and cyclonedds, `nros sync`, a cyclone-backed
`examples/native/rust/talker` build, and `ros2 topic echo /chatter` receiving
its messages over `rmw_cyclonedds_cpp` with `ros2 topic list` showing
`/chatter`.
