---
id: 1144
title: "`ros2-box-env.sh`'s PATH prepend is undone by `activate.sh` in a NON-login shell — the host's glibc-2.39 `just` wins and dies"
status: resolved
type: bug
area: build, testing
severity: medium
found: 2026-09-06
resolved_in: "phase-433 W1 follow-up — scripts/dev/ros2-box-env.sh"
related: [0400, 0401, 0759]
---

# Whether the box's tools are on PATH depends on `bash -c` vs `bash -lc`

`scripts/dev/ros2-box-env.sh:178` prepends the box's own tool dir and says why:

```sh
# BEFORE activate.sh: it sources scripts/sdk-env.sh, which shells out to `just`,
# and the host's ~/.cargo/bin/just would otherwise win the PATH race and print
# `GLIBC_2.xx not found`. activate.sh prepends packages/cli/target/release after
# this, which is correct — nros_box_publish put a box binary there.
export PATH="$CARGO_INSTALL_ROOT/bin:$PATH"

. "$_nros_box_root/activate.sh"
```

The reasoning holds only when `$HOME/.cargo/bin` is **already** on PATH, because
`activate.sh:90` prepends it conditionally:

```sh
if [ -d "$HOME/.cargo/bin" ]; then
    case ":$PATH:" in
        *":$HOME/.cargo/bin:"*) ;;
        *) export PATH="$HOME/.cargo/bin:$PATH" ;;
    esac
fi
```

In a **login** shell the container's profile has already put `~/.cargo/bin` on
PATH, the guard skips, and `~/.local-box/bin` stays in front. In a **non-login**
shell it has not, so `activate.sh` prepends the host's cargo bin **in front of**
the dir `ros2-box-env.sh` just prepended — and the host's `just`,
`cargo-nextest` and friends are glibc-2.39 binaries that cannot run in a
Jammy container.

## Measured

Non-login (`podman exec -u aeon ros2 bash -c`), `ros2-box-env.sh` alone:

```
PRE  PATH-head=/usr/local/sbin:/usr/local/bin:/usr/sbin
POST CARGO_INSTALL_ROOT=/home/aeon/.local-box
POST PATH-head=/home/aeon/.nros-box/sdk/espflash/…:/home/aeon/.nros-box/sdk/…
$ command -v just
/home/aeon/.cargo/bin/just          # ← the HOST binary
```

with `/home/aeon/.local-box/bin/just` present and executable. The failure it
produces names neither PATH nor the box:

```
just: /lib/x86_64-linux-gnu/libc.so.6: version `GLIBC_2.39' not found (required by just)
```

`docs/development/ros2-on-non-ubuntu.md` already records that this class of
error "surfaces as `just: … not found` or `no such command: nextest` rather than
as anything mentioning glibc" — this is the same class, arriving through PATH
order rather than through a missing install.

## Why it has not bitten before

The documented entry point is `distrobox enter ros2 -- bash -c '...'`, and
distrobox's wrapper gives a login-ish environment where `~/.cargo/bin` is
already present. Driving the container directly with `podman exec` — which is
faster and what an agent reaches for — does not.

## Fixed — establish the precondition, then assert the result

Neither of the two obvious repairs, and the reason matters.

**Not "prepend the box dir again after `activate.sh`".** `activate.sh` also
prepends `packages/cli/target/release` — **unconditionally**, and deliberately
(its own comment: each worktree's CLI wins, whether or not it is built yet). A
box dir in front of that inverts it: `nros_box_publish` writes the CLI to both
places, but a later in-box `just setup-cli` rewrites only
`packages/cli/target/release/nros`, so the install-root copy would then shadow a
fresh CLI with a stale one. Fixing a shadowing bug by arming a second one is not
a fix.

**What landed instead**: `ros2-box-env.sh` seeds `$HOME/.cargo/bin` at the
**TAIL** of PATH before sourcing `activate.sh`. That makes `activate.sh`'s
conditional prepend a no-op *by construction* in both shell kinds — the guard
was always written against the precondition "already on PATH", and we now
establish it rather than assume it — while every prepend `activate.sh` makes
keeps its designed relative order. `cargo`/`rustup` stay reachable: rustup's own
shims are old-glibc and do run in the box; it is the cargo-INSTALLED tools
beside them that do not.

**And the assertion, keyed on the HOST dir rather than on `local-box`**:
`nros_box_check_path` (defined and run at the end of `ros2-box-env.sh`) walks
every executable the box installed into `$CARGO_INSTALL_ROOT/bin` and fails if
any of them resolves into `$HOME/.cargo/bin`, naming the shadowed pair. Keying
on the host dir rather than on the box dir is what keeps `nros` — which
legitimately resolves to `packages/cli/target/release/nros` — from being a false
positive, and it covers `cargo-nextest`/`bindgen` for free rather than just
`just`. A sourced refusal only stops a caller that joins with `&&`, so the
documented entry form is now `. scripts/dev/ros2-box-env.sh && <cmd>` (the
issue-0759 refusal had the same weakness).

## Verification

Not the box: another process held the box tree while this was fixed, so it is
simulated. `tmp/1144-path-sim.sh` (throwaway, not committed) runs the REAL
`ros2-box-env.sh` — HEAD's copy and the fixed one — inside a mini-checkout whose
`activate.sh` carries the two prepends verbatim, with a fake `$HOME` holding a
`.cargo/bin/just` and a `.local-box/bin/just`:

```
before login     exit=0  just -> …/.local-box/bin/just  (BOX-just)
before non-login exit=0  just -> …/.cargo/bin/just      (HOST-just)   ← the bug
after  login     exit=0  just -> …/.local-box/bin/just  (BOX-just)
after  non-login exit=0  just -> …/.local-box/bin/just  (BOX-just)
```

Negative control — swap in an `activate.sh` that prepends `$HOME/.cargo/bin`
UNCONDITIONALLY (the regression the assertion exists for): `before` reports the
host binary and exits 0; `after` refuses with exit 1 and names both paths.

Unverified without the box: that the box's real `just`/`cargo-nextest`/`nros`
then run to completion under a `podman exec … bash -c` job. The PATH ordering is
what this issue is about and that part is measured; the tools were already known
present (phase-433 W1 census).

## Not a fix

Sourcing `activate.sh` first and `ros2-box-env.sh` second works, and is what
phase-433 W1 used to get moving, but it is a workaround: `ros2-box-env.sh`
sources `activate.sh` itself, so the first source only exists to make
`activate.sh`'s guard a no-op. Documenting the workaround instead of fixing the
order would leave the trap armed for the next caller.
