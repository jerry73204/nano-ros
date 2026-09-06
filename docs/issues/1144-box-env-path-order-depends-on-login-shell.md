---
id: 1144
title: "`ros2-box-env.sh`'s PATH prepend is undone by `activate.sh` in a NON-login shell — the host's glibc-2.39 `just` wins and dies"
status: open
type: bug
area: build, testing
severity: medium
found: 2026-09-06
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

## Fix

Make `ros2-box-env.sh` put its dir in front **after** sourcing `activate.sh`, or
have it prepend both times. The comment's intent (box tools beat host tools) is
right; only the ordering assumption is wrong, and it is wrong silently.

A cheap assertion is worth adding wherever the box's tools are required:

```sh
command -v just | grep -q local-box || { echo "host just won the PATH race"; exit 1; }
```

That line is what turned this from a mystery into a measurement.

## Not a fix

Sourcing `activate.sh` first and `ros2-box-env.sh` second works, and is what
phase-433 W1 used to get moving, but it is a workaround: `ros2-box-env.sh`
sources `activate.sh` itself, so the first source only exists to make
`activate.sh`'s guard a no-op. Documenting the workaround instead of fixing the
order would leave the trap armed for the next caller.
