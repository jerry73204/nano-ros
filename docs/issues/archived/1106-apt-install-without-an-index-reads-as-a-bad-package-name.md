---
id: 1106
title: "`apt-get install` with an empty index reports a bad package NAME, and the names were right"
status: resolved
area: cli, testing
severity: medium
related: [1025, 1070, 1038, 0062]
---

# The third fault in one lane, visible only once the first two were fixed

## What

`native_install_command` composed `sudo apt-get install -y <list>`. A container
image ships with an EMPTY `/var/lib/apt/lists`, and apt against an empty index
does not say "no index" — it says:

```
E: Unable to locate package libgcrypt20-dev
E: Unable to locate package libglib2.0-dev
E: Unable to locate package libpixman-1-dev
E: Unable to locate package openocd
E: Unable to locate package qemu-system-arm
```

Every one of those is a real Debian package. So the failure reads as a bad
package NAME in `nros-sdk-index.toml`, and the names are correct — the
`[prereq.*]` entries map `libglib2-dev` to apt `libglib2.0-dev` exactly as they
should. The evidence points at the declaration, and the declaration is fine.

## Why it took three fixes to see

The nightly `esp32` cell has been red continuously, and a lane that is already
red reports its FIRST failure. Three independent faults were stacked in it, each
invisible until the one in front was fixed:

1. **issue 1025** — the flash packer read a directory the build never wrote, so
   `Build (esp32)` failed and nothing after it ran.
2. **issue 1070** — `--sudo` carried `requires = "system"` in clap, so
   `nros setup --tool esp32-qemu --sudo` exited 2 on a usage error before
   installing anything, skipping every later step.
3. **this** — with provisioning finally reached, it ran `apt-get install`
   against an index that does not exist.

Each fix moved the failure one step earlier in the job and revealed the next.
That is the shape a uniformly-red lane has: no signal capacity, so a new fault
and yesterday's fault look identical (CLAUDE.md, "A red CI lane answers one of
two questions").

## Fix, and why it is a RETRY rather than an unconditional update

```
sudo apt-get install -y <list> || { sudo apt-get update && sudo apt-get install -y <list>; }
```

Prefixing `apt-get update &&` would be simpler and is WRONG here: an update needs
the NETWORK, and a host that is offline with a warm index installs fine today.
Making every install fetch an index would break exactly that host to fix a
different one — the same class as the Corrosion note in CLAUDE.md, where a
resolver that reached for git failed offline while the store already held the
package.

Install-first leaves the warm path untouched: no network, no wait, and the retry
branch is never entered. The update is paid only where the first attempt failed,
which is the cold-container case this exists for.

The cost is that a package that really IS missing prints its error twice. That is
the right trade — the second is the real answer and both name the package.

## Verified in a container, both directions

Not from the mechanism, which is what made the CI log misleading in the first
place:

| container state | command | result |
| --- | --- | --- |
| `rm -rf /var/lib/apt/lists/*` | old | **exit 100**, `Unable to locate package libglib2.0-dev` |
| `rm -rf /var/lib/apt/lists/*` | new | **exit 0**, packages installed |
| warm index | new | exit 0, retry branch **never entered** (0 occurrences) |

The third row is the one that justifies the shape: it is the offline-warm host
the simpler fix would have broken.

Unit-tested too, on the composed STRING (install appears first, the retry
re-installs rather than only refreshing, and `sh -n` accepts it — it is both run
via `sh -c` and printed for a human to paste), plus a guard that the other three
managers are unchanged and `brew` never gains a `sudo`.
