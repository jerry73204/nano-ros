---
id: 1078
title: "The five XRCE pool knobs that size ~86 % of `xrce_session_state_t` appear nowhere in the static pool inventory"
status: resolved
type: bug
area: rmw, docs
severity: low
found: 2026-09-05
related: [0271, 0739, phase-420]
---

# A knob nobody can enumerate is a knob nobody sets

`book/src/reference/static-pool-inventory.md` is the answer to "what static
memory does an image budget, and which knob moves it" — issue 0271's rule, which
`gen-pool-inventory.py` exists to keep true. Five XRCE knobs are not in it:

- `NROS_XRCE_MAX_SUBSCRIBERS`
- `NROS_XRCE_MAX_SERVICE_SERVERS`
- `NROS_XRCE_MAX_SERVICE_CLIENTS`
- `NROS_XRCE_SUBSCRIBER_RING_DEPTH`
- `NROS_XRCE_BUFFER_SIZE`

Between them they size roughly 86 % of `xrce_session_state_t`. Absent before
phase-420 W9 and absent after it — this is not a regression W9 introduced, and
W9 is only where it was noticed.

Verify: `grep NROS_XRCE_MAX_SUBSCRIBERS book/src/reference/static-pool-inventory.md`
returns nothing, on this commit and on any earlier one.

## Why they are missing

`gen-pool-inventory.py` finds knobs by reading the *sources* that name them. The
XRCE knobs are `-D` defines the build script passes to the C compiler, not Rust
constants with a nameable declaration site, so the generator's scan never sees
them.

## Why it is cheap now, and was not before

phase-420 W9 moved the whole XRCE configuration surface into
`packages/rmw/xrce/xrce-config.txt`, whose `define` records enumerate exactly
these five with their knob names and minimums:

```
define  XRCE_MAX_SUBSCRIBERS       NROS_XRCE_MAX_SUBSCRIBERS       1
define  XRCE_MAX_SERVICE_SERVERS   NROS_XRCE_MAX_SERVICE_SERVERS   1
define  XRCE_MAX_SERVICE_CLIENTS   NROS_XRCE_MAX_SERVICE_CLIENTS   1
define  XRCE_SUBSCRIBER_RING_DEPTH NROS_XRCE_SUBSCRIBER_RING_DEPTH 1
define  XRCE_BUFFER_SIZE           NROS_XRCE_BUFFER_SIZE           64
```

So the fix is for `gen-pool-inventory.py` to read that manifest as a source of
knobs, rather than to hand-add five rows — which would be five more
restatements, the shape phase-420 W9 spent itself removing.

## Not just a docs gap

Issue 0739 measured ~145 KB in one image lost to a pool nobody could enumerate.
These five are the same class: an integrator sizing an XRCE image has no
document that names them.

## Resolution — `gen-pool-inventory.py` reads the manifest (2026-09-05)

`scripts/gen-pool-inventory.py` now seeds its knob table from
`packages/rmw/xrce/xrce-config.txt` before it scans any Rust, using that
manifest's OWN parser — imported from `scripts/check-xrce-config-manifest.py`,
the gate over the file, rather than written a second time. No row is restated
here; both the env names and the defaults are read out of the tree.

The default comes from whichever file actually states it, which is not the same
file for the two record types:

* a `knob` row fills a `@TOKEN@` in an upstream `config.h.in`, which holds no
  default of its own, so the row's `<default>` column is the figure;
* a `define` row deliberately has NO default column — `xrce-config.txt` says in
  as many words that `nros-rmw-xrce/src/internal.h` holds it in an `#ifndef`
  and "that is the one statement of it" — so the generator pairs the row's
  MACRO against that `#ifndef` block. A macro with no guard is not overridable,
  so it yields no figure rather than an invented one.

Eight rows now appear where two did:

```
| `NROS_XRCE_BUFFER_SIZE`           | 1024 | `packages/rmw/xrce` |
| `NROS_XRCE_CUSTOM_TRANSPORT_MTU`  | 4096 | `packages/rmw/xrce` |
| `NROS_XRCE_MAX_SERVICE_CLIENTS`   |    4 | `packages/rmw/xrce` |
| `NROS_XRCE_MAX_SERVICE_SERVERS`   |    4 | `packages/rmw/xrce` |
| `NROS_XRCE_MAX_SUBSCRIBERS`       |    8 | `packages/rmw/xrce` |
| `NROS_XRCE_STREAM_HISTORY`        |   16 | `packages/rmw/xrce` |
| `NROS_XRCE_SUBSCRIBER_RING_DEPTH` |   32 | `packages/rmw/xrce` |
| `NROS_XRCE_TRANSPORT_MTU`         | 4096 | `packages/rmw/xrce` |
```

### Three things this issue's text got wrong, found while fixing it

1. **The minimums it quotes are stale.** It lists the three entity caps at
   minimum `1`; the manifest now says `0` (issue 1033 — a cap of 0 on an image
   that creates none of that entity is the honest answer, and it is what takes
   `xrce_session_state_t` from 76,624 to 54,928 bytes on the zephyr cpp
   listener).

2. **A minimum is not a default, and publishing it would have been worse than
   the gap.** The `define` rows' third column is a floor. Reading it would have
   printed `NROS_XRCE_BUFFER_SIZE = 64` for a 1024-byte buffer — a wrong figure
   in the one table that exists to be trusted, which is why the generator pairs
   the macro against `internal.h` instead. It is the first mutation in the
   sweep below.

3. **It undercounts by one, and the missing one was invisible everywhere.**
   The `knob` rows belong in this table too — they size
   `XRCE_STREAM_BUFFER_SIZE = MTU x STREAM_HISTORY`, twice per session, which
   is the 131,072 bytes issue 0968 measured. Including them surfaced
   `NROS_XRCE_TRANSPORT_MTU`, which no `.rs` file names anywhere in the tree
   (`git grep NROS_XRCE_TRANSPORT_MTU -- '*.rs'` is empty), so it was absent
   from this page for the same reason the five `define`s were and nobody had
   noticed. Its two rows (UDP and TCP) agree, so they collapse to one; rows
   that DISAGREED would carry the table's existing `conflicting defaults` flag.

`NROS_XRCE_CUSTOM_TRANSPORT_MTU` and `NROS_XRCE_STREAM_HISTORY` were the two
knobs already present, reached through `platform_config.rs`'s ladder
`xrce_env_key` match — which can state no figure, so both rendered as
`computed - see platform_config.rs:1029`. The manifest owns them now: it is
where the number is, and the seeding order is what decides that, so the
self-test pins it.

Bytes are still not computed for these. A `// nros-pool:` annotation for
`xrce_session_state_t` would need a line in `internal.h` or the manifest, both
outside this change; the knobs are enumerable now, which is what 0271 asked for.

### The self-test asserts a RELATION, not five numbers

The first version of the integration control listed the five defaults —
`("NROS_XRCE_BUFFER_SIZE", 1024)` and four siblings — and was wrong in exactly
the way this issue is about. Five figures in a THIRD file means the generator
disagrees with `internal.h` by construction the moment someone legitimately
retunes one, and the failure tells them to edit the generator: a second home,
made permanent by its own test. It caught a deliberate 1024 to 2048 retune and
called it a regression.

It now asserts the property instead: **every `define` row's published default IS
the `#ifndef` guard for the macro that row binds; every `knob` row's IS its own
column** — plus the manifest's own documented invariant, that a `define` row's
macro HAS such a guard (the manifest states no default for a `define` precisely
because `internal.h` does, so neither file stating one is the real defect). A
retune moves both sides together and the page follows; a broken wire moves one
side only. Verified both directions: with `internal.h` at 2048 and nothing else
edited, the generator writes, `--check` is clean and the page reads
`| NROS_XRCE_BUFFER_SIZE | 2048 |`; with the tree retuned AND the macro/guard
pairing broken, it still fails.

Mutations swept, all caught, none breaking record structure: the `define`
default read from the manifest's MIN column; the default paired by ENV name
instead of the macro the row binds (the crossed wire — shape perfectly valid);
duplicate rows on one env overwriting instead of collapsing or flagging; an
unguarded bare `#define` counted as a knob default; the manifest scan disabled;
the manifest seeded AFTER the Rust scans with the same rows (the ladder's
computed default wins and the relation breaks); `value` rows treated as env
knobs.

### One stale sentence corrected next door

`xrce-config.txt`'s `UCLIENT_PLATFORM_ZEPHYR` comment said
"`transport_zephyr_udp.c` is compiled by nobody (issue 1073)". Issue 1073
DELETED that file the same day, so the sentence named something that no longer
exists. The `flag … never` row it annotates is unchanged and still correct — it
selects upstream's Zephyr PLATFORM, not our TU — and the comment now says that.

Gates: `python3 scripts/gen-pool-inventory.py` + `--check` clean,
`just check pool-inventory`, `just check xrce-config-manifest`,
`just check gate-selftests`, and `python3 scripts/check-xrce-source-manifest.py`
(it reads both XRCE manifests, so the comment edit is in its blast radius).
