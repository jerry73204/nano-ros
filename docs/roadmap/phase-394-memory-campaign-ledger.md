# Phase 394 — memory unification and optimization: the campaign ledger

**Status (2026-08-27). Ledger open; the W1 instrument has landed.** This doc
coordinates work that already had three phase docs and eight issues but no single
place saying who is doing what. It owns no design of its own — every decision
lives in the phase it belongs to.

## Amendment 2026-08-29 — what a board measurement added

A bring-up session on mr_canhubk3/s32k344 took the action image from 98.73 %
SRAM (non-functional — it could not carry the instrumentation needed to debug
it) to 85.60 % and working. Four items came out of it, three of which no phase
in this campaign covered:

| item | where it landed | status |
| --- | --- | --- |
| Tightly-coupled memory was never a placement target — 192 KiB idle at 0 % | phase-392 amendment A, [issue 0880](../issues/0880-tcm-unused-while-sram-exhausted.md) | stacks moved; 80 KiB of DTCM still free |
| Pools are sized for worst cases that are SUMMED, never overlapped | phase-392 amendment B | new wave, tier-gated |
| The libc arena regresses because only the symbol is gated, not the config | phase-391 W1b | gate item added |
| Flash is 4 MiB at 8.3 % and is NOT fungible with RAM | phase-392 amendment D | scoped: post-mortem log, config storage, ITCM |

## Amendment 2026-08-31 — the arena has no symbol, so the instrument cannot price it

A design review of the subscription buffer, from the ASI consumer side, found
that the campaign's largest single consumer is invisible to the campaign's own
instrument. `Executor` holds its arena inline, so it lands on whichever task
calls `spin` rather than in `.bss` — and W1 prices pools by reading symbols.

| item | where it landed | status |
| --- | --- | --- |
| The executor arena is stack-resident, so `mem-report` cannot see it and task stacks carry a term proportional to the subscription graph | [RFC-0002 § 4.4b](../design/0002-rt-execution-model.md) decision; phase-392 lever 1b + wave W6 | opened |

Two things it does NOT change, recorded so they are not re-litigated from this
amendment: it does not decide whether payload buffers become heap-backed (that
is amendment B's wave, and a `.bss` arena only makes it measurable), and it does
not size subscriptions to their type (that is phase-392 W3a). The three
compound; none depends on another.

One correction is recorded rather than a gap: *field storage mode does not
shrink wire buffers.* Phase-392 lever 2 already said so; it has since been
proposed twice in the opposite direction, so the amendment restates it as a
decision record.

## What the campaign is

Four phase docs, opened 2026-08-26 from one allocation review, that are really
one effort in two halves:

| phase | half | one line |
| --- | --- | --- |
| [390](phase-390-storage-mode-rename-inline-heap-view.md) | vocabulary | rename RFC-0033 storage modes `owned`/`borrowed` to `inline`/`heap`/`view` |
| [391](phase-391-allocation-unification-and-tier-model.md) | dynamic | one `#[global_allocator]`, one funnel, a tier model, rlsf |
| [392](phase-392-static-memory-space-campaign.md) | static | 27% of a safety-island image is message buffers nobody can price |
| [380](archived/phase-380-serialized-size-bound.md) | static | serialized size bound (W0–W3, W5 landed) |

The dependency order the docs state: 392 depends on 390 for vocabulary and on
391 for the gate that verifies its claims, and 392 W1 is the instrument
everything else is measured with — *"Do this first."*

## The instrument (392 W1) — landed

`scripts/nros-mem-report.py`, `just mem-report <elf>`. Reads a built image's
symbol table and reports RAM by symbol, by crate, and by declared pool, with the
unattributed gap called out. `--json` plus `--baseline` gives the before/after
delta a saving should be reported as.

**W1 as written is not achievable and the plan should be amended.** It says
"annotate the rest; add a gate rejecting new unannotated pools". The four
unpriced pools cannot be annotated, and they all fail for one reason:

| pool | why no static formula |
| --- | --- |
| `SERVICE_BUFFERS` | `ZPICO_MAX_QUERYABLES` has a **computed** default — there is no integer to write |
| `MESSAGE_INFO_TABLE` | element gains three fields under `alloc` + `safety-e2e`; issue 0739 declined to annotate it, correctly |
| `SUBSCRIBER_BUFFERS` | array of structs — element size is a `sizeof`, not a knob product |
| `__nros_comp_buf_N` | codegen emits `sizeof(component class)` |

The size is known to the **compiler**, not to a comment, and a hand-written
figure in a comment is the drift class this tree already gates against
(`check-ffi-struct-mirrors`). So the instrument measures rather than declares,
and the two mechanisms compose: `--check` joins each declared formula to its
measured symbol and requires agreement on a default-built image, which turns the
inventory's published numbers from a claim into a checked fact. Gate:
`check-mem-report` (selftest, source-only) plus the fixture-backed test
`static_memory_declared_pools`.

## What the instrument found immediately

[Issue 0827](../issues/archived/0827-unused-rmw-pools-dominate-static-ram.md). Static RAM
is a property of the RMW, not of the node — **identical to the byte** across
talker, listener, service-server and action-server:

| RMW | RAM in symbols |
| --- | ---: |
| zenoh | 345,379 |
| cyclonedds | 69,381 |
| xrce | 10,340 |

A talker reserves 144,128 bytes of `SERVICE_BUFFERS` and 131,072 bytes of
`LARGE_PAYLOADS` it cannot reach: 80% of its static RAM. The pools are sized at
the backend, where the entity set is unknown, instead of at the image, where it
is known.

## Ledger

Claim a row before starting; a row names the phase or issue that owns the
decision, never this doc.

| item | owns | state | notes |
| --- | --- | --- | --- |
| 392 W1 instrument | this session | **landed** | `just mem-report`, `check-mem-report`, `static_memory_declared_pools` |
| 392 W1 amendment | this session | **landed** | annotate-the-rest is not achievable; measure instead (above) |
| issue 0827 | unclaimed | filed | pools sized at the backend, not the image |
| issue 0810 | unclaimed | open | executor arena at `MAX_CBS * sizeof(ActionClient)` |
| issue 0811 | fan-out agent | **landed, unbuilt** | five ports; a real cross-allocator use-after-free, not a latent mismatch |
| issue 0812 | fan-out agent | **landed, compiles** | TWO mallocs, not one; removed with zero new state. `nros-rmw-cffi`, `nros-rmw-zenoh` and `nros-c` all check clean with `lending` — the last of those is compiled by no lane |
| issue 0813 | fan-out agent | **landed** | now the `ZPICO_PUBLISHER_TX_BUFFER_SIZE` knob; pool row deliberately NOT added (see below) |
| issue 0814 | unclaimed | open | zero-copy surface behind a feature only a posix test crate enables |
| issue 0815 | this session | **superseded in part** | 3-of-46 pricing — see the W1 amendment |
| issue 0816 | fan-out agent | **gate landed, claims still unbacked** | the missing thing is a FIXTURE, not a gate — see below |
| issue 0817 | — | resolved | sixteen Zephyr funnel bypasses (391 prerequisite) |
| 390 W1 rename | unclaimed | open | `owned`/`borrowed` to `inline`/`heap`/`view` |
| 391 waves | unclaimed | open | one funnel, tier model, rlsf |
| 392 W2 arena | unclaimed | open | `NROS_ARENA_REQUIRED` linker symbol; hand-written `main`s explored, not assumed away |
| 392 W3 wire sizing | unclaimed | blocked on W1 | now unblocked — the saving can be measured |
| 392 W4 net stack | unclaimed | open | 27,760 B of net pools in a serial-only image; needs triage first |

## A pool row is a claim about the reader's image, not about the source

Issue 0813's fix first annotated the publisher loan arena, and the inventory
priced it at 8,192 bytes. The row was dropped before landing: the arena is behind
`feature = "lending"`, which [issue
0814](../issues/0814-lending-never-exercised-on-hardware.md) measured as enabled
by one posix test crate and no shipped image — `nm` on a built zenoh example
finds zero of them. The inventory is the page people use to rightsize a board, so
a row there asserts *your image contains this*. The knob is enumerated either
way, which is what 0813 asked for.

Rule for the rest of the campaign: **annotate a pool only if a built image
contains the symbol, and confirm with `just mem-report` before adding the row.**
The generator does no `cfg` analysis and cannot catch this class on its own.

## Three findings the fan-out produced that outrank their issues

**No image in this tree is built no-alloc, so the book's promise cannot be
checked yet.** All 13 bare-metal Rust example leaves enable `alloc`, RTIC
included, and there is no Embassy example at all. Issue 0816 framed this as a
missing gate; the gate now exists (`scripts/check-no-alloc-image.py`, selftest
wired as `check-no-alloc-image`) and reports **0 of 4 book claims backed** — not
because it is weak but because there is nothing to point it at. The remaining
half of 0816 is a `Cargo.toml` plus a `fixtures.toml` row, not more tooling.

**The platform allocation funnel is absent from two of the three native
backends.** `nros_platform_alloc` is defined in the zenoh talker and undefined in
the cyclonedds and xrce ones. Phase 391's whole premise is one funnel; on a
hosted image, two thirds of the backends do not reach it. Whether that is
intended for hosted builds or is a real gap is a phase 391 question, and it
should be answered before the tier model is built on top of the assumption.

**A public C symbol can be declared by the header and compiled by no lane.**
`nros_publisher_loan` and its commit/discard siblings are emitted into the
committed `nros_generated.h` unconditionally, while nothing in the tree enables
`nros-c/lending` or `nros-cpp/lending` — not a test crate, not cmake, not an
example. So the public header declares symbols no build produces, and no lane
would have caught a change to them. That is the `check-required-features-reachable`
class one level out: the manifest gate asks whether a target with
`required-features` is reachable, and says nothing about a cbindgen-exported
surface behind a feature nobody enables.

## Measure the fixtures, not the example leaves

`examples/**/target-*/` is the pre-phase-340 layout. P2 moved fixture builds into
the shared cargo group under `build/cargo-fixtures/<host-hash>/<profile>/`, and
the per-leaf directories are leftovers nothing rewrites — the ones here were
three weeks stale while `just build-test-fixtures` reported success, because it
no longer writes there at all. Issue 0827's first draft was measured on them.
The pool figures were unchanged so the conclusion held, but the totals were
wrong by ~2.5 KB and a "to the byte" claim was wrong by 16.

The staleness probe cannot help here: it guards fixtures the harness RESOLVES,
and a path typed by hand into a tool is not one of those. So the rule is manual
and belongs with the tool: **check the artifact's mtime against its sources
before quoting a number from it.**

## A green platform build is not coverage of that platform's files

Issue 0811 changed `net.c` on five ports. Tier 1 and tier 2 both went green
having compiled exactly ONE of them --- every `net.c.o` in the tree was
`nros_platform_posix_build`. Worse, two of the per-platform builds go green
while deliberately skipping the file:

| build | what it does with `net.c` |
| --- | --- |
| `just threadx_linux build-c-port` | **excludes it** --- "no NetX Duo", says so in the recipe |
| `just freertos build-c-port` | **excludes it** --- "no lwIP in this harness" |
| `just threadx_riscv64 build` | never compiles it; delegates to `build-fixture-extras` |
| zephyr, any config | only under `if(CONFIG_NET_SOCKETS)` --- a serial image never compiles it |

So "ThreadX built clean" and "the file I edited compiles" are independent
statements, and the first is the one CI reports. What actually verified the
five ports, on 2026-08-27:

| port | how |
| --- | ---: |
| posix | tier 1 + tier 2 |
| zephyr | `just zephyr build-c` --- 6 configs, exit 0 |
| freertos | `just freertos build-examples` --- `net.c.obj` built |
| threadx | direct `cc -fsyntax-only` under the file's own `-Wall -Wextra -Wpedantic -Werror`, against the vendored NetX Duo + ThreadX headers |
| esp-idf | `just esp_idf build` --- exit 0 |

Before claiming a platform change is verified, grep the build log for the FILE,
not for the platform's name.

## Working rule for this campaign

Every saving is reported as a measured delta between two `just mem-report --json`
runs, naming the image. Phase 392 opened with a hand-pasted `nm` table; the point
of W1 is that no later wave has to do that again, and no later wave gets to claim
a saving it did not measure.
