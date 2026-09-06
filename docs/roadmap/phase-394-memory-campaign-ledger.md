# Phase 394 — memory unification and optimization: the campaign ledger

**Status (2026-09-06). Ledger open. 390 and 391 are done through W5; 392 has
W1, W2, W3 (except W3c), W4 and W5 landed, with W6 in flight. Of the eighteen issues on
the ledger, nine are resolved and nine open — two of those claimed today, plus
the W6 wave.** This doc
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
| 392 W1 instrument | — | **landed** | `just mem-report`, `check-mem-report`, `static_memory_declared_pools` |
| 392 W1 amendment | — | **landed** | annotate-the-rest is not achievable; measure instead (above) |
| 392 W2 arena | — | **landed as issue 0900** | W2 IS 0900, and 0900 is resolved + archived. The ledger carried it as "unclaimed / open" for nine days |
| 392 W3a / W3b | — | **landed** | subscription buffer sized from the type's own bound — Rust only |
| 392 W3c | unclaimed | **open** | derived-by-default on the Rust path; the global becomes the opt-out |
| 392 W3d | — | **landed, under another phase** | 0896 layers 1-3, delivered as phase-408 W1. The C publish helper sizes `buf[<PREFIX>_TX_MAX_SERIALIZED_SIZE]` for a bounded type (`packs/c/message.h.jinja:178`), and `message_size_bound_parity.rs` asserts the C constant, the C++ constant and the Rust const are one number, both encodings |
| 392 W3e | — | **superseded** | by [phase 408](phase-408-cpp-message-derived-buffers.md) — the fork it described does not exist |
| 392 W3f | — | **delivered as a refusal** | Cyclone records that the hint is inapplicable; see the phase's 2026-09-02 amendment, which corrects this row's original claim |
| 392 W3g | — | **landed, under another phase** | 0896 layers 5-6, delivered as phase-403 W0/W6. The split is `_TX_/_RX_MAX_SERIALIZED_SIZE` (TX is XCDR1, RX is `max(XCDR1, XCDR2)`), and the unbounded diagnostic is a POISON TOKEN — `NROS_UNBOUNDED__<type>__field_<name>`, an undefined identifier that names the offending field at the compile error |
| 392 W4 net stack | — | **landed and MEASURED** | 38,334 B of `.bss`+`.data` and 78,696 B of flash on mps2/an385, two images from one tree differing only in networking. Larger than this row's original 27,760 B estimate, because that counted `net_*` symbols and missed four stacks |
| 392 W5.a-W5.g | — | **landed; NO SHIPPED SAVING** | the mechanism, the diagnostic, the checked override and the measurement all exist. The figure reaches one of six compiled cargo units and carries the infra half only. Do not quote 143,456 B as shipped |
| 392 W6 arena static | **claimed 2026-09-06** | in flight | the executor arena becomes a named static, so the campaign's largest allocation stops being invisible to the campaign's own instrument |
| 390 W1-W5 | — | **landed** | the `inline`/`heap`/`view` rename shipped; this row said "open" for nine days |
| 390 W6 | unclaimed | open | the emitter's OWN vocabulary; found while rebasing W5, never started |
| 391 W1-W5 | — | **landed** | one funnel, the tier model, rlsf, and the tier is gated |
| issue 0810 | unclaimed | open | executor arena at `MAX_CBS * sizeof(ActionClient)`. Overlaps W6 — read that wave first |
| issue 0811 | — | **resolved** | five ports; a real cross-allocator use-after-free |
| issue 0812 | — | **resolved** | TWO mallocs, not one; removed with zero new state |
| issue 0813 | — | **resolved** | now the `ZPICO_PUBLISHER_TX_BUFFER_SIZE` knob; pool row deliberately NOT added (see below) |
| issue 0814 | unclaimed | open | zero-copy surface behind a feature only a posix test crate enables — `nm` on a built zenoh example finds zero |
| issue 0815 | unclaimed | **open, superseded in part** | 3-of-46 pricing — the W1 amendment answers the annotation half, not the rest |
| issue 0816 | unclaimed | **gate landed, claims still unbacked** | the missing thing is a FIXTURE, not a gate |
| issue 0817 | — | resolved | sixteen Zephyr funnel bypasses (391 prerequisite) |
| issue 0827 | — | **resolved** | pools sized at the backend, not the image. `nros sync` now writes the derived budget — 176,720 B off a talker. The ledger carried it as "filed" |
| issue 0857 | — | **resolved, in the merge queue** | `Node::ENTITY_BOUNDS` defaulted to the knob caps and 81 of 99 in-tree classes declared nothing. Measured −50,256 B on one slot store, −14.1 % of image `.bss` |
| issue 0880 | unclaimed | open | 192 KiB of TCM at 0 % while SRAM is exhausted; stacks moved, 80 KiB of DTCM still free |
| issue 0973 | **claimed 2026-09-06** | in flight | no resolved model describes endpoint wiring, so W5's application half never arrives. Answered (wiring is authored, nobody authors it); what remains is writing that down |
| issue 1015 | — | **resolved** | the pool floor was in the wrong LAYER — applied at the shared derivation, where XRCE legitimately wants zero |
| issue 1028 | **claimed 2026-09-06** | in flight | NuttX classified `hosted` because `target_os != "none"`, so an RTOS takes the 32-slot host budget: **106,752 B** measured waste |
| issue 1033 | — | **resolved** | 62 % of the XRCE session struct is slots the image does not have. Also fixed six zephyr C++ leaves that could not CONFIGURE at all |
| issue 1061 | — | **resolved** | a leaf declares what the probe cannot read |
| issue 1125 | unclaimed | open | `ZPICO_MAX_LARGE_SUBSCRIBERS` is never derived; `LARGE_PAYLOADS` is the biggest symbol left in the esp32 image |
| issue 1131 | unclaimed | open | fifteen knob-sized C arrays with no ruling on whether zero is a legal size |

## Amendment 2026-09-06 — the ledger had drifted, and drift is the failure this doc exists to prevent

This doc was opened because work spread across three phases and eight issues had
no single place saying who was doing what. Nine days later its own table was the
stalest record of the campaign in the tree: it listed 0827 as "filed" after it
was resolved and archived, 392 W2 as "unclaimed / open" when W2 IS issue 0900
and 0900 had closed, 390 and 391 as open when both had landed W1-W5, and W4 as
"needs triage first" after the triage was answered and the saving measured at
38,334 B.

That is the same shape as the defects this campaign keeps finding — a record
that reads as authoritative while describing a state that no longer exists. The
phase-392 doc had it too: its header said W5.g "has not been run" for six days
after two dated amendments in the same file recorded running it.

Neither was caught by a gate, because neither is checkable: no tool can know
that a table row is out of date. What makes it self-correcting is the ledger's
own rule, applied on the way OUT as well as in — **a row is claimed before
starting and settled when it lands**, in the same change that lands it.

Three rows were claimed today (392 W6, 0973, 1028), each by a parallel agent
working in its own tree.

### The one gap the table admitted, then closed the same day

W3d and W3g were first written into this table as **unverified**: they are issue
0896's layers 1-3 and 5-6, 0896 is resolved and archived, and phase-392 carries
no status marker for either wave. Rather than guess a status, the rows said so.

Ten minutes of reading settled it, and the answer explains the gap. **Both
landed — under different phases.** W3d is phase-408 W1 and W3g is phase-403
W0/W6, so the work was done, recorded, and tested where it happened; only the
phase-392 waves that ASKED for it were never marked. A wave that migrates leaves
its origin doc looking unfinished, and nothing connects the two ends.

The evidence, so the next reader does not repeat the search:

* `packs/c/message.h.jinja:178` — a bounded type's publish helper sizes
  `buf[<PREFIX>_TX_MAX_SERIALIZED_SIZE]`, not the global knob.
* `rosidl-codegen/tests/message_size_bound_parity.rs` — the C header's
  constants, the C++ header's constants and `nros_serdes::size::
  max_serialized_size` are asserted equal for every type in the corpus, both
  encodings. That IS W3d's stated acceptance.
* `generator/common.rs:1964` — the unbounded case emits
  `NROS_UNBOUNDED__<type>__field_<name>`, an undefined identifier, so a type
  with no bound fails to COMPILE and the error names the field. That is W3g's
  diagnostic, and it is the same decision issue 0964 reached from the C++ side.

**One acceptance criterion was wrong as written, and the outcome is better than
it asked for.** W3d said "a C image's publish helpers stop referencing
`NROS_PUB_BUFFER_SIZE`". They still do — in the arm for a type that genuinely
has no bound, with the reason stated in a comment beside it. Removing it there
would mean an unbounded type has no publish helper at all. The knob stopped
being the DEFAULT, which is what the wave was for; the criterion confused that
with removing the symbol.

Recorded because the lesson generalises past this row: a status nobody can
confirm is usually not unknowable, it is filed somewhere the question did not
look.