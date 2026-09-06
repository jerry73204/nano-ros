# Phase 392 — 27% of a safety-island image is message buffers nobody can price

**Status (2026-09-07). W1, W2, W3, W4 and W5.a-W5.g landed. W3a/W3b/W3c landed
here, W3d landed as phase-408 W1, W3g as phase-403 W0/W6, W3e is superseded by
phase 408 and W3f was delivered as a recorded refusal. W2 IS issue 0900, which
is RESOLVED. Open: W6, and amendment B's wave.**

**W3c landed with its acceptance NOT met, and the reason is a finding rather
than an omission — read its wave entry before quoting it.** The default now
derives; no image in the tree can show that in `mem-report`, because every
consumer of the derived number is a RUNTIME arena allocation inside a
build-time-sized static.

Opened from a memory-allocation review that measured a real 320 KiB-class board
image.

Landed since: **W1** (pool inventory to full coverage, plus `just mem-report`
and the `static_memory_declared_pools` test that makes a published pool figure
answer to the linker), **W3a/W3b** (a subscription buffer sized from the
message's own `MAX_SERIALIZED_SIZE_*` rather than by hand — Rust only; the
C/C++ half is [issue 0896](../issues/archived/0896-c-cpp-subscriptions-never-state-a-buffer-hint.md)),
and **W5.b1/c/d/e/f** (the queryable table sized by declaration — 143,456 B off
a talker — with an exhausted table that names the declaration and a knob that is
checked at build time).

**W5.g HAS been run** — the "still open" text here said otherwise for six days
after the measurement landed, which is the drift this header now exists to
prevent. Read the two dated amendments below (`W5.g measured 2026-08-30` and
`W5.g DIAGNOSED AND FIXED 2026-09-02`) plus
[issue 0965](../issues/archived/0965-no-entity-inventory-so-three-consumers-cannot-be-derived.md),
which measured it on six configured Zephyr images. What remains open from W5 is
not the measurement but the SAVING: the figure reaches one of six compiled cargo
units, carries the infrastructure half only, and W5.d's 143,456 B is still a
measurement of what the mechanism WOULD save on a leaf where it does not run.
Nothing in W5 may be quoted as a shipped saving. Sizes
below are `nm` output from `build-board/zephyr/zephyr.elf` on
mr_canhubk3/s32k344 (zenoh over serial), not estimates. Depends on
[phase 390](phase-390-storage-mode-rename-inline-heap-view.md) for vocabulary
and [phase 391](phase-391-allocation-unification-and-tier-model.md) for the
gate that verifies the claims.

## Where the RAM goes

| bytes | symbol | kind |
| --- | --- | --- |
| 49,152 | `nros_rmw_zenoh::shim::subscriber::SMALL_PAYLOADS` | wire buffers |
| 32,768 | `nros_thread_stacks` | stacks |
| 30,080 | `__nros_comp_buf_0..3` | deserialised components |
| 19,944 | `g_sessions` | zenoh-pico |
| 17,712 | `SERVICE_BUFFERS` | wire buffers |
| 16,460 | `kheap__system_heap` | the heap |
| 12,288 | `rust_adapter::static_subscriber_storage::SLOTS` | subscriber storage |
| 8,192 | `LARGE_PAYLOADS` | wire buffers |
| 3,584 | `MESSAGE_INFO_TABLE` | |
| 2,640 | `SUBSCRIBER_BUFFERS` | ring metadata |

**Message buffers total 123,648 B — 27% of the 458,752 B of SRAM+DTCM.**

A separate 27,760 B is Ethernet rings, `net_buf` pools and a TCP connection
slab, in an image whose only transport is a serial line.

For scale, one measurement already banked outside this phase: the libc malloc
arena was 24,576 B of `.bss`, `malloc_prepare` ran at boot to initialise it,
and `malloc` itself had been garbage-collected because nothing calls it. Setting
`CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=0` moved `.bss` from 367,566 to 343,010 —
**7.7% of SRAM held by a heap with no allocator**, invisible until someone
listed symbols by size. That is the shape of everything below.

## The levers, in order of leverage

Three shrink what is reserved; **1b shrinks nothing and is listed anyway**,
because it is what makes the other three measurable.

### 1. Wire buffers — 48 bytes of RAM per byte of knob

```
SMALL_PAYLOADS = MAX_SUBSCRIBERS x RING_DEPTH x SUBSCRIPTION_BUFFER_SIZE
               = 12 x 4 x 1024 = 49,152
```

Every byte of `SUBSCRIPTION_BUFFER_SIZE` costs 48 bytes, because the buffer is
uniform across every subscriber regardless of what each one carries.

Codegen already knows each subscription's type, and therefore its maximum
serialised size. **Sizing each subscriber's buffer to its own type** instead of
to a global constant is the largest single win available, and it needs no
allocator — the buffers stay static.

Half the mechanism already exists: `MAX_LARGE_SUBSCRIBERS` /
`SUBSCRIBER_LARGE_SIZE` is a two-class split (1x4x2048 large, 12x4x1024 small).
It is simply **decoupled from codegen**, so a human picks which subscribers are
"large".

### 1b. The arena is invisible to the instrument, so none of lever 1 is visible

> **Corrected 2026-09-06 by W6, which MEASURED it.** This section said
> *"`Executor` holds `arena: [MaybeUninit<u8>; ARENA_SIZE]` inline, so the arena
> lands on whichever task calls `spin`, not in `.bss`"*. **The mechanism was
> wrong; the consequence was right.** `Executor` has held `arena: &'s mut
> [MaybeUninit<u8>]` — a slice borrowed from caller-supplied backing — since
> phase-271 (issue 0110), five phases before this section was written. Placement
> is therefore the CALLER's, and the tree answered it three ways: C statics
> (`.bss`), the C++ `Node::GlobalStorageHolder` (`.bss`), and — on **every Rust
> board** — a `Box::leak` in the `alloc` convenience constructors, which is the
> one that has no symbol. So the arena was invisible because it was on the HEAP,
> not because it was on a stack, and the fix was not "move it off the stack" but
> "give the backing a name". Everything below about *what that costs the
> campaign* stands unchanged; the stack arithmetic does not. The measurement is
> in W6.

The arena — and every `SubInfoEntry::buffer` inside it — was reserved somewhere
the symbol table could not describe.

That defeats this campaign's own instrument. W1 exists to price every pool by
reading symbols; a heap allocation has no symbol, so **the largest single RAM
consumer in a Rust image was the one thing `mem-report` could not see**.
Measured on the native zenoh talker before W6: the largest `nros_node` RAM
symbol in the whole ELF was **1 byte**, and `nros_node` did not appear in the
by-crate table at all.

Moving it to a named static changes no allocation strategy — the same bytes,
reserved for the same lifetime — and buys two things this campaign is otherwise
paying for: a symbol `mem-report` can price, and a map-file bound. It also takes
the term out of the image's allocator arena, which on an RTOS is itself a fixed
static (see W6 and issue 1145). Decision recorded in
[RFC-0002 § 4.4b](../design/0002-rt-execution-model.md); wave W6 below.

The third thing this section used to claim — a task stack that no longer carries
a term proportional to the subscription graph — is **not** what W6 bought,
because the stack never carried it: the same phase-271 change that made the
arena a borrowed slice took it out of the frame. The FreeRTOS app-task stack is
a separate number, and issue 1146 has now DERIVED it: 14 images built and run,
worst app-task peak 36 152 bytes on the Rust path, so the 384 KiB default became
128 KiB and every image prints its own peak at boot. It moved for its own
reasons, not for the arena's.

### 2. Component buffers — 1:1 with per-field storage mode

```rust
// packages/cli/nros-cli-core/src/codegen/entry/emit_cpp.rs:390
"alignas(::{cls}) static unsigned char __nros_comp_buf_{i}[sizeof(::{cls})];"
```

`sizeof(component class)`, which inlines its deserialised message members. This
is the storage that RFC-0033's per-field `mode` actually moves — `heap` and
`view` shrink it, `inline` does not.

**The distinction that decides this phase:** wire buffers hold *serialised* CDR
and are unaffected by `mode`; component buffers hold *deserialised* messages and
are affected 1:1. Conflating them is how a field-mode change gets predicted to
save 49 KiB and saves none of it.

### 3. Executor arena — a 4.9x hand-tuned guess

[Issue 0810](../issues/0810-executor-arena-sized-by-worst-case-shape.md): the
derivation budgets every slot at `sizeof(ActionClient)`, giving 254,720 B for a
board that registers no action clients; the image ships a hand-picked 52,224 B.
Unchecked in both directions, and undersizing fails at runtime.

## Amendment 2026-08-29 — four additions from a board measurement

Added after a session that took the mr_canhubk3/s32k344 action image from
98.73 % SRAM (non-functional: it could not be instrumented) to 85.60 % and
working. Three of the four were absent from this campaign entirely.

### A. Tightly-coupled memory was never a placement target

This document counts `458,752 B of SRAM+DTCM` in its denominator and then never
places anything in the DTCM half. Measured on the action image:

```
RAM   323 528 / 327 680   98.73 %
ITCM        0 /  65 536    0.00 %
DTCM        0 / 131 072    0.00 %
```

192 KiB idle on the same die while the image would not fit. Both regions were
already declared — the devicetree gives them `zephyr,memory-region` and Zephyr's
linker script emits matching `NOLOAD` sections — and nothing in the tree had
ever placed a symbol in either.

[Issue 0880](../issues/0880-tcm-unused-while-sram-exhausted.md) and
`CONFIG_NROS_ZEPHYR_STACKS_IN_DTCM` land the first tenant: the 48 KiB task stack
array. SRAM 98.73 % → 85.60 %, DTCM 0 % → 37.5 %, board boots and reaches its
ready state with zero faults.

**80 KiB of DTCM is still free**, which changes the arithmetic of every lever
below: a pool that cannot be shrunk may still be *moved*.

The constraint that decides what may move: on Cortex-M7 the TCMs hang off the
CPU's private bus and are typically **not reachable by other bus masters**. A
buffer a DMA engine touches must not go there. Stacks are safe by construction;
`LARGE_PAYLOADS` is only safe while the link stays polled/ISR, and
[issue 0852](../issues/0852-zephyr-serial-rx-is-polled-and-overruns.md)'s fix
direction includes eDMA. **Verify reachability before moving any buffer.**

### B. Pools into the tiered arena — the sharing this campaign does not model

Every wire and component pool is sized for its own simultaneous worst case, and
the totals are then added. `SMALL_PAYLOADS` assumes twelve subscribers each four
deep each full; `SERVICE_BUFFERS` assumes every queryable in flight at once.
**They do not peak together, and nothing in this campaign captures that.**

Levers 1 and 2 above shrink each pool against its own worst case. They do not
address the worst cases being summed rather than overlapped. One arena behind
`nros_platform_alloc` (phase 391) sized for the *aggregate* peak instead of the
*sum of individual* peaks is a different and possibly larger win, and it is the
question this campaign has not asked.

What it costs, and why this is a wave rather than a decision:

- **Fragmentation.** Static pools cannot fragment. rlsf's bound is
  `1/SLLEN` internal, but external fragmentation across mixed lifetimes is a
  property of the traffic, not of the allocator. Needs measurement, not
  argument.
- **An allocator call on the RX hot path.** Bounded with rlsf, not free. The
  serial RX path already allocates twice per frame
  (`_Z_SERIAL_MAX_COBS_BUF_SIZE` + `_Z_SERIAL_MFS_SIZE`), so on that path this
  would not be a new class of cost — but on the subscriber path it would be.
- **The bare-metal tier must stay heap-free** (RFC-0034). So this is
  **tier-gated**, never universal: the `inline` tier keeps static pools, and
  only tiers that already admit an allocator may share the arena.

**Do lever 1 first regardless.** Per-type sizing shrinks the pools whether or
not they later share an arena, and a smaller worst case makes the sharing
question cheaper to answer.

**This REOPENS a question this document had closed, and the two texts must not
be left disagreeing.** "Explicitly out of scope" below declines moving payload
buffers to the heap, and it is the same move. Recorded 2026-08-29, on a rebase
that brought the two into one file.

The framing here is better — tier-gated rather than universal, aggregate peak
rather than risk appetite, a wave rather than a decision — and it answers one of
the two grounds the refusal rested on: an allocation that can fail mid-callback
is confined to tiers that already admit an allocator.

It does not answer the other, which is the stronger one. Sharing widens the
arena's block-size range from infrastructure-only (~2^6) to payload-inclusive
(~2^16), and that range is precisely what makes [phase
391](phase-391-allocation-unification-and-tier-model.md)'s constant-time
allocator sizeable. The refusal was not about whether pooling wastes RAM — it
plainly does — but about the cost landing on the allocator that phase 391 is
built around.

So the question is live, not settled either way, and the deciding measurement is
NAMED rather than argued: what the wider block-size range costs rlsf in control
words and in its `1/SLLEN` bound, on a real image, against the aggregate-vs-sum
saving it buys. Until that exists, neither section may be treated as the
campaign's position, and lever 1 proceeds regardless — which both texts already
agree on.

### C. Field storage mode does NOT shrink wire buffers — restated, because it keeps being proposed

Lever 2 above already draws this distinction. Restating it as a decision record
because the opposite has now been proposed twice:

> Offloading a large field to `heap` will reduce the message size and therefore
> the payload buffer.

**It will not.** A `heap` field changes where the *deserialised* value lives; the
*serialised* CDR on the wire is byte-identical. `SMALL_PAYLOADS`,
`LARGE_PAYLOADS` and `SERVICE_BUFFERS` hold serialised bytes and are unmoved by
any per-field mode. `__nros_comp_buf_N` holds the deserialised struct and shrinks
1:1.

The idea underneath it is still right, but it is lever 1, not lever 2: wire
buffers should be sized **per subscriber from its own type's maximum serialised
size**, instead of every subscriber paying a global constant. Today
`LARGE_PAYLOADS` is not computed from any message size at all — it is
`MAX_LARGE_SUBSCRIBERS x RING_DEPTH x SUBSCRIBER_LARGE_SIZE`, three constants a
human picks.

### D. Flash is 4 MiB at 8.3 %, and it is not fungible with RAM

Worth stating so it stops being re-proposed as capacity relief. There is no MMU
and no demand paging: **flash cannot back RAM on this part.** Code is already
XIP, `.data` is 3,564 B, and read-only data already lives in flash — so there is
no copy to eliminate.

What the spare flash IS good for, in order of value to this campaign:

1. **A post-mortem fault log.** Persist the fault dump — PC, LR, thread name,
   stack sentinel state — across reset. This directly serves issue 0852, whose
   whole difficulty has been that the board is at 96 %+ SRAM and cannot afford
   the instrumentation needed to observe its own crash. A flash log costs no
   SRAM at all.
2. **Parameter and configuration storage**, removing any temptation to hold
   defaults in RAM.
3. **ITCM relocation** (`CONFIG_CODE_DATA_RELOCATION`) — flash to ITCM for hot
   paths. A determinism lever, not a capacity one, and the 64 KiB of ITCM is
   still entirely unused.

## Waves

**W1 — pool inventory to full coverage.**
[Issue 0815](../issues/0815-pool-inventory-prices-3-of-46-knobs.md): 46 knobs
found, 3 priced, **66,304 bytes of unpriced pools** — more than the 57,344 that
is priced. Annotate the rest; add a gate rejecting new unannotated pools.
`__nros_comp_buf_N` cannot carry a static annotation (it is generated from
`sizeof`), so the generator emits its figure instead. Do this first: it is the
instrument every later wave is measured with.

**Amended 2026-08-27 — "annotate the rest" is not achievable; the instrument
measures instead. Landed.** All four unpriced pools fail for the same reason,
and `__nros_comp_buf_N` is not the exception this wave assumed, it is the rule:
`SERVICE_BUFFERS` is a product including `ZPICO_MAX_QUERYABLES`, whose default is
*computed*, so there is no integer to write down; `MESSAGE_INFO_TABLE`'s element
gains three fields under `alloc` + `safety-e2e`, which is why [issue
0739](../issues/archived/0739-static-pool-inventory-not-enumerable.md) declined to
annotate it and was right to; `SUBSCRIBER_BUFFERS` is an array of structs. The
size is known to the COMPILER, not to a comment, and a hand-written figure in a
comment is the drift class this tree already gates against
(`check-ffi-struct-mirrors`).

So W1 shipped as `scripts/nros-mem-report.py` / `just mem-report <elf>`: it reads
a built image's symbol table and attributes RAM by symbol, by crate and by
declared pool, with the unattributed gap called out. The declared and measured
mechanisms compose rather than compete — `--check` joins each `// nros-pool:`
formula to its measured symbol and requires agreement on a default-built image,
which turns the inventory's published figures from a claim into a checked fact
(gate `check-mem-report`, plus the fixture-backed test
`static_memory_declared_pools`). W3 is unblocked: a saving can now be reported as
a measured delta between two `--json` runs.

The first thing it measured is [issue
0827](../issues/archived/0827-unused-rmw-pools-dominate-static-ram.md) — static RAM is a
property of the RMW, not of the node, identical to the byte across four roles,
and a talker reserves 80% of its static RAM in pools it cannot reach.

**W2 — precise executor arena.** Entry codegen emits `NROS_ARENA_REQUIRED` as
the sum of *actual* entry sizes; `static_assert` against `ARENA_SIZE` moves the
failure from runtime to build. Encoding the requirement as a linker symbol whose
*size* is the figure lets `nm` check it across the C/Rust boundary without
running anything.

Hand-written `main`s create entities at runtime, have no generated entry, and
cannot be sized statically. **This wave explores that case rather than assuming
it away**: the likely answer is a runtime high-water mark reported at teardown
plus a CI lane that fails when it exceeds the configured arena — the generated
path proves its number statically, the hand-written path measures it, and both
report through one figure.

**W2 IS [issue 0900](../issues/archived/0900-arena-slots-budgeted-at-action-client-worst-case.md),
and its runtime half has landed.** The issue was filed from the other end —
measuring that `ARENA_SIZE` is 74,240 bytes on every generated config in the
tree, a talker included, while a timer-only executor claims **32 bytes** — and
only then met this wave. Recorded here so the two do not diverge into separate
efforts against one defect.

Delivered against the plan above:

* the runtime measurement this wave predicted — `Executor::arena_used()` /
  `arena_capacity()` plus a one-shot first-spin advisory naming the
  `NROS_EXECUTOR_ARENA_SIZE` value to set. The arena is a BUMP allocator, so
  that figure is exact rather than a high-water estimate;
* `NROS_EXECUTOR_ACTION_CLIENTS`, which stops every slot being budgeted at the
  ActionClient worst case (74,240 -> 16,384 at the defaults, with the default
  byte-identical to the old formula so no image moves).

Still owed by this wave: the STATIC half — `NROS_ARENA_REQUIRED` emitted by
entry codegen and checked by `nm` — and a CI lane.

~~Note the correction issue 0900 records: the arena is **inline on the task
stack**, not in `.bss`, so `mem-report` cannot see it and a linker-symbol check
will not either. Sizing it needs a stack probe, which this wave's `nm` plan did
not anticipate.~~ **That paragraph is wrong twice over and is struck rather than
deleted, because it is the sentence that sent W6 looking for the arena on a task
stack.** The arena has been a BORROWED SLICE since phase-271 — placement is the
caller's — and since W6 the Rust caller's backing is a named `.bss` static
(`nros_node::executor::backing::EXECUTOR_BACKING`), which `mem-report` and `nm`
both see. The stack probe it asks for exists and was run for a different
question (issue 1146): on the FreeRTOS app task the arena contributes nothing to
the frame, and the whole register pass peaks at 36 152 bytes.

**W3 — per-subscriber wire sizing.** Lever 1. Requires W1 so the saving is
measured rather than asserted.

**Surveyed 2026-08-27. The mechanism is more built than this doc assumed, and
the missing piece has a language reason.**

What already exists, end to end: `rx_buffer_hint` on `TopicDesc` and on
`rmw_subscription_options_t`; `alloc_payload_block(hint)` in the zenoh shim,
which picks the large class when the hint exceeds
`ZPICO_SUBSCRIBER_SIZE_THRESHOLD` (2048); and, from phase 380,
`M::MAX_SERIALIZED_SIZE_XCDR1`/`_XCDR2` as PROVIDED consts computed from the
schema, plus `size::bound_fits::<M>` which takes the larger of the two.

What was missing, at survey time, is that **nothing set the hint**. The only
setter in the tree was one bench site; `rust_adapter` passed a literal `0`. So
every real subscription took the small class, and the large pool — 2 x 4 x 16384
= 131,072 B, already reserved — sat unused.

**Corrected 2026-08-29: the RUST half of that is fixed and the C/C++ half is
not.** `2adf2b739` wired `create_subscription::<M>` to pass
`subscription_rx_hint::<M>(RX_BUF)` — the type's own bound, falling back to
`RX_BUF` only where no bound exists. A Rust subscription to a 4 KiB type now
routes large.

A C or C++ subscription to the same type still hints 0. The field is declared,
`rust_adapter` reads it, the shim routes by it — and no source under
`packages/api/nros-c`, `packages/api/nros-cpp`, `packages/cli/rosidl-*` or
`examples/` writes it. The producer is the Rust executor and nothing else, so
W3a's saving applies to half the tree.
[Issue 0896](../issues/archived/0896-c-cpp-subscriptions-never-state-a-buffer-hint.md)
carries the survey and the constraint that decides the design: the bound is a
PROVIDED const computed from `FIELDS`, a C message has no such trait, and
whatever carries the number to the C call site must not become a SECOND
computation of it — that is the sizes-header mirror class
(0088 -> 0114 -> 0122 -> 0123 -> 0245 -> 0268) one lane over.

The cost of that shows up in the build error `create_subscription` raises when a
type does not fit: *"Raise the knob to at least the type's bound."* That knob is
GLOBAL. For a 4 KiB message type:

| remedy | SMALL_PAYLOADS | delta |
| --- | ---: | ---: |
| today: raise `ZPICO_SUBSCRIBER_BUFFER_SIZE` 1024 -> 4096 | 8 x 4 x 4096 = 131,072 | **+98,304 B** |
| route it to the large class instead | 8 x 4 x 1024 = 32,768 | **0** — the large pool is already there |

And it is charged twice: `NROS_SUBSCRIPTION_BUFFER_SIZE` sizes the executor
arena entry as well, so raising it grows every arena slot too.

**Why the split is not one wave.** The arena entry is
`SubInfoEntry<M, F, const RX_BUF: usize>`, and on stable Rust an associated
const of a type parameter cannot be used as a const-generic argument
(`error: generic parameters may not be used in const operations`, checked on
edition 2024). So:

- **W3a — route the zenoh block by the type's bound. LANDED for Rust
  (`2adf2b739`); the C/C++ half is issue 0896 and is NOT done.**
  `rx_buffer_hint` is a
  runtime `usize`, so `create_subscription::<M>` can pass
  `max(XCDR1, XCDR2)` with no unstable feature. A type between the small size
  and `ZPICO_SUBSCRIBER_LARGE_SIZE` stops being a build error and starts being
  a large-class subscriber. Unbounded types keep the default: phase 380 is
  explicit that `None` means "no bound exists", never "unknown" — do not size a
  buffer from a fallback.
- **W3b — arena sizing, at any site where the type is named. LANDED.** The
  constraint is narrower than "only codegen": a *generic parameter* may not
  appear in a const operation, but a *concrete type's* associated const may, and
  that compiles on stable (checked, edition 2024). `emit_rust.rs` turns out to
  emit no subscriptions at all — the Rust call site is user code — so the fix is
  `nros::rx_buffer_for!(Msg)`, expanding at whatever site names the type:

  ```rust
  node.subscription::<PointCloud2>("points")
      .rx_buffer::<{ nros::rx_buffer_for!(PointCloud2) }>()
      .build(on_cloud)?;
  ```

  `.rx_buffer::<N>()` already existed; what was missing is a number that cannot
  drift. A literal is correct until a field is appended, after which the sample
  is received, ACKed and dropped at the transport — the failure
  `report_dropped_take` describes and that needs a packet capture to attribute.
  An unbounded type expands to `NROS_SUBSCRIPTION_BUFFER_SIZE`, not to an
  invented number, because phase 380 forbids sizing a buffer from a fallback.

  Tested from OUTSIDE the crate (a macro body resolves in the caller, so an
  in-crate test would see private names a consumer cannot), including use in
  const-generic position — the property the whole wave exists for.

**W3 has no MEASURED saving yet.** W3a changes which size class a subscription
routes to; it does not by itself shrink a pool, and the 98,304 B in the table
above is an avoided COST on a hypothetical 4 KiB type, not an observed delta on
a real image. Per this phase's rule, W3 claims nothing until `just mem-report
--json --baseline` shows it on an image with a type large enough to route
differently — and no example in the tree has one today, which is its own finding
about the fixture corpus rather than about the wave.

### W3 remainder — the goal is a buffer derived from the message size, everywhere

W3a and W3b each derive a buffer from the type. Neither makes it the norm, and
the gap between "possible" and "always" is where every remaining byte is.

Stated as a target so the remaining waves can be judged against it: **no
subscription and no publish helper in any image sizes a buffer from a global
constant when the type has a bound.** Where a type genuinely has no bound, the
fallback must be loud and must name the field that costs it.

Four things stand between here and that, and they are not the same kind of
problem.

**1. Rust is opt-in.** `rx_buffer_for!(Msg)` is correct and tested, and it is
only used where a consumer writes it. A consumer who does not know the macro
exists gets the default, silently, which is the same failure the macro was
written to prevent — the sample is received, ACKed and dropped. Making the
derived size the DEFAULT at a site that names the type, with the global as the
explicit opt-out, is the whole of W3c.

**RESOLVED by W3c (2026-09-07), on the backends that can reach the bound.** The
derivation is what a subscription gets without asking and `.rx_buffer::<N>()`
is the opt-out. The residue is issue 1179: at a type-erased registration site
the only bound in scope is `MessageForRmw`, which requires
`nros_serdes::schema::Message` only on a backend declaring `type-descriptors`
(Cyclone), so on zenoh and XRCE the default is unchanged and
`.rx_buffer_from_type()` / `rx_buffer_for!` — which name the schema themselves
— remain the route.

**2. C/C++ has no site that names the type — and this is the design fork.**
RFC-0043 typed components subscribe RAW: the type reaches the runtime as a
STRING. `add_arena_subscription_c_callback::<BUF>` therefore takes
`BUF = DEFAULT_RX_BUF_SIZE`, a global, on all three register variants, and no
per-call number can vary it because a const generic is fixed at the Rust call
site — which is inside nros-cpp, not in user code.

phase-402 already delivered the runtime half: `nros_cpp_subscription_options_t
.rx_buffer_hint` is read and passed to the backend (issue 0896 is stale on this
point — it says the register has no slot, which was true when filed). But the
hint is a `usize` routing a backend size CLASS. **The arena entry is still a
const-generic array.** So the C/C++ path needs one of:

  * **(a) monomorphise per type.** Codegen emits a Rust shim per message type
    that instantiates the const generic. Exact, no runtime cost, and it puts a
    generated Rust crate into a C-only image — which the 0896 survey found is
    exactly what does not exist there today (`packages/api/nros-c` links no
    message crate, and no `nros_ws_runtime` is generated for a C workspace).
  * **(b) make the arena entry runtime-sized** within a bounded pool, so the
    hint sizes it the way it already sizes the backend block. Removes the const
    generic from the path entirely and serves both languages, at the cost of the
    entry no longer being a plain `[u8; N]`.

  (a) preserves the static shape and costs build machinery; (b) preserves the
  build shape and costs the static guarantee this campaign is otherwise
  defending. **Neither is obviously right and the choice should be made
  deliberately, not by whichever is implemented first.**

**3. The number has to exist in C at all.** Issue 0896 layers 1-2: one
traversal in `rosidl-codegen` emitting both the field-expression string and the
`nros_serdes::FieldType` value, then `<PREFIX>_RX_MAX_SERIALIZED_SIZE_XCDR{1,2}`
from the C and C++ template packs, tested equal to the Rust const for the same
type. This is a prerequisite for 2 under either option, and it independently
fixes the publish side (layer 3): the generated helpers stack
`NROS_PUB_BUFFER_SIZE` (global, default 256, checked against nothing).

**4. Unbounded types are the residue, and two of them are ours.** A bound the
`.msg` does not state cannot be invented — phase 380's rule. Two follow-ons,
both already scoped in 0896: name the offending FIELD when the bound is `None`
(`size::first_unbounded`), and thread RFC-0033 `cap` as a **transmit-only**
bound, which forces `_TX_MAX_SERIALIZED_SIZE_*` and `_RX_MAX_SERIALIZED_SIZE_*`
apart. A cap bounds our storage; a remote publisher is bound by the `.msg` and
may send more, so sizing a RECEIVE buffer from a cap reintroduces the drop this
whole lever exists to stop.

### Waves

**W3c — derived-by-default on the Rust path.** `rx_buffer_for!` becomes what a
subscription gets without asking; the global becomes the opt-out. Acceptance:
an example that subscribes to a bounded type over the small class shows the
change in `mem-report --baseline` without its source being edited.

**LANDED 2026-09-07, and the acceptance is NOT met. Three things about the
wave were different from what this entry assumed, and all three were measured
before any code was written.**

*1. The mechanism moved.* `rx_buffer_for!` is no longer the thing to make
default. phase-403 W2 landed `.rx_buffer_from_type()`, a RUNTIME-sized sibling
that sizes the arena ALLOCATION rather than a const-generic array, and its own
doc named this wave as the decision it was deferring: "sizing every
subscription from its type by default would move every image's arena occupancy
at once, which is a decision, not a refactor". W3c is that decision, taken on
that mechanism. `rx_buffer_for!` keeps a narrower job — `.message_info()` and
`.safety()` entries hold a real inline `[u8; RX]`, so their size can still only
come from a const-generic argument named at the call site.

*What landed:* the derivation moved into
`Executor::register_subscription_buffered_on`, the ONE choke point all four
default registration sites funnel through (`create_subscription`,
`create_subscription_in`, the builder's `build()`, and the `/clock` time
source), so it is the default at all four rather than at whichever one a wave
happened to touch. `rx_bytes: None` there means DERIVE; `.rx_buffer::<N>()`
passes `Some(N)` and is the opt-out. Naming a number is now the only way to
spend more arena than the type needs, which is the direction that should
require saying so.

*The saving, measured* (`cargo test -p nros-node --features
std,alloc,needs-type-descriptors --lib`, positive control = revert the
choke-point line and watch the same test go red at `left: 5376, right: 2340`):
a 12-byte bounded type at `QoSProfile::default().depth` costs **13,632 -> 2,500
bytes** of arena per registration, 11,132 recovered; at depth 2, **5,376 ->
2,340**.

*2. The acceptance names an example that does not exist.* "A bounded type over
the small class" means a bound above `SMALL_CLASS_CEILING` (2048). Every Rust
subscription in `examples/` was enumerated: `StringMsg` (`FieldType::String`,
unbounded — so no bound to derive), `Int32` (4 bytes), `SensorReading`
(hand-written, no schema), the px4 types (all well under 2048). None is over
the small class. This wave's own W3 section already recorded the fact —
"no example in the tree has one today, which is its own finding about the
fixture corpus rather than about the wave" — and W3c's acceptance was written
as though it had been fixed.

*3. `mem-report` cannot see this saving on ANY image, and that is structural.*
The derived number reaches `buffered_region_size` / `TripleBuffer::init` /
`SpscRing::init` — runtime allocations out of the executor arena. The arena is
`nros_node::executor::backing::EXECUTOR_BACKING`, a `.bss` static whose size is
`config::ARENA_SIZE`, derived in `nros-node/build.rs` from KNOBS and declared
entity COUNTS. No registration can move it. Measured directly, before and
after, on a real ELF containing a derived-default registration:
`EXECUTOR_BACKING` is **86,216 bytes on both sides**, and every other RAM
symbol is unchanged. Positive control for that probe: rebuilding with
`NROS_SUBSCRIPTION_BUFFER_SIZE=2048` moves the same symbol to **98,504**
(+12,288), so the probe detects exactly the class of change W3c would have had
to produce.

So a mem-report delta from this lever needs the BUILD-TIME arena derivation to
learn the per-type bounds, which is `nros-node/build.rs`'s `rx_recv_size` term
and `nros_derive_message_bound_knobs` — phase-403 step 3's lane, not this one,
and deliberately deferred there ("a DESIGN choice with two bad options").

*Filed:* [issue 1179](../issues/1179-derived-rx-default-unreachable-without-schema.md)
— the default derives only on a backend declaring `type-descriptors` (today:
Cyclone alone, verified across all four `nros-rmw.toml`), because
`MessageForRmw` carries no schema on the other arm and phase-380 W4 refused to
tighten it. The same constraint makes W3a inert on zenoh, which is worth
knowing before quoting W3a's "a Rust subscription to a 4 KiB type now routes
large" — true on Cyclone, false on the backend that has the size classes.
[issue 1180](../issues/1180-mem-report-baseline-cannot-match-llvm-suffixed-symbols.md)
— `mem-report --baseline` matches symbols by name including the per-build
`.llvm.<hash>` suffix, so it silently prints "no delta" for `EXECUTOR_BACKING`
whatever happened to it. That one nearly turned this wave's measurement into a
probe that could only return nothing.

**W3d — emit the constants, and retarget the publish helper.** 0896 layers 1-3.
Acceptance: the emitted constant equals the Rust const for every type in the
message corpus (one test, both encodings), and a C image's publish helpers stop
referencing `NROS_PUB_BUFFER_SIZE`.

**LANDED, as phase-408 W1** — recorded here 2026-09-06, because the work moved
phases and this wave was left unmarked for long enough to read as unstarted.
`packs/c/message.h.jinja:178` sizes a bounded type's helper from
`<PREFIX>_TX_MAX_SERIALIZED_SIZE`; `rosidl-codegen/tests/
message_size_bound_parity.rs` asserts the C constants, the C++ constants and
`nros_serdes::size::max_serialized_size` are one number, both encodings.

The second acceptance clause was WRONG as written and the outcome is better than
it asked for: the helpers still name `NROS_PUB_BUFFER_SIZE` in the arm for a
type that genuinely has no bound, with the reason in a comment beside it.
Deleting it there leaves an unbounded type with no publish helper. The knob
stopped being the DEFAULT — that was the wave's point; the criterion confused it
with removing the symbol.

**W3e — SUPERSEDED by [phase 408](phase-408-cpp-message-derived-buffers.md);
the fork it described does not exist.** W3e asked for a decision between
monomorphising the const generic per type and making the arena entry
runtime-sized. Surveying the code for phase 408 found the second is **already
built on the path C/C++ actually takes**: `add_arena_subscription_c_callback`
stores no `[u8; RX_BUF]`, it calls `arena_alloc_with_trailing` with
`buffered_region_size(qos.depth, RX_BUF)` and initialises a `TripleBuffer` /
`SpscRing` over those trailing bytes. `RX_BUF` is consumed only as a VALUE, so
it is a runtime parameter that has not been spelled as one.

The distinction the fork was reaching for is still real, and worth stating in
the form that survives: the TYPED Rust entries (`SubInfoEntry`,
`SubSafetyEntry`) do hold `buffer: [u8; RX_BUF]`, so for them the const generic
is load-bearing and `rx_buffer_for!` is the right answer. The BUFFERED entries
are trailing-allocated. Two entry shapes in one tree — a claim about "the arena
buffer" that does not say which is not a claim about anything.

**W3f — Cyclone consumes the hint, or records that it will not. DELIVERED
2026-09-02, by the second arm.** It records that it will not, in the RMW's own
docs (`docs/reference/cyclonedds-known-limitations.md`, "`rx_buffer_hint` is
inapplicable here") and at the parameter itself in `subscription_create`.

The wave offered "either wire it or state that the hint is zenoh-only". Reading
the backend showed the first arm has nothing to reach: there is no receive buffer
in this backend to size. The sample arrives in a serdata sized by the sample, and
the destination is the caller's buffer, whose capacity arrives on every `take`.
The one candidate consumer — the `dds_ostream` that re-serialised the typed
sample and grew by `realloc` — went with the CDR round trip in
[issue 0969](../issues/0969-cyclone-take-cdr-round-trip.md). So the hint is
INAPPLICABLE here rather than unimplemented, and the ordering note below is
correspondingly wrong: W3f does not make W3c/W3d/W3e observable on a Cyclone
image, because nothing in this backend was ever going to route on the hint. What
IS observable there is the executor arena, which nano-ros sizes itself from the
same bound. Measure the arena, not the backend.

Original text follows. Filed as
[issue 0958](../issues/archived/0958-cyclonedds-ignores-rx-buffer-hint.md), which carries
the evidence: `subscription_create` takes the options struct and names none of
it, so the parameter is discarded at a comment. `grep
rx_buffer_hint packages/rmw/cyclonedds/` returns nothing today, so a consumer on
that backend gets no routing from any of the above. Either wire it or state in
the RMW's own docs that the hint is zenoh-only, so a consumer stops looking for
an effect that cannot occur. Cheap, and it is the difference between W3 helping
one backend and helping the tree.

**W3g — unbounded diagnostics and the TX/RX split.** 0896 layers 5-6, in that
order: the diagnostic is cheap and immediately useful, the split is a naming
change that must land before caps are honoured anywhere.

**LANDED, as phase-403 W0/W6** — recorded here 2026-09-06, same reason as W3d.
The split is `_TX_MAX_SERIALIZED_SIZE` / `_RX_MAX_SERIALIZED_SIZE`, classified
in `bounds::BoundState` so the emitted constants and the exported inventory
cannot disagree about which encoding feeds which direction (TX writes XCDR1; RX
must hold either, so it takes the max — RFC-0055). The diagnostic is a POISON
TOKEN, `generator/common.rs:1964`: an unbounded type emits
`NROS_UNBOUNDED__<type>__field_<name>`, an undefined identifier, so the type
fails to COMPILE and the error names the offending field. Same decision issue
0964 reached from the C++ side.

**Ordering note.** W3d unblocks W3e, and W3f is independent of both — a
consumer on Cyclone sees nothing from W3c/W3d/W3e until W3f exists. If the
motivating consumer is a Cyclone image (the an536 lane is), W3f is not the
last wave; it is the first one that makes any of the others observable there.

**Amended 2026-09-02 — the second sentence was wrong.** W3f closed by recording
that the hint is inapplicable, so it does not make W3c/W3d/W3e observable on
Cyclone; nothing there was ever going to route on the hint. The an536 lane's
saving comes from the arena, which nano-ros sizes itself, so W3c/W3d/W3e reach
that consumer without W3f at all. What W3f bought is that nobody spends a
measurement looking for a backend effect that cannot occur.

**W4 — drop the network stack from serial images.** 27,760 B.

**TRIAGE ANSWERED (2026-08-27): headers only.** zenoh-pico's Zephyr layer needs
Zephyr's networking HEADERS at compile time and does not pull the pools. The
27,760 B is enabled by the image's own Kconfig, not by the transport.

Three independent lines of evidence:

*1. Kconfig dependency chains.* `config NROS_RMW_ZENOH` (zephyr/Kconfig) has NO
`depends on NET_SOCKETS` and selects nothing networking. Its siblings do —
`NROS_RMW_XRCE` is `depends on NET_SOCKETS`, `NROS_RMW_CYCLONEDDS` is
`depends on NET_SOCKETS && POSIX_API && CPP`. `NROS_ZENOH_LINK_SERIAL` has no
networking dependency either, and `NROS_TRANSPORT_SERIAL` only
`select NROS_ZENOH_LINK_SERIAL`. So nothing in our Kconfig requires networking
for a zenoh serial image.

*2. The #include graph.* In zenoh-pico's `src/system/zephyr/network.c`, `<netdb.h>`
and `<sys/socket.h>` are already guarded by `#if defined(CONFIG_NET_SOCKETS)`.
`<zephyr/net/net_if.h>` is NOT guarded — that is the one wart — but every
`net_if_*` USE is: all 19 call sites sit inside link-feature guards
(`Z_FEATURE_LINK_UDP_MULTICAST` and friends), 0 unguarded, checked by walking
the preprocessor stack rather than by eye. So on a serial build no networking
code is compiled; only a header is included.

*3. Symbols in a built image.* `zephyr-workspace/build-cortex-m-c-talker-zenoh`
(mps2/an385, zenoh over TCP) carries **22,580 B** of networking RAM — the same
order as the mr_canhubk3 figure on a different board/config. The largest are
`_k_mem_slab_buf_tcp_conns_slab` 9,600, `net_buf_data_rx_bufs` 4,096,
`net_buf_data_tx_bufs` 4,096. Every one is a Zephyr net-subsystem symbol; none
belongs to zenoh-pico.

And the pools have a named source: `examples/zephyr/c/talker/prj-zenoh.conf`
sets `CONFIG_NET_TCP=y`, `NET_PKT_RX/TX_COUNT=32`, `NET_BUF_RX/TX_COUNT=64`.
That is the image's config, correct for a TCP image and simply inherited by
anything that copies it.

**So the fix is conf-level, not code-level**, and needs no vendored change: a
serial image should not enable `NETWORKING`/`NET_TCP`/`NET_PKT_*`/`NET_BUF_*`.

**One caveat that still needs a build to settle.** Because
`#include <zephyr/net/net_if.h>` is unconditional, a serial image still needs
Zephyr's net headers to COMPILE with `CONFIG_NETWORKING=n`. Zephyr ships those
headers unconditionally and they are declaration-only, so this is expected to
hold — but it is not proven here, and if it does not hold the remedy is guarding
that include in zenoh-pico, which is VENDORED and must be reported rather than
patched in place.

### MEASURED 2026-09-02 — 38,334 B on mps2/an385, and the caveat is settled

Two images built from one tree, differing ONLY in networking:

| | `.bss`+`.data` | FLASH |
| --- | ---: | ---: |
| zenoh over TCP (`mps2-an385.conf`) | 1,212,854 B | 362,956 B |
| zenoh over serial (`+ mps2-an385-serial.conf`) | 1,174,520 B | 284,260 B |
| **saving** | **38,334 B** | 78,696 B |

103 symbols disappear (38,917 B); the serial path adds 1,080 B back
(`_z_serial_rx_storage` and its ring). The config diff outside networking is two
lines, `CONFIG_PIPES` and `CONFIG_POSIX_HOST_NAME_MAX`, both selected BY
networking.

**Larger than this wave's own estimates** — 27,760 B reported for mr_canhubk3,
22,580 B cross-checked here — because those counted symbols matching `net_*`
patterns. The stack also drags in `work_q_stack` (4,160), `rx_stack` (1,600),
`mgmt_stack` (896) and `contexts` (2,752), which no `net_` grep finds.

**The open caveat is settled, favourably.** zenoh-pico's unguarded
`#include <zephyr/net/net_if.h>` COMPILES cleanly with `CONFIG_NETWORKING=n` —
the headers are declaration-only, as predicted. **No vendored change is needed.**
What the image does need is two symbols the net stack was supplying implicitly,
found by LINKING rather than reading: `ring_buf_get` (zenoh-pico
`src/system/zephyr/network.c:941`) and `z_impl_sys_rand_get` (`zpico.c`), so the
fragment names `CONFIG_RING_BUFFER` and `CONFIG_TEST_RANDOM_GENERATOR`.

**The fix is a BOARD fragment, not a leaf conf — the triage above is wrong on
that point.** Zephyr merges last-wins and `mps2-an385.conf` is appended after
every leaf conf while setting `CONFIG_NET_NATIVE=y` / `NET_DRIVERS=y` /
`NET_CONFIG_SETTINGS=y` unconditionally, so a `CONFIG_NETWORKING=n` written in
the leaf is DEAD and looks like it should work. Issue 0876 exactly, one wave
over. `cmake/zephyr/mps2-an385-serial.conf` is a DELTA listed after the board
fragment, never in place of it.

**One wrong measurement, recorded so it is not repeated.** The first version of
that fragment REPLACED the board conf instead of delta-ing it, which also
dropped `MAIN_STACK_SIZE` (131072 -> 16384), `HW_STACK_PROTECTION` and
`MPU_STACK_GUARD`. It reported a 613,034-byte "saving" of which ~29,000 was
networking. A saving measured against a differently-configured image is not a
saving.

**Still not measured:** the mr_canhubk3 board itself, for the reason below. The
38,334 B is mps2/an385 with a C talker, and it is this wave's number.

**NOT MEASURED, and deliberately not guessed.** The mr_canhubk3/s32k344 board is
not in this tree — no board directory, no conf, no `build-board/` — so its image
cannot be built or measured here. `scripts/nros-mem-report.py` and
`just mem-report` do not exist in this tree or on `origin/main` either, so no
`--json --baseline` delta was available. Per this phase's own rule that no wave
claims a saving it did not measure, the 27,760 B remains the originally reported
figure and this wave contributes the triage plus the 22,580 B cross-check above,
not a new saving.

**W5 — queryable pools sized by declaration, not by guess.** Lever 1, and the
largest single figure this phase has measured: 144,128 B on a native talker,
39% of its static RAM, in service buffers for services it does not have.

`ZPICO_MAX_QUERYABLES` decides that pool (it sizes `SERVICE_BUFFERS` as
`ZPICO_MAX_SESSIONS * ZPICO_MAX_QUERYABLES`, and the C shim's queryable table
alongside). Its default is `if hosted { 32 } else { 8 }` — a literal chosen for
headroom in `nros-zpico-build`, because at that point nothing knows the answer.
Six inputs decide the right number and none of them meet: the app's service
count (known only to the resolved model), the parameter services (6) and
lifecycle services (5) (known only to `nros-node`, behind cargo features),
`ZPICO_MAX_SESSIONS`, the hosted/embedded split, and the literal.

### The shape

One declaration site, two front-ends, one consumer.

```
                    system.toml + launch files
                              |
                       nros sync resolves
                              |
                    +---------v---------+
                    |   SystemModel     |  app service-server count
                    | (build artifact)  |  features = [param_services, lifecycle]
                    +----+---------+----+
              Rust entry |         | C/C++ entry
            nros::main!  |         | nano_ros_entry()
                         v         v
                  one declared figure, delivered as env to cargo
                              |
                              v
             nros-zpico-build  -- sizes --> C shim queryable table
                               `- sizes --> SERVICE_BUFFERS (Rust pool)
                                    ^
                                    | adds, from nros-node
                     PARAM_SERVICE_QUERYABLES / LIFECYCLE_SERVICE_QUERYABLES
```

DECLARED, from the model: how many service servers the application has, and
whether the infrastructure services exist. DERIVED, from Rust: how many
queryables each of those features costs. That split is what keeps issue 0460
closed — codegen sees the user's entities and never the runtime's, so it must
never own the second number.

### Why not const generics

Checked, because the W5 endgame in [phase
391](phase-391-allocation-unification-and-tier-model.md) sized component cells
exactly that way and the parallel is tempting. `SERVICE_BUFFERS` is a private
`static mut`: no header, no `#[no_mangle]`, no `repr(C)`. C round-trips one
opaque `*mut c_void` token (`session_index * ZPICO_MAX_QUERYABLES + local`) that
it never does arithmetic on, and bounds its own handles against its own table.
So the two tables are NOT layout-coupled — this is not the issue-0135 class, and
a const generic would reach no non-Rust consumer.

It is still the wrong tool. A const generic needs a TYPE to carry the bound. A
Rust entry has one (`Node::ENTITY_BOUNDS`); a C/C++ entry does not — it is
cmake-driven through `nano_ros_entry()`. Manufacturing one leaves two exits,
both bad: a non-generic C entry point that picks some N (a hand-picked number
again, which is the thing being removed), or a sizing parameter on the C API,
which stops it being a thin wrapper of Rust.

### The channel already exists

`*_OPAQUE_U64S` is the established thin-wrapper channel and it runs Rust -> C:
Rust owns the type, a build step computes `size_of`, a generated header carries
the number, C declares opaque storage. This need runs the other way — the
application declares, the backend consumes — which is the same direction W2's
`NROS_ARENA_REQUIRED` needs.

The delivery mechanism is proven, not hypothetical: phase-351 W5's
`nros_resolve_board_facts` resolves facts through a CLI verb and attaches them
with `corrosion_set_env_vars`, which reaches the cargo invocation where
`set(ENV{...})` does not (issue 0460). A declared entity figure is one more fact
on that path, and `nros ws model-dims` is the existing seam for asking the model
a question from one implementation rather than a second one in cmake.

Net C/C++ API change: none. No function, no parameter, no generic in a header.

### Waves

* **W5.a — the counts get one definition.** LANDED. They had seven spellings and
  none was a definition; two were wrong, both saying lifecycle was 6 (it is 5,
  so the widely-quoted "twelve slots before the application declares anything"
  is eleven), including the message a user sees when the table overflows.
  `check-infra-queryable-counts` ties each constant to the number of creation
  sites, because a constant alone is still a hand-typed literal that drifts the
  same way the prose did.

* **W5.b — the model answers the question.** SPLIT, after looking at what the
  model actually contains. This wave was written assuming the resolved model
  could answer both halves. It cannot, and the difference decides who can do
  the work.

  MEASURED — every resolved model in the tree
  (`examples/*/build/nros/models/*/*.yaml`), full key set: `meta`, `structure`
  (`scopes`, `nodes` -> `pkg`, `exec`, `node_name`, `params`, `remaps`,
  `lifecycle_autostart`, `scope`), `execution` (`deploy`, `features`, `tiers`,
  `bridges`, `bindings`). There is NO entity inventory: a node's publishers,
  subscriptions and service servers appear nowhere.

  * **W5.b1 — the infrastructure flags.** LANDED. `execution.features` carries
    `param_services` and `lifecycle` verbatim, which is exactly the half a build
    script cannot otherwise see (cargo exposes no other crate's features).
    `nros ws entity-facts` reports it; W5.c delivers it; the consumer adds what
    those features COST rather than being told. Unknown names are ignored, not
    refused — `safety` is a real feature here and is not a queryable question.

    Measured over every resolved model in the tree: 92 `none`, 22
    `param+lifecycle`, 1 `lifecycle`. On a model declaring neither, the table
    goes 32 -> 8 slots; on one declaring both, 32 -> 19.

  * **W5.b2 — the application's own service-server count.** NOT AVAILABLE, and
    this is the THIRD reading of the same question. The first draft said the
    model could not answer it and proposed extending the spec. The second said
    `ros-launch-manifest` already modelled it, so it was available. Both were
    wrong, in opposite directions, and the truth is in between.

    The SPEC models it: `structure.services` maps a service FQN to
    `ServiceWiring { srv_type, server: Vec<String>, client }`, `structure.actions`
    is the same type, and rlm's own `service-wiring` rule already reasons over
    `!svc.server.is_empty()`. Extending it would have been wrong as well as
    unnecessary — **rlm is the platform-neutral SPEC, `play_launch` is its Linux
    implementation, and neither may carry nano-ros concerns.** Service wiring is
    a general ROS graph concept; a static-RAM sizing rule is not, and stays in
    this tree's build layer.

    The RESOLVER never emits it. MEASURED: zero of the 115 resolved models in
    this tree carry ANY layer-1 wiring — no `topics`, no `services`, no
    `actions`. A plain `<node>` launch element names a node; it does not say
    what that node serves, and nothing else in the resolver's inputs does
    either. `nros metadata`'s component sidecar does not either: it carries
    name, class, sources and callback groups, no endpoint inventory.

    So an absent `services` map does NOT mean "this system has no service
    servers". `examples/workspaces/c`'s `service_server_model.yaml` describes
    exactly one node, called `add_server`, and carries no `services` key at all.

    **`nros ws entity-facts` therefore ABSTAINS rather than reporting a zero it
    cannot support.** Reporting 0 there would size the table to the
    infrastructure alone and exhaust it the moment that node registers — a
    confident wrong number, sized exactly, which is this campaign's own failure
    shape rather than a new one. The discriminator is whether ANY wiring was
    described: if it was, an empty `services` map is a real zero; if nothing
    was, the question is unanswered and the verb says nothing. The app term
    stays `UNDECLARED_HEADROOM` in the consumer, labelled there as the guess it
    is.

    The counting machinery is landed and correct for the day the resolver does
    emit wiring: endpoint refs are counted per SERVER, not per service name (two
    nodes serving one name are two queryables), and an action server costs
    `ACTION_SERVER_QUERYABLES` because an action is three services on the wire.

  ### If `services[].server` arrives, it is AUTHORITATIVE

  DECIDED, and unchanged by the above — it is the rule for when the input
  exists. The launch declaration is the contract: an image is sized for exactly
  the service servers its model declares.

  The alternative was to treat it as a floor and add headroom, which is safe and
  wrong — it re-creates in miniature the guess this whole wave exists to delete,
  and a guess derived from something real is still a guess nobody can audit.

  What being authoritative COSTS is paid in W5.e, in this wave: a node that
  creates a service server its model does not declare exhausts the table, so
  "the declaration is wrong" and "the table is too small" become the same event
  and the runtime must name the first.

  ### Future work: counting in code

  The declaration is a stepping stone, not the destination — and after the
  measurement above it is not even the near-term path, since nothing populates
  it today. The count a node's CODE produces is the ground truth, and deriving
  it there would make a launch declaration checkable rather than trusted — the
  same move W5.a made for the infrastructure counts, where a constant beside the
  creation sites replaced seven prose spellings and a gate holds it to the sites
  themselves.

  Out of scope for W5 and deliberately not designed here: it needs the entity
  set to be visible at build time in BOTH languages, which is the asymmetry that
  ruled out const generics (`Node::ENTITY_BOUNDS` exists for Rust; C/C++
  entities are created at runtime in C with no declaration site). Solving that
  is the real end state; W5 gets the infrastructure saving without waiting for
  it.

* **W5.c — delivery.** LANDED. `NanoRosEntityFacts.cmake` runs the verb per
  entry and attaches the result with `corrosion_set_env_vars` — the phase-351 W5
  carrier, for the reason issue 0460 records: a workspace member's own
  `.cargo/config.toml` is never read (Corrosion runs cargo from the workspace
  root) and `set(ENV{...})` reaches only the configure-time process.

  One thing is different from board facts and it decides the shape. Exactly one
  BOARD is active per configure; several MODELS can be, one per entry. There is
  one runtime staticlib per configure and every entry links it, so entries
  ACCUMULATE: union of the infrastructure flags, max of the application counts,
  and ONE abstaining entry makes the whole configure's count unknown — a shared
  table has to satisfy the largest, and an unknown is not smaller than anything.
  Applied once by `nros_synth_runtime_umbrella`, which already runs after the
  SUBDIRS loop that processes the entries.

  Failure is soft throughout, deliberately, exactly as `nros_resolve_board_facts`
  is: a model that is not resolved yet, a CLI that is not built, an entry
  addressed the `MODEL` way at a path that does not exist. None of those is a
  configuration error; each means "this configure carries no entity facts",
  which is the state every build was in before this wave.

  Verified by a script-mode cmake probe over real models — the union, the
  abstention, a wired model scoring 1 service + 1 action = 4, and the soft skip
  when a model will not load. NOT yet verified end-to-end through a real
  configure; see W5.g.

* **W5.d — consumption.** LANDED. `nros-zpico-build` computes
  `app_declared + PARAM_SERVICE_QUERYABLES + LIFECYCLE_SERVICE_QUERYABLES` and
  sizes the C table and `SERVICE_BUFFERS` from ONE computation. Two sizings from
  one number, not two numbers that must coincidentally agree.

  MEASURED on `examples/native/rust/talker`, `nros-relwithdebinfo`, built twice
  and diffed with `nros-mem-report --baseline`:

  | | before | after | delta |
  | --- | ---: | ---: | ---: |
  | `SERVICE_BUFFERS` | 144,128 | 4,504 | **−139,624** |
  | `g_sessions` | 24,480 | 20,640 | **−3,840** |
  | RAM (.bss + .data) | 365,778 | 222,322 | **−143,456 (−39.2%)** |

  The `g_sessions` line was NOT predicted and is the confirmation that matters:
  the C shim's per-session `stored_queries[N][M]` and `last_reply_seq[N]` are
  sized by the same knob, so one number really does size both sides. A design
  that had left them independent would have moved only the Rust figure.

  The rule is a pure function (`queryable_default_from`) with the environment
  lifted out, because a build script reading env directly is untestable
  in-process and a sizing rule verified by reading is how this phase's other
  defects survived. Seven cases, including both refusals: a malformed count and
  an unknown infrastructure spelling PANIC rather than falling back to
  "undeclared", which is the `.max(1)` shape 0827 measured.

  It introduces TWO deliberate mirrors — `nros-zpico-build` cannot depend on
  `nros-node` to read the constants, nor see its features. `check-infra-queryable-counts`
  holds them to the definitions, which is the entire difference between these and
  the seven prose spellings W5.a replaced. Verified by drifting the lifecycle
  mirror to its historical wrong value of 6 and watching the gate name the file.

* **W5.e — an exhausted table names the right fault.** LANDED, in the half that
  authoritativeness makes urgent. The same exhaustion is now two different
  faults and the message says which. Sized from the backend's own budget, the
  table is too small and the fix is the knob — issue 0460's message, unchanged.
  Sized from a DECLARATION (`ZPICO_QUERYABLE_TABLE_DECLARED`, emitted beside the
  size it describes so the two cannot disagree), it holds what the model said
  this image would create, so reaching the limit means the image created an
  UNDECLARED service server, and pointing that reader at `ZPICO_MAX_QUERYABLES`
  sends them to enlarge a table that was already right. Both arms are `&'static
  str` constants, so the `no_std` half gets the same split — the half issue 0460
  found had no explanation at all.

  NOT landed: the build-time failure for an image that declares nothing. That
  still needs the hand-written-`main` question settled (see Open, below), and it
  is what blocks W5.f's deletion.

* **W5.f — RETIREMENT.** HALF LANDED, and the half that is not is blocked rather
  than skipped.

  LANDED: `ZPICO_MAX_QUERYABLES` stops being an independent opinion and becomes
  a CHECKED override. Two mechanisms deciding one number is how this phase's
  other defects were born. The check compares against a DERIVED floor, not the
  budgeted default, and that distinction is the point: an image carrying both
  service families provably claims eleven slots at boot, so below that it cannot
  start and the build is REFUSED; the default adds `UNDECLARED_HEADROOM` for an
  application count nothing can supply, and being under THAT is reported with a
  `cargo:warning` naming both numbers, because refusing a build for being under
  a guess is the defect this wave exists to remove. The floor and the default
  share one parser of the infrastructure spelling — two readings of one string
  is how a rule and its check come to disagree.

  This is issue 0460 caught a stage earlier: `zephyr/Kconfig` defaults
  `CONFIG_NROS_MAX_QUERYABLES` to 8, and an entry enabling both families needs
  eleven. That image used to build cleanly and die at boot.

  NOT LANDED: deleting `if hosted { 32 } else { 8 }` and the
  `CARGO_CFG_TARGET_OS` sniff behind it. The old path is still REACHABLE, which
  is this wave's own precondition for deletion — a bare `cargo build` of a leaf,
  the standalone `check-rmw-*` projects, `cargo test` from the repo root, and
  every hand-written `main` declare nothing and must still build. Deleting the
  literal before W5.e's build-time failure would not make those declare; it
  would make them fail.

* **W5.g — measure and gate.** MEASURED. This bullet said "NOT MEASURED" for
  six days after the measurement landed; the record is the two dated amendments
  below and issue 0965. The paragraphs that follow are kept as the state at the
  time of writing, because the reasoning about WHY the pure-cargo leaf was the
  wrong subject is still what a reader needs.

  The reason is specific. W5.d's 143,456 B was measured on
  `examples/native/rust/talker`, a standalone pure-cargo leaf with no bringup
  and no model — so nothing declares for it and this wave changes it by zero
  bytes. The images this wave DOES change are the cmake-configured workspaces,
  and those have no committed root `CMakeLists.txt`: the fixture builder
  synthesises it, so measuring means a fixture build rather than a reconfigure.
  That is the next session's first job, not an estimate to write down here.

  What to run: `just build-test-fixtures lane=native`, then `just mem-report
  --json --baseline` on a `workspaces/features` entry (declares
  `param+lifecycle`, so 32 -> 19 slots) and a `workspaces/rust` one (declares
  neither, so 32 -> 8). The gating property is that a role which declares
  services keeps exactly what it declares. Confirm the delivery reached cargo at
  the same time: `NROS_DECLARED_INFRA_QUERYABLES` must appear in the configure's
  `build.ninja`, where today it does not.

  **MEASURED 2026-09-06 — on configured Zephyr images, which existed all
  along.** The recipe's `lane=native` route does not work (below), but its
  QUESTION is answered: `zephyr-workspace/build-cpp-*` are CMake configures
  through the real seam, each carrying `nros/entity_inventory.cmake` from the
  reader. Six report `derived`:

  | image | `MAX_CBS` | `ACTION_CLIENTS` | `MAX_SUBSCRIBERS` | `MAX_PUBLISHERS` |
  | --- | ---: | ---: | ---: | ---: |
  | `cpp-action-client-xrce` | 1 | 1 | 1 | 0 |
  | `cpp-action-server-xrce` | 2 | 1 | 0 | 2 |
  | `cpp-listener-xrce` | 1 | 0 | 1 | 0 |
  | `cpp-service-client-xrce` | 2 | 0 | 0 | 0 |
  | `cpp-service-server-xrce` | 1 | 0 | 0 | 0 |
  | `cpp-talker-xrce` | 1 | 0 | 0 | 1 |

  Against the crate defaults these replace (`MAX_CBS` 4, and the session pools
  at 8) that is the wave's saving on a real cmake workspace image, which is what
  W5.g asked for.

  **The queryable table specifically is NOT in that list, and that is the
  finding.** `NROS_DERIVED_MAX_QUERYABLES` is absent from every one of the six.
  The inventory declines it deliberately: it cannot see whether an image enables
  the parameter or lifecycle service families (6 and 5 queryables), so it emits
  the count only where something else states the infra addend. Measuring the
  "queryable-table saving" therefore has no number to report on these images —
  not because the wave failed, but because the knob is one the derivation
  refuses to guess. Issue 1061 records the same refusal on the cargo side.

  What follows is the earlier attempt, kept because the `lane=native` recipe is
  still wrong and the next person will otherwise re-run it.

  **ATTEMPTED 2026-09-05, and the recipe above does not work as written.** Ran
  it; recording what it produced rather than an estimate, per this phase's own
  rule.

  `just build-test-fixtures lane=native` completes green (after two unrelated
  breaks that had to be fixed first, below), and
  `workspace-fixtures-build.sh linux rust` builds 21 entries — **through cargo,
  producing zero CMake configures**. There is no `build.ninja` to inspect,
  because the native workspace entries are cargo leaves. That also confirms
  issue 0965's statement that "every entry in `examples/workspaces/` targets
  Zephyr or FVP, so there is no host entry to configure": the directories
  `native_entry` and `native_rust_params_entry` exist, which makes the claim
  look wrong, and they are cargo-built, which makes it right.

  So the measurement needs a CMake-configured workspace image, and on this tree
  that means a Zephyr or FVP cross build — not `lane=native`. The instruction
  above should be rewritten for whoever picks this up.

  What IS measurable now, and consistent with the expectation above:
  `NROS_DECLARED_INFRA_QUERYABLES` appears in **0 of 26** existing workspace
  `build.ninja` files, as do `NROS_DERIVED_MAX_QUERYABLES` and
  `NROS_DERIVED_MAX_SUBSCRIBERS`. Those configures are stale, so this is
  consistent-with rather than proof-of; it is not a substitute for the A/B.

  **Two blockers were cleared to get this far**, both unrelated to W5:

  * 24 `build.ninja` still referenced `sertype_min.cpp`, retired on main by
    `6ab5ab7db` (issue 0976). Re-configured 21 in place (`cmake <build-dir>`,
    the documented fix rather than a wipe); the other 3 are synthesised
    workspace dirs with no committed root `CMakeLists.txt`.
  * zenoh-pico did not compile for FreeRTOS + lwIP:
    `_z_ipv6_port_to_endpoint` uses `INET6_ADDRSTRLEN` unconditionally while
    `lwip/inet.h` defines it only under `LWIP_IPV6`. Compile-check units u10
    (`freertos_firmware`) and u11 (`orch_tiers_freertos`) died Error 101. Fixed
    in the fork (`jerry73204/zenoh-pico` `nano-ros`, `49b4e635`) and the pin
    bumped forward.

### Open, and deliberately not assumed away

**Hand-written `main`s.** They create entities at runtime, have no generated
entry, and cannot declare — so under W5.e they cannot build. W2 has the same
problem for the arena and proposes a runtime high-water mark plus a CI lane;
queryables should ride that answer rather than invent a second one. This couples
W5.e to W2's timing.

**Nothing populates the model's service wiring.** `ros-launch-manifest` models
it and the resolver never emits it: zero of 115 models here carry any layer-1
wiring. Until something does, W5.b2's application term is a labelled guess and
the sizing is exact only for the infrastructure half. Two ways out, and they are
not equivalent: teach the resolver to describe endpoints (a `play_launch`
question, and it can only describe what the launch inputs say), or count in code
(the ground truth, and the real end state — see Future work above). This is the
one that decides whether "sized by declaration" ever becomes "sized exactly".

**`ZPICO_MAX_SESSIONS`** multiplies the pool and has no declaration path at all.
Either it joins the model or it stays a knob and this phase says so explicitly.
It is currently 1 everywhere, which is why it has never been the visible term.

### W6 — the executor arena becomes a named static

**LANDED 2026-09-06, on a premise that was half wrong.** Read the correction
first: it changes what the wave did and what it could not do.

#### The premise, verified before building on it

Step 1 as written — *"replace `Executor`'s inline `arena` field with a
reference to a platform-provided static"* — **was already done, five phases
earlier.** phase-271 (issue 0110) moved the executor's sized tables off
build-time consts, and `Executor` has held

```rust
pub(crate) arena: &'s mut [MaybeUninit<u8>],
```

ever since: a slice carved out of caller-supplied backing by
`executor::storage::carve`. phase-403 had already caught the stale claim
(2026-08-31) and listed the sites to fix; nothing had fixed them. RFC-0002 § 4.4's
stable-pointer claim **does still hold** — `Dispatcher` binds the arena at
construction, so the ownership change was confined to construction exactly as
the wave predicted.

So the real defect was one layer down, in WHO SUPPLIES the backing. Surveyed
exhaustively:

| entry path | supplier | placement | visible to `mem-report`? |
| --- | --- | --- | --- |
| C (all 34 in-tree `nros_executor_t` objects) | file-scope `static struct { … } app;` | `.bss` | counted, mis-attributed (issue 1147) |
| C++ / all six cmake entry templates | `Node::GlobalStorageHolder<0>::storage` | `.bss` | counted, mis-attributed (issue 1147) |
| **Rust — every board**: linux, zephyr, freertos, nuttx, threadx, esp32-qemu, mps2-an385, RTIC, bridge codegen, scaffold | `Executor::open`/`open_sized`/`from_session` → `Box::leak` | **heap** | **no — no symbol at all** |
| `heap-free-poc-mps2`, `large-msg-baremetal` | their own `static mut` + `open_in` | `.bss` | yes |

**The arena was invisible because it was on the HEAP, not because it was on a
stack.** Both the campaign's § 1b and RFC-0002 § 4.4b said stack; both were
corrected with this wave, along with `platform-implementation-notes.md`,
`freertos-lan9118-debugging.md`, `book/src/porting/custom-platform.md`,
`nros-node/build.rs`'s comment and — the one that reached users — the RUNTIME
advisory string in `report_arena_headroom`, which told every over-provisioned
image that "the arena is INLINE ON THE TASK STACK".

#### What landed

`nros_node::executor::backing` — a named `.bss` static the `alloc` convenience
constructors serve from, falling back to the old `Box::leak` when they cannot.
One change, every Rust board.

* **Named**: `nros_node::executor::backing::EXECUTOR_BACKING`, sized
  `ExecutorSizing::DEFAULT.u64_len()`.
* **Placeable**: `NROS_EXECUTOR_BACKING_SECTION=<name>` emits
  `#[unsafe(link_section = "…")]` on it (amendment A / issue 0880's `DTCM`
  pattern). The section must be `NOLOAD` — the static is uninitialised — and
  must be reachable by every bus master touching the executor's buffers, which
  TCM on Cortex-M7 typically is not. **Nothing places it yet**; the seam exists
  and is compile-verified, the linker fragment is not written.
* **Declinable**: `NROS_EXECUTOR_BACKING_U64S=0` (build-time env; on Zephyr also
  `CONFIG_NROS_EXECUTOR_BACKING_U64S`, declared by issue 1171 with the `-1`
  DERIVE sentinel as its default rather than the `0` that would have disabled
  the static platform-wide) emits no static at all. A non-zero value overrides
  its size, which is also how a fat entry keeps a static instead of falling back
  to the heap — and how an image that lowers its RTOS allocator arena STATES what
  it is paying back, so the pairing cannot drift (issues 1145, 1171).
* The heap arm still runs, legitimately, in three cases: the opt-out, a SECOND
  executor (tiered boot opens one per tier), and an entry sized past the
  reservation.

**No `// nros-pool:` annotation**, deliberately. The inventory evaluates a pool
as a PRODUCT of knobs at literal defaults; this is a SUM (nine carved tables
plus the arena, with alignment padding) whose largest term is itself derived.
Same reasoning as the two existing deliberate non-annotations. `mem-report`
prices the symbol from the ELF, which needs no formula and cannot drift.

#### Acceptance evidence — MEASURED

Image: `examples/native/rust/talker`, zenoh, built by
`bash scripts/build/fixtures-build.sh linux rust --id native-rust-talker` →
`build/cargo-fixtures/linux/nros-relwithdebinfo/talker`.

```
python3 scripts/nros-mem-report.py build/cargo-fixtures/linux/nros-relwithdebinfo/talker --json
```

| | before | after | delta |
| --- | ---: | ---: | ---: |
| `.bss` | 378,050 | 399,618 | **+21,568** |
| RAM attributed to symbols | 383,112 | 404,673 | +21,561 |
| by-crate `nros_node` | **6** | **21,567** | +21,561 |
| `nros_node::executor::backing::EXECUTOR_BACKING` | *(absent)* | **21,560** | **NEW** |

Every other RAM symbol is **+0**. The new row is 5.1 % of the image's RAM and is
now the fourth-largest symbol in it; before the change the largest `nros_node`
RAM symbol in the whole ELF was 1 byte. `nm` type letter is `b` — genuinely
`.bss`, not `.data`, so it costs no flash.

**The first before/after was thrown away, and saying why is the point.** It
reported `−11,199` bytes. The two builds had run different configurations: the
baseline was built before `nros sync`'s component probe had artifacts to read,
so it took the crate-default pool budgets (`SMALL_PAYLOADS` 32,768) while the
after build took the probed ones (4,096). That is W5's own retracted-claim
failure, reproduced within this wave, and it was caught by checking that the
unrelated symbols were unchanged. The table above is the re-run in which they
are.

**And it RUNS, not merely links.** With the ROS router up
(`nros_router_exec tcp/127.0.0.1:7452`, ROS setup sourced so the paired
`libzenohc.so` loads — issue 0774): session open, node + publisher registered,
`Publishing: 'Hello World: 1..7'`. The corrected advisory appears in that run
too: `arena over-provisioned: set NROS_EXECUTOR_ARENA_SIZE=1024 … 192/8192 bytes
claimed at first spin` — no truncation, no false placement claim. Plus 328
`nros-node` unit tests, which register entities into an arena carved from the
static.

#### What this wave did NOT do

* **Half of the acceptance is not met.** *"One task-stack knob is reduced by a
  measured amount"* did not happen, and the wave's own step 4 named a target
  that does not exist: `APP_TASK_STACK` was deleted in phase-76, the live
  default is `app_stack_bytes` = **384 KiB** (not 64 KB), nothing in the tree
  sets the override, the C/C++ carrier mirrors a third number (512 KiB), and no
  example pins `NROS_EXECUTOR_ARENA_SIZE=8192`. The wave's hypothesis — "the
  stack is carrying the arena" — was already false when it was written, because
  phase-271 had taken the arena out of that frame. Deriving that number needs a
  FreeRTOS image and a stack high-water reading. → **issue 1146**.

  **CLOSED 2026-09-07 by issue 1146, and the wave's remaining premise did not
  survive either.** 14 FreeRTOS QEMU images were built and RUN to a live zenoh
  session with `uxTaskGetStackHighWaterMark` read at the end of the register
  pass. The app task's worst peak is **36 152 bytes** on the Rust path and
  **18 176** on the C path — so the 384 KiB was over-provisioned by 10x, not
  because it carried the arena (it never did) but because nobody had ever
  looked. `Executor::open`, which three documents priced at "over 160 KiB",
  costs **8 952**. The default is 128 KiB now, and every image prints its own
  peak at boot so the number stays derived rather than inherited. What the wave
  DID predict correctly is that the saving is real and is charged per TASK: a
  spawned tier with `stack_bytes = 0` takes the app default too, so a 2-tier
  image stops reserving 768 KiB of the 2 MiB FreeRTOS heap and reserves 256 KiB.
  It is a heap-BUDGET saving, not a `.bss` saving, until `configTOTAL_HEAP_SIZE`
  follows — which is issue 1145's half.
* **Only ONE platform was built and measured: native/linux.** No Zephyr,
  FreeRTOS, NuttX, ThreadX, ESP32 or bare-metal image was built. The change
  reaches them all by construction (one shared constructor), and on an RTOS the
  move is *not* free the way it is on a host: the allocator arena those images
  draw from is itself a fixed static and nothing lowers it, so they would
  reserve the bytes twice. That is why the opt-out knob exists and why the
  pairing is → **issue 1145**.
* **The C/C++ arm is left mis-attributed.** It was never invisible — it is in
  `.bss` — but `mem-report` files `nros::Node::GlobalStorageHolder<0>::storage`
  under the Rust `nros` crate, and plain C's `app` struct under
  `(C / asm / no path)`. → **issue 1147**.

### W6 on Zephyr — MEASURED 2026-09-06, and the pairing is now real

W6's native measurement showed the arena becoming VISIBLE (+21,568 B of `.bss`,
a move rather than a saving). On an RTOS the second half exists, and issue 1145
asked for it: the allocator arena the backing used to come out of is itself a
fixed static, so an image built after W6 without lowering that knob reserves the
bytes twice.

Done for `examples/zephyr/rust/talker` (zenoh), both boards it builds for.
`malloc_arena` is an `nm`-visible symbol, so both halves read off one ELF:

| symbol | before | after |
| --- | ---: | ---: |
| `malloc_arena` (mps2_an385) | 1,048,576 | 961,320 |
| `EXECUTOR_BACKING` (mps2_an385) | 87,256 | 87,256 |
| `EXECUTOR_BACKING` (native_sim/native/64) | 88,328 | 88,328 |

Whole-image RAM on mps2/an385, from west's own report:

    before   RAM: 1789196 B / 4 MB  (42.66%)
    after    RAM: 1701940 B / 4 MB  (40.58%)
    delta         -87,256 B

Exactly the backing size, to the byte — which is the claim, not a coincidence:
the arena was lowered by the smaller of the two boards' measured backings so
neither loses headroom it had before W6.

**It boots and delivers, not merely links.** `native_sim/native/64` published 20
messages in 25 s (identical to the pre-change control), and a stock ROS 2
consumer received them:

    $ ros2 topic echo /chatter std_msgs/msg/String --once
    data: 'Hello World: 9'

So on this leaf W6 is now free rather than costing 87,256 B, and the executor's
storage is priceable for the first time.

**Scope, stated plainly.** ONE leaf, ONE platform. Every other Zephyr Rust leaf
still reserves twice, and FreeRTOS, NuttX, ThreadX and ESP32 are untouched —
1145 asks for one platform per commit precisely because the failure mode is a
runtime allocation failure that a six-platform diff makes unattributable. The
subtrahend is also a HAND-COPIED literal, so an executor knob change makes it
stale with nothing to say so: issue 1171.

**Interaction with amendment B.** Unchanged: this does not decide whether
payload buffers become heap-backed, it makes the question measurable. A `.bss`
arena is the better starting point for that measurement.

**Non-goal, respected.** Sizing each subscription to its type is W3a's; nothing
here touches buffer sizing.

## Explicitly out of scope

**Moving payload buffers to the heap.** REOPENED by amendment B above
(2026-08-29) and no longer this campaign's settled position — read the two
together, and see B for the measurement that decides it. The original reasoning
stands as the case against, unchanged:

It would convert `12 x 4 x 1024` of
always-reserved RAM into peak-of-concurrent, which is a real saving, and it is
declined deliberately. A statically provable buffer would become an allocation
that can fail mid-callback, and it would widen the heap's block-size range from
infrastructure-only (~2^6) to payload-inclusive (~2^16) — which is precisely
what makes [phase 391](phase-391-allocation-unification-and-tier-model.md)'s
constant-time allocator sizeable. The two decisions are coupled; this is the
side of the coupling that keeps both defensible.

## W5.g measured 2026-08-30 — the delivery does not reach any workspace checked

W5.d's 143,456 B was measured on a pure-cargo leaf, which this wave changes by
zero bytes. W5.g was to re-measure on a real cmake workspace image. It did not
get as far as a byte count, because the ENV NEVER ARRIVES.

Method: reconfigure each workspace and look for `nros_entity_facts_env`'s own
status line, `"nano-ros: queryable table sized from the declaration"`. A
configure that applies the env prints it; one that does not, does not.

| workspace | umbrella created | entity-facts line |
| --- | --- | --- |
| `features` (Rust entries) | no | **no** |
| `c` (pure C) | no | **no** |
| `mixed` (C + a Rust node pkg) | YES | **no** |

Two of the three are explained and one is not.

**`c` / `cpp` are by construction.** `nros_synth_runtime_umbrella` returns early
for a pure-C/C++ workspace (`NanoRosRuntimeCrate.cmake:202`, "keep
nros_cpp-static as the umbrella"), and `nros_entity_facts_env` is called AFTER
that return. So the C and C++ workspaces — the ones whose images this campaign
is about — cannot receive the figure through this path at all. That is a design
consequence nobody wrote down, not a bug in the wave.

**`features` has no cmake runtime crate.** Its entries build through cargo
directly, so there is no `nros_ws_runtime-static` to attach env to. The models
that would benefit MOST are here: W5.b1 measured 22 of 115 models as
`param+lifecycle` (32 slots -> 19) and they are `features` models.

**`mixed` is NOT diagnosed, and the previous diagnosis here was wrong.**
Retracted rather than edited, because the reasoning error is the useful part.

It claimed `_NRA_MODEL` is empty for every entry, from this measurement:

```
grep -rl nano_ros_entry examples/workspaces/*/src/*/CMakeLists.txt \
  | xargs grep -lE '^\s*MODEL ' | wc -l   ->  0
```

That number is real and proves only that **no entry passes `MODEL`
literally** — which is expected, since `LAUNCH` and `MODEL` are mutually
exclusive and `LAUNCH` is canonical. The conclusion drawn from it skipped the
ninety lines where `NanoRosEntry.cmake` POPULATES `_NRA_MODEL` from the launch
input:

```cmake
execute_process(
    COMMAND "${_nra_mp_tool}" model-path ${_nra_mp_args}
    OUTPUT_VARIABLE _NRA_MODEL      # ← set here, for every LAUNCH entry
```

So `_NRA_MODEL` is populated, through `nros model-path` — which IS
`model_location`, reached via the CLI. The file even says so: the mapping rule
"lives ONCE in nros_orchestration_ir::model_location … NOT re-implemented here
(the second-spelling drift class)". Calling it a hand-derived path was exactly
backwards.

**What is still unexplained**, narrowed: for `mixed`, either `nros model-path`
fails, or the model file does not exist at the moment
`nros_record_entity_facts` tests `EXISTS`, or `_NANO_ROS_CODEGEN_TOOL` is
undefined at that point in the include order. Those are three checks, not a
theory.

### W5.g DIAGNOSED AND FIXED 2026-09-02 — an ORDERING inversion, not a workspace quirk

`mixed` is explained, and the explanation covers all three rows: the consumer ran
BEFORE any producer, everywhere.

`nros_synth_runtime_umbrella` called `nros_entity_facts_env` inline, under a
comment asserting "the accumulation is complete only after the SUBDIRS loop
above has processed them all". That is true of the NODE packages and false of the
ENTRY, which `nano_ros_add_entry` declares LAST by design — `NanoRosEntry.cmake`
says so in its own words, "the first point in a configure that is guaranteed to
be AFTER every `nano_ros_node_register()`". `nros_record_entity_facts` runs
there.

Measured by tracing a `mixed` reconfigure:

```
line 17:  facts_env(nros_ws_runtime-static) seen=<empty>   <- consumer
lines 19-85: model OK: system_model.yaml, ... (7 models)   <- producers
```

The accumulator was read before anything filled it, on every configure. No
workspace could ever have printed the status line, which is why W5.g found none
in three and could not explain the third.

**Fixed** by deferring to the top-level scope —
`cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}" CALL nros_entity_facts_env
…)` — the same idiom and the same reasoning as
`_nano_ros_support_schedule_flush` one module over, including its warning that
deferring to the CURRENT directory would fire at the end of whichever scope
called first and so be the bug rather than a smaller version of it.

The line now prints on `mixed`:

    nano-ros: queryable table sized from the declaration — infrastructure none,
    application count undeclared (no model here describes wiring)

**Delivery is PARTIAL, and NO SAVING IS MEASURED.** After a full rebuild the
`ZPICO_MAX_QUERYABLES` the units actually compiled are:

| cargo root | value |
| --- | ---: |
| `nros_ws_runtime_*` (the unit that got the env) | **8** |
| `nros_ws_runtime_*` (a second unit) | 32 |
| `nano-ros_*` (repo-root dir, 4 units) | 32 |

So one unit sits at 8 and five at 32. **Whether the 8 is the env's doing is NOT
established** — 8 is also the non-hosted default, and the attempted before/after
build measured the same configuration twice (the revert silently did not apply,
so both halves ran WITH the fix and both reported `SERVICE_BUFFERS` = 36,032).
The causal claim is withdrawn rather than repaired here; what IS verified is that
the status line was absent on every configure before the fix and present after,
which is the ordering defect itself. The env reaches the umbrella's
`zpico-sys` and not the other instantiations, which are separate cargo units
under a different workspace root (issue 0616's shape: a `--target-dir` serves one
root, and `-C metadata` keys a unit by the path it was reached by). Sizing one of
six is not a shipped number, and no byte figure is claimed from it.

**And the application half was absent** — "application count undeclared (no
model here describes wiring)" is issue 0973: `describes_wiring()` was false for
every resolved model in the tree, so only the INFRA figure was ever delivered.

**It is no longer absent, as of phase-412 (2026-09-05).** Endpoint wiring is an
AUTHORED artifact — a `<stem>.contract.yaml` beside the launch file — and
phase-412 wrote the first five, in `examples/workspaces/cpp`. Measured
2026-09-06 on freshly resolved models:

```
$ nros ws entity-facts --model .../service_server.launch.xml model
NROS_DECLARED_INFRA_QUERYABLES=none
NROS_DECLARED_SERVICE_SERVERS=1
$ nros ws entity-facts --model .../action_server.launch.xml model
NROS_DECLARED_SERVICE_SERVERS=3          # one action server is 3 queryables
```

So the application half now reaches a build wherever a contract exists, and
abstains everywhere else — which is the designed behaviour, not the gap. 0973 is
resolved on that basis; W5.f's "retire both headroom constants once every image
declares" is still open, and now has a route rather than a wall.

**So the honest state of W5:** the sizing logic, the exhaustion diagnostic and
the checked override all landed and are unit-tested; the figure they consume now
REACHES a build for the first time, on one of six compiled units, carrying the
infra half only. The 143,456 B in W5.d remains a measurement
of what the mechanism WOULD save, on a leaf where the mechanism does not run.
Nothing in this wave should be quoted as a shipped saving until the `mixed` case
is explained and a before/after `mem-report` exists.

## Issues homed here (survey 2026-09-03)
Every open issue was checked for a home phase; these had none, or were
mentioned here only in passing. A mention is not an owner — an issue with
no work item is an issue nobody is accountable for, which is the same shape
as a gate sitting in a lane no CI job runs. Each row is a work item: the issue
holds the evidence, the item is *close it*.

| issue | why it belongs here |
| --- | --- |
| [#0852](../issues/0852-zephyr-serial-rx-is-polled-and-overruns.md) | the zenoh read task inherits the executor's priority on Zephyr |
| [#0880](../issues/0880-tcm-unused-while-sram-exhausted.md) | 192 KiB of tightly-coupled memory sits at 0 % while SRAM is exhausted |
| [#0969](../issues/0969-cyclone-take-cdr-round-trip.md) | the Cyclone RMW deserializes every received sample and re-serializes it, so `try_recv_raw` costs a full round trip. **Round trip removed; cost measured** — ~46 ns/message floor (176 ns at 16 KB). The allocation saving this row assumed did NOT appear: count unchanged, bytes a crossover at ~6 KB. Remaining: the third site, per 0976 |


## Adopted issue (2026-09-04)

* **[#1028](../issues/archived/1028-nuttx-classified-hosted-takes-linux-queryable-budget.md)**
  — RESOLVED 2026-09-06. NuttX was classified `hosted` because its `target_os`
  is not `"none"`, so `runner.rs` picked the 32-slot queryable budget meant for
  a host. MEASURED on an image with ZERO queryables: `SERVICE_BUFFERS` was
  142,336 B of `.bss` against 35,584 B at the embedded budget — **106,752 B**
  wasted. Harmless on qemu-virt, not harmless on a real part, which is this
  phase's whole subject.

  It arrived as a by-product of an issue-0870 investigation and was filed
  separately rather than buried in a killed lead. That is why it had no phase.

  The predicate is now `target_os_is_hosted()` over an explicit `RTOS_TARGET_OS`
  list (`7c5e52845`, which measured the saving), and the CLASS is swept and
  gated: `nros-rmw-zenoh`'s `effective_client_locator` asked the same question
  with the same wrong predicate, and `check-rtos-target-os` now holds both the
  list (against every triple this tree names) and the `cfg` spellings (against
  the reachable RTOS set).

  **This does not close W5's Open item.** The fallback budget is still a guess;
  1028 only stopped it being the WRONG guess for NuttX. A standalone copy-out
  example reaches none of the three declaration channels — no SystemModel, not
  Zephyr, not a cargo leaf — so the guess is the only budget it can get. That
  gap is
  [#1142](../issues/1142-nuttx-standalone-example-has-no-entity-declaration.md).
