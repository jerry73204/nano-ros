# Phase 403 — the type's bound sizes every receive buffer, and third parties can say what they need

**Status (2026-09-04). W1, W4 (one half), W6, W7, W7b, W8, W9 and steps 1-3
are LANDED; W0, W2, W3 and W5 remain.** The 2026-08-30 line said "design,
nothing landed" and outlived that by five days and eleven commits -- the body
below had been recording `LANDED` per wave the whole time, so the phase read as
unstarted to anyone who stopped at the header. Opened because
[phase 402](archived/phase-402-c-subscription-options-struct.md) delivered the PLUMBING
for a per-type receive hint and stopped there: the hint now reaches the backend
and changes nothing's size. Depends on
[issue 0896](../issues/archived/0896-c-cpp-subscriptions-never-state-a-buffer-hint.md)
for the bound itself and overlaps
[issue 0900](../issues/archived/0900-arena-slots-budgeted-at-action-client-worst-case.md)
for the arena.

## There are TWO receive buffers per subscription, and neither is precise

This is the fact the earlier work kept eliding, and it is why "we wired the
hint" did not make anything smaller.

**1. The RUNTIME-OWNED take buffer.** `rmw_vtable.h`'s `take` slot says it
outright — "the payload is bytes and **the caller owns the buffer**, so it needs
the length back". The runtime hands `buf`/`buf_len` to the backend. That buffer
is `RawSubscription<RX_BUF>`'s inline `[u8; RX_BUF]`, or the arena's buffered
region. `RX_BUF` is a CONST GENERIC.

**2. The BACKEND-INTERNAL staging block.** zenoh-pico keeps its own payload
pools and picks between them with `alloc_payload_block(hint)`:

```rust
if rx_buffer_hint > SMALL_CLASS_CEILING { /* LARGE_PAYLOADS */ } else { /* SMALL_PAYLOADS */ }
```

Two statically sized classes. A 68-byte type and a 1000-byte type get the same
small block.

So today: buffer 1 is one global size for every subscription in the image, and
buffer 2 is a two-way choice. The hint improves ROUTING — it stops a 4 KiB type
being silently small-classed and dropped, which is real — and it sizes nothing.

## Why buffer 1 cannot simply take the hint

`RX_BUF` is a const generic, so a runtime value cannot reach it. The Rust
generic path CAN monomorphise per type (`rx_buffer_for!` already computes the
number). The C path cannot: `create_subscription` there is type-erased by
design — RFC-0043 components subscribe raw with the type name as a STRING — so
there is no `M` to monomorphise on.

That is the whole reason this phase exists rather than being a parameter change.

## The C++ component path still has `M`, and that changes W3's scope (2026-08-31)

"The C path cannot monomorphise" is true of `create_subscription_raw`, and it is
NOT true of the path the embedded C++ images actually take. Traced on
mr-canhubk344:

```
NROS_SUBSCRIBE(Msg, method, topic)                         component_node.hpp:632
  -> ComponentNode::create_subscription<M, C, Method>      component_node.hpp:287
    -> bind_subscription<M, C, Method>                     component.hpp:98
      -> create_subscription_raw(node, topic, M::TYPE_NAME, trampoline, self, qos)
```

`M` is a template parameter the whole way down and is erased only in that last
call, in a C++ header, where `M` is still in scope. The generated C++ type
already carries the number:

```cpp
static constexpr const char* TYPE_NAME = "nav_msgs::msg::dds_::Odometry_";
static constexpr size_t SERIALIZED_SIZE_MAX = 1804;
```

Two consequences for the wave plan:

* **W3's lifetime contract is not needed for this path.** `bind_subscription`'s
  own doc says it "registers a RAW subscription (so the executor arena owns it,
  no C++ `Subscription<M>` storage object)". The ARENA owns the buffer, not the
  caller, so there is nothing to hand across FFI and no `'a` to get wrong. The
  change is a runtime `size_t` threaded from `bind_subscription` into
  `create_subscription_raw` and on to the slot allocation. W3 as written -- the
  caller supplies the bytes -- remains the answer for the hand-written C API,
  which genuinely has no `M`.
* **It collapses into W5.** Once a per-subscription byte count reaches the arena,
  sizing the slot from it IS W5. For the C++ component path W3 and W5 are one
  change, and it needs no ABI slot.

### The measurement case, with numbers

The island entry (4 RFC-0043 components, zenoh over serial). Subscribed bounds,
read from the generated headers rather than from wire observation:

| type | `SERIALIZED_SIZE_MAX` |
| --- | ---: |
| `Control` | 2052 |
| `Odometry` | 1804 |
| `OperationModeState` | 572 |
| `VelocityReport` | 549 |
| `SteeringReport` | 527 |
| `RouteState` | 524 |
| `GearCommand` | 524 |

33 handles, 13 of which receive. Today every slot is charged the largest
subscription's buffer, so the correct global value (2052, set by `Control`) gives
`36 * (3 * 2052 + 512) + 2048 = 242096 B` against 77968 B of DTCM. Sizing each
slot from its own type is roughly 47 KiB and fits with room.

That is an 5x reduction on a real image, and it is the before/after this phase
demands. The image is named: `src/zephyr_entry` on `mr_canhubk3/s32k344`, serial
transport, in the simple-autoware-safety-island superproject.

## The third-party contract, which is the part with no answer today

`rmw_subscription_options_t.rx_buffer_hint` is in the ABI and reaches every
backend through the vtable's `create_subscription`. What is missing is both
halves of an actual contract:

* **What is a backend OBLIGED to do with the hint?** The field's doc says a
  "size-classing backend (zenoh-pico) can pick a small/large receive buffer" —
  `can`, not `must`, and nothing says what a backend that ignores it must
  guarantee instead. A third party reading only the header cannot tell whether
  ignoring it is conformant.
* **A backend cannot say what it NEEDS or what it CHOSE.** The flow is
  one-directional. The runtime cannot ask "given this type, how big must my take
  buffer be?", and cannot learn that a backend rounded 68 up to 1024. So the
  runtime sizes buffer 1 blind, which is exactly why it uses a global constant.

Both are ABI questions, and getting them wrong is expensive to undo, so they are
W1 rather than an afterthought.

## Two decisions that shrink this phase (2026-08-31)

**1. The ABI may be broken.** nano-ros is not released. "Adding a mandatory
vtable slot breaks every out-of-tree backend at once" was the stated reason W1's
`required_rx_bytes` had to be OPTIONAL and NULLable, and the reason W3's change
to `nros_subscription_t`'s size looked expensive. Neither is a constraint now.
Shape both on the merits.

**2. Every message type has a derived upper bound, by requirement.** A user MUST
bound it in the `.msg` (`string<=64`) or cap it in the codegen config. An
unbounded type is a BUILD ERROR, not a fallback to a configured default.

The second is the larger of the two, because the fallback is what forced a global
constant to exist at all. Consequences:

* The "unbounded" branch stops being a runtime size question and becomes a
  codegen diagnostic. The C pack already does exactly this -- `message.h.jinja`
  emits an `unbounded_token` so naming the size constant of an unbounded type is
  a deliberate compile error, with `unbounded_reason` naming the member that cost
  the bound. That precedent is the shape to follow, not a new one.
* Phase-380's rule is untouched and still governs: `None` means "no bound
  EXISTS", never "unknown", and the code must never invent a number. Erroring
  honours that rule; substituting a default is what it forbade.
* `DEFAULT_RX_BUF_SIZE` may have no remaining purpose once every type is bounded.
  That is a W5 question, not a W2 one -- the constant is load-bearing in the
  arena derivation and in the C API's `MESSAGE_BUFFER_SIZE` welding.

## The arena already allocates variable-size, which is why W5 is small

`Executor::arena_alloc<T>` is a BUMP allocator (`executor/spin.rs`), with a
`trailing_bytes` variant. Allocation is already per-entry and variable; nothing
about the allocator has to change to give one subscription a different buffer
from another.

So `MAX_CBS * per_entry` is not the shape of the allocations. It is a build-time
ESTIMATE of how large the arena must be, and it is the estimate -- not the
allocator -- that budgets every slot at the worst case. W5 is therefore two
things, neither of them structural:

1. Pass the per-type byte count to the allocation site (see the C++ component
   path section above -- `M` is in scope there).
2. Make the arena SIZE estimate honest. Issue 0900's W1 already landed the
   measurement half: `arena_used()` / `arena_capacity()` plus a first-spin
   advisory naming the value to set. An image can be built once, read its own
   advisory, and pin the number -- so a build-time entity inventory is NOT a
   prerequisite for the mechanism, only for deriving the estimate automatically.

Two stale doc comments claim the arena is inline on the task stack and invisible
to the linker: `executor/arena.rs` (in `report_arena_headroom`) and
`executor/spin.rs` (on `arena_capacity`). Both predate phase-271 and both should
go with this wave; see the W5 note.

> **DONE by phase-392 W6 (2026-09-06), and it was wider than two comments.** The
> `arena.rs` one was not only a doc comment — the claim was in the RUNTIME
> advisory STRING, so every over-provisioned image was told "the arena is INLINE
> ON THE TASK STACK". Corrected there plus `nros-node/build.rs`,
> `nros/src/metadata_mode.rs`, RFC-0002 § 4.4b, phase-392 § 1b,
> `platform-implementation-notes.md`, `freertos-lan9118-debugging.md`,
> `book/src/porting/custom-platform.md`, CLAUDE.md and AGENTS.md.
> W6 also answered the placement question this section left open: the third
> path, which the survey above missed, is the Rust `alloc` convenience
> constructors — they `Box::leak`ed the backing, so on EVERY Rust board it was
> on the HEAP and had no symbol at all. It is a named `.bss` static now
> (`nros_node::executor::backing::EXECUTOR_BACKING`, measured 21,560 B on the
> native talker), so a linker-symbol check for `NROS_ARENA_REQUIRED` now works
> for the Rust path too, not only the C/C++ component path.

## Waves

**W0 -- an unbounded type stops being sizeable (landed 2026-08-31).** The
precondition for every other wave: decision 2 above, made real at the one site
that silently substituted a number. `rx_buffer_for!` expanded to
`DEFAULT_RX_BUF_SIZE` for a type with no bound; it now fails the build, in an
inline `const` block so the refusal does not depend on whether the macro was
written in const-generic argument position or as a plain expression. The
diagnostic names both remedies and points at the codegen diagnostic for the
member. `packages/api/nros/tests/rx_buffer_for.rs::an_unbounded_type_keeps_the_configured_default`
is RENAMED, not deleted, into `an_unbounded_type_is_refused_rather_than_defaulted`,
and the compile-time half is a `compile_fail` doctest with a compiling positive
control beside it.

### W0's blocker: the `cap` escape hatch does not reach the bound

Decision 2 says a user MUST bound a type "in the `.msg` or as a `cap` in
`nros-codegen.toml`". **The second half is not true today**, and until it is, the
rule has only one remedy -- editing a `.msg` you may not own.

The `cap` mechanism EXISTS and works, for STORAGE.
`rosidl-lower/src/config.rs` parses `[fields] "pkg/Msg.field" = N` (or
`{ cap = N, mode = "..." }`) into `CapacityResolver`, and the emitted container
becomes `heapless::String<N>` / `heapless::Vec<T, N>`. Its own module doc says
what it is for: "Only **unbounded** fields consult the resolver."

The bound is computed on a different path that never sees it.
`schema_value::build_schema` lowers IDL `String` -> `SerdeFieldType::String`
and `Sequence` -> `SerdeFieldType::Sequence` unconditionally -- `bound_message`
takes `(owner, msg, version, lookup)` and no resolver -- so
`max_serialized_size` returns `None` however the field is capped. The emitted
Rust `Message::FIELDS` says the same (`generator/common.rs`'s
`render_field_type_expr`).

Proven by the tree's own goldens, not by reading:

* `fingerprint-corpus/nros-codegen.toml` caps `Shapes.text` at 32, and
  `expected/configured/Shapes.h` still says
  `Reason: unbounded member: text (string)` and emits the poison token.
* `diagnostic-msgs/nros-codegen.toml` caps `KeyValue.key` at 32, and the
  committed `key_value.rs` stores `heapless::String<32>` while its `FIELDS` say
  `FieldType::String`.

The C header at `packs/c/message.h.jinja` already prints the advice ("or give it
a `cap` in `nros-codegen.toml`"), so the diagnostic currently sends users to a
knob that will not help them.

Wiring cap -> bound is NOT a mechanical fix and is deliberately not done here.
`Message::FIELDS` is also what backends that build wire-type descriptors at
runtime read (Cyclone dynamic types), and DDS treats `string` and `string<32>`
as different types for type-consistency matching. So "the cap narrows the bound"
is a decision about whether a cap is a claim about OUR STORAGE (true: a longer
sample already fails with `CapacityExceeded`, so a buffer sized to the cap loses
nothing) or a claim about THE WIRE (false, and asserting it could stop a
subscription matching). That belongs to the owner, not to W0.

Until it is settled, three consequences:

1. Codegen cannot be made to REFUSE an unbounded type. It would break every
   in-tree type carrying a `string`, with no remedy short of forking the `.msg`.
2. The remaining fallback in codegen stays: `{Msg}_publish` sizes its stack
   buffer from `NROS_PUB_BUFFER_SIZE` (256) for an unbounded type
   (`packs/c/message.h.jinja`). Poisoning it like its RX sibling is right under
   the rule, and is blocked on the same gap -- the helper is `static inline`, so
   the error would fire for anyone merely INCLUDING the header.
3. The C++ pack is worse than a fallback and should be looked at on its own:
   `types.rs::compute_serialized_size_max` never reports unboundedness at all.
   It ADDS 512 per nested message and 256 per string and always emits
   `SERIALIZED_SIZE_MAX`, which is what every `uint8_t buf[M::SERIALIZED_SIZE_MAX]`
   in `nros-cpp`'s headers is sized from -- including the per-type numbers in
   this document's own measurement table. Correcting it requires changing
   `nros-cpp`, so it is named here rather than attempted.

**W1 — the contract, in the ABI and in prose.** Two additions to
`nros/rmw_vtable.h` + `rmw_entity.h`:

**W1 — the contract, in the ABI and in prose. LANDED 2026-08-31.** Two additions
to `nros/rmw_vtable.h` + `rmw_entity.h`:

1. `rx_buffer_hint`'s doc becomes normative: a backend MUST NOT deliver a sample
   larger than the hint into a caller buffer smaller than it, and MUST report
   the size it settled on. Ignoring the hint stays legal — the guarantee is
   about not lying, not about honouring it.
2. An OPTIONAL vtable slot `required_rx_bytes(type_name, type_hash, hint) ->
   size_t` so a backend can answer "for this type at this hint, a take buffer of
   N is enough". `NULL` means "the hint is the answer", which is what every
   current backend would return. The runtime uses it to size buffer 1 instead of
   guessing.

   Optional and NULLable because `check-rmw-abi-shape` treats a vtable slot as a
   contract: adding a mandatory one breaks every out-of-tree backend at once,
   and the campaign's own rule is that a gap goes in the ledger with an issue id
   rather than being forced.

   **The `-> size_t` return above is what landed as an out-parameter**, because
   the header's own preamble makes it one rule with no exceptions: every slot
   returns `rmw_ret_t`, every ANSWER is an out-parameter, and no slot may
   multiplex a count with a status — gated by `scripts/check-rmw-ret-sign.py`.
   A `size_t` return also has no way to say "I cannot size this type", which is
   a PER-TYPE answer that NULLing the whole slot cannot express.
   The landed slot is

   ```c
   rmw_ret_t (*required_rx_bytes)(const char *type_name,
       const char *type_hash, size_t hint, size_t *out_bytes);
   ```

   appended LAST in `nros_rmw_vtable_t` (slot 75), NULL in every in-tree
   backend, `NROS_RMW_RET_UNSUPPORTED` reserved for "cannot size this type"
   with the same fallback as a NULL slot.

   **The optionality argument above is also retired.** nano-ros is unreleased
   and this ABI may be broken, so "breaks every out-of-tree backend at once" is
   not a reason for anything in this phase. The slot is still OPTIONAL, on
   three arguments that survive without it: a slot cannot be REQUIRED before
   something dispatches it (`check-rmw-required-slots.sh` holds the required set
   equal to the `.expect()`ed set — requiring one nothing calls is issue 0349);
   mandatory does not delete the "no opinion" answer, it relocates it into five
   identical backend bodies and a defaulted `RustBackend` method; and it is slot
   75, while uORB's C++14 positional initialiser stops at slot 17 and cannot
   skip. **W4 should promote it to required in the same commit that adds the
   dispatch site** — that is a registration-check change, not an ABI change, so
   nothing is foreclosed by leaving it optional here.

   **W4's answer (2026-08-31): it stays OPTIONAL, and W4 could not have promoted
   it.** The dispatch site is in the executor and `nros`'s node API, which W3/W5
   own; W4 filled the slot for one backend and added no caller, so there was
   nothing for `check-rmw-required-slots.sh` to hold the required set equal to.
   On the merits the sequencing argument dissolves once a consumer exists and
   the other two survive: cyclonedds and XRCE keep ONE receive buffer, so
   `*out_bytes = hint` is genuinely all they have to say, and slot 75 stays
   unreachable from a positional C++14 initialiser however the check is written.
   A slot most backends must fill with a restatement of the documented default
   is worse than a NULL whose meaning the header pins down.

   **The slot takes no session or node handle, and for zenoh that is CORRECT.**
   Checked against the thing it was suspected of: `alloc_payload_block(hint)`
   reads the hint and build-time constants and nothing else — no session, no
   node, no config the session carries. zenoh's real class choice is therefore
   expressible without a session argument, and the ABI needs no correction on
   this count. A backend whose classes depended on session config could not use
   this slot, and that would be the ABI's problem; no in-tree backend is one.

   **`rx_buffer_hint`'s `0` also changed meaning** and W2/W3 inherit it: every
   message type now has a derived upper bound (`.msg` bound or a
   `nros-codegen.toml` cap; unbounded is a BUILD ERROR), so the runtime always
   has a number. `0` means "this caller stated nothing", never "this type is
   unbounded". The "falls back to a configured default" framing is gone with
   it.

**W2 — buffer 1, Rust path.** `create_subscription::<M>` already knows `M`;
size `RX_BUF` from `M::MAX_SERIALIZED_SIZE_*` instead of the global default.
This is the cheap half and it needs no ABI.

**W3 — buffer 1, C path: decouple the storage.** The const generic cannot take a
runtime number, so the buffer stops living inside the entity:

```rust
pub struct RawSubscriptionRef<'a> {
    handle: RmwSubscriber,
    buffer: &'a mut [u8],   // runtime length
    event_regs: EventRegs,
}
```

The caller supplies the bytes, which is what `{Msg}_subscribe` is already
positioned to do — it knows the type's `RX_MAX_SERIALIZED_SIZE` and currently
passes it only as a hint. Issue 0896 records this as "buffer decoupling",
deferred there, owned here.

**Cost to be honest about:** this introduces a lifetime contract across FFI. A
`static` buffer is fine; a stack array that outlives nothing is a footgun, and
the C API cannot express the lifetime. The mitigation is that the generated
`{Msg}_subscribe` macro owns the declaration, so the common path never writes it
by hand — but a hand-rolled caller can still get it wrong, and that must be said
in the header rather than discovered.

**W4 — buffer 2, exact classes. LANDED 2026-08-31, one half of it.** With W1's
`required_rx_bytes`, a backend can be asked for an exact size rather than
choosing a class. zenoh-pico keeps its pools (a target has no allocator) but the
CLASS BOUNDARIES stop being two arbitrary constants and become the distinct
sizes an image's types actually need.

*What landed.* zenoh-pico answers the slot, and it answers the MINIMUM rather
than the class it rounds to — the tightening is in `rmw_vtable.h` and RFC-0005,
because a backend replying with its class size collapses exactly the distinction
this wave exists to recover. Three cases: `hint == 0` gets the small class
stride (the caller stated nothing and a type is a STRING across this ABI, so
there is no schema to consult, but the ceiling on what `take` can hand back is
still exact); a hint any class can hold gets itself, because zenoh-pico adds
NOTHING to the payload the caller must hold; and a hint past the largest class
gets `NROS_RMW_RET_UNSUPPORTED`.

The slot also stopped being inert without the gate noticing, which it would not
have: `check-rmw-slot-producers.py` scanned `src/vtable.{c,cpp}` and the Rust
adapter and NOT a backend's own table, so zenoh's publisher-loan trio had read
as "no producer" since phase 124.A.4.b. Widened, and the `rx-sizing` inert
family removed with its now-false reason.

*The other half is what the wave text already said it was gated on, and the
survey is now conclusive.* Making the class SIZES derive automatically needs a
per-image list of subscribed types and their bounds. It does not exist and
cannot be assembled today: 0 of the 115 resolved `SystemModel`s in the tree
carry any topic wiring (`entity_facts.rs::describes_wiring` abstains on every
one), and even where wiring exists the model knows a type's NAME and nothing
knows its SIZE — `rosidl-codegen`'s `schema_value::bound_message` computes the
bound but is codegen-local and no model walk calls it. `entity-facts` is the
extension point when someone builds it, the way `ZPICO_MAX_QUERYABLES` was.

*And a second gate, which the wave text did not name.* Even with the inventory,
per-type classes buy a zenoh image nothing until the HINT arrives, and on the
C/C++ path it is still `0` (issue 0896, W3/W5's half). The image the phase names
is a C++ component image, so W4's classes reach it only after W3/W5.

*So what W4 could deliver on buffer 2, it delivered as a declaration rather than
a derivation:* `ZPICO_MAX_LARGE_SUBSCRIBERS = 0` is now legal. Issue 0827's
floor of 1 was justified by "the lookup path indexes the pool unconditionally",
which is true of the other two floored knobs and false of this one —
`alloc_payload_block` bounds-checks before it subscripts. The floor was
reserving `RING_DEPTH x LARGE_SIZE` for a class an image may never route into,
which is the waste 0827 exists to remove, kept alive by 0827's own guard. An
image whose types all fit the small class now says so, and a hint no class can
hold is REFUSED at `create_subscription` instead of being served a block too
small for it — the top end of issue 0841, which 0841's fix left open.

*Measured (2026-08-31), on `examples/native/rust/listener` at
`--release`, which runs `alloc_payload_block` on every subscription and was run
against `examples/native/rust/talker` over a local router to confirm delivery:*

| | `LARGE_PAYLOADS` | `SMALL_PAYLOADS` | `.bss` |
| --- | ---: | ---: | ---: |
| defaults (`ZPICO_MAX_LARGE_SUBSCRIBERS=2`) | 131,072 | 32,768 | 407,170 |
| `ZPICO_MAX_LARGE_SUBSCRIBERS=0` | absent | 32,768 | 276,098 |

131,072 B of `.bss`, exactly the pool. The baseline matches the figures
`static-pool-inventory.md` already publishes for both pools, and the after-column
is a configuration the pre-W4 tree could not BUILD (`env_usize_min` panicked
below the floor), so the numbers cannot have come from an unchanged binary. The
saving is available to any zenoh image that never routes a subscription large;
whether a given image qualifies is a claim its author makes, and getting it
wrong now fails the build's first subscription rather than dropping samples.

**W5 — arena slots.** Issue 0900's remaining half. Once W2/W3 make a
subscription's buffer a known per-type number, an arena slot can be sized from
the entities that will occupy it rather than from `MAX_CBS x ActionClient`.
**The "arena is on the task stack" correction is itself stale (measured
2026-08-31).** 0900 says the arena is inline on the task stack, so `nm` and
`mem-report` cannot see it and `NROS_ARENA_REQUIRED` cannot work. That is not
what the code does any more, and it changes what W5 has to measure.

Placement is CALLER-DETERMINED, and has been since phase-271 (issue 0110) moved
the six sized tables off build-time consts:

* `Executor` holds `arena: &'s mut [MaybeUninit<u8>]` -- a borrowed slice
  (`executor/spin.rs`). Nothing arena-sized is inline in it.
* `ExecutorInlineStorage` (`executor/storage.rs`) DOES hold `backing` inline, and
  the C FFI sizes its `_opaque` from that type. A stack-declared
  `nros_executor_t` therefore does put the arena on the stack -- that is the case
  0900 saw.
* The C++ component entry does NOT take that path. `main` ->
  `ZephyrBoard::run_components` -> `nros::init()` -> `Node::GlobalStorageHolder`,
  whose `static uint8_t storage[NROS_CPP_EXECUTOR_STORAGE_SIZE]` is `.bss`.

Measured on mr-canhubk344 (RFC-0043 components, zenoh, serial): DTCM tracked
ARENA_SIZE one-for-one across a MAX_CBS change of 24 -> 36, +26992 B observed
against +24576 B predicted. The arena was in `.bss` on that image, is
linker-visible, and `NROS_ARENA_REQUIRED` would work for this path.

So W5 needs BOTH: a linker-symbol check for the `.bss` placement (the C/C++
component path, which is where the embedded images are) and a stack probe for the
`ExecutorInlineStorage` placement. Sizing against either one alone reports the
wrong number for half the images. `arena.rs`'s doc comment states the stack case
as though it were the only one and should be fixed with this wave.

Not to be confused with a separate finding on the same board: the C++ init call
chain needs more than 16 KiB of MAIN stack (16384 overflows in `open_in`, 32768
does not) at a CONSTANT arena size of ~51 KiB. That is the depth of the init
chain, not the arena, and the two were briefly conflated during bring-up.

## W6 -- the derived bound must leave codegen (2026-08-31, owner's direction)

Codegen is the right place to DERIVE a bound. It is the wrong place for the
bound to STOP, and today it stops there: the number is emitted as a per-type
constant inside a generated header and nothing downstream can ask for it.

The distinction is the one RFC-0049 already draws for platform config
-- capabilities are facts, knobs are policy:

* A **cap** (`string<=64` in the `.msg`, or a `cap` in the codegen config) is
  POLICY. It is an author's declaration about their interface and belongs with
  the interface.
* A **bound** is a DERIVED FACT. Codegen computes it once, and no later stage
  should re-derive it, re-guess it, or fall back past it.

Every consumer that needs the fact and cannot get it currently invents a
substitute:

| consumer | what it does instead |
| --- | --- |
| arena derivation (`nros-node/build.rs`) | `MAX_CBS * worst case` |
| zenoh payload classes | two hand-set constants, `SUBSCRIBER_BUFFER_SIZE` / `SUBSCRIBER_LARGE_SIZE` |
| `NROS_MAX_LARGE_SUBSCRIBERS` | a human counts which types exceed the ceiling |
| the C API's `MESSAGE_BUFFER_SIZE` | welded equal to `DEFAULT_RX_BUF_SIZE` |

That last row is not hypothetical. Bringing the island up on
mr-canhubk344 required reading `Control 2052` and `Odometry 1804` out of
generated C++ headers BY EYE and copying them into a board `.conf` to set
`NROS_MAX_LARGE_SUBSCRIBERS=2` and `NROS_SUBSCRIBER_LARGE_SIZE=2560`. Then the
arena had to be pinned by guesswork, and the first guess (40960) was too small --
the image failed at `create_subscription` with the arena exhausted. Each of those
is a number the build already knew and could not say. This is the
"a knob nobody can enumerate is a knob nobody sets" failure that issues 0271 and
0739 record, reproduced end to end.

**W6: export the derived bounds as build metadata.** The channel already exists
for the executor's own numbers -- `DEP_NROS_NODE_RX_BUF_SIZE` and friends reach
the C API through Cargo `links` -- plus a manifest for the CMake/Kconfig side
that the Zephyr lane reads. With the inventory available:

* the arena sizes from the entities actually registered, rather than
  `MAX_CBS * worst case`;
* W4's "class boundaries become the distinct sizes an image's types actually
  need" becomes expressible -- it is blocked today precisely on this missing
  inventory, which is why the phase notes W4 is "gated on the same thing issue
  0900 is gated on";
* `NROS_MAX_LARGE_SUBSCRIBERS` and `NROS_SUBSCRIBER_LARGE_SIZE` stop being
  numbers a human reads off a header.

It also answers this phase's standing open question about where an entity
inventory comes from: it is codegen's output, currently discarded.

## What a `cap` may and may not claim (2026-08-31, SUPERSEDED and re-ruled)

> **This section was reversed on the day it was written.** The first ruling read
> the ROS 2 evidence correctly and drew the wrong conclusion from it; the
> reversal is below. Both are kept, because "a cap is storage-only" was
> load-bearing prose for half a day and a reader who finds only the answer
> cannot tell which question it settles.

W0 found that the `cap` in `nros-codegen.toml` reaches the STORAGE container and
not the bound, so the C header's advice ("bound it in the `.msg`, or give it a
`cap`") named a knob that could not help. The question was whether to wire `cap`
into the bound.

**What ROS 2 does**, read from the Humble installation rather than from memory:

```c
/* rosidl_typesupport_introspection_c/message_introspection.h */
uint8_t type_id_;             /* ROS_TYPE_STRING (16), bounded or not */
size_t  string_upper_bound_;  /* the bound, carried separately */
bool    is_upper_bound_;      /* same shape for arrays and sequences */
```

There is no `BOUNDED_STRING` type id: a bound is an ATTRIBUTE of the field, not a
different type. And the bound is not on the wire -- a bounded and an unbounded
string serialize to identical CDR (length, bytes, terminator). Our own generated
`Odometry` shows it: `child_frame_id` is `[u8; 256]` in storage and goes out
through `write_string(fixed_str(..))`, the same encoding an unbounded field uses.

**The first ruling** (superseded) read that as a reason a cap must NOT become the
bound: a `.msg` bound is contractual, a config cap is unilateral, and a
conforming remote publisher never agreed to it.

### The re-ruling: a cap DOES set the bound (owner, 2026-08-31)

The evidence above is unchanged and was not re-litigated. What changed is the
conclusion drawn from it.

A cap cannot change the type name, the encoding, or endpoint matching -- that is
what the Humble read established. So the ONLY exposure a cap creates is a peer
sending more than the cap, and that case is already handled LOUDLY: the runtime
reports it through `report_dropped_take` and counts it in `DROPPED_TAKES`. A cap
is therefore a stated deployment assumption that FAILS LOUDLY, not a silent
truncation, and the first ruling's "silent shortfall" was the wrong name for it.

Against that, "storage-only" left the rule unusable in practice: **86 of 126
stock ROS Humble types have no `.msg` bound** (measured below), across every
package an Autoware-facing image consumes. Requiring the interface to be edited
means editing vendored `.msg` files we do not own, which forks the interface --
strictly worse for interop than a cap that no participant can observe.

| | where it lives | may a bound come from it? |
| --- | --- | --- |
| `.msg` bound (`string<=64`) | the interface, shared by every participant | YES -- and it WINS over any cap, in both directions |
| `inline` cap in our codegen config | our build only, ENFORCED at deserialize | YES -- a stated assumption, enforced and reported |
| `heap` / `view` cap | our build only, enforced NOWHERE | NO |
| built-in 256 / 64 fallback | nobody stated it | NO |

**Only an `inline` cap bounds**, and that is RFC-0033's own "What each mode
GUARANTEES" table rather than a new policy: `inline` is "bounded, statically
provable -- the size is in the type", `heap` is "`alloc::Vec<T>` (cap = hint)",
and `view` is a slice into the receive buffer with "no fixed capacity". Read off
the emitters, not the prose: an `inline` field decodes through
`heapless::String::try_from(s).map_err(|_| DeserError::CapacityExceeded)`, so a
sample above the cap cannot be decoded into the type at all; a `heap` field
decodes through `heap::String::from(s)` and `nros_type_for_field_heap` does not
even take the cap; a `view` field decodes through a bare `reader.read_string()?`
with no length check anywhere. A cap in the latter two is a number nothing
enforces, so promoting it would put a fabricated bound under a receive buffer --
which IS the failure the first ruling named, correctly, just about a different
two thirds of the feature.

**The built-in fallback is not a bound either.** `CapacityResolver::resolve`
always answers, so the naive wiring would have bounded every unbounded string in
the tree at 256 and quietly satisfied W0's rule everywhere -- deleting the rule.
`resolve_configured` / `declared_bound` separate "a config file states this" from
"this is what codegen does when told nothing"; `[defaults]` counts as stated, the
level-6 constant does not.

**A `.msg` bound wins by construction, not by precedence.** A bounded shape has
its own arm in `schema_value::lower` and in `render_field_type_expr` and never
consults the resolver, so a cap can neither widen nor narrow an interface bound.

**One cap is keyed on the DECLARING type, so it is transitive.** One
`"std_msgs/Header.frame_id" = 64` bounds `header` in every message that nests a
`Header`, at any depth and across package boundaries. Verified by test
(`one_cap_on_the_declaring_type_bounds_every_message_that_nests_it`), with a
negative control that a cap keyed on the CONTAINING type does NOT reach a nested
field -- W6 had just fixed a bug in this exact walk where a bare nested reference
resolved against the top-level package, and the same mistake here would have made
the direct case pass while every nested `Header` stayed unbounded.

### Measured: what capping actually buys (2026-08-31)

Over the same 12 stock ROS Humble interface packages W6 measured
(`geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `builtin_interfaces`,
`action_msgs`, `diagnostic_msgs`, `trajectory_msgs`, `shape_msgs`,
`visualization_msgs`, `tf2_msgs`, `stereo_msgs`), 126 message types, XCDR1,
resolving nested types across the whole `/opt/ros/humble/share` tree:

| config | bounded | unbounded |
| --- | --- | --- |
| none (the `.msg` alone) | 40 | 86 |
| ONE line: `"std_msgs/Header.frame_id" = 64` | **60** | 66 |
| per-package `string`/`sequence` caps for the 12, plus 4 per-field overrides | **121** | 5 |

The one-line row is the transitivity claim as a number: `header.frame_id` alone
costs 50 of the 126 types their bound, and capping it once moves `geometry_msgs`
14 -> 25, `sensor_msgs` 3 -> 11 and `std_msgs` 15 -> 16 bounded, with no entry
naming any of those packages.

The 5 that remain unbounded under the full config are all the SAME shape --
`sensor_msgs/JointState.name`, `sensor_msgs/MultiDOFJointState.joint_names`,
`trajectory_msgs/JointTrajectory.joint_names`,
`trajectory_msgs/MultiDOFJointTrajectory.joint_names`,
`visualization_msgs/InteractiveMarkerUpdate.erases` -- a `string[]`, i.e. a
sequence whose ELEMENT is an unbounded string. A config key names a FIELD, and an
element is not one; the emitter spells such an element
`heapless::String<NROS_DEFAULT_STRING_CAPACITY>` from a built-in constant nobody
chose, so claiming a bound from it would be claiming 256 bytes per element that
no config states. **`string[]` therefore cannot be bounded by cap today** -- only
by a `.msg` bound. Open: whether the config gains an element key. **Answered by
W7 below: it does, and the five are bounded.**

### W7 LANDED 2026-08-31 -- a cap has TWO dimensions, because a `.msg` has two

`cap` bounds the field. For a `string[]` that is the sequence LENGTH, and a
length alone bounds nothing -- 16 unbounded strings are still unbounded. The
config now carries the second number in the same entry:

```toml
[fields]
"sensor_msgs/JointState.name" = { cap = 16, element_cap = 32, mode = "inline" }

[defaults]                       # or once, at any level
sequence = { cap = 16, element_cap = 32 }
```

**This is not a new idea, it is the `.msg`'s own.** ROS 2's parser strips the
array suffix and THEN parses the base type, so the two bounds have always been
independent dimensions of one field; ours agrees, and now says so in a test
(`parser::tests::a_string_bound_and_an_array_suffix_are_independent_dimensions`,
all four combinations).

| `.msg` | shape |
| --- | --- |
| `string<=10[<=5]` | `BoundedSequence { element_type: BoundedString(10), max_size: 5 }` |
| `string[<=5]` | `BoundedSequence { element_type: String, max_size: 5 }` |
| `string<=10[5]` | `Array { element_type: BoundedString(10), size: 5 }` |
| `string<=10` | `BoundedString(10)` |

**`element_cap` is lowered by REWRITING the shape, not by threading a number.**
`CapacityResolver::element_capped` returns the field type the `.msg` spelling
would have produced, and every emitter -- the Rust `heapless::String<N>` whose
`try_from` returns `CapacityExceeded`, the C `char[N]` that
`nros_cdr_read_string` sizes with `sizeof`, the C++ `nros::FixedString<N>`, the
schema value, the emitted `Message::FIELDS` -- already handled a bounded
element, because a `.msg` has always been able to state one. So there is one
spelling of "a bounded element", not two that can drift, and the claim is
enforced by the container the rewrite produces rather than asserted. The corpus
pins the identity directly: `Bounded.labels` spells `string<=8[<=4]` in its
`.msg` and `Capped.tags` gets `{ cap = 4, element_cap = 8 }` from
`nros-codegen.toml`, and the two emit the same container in all three languages.

Four rules, all tested:

* **Shape.** Only an array/sequence whose element is an unbounded
  `string`/`wstring` has the dimension. A `[fields]` `element_cap` naming
  anything else is a BUILD ERROR naming the field and what it actually is
  (`GeneratorError::ElementCapShape`), raised from all three language funnels;
  `element_cap` under a `string` LEVEL key is a parse error, since a string has
  no elements whatever field it reaches.
* **The `.msg` wins, per dimension.** `string<=10[]` with `element_cap = 32`
  keeps 10 -- by construction, not by precedence: a bounded element is not an
  unbounded string, so there is no dimension left for the config to name.
* **Only a bounding mode bounds**, `StorageMode::cap_bounds_the_wire`, the same
  rule as `cap` and for the same reason. A fixed array and a `.msg`-bounded
  sequence are not configurable shapes, so their mode is `inline` by
  construction and no `mode` key can turn their element bound off.
* **Its own level chain.** `element_cap` resolves through `[fields]` ->
  `[types]` -> `[packages]` -> `[defaults]` INDEPENDENTLY of `cap`, so naming a
  field to override its length does not silently delete an element default set
  once.

Measured over the same 12 packages, 126 types, resolving nested types across
`/opt/ros/humble/share`:

| config | bounded |
| --- | ---: |
| none (the `.msg` alone) | 40 |
| per-package + per-field caps (the row above) | 121 |
| the same, plus `[defaults] sequence.element_cap = 32` | **126** |

The five, derived (TX = XCDR1, RX = max of the two encodings):

| type | TX | RX |
| --- | ---: | ---: |
| `sensor_msgs/JointState` | 4204 | 4208 |
| `sensor_msgs/MultiDOFJointState` | 12396 | 14704 |
| `trajectory_msgs/JointTrajectory` | 9564 | 9564 |
| `trajectory_msgs/MultiDOFJointTrajectory` | 40156 | 49320 |
| `visualization_msgs/InteractiveMarkerUpdate` | 34158429 | 34234821 |

The last row is issue 0962 made concrete rather than a W7 regression: its
`markers` chain is the five-deep `InteractiveMarker -> controls -> markers ->
points` nesting, and `element_cap` contributes a few hundred bytes to `erases`.
W7 cannot deepen such a chain -- a string is a leaf -- which is pinned by
`an_element_cap_cannot_deepen_a_bounded_sequence_chain`; the observation and why
no diagnostic was added are recorded in 0939.

**Found while landing it: the C sequence-of-strings struct had its dimensions
transposed.** All three emission sites spelled
`{elem} data{elem_suffix}[{cap}]`, so a `string[]` under the built-in 256/64 came
out `char data[256][64]` -- which C reads as 256 slots of `char[64]`, i.e. 256
strings of 63 characters where the config said 64 strings of 255. The total byte
count is identical either way, which is why it survived: the struct was the right
SIZE and the wrong SHAPE, and `nros_cdr_read_string(..., sizeof(data[i]))` then
enforced the sequence cap on the string and no cap at all on the count. Latent
while a bounded element could only come from a `.msg`; W7 makes the two numbers
CLAIMED, so a transposed struct is the inventory disagreeing with the storage in
the same header. Fixed in one shared `c_sequence_struct` helper rather than a
fourth copy of the format string. Moves `expected/{inline,configured}/Shapes.h`
and `Probe.srv.h`.

### W7b LANDED 2026-08-31 -- issue 0962's options 2 and 4, chosen by the owner

W7 made deep bounded chains easy to BUILD from configuration, which is what made
issue 0962 load-bearing rather than hypothetical. Two of its listed options were
chosen; "cap the derived bound" was not, because capping the derived bound IS
substituting a number nobody derived.

**Option 4 -- make the multiplication visible.** `schema_value::sequence_chains`
walks the SAME schema the bound is derived from and reports every NESTED chain of
repeated members, deepest path first, one factor per level. It rides the W6
inventory on all three transports off one model: `sequence_chains` in the JSON,
three PARALLEL `_CHAIN_PATHS` / `_CHAIN_FACTORS` / `_CHAIN_ELEMENTS` cmake lists
(so a consumer `foreach`es them rather than parsing a delimiter), and the same
JSON on the `links` channel. Omitted for a type that nests nothing, which is
almost all of them.

Fixed arrays are factors too: `size_bound` iterates them identically, so a
`Pose[100]` of a type carrying a `BoundedSequence(128)` really does cost 12800
elements. What differs is the REMEDY, and the diagnostic says that in prose
rather than by dropping a factor.

**Option 2 -- a per-type total budget.** `[types."pkg/Msg"] max_serialized = N`,
and `[types.*]` only: `sequence`/`string` at a level are per-field CAPACITIES
that compose down the chain, and a total does not, so a `[defaults]` or
`[packages.*]` budget is a parse error rather than a key that quietly means
something else at each level. It is a CEILING CHECKED AGAINST and never a value
substituted -- a derived total under budget is exported unchanged, which is
phase-380's rule and the one this campaign keeps re-enforcing. Over budget is a
BUILD ERROR, raised from the C header emitter (which derives the number the
`#define` states) and from `BoundInventory::check_budgets`, which each driver
calls once per package so one build names every type that blew its budget.

**Measured on the type that motivated the issue.** `/opt/ros/humble`,
`visualization_msgs`, under the uniform cap of 128 the issue names:

| type | derived RX | worst chain |
| --- | ---: | --- |
| `InteractiveMarkerUpdate` | 19,379,320,485 | `markers.controls.markers.points = 128 x 128 x 128 x 128` |
| `InteractiveMarkerInit` | 19,379,256,985 | same |
| `InteractiveMarker` | 151,400,445 | `controls.markers.points = 128 x 128 x 128` |
| `MarkerArray` | 1,182,217 | `markers.points = 128 x 128` |

19.4 GB from caps of 128, and none of it trips the unbounded build error. "Does
not terminate in any useful sense" now has a number behind it. The diagnostic the
budget produces, and the evidence that a budget-free type is unaffected in every
respect, are in
[issue 0962](../issues/archived/0962-nested-bounded-sequences-cost-the-product-of-their-caps.md).

This does NOT fix the multiplication. `size_bound` still walks a bounded sequence
element by element; what changed is that the number is legible and that a user
can turn it into a build error instead of a runtime surprise.

Also found while measuring: `nros_serdes::size::size_bound` WALKS a bounded
sequence element by element, so a capped sequence nested three deep costs the
PRODUCT of the caps. `visualization_msgs` nests
`InteractiveMarkerInit -> markers -> controls -> markers -> points`, and a
uniform cap of 128 there does not terminate in any useful time. Pre-existing (a
`.msg`-bounded sequence has the same shape) but newly reachable, because a cap is
now a way to create deeply nested bounded sequences. Not fixed here. **W7b above
makes it visible and checkable** -- 19,379,256,985 bytes, measured -- without
fixing the multiplication itself.

### W6 LANDED 2026-08-31 -- and it found that the C++ number was never a bound

One data model (`rosidl_codegen::bounds::BoundInventory`), three transports:

| transport | file / key | consumer |
| --- | --- | --- |
| artifact | `<gen>/nros_message_bounds.json` | anything that reads a file |
| CMake | `<gen>/nros_message_bounds.cmake`, `include()` it | the Zephyr / Kconfig lane |
| Cargo `links` | `DEP_NROS_MSGS_<PKG>_BOUNDS_JSON` | a dependent's `build.rs` |

Every number is derived by `nros_serdes::size::max_serialized_size` -- THE size
rule, the same function `M::MAX_SERIALIZED_SIZE_XCDR*` uses. A type with no
bound carries a state (`unbounded` / `unresolved`) and the member or nested type
that costs it, and sets NO size key on any transport. `BoundState::classify` is
shared with the C header emitter, so the exported fact and the emitted `#define`
cannot drift.

**The C++ pack's `SERIALIZED_SIZE_MAX` is an ESTIMATE, and this phase has been
quoting it as a bound.** `rosidl_codegen::types::compute_serialized_size_max`
charges a flat 512 bytes per nested message and a flat default capacity per
string, and it ALWAYS returns a number -- it has no way to say "unbounded".
Measured over 120 types in 12 stock ROS Humble interface packages
(`geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `builtin_interfaces`,
`action_msgs`, `diagnostic_msgs`, `trajectory_msgs`, `shape_msgs`,
`visualization_msgs`, `tf2_msgs`, `stereo_msgs`):

* **81 of 120** types have NO derived bound at all, and the C++ header states a
  size for every one of them.
* Of the 39 that ARE bounded, the estimate matched the derived bound **zero**
  times: 38 over (`geometry_msgs/Twist` 1028 against 64, a factor of 16) and 1
  under (`std_msgs/Empty` 4 against 8 -- the XCDR2 DHEADER).
* The flat 512 makes under-estimating structural, not accidental: a nested type
  whose own bound exceeds 512 is charged 512. Pinned by
  `bounds::tests::the_cpp_packs_constant_under_estimates_a_large_nested_type`.

So **the `Control 2052` / `Odometry 1804` table earlier in this document is a
table of estimates**, and at least `Odometry` has no bound to estimate:
`std_msgs/Header.frame_id` is an unbounded `string`. The 5x arena figure derived
from those numbers has to be re-taken from the inventory before W5 claims it.
The inventory does NOT export the estimate, in either direction.

Two defects fell out, both fixed here because W6 could not produce real numbers
without them:

* **A bare nested reference lost its package.** `schema_value` resolved a bare
  `Pose` inside `geometry_msgs/PoseWithCovariance` against the TOP-LEVEL
  message's package, so `nav_msgs/msg/Odometry` came back "nested type `Pose`
  could not be resolved". Latent because no caller had a cross-package lookup at
  all. The lowering now threads the declaring package. Moves one golden:
  `fingerprint-corpus/expected/{inline,configured}/Nested.h` now names
  `fingerprint-corpus/Shapes` where it named `Shapes` -- the diagnostic and the
  poison token both gain the package, which is the point.
* **The C drivers had no cross-package search path.** `Unresolved` is a
  search-path problem, not a property of a message (issue 0896), and the C paths
  had a same-package-only lookup while the C++ path had none. Both now use one
  `nested_msg_lookup` over the same interface index the Rust path already uses.
  Consequence to watch: a C type that nests across packages now gets a real
  `TX_MAX_SERIALIZED_SIZE`, so `{Msg}_publish` stacks the exact bound instead of
  the 256-byte `NROS_PUB_BUFFER_SIZE`. Nothing working regresses -- such a type
  could not serialize into 256 bytes and failed at publish -- but the stack
  frame grows where it was previously wrong.

**Still open, and NOT fixed here (needs a decision, not a patch).** This phase
says a user MUST bound a type "in the `.msg` (`string<=64`) or cap it in the
codegen config". Only the first half reaches the bound: `bound_message` reads
the parsed `.msg` and never consults `CapacityResolver`, so a
`nros-codegen.toml` cap does not make a type bounded. That is why stock
`nav_msgs/msg/Odometry` reports `unbounded`. The C++ estimate hid this by
silently substituting `CPP_DEFAULT_STRING_CAPACITY` for every unbounded string
-- which is precisely the invented number phase-380 forbids. Whoever owns "an
unbounded type is a BUILD ERROR" has to decide whether a config cap
participates in the derivation; until then, most real types are honestly
unbounded and the inventory says so.

### W8 LANDED 2026-08-31 (issue 0963) -- the inventory finally has a reader

W6 exported the bound over three transports and W7/W7b made the numbers real.
Nothing read any of them, which is issue 0963: the build knew every size and
could say none of them, so `NROS_MAX_LARGE_SUBSCRIBERS` and
`NROS_SUBSCRIBER_LARGE_SIZE` were still produced by reading generated C++
headers by eye -- headers W6 proved state an ESTIMATE.

`cmake/NanoRosMessageBounds.cmake` is the reader. `nros_find_interfaces()`
composes every fragment the image's interface closure produced and derives the
FOUR knobs a bound inventory can answer, writing the answer and its provenance
to `<build>/nros/message_bound_knobs.cmake`. The Zephyr knob resolver reads
that file, with `-1` as the Kconfig spelling of "nothing here chose a number".

**Scope, stated as a boundary and not as a to-do.** A bound inventory knows
every TYPE'S SIZE and nothing about WHICH ENTITIES AN IMAGE CREATES. So the
four size knobs derive and `NROS_EXECUTOR_MAX_CBS` /
`NROS_EXECUTOR_ARENA_SIZE` / `NROS_MAX_SUBSCRIBERS` / `NROS_MAX_PUBLISHERS`
deliberately do not: a package's type count is not an image's entity count, and
deriving one from the other is the plausible-wrong-number this campaign exists
to remove. W4's survey is still the state of the art -- 0 of 115 resolved
SystemModels carry topic wiring, and the C++ components register in
constructors at runtime.

**Precedence: a derived value is a DEFAULT, never an override.** Highest first:
an environment value, then Kconfig / a board `.conf`, then the derivation, then
the crate's own literal. The last rung is "leave the knob unresolved", the
tri-state `NROS_EXECUTOR_ARENA_SIZE` already used, so the fallback literal
stays written in exactly one place.

**Measured on the reference image** -- the mr-canhubk344 island entry, its own
11 interface packages and 84 types, regenerated from the args files its build
dir already holds, derived with `cmake -P`:

| knob | hand-set on the board | derived | where the hand-set number came from |
| --- | ---: | ---: | --- |
| `NROS_MAX_LARGE_SUBSCRIBERS` | 2 | **0** | `Control 2052` / `Odometry 1804` read off C++ headers |
| `NROS_SUBSCRIBER_LARGE_SIZE` | 2560 | not derived (0 blocks) | rounded up from that 2052 |
| `NROS_SUBSCRIBER_BUFFER_SIZE` | 1024 (never stated; the Kconfig default) | 1496 | nobody chose it |
| `NROS_SUBSCRIPTION_BUFFER_SIZE` | 512 | 1496 | a hand computation that reaches ~718 and then writes 512 |

The derived bounds behind those: `Control` 114 against an estimate of 2052 --
a factor of 18 -- `Odometry` 880 against 1804, `VelocityReport` 108, the rest
21-27. **Nothing in the island's linked closure exceeds the 2048 B class
split**, so the large class is empty and its `2 x 4 x 2560 = 20,480` bytes of
`.bss` are reserved for a class no type can route into.

**And it found a live defect, which is the point of a derivation over an
eye.** `CONFIG_NROS_SUBSCRIPTION_BUFFER_SIZE=512` is smaller than
`nav_msgs/Odometry`, the largest type that board conf itself names as
subscribed: the derived RX bound is 880, and the conf's own prose computes
~718 before writing 512. A sample above the buffer is dropped at `take`
with `BufferTooSmall`, silently on the C++ arena dispatch path -- the exact
failure the conf's neighbouring comment warns about. Not verified on silicon
here; the board is the owner's.

**The over-approximation, measured rather than asserted, and it is the
dominant term.** The derived 1496 is set by `std_msgs/Float64MultiArray`, which
the island links and never receives. Over the ten types it actually subscribes
to the answer is 880 (`nav_msgs/Odometry`). Priced in `.bss` at that image's
own `MAX_SUBSCRIBERS = 12` and `RING_DEPTH = 4`:

| | `SMALL_PAYLOADS` | `LARGE_PAYLOADS` | total |
| --- | ---: | ---: | ---: |
| today, as the board conf stands | 12 x 4 x 1024 = 49,152 | 2 x 4 x 2560 = 20,480 | 69,632 |
| derived over the LINKED closure (1496) | 71,808 | 0 | 71,808 |
| derived over the SUBSCRIBED types (880) | 42,240 | 0 | 42,240 |

So the honest headline is not a saving. Letting all four derive as the closure
stands today costs this image **+2,176 B**: the 20,480 B of large pool it stops
reserving is more than spent on a small class sized by a type it never
receives. The 27,392 B saving is real and is in the third row -- it needs the
ENTITY inventory, which is the same second source `MAX_CBS` and the arena need.
What the derivation buys unconditionally is that the numbers stop being wrong
in the unsafe direction, and that the trade above is visible at configure time
instead of after a flash.

Two further notes on this image specifically. Its `.conf` PINS
`NROS_SUBSCRIPTION_BUFFER_SIZE=512` and `NROS_EXECUTOR_ARENA_SIZE=40960`, so
both keep their stated values and the arena does not move -- the +2,176 above
is the whole effect. And its `.conf` also pins `MAX_LARGE_SUBSCRIBERS=2` /
`SUBSCRIBER_LARGE_SIZE=2560`, so the 20,480 B is only recovered once those two
lines are deleted; a derived value never overrides a stated one.

**The refusal is not hypothetical either.** With the island's shipped
`nros-codegen.toml` -- two `inline` caps -- 60 of its 84 linked types are
bounded and 24 are not, so the derivation REFUSES and every knob keeps its
configured value. It names all 24 and the member that costs each one. Adding a
`[defaults]` string/sequence cap block closes the closure (84 of 84) and is
what produced the table above. That is W0's rule reaching the place it was
always aimed at: an unbounded type is not sizeable, and the build now says so
where the size is chosen rather than only where the type is generated.

**Ordering, stated rather than assumed.** A Zephyr module's CMakeLists is
processed during `find_package(Zephyr)`, so `nros_resolve_knobs()` runs before
the application reaches its own `nros_find_interfaces()`. The knobs file being
read is therefore the one the PREVIOUS configure wrote, and it is registered
with `CMAKE_CONFIGURE_DEPENDS` so ninja re-runs cmake by itself once the
interfaces lane writes different bytes into it. Write-if-changed is
load-bearing rather than tidy: rewriting identical bytes would re-arm that
reconfigure forever. An image whose closure has just changed builds once at its
old sizes, re-configures, and builds again at the derived ones -- never
silently, since the configure says which of the two happened.

Gate: `just check message-bound-knobs` (`tests/cmake-message-bounds-tests.sh`,
38 assertions, `cmake -P` against hand-written fragments -- no build, no
codegen). It pins the derived VALUES, the composition across packages, the
refusal and its negative control, the per-fragment schema check, and the
write-if-changed.

### W9 LANDED 2026-08-31 (issue 0965) -- the ENTITY inventory, and the number the bring-up got wrong

W8's boundary was honest and it was also the end of what a BOUND inventory can
do. It prices a TYPE; three consumers need to know WHICH ENTITIES AN IMAGE
CREATES, and until something said, `NROS_EXECUTOR_MAX_CBS` stayed hand-counted,
the arena stayed six bisections deep, and W4's payload classes derived over the
LINKED closure at a measured cost of 2176 bytes instead of a 27392-byte saving.

**The design fork, and why the choice is not a matter of taste.** Issue 0965
named two producers -- a descriptor emitted from the registration macros, or an
author-stated manifest -- and a hybrid. The macros DO know the entity kind and
the type `M`; that is how W3 got the per-type bound to the arena. But anything
they emit is a LINK-SECTION fact and exists only after linking, while
`NROS_EXECUTOR_MAX_CBS` is a `const` compiled into `nros-node` before a single
component TU is compiled. Emitted evidence can VERIFY a count and can never
SUPPLY one. That is the direction of the build graph, not a gap in the tooling,
so the declaration supplies and the running image verifies -- the hybrid, with
the halves assigned by what each can actually do.

**Where the author states it.** `nano_ros_node_register(... ENTITIES ...)`,
beside `CLASS`, `SHAPE` and `CALLBACK_GROUPS`, travelling the channel that
declaration already travels: `nros-metadata.json`. Each spec is
`<kind>[:<type>[:<name>]]` with an optional `*N` repeat, and the type name is
the `pkg/msg/Name` spelling the bound inventory already keys on, so the two
inventories join without a second naming convention.

**One data model, three transports** -- `nros_cli_core::entity_inventory`,
shaped after `rosidl_codegen::bounds` rather than as a second mechanism: the
canonical JSON, the `include()`able CMake fragment, and `KEY=VALUE` env lines.
The env line IS the cargo transport here, because the knob it feeds is read
from the environment by `nros-node/build.rs` and there is no generated crate to
hang a `links` key on; it is the same carrier `nros ws entity-facts` already
publishes through `corrosion_set_env_vars`.

**A publisher claims NO callback slot, and this is the wave's real finding.**
`MAX_CBS` sizes the executor's callback-entry table. Every registration that
claims an entry calls `Executor::next_entry_slot()`; the 24 sites that do are
subscriptions, timers, services, service clients, action servers, action
clients and guard conditions. `create_publisher` is not one -- on the C++ path
it writes an `RmwPublisher` into caller-owned storage, and on the C path there
is no `nros_executor_add_publisher` to increment `handle_count`.

So the bring-up log's table is wrong in a way nobody could see. It records "33
handles" for the island and sets `MAX_CBS=36` from it; 33 is the ENTITY count
and 14 of those are publishers. The slot demand is 19. (Its per-node columns
are also mis-attributed -- it reads 6 timers and 2 services where the source
has 4 timers, 2 service servers and 2 service clients. The TOTAL happens to
come out right, which is exactly why a hand-count is not evidence.)

Gate: `just check entity-slot-costs`
(`scripts/check-entity-slot-costs.py`). `EntityKind::callback_slots()` is a
MIRROR -- the CLI is a host binary and `nros-node` is `no_std` and built for the
target -- so it is held to the `next_entry_slot()` sites, the same way
`ACTION_SERVER_QUERYABLES` is held to its creation sites. Five self-test
mutations, including "a publisher started claiming a slot" in both the Rust and
the C accounting.

**An under-report cannot be silent, in three layers.**

1. **Composition REFUSES on incomplete data.** One component in the image with
   no `ENTITIES` and nothing is derived for the WHOLE image -- the same rule
   `nros_derive_message_bound_knobs` holds when any type in the closure is
   unbounded, and for the same reason. `ENTITIES NONE` is how a component that
   really creates nothing says so, so ABSENCE always means "nobody said". An
   `ENTITIES` list that is present and empty reads as absent too.
2. **The derived value carries NO headroom**, deliberately. It is exactly the
   declared demand, which makes the running image a checker of its own
   declaration.
3. **A short declaration is a NAMED boot failure.** Registration past the table
   returns `NodeError::ExecutorFull`, which names the knob, and
   `ComponentNode`'s `ok()` flag halts boot naming the failing node. This is
   why `MAX_CBS` is the right FIRST consumer and the arena is not: an
   under-sized arena halts DURING entity creation, before the first spin, which
   is exactly why issue 0900 W1's advisory cannot cover it.

**Precedence, unchanged from W8.** `NROS_EXECUTOR_MAX_CBS` moved from the plain
`_nros_resolve_knob` to `_nros_resolve_derivable_knob` and its Kconfig default
became the `-1` sentinel. Environment > Kconfig / board `.conf` > derived >
crate default. Every image built before this wave declares no entities, so its
inventory refuses, so it falls to rung 4 and the crate default of 4 -- exactly
where it was.

**Measured on the reference image**, the mr-canhubk344 island entry, over the
`nros-metadata.json` its own board configure wrote (`build-board/`, 4 C++
components):

| | source | value |
| --- | --- | ---: |
| hand-set today | `boards/mr_canhubk3_s32k344.conf` | 36 |
| the bring-up log's hand-count | `phase-3-canhubk344-real-silicon.md` | 33 |
| DERIVED, entities declared | `NROS_ENTITY_INVENTORY_ENTITY_TOTAL` | 33 |
| DERIVED, callback slots | `NROS_DERIVED_EXECUTOR_MAX_CBS` | **19** |

Per component, as the fragment records it:

| component | entities | slots |
| --- | ---: | ---: |
| `mrm_handler` | 15 | 10 |
| `stop_mode_operator` | 8 | 4 |
| `mrm_emergency_stop_operator` | 5 | 3 |
| `mrm_comfortable_stop_operator` | 5 | 2 |
| **total** | **33** | **19** |

and by kind: 14 publishers, 11 subscriptions, 4 timers, 2 service servers, 2
service clients, 0 actions, 0 guard conditions.

Run against the SAME metadata with no declarations -- which is the island's
tree as it stands -- the inventory refuses, names all four components, and
publishes no number. That is the before/after: the refusal is the current
state, and 19 is what the declaration buys.

**What 17 slots are worth, and what is NOT claimed.** The arena is
`max_cbs * (3 * SUBSCRIPTION_BUFFER_SIZE + 512) + 2048` when nothing pins it,
so 36 -> 19 is 17 slots of arena the island stops reserving. The island PINS
`CONFIG_NROS_EXECUTOR_ARENA_SIZE=40960`, so that saving is not automatic there
and this wave does not claim it: the two knobs must move together, which is
what W8's own Kconfig help already says. What IS claimed is the 17 slots'
worth of `group_sched_table` (~168 B per slot, phase-409's measurement) and
`entries`, which scale with `MAX_CBS` with no second knob to pin them.

**Not measured on hardware.** The island was not flashed for this wave. The
number is a DEFAULT under a `.conf` that still states 36, so adopting it is a
deliberate act by whoever next brings the board up; the code claim -- that a
publisher claims no slot -- is verified by reading all 24 registration sites
and by the gate, not by a boot.

**What W9 does NOT derive, and why it is not one more patch.** The arena and
the zenoh payload classes need the two inventories JOINED per subscription:
this wave's per-entity list against W8's per-type size. A total taken from
either half alone is the same confident wrong number the campaign exists to
remove, so the join is named as its own work rather than bolted onto either
reader. `NROS_MAX_SUBSCRIBERS` / `NROS_MAX_PUBLISHERS` are a straight read of
`NROS_ENTITY_COUNT_SUBSCRIPTION` / `_PUBLISHER` and were left out only for
scope discipline -- one consumer, wired end to end, was the brief.

Gate: `just check entity-inventory-knobs`
(`tests/cmake-entity-inventory-tests.sh`, 24 assertions, `cmake -P` with a
STUBBED CLI so it needs cmake and no cargo build) plus 19 unit tests in
`nros_cli_core::entity_inventory` and `cmd::entity_inventory`, including the
island case above.

## Relationship to phase-408 (PR #130)

phase-408, "a C/C++ subscription sizes its buffer from its own message type",
was opened independently and reaches the same conclusion this phase's W3/W5
implemented:

> `RX_BUF` is consumed only as a VALUE [...] A const generic carrying a number
> that is never used in a type is a runtime parameter that has not been spelled
> as one. So this campaign is not "design a mechanism". It is: produce the
> number, get it to the call site, and stop passing it as a const.

That is W3/W5 exactly. `add_arena_subscription_c_callback` now spends
`rx_buffer_hint` on the allocation instead of `RX_BUF`, and
`bind_subscription<M, C, Method>` supplies `M::SERIALIZED_SIZE_MAX` at the point
the type is erased. Verified on mr-canhubk344: all 33 entities across four
RFC-0043 components register, where boot previously halted at the first
`create_subscription`.

phase-408's own distinction also holds and is worth keeping: the typed Rust
entries (`SubInfoEntry`, `SubSafetyEntry`) really do hold `buffer: [u8; RX_BUF]`,
so for those the const generic is load-bearing and `rx_buffer_for!` remains the
answer. W2 covers that path; W3/W5 covers the buffered one.

**This is not a competing implementation.** phase-408's document is the better
statement of WHY, written from a design review rather than from a bring-up, and
it names the phase-392 W3 remainder it deliberately excludes. Whoever merges
should treat that doc as the specification and these commits as its delivery --
or close phase-408 as delivered, citing this. The two should not both land as
open work.

**Closed as delivered 2026-08-31**, by phase-408's author, citing this
section. That doc now carries a DELIVERED status pointing here, keeps its
waves as the acceptance criteria this phase's measurement can be read
against, and records the premise it got wrong: "C/C++ has no site that names
the type" is true of the hand-written C API and not of C++, where
`bind_subscription<M, C, Method>` keeps `M` to the last call.

One correction to carry across: phase-408 scopes itself to "the C and C++ path",
and the C++ path turned out NOT to need the ABI work the phase anticipated.
`NROS_SUBSCRIBE` -> `create_subscription<M, C, Method>` -> `bind_subscription`
keeps `M` as a template parameter to the last call, in a header, and the arena
owns the buffer there rather than the caller. The hand-written C API, which
genuinely has no `M`, is the part that still needs a caller-supplied buffer.

## Deriving the arena and the payload classes: the remaining steps

Two inventories now exist and neither derives these on its own. W6/W8 price a
TYPE; W9 lists an image's ENTITIES. The arena and the payload classes both need
the two JOINED per subscription, and a total from either half alone is the same
confident wrong number this campaign keeps removing.

The two are NOT equally blocked, and the difference is worth stating because it
sets the order.

### Step 1 LANDED 2026-09-01 -- the payload classes are derived over the JOIN

A payload class is about ONE sample's size, so it needs "which types are
RECEIVED" times "their bound". W9 supplies the first; W6/W8 supply the second.
`nros_derive_message_bound_knobs()` now takes an `ENTITY_INVENTORY` argument
and `nros_find_interfaces()` passes W9's fragment, so the three payload-class
knobs derive over the image's SUBSCRIPTIONS.

**Measured on the reference island** -- the mr-canhubk344 entry, over the 84
bounded types in the 11 `nros_message_bounds.cmake` fragments its own board
configure wrote, joined against an entity declaration built from the island's
published `docs/topic-contract.md`. Priced in `.bss` at that image's own
`MAX_SUBSCRIBERS = 12` and `RING_DEPTH = 4`:

| basis | small class | SMALL | LARGE | total |
| --- | ---: | ---: | ---: | ---: |
| hand-set today | 1024 | 49152 | 20480 | 69632 |
| derived over the LINKED closure | 1496 | 71808 | 0 | 71808 |
| derived over the SUBSCRIBED set | **880** | **42240** | 0 | **42240** |

**Which type sets the small class, before and after.** Before:
`std_msgs/msg/Float64MultiArray` at 1496 B, which the island links and never
receives. After: `nav_msgs/msg/Odometry` at 880 B, the largest of the ten
topics the contract says it subscribes to. That is the whole finding, and it is
worth 29,568 B against the middle row and 27,392 B against the hand-set one.

Residual, stated: W9's declaration for this image reads `sub*7` on
`mrm_handler` and the published topic contract names six of those seven, so the
measurement above is over TEN subscriptions and not eleven. A missing
subscription can only ADD a type, never remove one, so the answer can only move
UP -- and only if that seventh type exceeds 880 B, which no type in the
island's own closure does except the four `std_msgs` multi-arrays it links from
`geometry_msgs`. Not measured on hardware; the board is the owner's.

**Which kinds count as RECEIVING, and why it is two predicates.**
`EntityKind::receives()` is the semantic set, read off the arena entry types
rather than off the names, which mislead in both directions: a service SERVER
receives requests (`SrvRawEntry` carries a `req_buffer`), a service CLIENT
receives replies (`ServiceClientRawArenaEntry<REPLY_BUF>`), and an action
server and an action client EACH carry three receive buffers
(`GOAL_BUF`/`RESULT_BUF`/`FEEDBACK_BUF`). A publisher is not in it: it
serialises into a per-call stack array, which is a transmit buffer.

`EntityKind::receives_topic_sample()` is narrower -- subscriptions only -- and
it is what the payload classes join on. That is MEASURED, not assumed: the
pools `SMALL_PAYLOADS`/`LARGE_PAYLOADS` are reached through exactly one
allocation, `alloc_payload_block(rx_buffer_hint)`, with exactly one caller, the
`declare_subscriber` path. Including the other receiving kinds would not make
the number safer; it would make it describe a pool those entities never
allocate from. Both sets are published, because the arena (step 3) needs the
wider one and a second derivation of it is how two green tools come to
disagree.

**The take buffer deliberately does NOT join.**
`NROS_SUBSCRIPTION_BUFFER_SIZE` is not subscription-only whatever its name
says: `nros-node/build.rs` turns it into `DEFAULT_RX_BUF_SIZE`, the default
const generic for `RawSubscription`, `RawServiceServer`, `RawServiceClient`,
`ActionServerCore` and `ActionClientCore`, and `executor/types.rs` then defines
`DEFAULT_TX_BUF = DEFAULT_RX_BUF_SIZE`. A type this image only PUBLISHES still
has to fit. Narrowing it to the subscribed set would size a buffer too small,
which is the failure this campaign exists to remove, so it keeps the closure
basis. `NROS_MESSAGE_BOUNDS_BASIS` records which set each number used.

**The type-name spellings DO join, for messages, and cannot for anything
else.** W9 records `pkg/msg/Name`, which is exactly what
`TypeBoundEntry::type_name` is keyed on, so a subscription joins by string
equality with no normalisation. Services and actions do NOT join and the
reason is structural rather than a spelling problem:
`BoundInventory::record_message` is called for `.msg` files and for nothing
else in all four producers, so no `pkg/srv/Name_Request` and no
`pkg/action/Name_Result` has an entry to join against, however well-formed the
declaration is. That is why the join REFUSES on an unpriced type and names it,
and why the refusal message says which of the two causes is likelier. Pricing
the service and action sub-messages is its own work; nothing in step 1 needs
them, because none of those entities allocates from the payload pools.

**Refusal, and the one case that is not one.** With the entity fragment's own
status `derived`, the payload classes are derived from the declaration or not
at all: a subscribed set that did not resolve (a component declared no
`ENTITIES`, or a subscription states no type), or a named type the bound
inventory cannot price, REFUSES all three knobs and each keeps its configured
value. No fall-back to the closure -- that publishes the middle row of the
table above while every status still reads `derived`.

The case that is NOT a refusal is the image that declared NOTHING: no fragment,
or a fragment whose own status is `refused`. That is every image built before
W9, and it keeps W8's closure answer byte for byte with
`NROS_MESSAGE_BOUNDS_BASIS = closure`, a status line saying so, and a paragraph
in the generated file saying what a declaration would buy. Refusing there would
take those images from a derived over-approximation back to the hand-set
numbers W8 exists to replace, which is a regression and not a safety property.
A fragment from a STALE CLI is the same case with a warning, and deliberately
not a `FATAL_ERROR`: the fragment's producer (`nano_ros_entry()`) runs LATER in
the same configure than the reader does, so a fatal would abort before the
stale fragment could ever be rewritten.

**Schema.** `ENTITY_INVENTORY_SCHEMA_VERSION` 1 -> 2. Nothing moved; the
fragment gained `NROS_ENTITY_SUBSCRIBED_TYPES` / `NROS_ENTITY_RECEIVED_TYPES`,
each with its own status, and it still bumps -- a reader that took their
absence for "this image receives nothing" would derive a payload class over an
EMPTY set. Absence has to be distinguishable from zero here for the same reason
`ENTITIES NONE` exists.

**Ordering, same lag as W8's.** The entity fragment is composed by
`nano_ros_entry()`, which runs later in a configure than `nros_find_interfaces()`
does, so the fragment read is the one the PREVIOUS configure wrote. Closed the
same way: `CMAKE_CONFIGURE_DEPENDS` on the fragment plus a write-if-changed
producer, so ninja re-runs cmake by itself. An image whose declaration has just
changed builds once at its old payload classes and again at the derived ones,
never silently -- the status line names the basis.

Gates: `just check message-bound-knobs` (80 assertions, up from 38 -- cases L,
M and N are the join, its refusals and the back-compat),
`just check entity-inventory-knobs` (27, up from 24) and 26 unit tests in
`nros_cli_core::entity_inventory`.

### Step 2 LANDED 2026-09-02 -- QoS depth in the declaration. Blocks the arena.

The arena's per-subscription cost is
`sizeof(entry) + buffered_region_size(depth, bound)`, and that region is
`(depth + 1) * bound + (depth + 1) * 8`. **Depth is a multiplier on the bound**,
measured on this island at ten subscriptions: 86108 bytes at the ROS default
depth 10, 24516 at depth 1.

W9's entity record carries `kind`, `type_name` and `name`. It does NOT carry
depth, so a derived arena today would be wrong by up to 10x.

Defaulting it is the worst option available. The ROS default IS 10, so assuming
it inflates the arena tenfold on an image that states depth 1, and assuming 1
UNDER-sizes an image that took the default -- the unsafe direction. Depth must be
declared, on W9's own principle: the declaration supplies, the running image
verifies.

#### The design (2026-09-03)

**The contract is the single source of truth for SIZING, and the two authoring
modes are both legal.**

| mode | the code writes | the contract says | disagreement |
| --- | --- | --- | --- |
| the impl states QoS | `NROS_SUBSCRIBE(M, m, "t", QoS(1))` | `...@depth=1` | USER ERROR -- detected |
| the contract states QoS | `NROS_SUBSCRIBE(M, m, "t")` | `...@depth=10` | impossible, one number |

Mode 2 needs no new C++ surface: `NROS_SUBSCRIBE` is already variadic, so
omitting the QoS is the natural spelling and the declared depth fills in.

**Detection is COMPILE-TIME, and that is reachable rather than aspirational.**
Every piece already exists:

* `nros::QoS` has `explicit constexpr QoS(int)` and constexpr builders
  (`reliable()`, `best_effort()`), so `::nros::QoS(1).best_effort()` is a
  constant expression;
* `constexpr int depth() const` reads it back at compile time (`qos.hpp:347`);
* generated messages carry `static constexpr const char* TYPE_NAME`;
* topics are string literals at every call site in this tree;
* `NROS_SUBSCRIBE` is used as a STATEMENT, so a `static_assert` may precede the
  call it expands to.

So codegen emits the declared depths as a `constexpr` table and the macro
asserts against it. A mismatch fails the BUILD, naming the topic and both
numbers. Nothing is checked at runtime and nothing costs a byte.

    // generated, per image
    struct NrosDeclaredQoS { const char* type; const char* topic; int depth; };
    inline constexpr NrosDeclaredQoS NROS_DECLARED_QOS[] = { ... };
    constexpr int nros_declared_depth(std::string_view type, std::string_view topic);

`nros_declared_depth` is a linear constexpr search, which C++17 evaluates at
compile time; the table is per image and tens of entries, so the cost is a
compile-time loop nobody notices and no runtime storage at all.

**The C++17 obstacle, stated because it shapes the macro.** There is no
`__VA_OPT__` before C++20, so the macro cannot branch on "a QoS was passed"
with the obvious spelling. It dispatches on ARGUMENT COUNT instead -- the
standard trick, and the reason `NROS_SUBSCRIBE` gains a helper macro rather
than an `if constexpr`.

**Boot-time is the fallback, per call site, not per project.** A topic that is
not a literal (built at runtime, or forwarded through a variable) cannot be
looked up at compile time. Those get the same check at construction with an
explicit error naming the topic and both depths -- the idiom
`NodeError::ExecutorFull` and `NodeTableFull` already use, where the running
image checks its own declaration. The compile-time path is preferred and the
boot-time path is honest about what it costs.

**A subscription that declares no depth and passes no QoS is NOT an error.** It
is an image that has not opted in. The arena then REFUSES to derive rather than
defaulting, exactly as W8 refuses on an unbounded type: 10 inflates tenfold, 1
under-sizes, and the hand-set knob stays until someone declares.

**Syntax.** The spec is parsed `splitn(3, ':')` as `kind:type:name`, so the name
takes the rest and a fourth positional would be ambiguous against a topic. Depth
attaches as a named attribute:

    sub:nav_msgs/msg/Odometry:/localization/kinematic_state@depth=10

Named rather than positional so reliability, history and durability can follow
without another grammar change.

**What this does NOT do.** It does not read QoS out of C++ source. An earlier
sketch had a gate parsing `NROS_SUBSCRIBE` call sites for a QoS literal; that
only ever sees the literal forms and silently passes everything else, which is
the "correct but unreachable" shape this campaign keeps finding. Comparing the
actual `QoS` object against the declared number is exact.

#### What landed (2026-09-03)

The design above, implemented as written. Where it needed a decision it did not
make, the decision and its reason are below.

**Grammar.** `@depth=N` is a named attribute split off the WHOLE spec BEFORE the
`splitn(3, ':')`, not off the last field. Neither a ROS type name nor a ROS
topic name may contain `@` (REP-144), so the split is unambiguous, and an
attribute then attaches to the declaration rather than to whichever positional
field happens to be last. `@depth=0` and an unknown `@attr=` are ERRORS, not
skipped rows -- `@depth=0` because KEEP_LAST(0) holds no sample, so it can only
be a typo for "I did not want to say", which is what OMITTING it means. A depth
on a `timer` or a `guard_condition` is an error too: those carry no QoS.

**Absence.** `EntityDecl::depth` is `Option<u32>`; the JSON row carries no
`depth` key at all when nobody declared; the CMake fragment publishes
`NROS_ENTITY_UNDECLARED_DEPTH_COUNT` beside the list, which is the number a size
consumer must refuse on; and the C++ side spells it
`nros::DECLARED_DEPTH_UNDECLARED == -1`, never 0. Schema bumped 2 -> 3.

**Delivery is PER COMPONENT, which the design did not specify.** The knobs are
composed image-wide by `nano_ros_entry()`, which runs after every register call
and therefore after each component library's include path is already set; a
table written there would reach a component's TUs one configure late, so the
check would silently not run on the configure that changed the declaration. And
the table is keyed `(type, topic)`, which two components in one image may
legitimately share at different depths. So `nano_ros_node_register()` renders
its own component's rows -- `nros ws entity-inventory --component <pkg>::<name>
--output-header` -- into `<binary-dir>/nros-declared-qos/<name>/nros/` and puts
that dir on the target's PRIVATE include path. One renderer, one grammar, no
parsing in cmake.

**The header is an X-MACRO**, not a C++ array: pure preprocessor, so it can be
included in any order and `nros/declared_qos.hpp` owns the one definition of the
table's type. Each row is emitted under BOTH spellings of the type, the ROS
`pkg/msg/Name` the declaration used and the DDS-mangled
`pkg::msg::dds_::Name_` a generated class carries as `TYPE_NAME`. The mangling
mirrors `packs/cpp/message.hpp.jinja` and is held to it by a test, because a
mangling change would leave every `static_assert` in the tree keyed on a name no
message class carries -- vacuously true, and green.

**The macro dispatches on argument count** through `_NROS_SUB_PICK`, as the
design said it would have to. Three arguments select `_NROS_SUB_3`, which fills
the declared depth in; four select `_NROS_SUB_4`, which asserts. Fewer names
`_NROS_SUB_TOO_FEW`, whose `static_assert` is about the call rather than about
an undeclared identifier three expansions away.

**The diagnostic needs TWO assertions, and that is a C++17 tax rather than a
choice.** A `static_assert` message must be a string LITERAL, so it can carry
the topic (via `#topic`) and can NOT interpolate two `constexpr int`s; template
ARGUMENTS are printed by both compilers, so `declared_depth_agrees<1, 10>`
carries the numbers and can NOT carry a string. So the macro emits the assertion
for the topic and a `sizeof` that instantiates the template for the numbers.
gcc prints all three facts plus `note: '(1 == 10)' evaluates to false`; clang
prints `requirement '1 == 10'` and the instantiation. Splitting it is not
elegant; a check that names neither the topic nor the numbers is one nobody can
act on.

**Boot-time is per CALL SITE and it is also unconditional.**
`ComponentNode::create_subscription` and `create_subscription_in` run the same
comparison against the same table through the same `nros::declared_depth`, and a
disagreement goes through `set_error` -- so it halts boot naming the node, in
the `NodeError::ExecutorFull` idiom -- after printing the topic and both depths.
It covers the two shapes no macro can: a topic that is not a constant
expression, and a caller that reaches `create_subscription<M, C, Method>`
directly. `NROS_SUBSCRIBE_DYNAMIC` is the explicit spelling for the first, so
the count of call sites that gave up the compile-time check stays visible.

**Proof that the check FAILS.** `packages/api/nros-cpp/tests/compile/
declared_qos_depth_probe.cpp` declares `@depth=1` and passes `nros::QoS(10)`;
`just check cpp` compiles it EXPECTING failure and greps the diagnostic for the
topic, `declared_depth_agrees<1, 10>` and `nano_ros_node_register`. A rejection
for the wrong reason -- a typo, a missing include -- would otherwise read as a
pass, and the positive TU beside it is compiled clean FIRST for the same reason.
A `check-declared-qos-header` gate is the DELIVERY half, and it does not exist
yet -- this paragraph named it as though it did, which is why
`check-doc-recipe-refs` was red on main. What it would do: drive
`_nros_emit_declared_qos_header()` in a real configure with the real CLI and
assert the header lands, the dir is on the include path, and the dir is PRIVATE
-- because a table that never arrives disables every assertion silently and
looks exactly like a component whose depths all agree.

**Not done here.** Nothing sizes from depth yet; that is step 3. The publish
side is not in the table -- keyed `(type, topic)`, a publisher and a
subscription on one pair would be two rows with one key, and nothing consults a
publisher's depth today.

### Step 3 LANDED 2026-09-03 -- the arena. Last, because its failure cannot report itself.

With depth present the arena is a straight sum: `arena_alloc` is a BUMP
allocator, so the total IS the sum of the allocations. Two things are still
needed:

* **A probe per arena entry kind.** There are ten distinct entry types
  (`SubBufferedRawCEntry`, `SrvRawEntry`, `TimerEntry`,
  `ServiceClientRawArenaEntry`, the action entries, ...), each with its own size
  and buffer terms. The existing size probe exports `RAW_SUBSCRIPTION_SIZE` and
  friends, but those are the L1 POLLING handles, not these arena entries.
* **The verification half**, which already exists: issue 0900 W1 landed
  `arena_used()` and a first-spin advisory reporting what was actually claimed.

**Why the arena is last.** An under-sized arena halts DURING entity creation --
before the first spin -- so the advisory never prints. `MAX_CBS` was the right
first consumer for the opposite reason: it fails AT registration with
`ExecutorFull`, which names the knob. A derived arena is the one number whose
failure mode cannot report itself, so it should land only once the inputs
feeding it are themselves verified.

## Measurement, first not last

Every wave here claims bytes, and this campaign has twice published a number
that turned out to describe a path the mechanism did not run on (phase-392 W5.d
measured on a pure-cargo leaf; W5.g then found the figure reaches no workspace
checked). So:

**No wave in this phase is done until a before/after exists on an image that
actually runs the changed code**, and the doc names which image. Compile-tier
green is not evidence of a saving — phase 402 is explicitly marked that way.

## Explicitly not in this phase

The publish side. `{Msg}_publish` already stacks the type's exact TX bound; that
half of issue 0896 is finished and needs nothing here.
