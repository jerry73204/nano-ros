# Phase 408 — a C/C++ subscription sizes its buffer from its own message type

**Status (2026-09-01). W1, W2 and W4 are LANDED on the C pack — by other work,
not by this phase.** Re-measured against the tree rather than read off this
file, which said "no code yet" and sent the next reader looking in the wrong
place. What that reader found:

| wave | state | evidence |
| --- | --- | --- |
| W1 emit the constant | **done (C)** | `packs/c/message.h.jinja` emits `_TX_/_RX_MAX_SERIALIZED_SIZE`, comment cites 0896 |
| W2 retarget publish helper | **done** | bounded types get `_TX_MAX_SERIALIZED_SIZE`; `NROS_PUB_BUFFER_SIZE` survives only as the unbounded fallback, which is the honest answer |
| W3 runtime `RX_BUF` | **done** | landed under phase-403 W3/W5, not here — `create_subscription_raw` takes `rx_bytes` and `add_arena_subscription_c_callback` spends it. See phase-403's "Relationship to phase-408" section, which says so and predates this table |
| W4 deliver at the call site | **done (C)** | `<Type>_subscribe` passes the constant; an unbounded type gets a POISONED macro naming the costing member |
| W5 `_with_info` keeps the hint | **done** | W5a routed it to the backend; W5b (2026-09-01) put the info + validated entries' payload slot in the arena's trailing region, so the hint sizes the allocation too |
| C++ pack | **done (2026-09-01)** | emits the REAL bound (`bounds.rs`, not the estimator) as `TX_/RX_MAX_SERIALIZED_SIZE` + `rx_size_bound<M>`; `bind_subscription` spends it. The trap below is why this took a rewrite rather than a wiring change |

**The C++ pack had a trap in it, and the trap is why the fix was a rewrite.**
Recorded because the number that was already there looked usable and was not: It emits a single
`SERIALIZED_SIZE_MAX` from `types::compute_serialized_size_max`, and that number
must NOT be reused as the receive hint. `bounds.rs` says why, in its own words:

> It is deliberately NOT `crate::types::compute_serialized_size_max` … That
> function ESTIMATES: it charges a flat 512 bytes per nested message and a flat
> default capacity per string, and it always returns a value, so it can never
> report "unbounded". A flat 512 for a nested type whose own bound exceeds 512
> is an UNDER-estimate, which is the direction that matters.

Wiring the existing C++ constant into `rx_buffer_hint` would ship an UNDER-sized
receive buffer — the drop-every-sample failure this whole cluster is about,
reintroduced by reusing a number that was already there. The C++ work is
*emit the real bound from `bounds.rs`*, not *use the number in the header*.

**Two numbers, not one — the distinction W3 turns on.** The C path already
carries a runtime hint to the BACKEND for payload-class routing (phase-402,
`subscription.rs:301`). What is still fixed at `DEFAULT_RX_BUF_SIZE` is the
const that sizes the EXECUTOR ARENA's trailing region. W3 is about the second;
the first is done for one of the three variants.
Scope is the C and C++ path only. The other obstacles to
[phase 392](phase-392-static-memory-space-campaign.md)'s "W3 remainder" — Rust
being opt-in (W3c), Cyclone not consuming the hint (W3f), unbounded diagnostics
and the TX/RX split (W3g) — are deliberately out of scope here and stay where
they are.

Carries [issue 0896](../issues/archived/0896-c-cpp-subscriptions-never-state-a-buffer-hint.md).

## The finding that changes the plan

**phase-392 W3e and issue 0896 both describe a design fork — monomorphise the
const generic per type, or make the arena entry runtime-sized. On the path the
C/C++ subscription actually takes, the second option is already built.**

`add_arena_subscription_c_callback<const RX_BUF: usize>` never stores a
`[u8; RX_BUF]`. It computes

```rust
let (_slot_count, trailing_bytes) = buffered_region_size(qos.depth, RX_BUF);
let (entry_offset, trailing_offset) =
    self.arena_alloc_with_trailing::<SubBufferedRawCEntry>(trailing_bytes)?;
```

and `SubBufferedRawCEntry` holds `buffer: BufferStrategy` — a `TripleBuffer` or
`SpscRing` **initialised over trailing arena bytes at runtime**:

```rust
pub(crate) struct SubBufferedRawCEntry {
    pub(crate) handle: session::RmwSubscriber,
    pub(crate) buffer: BufferStrategy,
    pub(crate) callback: RawSubscriptionCallback,
    pub(crate) context: *mut core::ffi::c_void,
}
```

`RX_BUF` is consumed only as a VALUE — by `buffered_region_size`, and by
`TripleBuffer::init(ptr, RX_BUF)` / `SpscRing::init(ptr, RX_BUF, depth)`. A
const generic carrying a number that is never used in a type is a runtime
parameter that has not been spelled as one.

So this campaign is not "design a mechanism". It is: **produce the number, get
it to the call site, and stop passing it as a const.**

**The distinction is real and worth keeping.** The typed Rust entries
(`SubInfoEntry`, `SubSafetyEntry`) genuinely do hold `buffer: [u8; RX_BUF]`, so
for them the const generic is load-bearing and `rx_buffer_for!` is the right
answer. The buffered entries — which is what every C/C++ subscription and the
depth>1 Rust subscriptions use — are trailing-allocated. One tree, two entry
shapes; a claim about "the arena buffer" that does not say which is not a claim
about anything.

## What already exists, verified

* `nros_cpp_subscription_options_t.rx_buffer_hint` (phase-402) on all three
  register variants, read by `read_subscription_options`, passed to
  `add_arena_subscription_c_callback` at `subscription.rs:301`. **Issue 0896's
  closing statement that there is no slot for a hint was true when filed and is
  not now.**
* The hint currently routes only the BACKEND size class. It does not reach
  `RX_BUF`, so the arena's trailing region is still sized from
  `config::DEFAULT_RX_BUF_SIZE` on every C/C++ subscription in the image.
* `_with_info` discards the hint entirely (`_rx_buffer_hint`).
* `nros_serdes::size::max_serialized_size` computes the bound from `FIELDS`;
  phase 380 provides it as `MAX_SERIALIZED_SIZE_XCDR{1,2}` on the Rust trait.
  Nothing emits it for C or C++.

## What has to be true when this is done

1. A C or C++ subscription to a bounded type sizes its arena trailing region
   from that type, not from `DEFAULT_RX_BUF_SIZE`.
2. The number is computed ONCE, by `max_serialized_size`, and asserted equal to
   the Rust const for the same type. Two implementations of "how big can this
   get" is the class this campaign keeps finding (0088 → 0114 → 0122 → 0123 →
   0245 → 0268).
3. An unbounded type still gets the configured default, and the diagnostic
   names the FIELD. `None` means unbounded, never unknown.
4. The saving is shown by `mem-report --json --baseline` on a named image, not
   asserted from a table.

## Waves

**All of the below are delivered by phase 403 W3/W5 except where noted. They are
kept as written, because the acceptance criteria are the useful part and phase
403's measurement should be readable against them.**


**W1 — emit the constant.** Issue 0896 layers 1–2: one traversal in
`rosidl-codegen` that builds the `nros_serdes::FieldType` value alongside the
existing expression string, so a new variant handled by one output and forgotten
by the other is a compile error. Then
`<PREFIX>_RX_MAX_SERIALIZED_SIZE_XCDR{1,2}` from `packs/c/message.h.jinja` and
the C++ sibling.

*Trap, from 0896:* nested types must be resolved to build a sizeable value, and
resolution can fail. **A failure to resolve must fail the generate**, not emit
"no bound" — otherwise this campaign's own defect returns wearing a different
hat.

Acceptance: emitted constant equals the Rust const for every type in the message
corpus, both encodings, one test.

**W2 — retarget the publish helper.** 0896 layer 3, lands with W1 and is
independent of everything after it. The generated helpers stack
`NROS_PUB_BUFFER_SIZE` (global, default 256, checked against nothing); per-type
is exact and strictly smaller for every type under 256 B.

Acceptance: no generated publish helper references the global.

**W3 — `RX_BUF` becomes a runtime argument on the buffered C path.** The
mechanical half of the finding above. `add_arena_subscription_c_callback` and
its two siblings take `rx_buf: usize` instead of `const RX_BUF: usize`; the
const generic remains only where an entry really is `[u8; N]`.

Acceptance: the three C++ register variants no longer name
`DEFAULT_RX_BUF_SIZE`, and a subscription created with a hint allocates trailing
bytes proportional to it — asserted in the executor tests, which already
construct these entries directly.

**W4 — deliver the number at the call site.** A generated `_subscribe` helper
per message type that fills `rx_buffer_hint` from W1's constant, so the C/C++
consumer gets the derived size without naming a number. This is 0896 layer 4,
now unblocked by phase-402's options struct.

Acceptance: a C example subscribing to a bounded type shows a smaller arena in
`mem-report --baseline` **without its source being edited**, beyond switching to
the generated helper.

**W5 — `_with_info` stops discarding the hint.** ~~Small~~ — it is TWO things,
and only one of them is small. Split after reading the entries.

The plain C path was already migrated to a runtime-sized `BufferStrategy` over
trailing arena bytes. The info and validated paths were NOT — their entries
stored a real `[u8; RX_BUF]` —

```rust
type Entry<const N: usize> = SubBufferedRawInfoCEntry<N>;
buffer: [0u8; RX_BUF],
```

— which is exactly the case W3 says to leave const. So "pass the hint" means two
different jobs on this path:

* **W5a — the hint reaches the BACKEND. LANDED 2026-09-01.** The two executor
  entry points take `rx_buffer_hint` and set `topic.with_rx_buffer_hint(...)`,
  so zenoh-pico's payload size-class routing sees it. The two C++ sites stop
  destructuring it into `_rx_buffer_hint` and dropping it. This is the half the
  size-class cluster is actually about, and it needs no entry change.
* **W5b — the hint sizes the ARENA. LANDED 2026-09-01.**
  `SubBufferedRawInfoCEntry` and `SubBufferedRawSafetyCEntry` are off `[u8; N]`
  and are no longer generic at all; their payload slot is a `TrailingBuf` in the
  arena's trailing region, sized at registration from `rx_buffer_hint`. Both
  registration sites compute `rx_bytes` the way the plain C path does, so
  `RX_BUF` survives only as the stated-nothing fallback and an unhinted
  subscription claims exactly what it always did.

  **Not the `BufferStrategy` W5a predicted, and the prediction was wrong for a
  reason worth keeping.** A triple buffer or ring DECOUPLES the producer's slot
  from the consumer's, and these two entries carry PER-SAMPLE side data beside
  the payload — the wire attachment for the info variant, the integrity status
  for the validated one. That is why both were flat in the first place
  (`SubBufferedRawInfoEntry`'s own doc-comment says so), and it has not changed.
  What made them expensive was the CONST, not the flatness: `RX_BUF` arrives as
  `DEFAULT_RX_BUF_SIZE` at every call site. So the migration moves the bytes out
  to the trailing region and keeps one flat slot, one sample per dispatch —
  which is what the acceptance asked for, since the acceptance is about where
  the bytes come from, not about which strategy manages them. Adopting a
  `BufferStrategy` here would have decoupled the attachment from its message.

  One slot, not `buffered_region_size(depth, ...)`: neither entry has a queue to
  size.

Both numbers matter and they are not the same number — the distinction this
phase's status section already had to make once. As of W5b an info-callback
subscription spends its hint on both.

## Explicitly out of scope

Rust-path defaults (W3c), Cyclone consuming the hint (W3f), unbounded
diagnostics and the TX/RX cap split (W3g). All three remain in phase 392.

**One consequence worth stating for whoever measures this:** the motivating
consumer (the ASI an536 lane) is a **Cyclone** image, and
`grep rx_buffer_hint packages/rmw/cyclonedds/` returns nothing. W3 here shrinks
its arena regardless — that is the executor's own allocation, not the backend's
— but the backend-class routing W1/W4 also feed will show no effect there until
phase-392 W3f exists. Measure the arena, not the backend, when validating this
phase on that lane.

## The C++ half of W1 + W4 LANDED 2026-09-01

The C pack already emitted `<PREFIX>_TX/RX_MAX_SERIALIZED_SIZE`, poisoned them
for an unbounded type, and delivered the RX number through a generated
`{Msg}_subscribe` macro (phase-403 W6/W7). The C++ pack emitted none of it, and
`bind_subscription<M, C, Method>` — the one place a C++ subscription still has
`M` in hand — spent `M::SERIALIZED_SIZE_MAX` on the hint. That constant is
`compute_serialized_size_max`, an ESTIMATE (issue 0964): over 120 stock Humble
types it matched the derived bound **zero** times, and its flat 512-per-nested
charge makes it UNDER-estimate exactly the large types whose receive buffer
matters.

**One derivation, two languages.** `generator::common::derive_message_bound` is
now the only place the classification happens; `generator/msg.rs` (C) and
`generator/cpp.rs` (C++) both call it. Copying the C emitter's inline block into
the C++ one would have been a second implementation of "how big can this type
get" along the LANGUAGE axis — the sizes-header mirror class wearing a new hat —
so the block moved instead of being duplicated. The poison token is built from
the same flat C-style name in both, so one type has ONE token in C and C++.

`generate_cpp_message_package_with_lookup` is the sibling of the C entry point;
the two real call sites (`cargo-nano-ros`, `rosidl-bindgen`) pass the resolver
they already had for the inventory and the type hash, so a cross-package nested
field now gets a real bound in the C++ header instead of `Unresolved`.

**The poison is a class template, not a constant, because C++ evaluates a
`static constexpr` member initializer when the class is DEFINED.** A poisoned
initializer would break every TU that merely includes the header — including one
that only publishes — so an unbounded type states no `TX/RX_MAX_SERIALIZED_SIZE`
at all and carries `tx_size_bound<>` / `rx_size_bound<>` templates whose
`static_assert` fires on INSTANTIATION. Measured output:

```
error: static assertion failed: NROS_UNBOUNDED__p_msg_unbounded__field_data:
       p/Unbounded states no serialized-size bound -- unbounded member: data (string).
note: required from 'constexpr const size_t nros::rx_size_bound<...Unbounded>::value'
```

Type and member, at the point a number was asked for.

**Delivery.** `nros::rx_size_bound<M>::value` (new header `nros/size_bound.hpp`)
is the one spelling every C++ subscribe path uses, and `bind_subscription` now
passes it. It has three arms, and the third is why it is a trait rather than a
direct member read: a generated type WITH a bound gives the derived number; a
generated type WITHOUT one gives the poison; a type with no
`nros_derived_size_bounds` marker at all — an older codegen, or a hand-written
message-shaped struct — keeps the estimate, so out-of-tree stubs still compile.
`bind_subscription_sized<M, C, Method>(node, topic, self, rx_bytes, qos)` is the
escape hatch, the C++ sibling of the generated `{Msg}_subscribe_sized` macro.

**Deliberately NOT done.** `SERIALIZED_SIZE_MAX` stays: `Subscription<M>`,
`Client<Svc>`, `Future<T>`, `Stream<T>`, the four action tiers, `tick_ctx.hpp`
and two example workspaces stack buffers on it, and retiring it is issue 0964's
own decision, not a side effect of this one. It is now documented in the emitted
header as an estimate, and the only path where it still reaches a receive-buffer
hint is the no-marker fallback above. The publish side is untouched (the C
`{Msg}_publish` already stacks the TX bound; the C++ publish path serializes in
Rust, where the exports pack still uses the estimate).

Verified: `cargo test -p rosidl-codegen --test message_size_bound_parity` (the
new gate — every corpus type, both resolvers, both encodings, C++ == C ==
`max_serialized_size`, with a both-arms-reached control), the regenerated
`codegen_golden` corpus, `just check cpp` (including the new
`rx_size_bound / bind_subscription hint` instantiation TU), `just check c`,
`just check cli-tests`, `just check fast`.

**Not measured here.** No `mem-report --baseline` was taken: W3's runtime
`rx_buf` plumbing landed in phase-403 W3/W5, so the bytes this changes are the
DIFFERENCE between the estimate and the derived bound on a real image, and this
worktree has no built image to compare. W4's acceptance ("a smaller arena
without editing the source") is therefore still owed a number.
