---
id: 1125
title: "`ZPICO_MAX_LARGE_SUBSCRIBERS` is never derived, and `LARGE_PAYLOADS` is the biggest symbol left in the esp32 image"
status: resolved
area: rmw, memory, codegen
severity: medium
related: [0827, 0963, 0896, 1052, 1061, 1122, 0841]
resolved: 2026-09-07
---

## What

Issue 0827's derivation now sizes a leaf's pools from what it declares, and issue
0963's join sizes the payload CLASSES from what an image subscribes to. Neither
answers `ZPICO_MAX_LARGE_SUBSCRIBERS`.

Measured on `examples/native/rust/talker` with the derived budget applied:

| bytes | symbol |
| ---: | --- |
| **131,072** | `nros_rmw_zenoh::shim::subscriber::LARGE_PAYLOADS` |
| 83,784 | `g_sessions` |
| 4,504 | `SERVICE_BUFFERS` |
| 4,096 | `SMALL_PAYLOADS` |

`LARGE_PAYLOADS` is `MAX_LARGE_SUBSCRIBERS × RING_DEPTH × LARGE_SIZE` and is the
single largest symbol in that image — larger than everything 0827 recovered
(176,720 B) leaves behind in the same class.

## Why neither derivation reaches it

The entity inventory counts SUBSCRIPTIONS. It has no notion of a *large*
subscriber, because "large" is not a property of the declaration — it is a
property of the TYPE's bound relative to the class ceiling
(`NROS_MESSAGE_BOUNDS_DEFAULT_SMALL_CEILING`). That is the message-bound
inventory's question, not the entity inventory's.

Issue 0963's join DOES compute `NROS_DERIVED_MAX_LARGE_SUBSCRIBERS` — over the
SUBSCRIBED types, counting entities rather than distinct types so two
subscriptions on one large type reserve two blocks. So the number exists for a
CMake image whose payload classes derive.

What is missing is the cargo-leaf side: `leaf_entity_env` writes the
`[env]` sidecar from the ENTITY inventory alone, and the message-bound inventory
is not part of that path.

## What answering it needs

The leaf's per-type bounds. `nros codegen` already emits the message-bound
fragments (`NROS_MESSAGE_BOUND_<t>_{STATE,RX}`); the question is whether a cargo
leaf can read them the way the CMake path does, and whether the SUBSCRIBED set is
knowable there — issue 1061's manifest declaration carries the type in
`sub:std_msgs/msg/String:/chatter`, so the input may already be present.

## Do NOT guess it

Same rule as `ZPICO_MAX_QUERYABLES` (issue 1061): a pool short of what the image
receives is not a smaller pool, it is a dropped sample or a refused creation. The
crate default is large and safe. A derived number here has to come from the
bounds, or not be written.

## Resolution (2026-09-07)

Fixed. The cargo-leaf lane now derives the payload classes through the channel
issue 1061 built, and the number reaches the crate that reserves the pool.

### What was verified before fixing

Re-measured on this tree rather than taken from the report above, because four
issues this week described work that had already landed.

`examples/native/rust/talker` — one publisher, one timer, **no subscription** —
built `cargo build --release` with `rmw-zenoh`, after `nros sync`:

```
$ /usr/bin/nm -S --size-sort tmp/t1125-before/release/talker | tail -2
00000000000dae6c 0000000000020000 b nros_rmw_zenoh::shim::subscriber::LARGE_PAYLOADS
00000000000fc138 0000000000023300 b nros_rmw_zenoh::shim::subscriber::SERVICE_BUFFERS
```

`0x20000` = **131,072 B**, and the sidecar `nros sync` wrote confirmed the
premise exactly:

```toml
[env]
NROS_EXECUTOR_ACTION_CLIENTS = "0"
NROS_EXECUTOR_MAX_CBS = "1"
NROS_RMW_SUBSCRIBER_SLOTS = "0"
ZPICO_MAX_PUBLISHERS = "1"
ZPICO_MAX_SUBSCRIBERS = "1"
```

Five keys, and no payload-class key among them. The same build dir compiled
`pub const MAX_LARGE_SUBSCRIBERS: usize = 2;` — the crate default — while
`SMALL_PAYLOADS` had already shrunk to 4,096 B off `ZPICO_MAX_SUBSCRIBERS = 1`.
So the entity half of the budget was working and the bound half was absent,
which is what this issue said.

### The fix

`nros_cli_core::leaf_payload_classes` — the cargo-leaf twin of
`_nros_bounds_join_subscribed`. It reads `EntityInventory::subscribed_types()`
(the probe's answer) and joins it against the per-type bounds `nros sync` has
ALREADY written into `<leaf>/generated/<pkg>/nros_message_bounds.json`, using
`rosidl_codegen::bounds::bounds_from_json` — a reader added beside the writer,
so the artifact keeps one parser.

Three keys are appended to the `[env]` sidecar, with the same guards
`_nros_bounds_publish_payload_classes` uses:

| key | when |
| --- | --- |
| `ZPICO_MAX_LARGE_SUBSCRIBERS` | always, when the join derives |
| `NROS_SUBSCRIBER_BUFFER_SIZE` | when something subscribed fits under the split |
| `ZPICO_SUBSCRIBER_LARGE_SIZE` | only when the large count is non-zero |

**All three or none, deliberately.** The runtime routes on
`min(ZPICO_SUBSCRIBER_SIZE_THRESHOLD, NROS_SUBSCRIBER_BUFFER_SIZE)`
(`SMALL_CLASS_CEILING`, issue 0841), not on the threshold alone. Publishing the
large COUNT while leaving the small BLOCK at its 1024-byte default would
classify a 1,500-byte subscribed type "small" here and route it LARGE at
runtime — into a class this derivation had just declared empty, which
`alloc_payload_block` then refuses at `create_subscription`. The small block has
to move with the count for the classification and the routing to be one
predicate. That is why `_nros_bounds_publish_payload_classes` publishes the trio
together, and this module does not publish a subset.

Every failure is a REFUSAL that leaves the crate defaults standing, and the
refusal is written into the sidecar as a comment naming the type: a subscribed
type with no bound, a subscribed type no `generated/` artifact prices, a
malformed or wrong-schema artifact. The defaults are LARGE, not wrong.

**One case this beats the CMake lane on, and it is this leaf's case.** With
nothing subscribed, the answer is 0 and the bound table is never read at all —
so `std_msgs/msg/String` being unbounded in the linked closure cannot refuse it.
The CMake path reaches the same conclusion only through issue 0963's join;
a leaf that publishes an unbounded type and subscribes to nothing gets the
answer here without one.

### Is zero legal?

**Yes, and this issue did not have to re-decide it — phase-403 W4 already ruled
it, with the mechanism stated, and the ruling is re-checked here by
measurement.**

* `packages/rmw/zenoh/nros-rmw-zenoh/build.rs:78` is a bare `env_usize(...)`
  with no `.max(1)`. W4 removed issue 0827's floor on the ground that this knob
  never met its premise.
* `alloc_payload_block` (`shim/subscriber.rs`) tests
  `if idx >= MAX_LARGE_SUBSCRIBERS` **before** subscripting `LARGE_PAYLOADS`, so
  a zero-length pool returns `None` and is never indexed. That is the property
  the other floored knobs lack.
* `LARGEST_PAYLOAD_CLASS` already reads `if MAX_LARGE_SUBSCRIBERS > 0 && ...`
  and falls back to the small block, and `required_rx_bytes` answers from it —
  so a hint no class can hold is refused at `create_subscription`, not dropped
  at the transport.
* It is a **Rust static**, not a fixed C array, so issue 1015's floor
  (`c_array_pool_floor`) does not apply and is deliberately not called on this
  row. `check-c-array-pool-floors --audit` lists 21 knob-sized C arrays and this
  is not one of them — nothing to rule under issue 1131.
* Measured: the `= 0` build links, and `nm` no longer lists the symbol.

### The measurement

Same leaf, same command, same tree; only `nros sync` ran between them.

```sh
cd examples/native/rust/talker
nros sync
cargo build --release            # CARGO_TARGET_DIR=tmp/t1125-{before,after}
python3 scripts/nros-mem-report.py tmp/t1125-<w>/release/talker --json
```

| | before | after | delta |
| --- | ---: | ---: | ---: |
| `section_ram_total` (.bss + .data) | 419,490 | 288,418 | **−131,072** |
| `ram_symbol_total` | 406,377 | 275,297 | −131,080 |
| `text` (`size(1)`) | 854,363 | 854,187 | −176 |
| `data` (`size(1)`) | 25,208 | 25,208 | **0** |

The **control** is a symbol-by-symbol diff of `.bss`, not the totals — W5 had to
withdraw a causal claim for exactly the reason this guards against:

```
$ diff <(nm -S before | awk '$3~/^[bB]$/{print $2,$4}' | sort -k2) \
       <(nm -S after  | awk '$3~/^[bB]$/{print $2,$4}' | sort -k2)
< 0000000000020000 ...nros_rmw_zenoh4shim10subscriber14LARGE_PAYLOADS
< 0000000000000008 ...nros_rmw_zenoh4shim10subscriber18NEXT_LARGE_PAYLOAD
```

Two symbols leave and NOTHING else changes size — `SERVICE_BUFFERS` (144,128),
`g_sessions` (88,120), `EXECUTOR_BACKING` (21,560), `SMALL_PAYLOADS` (4,096) are
byte-identical across the pair, and `data` is unchanged. 131,072 + 8 = 131,080,
which is the `ram_symbol_total` delta exactly. The `.bss` section alone moves
130,880 rather than 131,072 because 192 bytes shift into section alignment; the
`.bss + .data` total is exact.

`nros-mem-report --baseline` shows no row for this: its delta column annotates
symbols present in BOTH files, and the whole point here is that one is gone.

### Gates

* **`check-payload-class-ceiling`** (new, fast lane). The small/large split now
  has THREE spellings — the cmake `NROS_MESSAGE_BOUNDS_DEFAULT_SMALL_CEILING`,
  the Rust `DEFAULT_SMALL_CLASS_CEILING` this fix added, and
  `ZPICO_SUBSCRIBER_SIZE_THRESHOLD`'s crate default, which is what the shim
  actually routes on. cmake cannot read a Rust `const`, so agreement is checked
  rather than made structural. A ceiling above the runtime's is issue 0841 one
  class up, and it is invisible to every other knob gate: each number is derived
  correctly and delivered faithfully, and they disagree about what they MEAN.
  Selftest runs on the normal path; the negative control was also exercised
  live, by perturbing the const and watching the gate go red.
* **`check-knob-delivery`** gained `NROS_DERIVED_MAX_LARGE_SUBSCRIBERS` and
  `NROS_DERIVED_SUBSCRIBER_LARGE_SIZE` — the gap issue 1122 §"3. A gate" names.
  Both resolve under the `ZPICO_`-prefixed spelling, which is why they are
  written from the call sites rather than guessed from the derived name.
* `check-kconfig-knob-forwarding` needs nothing: it holds
  `_nros_resolve_knob(<ENV>)` to a Rust reader, and these knobs go through
  `_nros_resolve_derivable_knob` onto the cargo command
  (`NROS_RESOLVED_KNOBS` -> `_nros_knob_env`), which `nros-rmw-zenoh/build.rs`
  reads as plain env. Verified by reading both ends.

### Siblings swept, and what is left

`LARGE_PAYLOADS`'s two sibling pools in the same shim were checked, not assumed:

* `SMALL_PAYLOADS` = `ZPICO_MAX_SUBSCRIBERS` x `RING_DEPTH` x
  `NROS_SUBSCRIBER_BUFFER_SIZE`. The count was already derived (issue 0827); the
  BLOCK now is too, by this fix.
* `SUBSCRIBER_BUFFERS` is `ZPICO_MAX_SUBSCRIBERS`-sized and was already derived
  (312 B in this image).
* `ZPICO_SUBSCRIBER_LARGE_SIZE` — the third of the trio, now derived.

**Left undone, deliberately:**

* **Issue 1122 is NOT closed by this.** It is the same family on the CMake
  lanes: every `NROS_DERIVED_*` value is published lane-independently and
  consumed only under `zephyr/`, so a FreeRTOS / ThreadX / NuttX / posix image
  still compiles crate defaults. That fix has to cross a lane boundary in
  `cmake/`, and it needs a decision this one does not (a non-Zephyr lane has no
  Kconfig and therefore no spelling for the `-1` opt-in). This issue's leaf lane
  has its own opt-in already: the sidecar is `[env]` without `force`, so any
  value the caller states wins.
* `SERVICE_BUFFERS` is now the largest symbol in this image at 144,128 B, from
  `ZPICO_MAX_QUERYABLES` = 8. That knob is deliberately NOT derived on a cargo
  leaf (issue 1061: the inventory cannot see whether the image enables the
  parameter or lifecycle service families, and a short queryable table is a
  registration failure). Unchanged here.
* `NROS_SUBSCRIPTION_BUFFER_SIZE` (`nros-node`'s take buffer, derived over the
  CLOSURE and aliased by `DEFAULT_TX_BUF`) is not derived on the leaf lane. It
  is a different basis with a different consumer set; narrowing it is the
  under-derivation `NanoRosMessageBounds.cmake` warns about.
* The book and `docs/guides/embedded-tuning.md` document
  `ZPICO_SUBSCRIBER_BUFFER_SIZE`, which **no build script reads** — the name
  moved to `NROS_SUBSCRIBER_BUFFER_SIZE` in phase-403. Found while writing the
  sidecar; not fixed here, filed separately.
