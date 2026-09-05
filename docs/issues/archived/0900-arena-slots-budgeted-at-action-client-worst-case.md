---
id: 900
title: "Every executor arena slot is budgeted at the ActionClient worst case, so a pub/sub-only image carries ~56 KiB it cannot use"
status: resolved
area: core, memory
severity: medium
found: 2026-08-29
related: [0896, 0271, 0739, phase-392, phase-392-W2]
---

# The arena is sized for the entity an image does not have

## What was measured

`ARENA_SIZE` is **74,240 bytes on every generated config in the tree**, without
variation:

| image | `MAX_CBS` | `DEFAULT_RX_BUF_SIZE` | `ARENA_SIZE` |
| --- | ---: | ---: | ---: |
| `threadx-linux/rust/talker` | 4 | 1024 | 74,240 |
| `workspaces/c` (xrce) | 4 | 1024 | 74,240 |
| `workspaces/realtime-c` (nuttx riscv32imac) | 4 | 1024 | 74,240 |
| `workspaces/realtime-cpp` (nuttx riscv32imac) | 4 | 1024 | 74,240 |

The first row is a TALKER. It publishes on a timer and owns no action client,
no action server, and no service. It carries the same arena as everything else,
on a 32-bit embedded target as on a host.

## Why

`nros-node/build.rs:90-135` derives the arena by budgeting EVERY slot at the
largest entity that could occupy one:

```rust
const ACTION_CLIENT_PER_SERVICE:   usize = 4096 + 384;
const ACTION_CLIENT_SERVICES:      usize = 3;
const ACTION_CLIENT_FEEDBACK_SUBS: usize = 3;
const ACTION_CLIENT_SUB_OVERHEAD:  usize = 1536;
const ARENA_BASE_OVERHEAD:         usize = 2048;

let per_entry = ACTION_CLIENT_SERVICES * ACTION_CLIENT_PER_SERVICE
              + ACTION_CLIENT_FEEDBACK_SUBS * rx_buf_size
              + ACTION_CLIENT_SUB_OVERHEAD;
let derived_arena = (max_cbs * per_entry + ARENA_BASE_OVERHEAD).max(ARENA_FLOOR);
```

At the defaults: `per_entry` = 14,976 + 3·1024 = 18,048, and
`4 × 18,048 + 2,048` = **74,240**. The build script says so itself:

> Subscription / service entries are strictly smaller, so budget every slot at
> the action-client size.
>
> Embedded targets that never instantiate an `ActionClient` can override the
> derived size with `NROS_EXECUTOR_ARENA_SIZE`. A pub/sub-only workload only
> needs `3 × rx_buf + 512` per entry.

Taking that note at its word, a pub/sub-only image needs
`4 × (3·1024 + 512) + 2,048` = **16,384** bytes. The difference is
**57,856 B (~56.5 KiB)** carried by every image with no action client — which
is most of them.

## Two things make this worse than a loose default

**The rx buffer is amplified 12x.** `rx_buf_size` enters `per_entry` THREE
times (goal/result/feedback), and `per_entry` is multiplied by `MAX_CBS`. So
`NROS_SUBSCRIPTION_BUFFER_SIZE` is charged `3 × MAX_CBS` = 12x into the arena at
the defaults, on top of its per-subscription cost. Anyone raising that knob to
fit one large message type pays for it twelve times over here. That coupling is
also why issue 0896's per-type receive sizing cannot help this number: the arena
slot is sized from the GLOBAL knob regardless of what any individual
subscription needs.

**The escape hatch is a knob nobody can find.** `NROS_EXECUTOR_ARENA_SIZE`
exists, is honoured, and has a Kconfig sentinel — but nothing computes a right
value for an image, nothing warns when the derived one is 4x what the image
uses, and the correct replacement (`3 × rx_buf + 512` per entry) appears only in
a build-script comment. That is the shape of issues 0271 / 0739: "a knob nobody
can enumerate is a knob nobody sets", which cost ~145 KB in one image.

## Where the bytes land: THE STACK, not `.bss` (measured)

The first draft of this issue guessed `.bss` via the C API's
`nros_executor_t._opaque`. `just mem-report` on
`examples/workspaces/c/build/posix-zenoh-native/cmake/native_entry` refutes it —
there is **no arena symbol in the image at all**. Its RAM is the zenoh pools
(`SERVICE_BUFFERS` 144,128 / `LARGE_PAYLOADS` 131,072 / `SMALL_PAYLOADS` 32,768)
and nothing resembling 74,240 appears.

(That run also printed a STALE-IMAGE banner, so its byte counts describe an
older tree and are quoted here only as shape. The ABSENCE of an arena symbol is
structural, not a staleness artifact.)

The reason is written down one lane over, in
`docs/reference/platform-implementation-notes.md:143`:

> Stack overflow -> "Invalid mbox": `Executor` has an inline
> `arena: [MaybeUninit<u8>; ARENA_SIZE]` **on the task stack**. Action examples
> use `NROS_EXECUTOR_ARENA_SIZE=8192`; `APP_TASK_STACK` must be 16384 words
> (64 KB) for headroom.

The board path — which is what every example uses — builds the executor through
`Executor::open_sized` (`nros-board-linux/src/lib.rs:325`) with the arena inline
in the value, so it lives wherever that value lives: a task stack. The C API's
`_opaque` route is the L1 polling path, a different and less-travelled one.

This makes the defect worse than a static-RAM overshoot in two ways:

* **Stack is the scarcest resource on an RTOS target**, and a 74,240-byte frame
  is not something a per-task stack absorbs quietly. It is charged to whichever
  task calls spin, and it interacts with issue 0667's finding that a task's
  `stack_bytes` is a floor the port raises rather than a number the caller
  controls.
* **The workaround is already load-bearing in the tree.** FreeRTOS action
  examples pin `NROS_EXECUTOR_ARENA_SIZE=8192` — a 9x reduction from the
  derived value — and STILL need a 64 KB app task stack. That override is
  evidence the derivation does not describe real images: someone already had to
  discover the right number by hitting "Invalid mbox" and working backwards.

So the saving is not (only) ~56 KiB of static RAM; it is stack headroom on every
image that spins, and the difference between an image that boots and one that
dies in an allocator with an unrelated-looking error.

**Still measure rather than derive.** `mem-report` reads `.bss`/`.data` and
therefore cannot see this at all — sizing the change needs a stack-usage probe
(worst-case frame at the spin call, or a high-water mark on a running RTOS
image), not a symbol table.

## Measured on a live executor: 32 bytes of 74,240

The derivation's error is not the ~4.5x this issue first estimated from the
pub/sub-only formula. Opening a real `Executor` and registering one timer
(`nros-tests/tests/component_dispatch.rs::executor_arena_is_over_provisioned_for_a_timer_only_image`)
claims **32 bytes of 74,240** — a factor of **2,320**. The arena is a BUMP
allocator (`Executor::arena_alloc` charges `size_of::<T>()` and reserves
nothing per slot), so that number is exact, not a lower bound.

That reframes the fix: the allocator has always known the right answer and had
no way to say it.

## Landed: the executor can now say what it uses (W1)

* `Executor::arena_used()` / `Executor::arena_capacity()` — public accessors,
  ledgered as extensions (upstream has no arena to ask about; rclcpp
  heap-allocates each entity).
* A one-shot advisory on the first `spin_once` when the arena is grossly
  over-provisioned, naming the `NROS_EXECUTOR_ARENA_SIZE` value to set. Emitted
  through `nros_log`, never stdio (issue 0589), from a process-scoped static
  rather than an `Executor` field — the same reason `DROPPED_TAKES` is a static:
  a field would move `EXECUTOR_OPAQUE_U64S` and every image's executor footprint
  to buy a diagnostic. Process scope is also correct here rather than merely
  cheap, since the value it names is a build-time constant identical for every
  executor in the image.
* The threshold is a separate `const fn arena_is_over_provisioned` so it is
  testable — the reporter is one-shot, so a test calling it twice would silently
  check nothing the second time. Four unit tests cover the boundary, a
  well-sized arena staying quiet, overflow, and the issue-0460 zero-capacity
  sentinel (a fault, NOT headroom — calling it over-provisioned would bury a
  fatal misconfiguration under an advisory about wasted space).

This does not shrink anything by itself. It converts the folklore into a number
a user can read, which is the precondition for the rest.

**Now verified, and it found a second defect.** The advisory does fire and does
reach a sink — the earlier silence was `nros_log` holding records in its `early`
ring because a bare `cargo test` installs no sink, then replaying them on
`init`. Nothing was lost.

But the first version of the line was ~450 bytes against `nros_log`'s 256-byte
call-site format buffer (`buffer-size-256`, the default), which truncates with a
`…`. The sink received:

```
executor arena is 74240 bytes and 32…
```

Every word of the explanation and NONE of the value to set — a diagnostic cut
before its actionable half, which is worse than no diagnostic because it reads
as though it helped. **An embedded log line has a hard budget, so a message that
explains itself at length delivers exactly the folklore it was written to
replace.** The line now leads with `NROS_EXECUTOR_ARENA_SIZE=<n>`, keeps the
reasoning in a code comment and here, and the test asserts the message contains
no `…` rather than only checking its wording.

`tests/executor_arena_advisory.rs` is its own test binary: the flag is
process-scoped, so any other spinning test in the same binary would consume it
first, and tests run in parallel — sharing would make it pass or fail by
scheduling. The one-shot contract is asserted by spinning twice in that same
test for the same reason (a separate test would observe an already-consumed
flag and assert nothing, the vacuous shape `check-no-vacuous-tests` exists to
catch).

## Landed: the derivation is per-kind (W2)

`NROS_EXECUTOR_ACTION_CLIENTS` — how many of the `MAX_CBS` slots the arena
budgets at ActionClient size rather than pub/sub size:

```rust
let derived_arena = (action_clients * action_client_entry
    + max_cbs.saturating_sub(action_clients) * pubsub_entry
    + ARENA_BASE_OVERHEAD).max(ARENA_FLOOR);
```

Measured, by building both ways:

| `NROS_EXECUTOR_ACTION_CLIENTS` | `ARENA_SIZE` |
| --- | ---: |
| unset (defaults to `MAX_CBS` = 4) | 74,240 |
| `0` | **16,384** |

**The default is byte-identical to the old formula**, so no existing image
moves — gated by `the_default_derivation_is_unchanged`, which recomputes the
historical arithmetic and compares. That is the point of the wave: the knob
exists so an image CAN shrink its arena, not so every image silently does, and
a change that moves the default moves every image's task-stack frame.

A COUNT rather than a "which entity is heaviest" enum, because Kconfig knobs
are ints and `knob_usize` is the one spelling that reaches the Zephyr Rust lane
(issue 0460). An enum would need a second reader shape for no gain. Forwarded
in `nros_cargo_build.cmake` and declared in `zephyr/Kconfig`, so it reaches the
Zephyr Rust lane rather than being silently defaulted; `check-kconfig-knob-
forwarding` covers it, and `check-pool-inventory` caught the new knob and
required it be enumerated — which is the gate for exactly the 0271/0739 failure
this issue keeps citing.

Lowering it too far fails at REGISTRATION, not at link, so `arena_alloc`'s
`BufferTooSmall` path now names both knobs and the numbers involved
(`report_arena_exhausted`, one-shot, `nros_log`, inside the 256-byte budget).
`BufferTooSmall` is returned by a dozen other paths, so without this, arena
exhaustion is indistinguishable from a message that did not fit a receive
buffer on a target where a return code is all you get.

Together with W1 the loop closes: build once at the defaults, read the
first-spin advisory, set the knob, and be told plainly if it was set too low.

**Not yet done:** nothing sets it automatically. Deriving `action_clients` from
a declared entity inventory is still the real fix, and still blocked on the
inventory existing — see below.

## Landed: the knob is now checked against the LINKED image (W3, option 3)

`scripts/check-action-client-arena-budget.py`, on the fast line as
`just check action-client-arena-budget`. An image that never links the
action-client creation path cannot have an action client. That is a post-link
fact, so it cannot SIZE a compile-time constant — but it can GATE one:

* **knob > 0, nothing heavy linked** — the image carries arena it cannot use.
  **ADVISORY.** It is the shipped state of nearly everything (the knob defaults
  to `MAX_CBS`, and nothing has ever set it), so failing on it would produce a
  uniformly red lane, which has no signal capacity at all: a regression landing
  in it would look exactly like yesterday's red.
* **knob == 0, an action entity IS linked** — registration fails at RUNTIME with
  `NodeError::BufferTooSmall`, on a target where a return code is all you get.
  **FAILS.**

**This is the only half of the fix that reaches embedded images.** The
sidecar/metadata half cannot: a cross-compiled component is unprobeable by
construction (`metadata_build.rs`'s own test is named
`build_std_with_a_foreign_target_is_unprobeable`). `nm` reads a cross ELF
perfectly well, which is exactly where a 56.5 KiB stack frame decides whether an
image boots.

### The obvious symbol was the wrong one, and would have passed vacuously

`ActionClientCore::new` fails twice over. It is **inlined away** — it appears in
NO image in the tree, action clients included — and `ActionClientCore` as a TYPE
appears in **every** image that links the executor: 6 symbols (drop glue + five
methods) in `qemu-rtic-talker`, a pub/sub-only image, byte-identical in count to
`qemu-rtic-action-client`. `spin.rs` names the default-sized monomorphisation
directly, so the dispatch code is linked whether or not anything creates a
client. A probe on the type name would have reported "has an action client" for
the whole tree and never fired.

The discriminator is the CREATION path. Measured over 58 built ELFs on
2026-09-01 (20 Cortex-M `thumbv7m-none-eabi`, 38 x86_64), `create_action_client*`
and `create_action_server*` matched exactly the four action clients and the three
action servers and nothing else; `service-client` / `service-server` are the
nearest misses and score 0:

| image | client | server | `ActionClientCore` |
| --- | ---: | ---: | ---: |
| `qemu-rtic-action-client` | 1 | 0 | 6 |
| `qemu-rtic-action-server` | 0 | 1 | 6 |
| `qemu-rtic-talker` | 0 | 0 | 6 |
| `qemu-rtic-service-client` | 0 | 0 | 6 |
| `action-client` (x86_64) | 1 | 0 | 6 |
| `action-server` (x86_64) | 0 | 1 | 6 |

Over all 221 images the tool can reach on a fully built tree, the verdicts and
the file NAMES agree exactly: 35 "matched", every one of them an
`*action-client*` or `*action-server*` binary.

### The action SERVER counts too, though the knob is named for clients

The knob's real meaning is "how many slots are budgeted at the WORST-CASE entry
size", and `build.rs` picked the action client as that worst case. It is not.
Measured from `target/nros-cpp-generated/nros/nros_cpp_config_generated.h`
(x86_64, default 1024-byte buffers):

    NROS_CPP_RAW_ACTION_CLIENT_OPAQUE_U64S = 654  ->  5,232 B
    NROS_CPP_RAW_ACTION_SERVER_OPAQUE_U64S = 799  ->  6,392 B

**Caveat on those two numbers, added on integration.** `NROS_CPP_RAW_ACTION_*_
OPAQUE_U64S` are the C++ FFI OPAQUE HANDLE sizes, not arena entry sizes. The
arena budgets an action client at 18,048 B from a different formula
(`3 x (4096+384) + 3 x rx_buf + 1536`), so 6,392 vs 5,232 does NOT say the
action-server ARENA ENTRY exceeds the client's — only that the C++ handle does.

The conclusion the gate acts on survives the correction, for a different reason:
the arena demonstrably stores action servers (`ActionServerArenaEntry`,
`ActionServerRawArenaEntry` in `arena.rs`), so an action-server image occupies
slots and must not be advised to zero the knob. Treating both entities as heavy
is conservative and correct whichever handle is larger. The size ORDERING is
recorded as suggestive, not as the justification.


against a pub/sub entry budgeted at `3 x rx_buf + 512` = 3,584 B. So an action
server needs a slot a pub/sub budget cannot hold either, and advising an
action-server image to set `NROS_EXECUTOR_ACTION_CLIENTS=0` would have traded the
advisory for the exact `BufferTooSmall` this gate exists to prevent. Both
entities are "heavy" here. (A follow-up worth filing: the derivation's own
comment still calls the action client the worst case, and it is 1,160 bytes
short of one.)

### What it cannot decide, and says so

A C or C++ image links `nros-c` / `nros-cpp` with `--whole-archive` (the issue
0475 shape), so the staticlib is in the image whether or not the app calls any
of it. `NodeHandle::create_action_client_raw` is a GLOBAL in **every** C entry:
`native_entry` (a talker) and `native_action_client_entry` are indistinguishable
on this probe. Those are reported INDETERMINATE, never OK, detected by the FFI
surface itself (`nros_cpp_action_client_create` / `nros_action_client_init`),
which is absent from every pure-Rust image measured. CMake-built entries outside
a cargo profile dir are unreachable for a second, independent reason — there is
no `nros_node_config.rs` to attribute to them — so the knob half is missing for
exactly the same population.

A tree with nothing built exits 78 and records a **SKIP**, never a pass: a gate
that examined zero images must not read like one that agreed.

### What the tree looks like today

Zero images are in the dangerous direction — nothing anywhere sets the knob to
0, so there is no live `BufferTooSmall` to find. **181 of 221 images are
over-budgeted, 28 of them cross-compiled** (`thumbv7m-none-eabi`,
`riscv32imc-unknown-none-elf`), each carrying a 74,240-byte arena on a task
stack for an entity it does not have. The gate does not fix that; it makes it
enumerable, which is what `NROS_EXECUTOR_ACTION_CLIENTS` never was.

**Still not done:** nothing sets the knob automatically. This gate reports the
mismatch, it does not resolve it.

## This is phase-392 W2, found from the other end

Filed from a measurement — `ARENA_SIZE` is 74,240 bytes on every generated
config in the tree — and only afterwards met
[phase-392](../../roadmap/phase-392-static-memory-space-campaign.md)'s **W2,
"precise executor arena"**, which had already planned this work and even
anticipated the runtime-measurement half:

> the likely answer is a runtime high-water mark reported at teardown plus a CI
> lane that fails when it exceeds the configured arena

W1 here delivered that half (exactly, not approximately: the arena is a bump
allocator, so `arena_used()` is the claimed total rather than a high-water
estimate). W2 here added `NROS_EXECUTOR_ACTION_CLIENTS`, a lever the phase did
not have.

Still owed to the phase: the STATIC half — `NROS_ARENA_REQUIRED` emitted by
entry codegen, checked by `nm` — plus a CI lane. **With one correction the phase
could not have known:** the arena is inline on the TASK STACK, not in `.bss`, so
a linker-symbol check cannot see it. That half needs rethinking, not just
writing.

## Direction for the rest

## Not to be confused with

Issue 0896, which is about a SUBSCRIPTION's receive buffer taking the small size
class because nothing states a per-type bound. This is one level up: even with a
perfect per-type hint, the arena slot holding that subscription is still
budgeted as though it were an action client. The two share the
`NROS_SUBSCRIPTION_BUFFER_SIZE` knob and nothing else.

## The remedy this issue assumed does not exist (2026-08-31)

The obvious fix — derive `NROS_EXECUTOR_ACTION_CLIENTS` from what the image
declares, the way `entity-facts` already derives the queryable figures —
**cannot be done**, and the reason is structural rather than missing plumbing.

**The mechanism is already in place.** `nros-node/build.rs` takes
`NROS_EXECUTOR_ACTION_CLIENTS` (default `max_cbs`, so the old formula reproduces
byte for byte and no existing image moves) and budgets the two entry SHAPES
separately instead of charging every slot the larger one. The knob is plumbed
through `zephyr/Kconfig`, `platform_config.rs`, the generated configuration
surface, and the runtime advisory names it when an arena is too small. Setting
it to 0 takes 74,240 bytes to 16,384.

**Nothing sets it, and nothing can.** `source_metadata.rs` — the structure a
model is built from — contains ZERO occurrences of "client". Its entity types
are:

```
SourcePublisher   SourceSubscriber   SourceTimer
SourceService   (a `callback` — the SERVER side)
SourceAction    (goal/cancel/accepted callbacks — the SERVER side)
```

nano-ros declares the entities that need CODEGEN REGISTRATION: publishers,
subscribers, timers, and the server halves of services and actions. A client —
`ActionClientCore::new`, a service client — is constructed imperatively in user
code and is invisible to the build. That is a coherent design; it simply means
the arena cannot be sized from the model, because the model does not know.

So the count is not "not yet wired". It is **not expressible**.

## The decision this needs

1. **Declare client-side entities.** Extend `SourceMetadata` with action and
   service clients so `entity-facts` can report them. Buys the automatic
   derivation, and would let other table sizing tighten too. Costs a schema
   addition, a resolver change and a migration for every existing model — and
   asks users to declare something they currently just construct.
2. **Leave the knob manual and flip the DEFAULT.** Today it defaults to
   `max_cbs`, the worst case, so nobody pays a surprise. Defaulting to 0 would
   make every pub/sub image 56.5 KiB smaller and break every action-client image
   at REGISTRATION rather than at link — a runtime failure for a compile-time
   fact, which `arena_used()` and the first-spin advisory soften but do not fix.
3. **Derive it from the LINKER, not the model.** An image that never references
   `ActionClientCore::new` cannot have one. That is a post-link fact, so it
   cannot size a compile-time constant — but it can power a GATE that fails an
   image whose knob is larger than its linked reality, turning a manual setting
   into a checked one.

(3) is the interesting one: it keeps the knob manual but stops it being a guess,
needs no schema change, and is the only option verifiable with `nm`. That last
point matters here — the arena is INLINE ON THE TASK STACK, so `just mem-report`
cannot see it at all. A symbol probe cannot measure the arena, but it can see
whether the entity justifying its size was linked in.

Recorded rather than acted on: choosing between these is a design decision, and
this issue's original remedy assumed information the tree does not carry.

## Option 1 explored — cheaper than stated, and unsafe alone (2026-09-01)

Two objections raised against option 1 above were WRONG, and the correction
matters because it made option 1 look more expensive than it is:

* **"Costs a migration for every existing model"** — no. `SourceMetadata` is a
  DERIVED sidecar, content-addressed by `inputs_digest` and stamped with a
  generator version. It is regenerated, not hand-maintained. A new field needs
  `#[serde(default)]` (the struct is `deny_unknown_fields`) and nothing else.
* **"Asks users to declare what they construct"** — no. Users declare nothing.
  `metadata_mode::record_entity` already captures the client when the node
  builds it, and `EntityKind` ALREADY HAS `ActionClient` and `ServiceClient`.

The gap is one place. `node_metadata.rs:782-784` writes arrays for publishers,
subscribers, timers, `services` (`ServiceServer`) and `actions`
(`ActionServer`) — and never writes one for the client kinds. The data is
recorded and then dropped on the way out.

So the change is small: two more `write_entity_array` calls, two small structs
(clients register no callbacks, so they are simpler than their server
counterparts), a sum in `entity-facts`, and one more key through the cmake seam
that already delivers the queryable figures. `build.rs` needs no change.

### The hazard that stops it being sufficient

**A cross-compiled component is UNPROBEABLE by construction.** Metadata mode
runs the component to capture entities and needs a host build;
`metadata_build.rs`'s own test is named
`build_std_with_a_foreign_target_is_unprobeable`, and the probe comments record
that "ONE unprobeable component degrades to the sidecar-less path". Four
embedded example leaves carry `.unprobeable.` markers today — including
`examples/qemu-arm-freertos/rust/action-client`, which is an action client.

If the derivation sums clients from a sidecar-less component it gets **0**, and
sizes the arena DOWN for an image that may well have an action client. That is
option 2's registration failure arriving silently, on the targets least able to
report it.

So the count must be a TRI-STATE, not a number: `known(n)` versus `unknown`,
with `unknown` falling back to today's worst-case default. Absence of evidence
is not evidence of absence, and this is the shape where that distinction is
load-bearing.

### And that limits what option 1 can buy

With the tri-state, the tight arena reaches PROBED components — host and native
— and never reaches the unprobeable cross builds. The 56.5 KiB is a stack cost,
so it matters most exactly where option 1 cannot help.

**Option 3 covers that half.** `nm` works fine on a cross ELF: an image that
never references `ActionClientCore::new` cannot have an action client, so the
knob can be CHECKED against the linked reality precisely where it cannot be
derived.

So the two are complements, not alternatives:

* **1** — automatic and tight where the component can be probed;
* **3** — a checked manual knob where it cannot.

Neither alone is both safe and useful across the tree.

## The saving, MEASURED (2026-09-04) — and the default is wrong for action images too

Everything above is mechanism. This is what it delivers, read from
`nros-node`'s generated `ARENA_SIZE` rather than argued from the formula. The
arena is inline on the task stack, so `just mem-report` cannot see it; the
generated constant is the number that sizes the reservation.

Isolated `CARGO_TARGET_DIR` per run, one variable, defaults otherwise
(`MAX_CBS=4`, `DEFAULT_RX_BUF_SIZE=1024`):

| `NROS_EXECUTOR_ACTION_CLIENTS` | `ARENA_SIZE` | saved vs default |
| --- | ---: | ---: |
| unset (= `MAX_CBS`, today's default) | 74,240 | — |
| 1 | 30,848 | **43,392 B (42.4 KiB)** |
| 0 | 16,384 | **57,856 B (56.5 KiB)** |

The 56.5 KiB this issue has claimed throughout is confirmed to the byte.

**What is new is the middle row, and it changes who this is for.** This issue is
written as though the saving belongs to pub/sub images — "every pub/sub image
56.5 KiB smaller" is how option 2 is phrased. But an image with ONE action
client, which is what an action-client image typically has, still saves
**43,392 bytes**, because the default budgets ALL FOUR slots at the action-client
size rather than the one that needs it. The default is not merely conservative
for images with no action client; it is wrong by 42 KiB for the images it was
chosen to protect.

That also removes the sharpest edge from option 2's objection. Flipping the
default to 0 was rejected for breaking "every action-client image at
REGISTRATION". The derivation makes that moot where it runs — an action-client
image derives 1, not 0 — and the gate (W3) catches `knob == 0` with an action
entity linked. The remaining exposure is an image that is neither derived nor
gated, which is the same set the tri-state already falls back to worst-case for.

Not measured here: an end-to-end image build showing the stack reservation
shrink. The constant is what sizes it, and no image in the tree currently
derives a non-default value, so that measurement needs an image built through
the CMake seam with a declaring `nano_ros_node_register`.


## The end-to-end measurement this issue was waiting for (2026-09-05)

The section above closes with "Not measured here: an end-to-end image build
showing the stack reservation shrink … no image in the tree currently derives a
non-default value, so that measurement needs an image built through the CMake
seam with a declaring `nano_ros_node_register`."

That is no longer true, and it did not need the CMake seam. Issue 0827's
derivation makes `nros sync` write `NROS_EXECUTOR_ACTION_CLIENTS` and
`NROS_EXECUTOR_MAX_CBS` into a per-leaf `[env]` sidecar, computed from the
metadata probe through the same `entity_inventory` rules. So in-tree cargo
leaves now derive non-default values, and the constant can be read from a real
build instead of a synthetic one.

Read from `nros-node`'s generated `nros_node_config.rs`, same host, same tree,
same profile, sidecar present vs absent:

| leaf | | `MAX_CBS` | `ARENA_SIZE` |
| --- | --- | ---: | ---: |
| `native/rust/talker` | stock | 4 | 74,240 |
| | derived | 1 | **8,192** |
| | **saved** | | **66,048 (64.5 KiB)** |
| `native/rust/action-client` | stock | 4 | 74,240 |
| | derived | 1 | **20,096** |
| | **saved** | | **54,144 (52.9 KiB)** |

**Both rows are larger than this issue predicted, and for a reason worth
stating.** The 56.5 KiB and 43,392 B figures above were measured with `MAX_CBS`
held at its default of 4 and only `ACTION_CLIENTS` varied. A derived image moves
BOTH: the talker declares one callback, so it gets `MAX_CBS=1` as well, and the
two compound.

The action-client row is the one that settles the argument this issue kept
having with itself. An image that genuinely HAS an action client — the case the
worst-case default was chosen to protect — still recovers **52.9 KiB**, because
the default budgets all four slots at action-client size when exactly one needs
it. The default was not conservative for that image; it was wrong by 52 KiB.

### What that does to the three options

Option 1 ("declare client-side entities … costs a schema addition, a resolver
change and a migration") is delivered for probeable leaves at none of those
costs: the probe already records `action_clients`, so nothing new is declared
and no model migrates. Option 3's gate stays as the backstop, and option 2 —
flipping the default — is not needed where the derivation runs.

### Where it does not run

A leaf whose metadata is `.json.unprobeable` derives nothing and keeps the
worst-case default: correct, and loud rather than silent, since `nros sync` now
says so. That set is issue 1061. An image built through the CMake seam gets the
same numbers from `entity-inventory` directly, which is the path phase-403 W9
already wired.
