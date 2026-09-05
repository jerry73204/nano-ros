# Phase 391 — one funnel, one arena, a constant-time allocator behind it, and a link-time gate that proves it

**Status (2026-08-30). W1-W5 landed; the tier is real and gated.** Opened from a
memory-allocation review. [Issue 0817](../issues/archived/0817-platform-funnel-bypassed-in-zephyr-port.md)
(the sixteen Zephyr funnel bypasses) is fixed and archived.

Landed since: **W1/W1b/W4** (`heap-free` tier declared per image and gated in
`ci-l3` on a real heap-free build), **W2** (rlsf behind the funnel, O(1)
alloc/free, pool bound compiler-checked, measured before/after on a named
image), **W3** (Zephyr's funnel is rlsf-backed; `kheap` 65,536 -> 1,024,
measured A/B on `build-c-talker-zenoh`), and the **W5 endgame** — per-class
exact cell registries via `Node::ENTITY_BOUNDS`, trampoline contexts moved into
the cell, and `node_runtime` finally dropping its `alloc` gate, which was the
finish signal this phase was written to reach.

Read the per-wave "LANDED — measured" blocks below rather than this summary
before quoting a number; each carries its own A/B.

Depends on [phase 390](phase-390-storage-mode-rename-inline-heap-view.md) for
vocabulary only, not for code.

## Where this starts from

The architecture RFC-0034 D6 describes is already built, and mostly true:

- one `#[global_allocator]` in the tree — `nros-platform/src/lib.rs`
- one funnel — `nros_platform_alloc`
- `zpico-alloc`'s own docs: *"this crate is the ARENA, not the allocator"*
- bare-metal platforms already ship the arena in the platform package:
  `static HEAP: FreeListHeap<HEAP_SIZE>` in `nros-platform-{mps2-an385,stm32f4,esp32-qemu}/src/memory.rs`

Verified end-to-end on mr_canhubk3/s32k344 by disassembly: `z_malloc`
tail-calls `nros_platform_alloc`, so zenoh-pico's 42 allocation sites and the
Rust global allocator share one funnel.

**So this phase is not "build a funnel". It is: replace what sits behind the
funnel, and make the property checkable.**

## The gap: the arena is not real-time

`zpico-alloc::FreeListHeap` is first-fit with an address-ordered free list and
a 64-byte slab fast path:

```rust
let mut current = self.get_free_list();
while !current.is_null() {                  // O(n) walk
    if (*current).size >= aligned_size {
```

Good fragmentation behaviour — Robson (1977) showed first-fit is near-optimal
and best-fit is nearly worst-possible — but **O(n)**, so it has no worst-case
execution bound. That is the property a safety-island image needs and does not
have.

## rlsf, and what it costs (measured, not estimated)

[TLSF](http://www.gii.upv.es/tlsf/files/papers/ecrts04_tlsf.pdf) is O(1) for
allocate and free regardless of heap state, via two-level segregated free lists
plus bitmaps and a `CLZ` — a single instruction on Cortex-M7. Internal
fragmentation is bounded at `1/SLLEN`.

Candidates surveyed:

| impl | license | cert claims | verdict |
| --- | --- | --- | --- |
| [rlsf](https://github.com/yvt/rlsf) 0.2.3 | MIT/Apache-2.0 | none | **chosen** |
| [o1heap](https://github.com/pavel-kirienko/o1heap) | MIT | MISRA C:2012, published WCMC formula | rejected — see below |
| [mattconte/tlsf](https://github.com/mattconte/tlsf) | BSD | none | unmaintained since 2016 |
| [UPV original](http://www.gii.upv.es/tlsf/main/license) / ros2/tlsf | **GPL/LGPL dual** | none | licence non-starter |

o1heap has the better certification story and was still rejected on merit: it
is **not TLSF**. It is single-level, one bin per power of two, and rounds every
request up:

```c
const size_t alloc_size = roundUpToPowerOf2(amount + O1HEAP_ALIGNMENT);
```

Worst-case internal fragmentation approaches 100%. A 6,220,800-byte 1080p frame
rounds to 8,388,608 — 2.17 MB wasted on one message. Disqualifying for the
large-payload case, whatever the paperwork says.

**Measured cost of rlsf on `thumbv7em-none-eabihf`, `opt-level="z"` + LTO:**

| FLLEN/SLLEN | `.text` | `.bss` | max internal frag |
| --- | --- | --- | --- |
| 8/8 | 608 B | 276 B | 12.500% |
| 12/8 | 608 B | 412 B | 12.500% |
| **12/16** | **600 B** | **796 B** | **6.250%** |
| 16/16 | 608 B | 1060 B | 6.250% |
| 16/32 (needs a `u32` SL bitmap) | 596 B | 2116 B | 3.125% |

Code size is flat; the fragmentation bound is bought with `.bss`, ~136 B per FL
class. Per-allocation overhead is an 8 B header
(`GRANULARITY/2`, `GRANULARITY = size_of::<usize>() * 4` = 16 on 32-bit) plus
16 B granularity rounding plus the class rounding above.

**Net against the current image**, replacing rather than adding — Zephyr's
`sys_heap` is 1,856 B of code that garbage-collects once nothing calls
`k_malloc`:

```
text  +600 (rlsf) +282 (glue, no realloc) -1856 (sys_heap)  =  -974 B
bss   +796 (control struct); the 16,384 B k_heap becomes the rlsf arena
```

**A net flash shrink and 796 B of RAM.** Implementation footprint is not the
argument in either direction.

## What makes this defensible now

The decision that payload buffers **stay static** (they do not move to the
heap) is what makes TLSF sizeable here. Robson's bound scales with the ratio of
largest to smallest block; a heap holding both 20-byte key expressions and
megabyte payloads has a ~2^16 spread and a punishing worst case. A heap that
holds only *infrastructure* — zenoh-pico's sessions, key expressions and
strings, and Rust `String`/`Vec` churn — has a narrow spread, and the bound
becomes cheap to defend.

## The tier model

Heap-freedom is not nano-ros's to give up or keep — **it is the vendor RMW that
requires a heap.** zenoh-pico reaches the allocator from 42 call sites and is
third-party C. A consumer who brings a heap-free RMW must still get a heap-free
image, so this is a tier, not a global choice:

| tier | rule | who can reach it |
| --- | --- | --- |
| `heap-free` | **no** allocation symbol in the linked image | consumers with a heap-free RMW; embassy/RTIC integrations |
| `unified` | allocation symbols only inside `nros_platform_*` backend objects | the zenoh and cyclone tiers |

The tree is already built for this: `alloc` is a Cargo feature and every core
crate gates `extern crate alloc` on it. What is missing is enforcement — the
book already promises "fully no-alloc" for embassy and RTIC with nothing
checking it ([issue 0816](../issues/0816-no-alloc-claimed-but-unenforced.md)).

## Waves

**W1 — link-time allocation gate.** `nm` the built image, deny
`malloc`/`calloc`/`realloc`/`free`/`k_malloc`/`k_free`/`pvPortMalloc`/... at the
tier's strictness. A symbol gate, not a source grep — a grep cannot see
vendored C, which is where 42 of the sites are. Closes issue 0816; would have
caught all sixteen sites in issue 0817. **Do this first** — it is what verifies
every later wave.

**W1b — the gate must also assert the libc arena is ZERO.**
Added 2026-08-29. W1 denies the `malloc` *symbol*, which is necessary and not
sufficient: `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE` reserves its pool in `.bss`
**whether or not any caller survives the linker**. On mr_canhubk3/s32k344 the
image contains no `malloc` and no `free` — every allocation goes through
`nros_platform_alloc` — and the arena was reserved anyway.

This has now been found on the same board **twice**: 24,576 B during the survey
that opened this campaign, and 8,192 B again on 2026-08-29 after the setting was
lost in a rebuild. A finding that regresses is not a finding, it is a gate that
was never written.

So the symbol gate gains a companion assertion on the *configuration*: with the
unified arena behind the funnel there is no libc allocator to serve, and
`CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE` must be `0` in every image at a tier that
routes through `nros_platform_alloc`. Same argument, same wave, for
`CONFIG_HEAP_MEM_POOL_SIZE`, which W5 already sets to 0.

Dead code is collected. Dead **reservations** are not, and only a gate on the
config catches them.

**W2 — rlsf behind the funnel.** Replace `FreeListHeap`'s internals in
`zpico-alloc` and the three `nros-platform-*/src/memory.rs` statics. The arena
and the `z_malloc`/`z_free` shim structure do not change; only the algorithm
does.

**Vetted 2026-08-27 — rlsf clears every prerequisite, and two costs the survey
above did not know are lower than it assumed.**

*Suitability.* `#![no_std]` unconditionally (`lib.rs:2`, not cfg-gated); its
`std` feature is opt-in and off by default. Edition 2021, `rust-version` 1.61.
Dependencies are `cfg-if`, `const-default` (`default-features = false`) and
`rustversion` — none reaches std. The API is the shape this funnel needs:
`Tlsf::new()` is a `pub const fn` (so it can back a `static`),
`insert_free_block_ptr(NonNull<[u8]>)` takes a static arena, and
`allocate(Layout) -> Option<NonNull<u8>>` / `deallocate(ptr, align)` match the
funnel's two operations. `reallocate` exists if the glue ever wants it.

*The tuning table is expressible and self-consistent with the source.*
`Tlsf<'pool, FLBitmap, SLBitmap, const FLLEN: usize, const SLLEN: usize>` is
generic over exactly the parameters the table varies, with the bitmaps as
separate type parameters — which is why the `16/32` row needs a `u32` SL
bitmap: `sl_bitmap: [SLBitmap; FLLEN]` has to be wide enough for SLLEN.

*Two things that lower the cost of adopting it:*

* **rlsf 0.2.3 is already vendored in the local cargo registry** (0.2.2 and
  0.2.3 both present), so adding it needs no network — which matters because
  `--locked` is injected project-wide by the `scripts/bin/cargo` shim.
* **It is already a transitive dependency of `nros-board-esp32-qemu`**, via
  `esp-alloc 0.9.0`. rlsf therefore already compiles for a bare-metal target in
  this tree. "Does it build for our targets" is retired as a risk, and
  `nros-platform-esp32-qemu` would be consolidating onto an allocator its own
  board already links rather than adding a second one.

*Measured on `thumbv7m-none-eabi`, `opt-level="z"` + LTO + `codegen-units=1`* —
a DIFFERENT target from the survey table above, which is `thumbv7em-none-eabihf`:

```
FLLEN=12 SLLEN=16, u16/u16 bitmaps
  control struct (.bss)   796 B     <- matches the table's 12/16 row exactly
  code + 3 C wrappers     720 B
```

The `.bss` figure reproduces the survey's 796 B independently, on M3 rather than
M7, which is expected — the control struct is `fl_bitmap` + `[SLBitmap; FLLEN]`
+ `[[Option<NonNull>; SLLEN]; FLLEN]`, and both targets are 32-bit. The harness
was validated by decomposition rather than asserted: total probe `.bss` was
4,892 B = 796 (control) + 4,096 (the probe's own arena).

The code figure is NOT directly comparable to the table's `600 B (rlsf) + 282 B
(glue)`: this 720 B is rlsf plus three thin `extern "C"` wrappers under LTO,
not the same glue. Same order; do not subtract them from each other.

**THE SURVEY'S CHOSEN 12/16 CANNOT SERVE TWO OF THE THREE PLATFORMS.** This is
the one measurement that contradicts the section above, and it is a correctness
constraint, not a tuning preference.

rlsf caps the pool it can hold (`tlsf.rs`):

```rust
const MAX_POOL_SIZE: Option<usize> = {
    let shift = GRANULARITY_LOG2 + FLLEN as u32;   // GRANULARITY_LOG2 = 4 on 32-bit
    if shift < usize::BITS { Some(1 << shift) } else { None }
};
```

`GRANULARITY = size_of::<usize>() * 4` = 16, so `MAX_POOL_SIZE = 1 << (4 + FLLEN)`
and the largest single block is `(GRANULARITY << FLLEN) - GRANULARITY`.

| FLLEN | max pool | verdict against our arenas |
| --- | --- | --- |
| **12** (the row this doc chose) | **64 KiB** | too small for two of three |
| 14 | 256 KiB | covers the 128 KiB arenas |
| 18 | 4 MiB | covers the 2 MiB arena |

The arenas, from the statics this wave is supposed to convert:

| platform | `DEFAULT_HEAP_SIZE` | needs |
| --- | --- | --- |
| `nros-platform-stm32f4` | 32 KiB | FLLEN >= 12 (12/16 is fine) |
| `nros-platform-mps2-an385` | 128 KiB | **FLLEN >= 14** |
| `nros-platform-mps2-an385` (one cfg) | 2 MiB | **FLLEN >= 18** |

So the `.bss` figure in the survey table is an understatement for the platforms
that matter: at ~136 B per FL class, 12 -> 18 is roughly +816 B on top of the
796 B, i.e. ~1.6 KiB rather than 796 B for the 2 MiB arena. That is still small
against a 2 MiB heap, but it is not the number this doc currently promises, and
picking 12/16 as written would fail to hold the pool at all.

**DESIGN REVISION (measured 2026-08-27): a DEFAULT const parameter, with the
adequacy of FLLEN for N asserted at compile time.** This supersedes the three
options first sketched here (one-size FLLEN / derive-from-N / split the arena).

```rust
pub struct FreeListHeap<const N: usize, const FLLEN: usize = 18> { .. }

impl<const N: usize, const FLLEN: usize> FreeListHeap<N, FLLEN> {
    const MAX_POOL: usize = 1usize << (4 + FLLEN);   // GRANULARITY_LOG2 = 4, 32-bit
    pub const fn new() -> Self {
        assert!(N <= Self::MAX_POOL,
                "NROS_HEAP_SIZE exceeds rlsf MAX_POOL_SIZE for this FLLEN");
        ..
    }
}
```

Every existing call site keeps working — `FreeListHeap<N>` takes the default —
and a platform that wants a smaller control struct names its own:
`FreeListHeap<{32 * 1024}, 12>`.

*Verified on `thumbv7m-none-eabi`, both directions:*

* `FreeListHeap<{32 * 1024}, 12>` and `FreeListHeap<{128 * 1024}>` compile.
* `FreeListHeap<{128 * 1024}, 12>` — a 128 KiB arena against a 64 KiB max pool
  — **fails the build**: `error[E0080]: evaluation panicked: NROS_HEAP_SIZE
  exceeds rlsf MAX_POOL_SIZE for this FLLEN`. The negative control was run
  deliberately, because a guard nothing can trip is this campaign's own named
  trap.

Default const parameters and `assert!` in a `const fn` both work on the pinned
toolchain, so this needs no `generic_const_exprs` — which is what blocked the
derive-FLLEN-from-N option.

*Measured control-struct cost, SLLEN=16, same target:*

| FLLEN | `.bss` | max pool | fits |
| --- | --- | --- | --- |
| 12 | **796 B** | 64 KiB | stm32f4 (32 KiB) |
| 14 | **928 B** | 256 KiB | mps2-an385 (128 KiB) |
| 18 | **1,192 B** | 4 MiB | mps2-an385 (2 MiB cfg) |

**This corrects the survey table's per-class figure.** The measured slope is
**66 B per FL class** at SLLEN=16, not the ~136 B this doc claimed — 136 is the
SLLEN=32 slope. So the worst case (FLLEN=18) is +396 B over the already-accepted
796 B, not the ~+816 B a reader would extrapolate. Sizing for the largest arena
is therefore much cheaper than it first appeared, and the default of 18 costs a
32 KiB board 1,192 B (3.6% of its arena) if it does not override.

**W2 LANDED — measured before/after on a named image (2026-08-27).**

`qemu-bsp-talker`, mps2-an385, `thumbv7m-none-eabi`, `nros-relwithdebinfo`,
built by `just qemu build-fixtures`, measured with
`scripts/nros-mem-report.py`. Both arms built from the SAME tree with only
`zpico-alloc` differing (the pre-W2 arm produced by
`git checkout <W2>~1 -- packages/rmw/zenoh/zpico-alloc/`), so the delta names
this wave and nothing else:

| | before (first-fit) | after (rlsf) | delta |
| --- | --- | --- | --- |
| `nros_platform_mps2_an385::memory::HEAP` | 131,608 B | 132,792 B | **+1,184 B** |
| RAM (`.bss` + `.data`) | 386,900 B | 388,084 B | **+1,184 B** |

RAM total moves by exactly the HEAP delta, so nothing else shifted. The
decomposition holds:

```
arena      131,072 -> 131,072   (unchanged, by design)
metadata       536 ->   1,720   (+1,184)
                             = 512 slab + 1,192 rlsf control + 16 padding/flags
```

**This is a COST, not a saving, and it should be read as one.** The survey above
projects a net flash shrink and 796 B of RAM, but that arithmetic is the ZEPHYR
case, where rlsf REPLACES `sys_heap` (−1,856 B of text) and the 16 KiB `k_heap`
becomes the arena. On bare-metal there is no `sys_heap` to remove, so the
control struct is added with nothing offsetting it. W3 is where the offset
appears; W2 alone buys a worst-case execution bound and pays 1,184 B for it on a
458,752 B part (0.26%).

A cross-check worth keeping: an image built 32 commits earlier measured the
identical 131,608 B / 386,900 B, so none of the intervening work touched this
image's RAM — which is why the isolated rebuild and the historical image agree.

**BLOCKED on file ownership, not on technique.** `FreeListHeap`'s implementation
is `packages/rmw/zenoh/zpico-alloc/src/lib.rs`, and the rlsf dependency would be
added to that crate's manifest. The three `nros-platform-*/src/memory.rs` files
are 94-line wrappers that only `use zpico_alloc::FreeListHeap` and size the
arena from `NROS_HEAP_SIZE`; changing them alone would leave the tree pointing
at a type whose O(n) first-fit walk is unchanged — a diff that reads as "W2
landed" while delivering none of its property.

**W3 — Zephyr tier: `CONFIG_HEAP_MEM_POOL_SIZE=0`.** Repoint
`nros_platform_alloc` at rlsf and let `sys_heap` garbage-collect. Requires that
no Zephyr subsystem calling `k_malloc` is enabled (fs, mcumgr, net,
`log_mgmt`, cfb — none are in the serial image, but this needs a link test, not
an assertion). Prerequisite
[issue 0811](../issues/archived/0811-zephyr-net-iptcp-allocator-provenance-mismatch.md)
is RESOLVED (e71548e44) — the multicast-teardown use-after-free is gone, so the
two-allocator overlap no longer protects anything.

**TRIAGE (2026-08-28) — measured on
`zephyr-workspace/build-cortex-m-c-talker-zenoh` (mps2/an385, zenoh/TCP; STALE
vs today's tree but structurally current):**

* **The funnel is already unified on Zephyr, proven by disassembly, not
  assumed:** `z_malloc` is `b.w nros_platform_alloc` AND the compiler-emitted
  `__rust_alloc` is `b.w nros_platform_alloc`. One funnel; behind it,
  `nros-platform-zephyr/src/platform.c:160-183` is three `k_malloc`/`k_free`
  lines. W3 is exactly those three lines plus a conf.
* **The prize:** `kheap__system_heap` = 131,072 B `.bss`
  (`CONFIG_HEAP_MEM_POOL_SIZE=131072` in this image, not the doc's 16 KiB
  example) + the live `sys_heap`/`k_heap` text
  (`sys_heap_aligned_alloc` 232, `k_heap_aligned_alloc` 130, …). The rlsf arena
  replaces the k_heap byte-for-byte at whatever size the knob picks, and W2's
  `.bss` cost (+1,192 control) finally buys its offset here.
* **A premise correction for this doc's own header:** "one
  `#[global_allocator]` in the tree — `nros-platform/src/lib.rs`" is the
  RUST-LANE story. The C-lane Zephyr image contains NO `nros-platform` crate at
  all (crate census by mangled prefixes: `nros_cpp`, `nros_node`,
  `nros_rmw_cffi`, `nros_rmw_zenoh`, `nros_platform_cffi` — nothing else);
  there, **`nros-c` owns the `#[global_allocator]`** ("behind the platform
  vtable", its own phase-361 W8.c comment). One per GRAPH, two in the tree.

**Implementation shape that follows from the census:** the arena must live in a
crate present in EVERY Zephyr image's graph, and `nros-c` is that crate (it is
also already the allocator owner). Zephyr-gated: `nros-c` gains a
`zpico-alloc` dependency + `static HEAP: FreeListHeap<{knob}>` + three
`extern "C"` exports; `platform.c`'s three lines call those instead of
`k_malloc`; the image conf sets `CONFIG_HEAP_MEM_POOL_SIZE=0`. Then the link
test the wave demands: `nm` the image — `k_malloc` and `sys_heap_*` must be
GONE (garbage-collected), which is simultaneously the proof that no enabled
subsystem needed them. Rust-lane images keep `nros-platform`'s opt-in
allocator and get the same arena through the same three C symbols.

**W3 LANDED — measured A/B on `build-c-talker-zenoh` (mps2/an385, zenoh/TCP),
both arms same tree via stash (2026-08-28):**

| | before | after | delta |
| --- | --- | --- | --- |
| `.bss` | 445,498 | 448,234 | **+2,736** = arena 67,248 − kheap freed 64,512 (closes exactly) |
| text+data (flash) | 316,387 | 317,605 | **+1,218** (rlsf + spinlock glue) |
| `kheap__system_heap` | 65,536 | **1,024** | `HEAP_MEM_POOL_ADD_SIZE_MQUEUE` — POSIX mqueue's declared floor |
| `k_malloc`-family syms | 6 | 5 | kept by the mqueue path |

Read the residuals honestly: on THIS image (TCP + POSIX mqueue) the kernel heap
cannot fully disappear — Zephyr's ADD_SIZE mechanism keeps a 1 KiB floor for
mqueue, which is the mechanism working as designed, not a failure. The
"k_malloc GONE from nm" outcome is real exactly for the serial images phase-392
targets (no net, no mqueue). What every image gets today: the funnel is O(1)
(rlsf), the heap is knob-sized by choice (`NROS_ZEPHYR_HEAP_SIZE`), and the
spinlock in `platform.c` supplies the thread-safety FreeListHeap's contract
demands — Zephyr is multi-threaded (zenoh read/lease tasks), which this doc's
W3 sketch had not accounted for.

Found and fixed on the way, both pre-existing:

* **One `0xFF` byte put every FreeListHeap arena in FLASH.** `slab_free_bitmap:
  AtomicU8::new(0xFF)` was the only nonzero byte in the static, which places
  the ENTIRE struct — arena included — in `.data` rather than `.bss`: 67 KB of
  load image here, and every bare-metal port has paid its arena size in flash
  since the slab landed. Sense inverted (set = USED, init 0); arena measured
  back in `.bss` (`b`).
* **`just zephyr build-one` was broken for `mps2/an385`** — the entropy
  fragment's case matched only `qemu_cortex_m3*`, so the board this repo
  actually targets failed on `z_impl_sys_rand_get`. Case widened; comment now
  also warns that `qemu_cortex_m3` is the TI LM3S6965 (256K/64K), 60x too
  small for a zenoh image.

Expected delta, to be MEASURED not asserted: −131,072 `kheap` `.bss`, −~600+ B
`sys_heap` text, +arena (knob-sized `.bss`), +~600 B rlsf text, +1,192 B rlsf
control — net text shrink, and the arena knob finally sized by choice rather
than by `HEAP_MEM_POOL_SIZE`'s guess.

**W4 — declare the tier per image, and gate it in CI.** Tier becomes a build
input; the W1 gate reads it. `heap-free` gets at least one lane that actually
builds and links.

**W5 — a static component pool in `node_runtime`, so the `heap-free` tier is
USEFUL rather than merely reachable.**
[Issue 0843](../issues/archived/0843-node-runtime-forces-alloc-on-every-cffi-image.md)
decoupled the allocation gate from the transport gate, so a cffi image now links
without `alloc`. What it did not do is leave anything useful behind: with
`alloc` off, `node_runtime` is gated out entirely, and it is the only path to a
running `Executor`. This wave removes the reason it needs a heap.

*Why this is nano-ros's problem and not a vendor's.* All four backends reach the
executor through ONE cffi seam, the seam allocates nothing, and the executor's
dispatch algorithm links heap-free. Every allocation between here and a working
heap-free image is ours:

| site | uses | why it allocates |
| --- | --- | --- |
| `node_runtime` registries | 35 `String`, 6 `Vec` | owned names + unbounded entity lists |
| `node_runtime` cells | 17 `Arc<ComponentCell>` | closures must outlive the executor |
| `node_runtime` slots | 3 `Box<dyn ComponentSlot>` | type-erased per-component state |
| `executor/spin.rs` | `leak_default_backing()` | leaks an arena when the caller supplies none |
| `executor/handles.rs` | `EventRegs` | boxed event callbacks |

*The count is not known at compile time; the BOUND is* — and that is all a pool
needs. `register_node::<C>()` is a runtime call, but the tree already answers
this shape twice, and `node_runtime` is the outlier:

| layer | bound | on overflow |
| --- | --- | --- |
| executor node table | `NROS_EXECUTOR_MAX_NODES` (build.rs, default 4) -> `config::MAX_NODES` | `NodeError::NodeTableFull` |
| `node_metadata` | `DEFAULT_MAX_METADATA_NODES` = 8, const-generic | bounded |
| **`node_runtime`** | **none** — `Vec<Arc<ComponentCell>>` grows without limit | — |

*Capacity comes from a BUILD-SCRIPT KNOB, not a const generic, and the reason is
FFI.* `node_runtime` carries nine `extern "C"` sites and backs
`__nros_component_<pkg>_install`, the uniform cross-language component-install
seam. A const generic would put a type parameter on a type that crosses into
C/C++; a baked `pub const` is invisible at the ABI. The tree already follows this
rule without stating it: `node_metadata` has ZERO `extern "C"` sites and uses
const generics freely; `node_runtime` has nine and uses none. So:
`NROS_RUNTIME_MAX_COMPONENTS`, emitted as a `pub const` exactly as
`NROS_EXECUTOR_MAX_NODES` is.

*What the pool buys, beyond the heap.* Cells in a `'static` pool outlive every
closure by construction, so the `Arc` refcount is proving a lifetime the pool
already guarantees. Closures and trampolines hold a `ComponentId` index instead.
All 17 `Arc` uses go, and they go because the ownership model got simpler, not
because they were worked around.

**Acceptance:** an image that CALLS runtime code — not one that names types —
links at tier `heap-free` and passes the W1 gate with `symbols read` well above
1. Three probes have already passed that gate vacuously at `symbols read: 1`
(`qos::DEFAULT`, `DEFAULT_MAX_METADATA_NODES`, and
`size_of::<internals::RmwSession>()`, which pulls no code even for a real type).
A pass without that symbol count is not evidence.

**RESOLVED (2026-08-27): caller-supplied storage, mirroring `Executor::open_in`.**
`Box<dyn ComponentSlot>` was the open question — the pool is heterogeneous
(`TypedSlot<C>` is generic over `C`), so cell storage cannot be
`[ComponentCell; N]`. The answer is the idiom this crate already uses one layer
down rather than a second one:

```rust
// existing, executor/spin.rs
pub unsafe fn open_in(
    config: &ExecutorConfig<'_>,
    backing: &'s mut [MaybeUninit<u64>],
    sizing: ExecutorSizing,
) -> Result<Self, NodeError>
```

`ExecutorSizing` is `{ cbs, sc, arena }` with a `DEFAULT` built from the
`MAX_CBS`/`MAX_SC`/`ARENA_SIZE` consts and a `u64_len()` that says how large the
backing must be. W5's analogue:

```rust
pub struct RuntimeSizing {
    pub components: usize,    // pool slots
    pub slot_bytes: usize,    // per-slot storage for the erased TypedSlot<C>
}
impl RuntimeSizing {
    pub const DEFAULT: Self = Self {
        components: config::MAX_COMPONENTS,        // NROS_RUNTIME_MAX_COMPONENTS
        slot_bytes: config::COMPONENT_SLOT_BYTES,  // NROS_RUNTIME_COMPONENT_SLOT_BYTES
    };
    pub const fn u64_len(&self) -> usize { .. }
}

pub unsafe fn ExecutorNodeRuntime::new_in(
    executor: Executor<'_>,
    backing: &'static mut [MaybeUninit<u64>],
    sizing: RuntimeSizing,
) -> Result<Self, NodeError>
```

Why this over sizing slots to the largest `C::State`: that figure is known to
codegen and NOT to a hand-written `main`, which is the split phase-392 W2 hit and
had to solve twice. Caller-supplied storage sidesteps it — codegen can emit a
`static` sized exactly, a hand-written `main` can declare one, and neither needs
the toolchain to know the other's answer. It also keeps the existing
alloc-convenience constructors meaningful: they become the `leak` variants of
`new_in`, exactly as `from_session` is to `open_in` today.

`slot_bytes` is a per-slot BYTE budget rather than a type: the pool erases
`TypedSlot<C>` into `&'static mut dyn ComponentSlot` carved from the backing, so
a slot too large to fit is a registration error (the `Full` case), not a compile
error in a generic the FFI seam would have to name.

### Sequencing

1. `nros` gains a `build.rs` (it has none today) emitting `MAX_COMPONENTS` and
   `COMPONENT_SLOT_BYTES` from `NROS_RUNTIME_*` knobs, matching
   `nros-node/build.rs`'s `NROS_EXECUTOR_MAX_NODES` -> `config::MAX_NODES`.
   NOTE this adds `nros` to the CLI freshness closure — regenerate
   `packages/cli/cli-source-dirs.txt` with `scripts/gen-cli-source-dirs.py`
   (CLAUDE.md) or `setup-cli` will skip rebuilds it should do.
2. `RuntimeSizing` + `new_in`, with the existing constructors reimplemented as
   the leaking variants. No behaviour change yet.
3. Pool + `ComponentId`; delete the 17 `Arc<ComponentCell>` clones. The
   trampolines at `component_tick_trampoline` / `component_drop_trampoline` /
   the three action+service ones currently carry an `Arc` through a
   `*mut c_void`; they carry an index instead.
4. `String`/`Vec` registries to `heapless`, capacities from the same knob family.
5. Gate `node_runtime` on `rmw-cffi` alone again — issue 0843's `alloc` half of
   the gate comes OFF, because the module no longer needs a heap. That is the
   signal the wave is done.

Steps 1-2 are inert and independently verifiable; 3 is the behavioural one.

### Steps 1 and 2 are LANDED (2026-08-27)

* **W5.1** — `packages/api/nros/build.rs` emits `config::MAX_COMPONENTS` (knob
  `NROS_RUNTIME_MAX_COMPONENTS`, default 4) and `config::COMPONENT_SLOT_BYTES`
  (default 512) via `nros_zephyr_build::knob_usize`. Verified the knob reaches
  the constant, not just that the script runs: a build with
  `NROS_RUNTIME_MAX_COMPONENTS=9` emitted `MAX_COMPONENTS: usize = 9`. Moved 16
  leaf lockfiles (one line each, `nros-zephyr-build`), all via `just lock-update`.
* **W5.2** — `packages/api/nros/src/runtime_storage.rs`: `RuntimeSizing
  { components, slot_bytes }`, `DEFAULT`, `u64_len()`. Four tests; the rounding
  test was verified to FAIL against the buggy spelling (rounding the total
  rather than each slot gives 5 words where 6 are needed).

### Step 3 — hand-off notes, because it is the unsafe one

Step 3 is deliberately NOT a tail-end-of-session change. It rewrites lifetime
invariants across the FFI seam, and **its failure mode is use-after-free, not a
compile error**. Start here:

**Baseline, captured 2026-08-27 and green:**

```
cargo nextest run -p nros-tests --features component-runtime-test \
    -E 'binary(component_runtime)'
  3 tests run: 3 passed
    runtime_registers_single_component_and_spins_once
    runtime_creates_publisher_for_declared_entity
    runtime_propagates_init_failure
```

**Note the `required-features`.** `component_runtime`, `component_dispatch` and
`component_param` are all behind `component-runtime-test`; without it cargo
skips them SILENTLY and a bare `nextest run` reports "0 tests run" while looking
successful. That is the issue-0652 class, and it is easy to mistake for "there
are no tests for this".

**The `Arc` lifecycle to be replaced** (`node_runtime.rs`):

| site | what it does |
| --- | --- |
| ~1566 | `Arc::into_raw(cell.clone())` -> `executor.enroll_component(raw, tick, drop)` |
| `component_tick_trampoline` | BORROWS that pointer each `spin_once` |
| `component_drop_trampoline` | RECLAIMS it on `Executor::drop` |
| ~905 / ~954 / ~997 | `Box::into_raw` of `ServiceServerCtx` / `ActionServerCtx` / `ActionClientCtx`, each holding its own `Arc<ComponentCell>` |

So the refcount is doing two jobs: keeping the cell alive behind four raw
pointers, and sharing it. A `'static` pool answers the first by construction —
which is the point — but every one of those provenance pairings has to be
re-established by hand as an index. The three tests above spin briefly and tear
down cleanly, so they would plausibly stay green over a latent
use-after-free; they are a floor, not the acceptance.

**`new_in` lands WITH step 3, not before.** W5.2 deliberately stopped at the
sizing arithmetic: a constructor that accepts backing and ignores it, because
the pool does not exist yet, would be an API that lies. So step 3 introduces the
constructor, the pool and the index migration together.

### 3a LANDED; 3b hit a structural blocker worth stating before anyone retries

**3a is in** (`runtime_storage::{Slot, carve}`): non-overlapping 8-aligned slots
from a caller-supplied backing, a fail-loud panic on a short one naming both
sizes, and `Slot::fits::<T>()` as a registration-time check. Eight tests,
including a `should_panic` control verified to FAIL when the assert is removed.

**3b was attempted and reverted.** The first half — `ComponentCell.slot` from
`RefCell<Box<dyn ComponentSlot>>` to `RefCell<&'static mut dyn ComponentSlot>`,
plus `new_in` and a bump `ComponentPool` — compiles down to exactly two errors,
and they are the whole problem:

```
node_runtime.rs:434   slot: RefCell::new(Box::new(TypedSlot::<C> { .. }))   // register_node
node_runtime.rs:1615  slot: RefCell::new(Box::new(TypedSlot::<C> { .. }))   // register_node_borrowed
```

`register_node` has the runtime and can draw a slot. **`register_node_borrowed`
cannot** — its signature is

```rust
fn register_node_borrowed<'p, C: ExecutableNode + 'static>(
    executor: &mut Executor<'static>,
    params: &'p [(&'p str, &'p str)],
    node_identity: Option<(&'static str, &'static str)>,
    remaps: &'p [(&'p str, &'p str)],
    qos_overrides: &'static [..],
) -> NodeResult<Arc<ComponentCell>>
```

no runtime, no pool. And it is not an internal detail: it is what
`install_node_typed` / `install_node_typed_with_params` /
`install_node_typed_with_launch` / `..._with_node_identity` call — i.e. the
`__nros_component_<pkg>_install` seam that C, C++ and every generated entry
enter through. So **W5 cannot proceed without deciding where the FFI install
path's slot storage comes from**, and that is a question about the seam, not
about `node_runtime`'s internals.

Three shapes, none chosen:

1. **Thread storage through the seam** — `install_node_typed` grows a
   backing/sizing argument. Honest and explicit, and it changes a signature that
   C, C++ and codegen all emit calls to; every generated entry would have to
   supply storage it currently does not know about.
2. **A module-level `static` pool** for the borrowed path, sized by
   `MAX_COMPONENTS`. No signature change, so the seam is untouched — but it
   reintroduces a fixed global exactly where W5 was trying to make storage
   caller-owned, and two runtimes in one image would share it.
3. **Split the paths** — caller-supplied storage for `register_node`, keep the
   boxed slot on the borrowed/FFI path and accept that `heap-free` excludes
   images that install through the C seam. Smallest change; narrows the tier's
   reach to pure-Rust entries.

Note (3) makes the tier's scope a deliberate decision rather than an accident,
which is more than the current state offers — but it should be chosen, not
defaulted into.

**RESOLVED (2026-08-28): a fourth shape supersedes all three — the MACRO emits
the storage, because the seam is a thin trampoline over a concrete type.**

The blocker assumed the FFI install path has no place to get storage from.
Reading the emit shows it does — the macro itself. The C ABI symbol is:

```rust
// nros-macros/src/lib.rs:346 — one emission PER nros::node! expansion
#[unsafe(no_mangle)]
pub extern "C" fn __nros_component_<pkg>_install(
    _node: *const c_void, executor: *mut c_void, _self: *mut c_void,
) -> i32 {
    unsafe { ::nros::install_node_typed::<#node_ty>(executor) }
}
```

Three consequences, each checked:

* **`#node_ty` is CONCRETE at emit time.** So the macro can emit, beside the
  trampoline, a `static` sized `size_of::<TypedSlot<#node_ty>>()` — EXACTLY, no
  `slot_bytes` byte-budget guessing at all on this path. This is the same
  stable-Rust distinction the static half recorded for phase-392 W3a: a generic
  parameter cannot appear in a const operation, but a concrete type's constants
  can. (A `static` inside the generic `install_node_typed` body would NOT work —
  Rust shares one static across all monomorphisations — which is exactly why the
  emission site is the macro, where the type is spelled out.)
* **The C/C++ ABI does not change.** `__nros_component_<pkg>_install` keeps its
  `(ptr, ptr, ptr) -> i32` shape; the storage argument is added to the RUST
  generic behind it (`install_node_typed_in`, with the old names delegating).
  Checked the callers of the Rust fn: the two macro emit sites
  (`nros-macros/src/lib.rs:351`, `main_macro.rs`) and two test files —
  `board/runtime.rs` mentions it only in docs. No foreign caller names it.
* **Multi-instance is real and must be capped, not assumed away.** The launch
  path bakes one identity per node in the plan (`main_macro.rs:848`,
  `node_identity_bakes`) and can name the same component class twice. So the
  emitted storage is a small per-class ARRAY, capacity from a knob
  (`NROS_RUNTIME_MAX_CLASS_INSTANCES`, default small), and an install past it is
  the same `Full`-style registration error as everywhere else in this campaign.

Where this leaves the two paths — both caller-owned, no global, no shared pool:

| path | storage | sizing |
| --- | --- | --- |
| FFI / generated (`install_node_typed*`) | macro-emitted per-class static array | EXACT — `size_of::<TypedSlot<C>>()`, concrete at emit |
| dynamic Rust (`register_node` / `new_in`) | caller-supplied backing via 3a's `carve` | `slot_bytes` budget, `Slot::fits::<T>()` at registration |

And the `Arc<ComponentCell>` migration rides the same emission: `ComponentCell`
is const-constructible field-by-field (`RefCell::new`, `AtomicUsize::new`,
`Cell::new` are all const; the slot field becomes
`RefCell<Option<&'static mut dyn ComponentSlot>>` so `None` const-inits it, and
the `String`/`Vec` registries go `heapless`, whose `new` is also const). A
per-emit `static` cell makes every one of the 17 `Arc` clones a plain
`&'static ComponentCell`; `component_drop_trampoline` stops reclaiming a leaked
refcount and instead drops the slot state in place.

Per-class static cost is visible, not hidden: each class pays
`instances x size_of::<TypedSlot<C>>()` in `.bss` under its own symbol, which is
exactly the granularity `nros-mem-report` attributes.

### STOPPED before the cell rework (2026-08-28) — the cell is ~20 KB and that is a fork, not a detail

The heapless registry conversion LANDED (all 35 `String` -> `IdStr` =
`heapless::String<128>`, all registries `heapless::Vec<_, CELL_REG_CAP = 32>`,
every overflow a registration error). It is also what surfaced the blocker.

**`ComponentCell` is now ~19.5 KB, ANALYTIC, not measured**: 4 registries x 32
entries x (~136 B IdStr + <=16 B payload), plus the slot ref and counters. (The
sizeof probe could not be run: the crate's TEST target's fingerprint refuses to
rebuild while the lib target rebuilds fine — noted, not chased. The bound is
arithmetic over the field types and cannot be off by enough to change the
decision below.)

Two consequences, one immediate:

* **INC-1 itself changed the heap profile TODAY.** `Arc::new(ComponentCell)`
  used to allocate a small struct whose four `Vec`s grew on demand;
  it now allocates the full worst-case ~20 KB per component up front. On the
  128 KiB bare-metal arena, four components is ~80 KiB — the campaign's own
  issue-0810 shape ("sized by worst-case shape") introduced by the campaign.
  `CELL_REG_CAP = 32` (borrowed from the metadata twin) is per-PLAN-shaped, not
  per-component-shaped, and is probably 4-8x too big for a typical component.
* **Static-izing cells multiplies it per CLASS.** The per-emit static cell —
  the planned Arc replacement — would bake ~20 KB x instances for every
  component class in the image, before any component registers.

So the Arc rework is BLOCKED on a sizing decision, and it is the same decision
issue 0827 poses one layer down ("pools sized at the backend"):

1. **Per-class exact sizing via the macro.** Codegen knows each class's entity
   count; the macro already monomorphises the trampolines per class, so it can
   emit a per-class cell type with exactly-sized registries. Costs: the cell
   stops being ONE concrete type, so `dispatch_into_cell` and friends go
   generic and the tick/drop trampolines must be emitted per class (they can
   be — they are macro output already). Zero waste; the FFI boundary stays
   `*mut c_void`.
2. **Shrink `CELL_REG_CAP` to a per-component default (4-8) + knob.** One-line,
   uniform, keeps the cell concrete; still worst-case-shaped, just a smaller
   worst case.
3. **Registries as slices into a runtime-owned pool** (the entity registries
   join the slot pool). Exact-ish, keeps one cell type, adds carve complexity.

**(2) LANDED (2026-08-28):** `CELL_REG_CAP` is now `config::MAX_CELL_ENTITIES`
(knob `NROS_RUNTIME_MAX_CELL_ENTITIES`, default **8**, per KIND), verified to
propagate (a build with 13 emitted 13). The analytic cell drops from ~19.5 KB
to **~4.9 KB** — still worst-case-shaped, 4x smaller worst case. (1) remains
the honest endpoint for the static-cell design and stays with review: the trade
is RAM vs a per-class generic cell.

The Box-leaked ctxs ride the same fork: their storage lands wherever the cells
do, and per-class emission sizes them exactly too (a class with no actions pays
zero action-ctx bytes).

### W5 endgame design (2026-08-28) — per-class cells, drafted before implementation

**Measured motivation landed the same day as issue 0857:** the interim inline
registries cost CELL_REG_CAP × ~1.35 KiB per cell (`EmbeddedRawPublisher`
embeds a 1 KiB `TxArena`), still `Arc`'d onto the heap — the second component's
~17.5 KiB cell OOM'd the esp32 workspace entry's 48 KiB heap
(`test_esp32_workspace_entry_e2e`, red 3/3 solo, symbolized to
`listener_pkg::register`'s `Box::new_uninit`). Interim per-image knob
`NROS_RUNTIME_MAX_CELL_ENTITIES=2` on the fixture row took the cell to
~3.5 KiB and the test green; the exact-cells design below retires the
worst-case pad structurally.

Fork option (1) taken (sole-owner call, the campaign mandate): the macro emits a
PER-CLASS cell alongside the per-class slot store it already emits, with
registries sized exactly. Key structural facts, read from today's tree:

* `ComponentCell` is consumed through `&ComponentCell` at 9 `dispatch_into_cell`
  sites, `tick_one_cell`, and the enroll/drop trampolines' `*mut c_void`. The
  FFI boundary only ever sees the void pointer, so the cell may go generic
  WITHOUT touching any C signature — the "const generics only where other
  languages cannot see them" rule holds.
* The tick/drop trampolines are shared today (`component_tick_trampoline` at
  one address for every class). Per-class cells need per-class trampolines —
  which the macro can emit exactly as it emits the install trampoline, and the
  executor's `enroll_component(raw, tick, drop)` already takes them as
  ARGUMENTS, so the executor does not change at all.
* Shape: `ComponentCell` becomes `ComponentCell<const PUBS: usize, const SUBS:
  usize, const SVCS: usize, const ACTS: usize>` (or one `N` if the counts
  collapse cleanly); `TypedSlot<C>` and the cell fuse into the macro-emitted
  per-class static — which also absorbs the ctx slabs, so a class with no
  actions pays zero action-ctx bytes. `dispatch_into_cell` and friends take a
  thin non-generic view struct (`CellRef { slot: &RefCell<...>, regs: ... }`)
  or go generic; the VIEW keeps monomorphisation from multiplying the dispatch
  code per class, and is the preferred spelling.
* The dynamic path (`register_node`) keeps pool-backed cells at the knob caps —
  two cell layouts, ONE view type. The view is what `tick_one_cell` and the
  trampolines actually need, and it is what makes the split affordable.
* ~~`ComponentSlotStorage::take()`'s `fetch_add` must become load+store in the
  same pass~~ — VERIFIED UNNECESSARY (step 1): 0851's failure was zpico-alloc's
  `Atomic<T>` shim lacking `fetch_add` on riscv32imc at COMPILE time;
  `node_runtime` uses `portable_atomic::AtomicUsize`, whose riscv32imc polyfill
  provides RMW, and the esp32c3 workspace entry builds and registers through
  `take()` today. Leave it.

Sequencing: view-struct refactor first (inert, all tests must stay green), then
the macro emission of per-class cell+trampolines, then the Arc deletion, then
the ctx slabs, then `node_runtime` drops its `alloc` gate — the finish signal —
and W1/W4's heap-free image follows.

**Acceptance (unchanged):** an image that CALLS runtime code links at tier
`heap-free` and passes the W1 gate with `symbols read` well above 1. Three
probes have already passed that gate vacuously at `symbols read: 1`.



## Related, not owned here

- [issue 0812](../issues/archived/0812-publisher-loan-heap-allocates-per-loan.md) —
  `Box::new` per loan. As written, `lending` and `heap-free` are mutually
  exclusive for no inherent reason. Fixing it is a precondition for the loan
  API existing on the heap-free tier.
- [issue 0814](../issues/0814-lending-never-exercised-on-hardware.md) — the
  whole zero-copy surface is posix-test-only.
- Whether `heap` survives as a storage mode at all. Payload buffers staying
  static means no payload field needs it; the question is whether infrastructure
  use justifies keeping a mode nobody applies to messages.

## Issues homed here (survey 2026-09-03)
Every open issue was checked for a home phase; these had none, or were
mentioned here only in passing. A mention is not an owner — an issue with
no work item is an issue nobody is accountable for, which is the same shape
as a gate sitting in a lane no CI job runs. Each row is a work item: the issue
holds the evidence, the item is *close it*.

| issue | why it belongs here |
| --- | --- |
| [#0816](../issues/0816-no-alloc-claimed-but-unenforced.md) | the book promises no-alloc integrations and nothing checks the linked image |
| [#0827](../issues/archived/0827-unused-rmw-pools-dominate-static-ram.md) | static RAM is a property of the RMW, not of the node |
| [#0857](../issues/0857-cell-registry-inline-capacity-heap-regression.md) | ComponentCell's inline registries cost worst-case x biggest-payload heap per component |

