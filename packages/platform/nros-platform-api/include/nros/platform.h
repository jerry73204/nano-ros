#ifndef NROS_PLATFORM_H
#define NROS_PLATFORM_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * @file platform.h
 * @brief Canonical C ABI for the nros platform abstraction.
 *
 * RFC-0042 D1 / phase-241 wave B — this is THE single canonical platform header,
 * owned by `nros-platform-api` (the lowest crate, no deps). `nros-c` and
 * `nros-platform-cffi` re-export it, so neither package's consumers need the
 * other's include dir (it breaks the historical nros-c↔cffi header tangle, and
 * there is exactly one file named `<nros/platform.h>` — no include-order race).
 *
 * A platform implementor supplies the symbols declared here. Every nros binary
 * links exactly one platform implementation; resolution is at link time — no
 * runtime registration. Implementations may be any language with a C ABI; for
 * Rust platform crates, `nros-platform-cffi` re-exports the Rust impl as
 * `#[unsafe(no_mangle)] extern "C"` symbols matching the names below (its
 * `src/lib.rs` extern block is the hand-written mirror, guarded byte-for-byte by
 * `c_stub_platform.rs`).
 *
 * Companion to the canonical-C-ABI RMW vtable (`<nros/rmw_vtable.h>`); the
 * platform layer sits one tier below RMW.
 *
 * # Return-value conventions
 *
 *  - `int8_t` returns: `0` = success, non-zero = error.
 *  - Pointer returns: `NULL` = allocation failure or not-implemented;
 *    non-`NULL` is the resource handle.
 *  - `clock_*` / `time_*` returns are absolute / monotonic counters and
 *    never error. If the platform has no clock, return `0`.
 *
 * # Threading
 *
 * All symbols must be safe to invoke from any thread.
 * `mutex_*` / `condvar_*` must be safe under concurrent callers.
 * `mutex_rec_*` must support same-thread re-entry (zenoh-pico
 * re-enters the same mutex).
 *
 * RTOS yields (`yield_now`) are **not** ISR-safe. Bare-metal yields
 * built on `core::hint::spin_loop()` are.
 */

/* ---- Platform capability defaults (RFC-0042 D2 / phase-241 B + C) ----
 *
 * Platform-*constant* capabilities. The single source of truth for a board's
 * *variable* capabilities is its `nros-board.toml` `[board.capabilities]`, lowered
 * to `-DNROS_PLATFORM_HAS_MALLOC` (etc.) by the build (phase-241 C.2). These
 * blocks supply only the per-platform constants, and never override a `-D` the
 * board already set (every `#define` is `#ifndef`-guarded).
 *
 *  - `NROS_PLATFORM_HAS_MALLOC` gates the canonical `malloc`/`free` shim below.
 *    Absent → a TU using the nros-cpp heap containers fails to *compile* (not
 *    link) — the issue-0038 guard.
 *  - `NROS_PLATFORM_HAS_ATOMICS` is true on every supported target today.
 *
 * RTOS-config-derived capabilities are intentionally NOT defaulted here:
 *  - FreeRTOS heap ← `configSUPPORT_DYNAMIC_ALLOCATION` (FreeRTOSConfig.h),
 *  - Zephyr heap/mutex ← `CONFIG_HEAP_MEM_POOL_SIZE` / `CONFIG_MULTITHREADING`.
 *    Those boards declare `heap = true` in board.toml and the build lowers it to
 *    the Kconfig/FreeRTOSConfig knob, keeping the C view tied to the RTOS config.
 */
/* HAS_ATOMICS is a constant on every supported target. */
#ifndef NROS_PLATFORM_HAS_ATOMICS
#  define NROS_PLATFORM_HAS_ATOMICS
#endif

/* HAS_MALLOC by platform. POSIX (and the POSIX-mapped hosted platforms: NuttX,
 * ThreadX-linux, native) and the heap RTOSes (Zephyr, FreeRTOS) always run with
 * an allocator under nros — the generator always configures the RTOS heap
 * (`CONFIG_HEAP_MEM_POOL_SIZE`, `configSUPPORT_DYNAMIC_ALLOCATION`), and each
 * declares `heap = true` in its `nros-board.toml`. They get the canonical
 * malloc/free unconditionally here.
 *
 * Bare-metal / ThreadX-RV64 / ESP / custom do NOT default a heap: they opt in
 * via the board.toml-derived `-DNROS_PLATFORM_HAS_MALLOC` (phase-241 C.2), so a
 * genuinely heap-less board still fails to compile a heap container (the #38
 * compile-gate). */
/* Every hosted/RTOS platform has a heap (POSIX, Zephyr, FreeRTOS, ThreadX-linux,
 * NuttX, and the default/unspecified case). This mirrors the retired nros-c
 * dispatch, where any non-bare-metal platform fell through to `posix.h`'s
 * unconditional malloc. Only bare-metal (incl. ThreadX-RV64 / ESP, which map to
 * `NROS_PLATFORM_BAREMETAL`) withholds the heap and opts in via the
 * board.toml-derived `-DNROS_PLATFORM_HAS_MALLOC` (phase-241 C.2 / the #38 gate). */
#if !defined(NROS_PLATFORM_BAREMETAL)
#  ifndef NROS_PLATFORM_HAS_MALLOC
#    define NROS_PLATFORM_HAS_MALLOC
#  endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Return codes ----
 *
 * phase-364 W1 (RFC-0076 D3). This vocabulary was declared and used by NO
 * port: 26 entry points returned a bare `int8_t` 0/-1, so "this platform
 * never does this" and "this failed just now" were the same value.
 *
 * That distinction is not academic. A caller may cache `UNSUPPORTED` forever
 * and MUST retry `NOMEM`: issue 0246 is a transient `pthread_create` failure
 * on NuttX under load, and phase-359 W10's worker pool — unable to ask — caches
 * every refusal, disabling a priority level for the process on what may have
 * been a momentary shortage.
 *
 * `nros_platform_ret_t` is `int8_t` because that is the width the functions
 * already return; it was `int32_t` and, having no users, cost nothing to
 * narrow. Every code below is non-zero, so a caller testing `!= 0` — which is
 * every caller today — is unaffected by ports becoming more specific.
 *
 * The values are written WITHOUT a cast on purpose. `((nros_platform_ret_t) 0)`
 * is the more careful C, and it is why bindgen emitted none of these: it cannot
 * evaluate a cast into a constant, so `nros-platform-cffi` carried a
 * hand-written Rust copy of all three — a mirror of exactly the kind
 * `check-ffi-struct-mirrors` exists to prevent, kept in step by nothing. Bare
 * integer literals cross the generator, so there is now one definition. */

typedef int8_t nros_platform_ret_t;

/** Operation completed successfully. */
#define NROS_PLATFORM_RET_OK              0
/** Generic failure not covered by a more specific code. */
#define NROS_PLATFORM_RET_ERROR           -1
/** The platform does not implement this operation — and never will, in this
 *  build. A single-threaded bare-metal port has no tasks; a port without an
 *  ISR-safe signal has none. Cacheable: asking again cannot change the answer,
 *  so a caller may record it once and stop trying. */
#define NROS_PLATFORM_RET_UNSUPPORTED     -5
/** A resource was exhausted NOW — kernel heap, task slot, handle table. The
 *  operation is supported and may succeed later. NOT cacheable: a caller that
 *  treats this as permanent turns a momentary shortage into a dead feature
 *  (issue 0246). */
#define NROS_PLATFORM_RET_NOMEM           -6
/** The caller passed something impossible — a NULL where storage is required,
 *  a zero-sized stack, an out-of-range priority. Retrying unchanged cannot
 *  help; this is a bug in the caller, not a condition of the platform. */
#define NROS_PLATFORM_RET_INVALID         -7
/** A bounded wait reached its deadline without the event. Not a failure of the
 *  call — `nros_platform_wake_wait_ms` already returns `1` for this and keeps
 *  doing so; this code is for the operations that have no such convention. */
#define NROS_PLATFORM_RET_TIMEOUT         -8

/* ---- Clock (monotonic) ---- */

/** Monotonic nanoseconds since a platform-defined epoch (boot, program
 *  start, …). Never decreases. Wraps after ~584 years.
 *
 *  Must be backed by a hardware counter or the OS tick — never by a
 *  software counter that only advances when polled.
 *
 *  Available immediately after platform init, before any other nros
 *  subsystem. SHOULD be callable from an ISR; a port whose clock is not
 *  ISR-safe must say so in its port documentation.
 *
 *  RFC-0073: this replaced the former `clock_ms` / `clock_us` pair. Ports
 *  that can convert without a runtime division should — where the counter
 *  frequency divides 1e9 (25/50/100/125/200/250 MHz) a compile-time
 *  ns-per-cycle multiply is ~2.5x cheaper than the divide it replaces. */
uint64_t nros_platform_clock_ns(void);

/** Granularity of `nros_platform_clock_ns`, in nanoseconds: the smallest
 *  non-zero difference two successive reads can report.
 *
 *  Examples: 1000000 for a 1 kHz tick, 40 for a 25 MHz cycle counter,
 *  1000 for a microsecond hardware timer.
 *
 *  Must be non-zero, and constant for the lifetime of the program after
 *  platform init. A port whose underlying rate is only known at runtime
 *  returns the resolved value; one whose rate can change under it returns
 *  the COARSEST value it may exhibit. There is no "unknown" encoding — a
 *  port that cannot answer honestly is reporting a clock it cannot
 *  honestly offer. */
uint64_t nros_platform_clock_resolution_ns(void);

/* RFC-0073 / phase-352 W6 — `nros_platform_clock_ms` and
 * `nros_platform_clock_us` are RETIRED. They were kept for one release as
 * `static inline` wrappers behind NROS_PLATFORM_LEGACY_CLOCK_UNITS; both
 * the wrappers and the escape hatch are now gone. Callers divide
 * `nros_platform_clock_ns()` themselves — and a caller that hand-declares
 * either retired name is caught by
 * `scripts/check-retired-platform-clock-symbols.py` (issue #555). */

/* ---- Clock (wall-clock epoch) ---- */

/** Microseconds since the UNIX EPOCH (1970-01-01T00:00:00Z), or `0` when
 *  this platform has no wall-clock source.
 *
 *  READ THIS BEFORE REACHING FOR `nros_platform_clock_ns` (issue 0758).
 *  The two clocks in this header differ by one word in their names and by
 *  the only property that matters for interop:
 *
 *    - `nros_platform_clock_ns` is MONOTONIC and boot-relative. Use it for
 *      durations, deadlines, spin gaps, timeouts — anything comparing two
 *      readings from THIS image. It is meaningless to a peer.
 *    - `nros_platform_epoch_us` is ABSOLUTE. Use it for message stamps and
 *      anything a peer will compare against its own clock.
 *
 *  Reaching for the wrong one does not fail to build and does not fail
 *  locally; it fails at a peer, which is the expensive place to find out.
 *  The concrete case is the consumer this exists for: an embedded island
 *  stamped control commands from its boot epoch and Autoware's
 *  `vehicle_cmd_gate` rejected every one as stale, so autonomous mode could
 *  never actuate.
 *
 *  `0` MEANS "NO WALL CLOCK", not "the epoch". Per this header's clock rule
 *  (see the top of file: "If the platform has no clock, return `0`") this
 *  never errors. A caller that gets `0` knows the image cannot stamp
 *  absolute time and should keep publishing boot-relative stamps knowingly,
 *  rather than publishing a confidently wrong absolute one. 1970 is not a
 *  plausible reading, so the sentinel costs no real value.
 *
 *  Not required to be monotonic: a platform that acquires its epoch after
 *  boot (SNTP, RTC handoff) will JUMP when it does, and may jump backwards.
 *  Callers needing monotonicity use `nros_platform_clock_ns`.
 *
 *  Need not be ISR-safe, and unlike `nros_platform_clock_ns` need not be
 *  available immediately after platform init — a port that acquires its
 *  epoch over the network necessarily answers `0` until it has. */
uint64_t nros_platform_epoch_us(void);

/* ---- Allocation ---- */

/** Allocate `size` bytes; return `NULL` on failure. May be called from
 *  any thread. */
void *nros_platform_alloc(size_t size);

/** Resize the block at `ptr` to `size` bytes. Equivalent to libc
 *  `realloc`: `NULL` ptr → fresh alloc; `0` size → free + return `NULL`.
 *  Preserves contents up to `min(old, new)`. */
void *nros_platform_realloc(void *ptr, size_t size);

/** Free a previously allocated block. `NULL` is a no-op. */
void nros_platform_dealloc(void *ptr);

/* Canonical `malloc`/`free` C-ABI surface (issue-0038). nros-cpp's heap
 * containers (`heap_string.hpp`, `heap_sequence.hpp`) allocate through
 * `nros_platform_malloc` / `nros_platform_free` so C and C++ share one
 * allocator. Defined ONCE here (replacing the former 5 per-header copies) as a
 * thin forward to the platform `alloc`/`dealloc` funnel (RFC-0034 D6), GATED on
 * the heap capability: a board without `NROS_PLATFORM_HAS_MALLOC` does not get
 * the canonical malloc/free, so using a heap container on a heap-less board is a
 * compile error (the 241.A gate), not a latent link failure. */
#ifdef NROS_PLATFORM_HAS_MALLOC
static inline void *nros_platform_malloc(size_t size) {
    return nros_platform_alloc(size);
}

static inline void nros_platform_free(void *ptr) {
    nros_platform_dealloc(ptr);
}
#endif /* NROS_PLATFORM_HAS_MALLOC */

/** Bytes currently allocated from the platform heap, or `0` if the port
 *  does not instrument it. Phase 230 / RFC-0034 D7: the true unified figure
 *  where the platform owns one kernel heap shared by the C side and the
 *  Rust `#[global_allocator]`. */
size_t nros_platform_heap_used_bytes(void);

/** Smallest number of bytes ever left unused on the CALLING task's stack, or
 *  `0` if this port cannot report it.
 *
 *  HEADROOM, not usage, because that is what both kernels natively track and
 *  it is the number a safety argument needs: how close the worst observed
 *  excursion came to the end of the stack.
 *
 *  The heap has had `nros_platform_heap_used_bytes` since RFC-0034 D7; the
 *  stack had nothing, and stack overflow is the classic way one component
 *  corrupts another's state. ISO 26262 treats spatial freedom from
 *  interference as a first-class requirement and AUTOSAR pairs memory
 *  protection with stack monitoring for exactly this reason. Without a
 *  portable probe the only recourse is a per-RTOS one: this repo's Zephyr
 *  lane greps `thread_analyzer` printk output in CI, which is a text scrape
 *  of a debug facility standing in for a platform capability, and reports
 *  nothing on any other port.
 *
 *  SELF only, deliberately. Both kernels answer for the calling task with no
 *  handle (`uxTaskGetStackHighWaterMark(NULL)`, `k_thread_stack_space_get`
 *  with `k_current_get()`), whereas answering for an arbitrary task needs a
 *  native handle this ABI does not carry -- on Zephyr the task storage is a
 *  `pthread_t` and the mapping to `k_thread *` is not public. A task
 *  reporting its own headroom is also the shape the callers want: each tier
 *  says how much of its own stack it has ever needed.
 *
 *  `0` means "this port does not instrument it", matching
 *  `nros_platform_heap_used_bytes`. It is not a claim that the stack is
 *  full. */
size_t nros_platform_task_stack_unused_bytes(void);

/** Total managed heap size in bytes (used + free), or `0` if unknown. */
size_t nros_platform_heap_total_bytes(void);

/* ---- Atomics ---- */
/* RFC-0042 D1 / phase-243 — single portable atomic-bool pair (was per-RTOS
 * `static inline` in the retired nros-c sub-headers: posix `__atomic`, zephyr
 * `atomic_set`, freertos critical-section, bare-metal barrier). The `__atomic_*`
 * builtins lower to a plain load/store + acquire/release fence for a
 * naturally-aligned 1-byte `bool` on every target nros builds — no CAS /
 * A-extension needed (riscv32imc never builds nros-c). Used directly (not C11
 * `<stdatomic.h>`, which does not compile under g++). Header-only inline, so
 * there is no extern symbol + no Rust-mirror entry. */
static inline void nros_platform_atomic_store_bool(bool *ptr, bool value) {
    __atomic_store_n(ptr, value, __ATOMIC_RELEASE);
}

static inline bool nros_platform_atomic_load_bool(const bool *ptr) {
    return __atomic_load_n(ptr, __ATOMIC_ACQUIRE);
}

/* ---- Sleep ---- */

/** Sleep at least `us` microseconds. Spin if the platform clock has no
 *  sub-millisecond timer. */
void nros_platform_sleep_us(size_t us);

/** Sleep at least `ms` milliseconds. */
void nros_platform_sleep_ms(size_t ms);

/** Sleep at least `s` seconds. */
void nros_platform_sleep_s(size_t s);

/* ---- Cooperative yield ---- */

/** Voluntarily yield the current task / thread. On bare-metal,
 *  `core::hint::spin_loop()` is acceptable; on RTOSes use the native
 *  cooperative-yield primitive (`k_yield`, `vPortYield`,
 *  `tx_thread_relinquish`, `sched_yield`, …). RTOS yields are **not**
 *  ISR-safe. */
void nros_platform_yield_now(void);

/* ---- Random ---- */

/** Random `u8`. Cryptographically random where the platform has an
 *  entropy source; otherwise a seeded PRNG. Must be deterministic
 *  within a single test session for reproducibility. */
uint8_t  nros_platform_random_u8(void);
/** Random `u16`. See `random_u8` notes. */
uint16_t nros_platform_random_u16(void);
/** Random `u32`. See `random_u8` notes. */
uint32_t nros_platform_random_u32(void);
/** Random `u64`. See `random_u8` notes. */
uint64_t nros_platform_random_u64(void);
/** Fill `len` bytes at `buf` with random data. */
void     nros_platform_random_fill(void *buf, size_t len);

/* ---- Wall clock ---- */

/** Wall-clock nanoseconds since the Unix epoch, or `0` if the platform has
 *  no real-time clock.
 *
 *  ONE symbol for one fact, mirroring what RFC-0073 / phase-352 did for the
 *  monotonic clock. It replaced `time_now_ms` + `time_since_epoch_secs` +
 *  `time_since_epoch_nanos` (issue 0532 item 5).
 *
 *  Why the split had to go, beyond tidiness: the ABI spent ONE INSTANT over
 *  two symbols, and each call sampled the clock separately (the POSIX port
 *  issued its own `clock_gettime` in each). A second boundary landing between
 *  the two reads paired the OLD second with the NEW sub-second remainder — a
 *  timestamp that jumped a full second BACKWARDS, rarely and silently. Both
 *  `nros-core` and `nros-node` carried a bounded re-read loop to paper over
 *  it; a single read cannot tear, so those loops are gone.
 *
 *  `u64` ns spans ~584 years from 1970, so it also retires the `uint32_t`
 *  seconds field, which overflowed in 2106.
 *
 *  Ports convert from whatever they have — a port with only seconds returns
 *  `secs * 1000000000ULL`, and one with no RTC returns 0. Callers wanting
 *  milliseconds divide by 1000000; wanting a `(secs, nanos)` pair, divide and
 *  remainder by 1000000000. */
uint64_t nros_platform_time_now_ns(void);

/* Issue 0532 item 5 — `nros_platform_time_now_ms`,
 * `nros_platform_time_since_epoch_secs` and
 * `nros_platform_time_since_epoch_nanos` are RETIRED, the same way
 * phase-352 W6 retired `clock_ms` / `clock_us` rather than keeping
 * wrappers. A caller that hand-declares any of the three is caught by
 * `scripts/check-retired-platform-clock-symbols.py`. */

/* ---- Threading: tasks ---- */

/* ---- Task attributes ----
 *
 * phase-364 W3 (RFC-0076 D2). `attr` used to be an undefined `void *`: IGNORED
 * by the posix and zephyr ports, a PRIVATE struct in the freertos and esp-idf
 * ports, and MANDATORY on threadx (a `NULL` was a hard failure). Three struct
 * types, no shared definition, and therefore no portable way to ask for a stack
 * size — which is why phase-359 W7 wrote a bespoke C shim to spawn a NuttX tier
 * with a 64 KiB stack, and why W10's generic spawn passes `NULL` and would fail
 * on ThreadX.
 *
 * One type now, defined here, and `attr == NULL` means "every default" on
 * EVERY port. */
typedef struct {
    /** Task name for the kernel's own tables and crash dumps. `NULL` = the
     *  port's default. Ports whose kernel has no name concept ignore it. */
    const char *name;
    /** Minimum stack size in BYTES. `0` = the port's default.
     *
     *  A FLOOR, not an exact size (issue 0612). Every port has a minimum of its
     *  own — `PTHREAD_STACK_MIN`, `configMINIMAL_STACK_SIZE`, `TX_MINIMUM_STACK`
     *  — and those differ by an order of magnitude, and on POSIX differ by
     *  ARCHITECTURE (16384 on glibc/x86_64, 131072 on glibc/aarch64). So no
     *  portable caller can name a number that is legal everywhere, and a port
     *  that treats a small request as an error turns a reasonable ask into a
     *  dead capability: that is exactly how `Executor::signal_fd()` returned
     *  `NotInitialized` on every Linux host. A port raises this to its own
     *  minimum; it never lowers it, and never refuses for being too small.
     *
     *  Always bytes, never words: FreeRTOS's `xTaskCreate` takes words, and the
     *  private struct it replaced called the field `stack_depth` while ThreadX's
     *  identically-named field was bytes. The conversion belongs in the one port
     *  that needs it, not in every caller. */
    size_t stack_bytes;
    /** Caller-provided stack memory, or `NULL` to let the port obtain it.
     *
     *  ThreadX requires the stack from its caller; POSIX, FreeRTOS and ESP-IDF
     *  let the kernel allocate and ignore this. A port that needs memory and is
     *  given `NULL` obtains it itself and releases it in `task_free`. */
    void *stack_mem;
    /** Scheduling priority in the NORMALISED band: `0` = least urgent, larger
     *  = more urgent, `NROS_PLATFORM_PRIORITY_INHERIT` = keep the creating
     *  task's.
     *
     *  phase-364 W5. This was "platform-native", and the natives disagree: `0`
     *  is the HIGHEST priority on ThreadX and the LOWEST on FreeRTOS, while
     *  Zephyr runs lower-is-more-urgent with negatives reserved for cooperative
     *  threads. A tier priority is authored ONCE, in `system.toml`, and
     *  deployed to several of them — so the same number meant "run me first" on
     *  one board and "run me last" on another, with nothing in the ABI
     *  recording which convention a port used.
     *
     *  Each port maps this band onto its own range and documents the map at its
     *  `task_init`. Use `NROS_PLATFORM_PRIORITY_RAW(n)` to bypass the band when
     *  tuning one RTOS against its own documentation — that is a legitimate
     *  thing to do, and the band should not make it impossible. */
    int32_t priority;
    /** SMP core to pin to, or `-1` for unpinned. Ignored on single-core. */
    int8_t core;
    /** `NROS_PLATFORM_TASK_*` flags below. */
    uint8_t flags;
} nros_platform_task_attr_t;

/** The task is never joined; its resources are reclaimed when it exits. */
#define NROS_PLATFORM_TASK_DETACHED 0x01u

/* ---- Normalised priority band (phase-364 W5) ----
 *
 * `0` = least urgent … `NROS_PLATFORM_PRIORITY_MAX` = most urgent. A port maps
 * the band onto its native range; the map is documented at each port's
 * `task_init` and is the ONLY place a direction is decided.
 *
 * The band is deliberately small. It is a portable ORDERING, not a claim that
 * every RTOS has 256 usable levels — ThreadX ships 32 by default, FreeRTOS's
 * `configMAX_PRIORITIES` is commonly 5 to 32, and a map that pretended
 * otherwise would collapse distinct tiers onto one level without saying so. */
#define NROS_PLATFORM_PRIORITY_MIN      0
#define NROS_PLATFORM_PRIORITY_MAX      255
/** Keep the creating task's priority. */
#define NROS_PLATFORM_PRIORITY_INHERIT  INT32_MIN

/** Escape hatch: pass `n` to the kernel untouched, bypassing the band.
 *
 * Encoded as a large negative so it cannot collide with a band value, and so a
 * port that has not implemented the escape sees an out-of-band number rather
 * than a plausible-looking priority. */
#define NROS_PLATFORM_PRIORITY_RAW(n)   (-0x40000000 - (int32_t) (n))
/** True when `p` came from `NROS_PLATFORM_PRIORITY_RAW`. */
#define NROS_PLATFORM_PRIORITY_IS_RAW(p) \
    ((p) <= -0x40000000 && (p) != NROS_PLATFORM_PRIORITY_INHERIT)
/** The raw value `p` carries. Only valid when `IS_RAW(p)`. */
#define NROS_PLATFORM_PRIORITY_RAW_VALUE(p) (-0x40000000 - (int32_t) (p))

/** Fill `attr` with the defaults — equivalent to passing `NULL` to
 *  `task_init`.
 *
 *  Callers use this rather than a designated initialiser so that a field added
 *  to the struct later stays source-compatible for out-of-tree ports. */
void nros_platform_task_attr_init(nros_platform_task_attr_t *attr);

/** Spawn a new task. `task` is opaque caller-provided storage (size from
 *  `nros_platform_task_storage_size`); `attr` is a
 *  `nros_platform_task_attr_t *`, or `NULL` for every default; `entry` is the
 *  task entry point; `arg` is forwarded to `entry`.
 *
 *  Returns `NROS_PLATFORM_RET_OK`, or `INVALID` (a NULL where storage or an
 *  entry is required), `NOMEM` (resources exhausted now — retry may succeed) or
 *  `UNSUPPORTED` (this port has no tasks at all). */
int8_t nros_platform_task_init(void *task, void *attr,
                               void *(*entry)(void *), void *arg);

/** Block until `task` exits. Cleans up task storage on success. */
int8_t nros_platform_task_join(void *task);

/** Mark `task` as detached — its storage is reclaimed on exit without
 *  a join. */
int8_t nros_platform_task_detach(void *task);

/** Request `task` to terminate at the next cancellation point.
 *  Cooperative: a task that never reaches a cancel point will not stop. */
int8_t nros_platform_task_cancel(void *task);

/** Terminate the calling task immediately. Does not return. */
void nros_platform_task_exit(void);

/** Free task storage allocated by `task_init`. Called after `task_join`
 *  or `task_detach + exit`. */
void nros_platform_task_free(void **task);

/** Opaque-storage sizing for `task`, mirroring the wake primitive's
 *  probes below. Both are pure functions (no global state) and may be
 *  called before `nros_platform_task_init`.
 *
 *  phase-359 W10 — added because `task_init`'s "size determined by the
 *  implementor" had no way to ASK. A C caller can write
 *  `pthread_t t;` and pass `&t`; a Rust caller cannot, and hard-coding
 *  a size is precisely issue 0570 (Rust's 20-byte `pthread_attr_t` met
 *  NuttX's 56-byte one and smashed the caller's frame). The wake
 *  primitive already solved this the right way; tasks now match it. */
size_t nros_platform_task_storage_size(void);
size_t nros_platform_task_storage_align(void);
/** Opaque-storage sizing for the lock family, matching `wake` and `task`.
 *  Pure functions, callable before the corresponding `_init`. */
size_t nros_platform_mutex_storage_size(void);
size_t nros_platform_mutex_storage_align(void);
size_t nros_platform_mutex_rec_storage_size(void);
size_t nros_platform_mutex_rec_storage_align(void);
size_t nros_platform_condvar_storage_size(void);
size_t nros_platform_condvar_storage_align(void);

/* ---- Opaque-storage sizes, at COMPILE time ----
 *
 * phase-364 W2 (RFC-0076 D1). The probes above answer a caller that allocates
 * at runtime. They cannot answer one that embeds these objects BY VALUE —
 * zenoh-pico's `_z_mutex_t` is a type, and a function call cannot size an
 * array — so the same facts are also available as macros.
 *
 * These are UPPER BOUNDS over every supported port, not per-port exact sizes: a
 * single header cannot know which port will link, and a consumer that embeds by
 * value must reserve enough for the one that does. Each port asserts its own
 * type fits (`_Static_assert` in its `platform.c`), so a bound that stops being
 * true is a compile error in the port rather than a silent overrun in the
 * consumer.
 *
 * They replace a hand-computed table in `zpico-sys` that recorded other
 * platforms' sizes as `≈` values with a "2× safety margin", checked by nobody,
 * about structs that are Kconfig-dependent on at least two of those platforms —
 * issue 0570's shape. The numbers below are deliberately generous for the same
 * reason the old ones were; the difference is that they are now checked.
 *
 * A port may raise a bound by defining it before including this header. */
#ifndef NROS_PLATFORM_TASK_STORAGE_SIZE
/* 512. ThreadX is the large one, and the number here is MEASURED, not
 * estimated: `sizeof(TX_THREAD)` is **360** bytes on the 64-bit Linux port,
 * plus this port's owned-stack pointer and padding.
 *
 * The estimate it replaces is worth keeping visible. `zpico-sys` recorded
 * "ThreadX TX_THREAD ≈ 232" and sized its task storage at 256 — correct for a
 * 32-bit port and 128 bytes short on a 64-bit one. That table was hand-written
 * prose about another platform's struct, which is why phase-364 W2 replaced it
 * with probes and `_Static_assert`s; this bound was raised BECAUSE one of those
 * asserts failed on the first real ThreadX build, rather than because someone
 * re-read the comment. */
#  define NROS_PLATFORM_TASK_STORAGE_SIZE     512
#endif
#ifndef NROS_PLATFORM_MUTEX_STORAGE_SIZE
/* 256, not a tighter fit, and the reason is recorded rather than guessed:
 * `zpico-sys` bounded this at 64 B, met ThreadX's `TX_MUTEX` (~120 B with
 * ownership / inheritance / suspension-list fields), and silently corrupted the
 * neighbouring field — presenting as a HANG in `Executor::open` after the zenoh
 * handshake, because every mutex op on the in-band executor trampled a
 * neighbour. Over-reserving costs bytes; under-reserving costs that. */
#  define NROS_PLATFORM_MUTEX_STORAGE_SIZE    256
#endif
#ifndef NROS_PLATFORM_MUTEX_REC_STORAGE_SIZE
#  define NROS_PLATFORM_MUTEX_REC_STORAGE_SIZE 256
#endif
#ifndef NROS_PLATFORM_CONDVAR_STORAGE_SIZE
/* POSIX `pthread_cond_t` is 48 B; a composite RTOS condvar
 * (`TX_MUTEX + TX_SEMAPHORE + UINT`) is ~184. Same reasoning as the mutex. */
#  define NROS_PLATFORM_CONDVAR_STORAGE_SIZE  256
#endif
#ifndef NROS_PLATFORM_STORAGE_ALIGN
/* Every opaque type on every supported port is at most 8-byte aligned. */
#  define NROS_PLATFORM_STORAGE_ALIGN         8
#endif


/* ---- Threading: non-recursive mutex ---- */

int8_t nros_platform_mutex_init(void *m);
int8_t nros_platform_mutex_drop(void *m);
int8_t nros_platform_mutex_lock(void *m);
int8_t nros_platform_mutex_try_lock(void *m);
int8_t nros_platform_mutex_unlock(void *m);

/* ---- Threading: recursive mutex ---- */

/** Initialise a recursive mutex (same-thread re-entry permitted).
 *  Required by zenoh-pico. */
int8_t nros_platform_mutex_rec_init(void *m);
int8_t nros_platform_mutex_rec_drop(void *m);
int8_t nros_platform_mutex_rec_lock(void *m);
int8_t nros_platform_mutex_rec_try_lock(void *m);
int8_t nros_platform_mutex_rec_unlock(void *m);

/* ---- Threading: condition variables ---- */

int8_t nros_platform_condvar_init(void *cv);
int8_t nros_platform_condvar_drop(void *cv);
int8_t nros_platform_condvar_signal(void *cv);
int8_t nros_platform_condvar_signal_all(void *cv);

/** Phase 124.B.7.a — ISR-safe signal.
 *
 *  Callable from interrupt context. `nros_platform_condvar_signal`
 *  is NOT ISR-safe on every platform (POSIX `pthread_cond_signal`
 *  isn't on the async-signal-safe function list; RTOS condvar
 *  primitives often require thread context). Backends MUST use
 *  this variant when triggering from an ISR or POSIX signal handler.
 *
 *  Per-platform implementation:
 *  * POSIX: `pipe` write — async-signal-safe; a runtime worker thread
 *    forwards to the underlying condvar. (Linux may use `eventfd`
 *    instead, which is NOT POSIX — `signalfd`/`eventfd` are Linux
 *    syscalls, which is why `nros-node`'s worker is
 *    `target_os = "linux"`-gated.)
 *  * Zephyr: `k_sem_give` on the wake semaphore (ISR-safe).
 *  * FreeRTOS: `xSemaphoreGiveFromISR` + `portYIELD_FROM_ISR` on
 *    the wake semaphore.
 *  * NuttX: `sem_post` (POSIX-safe under NuttX) on the wake sem.
 *  * ThreadX: `tx_event_flags_set` on the wake event flag group
 *    (ISR-safe).
 *  * Bare-metal: atomic flag store + `__SEV()` (Cortex-M).
 *
 *  Returns non-zero on error (e.g. ISR-unsafe call on a backend
 *  that mandates ISR-context-only via a separate primitive).
 *  Backends without an ISR-safe path return non-zero so callers
 *  can fall back to thread-context signal (with the obvious
 *  latency cost). */
int8_t nros_platform_condvar_signal_from_isr(void *cv);

/** Atomically release `m` and block on `cv`. The mutex is re-acquired
 *  before this function returns.
 *
 *  **NOT REAL-TIME — this wait is UNBOUNDED.** Every port implements it with
 *  its forever spelling (`TX_WAIT_FOREVER`, `portMAX_DELAY`, `K_FOREVER`), so
 *  a caller that blocks here has no deadline and no way to be woken by one.
 *  Nothing on the executor path calls it, and nothing new should: the
 *  executor's own wait uses the BOUNDED `nros_platform_wake_wait_ms`, and a
 *  condvar caller that needs a bound has `condvar_wait_until` directly below.
 *
 *  Kept rather than deleted because removing an exported symbol breaks
 *  out-of-tree ports, and marked here rather than left neutral because
 *  `condvar_wait` is the OBVIOUS spelling — the next port reaches for it and
 *  loses the bound silently. `scripts/check-no-unbounded-condvar-wait.sh`
 *  enforces the "nothing new" half; see issue 1196 and phase-436 W5.
 *
 *  "No unbounded wait" is only a property of the system if it is a property of
 *  the API. */
int8_t nros_platform_condvar_wait(void *cv, void *m);

/** Like `condvar_wait`, but with an absolute monotonic deadline in
 *  MILLISECONDS on the `nros_platform_clock_ns()` epoch — i.e.
 *  `clock_ns() / 1000000`. Returns non-zero on timeout.
 *
 *  Spelt out because this said "`clock_ms` units" after RFC-0073 /
 *  phase-352 W6 retired `nros_platform_clock_ms`: it named a function
 *  that no longer exists, leaving a port author no way to resolve the
 *  unit from this header. The unit itself never changed — every port
 *  names the parameter `abstime_ms` and the Rust trait says
 *  milliseconds — so this is the wording catching up, not an ABI
 *  change. */
int8_t nros_platform_condvar_wait_until(void *cv, void *m, uint64_t abstime);

/* ---- Threading: wake primitive (Phase 130) ----
 *
 * Binary-semaphore-shaped primitive used by the executor's wake_flag /
 * spin_once cv-wait pair. Separate from `nros_platform_condvar_*` so
 * the executor doesn't inherit zenoh-pico's pthread-shaped Zephyr
 * ABI (which on libc hangs past `pthread_cond_timedwait` deadlines).
 *
 * Per-platform impl:
 *  * POSIX:    `sem_t` with `sem_timedwait` (`CLOCK_MONOTONIC`).
 *  * Zephyr:   `k_sem` (kernel-native, libc-pthread-free).
 *  * FreeRTOS: `xSemaphoreCreateBinary` + `xSemaphoreGiveFromISR`.
 *  * NuttX:    POSIX `sem_t` via NuttX libc (`sem_timedwait`).
 *  * ThreadX:  `tx_semaphore` (ISR-safe `tx_semaphore_put`).
 *  * bare:     atomic flag + busy-spin against the platform clock.
 *
 * `wait_ms` returns 0 on signal, 1 on timeout, -1 on error.
 * `signal_from_isr` returns -1 if the backend has no ISR-safe path;
 * callers may fall back to `signal()` accepting the latency cost.
 * Storage is opaque to the caller; size/alignment via the probe
 * helpers below. */
int8_t  nros_platform_wake_init(void *w);
int8_t  nros_platform_wake_drop(void *w);
int8_t  nros_platform_wake_wait_ms(void *w, uint32_t timeout_ms);
int8_t  nros_platform_wake_signal(void *w);
int8_t  nros_platform_wake_signal_from_isr(void *w);

/** Opaque-storage sizing. Both helpers are pure functions (no global
 *  state) and may be called before `nros_platform_wake_init`. */
size_t  nros_platform_wake_storage_size(void);
size_t  nros_platform_wake_storage_align(void);

/* ---- Critical section ---- */
/* Phase 121.9 — global mutual exclusion against preemption + ISR
 * delivery. Backs the Rust `critical_section::Impl` registration used
 * by DDS, nros-rmw-{xrce,zenoh}, and any consumer of
 * `critical_section::with()`. Reentrant by contract: every acquire
 * must be paired with exactly one release, and the platform handles
 * nesting (PRIMASK already stacks; pthread side uses a recursive
 * mutex).
 *
 * The `uint32_t` token holds whatever the platform needs to restore
 * the prior posture — Cortex-M PRIMASK bit, Cortex-R CPSR I-bit,
 * RISC-V `mstatus.MIE` snapshot, pthread depth counter — and is
 * opaque to the caller. */
uint32_t nros_platform_critical_section_acquire(void);
void     nros_platform_critical_section_release(uint32_t token);

/* ---- Logging (Phase 88) ---- */
/* Per-platform leveled log delivery, matching the post-Phase-129
 * pattern: portable facade (`nros-log`) formats into a buffer and
 * hands the rendered text to whichever `nros-platform-<rtos>` is
 * linked in. POSIX writes to stderr; Zephyr routes through
 * `LOG_MODULE_DECLARE(nros)` (or `printk` fallback); ESP-IDF goes
 * through `esp_log_write`; NuttX through `syslog`; FreeRTOS /
 * ThreadX / bare-metal expose a `register_log_writer(...)` helper
 * in their platform crate so boards register the actual writer
 * (UART / semihosting / defmt / RTT) once at startup.
 *
 * `severity`: matches `nros_log::Severity::as_u8()`:
 *   0 = Trace, 1 = Debug, 2 = Info, 3 = Warn, 4 = Error, 5 = Fatal.
 * Implementors should map onto the platform's nearest level.
 *
 * `name_ptr` / `name_len`: logger name (NOT null-terminated; UTF-8).
 *   May be empty (the catch-all logger).
 * `msg_ptr`  / `msg_len`:  already-formatted message body (NOT null-
 *   terminated; UTF-8). Implementors that need a `CStr` must copy
 *   into their own buffer + append `'\0'`.
 *
 * No return value: log delivery never fails from the caller's POV.
 * Platforms drop silently on internal overflow (e.g. RTT ring full).
 *
 * `nros_platform_log_flush`: best-effort drain of any internal
 * buffer (RTT, syslog buffered chan, etc.). Default impl = no-op.
 *
 * Thread / ISR safety is documented per platform — see the table
 * in `docs/roadmap/phase-88-nros-log.md`. The ABI itself is
 * synchronous + reentrant-safe at the caller layer (a recursion
 * guard in `nros-log` prevents fan-out re-entry). */
void nros_platform_log_write(
    uint8_t        severity,
    const uint8_t *name_ptr, uintptr_t name_len,
    const uint8_t *msg_ptr,  uintptr_t msg_len);
void nros_platform_log_flush(void);

/* Board-supplied writer hook. ONLY meaningful on platforms whose
 * `nros_platform_log_write` impl is itself a thin dispatcher to a
 * board-registered fn (FreeRTOS, ThreadX, bare-metal). On platforms
 * with a native logger (POSIX, Zephyr, ESP-IDF, NuttX), the symbol
 * is absent and the board should not link against it.
 *
 * Board crates call this ONCE at startup, before any task / thread
 * begins logging. Re-calling replaces the registration.
 *
 * `writer` matches the signature of `nros_platform_log_write`.
 * `flusher` is optional (pass NULL for fully-synchronous writers). */
typedef void (*nros_platform_log_writer_fn_t)(
    uint8_t        severity,
    const uint8_t *name_ptr, uintptr_t name_len,
    const uint8_t *msg_ptr,  uintptr_t msg_len);

typedef void (*nros_platform_log_flush_fn_t)(void);

void nros_platform_register_log_writer(
    nros_platform_log_writer_fn_t writer,
    nros_platform_log_flush_fn_t  flusher);

/*
 * Runtime locator override (nano-ros #166 / phase-286 W1).
 *
 * Returns a middleware locator to use INSTEAD of the build-time-baked default
 * (`CONFIG_NROS_ZENOH_LOCATOR` / `option_env!("NROS_LOCATOR")`), or NULL to keep
 * the bake. Only the native_sim/native_posix platform impl returns non-NULL
 * (from `-testargs --nros-locator=<loc>`); every real-embedded impl returns
 * NULL. Entry points (Rust `zephyr_component_main!`, the C/C++ entries) prefer
 * this when non-NULL so native_sim tests can each dial a distinct per-test
 * zenohd, retiring the shared-baked-port serialization.
 *
 * The returned pointer is owned by the platform (argv-backed) and valid for the
 * process lifetime; the caller must not free it.
 */
const char* nros_runtime_locator_override(void);

/* ---- Fatal error (phase-366 / RFC-0077) ----
 *
 * The image's one ending. Rust's `#[panic_handler]`, a C precondition failure
 * and a C++ terminate all arrive here, so an operator debugging a board has one
 * place to look instead of four.
 *
 * Before this existed the ABI could express ALLOCATION but not fatality, so each
 * language invented its own: `nros-c` spun silently, `nros-board-nuttx` printed
 * and exited 1, the ThreadX RV64 board wrote UART and exited QEMU, the FreeRTOS
 * board did semihosting + `bkpt` + spin. Four reasonable answers to four
 * different questions, none of them the image's to choose.
 *
 * `msg` is a DIAGNOSTIC, not a C string: `len` bytes, no NUL required, possibly
 * empty (`msg` may be NULL only when `len == 0`). It is not necessarily UTF-8 —
 * a port that forwards to a text sink should treat it as bytes.
 *
 * Contract on the implementor:
 *   - MUST NOT return. Declared `_Noreturn` so a caller (and Rust's `-> !`) can
 *     rely on it.
 *   - MUST tolerate being called from ANY context — interrupt, scheduler
 *     locked, or before the kernel starts. That rules out anything needing a
 *     scheduler, which is why this is not simply "log at Fatal then halt".
 *   - SHOULD emit `msg` somewhere a human can read before terminating. A silent
 *     halt is the one behaviour no RTOS in this tree chose for itself.
 *
 * Spelled through `NROS_PLATFORM_NORETURN` because this header is included from
 * C++ too (`nros-cpp`), where `_Noreturn` is not a keyword — that mismatch is a
 * hard `'_Noreturn' does not name a type` in every C++ TU, which is how it was
 * found.
 *
 * Ports map it to the fatal path they already have (`k_panic`, `PANIC()`,
 * `esp_system_abort`, the FreeRTOS hook, `abort()`). A C/C++ image overrides the
 * port's definition by defining this symbol strongly; a Rust image declares its
 * own `#[panic_handler]` in the entry package. Either way the choice belongs to
 * whoever builds the image — RFC-0077.
 */
#if defined(__cplusplus)
#  define NROS_PLATFORM_NORETURN [[noreturn]]
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#  define NROS_PLATFORM_NORETURN _Noreturn
#else
#  define NROS_PLATFORM_NORETURN __attribute__((noreturn))
#endif

NROS_PLATFORM_NORETURN void nros_platform_panic(const char *msg, size_t len);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* NROS_PLATFORM_H */
