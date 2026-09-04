/*
 * Phase 121.3.zephyr — native C implementation of the canonical
 * platform ABI for Zephyr RTOS.
 *
 * Behavioural parity with `nros-platform-zephyr`'s Rust impl. The
 * Rust port had to go through C shims for Zephyr's static-inline
 * macros (`k_uptime_get`, `k_msleep`, `k_yield`, …); the native C
 * port can call them directly.
 *
 *   - Clock    — k_uptime_get() returns int64_t milliseconds since
 *                boot; us from the 64-bit cycle counter where the board
 *                provides one, else from the tick clock (issue #531).
 *   - Alloc    — k_malloc / k_realloc / k_free against the kernel heap.
 *   - Sleep    — k_msleep / k_usleep / k_sleep.
 *   - Yield    — k_yield().
 *   - Random   — sys_rand32_get() / sys_rand_get(). Default Zephyr
 *                build provides a PRNG; CONFIG_ENTROPY_GENERATOR
 *                upgrades to hardware entropy.
 *   - Time     — wall clock unsupported unless the user enables
 *                CONFIG_RTC; defaults return 0.
 *   - Tasks    — pthread_create via the module's stack-provisioning shim.
 *   - Mutexes  — pthread_mutex_t handles, matching zenoh-pico's Zephyr ABI.
 *   - Condvars — pthread_cond_t handles, matching zenoh-pico's Zephyr ABI.
 *
 * Build verification requires a Zephyr workspace; CMakeLists.txt
 * is designed to be consumed as a Zephyr module or as an external
 * project linked via the Zephyr interface library.
 */

#include <nros/platform.h>

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#ifdef CONFIG_POSIX_API
#include <zephyr/posix/pthread.h>
#include <zephyr/posix/sched.h>
#endif

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* ---- Clock ---- */

/* RFC-0073 — nanoseconds, from whichever source this board actually has.
 *
 * Issue #531: `k_cycle_get_64()` returns 0 unless
 * CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER is set (its __ASSERT compiles out in
 * release), and the Cortex-M SysTick driver only selects that symbol
 * `default y if (SYS_CLOCK_HW_CYCLES_PER_SEC > 60000000)`. So the cycle
 * counter is used only where the board provides one; everywhere else the
 * tick clock, which every board has. `IS_ENABLED` rather than `#ifdef` so
 * both arms keep compiling everywhere. */
uint64_t nros_platform_clock_ns(void) {
    if (IS_ENABLED(CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER)) {
        return (uint64_t) k_cyc_to_ns_floor64(k_cycle_get_64());
    }
    return (uint64_t) k_ticks_to_ns_floor64(k_uptime_ticks());
}

uint64_t nros_platform_clock_resolution_ns(void) {
    if (IS_ENABLED(CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER)) {
        /* One cycle. `sys_clock_hw_cycles_per_sec()` rather than the
         * Kconfig constant because a driver may only know its frequency
         * at runtime (CONFIG_TIMER_READS_ITS_FREQUENCY_AT_RUNTIME). */
        const uint64_t hz = (uint64_t) sys_clock_hw_cycles_per_sec();
        if (hz != 0U) {
            const uint64_t ns = 1000000000ULL / hz;
            return ns == 0U ? 1U : ns;
        }
    }
    return (uint64_t) k_ticks_to_ns_floor64(1);
}

/* issue 0758 — the Zephyr wall-clock epoch, acquired once over SNTP.
 *
 * WHY AN OFFSET AND NOT A QUERY PER CALL. `epoch_us` is on the message-stamp
 * path, so a network round trip per call is not an option — and SNTP's own
 * accuracy is worse than the monotonic clock's between acquisitions anyway. We
 * take ONE reading, subtract the monotonic clock at that instant to get a fixed
 * offset, and derive every later answer from `nros_platform_clock_ns()`. That
 * makes reads cheap, monotonic between acquisitions, and free of any failure
 * mode the network has.
 *
 * The jump the header warns about happens exactly once, when the offset is
 * installed: before it, callers get `0` (no wall clock) and stamp boot-relative
 * time knowingly; after it, absolute time. There is no window where a wrong
 * absolute value is published.
 *
 * Gated on CONFIG_SNTP so an image that does not want a network clock does not
 * link one, and answers `0` — which the header defines as "no wall clock here"
 * rather than an error. */
#ifdef CONFIG_SNTP
#include <zephyr/net/sntp.h>

/* Microseconds to add to `nros_platform_clock_ns()/1000` to get UNIX time.
 * Zero means "not acquired"; written once by the acquirer, read by every
 * `epoch_us` caller. `atomic` because the acquirer runs on the boot thread
 * while tiers may already be stamping. */
static atomic_t nros_zephyr_epoch_offset_lo = ATOMIC_INIT(0);
static atomic_t nros_zephyr_epoch_offset_hi = ATOMIC_INIT(0);

static uint64_t nros_zephyr_epoch_offset_get(void) {
    /* Read hi/lo twice and retry on a torn pair. A 64-bit offset does not fit
     * one atomic_t on 32-bit targets, and the value is written exactly once, so
     * a single retry is sufficient — this is not a general seqlock. */
    for (int attempt = 0; attempt < 2; attempt++) {
        uint32_t hi1 = (uint32_t) atomic_get(&nros_zephyr_epoch_offset_hi);
        uint32_t lo = (uint32_t) atomic_get(&nros_zephyr_epoch_offset_lo);
        uint32_t hi2 = (uint32_t) atomic_get(&nros_zephyr_epoch_offset_hi);
        if (hi1 == hi2) {
            return ((uint64_t) hi1 << 32) | (uint64_t) lo;
        }
    }
    return 0;
}

int nros_platform_epoch_acquire_sntp(const char *server, uint32_t timeout_ms);
int nros_platform_epoch_acquire_sntp(const char *server, uint32_t timeout_ms) {
    if (server == NULL || server[0] == '\0') {
        return -EINVAL;
    }
    struct sntp_time ts;
    int rc = sntp_simple(server, timeout_ms, &ts);
    if (rc != 0) {
        /* Leave the offset unset: the caller keeps getting 0 and keeps
         * stamping boot-relative time, which is the honest degradation. */
        return rc;
    }
    /* `fraction` is a 32-bit binary fraction of a second. Scale to us via a
     * 64-bit intermediate; >> 32 rather than / 2^32 so no division is emitted
     * on targets without one. */
    uint64_t frac_us = ((uint64_t) ts.fraction * 1000000ULL) >> 32;
    uint64_t now_us = ts.seconds * 1000000ULL + frac_us;
    uint64_t mono_us = nros_platform_clock_ns() / 1000ULL;
    uint64_t offset = now_us - mono_us;
    atomic_set(&nros_zephyr_epoch_offset_lo, (atomic_val_t) (uint32_t) offset);
    atomic_set(&nros_zephyr_epoch_offset_hi, (atomic_val_t) (uint32_t) (offset >> 32));
    return 0;
}

uint64_t nros_platform_epoch_us(void) {
    uint64_t offset = nros_zephyr_epoch_offset_get();
    if (offset == 0ULL) {
        return 0ULL; /* not acquired — no wall clock */
    }
    return offset + nros_platform_clock_ns() / 1000ULL;
}
#else  /* !CONFIG_SNTP */
/* No network clock compiled in. `0` is the honest answer, not a placeholder:
 * the header makes it mean "no epoch here", so a caller keeps stamping
 * boot-relative time knowingly rather than publishing a wrong absolute one. */
uint64_t nros_platform_epoch_us(void) {
    return 0;
}
#endif /* CONFIG_SNTP */

/* ---- Allocation ----
 *
 * phase-391 W3 — the funnel is backed by the rlsf arena in
 * `nros-platform/src/zephyr_heap.rs` (zpico-alloc's FreeListHeap, rlsf since
 * W2), not by `k_malloc`. Every image's Rust staticlib carries that module:
 * the C lane through nros-c's `platform-zephyr` feature, the Rust lane through
 * nros-platform directly. Once an image's conf also sets
 * `CONFIG_HEAP_MEM_POOL_SIZE=0`, `k_malloc`/`sys_heap_*` garbage-collect out
 * of the link — the wave's link test.
 *
 * The spinlock is load-bearing: FreeListHeap is single-threaded by contract
 * and Zephyr is not (zenoh-pico's read/lease tasks allocate concurrently with
 * the app). rlsf is O(1), so the critical section is short and bounded — which
 * is exactly why phase-391 chose it. The old `k_realloc` emulation also
 * memcpy'd the NEW size out of the old block (a documented best-effort
 * over-read); rlsf's own reallocate retires that too. */

extern void *nros_zephyr_heap_alloc(size_t size);
extern void *nros_zephyr_heap_realloc(void *ptr, size_t size);
extern void nros_zephyr_heap_free(void *ptr);

static struct k_spinlock nros_heap_lock;

void *nros_platform_alloc(size_t size) {
    if (size == 0) return NULL;
    k_spinlock_key_t key = k_spin_lock(&nros_heap_lock);
    void *out = nros_zephyr_heap_alloc(size);
    k_spin_unlock(&nros_heap_lock, key);
    if (out == NULL) {
        /* phase-8 diagnosis — arena exhaustion was completely silent. The
         * caller gets NULL and whatever it does next (retry, block, ignore)
         * is the only evidence, which is how a 64 KiB default cost a 9-step
         * bisect to attribute.
         *
         * printk, not LOG_*: the logging subsystem may itself allocate, and
         * this is the path that just failed to allocate. Emitted AFTER the
         * spinlock is released for the same reason -- never call into
         * anything reentrant while holding the funnel's lock. */
        extern size_t nros_zephyr_heap_capacity(void);
        /* issue 0968 — and WHO asked. The size alone is not actionable: three
         * zephyr xrce-cpp cells died on a byte-identical 427968-byte request
         * and nothing in the image, the Kconfig or the generated sizes header
         * accounted for it, so the only move left was to raise the knob until
         * it booted -- which buries the question instead of answering it.
         *
         * The return address costs nothing at runtime and turns the message
         * into a lead:
         *     arm-none-eabi-addr2line -f -e build/zephyr/zephyr.elf <caller>
         *
         * `__builtin_return_address(0)` is the immediate caller, which on this
         * path is the allocator shim rather than the code that wanted the
         * memory. That is still one frame closer than nothing, and it names
         * the SHIM so the reader knows which allocator to look behind. */
        printk("nros: HEAP EXHAUSTED: request %zu bytes, arena %zu bytes, "
               "caller %p\n"
               "      (addr2line -f -e zephyr.elf %p to name it; raise "
               "CONFIG_NROS_ZEPHYR_HEAP_SIZE / NROS_ZEPHYR_HEAP_SIZE only once "
               "you know what asked)\n",
               size, nros_zephyr_heap_capacity(),
               __builtin_return_address(0), __builtin_return_address(0));
    }
    return out;
}

void *nros_platform_realloc(void *ptr, size_t size) {
    k_spinlock_key_t key = k_spin_lock(&nros_heap_lock);
    void *out = nros_zephyr_heap_realloc(ptr, size);
    k_spin_unlock(&nros_heap_lock, key);
    return out;
}

void nros_platform_dealloc(void *ptr) {
    k_spinlock_key_t key = k_spin_lock(&nros_heap_lock);
    nros_zephyr_heap_free(ptr);
    k_spin_unlock(&nros_heap_lock, key);
}

/* ---- Heap stats (phase-230 Z5 / RFC-0034 D7, corrected phase-412) ----
 *
 * These report the arena the application ACTUALLY allocates from: the rlsf
 * heap in nros-platform's `zephyr_heap`, which `nros_platform_alloc` above
 * calls into and which therefore backs both zenoh-pico's `z_malloc` and
 * `__rust_alloc`. Its size is CONFIG_NROS_ZEPHYR_HEAP_SIZE.
 *
 * They used to answer from Zephyr's kernel heap `_system_heap`, sized by
 * CONFIG_HEAP_MEM_POOL_SIZE. That was right when phase-230 wrote it and wrong
 * from phase-391 W3 onward, which moved the funnel off the kernel heap --
 * `zephyr/Kconfig` says so under NROS_ZEPHYR_HEAP_SIZE ("CONFIG_HEAP_MEM_POOL_SIZE
 * does NOT govern application allocation any more; this does"), while this file
 * kept measuring the heap that no longer governs anything.
 *
 * The cost of that was not a wrong number, it was NO number: the one figure
 * that could size NROS_ZEPHYR_HEAP_SIZE reported a different arena, so the knob
 * could only ever be guessed. The island carries 94,208 bytes chosen that way.
 *
 * `peak`, not `used`, is what the knob wants -- `used` sampled at an arbitrary
 * instant reports whatever happened to be live at the moment of the read.
 * Requires nros-platform's `heap-stats` feature; returns 0 ("unknown")
 * otherwise, which is the convention this file already used. */
extern size_t nros_zephyr_heap_capacity(void);
extern size_t nros_zephyr_heap_used(void);
extern size_t nros_zephyr_heap_peak(void);

size_t nros_platform_heap_used_bytes(void) { return nros_zephyr_heap_used(); }

size_t nros_platform_heap_total_bytes(void) { return nros_zephyr_heap_capacity(); }

/* High-water mark, for sizing CONFIG_NROS_ZEPHYR_HEAP_SIZE from a measurement
 * rather than from a round number.
 *
 * NOT added to `nros/platform.h`: that header is the cross-port ABI, and every
 * other port would need a stub plus a regenerated cffi binding to carry a
 * figure only this port can produce. `nros_zephyr_heap_peak()` is reachable
 * directly by anything Zephyr-side that wants it. */
size_t nros_zephyr_platform_heap_peak_bytes(void) { return nros_zephyr_heap_peak(); }

/* ---- Sleep ---- */

void nros_platform_sleep_us(size_t us) {
    if (us == 0) return;
    k_usleep((int32_t) us);
}

void nros_platform_sleep_ms(size_t ms) {
    if (ms == 0) return;
    k_msleep((int32_t) ms);
}

void nros_platform_sleep_s(size_t s) {
    k_sleep(K_SECONDS((int32_t) s));
}

/* ---- Yield ---- */

void nros_platform_yield_now(void) {
    k_yield();
}

/* ---- Random ---- */

/* issue 0853 — a board with no hardware entropy produced the SAME zenoh id on
 * every boot.
 *
 * With `CONFIG_TEST_RANDOM_GENERATOR` (its own name says what it is) Zephyr
 * backs `sys_rand32_get` with a timer reading. The path from reset to the point
 * zenoh-pico draws its zid is deterministic, so that reading is the same number
 * every time, and the MR-CANHUBK344 came up as
 * `1322740661b45746fa29b1803f32f5eb` across resets, reflashes and power cycles.
 *
 * A zenoh id is the identity a router uses to hold a session's state for a
 * lease. A board that reboots into the same one is, to the router, the peer it
 * already has — so every reconnect-shaped measurement depends on router history
 * rather than on the firmware under test. That confounded a real A/B during
 * issue 0852.
 *
 * ONLY substituted when Zephyr's generator is the test stand-in. A board with
 * `CONFIG_ENTROPY_GENERATOR` has a real source and keeps using it — this must
 * not shadow good entropy with a PRNG.
 *
 * THIS IS NOT A CSPRNG AND MUST NOT BE USED AS ONE. It exists to make an
 * IDENTITY unique, not to make a secret unguessable. Anything needing
 * cryptographic randomness needs a real entropy source, which this part does
 * not have wired today. */
#if defined(CONFIG_TEST_RANDOM_GENERATOR) && !defined(CONFIG_ENTROPY_GENERATOR)

#include <zephyr/sys/util.h>

/* Survives a warm reset: Zephyr does not zero `.noinit`. That is the whole
 * trick for reset-to-reset uniqueness — the previous boot's state is still
 * here, and mixing it forward makes each boot differ from the last. At a COLD
 * start the same location holds whatever the SRAM powered up as, which differs
 * between parts and between power cycles. */
static uint32_t nros_rand_seed_carry __attribute__((section(".noinit")));
static uint32_t nros_rand_s[4];
static bool nros_rand_ready;

static uint32_t nros_rand_mix(uint32_t x) {
    /* splitmix32 — spreads a weak seed across all 32 bits before it becomes
     * PRNG state. Without this, two boots whose inputs differ in one low bit
     * produce visibly related streams. */
    x += 0x9e3779b9u;
    x = (x ^ (x >> 16)) * 0x21f0aaadu;
    x = (x ^ (x >> 15)) * 0x735a2d97u;
    return x ^ (x >> 15);
}

static void nros_rand_init(void) {
    /* Every independent thing this port can reach:
     *   - the carried `.noinit` word (previous boot, or cold-start SRAM noise)
     *   - a cycle counter, which is not constant across the reset paths
     *   - the uptime, for the case the cycle counter is coarse
     *   - the timer generator's own reading, which is at least not worse
     * None is a real entropy source and the comment above says so. Together
     * they satisfy what this issue asks: reset twice, get two ids. */
    uint32_t seed = nros_rand_seed_carry;
    seed = nros_rand_mix(seed ^ k_cycle_get_32());
    seed = nros_rand_mix(seed ^ (uint32_t) k_uptime_get_32());
    seed = nros_rand_mix(seed ^ sys_rand32_get());

    /* Carry a DIFFERENT value forward than the one seeding this boot, so a
     * reset that happens before any random is drawn still moves the state. */
    nros_rand_seed_carry = nros_rand_mix(seed ^ 0xa5a5a5a5u);

    for (int i = 0; i < 4; i++) {
        seed = nros_rand_mix(seed);
        nros_rand_s[i] = seed;
    }
    /* An all-zero xoshiro state is a fixed point and stays zero forever. */
    if ((nros_rand_s[0] | nros_rand_s[1] | nros_rand_s[2] | nros_rand_s[3]) == 0u) {
        nros_rand_s[0] = 0x9e3779b9u;
        nros_rand_s[1] = 0x243f6a88u;
        nros_rand_s[2] = 0xb7e15162u;
        nros_rand_s[3] = 0xdeadbeefu;
    }
    nros_rand_ready = true;
}

/* xoshiro128** — small, fast, and well distributed. Not cryptographic. */
static uint32_t nros_rand_u32(void) {
    if (!nros_rand_ready) {
        nros_rand_init();
    }
    const uint32_t r = nros_rand_s[1] * 5u;
    const uint32_t result = ((r << 7) | (r >> 25)) * 9u;
    const uint32_t t = nros_rand_s[1] << 9;
    nros_rand_s[2] ^= nros_rand_s[0];
    nros_rand_s[3] ^= nros_rand_s[1];
    nros_rand_s[1] ^= nros_rand_s[2];
    nros_rand_s[0] ^= nros_rand_s[3];
    nros_rand_s[2] ^= t;
    nros_rand_s[3] = (nros_rand_s[3] << 11) | (nros_rand_s[3] >> 21);
    return result;
}

#define NROS_RAND_U32() nros_rand_u32()

#else /* a real entropy source, or a generator this port should not second-guess */

#define NROS_RAND_U32() sys_rand32_get()

#endif

uint8_t  nros_platform_random_u8(void)   { return (uint8_t)  NROS_RAND_U32(); }
uint16_t nros_platform_random_u16(void)  { return (uint16_t) NROS_RAND_U32(); }
uint32_t nros_platform_random_u32(void)  { return NROS_RAND_U32(); }

uint64_t nros_platform_random_u64(void) {
    uint64_t hi = NROS_RAND_U32();
    uint64_t lo = NROS_RAND_U32();
    return (hi << 32) | lo;
}

void nros_platform_random_fill(void *buf, size_t len) {
#if defined(CONFIG_TEST_RANDOM_GENERATOR) && !defined(CONFIG_ENTROPY_GENERATOR)
    /* Byte-wise from the same stream, so `fill` cannot disagree with the
     * scalar draws about which generator is in use. Filling from
     * `sys_rand_get` here while the scalars used the seeded PRNG is exactly
     * how a zid ends up deterministic while everything else looks fine. */
    uint8_t *p = (uint8_t *) buf;
    size_t i = 0;
    while (i < len) {
        uint32_t v = nros_rand_u32();
        size_t n = (len - i) < 4u ? (len - i) : 4u;
        for (size_t k = 0; k < n; k++) {
            p[i + k] = (uint8_t) (v >> (8u * k));
        }
        i += n;
    }
#else
    sys_rand_get(buf, len);
#endif
}

/* ---- Wall clock — unsupported without CONFIG_RTC ---- */

/* No real-time clock on this port: 0 means "no wall clock", per the ABI. */
uint64_t nros_platform_time_now_ns(void)              { return 0; }

/* ---- Tasks ---- */

#ifdef CONFIG_POSIX_API

int nros_zephyr_task_create(pthread_t *thread,
                            void *(*entry)(void *),
                            void *arg);

void nros_platform_task_attr_init(nros_platform_task_attr_t *attr) {
    if (attr == NULL) {
        return;
    }
    memset(attr, 0, sizeof(*attr));
    attr->priority = INT32_MIN;
    attr->core = -1;
}

/* issue 0852 — the band -> native map for this port, and the ONLY place this
 * port decides a direction (per the `priority` contract in <nros/platform.h>).
 *
 * SCHED_RR, NOT SCHED_FIFO, and that is the load-bearing choice.
 *
 * Zephyr's POSIX layer splits the two policies across its two priority bands
 * (`lib/posix/options/pthread_sched.h`):
 *
 *     SCHED_FIFO           -> COOPERATIVE, max CONFIG_NUM_COOP_PRIORITIES - 1
 *     SCHED_RR/SCHED_OTHER -> PREEMPTIBLE, max CONFIG_NUM_PREEMPT_PRIORITIES - 1
 *
 * A cooperative thread is never preempted. The task this issue is about spends
 * its life in a `uart_poll_in` busy-poll, so making it cooperative would hand
 * it the CPU until it blocks — `k_yield()` from a coop thread does not drop
 * below its own priority, so the executor (preemptible) would not run at all
 * between frames. Trading a 20 ms starvation of the reader for an unbounded
 * starvation of everything else is not a fix. The posix port picks SCHED_FIFO
 * because on Linux that is simply "the real-time policy"; on Zephyr the same
 * constant means something different, which is exactly the per-port direction
 * decision this function exists to make.
 *
 * Within SCHED_RR the direction matches the band: higher is more urgent, and
 * `posix_to_zephyr_priority` inverts it into Zephyr's lower-is-more-urgent
 * numbering. So the map is a plain linear scale onto what the policy reports.
 *
 * A RAW value is clamped rather than passed through to fail. Zephyr validates
 * `sched_priority` against the policy's range and rejects anything outside it,
 * so an out-of-range raw number does not arrive as a loud error — it arrives as
 * a silently inherited priority, which is the very failure mode issue 0852 was.
 * `zpico_posix_set_priority` already clamps for the same reason.
 *
 * Returns < 0 for "no priority requested", read by the caller as "leave it
 * inherited". */
static int nros_zephyr_native_priority(int32_t priority) {
    if (priority == NROS_PLATFORM_PRIORITY_INHERIT) {
        return -1;
    }
#if defined(CONFIG_POSIX_PRIORITY_SCHEDULING)
    int lo = sched_get_priority_min(SCHED_RR);
    int hi = sched_get_priority_max(SCHED_RR);
    if (lo < 0 || hi <= lo) {
        return -1;
    }
    int want;
    if (NROS_PLATFORM_PRIORITY_IS_RAW(priority)) {
        want = (int) NROS_PLATFORM_PRIORITY_RAW_VALUE(priority);
    } else if (priority >= 0) {
        int32_t band =
            priority > NROS_PLATFORM_PRIORITY_MAX ? NROS_PLATFORM_PRIORITY_MAX : priority;
        want = lo + (int) (((int64_t) band * (hi - lo)) / NROS_PLATFORM_PRIORITY_MAX);
    } else {
        return -1;
    }
    if (want < lo) want = lo;
    if (want > hi) want = hi;
    return want;
#else
    /* No POSIX scheduling option in this image: nothing can be asked for, and
     * pretending otherwise would put us back to a silently dropped attribute. */
    (void) priority;
    return -1;
#endif
}

int nros_zephyr_task_create_prio(pthread_t *thread, void *(*entry)(void *), void *arg,
                                 int native_priority, const char *name);

int8_t nros_platform_task_init(void *task, void *attr,
                               void *(*entry)(void *), void *arg) {
    /* phase-364 W1 — see the posix port: INVALID for a caller-side
     * impossibility, NOMEM for a refused create (this port reaches Zephyr's
     * pthread layer, so it inherits EAGAIN-on-exhaustion semantics). */
    if (task == NULL || entry == NULL) return NROS_PLATFORM_RET_INVALID;

    /* phase-364 W3 — `attr` is accepted rather than ignored, but only the
     * fields this port can honour are read.
     *
     * Zephyr's native `k_thread_create` needs a `K_THREAD_STACK_DEFINE` region,
     * which carries MPU alignment requirements a caller cannot satisfy with a
     * plain allocation. This port therefore goes through Zephyr's POSIX layer,
     * where the stack comes from `CONFIG_PTHREAD_DYNAMIC_STACK` — so a
     * requested `stack_bytes` cannot be applied here and is deliberately not
     * silently pretended to be. A caller needing an exact Zephyr stack should
     * declare the thread in the image, which is the Zephyr-native answer. */
    const nros_platform_task_attr_t *a = (const nros_platform_task_attr_t *) attr;

    /* issue 0852 — `priority` IS honoured. It used to be discarded along with
     * `stack_bytes`, under the one comment above, but the reasoning that
     * justifies dropping the stack does not reach the priority: this port goes
     * through Zephyr's POSIX layer, `CONFIG_POSIX_PRIORITY_SCHEDULING` gives it
     * `pthread_attr_setschedparam`, and there is no alignment constraint in the
     * way. Accepting a priority and silently dropping it is the failure
     * RFC-0079 is about, and it cost issue 0852 six wrong hypotheses. */
    int native = nros_zephyr_native_priority(a != NULL ? a->priority
                                                       : NROS_PLATFORM_PRIORITY_INHERIT);

    /* `name` IS honoured, for the same reason `priority` above is.
     *
     * This is issue 0852 one field over. The ABI defines `name` as "Task name
     * for the kernel's own tables and crash dumps", every caller already
     * supplies one -- `PlatformTask::spawn_with` fills `attr.name`, and the
     * C++ tier spawn builds "nros-tier-<tier>" for it -- and the FreeRTOS port
     * passes it straight to `xTaskCreate`. Only this port dropped it, and
     * Zephyr is not a kernel with no name concept: CONFIG_THREAD_NAME and
     * `k_thread_name_set` have been there all along.
     *
     * The cost was paid in diagnosis, not in behaviour. In a CTF capture from
     * an FVP the nano-ros threads appear as `unknown (0x00281cc0)` and can
     * only be told apart by stack base -- one of them holding 13.7% of the
     * core with nothing to say what it was -- while the identical code on
     * FreeRTOS traces as `nros_app` and `nros_tier@1`. */
    const char *name = (a != NULL) ? a->name : NULL;

    return nros_zephyr_task_create_prio((pthread_t *) task, entry, arg, native, name) == 0
               ? NROS_PLATFORM_RET_OK
               : NROS_PLATFORM_RET_NOMEM;
}


int8_t nros_platform_task_join(void *task) {
    if (task == NULL) return -1;
    return pthread_join(*(pthread_t *) task, NULL) == 0 ? 0 : -1;
}

int8_t nros_platform_task_detach(void *task) {
    if (task == NULL) return -1;
    return pthread_detach(*(pthread_t *) task) == 0 ? 0 : -1;
}

int8_t nros_platform_task_cancel(void *task) {
    if (task == NULL) return -1;
    return pthread_cancel(*(pthread_t *) task) == 0 ? 0 : -1;
}

void nros_platform_task_exit(void) {
    pthread_exit(NULL);
}

void nros_platform_task_free(void **task) {
    (void) task;  /* caller-owned pthread_t storage */
}

/* phase-360 W5 follow-up — the probe lives WITH the implementation that owns
 * the storage. It used to sit outside every arm and hardcode `pthread_t`, so
 * the non-POSIX builds (the default for these fixtures since issue 0566) could
 * not compile it at all: the type is not declared there. A size stated in one
 * place and allocated in another is issue 0570's trap in the mechanism built to
 * avoid it. */
size_t nros_platform_task_storage_size(void) {
    return sizeof(pthread_t);
}

size_t nros_platform_task_storage_align(void) {
    return _Alignof(pthread_t);
}
/* phase-364 W2 (RFC-0076 D1) — opaque-storage sizing for the lock family, the
 * siblings of the `wake` and `task` probes.
 *
 * Two forms because callers need two. A Rust or otherwise-dynamic caller asks
 * at RUNTIME and allocates; zenoh-pico embeds `_z_mutex_t` BY VALUE and needs
 * the number at COMPILE time, which a function call cannot provide. The
 * `_Static_assert`s below are what stop the two from drifting: the macro and
 * the type are checked against each other in the port that owns both, so a
 * wrong macro is a compile error here rather than a buffer overrun in a
 * consumer.
 *
 * This replaces a hand-computed table in `zpico-sys` that guessed OTHER
 * platforms' struct sizes with `≈` and a "2× safety margin". */
size_t nros_platform_mutex_storage_size(void) { return sizeof(pthread_mutex_t); }
size_t nros_platform_mutex_storage_align(void) { return _Alignof(pthread_mutex_t); }
size_t nros_platform_mutex_rec_storage_size(void) { return sizeof(pthread_mutex_t); }
size_t nros_platform_mutex_rec_storage_align(void) { return _Alignof(pthread_mutex_t); }
size_t nros_platform_condvar_storage_size(void) { return sizeof(pthread_cond_t); }
size_t nros_platform_condvar_storage_align(void) { return _Alignof(pthread_cond_t); }

_Static_assert(NROS_PLATFORM_MUTEX_STORAGE_SIZE >= sizeof(pthread_mutex_t),
               "NROS_PLATFORM_MUTEX_STORAGE_SIZE too small for this port");
_Static_assert(NROS_PLATFORM_MUTEX_REC_STORAGE_SIZE >= sizeof(pthread_mutex_t),
               "NROS_PLATFORM_MUTEX_REC_STORAGE_SIZE too small for this port");
_Static_assert(NROS_PLATFORM_CONDVAR_STORAGE_SIZE >= sizeof(pthread_cond_t),
               "NROS_PLATFORM_CONDVAR_STORAGE_SIZE too small for this port");
_Static_assert(NROS_PLATFORM_TASK_STORAGE_SIZE >= sizeof(pthread_t),
               "NROS_PLATFORM_TASK_STORAGE_SIZE too small for this port");


/* ---- Mutex ---- */

int8_t nros_platform_mutex_init(void *m) {
    if (m == NULL) return -1;
    return pthread_mutex_init((pthread_mutex_t *) m, NULL) == 0 ? 0 : -1;
}

int8_t nros_platform_mutex_drop(void *m) {
    if (m == NULL) return 0;
    return pthread_mutex_destroy((pthread_mutex_t *) m) == 0 ? 0 : -1;
}

int8_t nros_platform_mutex_lock(void *m) {
    if (m == NULL) return -1;
    return pthread_mutex_lock((pthread_mutex_t *) m) == 0 ? 0 : -1;
}

int8_t nros_platform_mutex_try_lock(void *m) {
    if (m == NULL) return -1;
    int rc = pthread_mutex_trylock((pthread_mutex_t *) m);
    if (rc == 0)       return 0;
    if (rc == EBUSY)   return 1;
    return -1;
}

int8_t nros_platform_mutex_unlock(void *m) {
    if (m == NULL) return -1;
    return pthread_mutex_unlock((pthread_mutex_t *) m) == 0 ? 0 : -1;
}

int8_t nros_platform_mutex_rec_init(void *m) {
    if (m == NULL) return -1;
    pthread_mutexattr_t attr;
    if (pthread_mutexattr_init(&attr) != 0) return -1;
    if (pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE) != 0) {
        (void) pthread_mutexattr_destroy(&attr);
        return -1;
    }
    int rc = pthread_mutex_init((pthread_mutex_t *) m, &attr);
    (void) pthread_mutexattr_destroy(&attr);
    return rc == 0 ? 0 : -1;
}
int8_t nros_platform_mutex_rec_drop(void *m)     { return nros_platform_mutex_drop(m); }
int8_t nros_platform_mutex_rec_lock(void *m)     { return nros_platform_mutex_lock(m); }
int8_t nros_platform_mutex_rec_try_lock(void *m) { return nros_platform_mutex_try_lock(m); }
int8_t nros_platform_mutex_rec_unlock(void *m)   { return nros_platform_mutex_unlock(m); }

/* ---- Condvars ---- */

int8_t nros_platform_condvar_init(void *cv) {
    if (cv == NULL) return -1;
    pthread_condattr_t attr;
    if (pthread_condattr_init(&attr) != 0) return -1;
    (void) pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    int rc = pthread_cond_init((pthread_cond_t *) cv, &attr);
    (void) pthread_condattr_destroy(&attr);
    return rc == 0 ? 0 : -1;
}

int8_t nros_platform_condvar_drop(void *cv) {
    if (cv == NULL) return 0;
    return pthread_cond_destroy((pthread_cond_t *) cv) == 0 ? 0 : -1;
}

int8_t nros_platform_condvar_signal(void *cv) {
    if (cv == NULL) return -1;
    return pthread_cond_signal((pthread_cond_t *) cv) == 0 ? 0 : -1;
}

int8_t nros_platform_condvar_signal_all(void *cv) {
    if (cv == NULL) return -1;
    return pthread_cond_broadcast((pthread_cond_t *) cv) == 0 ? 0 : -1;
}

/* Phase 124.B.7.a — ISR-safe signal.
 *
 * Zephyr's k_condvar_signal documents that it MAY be called from
 * ISR context (the doc is split — newer kernels enforce thread
 * context). Use k_condvar_signal directly; if a backend exercises
 * the ISR path on a kernel build that rejects it, we'll need a
 * dedicated k_sem fallback. Track in the platform integration
 * tests. */
int8_t nros_platform_condvar_signal_from_isr(void *cv) {
    if (cv == NULL) return -1;
    return pthread_cond_signal((pthread_cond_t *) cv) == 0 ? 0 : -1;
}

int8_t nros_platform_condvar_wait(void *cv, void *m) {
    if (cv == NULL || m == NULL) return -1;
    return pthread_cond_wait((pthread_cond_t *) cv,
                             (pthread_mutex_t *) m) == 0 ? 0 : -1;
}

int8_t nros_platform_condvar_wait_until(void *cv, void *m, uint64_t abstime_ms) {
    if (cv == NULL || m == NULL) return -1;
    struct timespec ts = {
        .tv_sec = (time_t) (abstime_ms / 1000U),
        .tv_nsec = (long) ((abstime_ms % 1000U) * 1000000U),
    };
    int rc = pthread_cond_timedwait((pthread_cond_t *) cv,
                                    (pthread_mutex_t *) m,
                                    &ts);
    if (rc == 0)         return 0;
    if (rc == ETIMEDOUT) return 1;
    return -1;
}

#else

/* Issue #566 — the non-POSIX arm used to stub ~20 functions to -1, on a
 * kernel that has all of these natively. A -1 nobody checks is worse than
 * no implementation at all: it hands the caller a mutex that does not
 * lock. These are now backed by `k_mutex` / `k_condvar` / `k_thread`.
 *
 * Storage: the ABI's objects are caller-provided and opaque, and the
 * smallest consumer on this platform sizes them from `pthread_mutex_t`
 * (a `uint32_t`), which cannot hold a `struct k_mutex` inline. So the
 * caller's storage holds a POINTER to a heap-allocated kernel object —
 * the same shape the FreeRTOS port uses for its `SemaphoreHandle_t`.
 * That needs storage of at least `sizeof(void *)`, which every caller
 * provides on this 32-bit ABI. */

#define NROS_Z_HANDLE(ptr) (*(void **) (ptr))

int8_t nros_platform_mutex_init(void *m) {
    if (m == NULL) return -1;
/* RFC-0034 D6 -- allocations here go through `nros_platform_alloc`, NOT
 * `k_malloc` directly. On Zephyr both reach `_system_heap` today, so the
 * distinction looks cosmetic; it is not. The funnel is what lets the arena's
 * ALGORITHM be swapped (a constant-time allocator for the real-time tier)
 * without hunting every allocation site in the tree. A direct `k_malloc` here
 * would keep allocating from the kernel heap after the funnel moved, silently
 * splitting the one arena D6 exists to keep whole. */
    struct k_mutex *mu = nros_platform_alloc(sizeof(struct k_mutex));
    if (mu == NULL) return -1;
    if (k_mutex_init(mu) != 0) {
        nros_platform_dealloc(mu);
        return -1;
    }
    NROS_Z_HANDLE(m) = mu;
    return 0;
}

int8_t nros_platform_mutex_drop(void *m) {
    if (m == NULL) return 0;
    struct k_mutex *mu = NROS_Z_HANDLE(m);
    if (mu == NULL) return 0;
    nros_platform_dealloc(mu);
    NROS_Z_HANDLE(m) = NULL;
    return 0;
}

int8_t nros_platform_mutex_lock(void *m) {
    if (m == NULL) return -1;
    struct k_mutex *mu = NROS_Z_HANDLE(m);
    if (mu == NULL) return -1;
    return k_mutex_lock(mu, K_FOREVER) == 0 ? 0 : -1;
}

int8_t nros_platform_mutex_try_lock(void *m) {
    if (m == NULL) return -1;
    struct k_mutex *mu = NROS_Z_HANDLE(m);
    if (mu == NULL) return -1;
    /* 0 = acquired, 1 = would block (ABI's `try` contract), -1 = error. */
    int rc = k_mutex_lock(mu, K_NO_WAIT);
    if (rc == 0)      return 0;
    if (rc == -EBUSY) return 1;
    return -1;
}

int8_t nros_platform_mutex_unlock(void *m) {
    if (m == NULL) return -1;
    struct k_mutex *mu = NROS_Z_HANDLE(m);
    if (mu == NULL) return -1;
    return k_mutex_unlock(mu) == 0 ? 0 : -1;
}

/* `k_mutex` is recursive for the owning thread (`lock_count`, see
 * zephyr/kernel/mutex.c), which is exactly what the ABI requires of
 * `mutex_rec_*` — zenoh-pico deadlocks on a non-recursive one. */
int8_t nros_platform_mutex_rec_init(void *m)     { return nros_platform_mutex_init(m); }
int8_t nros_platform_mutex_rec_drop(void *m)     { return nros_platform_mutex_drop(m); }
int8_t nros_platform_mutex_rec_lock(void *m)     { return nros_platform_mutex_lock(m); }
int8_t nros_platform_mutex_rec_try_lock(void *m) { return nros_platform_mutex_try_lock(m); }
int8_t nros_platform_mutex_rec_unlock(void *m)   { return nros_platform_mutex_unlock(m); }

int8_t nros_platform_condvar_init(void *cv) {
    if (cv == NULL) return -1;
    struct k_condvar *c = nros_platform_alloc(sizeof(struct k_condvar));
    if (c == NULL) return -1;
    if (k_condvar_init(c) != 0) {
        nros_platform_dealloc(c);
        return -1;
    }
    NROS_Z_HANDLE(cv) = c;
    return 0;
}

int8_t nros_platform_condvar_drop(void *cv) {
    if (cv == NULL) return 0;
    struct k_condvar *c = NROS_Z_HANDLE(cv);
    if (c == NULL) return 0;
    nros_platform_dealloc(c);
    NROS_Z_HANDLE(cv) = NULL;
    return 0;
}

int8_t nros_platform_condvar_signal(void *cv) {
    if (cv == NULL) return -1;
    struct k_condvar *c = NROS_Z_HANDLE(cv);
    if (c == NULL) return -1;
    return k_condvar_signal(c) == 0 ? 0 : -1;
}

int8_t nros_platform_condvar_signal_all(void *cv) {
    if (cv == NULL) return -1;
    struct k_condvar *c = NROS_Z_HANDLE(cv);
    if (c == NULL) return -1;
    return k_condvar_broadcast(c) >= 0 ? 0 : -1;
}

int8_t nros_platform_condvar_wait(void *cv, void *m) {
    if (cv == NULL || m == NULL) return -1;
    struct k_condvar *c = NROS_Z_HANDLE(cv);
    struct k_mutex *mu = NROS_Z_HANDLE(m);
    if (c == NULL || mu == NULL) return -1;
    return k_condvar_wait(c, mu, K_FOREVER) == 0 ? 0 : -1;
}

int8_t nros_platform_condvar_wait_until(void *cv, void *m, uint64_t abstime_ms) {
    if (cv == NULL || m == NULL) return -1;
    struct k_condvar *c = NROS_Z_HANDLE(cv);
    struct k_mutex *mu = NROS_Z_HANDLE(m);
    if (c == NULL || mu == NULL) return -1;
    /* `abstime_ms` is on the same monotonic epoch as the platform clock;
     * k_condvar_wait takes a RELATIVE timeout. */
    uint64_t now_ms = nros_platform_clock_ns() / 1000000ULL;
    k_timeout_t rel = (abstime_ms > now_ms)
                          ? K_MSEC((uint32_t) (abstime_ms - now_ms))
                          : K_NO_WAIT;
    int rc = k_condvar_wait(c, mu, rel);
    if (rc == 0)         return 0;
    if (rc == -EAGAIN)   return 1; /* timed out — the ABI's `1` */
    return -1;
}

/* Tasks need a stack, and a dynamically-created thread needs
 * CONFIG_DYNAMIC_THREAD to allocate one. Where that is unavailable this
 * still cannot spawn — but it says so at the call rather than pretending,
 * and mutexes/condvars above no longer go down with it. */
#if defined(CONFIG_DYNAMIC_THREAD) && defined(CONFIG_THREAD_STACK_INFO)

#ifndef NROS_ZEPHYR_TASK_STACK_SIZE
#define NROS_ZEPHYR_TASK_STACK_SIZE 4096
#endif

struct nros_z_task {
    struct k_thread thread;
    k_thread_stack_t *stack;
    void *(*entry)(void *);
    void *arg;
};

static void nros_z_task_trampoline(void *p1, void *p2, void *p3) {
    (void) p2;
    (void) p3;
    struct nros_z_task *t = (struct nros_z_task *) p1;
    (void) t->entry(t->arg);
}

int8_t nros_platform_task_init(void *task, void *attr,
                               void *(*entry)(void *), void *arg) {
    /* Only `name` is read here. `priority` and `stack_bytes` remain unhandled
     * on this path -- it hardcodes K_PRIO_PREEMPT(5) and
     * NROS_ZEPHYR_TASK_STACK_SIZE -- which is a separate gap and is left
     * alone rather than half-fixed. This path is the non-CONFIG_POSIX_API
     * build and is not the one CI exercises. */
    const nros_platform_task_attr_t *a = (const nros_platform_task_attr_t *) attr;
    if (task == NULL || entry == NULL) return -1;
    struct nros_z_task *t = nros_platform_alloc(sizeof(struct nros_z_task));
    if (t == NULL) return -1;
    t->stack = k_thread_stack_alloc(NROS_ZEPHYR_TASK_STACK_SIZE, 0);
    if (t->stack == NULL) {
        nros_platform_dealloc(t);
        return -1;
    }
    t->entry = entry;
    t->arg = arg;
    k_thread_create(&t->thread, t->stack, NROS_ZEPHYR_TASK_STACK_SIZE,
                    nros_z_task_trampoline, t, NULL, NULL,
                    K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    /* Best-effort, as on the POSIX path: an unnamed thread is what this did
     * before, so a refusal must not fail the spawn. */
    if (a != NULL && a->name != NULL) {
        (void) k_thread_name_set(&t->thread, a->name);
    }
    NROS_Z_HANDLE(task) = t;
    return 0;
}

int8_t nros_platform_task_join(void *task) {
    if (task == NULL) return -1;
    struct nros_z_task *t = NROS_Z_HANDLE(task);
    if (t == NULL) return -1;
    return k_thread_join(&t->thread, K_FOREVER) == 0 ? 0 : -1;
}

int8_t nros_platform_task_detach(void *task) {
    (void) task; /* Zephyr threads need no detach; storage is freed by _free. */
    return 0;
}

int8_t nros_platform_task_cancel(void *task) {
    if (task == NULL) return -1;
    struct nros_z_task *t = NROS_Z_HANDLE(task);
    if (t == NULL) return -1;
    k_thread_abort(&t->thread);
    return 0;
}

void nros_platform_task_exit(void) {
    k_thread_abort(k_current_get());
}

void nros_platform_task_free(void **task) {
    if (task == NULL || *task == NULL) return;
    struct nros_z_task *t = (struct nros_z_task *) *task;
    if (t->stack != NULL) {
        (void) k_thread_stack_free(t->stack);
    }
    nros_platform_dealloc(t);
    *task = NULL;
}

/* The k_thread-backed storage this arm actually allocates. */
size_t nros_platform_task_storage_size(void) {
    return sizeof(struct nros_z_task);
}

size_t nros_platform_task_storage_align(void) {
    return _Alignof(struct nros_z_task);
}

#else /* no CONFIG_DYNAMIC_THREAD */

int8_t nros_platform_task_init(void *task, void *attr,
                               void *(*entry)(void *), void *arg) {
    (void) task; (void) attr; (void) entry; (void) arg;
    /* Needs CONFIG_POSIX_API, or CONFIG_DYNAMIC_THREAD +
     * CONFIG_THREAD_STACK_INFO for a dynamically allocated stack. */
    return -1;
}
int8_t nros_platform_task_join(void *task)   { (void) task; return -1; }
int8_t nros_platform_task_detach(void *task) { (void) task; return -1; }
int8_t nros_platform_task_cancel(void *task) { (void) task; return -1; }
void nros_platform_task_exit(void) {}
void nros_platform_task_free(void **task)    { (void) task; }

/* This arm cannot start a task at all (`task_init` returns -1), so there is no
 * storage to size. Zero is the honest answer, not a guess that would let a
 * caller allocate for a task it can never create. */
size_t nros_platform_task_storage_size(void)  { return 0; }
size_t nros_platform_task_storage_align(void) { return 1; }

#endif /* CONFIG_DYNAMIC_THREAD */

#endif

/* ============================================================
 *   Wake primitive (Phase 130)
 *
 *   Binary semaphore backed by `k_sem`. Bypasses libc pthread
 *   so the executor's spin_once wake is not subject to the
 *   Zephyr libc `pthread_cond_timedwait` deadline-hang
 *   (Phase 127.C.4). `k_sem_give` is ISR-safe per Zephyr spec.
 *   Available unconditionally — `k_sem` ships in every Zephyr
 *   kernel build, no Kconfig gate.
 * ============================================================ */

int8_t nros_platform_wake_init(void *w) {
    if (w == NULL) return -1;
    k_sem_init((struct k_sem *) w, 0u, 1u);
    return 0;
}

int8_t nros_platform_wake_drop(void *w) {
    /* k_sem has no destructor; reset to a known-empty state so any
     * stale waiter (impossible if the caller respects ownership)
     * sees -EAGAIN on the next take. */
    if (w == NULL) return 0;
    k_sem_reset((struct k_sem *) w);
    return 0;
}

int8_t nros_platform_wake_wait_ms(void *w, uint32_t timeout_ms) {
    if (w == NULL) return -1;
    k_timeout_t to = (timeout_ms == 0u) ? K_NO_WAIT : K_MSEC(timeout_ms);
    int rc = k_sem_take((struct k_sem *) w, to);
    if (rc == 0)        return 0;
    if (rc == -EAGAIN)  return 1;
    return -1;
}

int8_t nros_platform_wake_signal(void *w) {
    if (w == NULL) return -1;
    k_sem_give((struct k_sem *) w);
    return 0;
}

int8_t nros_platform_wake_signal_from_isr(void *w) {
    if (w == NULL) return -1;
    /* k_sem_give is documented ISR-safe on Zephyr. */
    k_sem_give((struct k_sem *) w);
    return 0;
}

size_t nros_platform_wake_storage_size(void) {
    return sizeof(struct k_sem);
}

size_t nros_platform_wake_storage_align(void) {
    return __alignof__(struct k_sem);
}

/* phase-359 W10 — opaque-storage sizing for `task`, the sibling of the wake
 * probes above. `task_init`'s contract says the implementor decides the size;
 * these let a caller ASK instead of hard-coding it (issue 0570's trap). */


/* ============================================================
 *   Critical section (Phase 121.9)
 * ============================================================ */
/* Zephyr's `irq_lock` returns the prior IRQ posture; `irq_unlock`
 * accepts the same value back. Reentrant: Zephyr's port layer stacks
 * the key word correctly across nested calls. */
uint32_t nros_platform_critical_section_acquire(void) {
    return (uint32_t) irq_lock();
}

void nros_platform_critical_section_release(uint32_t token) {
    irq_unlock((unsigned int) token);
}

/* ============================================================
 *   Logging (Phase 88)
 *
 *   When CONFIG_LOG=y, route through Zephyr's logging subsystem
 *   (`LOG_INF` / `LOG_WRN` etc., backed by `log_msg_runtime_create`).
 *   Falls back to `printk` when CONFIG_LOG is disabled so the
 *   message still reaches the system console.
 *
 *   Module name `nros` is registered with `LOG_MODULE_REGISTER` so
 *   Zephyr's shell `log enable warn nros` filters at the platform
 *   layer (in addition to the per-Logger threshold on the nros-log
 *   side). ISR-safe: Zephyr LOG queues for deferred processing.
 * ============================================================ */
#ifdef CONFIG_LOG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nros, CONFIG_LOG_DEFAULT_LEVEL);
#endif

#include <stdio.h>

#define NROS_PLATFORM_LOG_BUFSZ 1280

static void nros_platform_log_format(char *out, size_t out_sz,
                                     const uint8_t *name_ptr, uintptr_t name_len,
                                     const uint8_t *msg_ptr,  uintptr_t msg_len) {
    if (name_ptr != NULL && name_len > 0) {
        snprintf(out, out_sz, "%.*s: %.*s",
                 (int) name_len, (const char *) name_ptr,
                 (int) msg_len,  (const char *) msg_ptr);
    } else {
        snprintf(out, out_sz, "%.*s",
                 (int) msg_len, (const char *) msg_ptr);
    }
}

void nros_platform_log_write(uint8_t severity,
                             const uint8_t *name_ptr, uintptr_t name_len,
                             const uint8_t *msg_ptr,  uintptr_t msg_len) {
    if (msg_ptr == NULL && msg_len > 0) {
        return;
    }
    char buf[NROS_PLATFORM_LOG_BUFSZ];
    nros_platform_log_format(buf, sizeof(buf), name_ptr, name_len, msg_ptr, msg_len);
#ifdef CONFIG_LOG
    switch (severity) {
    case 5: /* Fatal */
    case 4: /* Error */ LOG_ERR("%s", buf); break;
    case 3: /* Warn  */ LOG_WRN("%s", buf); break;
    case 2: /* Info  */ LOG_INF("%s", buf); break;
    case 1: /* Debug */
    case 0: /* Trace */ LOG_DBG("%s", buf); break;
    default:            LOG_INF("%s", buf); break;
    }
#else
    static const char *labels[] = {
        "[TRACE]", "[DEBUG]", "[INFO]", "[WARN]", "[ERROR]", "[FATAL]",
    };
    const char *label = severity <= 5 ? labels[severity] : "[?]";
    printk("%s %s\n", label, buf);
#endif
}

void nros_platform_log_flush(void) {
#ifdef CONFIG_LOG
    /* Best-effort: yield so the log thread drains its deferred queue. */
    k_yield();
#endif
}

/* ============================================================
 * Runtime locator override — nano-ros #166 / phase-286 W1.
 *
 * native_sim test parallelism: the test harness starts a per-test zenohd on an
 * ephemeral port and launches the image with `-testargs --nros-locator=<loc>`.
 * Reading that here (preferred over the build-time-baked
 * `CONFIG_NROS_ZENOH_LOCATOR`) gives every test a distinct router port, so the
 * zenoh e2e lanes stop serializing on one shared baked port.
 *
 * Why `-testargs`: native_sim's own option parser ABORTS on an unregistered
 * option ("Incorrect option '--nros-locator=…'"). Everything after `-testargs`
 * is instead collected into the native-simulator "test args" argv, bypassing
 * that parser; the app reads it via the native-simulator public API
 * `nsi_get_test_cmd_line_args`. No NSI_TASK / option-struct registration needed.
 *
 * native_sim / native_posix only (`CONFIG_ARCH_POSIX`): on real embedded there
 * is no host argv channel, so the hook returns NULL and the baked locator
 * stands. The `loc` form matches the bake — `tcp/host:port` (zenoh) or bare
 * `host:port` (xrce), exactly as the example `build.rs` unifies `NROS_LOCATOR`.
 * ============================================================ */
#if defined(CONFIG_ARCH_POSIX)
/* Provided by the native-simulator runtime (linked into every native_sim
 * image). Prototype declared locally so this module does not couple to the
 * board-local `<nsi_cmdline.h>` include path. */
extern void nsi_get_test_cmd_line_args(int *argc, char ***argv);

const char *nros_runtime_locator_override(void) {
    static const char *cached;
    static int resolved;
    if (resolved) {
        return cached;
    }
    resolved = 1;
    cached = NULL;
    int argc = 0;
    char **argv = NULL;
    nsi_get_test_cmd_line_args(&argc, &argv);
    static const char prefix[] = "--nros-locator=";
    const size_t plen = sizeof(prefix) - 1;
    for (int i = 0; argv != NULL && i < argc; i++) {
        if (argv[i] != NULL && strncmp(argv[i], prefix, plen) == 0 && argv[i][plen] != '\0') {
            cached = argv[i] + plen;
        }
    }
    return cached;
}
#else
const char *nros_runtime_locator_override(void) {
    return NULL;
}
#endif

/* ---- Fatal error (phase-366 / RFC-0077) ----
 *
 * `printk` then `k_panic()`. Both halves are deliberate.
 *
 * `printk` rather than the `LOG_*` macros: the logging subsystem may be
 * deferred (`CONFIG_LOG_MODE_DEFERRED`), in which case a message logged here is
 * processed by a thread that will never run again. `printk` is synchronous and
 * works with the scheduler locked or from an ISR, which is the contract this
 * function has to honour.
 *
 * `k_panic()` rather than a spin: it routes into Zephyr's own fatal path, so an
 * image that installed `k_sys_fatal_error_handler` — the RTOS's own weak
 * override hook — still gets to run it. Spinning here would silently defeat
 * that.
 */
__attribute__((weak))
_Noreturn void nros_platform_panic(const char *msg, size_t len) {
    if (msg != NULL && len > 0) {
        printk("nros: PANIC %.*s\n", (int) len, msg);
    } else {
        printk("nros: PANIC\n");
    }
    k_panic();
    /* k_panic() is noreturn, but it is a macro on some lines and the compiler
     * cannot always see that; keep the contract explicit. */
    for (;;) {
    }
}

/* Headroom for the calling thread. `k_thread_stack_space_get` answers in
 * unused BYTES, which is the ABI's unit, so no conversion.
 *
 * GATED, and on BOTH Kconfigs (issue 1075). The original comment here said the
 * kernel "returns an error, which becomes the documented 0" without
 * CONFIG_INIT_STACKS, and that was wrong twice over:
 *
 *   1. it named only CONFIG_INIT_STACKS. `zephyr/kernel/thread.c` encloses
 *      `z_impl_k_thread_stack_space_get` in
 *      `#if defined(CONFIG_INIT_STACKS) && defined(CONFIG_THREAD_STACK_INFO)`,
 *      so a build with only the first still has no implementation;
 *   2. it reasoned about RUN time for a failure that happens at LINK time.
 *      Without both symbols the function is not compiled at all, so there is
 *      nothing to return an error — the reference is undefined and every
 *      native_sim image failed to link. Neither Kconfig is set anywhere in
 *      this repo.
 *
 * The other four ports already do this: freertos gates on
 * INCLUDE_uxTaskGetStackHighWaterMark, threadx on TX_ENABLE_STACK_CHECKING,
 * posix returns 0 outright. Zephyr was the one that needed it most — a syscall
 * is the only one of the four that can vanish from the link — and the one that
 * did not have it.
 *
 * `0` is the ABI's "not instrumented", which is exactly what a build without
 * the watermark is. Turning the Kconfigs ON is a separate decision about the
 * shipped image (stack painting costs boot time and per-thread bytes), not
 * about this bug. */
size_t nros_platform_task_stack_unused_bytes(void) {
#if defined(CONFIG_INIT_STACKS) && defined(CONFIG_THREAD_STACK_INFO)
    size_t unused = 0;
    if (k_thread_stack_space_get(k_current_get(), &unused) != 0) {
        return 0;
    }
    return unused;
#else
    return 0;
#endif
}
