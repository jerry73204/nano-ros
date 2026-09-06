/**
 * @file nros_platform_zephyr_shims.c
 * @brief Real-symbol wrappers around Zephyr kernel inlines.
 *
 * Several Zephyr APIs are declared `static inline` in headers
 * (`k_msleep`, `k_uptime_get`, `sys_rand_get`, etc.) and have no exported
 * symbol. Rust FFI can only call real symbols, so we wrap them here —
 * this TU is compiled by the Zephyr module build and exports the real
 * functions that `nros-platform-zephyr` declares as `extern "C"`.
 *
 * Real-function Zephyr APIs (`k_malloc`, `k_free`, `k_usleep`,
 * `sys_rand32_get`, `pthread_*`) are called directly from Rust and do
 * not need wrappers.
 */

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>

/* ── Clock / sleep / random (no POSIX dependency) ───────────────────── */

int64_t nros_zephyr_uptime_ms(void) {
    return k_uptime_get();
}

int32_t nros_zephyr_msleep(int32_t ms) {
    return k_msleep(ms);
}

void nros_zephyr_rand_fill(void* dst, size_t len) {
    sys_rand_get(dst, len);
}

/* Phase 77.22: cooperative yield. k_yield is declared `static inline`
 * in <zephyr/kernel.h>, so wrap it here to get a real callable symbol.
 */
void nros_zephyr_yield(void) {
    k_yield();
}

/* Phase 110.D — per-thread scheduling controls. `k_thread_priority_set`
 * and `k_current_get` are static inlines, so wrap each as a real
 * symbol that Rust FFI can link.
 */
void nros_zephyr_thread_priority_set(int prio) {
    k_thread_priority_set(k_current_get(), prio);
}

/* issue 0655 — the gate is `CONFIG_SCHED_CPU_MASK`, NOT
 * `CONFIG_SCHED_CPU_MASK_PIN_ONLY`.
 *
 * `k_thread_cpu_pin` is declared under `#ifdef CONFIG_SCHED_CPU_MASK`
 * (kernel.h) and implemented in `kernel/cpu_mask.c`; PIN_ONLY is a NARROWER
 * variant that additionally requires SMP and makes the mask immutable after
 * start. Gating on PIN_ONLY therefore compiled this call out of every image
 * nano-ros builds — including any that enabled the API correctly — and no
 * build could catch a mistake inside a branch the preprocessor deleted.
 *
 * Pin an ALREADY-STARTED thread by tid. `cpu_mask_mod` requires
 * `z_is_thread_prevented_from_running`, so this succeeds only for a thread
 * created with `K_FOREVER` and not yet started — which is exactly how
 * `nros_zephyr_tier_task_create` uses it. Kept separate from the CALLING-thread
 * variant below because the two have genuinely different preconditions, and
 * collapsing them is what hid issue 0655. */
int nros_zephyr_thread_cpu_pin_tid(void* tid, int cpu) {
#ifdef CONFIG_SCHED_CPU_MASK
    return k_thread_cpu_pin((k_tid_t)tid, cpu);
#else
    (void)tid;
    (void)cpu;
    return -ENOSYS;
#endif
}

/* Pin the CALLING thread. issue 0655: this CANNOT succeed — `k_current_get()`
 * is by definition running, so `cpu_mask_mod` returns -EINVAL every time. It
 * survives only for the BOOT tier, which Zephyr has already started before
 * `run_tiers` ever sees it and which therefore has no create/start window to
 * use. Callers must report the failure honestly rather than retry; spawned
 * tiers go through `nros_zephyr_tier_task_create`'s pinned path instead. */
int nros_zephyr_thread_cpu_pin(int cpu) {
#ifdef CONFIG_SCHED_CPU_MASK
    return k_thread_cpu_pin(k_current_get(), cpu);
#else
    (void)cpu;
    return -ENOSYS;
#endif
}

/* The "this image cannot answer" value. Deliberately not 0: a uniprocessor
 * image reporting cpu 0 is indistinguishable from a correct pin to cpu 0, which
 * is the exact confusion this marker exists to end. Mirrored in
 * `entry_tiers.rs` and `nros_tests::output`. */
#define NROS_ZEPHYR_CPU_UNKNOWN 0xFFFFFFFFu

/* issue 0260 / phase-356 — which CPU is the CALLING thread actually on?
 *
 * The pin markers report the kernel's verdict on `k_thread_cpu_pin`, i.e. that
 * the call was ACCEPTED. That is not the same claim as "the tier ran where it
 * was asked to", and the difference is invisible on a uniprocessor image, where
 * a pin to cpu 0 is accepted and cpu 0 is the only place it could have run.
 * #260's SMP fixture only earns its cost if the tier reports the CPU it was
 * OBSERVED on; otherwise it asserts exactly what the uniprocessor image already
 * asserts.
 *
 * `arch_curr_cpu()->id` is the API for it — and it is declared INSIDE
 * `#ifdef CONFIG_SMP` in `arch_interface.h`. The posix arch (native_sim) does
 * not provide it at all: its `arch_inlines.h` defines only `arch_num_cpus`. So
 * this is SMP-only by construction, not by choice, and a uniprocessor image
 * gets the sentinel rather than a fabricated 0 — reporting "cpu 0" on an image
 * that cannot answer the question is precisely the false evidence this exists
 * to remove.
 *
 * NOT `arch_proc_id()`, which this used first and which is wrong here: on
 * arm64 it returns the raw MPIDR_EL1, whose bit 31 is RES1, so a tier pinned to
 * core 1 reported `running_on=2147483649` (0x80000001) on the first SMP image
 * that ran. The number a `core` dim is written in is the LOGICAL cpu index, and
 * `arch_curr_cpu()->id` is that. Measured on qemu_cortex_a53 SMP, not reasoned
 * about — the raw-MPIDR value is plausible enough to survive review.
 *
 * Returns the CPU id, or NROS_ZEPHYR_CPU_UNKNOWN when the image cannot say. */
uint32_t nros_zephyr_current_cpu(void) {
#ifdef CONFIG_SMP
    return (uint32_t)arch_curr_cpu()->id;
#else
    return NROS_ZEPHYR_CPU_UNKNOWN;
#endif
}

/* issue 0758 W4 — acquire the wall-clock epoch, if this image asked for one.
 *
 * WHY THE DECISION LIVES IN C. Zephyr has TWO tier arms — `zephyr_run_tiers.c`
 * for C/C++ and `entry_tiers.rs` for Rust — and they must not drift. Putting
 * `#ifdef CONFIG_NROS_SNTP_EPOCH` in each would guarantee they eventually do,
 * and the Rust one CANNOT hold that ifdef anyway: Kconfig knobs reach the
 * Zephyr C lane and not the cargo lane (issue 0460), so `entry_tiers.rs` has no
 * way to see the symbol without a build-script knob read.
 *
 * So both arms call this unconditionally and the C side decides. An image
 * without the knob gets an empty function the linker keeps or drops; nothing
 * upstream has to know.
 *
 * Non-fatal by construction: a time server that is down leaves
 * `nros_platform_epoch_us()` answering 0, callers keep stamping boot-relative
 * time knowingly, and the image boots. Refusing to start a control island
 * because NTP is unreachable would be the wrong trade. */
void nros_zephyr_epoch_acquire_configured(void) {
#ifdef CONFIG_NROS_SNTP_EPOCH
    extern int nros_platform_epoch_acquire_sntp(const char *server, uint32_t timeout_ms);
    int rc = nros_platform_epoch_acquire_sntp(CONFIG_NROS_SNTP_SERVER, CONFIG_NROS_SNTP_TIMEOUT_MS);
    if (rc == 0) {
        printk("nros: wall-clock epoch acquired from %s\n", CONFIG_NROS_SNTP_SERVER);
    } else {
        printk("nros: SNTP epoch unavailable from %s (rc=%d) — stamps stay boot-relative\n",
               CONFIG_NROS_SNTP_SERVER, rc);
    }
#endif
}

/* phase-8 W4 — callback-tracing sink for the Zephyr CTF backend.
 *
 * This is the C ABI the core's `set_trace_sink` takes:
 * `unsafe extern "C" fn(u32, u32)`, chosen to BE this signature. The core
 * calls whatever pointer is installed and knows nothing about Zephyr; the
 * entry path installs this function; the dependency stays one-directional.
 *
 * WHY THE DECISION LIVES IN C, same reason as
 * `nros_zephyr_epoch_acquire_configured` above. Two facts force it:
 *
 *  1. `sys_trace_app_marker` is an OUT-OF-TREE Zephyr patch carried by the
 *     consumer, not by nano-ros (`patches/zephyr/0002-ctf-app-marker-event.patch`
 *     in autoware-safety-island; absent from `zephyr/patches/` here). If any
 *     nano-ros crate or TU named the symbol unconditionally, every Zephyr image
 *     built WITHOUT that patch would fail to link. Naming it only inside this
 *     `#ifdef`, in a TU the module already compiles, keeps an unpatched image
 *     linking against an empty function.
 *  2. The symbol exists only under `CONFIG_TRACING_CTF`, and Kconfig knobs
 *     reach the Zephyr C lane and NOT the cargo one (issue 0460) — so no Rust
 *     arm can hold this `#ifdef` without a build-script knob read.
 *
 * Declared `extern` at the call site rather than by including
 * <zephyr/tracing/tracing.h>: the header's `sys_trace_*` set is macro-driven
 * and an out-of-tree event is not in it, so the declaration has to be local
 * either way, and keeping it inside the guard means the name appears in the
 * TU only when the patch that defines it is present.
 *
 * `marker_id` / `arg` are the wire encoding agreed with the decoder
 * (`scripts/parse-zephyr-ctf.py`); this shim is a pure forwarder and imposes
 * no meaning on either field. */
void nros_zephyr_trace_marker(uint32_t marker_id, uint32_t arg) {
#ifdef CONFIG_TRACING_CTF
    extern void sys_trace_app_marker(uint32_t, uint32_t);
    sys_trace_app_marker(marker_id, arg);
#else
    (void)marker_id;
    (void)arg;
#endif
}

/* Phase 110.E.b — periodic timer for Sporadic-server budget refill.
 * Wraps `k_timer_*` (static inlines) plus a per-timer bridge struct
 * holding (callback, user_data) so the Rust side can pass an
 * `extern "C" fn(*mut c_void)` despite Zephyr's
 * `void(*)(struct k_timer *)` expiration signature.
 */
typedef struct {
    void (*cb)(void*);
    void* user_data;
} nros_zephyr_timer_bridge_t;

static void nros_zephyr_timer_expiry(struct k_timer* t) {
    nros_zephyr_timer_bridge_t* b = (nros_zephyr_timer_bridge_t*)k_timer_user_data_get(t);
    if (b && b->cb) {
        b->cb(b->user_data);
    }
}

void* nros_zephyr_timer_create_periodic(unsigned int period_us, void (*cb)(void*),
                                        void* user_data) {
    struct k_timer* t = k_malloc(sizeof(*t));
    if (!t) return NULL;
    nros_zephyr_timer_bridge_t* b = k_malloc(sizeof(*b));
    if (!b) {
        k_free(t);
        return NULL;
    }
    b->cb = cb;
    b->user_data = user_data;
    k_timer_init(t, nros_zephyr_timer_expiry, NULL);
    k_timer_user_data_set(t, b);
    k_timer_start(t, K_USEC(period_us), K_USEC(period_us));
    return t;
}

void nros_zephyr_timer_destroy(void* timer) {
    if (!timer) return;
    struct k_timer* t = (struct k_timer*)timer;
    nros_zephyr_timer_bridge_t* b = (nros_zephyr_timer_bridge_t*)k_timer_user_data_get(t);
    k_timer_stop(t);
    if (b) k_free(b);
    k_free(t);
}

/* Phase 110.E.b follow-up — oneshot variant (period = K_NO_WAIT
 * means fire once and stop).
 */
void* nros_zephyr_timer_create_oneshot(unsigned int timeout_us, void (*cb)(void*),
                                       void* user_data) {
    struct k_timer* t = k_malloc(sizeof(*t));
    if (!t) return NULL;
    nros_zephyr_timer_bridge_t* b = k_malloc(sizeof(*b));
    if (!b) {
        k_free(t);
        return NULL;
    }
    b->cb = cb;
    b->user_data = user_data;
    k_timer_init(t, nros_zephyr_timer_expiry, NULL);
    k_timer_user_data_set(t, b);
    /* Second arg = period; K_NO_WAIT (0) makes this a oneshot. */
    k_timer_start(t, K_USEC(timeout_us), K_NO_WAIT);
    return t;
}

/* Stop the timer without freeing. Returns 1 if the timer was running
 * and got stopped, 0 if it had already expired or was never started.
 */
int nros_zephyr_timer_cancel(void* timer) {
    if (!timer) return 0;
    struct k_timer* t = (struct k_timer*)timer;
    /* k_timer_status_get reports remaining time; 0 means already
     * fired. We use k_timer_remaining_get which returns 0 on fired. */
    unsigned int remaining = k_timer_remaining_get(t);
    k_timer_stop(t);
    return remaining > 0 ? 1 : 0;
}

/* ── BSD socket wrappers ────────────────────────────────────────────
 *
 * On native_sim, glibc's getaddrinfo/freeaddrinfo symbols override
 * Zephyr's POSIX wrappers. The glibc versions return POSIX addrinfo
 * layout (ai_flags first), but Zephyr's zsock_addrinfo has ai_next
 * first. Use Zephyr's zsock_* API directly to avoid the collision.
 *
 * Gated on CONFIG_NET_SOCKETS. These wrappers use socklen_t and struct
 * sockaddr unconditionally, so without the gate this TU cannot compile
 * without the IP stack -- and an image that only ever talks over UART is
 * then forced to carry one. On an S32K344 that is the difference between
 * fitting in 320 KiB and not: the serial talker sat at 94% of SRAM purely
 * because NET_SOCKETS dragged in the stack it never uses.
 */

#if defined(CONFIG_NET_SOCKETS)

#include <zephyr/net/socket.h>

int nros_zephyr_getaddrinfo(const char* node, const char* service,
                            const struct zsock_addrinfo* hints, struct zsock_addrinfo** res) {
    return zsock_getaddrinfo(node, service, hints, res);
}

void nros_zephyr_freeaddrinfo(struct zsock_addrinfo* res) {
    zsock_freeaddrinfo(res);
}

int nros_zephyr_socket(int family, int type, int proto) {
    return zsock_socket(family, type, proto);
}

int nros_zephyr_close(int fd) {
    return zsock_close(fd);
}

int nros_zephyr_connect(int fd, const struct sockaddr* addr, socklen_t addrlen) {
    return zsock_connect(fd, addr, addrlen);
}

int nros_zephyr_bind(int fd, const struct sockaddr* addr, socklen_t addrlen) {
    return zsock_bind(fd, addr, addrlen);
}

int nros_zephyr_listen(int fd, int backlog) {
    return zsock_listen(fd, backlog);
}

int nros_zephyr_accept(int fd, struct sockaddr* addr, socklen_t* addrlen) {
    return zsock_accept(fd, addr, addrlen);
}

int nros_zephyr_shutdown(int fd, int how) {
    return zsock_shutdown(fd, how);
}

int nros_zephyr_setsockopt(int fd, int level, int optname, const void* optval, socklen_t optlen) {
    return zsock_setsockopt(fd, level, optname, optval, optlen);
}

int nros_zephyr_fcntl(int fd, int cmd, int arg) {
    return zsock_fcntl(fd, cmd, arg);
}

ssize_t nros_zephyr_recv(int fd, void* buf, size_t len, int flags) {
    return zsock_recv(fd, buf, len, flags);
}

ssize_t nros_zephyr_recvfrom(int fd, void* buf, size_t len, int flags, struct sockaddr* src_addr,
                             socklen_t* addrlen) {
    return zsock_recvfrom(fd, buf, len, flags, src_addr, addrlen);
}

ssize_t nros_zephyr_send(int fd, const void* buf, size_t len, int flags) {
    return zsock_send(fd, buf, len, flags);
}

ssize_t nros_zephyr_sendto(int fd, const void* buf, size_t len, int flags,
                           const struct sockaddr* dest_addr, socklen_t addrlen) {
    return zsock_sendto(fd, buf, len, flags, dest_addr, addrlen);
}

#endif /* CONFIG_NET_SOCKETS */

/* ── Thread creation with Zephyr-managed stacks ─────────────────────
 *
 * Requires CONFIG_POSIX_API (or equivalent CONFIG_PTHREAD).
 * Only compiled when POSIX threads are available.
 */

#if defined(CONFIG_POSIX_API) || defined(CONFIG_PTHREAD)

#include <zephyr/posix/pthread.h>
#include <zephyr/posix/sched.h>
#include <string.h>
#include <zephyr/sys/printk.h>

#ifndef NROS_ZEPHYR_MAX_THREADS
#define NROS_ZEPHYR_MAX_THREADS 8
#endif
/* Issue 1131 — this is the platform's ENTIRE task pool: `nros_claim_thread_slot`
 * is the only path to a thread here, so at 0 the image can create no task at
 * all, zenoh-pico's read task included. That is issue 0839's failure with the
 * wall at zero instead of four — the image comes up, declares, transmits, and
 * never receives. The producer cannot reach 0 either
 * (`zephyr/CMakeLists.txt` emits the define under `if(CONFIG_...)`, which CMake
 * reads as false at 0, so the C default applies); this is the backstop for a
 * bare `-D`. Below the `#ifndef`, never above: an undefined identifier reads as
 * 0 in `#if`, so a guard above its own default fires on every build (1167). */
#if NROS_ZEPHYR_MAX_THREADS < 1
#error "NROS_ZEPHYR_MAX_THREADS must be >= 1: it sizes a C array (issue 1015)"
#endif

#ifndef NROS_ZEPHYR_STACK_SIZE
#define NROS_ZEPHYR_STACK_SIZE CONFIG_MAIN_STACK_SIZE
#endif

/* Transport/task stacks live in DTCM, not SRAM.
 *
 * The MR-CANHUBK344 has 128 KiB of DTCM and 64 KiB of ITCM that this image was
 * not using at all -- both reported 0 %% while SRAM sat at 98.7 %% and the
 * action image could no longer be instrumented for want of a few KiB. Stacks
 * are the ideal tenant: CPU-private, never a DMA target, and TCM access does
 * not contend with the system bus, so putting them here is a latency
 * improvement as well as an SRAM saving.
 *
 * `NROS_ZEPHYR_STACKS_IN_DTCM` gates it because the region is a property of
 * the SoC, not of Zephyr: a part without a DTCM (or one whose DTCM another
 * component already claims) must keep the default placement. The section name
 * comes from the devicetree `zephyr,memory-region = "DTCM"`, which is what
 * generates the linker region.
 *
 * DTCM is NOLOAD, which is correct for stacks -- they are uninitialised. */
#if defined(NROS_ZEPHYR_STACKS_IN_DTCM) && NROS_ZEPHYR_STACKS_IN_DTCM
Z_KERNEL_STACK_ARRAY_DEFINE_IN(nros_thread_stacks, NROS_ZEPHYR_MAX_THREADS,
                               NROS_ZEPHYR_STACK_SIZE,
                               __attribute__((section("DTCM"))));
#else
K_THREAD_STACK_ARRAY_DEFINE(nros_thread_stacks, NROS_ZEPHYR_MAX_THREADS, NROS_ZEPHYR_STACK_SIZE);
#endif

/* Stack slots are CLAIMED and RELEASED. The counter this replaces
 * (`static int nros_thread_index`, used as `nros_thread_stacks[idx++]`) only
 * ever ROSE, so a slot was spent for the life of the image even after its task
 * had exited. A steady session never noticed -- it creates its tasks once. A
 * RECONNECT is what spends them, and zenoh-pico reconnects on every lease
 * expiry, so the image walked out of slots and then could not reopen at all:
 *
 *     OUT OF THREAD SLOTS (NROS_ZEPHYR_MAX_THREADS=8) -- task not created
 *     ERROR ::_zp_unicast_failed] Reopen failed: -79
 *
 * observed on the action-server image at the third and fourth reconnect
 * (issue 0839). Raising the count only moves that wall further out.
 *
 * RELEASE IS ON JOIN, NOT ON CREATE-FAILURE-OR-EXIT, and that is deliberate:
 * `pthread_join` returning is proof the thread is gone, so its stack cannot
 * still be in use. A teardown that DETACHES instead of joining does not
 * release -- handing a live thread's stack to the next task would be a worse
 * bug than the leak. Same rule, and same reasoning, as the zenoh-pico fix in
 * issue 0822. */
static struct {
    pthread_t owner;
    bool in_use;
} nros_thread_slots[NROS_ZEPHYR_MAX_THREADS];
static K_MUTEX_DEFINE(nros_thread_slots_lock);

static int nros_claim_thread_slot(void) {
    int slot = -1;
    (void) k_mutex_lock(&nros_thread_slots_lock, K_FOREVER);
    for (int i = 0; i < NROS_ZEPHYR_MAX_THREADS; i++) {
        if (!nros_thread_slots[i].in_use) {
            nros_thread_slots[i].in_use = true;
            /* Clear the previous occupant before dropping the lock: the
             * matcher keys on owner, and a claimed-but-not-yet-owned slot
             * carrying a dead tid could be matched by a concurrent join. */
            (void) memset(&nros_thread_slots[i].owner, 0,
                          sizeof(nros_thread_slots[i].owner));
            slot = i;
            break;
        }
    }
    (void) k_mutex_unlock(&nros_thread_slots_lock);
    return slot;
}

static void nros_set_thread_slot_owner(int slot, pthread_t owner) {
    (void) k_mutex_lock(&nros_thread_slots_lock, K_FOREVER);
    nros_thread_slots[slot].owner = owner;
    (void) k_mutex_unlock(&nros_thread_slots_lock);
}

static void nros_free_thread_slot(int slot) {
    (void) k_mutex_lock(&nros_thread_slots_lock, K_FOREVER);
    nros_thread_slots[slot].in_use = false;
    (void) k_mutex_unlock(&nros_thread_slots_lock);
}

/* Called from `_z_task_join` once the join has RETURNED. */
void nros_zephyr_task_slot_release(pthread_t owner) {
    pthread_t none;
    (void) memset(&none, 0, sizeof(none));
    (void) k_mutex_lock(&nros_thread_slots_lock, K_FOREVER);
    for (int i = 0; i < NROS_ZEPHYR_MAX_THREADS; i++) {
        if (nros_thread_slots[i].in_use &&
            pthread_equal(nros_thread_slots[i].owner, none) == 0 &&
            pthread_equal(nros_thread_slots[i].owner, owner) != 0) {
            nros_thread_slots[i].in_use = false;
            break;
        }
    }
    (void) k_mutex_unlock(&nros_thread_slots_lock);
}

/* issue 0852 — the native SCHED_FIFO priority to be born with, or < 0 for
 * "inherit the creator's", which is what every caller got before this issue.
 *
 * `PTHREAD_EXPLICIT_SCHED` is the load-bearing half. Zephyr's
 * `pthread_attr_init` sets `PTHREAD_INHERIT_SCHED`
 * (zephyr/lib/posix/options/pthread.c), under which the policy and param set
 * here are IGNORED and the child silently takes the creator's priority. That
 * silent inheritance is the whole of issue 0852: the zenoh READ task was born
 * at `CONFIG_MAIN_STACK_PRIORITY` because the executor started it, landing it
 * at the executor's own priority, where `k_yield()` in the polled serial read
 * loop costs a full `CONFIG_TIMESLICE_SIZE` (20 ms, ~230 bytes at 115200)
 * instead of returning promptly. The UART overran and the frames it dropped
 * were the router's keepalives.
 *
 * Issue 0626 fixed the same drop one layer up, in `zpico_set_task_config`, and
 * its comment says a quietly-dropped scheduling attribute "must not be
 * reintroduced one layer down". It was, here. */
int nros_zephyr_task_create_prio(pthread_t* thread, void* (*entry)(void*), void* arg,
                                 int native_priority, const char* name) {
    int slot = nros_claim_thread_slot();
    if (slot < 0) {
        /* Say so. This used to return -1 silently, and the caller that loses
         * is usually zenoh-pico's READ task -- so the image comes up, declares
         * every entity, transmits happily, and then simply never receives
         * anything. The first visible symptom is the session expiring one
         * lease later, which points the reader at keepalives and the link
         * rather than at a task that was never created (issue 0839, where an
         * action image needed more tasks than the default four slots and the
         * only clue was 26 transmits against 4 receives).
         *
         * printk, not LOG_ERR: this runs before any log backend is guaranteed
         * up, and a silent failure here costs hours. */
        printk("nros: OUT OF THREAD SLOTS (NROS_ZEPHYR_MAX_THREADS=%d) -- task not created.\n"
               "nros: raise CONFIG_NROS_ZEPHYR_TASK_SLOTS; expect missing rx and session expiry.\n",
               (int) NROS_ZEPHYR_MAX_THREADS);
        return -1;
    }

    pthread_attr_t attr;
    (void)pthread_attr_init(&attr);
    (void)pthread_attr_setstack(&attr, &nros_thread_stacks[slot], NROS_ZEPHYR_STACK_SIZE);

    /* Best-effort, and deliberately so: a kernel that declines the policy
     * leaves the task at the inherited priority, which is exactly what it did
     * before this existed. A refusal must not turn into a spawn failure. */
    if (native_priority >= 0) {
        struct sched_param sp;
        (void)memset(&sp, 0, sizeof(sp));
        sp.sched_priority = native_priority;
        /* SCHED_RR, not SCHED_FIFO: on Zephyr SCHED_FIFO selects the
         * COOPERATIVE band, and a cooperative busy-poller starves every
         * preemptible thread. See `nros_zephyr_native_priority` in
         * nros-platform-zephyr/src/platform.c, which computes the value
         * passed in here against this same policy. */
        if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) == 0 &&
            pthread_attr_setschedpolicy(&attr, SCHED_RR) == 0) {
            (void)pthread_attr_setschedparam(&attr, &sp);
        }
    }

    int ret = pthread_create(thread, &attr, entry, arg);
    (void)pthread_attr_destroy(&attr);
    if (ret != 0) {
        nros_free_thread_slot(slot);
        return ret;
    }

    /* The name the caller asked for, applied to the kernel's own thread table.
     *
     * Best-effort for the same reason the priority above is: a kernel that
     * declines the name leaves the thread anonymous, which is exactly what it
     * was before this existed. A refusal must not turn into a spawn failure.
     *
     * `pthread_setname_np` self-gates on CONFIG_THREAD_NAME and returns 0 when
     * it is off, so there is no caller-side #ifdef to keep in step. */
    if (name != NULL) {
        (void)pthread_setname_np(*thread, name);
    }

    nros_set_thread_slot_owner(slot, *thread);
    return 0;
}

/* The pre-issue-0852 spelling. Kept so callers that have no priority to state
 * do not have to invent one; "inherit" is what they always got. */
int nros_zephyr_task_create(pthread_t* thread, void* (*entry)(void*), void* arg) {
    return nros_zephyr_task_create_prio(thread, entry, arg, -1, NULL);
}

#endif /* CONFIG_POSIX_API || CONFIG_PTHREAD */

/* ── errno read helper (Phase 92.5 diagnostic) ──────────────────────
 *
 * Zephyr's `errno` is thread-local and lives behind a per-thread
 * pointer that's only accessible through the `errno` macro. Rust
 * callers can't expand the macro, so wrap it here.
 */
#include <errno.h>
int nros_zephyr_errno(void) {
    return errno;
}

/* ── critical-section wrappers (Phase 71.6) ────────────────────────
 *
 * Zephyr's `irq_lock()` / `irq_unlock()` are static inline macros with
 * no exported symbols. nros-c / nros-cpp's Rust-side critical-section
 * impl needs real linkable symbols to call, so wrap them here.
 *
 * Used by the C/C++ API path on platform-zephyr to satisfy
 * `_critical_section_1_0_acquire` / `_critical_section_1_0_release`
 * referenced from dust-dds + portable-atomic when the
 * zephyr-lang-rust crate (which provides its own impl) isn't linked.
 */
unsigned int nros_zephyr_irq_lock(void) {
    return irq_lock();
}

void nros_zephyr_irq_unlock(unsigned int key) {
    irq_unlock(key);
}

/* phase-243 — the nros_platform_time_ns / sleep_ns exported wrappers are retired.
 * nros-c's no_std path (platform-zephyr) now uses the canonical
 * (nros_platform_clock_ns() / 1000ULL) / sleep_us() (nros-platform-zephyr provides them), so
 * no Rust caller needs the ns symbols here anymore. */

/* ── Phase 97.4.zephyr-native_sim debug printk shims ─────────────────
 *
 * Rust extern "C" can't directly call variadic `printk`. Provide
 * non-variadic wrappers per shape. Always exported; Rust call sites
 * are cfg-gated behind feature flags.
 */
void nros_zephyr_log(const char* msg) {
    printk("[nros] %s\n", msg);
}

void nros_zephyr_log_int(const char* tag, int64_t v) {
    printk("[nros] %s=%lld\n", tag, (long long)v);
}

void nros_zephyr_log_2int(const char* tag, int64_t a, int64_t b) {
    printk("[nros] %s=%lld,%lld\n", tag, (long long)a, (long long)b);
}

/* ── RT-tier task spawn (issue #128 / RFC-0015 Model 1) ─────────────
 *
 * One `k_thread` per priority tier, RAW Zephyr priority (negatives =
 * cooperative — the `[tiers.<name>.zephyr].priority` value verbatim,
 * which the POSIX pthread shim above cannot express). Static pool:
 * tier count is a compile-time property of the baked system, so a
 * small fixed pool avoids CONFIG_DYNAMIC_THREAD. C-ABI-shaped so the
 * phase-274 W3 C/C++ zephyr `run_tiers` can reuse it.
 */

#ifndef NROS_ZEPHYR_MAX_TIERS
#define NROS_ZEPHYR_MAX_TIERS 4
#endif

#ifndef NROS_ZEPHYR_TIER_STACK_SIZE
#define NROS_ZEPHYR_TIER_STACK_SIZE 16384
#endif

K_THREAD_STACK_ARRAY_DEFINE(nros_tier_stacks, NROS_ZEPHYR_MAX_TIERS, NROS_ZEPHYR_TIER_STACK_SIZE);
static struct k_thread nros_tier_threads[NROS_ZEPHYR_MAX_TIERS];
static int nros_tier_index;

static void nros_zephyr_tier_trampoline(void* entry, void* arg, void* unused) {
    (void)unused;
    printk("[nros] tier task entered\n");
    void* (*fn)(void*) = (void* (*)(void*))entry;
    (void)fn(arg);
    printk("[nros] tier task RETURNED (unexpected)\n");
}

/**
 * Spawn one tier task. `entry(arg)` runs on a pool thread at the RAW
 * Zephyr `priority` (cooperative if negative). `name` is the thread's
 * debug name (may be NULL). `stack_bytes` is the tier's DECLARED stack
 * (0 = undeclared): the pool slots are compile-time sized
 * (K_THREAD_STACK_ARRAY_DEFINE), so the request cannot resize them — a
 * request past the slot prints LOUD and the tier runs on the slot
 * anyway (phase-302 W2 / issue 0262: previously the knob was silently
 * ignored). Raise NROS_ZEPHYR_TIER_STACK_SIZE to honor bigger tiers.
 * Returns 0 on success, -1 when the pool is exhausted (more than
 * NROS_ZEPHYR_MAX_TIERS spawns).
 */
int nros_zephyr_tier_task_create(void* (*entry)(void*), void* arg, int32_t priority,
                                 const char* name, size_t stack_bytes, uint32_t core_plus1,
                                 int* pin_rc) {
    if (entry == NULL || nros_tier_index >= NROS_ZEPHYR_MAX_TIERS) {
        return -1;
    }
    if (stack_bytes > (size_t)NROS_ZEPHYR_TIER_STACK_SIZE) {
        printk("nros: tier stack request %u > fixed slot %u tier=`%s` — running with the "
               "slot; raise NROS_ZEPHYR_TIER_STACK_SIZE\n",
               (unsigned)stack_bytes, (unsigned)NROS_ZEPHYR_TIER_STACK_SIZE,
               (name != NULL) ? name : "?");
    }
    int idx = nros_tier_index++;

    /* issue 0655 — a tier that declares a `core` is created SUSPENDED
     * (`K_FOREVER`), pinned, then started.
     *
     * Zephyr's cpu mask is settable only on a thread that is "prevented from
     * running": `cpu_mask_mod` returns -EINVAL otherwise, and under
     * CONFIG_SCHED_CPU_MASK_PIN_ONLY it additionally asserts
     * "Running threads cannot change CPU pin". The previous code created every
     * tier with K_NO_WAIT and then had the tier pin ITSELF from its own entry
     * — i.e. always from a running thread — so the accept arm could not be
     * reached on any image, SMP or not. Creating suspended is the whole fix:
     * the window between create and start is the only place this API works.
     *
     * An undeclared core keeps K_NO_WAIT, so the common path is unchanged and
     * no tier pays a start-up round trip for a knob it does not use. */
    k_timeout_t start_delay = (core_plus1 != 0u) ? K_FOREVER : K_NO_WAIT;
    k_tid_t tid = k_thread_create(&nros_tier_threads[idx], nros_tier_stacks[idx],
                                  NROS_ZEPHYR_TIER_STACK_SIZE, nros_zephyr_tier_trampoline,
                                  (void*)entry, arg, NULL, (int)priority, 0, start_delay);
    if (tid == NULL) {
        return -1;
    }
    if (name != NULL) {
        (void)k_thread_name_set(tid, name);
    }
    if (core_plus1 != 0u) {
        /* Report the kernel's own return code to the caller, which owns the
         * accept/fallback marker text (it must stay in lockstep with
         * `nros_tests::output::ZEPHYR_CORE_PIN_*`). The pin is attempted
         * whether or not it succeeds, and the thread starts either way — a
         * tier that cannot be pinned still runs, unpinned and loudly, which
         * is the RFC-0052 fail-loud contract. */
        int rc = nros_zephyr_thread_cpu_pin_tid(tid, (int)(core_plus1 - 1u));
        if (pin_rc != NULL) {
            *pin_rc = rc;
        }
        k_thread_start(tid);
    }
    return 0;
}

/**
 * Set the CALLING thread's priority to a raw Zephyr priority. The tier
 * boot thread (`rust_main`) runs `tiers[0]` itself, so it must adopt that
 * tier's declared priority instead of keeping the main-thread default.
 */
void nros_zephyr_set_current_priority(int32_t priority) {
    k_thread_priority_set(k_current_get(), (int)priority);
}

/**
 * phase-296 W5.5 — apply a per-thread earliest-deadline (µs) on the CALLING
 * thread. `k_thread_deadline_set` takes CYCLES; convert from µs. Compiled to a
 * no-op when the kernel lacks EDF (`CONFIG_SCHED_DEADLINE`) so the image still
 * links; the Rust caller (`entry_tiers::apply_tier_deadline`) additionally gates
 * the CALL behind the `zephyr-edf` feature, so a no-op here means an honest
 * fall-through to the executor's cooperative deadline monitor.
 *
 * NOTE: this lives here (the Zephyr-module C shims, linked by BOTH the pure-Rust
 * `ZephyrBoard::run_tiers` image and the C/C++ `nros_board_zephyr_run_tiers`
 * path) rather than in `c/zephyr_run_tiers.c` — that file is compiled only into
 * the C/C++ entry image, so a definition there is invisible to the Rust link.
 *
 * Returns 1 when the kernel actually applied the deadline (EDF present) and 0
 * when it was a no-op (`CONFIG_SCHED_DEADLINE` unset) — the Rust caller logs
 * its "EDF deadline set" marker ONLY on a 1, so the marker can never fire
 * from an image where the kernel never applied anything.
 */
int nros_zephyr_set_current_deadline(unsigned int deadline_us) {
#ifdef CONFIG_SCHED_DEADLINE
    k_thread_deadline_set(k_current_get(), (int)k_us_to_cyc_near32(deadline_us));
    return 1; /* applied — kernel EDF present */
#else
    (void)deadline_us;
    return 0; /* not applied — no kernel EDF; executor monitor is sole enforcement */
#endif
}
