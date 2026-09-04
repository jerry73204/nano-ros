/**
 * zpico-sys Core Implementation
 *
 * This file implements the zpico API using zenoh-pico.
 * Platform-specific behavior is handled by zenoh-pico's platform layer.
 *
 * zpico provides a simplified C API that hides zenoh-pico's complex
 * ownership types from Rust FFI (avoiding struct size mismatch issues).
 */

#include "zpico.h"
#include <zenoh-pico.h>
#include <zenoh-pico/session/query.h>
#include <zenoh-pico/api/olv_macros.h>
#include "zpico_config_keys.h"
/* Phase 154 — `nros_platform_socket_get_fd` accessor for the
 * `get_session_fd` helper (used by ThreadX-Linux's
 * `select`-driven read-task wakeup path). Replaces the
 * `peer->_socket._fd` field access that no longer compiles
 * once `NROS_PLATFORM_ALIASES` is defined on the vendor build. */
#include <nros/platform.h>
#include <nros/platform_net.h>
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include <time.h>
#if defined(__linux__)
#include <sys/random.h>
#include <unistd.h>
#endif
#ifdef ZENOH_NUTTX
#include <unistd.h>
#endif

#ifdef ZENOH_ZEPHYR
#include <zephyr/kernel.h> // For printk
#include <zephyr/random/random.h>
// Issue 0626 — pthread scheduling attributes for the read/lease tasks.
#include <pthread.h>
#include <sched.h>
#if defined(CONFIG_POSIX_MULTI_PROCESS)
#include <zephyr/posix/unistd.h>
#endif
#elif defined(ZENOH_FREERTOS_LWIP)
// On FreeRTOS, route printk to semihosting for debug output.
// Uses SYS_WRITE0 semihosting call (null-terminated string to stdout).
#include <stdio.h>
static void _freertos_printk(const char* fmt, ...) {
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    // ARM semihosting SYS_WRITE0 (op=0x04): write null-terminated string.
    // ARM-only (the `r0`/`r1` register binds + `bkpt` are invalid elsewhere);
    // guarded so this TU also compiles when zpico-sys is built for a host target
    // (phase-243 — surfaced when the freertos config TU is host-compiled).
#if defined(__arm__) || defined(__thumb__)
    register unsigned r0 __asm__("r0") = 0x04;
    register const char* r1 __asm__("r1") = buf;
    __asm__ volatile("bkpt #0xAB" : : "r"(r0), "r"(r1) : "memory");
#else
    (void)buf;
#endif
}
#define printk(...) _freertos_printk(__VA_ARGS__)
#elif defined(ZENOH_THREADX)
// On ThreadX route printk through printf (Linux sim) or uart_puts (bare-metal)
#include <stdio.h>
#if defined(__linux__)
#include <sys/select.h>
#define printk(...) printf(__VA_ARGS__)
#else
#include "nxd_bsd.h"
extern void uart_puts(const char* s);
static void _threadx_printk(const char* fmt, ...) {
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uart_puts(buf);
}
#define printk(...) _threadx_printk(__VA_ARGS__)
#endif
#elif defined(ZENOH_NUTTX)
// Issue 0870 — NuttX had NO arm here, so it fell to the no-op below and every
// diagnostic in this file compiled away. That is not a platform limitation:
// NuttX has full POSIX stdio and this TU already includes <unistd.h> for it
// above. The cost was real -- two messages that name a fault outright never
// reached the console, including the one whose own comment calls a failed
// liveliness token "a SILENT graph outage ... say so on the console":
//
//     zpico: z_declare_subscriber (ring) failed: %d for '%s'
//     zpico: z_liveliness_declare_token failed: %d for '%s'
//
// so a NuttX failure could only ever be read through the collapsed Rust-side
// return code, which is how 0870 spent three diagnoses on the wrong layer.
#include <stdio.h>
#define printk(...) printf(__VA_ARGS__)
#elif defined(ZPICO_SMOLTCP) || defined(ZPICO_SERIAL)
#define printk(...) // No libc printf on bare-metal
#elif defined(ZENOH_LINUX) || defined(ZENOH_MACOS)
// Same hole issue 0870 found on NuttX, one arm over: the hosted POSIX build
// defines ZENOH_LINUX (or ZENOH_MACOS) and NONE of the arms above, so it fell
// to the no-op below and every diagnostic in this TU compiled away on the one
// platform where a developer is most likely to be reading them. A host has
// full stdio; there is no reason for it to be the quietest target in the tree.
#include <stdio.h>
#define printk(...) printf(__VA_ARGS__)
#else
#define printk(...) // No-op on other platforms
#endif

// Internal zenoh-pico headers for socket FD access (select()-based timeout).
// Needed for single-threaded builds and for ThreadX/NSOS, where we deliberately
// drive read + keepalive from zpico_spin_once instead of background tasks.
#if defined(ZENOH_THREADX)
#include "nxd_bsd.h"
#endif
#if !defined(ZPICO_SMOLTCP) && !defined(ZPICO_SERIAL) && !defined(ZENOH_FREERTOS_LWIP) &&          \
    !defined(ZENOH_THREADX) && Z_FEATURE_MULTI_THREAD != 1
#include "zenoh-pico/net/session.h"
#include "zenoh-pico/transport/transport.h"
#include "zenoh-pico/api/olv_macros.h"
#include <sys/select.h>
#elif defined(ZENOH_THREADX)
#include "zenoh-pico/net/session.h"
#include "zenoh-pico/transport/transport.h"
#include "zenoh-pico/api/olv_macros.h"
#endif

// ============================================================================
// Platform-Specific Declarations
// ============================================================================

#ifdef ZPICO_SMOLTCP
// External Rust FFI functions for smoltcp platform
extern int32_t smoltcp_init(void);
extern void smoltcp_cleanup(void);
extern uint64_t smoltcp_clock_now_ms(void);
#endif

// ============================================================================
// Internal Data Structures
// ============================================================================

// Subscriber entry with callback (supports legacy, attachment, and direct-write modes)
typedef struct {
    z_owned_subscriber_t subscriber;
    union {
        ZpicoCallback callback;                   // Legacy callback (payload only)
        ZpicoCallbackWithAttachment callback_ext; // Extended callback (with attachment)
        ZpicoNotifyCallback notify;               // Direct-write notify (len + attachment)
    };
    void* ctx;
    bool active;
    bool with_attachment; // true = use callback_ext, false = use callback
    // Direct-write fields (set when mode == direct_write)
    bool direct_write;      // true = direct-write mode
    uint8_t* buf_ptr;       // Pointer into Rust SUBSCRIBER_BUFFERS[i].data
    size_t buf_capacity;    // Size of the Rust buffer
    const bool* locked_ptr; // Pointer to Rust SUBSCRIBER_BUFFERS[i].locked (AtomicBool)
    // Phase 124.D.3.c — SPSC ring descriptor (set when mode == ring).
    // C is the sole producer, Rust the sole consumer. See
    // `zpico_ring_desc_t` in the header. NULL = not in ring mode.
    bool ring_mode;
    zpico_ring_desc_t* ring;
#if defined(Z_FEATURE_UNSTABLE_API)
    bool zero_copy; // true = zero-copy mode (borrows from zenoh-pico buffer)
    ZpicoZeroCopyCallback zero_copy_cb;
#endif
} subscriber_entry_t;

// Publisher entry
typedef struct {
    z_owned_publisher_t publisher;
    bool active;
} publisher_entry_t;

// Liveliness token entry
typedef struct {
    z_owned_liveliness_token_t token;
    bool active;
} liveliness_entry_t;

// Queryable entry for service servers
typedef struct {
    z_owned_queryable_t queryable;
    ZpicoQueryCallback callback;
    void* ctx;
    bool active;
} queryable_entry_t;

// ZPICO_MAX_PUBLISHERS, ZPICO_MAX_SUBSCRIBERS, ZPICO_MAX_QUERYABLES,
// ZPICO_MAX_LIVELINESS, ZPICO_GET_REPLY_BUF_SIZE, and ZPICO_GET_POLL_INTERVAL_MS
// are provided via -D compiler flags from build.rs (configurable with env vars).
// Defaults are provided here for non-Cargo build paths (e.g., Zephyr CMake).
#ifndef ZPICO_MAX_PUBLISHERS
#define ZPICO_MAX_PUBLISHERS 8
#endif
#ifndef ZPICO_MAX_SUBSCRIBERS
#define ZPICO_MAX_SUBSCRIBERS 8
#endif
#ifndef ZPICO_MAX_QUERYABLES
#define ZPICO_MAX_QUERYABLES 8
#endif
#ifndef ZPICO_MAX_LIVELINESS
#define ZPICO_MAX_LIVELINESS 16
#endif
/* Issue 0626 — the transport tasks' priority, on the NORMALISED 0-31 band
 * (0 = least urgent, larger = more urgent; phase-364 W5). 16 keeps the historic
 * behaviour of the one platform that already stated a value: it is what the
 * FreeRTOS board's `zenoh_read_priority` default has always been.
 *
 * This is a default, not a policy. Whether transport should outrank the
 * application's RT tiers is a per-system decision with real costs on both sides
 * (issues 0506 and 0623) — the point of these knobs is that the decision can be
 * MADE, which on Zephyr it previously could not be. */
#ifndef ZPICO_READ_TASK_PRIORITY
#define ZPICO_READ_TASK_PRIORITY 16
#endif
#ifndef ZPICO_LEASE_TASK_PRIORITY
#define ZPICO_LEASE_TASK_PRIORITY 16
#endif
#ifndef ZPICO_GRAPH_CACHE_SIZE
/* Bytes of NUL-separated liveliness keyexprs the graph cache holds.
 *
 * A ROS 2 liveliness keyexpr runs ~140 bytes. Measured against a host running
 * one stock `talker` plus the ROS 2 daemon, the domain carried 222 tokens —
 * ~31 KB — so 8192 held 51 of them and dropped 171. Sized for that measurement
 * rather than for the talker alone, and still a KNOB: an embedded image with a
 * small graph should turn it down. Overflow is COUNTED, never silently
 * truncated —
 * `dropped` is what `zpico_graph_cache_copy` reports, because a graph answer
 * that quietly omits entries is the plausible-wrong-answer failure phase-381
 * exists to avoid. */
#define ZPICO_GRAPH_CACHE_SIZE 65536
#endif

#ifndef ZPICO_GET_REPLY_BUF_SIZE
#define ZPICO_GET_REPLY_BUF_SIZE 4096
#endif
#ifndef ZPICO_GET_POLL_INTERVAL_MS
#define ZPICO_GET_POLL_INTERVAL_MS 10
#endif
#ifndef ZPICO_MAX_PENDING_GETS
#define ZPICO_MAX_PENDING_GETS 4
#endif

// issue 0348 / phase-328 — the shim is multi-session (handle-passing). Every
// per-session zenoh object that used to be a file-scope static (config, the
// session handle, the lifecycle flags, and the registration tables) now lives
// in `struct zpico_session`, defined below after the reply-slot typedefs. Every
// `zpico_*` entry point takes a leading `zpico_session_t*` handle into the
// compile-time session pool.

// Phase 237 — per-queryable seq-keyed reply slots. A reply can be sent long
// after the query callback returned (an action `get_result` held until the goal
// terminates), and concurrent goals mean several queries may be outstanding at
// once, so a single stored query per queryable would be overwritten. Each slot
// holds one cloned (owned) query; `query_handler` allocates a free slot and
// records its index as the reply seq, `zpico_query_reply(handle, seq)` consumes
// it. Override for servers fielding more concurrent in-flight requests.
#ifndef ZPICO_MAX_PENDING_REPLIES
#define ZPICO_MAX_PENDING_REPLIES 4
#endif
// `stored_queries` / `stored_query_valid` / `last_reply_seq` are per-session —
// they live in `struct zpico_session` (defined below). The index of the slot
// allocated by the most recent `query_handler` invocation is read by
// `zpico_queryable_take_reply_seq` from inside the (synchronous) user callback;
// -1 if the table was full (the reply is dropped — back-pressure).
#if defined(ZPICO_SMOLTCP) || defined(ZPICO_SERIAL)
static uint32_t g_session_zid_counter;
#else
static atomic_uint g_session_zid_counter;
#endif

// Context struct for blocking z_get reply (stack-allocated per call)
// ZPICO_GET_REPLY_BUF_SIZE is provided via -D compiler flag from build.rs

typedef struct {
    uint8_t buf[ZPICO_GET_REPLY_BUF_SIZE];
    size_t len;
    /* Phase 127.D — atomic+volatile so fat-LTO doesn't hoist the
     * read in `zpico_get_check` over the write inside
     * `get_reply_handler` (callback fired by zenoh-pico's RX path
     * during `zp_read`, which sits between two polls of the same
     * struct field). Plain volatile alone was not enough under
     * `lto = "fat"` + `opt-level = "s"` on cortex-m3. Use the GCC
     * `__atomic_*` builtins so the access is a real load/store
     * even after whole-program inlining. */
    _Atomic bool received;
    _Atomic bool done;
    /* Phase 108.C.zenoh.4-followup — counts every reply that arrives.
     * Single-response queries (`zpico_get*`) use `received` for "first
     * reply only" semantics; multi-response queries (liveliness) read
     * `reply_count` to learn how many distinct tokens responded. The
     * count is incremented in `get_reply_handler` regardless of
     * payload buffer occupancy. */
    uint32_t reply_count;
    /* phase-381 W1 — GRAPH ENUMERATION.
     *
     * A liveliness reply carries its information in the KEYEXPR, not the
     * payload: `@ros2_lv/<domain>/<zid>/<nid>/<eid>/…/<namespace>/<node>`. The
     * handler used to discard it and keep only `reply_count`, so the shim could
     * answer "does anything match" and never "what matched".
     *
     * When `collect` is set, the handler appends each reply's keyexpr into
     * `buf` as NUL-terminated strings instead of copying a payload there.
     * That buffer is otherwise DEAD on a liveliness query — the payload of a
     * liveliness token is empty, and the pre-W1 comment on
     * `zpico_liveliness_get_start` says so: "the reply handler still copies the
     * (typically empty) liveliness token bytes into the slot's buffer; we never
     * read them". So enumeration costs no new storage, which is what phase-381's
     * deferred bounded-memory decision turned out to be about.
     *
     * `entry_count` counts what was STORED; `reply_count` keeps counting what
     * ARRIVED. The difference is the truncation the caller must be able to see —
     * a graph query that silently drops entries is the plausible-wrong-answer
     * failure this phase exists to avoid. */
    bool collect;
    uint32_t entry_count;
#if Z_FEATURE_MULTI_THREAD == 1
    _z_mutex_t mutex;
    _z_condvar_t cond;
#endif
} get_reply_ctx_t;

/* phase-381 / issue 0903 — the GRAPH CACHE.
 *
 * Enumeration used to be a `z_liveliness_get` per question. That is an INTEREST
 * declaration under the hood (`_z_liveliness_query` sends TOKENS | KEYEXPRS |
 * RESTRICTED | CURRENT), and a token only reaches a get's callback when the
 * router tags its declaration with THAT interest id. Measured against a live
 * `rmw_zenoh_cpp` talker, two sweeps in flight did not both receive replies —
 * whichever started first got tokens and the other got `arrived=0` across 99
 * drains — and a single sweep saw 2-4 of the dozen tokens the talker declares,
 * a different subset each run. Widening the pattern to `**` made it worse.
 *
 * A SUBSCRIBER with `history` is what `rmw_zenoh_cpp` itself uses for its graph
 * cache, and it removes the whole class: one standing declaration, the current
 * tokens delivered once, every later change pushed. No sweeps, so nothing to
 * serialize, nothing to starve, and no per-sweep truncation. */
typedef struct {
    /* NUL-separated keyexprs, same layout `zpico_entry_at` already walks. */
    uint8_t buf[ZPICO_GRAPH_CACHE_SIZE];
    size_t len;
    uint32_t entry_count;
    /* Tokens that did not fit. Reported, never hidden. */
    uint32_t dropped;
    bool active;
    z_owned_subscriber_t sub;
#if Z_FEATURE_MULTI_THREAD == 1
    _z_mutex_t mutex;
#endif
} graph_cache_t;

// Static slots for non-blocking z_get operations
// ZPICO_MAX_PENDING_GETS is provided via -D compiler flag from build.rs,
// with a default fallback above for non-Cargo build paths.
/* Issue 1015 -- these three pools are FIXED C ARRAYS, so zero is not a smaller
 * pool: a zero-length array is a GNU extension that compiles silently, and
 * none of these is the last member of its struct. A derived 0 (an image that
 * declares no service servers derives exactly that) produced a board that
 * transmitted NOTHING -- no panic, no log, core in WFI -- while every gate
 * stayed green, because the value was derived correctly and delivered
 * faithfully. It was simply not a usable size.
 *
 * The derivation floors these at 1 (nros-cli-core entity_inventory.rs). This
 * is the second line, here rather than there because a floor in one producer
 * does not bind another, and this is where the array actually is. */
#if ZPICO_MAX_QUERYABLES < 1
#error "ZPICO_MAX_QUERYABLES must be >= 1: it sizes a C array (issue 1015)"
#endif
#if ZPICO_MAX_SUBSCRIBERS < 1
#error "ZPICO_MAX_SUBSCRIBERS must be >= 1: it sizes a C array (issue 1015)"
#endif
#if ZPICO_MAX_PUBLISHERS < 1
#error "ZPICO_MAX_PUBLISHERS must be >= 1: it sizes a C array (issue 1015)"
#endif

typedef struct {
    get_reply_ctx_t ctx;
    bool in_use;
} pending_get_slot_t;

// The `pending_gets` slot pool is per-session — see `struct zpico_session` below.

// Phase 127.D — diagnostic counters for reply-dispatch debugging.
// Read via `zpico_get_diag_counters` from Rust.
static volatile uint32_t g_diag_reply_handler_calls = 0;
static volatile uint32_t g_diag_reply_dropper_calls = 0;
static volatile uint32_t g_diag_get_start_calls = 0;
static volatile uint32_t g_diag_get_check_calls = 0;
static volatile uint32_t g_diag_get_check_returns_data = 0;
static volatile uint32_t g_diag_reply_not_ok = 0;
static volatile uint32_t g_diag_reply_to_slice_fail = 0;
static volatile uint32_t g_diag_reply_too_big = 0;
static volatile uint32_t g_diag_reply_already_received = 0;
static volatile uint32_t g_diag_reply_received_set = 0;
static volatile uint32_t g_diag_gck_invalid_arg = 0;
static volatile uint32_t g_diag_gck_not_in_use = 0;
static volatile uint32_t g_diag_gck_too_big = 0;
static volatile uint32_t g_diag_gck_timeout = 0;
static volatile uint32_t g_diag_gck_pending = 0;
static volatile uint32_t g_diag_handler_ctx_addr = 0;
static volatile uint32_t g_diag_check_ctx_addr = 0;
static volatile uint32_t g_diag_start_ctx_addr = 0;

// Reply waker callback — invoked when a pending get slot receives a reply
// or times out, allowing Rust async code to wake the corresponding Future.
// Per-session (stored in `struct zpico_session.reply_waker`); set via
// `zpico_set_reply_waker(s, fn)`. phase-328/#376 — carries the owning session's
// pool index alongside the slot so the Rust waker table can be session-scoped
// (two sessions' pending-get slot N must not wake each other's future).
typedef void (*zpico_waker_fn)(int32_t session_index, int32_t slot);

// ============================================================================
// Session pool (issue 0348 / phase-328 — multi-session handle-passing)
// ============================================================================
//
// Every per-session zenoh object that used to be a file-scope static now lives
// in `struct zpico_session`, held in a compile-time pool (`ZPICO_MAX_SESSIONS`,
// default 1 so single-session targets keep TODAY's footprint plus one `bool`
// per slot). No heap — embedded targets do not assume `malloc`. The diagnostic
// counters and the ZID uniquifier stay process-global (below / above).
//
// The generated header (zpico.h) exposes `zpico_session_t` as an opaque
// 0-length stub and every `zpico_*` prototype takes a `zpico_session_t*`. The
// concrete storage therefore lives under a DISTINCT tag `struct zpico_session`;
// each entry point reinterprets its `zpico_session_t*` handle as a
// `struct zpico_session*`.

// Some struct members need platform types whose headers were previously pulled
// in inside the spin-primitive section; hoist those includes above the struct.
#if Z_FEATURE_MULTI_THREAD == 1 && !defined(ZPICO_SMOLTCP)
#if defined(ZENOH_FREERTOS_LWIP)
#include <FreeRTOS.h>
#include <semphr.h>
#elif defined(ZENOH_NUTTX)
#include <semaphore.h>
#include <errno.h>
#endif
#endif

// phase-279 (#145) — dedicated tx-flush thread selector. Defined here (ahead of
// its previous site) because `struct zpico_session` gates its flush-task
// members on it. On multi-threaded, batching-on, non-ThreadX builds a dedicated
// low-duty thread absorbs the socket waits; ThreadX keeps the spin-driven flush
// (it runs no background tasks — read-task starvation, see zpico_open);
// single-threaded platforms have no tasks at all.
#if defined(ZPICO_TX_BATCH) && ZPICO_TX_BATCH == 1 && Z_FEATURE_MULTI_THREAD == 1 &&               \
    !defined(ZENOH_THREADX)
#define ZPICO_TX_BATCH_THREAD 1
#else
#define ZPICO_TX_BATCH_THREAD 0
#endif
#ifndef ZPICO_TX_BATCH_FLUSH_MS
#define ZPICO_TX_BATCH_FLUSH_MS 50
#endif

#ifndef ZPICO_MAX_SESSIONS
#define ZPICO_MAX_SESSIONS 1 // single-session targets keep TODAY's footprint
#endif

struct zpico_session {
    // Session handle + config + lifecycle (per handle — issue 0347's
    // single-session guard is gone; each handle is independent).
    z_owned_config_t config;
    z_owned_session_t session;
    bool session_open;
    bool initialized;

    // Registration tables.
    publisher_entry_t publishers[ZPICO_MAX_PUBLISHERS];
    subscriber_entry_t subscribers[ZPICO_MAX_SUBSCRIBERS];
    liveliness_entry_t liveliness[ZPICO_MAX_LIVELINESS];
    graph_cache_t graph_cache;
    queryable_entry_t queryables[ZPICO_MAX_QUERYABLES];

    // Phase 237 — per-queryable seq-keyed reply slots. Each slot holds one
    // cloned (owned) query so a reply can be sent after the query callback
    // returned (deferred get_result) even if more queries land meanwhile.
    z_owned_query_t stored_queries[ZPICO_MAX_QUERYABLES][ZPICO_MAX_PENDING_REPLIES];
    bool stored_query_valid[ZPICO_MAX_QUERYABLES][ZPICO_MAX_PENDING_REPLIES];
    int64_t last_reply_seq[ZPICO_MAX_QUERYABLES];

    // Non-blocking z_get slot pool.
    pending_get_slot_t pending_gets[ZPICO_MAX_PENDING_GETS];

    // Reply waker (async service client).
    zpico_waker_fn reply_waker;

#if defined(ZPICO_TX_BATCH) && ZPICO_TX_BATCH == 1 && ZPICO_TX_BATCH_THREAD == 0
    // phase-279 (#145) — rate-limited batch-flush cadence for platforms WITHOUT
    // the dedicated tx-flush thread (ThreadX + single-threaded). Was a pair of
    // function-local statics in zpico_spin_once.
    z_clock_t last_batch_flush;
    bool batch_flush_init;
#endif

    // Spin-wake primitive for multi-threaded spin_once(). Signaled by our
    // callbacks (sample_handler, query_handler, get reply handlers) so
    // spin_once() wakes immediately when application data arrives, rather than
    // sleeping for the full timeout duration.
#if Z_FEATURE_MULTI_THREAD == 1 && !defined(ZPICO_SMOLTCP)
#if defined(ZENOH_FREERTOS_LWIP)
    // FreeRTOS: binary semaphore — lightweight, no mutex needed.
    SemaphoreHandle_t spin_sem;
#elif defined(ZENOH_NUTTX)
    // NuttX: POSIX `sem_t` + `sem_timedwait`. The pthread mutex + condvar pair
    // hangs indefinitely inside the kernel's watchdog-backed semaphore wait
    // (Phase 55.12 follow-up); POSIX `sem_timedwait` does not share that path.
    sem_t spin_sem_posix;
    bool spin_sem_initialized;
#else
    // POSIX/Zephyr: mutex + condvar
    _z_mutex_t spin_mutex;
#if defined(ZENOH_THREADX) && Z_FEATURE_MULTI_THREAD == 1
    // issue #247 — multi-tier serialization of the spin-driven read path (see
    // _zpico_threadx_locked_read). TRY-lock: the losing spinner skips this
    // round's read (data is drained by the winner).
    _z_mutex_t threadx_read_mutex;
    bool threadx_read_mutex_initialized;
#endif
    _z_condvar_t spin_cv;
    bool spin_cv_initialized;
#endif
#endif

#if ZPICO_TX_BATCH_THREAD == 1
    // phase-279/282 W4 (#145) — dedicated tx-flush task + its optional
    // attributes (copied from the process-wide defaults at zpico_open).
    _z_task_t tx_flush_task;
    volatile bool tx_flush_run;
    bool flush_task_configured;
    z_task_attr_t flush_task_attr;
#if defined(ZENOH_ZEPHYR)
    /* issue 0852 — the struct the Zephyr port actually reads. `z_task_attr_t`
     * is `pthread_attr_t` here, and `_z_task_init` forwards `task_attributes`
     * to `nros_platform_task_init`, which reads this shape. */
    nros_platform_task_attr_t flush_nros_attr;
#endif
#endif

#if Z_FEATURE_MULTI_THREAD == 1
    // Read/lease task attributes (copied from the process-wide defaults at
    // zpico_open). When non-configured, NULL is passed (platform default).
    bool read_task_configured;
    bool lease_task_configured;
    zp_task_read_options_t read_task_opts;
    zp_task_lease_options_t lease_task_opts;
    z_task_attr_t read_task_attr;
    z_task_attr_t lease_task_attr;
    /* issue 0803 — the per-session copy the ALIAS task path reads. See the note
     * on `g_default_read_nros_attr`: `task_attributes` must point at an
     * `nros_platform_task_attr_t`, and `zpico_open` re-points it per session,
     * so the session needs storage of that type too. Fixing only the global
     * left this re-point handing the platform layer the pthread copy again. */
    nros_platform_task_attr_t read_nros_attr;
    nros_platform_task_attr_t lease_nros_attr;
#endif
};

static struct zpico_session g_sessions[ZPICO_MAX_SESSIONS];
static bool g_session_inuse[ZPICO_MAX_SESSIONS];

// Acquire a free pool slot (zeroed). NULL when the pool is exhausted.
zpico_session_t* zpico_session_acquire(void) {
    for (int i = 0; i < ZPICO_MAX_SESSIONS; i++) {
        if (!g_session_inuse[i]) {
            g_session_inuse[i] = true;
            memset(&g_sessions[i], 0, sizeof(g_sessions[i]));
            return (zpico_session_t*)&g_sessions[i];
        }
    }
    return NULL;
}

// Return a slot to the pool. The pointer is invalid afterwards.
void zpico_session_release(zpico_session_t* session) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (s == NULL || s < g_sessions || s >= g_sessions + ZPICO_MAX_SESSIONS) {
        return;
    }
    g_session_inuse[s - g_sessions] = false;
}

// phase-328/#376 — the session's pool index (0..ZPICO_MAX_SESSIONS), or -1 if
// the handle is not a valid pool slot. The Rust shim uses it to scope its
// process-global service-buffer / reply-waker tables per session.
int32_t zpico_session_index(zpico_session_t* session) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (s == NULL || s < g_sessions || s >= g_sessions + ZPICO_MAX_SESSIONS) {
        return -1;
    }
    return (int32_t)(s - g_sessions);
}

// Callback context packing (issue 0348 / phase-328). zenoh-pico closures carry
// a single `void* ctx`; pack the owning session's pool index and the table slot
// index into it instead of an `int idx` (which no longer identifies a session)
// or a back-pointer into a now-pooled array. At ZPICO_MAX_SESSIONS == 1 the
// session index is 0, so the packed value equals the old `(void*)(intptr_t)idx`.
static inline void* _zpico_pack_ctx(struct zpico_session* s, int slot_idx) {
    uintptr_t session_idx = (uintptr_t)(s - g_sessions);
    return (void*)((session_idx << 16) | (uintptr_t)(unsigned)slot_idx);
}
static inline struct zpico_session* _zpico_unpack_session(void* ctx) {
    return &g_sessions[(uintptr_t)ctx >> 16];
}
static inline int _zpico_unpack_slot(void* ctx) {
    return (int)((uintptr_t)ctx & 0xFFFF);
}

// Spin-wake helpers for multi-threaded spin_once(). Signal the OWNING
// session's wake object (the primitives now live in `struct zpico_session`) so
// spin_once() can wake immediately when application data arrives, rather than
// sleeping for the full timeout duration. Callers have `s` in hand.
#if Z_FEATURE_MULTI_THREAD == 1 && !defined(ZPICO_SMOLTCP)

#if defined(ZENOH_FREERTOS_LWIP)
// FreeRTOS: binary semaphore — lightweight, no mutex needed.
static inline void _zpico_notify_spin(struct zpico_session* s) {
    if (s->spin_sem != NULL) {
        xSemaphoreGive(s->spin_sem);
    }
}
#elif defined(ZENOH_NUTTX)
// NuttX: POSIX `sem_t` + `sem_timedwait`. We can't use the pthread
// mutex + condvar pair here — on NuttX the pthread-timed-wait path
// hangs indefinitely inside the kernel's watchdog-backed semaphore
// wait (Phase 55.12 follow-up). POSIX `sem_timedwait` does not share
// that code path and is safe.
static inline void _zpico_notify_spin(struct zpico_session* s) {
    if (s->spin_sem_initialized) {
        // sem_post is async-signal-safe; binary-ish semantics are fine
        // because we only care that spin_once wakes at least once per
        // event, not that every post is counted.
        sem_post(&s->spin_sem_posix);
    }
}
#else
// POSIX/Zephyr: mutex + condvar

#if defined(ZENOH_THREADX) && Z_FEATURE_MULTI_THREAD == 1
/* issue #247 — multi-tier serialization of the spin-driven read path. The
 * ThreadX arm predates multi-tier (RFC-0015 Model 1): TWO tier threads each
 * call zp_read on the SAME TCP stream, splitting message reassembly across
 * two callers — control replies (the write-filter's DeclareSubscriber /
 * DeclareFinal) get consumed-but-dropped, so a spawned tier's publisher
 * filter never opens and its puts are silently discarded (router traces
 * confirmed the reply was sent; the guest never processed it). TRY-lock:
 * the losing spinner skips this round's read (data is drained by the
 * winner) instead of blocking its 1 ms tier cadence. */
static int _zpico_threadx_locked_read(struct zpico_session* s) {
    if (s->threadx_read_mutex_initialized) {
        if (_z_mutex_try_lock(&s->threadx_read_mutex) != 0) {
            /* Another tier is mid-read — it drains the data. This is a
             * benign "no work this round" outcome, NOT a transport failure:
             * return 0 (Ok) so the executor's `consecutive_io_failures`
             * counter is not incremented. Returning a negative here made
             * the nros-c `spin_period` bail after SPIN_ERROR_TOLERANCE idle
             * rounds (issue 0387 / the issue-0355 "idle never counts" rule). */
            return 0;
        }
        int r = zp_read(z_session_loan_mut(&s->session), NULL);
        _z_mutex_unlock(&s->threadx_read_mutex);
        return r;
    }
    return zp_read(z_session_loan_mut(&s->session), NULL);
}
#endif

static inline void _zpico_notify_spin(struct zpico_session* s) {
    if (s->spin_cv_initialized) {
        _z_mutex_lock(&s->spin_mutex);
        _z_condvar_signal(&s->spin_cv);
        _z_mutex_unlock(&s->spin_mutex);
    }
}
#endif // ZENOH_FREERTOS_LWIP / ZENOH_NUTTX

#else
static inline void _zpico_notify_spin(struct zpico_session* s) {
    (void)s;
}
#endif

// ============================================================================
// phase-279 (#145) — dedicated tx-flush thread. Flushing from the executor/
// tier threads measured WORSE-or-equal vs no batching: the flush blocks its
// caller on the socket for up to a recv window, stalling the very timers that
// generate the puts. On multi-threaded platforms a dedicated low-duty thread
// absorbs those waits instead; tier threads only ever append to the batch.
// ThreadX deliberately runs no background tasks (read-task starvation — see
// zpico_open), so it keeps the spin-driven flush. Single-threaded platforms
// have no tasks at all.
//
// The ZPICO_TX_BATCH_THREAD / ZPICO_TX_BATCH_FLUSH_MS selectors are defined
// above, ahead of `struct zpico_session`, which gates its flush members on them.
#if ZPICO_TX_BATCH_THREAD == 1
/* phase-282 W4 (#145) — the flush task runs off the OWNING session (passed as
 * the task `arg`) and flushes that session's transport. */
static void* _zpico_tx_flush_task_fn(void* arg) {
    struct zpico_session* s = (struct zpico_session*)arg;
    while (s->tx_flush_run) {
        zp_batch_flush(z_session_loan(&s->session));
        z_sleep_ms(ZPICO_TX_BATCH_FLUSH_MS);
    }
    return NULL;
}
#endif

// Task Configuration — PROCESS-WIDE DEFAULTS (set before zpico_open, copied
// into the session at open). Boards call these before any session exists, so
// they cannot take a `zpico_session_t*` (documented exception to the handle
// rule — issue 0348 / phase-328).
// ============================================================================

#if Z_FEATURE_MULTI_THREAD == 1
// Optional default task attributes for read/lease tasks. When configured they
// are copied into the session at zpico_open and passed to zp_start_read_task()
// / zp_start_lease_task() instead of NULL (platform default).
static bool g_default_read_task_configured = false;
static bool g_default_lease_task_configured = false;
static zp_task_read_options_t g_default_read_task_opts;
static zp_task_lease_options_t g_default_lease_task_opts;
/* issue 0803 — the attribute the ALIAS task path reads.
 *
 * On a build whose `_z_task_init` comes from `platform_aliases.c` (everything
 * but ThreadX), that alias forwards `task_attributes` straight to
 * `nros_platform_task_init`, which reads it as `nros_platform_task_attr_t *`.
 * `ZENOH_LINUX` makes zenoh-pico's `unix.h` typedef `z_task_attr_t` to
 * `pthread_attr_t`, so the POSIX arm below used to fill a `pthread_attr_t` and
 * hand the platform layer 56 bytes of a different struct — `stack_bytes`,
 * `priority` and `flags` read out of pthread's opaque bytes. The requested 90
 * became whatever those bytes said, clamped into [1,99]: measured 1, for every
 * requested value, on every arm.
 *
 * `nros_zenoh_generic_platform.h` predicted exactly this — "correct only while
 * the value is always NULL, which it was" — and issue 0765 made it non-NULL.
 * Two headers disagreeing about one type is issue 0135's shape.
 *
 * So the POSIX arm fills THIS, and `task_attributes` points at it. The pointer
 * width is unchanged, so `zp_task_read_options_t`'s layout is untouched; what
 * changes is that the pointee is the struct the reader expects. */
static nros_platform_task_attr_t g_default_read_nros_attr;
static nros_platform_task_attr_t g_default_lease_nros_attr;

static z_task_attr_t g_default_read_task_attr;
static z_task_attr_t g_default_lease_task_attr;
#endif

#if ZPICO_TX_BATCH_THREAD == 1
/* phase-282 W4 (#145) — process-wide default attributes for the flush task, set
 * via zpico_set_flush_task_config() before zpico_open(), copied into the session
 * at open. Same platform matrix as the read/lease task attrs: FreeRTOS +
 * POSIX-like honour them, others ignore the attr. */
static bool g_default_flush_task_configured = false;
static z_task_attr_t g_default_flush_task_attr;
#if defined(ZENOH_ZEPHYR)
static nros_platform_task_attr_t g_default_flush_nros_attr;
#endif
#endif

// ============================================================================
// Internal Helper Functions
// ============================================================================

static uint64_t zpico_splitmix64(uint64_t* state) {
    uint64_t z = (*state += UINT64_C(0x9e3779b97f4a7c15));
    z = (z ^ (z >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    z = (z ^ (z >> 27)) * UINT64_C(0x94d049bb133111eb);
    return z ^ (z >> 31);
}

#if defined(ZENOH_FREERTOS_LWIP) || defined(ZENOH_THREADX)
static void zpico_mix_clock_bytes(uint64_t* seed, const void* data, size_t len) {
    const uint8_t* bytes = (const uint8_t*)data;
    for (size_t i = 0; i < len; i++) {
        *seed ^= (uint64_t)bytes[i] << ((i % sizeof(uint64_t)) * 8);
        *seed = zpico_splitmix64(seed);
    }
}
#endif

static uint32_t zpico_next_session_zid_counter(void) {
#if defined(ZPICO_SMOLTCP) || defined(ZPICO_SERIAL)
    return g_session_zid_counter++;
#else
    return atomic_fetch_add(&g_session_zid_counter, 1);
#endif
}

static void zpico_fill_session_zid(struct zpico_session* s, uint8_t bytes[ZPICO_ZID_SIZE]) {
#if defined(__linux__)
    if (getrandom(bytes, ZPICO_ZID_SIZE, 0) == ZPICO_ZID_SIZE) {
        return;
    }
#endif

#if defined(ZPICO_SMOLTCP) || defined(ZPICO_SERIAL)
    // Bare-metal/QEMU targets seed the platform RNG from board-specific
    // entropy before zpico_open(). Use that RNG directly; clock/address-only
    // fallback data is deterministic across separate QEMU processes and can
    // produce duplicate session ZIDs.
    z_random_fill(bytes, ZPICO_ZID_SIZE);
    return;
#endif

#if defined(ZENOH_ZEPHYR)
    sys_rand_get(bytes, ZPICO_ZID_SIZE);

    uint64_t zephyr_seed = k_cycle_get_64();
    zephyr_seed ^= (uint64_t)(uintptr_t)&s->session;
    zephyr_seed ^= (uint64_t)zpico_next_session_zid_counter();
#if defined(CONFIG_POSIX_MULTI_PROCESS)
    zephyr_seed ^= (uint64_t)getpid() << 32;
#endif
    for (size_t i = 0; i < ZPICO_ZID_SIZE; i += sizeof(uint64_t)) {
        uint64_t word = zpico_splitmix64(&zephyr_seed);
        for (size_t j = 0; j < sizeof(uint64_t) && i + j < ZPICO_ZID_SIZE; j++) {
            bytes[i + j] ^= (uint8_t)(word >> (j * 8));
        }
    }
    return;
#endif

    uint64_t seed = (uint64_t)(uintptr_t)&s->session;
    seed ^= (uint64_t)zpico_next_session_zid_counter();
#if defined(__linux__)
    seed ^= (uint64_t)getpid() << 32;
#endif
#if defined(ZPICO_SMOLTCP)
    seed ^= smoltcp_clock_now_ms() << 1;
    seed ^= z_random_u64();
    seed ^= z_random_u64() << 1;
#elif defined(ZPICO_SERIAL)
    seed ^= z_random_u64();
    seed ^= z_random_u64() << 1;
#elif defined(ZENOH_FREERTOS_LWIP) || defined(ZENOH_THREADX)
    z_clock_t now = z_clock_now();
    zpico_mix_clock_bytes(&seed, &now, sizeof(now));
    seed ^= z_random_u64();
    seed ^= z_random_u64() << 1;
#elif defined(CLOCK_REALTIME) && !defined(ZPICO_SERIAL)
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) == 0) {
        seed ^= (uint64_t)ts.tv_sec;
        seed ^= (uint64_t)ts.tv_nsec << 1;
    }
#endif

    for (size_t i = 0; i < ZPICO_ZID_SIZE; i += sizeof(uint64_t)) {
        uint64_t word = zpico_splitmix64(&seed);
        memcpy(&bytes[i], &word, sizeof(word));
    }
}

static void zpico_format_session_zid(char out[37], const uint8_t bytes[ZPICO_ZID_SIZE]) {
    static const char hex[] = "0123456789abcdef";
    size_t j = 0;
    for (size_t i = 0; i < ZPICO_ZID_SIZE; i++) {
        if (j == 8 || j == 13 || j == 18 || j == 21) {
            out[j++] = '-';
        }
        out[j++] = hex[bytes[i] >> 4];
        out[j++] = hex[bytes[i] & 0x0f];
    }
    out[j] = '\0';
}

/**
 * Internal callback for queryable that receives queries
 */
static void _zpico_release_reply_slot(struct zpico_session* s, int32_t handle, int64_t seq);

static void query_handler(z_loaned_query_t* query, void* arg) {
    struct zpico_session* s = _zpico_unpack_session(arg);
    int idx = _zpico_unpack_slot(arg);
    if (idx < 0 || idx >= ZPICO_MAX_QUERYABLES) {
        return;
    }

    queryable_entry_t* entry = &s->queryables[idx];
    if (!entry->active || entry->callback == NULL) {
        return;
    }

    // Get keyexpr
    const z_loaned_keyexpr_t* keyexpr = z_query_keyexpr(query);
    z_view_string_t keyexpr_view;
    z_keyexpr_as_view_string(keyexpr, &keyexpr_view);
    const char* keyexpr_str = z_string_data(z_view_string_loan(&keyexpr_view));
    size_t keyexpr_len = z_string_len(z_view_string_loan(&keyexpr_view));

    // Get payload
    const z_loaned_bytes_t* payload_bytes = z_query_payload(query);
    const uint8_t* payload_data = NULL;
    size_t payload_len = 0;

    z_owned_slice_t payload_slice;
    if (payload_bytes != NULL && z_bytes_len(payload_bytes) > 0) {
        if (z_bytes_to_slice(payload_bytes, &payload_slice) == 0) {
            payload_data = z_slice_data(z_slice_loan(&payload_slice));
            payload_len = z_slice_len(z_slice_loan(&payload_slice));
        }
    }

    // Phase 237 — allocate a free reply slot and clone the query into it so the
    // reply can be sent after this callback returns (deferred get_result) even
    // if more queries land meanwhile. Record the slot index as the reply seq for
    // `zpico_queryable_take_reply_seq` to hand to the (synchronous) callback.
    int64_t reply_seq = -1;
    for (int j = 0; j < ZPICO_MAX_PENDING_REPLIES; ++j) {
        if (!s->stored_query_valid[idx][j]) {
            if (z_query_clone(&s->stored_queries[idx][j], query) == 0) {
                s->stored_query_valid[idx][j] = true;
                reply_seq = j;
            }
            break;
        }
    }
    s->last_reply_seq[idx] = reply_seq;

    // Call user callback
    entry->callback(keyexpr_str, keyexpr_len, payload_data, payload_len, entry->ctx);

    /* issue 0902 — RECLAIM a slot the callback declined.
     *
     * The clone above happens BEFORE the callback runs, and the slot is
     * released in only three places: the `memset` at session open, queryable
     * teardown, and a fully successful `zpico_query_reply`. So a query that is
     * cloned and then dropped by the callback holds its slot for the life of
     * the queryable.
     *
     * The callback drops queries on two ordinary paths, and BOTH are hot:
     * an empty payload — which the Rust side documents as "liveliness probes
     * that zenoh-pico delivers through the same queryable callback as real
     * service requests" — and a full request ring. Neither is an error, and
     * both arrive on an idle link, so the four slots drain with ELAPSED TIME
     * rather than with traffic. Once they are gone `reply_seq` is -1 for
     * every later query, `zpico_query_reply` returns ZPICO_ERR_INVALID, and
     * the caller discards it: a goal is accepted, executed, and never
     * answered. The state is absorbing — nothing re-arms.
     *
     * `zpico_queryable_take_reply_seq` now CLEARS the seq as its name always
     * claimed, so a still-set value here means the callback never asked for
     * it, which means nobody holds this query and nobody will reply to it.
     * Drop it. A callback that DID take the seq keeps its slot for the
     * deferred reply, which is what the mechanism is for. */
    if (reply_seq >= 0 && s->last_reply_seq[idx] >= 0) {
        _zpico_release_reply_slot(s, idx, reply_seq);
        s->last_reply_seq[idx] = -1;
    }

    // Clean up slice
    if (payload_data != NULL) {
        z_slice_drop(z_slice_move(&payload_slice));
    }
    _zpico_notify_spin(s);
}

/**
 * Internal callback that receives zenoh samples and forwards to user callback.
 *
 * Supports three modes:
 * - direct_write: reads payload directly into Rust buffer via z_bytes_reader_read()
 * - with_attachment: copies payload via z_bytes_to_slice() (legacy path)
 * - legacy: copies payload only via z_bytes_to_slice()
 */
static void sample_handler(z_loaned_sample_t* sample, void* arg) {
    struct zpico_session* s = _zpico_unpack_session(arg);
    int idx = _zpico_unpack_slot(arg);
    if (idx < 0 || idx >= ZPICO_MAX_SUBSCRIBERS) {
        return;
    }

    subscriber_entry_t* entry = &s->subscribers[idx];
    if (!entry->active) {
        return;
    }

    // Get payload
    const z_loaned_bytes_t* payload = z_sample_payload(sample);
    size_t payload_len = z_bytes_len(payload);

#if defined(Z_FEATURE_UNSTABLE_API)
    if (entry->zero_copy) {
        if (entry->zero_copy_cb == NULL) {
            return;
        }
        // Get contiguous view — borrows directly from zenoh-pico's receive buffer
        z_view_slice_t view;
        if (z_bytes_get_contiguous_view(payload, &view) == 0) {
            const uint8_t* data = z_slice_data(z_view_slice_loan(&view));
            size_t len = z_slice_len(z_view_slice_loan(&view));

            // Get attachment (small copy, 33-37 bytes)
            const z_loaned_bytes_t* att = z_sample_attachment(sample);
            if (att != NULL) {
                z_owned_slice_t att_slice;
                if (z_bytes_to_slice(att, &att_slice) == 0) {
                    entry->zero_copy_cb(data, len, z_slice_data(z_slice_loan(&att_slice)),
                                        z_slice_len(z_slice_loan(&att_slice)), entry->ctx);
                    z_slice_drop(z_slice_move(&att_slice));
                } else {
                    entry->zero_copy_cb(data, len, NULL, 0, entry->ctx);
                }
            } else {
                entry->zero_copy_cb(data, len, NULL, 0, entry->ctx);
            }
        }
        _zpico_notify_spin(s);
        return;
    }
#endif

    if (entry->ring_mode) {
        // Phase 124.D.3.c — SPSC ring producer path. C is the sole
        // writer of `tail`; Rust the sole writer of `head`.
        if (entry->notify == NULL || entry->ring == NULL) {
            return;
        }
        // Drop empty-payload samples — zenoh-pico delivers background
        // probes / liveliness syncs through the regular subscription
        // path with a zero-length payload. Buffering them would let
        // the typed `take()` consume a slot whose CDR header check
        // then fails. Mirrors the legacy single-slot behaviour.
        if (payload_len == 0) {
            return;
        }
        zpico_ring_desc_t* r = entry->ring;

        // Acquire-load head (published by the Rust consumer) and a
        // relaxed-load of our own tail. Ring full when the gap is
        // slot_count — drop the newest message (matches DDS
        // KEEP_LAST overwrite-from-the-front intent loosely; a
        // dropped burst tail is reported via msg-lost accounting on
        // the Rust side from the sequence gap).
        uintptr_t head =
            atomic_load_explicit((const _Atomic uintptr_t*)r->head, memory_order_acquire);
        uintptr_t tail = atomic_load_explicit((_Atomic uintptr_t*)r->tail, memory_order_relaxed);
        if (tail - head >= r->slot_count) {
            // Ring full — drop. Still fire notify(len) so the Rust
            // side can observe the arrival for waker / lost-count.
            entry->notify(payload_len, NULL, 0, entry->ctx);
            _zpico_notify_spin(s);
            return;
        }

        uintptr_t slot = tail % r->slot_count;
        uint8_t* pay_dst = r->payload_base + slot * r->payload_stride;
        if (payload_len > r->payload_stride) {
            // Slot too small — report overflow via notify, don't
            // advance tail.
            entry->notify(payload_len, NULL, 0, entry->ctx);
            return;
        }
        z_bytes_reader_t reader = z_bytes_get_reader(payload);
        z_bytes_reader_read(&reader, pay_dst, payload_len);
        r->payload_len[slot] = payload_len;

        // Attachment into the parallel per-slot array.
        size_t att_written = 0;
        const z_loaned_bytes_t* attachment = z_sample_attachment(sample);
        if (attachment != NULL && r->att_stride > 0) {
            size_t att_len = z_bytes_len(attachment);
            if (att_len <= r->att_stride) {
                z_bytes_reader_t att_reader = z_bytes_get_reader(attachment);
                att_written =
                    z_bytes_reader_read(&att_reader, r->att_base + slot * r->att_stride, att_len);
            }
        }
        r->att_len[slot] = att_written;

        // Publish the slot: Release store advances tail so the Rust
        // consumer sees the payload + len writes above.
        atomic_store_explicit((_Atomic uintptr_t*)r->tail, tail + 1, memory_order_release);

        // Fire notify for the async waker. Pass NULL attachment —
        // the consumer reads the per-slot attachment array directly.
        entry->notify(payload_len, NULL, 0, entry->ctx);
        _zpico_notify_spin(s);
        return;
    }

    if (entry->direct_write) {
        // Direct-write mode: read payload directly into Rust static buffer
        if (entry->notify == NULL) {
            return;
        }

        // Check lock (Rust reader is processing)
        if (__atomic_load_n(entry->locked_ptr, __ATOMIC_ACQUIRE)) {
            return;
        }

        if (payload_len > entry->buf_capacity) {
            // Overflow: notify with len so Rust can set overflow flag
            entry->notify(payload_len, NULL, 0, entry->ctx);
            return;
        }

        // Read directly into Rust's static buffer — no malloc
        z_bytes_reader_t reader = z_bytes_get_reader(payload);
        z_bytes_reader_read(&reader, entry->buf_ptr, payload_len);

        // Attachment still uses z_bytes_to_slice (33-37 bytes, negligible)
        const z_loaned_bytes_t* attachment = z_sample_attachment(sample);
        if (attachment != NULL) {
            z_owned_slice_t attachment_slice;
            if (z_bytes_to_slice(attachment, &attachment_slice) == 0) {
                const uint8_t* att_data = z_slice_data(z_slice_loan(&attachment_slice));
                size_t att_len = z_slice_len(z_slice_loan(&attachment_slice));
                entry->notify(payload_len, att_data, att_len, entry->ctx);
                z_slice_drop(z_slice_move(&attachment_slice));
            } else {
                entry->notify(payload_len, NULL, 0, entry->ctx);
            }
        } else {
            entry->notify(payload_len, NULL, 0, entry->ctx);
        }
        _zpico_notify_spin(s);
        return;
    }

    // Legacy path: copy payload to owned slice (malloc + memcpy)
    z_owned_slice_t payload_slice;
    if (z_bytes_to_slice(payload, &payload_slice) != 0) {
        return; // Failed to get payload
    }

    const uint8_t* data = z_slice_data(z_slice_loan(&payload_slice));
    size_t len = z_slice_len(z_slice_loan(&payload_slice));

    if (entry->with_attachment) {
        // Extended callback with attachment
        if (entry->callback_ext == NULL) {
            z_slice_drop(z_slice_move(&payload_slice));
            return;
        }

        // Get attachment
        const z_loaned_bytes_t* attachment = z_sample_attachment(sample);
        if (attachment != NULL) {
            z_owned_slice_t attachment_slice;
            if (z_bytes_to_slice(attachment, &attachment_slice) == 0) {
                const uint8_t* att_data = z_slice_data(z_slice_loan(&attachment_slice));
                size_t att_len = z_slice_len(z_slice_loan(&attachment_slice));
                entry->callback_ext(data, len, att_data, att_len, entry->ctx);
                z_slice_drop(z_slice_move(&attachment_slice));
            } else {
                // Attachment exists but failed to convert - call with NULL attachment
                entry->callback_ext(data, len, NULL, 0, entry->ctx);
            }
        } else {
            // No attachment
            entry->callback_ext(data, len, NULL, 0, entry->ctx);
        }
    } else {
        // Legacy callback (payload only)
        if (entry->callback != NULL) {
            entry->callback(data, len, entry->ctx);
        }
    }

    z_slice_drop(z_slice_move(&payload_slice));
    _zpico_notify_spin(s);
}

// ============================================================================
// Session Lifecycle Implementation
// ============================================================================

int32_t zpico_init(zpico_session_t* session, const char* locator) {
    return zpico_init_with_config(session, locator, "client", NULL, 0);
}

int32_t zpico_init_with_config(zpico_session_t* session, const char* locator, const char* mode,
                               const zpico_property_t* properties, size_t num_properties) {
    struct zpico_session* s = (struct zpico_session*)session;
    /* issue 0348 / phase-328 — each handle is an INDEPENDENT session drawn from
       the pool, so issue 0347's "refuse a second init while a session is open"
       guard is gone: a second `zpico_session_acquire()` + init operates on its
       own slot and cannot clobber the first session's registrations. The
       per-handle `initialized` / `session_open` lifecycle is unchanged (a FAILED
       `zpico_open()` leaves `s->session_open == false` and re-inits normally —
       issue #64's esp32-c3 backoff depends on that). */

    // Initialize storage
    memset(s->publishers, 0, sizeof(s->publishers));
    memset(s->subscribers, 0, sizeof(s->subscribers));
    memset(s->liveliness, 0, sizeof(s->liveliness));
    memset(s->queryables, 0, sizeof(s->queryables));
    memset(s->stored_query_valid, 0, sizeof(s->stored_query_valid));
    for (int i = 0; i < ZPICO_MAX_QUERYABLES; i++) {
        s->last_reply_seq[i] = -1;
        for (int j = 0; j < ZPICO_MAX_PENDING_REPLIES; j++) {
            z_internal_query_null(&s->stored_queries[i][j]);
        }
    }
    s->session_open = false;

#ifdef ZPICO_SMOLTCP
    // Initialize smoltcp platform
    int ret = smoltcp_init();
    if (ret < 0) {
        return ZPICO_ERR_GENERIC;
    }
#endif

    // Initialize zenoh config
    z_config_default(&s->config);

    if (zp_config_insert(z_config_loan_mut(&s->config), Z_CONFIG_MODE_KEY, mode) < 0) {
        return ZPICO_ERR_CONFIG;
    }

    bool has_session_zid = false;
    for (size_t i = 0; i < num_properties; i++) {
        if (properties[i].key != NULL && strcmp(properties[i].key, "session_zid") == 0) {
            has_session_zid = true;
            break;
        }
    }
    if (!has_session_zid) {
        uint8_t zid_bytes[ZPICO_ZID_SIZE];
        char zid[37];
        zpico_fill_session_zid(s, zid_bytes);
        zpico_format_session_zid(zid, zid_bytes);
        if (zp_config_insert(z_config_loan_mut(&s->config), Z_CONFIG_SESSION_ZID_KEY, zid) < 0) {
            return ZPICO_ERR_CONFIG;
        }
    }

    /* Apply additional properties.
     *
     * The name -> `Z_CONFIG_*_KEY` map is DERIVED from zenoh-pico's own
     * config.h (`scripts/gen-zpico-config-keys.py` -> `zpico_config_keys.h`),
     * not hand-written here. The hand-written version covered 10 of the 23
     * keys upstream defines; a closed list drifts on every upstream bump and
     * nothing notices, because the old `else` branch dropped an unmapped key
     * without a word.
     *
     * Three refusals, all loud, all `ZPICO_ERR_CONFIG` so the session does not
     * open with a configuration the caller believes it asked for:
     *   - a NULL key or value,
     *   - a name no `Z_CONFIG_*_KEY` matches (the only place a typo can be
     *     caught: zenoh-pico's keys are a bare numbered enum with no schema
     *     validation of its own),
     *   - a TLS key on a build without `Z_FEATURE_LINK_TLS`, which would
     *     otherwise be accepted into the config map and never read.
     *
     * Two of the 23 keys also arrive as dedicated arguments, and the ORDER
     * around this loop decides what that means. `mode` is inserted above, and
     * `zp_config_insert` REPLACES for every key but one — so a `mode` property
     * wins over the `mode` argument. `connect` is inserted below (after the
     * link properties, so TLS endpoint parsing can see the root CA), and it is
     * the one key upstream inserts with `_z_str_intmap_insert_push` — so a
     * `connect` property ADDS an endpoint rather than being overwritten by the
     * locator, which is how a caller states more than one. */
    for (size_t i = 0; i < num_properties; i++) {
        if (properties[i].key == NULL || properties[i].value == NULL) {
            printk("[zpico] session config: property %u has a NULL %s\n", (unsigned)i,
                   properties[i].key == NULL ? "key" : "value");
            return ZPICO_ERR_CONFIG;
        }

        const zpico_config_key_entry* entry = NULL;
        for (size_t k = 0; k < ZPICO_CONFIG_KEY_COUNT; k++) {
            if (strcmp(properties[i].key, ZPICO_CONFIG_KEYS[k].name) == 0) {
                entry = &ZPICO_CONFIG_KEYS[k];
                break;
            }
        }
        if (entry == NULL) {
            printk("[zpico] session config: unknown key '%s' — zenoh-pico defines no such "
                   "run-time option (see zpico_config_keys.h for the %u accepted names)\n",
                   properties[i].key, (unsigned)ZPICO_CONFIG_KEY_COUNT);
            return ZPICO_ERR_CONFIG;
        }
#if Z_FEATURE_LINK_TLS != 1
        if (entry->needs_tls) {
            printk("[zpico] session config: key '%s' needs a zenoh-pico built with "
                   "Z_FEATURE_LINK_TLS; this build has none\n",
                   properties[i].key);
            return ZPICO_ERR_CONFIG;
        }
#endif

        if (zp_config_insert(z_config_loan_mut(&s->config), entry->key, properties[i].value) < 0) {
            printk("[zpico] session config: zenoh-pico refused key '%s'\n", properties[i].key);
            return ZPICO_ERR_CONFIG;
        }
    }

    // Insert connect endpoint after link properties so TLS endpoint parsing can
    // see root CA / verification settings supplied through session config.
    if (locator != NULL) {
        if (zp_config_insert(z_config_loan_mut(&s->config), Z_CONFIG_CONNECT_KEY, locator) < 0) {
            return ZPICO_ERR_CONFIG;
        }
    }

    s->initialized = true;
    return ZPICO_OK;
}

/* Issue 0626 follow-up — `sched_get_priority_{min,max}` come from Zephyr's
 * `lib/posix/options/sched.c`, compiled only under
 * CONFIG_POSIX_PRIORITY_SCHEDULING. native_sim images enable it, cortex-m ones
 * do not, so gating on ZENOH_ZEPHYR alone left every cortex-m link with
 * `undefined reference to sched_get_priority_min` and took the whole zephyr
 * fixture family down with it.
 *
 * Guarded rather than reimplemented: mapping onto whatever the policy REPORTS
 * is the point of the band — the range is a build-configuration property
 * (CONFIG_NUM_PREEMPT_PRIORITIES), not a constant — and an image without the
 * POSIX scheduling option has no range to map onto. Such an image keeps the
 * pre-0626 behaviour (transport tasks at the default priority) and opts in with
 * `CONFIG_POSIX_PRIORITY_SCHEDULING=y`. */
#if defined(ZENOH_ZEPHYR) && defined(CONFIG_POSIX_PRIORITY_SCHEDULING)
/* Issue 0626 — apply a NORMALISED 0-31 priority (0 = least urgent, larger =
 * more urgent) to a pthread attr, in the one place that knows how.
 *
 * The band is phase-364 W5's, chosen so a single authored number does not mean
 * "run me first" on one kernel and "run me last" on another. It is mapped onto
 * the SCHED_RR range for THIS build rather than an assumed one — that range is
 * a Zephyr build-configuration property (`CONFIG_NUM_PREEMPT_PRIORITIES`), not
 * a constant. See the body for why it is computed rather than queried.
 *
 * SCHED_RR rather than SCHED_FIFO: the transport tasks are not the only
 * runnable work at their level, and a FIFO thread that never blocks would keep
 * the CPU indefinitely. Round-robin preserves the priority ORDER, which is what
 * is being asked for, without turning a busy transport into a starvation
 * source. */
/* issue 0736 — NuttX's raw-priority sibling of `zpico_posix_set_priority`.
 *
 * Separate function on purpose. The Zephyr helper takes a NORMALISED 0-31
 * priority and maps it across `CONFIG_NUM_PREEMPT_PRIORITIES`; NuttX tier
 * priorities are authored RAW (`[tiers.*.nuttx] priority = 110`) and go
 * verbatim into `pthread_attr_setschedparam`, which is the vocabulary issue
 * 0623 settled on after a normalised scale silently inverted a tier against
 * the transport band. Two scales through one function is how that happened;
 * this keeps NuttX in the units its callers already write.
 *
 * `sched_get_priority_{min,max}` are safe to call here — the NuttX board's own
 * tier spawn already uses them (`nuttx_clamp_priority`), unlike Zephyr where
 * they sit behind an experimental Kconfig.
 *
 * SCHED_FIFO, not SCHED_RR: the tiers are FIFO, and mixing policies across one
 * priority ORDER only matters among equals. The read task blocks in `recvfrom`
 * with a timeout, so it is not the never-yielding thread the Zephyr helper's
 * round-robin choice guards against.
 *
 * PTHREAD_EXPLICIT_SCHED is load-bearing, for the same reason it is in the
 * Zephyr arm and in the board's tier spawn: the default is
 * PTHREAD_INHERIT_SCHED, under which the policy and param below are IGNORED and
 * the thread silently takes the creator's priority. That silent inheritance is
 * the defect this is fixing — do not reintroduce it one layer down.
 *
 * A raw 0 means "unset": keep whatever the port would have done. */
/* issue 0765 — the guard is not NuttX-only any more.
 *
 * Linux and macOS speak the same raw SCHED_FIFO priorities NuttX does, and the
 * hosted "may not be allowed to request it" concern is about the RESULT, not
 * about the vocabulary: the call either succeeds or returns EPERM, and both are
 * reportable. Writing a second, Linux-flavoured copy of this would be the
 * two-spellings failure issue 0623 already charged the tree for. */

/* issue 0852 — `zpico_posix_set_priority` DELETED, not left beside the new path.
 *
 * It mapped a private 0-31 band onto SCHED_RR and wrote it into a
 * `pthread_attr_t` that no Zephyr code path ever read. Two things made keeping
 * it actively harmful rather than merely dead: a second normalised scale in the
 * same scheduler is issue 0623 verbatim, and
 * `scripts/lib/priority_plan.py:resolve_zephyr_plan` MODELLED THIS FUNCTION —
 * so `check-tier-priority-plan-image` certified a band no image ever had while
 * `realtime-c`'s `system.toml` authored tier values against that fiction.
 * The band now has exactly one spelling: `NROS_PLATFORM_PRIORITY_MAX`, mapped
 * by `nros_zephyr_native_priority`. */
#endif /* ZENOH_ZEPHYR */

/* issue 0765 — OUTSIDE the ZENOH_ZEPHYR block below.
 *
 * This first landed inside `#if defined(ZENOH_ZEPHYR) && ...`, which meant it
 * compiled ONLY on Zephyr while its callers are the NuttX and Linux/macOS arms
 * of `zpico_set_task_config` — the two platforms where it therefore did not
 * exist. A helper defined under a guard narrower than its call sites is a
 * link-time hole with a compile-time smell; keep it beside the function that
 * uses it. */
/* Guarded on the macros THIS BUILD defines (`nros-zpico-build`'s
 * `build.define(...)`), not on compiler/OS macros.
 *
 * An earlier revision of this comment claimed `ZENOH_LINUX` was "not yet
 * defined at this point" because it came from a header further down. That was
 * wrong: these are `-D` flags and are set for the whole TU. The real reason the
 * first attempt failed to compile was that this helper sat INSIDE the
 * `ZENOH_ZEPHYR` block above — a helper guarded more narrowly than its call
 * sites (issue 0775). Recorded because the wrong explanation was plausible and
 * would have sent the next reader to the include order. */
#if defined(ZENOH_NUTTX) || defined(ZENOH_LINUX) || defined(ZENOH_MACOS)
/* issue 0765 — is SCHED_FIFO actually permitted for this process?
 *
 * This must be asked BEFORE the policy goes into a pthread ATTRIBUTE, because
 * an attribute is not best-effort the way a post-create `pthread_setschedparam`
 * is: with `PTHREAD_EXPLICIT_SCHED` set, `pthread_create` itself returns EPERM
 * and the thread is never created. Measured the hard way — setting the
 * transport band on an unprivileged host made the zenoh session fail to open
 * at all (`RMW session open failed — Backend("rmw_ret error")`), turning a
 * scheduling preference into a total outage.
 *
 * Probed by asking on THIS thread and putting it back, once per process. A
 * request that is refused changes nothing; one that succeeds is undone
 * immediately, so the probe cannot leave the caller on a policy it did not
 * choose. */
static int zpico_posix_rt_permitted(void) {
    static int cached = -1;
    if (cached >= 0) {
        return cached;
    }
    int old_policy;
    struct sched_param old_param;
    if (pthread_getschedparam(pthread_self(), &old_policy, &old_param) != 0) {
        cached = 0;
        return cached;
    }
    struct sched_param probe;
    memset(&probe, 0, sizeof(probe));
    probe.sched_priority = sched_get_priority_min(SCHED_FIFO);
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &probe) == 0) {
        (void)pthread_setschedparam(pthread_self(), old_policy, &old_param);
        cached = 1;
    } else {
        cached = 0;
    }
    return cached;
}

static void zpico_posix_fifo_set_priority(pthread_attr_t* attr, uint32_t raw) {
    if (raw == 0u) {
        return;
    }
    if (!zpico_posix_rt_permitted()) {
        /* Leave the attribute alone: the tasks inherit, exactly as before this
         * arm existed. The tier path has already printed the one line naming
         * `setcap cap_sys_nice+ep`, so saying it again per task would be noise
         * about a fact the operator has been told. */
        return;
    }
    int lo = sched_get_priority_min(SCHED_FIFO);
    int hi = sched_get_priority_max(SCHED_FIFO);
    int p = (int)raw;
    if (p < lo) {
        p = lo;
    }
    if (p > hi) {
        p = hi;
    }
    struct sched_param sp;
    sp.sched_priority = p;
    (void)pthread_attr_setschedpolicy(attr, SCHED_FIFO);
    (void)pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED);
    (void)pthread_attr_setschedparam(attr, &sp);
}
#endif

void zpico_set_task_config(uint32_t read_priority, uint32_t read_stack_bytes,
                           uint32_t lease_priority, uint32_t lease_stack_bytes) {
#if Z_FEATURE_MULTI_THREAD == 1
    memset(&g_default_read_task_attr, 0, sizeof(g_default_read_task_attr));
    memset(&g_default_lease_task_attr, 0, sizeof(g_default_lease_task_attr));

    // Platform-specific field assignment.
    // z_task_attr_t varies by platform — only FreeRTOS and POSIX-like
    // platforms have meaningful fields. ThreadX and generic use void*
    // and zenoh-pico ignores the attr entirely on those platforms.
#if defined(ZENOH_FREERTOS) || defined(ZENOH_FREERTOS_LWIP)
    g_default_read_task_attr.name = "zpico_read";
    g_default_read_task_attr.priority = (UBaseType_t)read_priority;
    g_default_read_task_attr.stack_depth = read_stack_bytes / sizeof(StackType_t);
    g_default_lease_task_attr.name = "zpico_lease";
    g_default_lease_task_attr.priority = (UBaseType_t)lease_priority;
    g_default_lease_task_attr.stack_depth = lease_stack_bytes / sizeof(StackType_t);
    g_default_read_task_opts.task_attributes = &g_default_read_task_attr;
    g_default_lease_task_opts.task_attributes = &g_default_lease_task_attr;
    g_default_read_task_configured = true;
    g_default_lease_task_configured = true;
/* issue 0775 — `ZENOH_NUTTX`, not `__NuttX__`.
 *
 * `__NuttX__` is a NuttX HEADER macro and this TU never pulls the header, so
 * this condition has never been true on NuttX. Measured from the build's own
 * flags: the NuttX lane compiles with `-DZENOH_NUTTX -DZENOH_GENERIC` and NO
 * `-DZENOH_LINUX`, so every arm below keyed on that list fell through to the
 * generic one — which discards the attributes these arms exist to set. */
#elif (defined(ZENOH_LINUX) || defined(ZENOH_MACOS) || defined(ZENOH_NUTTX) ||                     \
       defined(__NuttX__) || defined(ZENOH_ZEPHYR)) &&                                             \
    !defined(ZENOH_THREADX)
    // POSIX: stack size via pthread_attr. A ZERO means "leave the port's
    // default alone" (the convention `nros_platform_task_attr_t.stack_bytes`
    // already uses), so a caller that only wants to state a PRIORITY does not
    // have to invent a stack size to get one.
    pthread_attr_init(&g_default_read_task_attr);
    if (read_stack_bytes != 0u) {
        pthread_attr_setstacksize(&g_default_read_task_attr, (size_t)read_stack_bytes);
    }
    pthread_attr_init(&g_default_lease_task_attr);
    if (lease_stack_bytes != 0u) {
        pthread_attr_setstacksize(&g_default_lease_task_attr, (size_t)lease_stack_bytes);
    }
#if defined(ZENOH_ZEPHYR)
    // Issue 0626 — priority IS settable here, and used not to be set.
    //
    // The comment this replaces said "Priority requires SCHED_FIFO (root
    // privileges)". That is true of Linux and false of Zephyr, which has no
    // privilege model: a pthread priority is just a thread priority. The
    // sentence was written for the POSIX hosts that share this branch and then
    // applied to an RTOS, so the transport tasks on every Zephyr image ran at
    // whatever the default happened to be, with no way to state otherwise —
    // which made issue 0506's question (what preempts the RT tiers?)
    // unanswerable on this platform.
    //
    // PTHREAD_EXPLICIT_SCHED is load-bearing: the default is
    // PTHREAD_INHERIT_SCHED, under which the policy and param set below are
    // IGNORED and the new thread silently takes the creator's. A scheduling
    // attribute that is quietly dropped is the failure this issue is about, so
    // it must not be reintroduced one layer down.
    /* issue 0852 — fill the struct the Zephyr port actually READS.
     *
     * This used to call `zpico_posix_set_priority` on `g_default_read_task_attr`,
     * a `pthread_attr_t`. Nothing ever read it. `_z_task_init` on this port
     * (`zephyr/nros_zenoh_zephyr_system.c`) forwards `task_attributes` to
     * `nros_platform_task_init`, which reads an `nros_platform_task_attr_t` —
     * and `zephyr.h` typedefs `z_task_attr_t` to `pthread_attr_t`, so the two
     * are different types and the cast read past the end of a 12-byte object.
     * The bytes there are zero, so the transport was born at band 0: SCHED_RR
     * 0, k_thread 14 — the LEAST urgent preemptible slot, under an executor at
     * 0. Before that "fix" it merely tied with the executor.
     *
     * This is issue 0803 one platform over: that issue fixed exactly this cast
     * for NuttX/Linux/macOS and left Zephyr off the guard list — the one
     * platform whose `z_task_attr_t` genuinely IS a `pthread_attr_t`, so the
     * only one where the bug was guaranteed rather than incidental.
     *
     * BAND, not `NROS_PLATFORM_PRIORITY_RAW`: `nros_zephyr_native_priority`
     * maps `0..NROS_PLATFORM_PRIORITY_MAX` onto whatever `SCHED_RR` reports for
     * THIS image, which is the point of the normalised band. The old 0-31
     * scale is gone rather than left beside this one — two normalised scales
     * meeting in one scheduler is issue 0623 verbatim, and a 16 authored
     * against 0-31 maps to 0 through the 255-wide one, i.e. the bottom slot
     * again. */
    nros_platform_task_attr_init(&g_default_read_nros_attr);
    nros_platform_task_attr_init(&g_default_lease_nros_attr);
    if (read_priority != 0u) {
        g_default_read_nros_attr.priority = (int32_t)read_priority;
    }
    if (lease_priority != 0u) {
        g_default_lease_nros_attr.priority = (int32_t)lease_priority;
    }
    g_default_read_nros_attr.name = "zpico_read";
    g_default_lease_nros_attr.name = "zpico_lease";
    if (read_stack_bytes != 0u) {
        g_default_read_nros_attr.stack_bytes = read_stack_bytes;
    }
    if (lease_stack_bytes != 0u) {
        g_default_lease_nros_attr.stack_bytes = lease_stack_bytes;
    }
#else
    // issue 0765 / 0775 — Linux, macOS AND NuttX place the transport here.
    //
    // NuttX had its own `#elif defined(__NuttX__)` arm for a while. It never
    // compiled: `__NuttX__` is a NuttX HEADER macro and this TU never pulls the
    // header, while `nros-zpico-build` defines `ZENOH_NUTTX` AND `ZENOH_LINUX`
    // for that target — so NuttX has always fallen through to this branch,
    // which used to discard the priority. The arm is merged away rather than
    // re-guarded: all three ports want the identical raw-SCHED_FIFO call, and
    // a separate arm that agrees with this one is just somewhere for them to
    // drift apart (issue 0775).
    //
    // This arm used to discard the priority, on the grounds that RT scheduling
    // "needs a policy this process may not be allowed to request". True, and
    // not a reason to refuse to ASK: `setcap cap_sys_nice+ep` is exactly how
    // play_launch does this without root, and where the capability is absent
    // the attribute is simply ignored at spawn — the same best-effort contract
    // every other port here already has.
    //
    // Refusing to ask had a cost that only appeared once TIER priorities became
    // real (issue 0765): the tiers run SCHED_FIFO while the read and lease
    // tasks stay on SCHED_OTHER, and a SCHED_FIFO thread outranks every
    // SCHED_OTHER thread unconditionally. So the app preempted the link it
    // publishes over, with no way for an operator to state otherwise — issue
    // 0623's inversion, on the one platform that could not express the fix.
    /* issue 0803 — fill the struct the alias reader expects, not a
     * `pthread_attr_t`. `NROS_PLATFORM_PRIORITY_RAW` because this is a RAW
     * SCHED_FIFO value from the board's reserved band, not a value on the
     * normalised cross-RTOS band — the same vocabulary issue 0623 settled on
     * after a normalised scale silently inverted a tier against the transport. */
    nros_platform_task_attr_init(&g_default_read_nros_attr);
    nros_platform_task_attr_init(&g_default_lease_nros_attr);
    if (read_priority != 0u) {
        g_default_read_nros_attr.priority = NROS_PLATFORM_PRIORITY_RAW(read_priority);
    }
    if (lease_priority != 0u) {
        g_default_lease_nros_attr.priority = NROS_PLATFORM_PRIORITY_RAW(lease_priority);
    }
    if (read_stack_bytes != 0u) {
        g_default_read_nros_attr.stack_bytes = read_stack_bytes;
    }
    if (lease_stack_bytes != 0u) {
        g_default_lease_nros_attr.stack_bytes = lease_stack_bytes;
    }
#endif
    g_default_read_task_opts.task_attributes = (z_task_attr_t*)&g_default_read_nros_attr;
    g_default_lease_task_opts.task_attributes = (z_task_attr_t*)&g_default_lease_nros_attr;
    g_default_read_task_configured = true;
    g_default_lease_task_configured = true;
#elif defined(ZENOH_THREADX)
    // Issue 0626 — ThreadX carries the platform ABI's attributes now.
    //
    // This used to share the `#else` arm below, whose comment ("z_task_attr_t
    // is void* and zenoh-pico ignores it") was accurate when written and is
    // what made the priority unstatable here: `_z_task_init` in
    // `c/platform/threadx/task.c` discarded the attr and gave every zenoh task
    // the single compile-time `Z_TASK_PRIORITY`.
    //
    // The priority stays on the NORMALISED band; `task.c` inverts it, because
    // ThreadX counts 0 as the HIGHEST priority and the band counts larger as
    // more urgent. Doing the inversion at the spawn site keeps every caller in
    // one vocabulary (phase-364 W5) — that is the whole point of the band.
    //
    // Stacks are not forwarded: the ThreadX `_z_task_t` embeds its stack at the
    // compile-time `Z_TASK_STACK_SIZE`, so there is no larger region to point
    // at, and `task.c` documents the same refusal at the other end.
    nros_platform_task_attr_init(&g_default_read_task_attr);
    g_default_read_task_attr.name = "zpico_read";
    g_default_read_task_attr.priority = (int32_t)read_priority;
    nros_platform_task_attr_init(&g_default_lease_task_attr);
    g_default_lease_task_attr.name = "zpico_lease";
    g_default_lease_task_attr.priority = (int32_t)lease_priority;
    g_default_read_task_opts.task_attributes = &g_default_read_task_attr;
    g_default_lease_task_opts.task_attributes = &g_default_lease_task_attr;
    g_default_read_task_configured = true;
    g_default_lease_task_configured = true;
    (void)read_stack_bytes;
    (void)lease_stack_bytes;
#else
    // Bare-metal and other single-threaded platforms: `z_task_attr_t` is still
    // `void *` there (see `c/platform/bare-metal/platform.h`) and no task is
    // ever created, so there is nothing to configure.
    (void)read_priority;
    (void)read_stack_bytes;
    (void)lease_priority;
    (void)lease_stack_bytes;
#endif
#else
    (void)read_priority;
    (void)read_stack_bytes;
    (void)lease_priority;
    (void)lease_stack_bytes;
#endif
}

void zpico_set_flush_task_config(uint32_t priority, uint32_t stack_bytes) {
#if ZPICO_TX_BATCH_THREAD == 1
    memset(&g_default_flush_task_attr, 0, sizeof(g_default_flush_task_attr));
#if defined(ZENOH_FREERTOS) || defined(ZENOH_FREERTOS_LWIP)
    g_default_flush_task_attr.name = "zpico_flush";
    g_default_flush_task_attr.priority = (UBaseType_t)priority;
    g_default_flush_task_attr.stack_depth = stack_bytes / sizeof(StackType_t);
    g_default_flush_task_configured = true;
/* issue 0775 — `ZENOH_NUTTX`, not `__NuttX__`.
 *
 * `__NuttX__` is a NuttX HEADER macro and this TU never pulls the header, so
 * this condition has never been true on NuttX. Measured from the build's own
 * flags: the NuttX lane compiles with `-DZENOH_NUTTX -DZENOH_GENERIC` and NO
 * `-DZENOH_LINUX`, so every arm below keyed on that list fell through to the
 * generic one — which discards the attributes these arms exist to set. */
#elif (defined(ZENOH_LINUX) || defined(ZENOH_MACOS) || defined(ZENOH_NUTTX) ||                     \
       defined(__NuttX__) || defined(ZENOH_ZEPHYR)) &&                                             \
    !defined(ZENOH_THREADX)
#if defined(ZENOH_ZEPHYR)
    /* issue 0852 — the flush task is the THIRD transport task and it had the
     * same defect as read and lease: it filled a `pthread_attr_t` that the
     * Zephyr port never reads, and discarded the priority outright ("needs
     * SCHED_FIFO (root)" — true of Linux, meaningless on an RTOS with no
     * privilege model). Fixed in the same commit as its two siblings, because
     * fixing one transport task and leaving the third is how this class keeps
     * coming back. Band, not RAW — see the read/lease arm. */
    nros_platform_task_attr_init(&g_default_flush_nros_attr);
    if (priority != 0u) {
        g_default_flush_nros_attr.priority = (int32_t)priority;
    }
    g_default_flush_nros_attr.name = "zpico_flush";
    if (stack_bytes != 0u) {
        g_default_flush_nros_attr.stack_bytes = stack_bytes;
    }
    g_default_flush_task_configured = true;
#else
    /* POSIX: stack size via pthread_attr; priority needs SCHED_FIFO (root),
     * so only the stack size is applied — mirrors zpico_set_task_config.
     *
     * NOTE (issue 0852): on a LINUX/NuttX build `z_task_attr_t` is
     * `nros_platform_task_attr_t` (they reach `zenoh_generic_platform.h`), so
     * `pthread_attr_init` here would be initialising the wrong struct. It does
     * not bite today only because those targets default `ZPICO_TX_BATCH` off,
     * which compiles this whole arm out. Enable batching there and this needs
     * the same treatment as the Zephyr arm above. */
    pthread_attr_init(&g_default_flush_task_attr);
    pthread_attr_setstacksize(&g_default_flush_task_attr, (size_t)stack_bytes);
    g_default_flush_task_configured = true;
    (void)priority;
#endif
#else
    (void)priority;
    (void)stack_bytes;
#endif
#else
    /* No flush thread in this build (batching off / single-threaded /
     * ThreadX): nothing to configure. */
    (void)priority;
    (void)stack_bytes;
#endif
}

int32_t zpico_open(zpico_session_t* session) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (!s->initialized) {
        return ZPICO_ERR_GENERIC;
    }

#if Z_FEATURE_MULTI_THREAD == 1
    /* issue 0348 / phase-328 — copy the process-wide task-spawn DEFAULTS (set by
     * the board via zpico_set_task_config before any session existed) into this
     * session, and re-point the opts at the session's OWN attr copy. */
#if defined(ZENOH_ZEPHYR) || defined(ZENOH_THREADX)
    /* Issue 0626 — neither Zephyr nor ThreadX has a board-side caller for
     * `zpico_set_task_config` (the FreeRTOS board is its only one in the tree),
     * so before this the transport tasks got NULL attrs and an unstatable
     * priority. Apply the compile-time default here when nothing set one
     * explicitly; a board that DOES call the setter first wins, because that
     * sets `configured`.
     *
     * Stacks are passed as 0 deliberately: this path is stating a priority.
     * Zephyr reads 0 as "leave the port default alone", and ThreadX ignores the
     * field outright (its stack is embedded in `_z_task_t`). */
    if (!g_default_read_task_configured) {
        zpico_set_task_config(ZPICO_READ_TASK_PRIORITY, 0u, ZPICO_LEASE_TASK_PRIORITY, 0u);
    }
#endif
    s->read_task_configured = g_default_read_task_configured;
    s->lease_task_configured = g_default_lease_task_configured;
    s->read_task_attr = g_default_read_task_attr;
    s->lease_task_attr = g_default_lease_task_attr;
    s->read_task_opts = g_default_read_task_opts;
    s->lease_task_opts = g_default_lease_task_opts;
#if defined(ZENOH_NUTTX) || defined(ZENOH_LINUX) || defined(ZENOH_MACOS) ||                        \
    defined(ZENOH_ZEPHYR)
    /* issue 0852 — ZENOH_ZEPHYR joins this list. It was the platform the cast
     * below was guaranteed to break on, and it was the one left out. */
    /* issue 0803 — point at the per-session `nros_platform_task_attr_t`, which
     * is what `platform_aliases.c`'s `_z_task_init` forwards to
     * `nros_platform_task_init`. Pointing at `read_task_attr` handed it a
     * `pthread_attr_t`, whose bytes at the `priority` offset are 0 whatever was
     * requested — decoded as band value 0, the LEAST urgent RT priority, so the
     * transport landed at SCHED_FIFO 1 below every tier and 90/40/5 all
     * produced identical kernel tables. */
    s->read_nros_attr = g_default_read_nros_attr;
    s->lease_nros_attr = g_default_lease_nros_attr;
    if (s->read_task_configured) {
        s->read_task_opts.task_attributes = (z_task_attr_t*)&s->read_nros_attr;
    }
    if (s->lease_task_configured) {
        s->lease_task_opts.task_attributes = (z_task_attr_t*)&s->lease_nros_attr;
    }
#else
    if (s->read_task_configured) {
        s->read_task_opts.task_attributes = &s->read_task_attr;
    }
    if (s->lease_task_configured) {
        s->lease_task_opts.task_attributes = &s->lease_task_attr;
    }
#endif
#endif
#if ZPICO_TX_BATCH_THREAD == 1
    s->flush_task_configured = g_default_flush_task_configured;
    s->flush_task_attr = g_default_flush_task_attr;
#if defined(ZENOH_ZEPHYR)
    s->flush_nros_attr = g_default_flush_nros_attr;
#endif
#endif

    z_open_options_t open_opts;
    z_open_options_default(&open_opts);
#if Z_FEATURE_MULTI_THREAD == 1
    open_opts.auto_start_read_task = false;
    open_opts.auto_start_lease_task = false;
#endif
    int open_ret = z_open(&s->session, z_config_move(&s->config), &open_opts);
    if (open_ret < 0) {
        return ZPICO_ERR_SESSION;
    }

#ifdef ZPICO_SERIAL
    // Switch serial reads from blocking (needed for z_open handshake) to
    // non-blocking so zpico_spin_once doesn't block for 5s on idle iterations.
    extern void zpico_serial_set_nonblocking(void);
    zpico_serial_set_nonblocking();
#endif
    {
        z_id_t zid = z_info_zid(z_session_loan(&s->session));
        (void)zid;
    }

    // Start background tasks only in multi-threaded mode. ThreadX/NSOS is an
    // exception: its blocking BSD recv path can keep the read task runnable
    // long enough to starve the lease task, so zpico_spin_once() drives reads
    // and keepalives explicitly for that platform.
#if Z_FEATURE_MULTI_THREAD == 1 && !defined(ZENOH_THREADX)
#if defined(ZENOH_FREERTOS_LWIP)
    s->spin_sem = xSemaphoreCreateBinary();
#elif defined(ZENOH_NUTTX)
    if (sem_init(&s->spin_sem_posix, 0, 0) == 0) {
        s->spin_sem_initialized = true;
    }
#elif !defined(ZPICO_SMOLTCP)
    _z_mutex_init(&s->spin_mutex);
    _z_condvar_init(&s->spin_cv);
    s->spin_cv_initialized = true;
#endif

    const zp_task_read_options_t* read_opts = s->read_task_configured ? &s->read_task_opts : NULL;
    const zp_task_lease_options_t* lease_opts =
        s->lease_task_configured ? &s->lease_task_opts : NULL;

    if (zp_start_read_task(z_session_loan_mut(&s->session), read_opts) < 0) {
        z_close(z_session_loan_mut(&s->session), NULL);
        return ZPICO_ERR_TASK;
    }

    if (zp_start_lease_task(z_session_loan_mut(&s->session), lease_opts) < 0) {
        zp_stop_read_task(z_session_loan_mut(&s->session));
        z_close(z_session_loan_mut(&s->session), NULL);
        return ZPICO_ERR_TASK;
    }
#endif

#if defined(ZPICO_TX_BATCH) && ZPICO_TX_BATCH == 1
    /* phase-279 (#145) — opt-in tx batching: puts/gets append to the transport
     * write buffer instead of sending; one socket send per flush carries the
     * whole batch. Flush cadence = zpico_spin_once (every executor spin) +
     * zenoh-pico's own implicit flushes (batch-buffer overflow, any transport
     * message — so the lease keepalive bounds batch sit-time even without
     * spins). Express messages (query replies, gets, express publishers)
     * bypass the batch inside zenoh-pico. Compile-time knob, default OFF. */
    zp_batch_start(z_session_loan(&s->session));
#if ZPICO_TX_BATCH_THREAD == 1
    s->tx_flush_run = true;
#if defined(ZENOH_ZEPHYR)
    /* issue 0852 — hand the port the struct it reads, as read and lease do. */
    z_task_attr_t* flush_attr = s->flush_task_configured
                                    ? (z_task_attr_t*)&s->flush_nros_attr
                                    : NULL;
#else
    z_task_attr_t* flush_attr = s->flush_task_configured ? &s->flush_task_attr : NULL;
#endif
    if (_z_task_init(&s->tx_flush_task, flush_attr,
                     _zpico_tx_flush_task_fn, s) != 0) {
        /* No thread → flushes ride only on implicit sends (keepalives bound
         * sit-time to the lease interval). Loud, not fatal. */
        s->tx_flush_run = false;
        printk("zpico: tx-flush task init FAILED — batched puts flush on keepalives only\n");
    }
#endif
#endif

#if defined(ZENOH_THREADX) && Z_FEATURE_MULTI_THREAD == 1
    if (!s->threadx_read_mutex_initialized && _z_mutex_init(&s->threadx_read_mutex) == 0) {
        s->threadx_read_mutex_initialized = true;
    }
#endif

    s->session_open = true;
    return ZPICO_OK;
}

int32_t zpico_is_open(zpico_session_t* session) {
    struct zpico_session* s = (struct zpico_session*)session;
    return s->session_open ? 1 : 0;
}

/**
 * Phase 124.E.3 — streamed publish.
 *
 * Drives zenoh-pico's `z_bytes_writer` API to assemble the payload
 * chunk-by-chunk inside zenoh's `z_owned_bytes_t` (a refcounted
 * bytes object backed by zenoh-pico's allocator). The caller's
 * `size_cb` reports the total length once, then `chunk_cb` is
 * invoked repeatedly with cursor-into-staging buffers of up to
 * 1 KiB each. Each chunk is appended to the bytes object via
 * `z_bytes_writer_write_all`, then `z_publisher_put` ships the
 * assembled payload.
 *
 * The win over user-side `publish_raw(staging_buffer)`: the chunks
 * land directly in zenoh's allocator-managed `z_owned_bytes_t`
 * rather than first into a caller-owned `[u8; N]` stack array.
 * For a 32 KiB message, that's 32 KiB less stack pressure on the
 * publishing task.
 */
int32_t zpico_publish_streamed(zpico_session_t* session, int32_t handle, size_t total_len,
                               void (*chunk_cb)(uint8_t* out_buf, size_t cap, size_t* out_written,
                                                void* user_ctx),
                               void* user_ctx, const uint8_t* attachment, size_t attachment_len) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (handle < 0 || handle >= ZPICO_MAX_PUBLISHERS || !s->publishers[handle].active) {
        return ZPICO_ERR_INVALID;
    }
    if (chunk_cb == NULL) {
        return ZPICO_ERR_INVALID;
    }

    z_owned_bytes_writer_t writer;
    if (z_bytes_writer_empty(&writer) < 0) {
        return ZPICO_ERR_GENERIC;
    }

    /* Chunk buffer is fixed at 1 KiB. Trades publish_streamed
     * memory cost for chunk_cb invocation count. 1 KiB is the
     * smallest aligned size that still gives the caller meaningful
     * batching latitude — at 128 B you'd hit the callback ~250
     * times for a 32 KiB message. */
    uint8_t chunk[1024];
    size_t written_so_far = 0;
    while (written_so_far < total_len) {
        size_t want = total_len - written_so_far;
        if (want > sizeof(chunk)) {
            want = sizeof(chunk);
        }
        size_t actually_written = 0;
        chunk_cb(chunk, want, &actually_written, user_ctx);
        if (actually_written == 0) {
            /* EOF before total_len — abort the writer so we don't
             * publish a truncated payload. */
            z_bytes_writer_drop(z_bytes_writer_move(&writer));
            return ZPICO_ERR_PUBLISH;
        }
        if (actually_written > want) {
            actually_written = want;
        }
        if (z_bytes_writer_write_all(z_bytes_writer_loan_mut(&writer), chunk, actually_written) <
            0) {
            z_bytes_writer_drop(z_bytes_writer_move(&writer));
            return ZPICO_ERR_PUBLISH;
        }
        written_so_far += actually_written;
    }

    z_owned_bytes_t payload;
    z_bytes_writer_finish(z_bytes_writer_move(&writer), &payload);

    z_publisher_put_options_t opts;
    z_publisher_put_options_default(&opts);

    /* ROS interop attachment (sequence number + source timestamp +
     * GID) — same shape `zpico_publish_with_attachment` builds. */
    z_owned_bytes_t attachment_bytes;
    if (attachment != NULL && attachment_len > 0) {
        if (z_bytes_copy_from_buf(&attachment_bytes, attachment, attachment_len) < 0) {
            z_bytes_drop(z_bytes_move(&payload));
            return ZPICO_ERR_PUBLISH;
        }
        opts.attachment = z_bytes_move(&attachment_bytes);
    }

    if (z_publisher_put(z_publisher_loan(&s->publishers[handle].publisher), z_bytes_move(&payload),
                        &opts) < 0) {
        return ZPICO_ERR_PUBLISH;
    }

    return ZPICO_OK;
}

/**
 * Phase 124.F.2 — wire-level "is the agent still reachable?" probe.
 *
 * Issues one `zp_send_keep_alive` against the open session. On
 * zenoh-pico that's the closest match to a true ping primitive:
 * the function returns success when the keep-alive frame fired
 * down the transport layer, and a negative `z_result_t` when the
 * TCP send (or serial / shared-memory equivalent) reports a dead
 * link. Best-effort: a fresh-link silent failure (peer disappeared
 * but the OS hasn't reported the socket as half-closed) will still
 * report OK until the next send-side timeout.
 *
 * Returns ZPICO_OK on success, ZPICO_ERR_SESSION when no session
 * is open, ZPICO_ERR_TIMEOUT when the keep-alive failed (treated
 * as a probe timeout per the 124.F.1 semantics).
 */
int32_t zpico_send_keep_alive(zpico_session_t* session) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (!s->session_open) {
        return ZPICO_ERR_SESSION;
    }
    zp_send_keep_alive_options_t options;
    zp_send_keep_alive_options_default(&options);
    z_result_t ret = zp_send_keep_alive(z_session_loan(&s->session), &options);
    if (ret < 0) {
        return ZPICO_ERR_TIMEOUT;
    }
    return ZPICO_OK;
}

void zpico_close(zpico_session_t* session) {
    struct zpico_session* s = (struct zpico_session*)session;
    // Clean up publishers
    for (int i = 0; i < ZPICO_MAX_PUBLISHERS; i++) {
        if (s->publishers[i].active) {
            z_undeclare_publisher(z_publisher_move(&s->publishers[i].publisher));
            s->publishers[i].active = false;
        }
    }

    // Clean up subscribers
    for (int i = 0; i < ZPICO_MAX_SUBSCRIBERS; i++) {
        if (s->subscribers[i].active) {
            z_undeclare_subscriber(z_subscriber_move(&s->subscribers[i].subscriber));
            s->subscribers[i].active = false;
            s->subscribers[i].callback = NULL;
            s->subscribers[i].ctx = NULL;
        }
    }

    // Clean up liveliness tokens
    for (int i = 0; i < ZPICO_MAX_LIVELINESS; i++) {
        if (s->liveliness[i].active) {
            z_liveliness_undeclare_token(z_liveliness_token_move(&s->liveliness[i].token));
            s->liveliness[i].active = false;
        }
    }

    // Clean up queryables
    for (int i = 0; i < ZPICO_MAX_QUERYABLES; i++) {
        if (s->queryables[i].active) {
            z_undeclare_queryable(z_queryable_move(&s->queryables[i].queryable));
            s->queryables[i].active = false;
            s->queryables[i].callback = NULL;
            s->queryables[i].ctx = NULL;
        }
    }

    // Close session
    if (s->session_open) {
#if defined(ZPICO_TX_BATCH) && ZPICO_TX_BATCH == 1
#if ZPICO_TX_BATCH_THREAD == 1
        if (s->tx_flush_run) {
            s->tx_flush_run = false;
            _z_task_join(&s->tx_flush_task);
        }
#endif
        /* phase-279 (#145) — stop batching; zp_batch_stop flushes the remainder. */
        zp_batch_stop(z_session_loan(&s->session));
#endif
#if Z_FEATURE_MULTI_THREAD == 1
        // Stop background tasks (only in multi-threaded mode)
        zp_stop_read_task(z_session_loan_mut(&s->session));
        zp_stop_lease_task(z_session_loan_mut(&s->session));

#if defined(ZENOH_FREERTOS_LWIP)
        if (s->spin_sem != NULL) {
            vSemaphoreDelete(s->spin_sem);
            s->spin_sem = NULL;
        }
#elif defined(ZENOH_NUTTX)
        if (s->spin_sem_initialized) {
            s->spin_sem_initialized = false;
            sem_destroy(&s->spin_sem_posix);
        }
#elif !defined(ZPICO_SMOLTCP)
        s->spin_cv_initialized = false;
        _z_condvar_drop(&s->spin_cv);
        _z_mutex_drop(&s->spin_mutex);
#endif
#endif
        z_close(z_session_loan_mut(&s->session), NULL);
        s->session_open = false;
    }

#ifdef ZPICO_SMOLTCP
    // Cleanup smoltcp platform
    smoltcp_cleanup();
#endif

    s->initialized = false;
}

// ============================================================================
// Publisher Implementation
// ============================================================================

int32_t zpico_declare_publisher(zpico_session_t* session, const char* keyexpr) {
    return zpico_declare_publisher_ex(session, keyexpr, 0);
}

int32_t zpico_declare_publisher_ex(zpico_session_t* session, const char* keyexpr,
                                   int32_t is_express) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (!s->session_open) {
        return ZPICO_ERR_SESSION;
    }

    // Find free slot
    int idx = -1;
    for (int i = 0; i < ZPICO_MAX_PUBLISHERS; i++) {
        if (!s->publishers[i].active) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        return ZPICO_ERR_FULL;
    }

    z_view_keyexpr_t ke;
    int ke_ret = z_view_keyexpr_from_str(&ke, keyexpr);
    if (ke_ret < 0) {
        return ZPICO_ERR_KEYEXPR;
    }

    z_publisher_options_t pub_opts;
    z_publisher_options_default(&pub_opts);
    /* phase-279 (#145) — express publishers bypass tx batching inside zenoh-pico
     * (sent immediately, wire EXPRESS flag). Surfaced from TopicInfo::tx_express
     * through NrosRmwQos; harmless without batching. */
    pub_opts.is_express = (is_express != 0);
#if defined(ZPICO_TX_BATCH) && ZPICO_TX_BATCH == 1
    /* Batching queues puts behind the transport tx mutex; the default DROP
     * congestion control TRY-locks it and silently discards the put whenever a
     * flush is mid-send (waiting on the socket) — measured WORSE than no
     * batching (4.7 vs 8.6 msg/s). BLOCK = append-or-wait: every put lands in
     * the batch and ships with the next flush. Declare-time option (per-put
     * options carry no congestion control in zenoh-pico). */
    pub_opts.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
#endif
    int pub_ret = z_declare_publisher(z_session_loan(&s->session), &s->publishers[idx].publisher,
                                      z_view_keyexpr_loan(&ke), &pub_opts);
    if (pub_ret < 0) {
        printk("zpico: z_declare_publisher failed: %d for '%s'\n", pub_ret, keyexpr);
        return ZPICO_ERR_GENERIC;
    }

    s->publishers[idx].active = true;
    return idx;
}

int32_t zpico_publish(zpico_session_t* session, int32_t handle, const uint8_t* data, size_t len) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (handle < 0 || handle >= ZPICO_MAX_PUBLISHERS || !s->publishers[handle].active) {
        return ZPICO_ERR_INVALID;
    }

    z_owned_bytes_t payload;
    int bytes_ret = z_bytes_copy_from_buf(&payload, data, len);
    if (bytes_ret < 0) {
        printk("zpico: z_bytes_copy_from_buf failed: %d\n", bytes_ret);
        return ZPICO_ERR_PUBLISH;
    }

    int put_ret = z_publisher_put(z_publisher_loan(&s->publishers[handle].publisher),
                                  z_bytes_move(&payload), NULL);
    if (put_ret < 0) {
        printk("zpico: z_publisher_put failed: %d\n", put_ret);
        return ZPICO_ERR_PUBLISH;
    }

    return ZPICO_OK;
}

int32_t zpico_undeclare_publisher(zpico_session_t* session, int32_t handle) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (handle < 0 || handle >= ZPICO_MAX_PUBLISHERS || !s->publishers[handle].active) {
        return ZPICO_ERR_INVALID;
    }

    z_undeclare_publisher(z_publisher_move(&s->publishers[handle].publisher));
    s->publishers[handle].active = false;
    return ZPICO_OK;
}

// ============================================================================
// Subscriber Implementation
// ============================================================================

int32_t zpico_declare_subscriber(zpico_session_t* session, const char* keyexpr,
                                 ZpicoCallback callback, void* ctx) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (!s->session_open) {
        return ZPICO_ERR_SESSION;
    }

    // Find free slot
    int idx = -1;
    for (int i = 0; i < ZPICO_MAX_SUBSCRIBERS; i++) {
        if (!s->subscribers[i].active) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        return ZPICO_ERR_FULL;
    }

    s->subscribers[idx].callback = callback;
    s->subscribers[idx].ctx = ctx;
    s->subscribers[idx].with_attachment = false; // Legacy mode

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        s->subscribers[idx].callback = NULL;
        s->subscribers[idx].ctx = NULL;
        return ZPICO_ERR_KEYEXPR;
    }

    // Create closure for callback, passing index as context
    z_owned_closure_sample_t closure;
    z_closure_sample(&closure, sample_handler, NULL, _zpico_pack_ctx(s, idx));

    int sub_ret =
        z_declare_subscriber(z_session_loan(&s->session), &s->subscribers[idx].subscriber,
                             z_view_keyexpr_loan(&ke), z_closure_sample_move(&closure), NULL);
    if (sub_ret < 0) {
        printk("zpico: z_declare_subscriber failed: %d for '%s'\n", sub_ret, keyexpr);
        s->subscribers[idx].callback = NULL;
        s->subscribers[idx].ctx = NULL;
        return ZPICO_ERR_GENERIC;
    }

    s->subscribers[idx].active = true;
    return idx;
}

int32_t zpico_declare_subscriber_with_attachment(zpico_session_t* session, const char* keyexpr,
                                                 ZpicoCallbackWithAttachment callback, void* ctx) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (!s->session_open) {
        return ZPICO_ERR_SESSION;
    }

    // Find free slot
    int idx = -1;
    for (int i = 0; i < ZPICO_MAX_SUBSCRIBERS; i++) {
        if (!s->subscribers[i].active) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        return ZPICO_ERR_FULL;
    }

    s->subscribers[idx].callback_ext = callback;
    s->subscribers[idx].ctx = ctx;
    s->subscribers[idx].with_attachment = true; // Extended mode with attachment

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        s->subscribers[idx].callback_ext = NULL;
        s->subscribers[idx].ctx = NULL;
        return ZPICO_ERR_KEYEXPR;
    }

    // Create closure for callback, passing index as context
    z_owned_closure_sample_t closure;
    z_closure_sample(&closure, sample_handler, NULL, _zpico_pack_ctx(s, idx));

    int sub_ret =
        z_declare_subscriber(z_session_loan(&s->session), &s->subscribers[idx].subscriber,
                             z_view_keyexpr_loan(&ke), z_closure_sample_move(&closure), NULL);
    if (sub_ret < 0) {
        printk("zpico: z_declare_subscriber failed: %d for '%s'\n", sub_ret, keyexpr);
        s->subscribers[idx].callback_ext = NULL;
        s->subscribers[idx].ctx = NULL;
        return ZPICO_ERR_GENERIC;
    }

    s->subscribers[idx].active = true;
    return idx;
}

int32_t zpico_declare_subscriber_direct_write(zpico_session_t* session, const char* keyexpr,
                                              uint8_t* buf_ptr, size_t buf_capacity,
                                              const bool* locked_ptr, ZpicoNotifyCallback callback,
                                              void* ctx) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (!s->session_open) {
        return ZPICO_ERR_SESSION;
    }

    // Find free slot
    int idx = -1;
    for (int i = 0; i < ZPICO_MAX_SUBSCRIBERS; i++) {
        if (!s->subscribers[i].active) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        return ZPICO_ERR_FULL;
    }

    s->subscribers[idx].notify = callback;
    s->subscribers[idx].ctx = ctx;
    s->subscribers[idx].with_attachment = false;
    s->subscribers[idx].direct_write = true;
    s->subscribers[idx].buf_ptr = buf_ptr;
    s->subscribers[idx].buf_capacity = buf_capacity;
    s->subscribers[idx].locked_ptr = locked_ptr;

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        s->subscribers[idx].notify = NULL;
        s->subscribers[idx].ctx = NULL;
        s->subscribers[idx].direct_write = false;
        return ZPICO_ERR_KEYEXPR;
    }

    // Create closure for callback, passing index as context
    z_owned_closure_sample_t closure;
    z_closure_sample(&closure, sample_handler, NULL, _zpico_pack_ctx(s, idx));

    int sub_ret =
        z_declare_subscriber(z_session_loan(&s->session), &s->subscribers[idx].subscriber,
                             z_view_keyexpr_loan(&ke), z_closure_sample_move(&closure), NULL);
    if (sub_ret < 0) {
        printk("zpico: z_declare_subscriber failed: %d for '%s'\n", sub_ret, keyexpr);
        s->subscribers[idx].notify = NULL;
        s->subscribers[idx].ctx = NULL;
        s->subscribers[idx].direct_write = false;
        return ZPICO_ERR_GENERIC;
    }

    s->subscribers[idx].active = true;
    return idx;
}

/* phase-412 -- which exit `zpico_declare_subscriber_ring` took, and the raw
 * zenoh-pico code when the declare itself is what failed.
 *
 * The return value cannot carry this. Five exits collapse onto four ZPICO_ERR_*
 * codes, and ZPICO_ERR_GENERIC in particular DISCARDS `sub_ret`, the only
 * number saying what zenoh-pico objected to. The printk beside it carries both
 * -- to the console, which is not wired on the MR-CANHUBK344. On the one board
 * that needed the message it is written and unreadable.
 *
 * Globals rather than an out-param, so the ABI does not move: the Rust caller
 * reads them immediately after a failed call, on a path that already failed.
 *
 * Numbering is stable and append-only. 0 after a call means NO exit stamped,
 * which is itself a finding when the caller reports an error it believes came
 * from here.
 */
int32_t zpico_last_sub_declare_exit = 0;
int32_t zpico_last_sub_declare_ret = 0;

int32_t zpico_declare_subscriber_ring(zpico_session_t* session, const char* keyexpr,
                                      zpico_ring_desc_t* desc, ZpicoNotifyCallback callback,
                                      void* ctx) {
    struct zpico_session* s = (struct zpico_session*)session;
    /* Reset at entry: a value left by an earlier call must not be read as
     * this one's. 0 therefore means "entered but stamped no exit". */
    zpico_last_sub_declare_exit = 0;
    zpico_last_sub_declare_ret = 0;
    if (!s->session_open) {
        zpico_last_sub_declare_exit = 1;
        return ZPICO_ERR_SESSION;
    }
    if (desc == NULL || desc->slot_count == 0) {
        zpico_last_sub_declare_exit = 2;
        return ZPICO_ERR_INVALID;
    }

    int idx = -1;
    for (int i = 0; i < ZPICO_MAX_SUBSCRIBERS; i++) {
        if (!s->subscribers[i].active) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        zpico_last_sub_declare_exit = 3;
        return ZPICO_ERR_FULL;
    }

    s->subscribers[idx].notify = callback;
    s->subscribers[idx].ctx = ctx;
    s->subscribers[idx].with_attachment = false;
    s->subscribers[idx].direct_write = false;
    s->subscribers[idx].ring_mode = true;
    s->subscribers[idx].ring = desc;

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        s->subscribers[idx].notify = NULL;
        s->subscribers[idx].ctx = NULL;
        s->subscribers[idx].ring_mode = false;
        s->subscribers[idx].ring = NULL;
        zpico_last_sub_declare_exit = 4;
        return ZPICO_ERR_KEYEXPR;
    }

    z_owned_closure_sample_t closure;
    z_closure_sample(&closure, sample_handler, NULL, _zpico_pack_ctx(s, idx));

    int sub_ret =
        z_declare_subscriber(z_session_loan(&s->session), &s->subscribers[idx].subscriber,
                             z_view_keyexpr_loan(&ke), z_closure_sample_move(&closure), NULL);
    if (sub_ret < 0) {
        printk("zpico: z_declare_subscriber (ring) failed: %d for '%s'\n", sub_ret, keyexpr);
        s->subscribers[idx].notify = NULL;
        s->subscribers[idx].ctx = NULL;
        s->subscribers[idx].ring_mode = false;
        s->subscribers[idx].ring = NULL;
        zpico_last_sub_declare_exit = 5;
        zpico_last_sub_declare_ret = (int32_t)sub_ret;
        return ZPICO_ERR_GENERIC;
    }

    s->subscribers[idx].active = true;
        zpico_last_sub_declare_exit = 6;
    return idx;
}

#if defined(Z_FEATURE_UNSTABLE_API)
int32_t zpico_subscribe_zero_copy(zpico_session_t* session, const char* keyexpr,
                                  ZpicoZeroCopyCallback callback, void* ctx) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (!s->session_open) {
        return ZPICO_ERR_SESSION;
    }

    // Find free slot
    int idx = -1;
    for (int i = 0; i < ZPICO_MAX_SUBSCRIBERS; i++) {
        if (!s->subscribers[i].active) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        return ZPICO_ERR_FULL;
    }

    s->subscribers[idx].ctx = ctx;
    s->subscribers[idx].with_attachment = false;
    s->subscribers[idx].direct_write = false;
    s->subscribers[idx].zero_copy = true;
    s->subscribers[idx].zero_copy_cb = callback;

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        s->subscribers[idx].zero_copy = false;
        s->subscribers[idx].zero_copy_cb = NULL;
        s->subscribers[idx].ctx = NULL;
        return ZPICO_ERR_KEYEXPR;
    }

    // Create closure for callback, passing index as context
    z_owned_closure_sample_t closure;
    z_closure_sample(&closure, sample_handler, NULL, _zpico_pack_ctx(s, idx));

    int sub_ret =
        z_declare_subscriber(z_session_loan(&s->session), &s->subscribers[idx].subscriber,
                             z_view_keyexpr_loan(&ke), z_closure_sample_move(&closure), NULL);
    if (sub_ret < 0) {
        printk("zpico: z_declare_subscriber (zero_copy) failed: %d for '%s'\n", sub_ret, keyexpr);
        s->subscribers[idx].zero_copy = false;
        s->subscribers[idx].zero_copy_cb = NULL;
        s->subscribers[idx].ctx = NULL;
        return ZPICO_ERR_GENERIC;
    }

    s->subscribers[idx].active = true;
    return idx;
}
#else
// Stub when unstable API is not enabled — returns error
int32_t zpico_subscribe_zero_copy(zpico_session_t* session, const char* keyexpr,
                                  ZpicoZeroCopyCallback callback, void* ctx) {
    (void)session;
    (void)keyexpr;
    (void)callback;
    (void)ctx;
    return ZPICO_ERR_GENERIC;
}
#endif

int32_t zpico_undeclare_subscriber(zpico_session_t* session, int32_t handle) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (handle < 0 || handle >= ZPICO_MAX_SUBSCRIBERS || !s->subscribers[handle].active) {
        return ZPICO_ERR_INVALID;
    }

    z_undeclare_subscriber(z_subscriber_move(&s->subscribers[handle].subscriber));
    s->subscribers[handle].active = false;
    s->subscribers[handle].callback = NULL;
    s->subscribers[handle].ctx = NULL;
    s->subscribers[handle].with_attachment = false;
    return ZPICO_OK;
}

// ============================================================================
// Socket FD Helper (for select()-based timeout)
// ============================================================================

// get_session_fd() is only needed for select()-based paths. Most
// multi-threaded builds use background tasks, but ThreadX/NSOS uses the same
// select + read + keepalive path as single-threaded hosts.
#if !defined(ZPICO_SMOLTCP) && !defined(ZPICO_SERIAL) && !defined(ZENOH_FREERTOS_LWIP) &&          \
    (Z_FEATURE_MULTI_THREAD != 1 || defined(ZENOH_THREADX))
/**
 * Extract the socket file descriptor from the zenoh session.
 *
 * Path: s->session → _z_session_t._tp._transport._unicast._peers → first peer → _socket
 *
 * Returns -1 if the session is not unicast or has no connected peers.
 */
static int get_session_fd(struct zpico_session* s) {
    _z_session_t* session = _Z_RC_IN_VAL(z_session_loan(&s->session));
    if (session->_tp._type != _Z_TRANSPORT_UNICAST_TYPE) {
        return -1;
    }
    _z_transport_peer_unicast_t* peer =
        _z_transport_peer_unicast_slist_value(session->_tp._transport._unicast._peers);
    if (peer == NULL) {
        return -1;
    }
    // Phase 154 — read the BSD fd via the platform accessor
    // instead of reaching into the per-RTOS `_z_sys_net_socket_t`
    // struct fields. With `NROS_PLATFORM_ALIASES` now defined for
    // the vendor build (so the socket ABI matches the alias TU's
    // 32-byte opaque layout), the `._fd` / `._socket` field names
    // no longer exist at compile time in this TU. Every backend
    // stores `int fd` at offset 0 of its socket struct, so
    // `nros_platform_socket_get_fd` works uniformly across
    // FreeRTOS+lwIP, POSIX, ThreadX, and bare-metal.
    return nros_platform_socket_get_fd(&peer->_socket);
}
#endif

// ============================================================================
// Polling Implementation (zpico_poll — deleted in Phase 77.20; use
// zpico_spin_once() instead, which adds keep-alive handling)
// ============================================================================

int32_t zpico_spin_once(zpico_session_t* session, uint32_t timeout_ms) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (!s->session_open) {
        return ZPICO_ERR_SESSION;
    }

#if defined(ZPICO_TX_BATCH) && ZPICO_TX_BATCH == 1 && ZPICO_TX_BATCH_THREAD == 0
    /* phase-279 (#145) — rate-limited batch flush (platforms WITHOUT the
     * dedicated tx-flush thread: ThreadX + single-threaded). zenoh-pico's flush HOLDS the
     * transport tx mutex across the whole socket send (fd wait included), so
     * puts cannot append while a flush is in flight — the batch only
     * accumulates BETWEEN flushes. Flushing on every spin (1 ms tiers) ships
     * <=1 message per send and the flushes themselves compete with puts for
     * the send window, which MEASURED WORSE than no batching (4.7-4.9 vs 8.6
     * msg/s on the W1 harness). Flushing at a bounded cadence lets puts pile
     * into the write buffer cheaply and ships them as ONE send per interval.
     * Racy across tier threads sharing this session is benign (worst case one
     * extra flush). zenoh-pico's implicit flushes (buffer overflow, any
     * transport message) still bound memory + sit-time independently of this
     * cadence. The cadence state is per-session (was function-local statics). */
    {
        if (!s->batch_flush_init ||
            z_clock_elapsed_ms(&s->last_batch_flush) >= ZPICO_TX_BATCH_FLUSH_MS) {
            zp_batch_flush(z_session_loan(&s->session));
            s->last_batch_flush = z_clock_now();
            s->batch_flush_init = true;
        }
    }
#endif

#ifdef ZPICO_SMOLTCP
    // smoltcp: poll network and read available data. Uses single_read=true
    // to preserve partial TCP data across calls (non-blocking _z_read_tcp
    // may return fragments). single_read=false resets the zbuf on each call
    // which discards partial messages.
    //
    // Drain behaviour (timeout_ms == 0):
    //   Loop until no data remains. This prevents a race where a keep-alive
    //   is read first, the loop exits, _z_pending_query_process_timeout fires
    //   and removes the pending query, and then the reply — still in the
    //   staging buffer — is read in the next spin but cannot find its matching
    //   pending query (already removed), causing get_check to return TIMEOUT
    //   even though the reply arrived in time.
    //
    // Polling behaviour (timeout_ms > 0):
    //   Stop after the first processed message (original behaviour), and
    //   retry until data arrives or the timeout elapses.
    zp_read_options_t opts;
    opts.single_read = true;
    uint64_t start = smoltcp_clock_now_ms();
    int ret;
    do {
        ret = zp_read(z_session_loan_mut(&s->session), &opts);
        if (ret == 0 && timeout_ms != 0) break; // Processed one message (timeout mode)
        if (ret != 0 && timeout_ms == 0) break; // No data (drain mode)
        // timeout_ms == 0 && ret == 0: data processed, loop to drain next message
        // timeout_ms != 0 && ret != 0: no data yet, keep waiting (while decides)
    } while (ret == 0 || smoltcp_clock_now_ms() - start < timeout_ms);
    // Process query timeouts — in multi-threaded mode the lease task handles
    // this, but on single-threaded bare-metal (smoltcp) there is no lease task.
    // Without this call, timed-out queries are never cleaned up and their
    // dropper callbacks never fire, breaking service/action request flows.
    _z_pending_query_process_timeout(_Z_RC_IN_VAL(z_session_loan_mut(&s->session)));
    zp_send_keep_alive(z_session_loan_mut(&s->session), NULL);
    return ret;

#elif defined(ZPICO_SERIAL)
    // Serial-only bare-metal: poll UART and read available data.
    // Uses single_read=false because the single_read=true path calls
    // _z_unicast_recv_t_msg which returns _Z_ERR_TRANSPORT_RX_FAILED (-99)
    // on no data for datagram links. The single_read=false path uses
    // _z_unicast_client_read which returns _Z_NO_DATA_PROCESSED gracefully.
    // For serial (datagram), each COBS frame is atomic so there's no
    // partial data to preserve across calls (unlike TCP stream).
    z_clock_t start = z_clock_now();
    int ret;
    do {
        ret = zp_read(z_session_loan_mut(&s->session), NULL);
        if (ret == 0) break; // Data processed
        if (timeout_ms == 0) break;
    } while (z_clock_elapsed_ms(&start) < timeout_ms);
    _z_pending_query_process_timeout(_Z_RC_IN_VAL(z_session_loan_mut(&s->session)));
    zp_send_keep_alive(z_session_loan_mut(&s->session), NULL);
    return ret;

#elif defined(ZENOH_THREADX)
    // ThreadX/NSOS: avoid zenoh-pico background read/lease tasks. The read
    // task can block in the host-backed BSD recv path and prevent the lease
    // task from running before the 10s router lease expires. Poll here instead
    // so every executor spin also refreshes the transport keepalive.
    //
    // phase-297 W5 — the timed wait MUST be `z_sleep_ms` (tx_thread_sleep),
    // never a host select(): a host syscall blocks the pthread while the
    // ThreadX scheduler still counts this thread as running, so under strict
    // priority scheduling every other tier thread starves (the multi-tier
    // e2e saw the higher-priority tier's select() wait pin the whole image
    // to one tier — zero publishes from the other). Sleep first (a REAL
    // ThreadX yield — lower-priority tiers run inside it), then poll the fd
    // with a zero timeout and drain only what is already there. Same shape
    // as the other multi-threaded platforms (see the pitfall in
    // platform-implementation-notes.md).
    // phase-297 W5 / issue #247 — single-reader rule: the polled `zp_read`
    // path (`_zp_unicast_read`, single_read=false) does NOT take the
    // transport's `_mutex_rx` (only the background read task does), so two
    // tier threads polling concurrently race on the shared rx zbuf
    // (`_z_zbuf_reset` mid parse) and silently LOSE inbound frames — both
    // debugging tracks saw the spawned tier's interest replies vanish,
    // leaving its write filter closed forever (zero publishes).
    // Serialization is `s->threadx_read_mutex` (TRY-lock; a losing spinner
    // skips the round — the winner drains) via `_zpico_threadx_locked_read`
    // below; the frame-loss half of the bug is fixed in zenoh-pico itself
    // (`87f7a84d` — the polled read drains every buffered frame).
    int fd = get_session_fd(s);
    // issue 0387 — start at 0 (Ok "no work this round"), NOT a negative
    // timeout code. An idle select (`ready == 0`, no inbound frame) is the
    // steady state on a live-but-quiet session; every other multi-threaded
    // backend (FreeRTOS/Zephyr/POSIX/NuttX, below) returns 0 there. Returning
    // ZPICO_ERR_TIMEOUT made each idle spin increment the executor's
    // `consecutive_io_failures`, so nros-c's `nros_executor_spin_period` bailed
    // after SPIN_ERROR_TOLERANCE (16) quiet rounds and killed the session
    // (issue-0355's "idle spins must NEVER accumulate session_io_failures").
    // Only a genuine select error (`ready < 0`) or a real `zp_read` failure
    // still returns a negative code below.
    int ret = 0;
    if (timeout_ms > 0) {
        z_sleep_ms(timeout_ms);
    }
    if (fd >= 0) {
#if defined(__linux__)
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        int ready = select(fd + 1, &read_fds, NULL, NULL, &tv);
#else
        nx_bsd_fd_set read_fds;
        NX_BSD_FD_ZERO(&read_fds);
        NX_BSD_FD_SET(fd, &read_fds);
        struct nx_bsd_timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        int ready = nx_bsd_select(fd + 1, &read_fds, NULL, NULL, &tv);
#endif
        if (ready > 0) {
#if Z_FEATURE_MULTI_THREAD == 1
            /* issue #247 — one reader at a time (s->threadx_read_mutex). */
            ret = _zpico_threadx_locked_read(s);
#else
            ret = zp_read(z_session_loan_mut(&s->session), NULL);
#endif
        } else if (ready < 0) {
            ret = ready;
        }
    } else {
#if Z_FEATURE_MULTI_THREAD == 1
        ret = _zpico_threadx_locked_read(s);
#else
        ret = zp_read(z_session_loan_mut(&s->session), NULL);
#endif
    }
    _z_pending_query_process_timeout(_Z_RC_IN_VAL(z_session_loan_mut(&s->session)));
    zp_send_keep_alive(z_session_loan_mut(&s->session), NULL);
    return ret;

#elif defined(ZENOH_FREERTOS_LWIP)
    // FreeRTOS+lwIP: background read and lease tasks handle data and
    // keep-alives. Wait on a binary semaphore that _zpico_notify_spin()
    // signals when application data arrives (subscriptions, query replies).
    // This gives near-zero latency wake-up without busy-looping.
    if (timeout_ms > 0 && s->spin_sem != NULL) {
        xSemaphoreTake(s->spin_sem, pdMS_TO_TICKS(timeout_ms));
    }
    return 0;

#elif Z_FEATURE_MULTI_THREAD == 1
    // Multi-threaded (Zephyr/POSIX/NuttX): background read and lease tasks
    // handle data and keep-alives. Wait on a wake primitive that our
    // callbacks signal when application data arrives (subscriptions, query
    // replies, service requests). This gives near-zero latency wake-up
    // without busy-looping on select().
    //
    // NuttX note: the pthread-timed-wait path (`pthread_cond_timedwait`)
    // hangs indefinitely inside the kernel's watchdog-backed semaphore
    // wait (Phase 55.12 follow-up), so we can't reuse the condvar path
    // there. POSIX `sem_timedwait` does not share that code path and is
    // safe — it gives us the same early-wake optimisation the other
    // multi-threaded backends enjoy, replacing the old blind
    // `usleep(timeout_ms * 1000)` busy-sleep.
    if (timeout_ms > 0) {
#ifdef ZENOH_NUTTX
        if (s->spin_sem_initialized) {
            struct timespec deadline;
            clock_gettime(CLOCK_REALTIME, &deadline);
            deadline.tv_sec += (time_t)(timeout_ms / 1000);
            deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
            if (deadline.tv_nsec >= 1000000000L) {
                deadline.tv_sec += 1;
                deadline.tv_nsec -= 1000000000L;
            }
            // EINTR / ETIMEDOUT are both acceptable — the outer executor
            // loop re-checks arena state regardless of why we woke up.
            while (sem_timedwait(&s->spin_sem_posix, &deadline) != 0 && errno == EINTR) {
                // retry on signal
            }
        } else {
            // Fallback if sem_init failed at session open.
            usleep((useconds_t)timeout_ms * 1000);
        }
#else
        z_clock_t deadline = z_clock_now();
        z_clock_advance_ms(&deadline, (unsigned long)timeout_ms);
        _z_mutex_lock(&s->spin_mutex);
        _z_condvar_wait_until(&s->spin_cv, &s->spin_mutex, &deadline);
        _z_mutex_unlock(&s->spin_mutex);
#endif
    }
    return 0;

#else
    // Single-threaded (not smoltcp): use select() then zp_read()
    int fd = get_session_fd(s);
    if (fd >= 0 && timeout_ms > 0) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        int result = select(fd + 1, &read_fds, NULL, NULL, &tv);
        if (result <= 0) {
            zp_send_keep_alive(z_session_loan_mut(&s->session), NULL);
            return (result == 0) ? ZPICO_ERR_TIMEOUT : result;
        }
    }
    int ret = zp_read(z_session_loan_mut(&s->session), NULL);
    zp_send_keep_alive(z_session_loan_mut(&s->session), NULL);
    return ret;
#endif
}

bool zpico_uses_polling(void) {
    // Returns true if multi-threading is disabled
#if Z_FEATURE_MULTI_THREAD == 0
    return true;
#else
    return false;
#endif
}

// ============================================================================
// ZenohId Implementation
// ============================================================================

int32_t zpico_get_zid(zpico_session_t* session, uint8_t* zid_out) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (!s->session_open || zid_out == NULL) {
        return ZPICO_ERR_SESSION;
    }

    z_id_t zid = z_info_zid(z_session_loan(&s->session));
    memcpy(zid_out, zid.id, 16);
    return ZPICO_OK;
}

// ============================================================================
// Liveliness Implementation
// ============================================================================

int32_t zpico_declare_liveliness(zpico_session_t* session, const char* keyexpr) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (!s->session_open) {
        return ZPICO_ERR_SESSION;
    }

    // Find free slot
    int idx = -1;
    for (int i = 0; i < ZPICO_MAX_LIVELINESS; i++) {
        if (!s->liveliness[i].active) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        return ZPICO_ERR_FULL;
    }

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        return ZPICO_ERR_KEYEXPR;
    }

    int lv_ret = z_liveliness_declare_token(z_session_loan(&s->session), &s->liveliness[idx].token,
                                            z_view_keyexpr_loan(&ke), NULL);
    if (lv_ret < 0) {
        /* issue 0283 — a failed token is a SILENT graph outage (the ROS 2
         * tools see nothing) — say so on the console like the publisher /
         * subscriber declare paths do. */
        printk("zpico: z_liveliness_declare_token failed: %d for '%s'\n", lv_ret, keyexpr);
        return ZPICO_ERR_GENERIC;
    }

    s->liveliness[idx].active = true;
    return idx;
}

int32_t zpico_undeclare_liveliness(zpico_session_t* session, int32_t handle) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (handle < 0 || handle >= ZPICO_MAX_LIVELINESS || !s->liveliness[handle].active) {
        return ZPICO_ERR_INVALID;
    }

    z_liveliness_undeclare_token(z_liveliness_token_move(&s->liveliness[handle].token));
    s->liveliness[handle].active = false;
    return ZPICO_OK;
}

// ============================================================================
// Publish with Attachment Implementation
// ============================================================================

int32_t zpico_publish_with_attachment(zpico_session_t* session, int32_t handle, const uint8_t* data,
                                      size_t len, const uint8_t* attachment,
                                      size_t attachment_len) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (handle < 0 || handle >= ZPICO_MAX_PUBLISHERS || !s->publishers[handle].active) {
        return ZPICO_ERR_INVALID;
    }

    // Create payload
    z_owned_bytes_t payload;
    if (z_bytes_copy_from_buf(&payload, data, len) < 0) {
        return ZPICO_ERR_PUBLISH;
    }

    // Create put options with attachment
    z_publisher_put_options_t options;
    z_publisher_put_options_default(&options);

    z_owned_bytes_t attachment_bytes;
    if (attachment != NULL && attachment_len > 0) {
        if (z_bytes_copy_from_buf(&attachment_bytes, attachment, attachment_len) < 0) {
            z_bytes_drop(z_bytes_move(&payload));
            return ZPICO_ERR_PUBLISH;
        }
        options.attachment = z_bytes_move(&attachment_bytes);
    }

    if (z_publisher_put(z_publisher_loan(&s->publishers[handle].publisher), z_bytes_move(&payload),
                        &options) < 0) {
        return ZPICO_ERR_PUBLISH;
    }

    return ZPICO_OK;
}

// Phase 99.F — zero-copy publish via z_bytes_from_static_buf.
// Aliases the payload pointer instead of copying. Caller guarantees
// `data` outlives the call (z_publisher_put consumes the alias
// synchronously on posix/embedded transports). Attachment is still
// copied (small, fixed size).
int32_t zpico_publish_with_attachment_aliased(zpico_session_t* session, int32_t handle,
                                              const uint8_t* data, size_t len,
                                              const uint8_t* attachment, size_t attachment_len) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (handle < 0 || handle >= ZPICO_MAX_PUBLISHERS || !s->publishers[handle].active) {
        return ZPICO_ERR_INVALID;
    }

    // Alias the payload — no copy. zenoh-pico writes directly from
    // the caller-supplied buffer.
    z_owned_bytes_t payload;
    if (z_bytes_from_static_buf(&payload, data, len) < 0) {
        return ZPICO_ERR_PUBLISH;
    }

    z_publisher_put_options_t options;
    z_publisher_put_options_default(&options);

    z_owned_bytes_t attachment_bytes;
    if (attachment != NULL && attachment_len > 0) {
        if (z_bytes_copy_from_buf(&attachment_bytes, attachment, attachment_len) < 0) {
            z_bytes_drop(z_bytes_move(&payload));
            return ZPICO_ERR_PUBLISH;
        }
        options.attachment = z_bytes_move(&attachment_bytes);
    }

    if (z_publisher_put(z_publisher_loan(&s->publishers[handle].publisher), z_bytes_move(&payload),
                        &options) < 0) {
        return ZPICO_ERR_PUBLISH;
    }

    return ZPICO_OK;
}

// ============================================================================
// Queryable Implementation (for ROS 2 Services)
// ============================================================================

int32_t zpico_declare_queryable(zpico_session_t* session, const char* keyexpr,
                                ZpicoQueryCallback callback, void* ctx) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (!s->session_open) {
        return ZPICO_ERR_SESSION;
    }

    // Find free slot
    int idx = -1;
    for (int i = 0; i < ZPICO_MAX_QUERYABLES; i++) {
        if (!s->queryables[i].active) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        return ZPICO_ERR_FULL;
    }

    s->queryables[idx].callback = callback;
    s->queryables[idx].ctx = ctx;

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        s->queryables[idx].callback = NULL;
        s->queryables[idx].ctx = NULL;
        return ZPICO_ERR_KEYEXPR;
    }

    // Create closure for callback
    z_owned_closure_query_t closure;
    z_closure_query(&closure, query_handler, NULL, _zpico_pack_ctx(s, idx));

    // Set complete=true so that queries with Z_QUERY_TARGET_ALL_COMPLETE
    // (used by rmw_zenoh_cpp service clients) match this queryable.
    z_queryable_options_t opts;
    z_queryable_options_default(&opts);
    opts.complete = true;

    int q_ret =
        z_declare_queryable(z_session_loan(&s->session), &s->queryables[idx].queryable,
                            z_view_keyexpr_loan(&ke), z_closure_query_move(&closure), &opts);
    if (q_ret < 0) {
        printk("zpico: z_declare_queryable failed: %d for '%s'\n", q_ret, keyexpr);
        s->queryables[idx].callback = NULL;
        s->queryables[idx].ctx = NULL;
        return ZPICO_ERR_GENERIC;
    }

    s->queryables[idx].active = true;
    return idx;
}

int32_t zpico_undeclare_queryable(zpico_session_t* session, int32_t handle) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (handle < 0 || handle >= ZPICO_MAX_QUERYABLES || !s->queryables[handle].active) {
        return ZPICO_ERR_INVALID;
    }

    z_undeclare_queryable(z_queryable_move(&s->queryables[handle].queryable));
    s->queryables[handle].active = false;
    s->queryables[handle].callback = NULL;
    s->queryables[handle].ctx = NULL;
    // Phase 237 — drop any cloned queries still held in this queryable's reply
    // slots (unanswered deferred requests) so they don't leak session refs.
    for (int j = 0; j < ZPICO_MAX_PENDING_REPLIES; j++) {
        if (s->stored_query_valid[handle][j]) {
            z_query_drop(z_query_move(&s->stored_queries[handle][j]));
            s->stored_query_valid[handle][j] = false;
        }
    }
    s->last_reply_seq[handle] = -1;
    return ZPICO_OK;
}

// ============================================================================
// Service Client Implementation (z_get for ROS 2 service calls)
// ============================================================================

/**
 * Internal callback for z_get reply handling
 */
static void get_reply_handler(z_loaned_reply_t* reply, void* ctx) {
    get_reply_ctx_t* rctx = (get_reply_ctx_t*)ctx;

    // Only process successful replies
    if (!z_reply_is_ok(reply)) {
        g_diag_reply_not_ok++;
        return;
    }

    /* Bump the multi-reply count regardless of payload-buffer state.
     * Liveliness queries care about this count; single-response gets
     * ignore it. */
    rctx->reply_count++;

    /* phase-381 W1 — collection mode: the KEYEXPR is the payload.
     *
     * Appended NUL-terminated into the same `buf` a single-response get uses
     * for its payload; the two modes never share a slot, because `collect` is
     * set once at start and a slot serves one query. A keyexpr that does not
     * fit is DROPPED rather than truncated — half a keyexpr parses as a
     * different, plausible entity, which is worse than a missing one — and the
     * caller sees it as `reply_count > entry_count`. */
    if (rctx->collect) {
        const z_loaned_sample_t* ksample = z_reply_ok(reply);
        const z_loaned_keyexpr_t* ke = z_sample_keyexpr(ksample);
        z_view_string_t kstr;
        z_keyexpr_as_view_string(ke, &kstr);
        const char* kdata = z_string_data(z_view_string_loan(&kstr));
        size_t klen = z_string_len(z_view_string_loan(&kstr));
        /* +1 for the NUL that separates entries. */
        if (klen > 0 && rctx->len + klen + 1 <= ZPICO_GET_REPLY_BUF_SIZE) {
            memcpy(rctx->buf + rctx->len, kdata, klen);
            rctx->len += klen;
            rctx->buf[rctx->len++] = '\0';
            rctx->entry_count++;
        }
        /* `received` marks "this slot has something to read", which for a
         * collecting query is true from the first stored entry. */
        if (rctx->entry_count > 0) {
            __atomic_store_n(&rctx->received, true, __ATOMIC_SEQ_CST);
        }
        return;
    }

    // Skip if we already have a reply (only take first)
    if (rctx->received) {
        g_diag_reply_already_received++;
        return;
    }

    const z_loaned_sample_t* sample = z_reply_ok(reply);
    const z_loaned_bytes_t* payload = z_sample_payload(sample);

    // Copy payload to reply buffer
    z_owned_slice_t slice;
    if (z_bytes_to_slice(payload, &slice) == 0) {
        const uint8_t* data = z_slice_data(z_slice_loan(&slice));
        size_t len = z_slice_len(z_slice_loan(&slice));

        if (len <= ZPICO_GET_REPLY_BUF_SIZE) {
            memcpy(rctx->buf, data, len);
            rctx->len = len;
            __atomic_store_n(&rctx->received, true, __ATOMIC_SEQ_CST);
            g_diag_reply_received_set++;
        } else {
            g_diag_reply_too_big++;
        }
        z_slice_drop(z_slice_move(&slice));
    } else {
        g_diag_reply_to_slice_fail++;
    }
}

/**
 * Internal callback for z_get completion (dropper)
 */
static void get_reply_dropper(void* ctx) {
    get_reply_ctx_t* rctx = (get_reply_ctx_t*)ctx;
#if Z_FEATURE_MULTI_THREAD == 1
    _z_mutex_lock(&rctx->mutex);
    rctx->done = true;
    _z_condvar_signal(&rctx->cond);
    _z_mutex_unlock(&rctx->mutex);
#else
    rctx->done = true;
#endif
}

int32_t zpico_get(zpico_session_t* session, const char* keyexpr, const uint8_t* payload,
                  size_t payload_len, uint8_t* reply_buf, size_t reply_buf_size,
                  uint32_t timeout_ms) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (!s->session_open) {
        return ZPICO_ERR_SESSION;
    }

    // Stack-allocated reply context (safe for concurrent z_get calls)
    get_reply_ctx_t ctx;
    ctx.len = 0;
    ctx.received = false;
    ctx.done = false;
    ctx.reply_count = 0;
#if Z_FEATURE_MULTI_THREAD == 1
    _z_mutex_init(&ctx.mutex);
    _z_condvar_init(&ctx.cond);
#endif

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        return ZPICO_ERR_KEYEXPR;
    }

    // Set up get options
    z_get_options_t opts;
    z_get_options_default(&opts);
#if defined(ZPICO_TX_BATCH) && ZPICO_TX_BATCH == 1
    /* phase-279 (#145) — service/query latency guard: gets bypass the batch
     * (express) so request RTT gains no spin-period latency under batching. */
    opts.is_express = true;
#endif
    opts.target = Z_QUERY_TARGET_ALL;
    opts.timeout_ms = timeout_ms;
    // Use NONE consolidation so the reply callback fires immediately on each
    // partial reply, rather than AUTO (which becomes LATEST and buffers replies
    // until the ReplyFinal arrives). With LATEST, the dropper fires before the
    // reply callback, leaving received=false when the condvar is signalled.
    opts.consolidation.mode = Z_CONSOLIDATION_MODE_NONE;

    // Set payload if provided
    z_owned_bytes_t payload_bytes;
    if (payload != NULL && payload_len > 0) {
        if (z_bytes_copy_from_buf(&payload_bytes, payload, payload_len) < 0) {
            return ZPICO_ERR_GENERIC;
        }
        opts.payload = z_bytes_move(&payload_bytes);
    }

    // Create closure for reply handling with stack context
    z_owned_closure_reply_t callback;
    z_closure(&callback, get_reply_handler, get_reply_dropper, &ctx);

    // Send the query
    if (z_get(z_session_loan(&s->session), z_view_keyexpr_loan(&ke), "", z_move(callback), &opts) <
        0) {
        return ZPICO_ERR_GENERIC;
    }

    // For multi-threaded platforms, wait for reply via background threads
    // For single-threaded platforms, poll until reply or timeout
#if Z_FEATURE_MULTI_THREAD == 0
    // Single-threaded: poll until reply received or timeout.
    //
    // We drive _z_pending_query_process_timeout() on every iteration so that
    // zenoh-pico's own deadline fires the dropper (setting ctx.done = true)
    // while ctx is still live on the stack.  Without this, the wall-clock
    // check below would break out of the loop while a stale pending-query
    // entry still points at ctx, and the dropper would later fire on a
    // dangling pointer, silently corrupting the stack of the next caller.
    //
    // The wall-clock guard (timeout_ms + 2000 ms) is belt-and-suspenders:
    // in normal operation _z_pending_query_process_timeout fires the dropper
    // at opts.timeout_ms (== timeout_ms), so ctx.done becomes true before the
    // guard triggers.
    {
        z_clock_t start = z_clock_now();
        while (!ctx.done) {
            zp_read(z_session_loan_mut(&s->session), NULL);
            zp_send_keep_alive(z_session_loan_mut(&s->session), NULL);
            // Drive zenoh-pico's query timeout so its dropper fires cleanly
            // while ctx is still on the stack.
            _z_pending_query_process_timeout(_Z_RC_IN_VAL(z_session_loan_mut(&s->session)));
            if (ctx.received) {
                break;
            }
            // Safety wall-clock guard (fires 2 s after the zenoh deadline).
            if ((uint32_t)z_clock_elapsed_ms(&start) >= timeout_ms + 2000) {
                break;
            }
        }
    }
#else
    // Multi-threaded: wait for completion via condvar.
    // The dropper callback signals ctx.cond when the reply channel closes.
    //
    // IMPORTANT: Do NOT use _z_condvar_wait_until with a deadline here.
    // Zenoh fires the dropper at opts.timeout_ms — rely on that timeout
    // instead of a parallel OS deadline. Using a separate deadline creates
    // a use-after-free race: if the OS deadline expires first, we drop
    // ctx.mutex/ctx.cond while the background thread's dropper is still
    // pending, causing the dropper to lock/signal freed memory.
    {
        _z_mutex_lock(&ctx.mutex);
        while (!ctx.done) {
            _z_condvar_wait(&ctx.cond, &ctx.mutex);
        }
        _z_mutex_unlock(&ctx.mutex);
    }
    _z_condvar_drop(&ctx.cond);
    _z_mutex_drop(&ctx.mutex);
#endif

    // Check if we got a reply
    if (!ctx.received) {
        return -9; // ZPICO_ERR_TIMEOUT (defined in Rust FFI)
    }

    // Copy reply to output buffer
    if (ctx.len > reply_buf_size) {
        return ZPICO_ERR_FULL; // Buffer too small
    }

    memcpy(reply_buf, ctx.buf, ctx.len);
    return (int32_t)ctx.len;
}

// ============================================================================
// Non-blocking z_get (for async service client)
// ============================================================================

// Reply handler for pending get slots — reuses the same logic as get_reply_handler
static void pending_get_reply_handler(z_loaned_reply_t* reply, void* arg) {
    g_diag_reply_handler_calls++;
    struct zpico_session* s = _zpico_unpack_session(arg);
    int slot = _zpico_unpack_slot(arg);
    get_reply_ctx_t* rctx = &s->pending_gets[slot].ctx;
    /* Phase 127.D.2 — keep the address-recording side effect. It
     * defeats whole-program LTO alias analysis that would otherwise
     * prove the closure's `ctx` pointer disjoint from
     * `&s->pending_gets[handle].ctx` (because the slot table is
     * private to this TU and the callback type-erases `ctx` to
     * `void *`). Without this side effect the write to
     * `rctx->received` here was hoisted away from the read in
     * `zpico_get_check`, leaving the polling client spinning on a
     * stale `false`. _Atomic / volatile alone did not change the
     * symptom; recording the address through a non-static observer
     * does. See docs/research/qemu-lan9118-slirp-rx-stall.md.
     *
     * Bumping a counter unconditionally (vs. the "first write only"
     * pattern) keeps the side effect from being scheduled as a
     * one-shot branch that constant-folds away after the first hit. */
    /* Record the REAL reply-context address (not the packed handle): a volatile
     * store of an address derived from the pooled `g_sessions` array keeps the
     * write to `rctx->received` in `get_reply_handler` from being hoisted over
     * the read in `zpico_get_check`. */
    g_diag_handler_ctx_addr = (uint32_t)(uintptr_t)rctx;
    get_reply_handler(reply, rctx);
    _zpico_notify_spin(s);
    if (s->reply_waker) {
        s->reply_waker((int32_t)(s - g_sessions), slot);
    }
}

// Dropper for pending get slots — just sets the done flag (no condvar)
static void pending_get_dropper(void* arg) {
    g_diag_reply_dropper_calls++;
    struct zpico_session* s = _zpico_unpack_session(arg);
    int slot = _zpico_unpack_slot(arg);
    get_reply_ctx_t* rctx = &s->pending_gets[slot].ctx;
    __atomic_store_n(&rctx->done, true, __ATOMIC_SEQ_CST);
    _zpico_notify_spin(s);
    if (s->reply_waker) {
        s->reply_waker((int32_t)(s - g_sessions), slot);
    }
}

void zpico_get_diag_counters(uint32_t out[18]) {
    out[0] = g_diag_get_start_calls;
    out[1] = g_diag_get_check_calls;
    out[2] = g_diag_get_check_returns_data;
    out[3] = g_diag_reply_handler_calls;
    out[4] = g_diag_reply_dropper_calls;
    out[5] = g_diag_reply_not_ok;
    out[6] = g_diag_reply_already_received;
    out[7] = g_diag_reply_received_set;
    out[8] = g_diag_reply_too_big;
    out[9] = g_diag_reply_to_slice_fail;
    out[10] = g_diag_gck_invalid_arg;
    out[11] = g_diag_gck_not_in_use;
    out[12] = g_diag_gck_too_big;
    out[13] = g_diag_gck_timeout;
    out[14] = g_diag_gck_pending;
    out[15] = g_diag_start_ctx_addr;
    out[16] = g_diag_handler_ctx_addr;
    out[17] = g_diag_check_ctx_addr;
}

int32_t zpico_get_start(zpico_session_t* session, const char* keyexpr, const uint8_t* payload,
                        size_t payload_len, uint32_t timeout_ms) {
    return zpico_get_start_with_attachment(session, keyexpr, payload, payload_len, NULL, 0,
                                           timeout_ms);
}

/* Issue 0153 — attachment-carrying variant. rmw_zenoh_cpp's service server
 * REQUIRES the rmw attachment (sequence_number + source_timestamp + gid) on
 * the query: `rmw_service_server_is_available`'s take path
 * (service_take_request) deserializes it and errors the whole take when it
 * is absent, so a nano-ros client request without one reaches the ROS 2
 * server and then dies inside rcl ("service failed to take request") — the
 * client only ever sees Transport(Timeout). nano<->nano services tolerate a
 * missing attachment, which is why this stayed invisible in-tree. */
int32_t zpico_get_start_with_attachment(zpico_session_t* session, const char* keyexpr,
                                        const uint8_t* payload, size_t payload_len,
                                        const uint8_t* attachment, size_t attachment_len,
                                        uint32_t timeout_ms) {
    struct zpico_session* s = (struct zpico_session*)session;
    g_diag_get_start_calls++;
    if (!s->session_open) {
        return ZPICO_ERR_SESSION;
    }

    // Find a free slot, cleaning up zombie slots first.
    // A zombie slot has in_use=true but ctx.done=true, meaning the reply was
    // delivered by get_check but the dropper hadn't fired yet at that time.
    // Now that the dropper has fired (done=true), the slot can be reclaimed.
    int32_t slot = -1;
    for (int32_t i = 0; i < ZPICO_MAX_PENDING_GETS; i++) {
        if (s->pending_gets[i].in_use && s->pending_gets[i].ctx.done) {
            s->pending_gets[i].in_use = false;
        }
        if (!s->pending_gets[i].in_use) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        return ZPICO_ERR_FULL;
    }

    // Initialize slot context
    pending_get_slot_t* ps = &s->pending_gets[slot];
    ps->ctx.len = 0;
    ps->ctx.received = false;
    ps->ctx.done = false;
    ps->ctx.reply_count = 0;
    ps->in_use = true;

    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str(&ke, keyexpr);

    z_get_options_t opts;
    z_get_options_default(&opts);
#if defined(ZPICO_TX_BATCH) && ZPICO_TX_BATCH == 1
    /* phase-279 (#145) — service/query latency guard: gets bypass the batch
     * (express) so request RTT gains no spin-period latency under batching. */
    opts.is_express = true;
#endif
    opts.target = Z_QUERY_TARGET_ALL;
    opts.timeout_ms = (uint64_t)timeout_ms;
    // Use NONE consolidation so the reply callback fires immediately on each
    // partial reply, rather than AUTO (which becomes LATEST and buffers replies
    // until the ReplyFinal arrives). With LATEST, a race between partial and
    // final delivery could leave received=false when get_check is polled.
    opts.consolidation.mode = Z_CONSOLIDATION_MODE_NONE;

    // Declare payload_bytes in the same scope as opts and z_get() so it stays
    // alive until z_get() consumes it (z_move takes the address).
    z_owned_bytes_t payload_bytes;
    if (payload != NULL && payload_len > 0) {
        z_bytes_copy_from_buf(&payload_bytes, payload, payload_len);
        opts.payload = z_move(payload_bytes);
    }
    // Same lifetime rule for the rmw attachment (issue 0153).
    z_owned_bytes_t attachment_bytes;
    if (attachment != NULL && attachment_len > 0) {
        z_bytes_copy_from_buf(&attachment_bytes, attachment, attachment_len);
        opts.attachment = z_move(attachment_bytes);
    }

    z_owned_closure_reply_t callback;
    z_closure(&callback, pending_get_reply_handler, pending_get_dropper, _zpico_pack_ctx(s, slot));
    /* Same aliasing-defeat trick. */
    g_diag_start_ctx_addr = (uint32_t)(uintptr_t)&ps->ctx;

    z_result_t zret =
        z_get(z_session_loan(&s->session), z_view_keyexpr_loan(&ke), "", z_move(callback), &opts);
    if (zret < 0) {
        ps->in_use = false;
        return ZPICO_ERR_GENERIC;
    }

    return slot;
}

/* Non-blocking liveliness query.
 *
 * Used by `Client::wait_for_service` (and the action-client equivalent) to
 * implement rclcpp-style server discovery: issue a `z_liveliness_get` against
 * the server's wildcarded liveliness keyexpr, then poll `zpico_liveliness_get_check`
 * until either at least one matching token reports back or the dropper fires
 * empty-handed.
 *
 * Reuses the same `s->pending_gets` slot pool as `zpico_get_start` — a slot is
 * just a (received_flag, dropper_done_flag, payload_buf) triple, agnostic to
 * whether the caller will read the payload. The reply handler still copies
 * the (typically empty) liveliness token bytes into the slot's buffer; we
 * never read them. Only `received` matters.
 *
 * Returns the slot handle on success, ZPICO_ERR_* on failure.
 */
int32_t zpico_liveliness_get_start(zpico_session_t* session, const char* keyexpr,
                                   uint32_t timeout_ms) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (!s->session_open) {
        return ZPICO_ERR_SESSION;
    }

    int32_t slot = -1;
    for (int32_t i = 0; i < ZPICO_MAX_PENDING_GETS; i++) {
        if (s->pending_gets[i].in_use && s->pending_gets[i].ctx.done) {
            s->pending_gets[i].in_use = false;
        }
        if (!s->pending_gets[i].in_use) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        return ZPICO_ERR_FULL;
    }

    pending_get_slot_t* ps = &s->pending_gets[slot];
    ps->ctx.len = 0;
    ps->ctx.received = false;
    ps->ctx.done = false;
    ps->ctx.reply_count = 0;
    ps->ctx.collect = false;
    ps->ctx.entry_count = 0;
    ps->in_use = true;

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        ps->in_use = false;
        return ZPICO_ERR_KEYEXPR;
    }

    z_liveliness_get_options_t opts;
    z_liveliness_get_options_default(&opts);
    opts.timeout_ms = (uint64_t)timeout_ms;

    z_owned_closure_reply_t callback;
    z_closure(&callback, pending_get_reply_handler, pending_get_dropper, _zpico_pack_ctx(s, slot));

    z_result_t zret = z_liveliness_get(z_session_loan(&s->session), z_view_keyexpr_loan(&ke),
                                       z_move(callback), &opts);
    if (zret < 0) {
        ps->in_use = false;
        return ZPICO_ERR_GENERIC;
    }
    return slot;
}

/* phase-381 W1 — a liveliness query that KEEPS its replies' keyexprs.
 *
 * `zpico_liveliness_get_start` answers "does anything match". Reading the ROS
 * graph needs "WHAT matched": a liveliness reply's information is entirely in
 * its keyexpr (`@ros2_lv/<domain>/<zid>/<nid>/<eid>/…/<namespace>/<node>`), and
 * the handler used to discard it.
 *
 * Same slot pool, same start/poll shape, same dropper. The only difference is
 * that the handler appends each reply's keyexpr into the slot's buffer instead
 * of copying a payload into it — that buffer is dead weight on a liveliness
 * query, whose token payload is empty, so enumeration adds no storage.
 *
 * Poll with `zpico_liveliness_get_check` exactly as for the counting form, then
 * read entries with `zpico_liveliness_entry_count` / `zpico_liveliness_entry`.
 *
 * Returns the slot handle on success, ZPICO_ERR_* on failure.
 */
int32_t zpico_liveliness_collect_start(zpico_session_t* session, const char* keyexpr,
                                       uint32_t timeout_ms) {
    /* Deliberately NOT `zpico_liveliness_get_start` + set the flag.
     *
     * That was the first shape and it LOST EVERY REPLY. `collect` has to be
     * true before the query goes on the wire: zenoh-pico's RX task runs the
     * reply handler as soon as replies arrive, and against a local router with
     * latched liveliness tokens they arrive within microseconds — reliably
     * inside the window between `z_liveliness_get` returning and the caller
     * setting the flag. Replies in that window take the SINGLE-RESPONSE path,
     * which sets `received`, copies a payload nobody reads, and leaves
     * `entry_count` at zero.
     *
     * Measured against a live `rmw_zenoh_cpp` talker (issue 0903): the entity
     * enumeration returned zero every time while node enumeration worked, and
     * widening the wildcard to `@ros2_lv/<domain>/**` changed nothing — because
     * the pattern was never the problem.
     *
     * The comment this replaces called the race "worst case the first reply is
     * counted but not stored". That was wrong in two ways: it is not the first
     * reply, it is all of them, and a multi-threaded RX task is the DEFAULT on
     * POSIX rather than an edge case. */
    struct zpico_session* s = (struct zpico_session*)session;
    if (!s->session_open) {
        return ZPICO_ERR_SESSION;
    }

    int32_t slot = -1;
    for (int32_t i = 0; i < ZPICO_MAX_PENDING_GETS; i++) {
        if (s->pending_gets[i].in_use && s->pending_gets[i].ctx.done) {
            s->pending_gets[i].in_use = false;
        }
        if (!s->pending_gets[i].in_use) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        return ZPICO_ERR_FULL;
    }

    pending_get_slot_t* ps = &s->pending_gets[slot];
    ps->ctx.len = 0;
    ps->ctx.received = false;
    ps->ctx.done = false;
    ps->ctx.reply_count = 0;
    ps->ctx.entry_count = 0;
    ps->ctx.collect = true; /* BEFORE the query, which is the whole point */
    ps->in_use = true;

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        ps->in_use = false;
        return ZPICO_ERR_KEYEXPR;
    }

    z_liveliness_get_options_t opts;
    z_liveliness_get_options_default(&opts);
    opts.timeout_ms = (uint64_t)timeout_ms;

    z_owned_closure_reply_t callback;
    z_closure(&callback, pending_get_reply_handler, pending_get_dropper, _zpico_pack_ctx(s, slot));

    z_result_t zret = z_liveliness_get(z_session_loan(&s->session), z_view_keyexpr_loan(&ke),
                                       z_move(callback), &opts);
    if (zret < 0) {
        ps->in_use = false;
        return ZPICO_ERR_GENERIC;
    }
    return slot;
}

/* How many keyexprs this slot STORED.
 *
 * Distinct from `zpico_liveliness_get_count`, which reports how many replies
 * ARRIVED. The two differ exactly when the buffer could not hold an entry, so
 * a caller that wants to know whether the enumeration is complete compares
 * them — this is the truncation signal, and it is why entries are dropped whole
 * rather than truncated.
 *
 * Returns ZPICO_ERR_INVALID for a bad handle or a slot not in use.
 */
/* Has the collecting sweep FINISHED — dropper fired, no more replies coming?
 *
 * `zpico_liveliness_get_check` cannot answer this. It returns 1 as soon as ONE
 * reply has arrived, which is the right signal for "is the service up?" (its
 * only caller before phase-381) and the wrong one for enumeration: a caller
 * that restarts its query on that truncates every sweep to whatever landed in
 * the poll window, and never sees the rest of the graph.
 *
 * Measured against a live `rmw_zenoh_cpp` talker: collecting until `get_check`
 * went non-zero captured 2 tokens of the dozen it declares. This is the
 * distinction that fixes that.
 *
 * Returns 1 finished, 0 still collecting, ZPICO_ERR_INVALID for a bad handle.
 */
int32_t zpico_liveliness_collect_done(zpico_session_t* session, int32_t handle) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (handle < 0 || handle >= ZPICO_MAX_PENDING_GETS) {
        return ZPICO_ERR_INVALID;
    }
    pending_get_slot_t* ps = &s->pending_gets[handle];
    if (!ps->in_use) {
        return ZPICO_ERR_INVALID;
    }
    return __atomic_load_n(&ps->ctx.done, __ATOMIC_SEQ_CST) ? 1 : 0;
}

int32_t zpico_liveliness_entry_count(zpico_session_t* session, int32_t handle) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (handle < 0 || handle >= ZPICO_MAX_PENDING_GETS) {
        return ZPICO_ERR_INVALID;
    }
    if (!s->pending_gets[handle].in_use) {
        return ZPICO_ERR_INVALID;
    }
    return (int32_t)s->pending_gets[handle].ctx.entry_count;
}

/* Copy stored keyexpr `index` into `out`, NUL-terminated.
 *
 * Returns the number of bytes written excluding the NUL, ZPICO_ERR_INVALID for
 * a bad handle/index, or ZPICO_ERR_BUFFER when `cap` cannot hold the entry plus
 * its NUL. A short buffer is an ERROR rather than a truncation for the same
 * reason the handler drops rather than truncates: half a keyexpr names a
 * different, plausible entity.
 */
/* phase-381 / issue 0903 — graph cache: find `key` in the NUL-separated run.
 *
 * Returns the offset of the entry, or `SIZE_MAX` when absent. Split out so the
 * PUT and DELETE arms of the sample handler share one notion of identity, and
 * so it is testable without a session. */
static size_t graph_cache_find(const graph_cache_t* c, const char* key, size_t klen) {
    size_t off = 0;
    while (off < c->len) {
        const char* e = (const char*)c->buf + off;
        size_t n = strlen(e);
        if (n == klen && memcmp(e, key, klen) == 0) {
            return off;
        }
        off += n + 1;
    }
    return SIZE_MAX;
}

/* Sample handler for the liveliness subscriber.
 *
 * A liveliness sample carries its meaning in the KEYEXPR and its KIND: PUT is
 * "this token exists", DELETE is "it is gone". The payload is empty, as it is
 * for the query form. */
static void graph_cache_sample_handler(z_loaned_sample_t* sample, void* arg) {
    struct zpico_session* s = (struct zpico_session*)arg;
    if (s == NULL || !s->graph_cache.active) {
        return;
    }
    z_view_string_t ks;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &ks);
    const char* key = z_string_data(z_view_string_loan(&ks));
    size_t klen = z_string_len(z_view_string_loan(&ks));
    if (key == NULL || klen == 0) {
        return;
    }
    graph_cache_t* c = &s->graph_cache;
#if Z_FEATURE_MULTI_THREAD == 1
    _z_mutex_lock(&c->mutex);
#endif
    size_t at = graph_cache_find(c, key, klen);
    if (z_sample_kind(sample) == Z_SAMPLE_KIND_DELETE) {
        if (at != SIZE_MAX) {
            /* Close the gap so the run stays dense and `entry_count` keeps
             * indexing it. */
            size_t n = strlen((const char*)c->buf + at) + 1;
            memmove(c->buf + at, c->buf + at + n, c->len - at - n);
            c->len -= n;
            c->entry_count--;
        }
    } else if (at == SIZE_MAX) {
        if (c->len + klen + 1 <= sizeof(c->buf)) {
            memcpy(c->buf + c->len, key, klen);
            c->buf[c->len + klen] = '\0';
            c->len += klen + 1;
            c->entry_count++;
        } else {
            c->dropped++;
        }
    }
#if Z_FEATURE_MULTI_THREAD == 1
    _z_mutex_unlock(&c->mutex);
#endif
}

/* Start the standing graph cache on `keyexpr`.
 *
 * `history = true` is the whole point: the subscriber is delivered the tokens
 * that ALREADY exist, then every later change. Without it the cache would only
 * ever see nodes that appear after us, which is the same blind spot the query
 * form had for a different reason.
 *
 * Idempotent — a second call while active is a no-op, so every graph entry
 * point can call it without coordinating.
 */
int32_t zpico_graph_cache_start(zpico_session_t* session, const char* keyexpr) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (s == NULL || keyexpr == NULL) {
        return ZPICO_ERR_INVALID;
    }
    if (s->graph_cache.active) {
        return 0;
    }
    graph_cache_t* c = &s->graph_cache;
    c->len = 0;
    c->entry_count = 0;
    c->dropped = 0;
#if Z_FEATURE_MULTI_THREAD == 1
    if (_z_mutex_init(&c->mutex) != _Z_RES_OK) {
        return ZPICO_ERR_INVALID;
    }
#endif
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) != _Z_RES_OK) {
#if Z_FEATURE_MULTI_THREAD == 1
        _z_mutex_drop(&c->mutex);
#endif
        return ZPICO_ERR_INVALID;
    }
    z_owned_closure_sample_t closure;
    z_closure_sample(&closure, graph_cache_sample_handler, NULL, s);
    z_liveliness_subscriber_options_t opts;
    z_liveliness_subscriber_options_default(&opts);
    opts.history = true;
    /* Set active BEFORE declaring: the handler can fire from the RX path the
     * moment the declaration lands, and a flag set afterwards drops exactly the
     * history burst this call exists to collect. That ordering was defect 4 in
     * the query form (issue 0903); it is not repeated here. */
    c->active = true;
    z_result_t ret = z_liveliness_declare_subscriber(z_session_loan(&s->session), &c->sub,
                                                     z_view_keyexpr_loan(&ke),
                                                     z_closure_sample_move(&closure), &opts);
    if (ret != _Z_RES_OK) {
        c->active = false;
#if Z_FEATURE_MULTI_THREAD == 1
        _z_mutex_drop(&c->mutex);
#endif
        return ZPICO_ERR_INVALID;
    }
    return 0;
}

/* Stop the cache and release the subscriber. Idempotent. */
int32_t zpico_graph_cache_stop(zpico_session_t* session) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (s == NULL) {
        return ZPICO_ERR_INVALID;
    }
    if (!s->graph_cache.active) {
        return 0;
    }
    s->graph_cache.active = false;
    z_undeclare_subscriber(z_subscriber_move(&s->graph_cache.sub));
#if Z_FEATURE_MULTI_THREAD == 1
    _z_mutex_drop(&s->graph_cache.mutex);
#endif
    return 0;
}

int32_t zpico_entry_at(const uint8_t* buf, size_t len, uint32_t count, uint32_t index, char* out,
                       size_t cap);

/* How many tokens the cache currently holds, and how many did not fit.
 *
 * `out_dropped` is reported beside the count rather than folded into it,
 * because a graph answer that silently omits entries is the plausible-wrong-
 * answer failure this path exists to avoid.
 */
int32_t zpico_graph_entry_count(zpico_session_t* session, uint32_t* out_dropped) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (s == NULL || !s->graph_cache.active) {
        return ZPICO_ERR_INVALID;
    }
    graph_cache_t* c = &s->graph_cache;
#if Z_FEATURE_MULTI_THREAD == 1
    _z_mutex_lock(&c->mutex);
#endif
    uint32_t n = c->entry_count;
    if (out_dropped != NULL) {
        *out_dropped = c->dropped;
    }
#if Z_FEATURE_MULTI_THREAD == 1
    _z_mutex_unlock(&c->mutex);
#endif
    return (int32_t)n;
}

/* Copy cached keyexpr `index` into `out`, NUL-terminated.
 *
 * ONE entry per call, under the lock, rather than a bulk snapshot: the cache is
 * sized for a real ROS graph (tens of KB) and the callers are embedded, where a
 * buffer that size cannot live on the stack. It also keeps the capacity a fact
 * about the C side alone — a bulk copy forced the Rust caller to know
 * `ZPICO_GRAPH_CACHE_SIZE`, and two constants that must agree eventually do not.
 *
 * A token may appear or vanish between calls; that is inherent to enumerating a
 * LIVE graph and is why callers poll to convergence.
 */
int32_t zpico_graph_entry_at(zpico_session_t* session, uint32_t index, char* out, size_t cap) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (s == NULL || out == NULL || !s->graph_cache.active) {
        return ZPICO_ERR_INVALID;
    }
    graph_cache_t* c = &s->graph_cache;
#if Z_FEATURE_MULTI_THREAD == 1
    _z_mutex_lock(&c->mutex);
#endif
    int32_t ret = zpico_entry_at(c->buf, c->len, c->entry_count, index, out, cap);
#if Z_FEATURE_MULTI_THREAD == 1
    _z_mutex_unlock(&c->mutex);
#endif
    return ret;
}

int32_t zpico_entry_at(const uint8_t* buf, size_t len, uint32_t count, uint32_t index, char* out,
                       size_t cap) {
    if (buf == NULL || out == NULL || index >= count) {
        return ZPICO_ERR_INVALID;
    }
    /* Walk the NUL-separated run. `count` bounds the loop, so this cannot run
     * off the end of what the handler wrote. */
    const char* p = (const char*)buf;
    const char* end = (const char*)buf + len;
    for (uint32_t i = 0; i < index; i++) {
        while (p < end && *p != '\0') {
            p++;
        }
        if (p < end) {
            p++;
        }
    }
    size_t n = 0;
    while (p + n < end && p[n] != '\0') {
        n++;
    }
    if (n + 1 > cap) {
        return ZPICO_ERR_BUFFER;
    }
    memcpy(out, p, n);
    out[n] = '\0';
    return (int32_t)n;
}

int32_t zpico_liveliness_entry(zpico_session_t* session, int32_t handle, uint32_t index, char* out,
                               size_t cap) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (handle < 0 || handle >= ZPICO_MAX_PENDING_GETS) {
        return ZPICO_ERR_INVALID;
    }
    pending_get_slot_t* ps = &s->pending_gets[handle];
    if (!ps->in_use) {
        return ZPICO_ERR_INVALID;
    }
    return zpico_entry_at(ps->ctx.buf, ps->ctx.len, ps->ctx.entry_count, index, out, cap);
}

/* Check status of a pending liveliness query.
 *
 * Unlike `zpico_get_check`, the caller doesn't care about the reply payload —
 * just whether *any* matching liveliness token responded. Liveliness tokens
 * carry an empty (0-byte) payload, which `zpico_get_check` would otherwise
 * report as "still pending" (its return value `0` is overloaded).
 *
 * Returns:
 *   1 — at least one token reply seen, server is discoverable.
 *   0 — query still in flight, no replies yet.
 *  -9 — dropper fired with no replies (timeout, no matching server).
 *   ZPICO_ERR_INVALID — handle out of range or slot not in use.
 */
/* Phase 108.C.zenoh.4-followup — count of liveliness-token replies
 * received on this slot. Returns 0 while the query is still in
 * flight, ZPICO_ERR_INVALID for bad handles. After the dropper has
 * fired (i.e. `zpico_liveliness_get_check` would return 1 or -9),
 * the count is final and accurate up to the timeout. Used by the
 * subscriber-side `LivelinessChanged` bridge to report
 * `alive_count > 1` when more than one publisher matches the
 * wildcard liveliness keyexpr.
 *
 * The count is left intact on read; only `zpico_liveliness_get_check`
 * releases the slot, so callers should pair `count → check`.
 */
int32_t zpico_liveliness_get_count(zpico_session_t* session, int32_t handle) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (handle < 0 || handle >= ZPICO_MAX_PENDING_GETS) {
        return ZPICO_ERR_INVALID;
    }
    pending_get_slot_t* ps = &s->pending_gets[handle];
    if (!ps->in_use) {
        return ZPICO_ERR_INVALID;
    }
    /* Cap to int32 max — wildcards in practice match a handful of
     * tokens, never billions. */
    uint32_t c = ps->ctx.reply_count;
    return c > (uint32_t)INT32_MAX ? INT32_MAX : (int32_t)c;
}

int32_t zpico_liveliness_get_check(zpico_session_t* session, int32_t handle) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (handle < 0 || handle >= ZPICO_MAX_PENDING_GETS) {
        return ZPICO_ERR_INVALID;
    }

    pending_get_slot_t* ps = &s->pending_gets[handle];
    if (!ps->in_use) {
        return ZPICO_ERR_INVALID;
    }
    /* Same aliasing-defeat trick as `zpico_get_check`; see comment
     * in `pending_get_reply_handler`. */
    g_diag_check_ctx_addr = (uint32_t)(uintptr_t)&ps->ctx;

    if (__atomic_load_n(&ps->ctx.received, __ATOMIC_SEQ_CST)) {
        if (__atomic_load_n(&ps->ctx.done, __ATOMIC_SEQ_CST)) {
            ps->in_use = false;
        }
        return 1;
    }
    if (__atomic_load_n(&ps->ctx.done, __ATOMIC_SEQ_CST)) {
        ps->in_use = false;
        return -9; /* ZPICO_ERR_TIMEOUT */
    }
    return 0;
}

int32_t zpico_get_check(zpico_session_t* session, int32_t handle, uint8_t* reply_buf,
                        size_t reply_buf_size) {
    struct zpico_session* s = (struct zpico_session*)session;
    g_diag_get_check_calls++;
    if (handle < 0 || handle >= ZPICO_MAX_PENDING_GETS) {
        g_diag_gck_invalid_arg++;
        return ZPICO_ERR_INVALID;
    }

    pending_get_slot_t* ps = &s->pending_gets[handle];
    if (!ps->in_use) {
        g_diag_gck_not_in_use++;
        return ZPICO_ERR_INVALID;
    }

    /* Same aliasing-defeat trick as in `pending_get_reply_handler`. */
    g_diag_check_ctx_addr = (uint32_t)(uintptr_t)&ps->ctx;
    bool received_snap = __atomic_load_n(&ps->ctx.received, __ATOMIC_SEQ_CST);
    bool done_snap = __atomic_load_n(&ps->ctx.done, __ATOMIC_SEQ_CST);
    if (received_snap) {
        g_diag_get_check_returns_data++;
        // Reply arrived — copy data
        if (ps->ctx.len > reply_buf_size) {
            g_diag_gck_too_big++;
            // Only release slot if dropper has also fired
            if (done_snap) {
                ps->in_use = false;
            }
            return ZPICO_ERR_FULL;
        }
        memcpy(reply_buf, ps->ctx.buf, ps->ctx.len);
        int32_t len = (int32_t)ps->ctx.len;
        // Only release slot if dropper has also fired; otherwise the old
        // z_get's dropper callback still references this slot and would
        // corrupt it if the slot were reused before the dropper fires.
        if (done_snap) {
            ps->in_use = false;
        }
        return len;
    }

    if (done_snap) {
        g_diag_gck_timeout++;
        // Dropper fired without a reply — timeout
        ps->in_use = false;
        return -9; // ZPICO_ERR_TIMEOUT
    }

    g_diag_gck_pending++;
    // Not yet — still pending
    return 0;
}

void zpico_set_reply_waker(zpico_session_t* session, zpico_waker_fn fn) {
    struct zpico_session* s = (struct zpico_session*)session;
    s->reply_waker = fn;
}

// ============================================================================
// Query Reply Implementation (for service servers)
// ============================================================================

/* issue 0902 — release a reply slot on EVERY exit, not only the successful one.
 *
 * `query_handler` clones the query into the slot before the callback runs, and
 * the slot is freed in three places: the session-open `memset`, queryable
 * teardown, and a successful reply. So each error return below used to strand
 * its slot for the life of the queryable, and four exhausted slots make the
 * server answer nothing for ever.
 *
 * Freeing here is correct because NO CALLER CAN RETRY a seq -- checked, not
 * assumed: every `send_response` site either `?`-propagates or maps to
 * `ServiceReplyFailed` (`action_core.rs:373,410,822,895,1014,1037`), and the
 * deferred-reply flush has already `swap_remove`d the pending entry before it
 * looks at the result (`action_core.rs:688,695`). A retry has nothing to retry
 * WITH. Holding the slot would preserve a query nobody will ever answer. */
static void _zpico_release_reply_slot(struct zpico_session* s, int32_t handle, int64_t seq) {
    if (seq < 0 || seq >= ZPICO_MAX_PENDING_REPLIES) {
        return;
    }
    if (!s->stored_query_valid[handle][seq]) {
        return;
    }
    z_query_drop(z_query_move(&s->stored_queries[handle][seq]));
    s->stored_query_valid[handle][seq] = false;
}

int32_t zpico_query_reply(zpico_session_t* session, int32_t queryable_handle, int64_t reply_seq,
                          const char* keyexpr, const uint8_t* data, size_t len,
                          const uint8_t* attachment, size_t attachment_len) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (queryable_handle < 0 || queryable_handle >= ZPICO_MAX_QUERYABLES) {
        return ZPICO_ERR_INVALID;
    }
    // Phase 237 — `reply_seq` selects the cloned query captured by
    // `query_handler` (the slot index it recorded). The reply may arrive long
    // after the query callback returned (deferred get_result).
    if (reply_seq < 0 || reply_seq >= ZPICO_MAX_PENDING_REPLIES ||
        !s->stored_query_valid[queryable_handle][reply_seq]) {
        return ZPICO_ERR_INVALID;
    }
    z_owned_query_t* stored_query = &s->stored_queries[queryable_handle][reply_seq];

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        _zpico_release_reply_slot(s, queryable_handle, reply_seq);
        return ZPICO_ERR_KEYEXPR;
    }

    // Create payload
    z_owned_bytes_t payload;
    if (z_bytes_copy_from_buf(&payload, data, len) < 0) {
        _zpico_release_reply_slot(s, queryable_handle, reply_seq);
        return ZPICO_ERR_GENERIC;
    }

    // Create reply options with attachment
    z_query_reply_options_t options;
    z_query_reply_options_default(&options);
#if defined(ZPICO_TX_BATCH) && ZPICO_TX_BATCH == 1
    /* phase-279 (#145) — replies bypass the batch (express): service RTT must
     * not wait for the next spin flush when batching is on. */
    options.is_express = true;
#endif

    z_owned_bytes_t attachment_bytes;
    if (attachment != NULL && attachment_len > 0) {
        // Use explicitly provided attachment
        if (z_bytes_copy_from_buf(&attachment_bytes, attachment, attachment_len) < 0) {
            z_bytes_drop(z_bytes_move(&payload));
            _zpico_release_reply_slot(s, queryable_handle, reply_seq);
            return ZPICO_ERR_GENERIC;
        }
        options.attachment = z_bytes_move(&attachment_bytes);
    } else {
        // Echo back the original query's attachment (required by rmw_zenoh_cpp
        // which expects sequence_number, source_timestamp, gid in the reply).
        const z_loaned_bytes_t* query_att = z_query_attachment(z_query_loan(stored_query));
        if (query_att != NULL && z_bytes_len(query_att) > 0) {
            z_owned_slice_t att_slice;
            if (z_bytes_to_slice(query_att, &att_slice) == 0) {
                if (z_bytes_copy_from_buf(&attachment_bytes, z_slice_data(z_slice_loan(&att_slice)),
                                          z_slice_len(z_slice_loan(&att_slice))) == 0) {
                    options.attachment = z_bytes_move(&attachment_bytes);
                }
                z_slice_drop(z_slice_move(&att_slice));
            }
        }
    }

    // Reply using the cloned query held in this reply slot.
    if (z_query_reply(z_query_loan(stored_query), z_view_keyexpr_loan(&ke), z_bytes_move(&payload),
                      &options) < 0) {
        _zpico_release_reply_slot(s, queryable_handle, reply_seq);
        return ZPICO_ERR_GENERIC;
    }

    // Drop the cloned query + free the slot after reply.
    _zpico_release_reply_slot(s, queryable_handle, reply_seq);

    return ZPICO_OK;
}

// Phase 237 — return the reply-slot index allocated by the most recent
// `query_handler` for this queryable (the deferred-reply seq). Must be called
// from inside the synchronous query callback; -1 if the reply table was full.
int64_t zpico_queryable_take_reply_seq(zpico_session_t* session, int32_t queryable_handle) {
    struct zpico_session* s = (struct zpico_session*)session;
    if (queryable_handle < 0 || queryable_handle >= ZPICO_MAX_QUERYABLES) {
        return -1;
    }
    /* issue 0902 — a real TAKE. The name always said so; the function used to
     * be a pure read, which left the caller's decision invisible to
     * `query_handler` and made a declined query indistinguishable from a
     * deferred one. Clearing it is the claim: whatever is still set when the
     * callback returns belongs to nobody. */
    int64_t seq = s->last_reply_seq[queryable_handle];
    s->last_reply_seq[queryable_handle] = -1;
    return seq;
}

// ============================================================================
// Clock helpers (for FFI reentrancy guard timeout decomposition)
// ============================================================================

// Opaque buffer size: z_clock_t is uint64_t (8 bytes) on bare-metal,
// struct { uint64_t tv_sec; uint64_t tv_nsec; } (16 bytes) on ThreadX/POSIX.
_Static_assert(sizeof(z_clock_t) <= 16, "z_clock_t must fit in 16 bytes");

void zpico_clock_start(uint8_t* clock_buf) {
    z_clock_t now = z_clock_now();
    memcpy(clock_buf, &now, sizeof(z_clock_t));
}

unsigned long zpico_clock_elapsed_ms_since(uint8_t* clock_buf) {
    return z_clock_elapsed_ms((z_clock_t*)clock_buf);
}
