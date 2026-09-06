/*
 * Phase 121.6.threadx-c — ThreadX implementation of the canonical
 * timer ABI declared in `<nros/platform_timer.h>`.
 *
 * Backed by `tx_timer_create` / `tx_timer_change` / `tx_timer_deactivate`
 * / `tx_timer_delete`. The kernel timer callback runs in ThreadX's
 * timer-task context.
 */

#include <nros/platform_timer.h>

#include <tx_api.h>

#include <stddef.h>
#include <stdint.h>

typedef struct {
    TX_TIMER                       kernel;
    nros_platform_timer_callback_t callback;
    void                          *user_data;
    volatile int                   fired;
    volatile int                   cancelled;
    int                             periodic;
} nros_threadx_timer_t;

/* tx_timer_create's expiration_input is a `ULONG` — only 32 bits on
 * 64-bit ThreadX ports (linux/gnu typedef'd `ULONG = unsigned int`).
 * Pointer values don't fit. Maintain a small static registry of
 * active timers indexed by a 32-bit slot id; the trampoline looks
 * up the wrapper there. */
#ifndef NROS_THREADX_MAX_TIMERS
#  define NROS_THREADX_MAX_TIMERS 32
#endif
/* Issue 1131 — the registry is how a `ULONG` expiration_input finds its
 * wrapper, so a slot is the ONLY way a timer can exist on this port. At 0
 * `registry_claim` can never succeed, so `nros_platform_timer_create` returns
 * NULL for every timer the image asks for, forever — an image with no timers
 * asks for none and pays nothing extra, so 0 saves 32 pointers and costs the
 * whole timer ABI. Below the `#ifndef`, never above: an undefined identifier
 * reads as 0 in `#if`, so a guard above its own default fires always (1167). */
#if NROS_THREADX_MAX_TIMERS < 1
#error "NROS_THREADX_MAX_TIMERS must be >= 1: it sizes a C array (issue 1015)"
#endif
static nros_threadx_timer_t *s_timer_registry[NROS_THREADX_MAX_TIMERS];

/* Heap allocated by the application; expose via a setter so the
 * port stays decoupled from any specific pool. */
static TX_BYTE_POOL *s_timer_pool = NULL;

void nros_platform_threadx_set_timer_pool(void *pool) {
    s_timer_pool = (TX_BYTE_POOL *) pool;
}

static int registry_claim(nros_threadx_timer_t *t) {
    for (int i = 0; i < NROS_THREADX_MAX_TIMERS; ++i) {
        if (s_timer_registry[i] == NULL) {
            s_timer_registry[i] = t;
            return i;
        }
    }
    return -1;
}

static void registry_release(int id) {
    if (id >= 0 && id < NROS_THREADX_MAX_TIMERS) {
        s_timer_registry[id] = NULL;
    }
}

static void timer_trampoline(ULONG id) {
    if (id >= (ULONG) NROS_THREADX_MAX_TIMERS) return;
    nros_threadx_timer_t *t = s_timer_registry[id];
    if (t == NULL) return;
    if (t->cancelled) return;
    t->fired = 1;
    if (t->callback != NULL) {
        t->callback(t->user_data);
    }
}

static void *create_timer(uint32_t value_us, int periodic,
                          nros_platform_timer_callback_t callback,
                          void *user_data) {
    if (callback == NULL || value_us == 0 || s_timer_pool == NULL) return NULL;

    void *raw = NULL;
    if (tx_byte_allocate(s_timer_pool, &raw,
                         (ULONG) sizeof(nros_threadx_timer_t),
                         TX_NO_WAIT) != TX_SUCCESS) {
        return NULL;
    }
    nros_threadx_timer_t *t = (nros_threadx_timer_t *) raw;
    t->callback  = callback;
    t->user_data = user_data;
    t->fired     = 0;
    t->cancelled = 0;
    t->periodic  = periodic;

    int slot = registry_claim(t);
    if (slot < 0) {
        (void) tx_byte_release(raw);
        return NULL;
    }

    /* Convert microseconds to ThreadX ticks. configTICK_RATE is the
     * platform's TX_TIMER_TICKS_PER_SECOND. */
#ifndef TX_TIMER_TICKS_PER_SECOND
#  define TX_TIMER_TICKS_PER_SECOND 100u
#endif
    ULONG ticks = (ULONG) ((((uint64_t) value_us) * TX_TIMER_TICKS_PER_SECOND
                            + 999999ull) / 1000000ull);
    if (ticks == 0) ticks = 1;

    if (tx_timer_create(&t->kernel,
                        (CHAR *) "nros_timer",
                        timer_trampoline,
                        (ULONG) slot,
                        ticks,
                        periodic ? ticks : 0,
                        TX_AUTO_ACTIVATE) != TX_SUCCESS) {
        registry_release(slot);
        (void) tx_byte_release(raw);
        return NULL;
    }
    /* Stash slot id alongside the wrapper so destroy/cancel can
     * release. Reuse `fired`'s upper bits would be hacky — overload
     * the `periodic` field's high half instead since it's only 0/1.
     * Bitfield avoids the hack: store slot below. */
    t->periodic = periodic | (slot << 8);
    return (void *) t;
}

void *nros_platform_timer_create_periodic(uint32_t period_us,
                                          nros_platform_timer_callback_t callback,
                                          void *user_data) {
    return create_timer(period_us, /* periodic = */ 1, callback, user_data);
}

void *nros_platform_timer_create_oneshot(uint32_t timeout_us,
                                         nros_platform_timer_callback_t callback,
                                         void *user_data) {
    return create_timer(timeout_us, /* periodic = */ 0, callback, user_data);
}

void nros_platform_timer_destroy(void *handle) {
    if (handle == NULL) return;
    nros_threadx_timer_t *t = (nros_threadx_timer_t *) handle;
    int slot = t->periodic >> 8;
    t->cancelled = 1;
    (void) tx_timer_deactivate(&t->kernel);
    (void) tx_timer_delete(&t->kernel);
    registry_release(slot);
    (void) tx_byte_release(t);
}

int8_t nros_platform_timer_cancel(void *handle) {
    if (handle == NULL) return -1;
    nros_threadx_timer_t *t = (nros_threadx_timer_t *) handle;
    int prev_fired = t->fired;
    t->cancelled = 1;
    if (tx_timer_deactivate(&t->kernel) != TX_SUCCESS) return -1;
    return prev_fired == 0 ? 1 : 0;
}
