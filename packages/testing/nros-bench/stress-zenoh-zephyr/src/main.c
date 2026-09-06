/* phase-282 W2 (#145) — Zephyr streaming tx benchmark.
 *
 * Tight-loop publisher over the zpico C shim (the exact tx path every
 * nano-ros language front-end uses), bypassing the executor so the number
 * isolates the TRANSPORT tx path. Publishes STRESS_COUNT payloads of
 * STRESS_SIZE bytes as fast as the loop allows, then reports
 * `PUBLISH_DONE: sent=<n> elapsed_ms=<t>` — the same summary the native
 * `nros-bench/stress-zenoh` talker prints, so measurement procedures match.
 *
 * Payload layout mirrors stress-zenoh's build_payload() so the native
 * stress listener validates integrity:
 *   [0..4]  CDR header 00 01 00 00
 *   [4..8]  sequence number (u32 LE)
 *   [8..12] total size (u32 LE)
 *   [12..]  fill pattern (i & 0xFF)
 *
 * Keyexpr matches the listener's typed Int32 subscription on /stress_test
 * at domain 0 (Humble keyexpr format, TypeHashNotSupported segment).
 */

#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "zpico.h"

#ifndef STRESS_COUNT
#define STRESS_COUNT 5000
#endif
#ifndef STRESS_SIZE
#define STRESS_SIZE 64
#endif
/* Issue 1131 — `g_payload[STRESS_SIZE]` is a fixed array AND `build_payload()`
 * writes indices 0..11 (CDR header, seq, size) with no bounds test, so anything
 * under 12 is an out-of-bounds write on a static object, not a smaller payload.
 * 16 is the floor the CMake cache entry has always documented (`payload bytes
 * (>=16)`); the fill loop starts at 12, so below it the bench also measures a
 * payload the stress listener cannot validate. Below the `#ifndef`, never above
 * it: an undefined identifier reads as 0 in `#if` (issue 1167). */
#if STRESS_SIZE < 16
#error "STRESS_SIZE must be >= 16: build_payload() writes 12 header bytes (issue 1015)"
#endif
#ifndef STRESS_EXPRESS
#define STRESS_EXPRESS 0
#endif
#ifndef STRESS_INTERVAL_MS
#define STRESS_INTERVAL_MS 0
#endif

#define STRESS_KEYEXPR "0/stress_test/std_msgs::msg::dds_::Int32_/TypeHashNotSupported"

static uint8_t g_payload[STRESS_SIZE];

static void build_payload(uint32_t seq) {
    g_payload[0] = 0x00;
    g_payload[1] = 0x01;
    g_payload[2] = 0x00;
    g_payload[3] = 0x00;
    g_payload[4] = (uint8_t)(seq & 0xFF);
    g_payload[5] = (uint8_t)((seq >> 8) & 0xFF);
    g_payload[6] = (uint8_t)((seq >> 16) & 0xFF);
    g_payload[7] = (uint8_t)((seq >> 24) & 0xFF);
    uint32_t size = STRESS_SIZE;
    g_payload[8] = (uint8_t)(size & 0xFF);
    g_payload[9] = (uint8_t)((size >> 8) & 0xFF);
    g_payload[10] = (uint8_t)((size >> 16) & 0xFF);
    g_payload[11] = (uint8_t)((size >> 24) & 0xFF);
    for (uint32_t i = 12; i < STRESS_SIZE; i++) {
        g_payload[i] = (uint8_t)((i - 12) & 0xFF);
    }
}

int main(void) {
    printk("stress-zenoh-zephyr: locator=%s count=%d size=%d\n", CONFIG_NROS_ZENOH_LOCATOR,
           STRESS_COUNT, STRESS_SIZE);

    /* phase-328 (issue 0348) — the zpico C API is handle-passing; acquire one
     * pool slot for this single-session stress image. */
    zpico_session_t* s = zpico_session_acquire();
    if (s == NULL) {
        printk("STRESS_FAIL: session pool full\n");
        return 1;
    }

    /* Session open with retry: the router may come up after the image. */
    int32_t rc = -1;
    for (int attempt = 0; attempt < 40; attempt++) {
        rc = zpico_init(s, CONFIG_NROS_ZENOH_LOCATOR);
        if (rc == 0) {
            rc = zpico_open(s);
            if (rc == 0) {
                break;
            }
        }
        k_sleep(K_MSEC(500));
    }
    if (rc != 0) {
        printk("STRESS_FAIL: session open rc=%d\n", rc);
        return 1;
    }
    printk("session open\n");

    /* Phase 282 W3 (#145) — STRESS_EXPRESS=1 declares the publisher express:
     * its samples bypass tx batching (wire EXPRESS flag) even in a batching
     * image, so per-message latency is not flush-cadence-quantized. */
    int32_t handle = zpico_declare_publisher_ex(s, STRESS_KEYEXPR, STRESS_EXPRESS);
    if (handle < 0) {
        printk("STRESS_FAIL: declare rc=%d\n", handle);
        return 1;
    }

    /* Let discovery settle so the (already running) subscriber matches. */
    k_sleep(K_SECONDS(2));
    printk("Publishing...\n");

    int64_t t0 = k_uptime_get();
    for (uint32_t seq = 0; seq < STRESS_COUNT; seq++) {
        build_payload(seq);
        (void)zpico_publish(s, handle, g_payload, STRESS_SIZE);
#if STRESS_INTERVAL_MS > 0
        k_sleep(K_MSEC(STRESS_INTERVAL_MS));
#endif
    }
    int64_t elapsed = k_uptime_get() - t0;
    printk("PUBLISH_DONE: sent=%d elapsed_ms=%lld\n", STRESS_COUNT, elapsed);

    /* Drain: give the flush thread / lease keepalives time to ship any
     * batched remainder before closing (close also flushes). */
    k_sleep(K_SECONDS(3));
    zpico_close(s);
    zpico_session_release(s);
    printk("STRESS_EXIT\n");
    return 0;
}
