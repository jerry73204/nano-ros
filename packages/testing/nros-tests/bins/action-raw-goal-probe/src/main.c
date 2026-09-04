/// @file main.c
/// @brief Wire proof that `nros_action_client_send_goal_raw` ships exactly ONE
///        CDR encapsulation header (issue 0454 / phase-354 W3).
///
/// WHY THIS BINARY EXISTS AT ALL
///
/// Before it, NOTHING in the tree called `nros_action_client_send_goal_raw` —
/// no example, no fixture, no test. That is why the double-header defect
/// survived review and shipped: an FFI with no caller has no failing test to
/// write, and `scripts/check-goal-cdr-stripped.py` could only guard the shape
/// of the fix, never its effect. This is the caller.
///
/// WHY A RUNNING PROGRAM AND NOT A UNIT TEST
///
/// Phase-354 W3 asks for a goal that round-trips against a real peer,
/// "demonstrated on the wire rather than by reading the encoder". The defect is
/// invisible to encoder inspection: a correct goal and a double-encapsulated
/// one both serialize without error. Only a PEER parsing the bytes tells them
/// apart.
///
/// HOW IT IS FALSIFIABLE
///
/// The FFI parameter is `goal_cdr`: CDR, which begins with the 4-byte
/// encapsulation header. The core it feeds (`PollingActionClientCore::
/// send_goal_raw`) wants goal FIELDS, and appends them after a header the
/// writer already emitted. Before the fix those 4 bytes stayed in, shifting
/// every field right by four:
///
///     correct   : [00 01 00 00] [order:i32 = 7]
///     the defect: [00 01 00 00] [00 01 00 00] [order:i32 = 7]
///                                ^^^^^^^^^^^ four bytes nobody expects
///
/// MEASURED, not reasoned: with the strip removed the server logs `order 256`
/// (the request it parses is `[encap][GoalId(16)][order]`, so the extra four
/// bytes shift the tail and `order` reads a straddle, not the header itself —
/// the tidy "it reads 65536" prediction is wrong). The C action server prints
/// the order it parsed (`Received goal request with order %d`) and rejects
/// anything `>= 64`, so the regression is visible three ways at once: the
/// server's own line, the rejection, and the missing result. The consuming
/// test (`tests/action_raw_goal_e2e.rs`) asserts on the SERVER's output, which
/// is the strongest available statement that the right bytes went out — it does
/// not trust this program's view of what it sent.
///
/// ORDER = 7 deliberately: small enough for the default result buffer, inside
/// the server's accept range, and not a value the misparse can produce.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nros/action.h>
#include <nros/app_main.h>
#include <nros/check.h>
#include <nros/init.h>
#include <nros/node.h>

#include "example_interfaces.h"

/// The order we ask for. See the header note on why this value.
#define GOAL_ORDER 7

/// Fibonacci(order) yields `order + 1` terms — the server loops `0..=order`.
#define EXPECTED_SEQUENCE_LEN (GOAL_ORDER + 1)

/// Polling budget per phase. The polling (L1) client has no
/// `wait_for_action_server` — that primitive takes an executor, which this
/// client deliberately does not use — so discovery is absorbed by resending the
/// goal, the same shape the pre-338 NuttX client used.
#define SEND_ATTEMPTS 10
#define POLLS_PER_ATTEMPT 200
#define RESULT_POLLS 2000
#define POLL_SLEEP_US 10000 /* 10 ms */

static struct {
    nros_support_t support;
    nros_node_t node;
    nros_action_client_t action_client;
} app;

static void sleep_tick(void) {
    nros_platform_sleep_us(POLL_SLEEP_US);
}

/// Dump a byte prefix so a layout surprise is diagnosable from the log rather
/// than from a rebuild with printfs added.
static void dump_prefix(const char* what, const uint8_t* buf, int32_t len) {
    int32_t n = len < 24 ? len : 24;
    printf("%s (%d bytes):", what, len);
    for (int32_t i = 0; i < n; i++) {
        printf(" %02x", buf[i]);
    }
    printf("\n");
}

static uint32_t read_u32_le(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int nros_app_main(int argc, char** argv) {
    (void)argc;
    (void)argv;

#ifdef _IOLBF
    setvbuf(stdout, NULL, _IOLBF, 0);
#endif

    const char* locator = getenv("NROS_LOCATOR");
    if (!locator) {
        locator = NROS_ENTRY_LOCATOR;
    }
    const char* domain_str = getenv("ROS_DOMAIN_ID");
    uint8_t domain_id = (uint8_t)NROS_ENTRY_DOMAIN_ID;
    if (domain_str) {
        domain_id = (uint8_t)atoi(domain_str);
    }

    printf("raw-goal probe: locator=%s domain=%d\n", locator, domain_id);

    memset(&app, 0, sizeof(app));

    nros_action_type_t fibonacci_type = {
        .type_name = example_interfaces_action_fibonacci_get_type_name(),
        .type_hash = example_interfaces_action_fibonacci_get_type_hash(),
        .goal_serialized_size_max = 8,
        .result_serialized_size_max = 264,
        .feedback_serialized_size_max = 264,
    };

    NROS_CHECK_RET(nros_support_init(&app.support, locator, domain_id), 1);
    NROS_CHECK_RET(rclc_node_init_default(&app.node, "raw_goal_probe", "/", &app.support), 1);
    // The RAW arms live on the polling (L1) core — `polling_client_core()`
    // returns NULL for an executor-mode client, so this must be init_polling.
    NROS_CHECK_RET(nros_action_client_init_polling(&app.action_client, &app.node, &fibonacci_type,
                                                   "/fibonacci"),
                   1);
    printf("raw-goal probe ready\n");

    // Build the goal exactly as the FFI's parameter name promises: CDR, header
    // first. This is the whole point — a caller reading `goal_cdr` supplies
    // this, and the FFI must strip it before the core appends its own. Built by
    // hand rather than via the generated serializer so the bytes on the wire
    // are stated here, in the test, where a reader can check them.
    uint8_t goal_cdr[8];
    goal_cdr[0] = 0x00; /* encapsulation kind: CDR_LE */
    goal_cdr[1] = 0x01;
    goal_cdr[2] = 0x00; /* options */
    goal_cdr[3] = 0x00;
    const int32_t order = GOAL_ORDER;
    memcpy(&goal_cdr[4], &order, sizeof(order));
    dump_prefix("goal_cdr", goal_cdr, (int32_t)sizeof(goal_cdr));

    uint8_t goal_id[16];
    uint8_t resp[64];
    int32_t got = 0;
    int accepted = 0;

    for (int attempt = 0; attempt < SEND_ATTEMPTS && !accepted; attempt++) {
        nros_ret_t sent = nros_action_client_send_goal_raw(&app.action_client, goal_cdr,
                                                           sizeof(goal_cdr), &goal_id);
        if (sent != NROS_RET_OK) {
            // The server's queryable may not be routable yet; retry.
            sleep_tick();
            continue;
        }
        printf("goal sent (attempt %d): order=%d\n", attempt, order);

        // No `nros_action_client_poll` here: that arm drives the EXECUTOR
        // arena core and rejects a polling client outright (state POLLING vs
        // INITIALIZED). On a hosted build the zenoh session has its own read
        // task, so `take_*_raw` is the whole pump.
        for (int i = 0; i < POLLS_PER_ATTEMPT; i++) {
            got = nros_action_client_try_recv_goal_response_raw(&app.action_client, resp,
                                                                sizeof(resp));
            if (got > 0) {
                break;
            }
            sleep_tick();
        }
        if (got <= 0) {
            continue;
        }

        dump_prefix("goal response", resp, got);
        // Reply layout, per `try_recv_send_goal_reply`'s doc:
        // header (4) + accepted (u8) + stamp (i32 + u32).
        if (got < 5) {
            fprintf(stderr, "goal response too short (%d bytes) to hold an accept flag\n", got);
            return 2;
        }
        accepted = resp[4] != 0;
        if (!accepted) {
            fprintf(stderr,
                    "peer REJECTED the goal. With the pre-0454 double header the server reads "
                    "order 256 and refuses it as out of range — that is the bug's signature.\n");
            return 3;
        }
    }

    if (!accepted) {
        fprintf(stderr, "no goal response after %d send attempts — peer never replied\n",
                SEND_ATTEMPTS);
        return 2;
    }
    printf("goal accepted for order=%d\n", order);

    // Ask for the result. Acceptance alone is NOT the proof: a server could
    // accept a garbage order. The sequence LENGTH is a function of the order the
    // peer parsed, so it pins the bytes that actually crossed.
    NROS_CHECK_RET(nros_action_client_send_get_result_request_raw(&app.action_client, &goal_id), 4);

    uint8_t result[512];
    int32_t rlen = 0;
    for (int i = 0; i < RESULT_POLLS; i++) {
        rlen = nros_action_client_try_recv_result_raw(&app.action_client, result, sizeof(result));
        if (rlen > 0) {
            break;
        }
        sleep_tick();
    }
    if (rlen <= 0) {
        fprintf(stderr, "no get_result reply after %d polls\n", RESULT_POLLS);
        return 4;
    }
    dump_prefix("get_result reply", result, rlen);

    // Reply layout, per `try_recv_get_result_reply`'s doc: CDR header (4) +
    // status byte (1) + result payload. The payload's first member is
    // `int32[] sequence`, whose u32 length is 4-aligned WITHIN the CDR body
    // (body offset 0 is the status byte, so the length lands at body offset 4
    // = absolute offset 8, with three padding bytes between).
    if (rlen < 12) {
        fprintf(stderr, "get_result reply too short (%d bytes) to hold a sequence length\n", rlen);
        return 5;
    }
    const uint8_t status = result[4];
    const uint32_t seq_len = read_u32_le(&result[8]);
    printf("get_result: status=%u sequence_len=%u\n", (unsigned)status, (unsigned)seq_len);

    if (seq_len != (uint32_t)EXPECTED_SEQUENCE_LEN) {
        fprintf(stderr,
                "sequence length %u != %d — the peer computed Fibonacci for an order we did "
                "not send. A double encapsulation header is the known cause.\n",
                (unsigned)seq_len, EXPECTED_SEQUENCE_LEN);
        return 6;
    }

    printf("raw goal shipped exactly one encapsulation header\n");

    nros_action_client_fini(&app.action_client);
    rcl_node_fini(&app.node);
    rclc_support_fini(&app.support);
    return 0;
}

NROS_APP_MAIN_REGISTER()
