/*
 * phase-417 W5.e — the typed C service/client path COMPILES, in the shape a
 * ported rclc handler is written in.
 *
 * WHAT THIS PINS, and why it is not a hand-written mock:
 *
 * The include below is the fingerprint corpus's `expected/configured/
 * Probe.srv.h` — the COMMITTED OUTPUT of `packs/c/service.h.jinja`, re-recorded
 * by `cargo test -p rosidl-codegen --test codegen_golden` whenever the pack
 * changes. So this TU compiles what the generator actually emits, not a copy of
 * it that can rot. A pack edit that emits code C rejects fails HERE, in seconds,
 * instead of during a fixture build minutes later on some platform.
 *
 * Compile it with the golden dir on the include path (the `-Itarget/...` and
 * `-Ipackages/api/nros-c/include` entries are the ones every probe in this
 * directory already uses):
 *
 *   cc -fsyntax-only -std=c11 \
 *       -Ipackages/cli/rosidl-codegen/tests/fixtures/fingerprint-corpus/expected/configured \
 *       -Itarget/nros-c-generated \
 *       -Ipackages/api/nros-c/include \
 *       -Ipackages/platform/nros-platform-api/include \
 *       packages/api/nros-c/tests/compile/typed_service_handler.c
 *
 * The Probe service is the right specimen precisely because it is awkward: its
 * request carries an RFC-0033 `mode = "heap"` sequence (`items`) and a bounded
 * string, and its response carries a fixed array of strings. The heap field is
 * what makes the trampoline's `_fini` ordering load-bearing rather than
 * decorative.
 *
 * Before W5.e this file could not have existed: the pack emitted
 * `_get_type_support`, `_get_type_name`, `_get_type_hash` and the four
 * `_serialize`/`_deserialize` entry points, and NOTHING that joined them to
 * `nros_service_init`. Every C service in the tree therefore called CDR by
 * hand at the top and bottom of its handler.
 */

#include "Probe.srv.h"

#include "nros/client.h"
#include "nros/service.h"

/* ===================================================================
 * 1. The handler body, in the shape a ported rclc service reads.
 *
 * rclc:  void cb(const void* req, void* res)  — caller owns both, no
 *        allocator on the delivery path.
 * ours:  the same ownership, with the two pointers TYPED and the
 *        `void* context` rclc puts behind a separate `_with_context`
 *        registration.
 *
 * Nothing in this body serializes, deserializes, allocates, or names a
 * buffer size. That is the whole point of the stage.
 * =================================================================== */

static void probe_service_handler(const fingerprint_corpus_srv_probe_request* request,
                                  fingerprint_corpus_srv_probe_response* response, void* context) {
    int64_t total = 0;
    size_t i;

    (void)context;

    /* The heap sequence is already decoded — `.data` / `.size`, not bytes. */
    for (i = 0; i < request->items.size; ++i) {
        total += request->items.data[i];
    }

    response->sum = total;
    response->lines.size = 0;
}

/* The handler struct carries the caller-owned request and response storage, so
 * it needs the service's own storage duration. Static here, a member of the
 * node's context in a real program. */
static fingerprint_corpus_srv_probe_service_handler_t g_probe_handler;
static struct nros_service_t g_probe_service;

static nros_ret_t bring_up_typed_service(const struct nros_node_t* node) {
    nros_ret_t ret = fingerprint_corpus_srv_probe_service_handler_init(&g_probe_handler,
                                                                       probe_service_handler, NULL);
    if (ret != NROS_RET_OK) {
        return ret;
    }
    return fingerprint_corpus_srv_probe_service_init(&g_probe_service, node, "/add",
                                                     &g_probe_handler);
}

/* A refused request is queryable without a logger — `last_error` is one of the
 * NROS_SERVICE_TYPED_* codes and `error_count` is monotonic. The ERROR record
 * `nros_service_typed_report_error()` emits is the loud half; this is for a
 * caller that would rather poll than read logs. */
static bool typed_service_is_healthy(void) {
    return g_probe_handler.last_error == NROS_SERVICE_TYPED_OK && g_probe_handler.error_count == 0u;
}

/* ===================================================================
 * 2. The trampoline really is an `nros_service_callback_t`.
 *
 * A function POINTER, so a drift between the generated trampoline's argument
 * list and the executor's callback ABI is a compile error here rather than an
 * undefined-behaviour call through a mismatched pointer at run time. This is
 * the assertion the whole file exists for.
 * =================================================================== */

static const nros_service_callback_t k_trampoline =
    fingerprint_corpus_srv_probe_service_handle_request;

/* ===================================================================
 * 3. The client half — typed response callback, and the typed send/take pair.
 * =================================================================== */

static void probe_response_handler(const fingerprint_corpus_srv_probe_response* response,
                                   void* context) {
    (void)context;
    (void)response->sum;
}

static fingerprint_corpus_srv_probe_client_handler_t g_probe_client_handler;
static struct nros_client_t g_probe_client;

static const nros_response_callback_t k_response_trampoline =
    fingerprint_corpus_srv_probe_client_handle_response;

static nros_ret_t drive_typed_client(void) {
    /* Caller-supplied scratch. The service pack emits no
     * `_MAX_SERIALIZED_SIZE` constants yet (issue 0896 gave the MESSAGE pack
     * its own), so the buffer is the caller's decision rather than a hidden
     * 256-byte array that truncates without saying so. */
    uint8_t request_scratch[512];
    uint8_t response_scratch[512];
    fingerprint_corpus_srv_probe_request request;
    fingerprint_corpus_srv_probe_response response;
    nros_ret_t ret;

    ret = fingerprint_corpus_srv_probe_client_handler_init(&g_probe_client_handler,
                                                           probe_response_handler, NULL);
    if (ret != NROS_RET_OK) {
        return ret;
    }
    ret = fingerprint_corpus_srv_probe_client_set_response_callback(&g_probe_client,
                                                                    &g_probe_client_handler);
    if (ret != NROS_RET_OK) {
        return ret;
    }

    fingerprint_corpus_srv_probe_request_init(&request);
    fingerprint_corpus_srv_probe_response_init(&response);

    /* Async pair. */
    ret = fingerprint_corpus_srv_probe_client_send_request(
        &g_probe_client, &request, request_scratch, sizeof(request_scratch));
    if (ret != NROS_RET_OK) {
        return ret;
    }
    ret = fingerprint_corpus_srv_probe_client_take_response(
        &g_probe_client, &response, response_scratch, sizeof(response_scratch));
    /* NROS_RET_TRY_AGAIN is the "no reply yet" code the raw take already
     * returns, so a polling loop written against the byte API is unchanged. */
    if (ret != NROS_RET_OK && ret != NROS_RET_TRY_AGAIN) {
        return ret;
    }

    /* ...and the blocking convenience, which forwards to `nros_client_call` so
     * the timeout and the executor spin stay where they already live. */
    ret = fingerprint_corpus_srv_probe_client_call(&g_probe_client, &request, &response,
                                                   request_scratch, sizeof(request_scratch),
                                                   response_scratch, sizeof(response_scratch));

    fingerprint_corpus_srv_probe_request_fini(&request);
    fingerprint_corpus_srv_probe_response_fini(&response);
    return ret;
}

/* ===================================================================
 * 4. The loudness entry point is DECLARED, not just defined.
 *
 * Same reason `graph_query_entry_points.c` exists: a symbol the generated
 * headers call but that cbindgen never emitted would leave every typed service
 * header uncompilable while the Rust side stayed green.
 * =================================================================== */

static void (*const k_report_error)(int32_t, const char*) = nros_service_typed_report_error;

/* Reference everything so no compiler prunes the lookups this file exists to
 * force. */
const void* nros_typed_service_anchors[] = {
    (const void*)&k_trampoline,
    (const void*)&k_response_trampoline,
    (const void*)&k_report_error,
    (const void*)(const void* const*)&bring_up_typed_service,
    (const void*)(const void* const*)&typed_service_is_healthy,
    (const void*)(const void* const*)&drive_typed_client,
};
