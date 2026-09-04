/*
 * phase-417 W5.a (RFC-0089 stage 5) — TYPED subscription delivery for C.
 *
 * rclc hands a subscription callback a DESERIALISED message, on a path with no
 * allocator, by making the caller own the storage:
 *
 *     rclc_executor_add_subscription_with_context(&exec, &sub, &msg, &cb, &ctx, ON_NEW_DATA);
 *     typedef void (*rclc_subscription_callback_with_context_t)(const void*, void*);
 *
 * Ours had no `msg` slot and delivered `(const uint8_t*, size_t)`, so EVERY
 * ported callback body had to be rewritten around a hand-written
 * `<Msg>_deserialize` call. This TU is that ported body as it should now read:
 * the callback casts and reads FIELDS, and there is no CDR anywhere in it.
 *
 * Compile-only (no main). Two assertions:
 *
 *   1. the entry points exist with the DOCUMENTED signatures — taken as
 *      function POINTERS, so a drift in argument list or order fails here
 *      rather than at some consumer's call site, and
 *   2. the generated glue's SHAPE compiles: an erased deserialiser plus the
 *      five-argument rclc-ordered macro. The message type here is hand-written
 *      on purpose — `generated/` trees do not exist in a fresh clone, so a
 *      probe that included one could not run on the `check c` lane — but the
 *      two definitions below are byte-for-byte what
 *      `packages/cli/rosidl-codegen/packs/c/message.h.jinja` emits, modulo the
 *      type token.
 */

#include "nros/executor.h"
#include "nros/subscription.h"

/* ------------------------------------------------------------------ *
 * Stand-in for a generated message header.
 * ------------------------------------------------------------------ */

/* The struct a generated header defines: fixed-capacity members, so the whole
 * thing is one `.bss` object and nothing on this path allocates. */
typedef struct probe_msg_reading {
    int32_t value;
    char label[32];
} probe_msg_reading;

/* Generated: writes into caller storage, returns 0 / -1. */
int32_t probe_msg_reading_deserialize(probe_msg_reading* msg, const uint8_t* buffer,
                                      size_t buffer_size);

/* Generated (message.h.jinja): the type-erased form. A real function, not a
 * cast of the typed one — calling through a function pointer of a different
 * type is undefined even where it happens to work, and a CFI build traps. */
static inline int32_t probe_msg_reading_deserialize_erased(void* msg, const uint8_t* buffer,
                                                           size_t buffer_size) {
    return probe_msg_reading_deserialize((probe_msg_reading*)msg, buffer, buffer_size);
}

/* Generated (message.h.jinja): the five-argument, rclc-ordered registration.
 * The deserialiser and the receive-buffer hint both come from the ONE type
 * token, so they cannot disagree with each other or with the type. */
#define PROBE_MSG_READING_RX_MAX_SERIALIZED_SIZE 48
#define probe_msg_reading_executor_add_subscription_sized(executor, subscription, msg, cb, ctx,    \
                                                          invocation, rx_bytes)                    \
    nros_executor_add_subscription_typed_sized((executor), (subscription), (msg),                  \
                                               probe_msg_reading_deserialize_erased, (cb), (ctx),  \
                                               (invocation), (uint32_t)(rx_bytes))
#define probe_msg_reading_executor_add_subscription(executor, subscription, msg, cb, ctx,          \
                                                    invocation)                                    \
    probe_msg_reading_executor_add_subscription_sized((executor), (subscription), (msg), (cb),     \
                                                      (ctx), (invocation),                         \
                                                      PROBE_MSG_READING_RX_MAX_SERIALIZED_SIZE)

/* ------------------------------------------------------------------ *
 * 1. The entry points exist with the documented signatures.
 * ------------------------------------------------------------------ */

static nros_ret_t (*const kAddTyped)(
    struct nros_executor_t*, struct nros_subscription_t*, void*, nros_message_deserialize_fn_t,
    nros_typed_subscription_callback_t, void*,
    nros_executor_handle_invocation_t) = nros_executor_add_subscription_typed;

static nros_ret_t (*const kAddTypedSized)(struct nros_executor_t*, struct nros_subscription_t*,
                                          void*, nros_message_deserialize_fn_t,
                                          nros_typed_subscription_callback_t, void*,
                                          nros_executor_handle_invocation_t,
                                          uint32_t) = nros_executor_add_subscription_typed_sized;

/* The two callback typedefs are what a ported file writes its handlers against,
 * so they must be nameable and must have rclc's shapes. */
static const nros_typed_subscription_callback_t kNullTypedCb =
    (nros_typed_subscription_callback_t)0;
static const nros_message_deserialize_fn_t kErased = probe_msg_reading_deserialize_erased;

/* The raw path STAYS — this is additive, and a byte-oriented subscriber is a
 * legitimate thing to want. Naming it here is the assertion that W5.a did not
 * replace it. */
static nros_ret_t (*const kAddRaw)(struct nros_executor_t*, struct nros_subscription_t*,
                                   nros_executor_handle_invocation_t) =
    nros_executor_add_subscription;

/* ------------------------------------------------------------------ *
 * 2. A ported rclc node, as it now reads.
 * ------------------------------------------------------------------ */

typedef struct {
    int count;
} probe_app_ctx;

static struct {
    nros_executor_t executor;
    nros_subscription_t subscription;
    /* The caller's message storage. This is the whole mechanism: no allocator
     * is involved on the delivery path because the storage is already here,
     * exactly as rclc does it. */
    probe_msg_reading msg;
    probe_app_ctx app;
} g_app;

/* The ported callback body. `const void*` + `void*` is rclc's
 * `rclc_subscription_callback_with_context_t`; the cast and the field reads are
 * all a ported rclc handler contains. Nothing here deserialises, and nothing
 * here sees CDR bytes.
 *
 * It is also never called on a failed decode — a sample that cannot be decoded
 * into `msg` (a string longer than its declared bound included) is dropped and
 * counted, so this body cannot be handed the PREVIOUS message dressed as the
 * new one. */
static void probe_on_reading(const void* msgin, void* context) {
    const probe_msg_reading* msg = (const probe_msg_reading*)msgin;
    probe_app_ctx* ctx = (probe_app_ctx*)context;
    ctx->count += (msg->value != 0) ? 1 : 0;
}

nros_ret_t nros_typed_subscription_probe(void);
nros_ret_t nros_typed_subscription_probe(void) {
    (void)kAddTyped;
    (void)kAddTypedSized;
    (void)kAddRaw;
    (void)kNullTypedCb;
    (void)kErased;

    /* rclc:  rclc_executor_add_subscription_with_context(&exec, &sub, &msg, &cb, &ctx,
     *                                                     ON_NEW_DATA);
     * ours:  the same six arguments, in the same order. */
    return probe_msg_reading_executor_add_subscription(&g_app.executor, &g_app.subscription,
                                                       &g_app.msg, probe_on_reading, &g_app.app,
                                                       NROS_EXECUTOR_ON_NEW_DATA);
}
