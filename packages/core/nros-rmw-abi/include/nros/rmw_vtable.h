#ifndef NROS_RMW_VTABLE_H
#define NROS_RMW_VTABLE_H

/* Phase 376 W3.a — this header defines the GENERIC RMW ABI: its types are
 * `rmw_publisher_t`, `rmw_ret_t`, and so on, with no vendor prefix, because a
 * backend author implements RMW rather than nano-ros.
 *
 * That makes it impossible to share a translation unit with upstream's
 * `<rmw/rmw.h>`, which defines the same names differently. Refuse LOUDLY rather
 * than let one definition silently win a redefinition race — two types of one
 * name whose layouts disagree is the class this repo has already paid for three
 * times in hand-mirrored FFI structs.
 *
 * A target image never links real rmw, and host-side consumers reach the
 * backend through Rust, so nothing in this repo trips this today. It exists for
 * the day something does. */
#if defined(RMW_RMW_H_) || defined(RMW__RMW_H_)
#error "nros/rmw_vtable.h defines the generic RMW ABI and cannot share a translation unit with upstream <rmw/rmw.h>. Include one or the other."
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "nros/rmw_ret.h"
#include "nros/rmw_entity.h"
#include "nros/rmw_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file rmw_vtable.h
 * @brief C function table for plugging third-party RMW backends into nros.
 *
 * Implement the functions in nros_rmw_vtable_t and call
 * nros_rmw_cffi_register() before creating any nros sessions.
 *
 * **Storage ownership.** The runtime owns the entity-struct storage
 * (`rmw_session_t`, `rmw_publisher_t`, `rmw_subscription_t`,
 * `rmw_service_t`, `rmw_client_t`). Each
 * `create_*` call receives a runtime-allocated, zero-initialised struct
 * via the `out` pointer; the backend writes its `backend_data` into it.
 * (It does NOT write `can_loan_messages`: the runtime derives that from
 * the loan slots — issue 0814.) The runtime fills the metadata
 * fields (`topic_name`, `type_name`, `qos`) before calling
 * `create_*`; the backend reads them through the same struct.
 *
 * `destroy_*` releases the backend's `backend_data` only. The struct
 * shell stays valid until the runtime drops its owner.
 *
 * **Return-value conventions.** One rule, no exceptions:
 *
 *  - EVERY slot returns `rmw_ret_t`. `NROS_RMW_RET_OK` (which is 0, upstream's
 *    value) on success; a POSITIVE named constant on failure, upstream's where
 *    upstream has a name for it and one above `NROS_RMW_RET_EXTENSION_BASE`
 *    where it does not. See `<nros/rmw_ret.h>`.
 *  - An ANSWER is an out-parameter, never the return: `bool *taken`,
 *    `size_t *out_len`, `bool *out_has_data`. No slot multiplexes a count or a
 *    flag with a status, so no caller may test a status by its SIGN — gated by
 *    `scripts/check-rmw-ret-sign.py`.
 *  - Out-parameters are written only on `NROS_RMW_RET_OK` unless a slot says
 *    otherwise.
 *
 * This paragraph described the PRE-W3.d ABI until 2026-08-24 and was wrong on
 * five counts at once: it promised NEGATIVE error constants (step B adopted
 * upstream's positive numbering), it said `try_recv_*` returned a byte count
 * and `has_data` returned 1-or-0 (step A moved both to out-parameters), it said
 * `destroy_*` returned void (W5 gave all six a status), and it named five slots
 * that no longer exist — `publish_raw`, `send_reply`, `try_recv_raw`,
 * `try_recv_request`, `try_recv_reply_raw`, all renamed by W3.b. Prose in the
 * SSoT header is not covered by any of the shape gates, which compare
 * DECLARATIONS against the header and never read what the header says about
 * itself.
 */


/* ---- Phase 376 W4 — graph enumeration visitors ----
 *
 * Upstream returns `rcutils_string_array_t` and `rmw_names_and_types_t`, which
 * ALLOCATE — two levels deep for names-and-types. There is no allocator at this
 * seam, and a caller-provides-the-buffer shape is worse than it looks: the ROS
 * graph has no bound the CALLER can know, so a 128 KiB image would have to
 * reserve permanently for the worst graph it might ever meet, and the backend
 * would still have to materialise the whole list to fill the buffer.
 *
 * So enumeration is a VISITOR. Peak extra RAM is one entry, the backend streams
 * from state it already holds, and a caller with a bound stops early by
 * returning `false` — a normal outcome, not a truncation error. Same shape as
 * `process_raw_in_place` and `publish_streamed` already use here.
 *
 * Every string handed to a visitor is BORROWED for that call only.
 */

/** Visit one node. `enclave` is NULL where the backend does not track one —
 *  which is what lets a single slot answer both `rmw_get_node_names` and
 *  `rmw_get_node_names_with_enclaves`. Return `false` to stop. */
typedef bool (*rmw_node_visit_fn)(void *ctx, const char *node_name,
    const char *node_namespace, const char *enclave);

/** Visit one name and the types on it. `types_count` may legitimately be 0 on a
 *  partially discovered graph — reporting the name without a type beats
 *  dropping it. Return `false` to stop. */
typedef bool (*rmw_names_and_types_visit_fn)(void *ctx, const char *name,
    const char *const *types, size_t types_count);

/** Visit one discovered endpoint. Return `false` to stop. */
typedef bool (*rmw_topic_endpoint_info_visit_fn)(void *ctx,
    const rmw_topic_endpoint_info_t *info);

/** An opaque per-loan handle — phase-406 W3.
 *
 *  Was a bare `void *`. The type says nothing a reader can use, and nothing
 *  stops a caller returning a publisher's loan token to a subscription, or a
 *  token from one backend to another: both compile, and both are undefined
 *  behaviour discovered at run time on a target with no allocator to notice.
 *
 *  An INCOMPLETE type costs nothing at the ABI — it is still a pointer — and
 *  makes those two mistakes a compile error. The backend defines the struct
 *  privately; nobody outside it may dereference one, which was already the
 *  contract and is now enforced rather than documented.
 *
 *  Issue 0781 asked whether the subscription-side loan slots earn their place
 *  given nothing implements them. The answer taken here is KEEP AND TYPE: a
 *  slot nobody can use safely is worse than one nobody uses. */
typedef struct rmw_loan_token_t rmw_loan_token_t;

/** A visitor: the function and the context it needs — phase-406 W2.
 *
 *  Upstream fills a heap-owning out-parameter the caller must `fini`
 *  (`rmw_names_and_types_t`, `rcutils_string_array_t`). We visit instead: one
 *  call per entry, strings borrowed for the call, no allocation and nothing to
 *  leak — which is what makes the graph family available on a target with no
 *  allocator at all.
 *
 *  The pair was TWO arguments on twelve slots, and they are meaningless apart:
 *  a `visit` without its `ctx` cannot find its state, and a `ctx` without its
 *  `visit` goes nowhere. Grouping them is the same move as the type support,
 *  and it makes "pass one, forget the other" unrepresentable rather than merely
 *  discouraged.
 *
 *  `fn == NULL` is INVALID_ARGUMENT, never a silent no-op: a graph query that
 *  visits nothing is indistinguishable from an empty graph, which is the exact
 *  confusion issue 0903 cost days to.
 *
 *  Passed BY VALUE — two words, and nothing is written back through it. */
#define NROS_RMW_VISITOR_DEFINED 1
typedef struct rmw_node_visitor_t {
    /* Named `visit`, not `fn`: `fn` is a Rust keyword, and this struct is
       mirrored into Rust by bindgen (RFC-0054 — the C header is the SSoT,
       so the C side is where a name has to be chosen that both languages
       can spell). */
    rmw_node_visit_fn visit;
    void *ctx;
} rmw_node_visitor_t;

typedef struct rmw_names_and_types_visitor_t {
    /* Named `visit`, not `fn`: `fn` is a Rust keyword, and this struct is
       mirrored into Rust by bindgen (RFC-0054 — the C header is the SSoT,
       so the C side is where a name has to be chosen that both languages
       can spell). */
    rmw_names_and_types_visit_fn visit;
    void *ctx;
} rmw_names_and_types_visitor_t;

typedef struct rmw_topic_endpoint_info_visitor_t {
    /* Named `visit`, not `fn`: `fn` is a Rust keyword, and this struct is
       mirrored into Rust by bindgen (RFC-0054 — the C header is the SSoT,
       so the C side is where a name has to be chosen that both languages
       can spell). */
    rmw_topic_endpoint_info_visit_fn visit;
    void *ctx;
} rmw_topic_endpoint_info_visitor_t;


/* Phase 376 W5 — teardown and loan-return REPORT.
 *
 * These six returned `void`. That was never a target constraint: upstream
 * returns `rmw_ret_t` from all of them, and a backend that cannot release a
 * handle — a queue still draining, a token from another publisher, a
 * double-destroy — had no way to say so. The runtime then reported success it
 * had not verified, which on an RTOS is the difference between a leak that
 * shows up in a log and a leak that shows up as an allocation failure an hour
 * later with no provenance.
 *
 * A caller that genuinely has nothing to do with the status may ignore it;
 * `void` denied it the choice. Backends that cannot fail return
 * `NROS_RMW_RET_OK` unconditionally, which costs one instruction and keeps the
 * signature honest for the ones that can.
 */
typedef struct nros_rmw_vtable_t {
    /* ---- Session lifecycle ---- */
    /** Create a session (phase-301: renamed from `open` to the table's
     *  own `create_*` convention). The runtime supplies a
     *  zero-initialised `rmw_session_t` via @p out with
     *  `node_name` / `namespace_` already filled. The backend writes
     *  `out->backend_data`.
     *
     *  @param mode One of `nros_rmw_session_mode_t`. Passed as `uint8_t`
     *              rather than the enum to keep the slot's width fixed
     *              across compilers. A backend with no peer/client
     *              distinction must IGNORE it, not reject it.
     *  @param options NULLable; NULL means every default. Issue 0808 — the
     *              home for init-time context this flat list cannot grow
     *              without another break. `mode` is NOT moved into it: doing
     *              so would be a second break for no gain, and it is already
     *              a named argument every backend reads. What moved in are the
     *              two fields issue 0785 measured as GAPS, `localhost_only`
     *              and `enclave`. */
    rmw_ret_t (*create_session)(const char *locator, uint8_t mode,
                           uint32_t domain_id, const char *node_name,
                           const rmw_session_options_t *options,
                           rmw_session_t *out);
    rmw_ret_t (*destroy_session)(rmw_session_t *session);
    rmw_ret_t (*drive_io)(rmw_session_t *session, int32_t timeout_ms);

    /* ---- Publisher ---- */
    /** Create a publisher. The runtime fills `out->topic_name`,
     *  `out->type_name`, `out->qos` before this call; the backend
     *  writes `out->backend_data`. `out->can_loan_messages` is DERIVED
     *  by the runtime, not written here (issue 0814).
     *  `options` carries transport hints (phase-301: moved out of the
     *  QoS struct); NULL = all defaults. */
    rmw_ret_t (*create_publisher)(const rmw_node_t *node,
        const rmw_message_type_support_t *type_support,
        const char *topic_name,
        uint32_t domain_id, const rmw_qos_profile_t *qos,
        const rmw_publisher_options_t *options,
        rmw_publisher_t *out);
    rmw_ret_t (*destroy_publisher)(rmw_publisher_t *publisher);
    rmw_ret_t (*publish)(const rmw_publisher_t *publisher,
        rmw_byte_span_t payload);

    /* ---- Subscription (phase-301: rmw's term; was `subscriber`) ---- */
    /** `options` carries transport hints (phase-301: moved out of the
     *  QoS struct); NULL = all defaults. */
    rmw_ret_t (*create_subscription)(const rmw_node_t *node,
        const rmw_message_type_support_t *type_support,
        const char *topic_name,
        uint32_t domain_id, const rmw_qos_profile_t *qos,
        const rmw_subscription_options_t *options,
        rmw_subscription_t *out);
    rmw_ret_t (*destroy_subscription)(rmw_subscription_t *subscription);
    /** Upstream `rmw_take`. Phase 376 W3.b/W3.d step A.
     *
     *  `*taken` says whether a message was copied; `*out_len` is
     *  how many bytes, meaningful only when taken. Both are
     *  written only on `NROS_RMW_RET_OK`.
     *
     *  Second slot to retire `NROS_RMW_RET_NO_DATA`: an empty
     *  subscription is `taken = false` with OK, which is what
     *  upstream's `taken` out-parameter means.
     *
     *  Deviations from upstream, declared:
     *  - `buf` / `buf_len` / `*out_len` replace upstream's typed
     *    `void *ros_message`. There is no typesupport indirection
     *    on target — the payload is bytes and the caller owns the
     *    buffer, so it needs the length back.
     *  - no `rmw_subscription_allocation_t *`: it is an OPAQUE
     *    per-implementation handle (`{const char *implementation_identifier;
     *    void *data;}` in Humble's `rmw/types.h` — no allocator in
     *    it), and the only thing that produces one is
     *    `rmw_init_subscription_allocation`, whose other parameters
     *    are a typesupport pointer and a sequence bound, both
     *    declined ABI-wide. Nothing can make one, so the argument has
     *    nothing to point at. Two earlier reasons here were wrong:
     *    "pools are baked" (issue 0777 — cyclonedds calls
     *    `ddsrt_calloc` on this very path) and then "upstream
     *    pre-sizes an `rcutils_allocator_t`" (there is none).
     *
     *  Phase 403 W1 — `buf_len` is AUTHORITATIVE on every call, and a
     *  sample that does not fit it is a FAILURE (`*taken = false` with
     *  `NROS_RMW_RET_BUFFER_TOO_SMALL`), never a truncated success.
     *  That holds however the backend treated
     *  `rmw_subscription_options_t.rx_buffer_hint`, whose doc writes the
     *  rule out in full. */
    rmw_ret_t (*take)(const rmw_subscription_t *subscription,
        rmw_mut_byte_span_t *out, bool *taken);
    /** Phase 376 W3.d step A — status in the return, answer in the
     *  out-parameter, so no slot multiplexes a flag with a status.
     *  `*out_has_data` is written only on `NROS_RMW_RET_OK`.
     *
     *  RTOS addition: upstream has no equivalent, because a hosted
     *  caller reaches for a wait-set. This is the poll a loop with
     *  no wait-set needs, and it allocates nothing.
     *
     *  Logically read-only, and that is weaker than it sounds: zenoh's
     *  implementation fires deadline and liveliness callbacks from inside this
     *  probe and writes their cells, and cyclonedds' peeks its reader (which
     *  marks samples READ). The rule a backend must actually keep is that a
     *  probe may not CONSUME a message — the sample a `has_data` reports must
     *  still be there for the `take` that follows. The stronger "must not
     *  mutate subscription state" was recorded here and true of nobody
     *  (issue 0780). */
    rmw_ret_t (*has_data)(rmw_subscription_t *subscription,
        bool *out_has_data);

    /* ---- Service (phase-301: rmw's term; was `service_server`) ---- */
    /* Phase 193.1b — `qos` applies to both the request + reply endpoints
       (one profile per service, mirrors create_publisher/subscription). */
    rmw_ret_t (*create_service)(const rmw_node_t *node,
        const rmw_service_type_support_t *type_support,
        const char *service_name,
        uint32_t domain_id, const rmw_qos_profile_t *qos,
        rmw_service_t *out);
    rmw_ret_t (*destroy_service)(rmw_service_t *server);
    /** Upstream `rmw_take_request`. Phase 376 W3.b/W3.d step A.
     *
     *  `*taken` says whether a request was copied, `*out_len` how
     *  many bytes, `*seq_out` the sequence number to reply against.
     *  All three are written only on `NROS_RMW_RET_OK`.
     *
     *  Deviations from upstream, declared: the payload is bytes
     *  (`buf` / `buf_len` / `*out_len`) rather than a typed
     *  `void *ros_request`, and `*seq_out` stands in for
     *  `rmw_service_info_t *` — an RTOS reply needs the sequence
     *  and nothing else in that struct. */
    rmw_ret_t (*take_request)(const rmw_service_t *server,
        rmw_mut_byte_span_t *request, int64_t *seq_out, bool *taken);
    /** Phase 376 W3.d step A — the service-side sibling of
     *  `has_data`; same contract, same reason. */
    rmw_ret_t (*has_request)(rmw_service_t *server,
        bool *out_has_request);
    rmw_ret_t (*send_response)(const rmw_service_t *server,
        int64_t seq, rmw_byte_span_t response);

    /* ---- Client (phase-301: rmw's term; was `service_client`) ---- */
    rmw_ret_t (*create_client)(const rmw_node_t *node,
        const rmw_service_type_support_t *type_support,
        const char *service_name,
        uint32_t domain_id, const rmw_qos_profile_t *qos,
        rmw_client_t *out);
    rmw_ret_t (*destroy_client)(rmw_client_t *client);

    /** Phase 130.4 — non-blocking send_request_raw. Phase-301: the
     *  deprecated blocking `call_raw` slot is DELETED (rmw has no
     *  blocking call); this + `try_recv_reply_raw` is the ONE
     *  request/reply path and both slots are now REQUIRED for a backend
     *  that supports services.
     *
     *  Sends the request to the backend without blocking for a
     *  reply. Returns immediately.
     *
     *  `*sequence_id` is the id the backend assigned, written only on
     *  `NROS_RMW_RET_OK`. Upstream returns it for one reason and it is the
     *  same reason here: a client with two calls outstanding has nothing else
     *  to match a reply against.
     *
     *  Issue 0778 — this out-parameter was ABSENT until 2026-08-25, and every
     *  backend computed the id and threw it away (cyclonedds a
     *  `RequestId{guid, seq}`, zenoh a `fetch_add` into the rmw attachment,
     *  xrce `uxr_buffer_request`'s id). With nothing to correlate BY, each
     *  invented a policy: cyclonedds ABANDONED the first request when a second
     *  was sent, zenoh took FIRST REPLY WINS on the grounds that "a queryable
     *  is idempotent at the application layer" — which this ABI cannot
     *  enforce and which is false for `send_goal` and `SetParameters`, both of
     *  which travel this path. Same application code, different behaviour per
     *  transport. The id is what deletes both policies. */
    rmw_ret_t (*send_request)(const rmw_client_t *client,
        rmw_byte_span_t request, int64_t *sequence_id);

    /** Upstream `rmw_take_response`. Same shape and the same
     *  declared deviations as `take_request`.
     *
     *  `*seq_out` is the `sequence_id` of the request this reply answers,
     *  written only when `*taken` is true. It is the other half of issue
     *  0778: handing the id out at send time is useless if it does not come
     *  back. Mirrors `take_request`'s `seq_out`, which the SERVER side has
     *  always had — the asymmetry was the tell.
     *
     *  (The paragraph that used to sit here described `>= 0` = bytes and
     *  "other negative = backend error", the pre-W3.d shape, three phases
     *  after step A moved the count to an out-parameter and step B made the
     *  errors positive.) */
    rmw_ret_t (*take_response)(const rmw_client_t *client,
        rmw_mut_byte_span_t *reply, int64_t *seq_out, bool *taken);

    /* ---- Phase 108 — status events (optional) ---- */
    /** Register a callback for a subscription-side event. NULL function
     *  pointer = backend doesn't generate any subscription events.
     *  Specific kind unsupported on a backend that supports some
     *  events = `NROS_RMW_RET_UNSUPPORTED` return.
     *  `deadline_ms` is consulted for `REQUESTED_DEADLINE_MISSED`
     *  only; ignored otherwise. */
    rmw_ret_t (*subscription_event_init)(
        const rmw_subscription_t *subscription,
        rmw_event_type_t  kind,
        uint32_t               deadline_ms,
        rmw_status_event_callback_t cb,
        void                  *user_context);

    /** Register a callback for a publisher-side event. Same NULL /
     *  unsupported-kind conventions as `register_subscription_event`.
     *  `deadline_ms` is consulted for `OFFERED_DEADLINE_MISSED` only. */
    /* ---- Status events: how the three upstream parts map here ----
     *
     * Upstream's model is three-part: `*_event_init` fills an `rmw_event_t`
     * HANDLE, `rmw_event_set_callback` attaches a callback to that handle, and
     * `rmw_take_event` polls it for a status.
     *
     *  - `rmw_event_set_callback` is FUSED into `*_event_init`, which takes the
     *    callback directly. There is no handle to attach one to afterwards.
     *    What that costs is real and small: upstream can replace or clear a
     *    callback later, and we cannot.
     *
     *  - `rmw_take_event` is a SLOT (issue 0780), split per entity kind
     *    because we have no `rmw_event_t` to carry the entity — see
     *    `subscription_take_event` / `publisher_take_event` below.
     *
     * It was DECLINED until 2026-08-25 on two clauses that both turned out
     * false, and they are worth recording because each sounded right:
     *
     *   "Upstream polls because the WAIT SET said one was ready, and the wait
     *   set is declined here, so a poll would be blind."
     *      — This ABI is a poll-WITHOUT-a-wait-set design by construction.
     *        `has_data`'s own doc says so: "the poll a loop with no wait-set
     *        needs, and it allocates nothing." A poll with no wait set is the
     *        model here, not a blindness.
     *
     *   "Our callback already runs on the safe context — from inside
     *   `drive_io`, on the executor thread, never an ISR or a transport
     *   thread."
     *      — True of no backend. zenoh fires from `try_recv_raw` and
     *        `has_data`; cyclonedds' DDS listeners fire on Cyclone's own
     *        worker thread while its `drive_io` is a sleep with no callback
     *        path at all. So a cyclonedds status event has nowhere safe to be
     *        delivered from, and buffer-plus-poll IS `take_event`. A
     *        per-backend fact stated ABI-wide — the same shape as the
     *        network-flow decline W5 flipped.
     */

    /** Upstream `rmw_take_event`, subscription side.
     *
     *  `*taken` says whether an event was copied into `*out`; both are written
     *  only on `NROS_RMW_RET_OK`. `kind` selects which event to drain, and
     *  which member of the payload union is valid.
     *
     *  Deviations from upstream, declared: no `rmw_event_t *` — that handle is
     *  declined, so the entity plus the kind identifies the event — and the
     *  payload is our `rmw_event_payload_t` union rather than a `void *` the
     *  caller must know the shape of.
     *
     *  NULL is the normal answer for a backend that delivers status events
     *  through the `*_event_init` callback and has a safe context to do it
     *  from. It is NOT the right answer for a backend whose notifications
     *  arrive on a thread of its own. */
    rmw_ret_t (*subscription_take_event)(const rmw_subscription_t *subscription,
        rmw_event_type_t kind, rmw_event_payload_t *out, bool *taken);

    /** Upstream `rmw_take_event`, publisher side. Same contract as
     *  `subscription_take_event`; `rmw_take_event` is recorded as GROUPED onto
     *  that one, since upstream has a single name for both. */
    rmw_ret_t (*publisher_take_event)(const rmw_publisher_t *publisher,
        rmw_event_type_t kind, rmw_event_payload_t *out, bool *taken);

    rmw_ret_t (*publisher_event_init)(
        const rmw_publisher_t  *publisher,
        rmw_event_type_t  kind,
        uint32_t               deadline_ms,
        rmw_status_event_callback_t cb,
        void                  *user_context);

    /** Phase 108.B — manually assert this publisher's liveliness.
     *  Required for `MANUAL_BY_TOPIC` / `MANUAL_BY_NODE` liveliness
     *  kinds; no-op (return `NROS_RMW_RET_OK`) for other kinds.
     *  NULL function pointer = backend doesn't support manual
     *  liveliness; runtime returns `NROS_RMW_RET_OK` for AUTOMATIC /
     *  NONE callers and `NROS_RMW_RET_UNSUPPORTED` for MANUAL_*. */
    rmw_ret_t (*publisher_assert_liveliness)(
        const rmw_publisher_t *publisher);

    /** Phase 110.0 — backend's next internal-event deadline in
     *  milliseconds from now (lease keepalive, heartbeat, reader
     *  ACK-NACK timeout, etc.). The runtime caps its `drive_io`
     *  timeout against `min(user_timeout, timer_deadline, this)` so
     *  quiet links don't wake early, see no user-visible work, and
     *  round-trip back into `drive_io`.
     *
     *  Phase 376 W3.d step A — `*out_ms` carries the value and
     *  `*has_deadline` whether there is one; both are written only
     *  on `NROS_RMW_RET_OK`.
     *
     *  This slot was the ONE member of the eleven that step B's
     *  renumbering did not force: its old negative return was a
     *  "no deadline" SENTINEL, not an error code, so nothing would
     *  have collided. It is converted anyway because it had the
     *  shape every other conversion found a silent failure in — a
     *  backend that FAILED to compute its deadline returned `-1`
     *  and was read as "quiet link", which is exactly the reading
     *  that makes the executor sleep longer. It now has an error
     *  channel it never had.
     *
     *  NULL function pointer is permitted — the runtime treats it
     *  the same as `*has_deadline = false`. */
    rmw_ret_t (*next_deadline_ms)(const rmw_session_t *session,
        uint32_t *out_ms, bool *has_deadline);

    /** Phase 124.B.1 — executor wake callback.
     *
     *  The runtime calls this once per session right after `open`
     *  with `cb` pointing at a runtime-supplied function and `ctx`
     *  pointing at the executor's wake state. The backend stores
     *  both in its per-session state and calls `cb(ctx)` whenever
     *  its transport-notification path fires — datagram arrival,
     *  condvar wake-up, select-fd ready, etc. The runtime cb does
     *  flag-write + condvar-signal atomically so a `spin_once`
     *  blocked on the wake condvar resumes immediately.
     *
     *  `cb == NULL` clears any previously installed callback; the
     *  backend must drop the stored (cb, ctx) and never invoke
     *  again after this returns.
     *
     *  NULL slot = backend has no asynchronous wake path (purely
     *  poll-driven: XRCE, bare-metal). The runtime still drains the
     *  session on its deadline-bound cv-wait boundary. */
    rmw_ret_t (*set_wake_callback)(rmw_session_t *session,
                                         void (*cb)(void *ctx),
                                         void *ctx);

    /** Phase 124.A — zero-copy publisher loan.
     *
     *  Reserve a writable slot of at least `requested_len` bytes inside
     *  the backend's outbound buffer. Returns:
     *    * `NROS_RMW_RET_OK` + writes `*out_buf` / `*out_cap` / `*out_token`.
     *    * `NROS_RMW_RET_WOULD_BLOCK` if the backend has no slot
     *      available (caller may retry or fall back to a copy path).
     *      This line said `TRY_AGAIN` until issue 1126 — a spelling
     *      `rmw_ret.h` has never defined, in the header that
     *      IS the ABI contract a third-party backend writes against.
     *      The runtime reads `WOULD_BLOCK` (and `NO_DATA`) as "no slot,
     *      not an error"; every other code propagates as a failure.
     *    * `NROS_RMW_RET_INVALID_ARGUMENT` on bad pointers / size.
     *
     *  `*out_cap` may exceed `requested_len`. The slot's bytes are
     *  valid until the matching `pub_commit` or `pub_discard` runs.
     *  `*out_token` is an opaque per-loan handle the backend uses to
     *  match commit / discard back to the right slot.
     *
     *  NULL function pointer = backend doesn't natively lend; the
     *  runtime falls back to a per-publisher staging arena and emits
     *  a single memcpy on commit. */
    rmw_ret_t (*borrow_loaned_message)(const rmw_publisher_t *publisher,
                                size_t                 requested_len,
                                rmw_mut_byte_span_t   *out_slot,
                                rmw_loan_token_t     **out_token);

    /** Phase 124.A — commit a previously loaned slot.
     *
     *  `token` MUST be a value returned from a prior `pub_loan` on the
     *  same publisher. `actual_len` is the byte count actually
     *  written into the slot (≤ the loan's `out_cap`). Triggers the
     *  wire send.
     *
     *  NULL = paired NULL with `pub_loan`. */
    rmw_ret_t (*publish_loaned_message)(const rmw_publisher_t *publisher,
                                  rmw_loan_token_t     *token,
                                  size_t                actual_len);

    /** Phase 124.A — abandon a previously loaned slot.
     *
     *  Releases the slot without sending. `token` MUST be a value
     *  returned from a prior `pub_loan` on the same publisher.
     *
     *  NULL = paired NULL with `pub_loan`. */
    rmw_ret_t (*return_loaned_message_from_publisher)(const rmw_publisher_t *publisher, rmw_loan_token_t *token);

    /** Phase 124.A — zero-copy subscription borrow.
     *
     *  Borrow a read-only view of the next available message in
     *  place, without copying into a caller buffer. Returns:
     *    * `>= 0` — message length; writes `*out_buf` / `*out_token`.
     *    * `0` — no message ready (subscription empty).
     *    * `< 0` — error (see `rmw_ret_t` codes negated).
     *
     *  The view is valid until the matching `sub_release` runs.
     *  Only one borrow may be outstanding per subscription at a time —
     *  callers MUST release before requesting another borrow.
     *
     *  NULL function pointer = backend doesn't natively borrow; the
     *  runtime falls back to `try_recv_raw` into a staging buffer. */
    /** Upstream `rmw_take_loaned_message`. Phase 376 W3.b/W3.d step A.
     *
     *  `*taken` says whether a view was handed out; `*out_buf`,
     *  `*out_len` and `*out_token` describe it and are meaningful
     *  only when taken. All are written only on
     *  `NROS_RMW_RET_OK`.
     *
     *  Before this, the length was returned AND written to
     *  `*out_len`, and the runtime used the return — so a backend
     *  that disagreed with itself had one of its two answers
     *  silently ignored. There is now one length.
     *
     *  Deviation from upstream, declared: upstream loans a typed
     *  `void **loaned_message`; ours is a byte view plus an opaque
     *  token to release, because there is no typesupport on target
     *  and the backend owns the buffer until `sub_release`.
     *
     *  **No backend fills this slot today** (Cyclone NULL, XRCE NULL, and the
     *  Rust adapter leaves it at `EMPTY_VTABLE`), so every `try_borrow` through
     *  the C ABI takes the copy fallback. The slot is carried, not deleted,
     *  because it is the only shape that can hand a view to a caller which
     *  outlives the call — `nros-c` / `nros-cpp` `try_borrow` — where
     *  `process_raw_in_place`'s scoped callback cannot reach. Zenoh's native
     *  zero-copy receive is live but arrives through Rust
     *  `SlotBorrowing for ZenohSubscriber`, not through here. Recorded so
     *  "the slot exists" is not read as "the capability works": issue 0781. */
    rmw_ret_t (*take_loaned_message)(const rmw_subscription_t *subscription,
                           rmw_byte_span_t       *out_view,
                           rmw_loan_token_t     **out_token,
                           bool                  *taken);

    /** Phase 124.A — release a previously borrowed view.
     *
     *  `token` MUST be a value returned from a prior `sub_borrow`
     *  on the same subscription. Lets the next message advance into
     *  the buffer.
     *
     *  NULL = paired NULL with `sub_borrow`. */
    rmw_ret_t (*return_loaned_message_from_subscription)(const rmw_subscription_t *subscription, rmw_loan_token_t *token);

    /** Phase 124.C.1 — service-server availability probe.
     *
     *  Returns `1` if ≥ 1 matching server has been discovered on the
     *  RMW graph, `0` if none yet, or a negative `rmw_ret_t`
     *  constant on backend error. The runtime exposes this to user
     *  code as `nros_client_server_available()` /
     *  `Client<S>::server_available()` — clients use it to gate the
     *  first request so a startup-ordering race doesn't surface as
     *  a request-side timeout.
     *
     *  Implementation notes per backend:
     *  - **Zenoh**: `z_session` tracks matched queryables via
     *    interest declarations.
     *  - **Cyclone DDS / dust-DDS**: built-in topic readers expose
     *    matched-pub counts.
     *  - **XRCE**: agent has no participant enumeration; return
     *    `NROS_RMW_RET_UNSUPPORTED`.
     *
     *  NULL function pointer = backend cannot answer; the runtime
     *  surfaces `NROS_RMW_RET_UNSUPPORTED` to the caller.
     *
     *  Phase 376 W3.d step A — upstream's shape: the STATUS is the
     *  return value and the answer is an out-parameter. Previously
     *  this slot multiplexed both through one `int32_t` (1 = yes,
     *  0 = no, negative = error), which is what makes upstream's
     *  positive `RMW_RET_ERROR = 1` unadoptable — `1` would mean
     *  both "available" and "failed". Splitting them is what lets
     *  step B renumber at all.
     *
     *  A backend writes `*out_available` only on
     *  `NROS_RMW_RET_OK`; on any error the caller's value is
     *  untouched. The old contract's tolerance for "any positive
     *  value other than 1 means available" is gone with the int:
     *  a `bool` has no non-spec value to be lenient about.
     *
     *  Deviation from upstream, declared: no `node` parameter.
     *  `rmw_service_server_is_available` takes both a node and a
     *  client; an image has no node object to pass — the client
     *  reaches its session directly. */
    rmw_ret_t (*service_server_is_available)(
        const rmw_client_t *client,
        bool *out_available);

    /** Phase 124.D.1 — burst-take.
     *
     *  Drains up to `max_msgs` queued messages into a contiguous
     *  caller buffer in a single backend call, avoiding N × vtable
     *  dispatch when a burst-sensor subscription catches up on a
     *  backlog (e.g. a 100 Hz IMU feed polled at 10 Hz).
     *
     *  Storage contract:
     *    * `buf` is a contiguous `max_msgs * per_msg_cap` block.
     *    * The i-th delivered message lives at `buf + i * per_msg_cap`
     *      and has byte length `out_lens[i]`.
     *    * `out_lens` is at least `max_msgs` entries long.
     *
     *  Returns:
     *    * `>= 0` — count of messages taken (0..=max_msgs).
     *    * `< 0` — `rmw_ret_t` error code; partial drains MUST
     *      use the count form, not error-out.
     *
     *  NULL function pointer = backend doesn't natively batch; the
     *  runtime emits a `try_recv_raw` loop fallback in
     *  `CffiSubscriber::try_recv_sequence`. The fallback gives
     *  identical observable behaviour (each call still costs N
     *  vtable hops) but lets user code commit to the batched API. */
    /** Upstream `rmw_take_sequence`. Phase 376 W3.b/W3.d step A —
     *  the COUNT moves to `*taken`, matching upstream's
     *  `size_t *taken`, and the return carries only a status.
     *  `*taken` is written only on `NROS_RMW_RET_OK`; a partial
     *  drain reports what it got rather than erroring.
     *
     *  Issue 0971 — which leaves a question the count alone cannot
     *  answer: WHY the drain stopped. A batch that ends because a
     *  message did not fit `per_msg_cap` returns the same shape as
     *  one that drained the reader, and the message that stopped it
     *  is consumed — deliberately, for the reason the single take
     *  consumes it too: a sample left behind that no caller can
     *  ever take is a stuck subscription
     *  (`nros-verification`'s `try_recv_post_fix` /
     *  `no_silent_truncation`).
     *
     *  So a backend that stops a drain for a reason the caller must
     *  hear PARKS that status on the subscription and returns it
     *  from the NEXT `take` or `take_sequence`, which takes nothing
     *  else that call. That rule is what makes the fallback note
     *  above true rather than aspirational: without it the runtime's
     *  `try_recv_raw` loop and a native batch answer the same
     *  condition differently — the loop erroring out and discarding
     *  the count it had already earned, the native path reporting a
     *  count and no reason. */
    rmw_ret_t (*take_sequence)(const rmw_subscription_t *subscription,
                                  uint8_t              *buf,
                                  size_t                per_msg_cap,
                                  size_t                max_msgs,
                                  size_t               *out_lens,
                                  size_t               *taken);

    /** Phase 124.E.1 — streamed publish.
     *
     *  Caller hands the backend two callbacks. The backend invokes
     *  `size_cb` once to learn the total payload length, allocates
     *  a single slot of that size in its outbound buffer, then
     *  invokes `chunk_cb` repeatedly to fill the slot in chunks
     *  until the buffer is full. What it saves is the CALLER-side
     *  buffer holding a whole serialised message.
     *
     *  That justification used to read "saves the per-publisher
     *  staging buffer … where the staging buffer dominates `.bss`",
     *  which described a design this tree does not have (issue 0782).
     *  There is no per-publisher buffer and none of it is in `.bss`:
     *  `EmbeddedPublisher::publish` serialises into a per-CALL STACK
     *  array of `DEFAULT_TX_BUF` (= `NROS_SUBSCRIPTION_BUFFER_SIZE`,
     *  1024 by default), and the runtime's own NULL-slot fallback
     *  stages into a 4 KiB stack array and refuses anything larger.
     *  The saving is real and it is STACK — which on an MCU with
     *  small per-task stacks is the tighter budget of the two — but a
     *  reason has to name the thing it is about.
     *
     *  Callback contract:
     *    * `size_cb(*out_total_len, user_ctx)` — write the exact
     *      total payload length, in bytes, to `*out_total_len`.
     *      Called exactly once per `publish_streamed` invocation.
     *    * `chunk_cb(out_buf, cap, *out_written, user_ctx)` —
     *      write up to `cap` bytes starting at `out_buf`, then
     *      report the count written via `*out_written`. The backend
     *      may call `chunk_cb` repeatedly until the total promised
     *      by `size_cb` has been delivered. `*out_written == 0`
     *      means EOF; the backend tears down the slot.
     *
     *  Lesson from micro-ROS's
     *  `rmw_uros_set_continous_serialization_callbacks`: pass the
     *  callbacks per-call rather than binding them to publisher
     *  state, so different messages on the same publisher can use
     *  different serialisation strategies.
     *
     *  NULL function pointer = backend doesn't stream; the runtime
     *  falls back to a one-shot staging buffer (capped at the
     *  configured `NROS_MAX_STREAM_CHUNK`) + `publish_raw`. */
    rmw_ret_t (*publish_streamed)(
        rmw_publisher_t *publisher,
        void (*size_cb)(size_t *out_total_len, void *user_ctx),
        void (*chunk_cb)(uint8_t *out_buf, size_t cap,
                         size_t *out_written, void *user_ctx),
        void *user_ctx);

    /** Phase 124.F.1 — session-level connectivity probe.
     *
     *  Sends a wire-level round-trip probe ("is the peer / agent /
     *  router still reachable?") and waits up to `timeout_ms` for
     *  a reply. No discovery state required — cheaper than the
     *  service-availability probe (which needs matched-publication
     *  bookkeeping). Lesson from micro-ROS's
     *  `rmw_uros_ping_agent`.
     *
     *  Returns:
     *    * `NROS_RMW_RET_OK` — peer responded within budget.
     *    * `NROS_RMW_RET_TIMEOUT` — no reply before `timeout_ms`.
     *    * `NROS_RMW_RET_UNSUPPORTED` — backend can't probe (DDS
     *      with no participant introspection).
     *    * other negative — backend error.
     *
     *  Implementation notes per backend:
     *  - **Zenoh**: `z_send_ping` (or session keep-alive piggyback).
     *  - **XRCE**: `uxr_ping_agent_session_until_timeout`.
     *  - **DDS**: built-in participant ping if available, else
     *    `RET_UNSUPPORTED`.
     *
     *  NULL function pointer = runtime surfaces
     *  `NROS_RMW_RET_UNSUPPORTED` to the caller. */
    rmw_ret_t (*ping_session)(
        rmw_session_t *session,
        int32_t             timeout_ms);

    /* ---- Phase 231 (RFC-0038) — zero-copy in-place subscription take ---- */

    /** Capability query: does this subscription support process_raw_in_place()?
     *  The runtime consults it once at subscription registration to choose
     *  in-place dispatch over the buffered (copying) path.
     *
     *  `*out_supports` is written only on `NROS_RMW_RET_OK` (Phase 376 W3.d
     *  step A — capability out, status returned).
     *
     *  **The capability is the CONJUNCTION** of this probe answering true and
     *  `process_raw_in_place` being non-NULL. Either alone is unsupported.
     *
     *  Why the probe is not redundant with that nullity (issue 0781 proposed
     *  deleting it, and this is the counterexample that stopped it):
     *  `RustBackendAdapter::<R>::VTABLE` is a `const`, so it installs
     *  `process_raw_in_place` for EVERY `R: RustBackend` — the slot is non-NULL
     *  whatever the backend can do. The Rust-side answer is a runtime `&self`
     *  method (`Subscription::supports_process_in_place`) because
     *  `CffiSubscription` multiplexes over whichever backend registered, so it
     *  cannot become an associated const the vtable initializer could branch
     *  on. Today `nros-rmw-zenoh` says true and `nros-rmw-metadata` takes the
     *  `false` default behind an identically-shaped vtable: two backends, same
     *  nullity, different capability. Deriving from nullity would route
     *  metadata's subscriptions into in-place dispatch and every take would
     *  return `MessageTooLarge` from the trait default.
     *
     *  A C backend that knows its answer at compile time (XRCE says true,
     *  Cyclone leaves both NULL) may express it with nullity alone; the
     *  conjunction makes that spelling correct without making it the only one.
     *
     *  NULL function pointer = treated as unsupported (buffered path). */
    rmw_ret_t (*subscription_supports_in_place)(
        rmw_subscription_t *subscription,
        bool *out_supports);

    /** Borrow one ready message in place: hand its raw CDR bytes to `cb` (with
     *  the opaque `ctx`) for the duration of the call, then release the slot.
     *  `cb` MUST NOT re-enter this subscription's receive.
     *
     *  Deviation from upstream, declared. Avoiding the copy is NOT the reason —
     *  upstream already has a name for that, `rmw_take_loaned_message`, and we
     *  carry it (`take_loaned_message`). What this shape buys is that the
     *  borrow is SCOPED: it ends when `cb` returns, so there is no release
     *  token to hold and nothing a caller can forget. Upstream's loan is
     *  unscoped — a caller who misses
     *  `rmw_return_loaned_message_from_subscription` retires one entry of a
     *  fixed-depth receive ring for good, and a target with no reclaim and no
     *  swap does not get that entry back. That is the RTOS constraint; "no
     *  copy" is a property both shapes share.
     *
     *  Both are carried because they answer different callers: this one serves
     *  dispatch from inside the executor, where the callback frame is the
     *  natural scope, and the loan pair serves `nros-c` / `nros-cpp`, whose
     *  `try_borrow` hands a view back to a caller that outlives the call.
     *
     *  NULL function pointer = unsupported (the runtime uses the buffered
     *  path); see `subscription_supports_in_place` for the conjunction. */
    /** Phase 376 W3.d step A — "did it process one" moves to an
     *  out-parameter and the return is a plain status.
     *
     *  This retires `NROS_RMW_RET_NO_DATA` from this slot: an empty
     *  subscription is `*out_processed = false` with
     *  `NROS_RMW_RET_OK`, which is upstream's `taken = false`
     *  semantics. A sentinel that means "fine, but nothing" is
     *  exactly the shape that makes a status enum ambiguous.
     *
     *  `*out_processed` is written only on OK. */
    rmw_ret_t (*process_raw_in_place)(
        rmw_subscription_t *subscription,
        void                  *ctx,
        void                 (*cb)(void *ctx, rmw_byte_span_t message),
        bool                  *out_processed);
    /* ---- Phase 376 W4 — identity + matched counts (all optional) ---- */

    /** Upstream `rmw_get_implementation_identifier`.
     *
     *  The backend's name, static for the life of the image. A gid is only
     *  comparable with another carrying the same identifier, which matters here
     *  because `nros_rmw_cffi_register_named` admits several backends at once.
     *
     *  RESERVED, and NULL in every backend. **There is no runtime fallback**:
     *  this doc said the runtime answered with the registry name, and no such
     *  code was ever written — nothing in the tree calls this slot, so a NULL
     *  one is not "answered elsewhere", it is unanswerable. Corrected 2026-08-29
     *  (phase-393 W2) after grepping for the promised fallback and finding only
     *  this sentence.
     *
     *  Filling it is NOT what makes the identity correct. The identity that is
     *  load-bearing today is the one a backend stamps into `rmw_gid_t`, because
     *  `rmw_compare_gids_equal` compares that string before the bytes — so what
     *  matters is that a backend has ONE spelling of its name, not that it can
     *  be asked for it. Cyclone's `kImplementationIdentifier` is that shape.
     *
     *  Do not fill this slot to move a counter: `check-rmw-slot-producers`
     *  classifies any slot with a producer as `produced` whether or not
     *  anything reads it, which is exactly the overstatement issue 0800 exists
     *  to catch. It earns a body when a CALLER exists — a bridge image asking
     *  which of two linked backends it is on. */
    const char *(*get_implementation_identifier)(void);

    /** Upstream `rmw_get_serialization_format`.
     *
     *  The backend's wire encoding, as its cross-image identity STRING
     *  (RFC-0088 D2: the `u8` discriminant is assigned per image and means
     *  nothing outside it; the name is what two images can agree on). Static
     *  for the life of the image, like the identifier above.
     *
     *  PRODUCED by every backend since phase-421 W2. It was reserved until
     *  then for a stated reason — "every backend here speaks CDR, nothing
     *  asks, so a body would be parity shape with no reader" — and both halves
     *  of that reason have since stopped being true. uORB's wire is the PX4
     *  struct verbatim (RFC-0011), so it answers `"uorb"` where every other
     *  backend answers `"cdr"`; and `nros_rmw_cffi_register_named` admits
     *  several backends in one image, so a bridge has two sessions whose
     *  formats differ and one compile-time constant cannot describe both.
     *  `CffiSession::serialization_format` is the reader.
     *
     *  This is the ONLY per-session answer. `nros_node::IMAGE_SERIALIZATION_
     *  FORMAT` and the generated `NROS_SERIALIZATION_FORMAT` macro are
     *  compile-time constants and are meaningful only in a single-backend
     *  image; ask the slot whenever the image links more than one.
     *
     *  NULL slot: the runtime answers NULL — it does NOT guess `"cdr"`. A
     *  backend that does not declare its format has not told anyone what it
     *  speaks, and inventing an answer is how the identifier slot's doc came
     *  to promise a fallback nobody had written (corrected phase-393 W2). */
    const char *(*get_serialization_format)(void);

    /** Upstream `rmw_feature_supported`.
     *
     *  Whether the backend populates an optional piece of CONTENT — upstream's
     *  two values both concern message-info sequence numbers. Deliberately not
     *  expressed as slot nullity: a NULL pointer says the backend cannot
     *  perform an OPERATION, which is a different question from whether the
     *  data an implemented operation returns is populated.
     *
     *  NULL slot: the runtime answers `false` for every feature. */
    bool (*feature_supported)(rmw_feature_t feature);

    /** Upstream `rmw_get_gid_for_publisher`. Exact parity.
     *
     *  The backend zero-pads to the full width; see `rmw_gid_t`. */
    rmw_ret_t (*get_gid_for_publisher)(const rmw_publisher_t *publisher,
        rmw_gid_t *gid);

    /** Upstream `rmw_publisher_count_matched_subscriptions`. Exact parity.
     *
     *  Every backend already tracks this to implement liveliness events — see
     *  `service_server_is_available`, which is the same question one entity
     *  over. NULL where a backend has no discovery at all (XRCE). */
    rmw_ret_t (*publisher_count_matched_subscriptions)(
        const rmw_publisher_t *publisher, size_t *subscription_count);

    /** Upstream `rmw_subscription_count_matched_publishers`. Exact parity. */
    rmw_ret_t (*subscription_count_matched_publishers)(
        const rmw_subscription_t *subscription, size_t *publisher_count);

    /* ---- Phase 376 W4 — QoS read-back + clean shutdown (all optional) ---- */

    /** Upstream `rmw_publisher_get_actual_qos`. Exact parity.
     *
     *  We bake the REQUESTED profile and, until now, never read back the
     *  GRANTED one. On DDS the two differ whenever a writer and reader
     *  negotiate, and the difference is exactly what answers "why is nothing
     *  arriving" — so a consumer that cannot ask has to guess.
     *
     *  PARTIAL ANSWERS ARE ALLOWED (W5/B2). A backend that can determine four
     *  policies and not the fifth writes the four it knows and
     *  `*_UNKNOWN` for the fifth, then returns `NROS_RMW_RET_OK`. Until the
     *  policy values took upstream's numbering there was no `UNKNOWN` to write,
     *  so the contract had to be all-or-nothing — a partial answer would have
     *  been indistinguishable from a confident one, and W4 chose
     *  `NROS_RMW_RET_UNSUPPORTED` over lying.
     *
     *  `NROS_RMW_RET_UNSUPPORTED` now means what it says: this backend has no
     *  read-back at all. It is NOT the answer for "I know some of it".
     *
     *  A caller that treats `UNKNOWN` as a value rather than as an absence gets
     *  a wrong comparison, so `rmw_qos_profile_check_compatible` reports it as
     *  a WARNING rather than an incompatibility — upstream's
     *  `RMW_QOS_COMPATIBILITY_WARNING`, which was unreachable here until there
     *  was a sentinel to trigger it.
     *
     *  Six upstream entry points, six slots, deliberately: the name rule is
     *  mechanical so that no alias table has to be authored and kept true.
     *  Backends share ONE helper and write six one-line thunks — sharing an
     *  implementation is free, sharing an ABI slot is not. */
    rmw_ret_t (*publisher_get_actual_qos)(const rmw_publisher_t *publisher,
        rmw_qos_profile_t *qos);

    /** Upstream `rmw_subscription_get_actual_qos`. Exact parity. */
    rmw_ret_t (*subscription_get_actual_qos)(const rmw_subscription_t *subscription,
        rmw_qos_profile_t *qos);

    /** Upstream `rmw_client_request_publisher_get_actual_qos`. Exact parity.
     *
     *  The four service/client read-backs carry information available NOWHERE
     *  else: `rmw_client_t` and `rmw_service_t` have no `qos` field, and
     *  `create_client` / `create_service` take ONE profile for both
     *  directions, so the granted per-direction profile is otherwise
     *  unobservable. */
    rmw_ret_t (*client_request_publisher_get_actual_qos)(const rmw_client_t *client,
        rmw_qos_profile_t *qos);

    /** Upstream `rmw_client_response_subscription_get_actual_qos`. */
    rmw_ret_t (*client_response_subscription_get_actual_qos)(const rmw_client_t *client,
        rmw_qos_profile_t *qos);

    /** Upstream `rmw_service_request_subscription_get_actual_qos`. */
    rmw_ret_t (*service_request_subscription_get_actual_qos)(const rmw_service_t *service,
        rmw_qos_profile_t *qos);

    /** Upstream `rmw_service_response_publisher_get_actual_qos`. */
    rmw_ret_t (*service_response_publisher_get_actual_qos)(const rmw_service_t *service,
        rmw_qos_profile_t *qos);

    /** Upstream `rmw_publisher_wait_for_all_acked`.
     *
     *  Blocks until every sample this publisher sent has been acknowledged, or
     *  the timeout elapses. Without it an image that publishes and then halts
     *  cannot know whether anything left the box.
     *
     *  Deviation from upstream, declared: `uint32_t timeout_ms` for upstream's
     *  by-value `rmw_time_t`. Every duration in this ABI is u32 milliseconds
     *  (issue 0241) — one width, one unit, no per-call struct.
     *
     *  Best-effort backends (zenoh best-effort, XRCE) leave this NULL. */
    rmw_ret_t (*publisher_wait_for_all_acked)(const rmw_publisher_t *publisher,
        uint32_t timeout_ms);

    /* ---- Phase 376 W4 — with-info takes + entity callbacks (optional) ---- */

    /** Upstream `rmw_take_with_info`.
     *
     *  `take` plus the sample's metadata, written to caller-owned storage. See
     *  `rmw_message_info_t` for why this is a pointer parameter rather than the
     *  side table the runtime uses today.
     *
     *  Deviations from upstream, declared: the same two `take` declares —
     *  bytes (`buf`/`buf_len`/`*out_len`) instead of a typed `void *`, because
     *  there is no typesupport on target; and no allocation argument, because
     *  nothing in this ABI can produce upstream's opaque
     *  `rmw_subscription_allocation_t` — see `take`, which carries the full
     *  reason and the two wrong ones that preceded it.
     *
     *  NULL slot: the runtime falls back to `take`, and the caller gets no
     *  metadata — which is exactly today's behaviour for every C backend. */
    rmw_ret_t (*take_with_info)(const rmw_subscription_t *subscription,
        rmw_mut_byte_span_t *message, bool *taken, rmw_message_info_t *message_info);

    /** Upstream `rmw_take_loaned_message_with_info`.
     *
     *  `take_loaned_message` plus metadata; same deviations as that slot (a
     *  byte view and an opaque release token rather than a typed loan). */
    rmw_ret_t (*take_loaned_message_with_info)(const rmw_subscription_t *subscription,
        rmw_byte_span_t *out_view, rmw_loan_token_t **out_token,
        bool *taken, rmw_message_info_t *message_info);

    /* ---- Phase 376 W4 — graph introspection (all optional) ----
     *
     * NONE of these may block on the wire, and none takes a timeout: this
     * vtable's premise is that no background transport thread is assumed, so a
     * query-based backend keeps a `drive_io`-fed cache or leaves the slot NULL
     * rather than stalling the executor's only thread inside an introspection
     * call.
     *
     * A 128 KiB target cannot hold a graph cache, and NULL is the expected
     * answer there — the runtime surfaces UNSUPPORTED. XRCE cannot enumerate
     * participants at all.
     */

    /** Upstream `rmw_get_node_names` AND `rmw_get_node_names_with_enclaves`.
     *
     *  One slot, two upstream names: upstream split them only because appending
     *  to a fixed out-parameter list would have broken its ABI. A visitor has
     *  no such list, so the enclave is simply a fourth argument, NULL where
     *  untracked. Recorded in the checker's grouping table. */
    rmw_ret_t (*get_node_names)(const rmw_session_t *session,
        rmw_node_visitor_t visitor);

    /** Upstream `rmw_get_topic_names_and_types`. */
    rmw_ret_t (*get_topic_names_and_types)(const rmw_session_t *session,
        bool no_demangle, rmw_names_and_types_visitor_t visitor);

    /** Upstream `rmw_get_service_names_and_types`. */
    rmw_ret_t (*get_service_names_and_types)(const rmw_session_t *session,
        rmw_names_and_types_visitor_t visitor);

    /** Upstream `rmw_get_publisher_names_and_types_by_node`. */
    rmw_ret_t (*get_publisher_names_and_types_by_node)(const rmw_session_t *session,
        const char *node_name, const char *node_namespace, bool no_demangle,
        rmw_names_and_types_visitor_t visitor);

    /** Upstream `rmw_get_subscriber_names_and_types_by_node`. */
    rmw_ret_t (*get_subscriber_names_and_types_by_node)(const rmw_session_t *session,
        const char *node_name, const char *node_namespace, bool no_demangle,
        rmw_names_and_types_visitor_t visitor);

    /** Upstream `rmw_get_service_names_and_types_by_node`. */
    rmw_ret_t (*get_service_names_and_types_by_node)(const rmw_session_t *session,
        const char *node_name, const char *node_namespace,
        rmw_names_and_types_visitor_t visitor);

    /** Upstream `rmw_get_client_names_and_types_by_node`. */
    rmw_ret_t (*get_client_names_and_types_by_node)(const rmw_session_t *session,
        const char *node_name, const char *node_namespace,
        rmw_names_and_types_visitor_t visitor);

    /** Upstream `rmw_get_publishers_info_by_topic`. */
    rmw_ret_t (*get_publishers_info_by_topic)(const rmw_session_t *session,
        const char *topic_name, bool no_mangle,
        rmw_topic_endpoint_info_visitor_t visitor);

    /** Upstream `rmw_get_subscriptions_info_by_topic`. */
    rmw_ret_t (*get_subscriptions_info_by_topic)(const rmw_session_t *session,
        const char *topic_name, bool no_mangle,
        rmw_topic_endpoint_info_visitor_t visitor);

    /** Upstream `rmw_count_publishers`. */
    rmw_ret_t (*count_publishers)(const rmw_session_t *session,
        const char *topic_name, size_t *count);

    /** Upstream `rmw_count_subscribers`. */
    rmw_ret_t (*count_subscribers)(const rmw_session_t *session,
        const char *topic_name, size_t *count);

    /** Upstream `rmw_node_get_graph_guard_condition`.
     *
     *  Registers a callback fired when the graph CHANGES. Upstream returns a
     *  guard condition the caller adds to a wait set; we have no wait set to
     *  add it to, and guard conditions are an executor concept here, so this is
     *  the `set_wake_callback` shape instead — the one guard condition whose
     *  trigger is genuinely backend knowledge.
     *
     *  The callback is an EDGE, carrying no payload: delivering WHAT changed
     *  would mean buffering it, which is the graph cache a small target cannot
     *  afford.
     *
     *  Named after upstream mechanically, per the campaign's rule, but the
     *  honest name for this shape is `set_on_graph_change_callback` — flagged
     *  for W5 rather than decided quietly here. */
    rmw_ret_t (*node_get_graph_guard_condition)(rmw_session_t *session,
        rmw_event_callback_t callback, const void *user_data);

    /* ---- Phase 376 W4 — graph node lifecycle (optional) ----
     *
     * The rest of upstream's lifecycle group is deliberately absent, and the
     * absences are decisions rather than omissions:
     *
     *  - `rmw_init` / `rmw_shutdown` / `rmw_context_fini` are
     *    `create_session` / `destroy_session`. There is no second teardown
     *    phase: the session shell is caller-owned, so there is nothing left to
     *    free after the backend releases `backend_data`.
     *
     *  - `rmw_init_options_{init,copy,fini}` have nothing to do here. Upstream
     *    needs the trio because its options OWN heap and carry an
     *    `rcutils_allocator_t`, which cannot cross this seam; ours is a
     *    build-time POD, so "copy" is `=` and "fini" is nothing.
     *
     *    This does NOT decide what the options CARRY. Of Humble's eight
     *    fields, `create_session` takes `domain_id`; `implementation_identifier`
     *    and `impl` are answered elsewhere (the identity slot,
     *    `rmw_session_t::backend_data`); `allocator` is declined ABI-wide and
     *    `instance_id` is rcl-side process identity. `security_options` is
     *    declined on the target: it is a DDS-SROS2 keystore PATH plus an
     *    enforcement switch, and neither a filesystem nor a security plugin
     *    exists where this ABI runs. `localhost_only` and `enclave` are real
     *    GAPS — issue 0785, to be carried by whatever shape issue 0808 gives
     *    backend-private session config, because this flat argument list
     *    cannot grow either without another ABI break.
     *
     *    `discovery_options` is NOT in this list: it is an IRON field, and the
     *    recorded contract is Humble. It was named here until issue 0785, the
     *    third unchecked sentence about an upstream struct in this campaign
     *    (issue 0777 supplied the other two). Read the header, not the memory.
     *
     *  - `rmw_wait`, the wait set, and the guard conditions are declined
     *    together, because `has_data` / `has_request` + `drive_io` +
     *    `set_wake_callback` + `next_deadline_ms` ARE upstream's `rmw_wait`,
     *    decomposed. What a vtable `wait` would add is only the BLOCK, moved
     *    from the platform into the backend — and the block cannot live there:
     *    one executor drives sessions from several backends at once
     *    (`create_node_on(name, rmw)`), timers fire off the platform clock, and
     *    guard conditions fire from another thread or an ISR. A backend can
     *    only block on its own handles. `next_deadline_ms` is exactly the piece
     *    of `rmw_wait` that CANNOT be answered above the seam, and it is
     *    already a slot; that is the decomposition working, not a gap.
     */

    /** Upstream `rmw_create_node`.
     *
     *  Declares a node on the graph. NULL slot is the expected implementation
     *  in a static image: the runtime still tracks the node, the backend simply
     *  has nothing to declare.
     *
     *  Deviations from upstream, declared: no `rmw_context_t *` (an image has
     *  one session and reaches it directly), and the node is an OUT parameter
     *  rather than a returned pointer — no runtime allocation, the caller owns
     *  the storage, exactly as `create_publisher` does.
     *
     *  The runtime calls this once per distinct `(name, namespace_)`. */
    rmw_ret_t (*create_node)(rmw_session_t *session,
        const char *name, const char *namespace_,
        rmw_node_t *out);

    /** Upstream `rmw_destroy_node`.
     *
     *  Releases the backend's `backend_data`; the shell stays valid until its
     *  owner drops it. Called for every node the session created, from
     *  `close()`, BEFORE `destroy_session` — a backend's node state hangs off
     *  its session state, so the order is not free.
     *
     *  NULL slot: nothing is called and the runtime simply forgets the node.
     *  Correct only for a backend that allocates nothing in `create_node`;
     *  a backend that fills `backend_data` there must fill this too, or it
     *  leaks one node's state per session close (issue 0800 — this slot had a
     *  producer in no backend and a consumer nowhere, so the leak was
     *  indistinguishable from an optional slot nobody needed). */
    rmw_ret_t (*destroy_node)(rmw_node_t *node);

    /** Upstream `rmw_set_log_severity`. Exact parity.
     *
     *  Sets the verbosity of the BACKEND's own logging — Cyclone's `dds_log`,
     *  zenoh-pico's log, the XRCE client's. Not `nros_log`: that is the
     *  runtime's logger, already runtime-settable through
     *  `nros_log::Logger::set_level`, and it needs no ABI to reach.
     *
     *  This IS a slot rather than a plain ABI function, and the distinction is
     *  the one the pure functions turn on: "what does this middleware print"
     *  genuinely varies by middleware, so a per-backend answer is correct here
     *  rather than a defect. All three reference implementations
     *  (`librmw_{cyclonedds,fastrtps,zenoh}_cpp.so`) implement it with real
     *  bodies, which is the opposite of what they do for
     *  `rmw_get_serialized_message_size`.
     *
     *  Phase 376 W5 — this was DECLINED, on the reasoning that "log level is a
     *  build-time constant (nros_log); a runtime setter implies a mutable
     *  global". Both clauses were false: `Logger::level` is an `AtomicU8` with a
     *  public `set_level` already used by tests, and the compile-time part is a
     *  CEILING that defaults open. The decline described a design we do not
     *  have.
     *
     *  NULL slot: the backend has no adjustable logging, and the runtime
     *  surfaces `UNSUPPORTED`. */
    rmw_ret_t (*set_log_severity)(rmw_log_severity_t severity);

    /* ---- Phase 403 W1 — receive-buffer sizing (optional) ---- */

    /** How many bytes of TAKE buffer this type actually needs, so the runtime
     *  can stop sizing `take`'s `buf` from one global constant.
     *
     *  RTOS addition; upstream has no counterpart. Upstream's nearest thing is
     *  `rmw_get_serialized_message_size`, which is about a MESSAGE and which
     *  all three reference implementations leave unimplemented; this is about
     *  the buffer the CALLER must present, and the answer is the backend's
     *  because only the backend knows what its framing, size classes and
     *  attachment handling add on top of the payload.
     *
     *  `type_name` and `type_hash` name the type (both BORROWED for the call;
     *  `type_hash` may be NULL exactly as in `create_subscription`). `hint` is
     *  the same number the runtime would put in
     *  `rmw_subscription_options_t.rx_buffer_hint`, and carries the same
     *  meaning, including that `0` says the CALLER stated nothing rather than
     *  that the type is unbounded — every message type has a derived bound or
     *  the build fails. On `NROS_RMW_RET_OK`, `*out_bytes` is the MINIMUM
     *  take-buffer length that is sufficient for this type at this hint: a
     *  `take` given that many bytes must not fail for want of room, and no
     *  smaller number has that property. It may exceed `hint` — a backend that
     *  frames or pads says so here rather than discovering it at `take` time —
     *  and it may be smaller.
     *
     *  MINIMUM, tightened in phase-403 W4, and the word carries the whole
     *  value of the slot. A backend that keeps size CLASSES may not answer with
     *  the class it would round this type up to. Doing so is arithmetically
     *  safe and useless: a 68-byte type and a 1000-byte type sharing a class
     *  come back with one number, which is the global-constant answer the
     *  runtime already had without asking, and the runtime would spend the
     *  difference on every subscription in the image. Report what the type
     *  needs; the rounding is the backend's own business and stays there.
     *  If a backend genuinely cannot separate the two — its framing really
     *  does make the class size the floor — then the class size IS the
     *  minimum and answering it is correct; what is forbidden is reporting a
     *  rounding as though it were a requirement.
     *
     *  This is a QUERY about a type, not about an entity: it is answerable
     *  before any subscription exists, which is the point — the runtime has to
     *  size the buffer in order to create one.
     *
     *  NULL slot: FALLBACK class (RFC-0035's NULL-slot contract) — THE HINT IS
     *  THE ANSWER, never `UNSUPPORTED` to the caller. The runtime uses `hint`,
     *  and since a type without a derived bound is a build error, that is a
     *  real number rather than a fallback to a configured default. A backend
     *  that cannot answer for a PARTICULAR type returns
     *  `NROS_RMW_RET_UNSUPPORTED` and leaves `*out_bytes` untouched; the
     *  runtime falls back to the hint exactly as for a NULL slot.
     *
     *  OPTIONAL on the merits, not for compatibility. nano-ros is unreleased
     *  and this ABI may be broken, so "a mandatory slot breaks every
     *  out-of-tree backend" is not the reason and must not be cited as one.
     *  Three reasons that survive without it:
     *
     *  - A SLOT CANNOT BE REQUIRED BEFORE SOMETHING DISPATCHES IT. Required
     *    here means `first_missing_vtable_slot` REFUSES to register a backend
     *    that leaves it NULL, and `check-rmw-required-slots.sh` holds that set
     *    equal to the set the runtime `.expect()`s. Nothing calls this yet —
     *    phase-403 W3/W5 own the dispatch site — so requiring it now would
     *    refuse working backends over a function no caller reaches. That is
     *    issue 0349 exactly, and it cost three backends their registration
     *    once. (W4 filled the slot for zenoh-pico, which makes the slot
     *    PRODUCED; it is the CONSUMER that decides whether required is
     *    reachable, and there is still none.)
     *  - "NO OPINION" IS A REAL ANSWER, and mandatory does not delete it, only
     *    relocates it: five in-tree backends would each carry the same
     *    `*out_bytes = hint; return OK;` body, and the Rust ones would get it
     *    from a defaulted `RustBackend` trait method — the same special case,
     *    one layer up and less visible.
     *  - IT IS SLOT 75, AND TWO BACKENDS INITIALISE POSITIONALLY. uORB's
     *    C++14 initialiser stops at slot 17 and positional initialisation
     *    cannot skip, so reaching a mandatory slot 75 means writing 58
     *    meaningless entries to get there.
     *
     *  Promotion stays cheap and stays open: making this required later is a
     *  change to the registration check, not to the struct.
     *
     *  DECIDED 2026-08-31: this slot stays OPTIONAL permanently. Phase-403 W4
     *  recommended it after filling the slot for zenoh-pico, and that is now a
     *  ruling rather than a recommendation, so promotion is off the table
     *  unless the reason below stops being true.
     *  The first argument above dissolves once a dispatch site exists, but the
     *  other two do not, and they are the load-bearing ones: cyclonedds and
     *  XRCE keep ONE receive buffer, so "no opinion" is their true answer and
     *  requiring the slot only relocates it into identical bodies; and slot 75
     *  is out of reach of uORB's positional C++14 initialiser whatever the
     *  registration check says. A slot that most backends must fill with a
     *  restatement of the default is worse than a NULL whose meaning the
     *  header pins down, which this one does.
     *
     *  It does not weaken `take`'s obligation. The runtime may pass a `buf_len`
     *  smaller than what this returned — it has its own memory to answer to —
     *  and a sample that does not fit is still a reported failure rather than
     *  a truncated success. Answering here is how a backend avoids that
     *  outcome, not how it licenses one. */
    rmw_ret_t (*required_rx_bytes)(const char *type_name,
        const char *type_hash, size_t hint, size_t *out_bytes);

} nros_rmw_vtable_t;

/**
 * Session mode for `create_session`'s @p mode parameter (issue 0331).
 *
 * These values were previously an undocumented bare `uint8_t` with no legal-
 * value list — the only slot in the vtable without one — encoded inline as
 * `0u8` / `1u8` at the Rust boundary.
 *
 * Divergence from `rmw.h`, recorded deliberately: Humble's
 * `rmw_init_options_t` carries `instance_id`, `implementation_identifier`,
 * `domain_id`, `security_options`, `localhost_only`, `enclave`, `allocator`
 * and `impl` — and has NO session-mode concept. (This list said "domain_id,
 * enclave, security_options and discovery_options" until 2026-08-24, which
 * omitted five of the eight and named one, `discovery_options`, that is an
 * IRON field and does not exist in the distro our recorded contract is taken
 * from. What we do and do not carry from those eight is issue 0785.) This parameter is closest to zenoh's `whatami`, and a
 * backend that has no such notion (cyclonedds, XRCE) is expected to IGNORE it
 * rather than fail. Folding it into backend-private config behind the locator
 * — so the agnostic vtable stops carrying a backend-shaped field — is the
 * structural fix, and is not done here; see issue 0808. (This said "see issue
 * 0331" until 2026-08-26. That issue documented this enum and stated in its own
 * resolution that the structural half was NOT done, deferring it to issue 0330
 * part 3 — which resolved a different concern entirely. Both are closed, so the
 * fold was tracked by nothing; 0808 is its home.)
 */
typedef enum nros_rmw_session_mode_t {
    /** Connect to a router/agent as a client. The default. */
    NROS_RMW_SESSION_MODE_CLIENT = 0,
    /** Peer-to-peer, no router. Backends without a peer mode ignore this. */
    NROS_RMW_SESSION_MODE_PEER = 1,
} nros_rmw_session_mode_t;

/** Register a custom RMW backend under the implicit name "default".
 *  Legacy single-arg form retained for source compatibility with
 *  backend ctors authored before the named registry (Phase 104.B.2).
 *
 *  Deprecated (Phase 128.B.5): every in-tree backend now calls
 *  `nros_rmw_cffi_register_named` with its canonical name. The
 *  unnamed shim will be removed in a follow-up phase.
 *  Returns NROS_RMW_RET_OK. */
rmw_ret_t nros_rmw_cffi_register(const nros_rmw_vtable_t *vtable);

/** Phase 104.B.2 — register a backend under a stable name. Multiple
 *  backends can coexist (bridge nodes); consumers select via
 *  `nros_rmw_cffi_lookup` or the higher-level
 *  `Executor::node_builder(...).rmw(...)` path.
 *
 *  Names: UTF-8, NUL-terminated, ≤ 31 bytes (excluding NUL).
 *  Reserved: "zenoh", "dds", "xrce", "cyclonedds", future "uorb".
 *  "default" is the implicit name used by `nros_rmw_cffi_register`.
 *
 *  Duplicate registration of the same name overwrites the previous
 *  vtable (idempotent for ctor-fires-twice).
 *
 *  Returns:
 *    * NROS_RMW_RET_OK on success.
 *    * NROS_RMW_RET_INVALID_ARGUMENT if name or vtable is NULL,
 *      the name is empty, or exceeds 31 bytes.
 *    * NROS_RMW_RET_ERROR if the registry is full
 *      (NROS_RMW_MAX_BACKENDS reached). */
rmw_ret_t nros_rmw_cffi_register_named(const char *name,
                                            const nros_rmw_vtable_t *vtable);

/** Look up a backend's vtable by name. Returns NULL if no backend is
 *  registered under `name`. The returned pointer is valid for the
 *  program's lifetime. */
const nros_rmw_vtable_t *nros_rmw_cffi_lookup(const char *name);

/** Diagnostic helper — fills `buf` with pointers to up to `cap`
 *  registered backend names. Returns the total number of registered
 *  backends (may exceed `cap`; caller can re-query with a larger
 *  buffer). Pointer-valid for the program's lifetime. Pass
 *  `buf=NULL, cap=0` to query the count only. */
size_t nros_rmw_cffi_registered_names(const char **buf, size_t cap);


/** Phase 249 P4b.2 — convenience macro for static-library backends.
 *  Place in exactly one TU per backend to self-register the backend
 *  on library load. `REGISTER_FN` is a no-arg function that calls
 *  `nros_rmw_cffi_register_named` for the backend.
 *
 *  Example:
 *      static void zenoh_register(void) {
 *          nros_rmw_cffi_register_named("zenoh", &VTABLE);
 *      }
 *      NROS_RMW_REGISTER_BACKEND(zenoh_register)
 *
 *  HOSTED (Rust + C/C++ on a hosted loader): the macro expands to an
 *  `.init_array` constructor (`__attribute__((constructor))`) that the
 *  loader fires before `main()` — hence before `nros_support_init` /
 *  `nros::init` — calling `REGISTER_FN`. The `--whole-archive` link
 *  keeps this object's `.init_array` slot. `nros_rmw_cffi_register_named`
 *  is idempotent (same-name overwrite), so re-registration is harmless.
 *  This consolidates the former `linkme` section walk onto the ctor
 *  (RFC-0042 §D3.3; the `linkme` distributed slice / section walker is
 *  deleted).
 *
 *  EMBEDDED (bare-metal / RTOS: Zephyr, NuttX, esp-idf, VxWorks): the
 *  loader does not run `.init_array` constructors on the startup path,
 *  so the macro expands to nothing. Registration is wired explicitly
 *  by the board / typed carrier via an explicit `nros_rmw_<x>_register()`
 *  call (phase-249 P1). For C/C++-via-cmake the `nano_ros_link_rmw`
 *  strong stub is the primary registration trigger (P2b/P4a); this
 *  hosted ctor is belt-and-suspenders.
 *
 *  Gating mirrors the cyclonedds vtable.cpp constructor (off the RTOS
 *  targets) and requires GCC/Clang constructor-attribute support. */
#if (defined(__GNUC__) || defined(__clang__)) && !defined(__ZEPHYR__) &&       \
    !defined(__NuttX__) && !defined(ESP_PLATFORM) && !defined(__VXWORKS__)
#define NROS_RMW_REGISTER_BACKEND(REGISTER_FN)                                 \
    __attribute__((constructor)) static void nros_rmw_ctor_##REGISTER_FN(      \
        void) {                                                                \
        (void) REGISTER_FN();                                                  \
    }
#else
/* Embedded / unsupported toolchain: board calls nros_rmw_<x>_register(). */
#define NROS_RMW_REGISTER_BACKEND(REGISTER_FN)
#endif

#ifdef __cplusplus
}
#endif

#endif /* NROS_RMW_VTABLE_H */
