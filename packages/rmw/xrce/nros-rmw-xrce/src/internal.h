#ifndef NROS_RMW_XRCE_C_INTERNAL_H
#define NROS_RMW_XRCE_C_INTERNAL_H

/* Shared declarations across vtable.c / session.c / publisher.c /
 * subscriber.c / service.c / transport_custom.c.
 *
 * Phase 115.K.2 — backend state lives on the heap (one
 * `xrce_session_state` per session, malloc'd at `open`, parked in
 * `rmw_session_t::backend_data`). Per-entity state lives in slots
 * inside that struct; entity shells get a pointer to the matching slot
 * via their `backend_data` field. Mirrors the design ground truth in
 * `packages/rmw/xrce/nros-rmw-xrce/src/lib.rs` but without the
 * module-static `XrceSessionState` it relies on.
 */

#include "nros/platform.h"
#include "nros/rmw_entity.h"
#include "nros/rmw_event.h"
#include "nros/rmw_ret.h"

#include <uxr/client/client.h>
#include <uxr/client/core/session/common_create_entities.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Issue 0832 — this backend allocates through the platform funnel, never libc.
 *
 * The shim is linked ABOVE the platform layer, so libc's heap is not
 * necessarily the heap the platform owns: on Zephyr, FreeRTOS, ThreadX and
 * NuttX it is a different allocator entirely, and the `unified` tier promises
 * that every allocation reaches `nros_platform_alloc`. A raw `calloc`/`free`
 * here allocates from one heap and, once a board routes the other way, frees
 * to another.
 *
 * The zeroing lives in the helper rather than at the call sites. Every
 * allocation in this backend was `calloc(1, sizeof(X))` on a struct whose
 * fields are then only partly assigned, so the zeroing is load-bearing — and a
 * `memset` that has to be remembered at nine call sites is one that gets
 * forgotten at the tenth.
 */
static inline void* nros_xrce_calloc(size_t count, size_t size) {
    size_t total;
    void* ptr;

    if (count == 0 || size == 0) {
        count = size = 1;
    }
    /* The multiply is the one thing calloc does that alloc cannot, so keep its
       overflow check rather than trusting the product. */
    total = count * size;
    if (total / size != count) {
        return NULL;
    }
    if ((ptr = nros_platform_alloc(total)) != NULL) {
        memset(ptr, 0, total);
    }
    return ptr;
}

static inline void nros_xrce_free(void* ptr) {
    /* `nros_platform_dealloc` is a documented no-op on NULL, matching free(). */
    nros_platform_dealloc(ptr);
}

/* ---- Tunables (must mirror packages/rmw/xrce/nros-rmw-xrce/build.rs
 *      defaults so the C backend behaves the same as the Rust one
 *      under nominal config). ---- */

/* Phase 207.6 — guarded so the xrce-cffi build.rs env knobs
 * (`NROS_XRCE_MAX_SUBSCRIBERS`, `..._MAX_SERVICE_SERVERS`,
 * `..._MAX_SERVICE_CLIENTS`, `..._BUFFER_SIZE`) can shrink the
 * per-session struct on RAM-tight embedded targets. The default
 * `xrce_session_state_t` is ~390 KB; a pub-only bare-metal node can
 * drop it well below 32 KB by setting subscribers/services to 0 and
 * smaller per-entity buffers. */
#ifndef XRCE_MAX_SUBSCRIBERS
#define XRCE_MAX_SUBSCRIBERS 8
#endif
#ifndef XRCE_MAX_SERVICE_SERVERS
#define XRCE_MAX_SERVICE_SERVERS 4
#endif
#ifndef XRCE_MAX_SERVICE_CLIENTS
#define XRCE_MAX_SERVICE_CLIENTS 4
#endif
/* Phase 237 — outstanding deferred replies per service server. A reply may be
 * sent long after `take_request` returned (e.g. an action `get_result` held
 * until the goal terminates), so the request's `SampleIdentity` can't live in
 * the single request inbox (a later request overwrites it). Each in-flight
 * reply gets a seq-keyed token slot instead. Sized for the max concurrent
 * goals; override at build time if a server fields more. */
#ifndef XRCE_MAX_PENDING_REPLIES
#define XRCE_MAX_PENDING_REPLIES 4
#endif
/* Phase 237 follow-up — depth of the service-server request inbox ring. The
 * single inbox dropped a request that arrived before the previous one was
 * drained (two clients sending send_goal / get_result in the same spin window —
 * concurrent goals under load). A ring buffers a burst of arrivals so each is
 * read in order. Override on RAM-tight targets (depth 1 = single-inbox). */
#ifndef XRCE_SERVICE_REQUEST_RING_DEPTH
#define XRCE_SERVICE_REQUEST_RING_DEPTH 4
#endif
#ifndef XRCE_BUFFER_SIZE
#define XRCE_BUFFER_SIZE 1024
#endif
/* Phase 130.4 — bumped default from 4 to 16. Action server callbacks
 * that publish feedback + result + status_array + service replies
 * in a single user-handler invocation could exhaust 4 unACK'd slots
 * before the executor gets a chance to drain ACKs, causing
 * `uxr_buffer_reply` to return UXR_INVALID_REQUEST_ID. Sixteen
 * slots cover a typical action lifecycle (accept + 3-4 feedback
 * + complete + status_array + result) with room to spare. Costs
 * 16 * UXR_CONFIG_CUSTOM_TRANSPORT_MTU = 64 KiB of per-session
 * output stream buffer (was 16 KiB at history=4).
 *
 * Phase 130.6 — tunable via build.rs env override
 * `NROS_XRCE_STREAM_HISTORY=<n>`; defaults to 16 if unset. Tight-
 * RAM RTOS targets can drop to 8 (32 KiB) when the application
 * doesn't run server-side action callbacks that fan out replies.
 * Lower than 4 is rejected — reliable retransmission needs at
 * least 2 unACK'd messages outstanding plus headroom. */
#ifndef XRCE_STREAM_HISTORY
#define XRCE_STREAM_HISTORY 16
#endif
#if XRCE_STREAM_HISTORY < 4
#error "XRCE_STREAM_HISTORY must be >= 4 (reliable retransmit headroom)"
#endif
/* Stream buffer sized after the largest MTU compiled in. Embedded
 * (no_POSIX) builds drop the UDP profile, so fall back to the custom
 * transport MTU. */
#if defined(UCLIENT_PROFILE_UDP)
#define XRCE_STREAM_BUFFER_SIZE (UXR_CONFIG_UDP_TRANSPORT_MTU * XRCE_STREAM_HISTORY)
#else
#define XRCE_STREAM_BUFFER_SIZE (UXR_CONFIG_CUSTOM_TRANSPORT_MTU * XRCE_STREAM_HISTORY)
#endif
#define XRCE_CDR_HEADER_LEN 4
#define XRCE_DDS_NAME_BUF_SIZE 128
#define XRCE_PARTICIPANT_NAME_BUF_SIZE 64
#define XRCE_ENTITY_CREATION_TIMEOUT_MS 1000
#define XRCE_SESSION_FLUSH_TIMEOUT_MS 100
#define XRCE_SESSION_CREATION_RETRIES 3

/* Default agent UDP port, matches Micro-XRCE-DDS-Agent's default. */
#define XRCE_DEFAULT_AGENT_PORT 2018

/* Bounded busy-wait for service replies (ms). */
#define XRCE_SERVICE_REPLY_TIMEOUT_MS 50
#define XRCE_SERVICE_REPLY_TOTAL_MS 5000

/* ---- Per-entity slots ----------------------------------------------- */

/* Subscriber slot — N-deep ringbuffer.
 *
 * Phase 160.H.1 — grew from a single-message buffer to N entries so a
 * back-to-back publish burst doesn't silently overwrite unread
 * messages (root cause of `test_xrce_throughput_{100hz,burst}` only
 * receiving 1 of 100 msgs). Each ring entry carries its own
 * `data[XRCE_BUFFER_SIZE]` + `len`; the topic callback writes to
 * `entries[write_idx]` and advances, `take_serialized` reads from
 * `entries[read_idx]` and advances. `count` distinguishes empty (0)
 * from full (XRCE_SUBSCRIBER_RING_DEPTH). On full, the callback
 * drops the newest message (preserves in-order delivery of buffered
 * msgs); on overflow length the per-entry `overflow` flag is set so
 * `take_serialized` can surface `MESSAGE_TOO_LARGE`.
 *
 * TODO 115.K.2.x: deadline tracking, async wakers. The Rust impl carries
 * `deadline_cb`, `last_msg_at_ms`, etc. Skipped here per K.2 scope.
 */
#ifndef XRCE_SUBSCRIBER_RING_DEPTH
/* Phase 160.H.1 — depth 32 (was bumped 4 → 16 upstream first; raised
 * to 32 after re-testing 100Hz burst behaviour). The XRCE Agent
 * batches all queued outbound messages onto the subscriber's input
 * stream in a single `run_session_time(timeout_ms)` window — the
 * callback fires once per Data frame and the listener can't
 * `take_serialized`-drain in between because they all execute inside
 * the same `uxr_run_session_time` call. So the ring must hold an
 * entire publish burst — anything beyond depth is silently dropped
 * by the topic callback's ring-full policy. The burst test
 * publishes 100 msgs back-to-back; depth 32 caps the visible burst
 * at the head 32 (matches the `test_xrce_throughput_*` expectations
 * of ≥3 / ≥10) without paying the memory cost of a 100+ entry
 * ring. Memory cost: 32 × XRCE_BUFFER_SIZE per subscriber × 8
 * max = 256 KB. Tight-RAM RTOS targets can override via
 * `-DXRCE_SUBSCRIBER_RING_DEPTH=...` at build time. */
#define XRCE_SUBSCRIBER_RING_DEPTH 32
#endif

/* ---- Smallest legal size for the four PER-SLOT arrays (issue 1131) ---
 *
 * BELOW the `#ifndef` fallbacks above, never before one: in `#if` an undefined
 * identifier reads as 0, so a guard placed above its own default fires on every
 * build instead of protecting anything (issue 1167).
 *
 * These four are CAPACITIES INSIDE a slot, and that is what separates them from
 * the three slot COUNTS above, which issue 1033 ruled zero-legal. An image that
 * wants none of an entity sets its COUNT to 0 and the whole slot array goes
 * away — that is the 33,296-bytes-a-subscriber saving. Setting a capacity to 0
 * instead keeps the slots and makes each one unable to do its job, silently:
 *
 *   XRCE_BUFFER_SIZE=0          `xrce_stage_inbound` needs `len + 4 <= cap`, so
 *                               every inbound payload fails to stage and every
 *                               take reports MESSAGE_TOO_LARGE. The only
 *                               producer, `nros-rmw-xrce-cffi/build.rs`, already
 *                               refuses anything under 64.
 *   XRCE_SUBSCRIBER_RING_DEPTH=0  `count >= depth` is true at 0, so the topic
 *                               callback takes its ring-full drop arm for every
 *                               message and the subscriber receives nothing —
 *                               issue 1015's silence exactly. Kconfig already
 *                               says `range 1 1024`.
 *   XRCE_SERVICE_REQUEST_RING_DEPTH=0  same arm in `xrce_request_callback`;
 *                               every request is dropped, and the `% DEPTH` in
 *                               `XRCE_REQ_RING_POP` is one branch away.
 *   XRCE_MAX_PENDING_REPLIES=0  `take_request` finds no free token, returns
 *                               WOULD_BLOCK and leaves the request in the ring,
 *                               so the server can never answer anything.
 */
#if XRCE_BUFFER_SIZE < 1
#error "XRCE_BUFFER_SIZE must be >= 1: it sizes a C array (issue 1015)"
#endif
#if XRCE_SUBSCRIBER_RING_DEPTH < 1
#error "XRCE_SUBSCRIBER_RING_DEPTH must be >= 1: it sizes a C array (issue 1015)"
#endif
#if XRCE_SERVICE_REQUEST_RING_DEPTH < 1
#error "XRCE_SERVICE_REQUEST_RING_DEPTH must be >= 1: it sizes a C array (issue 1015)"
#endif
#if XRCE_MAX_PENDING_REPLIES < 1
#error "XRCE_MAX_PENDING_REPLIES must be >= 1: it sizes a C array (issue 1015)"
#endif

typedef struct xrce_subscriber_ring_entry {
    uint8_t data[XRCE_BUFFER_SIZE];
    size_t len;
    bool overflow;
} xrce_subscriber_ring_entry;

typedef struct xrce_subscriber_slot {
    xrce_subscriber_ring_entry entries[XRCE_SUBSCRIBER_RING_DEPTH];
    uint16_t write_idx;
    uint16_t read_idx;
    uint16_t count;
    /* `locked` mirrors the Rust impl: callbacks observing this drop
     * the message rather than overwriting a buffer mid-read. */
    bool locked;
    uint16_t datareader_id;
    bool active;
} xrce_subscriber_slot;

/* Phase 237 — seq-keyed reply token. Holds the `SampleIdentity` of a received
 * request whose reply may be deferred; `take_request` allocates one and
 * returns its index as the runtime `sequence_number`, `send_response` consumes it. */
typedef struct xrce_reply_token {
    SampleIdentity sample_id;
    bool in_use;
} xrce_reply_token;

/* One buffered request in the service-server inbox ring. */
typedef struct xrce_service_request_entry {
    uint8_t data[XRCE_BUFFER_SIZE];
    size_t len;
    bool overflow;
    SampleIdentity sample_id;
} xrce_service_request_entry;

/* Service-server slot — request inbox RING + deferred-reply token table. */
typedef struct xrce_service_server_slot {
    /* Phase 237 follow-up — SPSC ring of buffered requests (callback produces,
     * `take_request` consumes). `req_count` distinguishes empty (0) from
     * full (depth); on full the callback drops the newest, preserving in-order
     * delivery of the buffered requests. */
    xrce_service_request_entry req_ring[XRCE_SERVICE_REQUEST_RING_DEPTH];
    uint16_t req_write_idx;
    uint16_t req_read_idx;
    uint16_t req_count;
    uint16_t replier_id;
    bool active;
    /* Phase 237 — outstanding replies keyed by index (the runtime `seq`). */
    xrce_reply_token reply_tokens[XRCE_MAX_PENDING_REPLIES];
} xrce_service_server_slot;

/* Service-client slot — reply inbox. */
typedef struct xrce_service_client_slot {
    uint8_t data[XRCE_BUFFER_SIZE];
    size_t len;
    bool has_reply;
    bool overflow;
    uint16_t requester_id;
    /* Issue 0778 — the XRCE request id the buffered reply answers, recorded
     * by `xrce_reply_callback` (which used to `(void)request_id` it away) and
     * reported out of `take_response` as the sequence id. */
    uint16_t reply_request_id;
    bool active;
} xrce_service_client_slot;

/* ---- Per-session state ---------------------------------------------- */

struct xrce_session_state {
    /* Transport — UDP (POSIX builds only) or custom. Only one is
     * live at a time; the mode is captured at open via the locator
     * scheme. Embedded builds drop the UDP profile entirely, so the
     * field is gated to keep the struct size predictable across
     * targets. */
#if defined(UCLIENT_PROFILE_UDP)
    uxrUDPTransport udp;
#endif
    uxrCustomTransport custom;
    bool use_custom_transport;
    /* Phase 115.K.2.5.1.2.a-fix-transport — POSIX UDP via custom
     * transport. `udp_bridge.fd` is set by `xrce_posix_udp_init`
     * and read by the per-session trampolines through
     * `uxrCustomTransport.args`. */
    struct {
        int fd;
        void* sock;
        void* endpoint;
    } udp_bridge;
    /* Phase 115.K.2.5.1.5-serial — POSIX serial transport via
     * custom transport. Same shape as `udp_bridge`: an `int fd`
     * threaded through the trampolines via `uxrCustomTransport.args`. */
    struct {
        int fd;
    } serial_bridge;

    uxrSession session;

    /* Reliable streams. The buffers must outlive the session. */
    uint8_t output_reliable_buf[XRCE_STREAM_BUFFER_SIZE];
    uint8_t input_reliable_buf[XRCE_STREAM_BUFFER_SIZE];
    uxrStreamId output_reliable;
    uxrStreamId input_reliable;

    /* Participant + entity-id allocator. */
    uxrObjectId participant_oid;
    uint16_t next_entity_id;

    /* Per-entity slot pools. */
    xrce_subscriber_slot subscriber_slots[XRCE_MAX_SUBSCRIBERS];
    xrce_service_server_slot service_server_slots[XRCE_MAX_SERVICE_SERVERS];
    xrce_service_client_slot service_client_slots[XRCE_MAX_SERVICE_CLIENTS];

    /* Issue 0847 — how many entity handles still point HERE, and whether
     * `xrce_session_destroy` has already run.
     *
     * Every entity state (`xrce_publisher_state` and the three beside it)
     * holds a raw `xrce_session_state_t*`. `executor.close()` destroys the
     * session, and a publisher still alive in the caller's scope then runs its
     * own destructor and dereferences freed memory -- a plain use-after-free
     * across the C ABI, and the ordering the bench binaries and the docs both
     * show. It reproduced as SIGSEGV at every payload size.
     *
     * Two fields, no new pools. A pool was the other candidate: nulling each
     * live entity's back-pointer at close needs the session to ENUMERATE its
     * entities, and the slot tables above cover subscribers, service servers
     * and service clients but NOT publishers -- so that shape needs a fourth
     * static pool, on the backend whose whole campaign right now is removing
     * static RAM nobody can price (phase-392).
     *
     * Cyclone is immune to this and the difference is instructive: it stores
     * `dds_entity_t` HANDLES, which the library validates, so `dds_delete`
     * after its participant is gone returns an error instead of faulting. A
     * raw pointer cannot be validated once freed, so the memory has to outlive
     * the pointer instead.
     *
     * NOT thread-safe, deliberately and consistently with the rest of this
     * struct: XRCE is poll-based and single-threaded per session -- the
     * executor drives it. If that ever changes, this counter needs the same
     * protection as `next_entity_id` and the slot pools, which have none
     * either. */
    size_t live_entities;
    bool session_closed;
};

typedef struct xrce_session_state xrce_session_state_t;

/* Per-publisher state. */
typedef struct xrce_publisher_state {
    xrce_session_state_t* session_state;
    uxrObjectId datawriter_oid;
} xrce_publisher_state;

/* Per-subscriber state — the slot lives inside the session state. */
typedef struct xrce_subscriber_state {
    xrce_session_state_t* session_state;
    xrce_subscriber_slot* slot;
    uxrObjectId datareader_oid;
} xrce_subscriber_state;

/* Per-service-server state. */
typedef struct xrce_service_server_state {
    xrce_session_state_t* session_state;
    xrce_service_server_slot* slot;
    uxrObjectId replier_oid;
} xrce_service_server_state;

/* Per-service-client state. */
typedef struct xrce_service_client_state {
    xrce_session_state_t* session_state;
    xrce_service_client_slot* slot;
    uxrObjectId requester_oid;
} xrce_service_client_state;

/* ---- Helpers -------------------------------------------------------- */

/* ---- Issue 0847: entity/session lifetime ------------------------------ */

/* Register one entity against the session. Call at the single SUCCESS point of
 * a creator -- not where `session_state` is assigned, because several creators
 * fail after that assignment and free their own state, which would leave the
 * count high forever and leak the session. */
void xrce_session_entity_attach(xrce_session_state_t* st);

/* Has `xrce_session_destroy` already run? A destructor MUST check this before
 * touching `st->session` or `st->output_reliable`: after close, the uxr session
 * is deleted and the transport is shut, so the agent has already dropped every
 * entity and a DELETE_ENTITY has nowhere to go. Call BEFORE detaching -- detach
 * may free `st`. */
bool xrce_session_is_closed(const xrce_session_state_t* st);

/* Mark the session closed and free its state IF no entity still points at it.
 * Split out of `xrce_session_destroy` so the lifetime decision is testable
 * without a transport: `uxr_delete_session` waits on a session status and
 * faults without one, and this bug is about WHEN the state is freed, not about
 * the wire. */
void xrce_session_mark_closed(xrce_session_state_t* st);

/* Unregister one entity. Frees the session state when it was the last one and
 * the session is already closed -- so the memory outlives the pointers into it
 * without the session needing to know who they are. Never touch `st` after
 * this returns. */
void xrce_session_entity_detach(xrce_session_state_t* st);

/* Allocate the next entity id of the given type. Mirrors the Rust
 * impl's `alloc_entity_id`. */
uxrObjectId xrce_alloc_entity_id(xrce_session_state_t* st, uint8_t type);

/* Run the agent until all `count` request statuses are received,
 * returning OK only if every status is `UXR_STATUS_OK` /
 * `UXR_STATUS_OK_MATCHED`. */
rmw_ret_t xrce_confirm_entities(xrce_session_state_t* st, const uint16_t* requests,
                                uint8_t* statuses, size_t count);

/* Issue 0819 — stage one inbound XRCE payload into a receive slot.
 *
 * Every receive callback in this backend does the same three things: bound the
 * declared length against the slot, write the 4-byte CDR encapsulation header
 * the agent strips on the wire, and copy the payload in behind it. The copy is
 * the part that cannot be hand-written, and all three sites had it wrong the
 * same way.
 *
 * A fragmented message does NOT arrive as contiguous bytes. `uxr_next_input_
 * reliable_buffer_available` initialises the `ucdrBuffer` over the FIRST
 * fragment only and installs `on_full_input_buffer`, which swaps in the next
 * fragment when a read crosses the end; `read_format_data` deliberately
 * propagates that callback onto the buffer it hands the callback. So
 * `ub->iterator` addresses one fragment, `ucdr_buffer_remaining()` measures one
 * fragment, and a flat `memcpy` of the DECLARED length reads past the fragment
 * into whatever follows it. `ucdr_deserialize_array_uint8_t` is the read that
 * honours the chain, and is what this uses.
 *
 * Writes the header + payload into `dst` (capacity `dst_cap`) and sets
 * `*out_len` to the total staged length. Returns false when the message does
 * not fit or the reassembly failed — the caller's overflow flag, surfaced as
 * `NROS_RMW_RET_MESSAGE_TOO_LARGE`, which is loud. `*out_len` is 0 then. */
bool xrce_stage_inbound(uint8_t* dst, size_t dst_cap, struct ucdrBuffer* ub, size_t len,
                        size_t* out_len);

/* DDS topic-name conversion. Strips a leading '/' and prepends "rt/"
 * unless `avoid_ros_prefix` is non-zero. Writes a NUL-terminated
 * string into `out` (capacity `out_cap`); truncates if too long. */
void xrce_dds_topic_name(const char* topic_name, int avoid_ros_prefix, char* out, size_t out_cap);
void xrce_dds_request_topic(const char* service_name, char* out, size_t out_cap);
void xrce_dds_reply_topic(const char* service_name, char* out, size_t out_cap);
void xrce_dds_request_type(const char* type_name, char* out, size_t out_cap);
void xrce_dds_reply_type(const char* type_name, char* out, size_t out_cap);

/* QoS mapping. */
uxrQoS_t xrce_map_qos(const rmw_qos_profile_t* qos);

/* ---- Capacity diagnostics (issue 1033) ------------------------------- */

/* Severity for `nros_platform_log_write`, which takes `nros_log::Severity`'s
 * `as_u8()`: 0 Trace, 1 Debug, 2 Info, 3 Warn, 4 Error, 5 Fatal. Spelled here
 * rather than included from `nros/log.h`: that header belongs to `nros-c`, the
 * layer ABOVE this one, and an RMW backend must not depend upward. */
#define XRCE_LOG_ERROR 4

/* Report that this image was BUILT with fewer `kind` slots than it has just
 * been asked to create, then let the caller return
 * `NROS_RMW_RET_INVALID_CONFIG`.
 *
 * issue 1033. Until the entity caps joined the derivable ladder, exhausting one
 * meant creating a ninth subscriber on an image budgeted for eight — rare, and
 * arguably the caller's problem. Now the cap is DERIVED from what the image
 * DECLARED (`nano_ros_node_register(... ENTITIES ...)`), so it is exactly the
 * declared count, and the ordinary way to hit it is to create an entity that
 * was never declared. The remedy is a rebuild, and the reader needs to be told
 * WHICH knob and WHY it is the number it is — a bare `NROS_RMW_RET_ERROR`, the
 * code these paths used to return, says none of that and is the same value a
 * NULL `backend_data` returns three lines up.
 *
 * `NROS_RMW_RET_INVALID_CONFIG` is issue 0468's code, added for precisely this
 * on the zenoh side: "a COMPILE-TIME capacity or configuration made the call
 * impossible ... the remedy is a rebuild, never a different argument". This is
 * the same failure one backend over, so it gets the same code rather than a
 * second spelling of it.
 *
 * Delivery is `nros_platform_log_write` — the "platform's printk-equivalent"
 * `nros/rmw_ret.h` already names as where backends log at the failure site. It
 * reaches every platform including Zephyr native_sim, where Rust `std` stdio is
 * fatal (issue 0589).
 *
 * `kind` names the entity ("subscriber"), `name` the topic or service it was
 * for, `cap` the compile-time maximum and `knob` the Kconfig symbol that moves
 * it. */
void xrce_report_capacity_exhausted(const char* kind, const char* name, unsigned cap,
                                    const char* knob);

/* Report that the ONE per-session allocation could not be served, with the size
 * that was asked for and the knobs that decide it.
 *
 * issue 1033 — `sizeof(xrce_session_state_t)` is decided entirely at build time
 * and dominated by `subscriber_slots`, and the whole struct is a single
 * request. When it fails the caller gets `NROS_RMW_RET_BAD_ALLOC` and nothing
 * else, which is how a 309,696-byte request against a 65,536-byte Zephyr heap
 * presented as an anonymous boot failure for as long as issue 0968 was open.
 * The number is knowable at the failure site; printing it is the difference
 * between a nine-step bisect and one line. */
void xrce_report_session_alloc_failed(size_t bytes);

/* ---- session.c ---- */
rmw_ret_t xrce_session_create(const char* locator, uint8_t mode, uint32_t domain_id,
                              const char* node_name, const rmw_session_options_t* options,
                              rmw_session_t* out);
rmw_ret_t xrce_session_destroy(rmw_session_t* session);
rmw_ret_t xrce_session_drive_io(rmw_session_t* session, int32_t timeout_ms);
/* Phase 124.F.2 — connectivity probe via `uxr_ping_agent_session`. */
rmw_ret_t xrce_session_ping(rmw_session_t* session, int32_t timeout_ms);

/* ---- publisher.c ---- */
rmw_ret_t xrce_publisher_create(const rmw_node_t* node,
                                const rmw_message_type_support_t* type_support,
                                const char* topic_name, uint32_t domain_id,
                                const rmw_qos_profile_t* qos,
                                const rmw_publisher_options_t* options, rmw_publisher_t* out);
rmw_ret_t xrce_publisher_destroy(rmw_publisher_t* publisher);
/* Issue 0782 — exposed for the smoke test: the streamed-publish chunk loop,
 * which is the only part of that path reachable without an XRCE agent. */
size_t xrce_drive_streamed_body(uint8_t* body, size_t body_len, size_t total,
                                void (*chunk_cb)(uint8_t* out_buf, size_t cap, size_t* out_written,
                                                 void* user_ctx),
                                void* user_ctx);
rmw_ret_t xrce_publisher_publish_raw(const rmw_publisher_t* publisher, rmw_byte_span_t payload);
/* Phase 124.E.3 — streamed publish via `uxr_prepare_output_stream`. */
rmw_ret_t xrce_publisher_publish_streamed(rmw_publisher_t* publisher,
                                          void (*size_cb)(size_t* out_total_len, void* user_ctx),
                                          void (*chunk_cb)(uint8_t* out_buf, size_t cap,
                                                           size_t* out_written, void* user_ctx),
                                          void* user_ctx);

/* ---- subscriber.c ---- */
rmw_ret_t
xrce_subscription_create(const rmw_node_t* node, const rmw_message_type_support_t* type_support,
                         const char* topic_name, uint32_t domain_id, const rmw_qos_profile_t* qos,
                         const rmw_subscription_options_t* options, rmw_subscription_t* out);
rmw_ret_t xrce_subscription_destroy(rmw_subscription_t* subscriber);
rmw_ret_t xrce_subscription_take(const rmw_subscription_t* subscriber, rmw_mut_byte_span_t* out,
                                 bool* taken);
rmw_ret_t xrce_subscription_has_data(rmw_subscription_t* subscriber, bool* out_has_data);
/* Phase 231 (RFC-0038) — zero-copy in-place take over the XRCE static ring. */
rmw_ret_t xrce_subscription_supports_in_place(rmw_subscription_t* subscriber, bool* out_supports);
rmw_ret_t xrce_subscription_process_raw_in_place(rmw_subscription_t* subscriber, void* ctx,
                                                 void (*cb)(void* ctx, rmw_byte_span_t message),
                                                 bool* out_processed);

/* Topic data callback — single instance per session, registered at
 * session_open. Exposed so session.c can pass its address to
 * `uxr_set_topic_callback`. */
void xrce_topic_callback(uxrSession* session, uxrObjectId object_id, uint16_t request_id,
                         uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t length, void* args);

/* ---- service.c ---- */
rmw_ret_t xrce_service_create(const rmw_node_t* node,
                              const rmw_service_type_support_t* type_support,
                              const char* service_name, uint32_t domain_id,
                              const rmw_qos_profile_t* qos, rmw_service_t* out);
rmw_ret_t xrce_service_destroy(rmw_service_t* server);
rmw_ret_t xrce_service_take_request(const rmw_service_t* server, rmw_mut_byte_span_t* request,
                                    int64_t* seq_out, bool* taken);
rmw_ret_t xrce_service_has_request(rmw_service_t* server, bool* out_has_request);
rmw_ret_t xrce_service_send_response(const rmw_service_t* server, int64_t seq,
                                     rmw_byte_span_t response);

rmw_ret_t xrce_client_create(const rmw_node_t* node, const rmw_service_type_support_t* type_support,
                             const char* service_name, uint32_t domain_id,
                             const rmw_qos_profile_t* qos, rmw_client_t* out);
rmw_ret_t xrce_client_destroy(rmw_client_t* client);
/* Phase 130.4 — non-blocking split (phase-301: the deprecated blocking
 * `call_raw` slot was deleted from the vtable; this pair is the one
 * request/reply path). */
rmw_ret_t xrce_service_send_request_raw(const rmw_client_t* client, rmw_byte_span_t request_span,
                                        int64_t* sequence_id);
rmw_ret_t xrce_service_take_response(const rmw_client_t* client, rmw_mut_byte_span_t* reply,
                                     int64_t* seq_out, bool* taken);

void xrce_request_callback(uxrSession* session, uxrObjectId object_id, uint16_t request_id,
                           SampleIdentity* sample_id, struct ucdrBuffer* ub, uint16_t length,
                           void* args);
void xrce_reply_callback(uxrSession* session, uxrObjectId object_id, uint16_t request_id,
                         uint16_t reply_id, struct ucdrBuffer* ub, uint16_t length, void* args);

/* ---- transport_custom.c (Phase 115.K.2.4) -------------------------- */

/* Install a runtime-supplied transport vtable into the session's
 * `uxrCustomTransport`. Trampolines fan out to the user's
 * open/close/write/read callbacks. The session.c open path consults
 * `xrce_custom_transport_is_armed()` after a `custom://` locator and
 * routes accordingly. */
struct xrce_custom_ops_slot;
int xrce_custom_transport_is_armed(void);
rmw_ret_t xrce_custom_transport_install(xrce_session_state_t* st, bool framing);

/* Phase 115.K.2.5.1.2.a-fix-transport — POSIX UDP via custom
 * transport. Replaces the K.2.1 `uxr_init_udp_transport` direct
 * path. Resolves `host`/`port`, opens a connected UDP socket,
 * and wires `xrce_session_state_t::custom` with trampolines that
 * drive the socket via `poll()` + `recv()` / `send()`. The
 * resulting transport behaves like the legacy `xrce-sys` shape
 * the agent has interop'd with for years. */
rmw_ret_t xrce_posix_udp_init(xrce_session_state_t* st, const char* host, const char* port);

/* Zephyr UDP via the canonical nros platform networking ABI. Uses the same
 * Micro-XRCE custom transport shape as POSIX UDP, but delegates socket and
 * endpoint storage to nros_platform_udp_* instead of POSIX sockets. */
rmw_ret_t xrce_zephyr_udp_init(xrce_session_state_t* st, const char* host, const char* port);

/* Phase 129.NET.3 — platform-agnostic XRCE UDP. Mirrors the Zephyr
 * variant but without the per-platform `#if`. Works on every target
 * that satisfies the `nros_platform_udp_*` symbols at link time
 * (POSIX, Zephyr, FreeRTOS, ThreadX, ESP-IDF; bare-metal via
 * `nros-smoltcp`). Supersedes `xrce_posix_udp_init` /
 * `xrce_zephyr_udp_init` long-term; both are kept for one cycle
 * for fallback. */
rmw_ret_t xrce_nros_udp_init(xrce_session_state_t* st, const char* host, const char* port);

/* Phase 115.K.2.5.1.5-serial — POSIX serial transport via custom
 * transport. Opens a tty/pty `path`, configures termios (raw mode,
 * 8N1, baud from `XRCE_SERIAL_BAUD` env or 115200), and registers
 * read/write trampolines. framing=true (HDLC). */
rmw_ret_t xrce_posix_serial_init(xrce_session_state_t* st, const char* path);

#ifdef __cplusplus
}
#endif

#endif /* NROS_RMW_XRCE_C_INTERNAL_H */
