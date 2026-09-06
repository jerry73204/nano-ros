/* Phase 115.K.2.2 — subscriber path.
 *
 * Mirrors the Rust impl's `XrceSession::create_subscriber` /
 * `XrceSubscriber::take_serialized`. Single-slot ringbuffer: callbacks
 * overwrite stale data; oversize messages flag overflow and drop.
 *
 * The topic callback dispatches by datareader_id to the matching
 * slot in the per-session pool. It's registered ONCE in
 * `xrce_session_create` (see session.c) — re-registering per
 * subscriber would race with concurrent inbound messages.
 */

#include "internal.h"

#include "nros/rmw_ret.h"

#include <stdlib.h>
#include <string.h>

#include <uxr/client/client.h>
#include <uxr/client/core/session/object_id.h>

/* Topic callback — dispatches by datareader id. Registered once at
 * session_open via `uxr_set_topic_callback(..., xrce_topic_callback,
 * st)`. */
void xrce_topic_callback(uxrSession* session, uxrObjectId object_id, uint16_t request_id,
                         uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t length,
                         void* args) {
    (void)session;
    (void)request_id;
    (void)stream_id;

    xrce_session_state_t* st = (xrce_session_state_t*)args;
    if (st == NULL || ub == NULL) {
        return;
    }
    size_t len = (size_t)length;
    for (size_t i = 0; i < XRCE_MAX_SUBSCRIBERS; ++i) {
        xrce_subscriber_slot* slot = &st->subscriber_slots[i];
        if (!slot->active || slot->datareader_id != object_id.id) {
            continue;
        }
        /* Reader currently reading the slot — drop. */
        if (slot->locked) {
            return;
        }
        /* Phase 160.H.1 — ring full → drop the newest. Preserves
         * in-order delivery of the already-buffered messages; the
         * alternative (overwrite oldest) silently shifts the
         * sequence which is harder to diagnose. */
        if (slot->count >= XRCE_SUBSCRIBER_RING_DEPTH) {
            return;
        }
        xrce_subscriber_ring_entry* entry = &slot->entries[slot->write_idx];
        /* Issue 0819 — staging (bound, CDR header, fragment-aware copy) is
         * `xrce_stage_inbound`, shared with the two service inboxes because all
         * three had the same fragment bug. A refusal sets the entry's overflow
         * flag, which the take reports as NROS_RMW_RET_MESSAGE_TOO_LARGE. */
        entry->overflow = !xrce_stage_inbound(entry->data, XRCE_BUFFER_SIZE, ub, len, &entry->len);
        slot->write_idx = (uint16_t)((slot->write_idx + 1) % XRCE_SUBSCRIBER_RING_DEPTH);
        slot->count++;
        return;
    }
    /* TODO 115.K.2.x: bump a per-session "unmatched callback" counter
     * for diagnostics when the slot pool is full. */
}

rmw_ret_t
xrce_subscription_create(const rmw_node_t* node, const rmw_message_type_support_t* type_support,
                         const char* topic_name, uint32_t domain_id, const rmw_qos_profile_t* qos,
                         const rmw_subscription_options_t* options, rmw_subscription_t* out) {
    /* phase-406 W1 — one argument in, two locals out, so the body below is
       unchanged. A NULL type support is INVALID_ARGUMENT rather than an
       empty type: the identity is what the entity is keyed on, and one
       created without it matches nothing and reports nothing. */
    if (type_support == NULL) return NROS_RMW_RET_INVALID_ARGUMENT;
    const char* type_name = type_support->type_name;
    const char* type_hash = type_support->type_hash;
    (void)type_name;
    /* Phase 376 W5/B1 — the entity is created ON ITS NODE, as upstream does.
     * The node carries the route to its session (our `context`). */
    if (node == NULL) return NROS_RMW_RET_INVALID_ARGUMENT;
    rmw_session_t* session = node->session;
    (void)type_hash;
    (void)domain_id;
    (void)options;

    if (session == NULL || out == NULL || topic_name == NULL || type_name == NULL) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    xrce_session_state_t* st = (xrce_session_state_t*)session->backend_data;
    if (st == NULL) {
        return NROS_RMW_RET_ERROR;
    }

    /* Find a free slot. */
    xrce_subscriber_slot* slot = NULL;
    for (size_t i = 0; i < XRCE_MAX_SUBSCRIBERS; ++i) {
        if (!st->subscriber_slots[i].active) {
            slot = &st->subscriber_slots[i];
            break;
        }
    }
    if (slot == NULL) {
        /* issue 1033 — LOUD, and a code that says "rebuild", not the generic
         * one. See `xrce_report_capacity_exhausted` in internal.h. */
        xrce_report_capacity_exhausted("subscriber", topic_name, (unsigned)XRCE_MAX_SUBSCRIBERS,
                                       "NROS_XRCE_MAX_SUBSCRIBERS");
        return NROS_RMW_RET_INVALID_CONFIG;
    }

    xrce_subscriber_state* ss =
        (xrce_subscriber_state*)nros_xrce_calloc(1, sizeof(xrce_subscriber_state));
    if (ss == NULL) {
        return NROS_RMW_RET_BAD_ALLOC;
    }
    ss->session_state = st;
    ss->slot = slot;

    uxrObjectId topic_oid = xrce_alloc_entity_id(st, UXR_TOPIC_ID);
    uxrObjectId sub_oid = xrce_alloc_entity_id(st, UXR_SUBSCRIBER_ID);
    uxrObjectId dr_oid = xrce_alloc_entity_id(st, UXR_DATAREADER_ID);

    int avoid_ros = 0;
    if (qos != NULL) {
        avoid_ros = qos->avoid_ros_namespace_conventions != 0;
    }

    char dds_topic[XRCE_DDS_NAME_BUF_SIZE];
    char dds_type[XRCE_DDS_NAME_BUF_SIZE];
    xrce_dds_topic_name(topic_name, avoid_ros, dds_topic, sizeof(dds_topic));
    size_t tn_len = strlen(type_name);
    if (tn_len + 1 > sizeof(dds_type)) tn_len = sizeof(dds_type) - 1;
    memcpy(dds_type, type_name, tn_len);
    dds_type[tn_len] = '\0';

    uxrQoS_t xrce_qos = xrce_map_qos(qos);

    uint16_t req_topic =
        uxr_buffer_create_topic_bin(&st->session, st->output_reliable, topic_oid,
                                    st->participant_oid, dds_topic, dds_type, UXR_REPLACE);
    uint16_t req_sub = uxr_buffer_create_subscriber_bin(&st->session, st->output_reliable, sub_oid,
                                                        st->participant_oid, UXR_REPLACE);
    uint16_t req_dr = uxr_buffer_create_datareader_bin(&st->session, st->output_reliable, dr_oid,
                                                       sub_oid, topic_oid, xrce_qos, UXR_REPLACE);

    uint16_t requests[3] = {req_topic, req_sub, req_dr};
    uint8_t statuses[3] = {0, 0, 0};
    rmw_ret_t cret = xrce_confirm_entities(st, requests, statuses, 3);
    if (cret != NROS_RMW_RET_OK) {
        nros_xrce_free(ss);
        return cret;
    }

    /* Register slot for callback dispatch. */
    slot->datareader_id = dr_oid.id;
    slot->write_idx = 0;
    slot->read_idx = 0;
    slot->count = 0;
    slot->locked = false;
    for (size_t i = 0; i < XRCE_SUBSCRIBER_RING_DEPTH; ++i) {
        slot->entries[i].len = 0;
        slot->entries[i].overflow = false;
    }
    slot->active = true;
    ss->datareader_oid = dr_oid;

    /* Request continuous data delivery. */
    uxrDeliveryControl delivery = {
        .max_samples = UXR_MAX_SAMPLES_UNLIMITED,
        .max_elapsed_time = UXR_MAX_ELAPSED_TIME_UNLIMITED,
        .max_bytes_per_second = UXR_MAX_BYTES_PER_SECOND_UNLIMITED,
        .min_pace_period = 0,
    };
    (void)uxr_buffer_request_data(&st->session, st->output_reliable, dr_oid, st->input_reliable,
                                  &delivery);
    (void)uxr_run_session_time(&st->session, XRCE_SESSION_FLUSH_TIMEOUT_MS);

    /* Issue 0847 — this entity now holds a pointer into the session state, so
     * the session may not free itself until this handle is gone. */
    xrce_session_entity_attach(st);
    out->backend_data = ss;
    out->can_loan_messages = false;
    return NROS_RMW_RET_OK;
}

rmw_ret_t xrce_subscription_destroy(rmw_subscription_t* subscriber) {
    if (subscriber == NULL || subscriber->backend_data == NULL) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    xrce_subscriber_state* ss = (xrce_subscriber_state*)subscriber->backend_data;
    xrce_session_state_t* st = ss->session_state;

    if (ss->slot != NULL) {
        ss->slot->active = false;
        ss->slot->count = 0;
        ss->slot->write_idx = 0;
        ss->slot->read_idx = 0;
    }
    /* Phase 376 W5 — the slot reports now, but XRCE cannot know. The delete is
     * deliberately fire-and-forget (close-time teardown must not block on
     * agent acks), so the only failure this frame can see is a request that
     * would not BUFFER. That is worth reporting; the agent's own verdict is
     * not available at any price this path is willing to pay. */
    /* Issue 0847 — a CLOSED session has already deleted the uxr session and shut
     * the transport, so the agent dropped this entity with it and there is
     * nowhere for a DELETE_ENTITY to go. Reading `st->session` here is the
     * use-after-free itself, so the check comes first and the request is not
     * attempted. Skipping is not a failure: this is the supported teardown
     * order, so it returns OK. */
    rmw_ret_t ret = NROS_RMW_RET_OK;
    if (!xrce_session_is_closed(st)) {
        /* Phase 376 W5 — the slot reports now, but XRCE cannot know. The delete
         * is deliberately fire-and-forget (close-time teardown must not block on
         * agent acks), so the only failure this frame can see is a request that
         * would not BUFFER. That is worth reporting; the agent's own verdict is
         * not available at any price this path is willing to pay. */
        uint16_t req =
            uxr_buffer_delete_entity(&st->session, st->output_reliable, ss->datareader_oid);
        (void)uxr_run_session_time(&st->session, 0);
        ret = req == UXR_INVALID_REQUEST_ID ? NROS_RMW_RET_ERROR : NROS_RMW_RET_OK;
    }

    nros_xrce_free(ss);
    subscriber->backend_data = NULL;
    /* LAST — this may free `st`. Nothing may touch it afterwards. */
    xrce_session_entity_detach(st);
    return ret;
}

rmw_ret_t xrce_subscription_take(const rmw_subscription_t* subscriber, rmw_mut_byte_span_t* out,
                                 bool* taken) {
    /* phase-406 W2 — by pointer: `capacity` in, `len` out. */
    if (out == NULL) return NROS_RMW_RET_INVALID_ARGUMENT;
    uint8_t* buf = out->data;
    const size_t buf_len = out->capacity;
    size_t* out_len = &out->len;
    /* Phase 376 W3.b/W3.d step A — upstream `rmw_take`'s shape. */
    if (subscriber == NULL || subscriber->backend_data == NULL || out_len == NULL ||
        taken == NULL) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    xrce_subscriber_state* ss = (xrce_subscriber_state*)subscriber->backend_data;
    xrce_subscriber_slot* slot = ss->slot;
    if (slot == NULL || slot->count == 0) {
        /* Empty subscription: OK with `taken = false`, not a sentinel. */
        *taken = false;
        return NROS_RMW_RET_OK;
    }
    xrce_subscriber_ring_entry* entry = &slot->entries[slot->read_idx];
    /* Always consume the head slot regardless of outcome — overflow,
     * buffer-too-small, and successful read all advance the ring so a
     * single bad entry can't wedge the queue. */
    rmw_ret_t ret;
    if (entry->overflow) {
        ret = NROS_RMW_RET_MESSAGE_TOO_LARGE;
    } else if (entry->len > buf_len) {
        ret = NROS_RMW_RET_BUFFER_TOO_SMALL;
    } else {
        slot->locked = true;
        if (buf != NULL && entry->len > 0) {
            memcpy(buf, entry->data, entry->len);
        }
        slot->locked = false;
        *out_len = entry->len;
        *taken = true;
        ret = NROS_RMW_RET_OK;
    }
    entry->len = 0;
    entry->overflow = false;
    slot->read_idx = (uint16_t)((slot->read_idx + 1) % XRCE_SUBSCRIBER_RING_DEPTH);
    slot->count--;
    return ret;
}

rmw_ret_t xrce_subscription_has_data(rmw_subscription_t* subscriber, bool* out_has_data) {
    /* Phase 376 W3.d step A — flag out, status returned. */
    if (out_has_data == NULL) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    if (subscriber == NULL || subscriber->backend_data == NULL) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    xrce_subscriber_state* ss = (xrce_subscriber_state*)subscriber->backend_data;
    if (ss->slot == NULL) {
        /* No slot bound yet: legitimately nothing to take. */
        *out_has_data = false;
        return NROS_RMW_RET_OK;
    }
    *out_has_data = ss->slot->count > 0;
    return NROS_RMW_RET_OK;
}

/* Phase 231 (RFC-0038) — the XRCE backend already stages each message in a
 * static ring entry (`entry->data`), so it can hand the bytes to the callback
 * in place instead of copying into a caller buffer (copy #1 removed). */
rmw_ret_t xrce_subscription_supports_in_place(rmw_subscription_t* subscriber, bool* out_supports) {
    /* Phase 376 W3.d step A — capability out, status returned. */
    (void)subscriber;
    if (out_supports == NULL) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    *out_supports = true;
    return NROS_RMW_RET_OK;
}

rmw_ret_t xrce_subscription_process_raw_in_place(rmw_subscription_t* subscriber, void* ctx,
                                                 void (*cb)(void* ctx, rmw_byte_span_t message),
                                                 bool* out_processed) {
    /* Phase 376 W3.d step A — "processed one" out, status returned. An empty
     * subscription is OK with false rather than the NO_DATA sentinel. */
    if (out_processed == NULL) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    if (subscriber == NULL || subscriber->backend_data == NULL) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    xrce_subscriber_state* ss = (xrce_subscriber_state*)subscriber->backend_data;
    xrce_subscriber_slot* slot = ss->slot;
    if (slot == NULL || slot->count == 0) {
        *out_processed = false;
        return NROS_RMW_RET_OK;
    }
    xrce_subscriber_ring_entry* entry = &slot->entries[slot->read_idx];
    /* Always consume the head slot (overflow + success both advance) so a single
     * bad entry can't wedge the queue — mirrors take_serialized. */
    rmw_ret_t ret;
    if (entry->overflow) {
        ret = NROS_RMW_RET_MESSAGE_TOO_LARGE;
    } else {
        /* Borrow the ring entry in place — no copy into a caller buffer. The
         * callback must not re-enter this subscriber's receive (slot locked). */
        slot->locked = true;
        if (cb != NULL && entry->len > 0) {
            rmw_byte_span_t message = {entry->data, entry->len};
            cb(ctx, message);
        }
        slot->locked = false;
        *out_processed = true; /* one message processed */
        ret = NROS_RMW_RET_OK;
    }
    entry->len = 0;
    entry->overflow = false;
    slot->read_idx = (uint16_t)((slot->read_idx + 1) % XRCE_SUBSCRIBER_RING_DEPTH);
    slot->count--;
    return ret;
}
