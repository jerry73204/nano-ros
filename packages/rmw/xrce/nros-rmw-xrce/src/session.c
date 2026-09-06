/* Phase 115.K.2 — session lifecycle implementation.
 *
 * Mirrors the Rust `nros-rmw-xrce::XrceRmw::open` shape but in pure
 * C against `uxr_*`. Phase 115.K.2.1 supported UDP only; 115.K.2.4
 * adds a `custom://` locator scheme that routes through the runtime
 * transport vtable bridge in `transport_custom.c`.
 *
 * Allocation: a single `struct xrce_session_state` per session lives
 * on the heap (`malloc`). The pointer is parked in
 * `rmw_session_t::backend_data`. The runtime owns the entity-
 * shell `rmw_session_t` struct itself.
 */

#include <unistd.h>
#include "internal.h"

#include "nros/platform.h"
#include "nros/rmw_ret.h"

#include <uxr/client/client.h>
#if defined(UCLIENT_PROFILE_UDP)
#include <uxr/client/profile/transport/ip/udp/udp_transport.h>
#endif
#if defined(UCLIENT_PROFILE_UDP) && defined(UCLIENT_PLATFORM_POSIX)
#include <uxr/client/profile/transport/ip/udp/udp_transport_posix.h>
#endif
#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <uxr/client/core/session/object_id.h>
#include <uxr/client/util/ping.h>

#include <stdarg.h>
#include <stdint.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- Helpers ------------------------------------------------------- */

/* issue 1033 — the ONE place this backend renders a diagnostic. Both reporters
 * below funnel through it so there is a single answer to "where does an XRCE
 * capacity failure come out", and adding a third reporter does not add a third
 * spelling of the delivery.
 *
 * The buffer is a local, sized for a name at `XRCE_DDS_NAME_BUF_SIZE` plus the
 * fixed prose. `snprintf` truncates rather than overflowing, and a truncated
 * diagnostic is still strictly more than the nothing these paths printed
 * before. Never called from the topic/request callbacks — only from the create
 * paths and session open — so the stack cost is not on the RX path. */
#if defined(__GNUC__)
__attribute__((format(printf, 1, 2)))
#endif
static void
xrce_log_error(const char* fmt, ...) {
    char msg[352];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    size_t len = ((size_t)n < sizeof(msg)) ? (size_t)n : sizeof(msg) - 1;
    static const char kLogger[] = "nros_rmw_xrce";
    nros_platform_log_write(XRCE_LOG_ERROR, (const uint8_t*)kLogger, sizeof(kLogger) - 1,
                            (const uint8_t*)msg, (uintptr_t)len);
}

void xrce_report_capacity_exhausted(const char* kind, const char* name, unsigned cap,
                                    const char* knob) {
    xrce_log_error("no free %s slot for '%s': this image was BUILT with %s=%u. "
                   "The default is DERIVED from the entities this image declares "
                   "(nano_ros_node_register(... ENTITIES ...)), so an entity created "
                   "without being declared is not counted. State CONFIG_%s, or declare "
                   "it. Not truncating; the create fails.",
                   kind, (name != NULL) ? name : "(unnamed)", knob, cap, knob);
}

void xrce_report_session_alloc_failed(size_t bytes) {
    xrce_log_error("xrce_session_state_t (%lu bytes) did not fit. Its size is decided "
                   "at BUILD time by NROS_XRCE_MAX_SUBSCRIBERS x "
                   "NROS_XRCE_SUBSCRIBER_RING_DEPTH x NROS_XRCE_BUFFER_SIZE, plus the "
                   "service server/client slots; it is ONE allocation, so a heap larger "
                   "than the shortfall does not help. Lower a cap or raise the platform "
                   "heap (CONFIG_NROS_ZEPHYR_HEAP_SIZE on Zephyr).",
                   (unsigned long)bytes);
}

uxrObjectId xrce_alloc_entity_id(xrce_session_state_t* st, uint8_t type) {
    uint16_t id = st->next_entity_id++;
    return uxr_object_id(id, type);
}

rmw_ret_t xrce_confirm_entities(xrce_session_state_t* st, const uint16_t* requests,
                                uint8_t* statuses, size_t count) {
    bool ok = uxr_run_session_until_all_status(&st->session, XRCE_ENTITY_CREATION_TIMEOUT_MS,
                                               requests, statuses, count);
    if (!ok) {
        return NROS_RMW_RET_ERROR;
    }
    for (size_t i = 0; i < count; ++i) {
        if (statuses[i] != UXR_STATUS_OK && statuses[i] != UXR_STATUS_OK_MATCHED) {
            return NROS_RMW_RET_ERROR;
        }
    }
    return NROS_RMW_RET_OK;
}

/* Naming helpers — pure-C ports of `naming.rs`. */
static void copy_truncating(char* out, size_t out_cap, const char* src) {
    if (out == NULL || out_cap == 0) {
        return;
    }
    size_t len = strlen(src);
    if (len + 1 > out_cap) {
        len = out_cap - 1;
    }
    memcpy(out, src, len);
    out[len] = '\0';
}

static void append_truncating(char* out, size_t out_cap, const char* src) {
    if (out == NULL || out_cap == 0) {
        return;
    }
    size_t cur = strlen(out);
    if (cur + 1 >= out_cap) {
        return;
    }
    size_t avail = out_cap - cur - 1;
    size_t add = strlen(src);
    if (add > avail) {
        add = avail;
    }
    memcpy(out + cur, src, add);
    out[cur + add] = '\0';
}

/* Issue 0819 — the one inbound staging path. Contract in `internal.h`. */
bool xrce_stage_inbound(uint8_t* dst, size_t dst_cap, struct ucdrBuffer* ub, size_t len,
                        size_t* out_len) {
    if (out_len == NULL) {
        return false;
    }
    *out_len = 0;
    if (dst == NULL || ub == NULL) {
        return false;
    }
    if (len + XRCE_CDR_HEADER_LEN > dst_cap) {
        return false;
    }
    /* XRCE-DDS interop: the agent delivers the bare CDR-serialized sample
     * WITHOUT the 4-byte encapsulation header — that header belongs to the
     * DDS/RTPS side, which the agent owns. nano-ros's deserializers expect it,
     * so re-prepend it here; symmetric with the publish side, which strips it
     * before `uxr_buffer_topic`. */
    dst[0] = 0x00; /* CDR_LE representation id */
    dst[1] = 0x01;
    dst[2] = 0x00; /* options */
    dst[3] = 0x00;
    /* NOT `memcpy(dst + 4, ub->iterator, len)`. See the header comment: for a
     * fragmented message the iterator addresses only the first fragment, and
     * this call is the one that walks the chain. It reports failure through
     * `ub->error`, which is also how a genuinely short buffer surfaces. */
    if (len > 0 && !ucdr_deserialize_array_uint8_t(ub, dst + XRCE_CDR_HEADER_LEN, len)) {
        return false;
    }
    *out_len = len + XRCE_CDR_HEADER_LEN;
    return true;
}

void xrce_dds_topic_name(const char* topic_name, int avoid_ros_prefix, char* out, size_t out_cap) {
    if (out_cap == 0) return;
    out[0] = '\0';
    const char* src = topic_name;
    if (src && src[0] == '/') {
        src += 1;
    }
    if (!avoid_ros_prefix) {
        copy_truncating(out, out_cap, "rt/");
        append_truncating(out, out_cap, src ? src : "");
    } else {
        copy_truncating(out, out_cap, src ? src : "");
    }
}

void xrce_dds_request_topic(const char* service_name, char* out, size_t out_cap) {
    if (out_cap == 0) return;
    out[0] = '\0';
    const char* src = service_name;
    if (src && src[0] == '/') src += 1;
    copy_truncating(out, out_cap, "rq/");
    append_truncating(out, out_cap, src ? src : "");
    append_truncating(out, out_cap, "Request");
}

void xrce_dds_reply_topic(const char* service_name, char* out, size_t out_cap) {
    if (out_cap == 0) return;
    out[0] = '\0';
    const char* src = service_name;
    if (src && src[0] == '/') src += 1;
    copy_truncating(out, out_cap, "rr/");
    append_truncating(out, out_cap, src ? src : "");
    append_truncating(out, out_cap, "Reply");
}

/* Insert "Request_" / "Reply_" before a trailing '_' (matches Rust
 * impl: `example_interfaces::srv::dds_::AddTwoInts_` →
 * `example_interfaces::srv::dds_::AddTwoInts_Request_`). */
static void insert_before_trailing_underscore(const char* type_name, const char* insert, char* out,
                                              size_t out_cap) {
    if (out_cap == 0) return;
    out[0] = '\0';
    if (type_name == NULL) {
        copy_truncating(out, out_cap, insert);
        append_truncating(out, out_cap, "_");
        return;
    }
    size_t len = strlen(type_name);
    if (len > 0 && type_name[len - 1] == '_') {
        /* prefix = type_name without the trailing '_' */
        size_t prefix_len = len - 1;
        if (prefix_len + 1 > out_cap) {
            prefix_len = out_cap - 1;
        }
        memcpy(out, type_name, prefix_len);
        out[prefix_len] = '\0';
        append_truncating(out, out_cap, "_");
        append_truncating(out, out_cap, insert);
        append_truncating(out, out_cap, "_");
    } else {
        copy_truncating(out, out_cap, type_name);
        append_truncating(out, out_cap, "_");
        append_truncating(out, out_cap, insert);
        append_truncating(out, out_cap, "_");
    }
}

void xrce_dds_request_type(const char* type_name, char* out, size_t out_cap) {
    insert_before_trailing_underscore(type_name, "Request", out, out_cap);
}

void xrce_dds_reply_type(const char* type_name, char* out, size_t out_cap) {
    /* ROS 2 service reply type is `<Service>_Response_` (the topic keeps the
     * `Reply` suffix, but the type uses `Response`). */
    insert_before_trailing_underscore(type_name, "Response", out, out_cap);
}

uxrQoS_t xrce_map_qos(const rmw_qos_profile_t* qos) {
    uxrQoS_t out;
    if (qos == NULL) {
        out.durability = UXR_DURABILITY_VOLATILE;
        out.reliability = UXR_RELIABILITY_RELIABLE;
        out.history = UXR_HISTORY_KEEP_LAST;
        out.depth = 10;
        return out;
    }
    /* issue 0829 — this IS the SYSTEM_DEFAULT resolution for the XRCE backend,
     * and it needs no new branches: each ternary tests the ONE value that is
     * not the sentinel's answer, so `NROS_RMW_*_SYSTEM_DEFAULT` (0) already
     * lands on VOLATILE / RELIABLE / KEEP_LAST — what `rmw_cyclonedds_cpp` and
     * `rmw_zenoh_cpp` both resolve those three to. That was accidental until
     * now; it is recorded here so a later edit does not flip a test to
     * `== RELIABLE` and quietly send the sentinel the other way, which is
     * exactly what had happened one backend over
     * (`nros-rmw-cyclonedds/src/qos.cpp`, fixed alongside this).
     *
     * The DEPTH deliberately keeps the sentinel instead of resolving it. The
     * client encodes exactly this meaning — `optional_history_depth = false`
     * when `qos.depth == 0` (`create_entities_bin.c:148`), so the field is
     * simply absent from the CREATE submessage and the Agent's DDS layer
     * supplies its own. This backend genuinely has no depth of its own to
     * offer: every default on this path is the Agent's, so deferring is the
     * honest answer rather than a gap. */
    out.durability = (qos->durability == NROS_RMW_DURABILITY_TRANSIENT_LOCAL)
                         ? UXR_DURABILITY_TRANSIENT_LOCAL
                         : UXR_DURABILITY_VOLATILE;
    out.reliability = (qos->reliability == NROS_RMW_RELIABILITY_BEST_EFFORT)
                          ? UXR_RELIABILITY_BEST_EFFORT
                          : UXR_RELIABILITY_RELIABLE;
    out.history =
        (qos->history == NROS_RMW_HISTORY_KEEP_ALL) ? UXR_HISTORY_KEEP_ALL : UXR_HISTORY_KEEP_LAST;
    out.depth = qos->depth;
    return out;
}

/* ---- Session-key hashing ------------------------------------------- */

/* djb2 (DJB2_INIT=5381, multiplier=33) over the node name, salted with
 * the process id on POSIX. The XRCE session key must be UNIQUE PER CLIENT
 * SESSION at one agent: the agent keys its session table on it, so a
 * second client presenting the same key REBINDS the session and orphans
 * the first (issue 0150 — both C demo processes defaulted to the
 * phase-266 unified node name "node", djb2("node")=0x7C9B46AB collided,
 * and the listener silently received nothing). Name alone is not enough
 * entropy; the pid restores the uniqueness the pre-266 `nros_{pid}`
 * fallback provided. Embedded builds (one client per device, no pid)
 * keep the plain name hash. */
static uint32_t hash_session_key(const char* s) {
    uint32_t h = 5381u;
    if (s != NULL) {
        for (const unsigned char* p = (const unsigned char*)s; *p; ++p) {
            h = h * 33u + (uint32_t)*p;
        }
    }
#if defined(UCLIENT_PLATFORM_POSIX)
    {
        uint32_t pid = (uint32_t)getpid();
        h = h * 33u + (pid & 0xFFu);
        h = h * 33u + ((pid >> 8) & 0xFFu);
        h = h * 33u + ((pid >> 16) & 0xFFu);
        h = h * 33u + ((pid >> 24) & 0xFFu);
    }
#endif
    /* Ensure non-zero (XRCE-DDS may treat 0 specially). */
    return h == 0u ? 1u : h;
}

/* Parse `host:port`. On failure, returns 0 and leaves outputs
 * unchanged. The caller-provided `host_buf` receives a NUL-terminated
 * copy of the host substring up to its capacity. */
static int parse_host_port(const char* locator, char* host_buf, size_t host_buf_len,
                           uint16_t* port_out) {
    if (locator == NULL || host_buf == NULL || port_out == NULL) {
        return 0;
    }
    const char* colon = strrchr(locator, ':');
    if (colon == NULL) {
        return 0;
    }
    size_t host_len = (size_t)(colon - locator);
    if (host_len == 0 || host_len + 1 > host_buf_len) {
        return 0;
    }
    memcpy(host_buf, locator, host_len);
    host_buf[host_len] = '\0';
    long port = strtol(colon + 1, NULL, 10);
    if (port <= 0 || port > 0xffff) {
        return 0;
    }
    *port_out = (uint16_t)port;
    return 1;
}

/* ---- Session open / close / drive_io ------------------------------- */

/* `udp/host:port` or `udp4://host:port` strip the scheme prefix; bare
 * `host:port` is also accepted. `custom://...` selects the runtime
 * transport vtable bridge. */
static int locator_strip_udp_prefix(const char** locator) {
    static const char* const prefixes[] = {"udp/", "udp4://", "udp://"};
    for (size_t i = 0; i < sizeof(prefixes) / sizeof(prefixes[0]); ++i) {
        size_t plen = strlen(prefixes[i]);
        if (strncmp(*locator, prefixes[i], plen) == 0) {
            *locator = *locator + plen;
            return 1;
        }
    }
    return 0;
}

static int locator_is_custom(const char* locator) {
    if (locator == NULL) return 0;
    return strncmp(locator, "custom://", 9) == 0 || strcmp(locator, "custom") == 0;
}

/* Phase 115.K.2.5.1.5-serial — recognise locator forms that name a
 * serial / pty device:
 *   - `serial://<path>`
 *   - `serial:/<path>`        (some callers omit the second slash)
 *   - `/dev/...`              (bare absolute device path)
 *
 * Returns the substring pointing at the device path on match (caller
 * passes that to `xrce_posix_serial_init`), or NULL on no-match. */
static const char* locator_serial_path(const char* locator) {
    if (locator == NULL) return NULL;
    static const char* const sch_two = "serial://";
    static const char* const sch_one = "serial:/";
    if (strncmp(locator, sch_two, 9) == 0) {
        return locator + 9;
    }
    /* Match `serial:/` *only* if `serial://` did not match — checked
     * via prefix length above (9 vs 8). */
    if (strncmp(locator, sch_one, 8) == 0 && locator[8] != '/') {
        return locator + 8;
    }
    if (strncmp(locator, "/dev/", 5) == 0) {
        return locator;
    }
    return NULL;
}

rmw_ret_t xrce_session_create(const char* locator, uint8_t mode, uint32_t domain_id,
                              const char* node_name, const rmw_session_options_t* options,
                              rmw_session_t* out) {
    /* XRCE has no discovery to restrict and no enclave. */
    (void)mode;
    if (out == NULL || node_name == NULL) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    /* phase-206 W3 — a property this backend cannot honour is REFUSED, not
     * dropped. `mode` and `localhost_only` are hints a backend without the
     * concept may ignore; a configuration PROPERTY is not, because a silently
     * dropped one is indistinguishable from one that took effect and the
     * caller has no other way to find out. XRCE has no run-time option set of
     * its own yet; when it grows one, this becomes a lookup. */
    if (options != NULL && options->property_count != 0) {
        return NROS_RMW_RET_UNSUPPORTED;
    }
    if (out->backend_data != NULL) {
        return NROS_RMW_RET_ERROR;
    }

    xrce_session_state_t* st =
        (xrce_session_state_t*)nros_xrce_calloc(1, sizeof(xrce_session_state_t));
    if (st == NULL) {
        /* issue 1033 — say the SIZE. This is the request that dominated the
         * struct and it is the one an oversized cap kills; a bare BAD_ALLOC
         * names neither the number nor the knobs behind it. */
        xrce_report_session_alloc_failed(sizeof(xrce_session_state_t));
        return NROS_RMW_RET_BAD_ALLOC;
    }
    st->next_entity_id = 2; /* id 1 reserved for the participant */

    /* Phase 115.K.2.4 — `custom://...` routes through
     * `xrce_custom_transport_install`. UDP path mirrors K.2.1.
     * Phase 115.K.2.5.1.5-serial — `serial://...` / `/dev/...`
     * routes through `xrce_posix_serial_init` (POSIX hosts only).
     * Phase 127.C.4 — bare host:port locator now uses the
     * platform-blind `xrce_nros_udp_init` path on every target;
     * consumer must link a `nros_platform_udp_*` provider (POSIX
     * net.c auto-linked by xrce-cffi build.rs on libc hosts; Zephyr /
     * bare-metal pull `nros-platform-<rtos>` via cmake glue). */
#if defined(UCLIENT_PLATFORM_POSIX)
    const char* serial_path = locator_serial_path(locator);
#else
    const char* serial_path = NULL;
#endif
    /* #189 — implement the documented contract (`custom://` OR `serial/...`
     * routes through the installed vtable, nros-rmw-xrce-cffi lib.rs doc):
     * on non-POSIX builds a zenoh-style `serial/<device>#...` locator names
     * a UART the board wrapped via `set_custom_transport_ops` — there is no
     * POSIX serial fallback here, and the previous dispatch silently fell
     * through to the bare host:port UDP path (no UDP provider on a
     * serial-only bare-metal image → ConnectionFailed with zero bytes ever
     * written to the UART). Only reroute when a vtable is actually armed so
     * a genuine mis-locator still surfaces as the UDP parse error it always
     * was. POSIX keeps the `serial://<path>` → xrce_posix_serial_init route
     * unchanged (`serial/` vs `serial:/` prefixes are disjoint). */
    int route_custom = locator_is_custom(locator);
#if !defined(UCLIENT_PLATFORM_POSIX)
    if (!route_custom && locator != NULL && strncmp(locator, "serial/", 7) == 0 &&
        xrce_custom_transport_is_armed()) {
        route_custom = 1;
    }
#endif
    if (route_custom) {
        st->use_custom_transport = true;
        rmw_ret_t ret = xrce_custom_transport_install(st, /*framing=*/false);
        if (ret != NROS_RMW_RET_OK) {
            nros_xrce_free(st);
            return ret;
        }
        uxr_init_session(&st->session, &st->custom.comm, hash_session_key(node_name));
#if defined(UCLIENT_PLATFORM_POSIX)
    } else if (serial_path != NULL) {
        st->use_custom_transport = true;
        rmw_ret_t sret = xrce_posix_serial_init(st, serial_path);
        if (sret != NROS_RMW_RET_OK) {
            nros_xrce_free(st);
            return sret;
        }
        uxr_init_session(&st->session, &st->custom.comm, hash_session_key(node_name));
#endif
    } else {
        /* Issue 0330 — the backend owns its default agent endpoint. Agnostic
         * layers (nros-c's `nros_support_init`, the `NROS_ENTRY_LOCATOR`
         * macro, the RFC-0045 resolver's bottom rung) now hand us an ABSENT
         * locator instead of restating an XRCE fact — and absent is spelled
         * either NULL (Rust `None`) or `""` (the C/C++ ABI edges, which
         * cannot express NULL through a string-literal macro). Treat both the
         * same: fall back to the local agent host, with the port defaulting
         * to XRCE_DEFAULT_AGENT_PORT just below. */
        const char* addr_locator = (locator != NULL && locator[0] != '\0') ? locator : "127.0.0.1";
        (void)locator_strip_udp_prefix(&addr_locator);

        char host[64];
        uint16_t port = XRCE_DEFAULT_AGENT_PORT;
        if (parse_host_port(addr_locator, host, sizeof(host), &port) == 0) {
            size_t hlen = strlen(addr_locator);
            if (hlen == 0 || hlen + 1 > sizeof(host)) {
                nros_xrce_free(st);
                return NROS_RMW_RET_INVALID_ARGUMENT;
            }
            memcpy(host, addr_locator, hlen + 1);
        }
        char port_str[8];
        snprintf(port_str, sizeof(port_str), "%u", (unsigned)port);

        /* Phase 129.NET.3 — UDP via the canonical `nros_platform_udp_*`
         * ABI. Platform-blind: works on any target with a wired
         * platform-provider. */
        st->use_custom_transport = true;
        rmw_ret_t udp_ret = xrce_nros_udp_init(st, host, port_str);
        if (udp_ret != NROS_RMW_RET_OK) {
            nros_xrce_free(st);
            return udp_ret;
        }
        uxr_init_session(&st->session, &st->custom.comm, hash_session_key(node_name));
    }

    /* Topic / request / reply callbacks — single registration per
     * session. The session-state pointer is threaded through `args`
     * so the callbacks can find their slot pools without leaning on
     * a module global. */
    uxr_set_topic_callback(&st->session, xrce_topic_callback, st);
    uxr_set_request_callback(&st->session, xrce_request_callback, st);
    uxr_set_reply_callback(&st->session, xrce_reply_callback, st);

    if (!uxr_create_session_retries(&st->session, XRCE_SESSION_CREATION_RETRIES)) {
        /* Both UDP and `custom://` paths now go through
         * uxrCustomTransport — close via custom_transport. */
        uxr_close_custom_transport(&st->custom);
        nros_xrce_free(st);
        return NROS_RMW_RET_ERROR;
    }

    st->output_reliable =
        uxr_create_output_reliable_stream(&st->session, st->output_reliable_buf,
                                          sizeof(st->output_reliable_buf), XRCE_STREAM_HISTORY);
    st->input_reliable = uxr_create_input_reliable_stream(
        &st->session, st->input_reliable_buf, sizeof(st->input_reliable_buf), XRCE_STREAM_HISTORY);

    /* Create the DDS participant. ID 1 is reserved for it. */
    st->participant_oid = uxr_object_id(1, UXR_PARTICIPANT_ID);

    char name_buf[XRCE_PARTICIPANT_NAME_BUF_SIZE];
    copy_truncating(name_buf, sizeof(name_buf), node_name);

    uint16_t req =
        uxr_buffer_create_participant_bin(&st->session, st->output_reliable, st->participant_oid,
                                          (uint16_t)domain_id, name_buf, UXR_REPLACE);

    uint8_t status = 0;
    uint16_t requests[1] = {req};
    uint8_t statuses[1] = {0};
    rmw_ret_t cret = xrce_confirm_entities(st, requests, statuses, 1);
    (void)status;
    if (cret != NROS_RMW_RET_OK) {
        (void)uxr_delete_session(&st->session);
        uxr_close_custom_transport(&st->custom);
        nros_xrce_free(st);
        return cret;
    }

    out->backend_data = st;
    return NROS_RMW_RET_OK;
}

/* ---- Issue 0847: entity/session lifetime ------------------------------
 *
 * See the long note on `live_entities` in `internal.h` for why this is a
 * refcount rather than a back-pointer sweep. */

void xrce_session_entity_attach(xrce_session_state_t* st) {
    if (st == NULL) {
        return;
    }
    st->live_entities++;
}

bool xrce_session_is_closed(const xrce_session_state_t* st) {
    return st == NULL ? true : st->session_closed;
}

void xrce_session_entity_detach(xrce_session_state_t* st) {
    if (st == NULL) {
        return;
    }
    if (st->live_entities > 0) {
        st->live_entities--;
    }
    /* The last entity out of a CLOSED session turns off the lights. An open
     * session keeps its state: entities come and go while it runs. */
    if (st->session_closed && st->live_entities == 0) {
        nros_xrce_free(st);
    }
}

rmw_ret_t xrce_session_destroy(rmw_session_t* session) {
    if (session == NULL) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    xrce_session_state_t* st = (xrce_session_state_t*)session->backend_data;
    if (st == NULL) {
        return NROS_RMW_RET_ERROR;
    }
    (void)uxr_delete_session(&st->session);
    /* All three transport paths (custom://, serial://, udp://) now
     * sit on top of uxrCustomTransport — the K.2.5.1.2.a fix routed
     * UDP through the same surface to match xrce-sys's legacy
     * shape. Close once, regardless of `use_custom_transport`. */
    uxr_close_custom_transport(&st->custom);

    /* Issue 0847 — the lifetime decision, separated from the uxr teardown
     * above so it is reachable by a test. `uxr_delete_session` needs a live
     * transport (it waits on a session status and faults without one), and the
     * bug this fixes has nothing to do with the wire: it is about WHEN the
     * state is freed. `xrce_session_mark_closed` is the function that decides,
     * and it is the one `tests/entity_lifetime.c` drives. */
    session->backend_data = NULL;
    xrce_session_mark_closed(st);
    return NROS_RMW_RET_OK;
}

void xrce_session_mark_closed(xrce_session_state_t* st) {
    if (st == NULL) {
        return;
    }
    /* Issue 0847 — the session is CLOSED here, but it is only FREED once the
     * last entity handle pointing at it is gone. Freeing unconditionally is
     * what made an ordinary teardown order (close, then entities drop at end
     * of scope) a use-after-free.
     *
     * The handles are still valid objects afterwards: their destructors run
     * normally, see `session_closed`, skip the agent-side DELETE_ENTITY that
     * now has nowhere to go, and free themselves. The last one frees this.
     *
     * A deferred free is NOT an error: it is the supported ordering, not a
     * mistake by the caller. */
    st->session_closed = true;
    if (st->live_entities == 0) {
        nros_xrce_free(st);
    }
}

rmw_ret_t xrce_session_drive_io(rmw_session_t* session, int32_t timeout_ms) {
    if (session == NULL) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    xrce_session_state_t* st = (xrce_session_state_t*)session->backend_data;
    if (st == NULL) {
        return NROS_RMW_RET_ERROR;
    }
    int t = timeout_ms < 0 ? 0 : (int)timeout_ms;

    /* `uxr_run_session_time` returns as soon as the reliable output streams
     * are confirmed — so when the session holds a publisher with unconfirmed
     * WRITE_DATA (or a pending heartbeat) it returns almost immediately
     * (~0 us) instead of listening for `t` ms. XRCE is a *poll-based* backend
     * (no `set_wake_callback`): the executor's `spin_once(t)` paces by relying
     * on this call to block for `t`. When it returns early the spin loop
     * free-runs — a pub+sub node burns through a bounded loop in ~1 ms and
     * closes its session (DELETE_CLIENT) before its subscriber finishes DDS
     * discovery, so it never receives. See issue 0026.
     *
     * Drive the session across the whole `t` ms window — each pass services
     * inbound (delivering subscriber samples) — and yield ~1 ms when a pass
     * returns early, so the call consumes ~t ms wall-clock the way the caller
     * expects, without busy-spinning. (Mirrors the zpico_spin_once
     * `z_sleep_ms` fix for multi-threaded platforms.) */
    if (t == 0) {
        (void)uxr_run_session_time(&st->session, 0);
        return NROS_RMW_RET_OK;
    }
    /* Issue 0029 — use the platform clock/sleep, NOT POSIX
     * `clock_gettime(CLOCK_MONOTONIC)` / `nanosleep`. This path compiles for
     * every XRCE target incl. bare-metal Cortex-M (thumbv7m/thumbv7em), where
     * `<time.h>` does not declare `CLOCK_MONOTONIC` and there is no `nanosleep`.
     *
     * Issue 0035 — use the *monotonic* clock `nros_platform_clock_ns`, NOT the
     * *wall* clock `nros_platform_time_now_ns`. The latter is epoch time and
     * returns 0 on any platform without an RTC (Zephyr/native_sim, FreeRTOS,
     * ThreadX — see each provider's `platform.c`). With a 0 clock `elapsed_ms`
     * is always 0, `remaining` never reaches 0, and this loop spins forever
     * inside `uxr_run_session_time` — so `spin_once` never returns to run the
     * executor's readiness scan and buffered subscriber samples are never
     * delivered to callbacks. `nros_platform_clock_ns` is monotonic since
     * boot and is the contract for relative deadline deltas (mirrors
     * `uxr_millis` in platform_aliases.c).
     *
     * Issue 0548 / phase-352 W6 — `clock_ns` is the only exported clock
     * symbol. `clock_ms` was briefly a `static inline` wrapper and is now
     * retired outright, so the millisecond domain is this caller's own
     * division. */
    uint64_t start_ms = nros_platform_clock_ns() / 1000000u;
    for (;;) {
        uint64_t now_ms = nros_platform_clock_ns() / 1000000u;
        int elapsed_ms = (int)(now_ms - start_ms);
        int remaining = t - elapsed_ms;
        if (remaining <= 0) {
            break;
        }
        (void)uxr_run_session_time(&st->session, remaining);
        /* If the run returned well before `remaining`, yield ~1 ms so the next
         * pass picks up freshly-arrived inbound without busy-spinning. */
        uint64_t after_ms = nros_platform_clock_ns() / 1000000u;
        if ((after_ms - now_ms) < 1u) {
            nros_platform_sleep_ms(1);
        }
    }
    return NROS_RMW_RET_OK;
}

/* Phase 124.F.2 — session-level connectivity probe.
 *
 * micro-XRCE-DDS-Client ships `uxr_ping_agent_session`: a single
 * GET_INFO round-trip over the already-open session that doesn't
 * disturb the rest of the application's streams. One attempt per
 * call — the runtime's `timeout_ms` is the per-attempt budget. */
rmw_ret_t xrce_session_ping(rmw_session_t* session, int32_t timeout_ms) {
    if (session == NULL) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    xrce_session_state_t* st = (xrce_session_state_t*)session->backend_data;
    if (st == NULL) {
        return NROS_RMW_RET_ERROR;
    }
    int t = timeout_ms < 0 ? 0 : (int)timeout_ms;
    bool ok = uxr_ping_agent_session(&st->session, t, 1);
    return ok ? NROS_RMW_RET_OK : NROS_RMW_RET_TIMEOUT;
}
