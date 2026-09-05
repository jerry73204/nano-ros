// Services — Phase 117.X.3 + 117.12.B (cdds_request_header_t-shaped wire).
//
// Service traffic uses per-service typed Request / Response topics
// matching stock `rmw_cyclonedds_cpp`'s shape. Each typed struct
// carries the request-id correlation header inline as the first two
// fields:
//
//     struct <Pkg>::srv::dds_::<Svc>_Request_ {
//         unsigned long long rmw_writer_guid;   // lower 8 bytes of RTPS GUID
//         long long          rmw_sequence_number;  // monotonic per-client
//         /* user fields ... */
//     };
//
// This is bit-equivalent to upstream's `cdds_request_header_t
// request_header;` (see `rmw_cyclonedds_cpp/src/serdata.hpp:73-77`,
// `{uint64_t guid; int64_t seq;}` — 16 bytes) followed by user
// fields. Stock `rclcpp` clients/servers therefore match by
// `(topic_name, type_name)` and exchange byte-equal CDR.
//
// The 117.X.1 codegen helper injects the two header fields at IDL
// time when processing `.srv` inputs; consumers call
// `nros_rmw_cyclonedds_generate_from_msg(... INTERFACES <Foo.srv>)`
// and get the right typed struct without further manual work.
//
// Wire data path (per-call):
//
//   service_send_request_raw:  user-CDR bytes
//                       → build wire CDR `[encap][16-byte-header]
//                          [user CDR after-encap]`
//                       → dds_stream_read_sample into typed struct
//                       → dds_write
//   service_take_response_raw: poll reply reader, filter on
//                         (writer_guid, seq) match.
//
//   service_take_request:  dds_take typed struct
//                       → dds_stream_write_sample → wire CDR
//                       → split: (header, user payload)
//                       → stash header in slot, return slot index.
//
//   service_send_response: lookup slot → build wire CDR
//                       `[encap][header from slot][user reply]`
//                       → dds_stream_read_sample → dds_write.
//
// Slot table (32 entries, fixed) preserved verbatim from 117.7.B —
// only the wire location of the correlation pair changes.

#include "internal.hpp"

#include "nros_sertype.hpp"

#include "descriptors.hpp"
#include "qos.hpp"
#include "topic_prefix.hpp"

#include <dds/dds.h>
#include <dds/ddsi/ddsi_cdrstream.h>
#include <dds/ddsi/ddsi_serdata.h>
// phase-370 W4 — the ddsrt heap, for the transient samples below.
//
// They were `std::calloc`/`std::free`, which is the hazard
// docs/reference/cyclonedds-known-limitations.md names in as many words:
// "transient samples use `ddsrt_{malloc,calloc,free}`, never libc — RTOS heap
// is separate". `subscriber.cpp` beside this file already allocates its take
// buffer with `ddsrt_calloc`; the rule had been applied there and not here.
//
// On an RTOS the two heaps are genuinely different arenas, so a sample taken
// from libc and freed through Cyclone (or vice versa) is a cross-allocator
// free. On the host both reach `malloc`, which is why nothing surfaced until a
// cross build compiled this file — and there it surfaced as
// `'calloc' is not a member of 'std'`, a spelling complaint that says nothing
// about the heap.
#include <dds/ddsrt/heap.h>

#include <cstdint>
#include <cstring>
// The C header, not `<cstdlib>`: see `env_u64` for why the `std::` aliases
// cannot be relied on across the cross libcs this builds for.
#include <stdlib.h>
#include <new>
// ONE predicate for all three atomic-related sites in this file: this include,
// the `std::memory_order` fallback below, and the `ServiceAtomicI64` selection
// further down. They must agree, and when they did not, nothing here failed —
// the build died 1000 lines later on `ClientState has no member named
// pending_seq` (a member since replaced by the `outstanding` set, issue 0778),
// because an undefined type takes its members down with it.
//
// That is what happened on 2026-08-24: the type selection moved from platform
// names to this capability, and the include and the enum fallback stayed on the
// platform names. On threadx_linux the two disagree by construction — a 64-bit
// host build HAS the 8-byte atomic, so the capability test picks
// `std::atomic<int64_t>`, while `NROS_PLATFORM_THREADX` skipped `<atomic>`.
// Spelling the predicate once is what makes the three sites unable to drift
// apart again.
// TWO independent facts, and using `std::atomic<int64_t>` needs BOTH:
//
//   1. the target has an 8-byte atomic INSTRUCTION
//      (`__GCC_HAVE_SYNC_COMPARE_AND_SWAP_8`) — without it GCC lowers to
//      `__atomic_*_8` libatomic calls no embedded toolchain links, and the
//      failure is a link error a long way from here;
//   2. the toolchain actually ships the `<atomic>` HEADER — a freestanding
//      libstdc++ does not, and then the include itself is fatal.
//
// threadx_riscv64 is the case that separates them: riscv64 HAS the instruction
// and has NO header, so a capability-only test compiles on threadx_linux and
// dies with `fatal error: atomic: No such file or directory`. The old
// platform-name test got this right by accident — `!THREADX` covered both — and
// a test on either fact alone is wrong on one target or the other.
#if defined(__GCC_HAVE_SYNC_COMPARE_AND_SWAP_8) && defined(__has_include)
#if __has_include(<atomic>)
#define NROS_CYCLONE_HAS_STD_ATOMIC_I64 1
#endif
#endif
#ifndef NROS_CYCLONE_HAS_STD_ATOMIC_I64
#define NROS_CYCLONE_HAS_STD_ATOMIC_I64 0
#endif

#if NROS_CYCLONE_HAS_STD_ATOMIC_I64
#include <atomic>
#endif

// `<random>` is NOT part of that predicate — it is about hosted-ness, not about
// atomics, and it pairs with the `std::random_device` fallback near `env_u64`.
// Those two keep the platform test on purpose.
#if !defined(NROS_PLATFORM_FREERTOS) && !defined(NROS_PLATFORM_THREADX)
#include <random>
#endif

// Without `<atomic>` there is no `std::memory_order` either, and the non-atomic
// `ServiceAtomicI64` below still takes one in every signature.
#if !NROS_CYCLONE_HAS_STD_ATOMIC_I64
namespace std {
enum memory_order {
    memory_order_relaxed,
    memory_order_acquire,
    memory_order_release,
};
} // namespace std
#endif

namespace nros_rmw_cyclonedds {

// ---------------------------------------------------------------------------
// issue 0773 — length-or-status in ONE int32_t, after the codes went positive.
//
// Phase 376 W3.d step B renumbered `nros_rmw_ret_t` to upstream rmw's values,
// which are all NON-NEGATIVE (`ERROR` = 1, `INVALID_ARGUMENT` = 11, the
// extensions 1000+). Every helper below returns "byte count, or a status" in a
// single `int32_t`, and every caller tested `< 0` / `<= 0` for the status —
// a test that no longer fires. `NO_DATA` (1003) was therefore read as a
// 1003-byte sample on an EMPTY queue: the scratch buffer was never written,
// `taken` was set, and the shim sliced 1005 (`BUFFER_TOO_SMALL`, read as a
// length) out of a 256-byte buffer.
//
// The encoding is now explicit: a status travels NEGATED, so "is this a
// status?" is a property of the value rather than a coincidence of the code
// numbering. One spelling, used by every helper and every caller in this file.
static inline int32_t wire_status(rmw_ret_t r) {
    return -static_cast<int32_t>(r);
}
static inline bool wire_is_status(int32_t v) {
    return v < 0;
}
static inline rmw_ret_t wire_status_code(int32_t v) {
    return static_cast<rmw_ret_t>(-v);
}

namespace {

// Phase 192.4 — read a uint64 from the environment, falling back to a baked
// default. getenv returns null on RTOS targets with no environment, so the
// defaults apply there. No function-local statics (would need __cxa_guard on
// embedded); callers read once into a local where it matters.
uint64_t env_u64(const char* name, uint64_t fallback) {
    // `::getenv`, not `std::getenv` — same reason as the `::strtoull` below,
    // and phase-370 W4 is where that reason turned out to be general. Phase 203
    // recorded it for ONE symbol on ONE cross libc; the arm-none-eabi/newlib
    // cross aliases a different subset, and `getenv` is outside this one.
    // The unqualified C declaration is present on every target this builds for.
    const char* v = env_lookup(name);
    if (v == nullptr || v[0] == '\0') return fallback;
#if defined(__STDC_HOSTED__) && __STDC_HOSTED__ == 0
    // Unreachable on a freestanding target — `env_lookup` returns `nullptr`
    // there, so the check above already returned. Written out rather than left
    // to the optimiser because `strtoull` is not DECLARED under
    // `-ffreestanding`, and an undeclared call is a compile error whether or
    // not it can run.
    return fallback;
#else
    char* end = nullptr;
    // Phase 203 — use the global-namespace C name. A cross libc's `<cstdlib>`
    // does **not** alias every C function into `std::`, and WHICH subset it
    // aliases differs per libc: picolibc on the riscv64/threadx cross has
    // `getenv` and not `strtoull`; newlib on arm-none-eabi has neither. So the
    // rule is not "avoid `std::strtoull`" but "do not depend on the aliasing at
    // all here" — phase-370 W4 generalised it after the second libc disagreed.
    // `<stdlib.h>` is included below for the C declarations.
    unsigned long long parsed = ::strtoull(v, &end, 10);
    return (end != v && parsed > 0) ? static_cast<uint64_t>(parsed) : fallback;
#endif
}

// Default Cyclone service request/reply match timing (ms). Tunable at runtime
// via NROS_CYCLONE_MATCH_TIMEOUT_MS without recompiling.
constexpr uint64_t kDefaultMatchTimeoutMs = 5000;

// Selected by CAPABILITY, not by platform name. The constraint is that the
// target has no 8-byte atomic instruction: GCC then lowers std::atomic<int64_t>
// to __atomic_load_8 / __atomic_store_8 / __atomic_fetch_add_8 libatomic calls,
// which no embedded toolchain links, and the failure is a link error a long way
// from this line. Every 32-bit MCU target is in this set -- it is what the
// FreeRTOS and ThreadX names here used to stand in for, and Zephyr on Cortex-M
// is the same case that the name-based test missed (thumbv7em, 2026-08-24).
//
// Doing it by capability also keeps real atomics wherever they exist, including
// Zephyr on native_sim and a 64-bit ThreadX cross, which a platform-name test
// would have thrown away.
//
// Dropping the ordering is sound here because this struct is single-threaded by
// construction, not merely by convention: `pending_request[kWireScratch]` and
// `pending_request_len` sit beside these counters as PLAIN members and are
// mutated in the same calls. A caller that genuinely raced would be tearing a
// 64 KiB buffer long before the sequence number mattered.
#if !NROS_CYCLONE_HAS_STD_ATOMIC_I64
struct ServiceAtomicI64 {
    int64_t value;

    explicit ServiceAtomicI64(int64_t initial = 0) : value(initial) {}

    int64_t load(std::memory_order) const { return value; }
    void store(int64_t next, std::memory_order) { value = next; }
    int64_t fetch_add(int64_t delta, std::memory_order) {
        int64_t previous = value;
        value += delta;
        return previous;
    }
};
#else
using ServiceAtomicI64 = std::atomic<int64_t>;
#endif

constexpr std::size_t kRequestSlots = 32;

// Issue 0778 — how many requests one client may have in flight. Eight, not
// thirty-two: this is a per-CLIENT cost (8 bytes + a bool each), a client that
// needs more than a handful of concurrent calls is doing something the RTOS
// side of this ABI does not target, and running out is a loud WOULD_BLOCK
// rather than a silent abandon.
constexpr std::size_t kMaxOutstandingRequests = 8;
constexpr std::size_t kMaxTopicName = 256;
constexpr uint8_t kCdrLeHeader[4] = {0x00, 0x01, 0x00, 0x00};

// Wire-framing field widths. These are intentionally *separate* consts
// even where they share the value 4 — conflating the CDR encapsulation
// header with a CDR length prefix or the GetResult status field is the
// exact off-by-N class this section (192.2) exists to kill.
constexpr std::size_t kEncapLen = sizeof(kCdrLeHeader);      // 4-byte CDR encapsulation header
constexpr std::size_t kGuidBytes = 8;                        // request_header GUID (LE u64)
constexpr std::size_t kSeqBytes = 8;                         // request_header sequence (LE u64)
constexpr std::size_t kHeaderBytes = kGuidBytes + kSeqBytes; // 16-byte inlined request_header
constexpr std::size_t kCdrLenPrefix = sizeof(uint32_t); // 4-byte CDR sequence/array length field
constexpr std::size_t kStatusFieldLen = 4;              // GetResult status int8 + 3 pad

// Round @p pos up to the 4-byte CDR member alignment.
inline std::size_t cdr_align4(std::size_t pos) {
    return (pos + 3u) & ~std::size_t{3u};
}
// Per-call scratch ceiling. Tunable via env if a future user needs
// it; 64 KiB covers ROS 2's default service payload size budget.
constexpr std::size_t kWireScratch = 65536;

struct RequestId {
    uint64_t guid;
    int64_t seq;
};

struct RequestSlot {
    RequestId id{};
    bool in_use{false};
};

struct ServerState {
    dds_entity_t request_topic{0};
    dds_entity_t reply_topic{0};
    dds_entity_t reader{0};
    dds_entity_t writer{0};
    const dds_topic_descriptor_t* req_desc{nullptr};
    const dds_topic_descriptor_t* rep_desc{nullptr};
    RequestSlot slots[kRequestSlots];
};

struct ClientState {
    dds_entity_t request_topic{0};
    dds_entity_t reply_topic{0};
    dds_entity_t writer{0};
    dds_entity_t reader{0};
    const dds_topic_descriptor_t* req_desc{nullptr};
    const dds_topic_descriptor_t* rep_desc{nullptr};
    uint64_t my_guid{0};
    ServiceAtomicI64 next_seq{0};
    // Issue 0778 — the sequence ids this client is waiting on. Was a single
    // `pending_seq`, so a second send ABANDONED the first: it overwrote the one
    // id, and `take_response` then dropped the older reply as "for a different
    // in-flight call". Now a set, so N calls can be outstanding and each reply
    // retires its own.
    //
    // Ids only, deliberately. Replicating the 64 KiB `pending_request` staging
    // buffer N times is not affordable on the targets this backend runs on —
    // and it is not needed, because staging is only for the window BEFORE the
    // request writer has matched a server reader (VOLATILE QoS drops a write
    // sent earlier). After the match, sends go straight out and hold no buffer.
    int64_t outstanding[kMaxOutstandingRequests]{};
    bool outstanding_in_use[kMaxOutstandingRequests]{};
    // Phase 130.8 — the ONE pre-match staging slot, and the id it holds.
    // Service QoS is VOLATILE, so writing before the match can silently drop
    // the request.
    uint8_t pending_request[kWireScratch]{};
    std::size_t pending_request_len{0};
    int64_t pending_request_seq{-1};
};

bool service_topic_name(const char* service_name, const char* prefix, const char* suffix, char* out,
                        std::size_t out_cap) {
    if (service_name == nullptr || prefix == nullptr || suffix == nullptr || out == nullptr) {
        return false;
    }
    char with_suffix[kMaxTopicName];
    std::size_t blen = std::strlen(service_name);
    std::size_t slen = std::strlen(suffix);
    if (blen + slen + 1 > sizeof(with_suffix)) return false;
    std::memcpy(with_suffix, service_name, blen);
    std::memcpy(with_suffix + blen, suffix, slen);
    with_suffix[blen + slen] = '\0';
    return topic_prefix::apply(with_suffix, prefix, out, out_cap);
}

// Build the suffixed type name `<base>_Request_` / `<base>_Response_`
// the codegen helper (117.X.1) registers. Returns false on overflow.
//
// Phase 11W.12: the nros codegen emits SERVICE_NAME with a trailing
// underscore (`<pkg>::srv::dds_::<Svc>_`, mirroring the message
// `<Type>_` convention), but the registered DDS request/response types
// — and stock `rmw_cyclonedds_cpp` — are `<pkg>::srv::dds_::<Svc>_Request_`
// / `_Response_` with a *single* underscore before the suffix. Strip one
// trailing underscore from the base so both the no-trailing-`_` form
// (used by the backend's own roundtrip tests) and the codegen's
// trailing-`_` form resolve to the same registered descriptor.
bool service_type_name(const char* base, const char* suffix, char* out, std::size_t out_cap) {
    std::size_t blen = std::strlen(base);
    if (blen > 0 && base[blen - 1] == '_') {
        --blen;
    }
    std::size_t slen = std::strlen(suffix);
    if (blen + slen + 1 > out_cap) return false;
    std::memcpy(out, base, blen);
    std::memcpy(out + blen, suffix, slen);
    out[blen + slen] = '\0';
    return true;
}

uint32_t cdr_xcdr_version(const uint8_t* bytes) {
    uint8_t lo = bytes[1];
    if (lo == 0x06 || lo == 0x07 || lo == 0x0a || lo == 0x0b) return 2;
    return 1;
}

// Pull the lower 8 bytes of the 16-byte RTPS GUID from a Cyclone
// writer. Upstream `rmw_cyclonedds_cpp` stashes a 64-bit guid in
// `cdds_request_header_t` rather than the full 128-bit RTPS GUID, so
// we follow the same convention to stay wire-compatible. Returns 0 if
// dds_get_guid fails — caller must fall back to a random value.
uint64_t writer_guid_lo64(dds_entity_t writer) {
    dds_guid_t g{};
    if (dds_get_guid(writer, &g) != DDS_RETCODE_OK) return 0;
    uint64_t v = 0;
    std::memcpy(&v, g.v, 8);
    return v;
}

// Encode int64 little-endian into 8 bytes.
inline void put_le64(uint8_t* out, int64_t v) {
    for (int i = 0; i < 8; ++i) {
        out[i] = static_cast<uint8_t>((v >> (i * 8)) & 0xff);
    }
}
inline int64_t get_le64(const uint8_t* in) {
    int64_t v = 0;
    for (int i = 0; i < 8; ++i) {
        v |= static_cast<int64_t>(in[i]) << (i * 8);
    }
    return v;
}

bool type_ends_with(const dds_topic_descriptor_t* desc, const char* suffix) {
    if (desc == nullptr || desc->m_typename == nullptr || suffix == nullptr) {
        return false;
    }
    const std::size_t len = std::strlen(desc->m_typename);
    const std::size_t slen = std::strlen(suffix);
    return len >= slen && std::strcmp(desc->m_typename + len - slen, suffix) == 0;
}

struct DdsSequenceInt32 {
    uint32_t _maximum;
    uint32_t _length;
    int32_t* _buffer;
    bool _release;
};

// (Issue #68) `insert_goal_id_len_at` was removed: it re-inserted a pre-233.6
// `uint32(16)` goal_id length prefix on the service-request receive path, which a
// real `rcl_action` peer never sends and the post-233.6 action core no longer
// reads. See `split_wire_header`.

// Construct the wire CDR for a typed struct that has the 16-byte
// request_header inlined at offset 0. Inputs:
//   user_bytes    runtime-supplied CDR with 4-byte encap + user fields
//                 (no header).
//   id            request_id to inject.
// Outputs:
//   wire_cdr      buffer of size at least len(user_bytes) + 16.
// Returns total wire byte count, or negative on error.
int32_t build_wire_with_header(const uint8_t* user_bytes, size_t user_len, const RequestId& id,
                               uint8_t* wire_cdr, size_t wire_cap) {
    if (user_len < kEncapLen) return wire_status(NROS_RMW_RET_INVALID_ARGUMENT);
    size_t total = user_len + kHeaderBytes;
    if (total > wire_cap) return wire_status(NROS_RMW_RET_BUFFER_TOO_SMALL);
    // Encap copied verbatim.
    std::memcpy(wire_cdr, user_bytes, kEncapLen);
    // Little-endian guid.
    put_le64(wire_cdr + kEncapLen, static_cast<int64_t>(id.guid));
    // Little-endian seq.
    put_le64(wire_cdr + kEncapLen + kGuidBytes, id.seq);
    // User payload after encap.
    std::memcpy(wire_cdr + kEncapLen + kHeaderBytes, user_bytes + kEncapLen, user_len - kEncapLen);
    return static_cast<int32_t>(total);
}

// Inverse: parse the wire CDR (with leading header) into
// `(out_id, user_bytes_with_encap)`. The wire's 4-byte encap is
// preserved in `user_out` so the runtime sees a normal CDR-shaped
// payload (encap + user fields).
//
// Returns user-payload length (incl. 4-byte encap) on success, or
// negative error.
int32_t split_wire_header(const uint8_t* wire_cdr, size_t wire_len,
                          const dds_topic_descriptor_t* payload_desc, RequestId* out_id,
                          uint8_t* user_out, size_t user_cap) {
    if (wire_len < kEncapLen + kHeaderBytes) return wire_status(NROS_RMW_RET_INVALID_ARGUMENT);
    if (out_id != nullptr) {
        out_id->guid = static_cast<uint64_t>(get_le64(wire_cdr + kEncapLen));
        out_id->seq = get_le64(wire_cdr + kEncapLen + kGuidBytes);
    }
    size_t user_len = wire_len - kHeaderBytes; // (encap stays + user fields)
    if (user_len > user_cap) return wire_status(NROS_RMW_RET_BUFFER_TOO_SMALL);
    // Encap.
    std::memcpy(user_out, wire_cdr, kEncapLen);
    // User fields.
    std::memcpy(user_out + kEncapLen, wire_cdr + kEncapLen + kHeaderBytes, user_len - kEncapLen);
    // Phase 233.6 completion (issue #68) — do NOT re-insert a goal_id `uint32(16)`
    // length prefix here. ROS 2 `rcl_action` (over rmw_cyclonedds_cpp) serialises
    // the SendGoal/GetResult request's `goal_id` as a fixed `uint8[16]` array — no
    // length prefix — and the nano action core now reads it that way too
    // (`action_core::read_goal_id`, post-233.6). The old `insert_goal_id_len_at`
    // call mirrored the pre-233.6 prefixed framing; 233.6 dropped the subscriber.cpp
    // insert mirror but missed THIS service-request site, so a real rcl_action client's
    // goal arrived with a spurious `10 00 00 00` before the UUID → `order` read 4 bytes
    // early → "Goal was rejected" (order garbage). The wire payload already matches the
    // bare-array form after the request-header strip above; pass it through unchanged.
    (void)payload_desc;
    return static_cast<int32_t>(user_len);
}

// Run dds_stream_read_sample on @p wire_cdr, then dds_write. Caller
// owns @p wire_cdr.
// Publish the caller's wire CDR as a blob. Issues 0970 and 0969.
//
// This used to decode the bytes into a typed sample (`dds_stream_read_sample`)
// and hand THAT to `dds_write`, so Cyclone re-serialised them: a decode, an
// encode and a heap allocation per publish, to put back on the wire what the
// caller already had. `publisher.cpp` stopped doing that when issue 0970's
// message half landed; this is the service half, and it is the same three lines.
//
// The recorded blocker — Cyclone must expose `dds_writer_lookup_serdatatype` —
// was never the real one. That API recovers a sertype you do NOT own; registering
// our own with `dds_create_topic_sertype` makes the question moot, which is what
// the topics below now do.
//
// Two per-type adapters go with it, both artefacts of the round trip rather than
// of the wire. The `_SendGoal_*` / `_GetResult_Request_` branch memcpy'd the
// payload onto a typed struct because the stream reader mishandled it, and
// `write_fibonacci_get_result_response` hand-built the generated C layout because
// `dds_stream_read_sample` CRASHES on that type (phase 171.0.b). Nothing reads a
// stream here now, so neither has anything to work around.
rmw_ret_t write_typed(dds_entity_t writer, const dds_topic_descriptor_t* desc,
                      const uint8_t* wire_cdr, size_t wire_len) {
    if (writer <= 0 || desc == nullptr || wire_cdr == nullptr || wire_len < kEncapLen) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    // The sample IS the caller's bytes, header included: `from_sample` copies
    // them into the serdata and that is the whole of publish.
    const NrosCdrBlob blob{wire_cdr, wire_len};
    dds_return_t r = dds_write(writer, &blob);
    return (r == DDS_RETCODE_OK) ? NROS_RMW_RET_OK : NROS_RMW_RET_ERROR;
}

// Take the wire CDR straight out of the serdata. Issue 0969.
//
// Returns wire byte count, NROS_RMW_RET_NO_DATA, or negative error.
//
// This used to `dds_take` a typed sample, then re-serialise it through a
// `dds_ostream_t` and copy THAT out — a full decode plus a full encode plus two
// heap allocations, per take, to hand back bytes that were already on the wire.
// The retired `sertype_min` builder recorded the raw-CDR path as blocked on
// Cyclone exposing `dds_writer_lookup_serdatatype`; that blocker was real for
// PUBLISH and never existed here. `dds_takecdr` needs only a reader entity, and the reader already
// owns its sertype from `dds_create_topic(desc)`.
//
// `ddsi_serdata_size` counts the 4-byte `CDRHeader` and `to_ser` copies from
// `&d->hdr`, so one call delivers header + payload in the WIRE's own
// representation. The old path synthesised an XCDR1 little-endian header of its
// own, which is why the caller used to see a re-encoding rather than what the
// peer sent.
//
// The per-type adapters that stood in front of this are gone with it. They
// existed because the typed round trip mangled particular action messages; there
// is no round trip now, so `Fibonacci_GetResult_Response_` needs no hand-built
// struct and `_SendGoal_*` needs no memcpy branch. `ros2_action_e2e` is what
// makes their removal checkable — both directions against a real ROS 2 peer.
int32_t take_typed_wire(dds_entity_t reader, uint8_t* out_buf, size_t out_cap) {
    if (reader <= 0 || out_buf == nullptr) return wire_status(NROS_RMW_RET_INVALID_ARGUMENT);

    struct ddsi_serdata* d = nullptr;
    dds_sample_info_t si[1];
    for (;;) {
        dds_return_t taken = dds_takecdr(reader, &d, 1, si, DDS_ANY_STATE);
        if (taken < 0) return wire_status(NROS_RMW_RET_ERROR);
        if (taken == 0) return wire_status(NROS_RMW_RET_NO_DATA);
        if (si[0].valid_data) break;
        // A disposal or unregister carries no sample; drop it and look again
        // rather than reporting NO_DATA, which would end the caller's drain.
        ddsi_serdata_unref(d);
        d = nullptr;
    }

    const uint32_t total = ddsi_serdata_size(d);
    if (out_cap < total) {
        ddsi_serdata_unref(d);
        return wire_status(NROS_RMW_RET_BUFFER_TOO_SMALL);
    }
    ddsi_serdata_to_ser(d, 0, total, out_buf);
    ddsi_serdata_unref(d);
    return static_cast<int32_t>(total);
}

// Action sub-services reuse one service-create path but each carries a
// distinct DDS type. The action layer (`executor/action.rs`) passes the
// bare action type `<pkg>::action::dds_::<A>_` for both the send_goal
// and get_result services, distinguishing them only by the service
// name suffix `<action>/_action/{send_goal,get_result}` (see
// `ActionInfo::{send_goal,get_result}_key`). Map that suffix to the
// rosidl-synthesised wrapper base — `<A>_SendGoal_` / `<A>_GetResult_`
// — so the `_Request_`/`_Response_` lookup resolves the right
// descriptor. Non-action services pass through unchanged. This keeps
// the backend-agnostic contract (and the zenoh keyexpr) untouched.
bool action_effective_base(const char* service_name, const char* type_name, char* out,
                           std::size_t out_cap) {
    const char* infix = nullptr;
    std::size_t nlen = std::strlen(service_name);
    auto ends_with = [&](const char* suf) {
        std::size_t slen = std::strlen(suf);
        return nlen >= slen && std::strcmp(service_name + nlen - slen, suf) == 0;
    };
    if (ends_with("/_action/send_goal")) {
        infix = "_SendGoal_";
    } else if (ends_with("/_action/get_result")) {
        infix = "_GetResult_";
    }
    // issue #234 — idempotent infix injection. Two callers pass different
    // `type_name` forms for the same action send_goal / get_result service: the
    // raw / C / C++ path passes the BARE action type `<pkg>::action::dds_::<A>_`
    // and relies on us appending the wrapper infix (`_SendGoal_` / `_GetResult_`)
    // to reach `<A>_SendGoal_`; the typed Rust path (`node.rs` +
    // `executor/action.rs`) advertises the ALREADY-per-channel wrapper type
    // `<A>_SendGoal_` (what a real rcl_action peer matches on). Appending it
    // again produced the doubled `<A>_SendGoal_SendGoal_` that resolved no
    // descriptor → every rust cyclone action failed at creation with
    // UNSUPPORTED. Detect an already-suffixed form and pass it through unchanged.
    if (infix != nullptr) {
        std::size_t tlen = std::strlen(type_name);
        std::size_t ilen = std::strlen(infix);
        if (tlen >= ilen && std::strcmp(type_name + tlen - ilen, infix) == 0) {
            infix = nullptr;
        }
    }
    if (infix == nullptr) {
        std::size_t blen = std::strlen(type_name);
        if (blen + 1 > out_cap) return false;
        std::memcpy(out, type_name, blen + 1);
        return true;
    }
    // Strip the single trailing `_` from the action base, append the
    // wrapper infix (which itself ends in `_`, the marker the later
    // `service_type_name` strips before adding `_Request_`).
    std::size_t blen = std::strlen(type_name);
    if (blen > 0 && type_name[blen - 1] == '_') --blen;
    std::size_t ilen = std::strlen(infix);
    if (blen + ilen + 1 > out_cap) return false;
    std::memcpy(out, type_name, blen);
    std::memcpy(out + blen, infix, ilen);
    out[blen + ilen] = '\0';
    return true;
}

// `ros_form_to_dds` moved to descriptors.cpp (shared with `action_topic_type`
// / `find_descriptor`); declared in descriptors.hpp.

bool descriptors_for_service(const char* service_name, const char* type_name,
                             const dds_topic_descriptor_t** out_req,
                             const dds_topic_descriptor_t** out_rep) {
    char dds_type[kMaxTopicName];
    if (!ros_form_to_dds(type_name, dds_type, sizeof(dds_type))) {
        return false;
    }
    char base[kMaxTopicName];
    if (!action_effective_base(service_name, dds_type, base, sizeof(base))) {
        return false;
    }
    char req_type[kMaxTopicName];
    char rep_type[kMaxTopicName];
    if (!service_type_name(base, "_Request_", req_type, sizeof(req_type))) {
        return false;
    }
    if (!service_type_name(base, "_Response_", rep_type, sizeof(rep_type))) {
        return false;
    }
    *out_req = find_descriptor(req_type);
    *out_rep = find_descriptor(rep_type);
    return *out_req != nullptr && *out_rep != nullptr;
}

uint64_t random_seed_word() {
#if defined(NROS_PLATFORM_FREERTOS) || defined(NROS_PLATFORM_THREADX)
    return platform_random_u64();
#else
    std::random_device rd;
    return (static_cast<uint64_t>(rd()) << 32) ^ rd();
#endif
}

uint64_t random_guid64() {
    return random_seed_word();
}

bool request_writer_matched(dds_entity_t writer) {
    dds_publication_matched_status_t status{};
    return dds_get_publication_matched_status(writer, &status) == DDS_RETCODE_OK &&
           status.current_count > 0;
}

// Issue 0778 — the outstanding-request set. Linear over eight entries; a map
// would cost more than it saves at this size.
bool claim_outstanding(ClientState* state, int64_t seq) {
    for (std::size_t i = 0; i < kMaxOutstandingRequests; ++i) {
        if (!state->outstanding_in_use[i]) {
            state->outstanding[i] = seq;
            state->outstanding_in_use[i] = true;
            return true;
        }
    }
    return false;
}

bool release_outstanding(ClientState* state, int64_t seq) {
    for (std::size_t i = 0; i < kMaxOutstandingRequests; ++i) {
        if (state->outstanding_in_use[i] && state->outstanding[i] == seq) {
            state->outstanding_in_use[i] = false;
            return true;
        }
    }
    return false;
}

bool any_outstanding(const ClientState* state) {
    for (std::size_t i = 0; i < kMaxOutstandingRequests; ++i) {
        if (state->outstanding_in_use[i]) return true;
    }
    return false;
}

rmw_ret_t maybe_flush_request(ClientState* state) {
    if (state == nullptr || state->pending_request_len == 0) {
        return NROS_RMW_RET_OK;
    }
    // Services use RELIABLE + VOLATILE QoS. A write before the client
    // request writer has matched the server request reader can be accepted
    // locally but never delivered, which loses the first nonblocking action
    // send_goal request. Keep the buffered request pending until discovery
    // reports a match.
    if (!request_writer_matched(state->writer)) {
        return NROS_RMW_RET_OK;
    }
    rmw_ret_t r = write_typed(state->writer, state->req_desc, state->pending_request,
                              state->pending_request_len);
    if (r == NROS_RMW_RET_OK) {
        state->pending_request_len = 0;
        state->pending_request_seq = -1;
    }
    return r;
}

} // namespace

// =========================================================================
// Service server
// =========================================================================

rmw_ret_t service_create(const rmw_node_t* node, const rmw_service_type_support_t* type_support,
                         const char* service_name, uint32_t /*domain_id*/,
                         const rmw_qos_profile_t* qos, rmw_service_t* out) {
    /* phase-406 W1 — one argument in, two locals out, so the body below is
       unchanged. A NULL type support is INVALID_ARGUMENT rather than an
       empty type: the identity is what the entity is keyed on, and one
       created without it matches nothing and reports nothing. */
    if (type_support == nullptr) return NROS_RMW_RET_INVALID_ARGUMENT;
    const char* type_name = type_support->type_name;
    (void)type_name;
    // Phase 376 W5/B1 — the entity is created ON ITS NODE, as upstream does.
    // The node carries the route to its session (our `context`).
    if (node == nullptr) return NROS_RMW_RET_INVALID_ARGUMENT;
    rmw_session_t* session = node->session;
    if (out == nullptr || session == nullptr || service_name == nullptr || type_name == nullptr) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    out->backend_data = nullptr;

    dds_entity_t pp = session_participant(session);
    if (pp == 0) return NROS_RMW_RET_ERROR;

    const dds_topic_descriptor_t* req_desc = nullptr;
    const dds_topic_descriptor_t* rep_desc = nullptr;
    if (!descriptors_for_service(service_name, type_name, &req_desc, &rep_desc)) {
        return NROS_RMW_RET_UNSUPPORTED;
    }

    char req_topic[kMaxTopicName];
    char rep_topic[kMaxTopicName];
    if (!service_topic_name(service_name, "rq", "Request", req_topic, sizeof(req_topic)) ||
        !service_topic_name(service_name, "rr", "Reply", rep_topic, sizeof(rep_topic))) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }

    auto* state = new (std::nothrow) ServerState();
    if (state == nullptr) return NROS_RMW_RET_BAD_ALLOC;
    state->req_desc = req_desc;
    state->rep_desc = rep_desc;

    // Issue 0970 — OUR sertype, not Cyclone's generated one. Cyclone's deals in
    // typed C structs, which is what forced the CDR round trip in both
    // directions; `create_nros_sertype` makes the sample a span of CDR. On
    // failure the sertype is ours to free, on success the domain owns it.
    struct ddsi_sertype* req_st = create_nros_sertype(req_desc);
    struct ddsi_sertype* rep_st = create_nros_sertype(rep_desc);
    if (req_st == nullptr || rep_st == nullptr) {
        if (req_st != nullptr) ddsi_sertype_unref(req_st);
        if (rep_st != nullptr) ddsi_sertype_unref(rep_st);
        // A keyed descriptor is the only way this fails today, and a service
        // type has no keys — so this is a contract breach, not a runtime
        // condition a caller can recover from.
        return NROS_RMW_RET_ERROR;
    }
    state->request_topic =
        dds_create_topic_sertype(pp, req_topic, &req_st, nullptr, nullptr, nullptr);
    state->reply_topic =
        dds_create_topic_sertype(pp, rep_topic, &rep_st, nullptr, nullptr, nullptr);
    if (state->request_topic < 0 || state->reply_topic < 0) {
        if (state->request_topic > 0) (void)dds_delete(state->request_topic);
        if (state->reply_topic > 0) (void)dds_delete(state->reply_topic);
        delete state;
        return NROS_RMW_RET_ERROR;
    }

    // Phase 193.1b: honour the caller's profile, defaulting to
    // `rmw_qos_profile_services_default` (RELIABLE + VOLATILE +
    // KEEP_LAST(10)) — the stock-RMW-interop default (without it Cyclone
    // defaults to KEEP_LAST(1), surprising stock clients). One profile
    // applied to both request reader + reply writer.
    rmw_qos_profile_t svc_qos = qos != nullptr ? *qos : NROS_RMW_QOS_PROFILE_SERVICES_DEFAULT;
    dds_qos_t* dq_reader = make_dds_qos(&svc_qos);
    dds_qos_t* dq_writer = make_dds_qos(&svc_qos);
    state->reader = dds_create_reader(pp, state->request_topic, dq_reader, nullptr);
    state->writer = dds_create_writer(pp, state->reply_topic, dq_writer, nullptr);
    if (dq_reader != nullptr) dds_delete_qos(dq_reader);
    if (dq_writer != nullptr) dds_delete_qos(dq_writer);
    if (state->reader < 0 || state->writer < 0) {
        if (state->reader > 0) (void)dds_delete(state->reader);
        if (state->writer > 0) (void)dds_delete(state->writer);
        (void)dds_delete(state->request_topic);
        (void)dds_delete(state->reply_topic);
        delete state;
        return NROS_RMW_RET_ERROR;
    }

    out->backend_data = state;
    // Phase 177.36 — register both endpoints with the node graph (server:
    // request reader + reply writer; client: request writer + reply reader).
    graph_track_reader(session_graph(session), state->reader);
    graph_track_writer(session_graph(session), state->writer);
    return NROS_RMW_RET_OK;
}

rmw_ret_t service_destroy(rmw_service_t* server) {
    if (server == nullptr || server->backend_data == nullptr) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    auto* state = static_cast<ServerState*>(server->backend_data);
    dds_return_t rc = DDS_RETCODE_OK;
    if (state->reader > 0 && dds_delete(state->reader) < 0) rc = DDS_RETCODE_ERROR;
    if (state->writer > 0 && dds_delete(state->writer) < 0) rc = DDS_RETCODE_ERROR;
    if (state->request_topic > 0 && dds_delete(state->request_topic) < 0) rc = DDS_RETCODE_ERROR;
    if (state->reply_topic > 0 && dds_delete(state->reply_topic) < 0) rc = DDS_RETCODE_ERROR;
    delete state;
    server->backend_data = nullptr;
    return rc < 0 ? NROS_RMW_RET_ERROR : NROS_RMW_RET_OK;
}

static int32_t service_take_request_len(const rmw_service_t* server, uint8_t* buf, size_t buf_len,
                                        int64_t* seq_out) {
    if (server == nullptr || server->backend_data == nullptr || buf == nullptr) {
        return wire_status(NROS_RMW_RET_INVALID_ARGUMENT);
    }
    auto* state = static_cast<ServerState*>(server->backend_data);

    // issue 1088 — reserve the correlation slot BEFORE the destructive take.
    //
    // `take_typed_wire` is `dds_takecdr` + `ddsi_serdata_unref`: the sample is
    // consumed and freed. This used to run first, and only then look for a
    // free slot; with `kRequestSlots` requests outstanding the take succeeded,
    // the slot search failed, and the caller was told `taken = false` — which
    // upstream defines as "nothing was consumed" (`rmw.h:2348`). The request
    // was gone. A busy server stopped answering and looked idle.
    //
    // The XRCE sibling already had this right: it returns WOULD_BLOCK without
    // popping its ring (`xrce/src/service.c:293`). Same contract, two
    // implementations, and this was the one that lost data.
    std::size_t slot = kRequestSlots;
    for (std::size_t i = 0; i < kRequestSlots; ++i) {
        if (!state->slots[i].in_use) {
            slot = i;
            break;
        }
    }
    if (slot == kRequestSlots) {
        // Nothing consumed: the sample stays on the reader for the next take.
        return wire_status(NROS_RMW_RET_WOULD_BLOCK);
    }

    uint8_t wire[kWireScratch];
    int32_t wire_len = take_typed_wire(state->reader, wire, sizeof(wire));
    if (wire_is_status(wire_len) || wire_len == 0) return wire_len;

    RequestId id{};
    int32_t user_len =
        split_wire_header(wire, static_cast<size_t>(wire_len), state->req_desc, &id, buf, buf_len);
    if (wire_is_status(user_len)) return user_len;

    // Commit the slot reserved above with the (writer_guid, seq) pair so the
    // matching `service_send_response` can echo it back. The search happened
    // before the take, so this cannot fail.
    state->slots[slot].id = id;
    state->slots[slot].in_use = true;
    if (seq_out != nullptr) *seq_out = static_cast<int64_t>(slot);
    return user_len;
}

/* Phase 376 W3.b/W3.d step A — upstream's shape over the unchanged body above.
 * A THIN adapter rather than a rewrite: the length-returning logic below has
 * error paths that are easy to get subtly wrong (WOULD_BLOCK is an error here,
 * not "nothing to take"), so it is preserved verbatim and only the reporting
 * convention is translated. NO_DATA is the one code that becomes
 * `taken = false` with OK. */
rmw_ret_t service_take_request(const rmw_service_t* server, rmw_mut_byte_span_t* request,
                               int64_t* seq_out, bool* taken) {
    /* phase-406 W2 — see the XRCE sibling: one span in, the old names out. */
    if (request == nullptr) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    uint8_t* buf = request->data;
    size_t buf_len = request->capacity;
    size_t* out_len = &request->len;
    if (out_len == nullptr || taken == nullptr) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    int32_t n = service_take_request_len(server, buf, buf_len, seq_out);
    if (wire_is_status(n)) {
        const rmw_ret_t st = wire_status_code(n);
        // An empty queue and a would-block are NOT failures: report
        // `taken = false` with OK, which is what the shim's contract says.
        if (st == NROS_RMW_RET_NO_DATA || st == NROS_RMW_RET_WOULD_BLOCK) {
            *taken = false;
            return NROS_RMW_RET_OK;
        }
        return st;
    }
    *out_len = (size_t)n;
    *taken = true;
    return NROS_RMW_RET_OK;
}

rmw_ret_t service_has_request(rmw_service_t* server, bool* out_has_request) {
    // Phase 376 W3.d step A — flag out, status returned. Note the middle case:
    // a failing `dds_get_status_changes` used to return 0, indistinguishable
    // from "no request pending". It is now an error the caller can see.
    if (out_has_request == nullptr) return NROS_RMW_RET_INVALID_ARGUMENT;
    if (server == nullptr || server->backend_data == nullptr) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    auto* state = static_cast<ServerState*>(server->backend_data);

    // Issue 0778 — PEEK the reader, do not read the status flag.
    //
    // `dds_get_status_changes` CLEARS the status it reports. Two requests
    // arriving between two polls therefore produce ONE change: the server took
    // the first, and every later `has_request` answered false because no NEW
    // change had been recorded — the second request sat readable in the reader
    // and was answered only if a third happened to arrive and re-arm the edge.
    // A service that silently stranded every request after the first in a
    // batch. `service_two_outstanding` fails on the old code for exactly this.
    //
    // `subscriber.cpp` hit the same class and solved it by answering a
    // conservative `true`. That is not available here: `service_smoke` asserts
    // has_request is FALSE with no traffic, and it is right to — an
    // unconditional yes makes the probe useless for a caller deciding whether
    // to allocate a request buffer.
    //
    // So peek instead. `dds_read` does not consume the sample, and
    // `take_typed_wire`'s `dds_take` uses no state mask, so it will still take
    // a sample this call has marked READ. Accurate, and edge-free.
    void* samples[1] = {nullptr};
    dds_sample_info_t si[1];
    dds_return_t n = dds_read(state->reader, samples, si, 1, 1);
    if (n < 0) {
        return NROS_RMW_RET_ERROR;
    }
    *out_has_request = (n > 0 && si[0].valid_data);
    if (n > 0) {
        (void)dds_return_loan(state->reader, samples, n);
    }
    return NROS_RMW_RET_OK;
}

rmw_ret_t service_send_response(const rmw_service_t* server, int64_t seq,
                                rmw_byte_span_t response) {
    const uint8_t* data = response.data;
    size_t len = response.len;
    if (server == nullptr || server->backend_data == nullptr || data == nullptr || seq < 0 ||
        static_cast<std::size_t>(seq) >= kRequestSlots) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    auto* state = static_cast<ServerState*>(server->backend_data);
    auto& slot = state->slots[seq];
    if (!slot.in_use) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    // Wait for the reply reader before writing (services are RELIABLE +
    // VOLATILE, so a write before the reader matches is silently dropped).
    // Prefer the firm `current_count > 0` match (the nano-ros↔nano-ros fast
    // path). But stock `rmw_cyclonedds_cpp` clients on Cyclone 0.10.5 can leave
    // the writer's `current_count` at 0 even after the reply reader has been
    // discovered (`total_count > 0`) and is waiting — an under-reported
    // cross-RMW match-state. In that case, after a short grace, write anyway:
    // the discovered reader is present and the VOLATILE write reaches it.
    // Without this the server hangs the full timeout and the stock
    // `ros2 service call` gives up (117.12.B.1).
    const uint64_t deadline =
        platform_now_ms() + env_u64("NROS_CYCLONE_MATCH_TIMEOUT_MS", kDefaultMatchTimeoutMs);
    const uint64_t grace_until = platform_now_ms() + 750;
    bool ready = false;
    while (platform_now_ms() < deadline) {
        dds_publication_matched_status_t st{};
        if (dds_get_publication_matched_status(state->writer, &st) == DDS_RETCODE_OK) {
            if (st.current_count > 0) {
                ready = true;
                break;
            }
            if (st.total_count > 0 && platform_now_ms() >= grace_until) {
                ready = true;
                break;
            }
        }
        platform_sleep_ms(5);
    }
    if (!ready) return NROS_RMW_RET_TIMEOUT;

    uint8_t wire[kWireScratch];
    int32_t wire_len = build_wire_with_header(data, len, slot.id, wire, sizeof(wire));
    rmw_ret_t r;
    if (wire_is_status(wire_len)) {
        r = wire_status_code(wire_len);
    } else {
        r = write_typed(state->writer, state->rep_desc, wire, static_cast<size_t>(wire_len));
    }
    slot.in_use = false;
    return r;
}

// =========================================================================
// Service client
// =========================================================================

rmw_ret_t client_create(const rmw_node_t* node, const rmw_service_type_support_t* type_support,
                        const char* service_name, uint32_t /*domain_id*/,
                        const rmw_qos_profile_t* qos, rmw_client_t* out) {
    /* phase-406 W1 — one argument in, two locals out, so the body below is
       unchanged. A NULL type support is INVALID_ARGUMENT rather than an
       empty type: the identity is what the entity is keyed on, and one
       created without it matches nothing and reports nothing. */
    if (type_support == nullptr) return NROS_RMW_RET_INVALID_ARGUMENT;
    const char* type_name = type_support->type_name;
    (void)type_name;
    // Phase 376 W5/B1 — the entity is created ON ITS NODE, as upstream does.
    // The node carries the route to its session (our `context`).
    if (node == nullptr) return NROS_RMW_RET_INVALID_ARGUMENT;
    rmw_session_t* session = node->session;
    if (out == nullptr || session == nullptr || service_name == nullptr || type_name == nullptr) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    out->backend_data = nullptr;

    dds_entity_t pp = session_participant(session);
    if (pp == 0) return NROS_RMW_RET_ERROR;

    const dds_topic_descriptor_t* req_desc = nullptr;
    const dds_topic_descriptor_t* rep_desc = nullptr;
    if (!descriptors_for_service(service_name, type_name, &req_desc, &rep_desc)) {
        return NROS_RMW_RET_UNSUPPORTED;
    }

    char req_topic[kMaxTopicName];
    char rep_topic[kMaxTopicName];
    if (!service_topic_name(service_name, "rq", "Request", req_topic, sizeof(req_topic)) ||
        !service_topic_name(service_name, "rr", "Reply", rep_topic, sizeof(rep_topic))) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }

    auto* state = new (std::nothrow) ClientState();
    if (state == nullptr) return NROS_RMW_RET_BAD_ALLOC;
    state->req_desc = req_desc;
    state->rep_desc = rep_desc;
    state->next_seq.store(0, std::memory_order_relaxed);

    // Issue 0970 — OUR sertype, not Cyclone's generated one. Cyclone's deals in
    // typed C structs, which is what forced the CDR round trip in both
    // directions; `create_nros_sertype` makes the sample a span of CDR. On
    // failure the sertype is ours to free, on success the domain owns it.
    struct ddsi_sertype* req_st = create_nros_sertype(req_desc);
    struct ddsi_sertype* rep_st = create_nros_sertype(rep_desc);
    if (req_st == nullptr || rep_st == nullptr) {
        if (req_st != nullptr) ddsi_sertype_unref(req_st);
        if (rep_st != nullptr) ddsi_sertype_unref(rep_st);
        // A keyed descriptor is the only way this fails today, and a service
        // type has no keys — so this is a contract breach, not a runtime
        // condition a caller can recover from.
        return NROS_RMW_RET_ERROR;
    }
    state->request_topic =
        dds_create_topic_sertype(pp, req_topic, &req_st, nullptr, nullptr, nullptr);
    state->reply_topic =
        dds_create_topic_sertype(pp, rep_topic, &rep_st, nullptr, nullptr, nullptr);
    if (state->request_topic < 0 || state->reply_topic < 0) {
        if (state->request_topic > 0) (void)dds_delete(state->request_topic);
        if (state->reply_topic > 0) (void)dds_delete(state->reply_topic);
        delete state;
        return NROS_RMW_RET_ERROR;
    }

    // Phase 193.1b: honour the caller's profile, defaulting to
    // `rmw_qos_profile_services_default` (stock-RMW-interop default).
    rmw_qos_profile_t svc_qos = qos != nullptr ? *qos : NROS_RMW_QOS_PROFILE_SERVICES_DEFAULT;
    dds_qos_t* dq_writer = make_dds_qos(&svc_qos);
    dds_qos_t* dq_reader = make_dds_qos(&svc_qos);
    state->writer = dds_create_writer(pp, state->request_topic, dq_writer, nullptr);
    state->reader = dds_create_reader(pp, state->reply_topic, dq_reader, nullptr);
    if (dq_writer != nullptr) dds_delete_qos(dq_writer);
    if (dq_reader != nullptr) dds_delete_qos(dq_reader);
    if (state->writer < 0 || state->reader < 0) {
        if (state->writer > 0) (void)dds_delete(state->writer);
        if (state->reader > 0) (void)dds_delete(state->reader);
        (void)dds_delete(state->request_topic);
        (void)dds_delete(state->reply_topic);
        delete state;
        return NROS_RMW_RET_ERROR;
    }

    // Use the lower 8 bytes of the writer's RTPS GUID as the client
    // identity. Falls back to a random 64-bit value if dds_get_guid
    // fails or returns an all-zero prefix.
    state->my_guid = writer_guid_lo64(state->writer);
    if (state->my_guid == 0) {
        state->my_guid = random_guid64();
    }

    // Cyclone DDS 0.10.5 can miss local delivery when multiple service
    // clients are created back-to-back on one participant. Action clients
    // create send_goal/cancel/get_result clients in sequence, so leave a
    // small discovery window between creations.
    platform_sleep_ms(100);

    out->backend_data = state;
    // Phase 177.36 — register both endpoints with the node graph (server:
    // request reader + reply writer; client: request writer + reply reader).
    graph_track_reader(session_graph(session), state->reader);
    graph_track_writer(session_graph(session), state->writer);
    return NROS_RMW_RET_OK;
}

rmw_ret_t client_destroy(rmw_client_t* client) {
    if (client == nullptr || client->backend_data == nullptr) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    auto* state = static_cast<ClientState*>(client->backend_data);
    dds_return_t rc = DDS_RETCODE_OK;
    if (state->writer > 0 && dds_delete(state->writer) < 0) rc = DDS_RETCODE_ERROR;
    if (state->reader > 0 && dds_delete(state->reader) < 0) rc = DDS_RETCODE_ERROR;
    if (state->request_topic > 0 && dds_delete(state->request_topic) < 0) rc = DDS_RETCODE_ERROR;
    if (state->reply_topic > 0 && dds_delete(state->reply_topic) < 0) rc = DDS_RETCODE_ERROR;
    delete state;
    client->backend_data = nullptr;
    return rc < 0 ? NROS_RMW_RET_ERROR : NROS_RMW_RET_OK;
}

// Phase 130.8 — non-blocking send/recv split. Mirrors
// `xrce_service_send_request_raw` / `_take_response_raw` in the
// XRCE backend. Lets the executor's spin loop poll for a late-
// arriving reply without re-sending the request or blocking 5 s
// (Phase 127.C.4 root cause class). Phase-301: the deprecated
// blocking `call_raw` slot was deleted from the vtable; this pair
// is the one request/reply path.
rmw_ret_t service_send_request_raw(const rmw_client_t* client, rmw_byte_span_t request_span,
                                   int64_t* sequence_id) {
    const uint8_t* request = request_span.data;
    size_t req_len = request_span.len;
    if (client == nullptr || client->backend_data == nullptr || request == nullptr || req_len < 4) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    auto* state = static_cast<ClientState*>(client->backend_data);

    // Issue 0778 — a request already STAGED (written before the request writer
    // matched a server reader) is the one thing that still serialises sends.
    // There is one 64 KiB staging buffer and replicating it per outstanding
    // call is not affordable here, so a second send in that window is refused
    // rather than silently overwriting the first. Loud and retryable, instead
    // of a request that vanishes.
    //
    // Try to drain it first: the writer has usually matched by now, and then
    // this window does not exist at all.
    if (state->pending_request_len != 0) {
        rmw_ret_t drained = maybe_flush_request(state);
        if (drained != NROS_RMW_RET_OK && drained != NROS_RMW_RET_NO_DATA) {
            return drained;
        }
        if (state->pending_request_len != 0) {
            return NROS_RMW_RET_WOULD_BLOCK;
        }
    }

    RequestId my_id{};
    my_id.guid = state->my_guid;
    my_id.seq = state->next_seq.fetch_add(1, std::memory_order_relaxed);

    // Claim BEFORE writing: a reply can arrive as soon as the write lands, and
    // `take_response` only accepts ids in this set.
    if (!claim_outstanding(state, my_id.seq)) {
        // kMaxOutstandingRequests calls already in flight. The caller retries;
        // the alternative is dropping one of the requests it is waiting on.
        return NROS_RMW_RET_WOULD_BLOCK;
    }

    uint8_t wire_req[kWireScratch];
    int32_t wire_len = build_wire_with_header(request, req_len, my_id, wire_req, sizeof(wire_req));
    if (wire_is_status(wire_len)) {
        release_outstanding(state, my_id.seq);
        return wire_status_code(wire_len);
    }

    std::memcpy(state->pending_request, wire_req, static_cast<size_t>(wire_len));
    state->pending_request_len = static_cast<size_t>(wire_len);
    state->pending_request_seq = my_id.seq;
    rmw_ret_t pr = maybe_flush_request(state);
    if (pr != NROS_RMW_RET_OK && pr != NROS_RMW_RET_NO_DATA) {
        state->pending_request_len = 0;
        state->pending_request_seq = -1;
        release_outstanding(state, my_id.seq);
        return pr;
    }
    if (sequence_id != nullptr) *sequence_id = my_id.seq;
    return NROS_RMW_RET_OK;
}

static int32_t service_take_response_raw_len(const rmw_client_t* client, uint8_t* reply_buf,
                                             size_t reply_buf_len, int64_t* seq_out) {
    if (client == nullptr || client->backend_data == nullptr || reply_buf == nullptr) {
        return wire_status(NROS_RMW_RET_INVALID_ARGUMENT);
    }
    auto* state = static_cast<ClientState*>(client->backend_data);

    // Issue 0778 — nothing to wait for is not the same as nothing arrived, but
    // both are NO_DATA to the caller.
    if (!any_outstanding(state)) {
        return wire_status(NROS_RMW_RET_NO_DATA);
    }

    rmw_ret_t flush = maybe_flush_request(state);
    if (flush != NROS_RMW_RET_OK) {
        if (flush != NROS_RMW_RET_NO_DATA) {
            // The staged request will never go out; stop waiting for it. Any
            // OTHER outstanding call is untouched — that is the whole point of
            // the set.
            if (state->pending_request_seq >= 0) {
                release_outstanding(state, state->pending_request_seq);
            }
            state->pending_request_len = 0;
            state->pending_request_seq = -1;
        }
        return wire_status(flush);
    }

    // Issue 0778 — NO `DDS_DATA_AVAILABLE_STATUS` pre-filter here.
    //
    // Reading that status CLEARS it, and `subscriber.cpp` already documents
    // the consequence: "querying it as a pre-filter can clear/suppress the
    // subsequent take path while samples remain readable". The subscription
    // path removed its pre-filter for that reason; the reply path kept one,
    // and it did not matter while a client could only ever wait for a single
    // reply.
    //
    // With two replies outstanding it matters immediately: the first take
    // consumes the edge, the second reply is readable but the status no longer
    // says so, and the call reports NO_DATA forever. That is what
    // `service_two_outstanding` caught on its first run — one reply delivered,
    // one lost. `dds_take` below is the authoritative check, exactly as
    // `take_serialized` is on the subscription side.
    uint8_t wire_rep[kWireScratch];
    int32_t wlen = take_typed_wire(state->reader, wire_rep, sizeof(wire_rep));
    if (wire_is_status(wlen)) return wlen;

    RequestId got_id{};
    int32_t user_len = split_wire_header(wire_rep, static_cast<size_t>(wlen), state->rep_desc,
                                         &got_id, reply_buf, reply_buf_len);
    if (wire_is_status(user_len)) return user_len;

    // Issue 0778 — accept a reply for ANY request this client is waiting on,
    // and retire that one. It used to compare against a single `pending_seq`,
    // so a reply to anything but the newest send was dropped here as "for a
    // different in-flight call" — the other half of the abandon.
    if (got_id.guid == state->my_guid && release_outstanding(state, got_id.seq)) {
        if (seq_out != nullptr) *seq_out = got_id.seq;
        return user_len;
    }
    // Genuinely not ours: another client's guid, or a late reply to a call we
    // already retired. Drop it and surface NO_DATA so the executor retries on
    // the next spin tick.
    return wire_status(NROS_RMW_RET_NO_DATA);
}

/* Phase 376 W3.b/W3.d step A — upstream's shape over the unchanged body above.
 * A THIN adapter rather than a rewrite: the length-returning logic below has
 * error paths that are easy to get subtly wrong (WOULD_BLOCK is an error here,
 * not "nothing to take"), so it is preserved verbatim and only the reporting
 * convention is translated. NO_DATA is the one code that becomes
 * `taken = false` with OK. */
rmw_ret_t service_take_response(const rmw_client_t* client, rmw_mut_byte_span_t* reply,
                                int64_t* seq_out, bool* taken) {
    if (reply == nullptr) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    uint8_t* reply_buf = reply->data;
    size_t reply_buf_len = reply->capacity;
    size_t* out_len = &reply->len;
    if (out_len == nullptr || taken == nullptr) {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    int32_t n = service_take_response_raw_len(client, reply_buf, reply_buf_len, seq_out);
    if (wire_is_status(n)) {
        const rmw_ret_t st = wire_status_code(n);
        // An empty queue and a would-block are NOT failures: report
        // `taken = false` with OK, which is what the shim's contract says.
        if (st == NROS_RMW_RET_NO_DATA || st == NROS_RMW_RET_WOULD_BLOCK) {
            *taken = false;
            return NROS_RMW_RET_OK;
        }
        return st;
    }
    *out_len = (size_t)n;
    *taken = true;
    return NROS_RMW_RET_OK;
}

/* phase-393 W1 (issue 0823) — the four DDS entities behind a client or a
 * service, so their granted QoS can be read back like a publisher's.
 *
 * Upstream names them from the CALLER's point of view, which is worth spelling
 * out because the mapping inverts between the two sides: a client's REQUEST
 * publisher is its writer and its RESPONSE subscription is its reader, while a
 * service's REQUEST subscription is its reader and its RESPONSE publisher is
 * its writer. Getting that backwards reads a real QoS off the wrong entity —
 * an answer that looks right and is not, which is worse than no answer. */
dds_entity_t client_request_writer(const rmw_client_t* client) {
    if (client == nullptr || client->backend_data == nullptr) return 0;
    return static_cast<const ClientState*>(client->backend_data)->writer;
}

dds_entity_t client_response_reader(const rmw_client_t* client) {
    if (client == nullptr || client->backend_data == nullptr) return 0;
    return static_cast<const ClientState*>(client->backend_data)->reader;
}

dds_entity_t service_request_reader(const rmw_service_t* service) {
    if (service == nullptr || service->backend_data == nullptr) return 0;
    return static_cast<const ServerState*>(service->backend_data)->reader;
}

dds_entity_t service_response_writer(const rmw_service_t* service) {
    if (service == nullptr || service->backend_data == nullptr) return 0;
    return static_cast<const ServerState*>(service->backend_data)->writer;
}

} // namespace nros_rmw_cyclonedds
