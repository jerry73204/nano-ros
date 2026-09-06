/**
 * @file borrowed.h
 * @ingroup grp_cdr
 * @brief Borrowed (zero-copy) message-field views — RFC-0033 `borrowed` mode.
 *
 * A `mode = "borrowed"` field (Phase 235, issue 0021) is a read-only,
 * callback-scoped view that points directly into the live CDR receive buffer
 * instead of copying into a fixed array (`owned`) or malloc'd container
 * (`heap`). The generated `{Msg}_deserialize_view` walks the CDR stream and
 * sets these views' pointers into the buffer — no allocation, no `_fini`.
 *
 * Hard constraints (RFC-0033): the views are valid **only** for the duration of
 * the subscription callback; the buffer is reused immediately after. Copy out
 * anything that must outlive the callback.
 *
 * - byte sequences (`uint8[]`/`int8[]`/`bool[]`) → ::nros_view_bytes_t
 * - strings → ::nros_view_str_t
 * - multi-byte numeric sequences (`float32[]`, `uint16[]`, …) → an
 *   alignment-agnostic `nros_le_slice_view_<t>_t`: the raw little-endian bytes
 *   are borrowed and decoded per element on access, so the buffer need not be
 *   `T`-aligned (mirrors Rust's `nros_core::LeSliceView`).
 */

#ifndef NROS_VIEW_H
#define NROS_VIEW_H

#include <stdint.h>
#include <string.h>

#include "nros/cdr.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Borrowed UTF-8 string view (excludes the CDR trailing NUL from @c size). */
typedef struct {
    const char* data;
    size_t size;
} nros_view_str_t;

/** Borrowed byte-sequence view (`uint8[]`/`int8[]`/`bool[]`). */
typedef struct {
    const uint8_t* data;
    size_t size;
} nros_view_bytes_t;

/**
 * @brief Borrow a CDR string: read the 4-byte length, point @c out into the
 * buffer (excluding the trailing NUL), and advance the cursor. No copy.
 * @return 0 on success, negative if the buffer is too small.
 */
static inline int32_t nros_cdr_borrow_string(const uint8_t** ptr, const uint8_t* end,
                                             const uint8_t* origin, nros_view_str_t* out) {
    uint32_t slen;
    if (nros_cdr_read_u32(ptr, end, origin, &slen) < 0) return -1;
    if ((size_t)(end - *ptr) < slen) return -1;
    out->data = (const char*)*ptr;
    out->size = slen > 0 ? (size_t)(slen - 1) : 0; /* CDR string length includes NUL */
    *ptr += slen;
    return 0;
}

/**
 * @brief Borrow a byte sequence: read the 4-byte element count, point @c out
 * into the buffer, and advance the cursor. No copy.
 * @return 0 on success, negative if the buffer is too small.
 */
static inline int32_t nros_cdr_borrow_bytes(const uint8_t** ptr, const uint8_t* end,
                                            const uint8_t* origin, nros_view_bytes_t* out) {
    uint32_t len;
    if (nros_cdr_read_u32(ptr, end, origin, &len) < 0) return -1;
    if ((size_t)(end - *ptr) < len) return -1;
    out->data = *ptr;
    out->size = len;
    *ptr += len;
    return 0;
}

/*
 * Alignment-agnostic numeric views. For each element type T:
 *   nros_le_slice_view_<t>_t        — { const uint8_t* bytes; size_t count; }
 *   nros_cdr_borrow_le_slice_<t>()  — borrow `count` LE elements, advance cursor
 *   nros_le_slice_view_<t>_get()    — decode element `i` by value (no alignment)
 * Integer elements are assembled byte-by-byte as little-endian (portable across
 * host endianness); float/double bit-cast from the assembled unsigned bits.
 *
 * "Alignment-agnostic" is a statement about the HOST, not the wire: no `T*` is
 * ever formed into the buffer, so the buffer base may sit at any address. The
 * WIRE still places the first element on a sizeof(T) boundary of the stream
 * (measured from `origin`), and the borrow consumes that padding through
 * `nros_cdr_align` exactly as the owned readers and the Rust `CdrReader` do —
 * taking `*ptr` straight after the count read the padding as data for every
 * 8-byte element type and left the cursor short for every field after (issue
 * 1148). The count is wire data, so its byte length goes through
 * `nros_cdr_seq_byte_len`, which refuses a product that would wrap `size_t`;
 * on every 32-bit target a bare `(size_t)cnt * sizeof(T)` wraps to a length
 * that passes the bounds check (issue 1149). `_get` returns the zero value for
 * an index at or past `count` rather than reading outside the view.
 */

#define NROS__LE_VIEW_COMMON(SUFFIX, CT)                                                           \
    typedef struct {                                                                               \
        const uint8_t* bytes;                                                                      \
        size_t count;                                                                              \
    } nros_le_slice_view_##SUFFIX##_t;                                                             \
    static inline int32_t nros_cdr_borrow_le_slice_##SUFFIX(                                       \
        const uint8_t** ptr, const uint8_t* end, const uint8_t* origin,                            \
        nros_le_slice_view_##SUFFIX##_t* out) {                                                    \
        uint32_t cnt;                                                                              \
        size_t bytelen;                                                                            \
        if (nros_cdr_read_u32(ptr, end, origin, &cnt) < 0) return -1;                              \
        /* Element 0 starts on a sizeof(CT) boundary of the STREAM (issue 1148); the   */          \
        /* writer pads before an element, so an empty sequence carries no padding.    */           \
        if (cnt > 0 && nros_cdr_align(ptr, end, origin, sizeof(CT)) < 0) return -1;                \
        /* cnt is wire data: the product must be checked, not trusted (issue 1149). */             \
        if (nros_cdr_seq_byte_len(cnt, sizeof(CT), SIZE_MAX, &bytelen) < 0) return -1;             \
        if ((size_t)(end - *ptr) < bytelen) return -1;                                             \
        out->bytes = *ptr;                                                                         \
        out->count = (size_t)cnt;                                                                  \
        *ptr += bytelen;                                                                           \
        return 0;                                                                                  \
    }

/* Integer element: little-endian assembly into the host-native value. */
#define NROS__LE_VIEW_INT(SUFFIX, CT, UT)                                                          \
    NROS__LE_VIEW_COMMON(SUFFIX, CT)                                                               \
    static inline CT nros_le_slice_view_##SUFFIX##_get(nros_le_slice_view_##SUFFIX##_t v,          \
                                                       size_t i) {                                 \
        UT u = 0;                                                                                  \
        if (i >= v.count) {                                                                        \
            CT zero;                                                                               \
            memcpy(&zero, &u, sizeof(CT));                                                         \
            return zero;                                                                           \
        }                                                                                          \
        const uint8_t* p = v.bytes + i * sizeof(CT);                                               \
        for (size_t b = 0; b < sizeof(CT); ++b)                                                    \
            u |= (UT)p[b] << (8u * b);                                                             \
        CT out;                                                                                    \
        memcpy(&out, &u, sizeof(CT));                                                              \
        return out;                                                                                \
    }

/* Float element: assemble the LE bit pattern as an unsigned, then bit-cast. */
#define NROS__LE_VIEW_FLT(SUFFIX, CT, UT)                                                          \
    NROS__LE_VIEW_COMMON(SUFFIX, CT)                                                               \
    static inline CT nros_le_slice_view_##SUFFIX##_get(nros_le_slice_view_##SUFFIX##_t v,          \
                                                       size_t i) {                                 \
        UT u = 0;                                                                                  \
        if (i >= v.count) {                                                                        \
            CT zero;                                                                               \
            memcpy(&zero, &u, sizeof(CT));                                                         \
            return zero;                                                                           \
        }                                                                                          \
        const uint8_t* p = v.bytes + i * sizeof(CT);                                               \
        for (size_t b = 0; b < sizeof(CT); ++b)                                                    \
            u |= (UT)p[b] << (8u * b);                                                             \
        CT out;                                                                                    \
        memcpy(&out, &u, sizeof(CT));                                                              \
        return out;                                                                                \
    }

NROS__LE_VIEW_INT(u16, uint16_t, uint16_t)
NROS__LE_VIEW_INT(i16, int16_t, uint16_t)
NROS__LE_VIEW_INT(u32, uint32_t, uint32_t)
NROS__LE_VIEW_INT(i32, int32_t, uint32_t)
NROS__LE_VIEW_INT(u64, uint64_t, uint64_t)
NROS__LE_VIEW_INT(i64, int64_t, uint64_t)
NROS__LE_VIEW_FLT(f32, float, uint32_t)
NROS__LE_VIEW_FLT(f64, double, uint64_t)

#ifdef __cplusplus
}
#endif

#endif /* NROS_VIEW_H */
