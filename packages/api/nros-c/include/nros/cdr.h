/**
 * @file cdr.h
 * @ingroup grp_cdr
 * @brief CDR serialization helpers.
 *
 * Low-level read/write functions for CDR (Common Data Representation)
 * encoding.  These are used by generated message code and can also be
 * called directly for manual serialization.
 *
 * All functions advance the @c ptr cursor.  They return 0 on success
 * or a negative value if the buffer is too small.
 */

#ifndef NROS_CDR_H
#define NROS_CDR_H

#include "nros/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 * Write Functions
 * =================================================================== */

/**
 * @brief Write a boolean value to the buffer.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Boolean value to write.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
// phase-303 W4 (#0267) — XCDR2 DHEADER delimiters. Under XCDR1 (humble) these
// are no-ops returning -1 (mark) / 0. Under XCDR2 (iron/jazzy+) begin reserves a
// 4-byte DHEADER (returns its offset as the mark); end backpatches it with the
// member-block size. Generated `_serialize_inline` / `_deserialize_inline` wrap
// every struct so appendable types carry a DHEADER matching a modern ROS 2 peer.
// begin returns >=0 mark or -1 (XCDR1) on success, -2 on error.
// Writes the edition-appropriate 4-byte encapsulation header (XCDR1 or XCDR2).
int32_t nros_cdr_write_encaps_header(uint8_t** ptr, const uint8_t* end);

int64_t nros_cdr_begin_dheader(uint8_t** ptr, const uint8_t* end, const uint8_t* origin);
int32_t nros_cdr_end_dheader(int64_t mark, uint8_t** ptr, const uint8_t* end,
                             const uint8_t* origin);
int64_t nros_cdr_begin_dheader_read(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin);
int32_t nros_cdr_end_dheader_read(int64_t scope, const uint8_t** ptr, const uint8_t* end,
                                  const uint8_t* origin);

int32_t nros_cdr_write_bool(uint8_t** ptr, const uint8_t* end, const uint8_t* origin, bool value);

/**
 * @brief Write a u8 value to the buffer.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Value to write.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_write_u8(uint8_t** ptr, const uint8_t* end, const uint8_t* origin, uint8_t value);

/**
 * @brief Write an i8 value to the buffer.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Value to write.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_write_i8(uint8_t** ptr, const uint8_t* end, const uint8_t* origin, int8_t value);

/**
 * @brief Write a u16 value to the buffer (with alignment).
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Value to write.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_write_u16(uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                           uint16_t value);

/**
 * @brief Write an i16 value to the buffer.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Value to write.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_write_i16(uint8_t** ptr, const uint8_t* end, const uint8_t* origin, int16_t value);

/**
 * @brief Write a u32 value to the buffer (with alignment).
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Value to write.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_write_u32(uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                           uint32_t value);

/**
 * @brief Write an i32 value to the buffer.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Value to write.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_write_i32(uint8_t** ptr, const uint8_t* end, const uint8_t* origin, int32_t value);

/**
 * @brief Write a u64 value to the buffer (with alignment).
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Value to write.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_write_u64(uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                           uint64_t value);

/**
 * @brief Write an i64 value to the buffer.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Value to write.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_write_i64(uint8_t** ptr, const uint8_t* end, const uint8_t* origin, int64_t value);

/**
 * @brief Write a f32 value to the buffer (with alignment).
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Value to write.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_write_f32(uint8_t** ptr, const uint8_t* end, const uint8_t* origin, float value);

/**
 * @brief Write a f64 value to the buffer (with alignment).
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Value to write.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_write_f64(uint8_t** ptr, const uint8_t* end, const uint8_t* origin, double value);

/**
 * @brief Write a string to the buffer (length-prefixed).
 *
 * CDR strings are encoded as: u32 length (including null terminator)
 * + bytes + null terminator.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Null-terminated string to write.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_write_string(uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                              const char* value);

/**
 * @brief Write a string from a pointer+length pair (not null-terminated).
 *
 * Used by borrowed message types where the string field is a
 * `struct { const char* data; size_t size; }` rather than a `char[]`.
 *
 * CDR encoding: u32 length (size + 1 for null) + bytes + null terminator.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param data   String data (may not be null-terminated).
 * @param len    String length in bytes (excluding any null terminator).
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_write_string_n(uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                                const char* data, size_t len);

/* ===================================================================
 * Read Functions
 * =================================================================== */

/**
 * @brief Read a boolean value from the buffer.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Output: the value read.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_read_bool(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                           bool* value);

/**
 * @brief Read a u8 value from the buffer.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Output: the value read.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_read_u8(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                         uint8_t* value);

/**
 * @brief Read an i8 value from the buffer.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Output: the value read.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_read_i8(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                         int8_t* value);

/**
 * @brief Read a u16 value from the buffer (with alignment).
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Output: the value read.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_read_u16(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                          uint16_t* value);

/**
 * @brief Read an i16 value from the buffer.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Output: the value read.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_read_i16(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                          int16_t* value);

/**
 * @brief Read a u32 value from the buffer (with alignment).
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Output: the value read.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_read_u32(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                          uint32_t* value);

/**
 * @brief Read an i32 value from the buffer.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Output: the value read.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_read_i32(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                          int32_t* value);

/**
 * @brief Read a u64 value from the buffer (with alignment).
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Output: the value read.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_read_u64(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                          uint64_t* value);

/**
 * @brief Read an i64 value from the buffer.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Output: the value read.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_read_i64(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                          int64_t* value);

/**
 * @brief Read a f32 value from the buffer.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Output: the value read.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_read_f32(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                          float* value);

/**
 * @brief Read a f64 value from the buffer.
 *
 * @param ptr    Pointer to the cursor (advanced on success).
 * @param end    Pointer past the end of the buffer.
 * @param origin Buffer origin (for alignment calculations).
 * @param value  Output: the value read.
 * @return 0 on success, negative on overflow.
 */
NROS_PUBLIC
int32_t nros_cdr_read_f64(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                          double* value);

/**
 * @brief Read a string from the buffer into a fixed-size buffer.
 *
 * CDR strings are encoded as: u32 length (including null terminator)
 * + bytes + null terminator.
 *
 * @param ptr     Pointer to the cursor (advanced on success).
 * @param end     Pointer past the end of the buffer.
 * @param origin  Buffer origin (for alignment calculations).
 * @param value   Output buffer for the string.
 * @param max_len Maximum length of the output buffer.
 * @return 0 on success, negative on overflow or truncation.
 */
NROS_PUBLIC
int32_t nros_cdr_read_string(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                             char* value, size_t max_len);

/**
 * @brief Advance the cursor to the next @p alignment boundary of the CDR
 * stream, consuming the writer's padding.
 *
 * Alignment is measured from @p origin (the stream origin after the
 * encapsulation header), never from the buffer pointer: a `float64[]` whose
 * 4-byte count lands at stream offset 4 has its first element at offset 8,
 * whatever address the buffer happens to sit at. This is the Rust reader's
 * `CdrReader::align` behind the C ABI, so it carries the same edition rule
 * (XCDR2 caps 8-byte primitives at 4) and the header-only borrow readers in
 * `<nros/view.h>` cannot drift from it (issue 1148).
 *
 * @param ptr       Pointer to the cursor (advanced on success).
 * @param end       Pointer past the end of the buffer.
 * @param origin    Buffer origin (for alignment calculations).
 * @param alignment Boundary in bytes (1, 2, 4 or 8).
 * @return 0 on success, negative if @p alignment is 0 or the padding would run
 *         past @p end.
 */
NROS_PUBLIC
int32_t nros_cdr_align(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin,
                       size_t alignment);

/**
 * @brief The byte length of @p count wire elements of @p elem_size bytes, or
 * failure if the product does not fit below @p limit.
 *
 * @p count is untrusted — it came off the wire — and on every 32-bit target
 * `(size_t)count * elem_size` wraps for a count as small as `0x40000000`
 * (issue 1149): the wrapped length passes any bounds check, and the view then
 * reads far past the buffer while reporting success. Call sites pass
 * `SIZE_MAX` as @p limit; a test may pass a 32-bit limit to exercise the wrap
 * on a 64-bit host, which is the only way the native lane can see it.
 *
 * @return 0 and @p *out on success, -1 when `count * elem_size > limit`.
 */
static inline int32_t nros_cdr_seq_byte_len(uint32_t count, size_t elem_size, size_t limit,
                                            size_t* out) {
    if (elem_size == 0 || (size_t)count > limit / elem_size) return -1;
    *out = (size_t)count * elem_size;
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* NROS_CDR_H */
