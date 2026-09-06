/* Issues 1148 / 1149 — the RFC-0033 zero-copy numeric views against the CDR
 * stream the C writer itself produces.
 *
 * Compiled, LINKED against libnros_c.a and RUN by `just check c`; the other
 * TUs beside this one under `tests/compile/` are syntax-only. Two properties,
 * each with the layout that made it visible:
 *
 *   1148  the view starts on the element's boundary of the STREAM (measured
 *         from `origin`, not from the buffer pointer), and the cursor lands
 *         where the next field is. `uint32 a; float64[] xs` puts the count at
 *         stream offset 4 and the elements at 8 — already aligned, which is
 *         why the shipped tests (all 2- and 4-byte widths) never saw it. One
 *         more `uint32` in front moves the count to 8 and the elements to 16
 *         behind four bytes of padding; the pre-fix view returned those four
 *         padding bytes as the low half of element 0 and left every field
 *         after the sequence four bytes short.
 *
 *   1149  the count is wire data. `nros_cdr_seq_byte_len` is the ONE place a
 *         count meets `sizeof`, and it takes its limit as a parameter so this
 *         64-bit host can exercise the 32-bit wrap: `0x40000000 * 4` and
 *         `0xFFFFFFFF * 8` both wrap below a 32-bit SIZE_MAX to a length the
 *         bounds check would accept. The macro path is also fed a count of
 *         0xFFFFFFFF and must return -1 without reading anything — on a
 *         32-bit `size_t` that is the overflow check firing, on 64-bit the
 *         bounds check; either way an error, never a view.
 *
 * Every check prints and exits non-zero rather than `assert()`, so an NDEBUG
 * compile cannot turn this into a binary that passes by doing nothing.
 *
 * Copyright 2026 nros contributors
 * Licensed under Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nros/cdr.h>
#include <nros/view.h>

static int failures = 0;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                        \
            failures++;                                                                            \
        }                                                                                          \
    } while (0)

/* The layout under test, written by the C writers so the expectation is the
 * writer's own padding rule, not a hand-typed guess. `lead` extra u32 fields
 * move the count: 0 → count at 4, elements at 8 (aligned by luck); 1 → count
 * at 8, elements at 16 (four bytes of padding under XCDR1). */
typedef struct {
    uint8_t buf[128];
    const uint8_t* origin;
    const uint8_t* end;
    size_t elems_at; /* stream offset of element 0, per the writer */
    size_t tail_at;  /* stream offset of the trailing u32 */
} stream_t;

static void write_u32_then_f64_seq(stream_t* s, int lead, const double* xs, uint32_t n) {
    uint8_t* p = s->buf;
    const uint8_t* end = s->buf + sizeof s->buf;
    CHECK(nros_cdr_write_encaps_header(&p, end) == 0);
    uint8_t* origin = p;
    CHECK(nros_cdr_write_u32(&p, end, origin, 0xA5A5A5A5u) == 0);
    for (int i = 0; i < lead; ++i)
        CHECK(nros_cdr_write_u32(&p, end, origin, 0x5A5A5A5Au) == 0);
    CHECK(nros_cdr_write_u32(&p, end, origin, n) == 0);
    for (uint32_t i = 0; i < n; ++i) {
        CHECK(nros_cdr_write_f64(&p, end, origin, xs[i]) == 0);
        if (i == 0) s->elems_at = (size_t)(p - origin) - sizeof(double);
    }
    if (n == 0) s->elems_at = (size_t)(p - origin);
    s->tail_at = (size_t)(p - origin);
    CHECK(nros_cdr_write_u32(&p, end, origin, 0xCAFEF00Du) == 0);
    s->origin = origin;
    s->end = p;
}

static int xcdr1(const stream_t* s) {
    /* `00 01 00 00` is XCDR1 (PL_CDR_LE); XCDR2 caps 8-byte alignment at 4 and
     * the absolute-offset expectations below would not hold. */
    return s->buf[0] == 0x00 && s->buf[1] == 0x01;
}

static void test_alignment(int lead) {
    const double xs[2] = {1111.0, 2222.0};
    stream_t s;
    memset(&s, 0, sizeof s);
    write_u32_then_f64_seq(&s, lead, xs, 2);

    const uint8_t* ptr = s.origin;
    uint32_t a = 0;
    CHECK(nros_cdr_read_u32(&ptr, s.end, s.origin, &a) == 0);
    CHECK(a == 0xA5A5A5A5u);
    for (int i = 0; i < lead; ++i)
        CHECK(nros_cdr_read_u32(&ptr, s.end, s.origin, &a) == 0);

    nros_le_slice_view_f64_t view;
    CHECK(nros_cdr_borrow_le_slice_f64(&ptr, s.end, s.origin, &view) == 0);
    CHECK(view.count == 2);
    /* Byte-for-byte with the writer: the view begins where the writer put
     * element 0, and the cursor where it put the tail. */
    CHECK((size_t)(view.bytes - s.origin) == s.elems_at);
    CHECK((size_t)(ptr - s.origin) == s.tail_at);
    if (xcdr1(&s)) {
        /* The absolute rule, so a writer regression cannot hide behind a
         * matching reader: count at 4 + 4*lead, elements 8-aligned after it. */
        size_t count_at = 4 + 4 * (size_t)lead;
        size_t expect = (count_at + 4 + 7) & ~(size_t)7;
        CHECK(s.elems_at == expect);
        CHECK(lead == 0 ? expect == 8 : expect == 16);
    }
    CHECK(nros_le_slice_view_f64_get(view, 0) == 1111.0);
    CHECK(nros_le_slice_view_f64_get(view, 1) == 2222.0);

    uint32_t tail = 0;
    CHECK(nros_cdr_read_u32(&ptr, s.end, s.origin, &tail) == 0);
    CHECK(tail == 0xCAFEF00Du);
    CHECK(ptr == s.end);

    /* Same stream, the integer family of the macro. */
    ptr = s.origin;
    CHECK(nros_cdr_read_u32(&ptr, s.end, s.origin, &a) == 0);
    for (int i = 0; i < lead; ++i)
        CHECK(nros_cdr_read_u32(&ptr, s.end, s.origin, &a) == 0);
    nros_le_slice_view_u64_t uview;
    CHECK(nros_cdr_borrow_le_slice_u64(&ptr, s.end, s.origin, &uview) == 0);
    CHECK(uview.count == 2);
    CHECK((size_t)(uview.bytes - s.origin) == s.elems_at);
    uint64_t bits;
    memcpy(&bits, &xs[0], sizeof bits);
    CHECK(nros_le_slice_view_u64_get(uview, 0) == bits);
    CHECK((size_t)(ptr - s.origin) == s.tail_at);
}

static void test_empty_sequence_takes_no_padding(void) {
    /* `uint32 a; uint32 b; float64[] xs (empty); uint8 flag` — the writer pads
     * before an ELEMENT, so an empty sequence gets none, and the flag sits at
     * offset 12. A reader that aligns unconditionally reads it at 16. */
    uint8_t buf[64];
    uint8_t* p = buf;
    const uint8_t* end = buf + sizeof buf;
    CHECK(nros_cdr_write_encaps_header(&p, end) == 0);
    uint8_t* origin = p;
    CHECK(nros_cdr_write_u32(&p, end, origin, 1) == 0);
    CHECK(nros_cdr_write_u32(&p, end, origin, 2) == 0);
    CHECK(nros_cdr_write_u32(&p, end, origin, 0) == 0);
    CHECK(nros_cdr_write_u8(&p, end, origin, 0xAB) == 0);
    const uint8_t* wend = p;

    const uint8_t* ptr = origin;
    uint32_t v;
    CHECK(nros_cdr_read_u32(&ptr, wend, origin, &v) == 0);
    CHECK(nros_cdr_read_u32(&ptr, wend, origin, &v) == 0);
    nros_le_slice_view_f64_t view;
    CHECK(nros_cdr_borrow_le_slice_f64(&ptr, wend, origin, &view) == 0);
    CHECK(view.count == 0);
    uint8_t flag = 0;
    CHECK(nros_cdr_read_u8(&ptr, wend, origin, &flag) == 0);
    CHECK(flag == 0xAB);
    CHECK(ptr == wend);
}

static void test_count_overflow(void) {
    size_t n = 0;
    const size_t limit32 = 0xFFFFFFFFu;
    /* The two wire counts from issue 1149, against a 32-bit SIZE_MAX. */
    CHECK(nros_cdr_seq_byte_len(0x40000000u, sizeof(float), limit32, &n) == -1);
    CHECK(nros_cdr_seq_byte_len(0xFFFFFFFFu, sizeof(double), limit32, &n) == -1);
    CHECK(nros_cdr_seq_byte_len(0x20000000u, sizeof(double), limit32, &n) == -1);
    /* The largest count that fits, and an ordinary one. */
    CHECK(nros_cdr_seq_byte_len(0x1FFFFFFFu, sizeof(double), limit32, &n) == 0);
    CHECK(n == 0xFFFFFFF8u);
    CHECK(nros_cdr_seq_byte_len(3, sizeof(double), limit32, &n) == 0);
    CHECK(n == 24);
    CHECK(nros_cdr_seq_byte_len(0, sizeof(double), limit32, &n) == 0);
    CHECK(n == 0);
    /* The call sites' own limit, whatever this host's size_t is. */
    CHECK(nros_cdr_seq_byte_len(0xFFFFFFFFu, sizeof(double), SIZE_MAX, &n) ==
          (SIZE_MAX == limit32 ? -1 : 0));

    /* Through the macro: a 4-byte header, then a count of 0xFFFFFFFF and
     * nothing else. Must be an error, never a view — and the cursor must not
     * have moved past the count. */
    uint8_t buf[16] = {0x00, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
    const uint8_t* origin = buf + 4;
    const uint8_t* end = buf + sizeof buf;
    const uint8_t* ptr = origin;
    nros_le_slice_view_f64_t v64 = {NULL, 0};
    CHECK(nros_cdr_borrow_le_slice_f64(&ptr, end, origin, &v64) == -1);
    CHECK(v64.bytes == NULL && v64.count == 0);
    nros_le_slice_view_f32_t v32 = {NULL, 0};
    buf[4] = 0x00, buf[5] = 0x00, buf[6] = 0x00, buf[7] = 0x40; /* 0x40000000 */
    ptr = origin;
    CHECK(nros_cdr_borrow_le_slice_f32(&ptr, end, origin, &v32) == -1);
    CHECK(v32.bytes == NULL && v32.count == 0);
#if SIZE_MAX == 0xFFFFFFFFu
    puts("cdr_borrow_le_slice: 32-bit size_t — the macro path exercised the overflow check");
#else
    puts("cdr_borrow_le_slice: 64-bit size_t — the macro path fails on bounds; the overflow "
         "check was exercised through nros_cdr_seq_byte_len with a 32-bit limit");
#endif
}

static void test_get_out_of_range_reads_nothing(void) {
    const uint8_t bytes[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    nros_le_slice_view_u32_t v = {bytes, 2};
    CHECK(nros_le_slice_view_u32_get(v, 0) == 1);
    CHECK(nros_le_slice_view_u32_get(v, 1) == 0);
    CHECK(nros_le_slice_view_u32_get(v, 2) == 0);
    CHECK(nros_le_slice_view_u32_get(v, SIZE_MAX) == 0);
    nros_le_slice_view_f64_t e = {bytes, 0};
    CHECK(nros_le_slice_view_f64_get(e, 0) == 0.0);
}

int main(void) {
    test_alignment(0); /* the task's literal layout: u32 then float64[] */
    test_alignment(1); /* the padded layout issue 1148 measured */
    test_empty_sequence_takes_no_padding();
    test_count_overflow();
    test_get_out_of_range_reads_nothing();
    if (failures) {
        fprintf(stderr, "cdr_borrow_le_slice: %d check(s) FAILED\n", failures);
        return 1;
    }
    puts("cdr_borrow_le_slice: OK");
    return 0;
}
