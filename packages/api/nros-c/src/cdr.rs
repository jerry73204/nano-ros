//! CDR serialization helpers for generated C message types.
//!
//! Thin FFI wrappers over [`nros_node::CdrWriter`] / [`nros_node::CdrReader`].
//! The public `extern "C"` surface is preserved for the generated C message
//! code: the write/read helpers take a `(cursor, end, origin)` triple, and
//! advance `cursor` on success.

use core::ffi::c_char;
use nros_node::{CdrReader, CdrWriter, DHeaderMark, DHeaderScope, DeserError, SerError};

/// The wire encoding for the C tx/rx path. `false` = XCDR1, `true` = XCDR2
/// DELIMITED_CDR2 (the C codegen wraps each struct with `nros_cdr_begin/end_dheader`,
/// per-primitive align caps at 4).
///
/// **DEFAULT `false` (XCDR1) — corrected 2026-07-26 after live verification.** An
/// earlier version keyed this on `ros-<edition>` (XCDR2 on iron/jazzy+). That is
/// WRONG: default Jazzy is FINAL/XCDR1 on the wire and rejects an APPENDABLE/XCDR2
/// peer. XCDR2 is a PER-TYPE `@appendable` property, not an edition blanket; the
/// path stays built for that future opt-in. See #0267.
const XCDR2: bool = false;

#[inline]
fn writer_at(buf: &mut [u8], pos: usize) -> Result<CdrWriter<'_>, SerError> {
    if XCDR2 {
        CdrWriter::new_at_xcdr2(buf, pos)
    } else {
        CdrWriter::new_at(buf, pos)
    }
}

#[inline]
fn reader_at(buf: &[u8], pos: usize) -> Result<CdrReader<'_>, DeserError> {
    if XCDR2 {
        CdrReader::new_at_xcdr2(buf, pos)
    } else {
        CdrReader::new_at(buf, pos)
    }
}

/// Write the 4-byte CDR encapsulation header for the active ROS edition and
/// advance the cursor: XCDR1 `00 01 00 00` (humble) or XCDR2 DELIMITED_CDR2
/// `00 09 00 00` (iron/jazzy+). Replaces the hardcoded header the generated C
/// `_serialize` used to emit. Returns 0 on success, -1 on overrun/null.
///
/// # Safety
/// `ptr` is a `*mut *mut u8` cursor; `end` bounds the buffer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_write_encaps_header(ptr: *mut *mut u8, end: *const u8) -> i32 {
    if ptr.is_null() || end.is_null() {
        return -1;
    }
    let cur = *ptr;
    if cur.is_null() || (cur as *const u8).add(4) > end {
        return -1;
    }
    let hdr: [u8; 4] = if XCDR2 {
        [0x00, 0x09, 0x00, 0x00]
    } else {
        [0x00, 0x01, 0x00, 0x00]
    };
    core::ptr::copy_nonoverlapping(hdr.as_ptr(), cur, 4);
    *ptr = cur.add(4);
    0
}

// ===========================================================================
// DHEADER (XCDR2 appendable delimiter) — phase-303 W4 / #0267
// ===========================================================================

/// Begin a DHEADER-delimited struct. Under XCDR2 (iron/jazzy+) reserves a 4-byte
/// DHEADER at the cursor (aligned) and advances it, returning the mark (its byte
/// offset). Under XCDR1 (humble) this is a NO-OP returning `-1`. The generated C
/// `_serialize_inline` calls this at the top of every struct and passes the mark
/// to [`nros_cdr_end_dheader`]. Returns `-2` on any bounds/pointer failure.
///
/// # Safety
/// `ptr`/`end`/`origin` follow the same `(cursor, end, origin)` contract as the
/// write helpers.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_begin_dheader(
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
) -> i64 {
    if !XCDR2 {
        return -1;
    }
    if ptr.is_null() || end.is_null() || origin.is_null() {
        return -2;
    }
    let cur = *ptr;
    if cur.is_null() || (cur as *const u8) < origin || (cur as *const u8) > end {
        return -2;
    }
    let buf_len = (end as usize).wrapping_sub(origin as usize);
    let pos = (cur as usize).wrapping_sub(origin as usize);
    let slice = core::slice::from_raw_parts_mut(origin as *mut u8, buf_len);
    let mut w = match CdrWriter::new_at_xcdr2(slice, pos) {
        Ok(w) => w,
        Err(_) => return -2,
    };
    let mark = match w.begin_dheader() {
        Ok(m) => m,
        Err(_) => return -2,
    };
    *ptr = (origin as *mut u8).add(w.position());
    mark.raw().map(|m| m as i64).unwrap_or(-1)
}

/// Finish a DHEADER-delimited struct: backpatch the reserved slot (from
/// [`nros_cdr_begin_dheader`]) with the bytes written since. `mark < 0` (XCDR1)
/// is a no-op. Returns 0 on success, -1 on failure.
///
/// # Safety
/// Same buffer contract as the write helpers.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_end_dheader(
    mark: i64,
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
) -> i32 {
    if mark < 0 {
        return 0; // XCDR1 no-op
    }
    if ptr.is_null() || end.is_null() || origin.is_null() {
        return -1;
    }
    let cur = *ptr;
    if cur.is_null() || (cur as *const u8) < origin || (cur as *const u8) > end {
        return -1;
    }
    let buf_len = (end as usize).wrapping_sub(origin as usize);
    let pos = (cur as usize).wrapping_sub(origin as usize);
    let slice = core::slice::from_raw_parts_mut(origin as *mut u8, buf_len);
    let mut w = match CdrWriter::new_at_xcdr2(slice, pos) {
        Ok(w) => w,
        Err(_) => return -1,
    };
    if w.end_dheader(DHeaderMark::from_raw(Some(mark as usize)))
        .is_err()
    {
        return -1;
    }
    0
}

/// Begin reading a DHEADER-delimited struct. Under XCDR2 reads the 4-byte
/// DHEADER at the cursor (advancing it) and returns the struct's END offset;
/// under XCDR1 returns `-1` (no-op). Returns `-2` on failure. The generated C
/// `_deserialize` passes the result to [`nros_cdr_end_dheader_read`].
///
/// # Safety
/// Same `(cursor, end, origin)` contract as the read helpers.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_begin_dheader_read(
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
) -> i64 {
    if !XCDR2 {
        return -1;
    }
    if ptr.is_null() || end.is_null() || origin.is_null() {
        return -2;
    }
    let cur = *ptr;
    if cur.is_null() || cur < origin || cur > end {
        return -2;
    }
    let buf_len = (end as usize).wrapping_sub(origin as usize);
    let pos = (cur as usize).wrapping_sub(origin as usize);
    let slice = core::slice::from_raw_parts(origin, buf_len);
    let mut r = match CdrReader::new_at_xcdr2(slice, pos) {
        Ok(r) => r,
        Err(_) => return -2,
    };
    let scope = match r.begin_dheader() {
        Ok(s) => s,
        Err(_) => return -2,
    };
    *ptr = origin.add(r.position());
    scope.raw().map(|e| e as i64).unwrap_or(-1)
}

/// Finish reading a DHEADER-delimited struct: skip any unknown trailing members
/// by advancing the cursor to the struct END (`scope` from
/// [`nros_cdr_begin_dheader_read`]). `scope < 0` (XCDR1) is a no-op. Returns 0
/// on success, -1 if the reader over-read past the declared end.
///
/// # Safety
/// Same buffer contract as the read helpers.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_end_dheader_read(
    scope: i64,
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
) -> i32 {
    if scope < 0 {
        return 0; // XCDR1 no-op
    }
    if ptr.is_null() || end.is_null() || origin.is_null() {
        return -1;
    }
    let cur = *ptr;
    if cur.is_null() || cur < origin || cur > end {
        return -1;
    }
    let buf_len = (end as usize).wrapping_sub(origin as usize);
    let pos = (cur as usize).wrapping_sub(origin as usize);
    let slice = core::slice::from_raw_parts(origin, buf_len);
    let mut r = match CdrReader::new_at_xcdr2(slice, pos) {
        Ok(r) => r,
        Err(_) => return -1,
    };
    if r.end_dheader(DHeaderScope::from_raw(Some(scope as usize)))
        .is_err()
    {
        return -1;
    }
    // Advance the caller's cursor to the struct end (skip trailing members).
    *ptr = origin.add(r.position());
    0
}

// ===========================================================================
// Bridge helpers
// ===========================================================================

/// Run `f` against a positioned [`CdrWriter`] spanning `origin..end`.
///
/// Returns 0 on success (advancing `*ptr`), -1 on any bounds / alignment /
/// serializer failure.
#[inline]
unsafe fn with_writer<F>(ptr: *mut *mut u8, end: *const u8, origin: *const u8, f: F) -> i32
where
    F: FnOnce(&mut CdrWriter<'_>) -> Result<(), SerError>,
{
    if ptr.is_null() || end.is_null() || origin.is_null() {
        return -1;
    }
    let cur = *ptr;
    if cur.is_null() {
        return -1;
    }
    if (cur as *const u8) < origin || (cur as *const u8) > end {
        return -1;
    }
    let buf_len = (end as usize).wrapping_sub(origin as usize);
    let pos = (cur as usize).wrapping_sub(origin as usize);
    let slice = core::slice::from_raw_parts_mut(origin as *mut u8, buf_len);
    let mut w = match writer_at(slice, pos) {
        Ok(w) => w,
        Err(_) => return -1,
    };
    if f(&mut w).is_err() {
        return -1;
    }
    *ptr = (origin as *mut u8).add(w.position());
    0
}

/// Run `f` against a positioned [`CdrReader`] spanning `origin..end`.
#[inline]
unsafe fn with_reader<F>(ptr: *mut *const u8, end: *const u8, origin: *const u8, f: F) -> i32
where
    F: FnOnce(&mut CdrReader<'_>) -> Result<(), DeserError>,
{
    if ptr.is_null() || end.is_null() || origin.is_null() {
        return -1;
    }
    let cur = *ptr;
    if cur.is_null() {
        return -1;
    }
    if cur < origin || cur > end {
        return -1;
    }
    let buf_len = (end as usize).wrapping_sub(origin as usize);
    let pos = (cur as usize).wrapping_sub(origin as usize);
    let slice = core::slice::from_raw_parts(origin, buf_len);
    let mut r = match reader_at(slice, pos) {
        Ok(r) => r,
        Err(_) => return -1,
    };
    if f(&mut r).is_err() {
        return -1;
    }
    *ptr = origin.add(r.position());
    0
}

// ===========================================================================
// Write functions
// ===========================================================================

/// Write a CDR-encoded `bool` (1 byte) and advance the cursor.
///
/// Returns 0 on success, -1 if the cursor would overrun `end` or any
/// pointer is null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_write_bool(
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
    value: bool,
) -> i32 {
    with_writer(ptr, end, origin, |w| w.write_bool(value))
}

/// Write a CDR-encoded `uint8_t` and advance the cursor. See
/// [`nros_cdr_write_bool`] for the buffer contract.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_write_u8(
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
    value: u8,
) -> i32 {
    with_writer(ptr, end, origin, |w| w.write_u8(value))
}

/// Write a CDR-encoded `int8_t` and advance the cursor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_write_i8(
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
    value: i8,
) -> i32 {
    with_writer(ptr, end, origin, |w| w.write_i8(value))
}

/// Write a CDR-encoded `uint16_t` (2 bytes, little-endian, 2-byte
/// aligned) and advance the cursor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_write_u16(
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
    value: u16,
) -> i32 {
    with_writer(ptr, end, origin, |w| w.write_u16(value))
}

/// Write a CDR-encoded `int16_t` and advance the cursor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_write_i16(
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
    value: i16,
) -> i32 {
    with_writer(ptr, end, origin, |w| w.write_i16(value))
}

/// Write a CDR-encoded `uint32_t` (4 bytes, little-endian, 4-byte
/// aligned) and advance the cursor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_write_u32(
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
    value: u32,
) -> i32 {
    with_writer(ptr, end, origin, |w| w.write_u32(value))
}

/// Write a CDR-encoded `int32_t` and advance the cursor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_write_i32(
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
    value: i32,
) -> i32 {
    with_writer(ptr, end, origin, |w| w.write_i32(value))
}

/// Write a CDR-encoded `uint64_t` (8 bytes, little-endian, 8-byte
/// aligned) and advance the cursor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_write_u64(
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
    value: u64,
) -> i32 {
    with_writer(ptr, end, origin, |w| w.write_u64(value))
}

/// Write a CDR-encoded `int64_t` and advance the cursor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_write_i64(
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
    value: i64,
) -> i32 {
    with_writer(ptr, end, origin, |w| w.write_i64(value))
}

/// Write a CDR-encoded IEEE-754 `float` (4-byte aligned) and advance
/// the cursor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_write_f32(
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
    value: f32,
) -> i32 {
    with_writer(ptr, end, origin, |w| w.write_f32(value))
}

/// Write a CDR-encoded IEEE-754 `double` (8-byte aligned) and advance
/// the cursor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_write_f64(
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
    value: f64,
) -> i32 {
    with_writer(ptr, end, origin, |w| w.write_f64(value))
}

/// Write a null-terminated string (CDR: u32 length inc. null + bytes + null).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_write_string(
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
    value: *const c_char,
) -> i32 {
    if value.is_null() {
        return -1;
    }
    let mut len = 0usize;
    while *value.add(len) != 0 {
        len += 1;
    }
    let data = core::slice::from_raw_parts(value.cast::<u8>(), len);
    with_writer(ptr, end, origin, |w| write_string_payload(w, data))
}

/// Write a ptr+len string (CDR: u32 length inc. null + bytes + null).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_write_string_n(
    ptr: *mut *mut u8,
    end: *const u8,
    origin: *const u8,
    data: *const c_char,
    data_len: usize,
) -> i32 {
    if data.is_null() && data_len > 0 {
        return -1;
    }
    let slice = if data_len == 0 {
        &[][..]
    } else {
        core::slice::from_raw_parts(data.cast::<u8>(), data_len)
    };
    with_writer(ptr, end, origin, |w| write_string_payload(w, slice))
}

#[inline]
fn write_string_payload(w: &mut CdrWriter<'_>, data: &[u8]) -> Result<(), SerError> {
    let total = (data.len() + 1) as u32;
    w.write_u32(total)?;
    w.write_bytes(data)?;
    w.write_u8(0)
}

// ===========================================================================
// Read functions
// ===========================================================================

/// Read a CDR-encoded `bool` (1 byte) into `*value` and advance the
/// cursor.
///
/// Returns 0 on success, -1 if the cursor would overrun `end`, the
/// payload is malformed, or any pointer is null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_read_bool(
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
    value: *mut bool,
) -> i32 {
    if value.is_null() {
        return -1;
    }
    with_reader(ptr, end, origin, |r| {
        *value = r.read_bool()?;
        Ok(())
    })
}

/// Read a CDR-encoded `uint8_t` into `*value` and advance the cursor.
/// See [`nros_cdr_read_bool`] for the buffer contract.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_read_u8(
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
    value: *mut u8,
) -> i32 {
    if value.is_null() {
        return -1;
    }
    with_reader(ptr, end, origin, |r| {
        *value = r.read_u8()?;
        Ok(())
    })
}

/// Read a CDR-encoded `int8_t` into `*value` and advance the cursor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_read_i8(
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
    value: *mut i8,
) -> i32 {
    if value.is_null() {
        return -1;
    }
    with_reader(ptr, end, origin, |r| {
        *value = r.read_i8()?;
        Ok(())
    })
}

/// Read a CDR-encoded `uint16_t` (2-byte aligned) into `*value`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_read_u16(
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
    value: *mut u16,
) -> i32 {
    if value.is_null() {
        return -1;
    }
    with_reader(ptr, end, origin, |r| {
        *value = r.read_u16()?;
        Ok(())
    })
}

/// Read a CDR-encoded `int16_t` into `*value`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_read_i16(
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
    value: *mut i16,
) -> i32 {
    if value.is_null() {
        return -1;
    }
    with_reader(ptr, end, origin, |r| {
        *value = r.read_i16()?;
        Ok(())
    })
}

/// Read a CDR-encoded `uint32_t` (4-byte aligned) into `*value`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_read_u32(
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
    value: *mut u32,
) -> i32 {
    if value.is_null() {
        return -1;
    }
    with_reader(ptr, end, origin, |r| {
        *value = r.read_u32()?;
        Ok(())
    })
}

/// Read a CDR-encoded `int32_t` into `*value`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_read_i32(
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
    value: *mut i32,
) -> i32 {
    if value.is_null() {
        return -1;
    }
    with_reader(ptr, end, origin, |r| {
        *value = r.read_i32()?;
        Ok(())
    })
}

/// Read a CDR-encoded `uint64_t` (8-byte aligned) into `*value`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_read_u64(
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
    value: *mut u64,
) -> i32 {
    if value.is_null() {
        return -1;
    }
    with_reader(ptr, end, origin, |r| {
        *value = r.read_u64()?;
        Ok(())
    })
}

/// Read a CDR-encoded `int64_t` into `*value`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_read_i64(
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
    value: *mut i64,
) -> i32 {
    if value.is_null() {
        return -1;
    }
    with_reader(ptr, end, origin, |r| {
        *value = r.read_i64()?;
        Ok(())
    })
}

/// Read a CDR-encoded IEEE-754 `float` into `*value`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_read_f32(
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
    value: *mut f32,
) -> i32 {
    if value.is_null() {
        return -1;
    }
    with_reader(ptr, end, origin, |r| {
        *value = r.read_f32()?;
        Ok(())
    })
}

/// Read a CDR-encoded IEEE-754 `double` into `*value`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_read_f64(
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
    value: *mut f64,
) -> i32 {
    if value.is_null() {
        return -1;
    }
    with_reader(ptr, end, origin, |r| {
        *value = r.read_f64()?;
        Ok(())
    })
}

/// Read a CDR string into a fixed-size C buffer. Fails if the encoded length
/// exceeds `max_len`. Always null-terminates the output on success.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_read_string(
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
    value: *mut c_char,
    max_len: usize,
) -> i32 {
    if value.is_null() || max_len == 0 {
        return -1;
    }
    with_reader(ptr, end, origin, |r| {
        let str_len = r.read_u32()? as usize;
        if str_len > max_len {
            return Err(DeserError::UnexpectedEof);
        }
        let bytes = r.read_bytes(str_len)?;
        // CDR-encoded strings include a trailing null; strip it from the copy.
        let copy_len = str_len.saturating_sub(1);
        for (i, &b) in bytes.iter().take(copy_len).enumerate() {
            *value.add(i) = b as c_char;
        }
        *value.add(copy_len) = 0;
        Ok(())
    })
}

/// Advance the cursor to the next `alignment`-byte boundary of the CDR STREAM
/// (relative to `origin`, never to the buffer pointer), consuming the padding
/// the writer emitted. This is [`CdrReader::align`] behind the C ABI, so it
/// carries the same edition rule (XCDR2 caps 8-byte primitives at 4) and the
/// header-only readers in `<nros/view.h>` agree with the Rust reader byte for
/// byte instead of re-deriving the arithmetic (issue 1148).
///
/// Returns 0 on success, -1 if `alignment` is 0 or the padding would run past
/// `end`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cdr_align(
    ptr: *mut *const u8,
    end: *const u8,
    origin: *const u8,
    alignment: usize,
) -> i32 {
    if alignment == 0 {
        return -1;
    }
    with_reader(ptr, end, origin, |r| r.align(alignment))
}

// ===========================================================================
// Tests
// ===========================================================================

// Phase 214.G.2 follow-up — gate the unit-test module behind `feature
// = "std"`. The tests use `std::ffi::CStr` (line ~565) which doesn't
// resolve in a `--no-default-features` build, breaking the workspace-
// wide `cargo test --no-run` smoke gate. Wrap the whole `mod tests`
// in the std-feature cfg so it's silently dropped when std is off.
// phase-359 W10 — these tests contain no `std::` path at all; the gate was
// inherited, not required.
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_write_read_u32() {
        let mut buffer = [0u8; 16];
        let mut ptr = buffer.as_mut_ptr();
        let end = unsafe { buffer.as_ptr().add(buffer.len()) };
        let origin = buffer.as_ptr();
        unsafe {
            assert_eq!(nros_cdr_write_u32(&mut ptr, end, origin, 0x12345678), 0);
        }
        let mut read_ptr = buffer.as_ptr();
        let mut value: u32 = 0;
        unsafe {
            assert_eq!(nros_cdr_read_u32(&mut read_ptr, end, origin, &mut value), 0);
        }
        assert_eq!(value, 0x12345678);
    }

    #[test]
    fn test_write_read_i32() {
        let mut buffer = [0u8; 16];
        let mut ptr = buffer.as_mut_ptr();
        let end = unsafe { buffer.as_ptr().add(buffer.len()) };
        let origin = buffer.as_ptr();
        unsafe {
            assert_eq!(nros_cdr_write_i32(&mut ptr, end, origin, -12345), 0);
        }
        let mut read_ptr = buffer.as_ptr();
        let mut value: i32 = 0;
        unsafe {
            assert_eq!(nros_cdr_read_i32(&mut read_ptr, end, origin, &mut value), 0);
        }
        assert_eq!(value, -12345);
    }

    #[test]
    fn test_write_read_f64() {
        let mut buffer = [0u8; 16];
        let mut ptr = buffer.as_mut_ptr();
        let end = unsafe { buffer.as_ptr().add(buffer.len()) };
        let origin = buffer.as_ptr();
        let test_val = 1234567.89012345_f64;
        unsafe {
            assert_eq!(nros_cdr_write_f64(&mut ptr, end, origin, test_val), 0);
        }
        let mut read_ptr = buffer.as_ptr();
        let mut value: f64 = 0.0;
        unsafe {
            assert_eq!(nros_cdr_read_f64(&mut read_ptr, end, origin, &mut value), 0);
        }
        assert_eq!(value, test_val);
    }

    #[test]
    fn test_write_read_string() {
        let mut buffer = [0u8; 64];
        let mut ptr = buffer.as_mut_ptr();
        let end = unsafe { buffer.as_ptr().add(buffer.len()) };
        let origin = buffer.as_ptr();
        let test_str = b"Hello, World!\0";
        unsafe {
            assert_eq!(
                nros_cdr_write_string(&mut ptr, end, origin, test_str.as_ptr() as *const c_char),
                0
            );
        }
        let mut read_ptr = buffer.as_ptr();
        // `c_char`, not `i8` — the latter only matches on x86 (issue: `c_char`
        // is `u8` on ARM/aarch64), so a hardcoded `i8` buffer fails to compile there.
        let mut value: [c_char; 32] = [0; 32];
        unsafe {
            assert_eq!(
                nros_cdr_read_string(&mut read_ptr, end, origin, value.as_mut_ptr(), value.len()),
                0
            );
        }
        let result = unsafe { std::ffi::CStr::from_ptr(value.as_ptr()).to_str().unwrap() };
        assert_eq!(result, "Hello, World!");
    }

    #[test]
    fn test_alignment() {
        let mut buffer = [0u8; 32];
        let mut ptr = buffer.as_mut_ptr();
        let end = unsafe { buffer.as_ptr().add(buffer.len()) };
        let origin = buffer.as_ptr();
        unsafe {
            assert_eq!(nros_cdr_write_u8(&mut ptr, end, origin, 0xAA), 0);
            assert_eq!(nros_cdr_write_u32(&mut ptr, end, origin, 0x12345678), 0);
        }
        assert_eq!(buffer[0], 0xAA);
        assert_eq!(buffer[1], 0);
        assert_eq!(buffer[2], 0);
        assert_eq!(buffer[3], 0);
        assert_eq!(buffer[4], 0x78);
        assert_eq!(buffer[5], 0x56);
        assert_eq!(buffer[6], 0x34);
        assert_eq!(buffer[7], 0x12);
    }

    #[test]
    fn test_alignment_with_offset() {
        let mut buffer = [0u8; 32];
        let end = unsafe { buffer.as_ptr().add(buffer.len()) };
        buffer[0] = 0x00;
        buffer[1] = 0x01;
        buffer[2] = 0x00;
        buffer[3] = 0x00;
        let origin = unsafe { buffer.as_ptr().add(4) };
        let mut ptr = unsafe { buffer.as_mut_ptr().add(4) };
        let test_val: i64 = 0x0102030405060708;
        unsafe {
            assert_eq!(nros_cdr_write_i64(&mut ptr, end, origin, test_val), 0);
        }
        assert_eq!(ptr, unsafe { buffer.as_mut_ptr().add(12) });
        let mut read_ptr: *const u8 = unsafe { buffer.as_ptr().add(4) };
        let mut value: i64 = 0;
        unsafe {
            assert_eq!(nros_cdr_read_i64(&mut read_ptr, end, origin, &mut value), 0);
        }
        assert_eq!(value, test_val);
    }

    #[test]
    fn test_two_i64_after_header() {
        let mut buffer = [0u8; 32];
        let end = unsafe { buffer.as_ptr().add(buffer.len()) };
        buffer[0] = 0x00;
        buffer[1] = 0x01;
        buffer[2] = 0x00;
        buffer[3] = 0x00;
        let origin = unsafe { buffer.as_ptr().add(4) };
        let mut ptr = unsafe { buffer.as_mut_ptr().add(4) };
        unsafe {
            assert_eq!(nros_cdr_write_i64(&mut ptr, end, origin, 3), 0);
            assert_eq!(nros_cdr_write_i64(&mut ptr, end, origin, 5), 0);
        }
        assert_eq!(ptr, unsafe { buffer.as_mut_ptr().add(20) });
        let mut read_ptr: *const u8 = unsafe { buffer.as_ptr().add(4) };
        let mut a: i64 = 0;
        let mut b: i64 = 0;
        unsafe {
            assert_eq!(nros_cdr_read_i64(&mut read_ptr, end, origin, &mut a), 0);
            assert_eq!(nros_cdr_read_i64(&mut read_ptr, end, origin, &mut b), 0);
        }
        assert_eq!(a, 3);
        assert_eq!(b, 5);
    }

    #[test]
    fn test_null_safety() {
        let end: *const u8 = core::ptr::null();
        let origin: *const u8 = core::ptr::null();
        // NULL ptr → -1
        assert_eq!(
            unsafe { nros_cdr_write_u8(core::ptr::null_mut(), end, origin, 0) },
            -1
        );
        let mut val: u32 = 0;
        assert_eq!(
            unsafe { nros_cdr_read_u32(core::ptr::null_mut(), end, origin, &mut val) },
            -1
        );
    }
}
