//! CDR encoder/decoder with alignment handling

use crate::{
    CDR_LE_HEADER, CDR2_DELIMITED_LE_HEADER,
    error::{DeserError, SerError},
};

/// CDR writer for serialization.
///
/// Handles alignment and endianness for ROS 2 CDR encoding.
/// Alignment is computed relative to `origin` — when a 4-byte CDR
/// header is present, `origin = 4` so that fields align correctly
/// within the payload portion of the buffer.
/// CDR encoding version. XCDR1 is the historical default (PLAIN_CDR, no
/// DHEADER, 8-byte primitives align to 8). XCDR2 (phase-303 W2 / RFC-0055 /
/// #0267) is DELIMITED_CDR for APPENDABLE types: every struct is wrapped in a
/// 4-byte DHEADER and 8-byte primitives align to 4.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EncodingVersion {
    /// PLAIN_CDR (encapsulation `0x0001`). The default; byte-identical to every
    /// pre-W2 stream.
    #[default]
    Xcdr1,
    /// DELIMITED_CDR2 (encapsulation `0x0009`) — appendable + DHEADER.
    Xcdr2,
}

/// Opaque marker returned by [`CdrWriter::begin_dheader`], passed back to
/// [`CdrWriter::end_dheader`]. Under XCDR1 it carries nothing (the DHEADER calls
/// are no-ops); under XCDR2 it holds the reserved DHEADER position.
#[derive(Debug, Clone, Copy)]
pub struct DHeaderMark(Option<usize>);

impl DHeaderMark {
    /// The reserved DHEADER byte offset (XCDR2), or `None` under XCDR1. Lets the
    /// C FFI carry the mark across the `extern "C"` boundary (phase-303 W4).
    #[inline]
    pub fn raw(self) -> Option<usize> {
        self.0
    }

    /// Reconstruct a mark from its [`raw`](Self::raw) offset (FFI round-trip).
    #[inline]
    pub fn from_raw(at: Option<usize>) -> Self {
        DHeaderMark(at)
    }
}

/// Opaque scope returned by [`CdrReader::begin_dheader`], passed back to
/// [`CdrReader::end_dheader`]. Under XCDR1 it carries nothing (no-op); under
/// XCDR2 it holds the absolute buffer offset of the delimited struct's end.
#[derive(Debug, Clone, Copy)]
pub struct DHeaderScope(Option<usize>);

impl DHeaderScope {
    /// The delimited struct's end offset (XCDR2), or `None` under XCDR1 (FFI).
    #[inline]
    pub fn raw(self) -> Option<usize> {
        self.0
    }

    /// Reconstruct a scope from its [`raw`](Self::raw) offset (FFI round-trip).
    #[inline]
    pub fn from_raw(end: Option<usize>) -> Self {
        DHeaderScope(end)
    }
}

pub struct CdrWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
    /// Byte offset where payload data begins (0 for raw, 4 after CDR header).
    /// Alignment padding is calculated as `(pos - origin) % alignment`.
    origin: usize,
    /// CDR encoding version — drives DHEADER emission + the alignment cap.
    version: EncodingVersion,
    /// Phase 380 W3 — MEASURE mode: advance `pos` exactly as a real write
    /// would, and copy nothing.
    ///
    /// This is how `serialized_size` stays exact: it is not a second
    /// implementation that could disagree with the writer, it IS the writer,
    /// with the stores turned off. Every alignment rule, every DHEADER, every
    /// `len + 1` string prefix is therefore counted by the same code that emits
    /// it, and a change to one cannot drift from the other.
    measure: bool,
}

impl<'a> CdrWriter<'a> {
    /// Create a new CDR writer
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self {
            buf,
            pos: 0,
            origin: 0,
            version: EncodingVersion::Xcdr1,
            measure: false,
        }
    }

    /// Create a CDR writer positioned at `pos` bytes into `buf`.
    ///
    /// `origin` stays at 0, so alignment is computed relative to the start
    /// of `buf`. Used by FFI bridges that hand us a `(origin, cursor, end)`
    /// triple where `buf = origin..end` and the caller's cursor is `pos`.
    pub fn new_at(buf: &'a mut [u8], pos: usize) -> Result<Self, SerError> {
        if pos > buf.len() {
            return Err(SerError::BufferTooSmall);
        }
        Ok(Self {
            buf,
            pos,
            origin: 0,
            version: EncodingVersion::Xcdr1,
            measure: false,
        })
    }

    /// Like [`new_at`](Self::new_at) but XCDR2 (DELIMITED_CDR2) — 8-byte
    /// primitives align to 4 and [`begin_dheader`](Self::begin_dheader) emits a
    /// DHEADER. Used by the C FFI (`nros-c`) tx path, which manages the
    /// encapsulation header + cursor itself (phase-303 W4 / #0267).
    pub fn new_at_xcdr2(buf: &'a mut [u8], pos: usize) -> Result<Self, SerError> {
        if pos > buf.len() {
            return Err(SerError::BufferTooSmall);
        }
        Ok(Self {
            buf,
            pos,
            origin: 0,
            version: EncodingVersion::Xcdr2,
            measure: false,
        })
    }

    /// Create a new CDR writer with the 4-byte encapsulation header.
    ///
    /// Writes `[0x00, 0x01, 0x00, 0x00]` (CDR little-endian) at the start
    /// and sets `origin = 4` so subsequent alignment is relative to the
    /// payload, not the header. This is the normal entry point for
    /// serialising ROS 2 messages.
    pub fn new_with_header(buf: &'a mut [u8]) -> Result<Self, SerError> {
        if buf.len() < 4 {
            return Err(SerError::BufferTooSmall);
        }
        buf[0..4].copy_from_slice(&CDR_LE_HEADER);
        Ok(Self {
            buf,
            pos: 4,
            origin: 4,
            version: EncodingVersion::Xcdr1,
            measure: false,
        })
    }

    /// Create an XCDR2 (DELIMITED_CDR2) writer with the `0x0009` encapsulation
    /// header (phase-303 W2 / #0267). Each struct — top-level and nested — must
    /// be wrapped in [`begin_dheader`](Self::begin_dheader) /
    /// [`end_dheader`](Self::end_dheader); 8-byte primitives align to 4.
    pub fn new_with_header_xcdr2(buf: &'a mut [u8]) -> Result<Self, SerError> {
        if buf.len() < 4 {
            return Err(SerError::BufferTooSmall);
        }
        buf[0..4].copy_from_slice(&CDR2_DELIMITED_LE_HEADER);
        Ok(Self {
            buf,
            pos: 4,
            origin: 4,
            version: EncodingVersion::Xcdr2,
            measure: false,
        })
    }

    /// Phase 380 W3 — a writer that counts bytes and stores none.
    ///
    /// `serialized_size` needs the EXACT size of one message, which a type-level
    /// bound cannot give for an unbounded type and which a second walk of the
    /// schema could only approximate. Running the real writer with its stores
    /// disabled makes the count exact by construction: same alignment, same
    /// DHEADERs, same `len + 1` string prefix, because it is the same code.
    ///
    /// Pass an empty slice — the buffer is never touched:
    ///
    /// ```ignore
    /// let mut w = CdrWriter::measuring(&mut [], EncodingVersion::Xcdr1);
    /// value.serialize(&mut w)?;
    /// let bytes = w.position();
    /// ```
    ///
    /// The encapsulation header is NOT counted: like the real constructors'
    /// `origin`, a measuring writer starts at 0. Add
    /// [`crate::size::ENCAPSULATION_HEADER_BYTES`] for the payload a publisher
    /// hands the transport.
    pub fn measuring(buf: &'a mut [u8], version: EncodingVersion) -> Self {
        Self {
            buf,
            pos: 0,
            origin: 0,
            version,
            measure: true,
        }
    }

    /// The CDR encoding version this writer emits.
    #[inline]
    pub fn version(&self) -> EncodingVersion {
        self.version
    }

    /// Begin a DHEADER-delimited struct. Under XCDR2, aligns to 4, reserves a
    /// 4-byte size slot (backpatched by [`end_dheader`](Self::end_dheader)), and
    /// returns its position. Under XCDR1 this is a NO-OP (returns an empty mark),
    /// so generated `serialize` bodies can wrap every struct unconditionally
    /// while XCDR1 output stays byte-identical.
    #[inline]
    pub fn begin_dheader(&mut self) -> Result<DHeaderMark, SerError> {
        match self.version {
            EncodingVersion::Xcdr1 => Ok(DHeaderMark(None)),
            EncodingVersion::Xcdr2 => {
                self.align(4)?;
                if self.remaining() < 4 {
                    return Err(SerError::BufferTooSmall);
                }
                let at = self.pos;
                if !self.measure {
                    self.buf[at..at + 4].copy_from_slice(&[0, 0, 0, 0]);
                }
                self.pos += 4;
                Ok(DHeaderMark(Some(at)))
            }
        }
    }

    /// Finish a DHEADER-delimited struct: backpatch the reserved slot with the
    /// serialized size of the member block written since
    /// [`begin_dheader`](Self::begin_dheader). No-op under XCDR1.
    #[inline]
    pub fn end_dheader(&mut self, mark: DHeaderMark) -> Result<(), SerError> {
        if let Some(at) = mark.0 {
            let size = (self.pos - (at + 4)) as u32;
            if !self.measure {
                self.buf[at..at + 4].copy_from_slice(&size.to_le_bytes());
            }
        }
        Ok(())
    }

    /// Get current position in buffer
    #[inline]
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Get remaining capacity
    #[inline]
    pub fn remaining(&self) -> usize {
        if self.measure {
            // Nothing is stored, so nothing can overflow — a measuring pass must
            // never report BufferTooSmall or it would stop counting early and
            // under-report, which is the dangerous direction.
            return usize::MAX;
        }
        self.buf.len().saturating_sub(self.pos)
    }

    /// Get the written bytes
    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.pos]
    }

    /// Align to the given boundary (relative to origin). Under XCDR2 the
    /// alignment is capped at 4 — 8-byte primitives align to 4, not 8 (the one
    /// primitive-layout difference from XCDR1).
    #[inline]
    pub fn align(&mut self, alignment: usize) -> Result<(), SerError> {
        let alignment = match self.version {
            EncodingVersion::Xcdr2 => alignment.min(4),
            EncodingVersion::Xcdr1 => alignment,
        };
        let offset = self.pos - self.origin;
        let padding = (alignment - (offset % alignment)) % alignment;
        if self.remaining() < padding {
            return Err(SerError::BufferTooSmall);
        }
        // Fill padding with zeros
        for i in 0..padding {
            if !self.measure {
                self.buf[self.pos + i] = 0;
            }
        }
        self.pos += padding;
        Ok(())
    }

    /// Write a single byte without alignment
    #[inline]
    pub fn write_u8(&mut self, value: u8) -> Result<(), SerError> {
        if self.remaining() < 1 {
            return Err(SerError::BufferTooSmall);
        }
        if !self.measure {
            self.buf[self.pos] = value;
        }
        self.pos += 1;
        Ok(())
    }

    /// Write a boolean (serialized as a single byte: 0 = false, 1 = true)
    #[inline]
    pub fn write_bool(&mut self, value: bool) -> Result<(), SerError> {
        self.write_u8(value as u8)
    }

    /// Write i8 without alignment
    #[inline]
    pub fn write_i8(&mut self, value: i8) -> Result<(), SerError> {
        self.write_u8(value as u8)
    }

    /// Write bytes without alignment
    #[inline]
    pub fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), SerError> {
        if self.remaining() < bytes.len() {
            return Err(SerError::BufferTooSmall);
        }
        if !self.measure {
            self.buf[self.pos..self.pos + bytes.len()].copy_from_slice(bytes);
        }
        self.pos += bytes.len();
        Ok(())
    }

    /// Write u16 with alignment (little-endian)
    #[inline]
    pub fn write_u16(&mut self, value: u16) -> Result<(), SerError> {
        self.align(2)?;
        if self.remaining() < 2 {
            return Err(SerError::BufferTooSmall);
        }
        if !self.measure {
            self.buf[self.pos..self.pos + 2].copy_from_slice(&value.to_le_bytes());
        }
        self.pos += 2;
        Ok(())
    }

    /// Write u32 with alignment (little-endian)
    #[inline]
    pub fn write_u32(&mut self, value: u32) -> Result<(), SerError> {
        self.align(4)?;
        if self.remaining() < 4 {
            return Err(SerError::BufferTooSmall);
        }
        if !self.measure {
            self.buf[self.pos..self.pos + 4].copy_from_slice(&value.to_le_bytes());
        }
        self.pos += 4;
        Ok(())
    }

    /// Write u64 with alignment (little-endian)
    #[inline]
    pub fn write_u64(&mut self, value: u64) -> Result<(), SerError> {
        self.align(8)?;
        if self.remaining() < 8 {
            return Err(SerError::BufferTooSmall);
        }
        if !self.measure {
            self.buf[self.pos..self.pos + 8].copy_from_slice(&value.to_le_bytes());
        }
        self.pos += 8;
        Ok(())
    }

    /// Write i16 with alignment (little-endian)
    #[inline]
    pub fn write_i16(&mut self, value: i16) -> Result<(), SerError> {
        self.write_u16(value as u16)
    }

    /// Write i32 with alignment (little-endian)
    #[inline]
    pub fn write_i32(&mut self, value: i32) -> Result<(), SerError> {
        self.write_u32(value as u32)
    }

    /// Write i64 with alignment (little-endian)
    #[inline]
    pub fn write_i64(&mut self, value: i64) -> Result<(), SerError> {
        self.write_u64(value as u64)
    }

    /// Write f32 with alignment (little-endian)
    #[inline]
    pub fn write_f32(&mut self, value: f32) -> Result<(), SerError> {
        self.write_u32(value.to_bits())
    }

    /// Write f64 with alignment (little-endian)
    #[inline]
    pub fn write_f64(&mut self, value: f64) -> Result<(), SerError> {
        self.write_u64(value.to_bits())
    }

    /// Write a CDR string (4-byte length including null + data + null terminator)
    pub fn write_string(&mut self, s: &str) -> Result<(), SerError> {
        let len = s.len() + 1; // Include null terminator
        if len > u32::MAX as usize {
            return Err(SerError::StringTooLong);
        }
        self.write_u32(len as u32)?;
        self.write_bytes(s.as_bytes())?;
        self.write_u8(0)?; // Null terminator
        Ok(())
    }

    /// Write a sequence length (4-byte count)
    #[inline]
    pub fn write_sequence_len(&mut self, len: usize) -> Result<(), SerError> {
        if len > u32::MAX as usize {
            return Err(SerError::SequenceTooLong);
        }
        self.write_u32(len as u32)
    }
}

/// CDR reader for deserialization
///
/// Handles alignment and endianness for CDR decoding.
pub struct CdrReader<'a> {
    buf: &'a [u8],
    pos: usize,
    origin: usize,
    /// CDR encoding version — parsed from the encapsulation header; drives
    /// DHEADER handling + the alignment cap.
    version: EncodingVersion,
}

impl<'a> CdrReader<'a> {
    /// Create a new CDR reader
    pub fn new(buf: &'a [u8]) -> Self {
        Self {
            buf,
            pos: 0,
            origin: 0,
            version: EncodingVersion::Xcdr1,
        }
    }

    /// Create a CDR reader positioned at `pos` bytes into `buf`.
    ///
    /// `origin` stays at 0, so alignment is computed relative to the start
    /// of `buf`. Used by FFI bridges that hand us a `(origin, cursor, end)`
    /// triple where `buf = origin..end` and the caller's cursor is `pos`.
    pub fn new_at(buf: &'a [u8], pos: usize) -> Result<Self, DeserError> {
        if pos > buf.len() {
            return Err(DeserError::UnexpectedEof);
        }
        Ok(Self {
            buf,
            pos,
            origin: 0,
            version: EncodingVersion::Xcdr1,
        })
    }

    /// Like [`new_at`](Self::new_at) but XCDR2 (DELIMITED_CDR2) — the align cap +
    /// [`begin_dheader`](Self::begin_dheader) read the DHEADER. For the C FFI rx
    /// path, which strips the encapsulation header itself (phase-303 W4).
    pub fn new_at_xcdr2(buf: &'a [u8], pos: usize) -> Result<Self, DeserError> {
        if pos > buf.len() {
            return Err(DeserError::UnexpectedEof);
        }
        Ok(Self {
            buf,
            pos,
            origin: 0,
            version: EncodingVersion::Xcdr2,
        })
    }

    /// Create a new CDR reader, parsing and validating the encapsulation header
    ///
    /// Expects a 4-byte CDR header at the start of the buffer.
    pub fn new_with_header(buf: &'a [u8]) -> Result<Self, DeserError> {
        if buf.len() < 4 {
            return Err(DeserError::UnexpectedEof);
        }
        // Parse the encapsulation id. XCDR1 PLAIN_CDR (0x0000 BE / 0x0001 LE)
        // and XCDR2 DELIMITED_CDR2 (0x0008 BE / 0x0009 LE — appendable, the
        // form nano-ros emits/expects). We decode little-endian regardless.
        if buf[0] != 0x00 {
            return Err(DeserError::InvalidHeader);
        }
        let version = match buf[1] {
            0x00 | 0x01 => EncodingVersion::Xcdr1,
            0x08 | 0x09 => EncodingVersion::Xcdr2,
            _ => return Err(DeserError::InvalidHeader),
        };
        Ok(Self {
            buf,
            pos: 4,
            origin: 4,
            version,
        })
    }

    /// Get current position in buffer
    #[inline]
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Whether nothing has been read yet — the reader still sits on its
    /// alignment origin.
    ///
    /// CDR alignment is computed as `pos - origin`, so a caller that re-reads a
    /// request by taking the remaining bytes and building a fresh reader over
    /// them only gets identical alignment when the slice STARTS at the origin.
    /// That is a real precondition with no visible symptom when broken — the
    /// padding silently shifts — and `origin` is private, so this is how a
    /// caller checks it. See `parameter_services::rewind` (phase-382 W1').
    #[inline]
    pub fn is_at_origin(&self) -> bool {
        self.pos == self.origin
    }

    /// Get remaining bytes
    #[inline]
    pub fn remaining(&self) -> usize {
        self.buf.len().saturating_sub(self.pos)
    }

    /// Check if reader is at end of buffer
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.remaining() == 0
    }

    /// Align to the given boundary (relative to origin). Under XCDR2 the
    /// alignment is capped at 4 (mirrors [`CdrWriter::align`]).
    #[inline]
    pub fn align(&mut self, alignment: usize) -> Result<(), DeserError> {
        let alignment = match self.version {
            EncodingVersion::Xcdr2 => alignment.min(4),
            EncodingVersion::Xcdr1 => alignment,
        };
        let offset = self.pos - self.origin;
        let padding = (alignment - (offset % alignment)) % alignment;
        if self.remaining() < padding {
            return Err(DeserError::UnexpectedEof);
        }
        self.pos += padding;
        Ok(())
    }

    /// The CDR encoding version parsed from the header.
    #[inline]
    pub fn version(&self) -> EncodingVersion {
        self.version
    }

    /// Begin reading a DHEADER-delimited struct. Under XCDR2, aligns to 4, reads
    /// the 4-byte DHEADER size, and returns the absolute buffer position of the
    /// struct's END (so trailing unknown members can be skipped for forward
    /// compat). Under XCDR1 this is a NO-OP (returns `None`) — generated
    /// `deserialize` bodies wrap every struct unconditionally.
    #[inline]
    pub fn begin_dheader(&mut self) -> Result<DHeaderScope, DeserError> {
        match self.version {
            EncodingVersion::Xcdr1 => Ok(DHeaderScope(None)),
            EncodingVersion::Xcdr2 => {
                let size = self.read_u32()? as usize;
                let end = self
                    .pos
                    .checked_add(size)
                    .ok_or(DeserError::UnexpectedEof)?;
                if end > self.buf.len() {
                    return Err(DeserError::UnexpectedEof);
                }
                Ok(DHeaderScope(Some(end)))
            }
        }
    }

    /// Finish a DHEADER-delimited struct: skip any trailing bytes the writer
    /// included beyond the members we read (unknown appended members — XCDR2
    /// forward compatibility). No-op under XCDR1. Errors if we OVER-read past the
    /// declared end (a corrupt/mismatched stream).
    #[inline]
    pub fn end_dheader(&mut self, scope: DHeaderScope) -> Result<(), DeserError> {
        if let Some(end) = scope.0 {
            if self.pos > end {
                return Err(DeserError::DHeaderOverrun);
            }
            self.pos = end;
        }
        Ok(())
    }

    /// Read a single byte without alignment
    #[inline]
    pub fn read_u8(&mut self) -> Result<u8, DeserError> {
        if self.remaining() < 1 {
            return Err(DeserError::UnexpectedEof);
        }
        let value = self.buf[self.pos];
        self.pos += 1;
        Ok(value)
    }

    /// Read a boolean (deserialized from a single byte: 0 = false, non-zero = true)
    #[inline]
    pub fn read_bool(&mut self) -> Result<bool, DeserError> {
        Ok(self.read_u8()? != 0)
    }

    /// Read i8 without alignment
    #[inline]
    pub fn read_i8(&mut self) -> Result<i8, DeserError> {
        Ok(self.read_u8()? as i8)
    }

    /// Read bytes without alignment
    #[inline]
    pub fn read_bytes(&mut self, len: usize) -> Result<&'a [u8], DeserError> {
        if self.remaining() < len {
            return Err(DeserError::UnexpectedEof);
        }
        let bytes = &self.buf[self.pos..self.pos + len];
        self.pos += len;
        Ok(bytes)
    }

    /// Read u16 with alignment (little-endian)
    #[inline]
    pub fn read_u16(&mut self) -> Result<u16, DeserError> {
        self.align(2)?;
        if self.remaining() < 2 {
            return Err(DeserError::UnexpectedEof);
        }
        let value = u16::from_le_bytes([self.buf[self.pos], self.buf[self.pos + 1]]);
        self.pos += 2;
        Ok(value)
    }

    /// Read u32 with alignment (little-endian)
    #[inline]
    pub fn read_u32(&mut self) -> Result<u32, DeserError> {
        self.align(4)?;
        if self.remaining() < 4 {
            return Err(DeserError::UnexpectedEof);
        }
        let value = u32::from_le_bytes([
            self.buf[self.pos],
            self.buf[self.pos + 1],
            self.buf[self.pos + 2],
            self.buf[self.pos + 3],
        ]);
        self.pos += 4;
        Ok(value)
    }

    /// Read u64 with alignment (little-endian)
    #[inline]
    pub fn read_u64(&mut self) -> Result<u64, DeserError> {
        self.align(8)?;
        if self.remaining() < 8 {
            return Err(DeserError::UnexpectedEof);
        }
        let value = u64::from_le_bytes([
            self.buf[self.pos],
            self.buf[self.pos + 1],
            self.buf[self.pos + 2],
            self.buf[self.pos + 3],
            self.buf[self.pos + 4],
            self.buf[self.pos + 5],
            self.buf[self.pos + 6],
            self.buf[self.pos + 7],
        ]);
        self.pos += 8;
        Ok(value)
    }

    /// Read i16 with alignment (little-endian)
    #[inline]
    pub fn read_i16(&mut self) -> Result<i16, DeserError> {
        Ok(self.read_u16()? as i16)
    }

    /// Read i32 with alignment (little-endian)
    #[inline]
    pub fn read_i32(&mut self) -> Result<i32, DeserError> {
        Ok(self.read_u32()? as i32)
    }

    /// Read i64 with alignment (little-endian)
    #[inline]
    pub fn read_i64(&mut self) -> Result<i64, DeserError> {
        Ok(self.read_u64()? as i64)
    }

    /// Read f32 with alignment (little-endian)
    #[inline]
    pub fn read_f32(&mut self) -> Result<f32, DeserError> {
        Ok(f32::from_bits(self.read_u32()?))
    }

    /// Read f64 with alignment (little-endian)
    #[inline]
    pub fn read_f64(&mut self) -> Result<f64, DeserError> {
        Ok(f64::from_bits(self.read_u64()?))
    }

    /// Read a CDR string (4-byte length including null + data + null terminator)
    ///
    /// Returns a string slice pointing into the buffer (zero-copy).
    pub fn read_string(&mut self) -> Result<&'a str, DeserError> {
        let len = self.read_u32()? as usize;
        if len == 0 {
            return Err(DeserError::InvalidData);
        }
        if self.remaining() < len {
            return Err(DeserError::UnexpectedEof);
        }
        // Length includes null terminator, so actual string is len - 1 bytes
        let bytes = &self.buf[self.pos..self.pos + len - 1];
        self.pos += len;
        core::str::from_utf8(bytes).map_err(|_| DeserError::InvalidUtf8)
    }

    /// Read a sequence length (4-byte count)
    #[inline]
    pub fn read_sequence_len(&mut self) -> Result<usize, DeserError> {
        Ok(self.read_u32()? as usize)
    }

    // ── Borrowed slice readers (zero-copy for primitive sequences) ──

    /// Read a `uint8[]` / `byte[]` sequence as a borrowed slice.
    ///
    /// Returns `&'a [u8]` pointing directly into the CDR buffer. Zero-copy.
    /// Reads the 4-byte length prefix, then returns a slice of that length.
    pub fn read_slice_u8(&mut self) -> Result<&'a [u8], DeserError> {
        let len = self.read_u32()? as usize;
        self.read_bytes(len)
    }

    /// Read an `int8[]` sequence as a borrowed slice.
    pub fn read_slice_i8(&mut self) -> Result<&'a [u8], DeserError> {
        // i8 and u8 have identical CDR encoding (1 byte, no alignment)
        self.read_slice_u8()
    }

    /// Read a `bool[]` sequence as a borrowed `&[u8]` slice.
    ///
    /// CDR encodes booleans as single bytes (0/1). The returned slice
    /// contains raw bytes; the caller interprets 0 as false, non-zero as true.
    pub fn read_slice_bool(&mut self) -> Result<&'a [u8], DeserError> {
        self.read_slice_u8()
    }

    /// Read a multi-byte numeric sequence (`float32[]`, `uint16[]`, …) as a
    /// borrowed [`LeSliceView`] — the alignment-agnostic borrowed reader for
    /// RFC-0033 `borrowed` mode (Phase 229.6, issue 0007).
    ///
    /// Unlike a `&'a [T]` cast, this never requires the source buffer to be
    /// `T`-aligned: the view borrows the raw little-endian bytes zero-copy and
    /// decodes each element on access via `from_le_bytes`. Reads the 4-byte
    /// length prefix, aligns the reader to `T` within the CDR stream, then
    /// returns a view over `len * size_of::<T>()` bytes.
    ///
    /// The alignment is taken only when there IS a first element to align:
    /// the writer pads before each primitive it writes and never for an empty
    /// sequence, so an unconditional `align` here consumed padding that was
    /// never emitted and desynced any 1-byte field (or the end of the buffer)
    /// that followed an empty `float64[]`. `len` is wire data, so the byte
    /// length is a checked product — on a 32-bit target `u32::MAX * 8` wraps
    /// to a length the bounds check accepts (issues 1148/1149, whose C
    /// counterpart `nros_cdr_borrow_le_slice_*` had both defects).
    pub fn read_le_slice<T: LeDecode>(&mut self) -> Result<LeSliceView<'a, T>, DeserError> {
        let len = self.read_u32()? as usize;
        let byte_len = len.checked_mul(T::SIZE).ok_or(DeserError::InvalidData)?;
        if len > 0 {
            self.align(T::SIZE)?;
        }
        let bytes = self.read_bytes(byte_len)?;
        Ok(LeSliceView::new(bytes))
    }
}

/// A little-endian-decodable fixed-width numeric element of a borrowed sequence.
///
/// Implemented for the multi-byte CDR numeric primitives. Single-byte types
/// (`u8`/`i8`/`bool`) do not need this — they borrow directly as `&[u8]`.
pub trait LeDecode: Sized + Copy {
    /// Encoded width in bytes (CDR little-endian).
    const SIZE: usize;
    /// Decode one element from exactly [`SIZE`](Self::SIZE) little-endian bytes.
    fn from_le(bytes: &[u8]) -> Self;
}

macro_rules! impl_le_decode {
    ($($t:ty),+ $(,)?) => {$(
        impl LeDecode for $t {
            const SIZE: usize = core::mem::size_of::<$t>();
            #[inline]
            fn from_le(bytes: &[u8]) -> Self {
                let mut buf = [0u8; core::mem::size_of::<$t>()];
                buf.copy_from_slice(bytes);
                <$t>::from_le_bytes(buf)
            }
        }
    )+};
}
impl_le_decode!(u16, i16, u32, i32, u64, i64, f32, f64);

/// A borrowed, alignment-agnostic view over a CDR little-endian numeric
/// sequence (RFC-0033 `borrowed` mode). Borrows the raw payload bytes
/// zero-copy; decodes elements lazily on access, so the source buffer need not
/// be `T`-aligned. Valid only for the borrow lifetime `'a` (the subscription
/// callback scope).
#[derive(Clone, Copy)]
pub struct LeSliceView<'a, T> {
    bytes: &'a [u8],
    _marker: core::marker::PhantomData<fn() -> T>,
}

impl<'a, T: LeDecode> LeSliceView<'a, T> {
    /// Wrap raw little-endian payload bytes. `bytes.len()` must be a multiple of
    /// `T::SIZE` (guaranteed by [`CdrReader::read_le_slice`]).
    #[inline]
    pub fn new(bytes: &'a [u8]) -> Self {
        Self {
            bytes,
            _marker: core::marker::PhantomData,
        }
    }

    /// Number of elements in the view.
    #[inline]
    pub fn len(&self) -> usize {
        self.bytes.len() / T::SIZE
    }

    /// Whether the view is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    /// The raw little-endian payload bytes (zero-copy).
    #[inline]
    pub fn as_bytes(&self) -> &'a [u8] {
        self.bytes
    }

    /// Decode the element at `index`, or `None` if out of bounds.
    #[inline]
    pub fn get(&self, index: usize) -> Option<T> {
        let start = index.checked_mul(T::SIZE)?;
        let end = start.checked_add(T::SIZE)?;
        self.bytes.get(start..end).map(T::from_le)
    }

    /// Iterate over the decoded elements.
    #[inline]
    pub fn iter(&self) -> impl Iterator<Item = T> + 'a {
        let bytes = self.bytes;
        (0..bytes.len() / T::SIZE).map(move |i| T::from_le(&bytes[i * T::SIZE..(i + 1) * T::SIZE]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── phase-303 W2/W3 — XCDR2 (DELIMITED_CDR2 + DHEADER) ──────────────────

    /// A nested-struct serialize (Header-like: `{ Time{i32 sec, u32 nanosec};
    /// string frame_id }`) wrapped in DHEADERs under XCDR2 lays out the
    /// encapsulation, the top DHEADER, the nested-Time DHEADER, and the members
    /// at the canonical offsets — and round-trips through the reader.
    /// Serialize `std_msgs/Header { builtin_interfaces/Time stamp; string
    /// frame_id }` the way the generated code does (each struct DHEADER-wrapped),
    /// with `stamp = {sec:7, nanosec:9}`, `frame_id = "ab"`.
    fn serialize_header(w: &mut CdrWriter) {
        let h = w.begin_dheader().unwrap();
        // nested Time
        let t = w.begin_dheader().unwrap();
        w.write_i32(7).unwrap();
        w.write_u32(9).unwrap();
        w.end_dheader(t).unwrap();
        w.write_string("ab").unwrap();
        w.end_dheader(h).unwrap();
    }

    /// WIRE ORACLE (phase-303 #0267). Under XCDR1 the DHEADER wrap is a no-op, so
    /// nano-ros's Header bytes are BYTE-IDENTICAL to what a real ROS 2 Jazzy node
    /// puts ON THE WIRE by default. Captured live from `ros:jazzy-ros-base`
    /// (2026-07-26) — a Header pub → a `raw=True` subscriber (the actual
    /// negotiated RTPS payload, not just `serialize_message`):
    ///   - `rmw_fastrtps_cpp` (default): exactly these 19 bytes.
    ///   - `rmw_cyclonedds_cpp`: identical content + one trailing `00`
    ///     alignment pad (20 bytes) — decodes the same.
    ///
    /// Both use encapsulation `00 01` (XCDR1), NO DHEADER: modern Jazzy STILL
    /// defaults to XCDR1 on the wire. So nano-ros (humble, or a jazzy build
    /// talking to a default peer) already interoperates byte-for-byte. XCDR2 +
    /// DHEADER only appears with a non-default negotiated `data_representation`
    /// (the #0267 domain_bridge trigger) — covered by the XCDR2 path.
    #[test]
    fn xcdr1_header_matches_live_jazzy_wire_bytes() {
        let mut buf = [0u8; 64];
        let n = {
            let mut w = CdrWriter::new_with_header(&mut buf).unwrap();
            serialize_header(&mut w);
            w.position()
        };
        // Captured from `ros:jazzy-ros-base` rclpy serialize_message(Header):
        //   00 01 00 00 | 07 00 00 00 | 09 00 00 00 | 03 00 00 00 | 61 62 00
        let jazzy: [u8; 19] = [
            0x00, 0x01, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x03, 0x00,
            0x00, 0x00, 0x61, 0x62, 0x00,
        ];
        assert_eq!(
            &buf[..n],
            &jazzy,
            "nano-ros XCDR1 Header must equal live Jazzy wire bytes"
        );
    }

    /// The XCDR2 (jazzy build) Header: appendable → a DHEADER per struct. The
    /// encapsulation flips to `00 09`, and Cyclone/a modern peer negotiating
    /// XCDR2 reads the DHEADERs. Round-trips through the reader.
    #[test]
    fn xcdr2_header_has_dheaders_and_roundtrips() {
        let mut buf = [0u8; 64];
        let n = {
            let mut w = CdrWriter::new_with_header_xcdr2(&mut buf).unwrap();
            serialize_header(&mut w);
            w.position()
        };
        assert_eq!(&buf[0..2], &[0x00, 0x09], "XCDR2 DELIMITED encapsulation");
        // Round-trip via the auto-detecting reader.
        let mut r = CdrReader::new_with_header(&buf[..n]).unwrap();
        assert_eq!(r.version(), EncodingVersion::Xcdr2);
        let h = r.begin_dheader().unwrap();
        let t = r.begin_dheader().unwrap();
        assert_eq!(r.read_i32().unwrap(), 7);
        assert_eq!(r.read_u32().unwrap(), 9);
        r.end_dheader(t).unwrap();
        assert_eq!(r.read_string().unwrap(), "ab");
        r.end_dheader(h).unwrap();
    }

    #[test]
    fn xcdr2_nested_dheader_layout_and_roundtrip() {
        let mut buf = [0u8; 64];
        {
            let mut w = CdrWriter::new_with_header_xcdr2(&mut buf).unwrap();
            let top = w.begin_dheader().unwrap();
            // nested Time { i32 sec; u32 nanosec }
            let time = w.begin_dheader().unwrap();
            w.write_i32(7).unwrap();
            w.write_u32(9).unwrap();
            w.end_dheader(time).unwrap();
            // string frame_id
            w.write_string("ab").unwrap();
            w.end_dheader(top).unwrap();
        }
        // Header (encaps): 0x00 0x09 0x00 0x00.
        assert_eq!(&buf[0..4], &[0x00, 0x09, 0x00, 0x00]);
        // pos 4: top DHEADER (u32 LE = size of everything after it).
        // pos 8: nested Time DHEADER = 8 (two u32).
        assert_eq!(u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]), 8);
        // sec @12, nanosec @16.
        assert_eq!(u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]), 7);
        assert_eq!(u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]), 9);

        // Round-trip.
        let mut r = CdrReader::new_with_header(&buf).unwrap();
        assert_eq!(r.version(), EncodingVersion::Xcdr2);
        let top = r.begin_dheader().unwrap();
        let time = r.begin_dheader().unwrap();
        assert_eq!(r.read_i32().unwrap(), 7);
        assert_eq!(r.read_u32().unwrap(), 9);
        r.end_dheader(time).unwrap();
        assert_eq!(r.read_string().unwrap(), "ab");
        r.end_dheader(top).unwrap();
    }

    /// The DHEADER calls are pure NO-OPs under XCDR1 → a struct wrapped in
    /// begin/end_dheader emits byte-identical output to one that isn't. This is
    /// what lets generated code wrap unconditionally with zero Humble impact.
    #[test]
    fn xcdr1_dheader_calls_are_byte_identical_noops() {
        let mut a = [0u8; 32];
        let mut b = [0u8; 32];
        let na = {
            let mut w = CdrWriter::new_with_header(&mut a).unwrap();
            let d = w.begin_dheader().unwrap();
            w.write_i32(-2).unwrap();
            w.write_u32(3).unwrap();
            w.end_dheader(d).unwrap();
            w.position()
        };
        let nb = {
            let mut w = CdrWriter::new_with_header(&mut b).unwrap();
            w.write_i32(-2).unwrap();
            w.write_u32(3).unwrap();
            w.position()
        };
        assert_eq!(na, nb);
        assert_eq!(a[..na], b[..nb]);
    }

    /// XCDR2 forward-compat: a reader whose type has FEWER members than the
    /// writer sent skips the trailing (unknown) bytes via the DHEADER size.
    #[test]
    fn xcdr2_reader_skips_unknown_trailing_members() {
        let mut buf = [0u8; 32];
        {
            let mut w = CdrWriter::new_with_header_xcdr2(&mut buf).unwrap();
            let d = w.begin_dheader().unwrap();
            w.write_i32(11).unwrap();
            // A future writer appended an extra u32 the reader doesn't know.
            w.write_u32(22).unwrap();
            w.end_dheader(d).unwrap();
        }
        let mut r = CdrReader::new_with_header(&buf).unwrap();
        let d = r.begin_dheader().unwrap();
        assert_eq!(r.read_i32().unwrap(), 11);
        // The reader read only its known member (i32 @ pos 12); the extra u32 is
        // unknown. end_dheader skips it → pos advances to the struct end (16).
        assert_eq!(r.position(), 12);
        r.end_dheader(d).unwrap();
        assert_eq!(
            r.position(),
            16,
            "end_dheader must skip the unknown trailing member"
        );
    }

    #[test]
    fn test_write_read_u8() {
        let mut buf = [0u8; 16];
        let mut writer = CdrWriter::new(&mut buf);
        writer.write_u8(0x42).unwrap();
        writer.write_u8(0xFF).unwrap();

        let mut reader = CdrReader::new(&buf);
        assert_eq!(reader.read_u8().unwrap(), 0x42);
        assert_eq!(reader.read_u8().unwrap(), 0xFF);
    }

    #[test]
    fn le_slice_view_decodes_unaligned_f32() {
        // The whole point of the alignment guard (Phase 229.6): a view can sit
        // at an odd byte offset and still decode correctly — no `&[f32]` cast.
        let vals = [1.5f32, -2.25, 3.0e10, 0.0];
        let mut backing = [0u8; 1 + 4 * 4];
        backing[0] = 0xAA; // shift the f32 payload to an odd (1-byte) offset.
        for (i, v) in vals.iter().enumerate() {
            backing[1 + i * 4..1 + i * 4 + 4].copy_from_slice(&v.to_le_bytes());
        }
        let view: LeSliceView<f32> = LeSliceView::new(&backing[1..]);
        assert_eq!(view.len(), 4);
        assert!(!view.is_empty());
        for (i, v) in vals.iter().enumerate() {
            assert_eq!(view.get(i).unwrap(), *v);
        }
        assert_eq!(view.get(4), None);
        let collected: heapless::Vec<f32, 4> = view.iter().collect();
        assert_eq!(&collected[..], &vals[..]);
    }

    #[test]
    fn read_le_slice_roundtrips_through_cdr() {
        // Write a `uint16[]` sequence with the CDR writer, read it back as a
        // borrowed `LeSliceView` — values + count must match.
        let vals = [10u16, 4000, 65535, 1];
        let mut buf = [0u8; 64];
        let written = {
            let mut w = CdrWriter::new_with_header(&mut buf).unwrap();
            w.write_sequence_len(vals.len()).unwrap();
            for v in &vals {
                w.write_u16(*v).unwrap();
            }
            w.position()
        };
        let mut reader = CdrReader::new_with_header(&buf[..written]).unwrap();
        let view = reader.read_le_slice::<u16>().unwrap();
        assert_eq!(view.len(), vals.len());
        for (i, v) in vals.iter().enumerate() {
            assert_eq!(view.get(i).unwrap(), *v);
        }
    }

    #[test]
    fn read_le_slice_consumes_stream_padding_before_8_byte_elements() {
        // `uint32 a; uint32 b; float64[] xs; uint32 tail` — the count lands at
        // stream offset 8, so the first element sits at 16 behind 4 bytes of
        // padding. The view must start AT the element and leave the cursor
        // where `tail` is (issue 1148's C symptom: values read as garbage and
        // every following field misparsed).
        let vals = [1111.0f64, 2222.0];
        let mut buf = [0u8; 64];
        let written = {
            let mut w = CdrWriter::new_with_header(&mut buf).unwrap();
            w.write_u32(7).unwrap();
            w.write_u32(9).unwrap();
            w.write_sequence_len(vals.len()).unwrap();
            for v in &vals {
                w.write_f64(*v).unwrap();
            }
            w.write_u32(0xCAFE).unwrap();
            w.position()
        };
        let mut r = CdrReader::new_with_header(&buf[..written]).unwrap();
        assert_eq!(r.read_u32().unwrap(), 7);
        assert_eq!(r.read_u32().unwrap(), 9);
        let view = r.read_le_slice::<f64>().unwrap();
        assert_eq!(view.len(), 2);
        assert_eq!(view.get(0), Some(1111.0));
        assert_eq!(view.get(1), Some(2222.0));
        assert_eq!(r.read_u32().unwrap(), 0xCAFE);
        assert!(r.is_empty());
    }

    #[test]
    fn read_le_slice_of_zero_elements_takes_no_padding() {
        // `float64[] xs` (empty) then `uint8 flag`. The writer emits no
        // padding for a sequence with no first element, so a reader that
        // aligns unconditionally reads `flag` four bytes late.
        let mut buf = [0u8; 32];
        let written = {
            let mut w = CdrWriter::new_with_header(&mut buf).unwrap();
            w.write_sequence_len(0).unwrap();
            w.write_u8(0xAB).unwrap();
            w.position()
        };
        let mut r = CdrReader::new_with_header(&buf[..written]).unwrap();
        let view = r.read_le_slice::<f64>().unwrap();
        assert!(view.is_empty());
        assert_eq!(r.read_u8().unwrap(), 0xAB);
        assert!(r.is_empty());
    }

    #[test]
    fn test_write_read_u32_alignment() {
        let mut buf = [0u8; 16];
        let mut writer = CdrWriter::new(&mut buf);
        writer.write_u8(0x01).unwrap(); // Position 1
        writer.write_u32(0x12345678).unwrap(); // Should align to position 4

        assert_eq!(writer.position(), 8); // 1 byte + 3 padding + 4 bytes

        let mut reader = CdrReader::new(&buf);
        assert_eq!(reader.read_u8().unwrap(), 0x01);
        assert_eq!(reader.read_u32().unwrap(), 0x12345678);
    }

    #[test]
    fn test_write_read_string() {
        let mut buf = [0u8; 32];
        let mut writer = CdrWriter::new(&mut buf);
        writer.write_string("Hello").unwrap();

        let mut reader = CdrReader::new(&buf);
        assert_eq!(reader.read_string().unwrap(), "Hello");
    }

    #[test]
    fn test_encapsulation_header() {
        let mut buf = [0u8; 32];
        let mut writer = CdrWriter::new_with_header(&mut buf).unwrap();
        writer.write_u32(42).unwrap();

        assert_eq!(&buf[0..4], &CDR_LE_HEADER);

        let mut reader = CdrReader::new_with_header(&buf).unwrap();
        assert_eq!(reader.read_u32().unwrap(), 42);
    }

    #[test]
    fn test_alignment_with_header() {
        let mut buf = [0u8; 32];
        let mut writer = CdrWriter::new_with_header(&mut buf).unwrap();
        // After header (pos=4, origin=4), write u8 then u32
        writer.write_u8(0x01).unwrap(); // pos=5
        writer.write_u32(0xDEADBEEF).unwrap(); // Should align to pos=8

        assert_eq!(writer.position(), 12); // 4 header + 1 byte + 3 padding + 4 bytes

        let mut reader = CdrReader::new_with_header(&buf).unwrap();
        assert_eq!(reader.read_u8().unwrap(), 0x01);
        assert_eq!(reader.read_u32().unwrap(), 0xDEADBEEF);
    }
}

// =============================================================================
// Ghost model validation
// =============================================================================

#[cfg(test)]
mod ghost_checks {
    use super::*;
    use nros_ghost_types::CdrGhost;

    /// Structural check: construct CdrGhost from CdrWriter private fields.
    /// If a field is renamed or retyped, this fails to compile.
    fn ghost_from_writer(w: &CdrWriter) -> CdrGhost {
        CdrGhost {
            buf_len: w.buf.len(),
            pos: w.pos,
            origin: w.origin,
        }
    }

    #[test]
    fn ghost_new_state() {
        let mut buf = [0u8; 64];
        let writer = CdrWriter::new(&mut buf);
        let ghost = ghost_from_writer(&writer);
        assert_eq!(ghost.pos, 0);
        assert_eq!(ghost.origin, 0);
        assert_eq!(ghost.buf_len, 64);
    }

    #[test]
    fn ghost_header_origin() {
        let mut buf = [0u8; 64];
        let writer = CdrWriter::new_with_header(&mut buf).unwrap();
        let ghost = ghost_from_writer(&writer);
        assert_eq!(ghost.pos, 4);
        assert_eq!(ghost.origin, 4);
    }

    #[test]
    fn ghost_position_invariant() {
        let mut buf = [0u8; 64];
        let mut writer = CdrWriter::new_with_header(&mut buf).unwrap();
        writer.write_u32(42).unwrap();
        let ghost = ghost_from_writer(&writer);
        // After header: pos + remaining == buf_len
        assert_eq!(ghost.pos + writer.remaining(), ghost.buf_len);
    }

    #[test]
    fn test_read_slice_u8() {
        let mut buf = [0u8; 64];
        let mut writer = CdrWriter::new_with_header(&mut buf).unwrap();
        // Write a uint8[] sequence: [0x10, 0x20, 0x30]
        writer.write_u32(3).unwrap(); // length
        writer.write_u8(0x10).unwrap();
        writer.write_u8(0x20).unwrap();
        writer.write_u8(0x30).unwrap();
        let len = writer.position();

        let mut reader = CdrReader::new_with_header(&buf[..len]).unwrap();
        let slice = reader.read_slice_u8().unwrap();
        assert_eq!(slice, &[0x10, 0x20, 0x30]);
    }

    #[test]
    fn test_read_slice_u8_empty() {
        let mut buf = [0u8; 64];
        let mut writer = CdrWriter::new_with_header(&mut buf).unwrap();
        writer.write_u32(0).unwrap(); // length = 0
        let len = writer.position();

        let mut reader = CdrReader::new_with_header(&buf[..len]).unwrap();
        let slice = reader.read_slice_u8().unwrap();
        assert!(slice.is_empty());
    }
}

// =============================================================================
// Kani bounded model checking proofs
// =============================================================================

#[cfg(kani)]
mod verification {
    use super::*;

    // ---- Primitive write/read panic-freedom ----

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_write_u8_no_panic() {
        let mut buf = [0u8; 8];
        let mut writer = CdrWriter::new(&mut buf);
        let val: u8 = kani::any();
        let _ = writer.write_u8(val);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_write_bool_no_panic() {
        let mut buf = [0u8; 8];
        let mut writer = CdrWriter::new(&mut buf);
        let val: bool = kani::any();
        let _ = writer.write_bool(val);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_write_i16_no_panic() {
        let mut buf = [0u8; 16];
        let mut writer = CdrWriter::new(&mut buf);
        let val: i16 = kani::any();
        let _ = writer.write_i16(val);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_write_i32_no_panic() {
        let mut buf = [0u8; 16];
        let mut writer = CdrWriter::new(&mut buf);
        let val: i32 = kani::any();
        let _ = writer.write_i32(val);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_write_i64_no_panic() {
        let mut buf = [0u8; 16];
        let mut writer = CdrWriter::new(&mut buf);
        let val: i64 = kani::any();
        let _ = writer.write_i64(val);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_write_f32_no_panic() {
        let mut buf = [0u8; 16];
        let mut writer = CdrWriter::new(&mut buf);
        let val: f32 = kani::any();
        let _ = writer.write_f32(val);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_write_f64_no_panic() {
        let mut buf = [0u8; 16];
        let mut writer = CdrWriter::new(&mut buf);
        let val: f64 = kani::any();
        let _ = writer.write_f64(val);
    }

    // ---- Round-trip correctness: write then read produces the same value ----

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_roundtrip_u8() {
        let mut buf = [0u8; 8];
        let val: u8 = kani::any();
        let len = {
            let mut writer = CdrWriter::new(&mut buf);
            writer.write_u8(val).unwrap();
            writer.position()
        };
        let mut reader = CdrReader::new(&buf[..len]);
        assert_eq!(reader.read_u8().unwrap(), val);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_roundtrip_bool() {
        let mut buf = [0u8; 8];
        let val: bool = kani::any();
        let len = {
            let mut writer = CdrWriter::new(&mut buf);
            writer.write_bool(val).unwrap();
            writer.position()
        };
        let mut reader = CdrReader::new(&buf[..len]);
        assert_eq!(reader.read_bool().unwrap(), val);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_roundtrip_i16() {
        let mut buf = [0u8; 16];
        let val: i16 = kani::any();
        let len = {
            let mut writer = CdrWriter::new(&mut buf);
            writer.write_i16(val).unwrap();
            writer.position()
        };
        let mut reader = CdrReader::new(&buf[..len]);
        assert_eq!(reader.read_i16().unwrap(), val);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_roundtrip_i32() {
        let mut buf = [0u8; 16];
        let val: i32 = kani::any();
        let len = {
            let mut writer = CdrWriter::new(&mut buf);
            writer.write_i32(val).unwrap();
            writer.position()
        };
        let mut reader = CdrReader::new(&buf[..len]);
        assert_eq!(reader.read_i32().unwrap(), val);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_roundtrip_i64() {
        let mut buf = [0u8; 16];
        let val: i64 = kani::any();
        let len = {
            let mut writer = CdrWriter::new(&mut buf);
            writer.write_i64(val).unwrap();
            writer.position()
        };
        let mut reader = CdrReader::new(&buf[..len]);
        assert_eq!(reader.read_i64().unwrap(), val);
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_roundtrip_f32() {
        let mut buf = [0u8; 16];
        let val: f32 = kani::any();
        let len = {
            let mut writer = CdrWriter::new(&mut buf);
            writer.write_f32(val).unwrap();
            writer.position()
        };
        let mut reader = CdrReader::new(&buf[..len]);
        let result = reader.read_f32().unwrap();
        assert_eq!(val.to_bits(), result.to_bits());
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_roundtrip_f64() {
        let mut buf = [0u8; 16];
        let val: f64 = kani::any();
        let len = {
            let mut writer = CdrWriter::new(&mut buf);
            writer.write_f64(val).unwrap();
            writer.position()
        };
        let mut reader = CdrReader::new(&buf[..len]);
        let result = reader.read_f64().unwrap();
        assert_eq!(val.to_bits(), result.to_bits());
    }

    // ---- CDR header round-trip ----

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_roundtrip_with_header_i32() {
        let mut buf = [0u8; 16];
        let val: i32 = kani::any();
        let len = {
            let mut writer = CdrWriter::new_with_header(&mut buf).unwrap();
            writer.write_i32(val).unwrap();
            writer.position()
        };
        let mut reader = CdrReader::new_with_header(&buf[..len]).unwrap();
        assert_eq!(reader.read_i32().unwrap(), val);
    }

    // ---- Buffer exhaustion returns Err, never panics ----

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_write_buffer_exhaustion_u32() {
        let mut buf = [0u8; 3]; // Too small for u32
        let mut writer = CdrWriter::new(&mut buf);
        let val: u32 = kani::any();
        let result = writer.write_u32(val);
        assert!(result.is_err());
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_write_header_buffer_too_small() {
        let mut buf = [0u8; 3]; // Too small for 4-byte header
        let result = CdrWriter::new_with_header(&mut buf);
        assert!(result.is_err());
    }

    // ---- Deserialization of arbitrary bytes: Ok or Err, never panic ----

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_deserialize_arbitrary_bytes_i32() {
        let mut buf = [0u8; 8];
        buf[0] = kani::any();
        buf[1] = kani::any();
        buf[2] = kani::any();
        buf[3] = kani::any();
        buf[4] = kani::any();
        buf[5] = kani::any();
        buf[6] = kani::any();
        buf[7] = kani::any();
        let result = CdrReader::new_with_header(&buf);
        if let Ok(mut reader) = result {
            let _ = reader.read_i32(); // Ok or Err, not panic
        }
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_deserialize_empty_buffer() {
        let buf = [0u8; 0];
        let mut reader = CdrReader::new(&buf);
        assert!(reader.read_u8().is_err());
        assert!(reader.read_u32().is_err());
    }

    // ---- Alignment arithmetic correctness ----

    #[kani::proof]
    fn cdr_alignment_no_overflow() {
        let offset: usize = kani::any();
        let alignment: usize = kani::any();
        kani::assume(alignment > 0 && alignment <= 8);
        kani::assume(offset <= 1024); // Realistic buffer size
        let padding = (alignment - (offset % alignment)) % alignment;
        let aligned = offset + padding;
        assert!(aligned % alignment == 0);
        assert!(aligned >= offset);
        assert!(aligned < offset + alignment);
    }

    // ---- Position tracking consistency ----

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_writer_position_monotonic() {
        let mut buf = [0u8; 32];
        let mut writer = CdrWriter::new(&mut buf);
        let pos0 = writer.position();

        let val: u8 = kani::any();
        if writer.write_u8(val).is_ok() {
            assert!(writer.position() > pos0);
        }
    }

    #[kani::proof]
    #[kani::unwind(5)]
    fn cdr_writer_remaining_consistent() {
        const BUF_LEN: usize = 32;
        let mut buf = [0u8; BUF_LEN];
        let mut writer = CdrWriter::new(&mut buf);
        assert_eq!(writer.position() + writer.remaining(), BUF_LEN);

        let val: u32 = kani::any();
        let _ = writer.write_u32(val);
        assert_eq!(writer.position() + writer.remaining(), BUF_LEN);
    }
}
