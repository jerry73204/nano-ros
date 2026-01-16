//! CDR encoder/decoder with alignment handling

use crate::error::{DeserError, SerError};
use crate::CDR_LE_HEADER;

/// CDR writer for serialization
///
/// Handles alignment and endianness for CDR encoding.
pub struct CdrWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
    origin: usize,
}

impl<'a> CdrWriter<'a> {
    /// Create a new CDR writer
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self {
            buf,
            pos: 0,
            origin: 0,
        }
    }

    /// Create a new CDR writer with encapsulation header
    ///
    /// Writes the 4-byte CDR header and sets origin for alignment calculations.
    pub fn new_with_header(buf: &'a mut [u8]) -> Result<Self, SerError> {
        if buf.len() < 4 {
            return Err(SerError::BufferTooSmall);
        }
        buf[0..4].copy_from_slice(&CDR_LE_HEADER);
        Ok(Self {
            buf,
            pos: 4,
            origin: 4,
        })
    }

    /// Get current position in buffer
    #[inline]
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Get remaining capacity
    #[inline]
    pub fn remaining(&self) -> usize {
        self.buf.len().saturating_sub(self.pos)
    }

    /// Get the written bytes
    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.pos]
    }

    /// Align to the given boundary (relative to origin)
    #[inline]
    pub fn align(&mut self, alignment: usize) -> Result<(), SerError> {
        let offset = self.pos - self.origin;
        let padding = (alignment - (offset % alignment)) % alignment;
        if self.remaining() < padding {
            return Err(SerError::BufferTooSmall);
        }
        // Fill padding with zeros
        for i in 0..padding {
            self.buf[self.pos + i] = 0;
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
        self.buf[self.pos] = value;
        self.pos += 1;
        Ok(())
    }

    /// Write bytes without alignment
    #[inline]
    pub fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), SerError> {
        if self.remaining() < bytes.len() {
            return Err(SerError::BufferTooSmall);
        }
        self.buf[self.pos..self.pos + bytes.len()].copy_from_slice(bytes);
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
        self.buf[self.pos..self.pos + 2].copy_from_slice(&value.to_le_bytes());
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
        self.buf[self.pos..self.pos + 4].copy_from_slice(&value.to_le_bytes());
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
        self.buf[self.pos..self.pos + 8].copy_from_slice(&value.to_le_bytes());
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
}

impl<'a> CdrReader<'a> {
    /// Create a new CDR reader
    pub fn new(buf: &'a [u8]) -> Self {
        Self {
            buf,
            pos: 0,
            origin: 0,
        }
    }

    /// Create a new CDR reader, parsing and validating the encapsulation header
    ///
    /// Expects a 4-byte CDR header at the start of the buffer.
    pub fn new_with_header(buf: &'a [u8]) -> Result<Self, DeserError> {
        if buf.len() < 4 {
            return Err(DeserError::UnexpectedEof);
        }
        // Check for valid CDR header (we only support little-endian for now)
        if buf[0] != 0x00 || (buf[1] != 0x00 && buf[1] != 0x01) {
            return Err(DeserError::InvalidHeader);
        }
        Ok(Self {
            buf,
            pos: 4,
            origin: 4,
        })
    }

    /// Get current position in buffer
    #[inline]
    pub fn position(&self) -> usize {
        self.pos
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

    /// Align to the given boundary (relative to origin)
    #[inline]
    pub fn align(&mut self, alignment: usize) -> Result<(), DeserError> {
        let offset = self.pos - self.origin;
        let padding = (alignment - (offset % alignment)) % alignment;
        if self.remaining() < padding {
            return Err(DeserError::UnexpectedEof);
        }
        self.pos += padding;
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
}

#[cfg(test)]
mod tests {
    use super::*;

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
