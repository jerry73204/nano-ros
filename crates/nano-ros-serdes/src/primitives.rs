//! Primitive type serialization implementations

use crate::cdr::{CdrReader, CdrWriter};
use crate::error::{DeserError, SerError};
use crate::traits::{Deserialize, Serialize};

// === Boolean ===

impl Serialize for bool {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_u8(if *self { 1 } else { 0 })
    }
}

impl Deserialize for bool {
    #[inline]
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(reader.read_u8()? != 0)
    }
}

// === 8-bit integers ===

impl Serialize for u8 {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_u8(*self)
    }
}

impl Deserialize for u8 {
    #[inline]
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        reader.read_u8()
    }
}

impl Serialize for i8 {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_u8(*self as u8)
    }
}

impl Deserialize for i8 {
    #[inline]
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(reader.read_u8()? as i8)
    }
}

// === 16-bit integers ===

impl Serialize for u16 {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_u16(*self)
    }
}

impl Deserialize for u16 {
    #[inline]
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        reader.read_u16()
    }
}

impl Serialize for i16 {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i16(*self)
    }
}

impl Deserialize for i16 {
    #[inline]
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        reader.read_i16()
    }
}

// === 32-bit integers ===

impl Serialize for u32 {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_u32(*self)
    }
}

impl Deserialize for u32 {
    #[inline]
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        reader.read_u32()
    }
}

impl Serialize for i32 {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i32(*self)
    }
}

impl Deserialize for i32 {
    #[inline]
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        reader.read_i32()
    }
}

// === 64-bit integers ===

impl Serialize for u64 {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_u64(*self)
    }
}

impl Deserialize for u64 {
    #[inline]
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        reader.read_u64()
    }
}

impl Serialize for i64 {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i64(*self)
    }
}

impl Deserialize for i64 {
    #[inline]
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        reader.read_i64()
    }
}

// === Floating point ===

impl Serialize for f32 {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_f32(*self)
    }
}

impl Deserialize for f32 {
    #[inline]
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        reader.read_f32()
    }
}

impl Serialize for f64 {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_f64(*self)
    }
}

impl Deserialize for f64 {
    #[inline]
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        reader.read_f64()
    }
}

// === Character (CDR char is 1 byte) ===

impl Serialize for char {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        // CDR char is 1 byte, truncate to ASCII
        writer.write_u8(*self as u8)
    }
}

impl Deserialize for char {
    #[inline]
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(reader.read_u8()? as char)
    }
}

// === String slice (serialization only) ===

impl Serialize for str {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_string(self)
    }
}

impl Serialize for &str {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_string(self)
    }
}

// === Fixed-size arrays ===

impl<T: Serialize, const N: usize> Serialize for [T; N] {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        for item in self.iter() {
            item.serialize(writer)?;
        }
        Ok(())
    }
}

impl<T: Deserialize + Default + Copy, const N: usize> Deserialize for [T; N] {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        let mut arr = [T::default(); N];
        for item in arr.iter_mut() {
            *item = T::deserialize(reader)?;
        }
        Ok(arr)
    }
}

// === heapless::String ===

impl<const N: usize> Serialize for heapless::String<N> {
    #[inline]
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_string(self.as_str())
    }
}

impl<const N: usize> Deserialize for heapless::String<N> {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        let s = reader.read_string()?;
        heapless::String::try_from(s).map_err(|_| DeserError::CapacityExceeded)
    }
}

// === heapless::Vec ===

impl<T: Serialize, const N: usize> Serialize for heapless::Vec<T, N> {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_sequence_len(self.len())?;
        for item in self.iter() {
            item.serialize(writer)?;
        }
        Ok(())
    }
}

impl<T: Deserialize, const N: usize> Deserialize for heapless::Vec<T, N> {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        let len = reader.read_sequence_len()?;
        if len > N {
            return Err(DeserError::CapacityExceeded);
        }
        let mut vec = heapless::Vec::new();
        for _ in 0..len {
            vec.push(T::deserialize(reader)?)
                .map_err(|_| DeserError::CapacityExceeded)?;
        }
        Ok(vec)
    }
}

// === alloc types (behind feature flag) ===

#[cfg(feature = "alloc")]
mod alloc_impl {
    use super::*;
    use alloc::string::String;
    use alloc::vec::Vec;

    impl Serialize for String {
        #[inline]
        fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
            writer.write_string(self.as_str())
        }
    }

    impl Deserialize for String {
        fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
            let s = reader.read_string()?;
            Ok(String::from(s))
        }
    }

    impl<T: Serialize> Serialize for Vec<T> {
        fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
            writer.write_sequence_len(self.len())?;
            for item in self.iter() {
                item.serialize(writer)?;
            }
            Ok(())
        }
    }

    impl<T: Deserialize> Deserialize for Vec<T> {
        fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
            let len = reader.read_sequence_len()?;
            let mut vec = Vec::with_capacity(len);
            for _ in 0..len {
                vec.push(T::deserialize(reader)?);
            }
            Ok(vec)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bool_roundtrip() {
        let mut buf = [0u8; 16];
        let mut writer = CdrWriter::new(&mut buf);
        true.serialize(&mut writer).unwrap();
        false.serialize(&mut writer).unwrap();

        let mut reader = CdrReader::new(&buf);
        assert!(bool::deserialize(&mut reader).unwrap());
        assert!(!bool::deserialize(&mut reader).unwrap());
    }

    #[test]
    fn test_integers_roundtrip() {
        let mut buf = [0u8; 64];
        let mut writer = CdrWriter::new(&mut buf);

        42u8.serialize(&mut writer).unwrap();
        (-42i8).serialize(&mut writer).unwrap();
        1000u16.serialize(&mut writer).unwrap();
        (-1000i16).serialize(&mut writer).unwrap();
        100000u32.serialize(&mut writer).unwrap();
        (-100000i32).serialize(&mut writer).unwrap();
        10000000000u64.serialize(&mut writer).unwrap();
        (-10000000000i64).serialize(&mut writer).unwrap();

        let mut reader = CdrReader::new(&buf);
        assert_eq!(u8::deserialize(&mut reader).unwrap(), 42);
        assert_eq!(i8::deserialize(&mut reader).unwrap(), -42);
        assert_eq!(u16::deserialize(&mut reader).unwrap(), 1000);
        assert_eq!(i16::deserialize(&mut reader).unwrap(), -1000);
        assert_eq!(u32::deserialize(&mut reader).unwrap(), 100000);
        assert_eq!(i32::deserialize(&mut reader).unwrap(), -100000);
        assert_eq!(u64::deserialize(&mut reader).unwrap(), 10000000000);
        assert_eq!(i64::deserialize(&mut reader).unwrap(), -10000000000);
    }

    #[test]
    fn test_floats_roundtrip() {
        let mut buf = [0u8; 32];
        let mut writer = CdrWriter::new(&mut buf);

        3.14f32.serialize(&mut writer).unwrap();
        2.71828f64.serialize(&mut writer).unwrap();

        let mut reader = CdrReader::new(&buf);
        assert!((f32::deserialize(&mut reader).unwrap() - 3.14).abs() < 0.001);
        assert!((f64::deserialize(&mut reader).unwrap() - 2.71828).abs() < 0.00001);
    }

    #[test]
    fn test_heapless_string_roundtrip() {
        let mut buf = [0u8; 32];
        let mut writer = CdrWriter::new(&mut buf);

        let s: heapless::String<32> = heapless::String::try_from("Hello").unwrap();
        s.serialize(&mut writer).unwrap();

        let mut reader = CdrReader::new(&buf);
        let result: heapless::String<32> = Deserialize::deserialize(&mut reader).unwrap();
        assert_eq!(result.as_str(), "Hello");
    }

    #[test]
    fn test_heapless_vec_roundtrip() {
        let mut buf = [0u8; 64];
        let mut writer = CdrWriter::new(&mut buf);

        let mut v: heapless::Vec<u32, 8> = heapless::Vec::new();
        v.push(1).unwrap();
        v.push(2).unwrap();
        v.push(3).unwrap();
        v.serialize(&mut writer).unwrap();

        let mut reader = CdrReader::new(&buf);
        let result: heapless::Vec<u32, 8> = Deserialize::deserialize(&mut reader).unwrap();
        assert_eq!(result.as_slice(), &[1, 2, 3]);
    }

    #[test]
    fn test_array_roundtrip() {
        let mut buf = [0u8; 32];
        let mut writer = CdrWriter::new(&mut buf);

        let arr: [u32; 3] = [10, 20, 30];
        arr.serialize(&mut writer).unwrap();

        let mut reader = CdrReader::new(&buf);
        let result: [u32; 3] = Deserialize::deserialize(&mut reader).unwrap();
        assert_eq!(result, [10, 20, 30]);
    }
}
