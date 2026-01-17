//! Zenoh serialization support
//!
//! Provides serialization functions compatible with zenoh-cpp's `zenoh::ext::Serializer`.

use crate::{Error, Result};
use zenoh_pico_sys::*;

/// RMW GID size for attachment serialization
/// Note: This is 16 bytes for humble, even though RMW_GID_STORAGE_SIZE = 24
pub const RMW_GID_SIZE: usize = 16;

/// Serialize an RMW attachment in the format expected by rmw_zenoh_cpp (humble).
///
/// The format is:
/// - int64: sequence_number (little-endian, 8 bytes)
/// - int64: timestamp (little-endian, 8 bytes)
/// - sequence length (VLE for 16)
/// - 16 x uint8: GID
///
/// This uses zenoh's serializer to ensure compatibility with zenoh-cpp's deserializer.
pub fn serialize_rmw_attachment(
    sequence_number: i64,
    timestamp: i64,
    gid: &[u8; RMW_GID_SIZE],
) -> Result<SerializedBytes> {
    unsafe {
        // Create a serializer
        let mut serializer = core::mem::MaybeUninit::<ze_owned_serializer_t>::uninit();
        let result = ze_serializer_empty(serializer.as_mut_ptr());
        if result < 0 {
            return Err(Error::SerializationFailed);
        }
        let mut serializer = serializer.assume_init();

        // Get a mutable pointer to the internal writer
        let writer = &mut (*ze_serializer_loan_mut(&mut serializer))._writer;

        // Serialize sequence_number as int64 (little-endian)
        let seq_bytes = sequence_number.to_le_bytes();
        let result = z_bytes_writer_write_all(writer, seq_bytes.as_ptr(), seq_bytes.len());
        if result < 0 {
            ze_serializer_drop(ze_serializer_move(&mut serializer));
            return Err(Error::SerializationFailed);
        }

        // Serialize timestamp as int64 (little-endian)
        let ts_bytes = timestamp.to_le_bytes();
        let result = z_bytes_writer_write_all(writer, ts_bytes.as_ptr(), ts_bytes.len());
        if result < 0 {
            ze_serializer_drop(ze_serializer_move(&mut serializer));
            return Err(Error::SerializationFailed);
        }

        // Serialize GID as sequence of 16 uint8
        // First write sequence length using the proper VLE function
        let result = ze_serializer_serialize_sequence_length(
            ze_serializer_loan_mut(&mut serializer),
            RMW_GID_SIZE,
        );
        if result < 0 {
            ze_serializer_drop(ze_serializer_move(&mut serializer));
            return Err(Error::SerializationFailed);
        }

        // Then write the GID bytes directly (uint8 values don't need special encoding)
        // Need to get writer again after the above call
        let writer = &mut (*ze_serializer_loan_mut(&mut serializer))._writer;
        let result = z_bytes_writer_write_all(writer, gid.as_ptr(), gid.len());
        if result < 0 {
            ze_serializer_drop(ze_serializer_move(&mut serializer));
            return Err(Error::SerializationFailed);
        }

        // Finish serialization and get bytes
        let mut bytes = core::mem::MaybeUninit::<z_owned_bytes_t>::uninit();
        ze_serializer_finish(ze_serializer_move(&mut serializer), bytes.as_mut_ptr());
        let bytes = bytes.assume_init();

        Ok(SerializedBytes { inner: bytes })
    }
}

/// Owned serialized bytes
pub struct SerializedBytes {
    inner: z_owned_bytes_t,
}

impl SerializedBytes {
    /// Get the inner bytes for use with zenoh functions
    pub(crate) fn into_inner(mut self) -> z_owned_bytes_t {
        let bytes = core::mem::replace(&mut self.inner, unsafe { core::mem::zeroed() });
        core::mem::forget(self); // Don't run Drop
        bytes
    }
}

impl Drop for SerializedBytes {
    fn drop(&mut self) {
        unsafe {
            z_bytes_drop(z_bytes_move(&mut self.inner));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_serialize_attachment() {
        let gid = [1u8, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
        let result = serialize_rmw_attachment(42, 1000000, &gid);
        assert!(result.is_ok());
    }
}
