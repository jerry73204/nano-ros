//! Key expressions

use crate::{Error, Result};
use core::fmt;
use zenoh_pico_sys::*;

/// A key expression (topic path)
///
/// Key expressions are used to identify topics in zenoh. They support
/// wildcards for pattern matching.
///
/// This uses an owned key expression (`z_owned_keyexpr_t`) which copies
/// the string data, so it's safe to move the KeyExpr after creation.
pub struct KeyExpr {
    inner: z_owned_keyexpr_t,
    // Keep a copy of the string for Debug formatting
    #[cfg(feature = "std")]
    name: alloc::string::String,
    #[cfg(not(feature = "std"))]
    name: heapless::String<256>,
}

impl KeyExpr {
    /// Create a new key expression from a string
    pub fn new(s: &str) -> Result<Self> {
        // Check length limit
        if s.len() >= 256 {
            return Err(Error::InvalidKeyExpr);
        }

        // Create null-terminated string
        let mut buf = [0u8; 257];
        buf[..s.len()].copy_from_slice(s.as_bytes());

        unsafe {
            let mut keyexpr = core::mem::MaybeUninit::<z_owned_keyexpr_t>::uninit();
            let result = z_keyexpr_from_str(keyexpr.as_mut_ptr(), buf.as_ptr() as *const i8);
            if result < 0 {
                return Err(Error::InvalidKeyExpr);
            }

            #[cfg(feature = "std")]
            let name = alloc::string::String::from(s);
            #[cfg(not(feature = "std"))]
            let name = heapless::String::try_from(s).map_err(|_| Error::InvalidKeyExpr)?;

            Ok(Self {
                inner: keyexpr.assume_init(),
                name,
            })
        }
    }

    /// Get the loaned key expression pointer
    pub(crate) fn as_loaned(&self) -> *const z_loaned_keyexpr_t {
        unsafe { z_keyexpr_loan(&self.inner) }
    }
}

impl Drop for KeyExpr {
    fn drop(&mut self) {
        unsafe {
            z_keyexpr_drop(z_keyexpr_move(&mut self.inner));
        }
    }
}

impl fmt::Debug for KeyExpr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("KeyExpr").field(&self.name.as_str()).finish()
    }
}
