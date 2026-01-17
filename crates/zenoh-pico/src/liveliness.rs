//! Liveliness token support for ROS 2 discovery

use crate::{Error, KeyExpr, Result, Session};
use zenoh_pico_sys::*;

/// A liveliness token that advertises the existence of an entity
///
/// When a liveliness token is declared, subscribers on intersecting key expressions
/// will receive a PUT sample when connectivity is achieved, and a DELETE sample
/// if it's lost.
///
/// Liveliness tokens are automatically undeclared when dropped.
pub struct LivelinessToken {
    inner: z_owned_liveliness_token_t,
}

impl LivelinessToken {
    /// Create a new liveliness token (internal use)
    pub(crate) fn new(inner: z_owned_liveliness_token_t) -> Self {
        Self { inner }
    }

    /// Check if the token is valid
    pub fn is_valid(&self) -> bool {
        unsafe { z_internal_liveliness_token_check(&self.inner) }
    }
}

impl Drop for LivelinessToken {
    fn drop(&mut self) {
        unsafe {
            // Undeclare the token
            z_liveliness_undeclare_token(z_liveliness_token_move(&mut self.inner));
        }
    }
}

// LivelinessToken is Send but not Sync
unsafe impl Send for LivelinessToken {}

/// A 16-byte Zenoh session ID
#[derive(Clone, Copy, Debug)]
pub struct ZenohId {
    pub id: [u8; 16],
}

impl ZenohId {
    /// Create a new ZenohId from raw bytes
    pub fn from_bytes(bytes: [u8; 16]) -> Self {
        Self { id: bytes }
    }

    /// Format the ID as a hexadecimal string (for liveliness key expressions)
    pub fn to_hex_string(&self) -> alloc::string::String {
        use alloc::format;
        format!(
            "{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
            self.id[0], self.id[1], self.id[2], self.id[3],
            self.id[4], self.id[5], self.id[6], self.id[7],
            self.id[8], self.id[9], self.id[10], self.id[11],
            self.id[12], self.id[13], self.id[14], self.id[15]
        )
    }
}

impl Session {
    /// Get the session's Zenoh ID
    ///
    /// The Zenoh ID uniquely identifies this session in the Zenoh network.
    /// It is used in liveliness token key expressions for ROS 2 discovery.
    pub fn zid(&self) -> ZenohId {
        unsafe {
            let zid = z_info_zid(self.as_loaned());
            ZenohId::from_bytes(zid.id)
        }
    }

    /// Declare a liveliness token
    ///
    /// This advertises the existence of an entity at the given key expression.
    /// Other nodes subscribing to liveliness can discover this entity.
    pub fn declare_liveliness(&self, keyexpr: &KeyExpr) -> Result<LivelinessToken> {
        unsafe {
            let mut token = core::mem::MaybeUninit::<z_owned_liveliness_token_t>::uninit();

            let result = z_liveliness_declare_token(
                self.as_loaned(),
                token.as_mut_ptr(),
                keyexpr.as_loaned(),
                core::ptr::null(),
            );

            if result < 0 {
                return Err(Error::ZenohError(result));
            }

            Ok(LivelinessToken::new(token.assume_init()))
        }
    }
}
