//! Publisher

use crate::{Error, KeyExpr, Result, Session};
use core::ptr;
use zenoh_pico_sys::*;

/// A publisher for a specific key expression
///
/// Publishers are used to send data to the zenoh network.
pub struct Publisher {
    inner: z_owned_publisher_t,
}

impl Publisher {
    /// Create a new publisher
    pub(crate) fn new(session: &Session, keyexpr: &KeyExpr) -> Result<Self> {
        unsafe {
            let mut publisher = core::mem::MaybeUninit::<z_owned_publisher_t>::uninit();

            let result = z_declare_publisher(
                session.as_loaned(),
                publisher.as_mut_ptr(),
                keyexpr.as_loaned(),
                ptr::null(),
            );

            if result < 0 {
                return Err(Error::PublisherDeclarationFailed);
            }

            Ok(Self {
                inner: publisher.assume_init(),
            })
        }
    }

    /// Publish data
    pub fn put(&self, data: &[u8]) -> Result<()> {
        unsafe {
            // Create bytes from data
            let mut bytes = core::mem::MaybeUninit::<z_owned_bytes_t>::uninit();
            let result = z_bytes_copy_from_buf(bytes.as_mut_ptr(), data.as_ptr(), data.len());
            if result < 0 {
                return Err(Error::PublishFailed);
            }
            let mut bytes = bytes.assume_init();

            // Publish
            let result = z_publisher_put(
                z_publisher_loan(&self.inner),
                z_bytes_move(&mut bytes),
                ptr::null(),
            );

            if result < 0 {
                return Err(Error::PublishFailed);
            }

            Ok(())
        }
    }
}

impl Drop for Publisher {
    fn drop(&mut self) {
        unsafe {
            z_undeclare_publisher(z_publisher_move(&mut self.inner));
        }
    }
}

// Publisher is Send but not Sync
unsafe impl Send for Publisher {}
