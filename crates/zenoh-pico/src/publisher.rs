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

    /// Publish data without attachment
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

    /// Publish data with an attachment
    ///
    /// The attachment is sent alongside the payload and can be used for metadata
    /// like RMW sequence numbers, timestamps, and GIDs required by rmw_zenoh.
    pub fn put_with_attachment(&self, data: &[u8], attachment: &[u8]) -> Result<()> {
        unsafe {
            // Create bytes from payload data
            let mut payload_bytes = core::mem::MaybeUninit::<z_owned_bytes_t>::uninit();
            let result =
                z_bytes_copy_from_buf(payload_bytes.as_mut_ptr(), data.as_ptr(), data.len());
            if result < 0 {
                return Err(Error::PublishFailed);
            }
            let mut payload_bytes = payload_bytes.assume_init();

            // Create bytes from attachment data
            let mut attachment_bytes = core::mem::MaybeUninit::<z_owned_bytes_t>::uninit();
            let result = z_bytes_copy_from_buf(
                attachment_bytes.as_mut_ptr(),
                attachment.as_ptr(),
                attachment.len(),
            );
            if result < 0 {
                // Clean up payload bytes
                z_bytes_drop(z_bytes_move(&mut payload_bytes));
                return Err(Error::PublishFailed);
            }
            let mut attachment_bytes = attachment_bytes.assume_init();

            // Initialize options with default values
            let mut options = core::mem::MaybeUninit::<z_publisher_put_options_t>::uninit();
            z_publisher_put_options_default(options.as_mut_ptr());
            let mut options = options.assume_init();

            // Set the attachment
            options.attachment = z_bytes_move(&mut attachment_bytes);

            // Publish with options
            let result = z_publisher_put(
                z_publisher_loan(&self.inner),
                z_bytes_move(&mut payload_bytes),
                &options,
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
