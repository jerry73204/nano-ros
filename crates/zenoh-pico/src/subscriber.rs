//! Subscriber

use crate::{Error, KeyExpr, Result, Session};
use alloc::boxed::Box;
use core::ptr;
use zenoh_pico_sys::*;

/// A received sample (message)
#[derive(Debug)]
pub struct Sample {
    /// The key expression this sample was received on
    pub keyexpr: alloc::string::String,
    /// The payload data
    pub payload: alloc::vec::Vec<u8>,
    /// Optional attachment data (used by rmw_zenoh for metadata)
    pub attachment: Option<alloc::vec::Vec<u8>>,
}

/// Context for subscriber callback
struct CallbackContext<F> {
    callback: F,
}

/// A subscriber for a specific key expression
///
/// Subscribers receive data from the zenoh network via a callback.
/// Note: The callback context is owned by the zenoh-pico closure and freed via subscriber_drop.
pub struct Subscriber<F> {
    inner: z_owned_subscriber_t,
    _marker: core::marker::PhantomData<F>,
}

impl<F> Subscriber<F>
where
    F: FnMut(Sample) + Send + 'static,
{
    /// Create a new subscriber
    pub(crate) fn new(session: &Session, keyexpr: &KeyExpr, callback: F) -> Result<Self> {
        // Box the context so its address is stable
        let context = Box::new(CallbackContext { callback });
        let context_ptr = Box::into_raw(context);

        unsafe {
            let mut subscriber = core::mem::MaybeUninit::<z_owned_subscriber_t>::uninit();

            // Create closure using the proper helper function
            let mut closure = core::mem::MaybeUninit::<z_owned_closure_sample_t>::uninit();
            let result = z_closure_sample(
                closure.as_mut_ptr(),
                Some(subscriber_callback::<F>),
                Some(subscriber_drop::<F>),
                context_ptr as *mut cty::c_void,
            );

            if result < 0 {
                // Clean up context on failure
                let _ = Box::from_raw(context_ptr);
                return Err(Error::SubscriberDeclarationFailed);
            }
            let mut closure = closure.assume_init();

            let result = z_declare_subscriber(
                session.as_loaned(),
                subscriber.as_mut_ptr(),
                keyexpr.as_loaned(),
                z_closure_sample_move(&mut closure),
                ptr::null(),
            );

            if result < 0 {
                // Note: zenoh-pico already called subscriber_drop via z_closure_sample_move,
                // which freed the context. Do NOT free it again here.
                return Err(Error::SubscriberDeclarationFailed);
            }

            // Note: we do NOT take ownership of context_ptr here.
            // The zenoh-pico closure owns it and will free it via subscriber_drop.
            Ok(Self {
                inner: subscriber.assume_init(),
                _marker: core::marker::PhantomData,
            })
        }
    }
}

impl<F> Drop for Subscriber<F> {
    fn drop(&mut self) {
        unsafe {
            z_undeclare_subscriber(z_subscriber_move(&mut self.inner));
        }
    }
}

// Subscriber is Send but not Sync
unsafe impl<F: Send> Send for Subscriber<F> {}

/// C callback for subscriber
unsafe extern "C" fn subscriber_callback<F>(sample: *mut _z_sample_t, context: *mut cty::c_void)
where
    F: FnMut(Sample) + Send + 'static,
{
    if sample.is_null() || context.is_null() {
        return;
    }

    let ctx = &mut *(context as *mut CallbackContext<F>);
    let sample_ref = &*sample;

    // Get payload
    let payload = &sample_ref.payload;
    let len = z_bytes_len(payload as *const _z_bytes_t as *const z_loaned_bytes_t);

    // Read payload into a Vec
    let mut data = alloc::vec![0u8; len];
    let mut reader = z_bytes_get_reader(payload as *const _z_bytes_t as *const z_loaned_bytes_t);
    z_bytes_reader_read(&mut reader, data.as_mut_ptr(), len);

    // Get attachment if present
    let attachment_bytes = &sample_ref.attachment;
    let attachment_len =
        z_bytes_len(attachment_bytes as *const _z_bytes_t as *const z_loaned_bytes_t);

    let attachment = if attachment_len > 0 {
        let mut attachment_data = alloc::vec![0u8; attachment_len];
        let mut attachment_reader =
            z_bytes_get_reader(attachment_bytes as *const _z_bytes_t as *const z_loaned_bytes_t);
        z_bytes_reader_read(
            &mut attachment_reader,
            attachment_data.as_mut_ptr(),
            attachment_len,
        );
        Some(attachment_data)
    } else {
        None
    };

    // Extract key expression from sample
    let keyexpr_slice = &sample_ref.keyexpr._suffix._slice;
    let keyexpr_str = if keyexpr_slice.len > 0 && !keyexpr_slice.start.is_null() {
        let bytes = core::slice::from_raw_parts(keyexpr_slice.start, keyexpr_slice.len);
        alloc::string::String::from_utf8_lossy(bytes).into_owned()
    } else {
        alloc::string::String::new()
    };

    // Call the user callback
    let sample = Sample {
        keyexpr: keyexpr_str,
        payload: data,
        attachment,
    };
    (ctx.callback)(sample);
}

/// C callback for dropping the context
unsafe extern "C" fn subscriber_drop<F>(context: *mut cty::c_void) {
    if !context.is_null() {
        // Re-take ownership and drop
        let _ = Box::from_raw(context as *mut CallbackContext<F>);
    }
}
