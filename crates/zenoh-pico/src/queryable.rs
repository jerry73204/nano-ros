//! Queryable (for ROS 2 services)
//!
//! A queryable receives queries and sends replies. This is used to implement
//! ROS 2 service servers.

use crate::{Error, KeyExpr, Result, Session};
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::ptr;
use zenoh_pico_sys::*;

/// A received query (service request)
pub struct Query {
    /// The key expression this query was received on
    pub keyexpr: alloc::string::String,
    /// The query payload (request data)
    pub payload: Vec<u8>,
    /// Optional attachment data
    pub attachment: Option<Vec<u8>>,
    /// Internal query handle for replying
    query_ptr: *const z_loaned_query_t,
}

impl Query {
    /// Reply to this query with the given payload
    ///
    /// # Safety
    /// The query pointer must still be valid (within the callback scope).
    pub fn reply(&self, keyexpr: &KeyExpr, payload: &[u8]) -> Result<()> {
        self.reply_with_attachment(keyexpr, payload, None)
    }

    /// Reply to this query with payload and optional attachment
    pub fn reply_with_attachment(
        &self,
        keyexpr: &KeyExpr,
        payload: &[u8],
        attachment: Option<&[u8]>,
    ) -> Result<()> {
        unsafe {
            // Create reply options
            let mut options = core::mem::MaybeUninit::<z_query_reply_options_t>::uninit();
            z_query_reply_options_default(options.as_mut_ptr());
            let mut options = options.assume_init();

            // Create payload bytes
            let mut reply_payload = core::mem::MaybeUninit::<z_owned_bytes_t>::uninit();
            z_bytes_copy_from_buf(reply_payload.as_mut_ptr(), payload.as_ptr(), payload.len());
            let mut reply_payload = reply_payload.assume_init();

            // Handle attachment if provided
            let mut attachment_bytes = core::mem::MaybeUninit::<z_owned_bytes_t>::uninit();
            if let Some(att_data) = attachment {
                z_bytes_copy_from_buf(
                    attachment_bytes.as_mut_ptr(),
                    att_data.as_ptr(),
                    att_data.len(),
                );
                let attachment_bytes = attachment_bytes.assume_init();
                options.attachment = z_bytes_move(&mut { attachment_bytes } as *mut _);
            }

            // Send reply
            let result = z_query_reply(
                self.query_ptr,
                keyexpr.as_loaned(),
                z_bytes_move(&mut reply_payload),
                &options,
            );

            if result < 0 {
                return Err(Error::QueryReplyFailed);
            }

            Ok(())
        }
    }

    /// Reply with an error
    pub fn reply_err(&self, error_payload: &[u8]) -> Result<()> {
        unsafe {
            let mut options = core::mem::MaybeUninit::<z_query_reply_err_options_t>::uninit();
            z_query_reply_err_options_default(options.as_mut_ptr());
            let options = options.assume_init();

            let mut err_payload = core::mem::MaybeUninit::<z_owned_bytes_t>::uninit();
            z_bytes_copy_from_buf(
                err_payload.as_mut_ptr(),
                error_payload.as_ptr(),
                error_payload.len(),
            );
            let mut err_payload = err_payload.assume_init();

            let result =
                z_query_reply_err(self.query_ptr, z_bytes_move(&mut err_payload), &options);

            if result < 0 {
                return Err(Error::QueryReplyFailed);
            }

            Ok(())
        }
    }
}

/// Context for queryable callback
struct QueryableContext<F> {
    callback: F,
}

/// A queryable for a specific key expression
///
/// Queryables receive queries (requests) and can send replies (responses).
/// This is used to implement ROS 2 service servers.
pub struct Queryable<F> {
    inner: z_owned_queryable_t,
    _marker: core::marker::PhantomData<F>,
}

impl<F> Queryable<F>
where
    F: FnMut(Query) + Send + 'static,
{
    /// Create a new queryable
    pub(crate) fn new(session: &Session, keyexpr: &KeyExpr, callback: F) -> Result<Self> {
        // Box the context so its address is stable
        let context = Box::new(QueryableContext { callback });
        let context_ptr = Box::into_raw(context);

        unsafe {
            let mut queryable = core::mem::MaybeUninit::<z_owned_queryable_t>::uninit();

            // Create closure
            let mut closure = core::mem::MaybeUninit::<z_owned_closure_query_t>::uninit();
            let result = z_closure_query(
                closure.as_mut_ptr(),
                Some(queryable_callback::<F>),
                Some(queryable_drop::<F>),
                context_ptr as *mut cty::c_void,
            );

            if result < 0 {
                // Clean up context on failure
                let _ = Box::from_raw(context_ptr);
                return Err(Error::QueryableDeclarationFailed);
            }
            let mut closure = closure.assume_init();

            // Declare queryable
            let result = z_declare_queryable(
                session.as_loaned(),
                queryable.as_mut_ptr(),
                keyexpr.as_loaned(),
                z_closure_query_move(&mut closure),
                ptr::null(),
            );

            if result < 0 {
                return Err(Error::QueryableDeclarationFailed);
            }

            Ok(Self {
                inner: queryable.assume_init(),
                _marker: core::marker::PhantomData,
            })
        }
    }
}

impl<F> Drop for Queryable<F> {
    fn drop(&mut self) {
        unsafe {
            z_undeclare_queryable(z_queryable_move(&mut self.inner));
        }
    }
}

// Queryable is Send but not Sync
unsafe impl<F: Send> Send for Queryable<F> {}

/// C callback for queryable
unsafe extern "C" fn queryable_callback<F>(query: *mut z_loaned_query_t, context: *mut cty::c_void)
where
    F: FnMut(Query) + Send + 'static,
{
    if query.is_null() || context.is_null() {
        return;
    }

    let ctx = &mut *(context as *mut QueryableContext<F>);

    // Get keyexpr from query
    let query_keyexpr = z_query_keyexpr(query);
    let keyexpr_str = if !query_keyexpr.is_null() {
        let mut view = core::mem::MaybeUninit::<z_view_string_t>::uninit();
        z_keyexpr_as_view_string(query_keyexpr, view.as_mut_ptr());
        let view = view.assume_init();
        let loaned = z_view_string_loan(&view);
        let data = z_string_data(loaned);
        let len = z_string_len(loaned);
        if !data.is_null() && len > 0 {
            let bytes = core::slice::from_raw_parts(data as *const u8, len);
            alloc::string::String::from_utf8_lossy(bytes).into_owned()
        } else {
            alloc::string::String::new()
        }
    } else {
        alloc::string::String::new()
    };

    // Get payload
    let payload_ptr = z_query_payload(query);
    let payload = if !payload_ptr.is_null() {
        let len = z_bytes_len(payload_ptr);
        if len > 0 {
            let mut data = alloc::vec![0u8; len];
            let mut reader = z_bytes_get_reader(payload_ptr);
            z_bytes_reader_read(&mut reader, data.as_mut_ptr(), len);
            data
        } else {
            Vec::new()
        }
    } else {
        Vec::new()
    };

    // Get attachment if present
    let attachment_ptr = z_query_attachment(query);
    let attachment = if !attachment_ptr.is_null() {
        let len = z_bytes_len(attachment_ptr);
        if len > 0 {
            let mut data = alloc::vec![0u8; len];
            let mut reader = z_bytes_get_reader(attachment_ptr);
            z_bytes_reader_read(&mut reader, data.as_mut_ptr(), len);
            Some(data)
        } else {
            None
        }
    } else {
        None
    };

    // Create Query object
    let query_obj = Query {
        keyexpr: keyexpr_str,
        payload,
        attachment,
        query_ptr: query,
    };

    // Call user callback
    (ctx.callback)(query_obj);
}

/// C callback for dropping the context
unsafe extern "C" fn queryable_drop<F>(context: *mut cty::c_void) {
    if !context.is_null() {
        let _ = Box::from_raw(context as *mut QueryableContext<F>);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    #[test]
    fn test_query_struct() {
        let query = Query {
            keyexpr: "test/service".into(),
            payload: vec![1, 2, 3],
            attachment: None,
            query_ptr: ptr::null(),
        };
        assert_eq!(query.keyexpr, "test/service");
        assert_eq!(query.payload, vec![1, 2, 3]);
    }
}
