//! Zenoh session

use crate::{Config, Error, KeyExpr, Publisher, Query, Queryable, Result, Subscriber};
use core::ptr;
use zenoh_pico_sys::*;

/// A zenoh session
///
/// A session represents a connection to the zenoh network. It can be used
/// to declare publishers and subscribers.
pub struct Session {
    inner: z_owned_session_t,
    read_task_started: bool,
    lease_task_started: bool,
}

impl Session {
    /// Open a new session with the given configuration
    pub fn open(config: Config) -> Result<Self> {
        unsafe {
            let mut config = config.into_inner();
            let mut session = core::mem::MaybeUninit::<z_owned_session_t>::uninit();

            let result = z_open(
                session.as_mut_ptr(),
                z_config_move(&mut config),
                ptr::null(),
            );

            if result < 0 {
                return Err(Error::SessionOpenFailed);
            }

            let mut sess = Self {
                inner: session.assume_init(),
                read_task_started: false,
                lease_task_started: false,
            };

            // Start background tasks
            sess.start_tasks()?;

            Ok(sess)
        }
    }

    /// Start background read and lease tasks
    fn start_tasks(&mut self) -> Result<()> {
        unsafe {
            let session_mut = z_session_loan_mut(&mut self.inner);

            let result = zp_start_read_task(session_mut, ptr::null());
            if result < 0 {
                return Err(Error::TaskStartFailed);
            }
            self.read_task_started = true;

            let result = zp_start_lease_task(session_mut, ptr::null());
            if result < 0 {
                return Err(Error::TaskStartFailed);
            }
            self.lease_task_started = true;

            Ok(())
        }
    }

    /// Stop background tasks
    fn stop_tasks(&mut self) {
        unsafe {
            let session_mut = z_session_loan_mut(&mut self.inner);

            if self.read_task_started {
                zp_stop_read_task(session_mut);
                self.read_task_started = false;
            }

            if self.lease_task_started {
                zp_stop_lease_task(session_mut);
                self.lease_task_started = false;
            }
        }
    }

    /// Declare a publisher for the given key expression
    pub fn declare_publisher(&self, keyexpr: &KeyExpr) -> Result<Publisher> {
        Publisher::new(self, keyexpr)
    }

    /// Declare a subscriber for the given key expression
    pub fn declare_subscriber<F>(&self, keyexpr: &KeyExpr, callback: F) -> Result<Subscriber<F>>
    where
        F: FnMut(crate::Sample) + Send + 'static,
    {
        Subscriber::new(self, keyexpr, callback)
    }

    /// Declare a queryable for the given key expression
    ///
    /// Queryables receive queries and can send replies. Used for ROS 2 services.
    pub fn declare_queryable<F>(&self, keyexpr: &KeyExpr, callback: F) -> Result<Queryable<F>>
    where
        F: FnMut(Query) + Send + 'static,
    {
        Queryable::new(self, keyexpr, callback)
    }

    /// Get the loaned session pointer (internal use)
    pub(crate) fn as_loaned(&self) -> *const z_loaned_session_t {
        unsafe { z_session_loan(&self.inner) }
    }

    /// Get a pointer to the owned session (for FFI use)
    ///
    /// # Safety
    /// The returned pointer is only valid while the session is alive.
    pub fn as_owned_ptr(&self) -> *const z_owned_session_t {
        &self.inner
    }

    /// Get a loaned session pointer for FFI operations
    pub fn loan(&self) -> *const z_loaned_session_t {
        unsafe { z_session_loan(&self.inner) }
    }

    /// Get access to the inner owned session
    pub fn inner(&self) -> &z_owned_session_t {
        &self.inner
    }

    /// Close the session
    pub fn close(mut self) -> Result<()> {
        self.stop_tasks();

        unsafe {
            let session_mut = z_session_loan_mut(&mut self.inner);
            let result = z_close(session_mut, ptr::null());
            if result < 0 {
                return Err(Error::ZenohError(result));
            }
        }

        // Prevent Drop from running
        core::mem::forget(self);
        Ok(())
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        self.stop_tasks();

        unsafe {
            let session_mut = z_session_loan_mut(&mut self.inner);
            z_close(session_mut, ptr::null());
        }
    }
}

// Session is Send but not Sync (zenoh-pico is not thread-safe for same session)
unsafe impl Send for Session {}
