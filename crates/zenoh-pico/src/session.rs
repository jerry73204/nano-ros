//! Zenoh session
//!
//! # Executor Support
//!
//! By default, the session starts background threads for read and lease tasks.
//! For RTIC or other single-threaded executors, use `open_without_tasks()` and
//! manually call `poll_read()` and `send_keepalive()` from your executor tasks.
//!
//! ## Example: RTIC Usage
//!
//! ```ignore
//! // In init:
//! let session = Session::open_without_tasks(config)?;
//!
//! // In RTIC tasks:
//! #[task(priority = 1, shared = [session])]
//! async fn zenoh_poll(mut cx: zenoh_poll::Context) {
//!     loop {
//!         cx.shared.session.lock(|s| s.poll_read());
//!         Mono::delay(10.millis()).await;
//!     }
//! }
//! ```

use crate::{Config, Error, KeyExpr, Publisher, Query, Queryable, Result, Subscriber};
use core::ptr;
use zenoh_pico_sys::*;

/// A zenoh session
///
/// A session represents a connection to the zenoh network. It can be used
/// to declare publishers and subscribers.
///
/// # Task Management
///
/// By default, `open()` starts background threads for network I/O. For RTIC
/// or other cooperative executors, use `open_without_tasks()` and manually
/// drive the session with `poll_read()` and `send_keepalive()`.
pub struct Session {
    inner: z_owned_session_t,
    read_task_started: bool,
    lease_task_started: bool,
}

impl Session {
    /// Open a new session with the given configuration
    ///
    /// This starts background threads for read and lease tasks.
    /// For RTIC or single-threaded executors, use `open_without_tasks()` instead.
    pub fn open(config: Config) -> Result<Self> {
        let mut sess = Self::open_without_tasks(config)?;

        // Start background tasks
        sess.start_tasks()?;

        Ok(sess)
    }

    /// Open a new session without starting background tasks
    ///
    /// Use this for RTIC or other single-threaded executors where you need
    /// manual control over when network I/O occurs.
    ///
    /// After opening, you must periodically call:
    /// - `poll_read()` to process incoming messages (recommended: every 10ms)
    /// - `send_keepalive()` to maintain the session (recommended: every 1s)
    ///
    /// Optionally, you can later call `start_tasks()` to switch to background threads.
    pub fn open_without_tasks(config: Config) -> Result<Self> {
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

            Ok(Self {
                inner: session.assume_init(),
                read_task_started: false,
                lease_task_started: false,
            })
        }
    }

    /// Start background read and lease tasks
    ///
    /// This is called automatically by `open()`. Only call this manually if you
    /// used `open_without_tasks()` and later want to switch to background threads.
    ///
    /// Returns an error if tasks are already started.
    pub fn start_tasks(&mut self) -> Result<()> {
        if self.read_task_started || self.lease_task_started {
            return Err(Error::TaskStartFailed);
        }

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
    ///
    /// After stopping, you must manually call `poll_read()` and `send_keepalive()`.
    pub fn stop_tasks(&mut self) {
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

    /// Poll for incoming messages and events
    ///
    /// Call this periodically (recommended: every 10ms) when not using background tasks.
    /// This processes any pending network data and dispatches callbacks.
    ///
    /// # Returns
    ///
    /// - `Ok(())` if polling completed successfully
    /// - `Err(ZenohError)` if there was a network error
    pub fn poll_read(&mut self) -> Result<()> {
        unsafe {
            let session_mut = z_session_loan_mut(&mut self.inner);
            let result = zp_read(session_mut, ptr::null());
            if result < 0 {
                return Err(Error::ZenohError(result));
            }
            Ok(())
        }
    }

    /// Send keepalive to maintain the session
    ///
    /// Call this periodically (recommended: every 1s) when not using background tasks.
    /// This sends keepalive messages to prevent the session from timing out.
    ///
    /// # Returns
    ///
    /// - `Ok(())` if keepalive was sent successfully
    /// - `Err(ZenohError)` if there was a network error
    pub fn send_keepalive(&mut self) -> Result<()> {
        unsafe {
            let session_mut = z_session_loan_mut(&mut self.inner);
            let result = zp_send_keep_alive(session_mut, ptr::null());
            if result < 0 {
                return Err(Error::ZenohError(result));
            }
            Ok(())
        }
    }

    /// Send join message for peer discovery
    ///
    /// Call this periodically in peer mode to announce presence to other peers.
    /// Not needed in client mode.
    ///
    /// # Returns
    ///
    /// - `Ok(())` if join was sent successfully
    /// - `Err(ZenohError)` if there was a network error
    pub fn send_join(&mut self) -> Result<()> {
        unsafe {
            let session_mut = z_session_loan_mut(&mut self.inner);
            let result = zp_send_join(session_mut, ptr::null());
            if result < 0 {
                return Err(Error::ZenohError(result));
            }
            Ok(())
        }
    }

    /// Check if background read task is running
    pub fn is_read_task_running(&self) -> bool {
        self.read_task_started
    }

    /// Check if background lease task is running
    pub fn is_lease_task_running(&self) -> bool {
        self.lease_task_started
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
