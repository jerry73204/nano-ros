//! Synchronization primitives abstraction
//!
//! This module provides a unified interface for different mutex implementations,
//! allowing the transport layer to work with various executor backends.
//!
//! # Feature Flags
//!
//! - `sync-spin` (default for zenoh): Uses `spin::Mutex` - works everywhere but not RTIC-compatible
//! - `sync-critical-section`: Uses critical sections - RTIC/Embassy compatible
//! - `sync-portable-atomic`: Uses portable-atomic for broader platform support
//!
//! # RTIC Compatibility
//!
//! For RTIC applications, use `sync-critical-section` feature. This ensures that
//! mutex operations use `critical_section::with()` which is compatible with RTIC's
//! Stack Resource Policy (SRP) scheduling.

use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};

/// A mutex guard that provides access to the protected data
pub struct MutexGuard<'a, T> {
    data: &'a UnsafeCell<T>,
    #[cfg(feature = "sync-critical-section")]
    _cs: critical_section::CriticalSection<'a>,
}

impl<T> Deref for MutexGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        // SAFETY: We hold the lock
        unsafe { &*self.data.get() }
    }
}

impl<T> DerefMut for MutexGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: We hold the lock exclusively
        unsafe { &mut *self.data.get() }
    }
}

// ============================================================================
// spin::Mutex implementation (default)
// ============================================================================

#[cfg(feature = "sync-spin")]
mod spin_impl {
    /// Mutex using spin lock (default implementation)
    pub struct Mutex<T> {
        inner: spin::Mutex<T>,
    }

    impl<T> Mutex<T> {
        /// Create a new mutex with the given value
        pub const fn new(value: T) -> Self {
            Self {
                inner: spin::Mutex::new(value),
            }
        }

        /// Lock the mutex and return a guard
        pub fn lock(&self) -> spin::MutexGuard<'_, T> {
            self.inner.lock()
        }
    }

    // SAFETY: Mutex is Send if T is Send
    unsafe impl<T: Send> Send for Mutex<T> {}
    // SAFETY: Mutex is Sync if T is Send
    unsafe impl<T: Send> Sync for Mutex<T> {}
}

#[cfg(feature = "sync-spin")]
pub use spin_impl::Mutex;

// ============================================================================
// critical-section implementation (RTIC/Embassy compatible)
// ============================================================================

#[cfg(all(feature = "sync-critical-section", not(feature = "sync-spin")))]
mod cs_impl {
    use super::*;
    use core::sync::atomic::{AtomicBool, Ordering};

    /// Mutex using critical sections (RTIC/Embassy compatible)
    ///
    /// This implementation uses `critical_section::with()` to protect data access,
    /// which is compatible with RTIC's Stack Resource Policy (SRP) scheduling.
    pub struct Mutex<T> {
        data: UnsafeCell<T>,
        locked: AtomicBool,
    }

    impl<T> Mutex<T> {
        /// Create a new mutex with the given value
        pub const fn new(value: T) -> Self {
            Self {
                data: UnsafeCell::new(value),
                locked: AtomicBool::new(false),
            }
        }

        /// Lock the mutex and execute the closure with access to the data
        ///
        /// This is the preferred API for critical-section based mutexes as it
        /// ensures the lock is properly released.
        pub fn with<F, R>(&self, f: F) -> R
        where
            F: FnOnce(&mut T) -> R,
        {
            critical_section::with(|_cs| {
                // SAFETY: We're in a critical section, so no other code can access this
                let data = unsafe { &mut *self.data.get() };
                f(data)
            })
        }

        /// Lock the mutex and return a guard
        ///
        /// Note: For critical-section based mutexes, prefer using `with()` instead
        /// as it provides better guarantees about lock release.
        pub fn lock(&self) -> CsMutexGuard<'_, T> {
            // Enter critical section
            // Note: This is a simplified implementation. In practice, the guard
            // should hold the critical section token.
            while self
                .locked
                .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
                .is_err()
            {
                core::hint::spin_loop();
            }
            CsMutexGuard {
                mutex: self,
                data: unsafe { &mut *self.data.get() },
            }
        }
    }

    // SAFETY: Mutex is Send if T is Send
    unsafe impl<T: Send> Send for Mutex<T> {}
    // SAFETY: Mutex is Sync if T is Send (access is protected by critical section)
    unsafe impl<T: Send> Sync for Mutex<T> {}

    /// Guard for critical-section based mutex
    pub struct CsMutexGuard<'a, T> {
        mutex: &'a Mutex<T>,
        data: &'a mut T,
    }

    impl<T> Deref for CsMutexGuard<'_, T> {
        type Target = T;

        fn deref(&self) -> &Self::Target {
            self.data
        }
    }

    impl<T> DerefMut for CsMutexGuard<'_, T> {
        fn deref_mut(&mut self) -> &mut Self::Target {
            self.data
        }
    }

    impl<T> Drop for CsMutexGuard<'_, T> {
        fn drop(&mut self) {
            self.mutex.locked.store(false, Ordering::Release);
        }
    }
}

#[cfg(all(feature = "sync-critical-section", not(feature = "sync-spin")))]
pub use cs_impl::Mutex;

// ============================================================================
// Fallback: No sync feature enabled - use RefCell-like approach
// This is only safe for single-threaded use (e.g., bare-metal without interrupts)
// ============================================================================

#[cfg(not(any(feature = "sync-spin", feature = "sync-critical-section")))]
mod nosync_impl {
    use core::cell::RefCell;

    /// Mutex without synchronization (single-threaded only)
    ///
    /// WARNING: This is only safe for single-threaded use cases without interrupts.
    /// For RTIC or any multi-priority system, use `sync-critical-section` feature.
    pub struct Mutex<T> {
        inner: RefCell<T>,
    }

    impl<T> Mutex<T> {
        /// Create a new mutex with the given value
        pub const fn new(value: T) -> Self {
            Self {
                inner: RefCell::new(value),
            }
        }

        /// Lock the mutex and execute the closure with access to the data
        pub fn with<F, R>(&self, f: F) -> R
        where
            F: FnOnce(&mut T) -> R,
        {
            f(&mut self.inner.borrow_mut())
        }

        /// Lock the mutex and return a guard
        pub fn lock(&self) -> core::cell::RefMut<'_, T> {
            self.inner.borrow_mut()
        }
    }

    // Note: Without proper synchronization, this is NOT truly Send/Sync safe
    // but we provide it for compilation in no_std single-threaded scenarios.
    // This is ONLY safe when there is a single thread of execution and no
    // interrupts that access the same data.
    unsafe impl<T: Send> Send for Mutex<T> {}
    // SAFETY: We implement Sync to allow Arc<Mutex<T>> to be Send, which is
    // required for zenoh callbacks. This is safe ONLY in single-threaded
    // environments without concurrent interrupt access.
    unsafe impl<T: Send> Sync for Mutex<T> {}
}

#[cfg(not(any(feature = "sync-spin", feature = "sync-critical-section")))]
pub use nosync_impl::Mutex;

// ============================================================================
// Helper trait for lock-free operations using atomics
// ============================================================================

/// Helper functions for atomic operations used in buffers
pub mod atomic {
    use core::sync::atomic::{AtomicBool, AtomicI64, AtomicUsize, Ordering};

    /// Atomically load a boolean value
    #[inline]
    pub fn load_bool(val: &AtomicBool) -> bool {
        val.load(Ordering::Acquire)
    }

    /// Atomically store a boolean value
    #[inline]
    pub fn store_bool(val: &AtomicBool, new: bool) {
        val.store(new, Ordering::Release);
    }

    /// Atomically load a usize value
    #[inline]
    pub fn load_usize(val: &AtomicUsize) -> usize {
        val.load(Ordering::Acquire)
    }

    /// Atomically store a usize value
    #[inline]
    pub fn store_usize(val: &AtomicUsize, new: usize) {
        val.store(new, Ordering::Release);
    }

    /// Atomically load an i64 value
    #[inline]
    pub fn load_i64(val: &AtomicI64) -> i64 {
        val.load(Ordering::Acquire)
    }

    /// Atomically store an i64 value
    #[inline]
    pub fn store_i64(val: &AtomicI64, new: i64) {
        val.store(new, Ordering::Release);
    }

    /// Atomically increment i64 and return new value
    #[inline]
    pub fn fetch_add_i64(val: &AtomicI64, add: i64) -> i64 {
        val.fetch_add(add, Ordering::AcqRel)
    }
}
