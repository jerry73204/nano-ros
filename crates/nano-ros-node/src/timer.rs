//! Timer API for nano-ros
//!
//! This module provides timer support matching rclrs patterns while maintaining
//! `no_std` compatibility for embedded systems.
//!
//! # Overview
//!
//! Timers allow scheduling periodic or one-shot callbacks. In embedded environments
//! without background threads, timers must be processed manually via `process_timers()`.
//!
//! # Timer Modes
//!
//! - **Repeating**: Fires at regular intervals until canceled
//! - **OneShot**: Fires once after a delay, then becomes inert
//! - **Inert**: Never fires, useful as a placeholder
//!
//! # Example (with std)
//!
//! ```ignore
//! use nano_ros::prelude::*;
//! use nano_ros::timer::Duration;
//!
//! let mut node = ConnectedNode::connect(config, locator)?;
//!
//! // Create a repeating timer
//! let timer = node.create_timer_repeating(
//!     Duration::from_millis(100),
//!     || println!("Timer fired!"),
//! )?;
//!
//! // Process timers periodically
//! loop {
//!     node.process_timers(10); // 10ms elapsed
//!     std::thread::sleep(std::time::Duration::from_millis(10));
//! }
//! ```
//!
//! # Example (RTIC)
//!
//! ```ignore
//! // In RTIC, use a periodic task to process timers
//! #[task(priority = 2, shared = [node])]
//! async fn timer_process(mut cx: timer_process::Context) {
//!     loop {
//!         cx.shared.node.lock(|node| {
//!             node.process_timers(TIMER_PROCESS_INTERVAL_MS as u64);
//!         });
//!         Systick::delay(TIMER_PROCESS_INTERVAL_MS.millis()).await;
//!     }
//! }
//! ```

use core::marker::PhantomData;

/// Duration type for timer periods
///
/// This is a simple millisecond-based duration for `no_std` compatibility.
/// It can be converted to/from the ROS Duration type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TimerDuration {
    /// Duration in milliseconds
    millis: u64,
}

impl TimerDuration {
    /// Create a new duration from milliseconds
    pub const fn from_millis(millis: u64) -> Self {
        Self { millis }
    }

    /// Create a new duration from seconds
    pub const fn from_secs(secs: u64) -> Self {
        Self {
            millis: secs * 1000,
        }
    }

    /// Create a new duration from microseconds
    pub const fn from_micros(micros: u64) -> Self {
        Self {
            millis: micros / 1000,
        }
    }

    /// Create a zero duration
    pub const fn zero() -> Self {
        Self { millis: 0 }
    }

    /// Get duration as milliseconds
    pub const fn as_millis(&self) -> u64 {
        self.millis
    }

    /// Get duration as seconds (truncated)
    pub const fn as_secs(&self) -> u64 {
        self.millis / 1000
    }

    /// Check if duration is zero
    pub const fn is_zero(&self) -> bool {
        self.millis == 0
    }

    /// Saturating subtraction
    pub const fn saturating_sub(self, rhs: Self) -> Self {
        Self {
            millis: self.millis.saturating_sub(rhs.millis),
        }
    }
}

impl From<nano_ros_core::Duration> for TimerDuration {
    fn from(d: nano_ros_core::Duration) -> Self {
        let millis = (d.sec as i64 * 1000 + d.nanosec as i64 / 1_000_000) as u64;
        Self { millis }
    }
}

impl From<TimerDuration> for nano_ros_core::Duration {
    fn from(d: TimerDuration) -> Self {
        let sec = (d.millis / 1000) as i32;
        let nanosec = ((d.millis % 1000) * 1_000_000) as u32;
        nano_ros_core::Duration { sec, nanosec }
    }
}

/// Timer mode (repeating, one-shot, or inert)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerMode {
    /// Timer fires repeatedly at the specified period
    Repeating,
    /// Timer fires once then becomes inert
    OneShot,
    /// Timer never fires (placeholder)
    Inert,
}

/// Timer callback type
///
/// For `no_std` compatibility, we use a trait object approach with optional
/// heap allocation via the `alloc` feature. Without `alloc`, use function
/// pointers via `TimerCallbackFn`.
#[cfg(feature = "alloc")]
pub type TimerCallback = alloc::boxed::Box<dyn FnMut() + Send>;

/// Timer callback function pointer (no heap required)
pub type TimerCallbackFn = fn();

/// Internal timer state
///
/// Stored in the node's timer collection.
pub struct TimerState {
    /// Timer period in milliseconds
    period_ms: u64,
    /// Time elapsed since last fire in milliseconds
    elapsed_ms: u64,
    /// Timer mode
    mode: TimerMode,
    /// Whether the timer is canceled
    canceled: bool,
    /// Callback function pointer (no heap)
    callback_fn: Option<TimerCallbackFn>,
    /// Callback trait object (requires alloc)
    #[cfg(feature = "alloc")]
    callback_box: Option<TimerCallback>,
}

impl core::fmt::Debug for TimerState {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("TimerState")
            .field("period_ms", &self.period_ms)
            .field("elapsed_ms", &self.elapsed_ms)
            .field("mode", &self.mode)
            .field("canceled", &self.canceled)
            .field("has_callback_fn", &self.callback_fn.is_some())
            .finish()
    }
}

impl TimerState {
    /// Create a new timer state with function pointer callback
    pub fn new_with_fn(period: TimerDuration, mode: TimerMode, callback: TimerCallbackFn) -> Self {
        Self {
            period_ms: period.as_millis(),
            elapsed_ms: 0,
            mode,
            canceled: false,
            callback_fn: Some(callback),
            #[cfg(feature = "alloc")]
            callback_box: None,
        }
    }

    /// Create a new timer state with boxed callback (requires alloc)
    #[cfg(feature = "alloc")]
    pub fn new_with_box(period: TimerDuration, mode: TimerMode, callback: TimerCallback) -> Self {
        Self {
            period_ms: period.as_millis(),
            elapsed_ms: 0,
            mode,
            canceled: false,
            callback_fn: None,
            callback_box: Some(callback),
        }
    }

    /// Create an inert timer state
    pub fn new_inert(period: TimerDuration) -> Self {
        Self {
            period_ms: period.as_millis(),
            elapsed_ms: 0,
            mode: TimerMode::Inert,
            canceled: false,
            callback_fn: None,
            #[cfg(feature = "alloc")]
            callback_box: None,
        }
    }

    /// Get the timer period
    pub fn period(&self) -> TimerDuration {
        TimerDuration::from_millis(self.period_ms)
    }

    /// Get the timer mode
    pub fn mode(&self) -> TimerMode {
        self.mode
    }

    /// Check if the timer is canceled
    pub fn is_canceled(&self) -> bool {
        self.canceled
    }

    /// Cancel the timer
    pub fn cancel(&mut self) {
        self.canceled = true;
    }

    /// Reset the timer (uncancels and resets elapsed time)
    pub fn reset(&mut self) {
        self.canceled = false;
        self.elapsed_ms = 0;
    }

    /// Check if the timer is ready to fire
    pub fn is_ready(&self) -> bool {
        !self.canceled && self.mode != TimerMode::Inert && self.elapsed_ms >= self.period_ms
    }

    /// Get time until next call (0 if ready)
    pub fn time_until_next_call(&self) -> TimerDuration {
        if self.canceled || self.mode == TimerMode::Inert {
            return TimerDuration::from_millis(u64::MAX);
        }
        if self.elapsed_ms >= self.period_ms {
            TimerDuration::zero()
        } else {
            TimerDuration::from_millis(self.period_ms - self.elapsed_ms)
        }
    }

    /// Get time since last call
    pub fn time_since_last_call(&self) -> TimerDuration {
        TimerDuration::from_millis(self.elapsed_ms)
    }

    /// Set callback to function pointer
    pub fn set_callback_fn(&mut self, callback: TimerCallbackFn) {
        self.callback_fn = Some(callback);
        #[cfg(feature = "alloc")]
        {
            self.callback_box = None;
        }
    }

    /// Set callback to boxed callback (requires alloc)
    #[cfg(feature = "alloc")]
    pub fn set_callback_box(&mut self, callback: TimerCallback) {
        self.callback_fn = None;
        self.callback_box = Some(callback);
    }

    /// Set timer to repeating mode
    pub fn set_repeating(&mut self) {
        self.mode = TimerMode::Repeating;
    }

    /// Set timer to one-shot mode
    pub fn set_oneshot(&mut self) {
        self.mode = TimerMode::OneShot;
    }

    /// Set timer to inert mode
    pub fn set_inert(&mut self) {
        self.mode = TimerMode::Inert;
    }

    /// Update elapsed time and return true if timer should fire
    #[allow(dead_code)] // Used by ConnectedNode when zenoh feature is enabled
    pub(crate) fn update(&mut self, delta_ms: u64) -> bool {
        if self.canceled || self.mode == TimerMode::Inert {
            return false;
        }

        self.elapsed_ms = self.elapsed_ms.saturating_add(delta_ms);

        self.elapsed_ms >= self.period_ms
    }

    /// Fire the timer callback and handle mode-specific behavior
    #[allow(dead_code)] // Used by ConnectedNode when zenoh feature is enabled
    pub(crate) fn fire(&mut self) {
        // Execute callback
        if let Some(ref callback) = self.callback_fn {
            callback();
        }
        #[cfg(feature = "alloc")]
        if let Some(ref mut callback) = self.callback_box {
            callback();
        }

        // Handle mode-specific behavior
        match self.mode {
            TimerMode::Repeating => {
                // Reset elapsed time for next period
                self.elapsed_ms = self.elapsed_ms.saturating_sub(self.period_ms);
            }
            TimerMode::OneShot => {
                // Become inert after firing
                self.mode = TimerMode::Inert;
                self.elapsed_ms = 0;
            }
            TimerMode::Inert => {
                // Should not reach here
            }
        }
    }
}

/// A handle to a timer stored in a node
///
/// This is a lightweight handle that references a timer by index.
/// The actual timer state is stored in the `ConnectedNode`.
///
/// # Type Parameters
///
/// - `C`: Callback type marker (function pointer or boxed)
#[derive(Debug, Clone, Copy)]
pub struct TimerHandle<C = TimerCallbackFn> {
    /// Timer index in the node's timer collection
    index: usize,
    /// Phantom data for callback type
    _marker: PhantomData<C>,
}

impl<C> TimerHandle<C> {
    /// Create a new timer handle
    #[allow(dead_code)] // Used by ConnectedNode when zenoh feature is enabled
    pub(crate) fn new(index: usize) -> Self {
        Self {
            index,
            _marker: PhantomData,
        }
    }

    /// Get the timer index
    pub fn index(&self) -> usize {
        self.index
    }
}

/// Default maximum number of timers per node
pub const DEFAULT_MAX_TIMERS: usize = 8;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_timer_duration() {
        let d = TimerDuration::from_millis(1500);
        assert_eq!(d.as_millis(), 1500);
        assert_eq!(d.as_secs(), 1);

        let d2 = TimerDuration::from_secs(2);
        assert_eq!(d2.as_millis(), 2000);

        let d3 = TimerDuration::from_micros(5500);
        assert_eq!(d3.as_millis(), 5);
    }

    #[test]
    fn test_timer_duration_conversion() {
        let ros_dur = nano_ros_core::Duration::from_millis(1500);
        let timer_dur: TimerDuration = ros_dur.into();
        assert_eq!(timer_dur.as_millis(), 1500);

        let back: nano_ros_core::Duration = timer_dur.into();
        assert_eq!(back.sec, 1);
        assert_eq!(back.nanosec, 500_000_000);
    }

    fn test_callback() {
        // Empty callback for testing
    }

    #[test]
    fn test_timer_state_repeating() {
        let mut state = TimerState::new_with_fn(
            TimerDuration::from_millis(100),
            TimerMode::Repeating,
            test_callback,
        );

        assert_eq!(state.period().as_millis(), 100);
        assert_eq!(state.mode(), TimerMode::Repeating);
        assert!(!state.is_canceled());
        assert!(!state.is_ready());

        // Advance time
        assert!(!state.update(50));
        assert!(!state.is_ready());
        assert_eq!(state.time_until_next_call().as_millis(), 50);
        assert_eq!(state.time_since_last_call().as_millis(), 50);

        // Advance to ready
        assert!(state.update(50));
        assert!(state.is_ready());
        assert_eq!(state.time_until_next_call().as_millis(), 0);

        // Fire and check it repeats
        state.fire();
        assert_eq!(state.mode(), TimerMode::Repeating);
        assert!(!state.is_ready());
    }

    #[test]
    fn test_timer_state_oneshot() {
        let mut state = TimerState::new_with_fn(
            TimerDuration::from_millis(100),
            TimerMode::OneShot,
            test_callback,
        );

        assert_eq!(state.mode(), TimerMode::OneShot);

        // Advance to ready
        state.update(100);
        assert!(state.is_ready());

        // Fire and check it becomes inert
        state.fire();
        assert_eq!(state.mode(), TimerMode::Inert);
        assert!(!state.is_ready());
    }

    #[test]
    fn test_timer_state_inert() {
        let state = TimerState::new_inert(TimerDuration::from_millis(100));

        assert_eq!(state.mode(), TimerMode::Inert);
        assert!(!state.is_ready());
    }

    #[test]
    fn test_timer_cancel_reset() {
        let mut state = TimerState::new_with_fn(
            TimerDuration::from_millis(100),
            TimerMode::Repeating,
            test_callback,
        );

        state.update(50);
        state.cancel();
        assert!(state.is_canceled());
        assert!(!state.is_ready());

        state.update(100);
        assert!(!state.is_ready()); // Still canceled

        state.reset();
        assert!(!state.is_canceled());
        assert_eq!(state.time_since_last_call().as_millis(), 0);
    }

    #[test]
    fn test_timer_mode_changes() {
        let mut state = TimerState::new_with_fn(
            TimerDuration::from_millis(100),
            TimerMode::Repeating,
            test_callback,
        );

        state.set_oneshot();
        assert_eq!(state.mode(), TimerMode::OneShot);

        state.set_inert();
        assert_eq!(state.mode(), TimerMode::Inert);

        state.set_repeating();
        assert_eq!(state.mode(), TimerMode::Repeating);
    }

    #[test]
    fn test_timer_handle() {
        let handle: TimerHandle = TimerHandle::new(5);
        assert_eq!(handle.index(), 5);
    }
}
