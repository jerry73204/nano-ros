//! Unified Executor API for nano-ros
//!
//! This module provides the executor abstraction that works on both std and no_std targets.
//!
//! # Architecture
//!
//! ```text
//! Context → Executor → Node
//! ```
//!
//! - **Context**: Entry point, creates executors
//! - **Executor**: Owns nodes, processes callbacks via `spin_once()`
//! - **Node**: Creates publishers, subscribers, timers
//!
//! # Executor Types
//!
//! - **PollingExecutor**: Always available, no_std compatible. User calls `spin_once()` manually.
//! - **BasicExecutor**: std only. Has blocking `spin()` and async `spin_async()`.
//!
//! # Example (RTIC/Embedded)
//!
//! ```ignore
//! let ctx = Context::new(InitOptions::new().locator("tcp/192.168.1.1:7447"))?;
//! let mut executor = ctx.create_polling_executor();
//! let node = executor.create_node("my_node")?;
//!
//! node.create_subscription::<Int32>("/topic", handle_message as fn(&Int32))?;
//!
//! // In your main loop or RTIC task:
//! loop {
//!     executor.spin_once(10);  // 10ms delta
//!     // delay...
//! }
//! ```
//!
//! # Example (Desktop)
//!
//! ```ignore
//! let ctx = Context::new(InitOptions::new())?;
//! let mut executor = ctx.create_basic_executor();
//! let node = executor.create_node("my_node")?;
//!
//! node.create_subscription::<Int32>("/topic", |msg| {
//!     println!("Received: {}", msg.data);
//! })?;
//!
//! executor.spin(SpinOptions::default());
//! ```

use nano_ros_core::{Deserialize, RosMessage, Time};

use crate::context::RclrsError;
use crate::options::{PublisherOptions, SubscriberOptions};
use crate::timer::TimerDuration;

#[cfg(feature = "zenoh")]
use crate::{
    ConnectedNode, ConnectedPublisher, ConnectedSubscriber, IntoNodeOptions, NodeConfig,
    DEFAULT_MAX_TIMERS, DEFAULT_MAX_TOKENS, DEFAULT_RX_BUFFER_SIZE,
};

#[cfg(feature = "zenoh")]
use nano_ros_transport::TransportConfig;

// ═══════════════════════════════════════════════════════════════════════════
// SPIN RESULT AND OPTIONS
// ═══════════════════════════════════════════════════════════════════════════

/// Result of a single spin iteration
///
/// Contains counts of how many items were processed during `spin_once()`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SpinOnceResult {
    /// Number of subscription callbacks invoked
    pub subscriptions_processed: usize,
    /// Number of timers that fired
    pub timers_fired: usize,
    /// Number of service requests handled
    pub services_handled: usize,
}

impl SpinOnceResult {
    /// Create a new empty result
    pub const fn new() -> Self {
        Self {
            subscriptions_processed: 0,
            timers_fired: 0,
            services_handled: 0,
        }
    }

    /// Check if any work was done
    pub const fn any_work(&self) -> bool {
        self.subscriptions_processed > 0 || self.timers_fired > 0 || self.services_handled > 0
    }

    /// Total number of callbacks invoked
    pub const fn total(&self) -> usize {
        self.subscriptions_processed + self.timers_fired + self.services_handled
    }
}

/// Options controlling spin behavior (for BasicExecutor)
#[derive(Debug, Clone, Default)]
pub struct SpinOptions {
    /// Stop after this duration (in milliseconds)
    pub timeout_ms: Option<u64>,
    /// Only process immediately available work (spin_once semantics)
    pub only_next: bool,
    /// Stop after processing this many callbacks
    pub max_callbacks: Option<usize>,
}

impl SpinOptions {
    /// Create default spin options (spin forever)
    pub const fn new() -> Self {
        Self {
            timeout_ms: None,
            only_next: false,
            max_callbacks: None,
        }
    }

    /// Set a timeout duration
    pub const fn timeout_ms(mut self, ms: u64) -> Self {
        self.timeout_ms = Some(ms);
        self
    }

    /// Only process one round of work (equivalent to spin_once)
    pub const fn spin_once() -> Self {
        Self {
            timeout_ms: None,
            only_next: true,
            max_callbacks: None,
        }
    }

    /// Stop after processing N callbacks
    pub const fn max_callbacks(mut self, n: usize) -> Self {
        self.max_callbacks = Some(n);
        self
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// CALLBACK TRAITS
// ═══════════════════════════════════════════════════════════════════════════

/// Trait for subscription callbacks
///
/// Implemented for:
/// - Function pointers `fn(&M)` (no_std compatible, when alloc is disabled)
/// - Any `FnMut(&M) + Send` (includes fn pointers and closures, requires alloc)
pub trait SubscriptionCallback<M: RosMessage>: Send {
    /// Invoke the callback with a message
    fn call(&mut self, msg: &M);
}

// When alloc is enabled: use blanket impl for FnMut (covers both fn pointers and closures)
#[cfg(feature = "alloc")]
impl<M: RosMessage, F: FnMut(&M) + Send> SubscriptionCallback<M> for F {
    fn call(&mut self, msg: &M) {
        (self)(msg)
    }
}

// When alloc is disabled: only support function pointers
#[cfg(not(feature = "alloc"))]
impl<M: RosMessage> SubscriptionCallback<M> for fn(&M) {
    fn call(&mut self, msg: &M) {
        (self)(msg)
    }
}

/// Trait for timer callbacks
///
/// Implemented for:
/// - Function pointers `fn()` (no_std compatible, when alloc is disabled)
/// - Any `FnMut() + Send` (includes fn pointers and closures, requires alloc)
pub trait ExecutorTimerCallback: Send {
    /// Invoke the callback
    fn call(&mut self);
}

// When alloc is enabled: use blanket impl for FnMut
#[cfg(feature = "alloc")]
impl<F: FnMut() + Send> ExecutorTimerCallback for F {
    fn call(&mut self) {
        (self)()
    }
}

// When alloc is disabled: only support function pointers
#[cfg(not(feature = "alloc"))]
impl ExecutorTimerCallback for fn() {
    fn call(&mut self) {
        (self)()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SUBSCRIPTION HANDLE
// ═══════════════════════════════════════════════════════════════════════════

/// Handle to a subscription created through NodeHandle
///
/// This handle can be used to cancel the subscription.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SubscriptionHandle {
    index: usize,
}

impl SubscriptionHandle {
    pub(crate) fn new(index: usize) -> Self {
        Self { index }
    }

    /// Get the subscription index
    pub fn index(&self) -> usize {
        self.index
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TYPE-ERASED CALLBACK (internal)
// ═══════════════════════════════════════════════════════════════════════════

/// Type-erased subscription callback for storing in executor
#[cfg(feature = "zenoh")]
pub(crate) trait ErasedCallback: Send {
    /// Try to receive and process a message, returns true if message was processed
    fn try_process(&mut self) -> Result<bool, RclrsError>;
}

/// Subscription entry combining subscriber and callback
#[cfg(feature = "zenoh")]
pub(crate) struct SubscriptionEntry<
    M: RosMessage,
    const RX_BUF: usize = DEFAULT_RX_BUFFER_SIZE,
    C: SubscriptionCallback<M> = fn(&M),
> {
    pub subscriber: ConnectedSubscriber<M, RX_BUF>,
    pub callback: C,
}

#[cfg(feature = "zenoh")]
impl<M: RosMessage + Deserialize + Send, const RX_BUF: usize, C: SubscriptionCallback<M>>
    ErasedCallback for SubscriptionEntry<M, RX_BUF, C>
{
    fn try_process(&mut self) -> Result<bool, RclrsError> {
        match self.subscriber.try_recv() {
            Ok(Some(msg)) => {
                self.callback.call(&msg);
                Ok(true)
            }
            Ok(None) => Ok(false),
            Err(_) => Err(RclrsError::DeserializationFailed),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// NODE STATE (internal)
// ═══════════════════════════════════════════════════════════════════════════

/// Maximum subscriptions per node (for no_std)
pub const DEFAULT_MAX_SUBSCRIPTIONS: usize = 8;

/// Internal node state owned by executor
#[cfg(feature = "zenoh")]
pub struct NodeState<
    const MAX_TOKENS: usize = DEFAULT_MAX_TOKENS,
    const MAX_TIMERS: usize = DEFAULT_MAX_TIMERS,
    const MAX_SUBS: usize = DEFAULT_MAX_SUBSCRIPTIONS,
> {
    /// The underlying connected node
    pub(crate) inner: ConnectedNode<MAX_TOKENS, MAX_TIMERS>,
    /// Subscriptions with their callbacks (boxed for type erasure)
    #[cfg(feature = "alloc")]
    pub(crate) subscriptions: alloc::vec::Vec<alloc::boxed::Box<dyn ErasedCallback>>,
    // For no_std without alloc, we can't have type-erased callbacks easily
    // Users would need to use function pointers and manual polling
}

#[cfg(feature = "zenoh")]
impl<const MAX_TOKENS: usize, const MAX_TIMERS: usize, const MAX_SUBS: usize>
    NodeState<MAX_TOKENS, MAX_TIMERS, MAX_SUBS>
{
    /// Create a new node state
    pub(crate) fn new(inner: ConnectedNode<MAX_TOKENS, MAX_TIMERS>) -> Self {
        Self {
            inner,
            #[cfg(feature = "alloc")]
            subscriptions: alloc::vec::Vec::new(),
        }
    }

    /// Get the node name
    pub fn name(&self) -> &str {
        self.inner.name()
    }

    /// Get the node namespace
    pub fn namespace(&self) -> &str {
        self.inner.namespace()
    }

    /// Process all subscriptions, returns count of messages processed
    #[cfg(feature = "alloc")]
    pub(crate) fn process_subscriptions(&mut self) -> Result<usize, RclrsError> {
        let mut count = 0;
        for sub in &mut self.subscriptions {
            while sub.try_process()? {
                count += 1;
            }
        }
        Ok(count)
    }

    /// Process timers
    pub(crate) fn process_timers(&mut self, delta_ms: u64) -> usize {
        self.inner.process_timers(delta_ms)
    }

    /// Poll for incoming data (RTIC/polling mode)
    #[cfg(any(feature = "rtic", feature = "polling"))]
    pub(crate) fn poll_read(&mut self) -> Result<(), RclrsError> {
        self.inner.poll_read().map_err(|_| RclrsError::PollFailed)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// NODE HANDLE
// ═══════════════════════════════════════════════════════════════════════════

/// Handle to a node owned by an executor
///
/// This handle provides access to create publishers, subscribers, timers, etc.
/// The actual node data is owned by the executor.
#[cfg(feature = "zenoh")]
pub struct NodeHandle<
    'a,
    const MAX_TOKENS: usize = DEFAULT_MAX_TOKENS,
    const MAX_TIMERS: usize = DEFAULT_MAX_TIMERS,
    const MAX_SUBS: usize = DEFAULT_MAX_SUBSCRIPTIONS,
> {
    pub(crate) node: &'a mut NodeState<MAX_TOKENS, MAX_TIMERS, MAX_SUBS>,
}

#[cfg(feature = "zenoh")]
impl<'a, const MAX_TOKENS: usize, const MAX_TIMERS: usize, const MAX_SUBS: usize>
    NodeHandle<'a, MAX_TOKENS, MAX_TIMERS, MAX_SUBS>
{
    /// Create a new node handle
    pub(crate) fn new(node: &'a mut NodeState<MAX_TOKENS, MAX_TIMERS, MAX_SUBS>) -> Self {
        Self { node }
    }

    /// Get the node name
    pub fn name(&self) -> &str {
        self.node.name()
    }

    /// Get the node namespace
    pub fn namespace(&self) -> &str {
        self.node.namespace()
    }

    /// Get the fully qualified node name
    #[cfg(feature = "alloc")]
    pub fn fully_qualified_name(&self) -> alloc::string::String {
        let ns = self.namespace();
        let name = self.name();
        if ns == "/" {
            alloc::format!("/{}", name)
        } else {
            alloc::format!("{}/{}", ns, name)
        }
    }

    /// Get the node's clock
    pub fn get_clock(&self) -> &nano_ros_core::Clock {
        self.node.inner.get_clock()
    }

    /// Get the current time from the node's clock
    pub fn now(&self) -> Time {
        self.node.inner.now()
    }

    /// Create a publisher for the given topic
    pub fn create_publisher<M: RosMessage>(
        &mut self,
        options: PublisherOptions,
    ) -> Result<ConnectedPublisher<M>, RclrsError> {
        self.node
            .inner
            .create_publisher::<M>(options)
            .map_err(|_| RclrsError::PublisherCreationFailed)
    }

    /// Create a subscription with a callback
    ///
    /// The callback will be invoked during `executor.spin_once()` when messages arrive.
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Function pointer (no_std compatible)
    /// fn handle_msg(msg: &Int32) {
    ///     println!("Received: {}", msg.data);
    /// }
    /// node.create_subscription::<Int32>("/topic", handle_msg)?;
    ///
    /// // Closure (requires alloc)
    /// node.create_subscription::<Int32>("/topic", |msg| {
    ///     println!("Received: {}", msg.data);
    /// })?;
    /// ```
    #[cfg(feature = "alloc")]
    pub fn create_subscription<M, C>(
        &mut self,
        options: SubscriberOptions,
        callback: C,
    ) -> Result<SubscriptionHandle, RclrsError>
    where
        M: RosMessage + Deserialize + Send + 'static,
        C: SubscriptionCallback<M> + 'static,
    {
        let subscriber = self
            .node
            .inner
            .create_subscriber_sized::<M, DEFAULT_RX_BUFFER_SIZE>(options)
            .map_err(|_| RclrsError::SubscriberCreationFailed)?;

        let entry = SubscriptionEntry {
            subscriber,
            callback,
        };

        let index = self.node.subscriptions.len();
        self.node.subscriptions.push(alloc::boxed::Box::new(entry));

        Ok(SubscriptionHandle::new(index))
    }

    /// Create a timer with a callback
    ///
    /// The callback will be invoked during `executor.spin_once()` when the timer fires.
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Function pointer
    /// fn on_timer() {
    ///     println!("Timer fired!");
    /// }
    /// node.create_timer(TimerDuration::from_millis(1000), on_timer)?;
    ///
    /// // Closure (requires alloc)
    /// node.create_timer(TimerDuration::from_millis(1000), || {
    ///     println!("Timer fired!");
    /// })?;
    /// ```
    pub fn create_timer(
        &mut self,
        period: TimerDuration,
        callback: crate::timer::TimerCallbackFn,
    ) -> Result<crate::timer::TimerHandle, RclrsError> {
        self.node
            .inner
            .create_timer_repeating(period, callback)
            .map_err(|_| RclrsError::TimerCreationFailed)
    }

    /// Create a timer with a boxed callback (requires alloc)
    #[cfg(feature = "alloc")]
    pub fn create_timer_boxed<F>(
        &mut self,
        period: TimerDuration,
        callback: F,
    ) -> Result<crate::timer::TimerHandle, RclrsError>
    where
        F: FnMut() + Send + 'static,
    {
        self.node
            .inner
            .create_timer_repeating_boxed(period, callback)
            .map_err(|_| RclrsError::TimerCreationFailed)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// EXECUTOR TRAIT
// ═══════════════════════════════════════════════════════════════════════════

/// Common trait for all executors
///
/// This trait provides the minimal interface that all executors implement.
/// Use this for writing generic code that works with any executor type.
///
/// # Example
///
/// ```ignore
/// fn setup_robot<E: Executor>(executor: &mut E) -> Result<(), RclrsError> {
///     let node = executor.create_node_with_handle("robot")?;
///     // ...
///     Ok(())
/// }
/// ```
#[cfg(feature = "zenoh")]
pub trait Executor {
    /// Process one iteration of pending work
    ///
    /// - Polls transport for incoming messages
    /// - Invokes subscription callbacks for received messages
    /// - Fires ready timers
    ///
    /// # Arguments
    /// * `delta_ms` - Time elapsed since last call (for timer processing)
    ///
    /// # Returns
    /// Result with counts of processed items
    fn spin_once(&mut self, delta_ms: u64) -> SpinOnceResult;
}

// ═══════════════════════════════════════════════════════════════════════════
// POLLING EXECUTOR (no_std compatible)
// ═══════════════════════════════════════════════════════════════════════════

/// Maximum number of nodes in a PollingExecutor
pub const DEFAULT_MAX_NODES: usize = 4;

/// Executor for manual polling (RTIC, Embassy, bare-metal)
///
/// This executor requires the user to call `spin_once()` periodically.
/// It does NOT spawn background threads or use async runtimes.
///
/// # Type Parameters
///
/// - `MAX_NODES`: Maximum number of nodes this executor can manage
///
/// # Example
///
/// ```ignore
/// let ctx = Context::new(InitOptions::new().locator("tcp/192.168.1.1:7447"))?;
/// let mut executor: PollingExecutor<2> = ctx.create_polling_executor();
///
/// let node = executor.create_node("my_node")?;
/// node.create_subscription::<Int32>("/topic", handle_msg)?;
///
/// // In your main loop:
/// loop {
///     executor.spin_once(10);  // 10ms delta
///     // platform delay...
/// }
/// ```
#[cfg(feature = "zenoh")]
pub struct PollingExecutor<const MAX_NODES: usize = DEFAULT_MAX_NODES> {
    /// Domain ID for creating nodes
    domain_id: u32,
    /// Transport configuration
    transport_config: TransportConfig<'static>,
    /// Nodes owned by this executor
    #[cfg(feature = "alloc")]
    nodes: alloc::vec::Vec<NodeState>,
}

#[cfg(feature = "zenoh")]
impl<const MAX_NODES: usize> PollingExecutor<MAX_NODES> {
    /// Create a new polling executor
    pub(crate) fn new(domain_id: u32, transport_config: TransportConfig<'static>) -> Self {
        Self {
            domain_id,
            transport_config,
            #[cfg(feature = "alloc")]
            nodes: alloc::vec::Vec::new(),
        }
    }

    /// Create a node managed by this executor
    ///
    /// Returns a `NodeHandle` that can be used to create publishers, subscribers, etc.
    /// The node is owned by the executor and will be processed during `spin_once()`.
    #[cfg(feature = "alloc")]
    pub fn create_node<'a, 'b>(
        &'a mut self,
        opts: impl IntoNodeOptions<'b>,
    ) -> Result<NodeHandle<'a>, RclrsError> {
        let node_opts = opts.into_node_options();

        let config = NodeConfig {
            name: node_opts.name,
            namespace: node_opts.namespace.unwrap_or("/"),
            domain_id: self.domain_id,
        };

        let inner = ConnectedNode::new(config, &self.transport_config)
            .map_err(|_| RclrsError::NodeCreationFailed)?;

        let node_state = NodeState::new(inner);
        self.nodes.push(node_state);

        let node = self.nodes.last_mut().unwrap();
        Ok(NodeHandle::new(node))
    }

    /// Get the number of nodes in this executor
    #[cfg(feature = "alloc")]
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Process one iteration of all nodes
    ///
    /// Call this from your RTIC task or main loop. Typically every 10ms.
    ///
    /// # Arguments
    /// * `delta_ms` - Time elapsed since last call (for timer processing)
    #[cfg(feature = "alloc")]
    pub fn spin_once(&mut self, delta_ms: u64) -> SpinOnceResult {
        let mut result = SpinOnceResult::new();

        for node in &mut self.nodes {
            // Poll for incoming data (if using rtic/polling mode)
            #[cfg(any(feature = "rtic", feature = "polling"))]
            let _ = node.poll_read();

            // Process subscriptions
            if let Ok(count) = node.process_subscriptions() {
                result.subscriptions_processed += count;
            }

            // Process timers
            result.timers_fired += node.process_timers(delta_ms);
        }

        result
    }
}

#[cfg(all(feature = "zenoh", feature = "alloc"))]
impl<const MAX_NODES: usize> Executor for PollingExecutor<MAX_NODES> {
    fn spin_once(&mut self, delta_ms: u64) -> SpinOnceResult {
        PollingExecutor::spin_once(self, delta_ms)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// BASIC EXECUTOR (std only)
// ═══════════════════════════════════════════════════════════════════════════

/// Full-featured executor with blocking and async spin (std only)
///
/// This executor can run a blocking spin loop or spawn work on background threads.
///
/// # Example
///
/// ```ignore
/// let ctx = Context::new(InitOptions::new())?;
/// let mut executor = ctx.create_basic_executor();
///
/// let node = executor.create_node("my_node")?;
/// node.create_subscription::<Int32>("/topic", |msg| {
///     println!("Received: {}", msg.data);
/// })?;
///
/// // Blocking spin
/// executor.spin(SpinOptions::default());
/// ```
#[cfg(all(feature = "zenoh", feature = "std"))]
pub struct BasicExecutor {
    /// Domain ID for creating nodes
    domain_id: u32,
    /// Transport configuration
    transport_config: TransportConfig<'static>,
    /// Nodes owned by this executor
    nodes: std::vec::Vec<NodeState>,
    /// Flag to request halt
    halt_flag: std::sync::Arc<std::sync::atomic::AtomicBool>,
}

#[cfg(all(feature = "zenoh", feature = "std"))]
impl BasicExecutor {
    /// Create a new basic executor
    pub(crate) fn new(domain_id: u32, transport_config: TransportConfig<'static>) -> Self {
        Self {
            domain_id,
            transport_config,
            nodes: std::vec::Vec::new(),
            halt_flag: std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
        }
    }

    /// Create a node managed by this executor
    pub fn create_node<'a, 'b>(
        &'a mut self,
        opts: impl IntoNodeOptions<'b>,
    ) -> Result<NodeHandle<'a>, RclrsError> {
        let node_opts = opts.into_node_options();

        let config = NodeConfig {
            name: node_opts.name,
            namespace: node_opts.namespace.unwrap_or("/"),
            domain_id: self.domain_id,
        };

        let inner = ConnectedNode::new(config, &self.transport_config)
            .map_err(|_| RclrsError::NodeCreationFailed)?;

        let node_state = NodeState::new(inner);
        self.nodes.push(node_state);

        let node = self.nodes.last_mut().unwrap();
        Ok(NodeHandle::new(node))
    }

    /// Get the number of nodes in this executor
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Process one iteration of all nodes
    pub fn spin_once(&mut self, delta_ms: u64) -> SpinOnceResult {
        let mut result = SpinOnceResult::new();

        for node in &mut self.nodes {
            // Process subscriptions
            if let Ok(count) = node.process_subscriptions() {
                result.subscriptions_processed += count;
            }

            // Process timers
            result.timers_fired += node.process_timers(delta_ms);
        }

        result
    }

    /// Blocking spin loop
    ///
    /// Runs until one of:
    /// - `halt()` is called
    /// - Timeout expires (if set in options)
    /// - Max callbacks reached (if set in options)
    /// - `only_next` is true (single iteration)
    ///
    /// # Arguments
    /// * `opts` - Options controlling spin behavior
    pub fn spin(&mut self, opts: SpinOptions) {
        use std::time::{Duration, Instant};

        const POLL_INTERVAL_MS: u64 = 10;

        let start = Instant::now();
        let timeout = opts.timeout_ms.map(Duration::from_millis);
        let mut total_callbacks = 0usize;

        self.halt_flag
            .store(false, std::sync::atomic::Ordering::SeqCst);

        loop {
            // Check halt flag
            if self.halt_flag.load(std::sync::atomic::Ordering::SeqCst) {
                break;
            }

            // Check timeout
            if let Some(timeout) = timeout {
                if start.elapsed() >= timeout {
                    break;
                }
            }

            // Spin once
            let result = self.spin_once(POLL_INTERVAL_MS);
            total_callbacks += result.total();

            // Check max callbacks
            if let Some(max) = opts.max_callbacks {
                if total_callbacks >= max {
                    break;
                }
            }

            // Single iteration mode
            if opts.only_next {
                break;
            }

            // Sleep between iterations
            std::thread::sleep(Duration::from_millis(POLL_INTERVAL_MS));
        }
    }

    /// Request the executor to stop spinning
    ///
    /// This sets a flag that will cause `spin()` to exit on its next iteration.
    /// Safe to call from another thread.
    pub fn halt(&self) {
        self.halt_flag
            .store(true, std::sync::atomic::Ordering::SeqCst);
    }

    /// Check if halt has been requested
    pub fn is_halted(&self) -> bool {
        self.halt_flag.load(std::sync::atomic::Ordering::SeqCst)
    }

    /// Get a clone of the halt flag for use in signal handlers
    pub fn halt_flag(&self) -> std::sync::Arc<std::sync::atomic::AtomicBool> {
        self.halt_flag.clone()
    }
}

#[cfg(all(feature = "zenoh", feature = "std"))]
impl Executor for BasicExecutor {
    fn spin_once(&mut self, delta_ms: u64) -> SpinOnceResult {
        BasicExecutor::spin_once(self, delta_ms)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TESTS
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_spin_once_result() {
        let result = SpinOnceResult::new();
        assert_eq!(result.subscriptions_processed, 0);
        assert_eq!(result.timers_fired, 0);
        assert_eq!(result.services_handled, 0);
        assert!(!result.any_work());
        assert_eq!(result.total(), 0);

        let result = SpinOnceResult {
            subscriptions_processed: 2,
            timers_fired: 1,
            services_handled: 0,
        };
        assert!(result.any_work());
        assert_eq!(result.total(), 3);
    }

    #[test]
    fn test_spin_options() {
        let opts = SpinOptions::new();
        assert!(opts.timeout_ms.is_none());
        assert!(!opts.only_next);
        assert!(opts.max_callbacks.is_none());

        let opts = SpinOptions::spin_once();
        assert!(opts.only_next);

        let opts = SpinOptions::new().timeout_ms(5000).max_callbacks(100);
        assert_eq!(opts.timeout_ms, Some(5000));
        assert_eq!(opts.max_callbacks, Some(100));
    }

    #[test]
    fn test_subscription_handle() {
        let handle = SubscriptionHandle::new(42);
        assert_eq!(handle.index(), 42);
    }
}
