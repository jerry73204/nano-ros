# Unified Executor Design for nano-ros

This document describes the unified Context/Executor API design that works across
both `std` (desktop) and `no_std` (embedded/RTIC) targets.

## Design Goals

1. **Unified API** - Same `Context → Executor → Node` flow on all targets
2. **Same Node type** - No conditional compilation in node usage code
3. **Appropriate execution models** - Full `spin()` on std, `spin_once()` everywhere
4. **No accidental mixing** - Executor owns nodes, preventing dual-API misuse
5. **Portable code** - Write once, run on desktop and embedded

## Architecture Overview

```
                    ┌─────────────────────────────────────┐
                    │            Context                  │
                    │  (always available, no_std ok)      │
                    └──────────────┬──────────────────────┘
                                   │
            ┌──────────────────────┴──────────────────────┐
            │                                             │
            ▼                                             ▼
┌───────────────────────────┐             ┌───────────────────────────┐
│    PollingExecutor<N>     │             │     BasicExecutor         │
│    (always available)     │             │     (std only)            │
├───────────────────────────┤             ├───────────────────────────┤
│ + create_node()           │             │ + create_node()           │
│ + spin_once()             │             │ + spin_once()             │
│                           │             │ + spin()                  │
│                           │             │ + spin_async()            │
└───────────────────────────┘             └───────────────────────────┘
            │                                             │
            │         ┌───────────────────┐               │
            └────────►│       Node        │◄──────────────┘
                      │ (same type both)  │
                      ├───────────────────┤
                      │ + create_publisher│
                      │ + create_subscription
                      │ + create_timer    │
                      │ + get_clock()     │
                      └───────────────────┘
```

## Current State Analysis

### Existing Implementation

The current nano-ros codebase has:

1. **Context** (`context.rs`) - Exists but limited:
   - `Context::new(InitOptions)` - Creates context with domain_id
   - `Context::create_node()` - Creates node directly (no executor)
   - Gated behind `zenoh` feature

2. **ConnectedNode** (`connected.rs`) - Full node implementation:
   - Manages zenoh session, liveliness tokens
   - Creates publishers, subscribers, services, actions
   - Has timer support via `process_timers()`
   - Has clock via `get_clock()`, `now()`

3. **No Executor** - Currently missing:
   - No executor abstraction
   - No unified `spin_once()` method
   - No callback-based subscription processing
   - Users call methods directly on `ConnectedNode`

### Current Usage Pattern

```rust
// Current API (no executor)
let config = NodeConfig::new("my_node", "/");
let mut node = ConnectedNode::connect(config, "tcp/...")?;
let publisher = node.create_publisher::<Int32>("/topic")?;
let mut subscriber = node.create_subscriber::<Int32>("/topic")?;

// Manual polling
loop {
    if let Some(msg) = subscriber.try_recv()? {
        // process
    }
    node.process_timers(10);
    // sleep
}
```

## Proposed Design

### Core Types

```rust
// ═══════════════════════════════════════════════════════════════
// CONTEXT - Entry point (always available)
// ═══════════════════════════════════════════════════════════════

/// Context for creating executors and managing ROS 2 initialization
pub struct Context {
    domain_id: u32,
    #[cfg(feature = "zenoh")]
    transport_config: TransportConfig,
}

impl Context {
    /// Create a new context with options
    pub fn new(opts: InitOptions) -> Result<Self, RclrsError>;

    /// Create from environment (ROS_DOMAIN_ID, etc.)
    #[cfg(feature = "std")]
    pub fn from_env() -> Result<Self, RclrsError>;

    /// Create a polling executor (always available, no_std compatible)
    pub fn create_polling_executor<const MAX_NODES: usize>(&self)
        -> PollingExecutor<MAX_NODES>;

    /// Create a basic executor with full spin support (std only)
    #[cfg(feature = "std")]
    pub fn create_basic_executor(&self) -> BasicExecutor;

    /// Get domain ID
    pub fn domain_id(&self) -> u32;
}

// ═══════════════════════════════════════════════════════════════
// INIT OPTIONS - Configuration for context
// ═══════════════════════════════════════════════════════════════

/// Options for initializing a Context
pub struct InitOptions {
    domain_id: Option<u32>,
    #[cfg(feature = "zenoh")]
    locator: Option<&'static str>,
    #[cfg(feature = "zenoh")]
    session_mode: SessionMode,
}

impl InitOptions {
    pub fn new() -> Self;
    pub fn domain_id(self, id: u32) -> Self;
    #[cfg(feature = "zenoh")]
    pub fn locator(self, locator: &'static str) -> Self;
    #[cfg(feature = "zenoh")]
    pub fn session_mode(self, mode: SessionMode) -> Self;
}
```

### Executor Trait

```rust
/// Common trait for all executors
pub trait Executor {
    /// Create a node owned by this executor
    fn create_node<'a>(&mut self, opts: impl IntoNodeOptions<'a>)
        -> Result<&Node, RclrsError>;

    /// Process one iteration of pending work
    /// - Polls all subscriptions, invokes ready callbacks
    /// - Fires ready timers
    /// Returns result with counts of processed items
    fn spin_once(&mut self, delta_ms: u64) -> SpinOnceResult;
}

/// Result of a single spin iteration
pub struct SpinOnceResult {
    /// Number of subscription callbacks invoked
    pub subscriptions_processed: usize,
    /// Number of timers that fired
    pub timers_fired: usize,
    /// Number of service requests handled
    pub services_handled: usize,
}

/// Extended trait for executors with blocking/async spin (std only)
#[cfg(feature = "std")]
pub trait SpinExecutor: Executor {
    /// Blocking spin loop
    fn spin(&mut self, opts: SpinOptions) -> Vec<RclrsError>;

    /// Async spin (runs on background thread)
    fn spin_async(self, opts: SpinOptions)
        -> BoxFuture<'static, (Self, Vec<RclrsError>)>
    where
        Self: Sized;
}
```

### Polling Executor (no_std compatible)

```rust
/// Executor for manual polling (RTIC, Embassy, bare-metal)
///
/// This executor requires the user to call `spin_once()` periodically.
/// It does NOT spawn background threads or use async runtimes.
pub struct PollingExecutor<const MAX_NODES: usize = 4> {
    context: Context,
    nodes: heapless::Vec<NodeState, MAX_NODES>,
}

impl<const N: usize> PollingExecutor<N> {
    /// Create a node managed by this executor
    pub fn create_node<'a>(&mut self, opts: impl IntoNodeOptions<'a>)
        -> Result<NodeHandle<'_>, RclrsError>;

    /// Process one iteration of all nodes
    /// Call this from your RTIC task or main loop
    pub fn spin_once(&mut self, delta_ms: u64) -> SpinOnceResult;
}

impl<const N: usize> Executor for PollingExecutor<N> {
    // ... trait implementation
}
```

### Basic Executor (std only)

```rust
/// Full-featured executor with blocking and async spin
///
/// This executor can run a blocking spin loop or spawn an async task.
#[cfg(feature = "std")]
pub struct BasicExecutor {
    context: Context,
    nodes: Vec<NodeState>,
    halt_flag: Arc<AtomicBool>,
}

#[cfg(feature = "std")]
impl BasicExecutor {
    /// Create a node managed by this executor
    pub fn create_node<'a>(&mut self, opts: impl IntoNodeOptions<'a>)
        -> Result<NodeHandle<'_>, RclrsError>;

    /// Process one iteration (same as PollingExecutor)
    pub fn spin_once(&mut self, delta_ms: u64) -> SpinOnceResult;

    /// Blocking spin loop
    pub fn spin(&mut self, opts: SpinOptions) -> Vec<RclrsError>;

    /// Async spin (runs on separate thread)
    pub async fn spin_async(self, opts: SpinOptions)
        -> (Self, Vec<RclrsError>);

    /// Request the executor to stop spinning
    pub fn halt(&self);
}

#[cfg(feature = "std")]
impl Executor for BasicExecutor { /* ... */ }

#[cfg(feature = "std")]
impl SpinExecutor for BasicExecutor { /* ... */ }
```

### Spin Options

```rust
/// Options controlling spin behavior
#[derive(Default)]
pub struct SpinOptions {
    /// Stop after this duration
    pub timeout: Option<Duration>,
    /// Only process immediately available work (spin_once semantics)
    pub only_next: bool,
    /// Stop after processing this many callbacks
    pub max_callbacks: Option<usize>,
}

impl SpinOptions {
    pub fn new() -> Self;
    pub fn timeout(self, duration: Duration) -> Self;
    pub fn spin_once() -> Self;  // Convenience for only_next = true
}
```

### Node Handle

```rust
/// Handle to a node owned by an executor
///
/// This handle provides access to create publishers, subscribers, etc.
/// The actual node data is owned by the executor.
pub struct NodeHandle<'a> {
    // Reference to node inside executor
    node: &'a mut NodeState,
}

impl NodeHandle<'_> {
    // Publisher/Subscriber creation
    pub fn create_publisher<M: RosMessage>(
        &mut self,
        topic: &str
    ) -> Result<Publisher<M>, RclrsError>;

    pub fn create_subscription<M: RosMessage>(
        &mut self,
        topic: &str,
        callback: impl SubscriptionCallback<M>,
    ) -> Result<SubscriptionHandle, RclrsError>;

    // Timer creation
    pub fn create_timer(
        &mut self,
        period: Duration,
        callback: impl TimerCallback,
    ) -> Result<TimerHandle, RclrsError>;

    // Clock access
    pub fn get_clock(&self) -> &Clock;
    pub fn now(&self) -> Time;

    // Node info
    pub fn name(&self) -> &str;
    pub fn namespace(&self) -> &str;
    pub fn fully_qualified_name(&self) -> String;
}
```

### Callback Types

```rust
/// Trait for subscription callbacks (supports fn pointers and closures)
pub trait SubscriptionCallback<M: RosMessage>: Send {
    fn call(&mut self, msg: &M);
}

// Function pointer (no_std compatible)
impl<M: RosMessage> SubscriptionCallback<M> for fn(&M) {
    fn call(&mut self, msg: &M) { self(msg) }
}

// Boxed closure (requires alloc)
#[cfg(feature = "alloc")]
impl<M: RosMessage, F: FnMut(&M) + Send> SubscriptionCallback<M> for F {
    fn call(&mut self, msg: &M) { self(msg) }
}

/// Trait for timer callbacks
pub trait TimerCallback: Send {
    fn call(&mut self);
}

impl TimerCallback for fn() {
    fn call(&mut self) { self() }
}

#[cfg(feature = "alloc")]
impl<F: FnMut() + Send> TimerCallback for F {
    fn call(&mut self) { self() }
}
```

## Usage Examples

### RTIC with PollingExecutor

```rust
#![no_std]
#![no_main]

use nano_ros::prelude::*;
use rtic::app;

fn on_message(msg: &Int32) {
    defmt::info!("Received: {}", msg.data);
}

fn on_timer() {
    defmt::info!("Timer fired!");
}

#[app(device = pac, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        executor: PollingExecutor<2>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let ctx = Context::new(
            InitOptions::new()
                .domain_id(0)
                .locator("tcp/192.168.1.1:7447")
        ).unwrap();

        let mut executor = ctx.create_polling_executor();

        // Create node through executor
        let node = executor.create_node("rtic_node").unwrap();

        // Create subscription with function pointer callback
        node.create_subscription::<Int32>("/topic", on_message).unwrap();

        // Create timer
        node.create_timer(Duration::from_secs(1), on_timer).unwrap();

        // Spawn polling task
        ros_spin::spawn().ok();

        (Shared { executor }, Local {})
    }

    #[task(priority = 2, shared = [executor])]
    async fn ros_spin(mut cx: ros_spin::Context) {
        loop {
            cx.shared.executor.lock(|exec| {
                exec.spin_once(POLL_INTERVAL_MS as u64);
            });
            Mono::delay(POLL_INTERVAL_MS.millis()).await;
        }
    }
}
```

### Desktop with BasicExecutor

```rust
use nano_ros::prelude::*;

fn main() -> Result<(), RclrsError> {
    let ctx = Context::new(
        InitOptions::new()
            .domain_id(0)
            .locator("tcp/127.0.0.1:7447")
    )?;

    let mut executor = ctx.create_basic_executor();

    let node = executor.create_node("desktop_node")?;

    // Closure callback (requires alloc)
    let counter = std::sync::atomic::AtomicU32::new(0);
    node.create_subscription::<Int32>("/topic", move |msg| {
        counter.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        println!("Received: {} (count: {})", msg.data,
            counter.load(std::sync::atomic::Ordering::Relaxed));
    })?;

    node.create_timer(Duration::from_secs(1), || {
        println!("Timer tick!");
    })?;

    // Blocking spin
    executor.spin(SpinOptions::default());

    Ok(())
}
```

### Desktop with Manual Polling (same as embedded)

```rust
use nano_ros::prelude::*;

fn on_message(msg: &Int32) {
    println!("Received: {}", msg.data);
}

fn main() -> Result<(), RclrsError> {
    let ctx = Context::new(InitOptions::new())?;

    // PollingExecutor works on desktop too!
    let mut executor = ctx.create_polling_executor();

    let node = executor.create_node("polling_node")?;
    node.create_subscription::<Int32>("/topic", on_message)?;

    // Manual spin loop - identical to RTIC pattern
    loop {
        let result = executor.spin_once(10);
        if result.subscriptions_processed > 0 {
            println!("Processed {} messages", result.subscriptions_processed);
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}
```

### Generic Code (works with any executor)

```rust
use nano_ros::prelude::*;

fn setup_robot_node<E: Executor>(executor: &mut E) -> Result<(), RclrsError> {
    let node = executor.create_node("robot_node".namespace("/robot1"))?;

    node.create_subscription::<Twist>("/cmd_vel", handle_velocity)?;
    node.create_publisher::<Odometry>("/odom")?;
    node.create_timer(Duration::from_millis(100), publish_odom)?;

    Ok(())
}

// Works with both executor types:
fn main() {
    let ctx = Context::new(InitOptions::new()).unwrap();

    // Desktop: use BasicExecutor
    #[cfg(feature = "std")]
    {
        let mut executor = ctx.create_basic_executor();
        setup_robot_node(&mut executor).unwrap();
        executor.spin(SpinOptions::default());
    }

    // Embedded: use PollingExecutor
    #[cfg(not(feature = "std"))]
    {
        let mut executor = ctx.create_polling_executor();
        setup_robot_node(&mut executor).unwrap();
        loop {
            executor.spin_once(10);
            // platform-specific delay
        }
    }
}
```

## API Availability Matrix

| API | `no_std` | `alloc` | `std` |
|-----|----------|---------|-------|
| `Context::new()` | ✅ | ✅ | ✅ |
| `Context::from_env()` | ❌ | ❌ | ✅ |
| `ctx.create_polling_executor()` | ✅ | ✅ | ✅ |
| `ctx.create_basic_executor()` | ❌ | ❌ | ✅ |
| `PollingExecutor::spin_once()` | ✅ | ✅ | ✅ |
| `BasicExecutor::spin_once()` | ❌ | ❌ | ✅ |
| `BasicExecutor::spin()` | ❌ | ❌ | ✅ |
| `BasicExecutor::spin_async()` | ❌ | ❌ | ✅ |
| `Executor` trait | ✅ | ✅ | ✅ |
| `SpinExecutor` trait | ❌ | ❌ | ✅ |
| Function pointer callbacks | ✅ | ✅ | ✅ |
| Closure callbacks | ❌ | ✅ | ✅ |

## Feature Flags

```toml
[features]
default = ["std"]

# Core features
std = ["alloc", "nano-ros-core/std"]
alloc = ["nano-ros-core/alloc"]

# Transport
zenoh = ["nano-ros-transport/zenoh", "alloc"]

# Helpers
rtic = []  # RTIC timing constants and helpers
```

## Migration Path

### From Current API

```rust
// OLD (current)
let config = NodeConfig::new("my_node", "/");
let mut node = ConnectedNode::connect(config, "tcp/...")?;
let subscriber = node.create_subscriber::<Int32>("/topic")?;

loop {
    if let Some(msg) = subscriber.try_recv()? { ... }
    node.process_timers(10);
}

// NEW (with executor)
let ctx = Context::new(InitOptions::new().locator("tcp/..."))?;
let mut executor = ctx.create_polling_executor();
let node = executor.create_node("my_node")?;

node.create_subscription::<Int32>("/topic", |msg| { ... })?;

loop {
    executor.spin_once(10);
}
```

### Deprecation Strategy

1. Keep `ConnectedNode::connect()` as deprecated
2. Add deprecation warnings pointing to new API
3. Remove in next major version

## Implementation Notes

### Internal Node State

The executor owns `NodeState` which wraps the current `ConnectedNode`:

```rust
struct NodeState {
    inner: ConnectedNode,
    subscriptions: SubscriptionStorage,
    // ... other managed state
}

enum SubscriptionStorage {
    #[cfg(not(feature = "alloc"))]
    Static(heapless::Vec<SubscriptionEntry, MAX_SUBS>),
    #[cfg(feature = "alloc")]
    Dynamic(Vec<SubscriptionEntry>),
}
```

### Subscription Processing in spin_once

```rust
fn spin_once(&mut self, delta_ms: u64) -> SpinOnceResult {
    let mut result = SpinOnceResult::default();

    for node in &mut self.nodes {
        // Poll transport for incoming messages
        node.inner.poll_read()?;

        // Process each subscription
        for sub in &mut node.subscriptions {
            while let Some(msg) = sub.try_recv()? {
                sub.callback.call(&msg);
                result.subscriptions_processed += 1;
            }
        }

        // Process timers
        result.timers_fired += node.inner.process_timers(delta_ms);
    }

    result
}
```

## References

- [rclrs executor implementation](https://github.com/ros2-rust/ros2_rust/blob/main/rclrs/src/executor.rs)
- [RTIC documentation](https://rtic.rs/)
- [Phase 7 API Alignment Roadmap](../roadmap/phase-7-api-alignment.md)
