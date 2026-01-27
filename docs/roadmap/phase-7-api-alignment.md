# Phase 7: API Alignment with rclrs 0.6.0

This roadmap outlines the refactoring effort to align nano-ros API with rclrs 0.6.0 patterns while maintaining `no_std` compatibility for embedded systems.

## Goals

1. **API Familiarity** - Users familiar with rclrs can easily use nano-ros
2. **Pattern Consistency** - Same patterns work across both libraries
3. **Embedded Compatibility** - Maintain `no_std` support and static allocation
4. **Backwards Compatibility** - Deprecate old API gradually, don't break existing code

## Reference

- rclrs 0.6.0: `/home/aeon/repos/nano-ros/external/ros2_rust/rclrs/`
- API Comparison: `/home/aeon/repos/nano-ros/docs/api-comparison-rclrs.md`

---

## Phase 7.1: Context and Initialization (Foundation)

**Goal**: Introduce `Context` type as the foundation for node creation.

### Tasks

- [x] **7.1.1** Create `Context` struct in `nano-ros-node`
  ```rust
  pub struct Context {
      domain_id: u32,
      // Shared state for all nodes
  }

  impl Context {
      pub fn new(options: InitOptions) -> Result<Self, RclrsError>;
      pub fn default_from_env() -> Result<Self, RclrsError>;
      pub fn domain_id(&self) -> u32;
      pub fn ok(&self) -> bool;
  }
  ```

- [x] **7.1.2** Create `InitOptions` struct
  ```rust
  pub struct InitOptions {
      domain_id: Option<u32>,
  }

  impl InitOptions {
      pub fn new() -> Self;
      pub fn with_domain_id(self, domain_id: Option<u32>) -> Self;
  }
  ```

- [x] **7.1.3** Add `Context::create_node()` method (without executor initially)
  ```rust
  impl Context {
      pub fn create_node<'a>(&self, options: impl IntoNodeOptions<'a>)
          -> Result<Node, RclrsError>;
  }
  ```

- [x] **7.1.4** Implement `IntoNodeOptions` trait for string conversion
  ```rust
  pub trait IntoNodeOptions<'a> {
      fn into_node_options(self) -> NodeOptions<'a>;
  }

  impl<'a> IntoNodeOptions<'a> for &'a str {
      fn into_node_options(self) -> NodeOptions<'a> {
          NodeOptions::new(self)
      }
  }
  ```

- [x] **7.1.5** Add `NodeOptions` builder with namespace method
  ```rust
  pub struct NodeOptions<'a> {
      pub name: &'a str,
      pub namespace: Option<&'a str>,
  }

  impl<'a> NodeOptions<'a> {
      pub fn namespace(self, ns: &'a str) -> Self;
  }

  // Enable: context.create_node("my_node".namespace("/ns"))?
  impl<'a> IntoNodeOptions<'a> for NodeOptions<'a> { ... }
  ```

- [x] **7.1.6** Add `Node` type alias
  ```rust
  #[cfg(feature = "std")]
  pub type Node = Arc<ConnectedNode>;

  #[cfg(not(feature = "std"))]
  pub type Node<'a, const N: usize = DEFAULT_MAX_TOKENS> = &'a mut ConnectedNode<N>;
  ```

- [x] **7.1.7** Deprecate direct `ConnectedNode::connect()` in favor of `Context`
  ```rust
  #[deprecated(since = "0.2.0", note = "Use Context::create_node() instead")]
  pub fn connect(...) -> ...
  ```

- [x] **7.1.8** Update examples to use new Context API

- [x] **7.1.9** Add tests for Context initialization

### Acceptance Criteria
- Context can be created from environment or options
- Nodes can be created via `context.create_node("name".namespace("/ns"))`
- Old API still works but shows deprecation warnings
- All existing tests pass

---

## Phase 7.2: QoS Builder Pattern

**Goal**: Enable fluent QoS configuration via string/topic extension traits.

### Tasks

- [x] **7.2.1** Extend `QosSettings` with more fields
  ```rust
  pub struct QosSettings {
      pub history: QosHistoryPolicy,
      pub reliability: QosReliabilityPolicy,
      pub durability: QosDurabilityPolicy,
      pub depth: u32,
  }

  pub enum QosHistoryPolicy {
      KeepLast { depth: u32 },
      KeepAll,
  }

  pub enum QosReliabilityPolicy {
      Reliable,
      BestEffort,
  }

  pub enum QosDurabilityPolicy {
      Volatile,
      TransientLocal,
  }
  ```

- [x] **7.2.2** Add QoS builder methods
  ```rust
  impl QosSettings {
      pub fn keep_last(self, depth: u32) -> Self;
      pub fn keep_all(self) -> Self;
      pub fn reliable(self) -> Self;
      pub fn best_effort(self) -> Self;
      pub fn volatile(self) -> Self;
      pub fn transient_local(self) -> Self;
  }
  ```

- [x] **7.2.3** Add predefined QoS profiles
  ```rust
  pub const QOS_PROFILE_DEFAULT: QosSettings = ...;
  pub const QOS_PROFILE_SENSOR_DATA: QosSettings = ...;
  pub const QOS_PROFILE_SERVICES_DEFAULT: QosSettings = ...;
  pub const QOS_PROFILE_PARAMETERS: QosSettings = ...;
  ```

- [x] **7.2.4** Create `IntoPrimitiveOptions` trait for topic strings
  ```rust
  pub trait IntoPublisherOptions<'a> {
      fn into_publisher_options(self) -> PublisherOptions<'a>;
  }

  pub struct PublisherOptions<'a> {
      pub topic: &'a str,
      pub qos: QosSettings,
  }

  impl<'a> PublisherOptions<'a> {
      pub fn keep_last(self, depth: u32) -> Self;
      pub fn reliable(self) -> Self;
      // ... all QoS methods
  }
  ```

- [x] **7.2.5** Implement string-to-options conversion
  ```rust
  impl<'a> IntoPublisherOptions<'a> for &'a str {
      fn into_publisher_options(self) -> PublisherOptions<'a> {
          PublisherOptions {
              topic: self,
              qos: QOS_PROFILE_DEFAULT,
          }
      }
  }

  // Enable: node.create_publisher::<M>("topic".keep_last(10).reliable())?
  ```

- [x] **7.2.6** Update `create_publisher` to accept options
  ```rust
  pub fn create_publisher<M: RosMessage>(
      &mut self,
      options: impl IntoPublisherOptions<'_>,
  ) -> Result<Publisher<M>, RclrsError>;
  ```

- [x] **7.2.7** Apply same pattern to subscribers, services, actions

- [x] **7.2.8** Deprecate `create_publisher_with_qos()` methods

- [x] **7.2.9** Update examples and tests

### Acceptance Criteria
- `node.create_publisher::<M>("topic".keep_last(10).reliable())` works
- Predefined QoS profiles available
- Old `_with_qos` methods deprecated
- Same pattern works for all primitives

---

## Phase 7.3: Typed Parameters

**Goal**: Replace untyped `ParameterValue` with typed parameter API.

### Tasks

- [x] **7.3.1** Create `ParameterVariant` trait
  ```rust
  pub trait ParameterVariant: Clone {
      fn to_parameter_value(&self) -> ParameterValue;
      fn from_parameter_value(value: &ParameterValue) -> Option<Self>;
  }

  // Implement for: bool, i64, f64, String, Vec<i64>, Vec<f64>, Vec<bool>, Vec<String>
  ```

- [x] **7.3.2** Create `ParameterBuilder` struct
  ```rust
  pub struct ParameterBuilder<'a, T: ParameterVariant> {
      node: &'a Node,
      name: &'a str,
      default: Option<T>,
      description: Option<&'a str>,
      range: Option<ParameterRange<T>>,
  }

  impl<'a, T: ParameterVariant> ParameterBuilder<'a, T> {
      pub fn default(self, value: T) -> Self;
      pub fn description(self, desc: &'a str) -> Self;
      pub fn range(self, range: ParameterRange<T>) -> Self;
      pub fn mandatory(self) -> Result<MandatoryParameter<T>, RclrsError>;
      pub fn optional(self) -> Result<OptionalParameter<T>, RclrsError>;
      pub fn read_only(self) -> Result<ReadOnlyParameter<T>, RclrsError>;
  }
  ```

- [x] **7.3.3** Create typed parameter types
  ```rust
  pub struct MandatoryParameter<T: ParameterVariant> {
      // ...
  }

  impl<T: ParameterVariant> MandatoryParameter<T> {
      pub fn get(&self) -> T;
      pub fn set(&self, value: T) -> Result<(), ParameterError>;
  }

  pub struct OptionalParameter<T: ParameterVariant> {
      // ...
  }

  impl<T: ParameterVariant> OptionalParameter<T> {
      pub fn get(&self) -> Option<T>;
      pub fn set(&self, value: Option<T>) -> Result<(), ParameterError>;
  }

  pub struct ReadOnlyParameter<T: ParameterVariant> {
      // ...
  }

  impl<T: ParameterVariant> ReadOnlyParameter<T> {
      pub fn get(&self) -> T;
      // No set method
  }
  ```

- [x] **7.3.4** Add `declare_parameter` to Node
  ```rust
  impl Node {
      pub fn declare_parameter<'a, T: ParameterVariant>(
          &'a self,
          name: &'a str,
      ) -> ParameterBuilder<'a, T>;
  }
  ```

- [x] **7.3.5** Create `ParameterRange` types
  ```rust
  pub struct IntegerRange {
      pub from: i64,
      pub to: i64,
      pub step: i64,
  }

  pub struct FloatingPointRange {
      pub from: f64,
      pub to: f64,
      pub step: f64,
  }
  ```

- [x] **7.3.6** Add `use_undeclared_parameters()` for dynamic access
  ```rust
  impl Node {
      pub fn use_undeclared_parameters(&self) -> UndeclaredParameters<'_>;
  }
  ```

- [x] **7.3.7** Keep `ParameterServer` for `no_std` environments

- [x] **7.3.8** Update examples and tests

### Acceptance Criteria
- Typed parameters work: `node.declare_parameter("speed").default(5.0).mandatory()?`
- Range validation enforced
- Read-only parameters cannot be set
- `no_std` still supported via `ParameterServer`

---

## Phase 7.4: Timer API

**Goal**: Add timer support matching rclrs patterns.

**Status**: COMPLETE

### Tasks

- [x] **7.4.1** Create `Timer` struct
  ```rust
  // Implemented as TimerState and TimerHandle in nano-ros-node/src/timer.rs
  pub struct TimerState {
      period_ms: u64,
      elapsed_ms: u64,
      mode: TimerMode,
      canceled: bool,
      callback_fn: Option<TimerCallbackFn>,
      #[cfg(feature = "alloc")]
      callback_box: Option<TimerCallback>,
  }
  ```

- [x] **7.4.2** Add timer creation methods to Node
  ```rust
  impl ConnectedNode {
      pub fn create_timer_repeating(&mut self, period: TimerDuration, callback: TimerCallbackFn) -> Result<TimerHandle, ConnectedNodeError>;
      pub fn create_timer_oneshot(&mut self, delay: TimerDuration, callback: TimerCallbackFn) -> Result<TimerHandle, ConnectedNodeError>;
      pub fn create_timer_inert(&mut self, period: TimerDuration) -> Result<TimerHandle, ConnectedNodeError>;
      #[cfg(feature = "alloc")]
      pub fn create_timer_repeating_boxed<F>(&mut self, period: TimerDuration, callback: F) -> Result<TimerHandle, ConnectedNodeError>;
      #[cfg(feature = "alloc")]
      pub fn create_timer_oneshot_boxed<F>(&mut self, delay: TimerDuration, callback: F) -> Result<TimerHandle, ConnectedNodeError>;
  }
  ```

- [x] **7.4.3** Add Timer methods
  ```rust
  impl TimerState {
      pub fn cancel(&mut self);
      pub fn is_canceled(&self) -> bool;
      pub fn reset(&mut self);
      pub fn is_ready(&self) -> bool;
      pub fn time_until_next_call(&self) -> TimerDuration;
      pub fn time_since_last_call(&self) -> TimerDuration;
      pub fn period(&self) -> TimerDuration;
      pub fn set_callback_fn(&mut self, callback: TimerCallbackFn);
      pub fn set_repeating(&mut self);
      pub fn set_oneshot(&mut self);
      pub fn set_inert(&mut self);
  }

  impl ConnectedNode {
      pub fn cancel_timer(&mut self, handle: &TimerHandle) -> Result<(), ConnectedNodeError>;
      pub fn reset_timer(&mut self, handle: &TimerHandle) -> Result<(), ConnectedNodeError>;
      pub fn is_timer_ready(&self, handle: &TimerHandle) -> bool;
      pub fn is_timer_canceled(&self, handle: &TimerHandle) -> bool;
      pub fn get_timer_period(&self, handle: &TimerHandle) -> Option<TimerDuration>;
      pub fn time_until_next_call(&self, handle: &TimerHandle) -> Option<TimerDuration>;
      pub fn time_since_last_call(&self, handle: &TimerHandle) -> Option<TimerDuration>;
      pub fn process_timers(&mut self, delta_ms: u64) -> usize;
  }
  ```

- [x] **7.4.4** Integrate timers with executor (Phase 7.6)

- [x] **7.4.5** Add `no_std` variant using const generics
  ```rust
  pub struct ConnectedNode<
      const MAX_TOKENS: usize = DEFAULT_MAX_TOKENS,
      const MAX_TIMERS: usize = DEFAULT_MAX_TIMERS,
  > { ... }

  pub const DEFAULT_MAX_TIMERS: usize = 8;
  ```

- [x] **7.4.6** Add tests and examples
  - Unit tests in `timer.rs` for TimerDuration, TimerState, TimerHandle
  - TIMER_PROCESS_INTERVAL_MS constant in rtic.rs

### Acceptance Criteria
- Repeating timers fire at specified interval
- One-shot timers fire once
- Timers can be canceled and reset
- Callbacks can be changed at runtime

---

## Phase 7.5: Clock API

**Goal**: Add clock abstraction for time sources.

**Status**: COMPLETE

### Tasks

- [x] **7.5.1** Create `ClockType` enum
  ```rust
  // Implemented in nano-ros-core/src/clock.rs
  pub enum ClockType {
      SystemTime,   // Wall clock (default)
      SteadyTime,   // Monotonic clock
      RosTime,      // Simulation time (can be paused/scaled)
  }
  ```

- [x] **7.5.2** Create `Clock` struct
  ```rust
  // Implemented in nano-ros-core/src/clock.rs
  pub struct Clock {
      clock_type: ClockType,
  }

  impl Clock {
      pub const fn new(clock_type: ClockType) -> Self;
      pub const fn system() -> Self;
      pub const fn steady() -> Self;
      pub const fn ros_time() -> Self;
      pub const fn clock_type(&self) -> ClockType;
      pub fn now(&self) -> Time;
  }
  ```

- [x] **7.5.3** Extend `Time` type with more operations
  ```rust
  // Implemented in nano-ros-core/src/time.rs
  impl Time {
      pub const fn from_nanos(nanos: i64) -> Self;
      pub const fn to_nanos(&self) -> i64;
      pub const fn as_duration(&self) -> Duration;
      pub const fn from_duration(d: Duration) -> Self;
      pub fn from_secs_f64(secs: f64) -> Self;
      pub fn to_secs_f64(&self) -> f64;
  }

  impl Add<Duration> for Time { type Output = Time; }
  impl Sub<Duration> for Time { type Output = Time; }
  impl Sub<Time> for Time { type Output = Duration; }

  // Also extended Duration:
  impl Duration {
      pub const fn from_nanos(nanos: i64) -> Self;
      pub const fn to_nanos(&self) -> i64;
      pub fn from_secs_f64(secs: f64) -> Self;
      pub fn to_secs_f64(&self) -> f64;
  }

  impl Add for Duration { type Output = Duration; }
  impl Sub for Duration { type Output = Duration; }
  ```

- [x] **7.5.4** Add `get_clock()` to Node
  ```rust
  // Implemented in nano-ros-node/src/connected.rs
  impl ConnectedNode {
      pub fn get_clock(&self) -> &Clock;
      pub fn now(&self) -> Time;
      pub fn clock_type(&self) -> ClockType;
      pub fn set_clock_type(&mut self, clock_type: ClockType);
  }
  ```

- [x] **7.5.5** Add ROS time override support
  ```rust
  // Implemented in nano-ros-core/src/clock.rs
  impl Clock {
      pub fn set_ros_time_override(nanos: i64);
      pub fn set_ros_time_override_time(time: Time);
      pub fn clear_ros_time_override();
      pub fn is_ros_time_override_active() -> bool;
      pub fn get_ros_time_override() -> Option<Time>;
  }

  // For no_std environments:
  impl Clock {
      pub fn update_steady_time(delta_nanos: i64);
      pub fn update_steady_time_ms(delta_ms: u64);
      pub fn set_steady_time(nanos: i64);
      pub fn get_steady_time_nanos() -> i64;
  }
  ```

- [x] **7.5.6** Add tests for clock operations
  - Unit tests in `time.rs` for Time/Duration arithmetic
  - Unit tests in `clock.rs` for ClockType, Clock, ROS time override

### Acceptance Criteria
- System and steady clocks work
- ROS time can be overridden for simulation
- Time arithmetic works correctly
- Nodes have associated clock

---

## Phase 7.6: Unified Executor Model

**Goal**: Add unified executor abstraction that works on both std and no_std targets.

**Design Document**: See `docs/architecture/unified-executor-design.md` for full design.

### Design Principles

1. **Unified API** - Same `Context → Executor → Node` flow everywhere
2. **Two Executor Types**:
   - `PollingExecutor` - Always available, no_std compatible, user calls `spin_once()`
   - `BasicExecutor` - std only, has blocking `spin()` and `spin_async()`
3. **Executor owns nodes** - Prevents mixing manual polling with executor-managed spinning
4. **Callback-based subscriptions** - Stored in executor, invoked during `spin_once()`

### Tasks

#### 7.6.1 Core Types

- [x] **7.6.1.1** Deprecate `Context::create_node()` (deprecated, not removed for backwards compat)
  ```rust
  impl Context {
      pub fn new(opts: InitOptions) -> Result<Self, RclrsError>;
      #[cfg(feature = "std")]
      pub fn from_env() -> Result<Self, RclrsError>;
      pub fn create_polling_executor<const N: usize>(&self) -> PollingExecutor<N>;
      #[cfg(feature = "std")]
      pub fn create_basic_executor(&self) -> BasicExecutor;
  }
  ```

- [x] **7.6.1.2** Update `InitOptions` with transport configuration
  ```rust
  pub struct InitOptions {
      domain_id: Option<u32>,
      #[cfg(feature = "zenoh")]
      locator: Option<&'static str>,
      #[cfg(feature = "zenoh")]
      session_mode: SessionMode,
  }
  ```

- [x] **7.6.1.3** Create `SpinOnceResult` struct
  ```rust
  pub struct SpinOnceResult {
      pub subscriptions_processed: usize,
      pub timers_fired: usize,
      pub services_handled: usize,
  }
  ```

- [x] **7.6.1.4** Create `SpinOptions` struct
  ```rust
  pub struct SpinOptions {
      pub timeout: Option<Duration>,
      pub only_next: bool,
      pub max_callbacks: Option<usize>,
  }
  ```

#### 7.6.2 Executor Trait

- [x] **7.6.2.1** Create `Executor` trait (always available)
  ```rust
  pub trait Executor {
      fn create_node<'a>(&mut self, opts: impl IntoNodeOptions<'a>)
          -> Result<NodeHandle<'_>, RclrsError>;
      fn spin_once(&mut self, delta_ms: u64) -> SpinOnceResult;
  }
  ```

- [x] **7.6.2.2** Create `SpinExecutor` trait (std only)
  ```rust
  #[cfg(feature = "std")]
  pub trait SpinExecutor: Executor {
      fn spin(&mut self, opts: SpinOptions) -> Vec<RclrsError>;
      fn spin_async(self, opts: SpinOptions)
          -> BoxFuture<'static, (Self, Vec<RclrsError>)>
      where Self: Sized;
  }
  ```

#### 7.6.3 PollingExecutor (no_std compatible)

- [x] **7.6.3.1** Create `PollingExecutor<const MAX_NODES: usize>` struct
  ```rust
  pub struct PollingExecutor<const MAX_NODES: usize = 4> {
      context: Context,
      nodes: heapless::Vec<NodeState, MAX_NODES>,
  }
  ```

- [x] **7.6.3.2** Implement `Executor` trait for `PollingExecutor`

- [x] **7.6.3.3** Implement `spin_once()` to process all nodes
  - Poll transport for incoming messages
  - Iterate subscriptions, invoke ready callbacks
  - Fire ready timers
  - Handle service requests

#### 7.6.4 BasicExecutor (std only)

- [x] **7.6.4.1** Create `BasicExecutor` struct
  ```rust
  #[cfg(feature = "std")]
  pub struct BasicExecutor {
      context: Context,
      nodes: Vec<NodeState>,
      halt_flag: Arc<AtomicBool>,
  }
  ```

- [x] **7.6.4.2** Implement `Executor` trait for `BasicExecutor`

- [x] **7.6.4.3** Implement `spin_once()` (same logic as PollingExecutor)

- [x] **7.6.4.4** Implement blocking `spin()`
  ```rust
  pub fn spin(&mut self, opts: SpinOptions) -> Vec<RclrsError> {
      loop {
          self.spin_once(POLL_INTERVAL_MS);
          if self.should_stop(&opts) { break; }
          std::thread::sleep(Duration::from_millis(POLL_INTERVAL_MS));
      }
  }
  ```

- [ ] **7.6.4.5** Implement `spin_async()` (spawns background thread)

- [x] **7.6.4.6** Implement `halt()` method

#### 7.6.5 NodeHandle and Callbacks

- [x] **7.6.5.1** Create `NodeHandle<'a>` struct (reference to node in executor)
  ```rust
  pub struct NodeHandle<'a> {
      node: &'a mut NodeState,
  }
  ```

- [x] **7.6.5.2** Implement `NodeHandle` methods
  - `create_publisher<M>(topic)` - Same as current
  - `create_subscription<M>(topic, callback)` - With callback storage
  - `create_timer(period, callback)` - With callback storage
  - `get_clock()`, `now()` - Clock access
  - `name()`, `namespace()`, `fully_qualified_name()`

- [x] **7.6.5.3** Create `SubscriptionCallback<M>` trait
  ```rust
  pub trait SubscriptionCallback<M: RosMessage>: Send {
      fn call(&mut self, msg: &M);
  }

  impl<M: RosMessage> SubscriptionCallback<M> for fn(&M) { ... }

  #[cfg(feature = "alloc")]
  impl<M, F: FnMut(&M) + Send> SubscriptionCallback<M> for F { ... }
  ```

- [x] **7.6.5.4** Create `TimerCallback` trait
  ```rust
  pub trait TimerCallback: Send {
      fn call(&mut self);
  }
  ```

- [x] **7.6.5.5** Create internal `SubscriptionEntry` for callback storage
  ```rust
  struct SubscriptionEntry {
      subscriber: ConnectedSubscriber</* ... */>,
      callback: Box<dyn ErasedCallback>,  // or function pointer for no_std
  }
  ```

#### 7.6.6 Internal NodeState

- [x] **7.6.6.1** Create `NodeState` struct to wrap `ConnectedNode`
  ```rust
  struct NodeState {
      inner: ConnectedNode,
      subscriptions: SubscriptionStorage,
      // timers already in ConnectedNode
  }
  ```

- [x] **7.6.6.2** Implement subscription storage
  - `heapless::Vec` for no_std
  - `Vec` for std/alloc

#### 7.6.7 Deprecation and Migration

- [x] **7.6.7.1** Deprecate `Context::create_node()` direct method
  ```rust
  #[deprecated(since = "0.2.0", note = "Use create_polling_executor() or create_basic_executor()")]
  pub fn create_node(...) -> ...
  ```

- [x] **7.6.7.2** Deprecate `ConnectedNode::new()` direct usage
  ```rust
  #[deprecated(since = "0.2.0", note = "Use Context + Executor instead")]
  pub fn connect(...) -> ...
  ```

- [x] **7.6.7.3** Keep old API working for backwards compatibility

#### 7.6.8 Tests and Examples

- [x] **7.6.8.1** Add unit tests for `PollingExecutor`
  - Create nodes
  - Create subscriptions with callbacks
  - `spin_once()` invokes callbacks

- [x] **7.6.8.2** Add unit tests for `BasicExecutor`
  - All PollingExecutor tests
  - `spin()` with timeout
  - `halt()` stops spinning

- [x] **7.6.8.3** Update `native-rs-talker` example to use executor

- [x] **7.6.8.4** Update `native-rs-listener` example to use executor

- [x] **7.6.8.5** Update RTIC example to use `PollingExecutor`

- [x] **7.6.8.6** Action examples already use executor API (manual loop for complex handling)

### Acceptance Criteria

- [x] Unified `Context → Executor → Node` API works on all targets
- [x] `PollingExecutor` works on no_std (RTIC, Embassy)
- [x] `BasicExecutor` provides full `spin()` on std
- [x] `spin_async()` deferred to Phase 7.10 (requires Send bounds on zenoh-pico types)
- [x] Callbacks invoked during `spin_once()`
- [x] Function pointer callbacks work without alloc
- [x] Closure callbacks work with alloc
- [x] Old API still works (deprecated)
- [x] All examples updated to use executor API
- [x] Tests pass on both std and no_std

### Notes on spin_async()

The `spin_async()` method is commented out because zenoh-pico's C types are not `Send`.
This is a fundamental limitation of the FFI bindings. For async support:
- Phase 7.10 will address this with either a pure-Rust zenoh backend or unsafe Send wrappers
- For now, use `spin()` (blocking) or `spin_once()` (manual polling)

---

## Phase 7.7: Service Callbacks (Merged into 7.6)

**Note**: Service callback support is now part of Phase 7.6's executor model.
The `spin_once()` method handles service requests alongside subscriptions.

Tasks moved to 7.6:
- Service callback registration via `NodeHandle::create_service()`
- Service request handling in `spin_once()`

---

## Phase 7.8: Logger API

**Goal**: Add integrated logging with per-node loggers.

### Tasks

- [ ] **7.8.1** Create `Logger` struct
  ```rust
  pub struct Logger {
      name: String,
  }

  impl Logger {
      pub fn new(name: &str) -> Self;
  }
  ```

- [ ] **7.8.2** Create `LoggerHandle` with modifiers
  ```rust
  pub struct LoggerHandle<'a> {
      logger: &'a Logger,
      severity: LogSeverity,
      once: bool,
      skip_first: bool,
      throttle: Option<Duration>,
  }

  impl<'a> LoggerHandle<'a> {
      pub fn once(self) -> Self;
      pub fn skip_first(self) -> Self;
      pub fn throttle(self, duration: Duration) -> Self;
  }
  ```

- [ ] **7.8.3** Add severity methods to Logger
  ```rust
  impl Logger {
      pub fn debug(&self) -> LoggerHandle<'_>;
      pub fn info(&self) -> LoggerHandle<'_>;
      pub fn warn(&self) -> LoggerHandle<'_>;
      pub fn error(&self) -> LoggerHandle<'_>;
      pub fn fatal(&self) -> LoggerHandle<'_>;
  }
  ```

- [ ] **7.8.4** Add logger to Node
  ```rust
  impl Node {
      pub fn logger(&self) -> &Logger;
      pub fn debug(&self) -> LoggerHandle<'_>;
      pub fn info(&self) -> LoggerHandle<'_>;
      pub fn warn(&self) -> LoggerHandle<'_>;
      pub fn error(&self) -> LoggerHandle<'_>;
      pub fn fatal(&self) -> LoggerHandle<'_>;
  }
  ```

- [ ] **7.8.5** Create logging macros
  ```rust
  #[macro_export]
  macro_rules! log {
      ($logger:expr, $($arg:tt)*) => { ... };
  }

  #[macro_export]
  macro_rules! log_info {
      ($logger:expr, $($arg:tt)*) => { ... };
  }
  // etc.
  ```

- [ ] **7.8.6** Integrate with `log` crate backend

- [ ] **7.8.7** Add `no_std` support via `defmt` backend

### Acceptance Criteria
- Per-node logging with name prefix
- Throttle/once/skip_first modifiers work
- Integrates with standard `log` crate
- `no_std` works with `defmt`

---

## Phase 7.9: Worker Pattern

**Goal**: Add Worker<Payload> for state sharing across callbacks.

### Tasks

- [ ] **7.9.1** Create `Worker<Payload>` struct
  ```rust
  pub struct Worker<Payload> {
      node: Node,
      payload: Arc<Mutex<Payload>>,
  }
  ```

- [ ] **7.9.2** Add worker creation to Node
  ```rust
  impl Node {
      pub fn create_worker<P: Send + 'static>(&self, payload: P) -> Worker<P>;
  }
  ```

- [ ] **7.9.3** Add run methods to Worker
  ```rust
  impl<P: Send> Worker<P> {
      pub fn run<F, R>(&self, f: F) -> R
      where
          F: FnOnce(&mut P) -> R;

      pub fn listen<F>(&self, f: F) -> ActivityListener<P>
      where
          F: FnMut(&mut P) + Send + 'static;

      pub fn listen_until<F>(&self, f: F) -> Promise<()>
      where
          F: FnMut(&mut P) -> bool + Send + 'static;
  }
  ```

- [ ] **7.9.4** Add primitive creation from Worker
  ```rust
  impl<P: Send> Worker<P> {
      pub fn create_subscription<M, F>(
          &self,
          options: impl IntoSubscriberOptions<'_>,
          callback: F,
      ) -> Result<Subscription<M>, RclrsError>
      where
          M: RosMessage,
          F: FnMut(&mut P, M) + Send + 'static;

      pub fn create_service<S, F>(
          &self,
          options: impl IntoServiceOptions<'_>,
          callback: F,
      ) -> Result<Service<S>, RclrsError>
      where
          S: RosService,
          F: FnMut(&mut P, S::Request) -> S::Response + Send + 'static;

      pub fn create_timer_repeating<F>(
          &self,
          period: Duration,
          callback: F,
      ) -> Result<Timer, RclrsError>
      where
          F: FnMut(&mut P) + Send + 'static;
  }
  ```

- [ ] **7.9.5** Add tests and examples

### Acceptance Criteria
- Worker shares state across callbacks
- Primitives created from worker receive payload in callbacks
- Thread-safe access to payload

---

## Phase 7.10: Async Support and Promise Type

**Goal**: Add async/await support with Promise<T> for async results.

### Tasks

- [ ] **7.10.1** Add `Promise<T>` type alias
  ```rust
  pub type Promise<T> = futures::channel::oneshot::Receiver<T>;
  ```

- [ ] **7.10.2** Add async service client methods
  ```rust
  impl<S: RosService> ServiceClient<S> {
      pub fn call_async(
          &self,
          request: &S::Request,
      ) -> Result<Promise<S::Response>, RclrsError>;
  }
  ```

- [ ] **7.10.3** Add async subscription callbacks
  ```rust
  impl Node {
      pub fn create_async_subscription<M, F, Fut>(
          &self,
          options: impl IntoSubscriberOptions<'_>,
          callback: F,
      ) -> Result<Subscription<M>, RclrsError>
      where
          M: RosMessage,
          F: FnMut(M) -> Fut + Send + 'static,
          Fut: Future<Output = ()> + Send;
  }
  ```

- [ ] **7.10.4** Add async service server
  ```rust
  impl Node {
      pub fn create_async_service<S, F, Fut>(
          &self,
          options: impl IntoServiceOptions<'_>,
          callback: F,
      ) -> Result<Service<S>, RclrsError>
      where
          S: RosService,
          F: FnMut(S::Request) -> Fut + Send + 'static,
          Fut: Future<Output = S::Response> + Send;
  }
  ```

- [ ] **7.10.5** Add async action client methods
  ```rust
  impl<A: RosAction> ActionClient<A> {
      pub async fn send_goal_async(&self, goal: &A::Goal) -> Result<GoalHandle, RclrsError>;
      pub async fn get_result_async(&self, goal_id: &GoalId) -> Result<A::Result, RclrsError>;
  }
  ```

- [ ] **7.10.6** Gate async behind feature flag
  ```toml
  [features]
  async = ["futures"]
  ```

- [ ] **7.10.7** Add tests with tokio/async-std

### Acceptance Criteria
- `Promise<T>` works for async results
- Async callbacks work with executor
- Feature-gated to avoid bloat in no_std

---

## Phase 7.11: Action API Improvements

**Goal**: Align action API with rclrs fluent patterns.

### Tasks

- [ ] **7.11.1** Add fluent action client API
  ```rust
  impl<A: RosAction> ActionClient<A> {
      pub fn request_goal(&self, goal: A::Goal) -> GoalRequestClient<A>;
      pub fn receive_feedback(&self, goal_id: GoalId) -> FeedbackClient<A>;
      pub fn receive_result(&self, goal_id: GoalId) -> ResultClient<A>;
      pub fn watch_status(&self, goal_id: GoalId) -> StatusWatcher<A>;
  }
  ```

- [ ] **7.11.2** Add goal cancellation
  ```rust
  impl<A: RosAction> ActionClient<A> {
      pub fn cancel_goal(&self, goal_id: GoalId) -> Result<CancelResponse, RclrsError>;
      pub fn cancel_all_goals(&self) -> Result<Vec<CancelResponse>, RclrsError>;
      pub fn cancel_goals_prior_to(&self, time: Time) -> Result<Vec<CancelResponse>, RclrsError>;
  }
  ```

- [ ] **7.11.3** Add `CancelResponseCode` enum
  ```rust
  pub enum CancelResponseCode {
      Accept,
      Reject,
      UnknownGoal,
      GoalTerminated,
  }
  ```

- [ ] **7.11.4** Add status watching
  ```rust
  pub struct StatusWatcher<A: RosAction> { ... }

  impl<A: RosAction> StatusWatcher<A> {
      pub fn try_recv(&mut self) -> Result<Option<GoalStatus>, RclrsError>;
      pub async fn recv(&mut self) -> Result<GoalStatus, RclrsError>;
  }
  ```

- [ ] **7.11.5** Update action server for async callbacks
  ```rust
  impl Node {
      pub fn create_action_server<A, F, Fut>(
          &self,
          name: &str,
          callback: F,
      ) -> Result<ActionServer<A>, RclrsError>
      where
          A: RosAction,
          F: FnMut(RequestedGoal<A>) -> Fut + Send + 'static,
          Fut: Future<Output = TerminatedGoal<A>> + Send;
  }
  ```

- [ ] **7.11.6** Add `RequestedGoal` and `TerminatedGoal` types
  ```rust
  pub struct RequestedGoal<A: RosAction> {
      pub goal_id: GoalId,
      pub goal: A::Goal,
  }

  pub struct TerminatedGoal<A: RosAction> {
      pub status: GoalStatus,
      pub result: A::Result,
  }
  ```

### Acceptance Criteria
- Fluent action client API works
- Goal cancellation implemented
- Status watching works
- Async action server callbacks supported

---

## Phase 7.12: Error Type Alignment

**Goal**: Rename and align error types with rclrs.

### Tasks

- [ ] **7.12.1** Rename `ConnectedNodeError` to `RclrsError`
  ```rust
  pub enum RclrsError {
      ConnectionFailed,
      PublisherCreationFailed,
      SubscriberCreationFailed,
      // ... etc
  }
  ```

- [ ] **7.12.2** Add `RclrsError::first_error()` helper
  ```rust
  impl RclrsError {
      pub fn first_error(errors: Vec<Self>) -> Result<(), Self>;
  }
  ```

- [ ] **7.12.3** Keep old error name as alias during transition
  ```rust
  #[deprecated(since = "0.2.0", note = "Use RclrsError instead")]
  pub type ConnectedNodeError = RclrsError;
  ```

- [ ] **7.12.4** Update all code to use new error type

### Acceptance Criteria
- `RclrsError` is the primary error type
- Old name still works with deprecation warning

---

## Implementation Schedule

| Phase | Description                | Status      | Estimated Effort | Dependencies  |
|-------|----------------------------|-------------|------------------|---------------|
| 7.1   | Context and Initialization | ✅ Complete | 1 week           | None          |
| 7.2   | QoS Builder Pattern        | ✅ Complete | 1 week           | 7.1           |
| 7.3   | Typed Parameters           | ✅ Complete | 1 week           | 7.1           |
| 7.4   | Timer API                  | ✅ Complete | 3 days           | 7.1           |
| 7.5   | Clock API                  | ✅ Complete | 3 days           | 7.1           |
| 7.6   | Unified Executor Model     | ✅ Complete | 2 weeks          | 7.1, 7.4, 7.5 |
| 7.7   | *(Merged into 7.6)*        | -           | -                | -             |
| 7.8   | Logger API                 | Pending     | 3 days           | 7.1           |
| 7.9   | Worker Pattern             | Pending     | 3 days           | 7.6           |
| 7.10  | Async Support              | Pending     | 1 week           | 7.6           |
| 7.11  | Action API Improvements    | Pending     | 1 week           | 7.10          |
| 7.12  | Error Type Alignment       | ✅ Complete | 2 days           | All above     |

**Notes:**
- Phase 7.7 (Callback Subscriptions) merged into Phase 7.6 (Unified Executor Model)
- Phase 7.6 effort increased from 1 week to 2 weeks to account for merged scope
- See `docs/architecture/unified-executor-design.md` for Phase 7.6 design details
- `spin_async()` deferred to Phase 7.10 (Async Support) due to zenoh-pico Send bounds
- Phase 7.12 (Error Types) complete - `RclrsError` implemented in `context.rs`

**Completed**: 7.1, 7.2, 7.3, 7.4, 7.5, 7.6, 7.12
**Pending**: 7.8 (Logger), 7.9 (Worker), 7.10 (Async), 7.11 (Actions)

**Total Estimated Effort**: ~8 weeks (7 phases complete, 4 pending)

---

## Migration Guide

### Before (nano-ros 0.1.x)
```rust
use nano_ros::prelude::*;

let config = NodeConfig::new("my_node", "/ns").with_domain(42);
let mut node = ConnectedNode::connect(config, "tcp/127.0.0.1:7447")?;

let pub = node.create_publisher::<Int32>("/topic")?;
let mut sub = node.create_subscriber::<Int32>("/topic")?;

loop {
    if let Some(msg) = sub.try_recv()? {
        process(msg);
    }
}
```

### After - Desktop (nano-ros 0.2.x with BasicExecutor)
```rust
use nano_ros::prelude::*;

let context = Context::from_env(InitOptions::default().with_domain_id(Some(42)))?;
let mut executor = context.create_basic_executor();
let node = executor.create_node("my_node".namespace("/ns"))?;

let pub = node.create_publisher::<Int32>("topic".keep_last(10))?;
let _sub = node.create_subscription("topic", |msg: Int32| {
    process(msg);
})?;

executor.spin(SpinOptions::default())?;
```

### After - RTIC/Embedded (nano-ros 0.2.x with PollingExecutor)
```rust
use nano_ros::prelude::*;

// In RTIC init task
let context = Context::new(InitOptions::new()
    .with_locator("tcp/192.0.2.2:7447"))?;
let mut executor: PollingExecutor<4> = context.create_polling_executor();
let node = executor.create_node("embedded_node")?;

let pub = node.create_publisher::<Int32>("sensor_data")?;
let _sub = node.create_subscription("commands", handle_command as fn(&Command))?;

// In RTIC periodic task (e.g., every 10ms)
#[task(priority = 2)]
async fn zenoh_poll(mut cx: zenoh_poll::Context) {
    loop {
        cx.shared.executor.lock(|exec| {
            exec.spin_once(10);  // 10ms delta
        });
        Mono::delay(10.millis()).await;
    }
}
```

### Generic Code (works with both executors)
```rust
fn setup_ros_node<E: Executor>(executor: &mut E) -> Result<NodeHandle<'_>, RclrsError> {
    let node = executor.create_node("generic_node")?;
    let _pub = node.create_publisher::<Int32>("topic")?;
    Ok(node)
}

// Called with spin_once() - works everywhere
fn process_messages<E: Executor>(executor: &mut E, delta_ms: u64) {
    let result = executor.spin_once(delta_ms);
    if result.subscriptions_processed > 0 {
        // Handle received messages
    }
}
```

---

## Testing Strategy

1. **Unit tests** for each new type/method
2. **Integration tests** comparing behavior with rclrs
3. **Backwards compatibility tests** ensuring old API still works
4. **Embedded tests** ensuring `no_std` still works
5. **Performance benchmarks** comparing with current implementation

---

## Success Criteria

1. nano-ros API matches rclrs 0.6.0 patterns
2. Existing code continues to work (with deprecation warnings)
3. `no_std` support maintained
4. All tests pass
5. Examples updated to new API
6. Migration guide complete
