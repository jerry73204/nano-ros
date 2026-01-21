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

- [ ] **7.1.1** Create `Context` struct in `nano-ros-node`
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

- [ ] **7.1.2** Create `InitOptions` struct
  ```rust
  pub struct InitOptions {
      domain_id: Option<u32>,
  }

  impl InitOptions {
      pub fn new() -> Self;
      pub fn with_domain_id(self, domain_id: Option<u32>) -> Self;
  }
  ```

- [ ] **7.1.3** Add `Context::create_node()` method (without executor initially)
  ```rust
  impl Context {
      pub fn create_node<'a>(&self, options: impl IntoNodeOptions<'a>)
          -> Result<Node, RclrsError>;
  }
  ```

- [ ] **7.1.4** Implement `IntoNodeOptions` trait for string conversion
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

- [ ] **7.1.5** Add `NodeOptions` builder with namespace method
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

- [ ] **7.1.6** Add `Node` type alias
  ```rust
  #[cfg(feature = "std")]
  pub type Node = Arc<ConnectedNode>;

  #[cfg(not(feature = "std"))]
  pub type Node<'a, const N: usize = DEFAULT_MAX_TOKENS> = &'a mut ConnectedNode<N>;
  ```

- [ ] **7.1.7** Deprecate direct `ConnectedNode::connect()` in favor of `Context`
  ```rust
  #[deprecated(since = "0.2.0", note = "Use Context::create_node() instead")]
  pub fn connect(...) -> ...
  ```

- [ ] **7.1.8** Update examples to use new Context API

- [ ] **7.1.9** Add tests for Context initialization

### Acceptance Criteria
- Context can be created from environment or options
- Nodes can be created via `context.create_node("name".namespace("/ns"))`
- Old API still works but shows deprecation warnings
- All existing tests pass

---

## Phase 7.2: QoS Builder Pattern

**Goal**: Enable fluent QoS configuration via string/topic extension traits.

### Tasks

- [ ] **7.2.1** Extend `QosSettings` with more fields
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

- [ ] **7.2.2** Add QoS builder methods
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

- [ ] **7.2.3** Add predefined QoS profiles
  ```rust
  pub const QOS_PROFILE_DEFAULT: QosSettings = ...;
  pub const QOS_PROFILE_SENSOR_DATA: QosSettings = ...;
  pub const QOS_PROFILE_SERVICES_DEFAULT: QosSettings = ...;
  pub const QOS_PROFILE_PARAMETERS: QosSettings = ...;
  ```

- [ ] **7.2.4** Create `IntoPrimitiveOptions` trait for topic strings
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

- [ ] **7.2.5** Implement string-to-options conversion
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

- [ ] **7.2.6** Update `create_publisher` to accept options
  ```rust
  pub fn create_publisher<M: RosMessage>(
      &mut self,
      options: impl IntoPublisherOptions<'_>,
  ) -> Result<Publisher<M>, RclrsError>;
  ```

- [ ] **7.2.7** Apply same pattern to subscribers, services, actions

- [ ] **7.2.8** Deprecate `create_publisher_with_qos()` methods

- [ ] **7.2.9** Update examples and tests

### Acceptance Criteria
- `node.create_publisher::<M>("topic".keep_last(10).reliable())` works
- Predefined QoS profiles available
- Old `_with_qos` methods deprecated
- Same pattern works for all primitives

---

## Phase 7.3: Typed Parameters

**Goal**: Replace untyped `ParameterValue` with typed parameter API.

### Tasks

- [ ] **7.3.1** Create `ParameterVariant` trait
  ```rust
  pub trait ParameterVariant: Clone {
      fn to_parameter_value(&self) -> ParameterValue;
      fn from_parameter_value(value: &ParameterValue) -> Option<Self>;
  }

  // Implement for: bool, i64, f64, String, Vec<i64>, Vec<f64>, Vec<bool>, Vec<String>
  ```

- [ ] **7.3.2** Create `ParameterBuilder` struct
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

- [ ] **7.3.3** Create typed parameter types
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

- [ ] **7.3.4** Add `declare_parameter` to Node
  ```rust
  impl Node {
      pub fn declare_parameter<'a, T: ParameterVariant>(
          &'a self,
          name: &'a str,
      ) -> ParameterBuilder<'a, T>;
  }
  ```

- [ ] **7.3.5** Create `ParameterRange` types
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

- [ ] **7.3.6** Add `use_undeclared_parameters()` for dynamic access
  ```rust
  impl Node {
      pub fn use_undeclared_parameters(&self) -> UndeclaredParameters<'_>;
  }
  ```

- [ ] **7.3.7** Keep `ParameterServer` for `no_std` environments

- [ ] **7.3.8** Update examples and tests

### Acceptance Criteria
- Typed parameters work: `node.declare_parameter("speed").default(5.0).mandatory()?`
- Range validation enforced
- Read-only parameters cannot be set
- `no_std` still supported via `ParameterServer`

---

## Phase 7.4: Timer API

**Goal**: Add timer support matching rclrs patterns.

### Tasks

- [ ] **7.4.1** Create `Timer` struct
  ```rust
  pub struct Timer {
      period: Duration,
      callback: Option<Box<dyn FnMut() + Send>>,
      last_call: Instant,
      canceled: bool,
  }
  ```

- [ ] **7.4.2** Add timer creation methods to Node
  ```rust
  impl Node {
      pub fn create_timer_repeating<F>(
          &self,
          period: Duration,
          callback: F,
      ) -> Result<Timer, RclrsError>
      where
          F: FnMut() + Send + 'static;

      pub fn create_timer_oneshot<F>(
          &self,
          delay: Duration,
          callback: F,
      ) -> Result<Timer, RclrsError>
      where
          F: FnMut() + Send + 'static;

      pub fn create_timer_inert(
          &self,
          period: Duration,
      ) -> Result<Timer, RclrsError>;
  }
  ```

- [ ] **7.4.3** Add Timer methods
  ```rust
  impl Timer {
      pub fn cancel(&self) -> Result<(), RclrsError>;
      pub fn is_canceled(&self) -> bool;
      pub fn reset(&self) -> Result<(), RclrsError>;
      pub fn is_ready(&self) -> bool;
      pub fn time_until_next_call(&self) -> Duration;
      pub fn time_since_last_call(&self) -> Duration;
      pub fn get_timer_period(&self) -> Duration;

      pub fn set_callback<F>(&self, callback: F)
      where
          F: FnMut() + Send + 'static;
      pub fn set_repeating<F>(&self, callback: F);
      pub fn set_oneshot<F>(&self, callback: F);
      pub fn set_inert(&self);
  }
  ```

- [ ] **7.4.4** Integrate timers with executor (Phase 7.6)

- [ ] **7.4.5** Add `no_std` variant using const generics
  ```rust
  #[cfg(not(feature = "std"))]
  pub struct Timer<const MAX_TIMERS: usize> { ... }
  ```

- [ ] **7.4.6** Add tests and examples

### Acceptance Criteria
- Repeating timers fire at specified interval
- One-shot timers fire once
- Timers can be canceled and reset
- Callbacks can be changed at runtime

---

## Phase 7.5: Clock API

**Goal**: Add clock abstraction for time sources.

### Tasks

- [ ] **7.5.1** Create `ClockType` enum
  ```rust
  pub enum ClockType {
      RosTime,      // Simulation time (can be paused/scaled)
      SystemTime,   // Wall clock
      SteadyTime,   // Monotonic clock
  }
  ```

- [ ] **7.5.2** Create `Clock` struct
  ```rust
  pub struct Clock {
      clock_type: ClockType,
      // Internal state
  }

  impl Clock {
      pub fn new(kind: ClockType) -> Self;
      pub fn system() -> Self;
      pub fn steady() -> Self;
      pub fn clock_type(&self) -> ClockType;
      pub fn now(&self) -> Time;
  }
  ```

- [ ] **7.5.3** Extend `Time` type with more operations
  ```rust
  impl Time {
      pub fn from_nanos(nanos: i64) -> Self;
      pub fn to_nanos(&self) -> i64;
      pub fn as_duration(&self) -> Duration;
      pub fn from_duration(d: Duration) -> Self;
  }

  impl Add<Duration> for Time { ... }
  impl Sub<Duration> for Time { ... }
  impl Sub<Time> for Time { type Output = Duration; ... }
  ```

- [ ] **7.5.4** Add `get_clock()` to Node
  ```rust
  impl Node {
      pub fn get_clock(&self) -> &Clock;
  }
  ```

- [ ] **7.5.5** Add ROS time override support
  ```rust
  impl Clock {
      pub fn set_ros_time_override(&self, nanos: i64);
      pub fn clear_ros_time_override(&self);
  }
  ```

- [ ] **7.5.6** Add tests for clock operations

### Acceptance Criteria
- System and steady clocks work
- ROS time can be overridden for simulation
- Time arithmetic works correctly
- Nodes have associated clock

---

## Phase 7.6: Executor and Spin Model

**Goal**: Add executor abstraction with spin/spin_async support.

### Tasks

- [ ] **7.6.1** Create `Executor` struct
  ```rust
  pub struct Executor {
      context: Context,
      nodes: Vec<Node>,
      // Internal state
  }
  ```

- [ ] **7.6.2** Add executor creation from Context
  ```rust
  impl Context {
      pub fn create_basic_executor(&self) -> Executor;
  }
  ```

- [ ] **7.6.3** Add node creation through executor
  ```rust
  impl Executor {
      pub fn create_node<'a>(
          &mut self,
          options: impl IntoNodeOptions<'a>,
      ) -> Result<Node, RclrsError>;
  }
  ```

- [ ] **7.6.4** Create `SpinOptions` struct
  ```rust
  pub struct SpinOptions {
      max_iterations: Option<usize>,
      timeout: Option<Duration>,
      // Other conditions
  }

  impl SpinOptions {
      pub fn default() -> Self;
      pub fn max_iterations(self, count: usize) -> Self;
      pub fn timeout(self, duration: Duration) -> Self;
  }
  ```

- [ ] **7.6.5** Add blocking spin
  ```rust
  impl Executor {
      pub fn spin(&mut self, options: SpinOptions) -> Vec<RclrsError>;
  }
  ```

- [ ] **7.6.6** Add async spin (behind feature flag)
  ```rust
  #[cfg(feature = "async")]
  impl Executor {
      pub async fn spin_async(
          self,
          options: SpinOptions,
      ) -> (Self, Vec<RclrsError>);
  }
  ```

- [ ] **7.6.7** Create `ExecutorCommands` for runtime operations
  ```rust
  pub struct ExecutorCommands {
      // ...
  }

  impl ExecutorCommands {
      pub fn halt_spinning(&self);
      pub fn context(&self) -> &Context;
  }
  ```

- [ ] **7.6.8** Keep manual polling API for embedded
  ```rust
  impl Node {
      pub fn poll(&mut self) -> Result<(), RclrsError>;
  }
  ```

- [ ] **7.6.9** Update examples to use executor

### Acceptance Criteria
- `executor.spin()` processes all callbacks
- `spin_async` works with async runtime
- Manual polling still available
- SpinOptions control spin behavior

---

## Phase 7.7: Callback-based Subscriptions

**Goal**: Add callback-based subscription API alongside polling.

### Tasks

- [ ] **7.7.1** Add callback subscription creation
  ```rust
  impl Node {
      pub fn create_subscription<M, F>(
          &self,
          options: impl IntoSubscriberOptions<'_>,
          callback: F,
      ) -> Result<Subscription<M>, RclrsError>
      where
          M: RosMessage,
          F: FnMut(M) + Send + 'static;
  }
  ```

- [ ] **7.7.2** Support multiple callback signatures
  ```rust
  // Just message
  |msg: M| { ... }

  // Message with info (future)
  |msg: M, info: MessageInfo| { ... }
  ```

- [ ] **7.7.3** Add `set_callback` for runtime changes
  ```rust
  impl<M: RosMessage> Subscription<M> {
      pub fn set_callback<F>(&self, callback: F)
      where
          F: FnMut(M) + Send + 'static;
  }
  ```

- [ ] **7.7.4** Keep polling API as alternative
  ```rust
  impl<M: RosMessage> Subscription<M> {
      pub fn try_recv(&mut self) -> Result<Option<M>, RclrsError>;
  }
  ```

- [ ] **7.7.5** Integrate callbacks with executor spin

- [ ] **7.7.6** Add tests for callback subscriptions

### Acceptance Criteria
- Callback-based subscriptions work with executor
- Callbacks can be changed at runtime
- Polling still works for manual control
- Both patterns coexist

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

| Phase | Description | Estimated Effort | Dependencies |
|-------|-------------|------------------|--------------|
| 7.1 | Context and Initialization | 1 week | None |
| 7.2 | QoS Builder Pattern | 1 week | 7.1 |
| 7.3 | Typed Parameters | 1 week | 7.1 |
| 7.4 | Timer API | 3 days | 7.1 |
| 7.5 | Clock API | 3 days | 7.1 |
| 7.6 | Executor and Spin | 1 week | 7.1, 7.4 |
| 7.7 | Callback Subscriptions | 3 days | 7.6 |
| 7.8 | Logger API | 3 days | 7.1 |
| 7.9 | Worker Pattern | 3 days | 7.6, 7.7 |
| 7.10 | Async Support | 1 week | 7.6 |
| 7.11 | Action API Improvements | 1 week | 7.10 |
| 7.12 | Error Type Alignment | 2 days | All above |

**Total Estimated Effort**: ~8 weeks

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

### After (nano-ros 0.2.x with rclrs-style API)
```rust
use nano_ros::prelude::*;

let context = Context::from_env(InitOptions::default().with_domain_id(Some(42)))?;
let executor = context.create_basic_executor();
let node = executor.create_node("my_node".namespace("/ns"))?;

let pub = node.create_publisher::<Int32>("topic".keep_last(10))?;
let _sub = node.create_subscription("topic", |msg: Int32| {
    process(msg);
})?;

executor.spin(SpinOptions::default())?;
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
