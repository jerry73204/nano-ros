# API Comparison: rclrs 0.6.0 vs nano-ros

This document compares the rclrs 0.6.0 API with nano-ros to identify differences and guide refactoring efforts to align nano-ros with the rclrs API patterns.

## Executive Summary

| Aspect | rclrs 0.6.0 | nano-ros | Recommendation |
|--------|-------------|----------|----------------|
| Node type | `Arc<NodeState>` (type alias `Node`) | `ConnectedNode<MAX_TOKENS>` | Consider using `Node` type alias |
| Context | Separate `Context` type | Implicit in `ConnectedNode` | Add explicit `Context` |
| Executor | `Executor` with spin model | Manual polling or background threads | Add `Executor` abstraction |
| Pub/Sub creation | Via node methods with builder pattern | Via node methods with QoS parameter | Adopt builder pattern |
| Callbacks | Closure-based | Poll-based (try_recv) | Consider adding callback support |
| Async | Full async/await support | Synchronous only | Add async support |
| Parameters | Typed parameters with builders | Basic ParameterServer | Adopt typed parameter pattern |
| Worker | Worker<Payload> for state sharing | Not available | Add Worker pattern |
| Promise | futures::oneshot for async results | Not available | Add Promise type |

---

## 1. Initialization and Context

### rclrs 0.6.0
```rust
// Explicit context creation
let context = Context::default_from_env()?;
let context = Context::from_env(InitOptions::default())?;

// InitOptions for configuration
let options = InitOptions::default()
    .with_domain_id(Some(42));
let context = Context::new(args, options)?;

// Executor created from context
let executor = context.create_basic_executor();

// Nodes created from executor
let node = executor.create_node("my_node")?;
let node = executor.create_node("my_node".namespace("/ns"))?;
```

### nano-ros
```rust
// Direct node creation (no explicit context)
let config = NodeConfig::new("my_node", "/ns")
    .with_domain(42);
let node = ConnectedNode::connect(config, "tcp/127.0.0.1:7447")?;

// Or peer mode
let node = ConnectedNode::connect_peer(config)?;
```

### Differences
1. **No separate Context**: nano-ros combines context and node
2. **No Executor**: nano-ros uses manual polling or background threads
3. **Transport embedded**: nano-ros requires transport locator at node creation
4. **No InitOptions**: Configuration is in NodeConfig

### Recommendation
- Add `Context` type that holds shared state
- Add `Executor` trait/struct for spin model
- Keep transport config separate from node identity config

---

## 2. Node Creation and Configuration

### rclrs 0.6.0
```rust
// String automatically converted to NodeOptions
let node = executor.create_node("my_node")?;

// Builder pattern via string extensions
let node = executor.create_node(
    "my_node"
    .namespace("/my_namespace")
)?;

// Node methods
node.name()                    // After remapping
node.namespace()               // After remapping
node.fully_qualified_name()
node.domain_id()
node.get_clock()
node.logger()
```

### nano-ros
```rust
// Separate config object
let config = NodeConfig::new("my_node", "/my_namespace")
    .with_domain(42);
let node = ConnectedNode::connect(config, locator)?;

// Node methods
node.name()
node.namespace()
node.fully_qualified_name()
node.domain_id()
// No clock API
// No logger API
```

### Differences
1. **No string-to-options conversion**: nano-ros uses explicit NodeConfig
2. **No Clock API**: nano-ros lacks clock abstraction
3. **No Logger API**: nano-ros uses external logging (env_logger)
4. **No graph callbacks**: nano-ros lacks notify_on_graph_change

### Recommendation
- Implement `IntoNodeOptions` trait for string conversion
- Add `Clock` type and `node.get_clock()` method
- Add `Logger` type and severity-specific logging methods
- Add graph change notification API

---

## 3. Publisher API

### rclrs 0.6.0
```rust
// Creation with builder pattern
let pub = node.create_publisher::<MyMsg>("topic")?;
let pub = node.create_publisher::<MyMsg>(
    "topic"
    .keep_last(100)
    .transient_local()
    .reliable()
)?;

// Publishing
pub.publish(&msg)?;
pub.publish(msg)?;  // MessageCow allows both

// Methods
pub.topic_name()
pub.get_subscription_count()
pub.notify_on_subscriber_ready()
pub.borrow_loaned_message()  // Zero-copy
```

### nano-ros
```rust
// Creation with separate QoS
let pub = node.create_publisher::<MyMsg>("/topic")?;
let pub = node.create_publisher_with_qos::<MyMsg>("/topic", QosSettings::BEST_EFFORT)?;

// Publishing
pub.publish(&msg)?;
pub.publish_raw(&bytes)?;
pub.publish_with_buffer(&msg, &mut buf)?;

// No topic_name() method
// No subscription_count() method
// No graph notifications
```

### Differences
1. **No builder pattern for QoS**: nano-ros uses separate method
2. **No MessageCow**: nano-ros only accepts references
3. **No introspection**: Missing topic_name, subscription_count
4. **No graph notifications**: Missing notify_on_subscriber_ready
5. **No zero-copy**: Missing loaned messages

### Recommendation
- Implement QoS builder pattern via string extension traits
- Add `MessageCow` trait for flexible message passing
- Add `topic_name()`, `get_subscription_count()` methods
- Consider zero-copy support for performance

---

## 4. Subscriber API

### rclrs 0.6.0
```rust
// Callback-based
let sub = node.create_subscription(
    "topic",
    |msg: MyMsg| {
        println!("Received: {:?}", msg);
    }
)?;

// With worker for state sharing
let worker = node.create_worker(MyData::default());
let sub = worker.create_subscription(
    "topic",
    |data: &mut MyData, msg: MyMsg| {
        data.process(&msg);
    }
)?;

// Async callbacks
let sub = node.create_subscription(
    "topic",
    |msg: MyMsg| async move {
        // Async processing
    }
)?;

// Methods
sub.topic_name()
sub.set_callback(...)  // Runtime callback change
```

### nano-ros
```rust
// Poll-based only
let mut sub = node.create_subscriber::<MyMsg>("/topic")?;

// Manual polling
match sub.try_recv()? {
    Some(msg) => process(msg),
    None => { /* no message */ }
}

// Raw receive
let len = sub.try_recv_raw(&mut buf)?;

// No callback support
// No async support
// No worker pattern
```

### Differences
1. **Poll-based vs callback-based**: nano-ros uses polling, rclrs uses callbacks
2. **No Worker pattern**: nano-ros lacks state sharing abstraction
3. **No async callbacks**: nano-ros is synchronous only
4. **No runtime callback change**: nano-ros callbacks are fixed

### Recommendation
- Add callback-based subscription API alongside polling
- Implement Worker<Payload> pattern
- Add async callback support
- Add `set_callback()` for runtime changes

---

## 5. Service API

### rclrs 0.6.0
```rust
// Server with various callback signatures
let srv = node.create_service::<MySvc, _>(
    "service",
    |req: Request| -> Response { ... }
)?;

// With request ID
let srv = node.create_service::<MySvc, _>(
    "service",
    |req: Request, id: RequestId| -> Response { ... }
)?;

// With ServiceInfo
let srv = node.create_service::<MySvc, _>(
    "service",
    |req: Request, info: ServiceInfo| -> Response { ... }
)?;

// Async variant
let srv = node.create_async_service::<MySvc, _>(
    "service",
    |req| async move { Response { ... } }
)?;

// Client with Promise
let client = node.create_client::<MySvc>("service")?;
let promise: Promise<Response> = client.call(&request)?;
let response = promise.await?;

// Various output types
let (response, id) = client.call(request)?.await?;
let (response, info) = client.call(request)?.await?;
```

### nano-ros
```rust
// Server with handler closure
let mut server = node.create_service::<MySvc>("/service")?;
server.handle_request(|req| Response { ... })?;

// Or manual receive
if let Some((len, seq_num)) = server.try_recv_request()? {
    // Manual handling
}

// Client - blocking call
let mut client = node.create_client::<MySvc>("/service")?;
let response = client.call(&request)?;  // Blocks

// No async
// No Promise type
// No request ID in response
```

### Differences
1. **No async service**: nano-ros is synchronous
2. **No Promise type**: nano-ros blocks on call
3. **No ServiceInfo in callbacks**: Missing request metadata
4. **Simpler callback signatures**: nano-ros only supports basic form

### Recommendation
- Add async service support
- Implement Promise<T> return type
- Add ServiceInfo to callback signatures
- Support multiple callback signatures

---

## 6. Action API

### rclrs 0.6.0
```rust
// Client with fluent API
let client = node.create_action_client::<MyAction>("action")?;
let goal_client = client.request_goal(goal)?;
let feedback_client = client.receive_feedback(goal_id)?;
let result_client = client.receive_result(goal_id)?;
let status_watcher = client.watch_status(goal_id)?;

// Cancellation
client.cancel_goal(goal_id)?;
client.cancel_all_goals()?;
client.cancel_goals_prior_to(time)?;

// Server with async callback
let server = node.create_action_server::<MyAction, _>(
    "action",
    |requested_goal: RequestedGoal<A>| async {
        TerminatedGoal { result, status }
    }
)?;

// Or goal receiver pattern
let receiver = node.create_goal_receiver::<MyAction>("action")?;
```

### nano-ros
```rust
// Client - basic goal sending
let mut client = node.create_action_client::<MyAction>("/action")?;
let handle = client.send_goal(&goal)?;
if let Some((goal_id, feedback)) = client.try_recv_feedback()? { ... }

// Server - manual goal handling
let mut server = node.create_action_server::<MyAction>("/action")?;
if let Some(goal_id) = server.try_accept_goal(|goal| GoalResponse::Accept)? {
    server.publish_feedback(&goal_id, &feedback)?;
    server.set_goal_status(&goal_id, GoalStatus::Executing)?;
    server.complete_goal(&goal_id, GoalStatus::Succeeded, &result)?;
}
```

### Differences
1. **Less fluent client API**: nano-ros is more manual
2. **No status watcher**: Missing watch_status
3. **No cancel API**: Missing cancel_goal, cancel_all_goals
4. **No async server callback**: nano-ros is synchronous
5. **No RequestedGoal/TerminatedGoal**: Different abstraction

### Recommendation
- Add fluent client API (request_goal, receive_feedback, etc.)
- Add status watching
- Implement goal cancellation
- Add async server callbacks
- Consider RequestedGoal/TerminatedGoal patterns

---

## 7. QoS Settings

### rclrs 0.6.0
```rust
pub struct QoSProfile {
    pub history: QoSHistoryPolicy,      // KeepLast/KeepAll/SystemDefault
    pub reliability: QoSReliabilityPolicy,  // Reliable/BestEffort
    pub durability: QoSDurabilityPolicy,    // Volatile/TransientLocal
    pub deadline: QoSDuration,
    pub lifespan: QoSDuration,
    pub liveliness: QoSLivelinessPolicy,
    pub liveliness_lease: QoSDuration,
    pub avoid_ros_namespace_conventions: bool,
}

// Builder methods
profile.keep_last(100).reliable().transient_local()

// Predefined profiles
QoS_PROFILE_DEFAULT
QoS_PROFILE_SENSOR_DATA
QoS_PROFILE_SERVICES_DEFAULT
```

### nano-ros
```rust
pub struct QosSettings {
    pub reliable: bool,
    pub history_depth: u8,
}

impl QosSettings {
    pub const BEST_EFFORT: Self = ...;
    pub const RELIABLE: Self = ...;
}
```

### Differences
1. **Much simpler QoS**: nano-ros only has reliability and depth
2. **No deadline/lifespan/liveliness**: Missing advanced QoS
3. **No predefined profiles**: Only BEST_EFFORT and RELIABLE
4. **No builder pattern**: Direct struct construction

### Recommendation
- Extend QosSettings with more fields
- Add builder pattern methods
- Add predefined profile constants
- Consider compatibility layer for embedded constraints

---

## 8. Parameters

### rclrs 0.6.0
```rust
// Typed parameters with builder
let param: MandatoryParameter<i32> = node
    .declare_parameter("my_param")
    .default(42)
    .description("What this does")
    .range(IntegerRange { from: 0, to: 100, step: 1 })
    .mandatory()?;

let opt: OptionalParameter<Arc<str>> = node
    .declare_parameter("opt_param")
    .optional()?;

let readonly: ReadOnlyParameter<f64> = node
    .declare_parameter("readonly")
    .read_only()?;

// Access
let value = param.get();
param.set(new_value)?;
```

### nano-ros
```rust
// Basic parameter server
let mut server = ParameterServer::new();
server.set_parameter("my_param", ParameterValue::Integer(42))?;
server.set_parameter_with_descriptor("my_param", value, descriptor)?;

if let Some(param) = server.get_parameter("my_param") {
    if let ParameterValue::Integer(v) = param.value { ... }
}
```

### Differences
1. **No typed parameters**: nano-ros uses ParameterValue enum
2. **No builder pattern**: Direct method calls
3. **No parameter categories**: Missing Mandatory/Optional/ReadOnly
4. **Separate server**: Not integrated into Node

### Recommendation
- Add typed parameter API (MandatoryParameter<T>, etc.)
- Implement parameter builder pattern
- Integrate parameters into Node
- Add declare_parameter() method

---

## 9. Timer API

### rclrs 0.6.0
```rust
let timer = node.create_timer_repeating(Duration::from_millis(100), || { ... })?;
let timer = node.create_timer_oneshot(Duration::from_secs(5), || { ... })?;
let timer = node.create_timer_inert(Duration::from_millis(100))?;

timer.cancel()?;
timer.reset()?;
timer.is_ready()?;
timer.time_until_next_call()?;
```

### nano-ros
- **No Timer API**

### Recommendation
- Add Timer type with create_timer_repeating/oneshot/inert
- Integrate with executor spin model

---

## 10. Executor and Spinning

### rclrs 0.6.0
```rust
// Blocking spin
executor.spin(SpinOptions::default())?;

// Spin until condition
executor.spin(SpinOptions::until_promise_resolved(promise))?;
executor.spin(SpinOptions::max_iterations(100))?;

// Async spin
let (executor, errors) = executor.spin_async(options).await;

// ExecutorCommands during spin
commands.create_node(...)?;
commands.halt_spinning();
commands.query(async { ... })?;
commands.run(async { ... })?;
```

### nano-ros
```rust
// Manual polling (no executor)
loop {
    node.poll_read()?;  // ~10ms
    // Process messages
}

// Or background threads (automatic)
let node = ConnectedNode::connect(config, locator)?;
// Background threads automatically started
```

### Differences
1. **No Executor type**: nano-ros uses manual polling
2. **No spin model**: User implements their own loop
3. **No SpinOptions**: No control over spin behavior
4. **No ExecutorCommands**: No runtime node management

### Recommendation
- Add Executor abstraction with spin/spin_async
- Implement SpinOptions for spin control
- Add ExecutorCommands for runtime operations
- Keep manual polling as alternative for embedded

---

## 11. Worker Pattern

### rclrs 0.6.0
```rust
let worker: Worker<MyData> = node.create_worker(MyData::default());

// Run task with payload access
worker.run(|data: &mut MyData| { ... })?;

// Listen to activity
worker.listen(|data| { ... });
worker.listen_until(|data| -> bool { data.done });

// Create primitives from worker
let sub = worker.create_subscription("topic", |data, msg| { ... })?;
let srv = worker.create_service("srv", |data, req| { ... })?;
```

### nano-ros
- **No Worker pattern**

### Recommendation
- Implement Worker<Payload> type
- Allow creating primitives that share worker state
- Add run/listen/listen_until methods

---

## 12. Async Support

### rclrs 0.6.0
- Full async/await with futures
- Promise<T> for async results
- Async callbacks for subscriptions/services
- spin_async for non-blocking execution

### nano-ros
- Synchronous only
- Blocking calls
- Poll-based message reception

### Recommendation
- Add Promise<T> type (could use futures::oneshot)
- Add async method variants (_async suffix)
- Implement spin_async
- Consider async_trait for callbacks

---

## 13. Logging

### rclrs 0.6.0
```rust
node.logger()
node.debug()  // LoggerHandle
node.info()
node.warn()
node.error()
node.fatal()

log!(logger.once(), "Print once");
log!(logger.throttle(Duration::from_secs(5)), "Throttled");
log_warn!(node.warn().skip_first(), "Skip first");
```

### nano-ros
```rust
// Uses external crate
use log::{info, error, warn};
env_logger::init();

info!("Message");
```

### Differences
1. **No integrated logging**: nano-ros uses log crate
2. **No throttling/once/skip_first**: Missing advanced features
3. **No node-scoped logger**: No per-node logging

### Recommendation
- Add Logger type with severity methods
- Implement throttle/once/skip_first modifiers
- Create log macros that accept logger
- Keep log crate compatibility

---

## 14. Type System Differences

### Node Type
| rclrs | nano-ros | Recommendation |
|-------|----------|----------------|
| `type Node = Arc<NodeState>` | `ConnectedNode<MAX_T>` | Add `Node` alias |

### Return Types
| rclrs | nano-ros | Recommendation |
|-------|----------|----------------|
| `Arc<PublisherState<T>>` | `ConnectedPublisher<T>` | Consider Arc wrapper |
| `Arc<Subscription<T>>` | `ConnectedSubscriber<T>` | Consider Arc wrapper |
| `Promise<T>` | `Result<T>` (blocking) | Add Promise type |

### Error Types
| rclrs | nano-ros | Recommendation |
|-------|----------|----------------|
| `RclrsError` | `ConnectedNodeError` | Rename to `RclrsError`? |

---

## 15. Priority Refactoring Items

### High Priority (API Compatibility)
1. **Add Context type** - Central initialization
2. **Add Executor with spin model** - Standard ROS pattern
3. **QoS builder pattern** - String extension traits
4. **Typed parameters** - MandatoryParameter<T>, etc.
5. **Promise<T> type** - Async result handling

### Medium Priority (Functionality)
6. **Callback-based subscriptions** - Alongside polling
7. **Timer API** - Repeating/oneshot/inert timers
8. **Clock API** - RosTime/SteadyTime/SystemTime
9. **Logger API** - Per-node logging with modifiers
10. **Worker pattern** - State sharing

### Lower Priority (Advanced Features)
11. **Async support** - async/await integration
12. **Graph callbacks** - Topology change notifications
13. **Zero-copy** - Loaned messages
14. **Action fluent API** - request_goal, receive_feedback, etc.
15. **Service metadata** - ServiceInfo in callbacks

---

## 16. Backwards Compatibility Strategy

To maintain compatibility with existing nano-ros users while adopting rclrs patterns:

1. **Keep existing API** - Don't remove current methods
2. **Add new API alongside** - New methods follow rclrs patterns
3. **Deprecation warnings** - Mark old API as deprecated
4. **Migration guide** - Document how to update code
5. **Feature flags** - Enable new API incrementally

Example:
```rust
// Old API (deprecated but still works)
let pub = node.create_publisher::<M>("/topic")?;

// New API (rclrs-style)
let pub = node.create_publisher::<M>("topic".keep_last(10).reliable())?;
```

---

## 17. Embedded Considerations

When adopting rclrs patterns, maintain no_std compatibility:

1. **Avoid alloc for core types** - Use heapless where possible
2. **Const generics for buffers** - Keep compile-time sizing
3. **Optional async** - Behind feature flag
4. **Executor flexibility** - Support both spin and manual polling
5. **Memory footprint** - Arc/Mutex add overhead

Example approach:
```rust
#[cfg(feature = "std")]
pub type Node = Arc<NodeState>;

#[cfg(not(feature = "std"))]
pub type Node<'a> = &'a mut NodeState;
```
