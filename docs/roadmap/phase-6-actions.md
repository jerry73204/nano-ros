# Phase 6: ROS 2 Actions

**Status: IN PROGRESS**

## Executive Summary

Phase 6 adds ROS 2 action support to nano-ros, enabling goal-based long-running tasks
with feedback and cancellation. Actions are essential for navigation, manipulation,
and other complex robotic behaviors.

**Goals:**
1. Define `RosAction` trait and associated types
2. Implement action server with goal state machine
3. Implement action client with async goal tracking
4. Generate action bindings via `cargo nano-ros generate`
5. Validate interop with ROS 2 action clients/servers

---

## 1. Background

### What Are ROS 2 Actions?

Actions extend services with:
- **Asynchronous execution** - Goals execute over time, not immediately
- **Feedback** - Progress updates during execution
- **Cancellation** - Ability to cancel in-progress goals
- **Result** - Final outcome when goal completes

### Action Communication Pattern

```
┌──────────────────┐                      ┌──────────────────┐
│   Action Client  │                      │   Action Server  │
│                  │                      │                  │
│  send_goal() ────┼─── Goal Request ────►│  accept_goal()   │
│                  │                      │       │          │
│                  │◄── Goal Response ────┼───────┘          │
│                  │    (accepted/rejected)                  │
│                  │                      │                  │
│  feedback_cb() ◄─┼──── Feedback ────────┼─ publish_feedback│
│  feedback_cb() ◄─┼──── Feedback ────────┼─ publish_feedback│
│                  │                      │                  │
│  cancel_goal() ──┼─── Cancel Request ──►│  handle_cancel() │
│                  │◄── Cancel Response ──┼───────┘          │
│                  │                      │                  │
│  get_result() ───┼─── Result Request ──►│                  │
│  result_cb() ◄───┼──── Result ──────────┼─ set_succeeded() │
└──────────────────┘                      └──────────────────┘
```

### Underlying Communication Channels

An action uses **5 separate communication primitives**:

| Channel | Type | Purpose |
|---------|------|---------|
| `~/_action/send_goal` | Service | Submit new goal |
| `~/_action/cancel_goal` | Service | Request cancellation |
| `~/_action/get_result` | Service | Retrieve final result |
| `~/_action/feedback` | Topic | Progress updates |
| `~/_action/status` | Topic | Goal state transitions |

---

## 2. Architecture

### 2.1 Core Traits

```rust
// crates/nano-ros-core/src/action.rs

/// Trait for ROS 2 action types
pub trait RosAction: Sized {
    /// Goal message sent by client
    type Goal: RosMessage;
    /// Result message returned on completion
    type Result: RosMessage;
    /// Feedback message sent during execution
    type Feedback: RosMessage;

    /// Action type name (e.g., "nav2_msgs::action::dds_::NavigateToPose_")
    const ACTION_NAME: &'static str;
    /// Type hash for discovery
    const ACTION_HASH: &'static str;
}

/// Goal status states (matches action_msgs/msg/GoalStatus)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GoalStatus {
    Unknown = 0,
    Accepted = 1,
    Executing = 2,
    Canceling = 3,
    Succeeded = 4,
    Canceled = 5,
    Aborted = 6,
}
```

### 2.2 Action Server

```rust
// crates/nano-ros-node/src/action_server.rs

/// Server for handling action goals
pub struct ActionServer<A: RosAction, const MAX_GOALS: usize = 4> {
    // Communication channels
    send_goal_service: ConnectedServiceServer<SendGoal<A>>,
    cancel_goal_service: ConnectedServiceServer<CancelGoal>,
    get_result_service: ConnectedServiceServer<GetResult<A>>,
    feedback_publisher: ConnectedPublisher<FeedbackMessage<A>>,
    status_publisher: ConnectedPublisher<GoalStatusArray>,

    // Goal tracking
    active_goals: heapless::Vec<GoalHandle<A>, MAX_GOALS>,
}

/// Handle to an accepted goal
pub struct GoalHandle<A: RosAction> {
    pub goal_id: GoalId,
    pub goal: A::Goal,
    pub status: GoalStatus,
    pub result: Option<A::Result>,
}

impl<A: RosAction, const MAX_GOALS: usize> ActionServer<A, MAX_GOALS> {
    /// Process incoming goal requests (call in main loop)
    pub fn spin_once(&mut self) -> Option<GoalHandle<A>>;

    /// Publish feedback for a goal
    pub fn publish_feedback(&mut self, goal_id: &GoalId, feedback: &A::Feedback) -> Result<()>;

    /// Mark goal as succeeded with result
    pub fn set_succeeded(&mut self, goal_id: &GoalId, result: A::Result) -> Result<()>;

    /// Mark goal as aborted with result
    pub fn set_aborted(&mut self, goal_id: &GoalId, result: A::Result) -> Result<()>;

    /// Mark goal as canceled
    pub fn set_canceled(&mut self, goal_id: &GoalId, result: A::Result) -> Result<()>;
}
```

### 2.3 Action Client

```rust
// crates/nano-ros-node/src/action_client.rs

/// Client for sending action goals
pub struct ActionClient<A: RosAction> {
    send_goal_client: ConnectedServiceClient<SendGoal<A>>,
    cancel_goal_client: ConnectedServiceClient<CancelGoal>,
    get_result_client: ConnectedServiceClient<GetResult<A>>,
    feedback_subscriber: ConnectedSubscriber<FeedbackMessage<A>>,
    status_subscriber: ConnectedSubscriber<GoalStatusArray>,
}

/// Handle to a sent goal
pub struct ClientGoalHandle<A: RosAction> {
    pub goal_id: GoalId,
    status: GoalStatus,
}

impl<A: RosAction> ActionClient<A> {
    /// Send a goal (blocking until accepted/rejected)
    pub fn send_goal(&mut self, goal: &A::Goal) -> Result<ClientGoalHandle<A>>;

    /// Request cancellation of a goal
    pub fn cancel_goal(&mut self, goal_handle: &ClientGoalHandle<A>) -> Result<CancelResponse>;

    /// Get the result of a completed goal (blocking)
    pub fn get_result(&mut self, goal_handle: &ClientGoalHandle<A>) -> Result<A::Result>;

    /// Check for feedback (non-blocking)
    pub fn try_recv_feedback(&mut self) -> Option<(GoalId, A::Feedback)>;

    /// Check goal status (non-blocking)
    pub fn get_status(&self, goal_handle: &ClientGoalHandle<A>) -> GoalStatus;
}
```

### 2.4 Node API

```rust
// crates/nano-ros-node/src/connected.rs

impl<const MAX_TOKENS: usize> ConnectedNode<MAX_TOKENS> {
    /// Create an action server
    pub fn create_action_server<A: RosAction>(
        &mut self,
        action_name: &str,
    ) -> Result<ActionServer<A>, Error>;

    /// Create an action server with custom goal capacity
    pub fn create_action_server_sized<A: RosAction, const MAX_GOALS: usize>(
        &mut self,
        action_name: &str,
    ) -> Result<ActionServer<A, MAX_GOALS>, Error>;

    /// Create an action client
    pub fn create_action_client<A: RosAction>(
        &mut self,
        action_name: &str,
    ) -> Result<ActionClient<A>, Error>;
}
```

---

## 3. Required Message Types

### 3.1 From action_msgs Package

```
action_msgs/msg/GoalInfo
├── goal_id: unique_identifier_msgs/UUID
└── stamp: builtin_interfaces/Time

action_msgs/msg/GoalStatus
├── goal_info: GoalInfo
└── status: int8

action_msgs/msg/GoalStatusArray
└── status_list: GoalStatus[]

action_msgs/srv/CancelGoal
├── Request:
│   └── goal_info: GoalInfo
└── Response:
    ├── return_code: int8
    └── goals_canceling: GoalInfo[]
```

### 3.2 From unique_identifier_msgs Package

```
unique_identifier_msgs/msg/UUID
└── uuid: uint8[16]
```

### 3.3 Generated Per-Action Types

For each action `Foo`:

```
FooAction/
├── Foo_Goal           (user-defined goal fields)
├── Foo_Result         (user-defined result fields)
├── Foo_Feedback       (user-defined feedback fields)
├── Foo_SendGoal       (service: Goal → Response)
│   ├── Request: goal_id + goal
│   └── Response: accepted + stamp
├── Foo_GetResult      (service: goal_id → Result)
│   ├── Request: goal_id
│   └── Response: status + result
└── Foo_FeedbackMessage (topic message)
    ├── goal_id: UUID
    └── feedback: Foo_Feedback
```

---

## 4. Work Items

### 4.0 Critical Path to Zephyr Action Examples

The following dependency chain must be completed to enable Zephyr action examples:

```
4.1 Core Action Types (no dependencies)
 │
 ├──► 4.2 Required Message Bindings
 │     │
 │     └──► 4.3 Action Code Generation
 │           │
 │           ├──► 4.4 Action Server Implementation
 │           │     │
 │           │     └──► 4.6a Native Action Server Example
 │           │           │
 │           │           └──► 4.6b Zephyr Action Server Example
 │           │
 │           └──► 4.5 Action Client Implementation
 │                 │
 │                 └──► 4.6a Native Action Client Example
 │                       │
 │                       └──► 4.6b Zephyr Action Client Example
 │
 └──► [Zephyr Service Support - COMPLETE]
```

**Minimum viable path for Zephyr actions:**
1. 4.1 Core Action Types - **COMPLETE**
2. 4.2 Message Bindings - **COMPLETE**
3. 4.3 Code Generation - **COMPLETE** (basic), needs service wrappers
4. 4.4 Action Server - **COMPLETE** (all services implemented)
5. 4.5 Action Client - **COMPLETE** (all services implemented)
6. 4.6a Native Examples (~1 day)
7. 4.6b Zephyr Examples (~1 day)

### 4.1 Core Action Types

**Status:** COMPLETE
**Dependencies:** None
**Enables:** 4.2, 4.4, 4.5

- [x] Create `crates/nano-ros-core/src/action.rs`
- [x] Define `RosAction` trait with Goal/Result/Feedback associated types
- [x] Define `GoalStatus` enum (matches action_msgs/msg/GoalStatus)
- [x] Define `GoalId` type (UUID wrapper, 16 bytes)
- [x] Define `GoalInfo` struct (goal_id + stamp)
- [x] Define `GoalStatusStamped` struct (goal_info + status)
- [x] Add `is_terminal()` helper for GoalStatus
- [x] Add `is_active()` helper for GoalStatus
- [x] Implement Serialize/Deserialize for GoalId, GoalStatus, GoalInfo, GoalStatusStamped
- [x] Add action module to `nano-ros-core/src/lib.rs`
- [x] Re-export from `nano-ros` crate prelude
- [x] Unit tests for GoalStatus state transitions (15 tests pass)

### 4.2 Required Message Bindings

**Status:** COMPLETE
**Dependencies:** 4.1
**Enables:** 4.3

- [x] Add `action_msgs` to native-rs-action-server package.xml
- [x] Add `unique_identifier_msgs` to native-rs-action-server package.xml
- [x] Run `cargo nano-ros generate` to produce bindings
- [x] Verify GoalInfo serialization (goal_id: UUID, stamp: Time)
- [x] Verify GoalStatus serialization (goal_info + status: i8)
- [x] Verify GoalStatusArray serialization (status_list: sequence)
- [x] Verify CancelGoal service types (Request/Response)
- [x] Test UUID serialization matches ROS 2 wire format
- [ ] Copy generated bindings to Zephyr examples

### 4.3 Action Code Generation

**Status:** COMPLETE
**Dependencies:** 4.2
**Enables:** 4.4, 4.5

- [x] Verify `rosidl-parser` parses `.action` files correctly
- [x] Create action template in rosidl-codegen
- [x] Generate wrapper `FooAction` struct implementing `RosAction`
- [x] Generate `Foo_Goal` message type from action goal section
- [x] Generate `Foo_Result` message type from action result section
- [x] Generate `Foo_Feedback` message type from action feedback section
- [ ] Generate `Foo_SendGoal` service type (Request: goal_id + goal, Response: accepted + stamp)
- [ ] Generate `Foo_GetResult` service type (Request: goal_id, Response: status + result)
- [ ] Generate `Foo_FeedbackMessage` message (goal_id + feedback)
- [x] Update `cargo-nano-ros` CLI to handle action files
- [x] Test generation with `example_interfaces/action/Fibonacci`
- [x] Verify generated types compile with nano-ros

### 4.4 Action Server Implementation

**Status:** COMPLETE
**Dependencies:** 4.1, 4.3
**Enables:** 4.6a (native server example)

- [x] Define `ConnectedActionServer<A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>` struct
- [x] Create internal service: send_goal
- [x] Create internal service: cancel_goal
- [x] Create internal service: get_result
- [x] Create internal publisher: feedback
- [x] Create internal publisher: status
- [x] Define `ActiveGoal<A>` for tracking active goals
- [x] Define `CompletedGoal<A>` for storing results
- [x] Implement goal acceptance via `try_accept_goal()`
- [x] Implement goal state machine (ACCEPTED → EXECUTING → terminal)
- [x] Implement `publish_feedback(&mut self, goal_id, feedback)`
- [x] Implement `set_goal_status(&mut self, goal_id, status)`
- [x] Implement `complete_goal(&mut self, goal_id, status, result)`
- [x] Implement `try_handle_cancel()` for cancel requests
- [x] Implement `try_handle_get_result()` for result queries
- [x] Implement `publish_status()` for status publishing
- [x] Add `create_action_server()` to `ConnectedNode`
- [x] Add `create_action_server_sized()` for custom buffer/goal limits
- [ ] Unit tests for goal state machine
- [ ] Integration test with mock client

### 4.5 Action Client Implementation

**Status:** COMPLETE
**Dependencies:** 4.1, 4.3
**Enables:** 4.6a (native client example)

- [x] Define `ConnectedActionClient<A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>` struct
- [x] Create internal service client: send_goal
- [x] Create internal service client: cancel_goal
- [x] Create internal service client: get_result
- [x] Create internal subscriber: feedback
- [x] Create internal subscriber: status
- [x] Define `GoalHandle` for tracking sent goals
- [x] Implement `send_goal(&mut self, goal) -> Result<GoalHandle>`
- [x] Implement `try_recv_feedback(&mut self) -> Option<(GoalId, Feedback)>`
- [x] Implement `cancel_goal(&mut self, goal_id) -> Result<CancelResponse>`
- [x] Implement `get_result(&mut self, goal_id) -> Result<(GoalStatus, A::Result)>`
- [x] Implement `try_recv_status()` for status updates
- [x] Implement `get_goal_status()` helper
- [x] Add `create_action_client()` to `ConnectedNode`
- [x] Add `create_action_client_sized()` for custom buffer limits
- [ ] Unit tests for client operations
- [ ] Integration test with mock server

### 4.6 Examples

#### 4.6a Native Examples

**Status:** Scaffolding complete, awaiting action core
**Dependencies:** 4.4, 4.5
**Enables:** 4.6b, 4.7

- [x] Create `examples/native-rs-action-server/` directory structure
- [x] Create `examples/native-rs-action-client/` directory structure
- [x] Add package.xml with `example_interfaces` dependency
- [ ] Implement Fibonacci action server using `ActionServer<Fibonacci>`
- [ ] Implement Fibonacci action client using `ActionClient<Fibonacci>`
- [ ] Test native server ↔ native client communication
- [ ] Document example usage in README

#### 4.6b Zephyr Examples

**Status:** Blocked on action core (4.1-4.5)
**Dependencies:** 4.6a (native examples working first), Zephyr service support (COMPLETE)
**Enables:** Embedded action-based applications

- [ ] Create `examples/zephyr-rs-action-server/` directory structure
  - [ ] CMakeLists.txt with zenoh shim
  - [ ] prj.conf with queryable + publication support
  - [ ] Cargo.toml with zenoh-pico-shim dependency
  - [ ] Copy generated example_interfaces bindings
- [ ] Create `examples/zephyr-rs-action-client/` directory structure
  - [ ] CMakeLists.txt with zenoh shim
  - [ ] prj.conf with query + subscription support
  - [ ] Cargo.toml with zenoh-pico-shim dependency
  - [ ] Copy generated example_interfaces bindings
- [ ] Implement ZephyrActionServer wrapper over zenoh-pico-shim
  - [ ] Use `ShimQueryable` for send_goal, cancel_goal, get_result services
  - [ ] Use `ShimPublisher` for feedback, status topics
- [ ] Implement ZephyrActionClient wrapper over zenoh-pico-shim
  - [ ] Use `ShimContext::get()` for service calls
  - [ ] Use `ShimSubscriber` for feedback, status topics
- [ ] Test on native_sim/native/64 target
- [ ] Test Zephyr server ↔ native client
- [ ] Test native server ↔ Zephyr client
- [ ] Test on real hardware (NUCLEO-F429ZI)

**Prerequisites (all COMPLETE):**
- [x] Zephyr service support (Phase 3.1.6)
- [x] zenoh-pico queryable C shim (`zenoh_shim_declare_queryable`)
- [x] zenoh-pico query C shim (`zenoh_shim_get`)
- [x] Rust FFI wrappers in zenoh-pico-shim

### 4.7 Integration Tests

- [ ] nano-ros server ↔ nano-ros client
- [ ] nano-ros server ↔ ROS 2 client (`ros2 action send_goal`)
- [ ] ROS 2 server ↔ nano-ros client
- [ ] Test goal cancellation
- [ ] Test multiple concurrent goals
- [ ] Test feedback streaming

### 4.8 Documentation

- [ ] Add action section to CLAUDE.md
- [ ] Document action API in rustdoc
- [ ] Add action examples to user guide
- [ ] Document limitations vs rclcpp/rclpy

---

## 5. Goal State Machine

```
                    ┌─────────────┐
        send_goal() │             │
       ────────────►│  ACCEPTED   │
                    │             │
                    └──────┬──────┘
                           │
                    execute()
                           │
                           ▼
                    ┌─────────────┐
                    │             │◄──────────────┐
                    │  EXECUTING  │               │
                    │             │───────────────┤
                    └──────┬──────┘  cancel_goal()│
                           │                      │
           ┌───────────────┼───────────────┐      │
           │               │               │      │
    set_succeeded()  set_aborted()         │      │
           │               │               ▼      │
           ▼               ▼        ┌─────────────┐
    ┌─────────────┐ ┌─────────────┐ │             │
    │             │ │             │ │  CANCELING  │
    │  SUCCEEDED  │ │   ABORTED   │ │             │
    │             │ │             │ └──────┬──────┘
    └─────────────┘ └─────────────┘        │
                                    set_canceled()
                                           │
                                           ▼
                                    ┌─────────────┐
                                    │             │
                                    │  CANCELED   │
                                    │             │
                                    └─────────────┘
```

---

## 6. Key Expression Format

Action endpoints follow rmw_zenoh naming:

```
# Send goal service
<domain>/<action_name>/_action/send_goal/<type>/RIHS01_<hash>

# Cancel goal service
<domain>/<action_name>/_action/cancel_goal/action_msgs::srv::dds_::CancelGoal_/RIHS01_<hash>

# Get result service
<domain>/<action_name>/_action/get_result/<type>/RIHS01_<hash>

# Feedback topic
<domain>/<action_name>/_action/feedback/<type>/RIHS01_<hash>

# Status topic
<domain>/<action_name>/_action/status/action_msgs::msg::dds_::GoalStatusArray_/RIHS01_<hash>
```

---

## 7. Memory Considerations

### Static Allocation

```rust
// Configure maximum concurrent goals at compile time
let server: ActionServer<MyAction, 4> = node.create_action_server_sized("/my_action")?;

// Default is 4 concurrent goals
let server: ActionServer<MyAction> = node.create_action_server("/my_action")?;
```

### Memory per Goal

| Component | Size (approximate) |
|-----------|-------------------|
| GoalId (UUID) | 16 bytes |
| GoalStatus | 1 byte |
| Goal message | varies |
| Result message | varies |
| GoalHandle overhead | ~32 bytes |

For `MAX_GOALS = 4` with typical messages: ~500 bytes to 2KB total.

---

## 8. Limitations vs rclcpp/rclpy

| Feature | rclcpp/rclpy | nano-ros |
|---------|--------------|----------|
| Concurrent goals | Unlimited | Compile-time limit |
| Async send_goal | Yes | No (blocking only) |
| Goal callbacks | Full (accepted, feedback, result) | Polling-based |
| Goal handle lifetime | RAII | Manual tracking |
| Executor integration | Yes | Manual spin |

---

## 9. Dependencies

**Requires (completed):**
- Phase 3: Services (for goal/cancel/result services)
- Phase 4: Message generation (for action type generation)

**Enables:**
- nav2 integration (NavigateToPose, FollowPath, etc.)
- moveit integration (MoveGroup, Execute)
- Any long-running robotic task

---

## 10. Acceptance Criteria

### Action Server Complete When:
- [ ] `ros2 action send_goal` works with nano-ros server
- [ ] Feedback streams to ROS 2 client
- [ ] Cancellation works correctly
- [ ] Multiple concurrent goals supported

### Action Client Complete When:
- [ ] nano-ros client can call ROS 2 action servers
- [ ] Feedback received correctly
- [ ] Result retrieved after completion
- [ ] Cancellation request works

### Code Generation Complete When:
- [ ] `cargo nano-ros generate` produces action types
- [ ] Generated types compile without errors
- [ ] CDR serialization matches ROS 2 wire format

---

## 11. Implementation Order

```
1. Core Types (4.1)
   └── RosAction trait, GoalStatus, GoalId

2. Message Bindings (4.2)
   └── action_msgs, unique_identifier_msgs

3. Code Generation (4.3)
   └── action_nano_ros.rs.jinja template

4. Action Server (4.4)
   └── Goal state machine, feedback, results

5. Action Client (4.5)
   └── send_goal, cancel, get_result

6. Examples & Tests (4.6, 4.7)
   └── Fibonacci example, interop tests

7. Documentation (4.8)
   └── API docs, user guide
```

---

## 12. References

- [ROS 2 Actions Design](https://design.ros2.org/articles/actions.html)
- [action_msgs Package](https://github.com/ros2/rcl_interfaces/tree/master/action_msgs)
- [ROS 2 Action Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [rclcpp Action Implementation](https://github.com/ros2/rclcpp/tree/master/rclcpp_action)
