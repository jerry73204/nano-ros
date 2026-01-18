# nano-ros RTIC Integration Design

## Overview

This document describes the design for integrating RTIC (Real-Time Interrupt-driven Concurrency) support into nano-ros, enabling formal real-time guarantees and static analysis capabilities for safety-critical embedded ROS 2 applications.

## Background

### What is RTIC?

RTIC is a hardware-accelerated RTOS for Rust that leverages the ARM Cortex-M NVIC (Nested Vectored Interrupt Controller) for zero-overhead task scheduling. Key properties:

- **Stack Resource Policy (SRP)**: Compile-time deadlock-free guarantees
- **Hardware scheduling**: Uses NVIC priority levels for preemption
- **Single shared stack**: All tasks share one stack, reducing memory usage
- **Async support**: RTIC 2.x requires all software tasks to be async
- **Static analysis**: Compatible with WCET analysis tools (RAUK, Symex/EASY)

### RTIC vs Embassy

| Feature          | RTIC                       | Embassy                          |
|------------------|----------------------------|----------------------------------|
| Scheduling       | Preemptive (hardware NVIC) | Cooperative + priority executors |
| Stack model      | Single shared stack (SRP)  | Per-task stacks                  |
| Deadlock freedom | Compile-time proof         | Not guaranteed                   |
| WCET analysis    | RAUK, Symex/EASY           | Limited tooling                  |
| Platform support | ARM Cortex-M only          | Multi-platform                   |
| Async model      | Required in v2             | Native                           |

### Why RTIC for nano-ros?

For autonomous vehicle and safety-critical applications:

1. **Formal guarantees**: SRP provides mathematically proven deadlock freedom
2. **WCET analysis**: Tools like RAUK can analyze worst-case execution time
3. **Certification path**: Static analysis supports safety certification (ISO 26262)
4. **Efficiency**: Hardware scheduling has zero software overhead
5. **Predictability**: Bounded priority inversion through ceiling protocol

## Current nano-ros Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    User Application                      │
├─────────────────────────────────────────────────────────┤
│                     nano-ros API                         │
│           (Node, Publisher, Subscriber, Service)         │
├─────────────────────────────────────────────────────────┤
│                  nano-ros-transport                      │
│              (ZenohTransport with spin::Mutex)           │
├─────────────────────────────────────────────────────────┤
│                    zenoh-pico                            │
│              (C library with background tasks)           │
├─────────────────────────────────────────────────────────┤
│                    Hardware / OS                         │
└─────────────────────────────────────────────────────────┘
```

### Compatibility Assessment

| Component          | RTIC Compatible | Notes                          |
|--------------------|-----------------|--------------------------------|
| nano-ros-core      | Yes             | Trait-based, no runtime deps   |
| nano-ros-serdes    | Yes             | Pure CDR serialization         |
| nano-ros-macros    | Yes             | Compile-time only              |
| nano-ros-transport | **No**          | Uses spin::Mutex (8 instances) |
| zenoh-pico         | **Partial**     | Spawns background threads      |

## Proposed Architecture

### Pluggable Executor Model

```
┌─────────────────────────────────────────────────────────┐
│                    User Application                      │
├─────────────────────────────────────────────────────────┤
│                     nano-ros API                         │
│         (Node, Publisher, Subscriber, Service)           │
├─────────────────────────────────────────────────────────┤
│                  Executor Abstraction                    │
├──────────┬──────────┬──────────┬──────────┬────────────┤
│   RTIC   │ Embassy  │  Zephyr  │  Tokio   │  Polling   │
│(Cortex-M)│ (multi)  │ (native) │ (Linux)  │  (simple)  │
├──────────┴──────────┴──────────┴──────────┴────────────┤
│                  nano-ros-transport                      │
│           (zenoh-pico with pluggable sync)               │
├─────────────────────────────────────────────────────────┤
│                    Hardware / OS                         │
└─────────────────────────────────────────────────────────┘
```

### RTIC Application Structure

Following RTIC patterns, a nano-ros application would look like:

```rust
#![no_std]
#![no_main]

use panic_halt as _;
use nano_ros::prelude::*;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [EXTI0, EXTI1, EXTI2])]
mod app {
    use super::*;
    use rtic_monotonics::systick::prelude::*;

    systick_monotonic!(Mono, 1000);

    #[shared]
    struct Shared {
        // Zenoh session - shared between poll and publish tasks
        zenoh_session: ZenohSession,
    }

    #[local]
    struct Local {
        // Publisher owned by sensor task
        sensor_publisher: ZenohPublisher,
        // Subscriber buffer owned by receiver task
        sensor_buffer: SubscriberBuffer,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        Mono::start(cx.core.SYST, 84_000_000);

        // Initialize zenoh session (allocations happen here, not at runtime)
        let mut session = ZenohSession::new_client("tcp/192.168.1.1:7447")
            .expect("Failed to connect");

        // Create publisher and subscriber
        let publisher = session.create_publisher::<SensorMsg>("/sensor/data")
            .expect("Failed to create publisher");
        let (subscriber, buffer) = session.create_subscriber::<SensorMsg>("/cmd/vel")
            .expect("Failed to create subscriber");

        // Spawn background tasks
        zenoh_poll::spawn().ok();
        zenoh_keepalive::spawn().ok();
        sensor_read::spawn().ok();

        (
            Shared { zenoh_session: session },
            Local {
                sensor_publisher: publisher,
                sensor_buffer: buffer,
            },
        )
    }

    /// Zenoh network polling - handles incoming messages
    /// Priority 1 (low) - can be preempted by sensor tasks
    #[task(priority = 1, shared = [zenoh_session])]
    async fn zenoh_poll(mut cx: zenoh_poll::Context) {
        loop {
            cx.shared.zenoh_session.lock(|session| {
                session.poll_read();
            });
            Mono::delay(10.millis()).await;
        }
    }

    /// Zenoh keepalive - maintains session liveness
    /// Priority 1 (low)
    #[task(priority = 1, shared = [zenoh_session])]
    async fn zenoh_keepalive(mut cx: zenoh_keepalive::Context) {
        loop {
            cx.shared.zenoh_session.lock(|session| {
                session.send_keepalive();
            });
            Mono::delay(1.secs()).await;
        }
    }

    /// High-priority sensor reading and publishing
    /// Priority 3 (high) - preempts network tasks
    #[task(priority = 3, local = [sensor_publisher], shared = [zenoh_session])]
    async fn sensor_read(cx: sensor_read::Context) {
        let publisher = cx.local.sensor_publisher;

        loop {
            // Read sensor (time-critical)
            let reading = read_imu_sensor();

            // Publish via zenoh
            publisher.publish(&SensorMsg {
                linear_acceleration: reading.accel,
                angular_velocity: reading.gyro,
            });

            Mono::delay(10.millis()).await;  // 100 Hz
        }
    }

    /// Hardware interrupt for motor control
    /// Priority 4 (highest) - hard real-time deadline
    #[task(binds = TIM2, priority = 4)]
    fn motor_control(_cx: motor_control::Context) {
        // Time-critical motor control
        // Runs to completion, preempts all software tasks
        update_motor_pwm();
    }
}
```

## Required Changes

### 1. Replace spin::Mutex with RTIC-Compatible Patterns

**Current (incompatible):**
```rust
// nano-ros-transport/src/zenoh.rs
pub struct ZenohPublisher {
    attachment: spin::Mutex<RmwAttachment>,
}
```

**RTIC-compatible options:**

#### Option A: Critical Sections (Simple)
```rust
#[cfg(feature = "rtic")]
use cortex_m::interrupt;

impl ZenohPublisher {
    pub fn publish_raw(&mut self, data: &[u8]) -> Result<(), Error> {
        let (seq, ts, gid) = interrupt::free(|_| {
            self.attachment.sequence_number += 1;
            (self.attachment.sequence_number, /* ... */)
        });
        // ... publish
    }
}
```

#### Option B: RTIC Shared Resources (Recommended)
```rust
// User's RTIC app manages the mutex via #[shared]
#[shared]
struct Shared {
    publisher: ZenohPublisher,  // No internal mutex needed
}

#[task(shared = [publisher])]
async fn publish_task(mut cx: publish_task::Context) {
    cx.shared.publisher.lock(|pub| {
        pub.publish(&msg);
    });
}
```

#### Option C: Lock-Free with Atomics
```rust
pub struct RmwAttachment {
    sequence_number: AtomicI64,  // Already atomic
    timestamp: AtomicI64,        // Already atomic
    gid: [AtomicU8; 16],         // Make atomic
}
```

### 2. Disable zenoh-pico Background Threads

**Current:**
```rust
// zenoh-pico/src/session.rs
fn start_tasks(&mut self) -> Result<()> {
    unsafe {
        zp_start_read_task(session, ptr::null());   // Spawns thread
        zp_start_lease_task(session, ptr::null());  // Spawns thread
    }
}
```

**RTIC-compatible:**
```rust
#[cfg(feature = "rtic")]
impl ZenohSession {
    /// Don't start background threads - user provides RTIC tasks
    pub fn new_without_tasks(config: Config) -> Result<Self> {
        // Initialize session but don't call zp_start_*_task
    }

    /// Poll for incoming data (called from RTIC task)
    pub fn poll_read(&mut self) {
        unsafe { zp_read(self.session, ptr::null()); }
    }

    /// Send keepalive (called from RTIC task)
    pub fn send_keepalive(&mut self) {
        unsafe { zp_send_keep_alive(self.session, ptr::null()); }
    }
}
```

### 3. Static Buffer Allocation

**Current (dynamic):**
```rust
pub struct ConnectedNode {
    _entity_tokens: Vec<LivelinessToken>,
}
```

**RTIC-compatible (static):**
```rust
pub struct ConnectedNode<const MAX_TOKENS: usize = 16> {
    _entity_tokens: heapless::Vec<LivelinessToken, MAX_TOKENS>,
}
```

### 4. Feature Flags

```toml
[package]
name = "nano-ros-transport"

[features]
default = ["std"]
std = ["nano-ros-serdes/std"]
alloc = ["nano-ros-serdes/alloc"]

# Executor backends
rtic = []           # RTIC-compatible (no spin::Mutex, no background threads)
embassy = []        # Embassy-compatible
polling = []        # Simple polling mode (no executor)

# Synchronization backends (mutually exclusive)
sync-spin = ["dep:spin"]        # Default: spin::Mutex
sync-critical-section = []      # cortex_m::interrupt::free
sync-rtic = ["rtic"]           # Use RTIC shared resources

# Transport
zenoh = ["dep:zenoh-pico", "alloc"]
```

## RTIC Integration Macro

To simplify integration, provide a macro that generates RTIC task boilerplate:

```rust
/// Macro to generate RTIC tasks for nano-ros
///
/// Usage:
/// ```rust
/// #[rtic::app(device = pac, dispatchers = [EXTI0, EXTI1])]
/// mod app {
///     nano_ros::rtic_tasks!(Mono, zenoh_session);
///
///     #[shared]
///     struct Shared {
///         zenoh_session: ZenohSession,
///     }
/// }
/// ```
#[macro_export]
macro_rules! rtic_tasks {
    ($mono:ty, $session:ident) => {
        /// Zenoh read polling task
        #[task(priority = 1, shared = [$session])]
        async fn nano_ros_poll(mut cx: nano_ros_poll::Context) {
            loop {
                cx.shared.$session.lock(|s| {
                    s.poll_read();
                });
                <$mono>::delay(10.millis()).await;
            }
        }

        /// Zenoh keepalive task
        #[task(priority = 1, shared = [$session])]
        async fn nano_ros_keepalive(mut cx: nano_ros_keepalive::Context) {
            loop {
                cx.shared.$session.lock(|s| {
                    s.send_keepalive();
                });
                <$mono>::delay(1.secs()).await;
            }
        }
    };
}
```

## Static Analysis Integration

### WCET Analysis with RAUK

RAUK (Response-time Analysis Using KLEE) can analyze RTIC applications:

```bash
# Install RAUK
cargo install rauk

# Analyze nano-ros RTIC application
rauk analyze --target thumbv7em-none-eabihf ./examples/rtic-sensor-node
```

Output includes:
- Per-task WCET estimates
- Response time analysis
- Schedulability verdict

### Symbolic Execution with Symex/EASY

```bash
# Run EASY analysis
easy analyze --arch armv7em ./examples/rtic-sensor-node
```

## Platform Support Matrix

| Platform     | Executor | WCET Analysis | Real-Time | Use Case                |
|--------------|----------|---------------|-----------|-------------------------|
| ARM Cortex-M | RTIC     | RAUK/Symex    | Hard      | Safety-critical control |
| ARM Cortex-M | Embassy  | Limited       | Soft      | General embedded        |
| RISC-V       | Embassy  | None          | Soft      | IoT devices             |
| Zephyr (any) | Native   | None          | Depends   | RTOS integration        |
| Linux        | Tokio    | None          | None      | Development             |

## Memory Layout

### RTIC Single-Stack Model

```
┌─────────────────────┐ High address
│                     │
│   Shared Stack      │ ← All tasks share this stack
│   (grows down)      │
│                     │
├─────────────────────┤
│                     │
│   Static Data       │ ← #[shared] and #[local] resources
│   (.data, .bss)     │
│                     │
├─────────────────────┤
│                     │
│   Code              │ ← .text section
│   (.text)           │
│                     │
└─────────────────────┘ Low address
```

### Resource Allocation Timeline

```
init() ────────────────────────────────────────────────────────►
    │
    ├── Allocate ZenohSession (heap)
    ├── Allocate Publisher buffers (heap)
    ├── Allocate Subscriber buffers (heap)
    ├── Initialize #[shared] resources
    ├── Initialize #[local] resources
    └── Return (Shared, Local)

═══════════════════════════════════════════════════════════════
    RTIC tasks run (no allocations)
═══════════════════════════════════════════════════════════════

task() ────────────────────────────────────────────────────────►
    │
    ├── Access #[local] resources (exclusive, no lock)
    ├── Access #[shared] resources (with lock)
    └── Publish/Subscribe (uses pre-allocated buffers)
```

## Example: Sensor Fusion Node

A complete example showing IMU sensor fusion with ROS 2 communication:

```rust
#![no_std]
#![no_main]

use panic_halt as _;
use nano_ros::prelude::*;
use sensor_msgs::msg::Imu;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3])]
mod app {
    use super::*;
    use rtic_monotonics::systick::prelude::*;
    use stm32f4xx_hal::{gpio::*, i2c::*, prelude::*};

    systick_monotonic!(Mono, 1000);

    #[shared]
    struct Shared {
        zenoh: ZenohSession,
        imu_data: ImuReading,
    }

    #[local]
    struct Local {
        i2c: I2c1,
        imu_publisher: ZenohPublisher,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // Hardware setup
        let dp = cx.device;
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

        Mono::start(cx.core.SYST, clocks.sysclk().raw());

        // I2C for IMU
        let gpiob = dp.GPIOB.split();
        let i2c = I2c::new(dp.I2C1, (gpiob.pb8, gpiob.pb9), 400.kHz(), &clocks);

        // Zenoh session
        let mut zenoh = ZenohSession::new_without_tasks(
            Config::client("tcp/192.168.1.1:7447")
        ).expect("Zenoh init failed");

        let publisher = zenoh.create_publisher::<Imu>("/imu/data").unwrap();

        // Spawn tasks
        nano_ros_poll::spawn().ok();
        nano_ros_keepalive::spawn().ok();
        imu_read::spawn().ok();
        imu_publish::spawn().ok();

        (
            Shared {
                zenoh,
                imu_data: ImuReading::default(),
            },
            Local { i2c, imu_publisher: publisher },
        )
    }

    // Include nano-ros background tasks
    nano_ros::rtic_tasks!(Mono, zenoh);

    /// IMU reading task - high priority, 1kHz
    #[task(priority = 3, local = [i2c], shared = [imu_data])]
    async fn imu_read(mut cx: imu_read::Context) {
        loop {
            // Read IMU via I2C (time-critical)
            let raw = read_mpu6050(cx.local.i2c);

            // Update shared data
            cx.shared.imu_data.lock(|data| {
                data.accel = raw.accel;
                data.gyro = raw.gyro;
                data.timestamp = Mono::now();
            });

            Mono::delay(1.millis()).await;  // 1kHz
        }
    }

    /// IMU publish task - medium priority, 100Hz
    #[task(priority = 2, local = [imu_publisher], shared = [imu_data, zenoh])]
    async fn imu_publish(mut cx: imu_publish::Context) {
        loop {
            // Read latest IMU data
            let reading = cx.shared.imu_data.lock(|d| d.clone());

            // Convert to ROS message
            let msg = Imu {
                header: Header {
                    stamp: reading.timestamp.into(),
                    frame_id: heapless::String::from("imu_link"),
                },
                linear_acceleration: reading.accel.into(),
                angular_velocity: reading.gyro.into(),
                ..Default::default()
            };

            // Publish
            cx.local.imu_publisher.publish(&msg);

            Mono::delay(10.millis()).await;  // 100Hz
        }
    }
}
```

## Migration Path

### Phase 1: Add Feature Flags (Week 1)
- Add `rtic`, `embassy`, `polling` features
- Add `sync-*` features for mutex selection
- Ensure existing code works with `default` features

### Phase 2: Replace spin::Mutex (Week 2)
- Implement critical section alternative
- Gate mutex choice behind features
- Test on native with `polling` feature

### Phase 3: Disable Background Threads (Week 3)
- Add `new_without_tasks()` constructor
- Add `poll_read()` and `send_keepalive()` methods
- Create RTIC task macro

### Phase 4: Static Buffers (Week 4)
- Replace `Vec` with `heapless::Vec`
- Add const generics for buffer sizes
- Document memory requirements

### Phase 5: Examples & Documentation (Week 5)
- Create RTIC example (STM32F4)
- Create Embassy example (multi-platform)
- Document WCET analysis workflow
- Add integration tests

## References

- [RTIC Book](https://rtic.rs/2/book/en/)
- [RTIC Repository](https://github.com/rtic-rs/rtic) (external/rtic)
- [Embassy Repository](https://github.com/embassy-rs/embassy) (external/embassy)
- [Stack Resource Policy](https://link.springer.com/article/10.1007/BF00365393)
- [RAUK Thesis](https://www.uppsatser.se/uppsats/3122363010/)
- [Symex/EASY Thesis](http://www.diva-portal.org/smash/get/diva2:1998467/FULLTEXT01.pdf)
- [zenoh-pico Documentation](https://zenoh-pico.readthedocs.io/)
