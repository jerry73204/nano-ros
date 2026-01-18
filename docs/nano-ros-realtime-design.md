# nano-ros Real-Time Executor Integration Design

## Overview

This document outlines the design for integrating real-time execution frameworks into nano-ros, enabling formal scheduling analysis and real-time guarantees while maintaining cross-platform portability.

## Background

### nano-ros Current Architecture

nano-ros is a lightweight ROS 2 client library for embedded systems with:

- `no_std` compatible design for bare-metal and RTOS targets
- ROS 2 interoperability via zenoh/rmw_zenoh
- Zephyr RTOS and native Linux support
- Zero-copy CDR serialization

### Real-Time Requirements

For autonomous vehicle applications, certain components require:

- Bounded worst-case execution time (WCET)
- Deterministic scheduling
- Priority-based preemption
- Formal schedulability analysis

## Real-Time Framework Options

### RTIC (Real-Time Interrupt-driven Concurrency)

RTIC is a hardware-accelerated RTOS for Rust that provides:

- **Stack Resource Policy (SRP)**: Compile-time deadlock-free guarantees
- **Hardware scheduling**: Uses NVIC for zero-overhead task dispatch
- **Async support**: RTIC 2.0 requires all software tasks to be async
- **Analysis tools**: RAUK, Symex/EASY for WCET and schedulability analysis

#### Platform Support

| Architecture | Status |
|--------------|--------|
| ARM Cortex-M | ✅ Full support |
| RISC-V | ⚠️ Experimental |
| Others | ❌ Not supported |

#### Async Programming Model

RTIC 2.0 fully supports Rust async/await:

```rust
#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [EXTI0])]
mod app {
    use rtic_monotonics::systick::prelude::*;
    
    systick_monotonic!(Mono, 1000);

    #[task(priority = 2)]
    async fn sensor_task(_cx: sensor_task::Context) {
        loop {
            let data = read_sensor();
            Mono::delay(10.millis()).await;
        }
    }
}
```

Key characteristics:
- Software tasks must be `async fn`
- Each code section between `await` points is treated as an individual SRP task
- Compiler rejects `await` while holding a resource lock (preserves SRP LIFO requirement)

### Embassy

Embassy is an async embedded framework providing:

- **Cooperative multitasking**: Compile-time state machine transformation
- **Multi-priority executors**: Higher priority tasks can preempt lower ones
- **Broad platform support**: Cortex-M, RISC-V, ESP32, etc.
- **Integrated HALs**: embassy-stm32, embassy-nrf, embassy-rp

#### Platform Support

| Architecture | Status |
|--------------|--------|
| ARM Cortex-M | ✅ Full support |
| RISC-V | ✅ Good support |
| ESP32 (Xtensa/RISC-V) | ✅ Via esp-hal |
| AVR | ✅ Supported |

### Comparison

| Feature | RTIC | Embassy |
|---------|------|---------|
| Scheduling | Preemptive (hardware) | Cooperative + priority executors |
| Async support | ✅ Required in v2 | ✅ Native |
| WCET analysis | ✅ RAUK, Symex/EASY | ⚠️ Limited |
| Deadlock-free | ✅ Compile-time proof | ⚠️ Not guaranteed |
| Platform support | Cortex-M only | Multi-platform |
| Ecosystem integration | Separate HALs | Integrated HALs |

## zenoh-pico Integration Considerations

### zenoh-pico Architecture

zenoh-pico (the C library underlying nano-ros transport) provides:

- **Single-thread mode**: `Z_FEATURE_MULTI_THREAD=OFF` disables internal threading
- **Manual polling API**: `zp_read()`, `zp_send_keep_alive()`, `zp_send_join()`
- **Non-blocking I/O**: Uses I/O multiplexing for connection handling
- **Bounded operations**: Polling functions are designed for bounded execution

### Integration Pattern

zenoh-pico's polling API maps well to periodic RTIC/Embassy tasks:

```rust
// Periodic zenoh polling task
#[task(priority = 1, shared = [zenoh_session])]
async fn zenoh_poll(mut cx: zenoh_poll::Context) {
    loop {
        cx.shared.zenoh_session.lock(|session| {
            zp_read(session, ptr::null());
            zp_send_keep_alive(session, ptr::null());
        });
        Mono::delay(10.millis()).await;
    }
}
```

## Recommended Architecture

### Design Principle: Pluggable Executors

To maintain portability while enabling real-time guarantees, nano-ros should use an abstracted executor model:

```
┌─────────────────────────────────────────────────────────┐
│                    User Application                      │
├─────────────────────────────────────────────────────────┤
│                     nano-ros API                         │
│         (Node, Publisher, Subscriber, Service)           │
├─────────────────────────────────────────────────────────┤
│                  Executor Abstraction                    │
├──────────┬──────────┬──────────┬──────────┬────────────┤
│   RTIC   │ Embassy  │  Zephyr  │  Tokio   │  FreeRTOS  │
│(Cortex-M)│ (multi)  │ (native) │ (Linux)  │  (future)  │
├──────────┴──────────┴──────────┴──────────┴────────────┤
│                  nano-ros-transport                      │
│              (zenoh-pico / zenoh-rust)                   │
├─────────────────────────────────────────────────────────┤
│                    Hardware / OS                         │
└─────────────────────────────────────────────────────────┘
```

### Project Structure

```
nano-ros/
├── crates/
│   ├── nano-ros-core/
│   │   ├── src/
│   │   │   ├── lib.rs
│   │   │   ├── node.rs
│   │   │   ├── publisher.rs
│   │   │   ├── subscriber.rs
│   │   │   └── executor/
│   │   │       ├── mod.rs           # Executor trait
│   │   │       ├── rtic.rs          # RTIC backend
│   │   │       ├── embassy.rs       # Embassy backend
│   │   │       ├── zephyr.rs        # Zephyr native
│   │   │       └── tokio.rs         # Linux/std
│   │   └── Cargo.toml
│   ├── nano-ros-transport/
│   ├── nano-ros-serdes/
│   └── nano-ros-types/
└── examples/
    ├── rtic-example/
    ├── embassy-example/
    └── zephyr-example/
```

### Cargo Feature Flags

```toml
[package]
name = "nano-ros-core"
version = "0.1.0"
edition = "2021"

[features]
default = []

# Executor backends (mutually exclusive for main executor)
std = ["dep:tokio"]
rtic = ["dep:rtic", "dep:rtic-monotonics", "dep:rtic-sync"]
embassy = ["dep:embassy-executor", "dep:embassy-time", "dep:embassy-sync"]
zephyr = []

# Transport backends
zenoh-pico = ["nano-ros-transport/zenoh-pico"]
zenoh-rust = ["nano-ros-transport/zenoh-rust", "std"]

[dependencies]
# Core dependencies (always included)
nano-ros-transport = { path = "../nano-ros-transport" }
nano-ros-serdes = { path = "../nano-ros-serdes" }

# RTIC backend
rtic = { version = "2", optional = true }
rtic-monotonics = { version = "2", optional = true }
rtic-sync = { version = "1", optional = true }

# Embassy backend
embassy-executor = { version = "0.6", optional = true }
embassy-time = { version = "0.3", optional = true }
embassy-sync = { version = "0.6", optional = true }

# Tokio backend (Linux/std)
tokio = { version = "1", features = ["rt", "time", "sync"], optional = true }
```

### Executor Trait

```rust
// src/executor/mod.rs

use core::future::Future;
use core::time::Duration;

/// Trait for nano-ros executor backends
pub trait Executor {
    /// Error type for spawn operations
    type SpawnError;
    
    /// Handle returned when spawning a task
    type TaskHandle;

    /// Spawn a new task
    fn spawn<F>(&self, future: F) -> Result<Self::TaskHandle, Self::SpawnError>
    where
        F: Future<Output = ()> + Send + 'static;
}

/// Trait for time-based operations
pub trait DelayProvider {
    /// Delay for the specified duration
    fn delay(duration: Duration) -> impl Future<Output = ()>;
    
    /// Delay until the specified instant
    fn delay_until(instant: Self::Instant) -> impl Future<Output = ()>;
    
    /// Associated instant type
    type Instant: Copy;
    
    /// Get current time
    fn now() -> Self::Instant;
}

// Re-export appropriate backend based on features
#[cfg(feature = "rtic")]
mod rtic_impl;
#[cfg(feature = "rtic")]
pub use rtic_impl::*;

#[cfg(feature = "embassy")]
mod embassy_impl;
#[cfg(feature = "embassy")]
pub use embassy_impl::*;

#[cfg(feature = "std")]
mod tokio_impl;
#[cfg(feature = "std")]
pub use tokio_impl::*;
```

### RTIC Backend Implementation

```rust
// src/executor/rtic_impl.rs

use rtic_monotonics::Monotonic;

/// Macro to generate RTIC tasks for nano-ros
/// 
/// Usage in user's RTIC app:
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
        /// Zenoh polling task - handles network I/O
        #[task(priority = 1, shared = [$session])]
        async fn nano_ros_poll(mut cx: nano_ros_poll::Context) {
            loop {
                cx.shared.$session.lock(|s| {
                    $crate::transport::poll_session(s);
                });
                <$mono>::delay(10.millis()).await;
            }
        }
        
        /// Zenoh keepalive task - maintains session liveness
        #[task(priority = 1, shared = [$session])]
        async fn nano_ros_keepalive(mut cx: nano_ros_keepalive::Context) {
            loop {
                cx.shared.$session.lock(|s| {
                    $crate::transport::send_keepalive(s);
                });
                <$mono>::delay(1000.millis()).await;
            }
        }
    };
}

/// RTIC-specific node wrapper with priority configuration
pub struct RticNode<const PRIORITY: u8> {
    // Node internals
}

impl<const PRIORITY: u8> RticNode<PRIORITY> {
    pub fn new(name: &str) -> Self {
        Self { /* ... */ }
    }
}
```

### Embassy Backend Implementation

```rust
// src/executor/embassy_impl.rs

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

pub type SharedSession = Mutex<CriticalSectionRawMutex, ZenohSession>;

/// Spawn nano-ros background tasks on Embassy executor
pub fn spawn_nano_ros_tasks(
    spawner: &Spawner,
    session: &'static SharedSession,
) -> Result<(), embassy_executor::SpawnError> {
    spawner.spawn(nano_ros_poll_task(session))?;
    spawner.spawn(nano_ros_keepalive_task(session))?;
    Ok(())
}

#[embassy_executor::task]
async fn nano_ros_poll_task(session: &'static SharedSession) {
    loop {
        {
            let mut s = session.lock().await;
            crate::transport::poll_session(&mut s);
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[embassy_executor::task]
async fn nano_ros_keepalive_task(session: &'static SharedSession) {
    loop {
        {
            let mut s = session.lock().await;
            crate::transport::send_keepalive(&mut s);
        }
        Timer::after(Duration::from_secs(1)).await;
    }
}

/// Embassy delay provider implementation
pub struct EmbassyDelay;

impl super::DelayProvider for EmbassyDelay {
    type Instant = embassy_time::Instant;
    
    fn delay(duration: Duration) -> impl Future<Output = ()> {
        Timer::after(Duration::from_micros(duration.as_micros() as u64))
    }
    
    fn delay_until(instant: Self::Instant) -> impl Future<Output = ()> {
        Timer::at(instant)
    }
    
    fn now() -> Self::Instant {
        embassy_time::Instant::now()
    }
}
```

### Tokio Backend Implementation (Linux Development)

```rust
// src/executor/tokio_impl.rs

use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::time::{Duration, interval};

pub type SharedSession = Arc<Mutex<ZenohSession>>;

/// Spawn nano-ros background tasks on Tokio runtime
pub async fn spawn_nano_ros_tasks(session: SharedSession) {
    let poll_session = session.clone();
    let keepalive_session = session.clone();
    
    tokio::spawn(async move {
        let mut interval = interval(Duration::from_millis(10));
        loop {
            interval.tick().await;
            let mut s = poll_session.lock().await;
            crate::transport::poll_session(&mut s);
        }
    });
    
    tokio::spawn(async move {
        let mut interval = interval(Duration::from_secs(1));
        loop {
            interval.tick().await;
            let mut s = keepalive_session.lock().await;
            crate::transport::send_keepalive(&mut s);
        }
    });
}

/// Tokio delay provider implementation
pub struct TokioDelay;

impl super::DelayProvider for TokioDelay {
    type Instant = tokio::time::Instant;
    
    fn delay(duration: Duration) -> impl Future<Output = ()> {
        tokio::time::sleep(duration)
    }
    
    fn delay_until(instant: Self::Instant) -> impl Future<Output = ()> {
        tokio::time::sleep_until(instant)
    }
    
    fn now() -> Self::Instant {
        tokio::time::Instant::now()
    }
}
```

## Platform Support Matrix

| Platform | Executor | WCET Analysis | Real-Time | Use Case |
|----------|----------|---------------|-----------|----------|
| ARM Cortex-M | RTIC | ✅ RAUK/Symex | ✅ Hard | Safety-critical control |
| ARM Cortex-M | Embassy | ⚠️ Limited | ⚠️ Soft | General embedded |
| RISC-V | Embassy | ❌ | ⚠️ Soft | IoT devices |
| ESP32 | Embassy | ❌ | ⚠️ Soft | WiFi-enabled nodes |
| Zephyr (any) | Native | ❌ | ✅ Depends on config | Existing Zephyr projects |
| Linux | Tokio | ❌ | ❌ | Development/simulation |

## Usage Examples

### RTIC Example (Cortex-M with Analysis)

```rust
#![no_std]
#![no_main]

use panic_halt as _;
use stm32f4xx_hal as hal;
use nano_ros::{Node, Publisher};
use nano_ros::transport::ZenohSession;

#[rtic::app(device = hal::pac, dispatchers = [EXTI0, EXTI1, EXTI2])]
mod app {
    use super::*;
    use rtic_monotonics::systick::prelude::*;
    
    systick_monotonic!(Mono, 1000);
    
    // Include nano-ros background tasks
    nano_ros::rtic_tasks!(Mono, zenoh_session);

    #[shared]
    struct Shared {
        zenoh_session: ZenohSession,
    }

    #[local]
    struct Local {
        sensor_publisher: Publisher<SensorMsg>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // Initialize hardware
        let dp = cx.device;
        // ... clock setup ...
        
        Mono::start(cx.core.SYST, 84_000_000);
        
        // Initialize zenoh session
        let session = ZenohSession::new_client("tcp/192.168.1.1:7447");
        let publisher = session.create_publisher("/sensor/data");
        
        // Spawn nano-ros tasks
        nano_ros_poll::spawn().ok();
        nano_ros_keepalive::spawn().ok();
        
        // Spawn application tasks
        sensor_task::spawn().ok();
        
        (
            Shared { zenoh_session: session },
            Local { sensor_publisher: publisher },
        )
    }

    /// High-priority sensor processing task
    #[task(priority = 3, local = [sensor_publisher])]
    async fn sensor_task(cx: sensor_task::Context) {
        loop {
            let reading = read_sensor();
            cx.local.sensor_publisher.publish(&SensorMsg { value: reading });
            Mono::delay(10.millis()).await;
        }
    }

    /// Highest priority - motor control interrupt
    #[task(binds = TIM2, priority = 4)]
    fn motor_control(_cx: motor_control::Context) {
        // Time-critical motor control
        // Runs to completion, preempts all software tasks
    }
}
```

### Embassy Example (Multi-platform)

```rust
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use nano_ros::{Node, Publisher};
use nano_ros::executor::embassy::{spawn_nano_ros_tasks, SharedSession};
use static_cell::StaticCell;

static SESSION: StaticCell<SharedSession> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    
    // Initialize zenoh session
    let session = ZenohSession::new_client("tcp/192.168.1.1:7447");
    let session = SESSION.init(Mutex::new(session));
    
    // Spawn nano-ros background tasks
    spawn_nano_ros_tasks(&spawner, session).unwrap();
    
    // Spawn application tasks
    spawner.spawn(sensor_task(session)).unwrap();
}

#[embassy_executor::task]
async fn sensor_task(session: &'static SharedSession) {
    let publisher = {
        let s = session.lock().await;
        s.create_publisher("/sensor/data")
    };
    
    loop {
        let reading = read_sensor();
        publisher.publish(&SensorMsg { value: reading });
        Timer::after(Duration::from_millis(10)).await;
    }
}
```

### Linux Development Example

```rust
use nano_ros::{Node, Publisher, Subscriber};
use nano_ros::executor::tokio::spawn_nano_ros_tasks;
use std::sync::Arc;
use tokio::sync::Mutex;

#[tokio::main]
async fn main() {
    // Initialize zenoh session
    let session = ZenohSession::new_client("tcp/127.0.0.1:7447");
    let session = Arc::new(Mutex::new(session));
    
    // Spawn nano-ros background tasks
    spawn_nano_ros_tasks(session.clone()).await;
    
    // Create publisher
    let publisher = {
        let s = session.lock().await;
        s.create_publisher("/sensor/data")
    };
    
    // Application loop
    let mut interval = tokio::time::interval(Duration::from_millis(10));
    loop {
        interval.tick().await;
        let reading = simulate_sensor();
        publisher.publish(&SensorMsg { value: reading });
    }
}
```

## WCET Analysis Integration

### Using RAUK with RTIC

RAUK (Response-time Analysis Using KLEE) can analyze RTIC applications:

1. KLEE generates test vectors covering all execution paths
2. Test vectors are replayed on target hardware
3. WCET measurements derive worst-case response time
4. Schedulability analysis determines if deadlines are met

```bash
# Install RAUK
cargo install rauk

# Analyze RTIC application
rauk analyze --target thumbv7em-none-eabihf ./examples/rtic-example
```

### Using Symex/EASY

Symex provides cycle-accurate WCET estimates through symbolic execution:

```bash
# Run EASY analysis on RTIC app
easy analyze --arch armv7em ./examples/rtic-example
```

Output includes:
- Per-task WCET estimates
- Response time analysis
- Schedulability verdict
- Memory isolation verification

## Migration Guide

### From Bare Zephyr to RTIC

1. Replace Zephyr thread definitions with RTIC tasks
2. Convert mutexes to RTIC shared resources
3. Map Zephyr priorities to RTIC priorities (note: RTIC uses higher = more important)
4. Replace `k_sleep()` with `Mono::delay().await`

### From Bare Zephyr to Embassy

1. Convert threads to async tasks
2. Replace Zephyr primitives with embassy-sync equivalents
3. Use `Timer::after()` instead of `k_sleep()`
4. Configure embassy HAL for your target

## Recommendations

### For Safety-Critical Control (Motor, Sensor Fusion)

Use **RTIC** on ARM Cortex-M:
- Formal WCET analysis with RAUK/Symex
- Compile-time deadlock freedom
- Hardware-accelerated scheduling

### For General Embedded ROS Communication

Use **Embassy**:
- Broader platform support
- Easier async integration
- Good enough for soft real-time

### For Development and Testing

Use **Tokio** on Linux:
- Fast iteration
- Standard debugging tools
- CI/CD integration

### Hybrid Approach

For complex systems, combine approaches:
- RTIC for time-critical control loops
- Embassy in RTIC idle task for I/O
- Share data via RTIC channels

## Future Work

1. **FreeRTOS backend**: For existing FreeRTOS deployments
2. **Zephyr native backend**: Direct Zephyr scheduler integration
3. **Analysis tool integration**: Automated WCET extraction for nano-ros tasks
4. **QoS mapping**: Map ROS 2 QoS to executor priorities

## References

- [RTIC Book](https://rtic.rs/2/book/en/)
- [Embassy Documentation](https://embassy.dev/)
- [RAUK Thesis](https://www.uppsatser.se/uppsats/3122363010/)
- [Symex/EASY Thesis](http://www.diva-portal.org/smash/get/diva2:1998467/FULLTEXT01.pdf)
- [zenoh-pico Documentation](https://zenoh-pico.readthedocs.io/)
- [SRP Scheduling Theory](https://rtic.rs/2/book/en/)
