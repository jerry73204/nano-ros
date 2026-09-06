//! Rust mixed-workspace consumer — Node pkg (agnostic application logic).
//!
//! Imports msgs from BOTH worlds via the auto-managed
//! `[patch.crates-io]` block in this pkg's Cargo.toml (written by
//! `nros sync` / `nros generate-rust`):
//!
//!   * `local_msgs::msg::Greeting`      — workspace pkg
//!   * `extra_msgs::msg::Echo`          — workspace pkg, depends on local_msgs
//!   * `geometry_msgs::msg::Point`      — AMENT
//!   * `sensor_msgs::msg::Imu`          — AMENT (transitively pulls std_msgs +
//!                                        geometry_msgs)
//!
//! Node pkg shape: `register()` declares the node + 4 publishers + a 1 Hz
//! timer; `ExecutableNode::on_callback("on_tick")` publishes one of each
//! message per tick. The native board's `BoardEntry` runtime — emitted by
//! the sibling `main.rs` `nros::main!()` — owns `Executor::open`, RMW
//! registration and the spin loop, so this source carries only the
//! msg-coverage publishing logic.

use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};

use extra_msgs::msg::Echo;
use geometry_msgs::msg::Point;
use local_msgs::msg::Greeting;
use sensor_msgs::msg::Imu;

/// Consumer — publishes one of each workspace + AMENT message per tick.
pub struct Consumer;

impl Node for Consumer {
    const NAME: &'static str = "rust_consumer";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(4, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("rust_consumer"))?;
        let greet_pub = node.create_publisher_for_topic::<Greeting>("/greetings")?;
        let echo_pub = node.create_publisher_for_topic::<Echo>("/echoes")?;
        let point_pub = node.create_publisher_for_topic::<Point>("/points")?;
        let imu_pub = node.create_publisher_for_topic::<Imu>("/imu")?;
        let _timer =
            node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(1000))?;
        node.callback_for_name("on_tick")
            .publishes_entity(&greet_pub)?
            .publishes_entity(&echo_pub)?
            .publishes_entity(&point_pub)?
            .publishes_entity(&imu_pub)?;
        Ok(())
    }
}

impl ExecutableNode for Consumer {
    /// Monotonic counter — the next sequence number to publish.
    type State = i32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_tick" {
            let seq = *state;

            let mut g = Greeting::default();
            g.sequence = seq;
            let _ = ctx.publish_to_topic::<Greeting, 8>("/greetings", &g);

            let mut e = Echo::default();
            e.original = g;
            e.hop_count = 1;
            let _ = ctx.publish_to_topic::<Echo, 8>("/echoes", &e);

            let mut p = Point::default();
            p.x = seq as f64;
            p.y = (seq * 2) as f64;
            p.z = (seq * 3) as f64;
            let _ = ctx.publish_to_topic::<Point, 8>("/points", &p);

            let mut imu = Imu::default();
            imu.linear_acceleration.x = 9.81;
            let _ = ctx.publish_to_topic::<Imu, 8>("/imu", &imu);

            *state = state.wrapping_add(1);
        }
    }
}

nros::node!(Consumer);
