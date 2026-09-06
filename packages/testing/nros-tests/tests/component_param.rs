//! Phase 212.M-F.23 Wave 2 — declarative parameter dispatch on the
//! single-node component runtime.
//!
//! A component that declares a parameter (via the `nros::node!` declarative
//! path → `EntityKind::Parameter` in `ExecutorNodeRuntime::create_entity`)
//! must, at registration time, (a) lazily stand up the 6 ROS 2 parameter
//! services on the live executor and (b) seed the declared source default.
//! This is the runtime counterpart to the imperative `params` suite (which
//! exercises `executor.register_parameter_services()` / `declare_parameter()`
//! directly); here the wiring is driven entirely by the declarative metadata.
//!
//! `nros::node!` emits one per-crate export symbol, so a component lives in
//! its own test binary — this file carries the single param component, kept
//! separate from `component_dispatch` (which owns its own `node!`).

#![cfg(feature = "component-runtime-test")]

use nros_rmw_zenoh as _;

use std::time::Duration;

use nros::{
    Callback, CallbackCtx, ExecutableNode, Executor, ExecutorConfig, ExecutorNodeRuntime, Node,
    NodeContext, NodeOptions, NodeResult, ParameterDefault, TickCtx,
};
use nros_platform::RuntimeCtx;
use nros_tests::fixtures::{ZenohRouter, require_zenohd, zenohd_unique};
use rstest::rstest;

// =============================================================================
// ParamNode — declares one integer parameter with a non-zero source default.
// No callbacks; the parameter wiring is the whole point.
// =============================================================================

struct ParamNode;
impl Node for ParamNode {
    const NAME: &'static str = "mf23_param";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("mf23_param_node"))?;
        node.declare_parameter_for_name_with_default("start_value", ParameterDefault::Integer(7))?;
        Ok(())
    }
}
impl ExecutableNode for ParamNode {
    type State = ();
    fn init() -> Self::State {}
    fn on_callback(_state: &mut (), _cb: Callback<'_>, _ctx: &mut CallbackCtx<'_>) {}
    fn tick(_state: &mut Self::State, _ctx: &mut TickCtx<'_>) {}
}

nros::node!(ParamNode);
use self::register as param_register;

// =============================================================================
// Test
// =============================================================================

/// Registering a component that declares a parameter must lazily stand up the
/// ROS 2 parameter services on the executor and seed the declared default.
#[rstest]
fn dispatch_declares_and_seeds_parameter(zenohd_unique: ZenohRouter) {
    if !require_zenohd() {
        nros_tests::skip!("zenohd not found");
    }

    // The 6 parameter service servers carry inline 4 KiB req/reply buffers and
    // the `ParameterServer`'s parameter store holds the large (embedded-sized)
    // `ParameterValue` slots — together more than the Rust test harness's
    // default 2 MiB per-test-thread stack. The imperative `params` suite dodges
    // this by running the talker as a separate process (8 MiB main stack); here
    // we run the in-thread body on a thread with a generous stack instead.
    let locator = zenohd_unique.locator();
    std::thread::Builder::new()
        .stack_size(32 * 1024 * 1024)
        .spawn(move || run_param_dispatch(&locator))
        .expect("spawn param test thread")
        .join()
        .expect("param test thread panicked");
}

fn run_param_dispatch(locator: &str) {
    let cfg = ExecutorConfig::new(locator)
        .node_name("mf23_param_exec")
        .domain_id(182);
    let executor = Executor::open(&cfg).expect("Executor::open failed");
    let mut runtime = ExecutorNodeRuntime::from_executor(executor);

    // No parameter services until the declarative parameter is registered.
    assert!(
        runtime.executor().params().is_none(),
        "param services must not exist before a parameter is declared"
    );

    {
        let mut ctx = RuntimeCtx::with_runtime(&mut runtime);
        param_register(&mut ctx).expect("component register");
    }

    // The `*_register(&mut ctx)` wrapper installs through `install_node_typed`,
    // which enrolls into the EXECUTOR's component-tick registry — not the
    // runtime's `components` Vec. So the slot count lives on the executor, as
    // `component_dispatch.rs` already records for the identical path; this file
    // kept the pre-phase-258 observable and, being in no lane (issue 0652),
    // nobody ran it to find out.
    assert_eq!(runtime.executor().component_slot_count(), 1);

    // The Parameter arm must have lazily registered the param services and
    // seeded the default (7) — both observable on the live executor.
    assert!(
        runtime.executor().params().is_some(),
        "declaring a parameter must stand up the ROS 2 parameter services"
    );
    assert_eq!(
        runtime.executor().get_parameter_integer("start_value"),
        Some(7),
        "declared parameter must be seeded with its source default (7)"
    );

    // Spinning drives the param service servers; with nothing mutating the
    // value it must stay seeded.
    for _ in 0..5 {
        runtime
            .spin_once(Duration::from_millis(0))
            .expect("spin_once");
    }
    assert_eq!(
        runtime.executor().get_parameter_integer("start_value"),
        Some(7),
        "parameter value must remain seeded across spins"
    );
}
