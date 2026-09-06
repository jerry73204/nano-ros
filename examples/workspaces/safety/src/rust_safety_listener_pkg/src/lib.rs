//! SafeListener Node pkg — E2E message-integrity (CRC) subscriber on `/chatter`.
//!
//! This pkg IS the safety-CRC listener, so `safety-e2e` is unconditional (see
//! Cargo.toml — phase-277 W3.c). It declares a SAFETY subscription via
//! `create_subscription_for_callback_name_with_safety`. When the system
//! declares `[system].features = ["safety"]`, the zenoh backend attaches a
//! CRC + sequence number on publish, the runtime validates it on receive, and the
//! callback reads the per-message `CallbackCtx::integrity()` — CRC ok, sequence
//! gap, or duplicate — alongside the payload. The first WORKSPACE example of the
//! E2E-safety differentiator (phase-263 B1; the protocol itself is RFC-0028).

#![no_std]

use nros::{Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult};
use std_msgs::msg::Int32;

/// SafeListener — counts received messages + CRC failures / sequence gaps seen.
pub struct SafeListener;

#[derive(Default)]
pub struct Counts {
    received: u32,
    integrity_faults: u32,
}

impl Node for SafeListener {
    const NAME: &'static str = "safe_listener";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("safe_listener"))?;
        let _sub = node
            .create_subscription_for_callback_name_with_safety::<Int32>("on_chatter", "/chatter")?;
        // Track D — republish the count of CRC-VALIDATED messages on `/safe_ok`, so a
        // cross-process subscriber can observe the E2E-safety path end-to-end (the
        // count climbs only while integrity is valid). The publisher is declared
        // unconditionally; the gated `on_callback` body decides what reaches it.
        let pub_ok = node.create_publisher_for_topic::<Int32>("/safe_ok")?;
        node.callback_for_name("on_chatter")
            .publishes_entity(&pub_ok)?;
        Ok(())
    }
}

impl ExecutableNode for SafeListener {
    type State = Counts;

    fn init() -> Self::State {
        Counts::default()
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_chatter" {
            if ctx.message::<Int32>().is_ok() {
                state.received = state.received.wrapping_add(1);
            }
            // The integrity status describes the message just received (the
            // `.safety()` opt-in on the subscription; `safety-e2e` is
            // unconditional on this pkg — see Cargo.toml). A non-ok CRC /
            // sequence gap / dup bumps the fault counter; a valid one
            // republishes the running count on `/safe_ok` so a subscriber can
            // assert the E2E CRC-validate path works.
            if let Some(status) = ctx.integrity() {
                if status.is_valid() {
                    let msg = Int32 {
                        data: state.received as i32,
                    };
                    let _ = ctx.publish_to_topic::<Int32, 8>("/safe_ok", &msg);
                } else {
                    state.integrity_faults = state.integrity_faults.wrapping_add(1);
                }
            }
        }
    }
}

nros::node!(SafeListener);
