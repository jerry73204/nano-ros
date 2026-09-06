//! Issue 0976 — an OUTSIDE WITNESS for the Cyclone action wire format.
//!
//! `service.cpp` carries five per-action-type adapters that reshape CDR:
//! `strip_goal_id_len_at` removes a 4-byte length prefix before a goal UUID,
//! `strip_nested_cdr_at` removes a nested encapsulation header from inside a
//! message, and three more special-case `_SendGoal_*` / `_GetResult_*`. Two of
//! them DELETE BYTES — they correct a serialization that would otherwise be
//! wrong on the wire.
//!
//! Until this file, the only thing exercising them was
//! `test_native_cyclonedds_rust_action`, which is nano-ros server to nano-ros
//! client. Both ends share whatever convention the adapters implement, so that
//! test passes whether the bytes are ROS 2's or not — the single property the
//! adapters exist to provide is the one property it cannot observe. The message
//! and service paths already had witnesses (`ros2_pubsub_e2e`, `ros2_srv_e2e`);
//! actions had none.
//!
//! This is that witness: a stock `ros2 action send_goal`, over
//! `rmw_cyclonedds_cpp`, against the nano-ros action server. It is also what
//! unblocks issue 0970's service half — migrating `service.cpp` to a blob
//! sertype would change the action wire format, and nothing in the tree could
//! tell.

use std::process::Command;

use nros_tests::{
    fixtures,
    ros_env::{HostRosEnv, Middleware, RosEnv},
    ros2::{DEFAULT_ROS_DISTRO, require_ros2_cyclonedds},
};

/// A domain no concurrent copy of this test will pick.
///
/// Same scheme as the shell harnesses and `nros_tests::unique_ros_domain_id`:
/// a fixed domain is a shared bus, and two overlapping runs discover each
/// other's writers — which reads as a delivery bug rather than a collision
/// (issue 0580).
fn action_domain() -> u8 {
    nros_tests::unique_ros_domain_id()
}

/// A real ROS 2 client drives a goal on the nano-ros action server.
///
/// Asserts the RESULT CONTENT, not merely that the call returned: the adapters
/// move bytes around inside the goal and result messages, so a wrong shape
/// shows up as a wrong sequence or a goal that is never accepted. Checking only
/// for a zero exit would pass on a server that accepted the goal and computed
/// nothing, which is the vacuous shape `check-no-vacuous-tests` exists for.
#[test]
fn a_stock_ros2_client_drives_the_nano_ros_action_server() {
    if !require_ros2_cyclonedds() {
        nros_tests::skip!("ROS 2 + rmw_cyclonedds_cpp not available");
    }

    let server_bin = fixtures::build_native_rust_example_rmw(
        "action-server",
        "action-server",
        fixtures::Rmw::Cyclonedds,
    )
    .unwrap_or_else(|e| {
        nros_tests::skip!("native rust cyclonedds action-server fixture: {e}");
    });

    let domain = action_domain();
    let mut server_cmd = Command::new(&server_bin);
    server_cmd
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .env("ROS_DOMAIN_ID", domain.to_string());
    // Issue 1137 — the same bus as the `HostRosEnv` peer below. That env string
    // funnels through `ros2_env_setup_rmw_with_domain`, which has exported a
    // loopback-only `CYCLONEDDS_URI` since issue 1009; without the matching pin
    // here the two participants sit on different interfaces and `send_goal`
    // fails to discover the action at all.
    nros_tests::dds_isolation::apply_to_command(&mut server_cmd);
    let mut server = server_cmd
        .spawn()
        .expect("start the nano-ros action server");

    // Discovery over DDS multicast is not instant, and `send_goal` on an
    // undiscovered action fails rather than waiting.
    std::thread::sleep(std::time::Duration::from_secs(6));

    // `RosEnv`, not a hand-rolled `source /opt/ros/...` (RFC-0058). A second
    // spelling drifts from the first invisibly: the last one dropped the peer's
    // ROS_DOMAIN_ID, and another hardcoded `humble` so every guarded test
    // skipped forever on a jazzy host. `check-ros-env-spelling` caught this
    // file doing exactly that.
    let env = HostRosEnv::new(
        DEFAULT_ROS_DISTRO,
        Middleware::Cyclonedds { domain_id: domain },
    );
    let out = env
        .run("timeout 60 ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci '{order: 5}'")
        .expect("run ros2 action send_goal");

    let _ = server.kill();
    let _ = server.wait();

    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );

    assert!(
        text.contains("Goal accepted"),
        "a stock ROS 2 client must get the goal ACCEPTED — the server has to \
         read a real ROS 2 SendGoal request, UUID and all.\n\
         NOTE this does NOT exercise `strip_goal_id_len_at` (issue 0976): that \
         adapter fires only when nano-ros WRITES a SendGoal/GetResult request, \
         and here `ros2` writes them. What guards the server's receive side is \
         `split_wire_header`'s deliberate non-insertion (service.cpp:589-599, \
         issue #68).\n{text}"
    );
    assert!(
        text.contains("SUCCEEDED"),
        "the goal must reach SUCCEEDED, not merely be accepted.\n{text}"
    );
    // Fibonacci(5) — the result travels the `_GetResult_Response_` path, which
    // is the one carrying the hand-built struct workaround (phase 171.0.b).
    for n in ["0", "1", "2", "3", "5"] {
        assert!(
            text.contains(&format!("- {n}")),
            "the result sequence must contain {n}; a reshaped result decodes \
             to the wrong numbers rather than failing.\n{text}"
        );
    }
}
